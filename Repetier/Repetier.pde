/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

    Main author: repetier
    
Changelog:
    v0.2 20.8.2011  - first public version
*/
/**
\mainpage Repetier-Firmware for Arduino based RepRaps
<CENTER>Copyright &copy; 2011 by repetier
</CENTER>

\section Intro Introduction


\section GCodes Implemented GCodes

 look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
 and http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

Implemented Codes

- G0  -> G1
- G1  - Coordinated Movement X Y Z E
- G4  - Dwell S<seconds> or P<milliseconds>
- G20 - Units for G0/G1 are inches.
- G21 - Units for G0/G1 are mm.
- G28 - Home all axis or named axis.
- G90 - Use absolute coordinates
- G91 - Use relative coordinates
- G92 - Set current position to cordinates given

RepRap M Codes

- M104 - Set extruder target temp
- M105 - Read current temp
- M106 - Fan on
- M107 - Fan off
- M109 - Wait for extruder current temp to reach target temp.
- M114 - Display current position

Custom M Codes

- M80  - Turn on Power Supply
- M20  - List SD card
- M21  - Init SD card
- M22  - Release SD card
- M23  - Select SD file (M23 filename.g)
- M24  - Start/resume SD print
- M25  - Pause SD print
- M26  - Set SD position in bytes (M26 S12345)
- M27  - Report SD print status
- M28  - Start SD write (M28 filename.g)
- M29  - Stop SD write
- M30 <filename> - Delete file on sd card
- M42 P<pin number> S<value 0..255> - Change output of pin P to S. Does not work on most important pins.
- M80  - Turn on power supply
- M81  - Turn off power supply
- M82  - Set E codes absolute (default)
- M83  - Set E codes relative while in Absolute Coordinates (G90) mode
- M84  - Disable steppers until next move, 
        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
- M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
- M92  - Set axis_steps_per_unit - same syntax as G92
- M112 - Emergency kill
- M115- Capabilities string
- M117 <message> - Write message in status row on lcd
- M119 - Report endstop status
- M140 - Set bed target temp
- M190 - Wait for bed current temp to reach target temp.
- M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
- M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)
- M203 - Set temperture monitor to Sx
- M204 - Set PID parameter X => Kp Y => Ki Z => Kd
- M205 - Output EEPROM settings
- M206 - Set EEPROM value
- M220 S<Feedrate multiplier in percent> - Increase/decrease given feedrate
- M221 S<Extrusion flow multiplier in percent> - Increase/decrease given flow rate
- M231 S<OPS_MODE> X<Min_Distance> Y<Retract> Z<Backslash> F<ReatrctMove> - Set OPS parameter
- M232 - Read and reset max. advance values
- M233 X<AdvanceK> - Set temporary advance K-value to X
*/

#include "Configuration.h"
#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"
#include "fastio.h"
#include "ui.h"
#include <util/delay.h>

#ifdef SDSUPPORT
#include "SdFat.h"
#endif

#if UI_DISPLAY_TYPE==4
//#include <LiquidCrystal.h> // Uncomment this if you are using liquid crystal library
#endif

// ================ Sanity checks ================
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY<10000 || STEP_DOUBLER_FREQUENCY>20000
#error STEP_DOUBLER_FREQUENCY should be in range 10000-16000.
#endif
#endif
#ifdef EXTRUDER_SPEED
#error EXTRUDER_SPEED is not used any more. Values are now taken from extruder definition.
#endif
#if MAX_HALFSTEP_INTERVAL<=1900
#error MAX_HALFSTEP_INTERVAL must be greater then 1900
#endif
#ifdef ENDSTOPPULLUPS
#error ENDSTOPPULLUPS is now replaced by individual pullup configuration!
#endif
// ####################################################################################
// #          No configuration below this line - just some errorchecking              #
// ####################################################################################
#ifdef SUPPORT_MAX6675
#if !defined SCK_PIN || !defined MOSI_PIN || !defined MISO_PIN
#error For MAX6675 support, you need to define SCK_PIN, MISO_PIN and MOSI_PIN in pins.h
#endif
#endif
#if X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if EXT0_STEP_PIN<0
#error EXT0_STEP_PIN not set to a pin number.
#endif
#if EXT0_DIR_PIN<0
#error EXT0_DIR_PIN not set to a pin number.
#endif
#if MOVE_CACHE_SIZE<4
#error MOVE_CACHE_SIZE must be at least 5
#endif
#if OUTPUT_BUFFER_SIZE>250 || OUTPUT_BUFFER_SIZE<16
#error OUTPUT_BUFFER_SIZE must be in range 16..250
#endif


#define OVERFLOW_PERIODICAL  (int)(F_CPU/(TIMER0_PRESCALE*40))
// RAM usage of variables: Non RAMPS 114+MOVE_CACHE_SIZE*59+printer_state(32) = 382 Byte with MOVE_CACHE_SIZE=4
// RAM usage RAMPS adds: 96
// RAM usage SD Card:
byte unit_inches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float axis_steps_per_unit[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float inv_axis_steps_per_unit[4]; ///< Inverse of axis_steps_per_unit for faster conversion 
float max_feedrate[4] = MAX_FEEDRATE; ///< Maximum allowed feedrate.
float homing_feedrate[3] = HOMING_FEEDRATE;
byte STEP_PIN[3] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN};
#ifdef RAMP_ACCELERATION
//  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
  long max_acceleration_units_per_sq_second[4] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
  long max_travel_acceleration_units_per_sq_second[4] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
  /** Acceleration in steps/s^3 in printing mode.*/
  unsigned long axis_steps_per_sqr_second[4];
  /** Acceleration in steps/s^2 in movement mode.*/
  unsigned long axis_travel_steps_per_sqr_second[4];
#endif
PrinterState printer_state;
byte relative_mode = false;  ///< Determines absolute (false) or relative Coordinates (true).
byte relative_mode_e = false;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
byte debug_level = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = MAX_INACTIVE_TIME*1000L;
unsigned long stepper_inactive_time = STEPPER_INACTIVE_TIME*1000L;
PrintLine lines[MOVE_CACHE_SIZE]; ///< Cache for print moves.
PrintLine *cur = 0;               ///< Current printing line                
byte lines_write_pos=0;           ///< Position where we write the next cached line move.
byte lines_pos=0;                 ///< Position for executing line movement.
volatile byte lines_count=0;      ///< Number of lines cached 0 = nothing to do.
long baudrate = BAUDRATE;         ///< Communication speed rate.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
int maxadv=0;
#endif
int maxadv2=0;
float maxadvspeed=0;
#endif
byte pwm_pos[4] = {0,0,0,0}; // 0-2 = Heater 0-2 of extruder, 3 = Fan

#ifdef SIMULATE_FAN_PWM
int fan_speed=0;
int fan_pwm_pos=0;
#endif
int waitRelax=0; // Delay filament relax at the end of print, could be a simple timeout
#ifdef USE_OPS
byte printmoveSeen=0;
#endif
#ifdef DEBUG_FREE_MEMORY
int lowest_ram=16384;
int lowest_send=16384;

void check_mem() {
BEGIN_INTERRUPT_PROTECTED
  uint8_t * heapptr, * stackptr;
  heapptr = (uint8_t *)malloc(4);          // get heap pointer
  free(heapptr);      // free up the memory again (sets heapptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
  int newfree = (int)stackptr-(int)heapptr;
  if(newfree<lowest_ram) {
     lowest_ram = newfree;
  }
END_INTERRUPT_PROTECTED
}
void send_mem() {
  if(lowest_send>lowest_ram) {
    lowest_send = lowest_ram;
    out.println_int_P(PSTR("Free RAM:"),lowest_ram); 
  }
}
#endif

#ifdef SDSUPPORT
  Sd2Card card; // ~14 Byte
  SdVolume volume;
  SdFile root;
  SdFile file;
  uint32_t filesize = 0;
  uint32_t sdpos = 0;
  bool sdmode = false;
  bool sdactive = false;
  bool savetosd = false;
  int16_t n;
  
  void initsd(){
  sdactive = false;
  #if SDSS >- 1
    if(root.isOpen())
        root.close();
    if (!card.init(SPI_FULL_SPEED,SDSS)){
        //if (!card.init(SPI_HALF_SPEED,SDSS))
          out.println_P(PSTR("SD init fail"));
    }
    else if (!volume.init(&card))
          out.println_P(PSTR("volume.init failed"));
    else if (!root.openRoot(&volume)) 
          out.println_P(PSTR("openRoot failed"));
    else 
            sdactive = true;
  #endif
  }
  
  inline void write_command(GCode *code){
     unsigned int sum1=0,sum2=0; // for fletcher-16 checksum
      byte buf[52];
      byte p=2;
      file.writeError = false;
      int params = 128 | (code->params & ~1);
      *(int*)buf = params;      
      if(code->params & 2) {buf[p++] = code->M;}
      if(code->params & 4) {buf[p++] = code->G;}
      if(code->params & 8) {*(float*)&buf[p] = code->X;p+=4;}
      if(code->params & 16) {*(float*)&buf[p] = code->Y;p+=4;}
      if(code->params & 32) {*(float*)&buf[p] = code->Z;p+=4;}
      if(code->params & 64) {*(float*)&buf[p] = code->E;p+=4;}
      if(code->params & 256) {*(float*)&buf[p] = code->F;p+=4;}
      if(code->params & 512) {buf[p++] = code->T;}
      if(code->params & 1024) {*(long int*)&buf[p] = code->S;p+=4;}
      if(code->params & 2048) {*(long int*)&buf[p] = code->P;p+=4;}
      if(GCODE_HAS_STRING(code)) { // read 16 byte into string
       char *sp = code->text;
       for(byte i=0;i<16;++i) buf[p++] = *sp++;
      }
      byte *ptr = buf;
      byte len = p;
      while (len) {
        byte tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
          sum1 += *ptr++;
          if(sum1>=255) sum1-=255;
          sum2 += sum1;
          if(sum2>=255) sum2-=255;
        } while (--tlen);
      }
      buf[p++] = sum1;
      buf[p++] = sum2;
      file.write(buf,p);
      if (file.writeError){
          out.println_P(PSTR("error writing to file"));
      }
  }
#endif

void update_ramps_parameter() {
  printer_state.xMaxSteps = axis_steps_per_unit[0]*X_MAX_LENGTH;
  printer_state.yMaxSteps = axis_steps_per_unit[1]*Y_MAX_LENGTH;
  printer_state.zMaxSteps = axis_steps_per_unit[2]*Z_MAX_LENGTH;
  for(byte i=0;i<4;i++) {
    inv_axis_steps_per_unit[i] = 1.0f/axis_steps_per_unit[i];
#ifdef RAMP_ACCELERATION
    /** Acceleration in steps/s^3 in printing mode.*/
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
    /** Acceleration in steps/s^2 in movement mode.*/
    axis_travel_steps_per_sqr_second[i] = max_travel_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
#endif
  }
}
/** \brief Setup of the hardware

Sets the output and input pins in accordance to your configuration. Initializes the serial interface.
Interrupt routines to measure analog values and for the stepper timerloop are started.
*/
void setup()
{ 
#ifdef ENABLE_POWER_ON_STARTUP
  if(PS_ON_PIN > -1) {
     pinMode(PS_ON_PIN,OUTPUT); //GND
     digitalWrite(PS_ON_PIN, LOW);
  }

#endif
  //Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  SET_OUTPUT(Y_STEP_PIN);
  SET_OUTPUT(Z_STEP_PIN);
  
  //Initialize Dir Pins
#if X_DIR_PIN>-1
  SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN>-1
  SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN>-1
  SET_OUTPUT(Z_DIR_PIN);
#endif

  //Steppers default to disabled.
#if X_ENABLE_PIN > -1 
  if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  SET_OUTPUT(X_ENABLE_PIN);
#endif
#if Y_ENABLE_PIN > -1 
  if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  SET_OUTPUT(Y_ENABLE_PIN);
#endif
#if Z_ENABLE_PIN > -1 
  if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  SET_OUTPUT(Z_ENABLE_PIN);
#endif

  //endstop pullups
#if X_MIN_PIN>-1
  SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
  WRITE(X_MIN_PIN,HIGH);
#endif
#endif
#if Y_MIN_PIN>-1
  SET_INPUT(Y_MIN_PIN); 
#if ENDSTOP_PULLUP_Y_MIN
  WRITE(Y_MIN_PIN,HIGH);
#endif
#endif
#if Z_MIN_PIN>-1
  SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
  WRITE(Z_MIN_PIN,HIGH);
#endif
#endif
#if X_MAX_PIN>-1
  SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
  WRITE(X_MAX_PIN,HIGH);
#endif
#endif
#if Y_MAX_PIN>-1
  SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
  WRITE(Y_MAX_PIN,HIGH);
#endif
#endif
#if Z_MAX_PIN>-1
  SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
  WRITE(Z_MAX_PIN,HIGH);
#endif
#endif
#if FAN_PIN>-1
  SET_OUTPUT(FAN_PIN);
  WRITE(FAN_PIN,LOW);
#endif
#if EXT0_HEATER_PIN>-1
  SET_OUTPUT(EXT0_HEATER_PIN);
  WRITE(EXT0_HEATER_PIN,LOW);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
  SET_OUTPUT(EXT1_HEATER_PIN);
  WRITE(EXT1_HEATER_PIN,LOW);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
  SET_OUTPUT(EXT2_HEATER_PIN);
  WRITE(EXT2_HEATER_PIN,LOW);
#endif
#if USE_OPS==1
  printer_state.opsMode = OPS_MODE;
  printer_state.opsMinDistance = OPS_MIN_DISTANCE;
  printer_state.opsRetractDistance = OPS_RETRACT_DISTANCE;
  printer_state.opsRetractBackslash = OPS_RETRACT_BACKSLASH;
  printer_state.filamentRetracted = false;
#endif
  printer_state.feedrate = 3000; ///< Current feedrate in mm/min.
  printer_state.feedrateMultiply = 100;
  printer_state.extrudeMultiply = 100; // Is 10.24 * value here 100 percent)
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  printer_state.advance_executed = 0;
#endif
  printer_state.advance_steps_set = 0;
  printer_state.advance_lin_set = 0;
#endif
  printer_state.currentPositionSteps[0] = printer_state.currentPositionSteps[1] = printer_state.currentPositionSteps[2] = printer_state.currentPositionSteps[3] = 0;
  printer_state.maxJerk = MAX_JERK;
  printer_state.maxZJerk = MAX_ZJERK;
  printer_state.interval = 5000;
  printer_state.stepper_loops = 1;
  printer_state.flag0 = 1;
  epr_init_baudrate();
  Serial.begin(baudrate);
  out.println_P(PSTR("start"));
  UI_INITIALIZE;
  
  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) out.println_P(PSTR("PowerUp"));
  if(mcu & 2) out.println_P(PSTR("External Reset"));
  if(mcu & 4) out.println_P(PSTR("Brown out Reset"));
  if(mcu & 8) out.println_P(PSTR("Watchdog Reset"));
  if(mcu & 32) out.println_P(PSTR("Software Reset"));
  MCUSR=0;
  
  initExtruder();
  epr_init(); // Read settings from eeprom if wanted
  update_ramps_parameter();

#ifdef SDSUPPORT

  //power to SD reader
  #if SDPOWER > -1
    pinMode(SDPOWER,OUTPUT); 
    digitalWrite(SDPOWER,HIGH);
  #endif
  initsd();

#endif
#if USE_OPS==1 || defined(USE_ADVANCE)
  EXTRUDER_TCCR = 0; // need Normal not fastPWM set by arduino init
  EXTRUDER_TIMSK |= (1<<EXTRUDER_OCIE); // Activate compa interrupt on timer 0
#endif
  PWM_TCCR = 0;  // Setup PWM interrupt
  PWM_OCR = 64;
  PWM_TIMSK |= (1<<PWM_OCIE);  

  TCCR1A = 0;  // Steup timer 1 interrupt to no prescale CTC mode
  TCCR1C = 0;
  TIMSK1 = 0;  
  TCCR1B =  (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
  OCR1A=65500; //start off with a slow frequency.
  TIMSK1 |= (1<<OCIE1A); // Enable interrupt
}

/**
  Main processing loop. It checks perodically for new commands, checks temperatures
  and executes new incoming commands.
*/
void loop()
{
  gcode_read_serial();
  GCode *code = gcode_next_command();
  //UI_SLOW; // do longer timed user interface action
  UI_MEDIUM; // do check encoder
  if(code){
#ifdef SDSUPPORT
    if(savetosd){
        if(!(GCODE_HAS_M(code) && code->M==29)) { // still writing to file
            write_command(code);
        } else {
            file.sync();
            file.close();
            savetosd = false;
            out.println_P(PSTR("Done saving file."));
            UI_CLEAR_STATUS;
        }
#ifdef ECHO_ON_EXECUTE
        if(DEBUG_ECHO) {
           out.print_P(PSTR("Echo:"));
           gcode_print_command(code);
           out.println();
        }
#endif
        gcode_command_finished(); 
    } else {
        process_command(code);
    }
#else
    process_command(code);
#endif
  }
  //check heater every n milliseconds
  check_periodical();
  UI_MEDIUM; // do check encoder
  unsigned long curtime = millis();
  if(lines_count)
    previous_millis_cmd = curtime;
  if(max_inactive_time!=0 && (curtime-previous_millis_cmd) >  max_inactive_time ) kill(false); 
  if(stepper_inactive_time!=0 && (curtime-previous_millis_cmd) >  stepper_inactive_time ) { kill(true); }
  //void finishNextSegment();
#ifdef DEBUG_FREE_MEMORY
  send_mem();
#endif
}
void log_long_array(PGM_P ptr,long *arr) {
  out.print_P(ptr);
  for(byte i=0;i<4;i++) {
    out.print(' ');
    out.print(arr[i]);
  }
  out.println();
}
void log_float_array(PGM_P ptr,float *arr) {
  out.print_P(ptr);
  for(byte i=0;i<3;i++) 
    out.print_float_P(PSTR(" "),arr[i]);
  out.println_float_P(PSTR(" "),arr[3]);
}
void log_printLine(PrintLine *p) {
  out.println_int_P(PSTR("ID:"),(int)p);
  log_long_array(PSTR("Delta"),p->delta);
  //log_long_array(PSTR("Error"),p->error);
  //out.println_int_P(PSTR("Prim:"),p->primaryAxis);
  out.println_int_P(PSTR("Dir:"),p->dir);
  out.println_int_P(PSTR("Flags:"),p->flags);
  out.println_float_P(PSTR("fullSpeed:"),p->fullSpeed);
  out.println_long_P(PSTR("vMax:"),p->vMax);
  out.println_float_P(PSTR("Acceleration:"),p->acceleration);
  out.println_long_P(PSTR("Acceleration Prim:"),p->accelerationPrim);
  //out.println_long_P(PSTR("Acceleration Timer:"),p->facceleration);
  out.println_long_P(PSTR("Remaining steps:"),p->stepsRemaining);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  out.println_long_P(PSTR("advanceFull:"),p->advanceFull>>16);
  out.println_long_P(PSTR("advanceRate:"),p->advanceRate);
#endif
#endif
}
/** \brief Disable stepper motor for x direction. */
inline void disable_x() { 
#if (X_ENABLE_PIN > -1) 
  WRITE(X_ENABLE_PIN,!X_ENABLE_ON); 
#endif
}
/** \brief Disable stepper motor for y direction. */
inline void disable_y() { 
#if (Y_ENABLE_PIN > -1) 
  WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); 
#endif
}
/** \brief Disable stepper motor for z direction. */
inline void disable_z() {
#if (Z_ENABLE_PIN > -1)
 WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for x direction. */
inline void  enable_x() { 
#if (X_ENABLE_PIN > -1)
 WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for y direction. */
inline void  enable_y() {
#if (Y_ENABLE_PIN > -1) 
  WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for z direction. */
inline void  enable_z() {
#if (Z_ENABLE_PIN > -1)
 WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
}
/** \brief Optimized division 

Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
This version is optimized for a 16 bit dividend and recognises the special cases
of a 24 bit and 16 bit dividend, which offen, but not always occur in updating the
interval.
*/
inline long Div4U2U(unsigned long a,unsigned int b) {
  // r14/r15 remainder
  // r16 counter
  __asm__ __volatile__ (
  "clr r14 \n\t"
  "sub r15,r15 \n\t"
  "tst %D0 \n\t"
  "brne do32%= \n\t"
  "tst %C0 \n\t"
  "breq donot24%= \n\t"
  "rjmp do24%= \n\t"
  "donot24%=:" "ldi r16,17 \n\t" // 16 Bit divide
  "d16u_1%=:" "rol %A0 \n\t"
  "rol %B0 \n\t"
  "dec r16 \n\t"
  "brne	d16u_2%= \n\t"
  "rjmp end%= \n\t"
  "d16u_2%=:" "rol r14 \n\t"
  "rol r15 \n\t"
  "sub r14,%A2 \n\t"
  "sbc r15,%B2 \n\t"
  "brcc	d16u_3%= \n\t"
  "add r14,%A2 \n\t"
  "adc r15,%B2 \n\t"
  "clc \n\t"
  "rjmp d16u_1%= \n\t"
  "d16u_3%=:" "sec \n\t"
  "rjmp d16u_1%= \n\t"
  "do32%=:" // divide full 32 bit
  "rjmp do32B%= \n\t"
  "do24%=:" // divide 24 bit

  "ldi r16,25 \n\t" // 24 Bit divide
  "d24u_1%=:" "rol %A0 \n\t"
  "rol %B0 \n\t"
  "rol %C0 \n\t"
  "dec r16 \n\t"
  "brne	d24u_2%= \n\t"
  "rjmp end%= \n\t"
  "d24u_2%=:" "rol r14 \n\t"
  "rol r15 \n\t"
  "sub r14,%A2 \n\t"
  "sbc r15,%B2 \n\t"
  "brcc	d24u_3%= \n\t"
  "add r14,%A2 \n\t"
  "adc r15,%B2 \n\t"
  "clc \n\t"
  "rjmp d24u_1%= \n\t"
  "d24u_3%=:" "sec \n\t"
  "rjmp d24u_1%= \n\t"

  "do32B%=:" // divide full 32 bit

  "ldi r16,33 \n\t" // 32 Bit divide
  "d32u_1%=:" "rol %A0 \n\t"
  "rol %B0 \n\t"
  "rol %C0 \n\t"
  "rol %D0 \n\t"
  "dec r16 \n\t"
  "brne	d32u_2%= \n\t"
  "rjmp end%= \n\t"
  "d32u_2%=:" "rol r14 \n\t"
  "rol r15 \n\t"
  "sub r14,%A2 \n\t"
  "sbc r15,%B2 \n\t"
  "brcc	d32u_3%= \n\t"
  "add r14,%A2 \n\t"
  "adc r15,%B2 \n\t"
  "clc \n\t"
  "rjmp d32u_1%= \n\t"
  "d32u_3%=:" "sec \n\t"
  "rjmp d32u_1%= \n\t"

  "end%=:" // end
  :"=&r"(a)
  :"0"(a),"r"(b)
  :"r14","r15","r16"
  );
 return a; 
}

const uint16_t fast_div_lut[17] PROGMEM = {0,F_CPU/4096,F_CPU/8192,F_CPU/12288,F_CPU/16384,F_CPU/20480,F_CPU/24576,F_CPU/28672,F_CPU/32768,F_CPU/36864
  ,F_CPU/40960,F_CPU/45056,F_CPU/49152,F_CPU/53248,F_CPU/57344,F_CPU/61440,F_CPU/65536};

const uint16_t slow_div_lut[257] PROGMEM = {0,F_CPU/32,F_CPU/64,F_CPU/96,F_CPU/128,F_CPU/160,F_CPU/192,F_CPU/224,F_CPU/256,F_CPU/288,F_CPU/320,F_CPU/352
 ,F_CPU/384,F_CPU/416,F_CPU/448,F_CPU/480,F_CPU/512,F_CPU/544,F_CPU/576,F_CPU/608,F_CPU/640,F_CPU/672,F_CPU/704,F_CPU/736,F_CPU/768,F_CPU/800,F_CPU/832
 ,F_CPU/864,F_CPU/896,F_CPU/928,F_CPU/960,F_CPU/992,F_CPU/1024,F_CPU/1056,F_CPU/1088,F_CPU/1120,F_CPU/1152,F_CPU/1184,F_CPU/1216,F_CPU/1248,F_CPU/1280,F_CPU/1312
 ,F_CPU/1344,F_CPU/1376,F_CPU/1408,F_CPU/1440,F_CPU/1472,F_CPU/1504,F_CPU/1536,F_CPU/1568,F_CPU/1600,F_CPU/1632,F_CPU/1664,F_CPU/1696,F_CPU/1728,F_CPU/1760,F_CPU/1792
 ,F_CPU/1824,F_CPU/1856,F_CPU/1888,F_CPU/1920,F_CPU/1952,F_CPU/1984,F_CPU/2016
 ,F_CPU/2048,F_CPU/2080,F_CPU/2112,F_CPU/2144,F_CPU/2176,F_CPU/2208,F_CPU/2240,F_CPU/2272,F_CPU/2304,F_CPU/2336,F_CPU/2368,F_CPU/2400
 ,F_CPU/2432,F_CPU/2464,F_CPU/2496,F_CPU/2528,F_CPU/2560,F_CPU/2592,F_CPU/2624,F_CPU/2656,F_CPU/2688,F_CPU/2720,F_CPU/2752,F_CPU/2784,F_CPU/2816,F_CPU/2848,F_CPU/2880
 ,F_CPU/2912,F_CPU/2944,F_CPU/2976,F_CPU/3008,F_CPU/3040,F_CPU/3072,F_CPU/3104,F_CPU/3136,F_CPU/3168,F_CPU/3200,F_CPU/3232,F_CPU/3264,F_CPU/3296,F_CPU/3328,F_CPU/3360
 ,F_CPU/3392,F_CPU/3424,F_CPU/3456,F_CPU/3488,F_CPU/3520,F_CPU/3552,F_CPU/3584,F_CPU/3616,F_CPU/3648,F_CPU/3680,F_CPU/3712,F_CPU/3744,F_CPU/3776,F_CPU/3808,F_CPU/3840
 ,F_CPU/3872,F_CPU/3904,F_CPU/3936,F_CPU/3968,F_CPU/4000,F_CPU/4032,F_CPU/4064
 ,F_CPU/4096,F_CPU/4128,F_CPU/4160,F_CPU/4192,F_CPU/4224,F_CPU/4256,F_CPU/4288,F_CPU/4320,F_CPU/4352,F_CPU/4384,F_CPU/4416,F_CPU/4448,F_CPU/4480,F_CPU/4512,F_CPU/4544
 ,F_CPU/4576,F_CPU/4608,F_CPU/4640,F_CPU/4672,F_CPU/4704,F_CPU/4736,F_CPU/4768,F_CPU/4800,F_CPU/4832,F_CPU/4864,F_CPU/4896,F_CPU/4928,F_CPU/4960,F_CPU/4992,F_CPU/5024
 ,F_CPU/5056,F_CPU/5088,F_CPU/5120,F_CPU/5152,F_CPU/5184,F_CPU/5216,F_CPU/5248,F_CPU/5280,F_CPU/5312,F_CPU/5344,F_CPU/5376,F_CPU/5408,F_CPU/5440,F_CPU/5472,F_CPU/5504
 ,F_CPU/5536,F_CPU/5568,F_CPU/5600,F_CPU/5632,F_CPU/5664,F_CPU/5696,F_CPU/5728,F_CPU/5760,F_CPU/5792,F_CPU/5824,F_CPU/5856,F_CPU/5888,F_CPU/5920,F_CPU/5952,F_CPU/5984
 ,F_CPU/6016,F_CPU/6048,F_CPU/6080,F_CPU/6112,F_CPU/6144,F_CPU/6176,F_CPU/6208,F_CPU/6240,F_CPU/6272,F_CPU/6304,F_CPU/6336,F_CPU/6368,F_CPU/6400,F_CPU/6432,F_CPU/6464
 ,F_CPU/6496,F_CPU/6528,F_CPU/6560,F_CPU/6592,F_CPU/6624,F_CPU/6656,F_CPU/6688,F_CPU/6720,F_CPU/6752,F_CPU/6784,F_CPU/6816,F_CPU/6848,F_CPU/6880,F_CPU/6912,F_CPU/6944
 ,F_CPU/6976,F_CPU/7008,F_CPU/7040,F_CPU/7072,F_CPU/7104,F_CPU/7136,F_CPU/7168,F_CPU/7200,F_CPU/7232,F_CPU/7264,F_CPU/7296,F_CPU/7328,F_CPU/7360,F_CPU/7392,F_CPU/7424
 ,F_CPU/7456,F_CPU/7488,F_CPU/7520,F_CPU/7552,F_CPU/7584,F_CPU/7616,F_CPU/7648,F_CPU/7680,F_CPU/7712,F_CPU/7744,F_CPU/7776,F_CPU/7808,F_CPU/7840,F_CPU/7872,F_CPU/7904
 ,F_CPU/7936,F_CPU/7968,F_CPU/8000,F_CPU/8032,F_CPU/8064,F_CPU/8096,F_CPU/8128,F_CPU/8160,F_CPU/8192
 };
/** \brief approximates division of F_CPU/divisor 

In the stepper interrupt a division is needed, which is a slow operation. 
The result is used for timer calculation where small errors are ok. This
function uses lookup tables to find a fast approximation of the result.

*/
long CPUDivU2(unsigned int divisor) {
  long res;
  unsigned short table;
  if(divisor<8192) {  
    if(divisor<512) {
      if(divisor<10) divisor = 10;
      return Div4U2U(F_CPU,divisor); // These entries have overflows in lookuptable!
    }
    table = (unsigned short)&slow_div_lut[0];
    __asm__ __volatile__( // needs 64 ticks neu 49 Ticks
    "mov r18,%A1 \n\t"
    "andi r18,31 \n\t"  // divisor & 31 in r18
    "lsr %B1 \n\t" // divisor >> 4
    "ror %A1 \n\t"
    "lsr %B1 \n\t"
    "ror %A1 \n\t"
    "lsr %B1 \n\t"
    "ror %A1 \n\t"
    "lsr %B1 \n\t"
    "ror %A1 \n\t"
    "andi %A1,254 \n\t"
    "add %A2,%A1 \n\t" // table+divisor>>3
    "adc %B2,%B1 \n\t"
    "lpm %A0,Z+ \n\t" // y0 in res
    "lpm %B0,Z+ \n\t"  // %C0,%D0 are 0
    "movw r4,%A0 \n\t" // y0 nach gain (r4-r5)
    "lpm r0,Z+ \n\t" // gain = gain-y1
    "sub r4,r0 \n\t"
    "lpm r0,Z+ \n\t"
    "sbc r5,r0 \n\t"
    "mul r18,r4 \n\t" // gain*(divisor & 31)
    "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
    "mul r18,r5 \n\t"
    "add %B1,r0 \n\t"
    "mov %A2,r1 \n\t"
    "lsl %A1 \n\t"
    "rol %B1 \n\t"
    "rol %A2 \n\t"
    "lsl %A1 \n\t"
    "rol %B1 \n\t"
    "rol %A2 \n\t"
    "lsl %A1 \n\t"
    "rol %B1 \n\t"
    "rol %A2 \n\t"
    "sub %A0,%B1 \n\t"
    "sbc %B0,%A2 \n\t"
    "clr %C0 \n\t"
    "clr %D0 \n\t"
    "clr r1 \n\t"
    : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r4","r5");
    return res;
    /*unsigned short adr0 = (unsigned short)&slow_div_lut+(divisor>>4)&1022;
    long y0=	pgm_read_dword_near(adr0);
    long gain = y0-pgm_read_dword_near(adr0+2);
    return y0-((gain*(divisor & 31))>>5);*/
  } else {
    table = (unsigned short)&fast_div_lut[0];
    __asm__ __volatile__( // needs 49 ticks
    "movw r18,%A1 \n\t"
    "andi r19,15 \n\t"  // divisor & 4095 in r18,r19
    "lsr %B1 \n\t" // divisor >> 3, then %B1 is 2*(divisor >> 12)
    "lsr %B1 \n\t"
    "lsr %B1 \n\t"
    "andi %B1,254 \n\t"
    "add %A2,%B1 \n\t" // table+divisor>>11
    "adc %B2,r1 \n\t" //
    "lpm %A0,Z+ \n\t" // y0 in res
    "lpm %B0,Z+ \n\t"
    "movw r4,%A0 \n\t" // y0 to gain (r4-r5)
    "lpm r0,Z+ \n\t" // gain = gain-y1
    "sub r4,r0 \n\t"
    "lpm r0,Z+ \n\t"
    "sbc r5,r0 \n\t" // finished - result has max. 16 bit
    "mul r18,r4 \n\t" // gain*(divisor & 4095)
    "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
    "mul r19,r5 \n\t"
    "mov %A2,r0 \n\t" // %A2 = byte 3 of result
    "mul r18,r5 \n\t"
    "add %B1,r0 \n\t"
    "adc %A2,r1 \n\t"
    "mul r19,r4 \n\t"
    "add %B1,r0 \n\t"
    "adc %A2,r1 \n\t"
    "andi %B1,240 \n\t" // >> 12
    "swap %B1 \n\t"
    "swap %A2 \r\n"
    "mov %A1,%A2 \r\n"
    "andi %A1,240 \r\n"
    "or %B1,%A1 \r\n"
    "andi %A2,15 \r\n"
    "sub %A0,%B1 \n\t"
    "sbc %B0,%A2 \n\t"
    "clr %C0 \n\t"
    "clr %D0 \n\t"
    "clr r1 \n\t"
    : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r19","r4","r5");
    return res;
    /*
    // The asm mimics the following code
    unsigned short adr0 = (unsigned short)&fast_div_lut+(divisor>>11)&254;
    unsigned short y0=	pgm_read_word_near(adr0);
    unsigned short gain = y0-pgm_read_word_near(adr0+2);
    return y0-(((long)gain*(divisor & 4095))>>12);*/
  }
}
#ifdef Z_BACKSLASH
char lastzdir=0;
#endif

/**
  \brief Sets the destination coordinates to values stored in com.
  
  For the computation of the destination, the following facts are considered:
  - Are units inches or mm.
  - Reltive or absolute positioning with special case only extruder relative.
  - Offset in x and y direction for multiple extruder support.
*/
byte get_coordinates(GCode *com)
{
  register long p;
  register byte r=0;
  if(lines_count==0) {
    UI_STATUS(UI_TEXT_PRINTING);
  }
  if(GCODE_HAS_X(com)) {
    r = 1;
    if(unit_inches)
      p = com->X*25.4*axis_steps_per_unit[0];
    else
      p = com->X*axis_steps_per_unit[0];
    if(relative_mode)
        printer_state.destinationSteps[0] = printer_state.currentPositionSteps[0]+p;
    else
        printer_state.destinationSteps[0] = p+printer_state.offsetX;
  } else printer_state.destinationSteps[0] = printer_state.currentPositionSteps[0];
  if(GCODE_HAS_Y(com)) {
    r = 1;
    if(unit_inches)
      p = com->Y*25.4*axis_steps_per_unit[1];
    else
      p = com->Y*axis_steps_per_unit[1];
    if(relative_mode)
        printer_state.destinationSteps[1] = printer_state.currentPositionSteps[1]+p;
    else
        printer_state.destinationSteps[1] = p+printer_state.offsetY;
  } else printer_state.destinationSteps[1] = printer_state.currentPositionSteps[1];
  if(GCODE_HAS_Z(com)) {
    r = 1;
    if(unit_inches)
      p = com->Z*25.4*axis_steps_per_unit[2];
    else
      p = com->Z*axis_steps_per_unit[2];
    if(relative_mode) {
        printer_state.destinationSteps[2] = printer_state.currentPositionSteps[2]+p;
#ifdef Z_BACKSLASH
        if(p>0 && lastzdir!=1) {
          lastzdir = 1;
          printer_state.currentPositionSteps[2]-=Z_BACKSLASH*axis_steps_per_unit[2];
        } else if(p<0 && lastzdir!=-1) {
          lastzdir=-1;
          printer_state.currentPositionSteps[2]+=Z_BACKSLASH*axis_steps_per_unit[2];
        }
#endif
    } else {
#ifdef Z_BACKSLASH
        if(p>printer_state.destinationSteps[2] && lastzdir!=1) {
          lastzdir = 1;
          printer_state.currentPositionSteps[2]-=Z_BACKSLASH*axis_steps_per_unit[2];
        } else if(p<printer_state.destinationSteps[2] && lastzdir!=-1) {
          lastzdir=-1;
          printer_state.currentPositionSteps[2]+=Z_BACKSLASH*axis_steps_per_unit[2];
        }
#endif
        printer_state.destinationSteps[2] = p;
    }
  } else printer_state.destinationSteps[2] = printer_state.currentPositionSteps[2];
  if(GCODE_HAS_E(com) && !DEBUG_DRYRUN) {
    if(unit_inches)
      p = com->E*25.4*axis_steps_per_unit[3];
    else
      p = com->E*axis_steps_per_unit[3];
    if(relative_mode || relative_mode_e)
        printer_state.destinationSteps[3] = printer_state.currentPositionSteps[3]+p;
    else
        printer_state.destinationSteps[3] = p;
  } else printer_state.destinationSteps[3] = printer_state.currentPositionSteps[3];
  if(GCODE_HAS_F(com)) {
    if(com->F < 10)
      printer_state.feedrate = 10;
    else
      if(unit_inches)
        printer_state.feedrate = com->F*0.254f*(float)printer_state.feedrateMultiply;
      else
        printer_state.feedrate = com->F*(float)printer_state.feedrateMultiply*0.01;
  }
  return r || (GCODE_HAS_E(com) && printer_state.destinationSteps[3]!=printer_state.currentPositionSteps[3]); // ignore unprductive moves
}
inline float computeJerk(PrintLine *p1,PrintLine *p2) {
   float dx = p2->speedX*p2->startFactor-p1->speedX*p1->endFactor;
   float dy = p2->speedY*p2->startFactor-p1->speedY*p1->endFactor;
   if((p1->dir & 128)==0 && (p2->dir & 128)==0)
     return sqrt(dx*dx+dy*dy);
   float dz = (p2->speedZ-p1->speedZ)*printer_state.maxJerk/printer_state.maxZJerk;
   return sqrt(dx*dx+dy*dy+dz*dz);
}
inline float safeSpeed(PrintLine *p) {
  float safe = printer_state.maxJerk*0.5;
  if(p->dir & 64) {
    if(abs(p->speedZ)>printer_state.maxZJerk*0.5) {
      float safe2 = printer_state.maxZJerk*0.5*p->fullSpeed/abs(p->speedZ);
      if(safe2<safe) safe = safe2;
    }
  }
  return (safe<p->fullSpeed?safe:p->fullSpeed);
}
inline unsigned long U16SquaredToU32(unsigned int val) {
  long res;
   __asm__ __volatile__ ( // 15 Ticks
   "mul %A1,%A1 \n\t"
   "movw %A0,r0 \n\t"
   "mul %B1,%B1 \n\t"
   "movw %C0,r0 \n\t"
   "mul %A1,%B1 \n\t"
   "clr %A1 \n\t"
   "add %B0,r0 \n\t"
   "adc %C0,r1 \n\t"
   "adc %D0,%A1 \n\t"
   "add %B0,r0 \n\t"
   "adc %C0,r1 \n\t"
   "adc %D0,%A1 \n\t"
   "clr r1 \n\t"
  : "=&r"(res),"=r"(val)
  : "1"(val)
   );
  return res;
}
/** Update parameter used by 
*/
void updateStepsParameter(PrintLine *p/*,byte caller*/) {
    if(p->flags & FLAG_WARMUP) return;
    p->vStart = p->vMax*p->startFactor; //starting speed
    p->vEnd = p->vMax*p->endFactor;
    unsigned long vmax2 = U16SquaredToU32(p->vMax);
    p->accelSteps = ((vmax2-U16SquaredToU32(p->vStart))/(p->accelerationPrim<<1))+1; // Always add 1 for missing precision
    p->decelSteps = ((vmax2-U16SquaredToU32(p->vEnd))/(p->accelerationPrim<<1))+1;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceStart = (float)p->advanceFull*p->startFactor*p->startFactor;
    p->advanceEnd = (float)p->advanceFull*p->endFactor*p->endFactor;
#endif
#endif
    if(p->accelSteps+p->decelSteps>=p->stepsRemaining) { // can't reach limit speed
      unsigned int red = (p->accelSteps+p->decelSteps+2-p->stepsRemaining)>>1;
      if(red<p->accelSteps)
        p->accelSteps-=red;
      else
        p->accelSteps = 0;
      if(red<p->decelSteps) {
        p->decelSteps-=red;
      } else
        p->decelSteps = 0;
    }
    p->joinFlags|=FLAG_JOIN_STEPPARAMS_COMPUTED;
#ifdef DEBUG_QUEUE_MOVE
    if(DEBUG_ECHO) {
      out.print_int_P(PSTR("ID:"),(int)p);
    //  out.println_int_P(PSTR("/"),(int)caller);
      out.print_int_P(PSTR("vStart/End:"),p->vStart);
      out.println_int_P(PSTR("/"),p->vEnd);
      out.print_int_P(PSTR("accel/decel steps:"),p->accelSteps);
      out.println_int_P(PSTR("/"),p->decelSteps);
      out.print_float_P(PSTR("st./end factor:"),p->startFactor);
      out.println_float_P(PSTR("/"),p->endFactor);
#if USE_OPS==1
      if(!(p->dir & 128) && printer_state.opsMode==2)
        out.println_long_P(PSTR("Reverse at:"),p->opsReverseSteps);
#endif
      out.println_int_P(PSTR("flags:"),p->flags);
      out.println_int_P(PSTR("joinFlags:"),p->joinFlags);
    }
#endif
}
/** Checks, if the next segment has its stepping parameter computed. Normally this is the case and we have nothing to do.
If not, we do it here, bofore we have to do it in the interrupt routine.
*/
void finishNextSegment() {
  if(lines_count<=1) return; // nothing to do
  byte pos = lines_pos+1;
  if(pos>=MOVE_CACHE_SIZE) pos = 0;
  PrintLine *p = &lines[pos];
  if(p->joinFlags & FLAG_JOIN_END_FIXED) return;
  BEGIN_INTERRUPT_PROTECTED; 
  p->flags |= FLAG_BLOCKED;
  END_INTERRUPT_PROTECTED;
  updateStepsParameter(p/*,1*/);
  p->flags &= ~FLAG_BLOCKED;
}


void updateTrapezoids(byte p) {
  PrintLine *act = &lines[p],*prev;
  if(lines_count<4) {
    if(!(act->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED))
      updateStepsParameter(act/*,2*/);
    return;
  }
  byte n=lines_count-2; // ignore active segment and following segment
  if(n>PATH_PLANNER_CHECK_SEGMENTS) n=PATH_PLANNER_CHECK_SEGMENTS; // Limit time spend in updating path. If we have many short moves this takes to much time and with large moves it is not necessary.
  while(n>0 && lines_count>2) {
    p--;if(p==255) p = MOVE_CACHE_SIZE-1;
    prev = &lines[p];
    BEGIN_INTERRUPT_PROTECTED; 
    prev->flags |= FLAG_BLOCKED;    
    END_INTERRUPT_PROTECTED;
#if USE_OPS==1
    if(printer_state.opsMode && printmoveSeen) {
      if((prev->dir & 136)==136 && (act->dir & 136)!=136) {
        if((act->dir & 64)!=0 || act->distance>printer_state.opsMinDistance) { // Switch printing - travel
          act->joinFlags |= FLAG_JOIN_START_RETRACT | FLAG_JOIN_START_FIXED; // enable retract for this point
          prev->joinFlags |= FLAG_JOIN_END_FIXED;
          updateStepsParameter(prev/*,4*/);
          updateStepsParameter(act/*,4*/);
          act->flags &= ~FLAG_BLOCKED;
          prev->flags &= ~FLAG_BLOCKED;
          return;
        } else {
          act->joinFlags |= FLAG_JOIN_NO_RETRACT;
        }
      } else
      if((prev->dir & 136)!=136 && (act->dir & 136)==136) { // Switch travel - print
        prev->joinFlags |= FLAG_JOIN_END_RETRACT | FLAG_JOIN_END_FIXED; // reverse retract for this point
        if(printer_state.opsMode==2) {
          prev->opsReverseSteps = ((long)printer_state.opsPushbackSteps*(long)printer_state.maxExtruderSpeed*TIMER0_PRESCALE)/prev->fullInterval;
          long ponr = prev->stepsRemaining/(1.0+0.01*printer_state.opsMoveAfter);
          if(prev->opsReverseSteps>ponr)
            prev->opsReverseSteps = ponr;
        }
       //if((prev->joinFlags & FLAG_JOIN_START_RETRACT) /*&& (prev->distance<printer_state.opsMinDistance)*/) {
       /*   prev->joinFlags &= ~(FLAG_JOIN_END_RETRACT | FLAG_JOIN_START_RETRACT); // Disable retract, because the distance is short
          prev->joinFlags |= FLAG_JOIN_NO_RETRACT;
        } else {*/
          act->joinFlags |= FLAG_JOIN_START_FIXED; // Wait only with safe speeds!
          updateStepsParameter(prev/*,5*/);
          updateStepsParameter(act/*,5*/);
          act->flags &= ~FLAG_BLOCKED;
          prev->flags &= ~FLAG_BLOCKED;
          return;
       // }
      }
    }
#endif
    if(prev->joinFlags & FLAG_JOIN_END_FIXED) { // Nothing to update from here on
      act->joinFlags |= FLAG_JOIN_START_FIXED; // Wait only with safe speeds!
      prev->flags &= ~FLAG_BLOCKED;      
      updateStepsParameter(act/*,3*/);
      act->flags &= ~FLAG_BLOCKED;
      return;
    }
    float vmax = (act->fullSpeed>prev->fullSpeed?prev->fullSpeed:act->fullSpeed);
    float vleft = act->fullSpeed*act->endFactor;
    float vmax_left = sqrt(vleft*vleft+2.0*act->acceleration*act->distance);
    float vright = prev->fullSpeed*prev->startFactor;
    float vmax_right = sqrt(vright*vright+2.0*prev->acceleration*prev->distance);
    float junction_speed = (vmax_left<vmax_right?vmax_left:vmax_right);
    if(junction_speed>vmax) junction_speed = vmax;
    prev->endFactor = junction_speed/prev->fullSpeed;
    act->startFactor = junction_speed/act->fullSpeed;
    float jerk = computeJerk(prev,act);
    if(jerk>printer_state.maxJerk) {
      jerk =printer_state.maxJerk/jerk;
      prev->endFactor *= jerk;
      act->startFactor *= jerk;
      prev->joinFlags |= FLAG_JOIN_END_FIXED;
      act->joinFlags |= FLAG_JOIN_START_FIXED;
      updateStepsParameter(prev/*,6*/);
      updateStepsParameter(act/*,6*/);
      act->flags &= ~FLAG_BLOCKED;
      prev->flags &= ~FLAG_BLOCKED;
      return;
    }
    if(prev->endFactor>0.999 || act->startFactor>0.999) { // speed fixed by limit 
      prev->joinFlags |= FLAG_JOIN_END_FIXED;
      act->joinFlags |= FLAG_JOIN_START_FIXED;
      updateStepsParameter(act/*,7*/);
      updateStepsParameter(prev/*,7*/);
      act->flags &= ~FLAG_BLOCKED;
      prev->flags &= ~FLAG_BLOCKED;
      return;
    }
    // here prev is blocked
    updateStepsParameter(act/*,7*/);
    act->flags &= ~FLAG_BLOCKED; // unblock for interrupt routine
    prev->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    act = prev;
    n--;
    if(lines_count>=MOVE_CACHE_LOW) { // we have time for checks
        UI_MEDIUM; // do check encoder
        check_periodical(); // Temperature update
    }
  }
  if(!(act->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED)) // Always stop with computed segments
    updateStepsParameter(act/*,13*/);
  act->flags &= ~FLAG_BLOCKED;
}
void move_steps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop) {
  float saved_feedrate = printer_state.feedrate;
  for(byte i=0; i < 4; i++) {
      printer_state.destinationSteps[i] = printer_state.currentPositionSteps[i];
  }
  printer_state.destinationSteps[0]+=x;
  printer_state.destinationSteps[1]+=y;
  printer_state.destinationSteps[2]+=z;
#ifdef Z_BACKSLASH
  if(z>0 && lastzdir!=1) {
     lastzdir = 1;
     printer_state.currentPositionSteps[2]-=Z_BACKSLASH*axis_steps_per_unit[2];
  } else if(z<0 && lastzdir!=-1) {
     lastzdir=-1;
     printer_state.currentPositionSteps[2]+=Z_BACKSLASH*axis_steps_per_unit[2];
  }
#endif
  printer_state.destinationSteps[3]+=e;
  printer_state.feedrate = feedrate;
  queue_move(check_endstop,false);
  printer_state.feedrate = saved_feedrate;
  if(waitEnd)
    wait_until_end_of_move();
}
/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move.
*/
void queue_move(byte check_endstops,byte pathOptimize)
{
  printer_state.flag0 &= ~1; // Motor is enabled now 
  while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
    gcode_read_serial();
    check_periodical();
  }
  PrintLine *p = &lines[lines_write_pos];
  byte newPath=0;
  if(lines_count==0 && waitRelax==0 && pathOptimize) { // First line after some time - warmup needed
#ifdef DEBUG_OPS
    out.println_P(PSTR("New path"));
#endif
    byte w = 3;
    newPath=1;
    while(w) {
      p->flags = FLAG_WARMUP;
      p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
      p->dir = 0;
      p->primaryAxis = w;
      p->accelerationPrim = p->facceleration = 10000*(unsigned int)w;
      lines_write_pos++;
      if(lines_write_pos>=MOVE_CACHE_SIZE) lines_write_pos = 0;
BEGIN_INTERRUPT_PROTECTED
      lines_count++;
END_INTERRUPT_PROTECTED
      p = &lines[lines_write_pos];
      w--;
    }
  }
  
  bool critical=false;
  float axis_diff[4]; // Axis movement in mm
  long axis_interval[4];  
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS; 
  else p->flags = 0;
  p->joinFlags = 0;
  if(!pathOptimize) p->joinFlags = FLAG_JOIN_END_FIXED;
  p->dir = 0;
#if min_software_endstop_x == true
    if (printer_state.destinationSteps[0] < 0) printer_state.destinationSteps[0] = 0.0;
#endif
#if min_software_endstop_y == true
    if (printer_state.destinationSteps[1] < 0) printer_state.destinationSteps[1] = 0.0;
#endif
#if min_software_endstop_z == true
    if (printer_state.destinationSteps[2] < 0) printer_state.destinationSteps[2] = 0.0;
#endif

#if max_software_endstop_x == true
    if (printer_state.destinationSteps[0] > printer_state.xMaxSteps) printer_state.destinationSteps[0] = printer_state.xMaxSteps;
#endif
#if max_software_endstop_y == true
    if (printer_state.destinationSteps[1] > printer_state.yMaxSteps) printer_state.destinationSteps[1] = printer_state.yMaxSteps;
#endif
#if max_software_endstop_z == true
    if (printer_state.destinationSteps[2] > printer_state.zMaxSteps) printer_state.destinationSteps[2] = printer_state.zMaxSteps;
#endif
  //Find direction
#if DRIVE_SYSTEM==0
  for(byte i=0; i < 4; i++) {
    if((p->delta[i]=printer_state.destinationSteps[i]-printer_state.currentPositionSteps[i])>=0) {
      p->dir |= 1<<i;
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i];
    } else {
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i];
      p->delta[i] = -p->delta[i];
    }
    if(p->delta[i]) p->dir |= 16<<i;
    printer_state.currentPositionSteps[i] = printer_state.destinationSteps[i];
  }
#else
  long deltax = printer_state.destinationSteps[0]-printer_state.currentPositionSteps[0];
  long deltay = printer_state.destinationSteps[1]-printer_state.currentPositionSteps[1];
#if DRIVE_SYSTEM==1
  p->delta[2] = printer_state.destinationSteps[2]-printer_state.currentPositionSteps[2];
  p->delta[3] = printer_state.destinationSteps[3]-printer_state.currentPositionSteps[3]; 
  p->delta[0] = deltax+deltay; 
  p->delta[1] = deltax-deltay; 
#endif
#if DRIVE_SYSTEM==2 
p->delta[2] = printer_state.destinationSteps[2]-printer_state.currentPositionSteps[2]; 
p->delta[3] = printer_state.destinationSteps[3]-printer_state.currentPositionSteps[3]; 
p->delta[0] = deltay+deltax; 
p->delta[1] = deltay-deltax; 
#endif
  //Find direction 
  for(byte i=0; i < 4; i++) { 
    if(p->delta[i]>=0) { 
      p->dir |= 1<<i;
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i]; 
    } else { 
      axis_diff[i] = p->delta[i]*inv_axis_steps_per_unit[i]; 
      p->delta[i] = -p->delta[i]; 
    } 
    if(p->delta[i]) p->dir |= 16<<i;
    printer_state.currentPositionSteps[i] = printer_state.destinationSteps[i];
  }
#endif
  if(printer_state.extrudeMultiply!=100) {
    p->delta[3]=(p->delta[3]*printer_state.extrudeMultiply)/100;
  }
  if(!(p->dir & 240)) {
    if(newPath) { // need to delete dummy elements, otherwise commands can get locked.
      lines_count = 0;
      lines_pos = lines_write_pos;
    }
    return; // No move in command
  }
#if USE_OPS==1
  p->opsReverseSteps=0;
#endif

  //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  byte primary_axis;
  if(p->delta[1] > p->delta[0] && p->delta[1] > p->delta[2] && p->delta[1] > p->delta[3]) primary_axis = 1;
  else if (p->delta[0] > p->delta[2] && p->delta[0] > p->delta[3]) primary_axis = 0;
  else if (p->delta[2] > p->delta[3]) primary_axis = 2;
  else primary_axis = 3;
  p->primaryAxis = primary_axis;
  p->stepsRemaining = p->delta[primary_axis];
  //Feedrate calc based on XYZ travel distance
  if(p->dir & 112) {
    if(p->dir & 64) {
      p->distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1] + axis_diff[2] * axis_diff[2]);
    } else {
      p->distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
    }
  }  else if(p->dir & 128)
    p->distance = abs(axis_diff[3]);
  else {
    return; // no steps to take, we are finished
  }    
  float time_for_move = (float)(60*F_CPU)*p->distance / printer_state.feedrate; // time is in ticks
  if(lines_count<MOVE_CACHE_LOW && time_for_move<LOW_TICKS_PER_MOVE) { // Limit speed to keep cache full.
    time_for_move = LOW_TICKS_PER_MOVE;
    critical=true; 
  }
  UI_MEDIUM; // do check encoder
  // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
  long limitInterval = time_for_move/p->stepsRemaining; // until not violated by other constraints it is your target speed
  axis_interval[0] = 60.0*abs(axis_diff[0])*F_CPU/(max_feedrate[0]*p->stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
  if(axis_interval[0]>limitInterval) limitInterval = axis_interval[0];
  axis_interval[1] = 60.0*abs(axis_diff[1])*F_CPU/(max_feedrate[1]*p->stepsRemaining);
  if(axis_interval[1]>limitInterval) limitInterval = axis_interval[1];
  if(p->dir & 64) { // normally no move in z direction
    axis_interval[2] = 60.0*abs((float)axis_diff[2])*(float)F_CPU/(float)(max_feedrate[2]*p->stepsRemaining); // must prevent overflow!
    if(axis_interval[2]>limitInterval) limitInterval = axis_interval[2];
  } else axis_interval[2] = 0;
  axis_interval[3] = 60.0*abs(axis_diff[3])*F_CPU/(max_feedrate[3]*p->stepsRemaining);
  if(axis_interval[3]>limitInterval) limitInterval = axis_interval[3];  
  p->fullInterval = limitInterval>200 ? limitInterval : 200; // This is our target speed
  // new time at full speed = limitInterval*p->stepsRemaining [ticks]
  time_for_move = (float)limitInterval*(float)p->stepsRemaining; // for large z-distance this overflows with long computation
  float inv_time_s = (float)F_CPU/time_for_move;
  if(p->dir & 16) {
    axis_interval[0] = time_for_move/p->delta[0];
    p->speedX = axis_diff[0]*inv_time_s;
  } else p->speedX = 0;
  if(p->dir & 32) {
    axis_interval[1] = time_for_move/p->delta[1];
    p->speedY = axis_diff[1]*inv_time_s;
  } else p->speedY = 0;
  if(p->dir & 64) {
    axis_interval[2] = time_for_move/p->delta[2];
    p->speedZ = axis_diff[2]*inv_time_s;
  } else p->speedZ = 0;
  if(p->dir & 128)
    axis_interval[3] = time_for_move/p->delta[3];
  p->fullSpeed = p->distance*inv_time_s;
  

  //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
  byte is_print_move = (p->dir & 136)==136; // are we printing
#if USE_OPS==1
  if(is_print_move) printmoveSeen = 1;
#endif
  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
  #ifdef RAMP_ACCELERATION
    
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowest_axis_plateau_time_repro = 1e20; // repro to reduce division Unit: 1/s
    for(byte i=0; i < 4 ; i++) {
      p->error[i] = p->delta[primary_axis] >> 1;
      if(p->dir & (16<<i)) {
        // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
        slowest_axis_plateau_time_repro = min(slowest_axis_plateau_time_repro,
               (float)axis_interval[i] * (float)(is_print_move ?  axis_steps_per_sqr_second[i] : axis_travel_steps_per_sqr_second[i])); //  steps/s^2 * step/tick  Ticks/s^2
      }
    }
    p->accelerationPrim = slowest_axis_plateau_time_repro / axis_interval[primary_axis]; // a = v/t = F_CPU/(c*t): Steps/s^2
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    p->facceleration = 262144.0*(float)p->accelerationPrim/F_CPU; // will overflow without float!
    p->acceleration = slowest_axis_plateau_time_repro*p->fullSpeed/((float)F_CPU); // mm/s^2
    p->startFactor = p->endFactor = safeSpeed(p)/p->fullSpeed;
    p->vMax = F_CPU / p->fullInterval; // maximum steps per second, we can reach
   // if(p->vMax>46000)  // gets overflow in N computation
   //   p->vMax = 46000;
    //p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#ifdef USE_ADVANCE
  if((p->dir & 112)==0 || (p->dir & 128)==0 || (p->dir & 8)==0) {
#ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceRate = 0; // No head move or E move only or sucking filament back 
    p->advanceFull = 0;
#endif
    p->advanceL = 0;
  } else {
    float speedE = axis_diff[3]*inv_time_s; // [mm/s]
    float advlin = speedE*current_extruder->advanceL*0.001*axis_steps_per_unit[3];
    p->advanceL = (65536*advlin)/p->vMax; //advanceLscaled = (65536*vE*k2)/vMax
 #ifdef ENABLE_QUADRATIC_ADVANCE;
    p->advanceFull = 65536*current_extruder->advanceK*speedE*speedE; // Steps*65536 at full speed
    long steps = (U16SquaredToU32(p->vMax))/(p->accelerationPrim<<1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
    p->advanceRate = p->advanceFull/steps;
    if((p->advanceFull>>16)>maxadv) {
        maxadv = (p->advanceFull>>16);
        maxadvspeed = speedE;
    }
 #endif
    if(advlin>maxadv2) {
      maxadv2 = advlin;
      maxadvspeed = speedE;
    }
  }
#endif
    UI_MEDIUM; // do check encoder

    updateTrapezoids(lines_write_pos);
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
  #else
  #ifdef USE_ADVANCE
  #ifdef ENABLE_QUADRATIC_ADVANCE
    p->advanceRate = 0; // No advance for constant speeds
    p->advanceFull = 0;
  #endif
  #endif
  #endif
  
  // Correct integers for fixed point math used in bresenham_step
  if(p->fullInterval<MAX_HALFSTEP_INTERVAL || critical) 
    p->halfstep = 0;
  else {
    p->halfstep = 1;
    p->error[0] = p->error[1] = p->error[2] = p->error[3] = p->delta[primary_axis];
  }
#ifdef DEBUG_STEPCOUNT
  p->totalStepsRemaining = abs(p->delta[0])+abs(p->delta[1]);
#endif
#ifdef DEBUG_QUEUE_MOVE
  if(DEBUG_ECHO) {
    log_printLine(p);  
      out.println_long_P(PSTR("limitInterval:"), limitInterval);
      out.println_float_P(PSTR("Move distance on the XYZ space:"), p->distance);
      out.println_float_P(PSTR("Commanded feedrate:"), printer_state.feedrate);
      out.println_float_P(PSTR("Constant full speed move time:"), time_for_move);
      //log_long_array(PSTR("axis_int"),(long*)axis_interval);
      //out.println_float_P(PSTR("Plateau repro:"),slowest_axis_plateau_time_repro);
  }
#endif
  // Make result permanent
  lines_write_pos++;
  if(lines_write_pos>=MOVE_CACHE_SIZE) lines_write_pos = 0;
  waitRelax = 70;
BEGIN_INTERRUPT_PROTECTED
  lines_count++;
END_INTERRUPT_PROTECTED
#ifdef DEBUG_FREE_MEMORY
    check_mem();
#endif
}
inline unsigned int ComputeV(long timer,long accel) {
  unsigned int res;
  // 38 Ticks
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free  
     // Result LSB first: %A0, %B0, %A1
     "mul %B1,%A2 \n\t"
     "mov %A0,r1 \n\t"
     "mul %B1,%C2 \n\t"
     "mov %B0,r0 \n\t"
     "mov %A1,r1 \n\t"
     "mul %B1,%B2 \n\t"
     "add %A0,r0 \n\t"
     "adc %B0,r1 \n\t"
     "adc %A1,%D2 \n\t"
     "mul %C1,%A2 \n\t"
     "add %A0,r0 \n\t"
     "adc %B0,r1 \n\t"
     "adc %A1,%D2 \n\t"
     "mul %C1,%B2 \n\t"
     "add %B0,r0 \n\t"
     "adc %A1,r1 \n\t"
     "mul %D1,%A2 \n\t"
     "add %B0,r0 \n\t"
     "adc %A1,r1 \n\t"
     "mul %C1,%C2 \n\t"
     "add %A1,r0 \n\t"
     "mul %D1,%B2 \n\t"
     "add %A1,r0 \n\t"
     "lsr %A1 \n\t"
     "ror %B0 \n\t"
     "ror %A0 \n\t"
     "lsr %A1 \n\t"
     "ror %B0 \n\t"
     "ror %A0 \n\t"
     "clr r1 \n\t"
  :"=&r"(res),"=r"(timer),"=r"(accel)
  :"1"(timer),"2"(accel)
  : );
  // unsigned int v = ((timer>>8)*cur->accel)>>10;
  return res;
}
// Multiply two 16 bit values and return 32 bit result
inline unsigned long mulu6xu16to32(unsigned int a,unsigned int b) {
  unsigned long res;
  // 18 Ticks = 1.125 us
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free  
     // Result LSB first: %A0, %B0, %A1
     "clr r18 \n\t"
     "mul %B2,%B1 \n\t" // mul hig bytes
     "movw %C0,r0 \n\t"
     "mul %A1,%A2 \n\t" // mul low bytes
     "movw %A0,r0 \n\t"
     "mul %A1,%B2 \n\t"
     "add %B0,r0 \n\t"
     "adc %C0,r1 \n\t"
     "adc %D0,r18 \n\t"
     "mul %B1,%A2 \n\t"
     "add %B0,r0 \n\t"
     "adc %C0,r1 \n\t"
     "adc %D0,r18 \n\t"
     "clr r1 \n\t"
  :"=&r"(res),"=r"(a),"=r"(b)
  :"1"(a),"2"(b)
  :"r18" );
  // return (long)a*b;
  return res;
}
// Multiply two 16 bit values and return 32 bit result
inline unsigned int mulu6xu16shift16(unsigned int a,unsigned int b) {
  unsigned int res;
  // 18 Ticks = 1.125 us
  __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free  
     // Result LSB first: %A0, %B0, %A1
     "clr r18 \n\t"
     "mul %B2,%B1 \n\t" // mul hig bytes
     "movw %A0,r0 \n\t"
     "mul %A1,%A2 \n\t" // mul low bytes
     "mov r19,r1 \n\t"
     "mul %A1,%B2 \n\t"
     "add r19,r0 \n\t"
     "adc %A0,r1 \n\t"
     "adc %B0,r18 \n\t"
     "mul %B1,%A2 \n\t"
     "add r19,r0 \n\t"
     "adc %A0,r1 \n\t"
     "adc %B0,r18 \n\t"
     "clr r1 \n\t"
  :"=&r"(res),"=r"(a),"=r"(b)
  :"1"(a),"2"(b)
  :"r18","r19" );
  // return ((long)a*b)>>16;
  return res;
}

/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
*/
int lastblk=-1;
long cur_errupd;
inline long bresenham_step() {
  if(cur == 0) {
      sei();
      cur = &lines[lines_pos];
      if(cur->flags & FLAG_BLOCKED) { // This step is in computation - shouldn't happen
        if(lastblk!=(int)cur) {
          lastblk = (int)cur;
          out.println_int_P(PSTR("BLK "),(unsigned int)lines_count);
        }
        cur = 0;
        return 2000;
      }
      lastblk = -1;
#ifdef INCLUDE_DEBUG_NO_MOVE
      if(DEBUG_NO_MOVES) { // simulate a move, but do nothing in reality
        lines_pos++;
        if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
        cur = 0;
        cli();
        --lines_count;
        return 1000;
      }
#endif
      if(cur->flags & FLAG_WARMUP) {
          // This is a warmup move to initalize the path planner correctly. Just waste 
          // a bit of time to get the planning up to date.
#if USE_OPS==1
         if(cur->joinFlags & FLAG_JOIN_END_RETRACT) { // Make sure filament is pushed back
            if(printer_state.filamentRetracted) {
#ifdef DEBUG_OPS
              //sei();
              out.println_P(PSTR("DownW"));
              cli();
#endif
              printer_state.filamentRetracted = false;
              printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
            }
            if(printer_state.extruderStepsNeeded) {
              cur = 0;
              return 2000; // wait, work is done in other interrupt
            }
          } else if(cur->joinFlags & FLAG_JOIN_START_RETRACT) {
            if(!printer_state.filamentRetracted) {
#ifdef DEBUG_OPS
              //sei();
              out.println_P(PSTR("UpW"));
              cli();
#endif
              printer_state.filamentRetracted = true;
              cli();
              printer_state.extruderStepsNeeded-=printer_state.opsRetractSteps;
              cur->joinFlags |= FLAG_JOIN_WAIT_EXTRUDER_UP;
            }
            if(printer_state.extruderStepsNeeded) {
              cur = 0;
              return 2000; // wait, work is done in other interrupt
            }
          }
#endif
          if(lines_count<=cur->primaryAxis) {cur=0;return 2000;}
          lines_pos++;
          if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
          long wait = cur->accelerationPrim;
          cur = 0;
          --lines_count;
          return(wait); // waste some time for path optimization to fill up
      } // End if WARMUP
      //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
      if(cur->dir & 16) enable_x();
      if(cur->dir & 32) enable_y();
      if(cur->dir & 64) enable_z();
      if(cur->dir & 128) extruder_enable();
      cur->joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED; // don't touch this segment any more, just for safety
#if USE_OPS==1
      if(printer_state.opsMode) { // Enabled?
        if(cur->joinFlags & FLAG_JOIN_START_RETRACT) {
         if(!printer_state.filamentRetracted) {
#ifdef DEBUG_OPS
            out.println_P(PSTR("Up"));
#endif
            printer_state.filamentRetracted = true;
            cli();
            printer_state.extruderStepsNeeded-=printer_state.opsRetractSteps;
            sei();
            cur->joinFlags |= FLAG_JOIN_WAIT_EXTRUDER_UP;
         }
        }
        // If path optimizer ran out of samples, he might miss some retractions. Solve them before printing
        else if((cur->joinFlags & FLAG_JOIN_NO_RETRACT)==0 && printmoveSeen) {
          if((cur->dir & 136)==136) {
            if(printer_state.filamentRetracted) { // Printmove and filament is still up!
#ifdef DEBUG_OPS
              out.println_P(PSTR("DownA"));
#endif
              printer_state.filamentRetracted = false;
              cli();
              printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
              cur->joinFlags |= FLAG_JOIN_WAIT_EXTRUDER_DOWN;
            } 
          } /*else if(!printer_state.filamentRetracted) {
#ifdef DEBUG_OPS
            out.println_P(PSTR("UpA"));
#endif
            printer_state.filamentRetracted = true;
            cli();
            printer_state.extruderStepsNeeded-=printer_state.opsRetractSteps;
            cur->joinFlags |= FLAG_JOIN_WAIT_EXTRUDER_UP;
          }*/
        }
        if(cur->joinFlags & FLAG_JOIN_WAIT_EXTRUDER_UP) { // Wait for filament pushback
          cli();
          if(printer_state.extruderStepsNeeded<printer_state.opsMoveAfterSteps) {
            cur=0;
            return 4000;
          }
        } else if(cur->joinFlags & FLAG_JOIN_WAIT_EXTRUDER_DOWN) { // Wait for filament pushback
          cli();
          if(printer_state.extruderStepsNeeded) {
            cur=0;
            return 4000;
          }
        }
      } // End if opsMode
#endif
      sei(); // Allow interrupts
      if(cur->halfstep) {
        cur_errupd = cur->delta[cur->primaryAxis]<<1;
        //printer_state.interval = CPUDivU2(cur->vStart);
      } else
        cur_errupd = cur->delta[cur->primaryAxis];      
      if(!(cur->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED)) {// should never happen, but with bad timings???
        updateStepsParameter(cur/*,8*/);
      }
      printer_state.vMaxReached = cur->vStart;
      printer_state.stepNumber=0;
      printer_state.timer = 0;
      cli();
      //Determine direction of movement,check if endstop was hit
      if(cur->dir & 1) {
        WRITE(X_DIR_PIN,!INVERT_X_DIR);
      } else {
        WRITE(X_DIR_PIN,INVERT_X_DIR);
      }
      if(cur->dir & 2) {
        WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
      } else {
        WRITE(Y_DIR_PIN,INVERT_Y_DIR);
      }
      if(cur->dir & 4) {
        WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
      } else {
        WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      }
#if USE_OPS==0 && !defined(USE_ADVANCE)
      if(cur->dir & 8) {
        extruder_set_direction(1);
      } else {
        extruder_set_direction(0);
      }
#endif
#ifdef USE_ADVANCE
     long h = mulu6xu16to32(cur->vStart,cur->advanceL);
     int tred = ((
#ifdef ENABLE_QUADRATIC_ADVANCE
     printer_state.advance_executed = cur->advanceStart;
     cur->advanceStart+
#endif
     h)>>16);
     printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
     printer_state.advance_steps_set = tred;
#endif
    return printer_state.interval; // Wait an other 50% fro last step to make the 100% full
  } // End cur=0
  sei();
  /* For halfstepping, we divide the actions into even and odd actions to split
     time used per loop. */
  byte do_even;
  byte do_odd;
  if(cur->halfstep) {
    do_odd = cur->halfstep & 1;
    do_even = cur->halfstep & 2;
    cur->halfstep = 3-cur->halfstep;
  } else {
    do_even = 1;
    do_odd = 1;
  }
  cli();
  if(do_even) {
   if(cur->flags & FLAG_CHECK_ENDSTOPS) {
#if X_MIN_PIN>-1
    if((cur->dir & 17)==16) if(READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING) {
#if DRIVE_SYSTEM==0
      cur->dir&=~16;
#else
      cur->dir&=~48;
#endif
    }
#endif
#if Y_MIN_PIN>-1
    if((cur->dir & 34)==32) if(READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING) {
#if DRIVE_SYSTEM==0
      cur->dir&=~32;
#else
      cur->dir&=~48;
#endif
    }
#endif
#if X_MAX_PIN>-1
    if((cur->dir & 17)==17) if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING) {
#if DRIVE_SYSTEM==0
      cur->dir&=~16;
#else
      cur->dir&=~48;
#endif
    }
#endif
#if Y_MAX_PIN>-1
    if((cur->dir & 34)==34) if(READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING) {
#if DRIVE_SYSTEM==0
      cur->dir&=~32;
#else
      cur->dir&=~48;
#endif
    }
#endif
   }
   // Test Z-Axis every step if necessary, otherwise it could easyly ruin your printer!
#if Z_MIN_PIN>-1
   if((cur->dir & 68)==64) if(READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING) {cur->dir&=~64;}
#endif
#if Z_MAX_PIN>-1
   if((cur->dir & 68)==68) if(READ(Z_MAX_PIN)!= ENDSTOP_Z_MAX_INVERTING) {cur->dir&=~64;}
#endif
  }
  byte max_loops = (printer_state.stepper_loops<=cur->stepsRemaining ? printer_state.stepper_loops : cur->stepsRemaining);
  if(cur->stepsRemaining>0) {
   for(byte loop=0;loop<max_loops;loop++) {
    if(loop>0)
#if STEPPER_HIGH_DELAY>0
      delayMicroseconds(STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY);
#else
      delayMicroseconds(DOUBLE_STEP_DELAY);
#endif
     if(cur->dir & 128) {
      if((cur->error[3] -= cur->delta[3]) < 0) {
#if USE_OPS==1 || defined(USE_ADVANCE)
        if(cur->dir & 8)
          printer_state.extruderStepsNeeded++;
        else
          printer_state.extruderStepsNeeded--;
#else
        extruder_step();
#endif
        cur->error[3] += cur_errupd;
      }
    }    
    if(cur->dir & 16) {
      if((cur->error[0] -= cur->delta[0]) < 0) {
        WRITE(X_STEP_PIN,HIGH);
        cur->error[0] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
        cur->totalStepsRemaining--;
#endif
      }
    }
    if(cur->dir & 32) {
      if((cur->error[1] -= cur->delta[1]) < 0) {
        WRITE(Y_STEP_PIN,HIGH);
        cur->error[1] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
        cur->totalStepsRemaining--;
#endif
      }
    }
    if(cur->dir & 64) {
      if((cur->error[2] -= cur->delta[2]) < 0) {
        WRITE(Z_STEP_PIN,HIGH);
        cur->error[2] += cur_errupd;
      }
    }
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
#if USE_OPS==0 && !defined(USE_ADVANCE)
    extruder_unstep();
#endif
    WRITE(X_STEP_PIN,LOW);
    WRITE(Y_STEP_PIN,LOW);
    WRITE(Z_STEP_PIN,LOW);
  } // for loop
  if(do_odd) {
      sei(); // Allow interrupts for other types, timer1 is still disabled
#ifdef RAMP_ACCELERATION
      //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
      if (printer_state.stepNumber <= cur->accelSteps) { // we are accelerating
        printer_state.vMaxReached = ComputeV(printer_state.timer,cur->facceleration)+cur->vStart;
        if(printer_state.vMaxReached>cur->vMax) printer_state.vMaxReached = cur->vMax;
        unsigned int v;
        if(printer_state.vMaxReached>STEP_DOUBLER_FREQUENCY) {
 #if ALLOW_QUADSTEPPING
          if(printer_state.vMaxReached>STEP_DOUBLER_FREQUENCY*2) {
            printer_state.stepper_loops = 4;
            v = printer_state.vMaxReached>>2;
          } else {
            printer_state.stepper_loops = 2;
            v = printer_state.vMaxReached>>1;
          }
 #else
          printer_state.stepper_loops = 2;
          v = printer_state.vMaxReached>>1;
 #endif
        } else {
          printer_state.stepper_loops = 1;
          v = printer_state.vMaxReached;
        }
        printer_state.interval = CPUDivU2(v);
        printer_state.timer+=printer_state.interval;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        long advance_target =printer_state.advance_executed+cur->advanceRate;
        for(byte loop=1;loop<max_loops;loop++) advance_target+=cur->advanceRate;
        if(advance_target>cur->advanceFull)
          advance_target = cur->advanceFull;
        cli();
        long h = mulu6xu16to32(printer_state.vMaxReached,cur->advanceL);
        int tred = ((advance_target+h)>>16);
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
        printer_state.advance_executed = advance_target;
#else
        int tred = mulu6xu16shift16(printer_state.vMaxReached,cur->advanceL);
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
#endif
#endif
      } else if (cur->stepsRemaining <= cur->decelSteps) { // time to slow down
        if (!(cur->flags & FLAG_DECELERATING)) {
           printer_state.timer = 0;
           cur->flags |= FLAG_DECELERATING;
        }
        unsigned int v = ComputeV(printer_state.timer,cur->facceleration);
        if (v > printer_state.vMaxReached)   // if deceleration goes too far it can become too large
          v = cur->vEnd;
        else{
          v=printer_state.vMaxReached-v;
          if (v<cur->vEnd) v = cur->vEnd; // extra steps at the end of desceleration due to rounding erros
        }
        if(v>STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
          if(v>STEP_DOUBLER_FREQUENCY*2) {
            printer_state.stepper_loops = 4;
            v = v>>2;
          } else {
            printer_state.stepper_loops = 2;
            v = v>>1;
          }
#else
          printer_state.stepper_loops = 2;
          v = v>>1;
#endif
        } else {
          printer_state.stepper_loops = 1;
        }
        printer_state.interval = CPUDivU2(v);
        printer_state.timer+=printer_state.interval;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        long advance_target =printer_state.advance_executed-cur->advanceRate;
        for(byte loop=1;loop<max_loops;loop++) advance_target-=cur->advanceRate;
        if(advance_target<cur->advanceEnd)
          advance_target = cur->advanceEnd;
        long h=mulu6xu16to32(cur->advanceL,v);
        int tred = ((advance_target+h)>>16);
        cli();  
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
        printer_state.advance_executed = advance_target;
#else
        int tred=mulu6xu16shift16(cur->advanceL,v);
        cli();  
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
#endif
#endif
      } else {
        // If we had acceleration, we need to use the latest vMaxReached and interval
        // If we started full speed, we need to use cur->fullInterval and vMax
#ifdef USE_ADVANCE
        unsigned int v;
        if(!cur->accelSteps) {
          v = cur->vMax;
        } else {
          v = printer_state.vMaxReached;
        }
#ifdef ENABLE_QUADRATIC_ADVANCE
        long h=mulu6xu16to32(cur->advanceL,v);
        int tred = ((advance_target+h)>>16);
        cli();  
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
#else
        int tred=mulu6xu16shift16(cur->advanceL,v);
        cli();  
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
#endif
#endif
        if(!cur->accelSteps) {
          if(cur->vMax>STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
            if(cur->vMax>STEP_DOUBLER_FREQUENCY*2) {
              printer_state.stepper_loops = 4;
              printer_state.interval = cur->fullInterval>>2;
            } else {
              printer_state.stepper_loops = 2;
              printer_state.interval = cur->fullInterval>>1;
            }
#else
            printer_state.stepper_loops = 2;
            printer_state.interval = cur->fullInterval>>1;
#endif
          } else {
            printer_state.stepper_loops = 1;
            printer_state.interval = cur->fullInterval;
         }
       }
      }
#else
      printer_state.interval = cur->fullInterval; // without RAMPS always use full speed
#endif
    } // do_odd
    if(do_even) {
     printer_state.stepNumber+=max_loops;
     cur->stepsRemaining-=max_loops;
    }

#if USE_OPS==1
    if(printer_state.opsMode==2 && (cur->joinFlags & FLAG_JOIN_END_RETRACT) && printer_state.filamentRetracted && cur->stepsRemaining<=cur->opsReverseSteps) {
#ifdef DEBUG_OPS
      out.println_long_P(PSTR("DownX"),cur->stepsRemaining);
#endif
      // Point for retraction reversal reached.
      printer_state.filamentRetracted = false;
      cli();
      printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
#ifdef DEBUG_OPS
      sei();
      out.println_long_P(PSTR("N="),printer_state.extruderStepsNeeded);
#endif
  }
#endif
  } // stepsRemaining
  long interval;
  if(cur->halfstep) interval = (printer_state.interval>>1); // time to come back
  else interval = printer_state.interval;  
  if(do_even) {
    if(cur->stepsRemaining<=0 || (cur->dir & 240)==0) { // line finished
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining)
          out.println_long_P(PSTR("Missed steps:"),cur->totalStepsRemaining);
#endif

#if USE_OPS==1
     if(cur->joinFlags & FLAG_JOIN_END_RETRACT) { // Make sure filament is pushed back
        sei();
        if(printer_state.filamentRetracted) {
#ifdef DEBUG_OPS
          out.println_P(PSTR("Down"));
#endif
          printer_state.filamentRetracted = false;
          cli();
          printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
        }
        cli();
        if(printer_state.extruderStepsNeeded) {
#ifdef DEBUG_OPS
    //      sei();
    //      out.println_int_P(PSTR("W"),printer_state.extruderStepsNeeded);
#endif
          return 4000; // wait, work is done in other interrupt
        }
#ifdef DEBUG_OPS
          sei();
#endif
     }
#endif
     cli();
     lines_pos++;
     if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
     cur = 0;
     --lines_count;
       if(DISABLE_X) disable_x();
       if(DISABLE_Y) disable_y();
       if(DISABLE_Z) disable_z();
     if(lines_count==0) UI_STATUS(UI_TEXT_IDLE);
     interval = printer_state.interval = interval>>1; // 50% of time to next call to do cur=0 
   }  
#ifdef DEBUG_FREE_MEMORY
    check_mem();
#endif  
  } // Do even
  return interval;
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0
/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void kill(byte only_steppers)
{
  if((printer_state.flag0 & 1) && only_steppers) return;
  printer_state.flag0 |=1; 
  disable_x();
  disable_y();
  disable_z();
  extruder_disable();
  if(!only_steppers) {  
    extruder_set_temperature(0,0);
 #if NUM_EXTRUDER>1
    extruder_set_temperature(0,1);
 #endif
 #if NUM_EXTRUDER>2
    extruder_set_temperature(0,2);
 #endif
    heated_bed_set_temperature(0);
    UI_STATUS_UPD(UI_TEXT_KILLED);
    if(PS_ON_PIN > -1) {
      //pinMode(PS_ON_PIN,INPUT);  
      pinMode(PS_ON_PIN,OUTPUT); //GND
      digitalWrite(PS_ON_PIN, HIGH);
    }
  } else UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
}
long stepperWait = 0;
/** \brief Sets the timer 1 compare value to delay ticks.

This function sets the OCR1A compare counter  to get the next interrupt
at delay ticks measured from the last interrupt. delay must be << 2^24
*/
inline void setTimer(unsigned long delay)
{
  __asm__ __volatile__ (
  "cli \n\t"
  "tst %C[delay] \n\t" //if(delay<65536) {
  "brne else%= \n\t"
  "cpi %B[delay],255 \n\t"
  "breq else%= \n\t" // delay <65280
  "sts stepperWait,r1 \n\t" // stepperWait = 0;
  "sts stepperWait+1,r1 \n\t"	
  "sts stepperWait+2,r1 \n\t"  
  "lds %C[delay],%[time] \n\t" // Read TCNT1
  "lds %D[delay],%[time]+1 \n\t"
  "ldi r18,100 \n\t" // Add 100 to TCNT1
  "add %C[delay],r18 \n\t"
  "adc %D[delay],r1 \n\t"
  "cp %A[delay],%C[delay] \n\t" // delay<TCNT1+1
  "cpc %B[delay],%D[delay] \n\t"
  "brcc exact%= \n\t"	   
  "sts %[ocr]+1,%D[delay] \n\t" //  OCR1A = TCNT1+100;
  "sts %[ocr],%C[delay] \n\t"
  "rjmp end%= \n\t"  
  "exact%=: sts %[ocr]+1,%B[delay] \n\t" //  OCR1A = delay;
  "sts %[ocr],%A[delay] \n\t"
  "rjmp end%= \n\t"  
  "else%=: subi	%B[delay], 0x80 \n\t" //} else { stepperWait = delay-32768;
  "sbci	%C[delay], 0x00 \n\t"
  "sts stepperWait,%A[delay] \n\t"	
  "sts stepperWait+1,%B[delay] \n\t"	
  "sts stepperWait+2,%C[delay] \n\t"	
  "ldi	%D[delay], 0x80 \n\t" //OCR1A = 32768;
  "sts	%[ocr]+1, %D[delay] \n\t"
  "sts	%[ocr], r1 \n\t"
  "end%=: \n\t" 
  :[delay]"=&d"(delay) // Output
  :"0"(delay),[ocr]"i" (_SFR_MEM_ADDR(OCR1A)),[time]"i"(_SFR_MEM_ADDR(TCNT1)) // Input
  :"r18" // Clobber
  );
/* // Assembler above replaced this code 
  if(delay<65280) {
    stepperWait = 0;
    unsigned int count = TCNT1+100;
    if(delay<count)
      OCR1A = count;
    else
      OCR1A = delay;
  } else {
    stepperWait = delay-32768;
    OCR1A = 32768;
  }*/
}
volatile byte insideTimer1=0;
/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(TIMER1_COMPA_vect)
{
  if(insideTimer1) return;
  byte doExit;
  __asm__ __volatile__ (
  "ldi %[ex],0 \n\t"
  "lds r23,stepperWait+2 \n\t"
  "tst r23 \n\t" //if(stepperWait<65536) {
  "brne else%= \n\t" // Still > 65535
  "lds r23,stepperWait+1 \n\t"
  "tst r23 \n\t"
  "brne last%= \n\t" // Still not 0, go ahead
  "lds r22,stepperWait \n\t"
  "breq end%= \n\t" // stepperWait is 0, do your work
  "last%=: \n\t"
  "sts %[ocr]+1,r23 \n\t" //  OCR1A = stepper wait;
  "sts %[ocr],r22 \n\t"
  "sts stepperWait,r1 \n\t"
  "sts stepperWait+1,r1 \n\t"
  "rjmp end1%= \n\t"  
  "else%=: lds r22,stepperWait+1 \n\t" //} else { stepperWait = stepperWait-32768;
  "subi	r22, 0x80 \n\t"
  "sbci	r23, 0x00 \n\t"
  "sts stepperWait+1,r22 \n\t"	// ocr1a stays 32768
  "sts stepperWait+2,r23 \n\t"
  "end1%=: ldi %[ex],1 \n\t"	
  "end%=: \n\t" 
  :[ex]"=&d"(doExit):[ocr]"i" (_SFR_MEM_ADDR(OCR1A)):"r22","r23" );
  if(doExit) return;
  insideTimer1=1;
  OCR1A=61000;
  if(lines_count) {
    setTimer(bresenham_step());
  } else {
    if(waitRelax==0) {
#if USE_OPS==1
      // push filament in normal position if printings stops, so we have a defined starting position
      if(printer_state.opsMode && printer_state.filamentRetracted) { 
#ifdef DEBUG_OPS
        out.println_P(PSTR("DownI"));
#endif
        printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
        printer_state.filamentRetracted = false;
      }
      printmoveSeen = 0;
#endif
#ifdef USE_ADVANCE
      if(printer_state.advance_steps_set) {
        printer_state.extruderStepsNeeded-=printer_state.advance_steps_set;
#ifdef ENABLE_QUADRATIC_ADVANCE
        printer_state.advance_executed = 0;
#endif
        printer_state.advance_steps_set = 0;
      }
#endif
#if USE_OPS==1 || defined(USE_ADVANCE)
      if(!printer_state.extruderStepsNeeded) if(DISABLE_E) extruder_disable(); 
#else
      if(DISABLE_E) extruder_disable(); 
#endif
    } else waitRelax--;
    stepperWait = 0; // Importent becaus of optimization in asm at begin
    OCR1A = 65500; // Wait for next move
  }
#ifdef DEBUG_FREE_MEMORY
  check_mem();
#endif
  insideTimer1=0;
}

#if USE_OPS==1 || defined(USE_ADVANCE)
byte extruder_wait_dirchange=0; ///< Wait cycles, if direction changes. Prevents stepper from loosing steps.
char extruder_last_dir = 0;
byte extruder_speed = 0;
#endif


/** \brief Timer routine for extruder stepper.

Several methods need to move the extruder. To get a optima result,
all methods update the printer_state.extruderStepsNeeded with the
number of additional steps needed. During this interrupt, one step
is executed. This will keep the extruder moving, until the total
wanted movement is achieved. This will be done with the maximum
allowable speed for the extruder.
*/
ISR(EXTRUDER_TIMER_VECTOR)
{
#if USE_OPS==1 || defined(USE_ADVANCE)
  static byte accdelay=10;
  byte timer = EXTRUDER_OCR;
  byte wait; // Time to wait until next interrupt
  bool increasing = printer_state.extruderStepsNeeded>0;
  if(printer_state.extruderStepsNeeded==0) {
    if(extruder_speed<printer_state.minExtruderSpeed) {
      extruder_speed++;
      EXTRUDER_OCR = timer+extruder_speed;
    } else {
      extruder_last_dir = 0;
      extruder_speed=printer_state.minExtruderSpeed;
      accdelay =printer_state.extruderAccelerateDelay;
      EXTRUDER_OCR = timer+extruder_speed; // wait at start extruder speed for optimized delay
    }
  }  else if((increasing>0 && extruder_last_dir<0) || (!increasing && extruder_last_dir>0)) {
    EXTRUDER_OCR = timer+150; // Little delay to accomodate to reversed direction     
    extruder_last_dir = (increasing ? 1 : -1);
    extruder_set_direction(increasing ? 1 : 0);    
  } else {
    if(extruder_last_dir==0) {
      extruder_last_dir = (increasing ? 1 : -1);
      extruder_set_direction(increasing ? 1 : 0);    
    }
    extruder_step();
    printer_state.extruderStepsNeeded-=extruder_last_dir;
    if(extruder_speed>printer_state.maxExtruderSpeed) { // We can accelerate
      if(--accdelay==0) {
         accdelay = printer_state.extruderAccelerateDelay;
         extruder_speed--;
      }
    }
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
    EXTRUDER_OCR = timer+extruder_speed;
    extruder_unstep();
  }
/*
  EXTRUDER_OCR += printer_state.timer0Interval; // time to come back
  // The stepper signals are in strategical positions for optimal timing. If you
  // still have timing issues, add dummy commands between.
  if(printer_state.extruderStepsNeeded) {
    extruder_unstep();
    if(printer_state.extruderStepsNeeded<0) { // Backward step
      extruder_set_direction(0);
      if(extruder_wait_dirchange && extruder_last_dir==-1) {
        extruder_wait_dirchange--;
        return;
      }
      extruder_last_dir = 1;
      extruder_wait_dirchange=2;
      printer_state.extruderStepsNeeded++;
    } else { // Forward step
      extruder_set_direction(1);
      if(extruder_wait_dirchange && extruder_last_dir==1) {
        extruder_wait_dirchange--;
        return;
      }
      extruder_last_dir = -1;
      extruder_wait_dirchange=2;
      printer_state.extruderStepsNeeded--;
    }
    if(current_extruder->currentTemperatureC>=MIN_EXTRUDER_TEMP<<CELSIUS_EXTRA_BITS) // Saftey first
      extruder_step();
  } else {
    if(extruder_wait_dirchange)
      extruder_wait_dirchange--;
  }*/
#endif
}

/**
This timer is called 1953 timer per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR)
{
  static byte pwm_count = 0;
  static byte pwm_pos_set[4];
  PWM_OCR += 128; 
  if(pwm_count==0) {
#if EXT0_HEATER_PIN>-1
    if((pwm_pos_set[0] = pwm_pos[0])>0) WRITE(EXT0_HEATER_PIN,1);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
    if((pwm_pos_set[1] = pwm_pos[1])>0) WRITE(EXT1_HEATER_PIN,1);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
    if((pwm_pos_set[2] = pwm_pos[2])>0) WRITE(EXT2_HEATER_PIN,1);
#endif
#if FAN_PIN>-1
    if((pwm_pos_set[3] = pwm_pos[3])>0) WRITE(FAN_PIN,1);
#endif
  }
#if EXT0_HEATER_PIN>-1
  if(pwm_pos_set[0] == pwm_count) WRITE(EXT0_HEATER_PIN,0);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
    if(pwm_pos_set[1] == pwm_count) WRITE(EXT1_HEATER_PIN,0);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
    if(pwm_pos_set[2] == pwm_count) WRITE(EXT2_HEATER_PIN,0);
#endif
#if FAN_PIN>-1
    if(pwm_pos_set[3] == pwm_count) WRITE(FAN_PIN,0);
#endif
  sei(); 
  counter_periodical++; // Appxoimate a 25ms timer
  if(counter_periodical>=49) {
    counter_periodical=0;
    execute_periodical=1;
  }
// read analog values
#if ANALOG_INPUTS>0
if((ADCSRA & _BV(ADSC))==0) { // Conversion finished?
  osAnalogInputBuildup[osAnalogInputPos] += ADCW;
  if(++osAnalogInputCounter[osAnalogInputPos]>=_BV(ANALOG_INPUT_SAMPLE)) {
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE<12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos] <<
      (12-ANALOG_INPUT_BITS-ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE>12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos] >>
      (ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE-12);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE==12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos];
#endif
    osAnalogInputBuildup[osAnalogInputPos] = 0;
    osAnalogInputCounter[osAnalogInputPos] = 0;
    // Start next conversion
    if(++osAnalogInputPos>=ANALOG_INPUTS) osAnalogInputPos = 0;
    byte channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
      if(channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
      else
        ADCSRB &= ~_BV(MUX5);
#endif
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
  }
  ADCSRA |= _BV(ADSC);  // start next conversion
  }
#endif

 UI_FAST; // Short timed user interface action
 pwm_count++;
}


