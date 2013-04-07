/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

    Main author: repetier

*/
/**
\mainpage Repetier-Firmware for Arduino based RepRaps
<CENTER>Copyright &copy; 2011-2013 by repetier
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
- M32 <dirname> create subdirectory
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
- M204 - Set PID parameter X => Kp Y => Ki Z => Kd S<extruder> Default is current extruder. NUM_EXTRUDER=Heated bed
- M205 - Output EEPROM settings
- M206 - Set EEPROM value
- M220 S<Feedrate multiplier in percent> - Increase/decrease given feedrate
- M221 S<Extrusion flow multiplier in percent> - Increase/decrease given flow rate
- M231 S<OPS_MODE> X<Min_Distance> Y<Retract> Z<Backlash> F<ReatrctMove> - Set OPS parameter
- M232 - Read and reset max. advance values
- M233 X<AdvanceK> Y<AdvanceL> - Set temporary advance K-value to X and linear term advanceL to Y
- M251 Measure Z steps from homing stop (Delta printers). S0 - Reset, S1 - Print, S2 - Store to Z length (also EEPROM if enabled)
- M303 P<extruder/bed> S<drucktermeratur> Autodetect pid values. Use P<NUM_EXTRUDER> for heated bed.
- M350 S<mstepsAll> X<mstepsX> Y<mstepsY> Z<mstepsZ> E<mstepsE0> P<mstespE1> : Set microstepping on RAMBO board
- M400 - Wait until move buffers empty.
- M401 - Store x, y and z position.
- M402 - Go to stored position. If X, Y or Z is specified, only these coordinates are used. F changes feedrate fo rthat move.
- M500 Store settings to EEPROM
- M501 Load settings from EEPROM
- M502 Reset settings to the one in configuration.h. Does not store values in EEPROM!
- M908 P<address> S<value> : Set stepper current for digipot (RAMBO board)
*/

#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"
#include "fastio.h"
#include "ui.h"
#include <util/delay.h>
#include <SPI.h>

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
#ifdef EXT0_PID_PGAIN
#error The PID system has changed. Please use the new float number options!
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
#if EXT0_STEP_PIN<0 && NUM_EXTRUDER>0
#error EXT0_STEP_PIN not set to a pin number.
#endif
#if EXT0_DIR_PIN<0 && NUM_EXTRUDER>0
#error EXT0_DIR_PIN not set to a pin number.
#endif
#if MOVE_CACHE_SIZE<4
#error MOVE_CACHE_SIZE must be at least 5
#endif

#if DRIVE_SYSTEM==3
#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_DIAGONAL_ROD_STEPS (AXIS_STEPS_PER_MM * DELTA_DIAGONAL_ROD)
#define DELTA_DIAGONAL_ROD_STEPS_SQUARED (DELTA_DIAGONAL_ROD_STEPS * DELTA_DIAGONAL_ROD_STEPS)
#define DELTA_ZERO_OFFSET_STEPS (AXIS_STEPS_PER_MM * DELTA_ZERO_OFFSET)
#define DELTA_RADIUS_STEPS (AXIS_STEPS_PER_MM * DELTA_RADIUS)

#define DELTA_TOWER1_X_STEPS -SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER1_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_X_STEPS SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER3_X_STEPS 0.0
#define DELTA_TOWER3_Y_STEPS DELTA_RADIUS_STEPS

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#endif

#define OVERFLOW_PERIODICAL  (int)(F_CPU/(TIMER0_PRESCALE*40))
// RAM usage of variables: Non RAMPS 114+MOVE_CACHE_SIZE*59+printer_state(32) = 382 Byte with MOVE_CACHE_SIZE=4
// RAM usage RAMPS adds: 96
// RAM usage SD Card:
byte unit_inches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float axis_steps_per_unit[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float inv_axis_steps_per_unit[4]; ///< Inverse of axis_steps_per_unit for faster conversion
float max_feedrate[4] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z}; ///< Maximum allowed feedrate.
float homing_feedrate[3] = {HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z};
byte STEP_PIN[3] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN};
#ifdef RAMP_ACCELERATION
//  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
  long max_acceleration_units_per_sq_second[4] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
  long max_travel_acceleration_units_per_sq_second[4] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
  /** Acceleration in steps/s^3 in printing mode.*/
  unsigned long axis_steps_per_sqr_second[4];
  /** Acceleration in steps/s^2 in movement mode.*/
  unsigned long axis_travel_steps_per_sqr_second[4];
#endif
#if DRIVE_SYSTEM==3
DeltaSegment segments[DELTA_CACHE_SIZE];
unsigned int delta_segment_write_pos = 0; // Position where we write the next cached delta move
volatile unsigned int  delta_segment_count = 0; // Number of delta moves cached 0 = nothing in cache
byte lastMoveID = 0; // Last move ID
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
volatile byte lines_count=0;      ///< Number of lines cached 0 = nothing to do.
byte lines_pos=0;                 ///< Position for executing line movement.
long baudrate = BAUDRATE;         ///< Communication speed rate.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
int maxadv=0;
#endif
int maxadv2=0;
float maxadvspeed=0;
#endif
byte pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

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


void update_extruder_flags() {
  printer_state.flag0 &= ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
#if USE_OPS==1
  if(printer_state.opsMode!=0) {
    printer_state.flag0 |= PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
  }
#endif
#if defined(USE_ADVANCE)
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    if(extruder[i].advanceL!=0) {
      printer_state.flag0 |= PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    }
#ifdef ENABLE_QUADRATIC_ADVANCE
    if(extruder[i].advanceK!=0) printer_state.flag0 |= PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
#endif
  }
#endif
}

void update_ramps_parameter() {
#if DRIVE_SYSTEM==3
  printer_state.zMaxSteps = axis_steps_per_unit[0]*(printer_state.zLength - printer_state.zMin);
  long cart[3], delta[3];
  cart[0] = cart[1] = 0;
  cart[2] = printer_state.zMaxSteps;
  calculate_delta(cart, delta);
  printer_state.maxDeltaPositionSteps = delta[0];
  printer_state.xMaxSteps = (long)(axis_steps_per_unit[0]*(printer_state.xMin+printer_state.xLength));
  printer_state.yMaxSteps = (long)(axis_steps_per_unit[1]*(printer_state.yMin+printer_state.yLength));
  printer_state.xMinSteps = (long)(axis_steps_per_unit[0]*printer_state.xMin);
  printer_state.yMinSteps = (long)(axis_steps_per_unit[1]*printer_state.yMin);
  printer_state.zMinSteps = 0;
#else
  printer_state.xMaxSteps = (long)(axis_steps_per_unit[0]*(printer_state.xMin+printer_state.xLength));
  printer_state.yMaxSteps = (long)(axis_steps_per_unit[1]*(printer_state.yMin+printer_state.yLength));
  printer_state.zMaxSteps = (long)(axis_steps_per_unit[2]*(printer_state.zMin+printer_state.zLength));
  printer_state.xMinSteps = (long)(axis_steps_per_unit[0]*printer_state.xMin);
  printer_state.yMinSteps = (long)(axis_steps_per_unit[1]*printer_state.yMin);
  printer_state.zMinSteps = (long)(axis_steps_per_unit[2]*printer_state.zMin);
  // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
  printer_state.backlashDir &= 7;
  if(printer_state.backlashX!=0) printer_state.backlashDir |= 8;
  if(printer_state.backlashY!=0) printer_state.backlashDir |= 16;
  if(printer_state.backlashZ!=0) printer_state.backlashDir |= 32;
  /*if(printer_state.backlashDir & 56)
    OUT_P_LN("Backlash compensation enabled");
  else
    OUT_P_LN("Backlash compensation disabled");*/
#endif
#endif
  for(byte i=0;i<4;i++) {
    inv_axis_steps_per_unit[i] = 1.0f/axis_steps_per_unit[i];
#ifdef RAMP_ACCELERATION
    /** Acceleration in steps/s^3 in printing mode.*/
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
    /** Acceleration in steps/s^2 in movement mode.*/
    axis_travel_steps_per_sqr_second[i] = max_travel_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
#endif
  }
  float accel = max(max_acceleration_units_per_sq_second[0],max_travel_acceleration_units_per_sq_second[0]);
  printer_state.minimumSpeed = accel*sqrt(2.0f/(axis_steps_per_unit[0]*accel));
  update_extruder_flags();
}

/** \brief Setup of the hardware

Sets the output and input pins in accordance to your configuration. Initializes the serial interface.
Interrupt routines to measure analog values and for the stepper timerloop are started.
*/
void setup()
{
#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0>=0
SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1>=0
SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2>=0
SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3>=0
SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4>=0
SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5>=0
SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6>=0
SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7>=0
SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && PS_ON_PIN>-1
  SET_OUTPUT(PS_ON_PIN); //GND
  WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
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
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
  SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
  WRITE(X_MIN_PIN,HIGH);
#endif
#endif
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
  SET_INPUT(Y_MIN_PIN);
#if ENDSTOP_PULLUP_Y_MIN
  WRITE(Y_MIN_PIN,HIGH);
#endif
#endif
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
  SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
  WRITE(Z_MIN_PIN,HIGH);
#endif
#endif
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
  SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
  WRITE(X_MAX_PIN,HIGH);
#endif
#endif
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
  SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
  WRITE(Y_MAX_PIN,HIGH);
#endif
#endif
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
  SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
  WRITE(Z_MAX_PIN,HIGH);
#endif
#endif
#if FAN_PIN>-1
  SET_OUTPUT(FAN_PIN);
  WRITE(FAN_PIN,LOW);
#endif
#if FAN_BOARD_PIN>-1
  SET_OUTPUT(FAN_BOARD_PIN);
  WRITE(FAN_BOARD_PIN,LOW);
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
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1
  SET_OUTPUT(EXT3_HEATER_PIN);
  WRITE(EXT3_HEATER_PIN,LOW);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1
  SET_OUTPUT(EXT4_HEATER_PIN);
  WRITE(EXT4_HEATER_PIN,LOW);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1
  SET_OUTPUT(EXT5_HEATER_PIN);
  WRITE(EXT5_HEATER_PIN,LOW);
#endif

#ifdef XY_GANTRY
  printer_state.motorX = 0;
  printer_state.motorY = 0;
#endif

#if STEPPER_CURRENT_CONTROL!=CURRENT_CONTROL_MANUAL
  current_control_init(); // Set current if it is firmware controlled
#endif
  microstep_init();

#if USE_OPS==1
  printer_state.opsMode = OPS_MODE;
  printer_state.opsMinDistance = OPS_MIN_DISTANCE;
  printer_state.opsRetractDistance = OPS_RETRACT_DISTANCE;
  printer_state.opsRetractBacklash = OPS_RETRACT_BACKLASH;
  printer_state.opsMoveAfter = OPS_MOVE_AFTER;
  printer_state.filamentRetracted = false;
#endif
  printer_state.feedrate = 50; ///< Current feedrate in mm/s.
  printer_state.feedrateMultiply = 100;
  printer_state.extrudeMultiply = 100; 
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  printer_state.advance_executed = 0;
#endif
  printer_state.advance_steps_set = 0;
  printer_state.advance_lin_set = 0;
#endif
  for(byte i=0;i<NUM_EXTRUDER+3;i++) pwm_pos[i]=0;
  printer_state.currentPositionSteps[0] = printer_state.currentPositionSteps[1] = printer_state.currentPositionSteps[2] = printer_state.currentPositionSteps[3] = 0;
#if DRIVE_SYSTEM==3
  calculate_delta(printer_state.currentPositionSteps, printer_state.currentDeltaPositionSteps);
#endif
  printer_state.maxJerk = MAX_JERK;
  printer_state.maxZJerk = MAX_ZJERK;
  printer_state.interval = 5000;
  printer_state.stepper_loops = 1;
  printer_state.msecondsPrinting = 0;
  printer_state.filamentPrinted = 0;
  printer_state.flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
  printer_state.xLength = X_MAX_LENGTH;
  printer_state.yLength = Y_MAX_LENGTH;
  printer_state.zLength = Z_MAX_LENGTH;
  printer_state.xMin = X_MIN_POS;
  printer_state.yMin = Y_MIN_POS;
  printer_state.zMin = Z_MIN_POS;
  printer_state.waslasthalfstepping = 0;
#if ENABLE_BACKLASH_COMPENSATION
  printer_state.backlashX = X_BACKLASH;
  printer_state.backlashY = Y_BACKLASH;
  printer_state.backlashZ = Z_BACKLASH;
  printer_state.backlashDir = 0;
#endif  
#if USE_OPS==1 || defined(USE_ADVANCE)
  printer_state.extruderStepsNeeded = 0;
#endif
  epr_init_baudrate();
  RFSERIAL.begin(baudrate);
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

#if SDSUPPORT

  sd.initsd();

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

void defaultLoopActions() {
  //check heater every n milliseconds
  check_periodical();
  UI_MEDIUM; // do check encoder
  unsigned long curtime = millis();
  if(lines_count)
    previous_millis_cmd = curtime;
  if(max_inactive_time!=0 && (curtime-previous_millis_cmd) >  max_inactive_time ) kill(false);
  if(stepper_inactive_time!=0 && (curtime-previous_millis_cmd) >  stepper_inactive_time ) { kill(true); }
#if defined(SDCARDDETECT) && SDCARDDETECT>-1 && defined(SDSUPPORT) && SDSUPPORT
  sd.automount();
#endif
  //void finishNextSegment();
  DEBUG_MEMORY;
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
#if SDSUPPORT
    if(sd.savetosd){
        if(!(GCODE_HAS_M(code) && code->M==29)) { // still writing to file
            sd.write_command(code);
        } else {
            sd.finishWrite();
        }
#ifdef ECHO_ON_EXECUTE
        if(DEBUG_ECHO) {
           OUT_P("Echo:");
           gcode_print_command(code);
           out.println();
        }
#endif
        gcode_command_finished(code);
    } else {
        process_command(code,true);
    }
#else
    process_command(code,true);
#endif
  }
  defaultLoopActions();
}


/** \brief Optimized division

Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
This version is optimized for a 16 bit dividend and recognises the special cases
of a 24 bit and 16 bit dividend, which offen, but not always occur in updating the
interval.
*/
inline long Div4U2U(unsigned long a,unsigned int b) {
#if CPU_ARCH==ARCH_AVR
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
#else
  return a/b;
#endif
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
#if CPU_ARCH==ARCH_AVR
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
#else
  return F_CPU/divisor;
#endif
  }
}

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
    } else {
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
    if(com->F < 1)
      printer_state.feedrate = 1;
    else
      if(unit_inches)
        printer_state.feedrate = com->F*0.0042333f*(float)printer_state.feedrateMultiply;  // Factor is 25.5/60/100
      else
        printer_state.feedrate = com->F*(float)printer_state.feedrateMultiply*0.00016666666f;
  }
  return r || (GCODE_HAS_E(com) && printer_state.destinationSteps[3]!=printer_state.currentPositionSteps[3]); // ignore unproductive moves
}
inline unsigned int ComputeV(long timer,long accel) {
#if CPU_ARCH==ARCH_AVR
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
#else
  return ((timer>>8)*accel)>>10;
#endif
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
#if CPU_ARCH==ARCH_AVR
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
  return res;
#else
  return ((long)a*b)>>16;
#endif
}

/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
  This is a modified version that implements a bresenham 'multi-step' algorithm where the dominant
  cartesian axis steps may be less than the changing dominant delta axis.
*/
#if DRIVE_SYSTEM==3
int lastblk=-1;
long cur_errupd;
//#define DEBUG_DELTA_TIMER
// Current delta segment
DeltaSegment *curd;
// Current delta segment primary error increment
long curd_errupd, stepsPerSegRemaining;
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
		#endif
					cli();
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
			if(lines_count<=cur->primaryAxis) {	cur=0;return 2000;}
			lines_pos++;
			if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
			long wait = cur->accelerationPrim;
			cur = 0;
			--lines_count;
			return(wait); // waste some time for path optimization to fill up
		} // End if WARMUP
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
		// Set up delta segments
		if (cur->numDeltaSegments) {
			// If there are delta segments point to them here
			curd = &segments[cur->deltaSegmentReadPos++];
			if (cur->deltaSegmentReadPos >= DELTA_CACHE_SIZE) cur->deltaSegmentReadPos=0;
			// Enable axis - All axis are enabled since they will most probably all be involved in a move
			// Since segments could involve different axis this reduces load when switching segments and
			// makes disabling easier.
			enable_x();enable_y();enable_z();

			// Copy across movement into main direction flags so that endstops function correctly
			cur->dir |= curd->dir;
			// Initialize bresenham for the first segment
			if (cur->halfstep) {
				cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment;
				curd_errupd = cur->numPrimaryStepPerSegment = cur->numPrimaryStepPerSegment<<1;
			} else {
				cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment>>1;
				curd_errupd = cur->numPrimaryStepPerSegment;
			}
			stepsPerSegRemaining = cur->numPrimaryStepPerSegment;
	#ifdef DEBUG_DELTA_TIMER
			out.println_byte_P(PSTR("HS: "),cur->halfstep);
			out.println_long_P(PSTR("Error: "),curd_errupd);
	#endif
		} else curd=0;
		cur_errupd = (cur->halfstep ? cur->stepsRemaining << 1 : cur->stepsRemaining);

		if(!(cur->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED)) {// should never happen, but with bad timings???
			out.println_int_P(PSTR("LATE "),(unsigned int)lines_count);
			updateStepsParameter(cur/*,8*/);
		}
		printer_state.vMaxReached = cur->vStart;
		printer_state.stepNumber=0;
		printer_state.timer = 0;
		cli();
		//Determine direction of movement
		if (curd) {
			if(curd->dir & 1) {
				WRITE(X_DIR_PIN,!INVERT_X_DIR);
			} else {
				WRITE(X_DIR_PIN,INVERT_X_DIR);
			}
			if(curd->dir & 2) {
				WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
			} else {
				WRITE(Y_DIR_PIN,INVERT_Y_DIR);
			}
			if(curd->dir & 4) {
				WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
			} else {
				WRITE(Z_DIR_PIN,INVERT_Z_DIR);
			}
		}
	#if USE_OPS==1 || defined(USE_ADVANCE)
      if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)==0) // Set direction if no advance/OPS enabled
	#endif
		if(cur->dir & 8) {
			extruder_set_direction(1);
		} else {
			extruder_set_direction(0);
		}
	#ifdef USE_ADVANCE
		long h = mulu6xu16to32(cur->vStart,cur->advanceL);
		int tred = ((
		#ifdef ENABLE_QUADRATIC_ADVANCE
			(printer_state.advance_executed = cur->advanceStart)+
		#endif
			h)>>16);
		printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
		printer_state.advance_steps_set = tred;
	#endif
    if(printer_state.waslasthalfstepping && cur->halfstep==0) { // Switch halfstepping -> full stepping
      printer_state.waslasthalfstepping = 0;
      return printer_state.interval*3; // Wait an other 150% from last half step to make the 100% full
    } else if(!printer_state.waslasthalfstepping && cur->halfstep) { // Switch full to half stepping
      printer_state.waslasthalfstepping = 1;
    } else 
      return printer_state.interval; // Wait an other 50% from last step to make the 100% full
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
		if((cur->flags & FLAG_CHECK_ENDSTOPS) && (curd != 0)) {
	#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
			if((curd->dir & 17)==17) if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING) {
				curd->dir&=~16;
				cur->dir&=~16;
			}
	#endif
	#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
			if((curd->dir & 34)==34) if(READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING) {
				curd->dir&=~32;
				cur->dir&=~32;
			}
	#endif
	#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
			if((curd->dir & 68)==68) if(READ(Z_MAX_PIN)!= ENDSTOP_Z_MAX_INVERTING) {
				curd->dir&=~64;
				cur->dir&=~64;
			}
	#endif
		}
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
					if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)) { // Use interrupt for movement
						if(cur->dir & 8)
							printer_state.extruderStepsNeeded++;
						else
							printer_state.extruderStepsNeeded--;
					} else {
	#endif
						extruder_step();
	#if USE_OPS==1 || defined(USE_ADVANCE)
					}
	#endif
					cur->error[3] += cur_errupd;
				}
			}
			if (curd) {
				// Take delta steps
				if(curd->dir & 16) {
					if((cur->error[0] -= curd->deltaSteps[0]) < 0) {
						WRITE(X_STEP_PIN,HIGH);
						cur->error[0] += curd_errupd;
					#ifdef DEBUG_STEPCOUNT
						cur->totalStepsRemaining--;
					#endif
					}
				}

				if(curd->dir & 32) {
					if((cur->error[1] -= curd->deltaSteps[1]) < 0) {
						WRITE(Y_STEP_PIN,HIGH);
						cur->error[1] += curd_errupd;
					#ifdef DEBUG_STEPCOUNT
						cur->totalStepsRemaining--;
					#endif
					}
				}

				if(curd->dir & 64) {
					if((cur->error[2] -= curd->deltaSteps[2]) < 0) {
						WRITE(Z_STEP_PIN,HIGH);
						printer_state.countZSteps += ( cur->dir & 4 ? 1 : -1 );
						cur->error[2] += curd_errupd;
					#ifdef DEBUG_STEPCOUNT
						cur->totalStepsRemaining--;
					#endif
					}
				}

			#if STEPPER_HIGH_DELAY>0
				delayMicroseconds(STEPPER_HIGH_DELAY);
			#endif
				WRITE(X_STEP_PIN,LOW);
				WRITE(Y_STEP_PIN,LOW);
				WRITE(Z_STEP_PIN,LOW);
				stepsPerSegRemaining--;
				if (!stepsPerSegRemaining) {
					cur->numDeltaSegments--;
					if (cur->numDeltaSegments) {

						// Get the next delta segment
						curd = &segments[cur->deltaSegmentReadPos++];
						if (cur->deltaSegmentReadPos >= DELTA_CACHE_SIZE) cur->deltaSegmentReadPos=0;
						delta_segment_count--;

						// Initialize bresenham for this segment (numPrimaryStepPerSegment is already correct for the half step setting)
						cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment>>1;

						// Reset the counter of the primary steps. This is initialized in the line
						// generation so don't have to do this the first time.
						stepsPerSegRemaining = cur->numPrimaryStepPerSegment;

						// Change direction if necessary
						if(curd->dir & 1) {
							WRITE(X_DIR_PIN,!INVERT_X_DIR);
						} else {
							WRITE(X_DIR_PIN,INVERT_X_DIR);
						}
						if(curd->dir & 2) {
							WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
						} else {
							WRITE(Y_DIR_PIN,INVERT_Y_DIR);
						}
						if(curd->dir & 4) {
							WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
						} else {
							WRITE(Z_DIR_PIN,INVERT_Z_DIR);
						}
					} else {
						// Release the last segment
						delta_segment_count--;
						curd=0;
					}
				}
			}
	#if USE_OPS==1 || defined(USE_ADVANCE)
			if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)==0) // Use interrupt for movement
	#endif
			extruder_unstep();
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
				else {
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
				int tred = ((printer_state.advance_executed+h)>>16);
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
							printer_state.interval = cur->fullInterval<<2;
						} else {
							printer_state.stepper_loops = 2;
							printer_state.interval = cur->fullInterval<<1;
						}
		#else
						printer_state.stepper_loops = 2;
						printer_state.interval = cur->fullInterval<<1;
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
	if(cur->halfstep)
		interval = (printer_state.interval>>1);		// time to come back
	else
		interval = printer_state.interval;
	if(do_even) {
		if(cur->stepsRemaining<=0 || (cur->dir & 240)==0) { // line finished
//			out.println_int_P(PSTR("Line finished: "), (int) cur->numDeltaSegments);
//			out.println_int_P(PSTR("DSC: "), (int) delta_segment_count);
//			out.println_P(PSTR("F"));

			// Release remaining delta segments
			delta_segment_count -= cur->numDeltaSegments;
	#ifdef DEBUG_STEPCOUNT
			if(cur->totalStepsRemaining) {
				out.println_long_P(PSTR("Missed steps:"), cur->totalStepsRemaining);
				out.println_long_P(PSTR("Step/seg r:"), stepsPerSegRemaining);
				out.println_int_P(PSTR("NDS:"), (int) cur->numDeltaSegments);
				out.println_int_P(PSTR("HS:"), (int) cur->halfstep);
			}
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
	        DEBUG_MEMORY;
	} // Do even
	return interval;
}
#else
/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
  
  Normal non delta algorithm
*/
int lastblk=-1;
long cur_errupd;
inline long bresenham_step() {
  if(cur == 0) {
      sei();
      ANALYZER_ON(ANALYZER_CH0);
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
        NEXT_PLANNER_INDEX(lines_pos);
        cur = 0;
        cli();
        --lines_count;
        return 1000;
      }
#endif
        ANALYZER_OFF(ANALYZER_CH0);
      if(cur->flags & FLAG_WARMUP) {
          // This is a warmup move to initalize the path planner correctly. Just waste
          // a bit of time to get the planning up to date.
#if USE_OPS==1
         if(cur->joinFlags & FLAG_JOIN_END_RETRACT) { // Make sure filament is pushed back
            if(printer_state.filamentRetracted) {
	#ifdef DEBUG_OPS
              //sei();
              OUT_P_LN("DownW");
	#endif
              cli();
              printer_state.filamentRetracted = false;
              printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
              sei();
            }
            if(printer_state.extruderStepsNeeded) {
              cur = 0;
              return 2000; // wait, work is done in other interrupt
            }
          } else if(cur->joinFlags & FLAG_JOIN_START_RETRACT) {
            if(!printer_state.filamentRetracted) {
	#ifdef DEBUG_OPS
              OUT_P_LN("UpW");
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
          if(lines_count<=cur->primaryAxis) {
            cur=0;
            return 2000;
          }
          NEXT_PLANNER_INDEX(lines_pos);
          long wait = cur->accelerationPrim;
          cur = 0;
          cli();
          --lines_count;
          return(wait); // waste some time for path optimization to fill up
      } // End if WARMUP
/*if(DEBUG_ECHO) {
OUT_P_L_LN("MSteps:",cur->stepsRemaining);
  //OUT_P_F("Ln:",cur->startSpeed);
//OUT_P_F_LN(":",cur->endSpeed);
}*/
      //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
#ifdef XY_GANTRY
      if(cur->dir & 48) {
        enable_x();
        enable_y();
      }
#else
      if(cur->dir & 16) enable_x();
      if(cur->dir & 32) enable_y();
#endif
      if(cur->dir & 64) {
        enable_z();
      }
      if(cur->dir & 128) extruder_enable();
      cur->joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED; // don't touch this segment any more, just for safety
#if USE_OPS==1
      if(printer_state.opsMode) { // Enabled?
        if(cur->joinFlags & FLAG_JOIN_START_RETRACT) {
         if(!printer_state.filamentRetracted) {
	#ifdef DEBUG_OPS
            OUT_P_LN("Up");
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
              OUT_P_LN("DownA");
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
#if !defined(XY_GANTRY)
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
#else
      long gdx = (cur->dir & 1 ? cur->delta[0] : -cur->delta[0]); // Compute signed difference in steps
      long gdy = (cur->dir & 2 ? cur->delta[1] : -cur->delta[1]);
#if DRIVE_SYSTEM==1
      if(gdx+gdy>=0) {
        WRITE(X_DIR_PIN,!INVERT_X_DIR);
        ANALYZER_ON(ANALYZER_CH4);
      } else {        
        WRITE(X_DIR_PIN,INVERT_X_DIR);
        ANALYZER_OFF(ANALYZER_CH4);
      }
      if(gdx>gdy) {
        WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
        ANALYZER_ON(ANALYZER_CH5);
      } else {
        WRITE(Y_DIR_PIN,INVERT_Y_DIR);
        ANALYZER_OFF(ANALYZER_CH5);
      }
#endif
#if DRIVE_SYSTEM==2
      if(gdx+gdy>=0) {
        WRITE(X_DIR_PIN,!INVERT_X_DIR);
      } else {
        WRITE(X_DIR_PIN,INVERT_X_DIR);
      }
      if(gdx<=gdy) {
        WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
      } else {
        WRITE(Y_DIR_PIN,INVERT_Y_DIR);
      }
#endif
#endif
      if(cur->dir & 4) {
        WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
      } else {
        WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      }
#if USE_OPS==1 || defined(USE_ADVANCE)
      if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)==0) // Set direction if no advance/OPS enabled
#endif
        if(cur->dir & 8) {
          extruder_set_direction(1);
        } else {
          extruder_set_direction(0);
        }
#ifdef USE_ADVANCE
     long h = mulu6xu16to32(cur->vStart,cur->advanceL);
     int tred = ((
#ifdef ENABLE_QUADRATIC_ADVANCE
     (printer_state.advance_executed = cur->advanceStart)+
#endif
     h)>>16);
     printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
     printer_state.advance_steps_set = tred;
#endif
    if(printer_state.waslasthalfstepping && cur->halfstep==0) { // Switch halfstepping -> full stepping
      printer_state.waslasthalfstepping = 0;
      return printer_state.interval*3; // Wait an other 150% from last half step to make the 100% full
    } else if(!printer_state.waslasthalfstepping && cur->halfstep) { // Switch full to half stepping
      printer_state.waslasthalfstepping = 1;
    } else 
      return printer_state.interval; // Wait an other 50% from last step to make the 100% full
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
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
		if((cur->dir & 17)==16) if(READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING) {
#if DRIVE_SYSTEM==0
			cur->dir&=~16;
#else
			cur->dir&=~48;
#endif
		}
#endif
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
		if((cur->dir & 34)==32) if(READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING) {
#if DRIVE_SYSTEM==0
			cur->dir&=~32;
#else
			cur->dir&=~48;
#endif
		}
#endif
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
		if((cur->dir & 17)==17) if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING) {
#if DRIVE_SYSTEM==0
			cur->dir&=~16;
#else
			cur->dir&=~48;
#endif
		}
#endif
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
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
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
   if((cur->dir & 68)==64) if(READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING) {cur->dir&=~64;}
#endif
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
   if((cur->dir & 68)==68) if(READ(Z_MAX_PIN)!= ENDSTOP_Z_MAX_INVERTING) {cur->dir&=~64;}
#endif
  }
  byte max_loops = (printer_state.stepper_loops<=cur->stepsRemaining ? printer_state.stepper_loops : cur->stepsRemaining);
  if(cur->stepsRemaining>0) {
   for(byte loop=0;loop<max_loops;loop++) {
    ANALYZER_ON(ANALYZER_CH1);
    if(loop>0)
#if STEPPER_HIGH_DELAY>0
      delayMicroseconds(STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY);
#else
      delayMicroseconds(DOUBLE_STEP_DELAY);
#endif
     if(cur->dir & 128) {
      if((cur->error[3] -= cur->delta[3]) < 0) {
#if USE_OPS==1 || defined(USE_ADVANCE)
       if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)) { // Use interrupt for movement
          if(cur->dir & 8)
            printer_state.extruderStepsNeeded++;
          else
            printer_state.extruderStepsNeeded--;
        } else {
#endif
          extruder_step();
#if USE_OPS==1 || defined(USE_ADVANCE)
        }
#endif
        cur->error[3] += cur_errupd;
      }
    }
#if defined(XY_GANTRY)
#endif
    if(cur->dir & 16) {
      if((cur->error[0] -= cur->delta[0]) < 0) {
        ANALYZER_ON(ANALYZER_CH6);
#if DRIVE_SYSTEM==0 || !defined(XY_GANTRY)
        ANALYZER_ON(ANALYZER_CH2);
        WRITE(X_STEP_PIN,HIGH);
#else
#if DRIVE_SYSTEM==1
        if(cur->dir & 1) {
          printer_state.motorX++;
          printer_state.motorY++;
        } else {
          printer_state.motorX--;
          printer_state.motorY--;
        }
#endif
#if DRIVE_SYSTEM==2
        if(cur->dir & 1) {
          printer_state.motorX++;
          printer_state.motorY--;
        } else {
          printer_state.motorX--;
          printer_state.motorY++;
        }
#endif
#endif // XY_GANTRY
        cur->error[0] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
        cur->totalStepsRemaining--;
#endif
      }
    }
    if(cur->dir & 32) {
      if((cur->error[1] -= cur->delta[1]) < 0) {
        ANALYZER_ON(ANALYZER_CH7);
#if DRIVE_SYSTEM==0 || !defined(XY_GANTRY)
        ANALYZER_ON(ANALYZER_CH3);
        WRITE(Y_STEP_PIN,HIGH);
#else
#if DRIVE_SYSTEM==1
        if(cur->dir & 2) {
          printer_state.motorX++;
          printer_state.motorY--;
        } else {
          printer_state.motorX--;
          printer_state.motorY++;
        }
#endif
#if DRIVE_SYSTEM==2
        if(cur->dir & 2) {
          printer_state.motorX++;
          printer_state.motorY++;
        } else {
          printer_state.motorX--;
          printer_state.motorY--;
        }
#endif
#endif // XY_GANTRY
        cur->error[1] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
        cur->totalStepsRemaining--;
#endif
      }
    }
#if defined(XY_GANTRY)
    if(printer_state.motorX <= -2) {
      ANALYZER_ON(ANALYZER_CH2);
      WRITE(X_STEP_PIN,HIGH);
      printer_state.motorX += 2;
    } else if(printer_state.motorX >= 2) {
      ANALYZER_ON(ANALYZER_CH2);
      WRITE(X_STEP_PIN,HIGH);
      printer_state.motorX -= 2;
    }
    if(printer_state.motorY <= -2) {
      ANALYZER_ON(ANALYZER_CH3);
      WRITE(Y_STEP_PIN,HIGH);
      printer_state.motorY += 2;
    } else if(printer_state.motorY >= 2) {
      ANALYZER_ON(ANALYZER_CH3);
      WRITE(Y_STEP_PIN,HIGH);
      printer_state.motorY -= 2;
    }

#endif

    if(cur->dir & 64) {
      if((cur->error[2] -= cur->delta[2]) < 0) {
        WRITE(Z_STEP_PIN,HIGH);
        cur->error[2] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
        cur->totalStepsRemaining--;
#endif
      }
    }
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
#if USE_OPS==1 || defined(USE_ADVANCE)
    if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)==0) // Use interrupt for movement
#endif
      extruder_unstep();
    WRITE(X_STEP_PIN,LOW);
    WRITE(Y_STEP_PIN,LOW);
    WRITE(Z_STEP_PIN,LOW);
    ANALYZER_OFF(ANALYZER_CH1);
    ANALYZER_OFF(ANALYZER_CH2);
    ANALYZER_OFF(ANALYZER_CH3);
    ANALYZER_OFF(ANALYZER_CH6);
    ANALYZER_OFF(ANALYZER_CH7);
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
        cli();
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
#ifdef USE_ADVANCE
        unsigned int v0 = v;
#endif
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
        long h=mulu6xu16to32(cur->advanceL,v0);
        int tred = ((advance_target+h)>>16);
        cli();
        printer_state.extruderStepsNeeded+=tred-printer_state.advance_steps_set;
        printer_state.advance_steps_set = tred;
        sei();
        printer_state.advance_executed = advance_target;
#else
        int tred=mulu6xu16shift16(cur->advanceL,v0);
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
        int tred = ((printer_state.advance_executed+h)>>16);
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
              printer_state.interval = cur->fullInterval<<2;
            } else {
              printer_state.stepper_loops = 2;
              printer_state.interval = cur->fullInterval<<1;
            }
#else
            printer_state.stepper_loops = 2;
            printer_state.interval = cur->fullInterval<<1;
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
      OUT_P_L_LN("DownX",cur->stepsRemaining);
#endif
      // Point for retraction reversal reached.
      printer_state.filamentRetracted = false;
      cli();
      printer_state.extruderStepsNeeded+=printer_state.opsPushbackSteps;
#ifdef DEBUG_OPS
      sei();
      OUT_P_L_LN("N=",printer_state.extruderStepsNeeded);
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
          OUT_P_L_LN("Missed steps:",cur->totalStepsRemaining);
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
     NEXT_PLANNER_INDEX(lines_pos);
     cur = 0;
     --lines_count;
#ifdef XY_GANTRY
       if(DISABLE_X && DISABLE_Y) {
         disable_x();
         disable_y();
       }
#else
       if(DISABLE_X) disable_x();
       if(DISABLE_Y) disable_y();
#endif
       if(DISABLE_Z) disable_z();
     if(lines_count==0) UI_STATUS(UI_TEXT_IDLE);
     interval = printer_state.interval = interval>>1; // 50% of time to next call to do cur=0
   }
   DEBUG_MEMORY;
  } // Do even
  return interval;
}
#endif
void(* resetFunc) (void) = 0; //declare reset function @ address 0
/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void kill(byte only_steppers)
{
  if((printer_state.flag0 & PRINTER_FLAG0_STEPPER_DISABLED) && only_steppers) return;
  printer_state.flag0 |=PRINTER_FLAG0_STEPPER_DISABLED;
  disable_x();
  disable_y();
  disable_z();
  extruder_disable();
  if(!only_steppers) {
    for(byte i=0;i<NUM_EXTRUDER;i++)
      extruder_set_temperature(0,i);
    heated_bed_set_temperature(0);
    UI_STATUS_UPD(UI_TEXT_KILLED);
#if defined(PS_ON_PIN) && PS_ON_PIN>-1
      //pinMode(PS_ON_PIN,INPUT);
      SET_OUTPUT(PS_ON_PIN); //GND
      WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif
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
  DEBUG_MEMORY;
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
  if((printer_state.flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT)==0) return; // currently no need
  byte timer = EXTRUDER_OCR;
  bool increasing = printer_state.extruderStepsNeeded>0;
    
  // Require at least 2 steps in one direction before going to action
  if(abs(printer_state.extruderStepsNeeded)<2) {
    EXTRUDER_OCR = timer+printer_state.maxExtruderSpeed;
            ANALYZER_OFF(ANALYZER_CH2);
    extruder_last_dir = 0;
    return;
  }

/*  if(printer_state.extruderStepsNeeded==0) {
      extruder_last_dir = 0;
  }  else if((increasing>0 && extruder_last_dir<0) || (!increasing && extruder_last_dir>0)) {
    EXTRUDER_OCR = timer+50; // Little delay to accomodate to reversed direction     
    extruder_set_direction(increasing ? 1 : 0);    
    extruder_last_dir = (increasing ? 1 : -1);
    return;
  } else*/ {
    if(extruder_last_dir==0) {
      extruder_set_direction(increasing ? 1 : 0);    
      extruder_last_dir = (increasing ? 1 : -1);
    }
    extruder_step();
    printer_state.extruderStepsNeeded-=extruder_last_dir;
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif    
    extruder_unstep();
  }
  EXTRUDER_OCR = timer+printer_state.maxExtruderSpeed;
  
/* Version of 0.7 branch
  static byte accdelay=10;
  if(printer_state.extruderStepsNeeded==0) {
      extruder_last_dir = 0;
  }  else if((increasing>0 && extruder_last_dir<0) || (!increasing && extruder_last_dir>0)) {
    EXTRUDER_OCR = timer+50; // Little delay to accomodate to reversed direction     
    extruder_set_direction(increasing ? 1 : 0);    
    extruder_last_dir = (increasing ? 1 : -1);
    return;
  } else {
    if(extruder_last_dir==0) {
      extruder_set_direction(increasing ? 1 : 0);    
      extruder_last_dir = (increasing ? 1 : -1);
    }
    extruder_step();
    printer_state.extruderStepsNeeded-=extruder_last_dir;
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif    
    extruder_unstep();
  }
  EXTRUDER_OCR = timer+printer_state.maxExtruderSpeed;  
  }
  */
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
This timer is called 3906 timer per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR)
{
  static byte pwm_count = 0;
  static byte pwm_pos_set[NUM_EXTRUDER+3];
  static byte pwm_cooler_pos_set[NUM_EXTRUDER];
  PWM_OCR += 64; 
  if(pwm_count==0) {
#if EXT0_HEATER_PIN>-1
    if((pwm_pos_set[0] = pwm_pos[0])>0) WRITE(EXT0_HEATER_PIN,1);
#if EXT0_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[0] = extruder[0].coolerPWM)>0) WRITE(EXT0_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
    if((pwm_pos_set[1] = pwm_pos[1])>0) WRITE(EXT1_HEATER_PIN,1);
#if EXT1_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[1] = extruder[1].coolerPWM)>0) WRITE(EXT1_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
    if((pwm_pos_set[2] = pwm_pos[2])>0) WRITE(EXT2_HEATER_PIN,1);
#if EXT2_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[2] = extruder[2].coolerPWM)>0) WRITE(EXT2_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1
    if((pwm_pos_set[3] = pwm_pos[3])>0) WRITE(EXT3_HEATER_PIN,1);
#if EXT3_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[3] = extruder[3].coolerPWM)>0) WRITE(EXT3_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1
    if((pwm_pos_set[4] = pwm_pos[4])>0) WRITE(EXT4_HEATER_PIN,1);
#if EXT4_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[4] = pwm_pos[4].coolerPWM)>0) WRITE(EXT4_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1
    if((pwm_pos_set[5] = pwm_pos[5])>0) WRITE(EXT5_HEATER_PIN,1);
#if EXT5_EXTRUDER_COOLER_PIN>-1
    if((pwm_cooler_pos_set[5] = extruder[5].coolerPWM)>0) WRITE(EXT5_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if FAN_BOARD_PIN>-1
    if((pwm_pos_set[NUM_EXTRUDER+1] = pwm_pos[NUM_EXTRUDER+1])>0) WRITE(FAN_BOARD_PIN,1);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
    if((pwm_pos_set[NUM_EXTRUDER+2] = pwm_pos[NUM_EXTRUDER+2])>0) WRITE(FAN_PIN,1);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
    if((pwm_pos_set[NUM_EXTRUDER] = pwm_pos[NUM_EXTRUDER])>0) WRITE(HEATED_BED_HEATER_PIN,1);
#endif
  }
#if EXT0_HEATER_PIN>-1
    if(pwm_pos_set[0] == pwm_count && pwm_pos_set[0]!=255) WRITE(EXT0_HEATER_PIN,0);
#if EXT0_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[0] == pwm_count && pwm_cooler_pos_set[0]!=255) WRITE(EXT0_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
    if(pwm_pos_set[1] == pwm_count && pwm_pos_set[1]!=255) WRITE(EXT1_HEATER_PIN,0);
#if EXT1_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[1] == pwm_count && pwm_cooler_pos_set[1]!=255) WRITE(EXT1_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
    if(pwm_pos_set[2] == pwm_count && pwm_pos_set[2]!=255) WRITE(EXT2_HEATER_PIN,0);
#if EXT2_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[2] == pwm_count && pwm_cooler_pos_set[2]!=255) WRITE(EXT2_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1
    if(pwm_pos_set[3] == pwm_count && pwm_pos_set[3]!=255) WRITE(EXT3_HEATER_PIN,0);
#if EXT3_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[3] == pwm_count && pwm_cooler_pos_set[3]!=255) WRITE(EXT3_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1
    if(pwm_pos_set[4] == pwm_count && pwm_pos_set[4]!=255) WRITE(EXT4_HEATER_PIN,0);
#if EXT4_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[4] == pwm_count && pwm_cooler_pos_set[4]!=255) WRITE(EXT4_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1
    if(pwm_pos_set[5] == pwm_count && pwm_pos_set[5]!=255) WRITE(EXT5_HEATER_PIN,0);
#if EXT5_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[5] == pwm_count && pwm_cooler_pos_set[5]!=255) WRITE(EXT5_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if FAN_BOARD_PIN>-1
    if(pwm_pos_set[NUM_EXTRUDER+2] == pwm_count && pwm_pos_set[NUM_EXTRUDER+2]!=255) WRITE(FAN_BOARD_PIN,0);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
    if(pwm_pos_set[NUM_EXTRUDER+2] == pwm_count && pwm_pos_set[NUM_EXTRUDER+2]!=255) WRITE(FAN_PIN,0);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
    if(pwm_pos_set[NUM_EXTRUDER] == pwm_count && pwm_pos_set[NUM_EXTRUDER]!=255) WRITE(HEATED_BED_HEATER_PIN,0);
#endif
  sei(); 
  counter_periodical++; // Appxoimate a 100ms timer
  if(counter_periodical>=(int)(F_CPU/40960)) {
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


