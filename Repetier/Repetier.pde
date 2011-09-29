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
- M80  - Turn on power supply
- M81  - Turn off power supply
- M82  - Set E codes absolute (default)
- M83  - Set E codes relative while in Absolute Coordinates (G90) mode
- M84  - Disable steppers until next move, 
        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
- M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
- M92  - Set axis_steps_per_unit - same syntax as G92
- M115	- Capabilities string
- M140 - Set bed target temp
- M190 - Wait for bed current temp to reach target temp.
- M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
- M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)
- M203 - Set temperture monitor to Sx
- M204 - Set PID parameter X => Kp Y => Ki Z => Kd
- M205 - Output EEPROM settings
- M206 - Set EEPROM value
- M230 - Read and reset max. advance values
*/
#include "Configuration.h"
#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"
#include "fastio.h"
#include <util/delay.h>

#ifdef SDSUPPORT
#include "SdFat.h"
#endif

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
long advance_steps_executed=0;
int maxadv=0;
float maxadvspeed=0;
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
      int params = code->params & ~1;
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
    //out.println_int_P(PSTR("Axis "),i);
    inv_axis_steps_per_unit[i] = 1.0f/axis_steps_per_unit[i];
#ifdef RAMP_ACCELERATION
    // Need a minimum start speed to avoid special cases at low speeds
    //float min_speed = sqrt(2.0f/axis_steps_per_unit[i]*(max_acceleration_units_per_sq_second[i]>max_travel_acceleration_units_per_sq_second[i]?max_acceleration_units_per_sq_second[i]:max_travel_acceleration_units_per_sq_second[i]));
    //out.println_float_P(PSTR("Min speed "),min_speed);
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
#ifdef ENDSTOPPULLUPS
#if X_MIN_PIN>-1
  SET_INPUT(X_MIN_PIN); WRITE(X_MIN_PIN,HIGH);
#endif
#if Y_MIN_PIN>-1
  SET_INPUT(Y_MIN_PIN); WRITE(Y_MIN_PIN,HIGH);
#endif
#if Z_MIN_PIN>-1
  SET_INPUT(Z_MIN_PIN); WRITE(Z_MIN_PIN,HIGH);
#endif
#if X_MAX_PIN>-1
  SET_INPUT(X_MAX_PIN); WRITE(X_MAX_PIN,HIGH);
#endif
#if Y_MAX_PIN>-1
  SET_INPUT(Y_MAX_PIN); WRITE(Y_MAX_PIN,HIGH);
#endif
#if Z_MAX_PIN>-1
  SET_INPUT(Z_MAX_PIN); WRITE(Z_MAX_PIN,HIGH);
#endif
#endif
  printer_state.feedrate = 1500; ///< Current feedrate in mm/min.
  printer_state.advance_executed = printer_state.advance_target = 0;
  printer_state.maxJerk = MAX_JERK;
  epr_init_baudrate();
  Serial.begin(baudrate);
  out.println_P(PSTR("start"));
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
  setupTimerInterrupt();
}

/**
  Main processing loop. It checks perodically for new commands, checks temperatures
  and executes new incoming commands.
*/
void loop()
{
  gcode_read_serial();
  GCode *code = gcode_next_command();
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
        }
    } else {
        process_command(code);
    }
#else
    process_command(code);
#endif
  }
  //check heater every n milliseconds
  manage_temperatures(false);
  if(max_inactive_time!=0 && (millis()-previous_millis_cmd) >  max_inactive_time ) kill(false); 
  if(stepper_inactive_time!=0 && (millis()-previous_millis_cmd) >  stepper_inactive_time ) { kill(true); }
  void finishNextSegment();
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
  log_long_array(PSTR("Error"),p->error);
  out.println_int_P(PSTR("Prim:"),p->primaryAxis);
  out.println_int_P(PSTR("Dir:"),p->dir);
  out.println_int_P(PSTR("Flags:"),p->flags);
  out.println_float_P(PSTR("fullSpeed:"),p->fullSpeed);
  out.println_long_P(PSTR("vMax:"),p->vMax);
  out.println_float_P(PSTR("Acceleration:"),p->acceleration);
  out.println_long_P(PSTR("Acceleration Prim:"),p->accelerationPrim);
  out.println_long_P(PSTR("plateauN:"),p->plateauN);
  out.println_long_P(PSTR("Remaining steps:"),p->stepsRemaining);
#ifdef USE_ADVANCE
  out.println_long_P(PSTR("advanceFull:"),p->advanceFull>>16);
  out.println_long_P(PSTR("advanceRate:"),p->advanceRate);
  out.println_long_P(PSTR("adv. executed total:"),advance_steps_executed);
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

/**
  \brief Sets the destination coordinates to values stored in com.
  
  For the computation of the destination, the following facts are considered:
  - Are units inches or mm.
  - Reltive or absolute positioning with special case only extruder relative.
  - Offset in x and y direction for multiple extruder support.
*/
void get_coordinates(GCode *com)
{
  register long p;
  if(GCODE_HAS_X(com)) {
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
    if(unit_inches)
      p = com->Z*25.4*axis_steps_per_unit[2];
    else
      p = com->Z*axis_steps_per_unit[2];
    if(relative_mode)
        printer_state.destinationSteps[2] = printer_state.currentPositionSteps[2]+p;
    else
        printer_state.destinationSteps[2] = p;
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
        printer_state.feedrate = com->F*25.4f;
      else
        printer_state.feedrate = com->F;
  }
}
inline float computeJerk(PrintLine *p1,PrintLine *p2) {
   float dx = p2->speedX*p2->startFactor-p1->speedX*p1->endFactor;
   float dy = p2->speedY*p2->startFactor-p1->speedY*p1->endFactor;
   if((p1->dir & 128)==0 && (p2->dir & 128)==0)
     return sqrt(dx*dx+dy*dy);
   float dz = (p2->speedZ-p1->speedZ)*axis_steps_per_unit[2]*inv_axis_steps_per_unit[0];
   return sqrt(dx*dx+dy*dy+dz*dz);
}
inline float saveSpeed(float maxspeed) {
  float safe = printer_state.maxJerk*0.5;
  return (safe<maxspeed?safe:maxspeed);
}
/** Update parameter used by 
*/
void updateStepsParameter(PrintLine *p) {
    long vStart = p->vMax*p->startFactor; //starting speed
    long vEnd = p->vMax*p->endFactor;
    p->startN = (vStart*vStart/p->accelerationPrim)>>1; // should be divided by 2, so we have in reality 2*n
    p->endN = (vEnd*vEnd/p->accelerationPrim)>>1;
    p->accelSteps = p->plateauN-p->startN;
    p->decelSteps = p->plateauN-p->endN;
    float c0 = F_CPU*sqrt(2.0f/(float)p->accelerationPrim);
    p->startInterval = c0*(sqrt(p->startN+1)-sqrt(p->startN));
    p->endInterval = c0*(sqrt(p->endN+1)-sqrt(p->endN));
    //p->startInterval = p->fullInterval/p->startFactor; // interval in ticks/step at startspeed feedrate
    //p->endInterval = p->startInterval/p->endFactor;
#ifdef RAMP_ACCELERATION
    p->flags |= FLAG_ACCELERATION_ENABLED;
    if(p->fullInterval > p->startInterval || p->accelSteps<0) p->flags &= ~FLAG_ACCELERATION_ENABLED; // can start with target speedrate
#endif
#ifdef USE_ADVANCE
    p->advanceStart = (float)p->advanceFull*p->startFactor*p->startFactor;
    p->advanceEnd = (float)p->advanceFull*p->endFactor*p->endFactor;
#endif
    if(p->accelSteps+p->decelSteps>p->stepsRemaining) { // can't reach limit speed
      unsigned int red = (p->accelSteps+p->decelSteps+1-p->stepsRemaining)>>1;
      p->accelSteps-=red;
      p->decelSteps-=red;
    }
    p->joinFlags|=FLAG_JOIN_STEPPARAMS_COMPUTED;
#ifdef DEBUG_QUEUE_MOVE
    if(DEBUG_ECHO) {
      out.println_int_P(PSTR("ID:"),(int)p);
      out.println_int_P(PSTR("startN:"),p->startN);
      out.println_int_P(PSTR("endN:"),p->endN);
      out.println_int_P(PSTR("accelSteps:"),p->accelSteps);
      out.println_int_P(PSTR("decelSteps:"),p->decelSteps);
      out.println_float_P(PSTR("startFactor:"),p->startFactor);
      out.println_float_P(PSTR("endFactor:"),p->endFactor);
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
  updateStepsParameter(p);
  p->flags &= ~FLAG_BLOCKED;
}
void updateTrapezoids(byte p) {
  PrintLine *act = &lines[p],*prev;
  if(lines_count<3) {
    updateStepsParameter(act);
    return;
  }
  byte n=lines_count-1; // ignore active segment and following segment
  while(n>0) {
    p--;if(p==255) p = MOVE_CACHE_SIZE-1;
    prev = &lines[p];
    BEGIN_INTERRUPT_PROTECTED; 
    if(prev->joinFlags & FLAG_JOIN_END_FIXED) {
      act->flags &= ~FLAG_BLOCKED;
      SREG=sreg;
      return;
    } // must enable interrupt before exiting
    prev->flags |= FLAG_BLOCKED;    
    END_INTERRUPT_PROTECTED;
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
      prev->startFactor *= jerk;
      prev->joinFlags |= FLAG_JOIN_END_FIXED;
      updateStepsParameter(prev);
      act->joinFlags |= FLAG_JOIN_START_FIXED;
      act->flags &= ~FLAG_BLOCKED;
      prev->flags &= ~FLAG_BLOCKED;
      return;
    }
    if(prev->endFactor>0.999 || act->startFactor>0.999) { // speed fixed by limit 
      prev->joinFlags |= FLAG_JOIN_END_FIXED;
      updateStepsParameter(prev);
      act->joinFlags |= FLAG_JOIN_START_FIXED;
      act->flags &= ~FLAG_BLOCKED;
      prev->flags &= ~FLAG_BLOCKED;
      return;
    }
    act->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // if they were computed, they are now invalid
    prev->joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    act->flags &= ~FLAG_BLOCKED; // unblock for interrupt routine
    act = prev;
    n--;
  }
  prev->flags &= ~FLAG_BLOCKED;
}
/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move.
*/
void queue_move(byte check_endstops)
{
  while(lines_count>=MOVE_CACHE_SIZE) { // wait for a free entry in movement cache
    gcode_read_serial();
    manage_temperatures(false); 
  }
  PrintLine *p = &lines[lines_write_pos];
  float axis_diff[4]; // Axis movement in mm
  long axis_interval[4];  
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS; 
  else p->flags = 0;
  p->joinFlags = 0;
  p->dir = 0;
  if (min_software_endstops) {
    if (printer_state.destinationSteps[0] < 0) printer_state.destinationSteps[0] = 0.0;
    if (printer_state.destinationSteps[1] < 0) printer_state.destinationSteps[1] = 0.0;
    if (printer_state.destinationSteps[2] < 0) printer_state.destinationSteps[2] = 0.0;
  }
  if (max_software_endstops) {
    if (printer_state.destinationSteps[0] > printer_state.xMaxSteps) printer_state.destinationSteps[0] = printer_state.xMaxSteps;
    if (printer_state.destinationSteps[1] > printer_state.yMaxSteps) printer_state.destinationSteps[1] = printer_state.yMaxSteps;
    if (printer_state.destinationSteps[2] > printer_state.zMaxSteps) printer_state.destinationSteps[2] = printer_state.zMaxSteps;
  }
  //Find direction
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
  // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
  long limitInterval = time_for_move/p->stepsRemaining; // until not violated by other constraints it is your target speed
  axis_interval[0] = abs(axis_diff[0])*F_CPU/(max_feedrate[0]*p->stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
  if(axis_interval[0]>limitInterval) limitInterval = axis_interval[0];
  axis_interval[1] = abs(axis_diff[1])*F_CPU/(max_feedrate[1]*p->stepsRemaining);
  if(axis_interval[1]>limitInterval) limitInterval = axis_interval[1];
  if(p->dir & 64) { // normally no move in z direction
    axis_interval[2] = abs((float)axis_diff[2])*(float)F_CPU/(float)(max_feedrate[2]*p->stepsRemaining); // must prevent overflow!
    if(axis_interval[2]>limitInterval) limitInterval = axis_interval[2];
  } else axis_interval[2] = 0;
  axis_interval[3] = abs(axis_diff[3])*F_CPU/(max_feedrate[3]*p->stepsRemaining);
  if(axis_interval[3]>limitInterval) limitInterval = axis_interval[3];
  p->fullInterval = limitInterval; // This is our target speed
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
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  // TODO: maybe it's better to refactor into a generic enable(int axis) function, that will probably take more ram,
  // but will reduce code size
  if(p->dir & 16) enable_x();
  if(p->dir & 32) enable_y();
  if(p->dir & 64) enable_z();

  //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
  byte is_print_move = (p->dir & 136)==136; // are we printing

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
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    p->accelerationPrim = slowest_axis_plateau_time_repro / axis_interval[primary_axis]; // a = v/t = F_CPU/(c*t): Steps/s^2
    p->acceleration = slowest_axis_plateau_time_repro*p->fullSpeed/(float)F_CPU; // mm/s^2
    p->startFactor = p->endFactor = saveSpeed(p->fullSpeed)/p->fullSpeed;
    p->vMax = F_CPU / p->fullInterval; // maximum steps per second, we can reach
    if(p->vMax>46000)  // gets overflow in N computation
      p->vMax = 46000;
    p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#ifdef USE_ADVANCE
  if((p->dir & 112)==0 || (p->dir & 128)==0 || (p->dir & 8)==0) {
    p->advanceRate = 0; // No head move or E move only or sucking filament back 
    p->advanceFull = 0;
  } else {
    float speedE = axis_diff[3]*inv_time_s;
    p->advanceFull = 65536*current_extruder->advanceK*speedE*speedE;
    p->advanceRate = p->advanceFull/p->plateauN;
    if((p->advanceFull>>16)>maxadv) {
        maxadv = (p->advanceFull>>16);
        maxadvspeed = speedE;
      }
  }
#endif

    updateTrapezoids(lines_write_pos);
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
  #else
  #ifdef USE_ADVANCE
    p->advanceRate = 0; // No advance for constant speeds
    p->advanceFull = 0;
  #endif
  #endif
  
  // Correct integers for fixed point math used in bresenham_step
  if(p->fullInterval<MAX_HALFSTEP_INTERVAL) 
    p->halfstep = 0;
  else {
    p->halfstep = 1;
    p->error[0]<<=1;
    p->error[1]<<=1;
    p->error[2]<<=1;
    p->error[3]<<=1;
  }
#ifdef DEBUG_QUEUE_MOVE
  if(DEBUG_ECHO) {
    log_printLine(p);  
      out.println_long_P(PSTR("limitInterval:"), limitInterval);
      out.println_float_P(PSTR("Move distance on the XYZ space:"), p->distance);
      out.println_float_P(PSTR("Commanded feedrate:"), printer_state.feedrate);
      out.println_float_P(PSTR("Constant full speed move time:"), time_for_move);
      log_long_array(PSTR("axis_int"),(long*)axis_interval);
      out.println_float_P(PSTR("Plateau repro:"),slowest_axis_plateau_time_repro);
  }
#endif
  // Make result permanent
  lines_write_pos++;
  if(lines_write_pos>=MOVE_CACHE_SIZE) lines_write_pos = 0;
BEGIN_INTERRUPT_PROTECTED
  lines_count++;
END_INTERRUPT_PROTECTED
#ifdef DEBUG_FREE_MEMORY
    check_mem();
#endif
}
/** \brief Optimized division 

Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
This version is optimized for a 16 bit dividend and recognises the special cases
of a 24 bit and 16 bit dividend, which offen, but not always occur in updating the
interval.
*/
long Div4U2U(unsigned long a,unsigned int b) {
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
/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
*/
long cur_errupd;
inline long bresenham_step() {
  if(cur == 0) {
    if(!lines_count) {
#ifdef USE_ADVANCE
      printer_state.advance_target = 0;
      if(printer_state.advance_executed>0) {
        extruder_set_direction(0);
        _delay_us(1); 
        extruder_step();
        printer_state.advance_executed--;
        _delay_us(1); 
        extruder_unstep();
        advance_steps_executed++;
      }
#endif
      return 0; // nothing to to
    } else {
      cur = &lines[lines_pos];
      if(cur->flags & FLAG_BLOCKED) { // This step is in computation - shouldn't happen
        cur = 0;
        return 500;
      }
      cur->joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED; // don't touch this segment any more
      sei();
#ifdef INCLUDE_DEBUG_NO_MOVE
      if(DEBUG_NO_MOVES) {
        cur->dir = 0; // No steps in any direction, switches only the motors on.
      }
#endif
      if(cur->halfstep)
        cur_errupd = cur->delta[cur->primaryAxis]<<1;
      else
        cur_errupd = cur->delta[cur->primaryAxis];      
      if(!(cur->joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED)) // should never happen, but with bad timings???
        updateStepsParameter(cur);
      printer_state.stepNumber=0;
      printer_state.n = (cur->startN<<2)+1;
      printer_state.interval = cur->startInterval; // maximum allowed start speed
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
#ifdef USE_ADVANCE
      printer_state.advance_target = cur->advanceStart;
#endif
    }
  }
  sei();
  /* For halfstepping, we divide the actions into even and odd actions to split
     time used per loop. */
  byte do_even;
  byte do_odd;
  switch(cur->halfstep) {
    case 0:
      do_even = 1;
      do_odd = 1;
      break;
    case 1:
      cur->halfstep = 2;
      do_even = 0;
      do_odd = 1;
      break;
    case 2:
      cur->halfstep = 1;
      do_odd = 0;
      do_even = 1;
      break;
  }
  
  if(do_odd) {
#ifdef USE_ADVANCE
   unsigned int adv = printer_state.advance_target>>16;
    if(printer_state.advance_executed!=adv) {
      sei(); // can take too long for incoming data, give other interrupts a chance
      if(printer_state.advance_executed<adv) {
        cli();extruder_set_direction(1);sei();
        while(printer_state.advance_executed!=adv) {
          _delay_us(1); // Stepper driver needs 1us to detect switch
          cli();extruder_step();sei();
          advance_steps_executed++;
          _delay_us(1); // Stepper driver needs 1us to detect switch
          printer_state.advance_executed++;
          cli();extruder_unstep();sei();
        }
      } else {
        cli();extruder_set_direction(0);sei();
        while(printer_state.advance_executed!=adv) {
          _delay_us(1); // Stepper driver needs 1us to detect switch
          cli();extruder_step();sei();
          advance_steps_executed++;
          _delay_us(1); // Stepper driver needs 1us to detect switch
          printer_state.advance_executed--;
          cli();extruder_unstep();sei();
        }
      }
    }
#endif
   cli();
   if(cur->flags & FLAG_CHECK_ENDSTOPS) {
#if X_MIN_PIN>-1
    if(!(cur->dir & 1)) if(READ(X_MIN_PIN) != ENDSTOPS_INVERTING) {cur->dir&=~16;}
#endif
#if Y_MIN_PIN>-1
    if(!(cur->dir & 2)) if(READ(Y_MIN_PIN) != ENDSTOPS_INVERTING) {cur->dir&=~32;}
#endif
#if X_MAX_PIN>-1
    if((cur->dir & 1)) if(READ(X_MAX_PIN) != ENDSTOPS_INVERTING) {cur->dir&=~16;}
#endif
#if Y_MAX_PIN>-1
    if((cur->dir & 2)) if(READ(Y_MAX_PIN) != ENDSTOPS_INVERTING) {cur->dir&=~32;}
#endif
   }
   // Test Z-Axis every step if necessary, otherwise it could easyly ruin your printer!
   if(cur->dir & 64) { // normally no move on x axis, so we can skip the test
#if Z_MIN_PIN>-1
    if(!(cur->dir & 4)) if(READ(Z_MIN_PIN) != ENDSTOPS_INVERTING) {cur->dir&=~64;}
#endif
#if Z_MAX_PIN>-1
    if((cur->dir & 4)) if(READ(Z_MAX_PIN)!= ENDSTOPS_INVERTING) {cur->dir&=~64;}
#endif
   }
  }
  cli();
  if(cur->dir & 16) {
    if((cur->error[0] -= cur->delta[0]) < 0) {
      WRITE(X_STEP_PIN,HIGH);
      cur->error[0] += cur_errupd;
    }
  }
  if(cur->dir & 32) {
    if((cur->error[1] -= cur->delta[1]) < 0) {
      WRITE(Y_STEP_PIN,HIGH);
      cur->error[1] += cur_errupd;
    }
  }
  if(cur->dir & 64) {
    if((cur->error[2] -= cur->delta[2]) < 0) {
      WRITE(Z_STEP_PIN,HIGH);
      cur->error[2] += cur_errupd;
    }
  }
  if(cur->dir & 128) {
    if((cur->error[3] -= cur->delta[3]) < 0) {
      extruder_set_direction(cur->dir & 8);
      extruder_step();
      cur->error[3] += cur_errupd;
    }
  }    
  if(do_even) {
  sei(); // Allow interrupts for other types, timer1 is still disabled
#ifdef RAMP_ACCELERATION
  //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
  if(cur->flags & FLAG_ACCELERATION_ENABLED) {
    if (printer_state.stepNumber == 0) { // first step in line
        cur->flags |= FLAG_ACCELERATING;
    } else if (printer_state.stepNumber <= cur->accelSteps && (cur->flags & FLAG_ACCELERATING)!=0) { // we are accelerating
      printer_state.n += 4;
      if(printer_state.n==5) // Special case 0 => 1 as the error is too high in this case
        printer_state.interval = Div4U2U(printer_state.interval*1000,406); // Reduce error in step 0
      else
        printer_state.interval -= Div4U2U(printer_state.interval<<1,printer_state.n);
      if (printer_state.interval < cur->fullInterval) { // target speed reached, stop accelerating
        cur->flags &= ~FLAG_ACCELERATING;
        printer_state.interval = cur->fullInterval;
        printer_state.n = (cur->plateauN<<2)+1;
      }
#ifdef USE_ADVANCE
      printer_state.advance_target+=cur->advanceRate;
      if(printer_state.advance_target>cur->advanceFull)
        printer_state.advance_target = cur->advanceFull;
#endif
    } else if (cur->stepsRemaining <= cur->decelSteps && (cur->flags & FLAG_SKIP_DEACCELERATING)==0) { // time to slow down
        if ((cur->flags & FLAG_DECELERATING)==0) {
           cur->flags |= FLAG_DECELERATING;
           printer_state.n = printer_state.n-2; // includes the 4 subtracted later 
        }
        if(printer_state.n>4)				
          printer_state.n -= 4;        
        printer_state.interval += Div4U2U(printer_state.interval<<1,printer_state.n);
        if (printer_state.interval > cur->endInterval) {// we don't want to slow down under this value
	  printer_state.interval = cur->endInterval;
        }
#ifdef USE_ADVANCE
      printer_state.advance_target-=cur->advanceRate;
      if(printer_state.advance_target<cur->advanceEnd)
        printer_state.advance_target = cur->advanceEnd;
#endif
    } else {
      printer_state.interval = cur->fullInterval;
      printer_state.n = (cur->plateauN<<2)+1;
      cur->flags &= ~FLAG_ACCELERATING;
    }
  } else { // No acceleration 
      //Else, we are just use the full speed interval as current interval
      printer_state.interval = cur->fullInterval;
  }
#else
  printer_state.interval = cur->fullInterval; // without RAMPS always use full speed
#endif
  printer_state.stepNumber++;
  cur->stepsRemaining--;
  }
  cli();
  WRITE(X_STEP_PIN,LOW);
  WRITE(Y_STEP_PIN,LOW);
  WRITE(Z_STEP_PIN,LOW);
  extruder_unstep();
  long interval;
  if(cur->halfstep) interval = (printer_state.interval>>1); // time to come back
  interval = (printer_state.interval);  
  if(do_even) {
   if(cur->stepsRemaining<=0 || (cur->dir & 240)==0) { // line finished
    lines_pos++;
    if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
    cur = 0;
    if(!--lines_count) {
      if(DISABLE_X) disable_x();
      if(DISABLE_Y) disable_y();
      if(DISABLE_Z) disable_z();
      if(DISABLE_E) extruder_disable();  
    }
   }  
#ifdef DEBUG_FREE_MEMORY
    check_mem();
#endif  
  }
  return interval;
}



/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void kill(byte only_steppers)
{
  disable_x();
  disable_y();
  disable_z();
  extruder_disable();
  if(!only_steppers) {  
    extruder_set_temperature(0);
    heated_bed_set_temperature(0);
    if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);  
  }
}
/** \brief Sets the timer 1 compare value to delay ticks.

This function sets the OCR1A compare counter and the prescaler TCCR1B to get the next interrupt
at delay ticks measured from the last interrupt. 
*/
inline void setTimer(unsigned long delay)
{
  if (delay <= 65535L) {
    if(delay<500) delay=500;
    TCCR1B =  (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A = (unsigned int)delay;
  } else if (delay <= 524280L) {
    TCCR1B =  (_BV(WGM12) || _BV(CS11)); // prescale of /8 == 0.5 usec tick | 010 = clk/8
    OCR1A = (unsigned int)(delay >> 2);
  } else if (delay <= 4194240L) {
    TCCR1B =  (_BV(WGM12) | _BV(CS11) | _BV(CS10)); // prescale of /64 == 4 usec tick | 011 = clk/64
    OCR1A = (unsigned int)(delay >> 6);
  } else if (delay <= 16776960L) {
    TCCR1B =  (_BV(WGM12) | _BV(CS12)); // prescale of /256 == 16 usec tick | 100 = clk/256
    OCR1A = (unsigned int)(delay >> 8);
  } else if (delay <= 67107840L) {	 
    TCCR1B =  (_BV(WGM12) | _BV(CS12) | _BV(CS10)); // prescale of /1024 == 64 usec tick | 101 = clk/1024
    OCR1A = (unsigned int)(delay >> 10);
  } else {
    TCCR1B =  (_BV(WGM12) | _BV(CS12) | _BV(CS10)); // prescale of /1024 == 64 usec tick | 101 = clk/1024
    OCR1A = 65535;
  }
}

/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= ~(1<<OCIE1A);
  unsigned long ticks = 0; 
  if(lines_count)
    ticks = bresenham_step();
#ifdef DEBUG_FREE_MEMORY
  check_mem();
#endif
  if(ticks) setTimer((unsigned long)ticks);
  TIMSK1 |= (1<<OCIE1A); // Enable interrupt
}

/**
 \brief Initializes the timer interrupt for the steppers.
*/
void setupTimerInterrupt() {
  TCCR1A = 0;
  TCCR1C = 0;
  TIMSK1 = 0;  
  setTimer(SECONDS_TO_TICKS(0.01)); //start off with a slow frequency.
  TIMSK1 |= (1<<OCIE1A); // Enable interrupt
}


