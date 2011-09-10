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
*/
#include "Configuration.h"
#include "Reptier.h"
#include "Eeprom.h"
#include "pins_arduino.h"

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
volatile byte *step_port[3];
volatile byte *x_min_port,*y_min_port,*z_min_port;
byte x_min_pin,y_min_pin,z_min_pin;
#ifdef RAMP_ACCELERATION
  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
  long max_acceleration_units_per_sq_second[4] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
  long max_travel_acceleration_units_per_sq_second[4] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
  /** interval between two steps at the beginning of a ramp in Ticks */
  unsigned long axis_max_interval[4];
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
byte lines_write_pos=0;           ///< Position where we write the next cached line move.
byte lines_pos=0;                 ///< Position for executing line movement.
volatile byte lines_count=0;      ///< Number of lines cached 0 = nothing to do.
long baudrate = BAUDRATE;         ///< Communication speed rate.

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
    inv_axis_steps_per_unit[i] = 1.0f/axis_steps_per_unit[i];
#ifdef RAMP_ACCELERATION
    /** interval between two steps at the beginning of a ramp in Ticks */
    axis_max_interval[i] = F_CPU / (max_start_speed_units_per_second[i] * axis_steps_per_unit[i]);
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
  for(byte i=0; i < 3; i++) {
    pinMode(STEP_PIN[i],OUTPUT);
    step_port[i] = portOutputRegister(digitalPinToPort(STEP_PIN[i]));
    STEP_PIN[i] = digitalPinToBitMask(STEP_PIN[i]);
  }
  
  //Initialize Dir Pins
  if(X_DIR_PIN > -1) pinMode(X_DIR_PIN,OUTPUT);
  if(Y_DIR_PIN > -1) pinMode(Y_DIR_PIN,OUTPUT);
  if(Z_DIR_PIN > -1) pinMode(Z_DIR_PIN,OUTPUT);

  //Steppers default to disabled.
  if(X_ENABLE_PIN > -1) if(!X_ENABLE_ON) digitalWrite(X_ENABLE_PIN,HIGH);
  if(Y_ENABLE_PIN > -1) if(!Y_ENABLE_ON) digitalWrite(Y_ENABLE_PIN,HIGH);
  if(Z_ENABLE_PIN > -1) if(!Z_ENABLE_ON) digitalWrite(Z_ENABLE_PIN,HIGH);
  if(E_ENABLE_PIN > -1) if(!E_ENABLE_ON) digitalWrite(E_ENABLE_PIN,HIGH);

  //endstop pullups
  #ifdef ENDSTOPPULLUPS
    if(X_MIN_PIN > -1) { pinMode(X_MIN_PIN,INPUT); digitalWrite(X_MIN_PIN,HIGH);x_min_port = portInputRegister(digitalPinToPort(X_MIN_PIN));x_min_pin=digitalPinToBitMask(X_MIN_PIN);}
    if(Y_MIN_PIN > -1) { pinMode(Y_MIN_PIN,INPUT); digitalWrite(Y_MIN_PIN,HIGH);y_min_port = portInputRegister(digitalPinToPort(Y_MIN_PIN));y_min_pin=digitalPinToBitMask(Y_MIN_PIN);}
    if(Z_MIN_PIN > -1) { pinMode(Z_MIN_PIN,INPUT); digitalWrite(Z_MIN_PIN,HIGH);z_min_port = portInputRegister(digitalPinToPort(Z_MIN_PIN));z_min_pin=digitalPinToBitMask(Z_MIN_PIN);}
    if(X_MAX_PIN > -1) { pinMode(X_MAX_PIN,INPUT); digitalWrite(X_MAX_PIN,HIGH);}
    if(Y_MAX_PIN > -1) { pinMode(Y_MAX_PIN,INPUT); digitalWrite(Y_MAX_PIN,HIGH);}
    if(Z_MAX_PIN > -1) { pinMode(Z_MAX_PIN,INPUT); digitalWrite(Z_MAX_PIN,HIGH);}
  #endif
  //Initialize Enable Pins
  if(X_ENABLE_PIN > -1) pinMode(X_ENABLE_PIN,OUTPUT);
  if(Y_ENABLE_PIN > -1) pinMode(Y_ENABLE_PIN,OUTPUT);
  if(Z_ENABLE_PIN > -1) pinMode(Z_ENABLE_PIN,OUTPUT);
  printer_state.feedrate = 1500; ///< Current feedrate in mm/min.
  
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
  //log_long_array(PSTR("StepsRem"),p->axisStepsRemaining);
  log_long_array(PSTR("Delta"),p->delta);
  log_long_array(PSTR("Error"),p->error);
  out.println_int_P(PSTR("Prim:"),p->primaryAxis);
  out.println_int_P(PSTR("Dir:"),p->dir);
  out.println_int_P(PSTR("Flags:"),p->flags);
  //log_value(PSTR("vMin"),p->vMin);
  //log_value(PSTR("vMax"),p->vMax);
  //log_value(PSTR("acceleration"),p->acceleration);
  //log_value(PSTR("iv"),p->interval);
  out.println_long_P(PSTR("Acceleration:"),p->acceleration);
  out.println_long_P(PSTR("Start interval:"),p->startInterval);
  out.println_long_P(PSTR("Full interval:"),p->fullInterval);
  out.println_long_P(PSTR("startN:"),p->startN);
  out.println_long_P(PSTR("plateauN:"),p->plateauN);
  out.println_long_P(PSTR("Remaining steps:"),p->stepsRemaining);
  out.println_long_P(PSTR("Accel steps:"),p->accelSteps);
  out.println_long_P(PSTR("Decel steps:"),p->decelSteps);
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

/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
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
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS | FLAG_FIRST_STEP; 
  else p->flags = FLAG_FIRST_STEP;
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
  float d;
  if(p->dir & 112) {
    if(p->dir & 64) {
      d = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1] + axis_diff[2] * axis_diff[2]);
    } else {
      d = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
    }
  }  else if(p->dir & 128)
    d = abs(axis_diff[3]);
  else {
    return; // no steps to take, we are finished
  }  
  float time_for_move = (float)(60*F_CPU)*d / printer_state.feedrate; // time is in ticks
  // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
  long limitInterval = time_for_move/p->stepsRemaining; // until not violated by othe constraints it is your target speed
  axis_interval[0] = abs(axis_diff[0])*F_CPU/(max_feedrate[0]*p->stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
  if(axis_interval[0]>limitInterval) limitInterval = axis_interval[0];
  axis_interval[1] = abs(axis_diff[1])*F_CPU/(max_feedrate[1]*p->stepsRemaining);
  if(axis_interval[1]>limitInterval) limitInterval = axis_interval[1];
  if(p->dir & 64) { // normally no move in z direction
    axis_interval[2] = abs(axis_diff[2])*F_CPU/(max_feedrate[2]*p->stepsRemaining);
    if(axis_interval[2]>limitInterval) limitInterval = axis_interval[2];
  } else axis_interval[2] = 0;
  axis_interval[3] = abs(axis_diff[3])*F_CPU/(max_feedrate[3]*p->stepsRemaining);
  if(axis_interval[3]>limitInterval) limitInterval = axis_interval[3];
  p->fullInterval = limitInterval; // This is our target speed
  // new time at full speed = limitInterval*p->stepsRemaining [ticks]
  time_for_move = (float)limitInterval*(float)p->stepsRemaining; // for large z-distance this overflows with long computation
  if(p->dir & 16)
    axis_interval[0] = time_for_move/p->delta[0];
  if(p->dir & 32)
    axis_interval[1] = time_for_move/p->delta[1];
  if(p->dir & 64)
    axis_interval[2] = time_for_move/p->delta[2];
  if(p->dir & 128)
    axis_interval[3] = time_for_move/p->delta[3];
#ifdef DEBUG_QUEUE_MOVE
  if(DEBUG_ECHO) {
    out.println_long_P(PSTR("limitInterval:"), limitInterval);
    out.println_float_P(PSTR("Move distance on the XYZ space:"), d);
    out.println_float_P(PSTR("Commanded feedrate:"), printer_state.feedrate);
    out.println_float_P(PSTR("Constant full speed move time:"), time_for_move);
  }
#endif
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  // TODO: maybe it's better to refactor into a generic enable(int axis) function, that will probably take more ram,
  // but will reduce code size
  if(p->dir & 16) enable_x();
  if(p->dir & 32) enable_y();
  if(p->dir & 64) enable_z();

  long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
  byte is_print_move = p->delta[3] > 0; // are we printing

  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
  #ifdef RAMP_ACCELERATION
    p->startInterval = axis_max_interval[primary_axis]; // interval in ticks/step at startspeed feedrate
    p->endInterval = p->startInterval;
    
    unsigned long new_axis_max_intervals[4];
  
    //Calculate start speeds based on moving axes max start speed constraints.
    // which axis takes the longest time, if we would have no acceleration?
    byte slowest_start_axis = primary_axis;
    unsigned long slowest_start_axis_max_interval = p->startInterval; // axis interval of limiting axis
    for(byte i = 0; i < 4; i++) {
      p->error[i] = p->delta[primary_axis] >> 1;
      if (i != primary_axis && (p->dir & (16<<i)) &&  
            axis_max_interval[i] * p->delta[i] > p->delta[slowest_start_axis]*slowest_start_axis_max_interval) {
        slowest_start_axis = i;
        slowest_start_axis_max_interval = axis_max_interval[i];
      }
    }
    // Update start speeds based on slowest start axis  
    for(byte i = 0; i < 4; i++) {
      if(p->dir & (16<<i)) {
        // multiplying slowest_start_axis_max_interval by axis_steps_remaining[slowest_start_axis]
        // could lead to overflows when we have long distance moves (say, 390625*390625 > sizeof(unsigned long))
        float steps_remaining_ratio = p->delta[slowest_start_axis] / p->delta[i];
        new_axis_max_intervals[i] = slowest_start_axis_max_interval * steps_remaining_ratio; // start interval for axis x
        if(i == primary_axis) {
          p->startInterval = new_axis_max_intervals[i];
        }
      }
    }
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowest_axis_plateau_time_repro = 1e20; // repro to reduce division Unit: 1/s
    float rdeltav[4]; // f/delta_v [s/step]
    for(byte i=0; i < 4 ; i++) {
      if(p->dir & (16<<i)) {
        rdeltav[i] = (float)new_axis_max_intervals[i]*(float)axis_interval[i]/(float)(new_axis_max_intervals[i]-axis_interval[i]);
        slowest_axis_plateau_time_repro = min(slowest_axis_plateau_time_repro,
               rdeltav[i] * (float)(is_print_move ?  axis_steps_per_sqr_second[i] : axis_travel_steps_per_sqr_second[i]));
      }
    }
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    p->acceleration = slowest_axis_plateau_time_repro / rdeltav[primary_axis]; // Steps/s^2
    long vMax = F_CPU / interval; // maximum steps per second, we can reach
    long vMin = F_CPU / p->startInterval; //starting speed
    if(vMax>46000) { // gets overflow in N computation
      vMax = 46000;
      if(vMin>46000) vMin = 46000; // this means 348 Ticks/Steps on 16MHz, which is too fast for an arduino
    }
    p->startN = vMin*vMin/p->acceleration; // should be divided by 2, so we have in reality 2*n
    p->plateauN = vMax*vMax/p->acceleration;
    p->accelSteps = (p->plateauN-p->startN)/2;
#ifdef PATHDEPENDEND_RAMP
    if(p->accelSteps>=(p->stepsRemaining>>1)) {// Limit acceleration period to half move if longer 
      p->accelSteps = p->stepsRemaining>>1;
    }
    p->decelSteps = p->accelSteps;      
    if(lines_count>2) { // Only compute if we have time until this line is printed
      /*
        There are to targets, we have to consider
        1. Completely turn off deceleration/acceleration
        2. Reduce deceleration/acceleration to acceptable amounts
        
        For both methods, we need a value, indicating the forces that we have to handle.
        The main factor is the change of direction. For ease of computation, we only do
        this if a) we have only a movement in x/y direction and b) the feedrate is the same.
      */
      char prev_pos = lines_write_pos-1;
      if(prev_pos<0) prev_pos = MOVE_CACHE_SIZE-1;
      PrintLine *prev = &lines[prev_pos];
      if((prev->dir & 64) || (p->dir & 64) || (p->fullInterval !=prev->fullInterval)) {
         if(p->accelSteps>=(p->stepsRemaining>>1)) {// Limit acceleration period to half move if longer 
           p->accelSteps = p->stepsRemaining>>1;
         }
         p->decelSteps = p->accelSteps;  // to complex, go for safety
      } else {
        prev->flags |= FLAG_BLOCKED; // prevent from execution, just for savety
        long n_min = p->stepsRemaining<prev->stepsRemaining ? p->stepsRemaining : prev->stepsRemaining;
        float inv_d = 1f/d;
        float dx1 = axis_diff[0]*inv_d;
        float dy1 = axis_diff[1]*inv_d;
        float dx2 = prev->delta[0]*inv_axis_steps_per_unit[0];
        float dy2 = prev->delta[1]*inv_axis_steps_per_unit[1];
        float l2 = 1f/sqrt(dx2*dx2+dy2*dy2);
        dx2 *= l2;dy2 *= l2;
        float cos_alpha = dx1*dx2+dy2*dy2; // cos(alpha) 
        long vc;
        if(cos_alpha>0 || lines_count>2) {
           vc = vMin+(vMax-vMin)*cos_alpha;
           if(p->accelSteps>=(p->stepsRemaining>>1)) { // we can not reach end speed, watch out to high speeds!
             long vc_max = (long)sqrt(2*p->acceleration*n_min+vMin*vMin);
             if(vc>vc_max) vc = vc_max;
           }
        } else {vc = vMin;}
        long slowDownN = vc*vc/prev->acceleration;
        prev->decelSteps = (prev->
        pev->flags &= ~FLAG_BLOCKED;
      }
    } else { // no time for additional computation
      if(p->accelSteps>=(p->stepsRemaining>>1)) {// Limit acceleration period to half move if longer 
        p->accelSteps = p->stepsRemaining>>1;
      }
      p->decelSteps = p->accelSteps;      
    }
#else
    if(p->accelSteps>=(p->stepsRemaining>>1-5)) {// Limit acceleration period to half move if longer 
      p->accelSteps = p->stepsRemaining>>1-5;
    }
    p->decelSteps = p->accelSteps;      
#endif
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
#ifdef DEBUG_QUEUE_MOVE
    if(DEBUG_ECHO) {
      log_long_array(PSTR("axis_int"),(long*)axis_interval);
      log_long_array(PSTR("axis_max_int"),(long*)axis_max_interval);
      log_long_array(PSTR("new_axis_max_int"),(long*)new_axis_max_intervals);
      out.println_float_P(PSTR("Plateau repro:"),slowest_axis_plateau_time_repro);
    }
#endif
  #endif
  
  #ifdef RAMP_ACCELERATION
  p->flags |= FLAG_ACCELERATION_ENABLED;
  p->fullInterval = interval; // interval at full speed
  if(interval > p->startInterval || p->accelSteps<0) p->flags &= ~FLAG_ACCELERATION_ENABLED; // can start with target speedrate
  #endif
  // Correct integers for fixed point math used in bresenham_step
  p->startN<<=7;
  p->plateauN<<=7;
  p->startInterval<<=8;
  p->endInterval<<=8;
  p->fullInterval<<=8;
#ifdef DEBUG_QUEUE_MOVE
  if(DEBUG_ECHO) {
    log_printLine(p);  
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

/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
*/
long bresenham_step() {
  if(!lines_count) return 0; // nothing to to
  PrintLine *p = &lines[lines_pos];
#ifdef PATHDEPENDEND_RAMP
  if(p->flags & FLAG_BLOCKED) { // This step is in computation - shouldn't happen
    return 1000;
  }
#endif
#ifdef INCLUDE_DEBUG_NO_MOVE
  if(DEBUG_NO_MOVES) {
    p->dir = 0; // No steps in any direction, switches only the motors on.
  }
#endif
  long errupd = p->delta[p->primaryAxis];
  //If there are x or y steps remaining, perform Bresenham algorithm
  if(p->flags & FLAG_FIRST_STEP) {
    printer_state.stepNumber=0;
    printer_state.n = p->startN*4+256;
    //Determine direction of movement,check if endstop was hit
    if(p->dir & 1) {
      digitalWriteFast2(X_DIR_PIN,!INVERT_X_DIR);
    } else {
      digitalWriteFast2(X_DIR_PIN,INVERT_X_DIR);
    }
    if(p->dir & 2) {
      digitalWriteFast2(Y_DIR_PIN,!INVERT_Y_DIR);
    } else {
      digitalWriteFast2(Y_DIR_PIN,INVERT_Y_DIR);
    }
    if(p->dir & 4) {
      digitalWriteFast2(Z_DIR_PIN,!INVERT_Z_DIR);
    } else {
      digitalWriteFast2(Z_DIR_PIN,INVERT_Z_DIR);
    }
    extruder_set_direction((p->dir & 8) ? 1 : 0);
    p->flags &= ~FLAG_FIRST_STEP;
  }
  if(p->flags & FLAG_CHECK_ENDSTOPS) {
    if(X_MIN_PIN > -1 && !(p->dir & 1)) if((*x_min_port & x_min_pin ? HIGH : LOW) != ENDSTOPS_INVERTING) {p->dir&=~16;}
    if(Y_MIN_PIN > -1 && !(p->dir & 2)) if((*y_min_port & y_min_pin ? HIGH : LOW) != ENDSTOPS_INVERTING) {p->dir&=~32;}
    if(X_MAX_PIN > -1 && (p->dir & 1)) if(digitalReadFast2(X_MAX_PIN) != ENDSTOPS_INVERTING) {p->dir&=~16;}
    if(Y_MAX_PIN > -1 && (p->dir & 2)) if(digitalReadFast2(Y_MAX_PIN) != ENDSTOPS_INVERTING) {p->dir&=~32;}
  }
  // Test Z-Axis every step if necessary, otherwise it could easyly ruin your printer!
  if(p->dir & 64) { // normally no move on x axis, so we can skip the test
    if(Z_MIN_PIN > -1 && !(p->dir & 4)) if((*z_min_port & z_min_pin ? HIGH : LOW) != ENDSTOPS_INVERTING) {p->dir&=~64;}
    if(Z_MAX_PIN > -1 && (p->dir & 4)) if(digitalReadFast2(Z_MAX_PIN) != ENDSTOPS_INVERTING) {p->dir&=~64;}
  }
  if(p->dir & 16) {
    if((p->error[0] -= p->delta[0]) < 0) {
      *step_port[0] |= STEP_PIN[0]; // much faster then digitalWrite !
      p->error[0] += errupd;
    }
  }
  if(p->dir & 32) {
    if((p->error[1] -= p->delta[1]) < 0) {
      *step_port[1] |= STEP_PIN[1];
      p->error[1] += errupd;
    }
  }
  if(p->dir & 64) {
    if((p->error[2] -= p->delta[2]) < 0) {
      *step_port[2] |= STEP_PIN[2];
      p->error[2] += errupd;
    }
  }
  if(p->dir & 128) {
    if((p->error[3] -= p->delta[3]) < 0) {
      extruder_step();
      p->error[3] += errupd;
    }
  }    
  sei(); // Allow interrupts for other types, timer1 is still disabled
#ifdef RAMP_ACCELERATION
  //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
  if(p->flags & FLAG_ACCELERATION_ENABLED) {
    if (printer_state.stepNumber == 0) { // first step in line
        printer_state.interval = p->startInterval; // maximum allowed start speed
        p->flags |= FLAG_ACCELERATING;
    } else if (printer_state.stepNumber <= p->accelSteps && (p->flags & FLAG_ACCELERATING)!=0) { // we are accelerating
        if(printer_state.interval>=4194304) {
          long h = (2*printer_state.interval/printer_state.n);
          printer_state.interval -= h*256;  // prevent overflow for slow movements
        } else {
          printer_state.interval -= 512*printer_state.interval/printer_state.n;
        }
      printer_state.n += 1024;
      if (printer_state.interval < p->fullInterval) { // target speed reached, stop accelerating
        p->flags &= ~FLAG_ACCELERATING;
        printer_state.interval = p->fullInterval;
        printer_state.n = p->plateauN*4+256;
      }
    } else if (p->stepsRemaining <= p->decelSteps && (p->flags & FLAG_SKIP_DEACCELERATING)==0) { // time to slow down
        if ((p->flags & FLAG_DECELERATING)==0) {
           p->flags |= FLAG_DECELERATING;
           printer_state.n = -printer_state.n+512; // includes the 1024 added later 
        }				
        if(printer_state.interval>=4194304) {
          long h = (2*printer_state.interval/printer_state.n);
          printer_state.interval -= h*256;  // prevent overflow for slow movements
        } else {
          printer_state.interval -= 512*printer_state.interval/printer_state.n;
        }
        printer_state.n += 1024;
        if (printer_state.interval > p->endInterval) {// we don't want to slow down under this value
	  printer_state.interval = p->endInterval;
        }
    } else {
      printer_state.interval = p->fullInterval;
      p->flags &= ~FLAG_ACCELERATING;
    }
  } else { // No acceleration 
      //Else, we are just use the full speed interval as current interval
      printer_state.interval = p->fullInterval;
  }
#else
  printer_state.interval = p->fullInterval; // without RAMPS always use full speed
#endif
  printer_state.stepNumber++;
  p->stepsRemaining--;

  cli();
  *step_port[0] &= ~STEP_PIN[0]; // Set signal pins low
  *step_port[1] &= ~STEP_PIN[1];
  *step_port[2] &= ~STEP_PIN[2];
  extruder_unstep();
  if(p->stepsRemaining<=0 || (p->dir & 240)==0) { // line finished
    lines_pos++;
    if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
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
  return (printer_state.interval>>8); // time to come back
}
/** \brief Disable stepper motor for x direction. */
void disable_x() { if(X_ENABLE_PIN > -1) digitalWriteFast2(X_ENABLE_PIN,!X_ENABLE_ON); }
/** \brief Disable stepper motor for y direction. */
void disable_y() { if(Y_ENABLE_PIN > -1) digitalWriteFast2(Y_ENABLE_PIN,!Y_ENABLE_ON); }
/** \brief Disable stepper motor for z direction. */
void disable_z() { if(Z_ENABLE_PIN > -1) digitalWriteFast2(Z_ENABLE_PIN,!Z_ENABLE_ON); }
/** \brief Enable stepper motor for x direction. */
inline void  enable_x() { if(X_ENABLE_PIN > -1) digitalWriteFast2(X_ENABLE_PIN, X_ENABLE_ON); }
/** \brief Enable stepper motor for y direction. */
inline void  enable_y() { if(Y_ENABLE_PIN > -1) digitalWriteFast2(Y_ENABLE_PIN, Y_ENABLE_ON); }
/** \brief Enable stepper motor for z direction. */
inline void  enable_z() { if(Z_ENABLE_PIN > -1) digitalWriteFast2(Z_ENABLE_PIN, Z_ENABLE_ON); }



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

/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= ~(1<<OCIE1A);
  long ticks = 0; 
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

// Note - it is up to the user to call enableTimerInterrupt() after a call
// to this function.
/** \brief Sets the timer 1 compare value to delay ticks.

This function sets the OCR1A compare counter and the prescaler TCCR1B to get the next interrupt
at delay ticks measured from the last interrupt. 
*/
void setTimer(unsigned long delay)
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

