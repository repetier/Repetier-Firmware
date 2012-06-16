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
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Configuration.h"
#include "Reptier.h"
#include "Eeprom.h"

#if EEPROM_MODE!=0
extern void epr_eeprom_to_data();
inline byte epr_get_byte(uint pos) {
   return eeprom_read_byte ((unsigned char *)(EEPROM_OFFSET+pos));
}
inline int epr_get_int(uint pos) {
  return eeprom_read_word((unsigned int *)(EEPROM_OFFSET+pos));
}
inline long epr_get_long(uint pos) {
  return eeprom_read_dword((unsigned long*)(EEPROM_OFFSET+pos));
}
inline float epr_get_float(uint pos) {
  float v;
  eeprom_read_block(&v,(void *)(EEPROM_OFFSET+pos),4); // newer gcc have eeprom_read_block but not arduino 22
  return v;
}

inline void epr_set_byte(uint pos,byte value) {
  eeprom_write_byte((unsigned char *)(EEPROM_OFFSET+pos), value);
}
inline void epr_set_int(uint pos,int value) {
  eeprom_write_word((unsigned int*)(EEPROM_OFFSET+pos),value);
}
inline void epr_set_long(uint pos,long value) {
  eeprom_write_dword((unsigned long*)(EEPROM_OFFSET+pos),value);
}
inline void epr_set_float(uint pos,float value) {
  eeprom_write_block(&value,(void*)(EEPROM_OFFSET+pos), 4);
}
void epr_out_float(uint pos,PGM_P text) {
  out.print_P(PSTR("EPR:3 "));
  out.print(pos);
  out.print(' ');
  out.print(epr_get_float(pos));
  out.print(' ');
  out.println_P(text);  
}
void epr_out_long(uint pos,PGM_P text) {
  out.print_P(PSTR("EPR:2 "));
  out.print(pos);
  out.print(' ');
  out.print(epr_get_long(pos));
  out.print(' ');
  out.println_P(text);  
}
void epr_out_int(uint pos,PGM_P text) {
  out.print_P(PSTR("EPR:1 "));
  out.print(pos);
  out.print(' ');
  out.print(epr_get_int(pos));
  out.print(' ');
  out.println_P(text);  
}
void epr_out_byte(uint pos,PGM_P text) {
  out.print_P(PSTR("EPR:0 "));
  out.print(pos);
  out.print(' ');
  out.print((int)epr_get_byte(pos));
  out.print(' ');
  out.println_P(text);  
}

void epr_update(GCode *com) {
  if(GCODE_HAS_T(com) && GCODE_HAS_P(com)) switch(com->T) {
    case 0:
      if(GCODE_HAS_S(com)) epr_set_byte(com->P,(byte)com->S);
      break;
    case 1:
      if(GCODE_HAS_S(com)) epr_set_int(com->P,(int)com->S);
      break;
    case 2:
      if(GCODE_HAS_S(com)) epr_set_long(com->P,(long)com->S);
      break;
    case 3:
      if(GCODE_HAS_X(com)) epr_set_float(com->P,com->X);
      break;
  }
  epr_eeprom_to_data();
}

/** \brief Moves current settings to EEPROM.

The values the are currently set are used to fill the eeprom.*/
void epr_data_to_eeprom() {
  epr_set_long(EPR_BAUDRATE,baudrate);
  epr_set_long(EPR_MAX_INACTIVE_TIME,max_inactive_time);
  epr_set_long(EPR_STEPPER_INACTIVE_TIME,stepper_inactive_time);
//#define EPR_ACCELERATION_TYPE 1
  epr_set_float(EPR_XAXIS_STEPS_PER_MM,axis_steps_per_unit[0]);
  epr_set_float(EPR_YAXIS_STEPS_PER_MM,axis_steps_per_unit[1]);
  epr_set_float(EPR_ZAXIS_STEPS_PER_MM,axis_steps_per_unit[2]);
  epr_set_float(EPR_X_MAX_FEEDRATE,max_feedrate[0]);
  epr_set_float(EPR_Y_MAX_FEEDRATE,max_feedrate[1]);
  epr_set_float(EPR_Z_MAX_FEEDRATE,max_feedrate[2]);
  epr_set_float(EPR_X_HOMING_FEEDRATE,homing_feedrate[0]);
  epr_set_float(EPR_Y_HOMING_FEEDRATE,homing_feedrate[1]);
  epr_set_float(EPR_Z_HOMING_FEEDRATE,homing_feedrate[2]);
  epr_set_float(EPR_MAX_JERK,printer_state.maxJerk);
  epr_set_float(EPR_MAX_ZJERK,printer_state.maxZJerk);
#ifdef RAMP_ACCELERATION
  epr_set_float(EPR_X_MAX_ACCEL,max_acceleration_units_per_sq_second[0]);
  epr_set_float(EPR_Y_MAX_ACCEL,max_acceleration_units_per_sq_second[1]);
  epr_set_float(EPR_Z_MAX_ACCEL,max_acceleration_units_per_sq_second[2]);
  epr_set_float(EPR_X_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[0]);
  epr_set_float(EPR_Y_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[1]);
  epr_set_float(EPR_Z_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[2]);
#endif
#if USE_OPS==1
  epr_set_float(EPR_OPS_MIN_DISTANCE,printer_state.opsMinDistance);
  epr_set_byte(EPR_OPS_MODE,printer_state.opsMode);
  epr_set_float(EPR_OPS_MOVE_AFTER,printer_state.opsMoveAfter);
  epr_set_float(EPR_OPS_RETRACT_DISTANCE,printer_state.opsRetractDistance);
  epr_set_float(EPR_OPS_RETRACT_BACKSLASH,printer_state.opsRetractBackslash);
#endif

  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    epr_set_float(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
    epr_set_float(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
    epr_set_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
    epr_set_float(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
    epr_set_byte(o+EPR_EXTRUDER_HEAT_MANAGER,e->heatManager);
#ifdef TEMP_PID
    epr_set_byte(o+EPR_EXTRUDER_DRIVE_MAX,e->pidDriveMax);
    epr_set_byte(o+EPR_EXTRUDER_DRIVE_MIN,e->pidDriveMin);
    epr_set_long(o+EPR_EXTRUDER_PID_PGAIN,e->pidPGain);
    epr_set_long(o+EPR_EXTRUDER_PID_IGAIN,e->pidIGain);
    epr_set_long(o+EPR_EXTRUDER_PID_DGAIN,e->pidDGain);
    epr_set_byte(o+EPR_EXTRUDER_PID_MAX,e->pidMax);
#endif
    epr_set_long(o+EPR_EXTRUDER_X_OFFSET,e->yOffset);
    epr_set_long(o+EPR_EXTRUDER_Y_OFFSET,e->xOffset);
    epr_set_int(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
#endif
    epr_set_float(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);
#endif
  }
}
/** \brief Copy data from EEPROM to variables.
*/
void epr_eeprom_to_data() {
  baudrate = epr_get_long(EPR_BAUDRATE);
  max_inactive_time = epr_get_long(EPR_MAX_INACTIVE_TIME);
  stepper_inactive_time = epr_get_long(EPR_STEPPER_INACTIVE_TIME);
//#define EPR_ACCELERATION_TYPE 1
  axis_steps_per_unit[0] = epr_get_float(EPR_XAXIS_STEPS_PER_MM);
  axis_steps_per_unit[1] = epr_get_float(EPR_YAXIS_STEPS_PER_MM);
  axis_steps_per_unit[2] = epr_get_float(EPR_ZAXIS_STEPS_PER_MM);
  max_feedrate[0] = epr_get_float(EPR_X_MAX_FEEDRATE);
  max_feedrate[1] = epr_get_float(EPR_Y_MAX_FEEDRATE);
  max_feedrate[2] = epr_get_float(EPR_Z_MAX_FEEDRATE);
  homing_feedrate[0] = epr_get_float(EPR_X_HOMING_FEEDRATE);
  homing_feedrate[1] = epr_get_float(EPR_Y_HOMING_FEEDRATE);
  homing_feedrate[2] = epr_get_float(EPR_Z_HOMING_FEEDRATE);
  printer_state.maxJerk = epr_get_float(EPR_MAX_JERK);
  printer_state.maxZJerk = epr_get_float(EPR_MAX_ZJERK);
#ifdef RAMP_ACCELERATION
  max_acceleration_units_per_sq_second[0] = epr_get_float(EPR_X_MAX_ACCEL);
  max_acceleration_units_per_sq_second[1] = epr_get_float(EPR_Y_MAX_ACCEL);
  max_acceleration_units_per_sq_second[2] = epr_get_float(EPR_Z_MAX_ACCEL);
  max_travel_acceleration_units_per_sq_second[0] = epr_get_float(EPR_X_MAX_TRAVEL_ACCEL);
  max_travel_acceleration_units_per_sq_second[1] = epr_get_float(EPR_Y_MAX_TRAVEL_ACCEL);
  max_travel_acceleration_units_per_sq_second[2] = epr_get_float(EPR_Z_MAX_TRAVEL_ACCEL);
#endif
#if USE_OPS==1
  printer_state.opsMode = epr_get_byte(EPR_OPS_MODE);
  printer_state.opsMoveAfter = epr_get_float(EPR_OPS_MOVE_AFTER);
  printer_state.opsMinDistance = epr_get_float(EPR_OPS_MIN_DISTANCE);
  printer_state.opsRetractDistance = epr_get_float(EPR_OPS_RETRACT_DISTANCE);
  printer_state.opsRetractBackslash = epr_get_float(EPR_OPS_RETRACT_BACKSLASH);
#endif
  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    e->stepsPerMM = epr_get_float(o+EPR_EXTRUDER_STEPS_PER_MM);
    e->maxFeedrate = epr_get_float(o+EPR_EXTRUDER_MAX_FEEDRATE);
    e->maxStartFeedrate = epr_get_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
    e->maxAcceleration = epr_get_float(o+EPR_EXTRUDER_MAX_ACCELERATION);
    e->heatManager = epr_get_byte(o+EPR_EXTRUDER_HEAT_MANAGER);
#ifdef TEMP_PID
    e->pidDriveMax = epr_get_byte(o+EPR_EXTRUDER_DRIVE_MAX);
    e->pidDriveMin = epr_get_byte(o+EPR_EXTRUDER_DRIVE_MIN);
    e->pidPGain = epr_get_long(o+EPR_EXTRUDER_PID_PGAIN);
    e->pidIGain = epr_get_long(o+EPR_EXTRUDER_PID_IGAIN);
    e->pidDGain = epr_get_long(o+EPR_EXTRUDER_PID_DGAIN);
    e->pidMax = epr_get_byte(o+EPR_EXTRUDER_PID_MAX);
#endif
    e->yOffset = epr_get_long(o+EPR_EXTRUDER_X_OFFSET);
    e->xOffset = epr_get_long(o+EPR_EXTRUDER_Y_OFFSET);
    e->watchPeriod = epr_get_int(o+EPR_EXTRUDER_WATCH_PERIOD);    
 #ifdef USE_ADVANCE
 #ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = epr_get_float(o+EPR_EXTRUDER_ADVANCE_K);
 #endif
    e->advanceL = epr_get_float(o+EPR_EXTRUDER_ADVANCE_L);
 #endif
  }
  extruder_select(current_extruder->id);
  update_ramps_parameter();
  
}
#endif
void epr_init_baudrate() {
#if EEPROM_MODE!=0
  if(epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE) {
    baudrate = epr_get_long(EPR_BAUDRATE);    
  }
#endif
}
void epr_init() {
#if EEPROM_MODE!=0
  if(epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE)
    epr_eeprom_to_data();
  else {
    epr_data_to_eeprom();
    epr_set_byte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make datachange permanent
  }
#endif
}

/** \brief Writes all eeprom settings to serial console.

For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void epr_output_settings() {
#if EEPROM_MODE!=0
  epr_out_long(EPR_BAUDRATE,PSTR("Baudrate"));
  epr_out_long(EPR_MAX_INACTIVE_TIME,PSTR("Max. inactive time [ms,0=off]"));
  epr_out_long(EPR_STEPPER_INACTIVE_TIME,PSTR("Stop stepper afer inactivity [ms,0=off]"));
//#define EPR_ACCELERATION_TYPE 1
  epr_out_float(EPR_XAXIS_STEPS_PER_MM,PSTR("X-axis steps per mm"));
  epr_out_float(EPR_YAXIS_STEPS_PER_MM,PSTR("Y-axis steps per mm"));
  epr_out_float(EPR_ZAXIS_STEPS_PER_MM,PSTR("Z-axis steps per mm"));
  epr_out_float(EPR_X_MAX_FEEDRATE,PSTR("X-axis max. feedrate [mm/min]"));
  epr_out_float(EPR_Y_MAX_FEEDRATE,PSTR("Y-axis max. feedrate [mm/min]"));
  epr_out_float(EPR_Z_MAX_FEEDRATE,PSTR("Z-axis max. feedrate [mm/min]"));
  epr_out_float(EPR_X_HOMING_FEEDRATE,PSTR("X-axis homing feedrate [mm/min]"));
  epr_out_float(EPR_Y_HOMING_FEEDRATE,PSTR("Y-axis homing feedrate [mm/min]"));
  epr_out_float(EPR_Z_HOMING_FEEDRATE,PSTR("Z-axis homing feedrate [mm/min]"));
  epr_out_float(EPR_MAX_JERK,PSTR("Max. jerk [mm/s]"));
  epr_out_float(EPR_MAX_ZJERK,PSTR("Max. Z-jerk [mm/s]"));
#ifdef RAMP_ACCELERATION
  //epr_out_float(EPR_X_MAX_START_SPEED,PSTR("X-axis start speed [mm/s]"));
  //epr_out_float(EPR_Y_MAX_START_SPEED,PSTR("Y-axis start speed [mm/s]"));
  //epr_out_float(EPR_Z_MAX_START_SPEED,PSTR("Z-axis start speed [mm/s]"));
  epr_out_float(EPR_X_MAX_ACCEL,PSTR("X-axis acceleration [mm/s^2]"));
  epr_out_float(EPR_Y_MAX_ACCEL,PSTR("Y-axis acceleration [mm/s^2]"));
  epr_out_float(EPR_Z_MAX_ACCEL,PSTR("Z-axis acceleration [mm/s^2]"));
  epr_out_float(EPR_X_MAX_TRAVEL_ACCEL,PSTR("X-axis travel acceleration [mm/s^2]"));
  epr_out_float(EPR_Y_MAX_TRAVEL_ACCEL,PSTR("Y-axis travel acceleration [mm/s^2]"));
  epr_out_float(EPR_Z_MAX_TRAVEL_ACCEL,PSTR("Z-axis travel acceleration [mm/s^2]"));
#endif
#if USE_OPS==1
  epr_out_byte(EPR_OPS_MODE,PSTR("OPS operation mode [0=Off,1=Classic,2=Fast]"));
  epr_out_float(EPR_OPS_MOVE_AFTER,PSTR("OPS move after x% retract [%]"));
  epr_out_float(EPR_OPS_MIN_DISTANCE,PSTR("OPS min. distance for fil. retraction [mm]"));
  epr_out_float(EPR_OPS_RETRACT_DISTANCE,PSTR("OPS retraction length [mm]"));
  epr_out_float(EPR_OPS_RETRACT_BACKSLASH,PSTR("OPS retraction backslash [mm]"));
#endif
  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    epr_out_float(o+EPR_EXTRUDER_STEPS_PER_MM,PSTR("Extr. steps per mm"));
    epr_out_float(o+EPR_EXTRUDER_MAX_FEEDRATE,PSTR("Extr. max. feedrate [mm/min]"));
    epr_out_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,PSTR("Extr. start feedrate [mm/s]"));
    epr_out_float(o+EPR_EXTRUDER_MAX_ACCELERATION,PSTR("Extr. acceleration [mm/s^2]"));
    epr_out_byte(o+EPR_EXTRUDER_HEAT_MANAGER,PSTR("Heat manager [0-1]"));
#ifdef TEMP_PID
    epr_out_byte(o+EPR_EXTRUDER_DRIVE_MAX,PSTR("PID drive max"));
    epr_out_byte(o+EPR_EXTRUDER_DRIVE_MIN,PSTR("PID drive min"));
    epr_out_long(o+EPR_EXTRUDER_PID_PGAIN,PSTR("PID P-gain [*0.01]"));
    epr_out_long(o+EPR_EXTRUDER_PID_IGAIN,PSTR("PID I-gain [*0.001]"));
    epr_out_long(o+EPR_EXTRUDER_PID_DGAIN,PSTR("PID D-gain [*0.01]"));
    epr_out_byte(o+EPR_EXTRUDER_PID_MAX,PSTR("PID max value [0-255]"));
#endif
    epr_out_long(o+EPR_EXTRUDER_X_OFFSET,PSTR("X-offset [steps]"));
    epr_out_long(o+EPR_EXTRUDER_Y_OFFSET,PSTR("Y-offset [steps]"));
    epr_out_int(o+EPR_EXTRUDER_WATCH_PERIOD,PSTR("Temp. stabilize time [s]"));
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    epr_out_float(o+EPR_EXTRUDER_ADVANCE_K,PSTR("Advance K [0=off]"));
#endif
    epr_out_float(o+EPR_EXTRUDER_ADVANCE_L,PSTR("Advance L [0=off]"));
#endif    
  }
#else
  out.println_P(PSTR("No EEPROM support compiled."));
#endif
}

