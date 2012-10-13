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

#include "Reptier.h"
#include "Eeprom.h"

#if EEPROM_MODE!=0
extern void epr_eeprom_to_data();

byte epr_compute_checksum() {
  int i;
  byte checksum=0;
  for(i=0;i<2048;i++) {
    if(i==EEPROM_OFFSET+EPR_INTEGRITY_BYTE) continue;
    checksum += eeprom_read_byte ((unsigned char *)(i));
  }
  return checksum;
}

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
  OUT_P("EPR:3 ");
  OUT(pos);
  OUT(' ');
  OUT(epr_get_float(pos));
  OUT(' ');
  out.println_P(text);  
}
void epr_out_long(uint pos,PGM_P text) {
  OUT_P("EPR:2 ");
  OUT(pos);
  OUT(' ');
  OUT(epr_get_long(pos));
  OUT(' ');
  out.println_P(text);  
}
void epr_out_int(uint pos,PGM_P text) {
  OUT_P("EPR:1 ");
  OUT(pos);
  OUT(' ');
  OUT(epr_get_int(pos));
  OUT(' ');
  out.println_P(text);  
}
void epr_out_byte(uint pos,PGM_P text) {
  OUT_P("EPR:0 ");
  OUT(pos);
  OUT(' ');
  OUT((int)epr_get_byte(pos));
  OUT(' ');
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
  byte newcheck = epr_compute_checksum();
  if(newcheck!=epr_get_byte(EPR_INTEGRITY_BYTE))
    epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
  epr_eeprom_to_data();
}

/** \brief Moves current settings to EEPROM.

The values the are currently set are used to fill the eeprom.*/
void epr_data_to_eeprom(byte corrupted) {
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
#if HAVE_HEATED_BED
  epr_set_byte(EPR_BED_HEAT_MANAGER,heatedBedController.heatManager);
#ifdef TEMP_PID
  epr_set_byte(EPR_BED_DRIVE_MAX,heatedBedController.pidDriveMax);
  epr_set_byte(EPR_BED_DRIVE_MIN,heatedBedController.pidDriveMin);
  epr_set_float(EPR_BED_PID_PGAIN,heatedBedController.pidPGain);
  epr_set_float(EPR_BED_PID_IGAIN,heatedBedController.pidIGain);
  epr_set_float(EPR_BED_PID_DGAIN,heatedBedController.pidDGain);
  epr_set_byte(EPR_BED_PID_MAX,heatedBedController.pidMax);
#endif
#endif
  epr_set_float(EPR_X_HOME_OFFSET,printer_state.xMin);
  epr_set_float(EPR_Y_HOME_OFFSET,printer_state.yMin);
  epr_set_float(EPR_Z_HOME_OFFSET,printer_state.zMin);
  epr_set_float(EPR_X_LENGTH,printer_state.xLength);
  epr_set_float(EPR_Y_LENGTH,printer_state.yLength);
  epr_set_float(EPR_Z_LENGTH,printer_state.zLength);

  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    epr_set_float(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
    epr_set_float(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
    epr_set_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
    epr_set_float(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
    epr_set_byte(o+EPR_EXTRUDER_HEAT_MANAGER,e->tempControl.heatManager);
#ifdef TEMP_PID
    epr_set_byte(o+EPR_EXTRUDER_DRIVE_MAX,e->tempControl.pidDriveMax);
    epr_set_byte(o+EPR_EXTRUDER_DRIVE_MIN,e->tempControl.pidDriveMin);
    epr_set_float(o+EPR_EXTRUDER_PID_PGAIN,e->tempControl.pidPGain);
    epr_set_float(o+EPR_EXTRUDER_PID_IGAIN,e->tempControl.pidIGain);
    epr_set_float(o+EPR_EXTRUDER_PID_DGAIN,e->tempControl.pidDGain);
    epr_set_byte(o+EPR_EXTRUDER_PID_MAX,e->tempControl.pidMax);
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
  if(corrupted) {
    epr_set_long(EPR_PRINTING_TIME,0);
    epr_set_float(EPR_PRINTING_DISTANCE,0);
  }
  // Save version and build checksum
  epr_set_byte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
  epr_set_byte(EPR_INTEGRITY_BYTE,epr_compute_checksum());
}
/** \brief Copy data from EEPROM to variables.
*/
void epr_eeprom_to_data() {
  byte version = epr_get_byte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
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
#if HAVE_HEATED_BED
  heatedBedController.heatManager= epr_get_byte(EPR_BED_HEAT_MANAGER);
#ifdef TEMP_PID
  heatedBedController.pidDriveMax = epr_get_byte(EPR_BED_DRIVE_MAX);
  heatedBedController.pidDriveMin = epr_get_byte(EPR_BED_DRIVE_MIN);
  heatedBedController.pidPGain = epr_get_float(EPR_BED_PID_PGAIN);
  heatedBedController.pidIGain = epr_get_float(EPR_BED_PID_IGAIN);
  heatedBedController.pidDGain = epr_get_float(EPR_BED_PID_DGAIN);
  heatedBedController.pidMax = epr_get_byte(EPR_BED_PID_MAX);
#endif
#endif
  printer_state.xMin = epr_get_float(EPR_X_HOME_OFFSET);
  printer_state.yMin = epr_get_float(EPR_Y_HOME_OFFSET);
  printer_state.zMin = epr_get_float(EPR_Z_HOME_OFFSET);
  printer_state.xLength = epr_get_float(EPR_X_LENGTH);
  printer_state.yLength = epr_get_float(EPR_Y_LENGTH);
  printer_state.zLength = epr_get_float(EPR_Z_LENGTH);
  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    e->stepsPerMM = epr_get_float(o+EPR_EXTRUDER_STEPS_PER_MM);
    e->maxFeedrate = epr_get_float(o+EPR_EXTRUDER_MAX_FEEDRATE);
    e->maxStartFeedrate = epr_get_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
    e->maxAcceleration = epr_get_float(o+EPR_EXTRUDER_MAX_ACCELERATION);
    e->tempControl.heatManager = epr_get_byte(o+EPR_EXTRUDER_HEAT_MANAGER);
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = epr_get_byte(o+EPR_EXTRUDER_DRIVE_MAX);
    e->tempControl.pidDriveMin = epr_get_byte(o+EPR_EXTRUDER_DRIVE_MIN);
    e->tempControl.pidPGain = epr_get_float(o+EPR_EXTRUDER_PID_PGAIN);
    e->tempControl.pidIGain = epr_get_float(o+EPR_EXTRUDER_PID_IGAIN);
    e->tempControl.pidDGain = epr_get_float(o+EPR_EXTRUDER_PID_DGAIN);
    e->tempControl.pidMax = epr_get_byte(o+EPR_EXTRUDER_PID_MAX);
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
  if(version!=EEPROM_PROTOCOL_VERSION) {
    OUT_P_LN("Protocol version changed, upgrading");
    epr_data_to_eeprom(false); // Store new fields for changed version
  }
  extruder_select(current_extruder->id);
  update_ramps_parameter();
  initHeatedBed();
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
  byte check = epr_compute_checksum();
  byte storedcheck = epr_get_byte(EPR_INTEGRITY_BYTE);
  if(epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE && storedcheck==check) {
    epr_eeprom_to_data();
  } else {
    epr_set_byte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make datachange permanent
    epr_data_to_eeprom(storedcheck==check);
  }
#endif
}
/**
*/
void epr_update_usage() {
#if EEPROM_MODE!=0
  unsigned long seconds = (millis()-printer_state.msecondsPrinting);
  seconds += epr_get_long(EPR_PRINTING_TIME);
  epr_set_long(EPR_PRINTING_TIME,seconds);
  epr_set_float(EPR_PRINTING_DISTANCE,epr_get_float(EPR_PRINTING_DISTANCE)+printer_state.filamentPrinted*0.001);
  OUT_P_F_LN("Adding filament:",printer_state.filamentPrinted);
  printer_state.filamentPrinted = 0;
  printer_state.msecondsPrinting = millis();
  byte newcheck = epr_compute_checksum();
  if(newcheck!=epr_get_byte(EPR_INTEGRITY_BYTE))
    epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
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
  epr_out_float(EPR_PRINTING_DISTANCE,PSTR("Filament printed [m]"));
  epr_out_long(EPR_PRINTING_TIME,PSTR("Printer active [s]"));
  epr_out_long(EPR_MAX_INACTIVE_TIME,PSTR("Max. inactive time [ms,0=off]"));
  epr_out_long(EPR_STEPPER_INACTIVE_TIME,PSTR("Stop stepper after inactivity [ms,0=off]"));
//#define EPR_ACCELERATION_TYPE 1
  epr_out_float(EPR_XAXIS_STEPS_PER_MM,PSTR("X-axis steps per mm"));
  epr_out_float(EPR_YAXIS_STEPS_PER_MM,PSTR("Y-axis steps per mm"));
  epr_out_float(EPR_ZAXIS_STEPS_PER_MM,PSTR("Z-axis steps per mm"));
  epr_out_float(EPR_X_MAX_FEEDRATE,PSTR("X-axis max. feedrate [mm/s]"));
  epr_out_float(EPR_Y_MAX_FEEDRATE,PSTR("Y-axis max. feedrate [mm/s]"));
  epr_out_float(EPR_Z_MAX_FEEDRATE,PSTR("Z-axis max. feedrate [mm/s]"));
  epr_out_float(EPR_X_HOMING_FEEDRATE,PSTR("X-axis homing feedrate [mm/s]"));
  epr_out_float(EPR_Y_HOMING_FEEDRATE,PSTR("Y-axis homing feedrate [mm/s]"));
  epr_out_float(EPR_Z_HOMING_FEEDRATE,PSTR("Z-axis homing feedrate [mm/s]"));
  epr_out_float(EPR_MAX_JERK,PSTR("Max. jerk [mm/s]"));
  epr_out_float(EPR_MAX_ZJERK,PSTR("Max. Z-jerk [mm/s]"));
  epr_out_float(EPR_X_HOME_OFFSET,PSTR("X home pos [mm]"));
  epr_out_float(EPR_Y_HOME_OFFSET,PSTR("Y home pos [mm]"));
  epr_out_float(EPR_Z_HOME_OFFSET,PSTR("Z home pos [mm]"));
  epr_out_float(EPR_X_LENGTH,PSTR("X max length [mm]"));
  epr_out_float(EPR_Y_LENGTH,PSTR("Y max length [mm]"));
  epr_out_float(EPR_Z_LENGTH,PSTR("Z max length [mm]"));

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
#if HAVE_HEATED_BED
  epr_out_byte(EPR_BED_HEAT_MANAGER,PSTR("Bed Heat Manager [0-2]"));
#ifdef TEMP_PID
  epr_out_byte(EPR_BED_DRIVE_MAX,PSTR("Bed PID drive max"));
  epr_out_byte(EPR_BED_DRIVE_MIN,PSTR("Bed PID drive min"));
  epr_out_float(EPR_BED_PID_PGAIN,PSTR("Bed PID P-gain"));
  epr_out_float(EPR_BED_PID_IGAIN,PSTR("Bed PID I-gain"));
  epr_out_float(EPR_BED_PID_DGAIN,PSTR("Bed PID D-gain"));
  epr_out_byte(EPR_BED_PID_MAX,PSTR("Bed PID max value [0-255]"));
#endif
#endif
  // now the extruder
  for(byte i=0;i<NUM_EXTRUDER;i++) {
    int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    epr_out_float(o+EPR_EXTRUDER_STEPS_PER_MM,PSTR("Extr. steps per mm"));
    epr_out_float(o+EPR_EXTRUDER_MAX_FEEDRATE,PSTR("Extr. max. feedrate [mm/s]"));
    epr_out_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,PSTR("Extr. start feedrate [mm/s]"));
    epr_out_float(o+EPR_EXTRUDER_MAX_ACCELERATION,PSTR("Extr. acceleration [mm/s^2]"));
    epr_out_byte(o+EPR_EXTRUDER_HEAT_MANAGER,PSTR("Heat manager [0-1]"));
#ifdef TEMP_PID
    epr_out_byte(o+EPR_EXTRUDER_DRIVE_MAX,PSTR("PID drive max"));
    epr_out_byte(o+EPR_EXTRUDER_DRIVE_MIN,PSTR("PID drive min"));
    epr_out_float(o+EPR_EXTRUDER_PID_PGAIN,PSTR("PID P-gain"));
    epr_out_float(o+EPR_EXTRUDER_PID_IGAIN,PSTR("PID I-gain"));
    epr_out_float(o+EPR_EXTRUDER_PID_DGAIN,PSTR("PID D-gain"));
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

