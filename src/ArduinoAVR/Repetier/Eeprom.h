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

*/

#ifndef _EEPROM_H
#define _EEPROM_H

#include <avr/eeprom.h>

// Id to distinguish version changes 
#define EEPROM_PROTOCOL_VERSION 2

/** Where to start with our datablock in memory. Can be moved if you
have problems with other modules using the eeprom */

#define EEPROM_OFFSET               0
#define EPR_MAGIC_BYTE              0
#define EPR_ACCELERATION_TYPE       1
#define EPR_XAXIS_STEPS_PER_MM      3
#define EPR_YAXIS_STEPS_PER_MM      7
#define EPR_ZAXIS_STEPS_PER_MM     11
#define EPR_X_MAX_FEEDRATE         15
#define EPR_Y_MAX_FEEDRATE         19
#define EPR_Z_MAX_FEEDRATE         23
#define EPR_X_HOMING_FEEDRATE      27
#define EPR_Y_HOMING_FEEDRATE      31
#define EPR_Z_HOMING_FEEDRATE      35
#define EPR_MAX_JERK               39
#define EPR_OPS_MIN_DISTANCE       43
#define EPR_MAX_ZJERK              47
#define EPR_X_MAX_ACCEL            51
#define EPR_Y_MAX_ACCEL            55
#define EPR_Z_MAX_ACCEL            59
#define EPR_X_MAX_TRAVEL_ACCEL     63
#define EPR_Y_MAX_TRAVEL_ACCEL     67
#define EPR_Z_MAX_TRAVEL_ACCEL     71
#define EPR_BAUDRATE               75
#define EPR_MAX_INACTIVE_TIME      79
#define EPR_STEPPER_INACTIVE_TIME  83
#define EPR_OPS_RETRACT_DISTANCE   87
#define EPR_OPS_RETRACT_BACKLASH   91
#define EPR_EXTRUDER_SPEED         95
#define EPR_OPS_MOVE_AFTER         99
#define EPR_OPS_MODE              103
#define EPR_INTEGRITY_BYTE        104   // Here the xored sum over eeprom is stored
#define EPR_VERSION               105   // Version id for updates in EEPROM storage
#define EPR_BED_HEAT_MANAGER      106
#define EPR_BED_DRIVE_MAX         107
#define EPR_BED_PID_PGAIN         108
#define EPR_BED_PID_IGAIN         112
#define EPR_BED_PID_DGAIN         116
#define EPR_BED_PID_MAX           120
#define EPR_BED_DRIVE_MIN         124
#define EPR_PRINTING_TIME         125  // Time in seconds printing
#define EPR_PRINTING_DISTANCE     129  // Filament length printed
#define EPR_X_HOME_OFFSET         133
#define EPR_Y_HOME_OFFSET         137
#define EPR_Z_HOME_OFFSET         141
#define EPR_X_LENGTH              145
#define EPR_Y_LENGTH              149
#define EPR_Z_LENGTH              153
#define EPR_BACKLASH_X            157
#define EPR_BACKLASH_Y            161
#define EPR_BACKLASH_Z            165

#define EEPROM_EXTRUDER_OFFSET 200
// bytes per extruder needed, leave some space for future development
#define EEPROM_EXTRUDER_LENGTH 100
// Extruder positions relative to extruder start
#define EPR_EXTRUDER_STEPS_PER_MM        0
#define EPR_EXTRUDER_MAX_FEEDRATE        4
// Feedrate from halted extruder in mm/s
#define EPR_EXTRUDER_MAX_START_FEEDRATE  8
// Acceleration in mm/s^2
#define EPR_EXTRUDER_MAX_ACCELERATION   12
#define EPR_EXTRUDER_HEAT_MANAGER       16
#define EPR_EXTRUDER_DRIVE_MAX          17
#define EPR_EXTRUDER_PID_PGAIN          18
#define EPR_EXTRUDER_PID_IGAIN          22
#define EPR_EXTRUDER_PID_DGAIN          26
#define EPR_EXTRUDER_PID_MAX            30
#define EPR_EXTRUDER_X_OFFSET           31
#define EPR_EXTRUDER_Y_OFFSET           35
#define EPR_EXTRUDER_WATCH_PERIOD       39
#define EPR_EXTRUDER_ADVANCE_K          41
#define EPR_EXTRUDER_DRIVE_MIN          45
#define EPR_EXTRUDER_ADVANCE_L          46
#define EPR_EXTRUDER_WAIT_RETRACT_TEMP 50
#define EPR_EXTRUDER_WAIT_RETRACT_UNITS 52
#define EPR_EXTRUDER_COOLER_SPEED       54
#if EEPROM_MODE!=0

extern inline void epr_set_byte(uint pos,byte value);
extern inline void epr_set_int(uint pos,int value);
extern inline void epr_set_long(uint pos,long value);
extern inline void epr_set_float(uint pos,float value);
extern void epr_data_to_eeprom(byte corrupted);
extern void epr_eeprom_to_data();
extern void epr_eeprom_reset();

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
#endif

extern void epr_output_settings();
extern void epr_update(GCode *com);
extern void epr_init();
extern void epr_init_baudrate();
extern void epr_update_usage();
#endif

