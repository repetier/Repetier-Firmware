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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************** READ FIRST ************************

   This configuration file was created with the configuration tool. For that
   reason, it does not contain the same informations as the original Configuration.h file.
   It misses the comments and unused parts. Open this file file in the config tool
   to see and change the data. You can also upload it to newer/older versions. The system
   will silently add new options, so compilation continues to work.

   This file is optimized for version 1.0.3dev
   generator: http://www.repetier.com/firmware/dev/

   If you are in doubt which named functions use which pins on your board, please check the
   pins.h for the used name->pin assignments and your board documentation to verify it is
   as you expect.

*/

#define NUM_EXTRUDER 4
#define MOTHERBOARD 35
#include "pins.h"

// ################## EDIT THESE SETTINGS MANUALLY ################

// ################ END MANUAL SETTINGS ##########################

#define HOST_RESCUE 1
#undef FAN_BOARD_PIN
#define FAN_BOARD_PIN ORIG_FAN_PIN
#define BOARD_FAN_SPEED 255
#define BOARD_FAN_MIN_SPEED 0
#define FAN_THERMO_PIN -1
#define FAN_THERMO_MIN_PWM 128
#define FAN_THERMO_MAX_PWM 255
#define FAN_THERMO_MIN_TEMP 45
#define FAN_THERMO_MAX_TEMP 60
#define FAN_THERMO_THERMISTOR_PIN -1
#define FAN_THERMO_THERMISTOR_TYPE 1

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1
#define BLUETOOTH_SERIAL  -1
#define BLUETOOTH_BAUD  115200
#define MIXING_EXTRUDER 0

#define DRIVE_SYSTEM 0
#define XAXIS_STEPS_PER_MM 72.72
#define YAXIS_STEPS_PER_MM 72.72
#define ZAXIS_STEPS_PER_MM 520
#define EXTRUDER_FAN_COOL_TEMP 50
#define PDM_FOR_EXTRUDER 0
#define PDM_FOR_COOLER 0
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 20
#define DECOUPLING_TEST_MIN_TEMP_RISE 1
#define KILL_IF_SENSOR_DEFECT 1
#define RETRACT_ON_PAUSE 4
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""
#define SHARED_EXTRUDER_HEATER 0
#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
#define EXT0_Z_OFFSET 0
#define EXT0_STEPS_PER_MM 255
#define EXT0_TEMPSENSOR_TYPE 5
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE 1
#define EXT0_ENABLE_PIN ORIG_E0_ENABLE_PIN
#define EXT0_ENABLE_ON 1
#define EXT0_MIRROR_STEPPER 0
#define EXT0_STEP2_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR2_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE2 0
#define EXT0_ENABLE2_PIN ORIG_E0_ENABLE_PIN
#define EXT0_MAX_FEEDRATE 50
#define EXT0_MAX_START_FEEDRATE 20
#define EXT0_MAX_ACCELERATION 5000
#define EXT0_HEAT_MANAGER 1
#define EXT0_PREHEAT_TEMP 190
#define EXT0_WATCHPERIOD 1
#define EXT0_PID_INTEGRAL_DRIVE_MAX 255
#define EXT0_PID_INTEGRAL_DRIVE_MIN 40
#define EXT0_PID_PGAIN_OR_DEAD_TIME 8.94
#define EXT0_PID_I 0.51
#define EXT0_PID_D 20
#define EXT0_PID_MAX 255
#define EXT0_ADVANCE_K 0
#define EXT0_ADVANCE_L 0
#define EXT0_ADVANCE_BACKLASH_STEPS 0
#define EXT0_WAIT_RETRACT_TEMP 150
#define EXT0_WAIT_RETRACT_UNITS 0
#define EXT0_SELECT_COMMANDS ""
#define EXT0_DESELECT_COMMANDS ""
#define EXT0_EXTRUDER_COOLER_PIN -1
#define EXT0_EXTRUDER_COOLER_SPEED 255
#define EXT0_DECOUPLE_TEST_PERIOD 30000
#define EXT0_JAM_PIN -1
#define EXT0_JAM_PULLUP 0
#define EXT1_X_OFFSET 0
#define EXT1_Y_OFFSET 0
#define EXT1_Z_OFFSET 0
#define EXT1_STEPS_PER_MM 257
#define EXT1_TEMPSENSOR_TYPE 5
#define EXT1_TEMPSENSOR_PIN TEMP_2_PIN
#define EXT1_HEATER_PIN HEATER_2_PIN
#define EXT1_STEP_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE 1
#define EXT1_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define EXT1_ENABLE_ON 1
#define EXT1_MIRROR_STEPPER 0
#define EXT1_STEP2_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR2_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE2 0
#define EXT1_ENABLE2_PIN ORIG_E1_ENABLE_PIN
#define EXT1_MAX_FEEDRATE 100
#define EXT1_MAX_START_FEEDRATE 40
#define EXT1_MAX_ACCELERATION 5000
#define EXT1_HEAT_MANAGER 1
#define EXT1_PREHEAT_TEMP 190
#define EXT1_WATCHPERIOD 1
#define EXT1_PID_INTEGRAL_DRIVE_MAX 255
#define EXT1_PID_INTEGRAL_DRIVE_MIN 40
#define EXT1_PID_PGAIN_OR_DEAD_TIME 8.94
#define EXT1_PID_I 0.51
#define EXT1_PID_D 20
#define EXT1_PID_MAX 255
#define EXT1_ADVANCE_K 0
#define EXT1_ADVANCE_L 0
#define EXT1_ADVANCE_BACKLASH_STEPS 0
#define EXT1_WAIT_RETRACT_TEMP 150
#define EXT1_WAIT_RETRACT_UNITS 0
#define EXT1_SELECT_COMMANDS ""
#define EXT1_DESELECT_COMMANDS ""
#define EXT1_EXTRUDER_COOLER_PIN -1
#define EXT1_EXTRUDER_COOLER_SPEED 255
#define EXT1_DECOUPLE_TEST_PERIOD 30000
#define EXT1_JAM_PIN -1
#define EXT1_JAM_PULLUP 0
#define EXT2_X_OFFSET 0
#define EXT2_Y_OFFSET 0
#define EXT2_Z_OFFSET 0
#define EXT2_STEPS_PER_MM 257
#define EXT2_TEMPSENSOR_TYPE 5
#define EXT2_TEMPSENSOR_PIN TEMP_3_PIN
#define EXT2_HEATER_PIN HEATER_3_PIN
#define EXT2_STEP_PIN ORIG_E2_STEP_PIN
#define EXT2_DIR_PIN ORIG_E2_DIR_PIN
#define EXT2_INVERSE 1
#define EXT2_ENABLE_PIN ORIG_E2_ENABLE_PIN
#define EXT2_ENABLE_ON 1
#define EXT2_MIRROR_STEPPER 0
#define EXT2_STEP2_PIN ORIG_E2_STEP_PIN
#define EXT2_DIR2_PIN ORIG_E2_DIR_PIN
#define EXT2_INVERSE2 0
#define EXT2_ENABLE2_PIN ORIG_E2_ENABLE_PIN
#define EXT2_MAX_FEEDRATE 100
#define EXT2_MAX_START_FEEDRATE 40
#define EXT2_MAX_ACCELERATION 5000
#define EXT2_HEAT_MANAGER 1
#define EXT2_PREHEAT_TEMP 190
#define EXT2_WATCHPERIOD 1
#define EXT2_PID_INTEGRAL_DRIVE_MAX 255
#define EXT2_PID_INTEGRAL_DRIVE_MIN 40
#define EXT2_PID_PGAIN_OR_DEAD_TIME 8.94
#define EXT2_PID_I 0.51
#define EXT2_PID_D 20
#define EXT2_PID_MAX 255
#define EXT2_ADVANCE_K 0
#define EXT2_ADVANCE_L 0
#define EXT2_ADVANCE_BACKLASH_STEPS 0
#define EXT2_WAIT_RETRACT_TEMP 150
#define EXT2_WAIT_RETRACT_UNITS 0
#define EXT2_SELECT_COMMANDS ""
#define EXT2_DESELECT_COMMANDS ""
#define EXT2_EXTRUDER_COOLER_PIN -1
#define EXT2_EXTRUDER_COOLER_SPEED 255
#define EXT2_DECOUPLE_TEST_PERIOD 30000
#define EXT2_JAM_PIN -1
#define EXT2_JAM_PULLUP 0
#define EXT3_X_OFFSET 0
#define EXT3_Y_OFFSET 0
#define EXT3_Z_OFFSET 0
#define EXT3_STEPS_PER_MM 257
#define EXT3_TEMPSENSOR_TYPE 5
#define EXT3_TEMPSENSOR_PIN TEMP_4_PIN
#define EXT3_HEATER_PIN HEATER_4_PIN
#define EXT3_STEP_PIN ORIG_E3_STEP_PIN
#define EXT3_DIR_PIN ORIG_E3_DIR_PIN
#define EXT3_INVERSE 1
#define EXT3_ENABLE_PIN ORIG_E3_ENABLE_PIN
#define EXT3_ENABLE_ON 1
#define EXT3_MIRROR_STEPPER 0
#define EXT3_STEP2_PIN ORIG_E3_STEP_PIN
#define EXT3_DIR2_PIN ORIG_E3_DIR_PIN
#define EXT3_INVERSE2 0
#define EXT3_ENABLE2_PIN ORIG_E3_ENABLE_PIN
#define EXT3_MAX_FEEDRATE 100
#define EXT3_MAX_START_FEEDRATE 40
#define EXT3_MAX_ACCELERATION 5000
#define EXT3_HEAT_MANAGER 1
#define EXT3_PREHEAT_TEMP 190
#define EXT3_WATCHPERIOD 1
#define EXT3_PID_INTEGRAL_DRIVE_MAX 255
#define EXT3_PID_INTEGRAL_DRIVE_MIN 40
#define EXT3_PID_PGAIN_OR_DEAD_TIME 8.94
#define EXT3_PID_I 0.51
#define EXT3_PID_D 20
#define EXT3_PID_MAX 255
#define EXT3_ADVANCE_K 0
#define EXT3_ADVANCE_L 0
#define EXT3_ADVANCE_BACKLASH_STEPS 0
#define EXT3_WAIT_RETRACT_TEMP 150
#define EXT3_WAIT_RETRACT_UNITS 0
#define EXT3_SELECT_COMMANDS ""
#define EXT3_DESELECT_COMMANDS ""
#define EXT3_EXTRUDER_COOLER_PIN -1
#define EXT3_EXTRUDER_COOLER_SPEED 255
#define EXT3_DECOUPLE_TEST_PERIOD 30000
#define EXT3_JAM_PIN -1
#define EXT3_JAM_PULLUP 0

#define FEATURE_RETRACTION 1
#define AUTORETRACT_ENABLED 0
#define RETRACTION_LENGTH 2.5
#define RETRACTION_LONG_LENGTH 2.5
#define RETRACTION_SPEED 50
#define RETRACTION_Z_LIFT 0
#define RETRACTION_UNDO_EXTRA_LENGTH 0
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0
#define RETRACTION_UNDO_SPEED 40
#define FILAMENTCHANGE_X_POS 0
#define FILAMENTCHANGE_Y_POS 0
#define FILAMENTCHANGE_Z_ADD  2
#define FILAMENTCHANGE_REHOME 1
#define FILAMENTCHANGE_SHORTRETRACT 5
#define FILAMENTCHANGE_LONGRETRACT 50
#define JAM_METHOD 1
#define JAM_STEPS 220
#define JAM_SLOWDOWN_STEPS 320
#define JAM_SLOWDOWN_TO 70
#define JAM_ERROR_STEPS 500
#define JAM_MIN_STEPS 10
#define JAM_ACTION 1

#define RETRACT_DURING_HEATUP true
#define PID_CONTROL_RANGE 20
#define SKIP_M109_IF_WITHIN 2
#define SCALE_PID_TO_MAX 0
#define TEMP_HYSTERESIS 0
#define EXTRUDE_MAXLENGTH 160
#define NUM_TEMPS_USERTHERMISTOR0 29
#define USER_THERMISTORTABLE0 {{0,8000},{69,2280},{79,2200},{92,2120},{107,2040},{125,1960},{146,1886},{172,1810},{204,1736},{243,1661},{291,1586},{350,1511},{422,1437},{512,1362},{622,1287},{756,1212},{919,1138},{1345,988},{3139,539},{3388,464},{3591,390},{3749,314},{3866,240},{3949,160},{4005,80},{4041,0},{4064,-80},{4086,-240},{4093,-400}}
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1 {}
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2 {}
#define GENERIC_THERM_VREF 5
#define GENERIC_THERM_NUM_ENTRIES 33
#define TEMP_GAIN 1
#define HEATER_PWM_SPEED 0
#define COOLER_PWM_SPEED 0

// ############# Heated bed configuration ########################

#define HAVE_HEATED_BED 1
#define HEATED_BED_PREHEAT_TEMP 55
#define HEATED_BED_MAX_TEMP 120
#define SKIP_M190_IF_WITHIN 3
#define HEATED_BED_SENSOR_TYPE 8
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
#define HEATED_BED_SET_INTERVAL 5000
#define HEATED_BED_HEAT_MANAGER 1
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME   196
#define HEATED_BED_PID_IGAIN   33
#define HEATED_BED_PID_DGAIN 290
#define HEATED_BED_PID_MAX 255
#define HEATED_BED_DECOUPLE_TEST_PERIOD 300000
#define MIN_EXTRUDER_TEMP 150
#define MAXTEMP 280
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300
#define MILLISECONDS_PREHEAT_TIME 30000

// ##########################################################################################
// ##                             Laser configuration                                      ##
// ##########################################################################################

/*
If the firmware is in laser mode, it can control a laser output to cut or engrave materials.
Please use this feature only if you know about safety and required protection. Lasers are
dangerous and can hurt or make you blind!!!

The default laser driver only supports laser on and off. Here you control the intensity with
your feedrate. For exchangeable diode lasers this is normally enough. If you need more control
you can set the intensity in a range 0-255 with a custom extension to the driver. See driver.h
and comments on how to extend the functions non invasive with our event system.

If you have a laser - powder system you will like your E override. If moves contain a
increasing extruder position it will laser that move. With this trick you can
use existing fdm slicers to laser the output. Laser width is extrusion width.

Other tools may use M3 and M5 to enable/disable laser. Here G1/G2/G3 moves have laser enabled
and G0 moves have it disables.

In any case, laser only enables while moving. At the end of a move it gets
automatically disabled.
*/

#define SUPPORT_LASER 0
#define LASER_PIN -1
#define LASER_ON_HIGH 1
#define LASER_WARMUP_TIME 0
#define LASER_PWM_MAX 255
#define LASER_WATT 2

// ##                              CNC configuration                                       ##

/*
If the firmware is in CNC mode, it can control a mill with M3/M4/M5. It works
similar to laser mode, but mill keeps enabled during G0 moves and it allows
setting rpm (only with event extension that supports this) and milling direction.
It also can add a delay to wait for spindle to run on full speed.
*/

#define SUPPORT_CNC 0
#define CNC_WAIT_ON_ENABLE 300
#define CNC_WAIT_ON_DISABLE 0
#define CNC_ENABLE_PIN -1
#define CNC_ENABLE_WITH 1
#define CNC_DIRECTION_PIN -1
#define CNC_DIRECTION_CW 1
#define CNC_PWM_MAX 255
#define CNC_RPM_MAX 8000
#define CNC_SAFE_Z 150

#define DEFAULT_PRINTER_MODE 0

// ################ Endstop configuration #####################

#define MULTI_ZENDSTOP_HOMING 0
#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_X_MIN_INVERTING true
#define MIN_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_Y_MIN_INVERTING true
#define MIN_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_Z_MIN_INVERTING true
#define MIN_HARDWARE_ENDSTOP_Z true
#define ENDSTOP_PULLUP_Z2_MINMAX true
#define ENDSTOP_Z2_MINMAX_INVERTING false
#define MINMAX_HARDWARE_ENDSTOP_Z2 false
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_X_MAX_INVERTING true
#define MAX_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_Y_MAX_INVERTING true
#define MAX_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MAX true
#define ENDSTOP_Z_MAX_INVERTING true
#define MAX_HARDWARE_ENDSTOP_Z true
#define ENDSTOP_PULLUP_X2_MIN true
#define ENDSTOP_PULLUP_Y2_MIN true
#define ENDSTOP_PULLUP_Z2_MINMAX true
#define ENDSTOP_PULLUP_X2_MAX true
#define ENDSTOP_PULLUP_Y2_MAX true
#define ENDSTOP_X2_MIN_INVERTING false
#define ENDSTOP_Y2_MIN_INVERTING false
#define ENDSTOP_X2_MAX_INVERTING false
#define ENDSTOP_Y2_MAX_INVERTING false
#define MIN_HARDWARE_ENDSTOP_X2 false
#define MIN_HARDWARE_ENDSTOP_Y2 false
#define MAX_HARDWARE_ENDSTOP_X2 false
#define MAX_HARDWARE_ENDSTOP_Y2 false
#define X2_MIN_PIN -1
#define X2_MAX_PIN -1
#define Y2_MIN_PIN -1
#define Y2_MAX_PIN -1
#define Z2_MINMAX_PIN -1



#define max_software_endstop_r true

#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false
#define max_software_endstop_x false
#define max_software_endstop_y false
#define max_software_endstop_z false
#define DOOR_PIN -1
#define DOOR_PULLUP 1
#define DOOR_INVERTING 1
#define ENDSTOP_X_BACK_MOVE 6
#define ENDSTOP_Y_BACK_MOVE 6
#define ENDSTOP_Z_BACK_MOVE 6
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 6
#define ENDSTOP_X_BACK_ON_HOME 1
#define ENDSTOP_Y_BACK_ON_HOME 1
#define ENDSTOP_Z_BACK_ON_HOME 0
#define ALWAYS_CHECK_ENDSTOPS 1
#define MOVE_X_WHEN_HOMED 0
#define MOVE_Y_WHEN_HOMED 0
#define MOVE_Z_WHEN_HOMED 0

// ################# XYZ movements ###################

#define X_ENABLE_ON 1
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 1
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0
#define DISABLE_E 0
#define PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT
#define INVERT_X_DIR 0
#define INVERT_X2_DIR 0
#define INVERT_Y_DIR 1
#define INVERT_Y2_DIR 0
#define INVERT_Z_DIR 0
#define INVERT_Z2_DIR 0
#define INVERT_Z3_DIR 0
#define INVERT_Z4_DIR 0
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define X_MAX_LENGTH 418
#define Y_MAX_LENGTH 240
#define Z_MAX_LENGTH 210
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define PARK_POSITION_X 0
#define PARK_POSITION_Y 10
#define PARK_POSITION_Z_RAISE 10


#define DISTORTION_CORRECTION 0
#define DISTORTION_CORRECTION_POINTS 5
#define DISTORTION_LIMIT_TO 2
#define DISTORTION_CORRECTION_R 100
#define DISTORTION_PERMANENT 1
#define DISTORTION_UPDATE_FREQUENCY 15
#define DISTORTION_START_DEGRADE 0.5
#define DISTORTION_END_HEIGHT 1
#define DISTORTION_EXTRAPOLATE_CORNERS 0
#define DISTORTION_XMIN 10
#define DISTORTION_YMIN 10
#define DISTORTION_XMAX 190
#define DISTORTION_YMAX 190

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define FEATURE_BABYSTEPPING 1
#define BABYSTEP_MULTIPLICATOR 1

#define DELTA_SEGMENTS_PER_SECOND_PRINT 180 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70 // Less accurate setting for other moves
#define EXACT_DELTA_MOVES 1

// Delta settings
#define DELTA_HOME_ON_POWER 0

#define DELTASEGMENTS_PER_PRINTLINE 24
#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 0L
#define MAX_FEEDRATE_X 200
#define MAX_FEEDRATE_Y 200
#define MAX_FEEDRATE_Z 80
#define HOMING_FEEDRATE_X 40
#define HOMING_FEEDRATE_Y 40
#define HOMING_FEEDRATE_Z 40
#define HOMING_ORDER HOME_ORDER_XYZ
#define ZHOME_PRE_RAISE 0
#define ZHOME_PRE_RAISE_DISTANCE 10
#define RAISE_Z_ON_TOOLCHANGE 0
#define ZHOME_MIN_TEMPERATURE 0
#define ZHOME_HEAT_ALL 1
#define ZHOME_HEAT_HEIGHT 20
#define ZHOME_X_POS 999999
#define ZHOME_Y_POS 999999
#define ENABLE_BACKLASH_COMPENSATION 1
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define RAMP_ACCELERATION 1
#define STEPPER_HIGH_DELAY 0
#define DIRECTION_DELAY 0
#define STEP_DOUBLER_FREQUENCY 12000
#define ALLOW_QUADSTEPPING 1
#define DOUBLE_STEP_DELAY 1 // time in microseconds
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 50
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 50
#define INTERPOLATE_ACCELERATION_WITH_Z 0
#define ACCELERATION_FACTOR_TOP 100
#define MAX_JERK 25
#define MAX_ZJERK 0.3
#define PRINTLINE_CACHE_SIZE 24
#define MOVE_CACHE_LOW 10
#define LOW_TICKS_PER_MOVE 250000
#define EXTRUDER_SWITCH_XY_SPEED 100
#define DUAL_X_AXIS 0
#define DUAL_X_RESOLUTION 0
#define X2AXIS_STEPS_PER_MM 100
#define FEATURE_TWO_XSTEPPER 0
#define X2_STEP_PIN   ORIG_E1_STEP_PIN
#define X2_DIR_PIN    ORIG_E1_DIR_PIN
#define X2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_YSTEPPER 0
#define Y2_STEP_PIN   ORIG_E1_STEP_PIN
#define Y2_DIR_PIN    ORIG_E1_DIR_PIN
#define Y2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_ZSTEPPER 0
#define Z2_STEP_PIN   ORIG_E1_STEP_PIN
#define Z2_DIR_PIN    ORIG_E1_DIR_PIN
#define Z2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_THREE_ZSTEPPER 0
#define Z3_STEP_PIN   ORIG_E2_STEP_PIN
#define Z3_DIR_PIN    ORIG_E2_DIR_PIN
#define Z3_ENABLE_PIN ORIG_E2_ENABLE_PIN
#define FEATURE_FOUR_ZSTEPPER 0
#define Z4_STEP_PIN   ORIG_E3_STEP_PIN
#define Z4_DIR_PIN    ORIG_E3_DIR_PIN
#define Z4_ENABLE_PIN ORIG_E3_ENABLE_PIN
#define FEATURE_DITTO_PRINTING 1
#define USE_ADVANCE 1
#define ENABLE_QUADRATIC_ADVANCE 0


// ################# Misc. settings ##################

#define BAUDRATE 250000
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0
#define AUTOMATIC_POWERUP 0
#define KILL_METHOD 1
#define EMERGENCY_PARSER 1
#define ACK_WITH_LINENUMBER 1
#define KEEP_ALIVE_INTERVAL 2000
#define WAITING_IDENTIFIER "wait"
#define ECHO_ON_EXECUTE 1
#define EEPROM_MODE 1
#undef PS_ON_PIN
#define PS_ON_PIN -1
#define JSON_OUTPUT 1

/* ======== Servos =======
Control the servos with
M340 P<servoId> S<pulseInUS>   / ServoID = 0..3  pulseInUs = 500..2500
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/
#define FEATURE_SERVO 0
#define SERVO0_PIN 11
#define SERVO1_PIN -1
#define SERVO2_PIN -1
#define SERVO3_PIN -1
#define SERVO0_NEUTRAL_POS  -1
#define SERVO1_NEUTRAL_POS  -1
#define SERVO2_NEUTRAL_POS  -1
#define SERVO3_NEUTRAL_POS  -1
#define UI_SERVO_CONTROL 0
#define FAN_KICKSTART_TIME  200
#define MAX_FAN_PWM 255

        #define FEATURE_WATCHDOG 1

// #################### Z-Probing #####################

#define Z_PROBE_Z_OFFSET 0
#define Z_PROBE_Z_OFFSET_MODE 0
#define UI_BED_COATING 0
#define FEATURE_Z_PROBE 0
#define EXTRUDER_IS_Z_PROBE 0
#define Z_PROBE_DISABLE_HEATERS 0
#define Z_PROBE_BED_DISTANCE 10
#define Z_PROBE_PIN -1
#define Z_PROBE_PULLUP 0
#define Z_PROBE_ON_HIGH 0
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
#define Z_PROBE_WAIT_BEFORE_TEST 0
#define Z_PROBE_SPEED 2
#define Z_PROBE_XY_SPEED 150
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_REPETITIONS 1
#define Z_PROBE_USE_MEDIAN 0
#define Z_PROBE_HEIGHT 40
#define Z_PROBE_DELAY 0
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""
#define Z_PROBE_RUN_AFTER_EVERY_PROBE ""
#define Z_PROBE_REQUIRES_HEATING 0
#define Z_PROBE_MIN_TEMPERATURE 150
#define FEATURE_AUTOLEVEL 0
#define FEATURE_SOFTWARE_LEVELING 0
#define Z_PROBE_X1 20
#define Z_PROBE_Y1 20
#define Z_PROBE_X2 160
#define Z_PROBE_Y2 20
#define Z_PROBE_X3 100
#define Z_PROBE_Y3 160
#define BED_LEVELING_METHOD 0
#define BED_CORRECTION_METHOD 0
#define BED_LEVELING_GRID_SIZE 5
#define BED_LEVELING_REPETITIONS 5
#define BED_MOTOR_1_X 0
#define BED_MOTOR_1_Y 0
#define BED_MOTOR_2_X 200
#define BED_MOTOR_2_Y 0
#define BED_MOTOR_3_X 100
#define BED_MOTOR_3_Y 200
#define BENDING_CORRECTION_A 0
#define BENDING_CORRECTION_B 0
#define BENDING_CORRECTION_C 0
#define FEATURE_AXISCOMP 0
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0

#ifndef SDSUPPORT  // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT 1
#undef SDCARDDETECT
#define SDCARDDETECT ORIG_SDCARDDETECT
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0
#endif
#define SD_EXTENDED_DIR 1 /** Show extended directory including file length. Don't use this with Pronterface! */
#define SD_RUN_ON_STOP ""
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 1
#define ARC_SUPPORT 1
#define FEATURE_MEMORY_POSITION 1
#define FEATURE_CHECKSUM_FORCED 0
#define FEATURE_FAN_CONTROL 1
#define FEATURE_FAN2_CONTROL 0
#define FEATURE_CONTROLLER 21
#define ADC_KEYPAD_PIN -1
#define LANGUAGE_EN_ACTIVE 1
#define LANGUAGE_DE_ACTIVE 1
#define LANGUAGE_NL_ACTIVE 0
#define LANGUAGE_PT_ACTIVE 1
#define LANGUAGE_IT_ACTIVE 1
#define LANGUAGE_ES_ACTIVE 1
#define LANGUAGE_FI_ACTIVE 0
#define LANGUAGE_SE_ACTIVE 0
#define LANGUAGE_FR_ACTIVE 1
#define LANGUAGE_CZ_ACTIVE 0
#define LANGUAGE_PL_ACTIVE 0
#define LANGUAGE_TR_ACTIVE 0
#define LANGUAGE_RU_ACTIVE 0
#define UI_PRINTER_NAME "Stacker 500"
#define UI_PRINTER_COMPANY "Stacker LLC"
#define UI_PAGES_DURATION 4000
#define UI_SPEEDDEPENDENT_POSITIONING 0
#define UI_DISABLE_AUTO_PAGESWITCH 1
#define UI_AUTORETURN_TO_MENU_AFTER 30000
#define FEATURE_UI_KEYS 0
#define UI_ENCODER_SPEED 1
#define UI_REVERSE_ENCODER 0
#define UI_KEY_BOUNCETIME 10
#define UI_KEY_FIRST_REPEAT 500
#define UI_KEY_REDUCE_REPEAT 50
#define UI_KEY_MIN_REPEAT 50
#define FEATURE_BEEPER 1
#define CASE_LIGHTS_PIN -1
#define CASE_LIGHT_DEFAULT_ON 1
#define UI_START_SCREEN_DELAY 1000
#define UI_DYNAMIC_ENCODER_SPEED 1
        /**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 2,2
#define BEEPER_LONG_SEQUENCE 8,8
#define UI_SET_MIN_HEATED_BED_TEMP  30
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP   170
#define UI_SET_MAX_EXTRUDER_TEMP   280
#define UI_SET_EXTRUDER_FEEDRATE 2
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3


#define NUM_MOTOR_DRIVERS 0

#define DEBUG_ECHO_ASCII


#define MACHINE_TYPE "STACKER"
#define ALTERNATIVE_JERK
#define REDUCE_ON_SMALL_SEGMENTS
#define MAX_JERK_DISTANCE 0.8 
#define ZHOME_WAIT_UNSWING 500
#define BIG_OUTPUT_BUFFER // allows complete temperature line as output
#define CUSTOM_LOGO
#define LOGO_WIDTH 128
#define LOGO_HEIGHT 13
#define LOGO_BITMAP const unsigned char logo[] PROGMEM = {\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
0x0F, 0xFF, 0xF3, 0xFF, 0xF9, 0xFF, 0xFE, 0x7F, 0xFF, 0x9E, 0x0F, 0x87, 0xFF, 0xF7, 0xFF, 0xFC,\
0x1F, 0xFF, 0xF3, 0xFF, 0xF3, 0xFF, 0xFC, 0xFF, 0xFF, 0x3C, 0x1F, 0x07, 0xFF, 0xE7, 0xFF, 0xF8,\
0x1F, 0xFF, 0xE7, 0xFF, 0xF3, 0xFF, 0xFC, 0xFF, 0xFF, 0x3C, 0x3E, 0x0F, 0xFF, 0xEF, 0xFF, 0xF8,\
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00,\
0x38, 0x00, 0x00, 0x3C, 0x07, 0x80, 0x79, 0xE0, 0x00, 0x78, 0x00, 0x0E, 0x00, 0x0F, 0x00, 0xF8,\
0x3F, 0xFF, 0x80, 0x3C, 0x07, 0x3F, 0xF9, 0xE0, 0x00, 0x7B, 0xF8, 0x1F, 0xFF, 0x8F, 0x01, 0xF0,\
0x3F, 0xFF, 0xC0, 0x78, 0x0F, 0x7F, 0xF9, 0xC0, 0x00, 0x73, 0xF8, 0x1F, 0xFF, 0x8E, 0x7F, 0xE0,\
0x00, 0x03, 0xC0, 0x78, 0x0E, 0x00, 0x73, 0xC0, 0x00, 0xF0, 0x7C, 0x1E, 0x00, 0x1E, 0xFF, 0xC0,\
0x00, 0x03, 0xC0, 0x70, 0x0E, 0x00, 0xF3, 0xC0, 0x00, 0xF0, 0x3E, 0x3C, 0x00, 0x1E, 0xFF, 0xC0,\
0x7F, 0xFF, 0x80, 0xF0, 0x1E, 0x00, 0xF3, 0xFF, 0xFC, 0xF0, 0x1E, 0x3F, 0xFF, 0x9C, 0x07, 0xC0,\
0xFF, 0xFF, 0x00, 0xF0, 0x1E, 0x01, 0xE7, 0xFF, 0xFD, 0xE0, 0x1F, 0x3F, 0xFF, 0xBC, 0x03, 0xE0,\
0xFF, 0xFE, 0x01, 0xE0, 0x3E, 0x01, 0xE7, 0xFF, 0xF9, 0xE0, 0x0F, 0xBF, 0xFF, 0x3C, 0x01, 0xF0\
};

#if 0
Informations:

Extruder temperatures use a user defined table for ATC 104GT sensor. Here a adjustment to
real temperatures in nozzle is included. So in deviation to original temperature table these
temperatures are used:
-50 - 30: Original T
30-260: T-(T-30)*15/230
260-300: T-15

New table stops now at 285°C so max. settable temperature is 280°C
#endif

#endif

/* Below you will find the configuration string, that created this Configuration.h

========== Start configuration string ==========
{
    "editMode": 2,
    "processor": 0,
    "baudrate": 250000,
    "bluetoothSerial": -1,
    "bluetoothBaudrate": 115200,
    "xStepsPerMM": 72.72,
    "yStepsPerMM": 72.72,
    "zStepsPerMM": 520,
    "xInvert": "0",
    "x2Invert": 0,
    "xInvertEnable": "1",
    "eepromMode": 1,
    "yInvert": "1",
    "y2Invert": 0,
    "yInvertEnable": 0,
    "zInvert": "0",
    "z2Invert": 0,
    "z3Invert": 0,
    "z4Invert": 0,
    "zInvertEnable": "1",
    "extruder": [
        {
            "id": 0,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 255,
            "pidMax": 255,
            "sensorType": 5,
            "sensorPin": "TEMP_0_PIN",
            "heaterPin": "HEATER_0_PIN",
            "maxFeedrate": 50,
            "startFeedrate": 20,
            "invert": "1",
            "invertEnable": "1",
            "acceleration": 5000,
            "watchPeriod": 1,
            "pidP": 8.94,
            "pidI": 0.51,
            "pidD": 20,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 255,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "zOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 0",
                "step": "ORIG_E0_STEP_PIN",
                "dir": "ORIG_E0_DIR_PIN",
                "enable": "ORIG_E0_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 30,
            "jamPin": -1,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 0",
                "step": "ORIG_E0_STEP_PIN",
                "dir": "ORIG_E0_DIR_PIN",
                "enable": "ORIG_E0_ENABLE_PIN"
            },
            "preheat": 190
        },
        {
            "id": 1,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 255,
            "pidMax": 255,
            "sensorType": 5,
            "sensorPin": "TEMP_2_PIN",
            "heaterPin": "HEATER_2_PIN",
            "maxFeedrate": 100,
            "startFeedrate": 40,
            "invert": "1",
            "invertEnable": "1",
            "acceleration": 5000,
            "watchPeriod": 1,
            "pidP": 8.94,
            "pidI": 0.51,
            "pidD": 20,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 257,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "zOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 1",
                "step": "ORIG_E1_STEP_PIN",
                "dir": "ORIG_E1_DIR_PIN",
                "enable": "ORIG_E1_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 30,
            "jamPin": -1,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 1",
                "step": "ORIG_E1_STEP_PIN",
                "dir": "ORIG_E1_DIR_PIN",
                "enable": "ORIG_E1_ENABLE_PIN"
            },
            "preheat": 190
        },
        {
            "id": 2,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 255,
            "pidMax": 255,
            "sensorType": 5,
            "sensorPin": "TEMP_3_PIN",
            "heaterPin": "HEATER_3_PIN",
            "maxFeedrate": 100,
            "startFeedrate": 40,
            "invert": "1",
            "invertEnable": "1",
            "acceleration": 5000,
            "watchPeriod": 1,
            "pidP": 8.94,
            "pidI": 0.51,
            "pidD": 20,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 257,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "zOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 2",
                "step": "ORIG_E2_STEP_PIN",
                "dir": "ORIG_E2_DIR_PIN",
                "enable": "ORIG_E2_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 30,
            "jamPin": -1,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 2",
                "step": "ORIG_E2_STEP_PIN",
                "dir": "ORIG_E2_DIR_PIN",
                "enable": "ORIG_E2_ENABLE_PIN"
            },
            "preheat": 190
        },
        {
            "id": 3,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 255,
            "pidMax": 255,
            "sensorType": 5,
            "sensorPin": "TEMP_4_PIN",
            "heaterPin": "HEATER_4_PIN",
            "maxFeedrate": 100,
            "startFeedrate": 40,
            "invert": "1",
            "invertEnable": "1",
            "acceleration": 5000,
            "watchPeriod": 1,
            "pidP": 8.94,
            "pidI": 0.51,
            "pidD": 20,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 257,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "zOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 3",
                "step": "ORIG_E3_STEP_PIN",
                "dir": "ORIG_E3_DIR_PIN",
                "enable": "ORIG_E3_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 30,
            "jamPin": -1,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 3",
                "step": "ORIG_E3_STEP_PIN",
                "dir": "ORIG_E3_DIR_PIN",
                "enable": "ORIG_E3_ENABLE_PIN"
            },
            "preheat": 190
        }
    ],
    "uiLanguage": 0,
    "uiController": 0,
    "xMinEndstop": 1,
    "yMinEndstop": 1,
    "zMinEndstop": 1,
    "xMaxEndstop": 1,
    "yMaxEndstop": 1,
    "zMaxEndstop": 1,
    "x2MinEndstop": 0,
    "y2MinEndstop": 0,
    "x2MaxEndstop": 0,
    "y2MaxEndstop": 0,
    "motherboard": 35,
    "driveSystem": 0,
    "xMaxSpeed": 200,
    "xHomingSpeed": 40,
    "xTravelAcceleration": 1000,
    "xPrintAcceleration": 1000,
    "yMaxSpeed": 200,
    "yHomingSpeed": 40,
    "yTravelAcceleration": 1000,
    "yPrintAcceleration": 1000,
    "zMaxSpeed": 80,
    "zHomingSpeed": 40,
    "zTravelAcceleration": 50,
    "zPrintAcceleration": 50,
    "xMotor": {
        "name": "X motor",
        "step": "ORIG_X_STEP_PIN",
        "dir": "ORIG_X_DIR_PIN",
        "enable": "ORIG_X_ENABLE_PIN"
    },
    "yMotor": {
        "name": "Y motor",
        "step": "ORIG_Y_STEP_PIN",
        "dir": "ORIG_Y_DIR_PIN",
        "enable": "ORIG_Y_ENABLE_PIN"
    },
    "zMotor": {
        "name": "Z motor",
        "step": "ORIG_Z_STEP_PIN",
        "dir": "ORIG_Z_DIR_PIN",
        "enable": "ORIG_Z_ENABLE_PIN"
    },
    "enableBacklash": "1",
    "backlashX": 0,
    "backlashY": 0,
    "backlashZ": 0,
    "stepperInactiveTime": 360,
    "maxInactiveTime": 0,
    "xMinPos": 0,
    "yMinPos": 0,
    "zMinPos": 0,
    "xLength": 418,
    "yLength": 240,
    "zLength": 210,
    "alwaysCheckEndstops": "1",
    "disableX": "0",
    "disableY": "0",
    "disableZ": "0",
    "disableE": "0",
    "xHomeDir": "-1",
    "yHomeDir": "-1",
    "zHomeDir": "-1",
    "xEndstopBack": 1,
    "yEndstopBack": 1,
    "zEndstopBack": 0,
    "deltaSegmentsPerSecondPrint": 180,
    "deltaSegmentsPerSecondTravel": 70,
    "deltaDiagonalRod": 445,
    "deltaHorizontalRadius": 209.25,
    "deltaAlphaA": 210,
    "deltaAlphaB": 330,
    "deltaAlphaC": 90,
    "deltaDiagonalCorrA": 0,
    "deltaDiagonalCorrB": 0,
    "deltaDiagonalCorrC": 0,
    "deltaMaxRadius": 150,
    "deltaFloorSafetyMarginMM": 15,
    "deltaRadiusCorrA": 0,
    "deltaRadiusCorrB": 0,
    "deltaRadiusCorrC": 0,
    "deltaXOffsetSteps": 0,
    "deltaYOffsetSteps": 0,
    "deltaZOffsetSteps": 0,
    "deltaSegmentsPerLine": 24,
    "stepperHighDelay": 0,
    "directionDelay": 0,
    "stepDoublerFrequency": 12000,
    "allowQuadstepping": "1",
    "doubleStepDelay": 1,
    "maxJerk": 25,
    "maxZJerk": 0.3,
    "moveCacheSize": 24,
    "moveCacheLow": 10,
    "lowTicksPerMove": 250000,
    "enablePowerOnStartup": "1",
    "echoOnExecute": "1",
    "sendWaits": "1",
    "ackWithLineNumber": "1",
    "killMethod": 1,
    "useAdvance": "1",
    "useQuadraticAdvance": "0",
    "powerInverting": 0,
    "mirrorX": 0,
    "mirrorXMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorY": 0,
    "mirrorYMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorZ": 0,
    "mirrorZMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorZ3": "0",
    "mirrorZ3Motor": {
        "name": "Extruder 2",
        "step": "ORIG_E2_STEP_PIN",
        "dir": "ORIG_E2_DIR_PIN",
        "enable": "ORIG_E2_ENABLE_PIN"
    },
    "mirrorZ4": "0",
    "mirrorZ4Motor": {
        "name": "Extruder 3",
        "step": "ORIG_E3_STEP_PIN",
        "dir": "ORIG_E3_DIR_PIN",
        "enable": "ORIG_E3_ENABLE_PIN"
    },
    "dittoPrinting": "1",
    "featureServos": "0",
    "servo0Pin": 11,
    "servo1Pin": -1,
    "servo2Pin": -1,
    "servo3Pin": -1,
    "featureWatchdog": "1",
    "hasHeatedBed": "1",
    "enableZProbing": "0",
    "extrudeMaxLength": 160,
    "homeOrder": "HOME_ORDER_XYZ",
    "featureController": 21,
    "uiPrinterName": "Stacker 500",
    "uiPrinterCompany": "Stacker LLC",
    "uiPagesDuration": 4000,
    "uiHeadline": "",
    "uiDisablePageswitch": "1",
    "uiAutoReturnAfter": 30000,
    "featureKeys": "0",
    "uiEncoderSpeed": 1,
    "uiReverseEncoder": "0",
    "uiKeyBouncetime": 10,
    "uiKeyFirstRepeat": 500,
    "uiKeyReduceRepeat": 50,
    "uiKeyMinRepeat": 50,
    "featureBeeper": "1",
    "uiMinHeatedBed": 30,
    "uiMaxHeatedBed": 120,
    "uiMinEtxruderTemp": 170,
    "uiMaxExtruderTemp": 280,
    "uiExtruderFeedrate": 2,
    "uiExtruderRetractDistance": 3,
    "uiSpeeddependentPositioning": "0",
    "maxBedTemperature": 120,
    "bedSensorType": 8,
    "bedSensorPin": "TEMP_1_PIN",
    "bedHeaterPin": "HEATER_1_PIN",
    "bedHeatManager": 1,
    "bedPreheat": 55,
    "bedUpdateInterval": 5000,
    "bedPidDriveMin": 80,
    "bedPidDriveMax": 255,
    "bedPidP": 196,
    "bedPidI": 33,
    "bedPidD": 290,
    "bedPidMax": 255,
    "bedDecoupleTestPeriod": 300,
    "caseLightPin": -1,
    "caseLightDefaultOn": "1",
    "bedSkipIfWithin": 3,
    "gen1T0": 25,
    "gen1R0": 100000,
    "gen1Beta": 4036,
    "gen1MinTemp": -20,
    "gen1MaxTemp": 300,
    "gen1R1": 0,
    "gen1R2": 4700,
    "gen2T0": 25,
    "gen2R0": 100000,
    "gen2Beta": 4036,
    "gen2MinTemp": -20,
    "gen2MaxTemp": 300,
    "gen2R1": 0,
    "gen2R2": 4700,
    "gen3T0": 25,
    "gen3R0": 100000,
    "gen3Beta": 4036,
    "gen3MinTemp": -20,
    "gen3MaxTemp": 300,
    "gen3R1": 0,
    "gen3R2": 4700,
    "userTable0": {
        "r1": 0,
        "r2": 4700,
        "temps": [
            {
                "t": 1000,
                "r": 0,
                "adc": 0
            },
            {
                "t": 285,
                "r": 80.65,
                "adc": 69.0830221832
            },
            {
                "t": 275,
                "r": 93,
                "adc": 79.4564990611
            },
            {
                "t": 265,
                "r": 107.9,
                "adc": 91.9009338797
            },
            {
                "t": 255,
                "r": 125.8,
                "adc": 106.749347258
            },
            {
                "t": 245,
                "r": 147.5,
                "adc": 124.602888087
            },
            {
                "t": 235.7,
                "r": 174,
                "adc": 146.18998769
            },
            {
                "t": 226.3,
                "r": 206.5,
                "adc": 172.346377255
            },
            {
                "t": 217,
                "r": 246.8,
                "adc": 204.302983747
            },
            {
                "t": 207.6,
                "r": 296.9,
                "adc": 243.311953411
            },
            {
                "t": 198.3,
                "r": 359.7,
                "adc": 291.118346938
            },
            {
                "t": 188.9,
                "r": 439.3,
                "adc": 350.034732357
            },
            {
                "t": 179.6,
                "r": 540.6,
                "adc": 422.424340724
            },
            {
                "t": 170.2,
                "r": 671.4,
                "adc": 511.855940723
            },
            {
                "t": 160.9,
                "r": 841.4,
                "adc": 621.780236041
            },
            {
                "t": 151.5,
                "r": 1064,
                "adc": 755.912560722
            },
            {
                "t": 142.2,
                "r": 1360,
                "adc": 919.00990099
            },
            {
                "t": 123.5,
                "r": 2298,
                "adc": 1344.71420406
            },
            {
                "t": 67.4,
                "r": 15440,
                "adc": 3139.36444886
            },
            {
                "t": 58,
                "r": 22510,
                "adc": 3387.66813671
            },
            {
                "t": 48.7,
                "r": 33490,
                "adc": 3591.03299293
            },
            {
                "t": 39.3,
                "r": 50960,
                "adc": 3749.21307941
            },
            {
                "t": 30,
                "r": 79360,
                "adc": 3866.0385439
            },
            {
                "t": 20,
                "r": 126800,
                "adc": 3948.63878327
            },
            {
                "t": 10,
                "r": 208600,
                "adc": 4004.76793249
            },
            {
                "t": 0,
                "r": 353700,
                "adc": 4041.29882812
            },
            {
                "t": -10,
                "r": 620000,
                "adc": 4064.19081159
            },
            {
                "t": -30,
                "r": 2132000,
                "adc": 4085.99241822
            },
            {
                "t": -50,
                "r": 8743000,
                "adc": 4092.79982167
            }
        ],
        "numEntries": 29
    },
    "userTable1": {
        "r1": 0,
        "r2": 4700,
        "temps": [],
        "numEntries": 0
    },
    "userTable2": {
        "r1": 0,
        "r2": 4700,
        "temps": [],
        "numEntries": 0
    },
    "tempHysteresis": 0,
    "pidControlRange": 20,
    "skipM109Within": 2,
    "extruderFanCoolTemp": 50,
    "minTemp": 150,
    "maxTemp": 280,
    "minDefectTemp": -10,
    "maxDefectTemp": 300,
    "arcSupport": "1",
    "featureMemoryPositionWatchdog": "1",
    "forceChecksum": "0",
    "sdExtendedDir": "1",
    "featureFanControl": "1",
    "fanPin": "ORIG_FAN_PIN",
    "featureFan2Control": "0",
    "fan2Pin": "ORIG_FAN2_PIN",
    "fanThermoPin": -1,
    "fanThermoMinPWM": 128,
    "fanThermoMaxPWM": 255,
    "fanThermoMinTemp": 45,
    "fanThermoMaxTemp": 60,
    "fanThermoThermistorPin": -1,
    "fanThermoThermistorType": 1,
    "scalePidToMax": 0,
    "zProbePin": -1,
    "zProbeBedDistance": 10,
    "zProbeDisableHeaters": "0",
    "zProbePullup": "0",
    "zProbeOnHigh": "0",
    "zProbeXOffset": 0,
    "zProbeYOffset": 0,
    "zProbeWaitBeforeTest": "0",
    "zProbeSpeed": 2,
    "zProbeXYSpeed": 150,
    "zProbeHeight": 40,
    "zProbeStartScript": "",
    "zProbeFinishedScript": "",
    "featureAutolevel": "0",
    "zProbeX1": 20,
    "zProbeY1": 20,
    "zProbeX2": 160,
    "zProbeY2": 20,
    "zProbeX3": 100,
    "zProbeY3": 160,
    "zProbeSwitchingDistance": 1,
    "zProbeRepetitions": 1,
    "zProbeMedian": "0",
    "zProbeEveryPoint": "",
    "sdSupport": "1",
    "sdCardDetectPin": "ORIG_SDCARDDETECT",
    "sdCardDetectInverted": "0",
    "uiStartScreenDelay": 1000,
    "xEndstopBackMove": 6,
    "yEndstopBackMove": 6,
    "zEndstopBackMove": 6,
    "xEndstopRetestFactor": 3,
    "yEndstopRetestFactor": 3,
    "zEndstopRetestFactor": 6,
    "xMinPin": "ORIG_X_MIN_PIN",
    "yMinPin": "ORIG_Y_MIN_PIN",
    "zMinPin": "ORIG_Z_MIN_PIN",
    "xMaxPin": "ORIG_X_MAX_PIN",
    "yMaxPin": "ORIG_Y_MAX_PIN",
    "zMaxPin": "ORIG_Z_MAX_PIN",
    "x2MinPin": -1,
    "y2MinPin": -1,
    "x2MaxPin": -1,
    "y2MaxPin": -1,
    "deltaHomeOnPower": "0",
    "fanBoardPin": "ORIG_FAN_PIN",
    "heaterPWMSpeed": 0,
    "featureBabystepping": "1",
    "babystepMultiplicator": 1,
    "pdmForHeater": "0",
    "pdmForCooler": "0",
    "psOn": -1,
    "mixingExtruder": "0",
    "decouplingTestMaxHoldVariance": 20,
    "decouplingTestMinTempRise": 1,
    "featureAxisComp": "0",
    "axisCompTanXY": 0,
    "axisCompTanXZ": 0,
    "axisCompTanYZ": 0,
    "retractOnPause": 4,
    "pauseStartCommands": "",
    "pauseEndCommands": "",
    "distortionCorrection": "0",
    "distortionCorrectionPoints": 5,
    "distortionCorrectionR": 100,
    "distortionPermanent": "1",
    "distortionUpdateFrequency": 15,
    "distortionStartDegrade": 0.5,
    "distortionEndDegrade": 1,
    "distortionExtrapolateCorners": "0",
    "distortionXMin": 10,
    "distortionXMax": 190,
    "distortionYMin": 10,
    "distortionYMax": 190,
    "sdRunOnStop": "",
    "sdStopHeaterMotorsOnStop": "1",
    "featureRetraction": "1",
    "autoretractEnabled": "0",
    "retractionLength": 2.5,
    "retractionLongLength": 2.5,
    "retractionSpeed": 50,
    "retractionZLift": 0,
    "retractionUndoExtraLength": 0,
    "retractionUndoExtraLongLength": 0,
    "retractionUndoSpeed": 40,
    "filamentChangeXPos": 0,
    "filamentChangeYPos": 0,
    "filamentChangeZAdd": 2,
    "filamentChangeRehome": 1,
    "filamentChangeShortRetract": 5,
    "filamentChangeLongRetract": 50,
    "fanKickstart": 200,
    "servo0StartPos": -1,
    "servo1StartPos": -1,
    "servo2StartPos": -1,
    "servo3StartPos": -1,
    "uiDynamicEncoderSpeed": "1",
    "uiServoControl": 0,
    "killIfSensorDefect": "1",
    "jamSteps": 220,
    "jamSlowdownSteps": 320,
    "jamSlowdownTo": 70,
    "jamErrorSteps": 500,
    "jamMinSteps": 10,
    "jamAction": 1,
    "jamMethod": 1,
    "primaryPort": 0,
    "numMotorDrivers": 0,
    "motorDrivers": [
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        }
    ],
    "manualConfig": "#define DEBUG_ECHO_ASCII\n\n\n#define MACHINE_TYPE \"STACKER\"\n#define ALTERNATIVE_JERK\n#define REDUCE_ON_SMALL_SEGMENTS\n#define MAX_JERK_DISTANCE 0.8 \n#define ZHOME_WAIT_UNSWING 500\n#define BIG_OUTPUT_BUFFER \/\/ allows complete temperature line as output\n#define CUSTOM_LOGO\n#define LOGO_WIDTH 128\n#define LOGO_HEIGHT 13\n#define LOGO_BITMAP const unsigned char logo[] PROGMEM = {\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x0F, 0xFF, 0xF3, 0xFF, 0xF9, 0xFF, 0xFE, 0x7F, 0xFF, 0x9E, 0x0F, 0x87, 0xFF, 0xF7, 0xFF, 0xFC,\\\n0x1F, 0xFF, 0xF3, 0xFF, 0xF3, 0xFF, 0xFC, 0xFF, 0xFF, 0x3C, 0x1F, 0x07, 0xFF, 0xE7, 0xFF, 0xF8,\\\n0x1F, 0xFF, 0xE7, 0xFF, 0xF3, 0xFF, 0xFC, 0xFF, 0xFF, 0x3C, 0x3E, 0x0F, 0xFF, 0xEF, 0xFF, 0xF8,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x38, 0x00, 0x00, 0x3C, 0x07, 0x80, 0x79, 0xE0, 0x00, 0x78, 0x00, 0x0E, 0x00, 0x0F, 0x00, 0xF8,\\\n0x3F, 0xFF, 0x80, 0x3C, 0x07, 0x3F, 0xF9, 0xE0, 0x00, 0x7B, 0xF8, 0x1F, 0xFF, 0x8F, 0x01, 0xF0,\\\n0x3F, 0xFF, 0xC0, 0x78, 0x0F, 0x7F, 0xF9, 0xC0, 0x00, 0x73, 0xF8, 0x1F, 0xFF, 0x8E, 0x7F, 0xE0,\\\n0x00, 0x03, 0xC0, 0x78, 0x0E, 0x00, 0x73, 0xC0, 0x00, 0xF0, 0x7C, 0x1E, 0x00, 0x1E, 0xFF, 0xC0,\\\n0x00, 0x03, 0xC0, 0x70, 0x0E, 0x00, 0xF3, 0xC0, 0x00, 0xF0, 0x3E, 0x3C, 0x00, 0x1E, 0xFF, 0xC0,\\\n0x7F, 0xFF, 0x80, 0xF0, 0x1E, 0x00, 0xF3, 0xFF, 0xFC, 0xF0, 0x1E, 0x3F, 0xFF, 0x9C, 0x07, 0xC0,\\\n0xFF, 0xFF, 0x00, 0xF0, 0x1E, 0x01, 0xE7, 0xFF, 0xFD, 0xE0, 0x1F, 0x3F, 0xFF, 0xBC, 0x03, 0xE0,\\\n0xFF, 0xFE, 0x01, 0xE0, 0x3E, 0x01, 0xE7, 0xFF, 0xF9, 0xE0, 0x0F, 0xBF, 0xFF, 0x3C, 0x01, 0xF0\\\n};\n\n#if 0\nInformations:\n\nExtruder temperatures use a user defined table for ATC 104GT sensor. Here a adjustment to\nreal temperatures in nozzle is included. So in deviation to original temperature table these\ntemperatures are used:\n-50 - 30: Original T\n30-260: T-(T-30)*15\/230\n260-300: T-15\n\nNew table stops now at 285\u00b0C so max. settable temperature is 280\u00b0C\n#endif",
    "zHomeMinTemperature": 0,
    "zHomeXPos": 999999,
    "zHomeYPos": 999999,
    "zHomeHeatHeight": 20,
    "zHomeHeatAll": "1",
    "zProbeZOffsetMode": 0,
    "zProbeZOffset": 0,
    "zProbeDelay": 0,
    "uiBedCoating": "0",
    "langEN": "1",
    "langDE": "1",
    "langNL": "0",
    "langPT": "1",
    "langIT": "1",
    "langES": "1",
    "langFI": "0",
    "langSE": "0",
    "langFR": "1",
    "langCZ": "0",
    "langPL": "0",
    "langTR": "0",
    "langRU": "0",
    "interpolateAccelerationWithZ": 0,
    "accelerationFactorTop": 100,
    "bendingCorrectionA": 0,
    "bendingCorrectionB": 0,
    "bendingCorrectionC": 0,
    "preventZDisableOnStepperTimeout": "1",
    "supportLaser": "0",
    "laserPin": -1,
    "laserOnHigh": "1",
    "laserWarmupTime": 0,
    "defaultPrinterMode": 0,
    "laserPwmMax": 255,
    "laserWatt": 2,
    "supportCNC": "0",
    "cncWaitOnEnable": 300,
    "cncWaitOnDisable": 0,
    "cncEnablePin": -1,
    "cncEnableWith": "1",
    "cncDirectionPin": -1,
    "cncDirectionCW": "1",
    "cncPwmMax": 255,
    "cncRpmMax": 8000,
    "cncSafeZ": 150,
    "startupGCode": "",
    "jsonOutput": "1",
    "bedLevelingMethod": 0,
    "bedCorrectionMethod": 0,
    "bedLevelingGridSize": 5,
    "bedLevelingRepetitions": 5,
    "bedMotor1X": 0,
    "bedMotor1Y": 0,
    "bedMotor2X": 200,
    "bedMotor2Y": 0,
    "bedMotor3X": 100,
    "bedMotor3Y": 200,
    "zProbeRequiresHeating": "0",
    "zProbeMinTemperature": 150,
    "adcKeypadPin": -1,
    "sharedExtruderHeater": "0",
    "extruderSwitchXYSpeed": 100,
    "dualXAxis": "0",
    "boardFanSpeed": 255,
    "keepAliveInterval": 2000,
    "moveXWhenHomed": "0",
    "moveYWhenHomed": "0",
    "moveZWhenHomed": "0",
    "preheatTime": 30000,
    "multiZEndstopHoming": "0",
    "z2MinMaxPin": -1,
    "z2MinMaxEndstop": 0,
    "extruderIsZProbe": "0",
    "boardFanMinSpeed": 0,
    "doorPin": -1,
    "doorEndstop": 1,
    "zhomePreRaise": 0,
    "zhomePreRaiseDistance": 10,
    "dualXResolution": "0",
    "x2axisStepsPerMM": 100,
    "coolerPWMSpeed": 0,
    "maxFanPWM": 255,
    "raiseZOnToolchange": 0,
    "distortionLimitTo": 2,
    "automaticPowerup": 0,
    "hasTMC2130": "0",
    "TMC2130Sensorless": "0",
    "TMC2130Steathchop": "1",
    "TMC2130Interpolate256": "1",
    "TMC2130StallguardSensitivity": 0,
    "TMC2130PWMAmpl": 255,
    "TMC2130PWMGrad": 1,
    "TMC2130PWMAutoscale": "1",
    "TMC2130PWMFreq": 2,
    "TMC2130CSX": -1,
    "TMC2130CSY": -1,
    "TMC2130CSZ": -1,
    "TMC2130CSE0": -1,
    "TMC2130CSE1": -1,
    "TMC2130CSE2": -1,
    "TMC2130CurrentX": 1000,
    "TMC2130CurrentY": 1000,
    "TMC2130CurrentZ": 1000,
    "TMC2130CurrentE0": 1000,
    "TMC2130CurrentE1": 1000,
    "TMC2130CurrentE2": 1000,
    "TMC2130CoolstepTresholdX": 300,
    "TMC2130CoolstepTresholdY": 300,
    "TMC2130CoolstepTresholdZ": 300,
    "microstepX": 16,
    "microstepY": 16,
    "microstepZ": 16,
    "microstepE0": 16,
    "microstepE1": 16,
    "microstepE2": 16,
    "parkPosX": 0,
    "parkPosY": 200,
    "parkPosZ": 10,
    "emergencyParser": 1,
    "hostRescue": "1",
    "MAX31855SwCS": -1,
    "MAX31855SwCLK": -1,
    "tempGain": "1",
    "uiAnimation": "0",
    "uiPresetBedTempPLA": 60,
    "uiPresetBedABS": 110,
    "uiPresetExtruderPLA": 190,
    "uiPresetExtruderABS": 240,
    "interpolateZAxis": "1",
    "zAccelerationTop": 30,
    "maxHalfstepInterval": 1999,
    "hasMAX6675": false,
    "hasMAX31855": false,
    "hasGeneric1": false,
    "hasGeneric2": false,
    "hasGeneric3": false,
    "hasUser0": true,
    "hasUser1": false,
    "hasUser2": false,
    "numExtruder": 4,
    "version": 100.4,
    "primaryPortName": "",
    "hasMAX31855SW": false
}
========== End configuration string ==========

*/
