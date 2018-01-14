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

   This file is optimized for version 1.0.0dev
   generator: http://www.repetier.com/firmware/dev/

   If you are in doubt which named functions use which pins on your board, please check the
   pins.h for the used name->pin assignments and your board documentation to verify it is
   as you expect.

*/

// The follwing variables are required early to decide on the right modules.
#define NUM_EXTRUDER 2
#define MOTHERBOARD 405 // 405
#define EEPROM_MODE 2
#define RFSERIAL Serial
#define WAITING_IDENTIFIER "wait"
#define JSON_OUTPUT 1
#define FEATURE_SERVO 1
#define FEATURE_WATCHDOG 0
#define FEATURE_Z_PROBE 0
#define FEATURE_RETRACTION 1
#define DISTORTION_CORRECTION 0
#define USE_ADVANCE 1
#define ENABLE_QUADRATIC_ADVANCE 1
#define NUM_AXES 4 // X,Y,Z and E for extruder A,B,C would be 5,6,7
#define STEPPER_FREQUENCY 200000
#define PREPARE_FREQUENCY 600
#define BLOCK_FREQUENCY 250
#define VELOCITY_PROFILE 5 // 1 = linear, 3 = cubic, 5 = quintic velocity shape

// Next 7 lines are required to make the following work, do not change!
#include "pins.h"
#undef IO_TARGET
#define IO_TARGET 4
#undef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#include "src/drivers/drivers.h"
#include "src/io/redefine.h"

// Define ZProbe by referencing a endstop defined
CONFIG_VARIABLE_EQ(EndstopDriver, *ZProbe, &endstopZMin)

// Define xyz motors

#define XMOTOR_TYPE SimpleStepperDriver<IOX1Step, IOX1Dir, IOX1Enable>
#define XMOTOR_PARAMS (&endstopXMin, &endstopNone)
CONFIG_VARIABLE(XMOTOR_TYPE, XMotor, XMOTOR_PARAMS)
#define YMOTOR_TYPE SimpleStepperDriver<IOY1Step, IOY1Dir, IOY1Enable>
#define YMOTOR_PARAMS (&endstopNone, &endstopYMax)
CONFIG_VARIABLE(YMOTOR_TYPE, YMotor, YMOTOR_PARAMS)
#define ZMOTOR_TYPE SimpleStepperDriver<IOZ1Step, IOZ1Dir, IOZ1Enable>
#define ZMOTOR_PARAMS (&endstopZMin, &endstopNone)
CONFIG_VARIABLE(ZMOTOR_TYPE, ZMotor, ZMOTOR_PARAMS)
#define E0MOTOR_TYPE SimpleStepperDriver<IOE0Step, IOE0Dir, IOE0Enable>
#define E0MOTOR_PARAMS (&endstopZMin, &endstopNone)
CONFIG_VARIABLE(E0MOTOR_TYPE, E0Motor, E0MOTOR_PARAMS)

/** Axes are homed in order of priority (0..10) if homing direction is not 0. */
#define X_HOME_PRIORITY 0
#define Y_HOME_PRIORITY 1
#define Z_HOME_PRIORITY 2

// All fans in this list list become controllable with M106/M107
// by selecteing the fan number with P0..P<NUM_FANS-1>
#define NUM_FANS 1
#define FAN_LIST {&Fan1PWM}

// ################## EDIT THESE SETTINGS MANUALLY ################
// ################ END MANUAL SETTINGS ##########################

#undef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#define BOARD_FAN_SPEED 255
#define BOARD_FAN_MIN_SPEED 0
#define FAN_THERMO_PIN -1
#define FAN_THERMO_MIN_PWM 128
#define FAN_THERMO_MAX_PWM 255
#define FAN_THERMO_MIN_TEMP 45
#define FAN_THERMO_MAX_TEMP 60
#define FAN_THERMO_THERMISTOR_PIN -1
#define FAN_THERMO_THERMISTOR_TYPE 1
#undef Y_MIN_PIN
#define Y_MIN_PIN -1
#undef X_MAX_PIN
#define X_MAX_PIN -1
#undef Y_MAX_PIN
#define Y_MAX_PIN ORIG_Y_MIN_PIN
#undef Z_MAX_PIN
#define Z_MAX_PIN -1

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1
#define BLUETOOTH_SERIAL -1
#define BLUETOOTH_BAUD 250000
#define MIXING_EXTRUDER 0

#define DRIVE_SYSTEM 0
#define XAXIS_STEPS_PER_MM 610
#define YAXIS_STEPS_PER_MM 610
#define ZAXIS_STEPS_PER_MM 6400
#define EXTRUDER_FAN_COOL_TEMP 50
#define PDM_FOR_EXTRUDER 0
#define PDM_FOR_COOLER 0
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 15
#define DECOUPLING_TEST_MIN_TEMP_RISE 1
#define KILL_IF_SENSOR_DEFECT 0
#define RETRACT_ON_PAUSE 2
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""
#define SHARED_EXTRUDER_HEATER 0
#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
#define EXT0_Z_OFFSET 0
#define EXT0_STEPS_PER_MM 147
#define EXT0_TEMPSENSOR_TYPE 1
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE 1
#define EXT0_ENABLE_PIN ORIG_E0_ENABLE_PIN
#define EXT0_ENABLE_ON 0
#define EXT0_MIRROR_STEPPER 0
#define EXT0_STEP2_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR2_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE2 0
#define EXT0_ENABLE2_PIN ORIG_E0_ENABLE_PIN
#define EXT0_MAX_FEEDRATE 30
#define EXT0_MAX_START_FEEDRATE 5
#define EXT0_MAX_ACCELERATION 10000
#define EXT0_HEAT_MANAGER 1
#define EXT0_PREHEAT_TEMP 190
#define EXT0_WATCHPERIOD 1
#define EXT0_PID_INTEGRAL_DRIVE_MAX 220
#define EXT0_PID_INTEGRAL_DRIVE_MIN 40
#define EXT0_PID_PGAIN_OR_DEAD_TIME 20
#define EXT0_PID_I 0.6
#define EXT0_PID_D 65
#define EXT0_PID_MAX 255
#define EXT0_ADVANCE_K 0
#define EXT0_ADVANCE_L 0
#define EXT0_ADVANCE_BACKLASH_STEPS 0
#define EXT0_WAIT_RETRACT_TEMP 150
#define EXT0_WAIT_RETRACT_UNITS 0
#define EXT0_SELECT_COMMANDS "M117 Extruder 1"
#define EXT0_DESELECT_COMMANDS ""
#define EXT0_EXTRUDER_COOLER_PIN -1
#define EXT0_EXTRUDER_COOLER_SPEED 255
#define EXT0_DECOUPLE_TEST_PERIOD 20000
#define EXT0_JAM_PIN 35
#define EXT0_JAM_PULLUP 0
#define EXT1_X_OFFSET 9760
#define EXT1_Y_OFFSET 0
#define EXT1_Z_OFFSET -5440
#define EXT1_STEPS_PER_MM 147
#define EXT1_TEMPSENSOR_TYPE 1
#define EXT1_TEMPSENSOR_PIN TEMP_2_PIN
#define EXT1_HEATER_PIN HEATER_2_PIN
#define EXT1_STEP_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE 0
#define EXT1_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define EXT1_ENABLE_ON 0
#define EXT1_MIRROR_STEPPER 0
#define EXT1_STEP2_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR2_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE2 0
#define EXT1_ENABLE2_PIN ORIG_E1_ENABLE_PIN
#define EXT1_MAX_FEEDRATE 30
#define EXT1_MAX_START_FEEDRATE 5
#define EXT1_MAX_ACCELERATION 10000
#define EXT1_HEAT_MANAGER 1
#define EXT1_PREHEAT_TEMP 190
#define EXT1_WATCHPERIOD 1
#define EXT1_PID_INTEGRAL_DRIVE_MAX 220
#define EXT1_PID_INTEGRAL_DRIVE_MIN 40
#define EXT1_PID_PGAIN_OR_DEAD_TIME 20
#define EXT1_PID_I 0.6
#define EXT1_PID_D 65
#define EXT1_PID_MAX 255
#define EXT1_ADVANCE_K 0
#define EXT1_ADVANCE_L 0
#define EXT1_ADVANCE_BACKLASH_STEPS 0
#define EXT1_WAIT_RETRACT_TEMP 150
#define EXT1_WAIT_RETRACT_UNITS 0
#define EXT1_SELECT_COMMANDS "M117 Extruder 2\nM400\nM340 P0 S1950 R600\nG4 P300"
#define EXT1_DESELECT_COMMANDS "M340 P0 S1050 R600\nG4 P300"
#define EXT1_EXTRUDER_COOLER_PIN -1
#define EXT1_EXTRUDER_COOLER_SPEED 255
#define EXT1_DECOUPLE_TEST_PERIOD 20000
#define EXT1_JAM_PIN 33
#define EXT1_JAM_PULLUP 0

#define AUTORETRACT_ENABLED 0
#define RETRACTION_LENGTH 3
#define RETRACTION_LONG_LENGTH 13
#define RETRACTION_SPEED 40
#define RETRACTION_Z_LIFT 0
#define RETRACTION_UNDO_EXTRA_LENGTH 0
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0
#define RETRACTION_UNDO_SPEED 25
#define FILAMENTCHANGE_X_POS 5
#define FILAMENTCHANGE_Y_POS 5
#define FILAMENTCHANGE_Z_ADD 2
#define FILAMENTCHANGE_REHOME 1
#define FILAMENTCHANGE_SHORTRETRACT 2.5
#define FILAMENTCHANGE_LONGRETRACT 50
#define JAM_METHOD 1
#define JAM_STEPS 220
#define JAM_SLOWDOWN_STEPS 1000
#define JAM_SLOWDOWN_TO 75
#define JAM_ERROR_STEPS 1500
#define JAM_MIN_STEPS 10
#define JAM_ACTION 1

#define RETRACT_DURING_HEATUP true
#define PID_CONTROL_RANGE 20
#define SKIP_M109_IF_WITHIN 5
#define SCALE_PID_TO_MAX 0
#define TEMP_HYSTERESIS 0
#define EXTRUDE_MAXLENGTH 160
#define NUM_TEMPS_USERTHERMISTOR0 0
#define USER_THERMISTORTABLE0 \
    { \
    }
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1 \
    { \
    }
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2 \
    { \
    }
#define GENERIC_THERM_VREF 5
#define GENERIC_THERM_NUM_ENTRIES 33
#define HEATER_PWM_SPEED 0
#define COOLER_PWM_SPEED 0

// ############# Heated bed configuration ########################

#define HAVE_HEATED_BED 1
#define HEATED_BED_PREHEAT_TEMP 55
#define HEATED_BED_MAX_TEMP 120
#define SKIP_M190_IF_WITHIN 5
#define HEATED_BED_SENSOR_TYPE 1
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
#define HEATED_BED_SET_INTERVAL 5000
#define HEATED_BED_HEAT_MANAGER 3
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME 12
#define HEATED_BED_PID_IGAIN 33
#define HEATED_BED_PID_DGAIN 290
#define HEATED_BED_PID_MAX 255
#define HEATED_BED_DECOUPLE_TEST_PERIOD 60000
#define MIN_EXTRUDER_TEMP 150
#define MAXTEMP 275
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 290
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
#define ENDSTOP_PULLUP_X_MIN false
#define ENDSTOP_X_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_Y_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Y false
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_Z_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Z true
#define ENDSTOP_PULLUP_Z2_MINMAX true
#define ENDSTOP_Z2_MINMAX_INVERTING false
#define MINMAX_HARDWARE_ENDSTOP_Z2 false
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_X_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_X false
#define ENDSTOP_PULLUP_Y_MAX false
#define ENDSTOP_Y_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MAX true
#define ENDSTOP_Z_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Z false
#define max_software_endstop_r true

#define min_software_endstop_x false
#define min_software_endstop_y true
#define min_software_endstop_z false
#define max_software_endstop_x true
#define max_software_endstop_y false
#define max_software_endstop_z true
#define DOOR_PIN -1
#define DOOR_PULLUP 1
#define DOOR_INVERTING 1
#define ENDSTOP_X_BACK_MOVE 30
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 1
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_X_BACK_ON_HOME 0.5
#define ENDSTOP_Y_BACK_ON_HOME 0.5
#define ENDSTOP_Z_BACK_ON_HOME 0
#define ALWAYS_CHECK_ENDSTOPS 1
#define MOVE_X_WHEN_HOMED 0
#define MOVE_Y_WHEN_HOMED 0
#define MOVE_Z_WHEN_HOMED 0

// ################# XYZ movements ###################

#define X_ENABLE_ON 1
#define Y_ENABLE_ON 1
#define Z_ENABLE_ON 1
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0
#define DISABLE_E 0
#define PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT
#define INVERT_X_DIR 1
#define INVERT_Y_DIR 0
#define INVERT_Z_DIR 1
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1
#define X_MAX_LENGTH 240
#define Y_MAX_LENGTH 245
#define Z_MAX_LENGTH 225
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define Z2_MINMAX_PIN -1

#define DISTORTION_CORRECTION_POINTS 5
#define DISTORTION_LIMIT_TO 2
#define DISTORTION_CORRECTION_R 100
#define DISTORTION_PERMANENT 1
#define DISTORTION_UPDATE_FREQUENCY 15
#define DISTORTION_START_DEGRADE 5
#define DISTORTION_END_HEIGHT 10
#define DISTORTION_EXTRAPOLATE_CORNERS 0
#define DISTORTION_XMIN 10
#define DISTORTION_YMIN 10
#define DISTORTION_XMAX 225
#define DISTORTION_YMAX 230

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define FEATURE_BABYSTEPPING 1
#define BABYSTEP_MULTIPLICATOR 64

#define DELTA_SEGMENTS_PER_SECOND_PRINT 180 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70   // Less accurate setting for other moves
#define EXACT_DELTA_MOVES 1

// Delta settings
#define DELTA_HOME_ON_POWER 0

#define DELTASEGMENTS_PER_PRINTLINE 24
#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 1200L
#define MAX_FEEDRATE_X 300
#define MAX_FEEDRATE_Y 300
#define MAX_FEEDRATE_Z 20
#define HOMING_FEEDRATE_X 50
#define HOMING_FEEDRATE_Y 50
#define HOMING_FEEDRATE_Z 5
#define ZHOME_PRE_RAISE 0
#define ZHOME_PRE_RAISE_DISTANCE 10
#define RAISE_Z_ON_TOOLCHANGE 0
#define ZHOME_MIN_TEMPERATURE 0
#define ZHOME_HEAT_ALL 0
#define ZHOME_HEAT_HEIGHT 10
#define ZHOME_X_POS 140
#define ZHOME_Y_POS 45
#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define RAMP_ACCELERATION 1
#define STEPPER_HIGH_DELAY 0
#define DIRECTION_DELAY 0
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 800
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 50
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 1100
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1100
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100
#define INTERPOLATE_ACCELERATION_WITH_Z 1
#define ACCELERATION_FACTOR_TOP 75
#define MAX_JERK 5
#define MAX_ZJERK 0.3
#define PRINTLINE_CACHE_SIZE 32
#define MOVE_CACHE_LOW 10
#define LOW_TICKS_PER_MOVE 250000
#define EXTRUDER_SWITCH_XY_SPEED 100
#define DUAL_X_AXIS 0
#define DUAL_X_RESOLUTION 0
#define X2AXIS_STEPS_PER_MM 100
#define FEATURE_TWO_XSTEPPER 0
#define X2_STEP_PIN ORIG_E1_STEP_PIN
#define X2_DIR_PIN ORIG_E1_DIR_PIN
#define X2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_YSTEPPER 0
#define Y2_STEP_PIN ORIG_E1_STEP_PIN
#define Y2_DIR_PIN ORIG_E1_DIR_PIN
#define Y2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_ZSTEPPER 0
#define Z2_STEP_PIN ORIG_E1_STEP_PIN
#define Z2_DIR_PIN ORIG_E1_DIR_PIN
#define Z2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_THREE_ZSTEPPER 0
#define Z3_STEP_PIN ORIG_E2_STEP_PIN
#define Z3_DIR_PIN ORIG_E2_DIR_PIN
#define Z3_ENABLE_PIN ORIG_E2_ENABLE_PIN
#define FEATURE_FOUR_ZSTEPPER 0
#define Z4_STEP_PIN ORIG_E3_STEP_PIN
#define Z4_DIR_PIN ORIG_E3_DIR_PIN
#define Z4_ENABLE_PIN ORIG_E3_ENABLE_PIN
#define FEATURE_DITTO_PRINTING 0

// ################# Misc. settings ##################

#define BAUDRATE 115200
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0
#define KILL_METHOD 1
#define ACK_WITH_LINENUMBER 1
#define KEEP_ALIVE_INTERVAL 2000
#define ECHO_ON_EXECUTE 1
#undef PS_ON_PIN
#define PS_ON_PIN -1

/* ======== Servos =======
Control the servos with
M340 P<servoId> S<pulseInUS>   / ServoID = 0..3  pulseInUs = 500..2500
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/
#define SERVO0_PIN 5
#define SERVO1_PIN -1
#define SERVO2_PIN -1
#define SERVO3_PIN -1
#define SERVO0_NEUTRAL_POS 1050
#define SERVO1_NEUTRAL_POS -1
#define SERVO2_NEUTRAL_POS -1
#define SERVO3_NEUTRAL_POS -1
#define UI_SERVO_CONTROL 1

#define FEATURE_SERVO 1

// #################### Z-Probing #####################

#define Z_PROBE_Z_OFFSET 0.05
#define Z_PROBE_Z_OFFSET_MODE 1
#define UI_BED_COATING 1
#define EXTRUDER_IS_Z_PROBE 1
#define Z_PROBE_DISABLE_HEATERS 1
#define Z_PROBE_BED_DISTANCE 3
#define Z_PROBE_PIN ORIG_Z_MIN_PIN
#define Z_PROBE_PULLUP 0
#define Z_PROBE_ON_HIGH 1
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
#define Z_PROBE_WAIT_BEFORE_TEST 0
#define Z_PROBE_SPEED 2
#define Z_PROBE_XY_SPEED 150
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_REPETITIONS 1
#define Z_PROBE_HEIGHT 0
#define Z_PROBE_DELAY 0
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""
#define Z_PROBE_RUN_AFTER_EVERY_PROBE ""
#define Z_PROBE_REQUIRES_HEATING 1
#define Z_PROBE_MIN_TEMPERATURE 150
#define FEATURE_AUTOLEVEL 1
#define FEATURE_SOFTWARE_LEVELING 0
#define Z_PROBE_X1 60
#define Z_PROBE_Y1 130
#define Z_PROBE_X2 137
#define Z_PROBE_Y2 45
#define Z_PROBE_X3 137
#define Z_PROBE_Y3 210
#define BED_LEVELING_METHOD 2
#define BED_CORRECTION_METHOD 0
#define BED_LEVELING_GRID_SIZE 5
#define BED_LEVELING_REPETITIONS 5
#define BED_MOTOR_1_X 55
#define BED_MOTOR_1_Y 130
#define BED_MOTOR_2_X 137
#define BED_MOTOR_2_Y 45
#define BED_MOTOR_3_X 137
#define BED_MOTOR_3_Y 210
#define BENDING_CORRECTION_A 0
#define BENDING_CORRECTION_B 0
#define BENDING_CORRECTION_C 0
#define FEATURE_AXISCOMP 0
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0

#ifndef SDSUPPORT // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT 1
#undef SDCARDDETECT
#define SDCARDDETECT ORIG_SDCARDDETECT
#define SDCARDDETECTINVERTED 0
#endif
#define SD_EXTENDED_DIR 1 /** Show extended directory including file length. Don't use this with Pronterface! */
#define SD_RUN_ON_STOP ""
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 1
#define ARC_SUPPORT 0
#define FEATURE_MEMORY_POSITION 1
#define FEATURE_CHECKSUM_FORCED 0
#define FEATURE_FAN_CONTROL 1
#define FEATURE_FAN2_CONTROL 0
#define FEATURE_CONTROLLER 0 // 11
#define ADC_KEYPAD_PIN -1
#define LANGUAGE_EN_ACTIVE 1
#define LANGUAGE_DE_ACTIVE 1
#define LANGUAGE_NL_ACTIVE 1
#define LANGUAGE_PT_ACTIVE 1
#define LANGUAGE_IT_ACTIVE 1
#define LANGUAGE_ES_ACTIVE 1
#define LANGUAGE_FI_ACTIVE 1
#define LANGUAGE_SE_ACTIVE 1
#define LANGUAGE_FR_ACTIVE 1
#define LANGUAGE_CZ_ACTIVE 1
#define LANGUAGE_PL_ACTIVE 1
#define LANGUAGE_TR_ACTIVE 1
#define UI_PRINTER_NAME "FELIX Pro 1"
#define UI_PRINTER_COMPANY "FELIXprinters"
#define UI_PAGES_DURATION 4000
#define UI_SPEEDDEPENDENT_POSITIONING 0
#define UI_DISABLE_AUTO_PAGESWITCH 1
#define UI_AUTORETURN_TO_MENU_AFTER 30000
#define FEATURE_UI_KEYS 0
#define UI_ENCODER_SPEED 2
#define UI_REVERSE_ENCODER 0
#define UI_KEY_BOUNCETIME 10
#define UI_KEY_FIRST_REPEAT 500
#define UI_KEY_REDUCE_REPEAT 50
#define UI_KEY_MIN_REPEAT 50
#define FEATURE_BEEPER 0
#define CASE_LIGHTS_PIN 25
#define CASE_LIGHT_DEFAULT_ON 1
#define UI_START_SCREEN_DELAY 2000
#define UI_DYNAMIC_ENCODER_SPEED 1
#define UI_HEAD "E1:%e0\002C E2:%e1\002C B:%eb\002C"
/**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 2, 2
#define BEEPER_LONG_SEQUENCE 8, 8
#define UI_SET_MIN_HEATED_BED_TEMP 30
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP 80
#define UI_SET_MAX_EXTRUDER_TEMP 275
#define UI_SET_EXTRUDER_FEEDRATE 5
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3

#define NUM_MOTOR_DRIVERS 2
#define MOTOR_DRIVER_1(var) StepperDriver<51, 53, 49, 0, 0> var(3382, 0.2)
#define MOTOR_DRIVER_2(var) StepperDriver<39, 13, 40, 0, 0> var(3382, 0.2)

#define ALTERNATIVE_JERK
#define REDUCE_ON_SMALL_SEGMENTS
//#define CUSTOM_EVENTS
//#define CUSTOM_MENU
//#define CUSTOM_TRANSLATIONS
#define LIMIT_MOTORIZED_CORRECTION 0.5
#define HALFAUTOMATIC_LEVELING 1
// add z probe height routine
#define ZPROBE_HEIGHT_ROUTINE
#define ZPROBE_REF_HEIGHT 5.97
#define Z_UP_AFTER_HOME 10
#define CUSTOM_LOGO
#define LOGO_WIDTH 104
#define LOGO_HEIGHT 44
#define LOGO_BITMAP const unsigned char logo[] PROGMEM = { \
    0x00, 0x00, 0x01, 0xF0, 0x1F, 0xFC, 0x7F, 0xF1, 0xE0, 0x03, 0xC7, 0xC1, 0xF0, 0x00, 0x00, 0x1F, \
    0xF0, 0x1F, 0xFC, 0x7F, 0xF1, 0xE0, 0x03, 0xC3, 0xE1, 0xE0, 0x00, 0x00, 0x7F, 0xF0, 0x1F, 0xFC, \
    0x7F, 0xF1, 0xE0, 0x03, 0xC3, 0xE3, 0xE0, 0x00, 0x01, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, \
    0x03, 0xC1, 0xF7, 0xC0, 0x00, 0x07, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF, \
    0x80, 0x00, 0x1F, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF, 0x80, 0x00, 0x3F, \
    0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0x7F, 0x00, 0x00, 0x7F, 0xFF, 0xF0, 0x1F, \
    0xF8, 0x7F, 0xE1, 0xE0, 0x03, 0xC0, 0x3E, 0x00, 0x00, 0xFF, 0xFF, 0xF0, 0x1F, 0xF8, 0x7F, 0xE1, \
    0xE0, 0x03, 0xC0, 0x3F, 0x00, 0x01, 0xFF, 0xFF, 0xF0, 0x1F, 0xF8, 0x7F, 0xE1, 0xE0, 0x03, 0xC0, \
    0x7F, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF, 0x80, 0x03, \
    0xFF, 0xFC, 0x00, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xF7, 0xC0, 0x07, 0xFF, 0xF0, 0x00, \
    0x1E, 0x00, 0x78, 0x01, 0xFF, 0xC3, 0xC1, 0xF7, 0xC0, 0x07, 0xFF, 0xE0, 0x00, 0x1E, 0x00, 0x7F, \
    0xF1, 0xFF, 0xC3, 0xC3, 0xE3, 0xE0, 0x0F, 0xFF, 0xC0, 0x00, 0x1E, 0x00, 0x7F, 0xF1, 0xFF, 0xC3, \
    0xC3, 0xC1, 0xF0, 0x0F, 0xFF, 0x80, 0x00, 0x1E, 0x00, 0x7F, 0xF1, 0xFF, 0xC3, 0xC7, 0xC1, 0xF8, \
    0x0F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0x03, \
    0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x1F, 0xF0, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0xFF, 0xF0, 0x00, 0x02, 0x00, 0x40, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x1E, 0x1A, 0x3C, 0xF3, 0xC6, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF, \
    0xFF, 0xF0, 0x31, 0x22, 0x22, 0x44, 0x28, 0x88, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, \
    0x22, 0x22, 0x44, 0x28, 0x80, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x47, \
    0xE8, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x44, 0x08, 0x08, 0x00, \
    0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x42, 0x28, 0xC8, 0x00, 0x00, 0x00, 0x3F, \
    0xFF, 0xFF, 0xF0, 0x3E, 0x22, 0x22, 0x31, 0xC8, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, \
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFC, 0x00, 0x30, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x3F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x80, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, \
    0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
};

#endif

/* Below you will find the configuration string, that created this Configuration.h

========== Start configuration string ==========
{
    "editMode": 2,
    "processor": 1,
    "baudrate": 115200,
    "bluetoothSerial": -1,
    "bluetoothBaudrate": 250000,
    "xStepsPerMM": 610,
    "yStepsPerMM": 610,
    "zStepsPerMM": 6400,
    "xInvert": "1",
    "xInvertEnable": "1",
    "eepromMode": 2,
    "yInvert": "0",
    "yInvertEnable": "1",
    "zInvert": "1",
    "zInvertEnable": "1",
    "extruder": [
        {
            "id": 0,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 220,
            "pidMax": 255,
            "sensorType": 1,
            "sensorPin": "TEMP_0_PIN",
            "heaterPin": "HEATER_0_PIN",
            "maxFeedrate": 30,
            "startFeedrate": 5,
            "invert": "1",
            "invertEnable": "0",
            "acceleration": 10000,
            "watchPeriod": 1,
            "pidP": 20,
            "pidI": 0.6,
            "pidD": 65,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 147,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "M117 Extruder 1",
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
            "decoupleTestPeriod": 20,
            "jamPin": 35,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 0",
                "step": "ORIG_E0_STEP_PIN",
                "dir": "ORIG_E0_DIR_PIN",
                "enable": "ORIG_E0_ENABLE_PIN"
            },
            "preheat": 190,
            "inverse": 0
        },
        {
            "id": 1,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 220,
            "pidMax": 255,
            "sensorType": 1,
            "sensorPin": "TEMP_2_PIN",
            "heaterPin": "HEATER_2_PIN",
            "maxFeedrate": 30,
            "startFeedrate": 5,
            "invert": "0",
            "invertEnable": "0",
            "acceleration": 10000,
            "watchPeriod": 1,
            "pidP": 20,
            "pidI": 0.6,
            "pidD": 65,
            "advanceK": 0,
            "advanceL": 0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 147,
            "coolerPin": -1,
            "coolerSpeed": 255,
            "selectCommands": "M117 Extruder 2\\nM400\\nM340 P0 S1950 R600\\nG4 P300",
            "deselectCommands": "M340 P0 S1050 R600\\nG4 P300",
            "xOffset": 16,
            "yOffset": 0,
            "zOffset": -0.85,
            "xOffsetSteps": 9760,
            "yOffsetSteps": 0,
            "zOffsetSteps": -5440,
            "stepper": {
                "name": "Extruder 1",
                "step": "ORIG_E1_STEP_PIN",
                "dir": "ORIG_E1_DIR_PIN",
                "enable": "ORIG_E1_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 20,
            "jamPin": 33,
            "jamPullup": "0",
            "mirror": "0",
            "invert2": "0",
            "stepper2": {
                "name": "Extruder 1",
                "step": "ORIG_E1_STEP_PIN",
                "dir": "ORIG_E1_DIR_PIN",
                "enable": "ORIG_E1_ENABLE_PIN"
            },
            "preheat": 190,
            "inverse": 0
        }
    ],
    "uiLanguage": 0,
    "uiController": 0,
    "xMinEndstop": 4,
    "yMinEndstop": 0,
    "zMinEndstop": 2,
    "xMaxEndstop": 0,
    "yMaxEndstop": 4,
    "zMaxEndstop": 0,
    "motherboard": 405,
    "driveSystem": 0,
    "xMaxSpeed": 200,
    "xHomingSpeed": 50,
    "xTravelAcceleration": 1100,
    "xPrintAcceleration": 1100,
    "yMaxSpeed": 200,
    "yHomingSpeed": 50,
    "yTravelAcceleration": 1100,
    "yPrintAcceleration": 1100,
    "zMaxSpeed": 10,
    "zHomingSpeed": 3,
    "zTravelAcceleration": 100,
    "zPrintAcceleration": 100,
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
    "enableBacklash": "0",
    "backlashX": 0,
    "backlashY": 0,
    "backlashZ": 0,
    "stepperInactiveTime": 360,
    "maxInactiveTime": 1200,
    "xMinPos": 0,
    "yMinPos": 0,
    "zMinPos": 0,
    "xLength": 240,
    "yLength": 245,
    "zLength": 225,
    "alwaysCheckEndstops": "1",
    "disableX": "0",
    "disableY": "0",
    "disableZ": "0",
    "disableE": "0",
    "xHomeDir": "-1",
    "yHomeDir": "1",
    "zHomeDir": "-1",
    "xEndstopBack": 0.5,
    "yEndstopBack": 0.5,
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
    "stepDoublerFrequency": 80000,
    "allowQuadstepping": "1",
    "doubleStepDelay": 1,
    "maxJerk": 12.5,
    "maxZJerk": 0.3,
    "moveCacheSize": 32,
    "moveCacheLow": 10,
    "lowTicksPerMove": 250000,
    "enablePowerOnStartup": "1",
    "echoOnExecute": "1",
    "sendWaits": "1",
    "ackWithLineNumber": "1",
    "killMethod": 1,
    "useAdvance": "1",
    "useQuadraticAdvance": "1",
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
    "mirrorZ": "0",
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
    "dittoPrinting": "0",
    "featureServos": "1",
    "servo0Pin": 5,
    "servo1Pin": -1,
    "servo2Pin": -1,
    "servo3Pin": -1,
    "featureWatchdog": "1",
    "hasHeatedBed": "1",
    "enableZProbing": "1",
    "extrudeMaxLength": 160,
    "homeOrder": "HOME_ORDER_ZXYTZ",
    "featureController": 11,
    "uiPrinterName": "FELIX Pro 1",
    "uiPrinterCompany": "FELIXprinters",
    "uiPagesDuration": 4000,
    "uiHeadline": "E1:%e0\\002C E2:%e1\\002C B:%eb\\002C",
    "uiDisablePageswitch": "1",
    "uiAutoReturnAfter": 30000,
    "featureKeys": "0",
    "uiEncoderSpeed": 2,
    "uiReverseEncoder": "0",
    "uiKeyBouncetime": 10,
    "uiKeyFirstRepeat": 500,
    "uiKeyReduceRepeat": 50,
    "uiKeyMinRepeat": 50,
    "featureBeeper": "0",
    "uiMinHeatedBed": 30,
    "uiMaxHeatedBed": 120,
    "uiMinEtxruderTemp": 80,
    "uiMaxExtruderTemp": 275,
    "uiExtruderFeedrate": 5,
    "uiExtruderRetractDistance": 3,
    "uiSpeeddependentPositioning": "0",
    "maxBedTemperature": 120,
    "bedSensorType": 1,
    "bedSensorPin": "TEMP_1_PIN",
    "bedHeaterPin": "HEATER_1_PIN",
    "bedHeatManager": 3,
    "bedPreheat": 55,
    "bedUpdateInterval": 5000,
    "bedPidDriveMin": 80,
    "bedPidDriveMax": 255,
    "bedPidP": 12,
    "bedPidI": 33,
    "bedPidD": 290,
    "bedPidMax": 255,
    "bedDecoupleTestPeriod": 60,
    "caseLightPin": 25,
    "caseLightDefaultOn": "1",
    "bedSkipIfWithin": 5,
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
        "temps": [],
        "numEntries": 0
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
    "skipM109Within": 5,
    "extruderFanCoolTemp": 50,
    "minTemp": 150,
    "maxTemp": 275,
    "minDefectTemp": -10,
    "maxDefectTemp": 290,
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
    "zProbePin": "ORIG_Z_MIN_PIN",
    "zProbeBedDistance": 3,
    "zProbePullup": "0",
    "zProbeOnHigh": "1",
    "zProbeXOffset": 0,
    "zProbeYOffset": 0,
    "zProbeWaitBeforeTest": "0",
    "zProbeSpeed": 2,
    "zProbeXYSpeed": 150,
    "zProbeHeight": 0,
    "zProbeStartScript": "",
    "zProbeFinishedScript": "",
    "featureAutolevel": "1",
    "zProbeX1": 60,
    "zProbeY1": 130,
    "zProbeX2": 137,
    "zProbeY2": 45,
    "zProbeX3": 137,
    "zProbeY3": 210,
    "zProbeSwitchingDistance": 1,
    "zProbeRepetitions": 1,
    "zProbeEveryPoint": "",
    "sdSupport": "1",
    "sdCardDetectPin": "ORIG_SDCARDDETECT",
    "sdCardDetectInverted": "0",
    "uiStartScreenDelay": 2000,
    "xEndstopBackMove": 5,
    "yEndstopBackMove": 5,
    "zEndstopBackMove": 1,
    "xEndstopRetestFactor": 2,
    "yEndstopRetestFactor": 2,
    "zEndstopRetestFactor": 2,
    "xMinPin": "ORIG_X_MIN_PIN",
    "yMinPin": -1,
    "zMinPin": "ORIG_Z_MIN_PIN",
    "xMaxPin": -1,
    "yMaxPin": "ORIG_Y_MIN_PIN",
    "zMaxPin": -1,
    "deltaHomeOnPower": "0",
    "fanBoardPin": -1,
    "heaterPWMSpeed": 0,
    "featureBabystepping": "1",
    "babystepMultiplicator": 64,
    "pdmForHeater": "0",
    "pdmForCooler": "0",
    "psOn": -1,
    "mixingExtruder": "0",
    "decouplingTestMaxHoldVariance": 15,
    "decouplingTestMinTempRise": 1,
    "featureAxisComp": "0",
    "axisCompTanXY": 0,
    "axisCompTanXZ": 0,
    "axisCompTanYZ": 0,
    "retractOnPause": 2,
    "pauseStartCommands": "",
    "pauseEndCommands": "",
    "distortionCorrection": "1",
    "distortionCorrectionPoints": 5,
    "distortionCorrectionR": 100,
    "distortionPermanent": "1",
    "distortionUpdateFrequency": 15,
    "distortionStartDegrade": 5,
    "distortionEndDegrade": 10,
    "distortionExtrapolateCorners": "0",
    "distortionXMin": 10,
    "distortionXMax": 225,
    "distortionYMin": 10,
    "distortionYMax": 230,
    "sdRunOnStop": "",
    "sdStopHeaterMotorsOnStop": "1",
    "featureRetraction": "1",
    "autoretractEnabled": "0",
    "retractionLength": 3,
    "retractionLongLength": 13,
    "retractionSpeed": 40,
    "retractionZLift": 0,
    "retractionUndoExtraLength": 0,
    "retractionUndoExtraLongLength": 0,
    "retractionUndoSpeed": 25,
    "filamentChangeXPos": 5,
    "filamentChangeYPos": 5,
    "filamentChangeZAdd": 2,
    "filamentChangeRehome": 1,
    "filamentChangeShortRetract": 2.5,
    "filamentChangeLongRetract": 50,
    "fanKickstart": 200,
    "servo0StartPos": "1050",
    "servo1StartPos": -1,
    "servo2StartPos": -1,
    "servo3StartPos": -1,
    "uiDynamicEncoderSpeed": "1",
    "uiServoControl": 1,
    "killIfSensorDefect": "0",
    "jamSteps": 220,
    "jamSlowdownSteps": 1000,
    "jamSlowdownTo": 75,
    "jamErrorSteps": 1500,
    "jamMinSteps": 10,
    "jamAction": 1,
    "jamMethod": 1,
    "primaryPort": 1,
    "numMotorDrivers": 2,
    "motorDrivers": [
        {
            "t": "Stepper",
            "s": "StepperDriver<51,53,49,0,0> var(3382,0.2)",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 3382,
            "speed": 0.2,
            "dirPin": 53,
            "stepPin": 51,
            "enablePin": 49,
            "endstopPin": -1,
            "invertEndstop": "0",
            "minEndstop": "1",
            "endstopPullup": "1",
            "maxDistance": 20
        },
        {
            "t": "Stepper",
            "s": "StepperDriver<39,13,40,0,0> var(3382,0.2)",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 3382,
            "speed": 0.2,
            "dirPin": 13,
            "stepPin": 39,
            "enablePin": 40,
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
    "manualConfig": "#define ALTERNATIVE_JERK\n#define REDUCE_ON_SMALL_SEGMENTS\n#define CUSTOM_EVENTS\n#define CUSTOM_MENU\n#define CUSTOM_TRANSLATIONS\n#define LIMIT_MOTORIZED_CORRECTION 0.5\n#define HALFAUTOMATIC_LEVELING 1\n\/\/ add z probe height routine\n#define ZPROBE_HEIGHT_ROUTINE\n#define ZPROBE_REF_HEIGHT 5.97\n#define Z_UP_AFTER_HOME 10\n#define CUSTOM_LOGO\n\/\/#define LOGO_WIDTH 104\n#define LOGO_WIDTH 0\n#define LOGO_HEIGHT 44\n#define LOGO_BITMAP const unsigned char logo[] PROGMEM = {\\\n0x00, 0x00, 0x01, 0xF0, 0x1F, 0xFC, 0x7F, 0xF1, 0xE0, 0x03, 0xC7, 0xC1, 0xF0, 0x00, 0x00, 0x1F,\\\n0xF0, 0x1F, 0xFC, 0x7F, 0xF1, 0xE0, 0x03, 0xC3, 0xE1, 0xE0, 0x00, 0x00, 0x7F, 0xF0, 0x1F, 0xFC,\\\n0x7F, 0xF1, 0xE0, 0x03, 0xC3, 0xE3, 0xE0, 0x00, 0x01, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0,\\\n0x03, 0xC1, 0xF7, 0xC0, 0x00, 0x07, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF,\\\n0x80, 0x00, 0x1F, 0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF, 0x80, 0x00, 0x3F,\\\n0xFF, 0xF0, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0x7F, 0x00, 0x00, 0x7F, 0xFF, 0xF0, 0x1F,\\\n0xF8, 0x7F, 0xE1, 0xE0, 0x03, 0xC0, 0x3E, 0x00, 0x00, 0xFF, 0xFF, 0xF0, 0x1F, 0xF8, 0x7F, 0xE1,\\\n0xE0, 0x03, 0xC0, 0x3F, 0x00, 0x01, 0xFF, 0xFF, 0xF0, 0x1F, 0xF8, 0x7F, 0xE1, 0xE0, 0x03, 0xC0,\\\n0x7F, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xFF, 0x80, 0x03,\\\n0xFF, 0xFC, 0x00, 0x1E, 0x00, 0x78, 0x01, 0xE0, 0x03, 0xC0, 0xF7, 0xC0, 0x07, 0xFF, 0xF0, 0x00,\\\n0x1E, 0x00, 0x78, 0x01, 0xFF, 0xC3, 0xC1, 0xF7, 0xC0, 0x07, 0xFF, 0xE0, 0x00, 0x1E, 0x00, 0x7F,\\\n0xF1, 0xFF, 0xC3, 0xC3, 0xE3, 0xE0, 0x0F, 0xFF, 0xC0, 0x00, 0x1E, 0x00, 0x7F, 0xF1, 0xFF, 0xC3,\\\n0xC3, 0xC1, 0xF0, 0x0F, 0xFF, 0x80, 0x00, 0x1E, 0x00, 0x7F, 0xF1, 0xFF, 0xC3, 0xC7, 0xC1, 0xF8,\\\n0x0F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0x03,\\\n0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x1F, 0xF0, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0xFF, 0xF0, 0x00, 0x02, 0x00, 0x40, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x1E, 0x1A, 0x3C, 0xF3, 0xC6, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF,\\\n0xFF, 0xF0, 0x31, 0x22, 0x22, 0x44, 0x28, 0x88, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31,\\\n0x22, 0x22, 0x44, 0x28, 0x80, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x47,\\\n0xE8, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x44, 0x08, 0x08, 0x00,\\\n0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF0, 0x31, 0x22, 0x22, 0x42, 0x28, 0xC8, 0x00, 0x00, 0x00, 0x3F,\\\n0xFF, 0xFF, 0xF0, 0x3E, 0x22, 0x22, 0x31, 0xC8, 0x70, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00,\\\n0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFC, 0x00, 0x30, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x3F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x80,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x3F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\\\n0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F,\\\n0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00\\\n};",
    "zHomeMinTemperature": 0,
    "zHomeXPos": 140,
    "zHomeYPos": 45,
    "zHomeHeatHeight": 10,
    "zHomeHeatAll": "0",
    "zProbeZOffsetMode": 1,
    "zProbeZOffset": 0.05,
    "zProbeDelay": 0,
    "uiBedCoating": "1",
    "langEN": "1",
    "langDE": "1",
    "langNL": "1",
    "langPT": "1",
    "langIT": "1",
    "langES": "1",
    "langFI": "1",
    "langSE": "1",
    "langFR": "1",
    "langCZ": "1",
    "langPL": "1",
    "langTR": "1",
    "interpolateAccelerationWithZ": 1,
    "accelerationFactorTop": 75,
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
    "bedLevelingMethod": 2,
    "bedCorrectionMethod": 0,
    "bedLevelingGridSize": 5,
    "bedLevelingRepetitions": 5,
    "bedMotor1X": 55,
    "bedMotor1Y": 130,
    "bedMotor2X": 137,
    "bedMotor2Y": 45,
    "bedMotor3X": 137,
    "bedMotor3Y": 210,
    "zProbeRequiresHeating": "1",
    "zProbeMinTemperature": 150,
    "adcKeypadPin": -1,
    "sharedExtruderHeater": "0",
    "extruderSwitchXYSpeed": 100,
    "dualXAxis": "0",
    "boardFanSpeed": 255,
    "keepAliveInterval": 2000,
    "moveXWhenHomed": "1",
    "moveYWhenHomed": "1",
    "moveZWhenHomed": "1",
    "preheatTime": 30000,
    "multiZEndstopHoming": "0",
    "z2MinMaxPin": -1,
    "z2MinMaxEndstop": 0,
    "extruderIsZProbe": "1",
    "boardFanMinSpeed": 0,
    "doorPin": -1,
    "doorEndstop": 1,
    "zhomePreRaise": 0,
    "zhomePreRaiseDistance": 10,
    "dualXResolution": "0",
    "x2axisStepsPerMM": 100,
    "coolerPWMSpeed": 0,
    "raiseZOnToolchange": 0,
    "distortionLimitTo": 2,
    "uiPresetBedTempPLA": 55,
    "uiPresetBedABS": 80,
    "uiPresetExtruderPLA": 190,
    "uiPresetExtruderABS": 225,
    "uiAnimation": "1",
    "interpolateZAxis": "1",
    "zAccelerationTop": 20,
    "maxHalfstepInterval": 1999,
    "hasMAX6675": false,
    "hasMAX31855": false,
    "hasGeneric1": false,
    "hasGeneric2": false,
    "hasGeneric3": false,
    "hasUser0": false,
    "hasUser1": false,
    "hasUser2": false,
    "numExtruder": 2,
    "version": 100,
    "primaryPortName": "Serial",
    "zProbeDisableHeaters": "1"
}
========== End configuration string ==========

*/