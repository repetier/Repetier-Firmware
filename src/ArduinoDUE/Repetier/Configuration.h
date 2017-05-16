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

#define NUM_EXTRUDER 2
#define MOTHERBOARD 402
#define RFSERIAL Serial
#include "pins.h"

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
#undef X_MAX_PIN
#define X_MAX_PIN ORIG_X_MIN_PIN

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1
#define BLUETOOTH_SERIAL  -1
#define BLUETOOTH_BAUD  115200
#define MIXING_EXTRUDER 0

#define DRIVE_SYSTEM 0
#define XAXIS_STEPS_PER_MM 512
#define YAXIS_STEPS_PER_MM 519
#define ZAXIS_STEPS_PER_MM 23569
#define EXTRUDER_FAN_COOL_TEMP 50
#define PDM_FOR_EXTRUDER 0
#define PDM_FOR_COOLER 0
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 30
#define DECOUPLING_TEST_MIN_TEMP_RISE 1
#define KILL_IF_SENSOR_DEFECT 0
#define RETRACT_ON_PAUSE 2
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""
#define SHARED_EXTRUDER_HEATER 0
#define EXT0_X_OFFSET -22784
#define EXT0_Y_OFFSET 0
#define EXT0_Z_OFFSET 0
#define EXT0_STEPS_PER_MM 2230
#define EXT0_TEMPSENSOR_TYPE 13
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE 0
#define EXT0_ENABLE_PIN ORIG_E0_ENABLE_PIN
#define EXT0_ENABLE_ON 1
#define EXT0_MIRROR_STEPPER 0
#define EXT0_STEP2_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR2_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE2 0
#define EXT0_ENABLE2_PIN ORIG_E0_ENABLE_PIN
#define EXT0_MAX_FEEDRATE 45
#define EXT0_MAX_START_FEEDRATE 20
#define EXT0_MAX_ACCELERATION 3000
#define EXT0_HEAT_MANAGER 1
#define EXT0_PREHEAT_TEMP 190
#define EXT0_WATCHPERIOD 1
#define EXT0_PID_INTEGRAL_DRIVE_MAX 230
#define EXT0_PID_INTEGRAL_DRIVE_MIN 40
#define EXT0_PID_PGAIN_OR_DEAD_TIME 18.3
#define EXT0_PID_I 2.13
#define EXT0_PID_D 39
#define EXT0_PID_MAX 255
#define EXT0_ADVANCE_K 0
#define EXT0_ADVANCE_L 10
#define EXT0_ADVANCE_BACKLASH_STEPS 0
#define EXT0_WAIT_RETRACT_TEMP 150
#define EXT0_WAIT_RETRACT_UNITS 0
#define EXT0_SELECT_COMMANDS "G92 E0\nG1 E10 F300\nG1 E8 F1800\nG92 E0\nM117 Extruder 1\nG4 P1000"
#define EXT0_DESELECT_COMMANDS "G92 E0\nG1 E-2 F1800\nG92 E0"
#define EXT0_EXTRUDER_COOLER_PIN HEATER_3_PIN
#define EXT0_EXTRUDER_COOLER_SPEED 255
#define EXT0_DECOUPLE_TEST_PERIOD 30000
#define EXT0_JAM_PIN -1
#define EXT0_JAM_PULLUP 0
#define EXT1_X_OFFSET 207206
#define EXT1_Y_OFFSET 0
#define EXT1_Z_OFFSET 0
#define EXT1_STEPS_PER_MM 2230
#define EXT1_TEMPSENSOR_TYPE 13
#define EXT1_TEMPSENSOR_PIN TEMP_2_PIN
#define EXT1_HEATER_PIN HEATER_2_PIN
#define EXT1_STEP_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE 0
#define EXT1_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define EXT1_ENABLE_ON 1
#define EXT1_MIRROR_STEPPER 0
#define EXT1_STEP2_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR2_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE2 0
#define EXT1_ENABLE2_PIN ORIG_E1_ENABLE_PIN
#define EXT1_MAX_FEEDRATE 45
#define EXT1_MAX_START_FEEDRATE 20
#define EXT1_MAX_ACCELERATION 3000
#define EXT1_HEAT_MANAGER 1
#define EXT1_PREHEAT_TEMP 190
#define EXT1_WATCHPERIOD 1
#define EXT1_PID_INTEGRAL_DRIVE_MAX 230
#define EXT1_PID_INTEGRAL_DRIVE_MIN 40
#define EXT1_PID_PGAIN_OR_DEAD_TIME 18.3
#define EXT1_PID_I 2.13
#define EXT1_PID_D 39
#define EXT1_PID_MAX 255
#define EXT1_ADVANCE_K 0
#define EXT1_ADVANCE_L 10
#define EXT1_ADVANCE_BACKLASH_STEPS 0
#define EXT1_WAIT_RETRACT_TEMP 150
#define EXT1_WAIT_RETRACT_UNITS 0
#define EXT1_SELECT_COMMANDS "G92 E0\nG1 E10 F300\nG1 E8 F1800\nG92 E0\nM117 Extruder 2\nG4 P1000"
#define EXT1_DESELECT_COMMANDS "G92 E0\nG1 E-2 F1800\nG92 E0"
#define EXT1_EXTRUDER_COOLER_PIN HEATER_3_PIN
#define EXT1_EXTRUDER_COOLER_SPEED 255
#define EXT1_DECOUPLE_TEST_PERIOD 30000
#define EXT1_JAM_PIN -1
#define EXT1_JAM_PULLUP 0

#define FEATURE_RETRACTION 1
#define AUTORETRACT_ENABLED 0
#define RETRACTION_LENGTH 3
#define RETRACTION_LONG_LENGTH 13
#define RETRACTION_SPEED 40
#define RETRACTION_Z_LIFT 0.5
#define RETRACTION_UNDO_EXTRA_LENGTH 0
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0
#define RETRACTION_UNDO_SPEED 20
#define FILAMENTCHANGE_X_POS 0
#define FILAMENTCHANGE_Y_POS 0
#define FILAMENTCHANGE_Z_ADD  2
#define FILAMENTCHANGE_REHOME 1
#define FILAMENTCHANGE_SHORTRETRACT 1
#define FILAMENTCHANGE_LONGRETRACT 10
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
#define NUM_TEMPS_USERTHERMISTOR0 0
#define USER_THERMISTORTABLE0 {}
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1 {}
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2 {}
#define GENERIC_THERM_VREF 5
#define GENERIC_THERM_NUM_ENTRIES 33
#define HEATER_PWM_SPEED 0

// ############# Heated bed configuration ########################

#define HAVE_HEATED_BED 1
#define HEATED_BED_PREHEAT_TEMP 55
#define HEATED_BED_MAX_TEMP 120
#define SKIP_M190_IF_WITHIN 3
#define HEATED_BED_SENSOR_TYPE 13
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
#define HEATED_BED_SET_INTERVAL 5000
#define HEATED_BED_HEAT_MANAGER 1
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME   131.1
#define HEATED_BED_PID_IGAIN   3.76
#define HEATED_BED_PID_DGAIN 1143.1
#define HEATED_BED_PID_MAX 255
#define HEATED_BED_DECOUPLE_TEST_PERIOD 300000
#define MIN_EXTRUDER_TEMP 150
#define MAXTEMP 285
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 350
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


#define DEFAULT_PRINTER_MODE 0

// ################ Endstop configuration #####################

#define MULTI_ZENDSTOP_HOMING 0
#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_X_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_Y_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_Z_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Z true
#define ENDSTOP_PULLUP_Z2_MINMAX true
#define ENDSTOP_Z2_MINMAX_INVERTING false
#define MINMAX_HARDWARE_ENDSTOP_Z2 false
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_X_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_Y_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Y false
#define ENDSTOP_PULLUP_Z_MAX true
#define ENDSTOP_Z_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Z false
#define max_software_endstop_r true

#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false
#define max_software_endstop_x false
#define max_software_endstop_y true
#define max_software_endstop_z true
#define DOOR_PIN -1
#define DOOR_PULLUP 1
#define DOOR_INVERTING 0
#define ENDSTOP_X_BACK_MOVE 5
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 2
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_X_BACK_ON_HOME 1
#define ENDSTOP_Y_BACK_ON_HOME 0
#define ENDSTOP_Z_BACK_ON_HOME 0
#define ALWAYS_CHECK_ENDSTOPS 0
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
#define INVERT_X_DIR 0
#define INVERT_Y_DIR 0
#define INVERT_Z_DIR 1
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define X_MAX_LENGTH 356
#define Y_MAX_LENGTH 274
#define Z_MAX_LENGTH 230
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define Z2_MINMAX_PIN -1

#define DISTORTION_CORRECTION 0
#define DISTORTION_CORRECTION_POINTS 5
#define DISTORTION_CORRECTION_R 100
#define DISTORTION_PERMANENT 0
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
#define BABYSTEP_MULTIPLICATOR 200

#define DELTA_SEGMENTS_PER_SECOND_PRINT 180 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70 // Less accurate setting for other moves
#define EXACT_DELTA_MOVES 1

// Delta settings
#define DELTA_HOME_ON_POWER 0

#define DELTASEGMENTS_PER_PRINTLINE 24
#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 0L
#define MAX_FEEDRATE_X 400
#define MAX_FEEDRATE_Y 400
#define MAX_FEEDRATE_Z 10
#define HOMING_FEEDRATE_X 100
#define HOMING_FEEDRATE_Y 100
#define HOMING_FEEDRATE_Z 10
#define HOMING_ORDER HOME_ORDER_YZX
#define ZHOME_PRE_RAISE 2
#define ZHOME_PRE_RAISE_DISTANCE 10
#define ZHOME_MIN_TEMPERATURE 0
#define ZHOME_HEAT_ALL 1
#define ZHOME_HEAT_HEIGHT 20
#define ZHOME_X_POS 120
#define ZHOME_Y_POS 120
#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define RAMP_ACCELERATION 1
#define STEPPER_HIGH_DELAY 1
#define DIRECTION_DELAY 0
#define STEP_DOUBLER_FREQUENCY 80000
#define ALLOW_QUADSTEPPING 1
#define DOUBLE_STEP_DELAY 0 // time in microseconds
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1300
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 300
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 3000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 300
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100
#define INTERPOLATE_ACCELERATION_WITH_Z 0
#define ACCELERATION_FACTOR_TOP 100
#define MAX_JERK 5
#define MAX_ZJERK 0.5
#define PRINTLINE_CACHE_SIZE 16
#define MOVE_CACHE_LOW 10
#define LOW_TICKS_PER_MOVE 250000
#define EXTRUDER_SWITCH_XY_SPEED 300
#define DUAL_X_AXIS 1
#define DUAL_X_RESOLUTION 1
#define X2AXIS_STEPS_PER_MM 400
#define FEATURE_TWO_XSTEPPER 0
#define X2_STEP_PIN   ORIG_E2_STEP_PIN
#define X2_DIR_PIN    ORIG_E2_DIR_PIN
#define X2_ENABLE_PIN ORIG_E2_ENABLE_PIN
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

#define BAUDRATE 115200
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0
#define KILL_METHOD 1
#define ACK_WITH_LINENUMBER 1
#define KEEP_ALIVE_INTERVAL 2000
#define WAITING_IDENTIFIER "wait"
#define ECHO_ON_EXECUTE 1
#define EEPROM_MODE 1
#undef PS_ON_PIN
#define PS_ON_PIN -1
#define JSON_OUTPUT 0

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

        #define FEATURE_WATCHDOG 1

// #################### Z-Probing #####################

#define Z_PROBE_Z_OFFSET 0
#define Z_PROBE_Z_OFFSET_MODE 0
#define UI_BED_COATING 0
#define FEATURE_Z_PROBE 0
#define EXTRUDER_IS_Z_PROBE 0
#define Z_PROBE_BED_DISTANCE 10
#define Z_PROBE_PIN ORIG_Z_MIN_PIN
#define Z_PROBE_PULLUP 1
#define Z_PROBE_ON_HIGH 0
#define Z_PROBE_X_OFFSET 30
#define Z_PROBE_Y_OFFSET 20
#define Z_PROBE_WAIT_BEFORE_TEST 0
#define Z_PROBE_SPEED 10
#define Z_PROBE_XY_SPEED 250
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_REPETITIONS 1
#define Z_PROBE_HEIGHT 0
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
#define Z_PROBE_X3 20
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
#define SDCARDDETECTINVERTED 1
#endif
#define SD_EXTENDED_DIR 1 /** Show extended directory including file length. Don't use this with Pronterface! */
#define SD_RUN_ON_STOP ""
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 0
#define ARC_SUPPORT 1
#define FEATURE_MEMORY_POSITION 1
#define FEATURE_CHECKSUM_FORCED 0
#define FEATURE_FAN_CONTROL 1
#define FEATURE_FAN2_CONTROL 1
#define FEATURE_CONTROLLER 25
#define ADC_KEYPAD_PIN -1
#define LANGUAGE_EN_ACTIVE 1
#define LANGUAGE_DE_ACTIVE 1
#define LANGUAGE_NL_ACTIVE 1
#define LANGUAGE_PT_ACTIVE 0
#define LANGUAGE_IT_ACTIVE 0
#define LANGUAGE_ES_ACTIVE 0
#define LANGUAGE_FI_ACTIVE 0
#define LANGUAGE_SE_ACTIVE 0
#define LANGUAGE_FR_ACTIVE 0
#define LANGUAGE_CZ_ACTIVE 0
#define LANGUAGE_PL_ACTIVE 0
#define LANGUAGE_TR_ACTIVE 0
#define UI_PRINTER_NAME "Prodim XXL Pro"
#define UI_PRINTER_COMPANY "V1.0.9"
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
#define FEATURE_BEEPER 0
#define CASE_LIGHTS_PIN -1
#define CASE_LIGHT_DEFAULT_ON 1
#define UI_START_SCREEN_DELAY 1000
#define UI_DYNAMIC_ENCODER_SPEED 1
#define UI_HEAD "E:%ec/%Ec\002 B:%eb/%Eb\002"
        /**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 2,2
#define BEEPER_LONG_SEQUENCE 8,8
#define UI_SET_MIN_HEATED_BED_TEMP  40
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP   170
#define UI_SET_MAX_EXTRUDER_TEMP   285
#define UI_SET_EXTRUDER_FEEDRATE 2
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3


#define NUM_MOTOR_DRIVERS 0

// Firmware als basis V1.0.7 
// LAZY_DUAL_X_AXIS 0  >  1 in Repetier.h
#define LAZY_DUAL_X_AXIS 1
// Overstap naar Groot scherm > Nu Support voor groot LCD Zie voor veranderingen tabblad LCD 128x64 V1.0.0
//  RADDS LCD DISPLAY 4x20 naar sparkLCD + adapter on RADDS LCD PORT  + de veranderingen in de tabblad!
// #define SDCARDDETECTINVERTED van 0 naar 1
// Logo veranderd naar Prodim in Logo.h
// #define ENDSTOP_Y_BACK_ON_HOME 10 > 0   (voor groter print bereik in y-as)

// BED Temperature sensor van 100K EPcos..   naar    PT100 E3D/Ultimaker  (ivm PT100 board ondersteuning)
// Extr1 Temperature sensor van 100K EPcos..   naar    PT100 E3D/Ultimaker  (ivm PT100 board ondersteuning)
// Extr2 Temperature sensor van 100K EPcos..   naar    PT100 E3D/Ultimaker (ivm PT100 board ondersteuning)

// Exotische talen uitgechakeld nu aleen Engels, Duits en Nederlands Beschikbaar.
// UI_PRINTER_COMPANY Versie nummer V1.0.8 er in gezet.

#endif

/* Below you will find the configuration string, that created this Configuration.h

========== Start configuration string ==========
{
    "editMode": 2,
    "processor": 1,
    "baudrate": 115200,
    "bluetoothSerial": -1,
    "bluetoothBaudrate": 115200,
    "xStepsPerMM": 512,
    "yStepsPerMM": 519,
    "zStepsPerMM": 23569,
    "xInvert": "0",
    "xInvertEnable": "1",
    "eepromMode": 1,
    "yInvert": "0",
    "yInvertEnable": "1",
    "zInvert": "1",
    "zInvertEnable": "1",
    "extruder": [
        {
            "id": 0,
            "heatManager": 1,
            "pidDriveMin": 40,
            "pidDriveMax": 230,
            "pidMax": 255,
            "sensorType": 13,
            "sensorPin": "TEMP_0_PIN",
            "heaterPin": "HEATER_0_PIN",
            "maxFeedrate": 45,
            "startFeedrate": 20,
            "invert": "0",
            "invertEnable": "1",
            "acceleration": 3000,
            "watchPeriod": 1,
            "pidP": 18.3,
            "pidI": 2.13,
            "pidD": 39,
            "advanceK": 0,
            "advanceL": 10,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 2230,
            "coolerPin": "HEATER_3_PIN",
            "coolerSpeed": 255,
            "selectCommands": "G92 E0\\nG1 E10 F300\\nG1 E8 F1800\\nG92 E0\\nM117 Extruder 1\\nG4 P1000",
            "deselectCommands": "G92 E0\\nG1 E-2 F1800\\nG92 E0",
            "xOffset": -44.5,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": -22784,
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
            "pidDriveMax": 230,
            "pidMax": 255,
            "sensorType": 13,
            "sensorPin": "TEMP_2_PIN",
            "heaterPin": "HEATER_2_PIN",
            "maxFeedrate": 45,
            "startFeedrate": 20,
            "invert": "0",
            "invertEnable": "1",
            "acceleration": 3000,
            "watchPeriod": 1,
            "pidP": 18.3,
            "pidI": 2.13,
            "pidD": 39,
            "advanceK": 0,
            "advanceL": 10,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 2230,
            "coolerPin": "HEATER_3_PIN",
            "coolerSpeed": 255,
            "selectCommands": "G92 E0\\nG1 E10 F300\\nG1 E8 F1800\\nG92 E0\\nM117 Extruder 2\\nG4 P1000",
            "deselectCommands": "G92 E0\\nG1 E-2 F1800\\nG92 E0",
            "xOffset": 404.7,
            "yOffset": 0,
            "zOffset": 0,
            "xOffsetSteps": 207206,
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
        }
    ],
    "uiLanguage": 0,
    "uiController": 0,
    "xMinEndstop": 2,
    "yMinEndstop": 2,
    "zMinEndstop": 2,
    "xMaxEndstop": 2,
    "yMaxEndstop": 0,
    "zMaxEndstop": 0,
    "motherboard": 402,
    "driveSystem": 0,
    "xMaxSpeed": 400,
    "xHomingSpeed": 100,
    "xTravelAcceleration": 3000,
    "xPrintAcceleration": 1300,
    "yMaxSpeed": 400,
    "yHomingSpeed": 100,
    "yTravelAcceleration": 300,
    "yPrintAcceleration": 300,
    "zMaxSpeed": 10,
    "zHomingSpeed": 10,
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
    "maxInactiveTime": 0,
    "xMinPos": 0,
    "yMinPos": 0,
    "zMinPos": 0,
    "xLength": 356,
    "yLength": 274,
    "zLength": 230,
    "alwaysCheckEndstops": "0",
    "disableX": "0",
    "disableY": "0",
    "disableZ": "0",
    "disableE": "0",
    "xHomeDir": "-1",
    "yHomeDir": "-1",
    "zHomeDir": "-1",
    "xEndstopBack": 1,
    "yEndstopBack": 0,
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
    "stepperHighDelay": 1,
    "directionDelay": 0,
    "stepDoublerFrequency": 80000,
    "allowQuadstepping": "1",
    "doubleStepDelay": 0,
    "maxJerk": 5,
    "maxZJerk": 0.5,
    "moveCacheSize": 16,
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
    "mirrorX": "0",
    "mirrorXMotor": {
        "name": "Extruder 2",
        "step": "ORIG_E2_STEP_PIN",
        "dir": "ORIG_E2_DIR_PIN",
        "enable": "ORIG_E2_ENABLE_PIN"
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
    "homeOrder": "HOME_ORDER_YZX",
    "featureController": 25,
    "uiPrinterName": "Prodim XXL Pro",
    "uiPrinterCompany": "V1.0.9",
    "uiPagesDuration": 4000,
    "uiHeadline": "E:%ec\/%Ec\\002 B:%eb\/%Eb\\002",
    "uiDisablePageswitch": "1",
    "uiAutoReturnAfter": 30000,
    "featureKeys": "0",
    "uiEncoderSpeed": 1,
    "uiReverseEncoder": "0",
    "uiKeyBouncetime": 10,
    "uiKeyFirstRepeat": 500,
    "uiKeyReduceRepeat": 50,
    "uiKeyMinRepeat": 50,
    "featureBeeper": "0",
    "uiMinHeatedBed": 40,
    "uiMaxHeatedBed": 120,
    "uiMinEtxruderTemp": 170,
    "uiMaxExtruderTemp": 285,
    "uiExtruderFeedrate": 2,
    "uiExtruderRetractDistance": 3,
    "uiSpeeddependentPositioning": "0",
    "maxBedTemperature": 120,
    "bedSensorType": 13,
    "bedSensorPin": "TEMP_1_PIN",
    "bedHeaterPin": "HEATER_1_PIN",
    "bedHeatManager": 1,
    "bedPreheat": 55,
    "bedUpdateInterval": 5000,
    "bedPidDriveMin": 80,
    "bedPidDriveMax": 255,
    "bedPidP": 131.1,
    "bedPidI": 3.76,
    "bedPidD": 1143.1,
    "bedPidMax": 255,
    "bedDecoupleTestPeriod": 300,
    "caseLightPin": -1,
    "caseLightDefaultOn": "1",
    "bedSkipIfWithin": 3,
    "gen1T0": 25,
    "gen1R0": 100,
    "gen1Beta": 500,
    "gen1MinTemp": -20,
    "gen1MaxTemp": 300,
    "gen1R1": 4700,
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
        "r1": 4700,
        "r2": 4700,
        "temps": [
            {
                "t": 0,
                "r": 100,
                "adc": 83.5714285714
            },
            {
                "t": 10,
                "r": 103.9,
                "adc": 86.6927136395
            },
            {
                "t": 20,
                "r": 107.8,
                "adc": 89.8040930914
            },
            {
                "t": 30,
                "r": 111.7,
                "adc": 92.9056140066
            },
            {
                "t": 40,
                "r": 115.5,
                "adc": 95.9181707564
            },
            {
                "t": 50,
                "r": 119.4,
                "adc": 99.000364461
            },
            {
                "t": 60,
                "r": 123.2,
                "adc": 101.994177584
            },
            {
                "t": 70,
                "r": 127.1,
                "adc": 105.057224173
            },
            {
                "t": 80,
                "r": 130.9,
                "adc": 108.032468056
            },
            {
                "t": 90,
                "r": 134.7,
                "adc": 110.998611502
            },
            {
                "t": 100,
                "r": 138.5,
                "adc": 113.955696203
            }
        ],
        "numEntries": 11
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
    "maxTemp": 285,
    "minDefectTemp": -10,
    "maxDefectTemp": 350,
    "arcSupport": "1",
    "featureMemoryPositionWatchdog": "1",
    "forceChecksum": "0",
    "sdExtendedDir": "1",
    "featureFanControl": "1",
    "fanPin": "ORIG_FAN_PIN",
    "featureFan2Control": "1",
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
    "zProbeBedDistance": 10,
    "zProbePullup": "1",
    "zProbeOnHigh": "0",
    "zProbeXOffset": 30,
    "zProbeYOffset": 20,
    "zProbeWaitBeforeTest": "0",
    "zProbeSpeed": 10,
    "zProbeXYSpeed": 250,
    "zProbeHeight": 0,
    "zProbeStartScript": "",
    "zProbeFinishedScript": "",
    "featureAutolevel": "0",
    "zProbeX1": 20,
    "zProbeY1": 20,
    "zProbeX2": 160,
    "zProbeY2": 20,
    "zProbeX3": 20,
    "zProbeY3": 160,
    "zProbeSwitchingDistance": 1,
    "zProbeRepetitions": 1,
    "zProbeEveryPoint": "",
    "sdSupport": "1",
    "sdCardDetectPin": "ORIG_SDCARDDETECT",
    "sdCardDetectInverted": "1",
    "uiStartScreenDelay": 1000,
    "xEndstopBackMove": 5,
    "yEndstopBackMove": 5,
    "zEndstopBackMove": 2,
    "xEndstopRetestFactor": 3,
    "yEndstopRetestFactor": 3,
    "zEndstopRetestFactor": 3,
    "xMinPin": "ORIG_X_MIN_PIN",
    "yMinPin": "ORIG_Y_MIN_PIN",
    "zMinPin": "ORIG_Z_MIN_PIN",
    "xMaxPin": "ORIG_X_MIN_PIN",
    "yMaxPin": "ORIG_Y_MAX_PIN",
    "zMaxPin": "ORIG_Z_MAX_PIN",
    "deltaHomeOnPower": "0",
    "fanBoardPin": -1,
    "heaterPWMSpeed": 0,
    "featureBabystepping": "1",
    "babystepMultiplicator": 200,
    "pdmForHeater": "0",
    "pdmForCooler": "0",
    "psOn": -1,
    "mixingExtruder": "0",
    "decouplingTestMaxHoldVariance": 30,
    "decouplingTestMinTempRise": 1,
    "featureAxisComp": "0",
    "axisCompTanXY": 0,
    "axisCompTanXZ": 0,
    "axisCompTanYZ": 0,
    "retractOnPause": 2,
    "pauseStartCommands": "",
    "pauseEndCommands": "",
    "distortionCorrection": "0",
    "distortionCorrectionPoints": 5,
    "distortionCorrectionR": 100,
    "distortionPermanent": "0",
    "distortionUpdateFrequency": 15,
    "distortionStartDegrade": 0.5,
    "distortionEndDegrade": 1,
    "distortionExtrapolateCorners": "0",
    "distortionXMin": 10,
    "distortionXMax": 190,
    "distortionYMin": 10,
    "distortionYMax": 190,
    "sdRunOnStop": "",
    "sdStopHeaterMotorsOnStop": "0",
    "featureRetraction": "1",
    "autoretractEnabled": "0",
    "retractionLength": 3,
    "retractionLongLength": 13,
    "retractionSpeed": 40,
    "retractionZLift": 0.5,
    "retractionUndoExtraLength": 0,
    "retractionUndoExtraLongLength": 0,
    "retractionUndoSpeed": 20,
    "filamentChangeXPos": 0,
    "filamentChangeYPos": 0,
    "filamentChangeZAdd": 2,
    "filamentChangeRehome": 1,
    "filamentChangeShortRetract": 1,
    "filamentChangeLongRetract": 10,
    "fanKickstart": 200,
    "servo0StartPos": -1,
    "servo1StartPos": -1,
    "servo2StartPos": -1,
    "servo3StartPos": -1,
    "uiDynamicEncoderSpeed": "1",
    "uiServoControl": 0,
    "killIfSensorDefect": "0",
    "jamSteps": 220,
    "jamSlowdownSteps": 320,
    "jamSlowdownTo": 70,
    "jamErrorSteps": 500,
    "jamMinSteps": 10,
    "jamAction": 1,
    "jamMethod": 1,
    "primaryPort": 1,
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
            "enablePin": -1
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
            "enablePin": -1
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
            "enablePin": -1
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
            "enablePin": -1
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
            "enablePin": -1
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
            "enablePin": -1
        }
    ],
    "manualConfig": "\/\/ Firmware als basis V1.0.7 \n\/\/ LAZY_DUAL_X_AXIS 0  >  1 in Repetier.h\n#define LAZY_DUAL_X_AXIS 1\n\/\/ Overstap naar Groot scherm > Nu Support voor groot LCD Zie voor veranderingen tabblad LCD 128x64 V1.0.0\n\/\/  RADDS LCD DISPLAY 4x20 naar sparkLCD + adapter on RADDS LCD PORT  + de veranderingen in de tabblad!\n\/\/ #define SDCARDDETECTINVERTED van 0 naar 1\n\/\/ Logo veranderd naar Prodim in Logo.h\n\/\/ #define ENDSTOP_Y_BACK_ON_HOME 10 > 0   (voor groter print bereik in y-as)\n\n\/\/ BED Temperature sensor van 100K EPcos..   naar    PT100 E3D\/Ultimaker  (ivm PT100 board ondersteuning)\n\/\/ Extr1 Temperature sensor van 100K EPcos..   naar    PT100 E3D\/Ultimaker  (ivm PT100 board ondersteuning)\n\/\/ Extr2 Temperature sensor van 100K EPcos..   naar    PT100 E3D\/Ultimaker (ivm PT100 board ondersteuning)\n\n\/\/ Exotische talen uitgechakeld nu aleen Engels, Duits en Nederlands Beschikbaar.\n\/\/ UI_PRINTER_COMPANY Versie nummer V1.0.8 er in gezet.",
    "zHomeMinTemperature": 0,
    "zHomeXPos": 120,
    "zHomeYPos": 120,
    "zHomeHeatHeight": 20,
    "zHomeHeatAll": "1",
    "zProbeZOffsetMode": 0,
    "zProbeZOffset": 0,
    "uiBedCoating": "0",
    "langEN": "1",
    "langDE": "1",
    "langNL": "1",
    "langPT": "0",
    "langIT": "0",
    "langES": "0",
    "langFI": "0",
    "langSE": "0",
    "langFR": "0",
    "langCZ": "0",
    "langPL": "0",
    "langTR": "0",
    "interpolateAccelerationWithZ": 0,
    "accelerationFactorTop": 100,
    "bendingCorrectionA": 0,
    "bendingCorrectionB": 0,
    "bendingCorrectionC": 0,
    "preventZDisableOnStepperTimeout": "0",
    "supportLaser": "0",
    "laserPin": -1,
    "laserOnHigh": "1",
    "laserWarmupTime": 0,
    "defaultPrinterMode": 0,
    "supportCNC": "0",
    "cncWaitOnEnable": 300,
    "cncWaitOnDisable": 0,
    "cncEnablePin": -1,
    "cncEnableWith": "1",
    "cncDirectionPin": -1,
    "cncDirectionCW": "1",
    "startupGCode": "",
    "jsonOutput": "0",
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
    "extruderSwitchXYSpeed": 300,
    "dualXAxis": "1",
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
    "doorEndstop": 0,
    "zhomePreRaise": 2,
    "zhomePreRaiseDistance": 10,
    "dualXResolution": "1",
    "x2axisStepsPerMM": 400,
    "uiPresetBedTempPLA": 60,
    "uiPresetBedABS": 100,
    "uiPresetExtruderPLA": 205,
    "uiPresetExtruderABS": 260,
    "uiAnimation": "0",
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
    "primaryPortName": "Serial"
}
========== End configuration string ==========

*/