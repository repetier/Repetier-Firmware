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

/*
Early stage version for Stacke F1 printer - use with care

*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// ------- SELECT MODEL TO COMPILE -----------

#define STACKER_F1
// #define STACKER_F1_XL
// #define STACKER_F1_XL_IDEX

// ------- No further changes required below --------

// ******* Model dependent changes ***********
#if defined(STACKER_F1_XL_IDEX)
#define IDEX 1
#else
#define IDEX 0
#endif

#if defined(STACKER_F1_XL) || defined(STACKER_F1_XL_IDEX)
#define DUAL_Y 1
#define XL_FORMAT 1
#else
#define DUAL_Y 0
#define XL_FORMAT 0
#endif

// Adds z probe support
#define STACKER_WITH_ZPROBE 1
// Adds 2 z end stops
#define STACKER_2_Z_END_STOPS 1

/**************** READ FIRST ************************

   This configuration file was created with the configuration tool. For that
   reason, it does not contain the same informations as the original
   Configuration.h file. It misses the comments and unused parts. Open this file
   file in the config tool to see and change the data. You can also upload it to
   newer/older versions. The system will silently add new options, so
   compilation continues to work.

   This file is optimized for version 1.0.0dev
   generator: http://www.repetier.com/firmware/dev/

   If you are in doubt which named functions use which pins on your board,
   please check the pins.h for the used name->pin assignments and your board
   documentation to verify it is as you expect.

*/

// The follwing variables are required early to decide on the right modules.

// #define RAPS128_XY // define for usage of RAPS128 on xy axis
// #define DEBUG_POS

#define NUM_SERVOS 0
#define NUM_TOOLS 1 + IDEX
#define MOTHERBOARD MOTHERBOARD_STACKER3D_SUPER_MINI // Stacker 3d Superboard
#define EEPROM_MODE 1
#define RFSERIAL Serial
#define BLUETOOTH_SERIAL -1
#define JSON_OUTPUT 1
#define FEATURE_SERVO 0
#define FEATURE_WATCHDOG 1
#define FEATURE_RETRACTION 1
#define USE_ADVANCE 1
#define NUM_AXES 4 + IDEX        // X,Y,Z and E for extruder A,B,C would be 5,6,7
#define STEPPER_FREQUENCY 300000 // Maximum stepper frequency.

// Update frequency for new blocks. Must be higher then
// PREPARE_FREQUENCY.
// Number of blocks with constant stepper rate per second.
#define PREPARE_FREQUENCY 2000
#define BLOCK_FREQUENCY 1000
#define VELOCITY_PROFILE 2      // 0 = linear, 1 = cubic, 2 = quintic velocity shape
#define SLOW_DIRECTION_CHANGE 1 // can be reason for lost steps on slow drivers
// Smaller segments reduce join speed to prevent vibrations causing lost steps
#define SMALL_SEGMENT_SIZE 0.4
#define Z_SPEED 8    // Z positioning speed
#define XY_SPEED 200 // XY positioning speed for normal operations
#define A_SPEED 200  // A positioning speed for normal operations
// Speed for G0 moves. Independent from set F value! Set 0 to use F value.
#define G0_FEEDRATE 0
#define MAX_ROOM_TEMPERATURE 25 // No heating below this temperature!
// Start with controlling if temperature is +/- this value to target temperature
#define TEMPERATURE_CONTROL_RANGE 20
#if STACKER_WITH_ZPROBE
#define Z_PROBE_TYPE Z_PROBE_TYPE_DEFAULT
#else
#define Z_PROBE_TYPE Z_PROBE_TYPE_NONE
#endif
#define Z_PROBE_HEIGHT 5        // Distance bed-nozzle when trigger switches
#define Z_PROBE_BED_DISTANCE 10 // Optimal starting distance
#define Z_PROBE_SPEED 5         // Speed fo z testing
#define Z_PROBE_X_OFFSET 30     // x offset relative to extruder 0,0 offset
#define Z_PROBE_Y_OFFSET -23    // y offset relative to extruder 0,0 offset
#define Z_PROBE_COATING 0       // Coating thickness if not detected by probe
// Extra delay before starting again. Only needed on electronic probes keeping state for a while
#define Z_PROBE_DELAY 0
#define Z_PROBE_REPETITIONS 1 // How often should we probe, 1 is minimum
// 0 = use average, 1 = use middle value after ordering z
#define Z_PROBE_USE_MEDIAN 1
// Minimum distance required to safely untrigger probe - used for faster repeated measurement
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_BORDER 5 // Safety border to ensure position is allowed
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""
#define Z_PROBE_RUN_AFTER_EVERY_PROBE ""
#define LEVELING_METHOD 1 // Grid measurement
#define MAX_GRID_SIZE 6                   // Maximum grid size allocation in memory, imported grid can be smaller
#define ENABLE_BUMP_CORRECTION 1          // CPU intensive, so only activate if required
#define BUMP_CORRECTION_START_DEGRADE 0.5 // Until this height we correct 100%
#define BUMP_CORRECTION_END_HEIGHT 2      // From this height on we do no correction
#define BUMP_LIMIT_TO 2                   // Maximum allowed correction up/down

// 0 = Cartesian, 1 = CoreXYZ, 2 = delta, 3 = Dual X-Axis
#if IDEX
#define PRINTER_TYPE PRINTER_TYPE_DUAL_X
#else
#define PRINTER_TYPE PRINTER_TYPE_CARTESIAN
#endif
// steps to include as babysteps per 1/BLOCK_FREQUENCY seconds. Must be lower
// then STEPPER_FREQUENCY/BLOCK_FREQUENCY and be low enough to not loose steps.
#if IDEX
#define BABYSTEPS_PER_BLOCK \
    { 1, 1, 1, 1, 1 }
#else
#define BABYSTEPS_PER_BLOCK \
    { 1, 1, 1, 1 }
#endif
// If all axis end stops are hardware based we can skip the time consuming tests
// each step
#define NO_SOFTWARE_AXIS_ENDSTOPS
// Normally only a delta has motor end stops required. Normally you trigger
// using axis endstops.
#define NO_MOTOR_ENDSTOPS

#define FEATURE_CONTROLLER CONTROLLER_SPARKLCD
// Use more memory to speedup display updates
#define DISPLAY_FULL_BUFFER 1
// Direction 1 or -1
#define ENCODER_DIRECTION -1
// Uncomment to hide toogle light menu entry in controls
// #define NO_LIGHT_CONTROL
// Encoder speed 0 = fastest, 1 or 2 = slowest - set so 1 click is one menu move
// Default is 2 if not set by controller. Us eonly to fix wrong setting
// #define ENCODER_SPEED 2
// Set 1 if you want to replace the default themes and define them in
// configuration_io.h
#define CUSTOM_DEFAULT_THEMES 0

/* Ratios for core xyz. First index denotes motor and second axis.
For each motor you can set the ratio of x,y,z position that adds
to the position. 0 = no contribution. */
// X motor = x + y
#define COREXYZ_X_X 1
#define COREXYZ_X_Y 1
#define COREXYZ_X_Z 0
// Y motor = x - y
#define COREXYZ_Y_X 1
#define COREXYZ_Y_Y -1
#define COREXYZ_Y_Z 0
// Z motor = z
#define COREXYZ_Z_X 0
#define COREXYZ_Z_Y 0
#define COREXYZ_Z_Z 1

// Special geometry definition if printer type is delta
/*  =========== Parameter essential for delta calibration ===================

            C, Y-Axis
            |                        |___| Carriage horizontal offset
            |                        |
   \------------------------------------------
            |_________ X-axis        |    \ | / \                       |     \
   DELTA_DIAGONAL (length)    Each move this Rod Height /   \ \ is calculated /
   \                             \    Carriage is at printer center!   | A B
   \_____/--------------------------------
                                              |--| End effector horizontal
   offset (recommend set it to 0)
                                         |----| DELTA_HORIZONTAL_RADIUS
   (Horizontal rod pivot to pivot measure)

    Column angles are measured from X-axis counterclockwise
    "Standard" positions: alpha_A = 210, alpha_B = 330, alpha_C = 90
*/
#define DELTA_DIAGONAL 444.800f
#define DELTA_HORIZONTAL_RADIUS 209.900f
#define DELTA_PRINT_RADIUS 209.0f
#define DELTA_ANGLE_A 210.0f
#define DELTA_ANGLE_B 330.0f
#define DELTA_ANGLE_C 90.123f
#define DELTA_CORRECTION_A 0.0f
#define DELTA_CORRECTION_B 0.0f
#define DELTA_CORRECTION_C 0.0f
#define DELTA_RADIUS_CORRECTION_A 0.0f
#define DELTA_RADIUS_CORRECTION_B 0.0f
#define DELTA_RADIUS_CORRECTION_C -0.05f
#define DELTA_HOME_OFFSET_A 2.9f
#define DELTA_HOME_OFFSET_B 0.0f
#define DELTA_HOME_OFFSET_C 0.85f

// Minimum distance between both heads
#define DUAL_X_MIN_DISTANCE 64
#define LAZY_DUAL_X_AXIS 0

// Set all directions where no explicit test is required.
// This is for dummy endstops and for hardware endstops.
// Not disabling them is just a speed penalty
#define NO_XMIN_ENDSTOP_TEST
#define NO_XMAX_ENDSTOP_TEST
#define NO_YMIN_ENDSTOP_TEST
#define NO_YMAX_ENDSTOP_TEST
#define NO_ZMIN_ENDSTOP_TEST
#define NO_ZMAX_ENDSTOP_TEST
// #define NO_AMAX_ENDSTOP_TEST

#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0

// Next 7 lines are required to make the following work, do not change!
#include "boards/pins.h"
#undef IO_TARGET
#define IO_TARGET IO_TARGET_CLASS_DEFINITION
#undef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#include "drivers/drivers.h"
#include "io/redefine.h"

// Define ZProbe by referencing a endstop defined
CONFIG_VARIABLE_EQ(EndstopDriver, *ZProbe, ZPROBE_ADDRESS)

/** Axes are homed in order of priority (0..10) if homing direction is not 0. */
#define X_HOME_PRIORITY 1
#define A_HOME_PRIORITY 1
#define Y_HOME_PRIORITY 2
#if STACKER_2_Z_END_STOPS && !STACKER_WITH_ZPROBE
#define Z_HOME_PRIORITY 0
#else
#define Z_HOME_PRIORITY 3
#endif

// All fans in this list list become controllable with M106/M107
// by selecteing the fan number with P0..P<NUM_FANS-1>
#define NUM_FANS 1 + IDEX
#if IDEX
#define FAN_LIST \
    { &Fan1PWM, &Fan2PWM }
#else
#define FAN_LIST \
    { &Fan1PWM }
#endif

#define NUM_HEATED_BEDS 1
#define HEATED_BED_LIST \
    { &HeatedBed1 }

#define NUM_HEATED_CHAMBERS 0
#define HEATED_CHAMBER_LIST \
    { }

#define SERVO_LIST \
    { }
#if IDEX
#define TOOLS \
    { &ToolExtruder1, &ToolExtruder2 }
#else
#define TOOLS \
    { &ToolExtruder1 }
#endif

// Heaters enumerate all heaters, so we can loop over them
// or call commands on a specific heater number.
// Suggested order: extruder heaters, heated beds, heated chambers, additional
// heaters
#define NUM_HEATERS 2 + IDEX
#if IDEX
#define HEATERS \
    { &HeaterExtruder1, &HeaterExtruder2, &HeatedBed1 }
#else
#define HEATERS \
    { &HeaterExtruder1, &HeatedBed1 }
#endif

// Array to call motor related commands like microstepping/current if supported.
// Id's start at 0 and depend on position in this array.
#define NUM_MOTORS 4 + IDEX + DUAL_Y
#if IDEX
#if DUAL_X
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor, &E1Motor, &E2Motor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z"), PSTR("E0"), PSTR("E1") }
#else
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor, &AMotor, &E1Motor, &E2Motor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z"), PSTR("A"), PSTR("E0"), PSTR("E1") }
#endif
#else
#if DUAL_X
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor, &AMotor, &E1Motor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z"), PSTR("A"), PSTR("E0") }
#else
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor, &E1Motor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z"), PSTR("E0") }
#endif
#endif

// Define beeper list
#if BEEPER_PIN > -1
#define NUM_BEEPERS 1
#define BEEPER_LIST \
    { &MainBeeper }
#else
#define NUM_BEEPERS 0
#define BEEPER_LIST \
    { }
#endif

// Some common settings for trinamic driver settings
/**
 Chopper timing is an array with
 {toff, hend, hstrt}
 See TMC datasheets for more details. There are some predefined values to get
 you started: CHOPPER_TIMING_DEFAULT_12V = { 3, -1, 1 }
 CHOPPER_TIMING_DEFAULT_19V = { 4, 1, 1 }
 CHOPPER_TIMING_DEFAULT_24V = { 4, 2, 1 }
 CHOPPER_TIMING_DEFAULT_36V = { 5, 2, 4 }
 CHOPPER_TIMING_PRUSAMK3_24V = { 3, -2, 6 }

*/
#define TMC_CHOPPER_TIMING CHOPPER_TIMING_DEFAULT_12V
// true = interpolate to 256 microsteps for smoother motion
#define TMC_INTERPOLATE true
// Current used when motor stands still
#define TMC_HOLD_MULTIPLIER 0.5
// Reduce current on over temperature warnings by x milli ampere, 0 = disable
#define TMC_CURRENT_STEP_DOWN 50
// Define which data should be stored to eeprom
#define STORE_MOTOR_MICROSTEPPING 1
#define STORE_MOTOR_CURRENT 1
#define STORE_MOTOR_HYBRID_TRESHOLD 1
#define STORE_MOTOR_STEALTH 1
#define STORE_MOTOR_STALL_SENSITIVITY 1

// x axis extruders are 62mm width, distance after homing 503mm

#if XL_FORMAT
#if IDEX
// Stacke F1 IDEX dimensions and bed positions
// Extra parameter in case you have a dual x axis
#define DUAL_X_LEFT_OFFSET -60.0f
#define DUAL_X_RIGHT_OFFSET 565.0f
#define X_MAX_LENGTH 505.0f
#define A_MAX_LENGTH 505.0f
#define Y_MAX_LENGTH 405.0f
#define Z_MAX_LENGTH 405.0f
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define A_MIN_POS 0
#define BED_X_MIN 0
#define BED_X_MAX 505.0f
#define BED_Y_MIN 0
#define BED_Y_MAX 405.0f
#else
// Stacker F1 XL dimensions and bed positions
#define X_MAX_LENGTH 625.0f
#define A_MAX_LENGTH 625.0f
#define Y_MAX_LENGTH 405.0f
#define Z_MAX_LENGTH 405.0f
#define X_MIN_POS -60.0f
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define A_MIN_POS 0
#define BED_X_MIN 0
#define BED_X_MAX 505.0f
#define BED_Y_MIN 0
#define BED_Y_MAX 405.0f
#endif
#else
// Stacke F1 dimensions and bed position
#define X_MAX_LENGTH 310.0f
#define A_MAX_LENGTH 310.0f
#define Y_MAX_LENGTH 291.0f
#define Z_MAX_LENGTH 285.0f
#define X_MIN_POS -5.0f
#define Y_MIN_POS -3.0f
#define Z_MIN_POS 0
#define A_MIN_POS 0
#define BED_X_MIN 0
#define BED_X_MAX 285.0f
#define BED_Y_MIN 0.0f
#define BED_Y_MAX 285.0f
#endif

// Park position used when pausing from firmware side
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
#define PARK_POSITION_X (0)
#define PARK_POSITION_Y (70)
#else
#define PARK_POSITION_X (X_MIN_POS)
#define PARK_POSITION_Y (Y_MIN_POS + Y_MAX_LENGTH)
#endif
#define PARK_POSITION_Z_RAISE 10

#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 400
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A 400
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 400
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 50
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_A 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 50

#define XAXIS_STEPS_PER_MM 146 * 2
#define AAXIS_STEPS_PER_MM 146 * 2
#define YAXIS_STEPS_PER_MM 146 * 2
#define ZAXIS_STEPS_PER_MM 400 * 2
#define MAX_FEEDRATE_X 200
#define MAX_FEEDRATE_Y 200
#define MAX_FEEDRATE_Z 12
#define MAX_FEEDRATE_A 200

//#define EXTERNALSERIAL  use Arduino serial library instead of build in.
// Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware
// made for Arduino version earlier then 1.0 If it is incompatible you will get
// compiler errors about write functions not being compatible!
//#define COMPAT_PRE1

#define RETRACT_ON_PAUSE 2
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""

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
#define JAM_ACTION 1

#define PID_CONTROL_RANGE 20
#define SKIP_M109_IF_WITHIN 5
#define TEMP_HYSTERESIS 0
#define EXTRUDE_MAXLENGTH 1000

// ############# Heated bed configuration ########################

#define SKIP_M190_IF_WITHIN 5
#define MIN_EXTRUDER_TEMP 150
#define MILLISECONDS_PREHEAT_TIME 30000

// ################ Endstop/homing configuration #####################

#define DOOR_PIN -1
#define DOOR_PULLUP 1
#define DOOR_INVERTING 1
#define ENDSTOP_X_BACK_MOVE 2
#define ENDSTOP_Y_BACK_MOVE 2
#define ENDSTOP_Z_BACK_MOVE 2
#define ENDSTOP_A_BACK_MOVE 2
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_A_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_X_BACK_ON_HOME 0.5
#define ENDSTOP_Y_BACK_ON_HOME 1
#define ENDSTOP_Z_BACK_ON_HOME STACKER_2_Z_END_STOPS
#define ENDSTOP_A_BACK_ON_HOME 0.5
#define ALWAYS_CHECK_ENDSTOPS 0
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#ifdef STACKER_2_Z_END_STOPS
#define Z_HOME_DIR 1
#else
#define Z_HOME_DIR -1
#endif
#define A_HOME_DIR 1
#define MOVE_X_WHEN_HOMED 1
#define MOVE_Y_WHEN_HOMED 1
#define MOVE_Z_WHEN_HOMED 1
#define MOVE_A_WHEN_HOMED 1
#define HOMING_FEEDRATE_X 25
#define HOMING_FEEDRATE_Y 25
#define HOMING_FEEDRATE_Z 10
#define HOMING_FEEDRATE_A 25
#define ZHOME_PRE_RAISE 1
#define ZHOME_PRE_RAISE_DISTANCE 10
#define RAISE_Z_ON_TOOLCHANGE 1
#define ZHOME_MIN_TEMPERATURE 0
#define ZHOME_HEAT_ALL 0
#define ZHOME_HEIGHT 0
#if STACKER_WITH_ZPROBE
#define FIXED_Z_HOME_POSITION 1
#define ZHOME_X_POS 50
#define ZHOME_Y_POS 50
#else
#define FIXED_Z_HOME_POSITION 0
#define ZHOME_X_POS 0
#define ZHOME_Y_POS 0
#endif

// ################# XYZ movements ###################

#define PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT 0

// ##########################################################################################
// ##                           Movement settings ##
// ##########################################################################################

// Delta settings
#define DELTA_HOME_ON_POWER 0

#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 1200L

#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define MAX_JERK 10
#define MAX_ZJERK 0.5
#define MAX_AJERK 10
#define PRINTLINE_CACHE_SIZE 32

// ################# Misc. settings ##################

#define BAUDRATE 115200
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0
#undef PS_ON_PIN
#define PS_ON_PIN -1

#define FEATURE_AXISCOMP 0
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0
// Some boards have sd support on board. These define the
// values already in pins.h
#ifndef SDSUPPORT
#define SDSUPPORT 1
#undef SDCARDDETECT
#define SDCARDDETECT ORIG_SDCARDDETECT
#define SDCARDDETECTINVERTED 0
#endif
/** Show extended directory including file length. Don't use this with 
         Pronterface! */
#define SD_EXTENDED_DIR 1
#define SD_RUN_ON_STOP ""
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 1
#define UI_PRINTER_NAME "Stacker F1"
#define UI_PRINTER_COMPANY "Stacker"
#define UI_AUTORETURN_TO_MENU_AFTER 30000

#define CASE_LIGHT_DEFAULT_ON 0
#define UI_START_SCREEN_DELAY 2000

#define NUM_MOTOR_DRIVERS 0

//#define CUSTOM_EVENTS

#endif
