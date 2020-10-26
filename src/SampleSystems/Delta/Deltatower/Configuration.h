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
#define NUM_TOOLS 2
#define NUM_SERVOS 0
#define NUM_EXTRUDER 2
#define MOTHERBOARD MOTHERBOARD_RADDS
#define EEPROM_MODE 1
#define RFSERIAL Serial
#define BLUETOOTH_SERIAL 101
#define BLUETOOTH_BAUD 115200
#define WAITING_IDENTIFIER "wait"
#define JSON_OUTPUT 1
#define FEATURE_WATCHDOG 1
#define FEATURE_RETRACTION 1
#define NUM_AXES 4                   // X,Y,Z and E for extruder A,B,C would be 5,6,7
#define STEPPER_FREQUENCY 100000     // Maximum stepper frequency.
#define PREPARE_FREQUENCY 1000       // Update frequency for new blocks. Must be higher then PREPARE_FREQUENCY.
#define BLOCK_FREQUENCY 500          // Number of blocks with constant stepper rate per second.
#define VELOCITY_PROFILE 2           // 0 = linear, 1 = cubic, 2 = quintic velocity shape
#define SLOW_DIRECTION_CHANGE 1      // can be reason for lost steps on slow drivers
#define SMALL_SEGMENT_SIZE 0.4       // Smaller segments reduce join speed to prevent vibrations causing lost steps
#define Z_SPEED 100                  // Z positioning speed
#define XY_SPEED 100                 // XY positioning speed for normal operations
#define MAX_ROOM_TEMPERATURE 25      // No heating below this temperature!
#define TEMPERATURE_CONTROL_RANGE 20 // Start with controlling if temperature is +/- this value to target temperature
#define HOST_RESCUE 1                // Enable host rescue help system
//#define DEBUG_RESCUE                 // Uncomment to add power loss entry in debug menu while printing
#define POWERLOSS_LEVEL 2     // How much time do we have on powerloss, 0 = no move, 1 = short just raise Z, 2 = long full park move
#define POWERLOSS_UP 5        // How much to move up if mode 1 is active
#define Z_PROBE_TYPE 1        // 0 = no z probe, 1 = default z probe, 2 = Nozzle as probe
#define Z_PROBE_BORDER 2      // Safety border to ensure position is allowed
#define Z_PROBE_TEMPERATURE 0 // Temperature for type 2
// Set 1 if you want to replace the default themes and define them in configuration_io.h
#define CUSTOM_DEFAULT_THEMES 0

// 0 = Cartesian, 1 = CoreXYZ, 2 = delta
#define PRINTER_TYPE PRINTER_TYPE_DELTA
// steps to include as babysteps per 1/BLOCK_FREQUENCY seconds. Must be lower then STEPPER_FREQUENCY/BLOCK_FREQUENCY and be low enough to not loose steps.
#define BABYSTEPS_PER_BLOCK \
    { 1, 1, 1 }
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
            |                        |   \------------------------------------------
            |_________ X-axis        |    \                                        |
           / \                       |     \  DELTA_DIAGONAL (length)    Each move this Rod Height
          /   \                             \                                 is calculated
         /     \                             \    Carriage is at printer center!   |
         A      B                             \_____/--------------------------------
                                              |--| End effector horizontal offset (recommend set it to 0)
                                         |----| DELTA_HORIZONTAL_RADIUS (Horizontal rod pivot to pivot measure)

    Column angles are measured from X-axis counterclockwise
    "Standard" positions: alpha_A = 210, alpha_B = 330, alpha_C = 90
*/
#define DELTA_DIAGONAL 444.800f
#define DELTA_HORIZONTAL_RADIUS 209.900f
#define DELTA_PRINT_RADIUS 209.0f
#define BED_RADIUS 200.0f
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

// Set all directions where no explicit test is required.
// This is for dummy endstops and for hardware endstops.
// NOt disabling them is just a speed penalty
#define NO_XMIN_ENDSTOP_TEST
#define NO_XMAX_ENDSTOP_TEST
#define NO_YMIN_ENDSTOP_TEST
#define NO_YMAX_ENDSTOP_TEST
#define NO_ZMIN_ENDSTOP_TEST
#define NO_ZMAX_ENDSTOP_TEST

#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0

#define FEATURE_AXISCOMP 1
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0

// Next 7 lines are required to make the following work, do not change!
#include "boards/pins.h"
#undef IO_TARGET
#define IO_TARGET 4
#undef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#include "drivers/drivers.h"
#include "io/redefine.h"

// Define ZProbe by referencing a endstop defined
CONFIG_VARIABLE_EQ(EndstopDriver, *ZProbe, ZPROBE_ADDRESS)

/** Axes are homed in order of priority (0..10) if homing direction is not 0. */
#define X_HOME_PRIORITY 0
#define Y_HOME_PRIORITY 1
#define Z_HOME_PRIORITY 2

// All fans in this list list become controllable with M106/M107
// by selecteing the fan number with P0..P<NUM_FANS-1>
#define NUM_FANS 1
#define FAN_LIST \
    { &Fan1PWM }

#define NUM_HEATED_BEDS 1
#define HEATED_BED_LIST \
    { &HeatedBed1 }

#define NUM_HEATED_CHAMBERS 0
#define HEATED_CHAMBER_LIST \
    {}

#define TOOLS \
    { &ToolExtruder1, &ToolExtruder2 }

// Heaters enumerate all heaters, so we can loop over them
// or call commands on a specific heater number.
// Suggested order: extruder heaters, heated beds, heated chambers, additional heaters
#define NUM_HEATERS 3
#define HEATERS \
    { &HeaterExtruder1, &HeaterExtruder2, &HeatedBed1 }

#define SERVO_LIST \
    {}

// Array to call motor related commands like microstepping/current if supported.
// Id's start at 0 and depend on position in this array.
#define NUM_MOTORS 3
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z") }
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
 See TMC datasheets for more details. There are some predefined values to get you started:
 CHOPPER_TIMING_DEFAULT_12V = { 3, -1, 1 }
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

#define X_MAX_LENGTH 210
#define Y_MAX_LENGTH 210
#define Z_MAX_LENGTH 595.596
#define X_MIN_POS -210
#define Y_MIN_POS -210
#define Z_MIN_POS 0
// Park position used when pausing from firmware side
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
#define PARK_POSITION_X (0)
#define PARK_POSITION_Y (70)
#else
#define PARK_POSITION_X (X_MIN_POS)
#define PARK_POSITION_Y (Y_MIN_POS + Y_MAX_LENGTH)
#endif
#define PARK_POSITION_Z_RAISE 10

#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 600
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 600
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 600
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 600
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 600
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 600
#define XAXIS_STEPS_PER_MM 80
#define YAXIS_STEPS_PER_MM 80
#define ZAXIS_STEPS_PER_MM 80

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

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1
#define MIXING_EXTRUDER 0

#define DRIVE_SYSTEM 0
#define EXTRUDER_FAN_COOL_TEMP 50
#define KILL_IF_SENSOR_DEFECT 0
#define RETRACT_ON_PAUSE 2
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""
#define SHARED_EXTRUDER_HEATER 0

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

#define GENERIC_THERM_VREF 5
#define GENERIC_THERM_NUM_ENTRIES 33
#define HEATER_PWM_SPEED 0
#define COOLER_PWM_SPEED 0

// ############# Heated bed configuration ########################

#define SKIP_M190_IF_WITHIN 5
#define MIN_EXTRUDER_TEMP 150
#define MILLISECONDS_PREHEAT_TIME 30000

// ################ Endstop/homing configuration #####################

#define DOOR_PIN -1
#define DOOR_PULLUP 1
#define DOOR_INVERTING 1
#define ENDSTOP_X_BACK_MOVE 30
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 10
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_X_BACK_ON_HOME 0.5
#define ENDSTOP_Y_BACK_ON_HOME 0.5
#define ENDSTOP_Z_BACK_ON_HOME 20
#define ALWAYS_CHECK_ENDSTOPS 0
#define X_HOME_DIR 0
#define Y_HOME_DIR 0
#define Z_HOME_DIR 1
#define MOVE_X_WHEN_HOMED 0
#define MOVE_Y_WHEN_HOMED 0
#define MOVE_Z_WHEN_HOMED 0
#define HOMING_FEEDRATE_X 10
#define HOMING_FEEDRATE_Y 10
#define HOMING_FEEDRATE_Z 10
#define ZHOME_PRE_RAISE 0
#define ZHOME_PRE_RAISE_DISTANCE 10
#define RAISE_Z_ON_TOOLCHANGE 0
#define ZHOME_MIN_TEMPERATURE 0
#define ZHOME_HEAT_ALL 0
#define ZHOME_HEIGHT 590
#define FIXED_Z_HOME_POSITION 0
#define ZHOME_X_POS 140
#define ZHOME_Y_POS 45

// ################# XYZ movements ###################

#define PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT 0

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define FEATURE_BABYSTEPPING 1
#define BABYSTEP_MULTIPLICATOR 64

// Delta settings
#define DELTA_HOME_ON_POWER 0

#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 1200L
#define MAX_FEEDRATE_X 100
#define MAX_FEEDRATE_Y 100
#define MAX_FEEDRATE_Z 100

#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define RAMP_ACCELERATION 1
#define MAX_JERK 5
#define MAX_ZJERK 0.3
#define PRINTLINE_CACHE_SIZE 32
#define LOW_TICKS_PER_MOVE 250000

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

// #################### Z-Probing #####################

#define Z_PROBE_COATING 0
#define Z_PROBE_Z_OFFSET 0.05
#define Z_PROBE_Z_OFFSET_MODE 1
#define UI_BED_COATING 1
#define EXTRUDER_IS_Z_PROBE 0
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

// Leveling method
// 0 = none, 3 = 3 points, 1 = grid, 2 = 4 point symmetric
#define LEVELING_METHOD 2
#define L_P1_X 60
#define L_P1_Y 130
#define L_P2_X 137
#define L_P2_Y 45
#define L_P3_X 137
#define L_P3_Y 210
#define MAX_GRID_SIZE 5                   // Maximum grid size allocation in memory, imported grid can be smaller
#define ENABLE_BUMP_CORRECTION 1          // CPU intensive, so only activate if required
#define BUMP_CORRECTION_START_DEGRADE 0.5 // Until this height we correct 100%
#define BUMP_CORRECTION_END_HEIGHT 2      // From this height on we do no correction
#define BUMP_LIMIT_TO 0                   // Maximum allowed correction up/down, <= 0 off.

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

#define NUM_MOTOR_DRIVERS 0
//#define MOTOR_DRIVER_1(var) StepperDriver<51, 53, 49, 0, 0> var(3382, 0.2)
//#define MOTOR_DRIVER_2(var) StepperDriver<39, 13, 40, 0, 0> var(3382, 0.2)

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

#endif
