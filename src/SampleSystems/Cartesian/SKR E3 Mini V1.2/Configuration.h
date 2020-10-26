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

/* 
    Work in progress sample configuration for the SKR E3 Mini v1.2.
    Configured for a Creality Ender 3 or similar bedslinger type printer,
    with an installed BLTouch and a generic Creality Ender 3 128x64 graphics
    display.

    Check out https://docfirmwarev2.repetier.com/config/introduction for more thorough 
    information on configuring RFW V2!
*/

// The follwing variables are required early to decide on the right modules.
#define NUM_TOOLS 1
#define NUM_SERVOS 1                         // Number of servos available
#define MOTHERBOARD MOTHERBOARD_E3_MINI_V1_2
#define EEPROM_MODE EEPROM_SDCARD

#define RFSERIAL SerialUSB 
#define BLUETOOTH_SERIAL -1 // Set this to 2 if you're using a BTT TFT!
#define WAITING_IDENTIFIER "wait"

#define JSON_OUTPUT 0

#define FEATURE_WATCHDOG 1
#define FEATURE_RETRACTION 1

#define NUM_AXES 4               // X,Y,Z and E for extruder A,B,C would be 5,6,7
#define STEPPER_FREQUENCY 230000 // Maximum stepper frequency.
#define PREPARE_FREQUENCY 4000   // Update frequency for new blocks. Must be higher then PREPARE_FREQUENCY.
#define BLOCK_FREQUENCY 2000     // Number of blocks with constant stepper rate per second.
#define VELOCITY_PROFILE 2       // 0 = linear, 1 = cubic, 2 = quintic velocity shape
#define SLOW_DIRECTION_CHANGE 0  // can be reason for lost steps on slow drivers
#define SMALL_SEGMENT_SIZE 0.4   // Smaller segments reduce join speed to prevent vibrations causing lost steps

#define Z_SPEED 10   // Z positioning speed
#define XY_SPEED 150 // XY positioning speed for normal operations
#define E_SPEED 5    // Extrusion speed

#define G0_FEEDRATE 0 // Speed for G0 moves. Independent from set F value! Set 0 to use F value.

#define MAX_ROOM_TEMPERATURE 25      // Minimum temperature target needed to power heaters
#define TEMPERATURE_CONTROL_RANGE 20 // Start with PID controlling if temperature is +/- this value to target temperature
#define HOST_RESCUE 1                // Enable host rescue help system
//#define DEBUG_RESCUE               // Uncomment to add power loss entry in debug menu while printing
#define POWERLOSS_LEVEL 1            // How much time do we have on powerloss, 0 = no move, 1 = short just raise Z, 2 = long full park move
#define POWERLOSS_UP 5               // How much to move up if mode 1 is active

// 0 = Cartesian, 1 = CoreXYZ, 2 = delta, 3 = Dual X-Axis
#define PRINTER_TYPE PRINTER_TYPE_CARTESIAN
// steps to include as babysteps per 1/BLOCK_FREQUENCY seconds. Must be lower then STEPPER_FREQUENCY/BLOCK_FREQUENCY and be low enough to not loose steps.
#define BABYSTEPS_PER_BLOCK \
    { 10, 10, 10 }
// If all axis end stops are hardware based we can skip the time consuming tests each step
#define NO_SOFTWARE_AXIS_ENDSTOPS
// Normally only a delta has motor end stops required. Normally you trigger using axis endstops.
#define NO_MOTOR_ENDSTOPS

#define FEATURE_CONTROLLER CONTROLLER_ENDER_3_12864
// Use more memory to speedup display updates
#define DISPLAY_FULL_BUFFER 1
// Direction 1 or -1
#define ENCODER_DIRECTION 1

// Encoder speed 0 = fastest, 1 or 2 = slowest - set so 1 click is one menu move
// Default is 2 if not set by controller. Us only to fix wrong setting

// SKR E3 Mini - Leave this at 2, we scroll quite fast already. Anymore will be hard to use.  
#define ENCODER_SPEED 2 

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

#define DELTA_DIAGONAL 350.0f
#define DELTA_HORIZONTAL_RADIUS 210.0f
#define DELTA_PRINT_RADIUS 200.0f
#define DELTA_ANGLE_A 210.0f
#define DELTA_ANGLE_B 330.0f
#define DELTA_ANGLE_C 90.0f
#define DELTA_CORRECTION_A 0.0f
#define DELTA_CORRECTION_B 0.0f
#define DELTA_CORRECTION_C 0.0f
#define DELTA_RADIUS_CORRECTION_A 0.0f
#define DELTA_RADIUS_CORRECTION_B 0.0f
#define DELTA_RADIUS_CORRECTION_C 0.0f
#define DELTA_HOME_OFFSET_A 0.0f
#define DELTA_HOME_OFFSET_B 0.0f
#define DELTA_HOME_OFFSET_C 0.0f


#define FEATURE_AXISCOMP 0
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0

// Next 7 lines are required to make the following work, do not change!
#include "boards/pins.h"
#undef IO_TARGET
#define IO_TARGET IO_TARGET_CLASS_DEFINITION
#undef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#include "drivers/drivers.h"
#include "io/redefine.h"

// Define ZProbe by referencing a endstop defined
CONFIG_VARIABLE_EQ(EndstopDriver, *ZProbe, &endstopZProbe)

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
    { }

#define SERVO_LIST \
    { &ZProbeServo }

#define TOOLS \
    { &ToolExtruder1 }

// Heaters enumerate all heaters, so we can loop over them
// or call commands on a specific heater number.
// Suggested order: extruder heaters, heated beds, heated chambers, additional heaters
#define NUM_HEATERS 2
#define HEATERS \
    { &HeaterExtruder1, &HeatedBed1 }

// Array to call motor related commands like microstepping/current if supported.
// Id's start at 0 and depend on position in this array.
#define NUM_MOTORS 4
#define MOTORS \
    { &XMotor, &YMotor, &ZMotor, &E1Motor }
#define MOTOR_NAMES \
    { PSTR("X"), PSTR("Y"), PSTR("Z"), PSTR("E0") }

#define DEFAULT_TONE_VOLUME 85 // Default volume in percentage - (May not be fully linear)
#define NUM_BEEPERS 1
#define BEEPER_LIST \
    { &MainBeeper }

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
#define TMC_CURRENT_STEP_DOWN 150
// Define which data should be stored to eeprom
#define STORE_MOTOR_MICROSTEPPING 1
#define STORE_MOTOR_CURRENT 1
#define STORE_MOTOR_HYBRID_TRESHOLD 1
#define STORE_MOTOR_STEALTH 1
#define STORE_MOTOR_STALL_SENSITIVITY 1

#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define X_MAX_LENGTH 235
#define Y_MAX_LENGTH 235
#define Z_MAX_LENGTH 250
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0

#define BED_X_MIN X_MIN_POS
#define BED_X_MAX (X_MIN_POS + X_MAX_LENGTH)
#define BED_Y_MIN Y_MIN_POS
#define BED_Y_MAX (Y_MIN_POS + Y_MAX_LENGTH)

// Park position used when pausing from firmware side
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
#define PARK_POSITION_X (0)
#define PARK_POSITION_Y (70)
#else
#define PARK_POSITION_X (X_MIN_POS)
#define PARK_POSITION_Y (Y_MIN_POS + Y_MAX_LENGTH)
#endif

#define PARK_POSITION_Z_RAISE 10
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 800
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 800
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100

#define XAXIS_STEPS_PER_MM 80
#define YAXIS_STEPS_PER_MM 80
#define ZAXIS_STEPS_PER_MM 400

// ################## EDIT THESE SETTINGS MANUALLY ################
// ################ END MANUAL SETTINGS ##########################

#define RETRACT_ON_PAUSE 2
#define PAUSE_START_COMMANDS ""
#define PAUSE_END_COMMANDS ""

#define EXTRUDE_MAXLENGTH 160

// ############# Heated bed configuration ########################

#define MIN_EXTRUDER_TEMP 150

// ################ Endstop configuration #####################

#define ENDSTOP_X_BACK_MOVE 3
#define ENDSTOP_Y_BACK_MOVE 3
#define ENDSTOP_Z_BACK_MOVE 0

#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 2

#define ENDSTOP_X_BACK_ON_HOME 0.5
#define ENDSTOP_Y_BACK_ON_HOME 0.5
#define ENDSTOP_Z_BACK_ON_HOME 0

#define ALWAYS_CHECK_ENDSTOPS 0
#define MOVE_X_WHEN_HOMED 0
#define MOVE_Y_WHEN_HOMED 0
#define MOVE_Z_WHEN_HOMED 0

// ################# XYZ movements ###################

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT 1

#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 1200L

#define MAX_FEEDRATE_X 500
#define MAX_FEEDRATE_Y 500
#define MAX_FEEDRATE_Z 5

#define HOMING_FEEDRATE_X 80
#define HOMING_FEEDRATE_Y 80
#define HOMING_FEEDRATE_Z 10

#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0

#define MAX_JERK 5 // Note: Jerk has been renamed to "Yank" in V2!
#define MAX_ZJERK 0.3
#define PRINTLINE_CACHE_SIZE 32

// ################# Misc. settings ##################

#define BAUDRATE 115200
#define KILL_METHOD 1
#define ACK_WITH_LINENUMBER 1
#define KEEP_ALIVE_INTERVAL 2000
#define ECHO_ON_EXECUTE 1

#undef PS_ON_PIN
#define PS_ON_PIN -1
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0

// #################### Z-Probing #####################

// Depending on the here selected z probe type some options may be ignored.
// Z_PROBE_TYPE_NONE - No z probe available
// Z_PROBE_TYPE_DEFAULT - Default z probe triggers on a pin change
// Z_PROBE_TYPE_NOZZLE - Nozzle is z probe. Supports heating before probing.

// Z_PROBE_TYPE_BLTOUCH - BLtouch/3DTouch z probe. Contains logic for pin.
// Z_PROBE_BLTOUCH_DEPLOY_DELAY - Special delay to wait while it's pin deploys before probing. Defaults to 1s.

#define Z_PROBE_TYPE Z_PROBE_TYPE_BLTOUCH
#define Z_PROBE_BLTOUCH_DEPLOY_DELAY 150 // Increase this if you have an older BLTouch!

#define Z_PROBE_HEIGHT 3             // Distance bed-nozzle when trigger switches
#define Z_PROBE_BED_DISTANCE 10      // Optimal starting distance
#define Z_PROBE_SPEED 5              // Speed fo z testing
#define Z_PROBE_X_OFFSET 0           // x offset relative to extruder 0,0 offset
#define Z_PROBE_Y_OFFSET 26          // y offset relative to extruder 0,0 offset
#define Z_PROBE_COATING 0            // Coating thickness if not detected by probe
#define Z_PROBE_DELAY 0              // Extra delay before starting again. Only needed on electronic probes keeping state for a while
#define Z_PROBE_REPETITIONS 1        // How often should we probe, 1 is minimum
#define Z_PROBE_USE_MEDIAN 0         // 0 = use average, 1 = use middle value after ordering z
#define Z_PROBE_SWITCHING_DISTANCE 3 // Minimum distance required to safely untrigger probe - used for faster repeated measurement
#define Z_PROBE_BORDER 2             // Safety border to ensure position is allowed
#define Z_PROBE_PAUSE_HEATERS 0      // Pause all heaters when probing to reduce EMI artifacts
#define Z_PROBE_TEMPERATURE 0        // Temperature for Z_PROBE_TYPE_NOZZLE

#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""
#define Z_PROBE_RUN_AFTER_EVERY_PROBE ""

// Raise z before homing
#define ZHOME_PRE_RAISE 2 // This is a flag, not a unit! just leave at 2
// How much mm should z raise before homing
#define ZHOME_PRE_RAISE_DISTANCE 3
// Height in mm after homing.
#define ZHOME_HEIGHT 10
// Home Z at a fixed xy position (1)
#define FIXED_Z_HOME_POSITION 1
#define ZHOME_X_POS 100
#define ZHOME_Y_POS 100

// How to correct rotated beds
// 0 = Software side by rotating coordinates
// 1 = Move bed physically using 2 motors
#define LEVELING_CORRECTOR LEVELING_CORRECTOR_SOFTWARE

// Leveling method
// 0 = none, 3 = 3 points, 1 = grid, 2 = 4 point symmetric
#define LEVELING_METHOD 1
#define MAX_GRID_SIZE 15                  // Maximum grid size allocation in memory, imported grid can be smaller
#define ENABLE_BUMP_CORRECTION 1          // CPU intensive, so only activate if required
#define BUMP_CORRECTION_START_DEGRADE 0.5 // Until this height we correct 100%
#define BUMP_CORRECTION_END_HEIGHT 2      // From this height on we do no correction
#define BUMP_LIMIT_TO 0                   // Maximum allowed correction up/down, <= 0 off.

#define SD_MENU_SHOW_HIDDEN_FILES 0
#define SD_MENU_CACHE_SCROLL_ENTRIES 1 // Cache filenames while scrolling for better performance/infinite scroll

#define SD_SPI_SPEED_MHZ 30 // SD Card HW-SPI Speed in MHz. SD Card "High Speed" mode is 50Mhz, but you may be
                            // limited to less based on your processor.

#define SD_EXTENDED_DIR 1 /* Show extended directory including file length. Don't use this with Pronterface! */
#define SD_RUN_ON_STOP ""
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 1

#define ARC_SUPPORT 0

#define UI_PRINTER_NAME "SKR E3 Mini V1.2"
#define UI_PRINTER_COMPANY "BIGTREETECH"

#define UI_AUTORETURN_TO_MENU_AFTER 30000
#define CASE_LIGHT_DEFAULT_ON 1
#define UI_START_SCREEN_DELAY 4000 // This can be skipped by pressing any button

//#define CUSTOM_EVENTS



// Unimplemented
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0

/* // Not currently implemented yet
#define JAM_METHOD 1 
#define JAM_STEPS 220
#define JAM_SLOWDOWN_STEPS 1000
#define JAM_SLOWDOWN_TO 75
#define JAM_ERROR_STEPS 1500
#define JAM_MIN_STEPS 10
#define JAM_ACTION 1
*/

#endif