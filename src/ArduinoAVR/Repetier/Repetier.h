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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#ifndef _REPETIER_H
#define _REPETIER_H

#include <math.h>
#include <stdint.h>
//#define REPETIER_VERSION "0.92.10"
#define REPETIER_VERSION "1.0.1"

// Use new communication model for multiple channels - only until stable, then old version gets deleted
#define NEW_COMMUNICATION 1
// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################
// These are run time switchable debug flags
enum debugFlags {DEB_ECHO = 0x1, DEB_INFO = 0x2, DEB_ERROR = 0x4,DEB_DRYRUN = 0x8,
                 DEB_COMMUNICATION = 0x10, DEB_NOMOVES = 0x20, DEB_DEBUG = 0x40
                };

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE
/** write infos about path planner changes */
//#define DEBUG_PLANNER
/** Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data throughput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION 1
// Echo all ascii commands after receiving
//#define DEBUG_ECHO_ASCII
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE 1
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for searching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE
/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT
/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
//#define DEBUG_COM_ERRORS
/** Adds a menu point in quick settings to write debug informations to the host in case of hangs where the ui still works. */
//#define DEBUG_PRINT
//#define DEBUG_DELTA_OVERFLOW
//#define DEBUG_DELTA_REALPOS
//#define DEBUG_SPLIT
//#define DEBUG_JAM
// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH
// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK
// Debug reason for not mounting a sd card
//#define DEBUG_SD_ERROR
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG

#define DEBUG_MSG(x) {if(Printer::debugEcho()) { Com::printFLN(PSTR(x));HAL::delayMilliseconds(20);}}
#define DEBUG_MSG2(x,y) {if(Printer::debugEcho()) {Com::printFLN(PSTR(x),y);HAL::delayMilliseconds(20);}}
#define DEBUG_MSG_FAST(x) {if(Printer::debugEcho()) {Com::printFLN(PSTR(x));}}
#define DEBUG_MSG2_FAST(x,y) {if(Printer::debugEcho()) {Com::printFLN(PSTR(x),y);}}

#define CARTESIAN 0
#define XY_GANTRY 1
#define YX_GANTRY 2
#define DELTA 3
#define TUGA 4
#define BIPOD 5
#define XZ_GANTRY 8
#define ZX_GANTRY 9
#define GANTRY_FAKE 10

#define WIZARD_STACK_SIZE 8
#define IGNORE_COORDINATE 999999

#define IS_MAC_TRUE(x) (x!=0)
#define IS_MAC_FALSE(x) (x==0)
#define HAS_PIN(x) (defined( x ## _PIN) && x > -1)

// Uncomment if no analyzer is connected
//#define ANALYZER
// Channel->pin assignments
#define ANALYZER_CH0 63 // New move
#define ANALYZER_CH1 40 // Step loop
#define ANALYZER_CH2 53 // X Step
#define ANALYZER_CH3 65 // Y Step
#define ANALYZER_CH4 59 // X Direction
#define ANALYZER_CH5 64 // Y Direction
#define ANALYZER_CH6 58 // xsig
#define ANALYZER_CH7 57 // ysig

#ifdef ANALYZER
#define ANALYZER_ON(a) {WRITE(a,HIGH);}
#define ANALYZER_OFF(a) {WRITE(a,LOW);}
#else
#define ANALYZER_ON(a)
#define ANALYZER_OFF(a)
#endif

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define VIRTUAL_AXIS 4
// How big an array to hold X_AXIS..<MAX_AXIS>
#define Z_AXIS_ARRAY 3
#define E_AXIS_ARRAY 4
#define VIRTUAL_AXIS_ARRAY 5


#define A_TOWER 0
#define B_TOWER 1
#define C_TOWER 2
#define TOWER_ARRAY 3
#define E_TOWER_ARRAY 4

#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF ANALOG_REF_AVCC

#define HOME_ORDER_XYZ 1
#define HOME_ORDER_XZY 2
#define HOME_ORDER_YXZ 3
#define HOME_ORDER_YZX 4
#define HOME_ORDER_ZXY 5
#define HOME_ORDER_ZYX 6
#define HOME_ORDER_ZXYTZ 7 // Needs hot hotend for correct homing
#define HOME_ORDER_XYTZ 8 // Needs hot hotend for correct homing

#define NO_CONTROLLER 0
#define UICONFIG_CONTROLLER 1
#define CONTROLLER_SMARTRAMPS 2
#define CONTROLLER_ADAFRUIT 3
#define CONTROLLER_FOLTYN 4
#define CONTROLLER_VIKI 5
#define CONTROLLER_MEGATRONIC 6
#define CONTROLLER_RADDS 7
#define CONTROLLER_PIBOT20X4 8
#define CONTROLLER_PIBOT16X2 9
#define CONTROLLER_GADGETS3D_SHIELD 10
#define CONTROLLER_REPRAPDISCOUNT_GLCD 11
#define CONTROLLER_FELIX 12
#define CONTROLLER_RAMBO 13
#define CONTROLLER_OPENHARDWARE_LCD2004 14
#define CONTROLLER_SANGUINOLOLU_PANELOLU2 15
#define CONTROLLER_GAMEDUINO2 16
#define CONTROLLER_MIREGLI 17
#define CONTROLLER_GATE_3NOVATICA 18
#define CONTROLLER_SPARKLCD 19
#define CONTROLLER_BAM_DICE_DUE 20
#define CONTROLLER_VIKI2 21
#define CONTROLLER_LCD_MP_PHARAOH_DUE 22
#define CONTROLLER_SPARKLCD_ADAPTER 23
#define CONTROLLER_ZONESTAR 24
#define CONTROLLER_FELIX_DUE 405
#define CONTROLLER_ORCABOTXXLPRO2 25
#define CONTROLLER_AZSMZ_12864 26
#define CONTROLLER_REPRAPWORLD_GLCD 27

//direction flags
#define X_DIRPOS 1
#define Y_DIRPOS 2
#define Z_DIRPOS 4
#define E_DIRPOS 8
#define XYZ_DIRPOS 7
//step flags
#define XSTEP 16
#define YSTEP 32
#define ZSTEP 64
#define ESTEP 128
//combo's
#define XYZ_STEP 112
#define XY_STEP 48
#define XYZE_STEP 240
#define E_STEP_DIRPOS 136
#define Y_STEP_DIRPOS 34
#define X_STEP_DIRPOS 17
#define Z_STEP_DIRPOS 68

#define PRINTER_MODE_FFF 0
#define PRINTER_MODE_LASER 1
#define PRINTER_MODE_CNC 2

#define ILLEGAL_Z_PROBE -888

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "Configuration.h"

#if (LASER_PWM_MAX > 255 && SUPPORT_LASER) || (CNC_PWM_MAX > 255 && SUPPORT_CNC)
typedef uint16_t secondspeed_t;
#else
typedef uint8_t secondspeed_t;
#endif

#ifndef SHARED_EXTRUDER_HEATER
#define SHARED_EXTRUDER_HEATER 0
#endif

#ifndef DUAL_X_AXIS
#define DUAL_X_AXIS 0
#endif

#ifndef LAZY_DUAL_X_AXIS
#define LAZY_DUAL_X_AXIS 0
#endif

#ifndef MOVE_X_WHEN_HOMED
#define MOVE_X_WHEN_HOMED 0
#endif
#ifndef MOVE_Y_WHEN_HOMED
#define MOVE_Y_WHEN_HOMED 0
#endif
#ifndef MOVE_Z_WHEN_HOMED
#define MOVE_Z_WHEN_HOMED 0
#endif

#if SHARED_EXTRUDER_HEATER || MIXING_EXTRUDER
#undef EXT1_HEATER_PIN
#undef EXT2_HEATER_PIN
#undef EXT3_HEATER_PIN
#undef EXT4_HEATER_PIN
#undef EXT5_HEATER_PIN
#define EXT1_HEATER_PIN -1
#define EXT2_HEATER_PIN -1
#define EXT3_HEATER_PIN -1
#define EXT4_HEATER_PIN -1
#define EXT5_HEATER_PIN -1
#endif

#ifndef BOARD_FAN_SPEED
#define BOARD_FAN_SPEED
#endif

#ifndef MAX_JERK_DISTANCE
#define MAX_JERK_DISTANCE 0.6
#endif

#if defined(FAST_COREXYZ) && !(DRIVE_SYSTEM==XY_GANTRY || DRIVE_SYSTEM==YX_GANTRY || DRIVE_SYSTEM==XZ_GANTRY || DRIVE_SYSTEM==ZX_GANTRY || DRIVE_SYSTEM==GANTRY_FAKE)
#undef FAST_COREXYZ
#endif
#ifdef FAST_COREXYZ
#if DELTA_SEGMENTS_PER_SECOND_PRINT > 30
#undef DELTA_SEGMENTS_PER_SECOND_PRINT
#define DELTA_SEGMENTS_PER_SECOND_PRINT 30 // core is linear, no subsegments needed
#endif
#if DELTA_SEGMENTS_PER_SECOND_MOVE > 30
#undef DELTA_SEGMENTS_PER_SECOND_MOVE
#define DELTA_SEGMENTS_PER_SECOND_MOVE 30
#endif
#endif

inline void memcopy2(void *dest,void *source) {
	*((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void *dest,void *source) {
	*((int32_t*)dest) = *((int32_t*)source);
}

#ifndef JSON_OUTPUT
#define JSON_OUTPUT 0
#endif

#if !defined(ZPROBE_MIN_TEMPERATURE) && defined(ZHOME_MIN_TEMPERATURE)
#define ZPROBE_MIN_TEMPERATURE ZHOME_MIN_TEMPERATURE
#endif

#if FEATURE_Z_PROBE && Z_PROBE_PIN < 0
#error You need to define Z_PROBE_PIN to use z probe!
#endif

#if DISTORTION_CORRECTION
#if !FEATURE_Z_PROBE
#error Distortion correction requires the z probe feature to be enabled and configured!
#endif
#endif

#ifndef MAX_ROOM_TEMPERATURE
#define MAX_ROOM_TEMPERATURE 40
#endif
#ifndef ZHOME_X_POS
#define ZHOME_X_POS IGNORE_COORDINATE
#endif
#ifndef ZHOME_Y_POS
#define ZHOME_Y_POS IGNORE_COORDINATE
#endif

// MS1 MS2 Stepper Driver Micro stepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#if (MOTHERBOARD == 501) || MOTHERBOARD==502
#define MICROSTEP16 LOW,LOW
#else
#define MICROSTEP16 HIGH,HIGH
#endif
#define MICROSTEP32 HIGH,HIGH

#define GCODE_BUFFER_SIZE 1

#ifndef FEATURE_BABYSTEPPING
#define FEATURE_BABYSTEPPING 0
#define BABYSTEP_MULTIPLICATOR 1
#endif

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

#ifndef MINMAX_HARDWARE_ENDSTOP_Z2
#define MINMAX_HARDWARE_ENDSTOP_Z2 0
#define Z2_MINMAX_PIN -1
#endif

#if MINMAX_HARDWARE_ENDSTOP_Z2 && Z2_MINMAX_PIN > -1
#undef MULTI_ZENDSTOP_HOMING 
#define MULTI_ZENDSTOP_HOMING 1
#define MULTI_ZENDSTOP_ALL 3
#else
#define MULTI_ZENDSTOP_HOMING 0
#endif

#if (X_HOME_DIR < 0 && HAS_PIN(X2_MIN) && MIN_HARDWARE_ENDSTOP_X2) || (X_HOME_DIR > 0 && HAS_PIN(X2_MAX) && MAX_HARDWARE_ENDSTOP_X2)
#define MULTI_XENDSTOP_HOMING 1
#define MULTI_XENDSTOP_ALL 3
#else
#define MULTI_XENDSTOP_HOMING 0
#endif

#if (Y_HOME_DIR < 0 && HAS_PIN(Y2_MIN) && MIN_HARDWARE_ENDSTOP_Y2) || (Y_HOME_DIR > 0 && HAS_PIN(Y2_MAX) && MAX_HARDWARE_ENDSTOP_Y2)
#define MULTI_YENDSTOP_HOMING 1
#define MULTI_YENDSTOP_ALL 3
#else
#define MULTI_YENDSTOP_HOMING 0
#endif

#define SPEED_MIN_MILLIS 400
#define SPEED_MAX_MILLIS 60
#define SPEED_MAGNIFICATION 100.0f

#define SOFTWARE_LEVELING ((FEATURE_SOFTWARE_LEVELING) && (DRIVE_SYSTEM==DELTA))
/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/
#if !defined(ROD_RADIUS) && DRIVE_SYSTEM == DELTA
#define ROD_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)
#endif

#ifndef UI_SPEEDDEPENDENT_POSITIONING
#define UI_SPEEDDEPENDENT_POSITIONING 1
#endif

#if DRIVE_SYSTEM==DELTA || DRIVE_SYSTEM==TUGA || DRIVE_SYSTEM==BIPOD || defined(FAST_COREXYZ)
#define NONLINEAR_SYSTEM 1
#else
#define NONLINEAR_SYSTEM 0
#endif

#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL 1
#endif

#define GANTRY ( DRIVE_SYSTEM==XY_GANTRY || DRIVE_SYSTEM==YX_GANTRY || DRIVE_SYSTEM==XZ_GANTRY || DRIVE_SYSTEM==ZX_GANTRY || DRIVE_SYSTEM==GANTRY_FAKE)

//Step to split a circle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS calculation is started to correct the circle interpolation
#define N_ARC_CORRECTION 25

// Test for shared cooler
#if NUM_EXTRUDER == 6 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT4_EXTRUDER_COOLER_PIN == EXT5_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT4_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 5 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT3_EXTRUDER_COOLER_PIN == EXT5_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 4 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 3 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT0_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 2 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#else
#define SHARED_COOLER 0
#endif

#ifndef START_STEP_WITH_HIGH
#define START_STEP_WITH_HIGH 1
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE == 101
#define SUPPORT_MAX6675
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE == 102
#define SUPPORT_MAX31855
#endif

// Test for shared coolers between extruders and mainboard
#if EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == FAN_BOARD_PIN
#define SHARED_COOLER_BOARD_EXT 1
#else
#define SHARED_COOLER_BOARD_EXT 0
#endif

#if defined(UI_SERVO_CONTROL) && UI_SERVO_CONTROL > FEATURE_SERVO
#undef UI_SERVO_CONTROL
#define UI_SERVO_CONTROL FEATURE_SERVO
#endif

#if (defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1) || (defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1) || (defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1) || (defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1) || (defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1) || (defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1)
#define EXTRUDER_JAM_CONTROL 1
#else
#define EXTRUDER_JAM_CONTROL 0
#endif
#ifndef JAM_METHOD
#define JAM_METHOD 1
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE < 101
#define EXT0_ANALOG_INPUTS 1
#define EXT0_SENSOR_INDEX 0
#define EXT0_ANALOG_CHANNEL EXT0_TEMPSENSOR_PIN
#define ACCOMMA0 ,
#else
#define ACCOMMA0
#define EXT0_ANALOG_INPUTS 0
#define EXT0_SENSOR_INDEX EXT0_TEMPSENSOR_PIN
#define EXT0_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 1 && EXT1_TEMPSENSOR_TYPE < 101
#define EXT1_ANALOG_INPUTS 1
#define EXT1_SENSOR_INDEX EXT0_ANALOG_INPUTS
#define EXT1_ANALOG_CHANNEL ACCOMMA0 EXT1_TEMPSENSOR_PIN
#define ACCOMMA1 ,
#else
#define ACCOMMA1 ACCOMMA0
#define EXT1_ANALOG_INPUTS 0
#define EXT1_SENSOR_INDEX EXT1_TEMPSENSOR_PIN
#define EXT1_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 2 && EXT2_TEMPSENSOR_TYPE < 101
#define EXT2_ANALOG_INPUTS 1
#define EXT2_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS
#define EXT2_ANALOG_CHANNEL ACCOMMA1 EXT2_TEMPSENSOR_PIN
#define ACCOMMA2 ,
#else
#define ACCOMMA2 ACCOMMA1
#define EXT2_ANALOG_INPUTS 0
#define EXT2_SENSOR_INDEX EXT2_TEMPSENSOR_PIN
#define EXT2_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 3 && EXT3_TEMPSENSOR_TYPE < 101
#define EXT3_ANALOG_INPUTS 1
#define EXT3_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS
#define EXT3_ANALOG_CHANNEL ACCOMMA2 EXT3_TEMPSENSOR_PIN
#define ACCOMMA3 ,
#else
#define ACCOMMA3 ACCOMMA2
#define EXT3_ANALOG_INPUTS 0
#define EXT3_SENSOR_INDEX EXT3_TEMPSENSOR_PIN
#define EXT3_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 4 && EXT4_TEMPSENSOR_TYPE < 101
#define EXT4_ANALOG_INPUTS 1
#define EXT4_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS
#define EXT4_ANALOG_CHANNEL ACCOMMA3 EXT4_TEMPSENSOR_PIN
#define ACCOMMA4 ,
#else
#define ACCOMMA4 ACCOMMA3
#define EXT4_ANALOG_INPUTS 0
#define EXT4_SENSOR_INDEX EXT4_TEMPSENSOR_PIN
#define EXT4_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 5 && EXT5_TEMPSENSOR_TYPE < 101
#define EXT5_ANALOG_INPUTS 1
#define EXT5_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS
#define EXT5_ANALOG_CHANNEL ACCOMMA4 EXT5_TEMPSENSOR_PIN
#define ACCOMMA5 ,
#else
#define ACCOMMA5 ACCOMMA4
#define EXT5_ANALOG_INPUTS 0
#define EXT5_SENSOR_INDEX EXT5_TEMPSENSOR_PIN
#define EXT5_ANALOG_CHANNEL
#endif

#if HAVE_HEATED_BED && HEATED_BED_SENSOR_TYPE < 101
#define BED_ANALOG_INPUTS 1
#define BED_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS
#define BED_ANALOG_CHANNEL ACCOMMA5 HEATED_BED_SENSOR_PIN
#define BED_KOMMA ,
#else
#define BED_ANALOG_INPUTS 0
#define BED_SENSOR_INDEX HEATED_BED_SENSOR_PIN
#define BED_ANALOG_CHANNEL
#define BED_KOMMA ACCOMMA5
#endif

#if FAN_THERMO_THERMISTOR_PIN > -1 && FAN_THERMO_PIN > -1
#define THERMO_ANALOG_INPUTS 1
#define THERMO_ANALOG_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS
#define THERMO_ANALOG_CHANNEL BED_KOMMA FAN_THERMO_THERMISTOR_PIN
#define THERMO_COMMA ,
#else
#define THERMO_ANALOG_INPUTS 0
#define THERMO_ANALOG_CHANNEL
#define THERMO_COMMA BED_KOMMA
#endif

#if defined(ADC_KEYPAD_PIN) && (ADC_KEYPAD_PIN > -1)
#define KEYPAD_ANALOG_INPUTS 1
#define KEYPAD_ANALOG_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS+THERMO_ANALOG_INPUTS
#define KEYPAD_ANALOG_CHANNEL THERMO_COMMA ADC_KEYPAD_PIN
#else
#define KEYPAD_ANALOG_INPUTS 0
#define KEYPAD_ANALOG_CHANNEL
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
#endif

#define NUM_ANALOG_TEMP_SENSORS EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS+THERMO_ANALOG_INPUTS
/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS (EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS+THERMO_ANALOG_INPUTS+KEYPAD_ANALOG_INPUTS)
#if ANALOG_INPUTS > 0
/** Channels are the MUX-part of ADMUX register */
#define  ANALOG_INPUT_CHANNELS {EXT0_ANALOG_CHANNEL EXT1_ANALOG_CHANNEL EXT2_ANALOG_CHANNEL EXT3_ANALOG_CHANNEL EXT4_ANALOG_CHANNEL EXT5_ANALOG_CHANNEL BED_ANALOG_CHANNEL THERMO_ANALOG_CHANNEL KEYPAD_ANALOG_CHANNEL}
#endif

#define MENU_MODE_SD_MOUNTED 1
#define MENU_MODE_SD_PRINTING 2
#define MENU_MODE_PAUSED 4
#define MENU_MODE_FAN_RUNNING 8
#define MENU_MODE_PRINTING 16
#define MENU_MODE_FULL_PID 32
#define MENU_MODE_DEADTIME 64
#define MENU_MODE_FDM 128
#define MENU_MODE_LASER 256
#define MENU_MODE_CNC 512

#ifndef BENDING_CORRECTION_A
#define BENDING_CORRECTION_A 0
#endif

#ifndef BENDING_CORRECTION_B
#define BENDING_CORRECTION_B 0
#endif

#ifndef BENDING_CORRECTION_C
#define BENDING_CORRECTION_C 0
#endif

#ifndef ACCELERATION_FACTOR_TOP
#define ACCELERATION_FACTOR_TOP 100
#endif

#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL 2000
#endif

#include "HAL.h"
#define MAX_VFAT_ENTRIES (2)
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH 2

#include "ui.h"
#include "Communication.h"


#if UI_DISPLAY_TYPE != DISPLAY_U8G
#if (defined(USER_KEY1_PIN) && (USER_KEY1_PIN==UI_DISPLAY_D5_PIN || USER_KEY1_PIN==UI_DISPLAY_D6_PIN || USER_KEY1_PIN==UI_DISPLAY_D7_PIN)) || (defined(USER_KEY2_PIN) && (USER_KEY2_PIN==UI_DISPLAY_D5_PIN || USER_KEY2_PIN==UI_DISPLAY_D6_PIN || USER_KEY2_PIN==UI_DISPLAY_D7_PIN)) || (defined(USER_KEY3_PIN) && (USER_KEY3_PIN==UI_DISPLAY_D5_PIN || USER_KEY3_PIN==UI_DISPLAY_D6_PIN || USER_KEY3_PIN==UI_DISPLAY_D7_PIN)) || (defined(USER_KEY4_PIN) && (USER_KEY4_PIN==UI_DISPLAY_D5_PIN || USER_KEY4_PIN==UI_DISPLAY_D6_PIN || USER_KEY4_PIN==UI_DISPLAY_D7_PIN))
#error You cannot use DISPLAY_D5_PIN, DISPLAY_D6_PIN or DISPLAY_D7_PIN for "User Keys" with character LCD display
#endif
#endif


#ifndef SDCARDDETECT
#define SDCARDDETECT       -1
#endif

#ifndef SDSUPPORT
#define SDSUPPORT 0
#endif

#if SDSUPPORT
#include "SdFat.h"
#endif

#include "gcode.h"

#if ENABLE_BACKLASH_COMPENSATION && DRIVE_SYSTEM != CARTESIAN && !defined(ENFORCE_BACKLASH)
#undef ENABLE_BACKLASH_COMPENSATION
#define ENABLE_BACKLASH_COMPENSATION false
#endif

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t


#undef min
#undef max

class RMath
{
public:
    static inline float min(float a,float b)
    {
        if(a < b) return a;
        return b;
    }
    static inline float max(float a,float b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int32_t min(int32_t a,int32_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int32_t min(int32_t a,int32_t b, int32_t c)
    {
        if(a < b) return a < c ? a : c;
        return b<c ? b : c;
    }
    static inline float min(float a,float b, float c)
    {
        if(a < b) return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a,int32_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int min(int a,int b)
    {
        if(a < b) return a;
        return b;
    }
    static inline uint16_t min(uint16_t a,uint16_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int16_t max(int16_t a,int16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline uint16_t max(uint16_t a,uint16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline unsigned long absLong(long a)
    {
        return a >= 0 ? a : -a;
    }
    static inline int32_t sqr(int32_t a)
    {
        return a*a;
    }
    static inline uint32_t sqr(uint32_t a)
    {
        return a*a;
    }
#ifdef SUPPORT_64_BIT_MATH
    static inline int64_t sqr(int64_t a)
    {
        return a*a;
    }
    static inline uint64_t sqr(uint64_t a)
    {
        return a*a;
    }
#endif

    static inline float sqr(float a)
    {
        return a*a;
    }
};

class RVector3
{
public:
    float x, y, z;
    RVector3(float _x = 0,float _y = 0,float _z = 0):x(_x),y(_y),z(_z) {};
    RVector3(const RVector3 &a):x(a.x),y(a.y),z(a.z) {};


/*    const float &operator[](std::size_t idx) const
    {
        if(idx == 0) return x;
        if(idx == 1) return y;
        return z;
    };

    float &operator[](std::size_t idx)
    {
        switch(idx) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        }
        return 0;
    };*/

    inline bool operator==(const RVector3 &rhs)
    {
        return x==rhs.x && y==rhs.y && z==rhs.z;
    }
    inline bool operator!=(const RVector3 &rhs)
    {
        return !(*this==rhs);
    }
    inline RVector3& operator=(const RVector3 &rhs)
    {
        if(this!=&rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    inline RVector3& operator+=(const RVector3 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline RVector3& operator-=(const RVector3 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    inline RVector3 operator-() const
    {
        return RVector3(-x,-y,-z);
    }

    inline float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    inline float lengthSquared() const
    {
        return (x*x+y*y+z*z);
    }

    inline RVector3 cross(const RVector3 &b) const
    {
        return RVector3(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x);
    }
    inline float scalar(const RVector3 &b) const
    {
        return (x*b.x+y*b.y+z*b.z);
    }
    inline RVector3 scale(float factor) const
    {
        return RVector3(x*factor,y*factor,z*factor);
    }
    inline void scaleIntern(float factor)
    {
        x*=factor;
        y*=factor;
        z*=factor;
    }
    inline void setMinimum(const RVector3 &b)
    {
        x = RMath::min(x,b.x);
        y = RMath::min(y,b.y);
        z = RMath::min(z,b.z);
    }
    inline void setMaximum(const RVector3 &b)
    {
        x = RMath::max(x,b.x);
        y = RMath::max(y,b.y);
        z = RMath::max(z,b.z);
    }
    inline float distance(const RVector3 &b) const
    {
        float dx = b.x-x,dy = b.y-y, dz = b.z-z;
        return (sqrt(dx*dx+dy*dy+dz*dz));
    }
    inline float angle(RVector3 &direction)
    {
        return static_cast<float>(acos(scalar(direction)/(length()*direction.length())));
    }

    inline RVector3 normalize() const
    {
        float len = length();
        if(len != 0) len = static_cast<float>(1.0/len);
        return RVector3(x*len,y*len,z*len);
    }

    inline RVector3 interpolatePosition(const RVector3 &b, float pos) const
    {
        float pos2 = 1.0f - pos;
        return RVector3(x * pos2 + b.x * pos, y * pos2 + b.y * pos, z * pos2 + b.z * pos);
    }

    inline RVector3 interpolateDirection(const RVector3 &b,float pos) const
    {
        //float pos2 = 1.0f - pos;

        float dot = scalar(b);
        if (dot > 0.9995 || dot < -0.9995)
            return interpolatePosition(b,pos); // cases cause trouble, use linear interpolation here

        float theta = acos(dot) * pos; // interpolated position
        float st = sin(theta);
        RVector3 t(b);
        t -= scale(dot);
        float lengthSq = t.lengthSquared();
        float dl = st * ((lengthSq < 0.0001f) ? 1.0f : 1.0f / sqrt(lengthSq));
        t.scaleIntern(dl);
        t += scale(cos(theta));
        return t.normalize();
    }
};
	inline RVector3 operator+(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x += rhs.x;
		lhs.y += rhs.y;
		lhs.z += rhs.z;
		return lhs;
	}

	inline RVector3 operator-(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x -= rhs.x;
		lhs.y -= rhs.y;
		lhs.z -= rhs.z;
		return lhs;
	}

    inline RVector3 operator*(const RVector3 &lhs,float rhs) {
        return lhs.scale(rhs);
    }

    inline RVector3 operator*(float lhs,const RVector3 &rhs) {
        return rhs.scale(lhs);
    }

#if !defined(MAX_FAN_PWM) || MAX_FAN_PWM == 255
#define TRIM_FAN_PWM(x) x
#undef MAX_FAN_PWM
#define MAX_FAN_PWM 255
#else
#define TRIM_FAN_PWM(x) static_cast<uint8_t>(static_cast<unsigned int>(x) * MAX_FAN_PWM / 255)
#endif

extern const uint8 osAnalogInputChannels[] PROGMEM;
//extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
//extern uint osAnalogInputBuildup[ANALOG_INPUTS];
//extern uint8 osAnalogInputPos; // Current sampling position
#if ANALOG_INPUTS > 0
extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif
#define PWM_HEATED_BED    NUM_EXTRUDER
#define PWM_BOARD_FAN     PWM_HEATED_BED + 1
#define PWM_FAN1          PWM_BOARD_FAN + 1
#define PWM_FAN2          PWM_FAN1 + 1
#define PWM_FAN_THERMO    PWM_FAN2 + 1
#define NUM_PWM           PWM_FAN_THERMO + 1
extern uint8_t pwm_pos[NUM_PWM]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
extern int maxadv;
#endif
extern int maxadv2;
extern float maxadvspeed;
#endif


#include "Extruder.h"

void manage_inactivity(uint8_t debug);

extern void finishNextSegment();
#if NONLINEAR_SYSTEM
extern uint8_t transformCartesianStepsToDeltaSteps(long cartesianPosSteps[], long deltaPosSteps[]);
#if SOFTWARE_LEVELING
extern void calculatePlane(long factors[], long p1[], long p2[], long p3[]);
extern float calcZOffset(long factors[], long pointX, long pointY);
#endif
#endif
extern void linear_move(long steps_remaining[]);
#ifndef FEATURE_DITTO_PRINTING
#define FEATURE_DITTO_PRINTING false
#endif
#if FEATURE_DITTO_PRINTING && (NUM_EXTRUDER > 4 || NUM_EXTRUDER < 2)
#error Ditto printing requires 2 - 4 extruder.
#endif


extern millis_t previousMillisCmd;
extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;

extern void setupTimerInterrupt();
extern void motorCurrentControlInit();
extern void microstepInit();

#include "Printer.h"
#include "motion.h"
extern long baudrate;

#include "HAL.h"


extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter500ms;
extern void writeMonitor();
#if FEATURE_FAN_CONTROL
extern uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
extern uint8_t fan2Kickstart;
#endif

#if SDSUPPORT
extern char tempLongFilename[LONG_FILENAME_LENGTH+1];
extern char fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];
#define SHORT_FILENAME_LENGTH 14
#include "SdFat.h"

enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
class SDCard
{
public:
    SdFat fat;
    //Sd2Card card; // ~14 Byte
    //SdVolume volume;
    //SdFile root;
    //SdFile dir[SD_MAX_FOLDER_DEPTH+1];
    SdFile file;
#if JSON_OUTPUT
    GCodeFileInfo fileInfo;
#endif
    uint32_t filesize;
    uint32_t sdpos;
    //char fullName[13*SD_MAX_FOLDER_DEPTH+13]; // Fill name
    char *shortname; // Pointer to start of filename itself
    char *pathend; // File to char where pathname in fullname ends
    uint8_t sdmode;  // 1 if we are printing from sd card, 2 = stop accepting new commands
    bool sdactive;
    //int16_t n;
    bool savetosd;
    SdBaseFile parentFound;

    SDCard();
    void initsd();
    void writeCommand(GCode *code);
    bool selectFile(const char *filename,bool silent=false);
    void mount();
    void unmount();
    void startPrint();
    void pausePrint(bool intern = false);
    void continuePrint(bool intern = false);
    void stopPrint();
    inline void setIndex(uint32_t  newpos)
    {
        if(!sdactive) return;
        sdpos = newpos;
        file.seekSet(sdpos);
    }
    void printStatus();
    void ls();
#if JSON_OUTPUT
    void lsJSON(const char *filename);
    void JSONFileInfo(const char *filename);
    static void printEscapeChars(const char *s);
#endif
    void startWrite(char *filename);
    void deleteFile(char *filename);
    void finishWrite();
    char *createFilename(char *buffer,const dir_t &p);
    void makeDirectory(char *filename);
    bool showFilename(const uint8_t *name);
    void automount();
#ifdef GLENN_DEBUG
    void writeToFile();
#endif
private:
    uint8_t lsRecursive(SdBaseFile *parent,uint8_t level,char *findFilename);
// SdFile *getDirectory(char* name);
};

extern SDCard sd;
#endif

extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout
extern void updateStepsParameter(PrintLine *p/*,uint8_t caller*/);

#ifdef DEBUG_PRINT
extern int debugWaitLoop;
#endif

#if NONLINEAR_SYSTEM
#define NUM_AXIS 4
#endif

#define STR(s) #s
#define XSTR(s) STR(s)
#include "Commands.h"
#include "Eeprom.h"

#if CPU_ARCH == ARCH_AVR
#define DELAY1MICROSECOND        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")
#define DELAY2MICROSECOND        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\tnop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t")
#else
#define DELAY1MICROSECOND     HAL::delayMicroseconds(1);
#define DELAY2MICROSECOND     HAL::delayMicroseconds(2);
#endif

#ifdef FAST_INTEGER_SQRT
#define SQRT(x) ( HAL::integerSqrt(x) )
#else
#define SQRT(x) sqrt(x)
#endif

class PlaneBuilder {
	float sum_xx,sum_xy,sum_yy,sum_x,sum_y,sum_xz,sum_yz,sum_z,n;
	public:
	PlaneBuilder() {
		reset();
	}
	void reset() {
		sum_xx = sum_xy = sum_yy = sum_x = sum_y = sum_xz = sum_yz = sum_z = n = 0;
	}
	void addPoint(float x,float y,float z) {
		n++;
		sum_xx += x * x;
		sum_xy += x * y;
		sum_yy += y * y;
		sum_x  += x;
		sum_y  += y;
		sum_xz += x * z;
		sum_yz += y * z;
		sum_z  += z;
	}
	void createPlane(Plane &plane,bool silent=false) {
		float det = (sum_x * (sum_xy * sum_y - sum_x * sum_yy) + sum_xx * (n * sum_yy - sum_y * sum_y) + sum_xy * (sum_x * sum_y - n * sum_xy));
		plane.a = ((sum_xy * sum_y  - sum_x * sum_yy)  * sum_z + (sum_x * sum_y  - n      * sum_xy) * sum_yz + sum_xz * (n      * sum_yy - sum_y * sum_y))  / det;
		plane.b = ((sum_x  * sum_xy - sum_xx * sum_y)  * sum_z + (n     * sum_xx - sum_x  * sum_x)  * sum_yz + sum_xz * (sum_x  * sum_y  - n     * sum_xy)) / det;
		plane.c = ((sum_xx * sum_yy - sum_xy * sum_xy) * sum_z + (sum_x * sum_xy - sum_xx * sum_y)  * sum_yz + sum_xz * (sum_xy * sum_y  - sum_x * sum_yy)) / det;
		if(!silent) {
			Com::printF(PSTR("plane: a = "),plane.a,4);
			Com::printF(PSTR(" b = "),plane.b,4);
			Com::printFLN(PSTR(" c = "),plane.c,4);
		}
	}
};

#include "Drivers.h"

#include "Events.h"
#if defined(CUSTOM_EVENTS)
#include "CustomEvents.h"
#endif

// must be after CustomEvents as it might include definitions from there
#include "DisplayList.h"

#endif
