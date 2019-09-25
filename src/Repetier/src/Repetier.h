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

#ifndef REPETIER_VERSION
#define REPETIER_VERSION "2.0.0dev"
#endif

#ifndef EMERGENCY_PARSER
#define EMERGENCY_PARSER 1
#endif

#define MOTHERBOARD_RADDS 402
#define MOTHERBOARD_RAMPS_FD_INVERTED_HEATER 403
#define MOTHERBOARD_RAMPS_FD 404
#define MOTHERBOARD_FELIX 405
#define MOTHERBOARD_BAM_AND_DICE 406
#define MOTHERBOARD_ULTRATRONICS 409
#define MOTHERBOARD_SMARTRAMPS_NO_EEPROM 408
#define MOTHERBOARD_SMARTRAMPS_EEPROM 413
#define MOTHERBOARD_STACKER_3D_SUPERBOARD 412
#define MOTHERBOARD_RURAMPS4D 414
#define MOTHERBOARD_SHASTA 416
#define MOTHERBOARD_IKS3D 417
#define MOTHERBOARD_ALLIGATOR_REV1 500
#define MOTHERBOARD_ALLIGATOR_REV2 501
#define MOTHERBOARD_ALLIGATOR_REV3 502
#define MOTHERBOARD_USER_DEFINED_DUE 998
#define MOTHERBOARD_USER_DEFINED_AVR 999

// SAMD51 based boards
#define MOTHERBOARD_AGC_SMARTRAMPS_NO_EEPROM 1000
#define MOTHERBOARD_AGC_SMARTRAMPS_EEPROM 1001
#define MOTHERBOARD_AGC_USER_DEFINED 1002
#define MOTHERBOARD_AGC_RADDS 1003

// M0/SAMD21 based boards
#define MOTHERBOARD_MINITRONICS_2_0 2000

// Some helper macros

#define _CAT(a, ...) a##__VA_ARGS__
#define SWITCH_ENABLED_false 0
#define SWITCH_ENABLED_true 1
#define SWITCH_ENABLED_0 0
#define SWITCH_ENABLED_1 1
#define SWITCH_ENABLED_ 1
#define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
#define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))

// Use new communication model for multiple channels - only until stable, then old version gets deleted

// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################
// These are run time switchable debug flags
enum debugFlags { DEB_ECHO = 0x1,
                  DEB_INFO = 0x2,
                  DEB_ERROR = 0x4,
                  DEB_DRYRUN = 0x8,
                  DEB_COMMUNICATION = 0x10,
                  DEB_NOMOVES = 0x20,
                  DEB_DEBUG = 0x40
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

#define DEBUG_MSG(x) \
    { \
        if (Printer::debugEcho()) { \
            Com::printFLN(PSTR(x)); \
            HAL::delayMilliseconds(20); \
        } \
    }
#define DEBUG_MSG2(x, y) \
    { \
        if (Printer::debugEcho()) { \
            Com::printFLN(PSTR(x), y); \
            HAL::delayMilliseconds(20); \
        } \
    }
#define DEBUG_MSG_FAST(x) \
    { \
        if (Printer::debugEcho()) { \
            Com::printFLN(PSTR(x)); \
        } \
    }
#define DEBUG_MSG2_FAST(x, y) \
    { \
        if (Printer::debugEcho()) { \
            Com::printFLN(PSTR(x), y); \
        } \
    }

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "=" VALUE(var)

#define WIZARD_STACK_SIZE 8
#define IGNORE_COORDINATE 999999

#define IS_MAC_TRUE(x) (x != 0)
#define IS_MAC_FALSE(x) (x == 0)
#define HAS_PIN(x) (defined(x##_PIN) && x > -1)

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
#define ANALYZER_ON(a) \
    { \
        WRITE(a, HIGH); \
    }
#define ANALYZER_OFF(a) \
    { \
        WRITE(a, LOW); \
    }
#else
#define ANALYZER_ON(a)
#define ANALYZER_OFF(a)
#endif

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define A_AXIS 4
#define B_AXIS 5
#define C_AXIS 6
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

#include "utilities/RMath.h"
#include "utilities/RVector3.h"
extern void updateEndstops();

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
#define CONTROLLER_AZSMZ_12864_OLED 28

#define PRINTER_TYPE_CARESIAN 0
#define PRINTER_TYPE_CORE_XYZ 1
#define PRINTER_TYPE_DELTA 2
#define PRINTER_TYPE_DUAL_X 3

#define ILLEGAL_Z_PROBE -888

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#ifndef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#define CONFIG_VARIABLE(tp, name, values) extern tp name;
#define CONFIG_VARIABLE_EQ(tp, name, values) extern tp name;
#endif

class ServoInterface {
public:
    virtual int getPosition();
    virtual void setPosition(int pos, int32_t timeout);
    virtual void enable();
    virtual void disable();
};

#include "io/temperature_tables.h"
#include "Configuration.h"

#ifndef WAITING_IDENTIFIER
#define WAITING_IDENTIFIER "wait"
#endif

#ifndef HOST_RESCUE
#define HOST_RESCUE 1
#endif
#if EEPROM_MODE == 0 && HOST_RESCUE
#warning HOST_RESCUE requires eeprom support! Disabling feature.
#undef HOST_RESCUE
#define HOST_RESCUE 0
#endif

#ifndef E_SPEED
#define E_SPEED 2
#endif

extern ServoInterface* servos[NUM_SERVOS];

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

#ifndef BOARD_FAN_SPEED
#define BOARD_FAN_SPEED
#endif

#ifndef MAX_JERK_DISTANCE
#define MAX_JERK_DISTANCE 0.6
#endif

#ifndef JSON_OUTPUT
#define JSON_OUTPUT 0
#endif

#if !defined(ZPROBE_MIN_TEMPERATURE) && defined(ZHOME_MIN_TEMPERATURE)
#define ZPROBE_MIN_TEMPERATURE ZHOME_MIN_TEMPERATURE
#endif

#ifndef MAX_ROOM_TEMPERATURE
#define MAX_ROOM_TEMPERATURE 25
#endif
#ifndef ZHOME_X_POS
#define ZHOME_X_POS IGNORE_COORDINATE
#endif
#ifndef ZHOME_Y_POS
#define ZHOME_Y_POS IGNORE_COORDINATE
#endif

// MS1 MS2 Stepper Driver Micro stepping mode table
#define MICROSTEP1 LOW, LOW
#define MICROSTEP2 HIGH, LOW
#define MICROSTEP4 LOW, HIGH
#define MICROSTEP8 HIGH, HIGH
#if (MOTHERBOARD == 501) || MOTHERBOARD == 502
#define MICROSTEP16 LOW, LOW
#else
#define MICROSTEP16 HIGH, HIGH
#endif
#define MICROSTEP32 HIGH, HIGH

#define GCODE_BUFFER_SIZE 1

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

//Step to split a circle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS calculation is started to correct the circle interpolation
#define N_ARC_CORRECTION 25

#if (defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1) || (defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1) || (defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1) || (defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1) || (defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1) || (defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1)
#define EXTRUDER_JAM_CONTROL 1
#else
#define EXTRUDER_JAM_CONTROL 0
#endif
#ifndef JAM_METHOD
#define JAM_METHOD 1
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
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

#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL 2000
#endif

#ifndef MAX_VFAT_ENTRIES
#ifdef AVR_BOARD
#define MAX_VFAT_ENTRIES (2)
#else
#define MAX_VFAT_ENTRIES (3)
#endif
#endif

/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13 * MAX_VFAT_ENTRIES + 1)
#define SD_MAX_FOLDER_DEPTH 2

#if UI_DISPLAY_TYPE != DISPLAY_U8G
#if (defined(USER_KEY1_PIN) && (USER_KEY1_PIN == UI_DISPLAY_D5_PIN || USER_KEY1_PIN == UI_DISPLAY_D6_PIN || USER_KEY1_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY2_PIN) && (USER_KEY2_PIN == UI_DISPLAY_D5_PIN || USER_KEY2_PIN == UI_DISPLAY_D6_PIN || USER_KEY2_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY3_PIN) && (USER_KEY3_PIN == UI_DISPLAY_D5_PIN || USER_KEY3_PIN == UI_DISPLAY_D6_PIN || USER_KEY3_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY4_PIN) && (USER_KEY4_PIN == UI_DISPLAY_D5_PIN || USER_KEY4_PIN == UI_DISPLAY_D6_PIN || USER_KEY4_PIN == UI_DISPLAY_D7_PIN))
#error You cannot use DISPLAY_D5_PIN, DISPLAY_D6_PIN or DISPLAY_D7_PIN for "User Keys" with character LCD display
#endif
#endif

#ifndef SDCARDDETECT
#define SDCARDDETECT -1
#endif

#ifndef SDSUPPORT
#define SDSUPPORT 0
#endif

#if SDSUPPORT
#include "SdFat/SdFat.h"
#endif

#include "communication/gcode.h"

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t

#undef min
#undef max

#if !defined(MAX_FAN_PWM) || MAX_FAN_PWM == 255
#define TRIM_FAN_PWM(x) x
#undef MAX_FAN_PWM
#define MAX_FAN_PWM 255
#else
#define TRIM_FAN_PWM(x) static_cast<uint8_t>(static_cast<unsigned int>(x) * MAX_FAN_PWM / 255)
#endif

extern PWMHandler* fans[];
// extern const uint8 osAnalogInputChannels[] PROGMEM;
//extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
//extern uint osAnalogInputBuildup[ANALOG_INPUTS];
//extern uint8 osAnalogInputPos; // Current sampling position
//#if ANALOG_INPUTS > 0
//extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
//#endif

#ifndef FEATURE_DITTO_PRINTING
#define FEATURE_DITTO_PRINTING false
#endif

extern millis_t previousMillisCmd;
extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;

extern void setupTimerInterrupt();
extern void motorCurrentControlInit();
extern void microstepInit();

#include "PrinterTypes/Printer.h"
#include "motion/LevelingMethod.h"
#include "motion/MotionLevel1.h"
#include "motion/MotionLevel2.h"
#include "motion/MotionLevel3.h"
#if PRINTER_TYPE == 0
#include "PrinterTypes/PrinterTypeCartesian.h"
#elif PRINTER_TYPE == 1
#include "PrinterTypes/PrinterTypeCoreXYZ.h"
#elif PRINTER_TYPE == 2
#include "PrinterTypes/PrinterTypeDelta.h"
#elif PRINTER_TYPE == 3
#include "PrinterTypes/PrinterTypeDualXAxis.h"
#endif
#include "motion/VelocityProfile.h"

// #include "src/motion/motion.h"

extern long baudrate;

extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter500ms;
extern void writeMonitor();

extern char tempLongFilename[LONG_FILENAME_LENGTH + 1];
extern char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];
#if SDSUPPORT
#define SHORT_FILENAME_LENGTH 14

enum LsAction { LS_SerialPrint,
                LS_Count,
                LS_GetFilename };
class SDCard {
public:
#if ENABLE_SOFTWARE_SPI_CLASS
    SdFatSoftSpi<SD_SOFT_MISO_PIN, SD_SOFT_MOSI_PIN, SD_SOFT_SCK_PIN> fat;
#else
    SdFat fat;
#endif
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
    // char* shortname; // Pointer to start of filename itself
    // char* pathend;   // File to char where pathname in fullname ends
    uint8_t sdmode; // 1 if we are printing from sd card, 2 = stop accepting new commands
    bool sdactive;
    //int16_t n;
    bool savetosd;
    SdBaseFile parentFound;

    SDCard();
    void initsd();
    void writeCommand(GCode* code);
    bool selectFile(const char* filename, bool silent = false);
    void mount();
    void unmount();
    void startPrint();
    void pausePrint(bool intern = false);
    void pausePrintPart2();
    void continuePrint(bool intern = false);
    void stopPrint();
    void stopPrintPart2();
    inline void setIndex(uint32_t newpos) {
        if (!sdactive)
            return;
        sdpos = newpos;
        file.seekSet(sdpos);
    }
    void printStatus();
    void ls();
#if JSON_OUTPUT
    void lsJSON(const char* filename);
    void JSONFileInfo(const char* filename);
    static void printEscapeChars(const char* s);
#endif
    void startWrite(char* filename);
    void deleteFile(char* filename);
    void finishWrite();
    char* createFilename(char* buffer, const dir_t& p);
    void makeDirectory(char* filename);
    bool showFilename(const uint8_t* name);
    void automount();
#ifdef GLENN_DEBUG
    void writeToFile();
#endif
private:
    uint8_t lsRecursive(SdBaseFile* parent, uint8_t level, char* findFilename);
    // SdFile *getDirectory(char* name);
};

extern SDCard sd;
#endif

extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout
// extern void updateStepsParameter(PrintLine *p/*,uint8_t caller*/);

#ifdef DEBUG_PRINT
extern int debugWaitLoop;
#endif

#define STR(s) #s
#define XSTR(s) STR(s)
#include "communication/Commands.h"
#include "communication/Eeprom.h"

#if CPU_ARCH == ARCH_AVR
#define DELAY1MICROSECOND __asm__("nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t")
#define DELAY2MICROSECOND __asm__("nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\tnop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t" \
                                  "nop\n\t")
#else
#define DELAY1MICROSECOND HAL::delayMicroseconds(1);
#define DELAY2MICROSECOND HAL::delayMicroseconds(2);
#endif

#ifdef FAST_INTEGER_SQRT
#define SQRT(x) (HAL::integerSqrt(x))
#else
#define SQRT(x) sqrt(x)
#endif

#include "motion/Drivers.h"
#include "utilities/PlaneBuilder.h"

#include "Events.h"
#include "custom/customEvents.h"

// must be after CustomEvents as it might include definitions from there
// #include "controller/DisplayList.h"

#include "controller/gui.h"
#endif
