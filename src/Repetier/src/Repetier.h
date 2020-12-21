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
#include <type_traits>

#ifndef REPETIER_VERSION
#define REPETIER_VERSION "2.0.0dev"
#endif

#ifndef EMERGENCY_PARSER
#define EMERGENCY_PARSER 1
#endif

#ifndef HOST_PRIORITY_CONTROLS
#define HOST_PRIORITY_CONTROLS EMERGENCY_PARSER
#endif

#include "utilities/constants.h"

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
enum class debugFlags { DEB_ECHO = 0x1,
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

#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF ANALOG_REF_AVCC

#include "utilities/RMath.h"
extern void updateEndstops();

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#ifndef CONFIG_EXTERN
#define CONFIG_EXTERN extern
#define CONFIG_VARIABLE(tp, name, values) extern tp name;
#define CONFIG_VARIABLE_EQ(tp, name, values) extern tp name;
#endif

/*
  arm does not have a eeprom build in. Therefore boards can add a
  eeprom. Board definition must set the right type of eeprom
*/

#define EEPROM_NONE 0          /** Don't use eeprom */
#define EEPROM_I2C 1           /** Use external eeprom connected via I2C bus */
#define EEPROM_SPI_ALLIGATOR 2 /** Use spi connected eeprom on alligator boards */
#define EEPROM_SDCARD 3        /** Use mounted sd card as eeprom replacement */
#define EEPROM_FLASH 4         /** Use flash memory as eeprom replacement */

class GCode;
class ServoInterface {
public:
    virtual int getPosition() = 0;
    virtual void setPosition(int pos, int32_t timeout) = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void executeGCode(GCode* com) = 0;
};

enum class BootReason {
    SOFTWARE_RESET = 0,
    BROWNOUT = 1,
    LOW_POWER = 2,
    WATCHDOG_RESET = 3,
    EXTERNAL_PIN = 4,
    POWER_UP = 5,
    UNKNOWN = -1
};

#include "io/temperature_tables.h"
#include "Configuration.h"

#if NUM_AXES < 4
#error The minimum NUM_AXES allowed is 4!
#endif

#ifndef ALWAYS_CHECK_ENDSTOPS
#define ALWAYS_CHECK_ENDSTOPS 0
#endif

#ifndef WAITING_IDENTIFIER
#define WAITING_IDENTIFIER "wait"
#endif

#ifndef ACK_WITH_LINENUMBER
#define ACK_WITH_LINENUMBER 1
#endif

#ifndef ECHO_ON_EXECUTE
#define ECHO_ON_EXECUTE 1
#endif

#ifndef KILL_METHOD
#define KILL_METHOD 1 // reset on emergency stop
#endif

#ifndef FEATURE_WATCHDOG
#define FEATURE_WATCHDOG 1
#endif

#ifndef JSON_OUTPUT
#define JSON_OUTPUT 1
#endif

#ifndef SMALL_SEGMENT_SIZE
#define SMALL_SEGMENT_SIZE 0.4
#endif

#ifndef Z_PROBE_START_SCRIPT
#define Z_PROBE_START_SCRIPT ""
#endif

#ifndef Z_PROBE_FINISHED_SCRIPT
#define Z_PROBE_FINISHED_SCRIPT ""
#endif

#ifndef ARC_SUPPORT
#define ARC_SUPPORT 1
#endif

#ifndef HOST_RESCUE
#define HOST_RESCUE 1
#endif

#ifndef MIN_PRINTLINE_FILL
#define MIN_PRINTLINE_FILL 8
#endif
#if MIN_PRINTLINE_FILL > PRINTLINE_CACHE_SIZE
#undef MIN_PRINTLINE_FILL
#define MIN_PRINTLINE_FILL PRINTLINE_CACHE_SIZE
#endif

#ifndef MAX_BUFFERED_LENGTH_MM
#define MAX_BUFFERED_LENGTH_MM 200
#endif

#if EEPROM_MODE == 0 && HOST_RESCUE
#warning HOST_RESCUE requires eeprom support! Disabling feature.
#undef HOST_RESCUE
#define HOST_RESCUE 0
#endif

#ifndef E_SPEED
#define E_SPEED 2
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_A) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_A MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_B) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_B)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_B MAX_ACCELERATION_UNITS_PER_SQ_SECOND_B
#endif

#if !defined(MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_C) && defined(MAX_ACCELERATION_UNITS_PER_SQ_SECOND_C)
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_C MAX_ACCELERATION_UNITS_PER_SQ_SECOND_C
#endif

#ifndef SLOW_DIRECTION_CHANGE
#define SLOW_DIRECTION_CHANGE 1
#endif

#ifndef TMC_CHOPPER_TIMING
#define TMC_CHOPPER_TIMING CHOPPER_TIMING_DEFAULT_12V
#endif
#ifndef TMC_INTERPOLATE
#define TMC_INTERPOLATE true
#endif
#ifndef TMC_HOLD_MULTIPLIER
#define TMC_HOLD_MULTIPLIER 0.5
#endif
#ifndef TMC_CURRENT_STEP_DOWN
#define TMC_CURRENT_STEP_DOWN 50
#endif
#ifndef STORE_MOTOR_MICROSTEPPING
#define STORE_MOTOR_MICROSTEPPING 1
#endif
#ifndef STORE_MOTOR_CURRENT
#define STORE_MOTOR_CURRENT 1
#endif
#ifndef STORE_MOTOR_HYBRID_TRESHOLD
#define STORE_MOTOR_HYBRID_TRESHOLD 1
#endif
#ifndef STORE_MOTOR_STEALTH
#define STORE_MOTOR_STEALTH 1
#endif
#ifndef STORE_MOTOR_STALL_SENSITIVITY
#define STORE_MOTOR_STALL_SENSITIVITY 1
#endif

#ifndef DEFAULT_TONE_VOLUME
#define DEFAULT_TONE_VOLUME 100
#endif

#ifndef MINIMUM_TONE_VOLUME
#define MINIMUM_TONE_VOLUME 5
#endif

#ifndef NUM_BEEPERS
#define NUM_BEEPERS 0
#define BEEPER_LIST \
    { }
#endif

extern ServoInterface* servos[];

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

#ifndef PARK_POSITION_Z_UP_FIRST
#define PARK_POSITION_Z_UP_FIRST 0
#endif

#ifndef MAX_JERK_DISTANCE
#define MAX_JERK_DISTANCE 0.6
#endif

#ifndef JSON_OUTPUT
#define JSON_OUTPUT 0
#endif

#ifndef Z_PROBE_PAUSE_HEATERS
#define Z_PROBE_PAUSE_HEATERS 0
#endif

#ifndef Z_PROBE_PAUSE_BED_REHEAT_TEMP
#define Z_PROBE_PAUSE_BED_REHEAT_TEMP 5
#endif

#ifndef Z_PROBE_BLTOUCH_DEPLOY_DELAY
#define Z_PROBE_BLTOUCH_DEPLOY_DELAY 1000
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

// Firmware retraction settings 
#ifndef AUTORETRACT_ENABLED
#define AUTORETRACT_ENABLED 0
#endif
#ifndef RETRACTION_LENGTH 
#define RETRACTION_LENGTH 0.0
#endif
#ifndef RETRACTION_SPEED 
#define RETRACTION_SPEED 0
#endif
#ifndef RETRACTION_UNDO_SPEED 
#define RETRACTION_UNDO_SPEED 0
#endif
#ifndef RETRACTION_Z_LIFT 
#define RETRACTION_Z_LIFT 0.0
#endif
#ifndef RETRACTION_UNDO_EXTRA_LENGTH 
#define RETRACTION_UNDO_EXTRA_LENGTH 0.0
#endif
#ifndef RETRACTION_UNDO_EXTRA_LONG_LENGTH 
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0.0
#endif
#ifndef RETRACTION_LONG_LENGTH 
#define RETRACTION_LONG_LENGTH 0.0
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
#endif

#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL 2000
#endif

#undef min
#undef max

#if !defined(MAX_FAN_PWM) || MAX_FAN_PWM == 255
#define TRIM_FAN_PWM(x) x
#undef MAX_FAN_PWM
#define MAX_FAN_PWM 255
#else
#define TRIM_FAN_PWM(x) static_cast<uint8_t>(static_cast<unsigned int>(x) * MAX_FAN_PWM / 255)
#endif

struct FanController {
    PWMHandler* fan;
    millis_t time;
    fast8_t target;
    uint32_t timeout;
};

extern FanController fans[];
// extern const uint8 osAnalogInputChannels[] PROGMEM;
//extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
//extern uint osAnalogInputBuildup[ANALOG_INPUTS];
//extern uint8 osAnalogInputPos; // Current sampling position
//#if ANALOG_INPUTS > 0
//extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
//#endif

extern BeeperSourceBase* beepers[];

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
#if PRINTER_TYPE == PRINTER_TYPE_CARTESIAN
#include "PrinterTypes/PrinterTypeCartesian.h"
#elif PRINTER_TYPE == PRINTER_TYPE_CORE_XYZ
#include "PrinterTypes/PrinterTypeCoreXYZ.h"
#elif PRINTER_TYPE == PRINTER_TYPE_DELTA
#include "PrinterTypes/PrinterTypeDelta.h"
#elif PRINTER_TYPE == PRINTER_TYPE_DUAL_X
#include "PrinterTypes/PrinterTypeDualXAxis.h"
#endif
#include "motion/VelocityProfile.h"

extern long baudrate;

extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter500ms;
extern void writeMonitor();

extern char tempLongFilename[LONG_FILENAME_LENGTH + 1];
extern char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];
#if SDSUPPORT
#define SHORT_FILENAME_LENGTH 14

typedef SdFat sd_fsys_t;
typedef SdBaseFile sd_file_t;

enum class SDState {
    SD_UNMOUNTED,    // No SD Card detected/mounted
    SD_SAFE_EJECTED, // Manually ejected by M22
    SD_HAS_ERROR,    // Rare, when has error but left SD card in.
    SD_MOUNTED,      // When not printing/idle
    SD_PRINTING,     // While printing. Includes while paused.
    SD_WRITING       // While writing to a file. 5 minute timeout from last write.
};
class SDCard {
public:

    SDCard();
    void writeCommand(GCode* code);
    void startPrint();
    void continuePrint(const bool internal = false);

    void pausePrint(const bool internal = false);
    void printFullyPaused();

    void stopPrint(const bool silent = false);
    void printFullyStopped();

    static bool validGCodeExtension(const char* filename);
    inline void setIndex(uint32_t newpos) {
        if (state < SDState::SD_MOUNTED || !selectedFile.isOpen()) {
            return;
        }
        selectedFilePos = newpos;
        selectedFile.seekSet(selectedFilePos);
    }
    void printStatus(const bool getFilename = false);
#if JSON_OUTPUT
    void lsJSON(const char* filename);
    void JSONFileInfo(const char* filename);
    static void printEscapeChars(const char* s);
#endif
    void ls(const char* lsDir = Com::tSlash, const bool json = false);
    void ls(sd_file_t& rootDir, const bool json = false); // Will auto close rootdir

    void mount(const bool manual);
    void unmount(const bool manual);
    void automount();

    bool selectFile(const char* filename, const bool silent = false);
    void startWrite(const char* filename);
    void deleteFile(const char* filename);
    void makeDirectory(const char* filename);
    void finishWrite();
    void finishPrint();

    void printCardStats();
    bool printIfCardErrCode(); // Just prints a small hex errorcode for the lookup table
    bool getCardInfo(char* volumeLabelBuf = nullptr, uint8_t volumeLabelSize = 0, uint64_t* volumeSizeSectors = nullptr, uint64_t* usageBytes = nullptr, uint16_t* fileCount = nullptr, uint8_t* folderCount = nullptr);

    template <typename T>
    bool doForDirectory(sd_file_t& dir, T&& action, const bool recursive = false, size_t depth = 0) {
        if (!dir.isDir()) {
            return false;
        }
        sd_file_t file;
        dir.rewind();
        while (file.openNext(&dir)) {
            if (!action(file, dir, depth)) {
                file.close();
                return false;
            }
            if (recursive && file.isDir() && (depth + 1u) < SD_MAX_FOLDER_DEPTH) {
                depth++;
                if (!doForDirectory(file, action, recursive, depth)) {
                    file.close();
                    return false;
                }
                depth--;
            }
            file.close();
            HAL::pingWatchdog();
        }
        return true;
    }
    // Shorter than writing getname(templongfilename, sizeof(templongfilename)) etc
    // Will always inline return pointer to either tempLongFilename or the input buffer.
    template <size_t N>
    inline char* getFN(sd_file_t* file, char (&buf)[N]) {
        file->getName(buf, N);
        return buf;
    }
    template <size_t N>
    inline char* getFN(sd_file_t& file, char (&buf)[N]) {
        file.getName(buf, N);
        return buf;
    }
    inline char* getFN(sd_file_t* file, char* buf, const uint8_t size) {
        file->getName(buf, size);
        return buf;
    }
    inline char* getFN(sd_file_t& file, char* buf, const uint8_t size) {
        file.getName(buf, size);
        return buf;
    }
    inline char* getFN(sd_file_t* file) {
        return getFN(file, tempLongFilename);
    }
    inline char* getFN(sd_file_t& file) {
        return getFN(file, tempLongFilename);
    }
#ifdef GLENN_DEBUG
    void writeToFile();
#endif
    uint32_t selectedFileSize;
    uint32_t selectedFilePos;

    sd_file_t selectedFile;
    sd_fsys_t fileSystem;
    SDState state;

    char volumeLabel[21];

    bool scheduledPause;
    bool scheduledStop;
    millis_t lastWriteTimeMS;
    uint32_t writtenBytes;
#if JSON_OUTPUT
    GCodeFileInfo fileInfo;
#endif
private:
#if defined(ENABLE_SOFTWARE_SPI_CLASS) && ENABLE_SOFTWARE_SPI_CLASS
    SoftSpiDriver<SD_SOFT_MISO_PIN, SD_SOFT_MOSI_PIN, SD_SOFT_SCK_PIN> softSpi;
#endif

    size_t mountRetries;
    millis_t mountDebounceTimeMS;
    bool printingSilent; // Will not report when starting or finishing printing
};

extern SDCard sd;
#endif

extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout
// extern void updateStepsParameter(PrintLine *p/*,uint8_t caller*/);

#ifdef DEBUG_PRINT
extern int debugWaitLoop;
#endif

#include "communication/Commands.h"
#include "communication/Eeprom.h"

#if SDSUPPORT
#include "communication/CSVParser.h"
#endif

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
#include "drivers/zprobe.h"
#include "utilities/RVector3.h"

#include "Events.h"
#include "custom/customEvents.h"

// must be after CustomEvents as it might include definitions from there
// #include "controller/DisplayList.h"

#include "controller/gui.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#endif
