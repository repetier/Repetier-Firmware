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

#define REPETIER_VERSION "0.91"

// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE
/** Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data througput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for seraching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE
/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT
/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
#define DEBUG_COM_ERRORS
/** Adds a menu point in quick settings to write debg informations to the host in case of hangs where the ui still works. */
//#define DEBUG_PRINT
//#define DEBUG_DELTA_OVERFLOW
//#define DEBUG_DELTA_REALPOS
//#define DEBUG_SPLIT
// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH
// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG


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


// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
// Build median from 2^ANALOG_INPUT_SAMPLE samples
#define ANALOG_INPUT_SAMPLE 5
#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF ANALOG_REF_AVCC

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#define MICROSTEP16 HIGH,HIGH

#define HOME_ORDER_XYZ 1
#define HOME_ORDER_XZY 2
#define HOME_ORDER_YXZ 3
#define HOME_ORDER_YZX 4
#define HOME_ORDER_ZXY 5
#define HOME_ORDER_ZYX 6

#define TEMP_PID true // add pid control


#include "Configuration.h"

#ifndef FEATURE_BABYSTEPPING
#define FEATURE_BABYSTEPPING 0
#define BABYSTEP_MULTIPLICATOR 1
#endif

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

#define SPEED_MIN_MILLIS 300
#define SPEED_MAX_MILLIS 50
#define SPEED_MAGNIFICATION 100.0f

#ifndef UI_SPEEDDEPENDENT_POSITIONING
#define UI_SPEEDDEPENDENT_POSITIONING true
#endif

#if DRIVE_SYSTEM==3 || DRIVE_SYSTEM==4 || DRIVE_SYSTEM==5 || DRIVE_SYSTEM==6
#define NONLINEAR_SYSTEM true
#else
#define NONLINEAR_SYSTEM false
#endif

#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL true
#endif

#if DRIVE_SYSTEM==1 || DRIVE_SYSTEM==2
#define XY_GANTRY
#endif

//Step to split a cirrcle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS caluclation is startet to correct the circle interpolation
#define N_ARC_CORRECTION 25

#if NUM_EXTRUDER>0 && EXT0_TEMPSENSOR_TYPE<101
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

#if NUM_EXTRUDER>1 && EXT1_TEMPSENSOR_TYPE<101
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

#if NUM_EXTRUDER>2 && EXT2_TEMPSENSOR_TYPE<101
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

#if NUM_EXTRUDER>3 && EXT3_TEMPSENSOR_TYPE<101
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

#if NUM_EXTRUDER>4 && EXT4_TEMPSENSOR_TYPE<101
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

#if NUM_EXTRUDER>5 && EXT5_TEMPSENSOR_TYPE<101
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

#if HAVE_HEATED_BED==true && HEATED_BED_SENSOR_TYPE<101
#define BED_ANALOG_INPUTS 1
#define BED_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS
#define BED_ANALOG_CHANNEL ACCOMMA5 HEATED_BED_SENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define BED_ANALOG_INPUTS 0
#define BED_SENSOR_INDEX HEATED_BED_SENSOR_PIN
#define BED_ANALOG_CHANNEL
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
#endif

/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS (EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS)
#if ANALOG_INPUTS>0
/** Channels are the MUX-part of ADMUX register */
#define  ANALOG_INPUT_CHANNELS {EXT0_ANALOG_CHANNEL EXT1_ANALOG_CHANNEL EXT2_ANALOG_CHANNEL EXT3_ANALOG_CHANNEL EXT4_ANALOG_CHANNEL EXT5_ANALOG_CHANNEL BED_ANALOG_CHANNEL}
#endif

#define MENU_MODE_SD_MOUNTED 1
#define MENU_MODE_SD_PRINTING 2
#define MENU_MODE_SD_PAUSED 4
#define MENU_MODE_FAN_RUNNING 8
#define MENU_MODE_PRINTING 16

#include "HAL.h"
#include "gcode.h"
#define MAX_VFAT_ENTRIES (2)
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH 2

#include "ui.h"
#include "Communication.h"

#ifndef SDSUPPORT
#define SDSUPPORT false
#endif
#if SDSUPPORT
#include "SdFat.h"
#endif
#ifndef SDSUPPORT
#define SDSUPPORT false
#endif
#if SDSUPPORT
#include "SdFat.h"
#endif

#if ENABLE_BACKLASH_COMPENSATION && DRIVE_SYSTEM!=0
#undef ENABLE_BACKLASH_COMPENSATION
#define ENABLE_BACKLASH_COMPENSATION false
#endif

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t

#define IGNORE_COORDINATE 99999

#undef min
#undef max

class RMath {
public:
    static inline float min(float a,float b) {
        if(a<b) return a;
        return b;
    }
    static inline float max(float a,float b) {
        if(a<b) return b;
        return a;
    }
    static inline long min(long a,long b) {
        if(a<b) return a;
        return b;
    }
    static inline long max(long a,long b) {
        if(a<b) return b;
        return a;
    }
    static inline int min(int a,int b) {
        if(a<b) return a;
        return b;
    }
    static inline int max(int a,int b) {
        if(a<b) return b;
        return a;
    }
    static inline unsigned int min(unsigned int a,unsigned int b) {
        if(a<b) return a;
        return b;
    }
    static inline unsigned int max(unsigned int a,unsigned int b) {
        if(a<b) return b;
        return a;
    }
    static inline long sqr(long a) {return a*a;}
    static inline float sqr(float a) {return a*a;}
};

extern const uint8 osAnalogInputChannels[] PROGMEM;
extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
extern uint osAnalogInputBuildup[ANALOG_INPUTS];
extern uint8 osAnalogInputPos; // Current sampling position
extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
extern uint8_t pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
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
#ifdef SOFTWARE_LEVELING
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
#if OS_ANALOG_INPUTS>0
// Get last result for pin x
extern volatile uint osAnalogInputValues[OS_ANALOG_INPUTS];
#endif

#include "HAL.h"


extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter250ms;
extern void writeMonitor();



#if SDSUPPORT
extern char tempLongFilename[LONG_FILENAME_LENGTH+1];
extern char fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];
#define SHORT_FILENAME_LENGTH 14
#include "SdFat.h"

enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
class SDCard {
public:
  SdFat fat;
  //Sd2Card card; // ~14 Byte
  //SdVolume volume;
  //SdFile root;
  //SdFile dir[SD_MAX_FOLDER_DEPTH+1];
  SdFile file;
  uint32_t filesize;
  uint32_t sdpos;
  //char fullName[13*SD_MAX_FOLDER_DEPTH+13]; // Fill name
  char *shortname; // Pointer to start of filename itself
  char *pathend; // File to char where pathname in fullname ends
  bool sdmode;  // true if we are printing from sd card
  bool sdactive;
  //int16_t n;
  bool savetosd;
  SdBaseFile parentFound;

  SDCard();
  void initsd();
  void writeCommand(GCode *code);
  bool selectFile(char *filename,bool silent=false);
  void mount();
  void unmount();
  void startPrint();
  void pausePrint(bool intern = false);
  void continuePrint(bool intern=false);
  void stopPrint();
  inline void setIndex(uint32_t  newpos) { if(!sdactive) return; sdpos = newpos;file.seekSet(sdpos);}
  void printStatus();
  void ls();
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

#endif
