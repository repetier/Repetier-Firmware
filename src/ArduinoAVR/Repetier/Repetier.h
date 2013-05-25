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
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#ifndef _REPETIER_H
#define _REPETIER_H

#define REPETIER_VERSION "0.90alpha"

// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
#define DEBUG_QUEUE_MOVE
/** Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data througput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
//#define INCLUDE_DEBUG_NO_MOVE
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for seraching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE
/** \brief print ops related debug info. */
//#define DEBUG_OPS
/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT

#define DEBUG_DELTA_OVERFLOW

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


// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
// Build median from 2^ANALOG_INPUT_SAMPLE samples
#define ANALOG_INPUT_SAMPLE 5
#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)

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

#include "Configuration.h"

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

#include "HAL.h"
#include "gcode.h"

#define SD_MAX_FOLDER_DEPTH 2

#include "ui.h"

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

/*#if MOTHERBOARD==6 || MOTHERBOARD==62 || MOTHERBOARD==7
#if MOTHERBOARD!=7
#define SIMULATE_PWM
#endif
#define EXTRUDER_TIMER_VECTOR TIMER2_COMPA_vect
#define EXTRUDER_OCR OCR2A
#define EXTRUDER_TCCR TCCR2A
#define EXTRUDER_TIMSK TIMSK2
#define EXTRUDER_OCIE OCIE2A
#define PWM_TIMER_VECTOR TIMER2_COMPB_vect
#define PWM_OCR OCR2B
#define PWM_TCCR TCCR2B
#define PWM_TIMSK TIMSK2
#define PWM_OCIE OCIE2B
#else*/
#define EXTRUDER_TIMER_VECTOR TIMER0_COMPA_vect
#define EXTRUDER_OCR OCR0A
#define EXTRUDER_TCCR TCCR0A
#define EXTRUDER_TIMSK TIMSK0
#define EXTRUDER_OCIE OCIE0A
#define PWM_TIMER_VECTOR TIMER0_COMPB_vect
#define PWM_OCR OCR0B
#define PWM_TCCR TCCR0A
#define PWM_TIMSK TIMSK0
#define PWM_OCIE OCIE0B
//#endif

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
};

extern const uint8 osAnalogInputChannels[] PROGMEM;
extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
extern uint osAnalogInputBuildup[ANALOG_INPUTS];
extern uint8 osAnalogInputPos; // Current sampling position
extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
extern byte pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
extern int maxadv;
#endif
extern int maxadv2;
extern float maxadvspeed;
#endif


#include "Extruder.h"

void manage_inactivity(byte debug);

extern void finishNextSegment();
#if DRIVE_SYSTEM==3
extern byte calculate_delta(long cartesianPosSteps[], long deltaPosSteps[]);
extern void set_delta_position(long xaxis, long yaxis, long zaxis);
extern float rodMaxLength;
extern void split_delta_move(byte check_endstops,byte pathOptimize, byte softEndstop);
#ifdef SOFTWARE_LEVELING
extern void calculate_plane(long factors[], long p1[], long p2[], long p3[]);
extern float calc_zoffset(long factors[], long pointX, long pointY);
#endif
#endif
extern void linear_move(long steps_remaining[]);



extern unsigned long previous_millis_cmd;
extern unsigned long max_inactive_time;
extern unsigned long stepper_inactive_time;

extern void setupTimerInterrupt();
extern void current_control_init();
extern void microstep_init();
extern void check_mem();

#include "Printer.h"
#include "motion.h"
extern long baudrate;
#if OS_ANALOG_INPUTS>0
// Get last result for pin x
extern volatile uint osAnalogInputValues[OS_ANALOG_INPUTS];
#endif

#include "HAL.h"


extern unsigned int counter_periodical;
extern volatile byte execute_periodical;
extern byte counter_250ms;
extern void write_monitor();



#if SDSUPPORT


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
  char fullName[13*SD_MAX_FOLDER_DEPTH+13]; // Fill name
  char *shortname; // Pointer to start of filename itself
  char *pathend; // File to char where pathname in fullname ends
  bool sdmode;  // true if we are printing from sd card
  bool sdactive;
  //int16_t n;
  bool savetosd;
  SDCard();
  void initsd();
  void write_command(GCode *code);
  void selectFile(char *filename);
  inline void mount() {
    sdmode = false;
    initsd();
  }
  inline void unmount() {
    sdmode = false;
    sdactive = false;
  }
  inline void startPrint() {if(sdactive) sdmode = true; }
  inline void pausePrint() {sdmode = false;}
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
private:
  void lsRecursive(SdBaseFile *parent,byte level);
 // SdFile *getDirectory(char* name);
};

extern SDCard sd;
#endif

extern int waitRelax; // Delay filament relax at the end of print, could be a simple timeout
extern void updateStepsParameter(PrintLine *p/*,byte caller*/);


#if DRIVE_SYSTEM==3
#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_DIAGONAL_ROD_STEPS (AXIS_STEPS_PER_MM * DELTA_DIAGONAL_ROD)
#define DELTA_DIAGONAL_ROD_STEPS_SQUARED (DELTA_DIAGONAL_ROD_STEPS * DELTA_DIAGONAL_ROD_STEPS)
#define DELTA_ZERO_OFFSET_STEPS (AXIS_STEPS_PER_MM * DELTA_ZERO_OFFSET)
#define DELTA_RADIUS_STEPS (AXIS_STEPS_PER_MM * DELTA_RADIUS)

#define DELTA_TOWER1_X_STEPS -SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER1_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_X_STEPS SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER3_X_STEPS 0.0
#define DELTA_TOWER3_Y_STEPS DELTA_RADIUS_STEPS

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#endif

#define STR(s) #s
#define XSTR(s) STR(s)
#include "Commands.h"
#include "Eeprom.h"
#include "Communication.h"

#endif
