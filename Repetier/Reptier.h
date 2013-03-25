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

#include <avr/io.h>

#define REPETIER_VERSION "0.81"

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

#define NEW_XY_GANTRY

#include "Configuration.h"
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
#if CPU_ARCH==ARCH_AVR
#include <avr/io.h>
#else
#define PROGMEM
#define PGM_P const char *
#define PSTR(s) s
#define pgm_read_byte_near(x) (*(char*)x)
#define pgm_read_byte(x) (*(char*)x)
#endif

#define KOMMA
#if NUM_EXTRUDER>0 && EXT0_TEMPSENSOR_TYPE<101
#define EXT0_ANALOG_INPUTS 1
#define EXT0_SENSOR_INDEX 0
#define EXT0_ANALOG_CHANNEL EXT0_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT0_ANALOG_INPUTS 0
#define EXT0_SENSOR_INDEX EXT0_TEMPSENSOR_PIN
#define EXT0_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER>1 && EXT1_TEMPSENSOR_TYPE<101
#define EXT1_ANALOG_INPUTS 1
#define EXT1_SENSOR_INDEX EXT0_ANALOG_INPUTS
#define EXT1_ANALOG_CHANNEL KOMMA EXT1_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT1_ANALOG_INPUTS 0
#define EXT1_SENSOR_INDEX EXT1_TEMPSENSOR_PIN
#define EXT1_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER>2 && EXT2_TEMPSENSOR_TYPE<101
#define EXT2_ANALOG_INPUTS 1
#define EXT2_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS
#define EXT2_ANALOG_CHANNEL KOMMA EXT2_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT2_ANALOG_INPUTS 0
#define EXT2_SENSOR_INDEX EXT2_TEMPSENSOR_PIN
#define EXT2_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER>3 && EXT3_TEMPSENSOR_TYPE<101
#define EXT3_ANALOG_INPUTS 1
#define EXT3_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS
#define EXT3_ANALOG_CHANNEL KOMMA EXT3_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT3_ANALOG_INPUTS 0
#define EXT3_SENSOR_INDEX EXT3_TEMPSENSOR_PIN
#define EXT3_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER>4 && EXT4_TEMPSENSOR_TYPE<101
#define EXT4_ANALOG_INPUTS 1
#define EXT4_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS
#define EXT4_ANALOG_CHANNEL KOMMA EXT4_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT4_ANALOG_INPUTS 0
#define EXT4_SENSOR_INDEX EXT4_TEMPSENSOR_PIN
#define EXT4_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER>5 && EXT5_TEMPSENSOR_TYPE<101
#define EXT5_ANALOG_INPUTS 1
#define EXT5_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS
#define EXT5_ANALOG_CHANNEL KOMMA EXT5_TEMPSENSOR_PIN
#undef KOMMA
#define KOMMA ,
#else
#define EXT5_ANALOG_INPUTS 0
#define EXT5_SENSOR_INDEX EXT5_TEMPSENSOR_PIN
#define EXT5_ANALOG_CHANNEL
#endif

#if HAVE_HEATED_BED==true && HEATED_BED_SENSOR_TYPE<101
#define BED_ANALOG_INPUTS 1
#define BED_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS
#define BED_ANALOG_CHANNEL KOMMA  HEATED_BED_SENSOR_PIN
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
#define DEBUG_MEMORY check_mem();
#endif

/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS (EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS)
#if ANALOG_INPUTS>0
/** Channels are the MUX-part of ADMUX register */
#define  ANALOG_INPUT_CHANNELS {EXT0_ANALOG_CHANNEL EXT1_ANALOG_CHANNEL EXT2_ANALOG_CHANNEL EXT3_ANALOG_CHANNEL EXT4_ANALOG_CHANNEL EXT5_ANALOG_CHANNEL BED_ANALOG_CHANNEL}
#endif
#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)

#if MOTHERBOARD==8 || MOTHERBOARD==9 || CPU_ARCH!=ARCH_AVR
#define EXTERNALSERIAL
#endif
//#define EXTERNALSERIAL  // Force using arduino serial
#ifndef EXTERNALSERIAL
#define  HardwareSerial_h // Don't use standard serial console
#endif
#include <inttypes.h>
#include "Print.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#include "gcode.h"
#if CPU_ARCH==ARCH_AVR
#include "fastio.h"
#else
#define	READ(IO)  digitalRead(IO)
#define	WRITE(IO, v)  digitalWrite(IO, v)
#define	SET_INPUT(IO)  pinMode(IO, INPUT)
#define	SET_OUTPUT(IO)  pinMode(IO, OUTPUT)
#endif
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

/** TemperatureController manages one heater-temperature sensore loop. You can have up to
4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
typedef struct {
  byte pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
  byte sensorType; ///< Type of temperature sensor.
  byte sensorPin; ///< Pin to read extruder temperature.
  int currentTemperature; ///< Currenttemperature value read from sensor.
  int targetTemperature; ///< Target temperature value in units of sensor.
  float currentTemperatureC; ///< Current temperature in °C.
  float targetTemperatureC; ///< Target temperature in °C.
  unsigned long lastTemperatureUpdate; ///< Time in millis of the last temperature update.
  char heatManager; ///< How is temperature controled. 0 = on/off, 1 = PID-Control
#ifdef TEMP_PID
  long tempIState; ///< Temp. var. for PID computation.
  byte pidDriveMax; ///< Used for windup in PID calculation.
  byte pidDriveMin; ///< Used for windup in PID calculation.
  float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
  float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
  float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
  byte pidMax; ///< Maximum PWM value, the heater should be set.
  float tempIStateLimitMax;
  float tempIStateLimitMin;
  byte tempPointer;
  float tempArray[4];
#endif
} TemperatureController;
/** \brief Data to drive one extruder.

This structure contains all definitions for an extruder and all
current state variables, like current temperature, feeder position etc.
*/
typedef struct { // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
  byte id;
  long xOffset;
  long yOffset;
  float stepsPerMM;        ///< Steps per mm.
  byte enablePin;          ///< Pin to enable extruder stepper motor.
//  byte directionPin; ///< Pin number to assign the direction.
//  byte stepPin; ///< Pin number for a step.
  byte enableOn;
//  byte invertDir; ///< 1 if the direction of the extruder should be inverted.
  float maxFeedrate;      ///< Maximum feedrate in mm/s.
  float maxAcceleration;  ///< Maximum acceleration in mm/s^2.
  float maxStartFeedrate; ///< Maximum start feedrate in mm/s.
  long extrudePosition;   ///< Current extruder position in steps.
  int watchPeriod;        ///< Time in seconds, a M109 command will wait to stabalize temperature
  int waitRetractTemperature; ///< Temperature to retract the filament when waiting for heatup
  int waitRetractUnits;   ///< Units to retract the filament when waiting for heatup
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  float advanceK;         ///< Koefficient for advance algorithm. 0 = off
#endif
  float advanceL;
#endif
  TemperatureController tempControl;
  const char * PROGMEM selectCommands;
  const char * PROGMEM deselectCommands;
  byte coolerSpeed; ///< Speed to use when enabled
  byte coolerPWM; ///< current PWM setting
} Extruder;

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

extern Extruder *current_extruder;
extern Extruder extruder[];
// Initalize extruder and heated bed related pins
extern void initExtruder();
extern void initHeatedBed();
extern void updateTempControlVars(TemperatureController *tc);
extern void extruder_select(byte ext_num);
// Set current extruder position
//extern void extruder_set_position(float pos,bool relative);
// set the temperature of current extruder
extern void extruder_set_temperature(float temp_celsius,byte extr);
// Set temperature of heated bed
extern void heated_bed_set_temperature(float temp_celsius);
//extern long extruder_steps_to_position(float value,byte relative);
extern void extruder_set_direction(byte steps);
extern void extruder_disable();
#ifdef TEMP_PID
void autotunePID(float temp,int controllerId);
#endif

//#ifdef TEMP_PID
//extern byte current_extruder_out;
//#endif

/** \brief Sends the high-signal to the stepper for next extruder step. 

Call this function only, if interrupts are disabled.
*/
inline void extruder_step() {
#if NUM_EXTRUDER==1
  WRITE(EXT0_STEP_PIN,HIGH);
#else
  switch(current_extruder->id) {
  case 0:
#if NUM_EXTRUDER>0
    WRITE(EXT0_STEP_PIN,HIGH);
#endif
    break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
  case 1:
    WRITE(EXT1_STEP_PIN,HIGH);
    break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER>2
  case 2:
    WRITE(EXT2_STEP_PIN,HIGH);
    break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER>3
  case 3:
    WRITE(EXT3_STEP_PIN,HIGH);
    break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER>4
  case 4:
    WRITE(EXT4_STEP_PIN,HIGH);
    break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER>5
  case 5:
    WRITE(EXT5_STEP_PIN,HIGH);
    break;
#endif
  }
#endif
}
/** \brief Sets stepper signal to low for current extruder. 

Call this function only, if interrupts are disabled.
*/
inline void extruder_unstep() {
#if NUM_EXTRUDER==1
  WRITE(EXT0_STEP_PIN,LOW);
#else
  switch(current_extruder->id) {
  case 0:
#if NUM_EXTRUDER>0
    WRITE(EXT0_STEP_PIN,LOW);
#endif
    break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
  case 1:
    WRITE(EXT1_STEP_PIN,LOW);
    break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER>2
  case 2:
    WRITE(EXT2_STEP_PIN,LOW);
    break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER>3
  case 3:
    WRITE(EXT3_STEP_PIN,LOW);
    break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER>4
  case 4:
    WRITE(EXT4_STEP_PIN,LOW);
    break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER>5
  case 5:
    WRITE(EXT5_STEP_PIN,LOW);
    break;
#endif
  }
#endif
}
/** \brief Activates the extruder stepper and sets the direction. */
inline void extruder_set_direction(byte dir) {  
#if NUM_EXTRUDER==1
  if(dir)
    WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
  else
    WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
#else
  switch(current_extruder->id) {
#if NUM_EXTRUDER>0
  case 0:
  if(dir)
    WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
  else
    WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
    break;
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER>1
  case 1:
  if(dir)
    WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
  else
    WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
    break;
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER>2
  case 2:
  if(dir)
    WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
  else
    WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
    break;
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER>3
  case 3:
  if(dir)
    WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
  else
    WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
    break;
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER>4
  case 4:
  if(dir)
    WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
  else
    WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
    break;
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER>5
  case 5:
  if(dir)
    WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
  else
    WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
    break;
#endif
  }
#endif
}
inline void extruder_enable() {
#if NUM_EXTRUDER==1
#if EXT0_ENABLE_PIN>-1
    WRITE(EXT0_ENABLE_PIN,EXT0_ENABLE_ON ); 
#endif
#else
  if(current_extruder->enablePin > -1) 
    digitalWrite(current_extruder->enablePin,current_extruder->enableOn); 
#endif
}
extern void(* resetFunc) (void); 
// Read a temperature and return its value in °C
// this high level method supports all known methods
extern int read_raw_temperature(byte type,byte pin);
extern float heated_bed_get_temperature();
// Convert a raw temperature value into °C
extern float conv_raw_temp(byte type,int raw_temp);
// Converts a temperture temp in °C into a raw value
// which can be compared with results of read_raw_temperature
extern int conv_temp_raw(byte type,float temp);
// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern void manage_temperatures();
extern bool reportTempsensorError(); ///< Report defect sensors
extern byte manage_monitor;

void process_command(GCode *code,byte bufferedCommand);

void manage_inactivity(byte debug);

extern void wait_until_end_of_move();
extern void update_ramps_parameter();
extern void update_extruder_flags();
extern void finishNextSegment();
extern void printPosition();
extern void defaultLoopActions();
extern void change_feedrate_multiply(int factor); ///< Set feedrate multiplier
extern void set_fan_speed(int speed,bool wait); /// Set fan speed 0..255
extern void home_axis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
extern byte get_coordinates(GCode *com);
extern void move_steps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop);
extern void queue_move(byte check_endstops,byte pathOptimize);
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
extern inline void disable_x();
extern inline void disable_y();
extern inline void disable_z();
extern inline void enable_x();
extern inline void enable_y();
extern inline void enable_z();

#define PREVIOUS_PLANNER_INDEX(p) {p--;if(p==255) p = MOVE_CACHE_SIZE-1;}
#define NEXT_PLANNER_INDEX(idx) {++idx;if(idx==MOVE_CACHE_SIZE) idx=0;}

extern void kill(byte only_steppers);

extern float axis_steps_per_unit[];
extern float inv_axis_steps_per_unit[];
extern float max_feedrate[];
extern float homing_feedrate[];
extern float max_start_speed_units_per_second[];
extern long max_acceleration_units_per_sq_second[];
extern long max_travel_acceleration_units_per_sq_second[];
extern unsigned long axis_steps_per_sqr_second[];
extern unsigned long axis_travel_steps_per_sqr_second[];
extern byte relative_mode;    ///< Determines absolute (false) or relative Coordinates (true).
extern byte relative_mode_e;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

extern byte unit_inches;
extern unsigned long previous_millis_cmd;
extern unsigned long max_inactive_time;
extern unsigned long stepper_inactive_time;

extern void setupTimerInterrupt();
extern void current_control_init();
extern void microstep_init();
extern void print_temperatures();
extern void check_mem();
#if ARC_SUPPORT
extern void mc_arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif

#define PRINTER_FLAG0_STEPPER_DISABLED      1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     4
#define PRINTER_FLAG0_FORCE_CHECKSUM        8

typedef struct { 
  byte flag0; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect 
#if USE_OPS==1 || defined(USE_ADVANCE)
  volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//  float extruderSpeed;              ///< Extruder speed in mm/s.
  byte minExtruderSpeed;            ///< Timer delay for start extruder speed
  byte maxExtruderSpeed;            ///< Timer delay for end extruder speed
  byte extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
  long interval;                    ///< Last step duration in ticks.
#if USE_OPS==1
  bool filamentRetracted;           ///< Is the extruder filament retracted
#endif
  unsigned long timer;              ///< used for acceleration/deceleration timing
  unsigned long stepNumber;         ///< Step number in current move.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  long advance_executed;             ///< Executed advance steps
#endif
  int advance_steps_set;
  unsigned int advance_lin_set;
#endif
  long currentPositionSteps[4];     ///< Position in steps from origin.
  long destinationSteps[4];         ///< Target position in steps.
#if DRIVE_SYSTEM==3
#ifdef STEP_COUNTER
  long countZSteps;					///< Count of steps from last position reset
#endif
  long currentDeltaPositionSteps[4];
  long maxDeltaPositionSteps;
#endif
#ifdef SOFTWARE_LEVELING
  long levelingP1[3];
  long levelingP2[3];
  long levelingP3[3];
#endif
#if USE_OPS==1
  int opsRetractSteps;              ///< Retract filament this much steps
  int opsPushbackSteps;             ///< Retract+extra distance for backsash
  float opsMinDistance;
  float opsRetractDistance;
  float opsRetractBacklash;
  byte opsMode;                     ///< OPS operation mode. 0 = Off, 1 = Classic, 2 = Fast
  float opsMoveAfter;               ///< Start move after opsModeAfter percent off full retract.
  int opsMoveAfterSteps;            ///< opsMoveAfter converted in steps (negative value!).
#endif
  float minimumSpeed;               ///< lowest allowed speed to keep integration error small
  long xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  long yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  long zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  long xMinSteps;                   ///< For software endstops, limit of move in negative direction.
  long yMinSteps;                   ///< For software endstops, limit of move in negative direction.
  long zMinSteps;                   ///< For software endstops, limit of move in negative direction.
  float xLength;
  float xMin;
  float yLength;
  float yMin;
  float zLength;
  float zMin;
  float feedrate;                   ///< Last requested feedrate.
  int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
  unsigned int extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
  float maxJerk;                    ///< Maximum allowed jerk in mm/s
  float maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
  long offsetX;                     ///< X-offset for different extruder positions.
  long offsetY;                     ///< Y-offset for different extruder positions.
  unsigned int vMaxReached;         ///< MAximumu reached speed
  byte stepper_loops;
  unsigned long msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
  float filamentPrinted;            ///< mm of filament printed since counting started
  byte waslasthalfstepping;         ///< Indicates if last move had halfstepping enabled
#if ENABLE_BACKLASH_COMPENSATION
  float backlashX;
  float backlashY;
  float backlashZ;
  byte backlashDir;
#endif
#ifdef DEBUG_STEPCOUNT
  long totalStepsRemaining;
#endif
#if FEATURE_MEMORY_POSITION
  long memoryX;
  long memoryY;
  long memoryZ;
#endif
#ifdef XY_GANTRY
  char motorX;
  char motorY;
#endif
  inline byte isAdvanceActivated() {return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;}
} PrinterState;
extern PrinterState printer_state;

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_SKIP_ACCELERATING 32
#define FLAG_SKIP_DEACCELERATING 64
#define FLAG_BLOCKED 128

/** Are the step parameter computed */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1
/** The right speed is fixed. Don't check this block or any block to the left. */
#define FLAG_JOIN_END_FIXED 2
/** The left speed is fixed. Don't check left block. */
#define FLAG_JOIN_START_FIXED 4
/** Start filament retraction at move start */
#define FLAG_JOIN_START_RETRACT 8
/** Wait for filament pushback, before ending move */
#define FLAG_JOIN_END_RETRACT 16
/** Disable retract for this line */
#define FLAG_JOIN_NO_RETRACT 32
/** Wait for the extruder to finish it's up movement */
#define FLAG_JOIN_WAIT_EXTRUDER_UP 64
/** Wait for the extruder to finish it's down movement */
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN 128
// Printing related data
#if DRIVE_SYSTEM==3
// Allow the delta cache to store segments for every line in line cache. Beware this gets big ... fast.
// MAX_DELTA_SEGMENTS_PER_LINE * 
#define DELTA_CACHE_SIZE (MAX_DELTA_SEGMENTS_PER_LINE * MOVE_CACHE_SIZE)
typedef struct { 
	byte dir; 									///< Direction of delta movement.
	unsigned int deltaSteps[3]; 				///< Number of steps in move.
} DeltaSegment;
extern DeltaSegment segments[];					// Delta segment cache
extern unsigned int delta_segment_write_pos; 	// Position where we write the next cached delta move
extern volatile unsigned int delta_segment_count; // Number of delta moves cached 0 = nothing in cache
extern byte lastMoveID;
#endif
typedef struct { // RAM usage: 24*4+15 = 113 Byte
  byte primaryAxis;
  volatile byte flags;
  long timeInTicks;
  byte joinFlags;
  byte halfstep;                  ///< 0 = disabled, 1 = halfstep, 2 = fulstep
  byte dir;                       ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
  long delta[4];                  ///< Steps we want to move.
  long error[4];                  ///< Error calculation for Bresenham algorithm
  float speedX;                   ///< Speed in x direction at fullInterval in mm/s
  float speedY;                   ///< Speed in y direction at fullInterval in mm/s
  float speedZ;                   ///< Speed in z direction at fullInterval in mm/s
  float speedE;                   ///< Speed in E direction at fullInterval in mm/s
  float fullSpeed;                ///< Desired speed mm/s
  float invFullSpeed;             ///< 1.0/fullSpeed for fatser computation
  float acceleration;             ///< Real 2.0*distanceÜacceleration mm²/s²
  float maxJunctionSpeed;         ///< Max. junction speed between this and next segment
  float startSpeed;               ///< Staring speed in mm/s
  float endSpeed;                 ///< Exit speed in mm/s
  float distance;
#if DRIVE_SYSTEM==3
  byte numDeltaSegments;		  		///< Number of delta segments left in line. Decremented by stepper timer.
  byte moveID;							///< ID used to identify moves which are all part of the same line
  int deltaSegmentReadPos; 	 			///< Pointer to next DeltaSegment
  long numPrimaryStepPerSegment;		///< Number of primary bresenham axis steps in each delta segment
#endif
  unsigned long fullInterval;     ///< interval at full speed in ticks/step.
  unsigned long stepsRemaining;   ///< Remaining steps, until move is finished
  unsigned int accelSteps;        ///< How much steps does it take, to reach the plateau.
  unsigned int decelSteps;        ///< How much steps does it take, to reach the end speed.
  unsigned long accelerationPrim; ///< Acceleration along primary axis
  unsigned long facceleration;    ///< accelerationPrim*262144/F_CPU
  unsigned int vMax;              ///< Maximum reached speed in steps/s.
  unsigned int vStart;            ///< Starting speed in steps/s.
  unsigned int vEnd;              ///< End speed in steps/s
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
  long advanceRate;               ///< Advance steps at full speed
  long advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
  long advanceStart;
  long advanceEnd;
#endif
  unsigned int advanceL;         ///< Recomputated L value
#endif
#if USE_OPS==1
  long opsReverseSteps;           ///< How many steps are needed to reverse retracted filament at full speed
#endif
#ifdef DEBUG_STEPCOUNT
  long totalStepsRemaining;
#endif
} PrintLine;

extern PrintLine lines[];
extern byte lines_write_pos; // Position where we write the next cached line move
extern byte lines_pos; // Position for executing line movement
extern volatile byte lines_count; // Number of lines cached 0 = nothing to do
extern byte printmoveSeen;
extern long baudrate;
#if OS_ANALOG_INPUTS>0
// Get last result for pin x
extern volatile uint osAnalogInputValues[OS_ANALOG_INPUTS];
#endif
#define BEGIN_INTERRUPT_PROTECTED {byte sreg=SREG;__asm volatile( "cli" ::: "memory" );
#define END_INTERRUPT_PROTECTED SREG=sreg;}
#define ESCAPE_INTERRUPT_PROTECTED SREG=sreg;

#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
extern long CPUDivU2(unsigned int divisor);

extern unsigned int counter_periodical;
extern volatile byte execute_periodical;
extern byte counter_250ms;
extern void write_monitor();
extern void check_periodical();
#define CELSIUS_EXTRA_BITS 3
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
extern TemperatureController heatedBedController;
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif
#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
extern byte autotuneIndex;

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
#ifdef USE_OPS
extern byte printmoveSeen;
#endif
extern void updateStepsParameter(PrintLine *p/*,byte caller*/);

/** \brief Disable stepper motor for x direction. */
inline void disable_x() {
#if (X_ENABLE_PIN > -1)
  WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif
}
/** \brief Disable stepper motor for y direction. */
inline void disable_y() {
#if (Y_ENABLE_PIN > -1)
  WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif
}
/** \brief Disable stepper motor for z direction. */
inline void disable_z() {
#if (Z_ENABLE_PIN > -1)
 WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for x direction. */
inline void  enable_x() {
#if (X_ENABLE_PIN > -1)
 WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for y direction. */
inline void  enable_y() {
#if (Y_ENABLE_PIN > -1)
  WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
}
/** \brief Enable stepper motor for z direction. */
inline void  enable_z() {
#if (Z_ENABLE_PIN > -1)
 WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
}


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

#endif
