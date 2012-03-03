/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2  = 3,
// RAMPS 1.3/RAMPS 1.4 = 33
// Gen6 = 5, 
// Sanguinololu up to 1.1 = 6
// Sanguinololu 1.2 and above = 62
// Gen7 1.1 and above = 7
#define MOTHERBOARD 33
#include <avr/io.h>
#include "pins.h"


// ##########################################################################################
// ##                                        IMPORTANT                                     ##
// ##########################################################################################

// For easy configuration, the default settings enable parameter storage in EEPROM.
// This means, after the first upload many variables can only be changed using the special
// M commands as described in the documentation. Changing these value sin the configuration.h
// has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder 
// values except thermistor tables and some other parameter likely to change during usage
// like advance steps or ops mode.
// To override EEPROM settings with config settings, set EEPROM_MODE 0

// ##########################################################################################
// ##                               Calibration                                            ##
// ##########################################################################################


/** \brief Number of steps for a 1mm move in x direction. Overridden if EEPROM activated. */
#define XAXIS_STEPS_PER_MM 40
/** \brief Number of steps for a 1mm move in y direction  Overridden if EEPROM activated.*/
#define YAXIS_STEPS_PER_MM 40
/** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
#define ZAXIS_STEPS_PER_MM 3360

// ##########################################################################################
// ##                           Extruder configuration                                     ##
// ##########################################################################################

/** Number of extruders. Maximum 2 extruder. */
#define NUM_EXTRUDER 1

#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
// for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT0_STEPS_PER_MM 373
// What type of sensor is used?
// 1 is 100k thermistor
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 99 Generic thermistor table
// 100 is AD595
// 101 is MAX6675
#define EXT0_TEMPSENSOR_TYPE 5
// Position in analog input table below for reading temperatures or pin enabling SS for MAX6675
#define EXT0_TEMPSENSOR_PIN 0
// WHich pin enables the heater
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E_STEP_PIN
#define EXT0_DIR_PIN E_DIR_PIN
// set to 0/1 for normal / inverse direction
#define EXT0_INVERSE false
#define EXT0_ENABLE_PIN E_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT0_ENABLE_ON false
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use hiher values.
//  Overridden if EEPROM activated.
#define EXT0_MAX_FEEDRATE 1200
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT0_MAX_START_FEEDRATE 10
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT0_MAX_ACCELERATION 10000
/** Type of heat manager for this extruder. 
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better, but needs an output with PWM, which doesn't
      use Timer 0 and 1
 Overridden if EEPROM activated.
*/
#define EXT0_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT0_WATCHPERIOD 20

/** \brief The maximum value, I-gain can contribute to the output. 

A good value is slightly higher then the output needed for your temperature.
Values for startes:
130 => PLA for temperatures from 170-180°C
180 => ABS for temperatures around 240°C

The precice values may differ for different nozzle/resistor combination. 
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MAX 130
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MIN 50
/** P-gain in 0,01 units.  Overridden if EEPROM activated. */
#define EXT0_PID_PGAIN   500
/** I-gain in 0,001 units 

WATCH OUT: This value was in 0,01 units in earlier versions!
 Overridden if EEPROM activated.
*/
#define EXT0_PID_IGAIN   1
/** Dgain in 0,01 units.  Overridden if EEPROM activated.*/
#define EXT0_PID_DGAIN 3000
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT0_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.*/
#define EXT0_ADVANCE_K 0.0f
/** Number of entries in the user thermistortable 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0 28
/** Number of entries in the user thermistortable 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1 0
/** Number of entries in the user thermistortable 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2 0
/** \brief Set PID scaling 

PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by to methods.
Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
If your EXT0_PID_MAX is low, you should prefer the second method.
*/
#define SCALE_PID_TO_MAX 0

/** Temperature range for target temperature to hold in M109 command. 5 means +/-5°C

Uncomment define to use force the temperature into the range for given watchperiod. 
*/
//#define TEMP_HYSTERESIS 5

/** \brief Simulate PWM

If your extruder uses a pin without pwm or with a pwm already in use for other functions, you
can enable the PWM simulator instead. Only with PWM you can use PID control.
For Sanguino boards (6 and 62) this is enabled by default. To force simulated PWM uncomment
the define.
*/
//#define SIMULATE_PWM

/** Userdefined thermistor table

There are many different thermistors, which can be combined with different resistors. This result
in unpredictable number of tables. As a resolution, the user can define one table here, that can
be used as type 5 for thermister type in extruder/heated bed definition. Make sure, the number of entries
matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
increasing adc values. For more informations, read 
http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
This firmware works with increased precision, so the value read goes from 0 to 4095 and the temperature is 
temperature*8.

If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use themistor types 50-52 instead of 5-7! 
*/
#define USER_THERMISTORTABLE0  {\
  {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},\
  {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},\
  {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	}

#define USER_THERMISTORTABLE1  {}  
#define USER_THERMISTORTABLE2  {}  

/** If defined, creates a thermistortable at startup.

If you dont feel like computing the table on your own, you can use this generic method. It is
a simple approximation which may be not as accurate as a good table computed from the reference
values in the datasheet. You can increase precision if you use a temperature/resistance for
R0/T0, which is near your operating temperature. This will reduce precision for lower temperatures,
which are not realy important. The resistors must fit the following schematic:
@code
VREF ---- R2 ---+--- Termistor ---+-- GND
                |                 |
                +------ R1 -------+
                |                 |
                +---- Capacitor --+
                |
                V measured
@endcode                
                
If you don't have R1, set it to 0.
The capacitor is for reducing noise from long thermistor cable. If you don't have have one, it's OK.

If you don't need the generic table, uncomment the following define.
*/
//#define USE_GENERIC_THERMISTORTABLE 1
/** Reference resistance */
#define GENERIC_THERM_R0 1042.7
/** Temperature at reference resistance */
#define GENERIC_THERM_T0 170
/** Beta value of thermistor

You can use the beta from the datasheet or compute it yourself. See
http://reprap.org/wiki/MeasuringThermistorBeta
for more details.
*/
#define GENERIC_THERM_BETA 4036
#define GENERIC_THERM_R1 0
#define GENERIC_THERM_R2 4700
#define GENERIC_THERM_VREF 5
/** Supply voltage to ADC, can be changed be setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VADC 5
/** Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too. */
#define GENERIC_THERM_NUM_ENTRIES 40

// uncomment the following line for MAX6675 support.
//#define SUPPORT_MAX6675

// ############# Heated bed configuration ########################

/** \brief Switches fast between config for heated bed and non heated bed */
#define HAVE_HEATED_BED false

#if HAVE_HEATED_BED==true
// Select type of your heated bed. It's the same as for EXT0_TEMPSENSOR_TYPE
// set to 0 if you don't have a heated bed
#define HEATED_BED_SENSOR_TYPE 0
/** Index of analog sensor to read temperature of heated bed. look at ANALOG_INPUT_CHANNELS for the position or to add the Arduino pin id there. */
#define HEATED_BED_SENSOR_PIN 1
/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
// How often the temperature of the heated bed is set (msec)
#define HEATED_BED_SET_INTERVAL 5000
#else
#define HEATED_BED_SENSOR_TYPE 0
#define HEATED_BED_SENSOR_PIN -1
#define HEATED_BED_HEATER_PIN -1
#endif

// uncomment to use AREF for reference voltage
// on a GEN6 you want AVCC
#define USE_AVCC_FOR_TEMP
// how many samples do we want per reading. 1 sample takes 1/125000 seconds.
// more samples get more reliable values, but take more time.
#define ANALOG_SUPERSAMPLE 10
/** The number of analog sensors, we need to read out. These are the thermistors used for temperature
reading of the extruder and heated bed. */
#if HAVE_HEATED_BED==true
#define NUM_ANALOG_SENSORS 2
#else
#define NUM_ANALOG_SENSORS 1
#endif
/** Number of digital temp. sensors like MAX6675 */
#define NUM_DIGITAL_SENSORS 0
#define TEMP_PID true

//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 275

/**
Normally you need a PWM controlable output, to define different fan speeds. If you
don't have one, you can only turn your fan on or off.

As a sulution, you can simulate PWM for your pin. This works for all pins, even non-PWM pins!
If your fan is connected to a PWM output that is used by the firmware internally, you must
activate PWM simulation.

To active fan PWM simulation uncomment the next define.
*/
//#define SIMULATE_FAN_PWM

/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS NUM_ANALOG_SENSORS
#if ANALOG_INPUTS>0
/** Channels are the MUX-part of ADMUX register 

Put all the pin numbers for the analog sensors (temp. sensor for extruder and heated bed) in here.
In the configs of the sensor, use the index in this array. For the typical combination of
one extruder with heated bed, write:
#define  ANALOG_INPUT_CHANNELS {TEMP_0_PIN,TEMP_1_PIN}
*/
#if HAVE_HEATED_BED==true
#define  ANALOG_INPUT_CHANNELS {TEMP_0_PIN,TEMP_1_PIN}
#else
#define ANALOG_INPUT_CHANNELS {TEMP_0_PIN}
#endif
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
// Build median from 2^ANALOG_INPUT_SAMPLE samples
#define ANALOG_INPUT_SAMPLE 5
#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)
/** \brief Used reference, normally ANALOG_REF_AVCC or ANALOG_REF_AREF */
#define ANALOG_REF ANALOG_REF_AVCC
#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)


#endif
// ##########################################################################################
// ##                            Endstop configuration                                     ##
// ##########################################################################################

//// Endstop Settings
#define ENDSTOPPULLUPS 0 // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//set to true to invert the logic of the endstops
#define ENDSTOP_X_MIN_INVERTING false
#define ENDSTOP_Y_MIN_INVERTING false
#define ENDSTOP_Z_MIN_INVERTING false
#define ENDSTOP_X_MAX_INVERTING false
#define ENDSTOP_Y_MAX_INVERTING false
#define ENDSTOP_Z_MAX_INVERTING false

//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here



//// ADVANCED SETTINGS - to tweak parameters

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z true
#define DISABLE_E false

// Inverting axis direction
#define INVERT_X_DIR false
#define INVERT_Y_DIR true
#define INVERT_Z_DIR false

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

//If true, axis won't move to coordinates less than zero.
#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false

//If true, axis won't move to coordinates greater than the defined lengths below.
#define max_software_endstop_x true
#define max_software_endstop_y true
#define max_software_endstop_z true

// You can disable endstop checking for print moves. This is needed, if you get sometimes
// false signals from your endstops. If your endstops don't give false signals, you
// can set it on for safety.
#define ALWAYS_CHECK_ENDSTOPS true
// maximum positions in mm - only fixed numbers!
#define X_MAX_LENGTH 200
#define Y_MAX_LENGTH 200
#define Z_MAX_LENGTH 100

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

/** After x seconds of inactivity, the stepper motors are disabled.
    Set to 0 to leave them enabled.
    This helps cooling the Stepper motors between two print jobs. 
    Overridden if EEPROM activated.
*/
#define STEPPER_INACTIVE_TIME 120L
/** After x seconds of inactivity, the system will go down as far it can.
    It will at least disable all stepper motors and heaters. If the board has
    a power pin, it will be disabled, too. 
    Set value to 0 for disabled.  
    Overridden if EEPROM activated. 
*/
#define MAX_INACTIVE_TIME 0L
/** Maximum feedrate, the system allows. Higher feedrates are reduced to these values.
    The axis order in all axis related arrays is X, Y, Z, E 
     Overridden if EEPROM activated.
    */
#define MAX_FEEDRATE {15000, 15000, 100, 100}
/** Speed in mm/min for finding the home position.  Overridden if EEPROM activated. */
#define HOMING_FEEDRATE {2400,2400,100}

/** Comment this to disable ramp acceleration */
#define RAMP_ACCELERATION 1

//// Acceleration settings

/** \brief Use RAMP acceleration for faster printing speed. */
#ifdef RAMP_ACCELERATION
/** \brief X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts. Make sure your printer can go that high! 
 Overridden if EEPROM activated.
*/
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND {3000,3000,100,1000} 
/** \brief X, Y, Z max acceleration in mm/s^2 for travel moves.  Overridden if EEPROM activated.*/
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND {3000,3000,100,1000}
#endif

/** \brief Maximum allowable jerk.

Caution: This is no real jerk in a physical meaning.

The jerk determines your start speed and the maximum speed at the join of two segments.
It's unit is mm/s. If the printer is standing still, the start speed is jerk/2. At the
join of two segments, the speed difference is limited to the jerk value.

Examples:
For all examples jerk is assumed as 40.

Segment 1: vx = 50, vy = 0
Segment 2: vx = 0, vy = 50
v_diff = sqrt((50-0)^2+(0-50)^2) = 70.71
v_diff > jerk => vx_1 = vy_2 = jerk/v_diff*vx_1 = 40/70.71*50 = 28.3 mm/s at the join

Segment 1: vx = 50, vy = 0
Segment 2: vx = 35.36, vy = 35.36
v_diff = sqrt((50-35.36)^2+(0-35.36)^2) = 38.27 < jerk
Corner can be printed with full speed of 50 mm/s

Overridden if EEPROM activated.
*/
#define MAX_JERK 40.0
#define MAX_ZJERK 0.3

// ##########################################################################################
// ##                           Extruder control                                           ##
// ##########################################################################################

/** \brief Prescale factor, timer0 runs at.

All known arduino boards use 64. This value is needed for the extruder timing. */
#define TIMER0_PRESCALE 64

/** \brief speed of the extruder feeder

This is the maximum speed, the filament can be moved forward and backward without acceleration.
The value determines, how often we can update the extruder stepper, to come up with the desired
extruder position. Higher values increase precision. If you set the value too high, you will
lose steps. Only discrete values between 1 and 255 can be set for the timer. The effectife update
frequency is computed as:

f = floor(F_CPU/(TIMER0_PRESCALE*EXTRUDER_SPEED*STEPS_PER_MM))

Important: This is the speed, filament is pushed inside the extruder not the speed at the nozzle!
If you set the extruder steps_per_mm for 1mm pushed outside, cause skeinforge<40 needed it, you must
decrease the value to reflect this. (*filament_diameter^2/nozzle_diameter^2)
*/
#define EXTRUDER_SPEED 20.0

/* \brief Minimum temperature for extruder operation

This is a saftey value. If your extruder temperature is below this temperature, no
extruder steps are executed. This is to prevent your extruder to move unless the fiament
is at least molten. After havong some complains that the extruder does not work, I leave
it 0 as default.
*/

#define MIN_EXTRUDER_TEMP 0
/** \brief Activate ooze prevention system 

The ooze prevention system tries to prevent ooze, by a fast retract of the filament every time
printing stops. Most slicing software have already an option to do this. Using OPS_MODE=1 will
in fact mimic this. This works good, but can increase printing time. To reduce the additional
waiting time, the OPS has a fast mode, which performs the retraction during the travelling move.
The only reason, your slicer doesn't do it, is because it can't tell. There is simple no
G-Code command telling the firmware to do that.

You can always compile including OPS. Then you can disable/enable it anytime you want. To disable it
set USE_OPS 0

Caution: Don't enable anti-ooze in your slicer if you are using this. 
*/
#define USE_OPS 1

/** \brief Sets the ops operation mode

0: Off
1: Classic mode. Stop head, retract move to target, push filament back.
2: Fast mode. Retract during move, start pushing back the filament during move. For safty, we start
   at with a low speed and wait for the push back, before the pintmove starts. Normally there is some
   time needed to wait for the filament.

 Overridden if EEPROM activated.
*/
#define OPS_MODE 0

/** \brief Minimum distance for retraction.

If a travel move is shorter than this distance, no retraction will occur. This is to prevent
retraction with infill, where the angle to the perimeter needs a short stop. Unit is mm.
 Overridden if EEPROM activated.
*/
#define OPS_MIN_DISTANCE 0.8

/** \brief Move printhead only after x% of retract distance have been retracted.

 Overridden if EEPROM activated.*/
#define OPS_MOVE_AFTER 50.0
/** \brief Retraction distance in mm. If you want to enable OPS only sometimes, compile with
OPS support and set retraction distance to 0. If you set it to e.g. 3 in your eeprom settings it is enabled.
 Overridden if EEPROM activated.*/
#define OPS_RETRACT_DISTANCE 3.0

/** \brief Backslash produced by extruder reversal

If you are using a bowden extruder, you may need some extra distance to push the filament back into the 
original place. This is the value you enter here. Unit is mm.
 Overridden if EEPROM activated.
*/
#define OPS_RETRACT_BACKSLASH 0.0



/** \brief Enable advance algorithm.

Without a correct adjusted advance algorithm, you get blobs at points, where acceleration changes. The
effect increases with speed and acceleration difference. Using the advance method decreases this effect.
For more informations, read the wiki.
*/
#define USE_ADVANCE

/** The firmware supports trajectory smoothing. To acieve this, it divides the stepsize by 2, resulting in
the double computation cost. For slow movements this is not an issue, but for really fast moves this is 
too much. The value specified here is the number of clock cycles between a step on the driving axis.
If the interval at full speed is below this value, smoothing is disabled for that line.*/
#define MAX_HALFSTEP_INTERVAL 1999

// ##########################################################################################
// ##                           Communication configuration                                ##
// ##########################################################################################

//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

/** \brief Communication speed.

- 250000 : Fastes with errorrate of 0% with 16 or 32 MHz - update wiring_serial.c in your board files. See boards/readme.txt
- 115200 : Fast, but may produce communication errors on quite regular basis, Error rate -3,5%
- 76800 : Best setting for Arduino with 16 MHz, Error rate 0,2% page 198 AVR1284 Manual. Result: Faster communication then 115200
- 57600 : Should produce nearly no errors, on my gen 6 it's faster than 115200 because there are no errors slowing down the connection
- 38600

 Overridden if EEPROM activated.
*/
//#define BAUDRATE 76800
//#define BAUDRATE 57600
#define BAUDRATE 250000
/** \brief Size in byte of the output buffer */
#define OUTPUT_BUFFER_SIZE 64
/** \brief Activates buffered output.

The Arduino libraries have a buffered input for serial connections. Write operations are
always unbuffered, which means the controller will wait with execution, until the data
is send. To solve this handicap, all write operations have to use out insted of Serial.
If you uncomment this, you still have to use out, but it will use no buffers, so you
keep with your slow communication version.
*/

/**
Some boards like Gen7 have a power on pin, to enable the atx power supply. If this is defined,
the power will be turned on without the need to call M80 if initially started.
*/
#define ENABLE_POWER_ON_STARTUP

#define USE_BUFFERED_OUTPUT 
/** \brief Number of moves we can cache in advance.

This number of moves can be cached in advance. If you wan't to cache more, increase this. Especially on
many very short moves the cache may go empty. The minimum value is 5.
*/
#define MOVE_CACHE_SIZE 16

/** \brief Low filled cache size. 

If the cache contains less then MOVE_CACHE_LOW segments, the time per segment is limited to LOW_TICKS_PER_MOVE clock cycles.
If a move would be shorter, the feedrate will be reduced. This should prevent buffer underflows. Set this to 0 if you
don't care about empty buffers during print.
*/
#define MOVE_CACHE_LOW 12
/** \brief Cycles per move, if move cache is low. 

This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
if you are printing many very short segments at high speed.
*/
#define LOW_TICKS_PER_MOVE 200000
/** \brief Cache size for incoming commands.

There should be no reason to increase this cache. Commands are nearly immediately send to
execution.
*/
#define GCODE_BUFFER_SIZE 2
/** Appends the linenumber after ever ok send, to acknowledge the received command. Uncomment for plain ok ACK if your host has problems with this */
#define ACK_WITH_LINENUMBER
/** Communication errors can swollow part of the ok, which tells the host software to send
the next command. Not receiving it will cause your printer to stop. Sending this string every
second, if our queue is empty should prevent this. Uncomment if you don't wan't this feature. */
#define WAITING_IDENTIFIER "wait"

/** \brief Sets time for echo debug

You can set M111 1 which enables ECHO of commands send. This define specifies the position,
when it will be executed. In the original FiveD software, echo is done after receiving the
command. With checksum you know, how it looks from the sending string. With this define
uncommented, you will see the last command executed. To be more specific: It is written after
execution. This helps tracking errors, because there may be 8 or more commands in the queue
and it is elsewise difficult to know, what your reprap is currently doing.
*/
#define ECHO_ON_EXECUTE

/** \brief EEPROM storage mode 

Set the EEPROM_MODE to 0 if you always wan't to use the settings in this configuration file. If not,
set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
eeprom settings with configuration defaults, just select an other value. On the first call to epr_init()
it will detect a mismatch of the first byte and copys default values into EEPROM. If the first byte
matches, the stored values are used to overwrite the settings.

IMPORTANT: With mode <>0 some changes in configuration.h are not set any more, as they are 
           taken from the EEPROM.
*/
#define EEPROM_MODE 1
/** Comment out (using // at the start of the line) to disable SD support: */
#define SDSUPPORT 1
/** Show extended directory including file length. Don't use this with pronterface! */
#define SD_EXTENDED_DIR

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
#define DEBUG_ADVANCE
/** \brief print ops related debug info. */
//#define DEBUG_OPS
/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG
// ####################################################################################
// #                         Below this line only for experts                         #
// ####################################################################################


// ####################################################################################
// #          No configuration below this line - just some errorchecking              #
// ####################################################################################
#ifdef SUPPORT_MAX6675
#if !defined SCK_PIN || !defined MOSI_PIN || !defined MISO_PIN
#error For MAX6675 support, you need to define SCK_PIN, MISO_PIN and MOSI_PIN in pins.h
#endif
#endif
#if X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if EXT0_STEP_PIN<0
#error EXT0_STEP_PIN not set to a pin number.
#endif
#if EXT0_DIR_PIN<0
#error EXT0_DIR_PIN not set to a pin number.
#endif
#if MOVE_CACHE_SIZE<4
#error MOVE_CACHE_SIZE must be at least 5
#endif
#if OUTPUT_BUFFER_SIZE>250 || OUTPUT_BUFFER_SIZE<16
#error OUTPUT_BUFFER_SIZE must be in range 16..250
#endif
#endif

