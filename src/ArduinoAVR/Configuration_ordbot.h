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

/* Some words on units:

From 0.80 onwards the units used are unified for easier configuration, watch out when transfering from older configs!

Speed is in mm/s
Acceleration in mm/s^2
Temperature is in degrees celsius


##########################################################################################
##                                        IMPORTANT                                     ##
##########################################################################################

For easy configuration, the default settings enable parameter storage in EEPROM.
This means, after the first upload many variables can only be changed using the special
M commands as described in the documentation. Changing these value sin the configuration.h
has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder
values except thermistor tables and some other parameter likely to change during usage
like advance steps or ops mode.
To override EEPROM settings with config settings, set EEPROM_MODE 0

*/


// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

/** Number of extruders. Maximum 2 extruder. */
#define NUM_EXTRUDER 1

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// Gen3 PLUS for RepRap Motherboard V1.2 = 21
// MEGA/RAMPS up to 1.2       = 3
// RAMPS 1.3/RAMPS 1.4        = 33
// Azteeg X3                  = 34
// Gen6                       = 5
// Gen6 deluxe                = 51
// Sanguinololu up to 1.1     = 6
// Sanguinololu 1.2 and above = 62
// Melzi board                = 63  // Define REPRAPPRO_HUXLEY if you have one for correct HEATER_1_PIN assignment!
// Gen7 1.1 till 1.3.x        = 7
// Gen7 1.4.1 and later       = 71
// Teensylu (at90usb)         = 8 // requires Teensyduino
// Printrboard (at90usb)      = 9 // requires Teensyduino
// Foltyn 3D Master           = 12
// MegaTronics 1.0            = 70
// Megatronics 2.0            = 701
// RUMBA                      = 80  // Get it from reprapdiscount
// Rambo                      = 301
// Arduino Due                = 401 // This is only experimental
// Sanguish Beta              = 501

#define MOTHERBOARD 33

#include "pins.h"

// Uncomment the following line if oyu are using arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not beeing compatible!
//#define COMPAT_PRE1

/* Define the type of axis movements needed for your printer. The typical case
is a full cartesian system where x, y and z moves are handled by seperate motors.

0 = full cartesian system, xyz have seperate motors.
1 = z axis + xy H-gantry (x_motor = x+y, y_motor = x-y)
2 = z axis + xy H-gantry (x_motor = x+y, y_motor = y-x)
3 = Delta printers (Rostock, Kossel, RostockMax, Cerberus, etc)
Cases 1 and 2 cover all needed xy H gantry systems. If you get results mirrored etc. you can swap motor connections for x and y. If a motor turns in
the wrong direction change INVERT_X_DIR or INVERT_Y_DIR.
*/
#define DRIVE_SYSTEM 0

// ##########################################################################################
// ##                               Calibration                                            ##
// ##########################################################################################

/** Drive settings for the Delta printers
*/
#if DRIVE_SYSTEM==3
    // ***************************************************
    // *** These parameter are only for Delta printers ***
    // ***************************************************

    /** \brief Delta drive type: 0 - belts and pulleys, 1 - filament drive */
    #define DELTA_DRIVE_TYPE 0

    #if DELTA_DRIVE_TYPE == 0
      /** \brief Pitch in mm of drive belt. GT2 = 2mm */
      #define BELT_PITCH 2
      /** \brief Number of teeth on X, Y and Z tower pulleys */
      #define PULLEY_TEETH 20
      #define PULLEY_CIRCUMFERENCE (BELT_PITCH * PULLEY_TEETH)
    #elif DELTA_DRIVE_TYPE == 1
      /** \brief Filament pulley diameter in milimeters */
      #define PULLEY_DIAMETER 10
      #define PULLEY_CIRCUMFERENCE (PULLEY_DIAMETER * 3.1415927)
    #endif

    /** \brief Steps per rotation of stepper motor */
    #define STEPS_PER_ROTATION 400

    /** \brief Micro stepping rate of X, Y and Y tower stepper drivers */
    #define MICRO_STEPS 8

    /** \brief Number of delta moves in each line. Moves that exceed this figure will be split into multiple lines.
    Increasing this figure can use a lot of memory since 7 bytes * size of line buffer * MAX_SELTA_SEGMENTS_PER_LINE
    will be allocated for the delta buffer. With defaults 7 * 16 * 30 = 3360 bytes. This leaves ~1K free RAM on an Arduino
    Mega. */
    #define MAX_DELTA_SEGMENTS_PER_LINE 30

    // Calculations
    #define AXIS_STEPS_PER_MM ((float)(MICRO_STEPS * STEPS_PER_ROTATION) / PULLEY_CIRCUMFERENCE)
    #define XAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
    #define YAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
    #define ZAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#else
    // *******************************************************
    // *** These parameter are for all othe rprinter types ***
    // *******************************************************

    /** Drive settings for printers with cartesian drive systems */
    /** \brief Number of steps for a 1mm move in x direction.
    For xy gantry use 2*belt moved!
    Overridden if EEPROM activated. */
    #define XAXIS_STEPS_PER_MM 98.425196
    /** \brief Number of steps for a 1mm move in y direction.
    For xy gantry use 2*belt moved!
    Overridden if EEPROM activated.*/
    #define YAXIS_STEPS_PER_MM 98.425196
    /** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
    #define ZAXIS_STEPS_PER_MM 2560
#endif

// ##########################################################################################
// ##                           Extruder configuration                                     ##
// ##########################################################################################

// for each extruder, fan will stay on until extruder temperature is below this value
#define EXTRUDER_FAN_COOL_TEMP 50

#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
// for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT0_STEPS_PER_MM 413 //385
// What type of sensor is used?
// 1 is 100k thermistor (Epcos B57560G0107F000 - RepRap-Fab.org and many other)
// 2 is 200k thermistor
// 3 is mendel-parts thermistor (EPCOS G550)
// 4 is 10k thermistor
// 8 is ATC Semitec 104GT-2
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 60 is AD8494, AD8495, AD8496 or AD8497 (5mV/°C and 1/4 the price of AD595 but only MSOT_08 package)
// 97 Generic thermistor table 1
// 98 Generic thermistor table 2
// 99 Generic thermistor table 3
// 100 is AD595
// 101 is MAX6675
#define EXT0_TEMPSENSOR_TYPE 1
// Analog input pin for reading temperatures or pin enabling SS for MAX6675
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
// WHich pin enables the heater
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E0_STEP_PIN
#define EXT0_DIR_PIN E0_DIR_PIN
// set to false/true for normal / inverse direction
#define EXT0_INVERSE true
#define EXT0_ENABLE_PIN E0_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT0_ENABLE_ON false
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use hiher values.
//  Overridden if EEPROM activated.
#define EXT0_MAX_FEEDRATE 30
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT0_MAX_START_FEEDRATE 10
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT0_MAX_ACCELERATION 4000
/** Type of heat manager for this extruder.
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better but needs good PID values. Defaults are a good start for most extruder.
 Overridden if EEPROM activated.
*/
#define EXT0_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT0_WATCHPERIOD 1

/** \brief The maximum value, I-gain can contribute to the output.

A good value is slightly higher then the output needed for your temperature.
Values for starts:
130 => PLA for temperatures from 170-180°C
180 => ABS for temperatures around 240°C

The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MAX 140
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MIN 60
/** P-gain.  Overridden if EEPROM activated. */
#define EXT0_PID_P   24
/** I-gain. Overridden if EEPROM activated.
*/
#define EXT0_PID_I   0.88
/** Dgain.  Overridden if EEPROM activated.*/
#define EXT0_PID_D 80
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT0_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
K is the factor for the quadratic term, which is normally disabled in newer versions. If you want to use
the quadratic factor make sure ENABLE_QUADRATIC_ADVANCE is defined.
L is the linear factor and seems to be working better then the quadratic dependency.
*/
#define EXT0_ADVANCE_K 0.0f
#define EXT0_ADVANCE_L 0.0f

/** \brief Temperature to retract filament when extruder is heating up. Overridden if EEPROM activated.
*/
#define EXT0_WAIT_RETRACT_TEMP 		150
/** \brief Units (mm/inches) to retract filament when extruder is heating up. Overridden if EEPROM activated. Set
to 0 to disable.
*/
#define EXT0_WAIT_RETRACT_UNITS 	0

/** You can run any gcode command son extruder deselect/select. Seperate multiple commands with a new line \n.
That way you can execute some mechanical components needed for extruder selection or retract filament or whatever you need.
The codes are only executed for multiple extruder when changing the extruder. */
#define EXT0_SELECT_COMMANDS "M120 S5 P5\nM117 Extruder 1"
#define EXT0_DESELECT_COMMANDS ""
/** The extruder cooler is a fan to cool the extruder when it is heating. If you turn the etxruder on, the fan goes on. */
#define EXT0_EXTRUDER_COOLER_PIN -1
/** PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT0_EXTRUDER_COOLER_SPEED 255


// =========================== Configuration for second extruder ========================
#define EXT1_X_OFFSET 10
#define EXT1_Y_OFFSET 0
// for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT1_STEPS_PER_MM 373
// What type of sensor is used?
// 1 is 100k thermistor (Epcos B57560G0107F000 - RepRap-Fab.org and many other)
// 2 is 200k thermistor
// 3 is mendel-parts thermistor (EPCOS G550)
// 4 is 10k thermistor
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 8 is ATC Semitec 104GT-2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 60 is AD8494, AD8495, AD8496 or AD8497 (5mV/°C and 1/4 the price of AD595 but only MSOT_08 package)
// 97 Generic thermistor table 1
// 98 Generic thermistor table 2
// 99 Generic thermistor table 3
// 100 is AD595
// 101 is MAX6675
#define EXT1_TEMPSENSOR_TYPE 3
// Analog input pin for reading temperatures or pin enabling SS for MAX6675
#define EXT1_TEMPSENSOR_PIN TEMP_2_PIN
// WHich pin enables the heater
#define EXT1_HEATER_PIN HEATER_2_PIN
#define EXT1_STEP_PIN E1_STEP_PIN
#define EXT1_DIR_PIN E1_DIR_PIN
// set to 0/1 for normal / inverse direction
#define EXT1_INVERSE false
#define EXT1_ENABLE_PIN E1_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT1_ENABLE_ON false
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use eiher values.
//  Overridden if EEPROM activated.
#define EXT1_MAX_FEEDRATE 25
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT1_MAX_START_FEEDRATE 12
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT1_MAX_ACCELERATION 10000
/** Type of heat manager for this extruder.
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better but needs good PID values. Defaults are a good start for most extruder.
 Overridden if EEPROM activated.
*/
#define EXT1_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT1_WATCHPERIOD 1

/** \brief The maximum value, I-gain can contribute to the output.

A good value is slightly higher then the output needed for your temperature.
Values for starts:
130 => PLA for temperatures from 170-180°C
180 => ABS for temperatures around 240°C

The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define EXT1_PID_INTEGRAL_DRIVE_MAX 130
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define EXT1_PID_INTEGRAL_DRIVE_MIN 60
/** P-gain.  Overridden if EEPROM activated. */
#define EXT1_PID_P   24
/** I-gain.  Overridden if EEPROM activated.
*/
#define EXT1_PID_I   0.88
/** Dgain.  Overridden if EEPROM activated.*/
#define EXT1_PID_D 200
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT1_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
K is the factor for the quadratic term, which is normally disabled in newer versions. If you want to use
the quadratic factor make sure ENABLE_QUADRATIC_ADVANCE is defined.
L is the linear factor and seems to be working better then the quadratic dependency.
*/
#define EXT1_ADVANCE_K 0.0f
#define EXT1_ADVANCE_L 0.0f

#define EXT1_WAIT_RETRACT_TEMP 	150
#define EXT1_WAIT_RETRACT_UNITS	40
#define EXT1_SELECT_COMMANDS "M120 S5 P15\nM117 Extruder 2"
#define EXT1_DESELECT_COMMANDS ""
/** The extruder cooler is a fan to cool the extruder when it is heating. If you turn the etxruder on, the fan goes on. */
#define EXT1_EXTRUDER_COOLER_PIN -1
/** PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT1_EXTRUDER_COOLER_SPEED 255

/** If enabled you can select the distance your filament gets retracted during a
M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP true

/** PID control only works target temperature +/- PID_CONTROL_RANGE.
If you get much overshoot at the first temperature set, because the heater is going full power to long, you
need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15.
*/
#define PID_CONTROL_RANGE 15

/** Skip wait, if the extruder temperature is already within x degrees. Only fixed numbers, 0 = off */
#define SKIP_M109_IF_WITHIN 2

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

/** Userdefined thermistor table

There are many different thermistors, which can be combined with different resistors. This result
in unpredictable number of tables. As a resolution, the user can define one table here, that can
be used as type 5 for thermister type in extruder/heated bed definition. Make sure, the number of entries
matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
increasing adc values. For more informations, read
http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
This firmware works with increased precision, so the value reads go from 0 to 4095 and the temperature is
temperature*8.

If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use themistor types 50-52 instead of 5-7!
*/
/** Number of entries in the user thermistortable 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0 28
#define USER_THERMISTORTABLE0  {\
  {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},\
  {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},\
  {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	}

/** Number of entries in the user thermistortable 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1  {}
/** Number of entries in the user thermistortable 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2 0
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

If you need the generic table, uncomment the following define.
*/
//#define USE_GENERIC_THERMISTORTABLE_1

/* Some examples for different thermistors:

EPCOS B57560G104+ : R0 = 100000  T0 = 25  Beta = 4036
EPCOS 100K Thermistor (B57560G1104F) :  R0 = 100000  T0 = 25  Beta = 4092
ATC Semitec 104GT-2 : R0 = 100000  T0 = 25  Beta = 4267
Honeywell 100K Thermistor (135-104LAG-J01)  : R0 = 100000  T0 = 25  Beta = 3974

*/

/** Reference resistance */
#define GENERIC_THERM1_R0 100000
/** Temperature at reference resistance */
#define GENERIC_THERM1_T0 25
/** Beta value of thermistor

You can use the beta from the datasheet or compute it yourself. See
http://reprap.org/wiki/MeasuringThermistorBeta
for more details.
*/
#define GENERIC_THERM1_BETA 4036
/** Start temperature for generated thermistor table */
#define GENERIC_THERM1_MIN_TEMP -20
/** End Temperature for generated thermistor table */
#define GENERIC_THERM1_MAX_TEMP 300
#define GENERIC_THERM1_R1 0
#define GENERIC_THERM1_R2 4700

// The same for table 2 and 3 if needed

//#define USE_GENERIC_THERMISTORTABLE_2
#define GENERIC_THERM2_R0 1042.7
#define GENERIC_THERM2_T0 170
#define GENERIC_THERM2_BETA 4036
#define GENERIC_THERM2_MIN_TEMP -20
#define GENERIC_THERM2_MAX_TEMP 300
#define GENERIC_THERM2_R1 0
#define GENERIC_THERM2_R2 4700

//#define USE_GENERIC_THERMISTORTABLE_3
#define GENERIC_THERM3_R0 1042.7
#define GENERIC_THERM3_T0 170
#define GENERIC_THERM3_BETA 4036
#define GENERIC_THERM3_MIN_TEMP -20
#define GENERIC_THERM3_MAX_TEMP 300
#define GENERIC_THERM3_R1 0
#define GENERIC_THERM3_R2 4700

/** Supply voltage to ADC, can be changed be setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VREF 5
/** Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
Value is used for all generic tables created. */
#define GENERIC_THERM_NUM_ENTRIES 33

// uncomment the following line for MAX6675 support.
//#define SUPPORT_MAX6675

// ############# Heated bed configuration ########################

/** \brief Set true if you have a heated bed conected to your board, false if not */
#define HAVE_HEATED_BED true

#define HEATED_BED_MAX_TEMP 115
/** Skip M190 wait, if heated bed is already within x degrees. Fixed numbers only, 0 = off. */
#define SKIP_M190_IF_WITHIN 3

// Select type of your heated bed. It's the same as for EXT0_TEMPSENSOR_TYPE
// set to 0 if you don't have a heated bed
#define HEATED_BED_SENSOR_TYPE 1
/** Analog pin of analog sensor to read temperature of heated bed.  */
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
// How often the temperature of the heated bed is set (msec)
#define HEATED_BED_SET_INTERVAL 5000

/**
Heat manager for heated bed:
0 = Bang Bang, fast update
1 = PID controlled
2 = Bang Bang, limited check every HEATED_BED_SET_INTERVAL. Use this with relais driven beds to save life
*/
#define HEATED_BED_HEAT_MANAGER 1
/** \brief The maximum value, I-gain can contribute to the output.
The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
/** P-gain.  Overridden if EEPROM activated. */
#define HEATED_BED_PID_PGAIN   196
/** I-gain  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_IGAIN   33.02
/** Dgain.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_DGAIN 290
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define HEATED_BED_PID_MAX 255

/** Include PID control for all heaters. */
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
#define MAXTEMP 260

/** Extreme values to detect defect thermistors. */
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300

/** \brief Used reference, normally ANALOG_REF_AVCC or ANALOG_REF_AREF for experts ANALOG_REF_INT_2_56 = 2.56V and ANALOG_REF_INT_1_1=1.1V inernaly generated */
#define ANALOG_REF ANALOG_REF_AVCC


// ##########################################################################################
// ##                            Endstop configuration                                     ##
// ##########################################################################################

/* By default all endstops are pulled up to high. You need a pullup if you
use a mechanical endstop connected with gnd. Set value to false for no pullup
on this endstop.
*/
#define ENDSTOP_PULLUP_X_MIN false
#define ENDSTOP_PULLUP_Y_MIN false
#define ENDSTOP_PULLUP_Z_MIN false
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_PULLUP_Z_MAX false

//set to true to invert the logic of the endstops
#define ENDSTOP_X_MIN_INVERTING true
#define ENDSTOP_Y_MIN_INVERTING true
#define ENDSTOP_Z_MIN_INVERTING true
#define ENDSTOP_X_MAX_INVERTING false
#define ENDSTOP_Y_MAX_INVERTING false
#define ENDSTOP_Z_MAX_INVERTING true

// Set the values true where you have a hardware endstop. The Pin numbe ris taken from pins.h.

#define MIN_HARDWARE_ENDSTOP_X true
#define MIN_HARDWARE_ENDSTOP_Y true
#define MIN_HARDWARE_ENDSTOP_Z false
#define MAX_HARDWARE_ENDSTOP_X false
#define MAX_HARDWARE_ENDSTOP_Y false
#define MAX_HARDWARE_ENDSTOP_Z true

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
#define DISABLE_Z false
#define DISABLE_E false

// Inverting axis direction
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR 1

// Delta robot radius endstop
#define max_software_endstop_r true

//If true, axis won't move to coordinates less than zero.
#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false

//If true, axis won't move to coordinates greater than the defined lengths below.
#define max_software_endstop_x true
#define max_software_endstop_y true
#define max_software_endstop_z false

// If during homing the endstop is reached, ho many mm should the printer move back for the second try
#define ENDSTOP_X_BACK_MOVE 5
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 2

// For higher precision you can reduce the speed for the second test on the endstop
// during homing operation. The homing speed is divided by the value. 1 = same speed, 2 = half speed
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 2
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 2

// When you have several endstops in one circuit you need to disable it after homing by moving a
// small amount back. This is also the case with H-belt systems.
#define ENDSTOP_X_BACK_ON_HOME 2
#define ENDSTOP_Y_BACK_ON_HOME 14
#define ENDSTOP_Z_BACK_ON_HOME 5

// You can disable endstop checking for print moves. This is needed, if you get sometimes
// false signals from your endstops. If your endstops don't give false signals, you
// can set it on for safety.
#define ALWAYS_CHECK_ENDSTOPS true

// maximum positions in mm - only fixed numbers!
// For delta robot Z_MAX_LENGTH is maximum travel of the towers and should be set to the distance between the hotend
// and the platform when the printer is at its home position.
// If EEPROM is enabled these values will be overidden with the values in the EEPROM
#define X_MAX_LENGTH 165
#define Y_MAX_LENGTH 175
#define Z_MAX_LENGTH 80

// Coordinates for the minimum axis. Can also be negative if you want to have the bed start at 0 and the printer can go to the left side
// of the bed. Maximum coordinate is given by adding the above X_MAX_LENGTH values.
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU. Currently only works for RAMBO boards
#define MICROSTEP_MODES {8,8,8,8,8} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#if MOTHERBOARD==301
#define MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
#elif MOTHERBOARD==12
#define MOTOR_CURRENT {35713,35713,35713,35713,35713} // Values 0-65535 (3D Master 35713 = ~1A)
#endif

// Delta settings
#if DRIVE_SYSTEM==3
/** \brief Delta rod length
*/
#define DELTA_DIAGONAL_ROD 250.0 // mm

/** \brief Number of segments to generate for delta conversions per second of move
*/
#define DELTA_SEGMENTS_PER_SECOND_PRINT 200 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70 // Less accurate setting for other moves

/** \brief Horizontal offset of the universal joints on the end effector (moving platform).
*/
#define END_EFFECTOR_HORIZONTAL_OFFSET 33

/** \brief Horizontal offset of the universal joints on the vertical carriages.
*/
#define CARRIAGE_HORIZONTAL_OFFSET 18

/** \brief Printer radius in mm, measured from the center of the print area to the vertical smooth rod.
*/
#define PRINTER_RADIUS 175

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/
#define DELTA_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)

/** \brief Enable counter to count steps for Z max calculations
*/
#define STEP_COUNTER

/** \brief Experimental calibration utility for delta printers
*/
#define SOFTWARE_LEVELING

#endif

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
    The axis order in all axis related arrays is X, Y, Z
     Overridden if EEPROM activated.
    */
#define MAX_FEEDRATE_X 200
#define MAX_FEEDRATE_Y 200
#define MAX_FEEDRATE_Z 5

/** Speed in mm/min for finding the home position.  Overridden if EEPROM activated. */
#define HOMING_FEEDRATE_X 80
#define HOMING_FEEDRATE_Y 80
#define HOMING_FEEDRATE_Z 3

/** Set order of axis homing. Use HOME_ORDER_XYZ and replace XYZ with your order. */
#define HOMING_ORDER HOME_ORDER_ZXY
/* If you have a backlash in both z-directions, you can use this. For most printer, the bed will be pushed down by it's
own weight, so this is nearly never needed. */
#define ENABLE_BACKLASH_COMPENSATION true
#define Z_BACKLASH 0
#define X_BACKLASH 0
#define Y_BACKLASH 0

/** Comment this to disable ramp acceleration */
#define RAMP_ACCELERATION 1

/** If your stepper needs a longer high signal then given, you can add a delay here.
The delay is realized as a simple loop wasting time, which is not available for other
computations. So make it as low as possible. For the most common drivers no delay is needed, as the
included delay is already enough.
*/
#define STEPPER_HIGH_DELAY 0

/** The firmware can only handle 16000Hz interrupt frequency cleanly. If you need higher speeds
a faster solution is needed, and this is to double/quadruple the steps in one interrupt call.
This is like reducing your 1/16th microstepping to 1/8 or 1/4. It is much cheaper then 1 or 3
additional stepper interrupts with all it's overhead. As a result you can go as high as
40000Hz.
*/
#define STEP_DOUBLER_FREQUENCY 12000
/** If you need frequencies off more then 30000 you definitely need to enable this. If you have only 1/8 stepping
enabling this may cause to stall your moves when 20000Hz is reached.
*/
#define ALLOW_QUADSTEPPING true
/** If you reach STEP_DOUBLER_FREQUENCY the firmware will do 2 or 4 steps with nearly no delay. That can be too fast
for some printers causing an early stall.

*/
#define DOUBLE_STEP_DELAY 1 // time in us

/** The firmware supports trajectory smoothing. To acieve this, it divides the stepsize by 2, resulting in
the double computation cost. For slow movements this is not an issue, but for really fast moves this is
too much. The value specified here is the number of clock cycles between a step on the driving axis.
If the interval at full speed is below this value, smoothing is disabled for that line.*/
#define MAX_HALFSTEP_INTERVAL 1999

//// Acceleration settings

/** \brief X, Y, Z max acceleration in mm/s^2 for printing moves or retracts. Make sure your printer can go that high!
 Overridden if EEPROM activated.
*/
#define maxAccelerationMMPerSquareSecond_X 1500
#define maxAccelerationMMPerSquareSecond_Y 1500
#define maxAccelerationMMPerSquareSecond_Z 100

/** \brief X, Y, Z max acceleration in mm/s^2 for travel moves.  Overridden if EEPROM activated.*/
#define maxTravelAccelerationMMPerSquareSecond_X 3000
#define maxTravelAccelerationMMPerSquareSecond_Y 3000
#define maxTravelAccelerationMMPerSquareSecond_Z 100

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
#define MAX_JERK 20.0
#define MAX_ZJERK 0.3

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
#define MOVE_CACHE_LOW 10
/** \brief Cycles per move, if move cache is low.

This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
if you are printing many very short segments at high speed. Higher delays here allow higher values in PATH_PLANNER_CHECK_SEGMENTS.
*/
#define LOW_TICKS_PER_MOVE 250000

// ##########################################################################################
// ##                           Extruder control                                           ##
// ##########################################################################################

/** \brief Prescale factor, timer0 runs at.

All known arduino boards use 64. This value is needed for the extruder timing. */
#define TIMER0_PRESCALE 64

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
#define OPS_RETRACT_DISTANCE 1.5

/** \brief Backslash produced by extruder reversal

If you are using a bowden extruder, you may need some extra distance to push the filament back into the
original place. This is the value you enter here. Unit is mm.
 Overridden if EEPROM activated.
*/
#define OPS_RETRACT_BACKLASH 0.0

/** \brief Enable advance algorithm.

Without a correct adjusted advance algorithm, you get blobs at points, where acceleration changes. The
effect increases with speed and acceleration difference. Using the advance method decreases this effect.
For more informations, read the wiki.
*/
#define USE_ADVANCE

/** \brief enables quadratic component.

Uncomment to allow a quadratic advance dependency. Linear is the dominant value, so no real need
to activate the quadratic term. Only adds lots of computations and storage usage. */
#define ENABLE_QUADRATIC_ADVANCE


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
#define BAUDRATE 115200
//#define BAUDRATE 250000

/**
Some boards like Gen7 have a power on pin, to enable the atx power supply. If this is defined,
the power will be turned on without the need to call M80 if initially started.
*/
#define ENABLE_POWER_ON_STARTUP

/**
If you use an ATX power supply you need the power pin to work non inverting. For some speacial
boards you might need to make it inverting.
*/
#define POWER_INVERTING false
/** What shall the printer do, when it receives an M112 emergency stop signal?
 0 = Disable heaters/motors, wait for ever until someone presses reset.
 1 = restart by resetting the AVR controller. The USB connection will not reset if managed by a different chip!
*/
#define KILL_METHOD 1

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


/**************** duplicate motor driver ***************

If you have an unused extruder stepper free, you could use it to drive the second z motor
instead of driving both with one stepper. The same works for the other axis if needed.
*/

#define FEATURE_TWO_XSTEPPER false
#define X2_STEP_PIN   E1_STEP_PIN
#define X2_DIR_PIN    E1_DIR_PIN
#define X2_ENABLE_PIN E1_ENABLE_PIN

#define FEATURE_TWO_YSTEPPER false
#define Y2_STEP_PIN   E1_STEP_PIN
#define Y2_DIR_PIN    E1_DIR_PIN
#define Y2_ENABLE_PIN E1_ENABLE_PIN

#define FEATURE_TWO_ZSTEPPER false
#define Z2_STEP_PIN   E1_STEP_PIN
#define Z2_DIR_PIN    E1_DIR_PIN
#define Z2_ENABLE_PIN E1_ENABLE_PIN


/* Servos

If you need to control servos, enable this feature. You can control up to 4 servos.
Control the servos with
M340 P<servoId> S<pulseInUS>
servoID = 0..3
Servos are controlled by a pulse with normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.

WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/

#define FEATURE_SERVO false
// Servo pins on a RAMPS board are 11,6,5,4
#define SERVO0_PIN 11
#define SERVO1_PIN 6
#define SERVO2_PIN 5
#define SERVO3_PIN 4

/* Z-Probing */

#define FEATURE_Z_PROBE true
#define Z_PROBE_PIN 63
#define Z_PROBE_PULLUP true
#define Z_PROBE_ON_HIGH true
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
// Waits for a signal to start. Valid signals are probe hit and ok button.
// This is needful if you have the probe trigger by hand.
#define Z_PROBE_WAIT_BEFORE_TEST true
/** Speed of z-axis in mm/s when probing */
#define Z_PROBE_SPEED 2
#define Z_PROBE_XY_SPEED 150
/** The height is the difference between activated probe position and nozzle height. */
#define Z_PROBE_HEIGHT 39.91
/** These scripts are run before resp. after the z-probe is done. Add here code to activate/deactivate probe if needed. */
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""

/* Autoleveling allows it to z-probe 3 points to compute the inclination and compensates the error for the print.
   This feature requires a working z-probe and you should have z-endstop at the top not at the bottom.
   The same 3 points are used for the G29 command.
*/
#define FEATURE_AUTOLEVEL true
#define Z_PROBE_X1 100
#define Z_PROBE_Y1 20
#define Z_PROBE_X2 160
#define Z_PROBE_Y2 170
#define Z_PROBE_X3 20
#define Z_PROBE_Y3 170

/** Set to false to disable SD support: */
#ifndef SDSUPPORT  // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT false
/** If set to false all files with longer names then 8.3 or having a tilde in the name will be hidden */
#define SD_ALLOW_LONG_NAMES false
// Uncomment to enable or changed card detection pin. With card detection the card is mounted on insertion.
#define SDCARDDETECT -1
// Change to true if you get a inserted message on removal.
#define SDCARDDETECTINVERTED false
#endif
/** Show extended directory including file length. Don't use this with pronterface! */
#define SD_EXTENDED_DIR
// If you want support for G2/G3 arc commands set to true, otherwise false.
#define ARC_SUPPORT true

/** You can store the current position with M401 and go back to it with M402.
   This works only if feature is set to true. */
#define FEATURE_MEMORY_POSITION true

/** If a checksum is send, all future comamnds must also contain a checksum. Increases reliability especially for binary protocol. */
#define FEATURE_CHECKSUM_FORCED false

/** Should support for fan control be compiled in. If you enable this make sure
the FAN pin is not the same as for your second extruder. RAMPS e.g. has FAN_PIN in 9 which
is also used for the heater if you have 2 extruders connected. */
#define FEATURE_FAN_CONTROL true

/** For displays and keys there are too many permutations to handle them all in once.
For the most common available combinations you can set the controller type here, so
you don't need to configure uicong.h at all. Controller settings > 1 disable usage
of uiconfig.h

0 = no display
1 = Manual definition of display and keys parameter in uiconfig.h

The following settings override uiconfig.h!
2 = Smartcontroller from reprapdiscount on a RAMPS or RUMBA board
3 = Adafruit RGB controller
4 = Foltyn 3DMaster with display attached
5 = ViKi LCD - Check pin configuration in ui.h for feature controller 5!!!
*/
#define FEATURE_CONTROLLER 2

/**
Select the language to use.
0 = english
1 = german
2 = dutch
3 = brazilian portuguese
4 = italian
*/
#define UI_LANGUAGE 1

// This is line 2 of the status display at startup. Change to your like.
#define UI_VERSION_STRING2 "Ordbot"

/** How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION 4000

/** Uncomment if you don't want automatic page switching. You can still switch the
info pages with next/previous button/click-encoder */
#define UI_DISABLE_AUTO_PAGESWITCH true

/** Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_AUTORETURN_TO_MENU_AFTER 30000

#define FEATURE_UI_KEYS 0

/* Normally cou want a next/previous actions with every click of your encoder.
Unfotunately, the encoder have a different count of phase changes between clicks.
Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click.
*/
#define UI_ENCODER_SPEED 1
/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME 10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT 500
/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT 50
/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT 50

#define FEATURE_BEEPER true
/**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 2,2
#define BEEPER_LONG_SEQUENCE 8,8

// ###############################################################################
// ##                         Values for menu settings                          ##
// ###############################################################################

// Values used for preheat
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA 60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA   180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS 110
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS   240
// Extreme values
#define UI_SET_MIN_HEATED_BED_TEMP  55
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP   160
#define UI_SET_MAX_EXTRUDER_TEMP   270
#define UI_SET_EXTRUDER_FEEDRATE 2 // mm/sec
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3 // mm

#endif

