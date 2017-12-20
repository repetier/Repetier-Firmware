/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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


// ##########################################################################################
// ##	IMPORTANT
// ##########################################################################################

/* Some words on units:
Speed			is in	mm/s
Acceleration	in in	mm/s^2
Temperature		is in	degrees celsius

For easy configuration, the default settings enable parameter storage in EEPROM.
This means, after the first upload many variables can only be changed using the special
M commands as described in the documentation. Changing these values in the configuration.h
file has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder
values except thermistor tables and some other parameter likely to change during usage
like advance steps or ops mode.
To override EEPROM settings with config settings, set EEPROM_MODE 0 */


// ##########################################################################################
// ##	main hardware configuration
// ##########################################################################################

/** \brief Define the type of your device */
#define MOTHERBOARD							DEVICE_TYPE_RF2000
#define PROTOTYPE_PCB						0													// 1 = first PCB's / 0 = Final

/** \brief EEPROM storage mode
Set the EEPROM_MODE to 0 if you always want to use the settings in this configuration file. If not,
set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
EEPROM settings with configuration defaults, just select an other value. On the first call to epr_init()
it will detect a mismatch of the first byte and copy default values into EEPROM. If the first byte
matches, the stored values are used to overwrite the settings.

IMPORTANT: With mode <>0 some changes in Configuration.h are not set any more, as they are
           taken from the EEPROM. */
#define EEPROM_MODE							138


// ##########################################################################################
// ##    supported features
// ##########################################################################################

/** \brief Allows to use 6 additional hardware buttons */
#define FEATURE_EXTENDED_BUTTONS			1													// 1 = on, 0 = off

/** \brief Enables automatic compensation in z direction for the operationg mode "print" */
#define FEATURE_HEAT_BED_Z_COMPENSATION		1													// 1 = on, 0 = off

/** \brief Enables the precise heat bed scan */
#if FEATURE_HEAT_BED_Z_COMPENSATION

#define FEATURE_PRECISE_HEAT_BED_SCAN		1													// 1 = on, 0 = off

#endif // FEATURE_HEAT_BED_Z_COMPENSATION

/** \brief Allows/disallows to override Z-min via G0 and G1 */
#define FEATURE_Z_MIN_OVERRIDE_VIA_GCODE	1													// 1 = on, 0 = off

/** \brief Enables/disables the output of the finished object feature */
#define FEATURE_OUTPUT_FINISHED_OBJECT		1													// 1 = on, 0 = off

/** \brief Allows to pause the processing of G-Codes */
#define FEATURE_PAUSE_PRINTING				1													// 1 = on, 0 = off

/** \brief Enables/diables the emergency pause in case of too high pressure ... the emergency pause can be turned on only in case the general pause functionality is available */
#if FEATURE_PAUSE_PRINTING

#define FEATURE_EMERGENCY_PAUSE				1													// 1 = on, 0 = off

#endif // FEATURE_PAUSE_PRINTING

/** \brief Allows to cause an emergency stop via a 3-times push of the pause button */
#define FEATURE_EMERGENCY_STOP_VIA_PAUSE	0													// 1 = on, 0 = off

/** \brief Enables/disables the emergency stop in case of too high pressure */
#define FEATURE_EMERGENCY_STOP_ALL			1													// 1 = on, 0 = off

/** \brief Specifies whether the current print shall be aborted in case a temperature sensor is defect */
#define FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR		1										// 1 = abort, 0 = do not abort

/** \brief Enables/disables the set to x/y origin feature */
#define FEATURE_SET_TO_XY_ORIGIN			1													// 1 = on, 0 = off

/** \brief Enables/disables the park feature */
#define FEATURE_PARK						0													// 1 = on, 0 = off

/** \brief Enables safety checks for the manual moving into z-direction via the additional hardware buttons */
#define FEATURE_ENABLE_Z_SAFETY				1													// 1 = checks enabled, 0 = checks disabled

/** \brief Enables/disables the reset via the printer menu */
#define	FEATURE_RESET_VIA_MENU				1													// 1 = on, 0 = off

/** \brief Specifies whether the x, y and z-positions can be changed manually (e.g. via the "Position X/Y/Z" menus or via the hardware buttons) in case the according axis is unknown.
The position of an axis is unknown until the axis has been homed. The position of an axis becomes unknown in case its stepper is disabled.
Enabling of the following feature can be dangerous because it allows to manually drive the printer above its max x/y/z position. */
#define	FEATURE_ALLOW_UNKNOWN_POSITIONS		1													// 1 = allow, 0 = do not allow

/** \brief Ditto printing allows 2 extruders to do the same action. This effectively allows
to print an object two times at the speed of one. Works only with dual extruder setup. */
#define FEATURE_DITTO_PRINTING				0													// 1 = on, 0 = off

/** \brief You can store the current position with M401 and go back to it with M402. This works only if feature is set to true. */
#define FEATURE_MEMORY_POSITION				1													// 1 = on, 0 = off

/** \brief If a checksum is sent, all future comamnds must also contain a checksum. Increases reliability especially for binary protocol. */
#define FEATURE_CHECKSUM_FORCED				0													// 1 = on, 0 = off

/** \brief Enables/disables the support for the fan control */
#define FEATURE_FAN_CONTROL					1													// 1 = on, 0 = off

/** \brief Enables/disables the support for G2/G3 arc commands */
#define FEATURE_ARC_SUPPORT					1													// 1 = on, 0 = off

/** \brief Enables/disables the beeper */
#define FEATURE_BEEPER						1													// 1 = on, 0 = off

/** \brief A watchdog resets the printer, if a signal is not send within predifined time limits. That way we can be sure that the board
is always running and is not hung up for some unknown reason. */
#define FEATURE_WATCHDOG					1													// 1 = on, 0 = off

/** \brief Enables/disables the menu entry which allows to choose the currently installed hotend type */
#define FEATURE_CONFIGURABLE_HOTEND_TYPE	1													// 1 = on, 0 = off

/** \brief Defines whether the complete EEPROM shall be reset and filled with the values from Configuration.h whenever the EEPROM becomes corruped or its EEPROM_MODE is different to the value from Configuration.h. */
#define	FEATURE_FULL_EEPROM_RESET			1													// 1 = on, 0 = off

/** \brief Defines whether a change within the menu shall be stored to the EEPROM automatically or not. */
#define FEATURE_AUTOMATIC_EEPROM_UPDATE		1													// 1 = the EEPROM is updated automatically after each change via the menu, 0 = the EEPROM must be updated manually via the "Store to EEPROM" menu item

/** \brief Allows to use the service interval */
#define	FEATURE_SERVICE_INTERVAL			0													// 1 = on, 0 = off

/** \brief Allows to use the case light */
#define FEATURE_CASE_LIGHT					0													// 1 = on, 0 = off

/** \brief Allows to control up to 3 servos
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware! */
#define FEATURE_SERVO						1													// 1 = on, 0 = off

/** \brief Allows to choose whether pressing of the right menu button shall enter the file menu directly */
#define FEATURE_RIGHT_BUTTON_MENU			0													// 1 = on, 0 = off


// ##########################################################################################
// ##	common configuration
// ##########################################################################################

/** \brief Define the to-be-used micro steps */
#define	RF_MICRO_STEPS						32												


// ##########################################################################################
// ##	debugging
// ##########################################################################################

/** \brief Enables debug outputs which are used mainly for the development */
#define DEBUG_SHOW_DEVELOPMENT_LOGS			0													// 1 = on, 0 = off


#if FEATURE_HEAT_BED_Z_COMPENSATION 

/** \brief Enables debug outputs from the compensation in z direction */
#define DEBUG_HEAT_BED_Z_COMPENSATION		0													// 1 = on, 0 = off

/** \brief Enables debug outputs from the heat bed scan */
#define DEBUG_HEAT_BED_SCAN					0													// 0 = off, 1 = on, 2 = on with more debug outputs

#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

/** \brief Enables debug outputs about the pressure which is detected during the heat bed or work part scan */
#define DEBUG_REMEMBER_SCAN_PRESSURE		0													// 1 = on, 0 = off

#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION


/** \brief Enables debug outputs about the processing of commands */
#define	DEBUG_COMMAND_PEEK					0													// 1 = on, 0 = off

/** \brief Enables debug outputs about the processing of queue moves */
#define DEBUG_QUEUE_MOVE					0													// 1 = on, 0 = off

/** \brief Enables debug outputs about the processing of direct moves */
#define DEBUG_DIRECT_MOVE					0													// 1 = on, 0 = off

/** \brief Enables debug outputs about the heat bed temperature compensation */
#define	DEBUG_HEAT_BED_TEMP_COMPENSATION	0													// 1 = on, 0 = off

/** \brief Enables debug outputs about the configurable z endstops */
#define DEBUG_CONFIGURABLE_Z_ENDSTOPS		0													// 1 = on, 0 = off

/** \brief Allows M111 to set bit 5 (16) which disables all commands except M111. This can be used
to test your data througput or search for communication problems. */
#define INCLUDE_DEBUG_COMMUNICATION

/** \brief Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE

/** \brief Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for seraching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE

#ifdef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY						Commands::checkFreeMemory();
#else
#define DEBUG_MEMORY
#endif // DEBUG_FREE_MEMORY

/** \brief If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC

/** \brief If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT

/** \brief This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
#define DEBUG_COM_ERRORS

/** \brief Adds a menu point in quick settings to write debg informations to the host in case of hangs where the ui still works. */
//#define DEBUG_PRINT
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
#define ANALYZER_CH0						63 // New move
#define ANALYZER_CH1						40 // Step loop
#define ANALYZER_CH2						53 // X Step
#define ANALYZER_CH3						65 // Y Step
#define ANALYZER_CH4						59 // X Direction
#define ANALYZER_CH5						64 // Y Direction
#define ANALYZER_CH6						58 // xsig
#define ANALYZER_CH7						57 // ysig

#ifdef ANALYZER

#define ANALYZER_ON(a)						{WRITE(a,HIGH);}
#define ANALYZER_OFF(a)						{WRITE(a,LOW);}

#else

#define ANALYZER_ON(a)
#define ANALYZER_OFF(a)

#endif // ANALYZER


// ##########################################################################################
// ##	configuration of the extended buttons
// ##########################################################################################

#if FEATURE_EXTENDED_BUTTONS

#define EXTENDED_BUTTONS_COUNTER_NORMAL		4													// 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_COUNTER_FAST		4													// 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define	EXTENDED_BUTTONS_STEPPER_DELAY		1													// [us]
#define	EXTENDED_BUTTONS_Z_MIN				-(ZAXIS_STEPS_PER_MM *2)							// [steps]
#define	EXTENDED_BUTTONS_Z_MAX				long(ZAXIS_STEPS_PER_MM * (Z_MAX_LENGTH -2))		// [steps]

#endif // FEATURE_EXTENDED_BUTTONS


// ##########################################################################################
// ##	configuration of the output object functionality
// ##########################################################################################

#if FEATURE_OUTPUT_FINISHED_OBJECT

/** \brief The following script allows to configure the exact behavior of the automatic object output */
#define	OUTPUT_OBJECT_SCRIPT_PRINT			"G21\nG91\nG1 E-10\nG1 Z210 F5000\nG1 Y250 F7500"
#define	OUTPUT_OBJECT_SCRIPT_MILL			"G28 Z0\nG21\nG91\nG1 Y250 F7500"

#endif // FEATURE_OUTPUT_FINISHED_OBJECT


// ##########################################################################################
// ##	configuration of the pause functionality
// ##########################################################################################

#if FEATURE_PAUSE_PRINTING 

#if !FEATURE_HEAT_BED_Z_COMPENSATION && !FEATURE_WORK_PART_Z_COMPENSATION
	#error FEATURE_PAUSE_PRINTING can not be used without FEATURE_HEAT_BED_Z_COMPENSATION or FEATURE_WORK_PART_Z_COMPENSATION
#endif // !FEATURE_HEAT_BED_Z_COMPENSATION && !FEATURE_WORK_PART_Z_COMPENSATION

/** \brief Specifies the time interval after the pausing of the print at which the extruder current is reduced */
#define EXTRUDER_CURRENT_PAUSE_DELAY		5000												// [ms] or 0, in order to disable the lowering of the extruder current

/** \brief Specifies the extruder current which shall be use after pausing of the print and before continuing of the print */
#define	EXTRUDER_CURRENT_PAUSED				32													// ~0.5A

#endif // FEATURE_PAUSE_PRINTING


// ##########################################################################################
// ##	configuration of the park functionality
// ##########################################################################################

#if FEATURE_PARK

/** \brief Specifies the park position, in [mm] */
#define PARK_POSITION_X						0
#define	PARK_POSITION_Y						120
#define	PARK_POSITION_Z						175

#endif // FEATURE_PARK


// ##########################################################################################
// ##	configuration of the emergency pause functionality
// ##########################################################################################

#if FEATURE_EMERGENCY_PAUSE

/** \brief Specifies the pressure at which the emergency pause shall be performed, in [digits] */
#define EMERGENCY_PAUSE_DIGITS_MIN			-15000
#define EMERGENCY_PAUSE_DIGITS_MAX			15000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define	EMERGENCY_PAUSE_INTERVAL			100

/** \brief Specifies the number of pressure values which shall be averaged. The emergency pause can be detected each EMERGENCY_PAUSE_INTERVAL * EMERGENCY_PAUSE_CHECKS [ms] */
#define	EMERGENCY_PAUSE_CHECKS				10

#endif // FEATURE_EMERGENCY_PAUSE


// ##########################################################################################
// ##	configuration of the emergency z stop functionality
// ##########################################################################################

#if FEATURE_EMERGENCY_STOP_ALL

/** \brief Specifies the pressure at which the emergency z-stop shall be performed, in [digits] */
#define EMERGENCY_STOP_DIGITS_MIN			-5000
#define EMERGENCY_STOP_DIGITS_MAX			5000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms] */
#define	EMERGENCY_STOP_INTERVAL				10

/** \brief Specifies the number of pressure values which shall be averaged. The emergency stop can be detected each EMERGENCY_STOP_INTERVAL * EMERGENCY_STOP_CHECKS [ms] */
#define	EMERGENCY_STOP_CHECKS				3

#endif // FEATURE_EMERGENCY_STOP_ALL


// ##########################################################################################
// ##	configuration of the service intervall
// ##########################################################################################

#if FEATURE_SERVICE_INTERVAL

/** \brief Specifies the max printed hours [h] */
#define HOURS_PRINTED_UNTIL_SERVICE			100

/** \brief Specifies the max milling hours [h] */
#define HOURS_MILLED_UNTIL_SERVICE			100

/** \brief Specifies the max printed filament [m] */
#define FILAMENT_PRINTED_UNTIL_SERVICE		50000

#endif // FEATURE_SERVICE_INTERVAL


// ##########################################################################################
// ##	configuration of the stepper drivers
// ##########################################################################################

/** \brief Specifies whether the firmware shall wait a short time after turning on of the stepper motors - this shall avoid that the first steps are sent to the stepper before it is ready */
#define	STEPPER_ON_DELAY					25													// [ms]

#define DRV8711_NUM_CHANNELS				5

#if RF_MICRO_STEPS == 4
	#define DRV8711_REGISTER_00				0x0E11												// 0000 1110 0001 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0010, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF_MICRO_STEPS == 8
	#define DRV8711_REGISTER_00				0x0E19												// 0000 1110 0001 1001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0011, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF_MICRO_STEPS == 16
	#define DRV8711_REGISTER_00				0x0E21												// 0000 1110 0010 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0100, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF_MICRO_STEPS == 32
	#define DRV8711_REGISTER_00				0x0E29												// 0000 1110 0010 1001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0101, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF_MICRO_STEPS == 64
	#define DRV8711_REGISTER_00				0x0E31												// 0000 1110 0011 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0110, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#else
	#error this number of micro steps is not supported
#endif // RF_MICRO_STEPS

#define DRV8711_REGISTER_02					0x2097												// 0010 0000 1001 0111: TOFF = 10010111, PWMMODE = 0
#define DRV8711_REGISTER_03					0x31D7												// 0011 0001 1101 0111: TBLANK = 11010111, ABT = 1
#define DRV8711_REGISTER_04					0x4430												// 0100 0100 0011 0000: TDECAY = 00110000, DECMOD = 100
#define DRV8711_REGISTER_05					0x583C												// 0101 1000 0011 1100: SDTHR = 00111100, SDCNT = 00, VDIV = 10
#define DRV8711_REGISTER_06					0x60F0												// 0110 0000 1111 0000: OCPTH = 00, OCPDEG = 00, TDRIVEN = 11, TDRIVEP = 11, IDRIVEN = 00, IDRIVEP = 00
#define DRV8711_REGISTER_07					0x7000												// 0111 0000 0000 0000: OTS = 0, AOCP = 0, BOCP = 0, UVLO = 0, APDF = 0, BPDF = 0, STD = 0, STDLAT = 0


// ##########################################################################################
// ##	configuration of user-defined thermistor tables
// ##########################################################################################
/*
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
/** \brief Number of entries in the user thermistor table 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0			28
#define USER_THERMISTORTABLE0  {\
  {1*4,864*8},{21*4,280*8},{25*4,270*8},{29*4,260*8},{33*4,250*8},{39*4,240*8},{46*4,230*8},{54*4,220*8},{64*4,210*8},{75*4,200*8},\
  {90*4,190*8},{107*4,180*8},{128*4,170*8},{154*4,160*8},{184*4,150*8},{221*4,140*8},{265*4,130*8},{316*4,120*8},{375*4,110*8},\
  {441*4,105*8},{513*4,100*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	}

/** \brief Number of entries in the user thermistor table 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1			0
#define USER_THERMISTORTABLE1				{}

/** \brief Number of entries in the user thermistor table 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2			0
#define USER_THERMISTORTABLE2				{}

/** \brief If defined, creates a thermistor table at startup.

If you don't feel like computing the table on your own, you can use this generic method. It is
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
The capacitor is for reducing noise from long thermistor cable. If you don't have one, it's OK.

If you need the generic table, uncomment the following define.
*/
//#define USE_GENERIC_THERMISTORTABLE_1

/* Some examples for different thermistors:

EPCOS B57560G104+ : R0 = 100000  T0 = 25  Beta = 4036
EPCOS 100K Thermistor (B57560G1104F) :  R0 = 100000  T0 = 25  Beta = 4092
ATC Semitec 104GT-2 : R0 = 100000  T0 = 25  Beta = 4267
Honeywell 100K Thermistor (135-104LAG-J01)  : R0 = 100000  T0 = 25  Beta = 3974

*/

/** \brief Reference Temperature */
#define GENERIC_THERM1_T0					25

/** \brief Resistance at reference temperature */
#define GENERIC_THERM1_R0					200000

/** \brief Beta value of thermistor

You can use the beta from the datasheet or compute it yourself.
See http://reprap.org/wiki/MeasuringThermistorBeta for more details.
*/
#define GENERIC_THERM1_BETA					8304

/** \brief Start temperature for generated thermistor table */
#define GENERIC_THERM1_MIN_TEMP				-20

/** \brief End Temperature for generated thermistor table */
#define GENERIC_THERM1_MAX_TEMP				300
#define GENERIC_THERM1_R1					0
#define GENERIC_THERM1_R2					4700

// The same for table 2 and 3 if needed

/** \brief USE_GENERIC_THERMISTORTABLE_2 */
#define GENERIC_THERM2_T0					170
#define GENERIC_THERM2_R0					1042.7
#define GENERIC_THERM2_BETA					4036
#define GENERIC_THERM2_MIN_TEMP				-20
#define GENERIC_THERM2_MAX_TEMP				300
#define GENERIC_THERM2_R1					0
#define GENERIC_THERM2_R2					4700

/** \brief USE_GENERIC_THERMISTORTABLE_3 */
#define GENERIC_THERM3_T0					170
#define GENERIC_THERM3_R0					1042.7
#define GENERIC_THERM3_BETA					4036
#define GENERIC_THERM3_MIN_TEMP				-20
#define GENERIC_THERM3_MAX_TEMP				300
#define GENERIC_THERM3_R1					0
#define GENERIC_THERM3_R2					4700

/** \brief Supply voltage to ADC, can be changed by setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VREF					5

/** \brief Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
Value is used for all generic tables created. */
#define GENERIC_THERM_NUM_ENTRIES			33


// ##########################################################################################
// ##	duplicate motor drivers
// ##########################################################################################

/** \brief If you have an unused extruder stepper free, you could use it to drive the second z motor
instead of driving both with a single stepper. The same works for the other axis if needed. */

#define FEATURE_TWO_XSTEPPER				false
#define X2_STEP_PIN							E1_STEP_PIN
#define X2_DIR_PIN							E1_DIR_PIN
#define X2_ENABLE_PIN						E1_ENABLE_PIN

#define FEATURE_TWO_YSTEPPER				false
#define Y2_STEP_PIN							E1_STEP_PIN
#define Y2_DIR_PIN							E1_DIR_PIN
#define Y2_ENABLE_PIN						E1_ENABLE_PIN

#define FEATURE_TWO_ZSTEPPER				false
#define Z2_STEP_PIN							E1_STEP_PIN
#define Z2_DIR_PIN							E1_DIR_PIN
#define Z2_ENABLE_PIN						E1_ENABLE_PIN


// ##########################################################################################
// ##	configure the SD Card
// ##########################################################################################

/** \brief  Select whether the SD card is supported. */
#define SDSUPPORT							true

/** \brief  Change to true if you get a inserted message on removal. */
#define SDCARDDETECTINVERTED				false

/** \brief Show extended directory including file length. Don't use this with Pronterface! */
#define SD_EXTENDED_DIR						true

#define MAX_VFAT_ENTRIES					(2)

/** \brief Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH				(13*MAX_VFAT_ENTRIES+1)
#define SD_MAX_FOLDER_DEPTH					2


// ##########################################################################################
// ##	configuration of the manual steps
// ##########################################################################################

#if FEATURE_EXTENDED_BUTTONS

/** \brief Configuration of the manual steps */
#define DEFAULT_MANUAL_STEPS_X				(RF_MICRO_STEPS *2)
#define MAXIMAL_MANUAL_STEPS_X				(XAXIS_STEPS_PER_MM *10)
#define DEFAULT_MANUAL_STEPS_Y				(RF_MICRO_STEPS *2)
#define MAXIMAL_MANUAL_STEPS_Y				(YAXIS_STEPS_PER_MM *10)
#define DEFAULT_MANUAL_STEPS_Z				(RF_MICRO_STEPS *2)
#define MAXIMAL_MANUAL_STEPS_Z				(ZAXIS_STEPS_PER_MM *10)
#define DEFAULT_MANUAL_STEPS_E				(EXT0_STEPS_PER_MM /5)
#define MAXIMAL_MANUAL_STEPS_E				(EXT0_STEPS_PER_MM *10)

#endif // FEATURE_EXTENDED_BUTTONS


// ###############################################################################
// ##	Values for menu settings
// ###############################################################################

/** \brief 
Select the language to use.
0 = English
1 = German */
#define UI_LANGUAGE							0

/** \brief Animate switches between menus etc. */
#define UI_ANIMATION						false

/** \brief How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION					4000

/** \brief Delay of start screen in milliseconds */
#define UI_START_SCREEN_DELAY				1000

/** \brief Uncomment if you don't want automatic page switching. You can still switch the
info pages with next/previous button/click-encoder */
#define UI_DISABLE_AUTO_PAGESWITCH			true

/** \brief Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_PRINT_AUTORETURN_TO_MENU_AFTER	30000
#define UI_MILL_AUTORETURN_TO_MENU_AFTER	0

/** \brief Normally cou want a next/previous actions with every click of your encoder.
Unfotunately, the encoder have a different count of phase changes between clicks.
Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click. */
#define UI_ENCODER_SPEED					1

/** \brief There are 2 ways to change positions. You can move by increments of 1/0.1 mm resulting in more menu entries
and requiring many turns on your encode. The alternative is to enable speed dependent positioning. It will change
the move distance depending on the speed you turn the encoder. That way you can move very fast and very slow in the
same setting. */
#define UI_SPEEDDEPENDENT_POSITIONING		true

/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME					10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT					500

/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT				50

/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT					50

/** \brief Default beeper mode. */
#define BEEPER_MODE							1													// 1 = on, 0 = off

/** \brief Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the second is the number of repetitions.
Values must be in range 1..255 */
#define BEEPER_SHORT_SEQUENCE					2,2
#define BEEPER_LONG_SEQUENCE					8,8
#define BEEPER_START_PRINTING_SEQUENCE			100,2
#define BEEPER_ABORT_PRINTING_SEQUENCE			250,5
#define BEEPER_STOP_PRINTING_SEQUENCE			100,3
#define BEEPER_PAUSE_SEQUENCE					50,3
#define BEEPER_CONTINUE_SEQUENCE				50,2
#define BEEPER_START_HEAT_BED_SCAN_SEQUENCE		100,2
#define BEEPER_ABORT_HEAT_BED_SCAN_SEQUENCE		250,5
#define BEEPER_STOP_HEAT_BED_SCAN_SEQUENCE		100,3
#define BEEPER_START_WORK_PART_SCAN_SEQUENCE	100,2
#define BEEPER_ABORT_WORK_PART_SCAN_SEQUENCE	250,5
#define BEEPER_STOP_WORK_PART_SCAN_SEQUENCE		100,3
#define	BEEPER_ABORT_SET_POSITION_SEQUENCE		250,5
#define	BEEPER_ACCEPT_SET_POSITION_SEQUENCE		100,2
#define	BEEPER_SERVICE_INTERVALL_SEQUNCE		100,3
#define	BEEPER_ALIGN_EXTRUDERS_SEQUNCE			50,5
#define BEEPER_WRONG_FIRMWARE_SEQUNCE			1000,4

/** \brief Values used for preheat */
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA	60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA		180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS	110
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS		240

/** \brief Values used for unload(unmount)/load(mount) filament */
#define UI_SET_EXTRUDER_TEMP_UNMOUNT		120
#define	UI_SET_EXTRUDER_TEMP_MOUNT			200

/** \brief Extreme values */
#define UI_SET_MIN_HEATED_BED_TEMP			55
#define UI_SET_MAX_HEATED_BED_TEMP			160
#define UI_SET_MIN_EXTRUDER_TEMP			120
#define UI_SET_MAX_EXTRUDER_TEMP			270
#define UI_SET_EXTRUDER_FEEDRATE			2													// [mm/sec]
#define UI_SET_EXTRUDER_RETRACT_DISTANCE	3													// [mm]
#define COOLDOWN_THRESHOLD					40													// [°C]

#define	SHOW_DEBUGGING_MENU					0													// 1 = show, 0 = hide

#define SPEED_MIN_MILLIS					300
#define SPEED_MAX_MILLIS					50
#define SPEED_MAGNIFICATION					100.0f


// ##########################################################################################
// ##	general external EEPROM configuration
// ##########################################################################################

#define EEPROM_DELAY						10													// [ms]


// ##########################################################################################
// ##	external EEPROM which is used for the z-compensation (32.768 bytes)
// ##########################################################################################
/*
we use blocks of 2 kByte size for the structure of our EEPROM

00000 ... 01535 bytes [EEPROM_SECTOR_SIZE Bytes] = general information
  00000 [2 bytes] EEPROM format
  00002 [2 bytes] active work part z-compensation matrix (1 ... EEPROM_MAX_WORK_PART_SECTORS)
  00004 [2 bytes] active heat bed z-compensation matrix (1 ... EEPROM_MAX_HEAT_BED_SECTORS)

01536 ... 03071 bytes [EEPROM_SECTOR_SIZE Bytes] = heat bed z-compensation matrix 1
  01536 [2 bytes] x-dimension of the heat bed z-compensation matrix 1
  01538 [2 bytes] y-dimension of the heat bed z-compensation matrix 1
  01540 [2 bytes] used micro steps
  01542 [12 bytes] reserved
  01554 [x bytes] heat bed z-compensation matrix 1

03072 ... 04597 bytes [EEPROM_SECTOR_SIZE Bytes] = heat bed z-compensation matrix 2
  ...

15360 ... 16895 bytes [EEPROM_SECTOR_SIZE Bytes]  = work part compensation matrix 1
  15360 [2 bytes] x-dimension of the work part compensation matrix 1
  15362 [2 bytes] y-dimension of the work part compensation matrix 1
  15364 [2 bytes] used micro steps
  15366 [2 bytes] x start position of the work part scan [mm]
  15368 [2 bytes] y start position of the work part scan [mm]
  15370 [2 bytes] x step size of the work part scan [mm]
  15372 [2 bytes] y step size of the work part scan [mm]
  15374 [2 bytes] x end position of the work part scan [mm]
  15376 [2 bytes] y end position of the work part scan [mm]
  15378 [x bytes] work part z-compensation matrix 1

16895 ... 18430 bytes [2 kBytes]  = work part compensation matrix 2
  ...
*/

#define	EEPROM_OFFSET_SECTOR_FORMAT					0
#define EEPROM_OFFSET_DIMENSION_X					2
#define EEPROM_OFFSET_DIMENSION_Y					4
#define	EEPROM_OFFSET_MICRO_STEPS					6
#define EEPROM_OFFSET_X_START_MM					16
#define EEPROM_OFFSET_Y_START_MM					18
#define EEPROM_OFFSET_X_STEP_MM						20
#define EEPROM_OFFSET_Y_STEP_MM						22
#define EEPROM_OFFSET_X_END_MM						24
#define EEPROM_OFFSET_Y_END_MM						26
#define EEPROM_OFFSET_MAXTRIX_START					28

#define	EEPROM_SECTOR_SIZE							1536										// [bytes]
#define	EEPROM_MAX_WORK_PART_SECTORS				9
#define	EEPROM_MAX_HEAT_BED_SECTORS					9

#define EEPROM_OFFSET_HEADER_FORMAT					0											// [bytes]
#define EEPROM_OFFSET_ACTIVE_WORK_PART_Z_MATRIX		2											// [bytes]
#define EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX		4											// [bytes]

#define EEPROM_FORMAT								7


// ##########################################################################################
// ##	external EEPROM which is used for the type information (32.768 bytes)
// ##########################################################################################
/*
we use blocks of 2 kByte size for the structure of our EEPROM

00000 ... 02047 bytes [2 kBytes] = general information
  00000 [2 bytes] EEPROM format
  00002 [2 bytes] board type
*/

#define TYPE_EEPROM_OFFSET_HEADER_FORMAT	0													// [bytes]
#define TYPE_EEPROM_OFFSET_BOARD_TYPE		2													// [bytes]

#define TYPE_EEPROM_FORMAT					1


// ##########################################################################################
// ##	miscellaneous configurations
// ##########################################################################################

#if FEATURE_ENABLE_Z_SAFETY

/** \brief Specifies the maximal steps which can be moved into z-direction after the z-endstop has been reached */
#define	Z_OVERRIDE_MAX						(ZAXIS_STEPS_PER_MM / 2)

#endif // FEATURE_ENABLE_Z_SAFETY

/** \brief Specifies the minimal distance from z-min which must be reached before it is plausible that z-max is hit */
#define	Z_MIN_DISTANCE						(ZAXIS_STEPS_PER_MM * 5)

/** \brief Defines the Z-Offset stepsize in um */
#define	Z_OFFSET_STEP						25

/** \brief Specifies the interval which is used for the manual moves (from the hardware buttons and from the pause functionality) */
#define MANUAL_MOVE_INTERVAL				1													// [ms]

/** \brief Defines the I2C address for the strain gauge */
#define	I2C_ADDRESS_STRAIN_GAUGE			0x49

/** \brief Defines which strain gauge is used for the heat bed scan */
#define	ACTIVE_STRAIN_GAUGE					0x49

/** \brief Defines the I2C address for the external EEPROM which stores the z-compensation matrix */
#define	I2C_ADDRESS_EXTERNAL_EEPROM			0x50

/** \brief Defines the I2C address for the external EEPROM which stores type information */
#define	I2C_ADDRESS_TYPE_EEPROM				0x51

/** \brief Allows to use this firmware together with the non-Repetier PC applications
Without this special handling, the firmware may complain about checksum errors from non-Repetier PC applications (e.g. Cura, ...) and
non-Repetier PC applications may fall over the debug outputs of the firmware. */
#define	ALLOW_EXTENDED_COMMUNICATION		2													// 0 = do not allow, 1 = allow "Wait", 2 = allow "Wait" and debug outputs

/** \brief Configures the delay between the stop of a print and the clean-up like disabling of heaters, disabling of steppers and the outputting of the object */
#define	CLEAN_UP_DELAY_AFTER_STOP_PRINT		1000												// [ms]

/** \brief Configures the duration for which the processing of commands shall be blocked. */
#define	COMMAND_BLOCK_DELAY					1000												// [ms]

/** \brief Configuration of the external watchdog
The TPS3820 of the RF1000/RF2000 resets about 25 ms after the last time when it was triggered, the value of WATCHDOG_TIMEOUT should be less than half of this time. */
#define WATCHDOG_TIMEOUT					10													// [ms]
#define	WATCHDOG_MAIN_LOOP_TIMEOUT			10000												// [ms]

/** \brief Longer-lasting operations shall call our periodical actions at least each defined time interval */
#define	PERIODICAL_ACTIONS_CALL_INTERVAL	10													// [ms]

/** \brief The display shows that the device is idle after no new commands were processed for longer than the minimal idle time */
#define	MINIMAL_IDLE_TIME					500													// [ms]

/** \brief If enabled you can select the distance your filament gets retracted during a M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP				true

/** \brief add pid control */
#define TEMP_PID							true

/** \brief PID control only works target temperature +/- PID_CONTROL_RANGE.
If you get much overshoot at the first temperature set, because the heater is going full power too long, you
need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15. */
#define PID_CONTROL_RANGE					20

/** \brief Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter! */
#define EXTRUDE_MAXLENGTH					100

/** \brief Skip wait, if the extruder temperature is already within x degrees. Only fixed numbers, 0 = off */
#define SKIP_M109_IF_WITHIN					2

/** \brief Set PID scaling
PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by two methods.
Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
If your EXT0_PID_MAX is low, you should prefer the second method. */
#define SCALE_PID_TO_MAX					0

/** \brief Temperature range for target temperature to hold in M109 command. 5 means +/-5 degC
Uncomment define to force the temperature into the range for given watchperiod. */
//#define TEMP_HYSTERESIS					5
#define	TEMP_TOLERANCE						2.0													// [°C]

/** \brief Bits of the ADC converter */
#define ANALOG_INPUT_BITS					10

/** \brief Build median from 2^ANALOG_INPUT_SAMPLE samples */
#define ANALOG_INPUT_SAMPLE					5
#define ANALOG_REF_AREF						0
#define ANALOG_REF_AVCC						_BV(REFS0)
#define ANALOG_REF_INT_1_1					_BV(REFS1)
#define ANALOG_REF_INT_2_56					_BV(REFS0) | _BV(REFS1)
#define ANALOG_REF							ANALOG_REF_AVCC

/** \brief Step to split a circle in small Lines */
#define MM_PER_ARC_SEGMENT					1
#define MM_PER_ARC_SEGMENT_BIG				3

/** \brief After this count of steps a new SIN / COS caluclation is startet to correct the circle interpolation */
#define N_ARC_CORRECTION					25

/** \brief Communication speed.
Overridden if EEPROM activated. */
#define BAUDRATE							115200

/** \brief Some boards like Gen7 have a power on pin, to enable the atx power supply. If this is defined,
the power will be turned on without the need to call M80 if initially started. */
#define ENABLE_POWER_ON_STARTUP

/** \brief If you use an ATX power supply you need the power pin to work non inverting. For some special
boards you might need to make it inverting. */
#define POWER_INVERTING						false

/** \brief What shall the printer do, when it receives an M112 emergency stop signal?
 0 = Disable heaters/motors, wait forever until someone presses reset.
 1 = restart by resetting the AVR controller. The USB connection will not reset if managed by a different chip! */
#define KILL_METHOD							1

/** \brief Cache size for incoming commands.
There should be no reason to increase this cache. Commands are nearly immediately sent to
execution. */
#define GCODE_BUFFER_SIZE					2

/** \brief Appends the linenumber after every ok send, to acknowledge the received command. Uncomment for plain ok ACK if your host has problems with this */
#define ACK_WITH_LINENUMBER

/** \brief Communication errors can swollow part of the ok, which tells the host software to send
the next command. Not receiving it will cause your printer to stop. Sending this string every
second, if our queue is empty should prevent this. Comment it, if you don't wan't this feature. */
#define WAITING_IDENTIFIER					"wait"

/** \brief Sets time for echo debug
You can set M111 1 which enables ECHO of commands sent. This define specifies the position,
when it will be executed. In the original FiveD software, echo is done after receiving the
command. With checksum you know, how it looks from the sending string. With this define
uncommented, you will see the last command executed. To be more specific: It is written after
execution. This helps tracking errors, because there may be 8 or more commands in the queue
and it is elsewise difficult to know, what your reprap is currently doing. */
#define ECHO_ON_EXECUTE						1

/** \brief If the firmware is busy, it will send a busy signal to host signaling that
 everything is fine and it only takes a bit longer to finish. That way the 
 host can keep timeout short so in case of communication errors the resulting
 blobs are much smaller. Set to 0 to disable it. */
#define KEEP_ALIVE_INTERVAL					2000												// [ms]

/** \brief  Turn the case light on/off per default */
#define CASE_LIGHTS_DEFAULT_ON				0

/** \brief Define the on temperature and the off delay for the fan */
#define	CASE_FAN_ON_TEMPERATURE				50													// [°C]
#define	CASE_FAN_OFF_DELAY					60000												// [ms]

/** \brief Specify whether the case fan all be always on */
#define CASE_FAN_ALWAYS_ON					0													// 1 = always on, 0 = automatic switching and switching via G-Code

/** \brief Defines the default behavior of the Position X/Y/Z menus */
#define DEFAULT_MOVE_MODE_X					MOVE_MODE_SINGLE_MOVE
#define DEFAULT_MOVE_MODE_Y					MOVE_MODE_SINGLE_MOVE
#define DEFAULT_MOVE_MODE_Z					MOVE_MODE_SINGLE_MOVE

/** \brief Defines the default z scale */
#define DEFAULT_Z_SCALE_MODE				Z_VALUE_MODE_Z_MIN


#endif // CONFIGURATION_H
