/****************************************************************************************
* MegaTronics v3.0
*
****************************************************************************************/
#if MOTHERBOARD == 703
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN 58
#define ORIG_X_DIR_PIN 57
#define ORIG_X_ENABLE_PIN 59
#define ORIG_X_MIN_PIN 37
//2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN 40

#define ORIG_Y_STEP_PIN 5
#define ORIG_Y_DIR_PIN 17
#define ORIG_Y_ENABLE_PIN 4
#define ORIG_Y_MIN_PIN 41
#define ORIG_Y_MAX_PIN 38

#define ORIG_Z_STEP_PIN 16
#define ORIG_Z_DIR_PIN 11
#define ORIG_Z_ENABLE_PIN 3
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19

#define ORIG_E0_STEP_PIN 28
#define ORIG_E0_DIR_PIN 27
#define ORIG_E0_ENABLE_PIN 29

#define ORIG_E1_STEP_PIN 25
#define ORIG_E1_DIR_PIN 24
#define ORIG_E1_ENABLE_PIN 26

#define ORIG_E2_STEP_PIN 22
#define ORIG_E2_DIR_PIN 60
#define ORIG_E2_ENABLE_PIN 23

#define ORIG_SDCARDDETECT -1 // Ramps does not use this port
#define SDPOWER -1
#define SDSS 53

#define LED_PIN 13

#define ORIG_FAN_PIN 6
#define ORIG_FAN2_PIN 7

#define ORIG_PS_ON_PIN 12
//#define KILL_PIN -1

// EXTRUDER 0 - changed 10-9-2015
#define HEATER_0_PIN 2
// EXTRUDER 1 - changed 10-9-2015
#define HEATER_2_PIN 9
// EXTRUDER 2 - changed 10-9-2015
#define HEATER_3_PIN 8
// heater bed
#define HEATER_1_PIN 10

/*
Temperature sensors
ANALOG NUMBERING!

Thermistors
T0=15
T1=14
T2=13
T3=12

Thermocouple
S0=11
S1=10
S2(ext)=8
S3(ext)=9
*/

// Extruder 1 - Thermistor 1
#define TEMP_0_PIN 15
// Extruder 2 - Thermistor 2
#define TEMP_2_PIN 14
// Extruder 3 - Thermistor 3
#define TEMP_3_PIN 13
// Heated bed - Thermistor 4
#define TEMP_1_PIN 12

#define THERMOCOUPLE_0_PIN 11
#define THERMOCOUPLE_1_PIN 10
#define THERMOCOUPLE_2_PIN 8
#define THERMOCOUPLE_3_PIN 9

// Beeper on AUX-4
#define BEEPER_PIN 61
#define SDSUPPORT 1 // sd card reader on board

// #define UI_DISPLAY_RS_PIN 32
// #define UI_DISPLAY_ENABLE_PIN 31
// #define UI_DISPLAY_D4_PIN 14
// #define UI_DISPLAY_D5_PIN 30
// #define UI_DISPLAY_D6_PIN 39
// #define UI_DISPLAY_D7_PIN 15

//buttons are directly attached using AUX-2
////encoder A 59
////encoder B 64
////encoder click 33 //the click

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#endif
