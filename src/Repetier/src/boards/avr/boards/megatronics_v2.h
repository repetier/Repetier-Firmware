/****************************************************************************************
* MegaTronics v2.0
*
****************************************************************************************/
#if MOTHERBOARD == 701
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN 26
#define ORIG_X_DIR_PIN 27
#define ORIG_X_ENABLE_PIN 25
#define ORIG_X_MIN_PIN 37
//2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN 40

#define ORIG_Y_STEP_PIN 4
#define ORIG_Y_DIR_PIN 54
#define ORIG_Y_ENABLE_PIN 5
#define ORIG_Y_MIN_PIN 41
#define ORIG_Y_MAX_PIN 38

#define ORIG_Z_STEP_PIN 56
#define ORIG_Z_DIR_PIN 60
#define ORIG_Z_ENABLE_PIN 55
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19

#define ORIG_E0_STEP_PIN 35
#define ORIG_E0_DIR_PIN 36
#define ORIG_E0_ENABLE_PIN 34

#define ORIG_E1_STEP_PIN 29
#define ORIG_E1_DIR_PIN 39
#define ORIG_E1_ENABLE_PIN 28

#define ORIG_E2_STEP_PIN 23
#define ORIG_E2_DIR_PIN 24
#define ORIG_E2_ENABLE_PIN 22

#define ORIG_SDCARDDETECT -1 // Ramps does not use this port
#define SDPOWER -1
#define SDSS 53

#define LED_PIN 13

#define ORIG_FAN_PIN 7
#define ORIG_FAN2_PIN 6
#define ORIG_PS_ON_PIN 12

// EXTRUDER 1
#define HEATER_0_PIN 9
// Heated bed
#define HEATER_2_PIN 8
// EXTRUDER 2
#define HEATER_1_PIN 10

// Thermistor 0 ANALOG NUMBERING
#define TEMP_0_PIN 13
// Thermistor 1 ANALOG NUMBERING
#define TEMP_2_PIN 15
// Thermistor 2 for heated bed ANALOG NUMBERING
#define TEMP_1_PIN 14
// Thermocouple 0
#define TEMP_3_PIN 8
// Thermocouple 1
#define TEMP_4_PIN 4
#define THERMOCOUPLE_0_PIN 8
#define THERMOCOUPLE_1_PIN 4

// Beeper on AUX-4
#define BEEPER_PIN 64

#define LCD_PINS_RS 14
#define LCD_PINS_ENABLE 15
#define LCD_PINS_D4 30
#define LCD_PINS_D5 31
#define LCD_PINS_D6 32
#define LCD_PINS_D7 33

//buttons are directly attached using AUX-2
#define BTN_EN1 59
#define BTN_EN2 64
#define BTN_ENC 43

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#endif
