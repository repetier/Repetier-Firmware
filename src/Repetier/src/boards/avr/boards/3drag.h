// Felix board
#if MOTHERBOARD == 66
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// X/Y/Z Steppers and MIN endstops verified

#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_ENABLE_PIN 38
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN -1

#define ORIG_Y_STEP_PIN 60
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_ENABLE_PIN 56
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN -1

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_ENABLE_PIN 63
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN -1

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN -1
#define ORIG_E1_DIR_PIN -1
#define ORIG_E1_ENABLE_PIN -1

#define SDPOWER -1
#define SDSS 25
#define ORIG_SDCARDDETECT -1

#define LED_PIN 13
#define ORIG_FAN_PIN 8
#define ORIG_PS_ON_PIN -1

// schematic: HEATER1 (Extruder)
#define HEATER_0_PIN 10
// schematic: HEATER2 (Heated Bed)
#define HEATER_1_PIN 9
#define HEATER_2_PIN -1

// ANALOG NUMBERING
// schematic: THERM1 (Extruder)
#define TEMP_0_PIN 13
// schematic: THERM2 (Heated Bed)
#define TEMP_1_PIN 14
#define TEMP_2_PIN -1

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

// following pins (LCD, ENCODER, SDCARD) reverse engineered from schematic diagram:

#ifdef ULTRA_LCD
#ifdef NEWPANEL

#define LCD_PINS_RS 27
#define LCD_PINS_ENABLE 29
#define LCD_PINS_D4 37
#define LCD_PINS_D5 35
#define LCD_PINS_D6 33
#define LCD_PINS_D7 31

#define BTN_EN1 16
#define BTN_EN2 17
#define BTN_ENC 23

#endif
#endif //ULTRA_LCD

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51
#define MAX6675_SS 53
#endif
