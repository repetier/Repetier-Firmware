/****************************************************************************************
* FELIXprinters
*
****************************************************************************************/
#if MOTHERBOARD == 101
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
#endif

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
#define ORIG_Z_ENABLE_PIN 62
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN -1

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define LED_PIN 13
#define ORIG_FAN_PIN 9
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1

#define HEATER_0_PIN 10
//BED
#define HEATER_1_PIN 8
#define HEATER_2_PIN 7

// ANALOG NUMBERING
#define TEMP_0_PIN 13
// BED,ANALOG NUMBERING
#define TEMP_1_PIN 14
#define TEMP_2_PIN 15

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define SDPOWER 1
#define SDSS 53
#define ORIG_SDCARDDETECT 6
#define SDSUPPORT 1            // already defined in config.h
#define SDCARDDETECTINVERTED 1 // already defined in config.h

// these pins are defined in the SD library if building with SD support
// PINB.1, 20, SCK
#define SCK_PIN 52
// PINB.3, 22, MISO
#define MISO_PIN 50
// PINB.2, 21, MOSI
#define MOSI_PIN 51
//53	// PINB.0, 19, SS
#define MAX6675_SS -1

#define BEEPER_PIN -1 // Activate beeper on extension shield
#define BEEPER_TYPE 1

#endif //MOTHERBOARD == 101
