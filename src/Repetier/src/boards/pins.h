#ifndef _pins_h_main
#define _pins_h_main

#include "Arduino.h"

#define ARCH_AVR 1
#define ARCH_ARM 2
#define ARCH_DUE 2

#define CURRENT_CONTROL_MANUAL 1    // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2   // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3   // Use LTC2600 like Foltyn 3D Master
#define CURRENT_CONTROL_ALLIGATOR 4 //Use External DAC like Alligator
#define CURRENT_CONTROL_MCP4728 5   // Use an i2c DAC as a digipot like PrintrBoard Rev. F

inline void memcopy2(void* dest, void* source) {
    *((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void* dest, void* source) {
    *((int32_t*)dest) = *((int32_t*)source);
}

// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2       = 3
// RAMPS 1.3/RAMPS 1.4        = 33
// Azteeg X3                  = 34
// Azteeg X3 Pro              = 35
// MPX3  (mainly RAMPS compatible) = 38
// Ultimaker Shield 1.5.7     = 37
// Open Motion Controller     = 91
// Azteeg X1                  = 65
// 3Drag/Velleman K8200 (experimental) = 66
// Sethi 3D_1                 = 72
// Foltyn 3D Master           = 12
// MegaTronics 1.0            = 70
// Megatronics 2.0            = 701
// Megatronics 3.0            = 703 // Thermistors predefined not thermocouples
// Minitronics 1.0            = 702
// RUMBA                      = 80  // Get it from reprapdiscount
// FELIXprinters              = 101
// Rambo                      = 301
// PiBot for Repetier V1.0-1.3= 314
// PiBot for Repetier V1.4    = 315
// PiBot Controller V2.0      = 316
// Sanguish Beta              = 501
// Unique One rev. A          = 88
// SAV MK1                    = 89
// MJRice Pica Rev B          = 183
// MJRice Pica Rev C          = 184
// Zonestar ZRIB 2.1          = 39
// User layout defined in userpins.h = 999

#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 38 || MOTHERBOARD == 37 || MOTHERBOARD == 66 || MOTHERBOARD == 91 || MOTHERBOARD == 65 || MOTHERBOARD == 72 || MOTHERBOARD == 12 || MOTHERBOARD == 70 || MOTHERBOARD == 701 || MOTHERBOARD == 702 || MOTHERBOARD == 703 || MOTHERBOARD == 80 || MOTHERBOARD == 101 || MOTHERBOARD == 301 || MOTHERBOARD == 314 || MOTHERBOARD == 315 || MOTHERBOARD == 316 || MOTHERBOARD == 501 || MOTHERBOARD == 88 || MOTHERBOARD == 89 || MOTHERBOARD == 183 || MOTHERBOARD == 184 || MOTHERBOARD == 39 || MOTHERBOARD == MOTHERBOARD_USER_DEFINED_AVR

#define AVR_BOARD 1
#include "avr/pins.h"

// Due boards

// Arduino Due with RADDS       = 402
// Arduino Due with RAMPS-FD    = 403
// Arduino Due with RAMPS-FD V2 = 404
// Felix Printers for arm       = 405
// DAM&DICE DUE                 = 406
// Smart RAMPS for Due          = 408
// Smart RAMPS for Due with EEPROM = 413
// Ultratronics Board           = 409
// DUE3DOM                      = 410
// DUE3DOM MINI                 = 411
// STACKER 3D Superboard        = 412
// RURAMPS4D                    = 414
// Alligator Board rev1         = 500
// Alligator Board rev2         = 501
// Alligator Board rev3         = 502
// User defined due board       = 998

#elif MOTHERBOARD == MOTHERBOARD_RADDS || MOTHERBOARD == MOTHERBOARD_RAMPS_FD_INVERTED_HEATER || MOTHERBOARD == MOTHERBOARD_RAMPS_FD || MOTHERBOARD == MOTHERBOARD_FELIX || MOTHERBOARD == MOTHERBOARD_BAM_AND_DICE || MOTHERBOARD == MOTHERBOARD_SMARTRAMPS_NO_EEPROM || MOTHERBOARD == MOTHERBOARD_SMARTRAMPS_EEPROM || MOTHERBOARD == MOTHERBOARD_ULTRATRONICS || MOTHERBOARD == 410 || MOTHERBOARD == 411 || MOTHERBOARD == MOTHERBOARD_STACKER_3D_SUPERBOARD || MOTHERBOARD == MOTHERBOARD_RURAMPS4D || MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV1 || MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV2 || MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV3 || MOTHERBOARD == MOTHERBOARD_USER_DEFINED_DUE || MOTHERBOARD == MOTHERBOARD_IKS3D

#define DUE_BOARD 1
#include "due/pins.h"

#elif MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_NO_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_USER_DEFINED || MOTHERBOARD == MOTHERBOARD_AGC_RADDS

#define SAMD51_BOARD 1
#include "SAMD51/pins.h"

#elif MOTHERBOARD >= 2000 && MOTHERBOARD < 2500

#define M0_BOARD
#include "M0/pins.h"

#elif MOTHERBOARD >= 2500 && MOTHERBOARD < 3000

#define STM32F1_BOARD
#include "STM32F1/pins.h"

#elif MOTHERBOARD >= 3000 && MOTHERBOARD < 3500

#define STM32F4_BOARD
#include "STM32F4/pins.h"

#else

#error Motherboard with set ID is not available!

#endif

#ifdef AVR_BOARD
#include "avr/HAL.h"
#endif
#ifdef DUE_BOARD
#include "due/HAL.h"
#endif
#ifdef SAMD51_BOARD
#include "SAMD51/HAL.h"
#endif
#ifdef M0_BOARD
#include "M0/HAL.h"
#endif
#ifdef STM32F1_BOARD
#include "STM32F1/HAL.h"
#endif
#ifdef STM32F4_BOARD
#include "STM32F4/HAL.h"
#endif

#ifndef BEEPER_PIN
#define BEEPER_PIN -1
#endif

#ifndef UI_BACK_PIN
#define UI_BACK_PIN -1
#endif
#ifndef UI_RESET_PIN
#define UI_RESET_PIN -1
#endif
#ifndef UI_ENCODER_CLICK
#define UI_ENCODER_CLICK -1
#endif
#ifndef UI_ENCODER_A
#define UI_ENCODER_A -1
#endif
#ifndef UI_ENCODER_B
#define UI_ENCODER_B -1
#endif
#ifndef MAX_VFAT_ENTRIES
#ifdef AVR_BOARD
#define MAX_VFAT_ENTRIES (2)
#else
#define MAX_VFAT_ENTRIES (3)
#endif
#endif
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (13 * MAX_VFAT_ENTRIES + 1)
#define SD_MAX_FOLDER_DEPTH 2

#if UI_DISPLAY_TYPE != DISPLAY_U8G
#if (defined(USER_KEY1_PIN) && (USER_KEY1_PIN == UI_DISPLAY_D5_PIN || USER_KEY1_PIN == UI_DISPLAY_D6_PIN || USER_KEY1_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY2_PIN) && (USER_KEY2_PIN == UI_DISPLAY_D5_PIN || USER_KEY2_PIN == UI_DISPLAY_D6_PIN || USER_KEY2_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY3_PIN) && (USER_KEY3_PIN == UI_DISPLAY_D5_PIN || USER_KEY3_PIN == UI_DISPLAY_D6_PIN || USER_KEY3_PIN == UI_DISPLAY_D7_PIN)) || (defined(USER_KEY4_PIN) && (USER_KEY4_PIN == UI_DISPLAY_D5_PIN || USER_KEY4_PIN == UI_DISPLAY_D6_PIN || USER_KEY4_PIN == UI_DISPLAY_D7_PIN))
#error You cannot use DISPLAY_D5_PIN, DISPLAY_D6_PIN or DISPLAY_D7_PIN for "User Keys" with character LCD display
#endif
#endif

#ifndef SDCARDDETECT
#define SDCARDDETECT -1
#endif

#ifndef SDSUPPORT
#if FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD || FEATURE_CONTROLLER == CONTROLLER_FELIX_DUE \
    || FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2 || FEATURE_CONTROLLER == CONTROLLER_ENDER_3_12864 \
    || FEATURE_CONTROLLER == CONTROLLER_RADDS || FEATURE_CONTROLLER == CONTROLLER_SMARTRAMPS
#define SDSUPPORT 1
#endif
#endif

#ifndef SDSUPPORT
#define SDSUPPORT 0
#endif

#ifndef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0
#endif

#include "communication/gcode.h"
#if SDSUPPORT  
#include "SdFat/src/SdFat.h"
#endif

#endif
