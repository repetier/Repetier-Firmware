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

This firmware is a nearly complete rewrite of the sprinter firmware
by kliment (https://github.com/kliment/Sprinter)
which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/
#ifndef RF_DISPLAY
#define RF_DISPLAY

#define UI_FONT_DEFAULT_RU ISO_6x10
#define UI_FONT_SMALL_RU ISO_5x7

#if FEATURE_CONTROLLER == UICONFIG_CONTROLLER
#include "uiconfig.h"
#endif
// No controller at all
#if FEATURE_CONTROLLER == NO_CONTROLLER
#define UI_HAS_KEYS 0
#define UI_DISPLAY_TYPE NO_DISPLAY
#ifdef UI_MAIN
void uiInitKeys() {}
void uiCheckKeys(uint16_t &action) {}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif // UI_MAIN
#endif // NO_CONTROLLER

#if (FEATURE_CONTROLLER == CONTROLLER_SMARTRAMPS) || (FEATURE_CONTROLLER == CONTROLLER_GADGETS3D_SHIELD) || (FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD)  || (FEATURE_CONTROLLER == CONTROLLER_BAM_DICE_DUE) || (FEATURE_CONTROLLER == CONTROLLER_REPRAPWORLD_GLCD)
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#if FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD || (FEATURE_CONTROLLER == CONTROLLER_REPRAPWORLD_GLCD)
#define UI_DISPLAY_TYPE DISPLAY_U8G
#define U8GLIB_ST7920
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64
//select font size
#define UI_FONT_6X10 //default font
#ifdef UI_FONT_6X10
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#endif

//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_CHARSET 3
#else // 40x4 char display
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#endif

#define BEEPER_TYPE 1

#if FEATURE_CONTROLLER == CONTROLLER_GADGETS3D_SHIELD // Gadgets3d shield

#undef BEEPER_PIN
#define BEEPER_PIN             33
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      23
#define UI_DISPLAY_D1_PIN      25
#define UI_DISPLAY_D2_PIN      27
#define UI_DISPLAY_D3_PIN      29
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_ENCODER_A           35
#define UI_ENCODER_B           37
#define UI_ENCODER_CLICK       31
#define UI_RESET_PIN           41
#else  // Smartcontroller

#if MOTHERBOARD == 701 // Megatronics v2.0

#define UI_DISPLAY_RS_PIN 14
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 15
#define UI_DISPLAY_D0_PIN -1
#define UI_DISPLAY_D1_PIN -1
#define UI_DISPLAY_D2_PIN -1
#define UI_DISPLAY_D3_PIN -1
#define UI_DISPLAY_D4_PIN 30
#define UI_DISPLAY_D5_PIN 31
#define UI_DISPLAY_D6_PIN 32
#define UI_DISPLAY_D7_PIN 33
#define UI_ENCODER_A 61
#define UI_ENCODER_B 59
#define UI_ENCODER_CLICK 43
#define UI_RESET_PIN 66 // was 41 //AE3 was here and added this line 1/25/2014  (Note pin 41 is Y- endstop!)
#define UI_INVERT_MENU_DIRECTION 1

#elif MOTHERBOARD == 703 // Megatronics v3.0

#define UI_DISPLAY_RS_PIN 32
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 31
#define UI_DISPLAY_D0_PIN -1
#define UI_DISPLAY_D1_PIN -1
#define UI_DISPLAY_D2_PIN -1
#define UI_DISPLAY_D3_PIN -1
#define UI_DISPLAY_D4_PIN 14
#define UI_DISPLAY_D5_PIN 30
#define UI_DISPLAY_D6_PIN 39
#define UI_DISPLAY_D7_PIN 15
#define UI_ENCODER_A 45
#define UI_ENCODER_B 44
#define UI_ENCODER_CLICK 33
#define UI_INVERT_MENU_DIRECTION 1
#define UI_RESET_PIN -1

#elif MOTHERBOARD == 80 // Rumba has different pins as RAMPS!

#undef BEEPER_PIN
#define BEEPER_PIN             44
#define UI_DISPLAY_RS_PIN      19
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  42
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      18
#define UI_DISPLAY_D5_PIN      38
#define UI_DISPLAY_D6_PIN      41
#define UI_DISPLAY_D7_PIN      40
#define UI_ENCODER_A           12
#define UI_ENCODER_B           11
#define UI_ENCODER_CLICK       43
#define UI_RESET_PIN           46

#elif MOTHERBOARD == 37 // UltiMaker 1.5.7
#undef BEEPER_PIN
#define BEEPER_PIN 18
#define UI_DISPLAY_RS_PIN      20
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      16
#define UI_DISPLAY_D5_PIN      21
#define UI_DISPLAY_D6_PIN      5
#define UI_DISPLAY_D7_PIN      6
#define UI_ENCODER_A           42
#define UI_ENCODER_B           40
#define UI_ENCODER_CLICK       19
#define UI_RESET_PIN           -1

#elif MOTHERBOARD == 301 // Rambo has own pins layout

#undef BEEPER_PIN
#define BEEPER_PIN             79
#define UI_DISPLAY_RS_PIN      70
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  71
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      72
#define UI_DISPLAY_D5_PIN      73
#define UI_DISPLAY_D6_PIN      74
#define UI_DISPLAY_D7_PIN      75
#define UI_ENCODER_A           76
#define UI_ENCODER_B           77
#define UI_ENCODER_CLICK       78
#define UI_RESET_PIN           80
#undef SDCARDDETECT
#define SDCARDDETECT           81
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED   0
#undef SDSUPPORT
#define SDSUPPORT              1

#elif MOTHERBOARD == 501 // Alligator has own pins layout

#undef BEEPER_PIN
#define BEEPER_PIN             64
#define UI_DISPLAY_RS_PIN      18
#define UI_DISPLAY_ENABLE_PIN  15
#define UI_DISPLAY_D4_PIN      19
#define UI_ENCODER_A           14
#define UI_ENCODER_B           16
#define UI_ENCODER_CLICK       17
#define UI_RESET_PIN           -1
#undef SDCARDDETECT
#define SDCARDDETECT           87
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED   0
#ifndef UI_VOLTAGE_LEVEL
#define UI_VOLTAGE_LEVEL 1 // Set 1=5 o 0=3.3 V
#endif

#elif ((MOTHERBOARD == 410) || (MOTHERBOARD == 411))	// DUE3DOM / DUE3DOM MINI has own pins layout

#undef BEEPER_PIN
#define BEEPER_PIN             41
#define UI_DISPLAY_RS_PIN      42
#define UI_DISPLAY_ENABLE_PIN  43
#define UI_DISPLAY_D4_PIN      44
#define UI_DISPLAY_D5_PIN      45
#define UI_DISPLAY_D6_PIN      46
#define UI_DISPLAY_D7_PIN      47
#define UI_ENCODER_A           52
#define UI_ENCODER_B           50
#define UI_ENCODER_CLICK       48
#define UI_RESET_PIN           -1
#undef SDCARDDETECT
#define SDCARDDETECT           14
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED   0
#ifndef UI_VOLTAGE_LEVEL
#define UI_VOLTAGE_LEVEL 1 // Set 1=5 o 0=3.3 V
#endif

#elif MOTHERBOARD == 405 // Felix Pro 1

#undef BEEPER_PIN
#define BEEPER_PIN             -1
#define UI_DISPLAY_RS_PIN      42
#define UI_DISPLAY_ENABLE_PIN  44
#define UI_DISPLAY_D4_PIN      43
#define UI_ENCODER_A           52
#define UI_ENCODER_B           50
#define UI_ENCODER_CLICK       48
#define UI_RESET_PIN           -1

#elif MOTHERBOARD == 101 // Felix Pro 1

#undef BEEPER_PIN
#define BEEPER_PIN             -1
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D4_PIN      23
#define UI_ENCODER_A           35
#define UI_ENCODER_B           37
#define UI_ENCODER_CLICK       31
#define UI_RESET_PIN           -1

#elif ( MOTHERBOARD == 183 ) || ( MOTHERBOARD == 184 ) // MJRice Pica

#undef BEEPER_PIN
#define BEEPER_PIN 19
#define UI_DISPLAY_RS_PIN 33
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 30
#define UI_DISPLAY_D0_PIN -1
#define UI_DISPLAY_D1_PIN -1
#define UI_DISPLAY_D2_PIN -1
#define UI_DISPLAY_D3_PIN -1
#define UI_DISPLAY_D4_PIN 35
#define UI_DISPLAY_D5_PIN 32
#define UI_DISPLAY_D6_PIN 37
#define UI_DISPLAY_D7_PIN 36
#define UI_ENCODER_A 47
#define UI_ENCODER_B 48
#define UI_ENCODER_CLICK 31
#define UI_RESET_PIN -1
#define SDCARDDETECT 49

#elif ((MOTHERBOARD == 409))  // Ultratronics

#undef BEEPER_PIN
#define BEEPER_PIN             27
//#undef U8GLIB_ST7920
//#define U8GLIB_ST7920_HW
// CS
#define UI_DISPLAY_RS_PIN      62
// MOSI
#define UI_DISPLAY_ENABLE_PIN  75
// SCK
#define UI_DISPLAY_D4_PIN      76
#define UI_DISPLAY_D5_PIN      -1
#define UI_DISPLAY_D6_PIN      -1
#define UI_DISPLAY_D7_PIN      -1
#define UI_ENCODER_A           20
#define UI_ENCODER_B           21
#define UI_ENCODER_CLICK       64
#define UI_RESET_PIN           -1
#undef SDCARDDETECT
#define SDCARDDETECT           60
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED   0

#elif ( MOTHERBOARD == 414 ) || ( MOTHERBOARD == 415 ) // RURAMPS4D

#undef BEEPER_PIN
#define BEEPER_PIN        62
#define UI_DISPLAY_RS_PIN 63
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 64
#define UI_DISPLAY_D0_PIN -1
#define UI_DISPLAY_D1_PIN -1
#define UI_DISPLAY_D2_PIN -1
#define UI_DISPLAY_D3_PIN -1
#define UI_DISPLAY_D4_PIN 48
#define UI_DISPLAY_D5_PIN 50
#define UI_DISPLAY_D6_PIN 52
#define UI_DISPLAY_D7_PIN 53
#define UI_ENCODER_A 42
#define UI_ENCODER_B 44
#define UI_ENCODER_CLICK 40
#define UI_RESET_PIN -1
#define UI_INVERT_MENU_DIRECTION 1

#elif MOTHERBOARD == 402 // RADDS with RADDS2LCD Adapter 
// https://www.thingiverse.com/thing:1740725/files
#define BEEPER_TYPE 1
#undef BEEPER_PIN
#define BEEPER_PIN             41
#define UI_DISPLAY_RS_PIN      42
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  43
#define UI_DISPLAY_D0_PIN      44
#define UI_DISPLAY_D1_PIN      45
#define UI_DISPLAY_D2_PIN      46
#define UI_DISPLAY_D3_PIN      47
#define UI_DISPLAY_D4_PIN      44
#define UI_DISPLAY_D5_PIN      45
#define UI_DISPLAY_D6_PIN      46
#define UI_DISPLAY_D7_PIN      47

// swap these two numbers to invert rotary encoder scroll direction
#define UI_ENCODER_A           50
#define UI_ENCODER_B           52

#define UI_ENCODER_CLICK       48
#define UI_RESET_PIN           -1
#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION 0
#define UI_BUTTON_BACK         71

#elif MOTHERBOARD == 403 || MOTHERBOARD == 404

 // ramps-fd lcd adaptor needs to rotate connectors 180° to work!
#define UI_DISPLAY_RS_PIN         16
#define UI_DISPLAY_ENABLE_PIN     17
#define UI_DISPLAY_D4_PIN         23
#define UI_DISPLAY_D5_PIN         25
#define UI_DISPLAY_D6_PIN         27
#define UI_DISPLAY_D7_PIN         29
#define BEEPER_PIN                37
#define UI_ENCODER_A              33
#define UI_ENCODER_B              31
#define UI_ENCODER_CLICK          35
#define UI_RESET_PIN              -1
#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION   0
#define UI_BUTTON_BACK            -1
#undef SDCARDDETECT
#define SDCARDDETECT           49
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED   0
#undef SDSUPPORT
#define SDSUPPORT              1

#elif MOTHERBOARD == 408 || MOTHERBOARD == 413

// SMART RAMPS FOR DUE - CRITICAL NOTE: MUST REMOVE THE RESET HEADER JUMPER NEXT TO AUX-2 OTHERWISE BOARD WILL RESET LOOP CONTINUOUSLY
#define UI_DISPLAY_RS_PIN         44 //CS
#define UI_DISPLAY_ENABLE_PIN     42 //MOSI
#define UI_DISPLAY_D4_PIN         40 //SCK
#define UI_DISPLAY_D5_PIN         -1 //A0 LCD RS
#define UI_DISPLAY_D6_PIN         -1
#define UI_DISPLAY_D7_PIN         -1
#define BEEPER_PIN                66
#define UI_ENCODER_A              50
#define UI_ENCODER_B              47
#define UI_ENCODER_CLICK          67
#define UI_RESET_PIN              53
#define UI_DELAYPERCHAR           50
#define UI_INVERT_MENU_DIRECTION   0
#define UI_BUTTON_BACK            -1

#else  // RAMPS

#undef BEEPER_PIN
#define BEEPER_PIN             37
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      23
#define UI_DISPLAY_D1_PIN      25
#define UI_DISPLAY_D2_PIN      27
#define UI_DISPLAY_D3_PIN      29
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_ENCODER_A           33
#define UI_ENCODER_B           31
#define UI_ENCODER_CLICK       35
#define UI_RESET_PIN           41
#endif
#endif // smartcontroller

#if (FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD) && (MOTHERBOARD == 63) //Melzi V2 + ReprapDiscount GLCD (such as Wanhao Duplicator i3)
#define BEEPER_PIN             27
#define UI_DISPLAY_RS_PIN      17
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  16
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      11
#define UI_DISPLAY_D5_PIN      -1
#define UI_DISPLAY_D6_PIN      -1
#define UI_DISPLAY_D7_PIN      -1
#define UI_ENCODER_A           29
#define UI_ENCODER_B           30
#define UI_ENCODER_CLICK       28
#define UI_RESET_PIN           10
#define SDCARDDETECT          -1
#endif

#define UI_DELAYPERCHAR 50
#if FEATURE_CONTROLLER == CONTROLLER_BAM_DICE_DUE
#define UI_ENCODER_SPEED 2
#endif
#ifndef UI_INVERT_MENU_DIRECTION
#define UI_INVERT_MENU_DIRECTION 0
#endif

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
#if UI_RESET_PIN > -1
  UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
#endif
}
void uiCheckKeys(uint16_t &action) {
#if FEATURE_CONTROLLER == CONTROLLER_BAM_DICE_DUE
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_B, UI_ENCODER_A); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
#else
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
#endif
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
#if UI_RESET_PIN > -1
  UI_KEYS_BUTTON_LOW(UI_RESET_PIN, UI_ACTION_RESET);
#endif
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller 2 and 10

#if FEATURE_CONTROLLER == CONTROLLER_ADAFRUIT
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_I2C
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 16
#define UI_ROWS 2
#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 65504
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0
#define UI_DISPLAY_I2C_PULLUP 31
#define UI_I2C_CLOCKSPEED 400000L
#define UI_DISPLAY_RS_PIN _BV(15)
#define UI_DISPLAY_RW_PIN _BV(14)
#define UI_DISPLAY_ENABLE_PIN _BV(13)
#define UI_DISPLAY_D0_PIN _BV(12)
#define UI_DISPLAY_D1_PIN _BV(11)
#define UI_DISPLAY_D2_PIN _BV(10)
#define UI_DISPLAY_D3_PIN _BV(9)
#define UI_DISPLAY_D4_PIN _BV(12)
#define UI_DISPLAY_D5_PIN _BV(11)
#define UI_DISPLAY_D6_PIN _BV(10)
#define UI_DISPLAY_D7_PIN _BV(9)
#define UI_INVERT_MENU_DIRECTION 1
#define UI_HAS_I2C_KEYS
#define UI_HAS_I2C_ENCODER 0
#define UI_I2C_KEY_ADDRESS 0x40
#ifdef UI_MAIN
void uiInitKeys() {}
void uiCheckKeys(uint16_t &action) {}
inline void uiCheckSlowEncoder() {
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
  HAL::i2cWrite(0x12); // GIOA
  HAL::i2cStop();
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_READ);
  uint16_t keymask = HAL::i2cReadAck();
  keymask = keymask + (HAL::i2cReadNak() << 8);
  HAL::i2cStop();
}
void uiCheckSlowKeys(uint16_t &action) {
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
  HAL::i2cWrite(0x12); // GPIOA
  HAL::i2cStop();
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_READ);
  uint16_t keymask = HAL::i2cReadAck();
  keymask = keymask + (HAL::i2cReadNak() << 8);
  HAL::i2cStop();
  UI_KEYS_I2C_BUTTON_LOW(4, UI_ACTION_PREVIOUS); // Up button
  UI_KEYS_I2C_BUTTON_LOW(8, UI_ACTION_NEXT); // down button
  UI_KEYS_I2C_BUTTON_LOW(16, UI_ACTION_BACK); // left button
  UI_KEYS_I2C_BUTTON_LOW(2, UI_ACTION_OK); // right button
  UI_KEYS_I2C_BUTTON_LOW(1, UI_ACTION_MENU_QUICKSETTINGS); //Select button
}
#endif
#endif // Controller 3

#if FEATURE_CONTROLLER == CONTROLLER_FOLTYN
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 2
#define UI_COLS 20
#define UI_ROWS 4
// PINK.1, 88, D_RS
#define UI_DISPLAY_RS_PIN		63
#define UI_DISPLAY_RW_PIN		-1
// PINK.3, 86, D_E
#define UI_DISPLAY_ENABLE_PIN	        65
// PINF.5, 92, D_D4
#define UI_DISPLAY_D0_PIN		59
// PINK.2, 87, D_D5
#define UI_DISPLAY_D1_PIN		64
// PINL.5, 40, D_D6
#define UI_DISPLAY_D2_PIN		44
// PINK.4, 85, D_D7
#define UI_DISPLAY_D3_PIN		66
// PINF.5, 92, D_D4
#define UI_DISPLAY_D4_PIN		59
// PINK.2, 87, D_D5
#define UI_DISPLAY_D5_PIN		64
// PINL.5, 40, D_D6
#define UI_DISPLAY_D6_PIN		44
// PINK.4, 85, D_D7
#define UI_DISPLAY_D7_PIN		66
#define UI_DELAYPERCHAR		   50
#define UI_INVERT_MENU_DIRECTION 0
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_BUTTON_LOW(4); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(5);
  UI_KEYS_INIT_BUTTON_LOW(6);
  UI_KEYS_INIT_BUTTON_LOW(11);
  UI_KEYS_INIT_BUTTON_LOW(42);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_BUTTON_LOW(4, UI_ACTION_OK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(5, UI_ACTION_NEXT); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(6, UI_ACTION_PREVIOUS); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(11, UI_ACTION_BACK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(42, UI_ACTION_SD_PRINT ); // push button, connects gnd to pin
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller 4


#if FEATURE_CONTROLLER == CONTROLLER_VIKI // Viki Lcd

// You need to change these 3 button according to the positions
// where you put them into your board!
#define UI_ENCODER_A      7
#define UI_ENCODER_B      22
#define UI_RESET_PIN      32
// Set to -1 if you have not connected that pin
#define SDCARDDETECT      49
#define SDSS              53

#undef SDSUPPORT
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED 0

#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_I2C
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 0xFFE0
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0x01C0 // bits that are high always, for now the 3 viki leds
#define UI_DISPLAY_I2C_PULLUP 0x001F
#define UI_I2C_CLOCKSPEED 100000L // Note with very long cables make this much smaller, for 2ft cables I found 80000 worked ok

#define UI_DISPLAY_RS_PIN _BV(15)
#define UI_DISPLAY_RW_PIN _BV(14)
#define UI_DISPLAY_ENABLE_PIN _BV(13)
#define UI_DISPLAY_D0_PIN _BV(12)
#define UI_DISPLAY_D1_PIN _BV(11)
#define UI_DISPLAY_D2_PIN _BV(10)
#define UI_DISPLAY_D3_PIN _BV(9)
#define UI_DISPLAY_D4_PIN _BV(12)
#define UI_DISPLAY_D5_PIN _BV(11)
#define UI_DISPLAY_D6_PIN _BV(10)
#define UI_DISPLAY_D7_PIN _BV(9)


#undef BEEPER_PIN
#define BEEPER_PIN        _BV(5)
#define BEEPER_TYPE       2
#define BEEPER_ADDRESS    UI_DISPLAY_I2C_ADDRESS // I2C address of the chip with the beeper pin
#define UI_I2C_HEATBED_LED    _BV(8)
#define UI_I2C_HOTEND_LED     _BV(7)
#define UI_I2C_FAN_LED        _BV(6)

#define UI_INVERT_MENU_DIRECTION 0
#define UI_HAS_I2C_KEYS
#define UI_HAS_I2C_ENCODER 0
#define UI_I2C_KEY_ADDRESS 0x40
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on real pins. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B); // click encoder on real pins
  UI_KEYS_BUTTON_LOW(UI_RESET_PIN, UI_ACTION_RESET);
}
inline void uiCheckSlowEncoder() { }// not used in Viki
void uiCheckSlowKeys(uint16_t &action) {
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
  HAL::i2cWrite(0x12); // GPIOA
  HAL::i2cStop();
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_READ);
  unsigned int keymask = HAL::i2cReadAck();
  keymask = keymask + (HAL::i2cReadNak() << 8);
  HAL::i2cStop();
  UI_KEYS_I2C_BUTTON_LOW(4, UI_ACTION_MENU_SDCARD);       // Up button
  UI_KEYS_I2C_BUTTON_LOW(8, UI_ACTION_MENU_QUICKSETTINGS); // down button
  UI_KEYS_I2C_BUTTON_LOW(16, UI_ACTION_BACK);             // left button
  UI_KEYS_I2C_BUTTON_LOW(2, UI_ACTION_MENU_POSITIONS);    // right button
  UI_KEYS_I2C_BUTTON_LOW(1, UI_ACTION_OK);                //Select button

}
#endif
#endif // Controller 5

#if FEATURE_CONTROLLER == CONTROLLER_MEGATRONIC
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 0
#define UI_COLS 20
#define UI_ROWS 2

#if MOTHERBOARD==701 // Megatronics v2.0
#define UI_DISPLAY_RS_PIN 14
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 15
#define UI_DISPLAY_D4_PIN 30
#define UI_DISPLAY_D5_PIN 31
#define UI_DISPLAY_D6_PIN 32
#define UI_DISPLAY_D7_PIN 33
#define UI_ENCODER_A 61
#define UI_ENCODER_B 59
#define UI_ENCODER_CLICK 43

#define UI_SHIFT_OUT 17
#define UI_SHIFT_LD 42
#define UI_SHIFT_CLK 63

#else // RAMPS 1.4
#define UI_DISPLAY_RS_PIN 16
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 17
#define UI_DISPLAY_D4_PIN 23
#define UI_DISPLAY_D5_PIN 25
#define UI_DISPLAY_D6_PIN 27
#define UI_DISPLAY_D7_PIN 29
#define UI_ENCODER_A 64
#define UI_ENCODER_B 59
#define UI_ENCODER_CLICK 63

#define UI_SHIFT_OUT 40
#define UI_SHIFT_LD 42
#define UI_SHIFT_CLK 44
#endif

#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION 1
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK);

  SET_OUTPUT(UI_SHIFT_CLK);
  SET_OUTPUT(UI_SHIFT_LD);
  SET_INPUT(UI_SHIFT_OUT);

  WRITE(UI_SHIFT_OUT, HIGH);
  WRITE(UI_SHIFT_LD, HIGH);
}

void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
}

inline void uiCheckSlowEncoder() {} // not used

void uiCheckSlowKeys(uint16_t &action) {

  WRITE(UI_SHIFT_LD, LOW);
  WRITE(UI_SHIFT_LD, HIGH);

  for (int8_t i = 1; i <= 8; i++) {
    if (!READ(UI_SHIFT_OUT)) { // pressed button = logical 0 (false)
      switch (i) {
        case 1: action = UI_ACTION_Z_DOWN; break; // F3
        case 2: action = UI_ACTION_Z_UP; break; // F2
        case 3: action = UI_ACTION_EMERGENCY_STOP; break; // F1
        case 4: action = UI_ACTION_Y_UP; break; // UP
        case 5: action = UI_ACTION_X_UP; break; // RIGHT
        case 6: action = UI_ACTION_HOME_ALL; break; // MID
        case 7: action = UI_ACTION_Y_DOWN; break; // DOWN
        case 8: action = UI_ACTION_X_DOWN; break; // LEFT
      }
      i = 9; // if button detected, exit "for loop"
    }
    WRITE(UI_SHIFT_CLK, HIGH);
    WRITE(UI_SHIFT_CLK, LOW);
  }
}
#endif
#endif // Controller 6
#if FEATURE_CONTROLLER == CONTROLLER_RADDS
#undef SDSS
#define SDSS            10
#undef SPI_PIN
#define SPI_PIN         77
#undef SPI_CHAN
#define SPI_CHAN        0
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define BEEPER_TYPE 1
#define UI_COLS 20
#define UI_ROWS 4
#undef BEEPER_PIN
#define BEEPER_PIN             41
#define UI_DISPLAY_RS_PIN      42
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  43
#define UI_DISPLAY_D0_PIN      44
#define UI_DISPLAY_D1_PIN      45
#define UI_DISPLAY_D2_PIN      46
#define UI_DISPLAY_D3_PIN      47
#define UI_DISPLAY_D4_PIN      44
#define UI_DISPLAY_D5_PIN      45
#define UI_DISPLAY_D6_PIN      46
#define UI_DISPLAY_D7_PIN      47
#define UI_ENCODER_A           50
#define UI_ENCODER_B           52
#define UI_ENCODER_CLICK       48
#define UI_RESET_PIN           -1
#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION 0
#define UI_BUTTON_BACK         71
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_BACK);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_BACK, UI_ACTION_BACK);
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller 7

#if FEATURE_CONTROLLER == CONTROLLER_PIBOT20X4 || FEATURE_CONTROLLER == CONTROLLER_PIBOT16X2

#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION 1
#define BEEPER_SHORT_SEQUENCE 6,2 // Needs longer beep sequence
#define BEEPER_LONG_SEQUENCE 24,8
#define BEEPER_TYPE 1
#define BEEPER_TYPE_INVERTING 0

#if FEATURE_CONTROLLER == CONTROLLER_PIBOT16X2
#define UI_COLS 16
#define UI_ROWS 2
#else  ////20x04 Display
#define UI_COLS 20
#define UI_ROWS 4
#endif

#ifdef PiBot_V_1_4
#undef BEEPER_PIN
#define BEEPER_PIN             31
#define UI_DISPLAY_RS_PIN      45
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  44
#define UI_DISPLAY_D0_PIN      43
#define UI_DISPLAY_D1_PIN      42
#define UI_DISPLAY_D2_PIN      19
#define UI_DISPLAY_D3_PIN      18
#define UI_DISPLAY_D4_PIN      43
#define UI_DISPLAY_D5_PIN      42
#define UI_DISPLAY_D6_PIN      19
#define UI_DISPLAY_D7_PIN      18
#define UI_ENCODER_A           61
#define UI_ENCODER_B           62
#define UI_ENCODER_CLICK       63
#define UI_RESET_PIN           28
#define UI_DELAYPERCHAR 50
#define UI_BUTTON_OK       49
#define UI_BUTTON_NEXT     48
#define UI_BUTTON_PREVIOUS 47
#define UI_BUTTON_BACK     46
#define UI_BUTTON_SD_PRINT 29
#endif

#if PiBot_V_1_4==true || PiBot_V_1_6==true
#undef BEEPER_PIN
#define BEEPER_PIN             37
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      23
#define UI_DISPLAY_D1_PIN      25
#define UI_DISPLAY_D2_PIN      27
#define UI_DISPLAY_D3_PIN      29
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_ENCODER_A           33
#define UI_ENCODER_B           31
#define UI_ENCODER_CLICK       35
#define UI_RESET_PIN           41
#define UI_DELAYPERCHAR 50
#define UI_BUTTON_OK       4
#define UI_BUTTON_NEXT     6
#define UI_BUTTON_PREVIOUS 5
#define UI_BUTTON_BACK     11
#define UI_BUTTON_SD_PRINT 42
#endif

#if PiBot_V_2_0
#undef BEEPER_PIN
#define BEEPER_PIN             16
#define UI_DISPLAY_RS_PIN      43
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  42
#define UI_DISPLAY_D0_PIN      19
#define UI_DISPLAY_D1_PIN      18
#define UI_DISPLAY_D2_PIN      38
#define UI_DISPLAY_D3_PIN      41
#define UI_DISPLAY_D4_PIN      19
#define UI_DISPLAY_D5_PIN      18
#define UI_DISPLAY_D6_PIN      38
#define UI_DISPLAY_D7_PIN      41

#define UI_ENCODER_A           37
#define UI_ENCODER_B           36
// Vick BTN
#define UI_ENCODER_CLICK       69
// if you want, you can get the CNC Pin used 11
#define UI_RESET_PIN           -1

#define UI_DELAYPERCHAR        320
#define UI_BUTTON_OK           47
#define UI_BUTTON_NEXT         46
#define UI_BUTTON_PREVIOUS     45
#define UI_BUTTON_BACK         44
// if you want, you can get the CNC Pin used 10
#define UI_BUTTON_SD_PRINT     70
#endif

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_OK); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_NEXT);
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_PREVIOUS);
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_BACK);
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_SD_PRINT);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_BUTTON_LOW(UI_BUTTON_OK, UI_ACTION_OK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_NEXT, UI_ACTION_NEXT); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_PREVIOUS, UI_ACTION_PREVIOUS); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_BACK, UI_ACTION_BACK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_SD_PRINT, UI_ACTION_SD_PRINT ); // push button, connects gnd to pin
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif

#if FEATURE_CONTROLLER == CONTROLLER_FELIX
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_ENCODER_SPEED 2
#undef BEEPER_TYPE
#define BEEPER_TYPE 0
#undef BEEPER_PIN
#define BEEPER_PIN             -1
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_ENCODER_A           35
#define UI_ENCODER_B           37
#define UI_ENCODER_CLICK       31
#define UI_DELAYPERCHAR 50
#define UI_INVERT_MENU_DIRECTION 0

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller 12

#if FEATURE_CONTROLLER == CONTROLLER_RAMBO // SeeMeCNC LCD + Rambo
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define BEEPER_TYPE 1
#undef BEEPER_PIN
#define BEEPER_PIN             79
#define UI_DISPLAY_RS_PIN      70
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  71
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      72
#define UI_DISPLAY_D5_PIN      73
#define UI_DISPLAY_D6_PIN      74
#define UI_DISPLAY_D7_PIN      75
#define UI_ENCODER_A           76
#define UI_ENCODER_B           77
#define UI_ENCODER_CLICK       78
#define UI_KILL_PIN            80
#define UI_DELAYPERCHAR       50
#define UI_INVERT_MENU_DIRECTION 0
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK);
  UI_KEYS_INIT_BUTTON_LOW(UI_KILL_PIN);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
  UI_KEYS_BUTTON_LOW(UI_KILL_PIN, UI_ACTION_KILL);
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller 13

#if FEATURE_CONTROLLER == CONTROLLER_OPENHARDWARE_LCD2004
#undef SDSUPPORT
#define SDSUPPORT 1
#define SDCARDDETECT -1
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_I2C
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 65504
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0
#define UI_DISPLAY_I2C_PULLUP 31
#define UI_I2C_CLOCKSPEED 400000L
#define UI_DISPLAY_RS_PIN _BV(15)
#define UI_DISPLAY_RW_PIN _BV(14)
#define UI_DISPLAY_ENABLE_PIN _BV(13)
#define UI_DISPLAY_D0_PIN _BV(12)
#define UI_DISPLAY_D1_PIN _BV(11)
#define UI_DISPLAY_D2_PIN _BV(10)
#define UI_DISPLAY_D3_PIN _BV(9)
#define UI_DISPLAY_D4_PIN _BV(12)
#define UI_DISPLAY_D5_PIN _BV(11)
#define UI_DISPLAY_D6_PIN _BV(10)
#define UI_DISPLAY_D7_PIN _BV(9)
#define UI_INVERT_MENU_DIRECTION false
#define UI_HAS_I2C_KEYS
#define UI_HAS_I2C_ENCODER 0
#define UI_I2C_KEY_ADDRESS 0x40

#ifdef UI_MAIN
void uiInitKeys() {}
void uiCheckKeys(uint16_t &action) {}
inline void uiCheckSlowEncoder() {
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
  HAL::i2cWrite(0x12); // GIOA
  HAL::i2cStop();
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_READ);
  uint16_t keymask = HAL::i2cReadAck();
  keymask = keymask + (HAL::i2cReadNak() << 8);
  HAL::i2cStop();
}
void uiCheckSlowKeys(uint16_t &action) {
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
  HAL::i2cWrite(0x12); // GPIOA
  HAL::i2cStop();
  HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_READ);
  uint16_t keymask = HAL::i2cReadAck();
  keymask = keymask + (HAL::i2cReadNak() << 8);
  HAL::i2cStop();
  UI_KEYS_I2C_BUTTON_LOW(_BV(4), UI_ACTION_OK); // push button, connects gnd to pin
  UI_KEYS_I2C_BUTTON_LOW(_BV(1), UI_ACTION_BACK); // push button, connects gnd to pin
  UI_KEYS_I2C_BUTTON_LOW(_BV(0), UI_ACTION_SD_PRINT); // push button, connects gnd to pin
  UI_KEYS_I2C_BUTTON_LOW(_BV(3), UI_ACTION_PREVIOUS); // Up button
  UI_KEYS_I2C_BUTTON_LOW(_BV(2), UI_ACTION_NEXT); // down button
}
#endif
#endif // Controller 14

/*
	Sanguinololu + panelolu2
*/
#if FEATURE_CONTROLLER == CONTROLLER_SANGUINOLOLU_PANELOLU2
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_I2C
#define UI_DISPLAY_CHARSET 2
#define UI_COLS 20
#define UI_ROWS 4
#define UI_INVERT_MENU_DIRECTION 0

#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 65528
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0
#define UI_DISPLAY_I2C_PULLUP 23
#define UI_I2C_CLOCKSPEED 100000L
//#define UI_HAS_I2C_KEYS
//#define UI_HAS_I2C_ENCODER 0
//#define UI_I2C_KEY_ADDRESS UI_DISPLAY_I2C_ADDRESS
#define BEEPER_TYPE 2
#define BEEPER_TYPE_INVERTING 1
#define BEEPER_ADDRESS UI_DISPLAY_I2C_ADDRESS
#define COMPILE_I2C_DRIVER

#define UI_DISPLAY_RS_PIN 		_BV(15)
#define UI_DISPLAY_RW_PIN 		_BV(14)
#define UI_DISPLAY_ENABLE_PIN 	_BV(13)
#define UI_DISPLAY_D0_PIN 		_BV(12)
#define UI_DISPLAY_D1_PIN 		_BV(11)
#define UI_DISPLAY_D2_PIN 		_BV(10)
#define UI_DISPLAY_D3_PIN 		_BV(9)
#define UI_DISPLAY_D4_PIN 		_BV(12)
#define UI_DISPLAY_D5_PIN 		_BV(11)
#define UI_DISPLAY_D6_PIN 		_BV(10)
#define UI_DISPLAY_D7_PIN 		_BV(9)
#undef BEEPER_PIN
#define BEEPER_PIN _BV(5)
#define UI_I2C_HEATBED_LED    _BV(8)
#define UI_I2C_HOTEND_LED     _BV(7)
#define UI_I2C_FAN_LED        _BV(6)

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(10, 11); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(30); // push button, connects gnd to pin
}

void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(10, 11); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(30, UI_ACTION_OK); // push button, connects gnd to pin
}

inline void uiCheckSlowEncoder() {}

void uiCheckSlowKeys(uint16_t &action) {}
#endif // UI_MAIN
#endif // Controller 15

#if FEATURE_CONTROLLER == CONTROLLER_GAMEDUINO2
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_GAMEDUINO2
#define UI_DISPLAY_CHARSET 0
#define UI_COLS 30
#define UI_ROWS 4
#define UI_INVERT_MENU_DIRECTION 0

#define BEEPER_TYPE 0
#define BEEPER_TYPE_INVERTING 0

#define UI_DISPLAY_CS 49  // Pin for SPI select on gameduino 2 - depends on board and choice

#endif // Controller 16
#if FEATURE_CONTROLLER == CONTROLLER_MIREGLI // Miregli
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_KILL_PIN            76
#define UI_ENCODER_A           80
#define UI_ENCODER_B           73
#define UI_ENCODER_CLICK       63
#define UI_DELAYPERCHAR 50
#define MIREGLI
#define SDCARDDETECT -1 //53
#define BEEPER 78
#define LCD_CONTRAST 62
#define UI_DISPLAY_TYPE 14
#define LCD_PIN_BL 15
#define DOGLCD_A0  38
#define DOGLCD_CS  14
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64
#define U8GLIB_ST7920

#define UI_INVERT_MENU_DIRECTION false

//select font size
#define UI_FONT_6X10 //default font
#ifdef UI_FONT_6X10
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#endif

#ifdef UI_MAIN
void ui_init_keys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK);
  UI_KEYS_INIT_BUTTON_LOW(UI_KILL_PIN);
}
void ui_check_keys(int &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
  UI_KEYS_BUTTON_LOW(UI_KILL_PIN, UI_ACTION_KILL);
}
inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif
#endif // Controller 17
#if FEATURE_CONTROLLER == CONTROLLER_GATE_3NOVATICA
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define BEEPER_TYPE 1
#undef BEEPER_PIN
#define BEEPER_PIN             -1
#define UI_DISPLAY_RS_PIN      1
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  3
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      0
#define UI_DISPLAY_D5_PIN      2
#define UI_DISPLAY_D6_PIN      4
#define UI_DISPLAY_D7_PIN      6
#define UI_ENCODER_A           5
#define UI_ENCODER_B           7
#define UI_ENCODER_CLICK       39
#define UI_KILL_PIN            -1
#define UI_DELAYPERCHAR       320 // bylo 50
#define UI_INVERT_MENU_DIRECTION 1 // bylo 0
#define USER_KEY1_PIN     36
#define USER_KEY1_ACTION  UI_ACTION_LIGHTS_ONOFF
#define USER_KEY2_PIN     40
#define USER_KEY2_ACTION  UI_ACTION_PREHEAT_ALL
#define USER_KEY3_PIN     41
#define USER_KEY3_ACTION  UI_ACTION_WIZARD_FILAMENTCHANGE
#define USER_KEY4_PIN     -1
#define USER_KEY4_ACTION  UI_ACTION_DUMMY

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B);
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // CONTROLLER_GATE_3NOVATICA

#if FEATURE_CONTROLLER == CONTROLLER_SPARKLCD || FEATURE_CONTROLLER == CONTROLLER_SPARKLCD_ADAPTER
#if MOTHERBOARD != 402 && MOTHERBOARD != 412
#error This config only works with RADDS motherboard!
#endif
#define UI_DISPLAY_CHARSET 3
#define UI_DISPLAY_TYPE 5
#define U8GLIB_ST7920 // Currently only this display from u8g lib is included.
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64

//select font size
#define UI_FONT_6X10 //default font
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#define UI_DELAYPERCHAR		  50
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_INVERT_MENU_DIRECTION 0
#define UI_HAS_I2C_ENCODER 0
#undef UI_ENCODER_SPEED
#define UI_ENCODER_SPEED 2

//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_D0_PIN		-1
#define UI_DISPLAY_D1_PIN		-1
#define UI_DISPLAY_D2_PIN		-1
#define UI_DISPLAY_D3_PIN		-1
#define UI_DISPLAY_D5_PIN		-1
#define UI_DISPLAY_D6_PIN		-1
#define UI_DISPLAY_D7_PIN		-1

#if FEATURE_CONTROLLER == CONTROLLER_SPARKLCD
#if MOTHERBOARD == 412 // STACKER 3d Superboard
// PINK.1, 88, D_RS
#define UI_DISPLAY_RS_PIN		29 // 29
#define UI_DISPLAY_RW_PIN		-1
// PINK.3, 86, D_E
#define UI_DISPLAY_ENABLE_PIN	27 //25
// PINF.5, 92, D_D4
// PINF.5, 92, D_D4
#define UI_DISPLAY_D4_PIN	25 //	27
#define UI_ENCODER_A 33
#define UI_ENCODER_B 35
#define UI_ENCODER_CLICK 37
#else //  MOTHERBOARD == 412
// PINK.1, 88, D_RS
#define UI_DISPLAY_RS_PIN		25
#define UI_DISPLAY_RW_PIN		-1
// PINK.3, 86, D_E
#define UI_DISPLAY_ENABLE_PIN	        27
// PINF.5, 92, D_D4
// PINF.5, 92, D_D4
#define UI_DISPLAY_D4_PIN		29
#define UI_ENCODER_A 35
#define UI_ENCODER_B 33
#define UI_ENCODER_CLICK 37
#endif
#else // FEATURE_CONTROLLER == CONTROLLER_SPARKLCD
// PINK.1, 88, D_RS
#define UI_DISPLAY_RS_PIN		44
#define UI_DISPLAY_RW_PIN		-1
// PINK.3, 86, D_E
#define UI_DISPLAY_ENABLE_PIN	       45
#define UI_DISPLAY_D4_PIN		46
#define UI_ENCODER_A 50
#define UI_ENCODER_B 52
#define UI_ENCODER_CLICK 48
#endif

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin;
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif

#endif // CONTROLLER_sparkLCD

#if FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2
#if MOTHERBOARD != 402
#error This config only works with RADDS motherboard!
#endif
#define UI_DISPLAY_CHARSET 3
#define UI_DISPLAY_TYPE 5
#define U8GLIB_ST7920 // Currently only this display from u8g lib is included.
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64

//select font size
#define UI_FONT_6X10 //default font
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#undef UI_ANIMATION
#define UI_ANIMATION 0  // Animations are too slow
#define UI_DELAYPERCHAR		  50
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_INVERT_MENU_DIRECTION 0
#define UI_HAS_I2C_ENCODER 0
#define UI_ENCODER_SPEED 1

//SD Card
#undef SDSS
#define SDSS            10
#undef SPI_PIN
#define SPI_PIN         77
#undef SPI_CHAN
#define SPI_CHAN        0

//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_D0_PIN		-1
#define UI_DISPLAY_D1_PIN		-1
#define UI_DISPLAY_D2_PIN		-1
#define UI_DISPLAY_D3_PIN		-1
#define UI_DISPLAY_D5_PIN		-1
#define UI_DISPLAY_D6_PIN		-1
#define UI_DISPLAY_D7_PIN		-1

#define UI_DISPLAY_RS_PIN		44
#define UI_DISPLAY_RW_PIN		-1
#define UI_DISPLAY_ENABLE_PIN	       45
#define UI_DISPLAY_D4_PIN		46
#define UI_ENCODER_A 50
#define UI_ENCODER_B 52
#define UI_ENCODER_CLICK 48

#ifdef UI_MAIN
void uiInitKeys() {
    UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
    UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin;
}
void uiCheckKeys(uint16_t &action) {
    UI_KEYS_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
    UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif

#endif // CONTROLLER_ORCABOTXXLPRO2


#if FEATURE_CONTROLLER == CONTROLLER_VIKI2
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_U8G
//#define U8GLIB_ST7920
#define U8GLIB_ST7565_NHD_C2832_HW_SPI
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64
//select font size
#define UI_FONT_6X10 //default font
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display

//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_CHARSET 3
#define UI_INVERT_MENU_DIRECTION 0
#undef UI_ENCODER_SPEED
#define UI_ENCODER_SPEED 2
#define SDCARDDETECT        -1
#define UI_DISPLAY_RW_PIN -1
#define UI_ROTATE_180

#define BEEPER_TYPE 1

// SCK Pin:  UI_DISPLAY_D4_PIN
// Mosi Pin: UI_DISPLAY_ENABLE_PIN
// CD Pin:   UI_DISPLAY_RS_PIN

#if MOTHERBOARD == 33 // RAMPS

#define SDCARDDETECT 49 // sd card detect as shown on drawing
#undef BEEPER_PIN
#define BEEPER_PIN    35
// Display A0
#define UI_DISPLAY_D5_PIN 47
// Display CS
#define UI_DISPLAY_RS_PIN 32
#define UI_ENCODER_A 45
#define UI_ENCODER_B 41
#define UI_ENCODER_CLICK 43
#define UI_RESET_PIN -1
#define RED_BLUE_STATUS_LEDS
#define RED_STATUS_LED 39
#define BLUE_STATUS_LED 37
#elif MOTHERBOARD == 34 // Azteeg X3

#define SDCARDDETECT 49 // sd card detect as shown on drawing
#undef BEEPER_PIN
#define BEEPER_PIN    33
// Display A0
#define UI_DISPLAY_D5_PIN 31
// Display CS
#define UI_DISPLAY_RS_PIN 32
#define UI_ENCODER_A 22
#define UI_ENCODER_B 7
#define UI_ENCODER_CLICK 12
#define UI_RESET_PIN -1
#define RED_BLUE_STATUS_LEDS
#define RED_STATUS_LED 64
#define BLUE_STATUS_LED 63

#elif MOTHERBOARD == 35 // Azteeg X3 Pro

#undef SDCARDDETECT
// sd card detect as shown on drawing
#define SDCARDDETECT      49
#undef BEEPER_PIN
// 33 is the on board beeper
#define BEEPER_PIN        47
// Display A0
#define UI_DISPLAY_D5_PIN 44
// Display CS
#define UI_DISPLAY_RS_PIN 45
#define UI_ENCODER_A      22
#define UI_ENCODER_B       7
#define UI_ENCODER_CLICK  39
#define UI_RESET_PIN      -1
#define RED_BLUE_STATUS_LEDS
#define RED_STATUS_LED    32
#define BLUE_STATUS_LED   35

#elif MOTHERBOARD == 301 // RAMBO

#define SDCARDDETECT 72 // sd card detect as shown on drawing
#undef BEEPER_PIN
#define BEEPER_PIN         33
// Display A0
#define UI_DISPLAY_D5_PIN 70
// Display CS
#define UI_DISPLAY_RS_PIN 71
#define UI_ENCODER_A 85
#define UI_ENCODER_B 84
#define UI_ENCODER_CLICK 83
#define UI_RESET_PIN -1
#define RED_BLUE_STATUS_LEDS
#define RED_STATUS_LED 22
#define BLUE_STATUS_LED 32

#elif MOTHERBOARD == 9 || MOTHERBOARD == 92 // Printboard
// sd card detect as shown on drawing
#define SDCARDDETECT 72
#define SDSS          45
#undef BEEPER_PIN
#define BEEPER_PIN         32
// Display A0
#define UI_DISPLAY_D5_PIN 42
// Display CS
#define UI_DISPLAY_RS_PIN 43
#define UI_ENCODER_A 26
#define UI_ENCODER_B 27
#define UI_ENCODER_CLICK 47
#define UI_RESET_PIN -1
#define RED_BLUE_STATUS_LEDS
#define RED_STATUS_LED 12
#define BLUE_STATUS_LED 10

#elif MOTHERBOARD == 402 // RADDS

#undef SDCARDDETECT
#define SDCARDDETECT 14 // sd card detect as shown on drawing
//#undef SDSS
//#define SDSS          4
//#define SPI_PIN         87
//#define SPI_CHAN        1
/*#define SDSS            10
  #undef SPI_PIN
  #define SPI_PIN         77
  #undef SPI_CHAN
  #define SPI_CHAN        0

  #undef SDSUPPORT
  #define SDSUPPORT  0 // sd card does not work reliable due to spi sharing
*/

#undef BEEPER_PIN
#define BEEPER_PIN         41
// Hardware SPI creates artifacts on display, so we use software SPI
#undef U8GLIB_ST7565_NHD_C2832_HW_SPI
#define U8GLIB_ST7565_NHD_C2832_SW_SPI
// MOSI 43
#define UI_DISPLAY_ENABLE_PIN 31
//#define UI_DISPLAY_ENABLE_PIN 75
//76 // SCK pin
#define UI_DISPLAY_D4_PIN  33 //44
// Display A0 => LCD RS
#define UI_DISPLAY_D5_PIN 42
// Display CS => CS0 //4 //10
#define UI_DISPLAY_RS_PIN 35 //10
#define UI_ENCODER_A 50
#define UI_ENCODER_B 52
#define UI_ENCODER_CLICK 48
#define UI_RESET_PIN -1
#define RED_BLUE_STATUS_LEDS
// PWM2 Pin
#define RED_STATUS_LED 6
// PWM1 Pin
#define BLUE_STATUS_LED 5

#else
#error No predefined Viki 2 mapping for your board available
#endif

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
#if UI_RESET_PIN > -1
  UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
#endif
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_B, UI_ENCODER_A);
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
#if UI_RESET_PIN > -1
  UI_KEYS_BUTTON_LOW(UI_RESET_PIN, UI_ACTION_RESET);
#endif
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller VIKI 2

#if FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864 || FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864_OLED
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_U8G
//#define U8GLIB_ST7920
//#define U8GLIB_ST7565_NHD_C2832_HW_SPI
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64
//select font size
#define UI_FONT_6X10 //default font
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#undef UI_ANIMATION
#define UI_ANIMATION 0  // Animations are too slow

//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_CHARSET 3
#define UI_INVERT_MENU_DIRECTION 0
#undef UI_ENCODER_SPEED
#define UI_ENCODER_SPEED 2
//#define SDCARDDETECT        -1
//#define UI_DISPLAY_RW_PIN -1
#if FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864
#define UI_ROTATE_180
#endif



#define BEEPER_TYPE 1

// SCK Pin:  UI_DISPLAY_D4_PIN
// Mosi Pin: UI_DISPLAY_ENABLE_PIN
// CD Pin:   UI_DISPLAY_RS_PIN

#if MOTHERBOARD == 408 || MOTHERBOARD == 413 // SMART RAMPS

#undef SDCARDDETECT
#define SDCARDDETECT 49 // sd card detect as shown on drawing


#undef BEEPER_PIN
#define BEEPER_PIN         66
// Hardware SPI creates artifacts on display, so we use software SPI
#undef U8GLIB_ST7565_NHD_C2832_HW_SPI
#define U8GLIB_ST7565_NHD_C2832_SW_SPI
#define LCD_CONTRAST 62
// MOSI 43
#define UI_DISPLAY_ENABLE_PIN 51
//76 // SCK pin
#define UI_DISPLAY_D4_PIN  52 //44
// Display A0 => LCD RS
#define UI_DISPLAY_D5_PIN 59
// Display CS => CS0 //4 //10
#define UI_DISPLAY_RS_PIN 44 //10
#define UI_ENCODER_A 58
#define UI_ENCODER_B 40
#define UI_ENCODER_CLICK 67
#define UI_RESET_PIN 42
#else
#error No predefined AZSMZ_12864 mapping for your board available
#endif

#ifdef UI_MAIN
void uiInitKeys() {
    UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
    UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
    #if UI_RESET_PIN > -1
    UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
    #endif
}
void uiCheckKeys(uint16_t &action) {
    UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A, UI_ENCODER_B);
    UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK);
    #if UI_RESET_PIN > -1
    UI_KEYS_BUTTON_LOW(UI_RESET_PIN, UI_ACTION_RESET);
    #endif
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Controller AZSMZ_12864

#if (FEATURE_CONTROLLER == CONTROLLER_FYSETC_MINI_12864_V21)
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE DISPLAY_U8G
#define U8GLIB_MINI12864_2X_HW_SPI
#define UI_LCD_WIDTH 128
#define UI_LCD_HEIGHT 64
//select font size
#define UI_FONT_6X10 //default font
#ifdef UI_FONT_6X10
#define UI_FONT_WIDTH 6
#define UI_FONT_HEIGHT 10
#define UI_FONT_SMALL_HEIGHT 7
#define UI_FONT_DEFAULT repetier_6x10
#define UI_FONT_SMALL repetier_5x7
#define UI_FONT_SMALL_WIDTH 5 //smaller font for status display
#endif
//calculate rows and cols available with current font
#define UI_COLS (UI_LCD_WIDTH/UI_FONT_SMALL_WIDTH)
#define UI_ROWS (UI_LCD_HEIGHT/UI_FONT_HEIGHT)
#define UI_DISPLAY_CHARSET 3

#define BEEPER_TYPE 1

#if MOTHERBOARD == 190  // Fysetc F6
#define BEEPER_PIN             37
#define UI_DISPLAY_RESET_PIN   23
#define UI_DISPLAY_RS_PIN      17
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  51
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      52
#define UI_DISPLAY_D5_PIN      16
#define UI_DISPLAY_D6_PIN      -1
#define UI_DISPLAY_D7_PIN      -1
#define UI_ENCODER_A           33
#define UI_ENCODER_B           31
#define UI_ENCODER_CLICK       35
#define UI_RESET_PIN           41
#define LCD_CONTRAST           255
#define UI_INVERT_MENU_DIRECTION   0
#define UI_ENCODER_SPEED 2

#else

// Untested configuration for the other motherboards. Please adapt the pin mappings.
// The board connectors and pin mappings could be compatible with e.g. RAMPS but needs testing.
#error The Fysetc Mini 12864 pin mappings must be adapted to your motherboard in the DisplayList.h

#endif

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
#if UI_RESET_PIN > -1
  UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
#endif
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
#if UI_RESET_PIN > -1
  UI_KEYS_BUTTON_LOW(UI_RESET_PIN, UI_ACTION_RESET);
#endif
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
#endif // Fysetc Mini 12864 v2.1 Panel

#if FEATURE_CONTROLLER == CONTROLLER_LCD_MP_PHARAOH_DUE
#define UI_DISPLAY_TYPE 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_INVERT_MENU_DIRECTION 1
#define UI_DISPLAY_CHARSET 1
#define UI_DISPLAY_RS_PIN		42		// PINK.1, 88, D_RS
#define UI_DISPLAY_RW_PIN		-1
#define UI_DISPLAY_ENABLE_PIN	43		// PINK.3, 86, D_E
#define UI_DISPLAY_D0_PIN		44		// PINF.5, 92, D_D4
#define UI_DISPLAY_D1_PIN		45		// PINK.2, 87, D_D5
#define UI_DISPLAY_D2_PIN		46		// PINL.5, 40, D_D6
#define UI_DISPLAY_D3_PIN		47		// PINK.4, 85, D_D7
#define UI_DISPLAY_D4_PIN		44		// PINF.5, 92, D_D4
#define UI_DISPLAY_D5_PIN		45		// PINK.2, 87, D_D5
#define UI_DISPLAY_D6_PIN		46		// PINL.5, 40, D_D6
#define UI_DISPLAY_D7_PIN		47		// PINK.4, 85, D_D7
#define UI_DELAYPERCHAR		   50

#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_BUTTON_LOW(33); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(31);
  UI_KEYS_INIT_BUTTON_LOW(29);
  UI_KEYS_INIT_BUTTON_LOW(37);
  UI_KEYS_INIT_BUTTON_LOW(35);
  UI_KEYS_INIT_BUTTON_LOW(X_MIN_PIN);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_BUTTON_LOW(33, UI_ACTION_OK); //35 push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(35, UI_ACTION_PREVIOUS); //34 push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(31, UI_ACTION_NEXT); //43 push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(29, UI_ACTION_BACK); //44 push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(37, UI_ACTION_MENU_SDCARD ); //33 push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(X_MIN_PIN, UI_ACTION_RESET /*UI_ACTION_PAUSE*/);
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}

#endif
#endif // CONTROLLER_LCD_MP_PHARAOH_DUE

#if FEATURE_CONTROLLER == CONTROLLER_ZONESTAR

// Keypad
#if !defined(ADC_KEYPAD_PIN) || (ADC_KEYPAD_PIN < 0)
#error CONTROLLER_ZONESTAR requres ADC_KEYPAD_PIN = 1 defined in Configuration.h
#endif

// This must be defined in the Configuration.h since used in ADC tables
//#define ADC_KEYPAD_PIN         1    // A1 (D30, analog numbering)

// Display
// Define UI_DISPLAY_TYPE = DISPLAY_SR with pins to override default settings
// that work for original Zonestar hardware.
// For instance:
//   #define UI_DISPLAY_TYPE        DISPLAY_SR
//   #define UI_DISPLAY_DATA_PIN    29
//   #define UI_DISPLAY_CLOCK_PIN   28
//   #define UI_DISPLAY_ENABLE_PIN  -1 // for 2-wire or pin number for 3-wire
#undef UI_DISPLAY_TYPE
#define UI_DISPLAY_TYPE          DISPLAY_4BIT

#if MOTHERBOARD == 39 // ZRIB
#define BEEPER_TYPE            1
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_RESET_PIN           41

#elif MOTHERBOARD == 703 // MEGATRONICS 3

#define UI_DISPLAY_RS_PIN      32
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  31
#define UI_DISPLAY_D4_PIN      14
#define UI_DISPLAY_D5_PIN      30
#define UI_DISPLAY_D6_PIN      39
#define UI_DISPLAY_D7_PIN      15
#define BEEPER_TYPE            1

#elif MOTHERBOARD == 63 // Melzi

#define UI_DISPLAY_ENABLE_PIN    29
#define UI_DISPLAY_RS_PIN        28
#define UI_DISPLAY_RW_PIN        -1
#define UI_DISPLAY_D4_PIN        10
#define UI_DISPLAY_D5_PIN        11
#define UI_DISPLAY_D6_PIN        16
#define UI_DISPLAY_D7_PIN        17
#else
#error Unknown display - board combination. Please add your pin mapping in DisplayList.h
#endif

#define UI_DISPLAY_CHARSET       1
#define UI_COLS                  20
#define UI_ROWS                  4

// UI
#define UI_HAS_KEYS              1
#define UI_HAS_BACK_KEY          1
#define UI_DELAYPERCHAR          50
#define UI_INVERT_MENU_DIRECTION 1

// Opportunity to override the Enter key via Configuration.h
// By default it duplicates the Right key, but could be set to
// anything else, e.g. UI_ACTION_TOP_MENU.
#ifndef ADC_KEYPAD_CENTER_ACTION
#define ADC_KEYPAD_CENTER_ACTION UI_ACTION_OK
#endif

#ifdef UI_MAIN
// Nothing to init since ADC is read in a loop if ADC_KEYPAD_PIN > -1
inline void uiInitKeys() {}

// Read and decode ADC keypad (fast reads)
void uiCheckKeys(uint16_t &action) {
	struct {
		uint16_t min;
		uint16_t max;
		uint16_t action;
		} keys[] = {
		{   300,   500, UI_ACTION_BACK           },    // Left
		{   570,   870, UI_ACTION_PREVIOUS       },    // Up
		{  1150,  1450, ADC_KEYPAD_CENTER_ACTION },    // Center
		{  1900,  2200, UI_ACTION_OK             },    // Right
		{  2670,  2870, UI_ACTION_NEXT           }     // Down
	};
	const uint8_t numOfKeys = sizeof(keys) / sizeof(keys[0]);

	extern volatile uint16_t osAnalogInputValues[ANALOG_INPUTS];
	uint16_t adc = osAnalogInputValues[KEYPAD_ANALOG_INDEX] >> (ANALOG_REDUCE_BITS);
	if (adc < 4000) {
		for (int8_t i = 0; i < numOfKeys; ++i) {
			if ((adc > keys[i].min) && (adc < keys[i].max)) {
				action = keys[i].action;
				return;
			}
		}
	}
}

// Read and decode ADC keypad (slow reads)
inline void uiCheckSlowKeys(uint16_t &action) {}
#endif // UI_MAIN

#endif // CONTROLLER_ZONESTAR

#ifndef UI_HAS_I2C_ENCODER
#define UI_HAS_I2C_ENCODER 0
#endif


#if FEATURE_CONTROLLER != NO_CONTROLLER
#if UI_ROWS==4
#if UI_COLS==16
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 4x16
#elif UI_COLS==20
//#define UI_LINE_OFFSETS {0,0x20,0x40,0x60} // 4x20 with KS0073
#define UI_LINE_OFFSETS {0,0x40,0x14,0x54} // 4x20 with HD44780
#else
#if UI_DISPLAY_TYPE!=DISPLAY_GAMEDUINO2
#error Unknown combination off rows/columns - define UI_LINE_OFFSETS manually.
#else
#define UI_LINE_OFFSETS {} // dummy never used
#endif
#endif
#else
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 2x16, 2x20, 2x24
#endif
#include "uilang.h"
#endif

#define UI_VERSION_STRING "Repetier " REPETIER_VERSION

#ifdef UI_HAS_I2C_KEYS
#define COMPILE_I2C_DRIVER
#endif

#if UI_DISPLAY_TYPE != NO_DISPLAY


#if UI_DISPLAY_TYPE == DISPLAY_I2C
#define COMPILE_I2C_DRIVER
#endif

#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 1
#else
#define UI_TEMP_PRECISION 0
#endif
#endif

#define UI_INITIALIZE uid.initialize();
#define UI_FAST if((counterPeriodical & 3) == 3) {uid.fastAction();}
#define UI_MEDIUM uid.mediumAction();
#define UI_SLOW(allowMoves) uid.slowAction(allowMoves);
#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_F(status) uid.setStatusP(status);
#define UI_STATUS_UPD(status) {uid.setStatusP(PSTR(status));uid.refreshPage();}
#define UI_STATUS_UPD_F(status) {uid.setStatusP(status);uid.refreshPage();}
#define UI_STATUS_RAM(status) uid.setStatus(status);
#define UI_STATUS_UPD_RAM(status) {uid.setStatus(status);uid.refreshPage();}
#define UI_PROGRESS_UPD(percent, eta) uid.setProgress(percent, eta);
#define UI_ERROR(status) uid.setStatusP(PSTR(status),true);
#define UI_ERROR_P(status) uid.setStatusP(status,true);
#define UI_ERROR_UPD(status) {uid.setStatusP(PSTR(status),true);uid.refreshPage();}
#define UI_ERROR_RAM(status) uid.setStatus(status,true);
#define UI_ERROR_UPD_RAM(status) {uid.setStatus(status,true);uid.refreshPage();}
//#define UI_ERROR(msg) {uid.errorMsg=(void*)PSTR(msg);pushMenu((void*)&ui_menu_error,true);}
#define UI_CLEAR_STATUS {uid.statusMsg[0]=0;}
#define UI_RESET_MENU {uid.menuLevel=0;uid.refreshPage();}
#define UI_MESSAGE(menu) {uid.showMessage(menu);}
#define UI_ACTION(ac) {uid.executeAction(ac,true);}
#else
#define UI_INITIALIZE {}
#define UI_FAST {}
#define UI_MEDIUM {}
#define UI_SLOW(allowMoves) {}
#define UI_STATUS(status) {}
#define UI_STATUS_F(status) {}
#define UI_STATUS_RAM(status) {}
#define UI_STATUS_UPD(status) {}
#define UI_STATUS_UPD_F(status) {}
#define UI_STATUS_UPD_RAM(status) {}
#define UI_PROGRESS_UPD(percent, eta) {}
#define UI_CLEAR_STATUS {}
#define UI_ERROR(msg) {}
#define UI_ERROR_P(status) {}
#define UI_ERROR_UPD(status) {}
#define UI_ERROR_RAM(status) {}
#define UI_ERROR_UPD_RAM(status) {}
#define UI_RESET_MENU {}
#define UI_MESSAGE(menu) {}
#define UI_ACTION(ac)
#endif  // Display

// Beeper methods
#if BEEPER_TYPE==0 || FEATURE_BEEPER == 0
#define BEEP_SHORT {}
#define BEEP_LONG {}
#else
#define BEEP_SHORT beep(BEEPER_SHORT_SEQUENCE);
#define BEEP_LONG beep(BEEPER_LONG_SEQUENCE);
#endif


extern void beep(uint8_t duration, uint8_t count);
#ifdef UI_MAIN
#if (defined(USER_KEY1_PIN) && USER_KEY1_PIN > -1 && defined(USER_KEY1_ACTION)) || (defined(USER_KEY2_PIN) && USER_KEY2_PIN > -1 && defined(USER_KEY2_ACTION)) || (defined(USER_KEY3_PIN) && USER_KEY3_PIN > -1 && defined(USER_KEY3_ACTION)) || (defined(USER_KEY4_PIN) && USER_KEY4_PIN > -1 && defined(USER_KEY4_ACTION))
#define HAS_USER_KEYS
static void ui_check_Ukeys(uint16_t &action) {
    #if defined(USER_KEY1_PIN) && USER_KEY1_PIN > -1 && defined(USER_KEY1_ACTION)
    UI_KEYS_BUTTON_LOW(USER_KEY1_PIN, USER_KEY1_ACTION);
    #endif
    #if defined(USER_KEY2_PIN) && USER_KEY2_PIN > -1 && defined(USER_KEY2_ACTION)
    UI_KEYS_BUTTON_LOW(USER_KEY2_PIN, USER_KEY2_ACTION);
    #endif
    #if defined(USER_KEY3_PIN) && USER_KEY3_PIN > -1 && defined(USER_KEY3_ACTION)
    UI_KEYS_BUTTON_LOW(USER_KEY3_PIN, USER_KEY3_ACTION);
    #endif
    #if defined(USER_KEY4_PIN) && USER_KEY4_PIN > -1 && defined(USER_KEY4_ACTION)
    UI_KEYS_BUTTON_LOW(USER_KEY4_PIN, USER_KEY4_ACTION);
    #endif
}
#endif
#endif

#endif
