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

// Minitronics V2.0   SAMD21

#if MOTHERBOARD == MOTHERBOARD_MINITRONICS_2_0

#ifndef __SAMD21__
#error oops !Be sure to have 'Minitronics' selected from the 'tools-> Boards menu'.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Minitronics pin assignments
******************************************************************/

#define ORIG_X_STEP_PIN 1
#define ORIG_X_DIR_PIN 3
#define ORIG_X_MIN_PIN -1
#define ORIG_X_MAX_PIN 8
#define ORIG_X_ENABLE_PIN 0

#define ORIG_Y_STEP_PIN 29
#define ORIG_Y_DIR_PIN 28
#define ORIG_Y_MIN_PIN -1
#define ORIG_Y_MAX_PIN 9
#define ORIG_Y_ENABLE_PIN 0

#define ORIG_Z_STEP_PIN 16
#define ORIG_Z_DIR_PIN 17
#define ORIG_Z_MIN_PIN -1
#define ORIG_Z_MAX_PIN 4
#define ORIG_Z_ENABLE_PIN 0
//Note that in due A0 pins on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 11
// Because analog pin #
#define TEMP_0_PIN 4
#define HEATER_1_PIN 6
// Because analog pin #
#define TEMP_1_PIN 3
#define HEATER_2_PIN 10
// Because analog pin #
#define TEMP_2_PIN 2

#define ORIG_E0_STEP_PIN 14
#define ORIG_E0_DIR_PIN 15
#define ORIG_E0_ENABLE_PIN 0

#define ORIG_E1_STEP_PIN 20
#define ORIG_E1_DIR_PIN 13
#define ORIG_E1_ENABLE_PIN 21

#define SDPOWER -1
#define SDSS 2
#define LED_PIN 13
#define ORIG_FAN_PIN 24
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN -1 // Pin that has to be turned right after the start, to keep the power flowing.

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define SDA_PIN 20
#define SCL_PIN 21

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

#ifndef CUSTOM_CONTROLLER_PINS

#if FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD || FEATURE_CONTROLLER == CONTROLLER_SMARTRAMPS

#define UI_DISPLAY_RS_PIN 18 //CS
#define UI_DISPLAY_ENABLE_PIN MOSI_PIN
#define UI_DISPLAY_D4_PIN SCK_PIN
#define UI_DISPLAY_D5_PIN -1 //A0 LCD RS
#define UI_DISPLAY_D6_PIN -1
#define UI_DISPLAY_D7_PIN -1
#define BEEPER_PIN -1
#define UI_ENCODER_A 27
#define UI_ENCODER_B 26
#define UI_ENCODER_CLICK 23
#define UI_RESET_PIN 53
#define UI_DELAYPERCHAR 50
#define UI_BUTTON_BACK -1
#undef SDSS
#define SDSS 2
#undef SDCARDDETECT
#define SDCARDDETECT 22
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0
#undef SDSUPPORT
#define SDSUPPORT 1

#endif

#endif
