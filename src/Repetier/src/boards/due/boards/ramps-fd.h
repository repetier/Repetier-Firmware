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

/****************************************************************************/
// RAMPS-FD Board
//
#if MOTHERBOARD == 403 || MOTHERBOARD == 404
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#if MOTHERBOARD == 403
#define HEATER_PINS_INVERTED 1 // only old boards had the output inverted
#else
#define HEATER_PINS_INVERTED 0
#endif

/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN 63
#define ORIG_X_DIR_PIN 62
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 48

#define ORIG_Y_STEP_PIN 65
#define ORIG_Y_DIR_PIN 64
#define ORIG_Y_MIN_PIN 24
#define ORIG_Y_MAX_PIN 38
#define ORIG_Y_ENABLE_PIN 46

#define ORIG_Z_STEP_PIN 67
#define ORIG_Z_DIR_PIN 66
#define ORIG_Z_MIN_PIN 26
#define ORIG_Z_MAX_PIN 34
#define ORIG_Z_ENABLE_PIN 44

// Caution - Heater 0 and 1 are likely reversed compared with other boards,
// so you might need to assign HEATER_0_PIN to the heated bed.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 8
// Due analog pin #54
#define TEMP_0_PIN 7

#define HEATER_1_PIN 9
// Due analog pin #55
#define TEMP_1_PIN 6

#define HEATER_2_PIN 10
// Due analog pin #56
#define TEMP_2_PIN 5

#define HEATER_3_PIN 11
// Due analog pin #57
#define TEMP_3_PIN 4

// Due analog pin #58
#define TEMP_4_PIN 3

#define ORIG_E0_STEP_PIN 36
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 42

#define ORIG_E1_STEP_PIN 43
#define ORIG_E1_DIR_PIN 41
#define ORIG_E1_ENABLE_PIN 39

#define ORIG_E2_STEP_PIN 32
#define ORIG_E2_DIR_PIN 47
#define ORIG_E2_ENABLE_PIN 45

//#define SDSUPPORT      false
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
//#define SDSS		   -1
//#define ORIG_SDCARDDETECT   -1
#define SDCARDDETECTINVERTED false
#define LED_PIN -1
#define ORIG_FAN_PIN 12
#define ORIG_FAN2_PIN 2
#define ORIG_PS_ON_PIN 53
#define KILL_PIN -1
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#ifndef EEPROM_AVAILABLE         // User can override eeprom usage
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 1
#endif

#endif
