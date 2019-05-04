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

/*****************************************************************
* DUE3DOM Board
* http://www.due3dom.pl
******************************************************************/
#if MOTHERBOARD == 410
#ifndef __SAM3X8E__
#error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 2
#define ORIG_X_DIR_PIN 3
#define ORIG_X_MIN_PIN 38
#define ORIG_X_MAX_PIN 36
#define ORIG_X_ENABLE_PIN 22

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 34
#define ORIG_Y_MAX_PIN 32
#define ORIG_Y_ENABLE_PIN 26

#define ORIG_Z_STEP_PIN 61
#define ORIG_Z_DIR_PIN 60
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 15

#define ORIG_E0_STEP_PIN 64
#define ORIG_E0_DIR_PIN 63
#define ORIG_E0_ENABLE_PIN 62

#define ORIG_E1_STEP_PIN 51
#define ORIG_E1_DIR_PIN 53
#define ORIG_E1_ENABLE_PIN 65

#define ORIG_E2_STEP_PIN 24
#define ORIG_E2_DIR_PIN 23
#define ORIG_E2_ENABLE_PIN 49

// hotend1 heater
#define HEATER_0_PIN 7
// bed heater
#define HEATER_1_PIN 39
// hotend2 heater
#define HEATER_2_PIN 8

// hotend1 thermistor
#define TEMP_0_PIN 7
// bed thermistor
#define TEMP_1_PIN 6
// hotend2 thermistor
#define TEMP_2_PIN 5
// thermo fan thermistor
#define TEMP_3_PIN 2
#define THERMOCOUPLE_0_PIN 3
#define THERMOCOUPLE_1_PIN 4

// print fan
#define ORIG_FAN_PIN 11
// hotend1 cooler
#define ORIG_FAN2_PIN 9
// hotend2 cooler / thermo fan / board fan
#define FAN_THERMO_PIN 12

#define SDSUPPORT 1
#define SDPOWER -1
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1

#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN -1

#define SDA_PIN 20
#define SCL_PIN 21

//servo pins 5, 6, 13

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif
#endif
//End DUE3DOM Board

/*****************************************************************
* DUE3DOM MINI Board
* http://www.due3dom.pl
******************************************************************/
#if MOTHERBOARD == 411
#ifndef __SAM3X8E__
#error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 17
#define ORIG_X_DIR_PIN 16
#define ORIG_X_MIN_PIN 38
// on expansion port
#define ORIG_X_MAX_PIN 36
#define ORIG_X_ENABLE_PIN 22

#define ORIG_Y_STEP_PIN 2
#define ORIG_Y_DIR_PIN 3
#define ORIG_Y_MIN_PIN 34
// on expansion port
#define ORIG_Y_MAX_PIN 32
#define ORIG_Y_ENABLE_PIN 26

#define ORIG_Z_STEP_PIN 64
#define ORIG_Z_DIR_PIN 63
#define ORIG_Z_MIN_PIN 30
// on expansion port
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 15

#define ORIG_E0_STEP_PIN 61
#define ORIG_E0_DIR_PIN 60
#define ORIG_E0_ENABLE_PIN 62

// on expansion port
#define ORIG_E1_STEP_PIN -1
// on expansion port
#define ORIG_E1_DIR_PIN -1
// on expansion port
#define ORIG_E1_ENABLE_PIN -1

// on expansion port
#define ORIG_E2_STEP_PIN -1
// on expansion port
#define ORIG_E2_DIR_PIN -1
// on expansion port
#define ORIG_E2_ENABLE_PIN -1

// hotend1 heater
#define HEATER_0_PIN 13
// bed heater
#define HEATER_1_PIN 7
// on expansion port
#define HEATER_2_PIN -1

// hotend1 thermistor
#define TEMP_0_PIN 7
// bed thermistor
#define TEMP_1_PIN 6
// thermo fan thermistor
#define TEMP_2_PIN 5
// onboard thermistor NTC100K Beta3950
#define TEMP_3_PIN 2
// on expansion port
#define THERMOCOUPLE_0_PIN 3
// on expansion port
#define THERMOCOUPLE_1_PIN 4

#define SDSUPPORT 1
#define SDPOWER -1
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1

// hotend1 cooler
#define ORIG_FAN_PIN 9
// print fan
#define ORIG_FAN2_PIN 11
// thermo fan
#define FAN_THERMO_PIN 12
// 4-pin header FAN0 - only for 4-pin fans !!!
#define FAN_BOARD_PIN 8

#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN -1

#define SDA_PIN 20
#define SCL_PIN 21

//servo pins 5, 6, 13

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 1
#endif

#endif
//End DUE3DOM MINI Board
