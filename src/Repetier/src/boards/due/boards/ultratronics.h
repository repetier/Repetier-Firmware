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

// Ultratronics Board  (experimental, use with care probably even not working!)
// http://www.reprapworld.com
// https://reprapworld.com/documentation/datasheet_ultratronics10_05.pdf

#if MOTHERBOARD == MOTHERBOARD_ULTRATRONICS
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 35
#define ORIG_X_DIR_PIN 34
#define ORIG_X_MIN_PIN 31
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 37

#define ORIG_Y_STEP_PIN 22
#define ORIG_Y_DIR_PIN 23
#define ORIG_Y_MIN_PIN 12
#define ORIG_Y_MAX_PIN 11
#define ORIG_Y_ENABLE_PIN 33

#define ORIG_Z_STEP_PIN 25
#define ORIG_Z_DIR_PIN 26
#define ORIG_Z_MIN_PIN 29
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 24

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 3
// Due analog pin A0 = channel 7
#define TEMP_0_PIN 7

#define HEATER_1_PIN 2
// Due analog pin A1 = channel 6
#define TEMP_1_PIN 6
// Due analog pin #58

#define HEATER_2_PIN 8
// Due analog pin A2 = channel 5
#define TEMP_2_PIN 5

#define HEATER_3_PIN 7
// Due analog pin A3 = channel 4
#define TEMP_3_PIN 4

#define HEATER_4_PIN 9
// Due analog pin A4 = channel 3
#define TEMP_4_PIN 3

// Dua analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 65
#define THERMOCOUPLE_1_PIN 52
#define THERMOCOUPLE_2_PIN 51
#define THERMOCOUPLE_3_PIN 50

#define ORIG_E0_STEP_PIN 47
#define ORIG_E0_DIR_PIN 46
#define ORIG_E0_ENABLE_PIN 48

#define ORIG_E1_STEP_PIN 44
#define ORIG_E1_DIR_PIN 36
#define ORIG_E1_ENABLE_PIN 45

#define ORIG_E2_STEP_PIN 42
#define ORIG_E2_DIR_PIN 41
#define ORIG_E2_ENABLE_PIN 43

#define ORIG_E3_STEP_PIN 39
#define ORIG_E3_DIR_PIN 38
#define ORIG_E3_ENABLE_PIN 40

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 59
//#define NONSTANDARD_SDSS
#define MOSI_PIN 75
#define MISO_PIN 74
#define SCK_PIN 76

#define ORIG_SDCARDDETECT 60
#define SDCARDDETECTINVERTED 0
#define LED_PIN 13
#define ORIG_FAN_PIN 6
#define ORIG_FAN2_PIN 5
#define ORIG_PS_ON_PIN 32
#define KILL_PIN -1
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.
#define ENC424_SS 61

#define BEEPER_PIN 27

// 20 or 70
#define SDA_PIN 70
//21 or 71
#define SCL_PIN 71

// Servo pins: 5,6 und 39

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN, TEMP_0_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN, TEMP_2_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN, TEMP_3_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN, TEMP_4_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
// Ultronics has no eeprom for storing changeable data
// as a solution you can use sd card. But this requires always
// the same sd card when powering up the printer
//#define EEPROM_AVAILABLE EEPROM_NONE
#define EEPROM_AVAILABLE EEPROM_SDCARD

#ifndef WIRE_PORT
#define WIRE_PORT Wire1
#endif
#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 2
#endif

#define MB_SETUP \
    SET_OUTPUT(ORIG_FAN_PIN); \
    WRITE(ORIG_FAN_PIN, LOW); \
    SET_OUTPUT(ORIG_FAN2_PIN); \
    WRITE(ORIG_FAN2_PIN, LOW); \
    SET_OUTPUT(HEATER_0_PIN); \
    WRITE(HEATER_0_PIN, LOW); \
    SET_OUTPUT(HEATER_1_PIN); \
    WRITE(HEATER_1_PIN, LOW); \
    SET_OUTPUT(HEATER_2_PIN); \
    WRITE(HEATER_2_PIN, LOW); \
    SET_OUTPUT(HEATER_3_PIN); \
    WRITE(HEATER_3_PIN, LOW); \
    SET_OUTPUT(THERMOCOUPLE_0_PIN); \
    WRITE(THERMOCOUPLE_0_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_1_PIN); \
    WRITE(THERMOCOUPLE_1_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_2_PIN); \
    WRITE(THERMOCOUPLE_2_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_3_PIN); \
    WRITE(THERMOCOUPLE_3_PIN, HIGH); \
    SET_OUTPUT(ENC424_SS); \
    WRITE(ENC424_SS, HIGH); \
    SET_OUTPUT(SDSS); \
    WRITE(SDSS, HIGH)

#endif

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER == CONTROLLER_REPRAPWORLD_GLCD
#undef BEEPER_PIN
#define BEEPER_PIN 27
#define UI_DISPLAY_RS_PIN 62
#define UI_DISPLAY_ENABLE_PIN 75
#define UI_DISPLAY_D4_PIN 76
#define UI_DISPLAY_D5_PIN -1
#define UI_DISPLAY_D6_PIN -1
#define UI_DISPLAY_D7_PIN -1
#define UI_ENCODER_A 20
#define UI_ENCODER_B 21
#define UI_ENCODER_CLICK 64
#define UI_RESET_PIN -1
#define UI_BACK_PIN -1
#undef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT 60
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0

#elif FEATURE_CONTROLLER > 1

#error There is no pin definition for the selected display. Please add it to boards/due/ultratronics.h to use it!

#endif
#endif
