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

// STACKER 3D Superboard

/*
Board Pin Name  Firmware Name
HE1             HEATER_0
HE2             HEATER_2
HE3             HEATER_3
HE4             HEATER_4
HE5             HEATER_5
HE6             HEATER_6
HE7             HEATER_7
HE8             HEATER_8 / ORIG_FAN2_PIN
FAN             ORIG_FAN_PIN
H-BED           HEATER_1

*/

#if MOTHERBOARD == 412
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN 24
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 28
#define ORIG_X_MAX_PIN 34
#define ORIG_X_ENABLE_PIN 26

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 30
#define ORIG_Y_MAX_PIN 36
#define ORIG_Y_ENABLE_PIN 22

#define ORIG_Z_STEP_PIN 2
#define ORIG_Z_DIR_PIN 3
#define ORIG_Z_MIN_PIN 32
#define ORIG_Z_MAX_PIN 38
#define ORIG_Z_ENABLE_PIN 15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
/*
Input Pins Channel Number
AD0 CH0
AD1 CH1
AD2 CH2
AD3 CH3
AD4 CH4
AD5 CH5
AD6 CH6
AD7 CH7
AD8 CH8
AD9 CH9
AD10 CH10
AD11 CH11
AD12 CH12
AD13 CH13
AD14 CH14
AD15 CH15
*/
#define HEATER_0_PIN 7 ///< PWM channel 6
#define TEMP_0_PIN 7
// These pins are for bed !
#define HEATER_1_PIN 98
#define TEMP_1_PIN 6
#define HEATER_2_PIN 8 ///< PWM channel 5
#define TEMP_2_PIN 5
#define HEATER_3_PIN 9 ///< PWM Channel 4
#define TEMP_3_PIN 4
// D7
#define HEATER_4_PIN 11
#define TEMP_4_PIN 3
// D8
#define HEATER_5_PIN 12
#define HEATER_6_PIN 13
#define HEATER_7_PIN 100
#define HEATER_8_PIN 72
#define TEMP_5_PIN 0
#define TEMP_6_PIN 1
#define TEMP_7_PIN 2

#define THERMOCOUPLE_0_PIN 10
#define THERMOCOUPLE_1_PIN 11
#define THERMOCOUPLE_2_PIN 12
#define THERMOCOUPLE_3_PIN 13
#define THERMOCOUPLE_4_PIN 14
#define THERMOCOUPLE_5_PIN 0
#define THERMOCOUPLE_6_PIN 1
#define THERMOCOUPLE_7_PIN 2

#define ORIG_E0_STEP_PIN 105
#define ORIG_E0_DIR_PIN 106
#define ORIG_E0_ENABLE_PIN 107

#define ORIG_E1_STEP_PIN 102
#define ORIG_E1_DIR_PIN 101
#define ORIG_E1_ENABLE_PIN 103

#define ORIG_E2_STEP_PIN 51
#define ORIG_E2_DIR_PIN 53
#define ORIG_E2_ENABLE_PIN 49

#define ORIG_E3_STEP_PIN 41
#define ORIG_E3_DIR_PIN 40
#define ORIG_E3_ENABLE_PIN 99

#define ORIG_E4_STEP_PIN 5
#define ORIG_E4_DIR_PIN 4
#define ORIG_E4_ENABLE_PIN 31

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 77
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 97
// Same as heater 8, alias
#define ORIG_FAN2_PIN 72
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21
#define BEEPER_PIN -1

// Display controller

#ifndef CUSTOM_CONTROLLER_PINS
#define UI_DISPLAY_RS_PIN 29
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 27
#define UI_DISPLAY_D4_PIN 25
#define UI_DISPLAY_D5_PIN -1
#define UI_ENCODER_A 35
#define UI_ENCODER_B 33
#define UI_ENCODER_CLICK 37
#define UI_RESET_PIN -1
#define UI_BACK_PIN -1
#endif

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,

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
