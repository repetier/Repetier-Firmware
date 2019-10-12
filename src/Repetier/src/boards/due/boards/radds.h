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

// RADDS Board
// http://www.dr-henschke.de/RADDS_due.html
#if MOTHERBOARD == MOTHERBOARD_RADDS
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
#define HEATER_0_PIN 13
// Due analog pin #54
#define TEMP_0_PIN 7
#define HEATER_1_PIN 7
#define TEMP_1_PIN 3
// Due analog pin #58
#define HEATER_2_PIN 12
// Due analog pin #55
#define TEMP_2_PIN 6
#define HEATER_3_PIN 11
// Due analog pin #56
#define TEMP_3_PIN 5
// Due analog pin #57
#define TEMP_4_PIN 4

// Due analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 2
// There are no more analog pins freely available.
// You can use direction and enable pin from extruder 0 socket as they are also
// analog pins. Then you need to move the stepper driver to a different socket.

// Direction pin of extruder 0
#define THERMOCOUPLE_1_PIN 1
// Step pin of extruder 0
#define THERMOCOUPLE_2_PIN 0
// Enable pin of extruder 0
#define THERMOCOUPLE_3_PIN 10

#define ORIG_E0_STEP_PIN 61
#define ORIG_E0_DIR_PIN 60
#define ORIG_E0_ENABLE_PIN 62

#define ORIG_E1_STEP_PIN 64
#define ORIG_E1_DIR_PIN 63
#define ORIG_E1_ENABLE_PIN 65

#define ORIG_E2_STEP_PIN 51
#define ORIG_E2_DIR_PIN 53
#define ORIG_E2_ENABLE_PIN 49

// Extra driver on extension board
// Might require pin 66 high for some drivers!
#define ORIG_E3_STEP_PIN 35
#define ORIG_E3_DIR_PIN 33
#define ORIG_E3_ENABLE_PIN 37

// Extra driver on extension port
// Might require pin 25 high for some drivers!
#define ORIG_E4_STEP_PIN 29
#define ORIG_E4_DIR_PIN 27
#define ORIG_E4_ENABLE_PIN 31

#define ORIG_E5_STEP_PIN 67
#define ORIG_E5_DIR_PIN 66
#define ORIG_E5_ENABLE_PIN 68

#define EXTENSION_BOARD_MS1 67
#define EXTENSION_BOARD_MS2 68
#define EXTENSION_BOARD_MS3 69
// 66 -> not connected
// 25 -> not connected
// To set microstepping on startup set START_GCODE to e.g.
// "M42 P67 S255\nM42 P68 S255\nM42 P69 S255"

#ifndef SDSUPPORT
#define SDSUPPORT 1
#endif
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN 8
#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define SERVO1 5
#define SERVO2 6
#define SERVO3 39

#define BEEPER_PIN -1

// Controller related default pins

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER == CONTROLLER_SPARKLCD

#define UI_DISPLAY_RS_PIN 25
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 27
#define UI_DISPLAY_D4_PIN 29
#define UI_DISPLAY_D5_PIN -1
#define UI_ENCODER_A 35
#define UI_ENCODER_B 33
#define UI_ENCODER_CLICK 37
#define UI_RESET_PIN -1
#define UI_BACK_PIN -1

#elif FEATURE_CONTROLLER == CONTROLLER_SPARKLCD_ADAPTER || FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2

#define UI_DISPLAY_RS_PIN 44
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 45
#define UI_DISPLAY_D4_PIN 46
#define UI_DISPLAY_D5_PIN -1
#define UI_ENCODER_A 50
#define UI_ENCODER_B 52
#define UI_ENCODER_CLICK 48
#define UI_RESET_PIN -1
#define UI_BACK_PIN -1

#else

// This is for official display port usage

#undef BEEPER_PIN
#define BEEPER_PIN 41
#define UI_DISPLAY_RS_PIN 42
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN 43
#define UI_DISPLAY_D0_PIN 44
#define UI_DISPLAY_D1_PIN 45
#define UI_DISPLAY_D2_PIN 46
#define UI_DISPLAY_D3_PIN 47
#define UI_DISPLAY_D4_PIN 44
#define UI_DISPLAY_D5_PIN 45
#define UI_DISPLAY_D6_PIN 46
#define UI_DISPLAY_D7_PIN 47
#define UI_ENCODER_A 50
#define UI_ENCODER_B 52
#define UI_ENCODER_CLICK 48
#define UI_RESET_PIN -1
#define UI_BACK_PIN 71

#endif
#endif

// Servo pins: 5,6 und 39

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,
#define E5_PINS ORIG_E5_STEP_PIN, ORIG_E5_DIR_PIN, ORIG_E5_ENABLE_PIN,

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
