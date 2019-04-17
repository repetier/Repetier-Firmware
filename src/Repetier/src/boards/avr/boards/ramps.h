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

#if MOTHERBOARD == 33
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#elif MOTHERBOARD == 34
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define AZTEEG_X3
#elif MOTHERBOARD == 35
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define AZTEEG_X3_PRO
#elif MOTHERBOARD == 39
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define ZRIB_V2
#elif MOTHERBOARD == 38
#define RAMPS_V_1_3
#define MPX3
#endif
#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 39 || MOTHERBOARD == 38
#define KNOWN_BOARD 1

#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#ifdef RAMPS_V_1_3

#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_ENABLE_PIN 38
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN 2

#define ORIG_Y_STEP_PIN 60
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_ENABLE_PIN 56
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN 15

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_ENABLE_PIN 62
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1
#define SDSS 53
#define ORIG_SDCARDDETECT 49

#define LED_PIN 13
#define ORIG_FAN_PIN 9
#define ORIG_PS_ON_PIN 12

#define HEATER_0_PIN 10
#define HEATER_1_PIN 8
#define HEATER_2_PIN 9
// ANALOG NUMBERING
#define TEMP_0_PIN 13
#define TEMP_1_PIN 14
#define TEMP_2_PIN 15
#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#else // RAMPS_V_1_1 or RAMPS_V_1_2 as default

#define ORIG_X_STEP_PIN 26
#define ORIG_X_DIR_PIN 28
#define ORIG_X_ENABLE_PIN 24
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN -1 //2

#define ORIG_Y_STEP_PIN 38
#define ORIG_Y_DIR_PIN 40
#define ORIG_Y_ENABLE_PIN 36
#define ORIG_Y_MIN_PIN 16
#define ORIG_Y_MAX_PIN -1 //17

#define ORIG_Z_STEP_PIN 44
#define ORIG_Z_DIR_PIN 46
#define ORIG_Z_ENABLE_PIN 42
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN -1 //19

#define ORIG_E0_STEP_PIN 32
#define ORIG_E0_DIR_PIN 34
#define ORIG_E0_ENABLE_PIN 30

#define SDPOWER 48
#define SDSS 53
#define LED_PIN 13
#define ORIG_PS_ON_PIN -1
//#define SCL                21
//#define SDA                20

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS

#ifdef RAMPS_V_1_0 // RAMPS_V_1_0
#define HEATER_0_PIN 12
#define HEATER_1_PIN -1
#define ORIG_FAN_PIN 11

#else // RAMPS_V_1_1 or RAMPS_V_1_2
#define HEATER_0_PIN 10
#define HEATER_1_PIN 8
#define ORIG_FAN_PIN 9
#endif

// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN 2
#define TEMP_1_PIN 1
#endif

// SPI for Max6675 Thermocouple

// these pins are defined in the SD library if building with SD support
#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51
#define MAX6675_SS 53

#ifdef AZTEEG_X3
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED 0
#define ORIG_SDCARDDETECT 49
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN 4
#define ORIG_FAN2_PIN 5
#define LIGHT_PIN 6
// Activate beeper on extension shield
#define BEEPER_PIN 33
#define BEEPER_TYPE 1

// Only available with X3 shield
#define ORIG_E2_STEP_PIN 27
#define ORIG_E2_DIR_PIN 29
#define ORIG_E2_ENABLE_PIN 41
// Only available with X3 shield
#define ORIG_E3_STEP_PIN 23
#define ORIG_E3_DIR_PIN 25
#define ORIG_E3_ENABLE_PIN 40
// Only available with X3 shield
#define HEATER_3_PIN 17
#define TEMP_3_PIN 12
#define HEATER_4_PIN 16
#define TEMP_4_PIN 5

#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS E3_STEP_PIN, E3_DIR_PIN, E3_ENABLE_PIN,

#endif

#ifdef AZTEEG_X3_PRO
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED false
#define ORIG_SDCARDDETECT 49
#define SDSS 53
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN 5
#define ORIG_FAN2_PIN 6
#define LIGHT_PIN 11
// Activate beeper on extension shield
#define BEEPER_PIN 33
#define BEEPER_TYPE 1

#define ORIG_E2_STEP_PIN 23
#define ORIG_E2_DIR_PIN 25
#define ORIG_E2_ENABLE_PIN 40
#define ORIG_E3_STEP_PIN 27
#define ORIG_E3_DIR_PIN 29
#define ORIG_E3_ENABLE_PIN 41
#define ORIG_E4_STEP_PIN 43
#define ORIG_E4_DIR_PIN 37
#define ORIG_E4_ENABLE_PIN 42
#define HEATER_0_PIN 10
// bed
#define HEATER_1_PIN 8
#define HEATER_2_PIN 9
#define HEATER_3_PIN 16
#define HEATER_4_PIN 17
#define HEATER_5_PIN 4
// ANALOG NUMBERING
#define TEMP_0_PIN 13
// BED , ANALOG NUMBERING
#define TEMP_1_PIN 14
#define TEMP_2_PIN 15
#define TEMP_3_PIN 12
#define TEMP_4_PIN 11
#define TEMP_5_PIN 10

// Thermocouple 1 and 2
#define TEMP_6_PIN 4
#define TEMP_7_PIN 5
#define THERMOCOUPLE_0_PIN 4
#define THERMOCOUPLE_1_PIN 5

// Special extension board for x3 pro allows 2 more extruders

#define ORIG_E5_STEP_PIN 12
#define ORIG_E5_DIR_PIN 47
#define ORIG_E5_ENABLE_PIN 63
#define ORIG_E6_STEP_PIN 39
#define ORIG_E6_DIR_PIN 57
#define ORIG_E6_ENABLE_PIN 31

#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,
#define E5_PINS ORIG_E5_STEP_PIN, ORIG_E5_DIR_PIN, ORIG_E5_ENABLE_PIN,
#define E6_PINS ORIG_E6_STEP_PIN, ORIG_E6_DIR_PIN, ORIG_E6_ENABLE_PIN,

#endif

// Zonestar ZRIB V2.1 Board
#ifdef ZRIB_V2
#undef HEATER_2_PIN
#define HEATER_2_PIN 7
#define ORIG_FAN2_PIN 6
#define SD_DETECT_PIN 49
#define LCD_PINS_RS 16
#define LCD_PINS_ENABLE 17
#define LCD_PINS_D4 23
#define LCD_PINS_D5 25
#define LCD_PINS_D6 27
#define LCD_PINS_D7 29
#define BEEPER_PIN 37
#endif

#ifdef MPX3
#undef HEATER_1_PIN
#define HEATER_1_PIN 8

#undef FAN_PIN
#define FAN_PIN 9

#undef HEATER_0_PIN
#define HEATER_0_PIN 10

#undef HEATER_2_PIN
#define HEATER_2_PIN 7
#endif

#endif
