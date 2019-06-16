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

// Smart RAMPS without(408) and with EEPROM (413)
/*
    Smart RAMPS without eeprom is the classic RAMPS mounted on a due.
    The real smart RAMPS board includes a extra eeprom connected with I2C to
    compensate for the missing eeprom area on the due.

    A special problem is that where the AVR has it's SPI pins MISO/MOSI/CLK
    the due has no SPI. But some lcd controller expect them still there.

    SPI lies on pins
    70: SCK0
    10/77: CS0

    The following pins are on the ICSP connector and therefore not reachable
    74: MISO
    75: MOSI
    76: SCK

    SPI1 Pins not available on Arduino DUE
    SPI1 SPI1_MISO PE28 
    SPI1 SPI1_MOSI PE29 
    SPI1 SPI1_SPCK PE30 
    SPI1 SPI1_NPCS0 PE31 
    SPI1 SPI1_NPCS1 PF0 
    SPI1 SPI1_NPCS2 PF1 
    SPI1 SPI1_NPCS3 PF2 
*/

#if MOTHERBOARD == MOTHERBOARD_SMARTRAMPS_NO_EEPROM || MOTHERBOARD == MOTHERBOARD_SMARTRAMPS_EEPROM
#ifndef __SAM3X8E__
#erro oops !Be sure to have 'due Arduino' selected from the 'tools-> Boards menu'.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due pin assignments
******************************************************************/

#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN 2
#define ORIG_X_ENABLE_PIN 38

#define ORIG_Y_STEP_PIN 60
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN 15
#define ORIG_Y_ENABLE_PIN 56

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19
#define ORIG_Z_ENABLE_PIN 62

//Note that in due A0 pins on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 10
// Because analog pin #
#define TEMP_0_PIN 11
#define HEATER_1_PIN 8
// Because analog pin #
#define TEMP_1_PIN 12
#define HEATER_2_PIN 9
// Because analog pin #
#define TEMP_2_PIN 13

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1
// 10 if using HW spi. 53 if using SW SPI
#define SDSS 53
#define LED_PIN 13
#define ORIG_FAN_PIN 9
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1
#define SUICIDE_PIN -1 // Pin that has to be turned right after the start, to keep the power flowing.

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define SERVO1 11
#define SERVO2 6
#define SERVO3 5
#define SERVO4 4

#define TWI_CLOCK_FREQ 100000
#if MOTHERBOARD == MOTHERBOARD_SMARTRAMPS_NO_EEPROM
//	20 or 70
#define SDA_PIN -1
// 21 or 71
#define SCL_PIN -1
#define EEPROM_AVAILABLE EEPROM_NONE
#else
// this board supports eeprom
#define SDA_PIN 20 // 20 or 70
#define SCL_PIN 21 // 21 or 71

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values, these are for 24xx256
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

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864 || FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864_OLED

// MOSI 43
#define UI_DISPLAY_ENABLE_PIN 51
//76 // SCK pin
#define UI_DISPLAY_D4_PIN 52 //44
// Display A0 => LCD RS
#define UI_DISPLAY_D5_PIN 59
#define UI_DISPLAY_RS_PIN 44 //10
#define UI_DISPLAY_RW_PIN -1
#define UI_ENCODER_A 58
#define UI_ENCODER_B 40
#define UI_ENCODER_CLICK 67
#define UI_RESET_PIN 42
#define UI_RESET_PIN -1
#define UI_BACK_PIN -1
#undef SDSUPPORT
#define SDSUPPORT 1
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0
#define ENABLE_SOFTWARE_SPI_CLASS 1
#define SD_SOFT_MISO_PIN 50
#define SD_SOFT_MOSI_PIN 51
#define SD_SOFT_SCK_PIN 52
#ifndef BEEPER_PIN
#define BEEPER_PIN -1
#endif

#elif FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD || FEATURE_CONTROLLER == CONTROLLER_SMARTRAMPS
// SMART RAMPS FOR DUE - CRITICAL NOTE: MUST REMOVE THE RESET HEADER JUMPER NEXT TO AUX-2 OTHERWISE BOARD WILL RESET LOOP CONTINUOUSLY
#define UI_DISPLAY_RS_PIN 44     //CS
#define UI_DISPLAY_ENABLE_PIN 42 //MOSI
#define UI_DISPLAY_D4_PIN 40     //SCK
#define UI_DISPLAY_D5_PIN -1     //A0 LCD RS
#define UI_DISPLAY_D6_PIN -1
#define UI_DISPLAY_D7_PIN -1
#define BEEPER_PIN 66
#define UI_ENCODER_A 50
#define UI_ENCODER_B 47
#define UI_ENCODER_CLICK 67
#define UI_RESET_PIN 53
#define UI_DELAYPERCHAR 50
#define UI_BUTTON_BACK -1
#define ENABLE_SOFTWARE_SPI_CLASS 1
#define SD_SOFT_MISO_PIN 17
#define SD_SOFT_MOSI_PIN 51
#define SD_SOFT_SCK_PIN 16
#undef SDSS
#define SDSS 49
#undef SDCARDDETECT
#define SDCARDDETECT 52
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED 0
#undef SDSUPPORT
#define SDSUPPORT 1

#elif FEATURE_CONTROLLER == NO_CONTROLLER

#ifndef BEEPER_PIN
#define BEEPER_PIN -1
#endif
#undef SDSUPPORT
#define SDSUPPORT 0

#else

#error No known pin assignment for your display. Please define it in Configuration.h

#endif

#endif // CUSTOM_CONTROLLER_PINS

#endif // Motherboard match
