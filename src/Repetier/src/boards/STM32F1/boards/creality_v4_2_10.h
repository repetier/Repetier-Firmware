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

#pragma once

#define MAX_RAM 65536
/**
 * CREALITY 4.2.10 (STM32F103) board pin assignments
 */

#if NUM_TOOLS > 1
#error "CREALITY supports up to 1 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_NO_NATIVE_USB
// Board has no usb serial
#ifndef SerialUSB
#define SerialUSB Serial3
#endif

//
// Servos
//
#define SERVO_1_PIN PB0 // BLTouch OUT

//
// Limit Switches
//
#define ORIG_X_MIN_PIN PA3
#define ORIG_Y_MIN_PIN PA7
#define ORIG_Z_MIN_PIN PA5

#define BLTOUCH_Z_MIN PA5 // BLTouch IN

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
#define FIL_RUNOUT_PIN PA6 // "Pulled-high"
#endif

//
// Steppers
//
#define ORIG_X_ENABLE_PIN PC3
#ifndef ORIG_X_STEP_PIN
#define ORIG_X_STEP_PIN PC2
#endif
#ifndef ORIG_X_DIR_PIN
#define ORIG_X_DIR_PIN PB9
#endif

#define ORIG_Y_ENABLE_PIN PC3
#ifndef ORIG_Y_STEP_PIN
#define ORIG_Y_STEP_PIN PB8
#endif
#ifndef ORIG_Y_DIR_PIN
#define ORIG_Y_DIR_PIN PB7
#endif

#define ORIG_Z_ENABLE_PIN PC3
#ifndef ORIG_Z_STEP_PIN
#define ORIG_Z_STEP_PIN PB6
#endif
#ifndef ORIG_Z_DIR_PIN
#define ORIG_Z_DIR_PIN PB5
#endif

#define ORIG_E0_ENABLE_PIN PC3
#ifndef ORIG_E0_STEP_PIN
#define ORIG_E0_STEP_PIN PB4
#endif
#ifndef ORIG_E0_DIR_PIN
#define ORIG_E0_DIR_PIN PB3
#endif

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// Temperature Sensors
//
#define TEMP_0_PIN PC5 // TH1
#define TEMP_1_PIN PC4 // TB1 Bed

//
// Heaters / Fans
//
#define HEATER_0_PIN PA0 // HEATER1
#define HEATER_1_PIN PA1 // HOT BED

#define ORIG_FAN_PIN PA2 // FAN
#define FAN_SOFT_PWM

//
// SD Card
//
/* #define SD_DETECT_PIN PC7
#define SDCARD_CONNECTION ONBOARD
#define ONBOARD_SPI_DEVICE 1
#define ONBOARD_SD_CS_PIN PA4 // SDSS
#define SDIO_SUPPORT
#define NO_SD_HOST_DRIVE // This board's SD is only seen by the printer
*/
#ifndef SDSS
#define SDSS PA4
#endif

#define ORIG_SDCARDDETECT PC7
#define SDCARDDETECTINVERTED false

// BTT's bootloader seems to use up around 28kb?
#define FLASH_START 0x08007000ul

#ifndef STM32_FLASH_SIZE
#define STM32_FLASH_SIZE 512
#endif

#ifndef FLASH_SIZE
#define FLASH_SIZE (STM32_FLASH_SIZE == 512 ? 0x0080000ul : 0x0040000ul)
#endif

#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE EEPROM_BYTES * 8
#endif

#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_FLASH
#endif

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 1000000
#endif
#define SDPOWER -1

#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

#define EEPROM_SERIAL_ADDR 0x50
#define EEPROM_PAGE_WRITE_TIME 10

#undef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32

/*
#if ENABLED(CR10_STOCKDISPLAY) && NONE(RET6_12864_LCD, VET6_12864_LCD)
#error "Define RET6_12864_LCD or VET6_12864_LCD to select pins for CR10_STOCKDISPLAY with the Creality V4 controller."
#endif

#if ENABLED(RET6_12864_LCD)

// RET6 12864 LCD
#define LCD_PINS_RS PB12
#define LCD_PINS_ENABLE PB15
#define LCD_PINS_D4 PB13

#define BTN_ENC PB2
#define BTN_EN1 PB10
#define BTN_EN2 PB14

#define BEEPER_PIN PC6

#elif ENABLED(VET6_12864_LCD)

// VET6 12864 LCD
#define LCD_PINS_RS PA4
#define LCD_PINS_ENABLE PA7
#define LCD_PINS_D4 PA5

#define BTN_ENC PC5
#define BTN_EN1 PB10
#define BTN_EN2 PA6

#elif ENABLED(DWIN_CREALITY_LCD)

// RET6 DWIN ENCODER LCD
#define BTN_ENC PB14
#define BTN_EN1 PB15
#define BTN_EN2 PB12

//#define LCD_LED_PIN                     PB2
#ifndef BEEPER_PIN
#define BEEPER_PIN PB13
#undef SPEAKER
#endif

#elif ENABLED(DWIN_VET6_CREALITY_LCD)

// VET6 DWIN ENCODER LCD
#define BTN_ENC PA6
#define BTN_EN1 PA7
#define BTN_EN2 PA4

#define BEEPER_PIN PA5

#endif
*/
