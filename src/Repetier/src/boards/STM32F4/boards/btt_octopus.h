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

// Additional informations: https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-V1.0
/*

STM32F446 processor, 180MHz, 512kb flash, 128kb RAM
STM32F407 Prozessor, 162MHz, 512kb flash, 128kb RAM

TFT Serial: Serial1
Raspberry Serial: Serial2

*/
#pragma once

#if !defined(STM32F4) || !(defined(INI_BTT_OCTOPUS446) || defined(INI_BTT_OCTOPUS407))
#error "Oops! Select BTT_OCTOPUS_446 or BTT_OCTOPUS_407 in platformio.ini -> default_envs"
#endif

// Users expect Serial to be usb port!
#undef Serial
#define Serial SerialUSB

#define CPU_ARCH ARCH_ARM
#define MAX_RAM 131072

// Steppers
#define ORIG_X_STEP_PIN PF13 // MOTOR 0
#define ORIG_X_DIR_PIN PF12
#define ORIG_X_ENABLE_PIN PF14
#define ORIG_X_CS_PIN PC4
#define ORIG_X_MIN_PIN PG6
#define ORIG_X_MAX_PIN PG6
#define ORIG_X_DIAG_PIN PG6 // Diag 0
#define X_SERIAL_TX_PIN PC4
#define X_SERIAL_RX_PIN X_SERIAL_TX_PIN

#define ORIG_Y_STEP_PIN PG0 // MOTOR 1
#define ORIG_Y_DIR_PIN PG1
#define ORIG_Y_ENABLE_PIN PF15
#define ORIG_Y_CS_PIN PD11
#define ORIG_Y_MIN_PIN PG9
#define ORIG_Y_MAX_PIN PG9
#define ORIG_Y_DIAG_PIN PG9 // Diag 1
#define Y_SERIAL_TX_PIN PD11
#define Y_SERIAL_RX_PIN Y_SERIAL_TX_PIN

#define ORIG_Z_STEP_PIN PF11 // MOTOR 2
#define ORIG_Z_DIR_PIN PG3
#define ORIG_Z_ENABLE_PIN PG5
#define ORIG_Z_CS_PIN PC6
#define ORIG_Z_MIN_PIN PG10
#define ORIG_Z_MAX_PIN PG10
#define ORIG_Z_DIAG_PIN PG10 // Diag 2
#define Z_SERIAL_TX_PIN PC6
#define Z_SERIAL_RX_PIN Z_SERIAL_TX_PIN

#define ORIG_E0_STEP_PIN PG4 // MOTOR 3
#define ORIG_E0_DIR_PIN PC1
#define ORIG_E0_ENABLE_PIN PA0
#define ORIG_E0_CS_PIN PC7
#define ORIG_E0_DIAG_PIN PG11 // Diag 3
#define E0_SERIAL_TX_PIN PC7
#define E0_SERIAL_RX_PIN E0_SERIAL_TX_PIN

#define ORIG_E1_STEP_PIN PF9 // MOTOR 4
#define ORIG_E1_DIR_PIN PF10
#define ORIG_E1_ENABLE_PIN PG2
#define ORIG_E1_CS_PIN PF2
#define ORIG_E1_DIAG_PIN PG12 // Diag 4
#define E1_SERIAL_TX_PIN PF2
#define E1_SERIAL_RX_PIN E1_SERIAL_TX_PIN

#define ORIG_E2_STEP_PIN PC13 // MOTOR 5
#define ORIG_E2_DIR_PIN PF0
#define ORIG_E2_ENABLE_PIN PF1
#define ORIG_E2_CS_PIN PE4
#define ORIG_E2_DIAG_PIN PG13 // Diag 5
#define E2_SERIAL_TX_PIN PE4
#define E2_SERIAL_RX_PIN E2_SERIAL_TX_PIN

#define ORIG_E3_STEP_PIN PE2 // MOTOR 6
#define ORIG_E3_DIR_PIN PE3
#define ORIG_E3_ENABLE_PIN PD4
#define ORIG_E3_CS_PIN PE1
#define ORIG_E3_DIAG_PIN PG14 // Diag 6
#define E3_SERIAL_TX_PIN PE1
#define E3_SERIAL_RX_PIN E3_SERIAL_TX_PIN

#define ORIG_E4_STEP_PIN PE6 // MOTOR 7
#define ORIG_E4_DIR_PIN PA14
#define ORIG_E4_ENABLE_PIN PE0
#define ORIG_E4_CS_PIN PD3
#define ORIG_E4_DIAG_PIN PG15 // Diag 7
#define E4_SERIAL_TX_PIN PD3
#define E4_SERIAL_RX_PIN E4_SERIAL_TX_PIN

// Temperature Sensors
#define TEMP_0_PIN static_cast<int>(PF_4) // T0
#define TEMP_2_PIN static_cast<int>(PF_5) // T1
#define TEMP_3_PIN static_cast<int>(PF_6) // T2
#define TEMP_4_PIN static_cast<int>(PF_7) // T3
#define TEMP_1_PIN static_cast<int>(PF_3) // TB bed

#define THERMOCOUPLE_0_PIN static_cast<int>(PF_8) // J45

// Heaters / Fans
#define HEATER_0_PIN PA2  // HE0
#define HEATER_2_PIN PA3  // HE1
#define HEATER_3_PIN PB10 // HE2
#define HEATER_4_PIN PB11 // HE3
#define HEATER_1_PIN PA1  // BED_OUT

#define ORIG_FAN_PIN PA8   // Fan 0
#define ORIG_FAN2_PIN PE5  // Fan 1
#define ORIG_FAN3_PIN PD12 // Fan 2
#define ORIG_FAN4_PIN PD13 // Fan 3
#define ORIG_FAN5_PIN PD14 // Fan 4
#define ORIG_FAN6_PIN PD15 // Fan 5
// Fan 6-7 are always on

// Z-Probe on BL-Touch header
// GND 5V PB6 GND PB7
#define ZPROBE_PIN PB7
#define SERVO_1_PIN PB6

// Misc function
#define PS_ON_PIN PE11
#define POWER_LOSS_PIN PC0 // PWRDET
#define STATUS_LED_PIN PA13
#define TMC_SW_SERIAL_BAUD 19200 // Software serial baud rate

#ifndef TWI_CLOCK_FREQ
// #define TWI_CLOCK_FREQ 400000
#define TWI_CLOCK_FREQ 100000
#endif

// see eeprom device data sheet for the following values these are for 24xx256
#define I2C_SCL_PIN PB8 // Used for eeprom
#define I2C_SDA_PIN PB9
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_SIZE 4096         // 4KB (AT24C32)
#define EEPROM_PAGE_SIZE 32      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#define FLASH_START 0x08000000ul
#define FLASH_SIZE 0x0080000ul // flash size excluding bootloader
#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE 0x20000ul // use 128kb flash to prevent early destruction of flash but leave room for firmware
#define FLASH_SECTOR 7              // sector 7 is last 128kb
#endif
#ifndef EEPROM_AVAILABLE // Prefer I2C connected eeprom as eeprom storage
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

// SPI on board sd card (SPI3)
/* 
#define SCK_PIN PC10
#define MISO_PIN PC11
#define MOSI_PIN PC12
#define SPI3_SCK_PIN PC10
#define SPI3_MISO_PIN PC11
#define SPI3_MOSI_PIN PC12
#define CREATE_SPI3 spi3
#define SD_SPI_ADDRESS &spi3
#define ORIG_SDCARDDETECT PC14
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED 1
#ifndef SDSS
#define SDSS PC11 // PA4
#endif
#define INCLUDE_SDIOS 1
#define SD_SDIO 1 // Use SDIO for communication
*/

// lcd sd card as board uses sdio

#define SCK_PIN PA5
#define MISO_PIN PA6
#define MOSI_PIN PA7
#define ORIG_SDCARDDETECT PC15
#ifndef SDSUPPORT
#define SDSUPPORT 1
#endif
#define SDCARDDETECTINVERTED 0
#ifndef SDSS
#define SDSS PA4
#endif

// SPI2 on extension above on board sd card
#define SPI2_MOSI_PIN PC3
#define SPI2_MISO_PIN PC2
#define SPI2_SCK_PIN PB13
#define SPI2_SS_PIN PB12
#define CREATE_SPI2 spi2
#define SPI2_ADDRESS &spi2
//
// Misc. Functions
//
#define LED_PIN PB14
#define BTN_PIN PC10
#define ORIG_PS_ON_PIN PE11
#define KILL_PIN PC5
#define HAS_OTG_USB_HOST_SUPPORT // USB Flash Drive support
#define USES_DIAG_JUMPERS

#define SDPOWER -1

// Used Timers for pwm output: 1, 4, 5, 9, 11 (SW serial)
#define MOTION2_TIMER_NUM 6
#define MOTION3_TIMER_NUM 7
#define PWM_TIMER_NUM 10
#define SERVO_TIMER_NUM 8
#define TONE_TIMER_NUM 2

/**               ------                                      ------
 * (BEEPER) PE8  |10  9 | PE7  (BTN_ENC)         (MISO) PA6  |10  9 | PA5  (SCK)
 * (LCD_EN) PE9  | 8  7 | PE10 (LCD_RS)       (BTN_EN1) PB2  | 8  7 | PA4  (SD_SS)
 * (LCD_D4) PE12   6  5 | PE13 (LCD_D5)       (BTN_EN2) PB1    6  5 | PA7  (MOSI)
 * (LCD_D6) PE14 | 4  3 | PE15 (LCD_D7)     (SD_DETECT) PC15 | 4  3 | RESET
 *           GND | 2  1 | 5V                             GND | 2  1 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_03_PIN PE15
#define EXP1_04_PIN PE14
#define EXP1_05_PIN PE13
#define EXP1_06_PIN PE12
#define EXP1_07_PIN PE10
#define EXP1_08_PIN PE9
#define EXP1_09_PIN PE7
#define EXP1_10_PIN PE8

#define EXP2_03_PIN -1
#define EXP2_04_PIN PC15
#define EXP2_05_PIN PA7
#define EXP2_06_PIN PB2
#define EXP2_07_PIN PA4
#define EXP2_08_PIN PB1
#define EXP2_09_PIN PA5
#define EXP2_10_PIN PA6

// LCD / Controller
//                 EXP1                                        EXP2
//                ------                                      ------
// (BEEPER) PE8  |10  9 | PE7  (BTN_ENC)         (MISO) PA6  |10  9 | PA5  (SCK)
// (LCD_EN) PE9  | 8  7 | PE10 (LCD_RS)       (BTN_EN1) PB2  | 8  7 | PA4  (SD_SS)
// (LCD_D4) PE12   6  5 | PE13 (LCD_D5)       (BTN_EN2) PB1    6  5 | PA7  (MOSI)
// (LCD_D6) PE14 | 4  3 | PE15 (LCD_D7)     (SD_DETECT) PC15 | 4  3 | RESET
//           GND | 2  1 | 5V                             GND | 2  1 | --
//                ------                                      ------

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#define UI_DISPLAY_RS_PIN EXP1_07_PIN
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN EXP1_08_PIN
#define UI_DISPLAY_D4_PIN EXP1_06_PIN
#define UI_DISPLAY_D5_PIN EXP1_05_PIN
#define UI_DISPLAY_D6_PIN EXP1_04_PIN
#define UI_DISPLAY_D7_PIN EXP1_03_PIN
#define UI_ENCODER_A EXP2_08_PIN
#define UI_ENCODER_B EXP2_06_PIN
#define UI_ENCODER_CLICK EXP1_09_PIN
#define UI_SDCARDDETECT EXP2_04_PIN
#define UI_SD_MISO EXP2_10_PIN
#define UI_SD_MOSI EXP2_05_PIN
#define UI_SD_SCK EXP2_09_PIN
#define UI_SD_SS EXP2_07_PIN
#ifndef BEEPER_PIN
#define BEEPER_PIN EXP1_10_PIN
#endif

#if FEATURE_CONTROLLER == CONTROLLER_BTT_MINI_12864_V1
#define UI_SPI_CS EXP1_08_PIN
#define UI_DC EXP1_07_PIN
#define UI_SPI_SCK EXP2_09_PIN
#define UI_SPI_MOSI EXP2_05_PIN
#define UI_NEOPIXEL_PIN EXP1_05_PIN
#undef UI_DISPLAY_D5_PIN // Not DC but used for neopixel!
#define UI_DISPLAY_D5_PIN -1
#endif

#define UI_RESET_PIN EXP1_06_PIN

#endif
#endif
