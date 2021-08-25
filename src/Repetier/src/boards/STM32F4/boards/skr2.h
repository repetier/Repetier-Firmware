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
/*
 homepage: https://www.bigtree-tech.com/products/bigtreetech-skr-2.html
 technical detail: https://github.com/bigtreetech/SKR-2
 pinout: https://github.com/bigtreetech/SKR-2/blob/master/Hardware/BIGTREETECH%20SKR%202-Pin.pdf


 Notes: 
 - Only 3 end stops so min/max are the same pins!
 - Board uses 3 different hardware SPI so refer to the correct one:
   SPI => EXP2 spi, SPI2 => internal sd card, SPI3 => motors
*/
#pragma once

#if !defined(STM32F4) || !defined(INI_SKR2)
#error "Oops! Select SKR2 in platformio.ini -> default_envs"
#endif

// Users expect Serial to be usb port!
// #undef Serial
// #define Serial SerialUSB

#define CPU_ARCH ARCH_ARM
#define MAX_RAM 131072 // 128KB

#define ORIG_ZPROBE PE4
#define FIL_RUNOUT1 PC2 // E0DET
#define FIL_RUNOUT2 PA0 // E1DET

// Steppers
#define ORIG_X_STEP_PIN PE2 // XM
#define ORIG_X_DIR_PIN PE1
#define ORIG_X_ENABLE_PIN PE3
#define ORIG_X_CS_PIN PE0
#define ORIG_X_MIN_PIN PC1 // X-STOP
#define ORIG_X_MAX_PIN PC1 // X-STOP
#define ORIG_X_DIAG PC1    // X-STOP

#define ORIG_Y_STEP_PIN PD5
#define ORIG_Y_DIR_PIN PD4
#define ORIG_Y_ENABLE_PIN PD6
#define ORIG_Y_CS_PIN PD3
#define ORIG_Y_MIN_PIN PC3 // Y-STOP
#define ORIG_Y_MAX_PIN PC3 // Y-STOP
#define ORIG_Y_DIAG PC3    // Y-STOP

#define ORIG_Z_STEP_PIN PA15
#define ORIG_Z_DIR_PIN PA8
#define ORIG_Z_ENABLE_PIN PD1
#define ORIG_Z_CS_PIN PD0
#define ORIG_Z_MIN_PIN PC0 // Z-STOP
#define ORIG_Z_MAX_PIN PC0 // Z-STOP
#define ORIG_Z_DIAG PC0    // Z-STOP

#define ORIG_E0_STEP_PIN PD15
#define ORIG_E0_DIR_PIN PD14
#define ORIG_E0_ENABLE_PIN PC7
#define ORIG_E0_CS_PIN PC6
#define ORIG_E0_DIAG PC2 // E0DET

#define ORIG_E1_STEP_PIN PD11
#define ORIG_E1_DIR_PIN PD10
#define ORIG_E1_ENABLE_PIN PD13
#define ORIG_E1_CS_PIN PD12
#define ORIG_E1_DIAG PA0 // E1DET

#define ORIG_E2_STEP_PIN NO_PIN
#define ORIG_E2_DIR_PIN NO_PIN
#define ORIG_E2_ENABLE_PIN NO_PIN
#define ORIG_E2_CS_PIN NO_PIN

// Temperature Sensors
#define TEMP_0_PIN static_cast<int>(PA2) // TH0
#define TEMP_1_PIN static_cast<int>(PA1) // TB
#define TEMP_2_PIN static_cast<int>(PA3) // TH1

// Heaters / Fans
#define HEATER_0_PIN PB3 // HE0
#define HEATER_2_PIN PB4 // HE1
#define HEATER_1_PIN PD7 // BED

// Note: In schematic fan0 goes to fan1 and fan1 to fan0. We use numbering printed on board.
#define ORIG_FAN_PIN PB7  // Timer 4 Channel 2
#define ORIG_FAN2_PIN PB6 // Timer 4 Channel 1
#define ORIG_FAN3_PIN PB5 // Timer 3 Channel 2
#define PWM1_PIN          // TIM4_CH3
#define PWM2_PIN          // TIM4_CH4

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 400000
#endif

// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#define FLASH_START 0x08000000ul
#define FLASH_SIZE 0x0100000ul // flash size excluding bootloader 1024k
#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE 0x20000ul // use 128kb flash to prevent early destruction of flash but leave room for firmware
#define FLASH_SECTOR 11             // sector 11 is last 128kb
#endif
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_FLASH
#endif

// SPI for motor
#define SCK_PIN PE15
#define MISO_PIN PA14
#define MOSI_PIN PE14

// Board has a extra SPI for EXP2 - Use SPI3 for it

// #define SD_SPI3_ADDRESS &SPI_3
// #define CREATE_SPI3 SPI_3 // remove comment to activate the spi
#define SPI3_SCK_PIN PA5
#define SPI3_MISO_PIN PA6
#define SPI3_MOSI_PIN PA7
#define SPI3_CHIP_DETECT PC4

// SD card uses second SPI so we need to create and assign it to sd card
// #define SD_SPI_ADDRESS &SPI_2
#define CREATE_SPI2 SPI_2
#define SPI2_MISO_PIN PC8
#define SPI2_MOSI_PIN PD2
#define SPI2_SCK_PIN PC12
#define SPI2_SS_PIN PC11 // Chip select pin for internal sd card

#define SDSUPPORT 0
//
// Misc. Functions
//
#define LED_PIN
#define BTN_PIN
#define ORIG_PS_ON_PIN PE8
#define KILL_PIN -1
#define POWER_LOSS_PIN PC15
#define NEOPIXEL_PIN PE6
#define SAFE_POWER_PIN PC13
#define SERVO0_PIN PE5 // SERVOS

#ifndef SDSS
#define SDSS PA2
#endif
#define SDPOWER -1
#define ORIG_SDCARDDETECT PB0

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#define UI_DISPLAY_RS_PIN PE9
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN PB1
#define UI_DISPLAY_D4_PIN PE10
#define UI_DISPLAY_D5_PIN PE11
#define UI_DISPLAY_D6_PIN PE12
#define UI_DISPLAY_D7_PIN PE13
#define UI_ENCODER_A PB2
#define UI_ENCODER_B PE7
#define UI_ENCODER_CLICK PB0
#ifndef BEEPER_PIN
#define BEEPER_PIN PC5
#endif
#ifndef SDCARDDETECT
#define SDCARDDETECT PC4
#endif

#endif
#endif
