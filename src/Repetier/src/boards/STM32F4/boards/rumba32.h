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

// Additional informations: https://github.com/Aus3D/RUMBA32
/*
STM32F446 processor, 180MHz, 512kb flash, 128kb RAM
*/
#pragma once

#ifndef STM32F4
#error "Oops! Select STM32F4 in platformio.ini -> default_envs"
#endif

#define CPU_ARCH ARCH_ARM
#define MAX_RAM 131072 // Rumba32

// Steppers
#define ORIG_X_STEP_PIN PA0
#define ORIG_X_DIR_PIN PC15
#define ORIG_X_ENABLE_PIN PC11
#define ORIG_X_CS_PIN PC14
#define ORIG_X_MIN_PIN PB12
#define ORIG_X_MAX_PIN PB13

#define ORIG_Y_STEP_PIN PE5
#define ORIG_Y_DIR_PIN PE6
#define ORIG_Y_ENABLE_PIN PE3
#define ORIG_Y_CS_PIN PE4
#define ORIG_Y_MIN_PIN PB15
#define ORIG_Y_MAX_PIN PD8

#define ORIG_Z_STEP_PIN PE1
#define ORIG_Z_DIR_PIN PE2
#define ORIG_Z_ENABLE_PIN PB7
#define ORIG_Z_CS_PIN PE0
#define ORIG_Z_MIN_PIN PD9
#define ORIG_Z_MAX_PIN PD10

#define ORIG_E0_STEP_PIN PB5
#define ORIG_E0_DIR_PIN PB6
#define ORIG_E0_ENABLE_PIN PC12
#define ORIG_E0_CS_PIN PC13

#define ORIG_E1_STEP_PIN PD6
#define ORIG_E1_DIR_PIN PD7
#define ORIG_E1_ENABLE_PIN PD4
#define ORIG_E1_CS_PIN PD5

#define ORIG_E2_STEP_PIN PD2
#define ORIG_E2_DIR_PIN PD3
#define ORIG_E2_ENABLE_PIN PD0
#define ORIG_E2_CS_PIN PD1

// Temperature Sensors
#define TEMP_0_PIN PC4
#define TEMP_2_PIN PC3
#define TEMP_3_PIN PC2
#define TEMP_4_PIN PC1
#define TEMP_1_PIN PC0 // bed

// Heaters / Fans
#define HEATER_0_PIN PC6 // E0
#define HEATER_2_PIN PC7 // E1
#define HEATER_3_PIN PC8 // E2
#define HEATER_1_PIN PA1 // bed

#define ORIG_FAN_PIN PC9
#define ORIG_FAN2_PIN PA8

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 400000
#endif

// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

// I2C
#define SCK_PIN PA5
#define MISO_PIN PA6
#define MOSI_PIN PA7

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

//
// Misc. Functions
//
#define LED_PIN PB14
#define BTN_PIN PC10
#define ORIG_PS_ON_PIN PE11
#define KILL_PIN PC5

#define SDSS PA2
#define SDPOWER -1
#define SD_DETECT_PIN PB0
#define BEEPER_PIN PE8

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS

#define UI_DISPLAY_RS_PIN PE10
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN PE9
#define UI_DISPLAY_D4_PIN PE12
#define UI_DISPLAY_D5_PIN PE13
#define UI_DISPLAY_D6_PIN PE14
#define UI_DISPLAY_D7_PIN PE15
#define UI_ENCODER_A PB1
#define UI_ENCODER_B PB2
#define UI_ENCODER_CLICK PE7
#define UI_BACK_PIN -1
#define UI_RESET_PIN -1

#endif
