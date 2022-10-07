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
 Github homepage of board: https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3
 Homepage: https://biqu.equipment/collections/control-board/products/bigtreetech-skr-mini-e3-v2-0-32-bit-control-board-for-ender-3?variant=39982232174690
 MCU: ARM-Cortex M0 + STM32G0B1RET6 / STM32G0B0RET6
*/
#pragma once

#if !defined(skr_mini_e3_v3_0) || !defined(STM32G0xx)
#error "Oops! Select skr_mini_e3_v3_0 in platformio.ini -> default_envs"
#endif
#define skr_mini_e3_v1_2 // Prevent error message in included file

// SKR E3 Mini V2 has a small onboard AT24C32 I2C EEPROM. Powered @ 3.3V.
#define TWI_CLOCK_FREQ 1000000

#undef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C

// see eeprom device data sheet for the following values these are for 24xx256
#define I2C_SCL_PIN PB6 // Used for eeprom
#define I2C_SDA_PIN PB7
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_SIZE 4096         // 4KB (AT24C32)
#define EEPROM_PAGE_SIZE 32      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)

/*
    SKR E3 Mini V2 - We use V1.2 as a base since it shares almost all the same pins.
    We just replace/add any new V2 pins here.
*/
#include "skr_mini_e3_v1_2.h"

#undef ORIG_E0_ENABLE_PIN
#define ORIG_E0_ENABLE_PIN PD1

#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN PC6   // "FAN0" TIM2_CH3
#define ORIG_FAN1_PIN PC7  // "FAN1" TIM2_CH4
#define ORIG_FAN2_PIN PB15 // "FAN2" TIM1_CH3

#undef NEOPIXEL_PIN
#define NEOPIXEL_PIN PA8 // LED driving pin

#undef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT PC3

#undef ORIG_PS_ON_PIN
#define ORIG_PS_ON_PIN PC13

#undef STATUS_LED_PIN
#define STATUS_LED_PIN PD8

#undef LED_PIN
#define LED_PIN STATUS_LED_PIN

#undef TEMP_1_PIN
#define TEMP_1_PIN PC4 // BED

#define FILAMENT_SENSOR0 PC15 // E0-STOP

#define X_SERIAL_TX_PIN PC10
#define X_SERIAL_RX_PIN PC11
#define X_SERIAL Serial3

#define Y_SERIAL_TX_PIN PC10
#define Y_SERIAL_RX_PIN PC11
#define Y_SERIAL Serial3

#define Z_SERIAL_TX_PIN PC10
#define Z_SERIAL_RX_PIN PC11
#define Z_SERIAL Serial3

#define E0_SERIAL_TX_PIN PC10
#define E0_SERIAL_RX_PIN PC11
#define E0_SERIAL Serial3

// Default TMC slave addresses
#ifndef X_SLAVE_ADDRESS
#define X_SLAVE_ADDRESS 0
#endif
#ifndef Y_SLAVE_ADDRESS
#define Y_SLAVE_ADDRESS 2
#endif
#ifndef Z_SLAVE_ADDRESS
#define Z_SLAVE_ADDRESS 1
#endif
#ifndef E0_SLAVE_ADDRESS
#define E0_SLAVE_ADDRESS 3
#endif

#undef FLASH_START
#define FLASH_START 0x08002000ul

// Preselect timer
// Extruder/Fan use timer 1 and 2

#define MOTION2_TIMER_NUM 4
#define MOTION3_TIMER_NUM 15
#define PWM_TIMER_NUM 17
#define SERVO_TIMER_NUM 16
#define TONE_TIMER_NUM 3
#define TIMER_SERIAL TIM14
#define TIMER_SERIAL_RAW_IRQ RAW_TIM14_IRQHandler

// LCD / Controller

/**
 *              SKR Mini E3 V3.0
 *                 ------
 *             5V | 1  2 | GND
 *  (LCD_EN) PD6  | 3  4 | PB8  (LCD_RS)
 *  (LCD_D4) PB9  | 5  6   PA10 (BTN_EN2)
 *          RESET | 7  8 | PA9  (BTN_EN1)
 * (BTN_ENC) PA15 | 9 10 | PB5  (BEEPER)
 *                 ------ 
 *                 EXP1
 */

#define EXP1_03_PIN PD6
#define EXP1_04_PIN PB8
#define EXP1_05_PIN PB9
#define EXP1_06_PIN PA10
#define EXP1_07_PIN -1
#define EXP1_08_PIN PA9
#define EXP1_09_PIN PA15
#define EXP1_10_PIN PB5

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#if FEATURE_CONTROLLER == CONTROLLER_CR10_EXP3
#endif
#undef UI_ENCODER_CLICK
#define UI_ENCODER_CLICK EXP1_09_PIN
#undef UI_DISPLAY_ENABLE_PIN
#define UI_DISPLAY_ENABLE_PIN EXP1_03_PIN
#undef UI_ENCODER_A
#define UI_ENCODER_A EXP1_08_PIN
#undef UI_ENCODER_B
#define UI_ENCODER_B EXP1_06_PIN
#undef UI_DISPLAY_RS_PIN
#define UI_DISPLAY_RS_PIN EXP1_04_PIN
#undef UI_DISPLAY_ENABLE_PIN
#define UI_DISPLAY_ENABLE_PIN EXP1_03_PIN
#undef UI_DISPLAY_D4_PIN
#define UI_DISPLAY_D4_PIN EXP1_05_PIN
#undef UI_DISPLAY_D5_PIN
#define UI_DISPLAY_D5_PIN NO_PIN // NOT USED
#define UI_SPI_CS EXP1_08_PIN
#define UI_DC NO_PIN
#define UI_SPI_SCK EXP1_05_PIN
#define UI_SPI_MOSI EXP1_03_PIN
#ifndef BEEPER_PIN
#define BEEPER_PIN EXP1_10_PIN
#endif

#endif
#endif
