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

#ifndef creality_F103
#error "Oops! Select creality_F103 in platformio.ini -> default_envs"
#endif

// Board has no usb serial
#ifndef SerialUSB
#define SerialUSB Serial3
#endif

#ifndef SDSUPPORT
#define SDSUPPORT true
#endif

#define CPU_ARCH ARCH_ARM
#define MAX_RAM 65536

// Steppers
#ifndef ORIG_X_STEP_PIN
#define ORIG_X_STEP_PIN PC2
#endif
#ifndef ORIG_X_DIR_PIN
#define ORIG_X_DIR_PIN PB9
#endif
#ifndef ORIG_X_ENABLE_PIN
#define ORIG_X_ENABLE_PIN PC3
#endif

#ifndef ORIG_X_MIN_PIN
#define ORIG_X_MIN_PIN PA5
#endif

#ifndef ORIG_Y_STEP_PIN
#define ORIG_Y_STEP_PIN PB8
#endif
#ifndef ORIG_Y_DIR_PIN
#define ORIG_Y_DIR_PIN PB7
#endif
#ifndef ORIG_Y_ENABLE_PIN
#define ORIG_Y_ENABLE_PIN PC3
#endif

#ifndef ORIG_Y_MIN_PIN
#define ORIG_Y_MIN_PIN PA6
#endif

#ifndef ORIG_Z_STEP_PIN
#define ORIG_Z_STEP_PIN PB6
#endif
#ifndef ORIG_Z_DIR_PIN
#define ORIG_Z_DIR_PIN PB5
#endif
#ifndef ORIG_Z_ENABLE_PIN
#define ORIG_Z_ENABLE_PIN PC3
#endif

#ifndef ORIG_Z_MIN_PIN
#define ORIG_Z_MIN_PIN PA7
#endif

#ifndef ORIG_E0_STEP_PIN
#define ORIG_E0_STEP_PIN PB4
#endif
#ifndef ORIG_E0_DIR_PIN
#define ORIG_E0_DIR_PIN PB3
#endif
#ifndef ORIG_E0_ENABLE_PIN
#define ORIG_E0_ENABLE_PIN PC3
#endif

#ifndef ORIG_E0_MIN_PIN
#define ORIG_E0_MIN_PIN PA4 // E0-STOP
#endif

#define TMC_SW_SERIAL_BAUD 57600

#ifndef HAS_PIN_27_BOARD
#define SERVO_1_PIN PB0 // BLTouch OUT
#else
#define SERVO_1_PIN PC6
#endif
#define BLTOUCH_Z_MIN PB1

// Temperature Sensors
#define TEMP_0_PIN PC5 // HOTEND
#define TEMP_1_PIN PC4 // BED

#define TEMP_CPU ADC_CHANNEL_TEMPSENSOR
#define VREF_CPU ADC_CHANNEL_VREFINT

// Heaters / Fans
#define HEATER_0_PIN PA1 // Extruder
#define HEATER_1_PIN PA2 // Bed

#define ORIG_FAN_PIN PA0

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

#ifndef EEPROM_MODE
#define EEPROM_MODE EEPROM_NONE
#endif
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_NONE
#endif
//
// Misc. Functions
//

#ifndef SDSS
#define SDSS PA4
#endif

#define ORIG_SDCARDDETECT PC7
#define SDCARDDETECTINVERTED false

#define STATUS_LED_PIN NO_PIN

#define ORIG_PS_ON_PIN NO_PIN
#define ORIG_X_MAX_PIN NO_PIN
#define ORIG_Y_MAX_PIN NO_PIN
#define ORIG_Z_MAX_PIN NO_PIN
#define LED_PIN STATUS_LED_PIN

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

#define ORIG_FAN2_PIN PC7
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN PC6

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE
#error No pins defined for this controller, please add and submit if they work
#endif
#endif