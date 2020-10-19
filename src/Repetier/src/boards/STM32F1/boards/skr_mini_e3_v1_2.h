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

#ifndef STM32F1
#error "Oops! Select skr_mini_e3_v1_2 in platformio.ini -> default_envs"
#endif

#ifndef Serial
#define Serial SerialUSB
#endif

#ifndef SDSUPPORT
#define SDSUPPORT               true
#endif

#define CPU_ARCH                ARCH_ARM
#define MAX_RAM                 47960

// Steppers
#define ORIG_X_STEP_PIN         PB13
#define ORIG_X_DIR_PIN          PB12
#define ORIG_X_ENABLE_PIN       PB14
 
#define ORIG_X_MIN_PIN          PC0

#define ORIG_Y_STEP_PIN         PB10
#define ORIG_Y_DIR_PIN          PB2
#define ORIG_Y_ENABLE_PIN       PB11

#define ORIG_Y_MIN_PIN          PC1

#define ORIG_Z_STEP_PIN         PB0
#define ORIG_Z_DIR_PIN          PC5
#define ORIG_Z_ENABLE_PIN       PB1 

#define ORIG_Z_MIN_PIN          PC2

#define ORIG_E0_STEP_PIN        PB3
#define ORIG_E0_DIR_PIN         PB4
#define ORIG_E0_ENABLE_PIN      PD2

#define ORIG_E0_MIN_PIN         PC15 // E0-STOP

#define X_SW_SERIAL_TMC_PIN     PB15
#define Y_SW_SERIAL_TMC_PIN     PC6  
#define Z_SW_SERIAL_TMC_PIN     PC10
#define E0_SW_SERIAL_TMC_PIN    PC11  

#define TMC_SW_SERIAL_BAUD      57600

#define SERVO_1_PIN             PA1
#define BLTOUCH_Z_MIN           PC14

// Temperature Sensors
#define TEMP_0_PIN              PA0 // HOTEND
#define TEMP_1_PIN              PC3 // BED 

#define TEMP_CPU                ADC_CHANNEL_TEMPSENSOR
#define VREF_CPU                ADC_CHANNEL_VREFINT

// Heaters / Fans
#define HEATER_0_PIN            PC8
#define HEATER_1_PIN            PC9

#define ORIG_FAN_PIN            PA8

#define EEPROM_PAGE_SIZE        FLASH_PAGE_SIZE     // page write buffer size

// BTT's bootloader seems to use up around 28kb?
#define FLASH_START             0x08007000ul

#ifndef STM32_FLASH_SIZE
#define STM32_FLASH_SIZE        256
#endif

#ifndef FLASH_SIZE 
#define FLASH_SIZE              (STM32_FLASH_SIZE == 512 ? 0x0080000ul : 0x0040000ul)
#endif

#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE       EEPROM_BYTES * 8 
#endif

#if SDSUPPORT
#ifndef EEPROM_MODE
#define EEPROM_MODE             EEPROM_SDCARD
#endif

#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE        EEPROM_SDCARD 
#endif
#else 

#ifndef EEPROM_MODE
#define EEPROM_MODE             EEPROM_FLASH
#endif

#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE        EEPROM_FLASH 
#endif
#endif

#ifndef EEPROM_MODE
#define EEPROM_MODE             EEPROM_NONE
#endif
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE        EEPROM_NONE 
#endif
//
// Misc. Functions
//

#ifndef SDSS
#define SDSS                    PA4
#endif 

#define ORIG_SDCARDDETECT       PC4
#define SDCARDDETECTINVERTED    false

#define STATUS_LED_PIN          PA15             // Small PCB LED
#define NEOPIXEL_PIN            PC7

// Meant for Bigtree's relay module (auto shutdown) 
#define POWER_RELAY_MODULE_PIN  PC12 // AKA PT-DET (J7 Connector)
// I'm unsure how their module functions but I believe it could be tied to ORIG_PS_ON_PIN
// (which turns off and on a power supply)

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#define BEEPER_PIN              PB5

#define UI_ENCODER_A            PA9
#define UI_ENCODER_B            PA10 
#define UI_ENCODER_CLICK        PB6

#define UI_DISPLAY_ENABLE_PIN   PB7    // SW SPI MOSI
#define UI_DISPLAY_RS_PIN       PB8    // SW SPI CS
#define UI_DISPLAY_D4_PIN       PB9    // SW SPI SCK
#define UI_DISPLAY_D5_PIN       NO_PIN // NOT USED

#endif
#endif
 
/* // Moses - for debugging stm32f1 hal only
#define ENABLE_SOFTWARE_SPI_CLASS 1
#define SD_SOFT_MISO_PIN        PA6
#define SD_SOFT_MOSI_PIN        PA7
#define SD_SOFT_SCK_PIN         PA5
*/


#define ORIG_PS_ON_PIN          NO_PIN 
#define ORIG_X_MAX_PIN          NO_PIN
#define ORIG_Y_MAX_PIN          NO_PIN
#define ORIG_Z_MAX_PIN          NO_PIN
#define LED_PIN                 STATUS_LED_PIN

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 0
#endif
#define SDPOWER -1
