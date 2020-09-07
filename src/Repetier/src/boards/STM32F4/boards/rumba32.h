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

The Current (may 2020) MKS-Rumba32 and Aus3d RUMBA32 boards have a defect making
spi controlled TMC drivers not work properly. See
https://github.com/makerbase-mks/MKS-RUMBA32/wiki/About-RUMBA32-TMC-SPI-%22CONNECTION-ERROR%22-FAQ
for a solution.

STM32F446 processor, 180MHz, 512kb flash, 128kb RAM

EXP1 and EXP2 are rotated 180° compared to Smart Graphics Controller pins, so rotate cable accordingly

Expansion ports with orientation board power connector lower right corner.

EXP1:
                      GND     PE14   PE12       PE7      PE8
Header pins top:       9       7       5         3        1
Header pins bottom:    10      8       6         4        2
                      3.3V    PE15    PE13      PE10     PE7

EXP2:
                      GND     PB0     PB1       PB2      MISO
Header pins top:       9       7       5         3        1
Header pins bottom:    10      8       6         4        2
                      Kill   Reset    MOSI      PA2      SCK

EXP3:
                     PD13   PA9(TX1) (SCL)    PA4   PD14(PWM1)  GND      12V
Header pins top:      13      11       9       7       5         3        1
Header pins bottom:   14      12       10      8       6         4        2
                     PD12  PA10(RX1)  (SDA)   PA3   PD15(PWM2)  GND       5V

For a second serial connection add
-DHAVE_HWSERIAL1
to the build flags in platformio.ini.
In configration.h set BLUETOOTH_SERIAL 1
This makes EXP3 pin 11/12 also a serial as described in pin header above.

*/
#pragma once

#ifndef STM32F4
#error "Oops! Select RUMBA32 in platformio.ini -> default_envs"
#endif

// Users expect Serial to be usb port!
#undef Serial
#define Serial SerialUSB

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
#define TEMP_0_PIN static_cast<int>(PC_4) // T0
#define TEMP_2_PIN static_cast<int>(PC_3) // T1
#define TEMP_3_PIN static_cast<int>(PC_2) // T2
#define TEMP_4_PIN static_cast<int>(PC_1) // T3
#define TEMP_1_PIN static_cast<int>(PC_0) // TB bed

#define THERMOCOUPLE_0_PIN static_cast<int>(PA_3) // A10 on EXP3
#define THERMOCOUPLE_1_PIN static_cast<int>(PA_4) // A9 on EXP3

// Heaters / Fans
#define HEATER_0_PIN PC6 // E0 Timer 3 Channel 1
#define HEATER_2_PIN PC7 // E1 Timer 8 Channel 2
#define HEATER_3_PIN PC8 // E2 Timer 3 Channel 3
#define HEATER_1_PIN PA1 // bed Timer 5 Channel 2

// Note: In schematic fan0 goes to fan1 and fan1 to fan0. We use numbering printed on board.
#define ORIG_FAN_PIN PC9  // Timer 8 Channel 4
#define ORIG_FAN2_PIN PA8 // Timer 1 Channel 1
#define PWM1_PIN PD14     // TIM4_CH3
#define PWM2_PIN PD15     // TIM4_CH4

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 400000
#endif

// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#define FLASH_START 0x08000000ul
#define FLASH_SIZE 0x0080000ul // flash size excluding bootloader
#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE 0x20000ul // use 128kb flash to prevent early destruction of flash but leave room for firmware
#define FLASH_SECTOR 7              // sector 7 is last 128kb
#endif
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_FLASH
#endif

// I2C
#define SCK_PIN PA5
#define MISO_PIN PA6
#define MOSI_PIN PA7

//
// Misc. Functions
//
#define LED_PIN PB14
#define BTN_PIN PC10
#define ORIG_PS_ON_PIN PE11
#define KILL_PIN PC5

#ifndef SDSS
#define SDSS PA2
#endif
#define SDPOWER -1
#define ORIG_SDCARDDETECT PB0

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

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
#ifndef BEEPER_PIN
#define BEEPER_PIN PE8
#endif
#ifndef SDCARDDETECT
#define SDCARDDETECT PB0
#endif

#endif
#endif
