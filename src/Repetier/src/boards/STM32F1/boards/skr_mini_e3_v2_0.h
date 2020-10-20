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
#error "Oops! Select skr_mini_e3_v2_0 in platformio.ini -> default_envs"
#endif

// SKR E3 Mini V2 has a small onboard AT24C32 I2C EEPROM. Powered @ 3.3V.
#define TWI_CLOCK_FREQ          1000000

#undef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE        EEPROM_I2C

#define EEPROM_SERIAL_ADDR      0x50
#define EEPROM_PAGE_WRITE_TIME  10

/*
    SKR E3 Mini V2 - We use V1.2 as a base since it shares almost all the same pins.
    We just replace/add any new V2 pins here.
*/
#include "skr_mini_e3_v1_2.h" 

#undef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE        32

#define ORIG_FAN2_PIN           PC7
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN            PC6

#undef NEOPIXEL_PIN
#define NEOPIXEL_PIN            PA8

#undef ORIG_PS_ON_PIN
#define ORIG_PS_ON_PIN          PC13

#undef STATUS_LED_PIN
#define STATUS_LED_PIN          PA13

#undef LED_PIN 
#define LED_PIN                 STATUS_LED_PIN

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#undef UI_ENCODER_CLICK
#define UI_ENCODER_CLICK        PA15
#undef UI_DISPLAY_ENABLE_PIN
#define UI_DISPLAY_ENABLE_PIN   PB15

#endif
#endif 