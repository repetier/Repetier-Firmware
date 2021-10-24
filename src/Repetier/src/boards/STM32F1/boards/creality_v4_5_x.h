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

/**
 * Creality v4.5.2 and v4.5.3 (STM32F103RET6) board pin assignments
 */

//
// Release PB4 (Z_STEP_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

#define BOARD_NO_NATIVE_USB
// Board has no usb serial
#ifndef SerialUSB
#define SerialUSB Serial3
#endif

#define MAX_RAM 65536
//
// Limit Switches
//
#define ORIG_X_MIN_PIN PC4
#define ORIG_Y_MIN_PIN PC5
#define ORIG_Z_MIN_PIN PA4

#define FIL_RUNOUT_PIN PA7

//
// Probe
//
#define BLTOUCH_Z_MIN PA5

//
// Steppers
//
#define ORIG_X_ENABLE_PIN PC3
#define ORIG_X_STEP_PIN PB8
#define ORIG_X_DIR_PIN PB7

#define ORIG_Y_ENABLE_PIN PC3
#define ORIG_Y_STEP_PIN PB6
#define ORIG_Y_DIR_PIN PB5

#define ORIG_Z_ENABLE_PIN PC3
#define ORIG_Z_STEP_PIN PB4
#define ORIG_Z_DIR_PIN PB3

#define ORIG_E0_ENABLE_PIN PC3
#define ORIG_E0_STEP_PIN PC2
#define ORIG_E0_DIR_PIN PB9

//
// Temperature Sensors
//
#define TEMP_0_PIN PB1 // TH1 Extruder
#define TEMP_1_PIN PB0 // TB1 Bed

//
// Heaters / Fans
//

#define FAN_SOFT_PWM

//
// SD Card
//
// #define NO_SD_HOST_DRIVE // SD is only seen by the printer

#ifndef SDSS
#define SDSS PA4
#endif

#define ORIG_SDCARDDETECT PC7
#define SDCARDDETECTINVERTED false

#define SDIO_SUPPORT       // Extra added by Creality
#define SDIO_CLOCK 6000000 // In original source code overridden by Creality in sdio.h

//
// Misc. Functions
//
#define CASE_LIGHT_PIN PA6
