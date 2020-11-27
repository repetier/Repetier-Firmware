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

#ifndef PINS_H
#define PINS_H

#define CPU_ARCH ARCH_ARM
// Ensure right board selection
#ifndef STM32F4
#error Oops!  Make sure you have selected the right STM32F4 board in platformio.ini
#endif

#if MOTHERBOARD == MOTHERBOARD_RUMBA32
#include "boards/rumba32.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_STACKER3D_SUPER_MINI
#include "boards/stacker3d_super_mini.h"
#endif

#ifndef SDSSORIG
#define SDSSORIG -1
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED 0
#endif

// define non existent pins as empty placeholder

#ifndef ORIG_E2_STEP_PIN
#define ORIG_E2_STEP_PIN -1
#endif
#ifndef ORIG_E2_DIR_PIN
#define ORIG_E2_DIR_PIN -1
#endif
#ifndef ORIG_E2_ENABLE_PIN
#define ORIG_E2_ENABLE_PIN -1
#endif

#ifndef ORIG_E3_STEP_PIN
#define ORIG_E3_STEP_PIN -1
#endif
#ifndef ORIG_E3_DIR_PIN
#define ORIG_E3_DIR_PIN -1
#endif
#ifndef ORIG_E3_ENABLE_PIN
#define ORIG_E3_ENABLE_PIN -1
#endif

#ifndef ORIG_E4_STEP_PIN
#define ORIG_E4_STEP_PIN -1
#endif
#ifndef ORIG_E4_DIR_PIN
#define ORIG_E4_DIR_PIN -1
#endif
#ifndef ORIG_E4_ENABLE_PIN
#define ORIG_E4_ENABLE_PIN -1
#endif

#ifndef ORIG_E5_STEP_PIN
#define ORIG_E5_STEP_PIN -1
#endif
#ifndef ORIG_E5_DIR_PIN
#define ORIG_E5_DIR_PIN -1
#endif
#ifndef ORIG_E5_ENABLE_PIN
#define ORIG_E5_ENABLE_PIN -1
#endif

#ifndef ORIG_E6_STEP_PIN
#define ORIG_E6_STEP_PIN -1
#endif
#ifndef ORIG_E6_DIR_PIN
#define ORIG_E6_DIR_PIN -1
#endif
#ifndef ORIG_E6_ENABLE_PIN
#define ORIG_E6_ENABLE_PIN -1
#endif

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,
#define E5_PINS ORIG_E5_STEP_PIN, ORIG_E5_DIR_PIN, ORIG_E5_ENABLE_PIN,
#define E6_PINS ORIG_E6_STEP_PIN, ORIG_E6_DIR_PIN, ORIG_E6_ENABLE_PIN,

// Original pin assignmats to be used in configuration tool
#define X_STEP_PIN ORIG_X_STEP_PIN
#define X_DIR_PIN ORIG_X_DIR_PIN
#define X_ENABLE_PIN ORIG_X_ENABLE_PIN
#define X_MIN_PIN ORIG_X_MIN_PIN
#define X_MAX_PIN ORIG_X_MAX_PIN

#define Y_STEP_PIN ORIG_Y_STEP_PIN
#define Y_DIR_PIN ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN ORIG_Y_ENABLE_PIN
#define Y_MIN_PIN ORIG_Y_MIN_PIN
#define Y_MAX_PIN ORIG_Y_MAX_PIN

#define Z_STEP_PIN ORIG_Z_STEP_PIN
#define Z_DIR_PIN ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN ORIG_Z_ENABLE_PIN
#define Z_MIN_PIN ORIG_Z_MIN_PIN
#define Z_MAX_PIN ORIG_Z_MAX_PIN

#define E0_STEP_PIN ORIG_E0_STEP_PIN
#define E0_DIR_PIN ORIG_E0_DIR_PIN
#define E0_ENABLE_PIN ORIG_E0_ENABLE_PIN

#define E1_STEP_PIN ORIG_E1_STEP_PIN
#define E1_DIR_PIN ORIG_E1_DIR_PIN
#define E1_ENABLE_PIN ORIG_E1_ENABLE_PIN

#define E2_STEP_PIN ORIG_E2_STEP_PIN
#define E2_DIR_PIN ORIG_E2_DIR_PIN
#define E2_ENABLE_PIN ORIG_E2_ENABLE_PIN

#define E3_STEP_PIN ORIG_E3_STEP_PIN
#define E3_DIR_PIN ORIG_E3_DIR_PIN
#define E3_ENABLE_PIN ORIG_E3_ENABLE_PIN

#define E4_STEP_PIN ORIG_E4_STEP_PIN
#define E4_DIR_PIN ORIG_E4_DIR_PIN
#define E4_ENABLE_PIN ORIG_E4_ENABLE_PIN

#define E5_STEP_PIN ORIG_E5_STEP_PIN
#define E5_DIR_PIN ORIG_E5_DIR_PIN
#define E5_ENABLE_PIN ORIG_E5_ENABLE_PIN

#define E6_STEP_PIN ORIG_E6_STEP_PIN
#define E6_DIR_PIN ORIG_E6_DIR_PIN
#define E6_ENABLE_PIN ORIG_E6_ENABLE_PIN

#define FAN_PIN ORIG_FAN_PIN
#ifdef ORIG_FAN2_PIN
#define FAN2_PIN ORIG_FAN2_PIN
#endif

#ifndef PS_ON_PIN
#define PS_ON_PIN ORIG_PS_ON_PIN
#endif

#ifndef TEMP_0_PIN
#define TEMP_0_PIN -1
#endif

#ifndef TEMP_1_PIN
#define TEMP_1_PIN -1
#endif

#ifndef TEMP_2_PIN
#define TEMP_2_PIN -1
#endif

#ifndef TEMP_3_PIN
#define TEMP_3_PIN -1
#endif

#ifndef TEMP_4_PIN
#define TEMP_4_PIN -1
#endif

#ifndef TEMP_5_PIN
#define TEMP_5_PIN -1
#endif

#ifndef THERMOCOUPLE_0_PIN
#define THERMOCOUPLE_0_PIN -1
#endif

#ifndef THERMOCOUPLE_1_PIN
#define THERMOCOUPLE_1_PIN -1
#endif

#ifndef THERMOCOUPLE_2_PIN
#define THERMOCOUPLE_2_PIN -1
#endif

#ifndef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT -1
#endif
#ifndef SDCARDDETECT
#define SDCARDDETECT ORIG_SDCARDDETECT
#endif

#define SENSITIVE_PINS \
    { \
        0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, ORIG_PS_ON_PIN, \
            HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS E3_PINS E4_PINS E5_PINS E6_PINS TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN, TEMP_3_PIN, TEMP_4_PIN, TEMP_5_PIN, SDSS, \
            THERMOCOUPLE_0_PIN, THERMOCOUPLE_1_PIN, THERMOCOUPLE_2_PIN \
    }
#endif
