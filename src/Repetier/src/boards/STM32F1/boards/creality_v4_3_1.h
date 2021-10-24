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
 * CREALITY v4.3.1 (STM32F103) board pin assignments
 */

//
// Steppers
//
#define ORIG_X_STEP_PIN PB8
#define ORIG_X_DIR_PIN PB7

#define ORIG_Y_STEP_PIN PC2
#define ORIG_Y_DIR_PIN PB9

#include "creality_v4.h"
