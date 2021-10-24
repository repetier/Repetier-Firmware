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
 * Creality v4.5.3 (STM32F103RET6) board pin assignments
 */

#if NUM_TOOLS > 1
#error "Creality v4.5.3 only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define HEATER_0_PIN PB14 // HEATER1
#define HEATER_1_PIN PB13 // HOT BED
#define ORIG_FAN_PIN PB15 // FAN
// #define PROBE_ACTIVATION_SWITCH_PIN PB2 // Optoswitch to Enable Z Probe

#include "creality_v4_5_x.h"
