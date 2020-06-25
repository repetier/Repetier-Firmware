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
Reset all defines for target dependent IO pin handling. Handling is defined by
IO_TARGET with following meanings:

1: Init at firmware start
2: PWM Interrupt
3: 100ms call
4: Define class
5: Endstop update
6: define variables
7: Visualization for config
8: eepromHandle calls
9: updateDerived calls
10: restore from config
11: analog input loop
12: 500ms timer
13: tools.cpp template definitions
14: Solve firmware events
15: Periodical actions
16: Controls menu (prepare, printer is idle)
17: Configuration menu
18: Tune menu (if printer is printing)
19: Main menu
20: Wizard menu
21: Store recover data
22: Restore recover data
23: init late
24: beeper loop
25: servo inerrupt
*/

// #pragma message(VAR_NAME_VALUE(IO_TARGET))

enum class GUIAction;

#include "io_input.h"
#include "io_output.h"
#include "io_spi.h"
#include "io_pwm.h"
#include "io_servos.h"
#include "io_analog.h"
#include "io_temperature.h"
#include "io_endstop_definitions.h"
#include "io_stepper.h"
#include "io_heatManager.h"
#include "io_coolerManager.h"
#include "io_tools.h"
#include "io_beeper.h"
#include "io_light.h"
#include "../custom/customMacros.h"

// Add user configuration

#include "Configuration_io.h"
