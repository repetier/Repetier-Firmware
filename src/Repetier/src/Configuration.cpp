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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#undef CONFIG_EXTERN
#undef CONFIG_VARIABLE
#undef CONFIG_VARIABLE_EQ
#define CONFIG_EXTERN
#define CONFIG_VARIABLE(tp, name, values) tp name values;
#define CONFIG_VARIABLE_EQ(tp, name, values) tp name = values;

#include "Repetier.h"

// Create class instances form Configuration_io.h

#undef IO_TARGET
#define IO_TARGET IO_TARGET_DEFINE_VARIABLES
#include "io/redefine.h"

void updateEndstops() {
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ENDSTOP_UPDATE
#include "io/redefine.h"
}
