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

#undef TOOL_EXTRUDER

#if IO_TARGET == 4 // declare variable

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript) \
    extern ToolExtruder name;

#elif IO_TARGET == 6

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript) \
    ToolExtruder name(offx, offy, offz, &heater, &stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, PSTR(startScript), PSTR(endScript));

#else

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript)

#endif
