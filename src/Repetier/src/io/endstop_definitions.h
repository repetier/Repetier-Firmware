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

#undef ENDSTOP_NONE
#undef ENDSTOP_SWITCH

#if IO_TARGET == 4

#define ENDSTOP_NONE(name) extern EndstopNoneDriver name;
#define ENDSTOP_SWITCH(name, pin) extern EndstopSwitchDriver<pin> name;

#elif IO_TARGET == 6

#define ENDSTOP_NONE(name) EndstopNoneDriver name;
#define ENDSTOP_SWITCH(name, pin) EndstopSwitchDriver<pin> name;

#elif IO_TARGET == 5

#define ENDSTOP_NONE(name)
#define ENDSTOP_SWITCH(name, pin) name.update();

#else

#define ENDSTOP_NONE(name)
#define ENDSTOP_SWITCH(name, pin)

#endif
