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
#undef ENDSTOP_SWITCH_DEBOUNCE
#undef ENDSTOP_STEPPER
#undef ENDSTOP_MERGE2
#undef ENDSTOP_MERGE3
#undef ENDSTOP_MERGE4

#if IO_TARGET == 4

#define ENDSTOP_NONE(name) extern EndstopNoneDriver name;
#define ENDSTOP_SWITCH(name, pin) extern EndstopSwitchDriver<pin> name;
#define ENDSTOP_SWITCH_DEBOUNCE(name, pin, level) extern EndstopSwitchDebounceDriver<pin,level> name;
#define ENDSTOP_STEPPER(name) extern EndstopStepperControlledDriver name;
#define ENDSTOP_MERGE2(name, e1, e2) extern EndstopMerge2 name;
#define ENDSTOP_MERGE3(name, e1, e2, e3) extern EndstopMerge3 name;
#define ENDSTOP_MERGE4(name, e1, e2, e3, e4) extern EndstopMerge4 name;

#elif IO_TARGET == 6

#define ENDSTOP_NONE(name) EndstopNoneDriver name;
#define ENDSTOP_SWITCH(name, pin) EndstopSwitchDriver<pin> name;
#define ENDSTOP_SWITCH_DEBOUNCE(name, pin, level) EndstopSwitchDebounceDriver<pin,level> name;
#define ENDSTOP_STEPPER(name) EndstopStepperControlledDriver name;
#define ENDSTOP_MERGE2(name, e1, e2) EndstopMerge2 name(&e1, &e2);
#define ENDSTOP_MERGE3(name, e1, e2, e3) EndstopMerge3 name(&e1, &e2, &e3);
#define ENDSTOP_MERGE4(name, e1, e2, e3, e4) EndstopMerge4 name(&e1, &e2, &e3, &e4);

#elif IO_TARGET == 5

#define ENDSTOP_NONE(name)
#define ENDSTOP_SWITCH(name, pin) name.update();
#define ENDSTOP_SWITCH_DEBOUNCE(name, pin, level) name.update();
#define ENDSTOP_STEPPER(name) name.update();
#define ENDSTOP_MERGE2(name, e1, e2) name.update();
#define ENDSTOP_MERGE3(name, e1, e2, e3) name.update();
#define ENDSTOP_MERGE4(name, e1, e2, e3, e4) name.update();

#else

#define ENDSTOP_NONE(name)
#define ENDSTOP_SWITCH(name, pin)
#define ENDSTOP_SWITCH_DEBOUNCE(name, pin, level)
#define ENDSTOP_STEPPER(name)
#define ENDSTOP_MERGE2(name, e1, e2)
#define ENDSTOP_MERGE3(name, e1, e2, e3)
#define ENDSTOP_MERGE4(name, e1, e2, e3, e4)

#endif
