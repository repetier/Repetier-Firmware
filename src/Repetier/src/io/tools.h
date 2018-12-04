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
#undef TOOL_LASER
#undef TOOL_CNC
#undef JAM_DETECTOR_HW
#undef FILAMENT_DETECTOR

#if IO_TARGET == 4 // declare variable

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    extern ToolExtruder name; \
    extern void __attribute__((weak)) menuControl##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuConfig##name(GUIAction action, void* data);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    extern ToolLaser<toolPin, enablePin> name;

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    extern ToolCNC<dirPin, toolPin, enablePin> name;

#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    extern JamDetectorHW<inputPin, observer##Type> name; \
    extern void name##Int();

#define FILAMENT_DETECTOR(name, inputPin, tool) \
    extern FilamentDetector<inputPin> name;

#elif IO_TARGET == 6 // define variables

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    ToolExtruder name(offx, offy, offz, &heater, &stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, PSTR(startScript), PSTR(endScript), fan); \
    void __attribute__((weak)) menuControl##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Extruder @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuSelectableP(action, PSTR("Select Extruder"), selectToolAction, (void*)name.getToolId(), GUIPageType::ACTION); \
        name.getHeater()->showControlMenu(action); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuConfig##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Extruder @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuFloatP(action, PSTR("Resolution:"), name.getResolution(), 2, menuExtruderStepsPerMM, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Max Speed  :"), name.getMaxSpeed(), 0, menuExtruderMaxSpeed, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Max Accel. :"), name.getAcceleration(), 0, menuExtruderMaxAcceleration, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Max Jerk   :"), name.getMaxYank(), 1, menuExtruderMaxYank, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Diameter   :"), name.getDiameter(), 2, menuExtruderFilamentDiameter, &name, GUIPageType::FIXED_CONTENT); \
        name.getHeater()->showConfigMenu(action); \
        GUI::menuEnd(action); \
    }

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    ToolLaser<toolPin, enablePin> name(offx, offy, offz, &output, milliWatt, warmupUS, warmupPWM, bias, gamma, PSTR(startScript), PSTR(endScript));

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    ToolCNC<dirPin, toolPin, enablePin> name(offx, offy, offz, &output, rpm, startStopDelay, PSTR(startScript), PSTR(endScript));

#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    JamDetectorHW<inputPin, observer##Type> name(&observer, &tool, distanceSteps, jitterSteps, jamPercentage); \
    void name##Int() { name.interruptSignaled(); }

#define FILAMENT_DETECTOR(name, inputPin, tool) \
    FilamentDetector<inputPin> name(&tool);

#elif IO_TARGET == 10 // reset configs

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan) \
    name.reset(offx, offy, offz, diameter, resolution, yank, maxSpeed, acceleration, advance);
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    name.reset(offx, offy, offz, milliWatt, warmupUS, warmupPWM);
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    name.reset(offx, offy, offz, rpm, startStopDelay);
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.reset(distanceSteps, jitterSteps, jamPercentage);
#define FILAMENT_DETECTOR(name, inputPin, tool)

#elif IO_TARGET == 13 // template definitions in tools.cpp

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    template class ToolLaser<toolPin, enablePin>;
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    template class ToolCNC<dirPin, toolPin, enablePin>;
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    template class JamDetectorHW<inputPin, observer##Type>;
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    template class FilamentDetector<inputPin>;

#elif IO_TARGET == 1 // Setup

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    attachInterrupt(inputPin::pin(), name##Int, CHANGE);
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    name.setup();

#elif IO_TARGET == 8 // call eepromHandle if required

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.eepromHandle();
#define FILAMENT_DETECTOR(name, inputPin, tool)

#elif IO_TARGET == 14 // resolve firmware events

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    if (act.eventId == FIRMWARE_EVENT_JAM_DEBUG) { \
        Com::printF(PSTR("Jam signal "), act.param1.l); \
        Com::printF(PSTR(" switch after "), act.param2.l); \
        Com::printFLN(PSTR(" steps")); \
    }
#define FILAMENT_DETECTOR(name, inputPin, tool)

#elif IO_TARGET == 15 // Periodical actions

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.testForJam();
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    name.testFilament();

#elif IO_TARGET == 16 // Control tools manipulate menu

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    GUI::menuLongP(action, PSTR("Extruder "), name.getToolId() + 1, menuControl##name, nullptr, GUIPageType::MENU);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage)
#define FILAMENT_DETECTOR(name, inputPin, tool)

#elif IO_TARGET == 17 // config menu

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    GUI::menuLongP(action, PSTR("Extruder "), name.getToolId() + 1, menuConfig##name, nullptr, GUIPageType::MENU);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage)
#define FILAMENT_DETECTOR(name, inputPin, tool)

#else

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage)
#define FILAMENT_DETECTOR(name, inputPin, tool)

#endif
