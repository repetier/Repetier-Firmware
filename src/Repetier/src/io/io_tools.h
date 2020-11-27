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
#undef TOOL_CHANGE_CUSTOM_EVENT
#undef TOOL_CHANGE_SERVO
#undef TOOL_CHANGE_MERGE
#undef TOOL_CHANGE_LINK

#if IO_TARGET == IO_TARGET_CLASS_DEFINITION // declare variable

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    extern ToolExtruder name; \
    extern void __attribute__((weak)) menuControl##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuTune##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuConfig##name(GUIAction action, void* data);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    extern ToolLaser<toolPin, enablePin> name; \
    extern void __attribute__((weak)) menuControl##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuTune##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuConfig##name(GUIAction action, void* data);

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    extern ToolCNC<dirPin, toolPin, enablePin> name; \
    extern void __attribute__((weak)) menuControl##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuTune##name(GUIAction action, void* data); \
    extern void __attribute__((weak)) menuConfig##name(GUIAction action, void* data);

#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    extern JamDetectorHW<inputPin, observer##Type> name; \
    extern void name##Int(); \
    extern void __attribute__((weak)) menuConfig##name(GUIAction action, void* data);

#define FILAMENT_DETECTOR(name, inputPin, tool) \
    extern FilamentDetector<inputPin> name;

#define TOOL_CHANGE_CUSTOM_EVENT(name, tool) \
    extern ToolChangeCustomEvent name;

#define TOOL_CHANGE_SERVO(name, tool, servo, pos, timeout) \
    extern ToolChangeServo name;

#define TOOL_CHANGE_MERGE(name, tool, toolCHanger1, toolCHanger2) \
    extern ToolChangeMerge name;

#define TOOL_CHANGE_LINK(name, tool, toolCHanger) \
    extern ToolChangeLink name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // define variables

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
    void __attribute__((weak)) menuTune##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Extruder @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        name.getHeater()->showControlMenu(action); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuConfig##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Extruder @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        if (stepper.overridesResolution()) { \
            stepper.menuConfig(action, data); \
        } else { \
            GUI::menuFloatP(action, PSTR("Resolution :"), name.getResolution(), 2, menuExtruderStepsPerMM, &name, GUIPageType::FIXED_CONTENT); \
        } \
        GUI::menuFloatP(action, PSTR("Max Speed  :"), name.getMaxSpeed(), 0, menuExtruderMaxSpeed, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Max Accel. :"), name.getAcceleration(), 0, menuExtruderMaxAcceleration, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Max Jerk   :"), name.getMaxYank(), 1, menuExtruderMaxYank, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Diameter   :"), name.getDiameter(), 2, menuExtruderFilamentDiameter, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset X   :"), name.getOffsetX(), 2, menuToolOffsetX, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Y   :"), name.getOffsetY(), 2, menuToolOffsetY, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Z   :"), name.getOffsetZ(), 2, menuToolOffsetZ, &name, GUIPageType::FIXED_CONTENT); \
        if (name.changeHandler) { \
            name.changeHandler->configMenu(action); \
        } \
        if (name.coolantHandler) { \
            name.coolantHandler->configMenu(action); \
        } \
        name.getHeater()->showConfigMenu(action); \
        GUI::menuEnd(action); \
    }

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    ToolLaser<toolPin, enablePin> name(offx, offy, offz, &output, milliWatt, warmupUS, warmupPWM, bias, gamma, PSTR(startScript), PSTR(endScript)); \
    void __attribute__((weak)) menuControl##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Laser @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuSelectableP(action, PSTR("Select Laser"), selectToolAction, (void*)name.getToolId(), GUIPageType::ACTION); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuTune##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Laser @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuConfig##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Laser @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuFloatP(action, PSTR("Power       :"), name.getMilliWatt(), 0, ToolLaser<toolPin, enablePin>::menuToolLaserMilliWatt, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuLongP(action, PSTR("Warmup Power:"), name.getWarmupPower(), ToolLaser<toolPin, enablePin>::menuToolLaserWarmupPower, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuLongP(action, PSTR("Warmup Time :"), name.getWarmupTime(), ToolLaser<toolPin, enablePin>::menuToolLaserWarmupTime, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset X    :"), name.getOffsetX(), 2, menuToolOffsetX, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Y    :"), name.getOffsetY(), 2, menuToolOffsetY, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Z    :"), name.getOffsetZ(), 2, menuToolOffsetZ, &name, GUIPageType::FIXED_CONTENT); \
        if (name.changeHandler) { \
            name.changeHandler->configMenu(action); \
        } \
        if (name.coolantHandler) { \
            name.coolantHandler->configMenu(action); \
        } \
        GUI::menuEnd(action); \
    }

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    ToolCNC<dirPin, toolPin, enablePin> name(offx, offy, offz, &output, rpm, startStopDelay, PSTR(startScript), PSTR(endScript)); \
    void __attribute__((weak)) menuControl##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= CNC @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuSelectableP(action, PSTR("Select CNC"), selectToolAction, (void*)name.getToolId(), GUIPageType::ACTION); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuTune##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= CNC @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuEnd(action); \
    } \
    void __attribute__((weak)) menuConfig##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= CNC @ ="), name.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuFloatP(action, PSTR("Max. RPM :"), name.getRPM(), 0, ToolCNC<dirPin, toolPin, enablePin>::menuRPM, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset X :"), name.getOffsetX(), 2, menuToolOffsetX, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Y :"), name.getOffsetY(), 2, menuToolOffsetY, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuFloatP(action, PSTR("Offset Z :"), name.getOffsetZ(), 2, menuToolOffsetZ, &name, GUIPageType::FIXED_CONTENT); \
        if (name.changeHandler) { \
            name.changeHandler->configMenu(action); \
        } \
        if (name.coolantHandler) { \
            name.coolantHandler->configMenu(action); \
        } \
        GUI::menuEnd(action); \
    }

#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    JamDetectorHW<inputPin, observer##Type> name(&observer, &tool, distanceSteps, jitterSteps, jamPercentage); \
    void name##Int() { name.interruptSignaled(); } \
    void __attribute__((weak)) menuConfig##name(GUIAction action, void* data) { \
        GUI::menuStart(action); \
        char help[MAX_COLS]; \
        GUI::flashToStringLong(help, PSTR("= Jam Detector @ ="), tool.getToolId() + 1); \
        GUI::menuText(action, help, true); \
        GUI::menuBack(action); \
        GUI::menuLongP(action, PSTR("Sig. Distance:"), name.getDistanceSteps(), JamDetectorHW<inputPin, observer##Type>::menuDistanceSteps, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuLongP(action, PSTR("Jitter Tresh.:"), name.getJitterSteps(), JamDetectorHW<inputPin, observer##Type>::menuJitterSteps, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuLongP(action, PSTR("Jam Treshold :"), name.getJamPercentage(), JamDetectorHW<inputPin, observer##Type>::menuJamPercentage, &name, GUIPageType::FIXED_CONTENT); \
        GUI::menuEnd(action); \
    }

#define FILAMENT_DETECTOR(name, inputPin, tool) \
    FilamentDetector<inputPin> name(&tool);

#define TOOL_CHANGE_CUSTOM_EVENT(name, tool) \
    ToolChangeCustomEvent name(static_cast<Tool*>(&tool));

#define TOOL_CHANGE_SERVO(name, tool, servo, pos, timeout) \
    ToolChangeServo name(static_cast<Tool*>(&tool), &servo, pos, timeout);

#define TOOL_CHANGE_MERGE(name, tool, toolChanger1, toolChanger2) \
    ToolChangeMerge name(static_cast<Tool*>(&tool), static_cast<Tool*>(&toolChanger), static_cast<Tool*>(&toolCHanger2));

#define TOOL_CHANGE_LINK(name, tool, toolChanger) \
    ToolChangeLink name(static_cast<Tool*>(&tool), static_cast<ToolChangeHandler*>(&toolChanger));

#elif IO_TARGET == IO_TARGET_RESTORE_FROM_CONFIG // reset configs

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan) \
    name.reset(offx, offy, offz, diameter, resolution, yank, maxSpeed, acceleration, advance);
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    name.reset(offx, offy, offz, milliWatt, warmupUS, warmupPWM);
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    name.reset(offx, offy, offz, rpm, startStopDelay);
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.reset(distanceSteps, jitterSteps, jamPercentage);

#elif IO_TARGET == IO_TARGET_TEMPLATES // template definitions in tools.cpp

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    template class ToolLaser<toolPin, enablePin>;
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    template class ToolCNC<dirPin, toolPin, enablePin>;
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    template class JamDetectorHW<inputPin, observer##Type>;
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    template class FilamentDetector<inputPin>;

#elif IO_TARGET == IO_TARGET_INIT // Setup

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    attachInterrupt(inputPin::pin(), name##Int, CHANGE);
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    name.setup();
#define TOOL_CHANGE_CUSTOM_EVENT(name, tool) name.setup(&tool);

#elif IO_TARGET == IO_TARGET_EEPROM // call eepromHandle if required

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.eepromHandle();

#elif IO_TARGET == IO_TARGET_FIRMWARE_EVENTS // resolve firmware events

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    if (act.eventId == FIRMWARE_EVENT_JAM_DEBUG) { \
        Com::printF(PSTR("Jam signal "), act.param1.l); \
        Com::printF(PSTR(" switch after "), act.param2.l); \
        Com::printFLN(PSTR(" steps")); \
    }

#elif IO_TARGET == IO_TARGET_PERIODICAL_ACTIONS // Periodical actions

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.testForJam();
#define FILAMENT_DETECTOR(name, inputPin, tool) \
    name.testFilament();

#elif IO_TARGET == IO_TARGET_GUI_CONTROLS // Control tools manipulate menu

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    GUI::menuLongP(action, PSTR("Extruder "), name.getToolId() + 1, menuControl##name, nullptr, GUIPageType::MENU);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    GUI::menuLongP(action, PSTR("Laser "), name.getToolId() + 1, menuControl##name, nullptr, GUIPageType::MENU);

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    GUI::menuLongP(action, PSTR("CNC "), name.getToolId() + 1, menuControl##name, nullptr, GUIPageType::MENU);

#elif IO_TARGET == IO_TARGET_GUI_CONFIG // config menu

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    GUI::menuLongP(action, PSTR("Extruder "), name.getToolId() + 1, menuConfig##name, nullptr, GUIPageType::MENU);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    GUI::menuLongP(action, PSTR("Laser "), name.getToolId() + 1, menuConfig##name, nullptr, GUIPageType::MENU);

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    GUI::menuLongP(action, PSTR("CNC "), name.getToolId() + 1, menuConfig##name, nullptr, GUIPageType::MENU);

#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    GUI::menuLongP(action, PSTR("Jam Detector "), tool.getToolId() + 1, menuConfig##name, nullptr, GUIPageType::MENU);

#elif IO_TARGET == IO_TARGET_GUI_TUNE // Control tune manipulate menu

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    GUI::menuLongP(action, PSTR("Extruder "), name.getToolId() + 1, menuTune##name, nullptr, GUIPageType::MENU);

#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    GUI::menuLongP(action, PSTR("Laser "), name.getToolId() + 1, menuTune##name, nullptr, GUIPageType::MENU);

#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript) \
    GUI::menuLongP(action, PSTR("CNC "), name.getToolId() + 1, menuTune##name, nullptr, GUIPageType::MENU);

#endif

#ifndef TOOL_EXTRUDER
#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan)
#endif
#ifndef TOOL_LASER
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#endif
#ifndef TOOL_CNC
#define TOOL_CNC(name, offx, offy, offz, output, dirPin, toolPin, enablePin, rpm, startStopDelay, startScript, endScript)
#endif
#ifndef JAM_DETECTOR_HW
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage)
#endif
#ifndef FILAMENT_DETECTOR
#define FILAMENT_DETECTOR(name, inputPin, tool)
#endif
#ifndef TOOL_CHANGE_CUSTOM_EVENT
#define TOOL_CHANGE_CUSTOM_EVENT(name, tool)
#endif
#ifndef TOOL_CHANGE_SERVO
#define TOOL_CHANGE_SERVO(name, tool, servo, pos, timeout)
#endif
#ifndef TOOL_CHANGE_MERGE
#define TOOL_CHANGE_MERGE(name, tool, toolChanger1, toolChanger2)
#endif
#ifndef TOOL_CHANGE_LINK
#define TOOL_CHANGE_LINK(name, tool, toolChanger)
#endif
