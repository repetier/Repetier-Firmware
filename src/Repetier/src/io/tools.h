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
#undef JAM_DETECTOR_HW

#if IO_TARGET == 4 // declare variable

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    extern ToolExtruder name;
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    extern ToolLaser<toolPin, enablePin> name;
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    extern JamDetectorHW<inputPin, observer##Type> name; \
    extern void name##Int();

#elif IO_TARGET == 6

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan) \
    ToolExtruder name(offx, offy, offz, &heater, &stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, PSTR(startScript), PSTR(endScript), fan);
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    ToolLaser<toolPin, enablePin> name(offx, offy, offz, &output, milliWatt, warmupUS, warmupPWM, bias, gamma, PSTR(startScript), PSTR(endScript));
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    JamDetectorHW<inputPin, observer##Type> name(&observer, &tool, distanceSteps, jitterSteps, jamPercentage); \
    void name##Int() { name.interruptSignaled(); }

#elif IO_TARGET == 10 // reset configs

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan) \
    name.reset(offx, offy, offz, diameter, resolution, yank, maxSpeed, acceleration, advance);
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    name.reset(offx, offy, offz, milliWatt, warmupUS, warmupPWM);
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.reset(distanceSteps, jitterSteps, jamPercentage);

#elif IO_TARGET == 13 // template definitions in tools.cpp

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript) \
    template class ToolLaser<toolPin, enablePin>;
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    template class JamDetectorHW<inputPin, observer##Type>;

#elif IO_TARGET == 1 // Setup

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    attachInterrupt(inputPin::pin(), name##Int, CHANGE);

#elif IO_TARGET == 8 // call eepromHandle if required

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.eepromHandle();

#elif IO_TARGET == 14 // resolve firmware events

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    if (act.eventId == FIRMWARE_EVENT_JAM_DEBUG) { \
        Com::printF(PSTR("Jam signal "), act.param1.l); \
        Com::printF(PSTR(" switch after "), act.param2.l); \
        Com::printFLN(PSTR(" steps")); \
    }

#elif IO_TARGET == 15 // Periodical actions

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScrip, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage) \
    name.testForJam();

#else

#define TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, diameter, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript, fan)
#define TOOL_LASER(name, offx, offy, offz, output, toolPin, enablePin, milliWatt, warmupUS, warmupPWM, bias, gamma, startScript, endScript)
#define JAM_DETECTOR_HW(name, observer, inputPin, tool, distanceSteps, jitterSteps, jamPercentage)

#endif
