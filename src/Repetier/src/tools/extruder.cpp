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

#include "Repetier.h"

void ToolExtruder::reset(float offx, float offy, float offz, float diameter, float resolution, float yank, float maxSpeed, float acceleration, float advance) {
    resetBase(offx, offy, offz);
    this->diameter = diameter;
    stepsPerMM = resolution;
    this->yank = yank;
    this->acceleration = acceleration;
    this->maxSpeed = maxSpeed;
    this->advance = advance;
}

/// Called when the tool gets activated.
void ToolExtruder::activate() {
    Motion1::setMotorForAxis(stepper, E_AXIS);
    Motion1::maxYank[E_AXIS] = yank;
    Motion1::resolution[E_AXIS] = stepsPerMM;
    Motion1::currentPosition[E_AXIS] = Motion1::currentPositionTransformed[E_AXIS] = 0.0f;
    Motion1::maxFeedrate[E_AXIS] = maxSpeed;
    Motion1::maxAcceleration[E_AXIS] = Motion1::maxTravelAcceleration[E_AXIS] = acceleration;
    Motion1::advanceK = advance;
    if (changeHandler) {
        changeHandler->activate(this);
    }
    GCode::executeFString(startScript);
    Motion1::waitForEndOfMoves();
}

/// Gets called when the tool gets disabled.
void ToolExtruder::deactivate() {
    GCode::executeFString(endScript);
    if (changeHandler) {
        changeHandler->deactivate(this);
    }
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}

void ToolExtruder::copySettingsToMotion1() {
    Motion1::maxYank[E_AXIS] = yank;
    Motion1::resolution[E_AXIS] = stepsPerMM;
    Motion1::maxFeedrate[E_AXIS] = maxSpeed;
    Motion1::maxAcceleration[E_AXIS] = Motion1::maxTravelAcceleration[E_AXIS] = acceleration;
    Motion1::advanceK = advance;
}
/// Called on kill/emergency to disable the tool
void ToolExtruder::shutdown() {
    heater->setTargetTemperature(0);
    stepper->disable();
}

void ToolExtruder::init() {
    setEepromStart(EEPROM::reserve(EEPROM_SIGNATURE_EXTRUDER, 1, Tool::eepromSize() + 6 * 4));
}

void ToolExtruder::eepromHandle() {
    Tool::eepromHandle();
    uint pos = getEepromStart() + Tool::eepromSize();
    if (!stepper->overridesResolution()) {
        EEPROM::handleFloat(pos, PSTR("Steps per mm [steps/mm]"), 2, stepsPerMM);
    }
    EEPROM::handleFloat(pos + 4, PSTR("Yank [mm/s]"), 2, yank);
    EEPROM::handleFloat(pos + 8, PSTR("Max Speed [mm/s]"), 2, maxSpeed);
    EEPROM::handleFloat(pos + 12, PSTR("Acceleration [mm/s^3]"), 2, acceleration);
    EEPROM::handleFloat(pos + 16, PSTR("Advance [steps/mm]"), 2, advance);
    EEPROM::handleFloat(pos + 20, PSTR("Diameter [mm]"), 2, diameter);
    stepper->eepromHandle();
}

void ToolExtruder::setAdvance(float adv) {
    advance = adv;
    if (this == Tool::getActiveTool()) {
        Motion1::advanceK = advance;
    }
}

void ToolExtruder::updateDerived() {
    Motion1::advanceK = advance;
    Tool* t = Tool::getActiveTool();
    Motion1::setMotorForAxis(stepper, E_AXIS);
    Motion1::setToolOffset(-t->getOffsetX(), -t->getOffsetY(), -t->getOffsetZ());
}

void ToolExtruder::disableMotor() {
    stepper->disable();
}

void ToolExtruder::enableMotor() {
    stepper->enable();
    Printer::unsetAllSteppersDisabled();
}

void ToolExtruder::stepMotor() {
    stepper->step();
}

void ToolExtruder::unstepMotor() {
    stepper->unstep();
}

bool ToolExtruder::updateMotor() {
    return stepper->updateEndstop();
}

void ToolExtruder::directionMotor(bool dir) {
    stepper->dir(dir);
}

void ToolExtruder::retract(bool backwards, bool longRetract) {
    if (!Motion1::retractSpeed
        || (!Motion1::retractLength && !longRetract)
        || (!Motion1::retractLongLength && longRetract)) {
        // invalid settings
        return;
    }

    if ((Motion1::retracted && backwards) || (!Motion1::retracted && !backwards)) {
        return;
    }

    float prevSpeed = Printer::feedrate;
    if (Motion1::retractZLift && Motion1::isAxisHomed(Z_AXIS)) {
        float offset = -(getOffsetZ() + (backwards ? -Motion1::retractZLift : 0.0f));

        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE,
                                   IGNORE_COORDINATE,
                                   Motion1::currentPosition[Z_AXIS] + offset - Motion1::toolOffset[Z_AXIS]);
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], true); // Z-Hops are G1's.
        Motion1::toolOffset[Z_AXIS] = offset;
        Motion1::updatePositionsFromCurrentTransformed();

        //Motion1::setToolOffset(-getOffsetX(), -getOffsetY(), -(getOffsetZ() + offset));
    }
    float amount = 0.0f;
    if (!longRetract) {
        amount = backwards ? -Motion1::retractLength
                           : (Motion1::retractLength + Motion1::retractUndoExtraLength);
    } else {
        amount = backwards ? -Motion1::retractLongLength
                           : (Motion1::retractLongLength + Motion1::retractUndoExtraLongLength);
    }

    amount *= Printer::extrusionFactor;

    float speed = Motion1::retractSpeed;
    if (backwards && Motion1::retractUndoSpeed) {
        speed = Motion1::retractUndoSpeed;
    }
    speed *= 0.01f * static_cast<float>(Printer::feedrateMultiply);

    float prevE = Motion1::currentPositionTransformed[E_AXIS];
    Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, amount);
    Motion1::moveRelativeByPrinter(Motion1::tmpPosition, speed, false);
    Motion1::currentPositionTransformed[E_AXIS] = prevE;
    Motion1::updatePositionsFromCurrentTransformed();

    Printer::feedrate = prevSpeed;
    Motion1::retracted = backwards;
}


// ------------ JamDetectorHW ------------

template <class inputPin, class ObserverType>
JamDetectorHW<inputPin, ObserverType>::JamDetectorHW(ObserverType* _observer, Tool* _tool, int32_t _distanceSteps, int32_t _jitterSteps, int32_t _jamPercentage) {
    observer = _observer;
    tool = _tool;
    distanceSteps = _distanceSteps;
    jitterSteps = _jitterSteps;
    jamPercentage = _jamPercentage;
    errorSteps = (distanceSteps * jamPercentage) / 100;
    lastSignal = 0;
    eepromStart = EEPROM::reserve(EEPROM_SIGNATURE_JAM, 1, 3 * 4);
}

template <class inputPin, class ObserverType>
void JamDetectorHW<inputPin, ObserverType>::reset(int32_t _distanceSteps, int32_t _jitterSteps, int32_t _jamPercentage) {
    distanceSteps = _distanceSteps;
    jitterSteps = _jitterSteps;
    jamPercentage = _jamPercentage;
    errorSteps = (distanceSteps * jamPercentage) / 100;
}

template <class inputPin, class ObserverType>
void JamDetectorHW<inputPin, ObserverType>::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Jam Detector"), tool->getToolId() + 1);
    EEPROM::handleLong(eepromStart + 0, PSTR("Steps between signal changes [steps]"), distanceSteps);
    EEPROM::handleLong(eepromStart + 4, PSTR("Jitter signal range [steps]"), jitterSteps);
    EEPROM::handleLong(eepromStart + 8, PSTR("Jam signal at [%]"), jamPercentage);
    EEPROM::removePrefix();
    errorSteps = (distanceSteps * jamPercentage) / 100;
}

template <class inputPin, class ObserverType>
void JamDetectorHW<inputPin, ObserverType>::testForJam() {
    if (Tool::getActiveTool() != tool) {
        return;
    }
    if (Printer::isTestJamRequired()) {
        int32_t diff = labs(static_cast<int32_t>(observer->position - lastSignal));
        if (diff > errorSteps && !tool->hasError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT)) { // handle error
            tool->setError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT);
            EVENT_JAM_DETECTED;
            Com::printFLN(PSTR("important:Extruder jam detected"));
#if SDSUPPORT
            if (sd.state == SDState::SD_PRINTING) {
                sd.pausePrint(true);
                EVENT_JAM_DETECTED_END;
                return;
            }
#endif // SDSUPPORT
            GCodeSource::printAllFLN(PSTR("// action:out_of_filament T"), (int32_t)tool->getToolId());
            GCodeSource::printAllFLN(PSTR("RequestPause:Extruder Jam Detected!"));
            // GCodeSource::printAllFLN(PSTR("// action:pause")); // add later when host/server know new meaning!
            EVENT_JAM_DETECTED_END;
        }
    }
}

template <class inputPin, class ObserverType>
void JamDetectorHW<inputPin, ObserverType>::interruptSignaled() {
    if (!Printer::isJamcontrolDisabled()) {
        int32_t diff = labs(static_cast<int32_t>(observer->position - lastSignal));
        if (diff < jitterSteps) { // ignore swinging state at edge
            return;
        }
        if (diff < errorSteps) {
            tool->resetError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT);
        }
        if (Printer::isDebugJam()) { // report signal
            FirmwareEvent::queueEvent(FIRMWARE_EVENT_JAM_DEBUG, wizardVar(static_cast<int32_t>(tool->getToolId())), wizardVar(diff));
        }
    }
    lastSignal = observer->position;
}

template <class inputPin, class ObserverType>
void __attribute__((weak)) JamDetectorHW<inputPin, ObserverType>::menuDistanceSteps(GUIAction action, void* data) {
    JamDetectorHW<inputPin, ObserverType>* ext = reinterpret_cast<JamDetectorHW<inputPin, ObserverType>*>(data);
    DRAW_LONG_P(PSTR("Signal Distance:"), Com::tUnitSteps, ext->distanceSteps);
    if (GUI::handleLongValueAction(action, v, 0, 10000, 1)) {
        ext->distanceSteps = v;
    }
}

template <class inputPin, class ObserverType>
void __attribute__((weak)) JamDetectorHW<inputPin, ObserverType>::menuJitterSteps(GUIAction action, void* data) {
    JamDetectorHW<inputPin, ObserverType>* ext = reinterpret_cast<JamDetectorHW<inputPin, ObserverType>*>(data);
    DRAW_LONG_P(PSTR("Jitter Treshold:"), Com::tUnitSteps, ext->jitterSteps);
    if (GUI::handleLongValueAction(action, v, 0, 10000, 1)) {
        ext->jitterSteps = v;
    }
}

template <class inputPin, class ObserverType>
void __attribute__((weak)) JamDetectorHW<inputPin, ObserverType>::menuJamPercentage(GUIAction action, void* data) {
    JamDetectorHW<inputPin, ObserverType>* ext = reinterpret_cast<JamDetectorHW<inputPin, ObserverType>*>(data);
    DRAW_LONG_P(PSTR("Jam Treshold:"), Com::tUnitPercent, ext->jamPercentage);
    if (GUI::handleLongValueAction(action, v, 0, 1000, 1)) {
        ext->jamPercentage = v;
    }
}

// ----------------- FilamentDetector -------------

template <class inputPin>
FilamentDetector<inputPin>::FilamentDetector(Tool* _tool) {
    tool = _tool;
    lastFound = HAL::timeInMilliseconds();
}

template <class inputPin>
void FilamentDetector<inputPin>::setup() {
    if (!inputPin::get()) { // prevent error messages at startup
        tool->setError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT);
    }
}

template <class inputPin>
void FilamentDetector<inputPin>::testFilament() {
    if (Tool::getActiveTool() != tool) { // trigger only if active
        return;
    }
    if (inputPin::get()) {
        lastFound = HAL::timeInMilliseconds();
        if (tool->hasError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT)) {
            tool->resetError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT);
            Com::printFLN(PSTR("Filament inserted"));
        }
    } else if (Printer::isTestJamRequired()) {
        if (!tool->hasError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT) && (HAL::timeInMilliseconds() - lastFound) > 2000) { // handle error
            tool->setError(TOOL_ERROR_JAMMED_OR_NO_FILAMENT);
            EVENT_JAM_DETECTED;
            Com::printFLN(PSTR("important:No Filament detected"));
#if SDSUPPORT
            if (sd.state == SDState::SD_PRINTING) {
                sd.pausePrint(true);
                EVENT_JAM_DETECTED_END;
                return;
            }
#endif // SDSUPPORT
            GCodeSource::printAllFLN(PSTR("// action:out_of_filament T"), (int32_t)tool->getToolId());
            GCodeSource::printAllFLN(PSTR("RequestPause:No filament detected!"));
            // GCodeSource::printAllFLN(PSTR("// action:pause")); // add later when host/server know new meaning!
            EVENT_JAM_DETECTED_END;
        }
    }
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
