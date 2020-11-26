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

#if Z_PROBE_REPETITIONS == 1
#undef Z_PROBE_USE_MEDIAN
#define Z_PROBE_USE_MEDIAN 0
#endif

#define Z_CRASH_THRESHOLD_STEPS 50
#if Z_PROBE_TYPE == Z_PROBE_TYPE_DEFAULT

uint16_t ZProbeHandler::eprStart;
float ZProbeHandler::height;
float ZProbeHandler::bedDistance;
float ZProbeHandler::coating;
float ZProbeHandler::offsetX;
float ZProbeHandler::offsetY;
float ZProbeHandler::speed;
bool ZProbeHandler::activated;
uint16_t ZProbeHandler::userPausedHeaters;
bool ZProbeHandler::pauseHeaters;

float ZProbeHandler::getZProbeHeight() {
    return height;
}

void ZProbeHandler::setZProbeHeight(float _height) {
    height = _height;
}

bool ZProbeHandler::activate() {
    if (activated) {
        return true;
    }
    // Ensure x and y positions are valid
    if (!Motion1::isAxisHomed(X_AXIS) || !Motion1::isAxisHomed(Y_AXIS)) {
        Motion1::homeAxes((Motion1::isAxisHomed(X_AXIS) ? 0 : 1) + (Motion1::isAxisHomed(Y_AXIS) ? 0 : 2));
    }
    Printer::setZProbingActive(true);
    float cPos[NUM_AXES];
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, offsetX, offsetY, Z_PROBE_BORDER);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    GCode::executeFString(Com::tZProbeStartScript);
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-offsetX, -offsetY, 0);
    Motion1::setHardwareEndstopsAttached(true, ZProbe);
    activated = true;
    if (pauseHeaters) {
        bool set = false;
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i]->isPaused() || heaters[i]->isOff()) {
                userPausedHeaters |= (1 << i);
            } else {
                userPausedHeaters &= ~(1 << i);
                heaters[i]->pause();
                set = true;
            }
        }
        if (set) {
            HAL::delayMilliseconds(150);
        }
    }
    return true;
}

void ZProbeHandler::deactivate() {
    if (!activated) {
        return;
    }
    Motion1::setHardwareEndstopsAttached(false, ZProbe);
    float cPos[NUM_AXES];
    Tool* tool = Tool::getActiveTool();
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, tool->getOffsetX(), tool->getOffsetY(), Z_PROBE_BORDER);
    GCode::executeFString(Com::tZProbeEndScript);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-tool->getOffsetX(), -tool->getOffsetY(), -tool->getOffsetZ());
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    activated = false;
    Printer::setZProbingActive(false);
    if (pauseHeaters) {
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i]->isPaused()) {
                if (!((userPausedHeaters >> i) & 1)) {
                    heaters[i]->unpause();
                }
            }
        }
    }
}

float ZProbeHandler::runProbe(uint8_t repetitions, bool useMedian) {
    Motion1::callBeforeHomingOnSteppers();
    float measurements[repetitions = constrain(repetitions, 1u, 10u)] = { 0.0f };
    if (ZProbe->update()) {
        Com::printErrorFLN(PSTR("z-probe triggered before starting probing."));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }
    if (Leveling::isDistortionEnabled()) {
        Com::printWarningFLN(PSTR("z-probe with distortion enabled can return unexpected results!"));
    }
    bool wasActivated = activated;
    if (!activated) {
        activate();
    }
    bool alActive = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false, true);
    EndstopMode oldMode = Motion1::endstopMode;
    Motion1::endstopMode = EndstopMode::PROBING;
    Motion1::waitForEndOfMoves(); // defined starting condition
    Motion1::stopMask = 4u;       // z trigger is finished
    float cPos[NUM_AXES], tPos[NUM_AXES];
    int32_t cPosSteps[NUM_AXES], tPosSteps[NUM_AXES], corSteps[NUM_AXES];
    Motion1::copyCurrentPrinter(cPos);
    PrinterType::transform(cPos, cPosSteps);
    Motion1::copyCurrentPrinter(tPos);

    float secureDistance = (Motion1::maxPos[Z_AXIS] - Motion1::minPos[Z_AXIS]) * 1.5f;
    if (Motion1::currentPosition[Z_AXIS] > 0.5f * ZProbeHandler::optimumProbingHeight() + 0.1f && fabsf(Motion1::currentPosition[Z_AXIS] - ZProbeHandler::optimumProbingHeight()) < 1.0f) {
        secureDistance = getBedDistance() * 1.5f;
    }
    tPos[Z_AXIS] -= secureDistance;
    PrinterType::transform(tPos, tPosSteps);
    int32_t secureSteps = lround(secureDistance * Motion1::resolution[Z_AXIS]);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
    HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
    Motion1::stepsRemaining[Z_AXIS] = 0;
    tPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByPrinter(tPos, speed, false);
    Motion1::waitForEndOfMoves();
    Motion1::endstopMode = EndstopMode::DISABLED;

    if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
        Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }
    float z = secureDistance * ((fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]));
    measurements[0u] = z;
    FOR_ALL_AXES(i) {
        int32_t df = cPosSteps[i] - tPosSteps[i];
        corSteps[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
    }
    float zr = 0.0f;
    if (repetitions > 1u) {
        // We are now at z=0 do repetitions and assume correct them 100%
        float cPos2[NUM_AXES], tPos2[NUM_AXES], tPos3[NUM_AXES];
        FOR_ALL_AXES(i) {
            cPos2[i] = cPos[i];
            tPos2[i] = cPos[i];
            tPos3[i] = cPos[i];
        }
        // Go up to start position for repeated tests
        cPos2[Z_AXIS] = 0.0f;
        FOR_ALL_AXES(i) {
            Motion1::currentPositionTransformed[i] = cPos2[i];
        }
        Motion1::updatePositionsFromCurrentTransformed();
        Motion2::setMotorPositionFromTransformed();
        Motion1::endstopMode = EndstopMode::DISABLED;
        tPos2[Z_AXIS] = Z_PROBE_SWITCHING_DISTANCE;
        tPos3[Z_AXIS] = -1.0f;
        int32_t cPosSteps2[NUM_AXES], tPosSteps2[NUM_AXES], tPosSteps3[NUM_AXES], corSteps2[NUM_AXES] = { 0 };
        PrinterType::transform(cPos2, cPosSteps2);
        PrinterType::transform(tPos2, tPosSteps2);
        PrinterType::transform(tPos3, tPosSteps3);
        tPos2[E_AXIS] = IGNORE_COORDINATE;
        Motion1::moveByPrinter(tPos2, Motion1::maxFeedrate[X_AXIS], false);
        Motion1::waitForEndOfMoves();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
        GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

        for (fast8_t r = 1; r < repetitions && !Printer::breakLongCommand; r++) {
            // Go down
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
            HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
            Motion1::stepsRemaining[Z_AXIS] = 0;
            Motion1::endstopMode = EndstopMode::PROBING;
            tPos3[E_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByPrinter(tPos3, speed, false);
            Motion1::waitForEndOfMoves();
            Motion1::endstopMode = EndstopMode::DISABLED;

            if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
                Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
                Motion1::callAfterHomingOnSteppers();
                return ILLEGAL_Z_PROBE;
            }

            if (useMedian) {
                measurements[r] = z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            } else {
                zr += z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            }
            // Go up to tPos2
            FOR_ALL_AXES(i) {
                int32_t df = tPosSteps2[i] - tPosSteps3[i];
                corSteps2[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
            }
            Motion1::moveRelativeBySteps(corSteps2);
            Motion1::waitForEndOfMoves();
            FOR_ALL_AXES(i) {
                Motion1::currentPositionTransformed[i] = tPos2[i];
            }
            Motion1::updatePositionsFromCurrentTransformed();
            Motion2::setMotorPositionFromTransformed();
        }
        if (useMedian) {
            //Com::printArrayFLN("measure: ", measurements, ARRAY_SIZE(measurements), 4);
            float tmp;
            for (fast8_t i = 0; i < repetitions - 1; i++) {         // n numbers require at most n-1 rounds of swapping
                for (fast8_t j = 0; j < repetitions - i - 1; j++) { //
                    if (measurements[j] > measurements[j + 1]) {    // out of order?
                        tmp = measurements[j];                      // swap them:
                        measurements[j] = measurements[j + 1];
                        measurements[j + 1] = tmp;
                    }
                }
            }
            // Take median result
            z = static_cast<float>(measurements[repetitions >> 1u]);
        } else {
            z = (z + zr) / static_cast<float>(repetitions);
        }
        // Fix last correction
        FOR_ALL_AXES(i) {
            corSteps[i] -= corSteps2[i];
        }
    }
    z += height;
    z -= coating;
    /* DEBUG_MSG2_FAST("StartSteps", cPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("EndSteps", tPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("CorSteps", corSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("RemainingSteps", Motion1::stepsRemaining[Z_AXIS]);
    DEBUG_MSG2_FAST("Z:", z); */

    Motion1::moveRelativeBySteps(corSteps);
    Motion1::waitForEndOfMoves();

    // Restore starting position
    FOR_ALL_AXES(i) {
        Motion1::currentPositionTransformed[i] = cPos[i];
    }
    Motion1::updatePositionsFromCurrentTransformed();
    Motion2::setMotorPositionFromTransformed();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
    GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

    Motion1::endstopMode = oldMode;
    Motion1::setAutolevelActive(alActive, true);
    if (!wasActivated) {
        deactivate();
    }
    if (ZProbe->update()) {
        millis_t startTime = HAL::timeInMilliseconds();
        // wait up to 200ms to be sure signal stays
        while (ZProbe->update() && ((HAL::timeInMilliseconds() - startTime) < 200ul)) {
            Commands::checkForPeriodicalActions(false);
        }

        if (ZProbe->update()) {
            Com::printErrorFLN(PSTR("z-probe did not untrigger after moving back to start position."));
            Motion1::callAfterHomingOnSteppers();
            return ILLEGAL_Z_PROBE;
        }
    }
    Motion1::waitForEndOfMoves();
    Com::printF(Com::tZProbe, z, 3);
    Com::printF(Com::tSpaceXColon, Motion1::currentPosition[X_AXIS]);
    Com::printFLN(Com::tSpaceYColon, Motion1::currentPosition[Y_AXIS]);
    Motion1::callAfterHomingOnSteppers();
    return z;
}

bool ZProbeHandler::probingPossible() {
    return true;
}

float ZProbeHandler::xOffset() {
    return offsetX;
}

float ZProbeHandler::yOffset() {
    return offsetY;
}

void ZProbeHandler::init() {
    eepromReset();
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_Z_PROBE, 1, 25);
    activated = false;
}

void ZProbeHandler::eepromReset() {
    height = Z_PROBE_HEIGHT;
    bedDistance = Z_PROBE_BED_DISTANCE;
    speed = Z_PROBE_SPEED;
    offsetX = Z_PROBE_X_OFFSET;
    offsetY = Z_PROBE_Y_OFFSET;
    coating = Z_PROBE_COATING;
    pauseHeaters = Z_PROBE_PAUSE_HEATERS;
}

void ZProbeHandler::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Z-probe"));
    EEPROM::handleFloat(eprStart + 0, PSTR("Trigger height [mm]"), 3, height);
    EEPROM::handleFloat(eprStart + 20, PSTR("Coating [mm]"), 3, coating);
    EEPROM::handleFloat(eprStart + 4, PSTR("Min. nozzle distance [mm]"), 3, bedDistance);
    EEPROM::handleFloat(eprStart + 8, PSTR("Probing speed [mm/s]"), 3, speed);
    EEPROM::handleFloat(eprStart + 12, PSTR("X offset [mm]"), 3, offsetX);
    EEPROM::handleFloat(eprStart + 16, PSTR("Y offset [mm]"), 3, offsetY);
    EEPROM::handleByte(eprStart + 24, PSTR("Pause heaters [0/1]"), pauseHeaters);
    EEPROM::removePrefix();
}

float ZProbeHandler::optimumProbingHeight() {
    return bedDistance + (height > 0 ? 0 : -height);
}

void __attribute__((weak)) menuProbeCoating(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Coating Height:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, ZProbeHandler::getCoating(), 2);
    if (GUI::handleFloatValueAction(action, v, 0.0f, 5.0f, 0.01f)) {
        ZProbeHandler::setCoating(v);
    }
}
void __attribute__((weak)) menuProbeOffset(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data); // 0 = x, 1 = y
    GUI::flashToStringFlash(GUI::tmpString, PSTR("@ Offset:"), axis ? axisNames[Y_AXIS] : axisNames[X_AXIS]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM,
               axis ? ZProbeHandler::yOffset() : ZProbeHandler::xOffset(), 1);
    if (GUI::handleFloatValueAction(action, v, -100.0f, 100.0f, 0.5f)) {
        if (axis) {
            ZProbeHandler::setYOffset(v);
        } else {
            ZProbeHandler::setXOffset(v);
        }
    }
}
void ZProbeHandler::showConfigMenu(GUIAction action) {
    GUI::menuFloatP(action, PSTR("Coat. Height:"), ZProbeHandler::getCoating(), 2, menuProbeCoating, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("X Offset    :"), ZProbeHandler::xOffset(), 1, menuProbeOffset, reinterpret_cast<void*>(0), GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Y Offset    :"), ZProbeHandler::yOffset(), 1, menuProbeOffset, reinterpret_cast<void*>(1), GUIPageType::FIXED_CONTENT);
}
#endif

#if Z_PROBE_TYPE == Z_PROBE_TYPE_NOZZLE

uint16_t ZProbeHandler::eprStart;
float ZProbeHandler::height;
float ZProbeHandler::bedDistance;
float ZProbeHandler::speed;
bool ZProbeHandler::activated;
int16_t ZProbeHandler::probeTemperature;
int16_t ZProbeHandler::activateTemperature;
uint16_t ZProbeHandler::userPausedHeaters;
bool ZProbeHandler::pauseHeaters;

float ZProbeHandler::getZProbeHeight() {
    return height;
}

void ZProbeHandler::setZProbeHeight(float _height) {
    height = _height;
}

bool ZProbeHandler::activate() {
    if (activated) {
        return true;
    }
    // Ensure x and y positions are valid
    if (!Motion1::isAxisHomed(X_AXIS) || !Motion1::isAxisHomed(Y_AXIS)) {
        Motion1::homeAxes((Motion1::isAxisHomed(X_AXIS) ? 0 : 1) + (Motion1::isAxisHomed(Y_AXIS) ? 0 : 2));
    }
    Printer::setZProbingActive(true);
    float cPos[NUM_AXES];
    Tool* tool = Tool::getActiveTool();
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, tool->getOffsetX(), tool->getOffsetY(), Z_PROBE_BORDER);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    GCode::executeFString(Com::tZProbeStartScript);
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-tool->getOffsetX(), -tool->getOffsetY(), -tool->getOffsetZ());
    Motion1::setHardwareEndstopsAttached(true, ZProbe);
    Tool* t = Tool::getActiveTool();
    HeatManager* hm = t->getHeater();
    if (hm != nullptr) {
        activateTemperature = hm->getTargetTemperature();
        if (probeTemperature > activateTemperature) {
            hm->setTargetTemperature(probeTemperature);
            hm->waitForTargetTemperature();
        }
    }
    activated = true;
    if (pauseHeaters) {
        bool set = false;
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i] == hm && probeTemperature > activateTemperature) {
                continue; // skip active nozzle if it requires heating!
            }
            if (heaters[i]->isPaused() || heaters[i]->isOff()) {
                userPausedHeaters |= (1 << i);
            } else {
                userPausedHeaters &= ~(1 << i);
                heaters[i]->pause();
                set = true;
            }
        }
        if (set) {
            HAL::delayMilliseconds(150);
        }
    }
    return true;
}

void ZProbeHandler::deactivate() {
    if (!activated) {
        return;
    }
    Motion1::setHardwareEndstopsAttached(false, ZProbe);
    float cPos[NUM_AXES];
    Motion1::copyCurrentOfficial(cPos);
    Tool* tool = Tool::getActiveTool();
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, tool->getOffsetX(), tool->getOffsetY(), Z_PROBE_BORDER);
    GCode::executeFString(Com::tZProbeEndScript);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-tool->getOffsetX(), -tool->getOffsetY(), -tool->getOffsetZ());
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    HeatManager* hm = tool->getHeater();
    if (hm != nullptr) {
        hm->setTargetTemperature(activateTemperature);
    }
    activated = false;
    Printer::setZProbingActive(false);
    if (pauseHeaters) {
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i]->isPaused()) {
                if (!((userPausedHeaters >> i) & 1)) {
                    heaters[i]->unpause();
                }
            }
        }
    }
}

float ZProbeHandler::runProbe(uint8_t repetitions, bool useMedian) {
    Motion1::callBeforeHomingOnSteppers();
    float measurements[repetitions = constrain(repetitions, 1u, 10u)] = { 0.0f };
    if (ZProbe->update()) {
        Com::printErrorFLN(PSTR("z-probe triggered before starting probing."));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }
    if (Leveling::isDistortionEnabled()) {
        Com::printWarningFLN(PSTR("z-probe with distortion enabled can return unexpected results!"));
    }
    bool wasActivated = activated;
    if (!activated) {
        activate();
    }
    bool alActive = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false, true);
    EndstopMode oldMode = Motion1::endstopMode;
    Motion1::endstopMode = EndstopMode::PROBING;
    Motion1::waitForEndOfMoves(); // defined starting condition
    Motion1::stopMask = 4u;       // z trigger is finished
    float cPos[NUM_AXES], tPos[NUM_AXES];
    int32_t cPosSteps[NUM_AXES], tPosSteps[NUM_AXES], corSteps[NUM_AXES];
    Motion1::copyCurrentPrinter(cPos);
    PrinterType::transform(cPos, cPosSteps);
    Motion1::copyCurrentPrinter(tPos);

    float secureDistance = (Motion1::maxPos[Z_AXIS] - Motion1::minPos[Z_AXIS]) * 1.5f;
    if (Motion1::currentPosition[Z_AXIS] > 0.5f * ZProbeHandler::optimumProbingHeight() + 0.1f && fabsf(Motion1::currentPosition[Z_AXIS] - ZProbeHandler::optimumProbingHeight()) < 1.0f) {
        secureDistance = getBedDistance() * 1.5f;
    }
    tPos[Z_AXIS] -= secureDistance;
    PrinterType::transform(tPos, tPosSteps);
    int32_t secureSteps = lround(secureDistance * Motion1::resolution[Z_AXIS]);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
    HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
    Motion1::stepsRemaining[Z_AXIS] = 0;
    tPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByPrinter(tPos, speed, false);
    Motion1::waitForEndOfMoves();
    Motion1::endstopMode = EndstopMode::DISABLED;

    if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
        Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }

    float z = secureDistance * ((fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]));
    measurements[0u] = z;
    FOR_ALL_AXES(i) {
        int32_t df = cPosSteps[i] - tPosSteps[i];
        corSteps[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
    }
    float zr = 0.0f;
    if (repetitions > 1u) {
        // We are now at z=0 do repetitions and assume correct them 100%
        float cPos2[NUM_AXES], tPos2[NUM_AXES], tPos3[NUM_AXES];
        FOR_ALL_AXES(i) {
            cPos2[i] = cPos[i];
            tPos2[i] = cPos[i];
            tPos3[i] = cPos[i];
        }
        // Go up to start position for repeated tests
        cPos2[Z_AXIS] = 0.0f;
        FOR_ALL_AXES(i) {
            Motion1::currentPositionTransformed[i] = cPos2[i];
        }
        Motion1::updatePositionsFromCurrentTransformed();
        Motion2::setMotorPositionFromTransformed();
        Motion1::endstopMode = EndstopMode::DISABLED;
        tPos2[Z_AXIS] = Z_PROBE_SWITCHING_DISTANCE;
        tPos3[Z_AXIS] = -1.0f;
        int32_t cPosSteps2[NUM_AXES], tPosSteps2[NUM_AXES], tPosSteps3[NUM_AXES], corSteps2[NUM_AXES] = { 0 };
        PrinterType::transform(cPos2, cPosSteps2);
        PrinterType::transform(tPos2, tPosSteps2);
        PrinterType::transform(tPos3, tPosSteps3);
        tPos2[E_AXIS] = IGNORE_COORDINATE;
        Motion1::moveByPrinter(tPos2, Motion1::maxFeedrate[X_AXIS], false);
        Motion1::waitForEndOfMoves();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
        GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

        for (fast8_t r = 1; r < repetitions && !Printer::breakLongCommand; r++) {
            // Go down
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
            HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
            Motion1::stepsRemaining[Z_AXIS] = 0;
            Motion1::endstopMode = EndstopMode::PROBING;
            tPos3[E_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByPrinter(tPos3, speed, false);
            Motion1::waitForEndOfMoves();
            Motion1::endstopMode = EndstopMode::DISABLED;

            if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
                Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
                Motion1::callAfterHomingOnSteppers();
                return ILLEGAL_Z_PROBE;
            }

            if (useMedian) {
                measurements[r] = z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            } else {
                zr += z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            }
            // Go up to tPos2
            FOR_ALL_AXES(i) {
                int32_t df = tPosSteps2[i] - tPosSteps3[i];
                corSteps2[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
            }
            Motion1::moveRelativeBySteps(corSteps2);
            Motion1::waitForEndOfMoves();
            FOR_ALL_AXES(i) {
                Motion1::currentPositionTransformed[i] = tPos2[i];
            }
            Motion1::updatePositionsFromCurrentTransformed();
            Motion2::setMotorPositionFromTransformed();
        }
        if (useMedian) {
            //Com::printArrayFLN("measure: ", measurements, ARRAY_SIZE(measurements), 4);
            float tmp;
            for (fast8_t i = 0; i < repetitions - 1; i++) {         // n numbers require at most n-1 rounds of swapping
                for (fast8_t j = 0; j < repetitions - i - 1; j++) { //
                    if (measurements[j] > measurements[j + 1]) {    // out of order?
                        tmp = measurements[j];                      // swap them:
                        measurements[j] = measurements[j + 1];
                        measurements[j + 1] = tmp;
                    }
                }
            }
            // Take median result
            z = static_cast<float>(measurements[repetitions >> 1u]);
        } else {
            z = (z + zr) / static_cast<float>(repetitions);
        }
        // Fix last correction
        FOR_ALL_AXES(i) {
            corSteps[i] -= corSteps2[i];
        }
    }
    z += height;
    /* DEBUG_MSG2_FAST("StartSteps", cPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("EndSteps", tPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("CorSteps", corSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("RemainingSteps", Motion1::stepsRemaining[Z_AXIS]);
    DEBUG_MSG2_FAST("Z:", z); */

    Motion1::moveRelativeBySteps(corSteps);
    Motion1::waitForEndOfMoves();

    // Restore starting position
    FOR_ALL_AXES(i) {
        Motion1::currentPositionTransformed[i] = cPos[i];
    }
    Motion1::updatePositionsFromCurrentTransformed();
    Motion2::setMotorPositionFromTransformed();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
    GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

    Motion1::endstopMode = oldMode;
    Motion1::setAutolevelActive(alActive, true);
    if (!wasActivated) {
        deactivate();
    }
    if (ZProbe->update()) {
        millis_t startTime = HAL::timeInMilliseconds();
        // wait up to 200ms to be sure signal stays
        while (ZProbe->update() && ((HAL::timeInMilliseconds() - startTime) < 200ul)) {
            Commands::checkForPeriodicalActions(false);
        }

        if (ZProbe->update()) {
            Com::printErrorFLN(PSTR("z-probe did not untrigger after moving back to start position."));
            Motion1::callAfterHomingOnSteppers();
            return ILLEGAL_Z_PROBE;
        }
    }
    Motion1::waitForEndOfMoves();
    Com::printF(Com::tZProbe, z, 3);
    Com::printF(Com::tSpaceXColon, Motion1::currentPosition[X_AXIS]);
    Com::printFLN(Com::tSpaceYColon, Motion1::currentPosition[Y_AXIS]);
    Motion1::callAfterHomingOnSteppers();
    return z;
}

bool ZProbeHandler::probingPossible() {
    return true;
}

float ZProbeHandler::xOffset() {
    Tool* tool = Tool::getActiveTool();
    return tool->getOffsetX();
}

float ZProbeHandler::yOffset() {
    Tool* tool = Tool::getActiveTool();
    return tool->getOffsetY();
}

void ZProbeHandler::init() {
    eepromReset();
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_Z_PROBE, 2, 15);
    activated = false;
    activateTemperature = 0;
}

void ZProbeHandler::eepromReset() {
    height = Z_PROBE_HEIGHT;
    bedDistance = Z_PROBE_BED_DISTANCE;
    speed = Z_PROBE_SPEED;
    probeTemperature = Z_PROBE_TEMPERATURE;
    pauseHeaters = Z_PROBE_PAUSE_HEATERS;
}

void ZProbeHandler::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Z-probe"));
    EEPROM::handleFloat(eprStart + 0, PSTR("Trigger height [mm]"), 3, height);
    EEPROM::handleFloat(eprStart + 4, PSTR("Min. nozzle distance [mm]"), 3, bedDistance);
    EEPROM::handleFloat(eprStart + 8, PSTR("Probing speed [mm/s]"), 3, speed);
    EEPROM::handleInt(eprStart + 12, PSTR("Min. nozzle temp [deg C]"), probeTemperature);
    EEPROM::handleByte(eprStart + 14, PSTR("Pause heaters [0/1]"), pauseHeaters);
    EEPROM::removePrefix();
}

float ZProbeHandler::optimumProbingHeight() {
    return bedDistance + (height > 0 ? 0 : -height);
}

void __attribute__((weak)) menuProbeTemperature(GUIAction action, void* data) {
    int maxTemp = reinterpret_cast<int>(data);
    GUI::flashToString(GUI::tmpString, PSTR("Min. Nozzle Temp :"));
    DRAW_LONG(GUI::tmpString, Com::tUnitDegCelsius, ZProbeHandler::getProbingTemp());
    if (GUI::handleLongValueAction(action, v, 0, maxTemp, 5)) {
        ZProbeHandler::setProbingTemp(v);
    }
}
void ZProbeHandler::showConfigMenu(GUIAction action) {
    Tool* t = Tool::getActiveTool();
    HeatManager* hm = t->getHeater();
    if (hm != nullptr) {
        int maxTemp = hm->getMaxTemperature();
        GUI::menuLongP(action, PSTR("Nozzle Temp:"), ZProbeHandler::getProbingTemp(), menuProbeTemperature, reinterpret_cast<void*>(maxTemp), GUIPageType::FIXED_CONTENT);
    }
}
#endif

#if Z_PROBE_TYPE == Z_PROBE_TYPE_BLTOUCH

uint16_t ZProbeHandler::eprStart;
float ZProbeHandler::height;
float ZProbeHandler::bedDistance;
float ZProbeHandler::offsetX;
float ZProbeHandler::offsetY;
float ZProbeHandler::speed;
bool ZProbeHandler::activated;
uint16_t ZProbeHandler::userPausedHeaters;
int16_t ZProbeHandler::deployDelay;
bool ZProbeHandler::pauseHeaters;
float ZProbeHandler::getZProbeHeight() {
    return height;
}

void ZProbeHandler::setZProbeHeight(float _height) {
    height = _height;
}

bool ZProbeHandler::activate() {
    if (activated) {
        return true;
    }
    disableAlarmIfOn();
    // Ensure x and y positions are valid
    if (!Motion1::isAxisHomed(X_AXIS) || !Motion1::isAxisHomed(Y_AXIS)) {
        Motion1::homeAxes((Motion1::isAxisHomed(X_AXIS) ? 0 : 1) + (Motion1::isAxisHomed(Y_AXIS) ? 0 : 2));
    }
    Printer::setZProbingActive(true);
    float cPos[NUM_AXES];
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, offsetX, offsetY, Z_PROBE_BORDER);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    GCode::executeFString(Com::tZProbeStartScript);
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-offsetX, -offsetY, 0);
    Motion1::setHardwareEndstopsAttached(true, ZProbe);
    ZProbeServo.setPosition(647, 0);     // deploy pin
    HAL::delayMilliseconds(deployDelay); // give time to deploy
    if (isAlarmOn()) {                   // to close to bed, alarm triggered already from deploy, so raise 5mm
        Com::printWarningFLN(PSTR("Z-probe triggered before probing - raising z!"));
        cPos[Z_AXIS] += 5.0;
        Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[Z_AXIS], false);
        Motion1::waitForEndOfMoves();
        disableAlarmIfOn();
        ZProbeServo.setPosition(647, 0);     // deploy pin
        HAL::delayMilliseconds(deployDelay); // give time to deploy
        if (isAlarmOn()) {
            Motion1::setHardwareEndstopsAttached(false, ZProbe);
            // if BLTouch gives Alarm again, needle is blocked, stop printer
            Motion1::waitForEndOfMoves();                                                                                   // this avoids hard moves with very high feedrates and lost steps (where do they come from? only happens in case distortion correction is active.
            GCode::fatalError(PSTR("Z-Probe alarm triggered again before probing! Printer stopped to avoid a bed crash!")); // Z is raised to Z=30, probably by the fatalError
            return false;
        }
    }
    activated = true;
    if (pauseHeaters) {
        bool set = false;
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i]->isPaused() || heaters[i]->isOff()) {
                userPausedHeaters |= (1 << i);
            } else {
                userPausedHeaters &= ~(1 << i);
                heaters[i]->pause();
                set = true;
            }
        }
        if (set) {
            HAL::delayMilliseconds(150);
        }
    }
    return true;
}

void ZProbeHandler::deactivate() {
    if (!activated) {
        return;
    }
    ZProbeServo.setPosition(1473, 0); // stow pin
    Motion1::setHardwareEndstopsAttached(false, ZProbe);
    float cPos[NUM_AXES];
    Tool* tool = Tool::getActiveTool();
    Motion1::copyCurrentOfficial(cPos);
    PrinterType::closestAllowedPositionWithNewXYOffset(cPos, tool->getOffsetX(), tool->getOffsetY(), Z_PROBE_BORDER);
    GCode::executeFString(Com::tZProbeEndScript);
    cPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    Motion1::setToolOffset(-tool->getOffsetX(), -tool->getOffsetY(), -tool->getOffsetZ());
    Motion1::moveByOfficial(cPos, Motion1::moveFeedrate[X_AXIS], false);
    activated = false;
    Printer::setZProbingActive(false);
    if (pauseHeaters) {
        for (size_t i = 0; i < NUM_HEATERS; i++) {
            if (heaters[i]->isPaused()) {
                if (!((userPausedHeaters >> i) & 1)) {
                    heaters[i]->unpause();
                }
            }
        }
    }
}

float ZProbeHandler::runProbe(uint8_t repetitions, bool useMedian) {
    Motion1::callBeforeHomingOnSteppers();
    float measurements[repetitions = constrain(repetitions, 1u, 10u)] = { 0.0f };
    if (ZProbe->update()) {
        Com::printErrorFLN(PSTR("z-probe triggered before starting probing."));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }
    if (Leveling::isDistortionEnabled()) {
        Com::printWarningFLN(PSTR("z-probe with distortion enabled can return unexpected results!"));
    }
    bool wasActivated = activated;
    if (!activated) {
        activate();
    }
    bool alActive = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false, true);
    EndstopMode oldMode = Motion1::endstopMode;
    Motion1::endstopMode = EndstopMode::PROBING;
    Motion1::waitForEndOfMoves(); // defined starting condition
    Motion1::stopMask = 4u;       // z trigger is finished
    float cPos[NUM_AXES], tPos[NUM_AXES];
    int32_t cPosSteps[NUM_AXES], tPosSteps[NUM_AXES], corSteps[NUM_AXES];
    Motion1::copyCurrentPrinter(cPos);
    PrinterType::transform(cPos, cPosSteps);
    Motion1::copyCurrentPrinter(tPos);

    float secureDistance = (Motion1::maxPos[Z_AXIS] - Motion1::minPos[Z_AXIS]) * 1.5f;
    if (Motion1::currentPosition[Z_AXIS] > 0.5f * ZProbeHandler::optimumProbingHeight() + 0.1f && fabsf(Motion1::currentPosition[Z_AXIS] - ZProbeHandler::optimumProbingHeight()) < 1.0f) {
        secureDistance = getBedDistance() * 1.5f;
    }
    tPos[Z_AXIS] -= secureDistance;
    PrinterType::transform(tPos, tPosSteps);
    int32_t secureSteps = lround(secureDistance * Motion1::resolution[Z_AXIS]);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
    HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
    Motion1::stepsRemaining[Z_AXIS] = 0;
    tPos[E_AXIS] = IGNORE_COORDINATE;
    Motion1::moveByPrinter(tPos, speed, false);
    Motion1::waitForEndOfMoves();
    Motion1::endstopMode = EndstopMode::DISABLED;

    if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
        Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
        Motion1::callAfterHomingOnSteppers();
        return ILLEGAL_Z_PROBE;
    }

    float z = secureDistance * ((fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps[Z_AXIS] - cPosSteps[Z_AXIS]));
    measurements[0u] = z;
    FOR_ALL_AXES(i) {
        int32_t df = cPosSteps[i] - tPosSteps[i];
        corSteps[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
    }
    float zr = 0.0f;
    if (repetitions > 1u) {
        // We are now at z=0 do repetitions and assume correct them 100%
        float cPos2[NUM_AXES], tPos2[NUM_AXES], tPos3[NUM_AXES];
        FOR_ALL_AXES(i) {
            cPos2[i] = cPos[i];
            tPos2[i] = cPos[i];
            tPos3[i] = cPos[i];
        }
        // Go up to start position for repeated tests
        cPos2[Z_AXIS] = 0.0f;
        FOR_ALL_AXES(i) {
            Motion1::currentPositionTransformed[i] = cPos2[i];
        }
        Motion1::updatePositionsFromCurrentTransformed();
        Motion2::setMotorPositionFromTransformed();

        Motion1::endstopMode = EndstopMode::DISABLED;
        tPos2[Z_AXIS] = Z_PROBE_SWITCHING_DISTANCE;
        tPos3[Z_AXIS] = -1.0f;
        int32_t cPosSteps2[NUM_AXES], tPosSteps2[NUM_AXES], tPosSteps3[NUM_AXES], corSteps2[NUM_AXES] = { 0 };

        PrinterType::transform(cPos2, cPosSteps2);
        PrinterType::transform(tPos2, tPosSteps2);
        PrinterType::transform(tPos3, tPosSteps3);
        tPos2[E_AXIS] = IGNORE_COORDINATE;
        Motion1::moveByPrinter(tPos2, Motion1::maxFeedrate[X_AXIS], false);
        Motion1::waitForEndOfMoves();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
        GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

        for (fast8_t r = 1; r < repetitions && !Printer::breakLongCommand; r++) {
            // Go down
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0
            HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
            Motion1::stepsRemaining[Z_AXIS] = 0;
            Motion1::endstopMode = EndstopMode::PROBING;
            tPos3[E_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByPrinter(tPos3, speed, false);
            Motion1::waitForEndOfMoves();
            Motion1::endstopMode = EndstopMode::DISABLED;

            if (Motion1::stepsRemaining[Z_AXIS] < Z_CRASH_THRESHOLD_STEPS) {
                Com::printErrorFLN(PSTR("Failed to trigger probe endstop! Bed crash?"));
                Motion1::callAfterHomingOnSteppers();
                return ILLEGAL_Z_PROBE;
            }

            if (useMedian) {
                measurements[r] = z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            } else {
                zr += z - 1.0f + (Z_PROBE_SWITCHING_DISTANCE + 1.0f) * ((fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]) - Motion1::stepsRemaining[Z_AXIS]) / fabsf(tPosSteps3[Z_AXIS] - tPosSteps2[Z_AXIS]));
            }
            // Go up to tPos2
            FOR_ALL_AXES(i) {
                int32_t df = tPosSteps2[i] - tPosSteps3[i];
                corSteps2[i] = ((df > 0) ? 1 : ((df < 0) ? -1 : 0)) * (labs(df) - Motion1::stepsRemaining[i]);
            }
            Motion1::moveRelativeBySteps(corSteps2);
            Motion1::waitForEndOfMoves();
            FOR_ALL_AXES(i) {
                Motion1::currentPositionTransformed[i] = tPos2[i];
            }
            Motion1::updatePositionsFromCurrentTransformed();
            Motion2::setMotorPositionFromTransformed();
        }
        if (useMedian) {
            //Com::printArrayFLN("measure: ", measurements, ARRAY_SIZE(measurements), 4);
            float tmp;
            for (fast8_t i = 0; i < repetitions - 1; i++) {         // n numbers require at most n-1 rounds of swapping
                for (fast8_t j = 0; j < repetitions - i - 1; j++) { //
                    if (measurements[j] > measurements[j + 1]) {    // out of order?
                        tmp = measurements[j];                      // swap them:
                        measurements[j] = measurements[j + 1];
                        measurements[j + 1] = tmp;
                    }
                }
            }
            // Take median result
            z = static_cast<float>(measurements[repetitions >> 1u]);
        } else {
            z = (z + zr) / static_cast<float>(repetitions);
        }
        // Fix last correction
        FOR_ALL_AXES(i) {
            corSteps[i] -= corSteps2[i];
        }
    }
    z += height;
    /* DEBUG_MSG2_FAST("StartSteps", cPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("EndSteps", tPosSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("CorSteps", corSteps[Z_AXIS]);
    DEBUG_MSG2_FAST("RemainingSteps", Motion1::stepsRemaining[Z_AXIS]);
    DEBUG_MSG2_FAST("Z:", z); */

    Motion1::moveRelativeBySteps(corSteps);
    Motion1::waitForEndOfMoves();

    // Restore starting position
    FOR_ALL_AXES(i) {
        Motion1::currentPositionTransformed[i] = cPos[i];
    }
    Motion1::updatePositionsFromCurrentTransformed();
    Motion2::setMotorPositionFromTransformed();
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
    GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif

    Motion1::endstopMode = oldMode;
    Motion1::setAutolevelActive(alActive, true);
    if (!wasActivated) {
        deactivate();
    }
    if (ZProbe->update()) {
        millis_t startTime = HAL::timeInMilliseconds();
        // wait up to 200ms to be sure signal stays
        while (ZProbe->update() && ((HAL::timeInMilliseconds() - startTime) < 200ul)) {
            Commands::checkForPeriodicalActions(false);
        }

        if (ZProbe->update()) {
            Com::printErrorFLN(PSTR("z-probe did not untrigger after moving back to start position."));
            Motion1::callAfterHomingOnSteppers();
            return ILLEGAL_Z_PROBE;
        }
    }
    Motion1::waitForEndOfMoves();
    Com::printF(Com::tZProbe, z, 3);
    Com::printF(PSTR(" err:"), z - cPos[Z_AXIS], 3);
    Com::printF(Com::tSpaceXColon, Motion1::currentPosition[X_AXIS]);
    Com::printFLN(Com::tSpaceYColon, Motion1::currentPosition[Y_AXIS]);
    Motion1::callAfterHomingOnSteppers();
    return z;
}

bool ZProbeHandler::probingPossible() {
    return true;
}

float ZProbeHandler::xOffset() {
    return offsetX;
}

float ZProbeHandler::yOffset() {
    return offsetY;
}

void ZProbeHandler::init() {
    eepromReset();
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_Z_PROBE, 1, 23);
    activated = false;
}

void ZProbeHandler::eepromReset() {
    height = Z_PROBE_HEIGHT;
    bedDistance = Z_PROBE_BED_DISTANCE;
    speed = Z_PROBE_SPEED;
    offsetX = Z_PROBE_X_OFFSET;
    offsetY = Z_PROBE_Y_OFFSET;
    pauseHeaters = Z_PROBE_PAUSE_HEATERS;
    deployDelay = Z_PROBE_BLTOUCH_DEPLOY_DELAY;
}

void ZProbeHandler::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Z-probe"));
    EEPROM::handleFloat(eprStart + 0, PSTR("Trigger height [mm]"), 3, height);
    EEPROM::handleFloat(eprStart + 4, PSTR("Min. nozzle distance [mm]"), 3, bedDistance);
    EEPROM::handleFloat(eprStart + 8, PSTR("Probing speed [mm/s]"), 3, speed);
    EEPROM::handleFloat(eprStart + 12, PSTR("X offset [mm]"), 3, offsetX);
    EEPROM::handleFloat(eprStart + 16, PSTR("Y offset [mm]"), 3, offsetY);
    EEPROM::handleByte(eprStart + 20, PSTR("Pause heaters [0/1]"), pauseHeaters);
    EEPROM::handleInt(eprStart + 21, PSTR("Deploy delay [ms]"), deployDelay);
    EEPROM::removePrefix();
}

float ZProbeHandler::optimumProbingHeight() {
    return bedDistance + (height > 0 ? 0 : -height);
}

bool ZProbeHandler::isAlarmOn() {
    bool on = ZProbe->triggered();
    if (!on) {
        return false;
    }
    HAL::delayMilliseconds(20); // Normal high stays only 10ms so double test time for safety
    return ZProbe->triggered();
}

void ZProbeHandler::disableAlarmIfOn() {
    bool startAttach = ZProbe->isAttached();
    Motion1::setHardwareEndstopsAttached(true, ZProbe);
    millis_t startTime = HAL::timeInMilliseconds();
    while (isAlarmOn() && (HAL::timeInMilliseconds() - startTime) < 1500) {
        ZProbeServo.setPosition(2194, 0);
    }
    ZProbeServo.setPosition(1473, 0); // pin up
    Motion1::setHardwareEndstopsAttached(startAttach, ZProbe);
}

void menuProbeOffset(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data); // 0 = x, 1 = y
    GUI::flashToStringFlash(GUI::tmpString, PSTR("@ Offset:"), axis ? axisNames[Y_AXIS] : axisNames[X_AXIS]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM,
               axis ? ZProbeHandler::yOffset() : ZProbeHandler::xOffset(), 1);
    if (GUI::handleFloatValueAction(action, v, -100.0f, 100.0f, 0.5f)) {
        if (axis) {
            ZProbeHandler::setYOffset(v);
        } else {
            ZProbeHandler::setXOffset(v);
        }
    }
}

void menuProbeDeployDelay(GUIAction action, void* data) {
    DRAW_LONG_P(PSTR("Delay: "), Com::tUnitMilliSeconds, ZProbeHandler::getDeployDelay());
    if (GUI::handleLongValueAction(action, v, 0, 5000, 10)) {
        ZProbeHandler::setDeployDelay(v);
    }
}

void ZProbeHandler::showConfigMenu(GUIAction action) {
    GUI::menuLongP(action, PSTR("Deploy Delay:"), ZProbeHandler::getDeployDelay(), menuProbeDeployDelay, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("X Offset    :"), ZProbeHandler::xOffset(), 1, menuProbeOffset, reinterpret_cast<void*>(0), GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Y Offset    :"), ZProbeHandler::yOffset(), 1, menuProbeOffset, reinterpret_cast<void*>(1), GUIPageType::FIXED_CONTENT);
}

void menuBLTouchAlarmRelease(GUIAction action, void* data) {
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        ZProbeServo.setPosition(2194, 0);
    }
}
void menuBLTouchStow(GUIAction action, void* data) {
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        ZProbeServo.setPosition(1473, 0);
    }
}

void menuBLTouchDeploy(GUIAction action, void* data) {
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        ZProbeServo.setPosition(647, 0);
    }
}

void menuBLTouchSelfTest(GUIAction action, void* data) {
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        ZProbeServo.setPosition(1782, 0);
    }
}

void ZProbeHandler::showControlMenu(GUIAction action) {
    GUI::menuSelectableP(action, PSTR("Deploy Pin"), menuBLTouchDeploy, nullptr, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Stow Pin"), menuBLTouchStow, nullptr, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Self Test"), menuBLTouchSelfTest, nullptr, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Alarm Release"), menuBLTouchAlarmRelease, nullptr, GUIPageType::ACTION);
}
#endif
