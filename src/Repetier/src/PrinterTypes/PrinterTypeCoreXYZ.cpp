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

#include "../../Repetier.h"

#if PRINTER_TYPE == 1

void PrinterType::homeAxis(fast8_t axis) {
    Motion1::simpleHome(axis);
}

bool PrinterType::positionAllowed(float pos[NUM_AXES]) {
    if (Printer::isNoDestinationCheck()) {
        return true;
    }
    if (Printer::isHoming()) {
        return true;
    }
    for (fast8_t i = 0; i < 3; i++) {
        if (Motion1::axesHomed & axisBits[i]) {
            if (pos[i] < Motion1::minPos[i] || pos[i] > Motion1::maxPos[i]) {
                return false;
            }
        }
    }
    return true;
}

void PrinterType::transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]) {
    float px = lroundf(pos[X_AXIS] * Motion1::resolution[X_AXIS]);
    float py = lroundf(pos[Y_AXIS] * Motion1::resolution[Y_AXIS]);
    float pz = lroundf(pos[Z_AXIS] * Motion1::resolution[Z_AXIS]);
    motor[X_AXIS] = COREXYZ_X_X * px + COREXYZ_X_Y * py + COREXYZ_X_Z * pz;
    motor[Y_AXIS] = COREXYZ_Y_X * px + COREXYZ_Y_Y * py + COREXYZ_Y_Z * pz;
    motor[Z_AXIS] = COREXYZ_Z_X * px + COREXYZ_Z_Y * py + COREXYZ_Z_Z * pz;
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        motor[i] = lroundf(pos[i] * Motion1::resolution[i]);
    }
}

void PrinterType::disableAllowedStepper() {
    if (DISABLE_X)
        Motion1::motors[X_AXIS]->disable();
    if (DISABLE_Y)
        Motion1::motors[Y_AXIS]->disable();
    if (DISABLE_Z)
        Motion1::motors[Z_AXIS]->disable();
}

float PrinterType::accelerationForMoveSteps(fast8_t axes) {
    float acceleration = 500.0f;
    FOR_ALL_AXES(i) {
        if (axes & axisBits[i]) {
            acceleration = RMath::min(acceleration, Motion1::maxAcceleration[i]);
        }
    }
    return acceleration;
}

float PrinterType::feedrateForMoveSteps(fast8_t axes) {
    float feedrate = 100.0f;
    FOR_ALL_AXES(i) {
        if (axes & axisBits[i]) {
            feedrate = RMath::min(feedrate, Motion1::maxFeedrate[i]);
        }
    }
    return feedrate;
}

void PrinterType::deactivatedTool(fast8_t id) {}
void PrinterType::activatedTool(fast8_t id) {}
void PrinterType::eepromHandle() {}
void PrinterType::restoreFromConfiguration() {}
void PrinterType::init() {}
void PrinterType::updateDerived() {}
void PrinterType::enableMotors(fast8_t axes) {
    if (axes & 7) { // enable x,y,z as a group!
        Motion1::motors[X_AXIS]->enable();
        Motion1::motors[Y_AXIS]->enable();
        Motion1::motors[Z_AXIS]->enable();
    }
    for (fast8_t i = 3; i < NUM_AXES; i++) {
        if ((axes & axisBits[i]) != 0 && Motion1::motors[i]) {
            Motion1::motors[i]->enable();
        }
    }
    if ((axes & axisBits[E_AXIS]) != 0 && Motion1::dittoMode) {
        for (fast8_t i = 1; i <= Motion1::dittoMode; i++) {
            Tool::getTool(i)->enableMotor();
        }
    }
    Printer::unsetAllSteppersDisabled();
}
void PrinterType::setDittoMode(fast8_t count, bool mirror) {
    Motion1::dittoMode = count;
    Motion1::dittoMirror = mirror;
}

void PrinterType::transformedToOfficial(float trans[NUM_AXES], float official[NUM_AXES]) {
    Motion1::transformFromPrinter(
        trans[X_AXIS],
        trans[Y_AXIS],
        trans[Z_AXIS] - Motion1::zprobeZOffset,
        official[X_AXIS],
        official[Y_AXIS],
        official[Z_AXIS]);
    official[X_AXIS] -= Motion1::toolOffset[X_AXIS]; // Offset from active extruder or z probe
    official[Y_AXIS] -= Motion1::toolOffset[Y_AXIS];
    official[Z_AXIS] -= Motion1::toolOffset[Z_AXIS];
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        official[i] = trans[i];
    }
}

void PrinterType::officialToTransformed(float official[NUM_AXES], float trans[NUM_AXES]) {
    Motion1::transformToPrinter(official[X_AXIS] + Motion1::toolOffset[X_AXIS],
                                official[Y_AXIS] + Motion1::toolOffset[Y_AXIS],
                                official[Z_AXIS] + Motion1::toolOffset[Z_AXIS],
                                trans[X_AXIS],
                                trans[Y_AXIS],
                                trans[Z_AXIS]);
    trans[Z_AXIS] += Motion1::zprobeZOffset;
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        trans[i] = official[i];
    }
}

#endif
