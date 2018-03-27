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
void PrinterType::init() {}
void PrinterType::updateDerived() {}

#endif
