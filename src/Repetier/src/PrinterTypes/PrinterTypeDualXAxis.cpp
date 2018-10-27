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

#include "Repetier.h"

#if PRINTER_TYPE == 3

bool PrinterType::leftParked = false;
bool PrinterType::rightParked = false;
uint8_t PrinterType::lazyMode = false;
float PrinterType::posReal[2];
float PrinterType::targetReal;
bool PrinterType::dontChangeCoords = false;
float PrinterType::bedRectangle[2][2];
uint16_t PrinterType::eeprom; // start position eeprom
fast8_t PrinterType::activeAxis = 0;

void PrinterType::homeAxis(fast8_t axis) {
    // Tool::unselectTool();
    dontChangeCoords = true;
    if (axis == X_AXIS) {
        leftParked = rightParked = false;
        Motion1::simpleHome(X_AXIS);
        MCode_119(nullptr);
        Motion1::simpleHome(A_AXIS);
        Tool* tool1 = Tool::getTool(0);
        Tool* tool2 = Tool::getTool(1);
        targetReal = Motion1::minPos[X_AXIS];
        leftParked = true;
        rightParked = true;
        Motion1::currentPositionTransformed[X_AXIS] = posReal[0];
        Motion1::currentPositionTransformed[A_AXIS] = posReal[1];
        Motion1::updatePositionsFromCurrentTransformed();
        /* if (!lazyMode) {
            // Unpark active extruder
            FOR_ALL_AXES(i) {
                Motion1::tmpPosition[i] = Motion1::currentPositionTransformed[i];
            }
            if (Motion1::dittoMode) {
                Motion1::tmpPosition[X_AXIS] = Motion1::minPos[X_AXIS];
                if (Motion1::dittoMirror) {
                    Motion1::tmpPosition[A_AXIS] = Motion1::maxPos[X_AXIS] + Motion1::minPos[X_AXIS] - Motion1::tmpPosition[X_AXIS];
                } else {
                    Motion1::tmpPosition[A_AXIS] = Motion1::tmpPosition[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
                }
                leftParked = false;
                rightParked = false;
            } else if (Tool::getActiveToolId()) {
                rightParked = false;
                Motion1::tmpPosition[A_AXIS] = Motion1::maxPos[X_AXIS];
            } else {
                leftParked = false;
                Motion1::tmpPosition[X_AXIS] = Motion1::minPos[X_AXIS];
            }
            Motion1::moveByPrinter(Motion1::tmpPosition, XY_SPEED);
        } */
        dontChangeCoords = false;
        // Now both are in their park positions
    } else if (axis != A_AXIS) {
        Motion1::simpleHome(axis);
        dontChangeCoords = false;
    }
}

void PrinterType::park(GCode* com) {
    if (!Motion1::isAxisHomed(X_AXIS)) {
        Motion1::homeAxis(X_AXIS);
    }
    Motion1::copyCurrentPrinter(Motion1::tmpPosition);
    if (Motion1::dittoMode == 0) {
        if (activeAxis == 0) {
            Motion1::tmpPosition[X_AXIS] = posReal[0] + (com->hasX() ? com->X : 0);
            leftParked = Motion1::tmpPosition[X_AXIS] == posReal[0];
        } else if (activeAxis == 1) {
            Motion1::tmpPosition[A_AXIS] = posReal[1] - (com->hasX() ? com->X : 0);
            rightParked = Motion1::tmpPosition[X_AXIS] == posReal[1];
        }
    } else {
        Motion1::tmpPosition[X_AXIS] = posReal[0] + (com->hasX() ? com->X : 0);
        leftParked = Motion1::tmpPosition[X_AXIS] == posReal[0];
        Motion1::tmpPosition[A_AXIS] = posReal[1] - (com->hasX() ? com->X : 0);
        rightParked = Motion1::tmpPosition[X_AXIS] == posReal[1];
    }
    Motion1::moveByPrinter(Motion1::tmpPosition, XY_SPEED, false);
}

bool PrinterType::positionAllowed(float pos[NUM_AXES]) {
    if (Printer::isNoDestinationCheck()) {
        return true;
    }
    if (Printer::isHoming() || Motion1::endstopMode == EndstopMode::PROBING) {
        return true;
    }
    for (fast8_t i = 1; i < 3; i++) {
        if (Motion1::axesHomed & axisBits[i]) {
            if (pos[i] < Motion1::minPos[i] || pos[i] > Motion1::maxPos[i]) {
                return false;
            }
        }
    }
    // In ditto mode prevent moves past half.
    if (Motion1::dittoMode) {
        if (Motion1::dittoMirror) {
            // Extruders can collide in center, so subtract left parking distance as safety margin
            if (pos[X_AXIS] > 0.5f * (Motion1::minPos[X_AXIS] + Motion1::maxPos[X_AXIS]) - (posReal[0] - Motion1::minPos[X_AXIS])) {
                return false;
            }
        } else {
            if (pos[X_AXIS] > 0.5f * (Motion1::minPos[X_AXIS] + Motion1::maxPos[X_AXIS]) + posReal[1] - Motion1::maxPos[X_AXIS]) {
                return false;
            }
            if (pos[X_AXIS] < realPos[0]) {
                return false;
            }
        }
    } else {
        // Test X or A range depending on which is active
        if (activeAxis == 0) {
            return pos[X_AXIS] > posReal[0] && pos[X_AXIS] <= Motion1::maxPos[X_AXIS];
        } else {
            return pos[X_AXIS] >= 0 && pos[X_AXIS] <= posReal[1];
        }
    }
    return true;
}

void PrinterType::closestAllowedPositionWithNewXYOffset(float pos[NUM_AXES], float offX, float offY, float safety) {
    float offsets[3] = { offX, offY, 0 };
    float tOffMin, tOffMax;
    for (fast8_t i = 0; i < 3; i++) {
        Tool::minMaxOffsetForAxis(i, tOffMin, tOffMax);

        float p = pos[i] - offsets[i];
        float minP = Motion1::minPos[i] + safety + tOffMax - tOffMin;
        float maxP = Motion1::maxPos[i] - safety + tOffMax - tOffMin;
        if (p < minP) {
            pos[i] += minP - p;
        } else if (p > maxP) {
            pos[i] -= p - maxP;
        }
    }
}

bool PrinterType::positionOnBed(float pos[2]) {
    return pos[X_AXIS] >= bedRectangle[X_AXIS][0] && pos[X_AXIS] <= bedRectangle[X_AXIS][1] && pos[Y_AXIS] >= bedRectangle[Y_AXIS][0] && pos[Y_AXIS] <= bedRectangle[Y_AXIS][1];
}

void PrinterType::getBedRectangle(float& xmin, float& xmax, float& ymin, float& ymax) {
    xmin = bedRectangle[X_AXIS][0];
    xmax = bedRectangle[X_AXIS][1];
    ymin = bedRectangle[Y_AXIS][0];
    ymax = bedRectangle[Y_AXIS][1];
}

void PrinterType::transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]) {
    FOR_ALL_AXES(i) {
        motor[i] = lroundf(pos[i] * Motion1::resolution[i]);
    }
}

void PrinterType::disableAllowedStepper() {
    if (DISABLE_X) {
        XMotor.disable();
        AMotor.disable();
    }
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

void PrinterType::deactivatedTool(fast8_t id) {
    if (Motion1::dittoMode || (id == 0 && leftParked) || (id == 1 && rightParked) || (leftParked == false && rightParked == false)) {
        // already parked, nothing to do
        return;
    }
    // Move into park position
    COPY_ALL_AXES(Motion1::tmpPosition, Motion1::currentPositionTransformed);
    dontChangeCoords = true;
    if (id) { // right extruder
        Motion1::tmpPosition[A_AXIS] = posReal[1];
        rightParked = true;
    } else {
        Motion1::tmpPosition[X_AXIS] = posReal[0];
        leftParked = true;
    }
    Motion1::moveByPrinter(Motion1::tmpPosition, XY_SPEED, false);
    dontChangeCoords = false;
}

void PrinterType::activatedTool(fast8_t id) {
    if (Motion1::dittoMode || (id == 0 && !leftParked) || (id == 1 && !rightParked) || lazyMode) {
        return;
    }
    // Move out of park position
    COPY_ALL_AXES(Motion1::tmpPosition, Motion1::currentPositionTransformed);
    dontChangeCoords = true;
    if (Motion1::dittoMode) {
        Motion1::tmpPosition[X_AXIS] = targetReal;
        if (Motion1::dittoMirror) {
            Motion1::tmpPosition[A_AXIS] = Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS] - targetReal;
        } else {
            Motion1::tmpPosition[A_AXIS] = targetReal + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
        }
        rightParked = false;
        leftParked = false;
        activeAxis = 0;
    } else if (id) { // right extruder
        Motion1::tmpPosition[A_AXIS] = targetReal;
        rightParked = false;
        activeAxis = 1;
    } else {
        Motion1::tmpPosition[X_AXIS] = targetReal;
        leftParked = false;
        activeAxis = 0;
    }
    Motion1::moveByPrinter(Motion1::tmpPosition, XY_SPEED, false);
    dontChangeCoords = false;
}

void PrinterType::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Dual X"));
    EEPROM::handleFloat(eprStart + 9, PSTR("Bed X Min [mm]"), 2, bedRectangle[X_AXIS][0]);
    EEPROM::handleFloat(eprStart + 13, PSTR("Bed X Max [mm]"), 2, bedRectangle[X_AXIS][1]);
    EEPROM::handleFloat(eprStart + 17, PSTR("Bed Y Min [mm]"), 2, bedRectangle[Y_AXIS][0]);
    EEPROM::handleFloat(eprStart + 21, PSTR("Bed Y Max [mm]"), 2, bedRectangle[Y_AXIS][1]);
    EEPROM::handleFloat(eeprom + 0, PSTR("Offset Left [mm]"), 2, posReal[0]);
    EEPROM::handleFloat(eeprom + 4, PSTR("Offset Right [mm]"), 2, posReal[1]);
    EEPROM::handleByte(eeprom + 8, PSTR("Lazy Homing [0/1]"), lazyMode);
}

void PrinterType::restoreFromConfiguration() {
    lazyMode = LAZY_DUAL_X_AXIS;
    posReal[0] = DUAL_X_LEFT_OFFSET;
    posReal[1] = DUAL_X_RIGHT_OFFSET;
    bedRectangle[X_AXIS][0] = BED_X_MIN;
    bedRectangle[X_AXIS][1] = BED_X_MAX;
    bedRectangle[Y_AXIS][0] = BED_Y_MIN;
    bedRectangle[Y_AXIS][1] = BED_Y_MAX;

    PrinterType::updateDerived();
}

void PrinterType::init() {
    PrinterType::restoreFromConfiguration();
    eeprom = EEPROM::reserve(EEPROM_SIGNATURE_DUAL_X, 1, 25);
}

void PrinterType::updateDerived() {}

void PrinterType::enableMotors(fast8_t axes) {
    FOR_ALL_AXES(i) {
        if ((axes & axisBits[i]) != 0 && Motion1::motors[i]) {
            if (i == X_AXIS) {
                XMotor.enable();
                AMotor.enable();
            } else {
                Motion1::motors[i]->enable();
            }
        }
    }
    if ((axes & axisBits[E_AXIS]) != 0 && Motion1::dittoMode) {
        for (fast8_t i = 1; i <= Motion1::dittoMode; i++) {
            Tool::getTool(i)->enableMotor();
        }
    }
    Printer::unsetAllSteppersDisabled();
}
void PrinterType::queueMove(float feedrate, bool secondaryMove) {
    if (dontChangeCoords) { // don't think about coordinates wanted!
        return Motion1::queueMove(feedrate, secondaryMove);
    }
    targetReal = Motion1::destinationPositionTransformed[X_AXIS];
    // DEBUG_MSG2_FAST("Q1 X:", targetReal);
    // Set current to real position if parked
    if (leftParked) {
        Motion1::currentPositionTransformed[X_AXIS] = posReal[0];
    }
    if (rightParked) {
        Motion1::currentPositionTransformed[A_AXIS] = posReal[1];
    }
    // Check what we need to do
    if (leftParked && rightParked) {
        // DEBUG_MSG_FAST("Q2");
        // Seems we are in lazy mode with both tools parked, so test if we need to move one
        if (Motion1::currentPositionTransformed[E_AXIS] < Motion1::destinationPositionTransformed[E_AXIS]) {
            // DEBUG_MSG_FAST("Q4a");
            // Disable wrong X position now that we have something to do
            float backup[NUM_AXES]; // Backup old planned move
            FOR_ALL_AXES(i) {
                backup[i] = Motion1::destinationPositionTransformed[i];
                Motion1::destinationPositionTransformed[i] = Motion1::currentPositionTransformed[i];
            }
            dontChangeCoords = true;
            if (Motion1::dittoMode) {
                if (Motion1::dittoMirror) {
                    Motion1::destinationPositionTransformed[A_AXIS] = Motion1::maxPos[X_AXIS] + Motion1::minPos[X_AXIS] - targetReal;
                } else {
                    Motion1::destinationPositionTransformed[A_AXIS] = targetReal + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
                }
                Motion1::destinationPositionTransformed[X_AXIS] = targetReal;
                rightParked = false;
                leftParked = false;
            } else {
                if (Tool::getActiveToolId()) {
                    Motion1::destinationPositionTransformed[A_AXIS] = targetReal;
                    rightParked = false;
                } else {
                    Motion1::destinationPositionTransformed[X_AXIS] = targetReal;
                    leftParked = false;
                }
            }
            Motion1::queueMove(XY_SPEED, secondaryMove);
            // Motion1::waitForEndOfMoves();
            dontChangeCoords = false;
            FOR_ALL_AXES(i) {
                Motion1::destinationPositionTransformed[i] = backup[i];
            }
            targetReal = Motion1::destinationPositionTransformed[X_AXIS];
        } else {
            // DEBUG_MSG2("Q4b:", Motion1::destinationPositionTransformed[E_AXIS]);
            // Do not move X or a motor until we need to extrude
            // if (Motion1::destinationPositionTransformed[X_AXIS] != Motion1::currentPositionTransformed[X_AXIS]) {
            //   targetReal = Motion1::destinationPositionTransformed[X_AXIS];
            //}
            Motion1::destinationPositionTransformed[X_AXIS] = Motion1::currentPositionTransformed[X_AXIS];
            Motion1::destinationPositionTransformed[A_AXIS] = Motion1::currentPositionTransformed[A_AXIS];
        }
    }
    // Adjust X and A position accoridng to active tool and printing mode
    if (Motion1::dittoMode) {
        if (!leftParked) {
            if (Motion1::dittoMirror) {
                Motion1::destinationPositionTransformed[A_AXIS] = Motion1::maxPos[X_AXIS] + Motion1::minPos[X_AXIS] - Motion1::destinationPositionTransformed[X_AXIS];
            } else {
                Motion1::destinationPositionTransformed[A_AXIS] = Motion1::destinationPositionTransformed[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
            }
        }
    } else if (leftParked) {
        if (!rightParked) { // right tool active
            Motion1::destinationPositionTransformed[A_AXIS] = Motion1::destinationPositionTransformed[X_AXIS];
        }
        Motion1::destinationPositionTransformed[X_AXIS] = Motion1::currentPositionTransformed[X_AXIS];
    } else if (rightParked) { // left tool active
        Motion1::destinationPositionTransformed[A_AXIS] = Motion1::currentPositionTransformed[A_AXIS];
    }
    // Motion1::currentPosition[A_AXIS] = Motion1::destinationPositionTransformed[A_AXIS] - Motion1::toolOffset[X_AXIS];
    // DEBUG_MSG("Q6");
    return Motion1::queueMove(feedrate, secondaryMove);
}

void PrinterType::setDittoMode(fast8_t count, bool mirror) {
    Motion1::dittoMode = count;
    Motion1::dittoMirror = mirror;
    homeAxis(X_AXIS);
}

bool PrinterType::ignoreAxisForLength(fast8_t axis) {
    if (axis == A_AXIS && Motion1::dittoMode && !dontChangeCoords) {
        return true;
    }
    return false;
}

/** Converts transformed to real coordinates. Will use the 
 * official x position stored in targetReal instead of input.
 * This is ok because moves are based on reverse and thisis only called to update
 * current position. */
void PrinterType::transformedToOfficial(float trans[NUM_AXES], float official[NUM_AXES]) {
    Motion1::transformFromPrinter(
        targetReal,
        trans[Y_AXIS],
        trans[Z_AXIS] - Motion1::zprobeZOffset,
        official[X_AXIS],
        official[Y_AXIS],
        official[Z_AXIS]);
    official[X_AXIS] -= Motion1::toolOffset[X_AXIS]; // Offset from active extruder or z probe
    official[Y_AXIS] -= Motion1::toolOffset[Y_AXIS];
    official[Z_AXIS] -= Motion1::toolOffset[Z_AXIS];
    official[E_AXIS] = trans[E_AXIS];
    for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
        official[i] = trans[i];
    }
}

/** Converts official coordinates to transformed. This will not reflext parked
 * tools, so queue will still have to fix this. */
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
