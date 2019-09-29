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

#if PRINTER_TYPE == PRINTER_TYPE_DUAL_X

bool PrinterType::leftParked = false;
bool PrinterType::rightParked = false;
uint8_t PrinterType::lazyMode = false;
float PrinterType::endPos[2];
float PrinterType::targetReal;              // Official position
bool PrinterType::dontChangeCoords = false; // if true queueMove will not adjust positions!
float PrinterType::bedRectangle[2][2];
uint16_t PrinterType::eeprom; // start position eeprom
fast8_t PrinterType::activeAxis = 0;

void PrinterType::homeAxis(fast8_t axis) {
    if (axis == X_AXIS) {
        dontChangeCoords = true;
        leftParked = rightParked = false;
        Motion1::simpleHome(X_AXIS);
        MCode_119(nullptr);
        Motion1::simpleHome(A_AXIS);
        targetReal = /* lazyMode ? Motion1::minPos[X_AXIS] : */ endPos[activeAxis];
        leftParked = rightParked = lazyMode;
        Motion1::currentPositionTransformed[X_AXIS] = endPos[0];
        Motion1::currentPositionTransformed[A_AXIS] = endPos[1];
        Motion1::updatePositionsFromCurrentTransformed();
        Motion2::setMotorPositionFromTransformed();
        // Now both are in their park positions
        if (!lazyMode && Motion1::dittoMode) {
            FOR_ALL_AXES(i) {
                Motion1::tmpPosition[i] = Motion1::currentPositionTransformed[i];
            }
            Motion1::tmpPosition[X_AXIS] = Motion1::minPos[X_AXIS];
            if (Motion1::dittoMirror) {
                Motion1::tmpPosition[A_AXIS] = Motion1::maxPos[X_AXIS];
            } else {
                Motion1::tmpPosition[A_AXIS] = Motion1::tmpPosition[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
            }
            targetReal = Motion1::tmpPosition[X_AXIS];
            leftParked = rightParked = false;
            Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
            Motion1::updatePositionsFromCurrentTransformed();
            Motion2::setMotorPositionFromTransformed();
        }
    } else if (axis != A_AXIS) {
        Motion1::simpleHome(axis);
    }
    dontChangeCoords = false;
}

void PrinterType::park(GCode* com) {
    if (!Motion1::isAxisHomed(X_AXIS)) {
        Motion1::homeAxes(X_AXIS);
    }
    Motion1::copyCurrentPrinter(Motion1::tmpPosition);
    float f = com->hasF() ? com->F : Motion1::moveFeedrate[X_AXIS];
    float x = com->hasX() ? com->X : 0;
    if (Motion1::dittoMode == 0) {
        if (activeAxis == 0) {
            Motion1::tmpPosition[X_AXIS] = endPos[0] + x;
            leftParked = Motion1::tmpPosition[X_AXIS] == endPos[0] && lazyMode;
        } else if (activeAxis == 1) {
            Motion1::tmpPosition[A_AXIS] = endPos[1] - x;
            rightParked = Motion1::tmpPosition[A_AXIS] == endPos[1] && lazyMode;
        }
    } else {
        Motion1::tmpPosition[X_AXIS] = endPos[0] + x;
        leftParked = Motion1::tmpPosition[X_AXIS] == endPos[0] && lazyMode;
        Motion1::tmpPosition[A_AXIS] = endPos[1] - x;
        rightParked = Motion1::tmpPosition[A_AXIS] == endPos[1] && lazyMode;
    }
    dontChangeCoords = true;
    Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    dontChangeCoords = false;
}

// Is called after normalizing position coordinates!
bool PrinterType::positionAllowed(float pos[NUM_AXES]) {
    /*    Com::printF(PSTR("Test PA XT:"), pos[X_AXIS], 2);
    Com::printF(PSTR(" YT:"), pos[Y_AXIS], 2);
    Com::printF(PSTR(" ZT:"), pos[Z_AXIS], 2);
#if NUM_AXES > A_AXIS
    Com::printF(PSTR(" AT:"), pos[A_AXIS], 2);
#endif
    Com::println();
*/
    if (Printer::isNoDestinationCheck() || dontChangeCoords) {
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

    // Test X or A range
    return pos[X_AXIS] >= endPos[0] && pos[X_AXIS] <= Motion1::maxPos[X_AXIS] && pos[A_AXIS] >= 0 && pos[A_AXIS] <= endPos[1] && pos[A_AXIS] >= pos[X_AXIS] + DUAL_X_MIN_DISTANCE;
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

fast8_t PrinterType::axisForTool(fast8_t toolId) {
    // TODO: Add map for tool->axis
    return toolId;
}

void PrinterType::deactivatedTool(fast8_t id) {
    fast8_t axisId = axisForTool(id);
    if (Motion1::dittoMode || (axisId == 0 && Motion1::currentPositionTransformed[X_AXIS] == endPos[0]) || (axisId == 1 && Motion1::currentPositionTransformed[A_AXIS] == endPos[1]) || (leftParked && rightParked)) {
        // already parked, nothing to do
        return;
    }
    // Move into park position
    COPY_ALL_AXES(Motion1::tmpPosition, Motion1::currentPositionTransformed);
    dontChangeCoords = true;
    if (axisId) { // right extruder
        Motion1::tmpPosition[A_AXIS] = endPos[1];
        rightParked = lazyMode;
    } else {
        Motion1::tmpPosition[X_AXIS] = endPos[0];
        leftParked = lazyMode;
    }
    Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    dontChangeCoords = false;
}

void PrinterType::activatedTool(fast8_t id) {
    activeAxis = axisForTool(id);
    if (Motion1::dittoMode || lazyMode /* || (activeAxis == 0 && Motion1::currentPositionTransformed[X_AXIS] == endPos[0]) || (activeAxis == 1 && Motion1::currentPositionTransformed[A_AXIS] == endPos[1]) */) {
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
            Motion1::tmpPosition[A_AXIS] = targetReal - Motion1::minPos[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
        }
        rightParked = false;
        leftParked = false;
        activeAxis = 0;
    } else if (activeAxis) {                        // right tool
        if (targetReal < Motion1::minPos[X_AXIS]) { // prevent crash
            targetReal = endPos[1];
        }
        Motion1::tmpPosition[A_AXIS] = targetReal;
        rightParked = false;
    } else {                                        // left tool
        if (targetReal > Motion1::maxPos[X_AXIS]) { // prevent crash
            targetReal = endPos[0];
        }
        Motion1::tmpPosition[X_AXIS] = targetReal;
        leftParked = false;
    }
    Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    dontChangeCoords = false;
}

void PrinterType::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Dual X"));
    EEPROM::handleFloat(eeprom + 9, PSTR("Bed X Min [mm]"), 2, bedRectangle[X_AXIS][0]);
    EEPROM::handleFloat(eeprom + 13, PSTR("Bed X Max [mm]"), 2, bedRectangle[X_AXIS][1]);
    EEPROM::handleFloat(eeprom + 17, PSTR("Bed Y Min [mm]"), 2, bedRectangle[Y_AXIS][0]);
    EEPROM::handleFloat(eeprom + 21, PSTR("Bed Y Max [mm]"), 2, bedRectangle[Y_AXIS][1]);
    EEPROM::handleFloat(eeprom + 0, PSTR("Offset Left [mm]"), 2, endPos[0]);
    EEPROM::handleFloat(eeprom + 4, PSTR("Offset Right [mm]"), 2, endPos[1]);
    EEPROM::handleByte(eeprom + 8, PSTR("Lazy Homing [0/1]"), lazyMode);
}

void PrinterType::restoreFromConfiguration() {
    lazyMode = LAZY_DUAL_X_AXIS;
    endPos[0] = DUAL_X_LEFT_OFFSET;
    endPos[1] = DUAL_X_RIGHT_OFFSET;
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
    if (Motion1::dittoMode && (axes & axisBits[E_AXIS]) != 0) {
        for (fast8_t i = 1; i <= Motion1::dittoMode; i++) {
            Tool::getTool(i)->enableMotor();
        }
    }
    Printer::unsetAllSteppersDisabled();
}

bool PrinterType::queueMove(float feedrate, bool secondaryMove) {
    if (dontChangeCoords) { // don't think about coordinates wanted!
        return Motion1::queueMove(feedrate, secondaryMove);
    }
    targetReal = Motion1::destinationPositionTransformed[X_AXIS];
    // Set current to real position if parked
    if (leftParked || (Motion1::dittoMode == 0 && activeAxis == 1)) {
        Motion1::currentPositionTransformed[X_AXIS] = endPos[0];
    }
    if (rightParked || (Motion1::dittoMode == 0 && activeAxis == 0)) {
        Motion1::currentPositionTransformed[A_AXIS] = endPos[1];
    }
    // Check what we need to do
    if (lazyMode && leftParked && rightParked) { // unpark used extruders if needed
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
                    Motion1::destinationPositionTransformed[A_AXIS] = targetReal - Motion1::minPos[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
                }
                Motion1::destinationPositionTransformed[X_AXIS] = targetReal;
                rightParked = false;
                leftParked = false;
            } else {
                if (activeAxis) {
                    Motion1::destinationPositionTransformed[A_AXIS] = targetReal;
                    rightParked = false;
                } else {
                    Motion1::destinationPositionTransformed[X_AXIS] = targetReal;
                    leftParked = false;
                }
            }
            Motion1::queueMove(Motion1::moveFeedrate[X_AXIS], secondaryMove);
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
                Motion1::destinationPositionTransformed[A_AXIS] = Motion1::destinationPositionTransformed[X_AXIS] - Motion1::minPos[X_AXIS] + (Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]) * 0.5f;
            }
        }
    } else if (activeAxis) {                                                                               // move tool 1
        if (rightParked) {                                                                                 // right tool active
            Motion1::destinationPositionTransformed[A_AXIS] = Motion1::currentPositionTransformed[A_AXIS]; // parked, do not move
        } else {
            Motion1::destinationPositionTransformed[A_AXIS] = targetReal; // move it to current x
        }
        Motion1::destinationPositionTransformed[X_AXIS] = endPos[0];                                   // do not move x0 axis
    } else if (!leftParked) {                                                                          // move tool 0                                                                        // left tool active
        Motion1::destinationPositionTransformed[A_AXIS] = Motion1::currentPositionTransformed[A_AXIS]; // Don't move right side
    } else {                                                                                           // left side is parked so do not move
        Motion1::destinationPositionTransformed[X_AXIS] = Motion1::currentPositionTransformed[X_AXIS]; // do not move x axis
        Motion1::destinationPositionTransformed[A_AXIS] = Motion1::currentPositionTransformed[A_AXIS];
    }
    // Motion1::currentPosition[A_AXIS] = Motion1::destinationPositionTransformed[A_AXIS] - Motion1::toolOffset[X_AXIS];
    // DEBUG_MSG("Q6");
    return Motion1::queueMove(feedrate, secondaryMove);
}

void PrinterType::setDittoMode(fast8_t count, bool mirror) {
    // Test if all tools have z offset 0
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (Tool::getTool(i)->getOffsetZ() != 0) {
            Com::printWarningFLN(PSTR("Ditto mode requires all tool z offsets to be zero."));
            GUI::setStatusP(PSTR("Tool Z-Offsets not 0"), GUIStatusLevel::ERROR);
            return;
        }
    }
    GUI::setStatusP(PSTR("Homing ..."), GUIStatusLevel::BUSY);
    Printer::setHoming(true);
    Motion1::dittoMode = count;
    Motion1::dittoMirror = mirror;
    homeAxis(X_AXIS);
    Printer::setHoming(false);
    GUI::popBusy();
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

bool PrinterType::canSelectTool(fast8_t toolId) {
    return toolId == 0 || Motion1::dittoMode == 0;
}

void PrinterType::M290(GCode* com) {
    InterruptProtectedBlock lock;
    if (com->hasZ()) {
        float z = constrain(com->Z, -2, 2);
        Motion2::openBabysteps[Z_AXIS] += z * Motion1::resolution[Z_AXIS];
    }
}

PGM_P PrinterType::getGeometryName() {
    return PSTR("Dual X");
}
#endif
