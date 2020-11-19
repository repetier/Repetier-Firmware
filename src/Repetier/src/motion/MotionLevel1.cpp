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

uint8_t axisBits[NUM_AXES];
uint8_t allAxes;

uint Motion1::eprStart;
Motion1Buffer Motion1::buffers[PRINTLINE_CACHE_SIZE]; // Buffer storage
float Motion1::currentPosition[NUM_AXES];             // Current printer position
float Motion1::currentPositionTransformed[NUM_AXES];
float Motion1::destinationPositionTransformed[NUM_AXES];
float Motion1::tmpPosition[NUM_AXES];
float Motion1::maxFeedrate[NUM_AXES];
float Motion1::homingFeedrate[NUM_AXES];
float Motion1::moveFeedrate[NUM_AXES];
float Motion1::maxAcceleration[NUM_AXES];
float Motion1::maxTravelAcceleration[NUM_AXES];
float Motion1::maxAccelerationEEPROM[NUM_AXES];
float Motion1::maxTravelAccelerationEEPROM[NUM_AXES];
float Motion1::resolution[NUM_AXES];
float Motion1::minPos[NUM_AXES];
float Motion1::maxPos[NUM_AXES];
float Motion1::minPosOff[NUM_AXES];
float Motion1::maxPosOff[NUM_AXES];
float Motion1::rotMax[3]; // Max positive offset from rotation at z max
float Motion1::rotMin[3]; // Min negative offset from rotation at z max
float Motion1::g92Offsets[NUM_AXES];
float Motion1::toolOffset[3];
float Motion1::maxYank[NUM_AXES];
float Motion1::homeRetestDistance[NUM_AXES];
float Motion1::homeEndstopDistance[NUM_AXES];
float Motion1::homeRetestReduction[NUM_AXES];
float Motion1::memory[MEMORY_POS_SIZE][NUM_AXES + 1];
float Motion1::parkPosition[3];
float Motion1::autolevelTransformation[9]; ///< Transformation matrix
float Motion1::advanceK = 0;               // advance spring constant
float Motion1::advanceEDRatio = 0;         // Ratio of extrusion
bool Motion1::wasLastSecondary = false;
int32_t Motion1::intLengthBuffered = 0;
int32_t Motion1::maxIntLengthBuffered = static_cast<int32_t>(1000.0 * MAX_BUFFERED_LENGTH_MM);
StepperDriverBase* Motion1::drivers[] = MOTORS;
fast8_t Motion1::memoryPos;
StepperDriverBase* Motion1::motors[NUM_AXES];
EndstopDriver* Motion1::minAxisEndstops[NUM_AXES];
EndstopDriver* Motion1::maxAxisEndstops[NUM_AXES];
fast8_t Motion1::homeDir[NUM_AXES];
int8_t Motion1::homePriority[NUM_AXES];
fast8_t Motion1::axesHomed;
EndstopMode Motion1::endstopMode;
int32_t Motion1::stepsRemaining[NUM_AXES]; // Steps remaining when testing endstops
ufast8_t Motion1::axesTriggered;
ufast8_t Motion1::motorTriggered;
ufast8_t Motion1::axesDirTriggered = 0;
ufast8_t Motion1::motorDirTriggered;
ufast8_t Motion1::stopMask;
fast8_t Motion1::dittoMode = 0;   // copy extrusion signals
fast8_t Motion1::dittoMirror = 0; // mirror for dual x printer
fast8_t Motion1::alwaysCheckEndstops;
bool Motion1::autolevelActive = false;
float Motion1::totalBabystepZ = 0.0f; // Sum since homing for z helper functions
#if FEATURE_AXISCOMP
float Motion1::axisCompTanXY, Motion1::axisCompTanXZ, Motion1::axisCompTanYZ;
#endif

#if FEATURE_RETRACTION
bool Motion1::retracted;
float Motion1::retractLength;
float Motion1::retractLongLength;
float Motion1::retractSpeed;
float Motion1::retractZLift;
float Motion1::retractUndoSpeed;
float Motion1::retractUndoExtraLength;
float Motion1::retractUndoExtraLongLength;
#endif

volatile fast8_t Motion1::last;    /// newest entry
volatile fast8_t Motion1::first;   /// first entry
volatile fast8_t Motion1::process; /// being processed
volatile fast8_t Motion1::length;  /// number of entries
volatile fast8_t Motion1::lengthUnprocessed;

constexpr int numMotors = std::extent<decltype(Motion1::drivers)>::value;
static_assert(numMotors == NUM_MOTORS, "NUM_MOTORS not defined correctly");

void Motion1::init() {

    int i;
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_MOTION, 1, EPR_M1_TOTAL);
    for (i = 0; i < PRINTLINE_CACHE_SIZE; i++) {
        buffers[i].id = i;
        buffers[i].flags = 0;
        buffers[i].state = Motion1State::FREE;
    }
    allAxes = 0;
    axesHomed = 0;
    axesTriggered = 0;
    motorTriggered = 0;
    FOR_ALL_AXES(i) {
        axisBits[i] = (uint8_t)1 << i;
        allAxes |= axisBits[i];
        currentPosition[i] = 0;
        currentPositionTransformed[i] = 0;
        destinationPositionTransformed[i] = 0;
        g92Offsets[i] = 0;
    }
    toolOffset[X_AXIS] = toolOffset[Y_AXIS] = toolOffset[Z_AXIS] = 0;
    last = first = length = 0;
    process = lengthUnprocessed = 0;
    endstopMode = EndstopMode::DISABLED;
    resetTransformationMatrix(true);
    setFromConfig(); // initial config
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
}

void Motion1::setFromConfig() {
    Motion2::velocityProfileIndex = VELOCITY_PROFILE;
    Motion2::velocityProfile = velocityProfiles[Motion2::velocityProfileIndex];

    alwaysCheckEndstops = ALWAYS_CHECK_ENDSTOPS;

    resolution[X_AXIS] = XAXIS_STEPS_PER_MM;
    resolution[Y_AXIS] = YAXIS_STEPS_PER_MM;
    resolution[Z_AXIS] = ZAXIS_STEPS_PER_MM;
    resolution[E_AXIS] = 100;

    maxYank[X_AXIS] = MAX_JERK;
    maxYank[Y_AXIS] = MAX_JERK;
    maxYank[Z_AXIS] = MAX_ZJERK;
    maxYank[E_AXIS] = 10;

    maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
    maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
    maxFeedrate[E_AXIS] = 10;

    homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X;
    homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y;
    homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z;
    homingFeedrate[E_AXIS] = 10;

    moveFeedrate[X_AXIS] = XY_SPEED;
    moveFeedrate[Y_AXIS] = XY_SPEED;
    moveFeedrate[Z_AXIS] = Z_SPEED;
    moveFeedrate[E_AXIS] = E_SPEED;

    maxAcceleration[X_AXIS] = maxAccelerationEEPROM[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    maxAcceleration[Y_AXIS] = maxAccelerationEEPROM[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    maxAcceleration[Z_AXIS] = maxAccelerationEEPROM[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    maxAcceleration[E_AXIS] = maxAccelerationEEPROM[E_AXIS] = 1000;

    maxTravelAcceleration[X_AXIS] = maxTravelAccelerationEEPROM[X_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    maxTravelAcceleration[Y_AXIS] = maxTravelAccelerationEEPROM[Y_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    maxTravelAcceleration[Z_AXIS] = maxTravelAccelerationEEPROM[Z_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    maxTravelAcceleration[E_AXIS] = maxTravelAccelerationEEPROM[E_AXIS] = 1000;

    homeRetestDistance[X_AXIS] = ENDSTOP_X_BACK_MOVE;
    homeRetestDistance[Y_AXIS] = ENDSTOP_Y_BACK_MOVE;
    homeRetestDistance[Z_AXIS] = ENDSTOP_Z_BACK_MOVE;
    homeRetestDistance[E_AXIS] = 0;

    homeRetestReduction[X_AXIS] = ENDSTOP_X_RETEST_REDUCTION_FACTOR;
    homeRetestReduction[Y_AXIS] = ENDSTOP_Y_RETEST_REDUCTION_FACTOR;
    homeRetestReduction[Z_AXIS] = ENDSTOP_Z_RETEST_REDUCTION_FACTOR;
    homeRetestReduction[E_AXIS] = 0;

    homeEndstopDistance[X_AXIS] = ENDSTOP_X_BACK_ON_HOME;
    homeEndstopDistance[Y_AXIS] = ENDSTOP_Y_BACK_ON_HOME;
    homeEndstopDistance[Z_AXIS] = ENDSTOP_Z_BACK_ON_HOME;
    homeEndstopDistance[E_AXIS] = 1000;

    minPos[X_AXIS] = X_MIN_POS;
    minPos[Y_AXIS] = Y_MIN_POS;
    minPos[Z_AXIS] = Z_MIN_POS;
    minPos[E_AXIS] = 0;

    maxPos[X_AXIS] = X_MIN_POS + X_MAX_LENGTH;
    maxPos[Y_AXIS] = Y_MIN_POS + Y_MAX_LENGTH;
    maxPos[Z_AXIS] = Z_MIN_POS + Z_MAX_LENGTH;
    maxPos[E_AXIS] = 0;

    homeDir[X_AXIS] = X_HOME_DIR;
    homeDir[Y_AXIS] = Y_HOME_DIR;
    homeDir[Z_AXIS] = Z_HOME_DIR;
    homeDir[E_AXIS] = 0;

    homePriority[X_AXIS] = X_HOME_PRIORITY;
    homePriority[Y_AXIS] = Y_HOME_PRIORITY;
    homePriority[Z_AXIS] = Z_HOME_PRIORITY;
    homePriority[E_AXIS] = -1;

    motors[X_AXIS] = &XMotor;
    motors[Y_AXIS] = &YMotor;
    motors[Z_AXIS] = &ZMotor;
    motors[E_AXIS] = nullptr;

    parkPosition[X_AXIS] = PARK_POSITION_X;
    parkPosition[Y_AXIS] = PARK_POSITION_Y;
    parkPosition[Z_AXIS] = PARK_POSITION_Z_RAISE;

    minAxisEndstops[X_AXIS] = &endstopXMin;
    minAxisEndstops[Y_AXIS] = &endstopYMin;
    minAxisEndstops[Z_AXIS] = &endstopZMin;
    minAxisEndstops[E_AXIS] = nullptr;

    maxAxisEndstops[X_AXIS] = &endstopXMax;
    maxAxisEndstops[Y_AXIS] = &endstopYMax;
    maxAxisEndstops[Z_AXIS] = &endstopZMax;
    maxAxisEndstops[E_AXIS] = nullptr;

#if NUM_AXES > A_AXIS
    resolution[A_AXIS] = AAXIS_STEPS_PER_MM;
    maxYank[A_AXIS] = MAX_AJERK;
    maxFeedrate[A_AXIS] = MAX_FEEDRATE_A;
    homingFeedrate[A_AXIS] = HOMING_FEEDRATE_A;
    moveFeedrate[A_AXIS] = A_SPEED;
    maxAcceleration[A_AXIS] = maxAccelerationEEPROM[A_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_A;
    maxTravelAcceleration[A_AXIS] = maxTravelAccelerationEEPROM[A_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_A;
    homeRetestDistance[A_AXIS] = ENDSTOP_A_BACK_MOVE;
    homeRetestReduction[A_AXIS] = ENDSTOP_A_RETEST_REDUCTION_FACTOR;
    homeEndstopDistance[A_AXIS] = ENDSTOP_A_BACK_ON_HOME;
    minPos[A_AXIS] = A_MIN_POS;
    maxPos[A_AXIS] = A_MIN_POS + A_MAX_LENGTH;
    homeDir[A_AXIS] = A_HOME_DIR;
    homePriority[A_AXIS] = A_HOME_PRIORITY;
    motors[A_AXIS] = &AMotor;
    minAxisEndstops[A_AXIS] = &endstopAMin;
    maxAxisEndstops[A_AXIS] = &endstopAMax;
#endif
#if NUM_AXES > B_AXIS
    resolution[B_AXIS] = BAXIS_STEPS_PER_MM;
    maxYank[B_AXIS] = MAX_BJERK;
    maxFeedrate[B_AXIS] = MAX_FEEDRATE_B;
    homingFeedrate[B_AXIS] = HOMING_FEEDRATE_B;
    moveFeedrate[A_AXIS] = B_SPEED;
    maxAcceleration[B_AXIS] = maxAccelerationEEPROM[B_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_B;
    maxTravelAcceleration[B_AXIS] = maxTravelAccelerationEEPROM[B_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_B;
    homeRetestDistance[B_AXIS] = ENDSTOP_B_BACK_MOVE;
    homeRetestReduction[B_AXIS] = ENDSTOP_B_RETEST_REDUCTION_FACTOR;
    homeEndstopDistance[B_AXIS] = ENDSTOP_B_BACK_ON_HOME;
    minPos[B_AXIS] = B_MIN_POS;
    maxPos[B_AXIS] = B_MIN_POS + B_MAX_LENGTH;
    homeDir[B_AXIS] = B_HOME_DIR;
    homePriority[B_AXIS] = B_HOME_PRIORITY;
    motors[B_AXIS] = &BMotor;
    minAxisEndstops[B_AXIS] = &endstopBMin;
    maxAxisEndstops[B_AXIS] = &endstopBMax;
#endif
#if NUM_AXES > C_AXIS
    resolution[C_AXIS] = CAXIS_STEPS_PER_MM;
    maxYank[C_AXIS] = MAX_CJERK;
    maxFeedrate[C_AXIS] = MAX_FEEDRATE_C;
    homingFeedrate[C_AXIS] = HOMING_FEEDRATE_C;
    moveFeedrate[C_AXIS] = C_SPEED;
    maxAcceleration[C_AXIS] = maxAccelerationEEPROM[C_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_C;
    maxTravelAcceleration[C_AXIS] = maxTravelAccelerationEEPROM[C_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_C;
    homeRetestDistance[C_AXIS] = ENDSTOP_C_BACK_MOVE;
    homeRetestReduction[C_AXIS] = ENDSTOP_C_RETEST_REDUCTION_FACTOR;
    homeEndstopDistance[C_AXIS] = ENDSTOP_C_BACK_ON_HOME;
    minPos[C_AXIS] = C_MIN_POS;
    maxPos[C_AXIS] = C_MIN_POS + C_MAX_LENGTH;
    homeDir[C_AXIS] = C_HOME_DIR;
    homePriority[C_AXIS] = C_HOME_PRIORITY;
    motors[C_AXIS] = &CMotor;
    minAxisEndstops[C_AXIS] = &endstopCMin;
    maxAxisEndstops[C_AXIS] = &endstopCMax;
#endif

#if FEATURE_AXISCOMP
    axisCompTanXY = AXISCOMP_TANXY;
    axisCompTanXZ = AXISCOMP_TANXZ;
    axisCompTanYZ = AXISCOMP_TANYZ;
#endif

#if FEATURE_RETRACTION
    Printer::setAutoretract(AUTORETRACT_ENABLED, true);
    retractLength = RETRACTION_LENGTH;
    retractLongLength = RETRACTION_LONG_LENGTH;
    retractSpeed = RETRACTION_SPEED;
    retractZLift = RETRACTION_Z_LIFT;
    retractUndoSpeed = RETRACTION_UNDO_SPEED;
    retractUndoExtraLength = RETRACTION_UNDO_EXTRA_LENGTH;
    retractUndoExtraLongLength = RETRACTION_UNDO_EXTRA_LONG_LENGTH;
#endif

    FOR_ALL_AXES(i) {
        if (motors[i]) {
            motors[i]->setAxis(i);
        }
        if (maxAxisEndstops[i] && maxAxisEndstops[i]->implemented() == false) {
            maxAxisEndstops[i] = nullptr;
        }
        if (minAxisEndstops[i] && minAxisEndstops[i]->implemented() == false) {
            minAxisEndstops[i] = nullptr;
        }
    }
    resetTransformationMatrix(true);
    autolevelActive = false;
}

void Motion1::setAutolevelActive(bool state, bool silent) {
    if (state != autolevelActive) {
        autolevelActive = state;
        updateRotMinMax();
        updatePositionsFromCurrentTransformed();
    }
    if (!silent) {
        if (autolevelActive) {
            Com::printInfoFLN(Com::tAutolevelEnabled);
        } else {
            Com::printInfoFLN(Com::tAutolevelDisabled);
        }
    }
    printCurrentPosition();
}

/// Compute safety margin required by rotation and sheer to not leave allowed region
void Motion1::updateRotMinMax() {
    bool old = autolevelActive;
    autolevelActive = true; // we always assume on or toggling would causes wrong positions.
    float posTransformed[3], pos[3];
    for (fast8_t i = 0; i <= Z_AXIS; i++) {
        float vMin, vMax;
        Tool::minMaxOffsetForAxis(i, vMin, vMax);
        minPosOff[i] = minPos[i] /* + vMin */ - vMax;
        maxPosOff[i] = maxPos[i] /* + vMax */ - vMin;
    }
    minPosOff[Z_AXIS] = 0; // we can't go below bed!
    rotMax[X_AXIS] = rotMax[Y_AXIS] = rotMax[Z_AXIS] = 0;
    rotMin[X_AXIS] = rotMin[Y_AXIS] = rotMin[Z_AXIS] = 0;
    for (fast8_t a = 0; a < 8; a++) {
        pos[X_AXIS] = (a & 1) ? minPosOff[X_AXIS] : maxPosOff[X_AXIS];
        pos[Y_AXIS] = (a & 2) ? minPosOff[Y_AXIS] : maxPosOff[Y_AXIS];
        pos[Z_AXIS] = (a & 4) ? minPosOff[Z_AXIS] : maxPosOff[Z_AXIS];
        transformToPrinter(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], posTransformed[X_AXIS], posTransformed[Y_AXIS], posTransformed[Z_AXIS]);
        for (fast8_t i = 0; i <= Z_AXIS; i++) {
            rotMax[i] = RMath::max(rotMax[i], posTransformed[i] - pos[i]);
            rotMin[i] = RMath::min(rotMin[i], posTransformed[i] - pos[i]);
        }
    }
    // add some safety margin preventing triggering end stops.
    for (fast8_t i = 0; i <= Z_AXIS; i++) {
        rotMax[i] += 0.01; // Add small epsilon to compensate rounding errors in transformations
        rotMin[i] -= 0.01;
        minPosOff[i] += rotMin[i];
        maxPosOff[i] += rotMax[i];
    }
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        minPosOff[i] = minPos[i];
        maxPosOff[i] = maxPos[i];
    }
    autolevelActive = old;
#ifdef DEBUG_MOVES
    Com::printArrayFLN(PSTR("minPosOff:"), minPosOff, 3);
    Com::printArrayFLN(PSTR("maxPosOff:"), maxPosOff, 3);
    Com::printArrayFLN(PSTR("rotMin:"), rotMin, 3);
    Com::printArrayFLN(PSTR("rotMax:"), rotMax, 3);
    Com::printArrayFLN(PSTR("transform:"), autolevelTransformation, 9, 5);
#endif
}

void Motion1::fillPosFromGCode(GCode& code, float pos[NUM_AXES], float fallback) {
    if (code.hasX()) {
        pos[X_AXIS] = Printer::convertToMM(code.X);
    } else {
        pos[X_AXIS] = fallback;
    }
    if (code.hasY()) {
        pos[Y_AXIS] = Printer::convertToMM(code.Y);
    } else {
        pos[Y_AXIS] = fallback;
    }
    if (code.hasZ()) {
        pos[Z_AXIS] = Printer::convertToMM(code.Z);
    } else {
        pos[Z_AXIS] = fallback;
    }
#if NUM_AXES > E_AXIS
    if (code.hasE()) {
        pos[E_AXIS] = Printer::convertToMM(code.E);
    } else {
        pos[E_AXIS] = fallback;
    }
#endif
#if NUM_AXES > A_AXIS
    if (code.hasA()) {
        pos[A_AXIS] = Printer::convertToMM(code.A);
    } else {
        pos[A_AXIS] = fallback;
    }
#endif
#if NUM_AXES > B_AXIS
    if (code.hasB()) {
        pos[B_AXIS] = Printer::convertToMM(code.B);
    } else {
        pos[B_AXIS] = fallback;
    }
#endif
#if NUM_AXES > C_AXIS
    if (code.hasC()) {
        pos[C_AXIS] = Printer::convertToMM(code.C);
    } else {
        pos[C_AXIS] = fallback;
    }
#endif
}

void Motion1::fillPosFromGCode(GCode& code, float pos[NUM_AXES], float fallback[NUM_AXES]) {
    if (code.hasX()) {
        pos[X_AXIS] = Printer::convertToMM(code.X);
    } else {
        pos[X_AXIS] = fallback[X_AXIS];
    }
    if (code.hasY()) {
        pos[Y_AXIS] = Printer::convertToMM(code.Y);
    } else {
        pos[Y_AXIS] = fallback[Y_AXIS];
    }
    if (code.hasZ()) {
        pos[Z_AXIS] = Printer::convertToMM(code.Z);
    } else {
        pos[Z_AXIS] = fallback[Z_AXIS];
    }
#if NUM_AXES > E_AXIS
    if (code.hasE()) {
        pos[E_AXIS] = Printer::convertToMM(code.E);
    } else {
        pos[E_AXIS] = fallback[E_AXIS];
    }
#endif
#if NUM_AXES > A_AXIS
    if (code.hasA()) {
        pos[A_AXIS] = Printer::convertToMM(code.A);
    } else {
        pos[A_AXIS] = fallback[A_AXIS];
    }
#endif
#if NUM_AXES > B_AXIS
    if (code.hasB()) {
        pos[B_AXIS] = Printer::convertToMM(code.B);
    } else {
        pos[B_AXIS] = fallback[B_AXIS];
    }
#endif
#if NUM_AXES > C_AXIS
    if (code.hasC()) {
        pos[C_AXIS] = Printer::convertToMM(code.C);
    } else {
        pos[C_AXIS] = fallback[C_AXIS];
    }
#endif
}

float Motion1::getShowPosition(fast8_t axis) {
    if (Tool::getActiveTool()->showMachineCoordinates()) {
        return currentPosition[axis];
    }
    return currentPosition[axis] + g92Offsets[axis];
}

void Motion1::setMotorForAxis(StepperDriverBase* motor, fast8_t axis) {
    waitForEndOfMoves();
    motors[axis] = motor;
    if (motor != nullptr) {
        motor->setAxis(axis);
    }
    Motion3::setDirectionsForNewMotors();
}

fast8_t Motion1::buffersUsed() {
    InterruptProtectedBlock noInts;
    return length;
}

int32_t Motion1::getBufferedLengthMM() {
    InterruptProtectedBlock noInts;
    return intLengthBuffered;
}

void Motion1::waitForEndOfMoves() {
    while (buffersUsed() > 0) {
        Commands::checkForPeriodicalActions(false);
        GCode::keepAlive(FirmwareState::Processing, 3);
    }
}

void Motion1::waitForXFreeMoves(fast8_t n, bool allowMoves) {
    while (buffersUsed() >= PRINTLINE_CACHE_SIZE - n) {
        GCode::keepAlive(FirmwareState::Processing, 3);
        Commands::checkForPeriodicalActions(allowMoves);
    }
    if (buffersUsed() < MIN_PRINTLINE_FILL) {
        return;
    }
    while (getBufferedLengthMM() > maxIntLengthBuffered) { // wait for reduced length to limit buffer
        GCode::keepAlive(FirmwareState::Processing, 3);
        Commands::checkForPeriodicalActions(allowMoves);
    }
}

Motion1Buffer& Motion1::reserve() {
    // DEBUG_MSG_FAST("RES");
    waitForXFreeMoves(1);
    InterruptProtectedBlock noInts;
    fast8_t idx = first;
    first++;
    if (first >= PRINTLINE_CACHE_SIZE) {
        first = 0;
    }
    length++;
    lengthUnprocessed++;
    return buffers[idx];
}

// Gets called by Motion2::pop only inside interrupt protected block
/* void Motion1::pop() {
    Motion1Buffer& b = buffers[last];
    b.state = Motion1State::FREE;
    b.flags = 0; // unblock
    last++;
    if (last >= PRINTLINE_CACHE_SIZE) {
        last = 0;
    }
    length--;
} */

void Motion1::setIgnoreABC(float coords[NUM_AXES]) {
    for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
        coords[i] = IGNORE_COORDINATE;
    }
}

void Motion1::printCurrentPosition() {
    float x, y, z;
    Printer::realPosition(x, y, z);
    x += Motion1::g92Offsets[X_AXIS];
    y += Motion1::g92Offsets[Y_AXIS];
    z += Motion1::g92Offsets[Z_AXIS];
    Com::printF(Com::tXColon, x * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceZColon, z * (Printer::unitIsInches ? 0.03937 : 1), 3);
    if (Motion1::motors[E_AXIS]) {
        Com::printF(Com::tSpaceEColon, Motion1::currentPosition[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 4);
    }
#if NUM_AXES > A_AXIS
    Com::printF(Com::tSpaceAColon, Motion1::currentPosition[A_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 2);
#endif
#if NUM_AXES > B_AXIS
    Com::printF(Com::tSpaceBColon, Motion1::currentPosition[B_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 2);
#endif
#if NUM_AXES > C_AXIS
    Com::printF(Com::tSpaceCColon, Motion1::currentPosition[C_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 2);
#endif
    Com::println();
#ifdef DEBUG_POS
    Com::printF(PSTR("OffX:"), Motion1::toolOffset[X_AXIS]); // to debug offset handling
    Com::printF(PSTR(" OffY:"), Motion1::toolOffset[Y_AXIS]);
    Com::printF(PSTR(" OffZ:"), Motion1::toolOffset[Z_AXIS]);
    Com::printF(PSTR(" XS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][X_AXIS]);
    Com::printF(PSTR(" YS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][Y_AXIS]);
    Com::printF(PSTR(" ZS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][Z_AXIS]);
#if NUM_AXES > A_AXIS
    Com::printF(PSTR(" ZA:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][A_AXIS]);
#endif
    Com::println();
#endif
}

void Motion1::copyCurrentOfficial(float coords[NUM_AXES]) {
    FOR_ALL_AXES(i) {
        coords[i] = currentPosition[i];
    }
}

void Motion1::copyCurrentPrinter(float coords[NUM_AXES]) {
    FOR_ALL_AXES(i) {
        coords[i] = currentPositionTransformed[i];
    }
}

void Motion1::setTmpPositionXYZ(float x, float y, float z) {
    setIgnoreABC(tmpPosition);
    tmpPosition[X_AXIS] = x;
    tmpPosition[Y_AXIS] = y;
    tmpPosition[Z_AXIS] = z;
    tmpPosition[E_AXIS] = IGNORE_COORDINATE;
#if NUM_AXES > A_AXIS
    tmpPosition[A_AXIS] = IGNORE_COORDINATE;
#endif
#if NUM_AXES > B_AXIS
    tmpPosition[B_AXIS] = IGNORE_COORDINATE;
#endif
#if NUM_AXES > C_AXIS
    tmpPosition[C_AXIS] = IGNORE_COORDINATE;
#endif
}

void Motion1::setTmpPositionXYZE(float x, float y, float z, float e) {
    setIgnoreABC(tmpPosition);
    tmpPosition[X_AXIS] = x;
    tmpPosition[Y_AXIS] = y;
    tmpPosition[Z_AXIS] = z;
    tmpPosition[E_AXIS] = e;
#if NUM_AXES > A_AXIS
    tmpPosition[A_AXIS] = IGNORE_COORDINATE;
#endif
#if NUM_AXES > B_AXIS
    tmpPosition[B_AXIS] = IGNORE_COORDINATE;
#endif
#if NUM_AXES > C_AXIS
    tmpPosition[C_AXIS] = IGNORE_COORDINATE;
#endif
}

// Move with coordinates in official coordinates (before offset, transform, ...)
bool Motion1::moveByOfficial(float coords[NUM_AXES], float feedrate, bool secondaryMove) {
    bool movingEAxis = (coords[E_AXIS] != currentPosition[E_AXIS]);
    Printer::unparkSafety();
    FOR_ALL_AXES(i) {
        if (coords[i] != IGNORE_COORDINATE) {
            currentPosition[i] = coords[i];
        }
    }
    PrinterType::officialToTransformed(currentPosition, destinationPositionTransformed);

    Tool* tool = Tool::getActiveTool();
    if (coords[E_AXIS] == IGNORE_COORDINATE || Printer::debugDryrun()
#if MIN_EXTRUDER_TEMP > MAX_ROOM_TEMPERATURE
        || (tool != nullptr && tool->getHeater() != nullptr && (tool->getHeater()->getCurrentTemperature() < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed()))
#endif
    ) { // ignore
#if MIN_EXTRUDER_TEMP > MAX_ROOM_TEMPERATURE
        if (movingEAxis && coords[E_AXIS] != IGNORE_COORDINATE && !Printer::debugDryrun()) {
            Com::printWarningFLN(Com::tColdExtrusionPrevented);
        }
#endif
        destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS];
    }
    if (feedrate == IGNORE_COORDINATE) {
        feedrate = Printer::feedrate;
    }
    return PrinterType::queueMove(feedrate, secondaryMove);
}

// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void Motion1::arc(float position[NUM_AXES], float target[NUM_AXES], float* offset, float radius, uint8_t isclockwise, float feedrate, bool secondaryMove) {
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[X_AXIS] + offset[X_AXIS];
    float center_axis1 = position[Y_AXIS] + offset[Y_AXIS];
    //float linear_travel = 0; //target[axis_linear] - position[axis_linear];
    float r_axis0 = -offset[0]; // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    /*long xtarget = Printer::destinationSteps[X_AXIS];
    long ytarget = Printer::destinationSteps[Y_AXIS];
    long ztarget = Printer::destinationSteps[Z_AXIS];
    long etarget = Printer::destinationSteps[E_AXIS];
    */
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001)) {
        angular_travel += 2.0f * M_PI;
    }
    if (isclockwise) {
        angular_travel -= 2.0f * M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel) * radius; //hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001f) {
        return; // treat as succes because there is nothing to do;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate > 60.0f ? floor(millimeters_of_travel / RMath::min(static_cast<float>(MM_PER_ARC_SEGMENT_BIG), Printer::feedrate * 0.01666f * static_cast<float>(MM_PER_ARC_SEGMENT))) : floor(millimeters_of_travel / static_cast<float>(MM_PER_ARC_SEGMENT)));
    if (segments == 0) {
        segments = 1;
    }
    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel / segments;
    float invSeg = 1.0 / static_cast<float>(segments);
    float increments[NUM_AXES];
    float arc_target[NUM_AXES];

    for (fast8_t i = Z_AXIS; i < NUM_AXES; i++) {
        increments[i] = (target[i] - position[i]) * invSeg;
        arc_target[i] = position[i];
    }

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. This requires only two cos() and sin() computations to form the rotation
       matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       all double numbers are single precision on the Arduino. (True double precision will not have
       round off issues for CNC applications.) Single precision error can accumulate to be greater than
       tool precision in some cases. Therefore, arc path correction is implemented.

       Small angle approximation may be used to reduce computation overhead further. This approximation
       holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    for (i = 1; i < segments; i++) {
        // Increment (segments-1)

        if ((count & 3) == 0) {
            Commands::checkForPeriodicalActions(false);
        }

        if (count < N_ARC_CORRECTION) { //25 pieces
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cos(i * theta_per_segment);
            sin_Ti = sin(i * theta_per_segment);
            r_axis0 = -offset[0] * cos_Ti + offset[1] * sin_Ti;
            r_axis1 = -offset[0] * sin_Ti - offset[1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[X_AXIS] = center_axis0 + r_axis0;
        arc_target[Y_AXIS] = center_axis1 + r_axis1;
        for (fast8_t i = Z_AXIS; i < NUM_AXES; i++) {
            arc_target[i] += increments[i];
        }

        moveByOfficial(arc_target, feedrate, secondaryMove);
    }
    // Ensure last segment arrives at target location.
    moveByOfficial(target, feedrate, secondaryMove);
}

void Motion1::setToolOffset(float ox, float oy, float oz) {
    if (Motion1::isAxisHomed(X_AXIS) && Motion1::isAxisHomed(Y_AXIS)) {
        setTmpPositionXYZ(currentPosition[X_AXIS] + ox - toolOffset[X_AXIS],
                          currentPosition[Y_AXIS] + oy - toolOffset[Y_AXIS],
                          currentPosition[Z_AXIS]);
        moveByOfficial(tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        waitForEndOfMoves();
    } else {
        currentPosition[X_AXIS] += ox - toolOffset[X_AXIS];
        currentPosition[Y_AXIS] += oy - toolOffset[Y_AXIS];
        updatePositionsFromCurrent();
    }
    toolOffset[X_AXIS] = ox;
    toolOffset[Y_AXIS] = oy;
    updatePositionsFromCurrentTransformed();
    if (Motion1::isAxisHomed(Z_AXIS)) {
        setTmpPositionXYZ(currentPosition[X_AXIS],
                          currentPosition[Y_AXIS],
                          currentPosition[Z_AXIS] + oz - toolOffset[Z_AXIS]);
        moveByOfficial(tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        waitForEndOfMoves();
    } else {
        currentPosition[Z_AXIS] += oz - toolOffset[Z_AXIS];
        updatePositionsFromCurrent();
    }
    toolOffset[Z_AXIS] = oz;
    updatePositionsFromCurrentTransformed();
}

// Move to the printer coordinates (after offset, transform, ...)
bool Motion1::moveByPrinter(float coords[NUM_AXES], float feedrate, bool secondaryMove) {
    bool movingEAxis = (coords[E_AXIS] != destinationPositionTransformed[E_AXIS]);
    Printer::unparkSafety();
    FOR_ALL_AXES(i) {
        if (coords[i] == IGNORE_COORDINATE) {
            destinationPositionTransformed[i] = currentPositionTransformed[i];
        } else {
            destinationPositionTransformed[i] = coords[i];
        }
    }
    Tool* tool = Tool::getActiveTool();
    if (coords[E_AXIS] == IGNORE_COORDINATE || Printer::debugDryrun()
#if MIN_EXTRUDER_TEMP > MAX_ROOM_TEMPERATURE
        || (tool != nullptr && tool->getHeater() != nullptr && (tool->getHeater()->getCurrentTemperature() < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed()))
#endif
    ) {
#if MIN_EXTRUDER_TEMP > MAX_ROOM_TEMPERATURE
        if (movingEAxis && coords[E_AXIS] != IGNORE_COORDINATE && !Printer::debugDryrun()) {
            Com::printWarningFLN(Com::tColdExtrusionPrevented);
        }
#endif
        currentPositionTransformed[E_AXIS] = destinationPositionTransformed[E_AXIS];
    }
    PrinterType::transformedToOfficial(destinationPositionTransformed, currentPosition);
    return PrinterType::queueMove(feedrate, secondaryMove);
}

void Motion1::updatePositionsFromCurrent() {
    PrinterType::officialToTransformed(currentPosition, currentPositionTransformed);
}

void Motion1::updatePositionsFromCurrentTransformed() {
    PrinterType::transformedToOfficial(currentPositionTransformed, currentPosition);
}

// Move with coordinates in official coordinates (before offset, transform, ...)
bool Motion1::moveRelativeByOfficial(float coords[NUM_AXES], float feedrate, bool secondaryMove) {
    Printer::unparkSafety();
    FOR_ALL_AXES(i) {
        if (coords[i] != IGNORE_COORDINATE) {
            coords[i] += currentPosition[i];
        }
    }
    return moveByOfficial(coords, feedrate, secondaryMove);
}

// Move to the printer coordinates (after offset, transform, ...)
bool Motion1::moveRelativeByPrinter(float coords[NUM_AXES], float feedrate, bool secondaryMove) {
    Printer::unparkSafety();
    FOR_ALL_AXES(i) {
        if (coords[i] != IGNORE_COORDINATE) {
            coords[i] += currentPositionTransformed[i];
        }
    }
    return moveByPrinter(coords, feedrate, secondaryMove);
}

/** This function is required to fix some errors left over after homing/z-probing.
 * It assumes that we can move all axes in parallel.
 */
void Motion1::moveRelativeBySteps(int32_t coords[NUM_AXES]) {
    Printer::unparkSafety();
    fast8_t axesUsed = 0;
    FOR_ALL_AXES(i) {
        if (coords[i]) {
            axesUsed |= axisBits[i];
        }
    }
    if (axesUsed == 0) {
        return;
    }
    waitForEndOfMoves();
    int32_t lpos[NUM_AXES];
    Motion2::copyMotorPos(lpos);
    Motion1Buffer& buf = reserve();
    buf.flags = 0;
    buf.action = Motion1Action::MOVE_STEPS;
    buf.feedrate = 0.2 * PrinterType::feedrateForMoveSteps(axesUsed);
    buf.acceleration = 0.2 * PrinterType::accelerationForMoveSteps(axesUsed);
    buf.startSpeed = buf.endSpeed = buf.length = 0;
    FOR_ALL_AXES(i) {
        buf.start[i] = lpos[i];
        buf.length += RMath::sqr(static_cast<float>(coords[i]) / resolution[i]);
    }
    buf.length = sqrtf(buf.length);
    buf.intLength = static_cast<int32_t>(1000.0f * buf.length);
    buf.invLength = 1.0f / buf.length;
    FOR_ALL_AXES(i) {
        buf.unitDir[i] = coords[i] * buf.invLength;
    }
    buf.sa2 = 2.0f * buf.length * buf.acceleration;
    buf.state = Motion1State::BACKWARD_PLANNED;
    InterruptProtectedBlock noInts;
    intLengthBuffered += buf.intLength;
}

// Adjust position to offset
void Motion1::correctBumpOffset() {
    int32_t correct[NUM_AXES];
    float pos[NUM_AXES];
    if (!Printer::isHomedAll()) {
        return; // nonsense stored, can lead to problems
    }
    waitForEndOfMoves();
    copyCurrentPrinter(pos);
    Leveling::addDistortion(pos);
    int32_t* lp = Motion2::lastMotorPos[Motion2::lastMotorIdx];
    PrinterType::transform(pos, correct);
    FOR_ALL_AXES(i) {
        correct[i] -= lp[i];
    }
    moveRelativeBySteps(correct);
    waitForEndOfMoves(); // prevent anyone from changing start position
}

bool Motion1::queueMove(float feedrate, bool secondaryMove) {
    if (!PrinterType::positionAllowed(destinationPositionTransformed, currentPosition[Z_AXIS])) {
        Com::printWarningFLN(PSTR("Move to illegal position prevented! Position should not be trusted any more!"));
        if (Printer::debugEcho()) {
            Com::printF(PSTR("XT:"), destinationPositionTransformed[X_AXIS], 2);
            Com::printF(PSTR(" YT:"), destinationPositionTransformed[Y_AXIS], 2);
            Com::printF(PSTR(" ZT:"), destinationPositionTransformed[Z_AXIS], 2);
#if NUM_AXES > A_AXIS
            Com::printF(PSTR(" AT:"), destinationPositionTransformed[A_AXIS], 2);
#endif
            Com::println();
        }
        FOR_ALL_AXES(i) { // mark as unhomed, might even prevent moves if unhomed moves are prevented!
            setAxisHomed(i, false);
        }
        return false;
    }
    float delta[NUM_AXES];
    float e2 = 0, length2 = 0;
    fast8_t axisUsed = 0;
    fast8_t dirUsed = 0;
    FOR_ALL_AXES(i) {
        if (motors[i] == nullptr) { // for E, A, B, C make motors removeable
            delta[i] = 0;
            continue;
        }
        delta[i] = destinationPositionTransformed[i] - currentPositionTransformed[i];
        if (fabsf(delta[i]) >= POSITION_EPSILON) {
            axisUsed += axisBits[i];
            if (delta[i] >= 0) {
                dirUsed += axisBits[i];
            }
            float tmp = delta[i] * delta[i];
            if (!PrinterType::ignoreAxisForLength(i)) {
                length2 += tmp;
            }
            if (i == E_AXIS) {
                e2 = tmp;
                Printer::filamentPrinted += delta[i]; // Keep track of printer amount
            }
        }
    }
    if (length2 == 0) { // no move, ignore it
        return true;
    }

    insertWaitIfNeeded(); // for buffer fillup on first move
    // Some tools need to add changes on switch
    if (secondaryMove != wasLastSecondary) {
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            tool->secondarySwitched(secondaryMove);
        }
        wasLastSecondary = secondaryMove;
    }
    Motion1Buffer& buf = reserve(); // Buffer is blocked because state is set to FREE!

    buf.length = sqrtf(length2);
    buf.intLength = static_cast<int32_t>(1000.0f * buf.length);
    buf.invLength = 1.0 / buf.length;
#ifdef DEBUG_MOVES
    if (Printer::debugEcho()) {
        Com::printF("Move CX:", currentPositionTransformed[X_AXIS]);
#if NUM_AXES > A_AXIS
        Com::printF(" AX:", currentPositionTransformed[A_AXIS]);
#endif
        Com::printF(" CY:", currentPositionTransformed[Y_AXIS]);
        Com::printF(" CZ:", currentPositionTransformed[Z_AXIS]);
        Com::printF(" OX:", toolOffset[X_AXIS]);
        Com::printF(" l:", buf.length);
        Com::printFLN(" f:", feedrate);
        Com::printF("Move DX:", delta[X_AXIS]);
#if NUM_AXES > A_AXIS
        Com::printF(" DA:", delta[A_AXIS]);
#endif
        Com::printF(" DE:", delta[E_AXIS]);
        Com::printF(" DY:", delta[Y_AXIS]);
        Com::printFLN(" DZ:", delta[Z_AXIS]);
        // Com::printArrayFLN(PSTR("cpt4:"), Motion1::currentPositionTransformed, 5, 2);
        // Com::printArrayFLN(PSTR("mp:"), Motion2::lastMotorPos[Motion2::lastMotorIdx], 5);
    }
#endif
    buf.action = Motion1Action::MOVE;
    if (endstopMode != EndstopMode::DISABLED) {
        buf.flags = FLAG_CHECK_ENDSTOPS;
    } else {
        buf.flags = alwaysCheckEndstops ? FLAG_CHECK_ENDSTOPS : 0;
    }
    if (secondaryMove) {
        buf.setActiveSecondary();
    }
    buf.axisUsed = axisUsed;
    buf.axisDir = dirUsed;

    if ((buf.axisUsed & 15) > 8) { // not pure e move
        // Need to scale feedrate so E component is not part of speed
        float exceptE = 1.0f / sqrt(length2 - e2);
        buf.feedrate = feedrate * buf.length * exceptE; // increase so xyz speed remains F neglecting E part
        if ((dirUsed & axisBits[E_AXIS]) != 0 && delta[E_AXIS] > 0) {
            if (advanceEDRatio > 0.000001) {
                buf.eAdv = advanceEDRatio * advanceK * resolution[E_AXIS] * 0.001;
            } else {
                buf.eAdv = delta[E_AXIS] / buf.length * advanceK * resolution[E_AXIS] * 0.001;
            }
            buf.setAdvance();
        } else {
            buf.eAdv = 0;
        }
    } else {
        buf.feedrate = feedrate;
        buf.eAdv = 0.0;
    }

    Tool* tool = Tool::getActiveTool();
    if (tool == nullptr) {
        buf.secondSpeed = 0;
        buf.secondSpeedPerMMPS = 0;
    } else {
        buf.secondSpeed = tool->activeSecondaryValue;
        buf.secondSpeedPerMMPS = tool->activeSecondaryPerMMPS;
    }

    // Check axis contraints like max. feedrate and acceleration
    float reduction = 1.0;
    FOR_ALL_AXES(i) {
        buf.start[i] = currentPositionTransformed[i];
        if (!buf.isAxisMoving(i)) { // no influence here
            buf.speed[i] = 0;
            buf.unitDir[i] = 0;
            continue;
        }
        buf.unitDir[i] = delta[i] * buf.invLength;
        buf.speed[i] = buf.unitDir[i] * buf.feedrate;
        if (fabsf(buf.speed[i]) > maxFeedrate[i]) {
            reduction = RMath::min(reduction, maxFeedrate[i] / fabsf(buf.speed[i]));
        }
    }
    if (reduction != 1.0) {
        buf.feedrate *= reduction;
        FOR_ALL_AXES(i) {
            buf.speed[i] *= reduction;
        }
    }
    // Next we compute highest possible acceleration
    buf.acceleration = 1000000;
    if ((axisUsed & 8) == 0 && !secondaryMove) { // travel move uses different acceleration
        FOR_ALL_AXES(i) {
            if (!buf.isAxisMoving(i)) { // no influence here
                continue;
            }
            float a = fabs(maxTravelAcceleration[i] / buf.unitDir[i]);
            if (a < buf.acceleration) {
                buf.acceleration = a;
            }
        }
    } else {
        FOR_ALL_AXES(i) {
            if (!buf.isAxisMoving(i)) { // no influence here
                continue;
            }
            float a = fabs(maxAcceleration[i] / buf.unitDir[i]);
            if (a < buf.acceleration) {
                buf.acceleration = a;
            }
        }
    }
    buf.sa2 = 2.0f * buf.length * buf.acceleration;
    if (buf.isAdvance()) {
        buf.startSpeed = buf.endSpeed = 0;
        buf.maxJoinSpeed = buf.calculateSaveStartEndSpeed();
    } else {
        buf.startSpeed = buf.endSpeed = buf.maxJoinSpeed = buf.calculateSaveStartEndSpeed();
    }
    // Destination becomes new current
    float timeForMove = length / feedrate;
    COPY_ALL_AXES(currentPositionTransformed, destinationPositionTransformed);
    buf.state = Motion1State::RESERVED; // make it accessible
    {
        InterruptProtectedBlock noInts;
        intLengthBuffered += buf.intLength;
    }
    backplan(buf.id);
    return true;
}

void Motion1::insertWaitIfNeeded() {
    if (buffersUsed()) { // already printing, do not wait
        return;
    }
    for (fast8_t i = 0; i <= NUM_MOTION2_BUFFER; i++) {
        Motion1Buffer& buf = reserve();
        buf.action = Motion1Action::WAIT;
        if (i == 0) {
            buf.feedrate = ceilf((300.0 * 1000000.0) * invStepperFrequency);
        } else {
            buf.feedrate = ceilf((20.0 * 1000000.0) * invStepperFrequency);
        }
        buf.flags = 0;
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            buf.secondSpeed = tool->computeIntensity(0, false, tool->activeSecondaryValue, tool->activeSecondaryPerMMPS);
        } else {
            buf.secondSpeed = 0;
        }
        buf.state = Motion1State::FORWARD_PLANNED;
    }
}

void Motion1::WarmUp(uint32_t wait, int secondary) {
    Motion1Buffer& buf = reserve();
    buf.action = Motion1Action::WARMUP;
    buf.feedrate = ceilf((wait * 1000000.0) * invStepperFrequency);
    buf.flags = 0;
    buf.state = Motion1State::FORWARD_PLANNED;
    buf.secondSpeed = secondary;
}

void Motion1::backplan(fast8_t actId) {
    Motion1Buffer* next = nullptr;
    Motion1Buffer* act = nullptr;
    float lastJunctionSpeed;
    fast8_t maxLoops;
    {
        InterruptProtectedBlock noInts;
        maxLoops = lengthUnprocessed;
    }
    //Com::printFLN("BL:", (int)maxLoops);
    //Com::printFLN(" L:", (int)actId);
    do {
        if (next != nullptr) { // free processed block
            next->unblock();
        }
        next = act; // skip back to previous move
        act = &buffers[actId];
        if (!act->block()) {
            if (next != nullptr) {
                next->unblock();
            }
            //Com::printFLN("BF:");
            return;
        }
        if (actId > 0) {
            actId--;
        } else {
            actId = PRINTLINE_CACHE_SIZE - 1;
        }
        if (act->state >= Motion1State::BACKWARD_FINISHED || act->action != Motion1Action::MOVE /* || act->state == Motion1State::FREE*/) {
            // Everything before this is planned, so nothing we can do
            if (next) {
                next->unblock();
            }
            act->unblock();
            //Com::printFLN("BF2:");
            return;
        }
        if (next == nullptr) { // first element, just set start speed
            lastJunctionSpeed = act->endSpeed;
            maxLoops--;
            continue; // need 2 lines to plan
        }
        if (act->state == Motion1State::RESERVED) {
            act->calculateMaxJoinSpeed(*next); // changes act->state
            if (act->state >= Motion1State::BACKWARD_FINISHED) {
                // join speed is slower then start speed, stop planning
                break;
            }
        }
        // We have a previous move, so check highest end speed
        // Limits are:
        // next->maxJoinSpeed
        // First we compute what speed we can reach at allowMoves
        lastJunctionSpeed = sqrtf(lastJunctionSpeed * lastJunctionSpeed + next->sa2);
        //Com::printF("BP:", lastJunctionSpeed, 1);
        //Com::printFLN(" ", (int)next->state);
        if (lastJunctionSpeed > act->maxJoinSpeed) {
            // We can reach target speed, mark finished
            next->startSpeed = act->endSpeed = act->maxJoinSpeed;
            act->state = Motion1State::BACKWARD_FINISHED;
            next->unblock();
            act->unblock();
            return;
        }
        next->startSpeed = act->endSpeed = lastJunctionSpeed;
        act->state = Motion1State::BACKWARD_PLANNED;
        maxLoops--;
    } while (maxLoops);
    if (next) {
        next->unblock();
    }
    act->unblock();
}

// Called from motion2 timer interrupt, so only needs to protect for motion3 timer changes!
Motion1Buffer* Motion1::forward(Motion2Buffer* m2) {
    if (lengthUnprocessed == 0) {
        return nullptr;
    }
    Motion1Buffer* f = &buffers[process];
    // DEBUG_MSG2_FAST("fp0:", (int)f->state)
    if (f->block()) {
        // DEBUG_MSG2_FAST("fp1:", (int)f->action)
        if (f->action == Motion1Action::MOVE) {
            // DEBUG_MSG_FAST("fp1m")
            // First fix end speed to realistic value
            Motion1Buffer* f2 = nullptr;
            if (lengthUnprocessed > 1) {
                f2 = (process == PRINTLINE_CACHE_SIZE - 1 ? &buffers[0] : &buffers[process + 1]);
                if (!f2->block()) {
                    f->unblock();
                    // DEBUG_MSG_FAST("f2b");
                    return nullptr;
                }
                if (f2->action != Motion1Action::MOVE) {
                    f2->unblock();
                    f2 = nullptr;
                }
            }
            float maxEndSpeed = sqrt(f->startSpeed * f->startSpeed + f->sa2); // reachable end speed
            if (f->endSpeed > maxEndSpeed) {                                  // allowed to accelerate so far, copy value
                f->endSpeed = maxEndSpeed;
                if (f2 != nullptr) {
                    f2->startSpeed = f->endSpeed;
                    f2->state = Motion1State::BACKWARD_FINISHED;
                }
            }
            if (f2 != nullptr) {
                f2->unblock();
            }
            float invAcceleration = 1.0f / f->acceleration;
            // Where do we hit if we accelerate from both speed
            // and search for equal speed.
            // t1 is time to accelerate to feedrate
            if (fabsf(f->startSpeed - f->feedrate) < SPEED_EPSILON) {
                m2->t1 = 0.0f;
                m2->s1 = 0.0f;
            } else {
                m2->t1 = (f->feedrate - f->startSpeed) * invAcceleration;
                m2->s1 = m2->t1 * (f->startSpeed + 0.5 * f->acceleration * m2->t1);
            }
            // t3 is time to deccelerate to feedrate
            if (fabsf(f->endSpeed - f->feedrate) < SPEED_EPSILON) {
                m2->t3 = 0.0f;
                m2->s3 = 0.0f;
            } else {
                m2->t3 = (f->feedrate - f->endSpeed) * invAcceleration;
                m2->s3 = m2->t3 * (f->endSpeed + 0.5 * f->acceleration * m2->t3);
            }
            if (m2->s1 + m2->s3 > f->length) {
                // not enough space for constant speed!
                // We now have the choice to accelerate in between or to keep moving
                // with the higest limit speed. While accelerating is the fastest solution
                // the constant speed gives better extrusion results. As a compromise we
                // accelerate only if it is beneficial, e.g. there is considerate speed difference
                float sq = f->startSpeed * f->startSpeed + f->endSpeed * f->endSpeed;
                float vmax = RMath::max(f->startSpeed, f->endSpeed);
                float fmin = (2.0f * vmax * vmax - sq) / f->sa2;
                float fmid = (2.0f + fmin) * 0.3333333333333f;
                float sTerm = sqrtf(sq + f->sa2 * fmid) * 1.414213562f;
                if ((f->axisUsed & 3) != 0             // includes xy move
                    && f->startSpeed >= 9.99           // we already have a minimum speed
                    && 2.0f * f->startSpeed < fmid     // we can not double speed with acceleration
                ) {                                    // reduce acceleration to minimum for smooth curves
                    if (f->startSpeed > f->endSpeed) { // decelerate at end
                        m2->t1 = 0.0f;
                        m2->t3 = (f->startSpeed - f->endSpeed) * invAcceleration;
                        f->feedrate = f->startSpeed;
                        m2->s1 = 0.0f;
                        m2->s3 = m2->t3 * (f->endSpeed + 0.5f * f->acceleration * m2->t3);
                        if (m2->s3 > f->length) {            // rounding error can cause reversals, fix it
                            m2->t3 *= m2->s3 * f->invLength; // more time
                            m2->s3 = f->length;
                        }
                    } else { // accelerate only at start
                        m2->t1 = (f->endSpeed - f->startSpeed) * invAcceleration;
                        m2->t3 = 0.0f;
                        f->feedrate = f->endSpeed;
                        m2->s3 = 0.0f;
                        m2->s1 = (0.5f * f->acceleration * m2->t1 + f->startSpeed) * m2->t1;
                        if (m2->s1 > f->length) {            // rounding error can cause reversals, fix it
                            m2->t1 *= m2->s1 * f->invLength; // increase time to relax for reduced distance
                            m2->s1 = f->length;
                        }
                    }
                    m2->s2 = f->length - m2->s1 - m2->s3;
                    m2->t2 = m2->s2 / f->feedrate;
                } else { // we are slow, try to optimize
                    float div = 0.5f * invAcceleration;
                    m2->t1 = RMath::max(0.0f, (sTerm - 2.0f * f->startSpeed) * div);
                    m2->t3 = RMath::max(0.0f, (sTerm - 2.0f * f->endSpeed) * div);
                    f->feedrate = f->startSpeed + m2->t1 * f->acceleration;
                    m2->s1 = (0.5f * f->acceleration * m2->t1 + f->startSpeed) * m2->t1;
                    m2->s3 = (f->endSpeed + 0.5f * f->acceleration * m2->t3) * m2->t3;
                    m2->s2 = f->length - m2->s1 - m2->s3; // * (1.0 - fmid);
                    if (m2->s2 < 0) {                     // no negative distances!
                        if (m2->s1 > m2->s3) {
                            m2->t1 -= m2->s2 / fmid;
                            m2->s1 = f->length - m2->s3;
                        } else {
                            m2->t3 -= m2->s2 / fmid;
                            m2->s3 = f->length - m2->s1;
                        }
                        m2->s2 = 0.0f;
                        m2->t2 = 0.0f;
                    } else {
                        m2->t2 = m2->s2 / f->feedrate;
                    }
                }
                // Acceleration stops at feedrate so make sure it is set to right limit
                /* Com::printF(" f:", f->feedrate, 0);
                Com::printF(" t1:", m2->t1, 4);
                Com::printF(" t2:", m2->t2, 4);
                Com::printF(" t3:", m2->t3, 4);
                Com::printF(" fm:", fmid, 4);*/
            } else {
                m2->s2 = f->length - m2->s1 - m2->s3;
                m2->t2 = m2->s2 / f->feedrate;
                // Com::printFLN(" s2:", m2->s2,2);
            }
            /* Com::printF(" t1:", m2->t1, 4);
            Com::printF(" t2:", m2->t2, 4);
            Com::printF(" t3:", m2->t3, 4);
            Com::printF(" f:", f->feedrate, 2);
            Com::printF(" ss:", f->startSpeed, 1);
            Com::printF(" es:", f->endSpeed, 1);
            Com::printFLN(" l:", f->length, 4); */
        } else if (f->action == Motion1Action::MOVE_STEPS) {
            float invAcceleration = 1.0 / f->acceleration;
            // Where do we hit if we accelerate from both speed
            // and search for equal speed.
            // t1 is time to accelerate to feedrate
            m2->t1 = (f->feedrate - f->startSpeed) * invAcceleration;
            m2->s1 = m2->t1 * (f->startSpeed + 0.5 * f->acceleration * m2->t1);
            // t3 is time to deccelerate to feedrate
            m2->t3 = (f->feedrate - f->endSpeed) * invAcceleration;
            m2->s3 = m2->t3 * (f->endSpeed + 0.5 * f->acceleration * m2->t3);
            if (m2->s1 + m2->s3 > f->length) {
                float div = 0.5 * invAcceleration;
                float sterm = sqrtf(f->startSpeed * f->startSpeed + f->endSpeed * f->endSpeed + f->sa2) * 1.414213562;
                m2->t1 = RMath::max(0.0f, (sterm - 2.0f * f->startSpeed) * div);
                m2->t3 = RMath::max(0.0f, (sterm - 2.0f * f->endSpeed) * div);
                m2->t2 = 0;
                m2->s2 = 0;
                m2->s1 = (0.5 * f->acceleration * m2->t1 + f->startSpeed) * m2->t1;
                // Acceleration stops at feedrate so make sure it is set to right limit
                f->feedrate = f->startSpeed + m2->t1 * f->acceleration;
            } else {
                m2->s2 = f->length - m2->s1 - m2->s3;
                m2->t2 = m2->s2 / f->feedrate;
                // Com::printFLN(" s2:", m2->s2,2);
            }
            /* Com::printF("ss:", f->startSpeed, 0);
            Com::printF(" es:", f->endSpeed, 0);
            Com::printF(" f:", f->feedrate, 0);
            Com::printF(" t1:", m2->t1, 4);
            Com::printF(" t2:", m2->t2, 4);
            Com::printF(" t3:", m2->t3, 4);
            Com::printF(" s1:", m2->s1, 4);
            Com::printF(" s2:", m2->s2, 4);
            Com::printF(" l:", f->length, 4);
            Com::printFLN(" a:", f->acceleration, 4); */
        }
        f->state = Motion1State::FORWARD_PLANNED;
        {
            InterruptProtectedBlock noInts;
            process++;
            if (process >= PRINTLINE_CACHE_SIZE) {
                process = 0;
            }
            lengthUnprocessed--;
        }
        f->unblock(); // now protected by state!
        // Com::printFLN(" ff:", (int)f->id);
        return f;
    } else if (f->action == Motion1Action::MOVE_BABYSTEPS) {
        return f;
    } else {
        // Com::printFLN(PSTR("fw blocked"));
        return nullptr; // try next time
    }
}

float Motion1Buffer::calculateSaveStartEndSpeed() {
    float safe = feedrate;
    FOR_ALL_AXES(i) {
        if (axisUsed & axisBits[i]) {
            safe = RMath::min(safe, fabs(0.5 * Motion1::maxYank[i] / unitDir[i]));
        }
    }
    return safe;
}

/*
 Called for the motion executed first with the follow up
 segment as parameter.

 Since we now have higher order motion profiles that also
 have a real jerk (physically) we name the shock at velocity
 change now yank to prevent name conflicts.
*/
void Motion1Buffer::calculateMaxJoinSpeed(Motion1Buffer& next) {
    if (isAdvance() != next.isAdvance()) {
        // ensure starting with 0 velocity for advance
        maxJoinSpeed = 0; // endSpeed;
        state = Motion1State::BACKWARD_FINISHED;
        return;
    }
    state = Motion1State::JUNCTION_COMPUTED;
    float vStart, vEnd, factor = 1.0, corrFactor, yank;
    bool limited = false;
    bool isSpeedReduced = false;
    float yankFactor = 1.0f;
    if (length < SMALL_SEGMENT_SIZE) {
        isSpeedReduced = true;
        yankFactor = (SMALL_SEGMENT_SIZE * SMALL_SEGMENT_SIZE) / length * length;
    }
    // Smallest of joining target feed rates is maximum limit
    if (next.feedrate < feedrate) {
        corrFactor = next.feedrate / feedrate;
        maxJoinSpeed = next.feedrate;
        FOR_ALL_AXES(i) {
            if ((axisUsed & axisBits[i]) || (next.axisUsed & axisBits[i])) {
                if (limited) {
                    vEnd = next.speed[i] * factor;
                    vStart = speed[i] * corrFactor * factor;
                } else {
                    vEnd = next.speed[i];
                    vStart = speed[i] * corrFactor;
                }
                yank = fabs(vStart - vEnd) * yankFactor;
                if (yank > Motion1::maxYank[i]) {
                    factor *= Motion1::maxYank[i] / yank;
                    limited = true;
                }
            }
        }
    } else {
        corrFactor = feedrate / next.feedrate;
        maxJoinSpeed = feedrate;
        FOR_ALL_AXES(i) {
            if ((axisUsed & axisBits[i]) || (next.axisUsed & axisBits[i])) {
                if (limited) {
                    vEnd = next.speed[i] * corrFactor * factor;
                    vStart = speed[i] * factor;
                } else {
                    vEnd = next.speed[i] * corrFactor;
                    vStart = speed[i];
                }
                yank = fabs(vStart - vEnd) * yankFactor;
                if (yank > Motion1::maxYank[i]) {
                    factor *= Motion1::maxYank[i] / yank;
                    limited = true;
                }
            }
        }
    }

    // Com::printF("j:", maxJoinSpeed, 1);
    // Check per axis for violation of yank contraint

    if (limited) {
        maxJoinSpeed *= factor;
    }
    // check if safe speeds are higher, e.g. because we switch x with z move
    if (!isSpeedReduced && (next.startSpeed > maxJoinSpeed || endSpeed > maxJoinSpeed)) {
        maxJoinSpeed = endSpeed;
        state = Motion1State::BACKWARD_FINISHED;
    }
    // Com::printFLN("mj:", maxJoinSpeed, 1);
    //Com::printFLN(" v0:", speed[0], 1);
    //Com::printFLN(" v1:", next.speed[0], 1);
}

static int errcount = 0;
bool Motion1Buffer::block() {
    InterruptProtectedBlock noInts;
    if (flags & FLAG_BLOCKED) {
        /*if (errcount < 10) {
            errcount++;
            Com::printFLN("bf1 ", (int)id);
            //Com::printF(" id ", (int)id);
            //Com::printFLN(" f ", (int)flags);
        }*/
        return false;
    }
    if (state == Motion1State::FREE) {
        /* if (errcount < 10) {
            errcount++;
            Com::printF("bf2 ", (int)action);
            Com::printF(" id ", (int)id);
            Com::printFLN(" f ", (int)flags);
        }*/
        return false;
    }
    flags |= FLAG_BLOCKED;
    /*if (id == 6) {
        Com::printFLN("bf0 ", (int)id);
        // Com::printF(" id ", (int)id);
        // Com::printFLN(" f ", (int)flags);
    }*/
    return true;
}

void Motion1Buffer::unblock() {
    flags &= ~FLAG_BLOCKED;
    // Com::printF("bfu ", (int)id);
    // Com::printF(" id ", (int)id);
    // Com::printFLN(" f ", (int)flags);
}

void Motion1::moveToParkPosition() {
#if PARK_POSITION_Z_UP_FIRST != 0
    if (Motion1::parkPosition[Z_AXIS] > 0 && isAxisHomed(Z_AXIS)) {
        Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::min(maxPos[Z_AXIS], parkPosition[Z_AXIS] + currentPosition[Z_AXIS]));
        moveByOfficial(tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
    }
#endif
    if (isAxisHomed(X_AXIS) && isAxisHomed(Y_AXIS)) {
        setTmpPositionXYZ(parkPosition[X_AXIS], parkPosition[Y_AXIS], IGNORE_COORDINATE);
        moveByOfficial(tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    }
#if PARK_POSITION_Z_UP_FIRST == 0
    if (Motion1::parkPosition[Z_AXIS] > 0 && isAxisHomed(Z_AXIS)) {
        Motion1::moveByPrinter(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::min(maxPos[Z_AXIS], parkPosition[Z_AXIS] + currentPosition[Z_AXIS]));
        moveByOfficial(tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
    }
#endif
}

/// Pushes current position to memory stack. Return true on success.
bool Motion1::pushToMemory() {
    if (memoryPos == MEMORY_POS_SIZE) {
        return false;
    }
    FOR_ALL_AXES(i) {
        memory[memoryPos][i] = currentPosition[i];
    }
    memory[memoryPos][NUM_AXES] = Printer::feedrate;
    memoryPos++;
    return true;
}
/// Pop memorized position to tmpPosition
bool Motion1::popFromMemory() {
    if (memoryPos == 0) {
        return false;
    }
    memoryPos--;
    FOR_ALL_AXES(i) {
        tmpPosition[i] = memory[memoryPos][i];
    }
    Printer::feedrate = memory[memoryPos][NUM_AXES];
    return true;
}

bool Motion1::popFromMemory(float coords[NUM_AXES]) {
    if (memoryPos == 0) {
        return false;
    }
    memoryPos--;
    FOR_ALL_AXES(i) {
        coords[i] = memory[memoryPos][i];
    }
    Printer::feedrate = memory[memoryPos][NUM_AXES];
    return true;
}

bool Motion1::isAxisHomed(fast8_t axis) {
    return (axesHomed & axisBits[axis]) != 0;
}

void Motion1::setAxisHomed(fast8_t axis, bool state) {
    if (state) {
        axesHomed |= axisBits[axis];
    } else {
        axesHomed &= ~axisBits[axis];
    }
}

void Motion1::homeAxes(fast8_t axes) {
    GUI::setStatusP(PSTR("Homing ..."), GUIStatusLevel::BUSY);
    if (axes == 0) {
        axes = 127; // default is home all axes
    }
    waitForEndOfMoves();
    float oldCoordinates[NUM_AXES];
    copyCurrentOfficial(oldCoordinates); // store to redo position when finished
    Printer::setHoming(true);
#if ZHOME_PRE_RAISE
    // float zAmountRaised = 0;
    if (ZHOME_PRE_RAISE == 2
        || (Motion1::minAxisEndstops[Z_AXIS] && Motion1::minAxisEndstops[Z_AXIS]->update())) {
        if (!isAxisHomed(Z_AXIS) || currentPosition[Z_AXIS] + ZHOME_PRE_RAISE_DISTANCE < maxPos[Z_AXIS]) {
            setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_PRE_RAISE_DISTANCE);
            moveRelativeByOfficial(tmpPosition, homingFeedrate[Z_AXIS], false);
            waitForEndOfMoves();
            // zAmountRaised = ZHOME_PRE_RAISE_DISTANCE;
        }
    }
#endif
    // We measure in printer coordinates, so deactivate all corrections
    bool isAL = isAutolevelActive();
    setAutolevelActive(false, true);
    bool bcActive = Leveling::isDistortionEnabled();
    Leveling::setDistortionEnabled(false);
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    fast8_t activeToolId = Tool::getActiveToolId();
    callBeforeHomingOnSteppers();
#if FIXED_Z_HOME_POSITION
    if (axes & axisBits[Z_AXIS]) { // ensure x and y are homed
        if (!isAxisHomed(X_AXIS)) {
            axes |= axisBits[X_AXIS];
        }
        if (!isAxisHomed(Y_AXIS)) {
            axes |= axisBits[Y_AXIS];
        }
    }
#endif
    for (int priority = 0; priority <= 10; priority++) {
        FOR_ALL_AXES(i) {
            if ((axisBits[i] & axes) == 0 && axes != 0) {
                continue;
            }
            if (homePriority[i] == priority) {
                g92Offsets[i] = 0;
                if (i <= Z_AXIS) {
                    toolOffset[i] = 0;
                }
                if (i == Z_AXIS) { // Special case if we want z homing at certain position
#if FIXED_Z_HOME_POSITION
#if X_HOME_PRIORITY > Z_HOME_PRIORITY || Y_HOME_PRIORITY > Z_HOME_PRIORITY
#error X_HOME_PRIORITY and Y_HOME_PRIORITY must be smaller then Z_HOME_PRIORITY for option FIXED_Z_HOME_POSITION!
#endif
                    setTmpPositionXYZ(ZHOME_X_POS, ZHOME_Y_POS, IGNORE_COORDINATE);
                    moveByOfficial(tmpPosition, maxFeedrate[X_AXIS], false);
#endif
                    if (ZProbe != nullptr && homeDir[Z_AXIS] < 0) { // activate z probe
                        if (!ZProbeHandler::activate()) {
                            return;
                        }
                    }
                }
                PrinterType::homeAxis(i);
                oldCoordinates[i] = currentPosition[i]; // replace start coords with homed axes coords
                g92Offsets[i] = 0;
                if (i == Z_AXIS && ZProbe != nullptr && homeDir[Z_AXIS] < 0) {
                    ZProbeHandler::deactivate(); // activates tool offsets
                }
#if ZHOME_HEIGHT > 0
                if (i == Z_AXIS) {
                    setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEIGHT, IGNORE_COORDINATE);
                    moveByOfficial(tmpPosition, maxFeedrate[Z_AXIS], false);
                }
#endif
            }
        }
    }
    // all selected axes are now homed

    callAfterHomingOnSteppers(); // for motor drivers to turn off crash detection if wanted
    Printer::setHoming(false);

    // Test if all axes are homed
    bool ok = true;
    FOR_ALL_AXES(i) {
        if (homePriority[i] >= 0 && isAxisHomed(i) == false) {
            ok = false;
        }
    }
    Printer::setHomedAll(ok);
    // Reactivate corrections
    setAutolevelActive(isAL, true);
    Leveling::setDistortionEnabled(bcActive);

    if (axes & axisBits[Z_AXIS]) {
        totalBabystepZ = 0;
        Motion1::correctBumpOffset(); // activate bump offset, needs distorion enabled to have an effect!
        // Add z probe correctons
        int32_t motorPos[NUM_AXES];
        float oldPos[NUM_AXES];
        copyCurrentPrinter(oldPos);
        waitForEndOfMoves();
        int32_t* lp = Motion2::lastMotorPos[Motion2::lastMotorIdx];
        FOR_ALL_AXES(i) {
            motorPos[i] = lp[i];
        }
        float zpCorr = 0;
        float zRot = 0;
        if (isAL) {
            zRot = currentPosition[X_AXIS] * autolevelTransformation[2] + currentPosition[Y_AXIS] * autolevelTransformation[5];
        }
        // undo safety move from rotMin/max plus correct for real bed rotation
        if (homeDir[Z_AXIS] < 0) { // z min homing
            zpCorr = zpCorr - zRot + rotMin[Z_AXIS] - rotMax[Z_AXIS];
        } else { // z max homing
            zpCorr = zpCorr - zRot + rotMax[Z_AXIS] - rotMin[Z_AXIS];
        }

        if (homeDir[Z_AXIS] < 0 && ZProbe != nullptr) {
            zpCorr += ZProbeHandler::getCoating() - ZProbeHandler::getZProbeHeight();
        }
        if (zpCorr != 0.0f) { // anything to do?
            setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, zpCorr);
            bool isNoDest = Printer::isNoDestinationCheck();
            Printer::setNoDestinationCheck(true);
            moveRelativeByPrinter(tmpPosition, moveFeedrate[Z_AXIS], false);
            waitForEndOfMoves();
            Printer::setNoDestinationCheck(isNoDest);
            lp = Motion2::lastMotorPos[Motion2::lastMotorIdx];
            FOR_ALL_AXES(i) {
                lp[i] = motorPos[i];
                currentPositionTransformed[i] = oldPos[i];
            }
            updatePositionsFromCurrentTransformed();
        }
    }
    /*
#if ZHOME_PRE_RAISE
    if (zAmountRaised > 0 && !(axes & axisBits[Z_AXIS])) { // undo preraise if z was not homed
        setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, -zAmountRaised);
        moveRelativeByOfficial(tmpPosition, homingFeedrate[Z_AXIS], false);
    }
#endif
*/
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
    if (axes & axisBits[Z_AXIS]) {
        oldCoordinates[X_AXIS] = oldCoordinates[Y_AXIS] = 0;
    }
#endif
    oldCoordinates[E_AXIS] = currentPosition[E_AXIS];
    moveByOfficial(oldCoordinates, moveFeedrate[X_AXIS], false);     // make official pos = homing pos reagrdless of transformation
    if (Tool::getActiveTool() != nullptr && ok && (axes & 7) != 0) { // select only if all is homed or we get unwanted moves! Also only do it if position has changed allowing homing of non position axis in extruder selection.
        Tool::selectTool(activeToolId, true);
    }

    Motion1::printCurrentPosition();
    GUI::popBusy();
}

EndstopDriver& Motion1::endstopForAxisDir(fast8_t axis, bool maxDir) {
    if (maxDir) {
        switch (axis) {
        case X_AXIS:
            return endstopXMax;
        case Y_AXIS:
            return endstopYMax;
        case Z_AXIS:
            return endstopZMax;
        case E_AXIS:
            if (motors[E_AXIS] != nullptr) {
                return *motors[E_AXIS]->getMaxEndstop();
            }
            break;
#if NUM_AXES > A_AXIS
        case A_AXIS:
            return endstopAMax;
            break;
#endif
#if NUM_AXES > B_AXIS
        case B_AXIS:
            return endstopBMax;
            break;
#endif
#if NUM_AXES > C_AXIS
        case C_AXIS:
            return endstopCMax;
            break;
#endif
        }
    } else {
        switch (axis) {
        case X_AXIS:
            return endstopXMin;
        case Y_AXIS:
            return endstopYMin;
        case Z_AXIS:
            return endstopZMin;
        case E_AXIS:
            if (motors[E_AXIS] != nullptr) {
                return *motors[E_AXIS]->getMinEndstop();
            }
            break;
#if NUM_AXES > A_AXIS
        case A_AXIS:
            return endstopAMin;
            break;
#endif
#if NUM_AXES > B_AXIS
        case B_AXIS:
            return endstopBMin;
            break;
#endif
#if NUM_AXES > C_AXIS
        case C_AXIS:
            return endstopCMin;
            break;
#endif
        }
    }
    return endstopNone;
}

void Motion1::setHardwareEndstopsAttached(bool attach, EndstopDriver* specificDriver) {
    attach = alwaysCheckEndstops ? true : attach;
    if (specificDriver) {
        specificDriver->setAttached(attach);
    } else {
        if (ZProbe) {
            ZProbe->setAttached(attach);
        }
        FOR_ALL_AXES(i) {
            if (Motion1::maxAxisEndstops[i]) {
                Motion1::maxAxisEndstops[i]->setAttached(attach);
            }
            if (Motion1::minAxisEndstops[i]) {
                Motion1::minAxisEndstops[i]->setAttached(attach);
            }
        }
    }
}

bool Motion1::simpleHome(fast8_t axis) {
    if (homeDir[axis] == 0) { // nothing to do, just set to min
        setAxisHomed(axis, true);
        currentPosition[axis] = minPos[axis];
        updatePositionsFromCurrent();
        Motion2::setMotorPositionFromTransformed();
        return true;
    }
    bool ok = true;
    EndstopDriver& eStop = endstopForAxisDir(axis, homeDir[axis] > 0);
    const float secureDistance = (maxPosOff[axis] - minPosOff[axis]) * 1.5f;
    const EndstopMode oldMode = endstopMode;
    const EndstopMode newMode = axis == Z_AXIS && ZProbe != nullptr && homeDir[Z_AXIS] < 0 ? EndstopMode::PROBING : EndstopMode::STOP_HIT_AXES;
    endstopMode = newMode;
    Motion1::stopMask = axisBits[axis]; // when this endstop is triggered we are at home
    float dest[NUM_AXES];
    FOR_ALL_AXES(i) {
        dest[i] = IGNORE_COORDINATE;
    }
    setHardwareEndstopsAttached(true, &eStop);
    waitForEndOfMoves(); // defined starting condition

    // First test
    dest[axis] = homeDir[axis] * secureDistance;
    Motion1::axesTriggered = 0;
    if (!eStop.update()) { // don't test if we are still there
        moveRelativeByOfficial(dest, homingFeedrate[axis], false);
        waitForEndOfMoves();
    }
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    HAL::delayMilliseconds(50);

    // Move back for retest
    endstopMode = EndstopMode::DISABLED;
    dest[axis] = -homeDir[axis] * homeRetestDistance[axis];
    Motion1::axesTriggered = 0;
    moveRelativeByOfficial(dest, homingFeedrate[axis], false);
    waitForEndOfMoves();

    // retest
    endstopMode = newMode;
    dest[axis] = homeDir[axis] * homeRetestDistance[axis] * 1.5f;
    Motion1::axesTriggered = 0;
    if (!eStop.update()) {
        moveRelativeByOfficial(dest, homingFeedrate[axis] / homeRetestReduction[axis], false);
        waitForEndOfMoves();
    } else {
        Com::printWarningF(PSTR("Endstop for axis "));
        Com::print(axisNames[axis]);
        Com::printFLN(PSTR(" did not untrigger for retest!"));
        ok = false;
    }
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    HAL::delayMilliseconds(30);
    float minOff, maxOff;
    float curPos = homeDir[axis] > 0 ? maxPos[axis] : minPos[axis];
    Tool::minMaxOffsetForAxis(axis, minOff, maxOff);           // Offsets as defined in tool!
    dest[axis] = -homeDir[axis] * (homeEndstopDistance[axis]); // - Tool::getActiveTool()->getOffsetForAxis(axis));
    if (axis != Z_AXIS) {
        if (homeDir[axis] < 0) {
            dest[axis] += maxOff;
        } else {
            dest[axis] += minOff;
        }
    }
    if (homeDir[axis] < 0) {
        dest[axis] -= rotMin[axis];
    } else {
        dest[axis] -= rotMax[axis];
    }
    if (axis < Z_AXIS) {
        toolOffset[axis] = -Tool::getActiveTool()->getOffsetForAxis(axis);
    } else {                     // z axis special case
        if (homeDir[axis] < 0) { // security to not hit bed
            dest[axis] += rotMax[axis];
        } else {
            dest[axis] += rotMin[axis];
        }
        if (ZProbe == nullptr || homeDir[Z_AXIS] > 0) { // no zprobe involved!
            toolOffset[axis] = -Tool::getActiveTool()->getOffsetForAxis(axis);
        } else {
            // z probe with tool offset z = 0 active
            toolOffset[axis] = 5.0f; // move 5mm up to allow risk free activation of tool in homeAxes
#if ZHOME_HEIGHT > 0
            dest[axis] += static_cast<float>(ZHOME_HEIGHT) - curPos;
            curPos = static_cast<float>(ZHOME_HEIGHT);
#endif
            // coating, z probe height and tool offset get fixed in homeAxes
        }
    }
    endstopMode = EndstopMode::DISABLED;
    setHardwareEndstopsAttached(false, &eStop);
    moveRelativeByOfficial(dest, homingFeedrate[axis], false); // also adds toolOffset!
    waitForEndOfMoves();
    currentPosition[axis] = curPos;
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    endstopMode = oldMode;
    setAxisHomed(axis, true);
    Motion1::axesTriggered = 0;
    return ok;
}

void Motion1::callBeforeHomingOnSteppers() {
    FOR_ALL_AXES(i) {
        if (motors[i]) {
            motors[i]->beforeHoming();
        }
    }
}

void Motion1::callAfterHomingOnSteppers() {
    FOR_ALL_AXES(i) {
        if (motors[i]) {
            motors[i]->afterHoming();
        }
    }
}

void Motion1::reportBuffers() {
    Com::printFLN(PSTR("M1 Buffer:"));
    Com::printFLN(PSTR("length:"), (int)length);
    Com::printFLN(PSTR("lengthUP:"), (int)lengthUnprocessed);
    Com::printFLN(PSTR("first:"), (int)first);
    Com::printFLN(PSTR("last:"), (int)last);
    Com::printFLN(PSTR("process:"), (int)process);
}

PGM_P Motion1::getAxisString(fast8_t axis) {
    switch (axis) {
    case X_AXIS:
        return Com::tXAxis;
    case Y_AXIS:
        return Com::tYAxis;
    case Z_AXIS:
        return Com::tZAxis;
    case E_AXIS:
        return Com::tEAxis;
    case A_AXIS:
        return Com::tAAxis;
    case B_AXIS:
        return Com::tBAxis;
    case C_AXIS:
        return Com::tCAxis;
    }
    return Com::tXAxis; // make compiler happy
}

void Motion1::eepromHandle(bool firstImport) {
    int p = 0;
    FOR_ALL_AXES(i) {
        if (i == E_AXIS) {
            continue;
        }
        EEPROM::handlePrefix(getAxisString(i));
        EEPROM::handleFloat(eprStart + p + EPR_M1_RESOLUTION, Com::tEPRStepsPerMM, 3, resolution[i]);
#if PRINTER_TYPE == PRINTER_TYPE_DUAL_X
        if (i != A_AXIS) {
#endif
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
            if (i != X_AXIS && i != Y_AXIS) {
#endif
                EEPROM::handleFloat(eprStart + p + EPR_M1_MAX_FEEDRATE, PSTR("max. feedrate [mm/s]"), 3, maxFeedrate[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MAX_ACCELERATION, PSTR("max. print acceleration [mm/s^2]"), 3, maxAccelerationEEPROM[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MAX_TRAVEL_ACCELERATION, PSTR("max. travel acceleration [mm/s^2]"), 3, maxTravelAccelerationEEPROM[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_HOMING_FEEDRATE, PSTR("homing feedrate [mm/s]"), 3, homingFeedrate[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MOVE_FEEDRATE, PSTR("move feedrate [mm/s]"), 3, moveFeedrate[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MAX_YANK, PSTR("max. yank(jerk) [mm/s]"), 3, maxYank[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MIN_POS, PSTR("min. position [mm]"), 3, minPos[i]);
                EEPROM::handleFloat(eprStart + p + EPR_M1_MAX_POS, PSTR("max. position [mm]"), 3, maxPos[i]);
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
            }
#endif
#if PRINTER_TYPE == PRINTER_TYPE_DUAL_X
        }
#endif
        EEPROM::handleFloat(eprStart + p + EPR_M1_ENDSTOP_DISTANCE, PSTR("endstop distance after homing [mm]"), 3, homeEndstopDistance[i]);
        p += EPR_M1_ENDSTOP_DISTANCE + 4;
        motors[i]->eepromHandle();
    }
    EEPROM::removePrefix();
#if PRINTER_TYPE != PRINTER_TYPE_DUAL_X
    EEPROM::handleFloat(eprStart + EPR_M1_PARK_X, PSTR("Park position X [mm]"), 2, parkPosition[X_AXIS]);
#endif
    EEPROM::handleFloat(eprStart + EPR_M1_PARK_Y, PSTR("Park position Y [mm]"), 2, parkPosition[Y_AXIS]);
#if PRINTER_TYPE != PRINTER_TYPE_DUAL_X
    EEPROM::handleFloat(eprStart + EPR_M1_PARK_Z, PSTR("Park position Z raise [mm]"), 2, parkPosition[Z_AXIS]);
#endif
    EEPROM::handleByte(eprStart + EPR_M1_ALWAYS_CHECK_ENDSTOPS, PSTR("Always check endstops [0/1]"), alwaysCheckEndstops);
    Motion1::setHardwareEndstopsAttached((alwaysCheckEndstops || Printer::isHoming() || Printer::isZProbingActive())); // Probing/homing checks just in case.
    EEPROM::handleByte(eprStart + EPR_M1_VELOCITY_PROFILE, PSTR("Velocity Profile [0-2]"), Motion2::velocityProfileIndex);
    EEPROM::handleByte(eprStart + EPR_M1_AUTOLEVEL, PSTR("Auto level active [0/1]"), Motion1::autolevelActive);

#if FEATURE_AXISCOMP
    EEPROM::handleFloat(eprStart + EPR_M1_AXIS_COMP_XY, Com::tAxisCompTanXY, 6, axisCompTanXY);
    EEPROM::handleFloat(eprStart + EPR_M1_AXIS_COMP_XZ, Com::tAxisCompTanYZ, 6, axisCompTanYZ);
    EEPROM::handleFloat(eprStart + EPR_M1_AXIS_COMP_YZ, Com::tAxisCompTanXZ, 6, axisCompTanXZ);
#endif

#if FEATURE_RETRACTION

    bool autoRetract = Printer::isAutoretract();
    EEPROM::handleByte(eprStart + EPR_M1_AUTORETRACT, Com::tEPRAutoretractEnabled, autoRetract);
    Printer::setAutoretract(autoRetract, true);

    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_LENGTH, Com::tEPRRetractionLength, 2, retractLength);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_SPEED, Com::tEPRRetractionSpeed, 2, retractSpeed);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_ZLIFT, Com::tEPRRetractionZLift, 2, retractZLift);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_UNDO_SPEED, Com::tEPRRetractionUndoSpeed, 2, retractUndoSpeed);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_UNDO_EXTRA_LENGTH, Com::tEPRRetractionUndoExtraLength, 2, retractUndoExtraLength);

    EEPROM::setSilent(NUM_TOOLS < 2);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_UNDO_EXTRA_LONG_LENGTH, Com::tEPRRetractionUndoExtraLongLength, 2, retractUndoExtraLongLength);
    EEPROM::handleFloat(eprStart + EPR_M1_RETRACT_LONG_LENGTH, Com::tEPRRetractionLongLength, 2, retractLongLength);
    EEPROM::setSilent(0);
#endif

    // Rotation matrix is only read/written but not shown
    if (EEPROM::mode != EEPROMMode::REPORT) {
        for (fast8_t i = 0; i < 9; i++) {
            EEPROM::handleFloat(eprStart + EPR_M1_AUTOLEVEL_MATRIX + 4 * i, nullptr, 6, autolevelTransformation[i]);
        }
    }
    if (firstImport) {
        FOR_ALL_AXES(i) {
            maxAcceleration[i] = maxAccelerationEEPROM[i];
            maxTravelAcceleration[i] = maxTravelAccelerationEEPROM[i];
        }
    }
}
void Motion1::updateDerived() {
    if (Motion2::velocityProfileIndex > 2) {
        Motion2::velocityProfileIndex = 2;
    }
    Motion2::velocityProfile = velocityProfiles[Motion2::velocityProfileIndex];
    updateRotMinMax();
}

void Motion1::eepromReset() {
    setFromConfig();
}

/*
 Transforms theoretical correct coordinates to corrected coordinates resulting from bed rotation
 and shear transformations.

 We have 2 coordinate systems. The printer step position where we want to be. These are the positions
 we send to printers, the theoretical coordinates. In contrast we have the printer coordinates that
 we need to be at to get the desired result, the real coordinates.
*/
void Motion1::transformToPrinter(float x, float y, float z, float& transX, float& transY, float& transZ) {
#if FEATURE_AXISCOMP
    // Axis compensation:
    x = x + y * axisCompTanXY + z * axisCompTanXZ;
    y = y + z * axisCompTanYZ;
#endif
    if (autolevelActive) {
        transX = x * autolevelTransformation[0] + y * autolevelTransformation[3] + z * autolevelTransformation[6];
        transY = x * autolevelTransformation[1] + y * autolevelTransformation[4] + z * autolevelTransformation[7];
        transZ = x * autolevelTransformation[2] + y * autolevelTransformation[5] + z * autolevelTransformation[8];
    } else {
        transX = x;
        transY = y;
        transZ = z;
    }
}

/* Transform back to real printer coordinates. */
void Motion1::transformFromPrinter(float x, float y, float z, float& transX, float& transY, float& transZ) {
    if (autolevelActive) {
        transX = x * autolevelTransformation[0] + y * autolevelTransformation[1] + z * autolevelTransformation[2];
        transY = x * autolevelTransformation[3] + y * autolevelTransformation[4] + z * autolevelTransformation[5];
        transZ = x * autolevelTransformation[6] + y * autolevelTransformation[7] + z * autolevelTransformation[8];
    } else {
        transX = x;
        transY = y;
        transZ = z;
    }
#if FEATURE_AXISCOMP
    // Axis compensation:
    transY = transY - transZ * axisCompTanYZ;
    transX = transX - transY * axisCompTanXY - transZ * axisCompTanXZ;
#endif
}

void Motion1::resetTransformationMatrix(bool silent) {
    autolevelTransformation[0] = autolevelTransformation[4] = autolevelTransformation[8] = 1;
    autolevelTransformation[1] = autolevelTransformation[2] = autolevelTransformation[3] = autolevelTransformation[5] = autolevelTransformation[6] = autolevelTransformation[7] = 0;
    updateRotMinMax();
    if (!silent)
        Com::printInfoFLN(Com::tAutolevelReset);
}

#if LEVELING_METHOD > 0

void Motion1::buildTransformationMatrix(Plane& plane) {
    float z0 = plane.z(0, 0);
    float az = z0 - plane.z(1, 0); // ax = 1, ay = 0
    float bz = z0 - plane.z(0, 1); // bx = 0, by = 1
    // First z direction
    autolevelTransformation[6] = -az;
    autolevelTransformation[7] = -bz;
    autolevelTransformation[8] = 1;
    float len = sqrt(az * az + bz * bz + 1);
    autolevelTransformation[6] /= len;
    autolevelTransformation[7] /= len;
    autolevelTransformation[8] /= len;
    autolevelTransformation[0] = 1;
    autolevelTransformation[1] = 0;
    autolevelTransformation[2] = -autolevelTransformation[6] / autolevelTransformation[8];
    len = sqrt(autolevelTransformation[0] * autolevelTransformation[0] + autolevelTransformation[1] * autolevelTransformation[1] + autolevelTransformation[2] * autolevelTransformation[2]);
    autolevelTransformation[0] /= len;
    autolevelTransformation[1] /= len;
    autolevelTransformation[2] /= len;
    // cross(z,x) y,z)
    autolevelTransformation[3] = autolevelTransformation[7] * autolevelTransformation[2] - autolevelTransformation[8] * autolevelTransformation[1];
    autolevelTransformation[4] = autolevelTransformation[8] * autolevelTransformation[0] - autolevelTransformation[6] * autolevelTransformation[2];
    autolevelTransformation[5] = autolevelTransformation[6] * autolevelTransformation[1] - autolevelTransformation[7] * autolevelTransformation[0];
    len = sqrt(autolevelTransformation[3] * autolevelTransformation[3] + autolevelTransformation[4] * autolevelTransformation[4] + autolevelTransformation[5] * autolevelTransformation[5]);
    autolevelTransformation[3] /= len;
    autolevelTransformation[4] /= len;
    autolevelTransformation[5] /= len;
    updateRotMinMax();

    Com::printArrayFLN(Com::tTransformationMatrix, autolevelTransformation, 9, 6);
}
#endif
