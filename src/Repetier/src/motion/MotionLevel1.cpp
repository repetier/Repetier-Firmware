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

uint8_t axisBits[NUM_AXES];
uint8_t allAxes;

Motion1Buffer Motion1::buffers[PRINTLINE_CACHE_SIZE]; // Buffer storage
float Motion1::currentPosition[NUM_AXES];             // Current printer position
float Motion1::currentPositionTransformed[NUM_AXES];
float Motion1::destinationPositionTransformed[NUM_AXES];
float Motion1::tmpPosition[NUM_AXES];
float Motion1::maxFeedrate[NUM_AXES];
float Motion1::homingFeedrate[NUM_AXES];
float Motion1::maxAcceleration[NUM_AXES];
float Motion1::resolution[NUM_AXES];
float Motion1::minPos[NUM_AXES];
float Motion1::maxPos[NUM_AXES];
float Motion1::g92Offsets[NUM_AXES];
float Motion1::maxYank[NUM_AXES];
float Motion1::homeRetestDistance[NUM_AXES];
float Motion1::homeEndstopDistance[NUM_AXES];
float Motion1::homeRetestReduction[NUM_AXES];
float Motion1::memory[MEMORY_POS_SIZE][NUM_AXES + 1];
fast8_t Motion1::memoryPos;
StepperDriverBase* Motion1::motors[NUM_AXES];
fast8_t Motion1::homeDir[NUM_AXES];
fast8_t Motion1::homePriority[NUM_AXES];
fast8_t Motion1::axesHomed;
EndstopMode Motion1::endstopMode;
int32_t Motion1::stepsRemaining[NUM_AXES]; // Steps remaining when testing endstops
fast8_t Motion1::axesTriggered;
fast8_t Motion1::stopMask;

volatile fast8_t Motion1::last;    /// newest entry
volatile fast8_t Motion1::first;   /// first entry
volatile fast8_t Motion1::process; /// being processed
volatile fast8_t Motion1::length;  /// number of entries
volatile fast8_t Motion1::lengthUnprocessed;

void Motion1::init() {
    int i;
    for (i = 0; i < PRINTLINE_CACHE_SIZE; i++) {
        buffers[i].id = i;
        buffers[i].flags = 0;
        buffers[i].state = Motion1State::FREE;
    }
    allAxes = 0;
    axesHomed = 0;
    axesTriggered = 0;
    FOR_ALL_AXES(i) {
        axisBits[i] = (uint8_t)1 << i;
        allAxes |= axisBits[i];
        currentPosition[i] = 0;
        currentPositionTransformed[i] = 0;
        destinationPositionTransformed[i] = 0;
        g92Offsets[i] = 0;
    }
    last = first = length = 0;
    process = lengthUnprocessed = 0;
    endstopMode = EndstopMode::DISABLED;
    setFromConfig(); // initial config
}

void Motion1::setFromConfig() {
    resolution[X_AXIS] = XAXIS_STEPS_PER_MM;
    resolution[Y_AXIS] = YAXIS_STEPS_PER_MM;
    resolution[Z_AXIS] = ZAXIS_STEPS_PER_MM;
    resolution[E_AXIS] = EXT0_STEPS_PER_MM;

    maxYank[X_AXIS] = MAX_JERK;
    maxYank[Y_AXIS] = MAX_JERK;
    maxYank[Z_AXIS] = MAX_ZJERK;
    maxYank[E_AXIS] = EXT0_MAX_START_FEEDRATE;

    maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
    maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
    maxFeedrate[E_AXIS] = EXT0_MAX_FEEDRATE;

    homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X;
    homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y;
    homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z;
    homingFeedrate[E_AXIS] = EXT0_MAX_FEEDRATE;

    maxAcceleration[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    maxAcceleration[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    maxAcceleration[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    maxAcceleration[E_AXIS] = EXT0_MAX_ACCELERATION;

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
    homeEndstopDistance[E_AXIS] = EXT0_MAX_ACCELERATION;

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
    homePriority[E_AXIS] = 0;

    motors[X_AXIS] = &XMotor;
    motors[Y_AXIS] = &YMotor;
    motors[Z_AXIS] = &ZMotor;
    motors[E_AXIS] = &E0Motor;

#if NUM_AXES > A_AXIS
#endif
#if NUM_AXES > B_AXIS
#endif
#if NUM_AXES > C_AXIS
#endif
}

void Motion1::fillPosFromGCode(GCode& code, float pos[NUM_AXES], float fallback) {
    if (code.hasX()) {
        pos[X_AXIS] = code.X;
    } else {
        pos[X_AXIS] = fallback;
    }
    if (code.hasY()) {
        pos[Y_AXIS] = code.Y;
    } else {
        pos[Y_AXIS] = fallback;
    }
    if (code.hasZ()) {
        pos[Z_AXIS] = code.Z;
    } else {
        pos[Z_AXIS] = fallback;
    }
#if NUM_AXES > E_AXIS
    if (code.hasE()) {
        pos[E_AXIS] = code.E;
    } else {
        pos[E_AXIS] = fallback;
    }
#endif
#if NUM_AXES > A_AXIS
    if (code.hasA()) {
        pos[A_AXIS] = code.A;
    } else {
        pos[A_AXIS] = fallback;
    }
#endif
#if NUM_AXES > B_AXIS
    if (code.hasB()) {
        pos[B_AXIS] = code.B;
    } else {
        pos[B_AXIS] = fallback;
    }
#endif
#if NUM_AXES > C_AXIS
    if (code.hasC()) {
        pos[C_AXIS] = code.C;
    } else {
        pos[C_AXIS] = fallback;
    }
#endif
}

void Motion1::fillPosFromGCode(GCode& code, float pos[NUM_AXES], float fallback[NUM_AXES]) {
    if (code.hasX()) {
        pos[X_AXIS] = code.X;
    } else {
        pos[X_AXIS] = fallback[X_AXIS];
    }
    if (code.hasY()) {
        pos[Y_AXIS] = code.Y;
    } else {
        pos[Y_AXIS] = fallback[Y_AXIS];
    }
    if (code.hasZ()) {
        pos[Z_AXIS] = code.Z;
    } else {
        pos[Z_AXIS] = fallback[Z_AXIS];
    }
#if NUM_AXES > E_AXIS
    if (code.hasE()) {
        pos[E_AXIS] = code.E;
    } else {
        pos[E_AXIS] = fallback[E_AXIS];
    }
#endif
#if NUM_AXES > A_AXIS
    if (code.hasA()) {
        pos[A_AXIS] = code.A;
    } else {
        pos[A_AXIS] = fallback[A_AXIS];
    }
#endif
#if NUM_AXES > B_AXIS
    if (code.hasB()) {
        pos[B_AXIS] = code.B;
    } else {
        pos[B_AXIS] = fallback[B_AXIS];
    }
#endif
#if NUM_AXES > C_AXIS
    if (code.hasC()) {
        pos[C_AXIS] = code.C;
    } else {
        pos[C_AXIS] = fallback[C_AXIS];
    }
#endif
}

void Motion1::setMotorForAxis(StepperDriverBase* motor, fast8_t axis) {
    waitForEndOfMoves();
    motors[axis] = motor;
}

fast8_t Motion1::buffersUsed() {
    InterruptProtectedBlock noInts;
    return length;
}

void Motion1::waitForEndOfMoves() {
    while (buffersUsed() > 0) {
        Commands::checkForPeriodicalActions(false);
    }
}

void Motion1::waitForXFreeMoves(fast8_t n, bool allowMoves) {
    while (buffersUsed() >= PRINTLINE_CACHE_SIZE - n) {
        Commands::checkForPeriodicalActions(allowMoves);
    }
}

Motion1Buffer& Motion1::reserve() {
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
void Motion1::pop() {
    Motion1Buffer& b = buffers[last];
    b.state = Motion1State::FREE;
    b.flags = 0; // unblock
                 //DEBUG_MSG2_FAST("pop1:",(int)b.id)
    last++;
    if (last >= PRINTLINE_CACHE_SIZE) {
        last = 0;
    }
    length--;
}

void Motion1::setIgnoreABC(float coords[NUM_AXES]) {
    for (fast8_t i = 4; i < NUM_AXES; i++) {
        coords[i] = IGNORE_COORDINATE;
    }
}

void Motion1::copyCurrentOfficial(float coords[NUM_AXES]) {
    FOR_ALL_AXES(i) {
        coords[i] = currentPosition[i];
    }
}

void Motion1::setTmpPositionXYZ(float x, float y, float z) {
    setIgnoreABC(tmpPosition);
    tmpPosition[X_AXIS] = x;
    tmpPosition[Y_AXIS] = y;
    tmpPosition[Z_AXIS] = z;
    tmpPosition[E_AXIS] = 0;
}

void Motion1::setTmpPositionXYZE(float x, float y, float z, float e) {
    setIgnoreABC(tmpPosition);
    tmpPosition[X_AXIS] = x;
    tmpPosition[Y_AXIS] = y;
    tmpPosition[Z_AXIS] = z;
    tmpPosition[E_AXIS] = e;
}
// Move with coordinates in official coordinates (before offset, transform, ...)
void Motion1::moveByOfficial(float coords[NUM_AXES], float feedrate) {
    for (fast8_t i = 0; i < NUM_AXES; i++) {
        if (coords[i] != IGNORE_COORDINATE) {
            currentPosition[i] = coords[i];
        }
    }
    Printer::transformToPrinter(currentPosition[X_AXIS] + Printer::offsetX,
                                currentPosition[Y_AXIS] + Printer::offsetY,
                                currentPosition[Z_AXIS] + Printer::offsetZ,
                                destinationPositionTransformed[X_AXIS],
                                destinationPositionTransformed[Y_AXIS],
                                destinationPositionTransformed[Z_AXIS]);
    destinationPositionTransformed[Z_AXIS] += Printer::offsetZ2;
    if (coords[E_AXIS] != IGNORE_COORDINATE && !Printer::debugDryrun()
#if MIN_EXTRUDER_TEMP > 30
        && (Extruder::current->tempControl.currentTemperatureC > MIN_EXTRUDER_TEMP || Printer::isColdExtrusionAllowed() || Extruder::current->tempControl.sensorType == 0)
#endif
    ) {
        destinationPositionTransformed[E_AXIS] = coords[E_AXIS];
    }
    if (feedrate == IGNORE_COORDINATE) {
        feedrate = Printer::feedrate;
    }
#if NUM_AXES > A_AXIS
    for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
        destinationPositionTransformed[i] = coords[i];
    }
#endif
    queueMove(feedrate);
}

// Move to the printer coordinates (after offset, transform, ...)
void Motion1::moveByPrinter(float coords[NUM_AXES], float feedrate) {
    FOR_ALL_AXES(i) {
        if (coords[i] == IGNORE_COORDINATE) {
            destinationPositionTransformed[i] = currentPositionTransformed[i];
        } else {
            destinationPositionTransformed[i] = coords[i];
        }
    }
    Printer::transformFromPrinter(
        destinationPositionTransformed[X_AXIS],
        destinationPositionTransformed[Y_AXIS],
        destinationPositionTransformed[Z_AXIS] - Printer::offsetZ2,
        currentPosition[X_AXIS],
        currentPosition[Y_AXIS],
        currentPosition[Z_AXIS]);
    currentPosition[X_AXIS] -= Printer::offsetX; // Offset from active extruder or z probe
    currentPosition[Y_AXIS] -= Printer::offsetY;
    currentPosition[Z_AXIS] -= Printer::offsetZ;
#if NUM_AXES > A_AXIS
    for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
        currentPosition[i] = destinationPositionTransformed[i];
    }
#endif
    queueMove(feedrate);
}

void Motion1::updatePositionsFromCurrent() {
    Printer::transformToPrinter(currentPosition[X_AXIS] + Printer::offsetX,
                                currentPosition[Y_AXIS] + Printer::offsetY,
                                currentPosition[Z_AXIS] + Printer::offsetZ,
                                currentPositionTransformed[X_AXIS],
                                currentPositionTransformed[Y_AXIS],
                                currentPositionTransformed[Z_AXIS]);
    currentPositionTransformed[Z_AXIS] += Printer::offsetZ2;
#if NUM_AXES > E_AXIS
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        currentPositionTransformed[i] = currentPosition[i];
    }
#endif
}

void Motion1::updatePositionsFromCurrentTransformed() {
    Printer::transformFromPrinter(
        currentPositionTransformed[X_AXIS],
        currentPositionTransformed[Y_AXIS],
        currentPositionTransformed[Z_AXIS] - Printer::offsetZ2,
        currentPosition[X_AXIS],
        currentPosition[Y_AXIS],
        currentPosition[Z_AXIS]);
    currentPosition[X_AXIS] -= Printer::offsetX; // Offset from active extruder or z probe
    currentPosition[Y_AXIS] -= Printer::offsetY;
    currentPosition[Z_AXIS] -= Printer::offsetZ;
#if NUM_AXES > E_AXIS
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        currentPosition[i] = destinationPositionTransformed[i];
    }
#endif
}

// Move with coordinates in official coordinates (before offset, transform, ...)
void Motion1::moveRelativeByOfficial(float coords[NUM_AXES], float feedrate) {
    FOR_ALL_AXES(i) {
        if (coords[i] != IGNORE_COORDINATE) {
            coords[i] += currentPosition[i];
        }
    }
    moveByOfficial(coords, feedrate);
}

// Move to the printer coordinates (after offset, transform, ...)
void Motion1::moveRelativeByPrinter(float coords[NUM_AXES], float feedrate) {
    FOR_ALL_AXES(i) {
        if (coords[i] != IGNORE_COORDINATE) {
            coords[i] += currentPositionTransformed[i];
        }
    }
    moveByPrinter(coords, feedrate);
}

/** This function is required to fix some errors left over after homing/z-probing.
 * It assumes that we can move all axes in parallel.
 */
void Motion1::moveRelativeByStepsRelative(int32_t coords[NUM_AXES]) {
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
    buf.feedrate = PrinterType::feedrateForMoveSteps(axesUsed);
    buf.acceleration = PrinterType::accelerationForMoveSteps(axesUsed);
    buf.length = 0;
    buf.startSpeed = buf.endSpeed = 0;
    FOR_ALL_AXES(i) {
        buf.start[i] = lpos[i];
        buf.unitDir[i] = coords[i];
        buf.length += RMath::sqr(buf.unitDir[i] / resolution[i]);
    }
    buf.length = sqrtf(buf.length);
    buf.invLength = 1.0f / buf.length;
    buf.sa2 = 2.0f * buf.length * buf.acceleration;
    buf.state = Motion1State::BACKWARD_PLANNED;
}

void Motion1::queueMove(float feedrate) {
    float delta[NUM_AXES];
    float e2 = 0, length2 = 0;
    fast8_t axisUsed = 0;
    FOR_ALL_AXES(i) {
        if (motors[i] == nullptr) { // for E, A,B,C make motors removeable
            delta[i] = 0;
            continue;
        }
        delta[i] = destinationPositionTransformed[i] - currentPositionTransformed[i];
        if (fabsf(delta[i]) > POSITION_EPSILON) {
            axisUsed += axisBits[i];
            float tmp = delta[i] * delta[i];
            length2 += tmp;
            if (i == E_AXIS) {
                e2 = tmp;
            }
        }
    }

    if (length2 == 0) { // no move, ignore it
        return;
    }

    insertWaitIfNeeded();           // for buffer fillup on first move
    Motion1Buffer& buf = reserve(); // Buffer is blockes because state is set to FREE!

    buf.length = sqrtf(length2);
    buf.invLength = 1.0 / buf.length;

    buf.action = Motion1Action::MOVE;
    if (endstopMode != EndstopMode::DISABLED) {
        buf.flags = FLAG_CHECK_ENDSTOPS;
    } else {
        buf.flags = 0;
    }
    buf.axisUsed = axisUsed;

    if ((buf.axisUsed & 15) > 8) { // not pure e move
        // Need to scale feedrate so E component is not part of speed
        buf.feedrate = feedrate * buf.length / sqrt(length2 - e2);
    } else {
        buf.feedrate = feedrate;
    }

    if (Printer::mode == PRINTER_MODE_FFF) {
        buf.secondSpeed = Printer::fanSpeed;
    }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    else if (Printer::mode == PRINTER_MODE_LASER) {
        buf.secondSpeed = (actM1->axisUsed & 3) != 3 && (LaserDriver::laserOn || (actM1.axisUsed & 8) != 0) ? LaserDriver::intensity : 0);
    }
#endif

    // Check axis contraints like max. feedrate and acceleration
    float reduction = 1.0;
    // float invTime = buf.feedrate * buf.invLength;
    FOR_ALL_AXES(i) {
        buf.start[i] = currentPositionTransformed[i];
        if (!buf.isAxisMoving(i)) { // no influence here
            buf.speed[i] = 0;
            buf.unitDir[i] = 0;
            continue;
        }
        buf.unitDir[i] = delta[i] * buf.invLength;
        buf.speed[i] = buf.unitDir[i] * buf.feedrate;
        if (fabs(buf.speed[i]) > maxFeedrate[i]) {
            reduction = RMath::min(reduction, fabs(buf.speed[i]) / maxFeedrate[i]);
        }
    }
    if (reduction != 1.0) {
        buf.feedrate *= reduction;
        // invTime *= reduction;
        for (fast8_t i = 0; i < NUM_AXES; i++) {
            buf.speed[i] *= reduction;
        }
    }
    // Next we compute highest possible acceleration
    buf.acceleration = 1000000;
    FOR_ALL_AXES(i) {
        if (!buf.isAxisMoving(i)) { // no influence here
            continue;
        }
        float a = fabs(maxAcceleration[i] / buf.unitDir[i]);
        if (a < buf.acceleration) {
            buf.acceleration = a;
        }
    }
    buf.sa2 = 2.0f * buf.length * buf.acceleration;
    buf.startSpeed = buf.endSpeed = buf.maxJoinSpeed = buf.calculateSaveStartEndSpeed();
    // Destination becomes new current
    float timeForMove = length / feedrate;
    for (fast8_t i = 0; i < NUM_AXES; i++) {
        currentPositionTransformed[i] = destinationPositionTransformed[i];
    }
    buf.state = Motion1State::RESERVED; // make it accessible

    backplan(buf.id);
    // Com::printF("aid:", (int)buf.id);
    // Com::printFLN(" f:", (int)buf.flags);
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
        buf.state = Motion1State::FORWARD_PLANNED;
    }
}

void Motion1::LaserWarmUp(uint32_t wait) {
    Motion1Buffer& buf = reserve();
    buf.action = Motion1Action::LASER_WARMUP;
    buf.feedrate = ceilf((wait * 1000000.0) * invStepperFrequency);
    buf.flags = 0;
    buf.state = Motion1State::FORWARD_PLANNED;
}

void Motion1::backplan(fast8_t actId) {
    // DEBUG_MSG_FAST("bp1")
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
            act->calculateMaxJoinSpeed(*next);
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
        act->state = BACKWARD_PLANNED;
        maxLoops--;
    } while (maxLoops);
    next->unblock();
    act->unblock();
}

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
            float maxEndSpeed = sqrt(f->startSpeed * f->startSpeed + f->sa2);
            if (f->endSpeed > maxEndSpeed) {
                f->endSpeed = maxEndSpeed;
                if (f2 != nullptr) {
                    f2->startSpeed = f->endSpeed;
                    f2->state = Motion1State::BACKWARD_FINISHED;
                }
            }
            if (f2 != nullptr) {
                f2->unblock();
            }
            float invAcceleration = 1.0 / f->acceleration;
            // Where do we hit if we accelerate from both speed
            // and search for equal speed.
            // t1 is time to accelerate to feedrate
            if (fabsf(f->startSpeed - f->feedrate) < SPEED_EPSILON) {
                m2->t1 = 0;
                m2->s1 = 0;
            } else {
                m2->t1 = (f->feedrate - f->startSpeed) * invAcceleration;
                m2->s1 = m2->t1 * (f->startSpeed + 0.5 * f->acceleration * m2->t1);
            }
            // t3 is time to deccelerate to feedrate
            if (fabsf(f->endSpeed - f->feedrate) < SPEED_EPSILON) {
                m2->t3 = 0;
                m2->s3 = 0;
            } else {
                m2->t3 = (f->feedrate - f->endSpeed) * invAcceleration;
                m2->s3 = m2->t3 * (f->endSpeed + 0.5 * f->acceleration * m2->t3);
            }
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
            Com::printFLN(" l:", f->length, 4);*/
            //Com::printFLN(" a:", f->acceleration, 4);
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
    state = Motion1State::JUNCTION_COMPUTED;
    float vStart, vEnd, factor = 1.0, corrFactor, yank;
    bool limited = false, prevSmaller;
    if (next.feedrate < feedrate) {
        prevSmaller = true;
        corrFactor = next.feedrate / feedrate;
        maxJoinSpeed = next.feedrate;
    } else {
        prevSmaller = false;
        corrFactor = feedrate / next.feedrate;
        maxJoinSpeed = next.feedrate;
    }
    FOR_ALL_AXES(i) {
        if ((axisUsed & axisBits[i]) || (next.axisUsed & axisBits[i])) {
            if (prevSmaller) {
                if (limited) {
                    vEnd = next.speed[i] * factor;
                    vStart = speed[i] * corrFactor * factor;
                } else {
                    vEnd = next.speed[i];
                    vStart = speed[i] * corrFactor;
                }
            } else {
                if (limited) {
                    vEnd = next.speed[i] * corrFactor * factor;
                    vStart = speed[i] * factor;
                } else {
                    vEnd = next.speed[i] * corrFactor;
                    vStart = speed[i];
                }
            }
            yank = fabs(vStart - vEnd);
            if (yank > Motion1::maxYank[i]) {
                factor *= Motion1::maxYank[i] / yank;
                limited = true;
            }
        }
    }
    if (limited) {
        maxJoinSpeed *= factor;
    }
    // check if safe speeds are higher, e.g. because we switch x with z move
    if (next.endSpeed > maxJoinSpeed || startSpeed > maxJoinSpeed) {
        maxJoinSpeed = endSpeed;
        state = Motion1State::BACKWARD_FINISHED;
    }
    //Com::printFLN("mj:", maxJoinSpeed, 1);
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

void Motion1::enableMotors(fast8_t axes) {
    FOR_ALL_AXES(i) {
        if ((axes & axisBits[i]) != 0 && motors[i]) {
            motors[i]->enable();
        }
    }
    Printer::unsetAllSteppersDisabled();
}

bool Motion1::isAxisHomed(fast8_t axis) {
    return (axesHomed & axisBits[axis]) != 0;
}

void Motion1::setAxisHomed(fast8_t axis, bool state) {
    if (state) {
        axis |= axisBits[axis];
    } else {
        axis &= ~axisBits[axis];
    }
}

void Motion1::homeAxes(fast8_t axes) {
    Printer::setHoming(true);
    callBeforeHomingOnSteppers();
#if ZHOME_PRE_RAISE
    if (!isAxisHomed(Z_AXIS) || currentPosition[Z_AXIS] + ZHOME_PRE_RAISE_DISTANCE < maxPos[Z_AXIS]) {
        setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_PRE_RAISE_DISTANCE);
        moveRelativeByOfficial(tmpPosition, homingFeedrate[Z_AXIS]);
    }
#endif
    for (int priority = 0; priority <= 10; priority++) {
        FOR_ALL_AXES(i) {
            if ((axisBits[i] & axes) == 0 && axes != 0) {
                continue;
            }
            if (homePriority[i] == priority) {
                if (i == Z_AXIS) { // Special case if we want z homing at certain position
                    setTmpPositionXYZ(ZHOME_X_POS, ZHOME_Y_POS, IGNORE_COORDINATE);
                    moveByOfficial(tmpPosition, maxFeedrate[X_AXIS]);
                    if (ZProbe != nullptr && homeDir[Z_AXIS] < 0) { // activate z probe
                        ZProbeHandler::activate();
                    }
                }
                PrinterType::homeAxis(i);
                if (i == Z_AXIS && ZProbe != nullptr && homeDir[Z_AXIS] < 0) {
                    ZProbeHandler::deactivate();
                }
#if ZHOME_HEAT_HEIGHT > 0
                if (i == Z_AXIS) {
                    setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT);
                    moveByOfficial(tmpPosition, maxFeedrate[Z_AXIS]);
                }
#endif
            }
        }
    }

    callAfterHomingOnSteppers();
    Printer::setHoming(false);

    // Test if all axes are homed
    bool ok = true;
    FOR_ALL_AXES(i) {
        if (i != E_AXIS && isAxisHomed(i) == false) {
            ok = false;
        }
    }
    Printer::setHomedAll(ok);
}

void Motion1::simpleHome(fast8_t axis) {
    if (homeDir[axis] == 0) { // nothing to do, just set to min
        setAxisHomed(axis, true);
        currentPosition[axis] = minPos[axis];
        updatePositionsFromCurrent();
        Motion2::setMotorPositionFromTransformed();
        return;
    }
    float secureDistance = (maxPos[axis] - minPos[axis]) * 1.5f;
    EndstopMode oldMode = endstopMode;
    EndstopMode newMode = EndstopMode::STOP_HIT_AXES;
    if (axis == Z_AXIS && ZProbe != nullptr && homeDir[Z_AXIS] < 0) {
        newMode = EndstopMode::PROBING;
    }
    endstopMode = newMode;
    Motion1::stopMask = axisBits[axis]; // when this endstop is triggered we are at home
    float dest[NUM_AXES];
    FOR_ALL_AXES(i) {
        dest[i] = IGNORE_COORDINATE;
    }
    waitForEndOfMoves(); // defined starting condition
    dest[axis] = homeDir[axis] * secureDistance;
    Motion1::axesTriggered = 0;
    moveRelativeByOfficial(dest, homingFeedrate[axis]);
    waitForEndOfMoves();
    //Motion3::reportBuffers();
    HAL::delayMilliseconds(100);
    endstopMode = EndstopMode::DISABLED;
    dest[axis] = -homeDir[axis] * homeRetestDistance[axis];
    Motion1::axesTriggered = 0;
    moveRelativeByOfficial(dest, homingFeedrate[axis]);
    waitForEndOfMoves();
    //Motion3::reportBuffers();
    endstopMode = newMode;
    dest[axis] = homeDir[axis] * homeRetestDistance[axis] * 1.5;
    Motion1::axesTriggered = 0;
    moveRelativeByOfficial(dest, homingFeedrate[axis] / homeRetestReduction[axis]);
    waitForEndOfMoves();
    // Motion3::reportBuffers();
    HAL::delayMilliseconds(100);
    dest[axis] = -homeDir[axis] * homeEndstopDistance[axis];
    if (axis == Z_AXIS && ZProbe != nullptr && homeDir[Z_AXIS] < 0) {
        dest[axis] -= ZProbeHandler::getZProbeHeight();
    }
    endstopMode = EndstopMode::DISABLED;
    moveRelativeByOfficial(dest, homingFeedrate[axis]);
    waitForEndOfMoves();
    currentPosition[axis] = homeDir[axis] > 0 ? maxPos[axis] : minPos[axis];
    updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    endstopMode = oldMode;
    setAxisHomed(axis, true);
    Motion1::axesTriggered = 0;
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
