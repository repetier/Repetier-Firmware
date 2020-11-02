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

Motion2Buffer Motion2::buffers[NUM_MOTION2_BUFFER];
volatile fast8_t Motion2::length;
fast8_t Motion2::nextActId;
Motion2Buffer* Motion2::act;
Motion1Buffer* Motion2::actM1;
int32_t Motion2::lastMotorPos[2][NUM_AXES];
volatile int16_t Motion2::openBabysteps[NUM_AXES];
const int16_t Motion2::babystepsPerSegment[NUM_AXES] = BABYSTEPS_PER_BLOCK;
fast8_t Motion2::lastMotorIdx; // index to last pos
VelocityProfile* Motion2::velocityProfile = nullptr;
uint8_t Motion2::velocityProfileIndex;
int Motion2::advanceSteps = 0; // already included advance steps

void Motion2::init() {

    length = nextActId = 0;
    actM1 = nullptr;
    lastMotorIdx = 0;
    for (fast8_t i = 0; i < NUM_MOTION2_BUFFER; i++) {
        buffers[i].id = i;
    }
    FOR_ALL_AXES(i) {
        lastMotorPos[0][i] = 0;
        lastMotorPos[1][i] = 0;
        openBabysteps[i] = 0;
    }
}
// Timer gets called at PREPARE_FREQUENCY so it has enough time to
// prefill data structures required by stepper interrupt. Each segment planned
// is for a 1 / BLOCK_FREQUENCY long period of constant speed. We try to
// precompute up to 16 such tiny buffers and with the double frequency We
// should be on the safe side of never getting an underflow.
__attribute__((optimize("unroll-loops"))) void Motion2::timer() {
#ifdef DEBUG_REVERSAL
    static float lastL = 0;
#endif
    // First check if can push anything into next level
    Motion3Buffer* m3 = Motion3::tryReserve();
    if (m3 == nullptr) { // no free space, do nothing until free
        return;
    }
    // Check if we need to start a new M1 buffer
    if (actM1 == nullptr) {
        if (Motion1::lengthUnprocessed == 0) { // no moves stored
            if (length < NUM_MOTION2_BUFFER) {
                bool doBabystep = false;
                int16_t* babysteps = const_cast<int16_t*>(openBabysteps);
                int32_t doSteps[NUM_AXES];
                FOR_ALL_AXES(i) {
                    if (*babysteps) { // babysteps needed
                        doBabystep = true;
                        if (*babysteps > 0) {
                            doSteps[i] = RMath::min(babystepsPerSegment[i], *babysteps);
                        } else {
                            doSteps[i] = -RMath::min(babystepsPerSegment[i], -*babysteps);
                        }
                        // Com::printFLN(PSTR("BS:"), doSteps[i]);
                        *babysteps -= doSteps[i];
                    } else {
                        doSteps[i] = 0;
                    }
                    babysteps++; // next pointer
                }
                if (doBabystep) { // create pseudo motion
                    Motion1Buffer& buf = Motion1::reserve();
                    buf.flags = FLAG_ACTIVE_SECONDARY;
                    buf.action = Motion1Action::MOVE_BABYSTEPS;
                    buf.state = Motion1State::BACKWARD_PLANNED;
                    buf.maxJoinSpeed = 0;
                    int32_t* start = reinterpret_cast<int32_t*>(buf.start);
                    FOR_ALL_AXES(i) {
                        start[i] = doSteps[i];
                    }
                }
            } else {
                return;
            }
        }
        if (length == NUM_MOTION2_BUFFER) { // buffers full
            return;
        }
        act = &buffers[nextActId++];
        // try to get M1 block for processing
        actM1 = Motion1::forward(act);
        if (actM1 == nullptr) { // nothing to do, undo and return
            nextActId--;
            return;
        }
        if (nextActId == NUM_MOTION2_BUFFER) {
            nextActId = 0;
        }
        act->motion1 = actM1;
        Motion1::intLengthBuffered -= actM1->intLength;
        act->state = Motion2State::NOT_INITIALIZED;
        if (actM1->action == Motion1Action::MOVE && actM1->isCheckEndstops()) {
            // Compute number of steps required
            float pos[NUM_AXES];
            FOR_ALL_AXES(i) {
                pos[i] = actM1->start[i] + actM1->unitDir[i] * actM1->length;
            }
            PrinterType::transform(pos, act->stepsRemaining);
            int32_t* lp = lastMotorPos[lastMotorIdx];
            FOR_ALL_AXES(i) {
                act->stepsRemaining[i] = labs(act->stepsRemaining[i] - *lp);
                lp++;
            }
        }
#ifdef DEBUG_REVERSAL
        lastL = 0;
#endif
        // DEBUG_MSG2_FAST("new ", (int)actM1->action);
        InterruptProtectedBlock ip;
        length++;
    }
    if (actM1->action == Motion1Action::MOVE) {
        if (act->state == Motion2State::NOT_INITIALIZED) {
            act->nextState();
            lastMotorPos[lastMotorIdx][E_AXIS] = lroundf(actM1->start[E_AXIS] * Motion1::resolution[E_AXIS]);
        }
        if (act->state == Motion2State::ACCELERATE_INIT) {
            act->state = Motion2State::ACCELERATING;
            if (velocityProfile->start(0, actM1->startSpeed, actM1->feedrate, act->t1)) {
                act->nextState();
            }
            // DEBUG_MSG2_FAST("se:", VelocityProfile::segments);
            // sFactor = velocityProfile->s;
        } else if (act->state == Motion2State::ACCELERATING) {
            if (velocityProfile->next()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s;
        } else if (act->state == Motion2State::PLATEAU_INIT) {
            act->state = Motion2State::PLATEAU;
            if (velocityProfile->start(act->s1, actM1->feedrate, actM1->feedrate, act->t2)) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->s1;
        } else if (act->state == Motion2State::PLATEAU) {
            if (velocityProfile->next()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->s1;
        } else if (act->state == Motion2State::DECCELERATE_INIT) {
            act->state = Motion2State::DECELERATING;
            // act->soff = act->s1 + act->s2;
            if (velocityProfile->start(act->s1 + act->s2, actM1->feedrate, actM1->endSpeed, act->t3)) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->soff;
        } else if (act->state == Motion2State::DECELERATING) {
            if (velocityProfile->next()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->soff;
        } else if (act->state == Motion2State::FINISHED) {
            // DEBUG_MSG("finished")
            m3->directions = 0;
            m3->usedAxes = 0;
            m3->stepsRemaining = 1;
            m3->last = 1;
            // Need to add this move to handle finished state correctly
            Motion3::pushReserved();
            return;
        }
        float sFactor = velocityProfile->s;
        m3->last = Motion3::skipParentId == act->id;
        if (act->state == Motion2State::FINISHED) {
            // prevent rounding errors
            sFactor = actM1->length;
            m3->last = 1;
        } else {
            if (sFactor > actM1->length) {
                sFactor = actM1->length;
                m3->last = 1;
            }
        }
        // Com::printFLN("sf:", sFactor, 4);
        // Convert float position to integer motor position
        // This step catches all nonlinear behaviour from
        // acceleration profile and printer geometry
#ifdef DEBUG_REVERSAL
        if (sFactor < lastL) {
            if (sFactor < lastL - 0.0001) { // sometimes this happens with inprecision, so catch that case
                Com::printF(PSTR("reversal:"), sFactor, 6);
                Com::printF(Com::tColon, lastL, 6);
                Com::printF(Com::tColon, act->s1, 6);
                Com::printF(Com::tColon, act->s2, 6);
                Com::printFLN(Com::tColon, act->s3, 6);
            }
        }
        lastL = sFactor;
#endif
        float pos[NUM_AXES];
        FOR_ALL_AXES(i) {
            if (actM1->axisUsed & axisBits[i]) {
                pos[i] = actM1->start[i] + sFactor * actM1->unitDir[i];
            } else {
                pos[i] = actM1->start[i];
            }
        }
        Leveling::addDistortion(pos);
        fast8_t nextMotorIdx = 1 - lastMotorIdx;
        int32_t* np = lastMotorPos[nextMotorIdx];
        int32_t* lp = lastMotorPos[lastMotorIdx];
        PrinterType::transform(pos, np); // where we want to be in motor pos incl. bump corrections
        // Com::printFLN(PSTR("DS x="), np[0]); // TEST
        /* Com::printF(PSTR(" y="), np[1]);
        Com::printFLN(PSTR(" z="), np[2]);*/
        // Fill structures used to update bresenham
        m3->directions = 0;
        m3->usedAxes = 0;
        if ((m3->stepsRemaining = velocityProfile->stepsPerSegment) == 0) {
            if (m3->last) { // extreme case, normally never happens
                // m3->usedAxes = 0;
                m3->stepsRemaining = 1;
                // Need to add this move to handle finished state correctly
                Motion3::pushReserved();
            }
            return; // don't add empty moves
        }
        m3->errorUpdate = m3->stepsRemaining << 1;
        int* delta = m3->delta;
        int16_t* babysteps = const_cast<int16_t*>(openBabysteps);
        uint8_t* bits = axisBits;
        FOR_ALL_AXES(i) {
            *delta = *np - *lp;
            if (*babysteps) { // babysteps needed
                if (*delta > 0) {
                    if (*babysteps > 0) {
                        int16_t steps = RMath::min(babystepsPerSegment[i], *babysteps);
                        if (*delta + steps > static_cast<int>(m3->stepsRemaining)) {
                            steps = m3->stepsRemaining - *delta;
                        }
                        *babysteps -= steps;
                        *delta += steps;
                    } else {
                        int16_t steps = RMath::min(babystepsPerSegment[i], -*babysteps);
                        if (steps - *delta > static_cast<int>(m3->stepsRemaining)) {
                            steps = m3->stepsRemaining + *delta;
                        }
                        *babysteps += steps;
                        *delta -= steps;
                    }
                } else {
                    if (*babysteps < 0) {
                        int16_t steps = RMath::min(babystepsPerSegment[i], -*babysteps);
                        if (steps - *delta > static_cast<int>(m3->stepsRemaining)) {
                            steps = m3->stepsRemaining + *delta;
                        }
                        *babysteps += steps;
                        *delta -= steps;
                    } else {
                        int16_t steps = RMath::min(babystepsPerSegment[i], *babysteps);
                        if (*delta + steps > static_cast<int>(m3->stepsRemaining)) {
                            steps = m3->stepsRemaining - *delta;
                        }
                        *babysteps -= steps;
                        *delta += steps;
                    }
                }
            }
            if (i == E_AXIS && (advanceSteps != 0 || actM1->isAdvance())) {
                // handle advance of E
                int advTarget = velocityProfile->f * actM1->eAdv;
                int advDiff = advTarget - advanceSteps;
                /* Com::printF("adv:", advTarget);
                Com::printF(" d:", *delta);
                Com::printF(" as:", advanceSteps);
                Com::printF(" f:", velocityProfile->f, 2);
                Com::printFLN(" ea:", actM1->eAdv, 4); */

                *delta += advDiff;
                if (*delta > 0) { // extruding
                    if (*delta > static_cast<int>(m3->stepsRemaining)) {
                        advTarget += m3->stepsRemaining - *delta;
                        *delta = m3->stepsRemaining;
                    }
                    *delta <<= 1;
                    m3->directions |= *bits;
                    m3->usedAxes |= *bits;
                } else { // retracting, advDiff is always negative
                    if (*delta < -static_cast<int>(m3->stepsRemaining)) {
                        advTarget += m3->stepsRemaining + *delta;
                        *delta = -m3->stepsRemaining;
                    }
                    *delta = (-*delta) << 1;
                    m3->usedAxes |= *bits;
                }
                advanceSteps = advTarget;
            } else {
                if ((*delta <<= 1) < 0) {
                    *delta = -*delta;
                    m3->usedAxes |= *bits;
                } else if (*delta != 0) {
                    m3->directions |= *bits;
                    m3->usedAxes |= *bits;
                }
            }
            if (*delta > m3->errorUpdate) { // test if more steps wanted then possible, should never happen!
                                            // adjust *np by the number of steps we can not execute so physical step position does not get corrupted.
                                            // That way next segment can correct remaining steps.
#ifdef DEBUG_MOTION_ERRORS
                Com::printF(PSTR("StepError"), (int)i);
                Com::printF(PSTR(" d:"), *delta);
                Com::printFLN(PSTR(" eu:"), m3->errorUpdate);
#endif
                if (m3->directions & *bits) { // positive move, reduce *np
                    *np -= (*delta - m3->errorUpdate) >> 1;
                } else {
                    *np += (*delta - m3->errorUpdate) >> 1;
                }
                *delta = m3->errorUpdate;
            }
            m3->error[i] = -m3->stepsRemaining;
            delta++;
            np++;
            lp++;
            bits++;
            babysteps++;
        } // FOR_ALL_AXES
        /* Com::printF(PSTR("sf:"), sFactor, 4);
        Com::printF(PSTR(" s:"), (int)act->state);
        Com::printF(PSTR(" da:"), m3->delta[A_AXIS]);
        Com::printF(PSTR(" pa:"), pos[A_AXIS], 3);
        // Com::printF(PSTR(" lpx:"), lastMotorPos[lastMotorIdx][0]);
        Com::printF(PSTR(" lpa:"), lastMotorPos[lastMotorIdx][4]);
        // Com::printF(PSTR(" lpy:"), lp[1]);
        // Com::printF(PSTR(" lpz:"), lp[2]);
        // Com::printF(PSTR(" npx:"), lastMotorPos[nextMotorIdx][0]);
        Com::printFLN(PSTR(" npa:"), lastMotorPos[nextMotorIdx][4]);
        // Com::printF(PSTR(" npy:"), np[1]);
        // Com::printFLN(PSTR(" npz:"), np[2]);
        */
        lastMotorIdx = nextMotorIdx;
        m3->parentId = act->id;
        m3->checkEndstops = actM1->isCheckEndstops();
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            Motion1Buffer* motion1 = act->motion1;
            m3->secondSpeed = tool->computeIntensity(velocityProfile->f, motion1->isActiveSecondary(), motion1->secondSpeed, motion1->secondSpeedPerMMPS);
        } else {
            m3->secondSpeed = 0;
        }
        PrinterType::enableMotors(m3->usedAxes);
        if (m3->last) {
            actM1 = nullptr; // select next on next interrupt
        }
    } else if (actM1->action == Motion1Action::MOVE_STEPS) {
        if (act->state == Motion2State::NOT_INITIALIZED) {
            act->nextState();
        }
        if (act->state == Motion2State::ACCELERATE_INIT) {
            act->state = Motion2State::ACCELERATING;
            if (velocityProfile->start(0, actM1->startSpeed, actM1->feedrate, act->t1)) {
                act->nextState();
            }
            // DEBUG_MSG2_FAST("se:", VelocityProfile::segments);
            // sFactor = velocityProfile->s;
        } else if (act->state == Motion2State::ACCELERATING) {
            if (velocityProfile->next()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s;
        } else if (act->state == Motion2State::PLATEAU_INIT) {
            act->state = Motion2State::PLATEAU;
            if (velocityProfile->startConstSpeed(act->s1, actM1->feedrate, act->t2)) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->s1;
        } else if (act->state == Motion2State::PLATEAU) {
            if (velocityProfile->nextConstSpeed()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->s1;
        } else if (act->state == Motion2State::DECCELERATE_INIT) {
            act->state = Motion2State::DECELERATING;
            // act->soff = act->s1 + act->s2;
            if (velocityProfile->start(act->s1 + act->s2, actM1->feedrate, actM1->endSpeed, act->t3)) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->soff;
        } else if (act->state == Motion2State::DECELERATING) {
            if (velocityProfile->next()) {
                act->nextState();
            }
            // sFactor = velocityProfile->s + act->soff;
        } else if (act->state == Motion2State::FINISHED) {
            m3->directions = 0;
            m3->usedAxes = 0;
            m3->stepsRemaining = 1;
            // Need to add this move to handle finished state correctly
            Motion3::pushReserved();
            return;
        }
        float sFactor = velocityProfile->s;
        m3->last = Motion3::skipParentId == act->id;
        if (act->state == Motion2State::FINISHED) {
            // prevent rounding errors
            sFactor = actM1->length;
            m3->last = 1;
        } else {
            if (sFactor > actM1->length) {
                sFactor = actM1->length;
                m3->last = 1;
            }
        }
        // Convert float position to integer motor position
        // This step catches all nonlinear behaviour from
        // acceleration profile and printer geometry
        float pos[NUM_AXES];
        fast8_t nextMotorIdx = 1 - lastMotorIdx;
        int32_t* np = lastMotorPos[nextMotorIdx];
        int32_t* lp = lastMotorPos[lastMotorIdx];
        FOR_ALL_AXES(i) {
            np[i] = lroundf(actM1->start[i] + sFactor * actM1->unitDir[i]);
        }
        /* Com::printF(PSTR("sf:"), sFactor, 4);
        Com::printF(PSTR(" s:"), (int)act->state);
        Com::printF(PSTR(" lpx:"), lp[0]);
        Com::printF(PSTR(" lpa:"), lp[4]);
        // Com::printF(PSTR(" lpy:"), lp[1]);
        // Com::printF(PSTR(" lpz:"), lp[2]);
        Com::printF(PSTR(" npx:"), np[0]);
        Com::printFLN(PSTR(" npa:"), np[4]);
        // Com::printF(PSTR(" npy:"), np[1]);
        // Com::printFLN(PSTR(" npz:"), np[2]);
        */
        // Fill structures used to update bresenham
        m3->directions = 0;
        m3->usedAxes = 0;
        if ((m3->stepsRemaining = velocityProfile->stepsPerSegment) == 0) {
            if (m3->last) { // extreme case, normally never happens
                m3->usedAxes = 0;
                m3->stepsRemaining = 1;
                // Need to add this move to handle finished state correctly
                Motion3::pushReserved();
            }
            return; // don't add empty moves
        }
        m3->errorUpdate = (m3->stepsRemaining << 1);
        FOR_ALL_AXES(i) {
            if ((m3->delta[i] = ((np[i] - lp[i]) << 1)) < 0) {
                m3->delta[i] = -m3->delta[i];
                m3->usedAxes |= axisBits[i];
            } else if (m3->delta[i] != 0) {
                m3->directions |= axisBits[i];
                m3->usedAxes |= axisBits[i];
            }
            if (m3->delta[i] > m3->errorUpdate) { // test if more steps wanted then possible, should never happen!
                                                  // adjust *np by the number of steps we can not execute so physical step position does not get corrupted.
                                                  // That way next segment can correct remaining steps.
#ifdef DEBUG_MOTION_ERRORS
                Com::printF(PSTR("StepError_"), (int)i);
                Com::printF(PSTR(":"), (int32_t)m3->stepsRemaining);
                Com::printFLN(PSTR(","), m3->delta[i] >> 1);
#endif
                if (m3->directions & axisBits[i]) { // positive move, reduce *np
                    np[i] -= (m3->delta[i] - m3->errorUpdate) >> 1;
                } else {
                    np[i] += (m3->delta[i] - m3->errorUpdate) >> 1;
                }
                m3->delta[i] = m3->errorUpdate;
            }
            m3->error[i] = -(m3->stepsRemaining);
        }
        lastMotorIdx = nextMotorIdx;
        m3->parentId = act->id;
        m3->checkEndstops = actM1->isCheckEndstops();
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            Motion1Buffer* motion1 = act->motion1;
            m3->secondSpeed = tool->computeIntensity(velocityProfile->f, motion1->isActiveSecondary(), motion1->secondSpeed, motion1->secondSpeedPerMMPS);
        } else {
            m3->secondSpeed = 0;
        }
        PrinterType::enableMotors(m3->usedAxes);
        if (m3->last) {
            actM1 = nullptr; // select next on next interrupt
        }
    } else if (actM1->action == Motion1Action::MOVE_BABYSTEPS) {
        int32_t* start = reinterpret_cast<int32_t*>(actM1->start);
        m3->last = 1;
        m3->parentId = act->id;
        // Fill structures used to update bresenham
        m3->directions = 0;
        m3->usedAxes = 0;
        m3->stepsRemaining = static_cast<int32_t>(static_cast<float>(STEPPER_FREQUENCY) / static_cast<float>(BLOCK_FREQUENCY));
        m3->errorUpdate = (m3->stepsRemaining << 1);
        FOR_ALL_AXES(i) {
            m3->delta[i] = start[i] << 1;
            if (m3->delta[i] < 0) {
                m3->delta[i] = -m3->delta[i];
                m3->usedAxes |= axisBits[i];
            } else if (m3->delta[i] != 0) {
                m3->directions |= axisBits[i];
                m3->usedAxes |= axisBits[i];
            }
            m3->error[i] = -(m3->stepsRemaining);
        }
        m3->checkEndstops = Motion1::endstopMode != EndstopMode::DISABLED;
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            Motion1Buffer* motion1 = act->motion1;
            m3->secondSpeed = tool->computeIntensity(0, motion1->isActiveSecondary(), motion1->secondSpeed, motion1->secondSpeedPerMMPS);
        } else {
            m3->secondSpeed = 0;
        }
        PrinterType::enableMotors(m3->usedAxes);
        if (m3->last) {
            actM1 = nullptr; // select next on next interrupt
        }
    } else if (actM1->action == Motion1Action::WAIT || actM1->action == Motion1Action::WARMUP) { // just wait a bit
        m3->parentId = act->id;
        m3->usedAxes = 0;
        m3->checkEndstops = 0;
        m3->secondSpeed = actM1->secondSpeed;
        FOR_ALL_AXES(i) {
            m3->delta[i] = 0;
            m3->error[i] = 0;
        }
        if (actM1->feedrate > 32000) {
            m3->stepsRemaining = 32000;
            m3->last = 0;
            actM1->feedrate -= 32000;
        } else {
            m3->stepsRemaining = static_cast<unsigned int>(actM1->feedrate);
            m3->last = 1;
            actM1 = nullptr; // select next on next interrupt
        }
    } else {
        // Unknown action, skip it
        actM1 = nullptr;
    }
    Motion3::pushReserved();
}

// Gets called when an end stop is triggered during motion.
// Will stop all motions stored. For z probing and homing We
// Also note the remainig z steps.

void motorEndstopTriggered(fast8_t axis, bool dir) {
    ufast8_t mask = axisBits[axis];
    Motion1::motorTriggered |= mask;
    if (dir) {
        Motion1::motorDirTriggered |= mask;
    } else {
        Motion1::motorDirTriggered &= ~mask;
    }
}
void Motion2::motorEndstopTriggered(fast8_t axis, bool dir) {
    ufast8_t mask = axisBits[axis];
    Motion1::motorTriggered |= mask;
    if (dir) {
        Motion1::motorDirTriggered |= mask;
    } else {
        Motion1::motorDirTriggered &= ~mask;
    }
    // Com::printFLN(PSTR("MotorTrigger:"), (int)Motion1::motorTriggered); // TEST
    /*Motion1::setAxisHomed(axis, false);
    Motion2Buffer& m2 = Motion2::buffers[act->parentId];
    if (Motion1::endstopMode == EndstopMode::STOP_AT_ANY_HIT || Motion1::endstopMode == EndstopMode::PROBING) {
        FOR_ALL_AXES(i) {
            Motion1::stepsRemaining[i] = m2.stepsRemaining[i];
        }
        Motion3::skipParentId = act->parentId;
        if (Motion1::endstopMode == EndstopMode::STOP_AT_ANY_HIT) {
            // TODO: Trigger fatal error
        }
    } else { // only mark hit axis
        Motion1::stepsRemaining[axis] = m2.stepsRemaining[axis];
        if ((Motion1::stopMask & Motion1::axesTriggered) == Motion1::stopMask) {
            Motion3::skipParentId = act->parentId;
        }
    }*/
}

void endstopTriggered(fast8_t axis, bool dir) {
    InterruptProtectedBlock noInt;
    Motion2::endstopTriggered(Motion3::act, axis, dir);
}

void Motion2::endstopTriggered(Motion3Buffer* act, fast8_t axis, bool dir) {
    if (act == nullptr || act->checkEndstops == false) {
        if (act == nullptr && Motion3::length > 0) {
#if SHORT_STEP_PULSES == 0
            Motion3::unstepMotors();
#endif
            Motion3::activateNext();
            act = Motion3::act;
            if (act->checkEndstops == false) {
                return;
            }
        } else {
            return;
        }
    }
    if (axis == ZPROBE_AXIS) {                              // z probe can trigger before real z min, so ignore during regular print
        if (Motion1::endstopMode != EndstopMode::PROBING) { // ignore if not probing
            return;
        }
        axis = Z_AXIS; // Handle like z axis!
    }
    ufast8_t bitMask = axisBits[axis];
    Motion1::axesTriggered = bitMask;
    if (dir) {
        Motion1::axesDirTriggered |= bitMask;
    } else {
        Motion1::axesDirTriggered &= ~bitMask;
    }
    Motion2Buffer& m2 = Motion2::buffers[act->parentId];
    Motion1Buffer* m1 = m2.motion1;
    if ((m1->axisUsed & bitMask) == 0) { // not motion directory
        return;
    }
    if ((m1->axisDir & bitMask) != (Motion1::axesDirTriggered & bitMask)) {
        return; // we move away so it is a stale signal from other direction
    }
    Motion1::setAxisHomed(axis, false);
    if (Motion1::endstopMode == EndstopMode::STOP_AT_ANY_HIT || Motion1::endstopMode == EndstopMode::PROBING) {
        FOR_ALL_AXES(i) {
            Motion1::stepsRemaining[i] = m2.stepsRemaining[i];
        }
        Motion3::skipParentId = act->parentId;
        if (Motion1::endstopMode == EndstopMode::STOP_AT_ANY_HIT) {
            // TODO: Trigger fatal error
        }
    } else { // only mark hit axis
        Motion1::stepsRemaining[axis] = m2.stepsRemaining[axis];
        if ((Motion1::stopMask & Motion1::axesTriggered) == Motion1::stopMask) {
            Motion3::skipParentId = act->parentId;
        }
    }
}

void Motion2Buffer::nextState() {
    if (state == Motion2State::NOT_INITIALIZED) {
        if (t1 > 0) {
            state = Motion2State::ACCELERATE_INIT;
            return;
        }
        if (t2 > 0) {
            state = Motion2State::PLATEAU_INIT;
            return;
        }
        if (t3 > 0) {
            state = Motion2State::DECCELERATE_INIT;
            return;
        }
    }
    if (state == Motion2State::ACCELERATING) {
        if (t2 > 0) {
            state = Motion2State::PLATEAU_INIT;
            return;
        }
        if (t3 > 0) {
            state = Motion2State::DECCELERATE_INIT;
            return;
        }
    }
    if (state == Motion2State::PLATEAU) {
        if (t3 > 0) {
            state = Motion2State::DECCELERATE_INIT;
            return;
        }
    }
    state = Motion2State::FINISHED;
}

void Motion2::copyMotorPos(int32_t pos[NUM_AXES]) {
    FOR_ALL_AXES(i) {
        pos[i] = lastMotorPos[lastMotorIdx][i];
    }
}

void Motion2::setMotorPositionFromTransformed() {
    PrinterType::transform(Motion1::currentPositionTransformed, lastMotorPos[lastMotorIdx]);
}

void Motion2::reportBuffers() {
    Com::printFLN(PSTR("M2 Buffer:"));
    Com::printFLN(PSTR("m1 ptr:"), (int)actM1);
    Com::printFLN(PSTR("length:"), (int)length);
    Com::printFLN(PSTR("nextActId:"), (int)nextActId);
}
