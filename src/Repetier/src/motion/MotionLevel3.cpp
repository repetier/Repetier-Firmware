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

#ifndef SHORT_STEP_PULSES
#define SHORT_STEP_PULSES 0
#endif

Motion3Buffer Motion3::buffers[NUM_MOTION3_BUFFER];
ufast8_t Motion3::lastDirection = 128; // force update
fast8_t volatile Motion3::length;
fast8_t Motion3::last, Motion3::nextActId;
Motion3Buffer* Motion3::act;
Motion1Buffer* Motion3::actM1;
Motion2Buffer* Motion3::actM2;
fast8_t Motion3::skipParentId;
fast8_t Motion3::lastParentId;

void Motion3::init() {
    act = nullptr;
    actM1 = nullptr;
    lastParentId = skipParentId = 255; // does not exists so nothing is skipped
    length = last = nextActId = 0;
    for (fast8_t i = 0; i < NUM_MOTION3_BUFFER; i++) {
        buffers[i].id = i;
    }
}

void Motion3::setDirectionsForNewMotors() {
#ifdef XMOTOR_SWITCHABLE
    Motion1::motors[X_AXIS]->dir(lastDirection & 1);
#else
    XMotor.dir(lastDirection & 1);
#endif
    YMotor.dir(lastDirection & 2);
    ZMotor.dir(lastDirection & 4);
#if NUM_AXES > A_AXIS
    AMotor.dir(lastDirection & 16);
#endif
#if NUM_AXES > B_AXIS
    BMotor.dir(lastDirection & 32);
#endif
#if NUM_AXES > C_AXIS
    CMotor.dir(lastDirection & 64);
#endif
    if (Motion1::dittoMode) {
        for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
            Tool::tools[i]->directionMotor(lastDirection & 8);
        }
    } else {
        if (Motion1::motors[E_AXIS]) {
            Motion1::motors[E_AXIS]->dir(lastDirection & 8);
        }
    }
}
/* Motion3Buffer* Motion3::tryReserve() {
    if (length < NUM_MOTION3_BUFFER) {
        return &buffers[last];
    }
    return nullptr;
} */

/*
 Select next prepared element, update next pointer
 and set fan/laser intensity. Also set directions.
*/
bool Motion3::activateNext() {
    act = &buffers[nextActId++];
    bool newDir = (lastDirection != act->directions) && act->usedAxes;
    // on new line reset triggered axes
    if (act->parentId != lastParentId) {
        actM2 = &Motion2::buffers[act->parentId];
        actM1 = actM2->motion1;
        Motion1::axesTriggered = 0;
        Motion1::motorTriggered = 0;
        Motion3::skipParentId = 255;
    }
    if (newDir) {
        lastDirection = act->directions;
        // Set direction first to give driver some time
#ifdef XMOTOR_SWITCHABLE
        Motion1::motors[X_AXIS]->dir(act->directions & 1);
#else
        XMotor.dir(act->directions & 1);
#endif
        YMotor.dir(act->directions & 2);
        ZMotor.dir(act->directions & 4);
#if NUM_AXES > A_AXIS
        AMotor.dir(act->directions & 16);
#endif
#if NUM_AXES > B_AXIS
        BMotor.dir(act->directions & 32);
#endif
#if NUM_AXES > C_AXIS
        CMotor.dir(act->directions & 64);
#endif
        if (Motion1::dittoMode) {
            for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                Tool::tools[i]->directionMotor(act->directions & 8);
            }
        } else {
            if (Motion1::motors[E_AXIS]) {
                Motion1::motors[E_AXIS]->dir(act->directions & 8);
            }
        }
    }

    lastParentId = act->parentId;

    if (nextActId == NUM_MOTION3_BUFFER) {
        nextActId = 0;
    }
    // Tool related activations/changes
    Tool* tool = Tool::getActiveTool();
    if (tool != nullptr) {
        tool->sendSecondary(act->secondSpeed);
    }
    return newDir;
}

void Motion3::timer() {
    InterruptProtectedBlock noInts;
    // Test one motor endstop to simplify stepper logic
#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS) || !defined(NO_MOTOR_ENDSTOPS)
    static fast8_t testMotorId = 0;
#endif

#if SHORT_STEP_PULSES == 0
    unstepMotors();
#endif
    if (act == nullptr) {  // need new segment
        if (length == 0) { // nothing prepared
            return;
        }
#if SLOW_DIRECTION_CHANGE
        if (activateNext()) {
            return; // give time to motor to switch direction
        }
#else
        activateNext();
#endif
    }
    // Run bresenham algorithm for stepping forward

    if (act->usedAxes) {
        if (act->checkEndstops) {
            // Endstop handling - skip all following segments
            if (act->parentId == skipParentId) {
                if (act->last) {
                    Motion2::pop();
                }
                if (--length == 0) {
                    act = nullptr;
                } else {
                    activateNext();
                }
                return;
            }

            // Test one motor endstop to simplify stepper logic
#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS) || !defined(NO_MOTOR_ENDSTOPS)
            ufast8_t axisBit = axisBits[testMotorId];
#endif

#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS)
            if (actM1->axisUsed & axisBit) {
                if (actM1->axisDir & axisBit) {
                    if (Motion1::maxAxisEndstops[testMotorId] && Motion1::maxAxisEndstops[testMotorId]->update()) {
                        Motion2::endstopTriggered(act, testMotorId, true);
                        return;
                    }
                } else {
                    if (Motion1::minAxisEndstops[testMotorId] && Motion1::minAxisEndstops[testMotorId]->update()) {
                        Motion2::endstopTriggered(act, testMotorId, false);
                        return;
                    }
                }
            }
            if (Motion1::endstopMode == EndstopMode::PROBING) {
                if (ZProbe->update()) { // ignore z endstop here
                    Motion2::endstopTriggered(act, Z_AXIS, false);
                }
            }
#endif

#if !defined(NO_MOTOR_ENDSTOPS)
            StepperDriverBase* m = Motion1::motors[testMotorId];
            if (m != nullptr && (act->usedAxes & axisBit) != 0) {
                if (testMotorId == E_AXIS && Motion1::dittoMode) {
                    for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                        Tool* t = Tool::getTool(i);
                        if (t != nullptr && Tool::getTool(i)->updateMotor()) {
                            Motion2::motorEndstopTriggered(E_AXIS, act->directions & axisBit);
                        }
                    }
                } else if ((Motion1::motorTriggered & axisBit /* m->getAxisBit() */) == 0 && m->updateEndstop()) {
                    Motion2::motorEndstopTriggered(testMotorId /* m->getAxis() */, act->directions & axisBit);
                }
            }
#endif

#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS) || !defined(NO_MOTOR_ENDSTOPS)
            testMotorId++;
            if (testMotorId >= NUM_AXES) {
                testMotorId = X_AXIS;
            }
#endif
            if ((Motion1::motorTriggered & 1) == 0) {
                if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
                    Motion1::motors[X_AXIS]->step();
#else
                    XMotor.step();
#endif
                    actM2->stepsRemaining[X_AXIS]--;
                    act->error[X_AXIS] -= act->errorUpdate;
                }
            }
            if ((Motion1::motorTriggered & 2) == 0) {
                if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                    YMotor.step();
                    actM2->stepsRemaining[Y_AXIS]--;
                    act->error[Y_AXIS] -= act->errorUpdate;
                }
            }
            if ((Motion1::motorTriggered & 4) == 0) {
                if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                    ZMotor.step();
                    actM2->stepsRemaining[Z_AXIS]--;
                    act->error[Z_AXIS] -= act->errorUpdate;
                }
            }
#if NUM_AXES > E_AXIS
            if ((Motion1::motorTriggered & 8) == 0) {
                if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                    if (Motion1::dittoMode) {
                        for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                            Tool::getTool(i)->stepMotor();
                        }
                    } else {
                        if (Motion1::motors[E_AXIS]) {
                            Motion1::motors[E_AXIS]->step();
                        }
                    }
                    actM2->stepsRemaining[E_AXIS]--;
                    act->error[E_AXIS] -= act->errorUpdate;
                }
            }
#endif
#if NUM_AXES > A_AXIS
            if ((Motion1::motorTriggered & 16) == 0) {
                if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                    Motion1::motors[A_AXIS]->step();
                    actM2->stepsRemaining[A_AXIS]--;
                    act->error[A_AXIS] -= act->errorUpdate;
                }
            }
#endif
#if NUM_AXES > B_AXIS
            if ((Motion1::motorTriggered & 32) == 0) {
                if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                    Motion1::motors[B_AXIS]->step();
                    actM2->stepsRemaining[B_AXIS]--;
                    act->error[B_AXIS] -= act->errorUpdate;
                }
            }
#endif
#if NUM_AXES > C_AXIS
            if ((Motion1::motorTriggered & 64) == 0) {
                if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                    Motion1::motors[C_AXIS]->step();
                    actM2->stepsRemaining[C_AXIS]--;
                    act->error[C_AXIS] -= act->errorUpdate;
                }
            }
#endif
        } else { // untested steps
            if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
                Motion1::motors[X_AXIS]->step();
#else
                XMotor.step();
#endif
                act->error[X_AXIS] -= act->errorUpdate;
            }
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                YMotor.step();
                act->error[Y_AXIS] -= act->errorUpdate;
            }
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                ZMotor.step();
                act->error[Z_AXIS] -= act->errorUpdate;
            }
#if NUM_AXES > E_AXIS
            if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                if (Motion1::dittoMode) {
                    for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                        Tool::getTool(i)->stepMotor();
                    }
                } else {
                    if (Motion1::motors[E_AXIS]) {
                        Motion1::motors[E_AXIS]->step();
                    }
                }
                act->error[E_AXIS] -= act->errorUpdate;
            }
#endif
#if NUM_AXES > A_AXIS
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                Motion1::motors[A_AXIS]->step();
                act->error[A_AXIS] -= act->errorUpdate;
            }
#endif
#if NUM_AXES > B_AXIS
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                Motion1::motors[B_AXIS]->step();
                act->error[B_AXIS] -= act->errorUpdate;
            }
#endif
#if NUM_AXES > C_AXIS
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                Motion1::motors[C_AXIS]->step();
                act->error[C_AXIS] -= act->errorUpdate;
            }
#endif
        }
    }
    // Test if we are finished
    if (--act->stepsRemaining == 0) {
        if (act->last) {
            Motion2::pop();
        }
        // InterruptProtectedBlock noInts;
        --length;
        act = nullptr;
    }
#if SHORT_STEP_PULSES == 1
    unstepMotors();
#endif
}

void Motion3::reportBuffers() {
    Com::printFLN(PSTR("M3 Buffer:"));
    Com::printFLN(PSTR("length:"), (int)length);
    Com::printFLN(PSTR("last:"), (int)last);
    Com::printFLN(PSTR("nextActId:"), (int)nextActId);
    Com::printFLN(PSTR("lastParentId:"), (int)lastParentId);
    Com::printFLN(PSTR("skipParentId:"), (int)skipParentId);
}
