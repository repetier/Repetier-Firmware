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

Motion3Buffer Motion3::buffers[NUM_MOTION3_BUFFER];
fast8_t volatile Motion3::length;
fast8_t Motion3::last, Motion3::nextActId;
Motion3Buffer* Motion3::act;
fast8_t Motion3::skipParentId;
fast8_t Motion3::lastParentId;

void Motion3::init() {
    act = nullptr;
    lastParentId = skipParentId = 255; // does not exists so nothing is skipped
    length = last = nextActId = 0;
    for (fast8_t i = 0; i < NUM_MOTION3_BUFFER; i++) {
        buffers[i].id = i;
    }
}

/* Motion3Buffer* Motion3::tryReserve() {
    if (length < NUM_MOTION3_BUFFER) {
        return &buffers[last];
    }
    return nullptr;
} */

/* void Motion3::pushReserved() {
    last++;
    if (last == NUM_MOTION3_BUFFER) {
        last = 0;
    }
    InterruptProtectedBlock noInts;
    length++;
} */

/*
 Select next prepared element, update next pointer
 and set fan/laser intensity. Also set directions.
*/
void Motion3::activateNext() {
    /* if (length == 0) {
        act = nullptr;
        return;
    }*/
    act = &buffers[nextActId++];
    // Set direction first to give driver some time
#ifdef XMOTOR_SWITCHABLE
    Motion1::motors[X_AXIS]->dir(act->directions & 1);
#else
    XMotor.dir(act->directions & 1);
#endif
    YMotor.dir(act->directions & 2);
    ZMotor.dir(act->directions & 4);
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        if (act->usedAxes & axisBits[i]) {
            Motion1::motors[i]->dir(act->directions & axisBits[i]);
        }
    }
    // on new line reset triggered axes
    if (act->parentId != lastParentId) {
        // Com::printFLN("rst", act->parentId);
        Motion1::axesTriggered = 0;
        Motion1::motorTriggered = 0;
        Motion3::skipParentId = 255;
    }
    lastParentId = act->parentId;
    // TODO: Extruder and A,B,C motors

    if (nextActId == NUM_MOTION3_BUFFER) {
        nextActId = 0;
    }
    // Tool related activations/changes
    // Motion2Buffer& m2 = Motion2::buffers[act->parentId];
    // Motion1Buffer& m1 = *(m2.motion1);
    Tool* tool = Tool::getActiveTool();
    if (tool != nullptr) {
        tool->sendSecondary(act->secondSpeed);
    }
}

/* void Motion3::unstepMotors() {
    Motion1::motors[X_AXIS]->unstep();
    Motion1::motors[Y_AXIS]->unstep();
    Motion1::motors[Z_AXIS]->unstep();
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        if (Motion1::motors[i] != nullptr) {
            Motion1::motors[i]->unstep();
        }
    }
} */

void Motion3::timer() {
    static bool stepDone = false;
    if (stepDone) {
        unstepMotors();
    }
    if (act == nullptr) {  // need new segment
        if (length == 0) { // nothing prepared
            return;
        }
        activateNext();
#ifdef SLOW_DIRECTION_CHANGE
        // Give timer one stepper count to settle direction
        return;
#endif
    }
    // Run bresenham algorithm for stepping forward

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
        Motion2Buffer& m2 = Motion2::buffers[act->parentId];
        Motion1Buffer& m1 = *(m2.motion1);

        if (stepDone) { // last interrupt we had a step, so check endstops
#if !defined(NO_XMIN_ENDSTOP_TEST) || !defined(NO_XMAX_ENDSTOP_TEST)
            if (m1.axisUsed & 1) {
                if (m1.axisDir & 1) {
#ifndef NO_XMAX_ENDSTOP_TEST
                    if (endstopXMax.update()) {
                        Motion2::endstopTriggered(act, X_AXIS, true);
                        return;
                    }
#endif
                } else {
#ifndef NO_XMIN_ENDSTOP_TEST
                    if (endstopXMin.update()) {
                        Motion2::endstopTriggered(act, X_AXIS, false);
                        return;
                    }
#endif
                }
            }
#endif
#if !defined(NO_YMIN_ENDSTOP_TEST) || !defined(NO_YMAX_ENDSTOP_TEST)
            if (m1.axisUsed & 2) {
                if (m1.axisDir & 2) {
#ifndef NO_YMAX_ENDSTOP_TEST
                    if (endstopYMax.update()) {
                        Motion2::endstopTriggered(act, Y_AXIS, true);
                        return;
                    }
#endif
                } else {
#ifndef NO_YMIN_ENDSTOP_TEST
                    if (endstopYMin.update()) {
                        Motion2::endstopTriggered(act, Y_AXIS, false);
                        return;
                    }
#endif
                }
            }
#endif
#if !defined(NO_ZMIN_ENDSTOP_TEST) || !defined(NO_ZMAX_ENDSTOP_TEST)
            if (m1.axisUsed & 4) {
                if (m1.axisDir & 4) {
#ifndef NO_ZMAX_ENDSTOP_TEST
                    if (endstopZMax.update()) {
                        Motion2::endstopTriggered(act, Z_AXIS, true);
                        return;
                    }
#endif
                } else {
#ifndef NO_ZMIN_ENDSTOP_TEST
                    if (endstopZMin.update()) {
                        Motion2::endstopTriggered(act, Z_AXIS, false);
                        return;
                    }
#endif
                }
            }
#endif
#if NUM_AXES > A_AXIS
#if !defined(NO_AMIN_ENDSTOP_TEST) || !defined(NO_AMAX_ENDSTOP_TEST)
            if (m1.axisUsed & 16) {
                if (m1.axisDir & 16) {
#ifndef NO_AMAX_ENDSTOP_TEST
                    if (endstopAMax.update()) {
                        Motion2::endstopTriggered(act, A_AXIS, true);
                        return;
                    }
#endif
                } else {
#ifndef NO_AMIN_ENDSTOP_TEST
                    if (endstopAMin.update()) {
                        Motion2::endstopTriggered(act, , false);
                        return;
                    }
#endif
                }
            }
#endif
#endif
#if NUM_AXES > B_AXIS
            if (m1.axisUsed & 32) {
                if (m1.axisDir & 32) {
                    if (endstopBMax.update()) {
                        Motion2::endstopTriggered(act, B_AXIS, true);
                        return;
                    }
                } else {
                    if (endstopBMin.update()) {
                        Motion2::endstopTriggered(act, B_AXIS, false);
                        return;
                    }
                }
            }
#endif
#if NUM_AXES > C_AXIS
            if (m1.axisUsed & 32) {
                if (m1.axisDir & 32) {
                    if (endstopCMax.update()) {
                        Motion2::endstopTriggered(act, C_AXIS, true);
                        return;
                    }
                } else {
                    if (endstopCMin.update()) {
                        Motion2::endstopTriggered(act, C_AXIS, false);
                        return;
                    }
                }
            }
#endif
            stepDone = false;
        }

        if ((act->usedAxes & 1) && (Motion1::motorTriggered & 1) == 0) {
            if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
                stepDone = true;
#ifdef XMOTOR_SWITCHABLE
                if (Motion1::motors[X_AXIS]->stepCond()) {
                    Motion2::motorEndstopTriggered(X_AXIS, act->directions & 1);
                }
#else
                if (XMotor.stepCond()) {
                    Motion2::motorEndstopTriggered(X_AXIS, act->directions & 1);
                }
#endif
                m2.stepsRemaining[X_AXIS]--;
                act->error[X_AXIS] -= act->errorUpdate;
            }
        }
        if ((act->usedAxes & 2) && (Motion1::motorTriggered & 2) == 0) {
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                stepDone = true;
                if (YMotor.stepCond()) {
                    Motion2::motorEndstopTriggered(Y_AXIS, act->directions & 2);
                }
                m2.stepsRemaining[Y_AXIS]--;
                act->error[Y_AXIS] -= act->errorUpdate;
            }
        }
        if ((act->usedAxes & 4) && (Motion1::motorTriggered & 4) == 0) {
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                stepDone = true;
                if (Motion1::endstopMode == PROBING) { // ignore z endstop here
                    // Com::printFLN(PSTR("A"));
                    if (ZProbe->update()) {
                        Motion2::motorEndstopTriggered(Z_AXIS, act->directions & 4);
                    } else {
                        Motion1::motors[Z_AXIS]->step();
                    }
                } else {
                    // Com::printF(PSTR("Z "));
                    if (ZMotor.stepCond()) {
                        Motion2::motorEndstopTriggered(Z_AXIS, act->directions & 4);
                    }
                }
                m2.stepsRemaining[Z_AXIS]--;
                act->error[Z_AXIS] -= act->errorUpdate;
            }
        }
#if NUM_AXES > E_AXIS
        if ((act->usedAxes & 8) && (Motion1::motorTriggered & 8) == 0) {
            if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                stepDone = true;
                if (Motion1::dittoMode) {
                    for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                        Tool* t = Tool::getTool(i);
                        if (t != nullptr && Tool::getTool(i)->stepCondMotor()) {
                            Motion2::motorEndstopTriggered(E_AXIS, act->directions & 8);
                        }
                    }
                } else {
                    if (Motion1::motors[E_AXIS]) {
                        if (Motion1::motors[E_AXIS]->stepCond()) {
                            Motion2::motorEndstopTriggered(E_AXIS, act->directions & 8);
                        }
                    }
                }
                m2.stepsRemaining[E_AXIS]--;
                act->error[E_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > A_AXIS
        if ((act->usedAxes & 16) && (Motion1::motorTriggered & 16) == 0) {
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                stepDone = true;
                if (Motion1::motors[A_AXIS]->stepCond()) {
                    Motion2::motorEndstopTriggered(A_AXIS, act->directions & 16);
                }
                m2.stepsRemaining[A_AXIS]--;
                act->error[A_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > B_AXIS
        if ((act->usedAxes & 32) && (Motion1::motorTriggered & 32) == 0) {
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                stepDone = true;
                if (Motion1::motors[B_AXIS]->stepCond()) {
                    Motion2::motorEndstopTriggered(B_AXIS, act->directions & 32);
                }
                m2.stepsRemaining[B_AXIS]--;
                act->error[B_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > C_AXIS
        if ((act->usedAxes & 64) && (Motion1::motorTriggered & 64) == 0) {
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                stepDone = true;
                if (Motion1::motors[C_AXIS]->stepCond()) {
                    Motion2::motorEndstopTriggered(C_AXIS, act->directions & 64);
                }
                m2.stepsRemaining[C_AXIS]--;
                act->error[C_AXIS] -= act->errorUpdate;
            }
        }
#endif
    } else {
        stepDone = false;
        if (act->usedAxes & 1) {
            if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
                Motion1::motors[X_AXIS]->step();
#else
                XMotor.step();
#endif
                stepDone = true;
                act->error[X_AXIS] -= act->errorUpdate;
            }
        }
        if (act->usedAxes & 2) {
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                YMotor.step();
                stepDone = true;
                act->error[Y_AXIS] -= act->errorUpdate;
            }
        }
        if (act->usedAxes & 4) {
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                ZMotor.step();
                stepDone = true;
                act->error[Z_AXIS] -= act->errorUpdate;
            }
        }
#if NUM_AXES > E_AXIS
        if (act->usedAxes & 8) {
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
                stepDone = true;
                act->error[E_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > A_AXIS
        if (act->usedAxes & 16) {
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                Motion1::motors[A_AXIS]->step();
                stepDone = true;
                act->error[A_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > B_AXIS
        if (act->usedAxes & 32) {
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                Motion1::motors[B_AXIS]->step();
                stepDone = true;
                act->error[B_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > C_AXIS
        if (act->usedAxes & 64) {
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                stepDone = true;
                Motion1::motors[C_AXIS]->step();
                act->error[C_AXIS] -= act->errorUpdate;
            }
        }
#endif
    }
    // Test if we are finished
    if (--act->stepsRemaining == 0) {
        if (act->last) {
            Motion2::pop();
        }
        length--;
        if (length == 0) {
            act = nullptr;
        } else {
            activateNext();
        }
    }
}

void Motion3::reportBuffers() {
    Com::printFLN(PSTR("M3 Buffer:"));
    Com::printFLN(PSTR("length:"), (int)length);
    Com::printFLN(PSTR("last:"), (int)last);
    Com::printFLN(PSTR("nextActId:"), (int)nextActId);
    Com::printFLN(PSTR("lastParentId:"), (int)lastParentId);
    Com::printFLN(PSTR("skipParentId:"), (int)skipParentId);
}
