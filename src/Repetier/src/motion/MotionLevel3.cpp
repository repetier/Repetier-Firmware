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

Motion3Buffer* Motion3::tryReserve() {
    if (length < NUM_MOTION3_BUFFER) {
        return &buffers[last];
    }
    return nullptr;
}

void Motion3::pushReserved() {
    last++;
    if (last == NUM_MOTION3_BUFFER) {
        last = 0;
    }
    InterruptProtectedBlock noInts;
    length++;
}

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
    Motion1::motors[X_AXIS]->dir(act->directions & 1);
    Motion1::motors[Y_AXIS]->dir(act->directions & 2);
    Motion1::motors[Z_AXIS]->dir(act->directions & 4);
    for (fast8_t i = 3; i < NUM_AXES; i++) {
        if (act->usedAxes & axisBits[i]) {
            Motion1::motors[i]->dir(act->directions & axisBits[i]);
        }
    }
    // on new line reset triggered axes
    if (act->parentId != lastParentId) {
        // Com::printFLN("rst", act->parentId);
        Motion1::axesTriggered = 0;
        Motion3::skipParentId = 255;
    }
    lastParentId = act->parentId;
    // TODO: Extruder and A,B,C motors

    if (nextActId == NUM_MOTION3_BUFFER) {
        nextActId = 0;
    }
    if (Printer::mode == PRINTER_MODE_FFF) {
        Printer::setFanSpeedDirectly(static_cast<uint8_t>(act->secondSpeed));
    }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    else if (Printer::mode == PRINTER_MODE_LASER) {
        LaserDriver::changeIntensity(act->secondSpeed);
    }
#endif
}

void Motion3::unstepMotors() {
    Motion1::motors[X_AXIS]->unstep();
    Motion1::motors[Y_AXIS]->unstep();
    Motion1::motors[Z_AXIS]->unstep();
    for (fast8_t i = 3; i < NUM_AXES; i++) {
        if (Motion1::motors[i] != nullptr) {
            Motion1::motors[i]->unstep();
        }
    }
}

void Motion3::timer() {
    XMotor.unstep();
    YMotor.unstep();
    ZMotor.unstep();
    for (fast8_t i = 3; i < NUM_AXES; i++) {
        if (Motion1::motors[i] != nullptr) {
            Motion1::motors[i]->unstep();
        }
    }
    if (act == nullptr) {  // need new segment
        if (length == 0) { // nothing prepared
            return;
        }
        activateNext();
    }
    // Run bresenham algorithm for stepping forward

    if (act->checkEndstops) {
        // Endstop handling - skip all following segments
        if (act->parentId == skipParentId) {
            // Com::printFLN("sk", skipParentId);
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
        if ((act->usedAxes & 1) && (Motion1::axesTriggered & 1) == 0) {
            if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
                if (Motion1::motors[X_AXIS]->stepCond()) {
                    Motion2::endstopTriggered(act, X_AXIS);
                }
#else
                if (XMotor.stepCond()) {
                    Motion2::endstopTriggered(act, X_AXIS);
                }
#endif
                m2.stepsRemaining[X_AXIS]--;
                act->error[X_AXIS] -= act->errorUpdate;
            }
        }
        if ((act->usedAxes & 2) && (Motion1::axesTriggered & 2) == 0) {
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                if (YMotor.stepCond()) {
                    Motion2::endstopTriggered(act, Y_AXIS);
                }
                m2.stepsRemaining[Y_AXIS]--;
                act->error[Y_AXIS] -= act->errorUpdate;
            }
        }
        if ((act->usedAxes & 4) && (Motion1::axesTriggered & 4) == 0) {
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                if (Motion1::endstopMode == PROBING) { // ignore z endstop here
                    ZProbe->update();
                    if (ZProbe->triggered()) {
                        Motion2::endstopTriggered(act, Z_AXIS);
                    } else {
                        Motion1::motors[Z_AXIS]->step();
                    }
                } else {
                    if (ZMotor.stepCond()) {
                        Motion2::endstopTriggered(act, Z_AXIS);
                    }
                }
                m2.stepsRemaining[Z_AXIS]--;
                act->error[Z_AXIS] -= act->errorUpdate;
            }
        }
#if NUM_AXES > E_AXIS
        if ((act->usedAxes & 8) && (Motion1::axesTriggered & 8) == 0) {
            if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                if (Motion1::motors[E_AXIS]->stepCond()) {
                    Motion2::endstopTriggered(act, E_AXIS);
                }
                m2.stepsRemaining[E_AXIS]--;
                act->error[E_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > A_AXIS
        if ((act->usedAxes & 16) && (Motion1::axesTriggered & 16) == 0) {
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                if (Motion1::motors[A_AXIS]->stepCond()) {
                    Motion2::endstopTriggered(act, A_AXIS);
                }
                m2.stepsRemaining[A_AXIS]--;
                act->error[A_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > B_AXIS
        if ((act->usedAxes & 8) && (Motion1::axesTriggered & 8) == 0) {
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                if (Motion1::motors[B_AXIS]->stepCond()) {
                    Motion2::endstopTriggered(act, B_AXIS);
                }
                m2.stepsRemaining[B_AXIS]--;
                act->error[B_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > C_AXIS
        if ((act->usedAxes & 8) && (Motion1::axesTriggered & 8) == 0) {
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                if (Motion1::motors[C_AXIS]->stepCond()) {
                    Motion2::endstopTriggered(act, C_AXIS);
                }
                m2.stepsRemaining[C_AXIS]--;
                act->error[C_AXIS] -= act->errorUpdate;
            }
        }
#endif
    } else {
        if (act->usedAxes & 1) {
            if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
                Motion1::motors[X_AXIS]->step();
#else
                XMotor.step();
#endif
                act->error[X_AXIS] -= act->errorUpdate;
            }
        }
        if (act->usedAxes & 2) {
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                YMotor.step();
                act->error[Y_AXIS] -= act->errorUpdate;
            }
        }
        if (act->usedAxes & 4) {
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                ZMotor.step();
                act->error[Z_AXIS] -= act->errorUpdate;
            }
        }
#if NUM_AXES > E_AXIS
        if (act->usedAxes & 8) {
            if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                Motion1::motors[E_AXIS]->step();
                act->error[E_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > A_AXIS
        if (act->usedAxes & 16) {
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                Motion1::motors[A_AXIS]->step();
                act->error[A_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > B_AXIS
        if (act->usedAxes & 32) {
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                Motion1::motors[B_AXIS]->step();
                act->error[B_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > C_AXIS
        if (act->usedAxes & 64) {
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                Motion1::motors[C_AXIS]->step();
                act->error[C_AXIS] -= act->errorUpdate;
            }
        }
#endif
    }
    // Test if we are finished
    if (act->stepsRemaining-- == 0) {
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
