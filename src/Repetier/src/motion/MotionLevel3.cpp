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

Motion3Buffer Motion3::buffers[NUM_MOTION3_BUFFER];
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
        actM2 = &Motion2::buffers[act->parentId];
        actM1 = actM2->motion1;
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
    unstepMotors();
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

            // Test one motor endstop to simplify stepper logic
#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS) || !defined(NO_MOTOR_ENDSTOPS)
        static fast8_t testMotorId = 0;
        fast8_t axisBit = axisBits[testMotorId];
#endif

#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS)
        if (actM1->axisUsed & axisBit) {
            if (m1.axisDir & axisBit) {
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
        if (testMotorId == Z_AXIS && Motion1::endstopMode == PROBING) {
            if (ZProbe->update()) { // ignore z endstop here
                Motion2::endstopTriggered(Z_AXIS, act->directions & axisBit, false);
            }
        }
#endif

#if !defined(NO_MOTOR_ENDSTOPS)
        StepperDriverBase* m = Motion1::motors[testMotorId];
        if (m != nullptr && (act->usedAxes & axisBit)) {
            if (testMotorId == E_AXIS && Motion1::dittoMode) {
                for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
                    Tool* t = Tool::getTool(i);
                    if (t != nullptr && Tool::getTool(i)->updateMotor()) {
                        Motion2::motorEndstopTriggered(E_AXIS, act->directions & axisBit);
                    }
                }
            } else if ((Motion1::motorTriggered & m->getAxisBit()) == 0 && m->updateEndstop()) {
                Motion2::motorEndstopTriggered(m->getAxis(), act->directions & axisBit);
            }
        }
#endif

#if !defined(NO_SOFTWARE_AXIS_ENDSTOPS) || !defined(NO_MOTOR_ENDSTOPS)
        testMotorId++;
        if (testMotorId >= NUM_AXES) {
            testMotorId = 0;
        }
#endif

        if (/* (act->usedAxes & 1) && */ (Motion1::motorTriggered & 1) == 0) {
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
        if (/* (act->usedAxes & 2) && */ (Motion1::motorTriggered & 2) == 0) {
            if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
                YMotor.step();
                actM2->stepsRemaining[Y_AXIS]--;
                act->error[Y_AXIS] -= act->errorUpdate;
            }
        }
        if (/* (act->usedAxes & 4) && */ (Motion1::motorTriggered & 4) == 0) {
            if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
                ZMotor.step();
                actM2->stepsRemaining[Z_AXIS]--;
                act->error[Z_AXIS] -= act->errorUpdate;
            }
        }
#if NUM_AXES > E_AXIS
        if (/* (act->usedAxes & 8) && */ (Motion1::motorTriggered & 8) == 0) {
            if ((act->error[E_AXIS] += act->delta[E_AXIS]) > 0) {
                if (Motion1::motors[E_AXIS]) {
                    Motion1::motors[E_AXIS]->step();
                }
                actM2->stepsRemaining[E_AXIS]--;
                act->error[E_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > A_AXIS
        if (/* (act->usedAxes & 16) && */ (Motion1::motorTriggered & 16) == 0) {
            if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
                Motion1::motors[A_AXIS]->step();
                actM2->stepsRemaining[A_AXIS]--;
                act->error[A_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > B_AXIS
        if (/* (act->usedAxes & 32) && */ (Motion1::motorTriggered & 32) == 0) {
            if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
                Motion1::motors[B_AXIS]->step();
                actM2->stepsRemaining[B_AXIS]--;
                act->error[B_AXIS] -= act->errorUpdate;
            }
        }
#endif
#if NUM_AXES > C_AXIS
        if (/* (act->usedAxes & 64) && */ (Motion1::motorTriggered & 64) == 0) {
            if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
                Motion1::motors[C_AXIS]->step();
                actM2->stepsRemaining[C_AXIS]--;
                act->error[C_AXIS] -= act->errorUpdate;
            }
        }
#endif
    } else { // untested steps
             // if (act->usedAxes & 1) {
        if ((act->error[X_AXIS] += act->delta[X_AXIS]) > 0) {
#ifdef XMOTOR_SWITCHABLE
            Motion1::motors[X_AXIS]->step();
#else
            XMotor.step();
#endif
            act->error[X_AXIS] -= act->errorUpdate;
        }
        // }
        // if (act->usedAxes & 2) {
        if ((act->error[Y_AXIS] += act->delta[Y_AXIS]) > 0) {
            YMotor.step();
            act->error[Y_AXIS] -= act->errorUpdate;
        }
        // }
        // if (act->usedAxes & 4) {
        if ((act->error[Z_AXIS] += act->delta[Z_AXIS]) > 0) {
            ZMotor.step();
            act->error[Z_AXIS] -= act->errorUpdate;
        }
            //}
#if NUM_AXES > E_AXIS
        //if (act->usedAxes & 8) {
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
            //}
#endif
#if NUM_AXES > A_AXIS
        //if (act->usedAxes & 16) {
        if ((act->error[A_AXIS] += act->delta[A_AXIS]) > 0) {
            Motion1::motors[A_AXIS]->step();
            act->error[A_AXIS] -= act->errorUpdate;
        }
            //}
#endif
#if NUM_AXES > B_AXIS
        //if (act->usedAxes & 32) {
        if ((act->error[B_AXIS] += act->delta[B_AXIS]) > 0) {
            Motion1::motors[B_AXIS]->step();
            act->error[B_AXIS] -= act->errorUpdate;
        }
            //}
#endif
#if NUM_AXES > C_AXIS
        //if (act->usedAxes & 64) {
        if ((act->error[C_AXIS] += act->delta[C_AXIS]) > 0) {
            Motion1::motors[C_AXIS]->step();
            act->error[C_AXIS] -= act->errorUpdate;
        }
            //}
#endif
    }
    // Test if we are finished
    if (--act->stepsRemaining == 0) {
        if (act->last) {
            Motion2::pop();
        }
        if (--length == 0) {
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
