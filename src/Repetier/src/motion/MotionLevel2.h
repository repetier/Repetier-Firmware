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

/**
While level 1 was in real coordinates, level 2 goes deeper in th ecoordinates
as it not starts controlling the motors and motor positions are stored in steps
from 0, where 0 position depends on printer type.
 */

#define NUM_MOTION2_BUFFER 4
class Motion3Buffer;
class VelocityProfile;

enum class Motion2State {
    NOT_INITIALIZED = 0,
    ACCELERATE_INIT = 1,
    ACCELERATING = 2,
    PLATEAU_INIT = 3,
    PLATEAU = 4,
    DECCELERATE_INIT = 5,
    DECELERATING = 6,
    FINISHED = 7
};

class Motion2Buffer {
public:
    fast8_t id;
    Motion2State state;
    Motion1Buffer* motion1;
    float t1, t2, t3;
    float s1, s2, s3; // , soff;
    // float sScale1,sScale2,sScale3;
    // float sOffset2, sOffset3;
    int32_t stepsRemaining[NUM_AXES]; // Steps remaining when testing endstops
    void nextState();
};

class Motion2 {
    static Motion2Buffer* act; // used only inside m2 thraed
    static Motion1Buffer* actM1;
    static fast8_t nextActId;

public:
    static VelocityProfile* velocityProfile;
    static uint8_t velocityProfileIndex;
    static Motion2Buffer buffers[NUM_MOTION2_BUFFER];
    static volatile fast8_t length;
    static int32_t lastMotorPos[2][NUM_AXES];
    static fast8_t lastMotorIdx; // index to last pos
    static volatile int16_t openBabysteps[NUM_AXES];
    static const int16_t babystepsPerSegment[NUM_AXES];
    static int advanceSteps; // already included advance steps
    static void init();
    // Timer gets called at PREPARE_FREQUENCY so it has enough time to
    // prefill data structures required by stepper interrupt. Each segment planned
    // is for a 2000 / PREPARE_FREQUENCY long period of constant speed. We try to
    // precompute up to 16 such tiny buffers and with the double frequency We
    // should be on the safe side of never getting an underflow.
    static void timer();

    // Gets called when an end stop is triggered during motion.
    // Will stop all motions stored. For z probing and homing We
    // Also note the remainig z steps.
    static void endstopTriggered(Motion3Buffer* act, fast8_t axis, bool dir);
    static void motorEndstopTriggered(fast8_t axis, bool dir);

    /// Called from m3 timer when line is finished as planned
    static INLINE void pop() {
        InterruptProtectedBlock ip;
        length--;
        Tool* a = Tool::getActiveTool();
        if (a != nullptr) {
            a->moveFinished();
        }
        Motion1::pop();
    }
    static void reportBuffers();

    static void copyMotorPos(int32_t pos[NUM_AXES]);

    /// Assume we are at Motion1::currentPositionTransformed and set motor possitions accordingly.
    /// Use with care or positions might get wrong!
    static void setMotorPositionFromTransformed();
};
