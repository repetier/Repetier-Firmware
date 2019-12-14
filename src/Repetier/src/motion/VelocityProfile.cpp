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

const float invBlockFrequency = 1.0 / static_cast<float>(BLOCK_FREQUENCY);
const float invStepperFrequency = 1.0 / static_cast<float>(STEPPER_FREQUENCY);

static VelocityProfileLinear volLinear;
static VelocityProfileCubic volCubic;
static VelocityProfileQuintic volQuintic;
VelocityProfile* velocityProfiles[3] = { &volLinear, &volCubic, &volQuintic };

float VelocityProfile::f, VelocityProfile::s;
int VelocityProfile::segmentsLeft, VelocityProfile::segments, VelocityProfile::stepsPerSegment;

float VelocityProfile::d1, VelocityProfile::d2, VelocityProfile::d3, VelocityProfile::d4, VelocityProfile::d5;
float VelocityProfile::dt; // perfect dt for s computation
float VelocityProfile::ds1, VelocityProfile::ds2, VelocityProfile::ds3, VelocityProfile::ds4, VelocityProfile::ds5, VelocityProfile::ds6;

#if VELOCITY_PROFILE < 0 || VELOCITY_PROFILE > 2
#error VELOCITY_PROFILE must be 0 = linear, 1 = cubic, 2 = quintic
#endif

/** like start for constant speed to use faster math */
bool VelocityProfile::startConstSpeed(float s0, float speed, float time) {
    float fsegments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / fsegments;
    segments = static_cast<int>(fsegments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = segments - 1;
    f = speed;
    h *= time;
    ds1 = h * speed;
    s = ds1 + s0;
    return segmentsLeft <= 0;
}
/** like next for constant speed to use faster math */
bool VelocityProfile::nextConstSpeed() {
    if (segments <= 0) {
        return true; // invalid, start with next segment
    }
    s += ds1;
    return --segmentsLeft <= 0;
}

bool VelocityProfileLinear::start(float s0, float vstart, float vend, float time) {
    float fsegments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / fsegments;
    segments = static_cast<int>(fsegments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = segments - 1;

    float vdif = vend - vstart;
    ds2 = vdif * h * h;
    d1 = vdif * h;
    f = vstart + 0.5 * d1;
    h *= time;
    ds2 *= time;
    ds1 = h * vstart + 0.5 * ds2;
    s = ds1 + s0;
    return segmentsLeft <= 0;
}

bool VelocityProfileLinear::next() {
    if (segments <= 0) {
        return true; // invalid, start with next segment
    }
    f += d1;
    s += ds1;
    ds1 += ds2;
    return --segmentsLeft <= 0;
}

bool VelocityProfileCubic::start(float s0, float vstart, float vend, float time) {
    /*
    Divisions: 1
    Multiplication: 30
    Add/sub: 13
    Total: 41 Operations
    */
    float fsegments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / fsegments;
    segments = static_cast<int>(fsegments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = segments - 1;
    float h2 = h * h;
    float h3 = h2 * h;
    float h4 = h3 * h;

    // ti = 0,5*h
    float vdif = vstart - vend;
    float c3 = vdif + vdif; // 2 * vdif
    float c2 = -vdif - c3;  // -3 vdif
    float c2h2 = c2 * h2;
    float c3h3 = c3 * h3;
    f = c3h3 * 0.125 + c2h2 * 0.25 + vstart;
    d1 = 3.25 * c3h3 + 2.0 * c2h2;
    d2 = 9.0 * c3h3 + 2.0 * c2h2;
    d3 = 6.0 * c3h3;

    // Now update perfect s after the timeperiod

    float c0dt = vstart * h * time;
    float c2dt3 = c2 * h3 * time;
    float c3dt4 = c3 * h4 * time;

    s = c3dt4 * 0.25 + c2dt3 * 0.3333333333333 + c0dt + s0;
    ds1 = 3.75 * c3dt4 + 2.3333333333 * c2dt3 + c0dt;
    ds2 = 12.5 * c3dt4 + 4.0 * c2dt3;
    ds3 = 15 * c3dt4 + 2.0 * c2dt3;
    ds4 = 6.0 * c3dt4;

    return segmentsLeft <= 0;
}

bool VelocityProfileCubic::next() {
    // 7 float additions

    if (segments <= 0) {
        return true; // invalid, start with next segment
    }
    f += d1;
    d1 += d2;
    d2 += d3;

    s += ds1;
    ds1 += ds2;
    ds2 += ds3;
    ds3 += ds4;

    return --segmentsLeft <= 0;
}

bool VelocityProfileQuintic::start(float s0, float vstart, float vend, float time) {
    /*
    Divisions: 1
    Multiplication: 46
    Add/sub: 23
    Total: 66 Operations
    */
    float fsegments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / fsegments;
    segments = static_cast<int>(fsegments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = segments - 1;
    float v10 = vend - vstart;
    float h3v10 = h * h * h * v10;
    float h4v10 = h3v10 * h;
    float h5v10 = h4v10 * h;
    float h6v10 = h5v10 * h;
    float hv0 = h * vstart;
    // Now update perfect s after the timeperiod

    f = 0.1875f * h5v10 - 0.9375f * h4v10 + 1.25 * h3v10 + vstart;
    d1 = 45.375f * h5v10 - 75.0f * h4v10 + 2.5f * h3v10;
    d2 = 495.0f * h5v10 - 435.0f * h4v10 + 90.0f * h3v10;
    d3 = 1530.0f * h5v10 - 720.0f * h4v10 + 60.0f * h3v10;
    d4 = 1800.0f * h5v10 - 360.0f * h4v10;
    d5 = 720.0f * h5v10;

    h6v10 *= time;
    h5v10 *= time;
    h4v10 *= time;
    hv0 *= time;
    s = h6v10 - 3.0f * h5v10 + 2.5f * h4v10 + hv0 + s0;
    ds1 = 63.0f * h6v10 - 93.0f * h5v10 + 37.5f * h4v10 + hv0;
    ds2 = 602.0f * h6v10 - 540.0f * h5v10 + 125.0f * h4v10;
    ds3 = 2100.0f * h6v10 - 1170.0f * h5v10 + 150.0f * h4v10;
    ds4 = 3360.0f * h6v10 - 1080.0f * h5v10 + 60.0f * h4v10;
    ds5 = 2520.0f * h6v10 - 360.0f * h5v10;
    ds6 = 720.0f * h6v10;

    return segmentsLeft <= 0;
}

bool VelocityProfileQuintic::next() {
    // 7 float additions

    if (segments <= 0) {
        return true; // invalid, start with next segment
    }
    f += d1;
    d1 += d2;
    d2 += d3;
    d3 += d4;
    d4 += d5;

    s += ds1;
    ds1 += ds2;
    ds2 += ds3;
    ds3 += ds4;
    ds4 += ds5;
    ds5 += ds6;

    return --segmentsLeft <= 0;
}
