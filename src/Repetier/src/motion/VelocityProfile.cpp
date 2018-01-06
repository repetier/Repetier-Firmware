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

const float invBlockFrequency = 1.0 / static_cast<float>(BLOCK_FREQUENCY);
const float invStepperFrequency = 1.0 / static_cast<float>(STEPPER_FREQUENCY);

#if VELOCITY_PROFILE == 1

float VelocityProfile::f, VelocityProfile::d1, VelocityProfile::s, VelocityProfile::ds1, VelocityProfile::ds2;
int VelocityProfile::segmentsLeft, VelocityProfile::segments, VelocityProfile::stepsPerSegment;

bool VelocityProfile::start(float vstart, float vend, float time)
{
    float segments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / segments;
    VelocityProfile::segments = static_cast<int>(segments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = static_cast<int>(segments) - 1;

    float vdif = vend - vstart;
    ds2 = vdif * h * h;
    d1 = vdif * h;
    f = vstart + 0.5 * d1;
    s = h * vstart + 0.5 * ds2;
    ds1 = vstart * h + 0.5 * ds2;
    return segmentsLeft <= 0;
}

bool VelocityProfile::next()
{
    if (segments <= 0) {
        return false;
    }
    f += d1;
    s += ds1;
    ds1 += ds2;
    return --segmentsLeft <= 0;
}
#endif

#if VELOCITY_PROFILE == 3

float VelocityProfile::f, VelocityProfile::d1, VelocityProfile::d2, VelocityProfile::d3;
float VelocityProfile::dt; // perfect dt for s compuation
float VelocityProfile::s, VelocityProfile::ds1, VelocityProfile::ds2, VelocityProfile::ds3, VelocityProfile::ds4;
int VelocityProfile::segmentsLeft, VelocityProfile::segments, VelocityProfile::stepsPerSegment;

bool VelocityProfile::start(float vstart, float vend, float time)
{
    /*
    Divisions: 1
    Multiplication: 27
    Add/sub: 13
    Total: 41 Operations
    */
    float segments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / segments;
    VelocityProfile::segments = static_cast<int>(segments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = static_cast<int>(segments) - 1;
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

    float c0dt = vstart * h;
    float c2dt3 = c2 * h3;
    float c3dt4 = c3 * h4;

    s = c3dt4 * 0.25 + c2dt3 * 0.3333333333333 + c0dt;
    ds1 = 3.75 * c3dt4 + 2.3333333333 * c2dt3 + c0dt;
    ds2 = 12.5 * c3dt4 + 4.0 * c2dt3;
    ds3 = 15 * c3dt4 + 2.0 * c2dt3;
    ds4 = 6.0 * c3dt4;

    return segmentsLeft <= 0;
}

bool VelocityProfile::next()
{
    // 7 float additions

    if (segments <= 0) {
        return false;
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
#endif

#if VELOCITY_PROFILE == 5

float VelocityProfile::f, VelocityProfile::d1, VelocityProfile::d2, VelocityProfile::d3, VelocityProfile::d4, VelocityProfile::d5;
float VelocityProfile::dt; // perfect dt for s compuation
float VelocityProfile::s, VelocityProfile::ds1, VelocityProfile::ds2, VelocityProfile::ds3, VelocityProfile::ds4, VelocityProfile::ds5, VelocityProfile::ds6;
int VelocityProfile::segmentsLeft, VelocityProfile::segments, VelocityProfile::stepsPerSegment;

bool VelocityProfile::start(float vstart, float vend, float time)
{
    /*
    Divisions: 1
    Multiplication: 27
    Add/sub: 13
    Total: 41 Operations
    */
    float segments = ceil(time * static_cast<float>(BLOCK_FREQUENCY));
    float h = 1.0 / segments;
    VelocityProfile::segments = static_cast<int>(segments);
    stepsPerSegment = static_cast<int>(ceil(time * h * static_cast<float>(STEPPER_FREQUENCY)));
    // tTotal = segments * invStepperFrequency;
    segmentsLeft = static_cast<int>(segments) - 1;
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

    float c0dt = vstart * h;
    float c2dt3 = c2 * h3;
    float c3dt4 = c3 * h4;

    s = c3dt4 * 0.25 + c2dt3 * 0.3333333333333 + c0dt;
    ds1 = 3.75 * c3dt4 + 2.3333333333 * c2dt3 + c0dt;
    ds2 = 12.5 * c3dt4 + 4.0 * c2dt3;
    ds3 = 15 * c3dt4 + 2.0 * c2dt3;
    ds4 = 6.0 * c3dt4;

    return segmentsLeft <= 0;
}

bool VelocityProfile::next()
{
    // 7 float additions

    if (segments <= 0) {
        return false;
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
#endif
