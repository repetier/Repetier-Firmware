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

extern const float invBlockFrequency;
extern const float invStepperFrequency;

class VelocityProfile {
protected:
    static float d1, d2, d3, d4, d5;
    static float dt; // perfect dt for s computation
    static float ds1, ds2, ds3, ds4, ds5, ds6;

public:
    static float f; ///< speed
    static float s; ///< position 0..1
    static int segmentsLeft, segments, stepsPerSegment;
    /**
    Computes the first speed and initializes variables for
    fast updating following steps.
    Returns true if it was the last segment in planned time series
    */
    virtual bool start(float s0, float vstart, float vend, float time) = 0;
    /**
     Called for every update after start. Returns true if the
     last segment is reached.
    */
    virtual bool next() = 0;

    /** like start for constant speed to use faster math */
    bool startConstSpeed(float s0, float speed, float time);
    /** like next for constant speed to use faster math */
    bool nextConstSpeed();
};

/*
This class uses a linear velcity profile.
Acceleration is constant and real jerk is infinite and
only limited by flexibility of printer.
f is the speed at current position
s is the interval between start and end position (0..1)
*/
class VelocityProfileLinear : public VelocityProfile {
public:
    /**
    Computes the first speed and initializes variables for
    fast updating following steps.
    Returns true if it was the last segment in planned time series
    */
    virtual bool start(float s0, float vstart, float vend, float time);
    /**
     Called for every update after start. Returns true if the
     last segment is reached.
    */
    virtual bool next();
};

/*
This class uses a cubic velcity profile, such that at the start and
end of each move the accelerations are 0.
Jerk is linear over the move. Compared to a linear speed profile the
following facts are true:
a_max = 1.5 * a_linear
j_max = 6 * a_linear
*/
class VelocityProfileCubic : public VelocityProfile {
public:
    /**
    Computes the first speed and initializes variables for
    fast updating following steps.
    Returns true if it was the last segment in planned time series
    */
    virtual bool start(float s0, float vstart, float vend, float time);
    /**
     Called for every update after start. Returns true if the
     last segment is reached.
    */
    virtual bool next();
};

/*
a_max = 1.875 * a_linear
j_max = 5.625 * a_linear
*/
class VelocityProfileQuintic : public VelocityProfile {
public:
    /**
    Computes the first speed and initializes variables for
    fast updating following steps.
    Returns true if it was the last segment in planned time series
    */
    virtual bool start(float s0, float vstart, float vend, float time);
    /**
     Called for every update after start. Returns true if the
     last segment is reached.
    */
    virtual bool next();
};
extern VelocityProfile* velocityProfiles[3];
