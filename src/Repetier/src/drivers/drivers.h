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

// Some includes define them as macros causing problems
#undef min
#undef max

class PWMHandler {
public:
    virtual void set(fast8_t pwm) = 0;
    virtual void setFreq(uint32_t freq) = 0;
    virtual uint32_t getFreq() = 0;
    virtual fast8_t get() = 0;
};

class IOTemperature {
public:
    virtual float get() = 0;     /// Return current temperature
    virtual bool isDefect() = 0; /// Return true if sensor is defect
};

extern void motorEndstopTriggered(fast8_t axis, bool dir);
extern void endstopTriggered(fast8_t axis, bool dir);

#include "endstops.h"
#include "stepper.h"
#include "heatManager.h"
#include "coolerManager.h"
#include "../tools/tools.h"