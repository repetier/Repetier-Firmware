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

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef SERVO_ANALOG
extern ServoInterface* analogServoSlots[4];

#if IO_TARGET == IO_TARGET_CLASS_DEFINITION // class

#define SERVO_ANALOG(name, slot, output, minVal, maxVal, neutral) \
    class name##Class : public ServoInterface { \
        int position; \
\
    public: \
        name##Class() { \
        } \
        void init() { \
            analogServoSlots[slot] = this; \
            position = neutral; \
            HAL::servoMicroseconds(slot, neutral, 1000); \
        } \
        virtual int getPosition() { return position; } \
        virtual void setPosition(int pos, int32_t timeout) { \
            if (pos < minVal && pos != 0) { \
                pos = minVal; \
            } \
            if (pos > maxVal) { \
                pos = maxVal; \
            } \
            position = pos; \
            HAL::servoMicroseconds(slot, pos, timeout); \
        } \
        virtual void enable() { output::on(); } \
        virtual void disable() { output::off(); } \
        virtual void executeGCode(GCode* com) { \
            if (com->M == 340) { \
                uint16_t r = com->hasR() ? com->R : 0; \
                int s = com->hasS() ? com->S : 0; \
                setPosition(s, r); \
            } \
        } \
    }; \
    extern name##Class name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define SERVO_ANALOG(name, slot, output, minVal, maxVal, neutral) \
    name##Class name;

#elif IO_TARGET == IO_TARGET_INIT

#define SERVO_ANALOG(name, slot, output, minVal, maxVal, neutral) \
    name.init();

#endif

#ifndef SERVO_ANALOG
#define SERVO_ANALOG(name, slot, output, minVal, maxVal, neutral)
#endif
