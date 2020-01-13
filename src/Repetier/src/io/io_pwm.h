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
 This file defines all available PWM input types.

// speed 0 = slowest, 256 levels, 4 = fastest, 8 levels
 IO_PWM_SOFTWARE(name,pinname,speed)
 IO_PDM_SOFTWARE(name,pinname)
 // Fake class doing nothing for places that require a pwm output
 IO_PWM_FAKE(name)
 // Convert pwm to digital io. Values from onLevel on turn it
 // on, lower turn it off.
 IO_PWM_SWITCH(name,pinname,onLevel)
 IO_PWM_HARDWARE(name,pinid,frequency)
 // pwmname = name of a pwm output you want to add
 // kickstart functionality to.
 // time100ms = kickstart time in 100ms intervals.
 IO_PWM_KICKSTART(name,pwmname,time100ms)
 
 */

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_PWM_SOFTWARE
#undef IO_PWM_KICKSTART
#undef IO_PDM_SOFTWARE
#undef IO_PWM_FAKE
#undef IO_PWM_SWITCH
#undef IO_PWM_HARDWARE
#undef IO_PWM_MIN_SPEED
#undef IO_PWM_INVERTED
#undef IO_PWM_REPORT
#undef IO_PWM_SCALEDOWN

#if IO_TARGET == IO_TARGET_INIT // init

#define IO_PWM_HARDWARE(name, pinid, frequency) \
    name.id = HAL::initHardwarePWM(pinid, frequency);

#elif IO_TARGET == IO_TARGET_PWM // PWM interrupt

#define IO_PWM_SOFTWARE(name, pinname, speed) \
    if (pwm_count##speed == 0) { \
        if ((name.val = (name.pwm & pwmMasks[speed])) > 0) { \
            pinname::on(); \
        } else { \
            pinname::off(); \
        } \
    } else if (pwm_count##speed == name.val && pwm_count##speed != pwmMasks[speed]) { \
        pinname::off(); \
    }

#define IO_PDM_SOFTWARE(name, pinname) \
    { \
        uint8_t carry; \
        carry = name.error + name.pwm; \
        pinname::set(carry < name.error); \
        name.error = carry; \
    }

#elif IO_TARGET == IO_TARGET_100MS // 100ms

#define IO_PWM_KICKSTART(name, pwmname, timems) \
    if (name.kickcount > 0) { \
        if (--name.kickcount == 0) { \
            pwmname.set(name.pwm); \
        } \
    }

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // class

#define IO_PWM_SOFTWARE(name, pinname, speed) \
    template <class pinname> \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm, val; \
        name##Class() \
            : pwm(0) \
            , val(0) {} \
        void set(fast8_t _pwm) final { pwm = _pwm; } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class<pinname> name;

#define IO_PDM_SOFTWARE(name, pinname) \
    template <class pinname> \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm, error; \
        name##Class() \
            : pwm(0) \
            , error(0) {} \
        void set(fast8_t _pwm) final { pwm = _pwm; } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class<pinname> name;

#define IO_PWM_FAKE(name) \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm; \
        name##Class() \
            : pwm(0) {} \
        void set(fast8_t _pwm) final { pwm = _pwm; } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class name;

#define IO_PWM_SWITCH(name, pinname, onLevel) \
    template <class pinname> \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm; \
        name##Class() \
            : pwm(0) {} \
        void set(fast8_t _pwm) final { \
            pwm = _pwm; \
            pinname::set(_pwm >= onLevel); \
        } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class<pinname> name;

#define IO_PWM_HARDWARE(name, pinid, frequency) \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm, id; \
        name##Class() \
            : pwm(0) {} \
        void set(fast8_t _pwm) final { \
            if (pwm == _pwm) { \
                return; \
            } \
            pwm = _pwm; \
            HAL::setHardwarePWM(id, pwm); \
        } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class name;

#define IO_PWM_MIN_SPEED(name, pwmname, minValue, offBelow) \
    class name##Class : public PWMHandler { \
    public: \
        name##Class() void set(fast8_t _pwm) final { \
            if (_pwm > minValue) { \
                pwmname.set(_pwm); \
            } else if (offBelow) { \
                pwmname.set(0); \
            } else { \
                pwmname.set(minValue); \
            } \
        } \
        fast8_t get() final { return pwmname.get(); } \
    }; \
    extern name##Class name;

#define IO_PWM_SCALEDOWN(name, pwmname, maxValue) \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm; \
        name##Class() \
            : pwm(0) {} \
        void set(fast8_t _pwm) final { \
            pwm = _pwm; \
            uint8_t scaled = static_cast<uint8_t>(static_cast<uint16_t>(_pwm) * maxValue / 256); \
            if (scaled > maxValue) { \
                pwmname.set(maxValue); \
            } else { \
                pwmname.set(scaled); \
            } \
        } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class name;

#define IO_PWM_KICKSTART(name, pwmname, time100ms) \
    class name##Class : public PWMHandler { \
    public: \
        fast8_t pwm, kickcount; \
        name##Class() \
            : pwm(0) \
            , kickcount(0) {} \
        void set(fast8_t _pwm) final { \
            if (kickcount == 0 && _pwm < 85 && _pwm > pwm && time100ms > 0) { \
                pwm = _pwm; \
                kickcount = time100ms; \
                pwmname.set(255); \
            } else { \
                pwm = _pwm; \
                if (kickcount == 0) { \
                    pwmname.set(pwm); \
                } \
            } \
        } \
        fast8_t get() final { return pwm; } \
    }; \
    extern name##Class name;

#define IO_PWM_INVERTED(name, pwmname) \
    class name##Class : public PWMHandler { \
    public: \
        name##Class() {} \
        void set(fast8_t _pwm) final { \
            pwmname.set(255 - _pwm); \
        } \
        fast8_t get() final { return 255 - pwmname.get(); } \
    }; \
    extern name##Class name;
#define IO_PWM_REPORT(name, pwmname) \
    class name##Class : public PWMHandler { \
        fast8_t lastPwm; \
\
    public: \
        name##Class() { lastPwm = 0; } \
        void set(fast8_t _pwm) final { \
            pwmname.set(_pwm); \
            if (lastPwm != _pwm) { \
                Com::printFLN(PSTR(#name) "=", (int)_pwm); \
                lastPwm = _pwm; \
            } \
        } \
        fast8_t get() final { return lastPwm; } \
    }; \
    extern name##Class name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // variable

#define IO_PWM_SOFTWARE(name, pinname, speed) \
    name##Class<pinname> name;

#define IO_PDM_SOFTWARE(name, pinname) \
    name##Class<pinname> name;

#define IO_PWM_FAKE(name) \
    name##Class name;

#define IO_PWM_SWITCH(name, pinname, onLevel) \
    name##Class<pinname> name;

#define IO_PWM_HARDWARE(name, pinid, frequency) \
    name##Class name;

#define IO_PWM_MIN_SPEED(name, pwmname, minValue, offBelow) \
    name##Class name;

#define IO_PWM_KICKSTART(name, pwmname, timems) \
    name##Class name;
#define IO_PWM_INVERTED(name, pwmname) \
    name##Class name;

#define IO_PWM_REPORT(name, pwmname) \
    name##Class name;

#define IO_PWM_SCALEDOWN(name, pwmname, maxValue) \
    name##Class name;

#endif

#ifndef IO_PWM_SOFTWARE
#define IO_PWM_SOFTWARE(name, pinname, speed)
#endif
#ifndef IO_PDM_SOFTWARE
#define IO_PDM_SOFTWARE(name, pinname)
#endif
#ifndef IO_PWM_FAKE
#define IO_PWM_FAKE(name)
#endif
#ifndef IO_PWM_SWITCH
#define IO_PWM_SWITCH(name, pinname, onLevel)
#endif
#ifndef IO_PWM_HARDWARE
#define IO_PWM_HARDWARE(name, pinid, frequency)
#endif
#ifndef IO_PWM_MIN_SPEED
#define IO_PWM_MIN_SPEED(name, pwmname, minValue, offBelow)
#endif
#ifndef IO_PWM_KICKSTART
#define IO_PWM_KICKSTART(name, pwmname, timems)
#endif
#ifndef IO_PWM_INVERTED
#define IO_PWM_INVERTED(name, pwmname)
#endif
#ifndef IO_PWM_REPORT
#define IO_PWM_REPORT(name, pwmname)
#endif
#ifndef IO_PWM_SCALEDOWN
#define IO_PWM_SCALEDOWN(name, pwmname, maxValue)
#endif
