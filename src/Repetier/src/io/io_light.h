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

/*

Lights can be used for many reasons and can also indicate multiple states by using
on/off/burst/slow flash/fast flash or different colors. These can be triggered
from different sources like temperatures, pin states, pwm values, ...
So there will never be a satisfying one catches all solution. Therefore we
split light processing into 3 parts:
- Light state store
- State modifiers ("LIGHT_COND" etc)
- Light driver

Lights are updated in the 100ms timer. First state store resets light to off, white.
The following tests update store and last one wins. 

Example: You have a monochrome led that should be switched by M355 Sx. In case of an error
you want the light to blink slowly. M355 stores last send state in Printer::caseLightMode
variable. So first we need a output pin for the light, create a monochrome store and store
the variable in it. Then the driver should put the state to the output pin.

IO_OUTPUT(caseLightPin, 13)
LIGHT_STATE_MONOCHROME(caseLightState)
LIGHT_COND(caseLightState, true, Printer::caseLightMode, 255, 255, 255)
LIGHT_COND(caseLightState, Printer::isUIErrorMessage(), LIGHT_STATE_BLINK_SLOW, 255, 255, 255)
LIGHT_SOURCE_MONOCHROME(caseLightDriver, caseLightPin, caseLightState)



Definies the following macros:

#define LIGHT_STATE_MONOCHROME(name)
#define LIGHT_STATE_PWM(name) 
#define LIGHT_STATE_RGB(name)
#define LIGHT_COND(state, cond, mode, red, green, blue, brightness) 
#define LIGHT_SOURCE_MONOCHROME(name, output, state)
#define LIGHT_SOURCE_PWM(name, output, state) 


*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#ifndef LIGHT_STATE_OFF
#define LIGHT_STATE_OFF 0
#define LIGHT_STATE_ON 1
#define LIGHT_STATE_BURST 2
#define LIGHT_STATE_BLINK_FAST 3
#define LIGHT_STATE_BLINK_SLOW 4

#endif

#undef LIGHT_STATE_MONOCHROME
#undef LIGHT_STATE_RGB
#undef LIGHT_STATE_PWM
#undef LIGHT_SOURCE_MONOCHROME
#undef LIGHT_SOURCE_PWM
#undef LIGHT_COND

#if IO_TARGET == IO_TARGET_PERIODICAL_ACTIONS

#define LIGHT_SOURCE_PWM(name, output, state) output.set(state.updatePWM());

#endif

#if IO_TARGET == IO_TARGET_100MS // 100ms

#define LIGHT_STATE_MONOCHROME(name) name.reset();
#define LIGHT_STATE_RGB(name) name.reset();
#define LIGHT_STATE_PWM(name) name.reset();

#define LIGHT_SOURCE_MONOCHROME(name, output, state) output::set(state.on());
#define LIGHT_COND(state, cond, mode, red, green, blue, brightness) \
    if (cond) { \
        state.set(mode, red, green, blue, brightness); \
    }

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // class

class LightStoreBase {
public:
    LightStoreBase()
        : mode(0)
        , counter(0) { }
    virtual void reset();
    virtual void set(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) = 0;
    virtual bool on(); ///< Call only once per loop a sit manages blinking as well
    virtual uint8_t red() = 0;
    virtual uint8_t green() = 0;
    virtual uint8_t blue() = 0;
    virtual uint8_t brightness() = 0;

protected:
    fast8_t mode;
    fast8_t counter;
};

class LightStoreMonochrome : public LightStoreBase {
public:
    LightStoreMonochrome();
    virtual void set(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) final;
    virtual uint8_t red() final { return 255; };
    virtual uint8_t green() final { return 255; };
    virtual uint8_t blue() final { return 255; };
    virtual uint8_t brightness() final { return 255; };

private:
};

class LightStoreRGB : public LightStoreBase {
public:
    LightStoreRGB();
    virtual void reset() final;
    virtual void set(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) final;
    virtual uint8_t red() final { return redVal; };
    virtual uint8_t green() final { return greenVal; };
    virtual uint8_t blue() final { return blueVal; };
    virtual uint8_t brightness() final { return 255; };

private:
    uint8_t redVal;
    uint8_t greenVal;
    uint8_t blueVal;
};

class LightStorePWM : public LightStoreBase {
public:
    LightStorePWM()
        : LightStoreBase()
        , refreshRateMS(30)
        , targetPWM(255)
        , curPWM(0)
        , lastUpdate(0)
        , lastBrightness(0)
        , fadeStep(0)
        , finalSetBrightness(0)
        , finalSetMode(0) { };

    virtual void reset() final;
    virtual void set(uint8_t mode, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) final;
    virtual uint8_t red() final { return 255; };
    virtual uint8_t green() final { return 255; };
    virtual uint8_t blue() final { return 255; };
    virtual uint8_t brightness() final { return curPWM; };
    fast8_t updatePWM();

    inline fast8_t rolloverCheck(fast8_t step, bool add) {
#if CPU_ARCH == ARCH_AVR
        if (add) {
            if ((255 - curPWM) < step) {
                return step = (255 - curPWM);
            }
        } else {
            if (curPWM < step) {
                return step = curPWM;
            }
        }
#endif
        return step;
    }
    inline fast8_t computePWMStep(uint16_t durationMS, uint16_t condBrightness) {
        return constrain((refreshRateMS * condBrightness) / durationMS, 1, 255);
    };

private:
    const uint8_t refreshRateMS;
    fast8_t targetPWM;
    fast8_t curPWM;
    millis_t lastUpdate;
    uint8_t lastBrightness;
    fast8_t fadeStep;
    uint8_t finalSetBrightness;
    fast8_t finalSetMode;
};

#define LIGHT_STATE_MONOCHROME(name) \
    extern LightStoreMonochrome name;
#define LIGHT_STATE_RGB(name) \
    extern LightStoreMonochrome name;
#define LIGHT_STATE_PWM(name) \
    extern LightStorePWM name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // variable

#define LIGHT_STATE_MONOCHROME(name) \
    LightStoreMonochrome name;
#define LIGHT_STATE_RGB(name) \
    LightStoreMonochrome name;
#define LIGHT_STATE_PWM(name) \
    LightStorePWM name;

#endif

#ifndef LIGHT_COND
#define LIGHT_COND(state, cond, mode, red, green, blue, brightness)
#endif
#ifndef LIGHT_STATE_MONOCHROME
#define LIGHT_STATE_MONOCHROME(name)
#endif
#ifndef LIGHT_STATE_RGB
#define LIGHT_STATE_RGB(name)
#endif
#ifndef LIGHT_STATE_PWM
#define LIGHT_STATE_PWM(name)
#endif
#ifndef LIGHT_SOURCE_MONOCHROME
#define LIGHT_SOURCE_MONOCHROME(name, output, state)
#endif

#ifndef LIGHT_SOURCE_PWM
#define LIGHT_SOURCE_PWM(name, output, state)
#endif
