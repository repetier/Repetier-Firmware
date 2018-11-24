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

Definies the following macros:

IO_INPUT(name, pin)
IO_INPUT_INVERTED(name, pin)
IO_INPUT_PULLUP(name, pin)
IO_INPUT_INVERTED_PULLUP(name, pin)
IO_INPUT_DUMMY(name, state)

*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_INPUT
#undef IO_INPUT_INVERTED
#undef IO_INPUT_PULLUP
#undef IO_INPUT_INVERTED_PULLUP
#undef IO_INPUT_DUMMY

#if IO_TARGET == 1 // Init pins

#define IO_INPUT(name, pin) \
    SET_INPUT(pin);

#define IO_INPUT_INVERTED(name, pin) \
    SET_INPUT(pin);

#define IO_INPUT_PULLUP(name, pin) \
    SET_INPUT(pin); \
    PULLUP(pin, HIGH);

#define IO_INPUT_INVERTED_PULLUP(name, pin) \
    SET_INPUT(pin); \
    PULLUP(pin, HIGH);

#define IO_INPUT_DUMMY(name, state)

#elif IO_TARGET == 2 // PWM interrupt

#define IO_INPUT(name, pin)
#define IO_INPUT_INVERTED(name, pin)
#define IO_INPUT_PULLUP(name, pin)
#define IO_INPUT_INVERTED_PULLUP(name, pin)
#define IO_INPUT_DUMMY(name, state)

#elif IO_TARGET == 4 // define class

#define IO_INPUT(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return READ(_pin); \
        } \
        inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_INVERTED(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return !READ(_pin); \
        } \
        inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_PULLUP(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return READ(_pin); \
        } \
        inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_INVERTED_PULLUP(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return !READ(_pin); \
        } \
        inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_DUMMY(name, state) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return state; \
        } \
        inline static uint8_t pin() { return 255; } \
    };

#else

#define IO_INPUT(name, pin)
#define IO_INPUT_INVERTED(name, pin)
#define IO_INPUT_PULLUP(name, pin)
#define IO_INPUT_INVERTED_PULLUP(name, pin)
#define IO_INPUT_DUMMY(name, state)

#endif