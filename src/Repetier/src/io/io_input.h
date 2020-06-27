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
IO_INPUT_LOG(name, input, changeOnly)
IO_INPUT_OR(name, input1, input2)
IO_INPUT_AND(name, input1, input2)
*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_INPUT
#undef IO_INPUT_INVERTED
#undef IO_INPUT_PULLUP
#undef IO_INPUT_INVERTED_PULLUP
#undef IO_INPUT_DUMMY
#undef IO_INPUT_LOG
#undef IO_INPUT_OR
#undef IO_INPUT_AND

#if IO_TARGET == IO_TARGET_INIT // Init pins

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

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // define class

#define IO_INPUT(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return READ(_pin); \
        } \
        constexpr inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_INVERTED(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return !READ(_pin); \
        } \
        constexpr inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_PULLUP(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return READ(_pin); \
        } \
        constexpr inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_INVERTED_PULLUP(name, _pin) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return !READ(_pin); \
        } \
        constexpr inline static uint8_t pin() { return _pin; } \
    };

#define IO_INPUT_DUMMY(name, state) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return state; \
        } \
        constexpr inline static uint8_t pin() { return 255; } \
    };

#define IO_INPUT_LOG(name, input, changeOnly) \
    class name { \
        static fast8_t state; \
\
    public: \
        inline static fast8_t get() { \
            fast8_t val = input::get(); \
            if (state != val || !changeOnly) { \
                Com::printLogF(PSTR(#name)); \
                Com::printFLN(PSTR("="), static_cast<int>(val)); \
            } \
            state = val; \
            return state; \
        } \
        constexpr inline static uint8_t pin() { return input::pin(); } \
    };

#define IO_INPUT_OR(name, input1, input2) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return input1::get() || input2::get(); \
        } \
        constexpr inline static uint8_t pin() { return 255; } \
    };

#define IO_INPUT_AND(name, input1, input2) \
    class name { \
    public: \
        inline static fast8_t get() { \
            return input1::get() && input2::get(); \
        } \
        constexpr inline static uint8_t pin() { return 255; } \
    };

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define IO_INPUT_LOG(name, input, changeOnly) \
    fast8_t name::state = false;

#endif

#ifndef IO_INPUT
#define IO_INPUT(name, pin)
#endif
#ifndef IO_INPUT_INVERTED
#define IO_INPUT_INVERTED(name, pin)
#endif
#ifndef IO_INPUT_PULLUP
#define IO_INPUT_PULLUP(name, pin)
#endif
#ifndef IO_INPUT_INVERTED_PULLUP
#define IO_INPUT_INVERTED_PULLUP(name, pin)
#endif
#ifndef IO_INPUT_DUMMY
#define IO_INPUT_DUMMY(name, state)
#endif
#ifndef IO_INPUT_LOG
#define IO_INPUT_LOG(name, input, changeOnly)
#endif
#ifndef IO_INPUT_OR
#define IO_INPUT_OR(name, input1, input2)
#endif
#ifndef IO_INPUT_AND
#define IO_INPUT_AND(name, input1, input2)
#endif
