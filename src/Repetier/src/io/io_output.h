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

IO_OUTPUT(name, pin)
IO_OUTPUT_INVERTED(name, pin)
IO_OUTPUT_FAKE(name)
IO_OUTPUT_LOG(name, output, changeOnly)

*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_OUTPUT
#undef IO_OUTPUT_INVERTED
#undef IO_OUTPUT_FAKE
#undef IO_OUTPUT_LOG

#if IO_TARGET == IO_TARGET_INIT // Init pins

#define IO_OUTPUT(name, pin) \
    SET_OUTPUT(pin); \
    WRITE(pin, 0);

#define IO_OUTPUT_INVERTED(name, pin) \
    SET_OUTPUT(pin); \
    WRITE(pin, 1);

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // define class

#define IO_OUTPUT(name, pin) \
    class name { \
    public: \
        inline static void set(fast8_t val) { \
            if (val) { \
                WRITE(pin, 1); \
            } else { \
                WRITE(pin, 0); \
            } \
        } \
        inline static void on() { WRITE(pin, 1); } \
        inline static void off() { WRITE(pin, 0); } \
    };

#define IO_OUTPUT_INVERTED(name, pin) \
    class name { \
    public: \
        inline static void set(fast8_t val) { \
            if (val) { \
                WRITE(pin, 0); \
            } else { \
                WRITE(pin, 1); \
            } \
        } \
        inline static void on() { WRITE(pin, 0); } \
        inline static void off() { WRITE(pin, 1); } \
    };

#define IO_OUTPUT_FAKE(name) \
    class name { \
    public: \
        inline static void set(fast8_t val) {} \
        inline static void on() {} \
        inline static void off() {} \
    };

#define IO_OUTPUT_LOG(name, output, changeOnly) \
    class name { \
        static fast8_t state; \
\
    public: \
        inline static void set(fast8_t val) { \
            if (state != val || !changeOnly) { \
                Com::printLogF(PSTR(#name)); \
                Com::printFLN(PSTR("="), static_cast<int>(val)); \
            } \
            state = val; \
            output::set(val); \
        } \
        inline static void on() { set(true); } \
        inline static void off() { set(false); } \
    };
#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define IO_OUTPUT_LOG(name, output, changeOnly) \
    fast8_t name::state = false;

#endif
// Fallback to remove unused macros preventing errors!

#ifndef IO_OUTPUT
#define IO_OUTPUT(name, pin)
#endif
#ifndef IO_OUTPUT_INVERTED
#define IO_OUTPUT_INVERTED(name, pin)
#endif
#ifndef IO_OUTPUT_FAKE
#define IO_OUTPUT_FAKE(name)
#endif
#ifndef IO_OUTPUT_LOG
#define IO_OUTPUT_LOG(name, output, changeOnly)
#endif
