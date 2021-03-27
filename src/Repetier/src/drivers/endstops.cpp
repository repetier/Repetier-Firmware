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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"

template <class inp>
inline bool EndstopSwitchDriver<inp>::update() {
    if (state != inp::get()) {
        state = !state;
        historyUpdate(state);
        if (Printer::debugEndStop()) {
            Printer::reportFlagSet(PRINTER_REPORT_FLAG_ENDSTOPS);
        }
    }
    return state;
}

template <class inp, int axis, bool dir>
EndstopSwitchHardwareDriver<inp, axis, dir>::EndstopSwitchHardwareDriver(void_fn_t cb)
    : parent(nullptr)
    , callbackFunc(cb)
    , state(false)
    , attached(false) {
}

template <class inp, int axis, bool dir>
void EndstopSwitchHardwareDriver<inp, axis, dir>::setAttached(bool attach) {
    int attachPin = digitalPinToInterrupt(inp::pin()); // (CPU_ARCH != ARCH_AVR) ? inp::pin() : digitalPinToInterrupt(inp::pin())
    if (attach && !attached) {
        attachInterrupt(attachPin, callbackFunc, CHANGE);
        updateReal();
    } else if (!attach && attached) {
        detachInterrupt(attachPin);
    }
    attached = attach;
}

template <class inp, int axis, bool dir>
inline void EndstopSwitchHardwareDriver<inp, axis, dir>::updateReal() {
    fast8_t newState = inp::get();
    if (state != newState) {
        state = newState;
        historyUpdate(state);
        if (axis >= 0 && newState) { // tell motion planner
            endstopTriggered(axis, dir);
        }
        if (parent != nullptr) {
            parent->updateMaster();
        }
        if (Printer::debugEndStop()) {
            Printer::reportFlagSet(PRINTER_REPORT_FLAG_ENDSTOPS);
        }
    }
}

inline void EndstopStepperControlledDriver::set(bool triggered) {
    if (state != triggered) {
        state = triggered;
        historyUpdate(state);
        if (Printer::debugEndStop()) {
            Printer::reportFlagSet(PRINTER_REPORT_FLAG_ENDSTOPS);
        }
    }
}

template <class inp, int level>
inline bool EndstopSwitchDebounceDriver<inp, level>::update() {
    if (inp::get()) {
        if (state < level) {
            state++;
            if (state == level && Printer::debugEndStop()) {
                historyUpdate(state);
                Printer::reportFlagSet(PRINTER_REPORT_FLAG_ENDSTOPS);
            }
        }
    } else {
        if (state != 0) {
            state = 0;
            historyUpdate(state);
            if (Printer::debugEndStop()) {
                Printer::reportFlagSet(PRINTER_REPORT_FLAG_ENDSTOPS);
            }
        }
    }
    return state;
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
