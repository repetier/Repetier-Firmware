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

// ------------- CNC ------------------

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::init() {
    setEepromStart(EEPROM::reserve(EEPROM_SIGNATURE_CNC, 1, Tool::eepromSize() + 2 * 4));
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::eepromHandle() {
    Tool::eepromHandle();
    uint pos = getEepromStart() + Tool::eepromSize();
    EEPROM::handleFloat(pos, PSTR("RPM [-]"), 2, rpm);
    EEPROM::handleLong(pos + 4, PSTR("Start/Stop Delay [ms]"), startStopDelay);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::reset(float offx, float offy, float offz, float _rpm, int32_t _startStopDelay) {
    resetBase(offx, offy, offz);
    rpm = _rpm;
    startStopDelay = _startStopDelay;
}

/// Called when the tool gets activated.
template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::activate() {
    active = false;
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();
    enabledPin::on();
    GCode::executeFString(startScript);
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}
/// Gets called when the tool gets disabled.
template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::deactivate() {
    active = false;
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();

    enabledPin::off();
    GCode::executeFString(endScript);
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::copySettingsToMotion1() {
}

/// Called on kill/emergency to disable the tool
template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::shutdown() {
    enabledPin::off();
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();
    secondary->set(0);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::updateDerived() {
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::M3(GCode* com) {
    Motion1::waitForEndOfMoves();
    if (active) {
        secondary->set(0);
        activePin::off();
        Commands::waitMS(startStopDelay);
    }
    active = true;
    if (com->hasS()) {
        activeSecondaryValue = com->S;
        if (activeSecondaryValue < 0) {
            activeSecondaryValue = 0;
        } else if (activeSecondaryValue > 255) {
            activeSecondaryValue = 255;
        }
        activeSecondaryPerMMPS = 0;
    } else if (com->hasI()) {
        activeSecondaryValue = 0;
        if (com->I < 0) {
            com->I = 0;
        }
        if (com->I > rpm) {
            com->I = rpm;
        }
        activeSecondaryPerMMPS = 255.0 * com->I / rpm;
    } else {
        activeSecondaryValue = 255;
        activeSecondaryPerMMPS = 0;
    }
    if (com->hasR()) {
        Com::printF(PSTR("Fixed PWM:"), activeSecondaryValue);
        Com::printFLN(PSTR(" RPM:"), activeSecondaryValue * rpm / 255.0, 0);
    }
    activePin::on();
    dirPin::on();
    secondary->set(activeSecondaryValue);
    Commands::waitMS(startStopDelay);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::M4(GCode* com) {
    Motion1::waitForEndOfMoves();
    if (active) {
        secondary->set(0);
        activePin::off();
        Commands::waitMS(startStopDelay);
    }
    active = true;
    if (com->hasS()) {
        activeSecondaryValue = com->S;
        if (activeSecondaryValue < 0) {
            activeSecondaryValue = 0;
        } else if (activeSecondaryValue > 255) {
            activeSecondaryValue = 255;
        }
        activeSecondaryPerMMPS = 0;
    } else if (com->hasI()) {
        activeSecondaryValue = 0;
        if (com->I < 0) {
            com->I = 0;
        }
        if (com->I > rpm) {
            com->I = rpm;
        }
        activeSecondaryPerMMPS = 255.0 * com->I / rpm;
    } else {
        activeSecondaryValue = 255;
        activeSecondaryPerMMPS = 0;
    }
    if (com->hasR()) {
        Com::printF(PSTR("Fixed PWM:"), activeSecondaryValue);
        Com::printFLN(PSTR(" RPM:"), activeSecondaryValue * rpm / 255.0, 0);
    }
    activePin::on();
    dirPin::off();
    secondary->set(activeSecondaryValue);
    Commands::waitMS(startStopDelay);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::M5(GCode* com) {
    if (!active) {
        return;
    }
    active = false;
    Motion1::waitForEndOfMoves();
    secondary->set(0);
    activePin::off();
    Commands::waitMS(startStopDelay);
}

template <class dirPin, class enabledPin, class activePin>
void ToolCNC<dirPin, enabledPin, activePin>::setRPM(float newRPM) {
    rpm = newRPM;
}

template <class dirPin, class enabledPin, class activePin>
void __attribute__((weak)) ToolCNC<dirPin, enabledPin, activePin>::menuRPM(GUIAction action, void* data) {
    ToolCNC<dirPin, enabledPin, activePin>* ext = reinterpret_cast<ToolCNC<dirPin, enabledPin, activePin>*>(data);
    DRAW_FLOAT_P(PSTR("Max. RPM:"), Com::tUnitRPM, ext->getRPM(), 0);
    if (GUI::handleFloatValueAction(action, v, 0, 20000, 10)) {
        ext->setRPM(v);
    }
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
