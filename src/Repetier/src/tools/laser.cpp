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

// ------------- Laser ------------------

template <class enabledPin, class activePin>
void __attribute__((weak)) ToolLaser<enabledPin, activePin>::menuToolLaserMilliWatt(GUIAction action, void* data) {
    ToolLaser<enabledPin, activePin>* ext = reinterpret_cast<ToolLaser<enabledPin, activePin>*>(data);
    DRAW_FLOAT_P(PSTR("Power:"), Com::tUnitMilliWatt, ext->getMilliWatt(), 0);
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 10)) {
        ext->setMilliWatt(v);
    }
}

template <class enabledPin, class activePin>
void __attribute__((weak)) ToolLaser<enabledPin, activePin>::menuToolLaserWarmupPower(GUIAction action, void* data) {
    ToolLaser<enabledPin, activePin>* ext = reinterpret_cast<ToolLaser<enabledPin, activePin>*>(data);
    DRAW_LONG_P(PSTR("Warmup PWM:"), Com::tUnitPWM, ext->getWarmupPower());
    if (GUI::handleLongValueAction(action, v, 0, 255, 1)) {
        ext->setWarmupPower(v);
    }
}

template <class enabledPin, class activePin>
void __attribute__((weak)) ToolLaser<enabledPin, activePin>::menuToolLaserWarmupTime(GUIAction action, void* data) {
    ToolLaser<enabledPin, activePin>* ext = reinterpret_cast<ToolLaser<enabledPin, activePin>*>(data);
    DRAW_LONG_P(PSTR("Warmup Time:"), Com::tUnitMilliSeconds, ext->getWarmupTime());
    if (GUI::handleLongValueAction(action, v, 0, 1000, 1)) {
        ext->setWarmupTime(v);
    }
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::init() {
    setEepromStart(EEPROM::reserve(EEPROM_SIGNATURE_LASER, 1, Tool::eepromSize() + 2 * 4 + 2));
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::setMilliWatt(float mw) {
    milliWatt = mw;
    scalePower = 255.0 / milliWatt;
}
template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::setWarmupPower(int16_t pwm) {
    warmupPower = pwm;
}
template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::setWarmupTime(int32_t time) {
    warmup = time;
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::eepromHandle() {
    Tool::eepromHandle();
    uint pos = getEepromStart() + Tool::eepromSize();
    EEPROM::handleFloat(pos, PSTR("Power [mW]"), 2, milliWatt);
    EEPROM::handleLong(pos + 4, PSTR("Warmup [us]"), warmup);
    EEPROM::handleInt(pos + 8, PSTR("Warmup Power [PWM]"), warmupPower);
    scalePower = 255.0 / milliWatt;
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::reset(float offx, float offy, float offz, float _milliwatt, int32_t _warmup, int16_t _warmupPower) {
    resetBase(offx, offy, offz);
    milliWatt = _milliwatt;
    warmup = _warmup;
    warmupPower = _warmupPower;
}

/// Called when the tool gets activated.
template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::activate() {
    active = false;
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();

    enabledPin::on();
    GCode::executeFString(startScript);
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}

/// Gets called when the tool gets disabled.
template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::deactivate() {
    active = false;
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();

    enabledPin::off();
    GCode::executeFString(endScript);
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::copySettingsToMotion1() {
}

/// Called on kill/emergency to disable the tool
template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::shutdown() {
    enabledPin::off();
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();
    secondary->set(0);
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::updateDerived() {
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::updateGammaMap(bool report) {
    gammaMap[0] = 0;
    int offset = bias * 255.0f / milliWatt;
    if (report) {
        Com::printFLN(PSTR("Gamma Offset:"), (int)offset);
    }
    for (fast8_t i = 1; i <= 255; i++) {
        int intensity = static_cast<int>(lroundf(powf(static_cast<float>(i) / 255.0f, gamma) * 255.0));
        gammaMap[i] = static_cast<uint8_t>(map(intensity, 0, 255, offset, 255)); // scale gamma function and offset to max
        if (report) {
            Com::printF(PSTR("gamma"), (int)i);
            Com::printFLN(PSTR("="), (int)gammaMap[i]);
        }
    }
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::extractNewGammaCorrection(GCode* com) {
    float b = bias;
    float c = gamma;
    if (com->hasB()) {
        b = com->B;
        if (b < 0) {
            b = 0.0f;
        } else if (b > milliWatt) {
            b = milliWatt;
        }
    }
    if (com->hasC()) {
        c = com->C;
        if (c <= 0) {
            c = 1.0;
        }
    }
    if (c != gamma || b != bias) {
        gamma = c;
        bias = b;
        Motion1::waitForEndOfMoves();
        updateGammaMap(com->hasR());
    }
}

template <class enabledPin, class activePin>
int ToolLaser<enabledPin, activePin>::computeIntensity(float v, bool activeSecondary, int intensity, float intensityPerMM) {
    if (!activeSecondary) {
        return 0;
    }
    if (intensity) {
        return gammaMap[intensity];
    }
    float target = v * intensityPerMM;
    if (target < 0) {
        target = 0;
    }
    if (target > 255) {
        target = 255;
    }
    return gammaMap[static_cast<int>(target)];
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::extractG1(GCode* com) {
    if (!active) {
        return;
    }
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
        activeSecondaryPerMMPS = scalePower * com->I;
    }
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::M3(GCode* com) {
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
        activeSecondaryPerMMPS = scalePower * com->I;
    } else {
        activeSecondaryValue = 255;
        activeSecondaryPerMMPS = 0;
    }
    if (com->hasE()) {
        powderMode = com->E != 0;
    }
    extractNewGammaCorrection(com);
    if (com->hasR()) {
        Com::printF(PSTR("Fixed PWM:"), activeSecondaryValue);
        Com::printFLN(PSTR(" variable PWM/v:"), activeSecondaryPerMMPS);
    }
    activePin::on();
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::M4(GCode* com) {
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
        activeSecondaryPerMMPS = scalePower * com->I;
    } else {
        activeSecondaryValue = 255;
        activeSecondaryPerMMPS = 0;
    }
    if (com->hasE()) {
        powderMode = com->E != 0;
    }
    extractNewGammaCorrection(com);
    if (com->hasR()) {
        Com::printF(PSTR("Fixed PWM:"), activeSecondaryValue);
        Com::printFLN(PSTR(" variable PWM/v:"), activeSecondaryPerMMPS);
    }
    activePin::on();
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::M5(GCode* com) {
    active = false;
    activeSecondaryValue = 0;
    activeSecondaryPerMMPS = 0;
    activePin::off();
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::secondarySwitched(bool nowSecondary) {
    if (nowSecondary && warmup > 0 && (activeSecondaryValue > 0 || activeSecondaryPerMMPS > 0)) {
        Motion1::WarmUp(warmup, warmupPower);
    }
}

template <class enabledPin, class activePin>
void ToolLaser<enabledPin, activePin>::moveFinished() {
    secondary->set(0); // Disable laser after each move for safety!
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
