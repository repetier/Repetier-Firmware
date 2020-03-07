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

#include "Repetier.h"

void CoolerManagerHeater::update() {
    float temp = RMath::max(heater->getTargetTemperature(), heater->getCurrentTemperature());
    if (temp < startTemp) {
        pwm->set(0);
        return;
    }
    if (temp > maxTemp) {
        pwm->set(maxPWM);
        return;
    }
    pwm->set(RMath::min(static_cast<fast8_t>(255), minPWM + static_cast<fast8_t>((temp - startTemp) * diff)));
}

void CoolerManagerSensor::update() {
    float temp = heater->get();
    if (temp < startTemp) {
        pwm->set(0);
        return;
    }
    if (temp > maxTemp) {
        pwm->set(maxPWM);
        return;
    }
    pwm->set(RMath::min(static_cast<fast8_t>(255), minPWM + static_cast<fast8_t>((temp - startTemp) * diff)));
}

void CoolerManagerMotors::update() {
    if (Printer::areAllSteppersDisabled()) {
        if (onCount >= 0) {
            onCount--;
            if (onCount == -1) {
                pwm->set(offPWM);
            }
        }
    } else { // at least one motor is running
        if (onCount < 0) {
            pwm->set(onPWM);
        }
        onCount = postCooling;
    }
}
