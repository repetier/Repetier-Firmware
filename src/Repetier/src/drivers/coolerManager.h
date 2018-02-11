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

/** This class controls a can fan depending on measured heater temperature.
 * Use it to cool an extruder. 
 * It will enable when start temperature is reached and control fan speed
 * to be at maxPWM if maxTemp is reached.
 * */
class CoolerManagerHeater {
    HeatManager* heater;
    PWMHandler* pwm;
    float startTemp;
    float maxTemp;
    fast8_t minPWM;
    fast8_t maxPWM;
    float diff;

public:
    CoolerManagerHeater(HeatManager* _heater,
                        PWMHandler* _pwm,
                        float _startTemp,
                        float _maxTemp,
                        fast8_t _minPWM,
                        fast8_t _maxPWM)
        : heater(_heater)
        , pwm(_pwm)
        , startTemp(_startTemp)
        , maxTemp(_maxTemp)
        , minPWM(_minPWM)
        , maxPWM(_maxPWM) {
        diff = static_cast<float>(maxPWM - minPWM) / (maxTemp - startTemp);
    }
    void update() {
        float temp = heater->getTargetTemperature();
        if (temp < startTemp) {
            pwm->set(0);
            return;
        }
        if (temp > maxPWM) {
            pwm->set(maxPWM);
            return;
        }
        pwm->set(RMath::min(static_cast<fast8_t>(255), minPWM + static_cast<fast8_t>((temp - startTemp) * diff)));
    }
};

/** This manager works on any temperature sensor and controls a fan.
 * Use it to cool a board or whatever you want.
 */
class CoolerManagerSensor {
    IOTemperature* heater;
    PWMHandler* pwm;
    float startTemp;
    float maxTemp;
    fast8_t minPWM;
    fast8_t maxPWM;
    float diff;

public:
    CoolerManagerSensor(IOTemperature* _heater,
                        PWMHandler* _pwm,
                        float _startTemp,
                        float _maxTemp,
                        fast8_t _minPWM,
                        fast8_t _maxPWM)
        : heater(_heater)
        , pwm(_pwm)
        , startTemp(_startTemp)
        , maxTemp(_maxTemp)
        , minPWM(_minPWM)
        , maxPWM(_maxPWM) {
        diff = static_cast<float>(maxPWM - minPWM) / (maxTemp - startTemp);
    }
    void update() {
        float temp = heater->get();
        if (temp < startTemp) {
            pwm->set(0);
            return;
        }
        if (temp > maxPWM) {
            pwm->set(maxPWM);
            return;
        }
        pwm->set(RMath::min(static_cast<fast8_t>(255), minPWM + static_cast<fast8_t>((temp - startTemp) * diff)));
    }
};
