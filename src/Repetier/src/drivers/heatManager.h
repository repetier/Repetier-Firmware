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
enum HeaterError {
    NO_ERROR = 0,      // No error detected
    SENSOR_DEFECT = 1, // Sensor was reported defect
    NO_HEATUP = 2,     // Heating up does not change temperature
    LEAVING_RANGE = 3  // Loosing temperature
};

enum DecoupleMode {
    NO_HEATING = 0,  // Heaters are off
    FAST_RISING = 1, // Full power until control range is reached
    SWING_IN = 2,    // Closing target temperature
    HOLDING = 3,     // Holding temperature
    COOLING = 4      // Target was dropped but not off
};

class HeatManager {
protected:
    HeaterError error;
    float targetTemperature;
    float currentTemperature;
    IOTemperature* input;
    PWMHandler* output;
    fast8_t maxPWM;
    float decoupleVariance;
    millis_t decouplePeriod;

    millis_t lastDecoupleTest; ///< Last time of decoupling sensor-heater test
    float lastDecoupleTemp;    ///< Temperature on last test
    millis_t preheatStartTime; ///< Time (in milliseconds) when heat up was started
    int16_t preheatTemperature;
    DecoupleMode decoupleMode; // 0 = no heating, 1
    fast8_t errorCount;
    fast8_t wasOutsideRange; // 1 = if was above range, 2 = was below range
public:
    HeatManager(IOTemperature* i, PWMHandler* o, fast8_t maxPwm, float decVariance, millis_t decPeriod)
        : error(HeaterError::NO_ERROR)
        , targetTemperature(0)
        , currentTemperature(0)
        , input(i)
        , output(o)
        , maxPWM(maxPwm)
        , decoupleVariance(decVariance)
        , decouplePeriod(decPeriod)
        , decoupleMode(DecoupleMode::NO_HEATING)
        , errorCount(0) {}
    virtual void setTargetTemperature(float temp) {
        if (temp < currentTemperature) {
            decoupleMode = DecoupleMode::COOLING;
        } else {
            decoupleMode = DecoupleMode::FAST_RISING;
            lastDecoupleTest = HAL::timeInMilliseconds();
            lastDecoupleTemp = currentTemperature;
        }
        targetTemperature = temp;
    }
    inline float getTargetTemperature() { return targetTemperature; }
    inline void setCurrentTemperature(float temp) {
        currentTemperature = temp;
    }
    inline float getCurrentTemperature() { return currentTemperature; }
    inline float getPreheatTemperature() { return preheatTemperature; }
    inline HeaterError getError() { return error; }
    inline void resetError() { error = HeaterError::NO_ERROR; }
    inline void setError(HeaterError err) {
        error = err;
        Com::printFLN(PSTR("setError:"), (int)err);
        if (err != HeaterError::NO_ERROR) {
            output->set(0);
            decoupleMode = DecoupleMode::NO_HEATING;
        }
    }
    virtual void updateLocal(float tempError) = 0;
    int eepromSize() {
        return eepromSizeLocal() + 9;
    }
    void eepromHandle(int adr) {

        eepromHandleLocal(adr + eepromSize());
    }
    virtual void eepromHandleLocal(int adr) = 0;
    virtual void eepromResetLocal() = 0;
    virtual int eepromSizeLocal() = 0;
    void update();
    virtual void updateDerived() {}
    /** Waits until the set target temperature is reached */
    void waitForTargetTemperature();
    void reportTemperature(char c, int idx);
};

class HeatManagerBangBang : public HeatManager {

public:
    void updateLocal(float tempError) {
        output->set(currentTemperature > targetTemperature ? 0 : maxPWM);
    }
    int eepromSize() {
        return 1;
    }
    void resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod) {
        maxPWM = _maxPwm;
        decoupleVariance = decVariance;
        decouplePeriod = decPeriod;
    }
    void eepromHandleLocal(int adr);
    void eepromResetLocal();
    int eepromSizeLocal();
};

class HeatManagerPID : public HeatManager {
    float P, I, D;
    float IMax, IMin;
    float IState;
    float driveMin, driveMax;
    float lastTemperature;
    float actTemperature;
    fast8_t counter;
public:
    HeatManagerPID(IOTemperature* input, PWMHandler* output, fast8_t maxPwm, float decVariance, millis_t decPeriod,
                   float p, float i, float d, float _driveMin, float _driveMax)
        : HeatManager(input,
                      output, maxPwm, decVariance, decPeriod)
        , P(p)
        , I(i)
        , D(d)
        , driveMin(_driveMin)
        , driveMax(_driveMax) {
        IState = 0;
        actTemperature = lastTemperature = 20;
        counter = 0;
        updateDerived();
    }
    void updateLocal(float tempError);
    void updateDerived();
    void resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod,
                         float p, float i, float d, float _driveMin, float _driveMax);
    void eepromHandleLocal(int adr);
    void eepromResetLocal();
    int eepromSizeLocal();
};
