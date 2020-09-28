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
enum class HeaterError {
    NO_ERROR = 0,      // No error detected
    SENSOR_DEFECT = 1, // Sensor was reported defect
    NO_HEATUP = 2,     // Heating up does not change temperature
    LEAVING_RANGE = 3  // Loosing temperature
};

enum class DecoupleMode {
    NO_HEATING = 0,  // Heaters are off
    FAST_RISING = 1, // Full power until control range is reached
    SWING_IN = 2,    // Closing target temperature
    HOLDING = 3,     // Holding temperature
    COOLING = 4,     // Target was dropped but not off
    CALIBRATING = 5, // Signal that no updates should happen
    PAUSED = 6,
    UNPLUGGED = 7
};
enum class GUIAction;
// HeatManager instance pointer as data
extern void menuSetTemperature(GUIAction action, void* data);
extern void menuPreheatHeatManager(GUIAction action, void* data);
extern void menuSetPreheatTemperatureList(GUIAction action, void* data);
extern void menuSetPreheatTemperatureList(GUIAction action, void* data);

#define FLAG_HEATMANAGER_FREEZER 1                                     // Controls temperatures below room temperture. off is then <= -200°C
#define FLAG_HEATMANAGER_CONTROL_FULLTEMPERATURE_CONTROL_RANGE_RANGE 2 // If set the specialized manager also controls temperature outside TEMPERATURE_CONTROL_RANGE

class GCode;
class HeatManager {
protected:
    HeaterError error;
    float targetTemperature;
    float currentTemperature;
    float maxTemperature;
    IOTemperature* input;
    PWMHandler* output;
    uint8_t maxPWM;
    float decoupleVariance;
    millis_t decouplePeriod;
    DecoupleMode decoupleMode; // 0 = no heating, 1
    fast8_t errorCount;
    char heaterType;     // E = Extruder, B = bed, C = Chamber, O = Other
    fast8_t index;       // Type index for name reporting
    bool hotPluggable;   // If true will not panic when sensor is defect, will only disable this heater
    millis_t sampleTime; // Sample time for updates in ms

    millis_t lastDecoupleTest;  ///< Last time of decoupling sensor-heater test
    float lastDecoupleTemp;     ///< Temperature on last test
    millis_t preheatStartTime;  ///< Time (in milliseconds) when heat up was started
    int16_t preheatTemperature; ///< Temperture for preheat
    fast8_t wasOutsideRange;    // 1 = if was above range, 2 = was below range
    uint16_t eepromPos;         // Start position in eeprom
    millis_t lastUpdate;        // Time of last sampling
    fast8_t flags;
    float hysteresisTemperature; // Temperature corridor we need to be inside, 0 = off
    millis_t hysteresisTime;     // time in ms we need to be inside this corridor
    millis_t maxWait;            // longest time to wait for target temperature - causes error if exceeded, 0 = off

public:
    HeatManager(char htType, fast8_t _index, IOTemperature* i, PWMHandler* o, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod, bool _hotPluggable);
    void init();
    void initHysteresis(float _hysteresisTemperature, millis_t _hysteresisTime, millis_t _maxWait) {
        hysteresisTemperature = _hysteresisTemperature;
        hysteresisTime = _hysteresisTime;
        maxWait = _maxWait;
    }
    virtual void setTargetTemperature(float temp) {
        if (temp > maxTemperature) {
            Com::printWarningF(PSTR("Selected temp. was higher then max. temperaure. Max. Temp:"));
            Com::print(maxTemperature);
            Com::println();
            return;
        }
        if (temp <= ((flags & FLAG_HEATMANAGER_FREEZER) == 0 ? 0 : -200)) {
            decoupleMode = DecoupleMode::NO_HEATING;
        } else if (temp < currentTemperature) {
            decoupleMode = DecoupleMode::COOLING;
        } else {
            decoupleMode = DecoupleMode::FAST_RISING;
            lastDecoupleTest = HAL::timeInMilliseconds();
            lastDecoupleTemp = currentTemperature;
        }
        targetTemperature = temp;
    }
    inline bool isEnabled() {
        return decoupleMode != DecoupleMode::PAUSED && targetTemperature > MAX_ROOM_TEMPERATURE;
    }
    inline void pause() {
        if (decoupleMode != DecoupleMode::NO_HEATING) {
            decoupleMode = DecoupleMode::PAUSED;
            output->set(0);
        }
    }
    inline void unpause() {
        setTargetTemperature(targetTemperature);
    }
    inline bool isUnplugged() { return decoupleMode == DecoupleMode::UNPLUGGED; }
    inline bool isPaused() { return decoupleMode == DecoupleMode::PAUSED; }
    inline float getTargetTemperature() { return targetTemperature; }
    inline void setCurrentTemperature(float temp) {
        currentTemperature = temp;
    }
    float getStatefulTemperature(); ///< Returns temp or -333 on defect, -444 on decoupled
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
    inline uint8_t getMaxPWM() { return maxPWM; }
    inline void setMaxPWM(uint8_t m) { maxPWM = m; }
    virtual float getP() { return 0; }
    virtual float getI() { return 0; }
    virtual float getD() { return 0; }
    virtual void setPID(float p, float i, float d) { }
    inline millis_t getSampleTime() {
        return sampleTime;
    }
    inline float getDecoupleVariance() { return decoupleVariance; }
    inline void setDecoupleVariance(float val) { decoupleVariance = val; }
    inline millis_t getDecouplePeriod() { return decouplePeriod; }
    inline void setDecouplePeriod(millis_t val) { decouplePeriod = val; }
    inline float getHysteresisTemperature() { return hysteresisTemperature; }
    inline void setHysteresisTemperature(float val) { hysteresisTemperature = val >= 0 ? val : 0; }
    inline millis_t getHysteresisTime() { return hysteresisTime; }
    inline void setHysteresisTime(millis_t val) { hysteresisTime = val >= 0 ? val : 0; }
    inline millis_t getMaxWait() { return maxWait; }
    inline void setMaxWait(millis_t val) { maxWait = val >= 0 ? val : 0; }
    virtual void updateLocal(float tempError) = 0;
    void eepromHandle();
    virtual void eepromHandleLocal(int pos) = 0;
    virtual int eepromSizeLocal() { return 0; };
    void update();
    virtual void updateDerived() { }
    /** Waits until the set target temperature is reached */
    void waitForTargetTemperature();
    inline float getMaxTemperature() { return maxTemperature; }
    inline void setMaxTemperature(float val) { maxTemperature = val; }
    void reportTemperature(char c, int idx);
    virtual void autocalibrate(GCode* g) {
        Com::printWarningFLN(PSTR("Autocalibration for this tool not supported!"));
    }
    virtual void showControlMenu(GUIAction action); // Default set temperature
    void showBaseConfigMenu(GUIAction action);      // config menu of base class
    virtual void showConfigMenu(GUIAction action) { showBaseConfigMenu(action); }
    virtual bool hasConfigMenu();
    bool isExtruderHeater() const { return heaterType == 'E'; }
    bool isBedHeater() const { return heaterType == 'B'; }
    bool isChamberHeater() const { return heaterType == 'C'; }
    bool isOtherHeater() const { return heaterType == 'O'; }
    void printName();
    fast8_t getIndex() const { return index; }
    virtual int getMinTemperature() const { return 0; }
    bool isOff() const { return targetTemperature <= ((flags & FLAG_HEATMANAGER_FREEZER) == 0 ? MAX_ROOM_TEMPERATURE : getMinTemperature()); }
    static bool reportTempsensorError();
    static void disableAllHeaters();
    static void resetAllErrorStates();
};

class HeatManagerBangBang : public HeatManager {

public:
    HeatManagerBangBang(char htType, fast8_t _index, IOTemperature* _input, PWMHandler* output, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod, bool _hotPluggable)
        : HeatManager(htType, _index, _input,
                      output, maxTemp, maxPwm, _sampleTime, decVariance, decPeriod, _hotPluggable) {
    }
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
    virtual void autocalibrate(GCode* g) {
        Com::printInfoFLN(PSTR("No parameter needed. Nothing to calibrate!"));
    }
    void eepromHandleLocal(int adr);
    int eepromSizeLocal();
};

class HeatManagerPID : public HeatManager {
protected:
    float P, I, D;
    float IState;
    float driveMin, driveMax;
    float lastTemperature;
    float ki, kd;

public:
    HeatManagerPID(char htType, fast8_t _index, IOTemperature* _input, PWMHandler* output, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod,
                   float p, float i, float d, float _driveMin, float _driveMax, bool _hotPluggable)
        : HeatManager(htType, _index, _input,
                      output, maxTemp, maxPwm, _sampleTime, decVariance, decPeriod, _hotPluggable)
        , P(p)
        , I(i)
        , D(d)
        , driveMin(_driveMin)
        , driveMax(_driveMax) {
        IState = 0;
        lastTemperature = 20;
        updateDerived();
    }
    void updateLocal(float tempError);
    void updateDerived();
    void resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod,
                         float p, float i, float d, float _driveMin, float _driveMax);
    void eepromHandleLocal(int adr);
    int eepromSizeLocal();
    void autocalibrate(GCode* g);
    float getP() { return P; }
    float getI() { return I; }
    float getD() { return D; }
    void setPID(float p, float i, float d);
    float getDriveMin() { return driveMin; }
    float getDriveMax() { return driveMax; }
    void setDriveMin(float dm) {
        driveMin = dm;
        updateDerived();
    }
    void setDriveMax(float dm) {
        driveMax = dm;
        updateDerived();
    }
    // void showControlMenu(GUIAction action);
    void showConfigMenu(GUIAction action);
    virtual bool hasConfigMenu();
};

// peltierType 0 = only cooling, 1 = only heating, 2 = switch for direction
template <class flowPin, int peltierType, int minTemp>
class HeatManagerPeltierPID : public HeatManagerPID {

public:
    HeatManagerPeltierPID(char htType, fast8_t _index, IOTemperature* _input, PWMHandler* output, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod,
                          float p, float i, float d, float _driveMin, float _driveMax, bool _hotPluggable)
        : HeatManagerPID(htType, _index, _input,
                         output, maxTemp, maxPwm, _sampleTime, decVariance, decPeriod, p, i, d, _driveMin, _driveMax, _hotPluggable) {
        flags = FLAG_HEATMANAGER_FREEZER + FLAG_HEATMANAGER_CONTROL_FULLTEMPERATURE_CONTROL_RANGE_RANGE;
        targetTemperature = minTemp; // different off value!
        updateDerived();
    }
    virtual int getMinTemperature() const { return minTemp; }
    void updateLocal(float tempError);
    void autocalibrate(GCode* g);
};

/**
 Dynamic Dead Time 

 The normal dead time assumes same time for heating and cooling. Reality is that these
 differ. They even differ over temperatures, especially cooling at high temperatures
 is much faster then for lower temperatures. So what we really need are 2 sets - one for
 the high temperatures and one for the lower ones, then we can interpolate timings.

 deadUp is time to stop heating until the curve slows down.
 deadDown is time to start heating until you will see a raise.
*/
class HeatManagerDynDeadTime : public HeatManager {
    float temp1, deadUp1, deadDown1;
    float temp2, deadUp2, deadDown2;
    float lastTemperatures[4];
    fast8_t counter;
    float deadUp, deadDown;

    bool detectTimings(float temp, float& up, float& down, float reduce);

public:
    HeatManagerDynDeadTime(char htType, fast8_t _index, IOTemperature* input, PWMHandler* output, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod,
                           float _temp1, float _deadUp1, float _deadDown1, float _temp2, float _deadUp2, float _deadDown2, bool _hotPluggable)
        : HeatManager(htType, _index, input,
                      output, maxTemp, maxPwm, _sampleTime, decVariance, decPeriod, _hotPluggable)
        , temp1(_temp1)
        , deadUp1(_deadUp1)
        , deadDown1(_deadDown1)
        , temp2(_temp2)
        , deadUp2(_deadUp2)
        , deadDown2(_deadDown2) {
        lastTemperatures[0] = lastTemperatures[1] = lastTemperatures[2] = lastTemperatures[3] = 20;
        counter = 0;
        updateTimings();
    }
    void updateLocal(float tempError);
    void resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod,
                         float _temp1, float _deadUp1, float _deadDown1, float _temp2, float _deadUp2, float _deadDown2);
    void eepromHandleLocal(int adr);
    int eepromSizeLocal();
    void autocalibrate(GCode* g);
    void setTargetTemperature(float temp);
    void updateTimings();
    float getTemp1() { return temp1; }
    void setTemp1(float val) { temp1 = val; }
    float getDeadUp1() { return deadUp1; }
    void setDeadUp1(float val) { deadUp1 = val; }
    float getDeadDown1() { return deadDown1; }
    void setDeadDown1(float val) { deadDown1 = val; }
    float getTemp2() { return temp2; }
    void setTemp2(float val) { temp2 = val; }
    float getDeadUp2() { return deadUp2; }
    void setDeadUp2(float val) { deadUp2 = val; }
    float getDeadDown2() { return deadDown2; }
    void setDeadDown2(float val) { deadDown2 = val; }
    // void showControlMenu(GUIAction action);
    void showConfigMenu(GUIAction action);
    virtual bool hasConfigMenu() { return true; }
};

extern HeatManager* heaters[];
