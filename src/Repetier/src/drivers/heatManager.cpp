#include "Repetier.h"

HeatManager* heaters[] = HEATERS;
constexpr int numHeaters = std::extent<decltype(heaters)>::value;
static_assert(numHeaters == NUM_HEATERS, "NUM_HEATERS not defined correctly");

// HeatManager instance pointer as data
void menuSetTemperature(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    float value = hm->getTargetTemperature();
    DRAW_FLOAT_P(PSTR("Target Temperature:"), Com::tUnitDegCelsius, value, 0);
    if (GUI::handleFloatValueAction(action, value, hm->getMinTemperature(), hm->getMaxTemperature(), (ENCODER_SPEED == 2) ? 5 : 1)) {
        hm->setTargetTemperature(value);
    }
#endif
}

void menuDisableTemperature(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    hm->setTargetTemperature(hm->getMinTemperature());
    // Now remove the Disable button & move the display up if needed.
    GUI::cursorRow[GUI::level]--;
    if (GUI::topRow[GUI::level]) {
        GUI::topRow[GUI::level]--;
    }
#endif
}

void menuHMMaxPWM(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    float value = hm->getMaxPWM();
    DRAW_FLOAT_P(PSTR("Max. PWM:"), Com::tUnitPWM, value, 0);
    if (GUI::handleFloatValueAction(action, value, 1, 255, 1)) {
        hm->setMaxPWM(static_cast<uint8_t>(value));
    }
#endif
}

void menuHMSetMaxTemperature(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    float value = hm->getMaxTemperature();
    DRAW_FLOAT_P(PSTR("Max. Temperature:"), Com::tUnitDegCelsius, value, 0);
    if (GUI::handleFloatValueAction(action, value, 0, 1000, 1)) {
        hm->setMaxTemperature(value);
    }
#endif
}

void menuHMDecoupleVariance(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    float value = hm->getDecoupleVariance();
    DRAW_FLOAT_P(PSTR("Dec. variance:"), Com::tUnitDegCelsius, value, 0);
    if (GUI::handleFloatValueAction(action, value, 0, 1000, 1)) {
        hm->setDecoupleVariance(value);
    }
#endif
}

void menuHMDecouplePeriod(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    int32_t value = hm->getDecouplePeriod();
    DRAW_LONG_P(PSTR("Dec. max. period:"), Com::tUnitMilliSeconds, value);
    if (GUI::handleLongValueAction(action, value, 0, 600000, 1000)) {
        hm->setDecouplePeriod(value);
    }
#endif
}

void menuHMMaxWait(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    int32_t value = hm->getMaxWait();
    DRAW_LONG_P(PSTR("Max. wait f. target:"), Com::tUnitMilliSeconds, value);
    if (GUI::handleLongValueAction(action, value, 0, 3600000, 1000)) {
        hm->setMaxWait(value);
    }
#endif
}

void menuHMSetHysteresisTemperature(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    float value = hm->getHysteresisTemperature();
    DRAW_FLOAT_P(PSTR("Hyst. Temperature:"), Com::tUnitDegCelsius, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 20, 0.1f)) {
        hm->setHysteresisTemperature(value);
    }
#endif
}

void menuHMSetHysteresisTime(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    int32_t value = hm->getHysteresisTime();
    DRAW_LONG_P(PSTR("Hysteresis:"), Com::tUnitMilliSeconds, value);
    if (GUI::handleLongValueAction(action, value, 0, 3600000, 1000)) {
        hm->setHysteresisTime(value);
    }
#endif
}

HeatManager::HeatManager(char htType, fast8_t _index, IOTemperature* i, PWMHandler* o, float maxTemp, fast8_t maxPwm, millis_t _sampleTime, float decVariance, millis_t decPeriod, bool _hotPluggable)
    : error(HeaterError::NO_ERROR)
    , targetTemperature(0)
    , currentTemperature(0)
    , maxTemperature(maxTemp)
    , input(i)
    , output(o)
    , maxPWM(maxPwm)
    , decoupleVariance(decVariance)
    , decouplePeriod(decPeriod)
    , decoupleMode(DecoupleMode::NO_HEATING)
    , errorCount(0)
    , heaterType(htType)
    , index(_index)
    , hotPluggable(_hotPluggable)
    , sampleTime(_sampleTime)
    , flags(0)
    , hysteresisTemperature(0)
    , hysteresisTime(0)
    , maxWait(0) {
    lastUpdate = 0;
}

void HeatManager::init() {
    eepromPos = EEPROM::reserve(EEPROM_SIGNATURE_HEAT_MANAGER, 1, eepromSizeLocal() + 25);
}

float HeatManager::getStatefulTemperature() { ///< Returns temp or -333 on defect, -444 on decoupled
    if (error == HeaterError::SENSOR_DEFECT) {
        return -333.0;
    }
    if (error != HeaterError::NO_ERROR) {
        return -444.0;
    }
    return currentTemperature;
}

void HeatManager::update() {
    millis_t time = HAL::timeInMilliseconds();
    if (time - lastUpdate < sampleTime) {
        return;
    }
    lastUpdate = time;
    if (error != HeaterError::NO_ERROR) {
        if (input->isDefect()) {
            return; // do nothing in error state
        } else {
            error = HeaterError::NO_ERROR;
        }
    }
    if (errorCount > 0) {
        errorCount--;
    }
    if (decoupleMode == DecoupleMode::CALIBRATING) {
        setCurrentTemperature(input->get());
        return; // do not interfere with calibration
    }
    if (decoupleMode == DecoupleMode::PAUSED) {
        setCurrentTemperature(input->get());
        return; // do nothing in pause mode
    }
    if (decoupleMode == DecoupleMode::UNPLUGGED) {
        if (input->isDefect()) {
            return;
        }
        decoupleMode = DecoupleMode::NO_HEATING;
        Com::printF(PSTR("Heater "));
        printName();
        Com::printFLN(PSTR(" plugged in."));
    }
    if (input->isDefect()) {
        if (hotPluggable) {
            setCurrentTemperature(0);
            // setTargetTemperature(0); // unplugged mode will prevent errors
            output->set(0);
            decoupleMode = DecoupleMode::UNPLUGGED;
            Com::printF(PSTR("Heater "));
            printName();
            Com::printFLN(PSTR(" unplugged."));

            return;
        } else {
            errorCount += 2;
            if (errorCount > 10) {
                Com::printErrorF(PSTR("Heater "));
                printName();
                Com::printF(PSTR(" may be defective. Sensor reported unusual values."));
                Com::printFLN(PSTR(" This could mean a broken wire or a shorted contact on the sensor."));
                setError(HeaterError::SENSOR_DEFECT);
                GCode::fatalError(PSTR("Heater sensor defect"));
            }
        }
        return;
    } else {
        setCurrentTemperature(input->get());
    }
    if (targetTemperature <= ((flags & FLAG_HEATMANAGER_FREEZER) == 0 ? MAX_ROOM_TEMPERATURE : getMinTemperature())) { // heater disabled
        output->set(0);
        decoupleMode = DecoupleMode::NO_HEATING;
        return;
    }
    // Test for decoupled HeaterError

    float tempError = targetTemperature - currentTemperature;
    if (decouplePeriod > 0 && time - lastDecoupleTest > decouplePeriod) {
        // we should do a test
        if (decoupleMode == DecoupleMode::FAST_RISING) {
            if (currentTemperature - 1 < lastDecoupleTemp) {
                // we waited and nothing happened, that is not ok
                Com::printErrorF(PSTR("Heater "));
                printName();
                Com::printFLN(PSTR(" failed to rise under full power. Heater disabled."));
                Com::printErrorFLN(PSTR("If this wasn't due to a hardware defect, the decoupling period might be set too low."));
                Com::printErrorF(PSTR("No temperature rise after (ms):"));
                Com::print(decouplePeriod);
                Com::println();
                setError(HeaterError::NO_HEATUP);
                GCode::fatalError(PSTR("Heater decoupled during rising"));
                return;
            }
            lastDecoupleTemp = currentTemperature;
        } else if (decoupleMode == DecoupleMode::HOLDING) {
            if (fabs(tempError) > decoupleVariance) {
                // Temperature left target range
                Com::printErrorF(PSTR("The temperature for heater "));
                printName();
                Com::printFLN(PSTR(" deviated too far from the target temperature. Heater disabled."));
                Com::printErrorFLN(PSTR("This can happen on a hardware defect or if the decouple temperature variance is set too low."));
                Com::printErrorF(PSTR("Deviation:"));
                Com::printFloat(tempError, 2);
                Com::println();
                setError(HeaterError::LEAVING_RANGE);
                GCode::fatalError(PSTR("Heater decoupled during hold"));
                return;
            }
        }
        lastDecoupleTest = time;
    }
    if ((decoupleMode == DecoupleMode::FAST_RISING || decoupleMode == DecoupleMode::COOLING) && fabs(tempError) < decoupleVariance) {
        lastDecoupleTest = time;
        decoupleMode = DecoupleMode::HOLDING;
    }

    // Control heater

    if ((flags & FLAG_HEATMANAGER_CONTROL_FULLTEMPERATURE_CONTROL_RANGE_RANGE) == 0) {
        if (tempError > TEMPERATURE_CONTROL_RANGE) { // too cold
            output->set(maxPWM);
            wasOutsideRange = 2;
            if (decoupleMode == DecoupleMode::FAST_RISING) {

            } else {
                decoupleMode = DecoupleMode::FAST_RISING;
                lastDecoupleTest = HAL::timeInMilliseconds();
                lastDecoupleTemp = currentTemperature;
            }
            return;
        }
        if (tempError < -TEMPERATURE_CONTROL_RANGE) { // too hot
            output->set(0);
            decoupleMode = DecoupleMode::COOLING;
            wasOutsideRange = 1;
            return;
        }
    }
    updateLocal(tempError); // In control range
}

void HeatManager::eepromHandle() {
#ifndef PREVENT_CHANGE_MAX_TEMPERATURE
    EEPROM::handleFloat(eepromPos + 0, PSTR("Max. Temperature [deg C]"), 2, maxTemperature);
#endif
#ifndef PREVENT_CHANGE_MAX_PWM
    EEPROM::handleByte(eepromPos + 4, PSTR("Max. PWM [0-255]"), maxPWM);
#endif
    EEPROM::handleLong(eepromPos + 5, PSTR("Decouple test period [ms]"), decouplePeriod);
    EEPROM::handleFloat(eepromPos + 9, PSTR("Decouple variance [K]"), 2, decoupleVariance);
    EEPROM::handleFloat(eepromPos + 13, PSTR("Hysteresis Temp. [°C, 0=off]"), 2, hysteresisTemperature);
    EEPROM::handleLong(eepromPos + 17, PSTR("Hysteresis time [ms]"), hysteresisTime);
    EEPROM::handleLong(eepromPos + 21, PSTR("Max. heatup time [ms,0=off]"), maxWait);
    eepromHandleLocal(eepromPos + 25);
}

void HeatManager::disableAllHeaters() {
    for (fast8_t i = 0; i < NUM_HEATERS; i++) {
        heaters[i]->setTargetTemperature(heaters[i]->getMinTemperature());
    }
}

void HeatManager::waitForTargetTemperature() {
    if (isOff() || Printer::debugDryrun()) {
        return;
    }
    bool oldReport = Printer::isAutoreportTemp();
    Printer::setAutoreportTemp(true);
    millis_t startTime = HAL::timeInMilliseconds();
    millis_t waitUntil = 0;
    while (!Printer::breakLongCommand) {
        Commands::checkForPeriodicalActions(true);
        GCode::keepAlive(FirmwareState::WaitHeater);
        millis_t time = HAL::timeInMilliseconds();
        float absDiff = fabs(targetTemperature - currentTemperature);
        if (hysteresisTemperature > 0) { // User wants temp. within a corridor
            if (waitUntil == 0) {
                if (absDiff <= hysteresisTemperature) {
                    waitUntil = time + hysteresisTime;
                }
            } else {
                if (absDiff > hysteresisTemperature) {
                    waitUntil = 0;
                } else if (waitUntil - time > 2000000000UL) {
                    break;
                }
            }
        } else { // Just break if we are within one degree of target temperature
            if (absDiff <= 1.0f) {
                break;
            }
        }
        if (maxWait && startTime + maxWait - currentTemperature > 2000000000UL) {
            GCode::fatalError(PSTR("Target temp. not reached within set time limit"));
            break;
        }
    }
    Printer::setAutoreportTemp(oldReport);
}

bool HeatManager::hasConfigMenu() {
    return true;
}

void HeatManager::showBaseConfigMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
#ifndef PREVENT_CHANGE_MAX_TEMPERATURE
    GUI::menuFloatP(action, PSTR("Max. Temp:"), maxTemperature, 0, menuHMSetMaxTemperature, this, GUIPageType::FIXED_CONTENT);
#endif
#ifndef PREVENT_CHANGE_MAX_PWM
    GUI::menuLongP(action, PSTR("Max. PWM:"), maxPWM, menuHMMaxPWM, this, GUIPageType::FIXED_CONTENT);
#endif
    GUI::menuFloatP(action, PSTR("Decouple var.:"), decoupleVariance, 0, menuHMDecoupleVariance, this, GUIPageType::FIXED_CONTENT);
    GUI::menuLongP(action, PSTR("Decouple per.:"), decouplePeriod, menuHMDecouplePeriod, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Hyster. temp.:"), hysteresisTemperature, 1, menuHMSetHysteresisTemperature, this, GUIPageType::FIXED_CONTENT);
    GUI::menuLongP(action, PSTR("Hyster. time:"), hysteresisTime, menuHMSetHysteresisTime, this, GUIPageType::FIXED_CONTENT);
    GUI::menuLongP(action, PSTR("Max. wait:"), maxWait, menuHMMaxWait, this, GUIPageType::FIXED_CONTENT);
#endif
}

void HeatManager::reportTemperature(char c, int idx) {
    Com::print(c);
    if (idx >= 0) {
        Com::print(idx);
    }
    Com::print(':');
    Com::printFloat(getStatefulTemperature(), 1);
    Com::printF(Com::tSpaceSlash, targetTemperature, 0);
    Com::print(' ');
    if (c == 'B') {
        Com::print('B');
    }
    Com::print('@');
    if (idx > 0 || (idx == 0 && c == 'B')) {
        Com::print(idx);
    }
    Com::print(':');
    Com::print((int)(output->get()));
    Com::print(' ');
    // TODO: Report error state
}

void HeatManager::resetAllErrorStates() {
    for (uint8_t i = 0; i < NUM_HEATERS; i++) {
        heaters[i]->resetError();
    }
}

bool HeatManager::reportTempsensorError() {
#if NUM_HEATERS > 0
    if (!Printer::isAnyTempsensorDefect())
        return false;
    for (uint8_t i = 0; i < NUM_HEATERS; i++) {
        HeatManager* h = heaters[i];
        if (h->isBedHeater()) {
            Com::printF(Com::tHeatedBed);
        } else if (h->isExtruderHeater()) {
            Com::printF(Com::tExtruderSpace, i);
        } else if (h->isChamberHeater()) {
            Com::printF(PSTR("Heated Chamber "), i);
        } else {
            Com::printF(PSTR("Other:"));
        }
        HeaterError err = h->getError();
        if (err == HeaterError::SENSOR_DEFECT)
            Com::printF(Com::tTempSensorDefect);
        else
            Com::printF(Com::tTempSensorWorking);
        if (err == HeaterError::NO_HEATUP)
            Com::printF(PSTR(" temperature does not rise"));
        if (err == HeaterError::LEAVING_RANGE)
            Com::printF(PSTR(" temperature left control range"));
        Com::println();
    }
    Com::printErrorFLN(Com::tDryModeUntilRestart);
    return true;
#else
    return false;
#endif
}

void HeatManager::showControlMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    if (isOff()) {
        GUI::menuSelectableP(action, PSTR("Set Temp: Off"), menuSetTemperature, this, GUIPageType::FIXED_CONTENT);
    } else {
        GUI::flashToStringLong(GUI::tmpString, PSTR("Set Temp: @°C"), static_cast<int32_t>(lroundf(targetTemperature)));
        GUI::menuSelectable(action, GUI::tmpString, menuSetTemperature, this, GUIPageType::FIXED_CONTENT);
        GUI::menuSelectableP(action, PSTR("Disable"), menuDisableTemperature, this, GUIPageType::ACTION);
    }
#endif
}

void HeatManager::printName() {
    Com::print(heaterType);
    Com::print(static_cast<int32_t>(index));
}

void HeatManagerBangBang::eepromHandleLocal(int adr) {
}

int HeatManagerBangBang::eepromSizeLocal() {
    return 0;
}

void HeatManagerPID::updateLocal(float tempError) {
    if (wasOutsideRange) { // came back from uncontrolled area
        lastTemperature = currentTemperature;
        IState = output->get();
        wasOutsideRange = 0;
    }
    float pidTerm = P * tempError;
    IState += ki * tempError;
    IState = constrain(IState, driveMin, driveMax);
    pidTerm += IState;
    pidTerm += kd * (lastTemperature - currentTemperature);
    output->set(constrain((int)pidTerm, 0, maxPWM));
    lastTemperature = currentTemperature;
}
void HeatManagerPID::updateDerived() {
    float timeSeconds = getSampleTime() * 0.001;
    ki = I * timeSeconds;
    kd = D / timeSeconds;
}

void HeatManagerPID::resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod,
                                     float p, float i, float d, float _driveMin, float _driveMax) {
    maxPWM = _maxPwm;
    decoupleVariance = decVariance;
    decouplePeriod = decPeriod;
    P = p;
    I = i;
    D = d;
    driveMin = _driveMin;
    driveMax = _driveMax;
    updateDerived();
}

void HeatManagerPID::eepromHandleLocal(int pos) {
    EEPROM::handleFloat(pos, PSTR("P [-]"), 2, P);
    EEPROM::handleFloat(pos + 4, PSTR("I [-]"), 2, I);
    EEPROM::handleFloat(pos + 8, PSTR("D [-]"), 2, D);
    EEPROM::handleFloat(pos + 12, PSTR("Min. I Part [-]"), 2, driveMin);
    EEPROM::handleFloat(pos + 16, PSTR("Max. I Part [-]"), 2, driveMax);
    updateDerived();
}

int HeatManagerPID::eepromSizeLocal() {
    return 5 * 4;
}

void HeatManagerPID::setPID(float p, float i, float d) {
    P = p;
    I = i;
    D = d;
    updateDerived();
}

/* void HeatManagerPID::showControlMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    GUI::flashToStringLong(GUI::tmpString, PSTR("Set Temp: @°C"), static_cast<int32_t>(lroundf(targetTemperature)));
    GUI::menuSelectable(action, GUI::tmpString, menuSetTemperature, this, GUIPageType::FIXED_CONTENT);
#endif
} */

bool HeatManagerPID::hasConfigMenu() {
    return true;
}

void menuSetPIDP(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerPID* hm = reinterpret_cast<HeatManagerPID*>(data);
    float value = hm->getP();
    DRAW_FLOAT_P(PSTR("Proportial P:"), Com::tEmpty, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 500, 0.1)) {
        hm->setPID(value, hm->getI(), hm->getD());
    }
#endif
}

void menuSetPIDI(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerPID* hm = reinterpret_cast<HeatManagerPID*>(data);
    float value = hm->getI();
    DRAW_FLOAT_P(PSTR("Integral I:"), Com::tEmpty, value, 2);
    if (GUI::handleFloatValueAction(action, value, 0, 500, 0.02)) {
        hm->setPID(hm->getP(), value, hm->getD());
    }
#endif
}

void menuSetPIDD(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerPID* hm = reinterpret_cast<HeatManagerPID*>(data);
    float value = hm->getD();
    DRAW_FLOAT_P(PSTR("Damping D:"), Com::tEmpty, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 500, 0.1)) {
        hm->setPID(hm->getP(), hm->getI(), value);
    }
#endif
}

void menuSetDriveMin(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerPID* hm = reinterpret_cast<HeatManagerPID*>(data);
    float value = hm->getDriveMin();
    DRAW_FLOAT_P(PSTR("Min. I-Part:"), Com::tEmpty, value, 1);
    if (GUI::handleFloatValueAction(action, value, -500, 500, 0.1)) {
        hm->setDriveMin(value);
    }
#endif
}

void menuSetDriveMax(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerPID* hm = reinterpret_cast<HeatManagerPID*>(data);
    float value = hm->getDriveMax();
    DRAW_FLOAT_P(PSTR("Max. I-Part:"), Com::tEmpty, value, 1);
    if (GUI::handleFloatValueAction(action, value, -500, 500, 0.1)) {
        hm->setDriveMax(value);
    }
#endif
}

void HeatManagerPID::showConfigMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    // GUI::menuLongP(action, PSTR("Max. PWM:"), maxPWM, menuHMMaxPWM, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("P:"), P, 1, menuSetPIDP, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("I:"), I, 2, menuSetPIDI, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("D:"), D, 1, menuSetPIDD, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Min. I-Part:"), driveMin, 1, menuSetDriveMin, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Max. I-Part:"), driveMax, 1, menuSetDriveMax, this, GUIPageType::FIXED_CONTENT);
    showBaseConfigMenu(action);
#endif
}

void HeatManagerPID::autocalibrate(GCode* g) {
    ENSURE_POWER
    float temp = g->hasS() ? g->S : 150;
    int maxCycles = g->hasR() ? static_cast<int>(g->R) : 5;
    bool storeValues = g->hasX();
    int method = g->hasC() ? static_cast<int>(g->C) : 0;
    decoupleMode = DecoupleMode::CALIBRATING;
    if (method < 0)
        method = 0;
    if (method > 4)
        method = 4;
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    millis_t temp_millis = HAL::timeInMilliseconds();
    millis_t t1 = temp_millis;
    millis_t t2 = temp_millis;
    int32_t tHigh = 0;
    int32_t tLow;

    int32_t bias = maxPWM >> 1;
    int32_t d = bias;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTemp = 20;
    maxCycles = constrain(maxCycles, 5, 20);
    Com::printInfoFLN(Com::tPIDAutotuneStart);
    targetTemperature = temp;
    output->set(maxPWM);
    while (!Printer::breakLongCommand) {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif
        Commands::checkForPeriodicalActions(true); // update heaters etc.
        GCode::keepAlive(FirmwareState::WaitHeater);
        setCurrentTemperature(input->get());
        millis_t time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp, currentTemperature);
        minTemp = RMath::min(minTemp, currentTemperature);
        targetTemperature = heating ? temp : 0.0f;
        if (heating == true && currentTemperature > temp) { // switch heating -> off
            if (time - t2 > 5000) {
                heating = false;
                output->set(constrain(bias - d, 0, maxPWM));
                t1 = time;
                tHigh = t1 - t2;
                maxTemp = temp;
            }
        }
        if (heating == false && currentTemperature < temp) {
            if (time - t1 > 5000) {
                heating = true;
                t2 = time;
                tLow = t2 - t1; // half wave length
                if (cycles > 0) {
                    bias += (d * (tHigh - tLow)) / (tLow + tHigh);
                    bias = constrain(bias, 20, maxPWM - 20);
                    if (bias > maxPWM >> 1)
                        d = maxPWM - 1 - bias;
                    else
                        d = bias;

                    Com::printF(Com::tAPIDBias, bias);
                    Com::printF(Com::tAPIDD, d);
                    Com::printF(Com::tAPIDMin, minTemp);
                    Com::printFLN(Com::tAPIDMax, maxTemp);
                    if (cycles > 2) {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d) / (3.14159 * (maxTemp - minTemp));
                        Tu = static_cast<float>(tLow + tHigh) / 1000.0;
                        Com::printF(Com::tAPIDKu, Ku);
                        Com::printFLN(Com::tAPIDTu, Tu);
                        if (method == 0) {
                            Kp = 0.6 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu * 0.125;
                            Com::printFLN(Com::tAPIDClassic);
                        }
                        if (method == 1) {
                            Kp = 0.33 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu / 3.0;
                            Com::printFLN(Com::tAPIDSome);
                        }
                        if (method == 2) {
                            Kp = 0.2 * Ku;
                            Ki = Kp * 2.0 / Tu;
                            Kd = Kp * Tu / 3;
                            Com::printFLN(Com::tAPIDNone);
                        }
                        if (method == 3) {
                            Kp = 0.7 * Ku;
                            Ki = Kp * 2.5 / Tu;
                            Kd = Kp * Tu * 3.0 / 20.0;
                            Com::printFLN(Com::tAPIDPessen);
                        }
                        if (method == 4) {       //Tyreus-Lyben
                            Kp = 0.4545f * Ku;   //1/2.2 KRkrit
                            Ki = Kp / Tu / 2.2f; //2.2 Tkrit
                            Kd = Kp * Tu / 6.3f; //1/6.3 Tkrit[/code]
                            Com::printFLN(Com::tAPIDTyreusLyben);
                        }
                        Com::printFLN(Com::tAPIDKp, Kp);
                        Com::printFLN(Com::tAPIDKi, Ki);
                        Com::printFLN(Com::tAPIDKd, Kd);
                    }
                }
                output->set(constrain(bias + d, 0, maxPWM));
                cycles++;
                minTemp = temp;
            }
        }
        if (currentTemperature > (temp + 40)) {
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            setTargetTemperature(0);
            return;
        }
        if (time - temp_millis > 1000) {
            temp_millis = time;
            Commands::printTemperatures();
        }
        if (((time - t1) + (time - t2)) > (10L * 60L * 1000L * 2L)) { // 20 Minutes
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            setTargetTemperature(0);
            return;
        }
        if (cycles > maxCycles) {
            Com::printInfoFLN(Com::tAPIDFinished);
            if (storeValues) {
                P = Kp;
                I = Ki;
                D = Kd;
                Com::printInfoFLN(PSTR("New values stored until next reset. Send M500 to store permanently."));
            }
            setTargetTemperature(0);
            return;
        }
    } // loop
    setTargetTemperature(0);
}

// HeatManagerPeltierPID

template <class flowPin, int peltierType, int minTemp>
void HeatManagerPeltierPID<flowPin, peltierType, minTemp>::updateLocal(float tempError) {
    // pos tempError mean heating required
    float pidTerm = P * tempError;
    IState += ki * tempError;
    IState = constrain(IState, driveMin, driveMax);
    pidTerm += IState;
    pidTerm += kd * (lastTemperature - currentTemperature);
    if (peltierType == 0) { // only cooling
        if (pidTerm < 0) {
            output->set(constrain((int)-pidTerm, 0, maxPWM));
        } else {
            output->set(0); // need to heat but we can't
        }
    }
    if (peltierType == 1) { // only heating
        if (pidTerm < 0) {
            output->set(0); // need to heat but we can't
        } else {
            output->set(constrain((int)pidTerm, 0, maxPWM));
        }
    }
    if (peltierType == 2) { // heating (flowPin = 1) and cooling (flowPin = 0)
        if (pidTerm < 0) {
            flowPin::off();
            output->set(constrain((int)-pidTerm, 0, maxPWM));
        } else {
            flowPin::on();
            output->set(constrain((int)pidTerm, 0, maxPWM));
        }
    }
    lastTemperature = currentTemperature;
}

template <class flowPin, int peltierType, int minTemp>
void HeatManagerPeltierPID<flowPin, peltierType, minTemp>::autocalibrate(GCode* g) {
    ENSURE_POWER
    bool cooling = peltierType != PELTIER_HEATER;
    if (g->hasD()) {
        cooling = g->D <= 0;
    }
    flowPin::set(!cooling);
    float temp = g->hasS() ? g->S : 150;
    int maxCycles = g->hasR() ? static_cast<int>(g->R) : 5;
    bool storeValues = g->hasX();
    int method = g->hasC() ? static_cast<int>(g->C) : 0;
    decoupleMode = DecoupleMode::CALIBRATING;
    if (method < 0)
        method = 0;
    if (method > 4)
        method = 4;
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    millis_t temp_millis = HAL::timeInMilliseconds();
    millis_t t1 = temp_millis;
    millis_t t2 = temp_millis;
    int32_t tHigh = 0;
    int32_t tLow;

    int32_t bias = maxPWM >> 1;
    int32_t d = bias;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTempR = 20;
    maxCycles = constrain(maxCycles, 5, 20);
    Com::printInfoFLN(Com::tPIDAutotuneStart);
    targetTemperature = temp;
    output->set(maxPWM);
    while (!Printer::breakLongCommand) {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif
        Commands::checkForPeriodicalActions(true); // update heaters etc.
        GCode::keepAlive(FirmwareState::WaitHeater);
        setCurrentTemperature(input->get());
        millis_t time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp, currentTemperature);
        minTempR = RMath::min(minTempR, currentTemperature);
        if (cooling) {
            if (heating == true && currentTemperature < temp) { // switch heating -> off
                if (time - t2 > 5000) {
                    heating = false;
                    output->set(constrain(bias - d, 0, maxPWM));
                    t1 = time;
                    tHigh = t1 - t2;
                    maxTemp = temp;
                }
            }
            if (heating == false && currentTemperature > temp) {
                if (time - t1 > 5000) {
                    heating = true;
                    t2 = time;
                    tLow = t2 - t1; // half wave length
                    if (cycles > 0) {
                        bias += (d * (tHigh - tLow)) / (tLow + tHigh);
                        bias = constrain(bias, 20, maxPWM - 20);
                        if (bias > maxPWM >> 1)
                            d = maxPWM - 1 - bias;
                        else
                            d = bias;

                        Com::printF(Com::tAPIDBias, bias);
                        Com::printF(Com::tAPIDD, d);
                        Com::printF(Com::tAPIDMin, minTempR);
                        Com::printFLN(Com::tAPIDMax, maxTemp);
                        if (cycles > 2) {
                            // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                            Ku = (4.0 * d) / (3.14159 * (maxTemp - minTempR));
                            Tu = static_cast<float>(tLow + tHigh) / 1000.0;
                            Com::printF(Com::tAPIDKu, Ku);
                            Com::printFLN(Com::tAPIDTu, Tu);
                            if (method == 0) {
                                Kp = 0.6 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu * 0.125;
                                Com::printFLN(Com::tAPIDClassic);
                            }
                            if (method == 1) {
                                Kp = 0.33 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu / 3.0;
                                Com::printFLN(Com::tAPIDSome);
                            }
                            if (method == 2) {
                                Kp = 0.2 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu / 3;
                                Com::printFLN(Com::tAPIDNone);
                            }
                            if (method == 3) {
                                Kp = 0.7 * Ku;
                                Ki = Kp * 2.5 / Tu;
                                Kd = Kp * Tu * 3.0 / 20.0;
                                Com::printFLN(Com::tAPIDPessen);
                            }
                            if (method == 4) {       //Tyreus-Lyben
                                Kp = 0.4545f * Ku;   //1/2.2 KRkrit
                                Ki = Kp / Tu / 2.2f; //2.2 Tkrit
                                Kd = Kp * Tu / 6.3f; //1/6.3 Tkrit[/code]
                                Com::printFLN(Com::tAPIDTyreusLyben);
                            }
                            Com::printFLN(Com::tAPIDKp, Kp);
                            Com::printFLN(Com::tAPIDKi, Ki);
                            Com::printFLN(Com::tAPIDKd, Kd);
                        }
                    }
                    output->set(constrain(bias + d, 0, maxPWM));
                    cycles++;
                    minTempR = temp;
                }
            }
        } else {
            if (heating == true && currentTemperature > temp) { // switch heating -> off
                if (time - t2 > 5000) {
                    heating = false;
                    output->set(constrain(bias - d, 0, maxPWM));
                    t1 = time;
                    tHigh = t1 - t2;
                    maxTemp = temp;
                }
            }
            if (heating == false && currentTemperature < temp) {
                if (time - t1 > 5000) {
                    heating = true;
                    t2 = time;
                    tLow = t2 - t1; // half wave length
                    if (cycles > 0) {
                        bias += (d * (tHigh - tLow)) / (tLow + tHigh);
                        bias = constrain(bias, 20, maxPWM - 20);
                        if (bias > maxPWM >> 1)
                            d = maxPWM - 1 - bias;
                        else
                            d = bias;

                        Com::printF(Com::tAPIDBias, bias);
                        Com::printF(Com::tAPIDD, d);
                        Com::printF(Com::tAPIDMin, minTempR);
                        Com::printFLN(Com::tAPIDMax, maxTemp);
                        if (cycles > 2) {
                            // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                            Ku = (4.0 * d) / (3.14159 * (maxTemp - minTempR));
                            Tu = static_cast<float>(tLow + tHigh) / 1000.0;
                            Com::printF(Com::tAPIDKu, Ku);
                            Com::printFLN(Com::tAPIDTu, Tu);
                            if (method == 0) {
                                Kp = 0.6 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu * 0.125;
                                Com::printFLN(Com::tAPIDClassic);
                            }
                            if (method == 1) {
                                Kp = 0.33 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu / 3.0;
                                Com::printFLN(Com::tAPIDSome);
                            }
                            if (method == 2) {
                                Kp = 0.2 * Ku;
                                Ki = Kp * 2.0 / Tu;
                                Kd = Kp * Tu / 3;
                                Com::printFLN(Com::tAPIDNone);
                            }
                            if (method == 3) {
                                Kp = 0.7 * Ku;
                                Ki = Kp * 2.5 / Tu;
                                Kd = Kp * Tu * 3.0 / 20.0;
                                Com::printFLN(Com::tAPIDPessen);
                            }
                            if (method == 4) {       //Tyreus-Lyben
                                Kp = 0.4545f * Ku;   //1/2.2 KRkrit
                                Ki = Kp / Tu / 2.2f; //2.2 Tkrit
                                Kd = Kp * Tu / 6.3f; //1/6.3 Tkrit[/code]
                                Com::printFLN(Com::tAPIDTyreusLyben);
                            }
                            Com::printFLN(Com::tAPIDKp, Kp);
                            Com::printFLN(Com::tAPIDKi, Ki);
                            Com::printFLN(Com::tAPIDKd, Kd);
                        }
                    }
                    output->set(constrain(bias + d, 0, maxPWM));
                    cycles++;
                    minTempR = temp;
                }
            }
        }
        if ((!cooling && currentTemperature > (temp + 40)) || (!cooling && currentTemperature < (temp - 40))) {
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            setTargetTemperature(0);
            return;
        }
        if (time - temp_millis > 1000) {
            temp_millis = time;
            Commands::printTemperatures();
        }
        if (((time - t1) + (time - t2)) > (10L * 60L * 1000L * 2L)) { // 20 Minutes
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            setTargetTemperature(0);
            return;
        }
        if (cycles > maxCycles) {
            Com::printInfoFLN(Com::tAPIDFinished);
            if (storeValues) {
                P = Kp;
                I = Ki;
                D = Kd;
                Com::printInfoFLN(PSTR("New values stored until next reset. Send M500 to store permanently."));
            }
            setTargetTemperature(0);
            return;
        }
    } // loop
    setTargetTemperature(0);
}

// HeatManagerDynDeadTime

void HeatManagerDynDeadTime::updateLocal(float tempError) {
    counter = (counter + 1) & 3;
    float rising = (currentTemperature - lastTemperatures[counter]) * 2.5;
    lastTemperatures[counter] = currentTemperature;
    if (rising > 0) {
        output->set(currentTemperature + deadUp * rising < targetTemperature ? maxPWM : 0);
    } else {
        output->set(currentTemperature + deadDown * rising < targetTemperature ? maxPWM : 0);
    }
}

void HeatManagerDynDeadTime::setTargetTemperature(float temp) {
    HeatManager::setTargetTemperature(temp);
    updateTimings();
}

void HeatManagerDynDeadTime::updateTimings() {
    float scale;
    // file deepcode ignore FloatingPointEquals: <please specify a reason of ignoring this>
    if (temp1 == temp2) {
        deadUp = deadUp1;
        deadDown = deadDown1;
        return;
    }
    scale = (targetTemperature - temp1) / (temp2 - temp1);
    deadUp = scale * deadUp2 + (1.0 - scale) * deadUp1;
    deadDown = scale * deadDown2 + (1.0 - scale) * deadDown1;
    if (deadUp < 1) { // protect against nonsense
        deadUp = 1;
    }
    if (deadDown < 1) {
        deadDown = 1;
    }
    // Com::printFLN(PSTR("DeadUp"), deadUp, 2);
    // Com::printFLN(PSTR("DeadDown"), deadDown, 2);
}

void HeatManagerDynDeadTime::resetFromConfig(fast8_t _maxPwm, float decVariance, millis_t decPeriod,
                                             float _temp1, float _deadUp1, float _deadDown1, float _temp2, float _deadUp2, float _deadDown2) {
    maxPWM = _maxPwm;
    decoupleVariance = decVariance;
    decouplePeriod = decPeriod;
    temp1 = _temp1;
    deadUp1 = _deadUp1;
    deadDown1 = _deadDown1;
    temp2 = _temp2;
    deadUp2 = _deadUp2;
    deadDown2 = _deadDown2;
    updateTimings();
}

void HeatManagerDynDeadTime::eepromHandleLocal(int pos) {
    EEPROM::handleFloat(pos, PSTR("Temp 1 [deg C]"), 0, temp1);
    EEPROM::handleFloat(pos + 4, PSTR("Up Time 1 [s]"), 2, deadUp1);
    EEPROM::handleFloat(pos + 8, PSTR("Down Time 1 [s]"), 2, deadDown1);
    EEPROM::handleFloat(pos + 12, PSTR("Temp 2 [deg C]"), 0, temp2);
    EEPROM::handleFloat(pos + 16, PSTR("Up Time 2 [s]"), 2, deadUp2);
    EEPROM::handleFloat(pos + 20, PSTR("Down Time 2 [s]"), 2, deadDown2);
    updateTimings();
}

int HeatManagerDynDeadTime::eepromSizeLocal() {
    return 6 * 4;
}

/* void HeatManagerDynDeadTime::showControlMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    GUI::flashToStringLong(GUI::tmpString, PSTR("Set Temp: @°C"), static_cast<int32_t>(lroundf(targetTemperature)));
    GUI::menuSelectable(action, GUI::tmpString, menuSetTemperature, this, GUIPageType::FIXED_CONTENT);
#endif
} */

void menuSetDDPTime1(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getTemp1();
    DRAW_FLOAT_P(PSTR("Temp 1:"), Com::tUnitDegCelsius, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 1)) {
        hm->setTemp1(value);
    }
#endif
}
void menuSetDDPTime2(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getTemp2();
    DRAW_FLOAT_P(PSTR("Temp 2:"), Com::tUnitDegCelsius, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 1)) {
        hm->setTemp2(value);
    }
#endif
}
void menuSetDDPUp1(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getDeadUp1();
    DRAW_FLOAT_P(PSTR("Up Time 1:"), Com::tUnitSeconds, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 0.1)) {
        hm->setDeadUp1(value);
    }
#endif
}
void menuSetDDPUp2(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getDeadUp2();
    DRAW_FLOAT_P(PSTR("Up Time 2:"), Com::tUnitSeconds, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 0.1)) {
        hm->setDeadUp2(value);
    }
#endif
}
void menuSetDDPDown1(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getDeadDown1();
    DRAW_FLOAT_P(PSTR("Up Time 1:"), Com::tUnitSeconds, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 0.1)) {
        hm->setDeadDown1(value);
    }
#endif
}
void menuSetDDPDown2(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    HeatManagerDynDeadTime* hm = reinterpret_cast<HeatManagerDynDeadTime*>(data);
    float value = hm->getDeadDown2();
    DRAW_FLOAT_P(PSTR("Up Time 2:"), Com::tUnitSeconds, value, 1);
    if (GUI::handleFloatValueAction(action, value, 0, 600, 0.1)) {
        hm->setDeadDown2(value);
    }
#endif
}

void HeatManagerDynDeadTime::showConfigMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    // GUI::menuLongP(action, PSTR("Max. PWM:"), maxPWM, menuHMMaxPWM, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Temp 1:"), temp1, 0, menuSetDDPTime1, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Up Time 1:"), deadUp1, 1, menuSetDDPUp1, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Down Time 1:"), deadDown1, 1, menuSetDDPDown1, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Temp 2:"), temp2, 0, menuSetDDPTime2, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Up Time 2:"), deadUp2, 1, menuSetDDPUp2, this, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Down Time 2:"), deadDown2, 1, menuSetDDPDown2, this, GUIPageType::FIXED_CONTENT);
    showBaseConfigMenu(action);
#endif
}

bool HeatManagerDynDeadTime::detectTimings(float temp, float& up, float& down, float reduce) {
    fast8_t mode = 0;
    output->set(0);
    float avg[4], avgTemp;
    float last[16]; // increased precision 0.025s interval is measured
    millis_t lastTimes[16];
    float raise, referenceRaise = 0;
    int lastPos = 0;
    int goodCount = 0;
    millis_t lastTime = HAL::timeInMilliseconds();
    for (int i = 0; i < 16; i++) {
        lastTimes[i] = lastTime - 25;
        last[i] = currentTemperature;
        avg[i & 3] = currentTemperature;
    }
    targetTemperature = currentTemperature > temp - 10.0f ? temp - 10.0f : temp;
    millis_t timeReference = lastTime;
    millis_t timeSecond = HAL::timeInMilliseconds();
    while (mode != 5) {
        if (Printer::breakLongCommand) {
            return false;
        }
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif
        Commands::checkForPeriodicalActions(true); // update heaters etc.
        GCode::keepAlive(FirmwareState::WaitHeater);
        millis_t time = HAL::timeInMilliseconds();
        if (time - lastTime >= 25) { // store new
            setCurrentTemperature(input->get());
            avgTemp = currentTemperature;
            for (int j = 0; j < 4; j++) {
                avgTemp += avg[j];
            }
            avgTemp *= 0.2;
            lastPos = (lastPos + 1) & 15;
            avg[lastPos & 3] = currentTemperature;
            lastTime = time;
            raise = (avgTemp - last[lastPos]) * 1000.0 / static_cast<float>(time - lastTimes[lastPos]);
            // Com::printF(PSTR("Temp:"), avgTemp, 2);
            // Com::printFLN(PSTR(" Raise:"), raise, 2);
            last[lastPos] = avgTemp;
            lastTimes[lastPos] = lastTime;
            if (time - timeSecond > 1000) {
                timeSecond = time;
                Commands::printTemperatures();
            }
        } else {
            continue;
        }
        switch (mode) {
        case 0: // Wait for minimum 10°C below temp to start testing
            if (currentTemperature < temp - 10) {
                mode = 1;
                targetTemperature = temp;
                output->set(maxPWM);
            }
            break;
        case 1: // wait for temp raising
            if (currentTemperature >= temp) {
                mode = 2;
                referenceRaise = raise * reduce;
                timeReference = time;
                targetTemperature = 0.0f;
                output->set(0);
                goodCount = 0;
            }
            break;
        case 2: // wait for curve flattening
            if (raise < referenceRaise) {
                goodCount++;
                if (goodCount == 4) {
                    up = static_cast<float>(time - timeReference) * 0.001 - goodCount * 0.025;
                    Com::printFLN(PSTR("Dead Time Up:"), up, 2);
                    mode = 3;
                }
            } else {
                goodCount = 0;
            }
            break;
        case 3: // wait for temperature lower temp
            if (currentTemperature < temp) {
                mode = 4;
                referenceRaise = raise * reduce;
                timeReference = time;
                targetTemperature = temp;
                output->set(maxPWM);
                goodCount = 0;
            }
            break;
        case 4: // wait for curve turning up
            if (raise > referenceRaise) {
                goodCount++;
                if (goodCount == 4) {
                    down = static_cast<float>(time - timeReference) * 0.001 - goodCount * 0.025;
                    Com::printFLN(PSTR("Dead Time Down:"), down, 2);
                    mode = 5;
                }
            } else {
                goodCount = 0;
            }
            break;
        }
    }
    targetTemperature = 0.0f;
    output->set(0);
    return true;
}

void HeatManagerDynDeadTime::autocalibrate(GCode* g) {
    ENSURE_POWER
    decoupleMode = DecoupleMode::CALIBRATING;
    float reduce = g->hasF() ? g->F : 0.6f;
    if (g->hasA()) {
        Com::printFLN(PSTR("Detect timings for temp:"), g->A);
        if (detectTimings(g->A, deadUp1, deadDown1, reduce)) {
            temp1 = g->A;
        }
    }

    if (g->hasB()) {
        Com::printFLN(PSTR("Detect timings for temp:"), g->B);
        if (detectTimings(g->B, deadUp2, deadDown2, reduce)) {
            temp2 = g->B;
        }
    }
    EEPROM::markChanged();
    updateTimings();
    setTargetTemperature(0);
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
