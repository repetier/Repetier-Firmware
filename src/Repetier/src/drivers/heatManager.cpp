#include "../../Repetier.h"

void HeatManager::update() {
    if (error != HeaterError::NO_ERROR) {
        return; // do nothing in error state
    }
    if (errorCount > 0) {
        errorCount--;
    }
    if (input->isDefect()) {
        errorCount += 2;
        if (errorCount > 10) {
            Com::printErrorFLN(PSTR("Heater seems to be defect. Sensor reported unusual values."));
            Com::printErrorFLN(PSTR("This can be a broken wire or a shorted contact of the sensor."));
            setError(HeaterError::SENSOR_DEFECT);
        }
        return;
    } else {
        setCurrentTemperature(input->get());
    }
    if (targetTemperature <= MAX_ROOM_TEMPERATURE) { // heater disabled
        output->set(0);
        decoupleMode = DecoupleMode::NO_HEATING;
        return;
    }
    // Test for decoupled HeaterError

    millis_t time = HAL::timeInMilliseconds();
    float tempError = targetTemperature - currentTemperature;
    if (decouplePeriod > 0 && time - lastDecoupleTest > decouplePeriod) {
        // we should do a test
        if (decoupleMode == DecoupleMode::FAST_RISING) {
            if (currentTemperature - 1 < lastDecoupleTemp) {
                // we waited and nothing happened, that is not ok
                Com::printErrorFLN(PSTR("A heater did not rise while under full power, so we disabled the heater."));
                Com::printErrorFLN(PSTR("If it is no hardware defect, the decoupling period might be set too low."));
                Com::printErrorF(PSTR("No temperaure rise after (ms):"));
                Com::print(decouplePeriod);
                Com::println();
                setError(HeaterError::NO_HEATUP);
                return;
            }
            lastDecoupleTemp = currentTemperature;
        } else if (decoupleMode == DecoupleMode::HOLDING) {
            if (fabs(error) > decoupleVariance) {
                // Temperature left target range
                Com::printErrorFLN(PSTR("The temperature for a heater left the reached target area."));
                Com::printErrorFLN(PSTR("This can happen on a hardware defect or if the decouple temperature variance is set too low."));
                Com::printErrorF(PSTR("Deviation:"));
                Com::print(error);
                Com::println();
                setError(HeaterError::LEAVING_RANGE);
                return;
            }
        }
        lastDecoupleTest = time;
    }
    if ((decoupleMode == DecoupleMode::FAST_RISING || decoupleMode == DecoupleMode::COOLING) && fabs(error) < decoupleVariance) {
        lastDecoupleTest = time;
        decoupleMode = DecoupleMode::HOLDING;
    }

    // Control heater

    if (tempError > TEMPERATURE_CONTROL_RANGE) {
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
    if (tempError < -TEMPERATURE_CONTROL_RANGE) {
        output->set(0);
        decoupleMode = DecoupleMode::COOLING;
        wasOutsideRange = 1;
        return;
    }
    updateLocal(tempError);
}

void HeatManager::waitForTargetTemperature() {
    if (targetTemperature < MAX_ROOM_TEMPERATURE)
        return;
    if (Printer::debugDryrun())
        return;
    bool oldReport = Printer::isAutoreportTemp();
    Printer::setAutoreportTemp(true);
    while (true) {
        Commands::checkForPeriodicalActions(true);
        GCode::keepAlive(WaitHeater);
        if (fabs(targetTemperature - currentTemperature) <= 1) {
            Printer::setAutoreportTemp(oldReport);
            return;
        }
    }
}

void HeatManager::reportTemperature(char c, int idx) {
    Com::print(c);
    if (idx >= 0) {
        Com::print(idx);
    }
    Com::print(':');
    Com::printFloat(currentTemperature, 1);
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

void HeatManagerBangBang::eepromHandleLocal(int adr) {
}

void HeatManagerBangBang::eepromResetLocal() {
}

int HeatManagerBangBang::eepromSizeLocal() {
    return 0;
}

void HeatManagerPID::updateLocal(float tempError) {
    counter++;
    if(wasOutsideRange) {
        lastTemperature = actTemperature = currentTemperature;
        counter = 0;        
        if (wasOutsideRange) {
            IState = IMax;
        } else {
            IState = IMin;
        }
        wasOutsideRange = 0;
    }
    if (counter >= 10) {
        counter = 0;
        lastTemperature = actTemperature;
        actTemperature = currentTemperature;
    }
    float pidTerm = P * tempError;
    IState = constrain(IState + tempError, IMin, IMax);
    pidTerm += I * IState * 0.1f; // 0.1 = 10Hz
    pidTerm += D * (lastTemperature - actTemperature);
#if SCALE_PID_TO_MAX == 1
    pidTerm = (pidTerm * maxPWM) * 0.0039215;
#endif // SCALE_PID_TO_MAX
    output->set(constrain((int)pidTerm, 0, maxPWM));
}
void HeatManagerPID::updateDerived() {
    IMax = (float)driveMax * 10.0f / I;
    IMin = (float)driveMin * 10.0f / I;
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

void HeatManagerPID::eepromHandleLocal(int adr) {
}

void HeatManagerPID::eepromResetLocal() {
}

int HeatManagerPID::eepromSizeLocal() {
    return 0;
}
