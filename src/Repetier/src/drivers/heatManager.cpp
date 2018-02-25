#include "../../Repetier.h"

HeatManager* heaters[] = HEATERS;

void HeatManager::update() {
    if (error != HeaterError::NO_ERROR) {
        return; // do nothing in error state
    }
    if (errorCount > 0) {
        errorCount--;
    }
    if (decoupleMode == CALIBRATING) {
        return; // do not interfere with calibration
    }
    if (decoupleMode == PAUSED) {
        return; // do nothing in pause mode
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

void HeatManager::disableAllHeaters() {
    for (fast8_t i = 0; i < NUM_HEATERS; i++) {
        heaters[i]->setTargetTemperature(0);
    }
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

void HeatManager::resetAllErrorStates() {
    for(uint8_t i = 0; i < NUM_HEATERS; i++) {
        heaters[i]->resetError();
    }
}

bool HeatManager::reportTempsensorError() {
#if NUM_HEATERS > 0
    if(!Printer::isAnyTempsensorDefect()) return false;
    for(uint8_t i = 0; i < NUM_HEATERS; i++) {
        HeatManager *h = heaters[i];
        if(h->isBedHeater()) {
            Com::printF(Com::tHeatedBed);
        } else if(h->isExtruderHeater()) {
             Com::printF(Com::tExtruderSpace, i);
        } else if(h->isChamberHeater()) {
             Com::printF(PSTR("Heated Chamber "), i);
        } else {
            Com::printF(PSTR("Other:"));
        }
        HeaterError err = h->getError();
        if(err == HeaterError::SENSOR_DEFECT)
            Com::printF(Com::tTempSensorDefect);
        else
            Com::printF(Com::tTempSensorWorking);
        if(err == HeaterError::NO_HEATUP)
            Com::printF(PSTR(" temperature does not rise"));
        if(err == HeaterError::LEAVING_RANGE)
            Com::printF(PSTR(" temperature left control range"));
        Com::println();
    }
    Com::printErrorFLN(Com::tDryModeUntilRestart);
    return true;
#else
    return false;
#endif
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
    if (wasOutsideRange) {
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

void HeatManagerPID::setPID(float p, float i, float d) {
    P = p;
    I = i;
    D = d;
    updateDerived();
}

void HeatManagerPID::autocalibrate(GCode* g) {
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
    for (;;) {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif                                             // FEATURE_WATCHDOG
        Commands::checkForPeriodicalActions(true); // update heaters etc.
        GCode::keepAlive(WaitHeater);
        setCurrentTemperature(input->get());
        millis_t time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp, currentTemperature);
        minTemp = RMath::min(minTemp, currentTemperature);
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
        UI_MEDIUM;
        UI_SLOW(true);
    } // loop
    setTargetTemperature(0);
}