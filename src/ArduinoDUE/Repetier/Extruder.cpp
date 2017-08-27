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

uint8_t manageMonitor = 0; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
unsigned int counterPeriodical = 0;
volatile uint8_t executePeriodical = 0;
uint8_t counter500ms = 5;
#if FEATURE_DITTO_PRINTING
uint8_t Extruder::dittoMode = 0;
#endif
#ifdef SUPPORT_MAX6675
extern int16_t read_max6675(uint8_t ss_pin);
#endif
#ifdef SUPPORT_MAX31855
extern int16_t read_max31855(uint8_t ss_pin);
#endif

#if ANALOG_INPUTS > 0
const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif

#ifdef USE_GENERIC_THERMISTORTABLE_1
short temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
short temptable_generic2[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
short temptable_generic3[GENERIC_THERM_NUM_ENTRIES][2];
#endif
/** Makes updates to temperatures and heater state every call.

Is called every 100ms.
*/
static uint8_t extruderTempErrors = 0;

void Extruder::manageTemperatures()
{
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
    uint8_t errorDetected = 0;
#ifdef RED_BLUE_STATUS_LEDS
    bool hot = false;
#endif
    bool newDefectFound = false;
    millis_t time = HAL::timeInMilliseconds(); // compare time for decouple tests
#if NUM_TEMPERATURE_LOOPS > 0
    for(uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
    {
        TemperatureController *act = tempController[controller];
        // Get Temperature
        act->updateCurrentTemperature();
#if FAN_THERMO_PIN > -1
        // Special case thermistor controlled fan
        if(act == &thermoController) {
            if(act->currentTemperatureC < Printer::thermoMinTemp)
                pwm_pos[PWM_FAN_THERMO] = 0;
            else if(act->currentTemperatureC > Printer::thermoMaxTemp)
                pwm_pos[PWM_FAN_THERMO] = FAN_THERMO_MAX_PWM;
            else {
                // Interpolate target speed
                float out = FAN_THERMO_MIN_PWM + (FAN_THERMO_MAX_PWM-FAN_THERMO_MIN_PWM) * (act->currentTemperatureC - Printer::thermoMinTemp) / (Printer::thermoMaxTemp - Printer::thermoMinTemp);
                if(out > 255)
                    pwm_pos[PWM_FAN_THERMO] = FAN_THERMO_MAX_PWM;
                else
                    pwm_pos[PWM_FAN_THERMO] = static_cast<uint8_t>(out);
            }
            continue;
        }
#endif
        // Handle automatic cooling of extruders
        if(controller < NUM_EXTRUDER)
        {
#if ((SHARED_COOLER && NUM_EXTRUDER >= 2 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN) || SHARED_COOLER_BOARD_EXT) && EXT0_EXTRUDER_COOLER_PIN > -1
            if(controller == 0)
            {
                bool enable = false;
                for(uint8_t j = 0; j < NUM_EXTRUDER; j++)
                {
                    if(tempController[j]->currentTemperatureC >= EXTRUDER_FAN_COOL_TEMP || tempController[j]->targetTemperatureC >= EXTRUDER_FAN_COOL_TEMP)
                    {
                        enable = true;
                        break;
                    }
                }
#if SHARED_COOLER_BOARD_EXT
                if(pwm_pos[PWM_BOARD_FAN]) enable = true;
#endif
                extruder[0].coolerPWM = (enable ? extruder[0].coolerSpeed : 0);
            } // controller == 0
#else
            if(act->currentTemperatureC < EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC < EXTRUDER_FAN_COOL_TEMP)
                extruder[controller].coolerPWM = 0;
            else
                extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
#endif // NUM_EXTRUDER
        } // extruder controller
        // do skip temperature control while auto tuning is in progress
        if(controller == autotuneIndex) continue;

        // Check for obvious sensor errors
        if(act->currentTemperatureC < MIN_DEFECT_TEMPERATURE || act->currentTemperatureC > MAX_DEFECT_TEMPERATURE)   // no temp sensor or short in sensor, disable heater
        {
            errorDetected = 1;
            if(extruderTempErrors < 10)    // Ignore short temporary failures
                extruderTempErrors++;
            else
            {
                act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
                if(!Printer::isAnyTempsensorDefect())
                {
                    newDefectFound = true;
                    Printer::setAnyTempsensorDefect();
                    reportTempsensorError();
                }
                EVENT_HEATER_DEFECT(controller);
            }
        }
        else if(controller == HEATED_BED_INDEX && Extruder::getHeatedBedTemperature() > HEATED_BED_MAX_TEMP + 5) {
            errorDetected = 1;
            if(extruderTempErrors < 10)    // Ignore short temporary failures
            extruderTempErrors++;
            else
            {
                act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
                Com::printErrorFLN(PSTR("Heated bed exceeded max temperature!"));
                if(!Printer::isAnyTempsensorDefect())
                {
                    newDefectFound = true;
                    Printer::setAnyTempsensorDefect();
                    reportTempsensorError();
                }
                EVENT_HEATER_DEFECT(controller);
            }			
        }
#ifdef RED_BLUE_STATUS_LEDS
        if(act->currentTemperatureC > 50)
            hot = true;
#endif // RED_BLUE_STATUS_LEDS
        if(Printer::isAnyTempsensorDefect()) continue;
        uint8_t on = act->currentTemperatureC >= act->targetTemperatureC ? LOW : HIGH;
        // Make a sound if alarm was set on reaching target temperature
        if(!on && act->isAlarm())
        {
            beep(50 * (controller + 1), 3);
            act->setAlarm(false);  //reset alarm
        }

        // Run test if heater and sensor are decoupled
        bool decoupleTestRequired = !errorDetected && act->decoupleTestPeriod > 0 && (time - act->lastDecoupleTest) > act->decoupleTestPeriod; // time enough for temperature change?
        if(decoupleTestRequired && act->isDecoupleFullOrHold() && Printer::isPowerOn()) // Only test when powered
        {
            if(act->isDecoupleFull()) // Phase 1: Heating fully until target range is reached
            {
                if(act->currentTemperatureC - act->lastDecoupleTemp < DECOUPLING_TEST_MIN_TEMP_RISE)   // failed test
                {
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10)   // Ignore short temporary failures
                    {
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
                        
                        if(!Printer::isAnyTempsensorDefect())
                        {
                            Printer::setAnyTempsensorDefect();
                            newDefectFound = true;
                        }
                        UI_ERROR_P(Com::tHeaterDecoupled);
                        Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                        Com::printF(PSTR("Error:Temp. raised to slow. Rise = "),act->currentTemperatureC - act->lastDecoupleTemp);
                        Com::printF(PSTR(" after "),(int32_t)(time-act->lastDecoupleTest));
                        Com::printFLN(PSTR(" ms"));
                        EVENT_HEATER_DECOUPLED(controller);
                    }
                }
                else
                {
                    act->stopDecouple();
                    act->startFullDecouple(time);
                }
            }
            else     // Phase 2: Holding temperature inside a target corridor
            {
                if(fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE)   // failed test
                {
                    extruderTempErrors++;
                    errorDetected = 1;
                    if(extruderTempErrors > 10)   // Ignore short temporary failures
                    {
                        act->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
                        if(!Printer::isAnyTempsensorDefect())
                        {
                            Printer::setAnyTempsensorDefect();
                            newDefectFound = true;
                        }
                        UI_ERROR_P(Com::tHeaterDecoupled);
                        Com::printErrorFLN(Com::tHeaterDecoupledWarning);
                        Com::printF(PSTR("Error:Could not hold temperature "),act->lastDecoupleTemp);
                        Com::printF(PSTR(" measured "),act->currentTemperatureC);
                        Com::printFLN(PSTR(" deg. C"));
                        EVENT_HEATER_DECOUPLED(controller);
                    }
                }
                else
                {
                    act->lastDecoupleTest = time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }

#if TEMP_PID
        act->tempArray[act->tempPointer++] = act->currentTemperatureC;
        act->tempPointer &= 3;
        uint8_t output = 0;
        float error = act->targetTemperatureC - act->currentTemperatureC;
        if(act->targetTemperatureC < 20.0f) // heating is off
        {
            output = 0; // off is off, even if damping term wants a heat peak!
            act->stopDecouple();
        }
        else if(error > PID_CONTROL_RANGE) // Phase 1: full heating until control range reached
        {
            output = act->pidMax;
            act->startFullDecouple(time);
        }
        else if(error < -PID_CONTROL_RANGE) // control range left upper side!
            output = 0;
        else // control range handle by heat manager
        {
            if(act->heatManager == HTR_PID)
            {
                act->startHoldDecouple(time);
                float pidTerm = act->pidPGain * error;
                act->tempIState = constrain(act->tempIState + error, act->tempIStateLimitMin, act->tempIStateLimitMax);
                pidTerm += act->pidIGain * act->tempIState * 0.1; // 0.1 = 10Hz
                float dgain = act->pidDGain * (act->tempArray[act->tempPointer] - act->currentTemperatureC) * 3.333f;
                pidTerm += dgain;
#if SCALE_PID_TO_MAX == 1
                pidTerm = (pidTerm * act->pidMax) * 0.0039215;
#endif // SCALE_PID_TO_MAX
                output = constrain((int)pidTerm, 0, act->pidMax);
            }
            else if(act->heatManager == HTR_DEADTIME)     // dead-time control
            {
                act->startHoldDecouple(time);
                float raising = 3.333 * (act->currentTemperatureC - act->tempArray[act->tempPointer]); // raising dT/dt, 3.33 = reciproke of time interval (300 ms)
                act->tempIState = 0.25 * (3.0 * act->tempIState + raising); // damp raising
                output = (act->currentTemperatureC + act->tempIState * act->deadTime > act->targetTemperatureC ? 0 : act->pidDriveMax);
            }
            else // bang bang and slow bang bang
#endif // TEMP_PID
                if(act->heatManager == HTR_SLOWBANG)    // Bang-bang with reduced change frequency to save relais life
                {
                    if (time - act->lastTemperatureUpdate > HEATED_BED_SET_INTERVAL)
                    {
                        output = (on ? act->pidMax : 0);
                        act->lastTemperatureUpdate = time;
                        if(on) act->startFullDecouple(time);
                        else act->stopDecouple();
                    }
                    else continue;
                }
                else     // Fast Bang-Bang fallback
                {
                    output = (on ? act->pidMax : 0);
                    if(on) act->startFullDecouple(time);
                    else act->stopDecouple();
                }
        } // Temperatur control
#ifdef MAXTEMP
        if(act->currentTemperatureC > MAXTEMP) // Force heater off if MAXTEMP is exceeded
            output = 0;
#endif // MAXTEMP
        pwm_pos[act->pwmIndex] = output; // set pwm signal
#if LED_PIN > -1
        if(act == &Extruder::current->tempControl)
            WRITE(LED_PIN,on);
#endif // LED_PIN
    } // for controller

#ifdef RED_BLUE_STATUS_LEDS
    if(Printer::isAnyTempsensorDefect())
    {
        WRITE(BLUE_STATUS_LED,HIGH);
        WRITE(RED_STATUS_LED,HIGH);
    }
    else
    {
        WRITE(BLUE_STATUS_LED,!hot);
        WRITE(RED_STATUS_LED,hot);
    }
#endif // RED_BLUE_STATUS_LEDS

    if(errorDetected == 0 && extruderTempErrors > 0)
        extruderTempErrors--;
    if(newDefectFound)
    {
        Com::printFLN(PSTR("Disabling all heaters due to detected sensor defect."));
        for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
        {
            tempController[i]->targetTemperatureC = 0;
            pwm_pos[tempController[i]->pwmIndex] = 0;
        }
#if defined(KILL_IF_SENSOR_DEFECT) && KILL_IF_SENSOR_DEFECT > 0
        if(!Printer::debugDryrun() && PrintLine::linesCount > 0)    // kill printer if actually printing
        {
#if SDSUPPORT
            sd.stopPrint();
#endif // SDSUPPORT
            Printer::kill(0);
        }
#endif // KILL_IF_SENSOR_DEFECT
        Printer::debugSet(8); // Go into dry mode
    } // any sensor defect
#endif // NUM_TEMPERATURE_LOOPS

}

/* For pausing we negate target temperature, so negative value means paused extruder.
Since temp. is negative no heating will occur. */
void Extruder::pauseExtruders()
{
    disableAllExtruderMotors();
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].tempControl.targetTemperatureC > 0)
            extruder[i].tempControl.targetTemperatureC = -extruder[i].tempControl.targetTemperatureC;
    }
}

void Extruder::unpauseExtruders()
{
    // activate temperatures
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].tempControl.targetTemperatureC < 0)
            extruder[i].tempControl.targetTemperatureC = -extruder[i].tempControl.targetTemperatureC;
    }
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
        extruder[i].tempControl.waitForTargetTemperature();
}

#if EXTRUDER_JAM_CONTROL
void Extruder::markAllUnjammed()
{
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        extruder[i].tempControl.setJammed(false);
        extruder[i].tempControl.setSlowedDown(false);
        extruder[i].resetJamSteps();
    }
    if(Printer::feedrateMultiply == JAM_SLOWDOWN_TO)
        Commands::changeFeedrateMultiply(100);
    Printer::unsetAnyTempsensorDefect(); // stop alarm
    Com::printInfoFLN(PSTR("Marked all extruders as unjammed."));
    Printer::setUIErrorMessage(false);
}

void Extruder::resetJamSteps()
{
    jamStepsOnSignal = jamStepsSinceLastSignal;
    jamStepsSinceLastSignal = 0;
    Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0 + id, false);
}
#endif // EXTRUDER_JAM_CONTROL

void Extruder::initHeatedBed()
{
#if TEMP_PID
    heatedBedController.updateTempControlVars();
#endif
}

#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
void createGenericTable(short table[GENERIC_THERM_NUM_ENTRIES][2],short minTemp,short maxTemp,float beta,float r0,float t0,float r1,float r2)
{
    t0 += 273.15f;
    float rs, vs;
    if(r1==0)
    {
        rs = r2;
        vs = GENERIC_THERM_VREF;
    }
    else
    {
        vs =static_cast<float>((GENERIC_THERM_VREF * r1) / (r1 + r2));
        rs = (r2 * r1) / (r1 + r2);
    }
    float k = r0 * exp(-beta / t0);
    float delta = (maxTemp-minTemp) / (GENERIC_THERM_NUM_ENTRIES - 1.0f);
    for(uint8_t i = 0; i < GENERIC_THERM_NUM_ENTRIES; i++)
    {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG
        float t = maxTemp - i * delta;
        float r = exp(beta / (t + 272.65)) * k;
        float v = 4092 * r * vs / ((rs + r) * GENERIC_THERM_VREF);
        int adc = static_cast<int>(v);
        t *= 8;
        if(adc > 4092) adc = 4092;
        table[i][0] = (adc >> (ANALOG_REDUCE_BITS));
        table[i][1] = static_cast<int>(t);
#ifdef DEBUG_GENERIC
        Com::printF(Com::tGenTemp,table[i][0]);
        Com::printFLN(Com::tComma,table[i][1]);
#endif
    }
}
#endif

/** \brief Initalizes all extruder.

Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder()
{
    uint8_t i;
    Extruder::current = &extruder[0];
#ifdef USE_GENERIC_THERMISTORTABLE_1
    createGenericTable(temptable_generic1,GENERIC_THERM1_MIN_TEMP,GENERIC_THERM1_MAX_TEMP,GENERIC_THERM1_BETA,GENERIC_THERM1_R0,GENERIC_THERM1_T0,GENERIC_THERM1_R1,GENERIC_THERM1_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
    createGenericTable(temptable_generic2,GENERIC_THERM2_MIN_TEMP,GENERIC_THERM2_MAX_TEMP,GENERIC_THERM2_BETA,GENERIC_THERM2_R0,GENERIC_THERM2_T0,GENERIC_THERM2_R1,GENERIC_THERM2_R2);
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
    createGenericTable(temptable_generic3,GENERIC_THERM3_MIN_TEMP,GENERIC_THERM3_MAX_TEMP,GENERIC_THERM3_BETA,GENERIC_THERM3_R0,GENERIC_THERM3_T0,GENERIC_THERM3_R1,GENERIC_THERM3_R2);
#endif
#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN > -1 && NUM_EXTRUDER > 0
    SET_OUTPUT(EXT0_DIR_PIN);
    SET_OUTPUT(EXT0_STEP_PIN);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    SET_OUTPUT(EXT0_DIR2_PIN);
    SET_OUTPUT(EXT0_STEP2_PIN);
    SET_OUTPUT(EXT0_ENABLE2_PIN);
    WRITE(EXT0_ENABLE2_PIN,!EXT0_ENABLE_ON);
#endif	
#endif
#if defined(EXT1_STEP_PIN) && EXT1_STEP_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_DIR_PIN);
    SET_OUTPUT(EXT1_STEP_PIN);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
    SET_OUTPUT(EXT1_DIR2_PIN);
    SET_OUTPUT(EXT1_STEP2_PIN);
    SET_OUTPUT(EXT1_ENABLE2_PIN);
    WRITE(EXT1_ENABLE2_PIN,!EXT1_ENABLE_ON);
#endif
#endif
#if defined(EXT2_STEP_PIN) && EXT2_STEP_PIN > -1 && NUM_EXTRUDER > 2
    SET_OUTPUT(EXT2_DIR_PIN);
    SET_OUTPUT(EXT2_STEP_PIN);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
    SET_OUTPUT(EXT2_DIR2_PIN);
    SET_OUTPUT(EXT2_STEP2_PIN);
    SET_OUTPUT(EXT2_ENABLE2_PIN);
    WRITE(EXT2_ENABLE2_PIN,!EXT2_ENABLE_ON);
#endif
#endif
#if defined(EXT3_STEP_PIN) && EXT3_STEP_PIN > -1 && NUM_EXTRUDER > 3
    SET_OUTPUT(EXT3_DIR_PIN);
    SET_OUTPUT(EXT3_STEP_PIN);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
    SET_OUTPUT(EXT3_DIR2_PIN);
    SET_OUTPUT(EXT3_STEP2_PIN);
    SET_OUTPUT(EXT3_ENABLE2_PIN);
    WRITE(EXT3_ENABLE2_PIN,!EXT3_ENABLE_ON);
#endif
#endif
#if defined(EXT4_STEP_PIN) && EXT4_STEP_PIN > -1 && NUM_EXTRUDER > 4
    SET_OUTPUT(EXT4_DIR_PIN);
    SET_OUTPUT(EXT4_STEP_PIN);
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER
    SET_OUTPUT(EXT4_DIR2_PIN);
    SET_OUTPUT(EXT4_STEP2_PIN);
    SET_OUTPUT(EXT4_ENABLE2_PIN);
    WRITE(EXT4_ENABLE2_PIN,!EXT4_ENABLE_ON);
#endif
#endif
#if defined(EXT5_STEP_PIN) && EXT5_STEP_PIN > -1 && NUM_EXTRUDER > 5
    SET_OUTPUT(EXT5_DIR_PIN);
    SET_OUTPUT(EXT5_STEP_PIN);
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER
    SET_OUTPUT(EXT5_DIR2_PIN);
    SET_OUTPUT(EXT5_STEP2_PIN);
    SET_OUTPUT(EXT5_ENABLE2_PIN);
    WRITE(EXT5_ENABLE2_PIN,!EXT5_ENABLE_ON);
#endif
#endif

    for(i = 0; i < NUM_EXTRUDER; ++i)
    {
        Extruder *act = &extruder[i];
        if(act->enablePin > -1)
        {
            HAL::pinMode(act->enablePin, OUTPUT);
            HAL::digitalWrite(act->enablePin,!act->enableOn);
        }
        act->tempControl.lastTemperatureUpdate = HAL::timeInMilliseconds();
#if defined(SUPPORT_MAX6675) || defined(SUPPORT_MAX31855)
        if(act->tempControl.sensorType == 101 || act->tempControl.sensorType == 102)
        {
            WRITE(SCK_PIN, 0);
            SET_OUTPUT(SCK_PIN);
            WRITE(MOSI_PIN, 1);
            SET_OUTPUT(MOSI_PIN);
            WRITE(MISO_PIN, 1);
            SET_INPUT(MISO_PIN);
            //SET_OUTPUT(SS);
            //WRITE(SS, HIGH);
            HAL::pinMode(SS, OUTPUT);
            HAL::digitalWrite(SS, 1);
            HAL::pinMode(act->tempControl.sensorPin, OUTPUT);
            HAL::digitalWrite(act->tempControl.sensorPin, 1);
        }
#endif
    }
#if HEATED_BED_HEATER_PIN > -1
    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
    Extruder::initHeatedBed();
#endif
    HAL::analogStart();

}


/** \brief Select extruder ext_num.

This function changes and initializes a new extruder. This is also called, after the eeprom values are changed.
*/
void Extruder::selectExtruderById(uint8_t extruderId)
{
#if NUM_EXTRUDER > 0
    if(extruderId >= NUM_EXTRUDER)
        extruderId = 0;
    Com::printFLN(PSTR("SelectExtruder:"), static_cast<int>(extruderId));
    if(Printer::isHomed() && extruder[extruderId].zOffset < Extruder::current->zOffset) { // prevent extruder from hitting bed
        Printer::offsetZ = -extruder[extruderId].zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
        Commands::waitUntilEndOfAllMoves();
        Printer::updateCurrentPosition(true);
    }

#if NUM_EXTRUDER > 1
    bool executeSelect = false;
    if(extruderId != Extruder::current->id)
    {
        GCode::executeFString(Extruder::current->deselectCommands);
        executeSelect = true;
    }
    Commands::waitUntilEndOfAllMoves();
#endif
    float cx, cy, cz;
    Printer::realPosition(cx, cy, cz);
    float oldfeedrate = Printer::feedrate;
    Extruder::current->extrudePosition = Printer::currentPositionSteps[E_AXIS];
#if DUAL_X_AXIS
    // Park current extruder
    int32_t dualXPos = Printer::currentPositionSteps[X_AXIS] - Printer::xMinSteps;
    if(Printer::isHomed())
        PrintLine::moveRelativeDistanceInSteps(Extruder::current->xOffset - dualXPos, 0, 0, 0, EXTRUDER_SWITCH_XY_SPEED, true, false);
#endif	
    Extruder::current = &extruder[extruderId];
#ifdef SEPERATE_EXTRUDER_POSITIONS
    // Use separate extruder positions only if being told. Slic3r e.g. creates a continuous extruder position increment
    Printer::currentPositionSteps[E_AXIS] = Extruder::current->extrudePosition;
#endif
    Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
    Printer::axisStepsPerMM[E_AXIS] = Extruder::current->stepsPerMM;
    Printer::invAxisStepsPerMM[E_AXIS] = 1.0f / Printer::axisStepsPerMM[E_AXIS];
    Printer::maxFeedrate[E_AXIS] = Extruder::current->maxFeedrate;
//   max_start_speed_units_per_second[E_AXIS] = Extruder::current->maxStartFeedrate;
    Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = Extruder::current->maxAcceleration;
    Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS] =
        Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxAccelerationMMPerSquareSecond[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];
#if USE_ADVANCE
    Printer::maxExtruderSpeed = (ufast8_t)floor(HAL::maxExtruderTimerFrequency() / (Extruder::current->maxFeedrate * Extruder::current->stepsPerMM));
#if CPU_ARCH == ARCH_ARM
    if(Printer::maxExtruderSpeed > 40) Printer::maxExtruderSpeed = 40;
#else
    if(Printer::maxExtruderSpeed > 15) Printer::maxExtruderSpeed = 15;
#endif
    float maxdist = Extruder::current->maxFeedrate * Extruder::current->maxFeedrate * 0.00013888 / Extruder::current->maxAcceleration;
    maxdist -= Extruder::current->maxStartFeedrate * Extruder::current->maxStartFeedrate * 0.5 / Extruder::current->maxAcceleration;
    float fmax = ((float)HAL::maxExtruderTimerFrequency() / ((float)Printer::maxExtruderSpeed * Printer::axisStepsPerMM[E_AXIS])); // Limit feedrate to interrupt speed
    if(fmax < Printer::maxFeedrate[E_AXIS]) Printer::maxFeedrate[E_AXIS] = fmax;
#endif
    Extruder::current->tempControl.updateTempControlVars();
#if DUAL_X_AXIS
    // Unpark new current extruder
    if(executeSelect) {// Run only when changing
        Commands::waitUntilEndOfAllMoves();
        Printer::updateCurrentPosition(true);
        GCode::executeFString(Extruder::current->selectCommands);
        executeSelect = false;
    }
    Printer::currentPositionSteps[X_AXIS] = Extruder::current->xOffset - dualXPos;
    if(Printer::isHomed())
        PrintLine::moveRelativeDistanceInSteps(-Extruder::current->xOffset + dualXPos, 0, 0, 0, EXTRUDER_SWITCH_XY_SPEED, true, false);
    Printer::currentPositionSteps[X_AXIS] = dualXPos + Printer::xMinSteps;
    Printer::offsetX = 0;
    Printer::updateCurrentPosition(true);
#else	
    Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
#endif
    Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply); // needed to adjust extrusionFactor to possibly different diameter
    if(Printer::isHomed()) {
        Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
    }
    Printer::feedrate = oldfeedrate;
    Printer::updateCurrentPosition();
#if USE_ADVANCE
    HAL::resetExtruderDirection();
#endif

#if NUM_EXTRUDER > 1
    if(executeSelect) {// Run only when changing
        Commands::waitUntilEndOfAllMoves();
        GCode::executeFString(Extruder::current->selectCommands);
    }
#endif
#endif
}

void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr, bool beep, bool wait)
{
#if NUM_EXTRUDER > 0
#if SHARED_EXTRUDER_HEATER
    extr = 0; // map any virtual extruder number to 0
#endif // SHARED_EXTRUDER_HEATER
    bool alloffs = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloffs = false;
#ifdef MAXTEMP
    if(temperatureInCelsius > MAXTEMP) temperatureInCelsius = MAXTEMP;
#endif
    if(temperatureInCelsius < 0) temperatureInCelsius = 0;
    TemperatureController *tc = tempController[extr];
    if(tc->sensorType == 0) temperatureInCelsius = 0;
    //if(temperatureInCelsius==tc->targetTemperatureC) return;
    tc->setTargetTemperature(temperatureInCelsius);
    tc->updateTempControlVars();
    if(beep && temperatureInCelsius > MAX_ROOM_TEMPERATURE)
        tc->setAlarm(true);
    if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[extr].coolerPWM = extruder[extr].coolerSpeed;
    Com::printF(Com::tTargetExtr,extr,0);
    Com::printFLN(Com::tColon,temperatureInCelsius,0);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode && extr == 0)
    {
        TemperatureController *tc2 = tempController[1];
        tc2->setTargetTemperature(temperatureInCelsius);
        tc2->updateTempControlVars();
        if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[1].coolerPWM = extruder[1].coolerSpeed;
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extr == 0)
        {
            TemperatureController *tc2 = tempController[2];
            tc2->setTargetTemperature(temperatureInCelsius);
            tc2->updateTempControlVars();
            if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[2].coolerPWM = extruder[2].coolerSpeed;
        }
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extr == 0)
        {
            TemperatureController *tc2 = tempController[3];
            tc2->setTargetTemperature(temperatureInCelsius);
            tc2->updateTempControlVars();
            if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[3].coolerPWM = extruder[3].coolerSpeed;
        }
#endif
    }
#endif // FEATURE_DITTO_PRINTING
    if(wait && temperatureInCelsius > MAX_ROOM_TEMPERATURE
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN > 0
            && !(abs(tc->currentTemperatureC - tc->targetTemperatureC) < (SKIP_M109_IF_WITHIN))// Already in range
#endif
      )
    {
        Extruder *actExtruder = &extruder[extr];
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HEATING_EXTRUDER_ID));
        EVENT_WAITING_HEATER(actExtruder->id);
        bool dirRising = actExtruder->tempControl.targetTemperatureC > actExtruder->tempControl.currentTemperatureC;
        millis_t printedTime = HAL::timeInMilliseconds();
        millis_t waituntil = 0;
#if RETRACT_DURING_HEATUP
        uint8_t retracted = 0;
#endif
        millis_t currentTime;
        do
        {
            previousMillisCmd = currentTime = HAL::timeInMilliseconds();
            if( (currentTime - printedTime) > 1000 )   //Print Temp Reading every 1 second while heating up.
            {
                Commands::printTemperatures();
                printedTime = currentTime;
            }
            Commands::checkForPeriodicalActions(true);
            GCode::keepAlive(WaitHeater);
            //gcode_read_serial();
#if RETRACT_DURING_HEATUP
            if (actExtruder == Extruder::current && actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.currentTemperatureC > actExtruder->waitRetractTemperature)
            {
                PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
                retracted = 1;
            }
#endif
            if((waituntil == 0 &&
                    (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - 1
                     : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC + 1))
#if defined(TEMP_HYSTERESIS) && TEMP_HYSTERESIS>=1
                    || (waituntil != 0 && (abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)) > TEMP_HYSTERESIS)
#endif
              )
            {
                waituntil = currentTime + 1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabilize
            }
        }
        while(waituntil == 0 || (waituntil != 0 && (millis_t)(waituntil - currentTime) < 2000000000UL));
#if RETRACT_DURING_HEATUP
        if (retracted && actExtruder == Extruder::current)
        {
            PrintLine::moveRelativeDistanceInSteps(0, 0, 0, actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
        }
#endif
        EVENT_HEATING_FINISHED(actExtruder->id);
    }
    UI_CLEAR_STATUS;

    bool alloff = true;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;
#if EEPROM_MODE != 0
    if(alloff && !alloffs) // All heaters are now switched off?
        EEPROM::updatePrinterUsage();
#endif
    if(alloffs && !alloff)   // heaters are turned on, start measuring printing time
    {
        Printer::msecondsPrinting = HAL::timeInMilliseconds();
        Printer::filamentPrinted = 0;  // new print, new counter
        Printer::flag2 &= ~PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    }
#endif
}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius,bool beep)
{
    if(temperatureInCelsius > HEATED_BED_MAX_TEMP) temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if(temperatureInCelsius < 0) temperatureInCelsius = 0;
    if(heatedBedController.targetTemperatureC==temperatureInCelsius) return; // don't flood log with messages if killed
    heatedBedController.setTargetTemperature(temperatureInCelsius);
    if(beep && temperatureInCelsius>30) heatedBedController.setAlarm(true);
    Com::printFLN(Com::tTargetBedColon,heatedBedController.targetTemperatureC,0);
    if(temperatureInCelsius > 15)
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_SPEED;    // turn on the mainboard cooling fan
    else if(Printer::areAllSteppersDisabled())
        pwm_pos[PWM_BOARD_FAN] = 0;      // turn off the mainboard cooling fan only if steppers disabled
    EVENT_SET_BED_TEMP(temperatureInCelsius,beep);
}

float Extruder::getHeatedBedTemperature()
{
    TemperatureController *c = tempController[HEATED_BED_INDEX];
    return c->currentTemperatureC;
}

/** \brief Sends the high-signal to the stepper for next extruder step.
Call this function only, if interrupts are disabled.
*/
void Extruder::step()
{
#if NUM_EXTRUDER == 1
    WRITE(EXT0_STEP_PIN, START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
    TEST_EXTRUDER_JAM(0)
#endif
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
            WRITE(EXT1_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
            TEST_EXTRUDER_JAM(1)
#endif
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
                WRITE(EXT2_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
                TEST_EXTRUDER_JAM(2)
#endif
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
                WRITE(EXT3_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
                TEST_EXTRUDER_JAM(3)
#endif
            }
#endif
        }
#endif
#endif
        break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
        WRITE(EXT1_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
        WRITE(EXT2_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
        WRITE(EXT3_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER
        WRITE(EXT4_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,START_STEP_WITH_HIGH);
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER
        WRITE(EXT5_STEP2_PIN, START_STEP_WITH_HIGH);
#endif
#if EXTRUDER_JAM_CONTROL && defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
        TEST_EXTRUDER_JAM(5)
#endif
        break;
#endif
    }
#endif
}
/** \brief Sets stepper signal to low for current extruder.

Call this function only, if interrupts are disabled.
*/


void Extruder::unstep()
{
#if NUM_EXTRUDER == 1
    WRITE(EXT0_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
            WRITE(EXT1_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
                WRITE(EXT2_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
                WRITE(EXT3_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
            }
#endif // NUM_EXTRUDER > 3
        }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER > 0
        break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
        WRITE(EXT1_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
        WRITE(EXT2_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
        WRITE(EXT3_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER
        WRITE(EXT4_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,!START_STEP_WITH_HIGH);
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER
        WRITE(EXT5_STEP2_PIN, !START_STEP_WITH_HIGH);
#endif
        break;
#endif
    }
#endif
}
/** \brief Activates the extruder stepper and sets the direction. */
void Extruder::setDirection(uint8_t dir)
{
#if NUM_EXTRUDER == 1
    if(dir) {
        WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_DIR2_PIN, !EXT0_INVERSE2);
#endif
    } else {
        WRITE(EXT0_DIR_PIN, EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
        WRITE(EXT0_DIR2_PIN, EXT0_INVERSE2);
#endif
    }
    RESET_EXTRUDER_JAM(0, dir)
#else
    switch(Extruder::current->id)
    {
#if NUM_EXTRUDER > 0
    case 0:
        if(dir) {
            WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
            WRITE(EXT0_DIR2_PIN, !EXT0_INVERSE2);
#endif
        } else {
            WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
            WRITE(EXT0_DIR2_PIN, EXT0_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(0, dir)
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            if(dir) {
                WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
                WRITE(EXT1_DIR2_PIN, !EXT1_INVERSE2);
#endif
            } else {
                WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
                WRITE(EXT1_DIR2_PIN, EXT1_INVERSE2);
#endif
            }
            RESET_EXTRUDER_JAM(1, dir)
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                if(dir) {
                    WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
                    WRITE(EXT2_DIR2_PIN, !EXT2_INVERSE2);
#endif
                } else {
                    WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
                    WRITE(EXT2_DIR2_PIN, EXT2_INVERSE2);
#endif
                }
                RESET_EXTRUDER_JAM(2, dir)
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                if(dir) {
                    WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
                    WRITE(EXT3_DIR2_PIN, !EXT3_INVERSE2);
#endif
                } else {
                    WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
                    WRITE(EXT3_DIR2_PIN, EXT3_INVERSE2);
#endif
                }
                RESET_EXTRUDER_JAM(3, dir)
            }
#endif
        }
#endif
        break;
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
    case 1:
        if(dir) {
            WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
            WRITE(EXT1_DIR2_PIN, !EXT1_INVERSE2);
#endif
        } else {
            WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER
            WRITE(EXT1_DIR2_PIN, EXT1_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(1, dir)
        break;
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER > 2
    case 2:
        if(dir) {
            WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
            WRITE(EXT2_DIR2_PIN, !EXT2_INVERSE2);
#endif
        } else {
            WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER
            WRITE(EXT2_DIR2_PIN, EXT2_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(2, dir)
        break;
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER > 3
    case 3:
        if(dir) {
            WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
            WRITE(EXT3_DIR2_PIN, !EXT3_INVERSE2);
#endif
        } else {
            WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER
            WRITE(EXT3_DIR2_PIN, EXT3_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(3, dir)
        break;
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER > 4
    case 4:
        if(dir) {
            WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER
            WRITE(EXT4_DIR2_PIN, !EXT4_INVERSE2);
#endif
        } else {
            WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER
            WRITE(EXT4_DIR2_PIN, EXT4_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(4, dir)
        break;
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER > 5
    case 5:
        if(dir) {
            WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER
            WRITE(EXT5_DIR2_PIN, !EXT5_INVERSE2);
#endif
        } else {
            WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER
            WRITE(EXT5_DIR2_PIN, EXT5_INVERSE2);
#endif
        }
        RESET_EXTRUDER_JAM(5, dir)
        break;
#endif
    }
#endif
}

void Extruder::enable()
{
#if NUM_EXTRUDER == 1
#if EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN,EXT0_ENABLE_ON );
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER
    WRITE(EXT0_ENABLE2_PIN, EXT0_ENABLE_ON);
#endif
#endif
#else
#if NUM_EXTRUDER > 0
    switch(Extruder::current->id) {
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER && NUM_EXTRUDER > 0
        case 0:
            WRITE(EXT0_ENABLE2_PIN, EXT0_ENABLE_ON);
            break;
#endif
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER && NUM_EXTRUDER > 1
        case 1:
            WRITE(EXT1_ENABLE2_PIN, EXT1_ENABLE_ON);
            break;
#endif			
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER && NUM_EXTRUDER > 2
        case 2:
            WRITE(EXT2_ENABLE2_PIN, EXT2_ENABLE_ON);
            break;
#endif
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER && NUM_EXTRUDER > 3
        case 3:
            WRITE(EXT3_ENABLE2_PIN, EXT3_ENABLE_ON);
            break;
#endif
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER && NUM_EXTRUDER > 4
        case 4:
            WRITE(EXT4_ENABLE2_PIN, EXT4_ENABLE_ON);
            break;
#endif
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER && NUM_EXTRUDER > 5
        case 5:
            WRITE(EXT5_ENABLE2_PIN, EXT5_ENABLE_ON);
            break;
#endif
    }

    if(Extruder::current->enablePin > -1)
        digitalWrite(Extruder::current->enablePin,Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1) {
            digitalWrite(extruder[1].enablePin,extruder[1].enableOn);
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER && NUM_EXTRUDER > 1
            WRITE(EXT1_ENABLE2_PIN, EXT1_ENABLE_ON);
#endif
        }
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1) {
            digitalWrite(extruder[2].enablePin,extruder[2].enableOn);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER && NUM_EXTRUDER > 2
            WRITE(EXT2_ENABLE2_PIN, EXT2_ENABLE_ON);
#endif
        }
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1) {
            digitalWrite(extruder[3].enablePin,extruder[3].enableOn);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER && NUM_EXTRUDER > 3
            WRITE(EXT3_ENABLE2_PIN, EXT3_ENABLE_ON);
#endif
        }
#endif
    }
#endif	
#endif
#endif
}

void Extruder::disableCurrentExtruderMotor()
{
#if NUM_EXTRUDER > 0
    switch(Extruder::current->id) {
    #if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER && NUM_EXTRUDER > 0
    case 0:
        WRITE(EXT0_ENABLE2_PIN, !EXT0_ENABLE_ON);
        break;
    #endif
    #if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_ENABLE2_PIN, !EXT1_ENABLE_ON);
        break;
    #endif
    #if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_ENABLE2_PIN, !EXT2_ENABLE_ON);
        break;
    #endif
    #if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_ENABLE2_PIN, !EXT3_ENABLE_ON);
        break;
    #endif
    #if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_ENABLE2_PIN, !EXT4_ENABLE_ON);
        break;
    #endif
    #if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_ENABLE2_PIN, !EXT5_ENABLE_ON);
        break;
    #endif
}
    if(Extruder::current->enablePin > -1)
        HAL::digitalWrite(Extruder::current->enablePin,!Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1) {
            HAL::digitalWrite(extruder[1].enablePin,!extruder[1].enableOn);
    #if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER && NUM_EXTRUDER > 1
            WRITE(EXT1_ENABLE2_PIN, !EXT1_ENABLE_ON);
    #endif
        }
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1) {
            HAL::digitalWrite(extruder[2].enablePin,!extruder[2].enableOn);
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER && NUM_EXTRUDER > 2
            WRITE(EXT2_ENABLE2_PIN, !EXT2_ENABLE_ON);
#endif
        }
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1) {
            HAL::digitalWrite(extruder[3].enablePin,!extruder[3].enableOn);
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER && NUM_EXTRUDER > 3
            WRITE(EXT3_ENABLE2_PIN, !EXT3_ENABLE_ON);
#endif
        }
#endif
    }
#endif	
#endif
}

void Extruder::disableAllExtruderMotors()
{
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].enablePin > -1)
            HAL::digitalWrite(extruder[i].enablePin, !extruder[i].enableOn);
    }
#if defined(EXT0_MIRROR_STEPPER) && EXT0_MIRROR_STEPPER && NUM_EXTRUDER > 0
    WRITE(EXT0_ENABLE2_PIN, !EXT0_ENABLE_ON);
#endif
#if defined(EXT1_MIRROR_STEPPER) && EXT1_MIRROR_STEPPER && NUM_EXTRUDER > 1
    WRITE(EXT1_ENABLE2_PIN, !EXT1_ENABLE_ON);
#endif
#if defined(EXT2_MIRROR_STEPPER) && EXT2_MIRROR_STEPPER && NUM_EXTRUDER > 2
    WRITE(EXT2_ENABLE2_PIN, !EXT2_ENABLE_ON);
#endif
#if defined(EXT3_MIRROR_STEPPER) && EXT3_MIRROR_STEPPER && NUM_EXTRUDER > 3
    WRITE(EXT3_ENABLE2_PIN, !EXT3_ENABLE_ON);
#endif
#if defined(EXT4_MIRROR_STEPPER) && EXT4_MIRROR_STEPPER && NUM_EXTRUDER > 4
    WRITE(EXT4_ENABLE2_PIN, !EXT4_ENABLE_ON);
#endif
#if defined(EXT5_MIRROR_STEPPER) && EXT5_MIRROR_STEPPER && NUM_EXTRUDER > 5
    WRITE(EXT5_ENABLE2_PIN, !EXT5_ENABLE_ON);
#endif
}

uint8_t autotuneIndex = 255;
void Extruder::disableAllHeater()
{
#if NUM_TEMPERATURE_LOOPS > 0
    for(uint8_t i = 0; i <= HEATED_BED_INDEX; i++)
    {
        TemperatureController *c = tempController[i];
        c->targetTemperatureC = 0;
        pwm_pos[c->pwmIndex] = 0;
    }
#endif
    autotuneIndex = 255;
}

/** \brief Writes monitored temperatures.

This function is called every 250ms to write the monitored temperature. If monitoring is
disabled, the function is not called.
*/
void writeMonitor()
{
    Commands::printTemperatures(false);
}

bool reportTempsensorError()
{
#if NUM_TEMPERATURE_LOOPS > 0
    if(!Printer::isAnyTempsensorDefect()) return false;
    for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
    {
        if(i == NUM_EXTRUDER) Com::printF(Com::tHeatedBed);
        else if(i == HEATED_BED_INDEX) Com::printF(Com::tExtruderSpace,i);
        else Com::printF(PSTR("Other:"));
        TemperatureController *act = tempController[i];
        int temp = act->currentTemperatureC;
        if(temp < MIN_DEFECT_TEMPERATURE || temp > MAX_DEFECT_TEMPERATURE)
            Com::printF(Com::tTempSensorDefect);
        else
            Com::printF(Com::tTempSensorWorking);
        if(act->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT)
            Com::printF(PSTR(" marked defect"));	
        if(act->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED)
            Com::printF(PSTR(" marked decoupled"));
        Com::println();			
    }
    Com::printErrorFLN(Com::tDryModeUntilRestart);
    return true;
#else
    return false;
#endif
}

#ifdef SUPPORT_MAX6675

int16_t read_max6675(uint8_t ss_pin)
{
    static millis_t last_max6675_read = 0;
    static int16_t max6675_temp = 0;
    if (HAL::timeInMilliseconds() - last_max6675_read > 230)
    {
        HAL::spiInit(2);
        HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX6675
        HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine
        max6675_temp = HAL::spiReceive(0);
        max6675_temp <<= 8;
        max6675_temp |= HAL::spiReceive(0);
        HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX6675
        last_max6675_read = HAL::timeInMilliseconds();
    }
    return max6675_temp & 4 ? 2000 : max6675_temp >> 3; // thermocouple open?
}
#endif
#ifdef SUPPORT_MAX31855
int16_t read_max31855(uint8_t ss_pin)
{
    uint32_t data = 0;
    int16_t temperature;
    HAL::spiInit(2);
    HAL::digitalWrite(ss_pin, 0);  // enable TT_MAX31855
    HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine

    for (unsigned short byte = 0; byte < 4; byte++)
    {
        data <<= 8;
        data |= HAL::spiReceive();
    }

    HAL::digitalWrite(ss_pin, 1);  // disable TT_MAX31855

    //Process temp
    if (data & 0x00010000)
        return 20000; //Some form of error.
    else
    {
        data = data >> 18;
        temperature = data & 0x00001FFF;

        if (data & 0x00002000)
        {
            data = ~data;
            temperature = -1 * ((data & 0x00001FFF) + 1);
        }
    }
    return temperature;
}
#endif

#if FEATURE_RETRACTION
void Extruder::retractDistance(float dist)
{
    float oldFeedrate = Printer::feedrate;
    int32_t distance = static_cast<int32_t>(dist * stepsPerMM / Printer::extrusionFactor);
    int32_t oldEPos = Printer::currentPositionSteps[E_AXIS];
    float speed = distance > 0 ? EEPROM_FLOAT(RETRACTION_SPEED) : EEPROM_FLOAT(RETRACTION_UNDO_SPEED);
    PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -distance, RMath::max(speed, 1.f), false, false);
    Printer::currentPositionSteps[E_AXIS] = oldEPos; // restore previous extruder position
    Printer::feedrate = oldFeedrate;
}

void Extruder::retract(bool isRetract,bool isLong)
{
    float oldFeedrate = Printer::feedrate;
    float distance = (isLong ? EEPROM_FLOAT( RETRACTION_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_LENGTH));
    float zLiftF = EEPROM_FLOAT(RETRACTION_Z_LIFT);
    int32_t zlift = static_cast<int32_t>(zLiftF * Printer::axisStepsPerMM[Z_AXIS]);
    if(isRetract && !isRetracted())
    {
        retractDistance(distance);
        setRetracted(true);
        if(zlift > 0) {
            PrintLine::moveRelativeDistanceInStepsReal(0,0,zlift,0,Printer::maxFeedrate[Z_AXIS], false);
            Printer::coordinateOffset[Z_AXIS] -= zLiftF;
        }
    }
    else if(!isRetract && isRetracted())
    {
        distance += (isLong ? EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH) );
        if(zlift > 0) {
            PrintLine::moveRelativeDistanceInStepsReal(0,0,-zlift,0,Printer::maxFeedrate[Z_AXIS], false);
            Printer::coordinateOffset[Z_AXIS] += zLiftF;
        }
        retractDistance(-distance);
        setRetracted(false);
    }
    Printer::feedrate = oldFeedrate;
}
#endif

Extruder *Extruder::current;

#if NUM_EXTRUDER>0
const char ext0_select_cmd[] PROGMEM = EXT0_SELECT_COMMANDS;
const char ext0_deselect_cmd[] PROGMEM = EXT0_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>1
const char ext1_select_cmd[] PROGMEM = EXT1_SELECT_COMMANDS;
const char ext1_deselect_cmd[] PROGMEM = EXT1_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>2
const char ext2_select_cmd[] PROGMEM = EXT2_SELECT_COMMANDS;
const char ext2_deselect_cmd[] PROGMEM = EXT2_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>3
const char ext3_select_cmd[] PROGMEM = EXT3_SELECT_COMMANDS;
const char ext3_deselect_cmd[] PROGMEM = EXT3_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>4
const char ext4_select_cmd[] PROGMEM = EXT4_SELECT_COMMANDS;
const char ext4_deselect_cmd[] PROGMEM = EXT4_DESELECT_COMMANDS;
#endif
#if NUM_EXTRUDER>5
const char ext5_select_cmd[] PROGMEM = EXT5_SELECT_COMMANDS;
const char ext5_deselect_cmd[] PROGMEM = EXT5_DESELECT_COMMANDS;
#endif

#if NUM_EXTRUDER == 0
Extruder extruder[1];
#else
Extruder extruder[NUM_EXTRUDER] =
{
#if NUM_EXTRUDER > 0
    {
        0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_Z_OFFSET,EXT0_STEPS_PER_MM,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
        EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,EXT0_WATCHPERIOD
        ,EXT0_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT0_ADVANCE_K
#endif
        ,EXT0_ADVANCE_L,EXT0_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            0,EXT0_TEMPSENSOR_TYPE,EXT0_SENSOR_INDEX,EXT0_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_INTEGRAL_DRIVE_MIN,EXT0_PID_PGAIN_OR_DEAD_TIME,EXT0_PID_I,EXT0_PID_D,EXT0_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT0_DECOUPLE_TEST_PERIOD
        }
        ,ext0_select_cmd,ext0_deselect_cmd,EXT0_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 1
    ,{
        1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_Z_OFFSET,EXT1_STEPS_PER_MM,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
        EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,EXT1_WATCHPERIOD
        ,EXT1_WAIT_RETRACT_TEMP,EXT1_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT1_ADVANCE_K
#endif
        ,EXT1_ADVANCE_L,EXT1_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            1,EXT1_TEMPSENSOR_TYPE,EXT1_SENSOR_INDEX,EXT1_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_PGAIN_OR_DEAD_TIME,EXT1_PID_I,EXT1_PID_D,EXT1_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT1_DECOUPLE_TEST_PERIOD
        }
        ,ext1_select_cmd,ext1_deselect_cmd,EXT1_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 2
    ,{
        2,EXT2_X_OFFSET,EXT2_Y_OFFSET,EXT2_Z_OFFSET,EXT2_STEPS_PER_MM,EXT2_ENABLE_PIN,EXT2_ENABLE_ON,
        EXT2_MAX_FEEDRATE,EXT2_MAX_ACCELERATION,EXT2_MAX_START_FEEDRATE,0,EXT2_WATCHPERIOD
        ,EXT2_WAIT_RETRACT_TEMP,EXT2_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT2_ADVANCE_K
#endif
        ,EXT2_ADVANCE_L,EXT2_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            2,EXT2_TEMPSENSOR_TYPE,EXT2_SENSOR_INDEX,EXT2_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT2_PID_INTEGRAL_DRIVE_MAX,EXT2_PID_INTEGRAL_DRIVE_MIN,EXT2_PID_PGAIN_OR_DEAD_TIME,EXT2_PID_I,EXT2_PID_D,EXT2_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT2_DECOUPLE_TEST_PERIOD
        }
        ,ext2_select_cmd,ext2_deselect_cmd,EXT2_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 3
    ,{
        3,EXT3_X_OFFSET,EXT3_Y_OFFSET,EXT3_Z_OFFSET,EXT3_STEPS_PER_MM,EXT3_ENABLE_PIN,EXT3_ENABLE_ON,
        EXT3_MAX_FEEDRATE,EXT3_MAX_ACCELERATION,EXT3_MAX_START_FEEDRATE,0,EXT3_WATCHPERIOD
        ,EXT3_WAIT_RETRACT_TEMP,EXT3_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT3_ADVANCE_K
#endif
        ,EXT3_ADVANCE_L,EXT3_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            3,EXT3_TEMPSENSOR_TYPE,EXT3_SENSOR_INDEX,EXT3_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT3_PID_INTEGRAL_DRIVE_MAX,EXT3_PID_INTEGRAL_DRIVE_MIN,EXT3_PID_PGAIN_OR_DEAD_TIME,EXT3_PID_I,EXT3_PID_D,EXT3_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT3_DECOUPLE_TEST_PERIOD
        }
        ,ext3_select_cmd,ext3_deselect_cmd,EXT3_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 4
    ,{
        4,EXT4_X_OFFSET,EXT4_Y_OFFSET,EXT4_Z_OFFSET,EXT4_STEPS_PER_MM,EXT4_ENABLE_PIN,EXT4_ENABLE_ON,
        EXT4_MAX_FEEDRATE,EXT4_MAX_ACCELERATION,EXT4_MAX_START_FEEDRATE,0,EXT4_WATCHPERIOD
        ,EXT4_WAIT_RETRACT_TEMP,EXT4_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT4_ADVANCE_K
#endif
        ,EXT4_ADVANCE_L,EXT4_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            4,EXT4_TEMPSENSOR_TYPE,EXT4_SENSOR_INDEX,EXT4_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT4_PID_INTEGRAL_DRIVE_MAX,EXT4_PID_INTEGRAL_DRIVE_MIN,EXT4_PID_PGAIN_OR_DEAD_TIME,EXT4_PID_I,EXT4_PID_D,EXT4_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT4_DECOUPLE_TEST_PERIOD
        }
        ,ext4_select_cmd,ext4_deselect_cmd,EXT4_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
#if NUM_EXTRUDER > 5
    ,{
        5,EXT5_X_OFFSET,EXT5_Y_OFFSET,EXT5_Z_OFFSET,EXT5_STEPS_PER_MM,EXT5_ENABLE_PIN,EXT5_ENABLE_ON,
        EXT5_MAX_FEEDRATE,EXT5_MAX_ACCELERATION,EXT5_MAX_START_FEEDRATE,0,EXT5_WATCHPERIOD
        ,EXT5_WAIT_RETRACT_TEMP,EXT5_WAIT_RETRACT_UNITS
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        ,EXT5_ADVANCE_K
#endif
        ,EXT5_ADVANCE_L,EXT5_ADVANCE_BACKLASH_STEPS
#endif
        ,{
            5,EXT5_TEMPSENSOR_TYPE,EXT5_SENSOR_INDEX,EXT5_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
            ,0,EXT5_PID_INTEGRAL_DRIVE_MAX,EXT5_PID_INTEGRAL_DRIVE_MIN,EXT5_PID_PGAIN_OR_DEAD_TIME,EXT5_PID_I,EXT5_PID_D,EXT5_PID_MAX,0,0,0,{0,0,0,0}
#endif
            ,0,0,0,EXT5_DECOUPLE_TEST_PERIOD
        }
        ,ext5_select_cmd,ext5_deselect_cmd,EXT5_EXTRUDER_COOLER_SPEED,0,0,0
#if EXTRUDER_JAM_CONTROL
        ,0,0,10,0,0
#endif
    }
#endif
};
#endif // NUM_EXTRUDER

TemperatureController heatedBedController = {PWM_HEATED_BED,HEATED_BED_SENSOR_TYPE,BED_SENSOR_INDEX,HEATED_BED_HEAT_MANAGER,0,0,0,0
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0,{0,0,0,0}
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD};

#if FAN_THERMO_PIN > -1
TemperatureController thermoController = {PWM_FAN_THERMO,FAN_THERMO_THERMISTOR_TYPE,THERMO_ANALOG_INDEX,0,0,0,0,0
    #if TEMP_PID
    ,0,255,0,10,1,1,255,0,0,0,{0,0,0,0}
    #endif
,0,0,0,0};
#endif

#if NUM_TEMPERATURE_LOOPS > 0
TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] =
{
#if NUM_EXTRUDER > 0
    &extruder[0].tempControl
#endif
#if NUM_EXTRUDER > 1
    ,&extruder[1].tempControl
#endif
#if NUM_EXTRUDER > 2
    ,&extruder[2].tempControl
#endif
#if NUM_EXTRUDER > 3
    ,&extruder[3].tempControl
#endif
#if NUM_EXTRUDER > 4
    ,&extruder[4].tempControl
#endif
#if NUM_EXTRUDER > 5
    ,&extruder[5].tempControl
#endif
#if NUM_EXTRUDER == 0
    &heatedBedController
#else
    ,&heatedBedController
#endif
#if FAN_THERMO_PIN > -1
    ,&thermoController
#endif // FAN_THERMO_PIN
};
#endif
