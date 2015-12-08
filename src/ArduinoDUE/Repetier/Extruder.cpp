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
uint8_t counter250ms = 25;
#if FEATURE_DITTO_PRINTING
uint8_t Extruder::dittoMode = 0;
#endif
#if MIXING_EXTRUDER > 0
int Extruder::mixingS;
uint8_t Extruder::mixingDir = 10;
uint8_t Extruder::activeMixingExtruder = 0;
#endif // MIXING_EXTRUDER
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
                if(pwm_pos[NUM_EXTRUDER + 1]) enable = true;
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
#if MIXING_EXTRUDER
        if(controller > 0 && controller < NUM_EXTRUDER) continue; // Mixing extruder only test for ext 0
#endif // MIXING_EXTRUDER

        // Get Temperature
        act->updateCurrentTemperature();


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
#if HAVE_HEATED_BED
		else if(controller == NUM_EXTRUDER && Extruder::getHeatedBedTemperature() > HEATED_BED_MAX_TEMP + 5) {
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
#endif // HAVE_HEATED_BED
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

void TemperatureController::waitForTargetTemperature()
{
    if(targetTemperatureC < 30) return;
    if(Printer::debugDryrun()) return;
    millis_t time = HAL::timeInMilliseconds();
    while(true)
    {
        if( (HAL::timeInMilliseconds() - time) > 1000 )   //Print Temp Reading every 1 second while heating up.
        {
            Commands::printTemperatures();
            time = HAL::timeInMilliseconds();
        }
        Commands::checkForPeriodicalActions(true);
        if(fabs(targetTemperatureC - currentTemperatureC) <= 1)
            return;
    }
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
void TemperatureController::setJammed(bool on)
{
    if(on)
    {
        flags |= TEMPERATURE_CONTROLLER_FLAG_JAM;
        Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_DETECTED, true);
    }
    else flags &= ~(TEMPERATURE_CONTROLLER_FLAG_JAM);
}

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
#endif

void Extruder::initHeatedBed()
{
#if HAVE_HEATED_BED
#if TEMP_PID
    heatedBedController.updateTempControlVars();
#endif
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
#endif
#if defined(EXT1_STEP_PIN) && EXT1_STEP_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_DIR_PIN);
    SET_OUTPUT(EXT1_STEP_PIN);
#endif
#if defined(EXT2_STEP_PIN) && EXT2_STEP_PIN > -1 && NUM_EXTRUDER > 2
    SET_OUTPUT(EXT2_DIR_PIN);
    SET_OUTPUT(EXT2_STEP_PIN);
#endif
#if defined(EXT3_STEP_PIN) && EXT3_STEP_PIN > -1 && NUM_EXTRUDER > 3
    SET_OUTPUT(EXT3_DIR_PIN);
    SET_OUTPUT(EXT3_STEP_PIN);
#endif
#if defined(EXT4_STEP_PIN) && EXT4_STEP_PIN > -1 && NUM_EXTRUDER > 4
    SET_OUTPUT(EXT4_DIR_PIN);
    SET_OUTPUT(EXT4_STEP_PIN);
#endif
#if defined(EXT5_STEP_PIN) && EXT5_STEP_PIN > -1 && NUM_EXTRUDER > 5
    SET_OUTPUT(EXT5_DIR_PIN);
    SET_OUTPUT(EXT5_STEP_PIN);
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
            SET_OUTPUT(SS);
            WRITE(SS, HIGH);
            HAL::digitalWrite(act->tempControl.sensorPin, 1);
            HAL::pinMode(act->tempControl.sensorPin, OUTPUT);
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

void TemperatureController::updateTempControlVars()
{
#if TEMP_PID
    if(heatManager == HTR_PID && pidIGain != 0)   // prevent division by zero
    {
        tempIStateLimitMax = (float)pidDriveMax * 10.0f / pidIGain;
        tempIStateLimitMin = (float)pidDriveMin * 10.0f / pidIGain;
    }
#endif
}

/** \brief Select extruder ext_num.

This function changes and initializes a new extruder. This is also called, after the eeprom values are changed.
*/
void Extruder::selectExtruderById(uint8_t extruderId)
{
#if NUM_EXTRUDER > 0
#if MIXING_EXTRUDER
    if(extruderId >= VIRTUAL_EXTRUDER)
        extruderId = 0;
    activeMixingExtruder = extruderId;
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setMixingWeight(i, extruder[i].virtualWeights[extruderId]);
    extruderId = 0;
#endif
    if(extruderId >= NUM_EXTRUDER)
        extruderId = 0;
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
    bool executeSelect = false;
    if(extruderId != Extruder::current->id)
    {
        GCode::executeFString(Extruder::current->deselectCommands);
        executeSelect = true;
    }
    Commands::waitUntilEndOfAllMoves();
#endif
    Extruder::current->extrudePosition = Printer::currentPositionSteps[E_AXIS];
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
    float cx, cy, cz;
    Printer::realPosition(cx, cy, cz);
    float oldfeedrate = Printer::feedrate;
    Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
    Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply); // needed to adjust extrusionFactor to possibly different diameter
    if(Printer::isHomed())
        Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
    Printer::feedrate = oldfeedrate;
    Printer::updateCurrentPosition();
#if USE_ADVANCE
    HAL::resetExtruderDirection();
#endif

#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
    if(executeSelect) // Run only when changing
        GCode::executeFString(Extruder::current->selectCommands);
#endif
#endif
}

void Extruder::setTemperatureForExtruder(float temperatureInCelsius, uint8_t extr, bool beep, bool wait)
{
#if NUM_EXTRUDER > 0
#if MIXING_EXTRUDER
    extr = 0; // map any virtual extruder number to 0
#endif // MIXING_EXTRUDER
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
        bool dirRising = actExtruder->tempControl.targetTemperature > actExtruder->tempControl.currentTemperature;
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
                waituntil = currentTime + 1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabalize
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
#if HAVE_HEATED_BED
    if(temperatureInCelsius>HEATED_BED_MAX_TEMP) temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if(temperatureInCelsius<0) temperatureInCelsius = 0;
    if(heatedBedController.targetTemperatureC==temperatureInCelsius) return; // don't flood log with messages if killed
    heatedBedController.setTargetTemperature(temperatureInCelsius);
    if(beep && temperatureInCelsius>30) heatedBedController.setAlarm(true);
    Com::printFLN(Com::tTargetBedColon,heatedBedController.targetTemperatureC,0);
    if(temperatureInCelsius > 15)
        pwm_pos[NUM_EXTRUDER + 1] = 255;    // turn on the mainboard cooling fan
    else if(Printer::areAllSteppersDisabled())
        pwm_pos[NUM_EXTRUDER + 1] = 0;      // turn off the mainboard cooling fan only if steppers disabled
#endif
}

float Extruder::getHeatedBedTemperature()
{
#if HAVE_HEATED_BED
    TemperatureController *c = tempController[NUM_TEMPERATURE_LOOPS-1];
    return c->currentTemperatureC;
#else
    return -1;
#endif
}

#if MIXING_EXTRUDER > 0
void Extruder::setMixingWeight(uint8_t extr,int weight)
{
    uint8_t i;
    mixingS = 0;
    extruder[extr].mixingW = weight;
    for(i=0; i<NUM_EXTRUDER; i++)
    {
        extruder[i].mixingE = extruder[i].mixingW;
        mixingS += extruder[i].mixingW;
    }
}
void Extruder::step()
{
    if(PrintLine::cur != NULL && PrintLine::cur->isAllEMotors()) {
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
#endif
#if NUM_EXTRUDER > 1
        WRITE(EXT1_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
#endif
#if NUM_EXTRUDER > 2
        WRITE(EXT2_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
#endif
#if NUM_EXTRUDER > 3
        WRITE(EXT3_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
#endif
#if NUM_EXTRUDER > 4
        WRITE(EXT4_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
#endif
#if NUM_EXTRUDER > 5
        WRITE(EXT5_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
        TEST_EXTRUDER_JAM(5)
#endif
#endif
        return;
    }
    uint8_t best = 255,i;
    int bestError;
    if(mixingDir)
    {
        bestError = -10000;
        for(i = 0; i < NUM_EXTRUDER; i++)
        {
            if(extruder[i].mixingW == 0) continue;
            if(extruder[i].mixingE > bestError)
            {
                bestError = extruder[i].mixingE;
                best = i;
            }
            extruder[i].mixingE += extruder[i].mixingW;
        }
        if(best == 255) return; // no extruder has weight!
        extruder[best].mixingE -= mixingS;
    }
    else
    {
        bestError = 10000;
        for(i = 0; i < NUM_EXTRUDER; i++)
        {
            if(extruder[i].mixingW == 0) continue;
            if(extruder[i].mixingE < bestError)
            {
                bestError = extruder[i].mixingE;
                best = i;
            }
            extruder[i].mixingE -= extruder[i].mixingW;
        }
        if(best == 255) return; // no extruder has weight!
        extruder[best].mixingE += mixingS;
    }
#if NUM_EXTRUDER > 0
    if(best == 0)
    {
        WRITE(EXT0_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
    }
#endif
#if NUM_EXTRUDER > 1
    if(best == 1)
    {
        WRITE(EXT1_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
    }
#endif
#if NUM_EXTRUDER > 2
    if(best == 2)
    {
        WRITE(EXT2_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
    }
#endif
#if NUM_EXTRUDER > 3
    if(best == 3)
    {
        WRITE(EXT3_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
    }
#endif
#if NUM_EXTRUDER > 4
    if(best == 4)
    {
        WRITE(EXT4_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
    }
#endif
#if NUM_EXTRUDER > 5
    if(best == 5)
    {
        WRITE(EXT5_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
        TEST_EXTRUDER_JAM(5)
#endif
    }
#endif
}

void Extruder::unstep()
{
#if NUM_EXTRUDER > 0
    WRITE(EXT0_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 1
    WRITE(EXT1_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 2
    WRITE(EXT2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 3
    WRITE(EXT3_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 4
    WRITE(EXT4_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if NUM_EXTRUDER > 5
    WRITE(EXT5_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
}

void Extruder::setDirection(uint8_t dir)
{
    mixingDir = dir;
#if NUM_EXTRUDER > 0
    if(dir)
        WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
    else
        WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
    RESET_EXTRUDER_JAM(0, dir)
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
    if(dir)
        WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
    else
        WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
    RESET_EXTRUDER_JAM(1, dir)
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER > 2
    if(dir)
        WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
    else
        WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
    RESET_EXTRUDER_JAM(2, dir)
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER > 3
    if(dir)
        WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
    else
        WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
    RESET_EXTRUDER_JAM(3, dir)
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER > 4
    if(dir)
        WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
    else
        WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
    RESET_EXTRUDER_JAM(4, dir)
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER > 5
    if(dir)
        WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
    else
        WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
    RESET_EXTRUDER_JAM(5, dir)
#endif
}

void Extruder::enable()
{
#if NUM_EXTRUDER > 0 && defined(EXT0_ENABLE_PIN) && EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN, EXT0_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 1 && defined(EXT1_ENABLE_PIN) && EXT1_ENABLE_PIN > -1
    WRITE(EXT1_ENABLE_PIN, EXT1_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 2 && defined(EXT2_ENABLE_PIN) && EXT2_ENABLE_PIN > -1
    WRITE(EXT2_ENABLE_PIN, EXT2_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 3 && defined(EXT3_ENABLE_PIN) && EXT3_ENABLE_PIN > -1
    WRITE(EXT3_ENABLE_PIN, EXT3_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 4 && defined(EXT4_ENABLE_PIN) && EXT4_ENABLE_PIN > -1
    WRITE(EXT4_ENABLE_PIN, EXT4_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 5 && defined(EXT5_ENABLE_PIN) && EXT5_ENABLE_PIN > -1
    WRITE(EXT5_ENABLE_PIN, EXT5_ENABLE_ON );
#endif
}
#else // Normal extruder
/** \brief Sends the high-signal to the stepper for next extruder step.
Call this function only, if interrupts are disabled.
*/
void Extruder::step()
{
#if NUM_EXTRUDER == 1
    WRITE(EXT0_STEP_PIN, START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
    TEST_EXTRUDER_JAM(0)
#endif
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
        TEST_EXTRUDER_JAM(0)
#endif
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
            TEST_EXTRUDER_JAM(1)
#endif
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
                TEST_EXTRUDER_JAM(2)
#endif
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,START_STEP_WITH_HIGH);
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
#if EXTRUDER_JAM_CONTROL && defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
        TEST_EXTRUDER_JAM(1)
#endif
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
        TEST_EXTRUDER_JAM(2)
#endif
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
        TEST_EXTRUDER_JAM(3)
#endif
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,START_STEP_WITH_HIGH);
#if EXTRUDER_JAM_CONTROL && defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
        TEST_EXTRUDER_JAM(4)
#endif
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,START_STEP_WITH_HIGH);
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
#else
    switch(Extruder::current->id)
    {
    case 0:
#if NUM_EXTRUDER > 0
        WRITE(EXT0_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            WRITE(EXT1_STEP_PIN,!START_STEP_WITH_HIGH);
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                WRITE(EXT2_STEP_PIN,!START_STEP_WITH_HIGH);
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                WRITE(EXT3_STEP_PIN,!START_STEP_WITH_HIGH);
            }
#endif // NUM_EXTRUDER > 3
        }
#endif // FEATURE_DITTO_PRINTING
#endif // NUM_EXTRUDER > 0
        break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER > 1
    case 1:
        WRITE(EXT1_STEP_PIN,!START_STEP_WITH_HIGH);
        break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER > 2
    case 2:
        WRITE(EXT2_STEP_PIN,!START_STEP_WITH_HIGH);
        break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER > 3
    case 3:
        WRITE(EXT3_STEP_PIN,!START_STEP_WITH_HIGH);
        break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER > 4
    case 4:
        WRITE(EXT4_STEP_PIN,!START_STEP_WITH_HIGH);
        break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER > 5
    case 5:
        WRITE(EXT5_STEP_PIN,!START_STEP_WITH_HIGH);
        break;
#endif
    }
#endif
}
/** \brief Activates the extruder stepper and sets the direction. */
void Extruder::setDirection(uint8_t dir)
{
#if NUM_EXTRUDER == 1
    if(dir)
        WRITE(EXT0_DIR_PIN, !EXT0_INVERSE);
    else
        WRITE(EXT0_DIR_PIN, EXT0_INVERSE);
    RESET_EXTRUDER_JAM(0, dir)
#else
    switch(Extruder::current->id)
    {
#if NUM_EXTRUDER > 0
    case 0:
        if(dir)
            WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
        else
            WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
        RESET_EXTRUDER_JAM(0, dir)
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode)
        {
            if(dir)
                WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
            else
                WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
            RESET_EXTRUDER_JAM(1, dir)
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1)
            {
                if(dir)
                    WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
                else
                    WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
                RESET_EXTRUDER_JAM(2, dir)
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2)
            {
                if(dir)
                    WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
                else
                    WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
                RESET_EXTRUDER_JAM(3, dir)
            }
#endif
        }
#endif
        break;
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER > 1
    case 1:
        if(dir)
            WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
        else
            WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
        RESET_EXTRUDER_JAM(1, dir)
        break;
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER > 2
    case 2:
        if(dir)
            WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
        else
            WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
        RESET_EXTRUDER_JAM(2, dir)
        break;
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER > 3
    case 3:
        if(dir)
            WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
        else
            WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
        RESET_EXTRUDER_JAM(3, dir)
        break;
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER > 4
    case 4:
        if(dir)
            WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
        else
            WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
        RESET_EXTRUDER_JAM(4, dir)
        break;
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER > 5
    case 5:
        if(dir)
            WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
        else
            WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
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
#endif
#else
    if(Extruder::current->enablePin > -1)
        digitalWrite(Extruder::current->enablePin,Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1)
            digitalWrite(extruder[1].enablePin,extruder[1].enableOn);
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1)
            digitalWrite(extruder[2].enablePin,extruder[2].enableOn);
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1)
            digitalWrite(extruder[3].enablePin,extruder[3].enableOn);
#endif
    }
#endif
#endif
}

#endif  // MIXING_EXTRUDER > 0

void Extruder::disableCurrentExtruderMotor()
{
#if MIXING_EXTRUDER
#if NUM_EXTRUDER > 0 && defined(EXT0_ENABLE_PIN) && EXT0_ENABLE_PIN > -1
    WRITE(EXT0_ENABLE_PIN, !EXT0_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 1 && defined(EXT1_ENABLE_PIN) && EXT1_ENABLE_PIN > -1
    WRITE(EXT1_ENABLE_PIN, !EXT1_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 2 && defined(EXT2_ENABLE_PIN) && EXT2_ENABLE_PIN > -1
    WRITE(EXT2_ENABLE_PIN, !EXT2_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 3 && defined(EXT3_ENABLE_PIN) && EXT3_ENABLE_PIN > -1
    WRITE(EXT3_ENABLE_PIN, !EXT3_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 4 && defined(EXT4_ENABLE_PIN) && EXT4_ENABLE_PIN > -1
    WRITE(EXT4_ENABLE_PIN, !EXT4_ENABLE_ON );
#endif
#if NUM_EXTRUDER > 5 && defined(EXT5_ENABLE_PIN) && EXT5_ENABLE_PIN > -1
    WRITE(EXT5_ENABLE_PIN, !EXT5_ENABLE_ON );
#endif
#else // MIXING_EXTRUDER
    if(Extruder::current->enablePin > -1)
        HAL::digitalWrite(Extruder::current->enablePin,!Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1)
            HAL::digitalWrite(extruder[1].enablePin,!extruder[1].enableOn);
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1)
            HAL::digitalWrite(extruder[2].enablePin,!extruder[2].enableOn);
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1)
            HAL::digitalWrite(extruder[3].enablePin,!extruder[3].enableOn);
#endif
    }
#endif
#endif // MIXING_EXTRUDER
}
void Extruder::disableAllExtruderMotors()
{
    for(fast8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        if(extruder[i].enablePin > -1)
            HAL::digitalWrite(extruder[i].enablePin, !extruder[i].enableOn);
    }
}
#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM =
{
    {0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
    {365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
    {1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM =
{
    {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
    {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
    {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM =
{
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}
};

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM =
{
    {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
    {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
    {955*4, -11*8},{1008*4, -35*8}
};
// ATC 104GT
#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM =
{
    {0,8000},{69,2400},{79,2320},{92,2240},{107,2160},{125,2080},{146,2000},{172,1920},{204,1840},{222,1760},{291,1680},{350,1600},
    {422,1520},{511,1440},{621,1360},{755,1280},{918,1200},{1114,1120},{1344,1040},{1608,960},{1902,880},{2216,800},{2539,720},
    {2851,640},{3137,560},{3385,480},{3588,400},{3746,320},{3863,240},{3945,160},{4002,80},{4038,0},{4061,-80},{4075,-160}
};
#define NUMTEMPS_9 67 // 100k Honeywell 135-104LAG-J01
const short temptable_9[NUMTEMPS_9][2] PROGMEM =
{
    {1*4, 941*8},{19*4, 362*8},{37*4, 299*8}, //top rating 300C
    {55*4, 266*8},{73*4, 245*8},{91*4, 229*8},{109*4, 216*8},{127*4, 206*8},{145*4, 197*8},{163*4, 190*8},{181*4, 183*8},{199*4, 177*8},
    {217*4, 171*8},{235*4, 166*8},{253*4, 162*8},{271*4, 157*8},{289*4, 153*8},{307*4, 149*8},{325*4, 146*8},{343*4, 142*8},{361*4, 139*8},
    {379*4, 135*8},{397*4, 132*8},{415*4, 129*8},{433*4, 126*8},{451*4, 123*8},{469*4, 121*8},{487*4, 118*8},{505*4, 115*8},{523*4, 112*8},
    {541*4, 110*8},{559*4, 107*8},{577*4, 105*8},{595*4, 102*8},{613*4, 99*8},{631*4, 97*8},{649*4, 94*8},{667*4, 92*8},{685*4, 89*8},
    {703*4, 86*8},{721*4, 84*8},{739*4, 81*8},{757*4, 78*8},{775*4, 75*8},{793*4, 72*8},{811*4, 69*8},{829*4, 66*8},{847*4, 62*8},
    {865*4, 59*8},{883*4, 55*8},{901*4, 51*8},{919*4, 46*8},{937*4, 41*8},
    {955*4, 35*8},{973*4, 27*8},{991*4, 17*8},{1009*4, 1*8},{1023*4, 0}  //to allow internal 0 degrees C
};
#define NUMTEMPS_10 20 // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
const short temptable_10[NUMTEMPS_10][2] PROGMEM =
{
    {1*4, 704*8},{54*4, 216*8},{107*4, 175*8},{160*4, 152*8},{213*4, 137*8},{266*4, 125*8},{319*4, 115*8},{372*4, 106*8},{425*4, 99*8},
    {478*4, 91*8},{531*4, 85*8},{584*4, 78*8},{637*4, 71*8},{690*4, 65*8},{743*4, 58*8},{796*4, 50*8},{849*4, 42*8},{902*4, 31*8},
    {955*4, 17*8},{1008*4, 0}
};
#define NUMTEMPS_11 31 // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
const short temptable_11[NUMTEMPS_11][2] PROGMEM =
{
    {1*4, 936*8},{36*4, 300*8},{71*4, 246*8},{106*4, 218*8},{141*4, 199*8},{176*4, 185*8},{211*4, 173*8},{246*4, 163*8},{281*4, 155*8},
    {316*4, 147*8},{351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},
    {631*4, 97*8},{666*4, 92*8},{701*4, 87*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},
    {946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0}
};
#define NUMTEMPS_12 31 // 100k RS thermistor 198-961 (4.7k pullup)
const short temptable_12[NUMTEMPS_12][2] PROGMEM =
{
    {1*4, 929*8},{36*4, 299*8},{71*4, 246*8},{106*4, 217*8},{141*4, 198*8},{176*4, 184*8},{211*4, 173*8},{246*4, 163*8},{281*4, 154*8},{316*4, 147*8},
    {351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},{631*4, 97*8},{666*4, 91*8},
    {701*4, 86*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},{946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0*8}
};
#define NUMTEMPS_13 19
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
    {0,0},{908,8},{942,10*8},{982,20*8},{1015,8*30},{1048,8*40},{1080,8*50},{1113,8*60},{1146,8*70},{1178,8*80},{1211,8*90},{1276,8*110},{1318,8*120}
    ,{1670,8*230},{2455,8*500},{3445,8*900},{3666,8*1000},{3871,8*1100},{4095,8*2000}
};
#if NUM_TEMPS_USERTHERMISTOR0 > 0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif
const short * const temptables[13] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0 > 0
        ,(short int *)&temptable_5[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
        ,(short int *)&temptable_6[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
        ,(short int *)&temptable_7[0][0]
#else
        ,0
#endif
        ,(short int *)&temptable_8[0][0]
        ,(short int *)&temptable_9[0][0]
        ,(short int *)&temptable_10[0][0]
        ,(short int *)&temptable_11[0][0]
        ,(short int *)&temptable_12[0][0]
        ,(short int *)&temptable_13[0][0]
                                             };
const uint8_t temptables_num[13] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR0,NUM_TEMPS_USERTHERMISTOR1,NUM_TEMPS_USERTHERMISTOR2,NUMTEMPS_8,
                                 NUMTEMPS_9,NUMTEMPS_10,NUMTEMPS_11,NUMTEMPS_12,NUMTEMPS_13
                                           };


void TemperatureController::updateCurrentTemperature()
{
    uint8_t type = sensorType;
    // get raw temperature
    switch(type)
    {
    case 0:
        currentTemperature = 25;
        break;
#if ANALOG_INPUTS>0
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 97:
    case 98:
    case 99:
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS)); // Convert to 10 bit result
        break;
    case 13: // PT100 E3D
    case 50: // User defined PTC table
    case 51:
    case 52:
    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
    case 61: // HEATER_USES_AD8495 (Delivers 5mV/degC) 1.25v offset
    case 100: // AD595 / AD597
        currentTemperature = (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS));
        break;
#endif
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
        currentTemperature = read_max6675(sensorPin);
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102: // MAX31855
        currentTemperature = read_max31855(sensorPin);
        break;
#endif
    default:
        currentTemperature = 4095; // unknown method, return high value to switch heater off for safety
    }
    int currentTemperature = this->currentTemperature;
    //OUT_P_I_LN("OC for raw ",raw_temp);
    switch(type)
    {
    case 0:
        currentTemperatureC = 25;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    {
        type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        int16_t oldraw = pgm_read_word(&temptable[0]);
        int16_t oldtemp = pgm_read_word(&temptable[1]);
        int16_t newraw,newtemp = 0;
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;
        while(i < num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature)
            {
                //OUT_P_I("RC O:",oldtemp);OUT_P_I_LN(" OR:",oldraw);
                //OUT_P_I("RC N:",newtemp);OUT_P_I_LN(" NR:",newraw);
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp - oldtemp) / (newraw - oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
    }
    break;
    case 13:
    case 50: // User defined PTC thermistor
    case 51:
    case 52:
    {
        if(type > 49)
            type -= 46;
        else
            type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
        int16_t oldraw = pgm_read_word(&temptable[0]);
        int16_t oldtemp = pgm_read_word(&temptable[1]);
        int16_t newraw,newtemp = 0;
        while(i < num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature)
            {
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
    case 60: // AD8495 (Delivers 5mV/degC vs the AD595's 10mV)
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#else
        currentTemperatureC = ((float)currentTemperature * 660.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#endif
        break;
    case 61: // AD8495 1.25V Vref offset (like Adafruit 8495 breakout board)
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 250.0f;
#else
        currentTemperatureC = ((float)currentTemperature * 660.0f / (1024 << (2 - ANALOG_REDUCE_BITS))) - 250.0f;
#endif
        break;
    case 100: // AD595 / AD597   10mV/°C
        //return (int)((long)raw_temp * 500/(1024<<(2-ANALOG_REDUCE_BITS)));
#if CPU_ARCH == ARCH_AVR
        currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#else
#if FEATURE_CONTROLLER == CONTROLLER_LCD_MP_PHARAOH_DUE
        currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#else
        currentTemperatureC = ((float)currentTemperature * 330.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#endif
#endif
        break;
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
        currentTemperatureC = (float)currentTemperature / 4.0;
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102: // MAX31855
        currentTemperatureC = (float)currentTemperature / 4.0;
        break;
#endif
#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    case 97:
    case 98:
    case 99:
    {
        uint8_t i=2;
        const short *temptable;
#ifdef USE_GENERIC_THERMISTORTABLE_1
        if(type == 97)
            temptable = (const short *)temptable_generic1;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
        if(type == 98)
            temptable = (const short *)temptable_generic2;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
        if(type == 99)
            temptable = (const short *)temptable_generic3;
#endif
        short oldraw = temptable[0];
        short oldtemp = temptable[1];
        short newraw,newtemp;
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;
        while(i<GENERIC_THERM_NUM_ENTRIES*2)
        {
            newraw = temptable[i++];
            newtemp = temptable[i++];
            if (newraw > currentTemperature)
            {
                //OUT_P_I("RC O:",oldtemp);OUT_P_I_LN(" OR:",oldraw);
                //OUT_P_I("RC N:",newtemp);OUT_P_I_LN(" NR:",newraw);
                currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
        break;
    }
#endif
    }
}

void TemperatureController::setTargetTemperature(float target)
{
    targetTemperatureC = target;
    stopDecouple();
    int temp = TEMP_FLOAT_TO_INT(target);
    uint8_t type = sensorType;
    switch(sensorType)
    {
    case 0:
        targetTemperature = 0;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    {
        type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw = 0,newtemp;
        while(i<num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newtemp < temp)
            {
                targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS))- oldraw + (int32_t)(oldtemp - temp) * (int32_t)(oldraw - newraw) / (oldtemp - newtemp);
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
        break;
    }
    case 13: // PT100 E3D
    case 50: // user defined PTC thermistor
    case 51:
    case 52:
    {
        if(type > 49)
            type -= 46;
        else
            type--;
        uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
        uint8_t i = 2;
        const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
        short oldraw = pgm_read_word(&temptable[0]);
        short oldtemp = pgm_read_word(&temptable[1]);
        short newraw = 0,newtemp;
        while(i < num)
        {
            newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newtemp > temp)
            {
                targetTemperature = oldraw + (int32_t)(oldtemp - temp) * (int32_t)(oldraw - newraw) / (oldtemp-newtemp);
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = newraw;
        break;
    }
    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
        targetTemperature = (int)((int32_t)temp * (1024 << (2 - ANALOG_REDUCE_BITS))/ 1000);
        break;
    case 100: // HEATER_USES_AD595
        targetTemperature = (int)((int32_t)temp * (1024 << (2 - ANALOG_REDUCE_BITS))/ 500);
        break;
#ifdef SUPPORT_MAX6675
    case 101:  // defined HEATER_USES_MAX6675
        targetTemperature = temp * 4;
        break;
#endif
#ifdef SUPPORT_MAX31855
    case 102:  // defined HEATER_USES_MAX31855
        targetTemperature = temp * 4;
        break;
#endif
#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    case 97:
    case 98:
    case 99:
    {
        uint8_t i = 2;
        const short *temptable;
#ifdef USE_GENERIC_THERMISTORTABLE_1
        if(type == 97)
            temptable = (const short *)temptable_generic1;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
        if(type == 98)
            temptable = (const short *)temptable_generic2;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
        if(type == 99)
            temptable = (const short *)temptable_generic3;
#endif
        short oldraw = temptable[0];
        short oldtemp = temptable[1];
        short newraw,newtemp;
        while(i<GENERIC_THERM_NUM_ENTRIES*2)
        {
            newraw = temptable[i++];
            newtemp = temptable[i++];
            if (newtemp < temp)
            {
                targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - oldraw + (int32_t)(oldtemp-temp) * (int32_t)(oldraw-newraw) / (oldtemp-newtemp);
                return;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        targetTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - newraw;
        break;
    }
#endif
    }
}

uint8_t autotuneIndex = 255;
void Extruder::disableAllHeater()
{
#if NUM_TEMPERATURE_LOOPS > 0
    for(uint8_t i = 0; i < NUM_TEMPERATURE_LOOPS; i++)
    {
        TemperatureController *c = tempController[i];
        c->targetTemperature = 0;
        c->targetTemperatureC = 0;
        pwm_pos[c->pwmIndex] = 0;
    }
#endif
    autotuneIndex = 255;
}

#if TEMP_PID
void TemperatureController::autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeValues)
{
    float currentTemp;
    int cycles = 0;
    bool heating = true;

    uint32_t temp_millis = HAL::timeInMilliseconds();
    uint32_t t1 = temp_millis;
    uint32_t t2 = temp_millis;
    int32_t t_high = 0;
    int32_t t_low;

    int32_t bias = pidMax >> 1;
    int32_t d = pidMax >> 1;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp = 20, minTemp = 20;
    if(maxCycles < 5)
        maxCycles = 5;
    if(maxCycles > 20)
        maxCycles = 20;
    Com::printInfoFLN(Com::tPIDAutotuneStart);

    Extruder::disableAllHeater(); // switch off all heaters.
    autotuneIndex = controllerId;
    pwm_pos[pwmIndex] = pidMax;
    if(controllerId<NUM_EXTRUDER)
    {
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }
    for(;;)
    {
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        updateCurrentTemperature();
        currentTemp = currentTemperatureC;
        unsigned long time = HAL::timeInMilliseconds();
        maxTemp = RMath::max(maxTemp,currentTemp);
        minTemp = RMath::min(minTemp,currentTemp);
        if(heating == true && currentTemp > temp)   // switch heating -> off
        {
            if(time - t2 > (controllerId < NUM_EXTRUDER ? 2500 : 1500))
            {
                heating=false;
                pwm_pos[pwmIndex] = (bias - d);
                t1 = time;
                t_high = t1 - t2;
                maxTemp=temp;
            }
        }
        if(heating == false && currentTemp < temp)
        {
            if(time - t1 > (controllerId < NUM_EXTRUDER ? 5000 : 3000))
            {
                heating = true;
                t2 = time;
                t_low=t2 - t1; // half wave length
                if(cycles > 0)
                {
                    bias += (d*(t_high - t_low))/(t_low + t_high);
                    bias = constrain(bias, 20 ,pidMax - 20);
                    if(bias > pidMax/2) d = pidMax - 1 - bias;
                    else d = bias;

                    Com::printF(Com::tAPIDBias,bias);
                    Com::printF(Com::tAPIDD,d);
                    Com::printF(Com::tAPIDMin,minTemp);
                    Com::printFLN(Com::tAPIDMax,maxTemp);
                    if(cycles > 2)
                    {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d) / (3.14159*(maxTemp-minTemp));
                        Tu = ((float)(t_low + t_high)/1000.0);
                        Com::printF(Com::tAPIDKu,Ku);
                        Com::printFLN(Com::tAPIDTu,Tu);
                        Kp = 0.6*Ku;
                        Ki = 2*Kp/Tu;
                        Kd = Kp*Tu*0.125;
                        Com::printFLN(Com::tAPIDClassic);
                        Com::printFLN(Com::tAPIDKp,Kp);
                        Com::printFLN(Com::tAPIDKi,Ki);
                        Com::printFLN(Com::tAPIDKd,Kd);
                        /*
                        Kp = 0.33*Ku;
                        Ki = Kp/Tu;
                        Kd = Kp*Tu/3;
                        OUT_P_LN(" Some overshoot");
                        OUT_P_F_LN(" Kp: ",Kp);
                        OUT_P_F_LN(" Ki: ",Ki);
                        OUT_P_F_LN(" Kd: ",Kd);
                        Kp = 0.2*Ku;
                        Ki = 2*Kp/Tu;
                        Kd = Kp*Tu/3;
                        OUT_P_LN(" No overshoot");
                        OUT_P_F_LN(" Kp: ",Kp);
                        OUT_P_F_LN(" Ki: ",Ki);
                        OUT_P_F_LN(" Kd: ",Kd);
                        */
                    }
                }
                pwm_pos[pwmIndex] = (bias + d);
                cycles++;
                minTemp=temp;
            }
        }
        if(currentTemp > (temp + 40))
        {
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            Extruder::disableAllHeater();
            return;
        }
        if(time - temp_millis > 1000)
        {
            temp_millis = time;
            Commands::printTemperatures();
        }
        if(((time - t1) + (time - t2)) > (10L*60L*1000L*2L))   // 20 Minutes
        {
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            Extruder::disableAllHeater();
            return;
        }
        if(cycles > maxCycles)
        {
            Com::printInfoFLN(Com::tAPIDFinished);
            Extruder::disableAllHeater();
            if(storeValues)
            {
                pidPGain = Kp;
                pidIGain = Ki;
                pidDGain = Kd;
                heatManager = HTR_PID;
                EEPROM::storeDataIntoEEPROM();
            }
            return;
        }
        UI_MEDIUM;
        UI_SLOW(true);
    }
}
#endif

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
        else Com::printF(Com::tExtruderSpace,i);
        int temp = tempController[i]->currentTemperatureC;
        if(temp < MIN_DEFECT_TEMPERATURE || temp > MAX_DEFECT_TEMPERATURE)
            Com::printFLN(Com::tTempSensorDefect);
        else
            Com::printFLN(Com::tTempSensorWorking);
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
#if MIXING_EXTRUDER
            Printer::setAllEMotors(true);
#endif
    PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -distance, RMath::max(speed, 1.f), false, false);
#if MIXING_EXTRUDER
            Printer::setAllEMotors(false);
#endif
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            0,EXT0_TEMPSENSOR_TYPE,EXT0_SENSOR_INDEX,0,0,0,0,0,EXT0_HEAT_MANAGER
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            1,EXT1_TEMPSENSOR_TYPE,EXT1_SENSOR_INDEX,0,0,0,0,0,EXT1_HEAT_MANAGER
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            2,EXT2_TEMPSENSOR_TYPE,EXT2_SENSOR_INDEX,0,0,0,0,0,EXT2_HEAT_MANAGER
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            3,EXT3_TEMPSENSOR_TYPE,EXT3_SENSOR_INDEX,0,0,0,0,0,EXT3_HEAT_MANAGER
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            4,EXT4_TEMPSENSOR_TYPE,EXT4_SENSOR_INDEX,0,0,0,0,0,EXT4_HEAT_MANAGER
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
#if MIXING_EXTRUDER > 0
        ,10,10,{10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10}
#endif
        ,{
            5,EXT5_TEMPSENSOR_TYPE,EXT5_SENSOR_INDEX,0,0,0,0,0,EXT5_HEAT_MANAGER
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

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
TemperatureController heatedBedController = {NUM_EXTRUDER,HEATED_BED_SENSOR_TYPE,BED_SENSOR_INDEX,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#if TEMP_PID
        ,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN_OR_DEAD_TIME,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0,{0,0,0,0}
#endif
        ,0,0,0,HEATED_BED_DECOUPLE_TEST_PERIOD
                                            };
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif

#if NUM_TEMPERATURE_LOOPS > 0
TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] =
{
#if NUM_EXTRUDER>0
    &extruder[0].tempControl
#endif
#if NUM_EXTRUDER>1
    ,&extruder[1].tempControl
#endif
#if NUM_EXTRUDER>2
    ,&extruder[2].tempControl
#endif
#if NUM_EXTRUDER>3
    ,&extruder[3].tempControl
#endif
#if NUM_EXTRUDER>4
    ,&extruder[4].tempControl
#endif
#if NUM_EXTRUDER>5
    ,&extruder[5].tempControl
#endif
#if HAVE_HEATED_BED
#if NUM_EXTRUDER==0
    &heatedBedController
#else
    ,&heatedBedController
#endif
#endif
};
#endif
