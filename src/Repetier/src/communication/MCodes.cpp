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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42

void MCode_3(GCode* com) { // Spindle CW on, laser intensity
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M3(com);
    }
}

void MCode_4(GCode* com) { // Spindle CCW, laser intensity
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M4(com);
    }
}

void MCode_5(GCode* com) { // Spindle, laser off
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M5(com);
    }
}

void MCode_6(GCode* com) { // Tool Change
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M6(com);
    }
}

void MCode_7(GCode* com) { // Mist cooland On
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M7(com);
    }
}

void MCode_8(GCode* com) { // Flood Coolant on
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M8(com);
    }
}

void MCode_9(GCode* com) { // Coolant off
    Tool* t = Tool::getActiveTool();
    if (t) {
        t->M9(com);
    }
}

void MCode_17(GCode* com) {
    Motion1::waitForEndOfMoves();
    bool named = false;
    if (com->hasX()) {
        named = true;
        Motion1::motors[X_AXIS]->enable();
    }
    if (com->hasY()) {
        named = true;
        Motion1::motors[Y_AXIS]->enable();
    }
    if (com->hasZ()) {
        named = true;
        Motion1::motors[Z_AXIS]->enable();
    }
    if (com->hasE() && Motion1::motors[E_AXIS] != nullptr) {
        named = true;
        if (Motion1::motors[E_AXIS]) {
            Motion1::motors[E_AXIS]->enable();
        }
    }
    if (!named) {
        Motion1::motors[X_AXIS]->enable();
        Motion1::motors[Y_AXIS]->enable();
        Motion1::motors[Z_AXIS]->enable();
        for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
            if (Motion1::motors[i]) {
                Motion1::motors[i]->enable();
            }
        }
        Tool::enableMotors();
    }
    Printer::unsetAllSteppersDisabled();
}

void MCode_18(GCode* com) {
    Motion1::waitForEndOfMoves();
    bool named = false;
    if (com->hasX()) {
        named = true;
        Motion1::motors[X_AXIS]->disable();
    }
    if (com->hasY()) {
        named = true;
        Motion1::motors[Y_AXIS]->disable();
    }
    if (com->hasZ()) {
        named = true;
        Motion1::motors[Z_AXIS]->disable();
    }
    if (com->hasE() && Motion1::motors[E_AXIS] != nullptr) {
        named = true;
        if (Motion1::motors[E_AXIS]) {
            Motion1::motors[E_AXIS]->disable();
        }
    }
    if (!named) {
        Motion1::motors[X_AXIS]->disable();
        Motion1::motors[Y_AXIS]->disable();
        Motion1::motors[Z_AXIS]->disable();
        for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
            if (Motion1::motors[i]) {
                Motion1::motors[i]->disable();
            }
        }
        Tool::disableMotors();
    }
}

void MCode_20(GCode* com) {
#if SDSUPPORT
#if JSON_OUTPUT
    if (com->hasString() && com->text[1] == '2') { // " S2 P/folder"
        if (com->text[3] == 'P') {
            char* slashPos = strchr(com->text, '/');
            if (*slashPos) {
                sd.lsJSON(slashPos);
            } else {
                sd.lsJSON(com->text + 4);
            }
        }
    } else
        sd.ls();
#else
    sd.ls();
#endif
#endif
}

void MCode_21(GCode* com) {
#if SDSUPPORT
    sd.mount();
#endif
}

void MCode_22(GCode* com) {
#if SDSUPPORT
    sd.unmount();
#endif
}

void MCode_23(GCode* com) {
#if SDSUPPORT
    if (com->hasString()) {
        sd.fat.chdir();
        sd.selectFile(com->text);
    }
#endif
}

void MCode_24(GCode* com) {
#if SDSUPPORT
    sd.startPrint();
#endif
}

void MCode_25(GCode* com) {
#if SDSUPPORT
    sd.pausePrint();
#endif
}

void MCode_26(GCode* com) {
#if SDSUPPORT
    if (com->hasS())
        sd.setIndex(com->S);
#endif
}

void MCode_27(GCode* com) {
#if SDSUPPORT
    sd.printStatus();
#endif
}

void MCode_28(GCode* com) {
#if SDSUPPORT
    if (com->hasString())
        sd.startWrite(com->text);
#endif
}

void MCode_29(GCode* com) {
#if SDSUPPORT
//processed in write to file routine above
//savetosd = false;
#endif
}

void MCode_30(GCode* com) {
#if SDSUPPORT
    if (com->hasString()) {
        sd.fat.chdir();
        sd.deleteFile(com->text);
    }
#endif
}

void MCode_32(GCode* com) {
#if SDSUPPORT
    if (com->hasString()) {
        sd.fat.chdir();
        sd.makeDirectory(com->text);
    }
#endif
}

void MCode_36(GCode* com) {
#if JSON_OUTPUT && SDSUPPORT
    if (com->hasString()) {
        sd.JSONFileInfo(com->text);
    }
#endif
}

void MCode_42(GCode* com) {
    // Tool::getTool(com->T)->unstepMotor();
    // return;
    if (com->hasP()) {
        int pin_number = com->P;
        for (uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
            if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
                pin_number = -1;
                return;
            }
        }
        if (pin_number > -1) {
            if (com->hasS()) {
                if (com->S >= 0 && com->S <= 255) {
                    pinMode(pin_number, OUTPUT);
                    digitalWrite(pin_number, com->S);
                    // analogWrite(pin_number, com->S); // conflicts often with hardware pwm!
                    Com::printF(Com::tSetOutputSpace, pin_number);
                    Com::printFLN(Com::tSpaceToSpace, (int)com->S);
                } else
                    Com::printErrorFLN(PSTR("Illegal S value for M42"));
            } else {
                pinMode(pin_number, INPUT_PULLUP);
                Com::printF(Com::tSpaceToSpace, pin_number);
                Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
            }
        } else {
            Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
        }
    }
}

// Test z probe accuracy
void MCode_48(GCode* com) {
    if (!Printer::isHomedAll()) {
        Com::printWarningFLN(PSTR("Printer not homed!"));
        return;
    }
    bool oldDist = Leveling::isDistortionEnabled();
    Leveling::setDistortionEnabled(false);
    Motion1::setTmpPositionXYZ(com->hasX() ? com->X : Motion1::currentPosition[X_AXIS],
                               com->hasY() ? com->Y : Motion1::currentPosition[Y_AXIS], ZProbeHandler::optimumProbingHeight());
    PrinterType::closestAllowedPositionWithNewXYOffset(Motion1::tmpPosition, ZProbeHandler::xOffset(), ZProbeHandler::yOffset(), Z_PROBE_BORDER);
    Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    int n = com->hasP() ? com->P : 10;
    ZProbeHandler::activate();
    float sum = 0, minH = 1000, maxH = -1000, A = 0, Q = 0;
    bool ok = true;
    for (int i = 0; i < n; i++) {
        float z = ZProbeHandler::runProbe();
        if (z == ILLEGAL_Z_PROBE) {
            ok = false;
            break;
        }
        sum += z;
        minH = RMath::min(z, minH);
        maxH = RMath::max(z, maxH);
        float lastA = A;
        A = lastA + (z - lastA) / static_cast<float>(i + 1);
        Q += (z - lastA) * (z - A);
    }
    ZProbeHandler::deactivate();
    Leveling::setDistortionEnabled(oldDist);
    if (ok) {
        Q = sqrtf(Q / n);
        Com::printFLN(PSTR("Average: "), A, 4);
        Com::printFLN(PSTR("Min: "), minH, 4);
        Com::printFLN(PSTR("Max: "), maxH, 4);
        Com::printFLN(PSTR("Std. deviation: "), Q, 4);
    } else {
        Com::printFLN(PSTR("Aborted due to illegal measurement!"));
    }
}

void MCode_80(GCode* com) {
#if PS_ON_PIN > -1
    Motion1::waitForEndOfMoves();
    previousMillisCmd = HAL::timeInMilliseconds();
    SET_OUTPUT(PS_ON_PIN); //GND
    Printer::setPowerOn(true);
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif
}

void MCode_81(GCode* com) {
#if PS_ON_PIN > -1
    Motion1::waitForEndOfMoves();
    SET_OUTPUT(PS_ON_PIN); //GND
    Printer::setPowerOn(false);
    WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif
}

void MCode_82(GCode* com) {
    Printer::relativeExtruderCoordinateMode = false;
}

void MCode_83(GCode* com) {
    Printer::relativeExtruderCoordinateMode = true;
}

void MCode_84(GCode* com) {
    if (com->hasS()) {
        stepperInactiveTime = com->S * 1000;
    } else {
        Motion1::waitForEndOfMoves();
        Printer::kill(true);
    }
}

void MCode_85(GCode* com) {
    if (com->hasS())
        maxInactiveTime = (int32_t)com->S * 1000;
    else
        maxInactiveTime = 0;
}

void MCode_92(GCode* com) {
    Motion1::fillPosFromGCode(*com, Motion1::resolution, Motion1::resolution);
    Printer::updateDerivedParameter();
    if (com->hasE()) {
        Tool::getActiveTool()->setResolution(com->E);
    }
}

void MCode_99(GCode* com) {
    millis_t wait = 10000;
    if (com->hasS()) {
        wait = 1000 * com->S;
    }
    if (com->hasX()) {
        Motion1::motors[X_AXIS]->disable();
    }
    if (com->hasY()) {
        Motion1::motors[Y_AXIS]->disable();
    }
    if (com->hasZ()) {
        Motion1::motors[Z_AXIS]->disable();
    }
    wait += HAL::timeInMilliseconds();
#ifdef DEBUG_PRINT
    debugWaitLoop = 2;
#endif
    while (wait - HAL::timeInMilliseconds() < 100000) {
        Printer::defaultLoopActions();
    }
    if (com->hasX()) {
        Motion1::motors[X_AXIS]->enable();
    }
    if (com->hasY()) {
        Motion1::motors[Y_AXIS]->enable();
    }
    if (com->hasZ()) {
        Motion1::motors[Z_AXIS]->enable();
    }
    Printer::unsetAllSteppersDisabled();
}

void MCode_104(GCode* com) {
#if NUM_TOOLS > 0
    previousMillisCmd = HAL::timeInMilliseconds();
    if (HeatManager::reportTempsensorError() || Printer::debugDryrun()) {
        return;
    }
#ifdef EXACT_TEMPERATURE_TIMING
    Motion1::waitForEndOfMoves();
#else
    if (com->hasP() || (com->hasS() && com->S == 0)) {
        Motion1::waitForEndOfMoves();
    }
#endif
    Tool* tool = Tool::getActiveTool();
    if (com->hasT()) {
        tool = Tool::getTool(com->T);
    }
    if (tool == nullptr) {
        return;
    }
    if (Motion1::dittoMode) {
        for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
            tool = Tool::getTool(i);
            if (com->hasS()) {
                tool->getHeater()->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasP()) {
                tool->getHeater()->setTargetTemperature(tool->getHeater()->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
        }
    } else {
        if (com->hasS()) {
            tool->getHeater()->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
        } else if (com->hasP()) {
            tool->getHeater()->setTargetTemperature(tool->getHeater()->getPreheatTemperature() + (com->hasO() ? com->O : 0));
        }
    }
#endif // NUM_TOOLS > 0
}

void MCode_105(GCode* com) {
    Com::writeToAll = false;
    Commands::printTemperatures(com->hasX());
}

void MCode_106(GCode* com) {
    if (com->hasI()) {
        if (com->I != 0) {
            Printer::flag2 |= PRINTER_FLAG2_IGNORE_M106_COMMAND;
        } else {
            Printer::flag2 &= ~PRINTER_FLAG2_IGNORE_M106_COMMAND;
        }
    }
    if (!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
        int p = 0;
        if (com->hasP()) {
            p = static_cast<int>(com->P);
        } else if (Tool::getActiveTool()) {
            for (fast8_t i = 0; i < NUM_FANS; i++) {
                if (Tool::getActiveTool()->usesSecondary(fans[i].fan)) {
                    p = i;
                    break;
                }
            }
        }
        Printer::setFanSpeed(com->hasS() ? com->S : 255, false, p, com->hasD() ? static_cast<uint32_t>(com->D * 1000) : 0);
    }
}

void MCode_107(GCode* com) {
    if (!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
        int p = 0;
        if (com->hasP()) {
            p = static_cast<int>(com->P);
        } else if (Tool::getActiveTool()) {
            for (fast8_t i = 0; i < NUM_FANS; i++) {
                if (Tool::getActiveTool()->usesSecondary(fans[i].fan)) {
                    p = i;
                    break;
                }
            }
        }
        Printer::setFanSpeed(0, false, p);
    }
}

void MCode_108(GCode* com) {
    Printer::breakLongCommand = false;
}

void MCode_109(GCode* com) {
#if NUM_TOOLS > 0
    if (HeatManager::reportTempsensorError())
        return;
    previousMillisCmd = HAL::timeInMilliseconds();
    if (Printer::debugDryrun()) {
        return;
    }
#ifdef EXACT_TEMPERATURE_TIMING
    Motion1::waitForEndOfMoves();
#else
    if (com->hasP() || (com->hasS() && com->S == 0))
        Motion1::waitForEndOfMoves();
#endif
    Tool* tool = Tool::getActiveTool();
    if (com->hasT()) {
        tool = Tool::getTool(com->T);
    }
    if (tool == nullptr) {
        return;
    }
    if (Motion1::dittoMode) {
        for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
            EVENT_WAITING_HEATER(i);
            tool = Tool::getTool(i);
            if (com->hasS()) {
                tool->getHeater()->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasH()) {
                tool->getHeater()->setTargetTemperature(tool->getHeater()->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
        }
        for (fast8_t i = 0; i <= Motion1::dittoMode; i++) {
            tool = Tool::getTool(i);
            tool->getHeater()->waitForTargetTemperature();
            EVENT_HEATING_FINISHED(i);
        }
    } else {
        EVENT_WAITING_HEATER(tool->getToolId());
        if (com->hasS()) {
            tool->getHeater()->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
        } else if (com->hasH()) {
            tool->getHeater()->setTargetTemperature(tool->getHeater()->getPreheatTemperature() + (com->hasO() ? com->O : 0));
        }
        tool->getHeater()->waitForTargetTemperature();
        EVENT_HEATING_FINISHED(tool->getToolId());
    }
    previousMillisCmd = HAL::timeInMilliseconds();
#endif
}

void MCode_111(GCode* com) {
    if (com->hasS())
        Printer::setDebugLevel(static_cast<uint8_t>(com->S));
    if (com->hasP()) {
        if (com->P > 0)
            Printer::debugSet(static_cast<uint8_t>(com->P));
        else
            Printer::debugReset(static_cast<uint8_t>(-com->P));
    }
    if (Printer::debugDryrun()) { // simulate movements without printing
#if NUM_TOOLS > 0
        for (uint8_t i = 0; i < NUM_TOOLS; i++) {
            if (Tool::getTool(i)->getHeater()) {
                Tool::getTool(i)->getHeater()->setTargetTemperature(0);
            }
        }
#endif
#if NUM_HEATED_BEDS != 0
        for (uint8_t i = 0; i < NUM_HEATED_BEDS; i++) {
            heatedBeds[i]->setTargetTemperature(0);
        }
#endif
    }
}

void MCode_114(GCode* com) {
    Com::writeToAll = false;
    Motion1::printCurrentPosition();
    if (com->hasS() && com->S) {
        Com::printF(PSTR("XS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][X_AXIS]);
        Com::printF(PSTR(" YS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][Y_AXIS]);
        Com::printF(PSTR(" ZS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][Z_AXIS]);
#if NUM_AXES > A_AXIS
        Com::printF(PSTR(" AS:"), Motion2::lastMotorPos[Motion2::lastMotorIdx][A_AXIS]);
#endif
        Com::println();
    }
}

void MCode_115(GCode* com) {
    Com::writeToAll = false;
    Com::printFLN(Com::tFirmware);
#if FEATURE_CONTROLLER != NO_CONTROLLER
    Com::cap(PSTR("PROGRESS:1"));
#else
    Com::cap(PSTR("PROGRESS:0"));
#endif
    Com::cap(PSTR("AUTOREPORT_TEMP:1"));
#if ENABLED(HOST_RESCUE)
    Com::cap(PSTR("HOST_RESCUE:1"));
#else
    Com::cap(PSTR("HOST_RESCUE:0"));
#endif
    //#if EEPROM_MODE != 0
    Com::cap(PSTR("EEPROM:1"));
//#else
//    Com::cap(PSTR("EEPROM:0"));
//#endif
#if LEVELING_METHOD > 0 && Z_PROBE_TYPE > 0
    Com::cap(PSTR("AUTOLEVEL:1"));
#else
    Com::cap(PSTR("AUTOLEVEL:0"));
#endif
#if Z_PROBE_TYPE > 0
    Com::cap(PSTR("Z_PROBE:1"));
#else
    Com::cap(PSTR("Z_PROBE:0"));
#endif
#if PS_ON_PIN > -1
    Com::cap(PSTR("SOFTWARE_POWER:1"));
#else
    Com::cap(PSTR("SOFTWARE_POWER:0"));
#endif
    Com::cap(PSTR("TOGGLE_LIGHTS:1"));
    Com::cap(PSTR("PAUSESTOP:1"));
    Com::cap(PSTR("PREHEAT:1"));
#if defined(DOOR_PIN) && DOOR_PIN > -1
    Com::cap(PSTR("DOOR:1"));
#endif
#if EMERGENCY_PARSER
    Com::cap(PSTR("EMERGENCY_PARSER:1"));
#else
    Com::cap(PSTR("EMERGENCY_PARSER:0"));
#endif
    Commands::reportPrinterUsage();
}
void MCode_116(GCode* com) {
    for (fast8_t h = 0; h <= NUM_HEATERS; h++) {
        EVENT_WAITING_HEATER(h);
        heaters[h]->waitForTargetTemperature();
        EVENT_HEATING_FINISHED(h);
    }
}

void MCode_117(GCode* com) {
    if (com->hasString()) {
        GUI::setStatus(com->text, GUIStatusLevel::REGULAR);
#if JSON_OUTPUT && defined(WRITE_MESSAGES_To_JSON)
        Com::printF(PSTR("{\"message\":\""), com->text);
        Com::printFLN(PSTR("\"}"));
#endif
    }
}

void MCode_118(GCode* com) {
    if (com->hasString()) {
        Com::printF(Com::tEcho);
        Com::printFLN(com->text);
    }
}

void reportEndstop(EndstopDriver& d, PGM_P text) {
    if (d.implemented()) {
        Com::print(' ');
        Com::printF(text);
        d.report();
    }
}
void MCode_119(GCode* com) {
    Com::writeToAll = false;
    Motion1::waitForEndOfMoves();
    updateEndstops();
    updateEndstops();
    Com::printF(PSTR("endstops hit:"));
    reportEndstop(endstopXMin, Com::tXMinColon);
    reportEndstop(endstopXMax, Com::tXMaxColon);
    reportEndstop(endstopYMin, Com::tYMinColon);
    reportEndstop(endstopYMax, Com::tYMaxColon);
    reportEndstop(endstopZMin, Com::tZMinColon);
    reportEndstop(endstopZMax, Com::tZMaxColon);
#if NUM_AXES > A_AXIS
    reportEndstop(endstopAMin, Com::tAMinColon);
    reportEndstop(endstopAMax, Com::tAMaxColon);
#endif
#if NUM_AXES > B_AXIS
    reportEndstop(endstopBMin, Com::tBMinColon);
    reportEndstop(endstopBMax, Com::tBMaxColon);
#endif
#if NUM_AXES > C_AXIS
    reportEndstop(endstopCMin, Com::tCMinColon);
    reportEndstop(endstopCMax, Com::tCMaxColon);
#endif
    if (ZProbe != nullptr) {
        reportEndstop(*ZProbe, Com::tZProbeState);
    }
    Com::println();
}

void MCode_120(GCode* com) {
#if BEEPER_TYPE > 0
    if (com->hasS() && com->hasP())
        beep(com->S, com->P); // Beep test
#endif
}

void MCode_140(GCode* com) {
#if NUM_HEATED_BEDS > 0
    {
        previousMillisCmd = HAL::timeInMilliseconds();
        if (Printer::debugDryrun()) {
            return;
        }
        if (HeatManager::reportTempsensorError()) {
            return;
        }
        if (com->hasH()) { // one bed
            if (com->H < 0 || com->H >= NUM_HEATED_BEDS) {
                return;
            }
            fast8_t i = static_cast<fast8_t>(com->H);
            if (com->hasS()) {
                heatedBeds[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasP()) {
                heatedBeds[i]->setTargetTemperature(heatedBeds[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
        } else { // all beds
            for (fast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
                if (com->hasS()) {
                    heatedBeds[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
                } else if (com->hasP()) {
                    heatedBeds[i]->setTargetTemperature(heatedBeds[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
                }
            }
        }
    }
#endif
}

void MCode_141(GCode* com) {
#if NUM_HEATED_CHAMBERS > 0
    {
        previousMillisCmd = HAL::timeInMilliseconds();
        if (Printer::debugDryrun()) {
            return;
        }
        if (HeatManager::reportTempsensorError()) {
            return;
        }
        if (com->hasH()) { // one bed
            if (com->H < 0 || com->H >= NUM_HEATED_CHAMBERS) {
                return;
            }
            fast8_t i = static_cast<fast8_t>(com->H);
            if (com->hasS()) {
                heatedChambers[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasP()) {
                heatedChambers[i]->setTargetTemperature(heatedChambers[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
        } else { // all beds
            for (fast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
                if (com->hasS()) {
                    heatedChambers[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
                } else if (com->hasP()) {
                    heatedChambers[i]->setTargetTemperature(heatedChambers[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
                }
            }
        }
    }
#endif
}

void MCode_155(GCode* com) {
    Printer::setAutoreportTemp((com->hasS() && com->S != 0) || !com->hasS());
    Printer::lastTempReport = HAL::timeInMilliseconds();
}

void MCode_163(GCode* com) {
#if 0 // mixing extruder
    if (com->hasS() && com->hasP() && com->S < NUM_EXTRUDER && com->S >= 0)
        Extruder::setMixingWeight(com->S, com->P);
    Extruder::recomputeMixingExtruderSteps();
#endif
}

void MCode_164(GCode* com) {
#if 0 // mixing extruder
    if (!com->hasS() || com->S < 0 || com->S >= VIRTUAL_EXTRUDER)
        break; // ignore illegal values
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        extruder[i].virtualWeights[com->S] = extruder[i].mixingW;
    }
#if EEPROM_MODE != 0
    if (com->hasP() && com->P != 0) // store permanently to eeprom
        EEPROM::storeMixingRatios();
#endif
#endif
}

void MCode_170(GCode* com) {
    /* M170 - Set or retrieve preheat temperatures
        Parameter:
        B<bedPreheat> : Sets bed preheat temperature
        C<chamberPreheat> : Sets heated chamber temperature
        T<extruder> S<preheatTemp> : Sets preheat temperature for given extruder
        L0 : List preheat temperatures. Returns
        PREHEAT_BED:temp PREHEAT0:extr0 PREHEAT1:extr1 PREHEAT_CHAMBER:temp
        */
    // TODO preheat temperatures for extruders/bed
    /*{
            bool mod = false;
#if HAVE_HEATED_BED
            if (com->hasB()) {
                mod |= heatedBedController.preheatTemperature != static_cast<int16_t>(com->B);
                heatedBedController.preheatTemperature = com->B;
            }
#endif
#if NUM_EXTRUDER > 0
            if (com->hasT() && com->hasS() && com->T < NUM_EXTRUDER) {
                mod |= extruder[com->T].tempControl.preheatTemperature != static_cast<int16_t>(com->S);
                extruder[com->T].tempControl.preheatTemperature = com->S;
            }
#endif
            if (com->hasL()) {
#if HAVE_HEATED_BED
                Com::printF(PSTR("PREHEAT_BED:"), heatedBedController.preheatTemperature);
#endif
#if NUM_EXTRUDER > 0
                for (int i = 0; i < NUM_EXTRUDER; i++) {
                    Com::printF(PSTR(" PREHEAT"), i);
                    Com::printF(Com::tColon, extruder[i].tempControl.preheatTemperature);
                }
#endif
                Com::println();
            }
            if (mod) {
#if EEPROM_MODE != 0
#if HAVE_HEATED_BED
                HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP, heatedBedController.preheatTemperature);
#endif
#if NUM_EXTRUDER > 0
                for (int i = 0; i < NUM_EXTRUDER; i++) {
                    int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
                    Extruder* e = &extruder[i];
                    HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT, e->tempControl.preheatTemperature);
                }
#endif
                EEPROM::updateChecksum();
#endif
            }
        }*/
}

void MCode_190(GCode* com) {
#if NUM_HEATED_BEDS > 0
    {
        previousMillisCmd = HAL::timeInMilliseconds();
        if (Printer::debugDryrun() || HeatManager::reportTempsensorError()) {
            return;
        }
        UI_STATUS_UPD("Heating Bed");
        Motion1::waitForEndOfMoves();
        if (com->hasH()) { // one bed
            if (com->H < 0 || com->H >= NUM_HEATED_BEDS) {
                return;
            }
            fast8_t i = static_cast<fast8_t>(com->H);
            EVENT_WAITING_HEATER(1000 + i);
            if (com->hasS()) {
                heatedBeds[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasP()) {
                heatedBeds[i]->setTargetTemperature(heatedBeds[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
            heatedBeds[i]->waitForTargetTemperature();
            EVENT_HEATING_FINISHED(1000 + i);
        } else { // all beds
            for (fast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
                EVENT_WAITING_HEATER(1000 + i);
                if (com->hasS()) {
                    heatedBeds[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
                } else if (com->hasP()) {
                    heatedBeds[i]->setTargetTemperature(heatedBeds[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
                }
            }
            for (fast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
                heatedBeds[i]->waitForTargetTemperature();
                EVENT_HEATING_FINISHED(1000 + i);
            }
        }
        UI_CLEAR_STATUS;
        previousMillisCmd = HAL::timeInMilliseconds();
    }
#endif
}

void MCode_191(GCode* com) {
#if NUM_HEATED_CHAMBERS > 0
    {
        previousMillisCmd = HAL::timeInMilliseconds();
        if (Printer::debugDryrun() || HeatManager::reportTempsensorError()) {
            return;
        }
        UI_STATUS_UPD("Heating Chamber");
        Motion1::waitForEndOfMoves();
        if (com->hasH()) { // one bed
            if (com->H < 0 || com->H >= NUM_HEATED_CHAMBERS) {
                return;
            }
            fast8_t i = static_cast<fast8_t>(com->H);
            EVENT_WAITING_HEATER(2000 + i);
            if (com->hasS()) {
                heatedChambers[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
            } else if (com->hasP()) {
                heatedChambers[i]->setTargetTemperature(heatedChambers[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
            }
            heatedChambers[i]->waitForTargetTemperature();
            EVENT_HEATING_FINISHED(2000 + i);
        } else { // all beds
            for (fast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
                EVENT_WAITING_HEATER(2000 + i);
                if (com->hasS()) {
                    heatedChambers[i]->setTargetTemperature(com->S + (com->hasO() ? com->O : 0));
                } else if (com->hasP()) {
                    heatedChambers[i]->setTargetTemperature(heatedChambers[i]->getPreheatTemperature() + (com->hasO() ? com->O : 0));
                }
            }
            for (fast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
                heatedChambers[i]->waitForTargetTemperature();
                EVENT_HEATING_FINISHED(2000 + i);
            }
        }
        UI_CLEAR_STATUS;
        previousMillisCmd = HAL::timeInMilliseconds();
    }
#endif
}

void MCode_200(GCode* com) {
    uint8_t extruderId = Tool::getActiveToolId();
    if (com->hasT() && com->T < NUM_TOOLS)
        extruderId = com->T;
    float d = 0;
    if (com->hasR())
        d = com->R;
    if (com->hasD())
        d = com->D;
    Tool::getTool(extruderId)->setDiameter(d);
    if (extruderId == Tool::getActiveToolId())
        Commands::changeFlowrateMultiply(Printer::extrudeMultiply);
    if (d == 0) {
        Com::printFLN(PSTR("Disabled volumetric extrusion for extruder "), static_cast<int>(extruderId));
    } else {
        Com::printF(PSTR("Set volumetric extrusion for extruder "), static_cast<int>(extruderId));
        Com::printFLN(PSTR(" to "), d);
    }
}

void MCode_201(GCode* com) {
    Motion1::fillPosFromGCode(*com, Motion1::maxAcceleration, Motion1::maxAcceleration);
    Printer::updateDerivedParameter();
}

void MCode_202(GCode* com) {
    Motion1::fillPosFromGCode(*com, Motion1::maxAcceleration, Motion1::maxAcceleration);
    Printer::updateDerivedParameter();
}

void MCode_203(GCode* com) {
    if (com->hasX()) {
        Motion1::maxFeedrate[X_AXIS] = com->X / 60.0f;
    }
    if (com->hasY()) {
        Motion1::maxFeedrate[Y_AXIS] = com->Y / 60.0f;
    }
    if (com->hasZ()) {
        Motion1::maxFeedrate[Z_AXIS] = com->Z / 60.0f;
    }
    if (com->hasE()) {
        Motion1::maxFeedrate[E_AXIS] = com->E / 60.0f;
    }
#if NUM_AXES > A_AXIS
    if (com->hasA()) {
        Motion1::maxFeedrate[A_AXIS] = com->E / 60.0f;
    }
#endif
#if NUM_AXES > B_AXIS
    if (com->hasB()) {
        Motion1::maxFeedrate[B_AXIS] = com->E / 60.0f;
    }
#endif
#if NUM_AXES > C_AXIS
    if (com->hasC()) {
        Motion1::maxFeedrate[C_AXIS] = com->E / 60.0f;
    }
#endif
}

void MCode_204(GCode* com) {
    // Convert to new system
    HeatManager* pid = Tool::getActiveTool()->getHeater();
    if (com->hasS()) {
        if (com->S < 0)
            return;
        if (com->S < NUM_HEATERS)
            pid = heaters[com->S];
    }
    if (pid == nullptr) {
        return;
    }
    pid->setPID(com->hasX() ? com->X : pid->getP(),
                com->hasY() ? com->Y : pid->getI(),
                com->hasZ() ? com->Z : pid->getD());
}

void MCode_205(GCode* com) {
    Com::writeToAll = false;
    EEPROM::writeSettings();
}

void MCode_206(GCode* com) {
    Com::writeToAll = false;
    EEPROM::update(com);
}

void MCode_207(GCode* com) {
    Motion1::fillPosFromGCode(*com, Motion1::maxYank, Motion1::maxYank);
    Com::printF(Com::tJerkColon, Motion1::maxYank[X_AXIS]);
    Com::printFLN(Com::tZJerkColon, Motion1::maxYank[Z_AXIS]);
}

void MCode_209(GCode* com) {
    if (com->hasS())
        Printer::setAutoretract(com->S != 0);
}

void MCode_220(GCode* com) {
    Commands::changeFeedrateMultiply(com->getS(100));
}

void MCode_221(GCode* com) {
    Commands::changeFlowrateMultiply(com->getS(100));
}

void MCode_226(GCode* com) {
    if (!com->hasS() || !com->hasP())
        return;
    {
        bool comp = com->S;
        if (com->hasX()) {
            if (com->X == 0)
                HAL::pinMode(com->P, INPUT);
            else
                HAL::pinMode(com->P, INPUT_PULLUP);
        }
        do {
            Commands::checkForPeriodicalActions(true);
            GCode::keepAlive(WaitHeater);
        } while (HAL::digitalRead(com->P) != comp);
    }
}

void MCode_232(GCode* com) {
    // Report max advance has been removed
}

void MCode_251(GCode* com) {
#if Z_HOME_DIR > 0 && MAX_HARDWARE_ENDSTOP_Z
    Printer::zLength -= Printer::currentPosition[Z_AXIS];
    Printer::currentPositionSteps[Z_AXIS] = 0;
    Printer::updateDerivedParameter();
#if NONLINEAR_SYSTEM
    transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
#endif
    Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#if EEPROM_MODE != 0
    EEPROM::storeDataIntoEEPROM(false);
    Com::printFLN(Com::tEEPROMUpdated);
#endif
    Commands::printCurrentPosition();
#endif
}

void MCode_280(GCode* com) {
    bool mirror = false;
    fast8_t count = 0;
    if (com->hasS() && com->S > 0 && com->S < NUM_TOOLS) {
        count = com->S;
    }
    if (com->hasR()) {
        mirror = com->R != 0 && PrinterType::supportsDittoMirror();
    }
    if (com->hasS()) {
        Tool::selectTool(0);
        Motion1::waitForEndOfMoves();
        PrinterType::setDittoMode(count, mirror);
    }
    Com::printF(PSTR("DittoMode:"), (int)Motion1::dittoMode);
    Com::printFLN(PSTR(" Mirror:"), (int)Motion1::dittoMirror);
}

void MCode_281(GCode* com) {
#if FEATURE_WATCHDOG
    if (com->hasX()) {
        HAL::stopWatchdog();
        Com::printFLN(PSTR("Watchdog disabled"));
        return;
    }
    Com::printInfoFLN(PSTR("Triggering watchdog. If activated, the printer will reset."));
    Printer::kill(false);
    HAL::delayMilliseconds(200); // write output, make sure heaters are off for safety
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    InterruptProtectedBlock noInts; // don't disable interrupts on mega2560 and mega1280 because of bootloader bug
#endif
    while (1) {
    } // Endless loop to force triggering
#else
    Com::printInfoFLN(PSTR("Watchdog feature was not compiled into this version!"));
#endif
}

void MCode_300(GCode* com) {
#if defined(BEEPER_PIN) && BEEPER_PIN >= 0
    if (!Printer::tonesEnabled) {
        return;
    }
    u_int16_t beepS = 1;
    u_int16_t beepP = 1000;
    if (com->hasS())
        beepS = com->S;
    if (com->hasP())
        beepP = com->P;
    Printer::addToToneQueue({ beepS, beepP });
#endif
}

void MCode_302(GCode* com) {
    Printer::setColdExtrusionAllowed(!com->hasS() || (com->hasS() && com->S != 0));
}

void MCode_303(GCode* com) {
    int t = 0;
    if (com->hasP()) {
        t = com->P;
    }
    if (com->hasT()) {
        t = com->T;
    }
    if (t >= 0 && t < NUM_HEATERS) {
        heaters[t]->autocalibrate(com);
    } else {
        Com::printWarningFLN(PSTR("Bad heater id."));
    }
}

void MCode_320(GCode* com) {
    Motion1::setAutolevelActive(true);
    if (com->hasS() && com->S) {
        EEPROM::storeDataIntoEEPROM();
    }
}

void MCode_321(GCode* com) {
#if LEVELING_METHOD > 0
    Motion1::setAutolevelActive(false);
    if (com->hasS() && com->S) {
        if (com->S == 3) {
            Motion1::resetTransformationMatrix(false);
        }
        EEPROM::markChanged();
    }
#endif
}

void MCode_322(GCode* com) {
#if LEVELING_METHOD > 0
    Motion1::resetTransformationMatrix(false);
    if (com->hasS() && com->S) {
        EEPROM::markChanged();
    }
#endif
}

void MCode_323(GCode* com) {
    Leveling::execute_M323(com);
}

void MCode_340(GCode* com) {
#if NUM_SERVOS > 0
    uint8_t p = com->hasP() ? static_cast<uint8_t>(com->P) : 0;
    if (p < NUM_SERVOS) {
        ENSURE_POWER
        int s = com->hasS() ? com->S : 0;
        uint16_t r = com->hasR() ? com->R : 0; // auto off time in 100ms
        servos[com->P]->setPosition(s, r);
    }
#endif
}

void MCode_350(GCode* com) {
    if (com->hasP() && com->hasS() && com->P >= 0 && com->P < NUM_MOTORS) {
        if (Motion1::drivers[com->P]->implementsSetMicrosteps()) {
            Motion1::drivers[com->P]->setMicrosteps((int)com->S);
        } else {
            Com::printWarningFLN(PSTR("This driver does not support setting microsteps by software!"));
        }
    }
}

void MCode_355(GCode* com) {
    if (com->hasS()) {
        Printer::caseLightMode = static_cast<fast8_t>(com->S);
    }
    if (com->hasP()) {
        Printer::caseLightBrightness = constrain(static_cast<uint16_t>(com->P), 0, 255);
    }
    Printer::reportCaseLightStatus();
}

void MCode_360(GCode* com) {
    Com::writeToAll = false;
    Printer::showConfiguration();
}

void MCode_400(GCode* com) {
    Motion1::waitForEndOfMoves();
}

void MCode_401(GCode* com) {
    Motion1::pushToMemory();
}

void MCode_402(GCode* com) {
    Motion1::popFromMemory();
    Motion1::moveByOfficial(Motion1::tmpPosition, Printer::feedrate, false);
    // Printer::GoToMemoryPosition(com->hasX(), com->hasY(), com->hasZ(), com->hasE(), (com->hasF() ? com->F : Printer::feedrate));
}

void MCode_408(GCode* com) {
#if JSON_OUTPUT
    Printer::showJSONStatus(com->hasS() ? static_cast<int>(com->S) : 0);
#endif
}

void MCode_415(GCode* com) {
#if HOST_RESCUE
    if (com->hasS()) { // Enable rescue system
        Printer::enableRescue(com->S != 0);
    }
    if (com->hasZ()) { // Replace z
        Motion1::currentPosition[Z_AXIS] = com->Z;
        Motion1::g92Offsets[Z_AXIS] = 0;
        Motion1::updatePositionsFromCurrent();
        Motion2::setMotorPositionFromTransformed();
    }
    Printer::rescueReport();
#endif
}

void MCode_416(GCode* com) {
#if !EMERGENCY_PARSER
    Printer::handlePowerLoss();
#endif
}

void MCode_460(GCode* com) {
    // set thermo fan min/max temp - unimplemented
}

void MCode_500(GCode* com) {
#if EEPROM_MODE != 0
    EEPROM::storeDataIntoEEPROM(false);
    Com::printInfoFLN(Com::tConfigStoredEEPROM);
#else
    Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void MCode_501(GCode* com) {
#if EEPROM_MODE != 0
    EEPROM::readDataFromEEPROM();
    // Extruder::selectExtruderById(Extruder::current->id);
    Com::printInfoFLN(Com::tConfigLoadedEEPROM);
#else
    Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void MCode_502(GCode* com) {
    EEPROM::restoreEEPROMSettingsFromConfiguration();
}

void MCode_513(GCode* com) {
    // TODO: jam
    // Extruder::markAllUnjammed();
}

void MCode_530(GCode* com) {
    if (com->hasL()) {
        Printer::maxLayer = static_cast<int>(com->L);
    }
    if (com->hasS()) {
        Printer::setPrinting(static_cast<uint8_t>(com->S));
    } else {
        Printer::setPrinting(0);
    }
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    GUI::resetMenu();
}

void MCode_531(GCode* com) {
    strncpy(Printer::printName, com->text, 20);
    Printer::printName[20] = 0;
}

void MCode_532(GCode* com) {
    if (com->hasX())
        Printer::progress = com->X;
    if (Printer::progress > 100.0)
        Printer::progress = 100.0;
    else if (Printer::progress < 0)
        Printer::progress = 0;
    if (com->hasL())
        Printer::currentLayer = static_cast<int>(com->L);
}

void MCode_539(GCode* com) {
    if (com->hasS()) {
        Printer::setSupportStartStop(com->S != 0);
    }
    if (com->hasP()) {
        if (com->P) {
            Printer::setMenuMode(MENU_MODE_PAUSED, true);
        } else {
            Printer::setMenuMode(MENU_MODE_PAUSED, false);
            GUI::resetMenu();
        }
    }
}

void MCode_540(GCode* com) {
    Motion1::reportBuffers();
    Motion2::reportBuffers();
    Motion3::reportBuffers();
}

void MCode_600(GCode* com) {
    // #if FEATURE_CONTROLLER != NO_CONTROLLER && FEATURE_RETRACTION
    // uid.executeAction(UI_ACTION_WIZARD_FILAMENTCHANGE, true);
    // #endif
}

void MCode_601(GCode* com) {
    bool extruder = com->hasE() ? com->E != 0 : true;
    bool bed = com->hasB() && com->B != 0;
    bool chamber = com->hasC() && com->C != 0;
    bool all = extruder && !bed && !chamber;
    extruder |= all;
    chamber |= all;
    bed |= all;
    if (com->hasS() && com->S > 0) {
        for (fast8_t i = 0; i < NUM_HEATERS; i++) {
            HeatManager* h = heaters[i];
            if ((h->isExtruderHeater() && extruder) || (h->isBedHeater() && bed) || (h->isChamberHeater() && chamber)) {
                h->pause();
            }
        }
    } else {
        for (fast8_t i = 0; i < NUM_HEATERS; i++) {
            HeatManager* h = heaters[i];
            if ((h->isExtruderHeater() && extruder) || (h->isBedHeater() && bed) || (h->isChamberHeater() && chamber)) {
                h->unpause();
            }
        }
        if (com->hasP() && com->P != 0) {
            for (fast8_t i = 0; i < NUM_HEATERS; i++) {
                HeatManager* h = heaters[i];
                if ((h->isExtruderHeater() && extruder) || (h->isBedHeater() && bed) || (h->isChamberHeater() && chamber)) {
                    h->waitForTargetTemperature();
                }
            }
        }
    }
}

void MCode_602(GCode* com) {
    Motion1::waitForEndOfMoves();
    if (com->hasS())
        Printer::setDebugJam(com->S > 0);
    if (com->hasP())
        Printer::setJamcontrolDisabled(com->P > 0);
    Com::printF(PSTR("Debug Jam:"), (int)Printer::isDebugJam());
    Com::printFLN(PSTR(" Enabled:"), (int)!Printer::isJamcontrolDisabled());
}

void MCode_604(GCode* com) {
    // TODO: Jam handling
    /*    
#if EXTRUDER_JAM_CONTROL && NUM_EXTRUDER > 0
    uint8_t extId = Extruder::current->id;
    if (com->hasT())
        extId = com->T;
    if (extId >= NUM_EXTRUDER)
        return;
    Extruder& ext = extruder[extId];
    if (com->hasX())
        ext.jamSlowdownSteps = static_cast<int32_t>(com->X);
    if (com->hasY())
        ext.jamErrorSteps = static_cast<int32_t>(com->Y);
    if (com->hasZ())
        ext.jamSlowdownTo = static_cast<uint8_t>(com->Z);
#endif
*/
}

void MCode_606(GCode* com) {
    PrinterType::park(com);
}

void MCode_669(GCode* com) {
    millis_t t1 = HAL::timeInMilliseconds();
    GUI::refresh();
    millis_t diff = HAL::timeInMilliseconds() - t1;
    Com::printFLN(PSTR("LCD Refresh time:"), static_cast<int32_t>(diff));
}

void MCode_890(GCode* com) {
    /*
#if LEVELING_METHOD > 0 && FEATURE_Z_PROBE
    if (com->hasX() && com->hasY()) {
        float c = Printer::bendingCorrectionAt(com->X, com->Y);
        Com::printF(PSTR("Bending at ("), com->X);
        Com::printF(PSTR(","), com->Y);
        Com::printFLN(PSTR(") = "), c);
    }
#endif
*/
}

void MCode_900(GCode* com) {
    Tool* t = Tool::getActiveTool();
    if (com->hasT() && com->T < NUM_TOOLS) {
        t = Tool::getTool(com->T);
    }
    if (com->hasK() && com->K >= 0) {
        t->setAdvance(com->K);
    } else if (com->hasL() && com->L >= 0) {
        t->setAdvance(com->L);
    }
    if (com->hasR() && com->R > 0) {
        Motion1::advanceEDRatio = com->R;
    } else if (com->hasD() && com->hasW() && com->hasH()) {
        Motion1::advanceEDRatio = com->W * com->H / (com->D * com->D * 0.25f * M_PI);
    }
    Com::printF(PSTR("Advance K="));
    Com::printFloat(Motion1::advanceK, 2);
    Com::printF(PSTR(" E/D="));
    if (Motion1::advanceEDRatio == 0) {
        Com::printFloat(Motion1::advanceEDRatio, 4);
    } else {
        Com::printF(PSTR("Auto"));
    }
    Com::println();
}

void MCode_907(GCode* com) {
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
    // If "S" is specified, use that as initial default value, then update each axis w/ specific values as found later.
    if (com->hasS()) {
        for (int i = 0; i < 10; i++) {
            setMotorCurrentPercent(i, com->S);
        }
    }

    if (com->hasX())
        setMotorCurrentPercent(0, (float)com->X);
    if (com->hasY())
        setMotorCurrentPercent(1, (float)com->Y);
    if (com->hasZ())
        setMotorCurrentPercent(2, (float)com->Z);
    if (com->hasE())
        setMotorCurrentPercent(3, (float)com->E);
#endif
}

void MCode_908(GCode* com) {
    if (com->hasP() && com->hasS() && com->P >= 0 && com->P < NUM_MOTORS) {
        if (Motion1::drivers[com->P]->implementsSetMaxCurrent()) {
            Motion1::drivers[com->P]->setMaxCurrent((int)com->S);
        } else {
            Com::printWarningFLN(PSTR("This driver does not support setting current by software!"));
        }
    }
}

void MCode_909(GCode* com) {
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
    dacPrintValues();
#endif
}

void MCode_910(GCode* com) {
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
    dacCommitEeprom();
#endif
}

void MCode_998(GCode* com) {
    // TODO: debug messages
    // UI_MESSAGE(com->S);
}

void MCode_999(GCode* com) {
    Printer::failedMode = false;
    if (com->hasS()) {
        GCode::fatalError(Com::tTestM999);
        Printer::failedMode = true;
        GUI::setStatusP(Com::tTestM999, GUIStatusLevel::ERROR);
    } else {
        GCode::resetFatalError();
    }
}

void MCode_Stepper(GCode* com) {
    if (com->hasP() && com->P >= 0 && com->P < NUM_MOTORS) {
        Motion1::drivers[com->P]->handleMCode(*com);
    }
    if (!com->hasP()) {
        for (fast8_t i = 0; i < NUM_MOTORS; i++) {
            Motion1::drivers[i]->handleMCode(*com);
        }
    }
}