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

int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;
volatile uint8_t executePeriodical = 0;
unsigned int counterPeriodical = 0;
uint8_t counter500ms = 5;
millis_t lastCommandReceived = 0;

void Commands::commandLoop() {
//while(true) {
#ifdef DEBUG_PRINT
    debugWaitLoop = 1;
#endif
    Printer::breakLongCommand = false; // block is now finished
    if (!Printer::isBlockingReceive()) {
#if SDSUPPORT
        if (sd.scheduledPause) {
            if (Motion1::buffersUsed() == 0) {
                sd.printFullyPaused();
            }
        } else if (sd.scheduledStop) {
            if (Motion1::buffersUsed() == 0) {
                sd.printFullyStopped();
            }
        }
#endif
        GCode::readFromSerial();
        GCode* code = GCode::peekCurrentCommand();
        millis_t curTime = HAL::timeInMilliseconds();
        if (code) {
#if SDSUPPORT
            if (sd.state == SDState::SD_WRITING) {
                if (!(code->hasM() && code->M == 29u)) { // still writing to file
                    sd.writeCommand(code);
                } else {
                    sd.finishWrite();
                }
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
            lastCommandReceived = curTime;
        } else {
            if (Motion1::buffersUsed() > 0) { // if printing no need to reset
                lastCommandReceived = curTime;
            }
            if ((curTime - lastCommandReceived) > 2000ul) {
                lastCommandReceived = curTime;
                Printer::parkSafety(); // will handle allowed conditions it self
            }
#if SDSUPPORT
            // 5 minute timeout if we never receive anything.
            if (sd.state == SDState::SD_WRITING) {
                if ((curTime - sd.lastWriteTimeMS) > (5ul * 60000ul)) {
                    GUI::setStatusP(PSTR("Receive timeout!"), GUIStatusLevel::WARNING);
                    sd.finishWrite();
                }
            }
#endif
        }
    } else {
        GCode::keepAlive(FirmwareState::Paused);
    }
    Printer::defaultLoopActions();
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {
    Printer::handleInterruptEvent();
    FirmwareEvent::handleEvents();
    HAL::handlePeriodical();
    if (Printer::reportFlag) {
        if (Printer::isReportFlag(PRINTER_REPORT_FLAG_ENDSTOPS)) {
            MCode_119(nullptr);
            Printer::reportFlagReset(PRINTER_REPORT_FLAG_ENDSTOPS);
        }
    }
#if EMERGENCY_PARSER
    GCodeSource::prefetchAll();
#endif
    if (Printer::isRescueRequired() || Motion1::length != 0 || Printer::isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED)) {
        previousMillisCmd = HAL::timeInMilliseconds();
    }
    EVENT_PERIODICAL;
#if defined(DOOR_PIN) && DOOR_PIN > -1
    if (Printer::updateDoorOpen()) {
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            tool->shutdown();
        }
    }
#endif
#undef IO_TARGET
#define IO_TARGET IO_TARGET_PERIODICAL_ACTIONS
#include "../io/redefine.h"
    if (!executePeriodical) {
        return; // gets true every 100ms
    }

    executePeriodical = 0;
    EEPROM::timerHandler(); // store changes after timeout
    // include generic 100ms calls
#undef IO_TARGET
#define IO_TARGET IO_TARGET_100MS
#include "../io/redefine.h"
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif

    // Report temperatures every autoReportPeriodMS (default 1000ms), so we do not need to send M105
    if (Printer::isAutoreportTemp()) {
        millis_t now = HAL::timeInMilliseconds();
        if (now - Printer::lastTempReport > Printer::autoTempReportPeriodMS) {
            Printer::lastTempReport = now;
            Com::writeToAll = true; // need to be sure to receive correct receipient
            Commands::printTemperatures();
        }
    }
#if SDSUPPORT
    // Reports the sd file byte position every autoSDReportPeriodMS if set, and only if printing.
    if (Printer::isAutoreportSD() && sd.state == SDState::SD_PRINTING) {
        millis_t now = HAL::timeInMilliseconds();
        if (now - Printer::lastSDReport > Printer::autoSDReportPeriodMS) {
            Printer::lastSDReport = now;
            Com::writeToAll = true; // need to be sure to receive correct receipient
            sd.printStatus();
        }
    }
#endif
    EVENT_TIMER_100MS;
    // Extruder::manageTemperatures();
    if (--counter500ms == 0) {
        counter500ms = 5;
#undef IO_TARGET
#define IO_TARGET IO_TARGET_500MS
#include "../io/redefine.h"
        EVENT_TIMER_500MS;
        Printer::checkFanTimeouts();
    }
#if DISPLAY_DRIVER != DRIVER_NONE
    GUI::update();
#endif
    // If called from queueDelta etc. it is an error to start a new move since it
    // would invalidate old computation resulting in unpredicted behavior.
    // lcd controller can start new moves, so we disallow it if called from within
    // a move command.
}

/** \brief Waits until movement cache is empty.

Some commands expect no movement, before they can execute. This function
waits, until the steppers are stopped. In the meanwhile it buffers incoming
commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves() {
#ifdef DEBUG_PRINT
    debugWaitLoop = 8;
#endif
    while (Motion1::length) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(FirmwareState::Processing, 3);
    }
}

void Commands::waitMS(uint32_t wait) {
    millis_t end = HAL::timeInMilliseconds() + wait;
    while (static_cast<int32_t>(end - HAL::timeInMilliseconds()) > 0) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(FirmwareState::Processing, 3);
    }
}

void Commands::waitUntilEndOfAllBuffers() {
    GCode* code = nullptr;
#ifdef DEBUG_PRINT
    debugWaitLoop = 9;
#endif
    while (Motion1::length || (code != nullptr)) {
        //GCode::readFromSerial();
        code = GCode::peekCurrentCommand();
        if (code) {
#if SDSUPPORT
            if (sd.state == SDState::SD_WRITING) {
                if (!(code->hasM() && code->M == 29)) {
                    sd.writeCommand(code);
                } else {
                    sd.finishWrite();
                }
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }
        Commands::checkForPeriodicalActions(false); // only called from memory
    }
}

void Commands::printTemperatures() {
#if NUM_TOOLS > 0
    Tool* t = Tool::getActiveTool();
    if (t != nullptr && t->supportsTemperatures()) {
        t->getHeater()->reportTemperature('T', -1);
    }
#if NUM_TOOLS > 1
    for (int i = 0; i < NUM_TOOLS; i++) {
        t = Tool::getTool(i);
        if (t != nullptr && t->supportsTemperatures()) {
            t->getHeater()->reportTemperature('T', i);
        }
    }
#endif
#endif
#if NUM_HEATED_BEDS > 0
    for (int i = 0; i < NUM_HEATED_BEDS; i++) {
        heatedBeds[i]->reportTemperature('B', i > 0 ? i : -1);
    }
#endif
#if NUM_HEATED_CHAMBERS > 0
    for (int i = 0; i < NUM_HEATED_CHAMBERS; i++) {
        heatedChambers[i]->reportTemperature('C', i > 0 ? i : -1);
    }
#endif
    Com::println();
}

void Commands::changeFeedrateMultiply(int factor) {
    if (factor < 25) {
        factor = 25;
    } else if (factor > 500) {
        factor = 500;
    }
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

void Commands::changeFlowrateMultiply(int factor) {
    if (factor < 25) {
        factor = 25;
    } else if (factor > 200) {
        factor = 200;
    }
    Printer::extrudeMultiply = factor;
    // TODO: volumetric extrusion
    //if (Extruder::current->diameter <= 0)
    Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
    //else
    //    Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
    Com::printFLN(Com::tFlowMultiply, factor);
}

void Commands::reportPrinterUsage() {
#if EEPROM_MODE != 0
    float dist = Printer::filamentPrinted * 0.001 + Printer::filamentPrintedTotal;
    Com::printF(Com::tPrintedFilament, dist, 2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
    for (uint8_t i = 0; i < NUM_TOOLS; i++) {
        Tool* t = Tool::getTool(i);
        if (t->getHeater() != nullptr && t->getHeater()->isEnabled()) {
            alloff = false;
        }
    }
    int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + Printer::printingTime;
    int32_t tmp = seconds / 86400;
    seconds -= tmp * 86400;
    Com::printF(Com::tPrintingTime, tmp);
    tmp = seconds / 3600;
    Com::printF(Com::tSpaceDaysSpace, tmp);
    seconds -= tmp * 3600;
    tmp = seconds / 60;
    Com::printF(Com::tSpaceHoursSpace, tmp);
    Com::printFLN(Com::tSpaceMin);
#endif
}

/**
\brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode* com) {
    if (Printer::failedMode) {
        return;
    }
    if (EVENT_UNHANDLED_G_CODE(com)) {
        previousMillisCmd = HAL::timeInMilliseconds();
        return;
    }
    bool unknown = false;
    switch (com->G) {
    case 0: // G0 -> G1
    case 1: // G1
        GCode_0_1(com);
        break;
    case 2: // CW Arc
    case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        GCode_2_3(com);
        break;
    case 4: // G4 dwell
        GCode_4(com);
        break;
    case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
        GCode_10(com);
        break;
    case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
        GCode_11(com);
        break;
    case 20: // G20 Units to inches
        GCode_20(com);
        break;
    case 21: // G21 Units to mm
        GCode_21(com);
        break;
    case 28:
        GCode_28(com);
        break;
    case 29:
        GCode_29(com);
        break;
    case 30:
        GCode_30(com);
        break;
    case 31:
        GCode_31(com);
        break;
    case 32: // G32 Auto-Bed leveling
        GCode_32(com);
        break;
    case 33:
        GCode_33(com);
        break;
    case 90: // G90
        GCode_90(com);
        break;
    case 91:
        GCode_91(com);
        break;
    case 92:
        GCode_92(com);
        break;
    case 100:
        GCode_134(com);
        break;
    case 131:
        GCode_131(com);
        break;
    case 132:
        GCode_132(com);
        break;
    case 133:
        GCode_133(com);
        break;
    case 134:
        GCode_134(com);
        break;
    case 135: // G135
        GCode_135(com);
        break;
    case 201:
        GCode_201(com);
        break;
    case 202:
        GCode_202(com);
        break;
    case 203:
        GCode_203(com);
        break;
    case 204:
        GCode_204(com);
        break;
    case 205:
        GCode_205(com);
        break;
    case 320:
        unknown = PrinterType::runGCode(com);
        break;
    default:
        unknown = true;
    }
    if (unknown) {
        if (Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
    previousMillisCmd = HAL::timeInMilliseconds();
}
/**
\brief Execute the G command stored in com.
*/
extern void reportAnalog();
void Commands::processMCode(GCode* com) {
    if (Printer::failedMode && (com->M == 104 || com->M == 109 || com->M == 190 || com->M == 140 || com->M == 141 || com->M == 600 || com->M == 601)) {
        return; // one of the forbidden m codes was send
    }

    if (EVENT_UNHANDLED_M_CODE(com)) {
        return;
    }
    bool unknown = false;
    uint16_t mCode = com->isPriorityM() ? com->getPriorityM() : com->M;
    switch (mCode) {
    case 0:
        // HAL::reportHALDebug();
        break;
    case 3: // Spindle/laser
        MCode_3(com);
        break;
    case 4: // Spindle CCW
        MCode_4(com);
        break;
    case 5: // Spindle/laser off
        MCode_5(com);
        break;
    case 6: // Tool change
        MCode_6(com);
        break;
    case 7: // Mist coolant on
        MCode_7(com);
        break;
    case 8: // Flood coolant on
        MCode_8(com);
        break;
    case 9: // Coolant off
        MCode_9(com);
        break;
    case 20: // M20 - list SD card
        MCode_20(com);
        break;
    case 21: // M21 - init SD card
        MCode_21(com);
        break;
    case 22: //M22 - release SD card
        MCode_22(com);
        break;
    case 23: //M23 - Select file
        MCode_23(com);
        break;
    case 24: //M24 - Start SD print
        MCode_24(com);
        break;
    case 25: //M25 - Pause SD print
        MCode_25(com);
        break;
    case 26: //M26 - Set SD index
        MCode_26(com);
        break;
    case 27: //M27 - Get SD status
        MCode_27(com);
        break;
    case 28: //M28 - Start SD write
        MCode_28(com);
        break;
    case 29: //M29 - Stop SD write
        MCode_29(com);
        break;
    case 30: // M30 filename - Delete file
        MCode_30(com);
        break;
    case 32: // M32 directoryname
        MCode_32(com);
        break;
    case 36: // M36 JSON File Info
        MCode_36(com);
        break;
    case 42: // M42 -Change pin status via gcode
        MCode_42(com);
        break;
    case 48: // M48 Xpos Ypos Ptests - Test z probe accuracy
        MCode_48(com);
        break;
    case 80: // M80 - ATX Power On
        MCode_80(com);
        break;
    case 81: // M81 - ATX Power Off
        MCode_81(com);
        break;
    case 82: // M82
        MCode_82(com);
        break;
    case 83: // M83
        MCode_83(com);
        break;
    case 17: // M17 is to enable named axis
        MCode_17(com);
        break;
    case 18: // M18 is to disable named axis
        MCode_18(com);
        break;
    case 84: // M84
        MCode_84(com);
        break;
    case 85: // M85 S<seconds> - Set an inactivity shutdown timer. (maxInactiveTime)
        MCode_85(com);
        break;
    case 92: // M92
        MCode_92(com);
        break;
    case 99: // M99 S<time>
        MCode_99(com);
        break;

    case 104: // M104 T<extr> S<temp> O<offset> F0 (beep)temperature
        MCode_104(com);
        break;
    case 140: // M140 set bed temp
        MCode_140(com);
        break;
    case 141: // M141 set chamber temp
        MCode_141(com);
        break;
    case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
        MCode_105(com);
        break;
    case 108: // Break long commands
        MCode_108(com);
        break;
    case 109: // M109 - Wait for extruder heater to reach target.
        MCode_109(com);
        break;
    case 190: // M190 - Wait bed for heater to reach target.
        MCode_190(com);
        break;
    case 191: // M191 - Wait chamber for heater to reach target.
        MCode_191(com);
        break;
    case 155: // M155 S<1/0> Enable/disable auto report temperatures. When enabled firmware will emit temperatures every second.
        MCode_155(com);
        break;
    case 116: // Wait for temperatures to reach target temperature
        MCode_116(com);
        break;
    case 106: // M106 Fan On
        MCode_106(com);
        break;
    case 107: // M107 Fan Off
        MCode_107(com);
        break;
    case 111: // M111 enable/disable run time debug flags
        MCode_111(com);
        break;
    case 115: // M115
        MCode_115(com);
        break;
    case 114: // M114
        MCode_114(com);
        break;
    case 117: // M117 message to lcd
        MCode_117(com);
        break;
    case 118: // M118
        MCode_118(com);
        break;
    case 119: // M119
        MCode_119(com);
        break;
    case 120: // M120 Test beeper function
        MCode_120(com);
        break;
    case 122: // debug stepper driver
        MCode_Stepper(com);
        break;
    case 163: // M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
        MCode_163(com);
        break;
    case 164: /// M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights as virtual extruder S
        MCode_164(com);
        break;
    case 170: // preheat temperatures
        MCode_170(com);
        break;
    case 200: // M200 T<extruder> D<diameter>
        MCode_200(com);
        break;
    case 201: // M201 <XYZE> Set temporary axis print accelerations in units/s^2
        MCode_201(com);
        break;
    case 202: // M202 <XYZE> Set temporary axis travel accelerations in units/s^2
        MCode_202(com);
        break;
    case 203: // M203
        MCode_203(com);
        break;
    case 204: // M204
        MCode_204(com);
        break;
    case 205: // M205 Show EEPROM settings
#if EMERGENCY_PARSER == 0
        MCode_205(com);
#endif
        break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
        MCode_206(com);
        break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
        MCode_207(com);
        break;
    case 209: // M209 S<0/1> Enable/disable autoretraction
        MCode_209(com);
        break;
    case 218: // M218 T<toolid> X<offsetX> Y<offsetY> Z<offsetZ> - Store new tool offsets
        MCode_218(com);
        break;
    case 220: // M220 S<Feedrate multiplier in percent>
        MCode_220(com);
        break;
    case 221: // M221 S<Extrusion flow multiplier in percent>
        MCode_221(com);
        break;
    case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
        MCode_226(com);
        break;
    case 232: // M232
        MCode_232(com);
        break;
    case 233: // M233 now use M900 like Marlin
        MCode_900(com);
        break;
    case 251: // M251
        MCode_251(com);
        break;
    case 280: // M280
        MCode_280(com);
        break;
    case 281: // Trigger watchdog
        MCode_281(com);
        break;
    case 290: // M290 Z<babysteps> - Correct by adding baby steps for Z mm
#if EMERGENCY_PARSER == 0
        PrinterType::M290(com);
#endif
        break;
    case 300: // M300
        MCode_300(com);
        break;
    case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will allow, S0 will disallow.
        MCode_302(com);
        break;
    case 303: // M303
        MCode_303(com);
        break;
    case 320: // M320 Activate autolevel
        MCode_320(com);
        break;
    case 321: // M321 Deactivate autoleveling
        MCode_321(com);
        break;
    case 322: // M322 Reset auto leveling matrix
        MCode_322(com);
        break;
    case 323: // M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
        MCode_323(com);
        break;
    case 340: // M340
        MCode_340(com);
        break;
    case 350: // M350 Set micro stepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
        MCode_350(com);
        break;
    case 355: // M355 S<0/1/2/3/4> - Turn case light on/off/burst/blink fast/blink slow , no S = report status
        MCode_355(com);
        break;
    case 360: // M360 - show configuration
        MCode_360(com);
        break;
    case 400: // M400 Finish all moves
        MCode_400(com);
        break;
    case 401: // M401 Memory position
        MCode_401(com);
        break;
    case 402: // M402 Go to stored position
        MCode_402(com);
        break;
    case 408:
        MCode_408(com);
        break;
    case 415: // host rescue command
        MCode_415(com);
        break;
    case 416: // host detected power loss
        MCode_416(com);
        break;
    case 460: // M460 X<minTemp> Y<maxTemp> : Set temperature range for thermo controlled fan
        MCode_460(com);
        break;
    case 500: // M500 store to eeprom
        MCode_500(com);
        break;
    case 501: // M501 read from eeprom
        MCode_501(com);
        break;
    case 502: // M502 restore from configuration
        MCode_502(com);
        break;
    case 513:
        MCode_513(com);
        break;
    case 524: // Abort SD printing
        MCode_524(com);
        break;
        //- M530 S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
    case 530:
        MCode_530(com);
        break;
        //- M531 filename - Define filename being printed
    case 531:
        MCode_531(com);
        break;
        //- M532 X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
    case 532:
        MCode_532(com);
        break;
    case 539:
        MCode_539(com);
        break;
    case 540: // report motion buffers
        MCode_540(com);
        break;
    case 576: // M576 S1 enables out of order execution
        MCode_576(com);
        break;
    case 569: // Set stealthchop
        MCode_Stepper(com);
        break;
    case 575: // Update all serial baudrates
        MCode_575(com);
        break;
    case 600:
        MCode_600(com);
        break;
    case 601:
        MCode_601(com);
        break;
    case 602: // set jam control
        MCode_602(com);
        break;
    case 604:
        MCode_604(com);
        break;
    case 606: // Park extruder
        MCode_606(com);
        break;
    case 665:
    case 666:
        PrinterType::runMCode(com);
        break;
    case 669: // Measure lcd refresh time
        MCode_669(com);
        break;
    case 900: // M233 now use M900 like Marlin
        MCode_900(com);
        break;
    case 906: // Report TMC current
        MCode_Stepper(com);
        break;
    case 907: // M907 Set digital trimpot/DAC motor current using axis codes.
        MCode_907(com);
        break;
    case 908: // M908 Control digital trimpot directly.
        MCode_908(com);
        break;
    case 909: // M909 Read digital trimpot settings.
        MCode_909(com);
        break;
    case 910: // M910 - Commit digipot/DAC value to external EEPROM
        MCode_910(com);
        break;
#if 0 && UI_DISPLAY_TYPE != NO_DISPLAY
    // some debugging commands normally disabled
    case 888:
        Com::printFLN(PSTR("Selected language:"), (int)Com::selectedLanguage);
        Com::printF(PSTR("Translation:"));
        Com::printFLN(Com::translatedF(0));
        break;
    case 889:
        uid.showLanguageSelectionWizard();
        break;
    case 891:
        if(com->hasS())
            EEPROM::setVersion(com->S);
        break;
#endif
    case 876:
        MCode_876(com);
        break;
    case 890:
        MCode_890(com);
        break;
    case 911: // Report TMC prewarn
        MCode_Stepper(com);
        break;
    case 912: // Clear prewarn
        MCode_Stepper(com);
        break;
    case 913: // Hybrid treshold
        MCode_Stepper(com);
        break;
    case 914: // sensorless homing sensitivity
        MCode_Stepper(com);
        break;
    case 998:
        MCode_998(com);
        break;
    case 999: // Stop fatal error take down
        MCode_999(com);
        break;
    case 9999: // Switch to bootmode if possible
        HAL::switchToBootMode();
        break;
    /* case 888: {
        Motion1::waitForEndOfMoves();
        Com::printF(PSTR("XSteps:"), XMotor.position);
        Com::printF(PSTR(" YSteps:"), YMotor.position);
        int32_t* lp = Motion2::lastMotorPos[Motion2::lastMotorIdx];
        Com::printF(PSTR(" XPosSteps:"), lp[0]);
        Com::printFLN(PSTR(" YPosSteps:"), lp[1]);
    } break; */
    default:
        unknown = true;
    } // switch
    if (unknown) {
        if (Printer::debugErrors()) {
            Com::writeToAll = false;
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}

/**
\brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode* com) {
    // Set return channel for private commands. By default all commands send to all receivers.
    GCodeSource* actSource = GCodeSource::activeSource;
    GCodeSource::activeSource = com->source;
    Com::writeToAll = true;
    if (INCLUDE_DEBUG_COMMUNICATION) {
        if (Printer::debugCommunication()) {
            if (com->hasG() || (com->hasM() && com->M != 111)) {
                previousMillisCmd = HAL::timeInMilliseconds();
                GCodeSource::activeSource = actSource;
                return;
            }
        }
    }
    if (com->hasG()) {
        processGCode(com);
    } else if (com->hasM()) {
        if (!com->isPriorityM()) {
            processMCode(com);
        }
    } else if (com->hasT()) { // Process T code
        if (!Printer::failedMode) {
            Motion1::waitForEndOfMoves();
            Tool::selectTool(com->T);
        }
    } else {
        if (Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#ifdef DEBUG_DRYRUN_ERROR
    if (Printer::debugDryrun()) {
        Com::printFLN("Dryrun was enabled");
        com->printCommand();
        Printer::debugReset(8);
    }
#endif
    GCodeSource::activeSource = actSource;
}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
    HAL::serialFlush();
    HAL::resetHardware();
#else
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Printer::kill(false);
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_KILLED_ID));
    Commands::checkForPeriodicalActions(false);
    HAL::delayMilliseconds(200);
    Commands::checkForPeriodicalActions(false);
    InterruptProtectedBlock noInts;
    while (1) {
    }
#endif
}

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if (newfree < lowestRAMValue)
        lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
    if (lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM, lowestRAMValue);
    }
}
