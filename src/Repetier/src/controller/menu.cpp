#include "Repetier.h"

const char* const axisNames[] PROGMEM = {
    "X", "Y", "Z", "E", "A", "B", "C"
};

const int32_t baudrates[] PROGMEM = {
    38400, 56000, 57600, 76800, 115200, 128000, 230400, 250000, 256000, 460800, 500000, 0
};

void __attribute__((weak)) menuBabystepZ(GUIAction action, void* data) {
    DRAW_FLOAT_P(PSTR("Babystep Z:"), Com::tUnitMM, Motion1::totalBabystepZ, 2);
    if (GUI::handleFloatValueAction(action, v, 0.01f)) {
        GCode mod;
        mod.setZ(v - Motion1::totalBabystepZ);
        PrinterType::M290(&mod);
    }
}

void __attribute__((weak)) menuMoveAxisFine(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Move @-Axis (0.01mm):"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::getShowPosition(axis), 2);
    if (!Tool::getActiveTool()->showMachineCoordinates()) {
        v -= Motion1::g92Offsets[axis];
    }
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[axis], Motion1::maxPos[axis], 0.01f)) {
        Motion1::copyCurrentOfficial(Motion1::tmpPosition);
        Motion1::tmpPosition[axis] = v;
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[axis], false);
    }
}

void __attribute__((weak)) menuMoveAxis(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Move @-Axis (1mm):"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::getShowPosition(axis), 2);
    if (!Tool::getActiveTool()->showMachineCoordinates()) {
        v -= Motion1::g92Offsets[axis];
    }
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuMoveAxisFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[axis], Motion1::maxPos[axis], 1.0)) {
        Motion1::copyCurrentOfficial(Motion1::tmpPosition);
        Motion1::tmpPosition[axis] = v;
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[axis], false);
    }
}

void __attribute__((weak)) menuMoveE(GUIAction action, void* data) {
    DRAW_FLOAT_P(PSTR("Active Extruder:"), Com::tUnitMM, Motion1::currentPosition[E_AXIS], 2);
    if (GUI::handleFloatValueAction(action, v, 1.0)) {
        Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, v);
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[E_AXIS], false);
    }
}

void __attribute__((weak)) menuStepsPerMMFine(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Resolution @ Fine:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitStepsPerMM, Motion1::resolution[axis], 2);
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 0.01)) {
        Motion1::resolution[axis] = v;
    }
}

void __attribute__((weak)) menuStepsPerMM(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Resolution @ Coarse:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitStepsPerMM, Motion1::resolution[axis], 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuStepsPerMMFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 1.0)) {
        Motion1::resolution[axis] = v;
    }
}

void __attribute__((weak)) menuMinPosFine(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Min Pos @ Fine:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::minPos[axis], 2);
    if (GUI::handleFloatValueAction(action, v, -2000, 2000, 0.01)) {
        Motion1::minPos[axis] = v;
    }
}

void __attribute__((weak)) menuMinPos(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Min Pos @ Coarse:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::minPos[axis], 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuMinPosFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, -2000, 2000, 1.0)) {
        Motion1::minPos[axis] = v;
    }
}

void __attribute__((weak)) menuMaxPosFine(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max Pos @ Fine:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::maxPos[axis], 2);
    if (GUI::handleFloatValueAction(action, v, -2000, 2000, 0.01)) {
        Motion1::maxPos[axis] = v;
    }
}

void __attribute__((weak)) menuMaxPos(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max Pos @ Coarse:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::maxPos[axis], 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuMaxPosFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, -2000, 2000, 1.0)) {
        Motion1::maxPos[axis] = v;
    }
}

void __attribute__((weak)) menuHomingSpeed(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Homing @ Speed:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::homingFeedrate[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 1, 300, 1)) {
        Motion1::homingFeedrate[axis] = v;
    }
}

void __attribute__((weak)) menuMoveSpeed(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Move @ Speed:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::moveFeedrate[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 1, Motion1::maxFeedrate[axis], 1)) {
        Motion1::moveFeedrate[axis] = v;
    }
}

void __attribute__((weak)) menuMaxSpeed(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Speed:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::maxFeedrate[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 1, 1000, 1)) {
        Motion1::maxFeedrate[axis] = v;
    }
}

void __attribute__((weak)) menuMaxAcceleration(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Print Accel.:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS2, Motion1::maxAccelerationEEPROM[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 50, 20000, 50)) {
        Motion1::maxAcceleration[axis] = Motion1::maxAccelerationEEPROM[axis] = v;
    }
}

void __attribute__((weak)) menuMaxTravelAcceleration(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Travel Accel.:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS2, Motion1::maxTravelAccelerationEEPROM[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 50, 20000, 50)) {
        Motion1::maxTravelAcceleration[axis] = Motion1::maxTravelAccelerationEEPROM[axis] = v;
    }
}

void __attribute__((weak)) menuMaxYank(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Jerk:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::maxYank[axis], 1);
    if (GUI::handleFloatValueAction(action, v, 0.1, 100, 0.1)) {
        Motion1::maxYank[axis] = v;
    }
}
void __attribute__((weak)) menuConfigVolume(GUIAction action, void* data) {
    DRAW_LONG_P(PSTR("Tone/UI Volume:"), Com::tUnitPercent, Printer::toneVolume);
    if (GUI::handleLongValueAction(action, v, 0, 100, 1)) {
        if (v > MINIMUM_TONE_VOLUME && Printer::toneVolume <= MINIMUM_TONE_VOLUME) {
            BeeperSourceBase::muteAll(false);
        } else if (v <= MINIMUM_TONE_VOLUME && Printer::toneVolume > MINIMUM_TONE_VOLUME) {
            BeeperSourceBase::muteAll(true);
        }
        Printer::toneVolume = v;
    }
}
void __attribute__((weak)) menuConfigAxis(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("= Config @-Axis ="), axisNames[axis]);
    GUI::menuStart(action);
    GUI::menuText(action, GUI::tmpString, true);
    GUI::menuBack(action);
    GUI::menuFloatP(action, PSTR("Resolution  :"), Motion1::resolution[axis], 2, menuStepsPerMM, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Homing Speed:"), Motion1::homingFeedrate[axis], 0, menuHomingSpeed, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Move Speed  :"), Motion1::moveFeedrate[axis], 0, menuMoveSpeed, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Max Speed   :"), Motion1::maxFeedrate[axis], 0, menuMaxSpeed, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Print Accel.:"), Motion1::maxAcceleration[axis], 0, menuMaxAcceleration, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Travel Accel:"), Motion1::maxTravelAcceleration[axis], 0, menuMaxTravelAcceleration, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Max Jerk    :"), Motion1::maxYank[axis], 1, menuMaxYank, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Min Pos     :"), Motion1::minPos[axis], 2, menuMinPos, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Max Pos     :"), Motion1::maxPos[axis], 2, menuMaxPos, (void*)axis, GUIPageType::FIXED_CONTENT);
    GUI::menuEnd(action);
}

// RETRACT MENUS

void __attribute__((weak)) menuRetractLength(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Retract Length:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::retractLength, 2);
    if (GUI::handleFloatValueAction(action, v, 0.0f, 20.0f, 0.05f)) {
        Motion1::retractLength = v;
    }
}
void __attribute__((weak)) menuRetractSpeed(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Retract Speed:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::retractSpeed, 2);
    if (GUI::handleFloatValueAction(action, v, 0.1f, 100.0f, 2.5f)) {
        Motion1::retractSpeed = v;
    }
}
void __attribute__((weak)) menuRetractUndoSpeed(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Unretract Speed:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::retractUndoSpeed, 2);
    if (GUI::handleFloatValueAction(action, v, 0.1f, 100.0f, 2.5f)) {
        Motion1::retractUndoSpeed = v;
    }
}
void __attribute__((weak)) menuRetractZLift(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Retract zLift:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::retractZLift, 2);
    if (GUI::handleFloatValueAction(action, v, 0.0f, 5.0f, 0.05f)) {
        Motion1::retractZLift = v;
    }
}
void __attribute__((weak)) menuRetractUndoExtraLength(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Extra Unretract:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::retractUndoExtraLength, 2);
    if (GUI::handleFloatValueAction(action, v, -20.0f, 20.0f, 0.05f)) {
        Motion1::retractUndoExtraLength = v; // Can be negative
    }
}

// FOR TOOLS
void __attribute__((weak)) menuRetractLongLength(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Long Retract Length:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::retractLongLength, 2);
    if (GUI::handleFloatValueAction(action, v, 0.0f, 20.0f, 0.05f)) {
        Motion1::retractLongLength = v;
    }
}
void __attribute__((weak)) menuRetractUndoExtraLongLength(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Long Extra Unretract:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::retractUndoExtraLongLength, 2);
    if (GUI::handleFloatValueAction(action, v, -20.0f, 20.0f, 0.05f)) {
        Motion1::retractUndoExtraLongLength = v; // Can be negative
    }
}

void __attribute__((weak)) menuConfigRetraction(GUIAction action, void* data) {
#if FEATURE_RETRACTION
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Config Retract ="), true);
    GUI::menuBack(action);

    if (!Motion1::retractLength) {
        GUI::menuSelectableP(action, PSTR("Length:Off"), menuRetractLength, nullptr, GUIPageType::FIXED_CONTENT);
    } else {
        GUI::menuFloatP(action, PSTR("Length:"), Motion1::retractLength, 2, menuRetractLength, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuFloatP(action, PSTR("Speed:"), Motion1::retractSpeed, 2, menuRetractSpeed, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuFloatP(action, PSTR("Undo Speed:"), Motion1::retractUndoSpeed, 2, menuRetractUndoSpeed, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuFloatP(action, PSTR("Extra Undo:"), Motion1::retractUndoExtraLength, 2, menuRetractUndoExtraLength, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuFloatP(action, PSTR("ZLift:"), Motion1::retractZLift, 2, menuRetractZLift, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuOnOffP(action, PSTR("Autoretract:"), Printer::isAutoretract(), directAction, reinterpret_cast<void*>(GUI_DIRECT_ACTION_TOGGLE_AUTORETRACTIONS), GUIPageType::ACTION);

#if NUM_TOOLS > 1
        GUI::menuFloatP(action, PSTR("Long Length:"), Motion1::retractLongLength, 2, menuRetractLongLength, nullptr, GUIPageType::FIXED_CONTENT);

        GUI::menuFloatP(action, PSTR("Extra Long Undo:"), Motion1::retractUndoExtraLongLength, 2, menuRetractUndoExtraLongLength, nullptr, GUIPageType::FIXED_CONTENT);
#endif
    }
    GUI::menuEnd(action);
#endif
}

// END RETRACT MENUS

// PROBE MENUS
void __attribute__((weak)) menuProbeSpeed(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Probing Speed:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, ZProbeHandler::getSpeed(), 1);
    if (GUI::handleFloatValueAction(action, v, 0.20f, Motion1::homingFeedrate[Z_AXIS], 0.10f)) {
        ZProbeHandler::setSpeed(v);
    }
}
void __attribute__((weak)) menuProbeTrigHeight(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Trigger Height:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, ZProbeHandler::getZProbeHeight(), 2);
    if (GUI::handleFloatValueAction(action, v, -10.0f, 10.0f, 0.01f)) {
        ZProbeHandler::setZProbeHeight(v);
    }
}
void __attribute__((weak)) menuProbeBedDistance(GUIAction action, void* data) {
    GUI::flashToString(GUI::tmpString, PSTR("Bed Distance:"));
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, ZProbeHandler::getBedDistance(), 2);
    if (GUI::handleFloatValueAction(action, v, 0.05f, 20.0f, 0.01f)) {
        ZProbeHandler::setBedDistance(v);
    }
}

void __attribute__((weak)) menuConfigProbe(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Config Probe ="), true);
    GUI::menuBack(action);

    //generic probe options
    GUI::menuFloatP(action, PSTR("Probe Speed :"), ZProbeHandler::getSpeed(), 1, menuProbeSpeed, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Bed Distance:"), ZProbeHandler::getBedDistance(), 2, menuProbeBedDistance, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuFloatP(action, PSTR("Trig. Height:"), ZProbeHandler::getZProbeHeight(), 2, menuProbeTrigHeight, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuOnOffP(action, PSTR("Heater Pause:"), ZProbeHandler::getHeaterPause(), directAction, reinterpret_cast<void*>(GUI_DIRECT_ACTION_TOGGLE_PROBE_PAUSE), GUIPageType::ACTION);

    //unique probe options
    if (ZProbeHandler::hasConfigMenu()) {
        ZProbeHandler::showConfigMenu(action);
    }

    GUI::menuEnd(action);
}

class GCode;
void __attribute__((weak)) menuRunProbeOnce(GUIAction action, void* data) {
#if Z_PROBE_TYPE != Z_PROBE_TYPE_NONE
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        Printer::setZProbingActive(true);
        GCode innerCode;
        GCode_30(&innerCode);
    }
#endif
}
void __attribute__((weak)) menuRunAutolevel(GUIAction action, void* data) {
#if LEVELING_METHOD != LEVELING_METHOD_NONE && Z_PROBE_TYPE != Z_PROBE_TYPE_NONE
    if (!Printer::isHoming() && !Printer::isZProbingActive()) {
        Printer::setZProbingActive(true);
        GCode innerCode;
        GCode_32(&innerCode);
    }
#endif
}
void __attribute__((weak)) menuControlProbe(GUIAction action, void* data) {
#if Z_PROBE_TYPE != Z_PROBE_TYPE_NONE
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Control Probe ="), true);
    GUI::menuBack(action);

    GUI::menuSelectableP(action, PSTR("Run Single Probe"), menuRunProbeOnce, nullptr, GUIPageType::ACTION);
#if LEVELING_METHOD != LEVELING_METHOD_NONE // Odd case but best to check anyways
    GUI::menuSelectableP(action, PSTR("Run Autoleveling"), menuRunAutolevel, nullptr, GUIPageType::ACTION);
#endif
    //unique probe controls
    if (ZProbeHandler::hasControlMenu()) {
        ZProbeHandler::showControlMenu(action);
    }
    GUI::menuEnd(action);
#endif
}

void __attribute__((weak)) menuBaudrate(GUIAction action, void* data) {
    int32_t v = baudrate;
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddLong(v, 6);
        GUI::showValueP(PSTR("Baudrate"), Com::tUnitBaud, GUI::buf);
    } else {
        int32_t p = 0;
        int32_t rate;
        do {
            rate = pgm_read_dword(&(baudrates[(uint8_t)p]));
            if (rate == baudrate) {
                break;
            }
            p++;
        } while (rate != 0);
        if (rate == 0) {
            p -= 2;
        }
        if (GUI::handleLongValueAction(action, p, 0, 10, 1)) {
            baudrate = pgm_read_dword(&(baudrates[p]));
        } else if (action == GUIAction::CLICK) {
            // Update baudrates on click/pop
            HAL::serialSetBaudrate(baudrate);
        }
    }
}

void __attribute__((weak)) menuDebug(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Debug ="), true);
    GUI::menuBack(action);
    GUI::menuOnOffP(action, PSTR("Echo:"), Printer::debugEcho(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_ECHO, GUIPageType::ACTION);
    GUI::menuOnOffP(action, PSTR("Info:"), Printer::debugInfo(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_INFO, GUIPageType::ACTION);
    GUI::menuOnOffP(action, PSTR("Errors:"), Printer::debugErrors(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_ERRORS, GUIPageType::ACTION);
    GUI::menuOnOffP(action, PSTR("Dry Run:"), Printer::debugDryrun(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_DRYRUN, GUIPageType::ACTION);
    GUI::menuOnOffP(action, PSTR("No Moves:"), Printer::debugNoMoves(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_NO_MOVES, GUIPageType::ACTION);
    GUI::menuOnOffP(action, PSTR("Communication:"), Printer::debugCommunication(), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_DEBUG_COMMUNICATION, GUIPageType::ACTION);
#ifdef DEBUG_RESCUE
    GUI::menuSelectableP(action, PSTR("Simulate Power Loss"), directAction, (void*)GUI_DIRECT_ACTION_POWERLOSS, GUIPageType::ACTION);
#endif
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuInfo(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Information ="), true);
    GUI::menuBack(action);
    GUI::menuTextP(action, PSTR("Repetier-Firmware"));
    GUI::menuTextP(action, Com::tFirmwareVersion);
    GUI::menuTextP(action, Com::tFirmwareCompiled);
    GUI::menuTextP(action, Com::tPrinterName);
    GUI::menuTextP(action, Com::tVendor);
    if (Printer::isNativeUSB()) {
        GUI::menuTextP(action, PSTR("Using Native USB"));
    }
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuMove(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Move ="), true);
    GUI::menuBack(action);
    FOR_ALL_AXES(i) {
        if (i == A_AXIS && PRINTER_TYPE == PRINTER_TYPE_DUAL_X) {
            continue;
        }
        if (Motion1::motors[i] == nullptr) {
            continue;
        }
        GUI::flashToStringFlash(GUI::tmpString, PSTR("Move @:"), axisNames[i]);
        if (i == E_AXIS) {
            GUI::menuFloat(action, GUI::tmpString, Motion1::getShowPosition(i), 2, menuMoveE, (void*)(int)i, GUIPageType::FIXED_CONTENT);
        } else {
            GUI::menuFloat(action, GUI::tmpString, Motion1::getShowPosition(i), 2, menuMoveAxis, (void*)(int)i, GUIPageType::FIXED_CONTENT);
        }
    }
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuHome(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Home = "), true);
    GUI::menuBack(action);
    GUI::menuSelectableP(action, PSTR("Home All"), directAction, (void*)GUI_DIRECT_ACTION_HOME_ALL, GUIPageType::ACTION);
    FOR_ALL_AXES(i) {
        if (i == E_AXIS || (i == A_AXIS && PRINTER_TYPE == PRINTER_TYPE_DUAL_X)) {
            continue;
        }
        GUI::flashToStringFlash(GUI::tmpString, PSTR("Home @"), axisNames[i]);
        GUI::menuSelectable(action, GUI::tmpString, directAction, (void*)(GUI_DIRECT_ACTION_HOME_X + i), GUIPageType::ACTION);
    }
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuSpeedMultiplier(GUIAction action, void* data) {
    int32_t percent = Printer::feedrateMultiply;
    DRAW_LONG_P(PSTR("Speed Multiplier:"), Com::tUnitPercent, percent);
    if (GUI::handleLongValueAction(action, percent, 25, 500, 1)) {
        Commands::changeFeedrateMultiply(percent);
    }
}

void __attribute__((weak)) menuFlowMultiplier(GUIAction action, void* data) {
    int32_t percent = Printer::extrudeMultiply;
    DRAW_LONG_P(PSTR("Flow Multiplier:"), Com::tUnitPercent, percent);
    if (GUI::handleLongValueAction(action, percent, 25, 200, 1)) {
        Commands::changeFlowrateMultiply(percent);
    }
}

void __attribute__((weak)) menuTempControl(GUIAction action, void* data) {
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    GUI::menuStart(action);
    char help[MAX_COLS];
    if (hm->isBedHeater()) {
#if NUM_HEATED_BEDS > 1
        GUI::flashToStringLong(help, PSTR("= Bed @ ="), hm->getIndex() + 1);
#else
        GUI::flashToString(help, PSTR("= Bed ="));
#endif
    } else {
#if NUM_HEATED_CHAMBERS > 1
        GUI::flashToStringLong(help, PSTR("= Chamber @ ="), hm->getIndex() + 1);
#else
        GUI::flashToString(help, PSTR("= Chamber ="));
#endif
    }
    GUI::menuText(action, help, true);
    GUI::menuBack(action);
    hm->showControlMenu(action);
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuTempConfig(GUIAction action, void* data) {
    HeatManager* hm = reinterpret_cast<HeatManager*>(data);
    GUI::menuStart(action);
    char help[MAX_COLS];
    if (hm->isBedHeater()) {
#if NUM_HEATED_BEDS > 1
        GUI::flashToStringLong(help, PSTR("= Bed @ ="), hm->getIndex() + 1);
#else
        GUI::flashToString(help, PSTR("= Bed ="));
#endif
    } else {
#if NUM_HEATED_CHAMBERS > 1
        GUI::flashToStringLong(help, PSTR("= Chamber @ ="), hm->getIndex() + 1);
#else
        GUI::flashToString(help, PSTR("= Chamber ="));
#endif
    }
    GUI::menuText(action, help, true);
    GUI::menuBack(action);
    hm->showConfigMenu(action);
    GUI::menuEnd(action);
}

#if NUM_TOOLS > 1
void dittoToTmpString(int32_t mode, bool mirror) {
    if (mode == 0) {
        GUI::flashToString(GUI::tmpString, PSTR("Ditto Mode: Off"));
    } else if (mirror) {
        GUI::flashToString(GUI::tmpString, PSTR("Ditto Mode: Mirror"));
    } else {
        GUI::flashToStringLong(GUI::tmpString, PSTR("Ditto Mode: @ Obj."), static_cast<long>(mode + 1));
    }
}
void __attribute__((weak)) menuDitto(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Select Mode ="), true);
    GUI::menuBack(action);
    dittoToTmpString(0, false);
    GUI::menuSelectableP(action, GUI::tmpString, directAction, (void*)GUI_DIRECT_ACTION_DITTO_OFF, GUIPageType::ACTION);
#if PRINTER_TYPE == PRINTER_TYPE_DUAL_X
    dittoToTmpString(1, true);
    GUI::menuSelectableP(action, GUI::tmpString, directAction, (void*)GUI_DIRECT_ACTION_DITTO_MIRROR, GUIPageType::ACTION);
#endif
    for (int i = 1; i < NUM_TOOLS; i++) {
        dittoToTmpString(i, false);
        GUI::menuSelectableP(action, GUI::tmpString, directAction, (void*)(i - 1 + GUI_DIRECT_ACTION_DITTO_2), GUIPageType::ACTION);
    }
    GUI::menuEnd(action);
}
#endif

void __attribute__((weak)) menuControls(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Controls = "), true);
    GUI::menuBack(action);
    GUI::flashToStringLong(GUI::tmpString, PSTR("Speed: @%"), Printer::feedrateMultiply);
    GUI::menuSelectable(action, GUI::tmpString, menuSpeedMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::flashToStringLong(GUI::tmpString, PSTR("Flow: @%"), Printer::extrudeMultiply);
    GUI::menuSelectable(action, GUI::tmpString, menuFlowMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::flashToStringFloat(GUI::tmpString, PSTR("Babystep Z: @mm"), Motion1::totalBabystepZ, 2);
    GUI::menuSelectable(action, GUI::tmpString, menuBabystepZ, nullptr, GUIPageType::FIXED_CONTENT);
#ifndef NO_LIGHT_CONTROL
    GUI::menuSelectableP(action, PSTR("Toggle lights"), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_LIGHT, GUIPageType::ACTION);
#endif
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
    // Deltas have only home all so move the only function up
    GUI::menuSelectableP(action, PSTR("Home"), directAction, (void*)GUI_DIRECT_ACTION_HOME_ALL, GUIPageType::ACTION);
#else
    GUI::menuSelectableP(action, PSTR("Home"), menuHome, nullptr, GUIPageType::MENU);
#endif
    GUI::menuSelectableP(action, PSTR("Move"), menuMove, nullptr, GUIPageType::MENU);
#if NUM_FANS > 0
    GUI::menuSelectableP(action, PSTR("Fans"), menuFans, nullptr, GUIPageType::MENU);
#endif
#if Z_PROBE_TYPE != Z_PROBE_TYPE_NONE
    GUI::menuSelectableP(action, PSTR("Z-Probe"), menuControlProbe, nullptr, GUIPageType::MENU);
#endif
    GUI::menuSelectableP(action, PSTR("Disable Motors"), directAction, (void*)GUI_DIRECT_ACTION_DISABLE_MOTORS, GUIPageType::ACTION);
#undef IO_TARGET
#define IO_TARGET IO_TARGET_GUI_CONTROLS
#include "../io/redefine.h"
    // Bed and chamber have no own entry so add it and point to temperature manager
    for (fast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
#if NUM_HEATED_BEDS > 1
        GUI::menuLongP(action, PSTR("Bed "), i + 1, menuTempControl, heatedBeds[i], GUIPageType::MENU);
#else
        GUI::menuSelectableP(action, PSTR("Bed "), menuTempControl, heatedBeds[i], GUIPageType::MENU);
#endif
    }
    for (fast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
#if NUM_HEATED_CHAMBERS > 1
        GUI::menuLongP(action, PSTR("Chamber "), i + 1, menuTempControl, heatedChambers[i], GUIPageType::MENU);
#else
        GUI::menuSelectableP(action, PSTR("Chamber"), menuTempControl, heatedChambers[i], GUIPageType::MENU);
#endif
    }
#if NUM_TOOLS > 1
    dittoToTmpString(Motion1::dittoMode, Motion1::dittoMirror);
    GUI::menuSelectableP(action, GUI::tmpString, menuDitto, nullptr, GUIPageType::MENU);
#endif
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuFan(GUIAction action, void* data) {
    int id = reinterpret_cast<int>(data);
    PWMHandler* pwm = fans[reinterpret_cast<int>(data)].fan;
    int32_t percent = (pwm->get() * 100) / 255;
    GUI::flashToStringLong(GUI::tmpString, PSTR("Fan @ Speed:"), id + 1);
    DRAW_LONG(GUI::tmpString, Com::tUnitPercent, percent);
    if (GUI::handleLongValueAction(action, percent, 0, 100, 5)) {
        Printer::setFanSpeed((percent * 255) / 100, true, id);
    }
}

void __attribute__((weak)) menuFans(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Fans = "), true);
    GUI::menuBack(action);
    for (int i = 0; i < NUM_FANS; i++) {
        PWMHandler* fan = fans[i].fan;
        int32_t percent = (fan->get() * 100) / 255;
        GUI::flashToStringLong(GUI::tmpString, PSTR("Fan @:"), i + 1);
        GUI::menuLong(action, GUI::tmpString, percent, menuFan, (void*)i, GUIPageType::FIXED_CONTENT);
    }
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuTune(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Tune = "), true);
    GUI::menuBack(action);
    GUI::flashToStringLong(GUI::tmpString, PSTR("Speed: @%"), Printer::feedrateMultiply);
    GUI::menuSelectable(action, GUI::tmpString, menuSpeedMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::flashToStringLong(GUI::tmpString, PSTR("Flow: @%"), Printer::extrudeMultiply);
    GUI::menuSelectable(action, GUI::tmpString, menuFlowMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::flashToStringFloat(GUI::tmpString, PSTR("Babystep Z: @mm"), Motion1::totalBabystepZ, 2);
    GUI::menuSelectable(action, GUI::tmpString, menuBabystepZ, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuSelectableP(action, PSTR("Home"), menuHome, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Move"), menuMove, nullptr, GUIPageType::MENU);
#if NUM_FANS > 0
    GUI::menuSelectableP(action, PSTR("Fans"), menuFans, nullptr, GUIPageType::MENU);
#endif

#undef IO_TARGET
#define IO_TARGET IO_TARGET_GUI_TUNE
#include "../io/redefine.h"
    // Bed and chamber have no own entry so add it and point to temperature manager
    for (ufast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
#if NUM_HEATED_BEDS > 1
        GUI::menuLongP(action, PSTR("Bed "), i + 1, menuTempControl, heatedBeds[i], GUIPageType::MENU);
#else
        GUI::menuSelectableP(action, PSTR("Bed "), menuTempControl, heatedBeds[i], GUIPageType::MENU);
#endif
    }
    for (ufast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
#if NUM_HEATED_CHAMBERS > 1
        GUI::menuLongP(action, PSTR("Chamber "), i + 1, menuTempControl, heatedChambers[i], GUIPageType::MENU);
#else
        GUI::menuSelectableP(action, PSTR("Chamber"), menuTempControl, heatedChambers[i], GUIPageType::MENU);
#endif
    }
    GUI::menuEnd(action);
}

#if SDSUPPORT
void menuSDPrint(GUIAction action, void* dat);
void __attribute__((weak)) menuSDStartPrint(GUIAction action, void* data) {
    int pos = reinterpret_cast<int>(data);
    if (pos == -1) {
        if (GUI::folderLevel == 0) {
            return;
        }
        char* p = GUI::cwd;
        while (*p) {
            p++;
        }
        p--;
        p--;
        while (*p != '/') {
            p--;
        }
        *(++p) = '\0';
        GUI::folderLevel--;
        sd.fileSystem.chdir(GUI::cwd);
        GUI::replace(menuSDPrint, data, GUIPageType::MENU);
        return;
    }
    sd_file_t file;
    if (file.open(sd.fileSystem.cwv(), pos, O_RDONLY)) {
        sd.getFN(file);
        if (file.isDir()) {
            file.close();
            char* name = tempLongFilename;
            char* p = GUI::cwd;
            while (*p) {
                p++;
            }
            if (GUI::folderLevel >= SD_MAX_FOLDER_DEPTH) {
                return;
            }
            while (*name) {
                *p++ = *name++;
            }
            *p++ = '/';
            *p = '\0';
            GUI::folderLevel++;
            sd.fileSystem.chdir(GUI::cwd);
            GUI::replace(menuSDPrint, data, GUIPageType::MENU);
        } else { // File for print selected instead
            file.close();
            sd.fileSystem.chdir(GUI::cwd);
            if (sd.selectFile(tempLongFilename)) {
                GUI::pop();
                GUI::cwd[0] = '/'; // reset the GUI directory
                GUI::cwd[1] = '\0';
                GUI::folderLevel = 0u;
                sd.startPrint();
            }
        }
    }
}
static bool menuSDFilterName(sd_file_t* file, char* tempFilename, size_t size) {
    if (!file || (tempFilename && !size)) {
        return false;
    }
#if DISABLED(SD_MENU_SHOW_HIDDEN_FILES)
    if (file->isHidden()) {
        return false;
    }
#endif
    if (tempFilename) {
        if (tempFilename[0] == '.' && tempFilename[1] != '.') {
            return false; // MAC CRAP
        }
    }
    if (file->isDir()) {
        if (GUI::folderLevel < SD_MAX_FOLDER_DEPTH) {
            if (tempFilename) {
                GUI::flashToStringString(GUI::tmpString, PSTR("# @"), tempFilename);
                memcpy(tempFilename, GUI::tmpString, size);
            }
        } else {
            return false; // Hide any more folders since we can't go deeper.
        }
    }
    return true;
}
#if ENABLED(SD_MENU_CACHE_SCROLL_ENTRIES)
constexpr uint8_t menuSDCacheRows = 5u;
constexpr uint8_t menuSDCacheNameLen = LONG_FILENAME_LENGTH;
static uint16_t menuSDCacheLastIndexPos = 0u;
static struct menuSDCacheStruct {
    uint16_t dirIndexPos;
    char name[menuSDCacheNameLen + 1];
} menuSDNameCache[menuSDCacheRows] = { 0 };

void __attribute__((weak)) menuSDPrint(GUIAction action, void* data) {
    GUI::menuStart(action);
    if (sd.state < SDState::SD_MOUNTED) {
        // User was still inside the menu when their sdcard ejected.
        GUI::pop();
        GUI::refresh();
        return;
    }

    if (GUI::folderLevel > 0u) {
        GUI::menuText(action, GUI::cwd, true);
        GUI::menuSelectableP(action, PSTR("# Parent Directory"), menuSDStartPrint, reinterpret_cast<void*>(-1), GUIPageType::ACTION);
    } else {
        if (sd.volumeLabel[0u] == '\0') {
            GUI::menuTextP(action, PSTR("= SD Print ="), true);
        } else {
            GUI::menuText(action, sd.volumeLabel, true);
        }
        GUI::menuBack(action);
    }

    static bool reversedDir = false;
    static uint16_t lastRowDirItem = 0u, dirItemCount = 0u, dirMaxIndex = 0u;
    static uint8_t lastRow = 0u;
    sd.fileSystem.chdir(GUI::cwd);
    sd_file_t curDir = sd.fileSystem.open(GUI::cwd);

    if (action == GUIAction::ANALYSE) {
        curDir.rewind();
        lastRow = 0u;
        lastRowDirItem = 0u;
        dirItemCount = 0u;
        dirMaxIndex = 0u;
        size_t renderedRows = 0u;
        reversedDir = false;
        memset(menuSDNameCache, 0u, sizeof(menuSDNameCache));
        sd.doForDirectory(curDir, [&](sd_file_t file, sd_file_t dir, size_t depth) {
            if (menuSDFilterName(&file, sd.getFN(file), menuSDCacheNameLen)) {
                if (renderedRows < menuSDCacheRows) {
                    memcpy(menuSDNameCache[renderedRows].name, tempLongFilename, menuSDCacheNameLen);
                    menuSDNameCache[renderedRows++].dirIndexPos = file.dirIndex();
                }
                if (file.dirIndex() > dirMaxIndex) {
                    dirMaxIndex = file.dirIndex();
                }
                dirItemCount++;
            }
            return true;
        });
        menuSDCacheLastIndexPos = menuSDNameCache[menuSDCacheRows - 1u].dirIndexPos;
    }
    for (size_t i = 0u; i < menuSDCacheRows; i++) {
        if (menuSDNameCache[i].name[0u] != '\0') { // just in case.
            GUI::menuSelectable(action, menuSDNameCache[i].name, menuSDStartPrint,
                                reinterpret_cast<void*>(menuSDNameCache[i].dirIndexPos), GUIPageType::ACTION);
        }
    }
    uint8_t curRow = GUI::cursorRow[GUI::level];
    if (GUI::length[GUI::level] > menuSDCacheRows + 1) {
        if (action == GUIAction::NEXT
            && curRow == lastRow
            && menuSDCacheLastIndexPos < dirMaxIndex
            && curRow >= GUI::maxCursorRow[GUI::level]) {
            if (reversedDir) {
                menuSDCacheLastIndexPos = menuSDNameCache[menuSDCacheRows - 1u].dirIndexPos;
            }
            reversedDir = false;
            uint16_t startDirIndex = menuSDCacheLastIndexPos; // caps scan max
            bool hit = false;
            sd_file_t file;
            while (!hit && ((menuSDCacheLastIndexPos - startDirIndex) < 0xff)
                   && menuSDCacheLastIndexPos < dirMaxIndex) {
                if (file.open(&curDir, ++menuSDCacheLastIndexPos, O_RDONLY)
                    && menuSDFilterName(&file, sd.getFN(file), menuSDCacheNameLen)) {
                    memmove(&menuSDNameCache[0u], &menuSDNameCache[1u], (menuSDCacheRows - 1u) * sizeof(menuSDNameCache[0u]));
                    memcpy(menuSDNameCache[menuSDCacheRows - 1u].name, tempLongFilename, menuSDCacheNameLen);
                    menuSDNameCache[menuSDCacheRows - 1u].dirIndexPos = file.dirIndex();
                    hit = true;
                    lastRowDirItem++;
                }
            }
            file.close();
        } else if (curRow <= GUI::topRow[GUI::level]) {
            if (GUI::nextAction == GUIAction::PREVIOUS && menuSDCacheLastIndexPos) {
                if (!reversedDir) {
                    menuSDCacheLastIndexPos = menuSDNameCache[0u].dirIndexPos;
                }
                reversedDir = true;
                bool hit = false;
                sd_file_t file;
                while (!hit && menuSDCacheLastIndexPos) {
                    if (file.open(&curDir, --menuSDCacheLastIndexPos, O_RDONLY)
                        && menuSDFilterName(&file, sd.getFN(file), menuSDCacheNameLen)) {
                        memmove(&menuSDNameCache[1u], &menuSDNameCache[0u], (menuSDCacheRows - 1u) * sizeof(menuSDNameCache[0u]));
                        memcpy(menuSDNameCache[0u].name, tempLongFilename, menuSDCacheNameLen);
                        menuSDNameCache[0u].dirIndexPos = file.dirIndex();
                        hit = true;
                        lastRowDirItem--;
                    }
                }
                file.close();
                if (menuSDCacheLastIndexPos) {
                    GUI::cursorRow[GUI::level]++;
                }
            }
        }
        uint16_t curScrollPos = lastRowDirItem;
        curScrollPos += !lastRowDirItem ? (GUI::topRow[GUI::level] + 1u) : 3u; 
        GUI::showScrollbar(action, static_cast<float>(curScrollPos - 1u) / static_cast<float>(dirItemCount - 3u), 5u, dirItemCount);
    }
    curDir.close();
    lastRow = curRow; // For scrolling to the last row without doing a scan/moving the list.
    GUI::menuEnd(action);
}
#else
void __attribute__((weak)) menuSDPrint(GUIAction action, void* data) {
    GUI::menuStart(action);
    if (sd.state < SDState::SD_MOUNTED) {
        // User was still inside the menu when their sdcard ejected.
        GUI::pop();
        GUI::refresh();
        return;
    }

    if (GUI::folderLevel > 0u) {
        GUI::menuText(action, GUI::cwd, true);
        GUI::menuSelectableP(action, PSTR("# Parent Directory"), menuSDStartPrint, reinterpret_cast<void*>(-1), GUIPageType::ACTION);
    } else {
        if (sd.volumeLabel[0u] == '\0') {
            GUI::menuTextP(action, PSTR("= SD Print ="), true);
        } else {
            GUI::menuText(action, sd.volumeLabel, true);
        }
        GUI::menuBack(action);
    }

    sd.fileSystem.chdir(GUI::cwd);
    sd_file_t curDir = sd.fileSystem.open(GUI::cwd);
    ufast8_t count = 0u;
    sd.doForDirectory(curDir, [&](sd_file_t file, sd_file_t dir, size_t depth) {
        if (menuSDFilterName(&file, sd.getFN(file), sizeof(tempLongFilename))) {
            GUI::menuSelectable(action, tempLongFilename, menuSDStartPrint, reinterpret_cast<void*>(file.dirIndex()), GUIPageType::ACTION);
            if (count++ > 200u) { // Arbitrary maximum, limited only by how long someone would scroll
                return false;
            }
        }
        return true;
    });
    if (count > 3u) {
        GUI::showScrollbar(action);
    }
    curDir.close();
    GUI::menuEnd(action);
}
#endif
#endif

void __attribute__((weak)) menuConfig(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Configuration = "), true);
    GUI::menuBack(action);
#if EEPROM_MODE > 0
    if (!Printer::isNativeUSB()) {
        GUI::menuLongP(action, PSTR("Baudrate:"), baudrate, menuBaudrate, nullptr, GUIPageType::FIXED_CONTENT);
    }
#endif
#if NUM_BEEPERS > 0
    GUI::flashToStringLong(GUI::tmpString, PSTR("Tone Volume: @%"), Printer::toneVolume);
    GUI::menuSelectable(action, GUI::tmpString, menuConfigVolume, nullptr, GUIPageType::FIXED_CONTENT);
#endif
    FOR_ALL_AXES(i) {
        if (i == E_AXIS) {
            continue;
        }
        GUI::flashToStringFlash(GUI::tmpString, PSTR("@-Axis"), axisNames[i]);
        GUI::menuSelectable(action, GUI::tmpString, menuConfigAxis, (void*)((int)i), GUIPageType::MENU);
    }
#if Z_PROBE_TYPE != Z_PROBE_TYPE_NONE
    GUI::menuSelectableP(action, PSTR("Z-Probe"), menuConfigProbe, nullptr, GUIPageType::MENU);
#endif
#if FEATURE_RETRACTION
    GUI::menuSelectableP(action, PSTR("Retraction"), menuConfigRetraction, nullptr, GUIPageType::MENU);
#endif
#undef IO_TARGET
#define IO_TARGET IO_TARGET_GUI_CONFIG
#include "../io/redefine.h"
    // Bed and chamber have no own entry so add it and point to temperature manager
    for (fast8_t i = 0; i < NUM_HEATED_BEDS; i++) {
        if (heatedBeds[i]->hasConfigMenu()) {
#if NUM_HEATED_BEDS > 1
            GUI::menuLongP(action, PSTR("Bed "), i + 1, menuTempConfig, heatedBeds[i], GUIPageType::MENU);
#else
            GUI::menuSelectableP(action, PSTR("Bed"), menuTempConfig, heatedBeds[i], GUIPageType::MENU);
#endif
        }
    }
    for (fast8_t i = 0; i < NUM_HEATED_CHAMBERS; i++) {
        if (heatedChambers[i]->hasConfigMenu()) {
#if NUM_HEATED_CHAMBERS > 1
            GUI::menuLongP(action, PSTR("Chamber "), i + 1, menuTempConfig, heatedChambers[i], GUIPageType::MENU);
#else
            GUI::menuSelectableP(action, PSTR("Chamber"), menuTempConfig, heatedChambers[i], GUIPageType::MENU);
#endif
        }
    }
    GUI::menuSelectableP(action, PSTR("Store Settings"), directAction, (void*)GUI_DIRECT_ACTION_STORE_EEPROM, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Factory Reset"), directAction, (void*)GUI_DIRECT_ACTION_FACTORY_RESET, GUIPageType::ACTION);
    GUI::menuEnd(action);
}

void __attribute__((weak)) mainMenu(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Main Menu ="), true);
    GUI::menuBack(action);
    if (Printer::isPrinting()) {
        GUI::menuSelectableP(action, PSTR("Tune"), menuTune, nullptr, GUIPageType::MENU);
    } else {
        GUI::menuSelectableP(action, PSTR("Controls"), menuControls, nullptr, GUIPageType::MENU);
    }

    if ((Printer::isPrinting() || Printer::isMenuMode(MENU_MODE_PAUSED))) {
        if (Printer::isMenuMode(MENU_MODE_PAUSED)) {
            GUI::menuSelectableP(action, PSTR("Continue Print"), directAction, (void*)GUI_DIRECT_ACTION_CONTINUE_PRINT, GUIPageType::ACTION);
        } else {
            GUI::menuSelectableP(action, PSTR("Pause Print"), directAction, (void*)GUI_DIRECT_ACTION_PAUSE_PRINT, GUIPageType::ACTION);
        }
        GUI::menuSelectableP(action, PSTR("Stop Print"), directAction, (void*)GUI_DIRECT_ACTION_STOP_PRINT, GUIPageType::ACTION);
    } else {
#if SDSUPPORT
        if (sd.state >= SDState::SD_MOUNTED) {
            if (sd.volumeLabel[0u] == '\0') {
                GUI::menuSelectableP(action, PSTR("SD Print"), menuSDPrint, nullptr, GUIPageType::MENU);
            } else {
                GUI::flashToStringString(GUI::tmpString, PSTR("Media: @"), sd.volumeLabel);
                GUI::menuSelectable(action, GUI::tmpString, menuSDPrint, nullptr, GUIPageType::MENU);
            }
        }
#endif
    }
#if SDSUPPORT && SDCARDDETECT < 0 // Offer mount option
    if (sd.state <= SDState::SD_HAS_ERROR) {
        GUI::menuSelectableP(action, PSTR("Mount SD Card"), directAction, (void*)GUI_DIRECT_ACTION_MOUNT_SD_CARD, GUIPageType::ACTION);
    }
#endif

#undef IO_TARGET
#define IO_TARGET IO_TARGET_GUI_MAIN_MENU
#include "../io/redefine.h"
    GUI::menuSelectableP(action, PSTR("Information"), menuInfo, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Configuration"), menuConfig, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Debug"), menuDebug, nullptr, GUIPageType::MENU);
    /* GUI::menuSelectableP(action, PSTR("Warning 1"), warningScreen, (void*)"Test Warning", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Info 1"), infoScreen, (void*)"Test Info", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Error 1"), errorScreen, (void*)"Test Error", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Spin"), waitScreen, (void*)"Computing", GUIPageType::BUSY);
    GUI::menuSelectableP(action, PSTR("Spin 2"), waitScreen, (void*)"Computing slowly", GUIPageType::BUSY);
    GUI::menuSelectableP(action, PSTR("Item 10"), nullptr, nullptr, GUIPageType::POP); */
    GUI::menuEnd(action);
}
