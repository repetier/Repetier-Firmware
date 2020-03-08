#include "Repetier.h"

const char* const axisNames[] PROGMEM = {
    "X", "Y", "Z", "E", "A", "B", "C"
};

void __attribute__((weak)) menuMoveAxisFine(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Move @-Axis (0.01mm):"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMM, Motion1::getShowPosition(axis), 2);
    if (!Tool::getActiveTool()->showMachineCoordinates()) {
        v -= Motion1::g92Offsets[axis];
    }
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[axis], Motion1::maxPos[axis], 0.01)) {
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
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[E_AXIS], Motion1::maxPos[E_AXIS], 1.0)) {
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
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS2, Motion1::maxAcceleration[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 50, 20000, 50)) {
        Motion1::maxAcceleration[axis] = v;
    }
}

void __attribute__((weak)) menuMaxTravelAcceleration(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Travel Accel.:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS2, Motion1::maxTravelAcceleration[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 50, 20000, 50)) {
        Motion1::maxTravelAcceleration[axis] = v;
    }
}

void __attribute__((weak)) menuMaxYank(GUIAction action, void* data) {
    int axis = reinterpret_cast<int>(data);
    GUI::flashToStringFlash(GUI::tmpString, PSTR("Max @ Jerk:"), axisNames[axis]);
    DRAW_FLOAT(GUI::tmpString, Com::tUnitMMPS, Motion1::maxYank[axis], 0);
    if (GUI::handleFloatValueAction(action, v, 0.1, 100, 0.1)) {
        Motion1::maxYank[axis] = v;
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

const long baudrates[] PROGMEM = { 38400, 56000, 57600, 76800, 115200, 128000, 230400, 250000, 256000,
                                   460800, 500000, 0 };

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
            if (rate == baudrate)
                break;
            p++;
        } while (rate != 0);
        if (rate == 0)
            p -= 2;
        if (GUI::handleLongValueAction(action, p, 0, 10, 1)) {
            baudrate = pgm_read_dword(&(baudrates[p]));
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
        GUI::menuFloat(action, GUI::tmpString, Motion1::getShowPosition(i), 2, menuMoveAxis, (void*)(int)i, GUIPageType::FIXED_CONTENT);
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
        GUI::flashToStringLong(GUI::tmpString, PSTR("Ditto Mode: Mirror"), static_cast<long>(Motion1::dittoMode));
    } else {
        GUI::flashToStringLong(GUI::tmpString, PSTR("Ditto Mode: @ Obj."), static_cast<long>(Motion1::dittoMode + 1));
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
        dittoToTmpString(1, false);
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
#ifndef NO_LIGHT_CONTROL
    GUI::menuSelectableP(action, PSTR("Toggle lights"), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_LIGHT, GUIPageType::ACTION);
#endif
#if defined(BEEPER_PIN) && BEEPER_PIN >= 0
    GUI::menuSelectableP(action, PSTR("Toggle sounds"), directAction, (void*)GUI_DIRECT_ACTION_TOGGLE_SOUNDS, GUIPageType::ACTION);
#endif
    GUI::menuSelectableP(action, PSTR("Home"), menuHome, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Move"), menuMove, nullptr, GUIPageType::MENU);
#if NUM_FANS > 0
    GUI::menuSelectableP(action, PSTR("Fans"), menuFans, nullptr, GUIPageType::MENU);
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
    GUI::menuSelectableP(action, PSTR("Home"), menuHome, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Move"), menuMove, nullptr, GUIPageType::MENU);
#if NUM_FANS > 0
    GUI::menuSelectableP(action, PSTR("Fans"), menuFans, nullptr, GUIPageType::MENU);
#endif

#undef IO_TARGET
#define IO_TARGET IO_TARGET_GUI_TUNE
#include "../io/redefine.h"
    GUI::menuEnd(action);
}

#if SDSUPPORT
void __attribute__((weak)) menuSDStartPrint(GUIAction action, void* data) {
    int pos = reinterpret_cast<int>(data);
    int count = -1;
    dir_t* p = nullptr;
    FatFile* root = sd.fat.vwd();
    FatFile file;
    root->rewind();
    if (pos == -1) {
        if (GUI::folderLevel == 0) {
            return;
        }
        char* p = GUI::cwd;
        while (*p)
            p++;
        p--;
        p--;
        while (*p != '/') {
            p--;
        }
        p++;
        *p = 0;
        GUI::folderLevel--;
        GUI::cursorRow[GUI::level] = 1; // top of new directory
        GUI::topRow[GUI::level] = 0;
        sd.fat.chdir(GUI::cwd);
        return;
    }
    while (file.openNext(root, O_READ)) {
        count++;
        HAL::pingWatchdog();
        if (count < pos) {
            file.close();
            continue;
        }
        file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
        if (file.isDir()) {
            file.close();
            char* name = tempLongFilename;
            char* p = GUI::cwd;
            while (*p)
                p++;
            if (GUI::folderLevel >= SD_MAX_FOLDER_DEPTH) {
                return;
            }
            while (*name) {
                *p++ = *name++;
            }
            *p++ = '/';
            *p = 0;
            GUI::folderLevel++;
            GUI::cursorRow[GUI::level] = 1; // top of new directory
            GUI::topRow[GUI::level] = 0;
            sd.fat.chdir(GUI::cwd);
        } else { // File for print selected instead
            file.close();
            sd.file.close();
            sd.fat.chdir(GUI::cwd);
            if (sd.selectFile(tempLongFilename, false)) {
                sd.startPrint();
                GUI::level = 0;
            }
        }
        return;
    }
}

void __attribute__((weak)) menuSDPrint(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= SD Print ="), true);
    GUI::menuBack(action);
    int count = -1;
    dir_t* p = nullptr;
    FatFile* root = sd.fat.vwd();
    FatFile file;
    root->rewind();
    if (GUI::folderLevel > 0) {
        GUI::menuSelectableP(action, PSTR("# Parent Directory"), menuSDStartPrint, (void*)count, GUIPageType::ACTION);
    }
    while (file.openNext(root, O_READ)) {
        count++;
        HAL::pingWatchdog();
        file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
        if (GUI::folderLevel >= SD_MAX_FOLDER_DEPTH && strcmp(tempLongFilename, "..") == 0) {
            file.close();
            continue;
        }
        if (tempLongFilename[0] == '.' && tempLongFilename[1] != '.') {
            file.close();
            continue; // MAC CRAP
        }
        if (file.isDir()) {
            GUI::flashToStringString(GUI::tmpString, PSTR("# @"), tempLongFilename);
            GUI::menuSelectable(action, GUI::tmpString, menuSDStartPrint, (void*)count, GUIPageType::ACTION);
        } else {
            GUI::menuSelectable(action, tempLongFilename, menuSDStartPrint, (void*)count, GUIPageType::ACTION);
        }
        file.close();
        if (count > 200) // Arbitrary maximum, limited only by how long someone would scroll
            break;
    }

    GUI::menuEnd(action);
}
#endif

void __attribute__((weak)) menuConfig(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Configuration = "), true);
    GUI::menuBack(action);
#if EEPROM_MODE > 0
    GUI::menuLongP(action, PSTR("Baudrate:"), baudrate, menuBaudrate, nullptr, GUIPageType::FIXED_CONTENT);
#endif
    FOR_ALL_AXES(i) {
        if (i == E_AXIS) {
            continue;
        }
        GUI::flashToStringFlash(GUI::tmpString, PSTR("@-Axis"), axisNames[i]);
        GUI::menuSelectable(action, GUI::tmpString, menuConfigAxis, (void*)((int)i), GUIPageType::MENU);
    }
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
#if SDSUPPORT
    if (sd.sdactive) {
        if (sd.sdmode == 0 && !Printer::isPrinting()) {
            GUI::menuSelectableP(action, PSTR("SD Print"), menuSDPrint, nullptr, GUIPageType::MENU);
        } else if (sd.sdmode == 1) { // sd printing
            GUI::menuSelectableP(action, PSTR("Pause SD Print"), directAction, (void*)GUI_DIRECT_ACTION_PAUSE_SD_PRINT, GUIPageType::ACTION);
            GUI::menuSelectableP(action, PSTR("Stop SD Print"), directAction, (void*)GUI_DIRECT_ACTION_STOP_SD_PRINT, GUIPageType::ACTION);
        } else if (sd.sdmode == 2) { // sd paused
            GUI::menuSelectableP(action, PSTR("Continue SD Print"), directAction, (void*)GUI_DIRECT_ACTION_CONTINUE_SD_PRINT, GUIPageType::ACTION);
            GUI::menuSelectableP(action, PSTR("Stop SD Print"), directAction, (void*)GUI_DIRECT_ACTION_STOP_SD_PRINT, GUIPageType::ACTION);
        }
    }
#if SDCARDDETECT < 0 // Offer mount option
    else {
        GUI::menuSelectableP(action, PSTR("Mount SD Card"), directAction, (void*)GUI_DIRECT_ACTION_MOUNT_SD_CARD, GUIPageType::ACTION);
    }
#endif
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
