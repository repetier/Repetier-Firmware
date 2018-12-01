#include "Repetier.h"

void __attribute__((weak)) menuMoveX(GUIAction action, void* data) {
    DRAW_FLOAT_P(Com::tXColon, Com::tUnitMM, Motion1::currentPosition[X_AXIS], 2);
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[X_AXIS], Motion1::maxPos[X_AXIS], 1.0)) {
        Motion1::setTmpPositionXYZ(v, IGNORE_COORDINATE, IGNORE_COORDINATE);
        Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
    }
}

void __attribute__((weak)) menuMoveY(GUIAction action, void* data) {
    DRAW_FLOAT_P(Com::tYColon, Com::tUnitMM, Motion1::currentPosition[Y_AXIS], 2);
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[Y_AXIS], Motion1::maxPos[Y_AXIS], 1.0)) {
        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, v, IGNORE_COORDINATE);
        Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
    }
}

void __attribute__((weak)) menuMoveZ(GUIAction action, void* data) {
    DRAW_FLOAT_P(Com::tZColon, Com::tUnitMM, Motion1::currentPosition[Z_AXIS], 2);
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[Z_AXIS], Motion1::maxPos[Z_AXIS], 1.0)) {
        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, v);
        Motion1::moveByOfficial(Motion1::tmpPosition, Z_SPEED, false);
    }
}

void __attribute__((weak)) menuMoveE(GUIAction action, void* data) {
    DRAW_FLOAT_P(PSTR("Active Extruder:"), Com::tUnitMM, Motion1::currentPosition[E_AXIS], 2);
    if (GUI::handleFloatValueAction(action, v, Motion1::minPos[E_AXIS], Motion1::maxPos[E_AXIS], 1.0)) {
        Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, v);
        Motion1::moveByOfficial(Motion1::tmpPosition, Z_SPEED, false);
    }
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
    GUI::menuSelectableP(action, PSTR("Move X"), menuMoveX, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuSelectableP(action, PSTR("Move Y"), menuMoveY, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuSelectableP(action, PSTR("Move Z"), menuMoveZ, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuSelectableP(action, PSTR("Move E"), menuMoveE, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuHome(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Home = "), true);
    GUI::menuBack(action);
    GUI::menuSelectableP(action, PSTR("Home All"), directAction, (void*)GUI_DIRECT_ACTION_HOME_ALL, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Home X"), directAction, (void*)GUI_DIRECT_ACTION_HOME_X, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Home Y"), directAction, (void*)GUI_DIRECT_ACTION_HOME_Y, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Home Z"), directAction, (void*)GUI_DIRECT_ACTION_HOME_Z, GUIPageType::ACTION);
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
    DRAW_LONG_P(PSTR("Speed Multiplier:"), Com::tUnitPercent, percent);
    if (GUI::handleLongValueAction(action, percent, 25, 200, 1)) {
        Commands::changeFlowrateMultiply(percent);
    }
}

void __attribute__((weak)) menuControls(GUIAction action, void* data) {
    char help[MAX_COLS + 1];
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Controls = "), true);
    GUI::menuBack(action);
    GUI::flashToStringLong(help, PSTR("Speed: @%"), Printer::feedrateMultiply);
    GUI::menuSelectable(action, help, menuSpeedMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::flashToStringLong(help, PSTR("Flow: @%"), Printer::extrudeMultiply);
    GUI::menuSelectable(action, help, menuFlowMultiplier, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::menuSelectableP(action, PSTR("Disable Motors"), directAction, (void*)GUI_DIRECT_ACTION_DISABLE_MOTORS, GUIPageType::ACTION);
#undef IO_TARGET
#define IO_TARGET 16
#include "../io/redefine.h"
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuFan(GUIAction action, void* data) {
    int id = reinterpret_cast<int>(data);
    PWMHandler* pwm = fans[reinterpret_cast<int>(data)];
    int32_t percent = (pwm->get() * 100) / 255;
    char help[MAX_COLS + 1];
    GUI::flashToStringLong(help, PSTR("Fan @ Speed:"), id + 1);
    DRAW_LONG(help, Com::tUnitPercent, percent);
    if (GUI::handleLongValueAction(action, percent, 0, 100, 5)) {
        Printer::setFanSpeed((percent * 255) / 100, true, id);
    }
}

void __attribute__((weak)) menuFans(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Fans = "), true);
    GUI::menuBack(action);
    char help[MAX_COLS + 1];
    for (int i = 0; i < NUM_FANS; i++) {
        PWMHandler* fan = fans[i];
        int32_t percent = (fan->get() * 100) / 255;
        GUI::flashToStringLong(help, PSTR("Fan @:"), i + 1);
        GUI::menuLong(action, help, percent, menuFan, (void*)i, GUIPageType::FIXED_CONTENT);
    }
    GUI::menuEnd(action);
}

void __attribute__((weak)) menuConfig(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Configuration = "), true);
    GUI::menuBack(action);
#if EEPROM_MODE > 0
    GUI::menuSelectableP(action, PSTR("Baudrate"), menuBaudrate, nullptr, GUIPageType::FIXED_CONTENT);
#endif
    GUI::menuSelectableP(action, PSTR("Store Settings"), directAction, (void*)GUI_DIRECT_ACTION_STORE_EEPROM, GUIPageType::ACTION);
    GUI::menuSelectableP(action, PSTR("Factory Reset"), directAction, (void*)GUI_DIRECT_ACTION_FACTORY_RESET, GUIPageType::ACTION);
    GUI::menuEnd(action);
}

void __attribute__((weak)) mainMenu(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuTextP(action, PSTR("= Main Menu ="), true);
    GUI::menuBack(action);
    GUI::menuSelectableP(action, PSTR("Home"), menuHome, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Move"), menuMove, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Controls"), menuControls, nullptr, GUIPageType::MENU);
#if NUM_FANS > 0
    GUI::menuSelectableP(action, PSTR("Fans"), menuFans, nullptr, GUIPageType::MENU);
#endif
    GUI::menuSelectableP(action, PSTR("Debug"), menuDebug, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Information"), menuInfo, nullptr, GUIPageType::MENU);
    GUI::menuSelectableP(action, PSTR("Configuration"), menuConfig, nullptr, GUIPageType::MENU);
    /* GUI::menuSelectableP(action, PSTR("Warning 1"), warningScreen, (void*)"Test Warning", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Info 1"), infoScreen, (void*)"Test Info", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Error 1"), errorScreen, (void*)"Test Error", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectableP(action, PSTR("Spin"), waitScreen, (void*)"Computing", GUIPageType::BUSY);
    GUI::menuSelectableP(action, PSTR("Spin 2"), waitScreen, (void*)"Computing slowly", GUIPageType::BUSY);
    GUI::menuSelectableP(action, PSTR("Item 10"), nullptr, nullptr, GUIPageType::POP); */
    GUI::menuEnd(action);
}
