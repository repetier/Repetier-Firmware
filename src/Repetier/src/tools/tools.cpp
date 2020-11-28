#include "Repetier.h"

#ifndef NUM_TOOLS
#error "Number of tools NUM_TOOLS not defined."
#endif

#if NUM_TOOLS < 1
#error "You need at least one tool to compile firmware"
#endif

HeatManager* heatedBeds[] = HEATED_BED_LIST;
HeatManager* heatedChambers[] = HEATED_CHAMBER_LIST;
constexpr int numHeatedBeds = std::extent<decltype(heatedBeds)>::value;
static_assert(numHeatedBeds == NUM_HEATED_BEDS, "NUM_HEATED_BEDS not defined correctly");
constexpr int numHeatedChambers = std::extent<decltype(heatedChambers)>::value;
static_assert(numHeatedChambers == NUM_HEATED_CHAMBERS, "NUM_HEATED_CHAMBERS not defined correctly");

fast8_t Tool::activeToolId = 255;
Tool* Tool::activeTool = nullptr;
Tool* const Tool::tools[] = TOOLS;

ToolChangeCustomEvent::ToolChangeCustomEvent(Tool* tool) {
    if (tool) {
        tool->setToolChangeHandler(this);
    }
}

void ToolChangeCustomEvent::M6(GCode* com, Tool* _tool) {
    EVENT_CUSTOM_TOOL_CHANGE_M6(com, _tool);
}

void ToolChangeCustomEvent::setup(Tool* _tool) {
    EVENT_CUSTOM_TOOL_CHANGE_SETUP(_tool);
}

void ToolChangeCustomEvent::activate(Tool* tool) {
    EVENT_CUSTOM_TOOL_CHANGE_ACTIVATE(_tool);
}

void ToolChangeCustomEvent::deactivate(Tool* tool) {
    EVENT_CUSTOM_TOOL_CHANGE_DEACTIVATE(_tool);
}

ToolChangeServo::ToolChangeServo(Tool* tool, ServoInterface* _servo, int16_t _defaultPosition, int32_t _timeout) {
    servo = _servo;
    position = _defaultPosition;
    timeout = _timeout;
    if (tool) {
        tool->setToolChangeHandler(this);
    }
}

void ToolChangeServo::M6(GCode* com, Tool* tool) {
    servo->setPosition(position, timeout);
    HAL::delayMilliseconds(timeout);
}

void ToolChangeServo::activate(Tool* tool) {
    servo->setPosition(position, timeout);
    HAL::delayMilliseconds(timeout);
}

int ToolChangeServo::eepromSize() {
    return 2;
}

void ToolChangeServo::eepromHandle(int pos) {
    EEPROM::handleInt(pos, PSTR("Servo position [us]"), position);
}

void ToolChangeServo::configMenu(GUIAction action) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    GUI::menuLongP(action, PSTR("Servo Pos. :"), position, menuCHServoPosition, this, GUIPageType::FIXED_CONTENT);
#endif
}

#if FEATURE_CONTROLLER != NO_CONTROLLER
void __attribute__((weak)) menuCHServoPosition(GUIAction action, void* data) {
    ToolChangeServo* ext = reinterpret_cast<ToolChangeServo*>(data);
    int32_t pos = ext->position;
    DRAW_LONG_P(PSTR("Servo Pos.:"), Com::tUnitStepsPerMM, pos);
    if (GUI::handleLongValueAction(action, v, 0, 2000, 1)) {
        ext->position = v;
    }
}
#endif

ToolChangeMerge::ToolChangeMerge(Tool* tool, ToolChangeHandler* _t1, ToolChangeHandler* _t2) {
    t1 = _t1;
    t2 = _t2;
    if (tool) {
        tool->setToolChangeHandler(this);
    }
}

void ToolChangeMerge::M6(GCode* com, Tool* tool) {
    t1->M6(com, tool);
    t2->M6(com, tool);
}

void ToolChangeMerge::activate(Tool* tool) {
    t1->activate(tool);
    t2->activate(tool);
}

void ToolChangeMerge::deactivate(Tool* tool) {
    t1->deactivate(tool);
    t2->deactivate(tool);
}

int ToolChangeMerge::eepromSize() {
    return t1->eepromSize() + t2->eepromSize();
}

void ToolChangeMerge::eepromHandle(int pos) {
    t1->eepromHandle(pos);
    t2->eepromHandle(pos + t1->eepromSize());
}

void ToolChangeMerge::configMenu(GUIAction action) {
    t1->configMenu(action);
    t2->configMenu(action);
}

ToolChangeLink::ToolChangeLink(Tool* tool, ToolChangeHandler* _t) {
    t = _t;
    if (tool) {
        tool->setToolChangeHandler(this);
    }
}
void ToolChangeLink::M6(GCode* com, Tool* tool) {
    t->M6(com, tool);
}
void ToolChangeLink::activate(Tool* tool) {
    t->activate(tool);
}
void ToolChangeLink::deactivate(Tool* tool) {
    t->deactivate(tool);
}

void __attribute__((weak)) menuToolOffsetXFine(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset X (0.01mm):"), Com::tUnitMM, ext->getOffsetX(), 2);
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 0.01)) {
        ext->setOffsetForAxis(X_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuToolOffsetX(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset X (1mm):"), Com::tUnitMM, ext->getOffsetX(), 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuToolOffsetXFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 1)) {
        ext->setOffsetForAxis(X_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuToolOffsetYFine(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset Y (0.01mm):"), Com::tUnitMM, ext->getOffsetY(), 2);
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 0.01)) {
        ext->setOffsetForAxis(Y_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuToolOffsetY(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset Y (1mm):"), Com::tUnitMM, ext->getOffsetY(), 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuToolOffsetYFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 1)) {
        ext->setOffsetForAxis(Y_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuToolOffsetZFine(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset Z (0.01mm):"), Com::tUnitMM, ext->getOffsetZ(), 2);
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 0.01)) {
        ext->setOffsetForAxis(Z_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuToolOffsetZ(GUIAction action, void* data) {
    Tool* ext = reinterpret_cast<Tool*>(data);
    DRAW_FLOAT_P(PSTR("Offset Z (1mm):"), Com::tUnitMM, ext->getOffsetZ(), 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuToolOffsetZFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, 0, 100000, 1)) {
        ext->setOffsetForAxis(Z_AXIS, v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderStepsPerMMFine(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Resolution Fine:"), Com::tUnitStepsPerMM, ext->getResolution(), 2);
    if (GUI::handleFloatValueAction(action, v, 0.0f, 100000.0f, 0.01f)) {
        ext->setResolution(v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderStepsPerMM(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Resolution Coarse:"), Com::tUnitStepsPerMM, ext->getResolution(), 2);
    if (action == GUIAction::CLICK) { // catch default action
        GUI::replace(menuExtruderStepsPerMMFine, data, GUIPageType::FIXED_CONTENT);
        return;
    }
    if (GUI::handleFloatValueAction(action, v, 0.0f, 100000.0f, 1.0f)) {
        ext->setResolution(v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderMaxSpeed(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Max. Speed:"), Com::tUnitMMPS, ext->getMaxSpeed(), 0);
    if (GUI::handleFloatValueAction(action, v, 1, 1000, 1)) {
        ext->setMaxSpeed(v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderMaxAcceleration(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Max. Acceleration:"), Com::tUnitMMPS2, ext->getAcceleration(), 0);
    if (GUI::handleFloatValueAction(action, v, 50, 20000, 50)) {
        ext->setAcceleration(v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderMaxYank(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Max. Jerk"), Com::tUnitMMPS, ext->getMaxYank(), 1);
    if (GUI::handleFloatValueAction(action, v, 0.1, 100, 0.1)) {
        ext->setMaxYank(v);
        Tool::updateDerivedTools();
    }
}

void __attribute__((weak)) menuExtruderFilamentDiameter(GUIAction action, void* data) {
    ToolExtruder* ext = reinterpret_cast<ToolExtruder*>(data);
    DRAW_FLOAT_P(PSTR("Fil. Diameter"), Com::tUnitMM, ext->getDiameter(), 2);
    if (GUI::handleFloatValueAction(action, v, 0.1, 100, 0.01)) {
        ext->setDiameter(v);
        Tool::updateDerivedTools();
    }
}

void Tool::unselectTool() {
    if (activeTool == nullptr) {
        return;
    }
    Motion1::setToolOffset(0, 0, 0);
    activeTool->deactivate();
    PrinterType::deactivatedTool(activeToolId);
    activeTool = nullptr;
    activeToolId = 255;
}

void Tool::selectTool(fast8_t id, bool force) {
    bool doMove = activeToolId != 255;
    if (Printer::isZProbingActive()) { // stop probing when selecting a tool
        ZProbeHandler::deactivate();
    }
    // Test for valid tool id
    if (id < 0 || id >= NUM_TOOLS || !PrinterType::canSelectTool(id)) {
        Com::printWarningF(PSTR("Illegal tool number selected:"));
        Com::print((int)id);
        Com::println();
        if (activeTool != nullptr) {
            Com::printFLN(PSTR("SelectTool:"), static_cast<int>(activeToolId));
        }
        return;
    }
    if (!force && activeTool != nullptr && activeToolId == id) {
        activeTool->updateDerived(); // reset values
        return;                      // already selected
    }
    // Report new tool
    Com::printFLN(PSTR("SelectTool:"), static_cast<int>(id));
    float zOffset = -Motion1::toolOffset[Z_AXIS]; // opposite sign to extruder offset!
#if RAISE_Z_ON_TOOLCHANGE > 0
    float lastZ = Motion1::currentPosition[Z_AXIS];
    if (doMove && Motion1::isAxisHomed(Z_AXIS)) {
        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::min(lastZ + RAISE_Z_ON_TOOLCHANGE, Motion1::maxPos[Z_AXIS]));
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        Motion1::waitForEndOfMoves();
    }
#endif
    float zOffsetNew = tools[id]->getOffsetZ();
    if (activeTool != nullptr && Motion1::dittoMode == 0) {
        if (zOffsetNew < zOffset) { // will hit bed, activate early
            Motion1::setToolOffset(-activeTool->offsetX, -activeTool->offsetY, -zOffsetNew);
        }
        activeTool->deactivate();
        PrinterType::deactivatedTool(activeToolId);
    }
    // From here on new tool is active
    Tool::activeToolId = id;
    Tool::activeTool = tools[id];
    Motion1::advanceK = 0; // Gets activated by tool activation if supported!
    Motion2::advanceSteps = 0;
    if (Motion1::dittoMode == 0) {
        activeTool->activate();
        PrinterType::activatedTool(activeToolId);
        if (doMove) {
            Motion1::setToolOffset(-activeTool->offsetX, -activeTool->offsetY, -activeTool->offsetZ);
        } else {
            Motion1::currentPosition[X_AXIS] += -activeTool->offsetX - Motion1::toolOffset[X_AXIS];
            Motion1::currentPosition[Y_AXIS] += -activeTool->offsetY - Motion1::toolOffset[Y_AXIS];
            Motion1::currentPosition[Z_AXIS] += -activeTool->offsetZ - Motion1::toolOffset[Z_AXIS];
            Motion1::updatePositionsFromCurrent();
        }
    }
#if RAISE_Z_ON_TOOLCHANGE > 0
    if (doMove && Motion1::isAxisHomed(Z_AXIS)) {
        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, lastZ);
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        Motion1::waitForEndOfMoves();
    }
#endif
    PrinterType::toolchangeFinished();
}

void Tool::resetBase(float offX, float offY, float offZ) {
    offsetX = offX;
    offsetY = offY;
    offsetZ = offZ;
}

/* Tool* Tool::getTool(fast8_t id) {
    if (id < 0 || id >= NUM_TOOLS) {
        return nullptr;
    }
    return tools[id];
} */

void Tool::initTools() {
    constexpr int numTools = std::extent<decltype(Tool::tools)>::value;
    static_assert(numTools == NUM_TOOLS, "NUM_TOOLS not defined correctly");

    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        tools[i]->setToolId(i);
        tools[i]->init();
    }
}

void Tool::disableMotors() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        tools[i]->disableMotor();
    }
}

void Tool::enableMotors() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        tools[i]->enableMotor();
    }
}
void Tool::pauseHeaters() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        HeatManager* hm = tools[i]->getHeater();
        if (hm) {
            hm->pause();
        }
    }
}
void Tool::unpauseHeaters() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        HeatManager* hm = tools[i]->getHeater();
        if (hm) {
            hm->unpause();
        }
    }
}
void Tool::waitForTemperatures() {
    GUI::setStatusP(PSTR("Heating..."), GUIStatusLevel::BUSY); // inform user
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        HeatManager* hm = tools[i]->getHeater();
        if (hm) {
            hm->waitForTargetTemperature();
        }
    }
    GUI::popBusy();
}

void Tool::eepromHandleTools() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        EEPROM::handlePrefix(PSTR("Tool "), i + 1);
        tools[i]->eepromHandle();
    }
    EEPROM::removePrefix();
}

void Tool::eepromHandle() {
#if PRINTER_TYPE != PRINTER_TYPE_DUAL_X
    EEPROM::handleFloat(eepromStart, PSTR("X Offset [mm]"), 3, offsetX);
#endif
    EEPROM::handleFloat(eepromStart + 4, PSTR("Y Offset [mm]"), 3, offsetY);
    EEPROM::handleFloat(eepromStart + 8, PSTR("Z Offset [mm]"), 3, offsetZ);
    int pos = eepromStart + 12;
    if (changeHandler) {
        changeHandler->eepromHandle(pos);
        pos += changeHandler->eepromSize();
    }
    if (coolantHandler) {
        coolantHandler->eepromHandle(pos);
        // pos += coolantHandler->eepromSize();
    }
}

int Tool::eepromSize() {
    int size = 12;
    if (changeHandler) {
        size += changeHandler->eepromSize();
    }
    if (coolantHandler) {
        size += coolantHandler->eepromSize();
    }
    return size;
}
void Tool::updateDerivedTools() {
    if (activeTool != nullptr) {
        activeTool->updateDerived();
        activeTool->copySettingsToMotion1();
    }
}

float Tool::getOffsetForAxis(fast8_t axis) {
    switch (axis) {
    case X_AXIS:
        return offsetX;
    case Y_AXIS:
        return offsetY;
    case Z_AXIS:
        return offsetZ;
    }
    return 0;
}

void Tool::setOffsetForAxis(fast8_t axis, float offset) {
    switch (axis) {
    case X_AXIS:
        offsetX = offset;
        break;
    case Y_AXIS:
        offsetY = offset;
        break;
    case Z_AXIS:
        offsetZ = offset;
        break;
    }
}

void Tool::minMaxOffsetForAxis(fast8_t axis, float& min, float& max) {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        float val = 0;
        switch (axis) {
        case X_AXIS:
            val = tools[i]->offsetX;
            break;
        case Y_AXIS:
            val = tools[i]->offsetY;
            break;
        case Z_AXIS:
            val = tools[i]->offsetZ;
            break;
        }
        min = i ? RMath::min(min, val) : val;
        max = i ? RMath::max(max, val) : val;
    }
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"
