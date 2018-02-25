#include "../../Repetier.h"

#ifndef NUM_TOOLS
#error "Number of tools NUM_TOOLS not defined."
#endif

#if NUM_TOOLS < 1
#error "You need at least one tool to compile firmware"
#endif

HeatManager* heatedBeds[] = HEATED_BED_LIST;

fast8_t Tool::activeToolId = 255;
Tool* Tool::activeTool = nullptr;
Tool* Tool::tools[] = TOOLS;

void Tool::selectTool(fast8_t id) {
    if (id < 0 || id >= NUM_TOOLS) {
        Com::printErrorF(PSTR("Illegal tool number selected:"));
        Com::print((int)id);
        Com::println();
        return;
    }
    if (activeTool != nullptr && activeToolId == id) {
        activeTool->updateDerived(); // reset values
        return;                      // already selected
    }
    Com::printFLN(PSTR("SelectExtruder:"), static_cast<int>(id));
    float zOffset = -Printer::offsetZ; // opposite sign to extruder offset!
#if RAISE_Z_ON_TOOLCHANGE > 0
    float lastZ = Motion1::currentPosition[X_AXIS];
    Motion1::setTmpPositionXYZ(lastZ + RAISE_Z_ON_TOOLCHANGE, IGNORE_COORDINATE, IGNORE_COORDINATE);
    Motion1::moveByOfficial(Motion1::tmpPosition, Z_SPEED);
    Motion1::waitForEndOfMoves();
#endif
    float zOffsetNew = tools[id]->getOffsetZ();
    if (activeTool != nullptr) {
        if (zOffsetNew < zOffset) { // will hit bed, activate early
            Motion1::setToolOffset(-activeTool->getOffsetX(), -activeTool->getOffsetY(), -zOffsetNew);
        }
        activeTool->deactivate();
        PrinterType::deactivatedTool(activeToolId);
    }
    Tool::activeToolId = id;
    Tool::activeTool = tools[id];
    Motion1::advanceK = 0;
    Motion2::advanceSteps = 0;
    activeTool->activate();
    PrinterType::activatedTool(activeToolId);
    Motion1::setToolOffset(-activeTool->getOffsetX(), -activeTool->getOffsetY(), -activeTool->getOffsetZ());
#if RAISE_Z_ON_TOOLCHANGE > 0
    float lastZ = Motion1::currentPosition[X_AXIS];
    Motion1::setTmpPositionXYZ(lastZ, IGNORE_COORDINATE, IGNORE_COORDINATE);
    Motion1::moveByOfficial(Motion1::tmpPosition, Z_SPEED);
    Motion1::waitForEndOfMoves();
#endif
}

Tool* Tool::getTool(fast8_t id) {
    if (id < 0 || id >= NUM_TOOLS) {
        return nullptr;
    }
    return tools[id];
}

void Tool::initTools() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        tools[i]->init();
    }
}

void Tool::disableMotors() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        tools[i]->disableMotor();
    }
}

void Tool::eepromHandleTools() {
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        EEPROM::handlePrefix(PSTR("Tool"), i + 1);
        tools[i]->eepromHandle();
    }
    EEPROM::removePrefix();
}

void Tool::eepromHandle() {
    EEPROM::handleFloat(eepromStart, PSTR("X Offset [mm]"), 3, offsetX);
    EEPROM::handleFloat(eepromStart + 4, PSTR("Y Offset [mm]"), 3, offsetY);
    EEPROM::handleFloat(eepromStart + 8, PSTR("Z Offset [mm]"), 3, offsetZ);
}

void Tool::updateDerivedTools() {
    if (activeTool != nullptr) {
        activeTool->updateDerived();
    }
}
/// Called when the tool gets activated.
void ToolExtruder::activate() {
    Motion1::setMotorForAxis(stepper, E_AXIS);
    Motion1::maxYank[E_AXIS] = yank;
    Motion1::resolution[E_AXIS] = stepsPerMM;
    Motion1::currentPosition[E_AXIS] = Motion1::currentPositionTransformed[E_AXIS] = 0.0f;
    Motion1::advanceK = advance;
    GCode::executeFString(startScript);
    Motion1::waitForEndOfMoves();
}

/// Gets called when the tool gets disabled.
void ToolExtruder::deactivate() {
    GCode::executeFString(endScript);
    Motion1::setMotorForAxis(nullptr, E_AXIS);
}

/// Called on kill/emergency to disable the tool
void ToolExtruder::shutdown() {
    heater->setTargetTemperature(0);
    stepper->disable();
}

void ToolExtruder::init() {
    setEepromStart(EEPROM::reserve(2, 1, Tool::eepromSize() + 6 * 4));
}

void ToolExtruder::eepromHandle() {
    Tool::eepromHandle();
    uint pos = getEepromStart() + Tool::eepromSize();
    EEPROM::handleFloat(pos, PSTR("Steps per mm [steps/mm]"), 2, stepsPerMM);
    EEPROM::handleFloat(pos + 4, PSTR("Yank [mm/s]"), 2, yank);
    EEPROM::handleFloat(pos + 8, PSTR("Max Speed [mm/s]"), 2, maxSpeed);
    EEPROM::handleFloat(pos + 12, PSTR("Acceleration [mm/s^3]"), 2, acceleration);
    EEPROM::handleFloat(pos + 16, PSTR("Advance [steps/mm]"), 2, advance);
    EEPROM::handleFloat(pos + 20, PSTR("Diameter [mm]"), 2, diameter);
}

void ToolExtruder::setAdvance(float adv) {
    advance = adv;
    if (this == Tool::getActiveTool()) {
        Motion1::advanceK = advance;
    }
}

void ToolExtruder::updateDerived() {
    Motion1::advanceK = advance;
    Tool* t = Tool::getActiveTool();
    Motion1::setMotorForAxis(stepper, E_AXIS);
    Motion1::setToolOffset(-t->getOffsetX(), -t->getOffsetY(), -t->getOffsetZ());
}

void ToolExtruder::disableMotor() {
    stepper->disable();
}
