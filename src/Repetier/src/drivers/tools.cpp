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
Tool* const Tool::tools[NUM_TOOLS] = TOOLS;

void Tool::unselectTool() {
    if (activeTool == nullptr) {
        return;
    }
    Motion1::setToolOffset(0, 0, 0);
    activeTool->deactivate();
    DEBUG_MSG("UT3");
    PrinterType::deactivatedTool(activeToolId);
    DEBUG_MSG("UT4");
    activeTool = nullptr;
    activeToolId = 255;
}

void Tool::selectTool(fast8_t id, bool force) {
    if (Motion1::dittoMode) { // in ditto mode 0 is always active!
        id = 0;
    }
    if (id < 0 || id >= NUM_TOOLS) {
        Com::printErrorF(PSTR("Illegal tool number selected:"));
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
    Com::printFLN(PSTR("SelectTool:"), static_cast<int>(id));
    float zOffset = -Motion1::toolOffset[Z_AXIS]; // opposite sign to extruder offset!
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
    Motion1::advanceK = 0; // Gets activated by tool activation if supported!
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

void ToolExtruder::reset(float offx, float offy, float offz, float diameter, float resolution, float yank, float maxSpeed, float acceleration, float advance) {
    resetBase(offx, offy, offz);
    this->diameter = diameter;
    stepsPerMM = resolution;
    this->yank = yank;
    this->acceleration = acceleration;
    this->maxSpeed = maxSpeed;
    this->advance = advance;
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

void ToolExtruder::enableMotor() {
    stepper->enable();
}
void ToolExtruder::stepMotor() {
    stepper->step();
}
void ToolExtruder::unstepMotor() {
    stepper->unstep();
}
bool ToolExtruder::stepCondMotor() {
    return stepper->stepCond();
}
