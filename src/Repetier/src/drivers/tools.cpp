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
        return; // already selected
    }
    Com::printFLN(PSTR("SelectExtruder:"), static_cast<int>(id));
    float zOffset = Printer::offsetZ;
#if RAISE_Z_ON_TOOLCHANGE > 0
    float lastZ = Motion1::currentPosition[X_AXIS];
    Motion1::setTmpPositionXYZ(lastZ + RAISE_Z_ON_TOOLCHANGE, IGNORE_COORDINATE, IGNORE_COORDINATE);
    Motion1::moveByOfficial(Motion1::tmpPosition, Z_SPEED);
    Motion1::waitForEndOfMoves();
#endif
    if (activeTool != nullptr) {
        activeTool->deactivate();
        PrinterType::deactivatedTool(activeToolId);
    }
    Tool::activeToolId = id;
    Tool::activeTool = tools[id];
    if (activeTool->getOffsetZ() < zOffset) { // will hit bed, activate early
        Motion1::setToolOffset(activeTool->getOffsetX(), activeTool->getOffsetY(), activeTool->getOffsetZ());
    }
    activeTool->activate();
    PrinterType::activatedTool(activeToolId);
    if (activeTool->getOffsetZ() >= zOffset) {
        Motion1::setToolOffset(activeTool->getOffsetX(), activeTool->getOffsetY(), activeTool->getOffsetZ());
    }
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

/// Called when the tool gets activated.
void ToolExtruder::activate() {
    Motion1::setMotorForAxis(stepper, E_AXIS);
    Motion1::maxYank[E_AXIS] = yank;
    Motion1::resolution[E_AXIS] = stepsPerMM;
    Motion1::currentPosition[E_AXIS] = Motion1::currentPositionTransformed[E_AXIS] = 0.0f;
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
