#include "../Repetier.h"

template <class driver>
void AdjustResolutionStepperDriver<driver>::menuStepsPerMM(GUIAction action, void* data) {
    AdjustResolutionStepperDriver* stepper = reinterpret_cast<AdjustResolutionStepperDriver*>(data);
    DRAW_LONG_P(PSTR("Resolution:"), Com::tUnitStepsPerMM, stepper->to);
    if (GUI::handleLongValueAction(action, v, 0, stepper->from, 1)) {
        stepper->to = v;
    }
}

// Configuration in GUI
template <class driver>
void AdjustResolutionStepperDriver<driver>::menuConfig(GUIAction action, void* data) {
    GUI::menuLongP(action, PSTR("Resolution:"), to, AdjustResolutionStepperDriver<driver>::menuStepsPerMM, this, GUIPageType::FIXED_CONTENT);
}

template <class driver>
void AdjustResolutionStepperDriver<driver>::init() {
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_STEPPER, 1, 4);
}

template <class driver>
void AdjustResolutionStepperDriver<driver>::eepromHandle() {
    EEPROM::handleLong(eprStart, PSTR("Resolution [steps/mm]"), to);
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TOOLS_TEMPLATES
#include "../io/redefine.h"
