#include "../Repetier.h"

struct TMCChopperTiming {
    uint8_t toff;
    int8_t hend;
    uint8_t hstrt;
};

static constexpr int8_t TMCStallguardMin = -64,
                        TMCStallguardMax = 63;

static TMCChopperTiming tmcChopperTiming = TMC_CHOPPER_TIMING;

static PGM_P const motorNames[NUM_MOTORS] PROGMEM = MOTOR_NAMES;

int StepperDriverBase::motorIndex() {
    for (fast8_t i = 0; i < NUM_MOTORS; i++) {
        if (this == Motion1::drivers[i]) {
            return i;
        }
    }
    return 0;
}

void StepperDriverBase::printMotorName() {
    int idx = motorIndex();
    PGM_P* adr = (PGM_P*)pgm_read_word(&motorNames);
    Com::printF((const char*)pgm_read_word(&adr[idx]));
}

void StepperDriverBase::printMotorNumberAndName(bool newline) {
    Com::printF(PSTR("Motor "), motorIndex());
    Com::printF(PSTR(": "));
    printMotorName();
    if (newline) {
        Com::println();
    }
}

template <class driver>
void __attribute__((weak)) AdjustResolutionStepperDriver<driver>::menuStepsPerMM(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    AdjustResolutionStepperDriver* stepper = reinterpret_cast<AdjustResolutionStepperDriver*>(data);
    DRAW_LONG_P(PSTR("Resolution:"), Com::tUnitStepsPerMM, stepper->to);
    if (GUI::handleLongValueAction(action, v, 0, stepper->from, 1)) {
        stepper->to = v;
    }
#endif
}

// Configuration in GUI
template <class driver>
void __attribute__((weak)) AdjustResolutionStepperDriver<driver>::menuConfig(GUIAction action, void* data) {
#if FEATURE_CONTROLLER != NO_CONTROLLER
    GUI::menuLongP(action, PSTR("Resolution:"), to, AdjustResolutionStepperDriver<driver>::menuStepsPerMM, this, GUIPageType::FIXED_CONTENT);
#endif
}

template <class driver>
void AdjustResolutionStepperDriver<driver>::init() {
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_STEPPER, 1, 4);
}

template <class driver>
void AdjustResolutionStepperDriver<driver>::eepromHandle() {
    EEPROM::handleLong(eprStart, PSTR("Resolution [steps/mm]"), to);
}

#define PSB_MICROSTEP_POS 0
#if STORE_MOTOR_MICROSTEPPING
#define PSB_MICROSTEP_SIZE 2
#else
#define PSB_MICROSTEP_SIZE 0
#endif

#define PSB_CURRENT_POS PSB_MICROSTEP_POS + PSB_MICROSTEP_SIZE
#if STORE_MOTOR_CURRENT
#define PSB_CURRENT_SIZE 2
#else
#define PSB_CURRENT_SIZE 0
#endif

#define PSB_HYBRID_POS PSB_CURRENT_POS + PSB_CURRENT_SIZE
#if STORE_MOTOR_HYBRID_TRESHOLD
#define PSB_HYBRID_SIZE 4
#else
#define PSB_HYBRID_SIZE 0
#endif

#define PSB_STEALTH_POS PSB_HYBRID_POS + PSB_HYBRID_SIZE
#if STORE_MOTOR_STEALTH
#define PSB_STEALTH_SIZE 1
#else
#define PSB_STEALTH_SIZE 0
#endif

#define PSB_STALL_SENSITIVITY_POS PSB_STEALTH_POS + PSB_STEALTH_SIZE
#if STORE_MOTOR_STALL_SENSITIVITY
#define PSB_STALL_SENSITIVITY_SIZE 2
#else
#define PSB_STALL_SENSITIVITY_SIZE 0
#endif

#define PSB_EEPROM_SIZE PSB_STALL_SENSITIVITY_POS + PSB_STALL_SENSITIVITY_SIZE

void ProgrammableStepperBase::reserveEEPROM(uint16_t extraBytes) {
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_STEPPER, 1, PSB_EEPROM_SIZE + extraBytes);
}

void ProgrammableStepperBase::processEEPROM(uint8_t flags) {
#if STORE_MOTOR_MICROSTEPPING
    if (flags & 1) {
        EEPROM::handleInt(eprStart + PSB_MICROSTEP_POS, PSTR("Microsteps"), microsteps);
    }
#endif
#if STORE_MOTOR_CURRENT
    if (flags & 2) {
        EEPROM::handleInt(eprStart + PSB_CURRENT_POS, PSTR("RMS Current [mA]"), currentMillis);
    }
#endif
#if STORE_MOTOR_HYBRID_TRESHOLD
    if (flags & 4) {
        EEPROM::handleFloat(eprStart + PSB_HYBRID_POS, PSTR("Hybrid Treshold [mm/s]"), 1, hybridSpeed);
    }
#endif
#if STORE_MOTOR_STEALTH
    if (flags & 8) {
        EEPROM::handleByte(eprStart + PSB_STEALTH_POS, PSTR("Stealth [0/1]"), (uint8_t&)stealthChop);
    }
#endif
#if STORE_MOTOR_STALL_SENSITIVITY
    //Note, flag 32 is stallguard 4, we don't use that flag for anything else yet, other than a small change in eeprom text.
    if (flags & 16 && stallguardSensitivity != -128) {
        EEPROM::handleInt(eprStart + PSB_STALL_SENSITIVITY_POS, (flags & 32) ? 
                PSTR("Stall Sensitivity [0..255]") : PSTR("Stall Sensitivity [-64..63]"), stallguardSensitivity);
    }
#endif
}

void reportTMC2130(TMC2130Stepper* driver, ProgrammableStepperBase* b, int level) {
    uint8_t constat = driver->test_connection();
    Com::printF(PSTR("Status: "));
    switch (constat) {
    case 0:
        Com::printFLN(PSTR("Ok"));
        break;
    case 1:
        Com::printFLN(PSTR("No connection"));
        break;
    case 2:
        Com::printFLN(PSTR("No Power"));
        break;
    }
    Com::printFLN(PSTR("Enabled: "), driver->isEnabled(), BoolFormat::YESNO);
    Com::printF(PSTR("Current [mA]: "), driver->rms_current());
    Com::printFLN(PSTR(" set: "), b->getCurrentMillis());
    Com::printFLN(PSTR("Max. current [mA]: "), 1.4142 * driver->rms_current(), 0);
    Com::printF(PSTR("Microsteps: "), driver->microsteps());
    Com::printFLN(PSTR(" mres:"), (int)driver->mres());
    Com::printFLN(PSTR("StealthChop: "), driver->en_pwm_mode(), BoolFormat::ONOFF);
    if (b->getHybridSpeed() > 0) {
        Com::printFLN(PSTR("Hybrid treshold [mm/s]: "), b->getHybridSpeed(), 2);
    } else {
        Com::printFLN(PSTR("Hybrid mode disabled"));
    }
    if (level > 0) {
        const uint32_t tstep = driver->TSTEP();
        Com::printF(PSTR("TSTEP: "));
        if (tstep != 0xFFFFF) {
            Com::print(static_cast<int32_t>(tstep));
            Com::println();
        } else {
            Com::printFLN("max");
        }
        Com::printFLN(PSTR("TPWMTHRS: "), driver->TPWMTHRS());
        Com::printFLN(PSTR("TPOWERDOWN: "), (int)driver->TPOWERDOWN());
        Com::printF(PSTR("IRUN: "), driver->irun());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("IHOLD: "), driver->ihold());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("CS Actual: "), driver->cs_actual());
        Com::printFLN(PSTR("/31"));
        Com::printFLN(PSTR("vsense: "), driver->vsense());
        Com::printFLN(PSTR("toff: "), (int)driver->toff());
        Com::printFLN(PSTR("hstart: "), (int)driver->hysteresis_start());
        Com::printFLN(PSTR("hend: "), (int)driver->hysteresis_end());
        Com::printF(PSTR("Blank time: "), (int)driver->blank_time());
        Com::printFLN(PSTR(" tbl: "), (int)driver->tbl());
        Com::printFLN(PSTR("IOIN: "), driver->IOIN());
        Com::printFLN(PSTR("GSTAT: "), (int)driver->GSTAT());
    }
    if (level > 1) {
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::init() {
    disable();
    // driver->setSPISpeed(500000UL);
    driver->begin();
    CHOPCONF_t chopconf { 0 };
    chopconf.tbl = 1;
    chopconf.toff = tmcChopperTiming.toff;
    chopconf.intpol = TMC_INTERPOLATE;
    chopconf.hend = tmcChopperTiming.hend + 3;   // -3 .. 12
    chopconf.hstrt = tmcChopperTiming.hstrt - 1; // 1 .. 16 but hend+hstart <= 16
                                                 // #if ENABLED(TMC_SQUARE_WAVE_STEPPING)
                                                 //    chopconf.dedge = true;
                                                 // #endif
    driver->CHOPCONF(chopconf.sr);

    driver->rms_current(currentMillis, TMC_HOLD_MULTIPLIER);
    driver->microsteps(microsteps);
    driver->iholddelay(10);
    driver->TPOWERDOWN(128); // ~2s until driver lowers to hold current

    driver->en_pwm_mode(stealthChop);

    PWMCONF_t pwmconf { 0 };
    pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_grad = 5;
    pwmconf.pwm_ampl = 180;
    driver->PWMCONF(pwmconf.sr);

    if (hybridSpeed >= 0) {
        driver->TPWMTHRS(fclk * microsteps / static_cast<uint32_t>(256 * hybridSpeed * Motion1::resolution[getAxis()])); // need computed
    } else {
        driver->TPWMTHRS(0); // Only stealthChop or spreadCycle
    }

    if (stallguardSensitivity != -128) {
        stallguardSensitivity = constrain(stallguardSensitivity, -64, 63);
        driver->sgt(stallguardSensitivity);
    }
    driver->GSTAT(); // Clear GSTAT
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::eepromReserve() {
    reserveEEPROM(0);
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::eepromHandle() {
    PGM_P* adr = (PGM_P*)pgm_read_word(&motorNames);
    EEPROM::handlePrefix(adr[motorIndex()]);
    processEEPROM(31);
    EEPROM::removePrefix();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::reset(uint16_t _microsteps, uint16_t _currentMillis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity) {
    microsteps = _microsteps;
    currentMillis = _currentMillis;
    stealthChop = _stealthChop;
    hybridSpeed = _hybridThrs;
    stallguardSensitivity = _stallSensitivity;
    init();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::timer500ms() {
    if (debug != -1) {
        reportTMC2130(driver, this, debug);
    }
    // Access of bits results in reading register again, so we buffer result
    TMC2130_n::DRV_STATUS_t status { driver->DRV_STATUS() };
    if (status.sr == 0xFFFFFFFF || status.sr == 0x0) { // not in working state
        return;
    }
    if (status.ot) { // over temperature
        printMotorNumberAndName(false);
        Com::printFLN(" driver overtemperature! Current: ", currentMillis);
    }
    if (status.otpw) { // over temperature prewarn
        if (otpwCount < 255) {
            otpwCount++;
        }
        if (otpwCount == 1) {
            printMotorNumberAndName(false);
            Com::printFLN(" driver overtemperature warning! Current: ", currentMillis);
        }
#if TMC_CURRENT_STEP_DOWN > 0
        if (otpwCount > 4 && driver->isEnabled()) {
            if (currentMillis > 100) {
                currentMillis -= TMC_CURRENT_STEP_DOWN;
                driver->rms_current(currentMillis);
                printMotorNumberAndName(false);
                Com::printFLN(" current decreased to ", currentMillis);
            }
        }
#endif
        otpw = true;
    } else {
        otpwCount = 0;
    }
}

/// Set microsteps. Must be a power of 2.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::setMicrosteps(int _microsteps) {
    microsteps = _microsteps;
    driver->microsteps(microsteps);
    reportTMC2130(driver, this, 0);
}

/// Set max current as range 0..255 or mA depedning on driver
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::setMaxCurrent(int max) {
    currentMillis = max;
    driver->rms_current(max);
    reportTMC2130(driver, this, 0);
}

// Called before homing starts. Can be used e.g. to disable silent mode
// or otherwise prepare for endstop detection.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::beforeHoming() {
    if (hasStallguard()) {
        driver->TCOOLTHRS(0xFFFFF);
        driver->en_pwm_mode(false);
        driver->diag1_stall(true);
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::afterHoming() {
    if (hasStallguard()) {
        driver->TCOOLTHRS(0);
        driver->en_pwm_mode(stealthChop);
        driver->diag1_stall(false);
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2130Driver<stepCls, dirCls, enableCls, fclk>::handleMCode(GCode& com) {
    switch (com.M) {
    case 122: // print debug informations
    {
        Com::println();
        printMotorNumberAndName();
        reportTMC2130(driver, this, com.hasD() ? static_cast<int>(com.D) : 0);
        if (com.hasS()) {
            if (com.S) {
                debug = RMath::max(static_cast<int16_t>(0), RMath::min(static_cast<int16_t>(2), com.hasD() ? static_cast<int16_t>(com.D) : static_cast<int16_t>(0)));
            } else {
                debug = -1;
            }
        }
    } break;
    case 569: // Set steathchop with S
        if (com.hasS()) {
            stealthChop = com.S != 0;
            driver->en_pwm_mode(stealthChop);
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" stealthChop: "), stealthChop, BoolFormat::ONOFF);
        break;
    case 911: // Report TMC prewarn
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" temperature prewarn triggered: "), otpw, BoolFormat::TRUEFALSE);
        break;
    case 912: // Clear prewarn
        otpw = false;
        otpwCount = 0;
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" prewarn flag cleared"));
        break;
    case 913: // Hybrid treshold
        if (com.hasX()) {
            hybridSpeed = com.X;
            init();
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" hybrid treshold: "), hybridSpeed, 1);
        break;
    case 914: // sensorless homing sensitivity
        if (hasStallguard()) {
            if (com.hasS() && com.S >= -64 && com.S <= 63) {
                stallguardSensitivity = static_cast<int8_t>(com.S);
            }
            printMotorNumberAndName(false);
            Com::printFLN(PSTR(" stallguard sensitivity: "), stallguardSensitivity);
        }
        break;
    }
}

// ---- TMC5160 -------------
void reportTMC5160(TMC5160Stepper* driver, ProgrammableStepperBase* b, int level) {
    uint8_t constat = driver->test_connection();
    Com::printF(PSTR("Status: "));
    switch (constat) {
    case 0:
        Com::printFLN(PSTR("Ok"));
        break;
    case 1:
        Com::printFLN(PSTR("No connection"));
        break;
    case 2:
        Com::printFLN(PSTR("No power"));
        break;
    }
    Com::printFLN(PSTR("Enabled: "), driver->isEnabled(), BoolFormat::YESNO);
    Com::printF(PSTR("Current [mA]: "), driver->rms_current());
    Com::printFLN(PSTR(" set: "), b->getCurrentMillis());
    Com::printFLN(PSTR("Max. current [mA]: "), 1.4142 * driver->rms_current(), 0);
    Com::printF(PSTR("Microsteps: "), driver->microsteps());
    Com::printFLN(PSTR(" mres:"), (int)driver->mres());
    Com::printFLN(PSTR("StealthChop: "), driver->en_pwm_mode(), BoolFormat::ONOFF);
    if (b->getHybridSpeed() > 0) {
        Com::printFLN(PSTR("Hybrid treshold [mm/s]: "), b->getHybridSpeed(), 2);
    } else {
        Com::printFLN(PSTR("Hybrid mode disabled"));
    }
    if (level > 0) {
        const uint32_t tstep = driver->TSTEP();
        Com::printF(PSTR("TSTEP: "));
        if (tstep != 0xFFFFF) {
            Com::print(static_cast<int32_t>(tstep));
            Com::println();
        } else {
            Com::printFLN("max");
        }
        Com::printFLN(PSTR("TPWMTHRS: "), driver->TPWMTHRS());
        Com::printFLN(PSTR("TPOWERDOWN: "), (int)driver->TPOWERDOWN());
        Com::printF(PSTR("IRUN: "), driver->irun());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("IHOLD: "), driver->ihold());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("CS Actual: "), driver->cs_actual());
        Com::printFLN(PSTR("/31"));
        // Com::printFLN(PSTR("vsense: "), driver->vsense());
        Com::printFLN(PSTR("toff: "), (int)driver->toff());
        Com::printFLN(PSTR("hstart: "), (int)driver->hysteresis_start());
        Com::printFLN(PSTR("hend: "), (int)driver->hysteresis_end());
        Com::printF(PSTR("Blank time: "), (int)driver->blank_time());
        Com::printFLN(PSTR(" tbl: "), (int)driver->tbl());
        Com::printFLN(PSTR("IOIN: "), driver->IOIN());
        Com::printFLN(PSTR("GSTAT: "), (int)driver->GSTAT());
    }
    if (level > 1) {
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::init() {
    disable();
    // driver->setSPISpeed(500000UL);
    driver->begin();
    CHOPCONF_t chopconf { 0 };
    chopconf.tbl = 1;
    chopconf.toff = tmcChopperTiming.toff;
    chopconf.intpol = TMC_INTERPOLATE;
    chopconf.hend = tmcChopperTiming.hend + 3;   // -3 .. 12
    chopconf.hstrt = tmcChopperTiming.hstrt - 1; // 1 .. 16 but hend+hstart <= 16
                                                 // #if ENABLED(TMC_SQUARE_WAVE_STEPPING)
                                                 //    chopconf.dedge = true;
                                                 // #endif
    driver->CHOPCONF(chopconf.sr);

    driver->rms_current(currentMillis, TMC_HOLD_MULTIPLIER);
    driver->microsteps(microsteps);
    driver->iholddelay(10);
    driver->TPOWERDOWN(128); // ~2s until driver lowers to hold current

    driver->en_pwm_mode(stealthChop);

    TMC2160_n::PWMCONF_t pwmconf { 0 };
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    driver->PWMCONF(pwmconf.sr);

    if (hybridSpeed >= 0) {
        driver->TPWMTHRS(fclk * microsteps / static_cast<uint32_t>(256 * hybridSpeed * Motion1::resolution[getAxis()])); // need computed
    } else {
        driver->TPWMTHRS(0); // Only stealthChop or spreadCycle
    }

    if (stallguardSensitivity != -128) {
        stallguardSensitivity = constrain(stallguardSensitivity, -64, 63);
    }
    driver->GSTAT(); // Clear GSTAT
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::eepromReserve() {
    reserveEEPROM(0);
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::eepromHandle() {
    PGM_P* adr = (PGM_P*)pgm_read_word(&motorNames);
    EEPROM::handlePrefix(adr[motorIndex()]);
    processEEPROM(31);
    EEPROM::removePrefix();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::reset(uint16_t _microsteps, uint16_t _currentMillis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity) {
    microsteps = _microsteps;
    currentMillis = _currentMillis;
    stealthChop = _stealthChop;
    hybridSpeed = _hybridThrs;
    stallguardSensitivity = _stallSensitivity;
    init();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::timer500ms() {
    if (debug != -1) {
        reportTMC5160(driver, this, debug);
    }
    // Access of bits results in reading register again, so we buffer result
    TMC2130_n::DRV_STATUS_t status { driver->DRV_STATUS() };
    if (status.sr == 0xFFFFFFFF || status.sr == 0x0) { // not in working state
        return;
    }
    if (status.ot) { // over temperature
        printMotorNumberAndName(false);
        Com::printFLN(" driver overtemperature! Current: ", currentMillis);
    }
    if (status.otpw) { // over temperature prewarn
        if (otpwCount < 255) {
            otpwCount++;
        }
        if (otpwCount == 1) {
            printMotorNumberAndName(false);
            Com::printFLN(" driver overtemperature warning! Current: ", currentMillis);
        }
#if TMC_CURRENT_STEP_DOWN > 0
        if (otpwCount > 4 && driver->isEnabled()) {
            if (currentMillis > 100) {
                currentMillis -= TMC_CURRENT_STEP_DOWN;
                driver->rms_current(currentMillis);
                printMotorNumberAndName(false);
                Com::printFLN(" current decreased to ", currentMillis);
            }
        }
#endif
        otpw = true;
    } else {
        otpwCount = 0;
    }
}

/// Set microsteps. Must be a power of 2.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::setMicrosteps(int _microsteps) {
    microsteps = _microsteps;
    driver->microsteps(microsteps);
    reportTMC5160(driver, this, 0);
}

/// Set max current as range 0..255 or mA depedning on driver
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::setMaxCurrent(int max) {
    currentMillis = max;
    driver->rms_current(max);
    reportTMC5160(driver, this, 0);
}

// Called before homing starts. Can be used e.g. to disable silent mode
// or otherwise prepare for endstop detection.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::beforeHoming() {
    if (hasStallguard()) {
        driver->TCOOLTHRS(0xFFFFF);
        driver->en_pwm_mode(false);
        driver->diag1_stall(true);
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::afterHoming() {
    if (hasStallguard()) {
        driver->TCOOLTHRS(0);
        driver->en_pwm_mode(stealthChop);
        driver->diag1_stall(false);
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper5160Driver<stepCls, dirCls, enableCls, fclk>::handleMCode(GCode& com) {
    switch (com.M) {
    case 122: // print debug informations
    {
        Com::println();
        printMotorNumberAndName();
        reportTMC5160(driver, this, com.hasD() ? static_cast<int>(com.D) : 0);
        if (com.hasS()) {
            if (com.S) {
                debug = RMath::max(static_cast<int16_t>(0), RMath::min(static_cast<int16_t>(2), com.hasD() ? static_cast<int16_t>(com.D) : static_cast<int16_t>(0)));
            } else {
                debug = -1;
            }
        }
    } break;
    case 569: // Set steathchop with S
        if (com.hasS()) {
            stealthChop = com.S != 0;
            driver->en_pwm_mode(stealthChop);
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" stealthChop:"), stealthChop, BoolFormat::ONOFF);
        break;
    case 911: // Report TMC prewarn
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" temperature prewarn triggered: "), otpw, BoolFormat::TRUEFALSE);
        break;
    case 912: // Clear prewarn
        otpw = false;
        otpwCount = 0;
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" prewarn flag cleared"));
        break;
    case 913: // Hybrid treshold
        if (com.hasX()) {
            hybridSpeed = com.X;
            init();
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" hybrid treshold: "), hybridSpeed, 1);
        break;
    case 914: // sensorless homing sensitivity
        if (hasStallguard()) {
            if (com.hasS() && com.S >= -64 && com.S <= 63) {
                stallguardSensitivity = static_cast<int8_t>(com.S);
            }
            printMotorNumberAndName(false);
            Com::printFLN(PSTR(" stallguard sensitivity: "), stallguardSensitivity);
        }
        break;
    }
}

// ---- TMC2208 -------------

void reportTMC2208(TMC2208Stepper* driver, ProgrammableStepperBase* b, int level) {
    uint8_t constat = driver->test_connection();
    Com::printF(PSTR("Status: "));
    switch (constat) {
    case 0:
        Com::printFLN(PSTR("Ok"));
        break;
    case 1:
        Com::printFLN(PSTR("No connection"));
        break;
    case 2:
        Com::printFLN(PSTR("No power"));
        break;
    }
    Com::printFLN(PSTR("Enabled: "), driver->isEnabled(), BoolFormat::YESNO);
    Com::printF(PSTR("Current [mA]: "), driver->rms_current());
    Com::printFLN(PSTR(" set: "), b->getCurrentMillis());
    Com::printFLN(PSTR("Max. current [mA]: "), 1.4142 * driver->rms_current(), 0);
    Com::printF(PSTR("Microsteps: "), driver->microsteps());
    Com::printFLN(PSTR(" mres:"), (int)driver->mres());
    Com::printFLN(PSTR("StealthChop: "), b->getStealthChop(), BoolFormat::ONOFF);
    if (b->getHybridSpeed() > 0) {
        Com::printFLN(PSTR("Hybrid treshold [mm/s]: "), b->getHybridSpeed(), 2);
    } else {
        Com::printFLN(PSTR("Hybrid mode disabled"));
    }
    if (level > 0) {
        const uint32_t tstep = driver->TSTEP();
        Com::printF(PSTR("TSTEP: "));
        if (tstep != 0xFFFFF) {
            Com::print(static_cast<int32_t>(tstep));
            Com::println();
        } else {
            Com::printFLN("max");
        }
        Com::printFLN(PSTR("TPWMTHRS: "), driver->TPWMTHRS());
        Com::printFLN(PSTR("TPOWERDOWN: "), (int)driver->TPOWERDOWN());
        Com::printF(PSTR("IRUN: "), driver->irun());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("IHOLD: "), driver->ihold());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("CS Actual: "), driver->cs_actual());
        Com::printFLN(PSTR("/31"));
        // Com::printFLN(PSTR("vsense: "), driver->vsense());
        Com::printFLN(PSTR("toff: "), (int)driver->toff());
        Com::printFLN(PSTR("hstart: "), (int)driver->hysteresis_start());
        Com::printFLN(PSTR("hend: "), (int)driver->hysteresis_end());
        Com::printF(PSTR("Blank time: "), (int)driver->blank_time());
        Com::printFLN(PSTR(" tbl: "), (int)driver->tbl());
        Com::printFLN(PSTR("IOIN: "), driver->IOIN());
        Com::printFLN(PSTR("GSTAT: "), (int)driver->GSTAT());
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::init() {
    disable();
    driver->begin();

    TMC2208_n::GCONF_t gconf { 0 };
    gconf.pdn_disable = true;      // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealthChop;
    driver->GCONF(gconf.sr);

    TMC2208_n::CHOPCONF_t chopconf { 0 };
    chopconf.tbl = 1;
    chopconf.toff = tmcChopperTiming.toff;
    chopconf.intpol = TMC_INTERPOLATE;
    chopconf.hend = tmcChopperTiming.hend + 3;   // -3 .. 12
    chopconf.hstrt = tmcChopperTiming.hstrt - 1; // 1 .. 16 but hend+hstart <= 16
                                                 // #if ENABLED(TMC_SQUARE_WAVE_STEPPING)
                                                 //    chopconf.dedge = true;
                                                 // #endif
    driver->CHOPCONF(chopconf.sr);

    driver->rms_current(currentMillis, TMC_HOLD_MULTIPLIER);
    driver->microsteps(microsteps);
    driver->iholddelay(10);
    driver->TPOWERDOWN(128); // ~2s until driver lowers to hold current

    TMC2208_n::PWMCONF_t pwmconf { 0 };
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    driver->PWMCONF(pwmconf.sr);

    if (hybridSpeed >= 0) {
        driver->TPWMTHRS(fclk * microsteps / static_cast<uint32_t>(256 * hybridSpeed * Motion1::resolution[getAxis()])); // need computed
    } else {
        driver->TPWMTHRS(0); // Only stealthChop or spreadCycle
    }

    driver->GSTAT(0b111); // Clear GSTAT
    HAL::delayMilliseconds(200);
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::eepromHandle() {
    PGM_P* adr = (PGM_P*)pgm_read_word(&motorNames);
    EEPROM::handlePrefix(adr[motorIndex()]);
    processEEPROM(15);
    EEPROM::removePrefix();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::eepromReserve() {
    reserveEEPROM(0);
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::reset(uint16_t _microsteps, uint16_t _currentMillis, bool _stealthChop, float _hybridThrs) {
    microsteps = _microsteps;
    currentMillis = _currentMillis;
    stealthChop = _stealthChop;
    hybridSpeed = _hybridThrs;
    stallguardSensitivity = 8;
    init();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::timer500ms() {
    if (debug != -1) {
        reportTMC2208(driver, this, debug);
    }
    // Access of bits results in reading register again, so we buffer result
    TMC2208_n::DRV_STATUS_t status { driver->DRV_STATUS() };
    if (status.sr == 0xFFFFFFFF || status.sr == 0x0) { // not in working state
        return;
    }
    if (status.ot) { // over temperature
        printMotorNumberAndName(false);
        Com::printFLN(" driver overtemperature! Current: ", currentMillis);
    }
    if (status.otpw) { // over temperature prewarn
        if (otpwCount < 255) {
            otpwCount++;
        }
        if (otpwCount == 1) {
            printMotorNumberAndName(false);
            Com::printFLN(" driver overtemperature warning! Current: ", currentMillis);
        }
#if TMC_CURRENT_STEP_DOWN > 0
        if (otpwCount > 4 && driver->isEnabled()) {
            if (currentMillis > 100) {
                currentMillis -= TMC_CURRENT_STEP_DOWN;
                driver->rms_current(currentMillis);
                printMotorNumberAndName(false);
                Com::printFLN(" current decreased to ", currentMillis);
            }
        }
#endif
        otpw = true;
    } else {
        otpwCount = 0;
    }
}

/// Set microsteps. Must be a power of 2.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::setMicrosteps(int _microsteps) {
    microsteps = _microsteps;
    driver->microsteps(microsteps);
    reportTMC2208(driver, this, 0);
}

/// Set max current as range 0..255 or mA depedning on driver
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::setMaxCurrent(int max) {
    currentMillis = max;
    driver->rms_current(max);
    reportTMC2208(driver, this, 0);
}

// Called before homing starts. Can be used e.g. to disable silent mode
// or otherwise prepare for endstop detection.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::beforeHoming() {
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::afterHoming() {
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2208Driver<stepCls, dirCls, enableCls, fclk>::handleMCode(GCode& com) {
    switch (com.M) {
    case 122: // print debug informations
    {
        Com::println();
        printMotorNumberAndName();
        reportTMC2208(driver, this, com.hasD() ? static_cast<int>(com.D) : 0);
        if (com.hasS()) {
            if (com.S) {
                debug = RMath::max(static_cast<int16_t>(0), RMath::min(static_cast<int16_t>(2), com.hasD() ? static_cast<int16_t>(com.D) : static_cast<int16_t>(0)));
            } else {
                debug = -1;
            }
        }
    } break;
    case 569: // Set steathchop with S
        if (com.hasS()) {
            stealthChop = com.S != 0;
            TMC2208_n::GCONF_t gconf { 0 };
            gconf.pdn_disable = true;      // Use UART
            gconf.mstep_reg_select = true; // Select microsteps with UART
            gconf.i_scale_analog = false;
            gconf.en_spreadcycle = !stealthChop;
            driver->GCONF(gconf.sr);
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" stealthChop:"), stealthChop, BoolFormat::ONOFF);
        break;
    case 911: // Report TMC prewarn
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" temperature prewarn triggered: "), otpw, BoolFormat::TRUEFALSE);
        break;
    case 912: // Clear prewarn
        otpw = false;
        otpwCount = 0;
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" prewarn flag cleared"));
        break;
    case 913: // Hybrid treshold
        if (com.hasX()) {
            hybridSpeed = com.X;
            init();
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" hybrid treshold: "), hybridSpeed, 1);
        break;
    case 914: // sensorless homing sensitivity
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" stallguard sensitivity not supported by TMC2208 "));
        break;
    }
}

// ---- TMC2209 -------------
void reportTMC2209(TMC2209Stepper* driver, ProgrammableStepperBase* b, int level) {
    uint8_t constat = driver->test_connection();
    Com::printF(PSTR("Status: "));
    switch (constat) {
    case 0:
        Com::printFLN(PSTR("Ok"));
        break;
    case 1:
        Com::printFLN(PSTR("No connection"));
        break;
    case 2:
        Com::printFLN(PSTR("No power"));
        break;
    }
    Com::printFLN(PSTR("Enabled: "), driver->isEnabled(), BoolFormat::YESNO);
    Com::printF(PSTR("Current [mA]: "), driver->rms_current());
    Com::printFLN(PSTR(" set: "), b->getCurrentMillis());
    Com::printFLN(PSTR("Max. current [mA]: "), 1.4142 * driver->rms_current(), 0);
    Com::printF(PSTR("Microsteps: "), driver->microsteps());
    Com::printFLN(PSTR(" mres:"), (int)driver->mres());
    Com::printFLN(PSTR("StealthChop: "), b->getStealthChop(), BoolFormat::ONOFF);
    if (b->getHybridSpeed() > 0) {
        Com::printFLN(PSTR("Hybrid treshold [mm/s]: "), b->getHybridSpeed(), 2);
    } else {
        Com::printFLN(PSTR("Hybrid mode disabled"));
    }
    Com::printFLN(PSTR("StallGuard Result: "), driver->SG_RESULT());

    if (level > 0) {
        const uint32_t tstep = driver->TSTEP();
        Com::printF(PSTR("TSTEP: "));
        if (tstep != 0xFFFFF) {
            Com::print(static_cast<int32_t>(tstep));
            Com::println();
        } else {
            Com::printFLN("max");
        }
        Com::printFLN(PSTR("TPWMTHRS: "), driver->TPWMTHRS());
        Com::printFLN(PSTR("TPOWERDOWN: "), (int)driver->TPOWERDOWN());
        Com::printF(PSTR("IRUN: "), driver->irun());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("IHOLD: "), driver->ihold());
        Com::printFLN(PSTR("/31"));
        Com::printF(PSTR("CS Actual: "), driver->cs_actual());
        Com::printFLN(PSTR("/31"));
        // Com::printFLN(PSTR("vsense: "), driver->vsense());
        Com::printFLN(PSTR("toff: "), (int)driver->toff());
        Com::printFLN(PSTR("hstart: "), (int)driver->hysteresis_start());
        Com::printFLN(PSTR("hend: "), (int)driver->hysteresis_end());
        Com::printF(PSTR("Blank time: "), (int)driver->blank_time());
        Com::printFLN(PSTR(" tbl: "), (int)driver->tbl());
        Com::printFLN(PSTR("IOIN: "), driver->IOIN());
        Com::printFLN(PSTR("GSTAT: "), (int)driver->GSTAT());

        Com::printFLN(PSTR("COOLCONF: "), (int)driver->COOLCONF());
        Com::printFLN(PSTR("semin: "), (int)driver->semin());
        Com::printFLN(PSTR("seup: "), (int)driver->seup());
        Com::printFLN(PSTR("semax: "), (int)driver->semax());
        Com::printFLN(PSTR("sedn: "), (int)driver->sedn());
        Com::printFLN(PSTR("seimin: "), (int)driver->seimin()); 
        
    }
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::init() {
    disable();
    driver->begin();

    // The TMC2209_n namespace doesn't recreate all the register structs for the 2209 specifically
    // We're meant to use the 2208's. 
    // However, it does have it's own IOIN_t, COOLCONF_t, SG_RESULT_t, and SGTHRS_t tables!
    
    TMC2208_n::GCONF_t gconf { 0 };
    gconf.pdn_disable = true;      // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealthChop;
    driver->GCONF(gconf.sr);

    TMC2208_n::CHOPCONF_t chopconf { 0 };
    chopconf.tbl = 1;
    chopconf.toff = tmcChopperTiming.toff;
    chopconf.intpol = TMC_INTERPOLATE;
    chopconf.hend = tmcChopperTiming.hend + 3;   // -3 .. 12
    chopconf.hstrt = tmcChopperTiming.hstrt - 1; // 1 .. 16 but hend+hstart <= 16
                                                 // #if ENABLED(TMC_SQUARE_WAVE_STEPPING)
                                                 //    chopconf.dedge = true;
                                                 // #endif
    driver->CHOPCONF(chopconf.sr);

    driver->rms_current(currentMillis, TMC_HOLD_MULTIPLIER);
    driver->microsteps(microsteps);
    driver->iholddelay(10);
    driver->TPOWERDOWN(128); // ~2s until driver lowers to hold current

    
    
    TMC2208_n::PWMCONF_t pwmconf { 0 };
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    driver->PWMCONF(pwmconf.sr); 

    
    //TMC2209_n::COOLCONF_t coolconf { 0 };  

    if (hybridSpeed >= 0) {
        driver->TPWMTHRS(fclk * microsteps / static_cast<uint32_t>(256 * hybridSpeed * Motion1::resolution[getAxis()])); // need computed
    } else {
        driver->TPWMTHRS(0); // Only stealthChop or spreadCycle
    }

    if (stallguardSensitivity != -128) {
        stallguardSensitivity = constrain(stallguardSensitivity, 0, 255);
        driver->SGTHRS(stallguardSensitivity);
    }
    driver->GSTAT(); // Clear GSTAT
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::eepromHandle() {
    PGM_P* adr = (PGM_P*)pgm_read_word(&motorNames);
    EEPROM::handlePrefix(adr[motorIndex()]);
    processEEPROM(63); //StallGuard 4, flag 32
    EEPROM::removePrefix();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::eepromReserve() {
    reserveEEPROM(0);
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::reset(uint16_t _microsteps, uint16_t _currentMillis, bool _stealthChop, float _hybridThrs, int16_t _stallSensitivity) {
    microsteps = _microsteps;
    currentMillis = _currentMillis;
    stealthChop = _stealthChop;
    hybridSpeed = _hybridThrs;
    stallguardSensitivity = _stallSensitivity;
    init();
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::timer500ms() {
    if (debug != -1) {
        reportTMC2209(driver, this, debug);
    }
    // Access of bits results in reading register again, so we buffer result
    TMC2208_n::DRV_STATUS_t status { driver->DRV_STATUS() };
    if (status.sr == 0xFFFFFFFF || status.sr == 0x0) { // not in working state
        return;
    }
    if (status.ot) { // over temperature
        printMotorNumberAndName(false);
        Com::printFLN(" driver overtemperature! Current: ", currentMillis);
    }
    if (status.otpw) { // over temperature prewarn
        if (otpwCount < 255) {
            otpwCount++;
        }
        if (otpwCount == 1) {
            printMotorNumberAndName(false);
            Com::printFLN(" driver overtemperature warning! Current: ", currentMillis);
        }
#if TMC_CURRENT_STEP_DOWN > 0
        if (otpwCount > 4 && driver->isEnabled()) {
            if (currentMillis > 100) {
                currentMillis -= TMC_CURRENT_STEP_DOWN;
                driver->rms_current(currentMillis);
                printMotorNumberAndName(false);
                Com::printFLN(" current decreased to ", currentMillis);
            }
        }
#endif
        otpw = true;
    } else {
        otpwCount = 0;
    }
}

/// Set microsteps. Must be a power of 2.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::setMicrosteps(int _microsteps) {
    microsteps = _microsteps;
    driver->microsteps(microsteps);
    reportTMC2209(driver, this, 0);
}

/// Set max current as range 0..255 or mA depedning on driver
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::setMaxCurrent(int max) {
    currentMillis = max;
    driver->rms_current(max);
    reportTMC2209(driver, this, 0);
}

// Called before homing starts. Can be used e.g. to disable silent mode
// or otherwise prepare for endstop detection.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::beforeHoming() {
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::afterHoming() {
}

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
void TMCStepper2209Driver<stepCls, dirCls, enableCls, fclk>::handleMCode(GCode& com) {
    switch (com.M) {
    case 122: // print debug informations
    {
        Com::println();
        printMotorNumberAndName();
        reportTMC2209(driver, this, com.hasD() ? static_cast<int>(com.D) : 0);
        if (com.hasS()) {
            if (com.S) {
                debug = RMath::max(static_cast<int16_t>(0), RMath::min(static_cast<int16_t>(2), com.hasD() ? static_cast<int16_t>(com.D) : static_cast<int16_t>(0)));
            } else {
                debug = -1;
            }
        }
    } break;
    case 569: // Set steathchop with S
        if (com.hasS()) {
            stealthChop = com.S != 0;
            TMC2208_n::GCONF_t gconf { 0 };
            gconf.pdn_disable = true;      // Use UART
            gconf.mstep_reg_select = true; // Select microsteps with UART
            gconf.i_scale_analog = false;
            gconf.en_spreadcycle = !stealthChop;
            driver->GCONF(gconf.sr);
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" stealthChop:"), stealthChop, BoolFormat::ONOFF);
        break;
    case 911: // Report TMC prewarn
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" temperature prewarn triggered: "), otpw, BoolFormat::TRUEFALSE);
        break;
    case 912: // Clear prewarn
        otpw = false;
        otpwCount = 0;
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" prewarn flag cleared"));
        break;
    case 913: // Hybrid treshold
        if (com.hasX()) {
            hybridSpeed = com.X;
            init();
        }
        printMotorNumberAndName(false);
        Com::printFLN(PSTR(" hybrid treshold: "), hybridSpeed, 1);
        break;
        
    case 914: // sensorless homing sensitivity
        if (hasStallguard()) {
            if (com.hasS() && com.S >= 0 && com.S <= 255) {
                stallguardSensitivity = static_cast<int16_t>(com.S);
            }
            printMotorNumberAndName(false);
            Com::printFLN(PSTR(" stallguard sensitivity: "), stallguardSensitivity);
        }
        break;
    }
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TOOLS_TEMPLATES
#include "../io/redefine.h"
