#include <TMCStepper.h>

#ifndef TMC_SW_SERIAL_BAUD
#define TMC_SW_SERIAL_BAUD 19200
#endif

class EndstopDriver;
class GCode;
enum class GUIAction;

class StepperDriverBase {
protected:
    StepperDriverBase* parent;
    EndstopDriver* minEndstop;
    EndstopDriver* maxEndstop;
    bool direction;
    ufast8_t axisBit;
    fast8_t axis;

public:
    StepperDriverBase(EndstopDriver* minES, EndstopDriver* maxES)
        : parent(nullptr)
        , minEndstop(minES)
        , maxEndstop(maxES)
        , direction(true)
        , axisBit(8)
        , axis(E_AXIS) { }
    virtual ~StepperDriverBase() { }
    inline void setParent(StepperDriverBase* ptr) { parent = ptr; }
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
    inline bool updateEndstop() {
        if (direction) {
            return maxEndstop->update();
        } else {
            return minEndstop->update();
        }
    }
    virtual void setAxis(fast8_t ax) {
        axisBit = 1 << ax;
        axis = ax;
    }
    inline ufast8_t getAxisBit() { return axisBit; }
    inline fast8_t getAxis() { return axis; }
    void printMotorNumberAndName(bool newline = true);
    /// Allows initialization of driver e.g. current, microsteps
    virtual void init() { }
    /// Always executes the step
    virtual void step() = 0;
    /// Execute the step if motor end stop is not triggered
    virtual bool stepMotorEndStop() = 0;
    /// Set step signal low
    virtual void unstep() = 0;
    /// Set direction, true = max direction
    virtual void dir(bool d) = 0;
    /// Enable motor driver
    virtual void enable() = 0;
    /// Disable motor driver
    virtual void disable() = 0;
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() { return false; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() { return false; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) { }
    /// Set max current as range 0..255 or mA depending on driver
    virtual void setMaxCurrent(int max) { }
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() { }
    virtual void afterHoming() { }
    virtual void handleMCode(GCode& com) { }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return false; }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) { }
    virtual bool hasConfigMenu() { return false; }
    // Allow having own settings e.g. current, microsteps
    virtual void eepromHandle() { }
    int motorIndex();
    void printMotorName();
};

struct MixingStepperState {
    int mixingW;  ///< Weight for this extruder when mixing steps
    int mixingWB; ///< Weight after balancing extruder steps per mm
    int mixingE;  ///< Cumulated error for this step.
    float stepsPerMM;
    StepperDriverBase* stepper;
};

class MixingStepperDriver : public StepperDriverBase {
    ufast8_t nStepper;
    MixingStepperState* state;
    float stepsPerMM; ///< Biggest resolution of all substepper to always get enough steps
    void rebuildWeights();

public:
    MixingStepperDriver()
        : MixingStepperDriver(0, nullptr, nullptr, nullptr) { }
    MixingStepperDriver(fast8_t n, MixingStepperState* state, EndstopDriver* minES, EndstopDriver* maxES);
    inline ufast8_t numMotors() { return nStepper; } ///< Number of motors that get mixed
    void setWeight(ufast8_t motorId, int weight);
    /// Always executes the step
    virtual void step() override final;
    /// Execute the step if motor end stop is not triggered
    virtual bool stepMotorEndStop() override final;
    /// Set step signal low
    virtual void unstep() override final;
    /// Set direction, true = max direction
    virtual void dir(bool d) override final;
    /// Enable motor driver
    virtual void enable() override final;
    /// Disable motor driver
    virtual void disable() override final;
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final;
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final;
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final;
    /// Set max current as range 0..255 or mA depending on driver
    virtual void setMaxCurrent(int max) override final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final;
    virtual void afterHoming() override final;
    virtual void handleMCode(GCode& com) override final;
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return true; }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final;
    // Allow having own settings e.g. current, microsteps
    virtual void eepromHandle() override final;
};

/** Holds a position counter that can be observed, otherwise sends
 * all commands through to the original stepper driver.
 * You need this to detect a jam in extruder or maybe for other
 * future task requiring a position counter.
 */
template <class driver>
class ObservableStepperDriver : public StepperDriverBase {
    driver* stepper;

public:
    uint32_t position;
    ObservableStepperDriver(driver* _stepper)
        : StepperDriverBase(_stepper->getMinEndstop(), _stepper->getMaxEndstop())
        , stepper(_stepper) {
        stepper->setParent(this);
        position = 0;
    }
    virtual ~ObservableStepperDriver() { }
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
    /// Allows initialization of driver e.g. current, microsteps
    virtual void init() override final { }
    /// Always executes the step
    virtual void step() override final {
        if (direction) {
            position++;
        } else {
            position--;
        }
        stepper->step();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        if (stepper->stepMotorEndStop()) {
            if (direction) {
                position++;
            } else {
                position--;
            }
            return true;
        }
        return false;
    }
    /// Set step signal low
    virtual void unstep() override final {
        stepper->unstep();
    }
    /// Set direction, true = max direction
    virtual void dir(bool d) override final {
        direction = d;
        stepper->dir(d);
    }
    /// Enable motor driver
    virtual void enable() override final { stepper->enable(); }
    /// Disable motor driver
    virtual void disable() override final { stepper->disable(); }
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return stepper->implementsSetMicrosteps(); }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return stepper->implementsSetMaxCurrent(); }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final { stepper->setMicrosteps(microsteps); }
    /// Set max current as range 0..255
    virtual void setMaxCurrent(int max) override final { stepper->setMaxCurrent(max); }
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final { stepper->beforeHoming(); }
    virtual void afterHoming() override final { stepper->afterHoming(); }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return stepper->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final {
        stepper->menuConfig(action, data);
    }
    virtual bool hasConfigMenu() override final {
        return stepper->hasConfigMenu();
    }
    virtual void eepromHandle() override final {
        stepper->eepromHandle();
    }
};

/**
  Converts incoming step frequency from = steps/length unit to to = steps/length unit steps.
  The only requirement is that the incoming frequency must be higher then the outgoing
  frequency. The maximum error is one step.
 */
template <class driver>
class AdjustResolutionStepperDriver : public StepperDriverBase {
    driver* stepper;
    int32_t accumulator;
    int32_t from;
    int32_t to;
    uint16_t eprStart;

public:
    AdjustResolutionStepperDriver(driver* _stepper, int32_t _from, int32_t _to)
        : StepperDriverBase(_stepper->getMinEndstop(), _stepper->getMaxEndstop())
        , stepper(_stepper)
        , accumulator(0)
        , from(_from)
        , to(_to) { }
    virtual ~AdjustResolutionStepperDriver() { }
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
    virtual void setAxis(fast8_t ax) override final {
        StepperDriverBase::setAxis(ax);
        stepper->setAxis(ax);
    }
    /// Always executes the step
    virtual void step() override final {
        if (direction) {
            accumulator += to;
            if (accumulator >= to) {
                stepper->step();
                accumulator -= from;
            }
        } else {
            accumulator -= to;
            if (accumulator <= -to) {
                stepper->step();
                accumulator += from;
            }
        }
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        if (direction) {
            accumulator += to;
            if (accumulator >= to) {
                if (stepper->stepMotorEndStop()) {
                    accumulator -= from;
                    return true;
                }
            }
        } else {
            accumulator -= to;
            if (accumulator <= -to) {
                if (stepper->stepMotorEndStop()) {
                    accumulator += from;
                    return true;
                }
            }
        }
        return false;
    }
    /// Set step signal low
    virtual void unstep() override final {
        stepper->unstep();
    }
    /// Set direction, true = max direction
    virtual void dir(bool d) override final {
        direction = d;
        stepper->dir(d);
    }
    /// Enable motor driver
    virtual void enable() override final { stepper->enable(); }
    /// Disable motor driver
    virtual void disable() override final { stepper->disable(); }
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return stepper->implementsSetMicrosteps(); }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return stepper->implementsSetMaxCurrent(); }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final { stepper->setMicrosteps(microsteps); }
    /// Set max current as range 0..255
    virtual void setMaxCurrent(int max) override final { stepper->setMaxCurrent(max); }
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final { stepper->beforeHoming(); }
    virtual void afterHoming() override final { stepper->afterHoming(); }
    // If true the stepper usager will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return true; }
    static void menuStepsPerMM(GUIAction action, void* data);
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final;
    virtual bool hasConfigMenu() override final { return true; }
    virtual void init() override final;
    virtual void eepromHandle() override final;
    void restoreFromConfiguration(int32_t _to) {
        to = _to;
    }
};

/// Plain stepper driver with optional endstops attached.
template <class stepCls, class dirCls, class enableCls>
class SimpleStepperDriver : public StepperDriverBase {
public:
    SimpleStepperDriver(EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES) { }
    virtual ~SimpleStepperDriver() { }
    virtual void init() { disable(); }
    inline void step() override final {
        stepCls::on();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        stepCls::on();
        return true;
    }
    inline void unstep() override final {
        stepCls::off();
    }
    inline void dir(bool d) override final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() override final {
        enableCls::on();
    }
    inline void disable() override final {
        enableCls::off();
    }
};

class ProgrammableStepperBase {
protected:
    int16_t microsteps;
    int16_t currentMillis;
    float hybridSpeed;
    bool stealthChop;
    int8_t debug;
    bool otpw;
    uint8_t otpwCount;
    int16_t stallguardSensitivity;
    uint16_t eprStart;

public:
    ProgrammableStepperBase(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int16_t _stallguardSensitivity)
        : microsteps(_microsteps)
        , currentMillis(_current_millis)
        , hybridSpeed(_hybridThrs)
        , stealthChop(_stealthChop)
        , debug(-1)
        , otpw(false)
        , otpwCount(0u)
        , stallguardSensitivity(_stallguardSensitivity)
        , eprStart(0u) { }
    inline int16_t getMicrosteps() { return microsteps; }
    inline int16_t getCurrentMillis() { return currentMillis; }
    inline bool getStealthChop() { return stealthChop; }
    inline float getHybridSpeed() { return hybridSpeed; }
    inline bool hasStallguard() { return stallguardSensitivity != -128; }
    void reserveEEPROM(uint16_t extraBytes);
    void processEEPROM(uint8_t flags);
    template <class derived>
    static void menuSetCurrent(GUIAction action, derived* driver);
};

/// Plain stepper driver with optional endstops attached.
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
class TMCStepper2130Driver : public StepperDriverBase, public ProgrammableStepperBase {
    TMC2130Stepper* driver;

protected:
    bool isEnabled;

public:
    TMCStepper2130Driver(EndstopDriver* minES, EndstopDriver* maxES,
                         TMC2130Stepper* _driver, uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity)
        : StepperDriverBase(minES, maxES)
        , ProgrammableStepperBase(_microsteps, _current_millis, _stealthChop, _hybridThrs, _stallSensitivity)
        , driver(_driver)
        , isEnabled(false) { }
    virtual void init() override final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity);
    inline void step() override final {
        stepCls::on();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        stepCls::on();
        return true;
    }
    inline void unstep() override final {
        stepCls::off();
    }
    inline void dir(bool d) override final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() override final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() override final {
        enableCls::off();
        isEnabled = false;
    }
    void eepromHandle();
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) override final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final;
    virtual void afterHoming() override final;
    virtual void handleMCode(GCode& com) override final;
    virtual void menuConfig(GUIAction action, void* data) override final;
    virtual bool hasConfigMenu() override final { return true; }
    void timer500ms();
};

template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
class TMCStepper5160Driver : public StepperDriverBase, public ProgrammableStepperBase {
    TMC5160Stepper* driver;

protected:
    bool isEnabled;

public:
    TMCStepper5160Driver(EndstopDriver* minES, EndstopDriver* maxES,
                         TMC5160Stepper* _driver, uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity)
        : StepperDriverBase(minES, maxES)
        , ProgrammableStepperBase(_microsteps, _current_millis, _stealthChop, _hybridThrs, _stallSensitivity)
        , driver(_driver)
        , isEnabled(false) { }
    virtual void init() final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity);
    inline void step() override final {
        stepCls::on();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        stepCls::on();
        return true;
    }
    inline void unstep() override final {
        stepCls::off();
    }
    inline void dir(bool d) override final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() override final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() override final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() override final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) override final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final;
    virtual void afterHoming() override final;
    virtual void handleMCode(GCode& com) override final;
    virtual void menuConfig(GUIAction action, void* data) override final;
    virtual bool hasConfigMenu() override final { return true; }
    void timer500ms();
};

// TMC2208 does not have stallguard!
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
class TMCStepper2208Driver : public StepperDriverBase, public ProgrammableStepperBase {
    TMC2208Stepper* driver;

protected:
    bool isEnabled;

public:
    TMCStepper2208Driver(EndstopDriver* minES, EndstopDriver* maxES,
                         TMC2208Stepper* _driver, uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs)
        : StepperDriverBase(minES, maxES)
        , ProgrammableStepperBase(_microsteps, _current_millis, _stealthChop, _hybridThrs, -128)
        , driver(_driver)
        , isEnabled(false) { }
    virtual void init() override final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs);
    inline void step() override final {
        stepCls::on();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        stepCls::on();
        return true;
    }
    inline void unstep() override final {
        stepCls::off();
    }
    inline void dir(bool d) override final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() override final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() override final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() override final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) override final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final;
    virtual void afterHoming() override final;
    virtual void handleMCode(GCode& com) override final;
    virtual void menuConfig(GUIAction action, void* data) override final;
    virtual bool hasConfigMenu() override final { return true; }
    void timer500ms();
};

// New version of the 2208. Has stallguard 4 and addressable UART
template <class stepCls, class dirCls, class enableCls, uint32_t fclk>
class TMCStepper2209Driver : public StepperDriverBase, public ProgrammableStepperBase {
    TMC2209Stepper* driver;
    bool usesSoftwareSerial;

protected:
    bool isEnabled;

public:
    TMCStepper2209Driver(EndstopDriver* minES, EndstopDriver* maxES,
                         TMC2209Stepper* _driver, uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int16_t _stallSensitivity, bool _isSoftware)
        : StepperDriverBase(minES, maxES)
        , ProgrammableStepperBase(_microsteps, _current_millis, _stealthChop, _hybridThrs, _stallSensitivity)
        , driver(_driver)
        , usesSoftwareSerial(_isSoftware)
        , isEnabled(false) {
#if SW_CAPABLE_PLATFORM
        if (usesSoftwareSerial && driver != nullptr) {
            // Need to do this before ANY sort of serial commands or we'll get a infinite loop in the serial library
            driver->beginSerial(TMC_SW_SERIAL_BAUD); // starts just the serial.
        }
#endif
    }
    virtual void init() override final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int16_t _stallSensitivity);
    inline void step() override final {
        stepCls::on();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
        stepCls::on();
        return true;
    }
    inline void unstep() override final {
        stepCls::off();
    }
    inline void dir(bool d) override final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() override final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() override final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() override final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() override final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() override final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) override final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) override final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() override final;
    virtual void afterHoming() override final;
    virtual void handleMCode(GCode& com) override final;
    virtual void menuConfig(GUIAction action, void* data) override final;
    virtual bool hasConfigMenu() override final { return true; }
    void timer500ms();
};
/// Plain stepper driver with optional endstops attached.
class Mirror2StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2;
    Mirror2StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2) {
        m1->setParent(this);
        m2->setParent(this);
    }
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
    }
    inline void step() override final {
        motor1->step();
        motor2->step();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
#if defined(NO_MOTOR_ENDSTOPS)
        motor1->step();
        motor2->step();
        return true;
#else
        if (direction) {
            if (!motor1->getMaxEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMaxEndstop()->triggered()) {
                motor2->step();
            }
        } else {
            if (!motor1->getMinEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMinEndstop()->triggered()) {
                motor2->step();
            }
        }
        return true;
#endif
    }
    inline void unstep() override final {
        motor1->unstep();
        motor2->unstep();
    }
    inline void dir(bool d) override final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        direction = d;
    }
    inline void enable() override final {
        motor1->enable();
        motor2->enable();
    }
    inline void disable() override final {
        motor1->disable();
        motor2->disable();
    }
    inline void handleMCode(GCode& com) override final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return motor1->overridesResolution() || motor2->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
    }
    virtual bool hasConfigMenu() override final {
        return (motor1->hasConfigMenu() || motor2->hasConfigMenu());
    }
    virtual void eepromHandle() override final {
        motor1->eepromHandle();
        motor2->eepromHandle();
    }
};

/// Plain stepper driver with optional endstops attached.
class Mirror3StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2, *motor3;
    Mirror3StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, StepperDriverBase* m3, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2)
        , motor3(m3) {
        m1->setParent(this);
        m2->setParent(this);
        m3->setParent(this);
    }
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
        motor3->setAxis(ax);
    }
    inline void step() override final {
        motor1->step();
        motor2->step();
        motor3->step();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
#if defined(NO_MOTOR_ENDSTOPS)
        motor1->step();
        motor2->step();
        motor3->step();
        return true;
#else
        if (direction) {
            if (!motor1->getMaxEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMaxEndstop()->triggered()) {
                motor2->step();
            }
            if (!motor3->getMaxEndstop()->triggered()) {
                motor3->step();
            }
        } else {
            if (!motor1->getMinEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMinEndstop()->triggered()) {
                motor2->step();
            }
            if (!motor3->getMinEndstop()->triggered()) {
                motor3->step();
            }
        }
        return true;
#endif
    }
    inline void unstep() override final {
        motor1->unstep();
        motor2->unstep();
        motor3->unstep();
    }
    inline void dir(bool d) override final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        motor3->dir(d);
        direction = d;
    }
    inline void enable() override final {
        motor1->enable();
        motor2->enable();
        motor3->enable();
    }
    inline void disable() override final {
        motor1->disable();
        motor2->disable();
        motor3->disable();
    }
    inline void handleMCode(GCode& com) override final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
        motor3->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return motor1->overridesResolution() || motor2->overridesResolution() || motor3->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
        motor3->menuConfig(action, data);
    }
    virtual bool hasConfigMenu() override final {
        return (motor1->hasConfigMenu() || motor2->hasConfigMenu() || motor3->hasConfigMenu());
    }
    virtual void eepromHandle() override final {
        motor1->eepromHandle();
        motor2->eepromHandle();
        motor3->eepromHandle();
    }
};

/// Plain stepper driver with optional endstops attached.
class Mirror4StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2, *motor3, *motor4;
    Mirror4StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, StepperDriverBase* m3, StepperDriverBase* m4, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2)
        , motor3(m3)
        , motor4(m4) {
        m1->setParent(this);
        m2->setParent(this);
        m3->setParent(this);
        m4->setParent(this);
    }
    virtual void setAxis(fast8_t ax) override final {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
        motor3->setAxis(ax);
        motor4->setAxis(ax);
    }
    inline void step() override final {
        motor1->step();
        motor2->step();
        motor3->step();
        motor4->step();
    }
    /// Execute the step if motor end stop is not triggered
    bool stepMotorEndStop() override final {
#if defined(NO_MOTOR_ENDSTOPS)
        motor1->step();
        motor2->step();
        motor3->step();
        motor4->step();
        return true;
#else
        if (direction) {
            if (!motor1->getMaxEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMaxEndstop()->triggered()) {
                motor2->step();
            }
            if (!motor3->getMaxEndstop()->triggered()) {
                motor3->step();
            }
            if (!motor4->getMaxEndstop()->triggered()) {
                motor4->step();
            }
        } else {
            if (!motor1->getMinEndstop()->triggered()) {
                motor1->step();
            }
            if (!motor2->getMinEndstop()->triggered()) {
                motor2->step();
            }
            if (!motor3->getMinEndstop()->triggered()) {
                motor3->step();
            }
            if (!motor4->getMinEndstop()->triggered()) {
                motor4->step();
            }
        }
        return true;
#endif
    }
    inline void unstep() override final {
        motor1->unstep();
        motor2->unstep();
        motor3->unstep();
        motor4->unstep();
    }
    inline void dir(bool d) override final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        motor3->dir(d);
        motor4->dir(d);
        direction = d;
    }
    inline void enable() override final {
        motor1->enable();
        motor2->enable();
        motor3->enable();
        motor4->enable();
    }
    inline void disable() override final {
        motor1->disable();
        motor2->disable();
        motor3->disable();
        motor4->disable();
    }
    inline void handleMCode(GCode& com) override final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
        motor3->handleMCode(com);
        motor4->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() override final { return motor1->overridesResolution() || motor2->overridesResolution() || motor3->overridesResolution() || motor4->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) override final {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
        motor3->menuConfig(action, data);
        motor4->menuConfig(action, data);
    }
    virtual bool hasConfigMenu() override final {
        return (motor1->hasConfigMenu() || motor2->hasConfigMenu() || motor3->hasConfigMenu() || motor4->hasConfigMenu());
    }
    virtual void eepromHandle() override final {
        motor1->eepromHandle();
        motor2->eepromHandle();
        motor3->eepromHandle();
        motor4->eepromHandle();
    }
};
