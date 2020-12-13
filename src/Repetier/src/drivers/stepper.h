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
    virtual void step();
    /// Set step signal low
    virtual void unstep();
    /// Set direction, true = max direction
    virtual void dir(bool d);
    /// Enable motor driver
    virtual void enable();
    /// Disable motor driver
    virtual void disable();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps();
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent();
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps);
    /// Set max current as range 0..255 or mA depending on driver
    virtual void setMaxCurrent(int max);
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming();
    virtual void afterHoming();
    virtual void handleMCode(GCode& com);
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return true; }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data);
    // Allow having own settings e.g. current, microsteps
    virtual void eepromHandle();
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
    virtual void init() final { }
    /// Always executes the step
    virtual void step() final {
        if (direction) {
            position++;
        } else {
            position--;
        }
        stepper->step();
    }
    /// Set step signal low
    virtual void unstep() final {
        stepper->unstep();
    }
    /// Set direction, true = max direction
    virtual void dir(bool d) final {
        direction = d;
        stepper->dir(d);
    }
    /// Enable motor driver
    virtual void enable() final { stepper->enable(); }
    /// Disable motor driver
    virtual void disable() final { stepper->disable(); }
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return stepper->implementsSetMicrosteps(); }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return stepper->implementsSetMaxCurrent(); }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final { stepper->setMicrosteps(microsteps); }
    /// Set max current as range 0..255
    virtual void setMaxCurrent(int max) final { stepper->setMaxCurrent(max); }
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final { stepper->beforeHoming(); }
    virtual void afterHoming() final { stepper->afterHoming(); }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return stepper->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) {
        stepper->menuConfig(action, data);
    }
    virtual void eepromHandle() final {
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
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        stepper->setAxis(ax);
    }
    /// Always executes the step
    virtual void step() final {
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
    /// Set step signal low
    virtual void unstep() final {
        stepper->unstep();
    }
    /// Set direction, true = max direction
    virtual void dir(bool d) final {
        direction = d;
        stepper->dir(d);
    }
    /// Enable motor driver
    virtual void enable() final { stepper->enable(); }
    /// Disable motor driver
    virtual void disable() final { stepper->disable(); }
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return stepper->implementsSetMicrosteps(); }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return stepper->implementsSetMaxCurrent(); }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final { stepper->setMicrosteps(microsteps); }
    /// Set max current as range 0..255
    virtual void setMaxCurrent(int max) final { stepper->setMaxCurrent(max); }
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final { stepper->beforeHoming(); }
    virtual void afterHoming() final { stepper->afterHoming(); }
    // If true the stepper usager will not offer resolution modification in GUI
    virtual bool overridesResolution() final { return true; }
    static void menuStepsPerMM(GUIAction action, void* data);
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) final;
    virtual void init() final;
    virtual void eepromHandle() final;
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
    virtual void init() { disable(); }
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() final {
        enableCls::on();
    }
    inline void disable() final {
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
        , otpwCount(0)
        , stallguardSensitivity(_stallguardSensitivity) { }
    inline int16_t getMicrosteps() { return microsteps; }
    inline int16_t getCurrentMillis() { return currentMillis; }
    inline bool getStealthChop() { return stealthChop; }
    inline float getHybridSpeed() { return hybridSpeed; }
    inline bool hasStallguard() { return stallguardSensitivity != -128; }
    void reserveEEPROM(uint16_t extraBytes);
    void processEEPROM(uint8_t flags);
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
    virtual void init() final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int8_t _stallSensitivity);
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() final {
        enableCls::off();
        isEnabled = false;
    }
    void eepromHandle();
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final;
    virtual void afterHoming() final;
    virtual void handleMCode(GCode& com) final;
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
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final;
    virtual void afterHoming() final;
    virtual void handleMCode(GCode& com) final;
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
    virtual void init() final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs);
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final;
    virtual void afterHoming() final;
    virtual void handleMCode(GCode& com) final;
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
    virtual void init() final;
    void reset(uint16_t _microsteps, uint16_t _current_millis, bool _stealthChop, float _hybridThrs, int16_t _stallSensitivity);
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        dirCls::set(d);
        direction = d;
    }
    inline void enable() final {
        enableCls::on();
        isEnabled = true;
    }
    inline void disable() final {
        enableCls::off();
        isEnabled = false;
    }
    virtual void eepromHandle() final;
    void eepromReserve();
    // Return true if setting microsteps is supported
    virtual bool implementsSetMicrosteps() final { return true; }
    // Return true if setting current in software is supported
    virtual bool implementsSetMaxCurrent() final { return true; }
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) final;
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) final;
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() final;
    virtual void afterHoming() final;
    virtual void handleMCode(GCode& com) final;
    void timer500ms();
};
/// Plain stepper driver with optional endstops attached.
class Mirror2StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2;
    Mirror2StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2) { }
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
    }
    inline void step() final {
        motor1->step();
        motor2->step();
    }
    inline void unstep() final {
        motor1->unstep();
        motor2->unstep();
    }
    inline void dir(bool d) final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        direction = d;
    }
    inline void enable() final {
        motor1->enable();
        motor2->enable();
    }
    inline void disable() final {
        motor1->disable();
        motor2->disable();
    }
    inline void handleMCode(GCode& com) final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return motor1->overridesResolution() || motor2->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
    }
    virtual void eepromHandle() final {
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
        , motor3(m3) { }
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
        motor3->setAxis(ax);
    }
    inline void step() final {
        motor1->step();
        motor2->step();
        motor3->step();
    }
    inline void unstep() final {
        motor1->unstep();
        motor2->unstep();
        motor3->unstep();
    }
    inline void dir(bool d) final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        motor3->dir(d);
        direction = d;
    }
    inline void enable() final {
        motor1->enable();
        motor2->enable();
        motor3->enable();
    }
    inline void disable() final {
        motor1->disable();
        motor2->disable();
        motor3->disable();
    }
    inline void handleMCode(GCode& com) final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
        motor3->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return motor1->overridesResolution() || motor2->overridesResolution() || motor3->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
        motor3->menuConfig(action, data);
    }
    virtual void eepromHandle() final {
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
        , motor4(m4) { }
    virtual void setAxis(fast8_t ax) {
        StepperDriverBase::setAxis(ax);
        motor1->setAxis(ax);
        motor2->setAxis(ax);
        motor3->setAxis(ax);
        motor4->setAxis(ax);
    }
    inline void step() final {
        motor1->step();
        motor2->step();
        motor3->step();
        motor4->step();
    }
    inline void unstep() final {
        motor1->unstep();
        motor2->unstep();
        motor3->unstep();
        motor4->unstep();
    }
    inline void dir(bool d) final {
        // Com::printFLN(PSTR("SD:"), (int)d);
        motor1->dir(d);
        motor2->dir(d);
        motor3->dir(d);
        motor4->dir(d);
        direction = d;
    }
    inline void enable() final {
        motor1->enable();
        motor2->enable();
        motor3->enable();
        motor4->enable();
    }
    inline void disable() final {
        motor1->disable();
        motor2->disable();
        motor3->disable();
        motor4->disable();
    }
    inline void handleMCode(GCode& com) final {
        motor1->handleMCode(com);
        motor2->handleMCode(com);
        motor3->handleMCode(com);
        motor4->handleMCode(com);
    }
    // If true the stepper usage will not offer resolution modification in GUI
    virtual bool overridesResolution() { return motor1->overridesResolution() || motor2->overridesResolution() || motor3->overridesResolution() || motor4->overridesResolution(); }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) {
        motor1->menuConfig(action, data);
        motor2->menuConfig(action, data);
        motor3->menuConfig(action, data);
        motor4->menuConfig(action, data);
    }
    virtual void eepromHandle() final {
        motor1->eepromHandle();
        motor2->eepromHandle();
        motor3->eepromHandle();
        motor4->eepromHandle();
    }
};
