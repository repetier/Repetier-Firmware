class EndstopDriver;
class GCode;
enum class GUIAction;

class StepperDriverBase {
protected:
    EndstopDriver* minEndstop;
    EndstopDriver* maxEndstop;
    bool direction;
    fast8_t axisBit;
    fast8_t axis;

public:
    StepperDriverBase(EndstopDriver* minES, EndstopDriver* maxES)
        : minEndstop(minES)
        , maxEndstop(maxES)
        , direction(true) {}
    virtual ~StepperDriverBase() {}
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
    inline bool updateEndstop() {
        if (direction) {
            return maxEndstop->update();
        } else {
            return minEndstop->update();
        }
    }
    inline void setAxis(fast8_t ax) {
        axisBit = 1 << ax;
        axis = ax;
    }
    inline fast8_t getAxisBit() { return axisBit; }
    inline fast8_t getAxis() { return axis; }
    /// Allows initialization of driver e.g. current, microsteps
    virtual void init() {}
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
    virtual void setMicrosteps(int microsteps) {}
    /// Set max current as range 0..255 or mA depedning on driver
    virtual void setMaxCurrent(int max) {}
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() {}
    virtual void afterHoming() {}
    virtual void handleMCode(GCode& com) {}
    // If true the stepper usager will not offer resolution modification in GUI
    virtual bool overridesResolution() { return false; }
    // Configuration in GUI
    virtual void menuConfig(GUIAction action, void* data) {}
    // Allow having own settings e.g. current, microsteps
    virtual void eepromHandle() {}
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
        , stepper(_stepper) { position = 0; }
    virtual ~ObservableStepperDriver() {}
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
    /// Allows initialization of driver e.g. current, microsteps
    virtual void init() final {}
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
};

/**
  Converts incoming step frequency from = steps/length unit to to = steps/length unit steps.
  The only requirement is that the incoming frequency must be higher then the outgoing
  frequency. The maximum error is one step.
 */
template <class driver>
class AdjustResolutionStepperDriver : public StepperDriverBase {
    driver* stepper;
    uint32_t accumulator;
    uint32_t from;
    uint32_t to;
    uint16_t eprStart;

public:
    AdjustResolutionStepperDriver(driver* _stepper, int32_t _from, int32_t _to)
        : StepperDriverBase(_stepper->getMinEndstop(), _stepper->getMaxEndstop())
        , stepper(_stepper)
        , accumulator(0)
        , from(_from)
        , to(_to) {}
    virtual ~AdjustResolutionStepperDriver() {}
    inline EndstopDriver* getMinEndstop() { return minEndstop; }
    inline EndstopDriver* getMaxEndstop() { return maxEndstop; }
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
        : StepperDriverBase(minES, maxES) {}
    virtual void init() { disable(); }
    inline void step() final {
        stepCls::on();
    }
    inline void unstep() final {
        stepCls::off();
    }
    inline void dir(bool d) final {
        // Com::printFLN(PSTR("SD:"), (int)d);
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

/// Plain stepper driver with optional endstops attached.
class Mirror2StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2;
    Mirror2StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2) {}
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
};

/// Plain stepper driver with optional endstops attached.
class Mirror3StepperDriver : public StepperDriverBase {
public:
    StepperDriverBase *motor1, *motor2, *motor3;
    Mirror3StepperDriver(StepperDriverBase* m1, StepperDriverBase* m2, StepperDriverBase* m3, EndstopDriver* minES, EndstopDriver* maxES)
        : StepperDriverBase(minES, maxES)
        , motor1(m1)
        , motor2(m2)
        , motor3(m3) {}
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
        , motor4(m4) {}
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
};
