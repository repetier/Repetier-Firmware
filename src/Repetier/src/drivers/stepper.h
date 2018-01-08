class EndstopDriver;

class StepperDriverBase {
public:
    StepperDriverBase(EndstopDriver *minES, EndstopDriver *maxES):minEndstop(minES),maxEndstop(maxES),direction(true) {}
    virtual ~StepperDriverBase() {}
    EndstopDriver *getMinEndstop();
    EndstopDriver *getMaxEndstop();
    /// Allows initialization of driver e.g. current, microsteps
    virtual void init() {}
    /// Executes the step if endstop is not triggered. Return tru eif endstop is triggered
    virtual bool stepCond() = 0;
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
    virtual bool implementSetMicrosteps() {return false;}
    // Return true if setting current in software is supported
    virtual bool implementSetMaxCurrent() {return false;}
    /// Set microsteps. Must be a power of 2.
    virtual void setMicrosteps(int microsteps) {}
    /// Set max current as range 0..255
    virtual void setMaxCurrent(int max) {}
    // Called before homing starts. Can be used e.g. to disable silent mode
    // or otherwise prepare for endstop detection.
    virtual void beforeHoming() {}
    virtual void afterHoming() {}
    EndstopDriver *minEndstop;
    EndstopDriver *maxEndstop;
    bool direction;
    // uint32_t position;
};

/// Plain stepper driver with optional endstops attached.
template<class stepCls,class dirCls,class enableCls>
class SimpleStepperDriver : public StepperDriverBase {
public:
    SimpleStepperDriver(EndstopDriver *minES, EndstopDriver *maxES):StepperDriverBase(minES, maxES) {}
    inline bool stepCond() final {        
        if(direction) {
            maxEndstop->update();
            if(!maxEndstop->triggered()) {
                stepCls::on();
                return false;
            }
        } else {
            minEndstop->update();
            if(!minEndstop->triggered()) {
                stepCls::on();
                return false;
            }
        }
        return true;
    }
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
