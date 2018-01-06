
class EndstopDriver {
public:
    // Called by stepper driver before each step to update state
    virtual void update() = 0;
    // Returns state of endstop
    virtual bool triggered() = 0;
    // Returns true if this is a real endstop
    virtual bool implemented() = 0;
    // Special case for drivers that sense endstop to set state instead of using update
    virtual void set(bool triggered) {}
};

class EndstopNoneDriver: public EndstopDriver {
public:    
    inline virtual void update() final {}
    inline virtual bool get() final {
        return false;
    }
    inline virtual bool implemented() final {
        return false;
    }
    inline virtual bool triggered() final {
        return false;
    }
};

/**
  Input based on a digital switch. Input class must have a 
  get function to detect state.
 */
template<class inp>
class EndstopSwitchDriver: public EndstopDriver {
    fast8_t state;
public:
    EndstopSwitchDriver():state(false) {}
    inline virtual void update() final {
        state = inp::get();
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

template<class inp, int level>
class EndstopSwitchDebounceDriver: public EndstopDriver {
    fast8_t state;
public:    
    inline virtual void update() final {
        if(inp::get()) {
            if(state < level) {
                state++;
            }
        } else {
            state = 0;
        }
    }
    inline virtual bool triggert() final {
        return state == level;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

/**
  Input based on a digital switch. Input class must have a 
  get function to detect state.
 */
class EndstopStepperControlledDriver: public EndstopDriver {
    fast8_t state;
public:
    EndstopStepperControlledDriver():state(false) {}
    inline virtual void update() final {
    }
    inline virtual bool get() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void set(bool triggered) final {
        state = triggered;
    }
};
