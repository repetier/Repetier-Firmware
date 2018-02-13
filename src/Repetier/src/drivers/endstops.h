/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

class EndstopDriver {
public:
    // Called by stepper driver before each step to update state
    virtual bool update() = 0;
    // Returns state of endstop
    virtual bool triggered() = 0;
    // Returns true if this is a real endstop
    virtual bool implemented() = 0;
    // Special case for drivers that sense endstop to set state instead of using update
    virtual void set(bool triggered) {}
};

class EndstopNoneDriver : public EndstopDriver {
public:
    inline virtual bool update() final { return false; }
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
template <class inp>
class EndstopSwitchDriver : public EndstopDriver {
    fast8_t state;

public:
    EndstopSwitchDriver()
        : state(false) {}
    inline virtual bool update() final {
        return (state = inp::get());
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

template <class inp, int axis>
class EndstopSwitchHardwareDriver : public EndstopDriver {
    fast8_t state;

public:
    EndstopSwitchHardwareDriver()
        : state(false) {}
    inline void updateReal() {
        fast8_t newState = inp::get();
        if (state != newState) {
            if (axis >= 0 && newState) { // tell motion planner
                endstopTriggered(axis);
            }
            state = newState;
        }
    }

    inline virtual bool update() final {
        return state;
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

template <class inp, int level>
class EndstopSwitchDebounceDriver : public EndstopDriver {
    fast8_t state;

public:
    inline virtual bool update() final {
        if (inp::get()) {
            if (state < level) {
                state++;
            }
        } else {
            state = 0;
        }
        return state;
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
class EndstopStepperControlledDriver : public EndstopDriver {
    fast8_t state;

public:
    EndstopStepperControlledDriver()
        : state(false) {}
    inline virtual bool update() final {
        return state;
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void set(bool triggered) final {
        state = triggered;
    }
};

/** Merge 2 endstops into 1. Returns only true if both endstops are triggered. */
class EndstopMerge2 : public EndstopDriver {
    fast8_t state;
    EndstopDriver *e1, *e2;

public:
    EndstopMerge2(EndstopDriver* _e1, EndstopDriver* _e2)
        : state(false)
        , e1(_e1)
        , e2(_e2) {}
    inline virtual bool update() final {
        return (state = (e1->triggered() && e2->triggered()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

/** Merge 3 endstops into 1. Returns only true if both endstops are triggered. 
 * This is required for z max endstop of deltas that merge the 3 motor max endstops.
*/
class EndstopMerge3 : public EndstopDriver {
    fast8_t state;
    EndstopDriver *e1, *e2, *e3;

public:
    EndstopMerge3(EndstopDriver* _e1, EndstopDriver* _e2, EndstopDriver* _e3)
        : state(false)
        , e1(_e1)
        , e2(_e2)
        , e3(_e3) {}
    inline virtual bool update() final {
        return (state = (e1->triggered() && e2->triggered() && e3->triggered()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};

/** Merge 4 endstops into 1. Returns only true if both endstops are triggered. 
*/
class EndstopMerge4 : public EndstopDriver {
    fast8_t state;
    EndstopDriver *e1, *e2, *e3, *e4;

public:
    EndstopMerge4(EndstopDriver* _e1, EndstopDriver* _e2,
                  EndstopDriver* _e3, EndstopDriver* _e4)
        : state(false)
        , e1(_e1)
        , e2(_e2)
        , e3(_e3)
        , e4(_e4) {}
    inline virtual bool update() final {
        return (state = (e1->triggered() && e2->triggered() && e3->triggered() && e4->triggered()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
};
