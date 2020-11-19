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
    virtual void set(bool triggered) { }
    virtual void report() {
        Com::printF(update() ? Com::tHSpace : Com::tLSpace);
    }
    virtual void setParent(EndstopDriver* p) { }
    // Called from dependent end stops
    virtual void updateMaster() { }
    virtual void setAttached(bool attach) { }
    virtual bool isAttached() { return true; } // SW Endstops always "attached".
    virtual ~EndstopDriver() {}
};

class EndstopNoneDriver : public EndstopDriver {
public:
    inline virtual bool update() final {
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
template <class inp>
class EndstopSwitchDriver : public EndstopDriver {
    fast8_t state;
    EndstopDriver* parent;

public:
    EndstopSwitchDriver()
        : state(false)
        , parent(nullptr) { }
    virtual bool update() final;
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    virtual void setParent(EndstopDriver* p) {
        parent = p;
    }
};

template <class inp, int axis, bool dir>
class EndstopSwitchHardwareDriver : public EndstopDriver {
    fast8_t state;
    EndstopDriver* parent;
    void_fn_t callbackFunc;
    uint8_t attachPin;
    bool attached;

public:
    EndstopSwitchHardwareDriver(void_fn_t cb);
    void updateReal();

    inline virtual bool update() final {
        return state;
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void setParent(EndstopDriver* p) final {
        parent = p;
    }
    virtual void setAttached(bool attach) final;
    inline virtual bool isAttached() final {
        return attached;
    }
};

template <class inp, int level>
class EndstopSwitchDebounceDriver : public EndstopDriver {
    fast8_t state;

public:
    EndstopSwitchDebounceDriver()
        : state(0) { }
    inline virtual bool update() final;
    inline virtual bool triggered() final {
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
        : state(false) { }
    inline virtual bool update() final {
        return state;
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    virtual void set(bool triggered) final;
};

/** Merge 2 endstops into 1. Returns only true if both endstops are triggered. */
class EndstopMerge2 : public EndstopDriver {
    EndstopDriver *e1, *e2;
    fast8_t state;
    fast8_t axis;
    bool dir;

public:
    EndstopMerge2(EndstopDriver* _e1, EndstopDriver* _e2, fast8_t _axis, bool _dir)
        : e1(_e1)
        , e2(_e2)
        , state(0)
        , axis(_axis)
        , dir(_dir) {
        e1->setParent(this);
        e2->setParent(this);
    }
    inline virtual bool update() final {
        return (state = (e1->update() && e2->update()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void setAttached(bool attach) final {
        e1->setAttached(attach);
        e2->setAttached(attach);
    }
    inline virtual bool isAttached() final {
        return (e1->isAttached() && e2->isAttached());
    }
    virtual void report() final {
        Com::printF(update() ? Com::tHSpace : Com::tLSpace);
        Com::print('(');
        e1->report();
        e2->report();
        Com::print(')');
    }
    virtual void updateMaster() final {
        fast8_t oldState = state;
        update();
        if (state != oldState) {
            if (axis >= 0 && state) { // tell motion planner
                endstopTriggered(axis, dir);
            }
        }
    }
};

/** Merge 3 endstops into 1. Returns only true if both endstops are triggered. 
 * This is required for z max endstop of deltas that merge the 3 motor max endstops.
*/
class EndstopMerge3 : public EndstopDriver {
    EndstopDriver *e1, *e2, *e3;
    fast8_t state;
    fast8_t axis;
    bool dir;

public:
    EndstopMerge3(EndstopDriver* _e1, EndstopDriver* _e2, EndstopDriver* _e3, fast8_t _axis, bool _dir)
        : e1(_e1)
        , e2(_e2)
        , e3(_e3)
        , state(0)
        , axis(_axis)
        , dir(_dir) {
        e1->setParent(this);
        e2->setParent(this);
        e3->setParent(this);
    }
    inline virtual bool update() final {
        return (state = (e1->update() && e2->update() && e3->update()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void setAttached(bool attach) final {
        e1->setAttached(attach);
        e2->setAttached(attach);
        e3->setAttached(attach);
    }
    inline virtual bool isAttached() final {
        return (e1->isAttached() && e2->isAttached() && e3->isAttached());
    }
    virtual void report() final {
        Com::printF(update() ? Com::tHSpace : Com::tLSpace);
        Com::print('(');
        e1->report();
        e2->report();
        e3->report();
        Com::print(')');
    }
    virtual void updateMaster() final {
        fast8_t oldState = state;
        update();
        if (state != oldState) {
            if (axis >= 0 && state) { // tell motion planner
                endstopTriggered(axis, dir);
            }
            // Com::printFLN(PSTR("MState:"), (int)state); // TEST
        }
    }
};

/** Merge 4 endstops into 1. Returns only true if both endstops are triggered. 
*/
class EndstopMerge4 : public EndstopDriver {
    EndstopDriver *e1, *e2, *e3, *e4;
    fast8_t state;
    fast8_t axis;
    bool dir;

public:
    EndstopMerge4(EndstopDriver* _e1, EndstopDriver* _e2,
                  EndstopDriver* _e3, EndstopDriver* _e4, fast8_t _axis, bool _dir)
        : e1(_e1)
        , e2(_e2)
        , e3(_e3)
        , e4(_e4)
        , state(0)
        , axis(_axis)
        , dir(_dir) {
        e1->setParent(this);
        e2->setParent(this);
        e3->setParent(this);
        e4->setParent(this);
    }
    inline virtual bool update() final {
        return (state = (e1->update() && e2->update() && e3->update() && e4->update()));
    }
    inline virtual bool triggered() final {
        return state;
    }
    inline virtual bool implemented() final {
        return true;
    }
    inline virtual void setAttached(bool attach) final {
        e1->setAttached(attach);
        e2->setAttached(attach);
        e3->setAttached(attach);
        e4->setAttached(attach);
    }
    inline virtual bool isAttached() final {
        return (e1->isAttached() && e2->isAttached() && e3->isAttached() && e4->isAttached());
    }
    virtual void report() final {
        Com::printF(update() ? Com::tHSpace : Com::tLSpace);
        Com::print('(');
        e1->report();
        e2->report();
        e3->report();
        e4->report();
        Com::print(')');
    }
    virtual void updateMaster() final {
        fast8_t oldState = state;
        update();
        if (state != oldState) {
            if (axis >= 0 && state) { // tell motion planner
                endstopTriggered(axis, dir);
            }
        }
    }
};
