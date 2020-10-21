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

/*
  This is a segment run at constant velocity. 
*/
#define NUM_MOTION3_BUFFER 32

class Motion3Buffer { // 28 byte 4 axis 8 bit processor
public:
    fast8_t id;
    fast8_t parentId;
    ufast8_t directions;
    ufast8_t usedAxes;
    fast8_t last;
    bool checkEndstops;
    int delta[NUM_AXES];
    int error[NUM_AXES];
    int errorUpdate;
    unsigned int stepsRemaining;
    int secondSpeed;
};

class Motion3 {
    friend class Motion2;

public:
    static Motion3Buffer buffers[NUM_MOTION3_BUFFER];
    static volatile fast8_t length; // shared between threads
    static fast8_t last;            // used by pushing thread
    static fast8_t nextActId;       // used only inside interrupt
    static fast8_t skipParentId;    // Skip blocks from this id - endstop was triggered
    static fast8_t lastParentId;
    static ufast8_t lastDirection; // Last direction for faster switches
    static Motion3Buffer* act;
    static Motion2Buffer* actM2;
    static Motion1Buffer* actM1;
    static void init();
    /* Stepper timer called at STEPPER_FREQUENCY
    This is a very small code that is run at the highest interrupt level
    to ensure best accuracy in motion.
    */
    static void timer();
    static void setDirectionsForNewMotors();
    /** Return pointer to next available buffer or nullptr. */
    static inline Motion3Buffer* tryReserve() {
        if (length < NUM_MOTION3_BUFFER) {
            return &buffers[last];
        }
        return nullptr;
    }
    /** Increment head position an dincrement length */
    static inline void pushReserved() {
        last++;
        if (last == NUM_MOTION3_BUFFER) {
            last = 0;
        }
        InterruptProtectedBlock noInts;
        length++;
    }
    static bool activateNext(); // select next block, returns true if directions were changed
    static INLINE void unstepMotors() {
#ifdef XMOTOR_SWITCHABLE
        Motion1::motors[X_AXIS]->unstep();
#else
        XMotor.unstep();
#endif
        YMotor.unstep();
        ZMotor.unstep();
        if (Motion1::dittoMode) {
#if NUM_TOOLS > 6
            for (fast8_t i = 0; i <= NUM_TOOLS; i++) {
                Tool::getTool(i)->unstepMotor();
            }
#else
#if NUM_TOOLS > 0
            Tool::tools[0]->unstepMotor();
#endif
#if NUM_TOOLS > 1
            Tool::tools[1]->unstepMotor();
#endif
#if NUM_TOOLS > 2
            Tool::tools[2]->unstepMotor();
#endif
#if NUM_TOOLS > 3
            Tool::tools[3]->unstepMotor();
#endif
#if NUM_TOOLS > 4
            Tool::tools[4]->unstepMotor();
#endif
#if NUM_TOOLS > 5
            Tool::tools[5]->unstepMotor();
#endif
#endif
        } else {
            // Tool::getActiveTool()->unstepMotor();
            if (Motion1::motors[E_AXIS]) {
                Motion1::motors[E_AXIS]->unstep();
            }
        }
#if NUM_AXES > A_AXIS
        AMotor.unstep();
#endif
#if NUM_AXES > B_AXIS
        BMotor.unstep();
#endif
#if NUM_AXES > C_AXIS
        CMotor.unstep();
#endif
    }
    static void reportBuffers();
};
