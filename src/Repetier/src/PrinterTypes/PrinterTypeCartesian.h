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

#if PRINTER_TYPE == 0

class PrinterType {
public:
    // Are subdivisions required due to nonlinear kinematics
    static bool subdivisionsRequired() {
        return false;
    }

    static void transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]);

    static void homeAxis(fast8_t axis);

    static bool positionAllowed(float pos[NUM_AXES]);
    static void disableAllowedStepper();
    /** During probing or homing a move in steps might be needed.
     * This returns the acceleration to use. */
    static float accelerationForMoveSteps(fast8_t axes);
    static float feedrateForMoveSteps(fast8_t axes);
    static void deactivatedTool(fast8_t id);
    static void activatedTool(fast8_t id);
    static void eepromHandle();
    static void restoreFromConfiguration();
    static void init();
    static void updateDerived();
    static void enableMotors(fast8_t axes);
    static inline void queueMove(float feedrate, bool secondaryMove) {
        Motion1::queueMove(feedrate, secondaryMove);
    }
    static inline bool supportsDittoMirror() { return false; }
    static void setDittoMode(fast8_t count, bool mirror);
    static inline bool ignoreAxisForLength(fast8_t axis) { return false; }
    static void transformedToOfficial(float trans[NUM_AXES], float official[NUM_AXES]);
    static void officialToTransformed(float official[NUM_AXES], float trans[NUM_AXES]);
};
#endif
