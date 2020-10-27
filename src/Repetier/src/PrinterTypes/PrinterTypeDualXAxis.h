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

    What you need to know about dual X axis:
    We use the A axis to drive the right x axis, so define all settings for this!
    We support ditto and also mirroring the ditto mode.
    Lazy homing means that the tools remain inside homing position until an extrusion in positive direction happens.
*/

#if PRINTER_TYPE == PRINTER_TYPE_DUAL_X

#if NUM_AXES < 5
#error Dual X requires A axis for second X axis. So minimum NUM_AXES required is 5!
#endif

class PrinterType {
    static bool leftParked, rightParked;
    static uint8_t lazyMode;
    static int activeMotor;
    static float endPos[2], targetReal;
    static bool dontChangeCoords;
    static float bedRectangle[2][2];
    static float bedCenter;
    static uint16_t eeprom; // start position eeprom
    static fast8_t activeAxis;
    static bool xMoved;

public:
    // Are subdivisions required due to nonlinear kinematics
    static bool subdivisionsRequired() {
        return false;
    }

    static void transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]);

    static void homeAxis(fast8_t axis);
    static void closestAllowedPositionWithNewXYOffset(float pos[NUM_AXES], float offX, float offY, float safety);
    static bool positionOnBed(float pos[2]);
    static void getBedRectangle(float& xmin, float& xmax, float& ymin, float& ymax);

    static bool positionAllowed(float pos[NUM_AXES], float zOfficial);
    static void disableAllowedStepper();
    /** During probing or homing a move in steps might be needed.
     * This returns the acceleration to use. */
    static float accelerationForMoveSteps(fast8_t axes);
    static float feedrateForMoveSteps(fast8_t axes);
    static void deactivatedTool(fast8_t id);
    static void activatedTool(fast8_t id);
    static void toolchangeFinished();
    static void eepromHandle();
    static void restoreFromConfiguration();
    static void init();
    static void updateDerived();
    static void enableMotors(fast8_t axes);
    static bool queueMove(float feedrate, bool secondaryMove);
    static inline bool supportsDittoMirror() { return true; }
    static void setDittoMode(fast8_t count, bool mirror);
    static bool ignoreAxisForLength(fast8_t axis);
    static void transformedToOfficial(float trans[NUM_AXES], float official[NUM_AXES]);
    static void officialToTransformed(float official[NUM_AXES], float trans[NUM_AXES]);
    static void park(GCode* com);
    static fast8_t axisForTool(fast8_t toolId);
    static fast8_t getActiveAxis() { return activeAxis; }
    static bool canSelectTool(fast8_t toolId);
    static void M290(GCode* com);
    static void M360();
    static bool runMCode(GCode* com);
    static bool runGCode(GCode* com);
    static PGM_P getGeometryName();
};
#define MACHINE_TYPE "Dual X"

#endif
