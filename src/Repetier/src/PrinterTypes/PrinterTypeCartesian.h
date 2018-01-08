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
};
#endif
