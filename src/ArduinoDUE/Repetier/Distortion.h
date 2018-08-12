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

#ifndef _DISTORTION_H
#define _DISTORTION_H

#if DISTORTION_CORRECTION || defined(DOXYGEN)
/** \brief Handle distortion related stuff.

Distortion correction can be used to solve problems resulting from an uneven build plate.
It allows measuring a nxn grid with a z-probe and add these correction to all moves.
Normally you start at the bottom with 100% correction and at 0.5mm you start reducing correction
until it vanishes completely at 1-3 mm.

The stored values are steps required to reach the bumped level assuming you are at zMin. So if you have a 1mm indentation
it contains -steps per mm.
*/
class Distortion {
public:
    Distortion();
    void init();
    void enable(bool permanent = true);
    void disable(bool permanent = true);
    bool measure(void);
    /** \brief Compute distortion correction at given position.

    The current tool offset is added to the CNC position to reference the right distortion point.

    \param x coordinate in CMC steps.
    \param y coordinate in CMC steps.
    \param z coordinate in CMC steps.
    \return Correction required in z steps.
    */
    int32_t correct(int32_t x, int32_t y, int32_t z) const;
    void updateDerived();
    void reportStatus();
    bool isEnabled() {
        return enabled;
    }
    int32_t zMaxSteps() {
        return zEnd;
    }
    void set(float x, float y, float z);
    void showMatrix();
    void resetCorrection();
private:
    int matrixIndex(fast8_t x, fast8_t y) const;
    int32_t getMatrix(int index) const;
    void setMatrix(int32_t val, int index);
    bool isCorner(fast8_t i, fast8_t j) const;
    INLINE int32_t extrapolatePoint(fast8_t x1, fast8_t y1, fast8_t x2, fast8_t y2) const;
    void extrapolateCorner(fast8_t x, fast8_t y, fast8_t dx, fast8_t dy);
    void extrapolateCorners();

    // attributes
#if DRIVE_SYSTEM == DELTA
    int32_t step;
    int32_t radiusCorrectionSteps;
#else
    int32_t xCorrectionSteps, xOffsetSteps;
    int32_t yCorrectionSteps, yOffsetSteps;
#endif
    int32_t zStart, zEnd;
#if !DISTORTION_PERMANENT
    int32_t matrix[DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS];
#endif
    bool enabled;
};
#endif //DISTORTION_CORRECTION

#endif
