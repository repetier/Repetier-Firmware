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

#include "Repetier.h"

#if PRINTER_TYPE == PRINTER_TYPE_DELTA

#define degToRad(x) ((x)*M_PI / 180.0f)

// --------- Autocalibrate delta -------------
// Based on David Escher's great delta calibrator

#define NUM_DELTA_CALIB_POINTS 11
class DeltaCalibrator {
    float diagonal;
    float radius;
    float homedHeight;
    float xStop, yStop, zStop; // end stop offsets
    float xAdj, yAdj, zAdj;    // tower angle adjustments

    float Xbc, Xca, Xab, Ybc, Yca, Yab;
    float coreFa, coreFb, coreFc;
    float Q, Q2, D2[3];
    float towerX[3], towerY[3];
    float homedCarriageHeight;

    DeltaCalibrator() {
        diagonal = PrinterType::diagonal;
        radius = PrinterType::horizontalRadius;
        homedHeight = Motion1::maxPos[Z_AXIS];
        xStop = PrinterType::homeOffsetA;
        yStop = PrinterType::homeOffsetB;
        zStop = PrinterType::homeOffsetC;
        xAdj = yAdj = zAdj = 0;

        // prevent not initialized warnings
        Xbc = Xca = Xab = Ybc = Yca = Yab = 0;
        coreFa = coreFb = coreFc = Q = Q2 = D2[0] = D2[1] = D2[2] = 0;
        towerX[0] = towerX[1] = towerX[2] = towerY[0] = towerY[1] = towerY[2] = 0;
        homedCarriageHeight = 0;
        recalc();
    }

    DeltaCalibrator(DeltaCalibrator& o)
        : DeltaCalibrator() {
        diagonal = o.diagonal;
        radius = o.radius;
        homedHeight = o.homedHeight;
        xStop = o.xStop;
        yStop = o.yStop;
        zStop = o.zStop;
        xAdj = o.xAdj;
        yAdj = o.yAdj;
        zAdj = o.zAdj;
        recalc();
    }

    void storeInPrinter() {
        PrinterType::diagonal = diagonal;
        PrinterType::horizontalRadius = radius;
        Motion1::maxPos[Z_AXIS] = homedHeight;
        PrinterType::homeOffsetA = xStop;
        PrinterType::homeOffsetB = yStop;
        PrinterType::homeOffsetC = zStop;
        PrinterType::angleA += xAdj;
        PrinterType::angleB += yAdj;
        PrinterType::angleC += zAdj;
        EEPROM::markChanged();
    }

    float inverseTransform(float Ha, float Hb, float Hc) {
        float Fa = coreFa + Ha * Ha;
        float Fb = coreFb + Hb * Hb;
        float Fc = coreFc + Hc * Hc;
        float P = Xbc * Fa + Xca * Fb + Xab * Fc;
        float S = Ybc * Fa + Yca * Fb + Yab * Fc;
        float R = 2.0 * (Xbc * Ha + Xca * Hb + Xab * Hc);
        float U = 2.0 * (Ybc * Ha + Yca * Hb + Yab * Hc);
        float R2 = R * R, U2 = U * U;
        float A = U2 + R2 + Q2;
        float minusHalfB = S * U + P * R + Ha * Q2 + towerX[0] * U * Q - towerY[0] * R * Q;
        float C0 = S + towerX[0] * Q;
        float C = C0 * C0;
        C0 = P - towerY[0] * Q;
        C += C0 * C0 + (Ha * Ha - diagonal * diagonal) * Q2;
        return (minusHalfB - sqrtf(minusHalfB * minusHalfB - A * C)) / A;
    }

    void recalc() {
        float radiusA = radius + PrinterType::radiusCorrectionA;
        float radiusB = radius + PrinterType::radiusCorrectionB;
        float radiusC = radius + PrinterType::radiusCorrectionC;
        towerX[0] = radiusA * cos(degToRad(PrinterType::angleA + xAdj));
        towerY[0] = radiusA * sin(degToRad(PrinterType::angleA + xAdj));
        towerX[1] = radiusB * cos(degToRad(PrinterType::angleB + yAdj));
        towerY[1] = radiusB * sin(degToRad(PrinterType::angleB + yAdj));
        towerX[2] = radiusC * cos(degToRad(PrinterType::angleC + zAdj));
        towerY[2] = radiusC * sin(degToRad(PrinterType::angleC + zAdj));
        Xbc = towerX[2] - towerX[1];
        Xca = towerX[0] - towerX[2];
        Xab = towerX[1] - towerX[0];
        Ybc = towerY[2] - towerY[1];
        Yca = towerY[0] - towerY[2];
        Yab = towerY[1] - towerY[0];
        coreFa = RMath::sqr(towerX[0]) + RMath::sqr(towerY[0]);
        coreFb = RMath::sqr(towerX[1]) + RMath::sqr(towerY[1]);
        coreFc = RMath::sqr(towerX[2]) + RMath::sqr(towerY[2]);
        Q = 2.0 * (Xca * Yab - Xab * Yca);
        Q2 = Q * Q;
        D2[X_AXIS] = RMath::sqr(diagonal + PrinterType::correctionA);
        D2[Y_AXIS] = RMath::sqr(diagonal + PrinterType::correctionB);
        D2[Z_AXIS] = RMath::sqr(diagonal + PrinterType::correctionC);
        float tempHeight = diagonal;
        homedCarriageHeight = homedHeight + tempHeight - inverseTransform(tempHeight, tempHeight, tempHeight);
    }

    void normaliseEndstopAdjustments() {
        float eav = RMath::min(xStop, yStop, zStop);
        xStop -= eav;
        yStop -= eav;
        zStop -= eav;
        homedHeight += eav;
        homedCarriageHeight += eav;
    }

    float computeDerivative(int deriv, float ha, float hb, float hc) {
        constexpr float perturb = 0.2f; // perturbation amount in mm or degrees
        DeltaCalibrator hiParams(*this);
        DeltaCalibrator loParams(*this);
        switch (deriv) {
        case 0:
        case 1:
        case 2:
            break;

        case 3:
            hiParams.radius += perturb;
            loParams.radius -= perturb;
            break;

        case 4:
            hiParams.xAdj += perturb;
            loParams.xAdj -= perturb;
            break;

        case 5:
            hiParams.yAdj += perturb;
            loParams.yAdj -= perturb;
            break;

        case 6:
            hiParams.diagonal += perturb;
            loParams.diagonal -= perturb;
            break;
        }

        hiParams.recalc();
        loParams.recalc();

        float zHi = hiParams.inverseTransform(deriv == 0 ? ha + perturb : ha, deriv == 1 ? hb + perturb : hb, deriv == 2 ? hc + perturb : hc);
        float zLo = loParams.inverseTransform(deriv == 0 ? ha - perturb : ha, deriv == 1 ? hb - perturb : hb, deriv == 2 ? hc - perturb : hc);

        return (zHi - zLo) / (2.0f * perturb);
    }

    // Perform 3, 4, 6 or 7-factor adjustment.
    // The input vector contains the following parameters in this order:
    //  A, B and C endstop adjustments
    //  If we are doing 4-factor adjustment, the next argument is the delta radius. Otherwise:
    //  A tower X position adjustment
    //  B tower X position adjustment
    //  C tower Y position adjustment
    //  Diagonal rod length adjustment
    void adjust(int numFactors, float v[7], bool norm) {
        float oldCarriageHeightA = homedCarriageHeight + xStop; // save for later

        // Update endstop adjustments
        xStop += v[X_AXIS];
        yStop += v[Y_AXIS];
        zStop += v[Z_AXIS];
        if (norm) {
            normaliseEndstopAdjustments();
        }
        // Com::printFLN(PSTR("xstop:"), xStop, 3);
        // Com::printFLN(PSTR("ystop:"), yStop, 3);
        // Com::printFLN(PSTR("zstop:"), zStop, 3);
        if (numFactors >= 4) {
            radius += v[3];

            if (numFactors >= 6) {
                xAdj += v[4];
                yAdj += v[5];

                if (numFactors == 7) {
                    diagonal += v[6];
                }
            }
        }
        recalc();

        // Adjusting the diagonal and the tower positions affects the homed carriage height.
        // We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
        float heightError = homedCarriageHeight + xStop - oldCarriageHeightA - v[X_AXIS];
        homedHeight -= heightError;
        homedCarriageHeight -= heightError;
    }

    float transform(float machinePos[NUM_AXES], int axis) {
        return machinePos[Z_AXIS] + sqrtf(D2[axis] - RMath::sqr(machinePos[X_AXIS] - towerX[axis]) - RMath::sqr(machinePos[Y_AXIS] - towerY[axis]));
    }

public:
    static void runDeltaCalibration(int numFactors, bool normalise) {
        if (numFactors != 3 && numFactors != 4 && numFactors != 6 && numFactors != 7) {
            Com::printWarningFLN(PSTR("only 3, 4, 6 and 7 factors are supported"));
        }
        Com::printF(PSTR("Delta calibration with "), numFactors);
        Com::printFLN(PSTR(" factors started"));
        DeltaCalibrator deltaParams;
        // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
        RMatrix<NUM_DELTA_CALIB_POINTS, 3> probeMotorPositions;
        float corrections[NUM_DELTA_CALIB_POINTS];
        float initialSumOfSquares = 0.0;
        float rad = RMath::min(PrinterType::bedRadius, PrinterType::printRadius);
        float zBedProbePoints[NUM_DELTA_CALIB_POINTS];
        Motion1::setAutolevelActive(false, true);
        Leveling::setDistortionEnabled(false);
        Motion1::resetTransformationMatrix(true);
        Motion1::homeAxes(7); // Home x, y and z
        Motion1::setTmpPositionXYZ(0, 0, ZProbeHandler::optimumProbingHeight());
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        if (!ZProbeHandler::activate()) {
            return;
        }
        float r, deg;
        for (int i = 0; i < NUM_DELTA_CALIB_POINTS; ++i) {
            float pos[NUM_AXES] = { 0.0f };
            if (i < 6) {
                r = rad;
                deg = PrinterType::angleA + 60.0f * static_cast<float>(i);
            } else if (i == 6) {
                r = 0;
            } else {
                deg = 45.0f + static_cast<float>(i - 7) * 90.0f;
                r = 0.5f * rad;
            }
            pos[X_AXIS] = r * cos(degToRad(deg));
            pos[Y_AXIS] = r * sin(degToRad(deg));
            pos[Z_AXIS] = ZProbeHandler::optimumProbingHeight();
            pos[E_AXIS] = IGNORE_COORDINATE;
            PrinterType::closestAllowedPositionWithNewXYOffset(pos, ZProbeHandler::xOffset(), ZProbeHandler::yOffset(), Z_PROBE_BORDER + 1.0);
            Motion1::moveByOfficial(pos, Motion1::moveFeedrate[X_AXIS], false);
            zBedProbePoints[i] = ZProbeHandler::runProbe();
            if (zBedProbePoints[i] == ILLEGAL_Z_PROBE) {
                GCode::fatalError(PSTR("Delta calibration failed"));
                return;
            }
            zBedProbePoints[i] = pos[Z_AXIS] - zBedProbePoints[i]; // swap sign
            pos[Z_AXIS] = 0.0f;
            corrections[i] = 0.0f;

            probeMotorPositions(i, 0) = deltaParams.transform(pos, 0);
            probeMotorPositions(i, 1) = deltaParams.transform(pos, 1);
            probeMotorPositions(i, 2) = deltaParams.transform(pos, 2);

            initialSumOfSquares += RMath::sqr(zBedProbePoints[i]);
        }
        ZProbeHandler::deactivate();
#ifdef DEBUG_CALIBRATE
        probeMotorPositions.print(PSTR("MotorPositions"));
#endif
        // Do 1 or more Newton-Raphson iterations
        int iteration = 0;
        float expectedRmsError;
        while (true) {
            // Build a Nx7 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
            RMatrix<NUM_DELTA_CALIB_POINTS, 7> derivativeMatrix;
            for (int i = 0; i < NUM_DELTA_CALIB_POINTS; ++i) {
                for (int j = 0; j < numFactors; ++j) {
                    derivativeMatrix(i, j) = deltaParams.computeDerivative(j, probeMotorPositions(i, X_AXIS), probeMotorPositions(i, Y_AXIS), probeMotorPositions(i, Z_AXIS));
                }
            }
#ifdef DEBUG_CALIBRATE
            derivativeMatrix.print(PSTR("derivativeMatrix"));
#endif

            // Now build the normal equations for least squares fitting
            RMatrix<7, 8> normalMatrix; // max. data to store is 7x8 matrix
            for (int i = 0; i < numFactors; ++i) {
                for (int j = 0; j < numFactors; ++j) {
                    float temp = 0.0;
                    for (int k = 0; k < NUM_DELTA_CALIB_POINTS; ++k) {
                        temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
                    }
                    normalMatrix(i, j) = temp;
                }
                float temp = 0;
                for (int k = 0; k < NUM_DELTA_CALIB_POINTS; ++k) {
                    temp += derivativeMatrix(k, i) * -(zBedProbePoints[k] + corrections[k]);
                }
                normalMatrix(i, numFactors) = temp;
            }
#ifdef DEBUG_CALIBRATE
            normalMatrix.print(PSTR("normalMatrix"));
#endif

            float solution[7];
            normalMatrix.gaussJordan(solution, numFactors);
#ifdef DEBUG_CALIBRATE
            normalMatrix.print(PSTR("solvedMatrix"));
            Com::printArrayFLN(PSTR("Solution:"), solution, numFactors, 4);
#endif

            for (int i = 0; i < numFactors; ++i) {
                if (isnan(solution[i])) {
                    Com::printFLN(PSTR("Unable to calculate corrections."));
                    return;
                }
            }

            deltaParams.adjust(numFactors, solution, normalise);

            // Calculate the expected probe heights using the new parameters
            {
                float expectedResiduals[NUM_DELTA_CALIB_POINTS];
                float sumOfSquares = 0.0;
                for (int i = 0; i < NUM_DELTA_CALIB_POINTS; ++i) {
                    for (int axis = 0; axis < 3; ++axis) {
                        probeMotorPositions(i, axis) += solution[axis];
                    }
                    float newZ = deltaParams.inverseTransform(probeMotorPositions(i, 0), probeMotorPositions(i, 1), probeMotorPositions(i, 2));
                    corrections[i] = newZ;
                    expectedResiduals[i] = zBedProbePoints[i] + newZ;
                    sumOfSquares += RMath::sqr(expectedResiduals[i]);
                }

                expectedRmsError = sqrtf(sumOfSquares / NUM_DELTA_CALIB_POINTS);
            }

            // Decide whether to do another iteration Two is slightly better than one, but three doesn't improve things.
            // Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
            ++iteration;
            if (iteration == 2) {
                break;
            }
        }
        deltaParams.storeInPrinter();
        Com::printFLN(PSTR("Deviation before:"), sqrtf(initialSumOfSquares / NUM_DELTA_CALIB_POINTS), 2);
        Com::printFLN(PSTR("Deviation after:"), expectedRmsError, 2);
        Motion1::homeAxes(7); // Home x, y and z
    }
};

float PrinterType::diagonal;
float PrinterType::horizontalRadius;
float PrinterType::bedRadius;
float PrinterType::printRadius;
float PrinterType::printRadiusSquared;
float PrinterType::angleA, PrinterType::angleB, PrinterType::angleC;
float PrinterType::correctionA, PrinterType::correctionB, PrinterType::correctionC;
float PrinterType::radiusCorrectionA, PrinterType::radiusCorrectionB, PrinterType::radiusCorrectionC;
float PrinterType::diagonalSquaredA;
float PrinterType::diagonalSquaredB;
float PrinterType::diagonalSquaredC;
float PrinterType::APosX, PrinterType::APosY;
float PrinterType::BPosX, PrinterType::BPosY;
float PrinterType::CPosX, PrinterType::CPosY;
float PrinterType::homeOffsetA, PrinterType::homeOffsetB, PrinterType::homeOffsetC;
uint16_t PrinterType::eeprom; // start position eeprom
PrinterType::MotionMode PrinterType::mode = PrinterType::MotionMode::MOTION_DELTA;

void PrinterType::setMotionMode(MotionMode newMode) {
    if (mode == newMode) {
        return;
    }
    Motion1::waitForEndOfMoves();
    mode = newMode;
    Motion2::setMotorPositionFromTransformed();
}

bool PrinterType::isAnyEndstopTriggered(bool& moveX, bool& moveY, bool& moveZ) {
    moveX = Motion1::motors[X_AXIS]->getMaxEndstop()->triggered();
    moveY = Motion1::motors[Y_AXIS]->getMaxEndstop()->triggered();
    moveZ = Motion1::motors[Z_AXIS]->getMaxEndstop()->triggered();
    return (moveX || moveY || moveZ);
}

// Moves axis 10 mm down if it is triggered to get a clean start for probing
bool PrinterType::untriggerEndstops() {
    bool moveX, moveY, moveZ;
    if (!isAnyEndstopTriggered(moveX, moveY, moveZ)) {
        return false;
    }
    Motion1::currentPosition[X_AXIS] = Motion1::maxPos[Z_AXIS];
    Motion1::currentPosition[Y_AXIS] = Motion1::maxPos[Z_AXIS];
    Motion1::currentPosition[Z_AXIS] = Motion1::maxPos[Z_AXIS];
    Motion1::updatePositionsFromCurrent();
    setMotionMode(MotionMode::MOTION_PER_AXIS);
    Motion1::setTmpPositionXYZ(
        moveX ? -10.0f : 0,
        moveY ? -10.0f : 0,
        moveZ ? -10.0f : 0);
    Motion1::moveRelativeByOfficial(Motion1::tmpPosition, Motion1::homingFeedrate[Z_AXIS], false);
    Motion1::waitForEndOfMoves();
    moveX = Motion1::motors[X_AXIS]->getMaxEndstop()->triggered();
    moveY = Motion1::motors[Y_AXIS]->getMaxEndstop()->triggered();
    moveZ = Motion1::motors[Z_AXIS]->getMaxEndstop()->triggered();
    setMotionMode(MotionMode::MOTION_DELTA);
    return isAnyEndstopTriggered(moveX, moveY, moveZ);
}

void PrinterType::homeZ() {
    Motion1::toolOffset[X_AXIS] = Motion1::toolOffset[Y_AXIS] = 0;
    if (untriggerEndstops()) {
        Com::printErrorF(PSTR("Unable to untrigger endstops - giving up!"));
        return;
    }
    Motion1::currentPosition[X_AXIS] = 0;
    Motion1::currentPosition[Y_AXIS] = 0;
    Motion1::currentPosition[Z_AXIS] = Motion1::maxPos[Z_AXIS];
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    Motion1::simpleHome(Z_AXIS);

    // Correct end position
    Motion1::waitForEndOfMoves();
    float oldPosition[NUM_AXES];
    FOR_ALL_AXES(i) {
        oldPosition[i] = Motion1::currentPosition[i];
    }
    setMotionMode(MotionMode::MOTION_PER_AXIS);
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    int32_t mShift[NUM_AXES];
    mShift[X_AXIS] = -homeOffsetA * Motion1::resolution[X_AXIS];
    mShift[Y_AXIS] = -homeOffsetB * Motion1::resolution[Y_AXIS];
    mShift[Z_AXIS] = -homeOffsetC * Motion1::resolution[Z_AXIS];
    for (int i = 3; i < NUM_AXES; i++) {
        mShift[i] = 0;
    }
    Motion1::moveRelativeBySteps(mShift);
    Motion1::waitForEndOfMoves();
    setMotionMode(MotionMode::MOTION_DELTA);
    FOR_ALL_AXES(i) {
        Motion1::currentPosition[i] = oldPosition[i];
    }
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    Motion1::setAxisHomed(X_AXIS, true);
    Motion1::setAxisHomed(Y_AXIS, true);
    Motion1::setAxisHomed(Z_AXIS, true);
}

void PrinterType::homeAxis(fast8_t axis) {
    if (axis < Z_AXIS) {
        return; // Deltas can not home x or y
    } else if (axis > Z_AXIS) {
        Motion1::simpleHome(axis); // Non delta axis, default homing
    } else {                       // XYZ homing
        homeZ();
    }
}

/** Check if given position pos is an allowed position. Coordinate system here
 * is the transformed coordinates. In addition zOfficial is the z position in 
 * official coordinates.
 */
bool PrinterType::positionAllowed(float pos[NUM_AXES], float zOfficial) {
    if (Printer::isNoDestinationCheck()) {
        return true;
    }
    if (Printer::isHoming() || Motion1::endstopMode == EndstopMode::PROBING) {
        return true;
    }
    // Extra contraint to protect Z conditionbased on official coordinate system
    if (zOfficial < Motion1::minPos[Z_AXIS] - 0.01 || zOfficial > Motion1::maxPos[Z_AXIS] + 0.01) {
        return false;
    }
    /*if (pos[Z_AXIS] < Motion1::minPosOff[Z_AXIS] || pos[Z_AXIS] > Motion1::maxPosOff[Z_AXIS]) {
        return false;
    } */
    float px = pos[X_AXIS]; // + Motion1::toolOffset[X_AXIS]; // need to consider tool offsets
    float py = pos[Y_AXIS]; // + Motion1::toolOffset[Y_AXIS];
    return px * px + py * py <= printRadiusSquared;
}

void PrinterType::closestAllowedPositionWithNewXYOffset(float pos[NUM_AXES], float offX, float offY, float safety) {
    // offX and offY are with sign as stored in tool not when assigned later!
    // pos is in official coordinate system
    float offsets[2] = { offX, offY };
    float tOffMin, tOffMax;
    float tPos[2], t2Pos[2];
    for (int loop = 0; loop < 3; loop++) { // can need 2 iterations to be valid!
        float dist = 0, dist2 = 0, distbed = 0;
        for (fast8_t i = 0; i < Z_AXIS; i++) {
            tPos[i] = pos[i] - offsets[i];
            t2Pos[i] = pos[i] + Motion1::toolOffset[i];
            dist += tPos[i] * tPos[i];
            dist2 += t2Pos[i] * t2Pos[i]; // current tool offset
            distbed += pos[i] * pos[i];
        }
        if (distbed > dist && distbed > dist2 && (bedRadius - safety) / sqrt(distbed) < 1.001) {
            dist = sqrtf(distbed);
            float fac = (bedRadius - safety) / dist;
            if (fac >= 1.0f) { // inside bed, nothing to do
                return;
            }
            pos[X_AXIS] = tPos[X_AXIS] * fac;
            pos[Y_AXIS] = tPos[Y_AXIS] * fac;
        } else if (dist > dist2 && (printRadius - safety) / sqrt(dist) < 1.001) {
            dist = sqrtf(dist);
            float fac = (printRadius - safety) / dist;
            if (fac >= 1.0f) { // inside bed, nothing to do
                return;
            }
            tPos[X_AXIS] *= fac;
            tPos[Y_AXIS] *= fac;
            pos[X_AXIS] = tPos[X_AXIS] + offX;
            pos[Y_AXIS] = tPos[Y_AXIS] + offY;
        } else {
            dist2 = sqrtf(dist2);
            float fac = (printRadius - safety) / dist2;
            if (fac >= 1.0f) { // inside bed, nothing to do
                return;
            }
            t2Pos[X_AXIS] *= fac;
            t2Pos[Y_AXIS] *= fac;
            pos[X_AXIS] = t2Pos[X_AXIS] - Motion1::toolOffset[X_AXIS];
            pos[Y_AXIS] = t2Pos[Y_AXIS] - Motion1::toolOffset[Y_AXIS];
        }
    }
}

bool PrinterType::positionOnBed(float pos[2]) {
    return pos[X_AXIS] * pos[X_AXIS] + pos[Y_AXIS] * pos[Y_AXIS] <= bedRadius * bedRadius;
}

void PrinterType::getBedRectangle(float& xmin, float& xmax, float& ymin, float& ymax) {
    xmin = -bedRadius;
    xmax = bedRadius;
    ymin = -bedRadius;
    ymax = bedRadius;
}

void PrinterType::M360() {
    Com::config(PSTR("PrinterType:Delta"));
    Com::config(PSTR("BedRadiusXMin:"), bedRadius);
}

void PrinterType::transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]) {
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        motor[i] = lroundf(pos[i] * Motion1::resolution[i]);
    }
    if (mode == MotionMode::MOTION_PER_AXIS) {
        motor[X_AXIS] = lroundf(pos[Z_AXIS] * Motion1::resolution[X_AXIS]);
        motor[Y_AXIS] = lroundf(pos[Y_AXIS] * Motion1::resolution[Y_AXIS]);
        motor[Z_AXIS] = lroundf(pos[Z_AXIS] * Motion1::resolution[Z_AXIS]);
        return;
    }
    // Move them in delta mode
    float z = pos[Z_AXIS];
    float temp = APosY - pos[Y_AXIS];
    float opt = diagonalSquaredA - temp * temp;
    float temp2 = APosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        // TODO: Trigger error
        return;
    }
    motor[X_AXIS] = lroundf((sqrt(temp) + z) * Motion1::resolution[X_AXIS]);

    temp = BPosY - pos[Y_AXIS];
    opt = diagonalSquaredB - temp * temp;
    temp2 = BPosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        return;
    }
    motor[Y_AXIS] = lroundf((sqrt(temp) + z) * Motion1::resolution[Y_AXIS]);

    temp = CPosY - pos[Y_AXIS];
    opt = diagonalSquaredC - temp * temp;
    temp2 = CPosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        return;
    }
    motor[Z_AXIS] = lroundl((sqrt(temp) + z) * Motion1::resolution[Z_AXIS]);
}

void PrinterType::disableAllowedStepper() {
    // Disabling xyz can cause carriage to drop down, therefore we never
    // disable power to these drivers.
}

float PrinterType::accelerationForMoveSteps(fast8_t axes) {
    float acceleration = 500.0f;
    if (axes & 8) {
        FOR_ALL_AXES(i) {
            if (axes & axisBits[i]) {
                acceleration = RMath::min(acceleration, Motion1::maxTravelAcceleration[i]);
            }
        }
    } else {
        FOR_ALL_AXES(i) {
            if (axes & axisBits[i]) {
                acceleration = RMath::min(acceleration, Motion1::maxAcceleration[i]);
            }
        }
    }
    return acceleration;
}

float PrinterType::feedrateForMoveSteps(fast8_t axes) {
    float feedrate = 100.0f;
    FOR_ALL_AXES(i) {
        if (axes & axisBits[i]) {
            feedrate = RMath::min(feedrate, Motion1::maxFeedrate[i]);
        }
    }
    return feedrate;
}

void PrinterType::deactivatedTool(fast8_t id) { }

void PrinterType::activatedTool(fast8_t id) { }

void PrinterType::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Delta"));
    EEPROM::handleFloat(eeprom + 0, PSTR("Diagonal [mm]"), 2, diagonal);
    EEPROM::handleFloat(eeprom + 4, PSTR("Horizontal Radius [mm]"), 3, horizontalRadius);
    EEPROM::handleFloat(eeprom + 8, PSTR("Printable Radius [mm]"), 2, printRadius);
    EEPROM::handleFloat(eeprom + 60, PSTR("Bed Radius [mm]"), 2, bedRadius);
    EEPROM::handleFloat(eeprom + 12, PSTR("Angle A [mm]"), 3, angleA);
    EEPROM::handleFloat(eeprom + 16, PSTR("Angle B [mm]"), 3, angleB);
    EEPROM::handleFloat(eeprom + 20, PSTR("Angle C [mm]"), 3, angleC);
    EEPROM::handleFloat(eeprom + 24, PSTR("Diagonal Correction A [mm]"), 2, correctionA);
    EEPROM::handleFloat(eeprom + 28, PSTR("Diagonal Correction B [mm]"), 2, correctionB);
    EEPROM::handleFloat(eeprom + 32, PSTR("Diagonal Correction C [mm]"), 2, correctionC);
    EEPROM::handleFloat(eeprom + 36, PSTR("Horiz. Radius A [mm]"), 2, radiusCorrectionA);
    EEPROM::handleFloat(eeprom + 40, PSTR("Horiz. Radius B [mm]"), 2, radiusCorrectionB);
    EEPROM::handleFloat(eeprom + 44, PSTR("Horiz. Radius C [mm]"), 2, radiusCorrectionC);
    EEPROM::handleFloat(eeprom + 48, PSTR("Home Offset A [mm]"), 3, homeOffsetA);
    EEPROM::handleFloat(eeprom + 52, PSTR("Home Offset B [mm]"), 3, homeOffsetB);
    EEPROM::handleFloat(eeprom + 56, PSTR("Home Offset C [mm]"), 3, homeOffsetC);
    EEPROM::removePrefix();
}

void PrinterType::restoreFromConfiguration() {
    diagonal = DELTA_DIAGONAL;
    horizontalRadius = DELTA_HORIZONTAL_RADIUS;
    printRadius = DELTA_PRINT_RADIUS;
    angleA = DELTA_ANGLE_A;
    angleB = DELTA_ANGLE_B;
    angleC = DELTA_ANGLE_C;
    correctionA = DELTA_CORRECTION_A;
    correctionB = DELTA_CORRECTION_B;
    correctionC = DELTA_CORRECTION_C;
    radiusCorrectionA = DELTA_RADIUS_CORRECTION_A;
    radiusCorrectionB = DELTA_RADIUS_CORRECTION_B;
    radiusCorrectionC = DELTA_RADIUS_CORRECTION_C;
    homeOffsetA = DELTA_HOME_OFFSET_A;
    homeOffsetB = DELTA_HOME_OFFSET_B;
    homeOffsetC = DELTA_HOME_OFFSET_C;
    bedRadius = BED_RADIUS;
    PrinterType::updateDerived();
}

void PrinterType::init() {
    PrinterType::restoreFromConfiguration();
    eeprom = EEPROM::reserve(EEPROM_SIGNATURE_DELTA, 1, 16 * 4);
}

void PrinterType::updateDerived() {
    float radiusA = horizontalRadius + radiusCorrectionA;
    float radiusB = horizontalRadius + radiusCorrectionB;
    float radiusC = horizontalRadius + radiusCorrectionC;
    APosX = radiusA * cos(degToRad(angleA));
    APosY = radiusA * sin(degToRad(angleA));
    BPosX = radiusB * cos(degToRad(angleB));
    BPosY = radiusB * sin(degToRad(angleB));
    CPosX = radiusC * cos(degToRad(angleC));
    CPosY = radiusC * sin(degToRad(angleC));
    diagonalSquaredA = RMath::sqr(diagonal + correctionA);
    diagonalSquaredB = RMath::sqr(diagonal + correctionB);
    diagonalSquaredC = RMath::sqr(diagonal + correctionC);
    printRadiusSquared = printRadius * printRadius;
    Motion1::minPos[X_AXIS] = Motion1::minPos[Y_AXIS] = -printRadius;
    Motion1::maxPos[X_AXIS] = Motion1::maxPos[Y_AXIS] = printRadius;
    Motion1::moveFeedrate[X_AXIS] = Motion1::moveFeedrate[Y_AXIS] = Motion1::moveFeedrate[Z_AXIS];
    Motion1::maxFeedrate[X_AXIS] = Motion1::maxFeedrate[Y_AXIS] = Motion1::maxFeedrate[Z_AXIS];
    Motion1::maxAcceleration[X_AXIS] = Motion1::maxAcceleration[Y_AXIS] = Motion1::maxAcceleration[Z_AXIS];
    Motion1::maxTravelAcceleration[X_AXIS] = Motion1::maxTravelAcceleration[Y_AXIS] = Motion1::maxTravelAcceleration[Z_AXIS];
    Motion1::homingFeedrate[X_AXIS] = Motion1::homingFeedrate[Y_AXIS] = Motion1::homingFeedrate[Z_AXIS];
    Motion1::maxYank[X_AXIS] = Motion1::maxYank[Y_AXIS] = Motion1::maxYank[Z_AXIS];
    Motion1::updateRotMinMax();
}

void PrinterType::enableMotors(fast8_t axes) {
    if (axes & 7) { // enable x,y,z as a group!
        Motion1::motors[X_AXIS]->enable();
        Motion1::motors[Y_AXIS]->enable();
        Motion1::motors[Z_AXIS]->enable();
    }
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        if ((axes & axisBits[i]) != 0 && Motion1::motors[i]) {
            Motion1::motors[i]->enable();
        }
    }
    if ((axes & axisBits[E_AXIS]) != 0 && Motion1::dittoMode) {
        for (fast8_t i = 1; i <= Motion1::dittoMode; i++) {
            Tool::getTool(i)->enableMotor();
        }
    }
    Printer::unsetAllSteppersDisabled();
}

void PrinterType::setDittoMode(fast8_t count, bool mirror) {
    Motion1::dittoMode = count;
    Motion1::dittoMirror = mirror;
}

void PrinterType::transformedToOfficial(float trans[NUM_AXES], float official[NUM_AXES]) {
    Motion1::transformFromPrinter(
        trans[X_AXIS],
        trans[Y_AXIS],
        trans[Z_AXIS],
        official[X_AXIS],
        official[Y_AXIS],
        official[Z_AXIS]);
    official[X_AXIS] -= Motion1::toolOffset[X_AXIS]; // Offset from active extruder or z probe
    official[Y_AXIS] -= Motion1::toolOffset[Y_AXIS];
    official[Z_AXIS] -= Motion1::toolOffset[Z_AXIS];
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        official[i] = trans[i];
    }
}

void PrinterType::officialToTransformed(float official[NUM_AXES], float trans[NUM_AXES]) {
    Motion1::transformToPrinter(official[X_AXIS] + Motion1::toolOffset[X_AXIS],
                                official[Y_AXIS] + Motion1::toolOffset[Y_AXIS],
                                official[Z_AXIS] + Motion1::toolOffset[Z_AXIS],
                                trans[X_AXIS],
                                trans[Y_AXIS],
                                trans[Z_AXIS]);
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        trans[i] = official[i];
    }
}

bool PrinterType::canSelectTool(fast8_t toolId) {
    return true;
}

bool PrinterType::runMCode(GCode* com) {
    switch (com->M) {
    case 290:
        M290(com);
        return false;
    case 360:
        M360();
        return false;
    case 665: {
        if (com->hasL()) {
            diagonal = fabsf(com->L);
        }
        if (com->hasR()) {
            horizontalRadius = fabsf(com->R);
        }
        if (com->hasA()) {
            correctionA = com->A;
        }
        if (com->hasB()) {
            correctionB = com->B;
        }
        if (com->hasC()) {
            correctionC = com->C;
        }
        if (com->hasX()) {
            angleA = DELTA_ANGLE_A + com->X;
        }
        if (com->hasY()) {
            angleB = DELTA_ANGLE_A + com->Y;
        }
        if (com->hasZ()) {
            angleC = DELTA_ANGLE_A + com->Z;
        }
        EEPROM::markChanged();
    } break;
    case 666: {
        if (com->hasX()) {
            homeOffsetA = fabsf(com->X);
        }
        if (com->hasY()) {
            homeOffsetB = fabsf(com->Y);
        }
        if (com->hasZ()) {
            homeOffsetC = fabsf(com->Z);
        }
        EEPROM::markChanged();
    } break;
    }
    return true;
}

bool PrinterType::runGCode(GCode* com) {
    switch (com->G) {
    case 320: // calibrate delta
    {
        int factors = 6;
        bool normalise = true;
        if (com->hasP()) {
            factors = com->P;
        }
        if (com->hasU()) {
            normalise = com->U != 0;
        }
        DeltaCalibrator::runDeltaCalibration(factors, normalise);
        return false;
    }
    }
    return false;
}

void PrinterType::M290(GCode* com) {
    InterruptProtectedBlock lock;
    if (com->hasZ()) {
        float z = constrain(com->Z, -2.0f, 2.0f);
        Motion1::totalBabystepZ += z;
        Motion2::openBabysteps[X_AXIS] += z * Motion1::resolution[X_AXIS];
        Motion2::openBabysteps[Y_AXIS] += z * Motion1::resolution[Y_AXIS];
        Motion2::openBabysteps[Z_AXIS] += z * Motion1::resolution[Z_AXIS];
    }
    lock.unprotect();
    Com::printFLN(PSTR("BabystepZ:"), Motion1::totalBabystepZ, 4);
}

PGM_P PrinterType::getGeometryName() {
    return PSTR("Delta");
}

#endif
