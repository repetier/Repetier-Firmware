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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm
  firmware.

  Functions in this file are used to communicate using ascii or repetier
  protocol.
*/

#include "Repetier.h"

#if DISTORTION_CORRECTION

Distortion Printer::distortion;

void Printer::measureDistortion(void) {
    prepareForProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
    float actTemp[NUM_EXTRUDER];
    for (int i = 0; i < NUM_EXTRUDER; i++)
        actTemp[i] = extruder[i].tempControl.targetTemperatureC;
    Printer::moveToReal(
        IGNORE_COORDINATE, IGNORE_COORDINATE,
        RMath::max(EEPROM::zProbeHeight(), static_cast<float>(ZHOME_HEAT_HEIGHT)),
        IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Extruder::setTemperatureForExtruder(
            RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i,
            false, false);
    }
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(
                RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i,
                false, true);
    }
#else
    if (extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
        Extruder::setTemperatureForExtruder(
            RMath::max(actTemp[Extruder::current->id],
                       static_cast<float>(ZPROBE_MIN_TEMPERATURE)),
            Extruder::current->id, false, true);
#endif
#endif
    float oldFeedrate = Printer::feedrate;

    Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;

    if (!distortion.measure()) {
        GCode::fatalError(PSTR("G33 failed!"));
        return;
    }
    Printer::feedrate = oldFeedrate;
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
    for (int i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setTemperatureForExtruder(actTemp[i], i, false,
                                            actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
    Extruder::setTemperatureForExtruder(
        actTemp[Extruder::current->id], Extruder::current->id, false,
        actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
}

Distortion::Distortion() {}

void Distortion::init() {
    updateDerived();
#if !DISTORTION_PERMANENT
    resetCorrection();
#endif
#if EEPROM_MODE != 0
    enabled = EEPROM::isZCorrectionEnabled();
    Com::printFLN(PSTR("zDistortionCorrection:"), (int)enabled);
#else
    enabled = false;
#endif
}

void Distortion::updateDerived() {
#if DRIVE_SYSTEM == DELTA
    step = (2 * Printer::axisStepsPerMM[Z_AXIS] * DISTORTION_CORRECTION_R) / (DISTORTION_CORRECTION_POINTS - 1.0f);
    radiusCorrectionSteps = DISTORTION_CORRECTION_R * Printer::axisStepsPerMM[Z_AXIS];
#else
    xCorrectionSteps = (DISTORTION_XMAX - DISTORTION_XMIN) * Printer::axisStepsPerMM[X_AXIS] / (DISTORTION_CORRECTION_POINTS - 1);
    xOffsetSteps = DISTORTION_XMIN * Printer::axisStepsPerMM[X_AXIS];
    yCorrectionSteps = (DISTORTION_YMAX - DISTORTION_YMIN) * Printer::axisStepsPerMM[Y_AXIS] / (DISTORTION_CORRECTION_POINTS - 1);
    yOffsetSteps = DISTORTION_YMIN * Printer::axisStepsPerMM[Y_AXIS];

#endif
    zStart = DISTORTION_START_DEGRADE * Printer::axisStepsPerMM[Z_AXIS] + Printer::zMinSteps;
    zEnd = DISTORTION_END_HEIGHT * Printer::axisStepsPerMM[Z_AXIS] + Printer::zMinSteps;
}

void Distortion::enable(bool permanent, bool silent) {
    enabled = true;
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
    if (permanent)
        EEPROM::setZCorrectionEnabled(enabled);
#endif
    if (!silent) {
        Com::printFLN(Com::tZCorrectionEnabled);
    }
    // Problem is now we do not include the extra steps required
    Printer::updateCurrentPosition(false);
}

void Distortion::disable(bool permanent, bool silent) {
    enabled = false;
#if DISTORTION_PERMANENT && EEPROM_MODE != 0
    if (permanent)
        EEPROM::setZCorrectionEnabled(enabled);
#endif
#if DRIVE_SYSTEM != DELTA
    Printer::zCorrectionStepsIncluded = 0;
#endif
    Printer::updateCurrentPosition(
        false); // now we have a different z height ignoring extra steps included
    if (!silent) {
        Com::printFLN(Com::tZCorrectionDisabled);
    }
}

void Distortion::reportStatus() {
    Com::printFLN(enabled ? Com::tZCorrectionEnabled : Com::tZCorrectionDisabled);
}

void Distortion::resetCorrection(void) {
    Com::printInfoFLN(PSTR("Resetting Z correction"));
    for (int i = 0;
         i < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS; i++)
        setMatrix(0, i);
}

int Distortion::matrixIndex(fast8_t x, fast8_t y) const {
    return static_cast<int>(y) * DISTORTION_CORRECTION_POINTS + x;
}

int32_t Distortion::getMatrix(int index) const {
#if DISTORTION_PERMANENT
    return EEPROM::getZCorrection(index);
#else
    return matrix[index];
#endif
}
void Distortion::setMatrix(int32_t val, int index) {
#if DISTORTION_PERMANENT
#if EEPROM_MODE != 0
    EEPROM::setZCorrection(val, index);
#endif
#else
    matrix[index] = val;
#endif
}

bool Distortion::isCorner(fast8_t i, fast8_t j) const {
    return (i == 0 || i == DISTORTION_CORRECTION_POINTS - 1) && (j == 0 || j == DISTORTION_CORRECTION_POINTS - 1);
}

/**
 Extrapolates the changes from p1 to p2 to p3 which has the same distance as
 p1-p2.
*/
inline int32_t Distortion::extrapolatePoint(fast8_t x1, fast8_t y1, fast8_t x2,
                                            fast8_t y2) const {
    return 2 * getMatrix(matrixIndex(x2, y2)) - getMatrix(matrixIndex(x1, y1));
}

void Distortion::extrapolateCorner(fast8_t x, fast8_t y, fast8_t dx,
                                   fast8_t dy) {
    setMatrix((extrapolatePoint(x + 2 * dx, y, x + dx, y) + extrapolatePoint(x, y + 2 * dy, x, y + dy)) / 2.0,
              matrixIndex(x, y));
}

void Distortion::extrapolateCorners() {
    const fast8_t m = DISTORTION_CORRECTION_POINTS - 1;
    extrapolateCorner(0, 0, 1, 1);
    extrapolateCorner(0, m, 1, -1);
    extrapolateCorner(m, 0, -1, 1);
    extrapolateCorner(m, m, -1, -1);
}

bool Distortion::measure(void) {
    fast8_t ix, iy;

    disable(false);
    Printer::prepareForProbing();
    float z = RMath::max(
        EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0),
        static_cast<float>(ZHOME_HEAT_HEIGHT)); // EEPROM::zProbeBedDistance() +
                                                // (EEPROM::zProbeHeight() > 0 ?
                                                // EEPROM::zProbeHeight() : 0);
    Com::printFLN(PSTR("Reference Z for measurement:"), z, 3);
    updateDerived();
    /*
  #if DRIVE_SYSTEM == DELTA
      // It is not possible to go to the edges at the top, also users try
      // it often and wonder why the coordinate system is then wrong.
      // For that reason we ensure a correct behavior by code.
      Printer::homeAxis(true, true, true);
      Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE,
  EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ?
  EEPROM::zProbeHeight() : 0), IGNORE_COORDINATE,
  Printer::homingFeedrate[Z_AXIS]); #else if(!Printer::isXHomed() ||
  !Printer::isYHomed()) Printer::homeAxis(true, true, false);
      Printer::updateCurrentPosition(true);
      Printer::moveTo(Printer::invAxisStepsPerMM[X_AXIS] * ((isCorner(0, 0) ? 1
  : 0) * xCorrectionSteps + xOffsetSteps), Printer::invAxisStepsPerMM[Y_AXIS] *
  ((DISTORTION_CORRECTION_POINTS - 1) * yCorrectionSteps + yOffsetSteps),
  IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed()); #endif
  */
    // Com::printFLN(PSTR("radiusCorr:"), radiusCorrectionSteps);
    // Com::printFLN(PSTR("steps:"), step);
    int32_t zCorrection = 0;
#if Z_PROBE_Z_OFFSET_MODE == 1
    zCorrection -= Printer::zBedOffset * Printer::axisStepsPerMM[Z_AXIS];
#endif

    if (!Printer::startProbing(true)) {
        return false;
    }
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, z,
                        IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    for (iy = 0; iy < DISTORTION_CORRECTION_POINTS; iy++)
        for (fast8_t iix = 0; iix < DISTORTION_CORRECTION_POINTS; iix++) {
            ix = iy & 1 ? DISTORTION_CORRECTION_POINTS - 1 - iix : iix;
#if (DRIVE_SYSTEM == DELTA) && DISTORTION_EXTRAPOLATE_CORNERS
            if (isCorner(ix, iy))
                continue;
#endif
#if DRIVE_SYSTEM == DELTA
            float mtx = Printer::invAxisStepsPerMM[X_AXIS] * (ix * step - radiusCorrectionSteps);
            float mty = Printer::invAxisStepsPerMM[Y_AXIS] * (iy * step - radiusCorrectionSteps);
#else
            float mtx = Printer::invAxisStepsPerMM[X_AXIS] * (ix * xCorrectionSteps + xOffsetSteps);
            float mty = Printer::invAxisStepsPerMM[Y_AXIS] * (iy * yCorrectionSteps + yOffsetSteps);
#endif
            // Com::printF(PSTR("mx "),mtx);
            // Com::printF(PSTR("my "),mty);
            // Com::printF(PSTR("ix "),(int)ix);
            // Com::printFLN(PSTR("iy "),(int)iy);
            Printer::moveToReal(mtx, mty, z, IGNORE_COORDINATE,
                                EEPROM::zProbeXYSpeed());
            float zp = Printer::runZProbe(false, false, Z_PROBE_REPETITIONS);
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
            if (zp == ILLEGAL_Z_PROBE || fabs(z - zp + zCorrection * Printer::invAxisStepsPerMM[Z_AXIS]) > DISTORTION_LIMIT_TO) {
#else
            if (zp == ILLEGAL_Z_PROBE) {
#endif
                Com::printErrorFLN(
                    PSTR("Stopping distortion measurement due to errors."));
                Printer::finishProbing();
                return false;
            }
            setMatrix(floor(0.5f + Printer::axisStepsPerMM[Z_AXIS] * (z - zp)) + zCorrection,
                      matrixIndex(ix, iy));
        }
    Printer::finishProbing();
#if (DRIVE_SYSTEM == DELTA) && DISTORTION_EXTRAPOLATE_CORNERS
    extrapolateCorners();
#endif
    // make average center
    // Disabled since we can use grid measurement to get average plane if that is
    // what we want. Shifting z with each measuring is a pain and can result in
    // unexpected behavior.
    /*
  float sum = 0;
  for(int k = 0;k < DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS;
  k++) sum += getMatrix(k); sum /=
  static_cast<float>(DISTORTION_CORRECTION_POINTS *
  DISTORTION_CORRECTION_POINTS); for(int k = 0;k < DISTORTION_CORRECTION_POINTS
  * DISTORTION_CORRECTION_POINTS; k++) setMatrix(getMatrix(k) - sum, k);
  Printer::zLength -= sum * Printer::invAxisStepsPerMM[Z_AXIS];
  */
#if EEPROM_MODE
    EEPROM::storeDataIntoEEPROM();
#endif
    // print matrix
    Com::printInfoFLN(PSTR("Distortion correction matrix:"));
    for (iy = DISTORTION_CORRECTION_POINTS - 1; iy >= 0; iy--) {
        for (ix = 0; ix < DISTORTION_CORRECTION_POINTS; ix++)
            Com::printF(ix ? PSTR(", ") : PSTR(""), getMatrix(matrixIndex(ix, iy)));
        Com::println();
    }
    showMatrix();
    enable(true);
    return true;
    // Printer::homeAxis(false, false, true);
}

int32_t Distortion::correct(int32_t x, int32_t y, int32_t z) const {
    if (!enabled || Printer::isZProbingActive()) {
        return 0;
    }
    z += Printer::offsetZ * Printer::axisStepsPerMM[Z_AXIS] - Printer::zMinSteps;
    if (z > zEnd) {
        /* Com::printF(PSTR("NoCor z:"),z);
     Com::printF(PSTR(" zEnd:"),zEnd);
     Com::printF(PSTR(" en:"),(int)enabled);
     Com::printFLN(PSTR(" zp:"),(int)Printer::isZProbingActive());*/
        return 0;
    }
    x -= Printer::offsetX * Printer::axisStepsPerMM[X_AXIS]; // correct active tool offset
    y -= Printer::offsetY * Printer::axisStepsPerMM[Y_AXIS];
    if (false) {
        Com::printF(PSTR("correcting ("), x);
        Com::printF(PSTR(","), y);
    }
#if DRIVE_SYSTEM == DELTA
    x += radiusCorrectionSteps;
    y += radiusCorrectionSteps;
    int32_t fxFloor = (x - (x < 0 ? step - 1 : 0)) / step; // special case floor for negative integers!
    int32_t fyFloor = (y - (y < 0 ? step - 1 : 0)) / step;
#else
    x -= xOffsetSteps;
    y -= yOffsetSteps;
    int32_t fxFloor = (x - (x < 0 ? xCorrectionSteps - 1 : 0)) / xCorrectionSteps; // special case floor for negative integers!
    int32_t fyFloor = (y - (y < 0 ? yCorrectionSteps - 1 : 0)) / yCorrectionSteps;
#endif
// indexes to the matrix

// position between cells of matrix, range=0 to 1 - outside of the matrix the
// value will be outside this range and the value will be extrapolated
#if DRIVE_SYSTEM == DELTA
    int32_t fx = x - fxFloor * step; // Grid normalized coordinates
    int32_t fy = y - fyFloor * step;
    if (fxFloor < 0) {
        fxFloor = 0;
        fx = 0;
    } else if (fxFloor >= DISTORTION_CORRECTION_POINTS - 1) {
        fxFloor = DISTORTION_CORRECTION_POINTS - 2;
        fx = step;
    }
    if (fyFloor < 0) {
        fyFloor = 0;
        fy = 0;
    } else if (fyFloor >= DISTORTION_CORRECTION_POINTS - 1) {
        fyFloor = DISTORTION_CORRECTION_POINTS - 2;
        fy = step;
    }

    int32_t idx11 = matrixIndex(fxFloor, fyFloor);
    int32_t m11 = getMatrix(idx11), m12 = getMatrix(idx11 + 1);
    int32_t m21 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS);
    int32_t m22 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS + 1);
    int32_t zx1 = m11 + ((m12 - m11) * fx) / step;
    int32_t zx2 = m21 + ((m22 - m21) * fx) / step;
    int32_t correction_z = zx1 + ((zx2 - zx1) * fy) / step;
    /*if(z == Printer::zMinSteps) {
  Com::printF(PSTR("DT M11:"),m11);
  Com::printF(PSTR(" M12:"),m12);
  Com::printF(PSTR(" M21:"),m21);
  Com::printF(PSTR(" M22:"),m22);
  Com::printF(PSTR(" FX:"),fx);
  Com::printF(PSTR(" FY:"),fy);
  Com::printF(PSTR(" FFX:"),fxFloor);
  Com::printF(PSTR(" FFY:"),fyFloor);
  Com::printF(PSTR(" XP:"),x-radiusCorrectionSteps);
  Com::printF(PSTR(" Yp:"),y-radiusCorrectionSteps);
  Com::printF(PSTR(" STEP:"),step);
  Com::printFLN(PSTR(" ZCOR:"),correction_z);
  }*/
#else
    int32_t fx = x - fxFloor * xCorrectionSteps; // Grid normalized coordinates
    int32_t fy = y - fyFloor * yCorrectionSteps;
    if (fxFloor < 0) {
        fxFloor = 0;
        fx = 0;
    } else if (fxFloor >= DISTORTION_CORRECTION_POINTS - 1) {
        fxFloor = DISTORTION_CORRECTION_POINTS - 2;
        fx = xCorrectionSteps;
    }
    if (fyFloor < 0) {
        fyFloor = 0;
        fy = 0;
    } else if (fyFloor >= DISTORTION_CORRECTION_POINTS - 1) {
        fyFloor = DISTORTION_CORRECTION_POINTS - 2;
        fy = yCorrectionSteps;
    }

    int32_t idx11 = matrixIndex(fxFloor, fyFloor);
    int32_t m11 = getMatrix(idx11), m12 = getMatrix(idx11 + 1);
    int32_t m21 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS);
    int32_t m22 = getMatrix(idx11 + DISTORTION_CORRECTION_POINTS + 1);
    int32_t zx1 = m11 + ((m12 - m11) * fx) / xCorrectionSteps;
    int32_t zx2 = m21 + ((m22 - m21) * fx) / xCorrectionSteps;
    int32_t correction_z = zx1 + ((zx2 - zx1) * fy) / yCorrectionSteps;
#endif
    /* if(false) {
     Com::printF(PSTR(") by "), correction_z);
     Com::printF(PSTR(" ix= "), fxFloor); Com::printF(PSTR(" fx= "),
   (float)fx/(float)xCorrectionSteps,3); Com::printF(PSTR(" iy= "), fyFloor);
   Com::printFLN(PSTR(" fy= "), (float)fy/(float)yCorrectionSteps,3);
   }*/
    if (z > zStart && z > Printer::zMinSteps)
        // All variables are type int. For calculation we need float values
        correction_z = (correction_z * static_cast<float>(zEnd - z) / (zEnd - zStart));
    /* if(correction_z > 20 || correction_z < -20) {
           Com::printFLN(PSTR("Corr. error too big:"),correction_z);
       Com::printF(PSTR("fxf"),(int)fxFloor);
       Com::printF(PSTR(" fyf"),(int)fyFloor);
       Com::printF(PSTR(" fx"),fx);
       Com::printF(PSTR(" fy"),fy);
       Com::printF(PSTR(" x"),x);
       Com::printFLN(PSTR(" y"),y);
       Com::printF(PSTR(" m11:"),m11);
       Com::printF(PSTR(" m12:"),m12);
       Com::printF(PSTR(" m21:"),m21);
       Com::printF(PSTR(" m22:"),m22);
       Com::printFLN(PSTR(" step:"),step);
       correction_z = 0;
   }*/
    return correction_z;
}

void Distortion::set(float x, float y, float z) {
#if defined(DISTORTION_LIMIT_TO) && DISTORTION_LIMIT_TO != 0
    if (fabs(z) > DISTORTION_LIMIT_TO) {
        Com::printWarningFLN(
            PSTR("Max. distortion value exceeded - not setting this value."));
        return;
    }
#endif
#if DRIVE_SYSTEM == DELTA
    int ix = (x * Printer::axisStepsPerMM[Z_AXIS] + radiusCorrectionSteps + step / 2) / step;
    int iy = (y * Printer::axisStepsPerMM[Z_AXIS] + radiusCorrectionSteps + step / 2) / step;
#else
    int ix = (x * Printer::axisStepsPerMM[X_AXIS] - xOffsetSteps + xCorrectionSteps / 2) / xCorrectionSteps;
    int iy = (y * Printer::axisStepsPerMM[Y_AXIS] - yOffsetSteps + yCorrectionSteps / 2) / yCorrectionSteps;
#endif
    if (ix < 0)
        ix = 0;
    if (iy < 0)
        iy = 0;
    if (ix >= DISTORTION_CORRECTION_POINTS - 1)
        ix = DISTORTION_CORRECTION_POINTS - 1;
    if (iy >= DISTORTION_CORRECTION_POINTS - 1)
        iy = DISTORTION_CORRECTION_POINTS - 1;
    int32_t idx = matrixIndex(ix, iy);
    setMatrix(z * Printer::axisStepsPerMM[Z_AXIS], idx);
}

void Distortion::showMatrix() {
    for (int ix = 0; ix < DISTORTION_CORRECTION_POINTS; ix++) {
        for (int iy = 0; iy < DISTORTION_CORRECTION_POINTS; iy++) {
#if DRIVE_SYSTEM == DELTA
            float x = (-radiusCorrectionSteps + ix * step) * Printer::invAxisStepsPerMM[Z_AXIS];
            float y = (-radiusCorrectionSteps + iy * step) * Printer::invAxisStepsPerMM[Z_AXIS];
#else
            float x = (xOffsetSteps + ix * xCorrectionSteps) * Printer::invAxisStepsPerMM[X_AXIS];
            float y = (yOffsetSteps + iy * yCorrectionSteps) * Printer::invAxisStepsPerMM[Y_AXIS];
#endif
            int32_t idx = matrixIndex(ix, iy);
            float z = getMatrix(idx) * Printer::invAxisStepsPerMM[Z_AXIS];
            Com::printF(PSTR("G33 X"), x, 2);
            Com::printF(PSTR(" Y"), y, 2);
            Com::printFLN(PSTR(" Z"), z, 3);
        }
    }
}

#endif // DISTORTION_CORRECTION
