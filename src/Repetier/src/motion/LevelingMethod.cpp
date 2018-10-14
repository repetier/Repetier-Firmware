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
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

#if LEVELING_CORRECTOR == 0 // software correction

void LevelingCorrector::correct(Plane* plane) {
    Motion1::buildTransformationMatrix(*plane);
    Motion1::setAutolevelActive(true);
    EEPROM::markChanged();
}

#elif LEVELING_CORRECTOR == 1 // Motorized correction

float LevelingCorrector::points[3][2] = {
    { LC_P1_X, LC_P1_Y },
    { LC_P2_X, LC_P2_Y },
    { LC_P3_X, LC_P3_Y }
};
void LevelingCorrector::init() {}
void LevelingCorrector::handleEeprom() {}
void LevelingCorrector::resetEeprom() {}
void LevelingCorrector::correct(Plane* plane) {}

#endif

#if LEVELING_METHOD == 1 // Grid

float Leveling::grid[GRID_SIZE][GRID_SIZE];
uint16_t Leveling::eprStart;
float Levling::xMin, Leveling::xMax, Leveling::yMin, Leveling::yMax;
uint8_t Leveling::distortionEnabled;

void Leveling::init() {
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_GRID_LEVELING, 1, 4 * GRID_SIZE * GRID_SIZE + 17);
    resetEeprom();
}

void Leveling::handleEeprom() {
    EEPROM::setSilent(true);
    EEPROM::handleFloat(eprStart + 0, Com::empty, 2, xMin);
    EEPROM::handleFloat(eprStart + 0, Com::empty, 2, xMax);
    EEPROM::handleFloat(eprStart + 0, Com::empty, 2, yMin);
    EEPROM::handleFloat(eprStart + 0, Com::empty, 2, yMax);
    EEPROM::handleFloat(eprStart + 0, Com::empty, distortionEnabled);
    for (fast8_t i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
        EEPROM::handleFloat(eprStart + 17 + 4 * i, Com::empty, 2, grid[i]);
    }
    EEPROM::setSilent(false);
    updateDerived();
}

void Leveling::resetEeprom() {
    for (fast8_t i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
        grid[i] = 0;
    }
    distortionEnabled = false;
    xMin = yMin = xMax = yMax = 0;
    updateDerived();
}

void Leveling::updateDerived() {
    dx = (xMax - xMin) / (GRID_SIZE - 1);
    dy = (yMax - yMin) / (GRID_SIZE - 1);
}

void Leveling::measure() {
    float pos[NUM_AXES];
    Plane plane;
    PlaneBuilder builder;
    builder.reset();
    PrinterType::getBedRectangle(xMin, xMax, yMin, yMax);
    Motion1::setAutolevelActive(false);
    Motion1::homeAxes(7); // Home x, y and z
    float px = xMin, py = yMin;
    Motion1::setTmpPositionXYZ(px, py, ZProbeHandler::optimumProbingHeight());
    PrinterType::closestAllowedPositionWithNewXYOffset(Motion1::tmpPosition, ZProbe->xOffset(), ZProbe->yOffset(), ZPROBE_BORDER);
    Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
    ZProbeHandler::activate();
    Motion1::copyCurrentPrinter(pos);
    bool ok = true;
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (!ok) {
                break;
            }
            int xx;
            if (y & 1) {
                px = xMax - x * dx;
                xx = GRID_SIZE - x - 1;
            } else {
                px = xMin + x * dx;
                xx = x;
            }
            py = yMin + y * dy;
            pos[X_AXIS] = px + ZProbe->xOffset();
            pos[Y_AXIS] = py + ZProbe->yOffset();
            pos[Z_AXIS] = ZProbeHandler::optimumProbingHeight();
            if (PrinterType::positionAllowed(pos)) {
                if (ok) {
                    Motion1::moveByPrinter(pos, XY_SPEED, false);
                    float h = ZProbeHandler::runProbe();
                    ok &= h != ILLEGAL_Z_PROBE;
                    grid[xx][y] = h;
                    if (ok) {
                        builder.addPoint(px, py, h);
                    }
                }
            } else {
                grid[xx][y] = ILLEGAL_Z_PROBE;
            }
        }
    }
    if (ok) {
        LevelingCorrector::correct(&plane);
    } else {
        GCode::fatalError(PSTR("Leveling failed!"));
        resetEeprom();
    }
    Motion1::printCurrentPosition();
}

void Leveling::execute_G32(GCode* com) {
    measure();
}

#endif

#if LEVELING_METHOD == 2 // 4 points

void Leveling::measure() {
    Plane plane;
    PlaneBuilder builder;
    builder.reset();
    float h1, h2, h3, h4;
    const float apx = L_P1_X - L_P2_X;
    const float apy = L_P1_Y - L_P2_Y;
    const float abx = L_P3_X - L_P2_X;
    const float aby = L_P3_Y - L_P2_Y;
    const float ab2 = abx * abx + aby * aby;
    const float abap = apx * abx + apy * aby;
    const float t = abap / ab2;
    const float xx = L_P2_X + t * abx;
    const float xy = L_P2_Y + t * aby;
    float x1Mirror = L_P1_X + 2.0 * (xx - L_P1_X);
    float y1Mirror = L_P1_Y + 2.0 * (xy - L_P1_Y);
    Motion1::setAutolevelActive(false);
    Motion1::homeAxes(7); // Home x, y and z
    Motion1::setTmpPositionXYZ(L_P1_X, L_P1_Y, ZProbeHandler::optimumProbingHeight());
    Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
    ZProbeHandler::activate();
    bool ok = true;
    h1 = ZProbeHandler::runProbe();
    ok &= h1 != ILLEGAL_Z_PROBE;
    if (ok) {
        Motion1::setTmpPositionXYZ(L_P2_X, L_P2_Y, ZProbeHandler::optimumProbingHeight());
        Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
        h2 = ZProbeHandler::runProbe();
        ok &= h2 != ILLEGAL_Z_PROBE;
    }
    if (ok) {
        Motion1::setTmpPositionXYZ(L_P3_X, L_P3_Y, ZProbeHandler::optimumProbingHeight());
        Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
        h3 = ZProbeHandler::runProbe();
        ok &= h3 != ILLEGAL_Z_PROBE;
    }
    if (ok) {
        Motion1::setTmpPositionXYZ(x1Mirror, y1Mirror, ZProbeHandler::optimumProbingHeight());
        Motion1::moveByOfficial(Motion1::tmpPosition, XY_SPEED, false);
        h4 = ZProbeHandler::runProbe();
        ok &= h4 != ILLEGAL_Z_PROBE;
    }
    if (ok) {
        const float t2 = h2 + (h3 - h2) * t; // theoretical height for crossing point for symmetric axis
        h1 = t2 - (h4 - h1) * 0.5;           // remove bending part
        builder.addPoint(L_P1_X, L_P1_Y, h1);
        builder.addPoint(L_P2_X, L_P2_Y, h2);
        builder.addPoint(L_P3_X, L_P3_Y, h3);
        builder.createPlane(plane, false);
        LevelingCorrector::correct(&plane);
    } else {
        GCode::fatalError(PSTR("Leveling failed!"));
    }
    Motion1::printCurrentPosition();
}

void Leveling::execute_G32(GCode* com) {
    measure();
}

#endif