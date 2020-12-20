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

#if LEVELING_CORRECTOR == LEVELING_CORRECTOR_SOFTWARE // software correction

void LevelingCorrector::correct(Plane* plane) {
    // adjust current z
    Motion1::currentPositionTransformed[Z_AXIS] = plane->z(Motion1::currentPositionTransformed[X_AXIS], Motion1::currentPositionTransformed[Y_AXIS]);
    Motion1::updatePositionsFromCurrentTransformed();
    // enable rotation
#if LEVELING_METHOD > 0
    Motion1::buildTransformationMatrix(*plane);
    Motion1::setAutolevelActive(true);
#endif
    EEPROM::markChanged();
}

#elif LEVELING_CORRECTOR == LEVELING_CORRECTOR_MOTOR // Motorized correction, 3 motors

void LevelingCorrector::init() { }
void LevelingCorrector::handleEeprom() { }
void LevelingCorrector::resetEeprom() { }
void LevelingCorrector::correct(Plane* plane) {
    float h1 = plane->z(LC_P1_X, LC_P1_Y);
    float h2 = plane->z(LC_P2_X, LC_P2_Y);
    float h3 = plane->z(LC_P3_X, LC_P3_Y);
    h2 -= h1;
    h3 -= h1;
#if defined(LIMIT_MOTORIZED_CORRECTION)
    if (h2 < -LIMIT_MOTORIZED_CORRECTION)
        h2 = -LIMIT_MOTORIZED_CORRECTION;
    if (h2 > LIMIT_MOTORIZED_CORRECTION)
        h2 = LIMIT_MOTORIZED_CORRECTION;
    if (h3 < -LIMIT_MOTORIZED_CORRECTION)
        h3 = -LIMIT_MOTORIZED_CORRECTION;
    if (h3 > LIMIT_MOTORIZED_CORRECTION)
        h3 = LIMIT_MOTORIZED_CORRECTION;
#endif
    Com::printFLN(PSTR("Correction P2:"), h2, 2);
    Com::printFLN(PSTR("Correction P3:"), h3, 2);
#if LC_WAIT_BED_REMOVE > 0
    Com::printFLN(PSTR("Remove bed for precise correction!"));
    float extraZ = 0.5 * (Motion1::maxPos[Z_AXIS] - Motion1::minPos[Z_AXIS]);
    Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, extraZ);
    Motion1::moveRelativeByPrinter(Motion1::tmpPosition, Z_SPEED, false);
    Motion1::waitForEndOfMoves();
    h1 += extraZ;
#if NUM_HEATED_BEDS > 0
    while (heatedBeds[0]->isUnplugged() == false) {
        Commands::checkForPeriodicalActions(false);
        GCode::keepAlive(FirmwareState::Processing, 4);
    }
#endif
    for (int i = 0; i < LC_WAIT_BED_REMOVE * 10; i++) {
        Commands::checkForPeriodicalActions(false);
        HAL::delayMilliseconds(100);
        GCode::keepAlive(FirmwareState::Processing, 4);
    }
#endif
    StepperDriverBase* oldE = Motion1::motors[E_AXIS];
    float eRes = 0, ePos = 0, eAccel = 0, eJerk = 0, advance;
    bool isDry = Printer::debugDryrun();
    bool cold = Printer::isColdExtrusionAllowed();
    fast8_t ditto = Motion1::dittoMode;
    Motion1::dittoMode = 0;
    Printer::setColdExtrusionAllowed(true);
    Printer::debugReset(8);
    if (oldE != nullptr) {
        eRes = Motion1::resolution[E_AXIS];
        ePos = Motion1::currentPosition[E_AXIS];
        eAccel = Motion1::maxAcceleration[E_AXIS];
        eJerk = Motion1::maxYank[E_AXIS];
        advance = Motion1::advanceK;
    }
    Motion1::setMotorForAxis(&LC_P2_MOTOR, E_AXIS);
    Motion1::resolution[E_AXIS] = LC_STEPS_PER_MM;
    Motion1::advanceK = 0;
    Motion1::currentPosition[E_AXIS] = 0;
    Motion1::maxYank[E_AXIS] = 0.1;
    Motion1::maxAcceleration[E_AXIS] = 20;
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, h2);
    Motion1::moveByOfficial(Motion1::tmpPosition, LC_Z_SPEED, false);
    Motion1::waitForEndOfMoves();
    LC_P2_MOTOR.disable();

    Motion1::setMotorForAxis(&LC_P3_MOTOR, E_AXIS);
    Motion1::currentPosition[E_AXIS] = 0;
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();
    Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, h3);
    Motion1::moveByOfficial(Motion1::tmpPosition, LC_Z_SPEED, false);
    Motion1::waitForEndOfMoves();
    LC_P3_MOTOR.disable();

    // Bed is even now, restore extruder motor
    if (oldE != nullptr) {
        Motion1::resolution[E_AXIS] = eRes;
        Motion1::currentPosition[E_AXIS] = ePos;
        Motion1::maxAcceleration[E_AXIS] = eAccel;
        Motion1::maxYank[E_AXIS] = eJerk;
        Motion1::advanceK = advance;
    }
    Motion1::setMotorForAxis(oldE, E_AXIS);
    Motion1::updatePositionsFromCurrent();
    Motion2::setMotorPositionFromTransformed();

    Motion1::resetTransformationMatrix(true);
    EEPROM::markChanged();
    Motion1::currentPositionTransformed[Z_AXIS] = h1;
    Motion1::updatePositionsFromCurrentTransformed();
    Printer::setColdExtrusionAllowed(cold);
    Motion1::dittoMode = ditto;
    if (isDry) {
        Printer::debugSet(8);
    }
}

#endif

#if LEVELING_METHOD == LEVELING_METHOD_GRID // Grid

float Leveling::grid[MAX_GRID_SIZE][MAX_GRID_SIZE];
float Leveling::gridTemp;
char Leveling::autoImportDir[] = BUMP_DEFAULT_AUTOIMPORT_DIR;
uint16_t Leveling::eprStart;
float Leveling::xMin, Leveling::xMax, Leveling::yMin, Leveling::yMax;
uint8_t Leveling::distortionEnabled;
float Leveling::dx, Leveling::dy, Leveling::invDx, Leveling::invDy;
float Leveling::startDegrade, Leveling::endDegrade, Leveling::diffDegrade;
uint8_t Leveling::curGridSize;

void Leveling::init() {
    eprStart = EEPROM::reserve(EEPROM_SIGNATURE_GRID_LEVELING, 1, (4 * MAX_GRID_SIZE * MAX_GRID_SIZE) + 30);
    resetEeprom();
}

void Leveling::handleEeprom() {
    EEPROM::handleByte(eprStart + 20, PSTR("Bump Correction enabled [0/1]"), distortionEnabled);
    EEPROM::handleFloat(eprStart + 21, PSTR("100% Bump Correction until [mm]"), 2, startDegrade);
    EEPROM::handleFloat(eprStart + 25, PSTR("0% Bump Correction from [mm]"), 2, endDegrade);
    EEPROM::handleFloat(eprStart + 16, PSTR("Bump Correction temp. [deg C]"), 1, gridTemp);
    EEPROM::setSilent(true);
    EEPROM::handleFloat(eprStart + 0, Com::tEmpty, 2, xMin);
    EEPROM::handleFloat(eprStart + 4, Com::tEmpty, 2, xMax);
    EEPROM::handleFloat(eprStart + 8, Com::tEmpty, 2, yMin);
    EEPROM::handleFloat(eprStart + 12, Com::tEmpty, 2, yMax);
    EEPROM::handleByte(eprStart + 29, Com::tEmpty, curGridSize);
    curGridSize = RMath::min(curGridSize, MAX_GRID_SIZE);
    for (int y = 0; y < curGridSize; y++) {
        for (int x = 0; x < curGridSize; x++) {
            EEPROM::handleFloat(eprStart + 30 + 4 * (x + y * curGridSize), Com::tEmpty, 2, grid[x][y]);
        }
    }
    EEPROM::setSilent(false);
    updateDerived();
}

void Leveling::resetEeprom() {
    for (int y = 0; y < MAX_GRID_SIZE; y++) {
        for (int x = 0; x < MAX_GRID_SIZE; x++) {
            grid[x][y] = 0.0f;
        }
    }
    setDistortionEnabled(false);
    xMin = yMin = xMax = yMax = gridTemp = 0.0f;
    curGridSize = MAX_GRID_SIZE;
    startDegrade = BUMP_CORRECTION_START_DEGRADE;
    endDegrade = BUMP_CORRECTION_END_HEIGHT;
    updateDerived();
}

void Leveling::updateDerived() {
    dx = (xMax - xMin) / (curGridSize - 1);
    dy = (yMax - yMin) / (curGridSize - 1);
    invDx = 1.0 / dx;
    invDy = 1.0 / dy;
    if (endDegrade <= startDegrade) { // fix logic order if wrong
        endDegrade = startDegrade + 0.5;
    }
    diffDegrade = -1.0f / (endDegrade - startDegrade);
}

void Leveling::setDistortionEnabled(bool newState) {
#if ENABLE_BUMP_CORRECTION == 0
    newState = false;
#endif
    bool nonZero = false;
    if (newState) {
        for (int y = 0; y < curGridSize; y++) {
            for (int x = 0; x < curGridSize; x++) {
                nonZero |= grid[x][y] != 0;
            }
        }
        if (!nonZero) {
            newState = false;
            Com::printFLN(PSTR("All corrections are 0, disabling bump correction!"));
        } else {
            if (!xMin && !xMax && !yMin && !yMax) {
                PrinterType::getBedRectangle(xMin, xMax, yMin, yMax);
                updateDerived();
            }
        }
    }
    if (newState == distortionEnabled) {
        return;
    }
    distortionEnabled = newState;
    Motion1::correctBumpOffset();
}

bool Leveling::measure(GCode* com) {
    float pos[NUM_AXES];
    float tOffMinX, tOffMaxX, tOffMinY, tOffMaxY;
    Plane plane;
    PlaneBuilder builder;

    builder.reset();
    PrinterType::getBedRectangle(xMin, xMax, yMin, yMax);
    uint16_t gridSize = com->getP(MAX_GRID_SIZE);
    uint8_t repetitions = com->hasR() ? static_cast<uint8_t>(com->R) : Z_PROBE_REPETITIONS;
    bool useMedian = com->hasA() ? static_cast<bool>(com->A) : Z_PROBE_USE_MEDIAN;
    // Sanity check for bad eeprom config
    // Todo: triggers a fatal error right now
    if (xMin == xMax || yMin == yMax || gridSize > MAX_GRID_SIZE || gridSize < 3) {
        return false;
    }
    if (repetitions < 1u) {
        repetitions = 1u;
    }
    xMin += Z_PROBE_BORDER; // Safety border from config
    xMax -= Z_PROBE_BORDER;
    yMin += Z_PROBE_BORDER;
    yMax -= Z_PROBE_BORDER;
    // get get min and max offsets from tools (without z-probe offsets)
    Tool::minMaxOffsetForAxis(X_AXIS, tOffMinX, tOffMaxX);
    Tool::minMaxOffsetForAxis(Y_AXIS, tOffMinY, tOffMaxY);
    // calc min and max coordinates defining the area reachable by ZProbe
    xMin = RMath::max(xMin - tOffMaxX + ZProbeHandler::xOffset(), xMin);
    xMax = RMath::min(xMax - tOffMinX + ZProbeHandler::xOffset(), xMax);
    yMin = RMath::max(yMin - tOffMaxY + ZProbeHandler::yOffset(), yMin);
    yMax = RMath::min(yMax - tOffMinY + ZProbeHandler::yOffset(), yMax);

    updateDerived();

    setDistortionEnabled(false);
    Motion1::setAutolevelActive(false);
    Motion1::homeAxes(7); // Home x, y and z
    float px = Motion1::currentPosition[X_AXIS], py = Motion1::currentPosition[Y_AXIS];
    // move first z axis, deltas don't like moving xy at top!
    Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, ZProbeHandler::optimumProbingHeight());
    Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
    Motion1::setTmpPositionXYZ(px, py, ZProbeHandler::optimumProbingHeight());
    PrinterType::closestAllowedPositionWithNewXYOffset(Motion1::tmpPosition, ZProbeHandler::xOffset(), ZProbeHandler::yOffset(), Z_PROBE_BORDER);
    Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
    if (!ZProbeHandler::activate()) {
        return false;
    }

    uint16_t probePoints = gridSize * gridSize;
    Com::printF(PSTR("Autoleveling with "), probePoints);
    Com::printFLN(PSTR(" grid points..."));

    uint16_t curNum = 0;
    float dispZ = IGNORE_COORDINATE;
    probeProgInfo dat(px, py, dispZ, curNum, probePoints);
    GUI::push(probeProgress, nullptr, GUIPageType::BUSY);

    Motion1::copyCurrentPrinter(pos);
    bool ok = true;
    float tempDx = (xMax - xMin) / (gridSize - 1);
    float tempDy = (yMax - yMin) / (gridSize - 1);

    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            if (!ok || Printer::breakLongCommand) {
                break;
            }
            int xx;
            if (y & 1) { // zig zag pattern for faster measurement
                px = xMax - (x * tempDx);
                xx = gridSize - x - 1;
            } else {
                px = xMin + (x * tempDx);
                xx = x;
            }
            py = yMin + (y * tempDy);
            pos[X_AXIS] = px - ZProbeHandler::xOffset();
            pos[Y_AXIS] = py - ZProbeHandler::yOffset();
            pos[Z_AXIS] = ZProbeHandler::optimumProbingHeight();
            pos[E_AXIS] = IGNORE_COORDINATE;

            float bedPos[2] = { px, py };
            if (PrinterType::positionOnBed(bedPos) && PrinterType::positionAllowed(pos, pos[Z_AXIS])) {
                if (ok) {
#if NUM_HEATED_BEDS && Z_PROBE_PAUSE_BED_REHEAT_TEMP
                    if (x && ZProbeHandler::getHeaterPause() && heatedBeds[0u]->isPaused()) {
                        if (fabs(heatedBeds[0u]->getCurrentTemperature() - heatedBeds[0u]->getTargetTemperature()) > Z_PROBE_PAUSE_BED_REHEAT_TEMP) {
                            heatedBeds[0u]->unpause();
                            Motion1::waitForEndOfMoves();
                            GUI::setStatusP(PSTR("Reheating bed..."), GUIStatusLevel::REGULAR);
                            pos[Z_AXIS] += 3.0f;
                            Motion1::moveByPrinter(pos, Motion1::moveFeedrate[X_AXIS], false);
                            Motion1::waitForEndOfMoves();
                            heatedBeds[0u]->waitForTargetTemperature();
                            pos[Z_AXIS] -= 3.0f;
                            GUI::clearStatus();
                            heatedBeds[0u]->pause();
                            HAL::delayMilliseconds(150ul);
                        }
                    }
#endif
                    //Todo handle probe min bed temp (if disable heaters is on) over long durations
                    Motion1::moveByPrinter(pos, Motion1::moveFeedrate[X_AXIS], false);
                    float h = ZProbeHandler::runProbe(repetitions, useMedian);
                    ok &= h != ILLEGAL_Z_PROBE;
                    grid[xx][y] = h;
                    if (ok) {
                        curNum = ((y * gridSize) + x) + 1;
                        float diff = ZProbeHandler::getBedDistance() - h;
                        dispZ = diff;
                        GUI::contentChanged = true;
                        builder.addPoint(px, py, h);
                    } else if (h == ILLEGAL_Z_PROBE) {
                        dispZ = ILLEGAL_Z_PROBE;
                    }
                }
            } else {
                grid[xx][y] = ILLEGAL_Z_PROBE;
            }
        }
    }
    if (ok && builder.numPoints() < 3 && !Printer::breakLongCommand) {
        ok = false;
        Com::printFLN(PSTR("You need at least 3 valid points for correction!"));
    }
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
    Motion1::setTmpPositionXYZ(0, 0, IGNORE_COORDINATE); // better to go to center for deltas!
    Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
#endif
    ZProbeHandler::deactivate();
    if (ok && !Printer::breakLongCommand) {
        GUI::setStatusP(PSTR("Autolevel complete!"), GUIStatusLevel::INFO);
#if NUM_HEATED_BEDS
        gridTemp = heatedBeds[0]->getTargetTemperature();
#endif
        curGridSize = gridSize;
        updateDerived(); // Update Dx Dy etc.
        builder.createPlane(plane, false);
        LevelingCorrector::correct(&plane);
        // Reduce to distortion after bed correction
        for (int y = 0; y < curGridSize; y++) {
            for (int x = 0; x < curGridSize; x++) {
                if (grid[x][y] != ILLEGAL_Z_PROBE) {
                    grid[x][y] = plane.z(xPosFor(x), yPosFor(y)) - grid[x][y];
                }
            }
        }
        extrapolateGrid();
#if ENABLE_BUMP_CORRECTION
        setDistortionEnabled(true); // if we support it we should use it by default
        reportDistortionStatus();
#endif
    } else {
        GUI::pop();
        GUI::setStatusP(Com::tEmpty, GUIStatusLevel::REGULAR);
        GUI::contentChanged = true;
        if (!ok) {
            resetEeprom();
        }
    }
    Motion1::printCurrentPosition();
    return ok;
}

void Leveling::extrapolateGrid() {
    int illegalPoints = 0;
    int optX, optY, optNeighbours;
    float grid2[MAX_GRID_SIZE][MAX_GRID_SIZE];
    for (int y = 0; y < curGridSize; y++) {
        for (int x = 0; x < curGridSize; x++) {
            grid2[x][y] = grid[x][y];
            if (grid[x][y] == ILLEGAL_Z_PROBE) {
                illegalPoints++;
            }
        }
    }
    while (illegalPoints > 0) {
        // Find point with most neighbours as next point to optimize
        optNeighbours = -1;
        for (int y = 0; y < curGridSize; y++) {
            for (int x = 0; x < curGridSize; x++) {
                if (grid[x][y] != ILLEGAL_Z_PROBE) {
                    continue;
                }
                int n = extrapolateableNeighbours(x, y);
                if (n > optNeighbours) {
                    optNeighbours = n;
                }
                if (n > 0) {
                    grid2[x][y] = extrapolateNeighbours(x, y);
                    illegalPoints--;
                }
            }
        }
        if (optNeighbours <= 0) {
            for (int y = 0; y < curGridSize; y++) {
                for (int x = 0; x < curGridSize; x++) {
                    grid[x][y] = 0;
                }
            }
            Com::printWarningFLN(PSTR("Not enough data to extrapolate z distortion - disabled!"));
            return;
        }
        // Extrapolate all points with optNeigbours
        for (int y = 0; y < curGridSize; y++) {
            for (int x = 0; x < curGridSize; x++) {
                grid[x][y] = grid2[x][y];
            }
        }
    }
}

bool Leveling::extrapolateableNeighbours(int x, int y) {
    int n = 0;
    int x1, y1, x2, y2;
    for (int dir = 0; dir < 8; dir++) {
        x1 = x2 = x;
        y1 = y2 = y;
        bool ok1 = gridIndexForDir(dir, 1, x1, y1);
        bool ok2 = gridIndexForDir(dir, 2, x2, y2);
        if (ok1 && ok2 && grid[x1][y1] != ILLEGAL_Z_PROBE && grid[x2][y2] != ILLEGAL_Z_PROBE) {
            n++;
        }
    }
    return n;
}

float Leveling::extrapolateNeighbours(int x, int y) {
    int n = 0;
    int x1, y1, x2, y2;
    float correction = 0;
    for (int dir = 0; dir < 8; dir++) {
        x1 = x2 = x;
        y1 = y2 = y;
        bool ok1 = gridIndexForDir(dir, 1, x1, y1);
        bool ok2 = gridIndexForDir(dir, 2, x2, y2);
        if (ok1 && ok2 && grid[x1][y1] != ILLEGAL_Z_PROBE && grid[x2][y2] != ILLEGAL_Z_PROBE) {
            n++;
            correction += 2.0f * grid[x1][y1] - grid[x2][y2];
        }
    }
    if (n == 0) {
        return 0;
    }
    return correction / static_cast<float>(n);
}

bool Leveling::gridIndexForDir(int dir, int dist, int& x, int& y) {
    switch (dir) {
    case 0:
        x += dist;
        break;
    case 1:
        x += dist;
        y -= dist;
        break;
    case 2:
        y -= dist;
        break;
    case 3:
        x -= dist;
        y -= dist;
        break;
    case 4:
        x -= dist;
        break;
    case 5:
        x -= dist;
        y += dist;
        break;
    case 6:
        y += dist;
        break;
    case 7:
        x += dist;
        y += dist;
        break;
    }
    return validGridIndex(x, y);
}

#if ENABLE_BUMP_CORRECTION

void Leveling::addDistortion(float* pos) {
    if (!distortionEnabled || pos[Z_AXIS] >= endDegrade) {
        return; // no correction from here on
    }
    float factor;
    if (pos[Z_AXIS] >= startDegrade) {
        factor = 1.0f + (pos[Z_AXIS] - startDegrade) * diffDegrade;
    } else {
        factor = 1.0f;
    }
    pos[Z_AXIS] += factor * distortionAt(pos[X_AXIS], pos[Y_AXIS]);
}

void Leveling::subDistortion(float* pos) {
    if (!distortionEnabled || pos[Z_AXIS] >= endDegrade) {
        return; // no correction from here on
    }
    float factor;
    if (pos[Z_AXIS] >= startDegrade) {
        factor = 1.0 + (pos[Z_AXIS] - startDegrade) * diffDegrade;
    } else {
        factor = 1.0f;
    }
    pos[Z_AXIS] -= factor * distortionAt(pos[X_AXIS], pos[Y_AXIS]);
}

float Leveling::distortionAt(float xp, float yp) {
    xp -= xMin;
    yp -= yMin;
    int fx = floorf(xp * invDx);
    int fy = floorf(yp * invDy);
    xp -= fx * dx; // Now between 0 and dx
    xp *= invDx;   // Now between 0 and 1
    yp -= fy * dy;
    yp *= invDy;
    int fx2 = fx + 1;
    int fy2 = fy + 1;
    if (fx < 0) {
        fx = fx2 = 0;
    } else if (fx >= curGridSize) {
        fx = fx2 = curGridSize - 1;
    } else if (fx == curGridSize - 1) {
        fx2 = fx;
    }
    if (fy < 0) {
        fy = fy2 = 0;
    } else if (fy >= curGridSize) {
        fy = fy2 = curGridSize - 1;
    } else if (fy == curGridSize - 1) {
        fy2 = fy;
    }
    float xp1 = 1.0f - xp;
    float c1 = grid[fx][fy] * xp1 + grid[fx2][fy] * xp;
    float c2 = grid[fx][fy2] * xp1 + grid[fx2][fy2] * xp;
    return c1 * (1.0f - yp) + yp * c2;
}

void Leveling::reportDistortionStatus() {
    Com::printFLN(distortionEnabled ? PSTR("Z bump correction enabled") : PSTR("Z bump correction disabled"));
    Com::printF(PSTR("G33 X min:"), xMin);
    Com::printF(PSTR(" X max:"), xMax);
    Com::printF(PSTR(" Y min:"), yMin);
    Com::printF(PSTR(" Y max:"), yMax);
    Com::printF(PSTR(" Grid Size:"), curGridSize);
    Com::printF(PSTR("x"), curGridSize);
    Com::printFLN(PSTR(" Bed Temp:"), gridTemp, 1);
}

void Leveling::showMatrix() {
    for (int iy = 0; iy < curGridSize; iy++) {
        for (int ix = 0; ix < curGridSize; ix++) {
            Com::printF(PSTR("G33 X"), xPosFor(ix), 2);
            Com::printF(PSTR(" Y"), yPosFor(iy), 2);
            Com::printFLN(PSTR(" Z"), grid[ix][iy], 3);
        }
    }
}

void Leveling::set(float x, float y, float z) {
    if (!xMin && !xMax && !yMin && !yMax) {
        PrinterType::getBedRectangle(xMin, xMax, yMin, yMax);
        updateDerived();
    }
    int32_t ix = lroundf((x - xMin) * invDx);
    int32_t iy = lroundf((y - yMin) * invDy);
    if (validGridIndex(ix, iy)) {
        if (BUMP_LIMIT_TO <= 0 || fabsf(z) <= BUMP_LIMIT_TO) {
            grid[ix][iy] = z;
            Motion1::correctBumpOffset();
        } else {
            Com::printWarningFLN(PSTR("Max. distortion value exceeded - not setting this value."));
        }
    } else {
        Com::printFLN(PSTR("Invalid position - no grid point in neighbourhood."));
    }
}

void Leveling::execute_M323(GCode* com) {
    if (com->hasS() && com->S > 0l) {
// Auto import bed mesh
#if NUM_HEATED_BEDS > 0 && SDSUPPORT
        if (com->S == 2l) {
            if ((!com->hasX() || com->X < 1.0f)
                || (sd.state < SDState::SD_MOUNTED)) {
                return;
            }

            sd.fileSystem.chdir();
            if (!sd.fileSystem.exists(autoImportDir)) {
                Com::printF(Com::tErrorImportBump);
                Com::printFLN(PSTR(" Auto-import directory does not exist!"));
                return;
            }

            float maxDif = com->hasD() ? com->D : 10.0f;
            float lowestTemp = IGNORE_COORDINATE;
            uint32_t lowestTempFileIndex = 0ul;
            sd_file_t rootDir = sd.fileSystem.open(autoImportDir);
            sd.doForDirectory(
                rootDir, [&](sd_file_t& file, sd_file_t& dir, size_t depth) {
                    if (!file.isDir()
                        && strstr_P(sd.getFN(file), PSTR(".csv"))
                        && lowestTemp != 0.0f) {
                        CSVParser csv(&file);
                        float version = 0.0f, temp = 0.0f;
                        uint8_t gridSize = 0u;

                        if (csv.getField(Com::tBumpCSVHeader, version, CSVDir::NEXT)
                            && csv.getField(PSTR("GridSize"), gridSize, CSVDir::BELOW)
                            && csv.getField(PSTR("BedTemp"), temp, CSVDir::BELOW)
                            && gridSize <= MAX_GRID_SIZE) {
                            float diff = fabsf((com->X) - temp);
                            if (diff < 0.5f) { // Found exact one, just use it.
                                lowestTemp = 0.0f;
                                lowestTempFileIndex = file.dirIndex();
                                return false; // end the doForDir now.
                            } else if (diff <= maxDif) {
                                if (diff < lowestTemp) {
                                    lowestTemp = diff;
                                    lowestTempFileIndex = file.dirIndex();
                                }
                            }
                        }
                    }
                    return true;
                },
                false);

            if (lowestTemp < IGNORE_COORDINATE
                && lowestTempFileIndex) {
                rootDir.rewind();
                sd_file_t file;
                if (file.open(&rootDir, lowestTempFileIndex, O_RDONLY)) {
                    sd.fileSystem.chdir(autoImportDir);
                    importBumpMatrix(sd.getFN(file));
                    if (com->hasP() && com->P != 0l) {
                        EEPROM::markChanged();
                    }
                    file.close();
                } else {
                    Com::printF(Com::tErrorImportBump);
                    Com::printFLN(Com::tFileOpenFailed);
                }
            } else {
                Com::printF(Com::tErrorImportBump);
                Com::printF(PSTR(" No valid matrix file for "), com->X, 0u);
                Com::printF(Com::tUnitDegCelsius);
                bool isRoot = (autoImportDir[0u] == '/' && autoImportDir[1u] == '\0');
                Com::printFLN(PSTR(" found in "), isRoot ? PSTR("root") : autoImportDir);
            }
            sd.fileSystem.chdir();
            rootDir.close();
            return;
        }
#endif
        // End auto import bed mesh
        if (distortionEnabled != (com->S != 0l)) {
            setDistortionEnabled(!distortionEnabled);
            if (com->hasP() && com->P != 0l) {
                EEPROM::markChanged();
            }
        }
    }
    reportDistortionStatus();
}
void Leveling::importBumpMatrix(char* filename) {
#if SDSUPPORT
    if (sd.state < SDState::SD_MOUNTED) {
        Com::printF(Com::tErrorImportBump);
        Com::printFLN(Com::tNoMountedCard);
        return;
    }
    Motion1::waitForEndOfMoves();

    // Handle auto import directory changes
    if (filename[strlen(filename) - 1u] == '/') {
        sd_file_t dir;
        if (dir.open(filename) && dir.isDir()) {
            bool isRoot = (filename[0u] == '/' && filename[1u] == '\0');
            memcpy(autoImportDir, isRoot ? "/" : filename, sizeof(autoImportDir));
            Com::printFLN(PSTR("Bump matrix auto-import directory set to "), isRoot ? PSTR("root") : filename);
            dir.close();
            return;
        }
        dir.close();
    }
    // end handle auto import

    if (!CSVParser::validCSVExt(filename)) {
        Com::printF(Com::tErrorImportBump);
        Com::printFLN(Com::tInvalidFilename, filename);
        return;
    }

    sd_file_t tempFile;
    if (!tempFile.open(filename, O_RDWR) || !tempFile.fileSize()) {
        Com::printF(Com::tErrorImportBump);
        Com::printFLN(Com::tFileOpenFailed);
        return;
    }

    Com::printF(PSTR("Importing bump matrix file: "), filename);
    Com::printFLN(PSTR("..."));

    CSVParser csv(&tempFile);

    bool ok = true;
    float version = 0.0f;
    if (csv.getField(Com::tBumpCSVHeader, version, CSVDir::NEXT)) {
        // TODO: Handle any version iteration differences here
        // 5/8/20 first version (0.1)
    } else {
        ok = false;
    }

    uint8_t newSize = 0u;
    if (ok && (!csv.getField(PSTR("GridSize"), newSize, CSVDir::BELOW) || newSize > MAX_GRID_SIZE)) {
        if (newSize > 0u) {
            // Handle a grid mismatch error on it's own to inform the user about it.
            tempFile.close();
            Com::printF(Com::tErrorImportBump);
            Com::printF(PSTR("Grid size larger than max grid size "), newSize);
            Com::printFLN(" vs ", MAX_GRID_SIZE);
            return;
        }
        ok = false;
    }
    if (ok) { // Import xMin etc fields
        uint8_t valid = 0u;
        valid += csv.getField(PSTR("xMin"), xMin, CSVDir::BELOW);
        valid += csv.getField(PSTR("xMax"), xMax, CSVDir::BELOW);
        valid += csv.getField(PSTR("yMin"), yMin, CSVDir::BELOW);
        valid += csv.getField(PSTR("yMax"), yMax, CSVDir::BELOW);
        valid += csv.getField(PSTR("BedTemp"), gridTemp, CSVDir::BELOW);

        valid += csv.getField(PSTR("GridSize"), curGridSize, CSVDir::BELOW);
        float tempMaxZ = 0.0f;
        valid += csv.getField(PSTR("MaxPosZ"), tempMaxZ, CSVDir::BELOW);
        // Safety?
        if (fabs(Motion1::maxPos[Z_AXIS] - tempMaxZ) < 10.0f) {
            Motion1::maxPos[Z_AXIS] = tempMaxZ;
        } else {
            ok = false;
        }

        if (valid != 7u) {
            ok = false;
        }
    }

    if (!csv.seekGridPos(3, 0)) { // Seek to the start of the bump grid
        ok = false;
    }

    if (ok) {
        for (size_t iy = 0u; iy < curGridSize; iy++) {
            for (size_t ix = 0u; ix < curGridSize; ix++) {
                if (!csv.getNextCell(grid[ix][iy])) {
                    ok = false;
                    break;
                }
            }
        }
    }

    if (ok) {
        constexpr size_t transBufSize = (sizeof(Motion1::autolevelTransformation) / 4u);
        for (size_t i = 0u; i < transBufSize; i++) {
            if (!csv.getNextCell(Motion1::autolevelTransformation[i])) {
                ok = false;
                break;
            }
        }
    }

    tempFile.close();

    if (!ok) {
        Com::printF(Com::tErrorImportBump);
        Com::printFLN(Com::tSDReadError);
        resetEeprom();
        return;
    }

    updateDerived();

    if (Motion1::isAutolevelActive()) {
        Motion1::updateRotMinMax();
        Motion1::updatePositionsFromCurrentTransformed();
    } else {
        Motion1::setAutolevelActive(true);
    }

    Motion1::waitForEndOfMoves();
    setDistortionEnabled(true);
    Motion1::waitForEndOfMoves();

    Com::printArrayFLN(Com::tTransformationMatrix, Motion1::autolevelTransformation, 9u, 6u);
    reportDistortionStatus();

    Com::printFLN(PSTR("Bump matrix succesfully imported."));
#else
    Com::printF(Com::tErrorImportBump);
    Com::printFLN(PSTR("No SD Card support compiled!"));
#endif
}
void Leveling::exportBumpMatrix(char* filename) {
#if SDSUPPORT
    if (sd.state < SDState::SD_MOUNTED) {
        Com::printF(Com::tErrorExportBump);
        Com::printFLN(Com::tNoMountedCard);
        return;
    }

    if (!xMin && !xMax && !yMin && !yMax) {
        Com::printF(Com::tErrorExportBump);
        Com::printF(Com::tSpace);
        Com::printFLN(Com::tNoDistortionData);
        return;
    }
    if (!CSVParser::validCSVExt(filename)) {
        Com::printF(Com::tErrorExportBump);
        Com::printFLN(Com::tInvalidFilename, filename);
        return;
    }
    sd.fileSystem.chdir();

    sd_file_t tempFile;
    if (!tempFile.open(filename, O_RDWR | O_CREAT | O_TRUNC)) {
        Com::printF(Com::tErrorExportBump);
        Com::printFLN(Com::tFileOpenFailed);
        return;
    }
    tempFile.rewind();

    Com::printF(PSTR("Exporting bump matrix to "), filename);
    Com::printFLN(PSTR("..."));

    // As of version 0.1,
    // Autolevel, TrigHeight fields are meta data exposed to the user, but unused on import.

    constexpr char metaDataCols[] = PSTR("xMin,xMax,yMin,yMax,Autolevel,GridSize,BedTemp,TrigHeight,MaxPosZ");
    constexpr float bumpMatrixVers = 0.1f;

    tempFile.write(reinterpret_cast<const uint8_t*>(Com::tBumpCSVHeader), strlen(Com::tBumpCSVHeader));
    tempFile.write(',');
    tempFile.printField(bumpMatrixVers, '\n');

    tempFile.write(reinterpret_cast<const uint8_t*>(metaDataCols), sizeof(metaDataCols));
    tempFile.write('\n');

    tempFile.printField(xMin, ',', 2u);
    tempFile.printField(xMax, ',', 2u);
    tempFile.printField(yMin, ',', 2u);
    tempFile.printField(yMax, ',', 2u);

    tempFile.printField(static_cast<fast8_t>(Motion1::isAutolevelActive()), ',');
    tempFile.printField(static_cast<fast8_t>(curGridSize), ',');
    tempFile.printField(roundf(gridTemp), ',', 1u);
    tempFile.printField(static_cast<float>(Z_PROBE_HEIGHT), ',', 4u);
    tempFile.printField(Motion1::maxPos[Z_AXIS], '\n', 5u);

    for (size_t iy = 0u; iy < curGridSize; iy++) {
        for (size_t ix = 0u; ix < curGridSize; ix++) {
            if (ix == (curGridSize - 1u)) {
                tempFile.printField(grid[ix][iy], '\n', 6u);
            } else {
                tempFile.printField(grid[ix][iy], ',', 6u);
            }
        }
    }

    constexpr ufast8_t autoLevelSize = (sizeof(Motion1::autolevelTransformation) / sizeof(Motion1::autolevelTransformation[0u]));
    for (size_t i = 0ul; i < autoLevelSize; i++) {
        tempFile.printField(Motion1::autolevelTransformation[i], (i == (autoLevelSize - 1ul)) ? '\n' : ',', 6u);
    }

    float fileSize = (tempFile.fileSize() / 1000.0f);
    if (tempFile.sync() && tempFile.close()) {
        Com::printF(PSTR("Bump matrix written to SD Card. ("), filename);
        Com::printF(PSTR(" / "), fileSize, 2u);
        Com::printFLN(PSTR(" kB) "));
    } else {
        Com::printF(Com::tErrorExportBump);
        Com::printFLN(Com::tErrorWritingToFile);
    }
#else
    Com::printF(Com::tErrorExportBump);
    Com::printFLN(PSTR("No SD Card support compiled!"));
#endif
}
#else
void Leveling::reportDistortionStatus() {
    Com::printFLN(PSTR("No bump correction support compiled!"));
}
#endif

bool Leveling::execute_G32(GCode* com) {
    bool ok = measure(com);
    if (com->hasS() && com->S > 0) {
        EEPROM::markChanged();
    }
    return ok;
}
void Leveling::execute_G33(GCode* com) {
#if ENABLE_BUMP_CORRECTION
    if (com->hasL()) { // G33 L0 - List distortion matrix
        if (!xMin && !xMax && !yMin && !yMax) {
            Com::printWarningFLN(Com::tNoDistortionData);
            return;
        }
        reportDistortionStatus();
        showMatrix();
    } else if (com->hasR()) { // G33 R0 - Reset distortion matrix
        Com::printInfoFLN(PSTR("Resetting Z bump correction"));
        for (int y = 0; y < MAX_GRID_SIZE; y++) {
            for (int x = 0; x < MAX_GRID_SIZE; x++) {
                grid[x][y] = 0.0f;
            }
        }
        Motion1::correctBumpOffset();
    } else if (com->hasX() || com->hasY() || com->hasZ()) { // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
        if (com->hasX() && com->hasY() && com->hasZ()) {
            set(com->X, com->Y, com->Z);
        } else {
            Com::printErrorFLN(PSTR("You need to define X, Y and Z to set a point!"));
        }
    } else if (com->hasI()) { // Inform contained correction
        float pos[NUM_AXES];
        Motion1::copyCurrentPrinter(pos);
        float z0 = pos[Z_AXIS];
        addDistortion(pos);
        float diff = pos[Z_AXIS] - z0;
        Com::printF(PSTR("G33 Info x:"), Motion1::currentPosition[X_AXIS]);
        Com::printF(PSTR(" y:"), Motion1::currentPosition[Y_AXIS]);
        Com::printF(PSTR(" z:"), Motion1::currentPosition[Z_AXIS]);
        Com::printFLN(PSTR(" bump:"), diff);
    }
#else
    reportDistortionStatus();
#endif
}
#endif

#if LEVELING_METHOD == LEVELING_METHOD_4_POINT_SYMMETRIC // 4 points

bool Leveling::measure(GCode* com) {
    uint8_t repetitions = com->hasR() ? static_cast<uint8_t>(com->R) : Z_PROBE_REPETITIONS;
    bool useMedian = com->hasA() ? static_cast<bool>(com->A) : Z_PROBE_USE_MEDIAN;
    if (repetitions < 1u) {
        repetitions = 1u;
    }
    Plane plane;
    PlaneBuilder builder;
    builder.reset();
    float h1(0), h2(0), h3(0), h4(0);
    bool ok = true, prevAuto = Motion1::isAutolevelActive();
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
    Motion1::setAutolevelActive(false, true);
    Motion1::homeAxes(7); // Home x, y and z
    if (!Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P1_X, L_P1_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        if (!ZProbeHandler::activate()) {
            return false;
        }
        h1 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h1 != ILLEGAL_Z_PROBE;
    }
    if (ok && !Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P2_X, L_P2_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        h2 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h2 != ILLEGAL_Z_PROBE;
    }
    if (ok && !Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P3_X, L_P3_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        h3 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h3 != ILLEGAL_Z_PROBE;
    }
    if (ok && !Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(x1Mirror, y1Mirror, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        h4 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h4 != ILLEGAL_Z_PROBE;
    }
    ZProbeHandler::deactivate();
    if (ok && !Printer::breakLongCommand) {
        const float t2 = h2 + (h3 - h2) * t; // theoretical height for crossing point for symmetric axis
        h1 = t2 - (h4 - h1) * 0.5;           // remove bending part
        builder.addPoint(L_P1_X, L_P1_Y, h1);
        builder.addPoint(L_P2_X, L_P2_Y, h2);
        builder.addPoint(L_P3_X, L_P3_Y, h3);
        builder.createPlane(plane, false);
        LevelingCorrector::correct(&plane);
    } else if (ok && Printer::breakLongCommand) {
        Motion1::setAutolevelActive(prevAuto, true);
    }
    Motion1::printCurrentPosition();
    return ok;
}

bool Leveling::execute_G32(GCode* com) {
    bool ok = measure(com);
    if (com->hasS() && com->S > 0) {
        EEPROM::markChanged();
    }
    return ok;
}

#endif

#if LEVELING_METHOD == LEVELING_METHOD_3_POINTS // 3 points

bool Leveling::measure(GCode* com) {
    uint8_t repetitions = com->hasR() ? static_cast<uint8_t>(com->R) : Z_PROBE_REPETITIONS;
    bool useMedian = com->hasA() ? static_cast<bool>(com->A) : Z_PROBE_USE_MEDIAN;
    if (repetitions < 1u) {
        repetitions = 1u;
    }
    Plane plane;
    PlaneBuilder builder;
    builder.reset();
    float h1 = 0.0f, h2 = 0.0f, h3 = 0.0f;
    bool ok = true, prevAuto = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false, true);
    Motion1::homeAxes(7); // Home x, y and z
    if (!Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P1_X, L_P1_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        if (!ZProbeHandler::activate()) {
            return false;
        }
        h1 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h1 != ILLEGAL_Z_PROBE;
    }
    if (ok && !Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P2_X, L_P2_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        h2 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h2 != ILLEGAL_Z_PROBE;
    }
    if (ok && !Printer::breakLongCommand) {
        Motion1::setTmpPositionXYZ(L_P3_X, L_P3_Y, ZProbeHandler::optimumProbingHeight());
        ok &= Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[X_AXIS], false);
        h3 = ZProbeHandler::runProbe(repetitions, useMedian);
        ok &= h3 != ILLEGAL_Z_PROBE;
    }
    ZProbeHandler::deactivate();
    if (ok && !Printer::breakLongCommand) {
        builder.addPoint(L_P1_X, L_P1_Y, h1);
        builder.addPoint(L_P2_X, L_P2_Y, h2);
        builder.addPoint(L_P3_X, L_P3_Y, h3);
        builder.createPlane(plane, false);
        LevelingCorrector::correct(&plane);
    } else if (ok && Printer::breakLongCommand) {
        Motion1::setAutolevelActive(prevAuto, true);
    }
    Motion1::printCurrentPosition();
    return ok;
}

bool Leveling::execute_G32(GCode* com) {
    bool ok = measure(com);
    if (com->hasS() && com->S > 0) {
        EEPROM::markChanged();
    }
    return ok;
}

#endif
