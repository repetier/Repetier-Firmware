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

void GCode_0_1(GCode* com) {
#if defined(G0_FEEDRATE) && G0_FEEDRATE > 0
    float backupFeedrate = Printer::feedrate;
    if (com->G == 0 && G0_FEEDRATE > 0) {
        Printer::feedrate = G0_FEEDRATE;
    }
#endif
    if (com->hasP())
        Printer::setNoDestinationCheck(com->P == 0);
    Tool::getActiveTool()->extractG1(com);
    Printer::setDestinationStepsFromGCode(com); // For X Y Z E F
#if defined(G0_FEEDRATE) && G0_FEEDRATE > 0
    if (!(com->hasF() && com->F > 0.1)) {
        Printer::feedrate = backupFeedrate;
    }
#endif
#if UI_HAS_KEYS
    // ui can only execute motion commands if we are not waiting inside a move for an
    // old move to finish. For normal response times, we always leave one free after
    // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
    // gets filled while waiting, the lost is neglectable.
//        PrintLine::waitForXFreeLines(1, true);
#endif // UI_HAS_KEYS
#ifdef DEBUG_QUEUE_MOVE
    {

        InterruptProtectedBlock noInts;
        int lc = (int)PrintLine::linesCount;
        int lp = (int)PrintLine::linesPos;
        int wp = (int)PrintLine::linesWritePos;
        int n = (wp - lp);
        if (n < 0)
            n += PRINTLINE_CACHE_SIZE;
        noInts.unprotect();
        if (n != lc)
            Com::printFLN(PSTR("Buffer corrupted"));
    }
#endif
}

void GCode_2_3(GCode* com) {
#if ARC_SUPPORT
#endif
}

void GCode_4(GCode* com) {
    int32_t codenum;
    Motion1::waitForEndOfMoves();
    codenum = 0;
    if (com->hasP())
        codenum = com->P; // milliseconds to wait
    if (com->hasS())
        codenum = com->S * 1000;          // seconds to wait
    codenum += HAL::timeInMilliseconds(); // keep track of when we started waiting
    while ((uint32_t)(codenum - HAL::timeInMilliseconds()) < 2000000000) {
        GCode::keepAlive(Processing, 2);
        Commands::checkForPeriodicalActions(true);
    }
}

void GCode_10(GCode* com) {
#if FEATURE_RETRACTION && NUM_TOOLS > 0
#if NUM_TOOLS > 1
    Tool::getActiveTool()->retract(true, com->hasS() && com->S > 0);
#else
    Tool::getActiveTool()->retract(true, false);
#endif
#endif
}

void GCode_11(GCode* com) {
#if FEATURE_RETRACTION && NUM_TOOLS > 0
#if NUM_TOOLS > 1
    Tool::getActiveTool()->retract(false, com->hasS() && com->S > 0);
#else
    Tool::getActiveTool()->retract(false, false);
#endif
#endif
}

void GCode_20(GCode* com) {
    Printer::unitIsInches = 1;
}

void GCode_21(GCode* com) {
    Printer::unitIsInches = 0;
}

void GCode_28(GCode* com) {
    fast8_t homeAxis = 0;
    homeAxis |= com->hasX() ? 1 : 0;
    homeAxis |= com->hasY() ? 2 : 0;
    homeAxis |= com->hasZ() ? 4 : 0;
    homeAxis |= com->hasE() ? 8 : 0;
    homeAxis |= com->hasA() ? 16 : 0;
    homeAxis |= com->hasB() ? 32 : 0;
    homeAxis |= com->hasC() ? 64 : 0;
    Motion1::homeAxes(homeAxis);
}

void GCode_29(GCode* com) {
    /*
#if FEATURE_Z_PROBE
    // Printer::prepareForProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
    float actTemp[NUM_EXTRUDER];
    for (int i = 0; i < NUM_EXTRUDER; i++)
        actTemp[i] = extruder[i].tempControl.targetTemperatureC;
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::max(EEPROM::zProbeHeight(), static_cast<float>(ZHOME_HEAT_HEIGHT)), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    Motion1::waitForEndOfMoves();
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
    }
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
    }
#else
    if (extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
    bool ok = true;
    Printer::startProbing(true);
    bool oldAutolevel = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false);
    float sum = 0, last, oldFeedrate = Printer::feedrate;
    Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
    sum = Printer::runZProbe(true, false, Z_PROBE_REPETITIONS, false);
    if (sum == ILLEGAL_Z_PROBE)
        ok = false;
    if (ok) {
        Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
        last = Printer::runZProbe(false, false);
        if (last == ILLEGAL_Z_PROBE)
            ok = false;
        sum += last;
    }
    if (ok) {
        Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
        last = Printer::runZProbe(false, true);
        if (last == ILLEGAL_Z_PROBE)
            ok = false;
        sum += last;
    }
    if (ok) {
        sum *= 0.33333333333333;
        Com::printFLN(Com::tZProbeAverage, sum);
        if (com->hasS() && com->S) {
#if MAX_HARDWARE_ENDSTOP_Z
#if DRIVE_SYSTEM == DELTA
            Printer::zLength += sum - Printer::currentPosition[Z_AXIS];
            Printer::updateDerivedParameter();
            Printer::homeAxis(true, true, true);
#else
            Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
            float zup = Printer::runZMaxProbe();
            if (zup == ILLEGAL_Z_PROBE) {
                ok = false;
            } else
                Printer::zLength = zup + sum - ENDSTOP_Z_BACK_ON_HOME;
#endif // DELTA
            Com::printInfoFLN(Com::tZProbeZReset);
            Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#else
            Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
            Com::printFLN(PSTR("Adjusted z origin"));
#endif // max z endstop
        }
        Printer::feedrate = oldFeedrate;
        Motion1::setAutolevelActive(oldAutolevel);
        if (ok && com->hasS() && com->S == 2)
            EEPROM::storeDataIntoEEPROM();
    }
    Motion1::printCurrentPosition();
    Printer::finishProbing();
    Printer::feedrate = oldFeedrate;
    if (!ok) {
        GCode::fatalError(PSTR("G29 leveling failed!"));
        break;
    }
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
    }
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
    }
#else
    if (extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
#endif
*/
}

void GCode_30(GCode* com) {
#if Z_PROBE_TYPE
    // G30 [Pn] [S]
    // G30 (the same as G30 P3) single probe set Z0
    // G30 S1 Z<real_z_pos> - measures probe height (P is ignored) assuming we are at real height Z
    // G30 H<height> R<offset> Make probe define new Z and z offset (R) at trigger point assuming z-probe measured an object of H height.
    if (com->hasS()) {
        float curHeight = (com->hasZ() ? com->Z : Motion1::currentPosition[Z_AXIS]);

        float startHeight = ZProbeHandler::getBedDistance() + (ZProbeHandler::getZProbeHeight() > 0 ? ZProbeHandler::getZProbeHeight() : 0);
        Motion1::setTmpPositionXYZ(IGNORE_COORDINATE, IGNORE_COORDINATE, startHeight);
        Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::moveFeedrate[Z_AXIS], false);
        float zheight = ZProbeHandler::runProbe();
        if (zheight == ILLEGAL_Z_PROBE) {
             GCode::fatalError(PSTR("G30 probing failed!"));
            return;
        }
        float zProbeHeight = ZProbeHandler::getZProbeHeight() + startHeight - zheight;

        ZProbeHandler::setZProbeHeight(zProbeHeight); // will also report on output
        Com::printFLN(PSTR("Z-probe height [mm]:"), zProbeHeight);

    } else {
        uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
        if (p & 1) {
            ZProbeHandler::activate();
        }
        float z = ZProbeHandler::runProbe();
        if (z == ILLEGAL_Z_PROBE) {
            GCode::fatalError(PSTR("G30 probing failed!"));
        } else if (com->hasR() || com->hasH()) {
            float h = Printer::convertToMM(com->hasH() ? com->H : 0);
            float o = Printer::convertToMM(com->hasR() ? com->R : h);
#if ENABLE_BUMP_CORRECTION
            // Undo z distortion correction contained in z
            float zCorr = 0;
            if (Leveling::isDistortionEnabled()) {
                zCorr = Leveling::distortionAt(Motion1::currentPosition[X_AXIS], Motion1::currentPosition[Y_AXIS]);
                z += zCorr;
            }
#endif
            Motion1::g92Offsets[Z_AXIS] = o - h;
            Motion1::currentPosition[Z_AXIS] = z + h + Motion1::minPos[Z_AXIS];
            Motion1::updatePositionsFromCurrent();
            Motion1::setAxisHomed(Z_AXIS, true);
        }
        if (p & 2) {
            ZProbeHandler::deactivate();
        }
    }
#endif
}

void GCode_31(GCode* com) {
    // G31 display hall sensor output
    if (ZProbe != nullptr) {
        ZProbe->update();
        Com::printF(Com::tZProbeState);
        Com::printFLN(ZProbe->triggered() ? Com::tHSpace : Com::tLSpace);
    }
}

void GCode_32(GCode* com) {
    Leveling::execute_G32(com);
}

void GCode_33(GCode* com) {
    Leveling::execute_G33(com);
}

void GCode_90(GCode* com) {
    Printer::relativeCoordinateMode = false;
    if (com->internalCommand)
        Com::printInfoFLN(PSTR("Absolute positioning"));
}

void GCode_91(GCode* com) {
    Printer::relativeCoordinateMode = true;
    if (com->internalCommand)
        Com::printInfoFLN(PSTR("Relative positioning"));
}

void GCode_92(GCode* com) {
    Motion1::fillPosFromGCode(*com, Motion1::tmpPosition, IGNORE_COORDINATE);
    FOR_ALL_AXES(i) {
        if (i != E_AXIS && Motion1::tmpPosition[i] != IGNORE_COORDINATE) {
            Motion1::g92Offsets[i] = Motion1::tmpPosition[i] - Motion1::currentPosition[i];
        }
    }
    if (com->hasE()) {
        Motion1::destinationPositionTransformed[E_AXIS] = Motion1::currentPosition[E_AXIS] = Motion1::currentPositionTransformed[E_AXIS] = Printer::convertToMM(com->E);
    }
    // if (com->hasX() || com->hasY() || com->hasZ()) {
    Com::printF(PSTR("X_OFFSET:"), Motion1::g92Offsets[X_AXIS], 3);
    Com::printF(PSTR(" Y_OFFSET:"), Motion1::g92Offsets[Y_AXIS], 3);
    Com::printF(PSTR(" Z_OFFSET:"), Motion1::g92Offsets[Z_AXIS], 3);
#if NUM_AXES > A_AXIS
    Com::printF(PSTR(" A_OFFSET:"), Motion1::g92Offsets[A_AXIS], 3);
#endif
#if NUM_AXES > B_AXIS
    Com::printF(PSTR(" B_OFFSET:"), Motion1::g92Offsets[B_AXIS], 3);
#endif
#if NUM_AXES > C_AXIS
    Com::printF(PSTR(" C_OFFSET:"), Motion1::g92Offsets[C_AXIS], 3);
#endif
    Com::println();
    // }
}

// G100 Calibrate floor or rod radius
// Using manual control, adjust hot end to contact floor.
// G100 <no arguments> No action. Avoid accidental floor reset.
// G100 [X] [Y] [Z] set floor for argument passed in. Number ignored and may be absent.
// G100 R with X Y or Z flag error, sets only floor or radius, not both.
// G100 R[n] Add n to radius. Adjust to be above floor if necessary
// G100 R[0] set radius based on current z measurement. Moves to (0,0,0)
void GCode_100(GCode* com) {
    /*
#if DRIVE_SYSTEM == DELTA
    float currentZmm = Printer::currentPosition[Z_AXIS];
    if (currentZmm / Printer::zLength > 0.1) {
        Com::printErrorFLN(PSTR("Calibration code is limited to bottom 10% of Z height"));
        break;
    }
    if (com->hasR()) {
        if (com->hasX() || com->hasY() || com->hasZ())
            Com::printErrorFLN(PSTR("Cannot set radius and floor at same time."));
        else if (com->R != 0) {
            //add r to radius
            if (abs(com->R) <= 10)
                EEPROM::incrementRodRadius(com->R);
            else
                Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
        } else {
            // auto set radius. Head must be at 0,0 and touching
            // Z offset will be corrected for.
            if (Printer::currentPosition[X_AXIS] == 0
                && Printer::currentPosition[Y_AXIS] == 0) {
                if (Printer::isLargeMachine()) {
                    // calculate radius assuming we are at surface
                    // If Z is greater than 0 it will get calculated out for correct radius
                    // Use either A or B tower as they anchor x Cartesian axis and always have
                    // Radius distance to center in simplest set up.
                    float h = Printer::deltaDiagonalStepsSquaredB.f;
                    unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
                    // The correct Rod Radius would put us here at z==0 and B height is
                    // square root (rod length squared minus rod radius squared)
                    // Reverse that to get calculated Rod Radius given B height
                    h -= RMath::sqr((float)bSteps);
                    h = sqrt(h);
                    EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
                } else {
                    // calculate radius assuming we are at surface
                    // If Z is greater than 0 it will get calculated out for correct radius
                    // Use either A or B tower as they anchor x Cartesian axis and always have
                    // Radius distance to center in simplest set up.
                    unsigned long h = Printer::deltaDiagonalStepsSquaredB.l;
                    unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
                    // The correct Rod Radius would put us here at z==0 and B height is
                    // square root (rod length squared minus rod radius squared)
                    // Reverse that to get calculated Rod Radius given B height
                    h -= RMath::sqr(bSteps);
                    h = SQRT(h);
                    EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
                }
            } else
                Com::printErrorFLN(PSTR("First move to touch at x,y=0,0 to auto-set radius."));
        }
    } else {
        bool tooBig = false;
        if (com->hasX()) {
            if (abs(com->X) <= 10)
                EEPROM::setTowerXFloor(com->X + currentZmm + Printer::xMin);
            else
                tooBig = true;
        }
        if (com->hasY()) {
            if (abs(com->Y) <= 10)
                EEPROM::setTowerYFloor(com->Y + currentZmm + Printer::yMin);
            else
                tooBig = true;
        }
        if (com->hasZ()) {
            if (abs(com->Z) <= 10)
                EEPROM::setTowerZFloor(com->Z + currentZmm + Printer::zMin);
            else
                tooBig = true;
        }
        if (tooBig)
            Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
    }
    // after adjusting zero, physical position is out of sync with memory position
    // this could cause jerky movement or push head into print surface.
    // moving gets back into safe zero'ed position with respect to newle set floor or Radius.
    Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, 12.0, IGNORE_COORDINATE, IGNORE_COORDINATE);
#endif
*/
}

void GCode_131(GCode* com) {
#if false && PRINTER_TYPE == 2
    float cx, cy, cz;
    Printer::realPosition(cx, cy, cz);
    float oldfeedrate = Printer::feedrate;
    Motion1::toolOffset[X_AXIS] = 0;
    Motion1::toolOffset[Y_AXIS] = 0;
    Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
    Printer::feedrate = oldfeedrate;
#endif
}

void GCode_132(GCode* com) {
#if false && PRINTER_TYPE == 2
// TODO: G132 not working
    // G132 Calibrate endstop offsets
    // This has the probably unintended side effect of turning off leveling.
    Motion1::setAutolevelActive(false); // don't let transformations change result!
    Printer::coordinateOffset[X_AXIS] = 0;
    Printer::coordinateOffset[Y_AXIS] = 0;
    Printer::coordinateOffset[Z_AXIS] = 0;
    // I think this is coded incorrectly, as it depends on the start position of the
    // of the hot end, and so should first move to x,y,z= 0,0,0, but as that may not
    // be possible if the printer is not in the homes/zeroed state, the printer
    // cannot safely move to 0 z coordinate without crashing into the print surface.
    // so other than commenting, I'm not meddling.
    // but you will always get different counts from different positions.
    Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
    int32_t m = RMath::max(Printer::stepsRemainingAtXHit, RMath::max(Printer::stepsRemainingAtYHit, Printer::stepsRemainingAtZHit));
    int32_t offx = m - Printer::stepsRemainingAtXHit;
    int32_t offy = m - Printer::stepsRemainingAtYHit;
    int32_t offz = m - Printer::stepsRemainingAtZHit;
    Com::printFLN(Com::tTower1, offx);
    Com::printFLN(Com::tTower2, offy);
    Com::printFLN(Com::tTower3, offz);
#if EEPROM_MODE != 0
    if (com->hasS() && com->S > 0) {
        EEPROM::setDeltaTowerXOffsetSteps(offx);
        EEPROM::setDeltaTowerYOffsetSteps(offy);
        EEPROM::setDeltaTowerZOffsetSteps(offz);
    }
#endif
    PrintLine::moveRelativeDistanceInSteps(0, 0, -5 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);
    Printer::homeAxis(true, true, true);
#endif
}

void GCode_133(GCode* com) {
#if false && PRINTER_TYPE == 2
    // G133 Measure steps to top
    bool oldAuto = Motion1::isAutolevelActive();
    Motion1::setAutolevelActive(false); // don't let transformations change result!
    Printer::currentPositionSteps[X_AXIS] = 0;
    Printer::currentPositionSteps[Y_AXIS] = 0;
    Printer::currentPositionSteps[Z_AXIS] = 0;
    Printer::coordinateOffset[X_AXIS] = 0;
    Printer::coordinateOffset[Y_AXIS] = 0;
    Printer::coordinateOffset[Z_AXIS] = 0;
    Printer::currentNonlinearPositionSteps[A_TOWER] = 0;
    Printer::currentNonlinearPositionSteps[B_TOWER] = 0;
    Printer::currentNonlinearPositionSteps[C_TOWER] = 0;
    // similar to comment above, this will get a different answer from any different starting point
    // so it is unclear how this is helpful. It must start at a well defined point.
    Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
    int32_t offx = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtXHit;
    int32_t offy = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtYHit;
    int32_t offz = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtZHit;
    Com::printFLN(Com::tTower1, offx);
    Com::printFLN(Com::tTower2, offy);
    Com::printFLN(Com::tTower3, offz);
    Motion1::setAutolevelActive(oldAuto);
    PrintLine::moveRelativeD®istanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
    Printer::homeAxis(true, true, true);
#endif
}

void GCode_134(GCode* com) {
#if FEATURE_Z_PROBE && NUM_TOOLS > 1
    // - G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!) Px = reference extruder, Sx = only measure extrude x against reference, Zx = add to measured z distance for Sx for correction.
    float z = com->hasZ() ? com->Z : 0;
    int p = com->hasP() ? com->P : 0;
    int s = com->hasS() ? com->S : -1;
    int startExtruder = Extruder::current->id;
    extruder[p].zOffset = 0;
    float mins[NUM_EXTRUDER], maxs[NUM_EXTRUDER], avg[NUM_EXTRUDER];
    for (int i = 0; i < NUM_EXTRUDER; i++) { // silence unnecessary compiler warning
        avg[i] = 0;
    }
    bool bigError = false;

#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
    float actTemp[NUM_EXTRUDER];
    for (int i = 0; i < NUM_EXTRUDER; i++)
        actTemp[i] = extruder[i].tempControl.targetTemperatureC;
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    Motion1::waitForEndOfMoves();
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
    }
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
    }
#else
    if (extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
        Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif

#ifndef G134_REPETITIONS
#define G134_REPETITIONS 3
#endif
#ifndef G134_PRECISION
#define G134_PRECISION 0.05
#endif
    Printer::startProbing(true);
    for (int r = 0; r < G134_REPETITIONS && !bigError; r++) {
        Extruder::selectExtruderById(p);
        float refHeight = Printer::runZProbe(false, false);
        if (refHeight == ILLEGAL_Z_PROBE) {
            bigError = true;
            break;
        }
        for (int i = 0; i < NUM_EXTRUDER && !bigError; i++) {
            if (i == p)
                continue;
            if (s >= 0 && i != s)
                continue;
            extruder[i].zOffset = 0;
            Extruder::selectExtruderById(i);
            float height = Printer::runZProbe(false, false);
            if (height == ILLEGAL_Z_PROBE) {
                bigError = true;
                break;
            }
            float off = (height - refHeight + z);
            if (r == 0) {
                avg[i] = mins[i] = maxs[i] = off;
            } else {
                avg[i] += off;
                if (off < mins[i])
                    mins[i] = off;
                if (off > maxs[i])
                    maxs[i] = off;
                if (maxs[i] - mins[i] > G134_PRECISION) {
                    Com::printErrorFLN(PSTR("Deviation between measurements were too big, please repeat."));
                    bigError = true;
                    break;
                }
            }
        }
    }
    if (!bigError) {
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            if (s >= 0 && i != s)
                continue;
            extruder[i].zOffset = avg[i] * Motion1::resolution[Z_AXIS] / G134_REPETITIONS;
        }
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(0);
#endif
    }
    Extruder::selectExtruderById(startExtruder);
    Printer::finishProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
#if ZHOME_HEAT_ALL
    for (int i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
    for (int i = 0; i < NUM_EXTRUDER; i++)
        Extruder::setTemperatureForExtruder(actTemp[i], i, false, actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
    Extruder::setTemperatureForExtruder(actTemp[Extruder::current->id], Extruder::current->id, false, actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
#endif
}

void GCode_135(GCode* com) {
    /* #if DRIVE_SYSTEM == DELTA
    Com::printF(PSTR("CompDelta:"), Printer::currentNonlinearPositionSteps[A_TOWER]);
    Com::printF(Com::tComma, Printer::currentNonlinearPositionSteps[B_TOWER]);
    Com::printFLN(Com::tComma, Printer::currentNonlinearPositionSteps[C_TOWER]);
#ifdef DEBUG_REAL_POSITION
    Com::printF(PSTR("RealDelta:"), Printer::realDeltaPositionSteps[A_TOWER]);
    Com::printF(Com::tComma, Printer::realDeltaPositionSteps[B_TOWER]);
    Com::printFLN(Com::tComma, Printer::realDeltaPositionSteps[C_TOWER]);
#endif
    Com::printF(PSTR("PosFromSteps:"));
    Motion1::printCurrentPosition();
    break;
#endif // DRIVE_SYSTEM
*/
}

void GCode_201(GCode* com) {
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    commandG201(*com);
#endif
}

void GCode_202(GCode* com) {
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    commandG202(*com);
#endif
}

void GCode_203(GCode* com) {
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    commandG203(*com);
#endif
}

void GCode_204(GCode* com) {
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    commandG204(*com);
#endif
}

void GCode_205(GCode* com) {
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    commandG205(*com);
#endif
}
