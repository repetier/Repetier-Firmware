#include "../../Repetier.h"

#if ZPROBE_TYPE == 1
float ZProbeHandler::getZProbeHeight() {
    return height;
}

void ZProbeHandler::activate() {
    if (activated) {
        return;
    }
    float cx, cy, cz;
    realPosition(cx, cy, cz);
    // Fix position to be inside print area when probe is enabled
#if EXTRUDER_IS_Z_PROBE == 0
    float ZPOffsetX = EEPROM::zProbeXOffset();
    float ZPOffsetY = EEPROM::zProbeYOffset();
#if DRIVE_SYSTEM == DELTA
    float rad = EEPROM::deltaMaxRadius();
    float dx = Printer::currentPosition[X_AXIS] - ZPOffsetX;
    float dy = Printer::currentPosition[Y_AXIS] - ZPOffsetY;
    if (sqrtf(dx * dx + dy * dy) > rad)
#else
    if ((ZPOffsetX > 0 && Printer::currentPosition[X_AXIS] - ZPOffsetX < Printer::xMin) || (ZPOffsetY > 0 && Printer::currentPosition[Y_AXIS] - ZPOffsetY < Printer::yMin) || (ZPOffsetX < 0 && Printer::currentPosition[X_AXIS] - ZPOffsetX > Printer::xMin + Printer::xLength) || (ZPOffsetY < 0 && Printer::currentPosition[Y_AXIS] - ZPOffsetY > Printer::yMin + Printer::yLength))
#endif
    {
        Com::printErrorF(PSTR("Activating z-probe would lead to forbidden xy position: "));
        Com::print(Printer::currentPosition[X_AXIS] - ZPOffsetX);
        Com::printFLN(PSTR(", "), Printer::currentPosition[Y_AXIS] - ZPOffsetY);
        GCode::fatalError(PSTR("Could not activate z-probe offset due to coordinate constraints - result is inaccurate!"));
        return false;
    } else {
        if (runScript) {
            GCode::executeFString(Com::tZProbeStartScript);
        }
        float maxStartHeight = EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() > 0 ? EEPROM::zProbeHeight() : 0) + 0.1;
        if (currentPosition[Z_AXIS] > maxStartHeight && enforceStartHeight) {
            cz = maxStartHeight;
            moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, maxStartHeight, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]);
        }

        // Update position
        Motion1::toolOffset[X_AXIS] = -ZPOffsetX;
        Motion1::toolOffset[Y_AXIS] = -ZPOffsetY;
        Motion1::toolOffsetZ_AXIS] = 0;
#if FEATURE_AUTOLEVEL
        // we must not change z for the probe offset even if we are rotated, so add a correction for z
        float dx, dy;
        transformToPrinter(EEPROM::zProbeXOffset(), EEPROM::zProbeYOffset(), 0, dx, dy, Motion1::zprobeZOffset);
        //Com::printFLN(PSTR("ZPOffset2:"),Motion1::zprobeZOffset,3);
#endif
    }
#else
    if (runScript) {
        GCode::executeFString(Com::tZProbeStartScript);
    }
#endif
    Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
    return true;
    activated = true;
}

void ZProbeHandler::deactivate() {
    if (!activated) {
        return;
    }
}

float ZProbeHandler::runProbe() { return 0; }

bool ZProbeHandler::probingPossible() {
    return true;
}

float ZProbeHandler::xOffset() {
    return offsetX;
}

float ZProbeHandler::yOffset() {
    return offsetX;
}

void ZProbeHandler::init() {
    eprStart = EEPROM::reserve(2, 1, EPR_Z_PROBE_MEMORY);
    activated = false;
}

void ZProbeHandler::eepromHandle() {
    PGM_T pre = PSTR("Z-probe");
    EEPROM::handlePrefix(pre);
    EEPROM::handleFloat(PSTR("trigger height [mm]"), eprStart + EPR_Z_PROBE_HEIGHT, 3, height);
    EEPROM::handlePrefix(pre);
    EEPROM::handleFloat(PSTR("min. nozzle distance [mm]"), eprStart + EPR_Z_PROBE_BED_DISTANCE, 3, bedDistance);
    EEPROM::handlePrefix(pre);
    EEPROM::handleFloat(PSTR("trigger height [mm]"), eprStart + EPR_Z_PROBE_SPEED, 3, speed);
    EEPROM::handlePrefix(pre);
    EEPROM::handleFloat(PSTR("trigger height [mm]"), eprStart + EPR_Z_PROBE_X_OFFSET, 3, offsetX);
    EEPROM::handlePrefix(pre);
    EEPROM::handleFloat(PSTR("trigger height [mm]"), eprStart + EPR_Z_PROBE_Y_OFFSET, 3, offsetY);
}
#endif
