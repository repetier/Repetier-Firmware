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

#include "../../Repetier.h"

#if PRINTER_TYPE == 2

float PrinterType::diagonal;
float PrinterType::horizontalRadius;
float PrinterType::printRadius;
float PrinterType::printRadiusSquared;
float PrinterType::angleA, PrinterType::angleB, PrinterType::angleC;
float PrinterType::correctionA, PrinterType::correctionB, PrinterType::correctionC;
float PrinterType::diagonalSquaredA;
float PrinterType::diagonalSquaredB;
float PrinterType::diagonalSquaredC;
float PrinterType::APosX, PrinterType::APosY;
float PrinterType::BPosX, PrinterType::BPosY;
float PrinterType::CPosX, PrinterType::CPosY;
uint16_t PrinterType::eeprom; // start position eeprom

void PrinterType::homeAxis(fast8_t axis) {
    if (axis < Z_AXIS) {
        return; // Detals can not home x or y
    } else if (axis > Z_AXIS) {
        Motion1::simpleHome(axis); // Non delta axis, default homing
    } else {                       // XYZ homing
    }
}

bool PrinterType::positionAllowed(float pos[NUM_AXES]) {
    if (Printer::isNoDestinationCheck()) {
        return true;
    }
    if (Printer::isHoming()) {
        return true;
    }
    if (pos[Z_AXIS] < Motion1::minPos[Z_AXIS] || pos[Z_AXIS] > Motion1::maxPos[Z_AXIS]) {
        return false;
    }
    return pos[X_AXIS] * pos[X_AXIS] + pos[Y_AXIS] * pos[Y_AXIS] <= printRadiusSquared;
}

void PrinterType::transform(float pos[NUM_AXES], int32_t motor[NUM_AXES]) {
    float z = pos[Z_AXIS];
    for (fast8_t i = E_AXIS; i < NUM_AXES; i++) {
        motor[i] = lroundf(pos[i] * Motion1::resolution[i]);
    }
    float temp = APosY - pos[Y_AXIS];
    float opt = diagonalStepsSquaredA - temp * temp;
    float temp2 = APosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        // TODO: Trigger error
        return;
    }
    motor[X_AXIS] = lroundf(sqrt(temp) + z);

    temp = BPosY - pos[Y_AXIS];
    opt = diagonalStepsSquaredB - temp * temp;
    temp2 = BPosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        return;
    }
    motor[X_AXIS] = lroundf(sqrt(temp) + z);

    temp = CPosYSteps - pos[Y_AXIS];
    opt = diagonalStepsSquaredC - temp * temp;
    temp2 = CPosX - pos[X_AXIS];
    if ((temp = opt - temp2 * temp2) < 0) {
        return;
    }
    motor[C_TOWER] = lroundf(sqrt(temp) + z);
}

void PrinterType::disableAllowedStepper() {
}

float PrinterType::accelerationForMoveSteps(fast8_t axes) {
    float acceleration = 500.0f;
    FOR_ALL_AXES(i) {
        if (axes & axisBits[i]) {
            acceleration = RMath::min(acceleration, Motion1::maxAcceleration[i]);
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

void PrinterType::deactivatedTool(fast8_t id) {}
void PrinterType::activatedTool(fast8_t id) {}
void PrinterType::eepromHandle() {
    EEPROM::handlePrefix(PSTR("Delta"));
    EEPROM::handleFloat(eeprom + 0, PSTR("Diagonal [mm]"), 2, diagonal);
    EEPROM::handleFloat(eeprom + 0, PSTR("Horizontal Radius [mm]"), 2, horizontalRadius);
    EEPROM::handleFloat(eeprom + 0, PSTR("Printable Radius [mm]"), 2, printRadius);
    EEPROM::handleFloat(eeprom + 0, PSTR("Angle A [mm]"), 2, angleA);
    EEPROM::handleFloat(eeprom + 0, PSTR("Angle B [mm]"), 2, angleB);
    EEPROM::handleFloat(eeprom + 0, PSTR("Angle C [mm]"), 2, angleC);
    EEPROM::handleFloat(eeprom + 0, PSTR("Diagonal Correction A [mm]"), 2, correctionA);
    EEPROM::handleFloat(eeprom + 0, PSTR("Diagonal Correction B [mm]"), 2, correctionB);
    EEPROM::handleFloat(eeprom + 0, PSTR("Diagonal Correction C [mm]"), 2, correctionC);
    EEPROM::removePrefix();
}
void PrinterType::init() {
    eeprom = EEPROM::reserve(5, 1, 9 * 4);
}
void PrinterType::updateDerived() {
    float radiusA = horizontalRadius; // + EEPROM::deltaRadiusCorrectionA();
    float radiusB = horizontalRadius; // + EEPROM::deltaRadiusCorrectionB();
    float radiusC = horizontalRadius; // + EEPROM::deltaRadiusCorrectionC();
    APosX = radiusA * cos(angleA * M_PI / 180.0f);
    APosY = radiusA * sin(angleA * M_PI / 180.0f);
    BPosX = radiusB * cos(angleB * M_PI / 180.0f);
    BPosY = radiusB * sin(angleB * M_PI / 180.0f);
    CPosX = radiusC * cos(angleC * M_PI / 180.0f);
    CPosY = radiusC * sin(angleC * M_PI / 180.0f);
    diagonalSquaredA = RMath::sqr(diagonal + correctionA);
    diagonalSquaredB = RMath::sqr(diagonal + correctionA);
    diagonalSquaredC = RMath::sqr(diagonal + correctionA);
    printRadiusSquared = printRadius * printRadius;
}

#endif
