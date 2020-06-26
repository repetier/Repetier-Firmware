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

#if USE_ADVANCE
ufast8_t Printer::maxExtruderSpeed;        ///< Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//uint8_t Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float Printer::axisStepsPerMM[E_AXIS_ARRAY] = { XAXIS_STEPS_PER_MM, YAXIS_STEPS_PER_MM, ZAXIS_STEPS_PER_MM, 1 }; ///< Number of steps per mm needed.
float Printer::invAxisStepsPerMM[E_AXIS_ARRAY];                                                                  ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate[E_AXIS_ARRAY] = { MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z };                   ///< Maximum allowed feedrate.
float Printer::homingFeedrate[Z_AXIS_ARRAY] = { HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z };
#if DUAL_X_RESOLUTION
float Printer::axisX1StepsPerMM = XAXIS_STEPS_PER_MM;
float Printer::axisX2StepsPerMM = X2AXIS_STEPS_PER_MM;
#endif
#if DUAL_X_AXIS_MODE > 0 && LAZY_DUAL_X_AXIS == 0
float Printer::x1Length;
float Printer::x1Min;
#endif
#if RAMP_ACCELERATION
//  float max_start_speed_units_per_second[E_AXIS_ARRAY] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
float Printer::maxAccelerationMMPerSquareSecond[E_AXIS_ARRAY] = { MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z };                            ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS_ARRAY] = { MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z }; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS_ARRAY];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS_ARRAY];
// uint32_t Printer::maxInterval;
#endif
#if NONLINEAR_SYSTEM
long Printer::currentNonlinearPositionSteps[E_TOWER_ARRAY];
uint8_t lastMoveID = 0; // Last move ID
#endif
#if DRIVE_SYSTEM != DELTA
int32_t Printer::zCorrectionStepsIncluded = 0;
#endif
#if FEATURE_BABYSTEPPING
int16_t Printer::zBabystepsMissing = 0;
int16_t Printer::zBabysteps = 0;
#endif
uint8_t Printer::relativeCoordinateMode = false;         ///< Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false; ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

int32_t Printer::currentPositionSteps[E_AXIS_ARRAY];
float Printer::currentPosition[E_AXIS_ARRAY];
float Printer::destinationPositionTransformed[E_AXIS_ARRAY];
float Printer::currentPositionTransformed[E_AXIS_ARRAY];
float Printer::lastCmdPos[Z_AXIS_ARRAY];
int32_t Printer::destinationSteps[E_AXIS_ARRAY];
float Printer::coordinateOffset[Z_AXIS_ARRAY] = { 0, 0, 0 };
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
fast8_t Printer::stepsPerTimerCall = 1;
uint16_t Printer::menuMode = 0;
uint8_t Printer::mode = DEFAULT_PRINTER_MODE;
uint8_t Printer::fanSpeed = 0; // Last fan speed set with M106/M107
float Printer::extrudeMultiplyError = 0;
float Printer::extrusionFactor = 1.0;
uint8_t Printer::interruptEvent = 0;
fast8_t Printer::breakLongCommand = false;
int Printer::currentLayer = 0;
int Printer::maxLayer = -1;       // -1 = unknown
char Printer::printName[21] = ""; // max. 20 chars + 0
float Printer::progress = 0;
millis_t Printer::lastTempReport = 0;

#if EEPROM_MODE != 0
float Printer::zBedOffset = HAL::eprGetFloat(EPR_Z_PROBE_Z_OFFSET);
#else
float Printer::zBedOffset = Z_PROBE_Z_OFFSET;
#endif
#if FEATURE_AUTOLEVEL
float Printer::autolevelTransformation[9]; ///< Transformation matrix
#endif
uint32_t Printer::interval = 30000; ///< Last step duration in ticks.
uint32_t Printer::timer;            ///< used for acceleration/deceleration timing
uint32_t Printer::stepNumber;       ///< Step number in current move.
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
int32_t Printer::advanceExecuted; ///< Executed advance steps
#endif
int Printer::advanceStepsSet;
#endif
#if NONLINEAR_SYSTEM
int32_t Printer::maxDeltaPositionSteps;
floatLong Printer::deltaDiagonalStepsSquaredA;
floatLong Printer::deltaDiagonalStepsSquaredB;
floatLong Printer::deltaDiagonalStepsSquaredC;
float Printer::deltaMaxRadiusSquared;
float Printer::radius0;
int32_t Printer::deltaFloorSafetyMarginSteps = 0;
int32_t Printer::deltaAPosXSteps;
int32_t Printer::deltaAPosYSteps;
int32_t Printer::deltaBPosXSteps;
int32_t Printer::deltaBPosYSteps;
int32_t Printer::deltaCPosXSteps;
int32_t Printer::deltaCPosYSteps;
int32_t Printer::realDeltaPositionSteps[TOWER_ARRAY];
int16_t Printer::travelMovesPerSecond;
int16_t Printer::printMovesPerSecond;
#endif
#if !NONLINEAR_SYSTEM || defined(FAST_COREXYZ)
int32_t Printer::xMinStepsAdj, Printer::yMinStepsAdj, Printer::zMinStepsAdj; // adjusted to cover extruder/probe offsets
int32_t Printer::xMaxStepsAdj, Printer::yMaxStepsAdj, Printer::zMaxStepsAdj;
#endif
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z || NONLINEAR_SYSTEM
int32_t Printer::stepsRemainingAtZHit;
#endif
#if DRIVE_SYSTEM == DELTA
int32_t Printer::stepsRemainingAtXHit;
int32_t Printer::stepsRemainingAtYHit;
#endif
#if SOFTWARE_LEVELING
int32_t Printer::levelingP1[3];
int32_t Printer::levelingP2[3];
int32_t Printer::levelingP3[3];
#endif
//float Printer::minimumSpeed;               ///< lowest allowed speed to keep integration error small
//float Printer::minimumZSpeed;
int32_t Printer::xMaxSteps; ///< For software endstops, limit of move in positive direction.
int32_t Printer::yMaxSteps; ///< For software endstops, limit of move in positive direction.
int32_t Printer::zMaxSteps; ///< For software endstops, limit of move in positive direction.
int32_t Printer::xMinSteps; ///< For software endstops, limit of move in negative direction.
int32_t Printer::yMinSteps; ///< For software endstops, limit of move in negative direction.
int32_t Printer::zMinSteps; ///< For software endstops, limit of move in negative direction.
float Printer::xLength;
float Printer::xMin;
float Printer::yLength;
float Printer::yMin;
float Printer::zLength;
float Printer::zMin;
float Printer::feedrate;               ///< Last requested feedrate.
int Printer::feedrateMultiply;         ///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int Printer::extrudeMultiply; ///< Flow multiplier in percent (factor 1 = 100)
float Printer::maxJerk;                ///< Maximum allowed jerk in mm/s
#if DRIVE_SYSTEM != DELTA
float Printer::maxZJerk; ///< Maximum allowed jerk in z direction in mm/s
#endif
float Printer::offsetX;             ///< X-offset for different extruder positions.
float Printer::offsetY;             ///< Y-offset for different extruder positions.
float Printer::offsetZ;             ///< Z-offset for different extruder positions.
float Printer::offsetZ2 = 0;        ///< Z-offset without rotation correction.
speed_t Printer::vMaxReached;       ///< Maximum reached speed
uint32_t Printer::msecondsPrinting; ///< Milliseconds of printing time (means time with heated extruder)
float Printer::filamentPrinted;     ///< mm of filament printed since counting started
#if ENABLE_BACKLASH_COMPENSATION
float Printer::backlashX;
float Printer::backlashY;
float Printer::backlashZ;
uint8_t Printer::backlashDir;
#endif
float Printer::memoryX = IGNORE_COORDINATE;
float Printer::memoryY = IGNORE_COORDINATE;
float Printer::memoryZ = IGNORE_COORDINATE;
float Printer::memoryE = IGNORE_COORDINATE;
float Printer::memoryF = -1;
#if GANTRY && !defined(FAST_COREXYZ)
int8_t Printer::motorX;
int8_t Printer::motorYorZ;
#endif
#if FAN_THERMO_PIN > -1
float Printer::thermoMinTemp = FAN_THERMO_MIN_TEMP;
float Printer::thermoMaxTemp = FAN_THERMO_MAX_TEMP;
#endif
#ifdef DEBUG_SEGMENT_LENGTH
float Printer::maxRealSegmentLength = 0;
#endif
#ifdef DEBUG_REAL_JERK
float Printer::maxRealJerk = 0;
#endif
#if MULTI_XENDSTOP_HOMING
fast8_t Printer::multiXHomeFlags; // 1 = move X0, 2 = move X1
#endif
#if MULTI_YENDSTOP_HOMING
fast8_t Printer::multiYHomeFlags; // 1 = move Y0, 2 = move Y1
#endif
#if MULTI_ZENDSTOP_HOMING
fast8_t Printer::multiZHomeFlags; // 1 = move Z0, 2 = move Z1
#endif
#if CASE_LIGHTS_PIN > -1
fast8_t Printer::lightOn;
#endif
#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif
#if LAZY_DUAL_X_AXIS
bool Printer::sledParked = false;
#endif
fast8_t Printer::wizardStackPos;
wizardVar Printer::wizardStack[WIZARD_STACK_SIZE];
fast8_t Printer::safetyParked = 0; /// True if moved to a safety position to protect print
uint8_t Printer::rescueOn = false;
bool Printer::failedMode = false;

#if defined(DRV_TMC2130)
#if TMC2130_ON_X
TMC2130Stepper* Printer::tmc_driver_x = NULL;
#endif
#if TMC2130_ON_Y
TMC2130Stepper* Printer::tmc_driver_y = NULL;
#endif
#if TMC2130_ON_Z
TMC2130Stepper* Printer::tmc_driver_z = NULL;
#endif
#if TMC2130_ON_EXT0
TMC2130Stepper* Printer::tmc_driver_e0 = NULL;
#endif
#if TMC2130_ON_EXT1
TMC2130Stepper* Printer::tmc_driver_e1 = NULL;
#endif
#if TMC2130_ON_EXT2
TMC2130Stepper* Printer::tmc_driver_e2 = NULL;
#endif
#if TMC2130_ON_EXT3
TMC2130Stepper* Printer::tmc_driver_e3 = NULL;
#endif
#if TMC2130_ON_EXT4
TMC2130Stepper* Printer::tmc_driver_e4 = NULL;
#endif
#endif

#if !NONLINEAR_SYSTEM
void Printer::constrainDestinationCoords() {
    if (isNoDestinationCheck() || isHoming())
        return;
#if min_software_endstop_x
    if (destinationSteps[X_AXIS] < xMinStepsAdj)
        destinationSteps[X_AXIS] = xMinStepsAdj;
#endif
#if min_software_endstop_y
    if (destinationSteps[Y_AXIS] < yMinStepsAdj)
        destinationSteps[Y_AXIS] = yMinStepsAdj;
#endif
#if min_software_endstop_z
    if (isAutolevelActive() == false && destinationSteps[Z_AXIS] < zMinStepsAdj && !isZProbingActive())
        destinationSteps[Z_AXIS] = zMinStepsAdj;
#endif

#if max_software_endstop_x
    if (destinationSteps[X_AXIS] > xMaxStepsAdj)
        destinationSteps[X_AXIS] = xMaxStepsAdj;
#endif
#if max_software_endstop_y
    if (destinationSteps[Y_AXIS] > yMaxStepsAdj)
        destinationSteps[Y_AXIS] = yMaxStepsAdj;
#endif
#if max_software_endstop_z
    if (isAutolevelActive() == false && destinationSteps[Z_AXIS] > zMaxStepsAdj && !isZProbingActive())
        destinationSteps[Z_AXIS] = zMaxStepsAdj;
#endif
    EVENT_CONTRAIN_DESTINATION_COORDINATES
}
#endif

void Printer::setDebugLevel(uint8_t newLevel) {
    if (newLevel != debugLevel) {
        debugLevel = newLevel;
        if (debugDryrun()) {
            // Disable all heaters in case they were on
            Extruder::disableAllHeater();
        }
    }
    Com::printFLN(PSTR("DebugLevel:"), (int)newLevel);
}

void Printer::toggleEcho() {
    setDebugLevel(debugLevel ^ 1);
}

void Printer::toggleInfo() {
    setDebugLevel(debugLevel ^ 2);
}

void Printer::toggleErrors() {
    setDebugLevel(debugLevel ^ 4);
}

void Printer::toggleDryRun() {
    setDebugLevel(debugLevel ^ 8);
}

void Printer::toggleCommunication() {
    setDebugLevel(debugLevel ^ 16);
}

void Printer::toggleNoMoves() {
    setDebugLevel(debugLevel ^ 32);
}

void Printer::toggleEndStop() {
    setDebugLevel(debugLevel ^ 64);
}

bool Printer::isPositionAllowed(float x, float y, float z) {
    if (isNoDestinationCheck())
        return true;
    bool allowed = true;
#if DRIVE_SYSTEM == DELTA
    if (!isHoming()) {
        allowed = allowed && (z >= 0) && (z <= zLength + 0.05 + ENDSTOP_Z_BACK_ON_HOME);
        allowed = allowed && (x * x + y * y <= deltaMaxRadiusSquared);
    }
#else // DRIVE_SYSTEM
    if (!isHoming()) {
        allowed = allowed && x >= xMin - 0.01;
        allowed = allowed && x <= xMin + xLength + 0.01;
        allowed = allowed && y >= yMin - 0.01;
        allowed = allowed && y <= yMin + yLength + 0.01;
        allowed = allowed && z >= zMin - 0.01;
        allowed = allowed && z <= zMin + zLength + ENDSTOP_Z_BACK_ON_HOME + 0.01;
    }
#endif
    /*#if DUAL_X_AXIS
        // Prevent carriage hit by disallowing moves inside other parking direction.
        if(Extruder::current->id == 0) {
            if(x > xMin + xLength + 0.01)
                allowed = false;
        } else {
            if(x < xMin - 0.01)
                allowed = false;
        }
    #endif*/
    if (!allowed) {
        Printer::updateCurrentPosition(true);
        Commands::printCurrentPosition();
    }
    return allowed;
}

void Printer::setFanSpeedDirectly(uint8_t speed) {
    uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    if (pwm_pos[PWM_FAN1] == trimmedSpeed)
        return;
#if FAN_KICKSTART_TIME
    if (fanKickstart == 0 && speed > pwm_pos[PWM_FAN1] && speed < 85) {
        if (pwm_pos[PWM_FAN1])
            fanKickstart = FAN_KICKSTART_TIME / 100;
        else
            fanKickstart = FAN_KICKSTART_TIME / 25;
    }
#endif
    pwm_pos[PWM_FAN1] = trimmedSpeed;
#endif
}
void Printer::setFan2SpeedDirectly(uint8_t speed) {
    uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
    if (pwm_pos[PWM_FAN2] == trimmedSpeed)
        return;
#if FAN_KICKSTART_TIME
    if (fan2Kickstart == 0 && speed > pwm_pos[PWM_FAN2] && speed < 85) {
        if (pwm_pos[PWM_FAN2])
            fan2Kickstart = FAN_KICKSTART_TIME / 100;
        else
            fan2Kickstart = FAN_KICKSTART_TIME / 25;
    }
#endif
    pwm_pos[PWM_FAN2] = trimmedSpeed;
#endif
}

bool Printer::updateDoorOpen() {
#if defined(DOOR_PIN) && DOOR_PIN > -1 //  && SUPPORT_LASER should always be respected
    bool isOpen = isDoorOpen();
    uint8_t b = READ(DOOR_PIN) != DOOR_INVERTING;
    if (b && !isOpen) {
        UI_STATUS_F(Com::tSpace);
        bool all = Com::writeToAll;
        Com::writeToAll = true;
        Com::printFLN(PSTR("DoorOpened"));
        Com::writeToAll = all;
    } else if (!b && isOpen) {
        bool all = Com::writeToAll;
        Com::writeToAll = true;
        Com::printFLN(PSTR("DoorClosed"));
        Com::writeToAll = all;
        UI_STATUS_F(Com::tDoorOpen);
    }
    flag3 = (b ? flag3 | PRINTER_FLAG3_DOOR_OPEN : flag3 & ~PRINTER_FLAG3_DOOR_OPEN);
    return b;
#else
    return 0;
#endif
}

void Printer::reportPrinterMode() {
    Printer::setMenuMode(MENU_MODE_CNC + MENU_MODE_LASER + MENU_MODE_FDM, false);
    switch (Printer::mode) {
    case PRINTER_MODE_FFF:
        Printer::setMenuMode(MENU_MODE_FDM, true);
        Com::printFLN(Com::tPrinterModeFFF);
        break;
    case PRINTER_MODE_LASER:
        Printer::setMenuMode(MENU_MODE_LASER, true);
        Com::printFLN(Com::tPrinterModeLaser);
        break;
    case PRINTER_MODE_CNC:
        Printer::setMenuMode(MENU_MODE_CNC, true);
        Com::printFLN(Com::tPrinterModeCNC);
        break;
    }
}
void Printer::updateDerivedParameter() {
#if NONLINEAR_SYSTEM
    travelMovesPerSecond = EEPROM::deltaSegmentsPerSecondMove();
    printMovesPerSecond = EEPROM::deltaSegmentsPerSecondPrint();
    if (travelMovesPerSecond < 15)
        travelMovesPerSecond = 15; // lower values make no sense and can cause serious problems
    if (printMovesPerSecond < 15)
        printMovesPerSecond = 15;
#endif
#if DRIVE_SYSTEM == DELTA
    axisStepsPerMM[X_AXIS] = axisStepsPerMM[Y_AXIS] = axisStepsPerMM[Z_AXIS];
    maxAccelerationMMPerSquareSecond[X_AXIS] = maxAccelerationMMPerSquareSecond[Y_AXIS] = maxAccelerationMMPerSquareSecond[Z_AXIS];
    homingFeedrate[X_AXIS] = homingFeedrate[Y_AXIS] = homingFeedrate[Z_AXIS];
    maxFeedrate[X_AXIS] = maxFeedrate[Y_AXIS] = maxFeedrate[Z_AXIS];
    maxTravelAccelerationMMPerSquareSecond[X_AXIS] = maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = maxTravelAccelerationMMPerSquareSecond[Z_AXIS];
    zMaxSteps = axisStepsPerMM[Z_AXIS] * (zLength);
    towerAMinSteps = axisStepsPerMM[A_TOWER] * xMin;
    towerBMinSteps = axisStepsPerMM[B_TOWER] * yMin;
    towerCMinSteps = axisStepsPerMM[C_TOWER] * zMin;
    //radius0 = EEPROM::deltaHorizontalRadius();
    float radiusA = radius0 + EEPROM::deltaRadiusCorrectionA();
    float radiusB = radius0 + EEPROM::deltaRadiusCorrectionB();
    float radiusC = radius0 + EEPROM::deltaRadiusCorrectionC();
    deltaAPosXSteps = floor(radiusA * cos(EEPROM::deltaAlphaA() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaAPosYSteps = floor(radiusA * sin(EEPROM::deltaAlphaA() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaBPosXSteps = floor(radiusB * cos(EEPROM::deltaAlphaB() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaBPosYSteps = floor(radiusB * sin(EEPROM::deltaAlphaB() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaCPosXSteps = floor(radiusC * cos(EEPROM::deltaAlphaC() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaCPosYSteps = floor(radiusC * sin(EEPROM::deltaAlphaC() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
    deltaDiagonalStepsSquaredA.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionA() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
    deltaDiagonalStepsSquaredB.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionB() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
    deltaDiagonalStepsSquaredC.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionC() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
    if (deltaDiagonalStepsSquaredA.l > 65534 || 2 * radius0 * axisStepsPerMM[Z_AXIS] > 65534) {
        setLargeMachine(true);
#ifdef SUPPORT_64_BIT_MATH
        deltaDiagonalStepsSquaredA.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredA.l));
        deltaDiagonalStepsSquaredB.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredB.l));
        deltaDiagonalStepsSquaredC.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredC.l));
#else
        deltaDiagonalStepsSquaredA.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredA.l));
        deltaDiagonalStepsSquaredB.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredB.l));
        deltaDiagonalStepsSquaredC.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredC.l));
#endif
    } else {
        setLargeMachine(false);
        deltaDiagonalStepsSquaredA.l = RMath::sqr(deltaDiagonalStepsSquaredA.l);
        deltaDiagonalStepsSquaredB.l = RMath::sqr(deltaDiagonalStepsSquaredB.l);
        deltaDiagonalStepsSquaredC.l = RMath::sqr(deltaDiagonalStepsSquaredC.l);
    }
    deltaMaxRadiusSquared = RMath::sqr(EEPROM::deltaMaxRadius());
    long cart[Z_AXIS_ARRAY], delta[TOWER_ARRAY];
    cart[X_AXIS] = cart[Y_AXIS] = 0;
    cart[Z_AXIS] = zMaxSteps;
    transformCartesianStepsToDeltaSteps(cart, delta);
    maxDeltaPositionSteps = delta[0];
    xMaxSteps = yMaxSteps = zMaxSteps;
    xMinSteps = yMinSteps = zMinSteps = 0;
    deltaFloorSafetyMarginSteps = DELTA_FLOOR_SAFETY_MARGIN_MM * axisStepsPerMM[Z_AXIS];
#elif DRIVE_SYSTEM == TUGA
    deltaDiagonalStepsSquared.l = uint32_t(EEPROM::deltaDiagonalRodLength() * axisStepsPerMM[X_AXIS]);
    if (deltaDiagonalStepsSquared.l > 65534) {
        setLargeMachine(true);
        deltaDiagonalStepsSquared.f = float(deltaDiagonalStepsSquared.l) * float(deltaDiagonalStepsSquared.l);
    } else
        deltaDiagonalStepsSquared.l = deltaDiagonalStepsSquared.l * deltaDiagonalStepsSquared.l;
    deltaBPosXSteps = static_cast<int32_t>(EEPROM::deltaDiagonalRodLength() * axisStepsPerMM[X_AXIS]);
    xMaxSteps = static_cast<int32_t>(axisStepsPerMM[X_AXIS] * (xMin + xLength));
    yMaxSteps = static_cast<int32_t>(axisStepsPerMM[Y_AXIS] * yLength);
    zMaxSteps = static_cast<int32_t>(axisStepsPerMM[Z_AXIS] * (zMin + zLength));
    xMinSteps = static_cast<int32_t>(axisStepsPerMM[X_AXIS] * xMin);
    yMinSteps = 0;
    zMinSteps = static_cast<int32_t>(axisStepsPerMM[Z_AXIS] * zMin);
#else
#if DUAL_X_RESOLUTION
    if (Extruder::current->id == 0) // adjust resolution based on active extruder
        axisStepsPerMM[X_AXIS] = axisX1StepsPerMM;
    else
        axisStepsPerMM[X_AXIS] = axisX2StepsPerMM;
    invAxisStepsPerMM[X_AXIS] = 1.0 / axisStepsPerMM[X_AXIS];
#endif
    xMaxStepsAdj = xMaxSteps = static_cast<int32_t>(axisStepsPerMM[X_AXIS] * (xMin + xLength));
    yMaxStepsAdj = yMaxSteps = static_cast<int32_t>(axisStepsPerMM[Y_AXIS] * (yMin + yLength));
    zMaxStepsAdj = zMaxSteps = static_cast<int32_t>(axisStepsPerMM[Z_AXIS] * (zMin + zLength));
    xMinStepsAdj = xMinSteps = static_cast<int32_t>(axisStepsPerMM[X_AXIS] * xMin);
    yMinStepsAdj = yMinSteps = static_cast<int32_t>(axisStepsPerMM[Y_AXIS] * yMin);
    zMinStepsAdj = zMinSteps = static_cast<int32_t>(axisStepsPerMM[Z_AXIS] * zMin);
    for (fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        Extruder& e = extruder[i];
        xMaxStepsAdj = RMath::max(xMaxStepsAdj, xMaxSteps - e.xOffset);
        xMinStepsAdj = RMath::min(xMinStepsAdj, xMinSteps - e.xOffset);
        yMaxStepsAdj = RMath::max(yMaxStepsAdj, yMaxSteps - e.yOffset);
        yMinStepsAdj = RMath::min(yMinStepsAdj, yMinSteps - e.yOffset);
        zMaxStepsAdj = RMath::max(zMaxStepsAdj, zMaxSteps - e.zOffset);
        zMinStepsAdj = RMath::min(zMinStepsAdj, zMinSteps - e.zOffset);
    }
#if FEATURE_Z_PROBE
    xMaxStepsAdj = RMath::max(xMaxStepsAdj, xMaxSteps - static_cast<int32_t>(EEPROM::zProbeXOffset() * axisStepsPerMM[X_AXIS]));
    xMinStepsAdj = RMath::min(xMinStepsAdj, xMinSteps - static_cast<int32_t>(EEPROM::zProbeXOffset() * axisStepsPerMM[X_AXIS]));
    yMaxStepsAdj = RMath::max(yMaxStepsAdj, yMaxSteps - static_cast<int32_t>(EEPROM::zProbeYOffset() * axisStepsPerMM[Y_AXIS]));
    yMinStepsAdj = RMath::min(yMinStepsAdj, yMinSteps - static_cast<int32_t>(EEPROM::zProbeYOffset() * axisStepsPerMM[Y_AXIS]));
#endif
    // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    backlashDir &= XYZ_DIRPOS;
    if (backlashX != 0)
        backlashDir |= 8;
    if (backlashY != 0)
        backlashDir |= 16;
    if (backlashZ != 0)
        backlashDir |= 32;
#endif
#endif
    for (uint8_t i = 0; i < E_AXIS_ARRAY; i++) {
        invAxisStepsPerMM[i] = 1.0f / axisStepsPerMM[i];
#ifdef RAMP_ACCELERATION
        /** Acceleration in steps/s^3 in printing mode.*/
        maxPrintAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
        /** Acceleration in steps/s^2 in movement mode.*/
        maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif
    }
    // For numeric stability we need to start accelerations at a minimum speed and hence ensure that the
    // jerk is at least 2 * minimum speed.

    // For xy moves the minimum speed is multiplied with 1.41 to enforce the condition also for diagonals since the
    // driving axis is the problematic speed.
    float accel = RMath::max(maxAccelerationMMPerSquareSecond[X_AXIS], maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    float minimumSpeed = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[X_AXIS] * accel));
    accel = RMath::max(maxAccelerationMMPerSquareSecond[Y_AXIS], maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
    float minimumSpeed2 = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[Y_AXIS] * accel));
    if (minimumSpeed2 > minimumSpeed) {
        minimumSpeed = minimumSpeed2;
    }
    if (maxJerk < 2 * minimumSpeed) { // Enforce minimum start speed if target is faster and jerk too low
        maxJerk = 2 * minimumSpeed;
        Com::printFLN(PSTR("XY jerk was too low, setting to "), maxJerk);
    }
    accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS], maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#if DRIVE_SYSTEM != DELTA
    float minimumZSpeed = 0.5 * accel * sqrt(2.0f / (axisStepsPerMM[Z_AXIS] * accel));
    if (maxZJerk < 2 * minimumZSpeed) {
        maxZJerk = 2 * minimumZSpeed;
        Com::printFLN(PSTR("Z jerk was too low, setting to "), maxZJerk);
    }
#endif
    /*
    maxInterval = F_CPU / (minimumSpeed * axisStepsPerMM[X_AXIS]);
    uint32_t tmp = F_CPU / (minimumSpeed * axisStepsPerMM[Y_AXIS]);
    if(tmp < maxInterval)
        maxInterval = tmp;
#if DRIVE_SYSTEM != DELTA
    tmp = F_CPU / (minimumZSpeed * axisStepsPerMM[Z_AXIS]);
    if(tmp < maxInterval)
        maxInterval = tmp;
#endif
*/
    //Com::printFLN(PSTR("Minimum Speed:"),minimumSpeed);
    //Com::printFLN(PSTR("Minimum Speed Z:"),minimumZSpeed);
#if DISTORTION_CORRECTION
    distortion.updateDerived();
#endif // DISTORTION_CORRECTION
    Printer::updateAdvanceFlags();
    EVENT_UPDATE_DERIVED;
}
#if AUTOMATIC_POWERUP
void Printer::enablePowerIfNeeded() {
    if (Printer::isPowerOn())
        return;
    SET_OUTPUT(PS_ON_PIN); //GND
    Printer::setPowerOn(true);
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
    HAL::delayMilliseconds(500); // Just to ensure power is up and stable
}
#endif

/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void Printer::kill(uint8_t onlySteppers) {
    EVENT_KILL(onlySteppers);
    if (areAllSteppersDisabled() && onlySteppers)
        return;
    if (Printer::isAllKilled())
        return;
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    disableAllMotorDrivers();
#endif // defined
    disableXStepper();
    disableYStepper();
#if !defined(PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT)
    disableZStepper();
#else
    if (!onlySteppers)
        disableZStepper();
#endif
    Extruder::disableAllExtruderMotors();
    setAllSteppersDiabled();
    unsetHomedAll();
    if (!onlySteppers) {
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
            Extruder::setTemperatureForExtruder(0, i);
        Extruder::setHeatedBedTemperature(0);
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_STANDBY_ID));
#if defined(PS_ON_PIN) && PS_ON_PIN > -1 && !defined(NO_POWER_TIMEOUT)
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
        Printer::setPowerOn(false);
#endif
        Printer::setAllKilled(true);
    } else
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_STEPPER_DISABLED_ID));
#if FAN_BOARD_PIN > -1
#if HAVE_HEATED_BED
    if (heatedBedController.targetTemperatureC < 15) // turn off FAN_BOARD only if bed heater is off
#endif
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif // FAN_BOARD_PIN
    Commands::printTemperatures(false);
}

void Printer::updateAdvanceFlags() {
    Printer::setAdvanceActivated(false);
#if USE_ADVANCE
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].advanceL != 0) {
            Printer::setAdvanceActivated(true);
        }
#if ENABLE_QUADRATIC_ADVANCE
        if (extruder[i].advanceK != 0)
            Printer::setAdvanceActivated(true);
#endif
    }
#endif
}

void Printer::moveToParkPosition(bool zOnly) {
    if (Printer::isHomedAll()) { // for safety move only when homed!
        if (!zOnly) {
            moveToReal(EEPROM::parkX(), EEPROM::parkY(), IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS], true);
        }
        moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::min(zMin + zLength, currentPosition[Z_AXIS] + EEPROM::parkZ()), IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS], true);
    }
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
uint8_t Printer::moveTo(float x, float y, float z, float e, float f) {
    Printer::unparkSafety();
    if (x != IGNORE_COORDINATE) {
        destinationPositionTransformed[X_AXIS] = (x + Printer::offsetX);
        destinationSteps[X_AXIS] = destinationPositionTransformed[X_AXIS] * axisStepsPerMM[X_AXIS];
    } else {
        destinationPositionTransformed[X_AXIS] = currentPositionTransformed[X_AXIS];
        destinationSteps[X_AXIS] = currentPositionSteps[X_AXIS];
    }
    if (y != IGNORE_COORDINATE) {
        destinationPositionTransformed[Y_AXIS] = (y + Printer::offsetY);
        destinationSteps[Y_AXIS] = destinationPositionTransformed[Y_AXIS] * axisStepsPerMM[Y_AXIS];
    } else {
        destinationPositionTransformed[Y_AXIS] = currentPositionTransformed[Y_AXIS];
        destinationSteps[Y_AXIS] = currentPositionSteps[Y_AXIS];
    }
    if (z != IGNORE_COORDINATE) {
        destinationPositionTransformed[Z_AXIS] = (z + Printer::offsetZ);
        destinationSteps[Z_AXIS] = destinationPositionTransformed[Z_AXIS] * axisStepsPerMM[Z_AXIS];
    } else {
        destinationPositionTransformed[Z_AXIS] = currentPositionTransformed[Z_AXIS];
        destinationSteps[Z_AXIS] = currentPositionSteps[Z_AXIS];
    }
    if (e != IGNORE_COORDINATE) {
        destinationPositionTransformed[E_AXIS] = e;
        destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    } else {
        destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS];
        destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];
    }
    if (f != IGNORE_COORDINATE) {
        feedrate = f;
    }
#if NONLINEAR_SYSTEM
    // Disable software end stop or we get wrong distances when length < real length
    if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, false)) {
        Com::printWarningFLN(PSTR("moveTo / queueDeltaMove returns error"));
        return 0;
    }
#else
    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
#endif
    updateCurrentPosition(false);
    return 1;
}

void Printer::moveToCenter() {
#if DRIVE_SYSTEM == DELTA
    moveToReal(0, 0, IGNORE_COORDINATE, homingFeedrate[Z_AXIS], false);
#else
    moveToReal(xMin + 0.5 * xLength, yMin + 0.5 * yLength, IGNORE_COORDINATE, homingFeedrate[X_AXIS], false);
#endif
}

uint8_t Printer::moveToReal(float x, float y, float z, float e, float f, bool pathOptimize) {
    Printer::unparkSafety();
    // Com::printFLN(PSTR("MoveToReal X="),x,2);
    // Com::printArrayFLN(PSTR("CurPos:"), currentPositionTransformed);
    if (x == IGNORE_COORDINATE)
        x = currentPosition[X_AXIS];
    currentPosition[X_AXIS] = x;
    if (y == IGNORE_COORDINATE)
        y = currentPosition[Y_AXIS];
    currentPosition[Y_AXIS] = y;
    if (z == IGNORE_COORDINATE)
        z = currentPosition[Z_AXIS];
    currentPosition[Z_AXIS] = z;
    transformToPrinter(x + Printer::offsetX, y + Printer::offsetY, z + Printer::offsetZ, destinationPositionTransformed[X_AXIS], destinationPositionTransformed[Y_AXIS], destinationPositionTransformed[Z_AXIS]);
    destinationPositionTransformed[Z_AXIS] += offsetZ2;
    // There was conflicting use of IGNOR_COORDINATE
    destinationSteps[X_AXIS] = lroundf(destinationPositionTransformed[X_AXIS] * axisStepsPerMM[X_AXIS]);
    destinationSteps[Y_AXIS] = lroundf(destinationPositionTransformed[Y_AXIS] * axisStepsPerMM[Y_AXIS]);
    destinationSteps[Z_AXIS] = lroundf(destinationPositionTransformed[Z_AXIS] * axisStepsPerMM[Z_AXIS]);
    if (e != IGNORE_COORDINATE && !Printer::debugDryrun()
#if MIN_EXTRUDER_TEMP > 30
        && (Extruder::current->tempControl.currentTemperatureC > MIN_EXTRUDER_TEMP || Printer::isColdExtrusionAllowed() || Extruder::current->tempControl.sensorType == 0)
#endif
    ) {
        destinationPositionTransformed[E_AXIS] = e;
        destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    } else {
        destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS];
        destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];
    }
    if (f != IGNORE_COORDINATE)
        feedrate = f;
        // Com::printArrayFLN(PSTR("DestPos:"), destinationPositionTransformed);

#if NONLINEAR_SYSTEM
    if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, pathOptimize, true)) {
        Com::printWarningFLN(PSTR("moveToReal / queueDeltaMove returns error"));
        SHOWM(x);
        SHOWM(y);
        SHOWM(z);
        return 0;
    }
#else
    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, pathOptimize);
#endif
    return 1;
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
    coordinateOffset[X_AXIS] = xOff; // offset from G92
    coordinateOffset[Y_AXIS] = yOff;
    coordinateOffset[Z_AXIS] = zOff;
}

/** Computes currentPosition from currentPositionSteps including correction for offset. */
void Printer::updateCurrentPosition(bool copyLastCmd) {
#if DUAL_X_AXIS && LAZY_DUAL_X_AXIS
    if (!sledParked)
        currentPositionTransformed[X_AXIS] = static_cast<float>(currentPositionSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#else
    currentPositionTransformed[X_AXIS] = static_cast<float>(currentPositionSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#endif
    currentPositionTransformed[Y_AXIS] = static_cast<float>(currentPositionSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
#if NONLINEAR_SYSTEM
    currentPositionTransformed[Z_AXIS] = static_cast<float>(currentPositionSteps[Z_AXIS]) * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
#else
    currentPositionTransformed[Z_AXIS] = static_cast<float>(currentPositionSteps[Z_AXIS] - zCorrectionStepsIncluded) * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
#endif
    // currentPositionTransformed[E_AXIS] = currentPositionSteps[E_AXIS] * invAxisStepsPerMM[E_AXIS];
    transformFromPrinter(currentPositionTransformed[X_AXIS], currentPositionTransformed[Y_AXIS], currentPositionTransformed[Z_AXIS],
                         currentPosition[X_AXIS], currentPosition[Y_AXIS], currentPosition[Z_AXIS]);
    currentPosition[X_AXIS] -= Printer::offsetX; // Offset from active extruder or z probe
    currentPosition[Y_AXIS] -= Printer::offsetY;
    currentPosition[Z_AXIS] -= Printer::offsetZ;
    if (copyLastCmd) {
        lastCmdPos[X_AXIS] = currentPosition[X_AXIS];
        lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS];
        lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS];
    }
}

void Printer::updateCurrentPositionSteps() {
    float x_rotc, y_rotc, z_rotc;
    transformToPrinter(currentPosition[X_AXIS] + Printer::offsetX, currentPosition[Y_AXIS] + Printer::offsetY, currentPosition[Z_AXIS] + Printer::offsetZ, x_rotc, y_rotc, z_rotc);
    z_rotc += offsetZ2;
    destinationSteps[X_AXIS] = currentPositionSteps[X_AXIS] = lroundf(x_rotc * axisStepsPerMM[X_AXIS]);
    destinationSteps[Y_AXIS] = currentPositionSteps[Y_AXIS] = lroundf(y_rotc * axisStepsPerMM[Y_AXIS]);
    destinationSteps[Z_AXIS] = currentPositionSteps[Z_AXIS] = lroundf(z_rotc * axisStepsPerMM[Z_AXIS]);
    destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS] = lroundf(currentPositionTransformed[E_AXIS] * axisStepsPerMM[E_AXIS]);
#if NONLINEAR_SYSTEM
    transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
#endif
#if DRIVE_SYSTEM != DELTA
    zCorrectionStepsIncluded = 0;
#endif
}

/** \brief Sets the destination coordinates to values stored in com.

Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
\param com g-code with new destination position.
\return true if it is a move, false if no move results from coordinates.
 */

uint8_t Printer::setDestinationStepsFromGCode(GCode* com) {
    unparkSafety();
    register int32_t p;
    float x, y, z;
    bool posAllowed = true;
#if FEATURE_RETRACTION
    if (com->hasNoXYZ() && com->hasE() && isAutoretract()) { // convert into auto retract
        if (relativeCoordinateMode || relativeExtruderCoordinateMode) {
            Extruder::current->retract(com->E < 0, false);
        } else {
            p = convertToMM(com->E * axisStepsPerMM[E_AXIS]); // target position
            Extruder::current->retract(p < currentPositionSteps[E_AXIS], false);
        }
        return 0; // Fake no move so nothing gets added
    }
#endif
#if DISTORTION_CORRECTION == 0
    if (!com->hasNoXYZ()) {
#endif
        if (!relativeCoordinateMode) {
            if (com->hasX())
                currentPosition[X_AXIS] = lastCmdPos[X_AXIS] = convertToMM(com->X) - coordinateOffset[X_AXIS];
            if (com->hasY())
                currentPosition[Y_AXIS] = lastCmdPos[Y_AXIS] = convertToMM(com->Y) - coordinateOffset[Y_AXIS];
            if (com->hasZ())
                currentPosition[Z_AXIS] = lastCmdPos[Z_AXIS] = convertToMM(com->Z) - coordinateOffset[Z_AXIS];
        } else {
            if (com->hasX())
                currentPosition[X_AXIS] = (lastCmdPos[X_AXIS] += convertToMM(com->X));
            if (com->hasY())
                currentPosition[Y_AXIS] = (lastCmdPos[Y_AXIS] += convertToMM(com->Y));
            if (com->hasZ())
                currentPosition[Z_AXIS] = (lastCmdPos[Z_AXIS] += convertToMM(com->Z));
        }
        transformToPrinter(lastCmdPos[X_AXIS] + Printer::offsetX, lastCmdPos[Y_AXIS] + Printer::offsetY, lastCmdPos[Z_AXIS] + Printer::offsetZ, destinationPositionTransformed[X_AXIS], destinationPositionTransformed[Y_AXIS], destinationPositionTransformed[Z_AXIS]);
        destinationPositionTransformed[Z_AXIS] += offsetZ2;
        destinationSteps[X_AXIS] = lroundf(destinationPositionTransformed[X_AXIS] * axisStepsPerMM[X_AXIS]);
        destinationSteps[Y_AXIS] = lroundf(destinationPositionTransformed[Y_AXIS] * axisStepsPerMM[Y_AXIS]);
        destinationSteps[Z_AXIS] = lroundf(destinationPositionTransformed[Z_AXIS] * axisStepsPerMM[Z_AXIS]);

#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
        if (!isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
            if (!isXHomed()) {
                currentPositionSteps[X_AXIS] = destinationSteps[X_AXIS];
                currentPosition[X_AXIS] = destinationPositionTransformed[X_AXIS];
            }
#endif
#if MOVE_Y_WHEN_HOMED
            if (!isYHomed()) {
                currentPositionSteps[Y_AXIS] = destinationSteps[Y_AXIS];
                currentPosition[Y_AXIS] = destinationPositionTransformed[Y_AXIS];
            }
#endif
#if MOVE_Z_WHEN_HOMED
            if (!isZHomed()) {
                currentPositionSteps[Z_AXIS] = destinationSteps[Z_AXIS];
                currentPosition[Z_AXIS] = destinationPositionTransformed[Z_AXIS];
            }
#endif
        }
#endif

#if LAZY_DUAL_X_AXIS
        sledParked = false;
#endif
        posAllowed = com->hasNoXYZ() || Printer::isPositionAllowed(lastCmdPos[X_AXIS], lastCmdPos[Y_AXIS], lastCmdPos[Z_AXIS]);
#if DISTORTION_CORRECTION == 0
    }
#endif
#if DUAL_X_AXIS && LAZY_DUAL_X_AXIS
    if (sledParked) {
        destinationSteps[X_AXIS] = currentPositionSteps[X_AXIS];
        destinationSteps[Y_AXIS] = currentPositionSteps[Y_AXIS];
        destinationSteps[Z_AXIS] = currentPositionSteps[Z_AXIS];
    }
#endif
    if (com->hasE() && !Printer::debugDryrun()) {
        p = convertToMM(com->E * axisStepsPerMM[E_AXIS]);
        if (relativeCoordinateMode || relativeExtruderCoordinateMode) {
            if (
#if MIN_EXTRUDER_TEMP > 20
                (Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0) ||
#endif
                fabs(com->E) * extrusionFactor > EXTRUDE_MAXLENGTH) {
                p = 0;
                destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS] = 0;
            } else {
                destinationPositionTransformed[E_AXIS] = 0;
                currentPositionTransformed[E_AXIS] = -convertToMM(com->E);
            }

            destinationSteps[E_AXIS] = 0;
            currentPositionSteps[E_AXIS] = -p;
        } else {
            if (
#if MIN_EXTRUDER_TEMP > 20
                (Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0) ||
#endif
                fabs(p - currentPositionSteps[E_AXIS]) * extrusionFactor > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS]) {
                currentPositionSteps[E_AXIS] = p;
                destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS] = convertToMM(com->E);
            } else {
                destinationPositionTransformed[E_AXIS] = convertToMM(com->E);
            }
            destinationSteps[E_AXIS] = p;
        }
    } else {
        destinationPositionTransformed[E_AXIS] = currentPositionTransformed[E_AXIS];
        Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
    }
    if (com->hasF() && com->F > 0.1) {
        if (unitIsInches)
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply; // Factor is 25.5/60/100
        else
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
    }
    if (!posAllowed) {
        currentPositionSteps[E_AXIS] = destinationSteps[E_AXIS];
        return false; // ignore move
    }
    return !com->hasNoXYZ() || (com->hasE() && destinationSteps[E_AXIS] != currentPositionSteps[E_AXIS]); // ignore unproductive moves
}

void Printer::setup() {
    HAL::stopWatchdog();
    for (uint8_t i = 0; i < NUM_PWM; i++)
        pwm_pos[i] = 0;
#if FEATURE_CONTROLLER == CONTROLLER_VIKI
    HAL::delayMilliseconds(100);
#endif // FEATURE_CONTROLLER
#if defined(MB_SETUP)
    MB_SETUP;
#endif
#if UI_DISPLAY_TYPE != NO_DISPLAY
    Com::selectLanguage(0); // just make sure we have a language in case someone uses it early
#endif
    //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization
#if defined(EEPROM_AVAILABLE) && defined(EEPROM_SPI_ALLIGATOR) && EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
    HAL::spiBegin();
#endif
    HAL::hwSetup();
    EVENT_INITIALIZE_EARLY
#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0 >= 0
    SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1 >= 0
    SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2 >= 0
    SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3 >= 0
    SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4 >= 0
    SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5 >= 0
    SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6 >= 0
    SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7 >= 0
    SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && ENABLE_POWER_ON_STARTUP && (PS_ON_PIN > -1)
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
    Printer::setPowerOn(true);
#else
#if PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
    Printer::setPowerOn(false);
#else
    Printer::setPowerOn(true);
#endif
#endif
#if SDSUPPORT
    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER, HIGH);
#endif
#if defined(SDCARDDETECT) && SDCARDDETECT > -1
    SET_INPUT(SDCARDDETECT);
    PULLUP(SDCARDDETECT, HIGH);
#endif
#endif

    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
    SET_OUTPUT(Z_STEP_PIN);
    endXYZSteps();
    //Initialize Dir Pins
#if X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN > -1
    SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN > -1
    SET_OUTPUT(Z_DIR_PIN);
#endif

    //Steppers default to disabled.
#if X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER || DUAL_X_AXIS
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);
#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#endif

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);
#if Z2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#endif

#if FEATURE_THREE_ZSTEPPER
    SET_OUTPUT(Z3_STEP_PIN);
    SET_OUTPUT(Z3_DIR_PIN);
#if Z3_ENABLE_PIN > -1
    SET_OUTPUT(Z3_ENABLE_PIN);
    WRITE(Z3_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#endif

#if FEATURE_FOUR_ZSTEPPER
    SET_OUTPUT(Z4_STEP_PIN);
    SET_OUTPUT(Z4_DIR_PIN);
#if Z4_ENABLE_PIN > -1
    SET_OUTPUT(Z4_ENABLE_PIN);
    WRITE(Z4_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#endif

#if defined(DOOR_PIN) && DOOR_PIN > -1
    SET_INPUT(DOOR_PIN);
#if defined(DOOR_PULLUP) && DOOR_PULLUP
    PULLUP(DOOR_PIN, HIGH);
#endif
#endif
#if defined(POWERLOSS_PIN) && POWERLOSS_PIN > -1
    SET_INPUT(POWERLOSS_PIN);
#if defined(POWERLOSS_PULLUP) && POWERLOSS_PULLUP
    PULLUP(POWERLOSS_PIN, HIGH);
#endif
#endif

    Endstops::setup();

#if FEATURE_Z_PROBE && Z_PROBE_PIN > -1
    SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
    PULLUP(Z_PROBE_PIN, HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
    SET_OUTPUT(FAN2_PIN);
    WRITE(FAN2_PIN, LOW);
#endif
#if FAN_THERMO_PIN > -1
    SET_OUTPUT(FAN_THERMO_PIN);
    WRITE(FAN_THERMO_PIN, LOW);
#endif
#if FAN_BOARD_PIN > -1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN, LOW);
    pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
    SET_OUTPUT(EXT2_HEATER_PIN);
    WRITE(EXT2_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
    SET_OUTPUT(EXT3_HEATER_PIN);
    WRITE(EXT3_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
    SET_OUTPUT(EXT4_HEATER_PIN);
    WRITE(EXT4_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
    SET_OUTPUT(EXT5_HEATER_PIN);
    WRITE(EXT5_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT0_EXTRUDER_COOLER_PIN) && EXT0_EXTRUDER_COOLER_PIN > -1
    SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
    WRITE(EXT0_EXTRUDER_COOLER_PIN, LOW);
#endif
#if defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 1
    SET_OUTPUT(EXT1_EXTRUDER_COOLER_PIN);
    WRITE(EXT1_EXTRUDER_COOLER_PIN, LOW);
#endif
#if defined(EXT2_EXTRUDER_COOLER_PIN) && EXT2_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 2
    SET_OUTPUT(EXT2_EXTRUDER_COOLER_PIN);
    WRITE(EXT2_EXTRUDER_COOLER_PIN, LOW);
#endif
#if defined(EXT3_EXTRUDER_COOLER_PIN) && EXT3_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 3
    SET_OUTPUT(EXT3_EXTRUDER_COOLER_PIN);
    WRITE(EXT3_EXTRUDER_COOLER_PIN, LOW);
#endif
#if defined(EXT4_EXTRUDER_COOLER_PIN) && EXT4_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 4
    SET_OUTPUT(EXT4_EXTRUDER_COOLER_PIN);
    WRITE(EXT4_EXTRUDER_COOLER_PIN, LOW);
#endif
#if defined(EXT5_EXTRUDER_COOLER_PIN) && EXT5_EXTRUDER_COOLER_PIN > -1 && NUM_EXTRUDER > 5
    SET_OUTPUT(EXT5_EXTRUDER_COOLER_PIN);
    WRITE(EXT5_EXTRUDER_COOLER_PIN, LOW);
#endif
// Initialize jam sensors
#if defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
    SET_INPUT(EXT0_JAM_PIN);
    PULLUP(EXT0_JAM_PIN, EXT0_JAM_PULLUP);
#endif // defined
#if defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
    SET_INPUT(EXT1_JAM_PIN);
    PULLUP(EXT1_JAM_PIN, EXT1_JAM_PULLUP);
#endif // defined
#if defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
    SET_INPUT(EXT2_JAM_PIN);
    PULLUP(EXT2_JAM_PIN, EXT2_JAM_PULLUP);
#endif // defined
#if defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
    SET_INPUT(EXT3_JAM_PIN);
    PULLUP(EXT3_JAM_PIN, EXT3_JAM_PULLUP);
#endif // defined
#if defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
    SET_INPUT(EXT4_JAM_PIN);
    PULLUP(EXT4_JAM_PIN, EXT4_JAM_PULLUP);
#endif // defined
#if defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
    SET_INPUT(EXT5_JAM_PIN);
    PULLUP(EXT5_JAM_PIN, EXT5_JAM_PULLUP);
#endif // defined
    HAL::delayMilliseconds(1);
#if defined(EXT0_JAM_PIN) && EXT0_JAM_PIN > -1
    extruder[0].jamLastSignal = READ(EXT0_JAM_PIN);
#endif // defined
#if defined(EXT1_JAM_PIN) && EXT1_JAM_PIN > -1
    extruder[1].jamLastSignal = READ(EXT1_JAM_PIN);
#endif // defined
#if defined(EXT2_JAM_PIN) && EXT2_JAM_PIN > -1
    extruder[2].jamLastSignal = READ(EXT2_JAM_PIN);
#endif // defined
#if defined(EXT3_JAM_PIN) && EXT3_JAM_PIN > -1
    extruder[3].jamLastSignal = READ(EXT3_JAM_PIN);
#endif // defined
#if defined(EXT4_JAM_PIN) && EXT4_JAM_PIN > -1
    extruder[4].jamLastSignal = READ(EXT4_JAM_PIN);
#endif // defined
#if defined(EXT5_JAM_PIN) && EXT5_JAM_PIN > -1
    extruder[5].jamLastSignal = READ(EXT5_JAM_PIN);
#endif // defined
#if CASE_LIGHTS_PIN >= 0
    SET_OUTPUT(CASE_LIGHTS_PIN);
    WRITE(CASE_LIGHTS_PIN, CASE_LIGHT_DEFAULT_ON);
    lightOn = CASE_LIGHT_DEFAULT_ON;
#endif // CASE_LIGHTS_PIN
#if defined(UI_VOLTAGE_LEVEL) && defined(EXP_VOLTAGE_LEVEL_PIN) && EXP_VOLTAGE_LEVEL_PIN > -1
    SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
    WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
#endif // UI_VOLTAGE_LEVEL
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    LaserDriver::initialize();
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
    CNCDriver::initialize();
#endif // defined

#if GANTRY && !defined(FAST_COREXYZ)
    Printer::motorX = 0;
    Printer::motorYorZ = 0;
#endif
#ifdef RED_BLUE_STATUS_LEDS
    SET_OUTPUT(RED_STATUS_LED);
    SET_OUTPUT(BLUE_STATUS_LED);
    WRITE(BLUE_STATUS_LED, HIGH);
    WRITE(RED_STATUS_LED, LOW);
#endif // RED_BLUE_STATUS_LEDS
#if defined(DRV_TMC2130)
    SET_OUTPUT(MOSI_PIN);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    // TMC2130 motor drivers
#if TMC2130_ON_X
    Printer::tmc_driver_x = new TMC2130Stepper(X_ENABLE_PIN, X_DIR_PIN, X_STEP_PIN, TMC2130_X_CS_PIN);
    configTMC2130(Printer::tmc_driver_x, TMC2130_STEALTHCHOP_X, TMC2130_STALLGUARD_X,
                  TMC2130_PWM_AMPL_X, TMC2130_PWM_GRAD_X, TMC2130_PWM_AUTOSCALE_X, TMC2130_PWM_FREQ_X);
#endif
#if TMC2130_ON_Y > 0
    Printer::tmc_driver_y = new TMC2130Stepper(Y_ENABLE_PIN, Y_DIR_PIN, Y_STEP_PIN, TMC2130_Y_CS_PIN);
    configTMC2130(Printer::tmc_driver_y, TMC2130_STEALTHCHOP_Y, TMC2130_STALLGUARD_Y,
                  TMC2130_PWM_AMPL_Y, TMC2130_PWM_GRAD_Y, TMC2130_PWM_AUTOSCALE_Y, TMC2130_PWM_FREQ_Y);
#endif
#if TMC2130_ON_Z > 0
    Printer::tmc_driver_z = new TMC2130Stepper(Z_ENABLE_PIN, Z_DIR_PIN, Z_STEP_PIN, TMC2130_Z_CS_PIN);
    configTMC2130(Printer::tmc_driver_z, TMC2130_STEALTHCHOP_Z, TMC2130_STALLGUARD_Z,
                  TMC2130_PWM_AMPL_Z, TMC2130_PWM_GRAD_Z, TMC2130_PWM_AUTOSCALE_Z, TMC2130_PWM_FREQ_Z);
#endif
#if TMC2130_ON_EXT0 > 0
    Printer::tmc_driver_e0 = new TMC2130Stepper(EXT0_ENABLE_PIN, EXT0_DIR_PIN, EXT0_STEP_PIN, TMC2130_EXT0_CS_PIN);
    configTMC2130(Printer::tmc_driver_e0, TMC2130_STEALTHCHOP_EXT0, TMC2130_STALLGUARD_EXT0,
                  TMC2130_PWM_AMPL_EXT0, TMC2130_PWM_GRAD_EXT0, TMC2130_PWM_AUTOSCALE_EXT0, TMC2130_PWM_FREQ_EXT0);
#endif
#if TMC2130_ON_EXT1 > 0
    Printer::tmc_driver_e1 = new TMC2130Stepper(EXT1_ENABLE_PIN, EXT1_DIR_PIN, EXT1_STEP_PIN, TMC2130_EXT1_CS_PIN);
    configTMC2130(Printer::tmc_driver_e1, TMC2130_STEALTHCHOP_EXT1, TMC2130_STALLGUARD_EXT1,
                  TMC2130_PWM_AMPL_EXT1, TMC2130_PWM_GRAD_EXT1, TMC2130_PWM_AUTOSCALE_EXT1, TMC2130_PWM_FREQ_EXT1);
#endif
#if TMC2130_ON_EXT2 > 0
    Printer::tmc_driver_e2 = new TMC2130Stepper(EXT2_ENABLE_PIN, EXT2_DIR_PIN, EXT2_STEP_PIN, TMC2130_EXT2_CS_PIN);
    configTMC2130(Printer::tmc_driver_e2, TMC2130_STEALTHCHOP_EXT2, TMC2130_STALLGUARD_EXT2,
                  TMC2130_PWM_AMPL_EXT2, TMC2130_PWM_GRAD_EXT2, TMC2130_PWM_AUTOSCALE_EXT2, TMC2130_PWM_FREQ_EXT2);
#endif
#if TMC2130_ON_EXT3 > 0
    Printer::tmc_driver_e3 = new TMC2130Stepper(EXT3_ENABLE_PIN, EXT3_DIR_PIN, EXT3_STEP_PIN, TMC2130_EXT3_CS_PIN);
    configTMC2130(Printer::tmc_driver_e3, TMC2130_STEALTHCHOP_EXT3, TMC2130_STALLGUARD_EXT3,
                  TMC2130_PWM_AMPL_EXT3, TMC2130_PWM_GRAD_EXT3, TMC2130_PWM_AUTOSCALE_EXT3, TMC2130_PWM_FREQ_EXT3);
#endif
#if TMC2130_ON_EXT4 > 0
    Printer::tmc_driver_e4 = new TMC2130Stepper(EXT4_ENABLE_PIN, EXT4_DIR_PIN, EXT4_STEP_PIN, TMC2130_EXT4_CS_PIN);
    configTMC2130(Printer::tmc_driver_e4, TMC2130_STEALTHCHOP_EXT4, TMC2130_STALLGUARD_EXT4,
                  TMC2130_PWM_AMPL_EXT4, TMC2130_PWM_GRAD_EXT4, TMC2130_PWM_AUTOSCALE_EXT4, TMC2130_PWM_FREQ_EXT4);
#endif
#endif // DRV_TMC2130
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
    motorCurrentControlInit(); // Set current if it is firmware controlled
#endif
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    initializeAllMotorDrivers();
#endif // defined
    microstepInit();
#if FEATURE_AUTOLEVEL
    resetTransformationMatrix(true);
#endif             // FEATURE_AUTOLEVEL
    feedrate = 50; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
    extrudeMultiply = 100;
    lastCmdPos[X_AXIS] = lastCmdPos[Y_AXIS] = lastCmdPos[Z_AXIS] = 0;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    advanceExecuted = 0;
#endif
    advanceStepsSet = 0;
#endif
    maxJerk = MAX_JERK;
#if DRIVE_SYSTEM != DELTA
    maxZJerk = MAX_ZJERK;
#endif
    offsetX = offsetY = offsetZ = 0;
    interval = 5000;
    stepsPerTimerCall = 1;
    msecondsPrinting = 0;
    filamentPrinted = 0;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
    xLength = X_MAX_LENGTH;
    yLength = Y_MAX_LENGTH;
    zLength = Z_MAX_LENGTH;
    xMin = X_MIN_POS;
    yMin = Y_MIN_POS;
    zMin = Z_MIN_POS;
#if DUAL_X_AXIS_MODE > 0 && LAZY_DUAL_X_AXIS == 0
    x1Min = xMin;
    x1Length = xLength;
#endif
#if DRIVE_SYSTEM == DELTA
    radius0 = ROD_RADIUS;
#endif
#if ENABLE_BACKLASH_COMPENSATION
    backlashX = X_BACKLASH;
    backlashY = Y_BACKLASH;
    backlashZ = Z_BACKLASH;
    backlashDir = 0;
#endif
#if USE_ADVANCE
    extruderStepsNeeded = 0;
#endif
#if (MOTHERBOARD == 502)
    SET_INPUT(FTDI_COM_RESET_PIN);
    SET_INPUT(ESP_WIFI_MODULE_COM);
    SET_INPUT(MOTOR_FAULT_PIN);
    SET_INPUT(MOTOR_FAULT_PIGGY_PIN);
#endif //(MOTHERBOARD == 501) || (MOTHERBOARD == 502)
    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);
    Com::printFLN(Com::tStart);
    HAL::showStartReason();
    Extruder::initExtruder();
    // sets auto leveling in eeprom init
    EEPROM::init(); // Read settings from eeprom if wanted
    UI_INITIALIZE;
    for (uint8_t i = 0; i < E_AXIS_ARRAY; i++) {
        currentPositionSteps[i] = 0;
    }
    currentPosition[X_AXIS] = currentPosition[Y_AXIS] = currentPosition[Z_AXIS] = 0.0;
    //Commands::printCurrentPosition();
#if DISTORTION_CORRECTION
    distortion.init();
#endif // DISTORTION_CORRECTION

    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();

#if FEATURE_WATCHDOG
    HAL::startWatchdog();
#endif // FEATURE_WATCHDOG
#if SDSUPPORT
    sd.mount();
#endif
#if NONLINEAR_SYSTEM
    transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);

#if DELTA_HOME_ON_POWER
    homeAxis(true, true, true);
#endif
    setAutoretract(EEPROM_BYTE(AUTORETRACT_ENABLED));
    Commands::printCurrentPosition();
#endif // DRIVE_SYSTEM
    Extruder::selectExtruderById(0);

#if FEATURE_SERVO // set servos to neutral positions at power_up
#if defined(SERVO0_NEUTRAL_POS) && SERVO0_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(0, SERVO0_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO1_NEUTRAL_POS) && SERVO1_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(1, SERVO1_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO2_NEUTRAL_POS) && SERVO2_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(2, SERVO2_NEUTRAL_POS, 1000);
#endif
#if defined(SERVO3_NEUTRAL_POS) && SERVO3_NEUTRAL_POS >= 500
    HAL::servoMicroseconds(3, SERVO3_NEUTRAL_POS, 1000);
#endif
#endif
    EVENT_INITIALIZE;
#ifdef STARTUP_GCODE
    GCode::executeFString(Com::tStartupGCode);
#endif
#if EEPROM_MODE != 0 && UI_DISPLAY_TYPE != NO_DISPLAY
    if (EEPROM::getStoredLanguage() == 254) {
        Com::printFLN(PSTR("Needs language selection"));
        uid.showLanguageSelectionWizard();
    }
#endif // EEPROM_MODE
    rescueSetup();
}

void Printer::defaultLoopActions() {
    Commands::checkForPeriodicalActions(true); //check heater every n milliseconds
    UI_MEDIUM;                                 // do check encoder
    millis_t curtime = HAL::timeInMilliseconds();
    if (PrintLine::hasLines() || isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED))
        previousMillisCmd = curtime;
    else {
        curtime -= previousMillisCmd;
        if (maxInactiveTime != 0 && curtime > maxInactiveTime)
            Printer::kill(false);
        else
            Printer::setAllKilled(false); // prevent repeated kills
        if (stepperInactiveTime != 0 && curtime > stepperInactiveTime)
            Printer::kill(true);
    }
#if SDCARDDETECT > -1 && SDSUPPORT
    sd.automount();
#endif
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::syncEEPROM();
#endif

    DEBUG_MEMORY;
}

void Printer::MemoryPosition() {
    Commands::waitUntilEndOfAllMoves();
    updateCurrentPosition(false);
    realPosition(memoryX, memoryY, memoryZ);
    memoryE = currentPositionSteps[E_AXIS] * invAxisStepsPerMM[E_AXIS];
    memoryF = feedrate;
}

void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed) {
    if (memoryF < 0)
        return; // Not stored before call, so we ignore it
    bool all = !(x || y || z);
    moveToReal((all || x ? (lastCmdPos[X_AXIS] = memoryX) : IGNORE_COORDINATE), (all || y ? (lastCmdPos[Y_AXIS] = memoryY) : IGNORE_COORDINATE), (all || z ? (lastCmdPos[Z_AXIS] = memoryZ) : IGNORE_COORDINATE), (e ? memoryE : IGNORE_COORDINATE),
               feed);
    feedrate = memoryF;
    updateCurrentPosition(false);
}

#if DRIVE_SYSTEM == DELTA
void Printer::deltaMoveToTopEndstops(float feedrate) {
    for (fast8_t i = 0; i < 3; i++)
        Printer::currentPositionSteps[i] = 0;
    Printer::stepsRemainingAtXHit = -1;
    Printer::stepsRemainingAtYHit = -1;
    Printer::stepsRemainingAtZHit = -1;
    setHoming(true);
    transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
    PrintLine::moveRelativeDistanceInSteps(0, 0, (zMaxSteps + EEPROM::deltaDiagonalRodLength() * axisStepsPerMM[Z_AXIS]) * 1.5, 0, feedrate, true, true);
    offsetX = offsetY = offsetZ = offsetZ2 = 0;
    setHoming(false);
}
void Printer::homeXAxis() {
    destinationSteps[X_AXIS] = 0;
    if (!PrintLine::queueNonlinearMove(true, false, false)) {
        Com::printWarningFLN(PSTR("homeXAxis / queueDeltaMove returns error"));
    }
}
void Printer::homeYAxis() {
    Printer::destinationSteps[Y_AXIS] = 0;
    if (!PrintLine::queueNonlinearMove(true, false, false)) {
        Com::printWarningFLN(PSTR("homeYAxis / queueDeltaMove returns error"));
    }
}
void Printer::homeZAxis() { // Delta z homing
    bool homingSuccess = false;
    Endstops::resetAccumulator();
    deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
    // New safe homing routine by Kyrre Aalerud
    // This method will safeguard against sticky endstops such as may be gotten cheaply from china.
    // This can lead to head crashes and even fire, thus a safer algorithm to ensure the endstops actually respond as expected.
    //Endstops::report();
    // Check that all endstops (XYZ) were hit
    Endstops::fillFromAccumulator();
    if (Endstops::xMax() && Endstops::yMax() && Endstops::zMax()) {
        // Back off for retest
        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true, true);
        //Endstops::report();
        // Check for proper release of all (XYZ) endstops
        if (!(Endstops::xMax() || Endstops::yMax() || Endstops::zMax())) {
            // Rehome with reduced speed
            Endstops::resetAccumulator();
            deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR);
            Endstops::fillFromAccumulator();
            // Endstops::report();
            // Check that all endstops (XYZ) were hit again
            if (Endstops::xMax() && Endstops::yMax() && Endstops::zMax()) {
                homingSuccess = true; // Assume success in case there is no back move
#if defined(ENDSTOP_Z_BACK_ON_HOME)
                if (ENDSTOP_Z_BACK_ON_HOME > 0) {
                    PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_ON_HOME, 0, homingFeedrate[Z_AXIS], true, true);
                    //Endstops::report();
                    // Check for missing release of any (XYZ) endstop
                    if (Endstops::xMax() || Endstops::yMax() || Endstops::zMax()) {
                        homingSuccess = false; // Reset success flag
                    }
                }
#endif
            }
        } else {
            Com::printFLN(PSTR("Back move did not untrigger endstops!"), (float)ENDSTOP_Z_BACK_MOVE);
        }
    }
    // Check if homing failed.  If so, request pause!
    if (!homingSuccess) {
        setXHomed(false);
        setYHomed(false);
        setZHomed(false);
        GCodeSource::printAllFLN(PSTR("RequestPause:Homing failed!"));
    } else {
        setXHomed(true);
        setYHomed(true);
        setZHomed(true);
    }
    // Correct different end stop heights
    // These can be adjusted by two methods. You can use offsets stored by determining the center
    // or you can use the xyzMinSteps from G100 calibration. Both have the same effect but only one
    // should be measured as both have the same effect.
    long dx = -xMinSteps - EEPROM::deltaTowerXOffsetSteps();
    long dy = -yMinSteps - EEPROM::deltaTowerYOffsetSteps();
    long dz = -zMinSteps - EEPROM::deltaTowerZOffsetSteps();
    long dm = RMath::min(dx, dy, dz);
    //Com::printFLN(Com::tTower1,dx);
    //Com::printFLN(Com::tTower2,dy);
    //Com::printFLN(Com::tTower3,dz);
    dx -= dm; // now all dxyz are positive
    dy -= dm;
    dz -= dm;
    currentPositionSteps[X_AXIS] = 0; // here we should be
    currentPositionSteps[Y_AXIS] = 0;
    currentPositionSteps[Z_AXIS] = zMaxSteps;
    transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
    currentNonlinearPositionSteps[A_TOWER] -= dx;
    currentNonlinearPositionSteps[B_TOWER] -= dy;
    currentNonlinearPositionSteps[C_TOWER] -= dz;
    PrintLine::moveRelativeDistanceInSteps(0, 0, dm, 0, homingFeedrate[Z_AXIS], true, false);
    currentPositionSteps[X_AXIS] = 0; // now we are really here
    currentPositionSteps[Y_AXIS] = 0;
    currentPositionSteps[Z_AXIS] = zMaxSteps - zBedOffset * axisStepsPerMM[Z_AXIS]; // Extruder is now exactly in the delta center
    coordinateOffset[X_AXIS] = 0;
    coordinateOffset[Y_AXIS] = 0;
    coordinateOffset[Z_AXIS] = 0;
    transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
    realDeltaPositionSteps[A_TOWER] = currentNonlinearPositionSteps[A_TOWER];
    realDeltaPositionSteps[B_TOWER] = currentNonlinearPositionSteps[B_TOWER];
    realDeltaPositionSteps[C_TOWER] = currentNonlinearPositionSteps[C_TOWER];
    //maxDeltaPositionSteps = currentDeltaPositionSteps[X_AXIS];
#if defined(ENDSTOP_Z_BACK_ON_HOME)
    if (ENDSTOP_Z_BACK_ON_HOME > 0)
        maxDeltaPositionSteps += axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME;
#endif
    Extruder::selectExtruderById(Extruder::current->id);
#if FEATURE_BABYSTEPPING
    Printer::zBabysteps = 0;
#endif
}
// This home axis is for delta
void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) { // Delta homing code
    unparkSafety();
    bool nocheck = isNoDestinationCheck();
    setNoDestinationCheck(true);
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    bool oldLaser = LaserDriver::laserOn;
    LaserDriver::laserOn = false;
#endif
    bool autoLevel = isAutolevelActive();
    setAutolevelActive(false);
    if (!(X_MAX_PIN > -1 && Y_MAX_PIN > -1 && Z_MAX_PIN > -1
          && MAX_HARDWARE_ENDSTOP_X && MAX_HARDWARE_ENDSTOP_Y && MAX_HARDWARE_ENDSTOP_Z)) {
        Com::printErrorFLN(PSTR("Hardware setup inconsistent. Delta cannot home without max endstops."));
    }
    // The delta has to have home capability to zero and set position,
    // so the redundant check is only an opportunity to
    // gratuitously fail due to incorrect settings.
    // The following movements would be meaningless unless it was zeroed for example.
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HOME_DELTA_ID));
    // Homing Z axis means that you must home X and Y
    EVENT_BEFORE_Z_HOME;
    homeZAxis();
    moveToReal(0, 0, Printer::zLength - zBedOffset, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]); // Move to designed coordinates including translation
    updateCurrentPosition(true);
    updateHomedAll();
    UI_CLEAR_STATUS
    Commands::printCurrentPosition();
    setAutolevelActive(autoLevel);
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    LaserDriver::laserOn = oldLaser;
#endif
    Printer::updateCurrentPosition();
    setNoDestinationCheck(nocheck);
}
#else
#if DRIVE_SYSTEM == TUGA // Tuga printer homing
void Printer::homeXAxis() {
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR == -1 && MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR == 1 && MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR == 1)) {
        long offX = 0, offY = 0;
#if NUM_EXTRUDER > 1
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
#if X_HOME_DIR < 0
            offX = RMath::max(offX, extruder[i].xOffset);
            offY = RMath::max(offY, extruder[i].yOffset);
#else
            offX = RMath::min(offX, extruder[i].xOffset);
            offY = RMath::min(offY, extruder[i].yOffset);
#endif
        }
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        steps = (Printer::xMaxSteps - Printer::xMinSteps) * X_HOME_DIR;
        currentPositionSteps[X_AXIS] = -steps;
        currentPositionSteps[Y_AXIS] = 0;
        setHoming(true);
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
        PrintLine::moveRelativeDistanceInSteps(2 * steps, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps - offX : xMaxSteps + offX;
        currentPositionSteps[Y_AXIS] = 0; //(Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
        //PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,axisStepsPerMM[Y_AXIS]*-ENDSTOP_X_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[X_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        // PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,axisStepsPerMM[Y_AXIS]*2*ENDSTOP_X_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[X_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
        setHoming(false);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if (ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, false);
            // PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,axisStepsPerMM[Y_AXIS]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homingFeedrate[X_AXIS],true,false);
#endif
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps - offX : xMaxSteps + offX;
        currentPositionSteps[Y_AXIS] = 0; //(Y_HOME_DIR == -1) ? yMinSteps-offY : yMaxSteps+offY;
        coordinateOffset[X_AXIS] = 0;
        coordinateOffset[Y_AXIS] = 0;
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#if NUM_EXTRUDER > 1
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset - offX) * X_HOME_DIR, (Extruder::current->yOffset - offY) * Y_HOME_DIR, 0, 0, homingFeedrate[X_AXIS], true, false);
#endif
        setXHomed(true);
        setYHomed(true);
    }
}
void Printer::homeYAxis() {
    // Dummy function x and y homing must occur together
}
#else // Cartesian printer
void Printer::homeXAxis() {
#if defined(SENSORLESS_HOMING) && TMC2130_ON_X
    while (!Printer::tmc_driver_x->stst())
        ; // Wait for motor stand-still
    uint32_t coolstep_speed = Printer::tmc_driver_x->coolstep_min_speed();
    uint32_t stealth_max_sp = Printer::tmc_driver_x->stealth_max_speed();
    bool stealth_state = Printer::tmc_driver_x->stealthChop();
    Printer::tmcPrepareHoming(Printer::tmc_driver_x, TMC2130_TCOOLTHRS_X);
#endif
    bool nocheck = isNoDestinationCheck();
    setNoDestinationCheck(true);
    long steps;
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HOME_X_ID));
    Commands::waitUntilEndOfAllMoves();
    setHoming(true);
#if DUAL_X_AXIS && NUM_EXTRUDER == 2
    Extruder* curExtruder = Extruder::current;
    Extruder::current = &extruder[0];
    steps = (Printer::xMaxSteps - Printer::xMinSteps);
    currentPositionSteps[X_AXIS] = steps;
    PrintLine::moveRelativeDistanceInSteps(-3 * steps * -X_HOME_DIR / 2, 0, 0, 0, homingFeedrate[X_AXIS], true, true);                                                                        // first contact
    PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * ENDSTOP_X_BACK_MOVE * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, false);                                         // back move
    PrintLine::moveRelativeDistanceInSteps(-axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true); // final contact
#if defined(ENDSTOP_X_BACK_ON_HOME)
    if (ENDSTOP_X_BACK_ON_HOME > 0)
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * ENDSTOP_X_BACK_ON_HOME * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, false);
#endif
    Extruder::current = &extruder[1];
    currentPositionSteps[X_AXIS] = -steps;
    PrintLine::moveRelativeDistanceInSteps(2 * steps * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
    PrintLine::moveRelativeDistanceInSteps(-axisStepsPerMM[X_AXIS] * ENDSTOP_X_BACK_MOVE * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, false); // back move
    PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
    if (ENDSTOP_X_BACK_ON_HOME > 0)
        PrintLine::moveRelativeDistanceInSteps(-axisStepsPerMM[X_AXIS] * ENDSTOP_X_BACK_ON_HOME * -X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, false);
#endif
    Extruder::current = curExtruder;
#if LAZY_DUAL_X_AXIS
    currentPositionSteps[X_AXIS] = xMinSteps + Extruder::current->xOffset;
    sledParked = true;
    currentPosition[X_AXIS] = lastCmdPos[X_AXIS] = xMin;
#else
#if DUAL_X_AXIS_MODE > 0 && LAZY_DUAL_X_AXIS == 0
    if (Extruder::current->isLeftCarriage()) //left carriage
        currentPositionSteps[X_AXIS] = xMinSteps;
    else
        currentPositionSteps[X_AXIS] = Extruder::current->xOffset;
#else
    // Now position current extrude on x = 0
    PrintLine::moveRelativeDistanceInSteps(-Extruder::current->xOffset, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
    currentPositionSteps[X_AXIS] = xMinSteps;
#endif
#endif // LAZY_DUAL_X_AXIS
    updateCurrentPosition(false);
    offsetX = 0;
    setXHomed(true);

#else // DUAL_AXIS
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR == 1)) {
        coordinateOffset[X_AXIS] = 0;
        long offX = 0;
#if NUM_EXTRUDER > 1
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
#if X_HOME_DIR < 0
            offX = RMath::max(offX, extruder[i].xOffset);
#else
            offX = RMath::min(offX, extruder[i].xOffset);
#endif
            // Reposition extruder that way, that all extruders can be selected at home position.
#endif // NUM_EXTRUDER > 1
        steps = (Printer::xMaxSteps - Printer::xMinSteps) * X_HOME_DIR;
        currentPositionSteps[X_AXIS] = -steps;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(2 * steps, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps - offX : xMaxSteps + offX;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if (ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
#endif
        currentPositionSteps[X_AXIS] = (X_HOME_DIR == -1) ? xMinSteps - offX : xMaxSteps + offX;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
#if NUM_EXTRUDER > 1
#if X_HOME_DIR < 0
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset - offX) * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
#else
        PrintLine::moveRelativeDistanceInSteps(-(Extruder::current->xOffset - offX) * X_HOME_DIR, 0, 0, 0, homingFeedrate[X_AXIS], true, true);
#endif
#endif
        setXHomed(true);
    }
#endif // ELSE dual x axis
    setNoDestinationCheck(nocheck);
    setHoming(false);
#if defined(SENSORLESS_HOMING) && TMC2130_ON_X
    while (!Printer::tmc_driver_x->stst())
        ; // Wait for motor stand-still
    Printer::tmc_driver_x->coolstep_min_speed(coolstep_speed);
    Printer::tmc_driver_x->stealth_max_speed(stealth_max_sp);
    Printer::tmc_driver_x->stealthChop(stealth_state);
#endif
}

void Printer::homeYAxis() {
#if defined(SENSORLESS_HOMING) && TMC2130_ON_Y
    while (!Printer::tmc_driver_y->stst())
        ; // Wait for motor stand-still
    uint32_t coolstep_speed = Printer::tmc_driver_y->coolstep_min_speed();
    uint32_t stealth_max_sp = Printer::tmc_driver_y->stealth_max_speed();
    bool stealth_state = Printer::tmc_driver_y->stealthChop();
    Printer::tmcPrepareHoming(Printer::tmc_driver_y, TMC2130_TCOOLTHRS_Y);
#endif
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR == 1)) {
        coordinateOffset[Y_AXIS] = 0;
        long offY = 0;
#if NUM_EXTRUDER > 1
        for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
#if Y_HOME_DIR < 0
            offY = RMath::max(offY, extruder[i].yOffset);
#else
            offY = RMath::min(offY, extruder[i].yOffset);
#endif
            // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HOME_Y_ID));
        steps = (yMaxSteps - Printer::yMinSteps) * Y_HOME_DIR;
        currentPositionSteps[Y_AXIS] = -steps;
        setHoming(true);
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 2 * steps, 0, 0, homingFeedrate[Y_AXIS], true, true);
        currentPositionSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? yMinSteps - offY : yMaxSteps + offY;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * -ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * 2 * ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
        setHoming(false);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
        if (ENDSTOP_Y_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0, axisStepsPerMM[Y_AXIS] * -ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS], true, false);
#endif
        currentPositionSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? yMinSteps - offY : yMaxSteps + offY;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
#if NUM_EXTRUDER > 1
#if Y_HOME_DIR < 0
        PrintLine::moveRelativeDistanceInSteps(0, (Extruder::current->yOffset - offY) * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS], true, false);
#else
        PrintLine::moveRelativeDistanceInSteps(0, -(Extruder::current->yOffset - offY) * Y_HOME_DIR, 0, 0, homingFeedrate[Y_AXIS], true, false);
#endif
#endif
        setYHomed(true);
    }
#if defined(SENSORLESS_HOMING) && TMC2130_ON_Y
    while (!Printer::tmc_driver_y->stst())
        ; // Wait for motor stand-still
    Printer::tmc_driver_y->coolstep_min_speed(coolstep_speed);
    Printer::tmc_driver_y->stealth_max_speed(stealth_max_sp);
    Printer::tmc_driver_y->stealthChop(stealth_state);
#endif
}
#endif

/** \brief homes z axis.

Homing z axis is the most complicated homing part as it needs to correct for several parameters depending on rotation correction,
distortion correction, position, homing direction, sensor type and bed coating. Because we get lots of support questions from "wrong"
behavior due to misunderstandings of all these dependencies, I will try to describe the function as detailed as possible.

## Step 1: Test if homing is possible at all

Test if homing in Z_HOME_DIR has a matching hardware endstop. If not, no z homing is possible at all.

## Step 2: Activate z-probe if required

If homing to z min using a z-probe, activate the probe. Requires a position where this is possible, since probe offset could prevent the
move. Hence in this case x and y must be homed first and a z homing position given and reached before calling this function.

## Step 3: Fast homing to sensor

A move of 2 * z_length is send to printer. The move will stop when the endstop triggers.

## Step 4: Untrigger sensor

Move in opposite direction ENDSTOP_Z_BACK_MOVE mm to disable the endstop signal safely.

## Step 5: Retest endstop slowly

For best precision we rerun step 3 with 1/ENDSTOP_Z_RETEST_REDUCTION_FACTOR factor for speed. This should give a very accurate trigge rposition.

## Step 6: Deactivate z-probe if it was activated in step 2

## Step 7: Correct Z position

zCorrection sums up several influences:
- -axisStepsPerMM[Z_AXIS] * EEPROM::zProbeHeight() to compensate for z-prove trigger offset if probe was used.
- -axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR to add wanted extra distance ENDSTOP_Z_BACK_ON_HOME.
- axisStepsPerMM[Z_AXIS] * zBedOffset to correct for bed coating if correction mode Z_PROBE_Z_OFFSET_MODE == 0 and z min homing.
and moves the z axis up that distance.

## Step 8: Set position in CMC

For z min homing set currentPositionSteps[Z_AXIS] to zMinSteps. For z max homing set it to zMaxSteps - bed coating.

Set z offset according to selected tool z offset, if extruder is not sensor.

If distortion correction is enabled, set zCorrectionStepsIncluded and add value to currentPositionSteps.

#step 9: Compute RWC and correct rotation influence

Compute real position from position in steps. If rotation is on we did measure with a z-probe,
this result is wrong and we need to correct by the z change between origin and current position.

    currentPositionSteps[Z_AXIS] -= (axisStepsPerMM[Z_AXIS] * currentPosition[Z_AXIS] - zMinSteps);
    currentPosition[Z_AXIS] = zMin;

## Step 10: Update NMC for nonlinear systems

## Step 11: Set babysteps to 0
*/
void Printer::homeZAxis() { // Cartesian homing
#if defined(SENSORLESS_HOMING) && TMC2130_ON_Z
    while (!Printer::tmc_driver_z->stst())
        ; // Wait for motor stand-still
    uint32_t coolstep_speed = Printer::tmc_driver_z->coolstep_min_speed();
    uint32_t stealth_max_sp = Printer::tmc_driver_z->stealth_max_speed();
    bool stealth_state = Printer::tmc_driver_z->stealthChop();
    tmcPrepareHoming(Printer::tmc_driver_z, TMC2130_TCOOLTHRS_Z);
#endif
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR == -1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR == 1)) {
        offsetZ2 = 0;
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        if (!Printer::startProbing(true)) {
            return;
        }
#endif
        coordinateOffset[Z_AXIS] = 0; // G92 Z offset
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HOME_Z_ID));
        steps = (zMaxSteps - zMinSteps) * Z_HOME_DIR;
        currentPositionSteps[Z_AXIS] = -steps;
        setHoming(true);
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0 && Z_MIN_PIN == Z_PROBE_PIN && Z_HOME_DIR == -1
        HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, 2 * steps, 0, homingFeedrate[Z_AXIS], true, true);
        currentPositionSteps[Z_AXIS] = (Z_HOME_DIR == -1) ? zMinSteps : zMaxSteps;
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true, false);
#if defined(ZHOME_WAIT_UNSWING) && ZHOME_WAIT_UNSWING > 0
        HAL::delayMilliseconds(ZHOME_WAIT_UNSWING);
#endif
#if defined(Z_PROBE_DELAY) && Z_PROBE_DELAY > 0 && Z_MIN_PIN == Z_PROBE_PIN && Z_HOME_DIR == -1
        HAL::delayMilliseconds(Z_PROBE_DELAY);
#endif
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
#ifdef Z_PROBE_RUN_AFTER_EVERY_PROBE
        GCode::executeFString(PSTR(Z_PROBE_RUN_AFTER_EVERY_PROBE));
#endif
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * 2 * ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR, true, true);
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        Printer::finishProbing();
#endif
        setHoming(false);
        int32_t zCorrection = 0;
#if Z_HOME_DIR < 0 && MIN_HARDWARE_ENDSTOP_Z && FEATURE_Z_PROBE && Z_PROBE_PIN == Z_MIN_PIN
        // Fix error from z probe testing
        zCorrection -= axisStepsPerMM[Z_AXIS] * EEPROM::zProbeHeight();
        // Correct from bed rotation
        //updateCurrentPosition(true);
        //float xt,yt,zt;
        //transformToPrinter(currentPosition[X_AXIS],currentPosition[Y_AXIS],0,xt,yt,zt);
        //zCorrection -= zt;
#endif
#if defined(ENDSTOP_Z_BACK_ON_HOME)
        // If we want to go up a bit more for some reason
        if (ENDSTOP_Z_BACK_ON_HOME > 0)
            zCorrection -= axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR;
#endif
#if Z_HOME_DIR < 0
            // Fix bed coating
#if Z_PROBE_Z_OFFSET_MODE == 0 // Only if measure through coating e.g. inductive
        zCorrection += axisStepsPerMM[Z_AXIS] * zBedOffset;
#endif
#endif
        //Com::printFLN(PSTR("Z-Correction-Steps:"),zCorrection); // TEST
        PrintLine::moveRelativeDistanceInSteps(0, 0, zCorrection, 0, homingFeedrate[Z_AXIS], true, false);
        currentPositionSteps[Z_AXIS] = ((Z_HOME_DIR == -1) ? zMinSteps : zMaxSteps - zBedOffset * axisStepsPerMM[Z_AXIS]);
#if NUM_EXTRUDER > 0
#if (EXTRUDER_IS_Z_PROBE == 0 || Z_HOME_DIR > 0)
        // currentPositionSteps[Z_AXIS] -= Extruder::current->zOffset;
        Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
#endif
#endif
#if DISTORTION_CORRECTION && Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        // Special case where z probe is z min end stop and distortion correction is enabled
        if (Printer::distortion.isEnabled()) {
            Printer::zCorrectionStepsIncluded = Printer::distortion.correct(Printer::currentPositionSteps[X_AXIS], currentPositionSteps[Y_AXIS], currentPositionSteps[Z_AXIS]);
            currentPositionSteps[Z_AXIS] += Printer::zCorrectionStepsIncluded;
        }
#endif
        updateCurrentPosition(true);
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
        // If we have software leveling enabled and are not at 0,0 z position is not zero, but we measured
        // for z = 0, so we need to correct for rotation.
        currentPositionSteps[Z_AXIS] -= (axisStepsPerMM[Z_AXIS] * currentPosition[Z_AXIS] - zMinSteps);
        currentPosition[Z_AXIS] = zMin;
#endif
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
#endif
        setZHomed(true);
#if FEATURE_BABYSTEPPING
        Printer::zBabysteps = 0;
#endif
    }
#if defined(SENSORLESS_HOMING) && TMC2130_ON_Z
    while (!Printer::tmc_driver_z->stst())
        ; // Wait for motor stand-still
    Printer::tmc_driver_z->coolstep_min_speed(coolstep_speed);
    Printer::tmc_driver_z->stealth_max_speed(stealth_max_sp);
    Printer::tmc_driver_z->stealthChop(stealth_state);
#endif
}

/** \brief Main function for all homing operations.

For homing operations only this function should be used. It calls Printer::homeXAxis, Printer::homeYAxis and Printer::homeZAxis
after doing some initialization work. The order of operation and some extra functions are controlled by HOMING_ORDER.

\param xaxis True if homing of x axis is wanted.
\param yaxis True if homing of y axis is wanted.
\param zaxis True if homing of z axis is wanted.
*/
void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) { // home non-delta printer
    unparkSafety();
    bool nocheck = isNoDestinationCheck();
    setNoDestinationCheck(true);
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    bool oldLaser = LaserDriver::laserOn;
    LaserDriver::laserOn = false;
#endif
    float startX, startY, startZ;
    realPosition(startX, startY, startZ);
#if DISTORTION_CORRECTION
    bool distOn = Printer::distortion.isEnabled();
    Printer::distortion.disable(false);
#endif
#if !defined(HOMING_ORDER)
#define HOMING_ORDER HOME_ORDER_XYZ
#endif
#if Z_HOME_DIR < 0
#if ZHOME_PRE_RAISE == 1
    if (zaxis && Endstops::zProbe()) {
        PrintLine::moveRelativeDistanceInSteps(0, 0, ZHOME_PRE_RAISE_DISTANCE * axisStepsPerMM[Z_AXIS], 0, homingFeedrate[Z_AXIS], true, true);
    }
#elif ZHOME_PRE_RAISE == 2
    if (zaxis) {
        PrintLine::moveRelativeDistanceInSteps(0, 0, ZHOME_PRE_RAISE_DISTANCE * axisStepsPerMM[Z_AXIS], 0, homingFeedrate[Z_AXIS], true, true);
    }
#endif
#endif
#if Z_HOME_DIR < 0 && Z_PROBE_PIN == Z_MIN_PIN && FEATURE_Z_PROBE
#if HOMING_ORDER != HOME_ORDER_XYZ && HOMING_ORDER != HOME_ORDER_YXZ && HOMING_ORDER != HOME_ORDER_ZXYTZ && HOMING_ORDER != HOME_ORDER_XYTZ
#error Illegal homing order for z probe based homing!
#endif
    if (zaxis) { // we need to know xy position for z probe to work properly
        if (!xaxis && !isXHomed())
            xaxis = true;
        if (!yaxis && !isYHomed())
            yaxis = true;
    }
#endif
    if (zaxis) {
        EVENT_BEFORE_Z_HOME;
    }
#if HOMING_ORDER == HOME_ORDER_XYZ
    if (xaxis)
        homeXAxis();
    if (yaxis)
        homeYAxis();
    if (zaxis)
        homeZAxis();
#elif HOMING_ORDER == HOME_ORDER_XZY
    if (xaxis)
        homeXAxis();
    if (zaxis)
        homeZAxis();
    if (yaxis)
        homeYAxis();
#elif HOMING_ORDER == HOME_ORDER_YXZ
    if (yaxis)
        homeYAxis();
    if (xaxis)
        homeXAxis();
    if (zaxis)
        homeZAxis();
#elif HOMING_ORDER == HOME_ORDER_YZX
    if (yaxis)
        homeYAxis();
    if (zaxis)
        homeZAxis();
    if (xaxis)
        homeXAxis();
#elif HOMING_ORDER == HOME_ORDER_ZXY
    if (zaxis)
        homeZAxis();
    if (xaxis)
        homeXAxis();
    if (yaxis)
        homeYAxis();
#elif HOMING_ORDER == HOME_ORDER_ZYX
    if (zaxis)
        homeZAxis();
    if (yaxis)
        homeYAxis();
    if (xaxis)
        homeXAxis();
#elif HOMING_ORDER == HOME_ORDER_ZXYTZ || HOMING_ORDER == HOME_ORDER_XYTZ
    {
#if ZHOME_MIN_TEMPERATURE > 20
        float actTemp[NUM_EXTRUDER];
        for (int i = 0; i < NUM_EXTRUDER; i++)
            actTemp[i] = extruder[i].tempControl.targetTemperatureC;
#endif
        if (zaxis) {
#if HOMING_ORDER == HOME_ORDER_ZXYTZ
            homeZAxis();
            Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]);
#endif
            Commands::waitUntilEndOfAllMoves();
#if ZHOME_MIN_TEMPERATURE > 20
#if ZHOME_HEAT_ALL
            for (int i = 0; i < NUM_EXTRUDER; i++) {
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZHOME_MIN_TEMPERATURE)), i, false, false);
            }
            for (int i = 0; i < NUM_EXTRUDER; i++) {
                if (extruder[i].tempControl.currentTemperatureC < ZHOME_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZHOME_MIN_TEMPERATURE)), i, false, true);
            }
#else
            if (extruder[Extruder::current->id].tempControl.currentTemperatureC < ZHOME_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZHOME_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
        }
#if ZHOME_X_POS == IGNORE_COORDINATE
        if (xaxis)
#else
        if (xaxis || zaxis)
#endif
        {
            homeXAxis();
            //#if ZHOME_X_POS == IGNORE_COORDINATE
            if (X_HOME_DIR < 0)
                startX = Printer::xMin;
            else
                startX = Printer::xMin + Printer::xLength;
            //#else
            //        startX = ZHOME_X_POS;
            //#endif
        }

#if ZHOME_Y_POS == IGNORE_COORDINATE
        if (yaxis)
#else
        if (yaxis || zaxis)
#endif
        {
            homeYAxis();
            //#if ZHOME_Y_POS == IGNORE_COORDINATE
            if (Y_HOME_DIR < 0)
                startY = Printer::yMin;
            else
                startY = Printer::yMin + Printer::yLength;
            //#else
            //        startY = ZHOME_Y_POS;
            //#endif
        }

        if (zaxis) {
#if ZHOME_X_POS != IGNORE_COORDINATE || ZHOME_Y_POS != IGNORE_COORDINATE
            moveToReal(ZHOME_X_POS, ZHOME_Y_POS, IGNORE_COORDINATE, IGNORE_COORDINATE, homingFeedrate[X_AXIS]); // correct rotation!
            Commands::waitUntilEndOfAllMoves();
#endif
            homeZAxis(); // real z distance at that point to zero
            if (Z_HOME_DIR < 0)
                startZ = Printer::zMin;
            else
                startZ = Printer::zMin + Printer::zLength - zBedOffset;
            moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]); // correct rotation!
            Commands::waitUntilEndOfAllMoves();
#if ZHOME_MIN_TEMPERATURE > 20
#if ZHOME_HEAT_ALL
            for (int i = 0; i < NUM_EXTRUDER; i++)
                Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
            for (int i = 0; i < NUM_EXTRUDER; i++)
                Extruder::setTemperatureForExtruder(actTemp[i], i, false, actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
            Extruder::setTemperatureForExtruder(actTemp[Extruder::current->id], Extruder::current->id, false, actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
        }
    }

#endif // elif HOMING_ORDER == HOME_ORDER_ZXYTZ || HOMING_ORDER == HOME_ORDER_XYTZ
#if HOMING_ORDER != HOME_ORDER_ZXYTZ && HOMING_ORDER != HOME_ORDER_XYTZ
    if (xaxis) {
#if DUAL_X_AXIS
        startX = currentPosition[X_AXIS];
#else
        if (X_HOME_DIR < 0)
            startX = Printer::xMin;
        else
            startX = Printer::xMin + Printer::xLength;
#endif // else DUAL_X_AXIS
    }
    if (yaxis) {
        if (Y_HOME_DIR < 0)
            startY = Printer::yMin;
        else
            startY = Printer::yMin + Printer::yLength;
    }
    if (zaxis) {
        if (Z_HOME_DIR < 0)
            startZ = Printer::zMin;
        else
            startZ = Printer::zMin + Printer::zLength - Printer::zBedOffset;
    }
#endif // HOMING_ORDER != HOME_ORDER_ZXYTZ
    updateCurrentPosition(true);
#if DISTORTION_CORRECTION
    if (distOn) {
        Printer::distortion.enable(false);
    }
#endif
#if defined(Z_UP_AFTER_HOME) && Z_HOME_DIR < 0
#ifdef HOME_ZUP_FIRST
    PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * Z_UP_AFTER_HOME * Z_HOME_DIR, 0, homingFeedrate[Z_AXIS], true, false);
#endif
    if (zaxis)
        startZ = Z_UP_AFTER_HOME;
#endif
    moveToReal(startX, startY, startZ, IGNORE_COORDINATE, homingFeedrate[X_AXIS]);
#if (DUAL_X_AXIS && LAZY_DUAL_X_AXIS)
    moveToReal(startX, startY, startZ, IGNORE_COORDINATE, homingFeedrate[X_AXIS]);
    if (!sledParked && xaxis) { // park sled
        homeXAxis();
    }
#endif
    updateCurrentPosition(true);
#if DUAL_X_AXIS && LAZY_DUAL_X_AXIS == 1
    lastCmdPos[X_AXIS] = xMin;
#endif
    updateHomedAll();
    UI_CLEAR_STATUS
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    LaserDriver::laserOn = oldLaser;
#endif
    setNoDestinationCheck(nocheck);
    Printer::updateCurrentPosition();
    Commands::printCurrentPosition();
}
#endif // Not delta printer

/** \brief Execute a open baby step.

If zBabystepsMissing is not 0 this will do a z step in the desired direction. The old movement directions
get restored after execution.
*/
void Printer::zBabystep() {
#if FEATURE_BABYSTEPPING
    bool dir = zBabystepsMissing > 0;
    if (dir)
        zBabystepsMissing--;
    else
        zBabystepsMissing++;
#if DRIVE_SYSTEM == DELTA
    Printer::enableXStepper();
    Printer::enableYStepper();
#endif
    Printer::enableZStepper();
    Printer::unsetAllSteppersDisabled();
#if DRIVE_SYSTEM == DELTA
    bool xDir = Printer::getXDirection();
    bool yDir = Printer::getYDirection();
#endif
    bool zDir = Printer::getZDirection();
#if DRIVE_SYSTEM == DELTA
    Printer::setXDirection(dir);
    Printer::setYDirection(dir);
#endif
    Printer::setZDirection(dir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
    HAL::delayMicroseconds(DIRECTION_DELAY);
#else
    HAL::delayMicroseconds(10);
#endif
#if DRIVE_SYSTEM == DELTA
    startXStep();
    startYStep();
#endif // Drive system 3
    startZStep();
    HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 2);
    Printer::endXYZSteps();
    HAL::delayMicroseconds(10);
#if DRIVE_SYSTEM == 3
    Printer::setXDirection(xDir);
    Printer::setYDirection(yDir);
#endif
    Printer::setZDirection(zDir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
    HAL::delayMicroseconds(DIRECTION_DELAY);
#endif
    //HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 1);
#endif
}

void Printer::setCaseLight(bool on) {
#if CASE_LIGHTS_PIN > -1
    WRITE(CASE_LIGHTS_PIN, on);
    lightOn = on;
    reportCaseLightStatus();
#endif
}

void Printer::reportCaseLightStatus() {
#if CASE_LIGHTS_PIN > -1
    if (lightOn)
        Com::printInfoFLN(PSTR("Case lights on"));
    else
        Com::printInfoFLN(PSTR("Case lights off"));
#else
    Com::printInfoFLN(PSTR("No case lights"));
#endif
}

void Printer::handleInterruptEvent() {
    if (interruptEvent == 0)
        return;
    int event = interruptEvent;
    interruptEvent = 0;
    switch (event) {
#if EXTRUDER_JAM_CONTROL
    case PRINTER_INTERRUPT_EVENT_JAM_DETECTED:
        if (isJamcontrolDisabled())
            break;
        EVENT_JAM_DETECTED;
        Com::printFLN(PSTR("important:Extruder jam detected"));
        UI_ERROR_P(Com::translatedF(UI_TEXT_EXTRUDER_JAM_ID));
#if JAM_ACTION == 1 // start dialog
        Printer::setUIErrorMessage(false);
#if UI_DISPLAY_TYPE != NO_DISPLAY
        uid.executeAction(UI_ACTION_WIZARD_JAM_EOF, true);
#endif
#elif JAM_ACTION == 2 // pause host/print
#if SDSUPPORT
        if (sd.sdmode == 2) {
            sd.pausePrint(true);
            break;
        }
#endif // SDSUPPORT
        GCodeSource::printAllFLN(PSTR("// action:out_of_filament T"), (int32_t)Extruder::current->id);
        GCodeSource::printAllFLN(PSTR("RequestPause:Extruder Jam Detected!"));
        // GCodeSource::printAllFLN(PSTR("// action:pause")); // add later when host/server know new meaning!
#endif // JAM_ACTION
        EVENT_JAM_DETECTED_END;
        break;
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL1:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL2:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL3:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL4:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL5: {
        if (isJamcontrolDisabled())
            break;
        fast8_t extruderIndex = event - PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0;
        Extruder& ext = extruder[extruderIndex];
        int32_t steps = abs(extruder[extruderIndex].jamStepsOnSignal);
        EVENT_JAM_SIGNAL_CHANGED(extruderIndex, steps);
        if (steps > ext.jamSlowdownSteps && !ext.tempControl.isSlowedDown()) {
            extruder[extruderIndex].tempControl.setSlowedDown(true);
            Commands::changeFeedrateMultiply(ext.jamSlowdownTo);
            //UI_ERROR_P(Com::tFilamentSlipping);
            UI_MESSAGE(4);
        }
        if (isDebugJam()) {
            Com::printF(PSTR("Jam signal steps:"), steps);
            int32_t percent = static_cast<int32_t>(steps) * 100 / JAM_STEPS;
            Com::printF(PSTR(" / "), percent);
            Com::printFLN(PSTR("% on "), (int)extruderIndex);
        }
    } break;
#endif // EXTRUDER_JAM_CONTROL    case PRINTER_INTERRUPT_EVENT_JAM_DETECTED:
    }
}

#define START_EXTRUDER_CONFIG(i) \
    Com::printF(Com::tConfig); \
    Com::printF(Com::tExtrDot, i + 1); \
    Com::print(':');
void Printer::showConfiguration() {
    Com::config(PSTR("Baudrate:"), baudrate);
#ifndef EXTERNALSERIAL
    Com::config(PSTR("InputBuffer:"), SERIAL_BUFFER_SIZE - 1);
#endif
    Com::config(PSTR("NumExtruder:"), NUM_EXTRUDER);
    Com::config(PSTR("MixingExtruder:"), MIXING_EXTRUDER);
    Com::config(PSTR("HeatedBed:"), HAVE_HEATED_BED);
    Com::config(PSTR("SDCard:"), SDSUPPORT);
    Com::config(PSTR("Fan:"), FAN_PIN > -1 && FEATURE_FAN_CONTROL);
#if FEATURE_FAN2_CONTROL && defined(FAN2_PIN) && FAN2_PIN > -1
    Com::config(PSTR("Fan2:1"));
#else
    Com::config(PSTR("Fan2:0"));
#endif
    Com::config(PSTR("LCD:"), FEATURE_CONTROLLER != NO_CONTROLLER);
    Com::config(PSTR("SoftwarePowerSwitch:"), PS_ON_PIN > -1);
    Com::config(PSTR("XHomeDir:"), X_HOME_DIR);
    Com::config(PSTR("YHomeDir:"), Y_HOME_DIR);
    Com::config(PSTR("ZHomeDir:"), Z_HOME_DIR);
#if DRIVE_SYSTEM == DELTA
    Com::config(PSTR("XHomePos:"), 0, 2);
    Com::config(PSTR("YHomePos:"), 0, 2);
    Com::config(PSTR("ZHomePos:"), zMin + zLength, 3);
#else
    Com::config(PSTR("XHomePos:"), xMin + (X_HOME_DIR > 0 ? xLength : 0), 2);
    Com::config(PSTR("YHomePos:"), yMin + (Y_HOME_DIR > 0 ? yLength : 0), 2);
    Com::config(PSTR("ZHomePos:"), zMin + (Z_HOME_DIR > 0 ? zLength : 0), 3);
#endif
    Com::config(PSTR("SupportG10G11:"), FEATURE_RETRACTION);
    Com::config(PSTR("SupportLocalFilamentchange:"), FEATURE_RETRACTION);
    Com::config(PSTR("CaseLights:"), CASE_LIGHTS_PIN > -1);
    Com::config(PSTR("ZProbe:"), FEATURE_Z_PROBE);
    Com::config(PSTR("Autolevel:"), FEATURE_AUTOLEVEL);
    Com::config(PSTR("EEPROM:"), EEPROM_MODE != 0);
    Com::config(PSTR("PrintlineCache:"), PRINTLINE_CACHE_SIZE);
    Com::config(PSTR("JerkXY:"), maxJerk);
    Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);
#if DRIVE_SYSTEM != DELTA
    Com::config(PSTR("JerkZ:"), maxZJerk);
#endif
#if FEATURE_RETRACTION
    Com::config(PSTR("RetractionLength:"), EEPROM_FLOAT(RETRACTION_LENGTH));
    Com::config(PSTR("RetractionLongLength:"), EEPROM_FLOAT(RETRACTION_LONG_LENGTH));
    Com::config(PSTR("RetractionSpeed:"), EEPROM_FLOAT(RETRACTION_SPEED));
    Com::config(PSTR("RetractionZLift:"), EEPROM_FLOAT(RETRACTION_Z_LIFT));
    Com::config(PSTR("RetractionUndoExtraLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH));
    Com::config(PSTR("RetractionUndoExtraLongLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH));
    Com::config(PSTR("RetractionUndoSpeed:"), EEPROM_FLOAT(RETRACTION_UNDO_SPEED));
#endif // FEATURE_RETRACTION
    Com::config(PSTR("XMin:"), xMin);
    Com::config(PSTR("YMin:"), yMin);
    Com::config(PSTR("ZMin:"), zMin);
    Com::config(PSTR("XMax:"), xMin + xLength);
    Com::config(PSTR("YMax:"), yMin + yLength);
    Com::config(PSTR("ZMax:"), zMin + zLength);
    Com::config(PSTR("XSize:"), xLength);
    Com::config(PSTR("YSize:"), yLength);
    Com::config(PSTR("ZSize:"), zLength);
    Com::config(PSTR("XPrintAccel:"), maxAccelerationMMPerSquareSecond[X_AXIS]);
    Com::config(PSTR("YPrintAccel:"), maxAccelerationMMPerSquareSecond[Y_AXIS]);
    Com::config(PSTR("ZPrintAccel:"), maxAccelerationMMPerSquareSecond[Z_AXIS]);
    Com::config(PSTR("XTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    Com::config(PSTR("YTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
    Com::config(PSTR("ZTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#if DRIVE_SYSTEM == DELTA
    Com::config(PSTR("PrinterType:Delta"));
#else
    Com::config(PSTR("PrinterType:Cartesian"));
#endif // DRIVE_SYSTEM
    Com::config(PSTR("MaxBedTemp:"), HEATED_BED_MAX_TEMP);
    for (fast8_t i = 0; i < NUM_EXTRUDER; i++) {
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Jerk:"), extruder[i].maxStartFeedrate);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxSpeed:"), extruder[i].maxFeedrate);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Acceleration:"), extruder[i].maxAcceleration);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Diameter:"), extruder[i].diameter);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxTemp:"), MAXTEMP);
    }
}

#if JSON_OUTPUT
void Printer::showJSONStatus(int type) {
    bool firstOccurrence;

    Com::printF(PSTR("{\"status\": \""));
    if (PrintLine::linesCount == 0) {
        Com::print('I'); // IDLING
#if SDSUPPORT
    } else if (sd.sdactive) {
        Com::print('P'); // SD PRINTING
#endif
    } else {
        Com::print('B'); // SOMETHING ELSE, BUT SOMETHIG
    }

    //  "heaters": [27.5, 30.3, 30.6],
    Com::printF(PSTR("\",\"heaters\":["));
#if HAVE_HEATED_BED
    Com::print(heatedBedController.currentTemperatureC);
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Com::print(',');
        Com::print(extruder[i].tempControl.currentTemperatureC);
    }
    //  "active": [65.0, 195.0, 0.0],
    Com::printF(PSTR("],\"active\":["));
#if HAVE_HEATED_BED
    Com::print(heatedBedController.targetTemperatureC);
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Com::print(',');
        Com::print(extruder[i].tempControl.targetTemperatureC);
    }
    //  "standby": [-273.1, 0.0, 150.0],
    Com::printF(PSTR("],\"standby\":["));
#if HAVE_HEATED_BED
    Com::print(heatedBedController.targetTemperatureC);
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Com::print(',');
        Com::print(extruder[i].tempControl.targetTemperatureC);
    }
    //  "hstat": [0, 0, 0],
    //  hstat is 0 for heater off, 1 for standby, 2 for active and 3 for fault. We have just added 4 for "being auto-tuned'
    Com::printF(PSTR("],\"hstat\":["));
#if HAVE_HEATED_BED
    if (heatedBedController.isSensorDefect() || heatedBedController.isSensorDecoupled())
        Com::print((int)3);
    else
        Com::print((int)(heatedBedController.targetTemperatureC < 30 ? 0 : 2));
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        Com::print(',');
        if (extruder[i].tempControl.isSensorDefect() || extruder[i].tempControl.isSensorDecoupled())
            Com::print((int)3);
        else
            Com::print((int)(extruder[i].tempControl.targetTemperatureC < 30 ? 0 : 2));
    }
    //  "pos": [1.00, 205.00, 6.48],
    Com::printF(PSTR("],\"pos\":["));
    Com::print(currentPosition[X_AXIS]); // X
    Com::print(',');
    Com::print(currentPosition[Y_AXIS]); // Y
    Com::print(',');
    Com::print(currentPosition[Z_AXIS]); // Z
    //  "extr": [0.0, 0.0],
    Com::printF(PSTR("],\"extr\":["));
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (i)
            Com::print(',');
        Com::printFloat(extruder[i].tempControl.currentTemperatureC, 1);
    }
    //  "sfactor": 100.00,
    Com::printF(PSTR("],\"sfactor\":"), Printer::feedrateMultiply);
    //  "efactor": [100.00, 100.00],
    Com::printF(PSTR(",\"efactor\":["));
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (i)
            Com::print(',');
        Com::print((int)Printer::extrudeMultiply);
    }
    //  "tool": 0,
    Com::printF(PSTR("],\"tool\":"), Extruder::current->id);
    //"probe": "4",
    Com::printF(PSTR(",\"probe\":"));
    if (Endstops::zProbe())
        Com::print((int)0);
    else
        Com::print((int)1000);
    //  "fanPercent": [0.00, 100.00],
    Com::printF(PSTR(",\"fanPercent\":["));
#if FEATURE_FAN_CONTROL
    Com::print(getFanSpeed() / 2.55f);
#endif
#if FEATURE_FAN2_CONTROL
    Com::printF(Com::tComma, getFan2Speed() / 2.55f);
#endif
    Com::printF(PSTR("]"));
    //  "fanRPM": 0,
    //  "homed": [1, 1, 1],
    Com::printF(PSTR(",\"homed\":["));
    Com::print((int)isXHomed());
    Com::print(',');
    Com::print(isYHomed());
    Com::print(',');
    Com::print(isZHomed());
    Com::printF(PSTR("]"));
    if (type == 1) {
        //  "geometry": "cartesian",
#if DRIVE_SYSTEM == DELTA
        Com::printF(PSTR(",\"geometry\":\"Delta\""));
#else
        Com::printF(PSTR(",\"geometry\":\"Cartesian\""));
#endif
        //  "myName": "Ormerod"
        Com::printF(PSTR(",\"myName\":\"" UI_PRINTER_NAME "\""));
        Com::printF(PSTR(",\"firmwareName\":\"Repetier\""));
    }
    Com::printF(PSTR(",\"coords\": {"));
    Com::printF(PSTR("\"axesHomed\":["));
    Com::print((int)isXHomed());
    Com::print(',');
    Com::print(isYHomed());
    Com::print(',');
    Com::print(isZHomed());
    Com::printF(PSTR("],\"extr\":["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print(extruder[i].extrudePosition / extruder[i].stepsPerMM);
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"xyz\":["));
    Com::print(currentPosition[X_AXIS]); // X
    Com::print(',');
    Com::print(currentPosition[Y_AXIS]); // Y
    Com::print(',');
    Com::print(currentPosition[Z_AXIS]); // Z
    Com::printF(PSTR("]},\"currentTool\":"));
    Com::print(Extruder::current->id);
    Com::printF(PSTR(",\"params\": {\"atxPower\":"));
    Com::print(isPowerOn() ? '1' : '0');
    Com::printF(PSTR(",\"fanPercent\":["));
#if FEATURE_FAN_CONTROL
    Com::print(getFanSpeed() / 2.55f);
#endif
#if FEATURE_FAN2_CONTROL
    Com::printF(Com::tComma, getFan2Speed() / 2.55f);
#endif
    Com::printF(PSTR("],\"speedFactor\":"));
    Com::print(Printer::feedrateMultiply);
    Com::printF(PSTR(",\"extrFactors\":["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print((int)Printer::extrudeMultiply); // Really *100? 100 is normal
        firstOccurrence = false;
    }
    Com::printF(PSTR("]},"));
    // SEQ??
    Com::printF(PSTR("\"temps\": {"));
#if HAVE_HEATED_BED
    Com::printF(PSTR("\"bed\": {\"current\":"));
    Com::print(heatedBedController.currentTemperatureC);
    Com::printF(PSTR(",\"active\":"));
    Com::print(heatedBedController.targetTemperatureC);
    Com::printF(PSTR(",\"state\":"));
    Com::print(heatedBedController.targetTemperatureC > 0 ? '2' : '1');
    Com::printF(PSTR("},"));
#endif
    Com::printF(PSTR("\"heads\": {\"current\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print(extruder[i].tempControl.currentTemperatureC);
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"active\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print(extruder[i].tempControl.targetTemperatureC);
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"state\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_EXTRUDER; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print(extruder[i].tempControl.targetTemperatureC > EXTRUDER_FAN_COOL_TEMP ? '2' : '1');
        firstOccurrence = false;
    }
    Com::printF(PSTR("]}},\"time\":"));
    Com::print(HAL::timeInMilliseconds());

    switch (type) {
    default:
    case 0:
    case 1:
        break;
    case 2:
        // UNTIL PRINT ESTIMATE TIMES ARE IMPLEMENTED
        // NO DURATION INFO IS SUPPORTED
        Com::printF(PSTR(",\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\""));
#if (DRIVE_SYSTEM == DELTA)
        Com::printF(PSTR("delta"));
#elif (DRIVE_SYSTEM == CARTESIAN || DRIVE_SYSTEM == GANTRY_FAKE)
        Com::printF(PSTR("cartesian"));
#elif ((DRIVE_SYSTEM == XY_GANTRY) || (DRIVE_SYSTEM == YX_GANTRY))
        Com::printF(PSTR("coreXY"));
#elif (DRIVE_SYSTEM == XZ_GANTRY)
        Com::printF(PSTR("coreXZ"));
#endif
        Com::printF(PSTR("\",\"name\":\""));
        Com::printF(PSTR(UI_PRINTER_NAME));
        Com::printF(PSTR("\",\"tools\":["));
        firstOccurrence = true;
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            if (!firstOccurrence)
                Com::print(',');
            Com::printF(PSTR("{\"number\":"));
            Com::print(i);
            Com::printF(PSTR(",\"heaters\":[1],\"drives\":[1]"));
            Com::print('}');
            firstOccurrence = false;
        }
        Com::printF(PSTR("]"));
        break;
    case 3:
        Com::printF(PSTR(",\"currentLayer\":"));
#if SDSUPPORT
        if (sd.sdactive && sd.fileInfo.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            Com::print((int)(currentPosition[Z_AXIS] / sd.fileInfo.layerHeight));
        } else
            Com::print('0');
#else
        Com::printF(PSTR("-1"));
#endif
        Com::printF(PSTR(",\"extrRaw\":["));
        firstOccurrence = true;
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            if (!firstOccurrence)
                Com::print(',');
            Com::print(extruder[i].extrudePosition * Printer::extrudeMultiply);
            firstOccurrence = false;
        }
        Com::printF(PSTR("],"));
#if SDSUPPORT
        if (sd.sdactive) {
            Com::printF(PSTR("\"fractionPrinted\":"));
            float fraction;
            if (sd.filesize < 2000000)
                fraction = sd.sdpos / sd.filesize;
            else
                fraction = (sd.sdpos >> 8) / (sd.filesize >> 8);
            Com::print((float)floor(fraction * 1000) / 1000); // ONE DECIMAL, COULD BE DONE BY SHIFTING, BUT MEH
            Com::print(',');
        }
#endif
        Com::printF(PSTR("\"firstLayerHeight\":"));
#if SDSUPPORT
        if (sd.sdactive) {
            Com::print(sd.fileInfo.layerHeight);
        } else
            Com::print('0');
#else
        Com::print('0');
#endif
        break;
    case 4:
    case 5:
        Com::printF(PSTR(",\"axisMins\":["));
        Com::print((int)X_MIN_POS);
        Com::print(',');
        Com::print((int)Y_MIN_POS);
        Com::print(',');
        Com::print((int)Z_MIN_POS);
        Com::printF(PSTR("],\"axisMaxes\":["));
        Com::print((int)X_MAX_LENGTH);
        Com::print(',');
        Com::print((int)Y_MAX_LENGTH);
        Com::print(',');
        Com::print((int)Z_MAX_LENGTH);
        Com::printF(PSTR("],\"accelerations\":["));
        Com::print(maxAccelerationMMPerSquareSecond[X_AXIS]);
        Com::print(',');
        Com::print(maxAccelerationMMPerSquareSecond[Y_AXIS]);
        Com::print(',');
        Com::print(maxAccelerationMMPerSquareSecond[Z_AXIS]);
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            Com::print(',');
            Com::print(extruder[i].maxAcceleration);
        }
        Com::printF(PSTR("],\"firmwareElectronics\":\""));
#ifdef RAMPS_V_1_3
        Com::printF(PSTR("RAMPS"));
#elif (CPU_ARCH == ARCH_ARM)
        Com::printF(PSTR("Arduino Due"));
#else
        Com::printF(PSTR("AVR"));
#endif
        Com::printF(PSTR("\",\"firmwareName\":\"Repetier\",\"firmwareVersion\":\""));
        Com::printF(PSTR(REPETIER_VERSION));
        Com::printF(PSTR("\",\"minFeedrates\":[0,0,0"));
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            Com::printF(PSTR(",0"));
        }
        Com::printF(PSTR("],\"maxFeedrates\":["));
        Com::print(maxFeedrate[X_AXIS]);
        Com::print(',');
        Com::print(maxFeedrate[Y_AXIS]);
        Com::print(',');
        Com::print(maxFeedrate[Z_AXIS]);
        for (int i = 0; i < NUM_EXTRUDER; i++) {
            Com::print(',');
            Com::print(extruder[i].maxFeedrate);
        }
        Com::printF(PSTR("]"));
        break;
    }

    Com::printFLN(PSTR("}"));
}

#endif // JSON_OUTPUT

void Printer::pausePrint() {
#if SDSUPPORT
    if (Printer::isMenuMode(MENU_MODE_SD_PRINTING)) {
        sd.pausePrint(true);
    } else
#endif
        if (Printer::isMenuMode(MENU_MODE_PRINTING)) {
        GCodeSource::printAllFLN(PSTR("RequestPause:"));
        Printer::setMenuMode(MENU_MODE_PAUSED, true);
#if !defined(DISABLE_PRINTMODE_ON_PAUSE) || DISABLE_PRINTMODE_ON_PAUSE == 1
        Printer::setPrinting(false);
#endif
    }
}

void Printer::continuePrint() {
#if SDSUPPORT
    if (Printer::isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED)) {
        sd.continuePrint(true);
    } else
#endif
        if (Printer::isMenuMode(MENU_MODE_PAUSED)) {
        GCodeSource::printAllFLN(PSTR("RequestContinue:"));
    }
    setMenuMode(MENU_MODE_PAUSED, false);
}

void Printer::stopPrint() {
#if NEW_COMMUNICATION
    flashSource.close(); // stop flash printing if busy
#endif
#if SDSUPPORT
    if (Printer::isMenuMode(MENU_MODE_SD_PRINTING)) {
        sd.stopPrint();
    } else
#endif
    {
        GCodeSource::printAllFLN(PSTR("RequestStop:"));
    }
    if (!isUIErrorMessage()) {
        UI_RESET_MENU
    }
}

/** Rescue system uses the following layout in eeprom:
1 : mode
4 * NUM_AXES : Last received position
4 * NUM_AXES : Last position
4 * NUM_AXES : Last Offsets
*/

void Printer::enableRescue(bool on) {
#if HOST_RESCUE
    if (on && HAL::eprGetByte(EPR_RESCUE_MODE)) {
        HAL::eprSetByte(EPR_RESCUE_MODE, 0);
    }
    rescueOn = on;
    if (!on) {
        unparkSafety();
        UI_CLEAR_STATUS;
    }
    rescueReset(); // enabling/disabling removes old values
#endif
}
bool Printer::isRescue() {
#if HOST_RESCUE
    return rescueOn;
#else
    return false;
#endif
}
void Printer::rescueReport() {
#if HOST_RESCUE
    uint8_t mode = HAL::eprGetByte(EPR_RESCUE_MODE);
    Com::printF(PSTR("RESCUE_STATE:"));
    if (mode & 1) {
        for (fast8_t i = 0; i < 4; i++) {
            Com::print(' ');
            Com::print('L');
            Com::printF(axisNames[i]);
            Com::printF(Com::tColon, HAL::eprGetFloat(EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i), 2);
        }
        Com::printF(PSTR(" LT:"), (int)HAL::eprGetByte(EPR_RESCUE_TOOL));
    }
    if (mode & 2) {
        for (fast8_t i = 0; i < 4; i++) {
            Com::print(' ');
            Com::printF(axisNames[i]);
            Com::printF(Com::tColon, HAL::eprGetFloat(EPR_RESCUE_LAST_POS + sizeof(float) * i), 2);
        }
    }
    if (mode == 0) {
        Com::printFLN(PSTR(" OFF"));
    }
    Com::println();
#endif
}
void Printer::rescueStoreReceivedPosition() {
#if HOST_RESCUE
    if (!rescueOn) {
        return;
    }
    for (fast8_t i = 0; i < 4; i++) {
        if (i == 3) {
            HAL::eprSetFloat(EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i, Printer::currentPositionTransformed[i]);
        } else {
            HAL::eprSetFloat(EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i, Printer::lastCmdPos[i]);
        }
    }
    HAL::eprSetByte(EPR_RESCUE_TOOL, Extruder::current->id);
    HAL::eprSetByte(EPR_RESCUE_MODE, HAL::eprGetByte(EPR_RESCUE_MODE) | 1);
#endif
}
void Printer::rescueStorePosition() {
#if HOST_RESCUE
    if (!rescueOn) {
        return;
    }
    for (fast8_t i = 0; i < 4; i++) {

        if (i != E_AXIS) {
            HAL::eprSetFloat(EPR_RESCUE_LAST_POS + sizeof(float) * i, Printer::currentPosition[i]);
            HAL::eprSetFloat(EPR_RESCUE_OFFSETS + sizeof(float) * i, Printer::coordinateOffset[i]);
        } else {
            HAL::eprSetFloat(EPR_RESCUE_LAST_POS + sizeof(float) * i, Printer::currentPositionTransformed[i]);
        }
    }
    HAL::eprSetByte(EPR_RESCUE_MODE, HAL::eprGetByte(EPR_RESCUE_MODE) | 2);
#endif
}
void Printer::rescueRecover() {
#if HOST_RESCUE
    uint8_t mode = HAL::eprGetByte(EPR_RESCUE_MODE);
    if (mode == 0) {
        return;
    }
    if (mode & 2) {
        for (fast8_t i = 0; i < 4; i++) {
            if (i < 3) {
                Printer::coordinateOffset[i] = HAL::eprGetFloat(EPR_RESCUE_OFFSETS + sizeof(float) * i);
                Printer::currentPosition[i] = HAL::eprGetFloat(EPR_RESCUE_LAST_POS + sizeof(float) * i);
            } else {
                Printer::currentPositionTransformed[i] = HAL::eprGetFloat(EPR_RESCUE_LAST_POS + sizeof(float) * i);
            }
        }
        flag3 |= PRINTER_FLAG3_X_HOMED | PRINTER_FLAG3_Y_HOMED | PRINTER_FLAG3_Z_HOMED;
        flag1 |= PRINTER_FLAG1_HOMED_ALL;
    } else if (mode & 1) {
        for (fast8_t i = 0; i < 4; i++) {
            if (i < 3) {
                Printer::currentPosition[i] = HAL::eprGetFloat(EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i);
            } else {
                Printer::currentPositionTransformed[i] = HAL::eprGetFloat(EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i);
            }
        }
        flag3 |= PRINTER_FLAG3_X_HOMED | PRINTER_FLAG3_Y_HOMED | PRINTER_FLAG3_Z_HOMED;
        flag1 |= PRINTER_FLAG1_HOMED_ALL;
    }
    lastCmdPos[X_AXIS] = currentPosition[X_AXIS];
    lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS];
    lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS];
    updateCurrentPositionSteps();
#endif
}
int Printer::rescueStartTool() {
#if HOST_RESCUE
    if (HAL::eprGetByte(EPR_RESCUE_MODE)) {
        return static_cast<int>(HAL::eprGetByte(EPR_RESCUE_TOOL));
    } else {
        return 0;
    }
#else
    return 0;
#endif
}
void Printer::rescueSetup() {
#if HOST_RESCUE
    uint8_t mode = HAL::eprGetByte(EPR_RESCUE_MODE);
    if (mode) {
        rescueReport();
        enableXStepper();
        enableYStepper();
        enableZStepper();
        Extruder::enableCurrentExtruderMotor();
        rescueRecover();
    }
#endif
}

bool Printer::isRescueRequired() {
#if HOST_RESCUE
    return HAL::eprGetByte(EPR_RESCUE_MODE) != 0;
#else
    return false;
#endif
}
void Printer::rescueReset() {
#if HOST_RESCUE
    uint8_t mode = HAL::eprGetByte(EPR_RESCUE_MODE);
    if (mode) {
        HAL::eprSetByte(EPR_RESCUE_MODE, 0);
    }
#endif
}

void Printer::handlePowerLoss() {
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        Extruder::setTemperatureForExtruder(0, i);
    }
    Extruder::setHeatedBedTemperature(0);
    Com::printInfoFLN(PSTR("POWERLOSS_DETECTED"));
    if (rescueOn) {
        rescueStoreReceivedPosition();
#if POWERLOSS_LEVEL == 1 //z up only
        parkSafety(true);
#endif
#if POWERLOSS_LEVEL == 2 // full park - we are on a UPC :-)
        parkSafety(false);
#endif
        Commands::waitUntilEndOfAllMoves();
        rescueStorePosition();
        uint8_t old3 = flag3;
        kill(false);
        enableFailedModeP(PSTR("Power Loss"));
        rescueReport(); // in case we do survive it tell server position
        flag3 = old3;   // restore homed state
    }
}

void Printer::parkSafety(bool zOnly) {
    if (safetyParked || !rescueOn) {
        return; // nothing to unpark
    }
    rescueStoreReceivedPosition();
    safetyParked = 1;
    MemoryPosition();
    moveToParkPosition(zOnly);
    Commands::waitUntilEndOfAllMoves();
    rescueStorePosition();
    safetyParked = 2;
    Extruder::pauseExtruders(false);
    // GUI::setStatusP(PSTR("Waiting f. Data..."), GUIStatusLevel::BUSY); // inform user
}

void Printer::unparkSafety() {
    if (!safetyParked) {
        return; // nothing to unpark
    }
    Extruder::unpauseExtruders(true);
    bool needsPop = safetyParked > 1;
    safetyParked = 0;
    GoToMemoryPosition(true, true, false, false, Printer::maxFeedrate[X_AXIS]);
    GoToMemoryPosition(false, false, true, false, Printer::maxFeedrate[Z_AXIS] * 0.5);
    UI_CLEAR_STATUS;
}
void Printer::enableFailedModeP(PGM_P msg) {
    failedMode = true;
    UI_ERROR_P(msg);
    Com::printErrorFLN(msg);
    Com::printErrorFLN(Com::tM999);
}
void Printer::enableFailedMode(char* msg) {
    failedMode = true;
    UI_ERROR_RAM(msg);
    Com::printErrorF(Com::tEmpty);
    Com::print(msg);
    Com::println();
    Com::printErrorFLN(Com::tM999);
}

#if defined(DRV_TMC2130)
void Printer::configTMC2130(TMC2130Stepper* tmc_driver, bool tmc_stealthchop, int8_t tmc_sgt,
                            uint8_t tmc_pwm_ampl, uint8_t tmc_pwm_grad, bool tmc_pwm_autoscale, uint8_t tmc_pwm_freq) {
    tmc_driver->begin(); // Initiate pins and registries
    tmc_driver->test_connection();
    if (tmc_driver->test_connection() != 0) {
        Printer::kill(false);
    }
    TRINAMIC_WAIT_FOR_STANDSTILL(tmc_driver, 100) // Wait for motor stand-still
    tmc_driver->I_scale_analog(true);             // Set current reference source		// Using internal reference should be good enough and work on more drivers
    // tmc_driver->I_scale_analog(true);               // Set current reference source
    tmc_driver->interpolate(true);                // Set internal micro step interpolation
    tmc_driver->pwm_ampl(tmc_pwm_ampl);           // Chopper PWM amplitude
    tmc_driver->pwm_grad(tmc_pwm_grad);           // Velocity gradient for chopper PWM amplitude
    tmc_driver->pwm_autoscale(tmc_pwm_autoscale); // Chopper PWM autos scaling
    tmc_driver->pwm_freq(tmc_pwm_freq);           // Chopper PWM frequency selection
    tmc_driver->stealthChop(tmc_stealthchop);     // Enable extremely quiet stepping
    tmc_driver->sg_stall_value(tmc_sgt);          // StallGuard sensitivity
}

#if defined(SENSORLESS_HOMING)
void Printer::tmcPrepareHoming(TMC2130Stepper* tmc_driver, uint32_t coolstep_sp_min) {
    while (!tmc_driver->stst())
        ;                                            // Wait for motor stand-still
    tmc_driver->stealth_max_speed(0);                // Upper speed limit for stealthChop
    tmc_driver->stealthChop(false);                  // Turn off stealthChop
    tmc_driver->coolstep_min_speed(coolstep_sp_min); // Minimum speed for StallGuard triggering
    tmc_driver->sg_filter(false);                    // Turn off StallGuard filtering
    tmc_driver->diag1_stall(true);                   // Signal StallGuard on DIAG1 pin
#if MOTHERBOARD != 310                               // Rambo Einsy has diag0 and diag1 bound together so this could cause a defect
    tmc_driver->diag1_active_high(true);             // StallGuard pulses active high
#endif
}
#endif

#endif
