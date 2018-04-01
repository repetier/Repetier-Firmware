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

PWMHandler* fans[] = FAN_LIST;
#if USE_ADVANCE
ufast8_t Printer::maxExtruderSpeed;        ///< Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//uint8_t Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
#if DRIVE_SYSTEM != DELTA
int32_t Printer::zCorrectionStepsIncluded = 0;
#endif
#if FEATURE_BABYSTEPPING
int16_t Printer::zBabystepsMissing = 0;
int16_t Printer::zBabysteps = 0;
#endif
uint8_t Printer::relativeCoordinateMode = false;         ///< Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false; ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
fast8_t Printer::stepsPerTimerCall = 1;
uint16_t Printer::menuMode = 0;
uint8_t Printer::mode = DEFAULT_PRINTER_MODE;
float Printer::extrudeMultiplyError = 0;
float Printer::extrusionFactor = 1.0;
uint8_t Printer::interruptEvent = 0;
int Printer::currentLayer = 0;
int Printer::maxLayer = -1;       // -1 = unknown
char Printer::printName[21] = ""; // max. 20 chars + 0
float Printer::progress = 0;
millis_t Printer::lastTempReport = 0;
int32_t Printer::printingTime = 0;

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
float Printer::feedrate;                 ///< Last requested feedrate.
int Printer::feedrateMultiply;           ///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int Printer::extrudeMultiply;   ///< Flow multiplier in percent (factor 1 = 100)
float Printer::offsetX;                  ///< X-offset for different extruder positions.
float Printer::offsetY;                  ///< Y-offset for different extruder positions.
float Printer::offsetZ;                  ///< Z-offset for different extruder positions.
float Printer::offsetZ2 = 0;             ///< Z-offset without rotation correction.
speed_t Printer::vMaxReached;            ///< Maximum reached speed
uint32_t Printer::msecondsPrinting;      ///< Milliseconds of printing time (means time with heated extruder)
float Printer::filamentPrinted;          ///< mm of filament printed since counting started
float Printer::filamentPrintedTotal = 0; ///< mm of filament printed since counting started
#if ENABLE_BACKLASH_COMPENSATION
float Printer::backlashX;
float Printer::backlashY;
float Printer::backlashZ;
uint8_t Printer::backlashDir;
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
#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif
#if LAZY_DUAL_X_AXIS
bool Printer::sledParked = false;
#endif
fast8_t Printer::wizardStackPos;
wizardVar Printer::wizardStack[WIZARD_STACK_SIZE];

void Printer::setDebugLevel(uint8_t newLevel) {
    if (newLevel != debugLevel) {
        debugLevel = newLevel;
        if (debugDryrun()) {
            // Disable all heaters in case they were on
            HeatManager::disableAllHeaters();
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

int Printer::getFanSpeed(int fanId) {
    if (fanId < 0 || fanId >= NUM_FANS) {
        return 0;
    }
    return (int)fans[fanId]->get();
}

void Printer::setFanSpeedDirectly(uint8_t speed, int fanId) {
    uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
    if (fanId < 0 || fanId >= NUM_FANS) {
        return;
    }
    fans[fanId]->set(trimmedSpeed);
}

bool Printer::updateDoorOpen() {
#if defined(DOOR_PIN) && DOOR_PIN > -1 //  && SUPPORT_LASER should always be respected
    bool isOpen = isDoorOpen();
    uint8_t b = READ(DOOR_PIN) != DOOR_INVERTING;
    if (!b && isOpen) {
        UI_STATUS_F(Com::tSpace);
    } else if (b && !isOpen) {
        Com::printWarningFLN(Com::tDoorOpen);
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
#else
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
    XMotor.disable();
    YMotor.disable();
#if !defined(PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT)
    ZMotor.disable();
#else
    if (!onlySteppers)
        ZMotor.disable();
#endif
    Tool::disableMotors();
    setAllSteppersDiabled();
    unsetHomedAll();
    if (!onlySteppers) {
        for (uint8_t i = 0; i < NUM_TOOLS; i++) {
            Tool::getTool(i)->shutdown();
        }
        for (uint8_t i = 0; i < NUM_HEATED_BEDS; i++) {
            heatedBeds[i]->setTargetTemperature(0);
        }
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_STANDBY_ID));
#if defined(PS_ON_PIN) && PS_ON_PIN > -1 && !defined(NO_POWER_TIMEOUT)
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
        Printer::setPowerOn(false);
#endif
        Printer::setAllKilled(true);
    } else {
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_STEPPER_DISABLED_ID));
    }
    Commands::printTemperatures(false);
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
uint8_t Printer::moveTo(float x, float y, float z, float e, float f) {
    if (f != IGNORE_COORDINATE)
        feedrate = f;
    Motion1::setTmpPositionXYZE(x, y, z, e);
    Motion1::moveByPrinter(Motion1::tmpPosition, feedrate);
    return 1;
}

uint8_t Printer::moveToReal(float x, float y, float z, float e, float f, bool pathOptimize) {
    if (f != IGNORE_COORDINATE)
        feedrate = f;
    Motion1::setTmpPositionXYZE(x, y, z, e);
    Motion1::moveByOfficial(Motion1::tmpPosition, feedrate);
    return 1;
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
    Motion1::g92Offsets[X_AXIS] = xOff; // offset from G92
    Motion1::g92Offsets[Y_AXIS] = yOff;
    Motion1::g92Offsets[Z_AXIS] = zOff;
}

/** \brief Sets the destination coordinates to values stored in com.

Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
\param com g-code with new destination position.
\return true if it is a move, false if no move results from coordinates.
 */

void Printer::setDestinationStepsFromGCode(GCode* com) {
    float p;
    float x, y, z;
    bool posAllowed = true;
    float coords[NUM_AXES];
    Motion1::copyCurrentOfficial(coords);

#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
    if (!isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
        if (!Motion1::isAxisHomed(X_AXIS))
            com->unsetX();
#endif
#if MOVE_Y_WHEN_HOMED
        if (!Motion1::isAxisHomed(Y_AXIS))
            com->unsetY();
#endif
#if MOVE_Z_WHEN_HOMED
        if (!Motion1::isAxisHomed(Z_AXIS))
            com->unsetZ();
#endif
#if NUM_AXES > A_AXIS && MOVE_A_WHEN_HOMED
        if (!Motion1::isAxisHomed(A_AXIS))
            com->unsetA();
#endif
#if NUM_AXES > B_AXIS && MOVE_B_WHEN_HOMED
        if (!Motion1::isAxisHomed(B_AXIS))
            com->unsetB();
#endif
#if NUM_AXES > C_AXIS && MOVE_C_WHEN_HOMED
        if (!Motion1::isAxisHomed(C_AXIS))
            com->unsetC();
#endif
    }
#endif
    if (!relativeCoordinateMode) {
        if (com->hasX())
            coords[X_AXIS] = convertToMM(com->X) - Motion1::g92Offsets[X_AXIS];
        if (com->hasY())
            coords[Y_AXIS] = convertToMM(com->Y) - Motion1::g92Offsets[Y_AXIS];
        if (com->hasZ())
            coords[Z_AXIS] = convertToMM(com->Z) - Motion1::g92Offsets[Z_AXIS];
#if NUM_AXES > A_AXIS
        if (com->hasA())
            coords[A_AXIS] = convertToMM(com->A) - Motion1::g92Offsets[A_AXIS];
#endif
#if NUM_AXES > B_AXIS
        if (com->hasB())
            coords[B_AXIS] = convertToMM(com->B) - Motion1::g92Offsets[B_AXIS];
#endif
#if NUM_AXES > C_AXIS
        if (com->hasC())
            coords[C_AXIS] = convertToMM(com->C) - Motion1::g92Offsets[C_AXIS];
#endif
    } else {
        if (com->hasX())
            coords[X_AXIS] += convertToMM(com->X);
        if (com->hasY())
            coords[Y_AXIS] += convertToMM(com->Y);
        if (com->hasZ())
            coords[Z_AXIS] += convertToMM(com->Z);
#if NUM_AXES > A_AXIS
        if (com->hasA())
            coords[A_AXIS] += convertToMM(com->A);
#endif
#if NUM_AXES > B_AXIS
        if (com->hasB())
            coords[B_AXIS] += convertToMM(com->B);
#endif
#if NUM_AXES > C_AXIS
        if (com->hasC())
            coords[C_AXIS] += convertToMM(com->C);
#endif
    }
    if (com->hasE() && !Printer::debugDryrun()) {
        p = convertToMM(com->E);
        HeatManager* heater = Tool::getActiveTool()->getHeater();
        if (relativeCoordinateMode || relativeExtruderCoordinateMode) {
            if (fabs(com->E) * extrusionFactor > EXTRUDE_MAXLENGTH) {
                Com::printWarningF(PSTR("MAx. extrusion distance per move exceeded - ignoring move."));
                p = 0;
            }
            coords[E_AXIS] = Motion1::currentPosition[E_AXIS] + p;
        } else {
            if (fabs(p - Motion1::currentPosition[E_AXIS]) * extrusionFactor > EXTRUDE_MAXLENGTH) {
                p = Motion1::currentPosition[E_AXIS];
            }
            coords[E_AXIS] = p;
        }
    } else {
        coords[E_AXIS] = Motion1::currentPosition[E_AXIS];
    }
    if (com->hasF() && com->F > 0.1) {
        if (unitIsInches)
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply; // Factor is 25.5/60/100
        else
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
    }
    Motion1::moveByOfficial(coords, Printer::feedrate);
}

void Printer::setup() {
    HAL::stopWatchdog();

    // Start serial
    HAL::hwSetup();
    // Main board specific extra initialization
#if defined(MB_SETUP)
    MB_SETUP;
#endif

    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);
    Com::printFLN(Com::tStart);
    HAL::showStartReason();
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
    // Define io functions
#undef IO_TARGET
#define IO_TARGET 1
#include "src/io/redefine.h"

    Motion1::init();
    Motion2::init();
    Motion3::init();
    ZProbeHandler::init();
    PrinterType::init();
    Tool::initTools();
#if FEATURE_CONTROLLER == CONTROLLER_VIKI
    HAL::delayMilliseconds(100);
#endif // FEATURE_CONTROLLER
#if UI_DISPLAY_TYPE != NO_DISPLAY
    Com::selectLanguage(0); // just make sure we have a language in case someone uses it early
#endif
    //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization

    EVENT_INITIALIZE_EARLY

#if defined(DOOR_PIN) && DOOR_PIN > -1
    SET_INPUT(DOOR_PIN);
#if defined(DOOR_PULLUP) && DOOR_PULLUP
    PULLUP(DOOR_PIN, HIGH);
#endif
#endif

#if FEATURE_Z_PROBE && Z_PROBE_PIN > -1
    SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
    PULLUP(Z_PROBE_PIN, HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE

#if CASE_LIGHTS_PIN >= 0
    SET_OUTPUT(CASE_LIGHTS_PIN);
    WRITE(CASE_LIGHTS_PIN, CASE_LIGHT_DEFAULT_ON);
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

#ifdef RED_BLUE_STATUS_LEDS
    SET_OUTPUT(RED_STATUS_LED);
    SET_OUTPUT(BLUE_STATUS_LED);
    WRITE(BLUE_STATUS_LED, HIGH);
    WRITE(RED_STATUS_LED, LOW);
#endif // RED_BLUE_STATUS_LEDS
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
    motorCurrentControlInit(); // Set current if it is firmware controlled
#endif
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    initializeAllMotorDrivers();
#endif // defined
    microstepInit();
    feedrate = 50; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
    extrudeMultiply = 100;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    advanceExecuted = 0;
#endif
    advanceStepsSet = 0;
#endif
    offsetX = offsetY = offsetZ = 0;
    interval = 5000;
    stepsPerTimerCall = 1;
    msecondsPrinting = 0;
    filamentPrinted = 0;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
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
    EEPROM::init(); // Read settings from eeprom if wanted, run after initialization!
    HAL::analogStart();
    // Extruder::initExtruder();
    // sets auto leveling in eeprom init
    UI_INITIALIZE;
    //Commands::printCurrentPosition();
#if DISTORTION_CORRECTION
    distortion.init();
#endif // DISTORTION_CORRECTION

    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::delayMilliseconds(20);
    HAL::setupTimer();
    HAL::delayMilliseconds(20);

#if SDSUPPORT
    sd.mount();
#endif
#if FEATURE_WATCHDOG
    HAL::startWatchdog();
#endif // FEATURE_WATCHDOG
    HAL::delayMilliseconds(20);

    Tool::selectTool(0);
    // Extruder::selectExtruderById(0);
    HAL::delayMilliseconds(20);

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
    HAL::delayMilliseconds(20);
}

void Printer::defaultLoopActions() {
    Commands::checkForPeriodicalActions(true); //check heater every n milliseconds
    UI_MEDIUM;                                 // do check encoder
    millis_t curtime = HAL::timeInMilliseconds();
    if (Motion1::length != 0 || isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED))
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

void Printer::setCaseLight(bool on) {
#if CASE_LIGHTS_PIN > -1
    WRITE(CASE_LIGHTS_PIN, on);
    reportCaseLightStatus();
#endif
}

void Printer::reportCaseLightStatus() {
#if CASE_LIGHTS_PIN > -1
    if (READ(CASE_LIGHTS_PIN))
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
        GCodeSource::printAllFLN(PSTR("RequestPause:Extruder Jam Detected!"));
#endif // JAM_ACTION
        EVENT_JAM_DETECTED_END;
        break;
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL1:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL2:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL3:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL4:
    case PRINTER_INTERRUPT_EVENT_JAM_SIGNAL5: {
        // TODO: Jam control
       /* if (isJamcontrolDisabled())
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
        } */
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
    Com::config(PSTR("HeatedBed:"), NUM_HEATED_BEDS);
    Com::config(PSTR("SDCard:"), SDSUPPORT);
    Com::config(PSTR("Fan:"), NUM_FANS > 0);
#if NUM_FANS > 1
    Com::config(PSTR("Fan2:1"));
#else
    Com::config(PSTR("Fan2:0"));
#endif
    Com::config(PSTR("NumFans:"),(int)NUM_FANS);
    Com::config(PSTR("LCD:"), FEATURE_CONTROLLER != NO_CONTROLLER);
    Com::config(PSTR("SoftwarePowerSwitch:"), PS_ON_PIN > -1);
    Com::config(PSTR("XHomeDir:"), Motion1::homeDir[X_AXIS]);
    Com::config(PSTR("YHomeDir:"), Motion1::homeDir[Y_AXIS]);
    Com::config(PSTR("ZHomeDir:"), Motion1::homeDir[Z_AXIS]);
#if DRIVE_SYSTEM == DELTA
    Com::config(PSTR("XHomePos:"), 0, 2);
    Com::config(PSTR("YHomePos:"), 0, 2);
    Com::config(PSTR("ZHomePos:"), Motion1::maxPos[Z_AXIS], 3);
#else
    Com::config(PSTR("XHomePos:"), (Motion1::homeDir[X_AXIS] > 0 ? Motion1::maxPos[X_AXIS] : Motion1::minPos[X_AXIS]), 2);
    Com::config(PSTR("YHomePos:"), (Motion1::homeDir[Y_AXIS] > 0 ? Motion1::maxPos[Y_AXIS] : Motion1::minPos[Y_AXIS]), 2);
    Com::config(PSTR("ZHomePos:"), (Motion1::homeDir[Z_AXIS] > 0 ? Motion1::maxPos[Z_AXIS] : Motion1::minPos[Z_AXIS]), 3);
#endif
    Com::config(PSTR("SupportG10G11:"), FEATURE_RETRACTION);
    Com::config(PSTR("SupportLocalFilamentchange:"), FEATURE_RETRACTION);
    Com::config(PSTR("CaseLights:"), CASE_LIGHTS_PIN > -1);
    Com::config(PSTR("ZProbe:"), FEATURE_Z_PROBE);
    Com::config(PSTR("Autolevel:"), FEATURE_AUTOLEVEL);
    Com::config(PSTR("EEPROM:"), EEPROM_MODE != 0);
    Com::config(PSTR("PrintlineCache:"), PRINTLINE_CACHE_SIZE);
    Com::config(PSTR("JerkXY:"), Motion1::maxYank[X_AXIS]);
    Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);
#if DRIVE_SYSTEM != DELTA
    Com::config(PSTR("JerkZ:"), Motion1::maxYank[Z_AXIS]);
#endif
#if FEATURE_RETRACTION
    // TODO: Report retraction
    /* Com::config(PSTR("RetractionLength:"), EEPROM_FLOAT(RETRACTION_LENGTH));
    Com::config(PSTR("RetractionLongLength:"), EEPROM_FLOAT(RETRACTION_LONG_LENGTH));
    Com::config(PSTR("RetractionSpeed:"), EEPROM_FLOAT(RETRACTION_SPEED));
    Com::config(PSTR("RetractionZLift:"), EEPROM_FLOAT(RETRACTION_Z_LIFT));
    Com::config(PSTR("RetractionUndoExtraLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH));
    Com::config(PSTR("RetractionUndoExtraLongLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH));
    Com::config(PSTR("RetractionUndoSpeed:"), EEPROM_FLOAT(RETRACTION_UNDO_SPEED));*/
#endif // FEATURE_RETRACTION
    Com::config(PSTR("XMin:"), Motion1::minPos[X_AXIS]);
    Com::config(PSTR("YMin:"), Motion1::minPos[Y_AXIS]);
    Com::config(PSTR("ZMin:"), Motion1::minPos[Z_AXIS]);
    Com::config(PSTR("XMax:"), Motion1::maxPos[X_AXIS]);
    Com::config(PSTR("YMax:"), Motion1::maxPos[Y_AXIS]);
    Com::config(PSTR("ZMax:"), Motion1::maxPos[Z_AXIS]);
    Com::config(PSTR("XSize:"), Motion1::maxPos[X_AXIS] - Motion1::minPos[X_AXIS]);
    Com::config(PSTR("YSize:"), Motion1::maxPos[Y_AXIS] - Motion1::minPos[Y_AXIS]);
    Com::config(PSTR("ZSize:"), Motion1::maxPos[Z_AXIS] - Motion1::minPos[Z_AXIS]);
    Com::config(PSTR("XPrintAccel:"), Motion1::maxAcceleration[X_AXIS]);
    Com::config(PSTR("YPrintAccel:"), Motion1::maxAcceleration[Y_AXIS]);
    Com::config(PSTR("ZPrintAccel:"), Motion1::maxAcceleration[Z_AXIS]);
    Com::config(PSTR("XTravelAccel:"), Motion1::maxAcceleration[X_AXIS]);
    Com::config(PSTR("YTravelAccel:"), Motion1::maxAcceleration[Y_AXIS]);
    Com::config(PSTR("ZTravelAccel:"), Motion1::maxAcceleration[Z_AXIS]);
#if PRINTER_TYPE == 2
    Com::config(PSTR("PrinterType:Delta"));
#else
    Com::config(PSTR("PrinterType:Cartesian"));
#endif // PRINTER_TYPE
    if (NUM_HEATED_BEDS > 0) {
        Com::config(PSTR("MaxBedTemp:"), heatedBeds[0]->getMaxTemperature());
    }
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        Tool *t = Tool::getTool(i);
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Jerk:"), t->getMaxYank());
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxSpeed:"), t->getMaxSpeed());
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Acceleration:"), t->getAcceleration());
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("Diameter:"), t->getDiameter());
        START_EXTRUDER_CONFIG(i)
        Com::printFLN(PSTR("MaxTemp:"), t->getMaxTemp());
    }
}

#if JSON_OUTPUT
void Printer::showJSONStatus(int type) {
    bool firstOccurrence;

    Com::printF(PSTR("{\"status\": \""));
    if (Motion1::length == 0) {
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
#if NUM_HEATED_BEDS > 0
    Com::print(heatedBeds[0]->getCurrentTemperature());
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_TOOLS; i++) {
        Com::print(',');
        if (Tool::getTool(i)->getHeater() != nullptr) {
            Com::print(Tool::getTool(i)->getHeater()->getCurrentTemperature());
        } else {
            Com::print('0');
        }
    }
    //  "active": [65.0, 195.0, 0.0],
    Com::printF(PSTR("],\"active\":["));
#if NUM_HEATED_BEDS > 0
    Com::print(heatedBeds[0]->getTargetTemperature());
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_TOOLS; i++) {
        Com::print(',');
        if (Tool::getTool(i)->getHeater() != nullptr) {
            Com::print(Tool::getTool(i)->getHeater()->getTargetTemperature());
        } else {
            Com::print('0');
        }
    }
    //  "standby": [-273.1, 0.0, 150.0],
    Com::printF(PSTR("],\"standby\":["));
#if NUM_HEATED_BEDS > 0
    Com::print(heatedBeds[0]->getTargetTemperature());
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_TOOLS; i++) {
        Com::print(',');
        if (Tool::getTool(i)->getHeater() != nullptr) {
            Com::print(Tool::getTool(i)->getHeater()->getTargetTemperature());
        } else {
            Com::print('0');
        }
    }
    //  "hstat": [0, 0, 0],
    //  hstat is 0 for heater off, 1 for standby, 2 for active and 3 for fault. We have just added 4 for "being auto-tuned'
    Com::printF(PSTR("],\"hstat\":["));

#if NUM_HEATED_BEDS > 0
    if (heatedBeds[0]->getError() != HeaterError::NO_ERROR) {
        Com::print('3');
    } else {
        Com::print(heatedBeds[0]->getTargetTemperature() < MAX_ROOM_TEMPERATURE ? 0 : 2);
    }
#else
    Com::print((int)0);
#endif
    for (int i = 0; i < NUM_TOOLS; i++) {
        Com::print(',');
        if (Tool::getTool(i)->getHeater() != nullptr) {
            if (Tool::getTool(i)->getHeater()->getError() != HeaterError::NO_ERROR) {
                Com::print('3');
            } else {
                Com::print(Tool::getTool(i)->getHeater()->getTargetTemperature() < MAX_ROOM_TEMPERATURE ? 0 : 2);
            }
            Com::print(Tool::getTool(i)->getHeater()->getTargetTemperature());
        } else {
            Com::print('0');
        }
    }
    //  "pos": [1.00, 205.00, 6.48],
    Com::printF(PSTR("],\"pos\":["));
    Com::print(Motion1::currentPosition[X_AXIS]); // X
    Com::print(',');
    Com::print(Motion1::currentPosition[Y_AXIS]); // Y
    Com::print(',');
    Com::print(Motion1::currentPosition[Z_AXIS]); // Z
    //  "extr": [0.0, 0.0],
    Com::printF(PSTR("],\"extr\":["));
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (i)
            Com::print(',');
        if (Tool::getTool(i)->getHeater() != nullptr) {
            Com::print(Tool::getTool(i)->getHeater()->getCurrentTemperature());
        } else {
            Com::print('0');
        }
    }
    //  "sfactor": 100.00,
    Com::printF(PSTR("],\"sfactor\":"), Printer::feedrateMultiply);
    //  "efactor": [100.00, 100.00],
    Com::printF(PSTR(",\"efactor\":["));
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (i)
            Com::print(',');
        Com::print((int)Printer::extrudeMultiply);
    }
    //  "tool": 0,
    Com::printF(PSTR("],\"tool\":"), Tool::getActiveToolId());
    //"probe": "4",
    Com::printF(PSTR(",\"probe\":"));
    if (ZProbe->triggered())
        Com::print((int)0);
    else
        Com::print((int)1000);
    //  "fanPercent": [0.00, 100.00],
    Com::printF(PSTR(",\"fanPercent\":["));
    for (int i = 0; i < NUM_FANS; i++) {
        if (i > 0) {
            Com::printF(Com::tComma);
        }
        Com::print(getFanSpeed(i) / 2.55f);
    }
    Com::printF(PSTR("]"));
    //  "fanRPM": 0,
    //  "homed": [1, 1, 1],
    Com::printF(PSTR(",\"homed\":["));
    Com::print((int)Motion1::isAxisHomed(X_AXIS));
    Com::print(',');
    Com::print(Motion1::isAxisHomed(Y_AXIS));
    Com::print(',');
    Com::print(Motion1::isAxisHomed(Z_AXIS));
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
    Com::print((int)Motion1::isAxisHomed(X_AXIS));
    Com::print(',');
    Com::print(Motion1::isAxisHomed(Y_AXIS));
    Com::print(',');
    Com::print(Motion1::isAxisHomed(Z_AXIS));
    Com::printF(PSTR("],\"extr\":["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print(Motion1::currentPosition[Z_AXIS]);
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"xyz\":["));
    Com::print(Motion1::currentPosition[X_AXIS]); // X
    Com::print(',');
    Com::print(Motion1::currentPosition[Y_AXIS]); // Y
    Com::print(',');
    Com::print(Motion1::currentPosition[Z_AXIS]); // Z
    Com::printF(PSTR("]},\"currentTool\":"));
    Com::print(Tool::getActiveToolId());
    Com::printF(PSTR(",\"params\": {\"atxPower\":"));
    Com::print(isPowerOn() ? '1' : '0');
    Com::printF(PSTR(",\"fanPercent\":["));
    for (int i = 0; i < NUM_FANS; i++) {
        if (i > 0) {
            Com::printF(Com::tComma);
        }
        Com::print(getFanSpeed(i) / 2.55f);
    }
    Com::printF(PSTR("],\"speedFactor\":"));
    Com::print(Printer::feedrateMultiply);
    Com::printF(PSTR(",\"extrFactors\":["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Com::print((int)Printer::extrudeMultiply); // Really *100? 100 is normal
        firstOccurrence = false;
    }
    Com::printF(PSTR("]},"));
    // SEQ??
    Com::printF(PSTR("\"temps\": {"));
    for(int i = 0; i < NUM_HEATERS; i++) {
        HeatManager *h = heaters[i];
        if (!h->isBedHeater()) {
            continue;
        }
        Com::printF(PSTR("\"bed\": {\"current\":"));
        Com::print(h->getCurrentTemperature());
        Com::printF(PSTR(",\"active\":"));
        Com::print(h->getTargetTemperature());
        Com::printF(PSTR(",\"state\":"));
        Com::print(h->getTargetTemperature() > 0 ? '2' : '1');
        Com::printF(PSTR("},"));
        break;
    }
    Com::printF(PSTR("\"heads\": {\"current\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Tool *t = Tool::getTool(i);
        HeatManager *h = t->getHeater();
        if (h == nullptr) {
            Com::print(0);
        } else { 
            Com::print(h->getCurrentTemperature());
        }
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"active\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Tool *t = Tool::getTool(i);
        HeatManager *h = t->getHeater();
        if (h == nullptr) {
            Com::print(0);
        } else { 
            Com::print(h->getTargetTemperature());
        }
        firstOccurrence = false;
    }
    Com::printF(PSTR("],\"state\": ["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence)
            Com::print(',');
        Tool *t = Tool::getTool(i);
        HeatManager *h = t->getHeater();
        if (h == nullptr) {
            Com::print(0);
        } else { // Don't have that information, so fake it to make some sense
            Com::print(h->getTargetTemperature() > 50 ? '2' : '1');            
        //Com::print(extruder[i].tempControl.targetTemperatureC > EXTRUDER_FAN_COOL_TEMP ? '2' : '1');
        }
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
            Com::print((int)(Motion1::currentPosition[Z_AXIS] / sd.fileInfo.layerHeight));
        } else
            Com::print('0');
#else
        Com::printF(PSTR("-1"));
#endif
        Com::printF(PSTR(",\"extrRaw\":["));
        firstOccurrence = true;
        for (int i = 0; i < NUM_TOOLS; i++) {
            if (!firstOccurrence)
                Com::print(',');
            Com::print(Motion1::currentPosition[E_AXIS]);
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
        Com::print((int)Motion1::minPos[X_AXIS]);
        Com::print(',');
        Com::print((int)Motion1::minPos[Y_AXIS]);
        Com::print(',');
        Com::print((int)Motion1::minPos[Z_AXIS]);
        Com::printF(PSTR("],\"axisMaxes\":["));
        Com::print((int)Motion1::maxPos[X_AXIS]);
        Com::print(',');
        Com::print((int)Motion1::maxPos[Y_AXIS]);
        Com::print(',');
        Com::print((int)Motion1::maxPos[Z_AXIS]);
        Com::printF(PSTR("],\"accelerations\":["));
        Com::print(Motion1::maxAcceleration[X_AXIS]);
        Com::print(',');
        Com::print(Motion1::maxAcceleration[Y_AXIS]);
        Com::print(',');
        Com::print(Motion1::maxAcceleration[Z_AXIS]);
        for (int i = 0; i < NUM_TOOLS; i++) {
            Com::print(',');
            Com::print(Tool::getTool(i)->getAcceleration());
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
        for (int i = 0; i < NUM_TOOLS; i++) {
            Com::printF(PSTR(",0"));
        }
        Com::printF(PSTR("],\"maxFeedrates\":["));
        Com::print(Motion1::maxFeedrate[X_AXIS]);
        Com::print(',');
        Com::print(Motion1::maxFeedrate[Y_AXIS]);
        Com::print(',');
        Com::print(Motion1::maxFeedrate[Z_AXIS]);
        for (int i = 0; i < NUM_TOOLS; i++) {
            Com::print(',');
            Com::print(Tool::getTool(i)->getMaxSpeed());
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
    flashSource.close(); // stop flash printing if busy
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

/** \brief copies currentPosition to parameter. */
void Printer::realPosition(float& xp, float& yp, float& zp) {
    xp = Motion1::currentPosition[X_AXIS];
    yp = Motion1::currentPosition[Y_AXIS];
    zp = Motion1::currentPosition[Z_AXIS];
}
