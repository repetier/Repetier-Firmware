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

#if X_STEP_PIN < 0 || Y_STEP_PIN < 0 || Z_STEP_PIN < 0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if PRINTLINE_CACHE_SIZE < 4
#error PRINTLINE_CACHE_SIZE must be at least 5
#endif

//Inactivity shutdown variables
millis_t previousMillisCmd = 0;
millis_t maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
millis_t stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
int32_t baudrate = BAUDRATE; ///< Communication speed rate.
volatile int waitRelax = 0;  // Delay filament relax at the end of print, could be a simple timeout

ServoInterface* servos[] = SERVO_LIST;
constexpr int numServos = std::extent<decltype(servos)>::value;
static_assert(numServos == NUM_SERVOS, "NUM_SERVOS not defined correctly");

FanController fans[NUM_FANS];

BeeperSourceBase* beepers[] = BEEPER_LIST;
constexpr int numBeepers = std::extent<decltype(beepers)>::value;
static_assert(numBeepers == NUM_BEEPERS, "NUM_BEEPERS not defined correctly");

uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
uint8_t Printer::relativeCoordinateMode = false;         ///< Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false; ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

PromptDialogCallback Printer::activePromptDialog = nullptr; ///< Dialog ID that is active
bool Printer::promptSupported = false;                      ///< At least one connecte dhost supports host prompts
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;
uint8_t Printer::reportFlag = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
uint16_t Printer::menuMode = 0;
float Printer::extrudeMultiplyError = 0;
float Printer::extrusionFactor = 1.0;
uint8_t Printer::interruptEvent = 0;
fast8_t Printer::breakLongCommand = false;
int Printer::currentLayer = 0;
int Printer::maxLayer = -1;       // -1 = unknown
char Printer::printName[21] = ""; // max. 20 chars + 0
float Printer::progress = 0;

millis_t Printer::lastTempReport = 0;
millis_t Printer::autoTempReportPeriodMS = 1000;
millis_t Printer::lastSDReport = 0;
millis_t Printer::autoSDReportPeriodMS = 0;

int32_t Printer::printingTime = 0;

uint32_t Printer::interval = 30000;      ///< Last step duration in ticks.
uint32_t Printer::timer;                 ///< used for acceleration/deceleration timing
uint32_t Printer::stepNumber;            ///< Step number in current move.
float Printer::feedrate;                 ///< Last requested feedrate.
int Printer::feedrateMultiply;           ///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int Printer::extrudeMultiply;   ///< Flow multiplier in percent (factor 1 = 100)
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
#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif
fast8_t Printer::wizardStackPos;
wizardVar Printer::wizardStack[WIZARD_STACK_SIZE];
uint16_t Printer::rescuePos;       // EEPROM address for rescue
fast8_t Printer::safetyParked = 0; /// True if moved to a safety position to protect print
uint8_t Printer::rescueOn = false;
bool Printer::failedMode = false;
#ifndef CASE_LIGHT_DEFAULT_ON
#define CASE_LIGHT_DEFAULT_ON 0
#endif
fast8_t Printer::caseLightMode = CASE_LIGHT_DEFAULT_ON;
fast8_t Printer::caseLightBrightness = 255;

ufast8_t Printer::toneVolume = DEFAULT_TONE_VOLUME;

FirmwareEvent FirmwareEvent::eventList[4];
volatile fast8_t FirmwareEvent::start = 0;
volatile fast8_t FirmwareEvent::length = 0;

// Add event to queue if possible
bool FirmwareEvent::queueEvent(int id, wizardVar p1, wizardVar p2) {
    InterruptProtectedBlock lock;
    if (length == 4) {
        return false;
    }
    FirmwareEvent& act = eventList[(start + length) % 4];
    act.eventId = id;
    act.param1 = p1;
    act.param2 = p2;
    length++;
    return true;
}

// Resolve all event tasks
void FirmwareEvent::handleEvents() {
    while (length > 0) {
        FirmwareEvent& act = eventList[start];
#undef IO_TARGET
#define IO_TARGET IO_TARGET_FIRMWARE_EVENTS
#include "io/redefine.h"
        {
            InterruptProtectedBlock lock;
            start++;
            if (start == 4) {
                start = 0;
            }
            length--;
        }
    }
}

void Printer::setDebugLevel(uint8_t newLevel) {
    if (newLevel != debugLevel) {
        debugLevel = newLevel;
        if (debugDryrun()) {
            // Disable all heaters in case they were on
            HeatManager::disableAllHeaters();
        }
        Com::printFLN(PSTR("DebugLevel:"), (int)newLevel);
    }
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
    return (int)fans[fanId].fan->get();
}

void Printer::setFanSpeed(int speed, bool immediately, int fanId, uint32_t timeoutMS) {
    if (fanId < 0 || fanId >= NUM_FANS) {
        return;
    }
    speed = TRIM_FAN_PWM(constrain(speed, 0, 255));
    Tool* tool = nullptr;
    Tool* activeTool = Tool::getActiveTool();
    if (timeoutMS) {
        fans[fanId].timeout = timeoutMS;
        fans[fanId].target = speed;
        fans[fanId].time = HAL::timeInMilliseconds();
        return;
    }

    if (fans[fanId].timeout) { // remove existing timeout
        fans[fanId].timeout = 0;
        fans[fanId].target = 0;
        fans[fanId].time = 0;
    }
    Com::printF(PSTR("Fanspeed"), fanId);
    Com::printFLN(Com::tColon, speed);
    if (activeTool != nullptr && activeTool->usesSecondary(fans[fanId].fan)) {
        tool = activeTool;
        tool->setSecondaryFixed(speed);
        if (!immediately && Motion1::buffersUsed()) {
            return;
        }
    }
    if (Printer::getFanSpeed(fanId) == speed) {
        return; // nothing to do
    }
    if (tool && immediately) {
        Printer::setMenuMode(MENU_MODE_FAN_RUNNING, speed != 0);
        if (Motion1::length == 0 || immediately) {
            if (Tool::getActiveTool() && Tool::getActiveTool()->secondaryIsFan()) {
                for (fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++) {
                    Motion1::buffers[i].secondSpeed = speed; // fill all printline buffers with new fan speed value
                }
            }
        }
    }
    fans[fanId].fan->set(speed);
}

void Printer::checkFanTimeouts() {
    for (fast8_t i = 0; i < NUM_FANS; i++) {
        if (fans[i].timeout) {
            if ((HAL::timeInMilliseconds() - fans[i].time) > fans[i].timeout) {
                EVENT_FAN_TIMEOUT(i, fans[i].target);
                Printer::setFanSpeed(fans[i].target, true, i);
            }
        }
    }
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

void Printer::updateDerivedParameter() {
    /*#if DRIVE_SYSTEM == DELTA
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
*/
    EVENT_UPDATE_DERIVED;
}
#if AUTOMATIC_POWERUP
void Printer::enablePowerIfNeeded() {
    if (Printer::isPowerOn()) {
        return;
    }
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
    if (areAllSteppersDisabled() && onlySteppers) {
        return;
    }
    if (Printer::isAllKilled()) {
        return;
    }
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    disableAllMotorDrivers();
#endif // defined
    XMotor.disable();
    YMotor.disable();
#if defined(PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT) && PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT == 0
    ZMotor.disable();
#else
    if (!onlySteppers) {
        ZMotor.disable();
    }
#endif
    for (fast8_t i = A_AXIS; i < NUM_AXES; i++) {
        Motion1::motors[i]->disable();
    }
    Tool::disableMotors();
    setAllSteppersDisabled();

    FOR_ALL_AXES(i) {
#if defined(PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT) && PREVENT_Z_DISABLE_ON_STEPPER_TIMEOUT == 1
        if (i == Z_AXIS) {
            continue;
        }
#endif
        Motion1::setAxisHomed(i, false);
    }
    unsetHomedAll();
    if (!onlySteppers) {
        for (uint8_t i = 0; i < NUM_TOOLS; i++) {
            Tool::getTool(i)->shutdown();
        }
        for (uint8_t i = 0; i < NUM_HEATED_BEDS; i++) {
            heatedBeds[i]->setTargetTemperature(0);
        }
        UI_STATUS_UPD("Standby");
#if defined(PS_ON_PIN) && PS_ON_PIN > -1 && !defined(NO_POWER_TIMEOUT)
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
        Printer::setPowerOn(false);
#endif
        Printer::setAllKilled(true);
    } else {
        UI_STATUS_UPD("Motors disabled");
    }
    Commands::printTemperatures();
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
uint8_t Printer::moveTo(float x, float y, float z, float e, float f) {
    if (f != IGNORE_COORDINATE)
        feedrate = f;
    Motion1::setTmpPositionXYZE(x, y, z, e);
    Motion1::moveByPrinter(Motion1::tmpPosition, feedrate, false);
    return 1;
}

uint8_t Printer::moveToReal(float x, float y, float z, float e, float f, bool pathOptimize) {
    if (f != IGNORE_COORDINATE)
        feedrate = f;
    Motion1::setTmpPositionXYZE(x, y, z, e);
    Motion1::moveByOfficial(Motion1::tmpPosition, feedrate, false);
    return 1;
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
    Motion1::g92Offsets[X_AXIS] = xOff; // offset from G92
    Motion1::g92Offsets[Y_AXIS] = yOff;
    Motion1::g92Offsets[Z_AXIS] = zOff;
}

void Printer::setPrinting(uint8_t b) {
    if (!!isPrinting != !!b) { // Start with progress on print start
#if FEATURE_CONTROLLER != NO_CONTROLLER
        if (b) {
            GUI::callbacks[0] = printProgress;
        } else {
            GUI::callbacks[0] = startScreen;
        }
        GUI::resetMenu();
#endif
    }
    flag3 = (b ? flag3 | PRINTER_FLAG3_PRINTING : flag3 & ~PRINTER_FLAG3_PRINTING);
    Printer::setMenuMode(MENU_MODE_PRINTING, b);
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
    bool secondaryMove = false;
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
            if (fabsf(com->E) * extrusionFactor > EXTRUDE_MAXLENGTH) {
                Com::printWarningF(PSTR("Max. extrusion distance per move exceeded - ignoring move."));
                p = 0.0f;
            }
            coords[E_AXIS] = Motion1::currentPosition[E_AXIS] + p;
        } else {
            if (fabsf(p - Motion1::currentPosition[E_AXIS]) * extrusionFactor > EXTRUDE_MAXLENGTH) {
                p = Motion1::currentPosition[E_AXIS];
            }
            coords[E_AXIS] = p;
        }
#if FEATURE_RETRACTION
        if (com->hasNoXYZ() && isAutoretract()) { // Lone E moves.
            if (relativeCoordinateMode || relativeExtruderCoordinateMode) {
                Tool::getActiveTool()->retract(com->E < 0.0f, false);
            } else {
                Tool::getActiveTool()->retract(p < Motion1::currentPosition[E_AXIS], false);
            }
            return;
        }
#endif
        secondaryMove = Tool::getActiveTool()->isSecondaryMove(com->hasG() && com->G == 0, true);
    } else {
        coords[E_AXIS] = Motion1::currentPosition[E_AXIS];
        secondaryMove = Tool::getActiveTool()->isSecondaryMove(com->hasG() && com->G == 0, false);
    }
    if (com->hasF() && com->F > 0.1) {
        if (unitIsInches) {
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply; // Factor is 25.4/60/100
        } else {
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f; // Factor is 1/60/100
        }
    }
    Motion1::moveByOfficial(coords, feedrate, secondaryMove);
}

void Printer::setup() {
    HAL::stopWatchdog();

    // Main board specific extra initialization
#if defined(MB_SETUP)
    MB_SETUP;
#endif
    // HAL::serialSetBaudrate(115200);
    // Start serial
    HAL::hwSetup();

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

    Printer::setNativeUSB(!GCodeSource::hasBaudSources());

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
#ifndef NO_SPI
    HAL::spiInit();
#endif
    // Define io functions
#undef IO_TARGET
#define IO_TARGET IO_TARGET_INIT
#include "io/redefine.h"

    PWMHandler* tempFans[] = FAN_LIST;
    constexpr int numFans = std::extent<decltype(tempFans)>::value;
    static_assert(numFans == NUM_FANS, "NUM_FANS not defined correctly");

    for (fast8_t i = 0; i < NUM_FANS; i++) {
        fans[i].fan = tempFans[i];
    }
    HAL::analogStart();

    Motion1::init();
    Motion2::init();
    Motion3::init();
    ZProbeHandler::init();
    LevelingCorrector::init();
    Leveling::init();
    PrinterType::init();
    Tool::initTools();
    rescuePos = EEPROM::reserveRecover(1, 1, EPR_RESCUE_SIZE);

#undef IO_TARGET
#define IO_TARGET IO_TARGET_INIT_LATE
#include "io/redefine.h"

#if FEATURE_CONTROLLER == CONTROLLER_VIKI
    HAL::delayMilliseconds(100);
#endif // FEATURE_CONTROLLER
    //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization

    EVENT_INITIALIZE_EARLY

#if defined(DOOR_PIN) && DOOR_PIN > -1
    SET_INPUT(DOOR_PIN);
#if defined(DOOR_PULLUP) && DOOR_PULLUP
    PULLUP(DOOR_PIN, HIGH);
#endif
#endif

#if defined(UI_VOLTAGE_LEVEL) && defined(EXP_VOLTAGE_LEVEL_PIN) && EXP_VOLTAGE_LEVEL_PIN > -1
    SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
    WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
#endif // UI_VOLTAGE_LEVEL
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    LaserDriver::initialize();
#endif // defined

#ifdef RED_BLUE_STATUS_LEDS
    SET_OUTPUT(RED_STATUS_LED);
    SET_OUTPUT(BLUE_STATUS_LED);
    WRITE(BLUE_STATUS_LED, HIGH);
    WRITE(RED_STATUS_LED, LOW);
#endif // RED_BLUE_STATUS_LEDS
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    initializeAllMotorDrivers();
#endif // defined
    // microstepInit();
    feedrate = 50; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
    extrudeMultiply = 100;
    interval = 5000;
    msecondsPrinting = 0;
    filamentPrinted = 0;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
#if ENABLE_BACKLASH_COMPENSATION
    backlashX = X_BACKLASH;
    backlashY = Y_BACKLASH;
    backlashZ = Z_BACKLASH;
    backlashDir = 0;
#endif
#if (MOTHERBOARD == 502)
    SET_INPUT(FTDI_COM_RESET_PIN);
    SET_INPUT(ESP_WIFI_MODULE_COM);
    SET_INPUT(MOTOR_FAULT_PIN);
    SET_INPUT(MOTOR_FAULT_PIGGY_PIN);
#endif              //(MOTHERBOARD == 501) || (MOTHERBOARD == 502)
    EEPROM::init(); // Read settings from eeprom if wanted, run after initialization!
    // Extruder::initExtruder();
    // sets auto leveling in eeprom init
    GUI::init();

#if SDSUPPORT // Try mounting the SDCard first in case it has an eeprom file.
    sd.mount(true);
#endif

    EEPROM::init(); // Read settings from eeprom if wanted, run after initialization!
    updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();


#if FEATURE_WATCHDOG
    HAL::startWatchdog();
#endif

    Tool::selectTool(rescueStartTool());
    // Extruder::selectExtruderById(0);
    HAL::delayMilliseconds(20);

    EVENT_INITIALIZE;
#ifdef STARTUP_GCODE
    GCode::executeFString(Com::tStartupGCode);
#endif
    rescueSetup();
    playDefaultSound(DefaultSounds::RESET);
}

void Printer::defaultLoopActions() {
    Commands::checkForPeriodicalActions(true); //check heater every n milliseconds
    if (HAL::i2cError && HAL::i2cError != 255) {
        HAL::i2cError = 255; // Flag to show error message only once
        GCode::fatalError(Com::tI2CError);
    }
    millis_t curtime = HAL::timeInMilliseconds();
    if (isRescueRequired() || Motion1::length != 0 || isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED)) {
        previousMillisCmd = curtime;
    } else {
        curtime -= previousMillisCmd;
        if (maxInactiveTime != 0 && curtime > maxInactiveTime) {
            Printer::kill(false);
        } else {
            Printer::setAllKilled(false); // prevent repeated kills
        }
        if (stepperInactiveTime != 0 && curtime > stepperInactiveTime) {
            Printer::kill(true);
        }
    }
#if SDCARDDETECT > -1 && SDSUPPORT
    if(Printer::isAutomount()) {
        sd.automount();
    }
#endif
#if defined(EEPROM_AVAILABLE) && (EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH)
    HAL::syncEEPROM();
#endif
    DEBUG_MEMORY;
}

void Printer::reportCaseLightStatus() {
    if (Printer::caseLightMode) {
        Com::printInfoFLN(PSTR("Case lights on"));
    } else {
        Com::printInfoFLN(PSTR("Case lights off"));
    }
    Com::printInfoF(PSTR("Case light mode:"));
    Com::print((int32_t)caseLightMode);
    Com::println();
    Com::printInfoF(PSTR("Case light brightness:"));
    Com::print((int32_t)caseLightBrightness);
    Com::println();
}

void Printer::handleInterruptEvent() {
    if (interruptEvent == 0)
        return;
    int event = interruptEvent;
    interruptEvent = 0;
    switch (event) {
#if EXTRUDER_JAM_CONTROL
    case PRINTER_INTERRUPT_EVENT_JAM_DETECTED:
        if (isJamcontrolDisabled()) {
            break;
        }
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
        if (sd.state == SDState::SD_PRINTING) {
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
    Com::config(PSTR("NumExtruder:"), NUM_TOOLS);
    // Com::config(PSTR("MixingExtruder:"), MIXING_EXTRUDER);
    Com::config(PSTR("HeatedBed:"), NUM_HEATED_BEDS);
    Com::config(PSTR("HeatedChamber:"), NUM_HEATED_CHAMBERS);
    Com::config(PSTR("SDCard:"), SDSUPPORT);
    Com::config(PSTR("Fan:"), NUM_FANS > 0);
#if NUM_FANS > 1
    Com::config(PSTR("Fan2:1"));
#else
    Com::config(PSTR("Fan2:0"));
#endif
    Com::config(PSTR("NumFans:"), (int)NUM_FANS);
    Com::config(PSTR("LCD:"), FEATURE_CONTROLLER != NO_CONTROLLER);
    Com::config(PSTR("SoftwarePowerSwitch:"), PS_ON_PIN > -1);
    Com::config(PSTR("XHomeDir:"), Motion1::homeDir[X_AXIS]);
    Com::config(PSTR("YHomeDir:"), Motion1::homeDir[Y_AXIS]);
    Com::config(PSTR("ZHomeDir:"), Motion1::homeDir[Z_AXIS]);
#if PRINTER_TYPE == PRINTER_TYPE_DELTA
#if FIXED_Z_HOME_POSITION
    Com::config(PSTR("XHomePos:"), (float)ZHOME_X_POS, 2);
    Com::config(PSTR("YHomePos:"), (float)ZHOME_Y_POS, 2);
#else
    Com::config(PSTR("XHomePos:"), 0, 2);
    Com::config(PSTR("YHomePos:"), 0, 2);
#endif
    Com::config(PSTR("ZHomePos:"), Motion1::maxPos[Z_AXIS], 3);
#else
    Com::config(PSTR("XHomePos:"), (Motion1::homeDir[X_AXIS] > 0 ? Motion1::maxPos[X_AXIS] : Motion1::minPos[X_AXIS]), 2);
    Com::config(PSTR("YHomePos:"), (Motion1::homeDir[Y_AXIS] > 0 ? Motion1::maxPos[Y_AXIS] : Motion1::minPos[Y_AXIS]), 2);
#if ZHOME_HEIGHT > 0
    Com::config(PSTR("ZHomePos:"), (float)ZHOME_HEIGHT);
#else
    Com::config(PSTR("ZHomePos:"), (Motion1::homeDir[Z_AXIS] > 0 ? Motion1::maxPos[Z_AXIS] : Motion1::minPos[Z_AXIS]), 3);
#endif
#endif
    Com::config(PSTR("SupportG10G11:"), FEATURE_RETRACTION);
    Com::config(PSTR("SupportLocalFilamentchange:"), FEATURE_RETRACTION);
    Com::config(PSTR("CaseLights:"), 1);
    Com::config(PSTR("ZProbe:"), Z_PROBE_TYPE > 0 ? 1 : 0);
    Com::config(PSTR("Autolevel:"), LEVELING_METHOD > 0 ? 1 : 0);
    Com::config(PSTR("EEPROM:"), EEPROM_MODE != 0);
    Com::config(PSTR("PrintlineCache:"), PRINTLINE_CACHE_SIZE);
    Com::config(PSTR("JerkXY:"), Motion1::maxYank[X_AXIS]);
    Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);
#if PRINTER_TYPE != PRINTER_TYPE_DELTA
    Com::config(PSTR("JerkZ:"), Motion1::maxYank[Z_AXIS]);
#endif
#if FEATURE_RETRACTION
    Com::config(PSTR("RetractionLength:"), Motion1::retractLength);
    Com::config(PSTR("RetractionLongLength:"), Motion1::retractLongLength);
    Com::config(PSTR("RetractionSpeed:"), Motion1::retractSpeed);
    Com::config(PSTR("RetractionZLift:"), Motion1::retractZLift);
    Com::config(PSTR("RetractionUndoExtraLength:"), Motion1::retractUndoExtraLength);
    Com::config(PSTR("RetractionUndoExtraLongLength:"), Motion1::retractUndoExtraLongLength);
    Com::config(PSTR("RetractionUndoSpeed:"), Motion1::retractUndoSpeed);
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
    Com::config(PSTR("XPrintAccel:"), Motion1::maxAccelerationEEPROM[X_AXIS]);
    Com::config(PSTR("YPrintAccel:"), Motion1::maxAccelerationEEPROM[Y_AXIS]);
    Com::config(PSTR("ZPrintAccel:"), Motion1::maxAccelerationEEPROM[Z_AXIS]);
    Com::config(PSTR("XTravelAccel:"), Motion1::maxTravelAccelerationEEPROM[X_AXIS]);
    Com::config(PSTR("YTravelAccel:"), Motion1::maxTravelAccelerationEEPROM[Y_AXIS]);
    Com::config(PSTR("ZTravelAccel:"), Motion1::maxTravelAccelerationEEPROM[Z_AXIS]);
    PrinterType::M360();
    if (NUM_HEATED_BEDS > 0) {
        Com::config(PSTR("MaxBedTemp:"), heatedBeds[0]->getMaxTemperature());
    }
    if (NUM_HEATED_CHAMBERS > 0) {
        Com::config(PSTR("MaxChamberTemp:"), heatedChambers[0]->getMaxTemperature());
    }
    for (fast8_t i = 0; i < NUM_TOOLS; i++) {
        Tool* t = Tool::getTool(i);
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
    } else if (sd.state == SDState::SD_PRINTING) {
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
        if (i) {
            Com::print(',');
        }
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
        if (i) {
            Com::print(',');
        }
        Com::print((int)Printer::extrudeMultiply);
    }
    //  "tool": 0,
    Com::printF(PSTR("],\"tool\":"), Tool::getActiveToolId());
    //"probe": "4",
    Com::printF(PSTR(",\"probe\":"));
    if (ZProbe && ZProbe->triggered()) {
        Com::print((int)0);
    } else {
        Com::print((int)1000);
    }
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
    Com::print((int)Motion1::isAxisHomed(Y_AXIS));
    Com::print(',');
    Com::print((int)Motion1::isAxisHomed(Z_AXIS));
    Com::printF(PSTR("]"));
    if (type == 1) {
        Com::printF(PSTR(",\"geometry\":\""));
        Com::printF(PrinterType::getGeometryName());
        Com::printF(PSTR("\",\"myName\":\"" UI_PRINTER_NAME "\""));
        Com::printF(PSTR(",\"firmwareName\":\"Repetier\""));
    }
    Com::printF(PSTR(",\"coords\": {"));
    Com::printF(PSTR("\"axesHomed\":["));
    Com::print((int)Motion1::isAxisHomed(X_AXIS));
    Com::print(',');
    Com::print((int)Motion1::isAxisHomed(Y_AXIS));
    Com::print(',');
    Com::print((int)Motion1::isAxisHomed(Z_AXIS));
    Com::printF(PSTR("],\"extr\":["));
    firstOccurrence = true;
    for (int i = 0; i < NUM_TOOLS; i++) {
        if (!firstOccurrence) {
            Com::print(',');
        }
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
    for (int i = 0; i < NUM_HEATERS; i++) {
        HeatManager* h = heaters[i];
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
        Tool* t = Tool::getTool(i);
        HeatManager* h = t->getHeater();
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
        Tool* t = Tool::getTool(i);
        HeatManager* h = t->getHeater();
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
        Tool* t = Tool::getTool(i);
        HeatManager* h = t->getHeater();
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
        Com::printF(PrinterType::getGeometryName());
        Com::printF(PSTR("\",\"name\":\""));
        Com::printF(Com::tPrinterName);
        Com::printF(PSTR("\",\"tools\":["));
        firstOccurrence = true;
        for (int i = 0; i < NUM_TOOLS; i++) {
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
        if (sd.state == SDState::SD_PRINTING && sd.fileInfo.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
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
        if (sd.state == SDState::SD_PRINTING) {
            Com::printF(PSTR("\"fractionPrinted\":"));
            float fraction;
            if (sd.selectedFileSize < 2000000)
                fraction = sd.selectedFilePos / sd.selectedFileSize;
            else
                fraction = (sd.selectedFilePos >> 8) / (sd.selectedFileSize >> 8);
            Com::print((float)floor(fraction * 1000) / 1000); // ONE DECIMAL, COULD BE DONE BY SHIFTING, BUT MEH
            Com::print(',');
        }
#endif
        Com::printF(PSTR("\"firstLayerHeight\":"));
#if SDSUPPORT
        if (sd.state == SDState::SD_PRINTING) {
            Com::print(sd.fileInfo.layerHeight);
        } else {
            Com::print('0');
        }
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
        GUI::resetMenu();
    }
}

/** \brief copies currentPosition to parameter. */
void Printer::realPosition(float& xp, float& yp, float& zp) {
    xp = Motion1::currentPosition[X_AXIS];
    yp = Motion1::currentPosition[Y_AXIS];
    zp = Motion1::currentPosition[Z_AXIS];
}

/*

Rescue system uses the following layout in eeprom:
1 : mode
4 * NUM_AXES : Last received position
4 * NUM_AXES : Last position
4 * NUM_AXES : Last Offsets
*/

void Printer::enableRescue(bool on) {
#if HOST_RESCUE
    if (on && EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE) != 0) {
        EEPROM::setRecoverByte(rescuePos + EPR_RESCUE_MODE, 0);
    }
    rescueOn = on;
    if (!on) {
        unparkSafety();
        GUI::popBusy();
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
    uint8_t mode = EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE);
    Com::printF(PSTR("RESCUE_STATE:"));
    if (mode & 1) {
        FOR_ALL_AXES(i) {
            Com::print(' ');
            Com::print('L');
            Com::printF(axisNames[i]);
            Com::printF(Com::tColon, EEPROM::getRecoverFloat(rescuePos + EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i), 2);
        }
        Com::printF(PSTR(" LT:"), (int)EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_TOOL));
    }
    if (mode & 2) {
        FOR_ALL_AXES(i) {
            Com::print(' ');
            Com::printF(axisNames[i]);
            Com::printF(Com::tColon, EEPROM::getRecoverFloat(rescuePos + EPR_RESCUE_LAST_POS + sizeof(float) * i), 2);
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
    FOR_ALL_AXES(i) {
        EEPROM::setRecoverFloat(rescuePos + EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i, Motion1::currentPosition[i]);
    }
    EEPROM::setRecoverByte(rescuePos + EPR_RESCUE_TOOL, Tool::getActiveToolId());
    EEPROM::setRecoverByte(rescuePos + EPR_RESCUE_MODE, EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE) | 1);
#endif
}

void Printer::rescueStorePosition() {
#if HOST_RESCUE
    if (!rescueOn) {
        return;
    }
    FOR_ALL_AXES(i) {
        EEPROM::setRecoverFloat(rescuePos + EPR_RESCUE_LAST_POS + sizeof(float) * i, Motion1::currentPosition[i]);
        EEPROM::setRecoverFloat(rescuePos + EPR_RESCUE_OFFSETS + sizeof(float) * i, Motion1::g92Offsets[i]);
    }
    EEPROM::setRecoverByte(rescuePos + EPR_RESCUE_MODE, EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE) | 2);
#endif
}

void Printer::rescueRecover() {
#if HOST_RESCUE
    uint8_t mode = EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE);
    if (mode == 0) {
        return;
    }
    if (mode & 2) {
        FOR_ALL_AXES(i) {
            Motion1::currentPosition[i] = EEPROM::getRecoverFloat(rescuePos + EPR_RESCUE_LAST_POS + sizeof(float) * i);
            Motion1::g92Offsets[i] = EEPROM::getRecoverFloat(rescuePos + EPR_RESCUE_OFFSETS + sizeof(float) * i);
            Motion1::setAxisHomed(i, true);
        }
    } else if (mode & 1) {
        FOR_ALL_AXES(i) {
            Motion1::currentPosition[i] = EEPROM::getRecoverFloat(rescuePos + EPR_RESCUE_LAST_RECEIVED + sizeof(float) * i);
            Motion1::setAxisHomed(i, true);
        }
    }
#endif
}

int Printer::rescueStartTool() {
#if HOST_RESCUE
    if (EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE)) {
        return static_cast<int>(EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_TOOL));
    } else {
        return 0;
    }
#else
    return 0;
#endif
}

void Printer::rescueSetup() {
#if HOST_RESCUE
    uint8_t mode = EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE);
    if (mode) {
        rescueReport();
        FOR_ALL_AXES(i) { // prevent unwanted moves so we can continue
            if (i != E_AXIS && Motion1::motors[i]) {
                Motion1::motors[i]->enable();
            }
        }
        Printer::unsetAllSteppersDisabled();
        rescueRecover();
    }
#endif
}

bool Printer::isRescueRequired() {
#if HOST_RESCUE
    return EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE) != 0;
#else
    return false;
#endif
}

void Printer::rescueReset() {
#if HOST_RESCUE
    uint8_t mode = EEPROM::getRecoverByte(rescuePos + EPR_RESCUE_MODE);
    if (mode) {
        EEPROM::setRecoverByte(rescuePos + EPR_RESCUE_MODE, 0);
    }
#endif
}

void Printer::handlePowerLoss() {
    HeatManager::disableAllHeaters();
    Com::printErrorFLN(PSTR("POWERLOSS_DETECTED"));
    if (rescueOn) {
        rescueStoreReceivedPosition();
#if POWERLOSS_LEVEL == 1 //z up only
#endif
#if POWERLOSS_LEVEL == 2 // full park - we are on a UPC :-)
        parkSafety();
#endif
        Motion1::waitForEndOfMoves();
        rescueStorePosition();
        Tool::disableMotors();
        kill(false);
        enableFailedModeP(PSTR("Power Loss"));
    }
}

void Printer::parkSafety() {
    if (safetyParked || !rescueOn) {
        return; // nothing to unpark
    }
    rescueStoreReceivedPosition();
    safetyParked = 1;
    if (Motion1::pushToMemory()) {
        Motion1::moveToParkPosition();
        Motion1::waitForEndOfMoves();
        rescueStorePosition();
        safetyParked = 2;
    }
    Tool::pauseHeaters();
    GUI::setStatusP(PSTR("Waiting f. Data..."), GUIStatusLevel::BUSY); // inform user
}
void Printer::unparkSafety() {
    if (!safetyParked) {
        return; // nothing to unpark
    }
    Tool::unpauseHeaters();
    Tool::waitForTemperatures();
    bool needsPop = safetyParked > 1;
    safetyParked = 0;
    float pos[NUM_AXES];
    if (needsPop && Motion1::popFromMemory(pos)) {
        float z = pos[Z_AXIS];
        pos[Z_AXIS] = IGNORE_COORDINATE;
        Motion1::moveByOfficial(pos, Motion1::moveFeedrate[X_AXIS], false);
        FOR_ALL_AXES(i) {
            pos[i] = IGNORE_COORDINATE;
        }
        pos[Z_AXIS] = z;
        Motion1::moveByOfficial(pos, Motion1::moveFeedrate[Z_AXIS], false);
    }
    GUI::popBusy();
}

void Printer::enableFailedModeP(PGM_P msg) {
    failedMode = true;
    GUI::setStatusP(msg, GUIStatusLevel::ERROR);
    Com::printErrorFLN(msg);
    Com::printErrorFLN(Com::tM999);
}

void Printer::enableFailedMode(char* msg) {
    failedMode = true;
    GUI::setStatus(msg, GUIStatusLevel::ERROR);
    Com::printErrorF(Com::tEmpty);
    Com::print(msg);
    Com::println();
    Com::printErrorFLN(Com::tM999);
}

void Printer::playDefaultSound(DefaultSounds sound) {
#if NUM_BEEPERS > 0
    BeeperSourceBase* beeper = beepers[0];
    switch (sound) {
    case DefaultSounds::NEXT_PREV:
        beeper->playTheme(ThemeButtonNextPrev, false);
        break;
    case DefaultSounds::OK:
        beeper->playTheme(ThemeButtonOk, false);
        break;
    case DefaultSounds::SUCCESS:
        beeper->playTheme(ThemeNotifyConfirm, false);
        break;
    case DefaultSounds::WARNING:
        beeper->playTheme(ThemeNotifyWarning, false);
        break;
    case DefaultSounds::ERROR:
        beeper->playTheme(ThemeNotifyError, false);
        break;
    case DefaultSounds::RESET:
        beeper->playTheme(ThemeButtonReset, false);
        break;
    }
#endif
}
