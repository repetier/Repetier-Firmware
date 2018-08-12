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
*/

/**

Coordinate system transformations:

Level 1: G-code => Coordinates like send via g-codes.

Level 2: Real coordinates => Coordinates corrected by coordinate shift via G92
         currentPosition and lastCmdPos are from this level.
Level 3: Transformed and shifter => Include extruder offset and bed rotation.
         These variables are only stored temporary.

Level 4: Step position => Level 3 converted into steps for motor position
        currentPositionSteps and destinationPositionSteps are from this level.

Level 5: Nonlinear motor step position, only for nonlinear drive systems
         destinationDeltaSteps


*/

#ifndef PRINTER_H_INCLUDED
#define PRINTER_H_INCLUDED


#if defined(AUTOMATIC_POWERUP) && AUTOMATIC_POWERUP && PS_ON_PIN > -1
#define ENSURE_POWER {Printer::enablePowerIfNeeded();}
#else
#undef AUTOMATIC_POWERUP
#define AUTOMATIC_POWERUP 0
#define ENSURE_POWER {}
#endif

#if defined(DRV_TMC2130)
#include "Trinamic.h"
#endif

union floatLong {
    float f;
    uint32_t l;
#ifdef SUPPORT_64_BIT_MATH
    uint64_t L;
#endif
};

union wizardVar {
    float f;
    int32_t l;
    uint32_t ul;
    int16_t i;
    uint16_t ui;
    int8_t c;
    uint8_t uc;

    wizardVar(): i(0) {}
    wizardVar(float _f): f(_f) {}
    wizardVar(int32_t _f): l(_f) {}
    wizardVar(uint32_t _f): ul(_f) {}
    wizardVar(int16_t _f): i(_f) {}
    wizardVar(uint16_t _f): ui(_f) {}
    wizardVar(int8_t _f): c(_f) {}
    wizardVar(uint8_t _f): uc(_f) {}
};

#define PRINTER_FLAG0_STEPPER_DISABLED      1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     4
#define PRINTER_FLAG0_FORCE_CHECKSUM        8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE      16
#define PRINTER_FLAG0_AUTOLEVEL_ACTIVE      32
#define PRINTER_FLAG0_ZPROBEING             64
#define PRINTER_FLAG0_LARGE_MACHINE         128
#define PRINTER_FLAG1_HOMED_ALL             1
#define PRINTER_FLAG1_AUTOMOUNT             2
#define PRINTER_FLAG1_ANIMATION             4
#define PRINTER_FLAG1_ALLKILLED             8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE      16
#define PRINTER_FLAG1_NO_DESTINATION_CHECK  32
#define PRINTER_FLAG1_POWER_ON              64
#define PRINTER_FLAG1_ALLOW_COLD_EXTRUSION  128
#define PRINTER_FLAG2_BLOCK_RECEIVING       1
#define PRINTER_FLAG2_AUTORETRACT           2
#define PRINTER_FLAG2_RESET_FILAMENT_USAGE  4
#define PRINTER_FLAG2_IGNORE_M106_COMMAND   8
#define PRINTER_FLAG2_DEBUG_JAM             16
#define PRINTER_FLAG2_JAMCONTROL_DISABLED   32
#define PRINTER_FLAG2_HOMING                64
#define PRINTER_FLAG2_ALL_E_MOTORS          128 // Set all e motors flag
#define PRINTER_FLAG3_X_HOMED               1
#define PRINTER_FLAG3_Y_HOMED               2
#define PRINTER_FLAG3_Z_HOMED               4
#define PRINTER_FLAG3_PRINTING              8 // set explicitly with M530
#define PRINTER_FLAG3_AUTOREPORT_TEMP       16
#define PRINTER_FLAG3_SUPPORTS_STARTSTOP    32
#define PRINTER_FLAG3_DOOR_OPEN             64

// List of possible interrupt events (1-255 allowed)
#define PRINTER_INTERRUPT_EVENT_JAM_DETECTED 1
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0 2
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL1 3
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL2 4
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL3 5
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL4 6
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL5 7
// define an integer number of steps more than large enough to get to end stop from anywhere
#define HOME_DISTANCE_STEPS (Printer::zMaxSteps-Printer::zMinSteps+1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])
// Some defines to make clearer reading, as we overload these Cartesian memory locations for delta
#define towerAMaxSteps Printer::xMaxSteps
#define towerBMaxSteps Printer::yMaxSteps
#define towerCMaxSteps Printer::zMaxSteps
#define towerAMinSteps Printer::xMinSteps
#define towerBMinSteps Printer::yMinSteps
#define towerCMinSteps Printer::zMinSteps

class Plane {
public:
    // f(x, y) = ax + by + c
    float a, b, c;
    float z(float x, float y) {
        return a * x + y * b + c;
    }
};

#include "Distortion.h"

#include "Endstops.h"

#ifndef DEFAULT_PRINTER_MODE
#if NUM_EXTRUDER > 0
#define DEFAULT_PRINTER_MODE PRINTER_MODE_FFF
#elif defined(SUPPORT_LASER) && SUPPORT_LASER
#define DEFAULT_PRINTER_MODE PRINTER_MODE_LASER
#elif defined(SUPPORT_CNC) && SUPPORT_CNC
#define DEFAULT_PRINTER_MODE PRINTER_MODE_CNC
#else
#error No supported printer mode compiled
#endif
#endif

extern bool runBedLeveling(int save); // save = S parameter in gcode

/**
The Printer class is the main class for the control of the 3d printer. Here all
movement related key variables are stored like positions, accelerations.

## Coordinates

The firmware works with 4 different coordinate systems and understanding the
dependencies between them is crucial to a good understanding on how positions
are handled.

### Real world floating coordinates (RWC)

These coordinates are the real floating positions with any offsets subtracted,
which might be set with G92. This is used to show coordinates or for computations
based on real positions. Any correction coming from rotation or distortion is
not included in these coordinates. currentPosition and lastCmdPos use this coordinate
system.

When these coordinates need to be used for computation, the value of offsetX, offsetY and offsetZ
is always added. These are the offsets of the currently active tool to virtual tool center
(normally first extruder).

### Rotated floating coordinates (ROTC)

If auto leveling is active, printing to the official coordinates is not possible. We have to assume
that the bed is somehow rotated against the Cartesian mechanics from the printer. Applying
_transformToPrinter_ to the real world coordinates, rotates them around the origin to
be equal to the rotated bed. _transformFromPrinter_ would apply the opposite transformation.

### Cartesian motor position coordinates (CMC)

The position of motors is stored as steps from 0. The reason for this is that it is crucial that
no rounding errors ever cause addition of any steps. These positions are simply computed by
multiplying the ROTC coordinates with the axisStepsPerMM.

If distortion correction is enabled, there is an additional factor for the z position that
gets added: _zCorrectionStepsIncluded_ This value is recalculated by every move added to
reflect the distortion at any given xyz position.

### Nonlinear motor position coordinates (NMC)

In case of a nonlinear mechanic like a delta printer, the CMC does not define the motor positions.
An additional transformation converts the CMC coordinates into NMC.

### Transformations from RWC to CMC

Given:
- Target position for tool: x_rwc, y_rwc, z_rwc
- Tool offsets: offsetX, offsetY, offsetZ
- Offset from bed leveling: offsetZ2

Step 1: Convert to ROTC

    transformToPrinter(x_rwc + Printer::offsetX, y_rwc + Printer::offsetY, z_rwc +  Printer::offsetZ, x_rotc, y_rotc, z_rotc);
    z_rotc += offsetZ2

Step 2: Compute CMC

    x_cmc = static_cast<int32_t>(floor(x_rotc * axisStepsPerMM[X_AXIS] + 0.5f));
    y_cmc = static_cast<int32_t>(floor(y_rotc * axisStepsPerMM[Y_AXIS] + 0.5f));
    z_cmc = static_cast<int32_t>(floor(z_rotc * axisStepsPerMM[Z_AXIS] + 0.5f));

### Transformation from CMC to RWC

Note: _zCorrectionStepsIncluded_ comes from distortion correction and gets set when a move is queued by the queuing function.
Therefore it is not visible in the inverse transformation above. When transforming back, consider if the value was set or not!

Step 1: Convert to ROTC

    x_rotc = static_cast<float>(x_cmc) * invAxisStepsPerMM[X_AXIS];
    y_rotc = static_cast<float>(y_cmc) * invAxisStepsPerMM[Y_AXIS];
    #if NONLINEAR_SYSTEM
    z_rotc = static_cast<float>(z_cmc * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
    #else
    z_rotc = static_cast<float>(z_cmc - zCorrectionStepsIncluded) * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
    #endif

Step 2: Convert to RWC

    transformFromPrinter(x_rotc, y_rotc, z_rotc,x_rwc, y_rwc, z_rwc);
    x_rwc -= Printer::offsetX; // Offset from active extruder or z probe
    y_rwc -= Printer::offsetY;
    z_rwc -= Printer::offsetZ;
*/
class Printer {
    static uint8_t debugLevel;
public:
#if USE_ADVANCE || defined(DOXYGEN)
    static volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
    static ufast8_t maxExtruderSpeed;            ///< Timer delay for end extruder speed
    //static uint8_t extruderAccelerateDelay;     ///< delay between 2 speec increases
    static int advanceStepsSet;
#if ENABLE_QUADRATIC_ADVANCE || defined(DOXYGEN)
    static long advanceExecuted;             ///< Executed advance steps
#endif
#endif
    static uint16_t menuMode;
#if DUAL_X_RESOLUTION || defined(DOXYGEN)
    static float axisX1StepsPerMM;
    static float axisX2StepsPerMM;
#endif
    static float axisStepsPerMM[]; ///< Resolution of each axis in steps per mm.
    static float invAxisStepsPerMM[]; ///< 1/axisStepsPerMM for faster computation.
    static float maxFeedrate[]; ///< Maximum feedrate in mm/s per axis.
    static float homingFeedrate[]; // Feedrate in mm/s for homing.
    // static uint32_t maxInterval; // slowest allowed interval
    static float maxAccelerationMMPerSquareSecond[];
    static float maxTravelAccelerationMMPerSquareSecond[];
    static unsigned long maxPrintAccelerationStepsPerSquareSecond[];
    static unsigned long maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t relativeCoordinateMode;    ///< Determines absolute (false) or relative Coordinates (true).
    static uint8_t relativeExtruderCoordinateMode;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

    static uint8_t unitIsInches;
    static uint8_t mode;
    static uint8_t fanSpeed; // Last fan speed set with M106/M107
    static fast8_t stepsPerTimerCall;
    static float zBedOffset;
    static uint8_t flag0, flag1; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect, 8 = homed
    static uint8_t flag2, flag3;
    static uint32_t interval;    ///< Last step duration in ticks.
    static uint32_t timer;              ///< used for acceleration/deceleration timing
    static uint32_t stepNumber;         ///< Step number in current move.
    static float coordinateOffset[Z_AXIS_ARRAY];
    static int32_t currentPositionSteps[E_AXIS_ARRAY];     ///< Position in steps from origin.
    static float currentPosition[Z_AXIS_ARRAY]; ///< Position in global coordinates
    static float lastCmdPos[Z_AXIS_ARRAY]; ///< Last coordinates (global coordinates) send by g-codes
    static int32_t destinationSteps[E_AXIS_ARRAY];         ///< Target position in steps.
    static millis_t lastTempReport;
    static float extrudeMultiplyError; ///< Accumulated error during extrusion
    static float extrusionFactor; ///< Extrusion multiply factor
#if NONLINEAR_SYSTEM || defined(DOXYGEN)
    static int32_t maxDeltaPositionSteps;
    static int32_t currentNonlinearPositionSteps[E_TOWER_ARRAY];
    static floatLong deltaDiagonalStepsSquaredA;
    static floatLong deltaDiagonalStepsSquaredB;
    static floatLong deltaDiagonalStepsSquaredC;
    static float deltaMaxRadiusSquared;
    static int32_t deltaFloorSafetyMarginSteps;
    static int32_t deltaAPosXSteps;
    static int32_t deltaAPosYSteps;
    static int32_t deltaBPosXSteps;
    static int32_t deltaBPosYSteps;
    static int32_t deltaCPosXSteps;
    static int32_t deltaCPosYSteps;
    static int32_t realDeltaPositionSteps[TOWER_ARRAY];
    static int16_t travelMovesPerSecond;
    static int16_t printMovesPerSecond;
    static float radius0;
#endif
#if !NONLINEAR_SYSTEM || defined(FAST_COREXYZ) || defined(DOXYGEN)
    static int32_t xMinStepsAdj, yMinStepsAdj, zMinStepsAdj; // adjusted to cover extruder/probe offsets
    static int32_t xMaxStepsAdj, yMaxStepsAdj, zMaxStepsAdj;
#endif
#if DRIVE_SYSTEM != DELTA || defined(DOXYGEN)
    static int32_t zCorrectionStepsIncluded;
#endif
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z || NONLINEAR_SYSTEM || defined(DOXYGEN)
    static int32_t stepsRemainingAtZHit;
#endif
#if DRIVE_SYSTEM == DELTA || defined(DOXYGEN)
    static int32_t stepsRemainingAtXHit;
    static int32_t stepsRemainingAtYHit;
#endif
#if SOFTWARE_LEVELING || defined(DOXYGEN)
    static int32_t levelingP1[3];
    static int32_t levelingP2[3];
    static int32_t levelingP3[3];
#endif
#if FEATURE_AUTOLEVEL || defined(DOXYGEN)
    static float autolevelTransformation[9]; ///< Transformation matrix
#endif
#if FAN_THERMO_PIN > -1 || defined(DOXYGEN)
    static float thermoMinTemp;
    static float thermoMaxTemp;
#endif
#if LAZY_DUAL_X_AXIS || defined(DOXYGEN)
    static bool sledParked;
#endif
#if FEATURE_BABYSTEPPING || defined(DOXYGEN)
    static int16_t zBabystepsMissing;
    static int16_t zBabysteps;
#endif
    //static float minimumSpeed;               ///< lowest allowed speed to keep integration error small
    //static float minimumZSpeed;              ///< lowest allowed speed to keep integration error small
    static int32_t xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t xMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static int32_t yMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static int32_t zMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static float xLength;
    static float xMin;
    static float yLength;
    static float yMin;
    static float zLength;
    static float zMin;
    static float feedrate;                   ///< Last requested feedrate.
    static int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
    static unsigned int extrudeMultiply;     ///< Flow multiplier in percent (factor 1 = 100)
    static float maxJerk;                    ///< Maximum allowed jerk in mm/s
    static uint8_t interruptEvent;           ///< Event generated in interrupts that should/could be handled in main thread
#if DRIVE_SYSTEM!=DELTA || defined(DOXYGEN)
    static float maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
#endif
    static float offsetX;                     ///< X-offset for different tool positions.
    static float offsetY;                     ///< Y-offset for different tool positions.
    static float offsetZ;                     ///< Z-offset for different tool positions.
    static float offsetZ2;                    ///< Z-offset without rotation correction. Required for z probe corrections
    static speed_t vMaxReached;               ///< Maximum reached speed
    static uint32_t msecondsPrinting;         ///< Milliseconds of printing time (means time with heated extruder)
    static float filamentPrinted;             ///< mm of filament printed since counting started
#if ENABLE_BACKLASH_COMPENSATION || defined(DOXYGEN)
    static float backlashX;
    static float backlashY;
    static float backlashZ;
    static uint8_t backlashDir;
#endif
#if MULTI_XENDSTOP_HOMING || defined(DOXYGEN)
    static fast8_t multiXHomeFlags;  // 1 = move X0, 2 = move X1
#endif
#if MULTI_YENDSTOP_HOMING || defined(DOXYGEN)
    static fast8_t multiYHomeFlags;  // 1 = move Y0, 2 = move Y1
#endif
#if MULTI_ZENDSTOP_HOMING || defined(DOXYGEN)
	static fast8_t multiZHomeFlags;  // 1 = move Z0, 2 = move Z1
#endif
    static float memoryX;
    static float memoryY;
    static float memoryZ;
    static float memoryE;
    static float memoryF;
#if (GANTRY && !defined(FAST_COREXYZ)) || defined(DOXYGEN)
    static int8_t motorX;
    static int8_t motorYorZ;
#endif
#ifdef DEBUG_SEGMENT_LENGTH
    static float maxRealSegmentLength;
#endif
#ifdef DEBUG_REAL_JERK
    static float maxRealJerk;
#endif
    // Print status related
    static int currentLayer;
    static int maxLayer; // -1 = unknown
    static char printName[21]; // max. 20 chars + 0
    static float progress;
    static fast8_t wizardStackPos;
    static wizardVar wizardStack[WIZARD_STACK_SIZE];

#if defined(DRV_TMC2130)
#if TMC2130_ON_X
    static TMC2130Stepper* tmc_driver_x;
#endif
#if TMC2130_ON_Y
    static TMC2130Stepper* tmc_driver_y;
#endif
#if TMC2130_ON_Z
    static TMC2130Stepper* tmc_driver_z;
#endif
#if TMC2130_ON_EXT0
    static TMC2130Stepper* tmc_driver_e0;
#endif
#if TMC2130_ON_EXT1
    static TMC2130Stepper* tmc_driver_e1;
#endif
#if TMC2130_ON_EXT2
    static TMC2130Stepper* tmc_driver_e2;
#endif
#endif

    static void handleInterruptEvent();

    static INLINE void setInterruptEvent(uint8_t evt, bool highPriority) {
        if(highPriority || interruptEvent == 0)
            interruptEvent = evt;
    }
    static void reportPrinterMode();
    static INLINE void setMenuMode(uint16_t mode, bool on) {
        if(on)
            menuMode |= mode;
        else
            menuMode &= ~mode;
    }

    static INLINE bool isMenuMode(uint8_t mode) {
        return (menuMode & mode) == mode;
    }
    static void setDebugLevel(uint8_t newLevel);
    static void toggleEcho();
    static void toggleInfo();
    static void toggleErrors();
    static void toggleDryRun();
    static void toggleCommunication();
    static void toggleNoMoves();
    static void toggleEndStop();
    static INLINE uint8_t getDebugLevel() {
        return debugLevel;
    }
    static INLINE bool debugEcho() {
        return ((debugLevel & 1) != 0);
    }

    static INLINE bool debugInfo() {
        return ((debugLevel & 2) != 0);
    }

    static INLINE bool debugErrors() {
        return ((debugLevel & 4) != 0);
    }

    static INLINE bool debugDryrun() {
        return ((debugLevel & 8) != 0);
    }

    static INLINE bool debugCommunication() {
        return ((debugLevel & 16) != 0);
    }

    static INLINE bool debugNoMoves() {
        return ((debugLevel & 32) != 0);
    }

    static INLINE bool debugEndStop() {
        return ((debugLevel & 64) != 0);
    }

    static INLINE bool debugFlag(uint8_t flags) {
        return (debugLevel & flags);
    }

    static INLINE void debugSet(uint8_t flags) {
        setDebugLevel(debugLevel | flags);
    }

    static INLINE void debugReset(uint8_t flags) {
        setDebugLevel(debugLevel & ~flags);
    }
#if AUTOMATIC_POWERUP
    static void enablePowerIfNeeded();
#endif
    /** Sets the pwm for the fan speed. Gets called by motion control or Commands::setFanSpeed. */
    static void setFanSpeedDirectly(uint8_t speed);
    /** Sets the pwm for the fan 2 speed. Gets called by motion control or Commands::setFan2Speed. */
    static void setFan2SpeedDirectly(uint8_t speed);
    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper() {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if (FEATURE_TWO_XSTEPPER || DUAL_X_AXIS) && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
    }

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper() {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_FOUR_ZSTEPPER && (Z4_ENABLE_PIN > -1)
        WRITE(Z4_ENABLE_PIN, !Z_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for x direction. */
    static INLINE void  enableXStepper() {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
#if (FEATURE_TWO_XSTEPPER || DUAL_X_AXIS) && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, X_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for y direction. */
    static INLINE void  enableYStepper() {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, Y_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static INLINE void  enableZStepper() {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_FOUR_ZSTEPPER && (Z4_ENABLE_PIN > -1)
        WRITE(Z4_ENABLE_PIN, Z_ENABLE_ON);
#endif
    }

    static INLINE void setXDirection(bool positive) {
        if(positive) {
            WRITE(X_DIR_PIN, !INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER || DUAL_X_AXIS
            WRITE(X2_DIR_PIN, !INVERT_X2_DIR);
#endif
        } else {
            WRITE(X_DIR_PIN, INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER || DUAL_X_AXIS
            WRITE(X2_DIR_PIN, INVERT_X2_DIR);
#endif
        }
    }

    static INLINE void setYDirection(bool positive) {
        if(positive) {
            WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, !INVERT_Y2_DIR);
#endif
        } else {
            WRITE(Y_DIR_PIN, INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, INVERT_Y2_DIR);
#endif
        }
    }
    static INLINE void setZDirection(bool positive) {
        if(positive) {
            WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, !INVERT_Z2_DIR);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, !INVERT_Z3_DIR);
#endif
#if FEATURE_FOUR_ZSTEPPER
            WRITE(Z4_DIR_PIN, !INVERT_Z4_DIR);
#endif
        } else {
            WRITE(Z_DIR_PIN, INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, INVERT_Z2_DIR);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, INVERT_Z3_DIR);
#endif
#if FEATURE_FOUR_ZSTEPPER
            WRITE(Z4_DIR_PIN, INVERT_Z4_DIR);
#endif
        }
    }

    static INLINE bool getZDirection() {
        return ((READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
    }

    static INLINE bool getYDirection() {
        return((READ(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
    }

    static INLINE bool getXDirection() {
        return((READ(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
    }

    /** For large machines, the nonlinear transformation can exceed integer 32bit range, so floating point math is needed. */
    static INLINE uint8_t isLargeMachine() {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    }

    static INLINE void setLargeMachine(uint8_t b) {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    }

    static INLINE uint8_t isAdvanceActivated() {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    }

    static INLINE void setAdvanceActivated(uint8_t b) {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    }

    static INLINE uint8_t isHomedAll() {
        return flag1 & PRINTER_FLAG1_HOMED_ALL;
    }

    static INLINE void unsetHomedAll() {
        flag1 &= ~PRINTER_FLAG1_HOMED_ALL;
        flag3 &= ~(PRINTER_FLAG3_X_HOMED | PRINTER_FLAG3_Y_HOMED | PRINTER_FLAG3_Z_HOMED);
    }

    static INLINE void updateHomedAll() {
        bool b = isXHomed() && isYHomed() && isZHomed();
        flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED_ALL : flag1 & ~PRINTER_FLAG1_HOMED_ALL);
    }

    static INLINE uint8_t isXHomed() {
        return flag3 & PRINTER_FLAG3_X_HOMED;
    }

    static INLINE void setXHomed(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
        updateHomedAll();
    }

    static INLINE uint8_t isYHomed() {
        return flag3 & PRINTER_FLAG3_Y_HOMED;
    }

    static INLINE void setYHomed(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
        updateHomedAll();
    }

    static INLINE uint8_t isZHomed() {
        return flag3 & PRINTER_FLAG3_Z_HOMED;
    }

    static INLINE void setZHomed(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_Z_HOMED : flag3 & ~PRINTER_FLAG3_Z_HOMED);
        updateHomedAll();
    }
    static INLINE uint8_t isAutoreportTemp() {
        return flag3 & PRINTER_FLAG3_AUTOREPORT_TEMP;
    }

    static INLINE void setAutoreportTemp(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_AUTOREPORT_TEMP : flag3 & ~PRINTER_FLAG3_AUTOREPORT_TEMP);
    }

    static INLINE uint8_t isAllKilled() {
        return flag1 & PRINTER_FLAG1_ALLKILLED;
    }

    static INLINE void setAllKilled(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
    }

    static INLINE uint8_t isAutomount() {
        return flag1 & PRINTER_FLAG1_AUTOMOUNT;
    }

    static INLINE void setAutomount(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_AUTOMOUNT : flag1 & ~PRINTER_FLAG1_AUTOMOUNT);
    }

    static INLINE uint8_t isAnimation() {
        return flag1 & PRINTER_FLAG1_ANIMATION;
    }

    static INLINE void setAnimation(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ANIMATION : flag1 & ~PRINTER_FLAG1_ANIMATION);
    }

    static INLINE uint8_t isUIErrorMessage() {
        return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
    }

    static INLINE void setUIErrorMessage(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
    }

    static INLINE uint8_t isNoDestinationCheck() {
        return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
    }

    static INLINE void setNoDestinationCheck(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
    }

    static INLINE uint8_t isPowerOn() {
        return flag1 & PRINTER_FLAG1_POWER_ON;
    }

    static INLINE void setPowerOn(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_POWER_ON : flag1 & ~PRINTER_FLAG1_POWER_ON);
    }

    static INLINE uint8_t isColdExtrusionAllowed() {
        return flag1 & PRINTER_FLAG1_ALLOW_COLD_EXTRUSION;
    }

    static INLINE void setColdExtrusionAllowed(uint8_t b) {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLOW_COLD_EXTRUSION : flag1 & ~PRINTER_FLAG1_ALLOW_COLD_EXTRUSION);
        if(b)
            Com::printFLN(PSTR("Cold extrusion allowed"));
        else
            Com::printFLN(PSTR("Cold extrusion disallowed"));
    }

    static INLINE uint8_t isBlockingReceive() {
        return flag2 & PRINTER_FLAG2_BLOCK_RECEIVING;
    }

    static INLINE void setBlockingReceive(uint8_t b) {
        flag2 = (b ? flag2 | PRINTER_FLAG2_BLOCK_RECEIVING : flag2 & ~PRINTER_FLAG2_BLOCK_RECEIVING);
        Com::printFLN(b ? Com::tPauseCommunication : Com::tContinueCommunication);
    }

    static INLINE uint8_t isAutoretract() {
        return flag2 & PRINTER_FLAG2_AUTORETRACT;
    }

    static INLINE void setAutoretract(uint8_t b) {
        flag2 = (b ? flag2 | PRINTER_FLAG2_AUTORETRACT : flag2 & ~PRINTER_FLAG2_AUTORETRACT);
        Com::printFLN(PSTR("Autoretract:"), b);
    }

    static INLINE uint8_t isPrinting() {
        return flag3 & PRINTER_FLAG3_PRINTING;
    }

    static INLINE void setPrinting(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_PRINTING : flag3 & ~PRINTER_FLAG3_PRINTING);
        Printer::setMenuMode(MENU_MODE_PRINTING, b);
    }

    static INLINE uint8_t isStartStopSupported() {
        return flag3 & PRINTER_FLAG3_SUPPORTS_STARTSTOP;
    }

    static INLINE void setSupportStartStop(uint8_t b) {
        flag3 = (b ? flag3 | PRINTER_FLAG3_SUPPORTS_STARTSTOP : flag3 & ~PRINTER_FLAG3_SUPPORTS_STARTSTOP);
    }

    static INLINE uint8_t isDoorOpen() {
        return (flag3 & PRINTER_FLAG3_DOOR_OPEN) != 0;
    }

    static bool updateDoorOpen();

    static INLINE uint8_t isHoming() {
        return flag2 & PRINTER_FLAG2_HOMING;
    }

    static INLINE void setHoming(uint8_t b) {
        flag2 = (b ? flag2 | PRINTER_FLAG2_HOMING : flag2 & ~PRINTER_FLAG2_HOMING);
    }
    static INLINE uint8_t isAllEMotors() {
        return flag2 & PRINTER_FLAG2_ALL_E_MOTORS;
    }

    static INLINE void setAllEMotors(uint8_t b) {
        flag2 = (b ? flag2 | PRINTER_FLAG2_ALL_E_MOTORS : flag2 & ~PRINTER_FLAG2_ALL_E_MOTORS);
    }

    static INLINE uint8_t isDebugJam() {
        return (flag2 & PRINTER_FLAG2_DEBUG_JAM) != 0;
    }

    static INLINE uint8_t isDebugJamOrDisabled() {
        return (flag2 & (PRINTER_FLAG2_DEBUG_JAM | PRINTER_FLAG2_JAMCONTROL_DISABLED)) != 0;
    }

    static INLINE void setDebugJam(uint8_t b) {
        flag2 = (b ? flag2 | PRINTER_FLAG2_DEBUG_JAM : flag2 & ~PRINTER_FLAG2_DEBUG_JAM);
        Com::printFLN(PSTR("Jam debugging:"), b);
    }

    static INLINE uint8_t isJamcontrolDisabled() {
        return (flag2 & PRINTER_FLAG2_JAMCONTROL_DISABLED) != 0;
    }

    static INLINE void setJamcontrolDisabled(uint8_t b) {
#if EXTRUDER_JAM_CONTROL
        if(b)
            Extruder::markAllUnjammed();
#endif
        flag2 = (b ? flag2 | PRINTER_FLAG2_JAMCONTROL_DISABLED : flag2 & ~PRINTER_FLAG2_JAMCONTROL_DISABLED);
        Com::printFLN(PSTR("Jam control disabled:"), b);
    }

    static INLINE void toggleAnimation() {
        setAnimation(!isAnimation());
    }
    static INLINE float convertToMM(float x) {
        return (unitIsInches ? x * 25.4 : x);
    }
    static INLINE bool areAllSteppersDisabled() {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void setAllSteppersDiabled() {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void unsetAllSteppersDisabled() {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;
#if FAN_BOARD_PIN > -1
        pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_SPEED;
#endif // FAN_BOARD_PIN
    }
    static INLINE bool isAnyTempsensorDefect() {
        return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    }
    static INLINE void setAnyTempsensorDefect() {
        flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
        debugSet(8);
    }
    static INLINE void unsetAnyTempsensorDefect() {
        flag0 &= ~PRINTER_FLAG0_TEMPSENSOR_DEFECT;
    }
    static INLINE bool isManualMoveMode() {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE void setManualMoveMode(bool on) {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE bool isAutolevelActive() {
        return (flag0 & PRINTER_FLAG0_AUTOLEVEL_ACTIVE) != 0;
    }
    static void setAutolevelActive(bool on);
    static INLINE void setZProbingActive(bool on) {
        flag0 = (on ? flag0 | PRINTER_FLAG0_ZPROBEING : flag0 & ~PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE bool isZProbingActive() {
        return (flag0 & PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE void executeXYGantrySteps() {
#if (GANTRY) && !defined(FAST_COREXYZ)
        if(motorX <= -2) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorX += 2;
        } else if(motorX >= 2) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorX -= 2;
        }
        if(motorYorZ <= -2) {
            WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorYorZ += 2;
        } else if(motorYorZ >= 2) {
            WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorYorZ -= 2;
        }
#endif
    }
    static INLINE void executeXZGantrySteps() {
#if (GANTRY) && !defined(FAST_COREXYZ)
        if(motorX <= -2) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorX += 2;
        } else if(motorX >= 2) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorX -= 2;
        }
        if(motorYorZ <= -2) {
            WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_FOUR_ZSTEPPER
            WRITE(Z4_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorYorZ += 2;
        } else if(motorYorZ >= 2) {
            WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_FOUR_ZSTEPPER
            WRITE(Z4_STEP_PIN, START_STEP_WITH_HIGH);
#endif
            motorYorZ -= 2;
        }
#endif
    }
    static INLINE void startXStep() {
#if DUAL_X_AXIS
  #if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
            return;
        }
  #endif
        if(Extruder::current->id) {
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
        } else {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
        }
#else // DUAL_X_AXIS
#if MULTI_XENDSTOP_HOMING
        if(Printer::multiXHomeFlags & 1) {
            WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_XSTEPPER
        if(Printer::multiXHomeFlags & 2) {
            WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else // MULTI_XENDSTOP_HOMING
        WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
#endif
    }
    static INLINE void startYStep() {
#if MULTI_YENDSTOP_HOMING
        if(Printer::multiYHomeFlags & 1) {
            WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_YSTEPPER
        if(Printer::multiYHomeFlags & 2) {
            WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else
        WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
    }
    static INLINE void startZStep() {
#if MULTI_ZENDSTOP_HOMING
        if(Printer::multiZHomeFlags & 1) {
            WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
        }
#if FEATURE_TWO_ZSTEPPER
        if(Printer::multiZHomeFlags & 2) {
            WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#if FEATURE_THREE_ZSTEPPER
        if(Printer::multiZHomeFlags & 4) {
            WRITE(Z3_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#if FEATURE_FOUR_ZSTEPPER
        if(Printer::multiZHomeFlags & 8) {
            WRITE(Z4_STEP_PIN, START_STEP_WITH_HIGH);
        }
#endif
#else
        WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#if FEATURE_FOUR_ZSTEPPER
        WRITE(Z4_STEP_PIN, START_STEP_WITH_HIGH);
#endif
#endif
    }
    static INLINE void endXYZSteps() {
        WRITE(X_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER || DUAL_X_AXIS
        WRITE(X2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
        WRITE(Y_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
        WRITE(Z_STEP_PIN, !START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
#if FEATURE_FOUR_ZSTEPPER
        WRITE(Z4_STEP_PIN, !START_STEP_WITH_HIGH);
#endif
    }
    static INLINE speed_t updateStepsPerTimerCall(speed_t vbase) {
        if(vbase > STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
            if(vbase > STEP_DOUBLER_FREQUENCY * 2) {
                Printer::stepsPerTimerCall = 4;
                return vbase >> 2;
            } else {
                Printer::stepsPerTimerCall = 2;
                return vbase >> 1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            return vbase >> 1;
#endif
        } else {
            Printer::stepsPerTimerCall = 1;
        }
        return vbase;
    }
    static INLINE void disableAllowedStepper() {
#if DRIVE_SYSTEM == XZ_GANTRY || DRIVE_SYSTEM == ZX_GANTRY
        if(DISABLE_X && DISABLE_Z) {
            disableXStepper();
            disableZStepper();
        }
        if(DISABLE_Y) disableYStepper();
#else
#if GANTRY
        if(DISABLE_X && DISABLE_Y) {
            disableXStepper();
            disableYStepper();
        }
#else
        if(DISABLE_X) disableXStepper();
        if(DISABLE_Y) disableYStepper();
#endif
        if(DISABLE_Z) disableZStepper();
#endif
    }
    static INLINE float realXPosition() {
        return currentPosition[X_AXIS];
    }

    static INLINE float realYPosition() {
        return currentPosition[Y_AXIS];
    }

    static INLINE float realZPosition() {
        return currentPosition[Z_AXIS];
    }
    /** \brief copies currentPosition to parameter. */
    static INLINE void realPosition(float &xp, float &yp, float &zp) {
        xp = currentPosition[X_AXIS];
        yp = currentPosition[Y_AXIS];
        zp = currentPosition[Z_AXIS];
    }
    static INLINE void insertStepperHighDelay() {
#if STEPPER_HIGH_DELAY>0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
    }
    static void updateDerivedParameter();
    /** If we are not homing or destination check being disabled, this will reduce _destinationSteps_ to a
    valid value. In other words this works as software endstop. */
    static void constrainDestinationCoords();
    /** Computes _currentposition_ from _currentPositionSteps_ considering all active transformations. If the _copyLastCmd_ flag is true, the
    result is also copied to _lastCmdPos_ . */
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void updateCurrentPositionSteps();
    /** \brief Sets the destination coordinates to values stored in com.

    Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
    position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
    \param com g-code with new destination position.
    \return true if it is a move, false if no move results from coordinates.
     */
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    /** \brief Move to position without considering transformations.

    Computes the destinationSteps without rotating but including active offsets!
    The coordinates are in printer coordinates with no G92 offset.

    \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
    \return true if queuing was successful.
    */
    static uint8_t moveTo(float x, float y, float z, float e, float f);
    /** \brief Move to position considering transformations.

    Computes the destinationSteps including rotating and active offsets.
    The coordinates are in printer coordinates with no G92 offset.

    \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
    \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
    \param pathOptimize true if path planner should include it in calculation, otherwise default start/end speed is enforced.
    \return true if queuing was successful.
    */
    static uint8_t moveToReal(float x, float y, float z, float e, float f, bool pathOptimize = true);
    static void kill(uint8_t only_steppers);
    static void updateAdvanceFlags();
    static void setup();
    static void defaultLoopActions();
    static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
    static void setOrigin(float xOff, float yOff, float zOff);
    /** \brief Tests if the target position is allowed.

    Tests if the test position lies inside the defined geometry. For Cartesian
    printers this is the defined cube defined by x,y,z min and length. For
    delta printers the cylindrical shape is tested.

    \param x X position in mm.
    \param x Y position in mm.
    \param x Z position in mm.
    \return true if position is valid and can be reached. */
    static bool isPositionAllowed(float x, float y, float z);
    static INLINE int getFanSpeed() {
        return (int)pwm_pos[PWM_FAN1];
    }
    static INLINE int getFan2Speed() {
        return (int)pwm_pos[PWM_FAN2];
    }
#if NONLINEAR_SYSTEM || defined(DOXYGEN)
    static INLINE void setDeltaPositions(long xaxis, long yaxis, long zaxis) {
        currentNonlinearPositionSteps[A_TOWER] = xaxis;
        currentNonlinearPositionSteps[B_TOWER] = yaxis;
        currentNonlinearPositionSteps[C_TOWER] = zaxis;
    }
    static void deltaMoveToTopEndstops(float feedrate);
#endif
#if MAX_HARDWARE_ENDSTOP_Z || defined(DOXYGEN)
    static float runZMaxProbe();
#endif
#if FEATURE_Z_PROBE || defined(DOXYGEN)
    static bool startProbing(bool runScript, bool enforceStartHeight = true);
    static void finishProbing();
    static float runZProbe(bool first, bool last, uint8_t repeat = Z_PROBE_REPETITIONS, bool runStartScript = true, bool enforceStartHeight = true);
    static void measureZProbeHeight(float curHeight);
    static void waitForZProbeStart();
    static float bendingCorrectionAt(float x, float y);
#endif
    // Moved outside FEATURE_Z_PROBE to allow auto-level functional test on
    // system without Z-probe
    static void transformToPrinter(float x, float y, float z, float &transX, float &transY, float &transZ);
    static void transformFromPrinter(float x, float y, float z, float &transX, float &transY, float &transZ);
#if FEATURE_AUTOLEVEL || defined(DOXYGEN)
    static void resetTransformationMatrix(bool silent);
    //static void buildTransformationMatrix(float h1,float h2,float h3);
    static void buildTransformationMatrix(Plane &plane);
#endif
#if DISTORTION_CORRECTION || defined(DOXYGEN)
    static void measureDistortion(void);
    static Distortion distortion;
#endif
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed);
    static void zBabystep();

    static INLINE void resetWizardStack() {
        wizardStackPos = 0;
    }
    static INLINE void pushWizardVar(wizardVar v) {
        wizardStack[wizardStackPos++] = v;
    }
    static INLINE wizardVar popWizardVar() {
        return wizardStack[--wizardStackPos];
    }
    static void showConfiguration();
    static void setCaseLight(bool on);
    static void reportCaseLightStatus();
#if JSON_OUTPUT || defined(DOXYGEN)
    static void showJSONStatus(int type);
#endif
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
    static void pausePrint();
    static void continuePrint();
    static void stopPrint();
	static void moveToParkPosition();
#if FEATURE_Z_PROBE || defined(DOXYGEN)
    /** \brief Prepares printer for probing commands.

    Probing can not start under all conditions. This command therefore makes sure,
    a probing command can be executed by:
    - Ensuring all axes are homed.
    - Going to a low z position for fast measuring.
    - Go to a position, where enabling the z-probe is possible without leaving the valid print area.
    */
    static void prepareForProbing();
#endif
#if defined(DRV_TMC2130)
    static void configTMC2130(TMC2130Stepper* tmc_driver, bool tmc_stealthchop, int8_t tmc_sgt,
      uint8_t tmc_pwm_ampl, uint8_t tmc_pwm_grad, bool tmc_pwm_autoscale, uint8_t tmc_pwm_freq);
    static void tmcPrepareHoming(TMC2130Stepper* tmc_driver, uint32_t coolstep_sp_min);
#endif
};

#endif // PRINTER_H_INCLUDED
