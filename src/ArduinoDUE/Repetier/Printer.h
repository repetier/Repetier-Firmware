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

union floatLong
{
    float f;
    uint32_t l;
#ifdef SUPPORT_64_BIT_MATH
    uint64_t L;
#endif
};

union wizardVar
{
    float f;
    int32_t l;
    uint32_t ul;
    int16_t i;
    uint16_t ui;
    int8_t c;
    uint8_t uc;

    wizardVar():i(0) {}
    wizardVar(float _f):f(_f) {}
    wizardVar(int32_t _f):l(_f) {}
    wizardVar(uint32_t _f):ul(_f) {}
    wizardVar(int16_t _f):i(_f) {}
    wizardVar(uint16_t _f):ui(_f) {}
    wizardVar(int8_t _f):c(_f) {}
    wizardVar(uint8_t _f):uc(_f) {}
};

#define PRINTER_FLAG0_STEPPER_DISABLED      1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     4
#define PRINTER_FLAG0_FORCE_CHECKSUM        8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE      16
#define PRINTER_FLAG0_AUTOLEVEL_ACTIVE      32
#define PRINTER_FLAG0_ZPROBEING             64
#define PRINTER_FLAG0_LARGE_MACHINE         128
#define PRINTER_FLAG1_HOMED                 1
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

// List of possible interrupt events (1-255 allowed)
#define PRINTER_INTERRUPT_EVENT_JAM_DETECTED 1
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL0 2
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL1 3
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL2 4
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL3 5
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL4 6
#define PRINTER_INTERRUPT_EVENT_JAM_SIGNAL5 7
// define an integer number of steps more than large enough to get to endstop from anywhere
#define HOME_DISTANCE_STEPS (Printer::zMaxSteps-Printer::zMinSteps+1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])
// Some dfines to make clearer reading, as we overload these cartesion memory locations for delta
#define towerAMaxSteps Printer::xMaxSteps
#define towerBMaxSteps Printer::yMaxSteps
#define towerCMaxSteps Printer::zMaxSteps
#define towerAMinSteps Printer::xMinSteps
#define towerBMinSteps Printer::yMinSteps
#define towerCMinSteps Printer::zMinSteps

#if DISTORTION_CORRECTION
class Distortion
{
public:
    Distortion();
    void init();
    void enable(bool permanent = true);
    void disable(bool permanent = true);
    void measure(void);
    int32_t correct(int32_t x, int32_t y, int32_t z) const;
    void updateDerived();
    void reportStatus();
private:
    INLINE int matrixIndex(fast8_t x, fast8_t y) const;
    INLINE int32_t getMatrix(int index) const;
    INLINE void setMatrix(int32_t val, int index);
    bool isCorner(fast8_t i, fast8_t j) const;
    INLINE int32_t extrapolatePoint(fast8_t x1, fast8_t y1, fast8_t x2, fast8_t y2) const;
    void extrapolateCorner(fast8_t x, fast8_t y, fast8_t dx, fast8_t dy);
    void extrapolateCorners();
    void resetCorrection();
// attributes
    int32_t step;
    int32_t radiusCorrectionSteps;
    int32_t zStart,zEnd;
#if !DISTORTION_PERMANENT
    int32_t matrix[DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS];
#endif
    bool enabled;
};
#endif //DISTORTION_CORRECTION

#define ENDSTOP_X_MIN_ID 1
#define ENDSTOP_X_MAX_ID 2
#define ENDSTOP_Y_MIN_ID 4
#define ENDSTOP_Y_MAX_ID 8
#define ENDSTOP_Z_MIN_ID 16
#define ENDSTOP_Z_MAX_ID 32
#define ENDSTOP_Z2_MIN_ID 64
#define ENDSTOP_Z_PROBE_ID 128

// These endstops are only used with EXTENDED_ENDSTOPS
#define ENDSTOP_X2_MIN_ID 1
#define ENDSTOP_X2_MAX_ID 2
#define ENDSTOP_Y2_MIN_ID 4
#define ENDSTOP_Y2_MAX_ID 8
#define ENDSTOP_Z2_MAX_ID 16
#define ENDSTOP_Z3_MIN_ID 32
#define ENDSTOP_Z3_MAX_ID 64

class Endstops {
    static flag8_t lastState;
    static flag8_t lastRead;
    static flag8_t accumulator;
#ifdef EXTENDED_ENDSTOPS
    static flag8_t lastState2;
    static flag8_t lastRead2;
    static flag8_t accumulator2;
#endif
public:
    static void update();
    static void report();
    static INLINE bool anyXYZMax() {
        return (lastState & (ENDSTOP_X_MAX_ID|ENDSTOP_Z_MAX_ID|ENDSTOP_Z_MAX_ID)) != 0;
    }
    static INLINE void resetAccumulator() {
        accumulator = 0;
#ifdef EXTENDED_ENDSTOPS
        accumulator2 = 0;
#endif
    }
    static INLINE void fillFromAccumulator() {
        lastState = accumulator;
#ifdef EXTENDED_ENDSTOPS
        lastState2 = accumulator2;
#endif
    }
    static INLINE bool xMin() {
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
        return (lastState & ENDSTOP_X_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool xMax() {
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
        return (lastState & ENDSTOP_X_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool yMin() {
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
        return (lastState & ENDSTOP_Y_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool yMax() {
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
        return (lastState & ENDSTOP_Y_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zMin() {
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
        return (lastState & ENDSTOP_Z_MIN_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zMax() {
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
        return (lastState & ENDSTOP_Z_MAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool z2MinMax() {
#if (Z2_MINMAX_PIN > -1) && MINMAX_HARDWARE_ENDSTOP_Z2
        return (lastState & ENDSTOP_Z2_MINMAX_ID) != 0;
#else
        return false;
#endif
    }
    static INLINE bool zProbe() {
#if FEATURE_Z_PROBE
        return (lastState & ENDSTOP_Z_PROBE_ID) != 0;
#else
        return false;
#endif
    }
};

class Printer
{
public:
#if USE_ADVANCE
    static volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
    static ufast8_t maxExtruderSpeed;            ///< Timer delay for end extruder speed
    //static uint8_t extruderAccelerateDelay;     ///< delay between 2 speec increases
    static int advanceStepsSet;
#if ENABLE_QUADRATIC_ADVANCE
    static long advanceExecuted;             ///< Executed advance steps
#endif
#endif
    static uint8_t menuMode;
    static float axisStepsPerMM[];
    static float invAxisStepsPerMM[];
    static float maxFeedrate[];
    static float homingFeedrate[];
    static float maxAccelerationMMPerSquareSecond[];
    static float maxTravelAccelerationMMPerSquareSecond[];
    static unsigned long maxPrintAccelerationStepsPerSquareSecond[];
    static unsigned long maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t relativeCoordinateMode;    ///< Determines absolute (false) or relative Coordinates (true).
    static uint8_t relativeExtruderCoordinateMode;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

    static uint8_t unitIsInches;
    static float zBedOffset;
    static uint8_t debugLevel;
    static uint8_t flag0,flag1; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect, 8 = homed
    static uint8_t flag2;
    static uint8_t stepsPerTimerCall;
    static uint32_t interval;    ///< Last step duration in ticks.
    static uint32_t timer;              ///< used for acceleration/deceleration timing
    static uint32_t stepNumber;         ///< Step number in current move.
    static float coordinateOffset[Z_AXIS_ARRAY];
    static int32_t currentPositionSteps[E_AXIS_ARRAY];     ///< Position in steps from origin.
    static float currentPosition[Z_AXIS_ARRAY];
    static float lastCmdPos[Z_AXIS_ARRAY]; ///< Last coordinates send by gcodes
    static int32_t destinationSteps[E_AXIS_ARRAY];         ///< Target position in steps.
    static float extrudeMultiplyError; ///< Accumulated error during extrusion
    static float extrusionFactor; ///< Extrusion multiply factor
#if NONLINEAR_SYSTEM
    static int32_t maxDeltaPositionSteps;
    static int32_t currentDeltaPositionSteps[E_TOWER_ARRAY];
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
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z || NONLINEAR_SYSTEM
    static int32_t stepsRemainingAtZHit;
#endif
#if DRIVE_SYSTEM==DELTA
    static int32_t stepsRemainingAtXHit;
    static int32_t stepsRemainingAtYHit;
#endif
#if SOFTWARE_LEVELING
    static int32_t levelingP1[3];
    static int32_t levelingP2[3];
    static int32_t levelingP3[3];
#endif
#if FEATURE_AUTOLEVEL
    static float autolevelTransformation[9]; ///< Transformation matrix
#endif
    static int16_t zBabystepsMissing;
    static float minimumSpeed;               ///< lowest allowed speed to keep integration error small
    static float minimumZSpeed;              ///< lowest allowed speed to keep integration error small
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
    static unsigned int extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
    static float maxJerk;                    ///< Maximum allowed jerk in mm/s
    static uint8_t interruptEvent;           ///< Event generated in interrupts that should/could be handled in main thread
#if DRIVE_SYSTEM!=DELTA
    static float maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
#endif
    static float offsetX;                     ///< X-offset for different extruder positions.
    static float offsetY;                     ///< Y-offset for different extruder positions.
    static float offsetZ;                     ///< Y-offset for different extruder positions.
    static speed_t vMaxReached;         ///< Maximumu reached speed
    static uint32_t msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
    static float filamentPrinted;            ///< mm of filament printed since counting started
#if ENABLE_BACKLASH_COMPENSATION
    static float backlashX;
    static float backlashY;
    static float backlashZ;
    static uint8_t backlashDir;
#endif
    static float memoryX;
    static float memoryY;
    static float memoryZ;
    static float memoryE;
    static float memoryF;
#if GANTRY
    static int8_t motorX;
    static int8_t motorYorZ;
#endif
#ifdef DEBUG_SEGMENT_LENGTH
    static float maxRealSegmentLength;
#endif
#ifdef DEBUG_REAL_JERK
    static float maxRealJerk;
#endif
    static fast8_t wizardStackPos;
    static wizardVar wizardStack[WIZARD_STACK_SIZE];

    static void handleInterruptEvent();

    static INLINE void setInterruptEvent(uint8_t evt, bool highPriority)
    {
        if(highPriority || interruptEvent == 0)
            interruptEvent = evt;
    }

    static INLINE void setMenuMode(uint8_t mode,bool on)
    {
        if(on)
            menuMode |= mode;
        else
            menuMode &= ~mode;
    }

    static INLINE bool isMenuMode(uint8_t mode)
    {
        return (menuMode & mode) == mode;
    }

    static INLINE bool debugEcho()
    {
        return ((debugLevel & 1) != 0);
    }

    static INLINE bool debugInfo()
    {
        return ((debugLevel & 2) != 0);
    }

    static INLINE bool debugErrors()
    {
        return ((debugLevel & 4) != 0);
    }

    static INLINE bool debugDryrun()
    {
        return ((debugLevel & 8) != 0);
    }

    static INLINE bool debugCommunication()
    {
        return ((debugLevel & 16) != 0);
    }

    static INLINE bool debugNoMoves()
    {
        return ((debugLevel & 32) != 0);
    }

    static INLINE bool debugFlag(uint8_t flags)
    {
        return (debugLevel & flags);
    }

    static INLINE void debugSet(uint8_t flags)
    {
        debugLevel |= flags;
    }

    static INLINE void debugReset(uint8_t flags)
    {
        debugLevel &= ~flags;
    }

    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
    }

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, !Z_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for x direction. */
    static INLINE void  enableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, X_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for y direction. */
    static INLINE void  enableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, Y_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static INLINE void  enableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, Z_ENABLE_ON);
#endif
    }

    static INLINE void setXDirection(bool positive)
    {
        if(positive)
        {
            WRITE(X_DIR_PIN,!INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN,!INVERT_X_DIR);
#endif
        }
        else
        {
            WRITE(X_DIR_PIN,INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN,INVERT_X_DIR);
#endif
        }
    }

    static INLINE void setYDirection(bool positive)
    {
        if(positive)
        {
            WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, !INVERT_Y_DIR);
#endif
        }
        else
        {
            WRITE(Y_DIR_PIN, INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, INVERT_Y_DIR);
#endif
        }
    }
    static INLINE void setZDirection(bool positive)
    {
        if(positive)
        {
            WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, !INVERT_Z_DIR);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, !INVERT_Z_DIR);
#endif
        }
        else
        {
            WRITE(Z_DIR_PIN, INVERT_Z_DIR);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, INVERT_Z_DIR);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, INVERT_Z_DIR);
#endif
        }
    }

    static INLINE bool getZDirection()
    {
        return ((READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
    }

    static INLINE bool getYDirection()
    {
        return((READ(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
    }

    static INLINE bool getXDirection()
    {
        return((READ(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
    }

    static INLINE uint8_t isLargeMachine()
    {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    }

    static INLINE void setLargeMachine(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    }

    static INLINE uint8_t isAdvanceActivated()
    {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    }

    static INLINE void setAdvanceActivated(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    }

    static INLINE uint8_t isHomed()
    {
        return flag1 & PRINTER_FLAG1_HOMED;
    }

    static INLINE void setHomed(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED : flag1 & ~PRINTER_FLAG1_HOMED);
    }

    static INLINE uint8_t isAllKilled()
    {
        return flag1 & PRINTER_FLAG1_ALLKILLED;
    }

    static INLINE void setAllKilled(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
    }

    static INLINE uint8_t isAutomount()
    {
        return flag1 & PRINTER_FLAG1_AUTOMOUNT;
    }

    static INLINE void setAutomount(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_AUTOMOUNT : flag1 & ~PRINTER_FLAG1_AUTOMOUNT);
    }

    static INLINE uint8_t isAnimation()
    {
        return flag1 & PRINTER_FLAG1_ANIMATION;
    }

    static INLINE void setAnimation(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ANIMATION : flag1 & ~PRINTER_FLAG1_ANIMATION);
    }

    static INLINE uint8_t isUIErrorMessage()
    {
        return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
    }

    static INLINE void setUIErrorMessage(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
    }

    static INLINE uint8_t isNoDestinationCheck()
    {
        return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
    }

    static INLINE void setNoDestinationCheck(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
    }

    static INLINE uint8_t isPowerOn()
    {
        return flag1 & PRINTER_FLAG1_POWER_ON;
    }

    static INLINE void setPowerOn(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_POWER_ON : flag1 & ~PRINTER_FLAG1_POWER_ON);
    }

    static INLINE uint8_t isColdExtrusionAllowed()
    {
        return flag1 & PRINTER_FLAG1_ALLOW_COLD_EXTRUSION;
    }

    static INLINE void setColdExtrusionAllowed(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLOW_COLD_EXTRUSION : flag1 & ~PRINTER_FLAG1_ALLOW_COLD_EXTRUSION);
        if(b)
            Com::printFLN(PSTR("Cold extrusion allowed"));
        else
            Com::printFLN(PSTR("Cold extrusion disallowed"));
    }

    static INLINE uint8_t isBlockingReceive()
    {
        return flag2 & PRINTER_FLAG2_BLOCK_RECEIVING;
    }

    static INLINE void setBlockingReceive(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_BLOCK_RECEIVING : flag2 & ~PRINTER_FLAG2_BLOCK_RECEIVING);
        Com::printFLN(b ? Com::tPauseCommunication : Com::tContinueCommunication);
    }

    static INLINE uint8_t isAutoretract()
    {
        return flag2 & PRINTER_FLAG2_AUTORETRACT;
    }

    static INLINE void setAutoretract(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_AUTORETRACT : flag2 & ~PRINTER_FLAG2_AUTORETRACT);
        Com::printFLN(PSTR("Autoretract:"),b);
    }
    static INLINE uint8_t isHoming()
    {
        return flag2 & PRINTER_FLAG2_HOMING;
    }

    static INLINE void setHoming(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_HOMING : flag2 & ~PRINTER_FLAG2_HOMING);
    }

    static INLINE uint8_t isDebugJam()
    {
        return (flag2 & PRINTER_FLAG2_DEBUG_JAM) != 0;
    }

    static INLINE uint8_t isDebugJamOrDisabled()
    {
        return (flag2 & (PRINTER_FLAG2_DEBUG_JAM | PRINTER_FLAG2_JAMCONTROL_DISABLED)) != 0;
    }

    static INLINE void setDebugJam(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_DEBUG_JAM : flag2 & ~PRINTER_FLAG2_DEBUG_JAM);
        Com::printFLN(PSTR("Jam debugging:"),b);
    }

    static INLINE uint8_t isJamcontrolDisabled()
    {
        return (flag2 & PRINTER_FLAG2_JAMCONTROL_DISABLED) != 0;
    }

    static INLINE void setJamcontrolDisabled(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_JAMCONTROL_DISABLED : flag2 & ~PRINTER_FLAG2_JAMCONTROL_DISABLED);
        Com::printFLN(PSTR("Jam control disabled:"),b);
    }

    static INLINE void toggleAnimation()
    {
        setAnimation(!isAnimation());
    }
    static INLINE float convertToMM(float x)
    {
        return (unitIsInches ? x*25.4 : x);
    }
    static INLINE bool areAllSteppersDisabled()
    {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void setAllSteppersDiabled()
    {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void unsetAllSteppersDisabled()
    {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;
#if FAN_BOARD_PIN>-1
        pwm_pos[NUM_EXTRUDER + 1] = 255;
#endif // FAN_BOARD_PIN
    }
    static INLINE bool isAnyTempsensorDefect()
    {
        return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    }
    static INLINE void setAnyTempsensorDefect()
    {
        flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
    }
    static INLINE void unsetAnyTempsensorDefect()
    {
        flag0 &= ~PRINTER_FLAG0_TEMPSENSOR_DEFECT;
    }
    static INLINE bool isManualMoveMode()
    {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE void setManualMoveMode(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE bool isAutolevelActive()
    {
        return (flag0 & PRINTER_FLAG0_AUTOLEVEL_ACTIVE)!=0;
    }
    static void setAutolevelActive(bool on);
    static INLINE void setZProbingActive(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_ZPROBEING : flag0 & ~PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE bool isZProbingActive()
    {
        return (flag0 & PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE void executeXYGantrySteps()
    {
#if (GANTRY)
        if(motorX <= -2)
        {
            WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorX += 2;
        }
        else if(motorX >= 2)
        {
            WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorX -= 2;
        }
        if(motorYorZ <= -2)
        {
            WRITE(Y_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorYorZ += 2;
        }
        else if(motorYorZ >= 2)
        {
            WRITE(Y_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorYorZ -= 2;
        }
#endif
    }
    static INLINE void executeXZGantrySteps()
    {
#if (GANTRY)
        if(motorX <= -2)
        {
            WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorX += 2;
        }
        else if(motorX >= 2)
        {
            WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorX -= 2;
        }
        if(motorYorZ <= -2)
        {
            //ANALYZER_ON(ANALYZER_CH3); // I dont think i can use these as they are for the y - possible bug area though
            WRITE(Z_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorYorZ += 2;
        }
        else if(motorYorZ >= 2)
        {
            //ANALYZER_ON(ANALYZER_CH3); // I dont think i can use these as they are for the y - possible bug area though
            WRITE(Z_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_STEP_PIN,START_STEP_WITH_HIGH);
#endif
            motorYorZ -= 2;
        }
#endif
    }
    static INLINE void startXStep()
    {
        WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void startYStep()
    {
        WRITE(Y_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void startZStep()
    {
        WRITE(Z_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void endXYZSteps()
    {
        WRITE(X_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
        WRITE(Y_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
        WRITE(Z_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
    }
    static INLINE speed_t updateStepsPerTimerCall(speed_t vbase)
    {
        if(vbase > STEP_DOUBLER_FREQUENCY)
        {
#if ALLOW_QUADSTEPPING
            if(vbase > STEP_DOUBLER_FREQUENCY * 2)
            {
                Printer::stepsPerTimerCall = 4;
                return vbase >> 2;
            }
            else
            {
                Printer::stepsPerTimerCall = 2;
                return vbase >> 1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            return vbase >> 1;
#endif
        }
        else
        {
            Printer::stepsPerTimerCall = 1;
        }
        return vbase;
    }
    static INLINE void disableAllowedStepper()
    {
#if DRIVE_SYSTEM == XZ_GANTRY || DRIVE_SYSTEM == ZX_GANTRY
        if(DISABLE_X && DISABLE_Z)
        {
            disableXStepper();
            disableZStepper();
        }
        if(DISABLE_Y) disableYStepper();
#else
#if GANTRY
        if(DISABLE_X && DISABLE_Y)
        {
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
    static INLINE float realXPosition()
    {
        return currentPosition[X_AXIS];
    }

    static INLINE float realYPosition()
    {
        return currentPosition[Y_AXIS];
    }

    static INLINE float realZPosition()
    {
        return currentPosition[Z_AXIS];
    }
    static INLINE void realPosition(float &xp, float &yp, float &zp)
    {
        xp = currentPosition[X_AXIS];
        yp = currentPosition[Y_AXIS];
        zp = currentPosition[Z_AXIS];
    }
    static INLINE void insertStepperHighDelay()
    {
#if STEPPER_HIGH_DELAY>0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
    }
    static void constrainDestinationCoords();
    static void updateDerivedParameter();
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void kill(uint8_t only_steppers);
    static void updateAdvanceFlags();
    static void setup();
    static void defaultLoopActions();
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    static uint8_t moveTo(float x,float y,float z,float e,float f);
    static uint8_t moveToReal(float x,float y,float z,float e,float f);
    static void homeAxis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
    static void setOrigin(float xOff,float yOff,float zOff);
    static bool isPositionAllowed(float x,float y,float z);
    static INLINE int getFanSpeed()
    {
        return (int)pwm_pos[NUM_EXTRUDER + 2];
    }
#if NONLINEAR_SYSTEM
    static INLINE void setDeltaPositions(long xaxis, long yaxis, long zaxis)
    {
        currentDeltaPositionSteps[A_TOWER] = xaxis;
        currentDeltaPositionSteps[B_TOWER] = yaxis;
        currentDeltaPositionSteps[C_TOWER] = zaxis;
    }
    static void deltaMoveToTopEndstops(float feedrate);
#endif
#if MAX_HARDWARE_ENDSTOP_Z
    static float runZMaxProbe();
#endif
#if FEATURE_Z_PROBE
    static float runZProbe(bool first,bool last,uint8_t repeat = Z_PROBE_REPETITIONS,bool runStartScript = true);
    static void waitForZProbeStart();
    static float bendingCorrectionAt(float x,float y);
#endif
    // Moved outside FEATURE_Z_PROBE to allow auto-level functional test on
    // system without Z-probe
#if FEATURE_AUTOLEVEL
    static void transformToPrinter(float x,float y,float z,float &transX,float &transY,float &transZ);
    static void transformFromPrinter(float x,float y,float z,float &transX,float &transY,float &transZ);
    static void resetTransformationMatrix(bool silent);
    static void buildTransformationMatrix(float h1,float h2,float h3);
#endif
#if DISTORTION_CORRECTION
    static void measureDistortion(void);
    static Distortion distortion;
#endif
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed);
    static void zBabystep();

    static INLINE void resetWizardStack()
    {
        wizardStackPos = 0;
    }
    static INLINE void pushWizardVar(wizardVar v)
    {
        wizardStack[wizardStackPos++] = v;
    }
    static INLINE wizardVar popWizardVar()
    {
        return wizardStack[--wizardStackPos];
    }
    static void showConfiguration();
    static void setCaseLight(bool on);
    static void reportCaseLightStatus();
private:
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
};

#endif // PRINTER_H_INCLUDED
