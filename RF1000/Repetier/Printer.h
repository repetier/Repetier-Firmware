/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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


#ifndef PRINTER_H
#define PRINTER_H


// ##########################################################################################
// ##	status flags
// ##########################################################################################

#define PRINTER_FLAG0_STEPPER_DISABLED			1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT		2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT			4
#define PRINTER_FLAG0_FORCE_CHECKSUM			8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE			16
#define PRINTER_FLAG0_LARGE_MACHINE				128
#define PRINTER_FLAG1_HOMED						1
#define PRINTER_FLAG1_AUTOMOUNT					2
#define PRINTER_FLAG1_ANIMATION					4
#define PRINTER_FLAG1_ALLKILLED					8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE			16
#define PRINTER_FLAG1_NO_DESTINATION_CHECK		32
#define PRINTER_FLAG1_Z_ORIGIN_SET				64


class Printer
{
public:
#if defined(USE_ADVANCE)
    static volatile int		extruderStepsNeeded;				// This many extruder steps are still needed, <0 = reverse steps needed.
    static uint8_t			minExtruderSpeed;					// Timer delay for start extruder speed
    static uint8_t			maxExtruderSpeed;					// Timer delay for end extruder speed
    static int				advanceStepsSet;

#ifdef ENABLE_QUADRATIC_ADVANCE
    static long				advanceExecuted;					// Executed advance steps
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // defined(USE_ADVANCE)

    static uint8_t			menuMode;
    static float			axisStepsPerMM[];
    static float			invAxisStepsPerMM[];
    static float			maxFeedrate[];
    static float			homingFeedrate[];
    static float			maxAccelerationMMPerSquareSecond[];
    static float			maxTravelAccelerationMMPerSquareSecond[];
    static unsigned long	maxPrintAccelerationStepsPerSquareSecond[];
    static unsigned long	maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t			relativeCoordinateMode;				// Determines absolute (false) or relative Coordinates (true).
    static uint8_t			relativeExtruderCoordinateMode;		// Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
    static uint8_t			unitIsInches;
    static uint8_t			debugLevel;
    static uint8_t			flag0;
	static uint8_t			flag1;
    static uint8_t			stepsPerTimerCall;
    static unsigned long	interval;							// Last step duration in ticks.
    static unsigned long	timer;								// used for acceleration/deceleration timing
    static unsigned long	stepNumber;							// Step number in current move.
    static float			originOffsetMM[3];
    static long				queuePositionTargetSteps[4];		// Target position in steps.
    static long				queuePositionLastSteps[4];			// Position in steps from origin.
    static float			queuePositionLastMM[3];				// Position in mm from origin.
    static float			queuePositionCommandMM[3];			// Last coordinates send by gcodes

#if MAX_HARDWARE_ENDSTOP_Z
    static long				stepsRemainingAtZHit;
#endif // MAX_HARDWARE_ENDSTOP_Z

    static float			minimumSpeed;						// lowest allowed speed to keep integration error small
    static float			minimumZSpeed;						// lowest allowed speed to keep integration error small
    static long				maxSteps[3];						// For software endstops, limit of move in positive direction.
    static long				minSteps[3];						// For software endstops, limit of move in negative direction.
    static float			lengthMM[3];
    static float			minMM[3];
    static float			feedrate;							// Last requested feedrate.
    static int				feedrateMultiply;					// Multiplier for feedrate in percent (factor 1 = 100)
    static unsigned int		extrudeMultiply;					// Flow multiplier in percdent (factor 1 = 100)
    static float			maxJerk;							// Maximum allowed jerk in mm/s
    static float			maxZJerk;							// Maximum allowed jerk in z direction in mm/s
    static float			extruderOffset[2];					// offset for different extruder positions.
    static speed_t			vMaxReached;						// Maximumu reached speed
    static unsigned long	msecondsPrinting;					// Milliseconds of printing time (means time with heated extruder)
	static unsigned long	msecondsMilling;					// Milliseconds of milling time
    static float			filamentPrinted;					// mm of filament printed since counting started
    static uint8_t			wasLastHalfstepping;				// Indicates if last move had halfstepping enabled
	static long				ZOffset;							// Z Offset in um

#if ENABLE_BACKLASH_COMPENSATION
    static float			backlash[3];
    static uint8_t			backlashDir;
#endif // ENABLE_BACKLASH_COMPENSATION

#ifdef DEBUG_STEPCOUNT
    static long				totalStepsRemaining;
#endif // DEBUG_STEPCOUNT

#if FEATURE_MEMORY_POSITION
    static float			memoryX;
    static float			memoryY;
    static float			memoryZ;
    static float			memoryE;
#endif // FEATURE_MEMORY_POSITION

#ifdef DEBUG_SEGMENT_LENGTH
    static float			maxRealSegmentLength;
#endif // DEBUG_SEGMENT_LENGTH

#ifdef DEBUG_REAL_JERK
    static float			maxRealJerk;
#endif // DEBUG_REAL_JERK

#if FEATURE_HEAT_BED_Z_COMPENSATION
    static char				doHeatBedZCompensation;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    static char				doWorkPartZCompensation;
	static long				staticCompensationZ;				// this is the z-delta which can occur in case the x/y position of the z-origin from the work part scan is different to the x/y position of the z-origin from the moment of the start of the milling
#endif // FEATURE_WORK_PART_Z_COMPENSATION

    static long				queuePositionCurrentSteps[3];
	static char				stepperDirection[3];				// this is the current x/y/z-direction from the processing of G-Codes
	static char				blockZ;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    static long				compensatedPositionTargetStepsZ;
    static long				compensatedPositionCurrentStepsZ;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static long				directPositionTargetSteps[4];
    static long				directPositionCurrentSteps[4];
	static long				directPositionLastSteps[4];
	static char				waitMove;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_MILLING_MODE
	static char				operatingMode;
	static float			drillFeedrate;
	static float			drillZDepth;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	static char				ZEndstopType;
	static char				ZEndstopUnknown;
	static char				lastZDirection;
	static char				endstopZMinHit;
	static char				endstopZMaxHit;
	static long				stepsSinceZMinEndstop;
	static long				stepsSinceZMaxEndstop;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
	static char				HotendType;
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

#if FEATURE_CONFIGURABLE_MILLER_TYPE
	static char				MillerType;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if STEPPER_ON_DELAY
	static char				enabledStepper[3];
#endif // STEPPER_ON_DELAY

#if FEATURE_BEEPER
	static char				enableBeeper;
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
	static char				enableCaseLight;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
	static char				RGBLightMode;
	static char				RGBLightStatus;  
	static	unsigned long	RGBLightIdleStart;
	static	unsigned long	RGBLightLastChange;
	static char				RGBButtonBackPressed;
	static char				RGBLightModeForceWhite;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_230V_OUTPUT
	static char				enable230VOutput;
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
	static char				enableFET1;
	static char				enableFET2;
	static char				enableFET3;
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
	static unsigned long	prepareFanOff;
	static unsigned long	fanOffDelay;
#endif // FEATURE_CASE_FAN

#if FEATURE_TYPE_EEPROM
	static unsigned char	wrongType;
#endif // FEATURE_TYPE_EEPROM


	static inline void setMenuMode(uint8_t mode,bool on)
	{
        if(on)
            menuMode |= mode;
        else
            menuMode &= ~mode;
    } // setMenuMode

    static inline bool isMenuMode(uint8_t mode)
	{
        return (menuMode & mode)==mode;
    }// isMenuMode

    static inline bool debugEcho()
    {
        return ((debugLevel & 1)!=0);
    } // debugEcho

    static inline bool debugInfo()
    {
        return ((debugLevel & 2)!=0);
    } // debugInfo

    static inline bool debugErrors()
    {
        return ((debugLevel & 4)!=0);
    } // debugErrors

    static inline bool debugDryrun()
    {
        return ((debugLevel & 8)!=0);
    } // debugDryrun

    static inline bool debugCommunication()
    {
        return ((debugLevel & 16)!=0);
    } // debugCommunication

    static inline bool debugNoMoves()
	{
        return ((debugLevel & 32)!=0);
    }// debugNoMoves

    /** \brief Disable stepper motor for x direction. */
    static inline void disableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN,!X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		Printer::enabledStepper[X_AXIS] = 0;
#endif // STEPPER_ON_DELAY

		// when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
		setHomed(false);
		cleanupXPositions();

	} // disableXStepper

    /** \brief Disable stepper motor for y direction. */
    static inline void disableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN,!Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		Printer::enabledStepper[Y_AXIS] = 0;
#endif // STEPPER_ON_DELAY

		// when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
		setHomed(false);
		cleanupYPositions();

	} // disableYStepper

    /** \brief Disable stepper motor for z direction. */
    static inline void disableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		Printer::enabledStepper[Z_AXIS] = 0;
#endif // STEPPER_ON_DELAY

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
		Printer::lastZDirection		   = 0;
		Printer::endstopZMinHit		   = ENDSTOP_NOT_HIT;
		Printer::endstopZMaxHit		   = ENDSTOP_NOT_HIT;
		Printer::stepsSinceZMinEndstop = 0;
		Printer::stepsSinceZMaxEndstop = 0;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

		// when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
		setHomed(false);
		setZOriginSet(false);
		cleanupZPositions();

	} // disableZStepper

    /** \brief Enable stepper motor for x direction. */
    static inline void enableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN,X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		if( !Printer::enabledStepper[X_AXIS] )
		{
			Printer::enabledStepper[X_AXIS] = 1;
			HAL::delayMilliseconds( STEPPER_ON_DELAY );
		}
#endif // STEPPER_ON_DELAY

    } // enableXStepper

    /** \brief Enable stepper motor for y direction. */
    static inline void enableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN,Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		if( !Printer::enabledStepper[Y_AXIS] )
		{
			Printer::enabledStepper[Y_AXIS] = 1;
			HAL::delayMilliseconds( STEPPER_ON_DELAY );
		}
#endif // STEPPER_ON_DELAY
    } // enableYStepper

    /** \brief Enable stepper motor for z direction. */
    static inline void enableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN,Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
		if( !Printer::enabledStepper[Z_AXIS] )
		{
			Printer::enabledStepper[Z_AXIS] = 1;
			HAL::delayMilliseconds( STEPPER_ON_DELAY );
		}
#endif // STEPPER_ON_DELAY

    } // enableZStepper

    static inline void setXDirection(bool positive)
    {
        if(positive)
        {
            // extruder moves to the right
            if( stepperDirection[X_AXIS] != 1 )
            {
                WRITE(X_DIR_PIN,!INVERT_X_DIR);

#if FEATURE_TWO_XSTEPPER
                WRITE(X2_DIR_PIN,!INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER

                stepperDirection[X_AXIS] = 1;
            }
        }
        else
        {
            // extruder moves to the left
            if( stepperDirection[X_AXIS] != -1 )
            {
                WRITE(X_DIR_PIN,INVERT_X_DIR);

#if FEATURE_TWO_XSTEPPER
                WRITE(X2_DIR_PIN,INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER

                stepperDirection[X_AXIS] = -1;
            }
        }
    } // setXDirection

    static inline void setYDirection(bool positive)
    {
        if(positive)
        {
            // heat bed moves to the front
            if( stepperDirection[Y_AXIS] != 1 )
            {
                WRITE(Y_DIR_PIN,!INVERT_Y_DIR);

#if FEATURE_TWO_YSTEPPER
                WRITE(Y2_DIR_PIN,!INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER

                stepperDirection[Y_AXIS] = 1;
            }
        }
        else
        {
            // heat bed moves to the back
            if( stepperDirection[Y_AXIS] != -1 )
            {
                WRITE(Y_DIR_PIN,INVERT_Y_DIR);

#if FEATURE_TWO_YSTEPPER
                WRITE(Y2_DIR_PIN,INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER

                stepperDirection[Y_AXIS] = -1;
            }
        }
    } // setYDirection

    static inline void setZDirection(bool positive)
    {
		if( blockZ )
		{
			return;
		}

        if(positive)
        {
            // heat bed moves to the bottom
			WRITE( Z_DIR_PIN, !INVERT_Z_DIR );

#if FEATURE_TWO_ZSTEPPER
			WRITE( Z2_DIR_PIN, !INVERT_Z_DIR );
#endif // FEATURE_TWO_YSTEPPER

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
			increaseLastZDirection();
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

			stepperDirection[Z_AXIS] = 1;
        }
        else
        {
            // heat bed moves to the top
			WRITE( Z_DIR_PIN, INVERT_Z_DIR );

#if FEATURE_TWO_ZSTEPPER
			WRITE( Z2_DIR_PIN, INVERT_Z_DIR );
#endif // FEATURE_TWO_YSTEPPER

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
			Printer::decreaseLastZDirection();
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

			stepperDirection[Z_AXIS] = -1;
        }
    } // setZDirection

    static inline bool getZDirection()
    {
        return ((READ(Z_DIR_PIN)!=0) ^ INVERT_Z_DIR);
    } // getZDirection


#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	static inline void increaseLastZDirection()
	{
		lastZDirection = 1;
	} // increaseLastZDirection

	static inline void decreaseLastZDirection()
	{
		lastZDirection = -1;
	} // decreaseLastZDirection
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

    static inline bool getYDirection()
    {
        return((READ(Y_DIR_PIN)!=0) ^ INVERT_Y_DIR);
    } // getYDirection

    static inline bool getXDirection()
    {
        return((READ(X_DIR_PIN)!=0) ^ INVERT_X_DIR);
    } // getXDirection

    static inline uint8_t isLargeMachine()
    {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    } // isLargeMachine

    static inline void setLargeMachine(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    } // setLargeMachine

    static inline uint8_t isAdvanceActivated()
    {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    } // isAdvanceActivated

    static inline void setAdvanceActivated(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    } // setAdvanceActivated

    static inline uint8_t isHomed()
    {
        return flag1 & PRINTER_FLAG1_HOMED;
    } // isHomed

    static inline void setHomed(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED : flag1 & ~PRINTER_FLAG1_HOMED);
    } // setHomed

    static inline uint8_t isZOriginSet()
    {
        return flag1 & PRINTER_FLAG1_Z_ORIGIN_SET;
    } // isZOriginSet

    static inline void setZOriginSet(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_Z_ORIGIN_SET : flag1 & ~PRINTER_FLAG1_Z_ORIGIN_SET);
    } // setZOriginSet

    static inline uint8_t isAllKilled()
    {
        return flag1 & PRINTER_FLAG1_ALLKILLED;
    } // isAllKilled

    static inline void setAllKilled(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
    } // setAllKilled

    static inline uint8_t isAutomount()
    {
        return flag1 & PRINTER_FLAG1_AUTOMOUNT;
    } // isAutomount

    static inline void setAutomount(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_AUTOMOUNT : flag1 & ~PRINTER_FLAG1_AUTOMOUNT);
    } // setAutomount

    static inline uint8_t isAnimation()
    {
        return flag1 & PRINTER_FLAG1_ANIMATION;
    } // isAnimation

    static inline void setAnimation(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ANIMATION : flag1 & ~PRINTER_FLAG1_ANIMATION);
    } // setAnimation

    static inline uint8_t isUIErrorMessage()
    {
        return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
    } // isUIErrorMessage

    static inline void setUIErrorMessage(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
    } // setUIErrorMessage

    static inline uint8_t isNoDestinationCheck()
    {
        return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
    } // isNoDestinationCheck

    static inline void setNoDestinationCheck(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
    } // setNoDestinationCheck

    static inline void toggleAnimation()
	{
        setAnimation(!isAnimation());
    } // toggleAnimation

    static inline float convertToMM(float x)
    {
        return (unitIsInches ? x*25.4 : x);
    } // convertToMM

    static inline bool isXMinEndstopHit()
    {
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
        return READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING;
#else
        return false;
#endif // X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
    } // isXMinEndstopHit

    static inline bool isYMinEndstopHit()
    {
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
        return READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING;
#else
        return false;
#endif // Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
    } // isYMinEndstopHit

    static inline bool isZMinEndstopHit()
    {
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS

		if( ZEndstopType == ENDSTOP_TYPE_SINGLE )
		{
			if( operatingMode == OPERATING_MODE_PRINT )
			{
				// in case there is only one z-endstop and we are in operating mode "print", the z-min endstop must be connected
				return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
			}

			// in case there is only one z-endstop and we are in operating mode "mill", the z-min endstop is not connected and can not be detected
			return false;
		}

		// we end up here in case the z-min and z-max endstops are connected in a circuit
		if( READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING )
		{
			// either the min or the max endstop is hit
			if( ZEndstopUnknown )
			{
				// this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
				return false;
			}

			if( endstopZMaxHit == ENDSTOP_IS_HIT )
			{
				// when the z-max endstop is hit already we know that the z-min endstop is not hit
				return false;
			}

			if( endstopZMinHit == ENDSTOP_IS_HIT )
			{
				// we remember that the z-min endstop is hit at the moment
				return true;
			}

			if( stepsSinceZMaxEndstop && stepsSinceZMaxEndstop > MINIMAL_Z_ENDSTOP_MAX_TO_MIN_STEPS )
			{
				// the z-max endstop was hit a few steps ago, so the z-min endstop can not be hit right now
				return false;
			}
				
			if( lastZDirection > 0 )
			{
				// z-min was not hit and we are moving downwards, so z-min can not become hit right now
				return false;
			}

			// the last z-direction is unknown or the heat bed has been moved upwards, thus we have to assume that the z-min endstop is hit
			endstopZMinHit		  = ENDSTOP_IS_HIT;
			endstopZMaxHit		  = ENDSTOP_NOT_HIT;
			stepsSinceZMinEndstop = Z_ENDSTOP_MIN_TO_MAX_INITIAL_STEPS;
			stepsSinceZMaxEndstop = 0;
			return true;
		}

		// no z endstop is hit
		if( endstopZMinHit == ENDSTOP_IS_HIT )
		{
			endstopZMinHit = ENDSTOP_WAS_HIT;
		}
		if( endstopZMaxHit == ENDSTOP_IS_HIT )
		{
			endstopZMaxHit = ENDSTOP_WAS_HIT;
		}
		ZEndstopUnknown = 0;
		return false;

#else
        return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
        return false;
#endif // Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
    } // isZMinEndstopHit

    static inline bool isXMaxEndstopHit()
    {
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
        return READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING;
#else
        return false;
#endif // X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
    } // isXMaxEndstopHit

    static inline bool isYMaxEndstopHit()
    {
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
        return READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING;
#else
        return false;
#endif // Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
    } // isYMaxEndstopHit

    static inline bool isZMaxEndstopHit()
    {
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS

		if( ZEndstopType == ENDSTOP_TYPE_SINGLE )
		{
			if( operatingMode == OPERATING_MODE_MILL )
			{
				// in case there is only one z-endstop and we are in operating mode "mill", the z-max endstop must be connected
				return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
			}

			// in case there is only one z-endstop and we are in operating mode "print", the z-max endstop is not connected and can not be detected
			return false;
		}

		// we end up here in case the z-min and z-max endstops are connected in a circuit
		if( READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING )
		{
			// either the min or the max endstop is hit
			if( ZEndstopUnknown )
			{
				// this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
				return false;
			}

			if( endstopZMinHit == ENDSTOP_IS_HIT )
			{
				// when the z-min endstop is hit already we know that the z-max endstop is not hit
				return false;
			}

			if( endstopZMaxHit == ENDSTOP_IS_HIT )
			{
				// we remember that the z-max endstop is hit at the moment
				return true;
			}
				
			if( stepsSinceZMinEndstop && stepsSinceZMinEndstop < MINIMAL_Z_ENDSTOP_MIN_TO_MAX_STEPS )
			{
				// the z-min endstop was hit a few steps ago, so the z-max endstop can not be hit right now
				return false;
			}
				
			if( lastZDirection < 0 )
			{
				// z-max was not hit and we are moving upwards, so z-max can not become hit right now
				return false;
			}

//			g_debugInt32 = stepsSinceZMinEndstop;

			// the last z-direction is unknown or the heat bed has been moved downwards, thus we have to assume that the z-max endstop is hit
			endstopZMinHit		  = ENDSTOP_NOT_HIT;
			endstopZMaxHit		  = ENDSTOP_IS_HIT;
			stepsSinceZMinEndstop = 0;
			stepsSinceZMaxEndstop = Z_ENDSTOP_MAX_TO_MIN_INITIAL_STEPS;
			return true;
		}

		// no z endstop is hit
		if( endstopZMinHit == ENDSTOP_IS_HIT )
		{
			endstopZMinHit = ENDSTOP_WAS_HIT;
		}
		if( endstopZMaxHit == ENDSTOP_IS_HIT )
		{
			endstopZMaxHit = ENDSTOP_WAS_HIT;
		}
		ZEndstopUnknown = 0;
		return false;

#else
		return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
        return false;
#endif // Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
    } // isZMaxEndstopHit

    static inline bool areAllSteppersDisabled()
    {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    } // areAllSteppersDisabled

    static inline void setAllSteppersDisabled()
    {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;

		// when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
		setHomed(false);
		setZOriginSet(false);
    } // setAllSteppersDisabled

    static inline void unsetAllSteppersDisabled()
    {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;

#if FAN_BOARD_PIN>-1
        pwm_pos[NUM_EXTRUDER+1] = 255;
#endif // FAN_BOARD_PIN
    } // unsetAllSteppersDisabled

    static inline bool isAnyTempsensorDefect()
    {
#if FEATURE_MILLING_MODE
		if( Printer::operatingMode != OPERATING_MODE_PRINT )
		{
			// we do not support temperature sensors in case we are not in operating mode print
			return 0;
		}
#endif // FEATURE_MILLING_MODE

		return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    } // isAnyTempsensorDefect

    static inline bool isManualMoveMode()
    {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // isManualMoveMode

    static inline void setManualMoveMode(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // setManualMoveMode

    static inline void endXYZSteps()
    {
        WRITE(X_STEP_PIN,LOW);
        WRITE(Y_STEP_PIN,LOW);
        WRITE(Z_STEP_PIN,LOW);

#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,LOW);
#endif // FEATURE_TWO_XSTEPPER

#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,LOW);
#endif // FEATURE_TWO_YSTEPPER

#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,LOW);
#endif // FEATURE_TWO_ZSTEPPER

        ANALYZER_OFF(ANALYZER_CH1);
        ANALYZER_OFF(ANALYZER_CH2);
        ANALYZER_OFF(ANALYZER_CH3);
        ANALYZER_OFF(ANALYZER_CH6);
        ANALYZER_OFF(ANALYZER_CH7);

    } // endXYZSteps

    static inline speed_t updateStepsPerTimerCall(speed_t vbase)
    {
        if(vbase>STEP_DOUBLER_FREQUENCY)
        {
#if ALLOW_QUADSTEPPING
            if(vbase>STEP_DOUBLER_FREQUENCY*2)
            {
                Printer::stepsPerTimerCall = 4;
                return vbase>>2;
            }
            else
            {
                Printer::stepsPerTimerCall = 2;
                return vbase>>1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            return vbase>>1;
#endif // ALLOW_QUADSTEPPING
        }
        else
        {
            Printer::stepsPerTimerCall = 1;
        }
        return vbase;

    } // updateStepsPerTimerCall

    static inline void disableAllowedStepper()
    {
        if(DISABLE_X) disableXStepper();
        if(DISABLE_Y) disableYStepper();
        if(DISABLE_Z) disableZStepper();
    } // disableAllowedStepper

    static inline float lastCalculatedXPosition()
    {
		// return all values in [mm]
        return queuePositionLastMM[X_AXIS];
    } // lastCalculatedXPosition

    static inline float lastCalculatedYPosition()
    {
		// return all values in [mm]
        return queuePositionLastMM[Y_AXIS];
    } // lastCalculatedYPosition

    static inline float lastCalculatedZPosition()
    {
		// return all values in [mm]
        return queuePositionLastMM[Z_AXIS];
    } // lastCalculatedZPosition

	static inline void lastCalculatedPosition(float &xp,float &yp,float &zp)
    {
		// return all values in [mm]
        xp = lastCalculatedXPosition();
        yp = lastCalculatedYPosition();
        zp = lastCalculatedZPosition();
    } // lastCalculatedPosition

    static inline float targetXPosition()
    {
		// return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionTargetSteps[X_AXIS] + (float)directPositionTargetSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#else
        return (float)queuePositionTargetSteps[X_AXIS] * invAxisStepsPerMM[X_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // targetXPosition

	static inline float targetYPosition()
    {
		// return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionTargetSteps[Y_AXIS] + (float)directPositionTargetSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
#else
        return (float)queuePositionTargetSteps[Y_AXIS] * invAxisStepsPerMM[Y_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // targetYPosition

    static inline float targetZPosition()
    {
		// return all values in [mm]
		float	fvalue = (float)queuePositionTargetSteps[Z_AXIS];


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
		// add the current z-compensation
		fvalue += (float)Printer::compensatedPositionCurrentStepsZ;
		fvalue += (float)g_nZScanZPosition;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
		fvalue += (float)g_nZOriginPosition[Z_AXIS];
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
		// add the current manual z-steps
		fvalue += (float)Printer::directPositionTargetSteps[Z_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

		fvalue *= Printer::invAxisStepsPerMM[Z_AXIS];
		return fvalue;

    } // targetZPosition

    static inline void targetPosition(float &xp,float &yp,float &zp)
    {
		// return all values in [mm]
        xp = targetXPosition();
        yp = targetYPosition();
        zp = targetZPosition();

    } // targetPosition
	
    static inline float currentXPosition()
    {
		// return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionCurrentSteps[X_AXIS] + (float)directPositionCurrentSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#else
        return (float)queuePositionCurrentSteps[X_AXIS] * invAxisStepsPerMM[X_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // currentXPosition

    static inline float currentYPosition()
    {
		// return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionCurrentSteps[Y_AXIS] + (float)directPositionCurrentSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
#else
        return (float)queuePositionCurrentSteps[Y_AXIS] * invAxisStepsPerMM[Y_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // currentYPosition

    static inline float currentZPosition()
    {
		// return all values in [mm]
		float	fvalue = (float)queuePositionCurrentSteps[Z_AXIS];


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
		// add the current z-compensation
		fvalue += (float)Printer::compensatedPositionCurrentStepsZ;
		fvalue += (float)g_nZScanZPosition;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
		fvalue += (float)g_nZOriginPosition[Z_AXIS];
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
		// add the current manual z-steps
		fvalue += (float)Printer::directPositionCurrentSteps[Z_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

		fvalue *= Printer::invAxisStepsPerMM[Z_AXIS];
		return fvalue;

    } // currentZPosition

    static inline void currentPosition(float &xp,float &yp,float &zp)
    {
		// return all values in [mm]
        xp = currentXPosition();
        yp = currentYPosition();
        zp = currentZPosition();

    } // currentPosition

	static inline void insertStepperHighDelay()
	{
#if STEPPER_HIGH_DELAY>0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // #if STEPPER_HIGH_DELAY>0

    } // insertStepperHighDelay

    static void constrainQueueDestinationCoords();
    static void constrainDirectDestinationCoords();
    static void updateDerivedParameter();
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void kill(uint8_t only_steppers);
    static void updateAdvanceFlags();
    static void setup();
    static void defaultLoopActions();
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    static void moveTo(float x,float y,float z,float e,float f);
    static void moveToReal(float x,float y,float z,float e,float f);
    static void homeAxis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
    static uint8_t setOrigin(float xOff,float yOff,float zOff);
    static bool isPositionAllowed(float x,float y,float z);

	static inline int getFanSpeed()
	{
        return (int)pwm_pos[NUM_EXTRUDER+2];
    } // getFanSpeed

#if FEATURE_MEMORY_POSITION
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed);
#endif // FEATURE_MEMORY_POSITION

	static bool allowQueueMove( void );

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	static bool allowDirectMove( void );
	static bool allowDirectSteps( void );
	static bool processAsDirectSteps( void );
	static void resetDirectPosition( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
	static void performZCompensation( void );
	static void resetCompensatedPosition( void );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

private:
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();

};

#endif // PRINTER_H
