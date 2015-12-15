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


#ifndef MOTION_H
#define MOTION_H

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_SKIP_ACCELERATING 32
#define FLAG_SKIP_DEACCELERATING 64
#define FLAG_BLOCKED 128

#define FLAG_JOIN_STEPPARAMS_COMPUTED	1	// Are the step parameter computed
#define FLAG_JOIN_END_FIXED				2	// The right speed is fixed. Don't check this block or any block to the left.
#define FLAG_JOIN_START_FIXED			4	// The left speed is fixed. Don't check left block.
#define FLAG_JOIN_START_RETRACT			8	// Start filament retraction at move start
#define FLAG_JOIN_END_RETRACT			16	// Wait for filament pushback, before ending move
#define FLAG_JOIN_NO_RETRACT			32	// Disable retract for this line
#define FLAG_JOIN_WAIT_EXTRUDER_UP		64	// Wait for the extruder to finish it's up movement
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN	128	// Wait for the extruder to finish it's down movement


class UIDisplay;
class PrintLine
{
    friend class UIDisplay;

public:
    static uint8_t		linesPos;		// Position for executing line movement
    static PrintLine	lines[];
    static uint8_t		linesWritePos;	// Position where we write the next cached line move
    flag8_t				joinFlags;
    volatile flag8_t	flags;
	volatile uint8_t	started;

private:
    flag8_t				primaryAxis;
    int32_t				timeInTicks;
    flag8_t				halfStep;					///< 4 = disabled, 1 = halfstep, 2 = fulstep
    flag8_t				dir;						///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    int32_t				delta[4];					///< Steps we want to move.
    int32_t				error[4];					///< Error calculation for Bresenham algorithm
    float				speedX;						///< Speed in x direction at fullInterval in mm/s
    float				speedY;						///< Speed in y direction at fullInterval in mm/s
    float				speedZ;						///< Speed in z direction at fullInterval in mm/s
    float				speedE;						///< Speed in E direction at fullInterval in mm/s
    float				fullSpeed;					///< Desired speed mm/s
    float				invFullSpeed;				///< 1.0/fullSpeed for fatser computation
    float				accelerationDistance2;		///< Real 2.0*distanceÜacceleration mm²/s²
    float				maxJunctionSpeed;			///< Max. junction speed between this and next segment
    float				startSpeed;					///< Staring speed in mm/s
    float				endSpeed;					///< Exit speed in mm/s
    float				minSpeed;
    float				distance;
    ticks_t				fullInterval;				///< interval at full speed in ticks/step.
    uint16_t			accelSteps;					///< How much steps does it take, to reach the plateau.
    uint16_t			decelSteps;					///< How much steps does it take, to reach the end speed.
    uint32_t			accelerationPrim;			///< Acceleration along primary axis
    uint32_t			fAcceleration;				///< accelerationPrim*262144/F_CPU
    speed_t				vMax;						///< Maximum reached speed in steps/s.
    speed_t				vStart;						///< Starting speed in steps/s.
    speed_t				vEnd;						///< End speed in steps/s

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    int32_t				advanceRate;               ///< Advance steps at full speed
    int32_t				advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
    int32_t				advanceStart;
    int32_t				advanceEnd;
#endif // ENABLE_QUADRATIC_ADVANCE

    uint16_t			advanceL;					///< Recomputated L value
#endif // USE_ADVANCE

#ifdef DEBUG_STEPCOUNT
    int32_t				totalStepsRemaining;
#endif // DEBUG_STEPCOUNT


public:
    int32_t				stepsRemaining;            ///< Remaining steps, until move is finished
    static PrintLine*	cur;
	char				task;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	static PrintLine	direct;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	
	static volatile uint8_t linesCount; // Number of lines cached 0 = nothing to do

    inline bool areParameterUpToDate()
    {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // areParameterUpToDate

    inline void invalidateParameter()
    {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // invalidateParameter

    inline void setParameterUpToDate()
    {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
    } // setParameterUpToDate

    inline bool isStartSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_START_FIXED;
    } // isStartSpeedFixed

    inline void setStartSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
    } // setStartSpeedFixed

    inline void fixStartAndEndSpeed()
    {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
    } // fixStartAndEndSpeed

    inline bool isEndSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_END_FIXED;
    } // isEndSpeedFixed

    inline void setEndSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
    } // setEndSpeedFixed

    inline bool isWarmUp()
    {
        return flags & FLAG_WARMUP;
    } // isWarmUp

    inline uint8_t getWaitForXLinesFilled()
    {
        return primaryAxis;
    } // getWaitForXLinesFilled

    inline void setWaitForXLinesFilled(uint8_t b)
    {
        primaryAxis = b;
    } // setWaitForXLinesFilled

    inline bool isExtruderForwardMove()
    {
        return (dir & 136)==136;
    } // isExtruderForwardMove

    inline void block()
    {
        flags |= FLAG_BLOCKED;
    } // block

    inline void unblock()
    {
        flags &= ~FLAG_BLOCKED;
    } // unblock

    inline bool isBlocked()
    {
        return flags & FLAG_BLOCKED;
    } // isBlocked

    inline bool isCheckEndstops()
    {
        return flags & FLAG_CHECK_ENDSTOPS;
    } // isCheckEndstops

    inline bool isNominalMove()
    {
        return flags & FLAG_NOMINAL;
    } // isNominalMove

    inline bool setNominalMove()
    {
        flags |= FLAG_NOMINAL;
    } // setNominalMove

    inline void checkEndstops()
    {
        if(isCheckEndstops())
        {
            if(isXNegativeMove() && Printer::isXMinEndstopHit())
                setXMoveFinished();
            if(isYNegativeMove() && Printer::isYMinEndstopHit())
                setYMoveFinished();
            if(isXPositiveMove() && Printer::isXMaxEndstopHit())
                setXMoveFinished();
            if(isYPositiveMove() && Printer::isYMaxEndstopHit())
                setYMoveFinished();
        }

		// Test Z-Axis every step if necessary, otherwise it could easyly ruin your printer!
        if(isZNegativeMove() && Printer::isZMinEndstopHit())
		{
			setZMoveFinished();
		}
        if(isZPositiveMove() && Printer::isZMaxEndstopHit())
        {
#if MAX_HARDWARE_ENDSTOP_Z
            Printer::stepsRemainingAtZHit = stepsRemaining;
#endif // MAX_HARDWARE_ENDSTOP_Z

            setZMoveFinished();
        }
    } // checkEndstops

    inline void setXMoveFinished()
    {
        dir&=~16;
    } // setXMoveFinished

    inline void setYMoveFinished()
    {
        dir&=~32;
    } // setYMoveFinished

    inline void setZMoveFinished()
    {
        dir&=~64;
    } // setZMoveFinished

    inline void setXYMoveFinished()
    {
        dir&=~48;
    } // setXYMoveFinished

    inline bool isXPositiveMove()
    {
        return (dir & 17)==17;
    } // isXPositiveMove

    inline bool isXNegativeMove()
    {
        return (dir & 17)==16;
    } // isXNegativeMove

    inline bool isYPositiveMove()
    {
        return (dir & 34)==34;
    } // isYPositiveMove

    inline bool isYNegativeMove()
    {
        return (dir & 34)==32;
    } // isYNegativeMove

    inline bool isZPositiveMove()
    {
        return (dir & 68)==68;
    } // isZPositiveMove

    inline bool isZNegativeMove()
    {
        return (dir & 68)==64;
    } // isZNegativeMove

    inline bool isEPositiveMove()
    {
        return (dir & 136)==136;
    } // isEPositiveMove

    inline bool isENegativeMove()
    {
        return (dir & 136)==128;
    } // isENegativeMove

    inline bool isXMove()
    {
        return (dir & 16);
    } // isXMove

    inline bool isYMove()
    {
        return (dir & 32);
    } // isYMove

    inline bool isXOrYMove()
    {
        return dir & 48;
    } // isXOrYMove

    inline bool isZMove()
    {
        return (dir & 64);
    } // isZMove

    inline bool isEMove()
    {
        return (dir & 128);
    } // isEMove

    inline bool isEOnlyMove()
    {
        return (dir & 240)==128;
    } // isEOnlyMove

    inline bool isNoMove()
    {
        return (dir & 240)==0;
    } // isNoMove

    inline bool isXYZMove()
    {
        return dir & 112;
    } // isXYZMove

    inline bool isMoveOfAxis(uint8_t axis)
    {
        return (dir & (16<<axis));
    } // isMoveOfAxis

    inline void setMoveOfAxis(uint8_t axis)
    {
        dir |= 16<<axis;
    } // setMoveOfAxis

    inline void setPositiveDirectionForAxis(uint8_t axis)
    {
        dir |= 1<<axis;
    } // setPositiveDirectionForAxis

    inline static void resetPathPlanner()
    {
        linesCount	  = 0;
        linesPos	  = 0;
		linesWritePos = 0;
    } // resetPathPlanner

	inline static void resetLineBuffer()
	{
		cur = NULL;
		memset( lines, 0, sizeof( PrintLine ) * MOVE_CACHE_SIZE );
	} // resetLineBuffer

    inline void updateAdvanceSteps(speed_t v,uint8_t max_loops,bool accelerate)
    {
#ifdef USE_ADVANCE
        if(!Printer::isAdvanceActivated()) return;
#ifdef ENABLE_QUADRATIC_ADVANCE
        long advanceTarget = Printer::advanceExecuted;
        if(accelerate)
        {
            for(uint8_t loop = 0; loop<max_loops; loop++) advanceTarget += advanceRate;
            if(advanceTarget>advanceFull)
                advanceTarget = advanceFull;
        }
        else
        {
            for(uint8_t loop = 0; loop<max_loops; loop++) advanceTarget -= advanceRate;
            if(advanceTarget<advanceEnd)
                advanceTarget = advanceEnd;
        }

        long h = HAL::mulu16xu16to32(v,advanceL);
        int tred = ((advanceTarget + h) >> 16);

        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred-Printer::advanceStepsSet;
        if(tred>0 && Printer::advanceStepsSet<=0)
            Printer::extruderStepsNeeded += Extruder::current->advanceBacklash;
        else if(tred<0 && Printer::advanceStepsSet>=0)
            Printer::extruderStepsNeeded -= Extruder::current->advanceBacklash;

        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
        Printer::advanceExecuted = advanceTarget;
#else
        int tred = HAL::mulu6xu16shift16(v,advanceL);
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;

        if(tred>0 && Printer::advanceStepsSet<=0)
            Printer::extruderStepsNeeded += (Extruder::current->advanceBacklash << 1);
        else if(tred<0 && Printer::advanceStepsSet>=0)
            Printer::extruderStepsNeeded -= (Extruder::current->advanceBacklash << 1);

        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // USE_ADVANCE
    } // updateAdvanceSteps

    inline bool moveDecelerating()
    {
        if(stepsRemaining <= decelSteps)
        {
            if (!(flags & FLAG_DECELERATING))
            {
                Printer::timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        }
        else return false;
    } // moveDecelerating

    inline bool moveAccelerating()
    {
        return Printer::stepNumber <= accelSteps;
    } // moveAccelerating

    inline bool isFullstepping()
    {
        return halfStep == 4;
    } // isFullstepping

    inline void startXStep()
    {
        ANALYZER_ON(ANALYZER_CH6);
        ANALYZER_ON(ANALYZER_CH2);
        WRITE(X_STEP_PIN,HIGH);

#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,HIGH);
#endif // FEATURE_TWO_XSTEPPER

#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif // DEBUG_STEPCOUNT

    } // startXStep

    inline void startYStep()
    {
        ANALYZER_ON(ANALYZER_CH7);
        ANALYZER_ON(ANALYZER_CH3);
        WRITE(Y_STEP_PIN,HIGH);

#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,HIGH);
#endif // FEATURE_TWO_YSTEPPER

#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif // DEBUG_STEPCOUNT

    } // startYStep

    void updateStepsParameter();
    inline float safeSpeed();
    void calculateQueueMove(float axis_diff[],uint8_t pathOptimize);

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	void calculateDirectMove(float axis_diff[],uint8_t pathOptimize);
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

	void logLine();
    void logLine2();

    inline long getWaitTicks()
    {
        return timeInTicks;
    } // getWaitTicks

    inline void setWaitTicks(long wait)
    {
        timeInTicks = wait;
    } // setWaitTicks

    static inline bool hasLines()
    {
        return linesCount;
    } // hasLines

    static inline void setCurrentLine()
    {
        cur = &lines[linesPos];
    } // setCurrentLine

    static inline void removeCurrentLineForbidInterrupt()
    {
        linesPos++;
        if(linesPos>=MOVE_CACHE_SIZE) linesPos=0;
        cur = NULL;

        HAL::forbidInterrupts();
        --linesCount;
        if(!linesCount)
            Printer::setMenuMode(MENU_MODE_PRINTING,false);
    } // removeCurrentLineForbidInterrupt

    static inline void pushLine()
    {
        linesWritePos++;

        if(linesWritePos>=MOVE_CACHE_SIZE) linesWritePos = 0;
        Printer::setMenuMode(MENU_MODE_PRINTING,true);
        
		BEGIN_INTERRUPT_PROTECTED
        linesCount++;
        END_INTERRUPT_PROTECTED
    } // pushLine

    static PrintLine *getNextWriteLine()
    {
        return &lines[linesWritePos];
    } // getNextWriteLine

    static inline void computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current);
    static long performQueueMove();

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	static long performDirectMove( void );
	static void performDirectSteps( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

	static long performMove(PrintLine* move, char forQueue);
    static void waitForXFreeLines(uint8_t b=1);
	static bool checkForXFreeLines(uint8_t freeLines=1);
    static inline void forwardPlanner(uint8_t p);
    static inline void backwardPlanner(uint8_t p,uint8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
    static void prepareQueueMove(uint8_t check_endstops,uint8_t pathOptimize);

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	static void prepareDirectMove( void );
	static void stopDirectMove( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    static void moveRelativeDistanceInSteps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop);
    static void moveRelativeDistanceInStepsReal(long x,long y,long z,long e,float feedrate,bool waitEnd);

#if FEATURE_ARC_SUPPORT
    static void arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif // FEATURE_ARC_SUPPORT

    static inline void previousPlannerIndex(uint8_t &p)
    {
        p = (p ? p-1 : MOVE_CACHE_SIZE-1);
    } // previousPlannerIndex

    static inline void nextPlannerIndex(uint8_t& p)
    {
        p = (p == MOVE_CACHE_SIZE - 1 ? 0 : p + 1);
    } // nextPlannerIndex

	static inline void queueTask( char task )
	{
		PrintLine*	p;


		p = getNextWriteLine();
		p->task = task;
  
		nextPlannerIndex( linesWritePos );
		BEGIN_INTERRUPT_PROTECTED
		linesCount++;
		END_INTERRUPT_PROTECTED
		return;
	} // queueTask

	inline void enableSteppers( void )
	{
	    // Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
		if(isXMove())
		{
			Printer::enableXStepper();
			Printer::setXDirection(isXPositiveMove());
		}
        if(isYMove())
		{
			Printer::enableYStepper();
			Printer::setYDirection(isYPositiveMove());
		}
        if(isZMove() && !Printer::blockZ)
        {
            Printer::enableZStepper();
			Printer::unsetAllSteppersDisabled();
			Printer::setZDirection(isZPositiveMove());
        }
		if(isEMove())
		{
			Extruder::enable();
			Extruder::setDirection(isEPositiveMove());
		}
		started = 1;

	} // enableSteppers

};


inline void prepareBedUp( void )
{
	WRITE( Z_DIR_PIN, INVERT_Z_DIR );

#if FEATURE_TWO_ZSTEPPER
	WRITE( Z2_DIR_PIN, INVERT_Z_DIR );
#endif // FEATURE_TWO_ZSTEPPER

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	Printer::decreaseLastZDirection();
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

} // prepareBedUp


inline void prepareBedDown( void )
{
	WRITE( Z_DIR_PIN, !INVERT_Z_DIR );

#if FEATURE_TWO_ZSTEPPER
	WRITE( Z2_DIR_PIN, !INVERT_Z_DIR );
#endif // FEATURE_TWO_ZSTEPPER

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	Printer::increaseLastZDirection();
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

} // prepareBedDown


inline void startZStep( char nDirection )
{
    WRITE( Z_STEP_PIN,HIGH );

#if FEATURE_TWO_ZSTEPPER
    WRITE( Z2_STEP_PIN,HIGH );
#endif // FEATURE_TWO_ZSTEPPER

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	if( Printer::endstopZMinHit )	Printer::stepsSinceZMinEndstop += nDirection;
	if( Printer::endstopZMaxHit )	Printer::stepsSinceZMaxEndstop += nDirection;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

} // startZStep


inline void endZStep( void )
{
    WRITE( Z_STEP_PIN, LOW );

#if FEATURE_TWO_ZSTEPPER
    WRITE( Z2_STEP_PIN, LOW );
#endif // FEATURE_TWO_ZSTEPPER

} // endZStep


#endif // MOTION_H
