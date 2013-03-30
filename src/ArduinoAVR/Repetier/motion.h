#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_SKIP_ACCELERATING 32
#define FLAG_SKIP_DEACCELERATING 64
#define FLAG_BLOCKED 128

/** Are the step parameter computed */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1
/** The right speed is fixed. Don't check this block or any block to the left. */
#define FLAG_JOIN_END_FIXED 2
/** The left speed is fixed. Don't check left block. */
#define FLAG_JOIN_START_FIXED 4
/** Start filament retraction at move start */
#define FLAG_JOIN_START_RETRACT 8
/** Wait for filament pushback, before ending move */
#define FLAG_JOIN_END_RETRACT 16
/** Disable retract for this line */
#define FLAG_JOIN_NO_RETRACT 32
/** Wait for the extruder to finish it's up movement */
#define FLAG_JOIN_WAIT_EXTRUDER_UP 64
/** Wait for the extruder to finish it's down movement */
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN 128
// Printing related data
#if DRIVE_SYSTEM==3
// Allow the delta cache to store segments for every line in line cache. Beware this gets big ... fast.
// MAX_DELTA_SEGMENTS_PER_LINE *
#define DELTA_CACHE_SIZE (MAX_DELTA_SEGMENTS_PER_LINE * MOVE_CACHE_SIZE)
typedef struct
{
    byte dir; 									///< Direction of delta movement.
    unsigned int deltaSteps[3]; 				///< Number of steps in move.
} DeltaSegment;
extern DeltaSegment segments[];					// Delta segment cache
extern unsigned int delta_segment_write_pos; 	// Position where we write the next cached delta move
extern volatile unsigned int delta_segment_count; // Number of delta moves cached 0 = nothing in cache
extern byte lastMoveID;
#endif

class PrintLine   // RAM usage: 24*4+15 = 113 Byte
{
    public:
    static PrintLine lines[];
    static PrintLine *cur;
    static byte lines_write_pos; // Position where we write the next cached line move
    static byte lines_pos; // Position for executing line movement
    static volatile byte lines_count; // Number of lines cached 0 = nothing to do

    byte primaryAxis;
    volatile byte flags;
    long timeInTicks;
    byte joinFlags;
    byte halfstep;                  ///< 0 = disabled, 1 = halfstep, 2 = fulstep
    byte dir;                       ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
    long delta[4];                  ///< Steps we want to move.
    long error[4];                  ///< Error calculation for Bresenham algorithm
    float speedX;                   ///< Speed in x direction at fullInterval in mm/s
    float speedY;                   ///< Speed in y direction at fullInterval in mm/s
    float speedZ;                   ///< Speed in z direction at fullInterval in mm/s
    float speedE;                   ///< Speed in E direction at fullInterval in mm/s
    float fullSpeed;                ///< Desired speed mm/s
    float invFullSpeed;             ///< 1.0/fullSpeed for fatser computation
    float acceleration;             ///< Real 2.0*distanceÜacceleration mm²/s²
    float maxJunctionSpeed;         ///< Max. junction speed between this and next segment
    float startSpeed;               ///< Staring speed in mm/s
    float endSpeed;                 ///< Exit speed in mm/s
    float distance;
#if DRIVE_SYSTEM==3
    byte numDeltaSegments;		  		///< Number of delta segments left in line. Decremented by stepper timer.
    byte moveID;							///< ID used to identify moves which are all part of the same line
    int deltaSegmentReadPos; 	 			///< Pointer to next DeltaSegment
    long numPrimaryStepPerSegment;		///< Number of primary bresenham axis steps in each delta segment
#endif
    unsigned long fullInterval;     ///< interval at full speed in ticks/step.
    unsigned long stepsRemaining;   ///< Remaining steps, until move is finished
    unsigned int accelSteps;        ///< How much steps does it take, to reach the plateau.
    unsigned int decelSteps;        ///< How much steps does it take, to reach the end speed.
    unsigned long accelerationPrim; ///< Acceleration along primary axis
    unsigned long facceleration;    ///< accelerationPrim*262144/F_CPU
    unsigned int vMax;              ///< Maximum reached speed in steps/s.
    unsigned int vStart;            ///< Starting speed in steps/s.
    unsigned int vEnd;              ///< End speed in steps/s
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    long advanceRate;               ///< Advance steps at full speed
    long advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
    long advanceStart;
    long advanceEnd;
#endif
    unsigned int advanceL;         ///< Recomputated L value
#endif
#if USE_OPS==1
    long opsReverseSteps;           ///< How many steps are needed to reverse retracted filament at full speed
#endif
#ifdef DEBUG_STEPCOUNT
    long totalStepsRemaining;
#endif
    inline bool areParameterUpToDate()
    {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline void invalidateParameter()
    {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline void setParameterUpToDate()
    {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
    }
    inline bool isStartSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_START_FIXED;
    }
    inline bool setStartSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
    }
    inline bool fixStartAndEndSpeed() {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
    }
    inline bool isEndSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_END_FIXED;
    }
    inline bool setEndSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
    }
    inline bool isWarmUp()
    {
        return flags & FLAG_WARMUP;
    }
    inline bool isExtruderForwardMove()
    {
        return (dir & 136)==136;
    }
    inline void block()
    {
        flags |= FLAG_BLOCKED;
    }
    inline void unblock()
    {
        flags &= ~FLAG_BLOCKED;
    }
    inline bool isBlocked()
    {
        return flags & FLAG_BLOCKED;
    }
    inline bool checkEndstops()
    {
        return flags & FLAG_CHECK_ENDSTOPS;
    }
    inline void setXMoveFinished()
    {
#if DRIVE_SYSTEM==0 || DRIVE_SYSTEM==3
        dir&=~16;
#else
        dir&=~48;
#endif
    }
    inline void setYMoveFinished()
    {
#if DRIVE_SYSTEM==0 || DRIVE_SYSTEM==3
        dir&=~32;
#else
        dir&=~48;
#endif
    }
    inline void setZMoveFinished()
    {
        dir&=~64;
    }
    inline void setXYMoveFinished()
    {
        dir&=~48;
    }
    inline bool isXPositiveMove()
    {
        return (dir & 17)==17;
    }
    inline bool isXNegativeMove()
    {
        return (dir & 17)==16;
    }
    inline bool isYPositiveMove()
    {
        return (dir & 34)==34;
    }
    inline bool isYNegativeMove()
    {
        return (dir & 34)==32;
    }
    inline bool isZPositiveMove()
    {
        return (dir & 68)==68;
    }
    inline bool isZNegativeMove()
    {
        return (dir & 68)==64;
    }
    inline bool isEPositiveMove()
    {
        return (dir & 136)==136;
    }
    inline bool isENegativeMove()
    {
        return (dir & 136)==128;
    }
    inline bool isXMove()
    {
        return (dir & 16);
    }
    inline bool isYMove()
    {
        return (dir & 32);
    }
    inline bool isXOrYMove() {
        return dir & 48;
    }
    inline bool isZMove()
    {
        return (dir & 64);
    }
    inline bool isEMove()
    {
        return (dir & 128);
    }
    inline void updateAdvanceSteps(unsigned int v,byte max_loops,bool accelerate)
    {
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        long advance_target =printer.advance_executed+advanceRate;
        for(byte loop=1; loop<max_loops; loop++) advance_target=(accelerate ? advance_target+advanceRate : advance_target-advanceRate);
        if(advance_target>advanceFull)
            advance_target = advanceFull;
        HAL::forbidInterrupts();
        long h = HAL::mulu16xu16to32(v,advanceL);
        int tred = ((advance_target+h)>>16);
        printer.extruderStepsNeeded+=tred-printer.advance_steps_set;
        printer.advance_steps_set = tred;
        HAL::allowInterrupts();
        printer.advance_executed = advance_target;
#else
        int tred = HAL::mulu6xu16shift16(v,advanceL);
        HAL::forbidInterrupts();
        printer.extruderStepsNeeded+=tred-printer.advance_steps_set;
        printer.advance_steps_set = tred;
        HAL::allowInterrupts();
#endif
#endif
    }
    inline bool moveDecelerating()
    {
        if(stepsRemaining <= decelSteps)
        {
            if (!(flags & FLAG_DECELERATING))
            {
                printer.timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        }
        else return false;
    }
    inline bool moveAccelerating()
    {
        return printer.stepNumber <= accelSteps;
    }
    inline bool startXStep()
    {
        ANALYZER_ON(ANALYZER_CH6);
#if DRIVE_SYSTEM==0 || !defined(XY_GANTRY)
        ANALYZER_ON(ANALYZER_CH2);
        WRITE(X_STEP_PIN,HIGH);
#else
#if DRIVE_SYSTEM==1
        if(isXPositiveMove())
        {
            printer.motorX++;
            printer.motorY++;
        }
        else
        {
            printer.motorX--;
            printer.motorY--;
        }
#endif
#if DRIVE_SYSTEM==2
        if(isXPositiveMove())
        {
            printer.motorX++;
            printer.motorY--;
        }
        else
        {
            printer.motorX--;
            printer.motorY++;
        }
#endif
#endif
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif

    }
    inline bool startYStep()
    {
        ANALYZER_ON(ANALYZER_CH7);
#if DRIVE_SYSTEM==0 || !defined(XY_GANTRY)
        ANALYZER_ON(ANALYZER_CH3);
        WRITE(Y_STEP_PIN,HIGH);
#else
#if DRIVE_SYSTEM==1
        if(isYPositiveMove())
        {
            printer.motorX++;
            printer.motorY--;
        }
        else
        {
            printer.motorX--;
            printer.motorY++;
        }
#endif
#if DRIVE_SYSTEM==2
        if(isYPositiveMove())
        {
            printer.motorX++;
            printer.motorY++;
        }
        else
        {
            printer.motorX--;
            printer.motorY--;
        }
#endif
#endif // XY_GANTRY
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
    }
    void updateStepsParameter();
    inline float safeSpeed();
    void calculate_move(float axis_diff[],byte check_endstops,byte pathOptimize);
    inline long getWaitTicks() {return timeInTicks;}
    inline void setWaitTicks(long wait) {timeInTicks = wait;}

    static inline bool hasLines() {return lines_count;}
    static inline void setCurrentLine() {
        cur = &lines[lines_pos];
    }
    static inline void popLineForbidInterrupt() {
        lines_pos++;
        if(lines_pos>=MOVE_CACHE_SIZE) lines_pos=0;
        cur = 0;
        HAL::forbidInterrupts();
        --lines_count;
    }
    static long bresenhamStep();
    static inline void forwardPlanner(byte p);
    static inline void backwardPlanner(byte p,byte last);
    static void updateTrapezoids(byte p);
    static byte check_new_move(byte pathOptimize, byte waitExtraLines);
    static void queue_move(byte check_endstops,byte pathOptimize);
    static void move_steps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop);
    static void split_delta_move(byte check_endstops,byte pathOptimize, byte softEndstop);
#if ARC_SUPPORT
    static void arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif
    static inline void queue_E_move(long e_diff,byte check_endstops,byte pathOptimize);
};



#endif // MOTION_H_INCLUDED
