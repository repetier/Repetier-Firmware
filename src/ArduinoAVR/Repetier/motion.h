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
typedef struct {
	byte dir; 									///< Direction of delta movement.
	unsigned int deltaSteps[3]; 				///< Number of steps in move.
} DeltaSegment;
extern DeltaSegment segments[];					// Delta segment cache
extern unsigned int delta_segment_write_pos; 	// Position where we write the next cached delta move
extern volatile unsigned int delta_segment_count; // Number of delta moves cached 0 = nothing in cache
extern byte lastMoveID;
#endif
typedef struct { // RAM usage: 24*4+15 = 113 Byte
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
  inline bool areParameterUpToDate() {return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;}
  inline void invalidateParameter() {joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;}
  inline void setParameterUpToDate() {joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;}
  inline bool isStartSpeedFixed() {return joinFlags & FLAG_JOIN_START_FIXED;}
  inline bool setStartSpeedFixed(bool newState) {joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);}
  inline bool isEndSpeedFixed() {return joinFlags & FLAG_JOIN_END_FIXED;}
  inline bool setEndSpeedFixed(bool newState) {joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);}
  inline bool isWarmUp() {return flags & FLAG_WARMUP;}
  inline bool isExtruderForwardMove() {return (dir & 136)==136;}
  inline void block() {flags |= FLAG_BLOCKED;}
  inline void unblock() {flags &= ~FLAG_BLOCKED;}
  inline bool isBlocked() {return flags & FLAG_BLOCKED;}
} PrintLine;

extern PrintLine lines[];
extern byte lines_write_pos; // Position where we write the next cached line move
extern byte lines_pos; // Position for executing line movement
extern volatile byte lines_count; // Number of lines cached 0 = nothing to do
extern byte printmoveSeen;


#endif // MOTION_H_INCLUDED
