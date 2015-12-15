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


#include "Repetier.h"
#include <Wire.h>

#if defined(USE_ADVANCE)
uint8_t			Printer::minExtruderSpeed;							///< Timer delay for start extruder speed
uint8_t			Printer::maxExtruderSpeed;							///< Timer delay for end extruder speed
volatile int	Printer::extruderStepsNeeded;						///< This many extruder steps are still needed, <0 = reverse steps needed.
#endif // defined(USE_ADVANCE)

uint8_t			Printer::unitIsInches = 0;							///< 0 = Units are mm, 1 = units are inches.

//Stepper Movement Variables
float			Printer::axisStepsPerMM[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float			Printer::invAxisStepsPerMM[4];						///< Inverse of axisStepsPerMM for faster conversion
float			Printer::maxFeedrate[4] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z}; ///< Maximum allowed feedrate.
float			Printer::homingFeedrate[3];

#ifdef RAMP_ACCELERATION
float			Printer::maxAccelerationMMPerSquareSecond[4] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float			Printer::maxTravelAccelerationMMPerSquareSecond[4] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves

/** Acceleration in steps/s^3 in printing mode.*/
unsigned long	Printer::maxPrintAccelerationStepsPerSquareSecond[4];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long	Printer::maxTravelAccelerationStepsPerSquareSecond[4];
#endif // RAMP_ACCELERATION

uint8_t			Printer::relativeCoordinateMode = false;			///< Determines absolute (false) or relative Coordinates (true).
uint8_t			Printer::relativeExtruderCoordinateMode = false;	///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

long			Printer::queuePositionLastSteps[4];
float			Printer::queuePositionLastMM[3];
float			Printer::queuePositionCommandMM[3];
long			Printer::queuePositionTargetSteps[4];
float			Printer::originOffsetMM[3] = {0,0,0};
uint8_t			Printer::flag0 = 0;
uint8_t			Printer::flag1 = 0;

#if ALLOW_EXTENDED_COMMUNICATION < 2
uint8_t			Printer::debugLevel = 0; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
#else
uint8_t			Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
#endif // ALLOW_EXTENDED_COMMUNICATION < 2

uint8_t			Printer::stepsPerTimerCall = 1;
uint8_t			Printer::menuMode = 0;

unsigned long	Printer::interval;									///< Last step duration in ticks.
unsigned long	Printer::timer;										///< used for acceleration/deceleration timing
unsigned long	Printer::stepNumber;								///< Step number in current move.

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
long			Printer::advanceExecuted;							///< Executed advance steps
#endif // ENABLE_QUADRATIC_ADVANCE

int				Printer::advanceStepsSet;
#endif // USE_ADVANCE

#if MAX_HARDWARE_ENDSTOP_Z
long			Printer::stepsRemainingAtZHit;
#endif // MAX_HARDWARE_ENDSTOP_Z

float			Printer::minimumSpeed;								///< lowest allowed speed to keep integration error small
float			Printer::minimumZSpeed;
long			Printer::maxSteps[3];								///< For software endstops, limit of move in positive direction.
long			Printer::minSteps[3];								///< For software endstops, limit of move in negative direction.
float			Printer::lengthMM[3];
float			Printer::minMM[3];
float			Printer::feedrate;									///< Last requested feedrate.
int				Printer::feedrateMultiply;							///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int	Printer::extrudeMultiply;							///< Flow multiplier in percdent (factor 1 = 100)
float			Printer::maxJerk;									///< Maximum allowed jerk in mm/s
float			Printer::maxZJerk;									///< Maximum allowed jerk in z direction in mm/s
float			Printer::extruderOffset[2];							///< offset for different extruder positions.
unsigned int	Printer::vMaxReached;								///< Maximum reached speed
unsigned long	Printer::msecondsPrinting;							///< Milliseconds of printing time (means time with heated extruder)
unsigned long	Printer::msecondsMilling;							///< Milliseconds of milling time
float			Printer::filamentPrinted;							///< mm of filament printed since counting started
uint8_t			Printer::wasLastHalfstepping;						///< Indicates if last move had halfstepping enabled
long			Printer::ZOffset;									///< Z Offset in um

#if ENABLE_BACKLASH_COMPENSATION
float			Printer::backlash[3];
uint8_t			Printer::backlashDir;
#endif // ENABLE_BACKLASH_COMPENSATION

#ifdef DEBUG_STEPCOUNT
long			Printer::totalStepsRemaining;
#endif // DEBUG_STEPCOUNT

#if FEATURE_MEMORY_POSITION
float			Printer::memoryX;
float			Printer::memoryY;
float			Printer::memoryZ;
float			Printer::memoryE;
#endif // FEATURE_MEMORY_POSITION

#ifdef DEBUG_SEGMENT_LENGTH
float			Printer::maxRealSegmentLength = 0;
#endif // DEBUG_SEGMENT_LENGTH

#ifdef DEBUG_REAL_JERK
float			Printer::maxRealJerk = 0;
#endif // DEBUG_REAL_JERK

#ifdef DEBUG_PRINT
int				debugWaitLoop = 0;
#endif // DEBUG_PRINT

#if FEATURE_HEAT_BED_Z_COMPENSATION
char			Printer::doHeatBedZCompensation;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
char			Printer::doWorkPartZCompensation;
long			Printer::staticCompensationZ;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

long			Printer::queuePositionCurrentSteps[3];
char			Printer::stepperDirection[3];
char			Printer::blockZ;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
long			Printer::compensatedPositionTargetStepsZ;
long			Printer::compensatedPositionCurrentStepsZ;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
long			Printer::directPositionTargetSteps[4];
long			Printer::directPositionCurrentSteps[4];
long			Printer::directPositionLastSteps[4];
char			Printer::waitMove;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_MILLING_MODE
char			Printer::operatingMode;
float			Printer::drillFeedrate;
float			Printer::drillZDepth;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
char			Printer::ZEndstopType;
char			Printer::ZEndstopUnknown;
char			Printer::lastZDirection;
char			Printer::endstopZMinHit;
char			Printer::endstopZMaxHit;
long			Printer::stepsSinceZMinEndstop;
long			Printer::stepsSinceZMaxEndstop;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
char			Printer::HotendType;
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

#if FEATURE_CONFIGURABLE_MILLER_TYPE
char			Printer::MillerType;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if STEPPER_ON_DELAY
char			Printer::enabledStepper[3];
#endif // STEPPER_ON_DELAY

#if FEATURE_BEEPER
char			Printer::enableBeeper;
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
char			Printer::enableCaseLight;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
char			Printer::RGBLightMode;
char			Printer::RGBLightStatus;
unsigned long	Printer::RGBLightIdleStart;
unsigned long	Printer::RGBLightLastChange;
char			Printer::RGBButtonBackPressed;
char			Printer::RGBLightModeForceWhite;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_230V_OUTPUT
char			Printer::enable230VOutput;
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
char			Printer::enableFET1;
char			Printer::enableFET2;
char			Printer::enableFET3;
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
unsigned long	Printer::prepareFanOff;
unsigned long	Printer::fanOffDelay;
#endif // FEATURE_CASE_FAN

#if FEATURE_TYPE_EEPROM
unsigned char	Printer::wrongType;
#endif // FEATURE_TYPE_EEPROM


void Printer::constrainQueueDestinationCoords()
{
    if(isNoDestinationCheck()) return;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
#if max_software_endstop_x == true
    if (queuePositionTargetSteps[X_AXIS] + directPositionTargetSteps[X_AXIS] > Printer::maxSteps[X_AXIS]) Printer::queuePositionTargetSteps[X_AXIS] = Printer::maxSteps[X_AXIS] - directPositionTargetSteps[X_AXIS];
#endif // max_software_endstop_x == true

#if max_software_endstop_y == true
    if (queuePositionTargetSteps[Y_AXIS] + directPositionTargetSteps[Y_AXIS] > Printer::maxSteps[Y_AXIS]) Printer::queuePositionTargetSteps[Y_AXIS] = Printer::maxSteps[Y_AXIS] - directPositionTargetSteps[Y_AXIS];
#endif // max_software_endstop_y == true

#if max_software_endstop_z == true
    if (queuePositionTargetSteps[Z_AXIS] + directPositionTargetSteps[Z_AXIS] > Printer::maxSteps[Z_AXIS]) Printer::queuePositionTargetSteps[Z_AXIS] = Printer::maxSteps[Z_AXIS] - directPositionTargetSteps[Z_AXIS];
#endif // max_software_endstop_z == true
#else
#if max_software_endstop_x == true
    if (queuePositionTargetSteps[X_AXIS] > Printer::maxSteps[X_AXIS]) Printer::queuePositionTargetSteps[X_AXIS] = Printer::maxSteps[X_AXIS];
#endif // max_software_endstop_x == true

#if max_software_endstop_y == true
    if (queuePositionTargetSteps[Y_AXIS] > Printer::maxSteps[Y_AXIS]) Printer::queuePositionTargetSteps[Y_AXIS] = Printer::maxSteps[Y_AXIS];
#endif // max_software_endstop_y == true

#if max_software_endstop_z == true
    if (queuePositionTargetSteps[Z_AXIS] > Printer::maxSteps[Z_AXIS]) Printer::queuePositionTargetSteps[Z_AXIS] = Printer::maxSteps[Z_AXIS];
#endif // max_software_endstop_z == true
#endif //FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

} // constrainQueueDestinationCoords


void Printer::constrainDirectDestinationCoords()
{
    if(isNoDestinationCheck()) return;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
#if max_software_endstop_x == true
    if (queuePositionTargetSteps[X_AXIS] + directPositionTargetSteps[X_AXIS] > Printer::maxSteps[X_AXIS]) Printer::directPositionTargetSteps[X_AXIS] = Printer::maxSteps[X_AXIS] - queuePositionTargetSteps[X_AXIS];
#endif // max_software_endstop_x == true

#if max_software_endstop_y == true
    if (queuePositionTargetSteps[Y_AXIS] + directPositionTargetSteps[Y_AXIS] > Printer::maxSteps[Y_AXIS]) Printer::directPositionTargetSteps[Y_AXIS] = Printer::maxSteps[Y_AXIS] - queuePositionTargetSteps[Y_AXIS];
#endif // max_software_endstop_y == true

#if max_software_endstop_z == true
    if (queuePositionTargetSteps[Z_AXIS] + directPositionTargetSteps[Z_AXIS] > Printer::maxSteps[Z_AXIS]) Printer::directPositionTargetSteps[Z_AXIS] = Printer::maxSteps[Z_AXIS] - queuePositionTargetSteps[Z_AXIS];
#endif // max_software_endstop_z == true
#endif //FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

} // constrainDirectDestinationCoords


bool Printer::isPositionAllowed(float x,float y,float z)
{
    if(isNoDestinationCheck())  return true;
    bool allowed = true;

    if(!allowed)
	{
        Printer::updateCurrentPosition(true);
        Commands::printCurrentPosition();
    }
    return allowed;

} // isPositionAllowed


void Printer::updateDerivedParameter()
{
    maxSteps[X_AXIS] = (long)(axisStepsPerMM[X_AXIS]*(minMM[X_AXIS]+lengthMM[X_AXIS]));
    maxSteps[Y_AXIS] = (long)(axisStepsPerMM[Y_AXIS]*(minMM[Y_AXIS]+lengthMM[Y_AXIS]));
    maxSteps[Z_AXIS] = (long)(axisStepsPerMM[Z_AXIS]*(minMM[Z_AXIS]+lengthMM[Z_AXIS]));
    minSteps[X_AXIS] = (long)(axisStepsPerMM[X_AXIS]*minMM[X_AXIS]);
    minSteps[Y_AXIS] = (long)(axisStepsPerMM[Y_AXIS]*minMM[Y_AXIS]);
    minSteps[Z_AXIS] = (long)(axisStepsPerMM[Z_AXIS]*minMM[Z_AXIS]);

	// For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    backlashDir &= 7;
    if(backlashX!=0) backlashDir |= 8;
    if(backlashY!=0) backlashDir |= 16;
    if(backlashZ!=0) backlashDir |= 32;
#endif // ENABLE_BACKLASH_COMPENSATION

	for(uint8_t i=0; i<4; i++)
    {
        invAxisStepsPerMM[i] = 1.0f/axisStepsPerMM[i];

#ifdef RAMP_ACCELERATION
        /** Acceleration in steps/s^3 in printing mode.*/
        maxPrintAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
        /** Acceleration in steps/s^2 in movement mode.*/
        maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif // RAMP_ACCELERATION
    }
    float accel = RMath::max(maxAccelerationMMPerSquareSecond[X_AXIS],maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    minimumSpeed = accel*sqrt(2.0f/(axisStepsPerMM[X_AXIS]*accel));
    accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS],maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
    minimumZSpeed = accel*sqrt(2.0f/(axisStepsPerMM[Z_AXIS]*accel));
    Printer::updateAdvanceFlags();

} // updateDerivedParameter


/** \brief Stop heater and stepper motors. Disable power,if possible. */
void Printer::kill(uint8_t only_steppers)
{
    if(areAllSteppersDisabled() && only_steppers) return;
    if(Printer::isAllKilled()) return;

#if FAN_PIN>-1
	// disable the fan
	Commands::setFanSpeed(0,false);
#endif // FAN_PIN>-1

	setAllSteppersDisabled();
    disableXStepper();
    disableYStepper();
    disableZStepper();
    Extruder::disableAllExtruders();

#if FAN_BOARD_PIN>-1
    pwm_pos[NUM_EXTRUDER+1] = 0;
#endif // FAN_BOARD_PIN

    if(!only_steppers)
    {
        for(uint8_t i=0; i<NUM_TEMPERATURE_LOOPS; i++)
            Extruder::setTemperatureForExtruder(0,i);
        Extruder::setHeatedBedTemperature(0);
        UI_STATUS_UPD(UI_TEXT_KILLED);

#if defined(PS_ON_PIN) && PS_ON_PIN>-1
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif // defined(PS_ON_PIN) && PS_ON_PIN>-1

        Printer::setAllKilled(true);
    }
    else
	{
		UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
	}

} // kill


void Printer::updateAdvanceFlags()
{
    Printer::setAdvanceActivated(false);

#if defined(USE_ADVANCE)
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
        if(extruder[i].advanceL!=0)
        {
            Printer::setAdvanceActivated(true);
        }
#ifdef ENABLE_QUADRATIC_ADVANCE
        if(extruder[i].advanceK!=0) Printer::setAdvanceActivated(true);
#endif // ENABLE_QUADRATIC_ADVANCE
    }
#endif // defined(USE_ADVANCE)

} // updateAdvanceFlags


void Printer::moveTo(float x,float y,float z,float e,float f)
{
    if(x != IGNORE_COORDINATE)
        queuePositionTargetSteps[X_AXIS] = (x + Printer::extruderOffset[X_AXIS]) * axisStepsPerMM[X_AXIS];
    if(y != IGNORE_COORDINATE)
        queuePositionTargetSteps[Y_AXIS] = (y + Printer::extruderOffset[Y_AXIS]) * axisStepsPerMM[Y_AXIS];
    if(z != IGNORE_COORDINATE)
        queuePositionTargetSteps[Z_AXIS] = z * axisStepsPerMM[Z_AXIS];
    if(e != IGNORE_COORDINATE)
        queuePositionTargetSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    if(f != IGNORE_COORDINATE)
        feedrate = f;

    PrintLine::prepareQueueMove(ALWAYS_CHECK_ENDSTOPS,true);
    updateCurrentPosition(false);

} // moveTo


void Printer::moveToReal(float x,float y,float z,float e,float f)
{
    if(x == IGNORE_COORDINATE)
		x = queuePositionLastMM[X_AXIS];
    if(y == IGNORE_COORDINATE)
        y = queuePositionLastMM[Y_AXIS];
    if(z == IGNORE_COORDINATE)
        z = queuePositionLastMM[Z_AXIS];

    queuePositionLastMM[X_AXIS] = x;
    queuePositionLastMM[Y_AXIS] = y;
    queuePositionLastMM[Z_AXIS] = z;

	x += Printer::extruderOffset[X_AXIS];
    y += Printer::extruderOffset[Y_AXIS];

	if(x != IGNORE_COORDINATE)
        queuePositionTargetSteps[X_AXIS] = floor(x * axisStepsPerMM[X_AXIS] + 0.5);
    if(y != IGNORE_COORDINATE)
        queuePositionTargetSteps[Y_AXIS] = floor(y * axisStepsPerMM[Y_AXIS] + 0.5);
    if(z != IGNORE_COORDINATE)
        queuePositionTargetSteps[Z_AXIS] = floor(z * axisStepsPerMM[Z_AXIS] + 0.5);
    if(e != IGNORE_COORDINATE)
        queuePositionTargetSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
    if(f != IGNORE_COORDINATE)
        feedrate = f;

    PrintLine::prepareQueueMove(ALWAYS_CHECK_ENDSTOPS,true);

} // moveToReal


uint8_t Printer::setOrigin(float xOff,float yOff,float zOff)
{
	if( !isHomed() )
	{
		if( debugErrors() )
		{
			// we can not set the origin when we do not know the home position
			Com::printFLN( PSTR("setOrigin(): the origin can not be set because the home position is unknown") );
		}
		return 0;
	}

    originOffsetMM[X_AXIS] = xOff;
    originOffsetMM[Y_AXIS] = yOff;
    originOffsetMM[Z_AXIS] = zOff;

	if( debugInfo() )
	{
		// output the new origin offsets
	    Com::printF( PSTR("setOrigin(): x="), originOffsetMM[X_AXIS] );
	    Com::printF( PSTR("; y="), originOffsetMM[Y_AXIS] );
	    Com::printFLN( PSTR("; z="), originOffsetMM[Z_AXIS] );
	}
	return 1;

} // setOrigin


void Printer::updateCurrentPosition(bool copyLastCmd)
{
    queuePositionLastMM[X_AXIS] = (float)(queuePositionLastSteps[X_AXIS])*invAxisStepsPerMM[X_AXIS];
    queuePositionLastMM[Y_AXIS] = (float)(queuePositionLastSteps[Y_AXIS])*invAxisStepsPerMM[Y_AXIS];
    queuePositionLastMM[Z_AXIS] = (float)(queuePositionLastSteps[Z_AXIS])*invAxisStepsPerMM[Z_AXIS];
    queuePositionLastMM[X_AXIS] -= Printer::extruderOffset[X_AXIS];
    queuePositionLastMM[Y_AXIS] -= Printer::extruderOffset[Y_AXIS];

    if(copyLastCmd)
	{
        queuePositionCommandMM[X_AXIS] = queuePositionLastMM[X_AXIS];
        queuePositionCommandMM[Y_AXIS] = queuePositionLastMM[Y_AXIS];
        queuePositionCommandMM[Z_AXIS] = queuePositionLastMM[Z_AXIS];
    }

} // updateCurrentPosition


/**
  \brief Sets the destination coordinates to values stored in com.

  For the computation of the destination, the following facts are considered:
  - Are units inches or mm.
  - Reltive or absolute positioning with special case only extruder relative.
  - Offset in x and y direction for multiple extruder support.
*/
uint8_t Printer::setDestinationStepsFromGCode(GCode *com)
{
    register long	p;
    float			x, y, z;


    if(!relativeCoordinateMode)
    {
        if(com->hasX()) queuePositionCommandMM[X_AXIS] = queuePositionLastMM[X_AXIS] = convertToMM(com->X) - originOffsetMM[X_AXIS];
        if(com->hasY()) queuePositionCommandMM[Y_AXIS] = queuePositionLastMM[Y_AXIS] = convertToMM(com->Y) - originOffsetMM[Y_AXIS];
        if(com->hasZ())
		{
			queuePositionCommandMM[Z_AXIS] = queuePositionLastMM[Z_AXIS] = convertToMM(com->Z) - originOffsetMM[Z_AXIS];

/*			Com::printF( PSTR( "New z from G-Code: " ), queuePositionCommandMM[Z_AXIS] );
			Com::printF( PSTR( "; " ), queuePositionLastMM[Z_AXIS] );
			Com::printF( PSTR( "; " ), convertToMM(com->Z) );
			Com::printF( PSTR( "; " ), originOffsetMM[Z_AXIS] );
*/		}
	}
    else
    {
        if(com->hasX()) queuePositionLastMM[X_AXIS] = (queuePositionCommandMM[X_AXIS] += convertToMM(com->X));
        if(com->hasY()) queuePositionLastMM[Y_AXIS] = (queuePositionCommandMM[Y_AXIS] += convertToMM(com->Y));
        if(com->hasZ()) queuePositionLastMM[Z_AXIS] = (queuePositionCommandMM[Z_AXIS] += convertToMM(com->Z));
	}

	x = queuePositionCommandMM[X_AXIS] + Printer::extruderOffset[X_AXIS];
    y = queuePositionCommandMM[Y_AXIS] + Printer::extruderOffset[Y_AXIS];
    z = queuePositionCommandMM[Z_AXIS];

	long xSteps = static_cast<long>(floor(x * axisStepsPerMM[X_AXIS] + 0.5));
    long ySteps = static_cast<long>(floor(y * axisStepsPerMM[Y_AXIS] + 0.5));
    long zSteps = static_cast<long>(floor(z * axisStepsPerMM[Z_AXIS] + 0.5));
    
	if(com->hasX())
	{
		queuePositionTargetSteps[X_AXIS] = xSteps;
	}
	else
	{
		queuePositionTargetSteps[X_AXIS] = queuePositionLastSteps[X_AXIS];
	}

    if(com->hasY())
	{
		queuePositionTargetSteps[Y_AXIS] = ySteps;
	}
	else
	{
		queuePositionTargetSteps[Y_AXIS] = queuePositionLastSteps[Y_AXIS];
	}

    if(com->hasZ())
	{
		queuePositionTargetSteps[Z_AXIS] = zSteps;
//		Com::printFLN( PSTR( "; " ), queuePositionTargetSteps[Z_AXIS] );
	}
	else
	{
		queuePositionTargetSteps[Z_AXIS] = queuePositionLastSteps[Z_AXIS];
	}

	if(com->hasE() && !Printer::debugDryrun())
    {
        p = convertToMM(com->E * axisStepsPerMM[E_AXIS]);
        if(relativeCoordinateMode || relativeExtruderCoordinateMode)
        {
            if(
#if MIN_EXTRUDER_TEMP > 30
                Extruder::current->tempControl.currentTemperatureC<MIN_EXTRUDER_TEMP ||
#endif // MIN_EXTRUDER_TEMP > 30

                fabs(p - queuePositionLastSteps[E_AXIS]) > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS])
			{
                p = 0;
			}
            queuePositionTargetSteps[E_AXIS] = queuePositionLastSteps[E_AXIS] + p;
			
        }
		else
		{
            if(
#if MIN_EXTRUDER_TEMP > 30
                Extruder::current->tempControl.currentTemperatureC<MIN_EXTRUDER_TEMP ||
#endif // MIN_EXTRUDER_TEMP > 30

                fabs(p - queuePositionLastSteps[E_AXIS]) > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS])
			{
                queuePositionLastSteps[E_AXIS] = p;
			}
            queuePositionTargetSteps[E_AXIS] = p;
        }
    }
    else
	{
		queuePositionTargetSteps[E_AXIS] = queuePositionLastSteps[E_AXIS];
	}

    if(com->hasF())
    {
        if(unitIsInches)
            feedrate = com->F * 0.0042333f * (float)feedrateMultiply;  // Factor is 25.5/60/100
        else
            feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
    }

    if(!Printer::isPositionAllowed(x,y,z))
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "We should not be here." ) );
		}
        queuePositionLastSteps[E_AXIS] = queuePositionTargetSteps[E_AXIS];
        return false; // ignore move
    }
    return !com->hasNoXYZ() || (com->hasE() && queuePositionTargetSteps[E_AXIS] != queuePositionLastSteps[E_AXIS]); // ignore unproductive moves

} // setDestinationStepsFromGCode


void Printer::setup()
{
    HAL::stopWatchdog();

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
		Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
		Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
	}
	else
	{
		Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_MILL;
		Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_MILL;
		Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_MILL;
	}
#else
    Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
    Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
    Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
#endif // FEATURE_MILLING_MODE

    //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization
    HAL::hwSetup();

	HAL::allowInterrupts();

#if FEATURE_BEEPER
	enableBeeper = BEEPER_MODE;
#endif // FEATURE_BEEPER

	Wire.begin();

#if FEATURE_TYPE_EEPROM
	determineHardwareType();

	if( wrongType )
	{
		// this firmware is not for this hardware
		while( 1 )
		{
			// this firmware shall not try to do anything at this hardware
		}
	}
#endif // FEATURE_TYPE_EEPROM

#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0>=0
    SET_OUTPUT(ANALYZER_CH0);
#endif // ANALYZER_CH0>=0

#if ANALYZER_CH1>=0
    SET_OUTPUT(ANALYZER_CH1);
#endif // ANALYZER_CH1>=0

#if ANALYZER_CH2>=0
    SET_OUTPUT(ANALYZER_CH2);
#endif // ANALYZER_CH2>=0

#if ANALYZER_CH3>=0
    SET_OUTPUT(ANALYZER_CH3);
#endif // ANALYZER_CH3>=0

#if ANALYZER_CH4>=0
    SET_OUTPUT(ANALYZER_CH4);
#endif // ANALYZER_CH4>=0

#if ANALYZER_CH5>=0
    SET_OUTPUT(ANALYZER_CH5);
#endif // ANALYZER_CH5>=0

#if ANALYZER_CH6>=0
    SET_OUTPUT(ANALYZER_CH6);
#endif // ANALYZER_CH6>=0

#if ANALYZER_CH7>=0
    SET_OUTPUT(ANALYZER_CH7);
#endif // ANALYZER_CH7>=0
#endif // ANALYZER

#if defined(ENABLE_POWER_ON_STARTUP) && PS_ON_PIN>-1
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif // defined(ENABLE_POWER_ON_STARTUP) && PS_ON_PIN>-1

    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
    SET_OUTPUT(Z_STEP_PIN);

    //Initialize Dir Pins
#if X_DIR_PIN>-1
    SET_OUTPUT(X_DIR_PIN);
#endif // X_DIR_PIN>-1

#if Y_DIR_PIN>-1
    SET_OUTPUT(Y_DIR_PIN);
#endif // Y_DIR_PIN>-1

#if Z_DIR_PIN>-1
    SET_OUTPUT(Z_DIR_PIN);
#endif // Z_DIR_PIN>-1

    //Steppers default to disabled.
#if X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
    disableXStepper();
#endif // X_ENABLE_PIN > -1

#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
    disableYStepper();
#endif // Y_ENABLE_PIN > -1

#if Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    disableZStepper();
#endif // Z_ENABLE_PIN > -1

#if FEATURE_TWO_XSTEPPER
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);

#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X2_ENABLE_PIN,HIGH);
#endif // X2_ENABLE_PIN > -1
#endif // FEATURE_TWO_XSTEPPER

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);

#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y2_ENABLE_PIN,HIGH);
#endif // Y2_ENABLE_PIN > -1
#endif // FEATURE_TWO_YSTEPPER

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);

#if X2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
#endif // X2_ENABLE_PIN > -1
#endif // FEATURE_TWO_ZSTEPPER

    //endstop pullups
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN>-1
    SET_INPUT(X_MIN_PIN);

#if ENDSTOP_PULLUP_X_MIN
    PULLUP(X_MIN_PIN,HIGH);
#endif // ENDSTOP_PULLUP_X_MIN
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif // X_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_X

#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN>-1
    SET_INPUT(Y_MIN_PIN);

#if ENDSTOP_PULLUP_Y_MIN
    PULLUP(Y_MIN_PIN,HIGH);
#endif // ENDSTOP_PULLUP_Y_MIN
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif // Y_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_Y

#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN>-1
    SET_INPUT(Z_MIN_PIN);

#if ENDSTOP_PULLUP_Z_MIN
    PULLUP(Z_MIN_PIN,HIGH);
#endif // ENDSTOP_PULLUP_Z_MIN
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif // Z_MIN_PIN>-1
#endif // MIN_HARDWARE_ENDSTOP_Z

#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN>-1
    SET_INPUT(X_MAX_PIN);

#if ENDSTOP_PULLUP_X_MAX
    PULLUP(X_MAX_PIN,HIGH);
#endif // ENDSTOP_PULLUP_X_MAX
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif // X_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_X

#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN>-1
    SET_INPUT(Y_MAX_PIN);

#if ENDSTOP_PULLUP_Y_MAX
    PULLUP(Y_MAX_PIN,HIGH);
#endif // ENDSTOP_PULLUP_Y_MAX
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif // Y_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_Y

#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN>-1
    SET_INPUT(Z_MAX_PIN);

#if ENDSTOP_PULLUP_Z_MAX
    PULLUP(Z_MAX_PIN,HIGH);
#endif // ENDSTOP_PULLUP_Z_MAX
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif // Z_MAX_PIN>-1
#endif // MAX_HARDWARE_ENDSTOP_Z

#if FAN_PIN>-1
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN,LOW);
#endif // FAN_PIN>-1

#if FAN_BOARD_PIN>-1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN,LOW);
#endif // FAN_BOARD_PIN>-1

#if EXT0_HEATER_PIN>-1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // EXT0_HEATER_PIN>-1

#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1

#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    SET_OUTPUT(EXT2_HEATER_PIN);
    WRITE(EXT2_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2

#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    SET_OUTPUT(EXT3_HEATER_PIN);
    WRITE(EXT3_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3

#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    SET_OUTPUT(EXT4_HEATER_PIN);
    WRITE(EXT4_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4

#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    SET_OUTPUT(EXT5_HEATER_PIN);
    WRITE(EXT5_HEATER_PIN,HEATER_PINS_INVERTED);
#endif // defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5

#if EXT0_EXTRUDER_COOLER_PIN>-1
    SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
    WRITE(EXT0_EXTRUDER_COOLER_PIN,LOW);
#endif // EXT0_EXTRUDER_COOLER_PIN>-1

#if defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_EXTRUDER_COOLER_PIN);
    WRITE(EXT1_EXTRUDER_COOLER_PIN,LOW);
#endif // defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>1

#if defined(EXT2_EXTRUDER_COOLER_PIN) && EXT2_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>2
    SET_OUTPUT(EXT2_EXTRUDER_COOLER_PIN);
    WRITE(EXT2_EXTRUDER_COOLER_PIN,LOW);
#endif // defined(EXT2_EXTRUDER_COOLER_PIN) && EXT2_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>2

#if defined(EXT3_EXTRUDER_COOLER_PIN) && EXT3_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>3
    SET_OUTPUT(EXT3_EXTRUDER_COOLER_PIN);
    WRITE(EXT3_EXTRUDER_COOLER_PIN,LOW);
#endif // defined(EXT3_EXTRUDER_COOLER_PIN) && EXT3_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>3

#if defined(EXT4_EXTRUDER_COOLER_PIN) && EXT4_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>4
    SET_OUTPUT(EXT4_EXTRUDER_COOLER_PIN);
    WRITE(EXT4_EXTRUDER_COOLER_PIN,LOW);
#endif // defined(EXT4_EXTRUDER_COOLER_PIN) && EXT4_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>4

#if defined(EXT5_EXTRUDER_COOLER_PIN) && EXT5_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>5
    SET_OUTPUT(EXT5_EXTRUDER_COOLER_PIN);
    WRITE(EXT5_EXTRUDER_COOLER_PIN,LOW);
#endif // defined(EXT5_EXTRUDER_COOLER_PIN) && EXT5_EXTRUDER_COOLER_PIN>-1 && NUM_EXTRUDER>5

    motorCurrentControlInit(); // Set current if it is firmware controlled

	feedrate = 50; ///< Current feedrate in mm/s.
    feedrateMultiply = 100;
    extrudeMultiply = 100;
    queuePositionCommandMM[X_AXIS] = queuePositionCommandMM[Y_AXIS] = queuePositionCommandMM[Z_AXIS] = 0;

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceExecuted = 0;
#endif // ENABLE_QUADRATIC_ADVANCE

    advanceStepsSet = 0;
#endif // USE_ADVANCE

    for(uint8_t i=0; i<NUM_EXTRUDER+3; i++) pwm_pos[i]=0;
    queuePositionLastSteps[X_AXIS] = queuePositionLastSteps[Y_AXIS] = queuePositionLastSteps[Z_AXIS] = queuePositionLastSteps[E_AXIS] = 0;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	directPositionTargetSteps[X_AXIS]  = directPositionTargetSteps[Y_AXIS]  = directPositionTargetSteps[Z_AXIS]  = directPositionTargetSteps[E_AXIS]  = 0;
	directPositionCurrentSteps[X_AXIS] = directPositionCurrentSteps[Y_AXIS] = directPositionCurrentSteps[Z_AXIS] = directPositionCurrentSteps[E_AXIS] = 0;
	directPositionLastSteps[X_AXIS]	   = directPositionLastSteps[Y_AXIS]    = directPositionLastSteps[Z_AXIS]    = directPositionLastSteps[E_AXIS]    = 0;
	waitMove						   = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

	maxJerk = MAX_JERK;
    maxZJerk = MAX_ZJERK;
    extruderOffset[X_AXIS] = extruderOffset[Y_AXIS] = 0;

	ZOffset = 0;
    interval = 5000;
    stepsPerTimerCall = 1;
    msecondsPrinting = 0;
	msecondsMilling = 0;
    filamentPrinted = 0;
    flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
    lengthMM[X_AXIS] = X_MAX_LENGTH;
    lengthMM[Y_AXIS] = Y_MAX_LENGTH;
    lengthMM[Z_AXIS] = Z_MAX_LENGTH;
    minMM[X_AXIS] = X_MIN_POS;
    minMM[Y_AXIS] = Y_MIN_POS;
    minMM[Z_AXIS] = Z_MIN_POS;
    wasLastHalfstepping = 0;

#if ENABLE_BACKLASH_COMPENSATION
    backlash[X_AXIS] = X_BACKLASH;
    backlash[Y_AXIS] = Y_BACKLASH;
    backlash[Z_AXIS] = Z_BACKLASH;
    backlashDir = 0;
#endif // ENABLE_BACKLASH_COMPENSATION

#if FEATURE_HEAT_BED_Z_COMPENSATION
    doHeatBedZCompensation = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    doWorkPartZCompensation = 0;
	staticCompensationZ		= 0;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

    queuePositionCurrentSteps[X_AXIS] = 
    queuePositionCurrentSteps[Y_AXIS] = 
    queuePositionCurrentSteps[Z_AXIS] = 0;
	stepperDirection[X_AXIS]		  = 
	stepperDirection[Y_AXIS]		  = 
	stepperDirection[Z_AXIS]		  = 0;
	blockZ							  = 0;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
	compensatedPositionTargetStepsZ	 =
    compensatedPositionCurrentStepsZ = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    directPositionCurrentSteps[X_AXIS] =
    directPositionCurrentSteps[Y_AXIS] =
    directPositionCurrentSteps[Z_AXIS] =
    directPositionCurrentSteps[E_AXIS] =
    directPositionTargetSteps[X_AXIS]  =
    directPositionTargetSteps[Y_AXIS]  =
    directPositionTargetSteps[Z_AXIS]  =
    directPositionTargetSteps[E_AXIS]  = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_MILLING_MODE
	operatingMode = DEFAULT_OPERATING_MODE;
	drillFeedrate = 0.0;
	drillZDepth	  = 0.0;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	ZEndstopType		  = DEFAULT_Z_ENDSTOP_TYPE;
	ZEndstopUnknown		  = 0;
	lastZDirection		  = 0;
	endstopZMinHit		  = ENDSTOP_NOT_HIT;
	endstopZMaxHit		  = ENDSTOP_NOT_HIT;
	stepsSinceZMinEndstop = 0;
	stepsSinceZMaxEndstop = 0;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if STEPPER_ON_DELAY
	enabledStepper[X_AXIS] = 0;
	enabledStepper[Y_AXIS] = 0;
	enabledStepper[Z_AXIS] = 0;
#endif // STEPPER_ON_DELAY

#if FEATURE_CASE_LIGHT
	enableCaseLight = CASE_LIGHTS_DEFAULT_ON;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_230V_OUTPUT
	enable230VOutput = OUTPUT_230V_DEFAULT_ON;
	SET_OUTPUT(OUTPUT_230V_PIN);
	WRITE(OUTPUT_230V_PIN, enable230VOutput);
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
	enableFET1 = FET1_DEFAULT_ON;
	enableFET2 = FET2_DEFAULT_ON;
	enableFET3 = FET3_DEFAULT_ON;

	SET_OUTPUT(FET1);
	WRITE(FET1, enableFET1);
	
	SET_OUTPUT(FET2);
	WRITE(FET2, enableFET2);
	
	SET_OUTPUT(FET3);
	WRITE(FET3, enableFET3);
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
	fanOffDelay = CASE_FAN_OFF_DELAY;
#endif // FEATURE_CASE_FAN

#if FEATURE_RGB_LIGHT_EFFECTS
	RGBLightMode = RGB_LIGHT_DEFAULT_MODE;
	if( RGBLightMode == RGB_MODE_AUTOMATIC )
	{
		RGBLightStatus = RGB_STATUS_AUTOMATIC;
	}
	else
	{
		RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
	}
	RGBLightIdleStart	   = 0;
	RGBLightLastChange	   = 0;
	RGBButtonBackPressed   = 0;
	RGBLightModeForceWhite = 0;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if defined(USE_ADVANCE)
    extruderStepsNeeded = 0;
#endif // defined(USE_ADVANCE)

    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);

	if( Printer::debugInfo() )
	{
	    Com::printFLN(Com::tStart);
	}

#if FEATURE_WATCHDOG
	if( Printer::debugInfo() )
	{
	    Com::printFLN(Com::tStartWatchdog);
	}
    HAL::startWatchdog();
#endif // FEATURE_WATCHDOG

	UI_INITIALIZE;

    HAL::showStartReason();
    Extruder::initExtruder();
    EEPROM::init(); // Read settings from eeprom if wanted

#if FEATURE_CASE_LIGHT
    SET_OUTPUT(CASE_LIGHT_PIN);
	WRITE(CASE_LIGHT_PIN, enableCaseLight);
#endif // FEATURE_CASE_LIGHT

    SET_OUTPUT(CASE_FAN_PIN);

#if CASE_FAN_ALWAYS_ON
	WRITE(CASE_FAN_PIN, 1);
#else
	WRITE(CASE_FAN_PIN, 0);
#endif // CASE_FAN_ALWAYS_ON

	updateDerivedParameter();
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();

    Extruder::selectExtruderById(0);

#if SDSUPPORT
    sd.initsd();
#endif // SDSUPPORT

#if FEATURE_RGB_LIGHT_EFFECTS
	setRGBLEDs( 0, 0, 0 );

	switch( RGBLightMode )
	{
		case RGB_MODE_OFF:
		{
			setRGBTargetColors( 0, 0, 0 );
			break;
		}
		case RGB_MODE_WHITE:
		{
			setRGBTargetColors( 255, 255, 255 );
			break;
		}
		case RGB_MODE_AUTOMATIC:
		{
			setRGBTargetColors( g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB );
			break;
		}
		case RGB_MODE_MANUAL:
		{
			setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );
			break;
		}
	}
#endif // FEATURE_RGB_LIGHT_EFFECTS

} // setup


void Printer::defaultLoopActions()
{
    Commands::checkForPeriodicalActions();  //check heater every n milliseconds

	UI_MEDIUM; // do check encoder
    millis_t curtime = HAL::timeInMilliseconds();

    if(PrintLine::hasLines())
	{
        previousMillisCmd = curtime;
	}
    else
    {
        curtime -= previousMillisCmd;
        if(maxInactiveTime!=0 && curtime >  maxInactiveTime ) Printer::kill(false);
        else Printer::setAllKilled(false); // prevent repeated kills
        if(stepperInactiveTime!=0 && curtime >  stepperInactiveTime )
		{
            Printer::kill(true);
		}
    }

#if defined(SDCARDDETECT) && SDCARDDETECT>-1 && defined(SDSUPPORT) && SDSUPPORT
    sd.automount();
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1 && defined(SDSUPPORT) && SDSUPPORT

    DEBUG_MEMORY;

} // defaultLoopActions


#if FEATURE_MEMORY_POSITION
void Printer::MemoryPosition()
{
    updateCurrentPosition(false);
    lastCalculatedPosition(memoryX,memoryY,memoryZ);
    memoryE = queuePositionLastSteps[E_AXIS]*axisStepsPerMM[E_AXIS];

} // MemoryPosition


void Printer::GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed)
{
    bool all = !(x || y || z);
    float oldFeedrate = feedrate;


    moveToReal((all || x ? memoryX : IGNORE_COORDINATE)
               ,(all || y ? memoryY : IGNORE_COORDINATE)
               ,(all || z ? memoryZ : IGNORE_COORDINATE)
               ,(e ? memoryE:IGNORE_COORDINATE),
               feed);
    feedrate = oldFeedrate;

} // GoToMemoryPosition
#endif // FEATURE_MEMORY_POSITION


void Printer::homeXAxis()
{
    long steps;


    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR==1))
    {
        long offX = 0;

#if NUM_EXTRUDER>1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
#if X_HOME_DIR < 0
            offX = RMath::max(offX,extruder[i].xOffset);
#else
            offX = RMath::min(offX,extruder[i].xOffset);
#endif // X_HOME_DIR < 0
#endif // NUM_EXTRUDER>1

		UI_STATUS_UPD(UI_TEXT_HOME_X);

		steps = (Printer::maxSteps[X_AXIS]-Printer::minSteps[X_AXIS]) * X_HOME_DIR;
        queuePositionLastSteps[X_AXIS] = -steps;
        PrintLine::moveRelativeDistanceInSteps(2*steps,0,0,0,homingFeedrate[X_AXIS],true,true);
        queuePositionLastSteps[X_AXIS] = (X_HOME_DIR == -1) ? minSteps[X_AXIS]-offX : maxSteps[X_AXIS] + offX;
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * 2 * ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);

#if defined(ENDSTOP_X_BACK_ON_HOME)
        if(ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[X_AXIS] * -ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS],true,false);
#endif // defined(ENDSTOP_X_BACK_ON_HOME)

        queuePositionLastSteps[X_AXIS]	  = (X_HOME_DIR == -1) ? minSteps[X_AXIS]-offX : maxSteps[X_AXIS]+offX;
        queuePositionCurrentSteps[X_AXIS] = queuePositionLastSteps[X_AXIS];

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        directPositionTargetSteps[X_AXIS]  = 0;
        directPositionCurrentSteps[X_AXIS] = 0;
		directPositionLastSteps[X_AXIS]	   = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset-offX) * X_HOME_DIR,0,0,0,homingFeedrate[X_AXIS],true,false);
#endif // NUM_EXTRUDER>1

		// show that we are active
		previousMillisCmd = HAL::timeInMilliseconds();
	}

} // homeXAxis


void Printer::homeYAxis()
{
    long steps;


    if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR==1))
    {
        long offY = 0;

#if NUM_EXTRUDER>1
        // Reposition extruder that way, that all extruders can be selected at home pos.
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
#if Y_HOME_DIR<0
            offY = RMath::max(offY,extruder[i].yOffset);
#else
            offY = RMath::min(offY,extruder[i].yOffset);
#endif // Y_HOME_DIR<0
#endif // NUM_EXTRUDER>1

        UI_STATUS_UPD(UI_TEXT_HOME_Y);

		steps = (maxSteps[Y_AXIS]-Printer::minSteps[Y_AXIS]) * Y_HOME_DIR;
        queuePositionLastSteps[Y_AXIS] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,2*steps,0,0,homingFeedrate[Y_AXIS],true,true);
        queuePositionLastSteps[Y_AXIS] = (Y_HOME_DIR == -1) ? minSteps[Y_AXIS]-offY : maxSteps[Y_AXIS]+offY;
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*-ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*2*ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);

#if defined(ENDSTOP_Y_BACK_ON_HOME)
        if(ENDSTOP_Y_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[Y_AXIS]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS],true,false);
#endif // defined(ENDSTOP_Y_BACK_ON_HOME)

        queuePositionLastSteps[Y_AXIS]	  = (Y_HOME_DIR == -1) ? minSteps[Y_AXIS]-offY : maxSteps[Y_AXIS]+offY;
        queuePositionCurrentSteps[Y_AXIS] = queuePositionLastSteps[Y_AXIS];

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        directPositionTargetSteps[Y_AXIS]  = 0;
        directPositionCurrentSteps[Y_AXIS] = 0;
		directPositionLastSteps[Y_AXIS]	   = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps(0,(Extruder::current->yOffset-offY) * Y_HOME_DIR,0,0,homingFeedrate[Y_AXIS],true,false);
#endif // NUM_EXTRUDER>1

		// show that we are active
		previousMillisCmd = HAL::timeInMilliseconds();
    }

} // homeYAxis


void Printer::homeZAxis()
{
    long	steps;
	char	nProcess = 0;
	char	nHomeDir;


#if FEATURE_MILLING_MODE

	nProcess = 1;
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		// in operating mode "print" we use the z min endstop
		nHomeDir = -1;
	}
	else
	{
		// in operating mode "mill" we use the z max endstop
		nHomeDir = 1;
	}

#else

    if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR==1))
    {
		nProcess = 1;
		nHomeDir = Z_HOME_DIR;
	}

#endif // FEATURE_MILLING_MODE

	if( nProcess )
	{
        UI_STATUS_UPD(UI_TEXT_HOME_Z);

#if FEATURE_FIND_Z_ORIGIN
		g_nZOriginPosition[X_AXIS] = 0;
		g_nZOriginPosition[Y_AXIS] = 0;
		g_nZOriginPosition[Z_AXIS] = 0;
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        directPositionTargetSteps[Z_AXIS]  = 0;
        directPositionCurrentSteps[Z_AXIS] = 0;
		directPositionLastSteps[Z_AXIS]	   = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
		if( ZEndstopUnknown )
		{
			// in case we do not know which z-endstop is currently active, we always move downwards first
			// in case z-min was active, z-min will not be active anymore
			// in case z-max was active, it will remain active

			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "driving free downwards" ) );
			}
			PrintLine::moveRelativeDistanceInSteps(0,0,UNKNOWN_Z_ENDSTOP_DRIVE_FREE_STEPS,0,homingFeedrate[Z_AXIS],true,false);

			if( READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING )
			{
				// in case z-max is still active, we move upwards in order to drive it free
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "driving free upwards" ) );
				}
				PrintLine::moveRelativeDistanceInSteps(0,0,-UNKNOWN_Z_ENDSTOP_DRIVE_FREE_STEPS,0,homingFeedrate[Z_AXIS],true,false);
			}

			if( READ(Z_MAX_PIN) == ENDSTOP_Z_MAX_INVERTING )
			{
				// there is no active z-endstop any more, we can continue with all movements normally
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "driven free" ) );
				}

				lastZDirection		   = 0;
				endstopZMinHit		   = ENDSTOP_NOT_HIT;
				endstopZMaxHit		   = ENDSTOP_NOT_HIT;
				stepsSinceZMinEndstop = 0;
				stepsSinceZMaxEndstop = 0;
				ZEndstopUnknown		  = 0;
			}
		}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

		steps = (maxSteps[Z_AXIS] - minSteps[Z_AXIS]) * nHomeDir;
        queuePositionLastSteps[Z_AXIS] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,0,2*steps,0,homingFeedrate[Z_AXIS],true,true);
        queuePositionLastSteps[Z_AXIS] = (nHomeDir == -1) ? minSteps[Z_AXIS] : maxSteps[Z_AXIS];
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_MOVE * nHomeDir,0,homingFeedrate[Z_AXIS]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*2*ENDSTOP_Z_BACK_MOVE * nHomeDir,0,homingFeedrate[Z_AXIS]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);

#if FEATURE_MILLING_MODE
		// when the milling mode is active and we are in operating mode "mill", we use the z max endstop and we free the z-max endstop after it has been hit
		if( Printer::operatingMode == OPERATING_MODE_MILL )
		{
			PrintLine::moveRelativeDistanceInSteps(0,0,LEAVE_Z_MAX_ENDSTOP_AFTER_HOME,0,homingFeedrate[Z_AXIS],true,false);
		}
#else

#if defined(ENDSTOP_Z_BACK_ON_HOME)
        if(ENDSTOP_Z_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[Z_AXIS]*-ENDSTOP_Z_BACK_ON_HOME * nHomeDir,0,homingFeedrate[Z_AXIS],true,false);
#endif // defined(ENDSTOP_Z_BACK_ON_HOME)
#endif // FEATURE_MILLING_MODE

		queuePositionLastSteps[Z_AXIS]	  = (nHomeDir == -1) ? minSteps[Z_AXIS] : maxSteps[Z_AXIS];
        queuePositionCurrentSteps[Z_AXIS] = queuePositionLastSteps[Z_AXIS];

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        g_nZScanZPosition = 0;
        queueTask( TASK_INIT_Z_COMPENSATION );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

		// show that we are active
		previousMillisCmd = HAL::timeInMilliseconds();
	}

} // homeZAxis


void Printer::homeAxis(bool xaxis,bool yaxis,bool zaxis) // home non-delta printer
{
    float	startX,startY,startZ;
	char	homingOrder;


    lastCalculatedPosition(startX,startY,startZ);

#if FEATURE_MILLING_MODE
	if( operatingMode == OPERATING_MODE_PRINT )
	{
		homingOrder = HOMING_ORDER_PRINT;
	}
	else
	{
		homingOrder = HOMING_ORDER_MILL;
	}
#else
	homingOrder = HOMING_ORDER;
#endif // FEATURE_MILLING_MODE

	switch( homingOrder )
	{
		case HOME_ORDER_XYZ:
		{
			if(xaxis) homeXAxis();
			if(yaxis) homeYAxis();
			if(zaxis) homeZAxis();
			break;
		}
		case HOME_ORDER_XZY:
		{
			if(xaxis) homeXAxis();
			if(zaxis) homeZAxis();
			if(yaxis) homeYAxis();
			break;
		}
		case HOME_ORDER_YXZ:
		{
			if(yaxis) homeYAxis();
			if(xaxis) homeXAxis();
			if(zaxis) homeZAxis();
			break;
		}
		case HOME_ORDER_YZX:
		{
			if(yaxis) homeYAxis();
			if(zaxis) homeZAxis();
			if(xaxis) homeXAxis();
			break;
		}
		case HOME_ORDER_ZXY:
		{
			if(zaxis) homeZAxis();
			if(xaxis) homeXAxis();
			if(yaxis) homeYAxis();
			break;
		}
		case HOME_ORDER_ZYX:
		{
			if(zaxis) homeZAxis();
			if(yaxis) homeYAxis();
			if(xaxis) homeXAxis();
			break;
		}
	}

	if(xaxis)
    {
        if(X_HOME_DIR<0) startX = Printer::minMM[X_AXIS];
        else startX = Printer::minMM[X_AXIS]+Printer::lengthMM[X_AXIS];
    }
    if(yaxis)
    {
        if(Y_HOME_DIR<0) startY = Printer::minMM[Y_AXIS];
        else startY = Printer::minMM[Y_AXIS]+Printer::lengthMM[Y_AXIS];
    }
    if(zaxis)
    {
#if FEATURE_MILLING_MODE

		if( Printer::operatingMode == OPERATING_MODE_PRINT )
		{
			startZ = Printer::minMM[Z_AXIS];
		}
		else
		{
			startZ = Printer::minMM[Z_AXIS]+Printer::lengthMM[Z_AXIS];
		}
#else

		if(Z_HOME_DIR<0) startZ = Printer::minMM[Z_AXIS];
        else startZ = Printer::minMM[Z_AXIS]+Printer::lengthMM[Z_AXIS];

#endif // FEATURE_MILLING_MODE

		setZOriginSet(false);

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
		ZEndstopUnknown = false;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
    }
    updateCurrentPosition(true);
    moveToReal(startX,startY,startZ,IGNORE_COORDINATE,homingFeedrate[X_AXIS]);

    setHomed(true);
	UI_CLEAR_STATUS
    Commands::printCurrentPosition();

} // homeAxis


bool Printer::allowQueueMove( void )
{
	if( (g_pauseStatus != PAUSE_STATUS_NONE && g_pauseStatus != PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE) && !PrintLine::cur )
	{
		// do not allow to process new moves from the queue while the printing is paused
		return false;
	}

	if( !PrintLine::hasLines() )
	{
		// do not allow to process moves from the queue in case there is no queue
		return false;
	}

	// we are not paused and there are moves in our queue - we should process them
	return true;

} // allowQueueMove


#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
bool Printer::allowDirectMove( void )
{
	if( PrintLine::direct.stepsRemaining )
	{
		// the currently known direct movements must be processed as move
		return true;
	}

	// the currently known direct movements must be processed as single steps
	return false;

} // allowDirectMove


bool Printer::allowDirectSteps( void )
{
	if( PrintLine::direct.stepsRemaining )
	{
		// the currently known direct movements must be processed as move
		return false;
	}

	// the currently known direct movements must be processed as single steps
	return true;

} // allowDirectSteps


bool Printer::processAsDirectSteps( void )
{
	if( PrintLine::linesCount || waitMove )
	{
		// we are printing at the moment, thus all direct movements must be processed as single steps
		return true;
	}

	// we are not printing at the moment, thus all direct movements must be processed as move
	return false;

} // processAsDirectSteps


void Printer::resetDirectPosition( void )
{
	char	axis;


	// we may have to update our x/y/z queue positions - there is no need/sense to update the extruder queue position
	for( axis=0; axis<3; axis++ )
	{
		if( directPositionCurrentSteps[axis] )
		{
			queuePositionCurrentSteps[axis] += directPositionCurrentSteps[axis];
			queuePositionLastSteps[axis]	+= directPositionCurrentSteps[axis];
			queuePositionTargetSteps[axis]	+= directPositionCurrentSteps[axis];

			queuePositionCommandMM[axis]	= 
			queuePositionLastMM[axis]		= (float)(queuePositionLastSteps[axis])*invAxisStepsPerMM[axis];
		}
	}

	directPositionTargetSteps[X_AXIS]  = 
	directPositionTargetSteps[Y_AXIS]  = 
	directPositionTargetSteps[Z_AXIS]  = 
	directPositionTargetSteps[E_AXIS]  = 
	directPositionCurrentSteps[X_AXIS] = 
	directPositionCurrentSteps[Y_AXIS] = 
	directPositionCurrentSteps[Z_AXIS] = 
	directPositionCurrentSteps[E_AXIS] = 0;
	return;

} // resetDirectPosition
#endif FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
void Printer::performZCompensation( void )
{
	char	doZCompensation = 0;


	// the order of the following checks shall not be changed
#if FEATURE_HEAT_BED_Z_COMPENSATION
	if( doHeatBedZCompensation )
	{
		doZCompensation = 1;
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
	if( doWorkPartZCompensation )
	{
		doZCompensation = 1;
	}
#endif // FEATURE_WORK_PART_Z_COMPENSATION

	if( blockZ )
	{
		// do not perform any compensation in case the moving into z-direction is blocked
		doZCompensation = 0;
	}
	if( PrintLine::cur )
	{
		if( PrintLine::cur->isZMove() )
		{
			// do not peform any compensation while there is a "real" move into z-direction
			doZCompensation = 0;
		}
	}

	if( doZCompensation )
	{
		if( compensatedPositionCurrentStepsZ < compensatedPositionTargetStepsZ )
		{
			// we must move the heat bed do the bottom
			if( stepperDirection[Z_AXIS] >= 0 )
			{
				// here we shall move the z-axis only in case performQueueMove() is not moving into the other direction at the moment
				if( stepperDirection[Z_AXIS] == 0 )
				{
					// set the direction only in case it is not set already
					prepareBedDown();
					stepperDirection[Z_AXIS] = 1;
					HAL::delayMicroseconds( 1 );
				}
				startZStep( 1 );

#if STEPPER_HIGH_DELAY>0
				HAL::delayMicroseconds( STEPPER_HIGH_DELAY );
#endif // STEPPER_HIGH_DELAY>0

				endZStep();
				
				compensatedPositionCurrentStepsZ ++;
			}

			if( compensatedPositionCurrentStepsZ == compensatedPositionTargetStepsZ )
			{
				stepperDirection[Z_AXIS] = 0;

#if DEBUG_HEAT_BED_Z_COMPENSATION
				long	nDelay = micros() - g_nZCompensationUpdateTime;

				if( nDelay > g_nZCompensationDelayMax )		g_nZCompensationDelayMax = nDelay;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION
			}
		}
		else if( compensatedPositionCurrentStepsZ > compensatedPositionTargetStepsZ )
		{
			// we must move the heat bed to the top
			if( stepperDirection[Z_AXIS] <= 0 )
			{
				// here we shall move the z-axis only in case performQueueMove() is not moving into the other direction at the moment
				if( stepperDirection[Z_AXIS] == 0 )
				{
					// set the direction only in case it is not set already
					prepareBedUp();
					stepperDirection[Z_AXIS] = -1;
					HAL::delayMicroseconds( 1 );
				}
				startZStep( -1 );

#if STEPPER_HIGH_DELAY>0
				HAL::delayMicroseconds( STEPPER_HIGH_DELAY );
#endif // STEPPER_HIGH_DELAY>0

				endZStep();
				
				compensatedPositionCurrentStepsZ --;
			}

			if( compensatedPositionCurrentStepsZ == compensatedPositionTargetStepsZ )
			{
				stepperDirection[Z_AXIS] = 0;

#if DEBUG_HEAT_BED_Z_COMPENSATION
				long	nDelay = micros() - g_nZCompensationUpdateTime;

				if( nDelay > g_nZCompensationDelayMax )		g_nZCompensationDelayMax = nDelay;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION
			}
		}
	}

} // performZCompensation


void Printer::resetCompensatedPosition( void )
{
	if( compensatedPositionCurrentStepsZ )
	{
		queuePositionCurrentSteps[Z_AXIS] += compensatedPositionCurrentStepsZ;
		queuePositionLastSteps[Z_AXIS]	  += compensatedPositionCurrentStepsZ;
		queuePositionTargetSteps[Z_AXIS]  += compensatedPositionCurrentStepsZ;

		queuePositionCommandMM[Z_AXIS]	= 
		queuePositionLastMM[Z_AXIS]		= (float)(queuePositionLastSteps[Z_AXIS])*invAxisStepsPerMM[Z_AXIS];
	}

	compensatedPositionTargetStepsZ  = 0;
	compensatedPositionCurrentStepsZ = 0;

} // resetCompensatedPosition

#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
