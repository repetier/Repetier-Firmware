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

byte Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float Printer::axisStepsPerMM[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float Printer::invAxisStepsPerMM[4]; ///< Inverse of axisStepsPerMM for faster conversion
float Printer::maxFeedrate[4] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z}; ///< Maximum allowed feedrate.
float Printer::homingFeedrate[3] = {HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z};
#ifdef RAMP_ACCELERATION
//  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
long Printer::maxAccelerationMMPerSquareSecond[4] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long Printer::maxTravelAccelerationMMPerSquareSecond[4] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long Printer::maxPrintAccelerationStepsPerSquareSecond[4];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long Printer::maxTravelAccelerationStepsPerSquareSecond[4];
#endif
#if DRIVE_SYSTEM==3
long Printer::currentDeltaPositionSteps[4];
DeltaSegment segments[DELTA_CACHE_SIZE];
unsigned int delta_segment_write_pos = 0; // Position where we write the next cached delta move
byte lastMoveID = 0; // Last move ID
#endif
byte Printer::relativeCoordinateMode = false;  ///< Determines absolute (false) or relative Coordinates (true).
byte Printer::relativeExtruderCoordinateMode = false;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

long Printer::currentPositionSteps[4];
float Printer::currentPosition[3];
long Printer::destinationSteps[4];
long Printer::coordinateOffset[3] = {0,0,0};
byte Printer::flag0 = 0;
byte Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
byte Printer::stepsPerTimerCall = 1;
#if FEATURE_AUTOLEVEL
float Printer::autolevelTransformation[9]; ///< Transformation matrix
#endif
#if USE_OPS==1 || defined(USE_ADVANCE)
volatile  int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//  float extruderSpeed;              ///< Extruder speed in mm/s.
 byte Printer::minExtruderSpeed;            ///< Timer delay for start extruder speed
 byte Printer::maxExtruderSpeed;            ///< Timer delay for end extruder speed
 byte Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
 unsigned long Printer::interval;    ///< Last step duration in ticks.
#if USE_OPS==1
 bool Printer::filamentRetracted;           ///< Is the extruder filament retracted
#endif
 unsigned long Printer::timer;              ///< used for acceleration/deceleration timing
 unsigned long Printer::stepNumber;         ///< Step number in current move.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
 long Printer::advance_executed;             ///< Executed advance steps
#endif
 int Printer::advance_steps_set;
 unsigned int Printer::advance_lin_set;
#endif
#if DRIVE_SYSTEM==3
#ifdef STEP_COUNTER
 long Printer::countZSteps;					///< Count of steps from last position reset
#endif
long Printer::maxDeltaPositionSteps;
#endif
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z
 long Printer::stepsRemainingAtZHit;
#endif
#ifdef SOFTWARE_LEVELING
 long Printer::levelingP1[3];
 long Printer::levelingP2[3];
 long Printer::levelingP3[3];
#endif
#if USE_OPS==1
 int Printer::opsRetractSteps;              ///< Retract filament this much steps
 int Printer::opsPushbackSteps;             ///< Retract+extra distance for backsash
 float Printer::opsMinDistance;
 float Printer::opsRetractDistance;
 float Printer::opsRetractBacklash;
 byte Printer::opsMode;                     ///< OPS operation mode. 0 = Off, 1 = Classic, 2 = Fast
 float Printer::opsMoveAfter;               ///< Start move after opsModeAfter percent off full retract.
 int Printer::opsMoveAfterSteps;            ///< opsMoveAfter converted in steps (negative value!).
#endif
 float Printer::minimumSpeed;               ///< lowest allowed speed to keep integration error small
 long Printer::xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
 long Printer::yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
 long Printer::zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
 long Printer::xMinSteps;                   ///< For software endstops, limit of move in negative direction.
 long Printer::yMinSteps;                   ///< For software endstops, limit of move in negative direction.
 long Printer::zMinSteps;                   ///< For software endstops, limit of move in negative direction.
 float Printer::xLength;
 float Printer::xMin;
 float Printer::yLength;
 float Printer::yMin;
 float Printer::zLength;
 float Printer::zMin;
 float Printer::feedrate;                   ///< Last requested feedrate.
 int Printer::feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
 unsigned int Printer::extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
 float Printer::maxJerk;                    ///< Maximum allowed jerk in mm/s
 float Printer::maxZJerk;                   ///< Maximum allowed jerk in z direction in mm/s
 float Printer::offsetX;                     ///< X-offset for different extruder positions.
 float Printer::offsetY;                     ///< Y-offset for different extruder positions.
 unsigned int Printer::vMaxReached;         ///< Maximumu reached speed
 unsigned long Printer::msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
 float Printer::filamentPrinted;            ///< mm of filament printed since counting started
 byte Printer::waslasthalfstepping;         ///< Indicates if last move had halfstepping enabled
#if ENABLE_BACKLASH_COMPENSATION
 float Printer::backlashX;
 float Printer::backlashY;
 float Printer::backlashZ;
 byte Printer::backlashDir;
#endif
#ifdef DEBUG_STEPCOUNT
 long Printer::totalStepsRemaining;
#endif
#if FEATURE_MEMORY_POSITION
long Printer::memoryX;
long Printer::memoryY;
long Printer::memoryZ;
long Printer::memoryE;
#endif
#ifdef XY_GANTRY
char Printer::motorX;
char Printer::motorY;
#endif



void Printer::constrainDestinationCoords()
{
#if min_software_endstop_x == true
    if (destinationSteps[0] < xMinSteps) Printer::destinationSteps[0] = Printer::xMinSteps;
#endif
#if min_software_endstop_y == true
    if (destinationSteps[1] < yMinSteps) Printer::destinationSteps[1] = Printer::yMinSteps;
#endif
#if min_software_endstop_z == true
    if (destinationSteps[2] < zMinSteps) Printer::destinationSteps[2] = Printer::zMinSteps;
#endif

#if max_software_endstop_x == true
    if (destinationSteps[0] > Printer::xMaxSteps) Printer::destinationSteps[0] = Printer::xMaxSteps;
#endif
#if max_software_endstop_y == true
    if (destinationSteps[1] > Printer::yMaxSteps) Printer::destinationSteps[1] = Printer::yMaxSteps;
#endif
#if max_software_endstop_z == true
    if (destinationSteps[2] > Printer::zMaxSteps) Printer::destinationSteps[2] = Printer::zMaxSteps;
#endif
}
void Printer::updateDerivedParameter()
{
#if DRIVE_SYSTEM==3
    Printer::zMaxSteps = axisStepsPerMM[0]*(Printer::zLength - Printer::zMin);
    long cart[3], delta[3];
    cart[0] = cart[1] = 0;
    cart[2] = Printer::zMaxSteps;
    calculate_delta(cart, delta);
    Printer::maxDeltaPositionSteps = delta[0];
    Printer::xMaxSteps = (long)(axisStepsPerMM[0]*(Printer::xMin+Printer::xLength));
    Printer::yMaxSteps = (long)(axisStepsPerMM[1]*(Printer::yMin+Printer::yLength));
    Printer::xMinSteps = (long)(axisStepsPerMM[0]*Printer::xMin);
    Printer::yMinSteps = (long)(axisStepsPerMM[1]*Printer::yMin);
    Printer::zMinSteps = 0;
#else
    Printer::xMaxSteps = (long)(axisStepsPerMM[0]*(Printer::xMin+Printer::xLength));
    Printer::yMaxSteps = (long)(axisStepsPerMM[1]*(Printer::yMin+Printer::yLength));
    Printer::zMaxSteps = (long)(axisStepsPerMM[2]*(Printer::zMin+Printer::zLength));
    Printer::xMinSteps = (long)(axisStepsPerMM[0]*Printer::xMin);
    Printer::yMinSteps = (long)(axisStepsPerMM[1]*Printer::yMin);
    Printer::zMinSteps = (long)(axisStepsPerMM[2]*Printer::zMin);
    // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlashDir &= 7;
    if(Printer::backlashX!=0) Printer::backlashDir |= 8;
    if(Printer::backlashY!=0) Printer::backlashDir |= 16;
    if(Printer::backlashZ!=0) Printer::backlashDir |= 32;
#endif
#endif
    for(byte i=0; i<4; i++)
    {
        invAxisStepsPerMM[i] = 1.0f/axisStepsPerMM[i];
#ifdef RAMP_ACCELERATION
        /** Acceleration in steps/s^3 in printing mode.*/
        maxPrintAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
        /** Acceleration in steps/s^2 in movement mode.*/
        maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif
    }
    float accel = RMath::max(maxAccelerationMMPerSquareSecond[0],maxTravelAccelerationMMPerSquareSecond[0]);
    Printer::minimumSpeed = accel*sqrt(2.0f/(axisStepsPerMM[0]*accel));
    Printer::updateAdvanceFlags();
}
/**
  \brief Stop heater and stepper motors. Disable power,if possible.
*/
void Printer::kill(byte only_steppers)
{
    if(areAllSteppersDisabled() && only_steppers) return;
    setAllSteppersDiabled();
    disableXStepper();
    disableYStepper();
    disableZStepper();
    Extruder::disableCurrentExtruderMotor();
    if(!only_steppers)
    {
        for(byte i=0; i<NUM_TEMPERATURE_LOOPS; i++)
            Extruder::setTemperatureForExtruder(0,i);
        Extruder::setHeatedBedTemperature(0);
        UI_STATUS_UPD(UI_TEXT_KILLED);
#if defined(PS_ON_PIN) && PS_ON_PIN>-1
        //pinMode(PS_ON_PIN,INPUT);
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif
    }
    else UI_STATUS_UPD(UI_TEXT_STEPPER_DISABLED);
}
void Printer::updateAdvanceFlags()
{
    Printer::setAdvanceActivated(false);
#if defined(USE_ADVANCE)
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        if(extruder[i].advanceL!=0)
        {
            Printer::setAdvanceActivated(true);
        }
#ifdef ENABLE_QUADRATIC_ADVANCE
        if(extruder[i].advanceK!=0) Printer::setAdvanceActivated(true);
#endif
    }
#endif

}
void Printer::moveTo(float x,float y,float z,float e,float f)
{
    if(x!=IGNORE_COORDINATE)
        destinationSteps[0] = x*axisStepsPerMM[0];
    if(y!=IGNORE_COORDINATE)
        destinationSteps[1] = y*axisStepsPerMM[1];
    if(z!=IGNORE_COORDINATE)
        destinationSteps[2] = z*axisStepsPerMM[2];
    if(e!=IGNORE_COORDINATE)
        destinationSteps[3] = e*axisStepsPerMM[3];
    if(f!=IGNORE_COORDINATE)
        Printer::feedrate = f;
#if DRIVE_SYSTEM == 3
    PrintLine::split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
    PrintLine::queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
    updateCurrentPosition();
}
void Printer::moveToReal(float x,float y,float z,float e,float f)
{
#if FEATURE_AUTOLEVEL
    float startX,startY,startZ;
    realPosition(startX,startY,startZ);
    if(x==IGNORE_COORDINATE)
        x = startX;
    if(y==IGNORE_COORDINATE)
        y = startY;
    if(z==IGNORE_COORDINATE)
        z = startZ;
    currentPosition[0] = x;
    currentPosition[1] = y;
    currentPosition[2] = z;
    if(isAutolevelActive())
        transformToPrinter(x+Printer::offsetX,y+Printer::offsetY,z,x,y,z);
    else
#endif // FEATURE_AUTOLEVEL
    {
        x+= Printer::offsetX;
        y+= Printer::offsetY;
    }
    if(x!=IGNORE_COORDINATE)
        destinationSteps[0] = x*axisStepsPerMM[0]+coordinateOffset[0];
    if(y!=IGNORE_COORDINATE)
        destinationSteps[1] = y*axisStepsPerMM[1]+coordinateOffset[1];
    if(z!=IGNORE_COORDINATE)
        destinationSteps[2] = z*axisStepsPerMM[2]+coordinateOffset[2];
    if(e!=IGNORE_COORDINATE)
        destinationSteps[3] = e*axisStepsPerMM[3];
    if(f!=IGNORE_COORDINATE)
        Printer::feedrate = f;
#if DRIVE_SYSTEM == 3
    PrintLine::split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
    PrintLine::queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
}

void Printer::updateCurrentPosition()
{
    currentPosition[0] = (float)(currentPositionSteps[0]-coordinateOffset[0])*invAxisStepsPerMM[0];
    currentPosition[1] = (float)(currentPositionSteps[1]-coordinateOffset[1])*invAxisStepsPerMM[1];
    currentPosition[2] = (float)(currentPositionSteps[2]-coordinateOffset[2])*invAxisStepsPerMM[2];
#if FEATURE_AUTOLEVEL
    transformFromPrinter(currentPosition[0],currentPosition[1],currentPosition[2],currentPosition[0],currentPosition[1],currentPosition[2]);
#endif // FEATURE_AUTOLEVEL
    currentPosition[0] -= Printer::offsetX;
    currentPosition[1] -= Printer::offsetY;
    //Com::printArrayFLN(Com::tP,currentPosition,3,4);
}

/**
  \brief Sets the destination coordinates to values stored in com.

  For the computation of the destination, the following facts are considered:
  - Are units inches or mm.
  - Reltive or absolute positioning with special case only extruder relative.
  - Offset in x and y direction for multiple extruder support.
*/
byte Printer::setDestinationStepsFromGCode(GCode *com)
{
    register long p;
    if(!PrintLine::hasLines())
    {
        UI_STATUS(UI_TEXT_PRINTING);
    }
    float x,y,z;
    if(!relativeCoordinateMode)
    {
        if(!com->hasX()) x = currentPosition[0];
        else currentPosition[0] = x = convertToMM(com->X);
        if(!com->hasY()) y = currentPosition[1];
        else currentPosition[1] = y = convertToMM(com->Y);
        if(!com->hasZ()) z = currentPosition[2];
        else currentPosition[2] = z = convertToMM(com->Z);
    }
    else
    {
        if(com->hasX()) currentPosition[0] += (x = convertToMM(com->X));
        else x = 0;
        if(com->hasY()) currentPosition[1] += (y = convertToMM(com->Y));
        else y = 0;
        if(com->hasZ()) currentPosition[2] += (z = convertToMM(com->Z));
        else z = 0;
    }
#if FEATURE_AUTOLEVEL
    if(isAutolevelActive())
        transformToPrinter(x+Printer::offsetX,y+Printer::offsetY,z,x,y,z);
    else
#endif // FEATURE_AUTOLEVEL
    {
        if(!relativeCoordinateMode)
        {
            x+= Printer::offsetX;
            y+= Printer::offsetY;
        }
    }
    long xSteps = x*axisStepsPerMM[0];
    long ySteps = y*axisStepsPerMM[1];
    long zSteps = z*axisStepsPerMM[2];
    if(relativeCoordinateMode)
    {
        destinationSteps[0] = currentPositionSteps[0]+xSteps;
        destinationSteps[1] = currentPositionSteps[1]+ySteps;
        destinationSteps[2] = currentPositionSteps[2]+zSteps;
    }
    else
    {
        destinationSteps[0] = xSteps+coordinateOffset[0];
        destinationSteps[1] = ySteps+coordinateOffset[1];
        destinationSteps[2] = zSteps+coordinateOffset[2];
    }
    if(com->hasE() && !Printer::debugDryrun())
    {
        p = convertToMM(com->E*axisStepsPerMM[3]);
        if(relativeCoordinateMode || relativeExtruderCoordinateMode)
            destinationSteps[3] = currentPositionSteps[3]+p;
        else
            destinationSteps[3] = p;
    }
    else Printer::destinationSteps[3] = Printer::currentPositionSteps[3];
    if(com->hasF())
    {
        if(com->F < 1)
            Printer::feedrate = 1;
        else if(unitIsInches)
            Printer::feedrate = com->F*0.0042333f*(float)Printer::feedrateMultiply;  // Factor is 25.5/60/100
        else
            Printer::feedrate = com->F*(float)Printer::feedrateMultiply*0.00016666666f;
    }
    return !com->hasNoXYZ() || (com->hasE() && destinationSteps[3]!=currentPositionSteps[3]); // ignore unproductive moves
}

void Printer::setup()
{
#ifdef ANALYZER
// Channel->pin assignments
#if ANALYZER_CH0>=0
    SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1>=0
    SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2>=0
    SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3>=0
    SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4>=0
    SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5>=0
    SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6>=0
    SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7>=0
    SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && PS_ON_PIN>-1
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif

    //Initialize Step Pins
    SET_OUTPUT(X_STEP_PIN);
    SET_OUTPUT(Y_STEP_PIN);
    SET_OUTPUT(Z_STEP_PIN);

    //Initialize Dir Pins
#if X_DIR_PIN>-1
    SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN>-1
    SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN>-1
    SET_OUTPUT(Z_DIR_PIN);
#endif

    //Steppers default to disabled.
#if X_ENABLE_PIN > -1
    SET_OUTPUT(X_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
#endif
#if Y_ENABLE_PIN > -1
    SET_OUTPUT(Y_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
#endif
#if Z_ENABLE_PIN > -1
    SET_OUTPUT(Z_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
#endif
#if FEATURE_TWO_XSTEPPER
    SET_OUTPUT(X2_STEP_PIN);
    SET_OUTPUT(X2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(X2_ENABLE_PIN);
    if(!X_ENABLE_ON) WRITE(X2_ENABLE_PIN,HIGH);
#endif
#endif

#if FEATURE_TWO_YSTEPPER
    SET_OUTPUT(Y2_STEP_PIN);
    SET_OUTPUT(Y2_DIR_PIN);
#if Y2_ENABLE_PIN > -1
    SET_OUTPUT(Y2_ENABLE_PIN);
    if(!Y_ENABLE_ON) WRITE(Y2_ENABLE_PIN,HIGH);
#endif
#endif

#if FEATURE_TWO_ZSTEPPER
    SET_OUTPUT(Z2_STEP_PIN);
    SET_OUTPUT(Z2_DIR_PIN);
#if X2_ENABLE_PIN > -1
    SET_OUTPUT(Z2_ENABLE_PIN);
    if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
#endif
#endif

    //endstop pullups
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN>-1
    SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
    PULLUP(X_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif
#endif
#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN>-1
    SET_INPUT(Y_MIN_PIN);
#if ENDSTOP_PULLUP_Y_MIN
    PULLUP(Y_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif
#endif
#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN>-1
    SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
    PULLUP(Z_MIN_PIN,HIGH);
#endif
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN>-1
    SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
    PULLUP(X_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN>-1
    SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
    PULLUP(Y_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif
#endif
#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN>-1
    SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
    PULLUP(Z_MAX_PIN,HIGH);
#endif
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif
#endif
#if FEATURE_Z_PROBE && Z_PROBE_PIN>-1
    SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
    PULLUP(Z_PROBE_PIN,HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE
#if FAN_PIN>-1
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN,LOW);
#endif
#if FAN_BOARD_PIN>-1
    SET_OUTPUT(FAN_BOARD_PIN);
    WRITE(FAN_BOARD_PIN,LOW);
#endif
#if EXT0_HEATER_PIN>-1
    SET_OUTPUT(EXT0_HEATER_PIN);
    WRITE(EXT0_HEATER_PIN,LOW);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_HEATER_PIN);
    WRITE(EXT1_HEATER_PIN,LOW);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    SET_OUTPUT(EXT2_HEATER_PIN);
    WRITE(EXT2_HEATER_PIN,LOW);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    SET_OUTPUT(EXT3_HEATER_PIN);
    WRITE(EXT3_HEATER_PIN,LOW);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    SET_OUTPUT(EXT4_HEATER_PIN);
    WRITE(EXT4_HEATER_PIN,LOW);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    SET_OUTPUT(EXT5_HEATER_PIN);
    WRITE(EXT5_HEATER_PIN,LOW);
#endif

#ifdef XY_GANTRY
    Printer::motorX = 0;
    Printer::motorY = 0;
#endif

#if STEPPER_CURRENT_CONTROL!=CURRENT_CONTROL_MANUAL
    current_control_init(); // Set current if it is firmware controlled
#endif
    microstep_init();
#if FEATURE_AUTOLEVEL
    Printer::resetTransformationMatrix(true);
#endif // FEATURE_AUTOLEVEL
#if USE_OPS==1
    Printer::opsMode = OPS_MODE;
    Printer::opsMinDistance = OPS_MIN_DISTANCE;
    Printer::opsRetractDistance = OPS_RETRACT_DISTANCE;
    Printer::opsRetractBacklash = OPS_RETRACT_BACKLASH;
    Printer::opsMoveAfter = OPS_MOVE_AFTER;
    Printer::filamentRetracted = false;
#endif
    Printer::feedrate = 50; ///< Current feedrate in mm/s.
    Printer::feedrateMultiply = 100;
    Printer::extrudeMultiply = 100;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    Printer::advance_executed = 0;
#endif
    Printer::advance_steps_set = 0;
    Printer::advance_lin_set = 0;
#endif
    for(byte i=0; i<NUM_EXTRUDER+3; i++) pwm_pos[i]=0;
    Printer::currentPositionSteps[0] = Printer::currentPositionSteps[1] = Printer::currentPositionSteps[2] = Printer::currentPositionSteps[3] = 0;
#if DRIVE_SYSTEM==3
    calculate_delta(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
#endif
    Printer::maxJerk = MAX_JERK;
    Printer::maxZJerk = MAX_ZJERK;
    Printer::interval = 5000;
    Printer::stepsPerTimerCall = 1;
    Printer::msecondsPrinting = 0;
    Printer::filamentPrinted = 0;
    Printer::flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
    Printer::xLength = X_MAX_LENGTH;
    Printer::yLength = Y_MAX_LENGTH;
    Printer::zLength = Z_MAX_LENGTH;
    Printer::xMin = X_MIN_POS;
    Printer::yMin = Y_MIN_POS;
    Printer::zMin = Z_MIN_POS;
    Printer::waslasthalfstepping = 0;
#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlashX = X_BACKLASH;
    Printer::backlashY = Y_BACKLASH;
    Printer::backlashZ = Z_BACKLASH;
    Printer::backlashDir = 0;
#endif
#if defined(USE_ADVANCE)
    Printer::extruderStepsNeeded = 0;
#endif
    EEPROM::initBaudrate();
    HAL::serialSetBaudrate(baudrate);
    Com::printFLN(Com::tStart);
    UI_INITIALIZE;
    HAL::showStartReason();
    Extruder::initExtruder();
    EEPROM::init(); // Read settings from eeprom if wanted
    Printer::updateDerivedParameter();

#if SDSUPPORT
    sd.initsd();

#endif
    Commands::checkFreeMemory();
    Commands::writeLowestFreeRAM();
    HAL::setupTimer();
}

void Printer::defaultLoopActions()
{
    //check heater every n milliseconds
    Commands::checkForPeriodicalActions();
    UI_MEDIUM; // do check encoder
    unsigned long curtime = HAL::timeInMilliseconds();
    if(PrintLine::hasLines())
        previous_millis_cmd = curtime;
    if(max_inactive_time!=0 && (curtime-previous_millis_cmd) >  max_inactive_time ) Printer::kill(false);
    if(stepper_inactive_time!=0 && (curtime-previous_millis_cmd) >  stepper_inactive_time )
        Printer::kill(true);
#if defined(SDCARDDETECT) && SDCARDDETECT>-1 && defined(SDSUPPORT) && SDSUPPORT
    sd.automount();
#endif
    //void finishNextSegment();
    DEBUG_MEMORY;

}

#if DRIVE_SYSTEM==3
void delta_move_to_top_endstops(float feedrate)
{
    long up_steps = Printer::zMaxSteps;
    for (byte i=0; i<3; i++)
        Printer::currentPositionSteps[i] = 0;
    calculate_delta(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
    PrintLine::moveRelativeDistanceInSteps(0,0,Printer::zMaxSteps*ENDSTOP_Z_BACK_MOVE,0,feedrate, true, true);
}
void Printer::homeXAxis()
{
    Printer::destinationSteps[0] = 0;
    split_delta_move(true,false,false);
}
void Printer::homeYAxis()
{
    Printer::destinationSteps[1] = 0;
    split_delta_move(true,false,false);
}
void Printer::homeZAxis()
{
    delta_move_to_top_endstops(Printer::homingFeedrate[0]);
    PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[0]*-ENDSTOP_Z_BACK_MOVE,0,Printer::homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
    delta_move_to_top_endstops(Printer::homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR);
    Printer::currentPositionSteps[0] = 0;
    Printer::currentPositionSteps[1] = 0;
    Printer::currentPositionSteps[2] = Printer::zMaxSteps;
    Printer::coordinateOffset[0] = 0;
    Printer::coordinateOffset[1] = 0;
    Printer::coordinateOffset[2] = 0;
    calculate_delta(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
    Printer::maxDeltaPositionSteps = Printer::currentDeltaPositionSteps[0];
}

void Printer::homeAxis(bool xaxis,bool yaxis,bool zaxis)
{
    long steps;
    bool homeallaxis = (xaxis && yaxis && zaxis) || (!xaxis && !yaxis && !zaxis);
    if (X_MAX_PIN > -1 && Y_MAX_PIN > -1 && Z_MAX_PIN > -1 && MAX_HARDWARE_ENDSTOP_X & MAX_HARDWARE_ENDSTOP_Y && MAX_HARDWARE_ENDSTOP_Z)
    {
        UI_STATUS_UPD(UI_TEXT_HOME_DELTA);
        // Homing Z axis means that you must home X and Y
        if (homeallaxis || zaxis)
        {
            homeZAxis();
        }
        else
        {
            if (xaxis) Printer::destinationSteps[0] = 0;
            if (yaxis) Printer::destinationSteps[1] = 0;
            PrintLine::split_delta_move(true,false,false);
        }
        Printer::countZSteps = 0;
        UI_CLEAR_STATUS
    }
}
#else
void Printer::homeXAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_X && X_MIN_PIN > -1 && X_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_X && X_MAX_PIN > -1 && X_HOME_DIR==1))
    {
        UI_STATUS_UPD(UI_TEXT_HOME_X);
        steps = (Printer::xMaxSteps-Printer::xMinSteps) * X_HOME_DIR;
        Printer::currentPositionSteps[0] = -steps;
        PrintLine::moveRelativeDistanceInSteps(2*steps,0,0,0,homingFeedrate[0],true,true);
        Printer::currentPositionSteps[0] = 0;
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*-ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*2*ENDSTOP_X_BACK_MOVE * X_HOME_DIR,0,0,0,homingFeedrate[0]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_X_BACK_ON_HOME)
        if(ENDSTOP_X_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(axisStepsPerMM[0]*-ENDSTOP_X_BACK_ON_HOME * X_HOME_DIR,0,0,0,homingFeedrate[0],true,false);
#endif
        long offX = 0;
#if NUM_EXTRUDER>1
        for(byte i=0; i<NUM_EXTRUDER; i++) offX = max(offX,extruder[i].xOffset);
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        Printer::currentPositionSteps[0] = (X_HOME_DIR == -1) ? Printer::xMinSteps-offX : Printer::xMaxSteps+offX;
#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps((Extruder::current->xOffset-offX) * X_HOME_DIR,0,0,0,homingFeedrate[0],true,false);
#endif
    }
}
void Printer::homeYAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Y && Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Y && Y_MAX_PIN > -1 && Y_HOME_DIR==1))
    {
        UI_STATUS_UPD(UI_TEXT_HOME_Y);
        steps = (Printer::yMaxSteps-Printer::yMinSteps) * Y_HOME_DIR;
        currentPositionSteps[1] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,2*steps,0,0,homingFeedrate[1],true,true);
        currentPositionSteps[1] = 0;
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[1]*-ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[1]*2*ENDSTOP_Y_BACK_MOVE * Y_HOME_DIR,0,0,homingFeedrate[1]/ENDSTOP_X_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Y_BACK_ON_HOME)
        if(ENDSTOP_Y_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,axisStepsPerMM[1]*-ENDSTOP_Y_BACK_ON_HOME * Y_HOME_DIR,0,0,homingFeedrate[1],true,false);
#endif
        long offY = 0;
#if NUM_EXTRUDER>1
        for(byte i=0; i<NUM_EXTRUDER; i++) offY = max(offY,extruder[i].yOffset);
        // Reposition extruder that way, that all extruders can be selected at home pos.
#endif
        Printer::currentPositionSteps[1] = (Y_HOME_DIR == -1) ? Printer::yMinSteps-offY : Printer::yMaxSteps+offY;
#if NUM_EXTRUDER>1
        PrintLine::moveRelativeDistanceInSteps(0,(Extruder::current->yOffset-offY) * Y_HOME_DIR,0,0,homingFeedrate[1],true,false);
#endif
    }
}
void Printer::homeZAxis()
{
    long steps;
    if ((MIN_HARDWARE_ENDSTOP_Z && Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (MAX_HARDWARE_ENDSTOP_Z && Z_MAX_PIN > -1 && Z_HOME_DIR==1))
    {
        UI_STATUS_UPD(UI_TEXT_HOME_Z);
        steps = (Printer::zMaxSteps-Printer::zMinSteps) * Z_HOME_DIR;
        currentPositionSteps[2] = -steps;
        PrintLine::moveRelativeDistanceInSteps(0,0,2*steps,0,homingFeedrate[2],true,true);
        currentPositionSteps[2] = 0;
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[2]*-ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homingFeedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,false);
        PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[2]*2*ENDSTOP_Z_BACK_MOVE * Z_HOME_DIR,0,homingFeedrate[2]/ENDSTOP_Z_RETEST_REDUCTION_FACTOR,true,true);
#if defined(ENDSTOP_Z_BACK_ON_HOME)
        if(ENDSTOP_Z_BACK_ON_HOME > 0)
            PrintLine::moveRelativeDistanceInSteps(0,0,axisStepsPerMM[2]*-ENDSTOP_Z_BACK_ON_HOME * Z_HOME_DIR,0,homingFeedrate[2],true,false);
#endif
        currentPositionSteps[2] = (Z_HOME_DIR == -1) ? Printer::zMinSteps : Printer::zMaxSteps;
    }
}
void Printer::homeAxis(bool xaxis,bool yaxis,bool zaxis)
{
    float startX,startY,startZ;
    float coordX,coordY,coordZ;
#if FEATURE_AUTOLEVEL
    if(isAutolevelActive())
        transformFromPrinter(coordinateOffset[0]*invAxisStepsPerMM[0],coordinateOffset[1]*invAxisStepsPerMM[1],
                             coordinateOffset[2]*invAxisStepsPerMM[2],coordX,coordY,coordZ);
    else
#endif
    {
        coordX = coordinateOffset[0]*invAxisStepsPerMM[0];
        coordY = coordinateOffset[1]*invAxisStepsPerMM[1];
        coordZ = coordinateOffset[2]*invAxisStepsPerMM[2];
    }
    realPosition(startX,startY,startZ);
#if !defined(HOMING_ORDER)
#define HOMING_ORDER HOME_ORDER_XYZ
#endif
#if HOMING_ORDER==HOME_ORDER_XYZ
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER==HOME_ORDER_XZY
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER==HOME_ORDER_YXZ
    if(yaxis) homeYAxis();
    if(xaxis) homeXAxis();
    if(zaxis) homeZAxis();
#elif HOMING_ORDER==HOME_ORDER_YZX
    if(yaxis) homeYAxis();
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
#elif HOMING_ORDER==HOME_ORDER_ZXY
    if(zaxis) homeZAxis();
    if(xaxis) homeXAxis();
    if(yaxis) homeYAxis();
#elif HOMING_ORDER==HOME_ORDER_ZYX
    if(zaxis) homeZAxis();
    if(yaxis) homeYAxis();
    if(xaxis) homeXAxis();
#endif
    if(xaxis)
    {
        coordX = 0;
        if(X_HOME_DIR<0) startX = Printer::xMin;
        else startX = Printer::xMin+Printer::xLength;
    }
    if(yaxis)
    {
        coordY = 0;
        if(Y_HOME_DIR<0) startY = Printer::yMin;
        else startY = Printer::yMin+Printer::yLength;
    }
    if(zaxis)
    {
        coordZ = 0;
        if(Z_HOME_DIR<0) startZ = Printer::zMin;
        else startZ = Printer::zMin+Printer::zLength;
    }
#if FEATURE_AUTOLEVEL
    if(Printer::isAutolevelActive())
        Printer::transformToPrinter(coordX,coordY,coordZ,coordX,coordY,coordZ);
#endif
    coordinateOffset[0] = (long)(coordX*axisStepsPerMM[0]);
    coordinateOffset[1] = (long)(coordY*axisStepsPerMM[1]);
    coordinateOffset[2] = (long)(coordZ*axisStepsPerMM[2]);
    updateCurrentPosition();
    moveToReal(startX,startY,startZ,IGNORE_COORDINATE,homingFeedrate[0]);
    UI_CLEAR_STATUS
}
#endif



void Printer::setAutolevelActive(bool on)
{
#if FEATURE_AUTOLEVEL
    if(on == isAutolevelActive()) return;
    flag0 = (on ? flag0 | PRINTER_FLAG0_AUTOLEVEL_ACTIVE : flag0 & ~PRINTER_FLAG0_AUTOLEVEL_ACTIVE);
    if(on)
    {
        Com::printInfoFLN(Com::tAutolevelEnabled);
    }
    else
    {
        Com::printInfoFLN(Com::tAutolevelDisabled);
    }
    Printer::updateCurrentPosition();
#endif // FEATURE_AUTOLEVEL    if(isAutolevelActive()==on) return;
}
#if MAX_HARDWARE_ENDSTOP_Z
float Printer::runZMaxProbe()
{
    Commands::waitUntilEndOfAllMoves();
    long probeDepth = 2*(Printer::zMaxSteps-Printer::zMinSteps);
    stepsRemainingAtZHit = -1;
    setZProbingActive(true);
    PrintLine::moveRelativeDistanceInSteps(0,0,probeDepth,0,Z_PROBE_SPEED,true,true);
    if(stepsRemainingAtZHit<0)
    {
        Com::printErrorFLN(Com::tZProbeFailed);
        return -1;
    }
    setZProbingActive(false);
    currentPositionSteps[2] -= stepsRemainingAtZHit;
    probeDepth -= stepsRemainingAtZHit;
    float distance = (float)probeDepth*invAxisStepsPerMM[2];
    Com::printF(Com::tZProbeMax,distance);
    Com::printF(Com::tSpaceXColon,realXPosition());
    Com::printFLN(Com::tSpaceYColon,realYPosition());
    PrintLine::moveRelativeDistanceInSteps(0,0,-probeDepth,0,Z_PROBE_SPEED,true,true);
    return distance;
}
#endif

#if FEATURE_Z_PROBE
float Printer::runZProbe(bool first,bool last)
{
    if(first)
    {
        GCode::executeFString(Com::tZProbeStartScript);
        Printer::offsetX -= EEPROM::zProbeXOffset();
        Printer::offsetY -= EEPROM::zProbeYOffset();
    }
    Commands::waitUntilEndOfAllMoves();
    long probeDepth = 2*(Printer::zMaxSteps-Printer::zMinSteps);
    stepsRemainingAtZHit = -1;
    long offx = axisStepsPerMM[0]*EEPROM::zProbeXOffset();
    long offy = axisStepsPerMM[0]*EEPROM::zProbeYOffset();
    PrintLine::moveRelativeDistanceInSteps(-offx,-offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
    waitForZProbeStart();
    setZProbingActive(true);
    PrintLine::moveRelativeDistanceInSteps(0,0,-probeDepth,0,Z_PROBE_SPEED,true,true);
    if(stepsRemainingAtZHit<0)
    {
        Com::printErrorFLN(Com::tZProbeFailed);
        return -1;
    }
    setZProbingActive(false);
    currentPositionSteps[2] += stepsRemainingAtZHit;
    probeDepth -= stepsRemainingAtZHit;
    float distance = (float)probeDepth*invAxisStepsPerMM[2]+EEPROM::zProbeHeight();
    Com::printF(Com::tZProbe,distance);
    Com::printF(Com::tSpaceXColon,realXPosition());
    Com::printFLN(Com::tSpaceYColon,realYPosition());
    PrintLine::moveRelativeDistanceInSteps(0,0,probeDepth,0,Z_PROBE_SPEED,true,true);
    PrintLine::moveRelativeDistanceInSteps(offx,offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
    if(last)
    {
        GCode::executeFString(Com::tZProbeEndScript);
        Printer::offsetX += EEPROM::zProbeXOffset();
        Printer::offsetY += EEPROM::zProbeYOffset();
    }
    return distance;
}
void Printer::waitForZProbeStart()
{
#if Z_PROBE_WAIT_BEFORE_TEST
    if(isZProbeHit()) return;
#if UI_DISPLAY_TYPE!=0
    uid.setStatusP(Com::tHitZProbe);
    uid.refreshPage();
#endif
    while(!isZProbeHit())
    {
        defaultLoopActions();
    }
    HAL::delayMilliseconds(30);
    while(isZProbeHit())
    {
        defaultLoopActions();
    }
    HAL::delayMilliseconds(30);
    UI_CLEAR_STATUS;
#endif
}
#if FEATURE_AUTOLEVEL
void Printer::transformToPrinter(float x,float y,float z,float &transX,float &transY,float &transZ)
{
    transX = x*autolevelTransformation[0]+y*autolevelTransformation[3]+z*autolevelTransformation[6];
    transY = x*autolevelTransformation[1]+y*autolevelTransformation[4]+z*autolevelTransformation[7];
    transZ = x*autolevelTransformation[2]+y*autolevelTransformation[5]+z*autolevelTransformation[8];
}

void Printer::transformFromPrinter(float x,float y,float z,float &transX,float &transY,float &transZ)
{
    transX = x*autolevelTransformation[0]+y*autolevelTransformation[1]+z*autolevelTransformation[2];
    transY = x*autolevelTransformation[3]+y*autolevelTransformation[4]+z*autolevelTransformation[5];
    transZ = x*autolevelTransformation[6]+y*autolevelTransformation[7]+z*autolevelTransformation[8];
}

void Printer::resetTransformationMatrix(bool silent)
{
    autolevelTransformation[0] = autolevelTransformation[4] = autolevelTransformation[8] = 1;
    autolevelTransformation[1] = autolevelTransformation[2] = autolevelTransformation[3] =
                                     autolevelTransformation[5] = autolevelTransformation[6] = autolevelTransformation[7] = 0;
    if(!silent)
        Com::printInfoFLN(Com::tAutolevelReset);
}

void Printer::buildTransformationMatrix(float h1,float h2,float h3)
{
    float ax = EEPROM::zProbeX2()-EEPROM::zProbeX1();
    float ay = EEPROM::zProbeY2()-EEPROM::zProbeY1();
    float az = h1-h2;
    float bx = EEPROM::zProbeX3()-EEPROM::zProbeX1();
    float by = EEPROM::zProbeY3()-EEPROM::zProbeY1();
    float bz = h1-h3;
    // First z direction
    autolevelTransformation[6] = ay*bz-az*by;
    autolevelTransformation[7] = az*bx-ax*bz;
    autolevelTransformation[8] = ax*by-ay*bx;
    float len = sqrt(autolevelTransformation[6]*autolevelTransformation[6]+autolevelTransformation[7]*autolevelTransformation[7]+autolevelTransformation[8]*autolevelTransformation[8]);
    if(autolevelTransformation[8]<0) len = -len;
    autolevelTransformation[6] /= len;
    autolevelTransformation[7] /= len;
    autolevelTransformation[8] /= len;
    autolevelTransformation[3] = 0;
    autolevelTransformation[4] = autolevelTransformation[8];
    autolevelTransformation[5] = -autolevelTransformation[7];
    // cross(y,z)
    autolevelTransformation[0] = autolevelTransformation[4]*autolevelTransformation[8]-autolevelTransformation[5]*autolevelTransformation[7];
    autolevelTransformation[1] = autolevelTransformation[5]*autolevelTransformation[6]-autolevelTransformation[3]*autolevelTransformation[8];
    autolevelTransformation[2] = autolevelTransformation[3]*autolevelTransformation[7]-autolevelTransformation[4]*autolevelTransformation[6];
    len = sqrt(autolevelTransformation[0]*autolevelTransformation[0]+autolevelTransformation[2]*autolevelTransformation[2]);
    autolevelTransformation[0] /= len;
    autolevelTransformation[2] /= len;
    len = sqrt(autolevelTransformation[4]*autolevelTransformation[4]+autolevelTransformation[5]*autolevelTransformation[5]);
    autolevelTransformation[4] /= len;
    autolevelTransformation[5] /= len;
    Com::printArrayFLN(Com::tInfo,autolevelTransformation,9,5);
}
#endif

#endif
