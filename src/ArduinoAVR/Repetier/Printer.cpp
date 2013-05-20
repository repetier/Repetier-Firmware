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

byte unit_inches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float axis_steps_per_unit[4] = {XAXIS_STEPS_PER_MM,YAXIS_STEPS_PER_MM,ZAXIS_STEPS_PER_MM,1}; ///< Number of steps per mm needed.
float inv_axis_steps_per_unit[4]; ///< Inverse of axis_steps_per_unit for faster conversion
float max_feedrate[4] = {MAX_FEEDRATE_X, MAX_FEEDRATE_Y, MAX_FEEDRATE_Z}; ///< Maximum allowed feedrate.
float homing_feedrate[3] = {HOMING_FEEDRATE_X, HOMING_FEEDRATE_Y, HOMING_FEEDRATE_Z};
#ifdef RAMP_ACCELERATION
//  float max_start_speed_units_per_second[4] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
long max_acceleration_units_per_sq_second[4] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long max_travel_acceleration_units_per_sq_second[4] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y,MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long axis_steps_per_sqr_second[4];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long axis_travel_steps_per_sqr_second[4];
#endif
#if DRIVE_SYSTEM==3
long Printer::currentDeltaPositionSteps[4];
DeltaSegment segments[DELTA_CACHE_SIZE];
unsigned int delta_segment_write_pos = 0; // Position where we write the next cached delta move
volatile unsigned int  delta_segment_count = 0; // Number of delta moves cached 0 = nothing in cache
byte lastMoveID = 0; // Last move ID
#endif
byte relative_mode = false;  ///< Determines absolute (false) or relative Coordinates (true).
byte relative_mode_e = false;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

Printer printer;
long Printer::xMaxSteps;
long Printer::yMaxSteps;
long Printer::zMaxSteps;
long Printer::xMinSteps;
long Printer::yMinSteps;
long Printer::zMinSteps;
unsigned long Printer::interval;
long Printer::currentPositionSteps[4];
float Printer::currentPosition[3];
long Printer::destinationSteps[4];
long Printer::coordinateOffset[3] = {0,0,0};
byte Printer::flag0 = 0;
byte Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
byte Printer::stepsPerTimerCall = 1;
#if FEATURE_Z_PROBE || MAX_HARDWARE_ENDSTOP_Z
long Printer::stepsRemainingAtZHit;
#endif
#if FEATURE_AUTOLEVEL
float Printer::autolevelTransformation[9]; ///< Transformation matrix
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
    if (destinationSteps[0] > printer.xMaxSteps) Printer::destinationSteps[0] = printer.xMaxSteps;
#endif
#if max_software_endstop_y == true
    if (destinationSteps[1] > printer.yMaxSteps) Printer::destinationSteps[1] = printer.yMaxSteps;
#endif
#if max_software_endstop_z == true
    if (destinationSteps[2] > printer.zMaxSteps) Printer::destinationSteps[2] = printer.zMaxSteps;
#endif
}
void Printer::updateDerivedParameter()
{
#if DRIVE_SYSTEM==3
    printer.zMaxSteps = axis_steps_per_unit[0]*(printer.zLength - printer.zMin);
    long cart[3], delta[3];
    cart[0] = cart[1] = 0;
    cart[2] = printer.zMaxSteps;
    calculate_delta(cart, delta);
    printer.maxDeltaPositionSteps = delta[0];
    printer.xMaxSteps = (long)(axis_steps_per_unit[0]*(printer.xMin+printer.xLength));
    printer.yMaxSteps = (long)(axis_steps_per_unit[1]*(printer.yMin+printer.yLength));
    printer.xMinSteps = (long)(axis_steps_per_unit[0]*printer.xMin);
    printer.yMinSteps = (long)(axis_steps_per_unit[1]*printer.yMin);
    printer.zMinSteps = 0;
#else
    printer.xMaxSteps = (long)(axis_steps_per_unit[0]*(printer.xMin+printer.xLength));
    printer.yMaxSteps = (long)(axis_steps_per_unit[1]*(printer.yMin+printer.yLength));
    printer.zMaxSteps = (long)(axis_steps_per_unit[2]*(printer.zMin+printer.zLength));
    printer.xMinSteps = (long)(axis_steps_per_unit[0]*printer.xMin);
    printer.yMinSteps = (long)(axis_steps_per_unit[1]*printer.yMin);
    printer.zMinSteps = (long)(axis_steps_per_unit[2]*printer.zMin);
    // For which directions do we need backlash compensation
#if ENABLE_BACKLASH_COMPENSATION
    printer.backlashDir &= 7;
    if(printer.backlashX!=0) printer.backlashDir |= 8;
    if(printer.backlashY!=0) printer.backlashDir |= 16;
    if(printer.backlashZ!=0) printer.backlashDir |= 32;
#endif
#endif
    for(byte i=0; i<4; i++)
    {
        inv_axis_steps_per_unit[i] = 1.0f/axis_steps_per_unit[i];
#ifdef RAMP_ACCELERATION
        /** Acceleration in steps/s^3 in printing mode.*/
        axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        /** Acceleration in steps/s^2 in movement mode.*/
        axis_travel_steps_per_sqr_second[i] = max_travel_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
#endif
    }
    float accel = RMath::max(max_acceleration_units_per_sq_second[0],max_travel_acceleration_units_per_sq_second[0]);
    printer.minimumSpeed = accel*sqrt(2.0f/(axis_steps_per_unit[0]*accel));
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
    printer.setAdvanceActivated(false);
#if defined(USE_ADVANCE)
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        if(extruder[i].advanceL!=0)
        {
            printer.setAdvanceActivated(true);
        }
#ifdef ENABLE_QUADRATIC_ADVANCE
        if(extruder[i].advanceK!=0) printer.setAdvanceActivated(true);
#endif
    }
#endif

}
void Printer::moveTo(float x,float y,float z,float e,float f)
{
    if(x!=IGNORE_COORDINATE)
        destinationSteps[0] = x*axis_steps_per_unit[0];
    if(y!=IGNORE_COORDINATE)
        destinationSteps[1] = y*axis_steps_per_unit[1];
    if(z!=IGNORE_COORDINATE)
        destinationSteps[2] = z*axis_steps_per_unit[2];
    if(e!=IGNORE_COORDINATE)
        destinationSteps[3] = e*axis_steps_per_unit[3];
    if(f!=IGNORE_COORDINATE)
        printer.feedrate = f;
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
        transformToPrinter(x+printer.offsetX,y+printer.offsetY,z,x,y,z);
    else
    {
        x+= printer.offsetX;
        y+= printer.offsetY;
    }
#endif // FEATURE_AUTOLEVEL
    if(x!=IGNORE_COORDINATE)
        destinationSteps[0] = x*axis_steps_per_unit[0]+coordinateOffset[0];
    if(y!=IGNORE_COORDINATE)
        destinationSteps[1] = y*axis_steps_per_unit[1]+coordinateOffset[1];
    if(z!=IGNORE_COORDINATE)
        destinationSteps[2] = z*axis_steps_per_unit[2]+coordinateOffset[2];
    if(e!=IGNORE_COORDINATE)
        destinationSteps[3] = e*axis_steps_per_unit[3];
    if(f!=IGNORE_COORDINATE)
        printer.feedrate = f;
#if DRIVE_SYSTEM == 3
    PrintLine::split_delta_move(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
    PrintLine::queue_move(ALWAYS_CHECK_ENDSTOPS,true);
#endif
}

void Printer::updateCurrentPosition()
{
    currentPosition[0] = (float)(currentPositionSteps[0]-coordinateOffset[0])*inv_axis_steps_per_unit[0];
    currentPosition[1] = (float)(currentPositionSteps[1]-coordinateOffset[1])*inv_axis_steps_per_unit[1];
    currentPosition[2] = (float)(currentPositionSteps[2]-coordinateOffset[2])*inv_axis_steps_per_unit[2];
#if FEATURE_AUTOLEVEL
    transformFromPrinter(currentPosition[0],currentPosition[1],currentPosition[2],currentPosition[0],currentPosition[1],currentPosition[2]);
#endif // FEATURE_AUTOLEVEL
    currentPosition[0] -= printer.offsetX;
    currentPosition[1] -= printer.offsetY;
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
    if(!relative_mode)
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
        transformToPrinter(x+printer.offsetX,y+printer.offsetY,z,x,y,z);
    else
#endif // FEATURE_AUTOLEVEL
    {
        if(!relative_mode) {
            x+= printer.offsetX;
            y+= printer.offsetY;
        }
    }
    long xSteps = x*axis_steps_per_unit[0];
    long ySteps = y*axis_steps_per_unit[1];
    long zSteps = z*axis_steps_per_unit[2];
    if(relative_mode)
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
        p = convertToMM(com->E*axis_steps_per_unit[3]);
        if(relative_mode || relative_mode_e)
            destinationSteps[3] = currentPositionSteps[3]+p;
        else
            destinationSteps[3] = p;
    }
    else Printer::destinationSteps[3] = Printer::currentPositionSteps[3];
    if(com->hasF())
    {
        if(com->F < 1)
            printer.feedrate = 1;
        else if(unit_inches)
            printer.feedrate = com->F*0.0042333f*(float)printer.feedrateMultiply;  // Factor is 25.5/60/100
        else
            printer.feedrate = com->F*(float)printer.feedrateMultiply*0.00016666666f;
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
    printer.motorX = 0;
    printer.motorY = 0;
#endif

#if STEPPER_CURRENT_CONTROL!=CURRENT_CONTROL_MANUAL
    current_control_init(); // Set current if it is firmware controlled
#endif
    microstep_init();
#if FEATURE_AUTOLEVEL
    Printer::resetTransformationMatrix(true);
#endif // FEATURE_AUTOLEVEL
#if USE_OPS==1
    printer.opsMode = OPS_MODE;
    printer.opsMinDistance = OPS_MIN_DISTANCE;
    printer.opsRetractDistance = OPS_RETRACT_DISTANCE;
    printer.opsRetractBacklash = OPS_RETRACT_BACKLASH;
    printer.opsMoveAfter = OPS_MOVE_AFTER;
    printer.filamentRetracted = false;
#endif
    printer.feedrate = 50; ///< Current feedrate in mm/s.
    printer.feedrateMultiply = 100;
    printer.extrudeMultiply = 100;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    printer.advance_executed = 0;
#endif
    printer.advance_steps_set = 0;
    printer.advance_lin_set = 0;
#endif
    for(byte i=0; i<NUM_EXTRUDER+3; i++) pwm_pos[i]=0;
    Printer::currentPositionSteps[0] = Printer::currentPositionSteps[1] = Printer::currentPositionSteps[2] = Printer::currentPositionSteps[3] = 0;
#if DRIVE_SYSTEM==3
    calculate_delta(Printer::currentPositionSteps, printer.currentDeltaPositionSteps);
#endif
    printer.maxJerk = MAX_JERK;
    printer.maxZJerk = MAX_ZJERK;
    Printer::interval = 5000;
    Printer::stepsPerTimerCall = 1;
    printer.msecondsPrinting = 0;
    printer.filamentPrinted = 0;
    printer.flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
    printer.xLength = X_MAX_LENGTH;
    printer.yLength = Y_MAX_LENGTH;
    printer.zLength = Z_MAX_LENGTH;
    printer.xMin = X_MIN_POS;
    printer.yMin = Y_MIN_POS;
    printer.zMin = Z_MIN_POS;
    printer.waslasthalfstepping = 0;
#if ENABLE_BACKLASH_COMPENSATION
    printer.backlashX = X_BACKLASH;
    printer.backlashY = Y_BACKLASH;
    printer.backlashZ = Z_BACKLASH;
    printer.backlashDir = 0;
#endif
#if defined(USE_ADVANCE)
    printer.extruderStepsNeeded = 0;
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
    long probeDepth = 2*(printer.zMaxSteps-printer.zMinSteps);
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
    float distance = (float)probeDepth*inv_axis_steps_per_unit[2];
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
    if(first) {
        GCode::executeFString(Com::tZProbeStartScript);
        printer.offsetX -= EEPROM::zProbeXOffset();
        printer.offsetY -= EEPROM::zProbeYOffset();
    }
    Commands::waitUntilEndOfAllMoves();
    long probeDepth = 2*(printer.zMaxSteps-printer.zMinSteps);
    stepsRemainingAtZHit = -1;
    long offx = axis_steps_per_unit[0]*EEPROM::zProbeXOffset();
    long offy = axis_steps_per_unit[0]*EEPROM::zProbeYOffset();
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
    float distance = (float)probeDepth*inv_axis_steps_per_unit[2]+EEPROM::zProbeHeight();
    Com::printF(Com::tZProbe,distance);
    Com::printF(Com::tSpaceXColon,realXPosition());
    Com::printFLN(Com::tSpaceYColon,realYPosition());
    PrintLine::moveRelativeDistanceInSteps(0,0,probeDepth,0,Z_PROBE_SPEED,true,true);
    PrintLine::moveRelativeDistanceInSteps(offx,offy,0,0,EEPROM::zProbeXYSpeed(),true,true);
    if(last) {
        GCode::executeFString(Com::tZProbeEndScript);
        printer.offsetX += EEPROM::zProbeXOffset();
        printer.offsetY += EEPROM::zProbeYOffset();
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
