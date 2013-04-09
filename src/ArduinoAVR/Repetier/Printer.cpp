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
byte STEP_PIN[3] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN};
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
long Printer::destinationSteps[4];
byte Printer::flag0 = 0;
byte Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
byte Printer::stepsPerTimerCall = 1;

void Printer::constrainDestinationCoords() {
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
void Printer::updateAdvanceFlags() {
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
    if(com->hasX())
    {
        p = convertToMM(com->X*axis_steps_per_unit[0]);
        if(relative_mode)
            destinationSteps[0] = currentPositionSteps[0]+p;
        else
            destinationSteps[0] = p+printer.offsetX;
    }
    else destinationSteps[0] = currentPositionSteps[0];
    if(com->hasY())
    {
        p = convertToMM(com->Y*axis_steps_per_unit[1]);
        if(relative_mode)
            destinationSteps[1] = currentPositionSteps[1]+p;
        else
            destinationSteps[1] = p+printer.offsetY;
    }
    else destinationSteps[1] = currentPositionSteps[1];
    if(com->hasZ())
    {
        p = convertToMM(com->Z*axis_steps_per_unit[2]);
        if(relative_mode)
            destinationSteps[2] = currentPositionSteps[2]+p;
        else
            destinationSteps[2] = p;
    }
    else destinationSteps[2] = currentPositionSteps[2];
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

void Printer::setup() {
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
    if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
    SET_OUTPUT(X_ENABLE_PIN);
#endif
#if Y_ENABLE_PIN > -1
    if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
    SET_OUTPUT(Y_ENABLE_PIN);
#endif
#if Z_ENABLE_PIN > -1
    if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
    SET_OUTPUT(Z_ENABLE_PIN);
#endif

    //endstop pullups
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
    SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
    WRITE(X_MIN_PIN,HIGH);
#endif
#endif
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
    SET_INPUT(Y_MIN_PIN);
#if ENDSTOP_PULLUP_Y_MIN
    WRITE(Y_MIN_PIN,HIGH);
#endif
#endif
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
    SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
    WRITE(Z_MIN_PIN,HIGH);
#endif
#endif
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
    SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
    WRITE(X_MAX_PIN,HIGH);
#endif
#endif
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
    SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
    WRITE(Y_MAX_PIN,HIGH);
#endif
#endif
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
    SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
    WRITE(Z_MAX_PIN,HIGH);
#endif
#endif
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

