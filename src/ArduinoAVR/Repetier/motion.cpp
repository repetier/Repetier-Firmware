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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

// ================ Sanity checks ================
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY < 10000 || STEP_DOUBLER_FREQUENCY > 20000
#if CPU_ARCH==ARCH_AVR
#error STEP_DOUBLER_FREQUENCY should be in range 10000-16000.
#endif
#endif
#endif
#ifdef EXTRUDER_SPEED
#error EXTRUDER_SPEED is not used any more. Values are now taken from extruder definition.
#endif
#ifdef ENDSTOPPULLUPS
#error ENDSTOPPULLUPS is now replaced by individual pullup configuration!
#endif
#ifdef EXT0_PID_PGAIN
#error The PID system has changed. Please use the new float number options!
#endif
// ####################################################################################
// #          No configuration below this line - just some errorchecking              #
// ####################################################################################
#ifdef SUPPORT_MAX6675
#if !defined SCK_PIN || !defined MOSI_PIN || !defined MISO_PIN
#error For MAX6675 support, you need to define SCK_PIN, MISO_PIN and MOSI_PIN in pins.h
#endif
#endif
#if X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if EXT0_STEP_PIN<0 && NUM_EXTRUDER>0
#error EXT0_STEP_PIN not set to a pin number.
#endif
#if EXT0_DIR_PIN<0 && NUM_EXTRUDER>0
#error EXT0_DIR_PIN not set to a pin number.
#endif
#if PRINTLINE_CACHE_SIZE<4
#error PRINTLINE_CACHE_SIZE must be at least 5
#endif

//Inactivity shutdown variables
millis_t previousMillisCmd = 0;
millis_t maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
millis_t stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
long baudrate = BAUDRATE;         ///< Communication speed rate.
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
int maxadv = 0;
#endif
int maxadv2 = 0;
float maxadvspeed = 0;
#endif
uint8_t pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
volatile int waitRelax = 0; // Delay filament relax at the end of print, could be a simple timeout

PrintLine PrintLine::lines[PRINTLINE_CACHE_SIZE]; ///< Cache for print moves.
PrintLine *PrintLine::cur = NULL;               ///< Current printing line
#if CPU_ARCH == ARCH_ARM
volatile bool PrintLine::nlFlag = false;
#endif
ufast8_t PrintLine::linesWritePos = 0;            ///< Position where we write the next cached line move.
volatile ufast8_t PrintLine::linesCount = 0;      ///< Number of lines cached 0 = nothing to do.
ufast8_t PrintLine::linesPos = 0;                 ///< Position for executing line movement.

/**
Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands.
*/
void PrintLine::moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd, bool checkEndstop)
{
#if NUM_EXTRUDER > 0
    if(Printer::debugDryrun() || (MIN_EXTRUDER_TEMP > 30 && Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed()))
        e = 0; // should not be allowed for current temperature
#endif
    float savedFeedrate = Printer::feedrate;
    Printer::destinationSteps[X_AXIS] = Printer::currentPositionSteps[X_AXIS] + x;
    Printer::destinationSteps[Y_AXIS] = Printer::currentPositionSteps[Y_AXIS] + y;
    Printer::destinationSteps[Z_AXIS] = Printer::currentPositionSteps[Z_AXIS] + z;
    Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] + e;
    Printer::feedrate = feedrate;
#if NONLINEAR_SYSTEM
    if (!queueDeltaMove(checkEndstop, false, false))
    {
        Com::printWarningFLN(PSTR("moveRelativeDistanceInSteps / queueDeltaMove returns error"));
    }
#else
    queueCartesianMove(checkEndstop, false);
#endif
    Printer::feedrate = savedFeedrate;
    Printer::updateCurrentPosition(false);
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();
    previousMillisCmd = HAL::timeInMilliseconds();
}

void PrintLine::moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd)
{
    Printer::lastCmdPos[X_AXIS] += x * Printer::invAxisStepsPerMM[X_AXIS];
    Printer::lastCmdPos[Y_AXIS] += y * Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::lastCmdPos[Z_AXIS] += z * Printer::invAxisStepsPerMM[Z_AXIS];
    if(!Printer::isPositionAllowed( Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS]))
    {
        return; // ignore move
    }
#if NUM_EXTRUDER > 0
    if(Printer::debugDryrun() || (MIN_EXTRUDER_TEMP > 30 && Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed()))
        e = 0; // should not be allowed for current temperature
#endif
    Printer::moveToReal(Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS],
                        (Printer::currentPositionSteps[E_AXIS] + e) * Printer::invAxisStepsPerMM[E_AXIS],feedrate);
    Printer::updateCurrentPosition();
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();
    previousMillisCmd = HAL::timeInMilliseconds();
}

#if !NONLINEAR_SYSTEM
/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move.
*/
void PrintLine::queueCartesianMove(uint8_t check_endstops, uint8_t pathOptimize)
{
    Printer::unsetAllSteppersDisabled();
    waitForXFreeLines(1);
    uint8_t newPath = insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = getNextWriteLine();

    float axis_diff[E_AXIS_ARRAY]; // Axis movement in mm
    p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
#if MIXING_EXTRUDER
    if(Printer::isAllEMotors()) {
        p->flags |= FLAG_ALL_E_MOTORS;
    }
#endif
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    Printer::constrainDestinationCoords();
    //Find direction
    for(uint8_t axis = 0; axis < 4; axis++)
    {
        p->delta[axis] = Printer::destinationSteps[axis] - Printer::currentPositionSteps[axis];
        p->secondSpeed = Printer::fanSpeed;
        if(axis == E_AXIS)
        {
            if(Printer::mode == PRINTER_MODE_FFF)
            {
                Printer::extrudeMultiplyError += (static_cast<float>(p->delta[E_AXIS]) * Printer::extrusionFactor);
                p->delta[E_AXIS] = static_cast<int32_t>(Printer::extrudeMultiplyError);
                Printer::extrudeMultiplyError -= p->delta[E_AXIS];
                Printer::filamentPrinted += p->delta[E_AXIS] * Printer::invAxisStepsPerMM[axis];
            }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            else if(Printer::mode == PRINTER_MODE_LASER)
            {
                p->secondSpeed = ((p->delta[X_AXIS] != 0 || p->delta[Y_AXIS] != 0) && (LaserDriver::laserOn || p->delta[E_AXIS] != 0) ? LaserDriver::intensity : 0);
                p->delta[E_AXIS] = 0;
            }
#endif
        }
        if(p->delta[axis] >= 0)
            p->setPositiveDirectionForAxis(axis);
        else
            p->delta[axis] = -p->delta[axis];
        axis_diff[axis] = p->delta[axis] * Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::currentPositionSteps[axis] = Printer::destinationSteps[axis];
    }
    if(p->isNoMove())
    {
        if(newPath)   // need to delete dummy elements, otherwise commands can get locked.
            resetPathPlanner();
        return; // No steps included
    }
    float xydist2;
#if ENABLE_BACKLASH_COMPENSATION
    if((p->isXYZMove()) && ((p->dir & XYZ_DIRPOS)^(Printer::backlashDir & XYZ_DIRPOS)) & (Printer::backlashDir >> 3))   // We need to compensate backlash, add a move
    {
        waitForXFreeLines(2);
        uint8_t wpos2 = linesWritePos + 1;
        if(wpos2 >= PRINTLINE_CACHE_SIZE) wpos2 = 0;
        PrintLine *p2 = &lines[wpos2];
        memcpy(p2,p,sizeof(PrintLine)); // Move current data to p2
        uint8_t changed = (p->dir & XYZ_DIRPOS)^(Printer::backlashDir & XYZ_DIRPOS);
        float back_diff[4]; // Axis movement in mm
        back_diff[E_AXIS] = 0;
        back_diff[X_AXIS] = (changed & 1 ? (p->isXPositiveMove() ? Printer::backlashX : -Printer::backlashX) : 0);
        back_diff[Y_AXIS] = (changed & 2 ? (p->isYPositiveMove() ? Printer::backlashY : -Printer::backlashY) : 0);
        back_diff[Z_AXIS] = (changed & 4 ? (p->isZPositiveMove() ? Printer::backlashZ : -Printer::backlashZ) : 0);
        p->dir &= XYZ_DIRPOS; // x,y and z are already correct
        for(uint8_t i = 0; i < 4; i++)
        {
            float f = back_diff[i]*Printer::axisStepsPerMM[i];
            p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= XSTEP << i;
        }
        //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
        if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS]) p->primaryAxis = Y_AXIS;
        else if (p->delta[X_AXIS] > p->delta[Z_AXIS] ) p->primaryAxis = X_AXIS;
        else p->primaryAxis = Z_AXIS;
        p->stepsRemaining = p->delta[p->primaryAxis];
        //Feedrate calc based on XYZ travel distance
        xydist2 = back_diff[X_AXIS] * back_diff[X_AXIS] + back_diff[Y_AXIS] * back_diff[Y_AXIS];
        if(p->isZMove())
            p->distance = sqrt(xydist2 + back_diff[Z_AXIS] * back_diff[Z_AXIS]);
        else
            p->distance = sqrt(xydist2);
        // 56 seems to be xstep|ystep|e_posdir which just seems odd
        Printer::backlashDir = (Printer::backlashDir & 56) | (p2->dir & XYZ_DIRPOS);
        p->calculateMove(back_diff,pathOptimize);
        p = p2; // use saved instance for the real move
    }
#endif

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
    if(p->delta[Y_AXIS] > p->delta[X_AXIS] && p->delta[Y_AXIS] > p->delta[Z_AXIS] && p->delta[Y_AXIS] > p->delta[E_AXIS]) p->primaryAxis = Y_AXIS;
    else if (p->delta[X_AXIS] > p->delta[Z_AXIS] && p->delta[X_AXIS] > p->delta[E_AXIS]) p->primaryAxis = X_AXIS;
    else if (p->delta[Z_AXIS] > p->delta[E_AXIS]) p->primaryAxis = Z_AXIS;
    else p->primaryAxis = E_AXIS;
    p->stepsRemaining = p->delta[p->primaryAxis];
    if(p->isXYZMove())
    {
        xydist2 = axis_diff[X_AXIS] * axis_diff[X_AXIS] + axis_diff[Y_AXIS] * axis_diff[Y_AXIS];
        if(p->isZMove())
            p->distance = RMath::max((float)sqrt(xydist2 + axis_diff[Z_AXIS] * axis_diff[Z_AXIS]),fabs(axis_diff[E_AXIS]));
        else
            p->distance = RMath::max((float)sqrt(xydist2),fabs(axis_diff[E_AXIS]));
    }
    else
        p->distance = fabs(axis_diff[E_AXIS]);
    p->calculateMove(axis_diff,pathOptimize);
}
#endif

void PrintLine::calculateMove(float axis_diff[], uint8_t pathOptimize)
{
#if NONLINEAR_SYSTEM
    long axisInterval[VIRTUAL_AXIS_ARRAY]; // shortest interval possible for that axis
#else
    long axisInterval[E_AXIS_ARRAY];
#endif
    float timeForMove = (float)(F_CPU)*distance / (isXOrYMove() ? RMath::max(Printer::minimumSpeed, Printer::feedrate) : Printer::feedrate); // time is in ticks
    //bool critical = Printer::isZProbingActive();
    if(linesCount < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE)   // Limit speed to keep cache full.
    {
        //OUT_P_I("L:",lines_count);
        timeForMove += (3 * (LOW_TICKS_PER_MOVE - timeForMove)) / (linesCount + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
        //OUT_P_F_LN("Slow ",time_for_move);
        //critical = true;
    }
    timeInTicks = timeForMove;
    UI_MEDIUM; // do check encoder
    // Compute the slowest allowed interval (ticks/step), so maximum feedrate is not violated
    long limitInterval = timeForMove / stepsRemaining; // until not violated by other constraints it is your target speed
    if(isXMove())
    {
        axisInterval[X_AXIS] = fabs(axis_diff[X_AXIS]) * F_CPU / (Printer::maxFeedrate[X_AXIS] * stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
#if !NONLINEAR_SYSTEM
        limitInterval = RMath::max(axisInterval[X_AXIS], limitInterval);
#endif
    }
    else axisInterval[X_AXIS] = 0;
    if(isYMove())
    {
        axisInterval[Y_AXIS] = fabs(axis_diff[Y_AXIS]) * F_CPU / (Printer::maxFeedrate[Y_AXIS] * stepsRemaining);
#if !NONLINEAR_SYSTEM
        limitInterval = RMath::max(axisInterval[Y_AXIS], limitInterval);
#endif
    }
    else axisInterval[Y_AXIS] = 0;
    if(isZMove())   // normally no move in z direction
    {
        axisInterval[Z_AXIS] = fabs(axis_diff[Z_AXIS]) * (float)F_CPU / (float)(Printer::maxFeedrate[Z_AXIS] * stepsRemaining); // must prevent overflow!
#if !NONLINEAR_SYSTEM
        limitInterval = RMath::max(axisInterval[Z_AXIS], limitInterval);
#endif
    }
    else axisInterval[Z_AXIS] = 0;
    if(isEMove())
    {
        axisInterval[E_AXIS] = fabs(axis_diff[E_AXIS]) * F_CPU / (Printer::maxFeedrate[E_AXIS] * stepsRemaining);
#if !NONLINEAR_SYSTEM
        limitInterval = RMath::max(axisInterval[E_AXIS], limitInterval);
#endif
    }
    else axisInterval[E_AXIS] = 0;
#if NONLINEAR_SYSTEM
    if(axis_diff[VIRTUAL_AXIS] >= 0)
        axisInterval[VIRTUAL_AXIS] = fabs(axis_diff[VIRTUAL_AXIS]) * F_CPU / (Printer::maxFeedrate[Z_AXIS] * stepsRemaining);
    else
        axisInterval[VIRTUAL_AXIS] = fabs(axis_diff[VIRTUAL_AXIS]) * F_CPU / (Printer::maxFeedrate[E_AXIS] * stepsRemaining);
    limitInterval = RMath::max(axisInterval[VIRTUAL_AXIS], limitInterval);
#endif

    fullInterval = limitInterval = limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL; // This is our target speed
    // new time at full speed = limitInterval*p->stepsRemaining [ticks]
    timeForMove = (float)limitInterval * (float)stepsRemaining; // for large z-distance this overflows with long computation
    float inv_time_s = (float)F_CPU / timeForMove;
    if(isXMove())
    {
        axisInterval[X_AXIS] = timeForMove / delta[X_AXIS];
        speedX = axis_diff[X_AXIS] * inv_time_s;
        if(isXNegativeMove()) speedX = -speedX;
    }
    else speedX = 0;
    if(isYMove())
    {
        axisInterval[Y_AXIS] = timeForMove / delta[Y_AXIS];
        speedY = axis_diff[Y_AXIS] * inv_time_s;
        if(isYNegativeMove()) speedY = -speedY;
    }
    else speedY = 0;
    if(isZMove())
    {
        axisInterval[Z_AXIS] = timeForMove / delta[Z_AXIS];
        speedZ = axis_diff[Z_AXIS] * inv_time_s;
        if(isZNegativeMove()) speedZ = -speedZ;
    }
    else speedZ = 0;
    if(isEMove())
    {
        axisInterval[E_AXIS] = timeForMove / delta[E_AXIS];
        speedE = axis_diff[E_AXIS] * inv_time_s;
        if(isENegativeMove()) speedE = -speedE;
    }
#if NONLINEAR_SYSTEM
    axisInterval[VIRTUAL_AXIS] = limitInterval; //timeForMove/stepsRemaining;
#endif
    fullSpeed = distance * inv_time_s;
    //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
#if RAMP_ACCELERATION
    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowest_axis_plateau_time_repro = 1e15; // repro to reduce division Unit: 1/s
    uint32_t *accel = (isEPositiveMove() ?  Printer::maxPrintAccelerationStepsPerSquareSecond : Printer::maxTravelAccelerationStepsPerSquareSecond);
#if defined(INTERPOLATE_ACCELERATION_WITH_Z) && INTERPOLATE_ACCELERATION_WITH_Z != 0
    uint32_t newAccel[4];
    float accelFac = 100.0 + (EEPROM::accelarationFactorTop() - 100.0) * Printer::currentPosition[Z_AXIS] / Printer::zLength;
#if INTERPOLATE_ACCELERATION_WITH_Z == 1 || INTERPOLATE_ACCELERATION_WITH_Z == 3
    newAccel[X_AXIS] = static_cast<int32_t>(accel[X_AXIS] * accelFac) / 100;
    newAccel[Y_AXIS] = static_cast<int32_t>(accel[Y_AXIS] * accelFac) / 100;
#else
    newAccel[X_AXIS] = accel[X_AXIS];
    newAccel[Y_AXIS] = accel[Y_AXIS];
#endif
#if INTERPOLATE_ACCELERATION_WITH_Z == 2 || INTERPOLATE_ACCELERATION_WITH_Z == 3
    newAccel[Z_AXIS] = static_cast<int32_t>(accel[Z_AXIS] * accelFac) / 100;
#else
    newAccel[Z_AXIS] = accel[Z_AXIS];
#endif
    newAccel[E_AXIS] = accel[E_AXIS];
    accel = newAccel;
#endif
    for(uint8_t i = 0; i < 4 ; i++)
    {
        if(isMoveOfAxis(i))
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowest_axis_plateau_time_repro = RMath::min(slowest_axis_plateau_time_repro, (float)axisInterval[i] * (float)accel[i]); //  steps/s^2 * step/tick  Ticks/s^2
    }

    // Errors for delta move are initialized in timer (except extruder)
#if !NONLINEAR_SYSTEM
    error[X_AXIS] = error[Y_AXIS] = error[Z_AXIS] = delta[primaryAxis] >> 1;
#endif
#if NONLINEAR_SYSTEM
    error[E_AXIS] = stepsRemaining >> 1;
#endif
    invFullSpeed = 1.0 / fullSpeed;
    accelerationPrim = slowest_axis_plateau_time_repro / axisInterval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    fAcceleration = 262144.0 * (float)accelerationPrim / F_CPU; // will overflow without float!
    accelerationDistance2 = 2.0 * distance * slowest_axis_plateau_time_repro * fullSpeed/((float)F_CPU); // mm^2/s^2
    startSpeed = endSpeed = minSpeed = safeSpeed();
    // Can accelerate to full speed within the line
    if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
        setNominalMove();

    vMax = F_CPU / fullInterval; // maximum steps per second, we can reach
    // if(p->vMax>46000)  // gets overflow in N computation
    //   p->vMax = 46000;
    //p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#if USE_ADVANCE
    if(!isXYZMove() || !isEMove())
    {
#if ENABLE_QUADRATIC_ADVANCE
        advanceRate = 0; // No head move or E move only or sucking filament back
        advanceFull = 0;
#endif
        advanceL = 0;
    }
    else
    {
        float advlin = fabs(speedE) * Extruder::current->advanceL * 0.001 * Printer::axisStepsPerMM[E_AXIS];
        advanceL = (uint16_t)((65536L * advlin)/vMax); //advanceLscaled = (65536*vE*k2)/vMax
#if ENABLE_QUADRATIC_ADVANCE
        advanceFull = 65536*Extruder::current->advanceK * speedE * speedE; // Steps*65536 at full speed
        long steps = (HAL::U16SquaredToU32(vMax)) / (accelerationPrim << 1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
        advanceRate = advanceFull / steps;
        if((advanceFull >> 16) > maxadv)
        {
            maxadv = (advanceFull >> 16);
            maxadvspeed = fabs(speedE);
        }
#endif
        if(advlin > maxadv2)
        {
            maxadv2 = advlin;
            maxadvspeed = fabs(speedE);
        }
    }
#endif
    UI_MEDIUM; // do check encoder
    updateTrapezoids();
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
#else
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    advanceRate = 0; // No advance for constant speeds
    advanceFull = 0;
#endif
#endif
#endif

#ifdef DEBUG_STEPCOUNT
// Set in delta move calculation
#if !NONLINEAR_SYSTEM
    totalStepsRemaining = delta[X_AXIS] + delta[Y_AXIS] + delta[Z_AXIS];
#endif
#endif
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho())
    {
        logLine();
        Com::printFLN(Com::tDBGLimitInterval, limitInterval);
        Com::printFLN(Com::tDBGMoveDistance, distance);
        Com::printFLN(Com::tDBGCommandedFeedrate, Printer::feedrate);
        Com::printFLN(Com::tDBGConstFullSpeedMoveTime, timeForMove);
    }
#endif
    // Make result permanent
    if (pathOptimize) waitRelax = 70;
    pushLine();
    DEBUG_MEMORY;
}

/**
This is the path planner.

It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely to become active.

The method is called before lines_count is increased!
*/
void PrintLine::updateTrapezoids()
{
    ufast8_t first = linesWritePos;
    PrintLine *firstLine;
    PrintLine *act = &lines[linesWritePos];
    InterruptProtectedBlock noInts;

    // First we find out how far back we could go with optimization.

    ufast8_t maxfirst = linesPos; // first non fixed segment we might change
    if(maxfirst != linesWritePos)
        nextPlannerIndex(maxfirst); // don't touch the line printing
    // Now ignore enough segments to gain enough time for path planning
    millis_t timeleft = 0;
    // Skip as many stored moves as needed to gain enough time for computation
#if PRINTLINE_CACHE_SIZE < 10
#define minTime 4500L * PRINTLINE_CACHE_SIZE
#else
#define minTime 45000L
#endif
    while(timeleft < minTime && maxfirst != linesWritePos)
    {
        timeleft += lines[maxfirst].timeInTicks;
        nextPlannerIndex(maxfirst);
    }
    // Search last fixed element
    while(first != maxfirst && !lines[first].isEndSpeedFixed())
        previousPlannerIndex(first);
    if(first != linesWritePos && lines[first].isEndSpeedFixed())
        nextPlannerIndex(first);
    // now first points to last segment before the end speed is fixed
    // so start speed is also fixed.

    if(first == linesWritePos)   // Nothing to plan, only new element present
    {
        act->block(); // Prevent steppe rinterrupt from using this
        noInts.unprotect();
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        act->unblock();
        return;
    }
    // now we have at least one additional move for optimization
    // that is not a wait move
    // First is now the new element or the first element with non fixed end speed.
    // anyhow, the start speed of first is fixed
    firstLine = &lines[first];
    firstLine->block(); // don't let printer touch this or following segments during update
    noInts.unprotect();
    ufast8_t previousIndex = linesWritePos;
    previousPlannerIndex(previousIndex);
    PrintLine *previous = &lines[previousIndex]; // segment before the one we are inserting
#if DRIVE_SYSTEM != DELTA
    // filters z-move<->not z-move
    if((previous->primaryAxis == Z_AXIS && act->primaryAxis != Z_AXIS) || (previous->primaryAxis != Z_AXIS && act->primaryAxis == Z_AXIS))
    {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
#endif // DRIVE_SYSTEM

    if(previous->isEOnlyMove() != act->isEOnlyMove())
    {
        previous->maxJunctionSpeed = previous->endSpeed;
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
    else
    {
        computeMaxJunctionSpeed(previous, act); // Set maximum junction speed if we have a real move before
    }
    // Increase speed if possible neglecting current speed
    backwardPlanner(linesWritePos,first);
    // Reduce speed to reachable speeds
    forwardPlanner(first);

    // Update precomputed data
    do
    {
        lines[first].updateStepsParameter();
        //noInts.protect();
        lines[first].unblock();  // start with first block to release next used segment as early as possible
        nextPlannerIndex(first);
        lines[first].block();
        //noInts.unprotect();
    }
    while(first != linesWritePos);
    act->updateStepsParameter();
    act->unblock();
}

/* Computes the maximum junction speed of the newly added segment under
optimal conditions. There is no guarantee that the previous move will be able to reach the
speed at all, but if it could exceed it will never exceed this theoretical limit.

if you define ALTERNATIVE_JERK teh new jerk computations are used. These
use the cosine of the angle and the maximum speed
Jerk = (1-cos(alpha))*min(v1,v2)
This sets jerk to 0 on zero angle change.

        Old               New
0°:       0               0
30°:     51,8             13.4
45°:     76.53            29.3
90°:    141               100
180°:   200               200


von 100 auf 200
        Old               New(min)   New(max)
0°:     100               0          0
30°:    123,9             13.4       26.8
45°:    147.3             29.3       58.6
90°:    223               100        200
180°:   300               200        400

*/
inline void PrintLine::computeMaxJunctionSpeed(PrintLine *previous, PrintLine *current)
{
#if NONLINEAR_SYSTEM
    if (previous->moveID == current->moveID)   // Avoid computing junction speed for split delta lines
    {
        if(previous->fullSpeed > current->fullSpeed)
            previous->maxJunctionSpeed = current->fullSpeed;
        else
            previous->maxJunctionSpeed = previous->fullSpeed;
        return;
    }
#endif
#if USE_ADVANCE
    if(Printer::isAdvanceActivated())
    {
        // if we start/stop extrusion we need to do so with lowest possible end speed
        // or advance would leave a drolling extruder and can not adjust fast enough.
        if(previous->isEMove() != current->isEMove())
        {
            previous->setEndSpeedFixed(true);
            current->setStartSpeedFixed(true);
            previous->endSpeed = current->startSpeed = previous->maxJunctionSpeed = RMath::min(previous->endSpeed, current->startSpeed);
            previous->invalidateParameter();
            current->invalidateParameter();
            return;
        }
    }
#endif // USE_ADVANCE
    // if we are here we have to identical move types
    // either pure extrusion -> pure extrusion or
    // move -> move (with or without extrusion)
    // First we compute the normalized jerk for speed 1
    float factor = 1.0;
    float maxJoinSpeed = RMath::min(current->fullSpeed,previous->fullSpeed);
#if (DRIVE_SYSTEM == DELTA) // No point computing Z Jerk separately for delta moves
#ifdef ALTERNATIVE_JERK
    float jerk = maxJoinSpeed * (1.0 - (current->speedX * previous->speedX + current->speedY * previous->speedY + current->speedZ * previous->speedZ) / (current->fullSpeed * previous->fullSpeed));
#else
    float dx = current->speedX - previous->speedX;
    float dy = current->speedY - previous->speedY;
    float dz = current->speedZ - previous->speedZ;
    float jerk = sqrt(dx * dx + dy * dy + dz * dz);
#endif // ALTERNATIVE_JERK
#else // DELTA
#ifdef ALTERNATIVE_JERK
    float jerk = maxJoinSpeed * (1.0 - (current->speedX * previous->speedX + current->speedY * previous->speedY + current->speedZ * previous->speedZ) / (current->fullSpeed * previous->fullSpeed));
#else
    float dx = current->speedX - previous->speedX;
    float dy = current->speedY - previous->speedY;
    float jerk = sqrt(dx * dx + dy * dy);
#endif // ALTERNATIVE_JERK
#endif // DELTA
    if(jerk > Printer::maxJerk)
        factor = Printer::maxJerk / jerk; // always < 1.0!
#if DRIVE_SYSTEM != DELTA
    if((previous->dir | current->dir) & ZSTEP)
    {
        float dz = fabs(current->speedZ - previous->speedZ);
        if(dz > Printer::maxZJerk)
            factor = RMath::min(factor, Printer::maxZJerk / dz);
    }
#endif
    float eJerk = fabs(current->speedE - previous->speedE);
    if(eJerk > Extruder::current->maxStartFeedrate)
        factor = RMath::min(factor, Extruder::current->maxStartFeedrate / eJerk);

    previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho())
    {
        Com::printF(PSTR("ID:"), (int)previous);
        Com::printFLN(PSTR(" MJ:"), previous->maxJunctionSpeed);
    }
#endif // DEBUG_QUEUE_MOVE
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/decelleration steps and advanced parameter associated.
*/
void PrintLine::updateStepsParameter()
{
    if(areParameterUpToDate() || isWarmUp()) return;
    float startFactor = startSpeed * invFullSpeed;
    float endFactor   = endSpeed   * invFullSpeed;
    vStart = vMax * startFactor; //starting speed
    vEnd   = vMax * endFactor;
#if CPU_ARCH == ARCH_AVR
    uint32_t vmax2 = HAL::U16SquaredToU32(vMax);
    accelSteps = ((vmax2 - HAL::U16SquaredToU32(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
    decelSteps = ((vmax2 - HAL::U16SquaredToU32(vEnd))  /(accelerationPrim << 1)) + 1;
#else
    uint64_t vmax2 = static_cast<uint64_t>(vMax) * static_cast<uint64_t>(vMax);
    accelSteps = ((vmax2 - static_cast<uint64_t>(vStart) * static_cast<uint64_t>(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
    decelSteps = ((vmax2 - static_cast<uint64_t>(vEnd) * static_cast<uint64_t>(vEnd)) / (accelerationPrim << 1)) + 1;
#endif

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    advanceStart = (float)advanceFull * startFactor * startFactor;
    advanceEnd   = (float)advanceFull * endFactor   * endFactor;
#endif
#endif
    if(accelSteps + decelSteps >= stepsRemaining)   // can't reach limit speed
    {
        uint16_t red = (accelSteps + decelSteps + 2 - stepsRemaining) >> 1;
        accelSteps = accelSteps - RMath::min(accelSteps, red);
        decelSteps = decelSteps - RMath::min(decelSteps, red);
    }
    setParameterUpToDate();
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho())
    {
        Com::printFLN(Com::tDBGId,(int)this);
        Com::printF(Com::tDBGVStartEnd,(long)vStart);
        Com::printFLN(Com::tSlash,(long)vEnd);
        Com::printF(Com::tDBAccelSteps,(long)accelSteps);
        Com::printF(Com::tSlash,(long)decelSteps);
        Com::printFLN(Com::tSlash,(long)stepsRemaining);
        Com::printF(Com::tDBGStartEndSpeed,startSpeed,1);
        Com::printFLN(Com::tSlash,endSpeed,1);
        Com::printFLN(Com::tDBGFlags,(uint32_t)flags);
        Com::printFLN(Com::tDBGJoinFlags,(uint32_t)joinFlags);
    }
#endif
}

/**
Compute the maximum speed from the last entered move.
The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

start = last line inserted
last = last element until we check
*/
inline void PrintLine::backwardPlanner(ufast8_t start,ufast8_t last)
{
    PrintLine *act = &lines[start], *previous;
    float lastJunctionSpeed = act->endSpeed; // Start always with safe speed

    //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
    while(start != last)
    {
        previousPlannerIndex(start);
        previous = &lines[start];
        previous->block();
        // Avoid speed calc once crusing in split delta move
#if NONLINEAR_SYSTEM
        if (previous->moveID == act->moveID && lastJunctionSpeed == previous->maxJunctionSpeed)
        {
            act->startSpeed = RMath::max(act->minSpeed, previous->endSpeed = lastJunctionSpeed);
            previous->invalidateParameter();
            act->invalidateParameter();
        }
#endif

        /* if(prev->isEndSpeedFixed())   // Nothing to update from here on, happens when path optimize disabled
         {
             act->setStartSpeedFixed(true);
             return;
         }*/

        // Avoid speed calcs if we know we can accelerate within the line
        lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrt(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2)); // acceleration is acceleration*distance*2! What can be reached if we try?
        // If that speed is more that the maximum junction speed allowed then ...
        if(lastJunctionSpeed >= previous->maxJunctionSpeed)   // Limit is reached
        {
            // If the previous line's end speed has not been updated to maximum speed then do it now
            if(previous->endSpeed != previous->maxJunctionSpeed)
            {
                previous->invalidateParameter(); // Needs recomputation
                previous->endSpeed = RMath::max(previous->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
            }
            // If actual line start speed has not been updated to maximum speed then do it now
            if(act->startSpeed != previous->maxJunctionSpeed)
            {
                act->startSpeed = RMath::max(act->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
                act->invalidateParameter();
            }
            lastJunctionSpeed = previous->endSpeed;
        }
        else
        {
            // Block prev end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
            act->startSpeed = RMath::max(act->minSpeed, lastJunctionSpeed);
            lastJunctionSpeed = previous->endSpeed = RMath::max(lastJunctionSpeed, previous->minSpeed);
            previous->invalidateParameter();
            act->invalidateParameter();
        }
        act = previous;
    } // while loop
}

void PrintLine::forwardPlanner(ufast8_t first)
{
    PrintLine *act;
    PrintLine *next = &lines[first];
    float vmaxRight;
    float leftSpeed = next->startSpeed;
    while(first != linesWritePos)   // All except last segment, which has fixed end speed
    {
        act = next;
        nextPlannerIndex(first);
        next = &lines[first];
        /* if(act->isEndSpeedFixed())
         {
             leftSpeed = act->endSpeed;
             continue; // Nothing to do here
         }*/
        // Avoid speed calc once crusing in split delta move
#if NONLINEAR_SYSTEM
        if (act->moveID == next->moveID && act->endSpeed == act->maxJunctionSpeed)
        {
            act->startSpeed = leftSpeed;
            leftSpeed       = act->endSpeed;
            act->setEndSpeedFixed(true);
            next->setStartSpeedFixed(true);
            continue;
        }
#endif
        // Avoid speed calcs if we know we can accelerate within the line.
        vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrt(leftSpeed * leftSpeed + act->accelerationDistance2));
        if(vmaxRight > act->endSpeed)   // Could be higher next run?
        {
            if(leftSpeed < act->minSpeed)
            {
                leftSpeed = act->minSpeed;
                act->endSpeed = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            if(act->endSpeed == act->maxJunctionSpeed)  // Full speed reached, don't compute again!
            {
                act->setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            act->invalidateParameter();
        }
        else     // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
        {
            act->fixStartAndEndSpeed();
            act->invalidateParameter();
            if(act->minSpeed > leftSpeed)
            {
                leftSpeed = act->minSpeed;
                vmaxRight = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
            }
            act->startSpeed = leftSpeed;
            act->endSpeed = RMath::max(act->minSpeed,vmaxRight);
            next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
            next->setStartSpeedFixed(true);
        }
    } // While
    next->startSpeed = RMath::max(next->minSpeed, leftSpeed); // This is the new segment, which is updated anyway, no extra flag needed.
}


inline float PrintLine::safeSpeed()
{
    float safe(Printer::maxJerk * 0.5);
#if DRIVE_SYSTEM != DELTA
    if(isZMove())
    {
        float mz = Printer::maxZJerk * 0.5;
        if(isXOrYMove())
        {
            if(fabs(speedZ) > mz)
                safe = RMath::min(safe,mz * fullSpeed / fabs(speedZ));
        }
        else
        {
            safe = mz;
        }
    }
#endif
    if(isEMove())
    {
        if(isXYZMove())
            safe = RMath::min(safe, 0.5 * Extruder::current->maxStartFeedrate * fullSpeed / fabs(speedE));
        else
            safe = 0.5 * Extruder::current->maxStartFeedrate; // This is a retraction move
    }
    if(DRIVE_SYSTEM == DELTA || primaryAxis == X_AXIS || primaryAxis == Y_AXIS) // enforce minimum speed for numerical stability of explicit speed integration
        safe = RMath::max(Printer::minimumSpeed, safe);
    else if(primaryAxis == Z_AXIS)
    {
        safe = RMath::max(Printer::minimumZSpeed, safe);
    }
    return RMath::min(safe, fullSpeed);
}


/** Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
uint8_t PrintLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines)
{
    if(linesCount == 0 && waitRelax == 0 && pathOptimize)   // First line after some time - warmup needed
    {
#if NONLINEAR_SYSTEM
        uint8_t w = 3;
#else
        uint8_t w = 4;
#endif
        while(w--)
        {
            PrintLine *p = getNextWriteLine();
            p->flags = FLAG_WARMUP;
            p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
            p->dir = 0;
            p->setWaitForXLinesFilled(w + waitExtraLines);
#if NONLINEAR_SYSTEM
            p->setWaitTicks(50000);
#else
            p->setWaitTicks(25000);
#endif // NONLINEAR_SYSTEM
            pushLine();
        }
        return 1;
    }
    return 0;
}

void PrintLine::logLine()
{
#ifdef DEBUG_QUEUE_MOVE
    Com::printFLN(Com::tDBGId,(int)this);
    Com::printArrayFLN(Com::tDBGDelta,delta);
    Com::printFLN(Com::tDBGDir,(uint32_t)dir);
    Com::printFLN(Com::tDBGFlags,(uint32_t)flags);
    Com::printFLN(Com::tDBGFullSpeed,fullSpeed);
    Com::printFLN(Com::tDBGVMax,(int32_t)vMax);
    Com::printFLN(Com::tDBGAcceleration,accelerationDistance2);
    Com::printFLN(Com::tDBGAccelerationPrim,(int32_t)accelerationPrim);
    Com::printFLN(Com::tDBGRemainingSteps,stepsRemaining);
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    Com::printFLN(Com::tDBGAdvanceFull,advanceFull >> 16);
    Com::printFLN(Com::tDBGAdvanceRate,advanceRate);
#endif
#endif
#endif // DEBUG_QUEUE_MOVE
}

void PrintLine::waitForXFreeLines(uint8_t b, bool allowMoves)
{
    while(getLinesCount() + b > PRINTLINE_CACHE_SIZE)   // wait for a free entry in movement cache
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions(allowMoves);
    }
}


#if DRIVE_SYSTEM == DELTA
// pick one for verbose the other silent
#define RETURN_0(s) { Com::printErrorFLN(PSTR(s)); return 0; }
/*#define RETURN_0(s) { Com::print(s " "); SHOWS(temp); SHOWS(opt);\
   SHOWS(cartesianPosSteps[Z_AXIS]);\
   SHOWS(towerAMinSteps); ;\
   SHOWS(deltaPosSteps[A_TOWER]); \
   SHOWS(Printer::deltaAPosYSteps);\
   SHOWS(cartesianPosSteps[Y_AXIS]); \
   SHOW(Printer::deltaDiagonalStepsSquaredA.l);  return 0; }
   */
/**
  Calculate the delta tower position from a cartesian position
  @param cartesianPosSteps Array containing cartesian coordinates.
  @param deltaPosSteps Result array with tower coordinates.
  @returns 1 if cartesian coordinates have a valid delta tower position 0 if not.
*/
uint8_t transformCartesianStepsToDeltaSteps(int32_t cartesianPosSteps[], int32_t deltaPosSteps[])
{
    int32_t zSteps = cartesianPosSteps[Z_AXIS];
#if DISTORTION_CORRECTION
    static int cnt = 0;
    static int32_t lastZSteps = 9999999;
    static int32_t lastZCorrection = 0;
    cnt++;
    if(cnt >= DISTORTION_UPDATE_FREQUENCY || lastZSteps != zSteps)
    {
        cnt = 0;
        lastZSteps = zSteps;
        lastZCorrection = Printer::distortion.correct(cartesianPosSteps[X_AXIS], cartesianPosSteps[Y_AXIS], cartesianPosSteps[Z_AXIS]);
    }
    zSteps += lastZCorrection;
#endif
    //SHOWA("motion.c transformCart... cartesian ",cartesianPosSteps, 3);
    if(Printer::isLargeMachine())
    {
#ifdef SUPPORT_64_BIT_MATH
        // 64 bit is better for precision, so we use that if available.
        // A TOWER height
        uint64_t temp = RMath::absLong(Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS]);
        uint64_t opt = Printer::deltaDiagonalStepsSquaredA.L;

        temp *= temp;
        if (opt < temp)
            RETURN_0("Apos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS]);
        temp *= temp;
        if (opt < temp)
            RETURN_0("Apos x square ");

        deltaPosSteps[A_TOWER] = HAL::integer64Sqrt(opt - temp) + zSteps;
        if (deltaPosSteps[A_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("A hit floor");

        // B TOWER height
        temp = RMath::absLong(Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS]);
        opt = Printer::deltaDiagonalStepsSquaredB.L;
        temp *= temp;
        if (opt < temp)
            RETURN_0("Bpos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS]);
        temp *= temp;
        if (opt < temp)
            RETURN_0("Bpos x square ");

        deltaPosSteps[B_TOWER] = HAL::integer64Sqrt(opt - temp) + zSteps ;
        if (deltaPosSteps[B_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("B hit floor");

        // C TOWER height
        temp = RMath::absLong(Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS]);
        opt = Printer::deltaDiagonalStepsSquaredC.L ;

        temp = temp * temp;
        if ( opt < temp )
            RETURN_0("Cpos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS]);
        temp = temp * temp;
        if ( opt < temp )
            RETURN_0("Cpos x square ");

        deltaPosSteps[C_TOWER] = HAL::integer64Sqrt(opt - temp) + zSteps;
        if (deltaPosSteps[C_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("C hit floor");
#else
        float temp = Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS];
        float opt = Printer::deltaDiagonalStepsSquaredA.f - temp * temp;
        float temp2 = Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS];
        if ((temp = opt - temp2 * temp2) >= 0)
            deltaPosSteps[A_TOWER] = floor(0.5 + sqrt(temp)
                                           + zSteps);
        else
            return 0;
        if (deltaPosSteps[A_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

        temp = Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS];
        opt = Printer::deltaDiagonalStepsSquaredB.f - temp * temp;
        temp2 = Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS];
        if ((temp = opt - temp2 * temp2) >= 0)
            deltaPosSteps[B_TOWER] = floor(0.5 + sqrt(temp)
                                           + zSteps);
        else
            return 0;
        if (deltaPosSteps[B_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

        temp = Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS];
        opt = Printer::deltaDiagonalStepsSquaredC.f - temp * temp;
        temp2 = Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS];
        if ((temp = opt - temp2*temp2) >= 0)
            deltaPosSteps[C_TOWER] = floor(0.5 + sqrt(temp)
                                           + zSteps);
        else
            return 0;
        if (deltaPosSteps[C_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

        return 1;
#endif
    }
    else
    {
        // As we are right on the edge of many printers arm lengths, this is rewrittent to use unsigned long
        // This allows 52% longer arms to be used without performance penalty
        // the code is a bit longer, because we cannot use negative to test for invalid conditions
        // Also, previous code did not check for overflow of squared result
        // Overflow is also detected as a fault condition

        const uint32_t LIMIT = 65534; // Largest squarable int without overflow;

        // A TOWER height
        uint32_t temp = RMath::absLong(Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS]);
        uint32_t opt = Printer::deltaDiagonalStepsSquaredA.l;

        if (temp > LIMIT)
            RETURN_0("Apos y steps ");
        temp *= temp;
        if (opt < temp)
            RETURN_0("Apos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS]);
        if (temp > LIMIT)
            RETURN_0("Apos x steps ");
        temp *= temp;
        if (opt < temp)
            RETURN_0("Apos x square ");

        deltaPosSteps[A_TOWER] = SQRT(opt-temp) + zSteps;
        if (deltaPosSteps[A_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("A hit floor");

        // B TOWER height
        temp = RMath::absLong(Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS]);
        opt = Printer::deltaDiagonalStepsSquaredB.l;

        if (temp > LIMIT)
            RETURN_0("Bpos y steps ");
        temp *= temp;
        if (opt < temp)
            RETURN_0("Bpos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS]);
        if (temp > LIMIT )
            RETURN_0("Bpos x steps ");
        temp *= temp;
        if (opt < temp)
            RETURN_0("Bpos x square ");

        deltaPosSteps[B_TOWER] = SQRT(opt-temp) + zSteps ;
        if (deltaPosSteps[B_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("B hit floor");

        // C TOWER height
        temp = RMath::absLong(Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS]);
        opt = Printer::deltaDiagonalStepsSquaredC.l ;

        if (temp > LIMIT)
            RETURN_0("Cpos y steps ");
        temp = temp * temp;
        if ( opt < temp )
            RETURN_0("Cpos y square ");
        opt -= temp;

        temp = RMath::absLong(Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS]);
        if (temp > LIMIT)
            RETURN_0("Cpos x steps ");
        temp = temp * temp;
        if ( opt < temp )
            RETURN_0("Cpos x square ");

        deltaPosSteps[C_TOWER] = SQRT(opt - temp) + zSteps;
        if (deltaPosSteps[C_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive())
            RETURN_0("C hit floor");
        /*
                long temp = Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS];
                long opt = Printer::deltaDiagonalStepsSquaredA.l - temp * temp;
                long temp2 = Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS];
                if ((temp = opt - temp2 * temp2) >= 0)
        #ifdef FAST_INTEGER_SQRT
                    deltaPosSteps[A_TOWER] = HAL::integerSqrt(temp) + cartesianPosSteps[Z_AXIS];
        #else
                    deltaPosSteps[A_TOWER] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
        #endif
                else
                    return 0;

                temp = Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS];
                opt = Printer::deltaDiagonalStepsSquaredB.l - temp * temp;
                temp2 = Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS];
                if ((temp = opt - temp2*temp2) >= 0)
        #ifdef FAST_INTEGER_SQRT
                    deltaPosSteps[B_TOWER] = HAL::integerSqrt(temp) + cartesianPosSteps[Z_AXIS];
        #else
                    deltaPosSteps[B_TOWER] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
        #endif
                else
                    return 0;

                temp = Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS];
                opt = Printer::deltaDiagonalStepsSquaredC.l - temp * temp;
                temp2 = Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS];
                if ((temp = opt - temp2*temp2) >= 0)
        #ifdef FAST_INTEGER_SQRT
                    deltaPosSteps[C_TOWER] = HAL::integerSqrt(temp) + cartesianPosSteps[Z_AXIS];
        #else
                    deltaPosSteps[C_TOWER] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
        #endif
                else
                    return 0;*/
    }
    return 1;
}
#endif

#if DRIVE_SYSTEM==TUGA

/**
  Calculate the delta tower position from a cartesian position
  @param cartesianPosSteps Array containing cartesian coordinates.
  @param deltaPosSteps Result array with tower coordinates.
  @returns 1 if cartesian coordinates have a valid delta tower position 0 if not.

  X         Y
  *        *
   \      /
    \    /
     \  /
      \/
      /
     /
    /
   /
  *  Extruder


*/
uint8_t transformCartesianStepsToDeltaSteps(int32_t cartesianPosSteps[], int32_t tugaPosSteps[])
{
    tugaPosSteps[0] = cartesianPosSteps[0];
    tugaPosSteps[2] = cartesianPosSteps[2];
    int32_t y2 = Printer::deltaBPosXSteps-cartesianPosSteps[1];
    if(Printer::isLargeMachine())
    {
        float y2f = (float)y2 * (float)y2;
        float temp = Printer::deltaDiagonalStepsSquaredF - y2f;
        if(temp < 0) return 0;
        tugaPosSteps[1] = tugaPosSteps[0] + sqrt(temp);
    }
    else
    {
        y2 = y2*y2;
        int32_t temp = Printer::deltaDiagonalStepsSquared - y2;
        if(temp < 0) return 0;
        tugaPosSteps[1] = tugaPosSteps[0] + HAL::integerSqrt(temp);
    }
    return 1;
}
#endif


#if NONLINEAR_SYSTEM

void DeltaSegment::checkEndstops(PrintLine *cur,bool checkall)
{
    if(Printer::isZProbingActive())
    {
		Endstops::update();
#if FEATURE_Z_PROBE
        if(isZNegativeMove() && Endstops::zProbe())
        {
            cur->setXMoveFinished();
            cur->setYMoveFinished();
            cur->setZMoveFinished();
            dir = 0;
            Printer::stepsRemainingAtZHit = cur->stepsRemaining;
            cur->stepsRemaining = 0;
            return;
        }
#endif
#if DRIVE_SYSTEM == DELTA
        if(isZPositiveMove() && isXPositiveMove() && isYPositiveMove() && Endstops::anyXYZMax())
#else
        if(isZPositiveMove() && Endstops::zMax())
#endif
        {
            cur->setXMoveFinished();
            cur->setYMoveFinished();
            cur->setZMoveFinished();
            dir = 0;
            Printer::stepsRemainingAtZHit = cur->stepsRemaining;
            return;
        }
    }
    if(checkall)
    {
		if(!Printer::isZProbingActive())
			Endstops::update(); // do not test twice
        if(isXPositiveMove() && Endstops::xMax())
        {
#if DRIVE_SYSTEM == DELTA
            if(Printer::stepsRemainingAtXHit < 0)
                Printer::stepsRemainingAtXHit = cur->stepsRemaining;
#endif
            setXMoveFinished();
            cur->setXMoveFinished();
        }
        if(isYPositiveMove() && Endstops::yMax())
        {
#if DRIVE_SYSTEM == DELTA
            if(Printer::stepsRemainingAtYHit < 0)
                Printer::stepsRemainingAtYHit = cur->stepsRemaining;
#endif
            setYMoveFinished();
            cur->setYMoveFinished();
        }
#if DRIVE_SYSTEM != DELTA
        if(isXNegativeMove() && Endstops::xMin())
        {
            setXMoveFinished();
            cur->setXMoveFinished();
        }
        if(isYNegativeMove() && Endstops::yMin())
        {
            setYMoveFinished();
            cur->setYMoveFinished();
        }
#endif
        if(isZPositiveMove() && Endstops::zMax())
        {
#if MAX_HARDWARE_ENDSTOP_Z
            if(Printer::stepsRemainingAtZHit)
                Printer::stepsRemainingAtZHit = cur->stepsRemaining;
#endif
            setZMoveFinished();
            cur->setZMoveFinished();
        }
    }
    if(isZNegativeMove() && Endstops::zMin())
    {
        setZMoveFinished();
        cur->setZMoveFinished();
    }

}

void PrintLine::calculateDirectionAndDelta(int32_t difference[], ufast8_t *dir, int32_t delta[])
{
    *dir = 0;
    //Find direction
    for(uint8_t i = 0; i < 4; i++)
    {
        if(difference[i] >= 0)
        {
            delta[i] = difference[i];
            *dir |= X_DIRPOS << i;
        }
        else
        {
            delta[i] = -difference[i];
        }
        if(delta[i]) *dir |= XSTEP << i;
    }
}
/**
  Calculate and cache the delta robot positions of the cartesian move in a line.
  @return The largest delta axis move in a single segment
  @param p The line to examine.
*/
inline uint16_t PrintLine::calculateDeltaSubSegments(uint8_t softEndstop)
{
    uint8_t i;
    int32_t delta,diff;
    int32_t destinationSteps[Z_AXIS_ARRAY], destinationDeltaSteps[TOWER_ARRAY];
    // Save current position
#if (CPU_ARCH == ARCH_AVR) && !EXACT_DELTA_MOVES
    for(uint8_t i = 0; i < Z_AXIS_ARRAY; i++)
        destinationSteps[i] = Printer::currentPositionSteps[i];
#else
    float dx[Z_AXIS_ARRAY];
    for(int i = 0; i < Z_AXIS_ARRAY; i++)
        dx[i] = static_cast<float>(Printer::destinationSteps[i] - Printer::currentPositionSteps[i]) / static_cast<float>(numDeltaSegments);
#endif
//	out.println_byte_P(PSTR("Calculate delta segments:"), p->numDeltaSegments);
#ifdef DEBUG_STEPCOUNT
    totalStepsRemaining = 0;
#endif

    uint16_t maxAxisMove = 0;
    for (int s = numDeltaSegments; s > 0; s--)
    {
        DeltaSegment *d = &segments[s - 1];

#if (CPU_ARCH == ARCH_AVR) && !EXACT_DELTA_MOVES
        for(i = 0; i < Z_AXIS_ARRAY; i++)
        {
            // End of segment in cartesian steps

            // This method generates small waves which get larger with increasing number of delta segments. smaller??
            diff = Printer::destinationSteps[i] - destinationSteps[i];
            if(s == 1)
                destinationSteps[i] += diff;
            else if(s == 2)
                destinationSteps[i] += (diff >> 1);
            else if(s == 4)
                destinationSteps[i] += (diff >> 2);
            else if(diff<0)
                destinationSteps[i] -= HAL::Div4U2U(-diff, s);
            else
                destinationSteps[i] += HAL::Div4U2U(diff, s);
        }
#else
        float segment = static_cast<float>(numDeltaSegments - s + 1);
        for(i = 0; i < Z_AXIS_ARRAY; i++) // End of segment in cartesian steps
            // Perfect approximation, but slower, so we limit it to faster processors like arm
            destinationSteps[i] = static_cast<int32_t>(floor(0.5 + dx[i] * segment)) + Printer::currentPositionSteps[i];
#endif
        // Verify that delta calc has a solution
        if (transformCartesianStepsToDeltaSteps(destinationSteps, destinationDeltaSteps))
        {
            d->dir = 0;
            if (softEndstop)
            {
                destinationDeltaSteps[A_TOWER] = RMath::min(destinationDeltaSteps[A_TOWER], Printer::maxDeltaPositionSteps);
                destinationDeltaSteps[B_TOWER] = RMath::min(destinationDeltaSteps[B_TOWER], Printer::maxDeltaPositionSteps);
                destinationDeltaSteps[C_TOWER] = RMath::min(destinationDeltaSteps[C_TOWER], Printer::maxDeltaPositionSteps);
            }

            for(i = 0; i < TOWER_ARRAY; i++)
            {
                delta = destinationDeltaSteps[i] - Printer::currentDeltaPositionSteps[i];
                if (delta > 0)
                {
                    d->setPositiveMoveOfAxis(i);
#ifdef DEBUG_DELTA_OVERFLOW
                    if (delta > 65535)
                        Com::printFLN(Com::tDBGDeltaOverflow, delta);
#endif
                    d->deltaSteps[i] = static_cast<uint16_t>(delta);
                }
                else
                {
                    d->setMoveOfAxis(i);
#ifdef DEBUG_DELTA_OVERFLOW
                    if (-delta > 65535)
                        Com::printFLN(Com::tDBGDeltaOverflow, delta);
#endif
                    d->deltaSteps[i] = static_cast<uint16_t>(-delta);
                }
#ifdef DEBUG_STEPCOUNT
                totalStepsRemaining += d->deltaSteps[i];
#endif
                maxAxisMove = RMath::max(maxAxisMove,d->deltaSteps[i]);
                Printer::currentDeltaPositionSteps[i] = destinationDeltaSteps[i];
            }
        }
        else
        {
            // Illegal position - ignore move
            Com::printWarningF(Com::tInvalidDeltaCoordinate);
            Com::printF(PSTR(" x:"), destinationSteps[X_AXIS]);
            Com::printF(PSTR(" y:"), destinationSteps[Y_AXIS]);
            Com::printFLN(PSTR(" z:"), destinationSteps[Z_AXIS]);
            d->dir = 0;
            d->deltaSteps[A_TOWER] = d->deltaSteps[B_TOWER] = d->deltaSteps[C_TOWER] = 0;
            return 65535; // flag error
        }
    }
#ifdef DEBUG_STEPCOUNT
//		out.println_long_P(PSTR("initial StepsRemaining:"), p->totalStepsRemaining);
#endif
    return maxAxisMove;
}

uint8_t PrintLine::calculateDistance(float axisDiff[], uint8_t dir, float *distance)
{
    // Calculate distance depending on direction
    if(dir & XYZ_STEP)
    {
        if(dir & XSTEP)
            *distance = axisDiff[X_AXIS] * axisDiff[X_AXIS];
        else
            *distance = 0;
        if(dir & YSTEP)
            *distance += axisDiff[Y_AXIS] * axisDiff[Y_AXIS];
        if(dir & ZSTEP)
            *distance += axisDiff[Z_AXIS] * axisDiff[Z_AXIS];
        *distance = RMath::max((float)sqrt(*distance),fabs(axisDiff[E_AXIS]));
        return 1;
    }
    else
    {
        if(dir & ESTEP)
        {
            *distance = fabs(axisDiff[E_AXIS]);
            return 1;
        }
        *distance = 0;
        return 0;
    }
}

#if SOFTWARE_LEVELING
void PrintLine::calculatePlane(int32_t factors[], int32_t p1[], int32_t p2[], int32_t p3[])
{
    factors[0] = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
    factors[1] = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
    factors[2] = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
    factors[3] = p1[0] * ((p2[1] * p3[2]) - (p3[1] * p2[2])) + p2[0] * ((p3[1] * p1[2]) - (p1[1] * p3[2])) + p3[0] * ((p1[1] * p2[2]) - (p2[1] * p1[2]));
}

float PrintLine::calcZOffset(int32_t factors[], int32_t pointX, int32_t pointY)
{
    return (factors[3] - factors[X_AXIS] * pointX - factors[Y_AXIS] * pointY) / (float) factors[2];
}
#endif

inline void PrintLine::queueEMove(int32_t extrudeDiff,uint8_t check_endstops,uint8_t pathOptimize)
{
    Printer::unsetAllSteppersDisabled();
    waitForXFreeLines(1);
    uint8_t newPath = insertWaitMovesIfNeeded(pathOptimize, 1);
    PrintLine *p = getNextWriteLine();
    float axisDiff[5]; // Axis movement in mm
    if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS;
    else p->flags = 0;
#if MIXING_EXTRUDER
    if(Printer::isAllEMotors()) {
        p->flags |= FLAG_ALL_E_MOTORS;
    }
#endif
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    //Find direction
    for(uint8_t i = 0; i < 3; i++)
    {
        p->delta[i] = 0;
        axisDiff[i] = 0;
    }
    if (extrudeDiff >= 0)
    {
        p->delta[E_AXIS] = extrudeDiff;
        p->dir = E_STEP_DIRPOS;
    }
    else
    {
        p->delta[E_AXIS] = -extrudeDiff;
        p->dir = ESTEP;
    }
    Printer::currentPositionSteps[E_AXIS] = Printer::destinationSteps[E_AXIS];

    p->numDeltaSegments = 0;
    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
    p->primaryAxis = E_AXIS;
    p->stepsRemaining = p->delta[E_AXIS];
    axisDiff[E_AXIS] = p->distance = p->delta[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];
    axisDiff[VIRTUAL_AXIS] = -p->distance;
    p->moveID = lastMoveID++;
    p->calculateMove(axisDiff,pathOptimize);
}

/**
  Split a line up into a series of lines with at most DELTASEGMENTS_PER_PRINTLINE delta segments.
  @param check_endstops Check endstops during the move.
  @param pathOptimize Run the path optimizer.
  @param delta_step_rate delta step rate in segments per second for the move.
*/
uint8_t PrintLine::queueDeltaMove(uint8_t check_endstops,uint8_t pathOptimize, uint8_t softEndstop)
{
    //if (softEndstop && Printer::destinationSteps[Z_AXIS] < 0) Printer::destinationSteps[Z_AXIS] = 0; // now constrained at entry level including cylinder test
    int32_t difference[E_AXIS_ARRAY];
    float axis_diff[VIRTUAL_AXIS_ARRAY]; // Axis movement in mm. Virtual axis in 4;
    uint8_t secondSpeed = Printer::fanSpeed;
    for(fast8_t axis = 0; axis < E_AXIS_ARRAY; axis++)
    {
        difference[axis] = Printer::destinationSteps[axis] - Printer::currentPositionSteps[axis];
        if(axis == E_AXIS)
        {
            if(Printer::mode == PRINTER_MODE_FFF)
            {
                Printer::extrudeMultiplyError += (static_cast<float>(difference[E_AXIS]) * Printer::extrusionFactor);
                difference[E_AXIS] = static_cast<int32_t>(Printer::extrudeMultiplyError);
                Printer::extrudeMultiplyError -= difference[E_AXIS];
                axis_diff[E_AXIS] = difference[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];
                Printer::filamentPrinted += axis_diff[E_AXIS];
                axis_diff[E_AXIS] = fabs(axis_diff[E_AXIS]);
            }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            else if(Printer::mode == PRINTER_MODE_LASER)
            {
                secondSpeed = ((axis_diff[X_AXIS] != 0 || axis_diff[Y_AXIS] != 0) && (LaserDriver::laserOn || axis_diff[E_AXIS] != 0) ? LaserDriver::intensity : 0);
                axis_diff[E_AXIS] = 0;
            }
#endif
        }
        else
            axis_diff[axis] = fabs(difference[axis] * Printer::invAxisStepsPerMM[axis]);
    }

    float cartesianDistance;
    ufast8_t cartesianDir;
    int32_t cartesianDeltaSteps[E_AXIS_ARRAY];
    calculateDirectionAndDelta(difference, &cartesianDir, cartesianDeltaSteps);
    if (!calculateDistance(axis_diff, cartesianDir, &cartesianDistance))
    {
        // Appears the intent is to do nothing if no distance is detected.
        // This apparently is not an error condition, just early exit.
        return true;
    }

    if (!(cartesianDir & XYZ_STEP))
    {
        queueEMove(difference[E_AXIS], check_endstops,pathOptimize);
        return true;
    }

    int16_t segmentCount;
    float feedrate = RMath::min(Printer::feedrate, Printer::maxFeedrate[Z_AXIS]);
    if (cartesianDir & XY_STEP)
    {
        // Compute number of seconds for move and hence number of segments needed
        //float seconds = 100 * cartesianDistance / (Printer::feedrate * Printer::feedrateMultiply); multiply in feedrate included
        float seconds = cartesianDistance / feedrate;
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaSeconds, seconds);
#endif
        float sps = static_cast<float>((cartesianDir & E_STEP_DIRPOS) == E_STEP_DIRPOS ? Printer::printMovesPerSecond : Printer::travelMovesPerSecond);
        segmentCount = RMath::max(1, static_cast<int16_t>(sps * seconds));
#ifdef DEBUG_SEGMENT_LENGTH
        float segDist = cartesianDistance/(float)segmentCount;
        if(segDist > Printer::maxRealSegmentLength)
        {
            Printer::maxRealSegmentLength = segDist;
            Com::printFLN("StepsPerSecond:",sps);
            Com::printFLN(PSTR("New max. segment length:"),segDist);
        }
#endif
        //Com::printFLN(PSTR("Segments:"),segmentCount);
    }
    else
    {
        // Optimize pure Z axis move. Since a pure Z axis move is linear all we have to watch out for is unsigned integer overuns in
        // the queued moves;
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaZDelta, cartesianDeltaSteps[Z_AXIS]);
#endif
        segmentCount = (cartesianDeltaSteps[Z_AXIS] + (uint32_t)65534) / (uint32_t)65535;
    }
    // Now compute the number of lines needed
    int numLines = (segmentCount + DELTASEGMENTS_PER_PRINTLINE - 1) / DELTASEGMENTS_PER_PRINTLINE;
    // There could be some error here but it doesn't matter since the number of segments will just be reduced slightly
    int segmentsPerLine = segmentCount / numLines;

    int32_t startPosition[E_AXIS_ARRAY], fractionalSteps[E_AXIS_ARRAY];
    if(numLines > 1)
    {
        for (fast8_t i = 0; i < Z_AXIS_ARRAY; i++)
            startPosition[i] = Printer::currentPositionSteps[i];
        startPosition[E_AXIS] = 0;
        cartesianDistance /= numLines;
    }

#ifdef DEBUG_SPLIT
    Com::printFLN(Com::tDBGDeltaSegments, segmentCount);
    Com::printFLN(Com::tDBGDeltaNumLines, numLines);
    Com::printFLN(Com::tDBGDeltaSegmentsPerLine, segmentsPerLine);
#endif
    Printer::unsetAllSteppersDisabled(); // Motor is enabled now
    waitForXFreeLines(1);

    // Insert dummy moves if necessary
    // Nead to leave at least one slot open for the first split move
    insertWaitMovesIfNeeded(pathOptimize, RMath::min(PRINTLINE_CACHE_SIZE - 4, numLines));
    uint32_t oldEDestination = Printer::destinationSteps[E_AXIS]; // flow and volumetric extrusion changed virtual target
    Printer::currentPositionSteps[E_AXIS] = 0;

    for (int lineNumber = 1; lineNumber < numLines + 1; lineNumber++)
    {
        waitForXFreeLines(1);
        PrintLine *p = getNextWriteLine();
        // Downside a comparison per loop. Upside one less distance calculation and simpler code.
        if (numLines == 1)
        {
            // p->numDeltaSegments = segmentCount; // not neede, gets overwritten further down
            p->dir = cartesianDir;
            for (fast8_t i = 0; i < E_AXIS_ARRAY; i++)
            {
                p->delta[i] = cartesianDeltaSteps[i];
                fractionalSteps[i] = difference[i];
            }
            p->distance = cartesianDistance;
        }
        else
        {
            for (fast8_t i = 0; i < E_AXIS_ARRAY; i++)
            {
                Printer::destinationSteps[i] = startPosition[i] + (difference[i] * lineNumber) / numLines;
                fractionalSteps[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
                axis_diff[i] = fabs(fractionalSteps[i] * Printer::invAxisStepsPerMM[i]);
            }
            calculateDirectionAndDelta(fractionalSteps, &p->dir, p->delta);
            p->distance = cartesianDistance;
        }

        p->joinFlags = 0;
        p->secondSpeed = secondSpeed;
        p->moveID = lastMoveID;

        // Only set fixed on last segment
        if (lineNumber == numLines && !pathOptimize)
            p->setEndSpeedFixed(true);

        p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
#if MIXING_EXTRUDER
        if(Printer::isAllEMotors()) {
            p->flags |= FLAG_ALL_E_MOTORS;
        }
#endif
        p->numDeltaSegments = segmentsPerLine;

        uint16_t maxDeltaStep = p->calculateDeltaSubSegments(softEndstop);
        if (maxDeltaStep == 65535)
        {
            Com::printWarningFLN(PSTR("in queueDeltaMove to calculateDeltaSubSegments returns error."));
            return false;
        }

#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaMaxDS, (int32_t)maxDeltaStep);
#endif
        int32_t virtual_axis_move = (int32_t)maxDeltaStep * segmentsPerLine;
        if (virtual_axis_move == 0 && p->delta[E_AXIS] == 0)
        {
            if (numLines != 1)
            {
                Com::printErrorFLN(Com::tDBGDeltaNoMoveinDSegment);
                return false;  // Line too short in low precision area
            }
        }
        p->primaryAxis = VIRTUAL_AXIS; // Virtual axis will lead bresenham step either way
        if (virtual_axis_move > p->delta[E_AXIS])   // Is delta move or E axis leading
        {
            p->stepsRemaining = virtual_axis_move;
            axis_diff[VIRTUAL_AXIS] = p->distance;  //virtual_axis_move * Printer::invAxisStepsPerMM[Z_AXIS]; // Steps/unit same as all the towers
            // Virtual axis steps per segment
            p->numPrimaryStepPerSegment = maxDeltaStep;
        }
        else
        {
            // Round up the E move to get something divisible by segment count which is greater than E move
            p->numPrimaryStepPerSegment = (p->delta[E_AXIS] + segmentsPerLine - 1) / segmentsPerLine;
            p->stepsRemaining = p->numPrimaryStepPerSegment * segmentsPerLine;
            axis_diff[VIRTUAL_AXIS] = -p->distance; //p->stepsRemaining * Printer::invAxisStepsPerMM[Z_AXIS];
        }
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaStepsPerSegment, p->numPrimaryStepPerSegment);
        Com::printFLN(Com::tDBGDeltaVirtualAxisSteps, p->stepsRemaining);
#endif
        p->calculateMove(axis_diff, pathOptimize);
        for (uint8_t i = 0; i < E_AXIS_ARRAY; i++)
        {
            Printer::currentPositionSteps[i] += fractionalSteps[i];
        }
    }
    Printer::currentPositionSteps[E_AXIS] = Printer::destinationSteps[E_AXIS] = oldEDestination;
    lastMoveID++; // Will wrap at 255

    return true; // flag success
}

#endif

#if ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void PrintLine::arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise)
{
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[X_AXIS] + offset[X_AXIS];
    float center_axis1 = position[Y_AXIS] + offset[Y_AXIS];
    //float linear_travel = 0; //target[axis_linear] - position[axis_linear];
    float extruder_travel = (Printer::destinationSteps[E_AXIS] - Printer::currentPositionSteps[E_AXIS]) * Printer::invAxisStepsPerMM[E_AXIS];
    float r_axis0 = -offset[0];  // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    /*long xtarget = Printer::destinationSteps[X_AXIS];
    long ytarget = Printer::destinationSteps[Y_AXIS];
    long ztarget = Printer::destinationSteps[Z_AXIS];
    long etarget = Printer::destinationSteps[E_AXIS];
    */
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001))
    {
        angular_travel += 2.0f * M_PI;
    }
    if (isclockwise)
    {
        angular_travel -= 2.0f * M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel)*radius; //hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001f)
    {
        return;// treat as succes because there is nothing to do;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate > 60.0f ? floor(millimeters_of_travel / RMath::min(static_cast<float>(MM_PER_ARC_SEGMENT_BIG), Printer::feedrate * 0.01666f * static_cast<float>(MM_PER_ARC_SEGMENT))) : floor(millimeters_of_travel / static_cast<float>(MM_PER_ARC_SEGMENT)));
    if(segments == 0) segments = 1;
    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel / segments;
    //float linear_per_segment = linear_travel/segments;
    float extruder_per_segment = extruder_travel / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. This requires only two cos() and sin() computations to form the rotation
       matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       all double numbers are single precision on the Arduino. (True double precision will not have
       round off issues for CNC applications.) Single precision error can accumulate to be greater than
       tool precision in some cases. Therefore, arc path correction is implemented.

       Small angle approximation may be used to reduce computation overhead further. This approximation
       holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[4];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    //arc_target[axis_linear] = position[axis_linear];

    // Initialize the extruder axis
    arc_target[E_AXIS] = Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];

    for (i = 1; i < segments; i++)
    {
        // Increment (segments-1)

        if((count & 3) == 0)
        {
            GCode::readFromSerial();
            Commands::checkForPeriodicalActions(false);
            UI_MEDIUM; // do check encoder
        }

        if (count < N_ARC_CORRECTION)  //25 pieces
        {
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        }
        else
        {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti  = cos(i * theta_per_segment);
            sin_Ti  = sin(i * theta_per_segment);
            r_axis0 = -offset[0] * cos_Ti + offset[1] * sin_Ti;
            r_axis1 = -offset[0] * sin_Ti - offset[1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[X_AXIS] = center_axis0 + r_axis0;
        arc_target[Y_AXIS] = center_axis1 + r_axis1;
        //arc_target[axis_linear] += linear_per_segment;
        arc_target[E_AXIS] += extruder_per_segment;
        Printer::moveToReal(arc_target[X_AXIS],arc_target[Y_AXIS],IGNORE_COORDINATE,arc_target[E_AXIS],IGNORE_COORDINATE);
    }
    // Ensure last segment arrives at target location.
    Printer::moveToReal(target[X_AXIS],target[Y_AXIS],IGNORE_COORDINATE,target[E_AXIS],IGNORE_COORDINATE);
}
#endif



/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
  This is a modified version that implements a bresenham 'multi-step' algorithm where the dominant
  cartesian axis steps may be less than the changing dominant delta axis.
*/
#if NONLINEAR_SYSTEM
int lastblk =- 1;
int32_t cur_errupd;
//#define DEBUG_DELTA_TIMER
// Current delta segment
DeltaSegment *curd;
// Current delta segment primary error increment
int32_t curd_errupd, stepsPerSegRemaining;
int32_t PrintLine::bresenhamStep() // Version for delta printer
{
#if CPU_ARCH == ARCH_ARM
    if(!PrintLine::nlFlag)
#else
    if(cur == NULL)
#endif
    {
        setCurrentLine();
        if(cur->isBlocked())   // This step is in computation - shouldn't happen
        {
            if(lastblk != (int)cur)
            {
                HAL::allowInterrupts();
                lastblk = (int)cur;
                Com::printFLN(Com::tBLK, (int32_t)linesCount);
            }
            cur = NULL;
#if CPU_ARCH == ARCH_ARM
            PrintLine::nlFlag = false;
#endif
            return 2000;
        }
        HAL::allowInterrupts();
        lastblk = -1;
#if INCLUDE_DEBUG_NO_MOVE
        if(Printer::debugNoMoves())   // simulate a move, but do nothing in reality
        {
            //HAL::forbidInterrupts();
            //deltaSegmentCount -= cur->numDeltaSegments; // should always be zero
            removeCurrentLineForbidInterrupt();
            if(linesCount == 0) UI_STATUS_F(Com::translatedF(UI_TEXT_IDLE_ID));
            return 1000;
        }
#endif
        if(cur->isWarmUp())
        {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(linesCount <= cur->getWaitForXLinesFilled())
            {
                cur = NULL;
#if CPU_ARCH==ARCH_ARM
                PrintLine::nlFlag = false;
#endif
                return 2000;
            }
            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        } // End if WARMUP
#if FEATURE_Z_PROBE
        // z move may consist of mroe then 1 z line segment, so we better ignore them
        // if the probe was already hit.
        if(Printer::isZProbingActive() && Printer::stepsRemainingAtZHit >= 0)
        {
            removeCurrentLineForbidInterrupt();
            if(linesCount == 0) UI_STATUS_F(Com::translatedF(UI_TEXT_IDLE_ID));
            return 1000;
        }
#endif

        if(cur->isEMove()) Extruder::enable();
        cur->fixStartAndEndSpeed();
        // Set up delta segments
        if (cur->numDeltaSegments)
        {

            // If there are delta segments point to them here
            curd = &cur->segments[--cur->numDeltaSegments];
            // Enable axis - All axis are enabled since they will most probably all be involved in a move
            // Since segments could involve different axis this reduces load when switching segments and
            // makes disabling easier.
            Printer::enableXStepper();
            Printer::enableYStepper();
            Printer::enableZStepper();

            // Copy across movement into main direction flags so that endstops function correctly
            cur->dir |= curd->dir;
            // Initialize bresenham for the first segment
            cur->error[X_AXIS] = cur->error[Y_AXIS] = cur->error[Z_AXIS] = cur->numPrimaryStepPerSegment >> 1;
            curd_errupd = cur->numPrimaryStepPerSegment;
            stepsPerSegRemaining = cur->numPrimaryStepPerSegment;
        }
        else curd = NULL;
        cur_errupd = cur->stepsRemaining;

        if(!cur->areParameterUpToDate())  // should never happen, but with bad timings???
        {
            cur->updateStepsParameter();
        }
        Printer::vMaxReached = cur->vStart;
        Printer::stepNumber = 0;
        Printer::timer = 0;
        HAL::forbidInterrupts();
        //Determine direction of movement
        if (curd)
        {
            Printer::setXDirection(curd->isXPositiveMove());
            Printer::setYDirection(curd->isYPositiveMove());
            Printer::setZDirection(curd->isZPositiveMove());
        }
#if USE_ADVANCE
        if(!Printer::isAdvanceActivated()) // Set direction if no advance enabled
#endif
            Extruder::setDirection(cur->isEPositiveMove());
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
        HAL::delayMicroseconds(DIRECTION_DELAY);
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        Printer::advanceExecuted = cur->advanceStart;
#endif
        cur->updateAdvanceSteps(cur->vStart, 0, false);
#endif
        if(Printer::mode == PRINTER_MODE_FFF) {
            Printer::setFanSpeedDirectly(cur->secondSpeed);
        }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        else if(Printer::mode == PRINTER_MODE_LASER)
        {
            LaserDriver::changeIntensity(cur->secondSpeed);
        }
#endif
        return Printer::interval; // Wait an other 50% from last step to make the 100% full
    } // End cur=0
    HAL::allowInterrupts();

    if(curd != NULL)
    {
        curd->checkEndstops(cur,(cur->isCheckEndstops()));
    }
    int maxLoops = (Printer::stepsPerTimerCall <= cur->stepsRemaining ? Printer::stepsPerTimerCall : cur->stepsRemaining);
    HAL::forbidInterrupts();
    for(int loop = 0; loop < maxLoops; loop++)
    {
#if STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY
        if(loop > 0)
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY);
#endif
        if((cur->error[E_AXIS] -= cur->delta[E_AXIS]) < 0)
        {
#if USE_ADVANCE
            if(Printer::isAdvanceActivated())   // Use interrupt for movement
            {
                if(cur->isEPositiveMove())
                    Printer::extruderStepsNeeded++;
                else
                    Printer::extruderStepsNeeded--;
            }
            else
#endif
                Extruder::step();
            cur->error[E_AXIS] += cur_errupd;
        }
        if (curd)
        {
            // Take delta steps
            if(curd->isXMove())
                if((cur->error[X_AXIS] -= curd->deltaSteps[A_TOWER]) < 0)
                {
                    cur->startXStep();
                    cur->error[X_AXIS] += curd_errupd;
#ifdef DEBUG_REAL_POSITION
                    Printer::realDeltaPositionSteps[A_TOWER] += curd->isXPositiveMove() ? 1 : -1;
#endif
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }

            if(curd->isYMove())
                if((cur->error[Y_AXIS] -= curd->deltaSteps[B_TOWER]) < 0)
                {
                    cur->startYStep();
                    cur->error[Y_AXIS] += curd_errupd;
#ifdef DEBUG_REAL_POSITION
                    Printer::realDeltaPositionSteps[B_TOWER] += curd->isYPositiveMove() ? 1 : -1;
#endif
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }

            if(curd->isZMove())
                if((cur->error[Z_AXIS] -= curd->deltaSteps[C_TOWER]) < 0)
                {
                    cur->startZStep();
                    cur->error[Z_AXIS] += curd_errupd;
                    Printer::realDeltaPositionSteps[C_TOWER] += curd->isZPositiveMove() ? 1 : -1;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }
            stepsPerSegRemaining--;
            if (!stepsPerSegRemaining)
            {
                if (cur->numDeltaSegments)
                {
                    if(FEATURE_BABYSTEPPING && Printer::zBabystepsMissing/* && curd
                            && (curd->dir & XYZ_STEP) == XYZ_STEP*/)
                    {
                        // execute a extra babystep
                        //Printer::insertStepperHighDelay();
                        //Printer::endXYZSteps();
                        //HAL::delayMicroseconds(STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY + 1);
                        Printer::zBabystep();
                    }
                    // Get the next delta segment
                    curd = &cur->segments[--cur->numDeltaSegments];

                    // Initialize bresenham for this segment (numPrimaryStepPerSegment is already correct for the half step setting)
                    cur->error[X_AXIS] = cur->error[Y_AXIS] = cur->error[Z_AXIS] = cur->numPrimaryStepPerSegment >> 1;

                    // Reset the counter of the primary steps. This is initialized in the line
                    // generation so don't have to do this the first time.
                    stepsPerSegRemaining = cur->numPrimaryStepPerSegment;

                    // Change direction if necessary
                    Printer::setXDirection(curd->dir & X_DIRPOS);
                    Printer::setYDirection(curd->dir & Y_DIRPOS);
                    Printer::setZDirection(curd->dir & Z_DIRPOS);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
                    HAL::delayMicroseconds(DIRECTION_DELAY);
#endif

                }
                else
                    curd = 0;// Release the last segment
                //deltaSegmentCount--;
            }
        }
#if CPU_ARCH != ARCH_AVR
        if(loop < maxLoops - 1)
        {
#endif
            Printer::insertStepperHighDelay();
            Printer::endXYZSteps();
#if USE_ADVANCE
            if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
                Extruder::unstep();
#if CPU_ARCH != ARCH_AVR
        }
#endif
    } // for loop

    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#if RAMP_ACCELERATION
//If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (cur->moveAccelerating())
    {
        Printer::vMaxReached = HAL::ComputeV(Printer::timer, cur->fAcceleration) + cur->vStart;
        if(Printer::vMaxReached > cur->vMax) Printer::vMaxReached = cur->vMax;
        speed_t v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
        cur->updateAdvanceSteps(Printer::vMaxReached, maxLoops, true);
        Printer::stepNumber += maxLoops; // is only used by moveAccelerating
    }
    else if (cur->moveDecelerating())     // time to slow down
    {
        speed_t v = HAL::ComputeV(Printer::timer, cur->fAcceleration);
        if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
            v = cur->vEnd;
        else
        {
            v = Printer::vMaxReached - v;
            if (v < cur->vEnd) v = cur->vEnd; // extra steps at the end of desceleration due to rounding erros
        }
        cur->updateAdvanceSteps(v, maxLoops, false);
        v = Printer::updateStepsPerTimerCall(v);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
    }
    else
    {
        // If we had acceleration, we need to use the latest vMaxReached and interval
        // If we started full speed, we need to use cur->fullInterval and vMax
        cur->updateAdvanceSteps((!cur->accelSteps ? cur->vMax : Printer::vMaxReached), 0, true);
        if(!cur->accelSteps)
        {
            if(cur->vMax > STEP_DOUBLER_FREQUENCY)
            {
#if ALLOW_QUADSTEPPING
                if(cur->vMax > STEP_DOUBLER_FREQUENCY * 2)
                {
                    Printer::stepsPerTimerCall = 4;
                    Printer::interval = cur->fullInterval << 2;
                }
                else
                {
                    Printer::stepsPerTimerCall = 2;
                    Printer::interval = cur->fullInterval << 1;
                }
#else
                Printer::stepsPerTimerCall = 2;
                Printer::interval = cur->fullInterval << 1;
#endif
            }
            else
            {
                Printer::stepsPerTimerCall = 1;
                Printer::interval = cur->fullInterval;
            }
        }
    }
#else
    Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif
    PrintLine::cur->stepsRemaining -= maxLoops;

    if(cur->stepsRemaining <= 0 || cur->isNoMove())   // line finished
    {
        // Release remaining delta segments
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining || cur->numDeltaSegments)
        {
            Com::printFLN(PSTR("Missed steps:"), cur->totalStepsRemaining);
            Com::printFLN(PSTR("Step/seg r:"), stepsPerSegRemaining);
            Com::printFLN(PSTR("NDS:"), (int) cur->numDeltaSegments);
        }
#endif
        //HAL::forbidInterrupts();
        //deltaSegmentCount -= cur->numDeltaSegments; // should always be zero
        removeCurrentLineForbidInterrupt();
        Printer::disableAllowedStepper();
        if(linesCount == 0) {
            UI_STATUS_F(Com::translatedF(UI_TEXT_IDLE_ID));
            if(Printer::mode == PRINTER_MODE_FFF) {
                Printer::setFanSpeedDirectly(Printer::fanSpeed);
            }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            else if(Printer::mode == PRINTER_MODE_LASER) // Last move disables laser for safety!
            {
                LaserDriver::changeIntensity(0);
            }
#endif
        }
        Printer::interval >>= 1; // 50% of time to next call to do cur=0
        DEBUG_MEMORY;
    } // Do even
#if CPU_ARCH != ARCH_AVR
    Printer::insertStepperHighDelay();
    Printer::endXYZSteps();
#if USE_ADVANCE
    if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
        Extruder::unstep();
#endif
    return Printer::interval;
}
#else
/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.

  Normal non delta algorithm
*/
int lastblk = -1;
int32_t cur_errupd;
int32_t PrintLine::bresenhamStep() // version for cartesian printer
{
#if CPU_ARCH == ARCH_ARM
    if(!PrintLine::nlFlag)
#else
    if(cur == NULL)
#endif
    {
        ANALYZER_ON(ANALYZER_CH0);
        setCurrentLine();
        if(cur->isBlocked())   // This step is in computation - shouldn't happen
        {
            /*if(lastblk!=(int)cur) // can cause output errors!
            {
                HAL::allowInterrupts();
                lastblk = (int)cur;
                Com::printFLN(Com::tBLK,lines_count);
            }*/
            cur = NULL;
#if CPU_ARCH==ARCH_ARM
            PrintLine::nlFlag = false;
#endif
            return 2000;
        }
        HAL::allowInterrupts();
        lastblk = -1;
#if INCLUDE_DEBUG_NO_MOVE
        if(Printer::debugNoMoves())   // simulate a move, but do nothing in reality
        {
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif
        ANALYZER_OFF(ANALYZER_CH0);
        if(cur->isWarmUp())
        {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(linesCount<=cur->getWaitForXLinesFilled())
            {
                cur = NULL;
#if CPU_ARCH == ARCH_ARM
                PrintLine::nlFlag = false;
#endif
                return 2000;
            }
            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        } // End if WARMUP
        //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
#if GANTRY
#if DRIVE_SYSTEM == XY_GANTRY || DRIVE_SYSTEM == YX_GANTRY
        if(cur->isXOrYMove())
        {
            Printer::enableXStepper();
            Printer::enableYStepper();
        }
        if(cur->isZMove()) Printer::enableZStepper();
#else // XZ / ZX Gantry
        if(cur->isXOrZMove())
        {
            Printer::enableXStepper();
            Printer::enableZStepper();
        }
        if(cur->isYMove()) Printer::enableYStepper();
#endif
#else
        if(cur->isXMove()) Printer::enableXStepper();
        if(cur->isYMove()) Printer::enableYStepper();
        if(cur->isZMove()) Printer::enableZStepper();
#endif
        if(cur->isEMove()) Extruder::enable();
        cur->fixStartAndEndSpeed();
        HAL::allowInterrupts();
        cur_errupd = cur->delta[cur->primaryAxis];
        if(!cur->areParameterUpToDate())  // should never happen, but with bad timings???
        {
            cur->updateStepsParameter();
        }
        Printer::vMaxReached = cur->vStart;
        Printer::stepNumber=0;
        Printer::timer = 0;
        HAL::forbidInterrupts();
        //Determine direction of movement,check if endstop was hit
#if !(GANTRY)
        Printer::setXDirection(cur->isXPositiveMove());
        Printer::setYDirection(cur->isYPositiveMove());
        Printer::setZDirection(cur->isZPositiveMove());
#else // Any gantry type
        long gdx = (cur->dir & X_DIRPOS ? cur->delta[X_AXIS] : -cur->delta[X_AXIS]); // Compute signed difference in steps
#if DRIVE_SYSTEM == XY_GANTRY || DRIVE_SYSTEM == YX_GANTRY
        Printer::setZDirection(cur->isZPositiveMove());
        long gdy = (cur->dir & Y_DIRPOS ? cur->delta[Y_AXIS] : -cur->delta[Y_AXIS]);
        Printer::setXDirection(gdx + gdy >= 0);
#if DRIVE_SYSTEM == XY_GANTRY
        Printer::setYDirection(gdx > gdy);
#else
        Printer::setYDirection(gdx <= gdy);
#endif
#else // XZ or ZX core
        Printer::setYDirection(cur->isYPositiveMove());
        long gdz = (cur->dir & Z_DIRPOS ? cur->delta[Z_AXIS] : -cur->delta[Z_AXIS]);
        Printer::setXDirection(gdx + gdz >= 0);
#if DRIVE_SYSTEM == XZ_GANTRY
        Printer::setZDirection(gdx > gdz);
#else
        Printer::setZDirection(gdx <= gdz);
#endif
#endif // YZ or ZY Gantry
#endif // GANTRY
#if USE_ADVANCE
        if(!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
            Extruder::setDirection(cur->isEPositiveMove());
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
        HAL::delayMicroseconds(DIRECTION_DELAY);
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        Printer::advanceExecuted = cur->advanceStart;
#endif
        cur->updateAdvanceSteps(cur->vStart, 0, false);
#endif
        if(Printer::mode == PRINTER_MODE_FFF) {
            Printer::setFanSpeedDirectly(cur->secondSpeed);
        }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        else if(Printer::mode == PRINTER_MODE_LASER)
        {
            LaserDriver::changeIntensity(cur->secondSpeed);
        }
#endif
        return Printer::interval; // Wait an other 50% from last step to make the 100% full
    } // End cur=0
    cur->checkEndstops();
	fast8_t max_loops = Printer::stepsPerTimerCall;
	if(cur->stepsRemaining < max_loops)
		max_loops = cur->stepsRemaining;
    for(fast8_t loop = 0; loop < max_loops; loop++)
    {
#if STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY > 0
        if(loop)
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY);
#endif
        if((cur->error[E_AXIS] -= cur->delta[E_AXIS]) < 0)
        {
#if USE_ADVANCE
            if(Printer::isAdvanceActivated())   // Use interrupt for movement
            {
                if(cur->isEPositiveMove())
                    Printer::extruderStepsNeeded++;
                else
                    Printer::extruderStepsNeeded--;
            }
            else
#endif
                Extruder::step();
            cur->error[E_AXIS] += cur_errupd;
        }
#if CPU_ARCH == ARCH_AVR		
        if(cur->isXMove())
#endif
            if((cur->error[X_AXIS] -= cur->delta[X_AXIS]) < 0)
            {
                cur->startXStep();
                cur->error[X_AXIS] += cur_errupd;
            }
#if CPU_ARCH == ARCH_AVR
        if(cur->isYMove())
#endif
            if((cur->error[Y_AXIS] -= cur->delta[Y_AXIS]) < 0)
            {
                cur->startYStep();
                cur->error[Y_AXIS] += cur_errupd;
            }
#if CPU_ARCH == ARCH_AVR
        if(cur->isZMove())
#endif
            if((cur->error[Z_AXIS] -= cur->delta[Z_AXIS]) < 0)
            {
                cur->startZStep();
                cur->error[Z_AXIS] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
                cur->totalStepsRemaining--;
#endif
            }
#if (GANTRY)
#if DRIVE_SYSTEM == XY_GANTRY || DRIVE_SYSTEM == YX_GANTRY
        Printer::executeXYGantrySteps();
#else
        Printer::executeXZGantrySteps();
#endif
#endif
        Printer::insertStepperHighDelay();
#if USE_ADVANCE
        if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
		cur->stepsRemaining--;
        Extruder::unstep();
        Printer::endXYZSteps();
    } // for loop
    HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#if RAMP_ACCELERATION
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (cur->moveAccelerating())   // we are accelerating
    {
        Printer::vMaxReached = HAL::ComputeV(Printer::timer,cur->fAcceleration) + cur->vStart;
        if(Printer::vMaxReached > cur->vMax) Printer::vMaxReached = cur->vMax;
        unsigned int v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
        cur->updateAdvanceSteps(Printer::vMaxReached, max_loops, true);
        Printer::stepNumber += max_loops; // only used for moveAccelerating
    }
    else if (cur->moveDecelerating())     // time to slow down
    {
        unsigned int v = HAL::ComputeV(Printer::timer,cur->fAcceleration);
        if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
            v = cur->vEnd;
        else
        {
            v = Printer::vMaxReached - v;
            if (v<cur->vEnd) v = cur->vEnd; // extra steps at the end of desceleration due to rounding erros
        }
        cur->updateAdvanceSteps(v,max_loops,false); // needs original v
        v = Printer::updateStepsPerTimerCall(v);
        Printer::interval = HAL::CPUDivU2(v);
        Printer::timer += Printer::interval;
    }
    else // full speed reached
    {
        cur->updateAdvanceSteps((!cur->accelSteps ? cur->vMax : Printer::vMaxReached), 0, true);
        // constant speed reached
        if(cur->vMax > STEP_DOUBLER_FREQUENCY)
        {
#if ALLOW_QUADSTEPPING
            if(cur->vMax > STEP_DOUBLER_FREQUENCY * 2)
            {
                Printer::stepsPerTimerCall = 4;
                Printer::interval = cur->fullInterval << 2;
            }
            else
            {
                Printer::stepsPerTimerCall = 2;
                Printer::interval = cur->fullInterval << 1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            Printer::interval = cur->fullInterval << 1;
#endif
        }
        else
        {
            Printer::stepsPerTimerCall = 1;
            Printer::interval = cur->fullInterval;
        }
    }
#else
    Printer::stepsPerTimerCall = 1;
    Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif // RAMP_ACCELERATION    
    long interval = Printer::interval;
    if(cur->stepsRemaining <= 0 || cur->isNoMove())   // line finished
    {
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining)
        {
            Com::printF(Com::tDBGMissedSteps, cur->totalStepsRemaining);
            Com::printFLN(Com::tComma, cur->stepsRemaining);
        }
#endif
        removeCurrentLineForbidInterrupt();
        Printer::disableAllowedStepper();
        if(linesCount == 0)
        {
            UI_STATUS_F(Com::translatedF(UI_TEXT_IDLE_ID));
            if(Printer::mode == PRINTER_MODE_FFF) {
                Printer::setFanSpeedDirectly(Printer::fanSpeed);
            }
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            else if(Printer::mode == PRINTER_MODE_LASER) // Last move disables laser for safety!
            {
                LaserDriver::changeIntensity(0);
            }
#endif
        }
        interval = Printer::interval = interval >> 1; // 50% of time to next call to do cur=0
        DEBUG_MEMORY;
    } // Do even
    if(FEATURE_BABYSTEPPING && Printer::zBabystepsMissing)
    {
		HAL::forbidInterrupts();
        Printer::zBabystep();
    }
    return interval;
}
#endif
