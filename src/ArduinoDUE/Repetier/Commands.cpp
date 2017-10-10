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

#include "Repetier.h"

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
    while(true) {
#ifdef DEBUG_PRINT
        debugWaitLoop = 1;
#endif
        if(!Printer::isBlockingReceive()) {
            GCode::readFromSerial();
            GCode *code = GCode::peekCurrentCommand();
            //UI_SLOW; // do longer timed user interface action
            UI_MEDIUM; // do check encoder
            if(code) {
#if SDSUPPORT
                if(sd.savetosd) {
                    if(!(code->hasM() && code->M == 29))   // still writing to file
                        sd.writeCommand(code);
                    else
                        sd.finishWrite();
#if ECHO_ON_EXECUTE
                    code->echoCommand();
#endif
                } else
#endif
                Commands::executeGCode(code);
                code->popCurrentCommand();
            }
        } else {
            GCode::keepAlive(Paused);
            UI_MEDIUM;
        }
        Printer::defaultLoopActions();
    }
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {
    Printer::handleInterruptEvent();
    EVENT_PERIODICAL;
    if(!executePeriodical) return; // gets true every 100ms
    executePeriodical = 0;
    EVENT_TIMER_100MS;
    Extruder::manageTemperatures();
    if(--counter500ms == 0) {
        if(manageMonitor)
            writeMonitor();
        counter500ms = 5;
        EVENT_TIMER_500MS;
    }
    // If called from queueDelta etc. it is an error to start a new move since it
    // would invalidate old computation resulting in unpredicted behavior.
    // lcd controller can start new moves, so we disallow it if called from within
    // a move command.
    UI_SLOW(allowNewMoves);
}

/** \brief Waits until movement cache is empty.

Some commands expect no movement, before they can execute. This function
waits, until the steppers are stopped. In the meanwhile it buffers incoming
commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves() {
#ifdef DEBUG_PRINT
    debugWaitLoop = 8;
#endif
    while(PrintLine::hasLines()) {
        GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(Processing);
        UI_MEDIUM;
    }
}

void Commands::waitUntilEndOfAllBuffers() {
    GCode *code = NULL;
#ifdef DEBUG_PRINT
    debugWaitLoop = 9;
#endif
    while(PrintLine::hasLines() || (code != NULL)) {
        GCode::readFromSerial();
        code = GCode::peekCurrentCommand();
        UI_MEDIUM; // do check encoder
        if(code) {
#if SDSUPPORT
            if(sd.savetosd) {
                if(!(code->hasM() && code->M == 29))   // still writing to file
                    sd.writeCommand(code);
                else
                    sd.finishWrite();
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif

            Commands::executeGCode(code);
            code->popCurrentCommand();
        }
        Commands::checkForPeriodicalActions(false); // only called from memory
        UI_MEDIUM;
    }
}

void Commands::printCurrentPosition(FSTRINGPARAM(s)) {
    float x, y, z;
    Printer::realPosition(x, y, z);
    if (isnan(x) || isinf(x) || isnan(y) || isinf(y) || isnan(z) || isinf(z)) {
        Com::printErrorFLN(s); // flag where the error condition came from
    }
    x += Printer::coordinateOffset[X_AXIS];
    y += Printer::coordinateOffset[Y_AXIS];
    z += Printer::coordinateOffset[Z_AXIS];
    Com::printF(Com::tXColon, x, 2);
    Com::printF(Com::tSpaceYColon, y, 2);
    Com::printF(Com::tSpaceZColon, z, 3);
    Com::printFLN(Com::tSpaceEColon, Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS], 4);
    //Com::printF(PSTR("OffX:"),Printer::offsetX); // to debug offset handling
    //Com::printFLN(PSTR(" OffY:"),Printer::offsetY);
}

void Commands::printTemperatures(bool showRaw) {
#if NUM_EXTRUDER > 0
    float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE == 0
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
#else
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
    Com::printF(Com::tSpaceBColon,Extruder::getHeatedBedTemperature());
    Com::printF(Com::tSpaceSlash,heatedBedController.targetTemperatureC,0);
    if(showRaw) {
        Com::printF(Com::tSpaceRaw,(int)NUM_EXTRUDER);
        Com::printF(Com::tColon,(1023 << (2 - ANALOG_REDUCE_BITS)) - heatedBedController.currentTemperature);
    }
    Com::printF(Com::tSpaceBAtColon,(pwm_pos[heatedBedController.pwmIndex])); // Show output of auto tune when tuning!
#endif
#if TEMP_PID
    Com::printF(Com::tSpaceAtColon,(autotuneIndex == 255 ? pwm_pos[Extruder::current->id] : pwm_pos[autotuneIndex])); // Show output of auto tune when tuning!
#endif
#if NUM_EXTRUDER > 1
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        Com::printF(Com::tSpaceT,(int)i);
        Com::printF(Com::tColon,extruder[i].tempControl.currentTemperatureC);
        Com::printF(Com::tSpaceSlash,extruder[i].tempControl.targetTemperatureC,0);
#if TEMP_PID
        Com::printF(Com::tSpaceAt,(int)i);
        Com::printF(Com::tColon,(pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!
#endif
        if(showRaw) {
            Com::printF(Com::tSpaceRaw,(int)i);
            Com::printF(Com::tColon,(1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[i].tempControl.currentTemperature);
        }
    }
#elif NUM_EXTRUDER == 1
    if(showRaw) {
        Com::printF(Com::tSpaceRaw,(int)0);
        Com::printF(Com::tColon,(1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[0].tempControl.currentTemperature);
    }
#endif
    Com::println();
#endif
}
void Commands::changeFeedrateMultiply(int factor) {
    if(factor < 25) factor = 25;
    if(factor > 500) factor = 500;
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

void Commands::changeFlowrateMultiply(int factor) {
    if(factor < 25) factor = 25;
    if(factor > 200) factor = 200;
    Printer::extrudeMultiply = factor;
    if(Extruder::current->diameter <= 0)
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
    else
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
    Com::printFLN(Com::tFlowMultiply, factor);
}

#if FEATURE_FAN_CONTROL
uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
uint8_t fan2Kickstart;
#endif

void Commands::setFanSpeed(int speed, bool immediately) {
#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL
    if(Printer::fanSpeed == speed)
        return;
    speed = constrain(speed,0,255);
    Printer::setMenuMode(MENU_MODE_FAN_RUNNING,speed != 0);
    Printer::fanSpeed = speed;
    if(PrintLine::linesCount == 0 || immediately) {
        for(fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++)
            PrintLine::lines[i].secondSpeed = speed; // fill all printline buffers with new fan speed value
        Printer::setFanSpeedDirectly(speed);
    }
    Com::printFLN(Com::tFanspeed,speed); // send only new values to break update loops!
#endif
}

void Commands::setFan2Speed(int speed) {
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
    speed = constrain(speed,0,255);
    Printer::setFan2SpeedDirectly(speed);
    Com::printFLN(Com::tFan2speed,speed); // send only new values to break update loops!
#endif
}

void Commands::reportPrinterUsage() {
#if EEPROM_MODE != 0
    float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
    Com::printF(Com::tPrintedFilament, dist, 2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
#if NUM_EXTRUDER > 0
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;
#endif
    int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
    int32_t tmp = seconds / 86400;
    seconds -= tmp * 86400;
    Com::printF(Com::tPrintingTime,tmp);
    tmp = seconds / 3600;
    Com::printF(Com::tSpaceDaysSpace,tmp);
    seconds -= tmp * 3600;
    tmp = seconds / 60;
    Com::printF(Com::tSpaceHoursSpace,tmp);
    Com::printFLN(Com::tSpaceMin);
#endif
}

#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstepMS(uint8_t driver, int8_t ms1, int8_t ms2) {
    if(ms1 > -1) switch(driver) {
            case 0:
#if X_MS1_PIN > -1
                WRITE( X_MS1_PIN,ms1);
#endif
                break;
            case 1:
#if Y_MS1_PIN > -1
                WRITE( Y_MS1_PIN,ms1);
#endif
                break;
            case 2:
#if Z_MS1_PIN > -1
                WRITE( Z_MS1_PIN,ms1);
#endif
                break;
            case 3:
#if E0_MS1_PIN > -1
                WRITE(E0_MS1_PIN,ms1);
#endif
                break;
            case 4:
#if E1_MS1_PIN > -1
                WRITE(E1_MS1_PIN,ms1);
#endif
                break;
        }
    if(ms2 > -1) switch(driver) {
            case 0:
#if X_MS2_PIN > -1
                WRITE( X_MS2_PIN,ms2);
#endif
                break;
            case 1:
#if Y_MS2_PIN > -1
                WRITE( Y_MS2_PIN,ms2);
#endif
                break;
            case 2:
#if Z_MS2_PIN > -1
                WRITE( Z_MS2_PIN,ms2);
#endif
                break;
            case 3:
#if E0_MS2_PIN > -1
                WRITE(E0_MS2_PIN,ms2);
#endif
                break;
            case 4:
#if E1_MS2_PIN > -1
                WRITE(E1_MS2_PIN,ms2);
#endif
                break;
        }
}

void microstepMode(uint8_t driver, uint8_t stepping_mode) {
    switch(stepping_mode) {
        case 1:
            microstepMS(driver,MICROSTEP1);
            break;
        case 2:
            microstepMS(driver,MICROSTEP2);
            break;
        case 4:
            microstepMS(driver,MICROSTEP4);
            break;
        case 8:
            microstepMS(driver,MICROSTEP8);
            break;
        case 16:
            microstepMS(driver,MICROSTEP16);
            break;
        case 32:
            microstepMS(driver,MICROSTEP32);
            break;
    }
}

void microstepReadings() {
    Com::printFLN(Com::tMS1MS2Pins);
#if X_MS1_PIN > -1 && X_MS2_PIN > -1
    Com::printF(Com::tXColon,READ(X_MS1_PIN));
    Com::printFLN(Com::tComma,READ(X_MS2_PIN));
#elif X_MS1_PIN > -1
    Com::printFLN(Com::tXColon,READ(X_MS1_PIN));
#endif
#if Y_MS1_PIN > -1 && Y_MS2_PIN > -1
    Com::printF(Com::tYColon,READ(Y_MS1_PIN));
    Com::printFLN(Com::tComma,READ(Y_MS2_PIN));
#elif Y_MS1_PIN > -1
    Com::printFLN(Com::tYColon,READ(Y_MS1_PIN));
#endif
#if Z_MS1_PIN > -1 && Z_MS2_PIN > -1
    Com::printF(Com::tZColon,READ(Z_MS1_PIN));
    Com::printFLN(Com::tComma,READ(Z_MS2_PIN));
#elif Z_MS1_PIN > -1
    Com::printFLN(Com::tZColon,READ(Z_MS1_PIN));
#endif
#if E0_MS1_PIN > -1 && E0_MS2_PIN > -1
    Com::printF(Com::tE0Colon,READ(E0_MS1_PIN));
    Com::printFLN(Com::tComma,READ(E0_MS2_PIN));
#elif E0_MS1_PIN > -1
    Com::printFLN(Com::tE0Colon,READ(E0_MS1_PIN));
#endif
#if E1_MS1_PIN > -1 && E1_MS2_PIN > -1
    Com::printF(Com::tE1Colon,READ(E1_MS1_PIN));
    Com::printFLN(Com::tComma,READ(E1_MS2_PIN));
#elif E1_MS1_PIN > -1
    Com::printFLN(Com::tE1Colon,READ(E1_MS1_PIN));
#endif
}
#endif

void microstepInit() {
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
#if X_MS1_PIN > -1
    SET_OUTPUT(X_MS1_PIN);
#endif
#if Y_MS1_PIN > -1
    SET_OUTPUT(Y_MS1_PIN);
#endif
#if Z_MS1_PIN > -1
    SET_OUTPUT(Z_MS1_PIN);
#endif
#if E0_MS1_PIN > -1
    SET_OUTPUT(E0_MS1_PIN);
#endif
#if E1_MS1_PIN > -1
    SET_OUTPUT(E1_MS1_PIN);
#endif
#if X_MS2_PIN > -1
    SET_OUTPUT(X_MS2_PIN);
#endif
#if Y_MS2_PIN > -1
    SET_OUTPUT(Y_MS2_PIN);
#endif
#if Z_MS2_PIN > -1
    SET_OUTPUT(Z_MS2_PIN);
#endif
#if E0_MS2_PIN > -1
    SET_OUTPUT(E0_MS2_PIN);
#endif
#if E1_MS2_PIN > -1
    SET_OUTPUT(E1_MS2_PIN);
#endif
    for(int i = 0; i <= 4; i++) microstepMode(i, microstep_modes[i]);
#endif
}

/**
\brief Execute the Arc command stored in com.
*/
#if ARC_SUPPORT
void Commands::processArc(GCode *com) {
    float position[Z_AXIS_ARRAY];
    Printer::realPosition(position[X_AXIS],position[Y_AXIS],position[Z_AXIS]);
    if(!Printer::setDestinationStepsFromGCode(com)) return; // For X Y Z E F
    float offset[2] = {(com->hasI() ? com->I : 0),(com->hasJ() ? com->J : 0)};
    float target[E_AXIS_ARRAY] = {Printer::realXPosition(),Printer::realYPosition(),Printer::realZPosition(),Printer::destinationSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]};
    float r;
    if (com->hasR()) {
        /*
        We need to calculate the center of the circle that has the designated radius and passes
        through both the current position and the target position. This method calculates the following
        set of equations where [x,y] is the vector from current to target position, d == magnitude of
        that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
        the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
        length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
        [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

        d^2 == x^2 + y^2
        h^2 == r^2 - (d/2)^2
        i == x/2 - y/d*h
        j == y/2 + x/d*h

        O <- [i,j]
        -  |
        r      -     |
        -        |
        -           | h
        -              |
        [0,0] ->  C -----------------+--------------- T  <- [x,y]
        | <------ d/2 ---->|

        C - Current position
        T - Target position
        O - center of circle that pass through both C and T
        d - distance from C to T
        r - designated radius
        h - distance from center of CT to O

        Expanding the equations:

        d -> sqrt(x^2 + y^2)
        h -> sqrt(4 * r^2 - x^2 - y^2)/2
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

        Which can be written:

        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

        Which we for size and speed reasons optimize to:

        h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
        i = (x - (y * h_x2_div_d))/2
        j = (y + (x * h_x2_div_d))/2

        */
        r = com->R;
        // Calculate the change in position along each selected axis
        double x = target[X_AXIS]-position[X_AXIS];
        double y = target[Y_AXIS]-position[Y_AXIS];

        double h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
        // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
        // real CNC, and thus - for practical reasons - we will terminate promptly:
        if(isnan(h_x2_div_d)) {
            Com::printErrorFLN(Com::tInvalidArc);
            return;
        }
        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
        if (com->G == 3) {
            h_x2_div_d = -h_x2_div_d;
        }

        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
        the left hand circle will be generated - when it is negative the right hand circle is generated.


        T  <-- Target position

        ^
        Clockwise circles with this center         |          Clockwise circles with this center will have
        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
        \         |          /
        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
        |
        |

        C  <-- Current position                                 */


        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
        // even though it is advised against ever generating such circles in a single line of g-code. By
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the unadvisably long arcs as prescribed.
        if (r < 0) {
            h_x2_div_d = -h_x2_div_d;
            r = -r; // Finished with r. Set to positive for mc_arc
        }
        // Complete the operation by calculating the actual center of the arc
        offset[0] = 0.5 * (x - (y * h_x2_div_d));
        offset[1] = 0.5 * (y + (x * h_x2_div_d));

    } else { // Offset mode specific computations
        r = hypot(offset[0], offset[1]); // Compute arc radius for arc
    }
    // Set clockwise/counter-clockwise sign for arc computations
    uint8_t isclockwise = com->G == 2;
    // Trace the arc
    PrintLine::arc(position, target, offset, r, isclockwise);
}
#endif
extern bool runBedLeveling(GCode *com);
/**
\brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode *com) {
    uint32_t codenum; //throw away variable
    switch(com->G) {
        case 0: // G0 -> G1
        case 1: // G1
                if(com->hasS()) Printer::setNoDestinationCheck(com->S != 0);
                if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
                    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
#if UI_HAS_KEYS
                // ui can only execute motion commands if we are not waiting inside a move for an
                // old move to finish. For normal response times, we always leave one free after
                // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
                // gets filled while waiting, the lost is neglectible.
                PrintLine::waitForXFreeLines(1, true);
#endif // UI_HAS_KEYS
#ifdef DEBUG_QUEUE_MOVE
                {

                    InterruptProtectedBlock noInts;
                    int lc = (int)PrintLine::linesCount;
                    int lp = (int)PrintLine::linesPos;
                    int wp = (int)PrintLine::linesWritePos;
                    int n = (wp - lp);
                    if(n < 0) n += PRINTLINE_CACHE_SIZE;
                    noInts.unprotect();
                    if(n != lc)
                        Com::printFLN(PSTR("Buffer corrupted"));
                }
#endif
            break;
#if ARC_SUPPORT
        case 2: // CW Arc
        case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
                processArc(com);
            break;
#endif
        case 4: // G4 dwell
            Commands::waitUntilEndOfAllMoves();
            codenum = 0;
            if(com->hasP()) codenum = com->P; // milliseconds to wait
            if(com->hasS()) codenum = com->S * 1000; // seconds to wait
            codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting
            while((uint32_t)(codenum-HAL::timeInMilliseconds())  < 2000000000 ) {
                GCode::readFromSerial();
                Commands::checkForPeriodicalActions(true);
            }
            break;
#if FEATURE_RETRACTION && NUM_EXTRUDER > 0
        case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament accoriding to stored setting
#if NUM_EXTRUDER > 1
            Extruder::current->retract(true, com->hasS() && com->S > 0);
#else
            Extruder::current->retract(true, false);
#endif
            break;
        case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
#if NUM_EXTRUDER > 1
            Extruder::current->retract(false, com->hasS() && com->S > 0);
#else
            Extruder::current->retract(false, false);
#endif
            break;
#endif // FEATURE_RETRACTION
        case 50028: { //G28 Home all Axis one at a time
                uint8_t homeAllAxis = (com->hasNoXYZ() && !com->hasE());
                if(com->hasE())
                    Printer::currentPositionSteps[E_AXIS] = 0;
                if(homeAllAxis || !com->hasNoXYZ())
                    Printer::homeAxis(homeAllAxis || com->hasX(),homeAllAxis || com->hasY(),homeAllAxis || com->hasZ());
                Printer::updateCurrentPosition();
            }
            break;
#if FEATURE_Z_PROBE
        case 29: { // G29 3 points, build average or distortion compensation
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
                float actTemp[NUM_EXTRUDER];
                for(int i = 0; i < NUM_EXTRUDER; i++)
                    actTemp[i] = extruder[i].tempControl.targetTemperatureC;
                Printer::moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE,RMath::max(EEPROM::zProbeHeight(),static_cast<float>(ZHOME_HEAT_HEIGHT)),IGNORE_COORDINATE,Printer::homingFeedrate[Z_AXIS]);
                Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,false);
                }
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,true);
                }
#else
                if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),Extruder::current->id,false,true);
#endif
#endif
                bool ok = true;
                Printer::startProbing(true);
                bool oldAutolevel = Printer::isAutolevelActive();
                Printer::setAutolevelActive(false);
                float sum = 0, last,oldFeedrate = Printer::feedrate;
                Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
                sum = Printer::runZProbe(true,false,Z_PROBE_REPETITIONS,false);
                if(sum == ILLEGAL_Z_PROBE) ok = false;
                if(ok) {
                    Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
                    last = Printer::runZProbe(false,false);
                    if(last == ILLEGAL_Z_PROBE) ok = false;
                    sum+= last;
                }
                if(ok) {
                    Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
                    last = Printer::runZProbe(false,true);
                    if(last == ILLEGAL_Z_PROBE) ok = false;
                    sum += last;
                }
                if(ok) {
                    sum *= 0.33333333333333;
                    Com::printFLN(Com::tZProbeAverage, sum);
                    if(com->hasS() && com->S) {
#if MAX_HARDWARE_ENDSTOP_Z
                        Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                        float zup = Printer::runZMaxProbe();
                        if(zup == ILLEGAL_Z_PROBE) {
                            ok = false;
                        } else
                        Printer::zLength = zup + sum - ENDSTOP_Z_BACK_ON_HOME;
                        Com::printInfoFLN(Com::tZProbeZReset);
                        Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#else
                        Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                        Com::printFLN(PSTR("Adjusted z origin"));
#endif // max z endstop
                    }
                    Printer::feedrate = oldFeedrate;
                    Printer::setAutolevelActive(oldAutolevel);
                    if(ok && com->hasS() && com->S == 2)
                        EEPROM::storeDataIntoEEPROM();
                }
                Printer::updateCurrentPosition(true);
                printCurrentPosition(PSTR("G29 "));
                Printer::finishProbing();
                Printer::feedrate = oldFeedrate;
                if(!ok) {
                    GCode::fatalError(PSTR("G29 leveling failed!"));
                    break;
                }
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,false);
                }
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,true);
                }
#else
                if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),Extruder::current->id,false,true);
#endif
#endif
            }
            break;
        case 30:
            { // G30 single probe set Z0
                uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
                if(Printer::runZProbe(p & 1,p & 2) == ILLEGAL_Z_PROBE) {
                    GCode::fatalError(PSTR("G29 leveling failed!"));
                    break;
                }
                Printer::updateCurrentPosition(p & 1);
            }
            break;
        case 31:  // G31 display hall sensor output
            Endstops::update();
            Endstops::update();
            Com::printF(Com::tZProbeState);
            Com::printF(Endstops::zProbe() ? Com::tHSpace : Com::tLSpace);
            Com::println();
            break;
#if FEATURE_AUTOLEVEL
        case 32: // G32 Auto-Bed leveling
        {
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
            float actTemp[NUM_EXTRUDER];
            for(int i = 0; i < NUM_EXTRUDER; i++)
                actTemp[i] = extruder[i].tempControl.targetTemperatureC;
            Printer::moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE,RMath::max(EEPROM::zProbeHeight(),static_cast<float>(ZHOME_HEAT_HEIGHT)),IGNORE_COORDINATE,Printer::homingFeedrate[Z_AXIS]);
            Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,false);
            }
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,true);
            }
#else
            if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),Extruder::current->id,false,true);
#endif
#endif
            if(!runBedLeveling(com)) {
                GCode::fatalError(PSTR("G32 leveling failed!"));
                break;
            }
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,false);
            }
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,true);
            }
#else
            if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),Extruder::current->id,false,true);
#endif
#endif
            }
            break;
#endif

#endif
        case 90: // G90
            Printer::relativeCoordinateMode = false;
            if(com->internalCommand)
                Com::printInfoFLN(PSTR("Absolute positioning"));
            break;
        case 91: // G91
            Printer::relativeCoordinateMode = true;
            if(com->internalCommand)
                Com::printInfoFLN(PSTR("Relative positioning"));
            break;
        case 92: { // G92
                float xOff = Printer::coordinateOffset[X_AXIS];
                float yOff = Printer::coordinateOffset[Y_AXIS];
                float zOff = Printer::coordinateOffset[Z_AXIS];
                if(com->hasX()) xOff = com->X - Printer::currentPosition[X_AXIS];
                if(com->hasY()) yOff = com->Y - Printer::currentPosition[Y_AXIS];
                if(com->hasZ()) zOff = com->Z - Printer::currentPosition[Z_AXIS];
                Printer::setOrigin(xOff, yOff, zOff);
                if(com->hasE()) {
                    Printer::currentPositionSteps[E_AXIS] = com->E * Printer::axisStepsPerMM[E_AXIS];
                }
            }
            break;
#if FEATURE_Z_PROBE && NUM_EXTRUDER > 1
        case 134:
            { // - G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!) Px = reference extruder, Sx = only measure extrude x against reference, Zx = add to measured z distance for Sx for correction.
                float z = com->hasZ() ? com->Z : 0;
                int p = com->hasP() ? com->P : 0;
                int s = com->hasS() ? com->S : -1;
                int startExtruder = Extruder::current->id;
                extruder[p].zOffset = 0;
                float mins[NUM_EXTRUDER],maxs[NUM_EXTRUDER],avg[NUM_EXTRUDER];
                for(int i = 0; i < NUM_EXTRUDER; i++) { // silence unnecessary compiler warning
                      avg[i] = 0;
                }
                bool bigError = false;

#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
                float actTemp[NUM_EXTRUDER];
                for(int i = 0; i < NUM_EXTRUDER; i++)
                    actTemp[i] = extruder[i].tempControl.targetTemperatureC;
                Printer::moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE,ZHOME_HEAT_HEIGHT,IGNORE_COORDINATE,Printer::homingFeedrate[Z_AXIS]);
                Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,false);
                }
                for(int i = 0; i < NUM_EXTRUDER; i++) {
                    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                        Extruder::setTemperatureForExtruder(RMath::max(actTemp[i],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),i,false,true);
                }
#else
                if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id],static_cast<float>(ZPROBE_MIN_TEMPERATURE)),Extruder::current->id,false,true);
#endif
#endif

#ifndef G134_REPETITIONS
#define G134_REPETITIONS 3
#endif
#ifndef G134_PRECISION
#define G134_PRECISION 0.05
#endif
                Printer::startProbing(true);
                for(int r = 0; r < G134_REPETITIONS && !bigError; r++) {
                    Extruder::selectExtruderById(p);
                    float refHeight = Printer::runZProbe(false,false);
                    if(refHeight == ILLEGAL_Z_PROBE) {
                        bigError = true;
                        break;
                    }
                    for(int i = 0; i < NUM_EXTRUDER && !bigError; i++) {
                        if(i == p) continue;
                        if(s >= 0 && i != s) continue;
                        extruder[i].zOffset = 0;
                        Extruder::selectExtruderById(i);
                        float height = Printer::runZProbe(false,false);
                        if(height == ILLEGAL_Z_PROBE) {
                            bigError = true;
                            break;
                        }
                        float off = (height - refHeight + z);
                        if(r == 0) {
                            avg[i] = mins[i] = maxs[i] = off;
                        } else {
                            avg[i] += off;
                            if(off < mins[i]) mins[i] = off;
                            if(off > maxs[i]) maxs[i] = off;
                            if(maxs[i] - mins[i] > G134_PRECISION) {
                                Com::printErrorFLN(PSTR("Deviation between measurements were too big, please repeat."));
                                bigError = true;
                                break;
                            }
                        }
                    }
                }
                if(!bigError) {
                    for(int i = 0; i < NUM_EXTRUDER; i++) {
                        if(s >= 0 && i != s) continue;
                        extruder[i].zOffset = avg[i] * Printer::axisStepsPerMM[Z_AXIS] / G134_REPETITIONS;
                    }
#if EEPROM_MODE != 0
                    EEPROM::storeDataIntoEEPROM(0);
#endif
                }
                Extruder::selectExtruderById(startExtruder);
                Printer::finishProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
#if ZHOME_HEAT_ALL
                for(int i = 0; i < NUM_EXTRUDER; i++)
                    Extruder::setTemperatureForExtruder(actTemp[i],i,false,false);
                for(int i = 0; i < NUM_EXTRUDER; i++)
                    Extruder::setTemperatureForExtruder(actTemp[i],i,false, actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
                Extruder::setTemperatureForExtruder(actTemp[Extruder::current->id], Extruder::current->id, false, actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
            }
            break;
#endif
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
        case 201:
            commandG201(*com);
            break;
        case 202:
            commandG202(*com);
            break;
        case 203:
            commandG203(*com);
            break;
        case 204:
            commandG204(*com);
            break;
#endif // defined
        default:
            if(!EVENT_UNHANDLED_G_CODE(com) && Printer::debugErrors()) {
                Com::printF(Com::tUnknownCommand);
                com->printCommand();
            }
    }
    previousMillisCmd = HAL::timeInMilliseconds();
}
/**
\brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode *com) {
    switch( com->M ) {
#if SDSUPPORT
        case 20: // M20 - list SD card
#if JSON_OUTPUT
            if (com->hasString() && com->text[1] == '2') { // " S2 P/folder"
                if (com->text[3] == 'P') {
                    sd.lsJSON(com->text + 4);
                }
            } else sd.ls();
#else
            sd.ls();
#endif
            break;
        case 21: // M21 - init SD card
            sd.mount();
            break;
        case 22: //M22 - release SD card
            sd.unmount();
            break;
        case 23: //M23 - Select file
            if(com->hasString()) {
                sd.fat.chdir();
                sd.selectFile(com->text);
            }
            break;
        case 24: //M24 - Start SD print
            sd.startPrint();
            break;
        case 25: //M25 - Pause SD print
            sd.pausePrint();
            break;
        case 26: //M26 - Set SD index
            if(com->hasS())
                sd.setIndex(com->S);
            break;
        case 27: //M27 - Get SD status
            sd.printStatus();
            break;
        case 28: //M28 - Start SD write
            if(com->hasString())
                sd.startWrite(com->text);
            break;
        case 29: //M29 - Stop SD write
            //processed in write to file routine above
            //savetosd = false;
            break;
        case 30: // M30 filename - Delete file
            if(com->hasString()) {
                sd.fat.chdir();
                sd.deleteFile(com->text);
            }
            break;
        case 32: // M32 directoryname
            if(com->hasString()) {
                sd.fat.chdir();
                sd.makeDirectory(com->text);
            }
            break;
#endif
#if JSON_OUTPUT && SDSUPPORT
        case 36: // M36 JSON File Info
            if (com->hasString()) {
                sd.JSONFileInfo(com->text);
            }
            break;
#endif
        case 42: //M42 -Change pin status via gcode
            if (com->hasP()) {
                int pin_number = com->P;
                for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
                    if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
                        pin_number = -1;
                        break;
                    }
                }
                if (pin_number > -1) {
                    if(com->hasS()) {
                        if(com->S >= 0 && com->S <= 255) {
                            pinMode(pin_number, OUTPUT);
                            digitalWrite(pin_number, com->S);
                            analogWrite(pin_number, com->S);
                            Com::printF(Com::tSetOutputSpace, pin_number);
                            Com::printFLN(Com::tSpaceToSpace,(int)com->S);
                        } else
                            Com::printErrorFLN(PSTR("Illegal S value for M42"));
                    } else {
                        pinMode(pin_number, INPUT_PULLUP);
                        Com::printF(Com::tSpaceToSpace, pin_number);
                        Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
                    }
                } else {
                    Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
                }
            }
            break;
        case 80: // M80 - ATX Power On
#if PS_ON_PIN>-1
            Commands::waitUntilEndOfAllMoves();
            previousMillisCmd = HAL::timeInMilliseconds();
            SET_OUTPUT(PS_ON_PIN); //GND
            Printer::setPowerOn(true);
            WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif
            break;
        case 81: // M81 - ATX Power Off
#if PS_ON_PIN>-1
            Commands::waitUntilEndOfAllMoves();
            SET_OUTPUT(PS_ON_PIN); //GND
            Printer::setPowerOn(false);
            WRITE(PS_ON_PIN,(POWER_INVERTING ? LOW : HIGH));
#endif
            break;
        case 82: // M82
            Printer::relativeExtruderCoordinateMode = false;
            break;
        case 83: // M83
            Printer::relativeExtruderCoordinateMode = true;
            break;
        case 84: // M84
            if(com->hasS()) {
                stepperInactiveTime = com->S * 1000;
            } else {
                Commands::waitUntilEndOfAllMoves();
                Printer::kill(true);
            }
            break;
        case 85: // M85
            if(com->hasS())
                maxInactiveTime = (int32_t)com->S * 1000;
            else
                maxInactiveTime = 0;
            break;
        case 92: // M92
            if(com->hasX()) Printer::axisStepsPerMM[X_AXIS] = com->X;
            if(com->hasY()) Printer::axisStepsPerMM[Y_AXIS] = com->Y;
            if(com->hasZ()) Printer::axisStepsPerMM[Z_AXIS] = com->Z;
            Printer::updateDerivedParameter();
            if(com->hasE()) {
                Extruder::current->stepsPerMM = com->E;
                Extruder::selectExtruderById(Extruder::current->id);
            }
            break;
        case 99: { // M99 S<time>
                millis_t wait = 10000;
                if(com->hasS())
                    wait = 1000*com->S;
                if(com->hasX())
                    Printer::disableXStepper();
                if(com->hasY())
                    Printer::disableYStepper();
                if(com->hasZ())
                    Printer::disableZStepper();
                wait += HAL::timeInMilliseconds();
#ifdef DEBUG_PRINT
                debugWaitLoop = 2;
#endif
                while(wait-HAL::timeInMilliseconds() < 100000) {
                    Printer::defaultLoopActions();
                }
                if(com->hasX())
                    Printer::enableXStepper();
                if(com->hasY())
                    Printer::enableYStepper();
                if(com->hasZ())
                    Printer::enableZStepper();
            }
            break;

        case 104: // M104 temperature
#if NUM_EXTRUDER > 0
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
#ifdef EXACT_TEMPERATURE_TIMING
            Commands::waitUntilEndOfAllMoves();
#else
            if(com->hasP() || (com->hasS() && com->S == 0))
                Commands::waitUntilEndOfAllMoves();
#endif
            if (com->hasS()) {
                if(com->hasT() && com->T < NUM_EXTRUDER)
                    Extruder::setTemperatureForExtruder(com->S, com->T, com->hasF() && com->F > 0);
                else
                    Extruder::setTemperatureForExtruder(com->S, Extruder::current->id, com->hasF() && com->F > 0);
            }
#endif
            break;
        case 140: // M140 set bed temp
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
            if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F > 0);
            break;
        case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
            printTemperatures(com->hasX());
            break;
        case 109: // M109 - Wait for extruder heater to reach target.
#if NUM_EXTRUDER > 0
            {
                if(reportTempsensorError()) break;
                previousMillisCmd = HAL::timeInMilliseconds();
                if(Printer::debugDryrun()) break;
                Commands::waitUntilEndOfAllMoves();
                Extruder *actExtruder = Extruder::current;
                if(com->hasT() && com->T < NUM_EXTRUDER) actExtruder = &extruder[com->T];
                if (com->hasS()) Extruder::setTemperatureForExtruder(com->S, actExtruder->id, com->hasF() && com->F > 0, true);
            }
#endif
            previousMillisCmd = HAL::timeInMilliseconds();
            break;
        case 190: { // M190 - Wait bed for heater to reach target.
                if(Printer::debugDryrun()) break;
                UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HEATING_BED_ID));
                Commands::waitUntilEndOfAllMoves();
                if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F > 0);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN > 0
                if(abs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) < SKIP_M190_IF_WITHIN) break;
#endif
                EVENT_WAITING_HEATER(-1);
                uint32_t codenum; //throw away variable
                codenum = HAL::timeInMilliseconds();
                while(heatedBedController.currentTemperatureC + 0.5 < heatedBedController.targetTemperatureC && heatedBedController.targetTemperatureC > 25.0) {
                    if( (HAL::timeInMilliseconds() - codenum) > 1000 ) { //Print Temp Reading every 1 second while heating up.
                        printTemperatures();
                        codenum = previousMillisCmd = HAL::timeInMilliseconds();
                    }
                    Commands::checkForPeriodicalActions(true);
                    GCode::keepAlive(WaitHeater);
                }
                EVENT_HEATING_FINISHED(-1);
                UI_CLEAR_STATUS;
                previousMillisCmd = HAL::timeInMilliseconds();
            }
            break;
#if NUM_TEMPERATURE_LOOPS > 0
        case 116: // Wait for temperatures to reach target temperature
            for(fast8_t h = 0; h <= HEATED_BED_INDEX; h++) {
                EVENT_WAITING_HEATER(h < NUM_EXTRUDER ? h : -1);
                tempController[h]->waitForTargetTemperature();
                EVENT_HEATING_FINISHED(h < NUM_EXTRUDER ? h : -1);
            }
            break;
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
        case 106: // M106 Fan On
            if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
                if(com->hasP() && com->P == 1)
                    setFan2Speed(com->hasS() ? com->S : 255);
                else
                    setFanSpeed(com->hasS() ? com->S : 255);
            }
            break;
        case 107: // M107 Fan Off
            if(com->hasP() && com->P == 1)
                setFan2Speed(0);
            else
                setFanSpeed(0);
            break;
#endif
        case 111: // M111 enable/disable run time debug flags
            if(com->hasS()) Printer::setDebugLevel(static_cast<uint8_t>(com->S));
            if(com->hasP()) {
                if (com->P > 0) Printer::debugSet(static_cast<uint8_t>(com->P));
                else Printer::debugReset(static_cast<uint8_t>(-com->P));
            }
            if(Printer::debugDryrun()) { // simulate movements without printing
#if NUM_EXTRUDER > 1
                for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
                    Extruder::setTemperatureForExtruder(0, i);
#else
                Extruder::setTemperatureForExtruder(0, 0);
#endif
                Extruder::setHeatedBedTemperature(0,false);
            }
            break;
        case 115: // M115
            Com::printFLN(Com::tFirmware);
            reportPrinterUsage();
            break;
        case 114: // M114
            printCurrentPosition(PSTR("M114 "));
            break;
        case 117: // M117 message to lcd
            if(com->hasString()) {
                UI_STATUS_UPD_RAM(com->text);
            }
            break;
        case 119: // M119
            Commands::waitUntilEndOfAllMoves();
            Endstops::update();
            Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
            Endstops::report();
            break;
#if BEEPER_TYPE>0
        case 120: // M120 Test beeper function
            if(com->hasS() && com->hasP())
                beep(com->S, com->P); // Beep test
            break;
#endif
        case 200: { // M200 T<extruder> D<diameter>
                uint8_t extruderId = Extruder::current->id;
                if(com->hasT() && com->T < NUM_EXTRUDER)
                    extruderId = com->T;
                float d = 0;
                if(com->hasR())
                    d = com->R;
                if(com->hasD())
                    d = com->D;
                extruder[extruderId].diameter = d;
                if(extruderId == Extruder::current->id)
                    changeFlowrateMultiply(Printer::extrudeMultiply);
                if(d == 0) {
                    Com::printFLN(PSTR("Disabled volumetric extrusion for extruder "),static_cast<int>(extruderId));
                } else {
                    Com::printF(PSTR("Set volumetric extrusion for extruder "),static_cast<int>(extruderId));
                    Com::printFLN(PSTR(" to "),d);
                }
            }
            break;
#if RAMP_ACCELERATION
        case 201: // M201
            if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
            if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
            if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
            if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
            Printer::updateDerivedParameter();
            break;
        case 202: // M202
            if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
            if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
            if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
            if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
            Printer::updateDerivedParameter();
            break;
#endif
        case 203: // M203 Temperature monitor
            if(com->hasS())
                manageMonitor = com->S != 255;
            else
                manageMonitor = 0;
            break;
        case 204: { // M204
                TemperatureController *temp = &Extruder::current->tempControl;
                if(com->hasS()) {
                    if(com->S < 0) break;
                    if(com->S < NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
                    else temp = &heatedBedController;
                }
                if(com->hasX()) temp->pidPGain = com->X;
                if(com->hasY()) temp->pidIGain = com->Y;
                if(com->hasZ()) temp->pidDGain = com->Z;
                temp->updateTempControlVars();
            }
            break;
        case 205: // M205 Show EEPROM settings
            EEPROM::writeSettings();
            break;
        case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
            EEPROM::update(com);
            break;
        case 207: // M207 X<XY jerk> Z<Z Jerk>
            if(com->hasX())
                Printer::maxJerk = com->X;
            if(com->hasE()) {
                Extruder::current->maxStartFeedrate = com->E;
                Extruder::selectExtruderById(Extruder::current->id);
            }
            if(com->hasZ())
                Printer::maxZJerk = com->Z;
            Com::printF(Com::tJerkColon,Printer::maxJerk);
            Com::printFLN(Com::tZJerkColon,Printer::maxZJerk);
            break;
        case 209: // M209 S<0/1> Enable/disable autoretraction
            if(com->hasS())
                Printer::setAutoretract(com->S != 0);
            break;
        case 220: // M220 S<Feedrate multiplier in percent>
            changeFeedrateMultiply(com->getS(100));
            break;
        case 221: // M221 S<Extrusion flow multiplier in percent>
            changeFlowrateMultiply(com->getS(100));
            break;
        case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
            if(!com->hasS() || !com->hasP())
                break;
            {
                bool comp = com->S;
                if(com->hasX()) {
                    if(com->X == 0)
                        HAL::pinMode(com->S,INPUT);
                    else
                        HAL::pinMode(com->S,INPUT_PULLUP);
                }
                do {
                    Commands::checkForPeriodicalActions(true);
                    GCode::keepAlive(WaitHeater);
                } while(HAL::digitalRead(com->P) != comp);
            }
            break;
#if USE_ADVANCE
        case 223: // M223 Extruder interrupt test
            if(com->hasS()) {
                InterruptProtectedBlock noInts;
                Printer::extruderStepsNeeded += com->S;
            }
            break;
        case 232: // M232
            Com::printF(Com::tLinearStepsColon,maxadv2);
#if ENABLE_QUADRATIC_ADVANCE
            Com::printF(Com::tQuadraticStepsColon,maxadv);
#endif
            Com::printFLN(Com::tCommaSpeedEqual,maxadvspeed);
#if ENABLE_QUADRATIC_ADVANCE
            maxadv = 0;
#endif
            maxadv2 = 0;
            maxadvspeed = 0;
            break;
#endif
#if USE_ADVANCE
        case 233: // M233
            if(com->hasY())
                Extruder::current->advanceL = com->Y;
            Com::printF(Com::tLinearLColon,Extruder::current->advanceL);
#if ENABLE_QUADRATIC_ADVANCE
            if(com->hasX())
                Extruder::current->advanceK = com->X;
            Com::printF(Com::tQuadraticKColon,Extruder::current->advanceK);
#endif
            Com::println();
            Printer::updateAdvanceFlags();
            break;
#endif
#if Z_HOME_DIR>0 && MAX_HARDWARE_ENDSTOP_Z
        case 251: // M251
            Printer::zLength -= Printer::currentPosition[Z_AXIS];
            Printer::currentPositionSteps[Z_AXIS] = 0;
            Printer::updateDerivedParameter();
            Printer::updateCurrentPosition();
            Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#if EEPROM_MODE != 0
            EEPROM::storeDataIntoEEPROM(false);
            Com::printFLN(Com::tEEPROMUpdated);
#endif
            Commands::printCurrentPosition(PSTR("M251 "));
            break;
#endif
#if FEATURE_DITTO_PRINTING
        case 280: // M280
#if DUAL_X_AXIS
            Extruder::dittoMode = 0;
            Extruder::selectExtruderById(0);
            Printer::homeXAxis();
            if(com->hasS() && com->S > 0) {
                Extruder::current = &extruder[1];
                PrintLine::moveRelativeDistanceInSteps(-Extruder::current->xOffset + static_cast<int32_t>(Printer::xLength*0.5*Printer::axisStepsPerMM[X_AXIS]), 0, 0, 0, EXTRUDER_SWITCH_XY_SPEED, true, true);
                Printer::currentPositionSteps[X_AXIS] = Printer::xMinSteps;
                Extruder::current = &extruder[0];
                Extruder::dittoMode = 1;
            }
#else
            if(com->hasS()) { // Set ditto mode S: 0 = off, 1 = 1 extra extruder, 2 = 2 extra extruder, 3 = 3 extra extruders
                Extruder::dittoMode = com->S;
            }
#endif
            break;
#endif
        case 281: // Trigger watchdog
#if FEATURE_WATCHDOG
            {
                if(com->hasX()) {
                    HAL::stopWatchdog();
                    Com::printFLN(PSTR("Watchdog disabled"));
                    break;
                }
                Com::printInfoFLN(PSTR("Triggering watchdog. If activated, the printer will reset."));
                Printer::kill(false);
                HAL::delayMilliseconds(200); // write output, make sure heaters are off for safety
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
                InterruptProtectedBlock noInts;         // don't disable interrupts on mega2560 and mega1280 because of bootloader bug
#endif
                while(1) {} // Endless loop
            }
#else
            Com::printInfoFLN(PSTR("Watchdog feature was not compiled into this version!"));
#endif
            break;
#if defined(BEEPER_PIN) && BEEPER_PIN>=0
        case 300: { // M300
                int beepS = 1;
                int beepP = 1000;
                if(com->hasS()) beepS = com->S;
                if(com->hasP()) beepP = com->P;
                HAL::tone(BEEPER_PIN, beepS);
                HAL::delayMilliseconds(beepP);
                HAL::noTone(BEEPER_PIN);
            }
            break;
#endif
        case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will disallow.
            Printer::setColdExtrusionAllowed(!com->hasS() || (com->hasS() && com->S != 0));
            break;
        case 303: { // M303
#if defined(TEMP_PID) && NUM_TEMPERATURE_LOOPS > 0
                int temp = 150;
                int cont = 0;
                int cycles = 5;
                if(com->hasS()) temp = com->S;
                if(com->hasP()) cont = com->P;
                if(com->hasR()) cycles = static_cast<int>(com->R);
                if(cont >= HEATED_BED_INDEX) cont = HEATED_BED_INDEX;
                if(cont < 0) cont = 0;
                tempController[cont]->autotunePID(temp,cont,cycles,com->hasX());
#endif
            }
            break;

#if FEATURE_AUTOLEVEL
        case 320: // M320 Activate autolevel
            Printer::setAutolevelActive(true);
            if(com->hasS() && com->S) {
                EEPROM::storeDataIntoEEPROM();
            }
            break;
        case 321: // M321 Deactivate autoleveling
            Printer::setAutolevelActive(false);
            if(com->hasS() && com->S) {
                if(com->S == 3)
                    Printer::resetTransformationMatrix(false);
                EEPROM::storeDataIntoEEPROM();
            }
            break;
        case 322: // M322 Reset autoeveling matrix
            Printer::resetTransformationMatrix(false);
            if(com->hasS() && com->S) {
                EEPROM::storeDataIntoEEPROM();
            }
            break;
#endif // FEATURE_AUTOLEVEL
#if FEATURE_SERVO
        case 340: // M340
            if(com->hasP() && com->P<4 && com->P>=0) {
                int s = 0;
                if(com->hasS())
                    s = com->S;
                uint16_t r = 0;
                if(com->hasR())    // auto off time in ms
                    r = com->R;
                HAL::servoMicroseconds(com->P,s,r);
            }
            break;
#endif // FEATURE_SERVO
        case 350: { // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
                OUT_P_LN("Set Microstepping");
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
                if(com->hasS()) for(int i = 0; i <= 4; i++) microstepMode(i, com->S);
                if(com->hasX()) microstepMode(0, (uint8_t)com->X);
                if(com->hasY()) microstepMode(1, (uint8_t)com->Y);
                if(com->hasZ()) microstepMode(2, (uint8_t)com->Z);
                if(com->hasE()) microstepMode(3, (uint8_t)com->E);
                if(com->hasP()) microstepMode(4, com->P); // Original B but is not supported here
                microstepReadings();
#endif
            }
            break;
        case 355: // M355 S<0/1> - Turn case light on/off, no S = report status
            if(com->hasS()) {
                Printer::setCaseLight(com->S);
            } else
                Printer::reportCaseLightStatus();
            break;
        case 360: // M360 - show configuration
            Printer::showConfiguration();
            break;
        case 400: // M400 Finish all moves
            Commands::waitUntilEndOfAllMoves();
            break;
        case 401: // M401 Memory position
            Printer::MemoryPosition();
            break;
        case 402: // M402 Go to stored position
            Printer::GoToMemoryPosition(com->hasX(),com->hasY(),com->hasZ(),com->hasE(),(com->hasF() ? com->F : Printer::feedrate));
            break;
#if JSON_OUTPUT
        case 408: // M408
            Printer::showJSONStatus(com->hasS() ? static_cast<int>(com->S) : 0);
            break;
#endif
#if FAN_THERMO_PIN > -1
        case 460: // M460 X<minTemp> Y<maxTemp> : Set temperature range for thermo controlled fan
            if(com->hasX())
                Printer::thermoMinTemp = com->X;
            if(com->hasY())
                Printer::thermoMaxTemp = com->Y;
            break;
#endif
        case 500: { // M500
#if EEPROM_MODE != 0
                EEPROM::storeDataIntoEEPROM(false);
                Com::printInfoFLN(Com::tConfigStoredEEPROM);
#else
                Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
            }
            break;
        case 501: { // M501
#if EEPROM_MODE != 0
                EEPROM::readDataFromEEPROM(true);
                Extruder::selectExtruderById(Extruder::current->id);
                Com::printInfoFLN(Com::tConfigLoadedEEPROM);
#else
                Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
            }
            break;
        case 502: // M502
            EEPROM::restoreEEPROMSettingsFromConfiguration();
            break;
#if EXTRUDER_JAM_CONTROL
#ifdef DEBUG_JAM
        case 512:
            Com::printFLN(PSTR("Jam signal:"),(int16_t)READ(EXT0_JAM_PIN));
            break;
#endif // DEBUG_JAM
        case 513:
            Extruder::markAllUnjammed();
            break;
#endif // EXTRUDER_JAM_CONTROL
#ifdef DEBUG_QUEUE_MOVE
        case 533: { // M533 Write move data
                InterruptProtectedBlock noInts;
                int lc = (int)PrintLine::linesCount;
                int lp = (int)PrintLine::linesPos;
                int wp = (int)PrintLine::linesWritePos;
                int n = (wp-lp);
                if(n < 0) n += PRINTLINE_CACHE_SIZE;
                noInts.unprotect();
                if(n != lc)
                    Com::printFLN(PSTR("Buffer corrupted"));
                Com::printF(PSTR("Buf:"),lc);
                Com::printF(PSTR(",LP:"),lp);
                Com::printFLN(PSTR(",WP:"),wp);
                if(PrintLine::cur == NULL) {
                    Com::printFLN(PSTR("No move"));
                    if(PrintLine::linesCount > 0) {
                        PrintLine &cur = PrintLine::lines[PrintLine::linesPos];
                        Com::printF(PSTR("JFlags:"), (int)cur.joinFlags);
                        Com::printFLN(PSTR(" Flags:"), (int)cur.flags);
                        if(cur.isWarmUp()) {
                            Com::printFLN(PSTR(" warmup:"), (int)cur.getWaitForXLinesFilled());
                        }
                    }
                } else {
                    Com::printF(PSTR("Rem:"), PrintLine::cur->stepsRemaining);
                    Com::printFLN(PSTR(" Int:"), Printer::interval);
                }
            }
            break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
        case 534: // M534
            Com::printFLN(PSTR("Max. segment size:"), Printer::maxRealSegmentLength);
            if(com->hasS())
                Printer::maxRealSegmentLength = 0;
            break;
#endif
#ifdef DEBUG_REAL_JERK
            Com::printFLN(PSTR("Max. jerk measured:"), Printer::maxRealJerk);
            if(com->hasS())
                Printer::maxRealJerk = 0;
            break;
#endif

#if FEATURE_RETRACTION
        case 600: // M600
            uid.executeAction(UI_ACTION_WIZARD_FILAMENTCHANGE, true);
            break;
#endif
        case 601: // M601
            if(com->hasS() && com->S > 0)
                Extruder::pauseExtruders();
            else
                Extruder::unpauseExtruders();
            break;
        case 602: // M602
            Commands::waitUntilEndOfAllMoves();
            if(com->hasS()) Printer::setDebugJam(com->S > 0);
            if(com->hasP()) Printer::setJamcontrolDisabled(com->P > 0);
            break;
        case 603: // M03
            Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_DETECTED, true);
            break;
#if 0 && UI_DISPLAY_TYPE != NO_DISPLAY
        // some debugging commands normally disabled
        case 888: // M888
            Com::printFLN(PSTR("Selected language:"),(int)Com::selectedLanguage);
            Com::printF(PSTR("Translation:"));
            Com::printFLN(Com::translatedF(0));
            break;
        case 889: // M889
            uid.showLanguageSelectionWizard();
            break;
        case 891: // M891
            if(com->hasS())
                EEPROM::setVersion(com->S);
            break;
#endif
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
        case 890: // M890
            if(com->hasX() && com->hasY()) {
                float c = Printer::bendingCorrectionAt(com->X,com->Y);
                Com::printF(PSTR("Bending at ("),com->X);
                Com::printF(PSTR(","),com->Y);
                Com::printFLN(PSTR(") = "),c);
            }
            break;
#endif
        case 999: // M999 Stop fatal error take down
            if(com->hasS())
                GCode::fatalError(PSTR("Testing fatal error"));
            else
                GCode::resetFatalError();
            break;
        default:
            if(!EVENT_UNHANDLED_M_CODE(com) && Printer::debugErrors()) {
                Com::printF(Com::tUnknownCommand);
                com->printCommand();
            }
    }
}

/**
\brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com) {
    if (INCLUDE_DEBUG_COMMUNICATION) {
        if(Printer::debugCommunication()) {
            if(com->hasG() || (com->hasM() && com->M != 111)) {
                previousMillisCmd = HAL::timeInMilliseconds();
                return;
            }
        }
    }
    if(com->hasG()) processGCode(com);
    else if(com->hasM()) processMCode(com);
    else if(com->hasT()) {    // Process T code
        //com->printCommand(); // for testing if this the source of extruder switches
        Commands::waitUntilEndOfAllMoves();
        Extruder::selectExtruderById(com->T);
        SelectExtruder500XL(com->T);
    } else {
        if(Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#ifdef DEBUG_DRYRUN_ERROR
    if(Printer::debugDryrun()) {
        Com::printFLN("Dryrun was enabled");
        com->printCommand();
        Printer::debugReset(8);
    }
#endif

}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
    HAL::resetHardware();
#else
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Printer::kill(false);
    Extruder::manageTemperatures();
    for(uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
        pwm_pos[i] = 0;
#if EXT0_HEATER_PIN > -1 && NUM_EXTRUDER > 0
    WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
    WRITE(EXT2_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
    WRITE(EXT3_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
    WRITE(EXT4_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
    WRITE(EXT5_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    WRITE(FAN_PIN, 0);
#endif
#if HEATED_BED_HEATER_PIN > -1
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_KILLED_ID));
    HAL::delayMilliseconds(200);
    InterruptProtectedBlock noInts;
    while(1) {}
#endif
}

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if(newfree < lowestRAMValue)
        lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
    if(lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM, lowestRAMValue);
    }
}
