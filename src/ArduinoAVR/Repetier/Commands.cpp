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

const int sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop()
{
    while(true)
    {
#ifdef DEBUG_PRINT
        debugWaitLoop = 1;
#endif

        GCode::readFromSerial();
        GCode *code = GCode::peekCurrentCommand();
        //UI_SLOW; // do longer timed user interface action
        UI_MEDIUM; // do check encoder
        if(code)
        {
#if SDSUPPORT
            if(sd.savetosd)
            {
                if(!(code->hasM() && code->M == 29))   // still writing to file
                {
                    sd.writeCommand(code);
                }
                else
                {
                    sd.finishWrite();
                }
#ifdef ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            }
            else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }
        Printer::defaultLoopActions();
    }
}

void Commands::checkForPeriodicalActions()
{
    if(!executePeriodical) return;
    executePeriodical=0;
    Extruder::manageTemperatures();
    if(--counter250ms==0)
    {
        if(manageMonitor<=1+NUM_EXTRUDER)
            writeMonitor();
        counter250ms=5;
    }
    UI_SLOW;
}

/** \brief Waits until movement cache is empty.

  Some commands expect no movement, before they can execute. This function
  waits, until the steppers are stopped. In the meanwhile it buffers incoming
  commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves()
{
#ifdef DEBUG_PRINT
    debugWaitLoop = 8;
#endif
    while(PrintLine::hasLines())
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
        UI_MEDIUM;
    }
}
void Commands::waitUntilEndOfAllBuffers()
{
    GCode *code;
#ifdef DEBUG_PRINT
    debugWaitLoop = 9;
#endif
    while(PrintLine::hasLines() || (code = GCode::peekCurrentCommand()) != NULL)
    {
        GCode::readFromSerial();
        UI_MEDIUM; // do check encoder
        if(code)
        {
#if SDSUPPORT
            if(sd.savetosd)
            {
                if(!(code->hasM() && code->M == 29))   // still writing to file
                {
                    sd.writeCommand(code);
                }
                else
                {
                    sd.finishWrite();
                }
#ifdef ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            }
            else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }        Commands::checkForPeriodicalActions();
        UI_MEDIUM;
    }
}
void Commands::printCurrentPosition()
{
    float x,y,z;
    Printer::realPosition(x,y,z);
    x += Printer::coordinateOffset[X_AXIS];
    y += Printer::coordinateOffset[Y_AXIS];
    z += Printer::coordinateOffset[Z_AXIS];
    Com::printF(Com::tXColon,x*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceYColon,y*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceZColon,z*(Printer::unitIsInches?0.03937:1),2);
    Com::printFLN(Com::tSpaceEColon,Printer::currentPositionSteps[3]*Printer::invAxisStepsPerMM[3]*(Printer::unitIsInches?0.03937:1),2);
    //Com::printF(PSTR("OffX:"),Printer::offsetX); // to debug offset handling
    //Com::printFLN(PSTR(" OffY:"),Printer::offsetY);
}
void Commands::printTemperatures(bool showRaw)
{
    float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE==0
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
#else
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
#if HAVE_HEATED_BED
    Com::printF(Com::tSpaceBColon,Extruder::getHeatedBedTemperature());
    Com::printF(Com::tSpaceSlash,heatedBedController.targetTemperatureC,0);
    if(showRaw)
    {
        Com::printF(Com::tSpaceRaw,(int)NUM_EXTRUDER);
        Com::printF(Com::tColon,(1023<<(2-ANALOG_REDUCE_BITS))-heatedBedController.currentTemperature);
    }
    Com::printF(Com::tSpaceBAtColon,(pwm_pos[heatedBedController.pwmIndex])); // Show output of autotune when tuning!
#endif
#endif
#ifdef TEMP_PID
    Com::printF(Com::tSpaceAtColon,(autotuneIndex==255?pwm_pos[Extruder::current->id]:pwm_pos[autotuneIndex])); // Show output of autotune when tuning!
#endif
#if NUM_EXTRUDER>1
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
        Com::printF(Com::tSpaceT,(int)i);
        Com::printF(Com::tColon,extruder[i].tempControl.currentTemperatureC);
        Com::printF(Com::tSpaceSlash,extruder[i].tempControl.targetTemperatureC,0);
#ifdef TEMP_PID
        Com::printF(Com::tSpaceAt,(int)i);
        Com::printF(Com::tColon,(pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!
#endif
        if(showRaw)
        {
            Com::printF(Com::tSpaceRaw,(int)i);
            Com::printF(Com::tColon,(1023<<(2-ANALOG_REDUCE_BITS))-extruder[i].tempControl.currentTemperature);
        }
    }
#endif
    Com::println();
}
void Commands::changeFeedrateMultiply(int factor)
{
    if(factor<25) factor=25;
    if(factor>500) factor=500;
    Printer::feedrate *= (float)factor/(float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply,factor);
}
void Commands::changeFlowateMultiply(int factor)
{
    if(factor<25) factor=25;
    if(factor>200) factor=200;
    Printer::extrudeMultiply = factor;
    Com::printFLN(Com::tFlowMultiply,factor);
}
void Commands::setFanSpeed(int speed,bool wait)
{
#if FAN_PIN>=0
    speed = constrain(speed,0,255);
    Printer::setMenuMode(MENU_MODE_FAN_RUNNING,speed!=0);
    if(wait)
        Commands::waitUntilEndOfAllMoves(); // use only if neededthis to change the speed exactly at that point, but it may cause blobs if you do!
    if(speed!=pwm_pos[NUM_EXTRUDER+2])
        Com::printFLN(Com::tFanspeed,speed);
    pwm_pos[NUM_EXTRUDER+2] = speed;
#endif
}
void Commands::reportPrinterUsage()
{
#if EEPROM_MODE!=0
    float dist = Printer::filamentPrinted*0.001+HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
    Com::printF(Com::tPrintedFilament,dist,2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC>15) alloff = false;

    int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME);
    int32_t tmp = seconds/86400;
    seconds-=tmp*86400;
    Com::printF(Com::tPrintingTime,tmp);
    tmp=seconds/3600;
    Com::printF(Com::tSpaceDaysSpace,tmp);
    seconds-=tmp*3600;
    tmp = seconds/60;
    Com::printF(Com::tSpaceHoursSpace,tmp);
    Com::printFLN(Com::tSpaceMin);
#endif
}
#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DIGIPOT
// Digipot methods for controling current and microstepping

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
int digitalPotWrite(int address, unsigned int value) // From Arduino DigitalPotControl example
{
    WRITE(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    HAL::spiSend(address); //  send in the address and value via SPI:
    HAL::spiSend(value);
    WRITE(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void setMotorCurrent(uint8_t driver, unsigned int current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}
#endif

void motorCurrentControlInit() //Initialize Digipot Motor Current
{
#if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;

    HAL::spiInit(0); //SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
    for(int i=0; i<=4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrent(i,digipot_motor_current[i]);
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_LTC2600

void setMotorCurrent( uint8_t channel, unsigned short level )
{
    const uint8_t ltc_channels[] =  LTC2600_CHANNELS;
    if(channel>LTC2600_NUM_CHANNELS) return;
    uint8_t address = ltc_channels[channel];
    char i;


    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    // configure the pins
    WRITE( LTC2600_CS_PIN, HIGH );
    SET_OUTPUT( LTC2600_CS_PIN );
    WRITE( LTC2600_SCK_PIN, LOW );
    SET_OUTPUT( LTC2600_SCK_PIN );
    WRITE( LTC2600_SDI_PIN, LOW );
    SET_OUTPUT( LTC2600_SDI_PIN );

    // enable the command interface of the LTC2600
    WRITE( LTC2600_CS_PIN, LOW );

    // transfer command and address
    for( i=7; i>=0; i-- )
    {
        WRITE( LTC2600_SDI_PIN, address & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // transfer the data word
    for( i=15; i>=0; i-- )
    {
        WRITE( LTC2600_SDI_PIN, level & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // disable the ommand interface of the LTC2600 -
    // this carries out the specified command
    WRITE( LTC2600_CS_PIN, HIGH );

} // setLTC2600

void motorCurrentControlInit() //Initialize LTC2600 Motor Current
{
    const unsigned int ltc_current[] =  MOTOR_CURRENT;
    uint8_t i;
    for(i=0; i<LTC2600_NUM_CHANNELS; i++)
    {
        setMotorCurrent(i, ltc_current[i] );
    }
}
#endif

#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstepMS(uint8_t driver, int8_t ms1, int8_t ms2)
{
    if(ms1 > -1) switch(driver)
        {
        case 0:
            WRITE( X_MS1_PIN,ms1);
            break;
        case 1:
            WRITE( Y_MS1_PIN,ms1);
            break;
        case 2:
            WRITE( Z_MS1_PIN,ms1);
            break;
        case 3:
            WRITE(E0_MS1_PIN,ms1);
            break;
        case 4:
            WRITE(E1_MS1_PIN,ms1);
            break;
        }
    if(ms2 > -1) switch(driver)
        {
        case 0:
            WRITE( X_MS2_PIN,ms2);
            break;
        case 1:
            WRITE( Y_MS2_PIN,ms2);
            break;
        case 2:
            WRITE( Z_MS2_PIN,ms2);
            break;
        case 3:
            WRITE(E0_MS2_PIN,ms2);
            break;
        case 4:
            WRITE(E1_MS2_PIN,ms2);
            break;
        }
}

void microstepMode(uint8_t driver, uint8_t stepping_mode)
{
    switch(stepping_mode)
    {
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
    }
}
void microstepReadings()
{
    Com::printFLN(Com::tMS1MS2Pins);
    Com::printF(Com::tXColon,READ(X_MS1_PIN));
    Com::printFLN(Com::tComma,READ(X_MS2_PIN));
    Com::printF(Com::tYColon,READ(Y_MS1_PIN));
    Com::printFLN(Com::tComma,READ(Y_MS2_PIN));
    Com::printF(Com::tZColon,READ(Z_MS1_PIN));
    Com::printFLN(Com::tComma,READ(Z_MS2_PIN));
    Com::printF(Com::tE0Colon,READ(E0_MS1_PIN));
    Com::printFLN(Com::tComma,READ(E0_MS2_PIN));
    Com::printF(Com::tE1Colon,READ(E1_MS1_PIN));
    Com::printFLN(Com::tComma,READ(E1_MS2_PIN));
}
#endif

void microstepInit()
{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    SET_OUTPUT(X_MS2_PIN);
    SET_OUTPUT(Y_MS2_PIN);
    SET_OUTPUT(Z_MS2_PIN);
    SET_OUTPUT(E0_MS2_PIN);
    SET_OUTPUT(E1_MS2_PIN);
    for(int i=0; i<=4; i++) microstepMode(i,microstep_modes[i]);
#endif
}



/**
  \brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com)
{
    uint32_t codenum; //throw away variable
#ifdef INCLUDE_DEBUG_COMMUNICATION
    if(Printer::debugCommunication())
    {
        if(com->hasG() || (com->hasM() && com->M!=111))
        {
            previousMillisCmd = HAL::timeInMilliseconds();
            return;
        }
    }
#endif
    if(com->hasG())
    {
        switch(com->G)
        {
        case 0: // G0 -> G1
        case 1: // G1
            if(com->hasS())
                Printer::setNoDestinationCheck(com->S!=0);
            if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if NONLINEAR_SYSTEM
                PrintLine::queueDeltaMove(ALWAYS_CHECK_ENDSTOPS, true, true);
#else
                PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS,true);
#endif
            break;
#if ARC_SUPPORT
        case 2: // CW Arc
        case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        {
            float position[3];
            Printer::realPosition(position[X_AXIS],position[Y_AXIS],position[Z_AXIS]);
            if(!Printer::setDestinationStepsFromGCode(com)) break; // For X Y Z E F
            float offset[2] = {Printer::convertToMM(com->hasI()?com->I:0),Printer::convertToMM(com->hasJ()?com->J:0)};
            float target[4] = {Printer::realXPosition(),Printer::realYPosition(),Printer::realZPosition(),Printer::destinationSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]};
            float r;
            if (com->hasR())
            {
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
                r = Printer::convertToMM(com->R);
                // Calculate the change in position along each selected axis
                double x = target[X_AXIS]-position[X_AXIS];
                double y = target[Y_AXIS]-position[Y_AXIS];

                double h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
                // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
                // real CNC, and thus - for practical reasons - we will terminate promptly:
                if(isnan(h_x2_div_d))
                {
                    Com::printErrorFLN(Com::tInvalidArc);
                    break;
                }
                // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                if (com->G==3)
                {
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
                if (r < 0)
                {
                    h_x2_div_d = -h_x2_div_d;
                    r = -r; // Finished with r. Set to positive for mc_arc
                }
                // Complete the operation by calculating the actual center of the arc
                offset[0] = 0.5*(x-(y*h_x2_div_d));
                offset[1] = 0.5*(y+(x*h_x2_div_d));

            }
            else     // Offset mode specific computations
            {
                r = hypot(offset[0], offset[1]); // Compute arc radius for arc
            }
            // Set clockwise/counter-clockwise sign for arc computations
            uint8_t isclockwise = com->G == 2;
            // Trace the arc
            PrintLine::arc(position, target, offset,r, isclockwise);

            break;
        }
#endif
        case 4: // G4 dwell
            Commands::waitUntilEndOfAllMoves();
            codenum = 0;
            if(com->hasP()) codenum = com->P; // milliseconds to wait
            if(com->hasS()) codenum = com->S * 1000; // seconds to wait
            codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting
            while((uint32_t)(codenum-HAL::timeInMilliseconds())  < 2000000000 )
            {
                GCode::readFromSerial();
                Commands::checkForPeriodicalActions();
            }
            break;
        case 20: // Units to inches
            Printer::unitIsInches = 1;
            break;
        case 21: // Units to mm
            Printer::unitIsInches = 0;
            break;
        case 28:  //G28 Home all Axis one at a time
        {
            uint8_t home_all_axis = (com->hasNoXYZ() && !com->hasE());
            if(com->hasE())
            {
                Printer::currentPositionSteps[E_AXIS] = 0;
            }
            if(home_all_axis || !com->hasNoXYZ())
                Printer::homeAxis(home_all_axis || com->hasX(),home_all_axis || com->hasY(),home_all_axis || com->hasZ());
            Printer::updateCurrentPosition();
        }
        break;
#if FEATURE_Z_PROBE
        case 29: // 3 points, build average
        {
            GCode::executeFString(Com::tZProbeStartScript);
            bool oldAutolevel = Printer::isAutolevelActive();
            Printer::setAutolevelActive(false);
            float sum = 0,last,oldFeedrate = Printer::feedrate;
            Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            sum = Printer::runZProbe(true,false,Z_PROBE_REPETITIONS,false);
            if(sum<0) break;
            Printer::moveTo(EEPROM::zProbeX2(),EEPROM::zProbeY2(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            last = Printer::runZProbe(false,false);
            if(last<0) break;
            sum+= last;
            Printer::moveTo(EEPROM::zProbeX3(),EEPROM::zProbeY3(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            last = Printer::runZProbe(false,true);
            if(last<0) break;
            sum+= last;
            sum *= 0.33333333333333;
            Com::printFLN(Com::tZProbeAverage,sum);
            if(com->hasS() && com->S)
            {
#if MAX_HARDWARE_ENDSTOP_Z
#if DRIVE_SYSTEM == 3
                Printer::updateCurrentPosition();
                Printer::zLength += sum - Printer::currentPosition[Z_AXIS];
                Printer::updateDerivedParameter();
                Printer::homeAxis(true,true,true);
#else
                Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                Printer::zLength = Printer::runZMaxProbe() + sum-ENDSTOP_Z_BACK_ON_HOME;
#endif
                Com::printInfoFLN(Com::tZProbeZReset);
                Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#else
                Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                Com::printFLN(PSTR("Adjusted z origin"));
#endif
            }
            Printer::feedrate = oldFeedrate;
            Printer::setAutolevelActive(oldAutolevel);
            if(com->hasS() && com->S == 2)
                EEPROM::storeDataIntoEEPROM();
            Printer::updateCurrentPosition(true);
            printCurrentPosition();
        }
        break;
        case 30: // G30 single probe set Z0
        {
            uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
            bool oldAutolevel = Printer::isAutolevelActive();
            Printer::setAutolevelActive(false);
            Printer::runZProbe(p & 1,p & 2);
            Printer::setAutolevelActive(oldAutolevel);
            Printer::updateCurrentPosition(p & 1);
            printCurrentPosition();
        }
        break;
        case 31:  // G31 display hall sensor output
            Com::printF(Com::tZProbeState);
            Com::print(Printer::isZProbeHit() ? 'H' : 'L');
            Com::println();
            break;
#if FEATURE_AUTOLEVEL
        case 32: // G32 Auto-Bed leveling
        {
            GCode::executeFString(Com::tZProbeStartScript);
            //bool iterate = com->hasP() && com->P>0;
            Printer::coordinateOffset[0] = Printer::coordinateOffset[1] = Printer::coordinateOffset[2] = 0;
            Printer::setAutolevelActive(false); // iterate
            float h1,h2,h3,hc,oldFeedrate = Printer::feedrate;
            Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h1 = Printer::runZProbe(true,false,Z_PROBE_REPETITIONS,false);
            if(h1<0) break;
            Printer::moveTo(EEPROM::zProbeX2(),EEPROM::zProbeY2(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h2 = Printer::runZProbe(false,false);
            if(h2<0) break;
            Printer::moveTo(EEPROM::zProbeX3(),EEPROM::zProbeY3(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h3 = Printer::runZProbe(false,true);
            if(h3<0) break;
            Printer::buildTransformationMatrix(h1,h2,h3);
            //-(Rxx*Ryz*y-Rxz*Ryx*y+(Rxz*Ryy-Rxy*Ryz)*x)/(Rxy*Ryx-Rxx*Ryy)
            // z = z-deviation from origin due to bed transformation
            float z = -((Printer::autolevelTransformation[0]*Printer::autolevelTransformation[5]-
                         Printer::autolevelTransformation[2]*Printer::autolevelTransformation[3])*
                        (float)Printer::currentPositionSteps[Y_AXIS]*Printer::invAxisStepsPerMM[Y_AXIS]+
                        (Printer::autolevelTransformation[2]*Printer::autolevelTransformation[4]-
                         Printer::autolevelTransformation[1]*Printer::autolevelTransformation[5])*
                        (float)Printer::currentPositionSteps[X_AXIS]*Printer::invAxisStepsPerMM[X_AXIS])/
                      (Printer::autolevelTransformation[1]*Printer::autolevelTransformation[3]-Printer::autolevelTransformation[0]*Printer::autolevelTransformation[4]);
            Printer::zMin = 0;
            if(com->hasS() && com->S)
            {
#if MAX_HARDWARE_ENDSTOP_Z
#if DRIVE_SYSTEM==3
                /* Printer::offsetX = 0;
                 Printer::offsetY = 0;
                 Printer::moveToReal(0,0,cz,IGNORE_COORDINATE,Printer::homingFeedrate[X_AXIS]);
                     PrintLine::moveRelativeDistanceInSteps(Printer::offsetX-Printer::currentPositionSteps[0],Printer::offsetY-Printer::currentPositionSteps[1],0,0,Printer::homingFeedrate[0],true,ALWAYS_CHECK_ENDSTOPS);
                     Printer::offsetX = 0;
                     Printer::offsetY = 0;*/
                Printer::zLength += (h3+z)-Printer::currentPosition[Z_AXIS];
#else
                int32_t zBottom = Printer::currentPositionSteps[Z_AXIS] = (h3+z)*Printer::axisStepsPerMM[Z_AXIS];
                Printer::zLength = Printer::runZMaxProbe()+zBottom*Printer::invAxisStepsPerMM[2]-ENDSTOP_Z_BACK_ON_HOME;
#endif
                Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#else
#if DRIVE_SYSTEM!=3
                Printer::currentPositionSteps[Z_AXIS] = (h3+z)*Printer::axisStepsPerMM[Z_AXIS];
#endif
#endif
                Printer::setAutolevelActive(true);
                if(com->S == 2)
                    EEPROM::storeDataIntoEEPROM();
            }
            Printer::setAutolevelActive(true);
            Printer::updateDerivedParameter();
            Printer::updateCurrentPosition(true);
            printCurrentPosition();
#if DRIVE_SYSTEM==3
            Printer::homeAxis(true,true,true);
#endif
            Printer::feedrate = oldFeedrate;
        }
        break;
#endif
#endif
        case 90: // G90
            Printer::relativeCoordinateMode = false;
            break;
        case 91: // G91
            Printer::relativeCoordinateMode = true;
            break;
        case 92: // G92
        {
            float xOff = Printer::coordinateOffset[X_AXIS];
            float yOff = Printer::coordinateOffset[Y_AXIS];
            float zOff = Printer::coordinateOffset[Z_AXIS];
            if(com->hasX()) xOff = Printer::convertToMM(com->X)-Printer::currentPosition[X_AXIS];
            if(com->hasY()) yOff = Printer::convertToMM(com->Y)-Printer::currentPosition[Y_AXIS];
            if(com->hasZ()) zOff = Printer::convertToMM(com->Z)-Printer::currentPosition[Z_AXIS];
            Printer::setOrigin(xOff,yOff,zOff);
            if(com->hasE())
            {
                Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E)*Printer::axisStepsPerMM[E_AXIS];
            }
        }
        break;
#if DRIVE_SYSTEM==3
        case 131: // Remove offset
        {
            float cx,cy,cz;
            Printer::realPosition(cx,cy,cz);
            float oldfeedrate = Printer::feedrate;
            Printer::offsetX = 0;
            Printer::offsetY = 0;
            Printer::moveToReal(cx,cy,cz,IGNORE_COORDINATE,Printer::homingFeedrate[X_AXIS]);
            Printer::feedrate = oldfeedrate;
            Printer::updateCurrentPosition();
        }
        break;
        case 132: // Calibrate endstop offsets
        {
            Printer::setAutolevelActive(false); // don't let transformations change result!
            Printer::coordinateOffset[X_AXIS] = 0;
            Printer::coordinateOffset[Y_AXIS] = 0;
            Printer::coordinateOffset[Z_AXIS] = 0;
            Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
            int32_t m = RMath::max(Printer::stepsRemainingAtXHit,RMath::max(Printer::stepsRemainingAtYHit,Printer::stepsRemainingAtZHit));
            int32_t offx = m-Printer::stepsRemainingAtXHit;
            int32_t offy = m-Printer::stepsRemainingAtYHit;
            int32_t offz = m-Printer::stepsRemainingAtZHit;
            Com::printFLN(Com::tTower1,offx);
            Com::printFLN(Com::tTower2,offy);
            Com::printFLN(Com::tTower3,offz);
#if EEPROM_MODE!=0
            if(com->hasS() && com->S>0)
            {
                EEPROM::setDeltaTowerXOffsetSteps(offx);
                EEPROM::setDeltaTowerYOffsetSteps(offy);
                EEPROM::setDeltaTowerZOffsetSteps(offz);
            }
#endif
            Printer::homeAxis(true,true,true);
        }
        break;
        case 133: // Measure steps to top
            {
                bool oldAuto = Printer::isAutolevelActive();
            Printer::setAutolevelActive(false); // don't let transformations change result!
            Printer::currentPositionSteps[X_AXIS] = 0;
            Printer::currentPositionSteps[Y_AXIS] = 0;
            Printer::currentPositionSteps[Z_AXIS] = Printer::zMaxSteps;
            Printer::coordinateOffset[X_AXIS] = 0;
            Printer::coordinateOffset[Y_AXIS] = 0;
            Printer::coordinateOffset[Z_AXIS] = 0;
            Printer::currentDeltaPositionSteps[X_AXIS] = 0;
            Printer::currentDeltaPositionSteps[Y_AXIS] = 0;
            Printer::currentDeltaPositionSteps[Z_AXIS] = Printer::zMaxSteps;
            Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
            int32_t m = Printer::zMaxSteps*1.5;
            int32_t offx = m-Printer::stepsRemainingAtXHit;
            int32_t offy = m-Printer::stepsRemainingAtYHit;
            int32_t offz = m-Printer::stepsRemainingAtZHit;
            Com::printFLN(Com::tTower1,offx);
            Com::printFLN(Com::tTower2,offy);
            Com::printFLN(Com::tTower3,offz);
            Printer::setAutolevelActive(oldAuto);
            Printer::homeAxis(true,true,true);
            }
            break;
 /*       case 134:
            Com::printF(PSTR("CompDelta:"),Printer::currentDeltaPositionSteps[X_AXIS]);
            Com::printF(Com::tComma,Printer::currentDeltaPositionSteps[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::currentDeltaPositionSteps[Z_AXIS]);
#ifdef DEBUG_REAL_POSITION
            Com::printF(PSTR("RealDelta:"),Printer::realDeltaPositionSteps[X_AXIS]);
            Com::printF(Com::tComma,Printer::realDeltaPositionSteps[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::realDeltaPositionSteps[Z_AXIS]);
#endif
            Printer::updateCurrentPosition();
            Com::printF(PSTR("PosFromSteps:"));
            printCurrentPosition();
            break;
*/
#endif // DRIVE_SYSTEM
        }
        previousMillisCmd = HAL::timeInMilliseconds();
    }

    else if(com->hasM())    // Process M Code
    {

        switch( com->M )
        {
#if SDSUPPORT

        case 20: // M20 - list SD card
            sd.ls();
            break;
        case 21: // M21 - init SD card
            sd.mount();
            break;
        case 22: //M22 - release SD card
            sd.unmount();
            break;
        case 23: //M23 - Select file
            if(com->hasString())
            {
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
            if(com->hasString())
            {
                sd.fat.chdir();
                sd.deleteFile(com->text);
            }
            break;
        case 32: // M32 directoryname
            if(com->hasString())
            {
                sd.fat.chdir();
                sd.makeDirectory(com->text);
            }
            break;
#endif
        case 42: //M42 -Change pin status via gcode
            if (com->hasS() && com->hasP() && com->S>=0 && com->S<=255)
            {
                int pin_number = com->P;
                for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++)
                {
                    if (pgm_read_byte(&sensitive_pins[i]) == pin_number)
                    {
                        pin_number = -1;
                        break;
                    }
                }
                if (pin_number > -1)
                {
                    pinMode(pin_number, OUTPUT);
                    digitalWrite(pin_number, com->S);
                    analogWrite(pin_number, com->S);
                    Com::printF(Com::tSetOutputSpace,pin_number);
                    Com::printFLN(Com::tSpaceToSpace,(int)com->S);
                }
            }
            break;
        case 104: // M104
#if NUM_EXTRUDER>0
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
#ifdef EXACT_TEMPERATURE_TIMING
            Commands::waitUntilEndOfAllMoves();
#else
            if(com->hasP() || (com->hasS() && com->S == 0))
                Commands::waitUntilEndOfAllMoves();
#endif
            if (com->hasS())
            {
                if(com->hasT())
                    Extruder::setTemperatureForExtruder(com->S,com->T,com->hasF() && com->F>0);
                else
                    Extruder::setTemperatureForExtruder(com->S,Extruder::current->id,com->hasF() && com->F>0);
            }
#endif
            break;
        case 140: // M140 set bed temp
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
            if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F>0);
            break;
        case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
            printTemperatures(com->hasX());
            break;
        case 109: // M109 - Wait for extruder heater to reach target.
#if NUM_EXTRUDER>0
        {
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
            UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
            Commands::waitUntilEndOfAllMoves();
            Extruder *actExtruder = Extruder::current;
            if(com->hasT() && com->T<NUM_EXTRUDER) actExtruder = &extruder[com->T];
            if (com->hasS()) Extruder::setTemperatureForExtruder(com->S,actExtruder->id,com->hasF() && com->F>0);
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN>0
            if(abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)<(SKIP_M109_IF_WITHIN)) break; // Already in range
#endif
            bool dirRising = actExtruder->tempControl.targetTemperature > actExtruder->tempControl.currentTemperature;
            millis_t printedTime = HAL::timeInMilliseconds();
            millis_t waituntil = 0;
#if RETRACT_DURING_HEATUP
            uint8_t retracted = 0;
#endif
            millis_t currentTime;
            do
            {
                currentTime = HAL::timeInMilliseconds();
                if( (currentTime - printedTime) > 1000 )   //Print Temp Reading every 1 second while heating up.
                {
                    printTemperatures();
                    printedTime = currentTime;
                }
                Commands::checkForPeriodicalActions();
                //gcode_read_serial();
#if RETRACT_DURING_HEATUP
                if (actExtruder == Extruder::current && actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.currentTemperatureC > actExtruder->waitRetractTemperature)
                {
                    PrintLine::moveRelativeDistanceInSteps(0,0,0,-actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS],actExtruder->maxFeedrate,false,false);
                    retracted = 1;
                }
#endif
                if((waituntil == 0 &&
                    (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - 1 : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC+1))
#if defined(TEMP_HYSTERESIS) && TEMP_HYSTERESIS>=1
                        || (waituntil!=0 && (abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC))>TEMP_HYSTERESIS)
#endif
                  )
                {
                    waituntil = currentTime+1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabalize
                }
            }
            while(waituntil==0 || (waituntil!=0 && (millis_t)(waituntil-currentTime)<2000000000UL));
#if RETRACT_DURING_HEATUP
            if (retracted && actExtruder==Extruder::current)
            {
                PrintLine::moveRelativeDistanceInSteps(0,0,0,actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS],actExtruder->maxFeedrate,false,false);
            }
#endif
        }
        UI_CLEAR_STATUS;
#endif
        previousMillisCmd = HAL::timeInMilliseconds();
        break;
        case 190: // M190 - Wait bed for heater to reach target.
#if HAVE_HEATED_BED
            if(Printer::debugDryrun()) break;
            UI_STATUS_UPD(UI_TEXT_HEATING_BED);
            Commands::waitUntilEndOfAllMoves();
#if HAVE_HEATED_BED
            if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F>0);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN>0
            if(abs(heatedBedController.currentTemperatureC-heatedBedController.targetTemperatureC)<SKIP_M190_IF_WITHIN) break;
#endif
            codenum = HAL::timeInMilliseconds();
            while(heatedBedController.currentTemperatureC+0.5<heatedBedController.targetTemperatureC)
            {
                if( (HAL::timeInMilliseconds()-codenum) > 1000 )   //Print Temp Reading every 1 second while heating up.
                {
                    printTemperatures();
                    codenum = HAL::timeInMilliseconds();
                }
                Commands::checkForPeriodicalActions();
            }
#endif
#endif
            UI_CLEAR_STATUS;
            previousMillisCmd = HAL::timeInMilliseconds();
            break;
        case 116: // Wait for temperatures to reach target temperature
            if(Printer::debugDryrun()) break;
            {
                bool allReached = false;
                codenum = HAL::timeInMilliseconds();
                while(!allReached) {
                    allReached = true;
                    if( (HAL::timeInMilliseconds()-codenum) > 1000 )   //Print Temp Reading every 1 second while heating up.
                    {
                        printTemperatures();
                        codenum = HAL::timeInMilliseconds();
                    }
                    Commands::checkForPeriodicalActions();
                    for(uint8_t h=0;h<NUM_TEMPERATURE_LOOPS;h++) {
                        TemperatureController *act = tempController[h];
                        if(act->targetTemperatureC>30 && fabs(act->targetTemperatureC-act->currentTemperatureC)>1)
                            allReached = false;
                    }
                }
            }
            break;
#if FEATURE_DITTO_PRINTING
        case 280:
            if(com->hasS())   // Set ditto mode S: 0 = off, 1 = 1 extra extruder, 2 = 2 extra extruder, 3 = 3 extra extruders
            {
                Extruder::dittoMode = com->S;
            }
            break;
#endif

#if defined(BEEPER_PIN) && BEEPER_PIN>=0
        case 300:
        {
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
        case 303:
        {
#if defined(TEMP_PID) && NUM_TEMPERATURE_LOOPS>0
            int temp = 150;
            int cont = 0;
            if(com->hasS()) temp = com->S;
            if(com->hasP()) cont = com->P;
            if(cont>=NUM_TEMPERATURE_LOOPS) cont = NUM_TEMPERATURE_LOOPS;
            tempController[cont]->autotunePID(temp,cont,com->hasX());
#endif
        }
        break;
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
        case 106: //M106 Fan On
            setFanSpeed(com->hasS()?com->S:255,com->hasP());
            break;
        case 107: //M107 Fan Off
            setFanSpeed(0,com->hasP());
            break;
#endif
        case 80: // M80 - ATX Power On
#if PS_ON_PIN>-1
            Commands::waitUntilEndOfAllMoves();
            previousMillisCmd = HAL::timeInMilliseconds();
            SET_OUTPUT(PS_ON_PIN); //GND
            WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif
            break;
        case 81: // M81 - ATX Power Off
#if PS_ON_PIN>-1
            Commands::waitUntilEndOfAllMoves();
            SET_OUTPUT(PS_ON_PIN); //GND
            WRITE(PS_ON_PIN,(POWER_INVERTING ? LOW : HIGH));
#endif
            break;
        case 82:
            Printer::relativeExtruderCoordinateMode = false;
            break;
        case 83:
            Printer::relativeExtruderCoordinateMode = true;
            break;
        case 84:
            if(com->hasS())
            {
                stepperInactiveTime = com->S * 1000;
            }
            else
            {
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
            if(com->hasX()) Printer::axisStepsPerMM[0] = com->X;
            if(com->hasY()) Printer::axisStepsPerMM[1] = com->Y;
            if(com->hasZ()) Printer::axisStepsPerMM[2] = com->Z;
            Printer::updateDerivedParameter();
            if(com->hasE())
            {
                Extruder::current->stepsPerMM = com->E;
                Extruder::selectExtruderById(Extruder::current->id);
            }
            break;
        case 99: // M99 S<time>
            {
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
        case 111:
            if(com->hasS()) Printer::debugLevel = com->S;
            if(Printer::debugDryrun())   // simulate movements without printing
            {
                Extruder::setTemperatureForExtruder(0,0);
#if NUM_EXTRUDER>1
                Extruder::setTemperatureForExtruder(0,1);
#endif
#if HEATED_BED_TYPE!=0
                target_bed_raw = 0;
#endif
            }
            break;
        case 115:
            Com::printFLN(Com::tFirmware);
            reportPrinterUsage();
            break;
        case 114: // M114
            printCurrentPosition();
            break;
        case 117: // M117 message to lcd
            if(com->hasString())
            {
                UI_STATUS_UPD_RAM(com->text);
            }
            break;
        case 119: // M119
            Commands::waitUntilEndOfAllMoves();
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
            Com::printF(Com::tXMinColon);
            Com::printF(Printer::isXMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
            Com::printF(Com::tXMaxColon);
            Com::printF(Printer::isXMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
            Com::printF(Com::tYMinColon);
            Com::printF(Printer::isYMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
            Com::printF(Com::tYMaxColon);
            Com::printF(Printer::isYMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
            Com::printF(Com::tZMinColon);
            Com::printF(Printer::isZMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
            Com::printF(Com::tZMaxColon);
            Com::printF(Printer::isZMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif
            Com::println();
            break;
#if BEEPER_TYPE>0
        case 120: // Test beeper function
            if(com->hasS() && com->hasP())
                beep(com->S,com->P); // Beep test
            break;
#endif
#ifdef RAMP_ACCELERATION
        case 201: // M201
            if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[0] = com->X;
            if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[1] = com->Y;
            if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[2] = com->Z;
            if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[3] = com->E;
            Printer::updateDerivedParameter();
            break;
        case 202: // M202
            if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[0] = com->X;
            if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[1] = com->Y;
            if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[2] = com->Z;
            if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[3] = com->E;
            Printer::updateDerivedParameter();
            break;
#endif
        case 203: // M203 Temperature monitor
            if(com->hasS())
            {
                if(com->S<NUM_EXTRUDER) manageMonitor = com->S;
#if HAVE_HEATED_BED
                else manageMonitor=NUM_EXTRUDER; // Set 100 to heated bed
#endif
            }
            break;
        case 204:
        {
            TemperatureController *temp = &Extruder::current->tempControl;
            if(com->hasS())
            {
                if(com->S<0) break;
                if(com->S<NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
#if HAVE_HEATED_BED
                else temp = &heatedBedController;
#else
                else break;
#endif
            }
            if(com->hasX()) temp->pidPGain = com->X;
            if(com->hasY()) temp->pidIGain = com->Y;
            if(com->hasZ()) temp->pidDGain = com->Z;
            temp->updateTempControlVars();
        }
        break;
        case 503:
        case 205: // M205 Show EEPROM settings
            EEPROM::writeSettings();
            break;
        case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
            EEPROM::update(com);
            break;
        case 207: // M207 X<XY jerk> Z<Z Jerk>
            if(com->hasX())
                Printer::maxJerk = com->X;
            if(com->hasE())
            {
                Extruder::current->maxStartFeedrate = com->E;
                Extruder::selectExtruderById(Extruder::current->id);
            }
#if DRIVE_SYSTEM!=3
            if(com->hasZ())
                Printer::maxZJerk = com->Z;
            Com::printF(Com::tJerkColon,Printer::maxJerk);
            Com::printFLN(Com::tZJerkColon,Printer::maxZJerk);
#else
            Com::printFLN(Com::tJerkColon,Printer::maxJerk);
#endif
            break;
        case 220: // M220 S<Feedrate multiplier in percent>
            changeFeedrateMultiply(com->getS(100));
            break;
        case 221: // M221 S<Extrusion flow multiplier in percent>
            changeFlowateMultiply(com->getS(100));
            break;
#ifdef USE_ADVANCE
        case 223: // Extruder interrupt test
            if(com->hasS())
            {
                BEGIN_INTERRUPT_PROTECTED
                Printer::extruderStepsNeeded += com->S;
                END_INTERRUPT_PROTECTED
            }
            break;
        case 232:
            Com::printF(Com::tLinearStepsColon,maxadv2);
#ifdef ENABLE_QUADRATIC_ADVANCE
            Com::printF(Com::tQuadraticStepsColon,maxadv);
#endif
            Com::printFLN(Com::tCommaSpeedEqual,maxadvspeed);
#ifdef ENABLE_QUADRATIC_ADVANCE
            maxadv=0;
#endif
            maxadv2=0;
            maxadvspeed=0;
            break;
#endif
#ifdef USE_ADVANCE
        case 233:
            if(com->hasY())
                Extruder::current->advanceL = com->Y;
            Com::printF(Com::tLinearLColon,Extruder::current->advanceL);
#ifdef ENABLE_QUADRATIC_ADVANCE
            if(com->hasX())
                Extruder::current->advanceK = com->X;
            Com::printF(Com::tQuadraticKColon,Extruder::current->advanceK);
#endif
            Com::println();
            Printer::updateAdvanceFlags();
            break;
#endif
        case 400: // Finish all moves
            Commands::waitUntilEndOfAllMoves();
            break;
#if FEATURE_MEMORY_POSITION
        case 401: // Memory position
            Printer::MemoryPosition();
            break;
        case 402: // Go to stored position
            Printer::GoToMemoryPosition(com->hasX(),com->hasY(),com->hasZ(),com->hasE(),(com->hasF() ? com->F : Printer::feedrate));
            break;
#endif
        case 908: // Control digital trimpot directly.
        {
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
            uint8_t channel,current;
            if(com->hasP() && com->hasS())
                setMotorCurrent((uint8_t)com->P, (unsigned int)com->S);
#endif
        }
        break;
        case 500:
        {
#if EEPROM_MODE!=0
            EEPROM::storeDataIntoEEPROM(false);
            Com::printInfoF(Com::tConfigStoredEEPROM);
#else
            Com::printErrorF(Com::tNoEEPROMSupport);
#endif
        }
        break;
        case 501:
        {
#if EEPROM_MODE!=0
            EEPROM::readDataFromEEPROM();
            Extruder::selectExtruderById(Extruder::current->id);
            Com::printInfoF(Com::tConfigLoadedEEPROM);
#else
            Com::printErrorF(Com::tNoEEPROMSupport);
#endif
        }
        break;
        case 502:
            EEPROM::restoreEEPROMSettingsFromConfiguration();
            break;

        case 350: // Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
        {
            OUT_P_LN("Set Microstepping");
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
            if(com->hasS()) for(int i=0; i<=4; i++) microstepMode(i,com->S);
            if(com->hasX()) microstepMode(0,(uint8_t)com->X);
            if(com->hasY()) microstepMode(1,(uint8_t)com->Y);
            if(com->hasZ()) microstepMode(2,(uint8_t)com->Z);
            if(com->hasE()) microstepMode(3,(uint8_t)com->E);
            if(com->hasP()) microstepMode(4,com->P); // Original B but is not supported here
            microstepReadings();
#endif
        }
        break;
#if FEATURE_Z_PROBE
#if FEATURE_AUTOLEVEL
        case 320: // Activate autolevel
            Printer::setAutolevelActive(true);
            if(com->hasS() && com->S)
            {
                EEPROM::storeDataIntoEEPROM();
            }
            break;
        case 321: // Deactivate autoleveling
            Printer::setAutolevelActive(false);
            if(com->hasS() && com->S)
            {
                if(com->S==3)
                    Printer::resetTransformationMatrix(false);
                EEPROM::storeDataIntoEEPROM();
            }
            break;
        case 322: // Reset autoeveling matrix
            Printer::resetTransformationMatrix(false);
            if(com->hasS() && com->S)
            {
                EEPROM::storeDataIntoEEPROM();
            }
            break;
            /*        case 700: // test new square root function
                        if(com->hasS())
                            Com::printFLN(Com::tInfo,(int32_t)HAL::integerSqrt(com->S));
                        break;*/
#endif // FEATURE_AUTOLEVEL
#endif // FEATURE_Z_PROBE
#if FEATURE_SERVO
        case 340:
            if(com->hasP() && com->P<4 && com->P>=0)
            {
                int s = 0;
                if(com->hasS())
                    s = com->S;
                HAL::servoMicroseconds(com->P,s);
            }
            break;
#endif // FEATURE_SERVO
#if Z_HOME_DIR>0 && MAX_HARDWARE_ENDSTOP_Z
        case 251:
            Printer::zLength -= Printer::currentPosition[Z_AXIS];
            Printer::currentPositionSteps[Z_AXIS] = 0;
            Printer::updateDerivedParameter();
#if NONLINEAR_SYSTEM
            transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
#endif
            Printer::updateCurrentPosition();
            Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#if EEPROM_MODE!=0
            EEPROM::storeDataIntoEEPROM(false);
            Com::printFLN(Com::tEEPROMUpdated);
#endif
            Commands::printCurrentPosition();
            break;
#endif
#ifdef DEBUG_QUEUE_MOVE
        case 533: // Write move data
            Com::printF(PSTR("Buf:"),(int)PrintLine::linesCount);
            Com::printF(PSTR(",LP:"),(int)PrintLine::linesPos);
            Com::printFLN(PSTR(",WP:"),(int)PrintLine::linesWritePos);
            if(PrintLine::cur == NULL) {
                Com::printFLN(PSTR("No move"));
                if(PrintLine::linesCount>0) {
                    PrintLine &cur = PrintLine::lines[PrintLine::linesPos];
                    Com::printF(PSTR("JFlags:"),(int)cur.joinFlags);
                    Com::printFLN(PSTR("Flags:"),(int)cur.flags);
                    if(cur.isWarmUp()) {
                        Com::printFLN(PSTR("warmup:"),(int)cur.getWaitForXLinesFilled());
                    }
                }
            } else {
                Com::printF(PSTR("Rem:"),PrintLine::cur->stepsRemaining);
                Com::printFLN(PSTR("Int:"),Printer::interval);
            }
            break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
        case 534:
            Com::printFLN(PSTR("Max. segment size:"),Printer::maxRealSegmentLength);
            if(com->hasS())
                Printer::maxRealSegmentLength = 0;
            break;
#endif
#ifdef DEBUG_REAL_JERK
            Com::printFLN(PSTR("Max. jerk measured:"),Printer::maxRealJerk);
            if(com->hasS())
                Printer::maxRealJerk = 0;
            break;
#endif
/*        case 535:
            Com::printF(PSTR("Last commanded position:"),Printer::lastCmdPos[X_AXIS]);
            Com::printF(Com::tComma,Printer::lastCmdPos[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::lastCmdPos[Z_AXIS]);
            Com::printF(PSTR("Current position:"),Printer::currentPosition[X_AXIS]);
            Com::printF(Com::tComma,Printer::currentPosition[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::currentPosition[Z_AXIS]);
            Com::printF(PSTR("Position steps:"),Printer::currentPositionSteps[X_AXIS]);
            Com::printF(Com::tComma,Printer::currentPositionSteps[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::currentPositionSteps[Z_AXIS]);
#if NONLINEAR_SYSTEM
            Com::printF(PSTR("Nonlin. position steps:"),Printer::currentDeltaPositionSteps[X_AXIS]);
            Com::printF(Com::tComma,Printer::currentDeltaPositionSteps[Y_AXIS]);
            Com::printFLN(Com::tComma,Printer::currentDeltaPositionSteps[Z_AXIS]);
#endif // NONLINEAR_SYSTEM
            break;*/
        }
    }
    else if(com->hasT())      // Process T code
    {
        Commands::waitUntilEndOfAllMoves();
        Extruder::selectExtruderById(com->T);
    }
    else
    {
        if(Printer::debugErrors())
        {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}
void Commands::emergencyStop()
{
#if defined(KILL_METHOD) && KILL_METHOD==1
    HAL::resetHardware();
#else
    BEGIN_INTERRUPT_PROTECTED
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Printer::kill(false);
    Extruder::manageTemperatures();
    for(uint8_t i=0; i<NUM_EXTRUDER+3; i++)
        pwm_pos[i] = 0;
    pwm_pos[0] = pwm_pos[NUM_EXTRUDER] = pwm_pos[NUM_EXTRUDER+1] = pwm_pos[NUM_EXTRUDER+2]=0;
#if EXT0_HEATER_PIN>-1
    WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    WRITE(EXT1_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    WRITE(EXT2_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    WRITE(EXT3_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    WRITE(EXT4_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    WRITE(EXT5_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if FAN_PIN>-1
    WRITE(FAN_PIN,0);
#endif
#if HEATED_BED_HEATER_PIN>-1
    WRITE(HEATED_BED_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
    while(1) {}
    END_INTERRUPT_PROTECTED
#endif
}

void Commands::checkFreeMemory()
{
    int newfree = HAL::getFreeRam();
    if(newfree<lowestRAMValue)
    {
        lowestRAMValue = newfree;
    }
}
void Commands::writeLowestFreeRAM()
{
    if(lowestRAMValueSend>lowestRAMValue)
    {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM,lowestRAMValue);
    }

}
