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
int Commands::lowestRAMValue=MAX_RAM;
int Commands::lowestRAMValueSend=MAX_RAM;

void Commands::commandLoop()
{
    while(true)
    {
        GCode::readFromSerial();
        GCode *code = GCode::peekCurrentCommand();
        //UI_SLOW; // do longer timed user interface action
        UI_MEDIUM; // do check encoder
        if(code)
        {
#if SDSUPPORT
            if(sd.savetosd)
            {
                if(!(code->hasM() && code->M==29))   // still writing to file
                {
                    sd.write_command(code);
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
    if(!execute_periodical) return;
    execute_periodical=0;
    Extruder::manageTemperatures();
    if(--counter_250ms==0)
    {
        if(manage_monitor<=1+NUM_EXTRUDER)
            write_monitor();
        counter_250ms=5;
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
    while(PrintLine::hasLines())
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
        UI_MEDIUM;
    }
}
void Commands::printCurrentPosition()
{
    float x,y,z;
    Printer::realPosition(x,y,z);
    Com::printF(Com::tXColon,x*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceYColon,y*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceZColon,z*(Printer::unitIsInches?0.03937:1),2);
    Com::printFLN(Com::tSpaceEColon,Printer::currentPositionSteps[3]*Printer::invAxisStepsPerMM[3]*(Printer::unitIsInches?0.03937:1),2);
}
void Commands::printTemperatures()
{
    float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE==0
    Com::printF(Com::tTColon,temp);
#else
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceBColon,Extruder::getHeatedBedTemperature());
#endif
#ifdef TEMP_PID
    Com::printF(Com::tSpaceAtColon,(autotuneIndex==255?pwm_pos[Extruder::current->id]:pwm_pos[autotuneIndex])); // Show output of autotune when tuning!
#endif
#if NUM_EXTRUDER>1
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
        Com::printF(Com::tSpaceT,(int)i);
        Com::printF(Com::tColon,extruder[i].tempControl.currentTemperatureC);
#ifdef TEMP_PID
        Com::printF(Com::tSpaceAt,(int)i);
        Com::printF(Com::tColon,(pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!
#endif
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

    long seconds = (alloff ? 0 : (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME);
    long tmp = seconds/86400;
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

void set_current(uint8_t driver, unsigned int current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}
#endif

void current_control_init() //Initialize Digipot Motor Current
{
#if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;

    HAL::spiInit(0); //SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
    for(int i=0; i<=4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        set_current(i,digipot_motor_current[i]);
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_LTC2600

void set_current( uint8_t channel, unsigned short level )
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

void current_control_init() //Initialize LTC2600 Motor Current
{
    const unsigned int ltc_current[] =  MOTOR_CURRENT;
    uint8_t i;
    for(i=0; i<LTC2600_NUM_CHANNELS; i++)
    {
        set_current(i, ltc_current[i] );
    }
}
#endif

#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
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

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
    switch(stepping_mode)
    {
    case 1:
        microstep_ms(driver,MICROSTEP1);
        break;
    case 2:
        microstep_ms(driver,MICROSTEP2);
        break;
    case 4:
        microstep_ms(driver,MICROSTEP4);
        break;
    case 8:
        microstep_ms(driver,MICROSTEP8);
        break;
    case 16:
        microstep_ms(driver,MICROSTEP16);
        break;
    }
}
void microstep_readings()
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

              void microstep_init()
{
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    SET_OUTPUT(X_MS2_PIN);
    SET_OUTPUT(Y_MS2_PIN);
    SET_OUTPUT(Z_MS2_PIN);
    SET_OUTPUT(E0_MS2_PIN);
    SET_OUTPUT(E1_MS2_PIN);
    for(int i=0; i<=4; i++) microstep_mode(i,microstep_modes[i]);
#endif
}



/**
  \brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com)
{
    unsigned long codenum; //throw away variable
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
            if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if DRIVE_SYSTEM == 3
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
            Printer::realPosition(position[0],position[1],position[2]);
            if(!Printer::setDestinationStepsFromGCode(com)) break; // For X Y Z E F
            float offset[2] = {Printer::convertToMM(com->hasI()?com->I:0),Printer::convertToMM(com->hasJ()?com->J:0)};
            float target[4] = {Printer::realXPosition(),Printer::realYPosition(),Printer::realZPosition(),Printer::destinationSteps[3]*Printer::invAxisStepsPerMM[3]};
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
                double x = target[0]-position[0];
                double y = target[1]-position[1];

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
            while((unsigned long)(codenum-HAL::timeInMilliseconds())  < 2000000000 )
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
            uint8_t home_all_axis = (com->hasNoXYZ());
            Printer::homeAxis(home_all_axis || com->hasX(),home_all_axis || com->hasY(),home_all_axis || com->hasZ());
            Printer::updateCurrentPosition();
        }
        break;
#if FEATURE_Z_PROBE
        case 29: // 3 points, build average
        {
            bool oldAutolevel = Printer::isAutolevelActive();
            Printer::setAutolevelActive(false);
            float sum = 0,last,oldFeedrate = Printer::feedrate;
            Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            sum = Printer::runZProbe(true,false);
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
                Printer::currentPositionSteps[2] = sum*Printer::axisStepsPerMM[2];
                Com::printInfoFLN(Com::tZProbeZReset);
#if MAX_HARDWARE_ENDSTOP_Z
                Printer::zLength = Printer::runZMaxProbe()+sum-ENDSTOP_Z_BACK_ON_HOME;
                Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#endif
            }
            Printer::feedrate = oldFeedrate;
            Printer::setAutolevelActive(oldAutolevel);
            printCurrentPosition();
            if(com->hasS() && com->S == 2)
                EEPROM::storeDataIntoEEPROM();
            Printer::updateCurrentPosition();
        }
        break;
        case 30: //single probe set Z0
        {
            bool oldAutolevel = Printer::isAutolevelActive();
            Printer::setAutolevelActive(false);
            Printer::runZProbe(true,true);
            Printer::setAutolevelActive(oldAutolevel);
        }
        break;
        case 31:  //display hall sensor output
            Com::printF(Com::tZProbeState);
            Com::print(Printer::isZProbeHit() ? 'H' : 'L');
            Com::println();
            break;
        case 32: // Auto-Bed leveling
        {
            bool iterate = com->hasP() && com->P>0;
            Printer::coordinateOffset[0] = Printer::coordinateOffset[1] = Printer::coordinateOffset[2] = 0;
            Printer::setAutolevelActive(iterate);
#if DRIVE_SYSTEM==3
            Printer::homeAxis(true,true,true);
            Printer::setAutolevelActive(false);
            float aboveDist = Printer::zLength-Z_PROBE_GAP-EEPROM::zProbeHeight();
            PrintLine::moveRelativeDistanceInSteps(0,0,-Printer::axisStepsPerMM[0]*aboveDist,0,Printer::homingFeedrate[0], true, false);
#endif
            float h1,h2,h3,hc,oldFeedrate = Printer::feedrate;
            Printer::moveTo(EEPROM::zProbeX1(),EEPROM::zProbeY1(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h1 = Printer::runZProbe(true,false);
            if(h1<0) break;
            Printer::moveTo(EEPROM::zProbeX2(),EEPROM::zProbeY2(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h2 = Printer::runZProbe(false,false);
            if(h2<0) break;
            Printer::moveTo(EEPROM::zProbeX3(),EEPROM::zProbeY3(),IGNORE_COORDINATE,IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            h3 = Printer::runZProbe(false,true);
            if(h3<0) break;
            Printer::buildTransformationMatrix(h1,h2,h3);
#if DRIVE_SYSTEM==3
            // Compute z height at the tower positions
            float ox,oy,ozx,ozy,ozz,oz;
            Printer::transformFromPrinter(-Printer::deltaSin60RadiusSteps, Printer::deltaMinusCos60RadiusSteps,0,ox,oy,ozx);
            Printer::transformFromPrinter(Printer::deltaSin60RadiusSteps, Printer::deltaMinusCos60RadiusSteps,0,ox,oy,ozy);
            Printer::transformFromPrinter(0, Printer::deltaRadiusSteps,0,ox,oy,ozz);
            Com::printFLN(Com::tTower1,ozx);
            Com::printFLN(Com::tTower2,ozy);
            Com::printFLN(Com::tTower3,ozz);
            if(iterate) {
                ozx+=EEPROM::deltaTowerXOffsetSteps();
                ozy+=EEPROM::deltaTowerYOffsetSteps();
                ozz+=EEPROM::deltaTowerZOffsetSteps();
            }
            ox = RMath::min(ozx,RMath::min(ozy,ozz));
            oy = RMath::max(ozx,RMath::max(ozy,ozz));
            EEPROM::setDeltaTowerXOffsetSteps(-ox+ozx);
            EEPROM::setDeltaTowerYOffsetSteps(-ox+ozy);
            EEPROM::setDeltaTowerZOffsetSteps(-ox+ozz);
            Printer::zLength = aboveDist+(ox)*Printer::invAxisStepsPerMM[2]+(h1+h2+h3)/3.0;
            Printer::setAutolevelActive(true);
            Printer::updateDerivedParameter();
            Printer::homeAxis(true,true,true);
            aboveDist = Printer::zLength-Z_PROBE_GAP-EEPROM::zProbeHeight();
            Printer::moveTo(0,0,Z_PROBE_GAP+EEPROM::zProbeHeight(),IGNORE_COORDINATE,EEPROM::zProbeXYSpeed());
            hc = Printer::runZProbe(true,true);
            if(hc<0) break;
            Printer::zLength = aboveDist+hc;
            if(com->hasS() && com->S)
            {
                if(com->S == 2)
                    EEPROM::storeDataIntoEEPROM();
            }
            Printer::setAutolevelActive(true);
#else
            //-(Rxx*Ryz*y-Rxz*Ryx*y+(Rxz*Ryy-Rxy*Ryz)*x)/(Rxy*Ryx-Rxx*Ryy)
            float z = -((Printer::autolevelTransformation[0]*Printer::autolevelTransformation[5]-Printer::autolevelTransformation[2]*Printer::autolevelTransformation[3])*
                       (float)Printer::currentPositionSteps[1]*Printer::invAxisStepsPerMM[0]+(Printer::autolevelTransformation[2]*Printer::autolevelTransformation[4]-
                               Printer::autolevelTransformation[1]*Printer::autolevelTransformation[5])*(float)Printer::currentPositionSteps[0]*Printer::invAxisStepsPerMM[0])/
                      (Printer::autolevelTransformation[1]*Printer::autolevelTransformation[3]-Printer::autolevelTransformation[0]*Printer::autolevelTransformation[4]);
            long zBottom = Printer::currentPositionSteps[2] = (h3+z)*Printer::axisStepsPerMM[2];
            Printer::zMin = 0;
            if(com->hasS() && com->S)
            {
#if MAX_HARDWARE_ENDSTOP_Z
                Printer::zLength = Printer::runZMaxProbe()+zBottom*Printer::invAxisStepsPerMM[2]-ENDSTOP_Z_BACK_ON_HOME;
                Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#endif
                Printer::setAutolevelActive(true);
                if(com->S == 2)
                    EEPROM::storeDataIntoEEPROM();
            }
            Printer::setAutolevelActive(true);
            Printer::updateDerivedParameter();
            Printer::updateCurrentPosition();
            printCurrentPosition();
#endif
            Printer::feedrate = oldFeedrate;
        }
        break;
#endif
        case 90: // G90
            Printer::relativeCoordinateMode = false;
            break;
        case 91: // G91
            Printer::relativeCoordinateMode = true;
            break;
        case 92: // G92
        {
            float xOff = 0,yOff = 0, zOff = 0;
            float xPos,yPos,zPos;
            Printer::realPosition(xPos,yPos,zPos);
            if(com->hasX()) xOff = Printer::convertToMM(com->X)-xPos;
            if(com->hasY()) yOff = Printer::convertToMM(com->Y)-yPos;
            if(com->hasZ()) zOff = Printer::convertToMM(com->Z)-zPos;
            Printer::setOrigin(xOff,yOff,zOff);
            if(com->hasE())
            {
                Printer::currentPositionSteps[3] = Printer::convertToMM(com->E)*Printer::axisStepsPerMM[3];
            }
        }
        break;

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
                sd.selectFile(com->text);
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
                sd.deleteFile(com->text);
            break;
        case 32: // M32 directoryname
            if(com->hasString())
                sd.makeDirectory(com->text);
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
                    Extruder::setTemperatureForExtruder(com->S,com->T);
                else
                    Extruder::setTemperatureForExtruder(com->S,Extruder::current->id);
            }
#endif
            break;
        case 140: // M140 set bed temp
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
            if (com->hasS()) Extruder::setHeatedBedTemperature(com->S);
            break;
        case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
            printTemperatures();
            break;
        case 109: // M109 - Wait for extruder heater to reach target.
        {
#if NUM_EXTRUDER>0
            if(reportTempsensorError()) break;
            previousMillisCmd = HAL::timeInMilliseconds();
            if(Printer::debugDryrun()) break;
            UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
            Commands::waitUntilEndOfAllMoves();
            Extruder *actExtruder = Extruder::current;
            if(com->hasT() && com->T<NUM_EXTRUDER) actExtruder = &extruder[com->T];
            if (com->hasS()) Extruder::setTemperatureForExtruder(com->S,actExtruder->id);
#if defined(SKIP_M109_IF_WITHIN) && SKIP_M109_IF_WITHIN>0
            if(abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC)<(SKIP_M109_IF_WITHIN)) break; // Already in range
#endif
            bool dirRising = actExtruder->tempControl.targetTemperature > actExtruder->tempControl.currentTemperature;
            millis_t printedTime = HAL::timeInMilliseconds();
            millis_t waituntil = 0;
#if RETRACT_DURING_HEATUP
            uint8_t retracted = 0;
#endif
            millis_t cur_time;
            do
            {
                cur_time = HAL::timeInMilliseconds();
                if( (cur_time - printedTime) > 1000 )   //Print Temp Reading every 1 second while heating up.
                {
                    printTemperatures();
                    printedTime = cur_time;
                }
                Commands::checkForPeriodicalActions();
                //gcode_read_serial();
#if RETRACT_DURING_HEATUP
                if (actExtruder==Extruder::current && actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.currentTemperatureC > actExtruder->waitRetractTemperature)
                {
                    PrintLine::moveRelativeDistanceInSteps(0,0,0,-actExtruder->waitRetractUnits * Printer::axisStepsPerMM[3],actExtruder->maxFeedrate,false,false);
                    retracted = 1;
                }
#endif
                if((waituntil==0 && (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC-1:actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC+1))
#ifdef TEMP_HYSTERESIS
                        || (waituntil!=0 && (abs(actExtruder->tempControl.currentTemperatureC - actExtruder->tempControl.targetTemperatureC))>TEMP_HYSTERESIS)
#endif
                  )
                {
                    waituntil = cur_time+1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabalize
                }
            }
            while(waituntil==0 || (waituntil!=0 && (millis_t)(waituntil-cur_time)<2000000000UL));
#if RETRACT_DURING_HEATUP
            if (retracted && actExtruder==Extruder::current)
            {
                PrintLine::moveRelativeDistanceInSteps(0,0,0,actExtruder->waitRetractUnits * Printer::axisStepsPerMM[3],actExtruder->maxFeedrate,false,false);
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
            if (com->hasS()) Extruder::setHeatedBedTemperature(com->S);
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
#ifdef BEEPER_PIN
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
            tempController[cont]->autotunePID(temp,cont);
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
                maxInactiveTime = (long)com->S * 1000;
            else
                maxInactiveTime = 0;
            break;
        case 92: // M92
            if(com->hasX()) Printer::axisStepsPerMM[0] = com->X;
            if(com->hasY()) Printer::axisStepsPerMM[1] = com->Y;
            if(com->hasZ()) Printer::axisStepsPerMM[2] = com->Z;
            if(com->hasE()) Extruder::current->stepsPerMM = com->E;
            Printer::updateDerivedParameter();
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
                if(com->S<NUM_EXTRUDER) manage_monitor = com->S;
#if HAVE_HEATED_BED
                else manage_monitor=NUM_EXTRUDER; // Set 100 to heated bed
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
            if(com->hasS()) {
                BEGIN_INTERRUPT_PROTECTED
                Printer::extruderStepsNeeded+=com->S;
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
            Printer::memoryX = Printer::currentPositionSteps[0];
            Printer::memoryY = Printer::currentPositionSteps[1];
            Printer::memoryZ = Printer::currentPositionSteps[2];
            break;
        case 402: // Go to stored position
        {
            bool all = !(com->hasX() && com->hasY() && com->hasZ());
            PrintLine::moveRelativeDistanceInSteps((all || com->hasX() ? Printer::memoryX-Printer::currentPositionSteps[0] : 0)
                                                   ,(all || com->hasY() ? Printer::memoryY-Printer::currentPositionSteps[1] : 0)
                                                   ,(all || com->hasZ() ? Printer::memoryZ-Printer::currentPositionSteps[2] : 0)
                                                   ,0,(com->hasF() ? com->F : Printer::feedrate),false,ALWAYS_CHECK_ENDSTOPS);
        }
        break;
#endif
        case 908: // Control digital trimpot directly.
        {
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
            uint8_t channel,current;
            if(com->hasP() && com->hasS())
                set_current((uint8_t)com->P, (unsigned int)com->S);
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
            if(com->hasS()) for(int i=0; i<=4; i++) microstep_mode(i,com->S);
            if(com->hasX()) microstep_mode(0,(uint8_t)com->X);
            if(com->hasY()) microstep_mode(1,(uint8_t)com->Y);
            if(com->hasZ()) microstep_mode(2,(uint8_t)com->Z);
            if(com->hasE()) microstep_mode(3,(uint8_t)com->E);
            if(com->hasP()) microstep_mode(4,com->P); // Original B but is not supported here
            microstep_readings();
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
                Com::printFLN(Com::tInfo,(long)HAL::integerSqrt(com->S));
            break;*/
#endif // FEATURE_AUTOLEVEL
#endif // FEATURE_Z_PROBE
#if FEATURE_SERVO
        case 340:
            if(com->hasP() && com->P<4 && com->P>=0) {
                int s = 0;
                if(com->hasS())
                    s = com->S;
                HAL::servoMicroseconds(com->P,s);
            }
#endif // FEATURE_SERVO
#ifdef STEP_COUNTER
#if DRIVE_SYSTEM==3
        case 251:
            if(com->hasS())
            {
                if (com->S == 0)
                {
                    Printer::countZSteps = 0;
                    Com::printFLN(Com::tMeasurementReset);
                }
                else if (com->S == 1)
                {
                    Com::printFLN(Com::tMeasureDeltaSteps,Printer::countZSteps);
                    Com::printFLN(Com::tMeasureDelta,Printer::countZSteps * Printer::invAxisStepsPerMM[2]);
                }
                else if (com->S = 2)
                {
                    if (Printer::currentPositionSteps[0] == 0 && Printer::currentPositionSteps[1] == 0)
                    {
                        if (Printer::countZSteps < 0)
                            Printer::countZSteps = -Printer::countZSteps;
                        Printer::xMin = 0;
                        Printer::yMin = 0;
                        Printer::zMin = 0;
                        Printer::xLength = Printer::invAxisStepsPerMM[0] * Printer::countZSteps;
                        Printer::yLength = Printer::invAxisStepsPerMM[1] * Printer::countZSteps;
                        Printer::zLength = Printer::invAxisStepsPerMM[2] * Printer::countZSteps;
                        Printer::xMaxSteps = Printer::countZSteps;
                        Printer::yMaxSteps = Printer::countZSteps;
                        Printer::zMaxSteps = Printer::countZSteps;
                        for (uint8_t i=0; i<3; i++)
                        {
                            Printer::currentPositionSteps[i] = 0;
                        }
                        transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
                        Com::printFLN(Com::tMeasureOriginReset);
#if EEPROM_MODE!=0
                        EEPROM::storeDataIntoEEPROM(false);
                        Com::printFLN(Com::tEEPROMUpdated);
#endif
                    }
                    else
                    {
                        Com::printErrorFLN(Com::tMeasurementAbortedOrigin);
                    }
                }
            }
            break;
#endif
#endif
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
    kill(false);
    Extruder::manageTemperatures();
    for(uint8_t i=0; i<NUM_EXTRUDER+3; i++)
        pwm_pos[i] = 0;
#if EXT0_HEATER_PIN>-1
    WRITE(EXT0_HEATER_PIN,0);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    WRITE(EXT1_HEATER_PIN,0);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    WRITE(EXT2_HEATER_PIN,0);
#endif
#if FAN_PIN>-1
    WRITE(FAN_PIN,0);
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
