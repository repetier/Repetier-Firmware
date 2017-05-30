/*
    This file is additional to Repetier-Firmware.

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

    author of this additional File : RAyWB / Robert Ayrenschmalz

*/
#ifndef CustomEventsImpl_H
#define CustomEventsImpl_H



MCP23017 MCP1(0x24);  
PCAPWM   EXTPWM1(0x41);
MCP4725  DAC1(0x60);

// if you are not shure about the I2C Adresss of your components run Nick Gammon´s I2C Scanner
// see:  http://www.gammon.com.au/i2c



//#########################################################################################
//### GAMMA Variables
//### necessary to correct grayscale , mostly depending on material you want to engrave
//### for my setup these values work
//### i use a PLTB450B 1.6W Laser diode with analog modulation
//#########################################################################################

uint16_t Outval;
bool Gamma_on=true;
float GAMMA=1.4;
uint16_t LASER_BASE_OFFSET=1000;
uint16_t LASER_LIMIT=4095;
float newintens;


//#########################################################################################
//##### Initialization, Setup of I2C and Components
//#########################################################################################

void Custom_Init_Early()
{ 

#ifndef COMPILE_I2C_DRIVER
HAL::i2cInit(400000);  // usually 100000 Hz  , my setup works on 400000Hz
#endif  

#if defined( EPWM_OE) && EPWM_OE > -1  //Output Enable should be used to get defined Power Up Status
  SET_OUTPUT( EPWM_OE );
  WRITE( EPWM_OE, HIGH);
#endif  

#if defined( FEATURE_I2C_MACROS) && FEATURE_I2C_MACROS !=0
  MCP1.Init();
#endif 
 
#if defined( FEATURE_EXT_PWM) && FEATURE_EXT_PWM !=0
  EXTPWM1.Init();
  EXTPWM1.SetFREQ(1100);//PWM frequency PCA9685 referring Datasheet adjustable from 24 to 1526 Hz
                        //never tried over 1200  !!!!! all Pins use same PWM frequency
#endif 

#if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
  DAC1.Init();
#endif
 
#if defined( EPWM_OE) && EPWM_OE > -1
  WRITE( EPWM_OE, LOW);// output enable after power up otherwise undefined status
#endif
}

//#########################################################################################
//##### Initialization, Set DAC to defined Status
//##### just to show, of coarse DAC can be written in Custom_Init_Early also
//#########################################################################################

void Custom_Init()
{ 
 #if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
   DAC1.WriteOutput(0);
 #endif 
}

//#########################################################################################
//##### I2C Portexpander MCP23017
//##### Logic mostly from Github/kasperskaarhoj 
//##### modified to use HAL Routines by Robert Ayrenschmalz alias RAyWB
//#########################################################################################

MCP23017::MCP23017(uint8_t address)
{
  i2c_add=address<<1;
}

void MCP23017::Init(void)
{
	_GPIO = 0x0000;
 	_IODIR = 0xFFFF; //set all to inputs
	_GPPU = 0x0000;
	
   HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(0x05);
   HAL::i2cWrite(0x00);
   HAL::i2cStop(); 
	 writeRegister(MCP23017_IODIR, _IODIR);
	
}

void MCP23017::SetInput(uint8_t pin) {

   _IODIR |= 1 << pin;
	 writeRegister(MCP23017_IODIR, _IODIR);
}


void MCP23017::SetOutput(uint8_t pin) {
	
	 _IODIR &= ~(1 << pin);
	 writeRegister(MCP23017_IODIR, _IODIR);
}


bool MCP23017::Read(uint8_t pin) {
	 _GPIO = readRegister(MCP23017_GPIO);
	 if ( _GPIO & (1 << pin)) return HIGH;
	 else return LOW;
}

void MCP23017::Write(uint8_t pin, bool val) {
	//If this pin is an INPUT pin, a write here will
	//enable the internal pullup
	//otherwise, it will set the OUTPUT voltage
	//as appropriate.
	 bool isOutput = !(_IODIR & 1<<pin);

	  if (isOutput) {
		 //This is an output pin so just write the value
		  if (val) _GPIO |= 1 << pin;
		  else _GPIO &= ~(1 << pin);
		 writeRegister(MCP23017_GPIO, _GPIO);
	   }
	  else {
		 //This is an input pin, so we need to enable the pullup
		  if (val) _GPPU |= 1 << pin;
		  else _GPPU &= ~(1 << pin);
		 writeRegister(MCP23017_GPPU, _GPPU);
	   }
}

uint16_t MCP23017::ReadPort() {
 InterruptProtectedBlock noInts;
 noInts.protect();
	 _GPIO = readRegister(MCP23017_GPIO);
	 return _GPIO;
 noInts.unprotect(); 
}
void MCP23017::WritePort(uint16_t data) {
	 _GPIO = data;
	 writeRegister(MCP23017_GPIO, _GPIO);
}

//PRIVATE

void MCP23017::writeRegister(uint16_t regAddress, uint16_t data) {
	
	 HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(regAddress);
   HAL::i2cWrite(data>>8);
   HAL::i2cWrite(data);
   HAL::i2cStop(); 
}

uint16_t MCP23017::readRegister(uint16_t regAddress) {
	
	uint16_t data = 0x00;
	 
	 HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(regAddress);
	 HAL::i2cStop();
	 HAL::i2cStartWait(i2c_add + I2C_READ);
   data = (HAL::i2cReadAck()<<8);
   data += HAL::i2cReadNak();
   HAL::i2cStop();
   return data ;
} 

// End MCP23017

//#########################################################################################
//##### I2C DAC MCP4725
//#########################################################################################

#define MCP4725_COMMAND 0x40  // writes only Output,not EEprom

MCP4725::MCP4725(uint8_t address)
{
  i2c_add = address<<1;
}

void MCP4725::Init(void)
{
  HAL::i2cStartWait((i2c_add)+ I2C_WRITE);
  HAL::i2cWrite(MCP4725_COMMAND);  
  HAL::i2cStop();
}

void MCP4725::WriteOutput(uint16_t value)
{
  HAL::i2cStartWait((i2c_add)+ I2C_WRITE);
  HAL::i2cWrite(MCP4725_COMMAND);  
  HAL::i2cWrite(value>>4); // 12 Bit, write Bit11..Bit4
  HAL::i2cWrite(value<<4); // write Bit3..0 , followed by 4*0
  HAL::i2cStop();
}
 
//end MCP4725

//#########################################################################################
//### external PWM  PCA9685   16 channels
//#########################################################################################


PCAPWM::PCAPWM(uint8_t address)
{
  PCA_addr = address<<1;
}

void PCAPWM::Init(void)
{ 
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(0x0);
  HAL::i2cStop();
}

void PCAPWM::SetFREQ(float freq) 
{
  freq *= 0.99; 
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  HAL::i2cStartWait(PCA_addr + I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr + I2C_READ);
  uint8_t oldmode = HAL::i2cReadNak();
  HAL::i2cStop();
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  HAL::i2cStartWait(PCA_addr + I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(newmode);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0xFE);
  HAL::i2cWrite(prescale);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(oldmode);
  HAL::i2cStop();
  HAL::delayMilliseconds(50);
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(oldmode | 0xa1);
  HAL::i2cStop();
}

void PCAPWM::SetPWM(uint8_t num, uint16_t on, uint16_t off)
{
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x6+4*num);
  HAL::i2cWrite(on);
  HAL::i2cWrite(on>>8);
  HAL::i2cWrite(off);
  HAL::i2cWrite(off>>8);
  HAL::i2cStop();
}

void PCAPWM::SetPIN(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  if( val >4095) val = 4095;//min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      SetPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      SetPWM(num, 0, 4096);
    }
    else {
      SetPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      SetPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      SetPWM(num, 0, 4096);
    }
    else {
      SetPWM(num, 0, val);
    }
  }
}
 

//End PCA9685 


//#########################################################################################
//#### Laser Driver replacement
//#########################################################################################

bool SetLaser(uint16_t newIntensity)
 {
  
#if defined( FEATURE_PWM_LASER) && FEATURE_PWM_LASER !=0
  EXTPWM1.SetPIN(EXT_LASER_PIN,newIntensity,LASER_ON_HIGH);
#endif

#if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
  DAC1.WriteOutput(newIntensity);
#endif  

 
  return false;
 }


//#########################################################################################
//#### Timer 100ms
//#########################################################################################

void Custom_100MS()
 {
   
 }

//#########################################################################################
//#### Timer 500ms
//#########################################################################################

void Custom_500MS()
 {
  
 }

//#########################################################################################
//#### GCode addition/replacement
//#########################################################################################

 bool Custom_GCode(GCode *com) 
 {
  char buf[35];
  volatile float BackupFeedrate; //backup variable for G1 Feedrate
  uint32_t codenum; //throw away variable

//#########################################################################################
/* modified G0/G1 to use max.possible feedrate for G0 moves
   as most of CAM programs do not add feedrate when generating Gcode
   i use maximum feedrate from fastest axis for G0 moves as for slower axis
   feedrate is limited by Firmware */
//#########################################################################################
  
    switch(com->G) {
        case 0: // G0 -> G1
        case 1: // G1
        
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            {
                // disable laser for G0 moves
                bool laserOn = LaserDriver::laserOn;
                if(Printer::mode == PRINTER_MODE_LASER) {
                    if(com->G == 0) {
                        LaserDriver::laserOn = false;
                        LaserDriver::firstMove = true; //set G1 flag for Laser
                        Com::print("firstmove_true");//for debug
                    } else {
#if LASER_WARMUP_TIME > 0                        
                        uint16_t power = (com->hasX() || com->hasY()) && (LaserDriver::laserOn || com->hasE()) ? LaserDriver::intensity : 0;
                        if(power > 0 && LaserDriver::firstMove) {
                            PrintLine::waitForXFreeLines(1,true);
                            PrintLine::LaserWarmUp(LASER_WARMUP_TIME);
                            LaserDriver::firstMove = false;
                        }
#endif                        
                    }
                }                
#endif // defined Laser

                if(com->hasS()) Printer::setNoDestinationCheck(com->S != 0);
                if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if NONLINEAR_SYSTEM
                    if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, true)) {
                        Com::printWarningFLN(PSTR("executeGCode / queueDeltaMove returns error"));
                    }
#else
                    if(com->G == 0) { 
                    BackupFeedrate = Printer::feedrate; //backup feedrate
				           	// select faster axis max feed rate for g0,other axis speed is limited by maxFeedrate
                    if( Printer::maxFeedrate[X_AXIS]>=Printer::maxFeedrate[Y_AXIS])
                    Printer::feedrate = Printer::maxFeedrate[X_AXIS];//use faster axis max Feedrate for G0
                    else Printer::feedrate = Printer::maxFeedrate[Y_AXIS];

                    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
                    Printer::feedrate = BackupFeedrate;//restore Feedrate for G1
                    }//end go
                    else//g1
                    {
                    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
                    }//end g1
#endif
#if UI_HAS_KEYS
                // ui can only execute motion commands if we are not waiting inside a move for an
                // old move to finish. For normal response times, we always leave one free after
                // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
                // gets filled while waiting, the lost is neglectable.
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

#if defined(SUPPORT_LASER) && SUPPORT_LASER
                LaserDriver::laserOn = laserOn;
            }
#endif // defined
break;
  
  default:
     return false;
  }
  return true;
}

//#########################################################################################
//#### MCode addition/replacement
//#########################################################################################


bool Custom_MCode(GCode *com)
{
  
  char buf[20];
  char buf2[20]; 
  uint16_t PIN,Speed,angle ;
  
  switch(com->M) {

              case 3: // Spindle/laser on
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            if(Printer::mode == PRINTER_MODE_LASER) {
                if(com->hasS())
                
#if defined(GAMMA_CORRECTION) && (GAMMA_CORRECTION==true)
                      newint =(com->S);
                      if (Gamma_on)
                      {
                      Outval=pow(newintens/LASER_PWM_MAX,GAMMA)*LASER_PWM_MAX; //Gamma function
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale gamma function and offset to max 
                      Com::printFLN(PSTR("orig:"),(int)newintens);
                      }
                      else
                      {
                      Outval=newintens;
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale offset to max 
                      Com::printFLN(PSTR("Gamma off LV:"),(int)newintens);
                      }
                     
#else
                LaserDriver::intensity = constrain(com->S,0,LASER_PWM_MAX);
#endif                
                LaserDriver::laserOn = true;
                Com::printFLN(PSTR("LaserOn:"),(int)LaserDriver::intensity);
               
            }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
            if(Printer::mode == PRINTER_MODE_CNC) {
                Commands::waitUntilEndOfAllMoves();
                CNCDriver::spindleOnCW(com->hasS() ? com->S : CNC_RPM_MAX);
            }
#endif // defined
            break;
 
//########################################################################################################
// Standard M452 just switches to Laser mode
// Here added for Gamma correction and base offset to start at where material begins to change colour
// and Limit value .
// that makes a simple grayscale calibration possible.
// 
// Letters used in M452:
//                       C (Correction)<0/1> 0=off 1 =on
//                       K (K-factor) usually between 0.1 and 2.2
//                       P (base Power) depends on Laser Power and material
//                       L (Limit)  depends on Laser Power and material
//
// example for 12 bit PWM (LASER_PWM MAX 4095): Gamma on , Gamma 1.4 , start value 500 Limit 3000 
// Gcode command for this example is  : M452 C1 K1.4 P500 L3000
//########################################################################################################

       case 452:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            Commands::waitUntilEndOfAllMoves();
            Printer::mode = PRINTER_MODE_LASER;
            
            if(com->hasC())
            {if((com->C)==0) 
             {Gamma_on=false;
              Com::printFLN(PSTR("GAMMA OFF:"));
             }
             else 
             {Gamma_on=true;
              Com::printFLN(PSTR("GAMMA ON"));
             }
            }
            if(com->hasK())
            {
              GAMMA=(float)(com->K);
              Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1);
            }
            if(com->hasP())
            {
              LASER_BASE_OFFSET=(int)(com->P);
              Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
            }

             if(com->hasL())
            {
              LASER_LIMIT=(int)(com->L);
              Com::printFLN(PSTR("LASER_LIMIT:"),(int)LASER_LIMIT); 
            }
      
            
#endif
            Printer::reportPrinterMode();
            break;            
       

                  
  default:
     return false;
  }
  return true;
}

//#########################################################################################
//###  User defined Events
//#########################################################################################

int Custom_Execute(int action,bool allowMoves) 
{
            
  switch(action) {
  
        case UI_ACTION_X_UP001:
        case UI_ACTION_X_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP001) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]/100), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
 
        case UI_ACTION_X_UP01:
        case UI_ACTION_X_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP01) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]/10), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;    
      
        case UI_ACTION_X_UP1:
        case UI_ACTION_X_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[X_AXIS], 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
       
        case UI_ACTION_X_UP10:
        case UI_ACTION_X_DOWN10:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP10) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]*10), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
              
        case UI_ACTION_Y_UP001:
        case UI_ACTION_Y_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP001) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]/100, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP01:
        case UI_ACTION_Y_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP01) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]/10, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP1:
        case UI_ACTION_Y_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS], 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP10:
        case UI_ACTION_Y_DOWN10:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP10) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]*10, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
                     
        case UI_ACTION_Z_UP001:
        case UI_ACTION_Z_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP001) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS]/100, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual")); 
            break;
            
        case UI_ACTION_Z_UP01:
        case UI_ACTION_Z_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP01) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS]/10, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual"));           
            break;
            
        case UI_ACTION_Z_UP1:
        case UI_ACTION_Z_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS], 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual"));           
            break;
     
        case UI_ACTION_X_ZERO:
            if(!allowMoves) return UI_ACTION_X_ZERO;
            Printer::coordinateOffset[X_AXIS] -= Printer::currentPosition[X_AXIS];
            break;
            
        case UI_ACTION_Y_ZERO:
            if(!allowMoves) return UI_ACTION_Y_ZERO;
            Printer::coordinateOffset[Y_AXIS] -= Printer::currentPosition[Y_AXIS];
            break;
            
        case UI_ACTION_Z_ZERO:
            if(!allowMoves) return UI_ACTION_Z_ZERO;
            Printer::coordinateOffset[Z_AXIS] -= Printer::currentPosition[Z_AXIS];
            break;
        }
return 0 ;
}//Custom_Execute


//#########################################################################################
//#### Read buttons from MCP1 external I2C device  
//#### as we read the complete bitmask it´s possible to realize two or more buttons pressed
//#### at the same time in order to realize "shift" Functions etc.
//#########################################################################################

int Custom_CheckSlowKeys()
{
  int action=0;
  char buf[20];
  
#if defined( FEATURE_I2C_MACROS) && FEATURE_I2C_MACROS !=0
{
	 uint16_t buttonval = 0xFFFF-MCP1.ReadPort();
	    
switch (buttonval) {
  
      case 1:       RunMacro(HomeAll);
                    break;
            
      case 2:
                    GCode::executeFString(PSTR("M117 moveZ+ manual")); 
                    HAL::delayMilliseconds(100);
                    GCode::executeFString(PSTR("G91\n G1 Z0.001 F20 \n G90\n")); 
                    break;
   
      case 3:       action = UI_ACTION_Z_UP1 ;
                    break;
               
      case 4:       GCode::executeFString(PSTR("M117 set FFF Mode")); 
                    GCode::executeFString(PSTR("M451")); 
                    HAL::delayMilliseconds(200);
                    GCode::executeFString(PSTR("M117")); 
                    break;        
     
      case 5:       action = UI_ACTION_Z_DOWN01 ;
                    break;  
                  
      case 6:       action = UI_ACTION_Z_DOWN1 ;
                    break;        
      
      case 7:       action = UI_ACTION_Y_UP001 ;
                    break;
            
      case 8:       GCode::executeFString(PSTR("M117 set LASER Mode")); 
                    GCode::executeFString(PSTR("M452")); 
                    HAL::delayMilliseconds(200);
                    GCode::executeFString(PSTR("M117")); 
                    break;
           
      case 9:       action = UI_ACTION_Y_UP1 ;
                    break;

      case 10:
                    action = UI_ACTION_Y_UP1 ;
                    break;
     
      
      case 11:      action = UI_ACTION_Y_DOWN001 ;
                    break;        
      
      case 12:      action = UI_ACTION_Y_DOWN01 ;
                    break;  
                  
      case 13:      action = UI_ACTION_Y_DOWN1 ;
                    break;        
    
      case 14:      action = UI_ACTION_Y_DOWN10 ;
                    break;

      case 15:      action = UI_ACTION_Y_DOWN10 ;
                    break;
            
      case 16:      GCode::executeFString(PSTR("M117 set CNC Mode")); 
                    GCode::executeFString(PSTR("M453")); 
                    HAL::delayMilliseconds(200);
                    break;

      case 32:      if(Gamma_on==false)
                    {
                    Gamma_on=true;
                    GCode::executeFString(PSTR("M117 Gamma on")); 
                    Com::print("Gamma on");
                    HAL::delayMilliseconds(300);
                    }
                    else
                    {
                      Gamma_on=false;
                      GCode::executeFString(PSTR("M117 Gamma off")); 
                      Com::print("Gamma off");
                      HAL::delayMilliseconds(300);
                    } 
                    break;

      case 64:      LASER_BASE_OFFSET+=5; 
                    HAL::delayMilliseconds(200);  
                    Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
                    break;                    
                         
      case 128:     LASER_BASE_OFFSET-=5;  
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
                   
                    break;                   
      case 256:     GAMMA+=0.10;
                    HAL::delayMilliseconds(200);   
                    Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1);
                    break;                    
                         
      case 512:     GAMMA-=0.10; 
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1); 
                    break;                   
      
      case 1024:    LASER_LIMIT-=50;
                    if(LASER_LIMIT<LASER_PWM_MAX/4) LASER_LIMIT=LASER_PWM_MAX/4;
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LIMIT:"),(int)LASER_LIMIT); 
                    break;

      case 2048:    LASER_LIMIT+=50;
                    if(LASER_LIMIT>LASER_PWM_MAX) LASER_LIMIT=LASER_PWM_MAX;
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LIMIT:"),(int)LASER_LIMIT); 
                    break;
     
      default: 
                    break;
   }

   return(action);
 }
#endif //macros

}//SLOWKEYS



#endif
