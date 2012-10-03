/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Reptier.h"
#include "pins_arduino.h"
#include "ui.h"
#include "eeprom.h"

Extruder *current_extruder;
Extruder extruder[NUM_EXTRUDER] = {
 {0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_STEPS_PER_MM,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
   EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,EXT0_WATCHPERIOD
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
   ,EXT0_ADVANCE_K
#endif
   ,EXT0_ADVANCE_L
#endif
  ,{0,EXT0_TEMPSENSOR_TYPE,EXT0_TEMPSENSOR_PIN,0,0,0,0,0,EXT0_HEAT_MANAGER
#ifdef TEMP_PID
  ,0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_INTEGRAL_DRIVE_MIN,EXT0_PID_P,EXT0_PID_I,EXT0_PID_D,EXT0_PID_MAX,0,0,0,{0,0,0,0}
#endif 
  }
 } 
#if NUM_EXTRUDER>1
 ,{1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_STEPS_PER_MM,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
   EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,EXT1_WATCHPERIOD
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
   ,EXT1_ADVANCE_K
#endif
   ,EXT1_ADVANCE_L
#endif
 {1,EXT1_TEMPSENSOR_TYPE,EXT1_TEMPSENSOR_PIN,0,0,0,0,0,EXT1_HEAT_MANAGER
#ifdef TEMP_PID
  ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_P,EXT1_PID_I,EXT1_PID_D,EXT1_PID_MAX,0,0,0,{0,0,0,0}
#endif
 } 
 }
#endif
};

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
TemperatureController heatedBedController = {4,HEATED_BED_SENSOR_TYPE,HEATED_BED_SENSOR_PIN,0,0,0,0,0,HEATED_BED_HEAT_MANAGER
#ifdef TEMP_PID
,0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,0,{0,0,0,0}
#endif
};
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif

TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] = {&extruder[0].tempControl
#if NUM_EXTRUDER>1
,&extruder[1].tempControl
#if NUM_EXTRUDER>2
,&extruder[2].tempControl
#endif
#endif
#if HAVE_HEATED_BED
,&heatedBedController
#endif
};
#ifdef USE_GENERIC_THERMISTORTABLE
short temptable_generic[GENERIC_THERM_NUM_ENTRIES][2];
#endif

//#if HEATED_BED_SENSOR_TYPE!=0
//  int current_bed_raw = 0;
//  int target_bed_raw = 0;
//#endif

byte manage_monitor = 255; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
unsigned int counter_periodical=0;
volatile byte execute_periodical=0;
byte counter_250ms=25;
//byte heated_bed_output=0;
//int target_bed_celsius=0;

#ifdef SUPPORT_MAX6675
extern int read_max6675(byte ss_pin);
#endif

const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
uint8 osAnalogInputCounter[ANALOG_INPUTS];
uint osAnalogInputBuildup[ANALOG_INPUTS];
uint8 osAnalogInputPos=0; // Current sampling position
volatile uint osAnalogInputValues[ANALOG_INPUTS];

void initHeatedBed() {
#if HAVE_HEATED_BED
#ifdef TEMP_PID
    if(heatedBedController.pidIGain) { // prevent division by zero
       heatedBedController.tempIStateLimitMax = (float)heatedBedController.pidDriveMax*10.0f/heatedBedController.pidIGain;
       heatedBedController.tempIStateLimitMin = (float)heatedBedController.pidDriveMin*10.0f/heatedBedController.pidIGain;
    }
#endif
#endif
}
// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- initExtruder ------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** \brief Initalizes all extruder.

Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void initExtruder() {
  byte i;
  current_extruder = &extruder[0];
#ifdef USE_GENERIC_THERMISTORTABLE
#define GENERIC_THERM_T0K (GENERIC_THERM_T0+273.15f)
#if GENERIC_THERM_R1==0
#define GENERIC_VS GENERIC_THERM_VREF
#define GENERIC_RS GENERIC_THERM_R2
#else
#define GENERIC_VS (float)(GENERIC_THERM_VREF*GENERIC_THERM_R1)/(GENERIC_THERM_R1+GENERIC_THERM_R2)
#define GENERIC_RS (float)(GENERIC_THERM_R2*GENERIC_THERM_R1)/(GENERIC_THERM_R1+GENERIC_THERM_R2)
#endif
  float k = GENERIC_THERM_R0*exp(-GENERIC_THERM_BETA/GENERIC_THERM_T0K);
  float delta = 4092.0f/GENERIC_THERM_NUM_ENTRIES;
  for(i=0;i<GENERIC_THERM_NUM_ENTRIES;i++) {
    int adc = (int)(i*delta+1);
    if(adc>4095) adc=4095;
    temptable_generic[i][0] = (adc>>(ANALOG_REDUCE_BITS));
    float v = (float)(adc*GENERIC_THERM_VADC)/(4096.0f);
    float r = GENERIC_RS*v/(GENERIC_VS-v);
    temptable_generic[i][1] = (int)(8.0f*(GENERIC_THERM_BETA/log(r/k)-273.15f+0.5f /* for correct rounding */));
#ifdef DEBUG_GENERIC
    out.print_int_P(PSTR("GenTemp: "),temptable_generic[i][0]); 
    out.println_int_P(PSTR(","),temptable_generic[i][1]); 
#endif
  }
#endif
  SET_OUTPUT(EXT0_DIR_PIN);
  SET_OUTPUT(EXT0_STEP_PIN);
#ifdef EXT1_STEP_PIN
  SET_OUTPUT(EXT1_DIR_PIN);
  SET_OUTPUT(EXT1_STEP_PIN);
#endif
#ifdef EXT2_STEP_PIN
  SET_OUTPUT(EXT2_DIR_PIN);
  SET_OUTPUT(EXT2_STEP_PIN);
#endif
  
  for(i=0;i<NUM_EXTRUDER;++i) {
    Extruder *act = &extruder[i];
    if(act->enablePin > -1) {
      pinMode(act->enablePin,OUTPUT);
      if(!act->enableOn) digitalWrite(act->enablePin,HIGH);
    }
    act->tempControl.lastTemperatureUpdate = millis();
#ifdef SUPPORT_MAX6675
    if(act->sensorType==101) {
      WRITE(SCK_PIN,0);
      SET_OUTPUT(SCK_PIN);
      WRITE(MOSI_PIN,1);
      SET_OUTPUT(MOSI_PIN);
      WRITE(MISO_PIN,1);
      SET_INPUT(MISO_PIN);
      digitalWrite(act->tempControl.sensorPin,1);
      pinMode(act->tempControl.sensorPin,OUTPUT);
    }
#endif
  }
#if HEATED_BED_HEATER_PIN>-1
  SET_OUTPUT(HEATED_BED_HEATER_PIN);
  initHeatedBed();
#endif
  extruder_select(0);
#if ANALOG_INPUTS>0
  ADMUX = ANALOG_REF; // refernce voltage
  for(i=0;i<ANALOG_INPUTS;i++) {
     osAnalogInputCounter[i] = 0;
     osAnalogInputBuildup[i] = 0;
     osAnalogInputValues[i] = 0;
  }
	ADCSRA = _BV(ADEN)|_BV(ADSC)|ANALOG_PRESCALER;
  //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
  while (ADCSRA & _BV(ADSC) ) {} // wait for conversion
  	/* ADCW must be read once, otherwise the next result is wrong. */
  uint dummyADCResult;
  dummyADCResult = ADCW;
  // Enable interrupt driven conversion loop
  byte channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
  if(channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
  else
        ADCSRB &= ~_BV(MUX5);
#endif
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
  ADCSRA |= _BV(ADSC); // start conversion without interrupt!
#endif
  
}
// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- extruder_select ---------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** \brief Select extruder ext_num.

This function changes and initalizes a new extruder. This is also called, after the eeprom values are changed.
*/
void extruder_select(byte ext_num) {
   if(ext_num>=NUM_EXTRUDER)
     ext_num = 0;
   current_extruder->extrudePosition = printer_state.currentPositionSteps[3];
   printer_state.destinationSteps[0] -= current_extruder->xOffset;
   printer_state.destinationSteps[1] -= current_extruder->yOffset;
   current_extruder = &extruder[ext_num];
   printer_state.destinationSteps[0] += current_extruder->xOffset;
   printer_state.destinationSteps[1] += current_extruder->yOffset;
#ifdef SEPERATE_EXTRUDER_POSITIONS
   // Use seperate extruder positions only if beeing told. Slic3r e.g. creates a continuous extruder position increment
   printer_state.currentPositionSteps[3] = current_extruder->extrudePosition;
#endif
   printer_state.destinationSteps[3] = printer_state.currentPositionSteps[3];
   axis_steps_per_unit[3] = current_extruder->stepsPerMM;
   inv_axis_steps_per_unit[3] = 1.0f/axis_steps_per_unit[3];
   max_feedrate[3] = current_extruder->maxFeedrate;
//   max_start_speed_units_per_second[3] = current_extruder->maxStartFeedrate;
   max_acceleration_units_per_sq_second[3] = max_travel_acceleration_units_per_sq_second[3] = current_extruder->maxAcceleration;
   axis_travel_steps_per_sqr_second[3] = axis_steps_per_sqr_second[3] = max_acceleration_units_per_sq_second[3] * axis_steps_per_unit[3];
#if USE_OPS==1 || defined(USE_ADVANCE)
   printer_state.minExtruderSpeed = (byte)floor(F_CPU/(TIMER0_PRESCALE*current_extruder->maxStartFeedrate*current_extruder->stepsPerMM));
   printer_state.maxExtruderSpeed = (byte)floor(F_CPU/(TIMER0_PRESCALE*0.0166666*current_extruder->maxFeedrate*current_extruder->stepsPerMM));
   if(printer_state.maxExtruderSpeed>15) printer_state.maxExtruderSpeed = 15;
   if(printer_state.maxExtruderSpeed>=printer_state.minExtruderSpeed) {
     printer_state.maxExtruderSpeed = printer_state.minExtruderSpeed;
   } else {
     float maxdist = current_extruder->maxFeedrate*current_extruder->maxFeedrate*0.00013888/current_extruder->maxAcceleration;
     maxdist-= current_extruder->maxStartFeedrate*current_extruder->maxStartFeedrate*0.5/current_extruder->maxAcceleration;     
     printer_state.extruderAccelerateDelay = (byte)constrain(ceil(maxdist*current_extruder->stepsPerMM/(printer_state.minExtruderSpeed-printer_state.maxExtruderSpeed)),1,255);
   }
   float fmax=((float)F_CPU/((float)printer_state.maxExtruderSpeed*TIMER0_PRESCALE*axis_steps_per_unit[3]))*60.0; // Limit feedrate to interrupt speed
   if(fmax<max_feedrate[3]) max_feedrate[3] = fmax;
#endif
#ifdef TEMP_PID
    if(current_extruder->tempControl.pidIGain) { // prevent division by zero
       current_extruder->tempControl.tempIStateLimitMax = (float)current_extruder->tempControl.pidDriveMax*10.0f/current_extruder->tempControl.pidIGain;
       current_extruder->tempControl.tempIStateLimitMin = (float)current_extruder->tempControl.pidDriveMin*10.0f/current_extruder->tempControl.pidIGain;
    }
#endif
#if USE_OPS==1
   printer_state.opsRetractSteps = printer_state.opsRetractDistance*current_extruder->stepsPerMM;
   printer_state.opsPushbackSteps = (printer_state.opsRetractDistance+printer_state.opsRetractBackslash)*current_extruder->stepsPerMM;
   if(printer_state.opsMode<=1)
     printer_state.opsMoveAfterSteps = 0;
   else
     printer_state.opsMoveAfterSteps = (int)(-(float)printer_state.opsRetractSteps*(100.0-printer_state.opsMoveAfter)*0.01);
#endif
   queue_move(false,true); // Move head of new extruder to old position using last feedrate
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- extruder_set_temperature ------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

void extruder_set_temperature(float temp_celsius,byte extr) {
#ifdef MAXTEMP
  if(temp_celsius>MAXTEMP) temp_celsius = MAXTEMP;
#endif
  if(temp_celsius<0) temp_celsius=0;
  tempController[extr]->targetTemperature = conv_temp_raw(tempController[extr]->sensorType,temp_celsius);
  tempController[extr]->targetTemperatureC = temp_celsius;
   OUT_P_FX("TargetExtr",extr,0);
   OUT_P_FX_LN(":",temp_celsius,0);
#if USE_OPS==1  
  if(extr==current_extruder->id && temp_celsius<(MIN_EXTRUDER_TEMP)) { // Protect for cold filament
    printer_state.filamentRetracted = false;
    printmoveSeen = 0;
  }
#endif
  bool alloff = true;
  for(byte i=0;i<NUM_EXTRUDER;i++)
    if(tempController[i]->targetTemperatureC>15) alloff = false;
  if(alloff)
    epr_update_usage();
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- extruder_get_temperature ------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

float extruder_get_temperature() {
  return current_extruder->tempControl.currentTemperatureC;
  //return conv_raw_temp(current_extruder->tempControl.sensorType,current_extruder->tempControl.currentTemperature);
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- heated_bed_set_temperature ----------------------------------------
// ------------------------------------------------------------------------------------------------------------------

void heated_bed_set_temperature(float temp_celsius) {
#ifdef HAVE_HEATED_BED
   if(temp_celsius>HEATED_BED_MAX_TEMP) temp_celsius = HEATED_BED_MAX_TEMP;
   if(temp_celsius<0) temp_celsius = 0;
   heatedBedController.targetTemperatureC=temp_celsius;
   heatedBedController.targetTemperature = conv_temp_raw(heatedBedController.sensorType,temp_celsius);
   OUT_P_FX_LN("TargetBed:",heatedBedController.targetTemperatureC,0);
#endif     
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- heated_bed_get_temperature ----------------------------------------
// ------------------------------------------------------------------------------------------------------------------

float heated_bed_get_temperature() {
#ifdef HAVE_HEATED_BED 
   TemperatureController *c = tempController[NUM_TEMPERATURE_LOOPS-1];
   return c->currentTemperatureC;
   //return conv_raw_temp(c->sensorType,c->currentTemperature);
#else
   return -1;
#endif     
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- extruder_disable --------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** \brief Disable stepper motor of current extruder. */
void extruder_disable() {
  if(current_extruder->enablePin > -1) 
    digitalWrite(current_extruder->enablePin,!current_extruder->enableOn); 
}
#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM = {
{0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
{365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
{1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
/* Old table for 100k unknown type 61 values
{23*4,300*8},{25*4,295*8},{27*4,290*8},{28*4,285*8},{31*4,280*8},{33*4,275*8},{35*4,270*8},{38*4,265*8},{41*4,260*8},{44*4,255*8},
{48*4,250*8},{52*4,245*8},{56*4,240*8},{61*4,235*8},{66*4,230*8},{71*4,225*8},{78*4,220*8},{84*4,215*8},{92*4,210*8},{100*4,205*8},
{109*4,200*8},{120*4,195*8},{131*4,190*8},{143*4,185*8},{156*4,180*8},{171*4,175*8},{187*4,170*8},{205*4,165*8},{224*4,160*8},
{245*4,155*8},{268*4,150*8},{293*4,145*8},{320*4,140*8},{348*4,135*8},{379*4,130*8},{411*4,125*8},{445*4,120*8},{480*4,115*8},
{516*4,110*8},{553*4,105*8},{591*4,100*8},{628*4,95*8},{665*4,90*8},{702*4,85*8},{737*4,80*8},{770*4,75*8},{801*4,70*8},{830*4,65*8},
{857*4,60*8},{881*4,55*8},{903*4,50*8},{922*4,45*8},{939*4,40*8},{954*4,35*8},{966*4,30*8},{977*4,25*8},{985*4,20*8},{993*4,15*8},
{999*4,10*8},{1004*4,5*8},{1008*4,0*8} //safety*/
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM = {
   {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
   {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
   {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM = {
  {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
  {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
  {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	};

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM = {
   {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
   {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
   {955*4, -11*8},{1008*4, -35*8}};
#if NUM_TEMPS_USERTHERMISTOR0>0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR1>0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR2>0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif
const short * const temptables[7] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0>0
,(short int *)&temptable_5[0][0]
#else
,0
#endif
#if NUM_TEMPS_USERTHERMISTOR1>0
,(short int *)&temptable_6[0][0]
#else
,0
#endif
#if NUM_TEMPS_USERTHERMISTOR2>0
,(short int *)&temptable_7[0][0]
#else
,0
#endif
};
const byte temptables_num[7] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR0,NUM_TEMPS_USERTHERMISTOR1,NUM_TEMPS_USERTHERMISTOR2};

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- read_raw_temperature ----------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

int read_raw_temperature(byte type,byte pin) {
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 99:
      return (1023<<(2-ANALOG_REDUCE_BITS))-(osAnalogInputValues[pin]>>(ANALOG_REDUCE_BITS)); // Convert to 10 bit result
    case 50: // User defined PTC table
    case 51:
    case 52:
      return (osAnalogInputValues[pin]>>(ANALOG_REDUCE_BITS)); // Convert to 10 bit result    
    case 100: // AD595
      return (osAnalogInputValues[pin]>>(ANALOG_REDUCE_BITS));
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
      return read_max6675(pin);
#endif
  }
  return 4095; // unknown method, return high value to switch heater off for safety
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- conv_raw_temp -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

float conv_raw_temp(byte type,int raw_temp) {
  //OUT_P_I_LN("OC for raw ",raw_temp);
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: 
    case 6:
    case 7:
    {
      type--;
      byte num = pgm_read_byte(&temptables_num[type])<<1;
      byte i=2;
      const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
      short oldraw = pgm_read_word(&temptable[0]);
      short oldtemp = pgm_read_word(&temptable[1]);
      short newraw,newtemp;
      raw_temp = (1023<<(2-ANALOG_REDUCE_BITS))-raw_temp;
      while(i<num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newraw > raw_temp) {
          //OUT_P_I("RC O:",oldtemp);OUT_P_I_LN(" OR:",oldraw);
          //OUT_P_I("RC N:",newtemp);OUT_P_I_LN(" NR:",newraw);
          return TEMP_INT_TO_FLOAT(oldtemp + (float)(raw_temp-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
        }
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return TEMP_INT_TO_FLOAT(newtemp);}
    case 50: // User defined PTC thermistor
    case 51:
    case 52:
    {
      type-=46;
      byte num = pgm_read_byte(&temptables_num[type])<<1;
      byte i=2;
      const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
      short oldraw = pgm_read_word(&temptable[0]);
      short oldtemp = pgm_read_word(&temptable[1]);
      short newraw,newtemp;
      while(i<num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newraw > raw_temp)
          return TEMP_INT_TO_FLOAT(oldtemp + (float)(raw_temp-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return TEMP_INT_TO_FLOAT(newtemp);
    }
    case 100: // AD595
      return (int)((long)raw_temp * 500/(1024<<(2-ANALOG_REDUCE_BITS)));
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
      return raw_temp /4;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE
    case 99: {
      byte i=2;
      const short *temptable = (const short *)temptable_generic; 
      short oldraw = temptable[0];
      short oldtemp = temptable[1];
      short newraw,newtemp;
      raw_temp = (1023<<(2-ANALOG_REDUCE_BITS))-raw_temp;
      while(i<GENERIC_THERM_NUM_ENTRIES*2) {
        newraw = temptable[i++];
        newtemp = temptable[i++];
        if (newraw > raw_temp)
          return TEMP_INT_TO_FLOAT(oldtemp + (float)(raw_temp-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return TEMP_INT_TO_FLOAT(newtemp);
    }
#endif
  }
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- conv_temp_raw -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

int conv_temp_raw(byte type,float tempf) {
  int temp = TEMP_FLOAT_TO_INT(tempf);
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    {
      type--;
      byte num = pgm_read_byte(&temptables_num[type])<<1;
      byte i=2;
      const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
      short oldraw = pgm_read_word(&temptable[0]);
      short oldtemp = pgm_read_word(&temptable[1]);
      short newraw,newtemp;
      while(i<num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newtemp < temp)
          return (1023<<(2-ANALOG_REDUCE_BITS))- oldraw + (long)(oldtemp-temp)*(long)(oldraw-newraw)/(oldtemp-newtemp);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
    }
    case 50: // user defined PTC thermistor
    case 51:
    case 52:
    {
      type-=46;
      byte num = pgm_read_byte(&temptables_num[type])<<1;
      byte i=2;
      const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
      short oldraw = pgm_read_word(&temptable[0]);
      short oldtemp = pgm_read_word(&temptable[1]);
      short newraw,newtemp;
      while(i<num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newtemp > temp)
          return oldraw + (long)(oldtemp-temp)*(long)(oldraw-newraw)/(oldtemp-newtemp);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return newraw;
    }
    case 100: // HEATER_USES_AD595
      return (int)((long)temp * (1024<<(2-ANALOG_REDUCE_BITS))/ 500);
#ifdef SUPPORT_MAX6675
    case 101:  // defined HEATER_USES_MAX6675
      return temp * 4;
#endif
#ifdef USE_GENERIC_THERMISTORTABLE
    case 99: {
      byte i=2;
      const short *temptable = (const short *)temptable_generic;
      short oldraw = temptable[0];
      short oldtemp = temptable[1];
      short newraw,newtemp;
      while(i<GENERIC_THERM_NUM_ENTRIES*2) {
        newraw = temptable[i++];
        newtemp = temptable[i++];
        if (newtemp < temp)
          return (1023<<(2-ANALOG_REDUCE_BITS))- oldraw + (long)(oldtemp-temp)*(long)(oldraw-newraw)/(oldtemp-newtemp);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
    }
#endif
  }
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- disableAllHeater --------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
byte autotuneIndex = 255;
void disableAllHeater() {
  for(byte i=0;i<NUM_TEMPERATURE_LOOPS;i++) {
     TemperatureController *c = tempController[i];
     c->targetTemperature = 0;
     c->targetTemperatureC = 0;
     pwm_pos[c->pwmIndex] = 0;
  }
  autotuneIndex = 255;
}
// ------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ autotunePID -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

#ifdef TEMP_PID
void autotunePID(float temp,int controllerId)
{
  TemperatureController *c = tempController[controllerId];
  float currentTemp;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high;
  long t_low;

  long bias=c->pidMax>>1;
  long d = c->pidMax>>1;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float maxTemp=20, minTemp=20;
  
  OUT_P_LN("PID Autotune start");
  
  disableAllHeater(); // switch off all heaters.
  autotuneIndex = controllerId;
  pwm_pos[c->pwmIndex] = c->pidMax;
    
  for(;;) {
      c->currentTemperature = read_raw_temperature(c->sensorType,c->sensorPin);
      currentTemp = c->currentTemperatureC = conv_raw_temp(c->sensorType,c->currentTemperature);
      unsigned long time = millis();
      maxTemp=max(maxTemp,currentTemp);
      minTemp=min(minTemp,currentTemp);
      if(heating == true && currentTemp > temp) { // switch heating -> off
        if(time - t2 > 2500) {
          heating=false;
          pwm_pos[c->pwmIndex] = (bias - d);
          t1=time;
          t_high=t1 - t2;
          maxTemp=temp;
        }
      }
      if(heating == false && currentTemp < temp) {
        if(time - t1 > 5000) {
          heating=true;
          t2=time;
          t_low=t2 - t1; // half wave length
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,c->pidMax-20);
            if(bias > c->pidMax/2) d = c->pidMax - 1 - bias;
            else d = bias;

            OUT_P_I(" bias: ",bias);
            OUT_P_I(" d: ",d);
            OUT_P_F(" min: ",minTemp);
            OUT_P_F_LN(" max: ",maxTemp);
            if(cycles > 2) {
              // Parameter according Ziegler–Nichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
              Ku = (4.0*d)/(3.14159*(maxTemp-minTemp));
              Tu = ((float)(t_low + t_high)/1000.0);
              OUT_P_F(" Ku: ",Ku);
              OUT_P_F_LN(" Tu: ",Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu*0.125;
              OUT_P_LN(" Classic PID");
              OUT_P_F_LN(" Kp: ",Kp);
              OUT_P_F_LN(" Ki: ",Ki);
              OUT_P_F_LN(" Kd: ",Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              OUT_P_LN(" Some overshoot");
              OUT_P_F_LN(" Kp: ",Kp);
              OUT_P_F_LN(" Ki: ",Ki);
              OUT_P_F_LN(" Kd: ",Kd);
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              OUT_P_LN(" No overshoot");
              OUT_P_F_LN(" Kp: ",Kp);
              OUT_P_F_LN(" Ki: ",Ki);
              OUT_P_F_LN(" Kd: ",Kd);
              */
            }
          }
          pwm_pos[c->pwmIndex] = (bias + d);
          cycles++;
          minTemp=temp;
        }
    }
    if(currentTemp > (temp + 20)) {
      OUT_P_LN("PID Autotune failed! Temperature to high");
      disableAllHeater();
      return;
    }
    if(time - temp_millis > 1000) {
      temp_millis = time;
      print_temperatures();
    }
    if(((time - t1) + (time - t2)) > (10L*60L*1000L*2L)) {
      OUT_P_LN("PID Autotune failed! timeout");
      disableAllHeater();
      return;
    }
    if(cycles > 5) {
      OUT_P_LN("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h");
      disableAllHeater();
      return;
    }
    UI_MEDIUM;
    UI_SLOW;
  }
}
#endif

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- write_monitor -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** \brief Writes monitored temperatures.

This function is called every 250ms to write the monitored temperature. If monitoring is
disabled, the function is not called.
*/
void write_monitor() {
    out.print_long_P(PSTR("MTEMP:"),millis());
    TemperatureController *act = tempController[manage_monitor];
    OUT_P_F(" ",act->currentTemperatureC); 
    OUT_P_FX(" ",act->targetTemperatureC,0);
    OUT_P_I_LN(" ",pwm_pos[act->pwmIndex]);
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- manage temperatures -----------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** Makes updates to temperatures and heater state every call. 

Is called every 100ms.
*/

void manage_temperatures() {
  for(byte controller=0;controller<NUM_TEMPERATURE_LOOPS;controller++) {
    if(controller == autotuneIndex) continue;
    TemperatureController *act = tempController[controller];
    // Get Temperature
    //int oldTemp = act->currentTemperatureC;
    act->currentTemperature = read_raw_temperature(act->sensorType,act->sensorPin);
    act->currentTemperatureC = conv_raw_temp(act->sensorType,act->currentTemperature);
    byte on = act->currentTemperature>=act->targetTemperature ? LOW : HIGH;
#ifdef TEMP_PID
    act->tempArray[act->tempPointer++] = act->currentTemperatureC;
    act->tempPointer &= 3;
    if(act->heatManager==1) {
         byte output;
         float error = act->targetTemperatureC - act->currentTemperatureC;
         if(act->targetTemperatureC<20.0f) output = 0; // off is off, even if damping term wants a heat peak!
         else if(error>PID_CONTROL_RANGE) 
           output = act->pidMax;
         else if(error<-PID_CONTROL_RANGE)
           output = 0;
         else {
           float pidTerm = act->pidPGain * error;
           act->tempIState = constrain(act->tempIState+error,act->tempIStateLimitMin,act->tempIStateLimitMax);
           pidTerm += act->pidIGain * act->tempIState*0.1; 
           long dgain = act->pidDGain * (act->tempArray[act->tempPointer]-act->currentTemperatureC)*3.333f;
           pidTerm += dgain;
#if SCALE_PID_TO_MAX==1
           pidTerm = (pidTerm*act->pidMax)*0.0039062;
#endif
           output = constrain((int)pidTerm, 0, act->pidMax);
         }
         pwm_pos[act->pwmIndex] = output;
    } else
#endif
    if(act->heatManager == 2) { // Bang-bang with reduced change frequency to save relais life
        unsigned long time = millis();
        if (time - act->lastTemperatureUpdate > HEATED_BED_SET_INTERVAL) {
          pwm_pos[act->pwmIndex] = (on ? 255 : 0);
          act->lastTemperatureUpdate = time;
        }
    } else { // Fast Bang-Bang fallback
         pwm_pos[act->pwmIndex] = (on ? 255 : 0);
    }
#if LED_PIN>-1
    if(act == &current_extruder->tempControl)
        WRITE(LED_PIN,on);
#endif       
  }
}
// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- read_max6675 ------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

#ifdef SUPPORT_MAX6675
int read_max6675(byte ss_pin)
{
  int max6675_temp = 0;    
  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif  
  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
  digitalWrite(ss_pin, 0);  // enable TT_MAX6675
  delay(1);    // ensure 100ns delay - a bit extra is fine
  SPDR = 0;   // read MSB
  while ((SPSR & (1<<SPIF)) == 0);
  max6675_temp = SPDR;
  max6675_temp <<= 8;  
  SPDR = 0; // read LSB
  while ((SPSR & (1<<SPIF)) == 0);
  max6675_temp |= SPDR;
  digitalWrite(ss_pin, 1);  // disable TT_MAX6675
  return max6675_temp & 4 ? 2000 : max6675_temp >> 3; // thermocouple open?
}
#endif

