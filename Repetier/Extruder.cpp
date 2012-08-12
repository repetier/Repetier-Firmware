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

#include "Configuration.h"
#include "Reptier.h"
#include "pins_arduino.h"

Extruder *current_extruder;
Extruder extruder[NUM_EXTRUDER] = {
 {0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_STEPS_PER_MM,EXT0_TEMPSENSOR_TYPE,EXT0_TEMPSENSOR_PIN,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
   EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,0,0,0,0,0,EXT0_HEAT_MANAGER,EXT0_WATCHPERIOD
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
   ,EXT0_ADVANCE_K
#endif
   ,EXT0_ADVANCE_L
#endif
#ifdef TEMP_PID
  ,0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_INTEGRAL_DRIVE_MIN,EXT0_PID_PGAIN,EXT0_PID_IGAIN,EXT0_PID_DGAIN,EXT0_PID_MAX,0,0,0,0,0,0,0,0,0,0,0
#endif 
 } 
#if NUM_EXTRUDER>1
 ,{1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_STEPS_PER_MM,EXT1_TEMPSENSOR_TYPE,EXT1_TEMPSENSOR_PIN,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
   EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,0,0,0,0,0,EXT1_HEAT_MANAGER,EXT1_WATCHPERIOD
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
   ,EXT1_ADVANCE_K
#endif
   ,EXT1_ADVANCE_L
#endif
#ifdef TEMP_PID
  ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_PGAIN,EXT1_PID_IGAIN,EXT1_PID_DGAIN,EXT1_PID_MAX,0,0,0,0,0,0,0,0,0,0,0
#endif
 } 
#endif
};

#ifdef USE_GENERIC_THERMISTORTABLE
short temptable_generic[GENERIC_THERM_NUM_ENTRIES][2];
#endif

#if HEATED_BED_SENSOR_TYPE!=0
  int current_bed_raw = 0;
  int target_bed_raw = 0;
#endif

byte manage_monitor = 255; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
byte counter_periodical=0;
volatile byte execute_periodical=0;
byte counter_250ms=25;
byte heated_bed_output=0;
int target_bed_celsius=0;
#if HEATED_BED_HEATER_PIN > -1
unsigned long last_bed_set = 0;       ///< Time of last temperature setting for heated bed. So we can limit settings to desired frequency.
#endif

#ifdef SUPPORT_MAX6675
extern int read_max6675(byte ss_pin);
#endif

const uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
uint8 osAnalogInputCounter[ANALOG_INPUTS];
uint osAnalogInputBuildup[ANALOG_INPUTS];
uint8 osAnalogInputPos=0; // Current sampling position
volatile uint osAnalogInputValues[ANALOG_INPUTS];

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
    act->lastTemperatureUpdate = millis();
#ifdef SUPPORT_MAX6675
    if(act->sensorType==101) {
      WRITE(SCK_PIN,0);
      SET_OUTPUT(SCK_PIN);
      WRITE(MOSI_PIN,1);
      SET_OUTPUT(MOSI_PIN);
      WRITE(MISO_PIN,1);
      SET_INPUT(MISO_PIN);
      digitalWrite(act->sensorPin,1);
      pinMode(act->sensorPin,OUTPUT);
    }
#endif
  }
#if HEATED_BED_HEATER_PIN>-1
  pinMode(HEATED_BED_HEATER_PIN,OUTPUT);
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
    if(current_extruder->pidIGain) { // prevent division by zero
       current_extruder->tempIStateLimitMax = (long)current_extruder->pidDriveMax*1000L/current_extruder->pidIGain;
       current_extruder->tempIStateLimitMin = (long)current_extruder->pidDriveMin*1000L/current_extruder->pidIGain;
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

void extruder_set_temperature(int temp_celsius,byte extr) {
#ifdef MAXTEMP
  if(temp_celsius>(MAXTEMP<<CELSIUS_EXTRA_BITS)) temp_celsius = (MAXTEMP<<CELSIUS_EXTRA_BITS);
#endif
  if(temp_celsius<0) temp_celsius=0;
//#ifdef MINTEMP
//  if(temp_celsius<(MINTEMP<<CELSIUS_EXTRA_BITS)) temp_celsius = (MINTEMP<<CELSIUS_EXTRA_BITS);
//#endif
  extruder[extr].targetTemperature = conv_temp_raw(extruder[extr].sensorType,temp_celsius);
  extruder[extr].targetTemperatureC = temp_celsius;
   out.print_int_P(PSTR("TargetExtr"),(int)extr);
   out.println_int_P(PSTR(":"),temp_celsius>>CELSIUS_EXTRA_BITS);
#if USE_OPS==1  
  if(extr==current_extruder->id && temp_celsius<(MIN_EXTRUDER_TEMP<<CELSIUS_EXTRA_BITS)) { // Protect for cold filament
    printer_state.filamentRetracted = false;
    printmoveSeen = 0;
  }
#endif
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- extruder_get_temperature ------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

int extruder_get_temperature() {
  return conv_raw_temp(current_extruder->sensorType,current_extruder->currentTemperature);
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- heated_bed_set_temperature ----------------------------------------
// ------------------------------------------------------------------------------------------------------------------

void heated_bed_set_temperature(int temp_celsius) {
#if HEATED_BED_SENSOR_TYPE!=0  
   if(temp_celsius>(150<<CELSIUS_EXTRA_BITS)) temp_celsius = 150<<CELSIUS_EXTRA_BITS;
   target_bed_celsius=temp_celsius;
   target_bed_raw = conv_temp_raw(HEATED_BED_SENSOR_TYPE,temp_celsius);
   out.println_int_P(PSTR("TargetBed:"),target_bed_celsius>>CELSIUS_EXTRA_BITS);
#endif     
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- heated_bed_get_temperature ----------------------------------------
// ------------------------------------------------------------------------------------------------------------------

int heated_bed_get_temperature() {
#if HEATED_BED_SENSOR_TYPE!=0  
   return conv_raw_temp(HEATED_BED_SENSOR_TYPE,current_bed_raw);
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

int conv_raw_temp(byte type,int raw_temp) {
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
        if (newraw > raw_temp)
          return oldtemp + (long)(raw_temp-oldraw)*(long)(newtemp-oldtemp)/(newraw-oldraw);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return newtemp;}
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
          return oldtemp + (long)(raw_temp-oldraw)*(long)(newtemp-oldtemp)/(newraw-oldraw);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return newtemp;
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
          return oldtemp + (long)(raw_temp-oldraw)*(long)(newtemp-oldtemp)/(newraw-oldraw);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return newtemp;
    }
#endif
  }
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- conv_temp_raw -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

int conv_temp_raw(byte type,int temp) {
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
// ---------------------------------------------- write_monitor -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** \brief Writes monitored temperatures.

This function is called every 250ms to write the monitored temperature. If monitoring is
disabled, the function is not called.
*/
void write_monitor() {
    out.print_long_P(PSTR("MTEMP:"),millis());
    if(manage_monitor<NUM_EXTRUDER) {
      Extruder *e = &extruder[manage_monitor];
      out.print_int_P(PSTR(" "),e->currentTemperatureC>>CELSIUS_EXTRA_BITS); 
      out.print_int_P(PSTR(" "),e->targetTemperatureC>>CELSIUS_EXTRA_BITS);
      out.println_int_P(PSTR(" "),(int)pwm_pos[e->id]);
    }
#if HEATED_BED_SENSOR_TYPE!=0
    else {
      out.print_int_P(PSTR(" "),conv_raw_temp(HEATED_BED_SENSOR_TYPE,current_bed_raw)>>CELSIUS_EXTRA_BITS); 
      out.print_int_P(PSTR(" "),target_bed_celsius>>CELSIUS_EXTRA_BITS);
      out.println_int_P(PSTR(" "),(int)heated_bed_output);
    }
#endif
}

// ------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------- manage temperatures -----------------------------------------------
// ------------------------------------------------------------------------------------------------------------------

/** Makes updates to temperatures and heater state every call. It is optimized for speed,
 so only small updates are done every time. Call it frequently, minimum 40 times per sensor
 is suggested.
 the critical flag indicates, that the time for checking temp. is limited, so long operations
 are forbidden in this call.*/

void manage_temperatures() {
  byte manage_extruder = 0;  ///< Extruder number, we are looking at. 1+NUM_EXTRUDER is heated bed.
  for(manage_extruder=0;manage_extruder<NUM_EXTRUDER;manage_extruder++) {
    Extruder *act = &extruder[manage_extruder];
    // Get Temperature
       int oldTemp = act->currentTemperatureC;
       act->currentTemperature = read_raw_temperature(act->sensorType,act->sensorPin);
       act->currentTemperatureC = conv_raw_temp(act->sensorType,act->currentTemperature);
       byte on = act->currentTemperature>=act->targetTemperature ? LOW : HIGH;
#ifdef TEMP_PID
       act->tempArray[act->tempPointer++] = act->currentTemperatureC;
       act->tempPointer &= 7;
       if(act->heatManager==1) {
         byte output;
         int error = act->targetTemperatureC - act->currentTemperatureC;
         if(act->targetTemperatureC<(20<<CELSIUS_EXTRA_BITS)) output = 0; // off is off, even if damping term wants a heat peak!
         else if(error>(PID_CONTROL_RANGE<<CELSIUS_EXTRA_BITS)) 
           output = act->pidMax;
         else if(error<(-PID_CONTROL_RANGE<<CELSIUS_EXTRA_BITS))
           output = 0;
         else {
           long pidTerm = act->pidPGain * error; // *100
           act->tempIState = constrain(act->tempIState+error,act->tempIStateLimitMin,act->tempIStateLimitMax); // *1*1000ms
           pidTerm += act->pidIGain * act->tempIState/10; 
           long dgain = act->pidDGain * (act->tempArray[act->tempPointer]-act->currentTemperatureC); //*100
           pidTerm += dgain;
#if SCALE_PID_TO_MAX==1
           pidTerm = (pidTerm*act->pidMax)>>8;
#endif
           output = constrain(pidTerm/100, 0, act->pidMax) & 0xff;
          /*if(counter_250ms==1) { // some debug infos
            out.print_long_P(PSTR("PID:"),act->tempIState);
            out.print_long_P(PSTR(" "),pidTerm);
            out.print_long_P(PSTR(" "),act->pidPGain * error);
            out.println_long_P(PSTR(" "),dgain);
          } */
         }
         pwm_pos[act->id] = output;
       }
#endif
       if(act->heatManager == 0
#ifndef TEMP_PID
        || true
#endif
       ) {
         pwm_pos[act->id] = (on?255:0);
      }
#if LED_PIN>-1
      if(act == current_extruder)
        digitalWrite(LED_PIN,on);
#endif       
  }
  // Now manage heated bed
#if HEATED_BED_SENSOR_TYPE!=0
  unsigned long time = millis();
  current_bed_raw = read_raw_temperature(HEATED_BED_SENSOR_TYPE,HEATED_BED_SENSOR_PIN);
#if HEATED_BED_HEATER_PIN > -1
  if (time - last_bed_set > HEATED_BED_SET_INTERVAL) {
     digitalWrite(HEATED_BED_HEATER_PIN,current_bed_raw >= target_bed_raw ? LOW : HIGH);
     last_bed_set = time;
  }
#endif
  heated_bed_output = current_bed_raw >= target_bed_raw ? 0 : 255;
#endif
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

