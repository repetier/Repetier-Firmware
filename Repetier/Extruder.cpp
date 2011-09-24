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
 {0,EXT0_X_OFFSET,EXT0_Y_OFFSET,EXT0_STEPS_PER_MM,EXT0_TEMPSENSOR_TYPE,EXT0_TEMPSENSOR_PIN,EXT0_HEATER_PIN,EXT0_ENABLE_PIN,EXT0_DIR_PIN,EXT0_STEP_PIN,EXT0_ENABLE_ON,EXT0_INVERSE,
   EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,0,0,0,0,0,EXT0_HEAT_MANAGER,EXT0_WATCHPERIOD,EXT0_ADVANCE_K
#ifdef TEMP_PID
  ,0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_PGAIN,EXT0_PID_IGAIN,EXT0_PID_DGAIN,EXT0_PID_MAX
#endif 
 } 
#if NUM_EXTRUDER>1
 ,{1,EXT1_X_OFFSET,EXT1_Y_OFFSET,EXT1_STEPS_PER_MM,EXT1_TEMPSENSOR_TYPE,EXT1_TEMPSENSOR_PIN,EXT1_HEATER_PIN,EXT1_ENABLE_PIN,EXT1_DIR_PIN,EXT1_STEP_PIN,EXT1_ENABLE_ON,EXT1_INVERSE,
   EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,0,0,0,0,0,EXT1_HEAT_MANAGER,EXT1_WATCHPERIOD,EXT1_ADVANCE_K
#ifdef TEMP_PID
  ,0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_PGAIN,EXT1_PID_IGAIN,EXT1_PID_DGAIN,EXT1_PID_MAX
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

byte manage_extruder = 0;  ///< Extruder number, we are looking at. 1+NUM_EXTRUDER is heated bed.
int manage_sum = 0;        ///< Used for avagering temperature readings.
byte manage_step = 0;      ///< Processing step of temperature management.
byte manage_pin = 0;       ///< Used sensor pin.
long manage_lastcall = 0;  ///< Time of last call. So we can limit calls to desired frequency.
byte manage_monitor = 255; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed

#ifdef SUPPORT_MAX6675
extern int read_max6675(byte ss_pin);
#endif

#if ANALOG_INPUTS>0
static uint8 osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
static uint8 osAnalogInputCounter[ANALOG_INPUTS];
static uint osAnalogInputBuildup[ANALOG_INPUTS];
static uint8 osAnalogInputPos=0; // Current sampling position
volatile uint osAnalogInputValues[ANALOG_INPUTS];
ISR(ADC_vect) {
  osAnalogInputBuildup[osAnalogInputPos] += ADCW;
  if(++osAnalogInputCounter[osAnalogInputPos]>=_BV(ANALOG_INPUT_SAMPLE)) {
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE<12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos] <<
      (12-ANALOG_INPUT_BITS-ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE>12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos] >>
      (ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE-12);
#endif
#if ANALOG_INPUT_BITS+ANALOG_INPUT_SAMPLE==12
    osAnalogInputValues[osAnalogInputPos] =
      osAnalogInputBuildup[osAnalogInputPos];
#endif
    osAnalogInputBuildup[osAnalogInputPos] = 0;
    osAnalogInputCounter[osAnalogInputPos] = 0;
    // Start next conversion
    if(++osAnalogInputPos>=ANALOG_INPUTS) osAnalogInputPos = 0;
    ADMUX = (ADMUX & ~(0x1F)) | (pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]));
  }
  ADCSRA |= _BV(ADSC);  // start next conversion
}
#endif

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
  float delta = 1022.0f/GENERIC_THERM_NUM_ENTRIES;
  for(i=0;i<GENERIC_THERM_NUM_ENTRIES;i++) {
    int adc = (int)(i*delta+1);
    if(adc>1023) adc=1023;
    temptable_generic[i][0] = adc;
    float v = (float)(adc*GENERIC_THERM_VADC)/1024.0f;
    float r = GENERIC_RS*v/(GENERIC_VS-v);
    temptable_generic[i][1] = (int)(GENERIC_THERM_BETA/log(r/k)-273.15f+0.5f /* for correct rounding */);
#ifdef DEBUG_GENERIC
    out.print_int_P(PSTR("GenTemp: "),temptable_generic[i][0]); 
    out.println_int_P(PSTR(","),temptable_generic[i][1]); 
#endif
  }
#endif
  
  for(i=0;i<NUM_EXTRUDER;++i) {
    Extruder *act = &extruder[i];
    pinMode(act->directionPin,OUTPUT);
    pinMode(act->stepPin,OUTPUT);
    if(act->enablePin > -1) {
      pinMode(act->enablePin,OUTPUT);
      if(!act->enableOn) digitalWrite(act->enablePin,HIGH);
    }
    if(act->heaterPin > -1) pinMode(act->heaterPin,OUTPUT);
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
  ADMUX = (ADMUX & ~(0x1F)) | (osAnalogInputChannels[osAnalogInputPos]);
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
#endif
  
}
void extruder_select(byte ext_num) {
   if(ext_num>=NUM_EXTRUDER)
     ext_num = 0;
   current_extruder->extrudePosition = printer_state.currentPositionSteps[3];
   printer_state.currentPositionSteps[0] -= current_extruder->xOffset;
   printer_state.currentPositionSteps[1] -= current_extruder->yOffset;
   current_extruder = &extruder[ext_num];
   printer_state.currentPositionSteps[0] += current_extruder->xOffset;
   printer_state.currentPositionSteps[1] += current_extruder->yOffset;
   printer_state.currentPositionSteps[3] = current_extruder->extrudePosition;   
   axis_steps_per_unit[3] = current_extruder->stepsPerMM;
   inv_axis_steps_per_unit[3] = 1.0f/axis_steps_per_unit[3];
   printer_state.currentPositionSteps[3] = current_extruder->extrudePosition;
   max_feedrate[3] = current_extruder->maxFeedrate;
//   max_start_speed_units_per_second[3] = current_extruder->maxStartFeedrate;
   max_acceleration_units_per_sq_second[3] = max_travel_acceleration_units_per_sq_second[3] = current_extruder->maxAcceleration;
   axis_travel_steps_per_sqr_second[3] = axis_steps_per_sqr_second[3] = max_acceleration_units_per_sq_second[3] * axis_steps_per_unit[3];
   queue_move(false); // Move head of new extruder to old position using last feedrate
}
/*void extruder_set_position(float pos,bool relative) {
  current_extruder->extrudePosition = (relative ? current_extruder->extrudePosition : 0)+pos*current_extruder->stepsPerMM;
}*/
void extruder_set_temperature(int temp_celsius) {
#ifdef MAXTEMP
  if(temp_celsius>MAXTEMP) temp_celsius = MAXTEMP;
#endif
#ifdef MINTEMP
  if(temp_celsius<MINTEMP) temp_celsius = MINTEMP;
#endif
  current_extruder->targetTemperature = conv_temp_raw(current_extruder->sensorType,temp_celsius);
  current_extruder->targetTemperatureC = temp_celsius;
}
int extruder_get_temperature() {
  return conv_raw_temp(current_extruder->sensorType,current_extruder->currentTemperature);
}
void heated_bed_set_temperature(int temp_celsius) {
#if HEATED_BED_SENSOR_TYPE!=0  
   if(temp_celsius>150) temp_celsius = 150;
   target_bed_raw = conv_temp_raw(HEATED_BED_SENSOR_TYPE,temp_celsius);
#endif     
}
int heated_bed_get_temperature() {
#if HEATED_BED_SENSOR_TYPE!=0  
   return conv_raw_temp(HEATED_BED_SENSOR_TYPE,current_bed_raw);
#else
   return -1;
#endif     
}

/** \brief Disable stepper motor of current extruder. */
void extruder_disable() {
  if(current_extruder->enablePin > -1) 
    digitalWrite(current_extruder->enablePin,!current_extruder->enableOn); 
}
#define NUMTEMPS_1 61
const short temptable_1[NUMTEMPS_1][2] PROGMEM = {
{23,300},{25,295},{27,290},{28,285},{31,280},{33,275},{35,270},{38,265},{41,260},{44,255},
{48,250},{52,245},{56,240},{61,235},{66,230},{71,225},{78,220},{84,215},{92,210},{100,205},
{109,200},{120,195},{131,190},{143,185},{156,180},{171,175},{187,170},{205,165},{224,160},
{245,155},{268,150},{293,145},{320,140},{348,135},{379,130},{411,125},{445,120},{480,115},
{516,110},{553,105},{591,100},{628,95},{665,90},{702,85},{737,80},{770,75},{801,70},{830,65},
{857,60},{881,55},{903,50},{922,45},{939,40},{954,35},{966,30},{977,25},{985,20},{993,15},
{999,10},{1004,5},{1008,0} //safety
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM = {
   {1, 848},{54, 275}, {107, 228}, {160, 202},{213, 185}, {266, 171}, {319, 160}, {372, 150},
   {425, 141}, {478, 133},{531, 125},{584, 118},{637, 110},{690, 103},{743, 95},{796, 86},
   {849, 77},{902, 65},{955, 49},{1008, 17},{1020, 0} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM = {
  {1,864},{21,300},{25,290},{29,280},{33,270},{39,260},{46,250},{54,240},{64,230},{75,220},
  {90,210},{107,200},{128,190},{154,180},{184,170},{221,160},{265,150},{316,140},{375,130},
  {441,120},{513,110},{588,100},{734,80},{856,60},{938,40},{986,20},{1008,0},{1018,-20}	};

#define NUMTEMPS_4 20
short temptable_4[NUMTEMPS_4][2] PROGMEM = {
   {1, 430},{54, 137},{107, 107},{160, 91},{213, 80},{266, 71},{319, 64},{372, 57},{425, 51},
   {478, 46},{531, 41},{584, 35},{637, 30},{690, 25},{743, 20},{796, 14},{849, 7},{902, 0},
   {955, -11},{1008, -35}};
short temptable_5[NUM_TEMPS_USERTHERMISTOR][2] PROGMEM = USER_THERMISTORTABLE ;
const short *temptables[5] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0],(short int *)&temptable_5[0][0]};
const byte temptables_num[5] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR};

int read_raw_temperature(byte type,byte pin) {
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 99:
      return 1023-(osAnalogInputValues[pin]>>ANALOG_INPUT_SAMPLE);
    case 100: // AD595
      return (osAnalogInputValues[pin]>>ANALOG_INPUT_SAMPLE);
#ifdef SUPPORT_MAX6675
    case 101: // MAX6675
      return read_max6675(pin);
#endif
  }
  return 2000; // unknown method, return high value to switch heater off for safety
}
int conv_raw_temp(byte type,int raw_temp) {
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: {
      type--;
      byte num = pgm_read_byte(&temptables_num[type])<<1;
      byte i=2;
      const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
      short oldraw = pgm_read_word(&temptable[0]);
      short oldtemp = pgm_read_word(&temptable[1]);
      short newraw,newtemp;
      raw_temp = 1023-raw_temp;
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
    case 100: // AD595
      return (int)((long)raw_temp * 500/1024);
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
      raw_temp = 1023-raw_temp;
      while(i<GENERIC_THERM_NUM_ENTRIES*2) {
        newraw = temptable[i++];
        newtemp = temptable[i++];
        if (newraw > raw_temp)
          return oldtemp + (long)(raw_temp-oldraw)*(long)(newtemp-oldtemp)/(newraw-oldraw);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return newtemp;}
#endif
  }
}
int conv_temp_raw(byte type,int temp) {
  switch(type) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: {
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
          return 1023- oldraw + (long)(oldtemp-temp)*(long)(oldraw-newraw)/(oldtemp-newtemp);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return 1023-newraw;
    }
    case 100: // HEATER_USES_AD595
      return (int)((long)temp * 1024 / 500);
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
          return 1023- oldraw + (long)(oldtemp-temp)*(long)(oldraw-newraw)/(oldtemp-newtemp);
        oldtemp = newtemp;
        oldraw = newraw;
      }
      // Overflow: Set to last value in the table
      return 1023-newraw;
    }
#endif
  }
}

void monitor_temp(unsigned long time,int current,int target,int output) {
  if(manage_monitor == manage_extruder) { // Monitor temperatures
    out.print_long_P(PSTR("MTEMP:"),time);
    out.print_int_P(PSTR(" "),current); 
    out.print_int_P(PSTR(" "),target);
    out.println_int_P(PSTR(" "),output);
  }
}
// Makes updates to temperatures and heater state every call. It is optimized for speed,
// so only small updates are done every time. Call it frequently, minimum 40 times per sensor
// is suggested.
// the critical flag indicates, that the time for checking temp. is limited, so long operations
// are forbidden in this call.
void manage_temperatures(bool critical) {
  unsigned long time = millis();
  if(time-manage_lastcall<1000/((ANALOG_SUPERSAMPLE+1)*(NUM_ANALOG_SENSORS)*4+NUM_DIGITAL_SENSORS)) return; // 4 Times per second new temperatures
  manage_lastcall = time;
  if(manage_step==0) { // First value init
    if(manage_extruder<NUM_EXTRUDER) {
       Extruder *act = &extruder[manage_extruder];
       if(act->sensorType>100) { // no analog values needed
         if(critical) return;
         act->currentTemperature = read_raw_temperature(act->sensorType,act->sensorPin);
         manage_extruder++; // next to test
         return;
       }
       manage_pin = act->sensorPin;
       if(act->sensorType==100) { // no analog values needed
         manage_sum = act->currentTemperature = act->currentTemperature; // /ANALOG_SUPERSAMPLE;
       } else {
         manage_sum = 1023-act->currentTemperature; // /ANALOG_SUPERSAMPLE;
       } 
    } else {
#if HEATED_BED_SENSOR_TYPE!=0
#else
      manage_extruder = 0;
      manage_sum = 0;
      return;
#endif
    }
    manage_step++;
  } else {
    manage_sum = (9L*(long)manage_sum+(long)((osAnalogInputValues[manage_pin]>>ANALOG_INPUT_SAMPLE)))/10L;
    if(manage_step<ANALOG_SUPERSAMPLE) {
        manage_step++;
        return;
      }
  }
  if(manage_step == ANALOG_SUPERSAMPLE) { // conversion finished
    manage_step = 0;
    if(manage_extruder<NUM_EXTRUDER) {
       Extruder *act = &extruder[manage_extruder];
       int oldTemp = act->currentTemperatureC;
       if(act->sensorType==100) { // no analog values needed
         act->currentTemperature = manage_sum; // /ANALOG_SUPERSAMPLE;
       } else {
         act->currentTemperature = 1023-manage_sum; // /ANALOG_SUPERSAMPLE;
       } 
       act->currentTemperatureC = conv_raw_temp(act->sensorType,act->currentTemperature);
       byte on = act->currentTemperature>=act->targetTemperature ? LOW : HIGH;
#ifdef TEMP_PID
       if(act->heatManager==1) {
         long error = act->targetTemperatureC - act->currentTemperatureC;
         long pidTerm = act->pidPGain * error; // *100
         if(act->pidIGain) { // prevent division by zero
           long windup = (long)act->pidDriveMax*100L/act->pidIGain;
           act->tempIState = constrain(act->tempIState+error, -windup,windup); // *1*1000ms
           pidTerm += act->pidIGain * act->tempIState; 
         }
         pidTerm += act->pidDGain * (oldTemp-act->currentTemperatureC); //*100
         byte output = constrain(pidTerm/100, 0, act->pidMax) & 0xff;    
         if(act->targetTemperatureC<20) output = 0; // off is off, even if damping term wants a heat peak!
         analogWrite(act->heaterPin, output);
         monitor_temp(time,act->currentTemperatureC,act->targetTemperatureC,output);
       }
#endif
       if(act->heatManager == 0
#ifndef TEMP_PID
        || true
#endif
       ) {
         digitalWrite(act->heaterPin,on);
         monitor_temp(time,act->currentTemperatureC,act->targetTemperatureC,(on?255:0));
      }
#if LED_PIN>-1
      if(act == current_extruder)
        digitalWrite(LED_PIN,on);
#endif       
       act->lastTemperatureUpdate = time;
       manage_extruder++;
    } else { // heated bed measurement is ready
#if HEATED_BED_SENSOR_TYPE!=0
      current_bed_raw = read_raw_temperature(HEATED_BED_SENSOR_TYPE,HEATED_BED_SENSOR_PIN);
#if HEATED_BED_HEATER_PIN > -1
      digitalWrite(HEATED_BED_HEATER_PIN,current_bed_raw >= target_bed_raw ? LOW : HIGH);
#endif
      monitor_temp(time,conv_raw_temp(HEATED_BED_SENSOR_TYPE,current_bed_raw),conv_raw_temp(HEATED_BED_SENSOR_TYPE,target_bed_raw),current_bed_raw >= target_bed_raw ? 0 : 255);
#endif
      manage_extruder = 0; // Restart loop with extruder 0
    }
  }
}
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

