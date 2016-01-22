
/*
  Makers Tool Works RGB LED I2C control
  I2C Device ID = 21

  Repetier-Firmware port/rewrite by exuvo 2015-05-17
  Contributed to MTW by OhmEye October 2014
  */

// option switches
#define MTWLED_cool EXTRUDER_FAN_COOL_TEMP      // The temp at which the hotend is considered cooled down and safe
#define MTWLED_swing 4                          // how far off before the temperature is not considered 'at temp' in degrees C
#define MTWLED_endstoptimer 3                   // how many seconds to display endstop status

// patterns               id   R   G   B
#define mtwled_ready      10,  0, 30,  0    // Printer Ready
#define mtwled_startup    10, 30, 30, 30    // Printer startup
#define mtwled_temphit    14, 40, 40, 40    // Hotend is at target temp
#define mtwled_templow    10, 40,  0, 40    // Hotend heater is slightly lower than target temp
#define mtwled_temphigh   10, 40,  0,  0    // Hotend heater is slightly higher than target temp
#define mtwled_heateroff  11,  0,  0, 40    // Hotend heater is off but still hot
#define mtwled_heating0   10,  0,  0, 50    // Hotend heating up <10%
#define mtwled_heating1   10,  0,  0,100    // Hotend heating up <20%
#define mtwled_heating2   10,  0, 50,100    // Hotend heating up <30%
#define mtwled_heating3   10,  0,100,100    // Hotend heating up <40%
#define mtwled_heating4   10,  0,100, 50    // Hotend heating up <50%
#define mtwled_heating5   10,100,100,  0    // Hotend heating up <60%
#define mtwled_heating6   10,100, 50,  0    // Hotend heating up <70%
#define mtwled_heating7   10,100,  0,100    // Hotend heating up <80%
#define mtwled_heating8   10,100,  0, 50    // Hotend heating up <90%
#define mtwled_heating9   10,100,  0,  0    // Hotend heating up <100%

/*
  Pattern ID is the number of the pattern/animation to use
  Current patterns are:
      10 RGB	Solid color
      11 RGB 	Cylon
      12 RGB 	UFO PULSE
      13 XXX 	Color Chase
      14 XXX 	Rainbow Cycle
      15 RGB 	Color Chase Single Led
      16 RGB 	Slow fill then solid
      17 RGB	Repeating Blink
      18 XXX  Rainbow
      19 XXX  Theather Chase Rainbow
      90-99   Reserved for heating and cooling values
   R is a value from 0-127 for how red the color will be
   G is a value from 0-127 for how green the color will be
   B is a value from 0-127 for how blue the color will be
      Specifying colors is often optional, any color not given will be either 0 (none) or a default
      depending on the pattern selected.
   T is a timer in seconds for how long the pattern will override the default patterns
*/

// Pattern Selection Table for defaults that must not be changed
#define mtwled_nochange 	1	// Reserved for no change to LED Strip

#ifndef UINT8_MAX
#define UINT8_MAX 255
#endif // UINT8_MAX


union patterncode {  // access a pattern both as 32 bits and as array of 4 uint8_ts.
  uint32_t value;
  uint8_t part[4];
};

patterncode MTWLED_lastpattern;
uint16_t MTWLED_timer;
bool MTWLED_starup;
uint8_t waitingForHeaterIndex = UINT8_MAX;
float waitingForHeaterStartC;

uint32_t MTWLEDConvert(uint8_t pattern, uint8_t red, uint8_t green, uint8_t blue) {
  patterncode pc;
  pc.part[0] = pattern;
  pc.part[1] = red;
  pc.part[2] = green;
  pc.part[3] = blue;
  return pc.value;
}

// send pattern frame via I2C
void MTWLED_Write(uint8_t pattern, uint8_t red, uint8_t green, uint8_t blue, uint16_t timer) {
  if(pattern == 0) {
    return;
  }

  uint32_t converted = MTWLEDConvert(pattern, red, green, blue);

  if(converted != MTWLED_lastpattern.value) {  // don't send sequential identical patterns
    uint8_t frame[] = { 250, pattern, red, green, blue };

    HAL::i2cStart((21 << 1) + I2C_WRITE);
    HAL::i2cWrite(frame[0]);
    HAL::i2cWrite(frame[1]);
    HAL::i2cWrite(frame[2]);
    HAL::i2cWrite(frame[3]);
    HAL::i2cWrite(frame[4]);
    HAL::i2cStop();

    MTWLED_lastpattern.value = converted;  // update states

    if(timer) {
      MTWLED_timer = timer * 10U;
    }
  }
}

bool MTWLEDEndstop(bool force) {
  uint8_t endx=0, endy=0, endz=0;

  Endstops::update();

  #if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
    if(Endstops::xMin()) endx |= 1;
  #endif
  #if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
    if(Endstops::yMin()) endy |= 1;
  #endif
  #if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
    if(Endstops::zMin()) endz |= 1;
  #endif
  #if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
    if(Endstops::xMax()) endx |= 1;
  #endif
  #if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
    if(Endstops::yMax()) endy |= 1;
  #endif
  #if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
    if(Endstops::zMax()) endz |= 1;
  #endif

  if(force || endx || endy || endz) {
    MTWLED_Write(2, endx, endy, endz, MTWLED_endstoptimer);

    if(endx || endy || endz) {
        return true;
    }
  }

  return false;
}

void MTWLED_WaitingHeater(int8_t id){
  if(id == -1){
    waitingForHeaterIndex = NUM_TEMPERATURE_LOOPS - 1;
  } else if (id >= 0 && id < NUM_TEMPERATURE_LOOPS) {
    waitingForHeaterIndex = id;
  } else {
   //Error
   waitingForHeaterIndex = UINT8_MAX;
  }

  if(waitingForHeaterIndex < NUM_TEMPERATURE_LOOPS){
    waitingForHeaterStartC = tempController[waitingForHeaterIndex]->currentTemperatureC;
  }
}

void MTWLED_HeatingFinished(int8_t id){
  waitingForHeaterIndex = UINT8_MAX;
}

//does percentile display between start temp and target temp while a heater is heating up
bool MTWLEDTemp() {
	if(waitingForHeaterIndex >= NUM_TEMPERATURE_LOOPS){
    return false;
	}

  float target = tempController[waitingForHeaterIndex]->targetTemperatureC;
  float current = tempController[waitingForHeaterIndex]->currentTemperatureC;
  float start = waitingForHeaterStartC;

  uint8_t pattern = (10.0f * (1.0f - ((target - current) / (target - start))));

  switch(pattern){
    default:
    case 0:
      MTWLED_Write(mtwled_heating0, 0);
      break;
    case 1:
      MTWLED_Write(mtwled_heating1, 0);
      break;
    case 2:
      MTWLED_Write(mtwled_heating2, 0);
      break;
    case 3:
      MTWLED_Write(mtwled_heating3, 0);
      break;
    case 4:
      MTWLED_Write(mtwled_heating4, 0);
      break;
    case 5:
      MTWLED_Write(mtwled_heating5, 0);
      break;
    case 6:
      MTWLED_Write(mtwled_heating6, 0);
      break;
    case 7:
      MTWLED_Write(mtwled_heating7, 0);
      break;
    case 8:
      MTWLED_Write(mtwled_heating8, 0);
      break;
    case 9:
      MTWLED_Write(mtwled_heating9, 0);
      break;
  }

  return true;
}

void MTWLEDSetup(){
  HAL::i2cInit(400000L);
  MTWLED_Write(mtwled_startup, 1);
}

//Called every 100ms
void MTWLED_Update() {
  if(!MTWLED_starup) { // if this is first time display endstop status before clearing to ready
    MTWLEDSetup();
    MTWLEDEndstop(true);
    MTWLED_starup = true;
    return;
  }

  if(MTWLEDTemp() || MTWLEDEndstop(false) || MTWLED_lastpattern.part[0] == mtwled_nochange) {
    return;
  }

  if(MTWLED_timer > 0){
    MTWLED_timer--;
    return;
  }

  float currentTempC =  0;
  float targetTempC = 0;

  for(uint8_t i=0; i < NUM_EXTRUDER; i++){
    float target = tempController[i]->targetTemperatureC;
    float current = tempController[i]->currentTemperatureC;

    if(target > targetTempC || (targetTempC == 0 && target == 0 && current > currentTempC)){
      currentTempC = current;
      targetTempC = target;
    }
  }

  if(targetTempC == 0) {
    if(currentTempC > MTWLED_cool) { // heater is off but still warm
      MTWLED_Write(mtwled_heateroff, 0);
    } else {
      MTWLED_Write(mtwled_ready, 0);
    }

  } else {
    uint8_t swing = abs(targetTempC - currentTempC); // how far off from target temp we are

    if(swing < MTWLED_swing * 2) {                  // if within double the swing threshold
      if(swing < MTWLED_swing) {
        MTWLED_Write(mtwled_temphit, 0);  // close to target temp, so consider us 'at temp'
      } else {
        if(currentTempC >= targetTempC) {
          MTWLED_Write(mtwled_temphigh, 0);   // temp high, heater is off
        } else {
          MTWLED_Write(mtwled_templow, 0);    // temp low, heater is on
        }
      }
    }
  }
}

