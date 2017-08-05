#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3
#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions

// Updates the temperature of all extruders and heated bed if it's time.
// Toggles the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;
#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2    ///< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4    ///< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT    8    ///< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16   ///< Indicating sensor decoupling
#define TEMPERATURE_CONTROLLER_FLAG_JAM           32   ///< Indicates a jammed filament
#define TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN      64   ///< Indicates a slowed down extruder
#define TEMPERATURE_CONTROLLER_FLAG_FILAMENTCHANGE 128 ///< Indicates we are switching filament

#ifndef PID_TEMP_CORRECTION
#define PID_TEMP_CORRECTION 2.0
#endif

/** TemperatureController manages one heater-temperature sensor loop. You can have up to
4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
class TemperatureController
{
public:
    uint8_t pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
    uint8_t sensorType; ///< Type of temperature sensor.
    uint8_t sensorPin; ///< Pin to read extruder temperature.
    int8_t heatManager; ///< How is temperature controlled. 0 = on/off, 1 = PID-Control, 3 = dead time control
    int16_t currentTemperature; ///< Current temperature value read from sensor.
    //int16_t targetTemperature; ///< Target temperature value in units of sensor.
    float currentTemperatureC; ///< Current temperature in degC.
    float targetTemperatureC; ///< Target temperature in degC.
	float temperatureC; ///< For 1s updates temperature and last build a short time history
	float lastTemperatureC; ///< Used to compute D errors.
    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.
    float tempIState; ///< Temp. var. for PID computation.
    uint8_t pidDriveMax; ///< Used for windup in PID calculation.
    uint8_t pidDriveMin; ///< Used for windup in PID calculation.
#define deadTime pidPGain
    // deadTime is logically different value but physically overlays pidPGain for saving space
    float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
    float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
    float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
    uint8_t pidMax; ///< Maximum PWM value, the heater should be set.
    float tempIStateLimitMax;
    float tempIStateLimitMin;
    uint8_t flags;
    millis_t lastDecoupleTest;  ///< Last time of decoupling sensor-heater test
    float  lastDecoupleTemp;  ///< Temperature on last test
    millis_t decoupleTestPeriod; ///< Time between setting and testing decoupling.
    millis_t preheatStartTime;    ///< Time (in milliseconds) when heat up was started
    int16_t preheatTemperature;

    void setTargetTemperature(float target);
    void updateCurrentTemperature();
    void updateTempControlVars();
    inline bool isAlarm()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }
    inline void setAlarm(bool on)
    {
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_ALARM;
        else flags &= ~TEMPERATURE_CONTROLLER_FLAG_ALARM;
    }
    inline bool isDecoupleFull()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
	inline void removeErrorStates() {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_ALARM | TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT | TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED);
	}
    inline bool isDecoupleFullOrHold()
    {
        return flags & (TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    }
    inline void setDecoupleFull(bool on)
    {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
    }
    inline bool isDecoupleHold()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void setDecoupleHold(bool on)
    {
        flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
    }
    inline void startFullDecouple(millis_t &t)
    {
        if(isDecoupleFull()) return;
        lastDecoupleTest = t;
        lastDecoupleTemp = currentTemperatureC;
        setDecoupleFull(true);
    }
    inline void startHoldDecouple(millis_t &t)
    {
        if(isDecoupleHold()) return;
        if(fabs(currentTemperatureC - targetTemperatureC) + 1 > DECOUPLING_TEST_MAX_HOLD_VARIANCE) return;
        lastDecoupleTest = t;
        lastDecoupleTemp = targetTemperatureC;
        setDecoupleHold(true);
    }
    inline void stopDecouple()
    {
        setDecoupleFull(false);
    }
    inline bool isSensorDefect()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
    }
    inline bool isSensorDecoupled()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
    }
	static void resetAllErrorStates();
	fast8_t errorState();
    inline bool isFilamentChange()
    {
	    return flags & TEMPERATURE_CONTROLLER_FLAG_FILAMENTCHANGE;
    }
	inline bool isJammed()
	{
		return flags & TEMPERATURE_CONTROLLER_FLAG_JAM;
	}
    inline bool isSlowedDown()
    {
	    return flags & TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }
#if EXTRUDER_JAM_CONTROL
    inline void setFilamentChange(bool on)
    {
	    flags &= ~TEMPERATURE_CONTROLLER_FLAG_FILAMENTCHANGE;
	    if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_FILAMENTCHANGE;
    }
    void setJammed(bool on);
    inline void setSlowedDown(bool on)
    {
        flags &= ~TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }

#endif
    void waitForTargetTemperature();
    void autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeResult, int method);
   inline void startPreheatTime()
   {
       preheatStartTime = HAL::timeInMilliseconds();
   }
   inline void resetPreheatTime()
   {
       preheatStartTime = 0;
   }
   inline millis_t preheatTime()
   {
       return preheatStartTime == 0 ? 0 : HAL::timeInMilliseconds() - preheatStartTime;
   }
};
class Extruder;
extern Extruder extruder[];

#if EXTRUDER_JAM_CONTROL
#if JAM_METHOD == 1
#define _TEST_EXTRUDER_JAM(x,pin) {\
	uint8_t sig = READ(pin);extruder[x].jamStepsSinceLastSignal += extruder[x].jamLastDir;\
	if(extruder[x].jamLastSignal != sig && abs(extruder[x].jamStepsSinceLastSignal - extruder[x].jamLastChangeAt) > JAM_MIN_STEPS) {\
		if(sig) {extruder[x].resetJamSteps();} \
		extruder[x].jamLastSignal = sig;extruder[x].jamLastChangeAt = extruder[x].jamStepsSinceLastSignal;\
	} else if(abs(extruder[x].jamStepsSinceLastSignal) > extruder[x].jamErrorSteps && !Printer::isDebugJamOrDisabled() && !extruder[x].tempControl.isJammed() && !extruder[x].tempControl.isFilamentChange()) {\
	if(extruder[x].jamLastDir > 0) {\
	extruder[x].tempControl.setJammed(true);\
	} else {\
	extruder[x].tempControl.setFilamentChange(true);}} \
}
#define RESET_EXTRUDER_JAM(x,dir) extruder[x].jamLastDir = dir ? 1 : -1;
#elif JAM_METHOD == 2
#define _TEST_EXTRUDER_JAM(x,pin) {\
        uint8_t sig = READ(pin);\
		  if(sig != extruder[x].jamLastSignal) {\
			  extruder[x].jamLastSignal = sig;\
			  if(sig)\
				{extruder[x].tempControl.setFilamentChange(true);extruder[x].tempControl.setJammed(true);} \
			  else if(!Printer::isDebugJamOrDisabled() && extruder[x].tempControl.isJammed()) \
				{extruder[x].resetJamSteps();}}\
		  }
#define RESET_EXTRUDER_JAM(x,dir)
#elif JAM_METHOD == 3
#define _TEST_EXTRUDER_JAM(x,pin) {\
	uint8_t sig = !READ(pin);\
	if(sig != extruder[x].jamLastSignal) {\
		extruder[x].jamLastSignal = sig;\
		if(sig)\
		{extruder[x].tempControl.setFilamentChange(true);extruder[x].tempControl.setJammed(true);} \
		else if(!Printer::isDebugJamOrDisabled() && extruder[x].tempControl.isJammed()) \
		{extruder[x].resetJamSteps();}}\
	}
#define RESET_EXTRUDER_JAM(x,dir)
#else
#error Unknown value for JAM_METHOD
#endif
#define ___TEST_EXTRUDER_JAM(x,y) _TEST_EXTRUDER_JAM(x,y)
#define __TEST_EXTRUDER_JAM(x) ___TEST_EXTRUDER_JAM(x,EXT ## x ## _JAM_PIN)
#define TEST_EXTRUDER_JAM(x) __TEST_EXTRUDER_JAM(x)
#else
#define TEST_EXTRUDER_JAM(x)
#define RESET_EXTRUDER_JAM(x,dir)
#endif

#define EXTRUDER_FLAG_RETRACTED 1
#define EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT 2 ///< Waiting for the first signal to start counting

/** \brief Data to drive one extruder.

This structure contains all definitions for an extruder and all
current state variables, like current temperature, feeder position etc.
*/
class Extruder   // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
{
public:
    static Extruder *current;
#if FEATURE_DITTO_PRINTING
    static uint8_t dittoMode;
#endif
#if MIXING_EXTRUDER > 0
    static int mixingS; ///< Sum of all weights
    static uint8_t mixingDir; ///< Direction flag
    static uint8_t activeMixingExtruder;
	static void recomputeMixingExtruderSteps();
#endif
    uint8_t id;
    int32_t xOffset;
    int32_t yOffset;
    int32_t zOffset;
    float stepsPerMM;        ///< Steps per mm.
    int8_t enablePin;          ///< Pin to enable extruder stepper motor.
//  uint8_t directionPin; ///< Pin number to assign the direction.
//  uint8_t stepPin; ///< Pin number for a step.
    uint8_t enableOn;
//  uint8_t invertDir; ///< 1 if the direction of the extruder should be inverted.
    float maxFeedrate;      ///< Maximum feedrate in mm/s.
    float maxAcceleration;  ///< Maximum acceleration in mm/s^2.
    float maxStartFeedrate; ///< Maximum start feedrate in mm/s.
    int32_t extrudePosition;   ///< Current extruder position in steps.
    int16_t watchPeriod;        ///< Time in seconds, a M109 command will wait to stabilize temperature
    int16_t waitRetractTemperature; ///< Temperature to retract the filament when waiting for heat up
    int16_t waitRetractUnits;   ///< Units to retract the filament when waiting for heat up
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    float advanceK;         ///< Coefficient for advance algorithm. 0 = off
#endif
    float advanceL;
    int16_t advanceBacklash;
#endif // USE_ADVANCE
#if MIXING_EXTRUDER > 0
    int mixingW;   ///< Weight for this extruder when mixing steps
	int mixingWB;  ///< Weight after balancing extruder steps per mm
    int mixingE;   ///< Cumulated error for this step.
    int virtualWeights[VIRTUAL_EXTRUDER]; // Virtual extruder weights
#endif // MIXING_EXTRUDER > 0
    TemperatureController tempControl;
    const char * PROGMEM selectCommands;
    const char * PROGMEM deselectCommands;
    uint8_t coolerSpeed; ///< Speed to use when enabled
    uint8_t coolerPWM; ///< current PWM setting
    float diameter;
    uint8_t flags;
#if EXTRUDER_JAM_CONTROL
    int32_t jamStepsSinceLastSignal; // when was the last signal
    uint8_t jamLastSignal; // what was the last signal
    int8_t jamLastDir;
    int32_t jamStepsOnSignal;
    int32_t jamLastChangeAt;
	int32_t jamSlowdownSteps;
	int32_t jamErrorSteps;
	uint8_t jamSlowdownTo;
#endif

    // Methods here

#if EXTRUDER_JAM_CONTROL
    inline bool isWaitJamStartcount()
    {
        return flags & EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT;
    }
    inline void setWaitJamStartcount(bool on)
    {
        if(on) flags |= EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT;
        else flags &= ~(EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT);
    }
    static void markAllUnjammed();
    void resetJamSteps();
#endif
#if MIXING_EXTRUDER > 0
    static void setMixingWeight(uint8_t extr,int weight);
#endif
    static void step();
    static void unstep();
    static void setDirection(uint8_t dir);
    static void enable();
#if FEATURE_RETRACTION
    inline bool isRetracted() {return (flags & EXTRUDER_FLAG_RETRACTED) != 0;}
    inline void setRetracted(bool on) {
        flags = (flags & (255 - EXTRUDER_FLAG_RETRACTED)) | (on ? EXTRUDER_FLAG_RETRACTED : 0);
    }
    void retract(bool isRetract,bool isLong);
    void retractDistance(float dist,bool extraLength = false);
#endif
    static void manageTemperatures();
    static void disableCurrentExtruderMotor();
    static void disableAllExtruderMotors();
    static void selectExtruderById(uint8_t extruderId);
    static void disableAllHeater();
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temp_celsius,bool beep = false);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temp_celsius,uint8_t extr,bool beep = false,bool wait = false);
    static void pauseExtruders(bool bed = false);
    static void unpauseExtruders(bool wait = true);
};

#if HAVE_HEATED_BED
#define HEATED_BED_INDEX NUM_EXTRUDER
extern TemperatureController heatedBedController;
#else
#define HEATED_BED_INDEX NUM_EXTRUDER-1
#endif
#if FAN_THERMO_PIN > -1
#define THERMO_CONTROLLER_INDEX HEATED_BED_INDEX+1
extern TemperatureController thermoController;
#else
#define THERMO_CONTROLLER_INDEX HEATED_BED_INDEX
#endif
#define NUM_TEMPERATURE_LOOPS THERMO_CONTROLLER_INDEX+1

#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

//extern Extruder *Extruder::current;
#if NUM_TEMPERATURE_LOOPS > 0
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
#endif
extern uint8_t autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
