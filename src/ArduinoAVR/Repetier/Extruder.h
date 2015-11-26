#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3
#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions

//#if TEMP_PID
//extern uint8_t current_extruder_out;
//#endif

// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;
#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2  //< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4  //< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT    8  //< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16 //< Indicating sensor decoupling
#define TEMPERATURE_CONTROLLER_FLAG_JAM           32 //< Indicates a jammed filament
#define TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN      64 //< Indicates a slowed down extruder

/** TemperatureController manages one heater-temperature sensore loop. You can have up to
4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
class TemperatureController
{
public:
    uint8_t pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
    uint8_t sensorType; ///< Type of temperature sensor.
    uint8_t sensorPin; ///< Pin to read extruder temperature.
    int16_t currentTemperature; ///< Currenttemperature value read from sensor.
    int16_t targetTemperature; ///< Target temperature value in units of sensor.
    float currentTemperatureC; ///< Current temperature in degC.
    float targetTemperatureC; ///< Target temperature in degC.
    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.
    int8_t heatManager; ///< How is temperature controled. 0 = on/off, 1 = PID-Control, 3 = deat time control
#if TEMP_PID
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
    uint8_t tempPointer;
    float tempArray[4];
#endif
    uint8_t flags;
    millis_t lastDecoupleTest;  ///< Last time of decoupling sensor-heater test
    float  lastDecoupleTemp;  ///< Temperature on last test
    millis_t decoupleTestPeriod; ///< Time between setting and testing decoupling.

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
#if EXTRUDER_JAM_CONTROL
    inline bool isJammed()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_JAM;
    }
    void setJammed(bool on);
    inline bool isSlowedDown()
    {
        return flags & TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }
    inline void setSlowedDown(bool on)
    {
        flags &= ~TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
        if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
    }

#endif
    void waitForTargetTemperature();
#if TEMP_PID
    void autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeResult);
#endif
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
        } else if(abs(extruder[x].jamStepsSinceLastSignal) > JAM_ERROR_STEPS && !Printer::isDebugJamOrDisabled() && !extruder[x].tempControl.isJammed()) \
            extruder[x].tempControl.setJammed(true);\
    }
#define RESET_EXTRUDER_JAM(x,dir) extruder[x].jamLastDir = dir ? 1 : -1;
#elif JAM_METHOD == 2
#define _TEST_EXTRUDER_JAM(x,pin) {\
        uint8_t sig = READ(pin);\
          if(sig){extruder[x].tempControl.setJammed(true);} else if(!Printer::isDebugJamOrDisabled() && !extruder[x].tempControl.isJammed()) {extruder[x].resetJamSteps();}}
#define RESET_EXTRUDER_JAM(x,dir)
#elif JAM_METHOD == 3
#define _TEST_EXTRUDER_JAM(x,pin) {\
        uint8_t sig = !READ(pin);\
          if(sig){extruder[x].tempControl.setJammed(true);} else if(!Printer::isDebugJamOrDisabled() && !extruder[x].tempControl.isJammed()) {extruder[x].resetJamSteps();}}
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
    int16_t watchPeriod;        ///< Time in seconds, a M109 command will wait to stabalize temperature
    int16_t waitRetractTemperature; ///< Temperature to retract the filament when waiting for heatup
    int16_t waitRetractUnits;   ///< Units to retract the filament when waiting for heatup
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    float advanceK;         ///< Koefficient for advance algorithm. 0 = off
#endif
    float advanceL;
    int16_t advanceBacklash;
#endif // USE_ADVANCE
#if MIXING_EXTRUDER > 0
    int mixingW;   ///< Weight for this extruder when mixing steps
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
    int16_t jamStepsSinceLastSignal; // when was the last signal
    uint8_t jamLastSignal; // what was the last signal
    int8_t jamLastDir;
    int16_t jamStepsOnSignal;
    int16_t jamLastChangeAt;
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
    void retractDistance(float dist);
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
    static void pauseExtruders();
    static void unpauseExtruders();
};

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
extern TemperatureController heatedBedController;
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif
#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

//extern Extruder *Extruder::current;
#if NUM_TEMPERATURE_LOOPS > 0
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
#endif
extern uint8_t autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
