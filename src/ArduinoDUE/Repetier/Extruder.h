#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3
#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions

//#if TEMP_PID
//extern uint8_t current_extruder_out;
//#endif

// Updates the temperature of all extruders and heated bed if it's time.
// Toggles the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;
#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

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

#define HEATED_BED_INDEX NUM_EXTRUDER
extern TemperatureController heatedBedController;
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
