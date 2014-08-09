#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3

//#ifdef TEMP_PID
//extern uint8_t current_extruder_out;
//#endif

// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
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
#ifdef TEMP_PID
    float tempIState; ///< Temp. var. for PID computation.
    uint8_t pidDriveMax; ///< Used for windup in PID calculation.
    uint8_t pidDriveMin; ///< Used for windup in PID calculation.
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

    void setTargetTemperature(float target);
    void updateCurrentTemperature();
    void updateTempControlVars();
    inline bool isAlarm() {return flags & TEMPERATURE_CONTROLLER_FLAG_ALARM;}
    inline void setAlarm(bool on) {if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_ALARM; else flags &= ~TEMPERATURE_CONTROLLER_FLAG_ALARM;}
#ifdef TEMP_PID
    void autotunePID(float temp,uint8_t controllerId,bool storeResult);
#endif
};

class Extruder;
extern Extruder extruder[];

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
    float stepsPerMM;        ///< Steps per mm.
    uint8_t enablePin;          ///< Pin to enable extruder stepper motor.
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
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    float advanceK;         ///< Koefficient for advance algorithm. 0 = off
#endif
    float advanceL;
    int16_t advanceBacklash;
#endif
    TemperatureController tempControl;
    const char * PROGMEM selectCommands;
    const char * PROGMEM deselectCommands;
    uint8_t coolerSpeed; ///< Speed to use when enabled
    uint8_t coolerPWM; ///< current PWM setting

    /** \brief Sends the high-signal to the stepper for next extruder step.
    Call this function only, if interrupts are disabled.
    */
    static inline void step()
    {
#if NUM_EXTRUDER==1
        WRITE(EXT0_STEP_PIN,HIGH);
#else
        switch(Extruder::current->id)
        {
        case 0:
#if NUM_EXTRUDER>0
            WRITE(EXT0_STEP_PIN,HIGH);
#if FEATURE_DITTO_PRINTING
            if(Extruder::dittoMode) {
                WRITE(EXT1_STEP_PIN,HIGH);
#if NUM_EXTRUDER > 2
                if(Extruder::dittoMode > 1) {
                    WRITE(EXT2_STEP_PIN,HIGH);
                }
#endif
#if NUM_EXTRUDER > 3
                if(Extruder::dittoMode > 2) {
                    WRITE(EXT3_STEP_PIN,HIGH);
                }
#endif
            }
#endif
#endif
            break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
        case 1:
            WRITE(EXT1_STEP_PIN,HIGH);
            break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER>2
        case 2:
            WRITE(EXT2_STEP_PIN,HIGH);
            break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER>3
        case 3:
            WRITE(EXT3_STEP_PIN,HIGH);
            break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER>4
        case 4:
            WRITE(EXT4_STEP_PIN,HIGH);
            break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER>5
        case 5:
            WRITE(EXT5_STEP_PIN,HIGH);
            break;
#endif
        }
#endif
    }
    /** \brief Sets stepper signal to low for current extruder.

    Call this function only, if interrupts are disabled.
    */
    static inline void unstep()
    {
#if NUM_EXTRUDER==1
        WRITE(EXT0_STEP_PIN,LOW);
#else
        switch(Extruder::current->id)
        {
        case 0:
#if NUM_EXTRUDER>0
            WRITE(EXT0_STEP_PIN,LOW);
#if FEATURE_DITTO_PRINTING
            if(Extruder::dittoMode) {
                WRITE(EXT1_STEP_PIN,LOW);
#if NUM_EXTRUDER > 2
            if(Extruder::dittoMode > 1) {
                WRITE(EXT2_STEP_PIN,LOW);
            }
#endif
#if NUM_EXTRUDER > 3
            if(Extruder::dittoMode > 2) {
                WRITE(EXT3_STEP_PIN,LOW);
            }
#endif
            }
#endif
#endif
            break;
#if defined(EXT1_STEP_PIN) && NUM_EXTRUDER>1
        case 1:
            WRITE(EXT1_STEP_PIN,LOW);
            break;
#endif
#if defined(EXT2_STEP_PIN) && NUM_EXTRUDER>2
        case 2:
            WRITE(EXT2_STEP_PIN,LOW);
            break;
#endif
#if defined(EXT3_STEP_PIN) && NUM_EXTRUDER>3
        case 3:
            WRITE(EXT3_STEP_PIN,LOW);
            break;
#endif
#if defined(EXT4_STEP_PIN) && NUM_EXTRUDER>4
        case 4:
            WRITE(EXT4_STEP_PIN,LOW);
            break;
#endif
#if defined(EXT5_STEP_PIN) && NUM_EXTRUDER>5
        case 5:
            WRITE(EXT5_STEP_PIN,LOW);
            break;
#endif
        }
#endif
    }
    /** \brief Activates the extruder stepper and sets the direction. */
    static inline void setDirection(uint8_t dir)
    {
#if NUM_EXTRUDER==1
        if(dir)
            WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
        else
            WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
#else
        switch(Extruder::current->id)
        {
#if NUM_EXTRUDER>0
        case 0:
            if(dir)
                WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);
            else
                WRITE(EXT0_DIR_PIN,EXT0_INVERSE);
#if FEATURE_DITTO_PRINTING
            if(Extruder::dittoMode) {
                if(dir)
                    WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
                else
                    WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
            }
#endif
            break;
#endif
#if defined(EXT1_DIR_PIN) && NUM_EXTRUDER>1
        case 1:
            if(dir)
                WRITE(EXT1_DIR_PIN,!EXT1_INVERSE);
            else
                WRITE(EXT1_DIR_PIN,EXT1_INVERSE);
            break;
#endif
#if defined(EXT2_DIR_PIN) && NUM_EXTRUDER>2
        case 2:
            if(dir)
                WRITE(EXT2_DIR_PIN,!EXT2_INVERSE);
            else
                WRITE(EXT2_DIR_PIN,EXT2_INVERSE);
            break;
#endif
#if defined(EXT3_DIR_PIN) && NUM_EXTRUDER>3
        case 3:
            if(dir)
                WRITE(EXT3_DIR_PIN,!EXT3_INVERSE);
            else
                WRITE(EXT3_DIR_PIN,EXT3_INVERSE);
            break;
#endif
#if defined(EXT4_DIR_PIN) && NUM_EXTRUDER>4
        case 4:
            if(dir)
                WRITE(EXT4_DIR_PIN,!EXT4_INVERSE);
            else
                WRITE(EXT4_DIR_PIN,EXT4_INVERSE);
            break;
#endif
#if defined(EXT5_DIR_PIN) && NUM_EXTRUDER>5
        case 5:
            if(dir)
                WRITE(EXT5_DIR_PIN,!EXT5_INVERSE);
            else
                WRITE(EXT5_DIR_PIN,EXT5_INVERSE);
            break;
#endif
        }
#endif
    }
    static inline void enable()
    {
#if NUM_EXTRUDER==1
#if EXT0_ENABLE_PIN>-1
        WRITE(EXT0_ENABLE_PIN,EXT0_ENABLE_ON );
#endif
#else
        if(Extruder::current->enablePin > -1)
            digitalWrite(Extruder::current->enablePin,Extruder::current->enableOn);
#if FEATURE_DITTO_PRINTING
        if(Extruder::dittoMode) {
            if(extruder[1].enablePin > -1)
                digitalWrite(extruder[1].enablePin,extruder[1].enableOn);
#if NUM_EXTRUDER > 2
        if(Extruder::dittoMode > 1 && extruder[2].enablePin > -1)
                digitalWrite(extruder[2].enablePin,extruder[2].enableOn);
#endif
#if NUM_EXTRUDER > 3
        if(Extruder::dittoMode > 2 && extruder[3].enablePin > -1)
                digitalWrite(extruder[3].enablePin,extruder[3].enableOn);
#endif
        }
#endif
#endif
    }
    static void manageTemperatures();
    static void disableCurrentExtruderMotor();
    static void disableAllExtruderMotors();
    static void selectExtruderById(uint8_t extruderId);
    static void disableAllHeater();
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temp_celsius,bool beep = false);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temp_celsius,uint8_t extr,bool beep = false);
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
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
extern uint8_t autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
