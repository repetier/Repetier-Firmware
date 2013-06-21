#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#define CELSIUS_EXTRA_BITS 3

//#ifdef TEMP_PID
//extern byte current_extruder_out;
//#endif

// Updates the temperature of all extruders and heated bed if it's time.
// Toggels the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern byte manage_monitor;

/** TemperatureController manages one heater-temperature sensore loop. You can have up to
4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
class TemperatureController
{
    public:
    byte pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
    byte sensorType; ///< Type of temperature sensor.
    byte sensorPin; ///< Pin to read extruder temperature.
    int currentTemperature; ///< Currenttemperature value read from sensor.
    int targetTemperature; ///< Target temperature value in units of sensor.
    float currentTemperatureC; ///< Current temperature in ¡ãC.
    float targetTemperatureC; ///< Target temperature in ¡ãC.
    unsigned long lastTemperatureUpdate; ///< Time in millis of the last temperature update.
    char heatManager; ///< How is temperature controled. 0 = on/off, 1 = PID-Control
#ifdef TEMP_PID
    long tempIState; ///< Temp. var. for PID computation.
    byte pidDriveMax; ///< Used for windup in PID calculation.
    byte pidDriveMin; ///< Used for windup in PID calculation.
    float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
    float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
    float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
    byte pidMax; ///< Maximum PWM value, the heater should be set.
    float tempIStateLimitMax;
    float tempIStateLimitMin;
    byte tempPointer;
    float tempArray[4];
#endif

    void setTargetTemperature(float target);
    void updateCurrentTemperature();
    void updateTempControlVars();
#ifdef TEMP_PID
    void autotunePID(float temp,byte controllerId);
#endif
};
/** \brief Data to drive one extruder.

This structure contains all definitions for an extruder and all
current state variables, like current temperature, feeder position etc.
*/
class Extruder   // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
{
    public:
    static Extruder *current;
    byte id;
    long xOffset;
    long yOffset;
    float stepsPerMM;        ///< Steps per mm.
    byte enablePin;          ///< Pin to enable extruder stepper motor.
//  byte directionPin; ///< Pin number to assign the direction.
//  byte stepPin; ///< Pin number for a step.
    byte enableOn;
//  byte invertDir; ///< 1 if the direction of the extruder should be inverted.
    float maxFeedrate;      ///< Maximum feedrate in mm/s.
    float maxAcceleration;  ///< Maximum acceleration in mm/s^2.
    float maxStartFeedrate; ///< Maximum start feedrate in mm/s.
    long extrudePosition;   ///< Current extruder position in steps.
    int watchPeriod;        ///< Time in seconds, a M109 command will wait to stabalize temperature
    int waitRetractTemperature; ///< Temperature to retract the filament when waiting for heatup
    int waitRetractUnits;   ///< Units to retract the filament when waiting for heatup
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    float advanceK;         ///< Koefficient for advance algorithm. 0 = off
#endif
    float advanceL;
#endif
    TemperatureController tempControl;
    const char * PROGMEM selectCommands;
    const char * PROGMEM deselectCommands;
    byte coolerSpeed; ///< Speed to use when enabled
    byte coolerPWM; ///< current PWM setting

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
    static inline void setDirection(byte dir)
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
#endif
    }
    static void manageTemperatures();
    static void disableCurrentExtruderMotor();
    static void selectExtruderById(byte extruderId);
    static void disableAllHeater();
    static void initExtruder();
    static void initHeatedBed();
    static void setHeatedBedTemperature(float temp_celsius);
    static float getHeatedBedTemperature();
    static void setTemperatureForExtruder(float temp_celsius,byte extr);
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
extern Extruder extruder[];
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
extern byte autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
