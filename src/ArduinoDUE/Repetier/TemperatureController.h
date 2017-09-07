
#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2  //< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4  //< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT    8  //< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16 //< Indicating sensor decoupling
#define TEMPERATURE_CONTROLLER_FLAG_JAM           32 //< Indicates a jammed filament
#define TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN      64 //< Indicates a slowed down extruder

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
    uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.
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
