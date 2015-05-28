#ifndef DRIVERS_H_INCLUDED
#define DRIVERS_H_INCLUDED

/**
For some special printers you need to control extra motors. Possible reasons are
- Extruder switches
- Clearing surface
- Leveling

Repetier-Firmware supports up to 4 extra motors that can be controlled by
G201 P<motorId> X<pos>     - Go to position X with motor X
G202 P<motorId> X<setpos>  - Mark current position as X
G203 P<motorId>            - Report current motor position
G204 P<motorId> S<0/1>     - Enable/disable motor

These motors are already special and there might be different types, so we can not assume
one class fits all needs. So to keep it simple, the firmware defines this general
interface whcih a motor must implement. That way we can handle any type without changing
the main code.
*/
class MotorDriverInterface
{
public:
    virtual void initialize() = 0;
    virtual float getPosition() = 0;
    virtual void setCurrentAs(float newPos) = 0;
    virtual void gotoPosition(float newPos) = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
};

/**
Simple class to drive a stepper motor with fixed speed.
*/
template<int stepPin, int dirPin, int enablePin,bool invertDir, bool invertEnable>
class StepperDriver : public MotorDriverInterface
{
   int32_t position;
   int32_t delayUS;
   float stepsPerMM;
public:
    StepperDriver(float _stepsPerMM,float speed)
    {
        stepsPerMM = _stepsPerMM;
        delayUS = 500000 / (speed * stepsPerMM);
    }
    void initialize() {
        HAL::pinMode(enablePin, OUTPUT);
        HAL::pinMode(stepPin, OUTPUT);
        HAL::pinMode(dirPin, OUTPUT);
        HAL::digitalWrite(enablePin, !invertEnable);
    }
    float getPosition()
    {
        return position / stepsPerMM;
    }
    void setCurrentAs(float newPos)
    {
        position = floor(newPos * stepsPerMM + 0.5f);
    }
    void gotoPosition(float newPos)
    {
        enable();
        int32_t target = floor(newPos * stepsPerMM + 0.5f) - position;
        position += target;
        if(target > 0) {
            HAL::digitalWrite(dirPin, !invertDir);
        } else {
            target = -target;
            HAL::digitalWrite(dirPin, invertDir);
        }
        while(target) {
            HAL::digitalWrite(stepPin, HIGH);
            HAL::delayMicroseconds(delayUS);
            HAL::digitalWrite(stepPin, LOW);
            HAL::delayMicroseconds(delayUS);
            target--;
            HAL::pingWatchdog();
            if((target & 127) == 0)
                Commands::checkForPeriodicalActions(false);
        }
    }
    void enable()
    {
        HAL::digitalWrite(enablePin, invertEnable);
    }
    void disable()
    {
        HAL::digitalWrite(enablePin, !invertEnable);
    }
};

#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
class GCode;
extern void commandG201(GCode &code);
extern void commandG202(GCode &code);
extern void commandG203(GCode &code);
extern void commandG204(GCode &code);
extern void disableAllMotorDrivers();
extern MotorDriverInterface *getMotorDriver(int idx);
extern void initializeAllMotorDrivers();
#endif

#endif // DRIVERS_H_INCLUDED
