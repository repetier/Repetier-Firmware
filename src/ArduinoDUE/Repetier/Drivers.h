#ifndef DRIVERS_H_INCLUDED
#define DRIVERS_H_INCLUDED

/**
For some special printers you need to control extra motors. Possible reasons are
- Extruder switches
- Clearing surface
- Leveling

Repetier-Firmware supports up to 4 extra motors that can be controlled by
G201 P<motorId> X<pos>     - Go to position X with motor motorId
G202 P<motorId> X<setpos>  - Mark current position as X
G203 P<motorId>            - Report current motor position
G204 P<motorId> S<0/1>     - Enable/disable motor
G205 P<motorId> S<0/1> E<0/1> - Home motor, S1 = go back to stored position, E1
= home only if endstop was never met, meaning it was never homed with motor.

These motors are already special and there might be different types, so we can
not assume one class fits all needs. So to keep it simple, the firmware defines
this general interface which a motor must implement. That way we can handle any
type without changing the main code.
*/
class MotorDriverInterface {
public:
    virtual void initialize() = 0;
    virtual float getPosition() = 0;
    virtual void setCurrentAs(float newPos) = 0;
    virtual void gotoPosition(float newPos) = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void home(bool goToCurrent, bool onlyIfNotHomed) = 0;
    virtual bool endstopHit() = 0;
};

/**
Simple class to drive a stepper motor with fixed speed.
*/
template <int stepPin, int dirPin, int enablePin, bool invertDir,
          bool invertEnable>
class StepperDriver : public MotorDriverInterface {
    int32_t position;
    int32_t delayUS;
    float stepsPerMM;

public:
    StepperDriver(float _stepsPerMM, float speed) {
        stepsPerMM = _stepsPerMM;
        position = 0;
        delayUS = 500000 / (speed * stepsPerMM);
    }
    void initialize() {
        HAL::pinMode(enablePin, OUTPUT);
        HAL::pinMode(stepPin, OUTPUT);
        HAL::pinMode(dirPin, OUTPUT);
        HAL::digitalWrite(enablePin, !invertEnable);
    }
    bool endstopHit() {
        return false;
    }
    float getPosition() { return position / stepsPerMM; }
    void setCurrentAs(float newPos) {
        position = floor(newPos * stepsPerMM + 0.5f);
    }
    void gotoPosition(float newPos) {
        enable();
        int32_t target = floor(newPos * stepsPerMM + 0.5f) - position;
        position += target;
        if (target > 0) {
            HAL::digitalWrite(dirPin, !invertDir);
        } else {
            target = -target;
            HAL::digitalWrite(dirPin, invertDir);
        }
        while (target) {
            HAL::digitalWrite(stepPin, HIGH);
            HAL::delayMicroseconds(delayUS);
            HAL::digitalWrite(stepPin, LOW);
            HAL::delayMicroseconds(delayUS);
            target--;
            HAL::pingWatchdog();
            if ((target & 127) == 0) {
                Commands::checkForPeriodicalActions(false);
                GCode::keepAlive(Processing);
            }
        }
    }
    void enable() { HAL::digitalWrite(enablePin, invertEnable); }
    void disable() { HAL::digitalWrite(enablePin, !invertEnable); }
    void home(bool goToCurrent, bool onlyIfNotHomed) {}
};

/**
Simple class to drive a stepper motor with fixed speed with additional endstop.
Min position is 0 and max. position maxDistance.
*/
template <int stepPin, int dirPin, int enablePin, bool invertDir,
          bool invertEnable, int endstopPin, bool invertEndstop,
          bool minEndstop, bool endstopPullup>
class StepperDriverWithEndstop : public MotorDriverInterface {
    int32_t position;
    int32_t delayUS;
    float stepsPerMM;
    float maxDistance;
    bool isHomed;

public:
    StepperDriverWithEndstop(float _stepsPerMM, float speed, float maxDist) {
        stepsPerMM = _stepsPerMM;
        maxDistance = maxDist;
        isHomed = false;
        position = 0;
        delayUS = 500000 / (speed * stepsPerMM);
    }
    void initialize() {
        HAL::pinMode(enablePin, OUTPUT);
        HAL::pinMode(stepPin, OUTPUT);
        HAL::pinMode(dirPin, OUTPUT);
        HAL::pinMode(endstopPin, endstopPullup ? INPUT_PULLUP : INPUT);
        HAL::digitalWrite(enablePin, !invertEnable);
    }
    bool endstopHit() {
        return invertEndstop ? !HAL::digitalRead(endstopPin)
                             : HAL::digitalRead(endstopPin);
    }
    float getPosition() { return position / stepsPerMM; }
    void setCurrentAs(float newPos) {
        position = floor(newPos * stepsPerMM + 0.5f);
    }
    void gotoPosition(float newPos) {
        bool up = true;
        if (newPos < 0)
            newPos = 0;
        if (newPos > maxDistance)
            newPos = maxDistance;
        enable();
        int32_t target = floor(newPos * stepsPerMM + 0.5f) - position;
        position += target;
        if (target > 0) {
            HAL::digitalWrite(dirPin, !invertDir);
        } else {
            target = -target;
            up = false;
            HAL::digitalWrite(dirPin, invertDir);
        }
        while (target) {
            HAL::digitalWrite(stepPin, HIGH);
            HAL::delayMicroseconds(delayUS);
            HAL::digitalWrite(stepPin, LOW);
            HAL::delayMicroseconds(delayUS);
            target--;
            HAL::pingWatchdog();
            if ((target & 127) == 0) {
                Commands::checkForPeriodicalActions(false);
                GCode::keepAlive(Processing);
            }
            if (up != minEndstop) {
                if (endstopHit()) {
                    isHomed = true;
                    if (minEndstop)
                        position = 0;
                    else
                        setCurrentAs(maxDistance);
                    break;
                }
            }
        }
    }
    void home(bool goToCurrent, bool onlyIfNotHomed) {
        if (onlyIfNotHomed && isHomed)
            return;
        float origPosition = getPosition();
        if (minEndstop) {
            setCurrentAs(maxDistance);
            gotoPosition(0);
        } else {
            position = 0;
            gotoPosition(maxDistance);
        }
        if (goToCurrent)
            gotoPosition(origPosition);
    }
    void enable() { HAL::digitalWrite(enablePin, invertEnable); }
    void disable() { HAL::digitalWrite(enablePin, !invertEnable); }
};

#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
class GCode;
extern void commandG201(GCode& code);
extern void commandG202(GCode& code);
extern void commandG203(GCode& code);
extern void commandG204(GCode& code);
extern void commandG205(GCode& code);
extern void disableAllMotorDrivers();
extern MotorDriverInterface* getMotorDriver(int idx);
extern void initializeAllMotorDrivers();
#endif

#if defined(SUPPORT_LASER) && SUPPORT_LASER
/**
With laser support you can exchange a extruder by a laser. A laser gets
controlled by a digital pin. By default all intensities > 200 are always on, and
lower values are always off. You can overwrite this with a programmed event
EVENT_SET_LASER(intensity) that return false to signal the default
implementation that it has set it's value already.
EVENT_INITIALIZE_LASER should return false to prevent default initialization.
*/
class LaserDriver {
public:
    static secondspeed_t intensity; // Intensity to use for next move queued. This
                                    // is NOT the current value!
    static secondspeed_t intens;
    static bool laserOn; // Enabled by M3?
    static bool firstMove;
    static void initialize();
    static void changeIntensity(secondspeed_t newIntensity);
};
#endif

#if defined(SUPPORT_CNC) && SUPPORT_CNC
/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an
influence on the spindle. The motor also keeps running for G0 moves. M3 and M4
wait for old moves to be finished and then enables the motor. It then waits
CNC_WAIT_ON_ENABLE milliseconds for the spindle to reach target speed.
*/
class CNCDriver {
public:
    static int8_t direction;
    static secondspeed_t spindleSpeed;
    static uint16_t spindleRpm;

    /** Initialize cnc pins. EVENT_INITIALIZE_CNC should return false to prevent
   * default initialization.*/
    static void initialize();
    /** Turns off spindle. For event override implement
  EVENT_SPINDLE_OFF
  returning false.
  */
    static void spindleOff();
    /** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN.
  If CNC_DIRECTION_PIN is not -1 it sets direction to CNC_DIRECTION_CW. rpm is
  ignored. To override with event system, return false for the event
  EVENT_SPINDLE_CW(rpm)
  */
    static void spindleOnCW(int32_t rpm);
    /** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN.
  If CNC_DIRECTION_PIN is not -1 it sets direction to !CNC_DIRECTION_CW. rpm is
  ignored. To override with event system, return false for the event
  EVENT_SPINDLE_CCW(rpm)
  */
    static void spindleOnCCW(int32_t rpm);
};
#endif

#endif // DRIVERS_H_INCLUDED
