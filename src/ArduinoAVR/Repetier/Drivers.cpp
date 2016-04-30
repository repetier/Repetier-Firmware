#include "Repetier.h"

#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
MOTOR_DRIVER_1(motorDriver1);
#if NUM_MOTOR_DRIVERS > 1
MOTOR_DRIVER_2(motorDriver2);
#endif
#if NUM_MOTOR_DRIVERS > 2
MOTOR_DRIVER_3(motorDriver3);
#endif
#if NUM_MOTOR_DRIVERS > 3
MOTOR_DRIVER_4(motorDriver4);
#endif
#if NUM_MOTOR_DRIVERS > 4
MOTOR_DRIVER_5(motorDriver5);
#endif
#if NUM_MOTOR_DRIVERS > 5
MOTOR_DRIVER_6(motorDriver6);
#endif

MotorDriverInterface *motorDrivers[NUM_MOTOR_DRIVERS] =
{
    &motorDriver1
#if NUM_MOTOR_DRIVERS > 1
    , &motorDriver2
#endif
#if NUM_MOTOR_DRIVERS > 2
    , &motorDriver3
#endif
#if NUM_MOTOR_DRIVERS > 3
    , &motorDriver4
#endif
#if NUM_MOTOR_DRIVERS > 4
    , &motorDriver5
#endif
#if NUM_MOTOR_DRIVERS > 5
    , &motorDriver6
#endif
};

MotorDriverInterface *getMotorDriver(int idx)
{
    return motorDrivers[idx];
}

/**
Run motor P until it is at position X
*/
void commandG201(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->gotoPosition(code.X);
}

//G202 P<motorId> X<setpos>  - Mark current position as X
void commandG202(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->setCurrentAs(code.X);
}
//G203 P<motorId>            - Report current motor position
void commandG203(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    Com::printF(PSTR("Motor"),id);
    Com::printFLN(PSTR("Pos:"),motorDrivers[id]->getPosition());
}
//G204 P<motorId> S<0/1>     - Enable/disable motor
void commandG204(GCode &code)
{
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasS()) return;
    if(code.S)
        motorDrivers[id]->enable();
    else
        motorDrivers[id]->disable();
}

void disableAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->disable();
}
void initializeAllMotorDrivers()
{
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->initialize();
}

#endif // NUM_MOTOR_DRIVERS

#if defined(SUPPORT_LASER) && SUPPORT_LASER
uint8_t LaserDriver::intensity = 255; // Intensity to use for next move queued if we want lasers. This is NOT the current value!
bool LaserDriver::laserOn = false;
void LaserDriver::initialize()
{
    if(EVENT_INITALIZE_LASER)
    {
#if LASER_PIN > -1
        SET_OUTPUT(LASER_PIN);
        WRITE(LASER_PIN, !LASER_ON_HIGH); // be sure laser is off!
#endif
    }
    changeIntensity(0);
}
void LaserDriver::changeIntensity(uint8_t newIntensity)
{
    if(EVENT_SET_LASER(newIntensity))
    {
        // Default implementation
#if LASER_PIN > -1
#if (LASER_TYPE == LASER_ON_OFF)
        WRITE(LASER_PIN,(LASER_ON_HIGH ? newIntensity > 199 : newIntensity < 200));
#elif (LASER_TYPE == LASER_PWM)
        HAL::setPWM(LASER_PIN, newIntensity, !LASER_ON_HIGH);
#endif
#endif
    }
}
#endif // SUPPORT_LASER

#if defined(SUPPORT_CNC) && SUPPORT_CNC
/**
The CNC driver differs a bit from laser driver. Here only M3,M4,M5 have an influence on the spindle.
The motor also keeps running for G0 moves. M3 and M4 wait for old moves to be finished and then enables
the motor. It then waits CNC_WAIT_ON_ENABLE milliseconds for the spindle to reach target speed.
*/

int8_t CNCDriver::direction = 0;
/** Initialize cnc pins. EVENT_INITALIZE_CNC should return false to prevent default initalization.*/
void CNCDriver::initialize()
{
    if(EVENT_INITALIZE_CNC)
    {
#if CNC_ENABLE_PIN > -1
        SET_OUTPUT(CNC_ENABLE_PIN);
        WRITE(CNC_ENABLE_PIN,!CNC_ENABLE_WITH);
#endif
#if CNC_DIRECTION_PIN > -1
        SET_OUTPUT(CNC_DIRECTION_PIN);
#endif
#if CNC_PWM_PIN > -1
        SET_OUTPUT(CNC_PWM_PIN);
        WRITE(CNC_PWM_PIN, CNC_PWM_INV);
#endif
    }
}
/** Turns off spindle. For event override implement
EVENT_SPINDLE_OFF
returning false.
*/
void CNCDriver::spindleOff()
{
    if(direction == 0) return; // already off
    if(EVENT_SPINDLE_OFF)
    {
#if CNC_PWM_PIN > -1
        HAL::setPWM(0, CNC_PWM_PIN, CNC_PWM_INV);
#endif
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN,!CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_DISABLE);
	direction = 0;
}
/** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN. If
CNC_DIRECTION_PIN is not -1 it sets direction to CNC_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CW(rpm)
*/
void CNCDriver::spindleOnCW(int32_t rpm)
{
    if(direction == 1)
        return;
    spindleOff();
    direction = 1;
    if(EVENT_SPINDLE_CW(rpm)) {
#if CNC_PWM_PIN > -1
#if defined(CNC_MAX_RPM)
        if(rpm > CNC_MAX_RPM) {
            rpm = CNC_MAX_RPM;
        }
        HAL::setPWM((uint8_t)(rpm/(CNC_MAX_RPM/TOOL_PWM_STEPS)), CNC_PWM_PIN, CNC_PWM_INV);
#else
        HAL::setPWM(rpm, CNC_PWM_PIN, CNC_PWM_INV);
#endif
#endif
#if CNC_DIRECTION_PIN > -1
        WRITE(CNC_DIRECTION_PIN, CNC_DIRECTION_CW);
#endif
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN, CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_ENABLE);
}
/** Turns spindle on. Default implementation uses a enable pin CNC_ENABLE_PIN. If
CNC_DIRECTION_PIN is not -1 it sets direction to !CNC_DIRECTION_CW. rpm is ignored.
To override with event system, return false for the event
EVENT_SPINDLE_CCW(rpm)
*/
void CNCDriver::spindleOnCCW(int32_t rpm)
{
    if(direction == -1)
        return;
    spindleOff();
    direction = -1;
    if(EVENT_SPINDLE_CW(rpm)) {
#if CNC_PWM_PIN > -1
#if defined(CNC_MAX_RPM)
        if(rpm > CNC_MAX_RPM) {
            rpm = CNC_MAX_RPM;
        }
        HAL::setPWM((uint8_t)(rpm/(CNC_MAX_RPM/TOOL_PWM_STEPS)), CNC_PWM_PIN, CNC_PWM_INV);
#else
        HAL::setPWM(rpm, CNC_PWM_PIN, CNC_PWM_INV);
#endif
#endif
#if CNC_DIRECTION_PIN > -1
        WRITE(CNC_DIRECTION_PIN, !CNC_DIRECTION_CW);
#endif
#if CNC_ENABLE_PIN > -1
        WRITE(CNC_ENABLE_PIN, CNC_ENABLE_WITH);
#endif
    }
    HAL::delayMilliseconds(CNC_WAIT_ON_ENABLE);
}
#endif
