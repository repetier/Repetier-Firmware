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

MotorDriverInterface *motorDrivers[NUM_MOTOR_DRIVERS] = {
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

MotorDriverInterface *getMotorDriver(int idx) {
    return motorDrivers[idx];
}

/**
Run motor P until it is at position X
*/
void commandG201(GCode &code) {
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->gotoPosition(code.X);
}

//G202 P<motorId> X<setpos>  - Mark current position as X
void commandG202(GCode &code) {
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    if(!code.hasX()) return;
    motorDrivers[id]->setCurrentAs(code.X);
}
//G203 P<motorId>            - Report current motor position
void commandG203(GCode &code) {
    int id = 0;
    if(code.hasP())
        id = code.P;
    if(id < 0) id = 0;
    if(id >= NUM_MOTOR_DRIVERS) id = 0;
    Com::printF(PSTR("Motor"),id);
    Com::printFLN(PSTR("Pos:"),motorDrivers[id]->getPosition());
}
//G204 P<motorId> S<0/1>     - Enable/disable motor
void commandG204(GCode &code) {
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

void disableAllMotorDrivers() {
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->disable();
}
void initializeAllMotorDrivers() {
    for(int i = 0; i < NUM_MOTOR_DRIVERS; i++)
        motorDrivers[i]->initialize();
}

#endif // NUM_MOTOR_DRIVERS
