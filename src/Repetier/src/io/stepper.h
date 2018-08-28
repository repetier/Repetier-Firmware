/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#undef STEPPER_SIMPLE
#undef STEPPER_MIRROR2
#undef STEPPER_MIRROR3
#undef STEPPER_MIRROR4

#if IO_TARGET == 4 // declare variable

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    extern SimpleStepperDriver<stepPin, dirPin, enablePin> name;
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    extern Mirror2StepperDriver name;
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    extern Mirror3StepperDriver name;
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    extern Mirror4StepperDriver name;

#elif IO_TARGET == 6 // define variables

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    SimpleStepperDriver<stepPin, dirPin, enablePin> name(&minEndstop, &maxEndstop);
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    Mirror2StepperDriver name(&motor1, &motor2, &minEndstop, &maxEndstop);
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    Mirror3StepperDriver name(&motor1, &motor2, &motor3, &minEndstop, &maxEndstop);
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    Mirror4StepperDriver name(&motor1, &motor2, &motor3, &motor4, &minEndstop, &maxEndstop);

#elif IO_TARGET == 1 // Init drivers at startup

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    name.init();

#elif IO_TARGET == 8 // call eepromHandle if required

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop)
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop)
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop)
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop)

#elif IO_TARGET == 9 // call updatedDerived to activate new settings

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop)
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop)
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop)
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop)

#else

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop)
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop)
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop)
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop)

#endif
