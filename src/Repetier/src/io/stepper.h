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
#undef STEPPER_OBSERVEABLE
#undef STEPPER_ADJUST_RESOLUTION

#if IO_TARGET == IO_TARGET_CLASS_DEFINITION // declare variable

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    extern SimpleStepperDriver<stepPin, dirPin, enablePin> name; \
    typedef SimpleStepperDriver<stepPin, dirPin, enablePin> name##Type;
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    extern Mirror2StepperDriver name; \
    typedef Mirror2StepperDriver name##Type;
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    extern Mirror3StepperDriver name; \
    typedef Mirror3StepperDriver name##Type;
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    extern Mirror4StepperDriver name; \
    typedef Mirror4StepperDriver name##Type;
#define STEPPER_OBSERVEABLE(name, driver) \
    extern ObservableStepperDriver<driver##Type> name; \
    typedef ObservableStepperDriver<driver##Type> name##Type;
#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    extern AdjustResolutionStepperDriver<driver##Type> name; \
    typedef AdjustResolutionStepperDriver<driver##Type> name##Type;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // define variables

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    SimpleStepperDriver<stepPin, dirPin, enablePin> name(&minEndstop, &maxEndstop);
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    Mirror2StepperDriver name(&motor1, &motor2, &minEndstop, &maxEndstop);
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    Mirror3StepperDriver name(&motor1, &motor2, &motor3, &minEndstop, &maxEndstop);
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    Mirror4StepperDriver name(&motor1, &motor2, &motor3, &motor4, &minEndstop, &maxEndstop);
#define STEPPER_OBSERVEABLE(name, driver) \
    ObservableStepperDriver<driver##Type> name(&driver);
#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    AdjustResolutionStepperDriver<driver##Type> name(&driver, from, to);

#elif IO_TARGET == IO_TARGET_INIT // Init drivers at startup

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    name.init();

#elif IO_TRAGET == IO_TARGET_RESTORE_FROM_CONFIG

#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    name.restoreFromConfiguration(to);

#elif IO_TARGET == IO_TARGET_TOOLS_TEMPLATES

#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    template class AdjustResolutionStepperDriver<driver##Type>;

#elif IO_TARGET == IO_TARGET_UPDATE_DERIVED // call updatedDerived to activate new settings

#endif

#ifndef STEPPER_SIMPLE
#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_MIRROR2
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_MIRROR3
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_MIRROR4
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_OBSERVEABLE
#define STEPPER_OBSERVEABLE(name, driver)
#endif
#ifndef STEPPER_ADJUST_RESOLUTION
#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to)
#endif
