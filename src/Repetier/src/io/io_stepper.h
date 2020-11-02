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
#undef STEPPER_TMC2130_HW_SPI
#undef STEPPER_TMC5160_HW_SPI
#undef STEPPER_TMC2130_SW_SPI
#undef STEPPER_TMC5160_SW_SPI
#undef STEPPER_TMC2208_HW_UART
#undef STEPPER_TMC2209_HW_UART
#undef STEPPER_TMC2209_SW_UART

#if IO_TARGET == IO_TARGET_CLASS_DEFINITION // declare variable

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    extern SimpleStepperDriver<stepPin, dirPin, enablePin> name; \
    typedef SimpleStepperDriver<stepPin, dirPin, enablePin> name##Type;
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC2130Stepper name##Inst; \
    extern TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC5160Stepper name##Inst; \
    extern TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC2130Stepper name##Inst; \
    extern TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC5160Stepper name##Inst; \
    extern TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    extern TMC2208Stepper name##Inst; \
    extern TMCStepper2208Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper2208Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC2209Stepper name##Inst; \
    extern TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name##Type;
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    extern TMC2209Stepper name##Inst; \
    extern TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name; \
    typedef TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name##Type;
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
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC2130Stepper name##Inst(csPin, rsense, chainPos); \
    TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC5160Stepper name##Inst(csPin, rsense, chainPos); \
    TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC2130Stepper name##Inst(csPin, rsense, mosiPin, misoPin, sckPin, chainPos); \
    TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC5160Stepper name##Inst(csPin, rsense, mosiPin, misoPin, sckPin, chainPos); \
    TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    TMC2208Stepper name##Inst(&serial, rsense, (bool)true); \
    TMCStepper2208Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed);
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC2209Stepper name##Inst(&serial, rsense, slaveAddr); \
    TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, false);
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    TMC2209Stepper name##Inst(rxPin, txPin, rsense, slaveAddr); \
    TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk> name(&minEndstop, &maxEndstop, &name##Inst, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, true);
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

#elif IO_TARGET == IO_TARGET_INIT

#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.eepromReserve();
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.eepromReserve();
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.eepromReserve();
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.eepromReserve();
#if defined(SAMD51_BOARD) || defined(ARDUINO_ARCH_STM32)
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    serial.begin(115200); \
    name.eepromReserve();
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    serial.begin(115200); \
    name.eepromReserve();
#else
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    serial.begin(115200); \
    serial.setInterruptPriority(5); \
    name.eepromReserve();
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    serial.begin(115200); \
    serial.setInterruptPriority(5); \
    name.eepromReserve();
#endif
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.eepromReserve();

#elif IO_TARGET == IO_TARGET_INIT_LATE // Init drivers at startup

#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    SoftwareSerial::setInterruptPriority(1, 0); \
    name.init();
#define STEPPER_MIRROR2(name, motor1, motor2, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR3(name, motor1, motor2, motor3, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_MIRROR4(name, motor1, motor2, motor3, motor4, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    name.init();

#elif IO_TARGET == IO_TARGET_RESTORE_FROM_CONFIG

#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    name.restoreFromConfiguration(to);
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed);

#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);

#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.reset(microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity);

#elif IO_TARGET == IO_TARGET_TEMPLATES

#define STEPPER_ADJUST_RESOLUTION(name, driver, from, to) \
    template class AdjustResolutionStepperDriver<driver##Type>;
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper2130Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper5160Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    template class TMCStepper2208Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk>;
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    template class TMCStepper2209Driver<stepPin, dirPin, enablePin, fclk>;

#elif IO_TARGET == IO_TARGET_UPDATE_DERIVED // call updatedDerived to activate new settings

#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.init();

#elif IO_TARGET == IO_TARGET_500MS

#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop) \
    name.timer500ms();

#endif

#ifndef STEPPER_SIMPLE
#define STEPPER_SIMPLE(name, stepPin, dirPin, enablePin, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC2130_HW_SPI
#define STEPPER_TMC2130_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC5160_HW_SPI
#define STEPPER_TMC5160_HW_SPI(name, stepPin, dirPin, enablePin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC2130_SW_SPI
#define STEPPER_TMC2130_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC5160_SW_SPI
#define STEPPER_TMC5160_SW_SPI(name, stepPin, dirPin, enablePin, mosiPin, misoPin, sckPin, csPin, rsense, chainPos, microsteps, currentMillis, stealth, hybridSpeed, stallSensitivity, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC2208_HW_UART
#define STEPPER_TMC2208_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC2209_HW_UART
#define STEPPER_TMC2209_HW_UART(name, stepPin, dirPin, enablePin, serial, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop)
#endif
#ifndef STEPPER_TMC2209_SW_UART
#define STEPPER_TMC2209_SW_UART(name, stepPin, dirPin, enablePin, rxPin, txPin, rsense, microsteps, currentMillis, stealth, hybridSpeed, slaveAddr, stallSensitivity, fclk, minEndstop, maxEndstop)
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
