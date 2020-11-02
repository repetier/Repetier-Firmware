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

#undef HEAT_MANAGER_BANG_BANG
#undef HEAT_MANAGER_PID
#undef HEAT_MANAGER_DYN_DEAD_TIME
#undef HEAT_MANAGER_PELTIER_PID
#undef HEAT_MANAGER_DEFINE_HYSTERESIS

#if IO_TARGET == IO_TARGET_PERIODICAL_ACTIONS // periodical action

#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable) \
    name.update();
#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable) \
    name.update();
#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    name.update();
#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable) \
    name.update();

#elif IO_TARGET == IO_TARGET_INIT // init

#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable) \
    name.init();

#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable) \
    name.init();

#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    name.init();

#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable) \
    name.init();

#define HEAT_MANAGER_DEFINE_HYSTERESIS(name, hysteresisTemperature, hysteresisTime, maxWait) \
    name.initHysteresis(hysteresisTemperature, hysteresisTime, maxWait);

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION

#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable) \
    extern HeatManagerBangBang name;
#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable) \
    extern HeatManagerPID name;
#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    extern HeatManagerPeltierPID<flowPin, pType, minTemp> name;
#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable) \
    extern HeatManagerDynDeadTime name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable) \
    HeatManagerBangBang name(tp, index, &input, &output, maxTemp, maxPwm, 1000, decVariance, decPeriod, hotPlugable);

#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable) \
    HeatManagerPID name(tp, index, &input, &output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable);

#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    HeatManagerPeltierPID<flowPin, pType, minTemp> name(tp, index, &input, &output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable);

#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable) \
    HeatManagerDynDeadTime name(tp, index, &input, &output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable);

#elif IO_TARGET == IO_TARGET_RESTORE_FROM_CONFIG // restore from config

#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod);
#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax);
#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax);
#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2);
#define HEAT_MANAGER_DEFINE_HYSTERESIS(name, hysteresisTemperature, hysteresisTime, maxWait) \
    name.initHysteresis(hysteresisTemperature, hysteresisTime, maxWait);

#elif IO_TARGET == IO_TARGET_TEMPLATES // template definitions in tools.cpp

#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp) \
    template class HeatManagerPeltierPID<flowPin, pType, minTemp>;

#endif

#ifndef HEAT_MANAGER_BANG_BANG
#define HEAT_MANAGER_BANG_BANG(name, tp, index, input, output, maxTemp, maxPwm, decVariance, decPeriod, hotPlugable)
#endif
#ifndef HEAT_MANAGER_PID
#define HEAT_MANAGER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable)
#endif
#ifndef HEAT_MANAGER_PELTIER_PID
#define HEAT_MANAGER_PELTIER_PID(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, p, i, d, driveMin, driveMax, hotPlugable, pType, flowPin, minTemp)
#endif
#ifndef HEAT_MANAGER_DYN_DEAD_TIME
#define HEAT_MANAGER_DYN_DEAD_TIME(name, tp, index, input, output, maxTemp, maxPwm, sampleTime, decVariance, decPeriod, temp1, timeUp1, timeDown1, temp2, timeUp2, timeDown2, hotPlugable)
#endif

#ifndef HEAT_MANAGER_DEFINE_HYSTERESIS
#define HEAT_MANAGER_DEFINE_HYSTERESIS(name, hysteresisTemperature, hysteresisTime, maxWait)
#endif
