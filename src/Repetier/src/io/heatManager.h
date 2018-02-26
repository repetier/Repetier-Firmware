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

#if IO_TARGET == 3 // 100ms timer

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod) \
    name.update();
#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax) \
    name.update();

#elif IO_TARGET == 1 // init

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod) \
    name.init();
    
#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax) \
    name.init();

#elif IO_TARGET == 4

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod) \
    extern HeatManagerBangBang name;
#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax) \
    extern HeatManagerPID name;

#elif IO_TARGET == 6

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod) \
    HeatManagerBangBang name(tp, &input, &output, maxTemp, maxPwm, decVariance, decPeriod);

#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax) \
    HeatManagerPID name(tp, &input, &output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax);

#elif IO_TARGET == 10 // restore from config

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod);
#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax) \
    name.resetFromConfig(maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax);

#else

#define HEAT_MANAGER_BANG_BANG(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod)
#define HEAT_MANAGER_PID(tp, name, input, output, maxTemp, maxPwm, decVariance, decPeriod, p, i, d, driveMin, driveMax)

#endif
