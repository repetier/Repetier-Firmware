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

#undef COOLER_MANAGER_HEATER
#undef COOLER_MANAGER_SENSOR
#undef COOLER_MANAGER_MOTORS

#if IO_TARGET == IO_TARGET_500MS // 500ms timer

#define COOLER_MANAGER_HEATER(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    name.update();
#define COOLER_MANAGER_SENSOR(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    name.update();
#define COOLER_MANAGER_MOTORS(name, pwm, offPWM, onPWM, offDelaySeconds) \
    name.update();

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION

#define COOLER_MANAGER_HEATER(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    extern CoolerManagerHeater name;
#define COOLER_MANAGER_SENSOR(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    extern CoolerManagerSensor name;
#define COOLER_MANAGER_MOTORS(name, pwm, offPWM, onPWM, offDelaySeconds) \
    extern CoolerManagerMotors name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES

#define COOLER_MANAGER_HEATER(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    CoolerManagerHeater name(&heater, &pwm, minTemp, maxTemp, minPWM, maxPWM);
#define COOLER_MANAGER_SENSOR(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM) \
    CoolerManagerSensor name(&heater, &pwm, minTemp, maxTemp, minPWM, maxPWM);
#define COOLER_MANAGER_MOTORS(name, pwm, offPWM, onPWM, offDelaySeconds) \
    CoolerManagerMotors name(&pwm, offPWM, onPWM, offDelaySeconds);

#else

#define COOLER_MANAGER_HEATER(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM)
#define COOLER_MANAGER_SENSOR(name, heater, pwm, minTemp, maxTemp, minPWM, maxPWM)
#define COOLER_MANAGER_MOTORS(name, pwm, offPWM, onPWM, offDelaySeconds)

#endif
