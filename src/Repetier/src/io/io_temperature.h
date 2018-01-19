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

/* 

Thermocouples require a table for conversion form analog measured value
into a temperature. These tables depend on the type of thercouple.
Most are NTC type where resistence get lower on higher temperature,
but there are also some PTC types. So make sure to select the right
table type.

IO_TEMP_TABLE_NTC(name, tablename)
IO_TEMP_TABLE_PTC(name, tablename)

Report a temperature to caller. This can be a simple
conversion of an analog measured voltage or the result of
a call to a chip with SPI or I2C.

// Convert analog signal using a conversion table
IO_TEMPERATURE_TABLE(name,analog,table)
*/

#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_TEMP_TABLE_NTC
#undef IO_TEMP_TABLE_PTC
#undef IO_TEMPERATURE_TABLE

#if IO_TARGET == 1 // hardware init

#define IO_TEMP_TABLE_NTC(name, dataname)
#define IO_TEMP_TABLE_PTC(name, dataname)
#define IO_TEMPERATURE_TABLE(name, analog, table)

#elif IO_TARGET == 4 // class

class IOTemperature {
    public:
    virtual float get() = 0; /// Return current temperature
    virtual bool isDefect() = 0; /// Return true if sensor is defect
};

class IOTemperatureTable {
public:
    float interpolateNTC(int value, fast8_t num, const short *temptable);
    float interpolatePTC(int value, fast8_t num, const short *temptable);
    virtual float interpolateFor(int value) = 0; // Converts 0..4095 input into a temperature
};

#define IO_TEMP_TABLE_NTC(name, dataname) \
    extern const short dataname##_table[NUM_##dataname][2] PROGMEM; \
    class name##Class: public IOTemperatureTable {\
    public:\
        float interpolateFor(int value) final { \
            return interpolateNTC(value,NUM_##dataname,(const short *)&dataname##_table[0][0]); \
        } \
    }; \
    extern name##Class name;

#define IO_TEMP_TABLE_PTC(name, dataname) \
    extern const short dataname##_table[NUM_##dataname][2] PROGMEM; \
    class name##Class: public IOTemperatureTable {\
    public:\
        float interpolateFor(int value) final { \
            return  interpolatePTC(value,NUM_##dataname,(const short *)&dataname##_table[0][0]); \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_TABLE(name, analog, table) \
    class name##Class: public IOTemperature { \
    public: \
        float get() { \
            return table.interpolateFor(analog.get()); \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            return a < 20 || a > 4075; \
        } \
    }; \
    extern name##Class name;

#elif IO_TARGET == 6 // variable

#define IO_TEMP_TABLE_NTC(name, dataname) \
    const short dataname##_table[NUM_##dataname][2] PROGMEM = {dataname}; \
    name##Class name;

#define IO_TEMP_TABLE_PTC(name, dataname) \
    const short dataname##_table[NUM_##dataname][2] PROGMEM = {dataname}; \
    name##Class name;

#define IO_TEMPERATURE_TABLE(name,analog,table) \
    name##Class name;

#else

#define IO_TEMP_TABLE_NTC(name, dataname)
#define IO_TEMP_TABLE_PTC(name, dataname)
#define IO_TEMPERATURE_TABLE(name,analog,table)

#endif
