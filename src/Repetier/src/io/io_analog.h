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

/**
 Create a AnalogInput variable which returns a analog value
 for channel. Oversample defines how many samples will be used
 to build a stable average. Value 0-5 are good. Lower values
 have less oversampling.

 IO_ANALOG_INPUT(name, channel, oversample)
 */
#ifndef IO_TARGET
#error You need to set IO_TARGET before calling this file!
#endif

#undef IO_ANALOG_INPUT

#if IO_TARGET == IO_TARGET_INIT // hardware init

#define IO_ANALOG_INPUT(name, channel, oversample) \
    HAL::analogEnable(channel);

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // class
class AnalogInput {
public:
    virtual int get() = 0;
};

#define IO_ANALOG_INPUT(name, channel, oversample) \
    class name##Class : public AnalogInput { \
    public: \
        int32_t count; \
        int32_t sum; \
        int32_t minVal, maxVal; \
        int value; \
        name##Class() \
            : count((1 << oversample) + 2) \
            , sum(0) \
            , minVal(100000) \
            , maxVal(0) \
            , value(2048) {} \
        int get() final { return value; } \
    }; \
    extern name##Class name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // variable

#define IO_ANALOG_INPUT(name, channel, oversample) \
    name##Class name;

#elif IO_TARGET == IO_TARGET_ANALOG_INPUT_LOOP // analog read loop

#define IO_ANALOG_INPUT(name, channel, oversample) \
    { \
        int read = HAL::analogRead(channel); \
        name.sum += read; \
        if (read < name.minVal) { \
            name.minVal = read; \
        } \
        if (read > name.maxVal) { \
            name.maxVal = read; \
        } \
        if (--name.count == 0) { \
            name.value = (name.sum - name.minVal - name.maxVal) >> oversample; \
            name.minVal = 100000; \
            name.maxVal = 0; \
            name.sum = 0; \
            name.count = (1 << oversample) + 2; \
        } \
    }

#else

#define IO_ANALOG_INPUT(name, channel, oversample)

#endif
