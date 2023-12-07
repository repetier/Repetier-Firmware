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
#undef IO_TEMPERATURE_BETA
#undef IO_TEMPERATURE_TABLE_PERMANENT_ERROR
#undef IO_TEMPERATURE_BETA_PERMANENT_ERROR
#undef IO_TEMPERATURE_LINEAR_ANALOG
#undef IO_TEMPERATURE_LINEAR_ANALOG_PERMANENT_ERROR
#undef IO_TEMPERATURE_MAX31855
#undef IO_TEMPERATURE_MAX31865
#undef IO_TEMPERATURE_MAX6675
#undef IO_TEMPERATURE_FAKE
#undef IO_HOTTEST_OF_2
#undef IO_COOLEST_OF_2
#undef IO_FIRST_WORKING_OF_2
#undef IO_TEMPERATURE_OF_HEATER_SET

#if IO_TARGET == IO_TARGET_INIT // hardware init

#define IO_TEMP_TABLE_NTC(name, dataname)
#define IO_TEMP_TABLE_PTC(name, dataname)
#define IO_TEMPERATURE_TABLE(name, analog, table)
#define IO_TEMPERATURE_BETA(name, analog, beta, seriesResistance, thermistorR25, cCoefficient)
#define IO_TEMPERATURE_TABLE_PERMANENT_ERROR(name, analog, table)
#define IO_TEMPERATURE_BETA_PERMANENT_ERROR(name, analog, beta, seriesResistance, thermistorR25, cCoefficient)
#define IO_TEMPERATURE_MAX31855(name, spiDriver)
#define IO_TEMPERATURE_MAX31865(name, spiDriver, wires, RTDnominal, refResistor) \
    name.init();
#define IO_TEMPERATURE_MAX6675(name, spiDriver)

#define IO_HOTTEST_OF_2(name, temp1, temp2)
#define IO_COOLEST_OF_2(name, temp1, temp2)

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // class

class IOTemperatureTable {
public:
    float interpolateNTC(int value, fast8_t num, const short* temptable);
    float interpolatePTC(int value, fast8_t num, const short* temptable);
    virtual float interpolateFor(int value) = 0; // Converts 0..4095 input into a temperature
};

#define IO_TEMP_TABLE_NTC(name, dataname) \
    extern const short name##_table[NUM_##dataname][2] PROGMEM; \
    class name##Class : public IOTemperatureTable { \
    public: \
        float interpolateFor(int value) final { \
            return interpolateNTC(value, NUM_##dataname, (const short*)&name##_table[0][0]); \
        } \
    }; \
    extern name##Class name;

#define IO_TEMP_TABLE_PTC(name, dataname) \
    extern const short name##_table[NUM_##dataname][2] PROGMEM; \
    class name##Class : public IOTemperatureTable { \
    public: \
        float interpolateFor(int value) final { \
            return interpolatePTC(value, NUM_##dataname, (const short*)&name##_table[0][0]); \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_TABLE(name, analog, table) \
    class name##Class : public IOTemperature { \
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

#define IO_TEMPERATURE_LINEAR_ANALOG(name, analog, maxVoltage, offset, vPerKelvin) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            return (static_cast<float>(analog.get() / 4095.0) - offset) / vPerKelvin; \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            return a < 20 || a > 4075; \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_LINEAR_ANALOG_PERMANENT_ERROR(name, analog, maxVoltage, offset, vPerKelvin) \
    class name##Class : public IOTemperature { \
        bool hadError = false; \
\
    public: \
        float get() { \
            return (static_cast<float>(analog.get() / 4095.0) - offset) / vPerKelvin; \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            hadError |= a < 20 || a > 4075; \
            return hadError; \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_BETA(name, analog, beta, seriesResistance, thermistorR25, cCoefficient) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            constexpr float invBeta = 1.0f / beta; \
            constexpr float invRoom = (1.0f / (25.0f - (-273.15))); \
            constexpr float logR25 = logf(thermistorR25); \
            constexpr float alpha = invRoom - invBeta * logR25 - cCoefficient * logR25 * logR25 * logR25; \
            int aRead = analog.get(); \
            aRead = aRead > 4094 ? 4094 : aRead < 1 ? 1 : aRead; \
            float logResis = logf(seriesResistance * aRead / (4095 - aRead)); \
            float steinharthart = alpha + invBeta * logResis; \
            if (cCoefficient) { \
                steinharthart += cCoefficient * (logResis * logResis * logResis); \
            } \
            return ((1.0f / steinharthart) - 273.15f); \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            return a < 20 || a > 4075; \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_TABLE_PERMANENT_ERROR(name, analog, table) \
    class name##Class : public IOTemperature { \
        bool hadError = false; \
\
    public: \
        float get() { \
            return table.interpolateFor(analog.get()); \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            hadError |= a < 20 || a > 4075; \
            return hadError; \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_BETA_PERMANENT_ERROR(name, analog, beta, seriesResistance, thermistorR25, cCoefficient) \
    class name##Class : public IOTemperature { \
        bool hadError = false; \
\
    public: \
        float get() { \
            constexpr float invBeta = 1.0f / beta; \
            constexpr float invRoom = (1.0f / (25.0f - (-273.15))); \
            constexpr float logR25 = logf(thermistorR25); \
            constexpr float alpha = invRoom - invBeta * logR25 - cCoefficient * logR25 * logR25 * logR25; \
            int aRead = analog.get(); \
            aRead = aRead > 4094 ? 4094 : aRead < 1 ? 1 : aRead; \
            float logResis = logf(seriesResistance * aRead / (4095 - aRead)); \
            float steinharthart = alpha + invBeta * logResis; \
            if (cCoefficient) { \
                steinharthart += cCoefficient * (logResis * logResis * logResis); \
            } \
            return ((1.0f / steinharthart) - 273.15f); \
        } \
        bool isDefect() { \
            int a = analog.get(); \
            hadError |= a < 20 || a > 4075; \
            return hadError; \
        } \
    }; \
    extern name##Class name;

// https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf
#define IO_TEMPERATURE_MAX31855(name, spiDriver) \
    class name##Class : public IOTemperature { \
        int8_t errors; \
        float temp; \
        int32_t getData() { \
            spiDriver.begin(); \
            HAL::delayMicroseconds(1); \
            uint32_t data = spiDriver.transfer32(0); \
            spiDriver.end(); \
            return data; \
        } \
\
    public: \
        name##Class() \
            : errors(0) \
            , temp(0) { } \
        float get() { \
            if (errors >= 0) { \
                isDefect(); \
            } \
            if (errors < 0) { \
                errors = 0; \
            } \
            return temp; \
        } \
        bool isDefect() { \
            uint32_t data = getData(); \
            if (data & 65536) { \
                if (errors > 1) { \
                    return true; \
                } \
                errors++; \
                return false; \
            } else { \
                data = data >> 18; \
                int32_t temperature; \
                temperature = data & 0x00001FFF; \
                if (data & 0x00002000) { \
                    data = ~data; \
                    temperature = -1 * ((data & 0x00001FFF) + 1); \
                } \
                errors = -1; \
                temp = static_cast<float>(temperature) * 0.25f; \
            } \
            return false; \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_MAX31865(name, spiDriver, wires, RTDnominal, refResistor) \
    class name##Class : public IOTemperature { \
        int8_t errors; \
        float temp; \
        uint8_t fault; \
\
    public: \
        name##Class() \
            : errors(0) \
            , temp(0) \
            , fault(0) { } \
        float get() { \
            if (errors >= 0) { \
                isDefect(); \
            } \
            if (errors < 0) { \
                errors = 0; \
            } \
            return temp; \
        } \
        bool isDefect() { \
            temp = temperature(); \
            if (fault) { \
                Com::printFLN(PSTR("MAX31865 fault: "), fault); \
                if (errors > 1) { \
                    clearFault(); \
                    return true; \
                } \
                errors++; \
                return false; \
            } \
            return false; \
        } \
        void init() { \
            setWires(); \
            enableBias(true); \
            autoConvert(true); \
            setThresholds(0, 0xFFFF); \
            clearFault(); \
        } \
        uint8_t readFault(max31865_fault_cycle_t fault_cycle = MAX31865_FAULT_AUTO) { \
            if (fault_cycle) { \
                uint8_t cfg_reg = readRegister8(MAX31865_CONFIG_REG); \
                cfg_reg &= 0x11; \
                switch (fault_cycle) { \
                case MAX31865_FAULT_AUTO: \
                    writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10000100)); \
                    HAL::delayMicroseconds(1000); \
                    break; \
                case MAX31865_FAULT_MANUAL_RUN: \
                    writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001000)); \
                    return 0; \
                case MAX31865_FAULT_MANUAL_FINISH: \
                    writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001100)); \
                    return 0; \
                case MAX31865_FAULT_NONE: \
                default: \
                    break; \
                } \
            } \
            return readRegister8(MAX31865_FAULTSTAT_REG); \
        } \
        void clearFault() { \
            uint8_t t = (readRegister8(MAX31865_CONFIG_REG) & ~0x2C) | 2; \
            writeRegister8(MAX31865_CONFIG_REG, t); \
            fault = 0; \
        } \
        void enableBias(bool b) { \
            uint8_t t = readRegister8(MAX31865_CONFIG_REG); \
            if (b) { \
                t |= MAX31865_CONFIG_BIAS; \
            } else { \
                t &= ~MAX31865_CONFIG_BIAS; \
            } \
            writeRegister8(MAX31865_CONFIG_REG, t); \
        } \
        void autoConvert(bool b) { \
            uint8_t t = readRegister8(MAX31865_CONFIG_REG); \
            if (b) { \
                t |= MAX31865_CONFIG_MODEAUTO; \
            } else { \
                t &= ~MAX31865_CONFIG_MODEAUTO; \
            } \
            writeRegister8(MAX31865_CONFIG_REG, t); \
        } \
        void setThresholds(uint16_t lower, uint16_t upper) { \
            writeRegister8(MAX31865_LFAULTLSB_REG, lower & 0xFF); \
            writeRegister8(MAX31865_LFAULTMSB_REG, lower >> 8); \
            writeRegister8(MAX31865_HFAULTLSB_REG, upper & 0xFF); \
            writeRegister8(MAX31865_HFAULTMSB_REG, upper >> 8); \
        } \
        void setWires() { \
            uint8_t t = readRegister8(MAX31865_CONFIG_REG); \
            if (wires == 3) { \
                t |= MAX31865_CONFIG_3WIRE; \
            } else { \
                t &= ~MAX31865_CONFIG_3WIRE; \
            } \
            writeRegister8(MAX31865_CONFIG_REG, t); \
        } \
        float temperature() { \
            float Rt, temp; \
            Rt = static_cast<float>(readRTD()) / 32768.0f * refResistor; \
            uint8_t t = readRegister8(MAX31865_CONFIG_REG); \
            temp = (RTD_A * RTD_A - (4.0 * RTD_B)) + (((4.0 * RTD_B) / RTDnominal) * Rt); \
            temp = (sqrt(temp) - RTD_A) / (2.0 * RTD_B); \
            if (temp >= 0.0) \
                return temp; \
            Rt /= RTDnominal; \
            Rt *= 100; \
            float rpoly = Rt; \
            temp = -242.02; \
            temp += 2.2228 * rpoly; \
            rpoly *= Rt; \
            temp += 2.5859e-3 * rpoly; \
            rpoly *= temp -= 4.8260e-6 * rpoly; \
            rpoly *= Rt; \
            temp -= 2.8183e-8 * rpoly; \
            rpoly *= Rt; \
            temp += 1.5243e-10 * rpoly; \
            return temp; \
        } \
        uint16_t readRTD() { \
            uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG); \
            if (rtd & 1) { \
                fault = readFault(MAX31865_FAULT_NONE); \
                clearFault(); \
            } \
            return rtd >> 1; \
        } \
        uint8_t readRegister8(uint8_t addr) { \
            uint8_t ret = 0; \
            readRegisterN(addr, &ret, 1); \
            return ret; \
        } \
        uint16_t readRegister16(uint8_t addr) { \
            uint8_t buffer[2] = { 0, 0 }; \
            readRegisterN(addr, buffer, 2); \
            return (static_cast<uint16_t>(buffer[0]) << 8) + buffer[1]; \
        } \
        void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) { \
            spiDriver.begin(); \
            spiDriver.transfer(addr & 0x7F); \
            for (int i = 0; i < n; i++) \
                buffer[i] = spiDriver.transfer(0); \
            spiDriver.end(); \
        } \
        void writeRegister8(uint8_t addr, uint8_t data) { \
            spiDriver.begin(); \
            spiDriver.transfer(addr | 0x80); \
            spiDriver.transfer(data); \
            spiDriver.end(); \
        } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_MAX6675(name, spiDriver) \
    class name##Class : public IOTemperature { \
        int8_t errors; \
        millis_t lastRun; \
        float temp; \
        int16_t getData() { \
            spiDriver.begin(); \
            HAL::delayMicroseconds(1); \
            uint16_t data = spiDriver.transfer16(0); \
            spiDriver.end(); \
            return data; \
        } \
\
    public: \
        name##Class() \
            : errors(0) \
            , lastRun(0) \
            , temp(0) { } \
        float get() { \
            if (errors == 0) { \
                isDefect(); \
            } \
            if (errors < 0) { \
                errors = 0; \
            } \
            return temp; \
        } \
        bool isDefect() { \
            if (HAL::timeInMilliseconds() - lastRun < 230) { \
                return errors > 1; \
            } \
            uint16_t data = getData(); \
            if (data & 4) { \
                if (errors > 1) { \
                    return true; \
                } \
                errors++; \
                return false; \
            } else { \
                data = data >> 3; \
                errors = -1; \
                temp = static_cast<float>(data & 4095) * 0.25f; \
            } \
            return false; \
        } \
    }; \
    extern name##Class name;
#define IO_TEMPERATURE_FAKE(name, fakeTemp) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            return fakeTemp; \
        } \
        bool isDefect() { return false; } \
    }; \
    extern name##Class name;

#define IO_TEMPERATURE_OF_HEATER_SET(name, heater) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            return heater.getTargetTemperature(); \
        } \
        bool isDefect() { return false; } \
    }; \
    extern name##Class name;

#define IO_HOTTEST_OF_2(name, temp1, temp2) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            return RMath::max(temp1.get(), temp2.get()); \
        } \
        bool isDefect() { return temp1.isDefect() || temp2.isDefect(); } \
    }; \
    extern name##Class name;

#define IO_COOLEST_OF_2(name, temp1, temp2) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            return RMath::min(temp1.get(), temp2.get()); \
        } \
        bool isDefect() { return temp1.isDefect() || temp2.isDefect(); } \
    }; \
    extern name##Class name;

#define IO_FIRST_WORKING_OF_2(name, temp1, temp2) \
    class name##Class : public IOTemperature { \
    public: \
        float get() { \
            if (!temp1.isDefect()) { \
                return temp1.get(); \
            } \
            return temp2.get(); \
        } \
        bool isDefect() { return temp1.isDefect() && temp2.isDefect(); } \
    }; \
    extern name##Class name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // variable

#define IO_TEMP_TABLE_NTC(name, dataname) \
    const short name##_table[NUM_##dataname][2] PROGMEM = { dataname }; \
    name##Class name;

#define IO_TEMP_TABLE_PTC(name, dataname) \
    const short name##_table[NUM_##dataname][2] PROGMEM = { dataname }; \
    name##Class name;

#define IO_TEMPERATURE_TABLE(name, analog, table) \
    name##Class name;

#define IO_TEMPERATURE_LINEAR_ANALOG(name, analog, maxVoltage, offset, vPerKelvin) \
    name##Class name;

#define IO_TEMPERATURE_LINEAR_ANALOG_PERMANENT_ERROR(name, analog, maxVoltage, offset, vPerKelvin) \
    name##Class name;

#define IO_TEMPERATURE_BETA(name, analog, beta, seriesResistance, thermistorR25, cCoefficient) \
    name##Class name;

#define IO_TEMPERATURE_TABLE_PERMANENT_ERROR(name, analog, table) \
    name##Class name;

#define IO_TEMPERATURE_BETA_PERMANENT_ERROR(name, analog, beta, seriesResistance, thermistorR25, cCoefficient) \
    name##Class name;

#define IO_TEMPERATURE_MAX31855(name, spiDriver) \
    name##Class name;

#define IO_TEMPERATURE_MAX31865(name, spiDriver, wires, RTDnominal, refResistor) \
    name##Class name;

#define IO_TEMPERATURE_MAX6675(name, spiDriver) \
    name##Class name;

#define IO_TEMPERATURE_FAKE(name, fakeTemp) \
    name##Class name;

#define IO_TEMPERATURE_OF_HEATER_SET(name, heater) \
    name##Class name;

#define IO_HOTTEST_OF_2(name, temp1, temp2) \
    name##Class name;

#define IO_COOLEST_OF_2(name, temp1, temp2) \
    name##Class name;

#define IO_FIRST_WORKING_OF_2(name, temp1, temp2) \
    name##Class name;

#endif

#ifndef IO_TEMP_TABLE_NTC
#define IO_TEMP_TABLE_NTC(name, dataname)
#endif
#ifndef IO_TEMP_TABLE_PTC
#define IO_TEMP_TABLE_PTC(name, dataname)
#endif
#ifndef IO_TEMPERATURE_TABLE
#define IO_TEMPERATURE_TABLE(name, analog, table)
#endif
#ifndef IO_TEMPERATURE_LINEAR_ANALOG_PERMANENT_ERROR
#define IO_TEMPERATURE_LINEAR_ANALOG_PERMANENT_ERROR(name, analog, maxVoltage, offset, vPerKelvin)
#endif
#ifndef IO_TEMPERATURE_LINEAR_ANALOG
#define IO_TEMPERATURE_LINEAR_ANALOG(name, analog, maxVoltage, offset, vPerKelvin)
#endif
#ifndef IO_TEMPERATURE_BETA
#define IO_TEMPERATURE_BETA(name, analog, beta, seriesResistance, thermistorR25, cCoefficient)
#endif
#ifndef IO_TEMPERATURE_TABLE_PERMANENT_ERROR
#define IO_TEMPERATURE_TABLE_PERMANENT_ERROR(name, analog, table)
#endif
#ifndef IO_TEMPERATURE_BETA_PERMANENT_ERROR
#define IO_TEMPERATURE_BETA_PERMANENT_ERROR(name, analog, beta, seriesResistance, thermistorR25, cCoefficient)
#endif
#ifndef IO_TEMPERATURE_MAX31855
#define IO_TEMPERATURE_MAX31855(name, spiDriver)
#endif
#ifndef IO_TEMPERATURE_MAX31865
#define IO_TEMPERATURE_MAX31865(name, spiDriver, wires, RTDnominal, refResistor)
#endif
#ifndef IO_TEMPERATURE_MAX6675
#define IO_TEMPERATURE_MAX6675(name, spiDriver)
#endif
#ifndef IO_TEMPERATURE_FAKE
#define IO_TEMPERATURE_FAKE(name, fakeTemp)
#endif
#ifndef IO_HOTTEST_OF_2
#define IO_HOTTEST_OF_2(name, temp1, temp2)
#endif
#ifndef IO_COOLEST_OF_2
#define IO_COOLEST_OF_2(name, temp1, temp2)
#endif
#ifndef IO_FIRST_WORKING_OF_2
#define IO_FIRST_WORKING_OF_2(name, temp1, temp2)
#endif
#ifndef IO_TEMPERATURE_OF_HEATER_SET
#define IO_TEMPERATURE_OF_HEATER_SET(name, heater)
#endif
