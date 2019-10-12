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

#undef IO_SPI_HW
#undef IO_SPI_SW

#if IO_TARGET == IO_TARGET_INIT // setup

#define IO_SPI_HW(name, frequency, mode, msbfirst, csPin) \
    name.init();
#define IO_SPI_SW(name, delayus, mode, msbfirst, csPin, clkPin, misoPin, mosiPin) \
    name.init();

#elif IO_TARGET == IO_TARGET_CLASS_DEFINITION // class

/**
 * Classes using SPI will get a super easy interface. To start a transfer they
 * simply start with begin() and then exchange data using one of the transfer functions.
 * Communication is stopped by calling end(). The implementation is responsible for setting
 * the right mode and speed in the begin method.
 */
class RFSpiBase {
public:
    virtual void begin() = 0;
    virtual uint8_t transfer(uint8_t data) = 0;
    virtual void end() = 0;
    virtual bool msbFirst() = 0;
    virtual void csDoubleToggle() = 0;
    uint16_t transfer16(uint16_t data) {
        union {
            uint16_t val;
            struct {
                uint8_t lsb;
                uint8_t msb;
            };
        } t;
        t.val = data;
        if (msbFirst()) {
            t.msb = transfer(t.msb);
            t.lsb = transfer(t.lsb);
        } else {
            t.lsb = transfer(t.lsb);
            t.msb = transfer(t.msb);
        }
        return t.val;
    }
    uint32_t transfer32(uint32_t data) {
        union {
            uint32_t val;
            struct {
                uint8_t b0;
                uint8_t b1;
                uint8_t b2;
                uint8_t b3;
            };
        } t;
        t.val = data;
        if (msbFirst()) {
            t.b3 = transfer(t.b3);
            t.b2 = transfer(t.b2);
            t.b1 = transfer(t.b1);
            t.b0 = transfer(t.b0);
        } else {
            t.b0 = transfer(t.b0);
            t.b1 = transfer(t.b1);
            t.b2 = transfer(t.b2);
            t.b3 = transfer(t.b3);
        }
        return t.val;
    }
};

#define IO_SPI_HW(name, frequency, mode, msbfirst, csPin) \
    class name##Class : public RFSpiBase { \
    public: \
        name##Class() { \
        } \
        virtual void init() { \
            SET_OUTPUT(csPin); \
            WRITE(csPin, 1); \
        } \
        virtual void begin() { \
            HAL::spiBegin(frequency, mode, msbfirst); \
            WRITE(csPin, 0); \
        } \
        virtual uint8_t transfer(uint8_t data) { \
            return HAL::spiTransfer(data); \
        } \
        virtual void end() { \
            WRITE(csPin, 1); \
            HAL::spiEnd(); \
        } \
        virtual void csDoubleToggle() { \
            WRITE(csPin, 1); \
            HAL::delayMicroseconds(10); \
            WRITE(csPin, 0); \
        } \
        virtual bool msbFirst() { return msbfirst; } \
    }; \
    extern name##Class name;

#define IO_SPI_SW(name, delayus, mode, msbfirst, csPin, clkPin, misoPin, mosiPin) \
    class name##Class : public RFSpiBase { \
        inline __attribute__((always_inline)) bool modeCPHA() { return (mode & 1) != 0; } \
        inline __attribute__((always_inline)) bool modeCPOL() { return (mode & 2) != 0; } \
\
    public: \
        name##Class() { \
        } \
        virtual void init() { \
            SET_OUTPUT(csPin); \
            WRITE(csPin, 1); \
            SET_OUTPUT(clkPin); \
            if (misoPin >= 0) { \
                HAL::pinMode(misoPin, INPUT); \
            } \
            if (mosiPin >= 0) { \
                HAL::pinMode(mosiPin, OUTPUT); \
            } \
        } \
        virtual void begin() { \
            WRITE(clkPin, modeCPOL()); \
            if (!modeCPHA()) { \
                if (delayus > 0) { \
                    HAL::delayMicroseconds(delayus); \
                } \
            } \
            WRITE(csPin, 0); \
        } \
        virtual void csDoubleToggle() { \
            WRITE(csPin, 1); \
            HAL::delayMicroseconds(10); \
            WRITE(csPin, 0); \
        } \
        virtual uint8_t transfer(uint8_t data) { \
            if (!msbfirst) { \
                uint8_t lsb = 0; \
                for (fast8_t i = 0; i < 8; i++) { \
                    lsb <<= 1; \
                    if (data & 1) { \
                        lsb |= 1; \
                    } \
                    data >>= 1; \
                } \
                data = lsb; \
            } \
            uint8_t rxData = 0; \
            for (fast8_t i = 0; i < 8; i++) { \
                if (modeCPHA()) { \
                    WRITE(clkPin, !modeCPOL()); \
                    if (mosiPin >= 0) { \
                        HAL::digitalWrite(mosiPin, data & 128); \
                        data <<= 1; \
                    } \
                    if (delayus > 0) { \
                        HAL::delayMicroseconds(delayus); \
                    } \
                    WRITE(clkPin, modeCPHA() ? modeCPOL() : !modeCPOL()); \
                    if (misoPin >= 0) { \
                        if (msbfirst) { \
                            rxData <<= 1; \
                            if (HAL::digitalRead(misoPin)) { \
                                rxData |= 1; \
                            } \
                        } else { \
                            rxData >>= 1; \
                            if (HAL::digitalRead(misoPin)) { \
                                rxData |= 128; \
                            } \
                        } \
                    } \
                    if (delayus > 0) { \
                        HAL::delayMicroseconds(delayus); \
                    } \
                } else { \
                    if (mosiPin >= 0) { \
                        HAL::digitalWrite(mosiPin, data & 128); \
                        data <<= 1; \
                    } \
                    if (delayus > 0) { \
                        HAL::delayMicroseconds(delayus); \
                    } \
                    WRITE(clkPin, modeCPHA() ? modeCPOL() : !modeCPOL()); \
                    if (misoPin >= 0) { \
                        if (msbfirst) { \
                            rxData <<= 1; \
                            if (HAL::digitalRead(misoPin)) { \
                                rxData |= 1; \
                            } \
                        } else { \
                            rxData >>= 1; \
                            if (HAL::digitalRead(misoPin)) { \
                                rxData |= 128; \
                            } \
                        } \
                    } \
                    if (delayus > 0) { \
                        HAL::delayMicroseconds(delayus); \
                    } \
                    WRITE(clkPin, modeCPOL()); \
                } \
            } \
            return rxData; \
        } \
        virtual void end() { \
            WRITE(csPin, 1); \
        } \
        virtual bool msbFirst() { return msbfirst; } \
    }; \
    extern name##Class name;

#elif IO_TARGET == IO_TARGET_DEFINE_VARIABLES // variable

#define IO_SPI_HW(name, frequency, mode, msbfirst, csPin) \
    name##Class name;
#define IO_SPI_SW(name, delayus, mode, msbfirst, csPin, clkPin, misoPin, mosiPin) \
    name##Class name;

#else

#define IO_SPI_HW(name, frequency, mode, msbfirst, csPin)
#define IO_SPI_SW(name, delayus, mode, msbfirst, csPin, clkPin, misoPin, mosiPin)

#endif
