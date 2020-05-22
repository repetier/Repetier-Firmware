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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef HAL_H
#define HAL_H

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and tool chains,
  all hardware related code should be packed into the hal files.
*/

#include <avr/pgmspace.h>
#include <avr/io.h>

#define INLINE __attribute__((always_inline))

#if CPU_ARCH == ARCH_AVR
#include <avr/io.h>
#else
#define PROGMEM
#define PGM_P const char*
#define PSTR(s) s
#define pgm_read_byte_near(x) (*(uint8_t*)x)
#define pgm_read_byte(x) (*(uint8_t*)x)
#endif

#define PACK

#define FSTRINGVALUE(var, value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <avr/eeprom.h>
#include <avr/wdt.h>

// Which I2C port to use?
#ifndef WIRE_PORT
#define WIRE_PORT Wire
#endif

#define ANALOG_PRESCALER _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)
#define MAX_ANALOG_INPUTS 16
extern bool analogEnabled[MAX_ANALOG_INPUTS];

#if MOTHERBOARD == 8 || MOTHERBOARD == 88 || MOTHERBOARD == 9 || MOTHERBOARD == 92 || CPU_ARCH != ARCH_AVR
#define EXTERNALSERIAL
#endif
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#undef EXTERNALSERIAL
#define EXTERNALSERIAL
#endif

//#define EXTERNALSERIAL  // Force using Arduino serial
#ifndef EXTERNALSERIAL
#undef HardwareSerial_h
#define HardwareSerial_h // Don't use standard serial console
#endif
#include <inttypes.h>
#include "Stream.h"
#ifdef EXTERNALSERIAL
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#include "Arduino.h"
#include <Wire.h>
#if CPU_ARCH == ARCH_AVR
#include "fastio.h"
#else
#define READ(IO) digitalRead(IO)
#define WRITE(IO, v) digitalWrite(IO, v)
#define SET_INPUT(IO) pinMode(IO, INPUT)
#define SET_OUTPUT(IO) pinMode(IO, OUTPUT)
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

class InterruptProtectedBlock {
    uint8_t sreg;

public:
    inline void protect() {
        cli();
    }

    inline void unprotect() {
        SREG = sreg;
    }

    inline InterruptProtectedBlock(bool later = false) {
        sreg = SREG;
        if (!later)
            cli();
    }

    inline ~InterruptProtectedBlock() {
        SREG = sreg;
    }
};

#define SECONDS_TO_TICKS(s) (unsigned long)(s * (float)F_CPU)
#define ANALOG_INPUT_SAMPLE 5
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#define MAX_RAM 32767

#define bit_clear(x, y) x &= ~(1 << y) //cbi(x,y)
#define bit_set(x, y) x |= (1 << y)    //sbi(x,y)

#if NONLINEAR_SYSTEM

typedef uint16_t speed_t;
typedef uint32_t ticks_t;
typedef uint32_t millis_t;
typedef uint8_t flag8_t;
typedef int8_t fast8_t;
typedef uint8_t ufast8_t;

#define FAST_INTEGER_SQRT

#ifndef EXTERNALSERIAL
// Implement serial communication for one stream only!
/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul

  Modified to use only 1 queue with fixed length by Repetier
*/

#define SERIAL_BUFFER_SIZE 128
#define SERIAL_BUFFER_MASK 127
#undef SERIAL_TX_BUFFER_SIZE
#undef SERIAL_TX_BUFFER_MASK
#ifdef BIG_OUTPUT_BUFFER
#define SERIAL_TX_BUFFER_SIZE 128
#define SERIAL_TX_BUFFER_MASK 127
#else
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_TX_BUFFER_MASK 63
#endif

struct ring_buffer {
    uint8_t buffer[SERIAL_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};
struct ring_buffer_tx {
    uint8_t buffer[SERIAL_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

class RFHardwareSerial : public Stream {
public:
    ring_buffer* _rx_buffer;
    ring_buffer_tx* _tx_buffer;
    volatile uint8_t* _ubrrh;
    volatile uint8_t* _ubrrl;
    volatile uint8_t* _ucsra;
    volatile uint8_t* _ucsrb;
    volatile uint8_t* _udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udrie;
    uint8_t _u2x;

public:
    RFHardwareSerial(ring_buffer* rx_buffer, ring_buffer_tx* tx_buffer,
                     volatile uint8_t* ubrrh, volatile uint8_t* ubrrl,
                     volatile uint8_t* ucsra, volatile uint8_t* ucsrb,
                     volatile uint8_t* udr,
                     uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x);
    void begin(unsigned long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
#ifdef COMPAT_PRE1
    virtual void write(uint8_t);
#else
    virtual size_t write(uint8_t);
#endif
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool();
    int outputUnused(void); // Used for output in interrupts
};
extern RFHardwareSerial RFSerial;
#define RFSERIAL RFSerial
//extern ring_buffer tx_buffer;
#define WAIT_OUT_EMPTY \
    while (tx_buffer.head != tx_buffer.tail) { \
    }
#else
#define RFSERIAL Serial
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#if BLUETOOTH_SERIAL == 1
#define RFSERIAL2 Serial1
#elif BLUETOOTH_SERIAL == 2
#define RFSERIAL2 Serial2
#elif BLUETOOTH_SERIAL == 3
#define RFSERIAL2 Serial3
#elif BLUETOOTH_SERIAL == 4
#define RFSERIAL2 Serial4
#elif BLUETOOTH_SERIAL == 5
#define RFSERIAL2 Serial5
#endif
#endif
#endif

class HAL {
public:
#if FEATURE_WATCHDOG
    static bool wdPinged;
#endif
    static uint8_t i2cError;

    HAL();
    virtual ~HAL();
    static inline void hwSetup(void) { }

    static inline void digitalWrite(uint8_t pin, uint8_t value) {
        ::digitalWrite(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin) {
        return ::digitalRead(pin);
    }
    static inline void pinMode(uint8_t pin, uint8_t mode) {
        ::pinMode(pin, mode);
    }
    static inline void delayMicroseconds(unsigned int delayUs) {
        ::delayMicroseconds(delayUs);
    }
    static inline void delayMilliseconds(unsigned int delayMs) {
        ::delay(delayMs);
    }
    static inline void tone(int duration) {
#if BEEPER_PIN > -1
        ::tone(BEEPER_PIN, duration);
#endif
    }
    static inline void noTone() {
#if BEEPER_PIN > -1
        ::noTone(BEEPER_PIN);
#endif
    }
    static inline void eprSetByte(unsigned int pos, uint8_t value) {
        eeprom_write_byte((unsigned char*)(pos), value);
    }
    static inline void eprSetInt16(unsigned int pos, int16_t value) {
        eeprom_write_word((unsigned int*)(pos), value);
    }
    static inline void eprSetInt32(unsigned int pos, int32_t value) {
        eeprom_write_dword((uint32_t*)(pos), value);
    }
    static inline void eprSetFloat(unsigned int pos, float value) {
        eeprom_write_block(&value, (void*)(pos), 4);
    }
    static inline uint8_t eprGetByte(unsigned int pos) {
        return eeprom_read_byte((unsigned char*)(pos));
    }
    static inline int16_t eprGetInt16(unsigned int pos) {
        return eeprom_read_word((uint16_t*)(pos));
    }
    static inline int32_t eprGetInt32(unsigned int pos) {
        return eeprom_read_dword((uint32_t*)(pos));
    }
    static inline float eprGetFloat(unsigned int pos) {
        float v;
        eeprom_read_block(&v, (void*)(pos), 4); // newer gcc have eeprom_read_block but not arduino 22
        return v;
    }

    // Faster version of InterruptProtectedBlock.
    // For safety it may only be called from within an
    // interrupt handler.
    static inline void allowInterrupts() {
        sei();
    }

    // Faster version of InterruptProtectedBlock.
    // For safety it may only be called from within an
    // interrupt handler.
    static inline void forbidInterrupts() {
        cli();
    }
    static inline millis_t timeInMilliseconds() {
        return millis();
    }
    static inline char readFlashByte(PGM_P ptr) {
        return pgm_read_byte(ptr);
    }
    static inline int16_t readFlashWord(PGM_P ptr) {
        return pgm_read_word(ptr);
    }
    static inline void serialSetBaudrate(long baud) {
        RFSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable() {
        return RFSERIAL.available() > 0;
    }
    static inline uint8_t serialReadByte() {
        return RFSERIAL.read();
    }
    static inline void serialWriteByte(char b) {
        RFSERIAL.write(b);
    }
    static inline void serialFlush() {
        RFSERIAL.flush();
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions
    static void spiBegin(uint8_t ssPin = 0) {
#if SDSS >= 0
        SET_INPUT(MISO_PIN);
        SET_OUTPUT(MOSI_PIN);
        SET_OUTPUT(SCK_PIN);
        // SS must be in output mode even it is not chip select
        SET_OUTPUT(SDSS);
#if SDSSORIG > -1
        SET_OUTPUT(SDSSORIG);
#endif
        // set SS high - may be chip select for another SPI device
#if defined(SET_SPI_SS_HIGH) && SET_SPI_SS_HIGH
        WRITE(SDSS, HIGH);
#endif // SET_SPI_SS_HIGH
#endif
    }
    static inline void spiInit(uint8_t spiRate) {
        uint8_t r = 0;
        for (uint8_t b = 2; spiRate > b && r < 6; b <<= 1, r++)
            ;
        SET_OUTPUT(SS);
        WRITE(SS, HIGH);
        SET_OUTPUT(SCK);
        SET_OUTPUT(MOSI_PIN);
        SET_INPUT(MISO_PIN);
#ifdef PRR
        PRR &= ~(1 << PRSPI);
#elif defined PRR0
        PRR0 &= ~(1 << PRSPI);
#endif
        // See avr processor documentation
        SPCR = (1 << SPE) | (1 << MSTR) | (r >> 1);
        SPSR = (r & 1 || r == 6 ? 0 : 1) << SPI2X;
    }
    static inline uint8_t spiReceive(uint8_t send = 0xff) {
        SPDR = send;
        while (!(SPSR & (1 << SPIF))) {
        }
        return SPDR;
    }
    static inline void spiReadBlock(uint8_t* buf, size_t nbyte) {
        if (nbyte-- == 0)
            return;
        SPDR = 0XFF;
        for (size_t i = 0; i < nbyte; i++) {
            while (!(SPSR & (1 << SPIF))) {
            }
            buf[i] = SPDR;
            SPDR = 0XFF;
        }
        while (!(SPSR & (1 << SPIF))) {
        }
        buf[nbyte] = SPDR;
    }
    static inline void spiSend(uint8_t b) {
        SPDR = b;
        while (!(SPSR & (1 << SPIF))) {
        }
    }
    static inline void spiSend(const uint8_t* buf, size_t n) {
        if (n == 0)
            return;
        SPDR = buf[0];
        if (n > 1) {
            uint8_t b = buf[1];
            size_t i = 2;
            while (1) {
                while (!(SPSR & (1 << SPIF))) {
                }
                SPDR = b;
                if (i == n)
                    break;
                b = buf[i++];
            }
        }
        while (!(SPSR & (1 << SPIF))) {
        }
    }

    static inline __attribute__((always_inline)) void spiSendBlock(uint8_t token, const uint8_t* buf) {
        SPDR = token;
        for (uint16_t i = 0; i < 512; i += 2) {
            while (!(SPSR & (1 << SPIF))) {
            }
            SPDR = buf[i];
            while (!(SPSR & (1 << SPIF))) {
            }
            SPDR = buf[i + 1];
        }
        while (!(SPSR & (1 << SPIF))) {
        }
    }

    // I2C Support

    static void i2cSetClockspeed(uint32_t clockSpeedHz);
    static void i2cInit(uint32_t clockSpeedHz);
    static void i2cStartRead(uint8_t address7bit, uint8_t bytes);
    // static void i2cStart(uint8_t address7bit);
    static void i2cStartAddr(uint8_t address7bit, unsigned int pos, uint8_t readBytes);
    static void i2cStop(void);
    static void i2cWrite(uint8_t data);
    static int i2cRead(void);

    // Watchdog support

    inline static void startWatchdog() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        WDTCSR = (1 << WDCE) | (1 << WDE); // wdt FIX for arduino mega boards
        WDTCSR = (1 << WDIE) | (1 << WDP3);
#else
        wdt_enable(WDTO_4S);
#endif
    };
    inline static void stopWatchdog() {
        wdt_disable();
    }
    inline static void pingWatchdog() {
#if FEATURE_WATCHDOG
        wdPinged = true;
#endif
    };
#if NUM_SERVOS > 0
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo, int ms, uint16_t autoOff);
#endif
    static void analogStart();
    static void reportHALDebug() { }

protected:
private:
};
/*#if MOTHERBOARD==6 || MOTHERBOARD==62 || MOTHERBOARD==7
#if MOTHERBOARD!=7
#define SIMULATE_PWM
#endif
#define MOTION2_TIMER_VECTOR TIMER2_COMPA_vect
#define MOTION2_OCR OCR2A
#define MOTION2_TCCR TCCR2A
#define MOTION2_TIMSK TIMSK2
#define MOTION2_OCIE OCIE2A
#define PWM_TIMER_VECTOR TIMER2_COMPB_vect
#define PWM_OCR OCR2B
#define PWM_TCCR TCCR2B
#define PWM_TIMSK TIMSK2
#define PWM_OCIE OCIE2B
#else*/
#define MOTION2_TIMER_VECTOR TIMER0_COMPA_vect
#define MOTION2_OCR OCR0A
#define MOTION2_TCCR TCCR0A
#define MOTION2_TIMSK TIMSK0
#define MOTION2_OCIE OCIE0A
#define PWM_TIMER_VECTOR TIMER0_COMPB_vect
#define PWM_OCR OCR0B
#define PWM_TCCR TCCR0A
#define PWM_TIMSK TIMSK0
#define PWM_OCIE OCIE0B
//#endif
#endif // HAL_H
