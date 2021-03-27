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

#pragma once

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and tool chains,
  all hardware related code should be packed into the hal files.

  Timer usage Mega 2560:
  Timer 0: Time measurement, software pwm 
  Timer 1: Motion3 Stepper interrupt
  Timer 2: Motion2 Stepper interrupt
  Timer 3: Servos
*/

#include <avr/pgmspace.h>
#include <avr/io.h>

#define INLINE __attribute__((always_inline))
#define CPU_ARCH ARCH_AVR
#include <avr/io.h>

#define PACK

#define FSTRINGVALUE(var, value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <avr/eeprom.h>
#include <avr/wdt.h>

// Which I2C port to use?
#ifndef NO_I2C
#ifndef WIRE_PORT
#define WIRE_PORT Wire
#endif
#else
#undef WIRE_PORT
#endif

#define PWM_CLOCK_FREQ 3000
#define PWM_COUNTER_100MS PWM_CLOCK_FREQ / 10

#define ANALOG_PRESCALER _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)
#define MAX_ANALOG_INPUTS 16
extern bool analogEnabled[MAX_ANALOG_INPUTS];
extern uint16_t analogValues[MAX_ANALOG_INPUTS];

#include <inttypes.h>
#include "Stream.h"
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 128
#endif

#include "Arduino.h"
#if defined(WIRE_PORT)
#include <Wire.h>
#endif
#include "fastio.h"
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
        if (!later) {
            cli();
        }
    }

    inline ~InterruptProtectedBlock() {
        SREG = sreg;
    }
};

#define SECONDS_TO_TICKS(s) (unsigned long)(s * (float)F_CPU)

#define MAX_RAM 32767

#define bit_clear(x, y) x &= ~(1 << y) //cbi(x,y)
#define bit_set(x, y) x |= (1 << y)    //sbi(x,y)

typedef uint16_t speed_t;
typedef uint32_t ticks_t;
typedef uint32_t millis_t;
typedef uint8_t flag8_t;
typedef int8_t fast8_t;
typedef uint8_t ufast8_t;

#define FAST_INTEGER_SQRT
#define SERIAL_BUFFER_SIZE 128
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

struct ring_buffer {
    uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
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
    ring_buffer _rx_buffer;
    ring_buffer_tx _tx_buffer;
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
    RFHardwareSerial(volatile uint8_t* ubrrh, volatile uint8_t* ubrrl,
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
extern RFHardwareSerial Serial;
#ifndef RFSERIAL
#define RFSERIAL Serial
#endif
//extern ring_buffer tx_buffer;
#define RFSERIAL Serial
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#if BLUETOOTH_SERIAL == 1
#define RFSERIAL2 Serial1
extern RFHardwareSerial Serial1;
#elif BLUETOOTH_SERIAL == 2
#define RFSERIAL2 Serial2
extern RFHardwareSerial Serial2;
#elif BLUETOOTH_SERIAL == 3
#define RFSERIAL2 Serial3
extern RFHardwareSerial Serial3;
#elif BLUETOOTH_SERIAL == 4
#define RFSERIAL2 Serial4
extern RFHardwareSerial Serial4;
#else
#error Unknown Serial class for BLUETOOTH_SERIAL selected!
#endif
#endif

class HAL {
public:
#if FEATURE_WATCHDOG
    static bool wdPinged;
#endif
    static uint8_t i2cError;
    static BootReason startReason;

    HAL();
    virtual ~HAL();

    // Try to initialize pinNumber as hardware PWM. Returns internal
    // id if it succeeds or -1 if it fails. Typical reasons to fail
    // are no pwm support for that pin or an other pin uses same PWM
    // channel.
    static int initHardwarePWM(int pinNumber, uint32_t frequency);
    // Set pwm output to value. id is id from initHardwarePWM.
    static void setHardwarePWM(int id, int value);
    // Set pwm frequency to value. id is id from initHardwarePWM.
    static void setHardwareFrequency(int id, uint32_t frequency);
    // No DAC on megas
    // Initalize hardware DAC control on dacPin if supported.
    // Returns internal id if it succeeds or -1 if it fails.
    static fast8_t initHardwareDAC(fast8_t dacPin) { return -1; }
    // Set the DAC output to value. id is from initHardwareDAC.
    static void setHardwareDAC(fast8_t id, fast8_t value) { }
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
#if EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH
#error For AVR processors only real EEPROM is supported!
#endif
    static inline void eprSetByte(unsigned int pos, uint8_t value) {
        eeprom_write_byte((uint8_t*)(pos), value);
    }
    static inline void eprSetInt16(unsigned int pos, int16_t value) {
        eeprom_write_word((uint16_t*)(pos), value);
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
        return static_cast<int16_t>(eeprom_read_word((uint16_t*)(pos)));
    }
    static inline int32_t eprGetInt32(unsigned int pos) {
        return static_cast<int32_t>(eeprom_read_dword((uint32_t*)(pos)));
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
    static inline int16_t readFlashWord(const int16_t* ptr) {
        return pgm_read_word(ptr);
    }
    static inline const void* readFlashAddress(const void* adr) {
        return (const void*)pgm_read_word(adr);
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
    static void handlePeriodical();
    static void updateStartReason();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    static void spiInit(); // only called once to initialize for spi usage
    static void spiBegin(uint32_t clock, uint8_t mode, uint8_t msbfirst);
    static uint8_t spiTransfer(uint8_t);
    static void spiEnd();

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
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo, int ms, uint16_t autoOff);
    static void analogStart();
    static void analogEnable(int channel);
    static int analogRead(int channel) { return analogValues[channel]; }
    static void reportHALDebug() { }
    static void switchToBootMode();

protected:
private:
};

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
