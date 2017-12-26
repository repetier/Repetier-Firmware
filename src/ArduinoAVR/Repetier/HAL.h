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
#define PGM_P const char *
#define PSTR(s) s
#define pgm_read_byte_near(x) (*(uint8_t*)x)
#define pgm_read_byte(x) (*(uint8_t*)x)
#endif

#define PACK

#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <avr/eeprom.h>
#include <avr/wdt.h>
/** \brief Prescale factor, timer0 runs at.

All known Arduino boards use 64. This value is needed for the extruder timing. */
#define TIMER0_PRESCALE 64

#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)

#if MOTHERBOARD==8 || MOTHERBOARD==88 || MOTHERBOARD==9 || MOTHERBOARD==92 || CPU_ARCH!=ARCH_AVR
#define EXTERNALSERIAL
#endif
#if NEW_COMMUNICATION && defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#undef EXTERNALSERIAL
#define EXTERNALSERIAL
#endif

//#define EXTERNALSERIAL  // Force using Arduino serial
#ifndef EXTERNALSERIAL
#undef HardwareSerial_h
#define  HardwareSerial_h // Don't use standard serial console
#endif
#include <inttypes.h>
#include "Stream.h"
#ifdef EXTERNALSERIAL
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#if CPU_ARCH==ARCH_AVR
#include "fastio.h"
#else
#define	READ(IO)  digitalRead(IO)
#define	WRITE(IO, v)  digitalWrite(IO, v)
#define	SET_INPUT(IO)  pinMode(IO, INPUT)
#define	SET_OUTPUT(IO)  pinMode(IO, OUTPUT)
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

class InterruptProtectedBlock
{
    uint8_t sreg;
public:
    inline void protect()
    {
        cli();
    }

    inline void unprotect()
    {
        SREG = sreg;
    }

    inline InterruptProtectedBlock(bool later = false)
    {
        sreg = SREG;
        if(!later)
            cli();
    }

    inline ~InterruptProtectedBlock()
    {
        SREG = sreg;
    }
};

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_INPUT_SAMPLE 5
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 10
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#define MAX_RAM 32767

#define bit_clear(x,y) x&= ~(1<<y) //cbi(x,y)
#define bit_set(x,y)   x|= (1<<y)//sbi(x,y)

/** defines the data direction (reading from I2C device) in i2cStart(),i2cRepStart() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2cStart(),i2cRepStart() */
#define I2C_WRITE   0

#if NONLINEAR_SYSTEM
// Maximum speed with 100% interrupt utilization is 27000 Hz at 16MHz cpu
// leave some margin for all the extra transformations. So we keep inside clean timings.
#define LIMIT_INTERVAL ((F_CPU/30000)+1)
#else
#define LIMIT_INTERVAL ((F_CPU/40000)+1)
#endif

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

struct ring_buffer
{
    uint8_t buffer[SERIAL_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};
struct ring_buffer_tx
{
    uint8_t buffer[SERIAL_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
};

class RFHardwareSerial : public Stream
{
public:
    ring_buffer *_rx_buffer;
    ring_buffer_tx *_tx_buffer;
    volatile uint8_t *_ubrrh;
    volatile uint8_t *_ubrrl;
    volatile uint8_t *_ucsra;
    volatile uint8_t *_ucsrb;
    volatile uint8_t *_udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udrie;
    uint8_t _u2x;
public:
    RFHardwareSerial(ring_buffer *rx_buffer, ring_buffer_tx *tx_buffer,
                     volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                     volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                     volatile uint8_t *udr,
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
#define WAIT_OUT_EMPTY while(tx_buffer.head != tx_buffer.tail) {}
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

class HAL
{
public:
#if FEATURE_WATCHDOG
    static bool wdPinged;
#endif
    HAL();
    virtual ~HAL();
    static inline void hwSetup(void)
    {}
    // return val*val
    static uint16_t integerSqrt(uint32_t a);
    /** \brief Optimized division

    Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
    This version is optimized for a 16 bit dividend and recognizes the special cases
    of a 24 bit and 16 bit dividend, which often, but not always occur in updating the
    interval.
    */
    static inline int32_t Div4U2U(uint32_t a,uint16_t b)
    {
#if CPU_ARCH==ARCH_AVR
        // r14/r15 remainder
        // r16 counter
        __asm__ __volatile__ (
            "clr r14 \n\t"
            "sub r15,r15 \n\t"
            "tst %D0 \n\t"
            "brne do32%= \n\t"
            "tst %C0 \n\t"
            "breq donot24%= \n\t"
            "rjmp do24%= \n\t"
            "donot24%=:" "ldi r16,17 \n\t" // 16 Bit divide
            "d16u_1%=:" "rol %A0 \n\t"
            "rol %B0 \n\t"
            "dec r16 \n\t"
            "brne	d16u_2%= \n\t"
            "rjmp end%= \n\t"
            "d16u_2%=:" "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc	d16u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d16u_1%= \n\t"
            "d16u_3%=:" "sec \n\t"
            "rjmp d16u_1%= \n\t"
            "do32%=:" // divide full 32 bit
            "rjmp do32B%= \n\t"
            "do24%=:" // divide 24 bit

            "ldi r16,25 \n\t" // 24 Bit divide
            "d24u_1%=:" "rol %A0 \n\t"
            "rol %B0 \n\t"
            "rol %C0 \n\t"
            "dec r16 \n\t"
            "brne	d24u_2%= \n\t"
            "rjmp end%= \n\t"
            "d24u_2%=:" "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc	d24u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d24u_1%= \n\t"
            "d24u_3%=:" "sec \n\t"
            "rjmp d24u_1%= \n\t"

            "do32B%=:" // divide full 32 bit

            "ldi r16,33 \n\t" // 32 Bit divide
            "d32u_1%=:" "rol %A0 \n\t"
            "rol %B0 \n\t"
            "rol %C0 \n\t"
            "rol %D0 \n\t"
            "dec r16 \n\t"
            "brne	d32u_2%= \n\t"
            "rjmp end%= \n\t"
            "d32u_2%=:" "rol r14 \n\t"
            "rol r15 \n\t"
            "sub r14,%A2 \n\t"
            "sbc r15,%B2 \n\t"
            "brcc	d32u_3%= \n\t"
            "add r14,%A2 \n\t"
            "adc r15,%B2 \n\t"
            "clc \n\t"
            "rjmp d32u_1%= \n\t"
            "d32u_3%=:" "sec \n\t"
            "rjmp d32u_1%= \n\t"

            "end%=:" // end
            :"=&r"(a)
            :"0"(a),"r"(b)
            :"r14","r15","r16"
        );
        return a;
#else
        return a/b;
#endif
    }
    static inline unsigned long U16SquaredToU32(unsigned int val)
    {
        long res;
        __asm__ __volatile__ ( // 15 Ticks
            "mul %A1,%A1 \n\t"
            "movw %A0,r0 \n\t"
            "mul %B1,%B1 \n\t"
            "movw %C0,r0 \n\t"
            "mul %A1,%B1 \n\t"
            "clr %A1 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,%A1 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,%A1 \n\t"
            "clr r1 \n\t"
            : "=&r"(res),"=r"(val)
            : "1"(val)
        );
        return res;
    }
    static inline unsigned int ComputeV(long timer,long accel)
    {
#if CPU_ARCH==ARCH_AVR
        unsigned int res;
        // 38 Ticks
        __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
            // Result LSB first: %A0, %B0, %A1
            "mul %B1,%A2 \n\t"
            "mov %A0,r1 \n\t"
            "mul %B1,%C2 \n\t"
            "mov %B0,r0 \n\t"
            "mov %A1,r1 \n\t"
            "mul %B1,%B2 \n\t"
            "add %A0,r0 \n\t"
            "adc %B0,r1 \n\t"
            "adc %A1,%D2 \n\t"
            "mul %C1,%A2 \n\t"
            "add %A0,r0 \n\t"
            "adc %B0,r1 \n\t"
            "adc %A1,%D2 \n\t"
            "mul %C1,%B2 \n\t"
            "add %B0,r0 \n\t"
            "adc %A1,r1 \n\t"
            "mul %D1,%A2 \n\t"
            "add %B0,r0 \n\t"
            "adc %A1,r1 \n\t"
            "mul %C1,%C2 \n\t"
            "add %A1,r0 \n\t"
            "mul %D1,%B2 \n\t"
            "add %A1,r0 \n\t"
            "lsr %A1 \n\t"
            "ror %B0 \n\t"
            "ror %A0 \n\t"
            "lsr %A1 \n\t"
            "ror %B0 \n\t"
            "ror %A0 \n\t"
            "clr r1 \n\t"
            :"=&r"(res),"=r"(timer),"=r"(accel)
            :"1"(timer),"2"(accel)
            : );
        // unsigned int v = ((timer>>8)*cur->accel)>>10;
        return res;
#else
        return ((timer >> 8) * accel) >> 10;
#endif
    }
// Multiply two 16 bit values and return 32 bit result
    static inline uint32_t mulu16xu16to32(unsigned int a,unsigned int b)
    {
        uint32_t res;
        // 18 Ticks = 1.125 us
        __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
            // Result LSB first: %A0, %B0, %A1
            "clr r18 \n\t"
            "mul %B2,%B1 \n\t" // mul hig bytes
            "movw %C0,r0 \n\t"
            "mul %A1,%A2 \n\t" // mul low bytes
            "movw %A0,r0 \n\t"
            "mul %A1,%B2 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,r18 \n\t"
            "mul %B1,%A2 \n\t"
            "add %B0,r0 \n\t"
            "adc %C0,r1 \n\t"
            "adc %D0,r18 \n\t"
            "clr r1 \n\t"
            :"=&r"(res),"=r"(a),"=r"(b)
            :"1"(a),"2"(b)
            :"r18" );
        // return (long)a*b;
        return res;
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned int mulu6xu16shift16(unsigned int a,unsigned int b)
    {
#if CPU_ARCH == ARCH_AVR
        unsigned int res;
        // 18 Ticks = 1.125 us
        __asm__ __volatile__ ( // 0 = res, 1 = timer, 2 = accel %D2=0 ,%A1 are unused is free
            // Result LSB first: %A0, %B0, %A1
            "clr r18 \n\t"
            "mul %B2,%B1 \n\t" // mul hig bytes
            "movw %A0,r0 \n\t"
            "mul %A1,%A2 \n\t" // mul low bytes
            "mov r19,r1 \n\t"
            "mul %A1,%B2 \n\t"
            "add r19,r0 \n\t"
            "adc %A0,r1 \n\t"
            "adc %B0,r18 \n\t"
            "mul %B1,%A2 \n\t"
            "add r19,r0 \n\t"
            "adc %A0,r1 \n\t"
            "adc %B0,r18 \n\t"
            "clr r1 \n\t"
            :"=&r"(res),"=r"(a),"=r"(b)
            :"1"(a),"2"(b)
            :"r18","r19" );
        return res;
#else
        return ((int32_t)a * b) >> 16;
#endif
    }
    static inline void digitalWrite(uint8_t pin,uint8_t value)
    {
        ::digitalWrite(pin,value);
    }
    static inline uint8_t digitalRead(uint8_t pin)
    {
        return ::digitalRead(pin);
    }
    static inline void pinMode(uint8_t pin,uint8_t mode)
    {
        ::pinMode(pin,mode);
    }
    static int32_t CPUDivU2(unsigned int divisor);
    static inline void delayMicroseconds(unsigned int delayUs)
    {
        ::delayMicroseconds(delayUs);
    }
    static inline void delayMilliseconds(unsigned int delayMs)
    {
        ::delay(delayMs);
    }
    static inline void tone(uint8_t pin,int duration)
    {
        ::tone(pin,duration);
    }
    static inline void noTone(uint8_t pin)
    {
        ::noTone(pin);
    }
    static inline void eprSetByte(unsigned int pos,uint8_t value)
    {
        eeprom_write_byte((unsigned char *)(EEPROM_OFFSET + pos), value);
    }
    static inline void eprSetInt16(unsigned int pos,int16_t value)
    {
        eeprom_write_word((unsigned int*)(EEPROM_OFFSET + pos),value);
    }
    static inline void eprSetInt32(unsigned int pos,int32_t value)
    {
        eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + pos),value);
    }
    static inline void eprSetFloat(unsigned int pos,float value)
    {
        eeprom_write_block(&value,(void*)(EEPROM_OFFSET + pos), 4);
    }
    static inline uint8_t eprGetByte(unsigned int pos)
    {
        return eeprom_read_byte ((unsigned char *)(EEPROM_OFFSET + pos));
    }
    static inline int16_t eprGetInt16(unsigned int pos)
    {
        return eeprom_read_word((uint16_t *)(EEPROM_OFFSET + pos));
    }
    static inline int32_t eprGetInt32(unsigned int pos)
    {
        return eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + pos));
    }
    static inline float eprGetFloat(unsigned int pos)
    {
        float v;
        eeprom_read_block(&v,(void *)(EEPROM_OFFSET + pos),4); // newer gcc have eeprom_read_block but not arduino 22
        return v;
    }

    // Faster version of InterruptProtectedBlock.
    // For safety it may only be called from within an
    // interrupt handler.
    static inline void allowInterrupts()
    {
        sei();
    }

    // Faster version of InterruptProtectedBlock.
    // For safety it may only be called from within an
    // interrupt handler.
    static inline void forbidInterrupts()
    {
        cli();
    }
    static inline millis_t timeInMilliseconds()
    {
        return millis();
    }
    static inline char readFlashByte(PGM_P ptr)
    {
        return pgm_read_byte(ptr);
    }
    static inline int16_t readFlashWord(PGM_P ptr)
    {
        return pgm_read_word(ptr);
    }
    static inline void serialSetBaudrate(long baud)
    {
        RFSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable()
    {
        return RFSERIAL.available() > 0;
    }
    static inline uint8_t serialReadByte()
    {
        return RFSERIAL.read();
    }
    static inline void serialWriteByte(char b)
    {
        RFSERIAL.write(b);
    }
    static inline void serialFlush()
    {
        RFSERIAL.flush();
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions
    static void spiBegin(uint8_t ssPin = 0)
    {
#if SDSS >= 0
        SET_INPUT(MISO_PIN);
        SET_OUTPUT(MOSI_PIN);
        SET_OUTPUT(SCK_PIN);
        // SS must be in output mode even it is not chip select
        SET_OUTPUT(SDSS);
#if SDSSORIG >- 1
        SET_OUTPUT(SDSSORIG);
#endif
        // set SS high - may be chip select for another SPI device
#if defined(SET_SPI_SS_HIGH) && SET_SPI_SS_HIGH
        WRITE(SDSS, HIGH);
#endif  // SET_SPI_SS_HIGH
#endif
    }
    static inline void spiInit(uint8_t spiRate)
    {
         uint8_t r = 0;
         for (uint8_t b = 2; spiRate > b && r < 6; b <<= 1, r++);
		SET_OUTPUT(SS);
		WRITE(SS,HIGH);
        SET_OUTPUT(SCK);
        SET_OUTPUT(MOSI_PIN);
        SET_INPUT(MISO_PIN);
#ifdef	PRR
        PRR &= ~(1<<PRSPI);
#elif defined PRR0
        PRR0 &= ~(1<<PRSPI);
#endif
        // See avr processor documentation
        SPCR = (1 << SPE) | (1 << MSTR) | (r >> 1);
        SPSR = (r & 1 || r == 6 ? 0 : 1) << SPI2X;

    }
    static inline uint8_t spiReceive(uint8_t send=0xff)
    {
        SPDR = send;
        while (!(SPSR & (1 << SPIF))) {}
        return SPDR;
    }
    static inline void spiReadBlock(uint8_t*buf,size_t nbyte)
    {
        if (nbyte-- == 0) return;
        SPDR = 0XFF;
        for (size_t i = 0; i < nbyte; i++)
        {
            while (!(SPSR & (1 << SPIF))) {}
            buf[i] = SPDR;
            SPDR = 0XFF;
        }
        while (!(SPSR & (1 << SPIF))) {}
        buf[nbyte] = SPDR;
    }
    static inline void spiSend(uint8_t b)
    {
        SPDR = b;
        while (!(SPSR & (1 << SPIF))) {}
    }
    static inline void spiSend(const uint8_t* buf , size_t n)
    {
        if (n == 0) return;
        SPDR = buf[0];
        if (n > 1)
        {
            uint8_t b = buf[1];
            size_t i = 2;
            while (1)
            {
                while (!(SPSR & (1 << SPIF))) {}
                SPDR = b;
                if (i == n) break;
                b = buf[i++];
            }
        }
        while (!(SPSR & (1 << SPIF))) {}
    }

    static inline __attribute__((always_inline))
    void spiSendBlock(uint8_t token, const uint8_t* buf)
    {
        SPDR = token;
        for (uint16_t i = 0; i < 512; i += 2)
        {
            while (!(SPSR & (1 << SPIF))) {}
            SPDR = buf[i];
            while (!(SPSR & (1 << SPIF))) {}
            SPDR = buf[i + 1];
        }
        while (!(SPSR & (1 << SPIF))) {}
    }

    // I2C Support

	static void i2cSetClockspeed(uint32_t clockSpeedHz);
    static void i2cInit(uint32_t clockSpeedHz);
    static unsigned char i2cStart(uint8_t address);
    static void i2cStartWait(uint8_t address);
    static void i2cStop(void);
    static void i2cWrite( uint8_t data );
    static uint8_t i2cReadAck(void);
    static uint8_t i2cReadNak(void);

    // Watchdog support

    inline static void startWatchdog()
    {
#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
        WDTCSR = (1<<WDCE) | (1<<WDE);								// wdt FIX for arduino mega boards
        WDTCSR = (1<<WDIE) | (1<<WDP3);
#else
        wdt_enable(WDTO_4S);
#endif
    };
    inline static void stopWatchdog()
    {
        wdt_disable();
    }
    inline static void pingWatchdog()
    {
#if FEATURE_WATCHDOG
      wdPinged = true;
#endif
    };
    inline static float maxExtruderTimerFrequency()
    {
        return (float)F_CPU/TIMER0_PRESCALE;
    }
#if FEATURE_SERVO
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo,int ms, uint16_t autoOff);
#endif
    static void analogStart();
#if USE_ADVANCE
    static void resetExtruderDirection();
#endif
protected:
private:
};
/*#if MOTHERBOARD==6 || MOTHERBOARD==62 || MOTHERBOARD==7
#if MOTHERBOARD!=7
#define SIMULATE_PWM
#endif
#define EXTRUDER_TIMER_VECTOR TIMER2_COMPA_vect
#define EXTRUDER_OCR OCR2A
#define EXTRUDER_TCCR TCCR2A
#define EXTRUDER_TIMSK TIMSK2
#define EXTRUDER_OCIE OCIE2A
#define PWM_TIMER_VECTOR TIMER2_COMPB_vect
#define PWM_OCR OCR2B
#define PWM_TCCR TCCR2B
#define PWM_TIMSK TIMSK2
#define PWM_OCIE OCIE2B
#else*/
#define EXTRUDER_TIMER_VECTOR TIMER0_COMPA_vect
#define EXTRUDER_OCR OCR0A
#define EXTRUDER_TCCR TCCR0A
#define EXTRUDER_TIMSK TIMSK0
#define EXTRUDER_OCIE OCIE0A
#define PWM_TIMER_VECTOR TIMER0_COMPB_vect
#define PWM_OCR OCR0B
#define PWM_TCCR TCCR0A
#define PWM_TIMSK TIMSK0
#define PWM_OCIE OCIE0B
//#endif
#endif // HAL_H
