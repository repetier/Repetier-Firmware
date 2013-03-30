#ifndef HAL_H
#define HAL_H

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

#include <avr/io.h>
#if CPU_ARCH==ARCH_AVR
#include <avr/io.h>
#else
#define PROGMEM
#define PGM_P const char *
#define PSTR(s) s
#define pgm_read_byte_near(x) (*(char*)x)
#define pgm_read_byte(x) (*(char*)x)
#endif

#include <avr/eeprom.h>

#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)

#if MOTHERBOARD==8 || MOTHERBOARD==9 || CPU_ARCH!=ARCH_AVR
#define EXTERNALSERIAL
#endif
//#define EXTERNALSERIAL  // Force using arduino serial
#ifndef EXTERNALSERIAL
#define  HardwareSerial_h // Don't use standard serial console
#endif
#include <inttypes.h>
#include "Print.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#include "gcode.h"
#if CPU_ARCH==ARCH_AVR
#include "fastio.h"
#else
#define	READ(IO)  digitalRead(IO)
#define	WRITE(IO, v)  digitalWrite(IO, v)
#define	SET_INPUT(IO)  pinMode(IO, INPUT)
#define	SET_OUTPUT(IO)  pinMode(IO, OUTPUT)
#endif

#define BEGIN_INTERRUPT_PROTECTED {byte sreg=SREG;__asm volatile( "cli" ::: "memory" );
#define END_INTERRUPT_PROTECTED SREG=sreg;}
#define ESCAPE_INTERRUPT_PROTECTED SREG=sreg;

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

class HAL
{
public:
    HAL();
    virtual ~HAL();
    // return val'val
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
        return ((timer>>8)*accel)>>10;
#endif
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned long mulu16xu16to32(unsigned int a,unsigned int b)
    {
        unsigned long res;
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
#if CPU_ARCH==ARCH_AVR
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
        return ((long)a*b)>>16;
#endif
    }
    static long CPUDivU2(unsigned int divisor);
    static inline void delayMicroseconds(unsigned int delayUs)
    {
        delayMicroseconds(delayUs);
    }

    static inline void epr_set_byte(unsigned int pos,byte value)
    {
        eeprom_write_byte((unsigned char *)(EEPROM_OFFSET+pos), value);
    }
    static inline void epr_set_int(unsigned int pos,int value)
    {
        eeprom_write_word((unsigned int*)(EEPROM_OFFSET+pos),value);
    }
    static inline void epr_set_long(unsigned int pos,long value)
    {
        eeprom_write_dword((unsigned long*)(EEPROM_OFFSET+pos),value);
    }
    static inline void epr_set_float(unsigned int pos,float value)
    {
        eeprom_write_block(&value,(void*)(EEPROM_OFFSET+pos), 4);
    }
    static inline byte epr_get_byte(unsigned int pos)
    {
        return eeprom_read_byte ((unsigned char *)(EEPROM_OFFSET+pos));
    }
    static inline int epr_get_int(unsigned int pos)
    {
        return eeprom_read_word((unsigned int *)(EEPROM_OFFSET+pos));
    }
    static inline long epr_get_long(unsigned int pos)
    {
        return eeprom_read_dword((unsigned long*)(EEPROM_OFFSET+pos));
    }
    static inline float epr_get_float(unsigned int pos)
    {
        float v;
        eeprom_read_block(&v,(void *)(EEPROM_OFFSET+pos),4); // newer gcc have eeprom_read_block but not arduino 22
        return v;
    }
    static inline void allowInterrupts()
    {
        sei();
    }
    static inline void forbidInterrupts()
    {
        cli();
    }
    static inline unsigned long timeInMilliseconds() {
        return millis();
    }
protected:
private:
};

#endif // HAL_H
