#ifndef HAL_H
#define HAL_H

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

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

class HAL
{
    public:
        HAL();
        virtual ~HAL();
        // return val'val
        static inline unsigned long U16SquaredToU32(unsigned int val) {
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

    protected:
    private:
};

#endif // HAL_H
