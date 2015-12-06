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


    Main author: repetier

    Initial port of HAL to Arduino Due: John Silvia
*/

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

#ifndef HAL_H
#define HAL_H

#include <inttypes.h>
#include "pins.h"
#include "Print.h"
#include "fastio.h"

// Hack to make 84 MHz Due clock work without changes to pre-existing code
// which would otherwise have problems with int overflow.
#undef F_CPU
#define F_CPU       21000000        // should be factor of F_CPU_TRUE
#define F_CPU_TRUE  84000000        // actual CPU clock frequency
#define EEPROM_BYTES 4096  // bytes of eeprom we simulate
#define SUPPORT_64_BIT_MATH  // Gives better results with high resultion deltas

// another hack to keep AVR code happy (i.e. SdFat.cpp)
#define SPR0    0
#define SPR1    1

// force SdFat to use HAL (whether or not using SW spi)
#undef  SOFTWARE_SPI
#define TIMER0_PRESCALE 128

// Some structures assume no padding, need to add this attribute on ARM
#define PACK    __attribute__ ((packed))

#define INLINE __attribute__((always_inline))

// do not use program space memory with Due
#define PROGMEM
#define PGM_P const char *
typedef char prog_char;
#undef PSTR
#define PSTR(s) s
#undef pgm_read_byte_near
#define pgm_read_byte_near(x) (*(int8_t*)x)
#undef pgm_read_byte
#define pgm_read_byte(x) (*(int8_t*)x)
#undef pgm_read_float
#define pgm_read_float(addr) (*(const float *)(addr))
#undef pgm_read_word
//#define pgm_read_word(addr) (*(const unsigned int *)(addr))
#define pgm_read_word(addr) (*(addr))
#undef pgm_read_word_near
#define pgm_read_word_near(addr) pgm_read_word(addr)
#undef pgm_read_dword
#define pgm_read_dword(addr) (*(addr))
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#undef pgm_read_dword_near
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define _BV(x) (1 << (x))

#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var


#define EXTRUDER_TIMER          TC0
#define EXTRUDER_TIMER_CHANNEL  0
#define EXTRUDER_TIMER_IRQ      ID_TC0
#define EXTRUDER_TIMER_VECTOR   TC0_Handler
#define PWM_TIMER               TC0
#define PWM_TIMER_CHANNEL       1
#define PWM_TIMER_IRQ           ID_TC1
#define PWM_TIMER_VECTOR        TC1_Handler
#define TIMER1_TIMER            TC2
#define TIMER1_TIMER_CHANNEL    2
#define TIMER1_TIMER_IRQ        ID_TC8
#define TIMER1_COMPA_VECTOR     TC8_Handler
#define SERVO_TIMER             TC2
#define SERVO_TIMER_CHANNEL     0
#define SERVO_TIMER_IRQ         ID_TC6
#define SERVO_COMPA_VECTOR      TC6_Handler
#define BEEPER_TIMER            TC1
#define BEEPER_TIMER_CHANNEL    0
#define BEEPER_TIMER_IRQ        ID_TC3
#define BEEPER_TIMER_VECTOR     TC3_Handler
#define DELAY_TIMER             TC1
#define DELAY_TIMER_CHANNEL     1
#define DELAY_TIMER_IRQ         ID_TC4  // IRQ not really used, needed for pmc id
#define DELAY_TIMER_CLOCK       TC_CMR_TCCLKS_TIMER_CLOCK2
#define DELAY_TIMER_PRESCALE    8

//#define SERIAL_BUFFER_SIZE      1024
//#define SERIAL_PORT             UART
//#define SERIAL_IRQ              ID_UART
//#define SERIAL_PORT_VECTOR      UART_Handler

// TWI1 if SDA pin = 20  TWI0 for pin = 70
#define TWI_INTERFACE   		TWI1
#define TWI_ID  				ID_TWI1


#define EXTRUDER_CLOCK_FREQ     60000 // extruder stepper interrupt frequency
#define PWM_CLOCK_FREQ          3906
#define TIMER1_CLOCK_FREQ       244
#define TIMER1_PRESCALE         2


#define SERVO_CLOCK_FREQ        1000
#define SERVO_PRESCALE          2      // Using TCLOCK1 therefore 2
#define SERVO2500US             (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 2500)
#define SERVO5000US             (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 5000)

#define AD_PRESCALE_FACTOR      84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES      4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES      1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)    (0x1u << channel)
#define ENABLED_ADC_CHANNELS    {TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN}

#define PULLUP(IO,v)            {pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL       1024u  // 8sec  (~16 seconds max)



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif

//#define	READ(pin)  PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)
#define READ_VAR(pin) (g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? 1 : 0) // does return 0 or pin value
#define _READ(pin) (DIO ##  pin ## _PORT->PIO_PDSR & DIO ##  pin ## _PIN ? 1 : 0) // does return 0 or pin value
#define READ(pin) _READ(pin)
//#define	WRITE_VAR(pin, v) PIO_SetOutput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, v, 0, PIO_PULLUP)
#define	WRITE_VAR(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define		_WRITE(port, v)			do { if (v) {DIO ##  port ## _PORT -> PIO_SODR = DIO ## port ## _PIN; } else {DIO ##  port ## _PORT->PIO_CODR = DIO ## port ## _PIN; }; } while (0)
#define WRITE(pin,v) _WRITE(pin,v)

#define	SET_INPUT(pin) pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
  PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define	SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
#define TOGGLE(pin) WRITE(pin,!READ(pin))
#define TOGGLE_VAR(pin) HAL::digitalWrite(pin,!HAL::digitalRead(pin))
#undef LOW
#define LOW         0
#undef HIGH
#define HIGH        1

// Protects a variable scope for interrupts. Uses RAII to force clearance of
// Interrupt block at the end resp. sets them to previous state.
// Uses ABSEPRI to allow higher level interrupts then the one changing firmware data
#if 1
class InterruptProtectedBlock {
  public:
    INLINE void protect() {
      __disable_irq();
    }

    INLINE void unprotect() {
      __enable_irq();
    }

    INLINE InterruptProtectedBlock(bool later = false) {
      if (!later)
      __disable_irq();
    }

    INLINE ~InterruptProtectedBlock() {
      __enable_irq();
    }
};
#else
class InterruptProtectedBlock {
    uint32_t mask;
  public:
    inline void protect() {
      mask = __get_PRIMASK();;
      __disable_irq();
    }

    inline void unprotect() {
      __set_PRIMASK(mask);
    }

    inline InterruptProtectedBlock(bool later = false) {
      mask = __get_PRIMASK();
      if (!later)
        __disable_irq();
    }

    inline ~InterruptProtectedBlock() {
      __set_PRIMASK(mask);
    }
};
#endif

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_INPUT_SAMPLE 6
#define ANALOG_INPUT_MEDIAN 10

// Bits of the ADC converter
#define ANALOG_INPUT_BITS 12
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

// maximum available RAM
#define MAX_RAM 98303

#define bit_clear(x,y) x&= ~(1<<y) //cbi(x,y)
#define bit_set(x,y)   x|= (1<<y)//sbi(x,y)

/** defines the data direction (reading from I2C device) in i2cStart(),i2cRepStart() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2cStart(),i2cRepStart() */
#define I2C_WRITE   0

#ifndef DUE_SOFTWARE_SPI
extern int spiDueDividors[];
#endif

static uint32_t    tone_pin;

/** Set max. frequency to 500000 Hz */
#define LIMIT_INTERVAL (F_CPU/500000)


typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;
typedef unsigned int flag8_t;
typedef int fast8_t;
typedef unsigned int ufast8_t;

#ifndef RFSERIAL
#define RFSERIAL Serial   // Programming port of the due
//#define RFSERIAL SerialUSB  // Native USB Port of the due
#endif

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#if BLUETOOTH_SERIAL == 1
#define BT_SERIAL Serial1
#elif BLUETOOTH_SERIAL == 2
#define BT_SERIAL Serial2
#elif BLUETOOTH_SERIAL == 3
#define BT_SERIAL Serial3
#elif BLUETOOTH_SERIAL == 100
#define BT_SERIAL Serial
#elif BLUETOOTH_SERIAL == 101
#define BT_SERIAL SerialUSB
#endif

class RFDoubleSerial : public Print
{
  public:
    RFDoubleSerial();
    void begin(unsigned long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
};
extern RFDoubleSerial BTAdapter;

#endif


#define OUT_P_I(p,i) //Com::printF(PSTR(p),(int)(i))
#define OUT_P_I_LN(p,i) //Com::printFLN(PSTR(p),(int)(i))
#define OUT_P_L(p,i) //Com::printF(PSTR(p),(long)(i))
#define OUT_P_L_LN(p,i) //Com::printFLN(PSTR(p),(long)(i))
#define OUT_P_F(p,i) //Com::printF(PSTR(p),(float)(i))
#define OUT_P_F_LN(p,i) //Com::printFLN(PSTR(p),(float)(i))
#define OUT_P_FX(p,i,x) //Com::printF(PSTR(p),(float)(i),x)
#define OUT_P_FX_LN(p,i,x) //Com::printFLN(PSTR(p),(float)(i),x)
#define OUT_P(p) //Com::printF(PSTR(p))
#define OUT_P_LN(p) //Com::printFLN(PSTR(p))
#define OUT_ERROR_P(p) //Com::printErrorF(PSTR(p))
#define OUT_ERROR_P_LN(p) {//Com::printErrorF(PSTR(p));//Com::println();}
#define OUT(v) //Com::print(v)
#define OUT_LN //Com::println()

union eeval_t {
  uint8_t     b[4];
  float       f;
  uint32_t    i;
  uint16_t    s;
  long        l;
} PACK;

class HAL
{
  public:
    // we use ram instead of eeprom, so reads are faster and safer. Writes store in real eeprom as well
    // as long as hal eeprom functions are used.
    static char virtualEeprom[EEPROM_BYTES];
    static bool wdPinged;
    
    HAL();
    virtual ~HAL();

    // do any hardware-specific initialization here
    static inline void hwSetup(void)
    {
      #if !FEATURE_WATCHDOG
        // Disable watchdog
        WDT_Disable(WDT);
      #endif
  
      HAL::i2cInit(TWI_CLOCK_FREQ);
      // make debugging startup easier
      //Serial.begin(115200);
      TimeTick_Configure(F_CPU_TRUE);

      // setup microsecond delay timer
      pmc_enable_periph_clk(DELAY_TIMER_IRQ);
      TC_Configure(DELAY_TIMER, DELAY_TIMER_CHANNEL, TC_CMR_WAVSEL_UP |
                   TC_CMR_WAVE | DELAY_TIMER_CLOCK);
      TC_Start(DELAY_TIMER, DELAY_TIMER_CHANNEL);
#if EEPROM_AVAILABLE && EEPROM_MODE != EEPROM_NONE
      // Copy eeprom to ram for faster access
      int i;
      for (i = 0; i < EEPROM_BYTES; i += 4) {
        eeval_t v = eprGetValue(i, 4);
        memcopy4(&virtualEeprom[i],&v.i);
      }
#else
      int i,n = 0;
      for (i = 0; i < EEPROM_BYTES; i += 4) {
        memcopy4(&virtualEeprom[i],&n);
      }
#endif
    }

    static uint32_t integer64Sqrt(uint64_t a);
    // return val'val
    static inline unsigned long U16SquaredToU32(unsigned int val)
    {
      return (unsigned long) val * (unsigned long) val;
    }
    static inline unsigned int ComputeV(long timer, long accel)
    {
      return static_cast<unsigned int>((static_cast<int64_t>(timer) * static_cast<int64_t>(accel)) >> 18);
      //return ((timer>>8)*accel)>>10;
    }
    // Multiply two 16 bit values and return 32 bit result
    static inline unsigned long mulu16xu16to32(unsigned int a, unsigned int b)
    {
      return (unsigned long) a * (unsigned long) b;
    }
    // Multiply two 16 bit values and return 32 bit result
    static inline unsigned int mulu6xu16shift16(unsigned int a, unsigned int b)
    {
      return ((unsigned long)a * (unsigned long)b) >> 16;
    }
    static inline unsigned int Div4U2U(unsigned long a, unsigned int b)
    {
      return ((unsigned long)a / (unsigned long)b);
    }
    static inline void digitalWrite(uint8_t pin, uint8_t value)
    {
      WRITE_VAR(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin)
    {
      return READ_VAR(pin);
    }
    static inline void pinMode(uint8_t pin, uint8_t mode)
    {
      if (mode == INPUT) {
        SET_INPUT(pin);
      }
      else SET_OUTPUT(pin);
    }
    static long CPUDivU2(speed_t divisor) {
      return F_CPU / divisor;
    }
    static INLINE void delayMicroseconds(uint32_t usec)
    { //usec += 3;
      uint32_t n = usec * (F_CPU_TRUE / 3000000);
      asm volatile(
        "L2_%=_delayMicroseconds:"       "\n\t"
        "subs   %0, #1"                 "\n\t"
        "bge    L2_%=_delayMicroseconds" "\n"
        : "+r" (n) :
      );
    }
    static inline void delayMilliseconds(unsigned int delayMs)
    {
      unsigned int del;
      while (delayMs > 0) {
        del = delayMs > 100 ? 100 : delayMs;
        delay(del);
        delayMs -= del;
#if FEATURE_WATCHDOG
        HAL::pingWatchdog();
#endif
      }
    }
    static inline void tone(uint8_t pin, int frequency) {
      // set up timer counter 1 channel 0 to generate interrupts for
      // toggling output pin.
      SET_OUTPUT(pin);
      tone_pin = pin;
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)BEEPER_TIMER_IRQ);
      // set interrupt to lowest possible priority
      NVIC_SetPriority((IRQn_Type)BEEPER_TIMER_IRQ, NVIC_EncodePriority(4, 6, 3));
      TC_Configure(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |
                   TC_CMR_TCCLKS_TIMER_CLOCK4);  // TIMER_CLOCK4 -> 128 divisor
      uint32_t rc = VARIANT_MCK / 128 / frequency;
      TC_SetRA(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc / 2);                   // 50% duty cycle
      TC_SetRC(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc);
      TC_Start(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);
      BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
      BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
      NVIC_EnableIRQ((IRQn_Type)BEEPER_TIMER_IRQ);
    }
    static inline void noTone(uint8_t pin) {
      TC_Stop(TC1, 0);
      WRITE_VAR(pin, LOW);
    }

    static inline void eprSetByte(unsigned int pos, uint8_t value)
    {
      eeval_t v;
      v.b[0] = value;
      eprBurnValue(pos, 1, v);
      *(uint8_t*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetInt16(unsigned int pos, int16_t value)
    {
      eeval_t v;
      v.s = value;
      eprBurnValue(pos, 2, v);
      memcopy2(&virtualEeprom[pos],&value);
    }
    static inline void eprSetInt32(unsigned int pos, int32_t value)
    {
      eeval_t v;
      v.i = value;
      eprBurnValue(pos, 4, v);
      memcopy4(&virtualEeprom[pos],&value);
    }
    static inline void eprSetLong(unsigned int pos, long value)
    {
      eeval_t v;
      v.l = value;
      eprBurnValue(pos, sizeof(long), v);
      memcopy4(&virtualEeprom[pos],&value);
      //*(long*)(void*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetFloat(unsigned int pos, float value)
    {
      eeval_t v;
      v.f = value;
      eprBurnValue(pos, sizeof(float), v);
      memcopy4(&virtualEeprom[pos],&value);
    }
    static inline uint8_t eprGetByte(unsigned int pos)
    {
      return *(uint8_t*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos,1);
      //return v.b[0];
    }
    static inline int16_t eprGetInt16(unsigned int pos)
    {
      int16_t v;
      memcopy2(&v,&virtualEeprom[pos]);
      return v;
      //return *(int16_t*)(void*)&virtualEeprom[pos];
      //eeval_t v;
      //v.i = 0;
      //v = eprGetValue(pos, 2);
      //return v.i;
    }
    static inline int32_t eprGetInt32(unsigned int pos)
    {
      int32_t v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //eeval_t v = eprGetValue(pos, 4);
      //return v.i;
    }
    static inline long eprGetLong(unsigned int pos)
    {
      int32_t v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //return *(long*)(void*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos, sizeof(long));
      //return v.l;
    }
    static inline float eprGetFloat(unsigned int pos) {
      float v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //return *(float*)(void*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos, sizeof(float));
      //return v.f;
    }

    // Write any data type to EEPROM
    static inline void eprBurnValue(unsigned int pos, int size, union eeval_t newvalue)
    {
#if EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
      uint8_t eeprom_temp[3];

      /*write enable*/
      eeprom_temp[0] = 6;//WREN
      WRITE( SPI_EEPROM1_CS , LOW );
      spiSend(SPI_CHAN_EEPROM1 , eeprom_temp , 1);
      WRITE( SPI_EEPROM1_CS , HIGH );
      delayMilliseconds(1);

      /*write addr*/
      eeprom_temp[0] = 2;//WRITE
      eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
      eeprom_temp[2] = (pos & 0xFF); //addrL
      WRITE( SPI_EEPROM1_CS , LOW );
      spiSend(SPI_CHAN_EEPROM1 , eeprom_temp , 3);

      spiSend(SPI_CHAN_EEPROM1 , &(newvalue.b[0]) , 1);
      for (int i = 1; i < size; i++) {
        pos++;
        // writes cannot cross page boundary
        if ((pos % EEPROM_PAGE_SIZE) == 0) {
          // burn current page then address next one
          WRITE( SPI_EEPROM1_CS , HIGH );
          delayMilliseconds(EEPROM_PAGE_WRITE_TIME);

          /*write enable*/
          eeprom_temp[0] = 6;//WREN
          WRITE( SPI_EEPROM1_CS , LOW );
          spiSend(SPI_CHAN_EEPROM1 , eeprom_temp , 1);
          WRITE( SPI_EEPROM1_CS , HIGH );

          eeprom_temp[0] = 2;//WRITE
          eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
          eeprom_temp[2] = (pos & 0xFF); //addrL
          WRITE( SPI_EEPROM1_CS , LOW );
          spiSend(SPI_CHAN_EEPROM1 , eeprom_temp , 3);
        }
        spiSend(SPI_CHAN_EEPROM1 , &(newvalue.b[i]) , 1);
      }
      WRITE( SPI_EEPROM1_CS , HIGH );
      delayMilliseconds(EEPROM_PAGE_WRITE_TIME);   // wait for page write to complete
#elif EEPROM_AVAILABLE == EEPROM_I2C
      i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_WRITE, pos);
      i2cWriting(newvalue.b[0]);        // write first byte
      for (int i = 1; i < size; i++) {
        pos++;
        // writes cannot cross page boundary
        if ((pos % EEPROM_PAGE_SIZE) == 0) {
          // burn current page then address next one
          i2cStop();
          delayMilliseconds(EEPROM_PAGE_WRITE_TIME);
          i2cStartAddr(EEPROM_SERIAL_ADDR << 1, pos);
        } else {
          i2cTxFinished();      // wait for transmission register to empty
        }
        i2cWriting(newvalue.b[i]);
      }
      i2cStop();          // signal end of transaction
      delayMilliseconds(EEPROM_PAGE_WRITE_TIME);   // wait for page write to complete
#endif//(MOTHERBOARD==500) || (MOTHERBOARD==501)
    }

    // Read any data type from EEPROM that was previously written by eprBurnValue
    static inline union eeval_t eprGetValue(unsigned int pos, int size)
    {
#if EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
      int i = 0;
      eeval_t v;
      uint8_t eeprom_temp[3];
      size--;

      eeprom_temp[0] = 3;//READ
      eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
      eeprom_temp[2] = (pos & 0xFF); //addrL
      WRITE( SPI_EEPROM1_CS , HIGH );
      WRITE( SPI_EEPROM1_CS , LOW );

      spiSend(SPI_CHAN_EEPROM1 , eeprom_temp , 3);

      for (i = 0; i < size; i++) {
        // read an incomming byte
        v.b[i] = spiReceive(SPI_CHAN_EEPROM1);
      }
      // read last byte
      v.b[i] = spiReceive(SPI_CHAN_EEPROM1);
      WRITE( SPI_EEPROM1_CS , HIGH );
      return v;
#elif EEPROM_AVAILABLE == EEPROM_I2C
      int i;
      eeval_t v;

      size--;
      // set read location
      i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_READ, pos);
      // begin transmission from device
      i2cStartBit();
      for (i = 0; i < size; i++) {
        // read an incomming byte
        v.b[i] = i2cReadAck();
      }
      // read last byte
      v.b[i] = i2cReadNak();
      return v;
#endif //(MOTHERBOARD==500) || (MOTHERBOARD==501)
    }

    static inline void allowInterrupts()
    {
      //__enable_irq();
    }
    static inline void forbidInterrupts()
    {
      //__disable_irq();
    }
    static inline unsigned long timeInMilliseconds()
    {
      return millis();
    }
    static inline char readFlashByte(PGM_P ptr)
    {
      return pgm_read_byte(ptr);
    }
    static inline void serialSetBaudrate(long baud)
    {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      BTAdapter.begin(baud);
#else
      RFSERIAL.begin(baud);
#endif
    }
    static inline bool serialByteAvailable()
    {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      return BTAdapter.available();
#else
      return RFSERIAL.available();
#endif
    }
    static inline uint8_t serialReadByte()
    {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      return BTAdapter.read();
#else
      return RFSERIAL.read();
#endif
    }
    static inline void serialWriteByte(char b)
    {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      BTAdapter.write(b);
#else
      RFSERIAL.write(b);
#endif
    }
    static inline void serialFlush()
    {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      BTAdapter.flush();
#else
      RFSERIAL.flush();
#endif
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions

#ifdef DUE_SOFTWARE_SPI
    // bitbanging transfer
    // run at ~100KHz (necessary for init)
    static uint8_t spiTransfer(uint8_t b)  // using Mode 0
    {
      for (int bits = 0; bits < 8; bits++) {
        if (b & 0x80) {
          WRITE(MOSI_PIN, HIGH);
        } else {
          WRITE(MOSI_PIN, LOW);
        }
        b <<= 1;

        WRITE(SCK_PIN, HIGH);
        delayMicroseconds(5);

        if (READ(MISO_PIN)) {
          b |= 1;
        }
        WRITE(SCK_PIN, LOW);
        delayMicroseconds(5);
      }
      return b;
    }
    static inline void spiBegin()
    {
      SET_OUTPUT(SDSS);
      WRITE(SDSS, HIGH);
      SET_OUTPUT(SCK_PIN);
      SET_INPUT(MISO_PIN);
      SET_OUTPUT(MOSI_PIN);
    }

    static inline void spiInit(uint8_t spiClock)
    {
      WRITE(SDSS, HIGH);
      WRITE(MOSI_PIN, HIGH);
      WRITE(SCK_PIN, LOW);
    }
    static inline uint8_t spiReceive()
    {
      WRITE(SDSS, LOW);
      uint8_t b = spiTransfer(0xff);
      WRITE(SDSS, HIGH);
      return b;
    }
    static inline void spiReadBlock(uint8_t*buf, uint16_t nbyte)
    {
      if (nbyte == 0) return;
      WRITE(SDSS, LOW);
      for (int i = 0; i < nbyte; i++)
      {
        buf[i] = spiTransfer(0xff);
      }
      WRITE(SDSS, HIGH);

    }
    static inline void spiSend(uint8_t b) {
      WRITE(SDSS, LOW);
      uint8_t response = spiTransfer(b);
      WRITE(SDSS, HIGH);
    }

    static inline void spiSend(const uint8_t* buf , size_t n)
    {
      uint8_t response;
      if (n == 0) return;
      WRITE(SDSS, LOW);
      for (uint16_t i = 0; i < n; i++) {
        response = spiTransfer(buf[i]);
      }
      WRITE(SDSS, HIGH);
    }

    inline __attribute__((always_inline))
    static void spiSendBlock(uint8_t token, const uint8_t* buf)
    {
      uint8_t response;

      WRITE(SDSS, LOW);
      response = spiTransfer(token);

      for (uint16_t i = 0; i < 512; i++)
      {
        response = spiTransfer(buf[i]);
      }
      WRITE(SDSS, HIGH);
    }

#else

    // hardware SPI
    static void spiBegin();
    // spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
    // Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
    static void spiInit(uint8_t spiClock);
    // Write single byte to SPI
    static void spiSend(byte b);
    static void spiSend(const uint8_t* buf , size_t n);
#if MOTHERBOARD == 500 || MOTHERBOARD == 501
    static void spiSend(uint32_t chan , const uint8_t* buf , size_t n);
    static void spiSend(uint32_t chan, byte b);
    static uint8_t spiReceive(uint32_t chan);
#endif
    // Read single byte from SPI
    static uint8_t spiReceive();
    // Read from SPI into buffer
    static void spiReadBlock(uint8_t*buf, uint16_t nbyte);

    // Write from buffer to SPI

    static void spiSendBlock(uint8_t token, const uint8_t* buf);
#endif  /*DUE_SOFTWARE_SPI*/

    // I2C Support
    static void i2cInit(unsigned long clockSpeedHz);
    static void i2cStartWait(unsigned char address);
    static unsigned char i2cStart(unsigned char address);
    static void i2cStartAddr(unsigned char address, unsigned int pos);
    static void i2cStop(void);
    static void i2cStartBit(void);
    static void i2cCompleted (void);
    static void i2cTxFinished(void);
    static void i2cWriting( uint8_t data );
    static unsigned char i2cWrite( unsigned char data );
    static unsigned char i2cReadAck(void);
    static unsigned char i2cReadNak(void);


    // Watchdog support
    inline static void startWatchdog() {
      WDT->WDT_MR = WDT_MR_WDRSTEN | WATCHDOG_INTERVAL | (WATCHDOG_INTERVAL << 16);
      WDT->WDT_CR = 0xA5000001;
    };
    inline static void stopWatchdog() {}
    inline static void pingWatchdog() {
#if FEATURE_WATCHDOG
      wdPinged = true;
#endif
    };

    inline static float maxExtruderTimerFrequency() {
      return (float)F_CPU_TRUE/32;
    }
#if FEATURE_SERVO
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo, int ms, uint16_t autoOff);
#endif

#if ANALOG_INPUTS > 0
    static void analogStart(void);
#endif
#if USE_ADVANCE
    static void resetExtruderDirection();
#endif
    static volatile uint8_t insideTimer1;
};

#endif // HAL_H
