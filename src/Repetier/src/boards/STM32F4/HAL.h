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
*/

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

// -------- STM32F4 HAL.h ---------------

// You can set different sizes if you want, but with binary mode it does not get faster
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 128
#endif

#ifndef HAL_H_STM32F4
#define HAL_H_STM32F4

#define USE_ARDUINO_SPI_LIB

#include <inttypes.h>
#include "pins.h"
#include "Print.h"
#include "fastio.h"
#include <IWatchdog.h>

// Which I2C port to use?
#ifndef WIRE_PORT
#define WIRE_PORT Wire
#endif

#ifndef F_CPU
#ifdef f_cpu
#define F_CPU f_cpu
#else
#define F_CPU 180000000L // should be factor of F_CPU_TRUE
#endif
#endif
#define EEPROM_BYTES 4096 // bytes of eeprom we simulate
#define F_CPU_TRUE F_CPU  // actual CPU clock frequency

// Some structures assume no padding, need to add this attribute on ARM
#define PACK __attribute__((packed))

#define INLINE __attribute__((always_inline))

// do not use program space memory with Due
#define PROGMEM
#ifndef PGM_P
#define PGM_P const char*
#endif
typedef char prog_char;
#undef PSTR
#define PSTR(s) s
#undef pgm_read_byte_near
#define pgm_read_byte_near(x) (*(int8_t*)x)
#undef pgm_read_byte
#define pgm_read_byte(x) (*(int8_t*)x)
#undef pgm_read_float
#define pgm_read_float(addr) (*(const float*)(addr))
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

#define FSTRINGVALUE(var, value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#ifndef MOTION2_TIMER_NUM
#define MOTION2_TIMER_NUM 6
#endif
#ifndef PWM_TIMER_NUM
#define PWM_TIMER_NUM 2
#endif
#ifndef MOTION3_TIMER_NUM
#define MOTION3_TIMER_NUM 7
#endif
#ifndef SERVO_TIMER_NUM
#define SERVO_TIMER_NUM 5
#endif
#ifndef TONE_TIMER_NUM
#define TONE_TIMER_NUM 11
#endif

#define SERIAL_BUFFER_SIZE 128
//#define SERIAL_PORT             UART
//#define SERIAL_IRQ              ID_UART
//#define SERIAL_PORT_VECTOR      UART_Handler

// TWI1 if SDA pin = 20  TWI0 for pin = 70
// #define TWI_INTERFACE TWI1
// #define TWI_ID ID_TWI1

#define EXTRUDER_CLOCK_FREQ 60000 // extruder stepper interrupt frequency
// #define PWM_CLOCK_FREQ          3906
// #define PWM_COUNTER_100MS       390
#define PWM_CLOCK_FREQ 5000
#define PWM_COUNTER_100MS 500
//#define MOTION3_CLOCK_FREQ       244
//#define MOTION3_PRESCALE         2

/*
servo pulses repeat every 20ms, so each block of 4 servos has 5ms as maximum 
value to wait. So we need F_CPU * 0.005 ticks = 600000 ticks. With 16 bit timer
that means prescale factor 16 => 7.5*timeInUs ticks
*/
#define SERVO_CLOCK_FREQ 1000
#define SERVO_PRESCALE (FCPU / 20000) // Using TCLOCK1 therefore 2
#define SERVO2500US ((F_CPU_TRUE / SERVO_PRESCALE) / 8)
#define SERVO5000US ((F_CPU_TRUE / SERVO_PRESCALE) / 4)

#define AD_PRESCALE_FACTOR 84 // 500 kHz ADC clock
#define AD_TRACKING_CYCLES 4  // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES 1  // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel) (0x1u << channel)
// #define MAX_ANALOG_INPUTS 16 // gets already set in pins_arduino.h

#define PULLUP(IO, v) \
    { ::pinMode(IO, (v != LOW ? INPUT_PULLUP : INPUT)); }

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL 8000000 // 8sec

#include "Arduino.h"
#ifdef MAX_WIRE_INTERFACES
#undef WIRE_INTERFACES_COUNT
#define WIRE_INTERFACES_COUNT MAX_WIRE_INTERFACES
#endif
#include <Wire.h>

#define _READ(pin) (LL_GPIO_IsInputPinSet(GPIO_##pin, GPIO_##pin##_MASK) ? 1 : 0)
#define _WRITE(port, v) \
    do { \
        if (v) { \
            LL_GPIO_SetOutputPin(GPIO_##port, GPIO_##port##_MASK); \
        } else { \
            LL_GPIO_ResetOutputPin(GPIO_##port, GPIO_##port##_MASK); \
        }; \
    } while (0)
#define WRITE(pin, v) _WRITE(pin, v)
//   level = digital_io_read(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn));
#define READ_VAR(pin) (LL_GPIO_IsInputPinSet(get_GPIO_Port(STM_PORT(pin)), STM_LL_GPIO_PIN(pin)) ? 1 : 0)
#define READ(pin) _READ(pin)
#define WRITE_VAR(pin, v) \
    do { \
        if (v) { \
            LL_GPIO_SetOutputPin(get_GPIO_Port(STM_PORT(pin)), STM_LL_GPIO_PIN(pin)); \
        } else { \
            LL_GPIO_ResetOutputPin(get_GPIO_Port(STM_PORT(pin)), STM_LL_GPIO_PIN(pin)); \
        } \
    } while (0)
#define WRITE(pin, v) _WRITE(pin, v)

#define SET_INPUT(pin) ::pinMode(pin, INPUT);
// pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId);
//  PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define SET_OUTPUT(pin) ::pinMode(pin, OUTPUT);
//PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1,
//                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
#define _TOGGLE(port) LL_GPIO_TogglePin(GPIO_##port, GPIO_##port##_mask);
#define TOGGLE(pin) _TOGGLE(pin)
#define TOGGLE_VAR(pin) HAL::digitalWrite(pin, !HAL::digitalRead(pin))
#undef LOW
#define LOW 0
#undef HIGH
#define HIGH 1

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
        mask = __get_PRIMASK();
        ;
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

#define SECONDS_TO_TICKS(s) (unsigned long)(s * (float)F_CPU)
#define ANALOG_INPUT_SAMPLE 6
#define ANALOG_INPUT_MEDIAN 10

// Bits of the ADC converter
#define ANALOG_INPUT_BITS 12
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

// maximum available RAM
#ifndef MAX_RAM
#error Board definition needs to provide MAX_RAM!
#endif

#define bit_clear(x, y) x &= ~(1 << y) //cbi(x,y)
#define bit_set(x, y) x |= (1 << y)    //sbi(x,y)

typedef uint32_t speed_t;
typedef uint32_t ticks_t;
typedef uint32_t millis_t;
typedef int32_t flag8_t;
typedef int32_t fast8_t;
typedef uint32_t ufast8_t;

#ifndef RFSERIAL
#define RFSERIAL Serial // Programming port of the due
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
#define RFSERIAL2 BT_SERIAL

class RFDoubleSerial : public Print {
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

union eeval_t {
    uint8_t b[4];
    float f;
    uint32_t i;
    uint16_t s;
    long l;
} PACK;

#if EEPROM_AVAILABLE == EEPROM_SDCARD
extern millis_t eprSyncTime;
#endif

class HAL {
public:
    // we use ram instead of eeprom, so reads are faster and safer. Writes store in real eeprom as well
    // as long as hal eeprom functions are used.
    static char virtualEeprom[EEPROM_BYTES];
    static bool wdPinged;

    HAL();
    virtual ~HAL();

    // Try to initialize pinNumber as hardware PWM. Returns internal
    // id if it succeeds or -1 if it fails. Typical reasons to fail
    // are no pwm support for that pin or an other pin uses same PWM
    // channel.
    static int initHardwarePWM(int pinNumber, uint32_t frequency);
    // Set pwm output to value. id is id from initHardwarePWM.
    static void setHardwarePWM(int id, int value);
    // do any hardware-specific initialization here
    static inline void hwSetup(void) {
#if !FEATURE_WATCHDOG
        // Disable watchdog
#endif

#if defined(TWI_CLOCK_FREQ) && TWI_CLOCK_FREQ > 0 //init i2c if we have a frequency
        HAL::i2cInit(TWI_CLOCK_FREQ);
#endif
#if defined(EEPROM_AVAILABLE) && defined(EEPROM_SPI_ALLIGATOR) && EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
        HAL::spiBegin();
#endif
        // make debugging startup easier
        //Serial.begin(115200);

#if EEPROM_AVAILABLE && EEPROM_MODE != EEPROM_NONE && EEPROM_AVAILABLE != EEPROM_SDCARD
        // Copy eeprom to ram for faster access
        int i;
        for (i = 0; i < EEPROM_BYTES; i += 4) {
            eeval_t v = eprGetValue(i, 4);
            memcopy4(&virtualEeprom[i], &v.i);
        }
#else
        int i,
            n = 0;
        for (i = 0; i < EEPROM_BYTES; i += 4) {
            memcopy4(&virtualEeprom[i], &n);
        }
#endif
    }

    static inline void digitalWrite(uint8_t pin, uint8_t value) {
        WRITE_VAR(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin) {
        return READ_VAR(pin);
    }
    static inline void pinMode(uint8_t pin, uint8_t mode) {
        if (mode == INPUT) {
            SET_INPUT(pin);
        } else
            SET_OUTPUT(pin);
    }
    static INLINE void delayMicroseconds(uint32_t usec) { //usec += 3;
        ::delayMicroseconds(usec);
    }
    static inline void delayMilliseconds(unsigned int delayMs) {
        unsigned int del;
        while (delayMs > 0) {
            del = delayMs > 100 ? 100 : delayMs;
            HAL_Delay(del);
            // delay(del);
            delayMs -= del;
#if FEATURE_WATCHDOG
            HAL::pingWatchdog();
#endif
        }
    }
    static void tone(int frequency);
    static void noTone();

#if EEPROM_AVAILABLE == EEPROM_SDCARD
    static void syncEEPROM(); // store to disk if changed
    static void importEEPROM();
#endif

    static inline void eprSetByte(unsigned int pos, uint8_t value) {
        eeval_t v;
        v.b[0] = value;
        eprBurnValue(pos, 1, v);
        *(uint8_t*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetInt16(unsigned int pos, int16_t value) {
        eeval_t v;
        v.s = value;
        eprBurnValue(pos, 2, v);
        memcopy2(&virtualEeprom[pos], &value);
    }
    static inline void eprSetInt32(unsigned int pos, int32_t value) {
        eeval_t v;
        v.i = value;
        eprBurnValue(pos, 4, v);
        memcopy4(&virtualEeprom[pos], &value);
    }
    static inline void eprSetLong(unsigned int pos, long value) {
        eeval_t v;
        v.l = value;
        eprBurnValue(pos, sizeof(long), v);
        memcopy4(&virtualEeprom[pos], &value);
    }
    static inline void eprSetFloat(unsigned int pos, float value) {
        eeval_t v;
        v.f = value;
        eprBurnValue(pos, sizeof(float), v);
        memcopy4(&virtualEeprom[pos], &value);
    }
    static inline uint8_t eprGetByte(unsigned int pos) {
        return *(uint8_t*)&virtualEeprom[pos];
    }
    static inline int16_t eprGetInt16(unsigned int pos) {
        int16_t v;
        memcopy2(&v, &virtualEeprom[pos]);
        return v;
    }
    static inline int32_t eprGetInt32(unsigned int pos) {
        int32_t v;
        memcopy4(&v, &virtualEeprom[pos]);
        return v;
    }
    static inline long eprGetLong(unsigned int pos) {
        int32_t v;
        memcopy4(&v, &virtualEeprom[pos]);
        return v;
    }
    static inline float eprGetFloat(unsigned int pos) {
        float v;
        memcopy4(&v, &virtualEeprom[pos]);
        return v;
    }

    // Write any data type to EEPROM
    static inline void eprBurnValue(unsigned int pos, int size, union eeval_t newvalue) {
#if EEPROM_AVAILABLE == EEPROM_I2C
        i2cStartAddr(EEPROM_SERIAL_ADDR, pos, 0);
        i2cWrite(newvalue.b[0]); // write first byte
        for (int i = 1; i < size; i++) {
            pos++;
            // writes can not cross page boundary
            if ((pos % EEPROM_PAGE_SIZE) == 0) {
                // burn current page then address next one
                i2cStop();
                delayMilliseconds(EEPROM_PAGE_WRITE_TIME);
                i2cStartAddr(EEPROM_SERIAL_ADDR, pos, 0);
            }
            i2cWrite(newvalue.b[i]);
        }
        i2cStop();                                 // signal end of transaction
        delayMilliseconds(EEPROM_PAGE_WRITE_TIME); // wait for page write to complete
#elif EEPROM_AVAILABLE == EEPROM_SDCARD
        eprSyncTime = HAL::timeInMilliseconds() | 1UL;
#endif
    }

    // Read any data type from EEPROM that was previously written by eprBurnValue
    static inline union eeval_t eprGetValue(unsigned int pos, int size) {
#if EEPROM_AVAILABLE == EEPROM_I2C
        int i;
        eeval_t v;
        // set read location
        i2cStartAddr(EEPROM_SERIAL_ADDR, pos, size);
        for (i = 0; i < size; i++) {
            // read an incomming byte
            v.b[i] = i2cRead();
        }
        return v;
#else
        eeval_t v;
        int i;
        for (i = 0; i < size; i++) {
            // read an incomming byte
            v.b[i] = 0;
        }
        return v;
#endif //(MOTHERBOARD==500) || (MOTHERBOARD==501) || (MOTHERBOARD==502)
    }

    static inline void allowInterrupts() {
        //__enable_irq();
    }
    static inline void forbidInterrupts() {
        //__disable_irq();
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
        // Serial.setInterruptPriority(1);
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        BTAdapter.begin(baud);
#else
        RFSERIAL.begin(baud);
#endif
    }
    static inline bool serialByteAvailable() {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        return BTAdapter.available();
#else
        return RFSERIAL.available();
#endif
    }
    static inline uint8_t serialReadByte() {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        return BTAdapter.read();
#else
        return RFSERIAL.read();
#endif
    }
    static inline void serialWriteByte(char b) {
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        BTAdapter.write(b);
#else
        RFSERIAL.write(b);
#endif
    }
    static inline void serialFlush() {
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

    static void spiInit(); // only called once to initialize for spi usage
    static void spiBegin(uint32_t clock, uint8_t mode, uint8_t msbfirst);
    static uint8_t spiTransfer(uint8_t);
#ifndef USE_ARDUINO_SPI_LIB
    static void spiEnd() {}
#else
    static void spiEnd();
#endif

    // I2C Support
    static void i2cSetClockspeed(uint32_t clockSpeedHz);
    static void i2cInit(uint32_t clockSpeedHz);
    static void i2cStartRead(uint8_t address7bit, uint8_t bytes);
    static void i2cStart(uint8_t address7bit);
    static void i2cStartAddr(uint8_t address7bit, unsigned int pos, uint8_t readBytes);
    static void i2cStop(void);
    static void i2cWrite(uint8_t data);
    static int i2cRead(void);

    // Watchdog support
    inline static void startWatchdog() {
        IWatchdog.begin(WATCHDOG_INTERVAL);
    };
    inline static void stopWatchdog() {}
    inline static void pingWatchdog() {
#if FEATURE_WATCHDOG
        wdPinged = true;
#endif
    };

#if NUM_SERVOS > 0
    static unsigned int servoTimings[4];
    static void servoMicroseconds(uint8_t servo, int ms, uint16_t autoOff);
#endif

    static void analogStart(void);
    static int analogRead(int channel);
    static void analogEnable(int channel);
    static volatile uint8_t insideTimer1;
};

#endif // HAL_H
