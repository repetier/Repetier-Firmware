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

// You can set different sizes if you want, but with binary mode it does not get faster
#ifndef SERIAL_RX_BUFFER_SIZE
#ifdef SERIAL_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE SERIAL_BUFFER_SIZE
#else
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#endif

#ifndef HAL_H
#define HAL_H

#define USE_ARDUINO_SPI_LIB

#include <inttypes.h>
#include "pins.h"
#include "Print.h"
#include "fastio.h"

// Which I2C port to use?
#ifndef WIRE_PORT
#define WIRE_PORT Wire
#endif

// Hack to make 84 MHz Due clock work without changes to pre-existing code
// which would otherwise have problems with int overflow.
#undef F_CPU
#define F_CPU 21000000      // should be factor of F_CPU_TRUE
#define F_CPU_TRUE 84000000 // actual CPU clock frequency
#define EEPROM_BYTES 4096   // bytes of eeprom we simulate

// another hack to keep AVR code happy (i.e. SdFat.cpp)
#define SPR0 0
#define SPR1 1

// force SdFat to use HAL (whether or not using SW spi)
#if MOTHERBOARD != 409 // special case ultratronics
#undef SOFTWARE_SPI
#endif

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
#define _BV(x) (1 << (x))

#define FSTRINGVALUE(var, value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#define PWM_CLOCK_FREQ 10000
#define PWM_COUNTER_100MS PWM_CLOCK_FREQ / 10

// Manipulate the SAM3X8E's real time timer for motion2
// instead of using a valuable PWM-capable timer 
// Only really good for < 8000Hz
#define MOTION2_USE_REALTIME_TIMER 1 
#if (DISABLED(MOTION2_USE_REALTIME_TIMER) || PREPARE_FREQUENCY > (PWM_CLOCK_FREQ / 2))
#define MOTION2_TIMER TC0
#define MOTION2_TIMER_CHANNEL 0
#define MOTION2_TIMER_IRQ ID_TC0
#define MOTION2_TIMER_VECTOR TC0_Handler
#endif

#define PWM_TIMER TC0
#define PWM_TIMER_CHANNEL 1
#define PWM_TIMER_IRQ ID_TC1
#define PWM_TIMER_VECTOR TC1_Handler
#define MOTION3_TIMER TC2
#define MOTION3_TIMER_CHANNEL 2
#define MOTION3_TIMER_IRQ ID_TC8
#define MOTION3_TIMER_VECTOR TC8_Handler
#define SERVO_TIMER TC2
#define SERVO_TIMER_CHANNEL 0
#define SERVO_TIMER_IRQ ID_TC6
#define SERVO_TIMER_VECTOR TC6_Handler
#define BEEPER_TIMER TC1
#define BEEPER_TIMER_CHANNEL 0
#define BEEPER_TIMER_IRQ ID_TC3
#define BEEPER_TIMER_VECTOR TC3_Handler

//#define SERIAL_BUFFER_SIZE      1024
//#define SERIAL_PORT             UART
//#define SERIAL_IRQ              ID_UART
//#define SERIAL_PORT_VECTOR      UART_Handler

// TWI1 if SDA pin = 20  TWI0 for pin = 70
#define TWI_INTERFACE TWI1
#define TWI_ID ID_TWI1

#define SERVO_CLOCK_FREQ 1000
#define SERVO_PRESCALE 2 // Using TCLOCK1 therefore 2
#define SERVO2500US (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 2500)
#define SERVO5000US (((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * 5000)

#define AD_PRESCALE_FACTOR 84 // 500 kHz ADC clock
#define AD_TRACKING_CYCLES 4  // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES 1  // 0 - 3      * 2 + 3 adc clock cycles

#define MAX_ANALOG_INPUTS 16
extern bool analogEnabled[MAX_ANALOG_INPUTS];

#define PULLUP(IO, v) \
    { ::pinMode(IO, (v != LOW ? INPUT_PULLUP : INPUT)); }

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL 1024u // 8sec  (~16 seconds max)

#include "Arduino.h"
#ifdef MAX_WIRE_INTERFACES
#undef WIRE_INTERFACES_COUNT
#define WIRE_INTERFACES_COUNT MAX_WIRE_INTERFACES
#endif
#include <Wire.h>

//#define	READ(pin)  PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)
#define READ_VAR(pin) (g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? 1 : 0) // does return 0 or pin value
#define _READ(pin) (DIO##pin##_PORT->PIO_PDSR & DIO##pin##_PIN ? 1 : 0)                               // does return 0 or pin value
#define READ(pin) _READ(pin)
//#define	WRITE_VAR(pin, v) PIO_SetOutput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, v, 0, PIO_PULLUP)
#define WRITE_VAR(pin, v) \
    do { \
        if (v) { \
            g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin; \
        } else { \
            g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; \
        } \
    } while (0)
#define _WRITE(port, v) \
    do { \
        if (v) { \
            DIO##port##_PORT->PIO_SODR = DIO##port##_PIN; \
        } else { \
            DIO##port##_PORT->PIO_CODR = DIO##port##_PIN; \
        }; \
    } while (0)
#define WRITE(pin, v) _WRITE(pin, v)

#define SET_INPUT(pin) ::pinMode(pin, INPUT);
// pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId);
//  PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define SET_OUTPUT(pin) ::pinMode(pin, OUTPUT);
//PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1,
//                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
#define TOGGLE(pin) WRITE(pin, !READ(pin))
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
#define MAX_RAM 98303

#define bit_clear(x, y) x &= ~(1 << y) //cbi(x,y)
#define bit_set(x, y) x |= (1 << y)    //sbi(x,y)

#ifndef DUE_SOFTWARE_SPI
extern int spiDueDividors[];
#endif

typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;
typedef int32_t flag8_t;
typedef int32_t fast8_t;
typedef unsigned int ufast8_t;

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
#endif

union eeval_t {
    uint8_t b[4];
    float f;
    uint32_t i;
    uint16_t s;
    long l;
} PACK;

#if EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH
extern millis_t eprSyncTime;
#endif
#if EEPROM_AVAILABLE == EEPROM_FLASH
extern void FEUpdateChanges(void);
extern void FEInit(void);
#endif

class HAL {
public:
    // we use ram instead of eeprom, so reads are faster and safer. Writes store in real eeprom as well
    // as long as hal eeprom functions are used.
    static char virtualEeprom[EEPROM_BYTES];
    static bool wdPinged;
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
    // do any hardware-specific initialization here
    static inline void hwSetup(void) {
        updateStartReason();
#if !FEATURE_WATCHDOG
        WDT_Disable(WDT); // Disable watchdog
#else
        WDT->WDT_MR |= WDT_MR_WDDBGHLT; // Disable watchdog when debugging only.
#endif

#if defined(TWI_CLOCK_FREQ) && TWI_CLOCK_FREQ > 0 //init i2c if we have a frequency
        HAL::i2cInit(TWI_CLOCK_FREQ);
#endif
#if defined(EEPROM_AVAILABLE) && defined(EEPROM_SPI_ALLIGATOR) && EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
        HAL::spiBegin();
#endif
        // make debugging startup easier
        //Serial.begin(115200);
        TimeTick_Configure(F_CPU_TRUE);

#if EEPROM_AVAILABLE && EEPROM_MODE != EEPROM_NONE && EEPROM_AVAILABLE != EEPROM_SDCARD && EEPROM_AVAILABLE != EEPROM_FLASH
        // Copy eeprom to ram for faster access
        int i;
        for (i = 0; i < EEPROM_BYTES; i += 4) {
            eeval_t v = eprGetValue(i, 4);
            memcopy4(&virtualEeprom[i], &v.i);
        }
#else
        int i, n = 0;
        for (i = 0; i < EEPROM_BYTES; i += 4) {
            memcopy4(&virtualEeprom[i], &n);
        }
#endif
#if EEPROM_AVAILABLE == EEPROM_FLASH && EEPROM_MODE != EEPROM_NONE
        FEInit();
#endif
        trng_enable(TRNG);
        randomSeed(trng_read_output_data(TRNG));
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
        } else if (mode == INPUT_PULLUP) {
            PULLUP(pin, HIGH);
        } else {
            SET_OUTPUT(pin);
        }
    }
    static INLINE void delayMicroseconds(uint32_t usec) { //usec += 3;
        uint32_t n = usec * (F_CPU_TRUE / 3000000);
        asm volatile(
            "L2_%=_delayMicroseconds:"
            "\n\t"
            "subs   %0, #1"
            "\n\t"
            "bge    L2_%=_delayMicroseconds"
            "\n"
            : "+r"(n)
            :);
    }
    static inline void delayMilliseconds(unsigned int delayMs) {
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
    static void tone(uint32_t frequency);
    static void noTone();

#if EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH
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
    static void eprBurnValue(unsigned int pos, int size, union eeval_t newvalue);

    // Read any data type from EEPROM that was previously written by eprBurnValue
    static inline union eeval_t eprGetValue(unsigned int pos, int size) {
#if EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
        int i = 0;
        eeval_t v;
        uint8_t eeprom_temp[3];
        size--;

        eeprom_temp[0] = 3;                   //READ
        eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
        eeprom_temp[2] = (pos & 0xFF);        //addrL
        WRITE(SPI_EEPROM1_CS, HIGH);
        WRITE(SPI_EEPROM1_CS, LOW);

        spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

        for (i = 0; i < size; i++) {
            // read an incomming byte
            v.b[i] = spiReceive(SPI_CHAN_EEPROM1);
        }
        // read last byte
        v.b[i] = spiReceive(SPI_CHAN_EEPROM1);
        WRITE(SPI_EEPROM1_CS, HIGH);
        return v;
#elif EEPROM_AVAILABLE == EEPROM_I2C
        int i;
        eeval_t v;
        // set read location
        i2cStartAddr(EEPROM_SERIAL_ADDR, pos, size);
        for (i = 0; i < size; i++) {
            // read an incomming byte
            int val = i2cRead();
            if (val != -1) {
                v.b[i] = val;
            } else {
                v.b[i] = 0;
            }
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
        return GetTickCount(); // millis();
    }
    static inline char readFlashByte(PGM_P ptr) {
        return pgm_read_byte(ptr);
    }
    static inline int16_t readFlashWord(PGM_P ptr) {
        return pgm_read_word(ptr);
    }

    static inline void serialSetBaudrate(long baud) {
        static bool serialInitialized = false;
        Serial.setInterruptPriority(1);
        if (serialInitialized && static_cast<Stream*>(&RFSERIAL) != &SerialUSB) { // When just updating the baudrate, don't restart USB.
            RFSERIAL.end();
        }
        RFSERIAL.begin(baud);
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        if (serialInitialized && static_cast<Stream*>(&RFSERIAL2) != &SerialUSB) { // When just updating the baudrate, don't restart USB.
            RFSERIAL2.end();
        }
        RFSERIAL2.begin(baud);
#endif
        serialInitialized = true;
    }
    static inline void serialFlush() {
        RFSERIAL.flush();
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
        RFSERIAL2.flush();
#endif
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
#ifndef USE_ARDUINO_SPI_LIB
    static void spiEnd() { }
#else
    static void spiEnd();
#endif
    // SPI related functions
#ifdef OLD_SPI
#ifdef DUE_SOFTWARE_SPI
    // bitbanging transfer
    // run at ~100KHz (necessary for init)
    static uint8_t
    spiTransfer(uint8_t b) // using Mode 0
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
    static inline void spiBegin(uint8_t ssPin = 0) {
        if (ssPin) {
            HAL::digitalWrite(ssPin, 0);
        } else {
            SET_OUTPUT(SDSS);
            WRITE(SDSS, HIGH);
        }
        SET_OUTPUT(SCK_PIN);
        SET_INPUT(MISO_PIN);
        SET_OUTPUT(MOSI_PIN);
    }

    static inline void spiInit(uint8_t spiClock) {
        WRITE(SDSS, HIGH);
        WRITE(MOSI_PIN, HIGH);
        WRITE(SCK_PIN, LOW);
    }
    static inline uint8_t spiReceive() {
        // WRITE(SDSS, LOW);
        uint8_t b = spiTransfer(0xff);
        // WRITE(SDSS, HIGH);
        return b;
    }
    static inline void spiReadBlock(uint8_t* buf, uint16_t nbyte) {
        if (nbyte == 0)
            return;
        // WRITE(SDSS, LOW);
        for (int i = 0; i < nbyte; i++) {
            buf[i] = spiTransfer(0xff);
        }
        // WRITE(SDSS, HIGH);
    }
    static inline void spiSend(uint8_t b) {
        // WRITE(SDSS, LOW);
        uint8_t response = spiTransfer(b);
        // WRITE(SDSS, HIGH);
    }

    static inline void spiSend(const uint8_t* buf, size_t n) {
        if (n == 0)
            return;
        // WRITE(SDSS, LOW);
        for (uint16_t i = 0; i < n; i++) {
            spiTransfer(buf[i]);
        }
        // WRITE(SDSS, HIGH);
    }

    inline __attribute__((always_inline)) static void spiSendBlock(uint8_t token, const uint8_t* buf) {
        // WRITE(SDSS, LOW);
        spiTransfer(token);

        for (uint16_t i = 0; i < 512; i++) {
            spiTransfer(buf[i]);
        }
        // WRITE(SDSS, HIGH);
    }

#else

    // hardware SPI
    static void
    spiBegin(uint8_t ssPin = 0);
    // spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
    // Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
    static void spiInit(uint8_t spiClock);
    // Write single byte to SPI
    static void spiSend(byte b);
    static void spiSend(const uint8_t* buf, size_t n);
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || MOTHERBOARD == 502
    static void spiSend(uint32_t chan, const uint8_t* buf, size_t n);
    static void spiSend(uint32_t chan, byte b);
    static uint8_t spiReceive(uint32_t chan);
#endif
    // Read single byte from SPI
    static uint8_t spiReceive();
    // Read from SPI into buffer
    static void spiReadBlock(uint8_t* buf, uint16_t nbyte);

    // Write from buffer to SPI

    static void spiSendBlock(uint8_t token, const uint8_t* buf);
#endif /*DUE_SOFTWARE_SPI*/
#endif
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
        //WDT->WDT_MR = WDT_MR_WDRSTEN | WATCHDOG_INTERVAL | 0x0fff0000; //(WATCHDOG_INTERVAL << 16);
        //WDT->WDT_CR = 0xA5000001;
        WDT->WDT_CR = 0xA5000001; //reset clock before updating WDD
        delayMicroseconds(92);    //must wait a minimum of 3 slow clocks before updating WDT_MR after writing WDT_CR
        WDT->WDT_MR = WDT_MR_WDRSTEN | WATCHDOG_INTERVAL | (WATCHDOG_INTERVAL << 16);
        WDT->WDT_CR = 0xA5000001;
    };
    inline static void stopWatchdog() { }
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
    static void analogEnable(int channel);
    static int analogRead(int channel) { return ADC->ADC_CDR[channel]; }
    static void reportHALDebug() { }
    static volatile uint8_t insideTimer1;
    static void switchToBootMode();
};

#endif // HAL_H
