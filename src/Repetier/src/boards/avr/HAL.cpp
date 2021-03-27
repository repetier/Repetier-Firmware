#include "Repetier.h"
#ifdef AVR_BOARD

#if defined(WIRE_PORT)
#include <compat/twi.h>
#endif

// New adc handling
#if MAX_ANALOG_INPUTS == 16
bool analogEnabled[MAX_ANALOG_INPUTS] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
uint16_t analogValues[MAX_ANALOG_INPUTS] = { 0 };
int8_t analogSamplePos = 15;
int8_t analogInputCount = 0;
#endif

// end adc handling

#if FEATURE_WATCHDOG
bool HAL::wdPinged = false;
#endif
uint8_t HAL::i2cError = 0;
BootReason HAL::startReason = BootReason::UNKNOWN;

// Seems that under AVR these are missing, wonder why new is not missing?
void operator delete(void* ptr, size_t size) {
    free(ptr);
}

void operator delete[](void* ptr, size_t size) {
    free(ptr);
}

//extern "C" void __cxa_pure_virtual() { }

HAL::HAL() {
    //ctor
}

HAL::~HAL() {
    //dtor
}

void HAL::setupTimer() {
    PWM_TCCR = 0; // Setup PWM interrupt
    PWM_OCR = 64;
    PWM_TIMSK |= _BV(PWM_OCIE);

    TCCR1A = 0; // Stepper timer 1 interrupt to no prescale CTC mode
    TCCR1B = _BV(WGM12);
    TCCR1C = 0;
    TIMSK1 = 0;
    TCCR1B = (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A = F_CPU / STEPPER_FREQUENCY; //start off with a slow frequency.
    TIMSK1 |= (1 << OCIE1A);           // Enable interrupt
#if F_CPU / PREPARE_FREQUENCY / 128 > 255
#error PREPARE_FREQUENCY is too low!
#endif
    uint8_t prepSteps = F_CPU / PREPARE_FREQUENCY / 128; // 250 for 500Hz
    OCR2A = static_cast<uint8_t>(prepSteps);
    TCCR2A = _BV(WGM21);            // CTC mode
    TCCR2B = _BV(CS22) + _BV(CS20); // Prescaler 1/128
    TIMSK2 |= _BV(OCIE2A);          // Enable interrupt

#if NUM_SERVOS > 0 || NUM_BEEPERS > 0
    TCCR3A = 0;         // normal counting mode
    TCCR3B = _BV(CS31); // set prescaler of 8
    TCNT3 = 0;          // clear the timer count
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);    // clear any pending interrupts;
    ETIMSK |= _BV(OCIE3A); // enable the output compare interrupt
#else
    TIFR3 = _BV(OCF3A);   // clear any pending interrupts;
    TIMSK3 = _BV(OCIE3A); // enable the output compare interrupt
#endif
#endif
}

// Print apparent cause of start/restart
void HAL::showStartReason() {
    if (startReason == BootReason::BROWNOUT) {
        Com::printInfoFLN(Com::tBrownOut);
    } else if (startReason == BootReason::WATCHDOG_RESET) {
        Com::printInfoFLN(Com::tWatchdog);
    } else if (startReason == BootReason::SOFTWARE_RESET) {
        Com::printInfoFLN(PSTR("Software reset"));
    } else if (startReason == BootReason::POWER_UP) {
        Com::printInfoFLN(Com::tPowerUp);
    } else if (startReason == BootReason::EXTERNAL_PIN) {
        Com::printInfoFLN(PSTR("External reset pin reset"));
    } else {
        Com::printInfoFLN(PSTR("Unknown reset reason"));
    }
}

void HAL::updateStartReason() {
    // Check startup - does nothing if bootloader sets MCUSR to 0
    uint8_t mcu = MCUSR;
    startReason = BootReason::UNKNOWN;
    if (mcu & 1) {
        startReason = BootReason::POWER_UP;
    }
    if (mcu & 2) {
        startReason = BootReason::EXTERNAL_PIN;
    }
    if (mcu & 4) {
        startReason = BootReason::BROWNOUT;
    }
    if (mcu & 8) {
        startReason = BootReason::WATCHDOG_RESET;
    }
    if (mcu & 32) {
        startReason = BootReason::SOFTWARE_RESET;
    }
    MCUSR = 0;
}

int HAL::getFreeRam() {
    int freeram = 0;
    InterruptProtectedBlock noInts;
    uint8_t *heapptr, *stackptr;
    heapptr = (uint8_t*)malloc(4); // get heap pointer
    free(heapptr);                 // free up the memory again (sets heapptr to 0)
    stackptr = (uint8_t*)(SP);     // save value of stack pointer
    freeram = (int)stackptr - (int)heapptr;
    return freeram;
}

void (*resetFunc)(void) = 0; //declare reset function @ address 0

void HAL::resetHardware() {
    resetFunc();
}

void HAL::analogStart() {
    if (analogInputCount > 0) {
        ADMUX = ANALOG_REF; // refernce voltage
        ADCSRA = _BV(ADEN) | _BV(ADSC) | ANALOG_PRESCALER;
        //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
        while (ADCSRA & _BV(ADSC)) { } // wait for conversion
                                       /* ADCW must be read once, otherwise the next result is wrong. */
                                       //uint dummyADCResult;
                                       //dummyADCResult = ADCW;
                                       // Enable interrupt driven conversion loop
        analogSamplePos = 0;
        while (!analogEnabled[analogSamplePos]) {
            analogSamplePos++;
        }
#if defined(ADCSRB) && defined(MUX5)
        if (analogSamplePos & 8) { // Reading channel 0-7 or 8-15?
            ADCSRB |= _BV(MUX5);
        } else {
            ADCSRB &= ~_BV(MUX5);
        }
#endif
        ADMUX = (ADMUX & ~(0x1F)) | (analogSamplePos & 7);
        ADCSRA |= _BV(ADSC); // start conversion without interrupt!
    }
}

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

#include <avr/io.h>

/****************************************************************************************
 Setting for I2C Clock speed. needed to change  clock speed for different peripherals
****************************************************************************************/

void HAL::i2cSetClockspeed(uint32_t clockSpeedHz) {
#if defined(WIRE_PORT)
    WIRE_PORT.setClock(clockSpeedHz);
#endif
}

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(uint32_t clockSpeedHz) {
#if defined(WIRE_PORT)
    WIRE_PORT.begin(); // create I2C master access
    WIRE_PORT.setClock(clockSpeedHz);
#endif
}

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
*************************************************************************/
/* void HAL::i2cStart(unsigned char address) {
    WIRE_PORT.beginTransmission(address);
} */

void HAL::i2cStartRead(uint8_t address, uint8_t bytes) {
#if defined(WIRE_PORT)
    if (!i2cError) {
        i2cError |= (WIRE_PORT.requestFrom(address, bytes) != bytes);
    }
#endif
}
/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 Also specifies internal address of device

 Input:   address and transfer direction of I2C device, internal address
*************************************************************************/
void HAL::i2cStartAddr(unsigned char address, unsigned int pos, uint8_t readBytes) {
#if defined(WIRE_PORT)
    if (!i2cError) {
        WIRE_PORT.beginTransmission(address);
        WIRE_PORT.write(pos >> 8);
        WIRE_PORT.write(pos & 255);
        if (readBytes) {
            i2cError |= WIRE_PORT.endTransmission();
            i2cError |= (WIRE_PORT.requestFrom(address, readBytes) != readBytes);
        }
    }
#endif
}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void) {
#if defined(WIRE_PORT)
    i2cError |= WIRE_PORT.endTransmission();
#endif
}

/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
*************************************************************************/
void HAL::i2cWrite(uint8_t data) {
#if defined(WIRE_PORT)
    if (!i2cError) {
        WIRE_PORT.write(data);
    }
#endif
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
int HAL::i2cRead(void) {
#if defined(WIRE_PORT)
    if (!i2cError && WIRE_PORT.available()) {
        return WIRE_PORT.read();
    }
#endif
    return -1; // should never happen, but better then blocking
}

#if NUM_SERVOS > 0 || NUM_BEEPERS > 0
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_ATmega128__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega2561__)
#define SERVO2500US F_CPU / 3200
#define SERVO5000US F_CPU / 1600
unsigned int HAL::servoTimings[4] = { 0, 0, 0, 0 };
unsigned int servoAutoOff[4] = { 0, 0, 0, 0 };
static uint8_t servoIndex = 0;
void HAL::servoMicroseconds(uint8_t servo, int ms, uint16_t autoOff) {
    servoTimings[servo] = (unsigned int)(((F_CPU / 1000000) * (long)ms) >> 3);
    servoAutoOff[servo] = (ms) ? (autoOff / 20) : 0;
}
SIGNAL(TIMER3_COMPA_vect) {
    fast8_t servoId = servoIndex >> 1;
    ServoInterface* act = analogServoSlots[servoId];
    if (act == nullptr) {
        OCR3A = SERVO2500US;
    } else {
        if (servoIndex & 1) { // disable
            act->disable();
            OCR3A = SERVO5000US;
            if (servoAutoOff[servoId]) {
                servoAutoOff[servoId]--;
                if (servoAutoOff[servoId] == 0)
                    HAL::servoTimings[servoId] = 0;
            }
        } else { // enable
            TCNT3 = 0;
            if (HAL::servoTimings[servoId]) {
                act->enable();
                OCR3A = HAL::servoTimings[servoId];
            } else {
                OCR3A = SERVO2500US;
            }
        }
    }
    servoIndex++;
    if (servoIndex > 7) {
        servoIndex = 0;
    }
}
#else
#error No servo support for your board, please diable FEATURE_SERVO
#endif
#endif

long __attribute__((used)) stepperWait = 0;

// ================== Interrupt handling ======================

/** \brief Sets the timer 1 compare value to delay ticks.

This function sets the OCR1A compare counter  to get the next interrupt
at delay ticks measured from the last interrupt. delay must be << 2^24
*/
inline void setTimer(uint32_t delay) {
    __asm__ __volatile__(
        "cli \n\t"
        "tst %C[delay] \n\t" //if(delay<65536) {
        "brne else%= \n\t"
        "cpi %B[delay],255 \n\t"
        "breq else%= \n\t"        // delay <65280
        "sts stepperWait,r1 \n\t" // stepperWait = 0;
        "sts stepperWait+1,r1 \n\t"
        "sts stepperWait+2,r1 \n\t"
        "lds %C[delay],%[time] \n\t" // Read TCNT1
        "lds %D[delay],%[time]+1 \n\t"
        "ldi r18,100 \n\t" // Add 100 to TCNT1
        "add %C[delay],r18 \n\t"
        "adc %D[delay],r1 \n\t"
        "cp %A[delay],%C[delay] \n\t" // delay<TCNT1+1
        "cpc %B[delay],%D[delay] \n\t"
        "brcc exact%= \n\t"
        "sts %[ocr]+1,%D[delay] \n\t" //  OCR1A = TCNT1+100;
        "sts %[ocr],%C[delay] \n\t"
        "rjmp end%= \n\t"
        "exact%=: sts %[ocr]+1,%B[delay] \n\t" //  OCR1A = delay;
        "sts %[ocr],%A[delay] \n\t"
        "rjmp end%= \n\t"
        "else%=: subi	%B[delay], 0x80 \n\t" //} else { stepperWait = delay-32768;
        "sbci	%C[delay], 0x00 \n\t"
        "sts stepperWait,%A[delay] \n\t"
        "sts stepperWait+1,%B[delay] \n\t"
        "sts stepperWait+2,%C[delay] \n\t"
        "ldi	%D[delay], 0x80 \n\t" //OCR1A = 32768;
        "sts	%[ocr]+1, %D[delay] \n\t"
        "sts	%[ocr], r1 \n\t"
        "end%=: \n\t"
        //:[delay]"=&d"(delay),[stepperWait]"=&d"(stepperWait) // Output
        : [ delay ] "=&d"(delay)                                                            // Output
        : "0"(delay), [ ocr ] "i"(_SFR_MEM_ADDR(OCR1A)), [ time ] "i"(_SFR_MEM_ADDR(TCNT1)) // Input
        : "r18"                                                                             // Clobber
    );
    /* // Assembler above replaced this code
      if(delay<65280) {
        stepperWait = 0;
        unsigned int count = TCNT1+100;
        if(delay<count)
          OCR1A = count;
        else
          OCR1A = delay;
      } else {
        stepperWait = delay-32768;
        OCR1A = 32768;
      }*/
}

ISR(TIMER2_COMPA_vect) {
    cbi(TIMSK2, OCIE2A); // prevent retrigger timer by disabling     static bool inside = false;
    sei();               // allow interruption since it is a long operation
    Motion2::timer();
    sbi(TIMSK2, OCIE2A);
}

// volatile uint8_t insideTimer1 = 0;
/** \brief Timer interrupt routine to drive the stepper motors.
*/
// Motion3
ISR(TIMER1_COMPA_vect) {
    cbi(TIMSK1, OCIE1A); // prevent retrigger timer by disabling insideTimer1.
    Motion3::timer();
    sbi(TIMSK1, OCIE1A);
}

ufast8_t pwmSteps[] = { 1, 2, 4, 8, 16 };
ufast8_t pwmMasks[] = { 255, 254, 252, 248, 240 };

/**
This timer is called 3906 timer per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR) {
    static uint8_t pwm_count0 = 0; // Used my IO_PWM_SOFTWARE!
    static uint8_t pwm_count1 = 0;
    static uint8_t pwm_count2 = 0;
    static uint8_t pwm_count3 = 0;
    static uint8_t pwm_count4 = 0;

// Add all generated pwm handlers
#undef IO_TARGET
#define IO_TARGET IO_TARGET_PWM
#include "io/redefine.h"

    counterPeriodical++;                          // Approximate a 100ms timer
    if (counterPeriodical >= PWM_COUNTER_100MS) { //  (int)(F_CPU/40960))
        counterPeriodical = 0;
        executePeriodical = 1;
    }
    pwm_count0++;
    pwm_count1 += 2;
    pwm_count2 += 4;
    pwm_count3 += 8;
    pwm_count4 += 16;

    GUI::handleKeypress();
#if FEATURE_WATCHDOG
    if (HAL::wdPinged) {
        IWatchdog.reload();
        HAL::wdPinged = false;
    }
#endif

    counterPeriodical++; // Approximate a 100ms timer
    if (counterPeriodical >= (int)(F_CPU / 40960)) {
        counterPeriodical = 0;
        executePeriodical = 1;
    }
    // read analog values
    if (analogInputCount > 0) {
        if ((ADCSRA & _BV(ADSC)) == 0) { // Conversion finished?
            analogValues[analogSamplePos] = ADCW << 2;
            do {
                analogSamplePos++;
            } while (analogSamplePos < MAX_ANALOG_INPUTS && !analogEnabled[analogSamplePos]);
            if (analogSamplePos == MAX_ANALOG_INPUTS) {
                analogSamplePos = 0;
                while (!analogEnabled[analogSamplePos]) {
                    analogSamplePos++;
                }
                // Execute operations on values
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ANALOG_INPUT_LOOP
#include "io/redefine.h"
            }
#if defined(ADCSRB) && defined(MUX5)
            if (analogSamplePos & 8) { // Reading channel 0-7 or 8-15?
                ADCSRB |= _BV(MUX5);
            } else {
                ADCSRB &= ~_BV(MUX5);
            }
#endif
            ADMUX = (ADMUX & ~(0x1F)) | (analogSamplePos & 7);
            ADCSRA |= _BV(ADSC); // start next conversion
        }
    }
}

void HAL::analogEnable(int channel) {
    if (!analogEnabled[channel]) {
        analogInputCount++;
    }
    analogEnabled[channel] = true;
}

void HAL::switchToBootMode() {
    // not relevant for AVR systems
}

// Called within checkForPeriodicalActions (main loop, more or less)
// as fast as possible
void HAL::handlePeriodical() {
}

void HAL::spiInit() {
    SPI.begin();
}

void HAL::spiBegin(uint32_t clock, uint8_t mode, uint8_t msbfirst) {
    SPI.beginTransaction(SPISettings(clock, msbfirst ? MSBFIRST : LSBFIRST, mode));
}

uint8_t HAL::spiTransfer(uint8_t data) {
    return SPI.transfer(data);
}

void HAL::spiEnd() {
    SPI.endTransaction();
}

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

inline void rf_store_char(unsigned char c, ring_buffer* buffer) {
    uint8_t i = (buffer->head + 1) & (SERIAL_RX_BUFFER_SIZE - 1);

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != buffer->tail) {
        buffer->buffer[buffer->head] = c;
        buffer->head = i;
    }
}

// Constructors ////////////////////////////////////////////////////////////////

RFHardwareSerial::RFHardwareSerial(volatile uint8_t* ubrrh, volatile uint8_t* ubrrl,
                                   volatile uint8_t* ucsra, volatile uint8_t* ucsrb,
                                   volatile uint8_t* udr,
                                   uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x) {
    _ubrrh = ubrrh;
    _ubrrl = ubrrl;
    _ucsra = ucsra;
    _ucsrb = ucsrb;
    _udr = udr;
    _rxen = rxen;
    _txen = txen;
    _rxcie = rxcie;
    _udrie = udrie;
    _u2x = u2x;
    _rx_buffer.head = _rx_buffer.tail = 0;
    _tx_buffer.head = _tx_buffer.tail = 0;
}

// Public Methods //////////////////////////////////////////////////////////////

void RFHardwareSerial::begin(unsigned long baud) {
    uint16_t baud_setting;
    bool use_u2x = true;

#if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600) {
        use_u2x = false;
    }
#endif

try_again:

    if (use_u2x) {
        *_ucsra = 1 << _u2x;
        baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
        *_ucsra = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    if ((baud_setting > 4095) && use_u2x) {
        use_u2x = false;
        goto try_again;
    }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    *_ubrrh = baud_setting >> 8;
    *_ubrrl = baud_setting;

    bit_set(*_ucsrb, _rxen);
    bit_set(*_ucsrb, _txen);
    bit_set(*_ucsrb, _rxcie);
    bit_clear(*_ucsrb, _udrie);
}

void RFHardwareSerial::end() {
    // wait for transmission of outgoing data
    while (_tx_buffer.head != _tx_buffer.tail)
        ;

    bit_clear(*_ucsrb, _rxen);
    bit_clear(*_ucsrb, _txen);
    bit_clear(*_ucsrb, _rxcie);
    bit_clear(*_ucsrb, _udrie);

    // clear a  ny received data
    _rx_buffer.head = _rx_buffer.tail;
}

int RFHardwareSerial::available(void) {
    return (unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer.head - _rx_buffer.tail) & (SERIAL_RX_BUFFER_SIZE - 1);
}
int RFHardwareSerial::outputUnused(void) {
    return SERIAL_TX_BUFFER_SIZE - (unsigned int)((SERIAL_TX_BUFFER_SIZE + _tx_buffer.head - _tx_buffer.tail) & (SERIAL_TX_BUFFER_SIZE - 1));
}

int RFHardwareSerial::peek(void) {
    if (_rx_buffer.head == _rx_buffer.tail) {
        return -1;
    }
    return _rx_buffer.buffer[_rx_buffer.tail];
}

int RFHardwareSerial::read(void) {
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer.head == _rx_buffer.tail) {
        return -1;
    }
    unsigned char c = _rx_buffer.buffer[_rx_buffer.tail];
    _rx_buffer.tail = (_rx_buffer.tail + 1) & (SERIAL_RX_BUFFER_SIZE - 1);
    return c;
}

void RFHardwareSerial::flush() {
    while (_tx_buffer.head != _tx_buffer.tail)
        ;
}

size_t RFHardwareSerial::write(uint8_t c) {
    uint8_t i = (_tx_buffer.head + 1) & (SERIAL_TX_BUFFER_SIZE - 1);

    // If the output buffer is full, there's nothing for it other than to
    // wait for the interrupt handler to empty it a bit
    while (i == _tx_buffer.tail) { }
    _tx_buffer.buffer[_tx_buffer.head] = c;
    _tx_buffer.head = i;

    bit_set(*_ucsrb, _udrie);
    return 1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

// Default serial connected to usb

RFHardwareSerial Serial(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
SIGNAL(USART0_RX_vect) {
    uint8_t c = UDR0;
    rf_store_char(c, &Serial._rx_buffer);
}

ISR(USART0_UDRE_vect) {
    if (Serial._tx_buffer.head == Serial._tx_buffer.tail) {
        bit_clear(UCSR0B, UDRIE0);
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = Serial._tx_buffer.buffer[Serial._tx_buffer.tail];
        UDR0 = c;
        Serial._tx_buffer.tail = (Serial._tx_buffer.tail + 1) & (SERIAL_TX_BUFFER_SIZE - 1);
    }
}

// Serial 1
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL == 1
RFHardwareSerial Serial1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRIE1, U2X1);
SIGNAL(USART1_RX_vect) {
    uint8_t c = UDR1;
    rf_store_char(c, &Serial1._rx_buffer);
}

ISR(USART1_UDRE_vect) {
    if (Serial1._tx_buffer.head == Serial1._tx_buffer.tail) {
        bit_clear(UCSR1B, UDRIE1);
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = Serial1._tx_buffer.buffer[Serial1._tx_buffer.tail];
        UDR1 = c;
        Serial1._tx_buffer.tail = (Serial1._tx_buffer.tail + 1) & (SERIAL_TX_BUFFER_SIZE - 1);
    }
}
#endif

// Serial 2
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL == 2
RFHardwareSerial Serial2(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UDR2, RXEN2, TXEN2, RXCIE2, UDRIE2, U2X2);
SIGNAL(USART2_RX_vect) {
    uint8_t c = UDR2;
    rf_store_char(c, &Serial2._rx_buffer);
}

ISR(USART2_UDRE_vect) {
    if (Serial2._tx_buffer.head == Serial2._tx_buffer.tail) {
        bit_clear(UCSR2B, UDRIE2);
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = Serial2._tx_buffer.buffer[Serial2._tx_buffer.tail];
        UDR2 = c;
        Serial2._tx_buffer.tail = (Serial2._tx_buffer.tail + 2) & (SERIAL_TX_BUFFER_SIZE - 1);
    }
}
#endif

// Serial 3
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL == 3
RFHardwareSerial Serial3(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UDR3, RXEN3, TXEN3, RXCIE3, UDRIE3, U2X3);
SIGNAL(USART3_RX_vect) {
    uint8_t c = UDR3;
    rf_store_char(c, &Serial3._rx_buffer);
}

ISR(USART3_UDRE_vect) {
    if (Serial3._tx_buffer.head == Serial3._tx_buffer.tail) {
        bit_clear(UCSR3B, UDRIE3);
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = Serial3._tx_buffer.buffer[Serial3._tx_buffer.tail];
        UDR3 = c;
        Serial3._tx_buffer.tail = (Serial3._tx_buffer.tail + 1) & (SERIAL_TX_BUFFER_SIZE - 1);
    }
}
#endif

void serialEventRun(void) { }

#endif
