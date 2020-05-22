#include "Repetier.h"
#ifdef AVR_BOARD
#include <compat/twi.h>

// New adc handling
#if MAX_ANALOG_INPUTS == 16
bool analogEnabled[MAX_ANALOG_INPUTS] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
#endif

// end adc handling

#if ANALOG_INPUTS > 0
uint8 osAnalogInputCounter[ANALOG_INPUTS];
uint osAnalogInputBuildup[ANALOG_INPUTS];
uint8 osAnalogInputPos = 0; // Current sampling position
#endif
#if FEATURE_WATCHDOG
bool HAL::wdPinged = false;
#endif
uint8_t HAL::i2cError = 0;
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
    PWM_TIMSK |= (1 << PWM_OCIE);

    TCCR1A = 0; // Stepper timer 1 interrupt to no prescale CTC mode
    TCCR1C = 0;
    TIMSK1 = 0;
    TCCR1B = (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A = 65500;                     //start off with a slow frequency.
    TIMSK1 |= (1 << OCIE1A);           // Enable interrupt
#if NUM_SERVOS > 0
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

void HAL::showStartReason() {
    // Check startup - does nothing if bootloader sets MCUSR to 0
    uint8_t mcu = MCUSR;
    if (mcu & 1)
        Com::printInfoFLN(Com::tPowerUp);
    if (mcu & 2)
        Com::printInfoFLN(Com::tExternalReset);
    if (mcu & 4)
        Com::printInfoFLN(Com::tBrownOut);
    if (mcu & 8)
        Com::printInfoFLN(Com::tWatchdog);
    if (mcu & 32)
        Com::printInfoFLN(Com::tSoftwareReset);
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
#if ANALOG_INPUTS > 0
    ADMUX = ANALOG_REF; // refernce voltage
    for (uint8_t i = 0; i < ANALOG_INPUTS; i++) {
        osAnalogInputCounter[i] = 0;
        osAnalogInputBuildup[i] = 0;
        osAnalogInputValues[i] = 0;
    }
    ADCSRA = _BV(ADEN) | _BV(ADSC) | ANALOG_PRESCALER;
    //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
    while (ADCSRA & _BV(ADSC)) { } // wait for conversion
    /* ADCW must be read once, otherwise the next result is wrong. */
    //uint dummyADCResult;
    //dummyADCResult = ADCW;
    // Enable interrupt driven conversion loop
    uint8_t channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
    if (channel & 8) // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
    else
        ADCSRB &= ~_BV(MUX5);
#endif
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!
#endif
}

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

#include <avr/io.h>

/****************************************************************************************
 Setting for I2C Clock speed. needed to change  clock speed for different peripherals
****************************************************************************************/

void HAL::i2cSetClockspeed(uint32_t clockSpeedHz)

{
    WIRE_PORT.setClock(clockSpeedHz);
}

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz) {
    WIRE_PORT.begin(); // create I2C master access
    WIRE_PORT.setClock(clockSpeedHz);
}

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
*************************************************************************/
/* void HAL::i2cStart(unsigned char address) {
    WIRE_PORT.beginTransmission(address);
} */

void HAL::i2cStartRead(uint8_t address, uint8_t bytes) {
    if (!i2cError) {
        i2cError |= (WIRE_PORT.requestFrom(address, bytes) != bytes);
    }
}
/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 Also specifies internal address of device

 Input:   address and transfer direction of I2C device, internal address
*************************************************************************/
void HAL::i2cStartAddr(unsigned char address, unsigned int pos, unsigned int readBytes) {
    if (!i2cError) {
        WIRE_PORT.beginTransmission(address);
        WIRE_PORT.write(pos >> 8);
        WIRE_PORT.write(pos & 255);
        if (readBytes) {
            i2cError |= WIRE_PORT.endTransmission();
            i2cError |= (WIRE_PORT.requestFrom(address, readBytes) != readBytes);
        }
    }
}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void) {
    i2cError |= WIRE_PORT.endTransmission();
}

/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
*************************************************************************/
void HAL::i2cWrite(uint8_t data) {
    if (!i2cError) {
        WIRE_PORT.write(data);
    }
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
int HAL::i2cRead(void) {
    if (!i2cError && WIRE_PORT.available()) {
        return WIRE_PORT.read();
    }
    return -1; // should never happen, but better then blocking
}

#if NUM_SERVOS > 0
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

// volatile uint8_t insideTimer1 = 0;
/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(MOTION3_COMPA_vect) {
    // if(insideTimer1) return;
    uint8_t doExit;
    __asm__ __volatile__(
        "ldi %[ex],0 \n\t"
        "lds r23,stepperWait+2 \n\t"
        "tst r23 \n\t"     //if(stepperWait<65536) {
        "brne else%= \n\t" // Still > 65535
        "lds r23,stepperWait+1 \n\t"
        "tst r23 \n\t"
        "brne last%= \n\t" // Still not 0, go ahead
        "lds r22,stepperWait \n\t"
        "breq end%= \n\t" // stepperWait is 0, do your work
        "last%=: \n\t"
        "sts %[ocr]+1,r23 \n\t" //  OCR1A = stepper wait;
        "sts %[ocr],r22 \n\t"
        "sts stepperWait,r1 \n\t"
        "sts stepperWait+1,r1 \n\t"
        "rjmp end1%= \n\t"
        "else%=: lds r22,stepperWait+1 \n\t" //} else { stepperWait = stepperWait-32768;
        "subi	r22, 0x80 \n\t"
        "sbci	r23, 0x00 \n\t"
        "sts stepperWait+1,r22 \n\t" // ocr1a stays 32768
        "sts stepperWait+2,r23 \n\t"
        "end1%=: ldi %[ex],1 \n\t"
        "end%=: \n\t"
        : [ ex ] "=&d"(doExit)
        : [ ocr ] "i"(_SFR_MEM_ADDR(OCR1A))
        : "r22", "r23");
    //        :[ex]"=&d"(doExit),[stepperWait]"=&d"(stepperWait):[ocr]"i" (_SFR_MEM_ADDR(OCR1A)):"r22","r23" );
    if (doExit)
        return;
    cbi(TIMSK1, OCIE1A); // prevent retrigger timer by disabling timer interrupt. Should be faster the guarding with insideTimer1.
    // insideTimer1 = 1;
    OCR1A = 61000;
    if (PrintLine::hasLines()) {
        setTimer(PrintLine::bresenhamStep());
    }
#if FEATURE_BABYSTEPPING
    else if (Printer::zBabystepsMissing) {
        Printer::zBabystep();
        setTimer(Printer::interval);
    }
#endif
    else {
        if (waitRelax == 0) {
        } else
            waitRelax--;
        stepperWait = 0; // Important because of optimization in asm at begin
        OCR1A = 65500;   // Wait for next move
    }
    DEBUG_MEMORY;
    sbi(TIMSK1, OCIE1A);
    //insideTimer1 = 0;
}

#if !defined(HEATER_PWM_SPEED)
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED < 0
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED > 4
#define HEATER_PWM_SPEED 4
#endif

#if HEATER_PWM_SPEED == 0
#define HEATER_PWM_STEP 1
#define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
#define HEATER_PWM_STEP 2
#define HEATER_PWM_MASK 254
#elif HEATER_PWM_SPEED == 2
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#elif HEATER_PWM_SPEED == 3
#define HEATER_PWM_STEP 8
#define HEATER_PWM_MASK 248
#elif HEATER_PWM_SPEED == 4
#define HEATER_PWM_STEP 16
#define HEATER_PWM_MASK 240
#endif

#if !defined(COOLER_PWM_SPEED)
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED < 0
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED > 4
#define COOLER_PWM_SPEED 4
#endif

#if COOLER_PWM_SPEED == 0
#define COOLER_PWM_STEP 1
#define COOLER_PWM_MASK 255
#elif COOLER_PWM_SPEED == 1
#define COOLER_PWM_STEP 2
#define COOLER_PWM_MASK 254
#elif COOLER_PWM_SPEED == 2
#define COOLER_PWM_STEP 4
#define COOLER_PWM_MASK 252
#elif COOLER_PWM_SPEED == 3
#define COOLER_PWM_STEP 8
#define COOLER_PWM_MASK 248
#elif COOLER_PWM_SPEED == 4
#define COOLER_PWM_STEP 16
#define COOLER_PWM_MASK 240
#endif

#define pulseDensityModulate(pin, density, error, invert) \
    { \
        uint8_t carry; \
        carry = error + (invert ? 255 - density : density); \
        WRITE(pin, (carry < error)); \
        error = carry; \
    }
/**
This timer is called 3906 timer per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR) {
    static uint8_t pwm_count_cooler = 0;
    static uint8_t pwm_count_heater = 0;
    static uint8_t pwm_pos_set[NUM_PWM];
#if NUM_EXTRUDER > 0 && ((defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1) || (NUM_EXTRUDER > 1 && EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN) || (NUM_EXTRUDER > 2 && EXT2_EXTRUDER_COOLER_PIN > -1 && EXT2_EXTRUDER_COOLER_PIN != EXT2_EXTRUDER_COOLER_PIN) || (NUM_EXTRUDER > 3 && EXT3_EXTRUDER_COOLER_PIN > -1 && EXT3_EXTRUDER_COOLER_PIN != EXT3_EXTRUDER_COOLER_PIN) || (NUM_EXTRUDER > 4 && EXT4_EXTRUDER_COOLER_PIN > -1 && EXT4_EXTRUDER_COOLER_PIN != EXT4_EXTRUDER_COOLER_PIN) || (NUM_EXTRUDER > 5 && EXT5_EXTRUDER_COOLER_PIN > -1 && EXT5_EXTRUDER_COOLER_PIN != EXT5_EXTRUDER_COOLER_PIN))
    static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];
#endif
    PWM_OCR += 64;
    if (pwm_count_heater == 0 && !PDM_FOR_EXTRUDER) {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
        if ((pwm_pos_set[0] = (pwm_pos[0] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT0_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1 && !MIXING_EXTRUDER
        if ((pwm_pos_set[1] = (pwm_pos[1] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT1_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2 && !MIXING_EXTRUDER
        if ((pwm_pos_set[2] = (pwm_pos[2] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT2_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3 && !MIXING_EXTRUDER
        if ((pwm_pos_set[3] = (pwm_pos[3] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT3_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4 && !MIXING_EXTRUDER
        if ((pwm_pos_set[4] = (pwm_pos[4] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT4_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5 && !MIXING_EXTRUDER
        if ((pwm_pos_set[5] = (pwm_pos[5] & HEATER_PWM_MASK)) > 0)
            WRITE(EXT5_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
        if ((pwm_pos_set[NUM_EXTRUDER] = (pwm_pos[NUM_EXTRUDER] & HEATER_PWM_MASK)) > 0)
            WRITE(HEATED_BED_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
    }
    if (pwm_count_cooler == 0 && !PDM_FOR_COOLER) {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[0] = (extruder[0].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT0_EXTRUDER_COOLER_PIN, 1);
#endif
#if !SHARED_COOLER && defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
#if EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
        if ((pwm_cooler_pos_set[1] = (extruder[1].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT1_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
#if EXT2_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[2] = (extruder[2].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT2_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
#if EXT3_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[3] = (extruder[3].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT3_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
#if EXT4_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[4] = (extruder[4].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT4_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
#if EXT5_EXTRUDER_COOLER_PIN > -1
        if ((pwm_cooler_pos_set[5] = (extruder[5].coolerPWM & COOLER_PWM_MASK)) > 0)
            WRITE(EXT5_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if FAN_BOARD_PIN > -1 && SHARED_COOLER_BOARD_EXT == 0
        if ((pwm_pos_set[PWM_BOARD_FAN] = (pwm_pos[PWM_BOARD_FAN] & COOLER_PWM_MASK)) > 0)
            WRITE(FAN_BOARD_PIN, 1);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
        if ((pwm_pos_set[PWM_FAN1] = (pwm_pos[PWM_FAN1] & COOLER_PWM_MASK)) > 0)
            WRITE(FAN_PIN, 1);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
        if ((pwm_pos_set[PWM_FAN2] = (pwm_pos[PWM_FAN2] & COOLER_PWM_MASK)) > 0)
            WRITE(FAN2_PIN, 1);
#endif
#if defined(FAN_THERMO_PIN) && FAN_THERMO_PIN > -1
        if ((pwm_pos_set[PWM_FAN_THERMO] = (pwm_pos[PWM_FAN_THERMO] & COOLER_PWM_MASK)) > 0)
            WRITE(FAN_THERMO_PIN, 1);
#endif
    }
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT0_HEATER_PIN, pwm_pos[0], pwm_pos_set[0], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0] != HEATER_PWM_MASK)
        WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if EXT0_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT0_EXTRUDER_COOLER_PIN, extruder[0].coolerPWM, pwm_cooler_pos_set[0], false);
#else
    if (pwm_cooler_pos_set[0] == pwm_count_cooler && pwm_cooler_pos_set[0] != COOLER_PWM_MASK)
        WRITE(EXT0_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT1_HEATER_PIN, pwm_pos[1], pwm_pos_set[1], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1] != HEATER_PWM_MASK)
        WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT1_EXTRUDER_COOLER_PIN, extruder[1].coolerPWM, pwm_cooler_pos_set[1], false);
#else
    if (pwm_cooler_pos_set[1] == pwm_count_cooler && pwm_cooler_pos_set[1] != COOLER_PWM_MASK)
        WRITE(EXT1_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT2_HEATER_PIN, pwm_pos[2], pwm_pos_set[2], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[2] == pwm_count_heater && pwm_pos_set[2] != HEATER_PWM_MASK)
        WRITE(EXT2_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT2_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT2_EXTRUDER_COOLER_PIN, extruder[2].coolerPWM, pwm_cooler_pos_set[2], false);
#else
    if (pwm_cooler_pos_set[2] == pwm_count_cooler && pwm_cooler_pos_set[2] != COOLER_PWM_MASK)
        WRITE(EXT2_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT3_HEATER_PIN, pwm_pos[3], pwm_pos_set[3], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[3] == pwm_count_heater && pwm_pos_set[3] != HEATER_PWM_MASK)
        WRITE(EXT3_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT3_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT3_EXTRUDER_COOLER_PIN, extruder[3].coolerPWM, pwm_cooler_pos_set[3], false);
#else
    if (pwm_cooler_pos_set[3] == pwm_count_cooler && pwm_cooler_pos_set[3] != COOLER_PWM_MASK)
        WRITE(EXT3_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT4_HEATER_PIN, pwm_pos[4], pwm_pos_set[4], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[4] == pwm_count_heater && pwm_pos_set[4] != HEATER_PWM_MASK)
        WRITE(EXT4_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT4_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT4_EXTRUDER_COOLER_PIN, extruder[4].coolerPWM, pwm_cooler_pos_set[4], false);
#else
    if (pwm_cooler_pos_set[4] == pwm_count_cooler && pwm_cooler_pos_set[4] != COOLER_PWM_MASK)
        WRITE(EXT4_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5 && !MIXING_EXTRUDER
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT5_HEATER_PIN, pwm_pos[5], pwm_pos_set[5], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[5] == pwm_count_heater && pwm_pos_set[5] != HEATER_PWM_MASK)
        WRITE(EXT5_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT5_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT5_EXTRUDER_COOLER_PIN, extruder[5].coolerPWM, pwm_cooler_pos_set[5], false);
#else
    if (pwm_cooler_pos_set[5] == pwm_count_cooler && pwm_cooler_pos_set[5] != COOLER_PWM_MASK)
        WRITE(EXT5_EXTRUDER_COOLER_PIN, 0);
#endif
#endif
#endif
#if FAN_BOARD_PIN > -1 && SHARED_COOLER_BOARD_EXT == 0
#if PDM_FOR_COOLER
    pulseDensityModulate(FAN_BOARD_PIN, pwm_pos[PWM_BOARD_FAN], pwm_pos_set[PWM_BOARD_FAN], false);
#else
    if (pwm_pos_set[PWM_BOARD_FAN] == pwm_count_cooler && pwm_pos_set[PWM_BOARD_FAN] != COOLER_PWM_MASK)
        WRITE(FAN_BOARD_PIN, 0);
#endif
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    if (fanKickstart == 0) {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN_PIN, pwm_pos[PWM_FAN1], pwm_pos_set[PWM_FAN1], false);
#else
        if (pwm_pos_set[PWM_FAN1] == pwm_count_cooler && pwm_pos_set[PWM_FAN1] != COOLER_PWM_MASK)
            WRITE(FAN_PIN, 0);
#endif
    } else {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN_PIN, MAX_FAN_PWM, pwm_pos_set[PWM_FAN1], false);
#else
        if ((MAX_FAN_PWM & COOLER_PWM_MASK) == pwm_count_cooler && (MAX_FAN_PWM & COOLER_PWM_MASK) != COOLER_PWM_MASK)
            WRITE(FAN_PIN, 0);
#endif
    }
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
    if (fan2Kickstart == 0) {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN2_PIN, pwm_pos[PWM_FAN2], pwm_pos_set[PWM_FAN2], false);
#else
        if (pwm_pos_set[PWM_FAN2] == pwm_count_cooler && pwm_pos_set[PWM_FAN2] != COOLER_PWM_MASK)
            WRITE(FAN2_PIN, 0);
#endif
    } else {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN2_PIN, MAX_FAN_PWM, pwm_pos_set[PWM_FAN2], false);
#else
        if ((MAX_FAN_PWM & COOLER_PWM_MASK) == pwm_count_cooler && (MAX_FAN_PWM & COOLER_PWM_MASK) != COOLER_PWM_MASK)
            WRITE(FAN2_PIN, 0);
#endif
    }
#endif
#if defined(FAN_THERMO_PIN) && FAN_THERMO_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(FAN_THERMO_PIN, pwm_pos[PWM_FAN_THERMO], pwm_pos_set[PWM_FAN_THERMO], false);
#else
    if (pwm_pos_set[PWM_FAN_THERMO] == pwm_count_cooler && pwm_pos_set[PWM_FAN_THERMO] != COOLER_PWM_MASK)
        WRITE(FAN_THERMO_PIN, 0);
#endif
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(HEATED_BED_HEATER_PIN, pwm_pos[NUM_EXTRUDER], pwm_pos_set[NUM_EXTRUDER], HEATER_PINS_INVERTED);
#else
    if (pwm_pos_set[NUM_EXTRUDER] == pwm_count_heater && pwm_pos_set[NUM_EXTRUDER] != HEATER_PWM_MASK)
        WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#endif
    counterPeriodical++; // Approximate a 100ms timer
    if (counterPeriodical >= (int)(F_CPU / 40960)) {
        counterPeriodical = 0;
        executePeriodical = 1;
#if FEATURE_FAN_CONTROL
        if (fanKickstart)
            fanKickstart--;
#endif
#if FEATURE_FAN2_CONTROL
        if (fan2Kickstart)
            fan2Kickstart--;
#endif
    }
// read analog values
#if ANALOG_INPUTS > 0
    if ((ADCSRA & _BV(ADSC)) == 0) { // Conversion finished?
        osAnalogInputBuildup[osAnalogInputPos] += ADCW;
        if (++osAnalogInputCounter[osAnalogInputPos] >= _BV(ANALOG_INPUT_SAMPLE)) {
            // update temperatures only when values have been read
            if (executePeriodical == 0 || osAnalogInputPos >= NUM_ANALOG_TEMP_SENSORS) {
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE < 12
                osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos] << (12 - ANALOG_INPUT_BITS - ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE > 12
                osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos] >> (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE - 12);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE == 12
                osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos];
#endif
            }
            osAnalogInputBuildup[osAnalogInputPos] = 0;
            osAnalogInputCounter[osAnalogInputPos] = 0;
            // Start next conversion
            if (++osAnalogInputPos >= ANALOG_INPUTS)
                osAnalogInputPos = 0;
            uint8_t channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
            if (channel & 8) // Reading channel 0-7 or 8-15?
                ADCSRB |= _BV(MUX5);
            else
                ADCSRB &= ~_BV(MUX5);
#endif
            ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
        }
        ADCSRA |= _BV(ADSC); // start next conversion
    }
#endif

    GUI::handleKeypress();
    pwm_count_cooler += COOLER_PWM_STEP;
    pwm_count_heater += HEATER_PWM_STEP;
#if FEATURE_WATCHDOG
    if (HAL::wdPinged) {
        wdt_reset();
        HAL::wdPinged = false;
    }
#endif
}

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

ring_buffer rx_buffer = { { 0 }, 0, 0 };
ring_buffer_tx tx_buffer = { { 0 }, 0, 0 };

inline void rf_store_char(unsigned char c, ring_buffer* buffer) {
    uint8_t i = (buffer->head + 1) & SERIAL_BUFFER_MASK;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != buffer->tail) {
        buffer->buffer[buffer->head] = c;
        buffer->head = i;
    }
}
#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
void rfSerialEvent() __attribute__((weak));
void rfSerialEvent() { }
#define serialEvent_implemented
#if defined(USART_RX_vect)
SIGNAL(USART_RX_vect)
#elif defined(USART0_RX_vect)
SIGNAL(USART0_RX_vect)
#else
#if defined(SIG_USART0_RECV)
SIGNAL(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
SIGNAL(SIG_UART0_RECV)
#elif defined(SIG_UART_RECV)
SIGNAL(SIG_UART_RECV)
#else
#error "Don't know what the Data Received vector is called for the first UART"
#endif
#endif
{
#if defined(UDR0)
    uint8_t c = UDR0;
#elif defined(UDR)
    uint8_t c = UDR;
#else
#error UDR not defined
#endif
    rf_store_char(c, &rx_buffer);
}
#endif

#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
#error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
    if (tx_buffer.head == tx_buffer.tail) {
        // Buffer empty, so disable interrupts
#if defined(UCSR0B)
        bit_clear(UCSR0B, UDRIE0);
#else
        bit_clear(UCSRB, UDRIE);
#endif
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = tx_buffer.buffer[tx_buffer.tail];
#if defined(UDR0)
        UDR0 = c;
#elif defined(UDR)
        UDR = c;
#else
#error UDR not defined
#endif
        tx_buffer.tail = (tx_buffer.tail + 1) & SERIAL_TX_BUFFER_MASK;
    }
}
#endif
#endif

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__))
#error BlueTooth option cannot be used with your mainboard
#endif
#if BLUETOOTH_SERIAL > 1 && !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#error BlueTooth serial 2 or 3 can be used only with boards based on ATMega2560 or ATMega1280
#endif
#if (BLUETOOTH_SERIAL == 1)
#if defined(USART1_RX_vect)
#define SIG_USARTx_RECV USART1_RX_vect
#define USARTx_UDRE_vect USART1_UDRE_vect
#else
#define SIG_USARTx_RECV SIG_USART1_RECV
#define USARTx_UDRE_vect SIG_USART1_DATA
#endif
#define UDRx UDR1
#define UCSRxA UCSR1A
#define UCSRxB UCSR1B
#define UBRRxH UBRR1H
#define UBRRxL UBRR1L
#define U2Xx U2X1
#define UARTxENABLE ((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1))
#define UDRIEx UDRIE1
#define RXxPIN 19
#elif (BLUETOOTH_SERIAL == 2)
#if defined(USART2_RX_vect)
#define SIG_USARTx_RECV USART2_RX_vect
#define USARTx_UDRE_vect USART2_UDRE_vect
#else
#define SIG_USARTx_RECV SIG_USART2_RECV
#define USARTx_UDRE_vect SIG_USART2_DATA
#endif
#define UDRx UDR2
#define UCSRxA UCSR2A
#define UCSRxB UCSR2B
#define UBRRxH UBRR2H
#define UBRRxL UBRR2L
#define U2Xx U2X2
#define UARTxENABLE ((1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2) | (1 << UDRIE2))
#define UDRIEx UDRIE2
#define RXxPIN 17
#elif (BLUETOOTH_SERIAL == 3)
#if defined(USART3_RX_vect)
#define SIG_USARTx_RECV USART3_RX_vect
#define USARTx_UDRE_vect USART3_UDRE_vect
#else
#define SIG_USARTx_RECV SIG_USART3_RECV
#define USARTx_UDRE_vect SIG_USART3_DATA
#endif
#define UDRx UDR3
#define UCSRxA UCSR3A
#define UCSRxB UCSR3B
#define UBRRxH UBRR3H
#define UBRRxL UBRR3L
#define U2Xx U2X3
#define UARTxENABLE ((1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3) | (1 << UDRIE3))
#define UDRIEx UDRIE3
#define RXxPIN 15
#else
#error Wrong serial port number for BlueTooth
#endif

SIGNAL(SIG_USARTx_RECV) {
    uint8_t c = UDRx;
    rf_store_char(c, &rx_buffer);
}

volatile uint8_t txx_buffer_tail = 0;

ISR(USARTx_UDRE_vect) {
    if (tx_buffer.head == txx_buffer_tail) {
        // Buffer empty, so disable interrupts
        bit_clear(UCSRxB, UDRIEx);
    } else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = tx_buffer.buffer[txx_buffer_tail];
        txx_buffer_tail = (txx_buffer_tail + 1) & SERIAL_TX_BUFFER_MASK;
        UDRx = c;
    }
}
#endif

// Constructors ////////////////////////////////////////////////////////////////

RFHardwareSerial::RFHardwareSerial(ring_buffer* rx_buffer, ring_buffer_tx* tx_buffer,
                                   volatile uint8_t* ubrrh, volatile uint8_t* ubrrl,
                                   volatile uint8_t* ucsra, volatile uint8_t* ucsrb,
                                   volatile uint8_t* udr,
                                   uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x) {
    _rx_buffer = rx_buffer;
    _tx_buffer = tx_buffer;
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
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    WRITE(RXxPIN, 1); // Pullup on RXDx
    UCSRxA = (1 << U2Xx);
    UBRRxH = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD - 1) / 2) >> 8);
    UBRRxL = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD - 1) / 2) & 0xFF);
    UCSRxB |= UARTxENABLE;
#endif
}

void RFHardwareSerial::end() {
    // wait for transmission of outgoing data
    while (_tx_buffer->head != _tx_buffer->tail)
        ;

    bit_clear(*_ucsrb, _rxen);
    bit_clear(*_ucsrb, _txen);
    bit_clear(*_ucsrb, _rxcie);
    bit_clear(*_ucsrb, _udrie);

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    UCSRxB = 0;
#endif
    // clear a  ny received data
    _rx_buffer->head = _rx_buffer->tail;
}

int RFHardwareSerial::available(void) {
    return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & SERIAL_BUFFER_MASK;
}
int RFHardwareSerial::outputUnused(void) {
    return SERIAL_TX_BUFFER_SIZE - (unsigned int)((SERIAL_TX_BUFFER_SIZE + _tx_buffer->head - _tx_buffer->tail) & SERIAL_TX_BUFFER_MASK);
}

int RFHardwareSerial::peek(void) {
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    }
    return _rx_buffer->buffer[_rx_buffer->tail];
}

int RFHardwareSerial::read(void) {
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail) {
        return -1;
    }
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (_rx_buffer->tail + 1) & SERIAL_BUFFER_MASK;
    return c;
}

void RFHardwareSerial::flush() {
    while (_tx_buffer->head != _tx_buffer->tail)
        ;
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    while (_tx_buffer->head != txx_buffer_tail)
        ;
#endif
}
#ifdef COMPAT_PRE1
void
#else
size_t
#endif
RFHardwareSerial::write(uint8_t c) {
    uint8_t i = (_tx_buffer->head + 1) & SERIAL_TX_BUFFER_MASK;

    // If the output buffer is full, there's nothing for it other than to
    // wait for the interrupt handler to empty it a bit
    while (i == _tx_buffer->tail) { }
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    while (i == txx_buffer_tail) { }
#endif
    _tx_buffer->buffer[_tx_buffer->head] = c;
    _tx_buffer->head = i;

    bit_set(*_ucsrb, _udrie);
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    bit_set(UCSRxB, UDRIEx);
#endif
#ifndef COMPAT_PRE1
    return 1;
#endif
}

// Preinstantiate Objects //////////////////////////////////////////////////////

#if defined(UBRRH) && defined(UBRRL)
RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
#elif defined(USBCON)
// do nothing - Serial object and buffers are initialized in CDC code
#else
#error no serial port defined (port 0)
#endif

#endif

#endif
