#include "Repetier.h"
#include <compat/twi.h>

#if ANALOG_INPUTS > 0
uint8 osAnalogInputCounter[ANALOG_INPUTS];
uint osAnalogInputBuildup[ANALOG_INPUTS];
uint8 osAnalogInputPos = 0; // Current sampling position
#endif
#if FEATURE_WATCHDOG
    bool HAL::wdPinged = false;
#endif
//extern "C" void __cxa_pure_virtual() { }

HAL::HAL()
{
    //ctor
}

HAL::~HAL()
{
    //dtor
}

uint16_t HAL::integerSqrt(uint32_t a)
{
// http://www.mikrocontroller.net/articles/AVR_Arithmetik#32_Bit_.2F_32_Bit
//-----------------------------------------------------------
// Fast and short 32 bits AVR sqrt routine, avr-gcc ABI compliant
// R25:R24 = SQRT (R25:R24:R23:R22) rounded to the
// nearest integer (0.5 rounds up)
// Destroys R18-R19,R22-R23,R26-R27
// Cycles incl call & ret = 265-310
// Stack incl call = 2-3
//-----------------------------------------------------------

    uint16_t b;

    __asm__ __volatile__ (
        "ldi   R19, 0xc0 \n\t"
        "clr   R18 \n\t"        // rotation mask in R19:R18
        "ldi   R27, 0x40 \n\t"
        "sub   R26, R26 \n\t"   // developing sqrt in R27:R26, C=0
        "1:  brcs  2f \n\t"           // C --> Bit is always 1
        "cp    %C1, R26 \n\t"
        "cpc   %D1, R27 \n\t"     // Does test value fit?
        "brcs  3f \n\t"           // C --> nope, bit is 0
        "2:  sub   %C1, R26 \n\t"
        "sbc   %D1, R27 \n\t"     // Adjust argument for next bit
        "or    R26, R18 \n\t"
        "or    R27, R19 \n\t"     // Set bit to 1
        "3:  lsr   R19 \n\t"
        "ror   R18 \n\t"          // Shift right mask, C --> end loop
        "eor   R27, R19 \n\t"
        "eor   R26, R18 \n\t"     // Shift right only test bit in result
        "rol   %A1 \n\t"          // Bit 0 only set if end of loop
        "rol   %B1 \n\t"
        "rol   %C1 \n\t"
        "rol   %D1 \n\t"          // Shift left remaining argument (C used at 1:)
        "sbrs  %A1, 0 \n\t"       // Skip if 15 bits developed
        "rjmp  1b \n\t"           // Develop 15 bits of the sqrt
        "brcs  4f \n\t"           // C--> Last bits always 1
        "cp    R26, %C1 \n\t"
        "cpc   R27, %D1 \n\t"     // Test for last bit 1
        "brcc  5f \n\t"           // NC --> bit is 0
        "4:  sbc   %B1, R19 \n\t"     // Subtract C (any value from 1 to 0x7f will do)
        "sbc   %C1, R26 \n\t"
        "sbc   %D1, R27 \n\t"     // Update argument for test
        "inc   R26 \n\t"          // Last bit is 1
        "5:  lsl   %B1 \n\t"          // Only bit 7 matters
        "rol   %C1 \n\t"
        "rol   %D1 \n\t"          // Remainder * 2 + C
        "brcs  6f \n\t"           // C --> Always round up
        "cp    R26, %C1 \n\t"
        "cpc   R27, %D1 \n\t"     // C decides rounding
        "6:  adc   R26, R19 \n\t"
        "adc   R27, R19 \n\t"     // Round up if C (R19=0)
        "mov   %B0, R27 \n\t"     // return in R25:R24 for avr-gcc ABI compliance
        "mov   %A0, R26 \n\t"
        :"=r"(b)
        :"r"(a)
        :"r18","r19","r27","r26" );
    return b;
}



const uint16_t fast_div_lut[17] PROGMEM = {0,F_CPU/4096,F_CPU/8192,F_CPU/12288,F_CPU/16384,F_CPU/20480,F_CPU/24576,F_CPU/28672,F_CPU/32768,F_CPU/36864
        ,F_CPU/40960,F_CPU/45056,F_CPU/49152,F_CPU/53248,F_CPU/57344,F_CPU/61440,F_CPU/65536
                                          };

const uint16_t slow_div_lut[257] PROGMEM = {0,0,0,0,0,0,0,0,F_CPU/256,F_CPU/288,F_CPU/320,F_CPU/352
        ,F_CPU/384,F_CPU/416,F_CPU/448,F_CPU/480,F_CPU/512,F_CPU/544,F_CPU/576,F_CPU/608,F_CPU/640,F_CPU/672,F_CPU/704,F_CPU/736,F_CPU/768,F_CPU/800,F_CPU/832
        ,F_CPU/864,F_CPU/896,F_CPU/928,F_CPU/960,F_CPU/992,F_CPU/1024,F_CPU/1056,F_CPU/1088,F_CPU/1120,F_CPU/1152,F_CPU/1184,F_CPU/1216,F_CPU/1248,F_CPU/1280,F_CPU/1312
        ,F_CPU/1344,F_CPU/1376,F_CPU/1408,F_CPU/1440,F_CPU/1472,F_CPU/1504,F_CPU/1536,F_CPU/1568,F_CPU/1600,F_CPU/1632,F_CPU/1664,F_CPU/1696,F_CPU/1728,F_CPU/1760,F_CPU/1792
        ,F_CPU/1824,F_CPU/1856,F_CPU/1888,F_CPU/1920,F_CPU/1952,F_CPU/1984,F_CPU/2016
        ,F_CPU/2048,F_CPU/2080,F_CPU/2112,F_CPU/2144,F_CPU/2176,F_CPU/2208,F_CPU/2240,F_CPU/2272,F_CPU/2304,F_CPU/2336,F_CPU/2368,F_CPU/2400
        ,F_CPU/2432,F_CPU/2464,F_CPU/2496,F_CPU/2528,F_CPU/2560,F_CPU/2592,F_CPU/2624,F_CPU/2656,F_CPU/2688,F_CPU/2720,F_CPU/2752,F_CPU/2784,F_CPU/2816,F_CPU/2848,F_CPU/2880
        ,F_CPU/2912,F_CPU/2944,F_CPU/2976,F_CPU/3008,F_CPU/3040,F_CPU/3072,F_CPU/3104,F_CPU/3136,F_CPU/3168,F_CPU/3200,F_CPU/3232,F_CPU/3264,F_CPU/3296,F_CPU/3328,F_CPU/3360
        ,F_CPU/3392,F_CPU/3424,F_CPU/3456,F_CPU/3488,F_CPU/3520,F_CPU/3552,F_CPU/3584,F_CPU/3616,F_CPU/3648,F_CPU/3680,F_CPU/3712,F_CPU/3744,F_CPU/3776,F_CPU/3808,F_CPU/3840
        ,F_CPU/3872,F_CPU/3904,F_CPU/3936,F_CPU/3968,F_CPU/4000,F_CPU/4032,F_CPU/4064
        ,F_CPU/4096,F_CPU/4128,F_CPU/4160,F_CPU/4192,F_CPU/4224,F_CPU/4256,F_CPU/4288,F_CPU/4320,F_CPU/4352,F_CPU/4384,F_CPU/4416,F_CPU/4448,F_CPU/4480,F_CPU/4512,F_CPU/4544
        ,F_CPU/4576,F_CPU/4608,F_CPU/4640,F_CPU/4672,F_CPU/4704,F_CPU/4736,F_CPU/4768,F_CPU/4800,F_CPU/4832,F_CPU/4864,F_CPU/4896,F_CPU/4928,F_CPU/4960,F_CPU/4992,F_CPU/5024
        ,F_CPU/5056,F_CPU/5088,F_CPU/5120,F_CPU/5152,F_CPU/5184,F_CPU/5216,F_CPU/5248,F_CPU/5280,F_CPU/5312,F_CPU/5344,F_CPU/5376,F_CPU/5408,F_CPU/5440,F_CPU/5472,F_CPU/5504
        ,F_CPU/5536,F_CPU/5568,F_CPU/5600,F_CPU/5632,F_CPU/5664,F_CPU/5696,F_CPU/5728,F_CPU/5760,F_CPU/5792,F_CPU/5824,F_CPU/5856,F_CPU/5888,F_CPU/5920,F_CPU/5952,F_CPU/5984
        ,F_CPU/6016,F_CPU/6048,F_CPU/6080,F_CPU/6112,F_CPU/6144,F_CPU/6176,F_CPU/6208,F_CPU/6240,F_CPU/6272,F_CPU/6304,F_CPU/6336,F_CPU/6368,F_CPU/6400,F_CPU/6432,F_CPU/6464
        ,F_CPU/6496,F_CPU/6528,F_CPU/6560,F_CPU/6592,F_CPU/6624,F_CPU/6656,F_CPU/6688,F_CPU/6720,F_CPU/6752,F_CPU/6784,F_CPU/6816,F_CPU/6848,F_CPU/6880,F_CPU/6912,F_CPU/6944
        ,F_CPU/6976,F_CPU/7008,F_CPU/7040,F_CPU/7072,F_CPU/7104,F_CPU/7136,F_CPU/7168,F_CPU/7200,F_CPU/7232,F_CPU/7264,F_CPU/7296,F_CPU/7328,F_CPU/7360,F_CPU/7392,F_CPU/7424
        ,F_CPU/7456,F_CPU/7488,F_CPU/7520,F_CPU/7552,F_CPU/7584,F_CPU/7616,F_CPU/7648,F_CPU/7680,F_CPU/7712,F_CPU/7744,F_CPU/7776,F_CPU/7808,F_CPU/7840,F_CPU/7872,F_CPU/7904
        ,F_CPU/7936,F_CPU/7968,F_CPU/8000,F_CPU/8032,F_CPU/8064,F_CPU/8096,F_CPU/8128,F_CPU/8160,F_CPU/8192
                                           };
/** \brief approximates division of F_CPU/divisor

In the stepper interrupt a division is needed, which is a slow operation.
The result is used for timer calculation where small errors are ok. This
function uses lookup tables to find a fast approximation of the result.

*/
int32_t HAL::CPUDivU2(unsigned int divisor)
{
#if CPU_ARCH==ARCH_AVR
    int32_t res;
    unsigned short table;
    if(divisor<8192)
    {
        if(divisor<512)
        {
            if(divisor<10) divisor = 10;
            return Div4U2U(F_CPU,divisor); // These entries have overflows in lookuptable!
        }
        table = (unsigned short)&slow_div_lut[0];
        __asm__ __volatile__( // needs 64 ticks neu 49 Ticks
            "mov r18,%A1 \n\t"
            "andi r18,31 \n\t"  // divisor & 31 in r18
            "lsr %B1 \n\t" // divisor >> 4
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "andi %A1,254 \n\t"
            "add %A2,%A1 \n\t" // table+divisor>>3
            "adc %B2,%B1 \n\t"
            "lpm %A0,Z+ \n\t" // y0 in res
            "lpm %B0,Z+ \n\t"  // %C0,%D0 are 0
            "movw r4,%A0 \n\t" // y0 nach gain (r4-r5)
            "lpm r0,Z+ \n\t" // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t"
            "mul r18,r4 \n\t" // gain*(divisor & 31)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "mov %A2,r1 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r4","r5");
        return res;
        /*unsigned short adr0 = (unsigned short)&slow_div_lut+(divisor>>4)&1022;
        long y0=	pgm_read_dword_near(adr0);
        long gain = y0-pgm_read_dword_near(adr0+2);
        return y0-((gain*(divisor & 31))>>5);*/
    }
    else
    {
        table = (unsigned short)&fast_div_lut[0];
        __asm__ __volatile__( // needs 49 ticks
            "movw r18,%A1 \n\t"
            "andi r19,15 \n\t"  // divisor & 4095 in r18,r19
            "lsr %B1 \n\t" // divisor >> 3, then %B1 is 2*(divisor >> 12)
            "lsr %B1 \n\t"
            "lsr %B1 \n\t"
            "andi %B1,254 \n\t"
            "add %A2,%B1 \n\t" // table+divisor>>11
            "adc %B2,r1 \n\t" //
            "lpm %A0,Z+ \n\t" // y0 in res
            "lpm %B0,Z+ \n\t"
            "movw r4,%A0 \n\t" // y0 to gain (r4-r5)
            "lpm r0,Z+ \n\t" // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t" // finished - result has max. 16 bit
            "mul r18,r4 \n\t" // gain*(divisor & 4095)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r19,r5 \n\t"
            "mov %A2,r0 \n\t" // %A2 = byte 3 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "mul r19,r4 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "andi %B1,240 \n\t" // >> 12
            "swap %B1 \n\t"
            "swap %A2 \r\n"
            "mov %A1,%A2 \r\n"
            "andi %A1,240 \r\n"
            "or %B1,%A1 \r\n"
            "andi %A2,15 \r\n"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r19","r4","r5");
        return res;
        /*
        // The asm mimics the following code
        unsigned short adr0 = (unsigned short)&fast_div_lut+(divisor>>11)&254;
        unsigned short y0=	pgm_read_word_near(adr0);
        unsigned short gain = y0-pgm_read_word_near(adr0+2);
        return y0-(((long)gain*(divisor & 4095))>>12);*/
    }
#else
    return F_CPU/divisor;
#endif
}

void HAL::setupTimer()
{
#if USE_ADVANCE
    EXTRUDER_TCCR = 0; // need Normal not fastPWM set by arduino init
    EXTRUDER_TIMSK |= (1<<EXTRUDER_OCIE); // Activate compa interrupt on timer 0
#endif
    PWM_TCCR = 0;  // Setup PWM interrupt
    PWM_OCR = 64;
    PWM_TIMSK |= (1<<PWM_OCIE);

    TCCR1A = 0;  // Steup timer 1 interrupt to no prescale CTC mode
    TCCR1C = 0;
    TIMSK1 = 0;
    TCCR1B =  (_BV(WGM12) | _BV(CS10)); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A=65500; //start off with a slow frequency.
    TIMSK1 |= (1<<OCIE1A); // Enable interrupt
#if FEATURE_SERVO
#if SERVO0_PIN>-1
    SET_OUTPUT(SERVO0_PIN);
    WRITE(SERVO0_PIN,LOW);
#endif
#if SERVO1_PIN>-1
    SET_OUTPUT(SERVO1_PIN);
    WRITE(SERVO1_PIN,LOW);
#endif
#if SERVO2_PIN>-1
    SET_OUTPUT(SERVO2_PIN);
    WRITE(SERVO2_PIN,LOW);
#endif
#if SERVO3_PIN>-1
    SET_OUTPUT(SERVO3_PIN);
    WRITE(SERVO3_PIN,LOW);
#endif
    TCCR3A = 0;             // normal counting mode
    TCCR3B = _BV(CS31);     // set prescaler of 8
    TCNT3 = 0;              // clear the timer count
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);     // clear any pending interrupts;
    ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt
#else
    TIFR3 = _BV(OCF3A);     // clear any pending interrupts;
    TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt
#endif
#endif
}

void HAL::showStartReason()
{
    // Check startup - does nothing if bootloader sets MCUSR to 0
    uint8_t mcu = MCUSR;
    if(mcu & 1) Com::printInfoFLN(Com::tPowerUp);
    if(mcu & 2) Com::printInfoFLN(Com::tExternalReset);
    if(mcu & 4) Com::printInfoFLN(Com::tBrownOut);
    if(mcu & 8) Com::printInfoFLN(Com::tWatchdog);
    if(mcu & 32) Com::printInfoFLN(Com::tSoftwareReset);
    MCUSR=0;
}
int HAL::getFreeRam()
{
    int freeram = 0;
    InterruptProtectedBlock noInts;
    uint8_t * heapptr, * stackptr;
    heapptr = (uint8_t *)malloc(4);          // get heap pointer
    free(heapptr);      // free up the memory again (sets heapptr to 0)
    stackptr =  (uint8_t *)(SP);           // save value of stack pointer
    freeram = (int)stackptr-(int)heapptr;
    return freeram;
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void HAL::resetHardware()
{
    resetFunc();
}

void HAL::analogStart()
{
#if ANALOG_INPUTS > 0
    ADMUX = ANALOG_REF; // refernce voltage
    for(uint8_t i = 0; i < ANALOG_INPUTS; i++)
    {
        osAnalogInputCounter[i] = 0;
        osAnalogInputBuildup[i] = 0;
        osAnalogInputValues[i] = 0;
    }
    ADCSRA = _BV(ADEN)|_BV(ADSC)|ANALOG_PRESCALER;
    //ADCSRA |= _BV(ADSC);                  // start ADC-conversion
    while (ADCSRA & _BV(ADSC) ) {} // wait for conversion
    /* ADCW must be read once, otherwise the next result is wrong. */
    uint dummyADCResult;
    dummyADCResult = ADCW;
    // Enable interrupt driven conversion loop
    uint8_t channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
    if(channel & 8)  // Reading channel 0-7 or 8-15?
        ADCSRB |= _BV(MUX5);
    else
        ADCSRB &= ~_BV(MUX5);
#endif
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
    ADCSRA |= _BV(ADSC); // start conversion without interrupt!
#endif
}

/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

#include <avr/io.h>

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz)
{
    /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
    TWSR = 0;                         /* no prescaler */
    TWBR = ((F_CPU/clockSpeedHz)-16)/2;  /* must be > 10 for stable operation */
}


/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char HAL::i2cStart(unsigned char address)
{
    uint8_t   twst;

    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

    // send device address
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wail until transmission completed and ACK/NACK has been received
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

    return 0;

}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void HAL::i2cStartWait(unsigned char address)
{
    uint8_t   twst;
    while ( 1 )
    {
        // send START condition
        TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

        // wait until transmission completed
        while(!(TWCR & (1<<TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

        // send device address
        TWDR = address;
        TWCR = (1<<TWINT) | (1<<TWEN);

        // wail until transmission completed
        while(!(TWCR & (1<<TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
        {
            /* device busy, send stop condition to terminate write operation */
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

            // wait until stop condition is executed and bus released
            while(TWCR & (1<<TWSTO));

            continue;
        }
        //if( twst != TW_MT_SLA_ACK) return 1;
        break;
    }

}


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void)
{
    /* send stop condition */
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO));
}


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char HAL::i2cWrite( unsigned char data )
{
    uint8_t   twst;
    // send data to the previously addressed device
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));
    // check value of TWI Status Register. Mask prescaler bits
    twst = TW_STATUS & 0xF8;
    if( twst != TW_MT_DATA_ACK) return 1;
    return 0;
}


/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadAck(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadNak(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
}

#if FEATURE_SERVO
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
#define SERVO2500US F_CPU/3200
#define SERVO5000US F_CPU/1600
unsigned int HAL::servoTimings[4] = {0,0,0,0};
unsigned int servoAutoOff[4] = {0,0,0,0};
static uint8_t servoIndex = 0;
void HAL::servoMicroseconds(uint8_t servo,int ms, uint16_t autoOff)
{
    if(ms<500) ms = 0;
    if(ms>2500) ms = 2500;
    servoTimings[servo] = (unsigned int)(((F_CPU/1000000)*(long)ms)>>3);
    servoAutoOff[servo] = (ms) ? (autoOff / 20) : 0;
}
SIGNAL (TIMER3_COMPA_vect)
{
    switch(servoIndex)
    {
    case 0:
        TCNT3 = 0;
        if(HAL::servoTimings[0])
        {
#if SERVO0_PIN>-1
            WRITE(SERVO0_PIN,HIGH);
#endif
            OCR3A = HAL::servoTimings[0];
        }
        else OCR3A = SERVO2500US;
        break;
    case 1:
#if SERVO0_PIN>-1
        WRITE(SERVO0_PIN,LOW);
#endif
        OCR3A = SERVO5000US;
        break;
    case 2:
        TCNT3 = 0;
        if(HAL::servoTimings[1])
        {
#if SERVO1_PIN>-1
            WRITE(SERVO1_PIN,HIGH);
#endif
            OCR3A = HAL::servoTimings[1];
        }
        else OCR3A = SERVO2500US;
        break;
    case 3:
#if SERVO1_PIN>-1
        WRITE(SERVO1_PIN,LOW);
#endif
        OCR3A = SERVO5000US;
        break;
    case 4:
        TCNT3 = 0;
        if(HAL::servoTimings[2])
        {
#if SERVO2_PIN>-1
            WRITE(SERVO2_PIN,HIGH);
#endif
            OCR3A = HAL::servoTimings[2];
        }
        else OCR3A = SERVO2500US;
        break;
    case 5:
#if SERVO2_PIN>-1
        WRITE(SERVO2_PIN,LOW);
#endif
        OCR3A = SERVO5000US;
        break;
    case 6:
        TCNT3 = 0;
        if(HAL::servoTimings[3])
        {
#if SERVO3_PIN>-1
            WRITE(SERVO3_PIN,HIGH);
#endif
            OCR3A = HAL::servoTimings[3];
        }
        else OCR3A = SERVO2500US;
        break;
    case 7:
#if SERVO3_PIN>-1
        WRITE(SERVO3_PIN,LOW);
#endif
        OCR3A = SERVO5000US;
        break;
    }
    if(servoIndex & 1)
    {
        uint8_t nr = servoIndex >> 1;
        if(servoAutoOff[nr])
        {
            servoAutoOff[nr]--;
            if(servoAutoOff[nr] == 0) HAL::servoTimings[nr] = 0;
        }
    }
    servoIndex++;
    if(servoIndex>7)
        servoIndex = 0;
}
#else
#error No servo support for your board, please diable FEATURE_SERVO
#endif
#endif

// ================== Interrupt handling ======================

/** \brief Sets the timer 1 compare value to delay ticks.

This function sets the OCR1A compare counter  to get the next interrupt
at delay ticks measured from the last interrupt. delay must be << 2^24
*/
inline void setTimer(uint32_t delay)
{
    __asm__ __volatile__ (
        "cli \n\t"
        "tst %C[delay] \n\t" //if(delay<65536) {
        "brne else%= \n\t"
        "cpi %B[delay],255 \n\t"
        "breq else%= \n\t" // delay <65280
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
        :[delay]"=&d"(delay) // Output
        :"0"(delay),[ocr]"i" (_SFR_MEM_ADDR(OCR1A)),[time]"i"(_SFR_MEM_ADDR(TCNT1)) // Input
        :"r18" // Clobber
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

volatile uint8_t insideTimer1 = 0;
long stepperWait = 0;
/** \brief Timer interrupt routine to drive the stepper motors.
*/
ISR(TIMER1_COMPA_vect)
{
    if(insideTimer1) return;
    uint8_t doExit;
    __asm__ __volatile__ (
        "ldi %[ex],0 \n\t"
        "lds r23,stepperWait+2 \n\t"
        "tst r23 \n\t" //if(stepperWait<65536) {
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
        "sts stepperWait+1,r22 \n\t"	// ocr1a stays 32768
        "sts stepperWait+2,r23 \n\t"
        "end1%=: ldi %[ex],1 \n\t"
        "end%=: \n\t"
        :[ex]"=&d"(doExit):[ocr]"i" (_SFR_MEM_ADDR(OCR1A)):"r22","r23" );
    if(doExit) return;
    insideTimer1 = 1;
    OCR1A = 61000;
    if(PrintLine::hasLines())
    {
        setTimer(PrintLine::bresenhamStep());
    }
    else if(FEATURE_BABYSTEPPING && Printer::zBabystepsMissing)
    {
        Printer::zBabystep();
        setTimer(Printer::interval);
    }
    else
    {
        if(waitRelax == 0)
        {
#if USE_ADVANCE
            if(Printer::advanceStepsSet)
            {
                Printer::extruderStepsNeeded -= Printer::advanceStepsSet;
#if ENABLE_QUADRATIC_ADVANCE
                Printer::advanceExecuted = 0;
#endif
                Printer::advanceStepsSet = 0;
            }
#endif
#if USE_ADVANCE
            if(!Printer::extruderStepsNeeded) if(DISABLE_E) Extruder::disableCurrentExtruderMotor();
#else
            if(DISABLE_E) Extruder::disableCurrentExtruderMotor();
#endif
        }
        else waitRelax--;
        stepperWait = 0; // Importent becaus of optimization in asm at begin
        OCR1A = 65500; // Wait for next move
    }
    DEBUG_MEMORY;
    insideTimer1 = 0;
}

#if !defined(HEATER_PWM_SPEED)
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED < 0
#define HEATER_PWM_SPEED 0
#endif
#if HEATER_PWM_SPEED > 2
#define HEATER_PWM_SPEED 2
#endif

#if HEATER_PWM_SPEED == 0
#define HEATER_PWM_STEP 1
#define HEATER_PWM_MASK 255
#elif HEATER_PWM_SPEED == 1
#define HEATER_PWM_STEP 2
#define HEATER_PWM_MASK 254
#else
#define HEATER_PWM_STEP 4
#define HEATER_PWM_MASK 252
#endif

#if !defined(COOLER_PWM_SPEED)
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED < 0
#define COOLER_PWM_SPEED 0
#endif
#if COOLER_PWM_SPEED > 2
#define COOLER_PWM_SPEED 2
#endif

#if COOLER_PWM_SPEED == 0
#define COOLER_PWM_STEP 1
#define COOLER_PWM_MASK 255
#elif COOLER_PWM_SPEED == 1
#define COOLER_PWM_STEP 2
#define COOLER_PWM_MASK 254
#else
#define COOLER_PWM_STEP 4
#define COOLER_PWM_MASK 252
#endif

#define pulseDensityModulate( pin, density,error,invert) {uint8_t carry;carry = error + (invert ? 255 - density : density); WRITE(pin, (carry < error)); error = carry;}
/**
This timer is called 3906 timer per second. It is used to update pwm values for heater and some other frequent jobs.
*/
ISR(PWM_TIMER_VECTOR)
{
    static uint8_t pwm_count_cooler = 0;
    static uint8_t pwm_count_heater = 0;
    static uint8_t pwm_pos_set[NUM_EXTRUDER + 3];
#if NUM_EXTRUDER > 0
    static uint8_t pwm_cooler_pos_set[NUM_EXTRUDER];
#endif
    PWM_OCR += 64;
    if(pwm_count_heater == 0 && !PDM_FOR_EXTRUDER)
    {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
        if((pwm_pos_set[0] = (pwm_pos[0] & HEATER_PWM_MASK)) > 0) WRITE(EXT0_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
        if((pwm_pos_set[1] = (pwm_pos[1] & HEATER_PWM_MASK)) > 0) WRITE(EXT1_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
        if((pwm_pos_set[2] = (pwm_pos[2] & HEATER_PWM_MASK)) > 0) WRITE(EXT2_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
        if((pwm_pos_set[3] = (pwm_pos[3] & HEATER_PWM_MASK)) > 0) WRITE(EXT3_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
        if((pwm_pos_set[4] = (pwm_pos[4] & HEATER_PWM_MASK)) > 0) WRITE(EXT4_HEATER_PIN,!HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
        if((pwm_pos_set[5] = (pwm_pos[5] & HEATER_PWM_MASK)) > 0) WRITE(EXT5_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
        if((pwm_pos_set[NUM_EXTRUDER] = (pwm_pos[NUM_EXTRUDER] & HEATER_PWM_MASK)) > 0) WRITE(HEATED_BED_HEATER_PIN, !HEATER_PINS_INVERTED);
#endif
    }
    if(pwm_count_cooler == 0 && !PDM_FOR_COOLER)
    {
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN > -1
        if((pwm_cooler_pos_set[0] = (extruder[0].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT0_EXTRUDER_COOLER_PIN, 1);
#endif
#if !SHARED_COOLER && defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
#if EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
        if((pwm_cooler_pos_set[1] = (extruder[1].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT1_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
#if EXT2_EXTRUDER_COOLER_PIN > -1
        if((pwm_cooler_pos_set[2] = (extruder[2].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT2_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
#if EXT3_EXTRUDER_COOLER_PIN > -1
        if((pwm_cooler_pos_set[3] = (extruder[3].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT3_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
#if EXT4_EXTRUDER_COOLER_PIN > -1
        if((pwm_cooler_pos_set[4] = (extruder[4].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT4_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if !SHARED_COOLER && defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
#if EXT5_EXTRUDER_COOLER_PIN > -1
        if((pwm_cooler_pos_set[5] = (extruder[5].coolerPWM & COOLER_PWM_MASK)) > 0) WRITE(EXT5_EXTRUDER_COOLER_PIN, 1);
#endif
#endif
#if FAN_BOARD_PIN > -1 && SHARED_COOLER_BOARD_EXT == 0
        if((pwm_pos_set[NUM_EXTRUDER + 1] = (pwm_pos[NUM_EXTRUDER + 1] & COOLER_PWM_MASK)) > 0) WRITE(FAN_BOARD_PIN,1);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
        if((pwm_pos_set[NUM_EXTRUDER + 2] = (pwm_pos[NUM_EXTRUDER + 2] & COOLER_PWM_MASK)) > 0) WRITE(FAN_PIN,1);
#endif
    }
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN > -1
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT0_HEATER_PIN, pwm_pos[0], pwm_pos_set[0], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[0] == pwm_count_heater && pwm_pos_set[0] != HEATER_PWM_MASK) WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if EXT0_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT0_EXTRUDER_COOLER_PIN, extruder[0].coolerPWM, pwm_cooler_pos_set[0], false);
#else
    if(pwm_cooler_pos_set[0] == pwm_count_cooler && pwm_cooler_pos_set[0] != COOLER_PWM_MASK) WRITE(EXT0_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT1_HEATER_PIN, pwm_pos[1], pwm_pos_set[1], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[1] == pwm_count_heater && pwm_pos_set[1] != HEATER_PWM_MASK) WRITE(EXT1_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && defined(EXT1_EXTRUDER_COOLER_PIN) && EXT1_EXTRUDER_COOLER_PIN > -1 && EXT1_EXTRUDER_COOLER_PIN != EXT0_EXTRUDER_COOLER_PIN
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT1_EXTRUDER_COOLER_PIN, extruder[1].coolerPWM, pwm_cooler_pos_set[1], false);
#else
    if(pwm_cooler_pos_set[1] == pwm_count_cooler && pwm_cooler_pos_set[1] != COOLER_PWM_MASK) WRITE(EXT1_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT2_HEATER_PIN, pwm_pos[2], pwm_pos_set[2], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[2] == pwm_count_heater && pwm_pos_set[2] != HEATER_PWM_MASK) WRITE(EXT2_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT2_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT2_EXTRUDER_COOLER_PIN, extruder[2].coolerPWM, pwm_cooler_pos_set[2], false);
#else
    if(pwm_cooler_pos_set[2] == pwm_count_cooler && pwm_cooler_pos_set[2] != COOLER_PWM_MASK) WRITE(EXT2_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER > 3
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT3_HEATER_PIN, pwm_pos[3], pwm_pos_set[3], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[3] == pwm_count_heater && pwm_pos_set[3] != HEATER_PWM_MASK) WRITE(EXT3_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT3_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT3_EXTRUDER_COOLER_PIN, extruder[3].coolerPWM, pwm_cooler_pos_set[3], false);
#else
    if(pwm_cooler_pos_set[3] == pwm_count_cooler && pwm_cooler_pos_set[3] != COOLER_PWM_MASK) WRITE(EXT3_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT4_HEATER_PIN, pwm_pos[4], pwm_pos_set[4], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[4] == pwm_count_heater && pwm_pos_set[4] != HEATER_PWM_MASK) WRITE(EXT4_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT4_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT4_EXTRUDER_COOLER_PIN, extruder[4].coolerPWM, pwm_cooler_pos_set[4], false);
#else
    if(pwm_cooler_pos_set[4] == pwm_count_cooler && pwm_cooler_pos_set[4] != COOLER_PWM_MASK) WRITE(EXT4_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER > 5
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(EXT5_HEATER_PIN, pwm_pos[5], pwm_pos_set[5], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[5] == pwm_count_heater && pwm_pos_set[5] != HEATER_PWM_MASK) WRITE(EXT5_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#if !SHARED_COOLER && EXT5_EXTRUDER_COOLER_PIN > -1
#if PDM_FOR_COOLER
    pulseDensityModulate(EXT5_EXTRUDER_COOLER_PIN, extruder[5].coolerPWM, pwm_cooler_pos_set[5], false);
#else
    if(pwm_cooler_pos_set[5] == pwm_count_cooler && pwm_cooler_pos_set[5] != COOLER_PWM_MASK) WRITE(EXT5_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#endif
#if FAN_BOARD_PIN > -1  && SHARED_COOLER_BOARD_EXT == 0
#if PDM_FOR_COOLER
    pulseDensityModulate(FAN_BOARD_PIN, pwm_pos[NUM_EXTRUDER + 1], pwm_pos_set[NUM_EXTRUDER + 1], false);
#else
    if(pwm_pos_set[NUM_EXTRUDER + 1] == pwm_count_cooler && pwm_pos_set[NUM_EXTRUDER + 1] != COOLER_PWM_MASK) WRITE(FAN_BOARD_PIN,0);
#endif
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    if(fanKickstart == 0)
    {
#if PDM_FOR_COOLER
        pulseDensityModulate(FAN_PIN, pwm_pos[NUM_EXTRUDER + 2], pwm_pos_set[NUM_EXTRUDER + 2], false);
#else
        if(pwm_pos_set[NUM_EXTRUDER + 2] == pwm_count_cooler && pwm_pos_set[NUM_EXTRUDER + 2] != COOLER_PWM_MASK) WRITE(FAN_PIN,0);
#endif
    }
#endif
#if HEATED_BED_HEATER_PIN > -1 && HAVE_HEATED_BED
#if PDM_FOR_EXTRUDER
    pulseDensityModulate(HEATED_BED_HEATER_PIN, pwm_pos[NUM_EXTRUDER], pwm_pos_set[NUM_EXTRUDER], HEATER_PINS_INVERTED);
#else
    if(pwm_pos_set[NUM_EXTRUDER] == pwm_count_heater && pwm_pos_set[NUM_EXTRUDER] != HEATER_PWM_MASK) WRITE(HEATED_BED_HEATER_PIN,HEATER_PINS_INVERTED);
#endif
#endif
    HAL::allowInterrupts();
    counterPeriodical++; // Appxoimate a 100ms timer
    if(counterPeriodical >= (int)(F_CPU/40960))
    {
        counterPeriodical = 0;
        executePeriodical = 1;
        if(fanKickstart) fanKickstart--;
    }
// read analog values
#if ANALOG_INPUTS > 0
    if((ADCSRA & _BV(ADSC)) == 0)   // Conversion finished?
    {
        osAnalogInputBuildup[osAnalogInputPos] += ADCW;
        if(++osAnalogInputCounter[osAnalogInputPos] >= _BV(ANALOG_INPUT_SAMPLE))
        {
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE < 12
            osAnalogInputValues[osAnalogInputPos] =
                osAnalogInputBuildup[osAnalogInputPos] << (12 - ANALOG_INPUT_BITS - ANALOG_INPUT_SAMPLE);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE > 12
            osAnalogInputValues[osAnalogInputPos] =
                osAnalogInputBuildup[osAnalogInputPos] >> (ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE - 12);
#endif
#if ANALOG_INPUT_BITS + ANALOG_INPUT_SAMPLE == 12
            osAnalogInputValues[osAnalogInputPos] = osAnalogInputBuildup[osAnalogInputPos];
#endif
            osAnalogInputBuildup[osAnalogInputPos] = 0;
            osAnalogInputCounter[osAnalogInputPos] = 0;
            // Start next conversion
            if(++osAnalogInputPos >= ANALOG_INPUTS) osAnalogInputPos = 0;
            uint8_t channel = pgm_read_byte(&osAnalogInputChannels[osAnalogInputPos]);
#if defined(ADCSRB) && defined(MUX5)
            if(channel & 8)  // Reading channel 0-7 or 8-15?
                ADCSRB |= _BV(MUX5);
            else
                ADCSRB &= ~_BV(MUX5);
#endif
            ADMUX = (ADMUX & ~(0x1F)) | (channel & 7);
        }
        ADCSRA |= _BV(ADSC);  // start next conversion
    }
#endif

    UI_FAST; // Short timed user interface action
    pwm_count_cooler += COOLER_PWM_STEP;
    pwm_count_heater += HEATER_PWM_STEP;
#if FEATURE_WATCHDOG
  if(HAL::wdPinged) {
     wdt_reset();
     HAL::wdPinged = false;
  }
#endif
}
#if USE_ADVANCE

static int8_t extruderLastDirection = 0;
#ifndef ADVANCE_DIR_FILTER_STEPS
#define ADVANCE_DIR_FILTER_STEPS 2
#endif

void HAL::resetExtruderDirection()
{
    extruderLastDirection = 0;
}
/** \brief Timer routine for extruder stepper.

Several methods need to move the extruder. To get a optima result,
all methods update the printer_state.extruderStepsNeeded with the
number of additional steps needed. During this interrupt, one step
is executed. This will keep the extruder moving, until the total
wanted movement is achieved. This will be done with the maximum
allowable speed for the extruder.
*/
ISR(EXTRUDER_TIMER_VECTOR)
{
    uint8_t timer = EXTRUDER_OCR;
    if(!Printer::isAdvanceActivated()) return; // currently no need
    if(Printer::extruderStepsNeeded > 0 && extruderLastDirection != 1)
    {
        if(Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS)
        {
            Extruder::setDirection(true);
            extruderLastDirection = 1;
            timer += 40; // Add some more wait time to prevent blocking
        }
    }
    else if(Printer::extruderStepsNeeded < 0 && extruderLastDirection != -1)
    {
        if(-Printer::extruderStepsNeeded >= ADVANCE_DIR_FILTER_STEPS)
        {
            Extruder::setDirection(false);
            extruderLastDirection = -1;
            timer += 40; // Add some more wait time to prevent blocking
        }
    }
    else if(Printer::extruderStepsNeeded != 0)
    {
        Extruder::step();
        Printer::extruderStepsNeeded -= extruderLastDirection;
        Printer::insertStepperHighDelay();
        Extruder::unstep();
    }
    EXTRUDER_OCR = timer + Printer::maxExtruderSpeed;
}
#endif

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

ring_buffer rx_buffer = { { 0 }, 0, 0};
ring_buffer_tx tx_buffer = { { 0 }, 0, 0};

inline void rf_store_char(unsigned char c, ring_buffer *buffer)
{
    uint8_t i = (buffer->head + 1) & SERIAL_BUFFER_MASK;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != buffer->tail)
    {
        buffer->buffer[buffer->head] = c;
        buffer->head = i;
    }
}
#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
void rfSerialEvent() __attribute__((weak));
void rfSerialEvent() {}
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
    uint8_t c  =  UDR0;
#elif defined(UDR)
    uint8_t c  =  UDR;
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
    if (tx_buffer.head == tx_buffer.tail)
    {
        // Buffer empty, so disable interrupts
#if defined(UCSR0B)
        bit_clear(UCSR0B, UDRIE0);
#else
        bit_clear(UCSRB, UDRIE);
#endif
    }
    else
    {
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
#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega1281__) || defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__))
#error BlueTooth option cannot be used with your mainboard
#endif
#if BLUETOOTH_SERIAL > 1 && !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
#error BlueTooth serial 2 or 3 can be used only with boards based on ATMega2560 or ATMega1280
#endif
#if (BLUETOOTH_SERIAL == 1)
#if defined(USART1_RX_vect)
#define SIG_USARTx_RECV   USART1_RX_vect
#define USARTx_UDRE_vect  USART1_UDRE_vect
#else
#define SIG_USARTx_RECV   SIG_USART1_RECV
#define USARTx_UDRE_vect  SIG_USART1_DATA
#endif
#define UDRx              UDR1
#define UCSRxA            UCSR1A
#define UCSRxB            UCSR1B
#define UBRRxH            UBRR1H
#define UBRRxL            UBRR1L
#define U2Xx              U2X1
#define UARTxENABLE       ((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1))
#define UDRIEx            UDRIE1
#define RXxPIN            19
#elif (BLUETOOTH_SERIAL == 2)
#if defined(USART2_RX_vect)
#define SIG_USARTx_RECV   USART2_RX_vect
#define USARTx_UDRE_vect  USART2_UDRE_vect
#else
#define SIG_USARTx_RECV SIG_USART2_RECV
#define USARTx_UDRE_vect  SIG_USART2_DATA
#endif
#define UDRx              UDR2
#define UCSRxA            UCSR2A
#define UCSRxB            UCSR2B
#define UBRRxH            UBRR2H
#define UBRRxL            UBRR2L
#define U2Xx              U2X2
#define UARTxENABLE       ((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2))
#define UDRIEx            UDRIE2
#define RXxPIN            17
#elif (BLUETOOTH_SERIAL == 3)
#if defined(USART3_RX_vect)
#define SIG_USARTx_RECV   USART3_RX_vect
#define USARTx_UDRE_vect  USART3_UDRE_vect
#else
#define SIG_USARTx_RECV SIG_USART3_RECV
#define USARTx_UDRE_vect  SIG_USART3_DATA
#endif
#define UDRx              UDR3
#define UCSRxA            UCSR3A
#define UCSRxB            UCSR3B
#define UBRRxH            UBRR3H
#define UBRRxL            UBRR3L
#define U2Xx              U2X3
#define UARTxENABLE       ((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3))
#define UDRIEx            UDRIE3
#define RXxPIN            15
#else
#error Wrong serial port number for BlueTooth
#endif

SIGNAL(SIG_USARTx_RECV)
{
    uint8_t c  =  UDRx;
    rf_store_char(c, &rx_buffer);
}

volatile uint8_t txx_buffer_tail = 0;

ISR(USARTx_UDRE_vect)
{
    if (tx_buffer.head == txx_buffer_tail)
    {
        // Buffer empty, so disable interrupts
        bit_clear(UCSRxB, UDRIEx);
    }
    else
    {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = tx_buffer.buffer[txx_buffer_tail];
        txx_buffer_tail = (txx_buffer_tail + 1) & SERIAL_TX_BUFFER_MASK;
        UDRx = c;
    }
}
#endif

// Constructors ////////////////////////////////////////////////////////////////

RFHardwareSerial::RFHardwareSerial(ring_buffer *rx_buffer, ring_buffer_tx *tx_buffer,
                                   volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                                   volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                                   volatile uint8_t *udr,
                                   uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x)
{
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

void RFHardwareSerial::begin(unsigned long baud)
{
    uint16_t baud_setting;
    bool use_u2x = true;

#if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600)
    {
        use_u2x = false;
    }
#endif

try_again:

    if (use_u2x)
    {
        *_ucsra = 1 << _u2x;
        baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else
    {
        *_ucsra = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    if ((baud_setting > 4095) && use_u2x)
    {
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
    WRITE(RXxPIN,1);            // Pullup on RXDx
    UCSRxA  = (1<<U2Xx);
    UBRRxH = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD -1) / 2) >> 8);
    UBRRxL = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD -1) / 2) & 0xFF);
    UCSRxB |= UARTxENABLE;
#endif
}

void RFHardwareSerial::end()
{
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

int RFHardwareSerial::available(void)
{
    return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & SERIAL_BUFFER_MASK;
}
int RFHardwareSerial::outputUnused(void)
{
    return SERIAL_TX_BUFFER_SIZE - (unsigned int)((SERIAL_TX_BUFFER_SIZE + _tx_buffer->head - _tx_buffer->tail) & SERIAL_TX_BUFFER_MASK);
}

int RFHardwareSerial::peek(void)
{
    if (_rx_buffer->head == _rx_buffer->tail)
    {
        return -1;
    }
    return _rx_buffer->buffer[_rx_buffer->tail];
}

int RFHardwareSerial::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail)
    {
        return -1;
    }
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (_rx_buffer->tail + 1) & SERIAL_BUFFER_MASK;
    return c;
}

void RFHardwareSerial::flush()
{
    while (_tx_buffer->head != _tx_buffer->tail)
        ;
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    while (_tx_buffer->head != txx_buffer_tail) ;
#endif
}
#ifdef COMPAT_PRE1
void
#else
size_t
#endif
RFHardwareSerial::write(uint8_t c)
{
    uint8_t i = (_tx_buffer->head + 1) & SERIAL_TX_BUFFER_MASK;

    // If the output buffer is full, there's nothing for it other than to
    // wait for the interrupt handler to empty it a bit
    while (i == _tx_buffer->tail) {}
#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
    while (i == txx_buffer_tail) {}
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
#error no serial port defined  (port 0)
#endif

#endif

