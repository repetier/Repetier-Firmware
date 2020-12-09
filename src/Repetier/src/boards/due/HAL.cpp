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

    Initial port of hardware abstraction layer to Arduino Due: John Silvia
*/

#include "Repetier.h"
#ifdef DUE_BOARD
#include <malloc.h>

#define DEBUG_TIMING 0
#define DEBUG_ISR_STEPPER_PIN 37
#define DEBUG_ISR_MOTION_PIN 35
#define DEBUG_ISR_TEMP_PIN 33

//extern "C" void __cxa_pure_virtual() { }
extern "C" char* sbrk(int i);

// New adc handling
bool analogEnabled[MAX_ANALOG_INPUTS] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false };
// end adc handling

// #define NUM_ADC_SAMPLES 2 + (1 << ANALOG_INPUT_SAMPLE)
/*
#if ANALOG_INPUTS > 0
int32_t osAnalogInputBuildup[ANALOG_INPUTS];
int32_t osAnalogSamples[ANALOG_INPUTS][ANALOG_INPUT_MEDIAN];
int32_t osAnalogSamplesSum[ANALOG_INPUTS];
static int32_t adcSamplesMin[ANALOG_INPUTS];
static int32_t adcSamplesMax[ANALOG_INPUTS];
static int adcCounter = 0, adcSamplePos = 0;
#endif
*/

static uint32_t adcEnable = 0;

char HAL::virtualEeprom[EEPROM_BYTES] = { 0, 0, 0, 0, 0, 0, 0 };
bool HAL::wdPinged = true;
uint8_t HAL::i2cError = 0;
BootReason HAL::startReason = BootReason::UNKNOWN;

volatile uint8_t HAL::insideTimer1 = 0;
#ifndef DUE_SOFTWARE_SPI
int spiDueDividors[] = { 10, 21, 42, 84, 168, 255, 255 };
#endif

HAL::HAL() {
    //ctor
}

HAL::~HAL() {
    //dtor
}

// Set up all timer interrupts
void HAL::setupTimer() {
#if DEBUG_TIMING
    SET_OUTPUT(DEBUG_ISR_STEPPER_PIN);
    SET_OUTPUT(DEBUG_ISR_MOTION_PIN);
    SET_OUTPUT(DEBUG_ISR_TEMP_PIN);
#endif

    uint32_t tc_count, tc_clock;
    pmc_set_writeprotect(false);
    // set 3 bits for interrupt group priority, 1 bits for sub-priority
    //NVIC_SetPriorityGrouping(4);

    // Timer for extruder control
#if (DISABLED(MOTION2_USE_REALTIME_TIMER) || PREPARE_FREQUENCY > (PWM_CLOCK_FREQ / 2))
    pmc_enable_periph_clk(MOTION2_TIMER_IRQ); // enable power to timer
    //NVIC_SetPriority((IRQn_Type)EXTRUDER_TIMER_IRQ, NVIC_EncodePriority(4, 4, 1));
    NVIC_SetPriority((IRQn_Type)MOTION2_TIMER_IRQ, 2);

    // count up to value in RC register using given clock
    TC_Configure(MOTION2_TIMER, MOTION2_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1);

    TC_SetRC(MOTION2_TIMER, MOTION2_TIMER_CHANNEL, (F_CPU_TRUE / 2) / PREPARE_FREQUENCY); // set frequency 43 for 60000Hz
    TC_Start(MOTION2_TIMER, MOTION2_TIMER_CHANNEL);                                       // start timer running

    // enable RC compare interrupt
    MOTION2_TIMER->TC_CHANNEL[MOTION2_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    // clear the "disable RC compare" interrupt
    MOTION2_TIMER->TC_CHANNEL[MOTION2_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;

    // allow interrupts on timer
    NVIC_EnableIRQ((IRQn_Type)MOTION2_TIMER_IRQ);
#else
    RTT_SetPrescaler(RTT, (32768 / PREPARE_FREQUENCY) - 1);
    RTT_EnableIT(RTT, RTT_MR_RTTINCIEN);
    NVIC_SetPriority(RTT_IRQn, 2);
    NVIC_EnableIRQ(RTT_IRQn);
#endif

    // Regular interrupts for heater control etc
    pmc_enable_periph_clk(PWM_TIMER_IRQ);
    //NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, NVIC_EncodePriority(4, 6, 0));
    NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, 6);

    TC_FindMckDivisor(PWM_CLOCK_FREQ, F_CPU_TRUE, &tc_count, &tc_clock, F_CPU_TRUE);
    TC_Configure(PWM_TIMER, PWM_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | tc_clock);

    TC_SetRC(PWM_TIMER, PWM_TIMER_CHANNEL, (F_CPU_TRUE / tc_count) / PWM_CLOCK_FREQ);
    TC_Start(PWM_TIMER, PWM_TIMER_CHANNEL);

    PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)PWM_TIMER_IRQ);

    // Timer for stepper motor control
    pmc_enable_periph_clk(MOTION3_TIMER_IRQ);
    //NVIC_SetPriority((IRQn_Type)MOTION3_TIMER_IRQ, NVIC_EncodePriority(4, 7, 1)); // highest priority - no surprises here wanted
    NVIC_SetPriority((IRQn_Type)MOTION3_TIMER_IRQ, 0); // highest priority - no surprises here wanted
    TC_Configure(MOTION3_TIMER, MOTION3_TIMER_CHANNEL,
                 TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_SetRC(MOTION3_TIMER, MOTION3_TIMER_CHANNEL, (F_CPU_TRUE / 2) / STEPPER_FREQUENCY);
    TC_Start(MOTION3_TIMER, MOTION3_TIMER_CHANNEL);

    MOTION3_TIMER->TC_CHANNEL[MOTION3_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    MOTION3_TIMER->TC_CHANNEL[MOTION3_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)MOTION3_TIMER_IRQ);

    NVIC_SetPriority(PIOA_IRQn, 1);
    NVIC_SetPriority(PIOB_IRQn, 1);
    NVIC_SetPriority(PIOC_IRQn, 1);
    NVIC_SetPriority(PIOD_IRQn, 1);
    // Servo control
#if NUM_SERVOS > 0 || NUM_BEEPER > 0
    pmc_enable_periph_clk(SERVO_TIMER_IRQ);
    //NVIC_SetPriority((IRQn_Type)SERVO_TIMER_IRQ, NVIC_EncodePriority(4, 5, 0));
    NVIC_SetPriority((IRQn_Type)SERVO_TIMER_IRQ, 3);

    TC_Configure(SERVO_TIMER, SERVO_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1);

    TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, (F_CPU_TRUE / SERVO_PRESCALE) / SERVO_CLOCK_FREQ);
    TC_Start(SERVO_TIMER, SERVO_TIMER_CHANNEL);

    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)SERVO_TIMER_IRQ);
#endif
#if NUM_BEEPERS > 0
    for (int i = 0; i < NUM_BEEPERS; i++) {
        if (beepers[i]->getOutputType() == 1) {
            // If we have any SW beepers, enable the beeper IRQ
            pmc_set_writeprotect(false);
            pmc_enable_periph_clk((uint32_t)BEEPER_TIMER_IRQ);
            NVIC_SetPriority((IRQn_Type)BEEPER_TIMER_IRQ, 1);

            TC_Configure(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);

            BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IER = TC_IER_CPCS | TC_IER_CPAS;
            BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS & ~TC_IER_CPAS;
            NVIC_EnableIRQ((IRQn_Type)BEEPER_TIMER_IRQ);
            break;
        }
    }
#endif
}

// Called within checkForPeriodicalActions (main loop, more or less) 
// as fast as possible
void HAL::handlePeriodical() {

}

struct TimerPWMPin {
    TimerPWMPin(int _pin, Pio* _pio, uint32_t _pio_pin, byte _tc_channel, bool _peripheral_A)
        : pin(_pin)
        , pio(_pio)
        , pio_pin(_pio_pin)
        , tc_global_chan(_tc_channel >> 1)
        , tc_local_chan((_tc_channel >> 1) % 3)
        , peripheral_A(_peripheral_A)
        , tio_line_AB(_tc_channel & 1)
        , lastSetDuty(0) {
        switch (_tc_channel / 6) {
        case 1:
            tc_base_address = TC1;
            break;
        case 2:
            tc_base_address = TC2;
            break;
        default:
            tc_base_address = TC0;
            break;
        }
    }
    int pin;
    Pio* pio;
    uint32_t pio_pin;
    byte tc_global_chan;  // 0 .. 8 What's our overall timer channel number?
    byte tc_local_chan;   // 0 .. 2 We're a timer channel inside a timer counter.
    bool peripheral_A;    // Do we need to set the peripheral to A instead of B?
    bool tio_line_AB;     // 0 = A, 1 = B Is this the TIOA or TIOB output pin?
    Tc* tc_base_address;  // TC0 .. TC2 timer counter registers
    ufast8_t lastSetDuty; // Last duty cycle we were set to. (for frequency changes).
};

// Each timer COUNTER has 3 timer channels.
// Each timer CHANNEL has a TIOA* and TIOB* pin where * is the number of the timer channel.
#define NUM_POSSIBLE_TIMER_PINS 18
static TimerPWMPin timer_pins[NUM_POSSIBLE_TIMER_PINS] = {
    { 2, PIOB, PIO_PB25B_TIOA0, TC0_CHA0, false },
    { 61, PIOA, PIO_PA2A_TIOA1, TC0_CHA1, true }, // TC0
    { 92, PIOA, PIO_PA5A_TIOA2, TC0_CHA2, true },

    { 13, PIOB, PIO_PB27B_TIOB0, TC0_CHB0, false },
    { 60, PIOA, PIO_PA3A_TIOB1, TC0_CHB1, true }, // TC0
    { 58, PIOA, PIO_PA6A_TIOB2, TC0_CHB2, true },

    { 108, PIOB, PIO_PB0B_TIOA3, TC1_CHA3, false },
    { 110, PIOB, PIO_PB2B_TIOA4, TC1_CHA4, false }, // TC1
    { 101, PIOB, PIO_PB4B_TIOA5, TC1_CHA5, false },

    { 109, PIOB, PIO_PB1B_TIOB3, TC1_CHB3, false },
    { 111, PIOB, PIO_PB3B_TIOB4, TC1_CHB4, false }, // TC1
    { 102, PIOB, PIO_PB5B_TIOB5, TC1_CHB5, false },

    { 5, PIOC, PIO_PC25B_TIOA6, TC2_CHA6, false },
    { 3, PIOC, PIO_PC28B_TIOA7, TC2_CHA7, false }, // TC2
    { 11, PIOD, PIO_PD7B_TIOA8, TC2_CHA8, false },

    { 4, PIOC, PIO_PC26B_TIOB6, TC2_CHB6, false },
    { 10, PIOC, PIO_PC29B_TIOB7, TC2_CHB7, false }, // TC2
    { 12, PIOD, PIO_PD8B_TIOB8, TC2_CHB8, false }
};

struct TimerPWMChannel {
    byte used_io;
    TimerPWMPin* timer_A;
    TimerPWMPin* timer_B;
};

static TimerPWMChannel timer_channel[9] = {
    { false, nullptr, nullptr },
    { false, nullptr, nullptr }, // TC0
    { false, nullptr, nullptr },

    { false, nullptr, nullptr },
    { false, nullptr, nullptr }, // TC1
    { false, nullptr, nullptr },

    { false, nullptr, nullptr },
    { false, nullptr, nullptr }, // TC2
    { false, nullptr, nullptr }
};
struct PWMPin {
    int pin;
    Pio* pio;
    uint32_t pio_pin;
    int channel;
    bool invert;
};

#define NUM_POSSIBLE_PWM_PINS 30
static PWMPin pwm_pins[NUM_POSSIBLE_PWM_PINS] = {
    { 0, PIOA, PIO_PA8B_PWMH0, PWM_CH0, false }, // Channel 0
    { 20, PIOB, PIO_PB12B_PWMH0, PWM_CH0, false },
    { 35, PIOC, PIO_PC3B_PWMH0, PWM_CH0, false },
    { 73, PIOA, PIO_PA21B_PWML0, PWM_CH0, true },
    { 67, PIOB, PIO_PB16B_PWML0, PWM_CH0, true },
    { 34, PIOC, PIO_PC2B_PWML0, PWM_CH0, true },
    { 42, PIOA, PIO_PA19B_PWMH1, PWM_CH1, false }, // Channel 1
    { 21, PIOB, PIO_PB13B_PWMH1, PWM_CH1, false },
    { 37, PIOC, PIO_PC5B_PWMH1, PWM_CH1, false },
    { 64, PIOA, PIO_PA12B_PWML1, PWM_CH1, true },
    { 62, PIOB, PIO_PB17B_PWML1, PWM_CH1, true },
    { 36, PIOC, PIO_PC4B_PWML1, PWM_CH1, true },
    { 16, PIOA, PIO_PA13B_PWMH2, PWM_CH2, false }, // Channel 2
    { 53, PIOB, PIO_PB14B_PWMH2, PWM_CH2, false },
    { 39, PIOC, PIO_PC7B_PWMH2, PWM_CH2, false },
    { 43, PIOA, PIO_PA20B_PWML2, PWM_CH2, true },
    { 63, PIOB, PIO_PB18B_PWML2, PWM_CH2, true },
    { 38, PIOC, PIO_PC6B_PWML2, PWM_CH2, true },
    { 1, PIOA, PIO_PA9B_PWMH3, PWM_CH3, false }, // Channel 3
    { 66, PIOB, PIO_PB15B_PWMH3, PWM_CH3, false },
    { 41, PIOC, PIO_PC9B_PWMH3, PWM_CH3, false },
    { 69, PIOA, PIO_PA0B_PWML3, PWM_CH3, true },
    { 64, PIOB, PIO_PB19B_PWML3, PWM_CH3, true }, // double value 64
    { 40, PIOC, PIO_PC8B_PWML3, PWM_CH3, true },
    // {??, PIOC, PIO_PC20B_PWMH4, PWM_CH4, false} // Channel 4
    { 9, PIOC, PIO_PC21B_PWML4, PWM_CH4, true },
    { 44, PIOC, PIO_PC19B_PWMH5, PWM_CH5, false }, // Channel 5
    { 8, PIOC, PIO_PC22B_PWML5, PWM_CH5, true },
    { 45, PIOC, PIO_PC18B_PWMH6, PWM_CH6, false }, // Channel 6
    { 7, PIOC, PIO_PC23B_PWML6, PWM_CH6, true },
    { 6, PIOC, PIO_PC24B_PWML7, PWM_CH7, true } // Channel 7
};

struct PWMChannel {
    bool used;
    PWMPin* pwm; // table index
    uint32_t scale;
    ufast8_t lastSetDuty;
};

static PWMChannel pwm_channel[8] = {
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 },
    { false, nullptr, 0 }
};

static void computePWMDivider(uint32_t frequency, uint32_t& div, uint32_t& scale) {
    uint32_t factor = 1;
    div = 0;
    if (frequency < 1) {
        frequency = 1;
    }
    do {
        scale = VARIANT_MCK / (frequency * factor);
        if (scale <= 65535) {
            return;
        }
        div++;
    } while ((factor <<= 1) <= 1024);

    if (scale > 65535) {
        scale = 65535;
    }
    if (div > 10) {
        div = 10;
    }
}

// Try to initialize pinNumber as hardware PWM. Returns internal
// id if it succeeds or -1 if it fails. Typical reasons to fail
// are no pwm support for that pin or an other pin uses same PWM
// channel.
int HAL::initHardwarePWM(int pinNumber, uint32_t frequency) {
    // Search pin mapping
    int foundPin = -1;
    for (int i = 0; i < NUM_POSSIBLE_PWM_PINS; i++) {
        if (pwm_pins[i].pin == pinNumber) {
            if (pwm_channel[pwm_pins[i].channel].used == false) { // ensure not used
                foundPin = i;
            }
            break;
        }
    }
    bool foundTimer = false;
    if (foundPin == -1) {
        for (int i = 0; i < NUM_POSSIBLE_TIMER_PINS; i++) {
            if (timer_pins[i].pin == pinNumber) {

                if (!((timer_channel[timer_pins[i].tc_global_chan].used_io >> timer_pins[i].tio_line_AB) & 1)) {
                    foundPin = i;
                    foundTimer = true;
                }
                break;
            }
        }
    }
    if (foundPin == -1) {
        return -1;
    }

    if (!frequency) {
        frequency = 1;
    }

    if (foundTimer) {
        TimerPWMPin& t = timer_pins[foundPin];
        TimerPWMChannel& c = timer_channel[t.tc_global_chan];
        c.used_io |= (1 << t.tio_line_AB);

        if (!t.tio_line_AB) {
            c.timer_A = &t;
        } else {
            c.timer_B = &t;
        }

        t.pio->PIO_PDR |= t.pio_pin;

        if (!t.peripheral_A) {
            t.pio->PIO_ABSR |= t.pio_pin;
        } else {
            t.pio->PIO_ABSR &= ~t.pio_pin;
        }

        pmc_enable_periph_clk(ID_TC0 + t.tc_global_chan);
        TC_Configure(t.tc_base_address, t.tc_local_chan,
                     TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_EEVT_XC0);

        TC_SetRC(t.tc_base_address, t.tc_local_chan, (F_CPU_TRUE / 2) / frequency);
        TC_Start(t.tc_base_address, t.tc_local_chan);

        // Avoid collisions with the pwm handler
        return (1 << 7 | (t.tc_global_chan << 1)) | t.tio_line_AB;
    }

    PWMPin& p = pwm_pins[foundPin];
    PWMChannel& c = pwm_channel[p.channel];
    c.used = true;
    c.pwm = &p;
    uint32_t div;
    computePWMDivider(frequency, div, c.scale);
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    // configuring the pwm pin
    PIO_Configure(
        p.pio,
        PIO_PERIPH_B, // port E and F would be A
        p.pio_pin,
        PIO_DEFAULT);

    PWMC_ConfigureChannelExt(
        PWM_INTERFACE,
        p.channel,               // channel
        div,                     // clock divider
        0,                       // left aligned
        p.invert ? 0 : (1 << 9), // polarity
        0,                       // interrupt on counter event at end's period
        0,                       // dead-time disabled
        0,                       // non inverted dead-time high output
        0                        // non inverted dead-time low output
    );

    PWMC_SetPeriod(
        PWM_INTERFACE,
        p.channel, // pin_info::channel,
        c.scale);

    PWMC_EnableChannel(PWM_INTERFACE, p.channel);
    setHardwarePWM(p.channel, 0); // init disabled
    return p.channel;
}
// Set pwm output to value. id is id from initHardwarePWM.
void HAL::setHardwarePWM(int id, int value) {
    if (id < 0) { // illegal id
        return;
    }
    if (id < 8) { // PWM channel 0..7
        PWMChannel& c = pwm_channel[id];
        uint32_t duty = (c.scale * value) / 255;
        c.lastSetDuty = value;
        if ((PWM_INTERFACE->PWM_SR & (1 << id)) == 0) { // disabled, set value
            PWM_INTERFACE->PWM_CH_NUM[id].PWM_CDTY = duty;
        } else { // just update
            PWM_INTERFACE->PWM_CH_NUM[id].PWM_CDTYUPD = duty;
        }
        return;
    }

    id &= ~0x80;

    if ((id >> 1) > 8) {
        return;
    }
    TimerPWMChannel& c = timer_channel[(id >> 1)];
    TimerPWMPin& t = *((id & 0x1) ? c.timer_B : c.timer_A);

    t.lastSetDuty = value;
    if (!value) {
        t.tc_base_address->TC_CHANNEL[t.tc_local_chan].TC_CMR &= t.tio_line_AB ? ~TC_CMR_BCPC_SET : ~TC_CMR_ACPC_SET;
    } else {

        t.tc_base_address->TC_CHANNEL[t.tc_local_chan].TC_CMR |= (t.tio_line_AB ? (TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET | TC_CMR_EEVT_XC0) : (TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET));

        uint32_t freq = t.tc_base_address->TC_CHANNEL[t.tc_local_chan].TC_RC;
        if (t.tio_line_AB) {
            TC_SetRB(t.tc_base_address, t.tc_local_chan, ((freq * value) / 255));
        } else {
            TC_SetRA(t.tc_base_address, t.tc_local_chan, ((freq * value) / 255));
        }
    }
}

void HAL::setHardwareFrequency(int id, uint32_t frequency) {
    if (id < 0 || !frequency) {
        return;
    }
    if (id < 8) {
        PWMChannel& c = pwm_channel[id];
        uint32_t divisor = 0;

        computePWMDivider(frequency, divisor, c.scale);

        if (divisor != (PWM_INTERFACE->PWM_CH_NUM[id].PWM_CMR & PWM_CMR_CPRE_Msk)) {
            // Only reconfigure the channel if we've got to redo the prescaler.
            PWMC_ConfigureChannelExt(PWM_INTERFACE, id, divisor, 0, c.pwm->invert ? 0 : (1 << 9), 0, 0, 0, 0);
            PWMC_EnableChannel(PWM_INTERFACE, id);
        }

        PWMC_SetPeriod(PWM_INTERFACE, id, c.scale);
        HAL::setHardwarePWM(id, c.lastSetDuty);
        return;
    }
    id &= ~0x80;
    if ((id >> 1) > 8) {
        return;
    }

    TimerPWMChannel& c = timer_channel[(id >> 1)];
    TimerPWMPin t = *((id & 0x1) ? c.timer_B : c.timer_A);

    TC_Stop(t.tc_base_address, t.tc_local_chan);
    TC_SetRC(t.tc_base_address, t.tc_local_chan, (F_CPU_TRUE / 2) / frequency);
    HAL::setHardwarePWM(id | 0x80, t.lastSetDuty);
    TC_Start(t.tc_base_address, t.tc_local_chan);
}

void HAL::analogEnable(int channel) {
    analogEnabled[channel] = true;
}

//#if ANALOG_INPUTS > 0
// Initialize ADC channels
void HAL::analogStart(void) {

#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || MOTHERBOARD == 502
    PIO_Configure(
        g_APinDescription[58].pPort,
        g_APinDescription[58].ulPinType,
        g_APinDescription[58].ulPin,
        g_APinDescription[58].ulPinConfiguration);
    PIO_Configure(
        g_APinDescription[59].pPort,
        g_APinDescription[59].ulPinType,
        g_APinDescription[59].ulPin,
        g_APinDescription[59].ulPinConfiguration);
#endif // (MOTHERBOARD==500) || (MOTHERBOARD==501) || (MOTHERBOARD==502)

    // ensure we can write to ADC registers
    ADC->ADC_WPMR = 0x41444300u;   //ADC_WPMR_WPKEY(0);
    pmc_enable_periph_clk(ID_ADC); // enable adc clock
    adcEnable = 0;

    for (int i = 0; i < MAX_ANALOG_INPUTS; i++) {
        if (analogEnabled[i]) {
            adcEnable |= (0x1u << i);
        }
    }

    // enable channels
    ADC->ADC_CHER = adcEnable;
    ADC->ADC_CHDR = !adcEnable;

    // Initialize ADC mode register (some of the following params are not used here)
    // HW trigger disabled, use external Trigger, 12 bit resolution
    // core and ref voltage stays on, normal sleep mode, normal not free-run mode
    // startup time 16 clocks, settling time 17 clocks, no changes on channel switch
    // convert channels in numeric order
    // set prescaler rate  MCK/((PRESCALE+1) * 2)
    // set tracking time  (TRACKTIM+1) * clock periods
    // set transfer period  (TRANSFER * 2 + 3)
    ADC->ADC_MR = ADC_MR_TRGEN_DIS | ADC_MR_TRGSEL_ADC_TRIG0 | ADC_MR_LOWRES_BITS_12 | ADC_MR_SLEEP_NORMAL | ADC_MR_FWUP_OFF | ADC_MR_FREERUN_OFF | ADC_MR_STARTUP_SUT64 | ADC_MR_SETTLING_AST17 | ADC_MR_ANACH_NONE | ADC_MR_USEQ_NUM_ORDER | ADC_MR_PRESCAL(AD_PRESCALE_FACTOR) | ADC_MR_TRACKTIM(AD_TRACKING_CYCLES) | ADC_MR_TRANSFER(AD_TRANSFER_CYCLES);

    ADC->ADC_IER = 0; // no ADC interrupts
    ADC->ADC_CGR = 0; // Gain = 1
    ADC->ADC_COR = 0; // Single-ended, no offset

    // start first conversion
    ADC->ADC_CR = ADC_CR_START;
}

//#endif

#if EEPROM_AVAILABLE == EEPROM_FLASH
millis_t eprSyncTime = 0; // in sync
void HAL::syncEEPROM() {  // store to disk if changed
    millis_t time = millis();
    if (eprSyncTime && (time - eprSyncTime > 2000)) { // Buffer writes only every 2 seconds to pool writes
        eprSyncTime = 0;
        FEUpdateChanges();
        Com::printFLN("EEPROM data updated");
    }
}
void HAL::importEEPROM() {
}
#endif
#if EEPROM_AVAILABLE == EEPROM_SDCARD

#if !SDSUPPORT
#error EEPROM using sd card requires SDCARDSUPPORT
#endif

millis_t eprSyncTime = 0ul; // in sync
sd_file_t eepromFile;
void HAL::syncEEPROM() {                                     // store to disk if changed
    if (eprSyncTime && (millis() - eprSyncTime > 15000ul)) { // Buffer writes only every 15 seconds to pool writes
        eprSyncTime = 0ul;
        if (sd.state < SDState::SD_MOUNTED) { // not mounted
            if (eepromFile.isOpen()) {
                eepromFile.close();
            }
            Com::printErrorFLN(PSTR("Could not write eeprom to sd card - no sd card mounted"));
            return;
        }

        eepromFile.rewind();
        if ((eepromFile.write(virtualEeprom, EEPROM_BYTES) != EEPROM_BYTES
             || !eepromFile.sync())) {
            Com::printErrorFLN(PSTR("Could not write eeprom to sd card"));
            sd.printIfCardErrCode();
        }
    }
}

void HAL::importEEPROM() {
    int readBytes = 0;
    if (!eepromFile.open("eeprom.bin", O_RDWR | O_CREAT | O_SYNC)
        || ((readBytes = eepromFile.read(virtualEeprom, EEPROM_BYTES)) != EEPROM_BYTES
            && readBytes)) { // Sometimes we have a 0 byte eeprom.bin
        Com::printFLN(Com::tOpenFailedFile, PSTR("eeprom.bin"));
    }
    EEPROM::readDataFromEEPROM();
    if (eprSyncTime) {
        eprSyncTime = HAL::timeInMilliseconds() | 1UL; // Reset any sync timer
    }
}

#endif

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
    int mcu = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
    switch (mcu) {
    case 0:
        startReason = BootReason::POWER_UP;
        break;
    case 1:
        // this is return from backup mode on SAM
        startReason = BootReason::BROWNOUT;
        break;
    case 2:
        startReason = BootReason::WATCHDOG_RESET;
        break;
    case 3:
        startReason = BootReason::SOFTWARE_RESET;
        break;
    case 4:
        startReason = BootReason::EXTERNAL_PIN;
        break;
    default:
        startReason = BootReason::UNKNOWN;
    }
}

// Return available memory
int HAL::getFreeRam() {
    struct mallinfo memstruct = mallinfo();
    register char* stack_ptr asm("sp");

    // avail mem in heap + (bottom of stack addr - end of heap addr)
    return (memstruct.fordblks + (int)stack_ptr - (int)sbrk(0));
}

// Reset peripherals and cpu
void HAL::resetHardware() {
    RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
}

#ifdef OLD_SPI
#ifndef DUE_SOFTWARE_SPI
// hardware SPI
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
bool spiInitMaded = false;
#endif
void HAL::spiBegin(uint8_t ssPin) {
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
    if (spiInitMaded == false) {
#endif // Configre SPI pins
        PIO_Configure(
            g_APinDescription[SCK_PIN].pPort,
            g_APinDescription[SCK_PIN].ulPinType,
            g_APinDescription[SCK_PIN].ulPin,
            g_APinDescription[SCK_PIN].ulPinConfiguration);
        PIO_Configure(
            g_APinDescription[MOSI_PIN].pPort,
            g_APinDescription[MOSI_PIN].ulPinType,
            g_APinDescription[MOSI_PIN].ulPin,
            g_APinDescription[MOSI_PIN].ulPinConfiguration);
        PIO_Configure(
            g_APinDescription[MISO_PIN].pPort,
            g_APinDescription[MISO_PIN].ulPinType,
            g_APinDescription[MISO_PIN].ulPin,
            g_APinDescription[MISO_PIN].ulPinConfiguration);

        // set master mode, peripheral select, fault detection
        SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS);
        SPI_Enable(SPI0);
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
        SET_OUTPUT(DAC0_SYNC);
#if NUM_TOOLS > 1
        SET_OUTPUT(DAC1_SYNC);
        WRITE(DAC1_SYNC, HIGH);
#endif
        SET_OUTPUT(SPI_EEPROM1_CS);
        SET_OUTPUT(SPI_EEPROM2_CS);
        SET_OUTPUT(SPI_FLASH_CS);
        WRITE(DAC0_SYNC, HIGH);
        WRITE(SPI_EEPROM1_CS, HIGH);
        WRITE(SPI_EEPROM2_CS, HIGH);
        WRITE(SPI_FLASH_CS, HIGH);
        if (ssPin) {
            HAL::digitalWrite(ssPin, 0);
        } else {
            WRITE(SDSS, HIGH);
        }
#endif // MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD==502)
        PIO_Configure(
            g_APinDescription[SPI_PIN].pPort,
            g_APinDescription[SPI_PIN].ulPinType,
            g_APinDescription[SPI_PIN].ulPin,
            g_APinDescription[SPI_PIN].ulPinConfiguration);
        spiInit(1);
#if (MOTHERBOARD == 500) || (MOTHERBOARD == 501) || (MOTHERBOARD == 502)
        spiInitMaded = true;
    }
#endif
}
// spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
// Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
void HAL::spiInit(uint8_t spiClock) {
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
    if (spiInitMaded == false) {
#endif
        if (spiClock > 4)
            spiClock = 1;
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
        // Set SPI mode 1, clock, select not active after transfer, with delay between transfers
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC,
                          SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
        // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
        SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1, SPI_CSR_NCPHA | SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
#endif // MOTHERBOARD==500 || MOTHERBOARD==501 || (MOTHERBOARD==502)

        // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
        SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA | SPI_CSR_CSAAT | SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
        SPI_Enable(SPI0);
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)
        spiInitMaded = true;
    }
#endif
}
// Write single byte to SPI
void HAL::spiSend(byte b) {
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
        ;
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
        ;
    // clear status
    SPI0->SPI_RDR;
    //delayMicroseconds(1);
}
void HAL::spiSend(const uint8_t* buf, size_t n) {
    if (n == 0)
        return;
    for (size_t i = 0; i < n - 1; i++) {
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
        while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
            ;
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
            ;
        SPI0->SPI_RDR;
        //        delayMicroseconds(1);
    }
    spiSend(buf[n - 1]);
}

// Read single byte from SPI
uint8_t HAL::spiReceive() {
    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN) | SPI_TDR_LASTXFER;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
        ;

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
        ;
    // get byte from receive register
    //delayMicroseconds(1);
    return SPI0->SPI_RDR;
}
#if MOTHERBOARD == 500 || MOTHERBOARD == 501 || (MOTHERBOARD == 502)

void HAL::spiSend(uint32_t chan, byte b) {
    uint8_t dummy_read = 0;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
        ;
    // write byte with address and end transmission flag
    SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
        ;
    // clear status
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        dummy_read = SPI0->SPI_RDR;
}

void HAL::spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
    uint8_t dummy_read = 0;
    if (n == 0)
        return;
    for (int i = 0; i < n - 1; i++) {
        while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
            ;
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
            ;
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
            dummy_read = SPI0->SPI_RDR;
    }
    spiSend(chan, buf[n - 1]);
}

uint8_t HAL::spiReceive(uint32_t chan) {
    uint8_t spirec_tmp;
    // wait for transmit register empty
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
        ;
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
        spirec_tmp = SPI0->SPI_RDR;

    // write dummy byte with address and end transmission flag
    SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

    // wait for receive register
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
        ;
    // get byte from receive register
    return SPI0->SPI_RDR;
}
#endif
// Read from SPI into buffer
void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte-- == 0)
        return;

    for (int i = 0; i < nbyte; i++) {
        //while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
        SPI0->SPI_TDR = 0x000000FF | SPI_PCS(SPI_CHAN);
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
            ;
        buf[i] = SPI0->SPI_RDR;
        // delayMicroseconds(1);
    }
    buf[nbyte] = spiReceive();
}

// Write from buffer to SPI

void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    SPI0->SPI_TDR = (uint32_t)token | SPI_PCS(SPI_CHAN);
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
        ;
    //while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
    //SPI0->SPI_RDR;
    for (int i = 0; i < 511; i++) {
        SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(SPI_CHAN);
        while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0)
            ;
        while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0)
            ;
        SPI0->SPI_RDR;
        //        delayMicroseconds(1);
    }
    spiSend(buf[511]);
}
#endif
#endif

/****************************************************************************************
 Setting for I2C Clock speed. needed to change  clock speed for different peripherals
****************************************************************************************/

void HAL::i2cSetClockspeed(uint32_t clockSpeedHz) {
    WIRE_PORT.setClock(clockSpeedHz);
}

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(uint32_t clockSpeedHz) {
    WIRE_PORT.begin(); // create I2C master access
    WIRE_PORT.setClock(clockSpeedHz);
}

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
*************************************************************************/
/* void HAL::i2cStart(uint8_t address) {
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
void HAL::i2cStartAddr(uint8_t address, unsigned int pos, uint8_t readBytes) {
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
unsigned int HAL::servoTimings[4] = { 0, 0, 0, 0 };
unsigned int servoAutoOff[4] = { 0, 0, 0, 0 };
static uint8_t servoIndex = 0;

void HAL::servoMicroseconds(uint8_t servo, int microsec, uint16_t autoOff) {
    servoTimings[servo] = (unsigned int)(((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * microsec);
    servoAutoOff[servo] = (microsec) ? (autoOff / 20) : 0;
}

// ================== Interrupt handling ======================

ServoInterface* analogServoSlots[4] = { nullptr, nullptr, nullptr, nullptr };
// Servo timer Interrupt handler
void SERVO_TIMER_VECTOR() {
    // apparently have to read status register
    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_SR;
#if NUM_SERVOS > 0
    static uint32_t interval = 0;
    fast8_t servoId = servoIndex >> 1;
    ServoInterface* act = analogServoSlots[servoId];
    if (act == nullptr) {
        TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO2500US);
    } else {
        if (servoIndex & 1) { // disable
            act->disable();
            TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL,
                     SERVO5000US - interval);
            if (servoAutoOff[servoId]) {
                servoAutoOff[servoId]--;
                if (servoAutoOff[servoId] == 0) {
                    HAL::servoTimings[servoId] = 0;
                }
            }
        } else { // enable
            InterruptProtectedBlock noInt;
            if (HAL::servoTimings[servoId]) {
                act->enable();
                interval = HAL::servoTimings[servoId];
                TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
            } else {
                interval = SERVO2500US;
                TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, interval);
            }
        }
    }
    servoIndex++;
    if (servoIndex > 7) {
        servoIndex = 0;
    }
#endif
// Add all generated servo interrupt handlers
#undef IO_TARGET
#define IO_TARGET IO_TARGET_SERVO_INTERRUPT
#include "io/redefine.h"
}
#endif

TcChannel* stepperChannel = (MOTION3_TIMER->TC_CHANNEL + MOTION3_TIMER_CHANNEL);
#ifndef STEPPERTIMER_EXIT_TICKS
#define STEPPERTIMER_EXIT_TICKS 105 // at least 2,5us pause between stepper calls
#endif

/** \brief Timer interrupt routine to drive the stepper motors.
*/
void MOTION3_TIMER_VECTOR() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_STEPPER_PIN, 1);
#endif
    // apparently have to read status register
    stepperChannel->TC_SR;
    /*  static bool inside = false; // prevent double call when not finished
    if(inside) {
        return;
    }    
    inside = true;*/
    Motion3::timer();
    // inside = false;
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_STEPPER_PIN, 0);
#endif
}

fast8_t pwmSteps[] = { 1, 2, 4, 8, 16 };
fast8_t pwmMasks[] = { 255, 254, 252, 248, 240 };

/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/
void PWM_TIMER_VECTOR() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 1);
#endif
    //InterruptProtectedBlock noInt;
    // apparently have to read status register
    PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_SR;

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
    // read analog values
    //#if ANALOG_INPUTS > 0
    // conversion finished?
    if ((ADC->ADC_ISR & adcEnable) == adcEnable) {
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ANALOG_INPUT_LOOP
#include "io/redefine.h"
        /*if (executePeriodical) {
            Com::printFLN("bed:",IOAnalogBed0.value);
        }*/
        /*adcCounter++;
        for (int i = 0; i < ANALOG_INPUTS; i++) {
            int32_t cur = ADC->ADC_CDR[osAnalogInputChannels[i]];
            osAnalogInputBuildup[i] += cur;
            adcSamplesMin[i] = RMath::min(adcSamplesMin[i], cur);
            adcSamplesMax[i] = RMath::max(adcSamplesMax[i], cur);
            if (adcCounter >= NUM_ADC_SAMPLES) {   // store new conversion result
                // Strip biggest and smallest value and round correctly
                osAnalogInputBuildup[i] = osAnalogInputBuildup[i] + (1 << (ANALOG_INPUT_SAMPLE - 1)) - (adcSamplesMin[i] + adcSamplesMax[i]);
                adcSamplesMin[i] = 100000;
                adcSamplesMax[i] = 0;
                osAnalogSamplesSum[i] -= osAnalogSamples[i][adcSamplePos];
                osAnalogSamplesSum[i] += (osAnalogSamples[i][adcSamplePos] = osAnalogInputBuildup[i] >> ANALOG_INPUT_SAMPLE);
                if(executePeriodical == 0 || i >= NUM_ANALOG_TEMP_SENSORS) {
                    osAnalogInputValues[i] = osAnalogSamplesSum[i] / ANALOG_INPUT_MEDIAN;
                }
                osAnalogInputBuildup[i] = 0;
            } // adcCounter >= NUM_ADC_SAMPLES
        } // for i
        if (adcCounter >= NUM_ADC_SAMPLES) {
            adcCounter = 0;
            adcSamplePos++;
            if (adcSamplePos >= ANALOG_INPUT_MEDIAN)
                adcSamplePos = 0;
        }*/
        ADC->ADC_CR = ADC_CR_START; // reread values
    }
    // #endif // ANALOG_INPUTS > 0
    pwm_count0++;
    pwm_count1 += 2;
    pwm_count2 += 4;
    pwm_count3 += 8;
    pwm_count4 += 16;
    GUI::handleKeypress();
#if FEATURE_WATCHDOG
    if (HAL::wdPinged) {
        WDT->WDT_CR = 0xA5000001;
        HAL::wdPinged = false;
    }
#endif

#if (ENABLED(MOTION2_USE_REALTIME_TIMER) && PREPARE_FREQUENCY <= (PWM_CLOCK_FREQ / 2))
    // Asynchronously reenable the RTT in here.
    // otherwise we'd need a busy loop to wait the 40us needed, or something.
    if (!(RTT->RTT_SR) && !(RTT->RTT_MR & RTT_MR_RTTINCIEN)) {
        RTT->RTT_MR |= RTT_MR_RTTINCIEN;
    }
#endif
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 0);
#endif
}

#if (DISABLED(MOTION2_USE_REALTIME_TIMER) || PREPARE_FREQUENCY > (PWM_CLOCK_FREQ / 2))
TcChannel* motion2Channel = (MOTION2_TIMER->TC_CHANNEL + MOTION2_TIMER_CHANNEL);

// MOTION2_TIMER IRQ handler
void MOTION2_TIMER_VECTOR() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
    motion2Channel->TC_SR; // faster replacement for above line!
    Motion2::timer();
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 0);
#endif
}
#else
extern "C" void RTT_Handler() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
    // It's possible for the RTT interrupt to actually preempt itself
    // unless you disable it and reenable it once the status is clear.
    // It takes two slow clock cycles (32.768kHz so 40-50us~) to clear. 
    RTT->RTT_SR;
    RTT->RTT_MR &= ~RTT_MR_RTTINCIEN;
    Motion2::timer(); 
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 0);
#endif
}
#endif

#if NUM_BEEPERS > 0
// IRQ handler for tone generator
void BEEPER_TIMER_VECTOR() {
    // always need to feed the beeper loop a bool "beeperIRQPhase"
    // beeper turns ON at at max counter timer hit. Turns OFF at RA compare
    // (our timer's "duty" counter)

    bool beeperIRQPhase = false;
    if ((BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_SR) & TC_SR_CPCS) {
        beeperIRQPhase = true;
    }
#undef IO_TARGET
#define IO_TARGET IO_TARGET_BEEPER_LOOP
#include "io/redefine.h"
    (void)beeperIRQPhase; // avoid gcc unused warning
}
#endif

void HAL::tone(uint32_t frequency) {
#if NUM_BEEPERS > 0
#if NUM_BEEPERS > 1
    ufast8_t curPlaying = 0;
    BeeperSourceBase* playingBeepers[NUM_BEEPERS];
    // Reduce freq to nearest 100hz, otherwise we can get some insane freq multiples (from eg primes).
    // also clamp max freq.
    constexpr ufast8_t reduce = 100;
    constexpr uint32_t maxFreq = 100000;
    uint32_t multiFreq = frequency - (frequency % reduce);
    for (size_t i = 0; i < (NUM_BEEPERS + curPlaying); i++) {
        uint16_t beeperCurFreq = 0;
        if (i >= NUM_BEEPERS) {
            if (multiFreq > maxFreq) {
                multiFreq = maxFreq;
            }
            beeperCurFreq = playingBeepers[i - NUM_BEEPERS]->getCurFreq();
            beeperCurFreq -= (beeperCurFreq % reduce);
            playingBeepers[i - NUM_BEEPERS]->setFreqDiv(curPlaying > 1 ? constrain((multiFreq / beeperCurFreq) - 1, 0, 1000) : 0);
        } else {
            if (beepers[i]->getOutputType() == 1 && beepers[i]->isPlaying()) {
                beeperCurFreq = beepers[i]->getCurFreq();
                beeperCurFreq -= (beeperCurFreq % reduce);
                if (!multiFreq) {
                    multiFreq = beeperCurFreq;
                }
                multiFreq = RMath::LCM(multiFreq, beeperCurFreq);
                playingBeepers[curPlaying++] = beepers[i];
            }
        }
    }
    frequency = multiFreq;
#endif
    if (frequency < 1) {
        return;
    }
    if (!(TC_GetStatus(BEEPER_TIMER, BEEPER_TIMER_CHANNEL) & TC_SR_CLKSTA)) {
        TC_Start(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);
    }
    // 100% volume is 50% duty
    float percent = (static_cast<float>(Printer::toneVolume) * 50.0f) * 0.0001f;
    uint32_t rc = (F_CPU_TRUE / 2) / frequency;
    uint32_t ra = static_cast<uint32_t>(static_cast<float>(rc) * percent);
    TC_SetRC(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc);
    TC_SetRA(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, ra);
    if (TC_ReadCV(BEEPER_TIMER, BEEPER_TIMER_CHANNEL) > rc) {
        BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_CCR = TC_CCR_SWTRG;
    }
#endif
}

void HAL::noTone() {
#if NUM_BEEPERS > 0
#if NUM_BEEPERS > 1
    // If any IO beeper is still playing, we can't stop the timer yet.
    for (size_t i = 0; i < NUM_BEEPERS; i++) {
        if (beepers[i]->getOutputType() == 1 && beepers[i]->isPlaying()) {
            constexpr uint32_t maxFreq = (F_CPU_TRUE / 2) / 100000;
            // if we're nearing/at our freq limit, refresh the divisors
            if (maxFreq == BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_RC) {
                HAL::tone(0);
            }
            return;
        }
    }
#endif
    TC_Stop(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);
#endif
}

void HAL::spiInit() {
    SPI.begin();
}

#ifdef USE_ARDUINO_SPI_LIB
void HAL::spiBegin(uint32_t clock, uint8_t mode, uint8_t msbfirst) {
    SPI.beginTransaction(SPISettings(clock, msbfirst ? MSBFIRST : LSBFIRST, mode));
}
uint8_t HAL::spiTransfer(uint8_t data) {
    return SPI.transfer(data);
}
void HAL::spiEnd() {
    SPI.endTransaction();
}
#else
static bool spiMsbfirst;
static int spiMode = 0;
void HAL::spiBegin(uint32_t clock, uint8_t mode, uint8_t msbfirst) {
    spiMsbfirst = msbfirst;
    uint8_t div;
    if (clock < (F_CPU / 255)) {
        div = 255;
    } else if (clock >= (F_CPU / 2)) {
        div = 2;
    } else {
        div = (F_CPU / (clock + 1)) + 1;
    }
    switch (mode) {
    case 0:
        spiMode = 2;
        break;
    case 1:
        spiMode = 0;
        break;
    case 2:
        spiMode = 3;
        break;
    case 3:
        spiMode = 1;
        break;
    }
    uint32_t config = (spiMode & 3) | SPI_CSR_CSAAT | SPI_CSR_SCBR(div) | SPI_CSR_DLYBCT(1);
    SPI_ConfigureNPCS(SPI_INTERFACE, SPI_INTERFACE_ID, config);
}
uint8_t HAL::spiTransfer(uint8_t data) {
    if (!spiMsbfirst)
        data = __REV(__RBIT(data));
    uint32_t d = data | SPI_PCS(SPI_INTERFACE_ID);
    if (spiMode == SPI_LAST)
        d |= SPI_TDR_LASTXFER;

    // SPI_Write(spi, _channel, _data);
    while ((SPI_INTERFACE->SPI_SR & SPI_SR_TDRE) == 0)
        ;
    SPI_INTERFACE->SPI_TDR = d;

    // return SPI_Read(spi);
    while ((SPI_INTERFACE->SPI_SR & SPI_SR_RDRF) == 0)
        ;
    d = SPI_INTERFACE->SPI_RDR;
    // Reverse bit order
    if (!spiMsbfirst)
        d = __REV(__RBIT(d));
    return d & 0xFF;
}
#endif

void HAL::eprBurnValue(unsigned int pos, int size, union eeval_t newvalue) {
#if EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
    uint8_t eeprom_temp[3];

    /*write enable*/
    eeprom_temp[0] = 6; //WREN
    WRITE(SPI_EEPROM1_CS, LOW);
    spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 1);
    WRITE(SPI_EEPROM1_CS, HIGH);
    delayMilliseconds(1);

    /*write addr*/
    eeprom_temp[0] = 2;                   //WRITE
    eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
    eeprom_temp[2] = (pos & 0xFF);        //addrL
    WRITE(SPI_EEPROM1_CS, LOW);
    spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);

    spiSend(SPI_CHAN_EEPROM1, &(newvalue.b[0]), 1);
    for (int i = 1; i < size; i++) {
        pos++;
        // writes cannot cross page boundary
        if ((pos % EEPROM_PAGE_SIZE) == 0) {
            // burn current page then address next one
            WRITE(SPI_EEPROM1_CS, HIGH);
            delayMilliseconds(EEPROM_PAGE_WRITE_TIME);

            /*write enable*/
            eeprom_temp[0] = 6; //WREN
            WRITE(SPI_EEPROM1_CS, LOW);
            spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 1);
            WRITE(SPI_EEPROM1_CS, HIGH);

            eeprom_temp[0] = 2;                   //WRITE
            eeprom_temp[1] = ((pos >> 8) & 0xFF); //addrH
            eeprom_temp[2] = (pos & 0xFF);        //addrL
            WRITE(SPI_EEPROM1_CS, LOW);
            spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);
        }
        spiSend(SPI_CHAN_EEPROM1, &(newvalue.b[i]), 1);
    }
    WRITE(SPI_EEPROM1_CS, HIGH);
    delayMilliseconds(EEPROM_PAGE_WRITE_TIME); // wait for page write to complete
#elif EEPROM_AVAILABLE == EEPROM_I2C
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
#elif EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH
    if (pos >= EEPROM::reservedEnd) {
        eprSyncTime = 1UL; // enforce fast write to finish before power is lost
    } else {
        eprSyncTime = HAL::timeInMilliseconds() | 1UL;
    }
#endif
}

// Dummy function to overload weak arduino function that always disables
// watchdog. We do not need that as we do this our self.
void watchdogSetup(void) {
}

void HAL::switchToBootMode() {
    Com::printFLN("Switching to bootmode code not supported for this chip.");
}
#endif
