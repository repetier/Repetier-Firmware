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
#ifdef SAMD51_BOARD
#include <malloc.h>

// Adds output signals to pins below to measure interrupt timings
// with a logic analyser. Set to 0 for production!
#define DEBUG_TIMING 1
#define DEBUG_ISR_STEPPER_PIN 37 // 39
#define DEBUG_ISR_MOTION_PIN 51  // 35
#define DEBUG_ISR_TEMP_PIN 49    // 33
#define DEBUG_ISR_ANALOG_PIN 53

//extern "C" void __cxa_pure_virtual() { }
extern "C" char* sbrk(int i);

char HAL::virtualEeprom[EEPROM_BYTES] = { 0, 0, 0, 0, 0, 0, 0 };
bool HAL::wdPinged = true;
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

/*
maximum time for 120MHz is 0.55924 seconds with prescaler 1024!
*/
#define SYNC_TIMER(timer) \
    while (timer->COUNT16.SYNCBUSY.reg != 0) {}

void getPrescaleFreq(uint32_t ticks, uint32_t& prescale, uint32_t& freq) {
    if (ticks < 65536) {
        prescale = TC_CTRLA_PRESCALER_DIV1;
        freq = ticks;
    } else if (ticks < 131072) {
        prescale = TC_CTRLA_PRESCALER_DIV2;
        freq = ticks >> 1;
    } else if (ticks < 262144) {
        prescale = TC_CTRLA_PRESCALER_DIV4;
        freq = ticks >> 2;
    } else if (ticks < 524288) {
        prescale = TC_CTRLA_PRESCALER_DIV8;
        freq = ticks >> 3;
    } else if (ticks < 1048576) {
        prescale = TC_CTRLA_PRESCALER_DIV16;
        freq = ticks >> 4;
    } else if (ticks < 4194304) {
        prescale = TC_CTRLA_PRESCALER_DIV64;
        freq = ticks >> 6;
    } else if (ticks < 16777216) {
        prescale = TC_CTRLA_PRESCALER_DIV256;
        freq = ticks >> 8;
    } else if (ticks < 67108864ul) {
        prescale = TC_CTRLA_PRESCALER_DIV1024;
        freq = ticks >> 10;
    } else { // too slow, set to slowest possible value
        prescale = TC_CTRLA_PRESCALER_DIV1024;
        freq = 65535;
    }
}
// Set up all timer interrupts
void HAL::setupTimer() {
#if DEBUG_TIMING
    SET_OUTPUT(DEBUG_ISR_STEPPER_PIN);
    SET_OUTPUT(DEBUG_ISR_MOTION_PIN);
    SET_OUTPUT(DEBUG_ISR_TEMP_PIN);
    SET_OUTPUT(DEBUG_ISR_ANALOG_PIN);
#endif
    uint32_t prescale, freq;

    GCLK->PCHCTRL[MOTION2_TIMER_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}

    MOTION2_TIMER->COUNT16.CTRLA.bit.ENABLE = 0; // can only change if disabled

    // Use match mode so that the timer counter resets when the count matches the
    // compare register
    MOTION2_TIMER->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    SYNC_TIMER(MOTION2_TIMER);

    // Enable the compare interrupt
    MOTION2_TIMER->COUNT16.INTENSET.reg = 0;
    MOTION2_TIMER->COUNT16.INTENSET.bit.MC0 = 1;
    getPrescaleFreq(F_CPU_TRUE / PREPARE_FREQUENCY, prescale, freq);
    MOTION2_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
    MOTION2_TIMER->COUNT16.CTRLA.reg |= prescale;
    NVIC_SetPriority((IRQn_Type)MOTION2_TIMER_IRQ, 2); // 1 = highest, 3 = lowest priority
    NVIC_EnableIRQ(MOTION2_TIMER_IRQ);                 // Enable IRQ for function call
    MOTION2_TIMER->COUNT16.COUNT.reg = map(MOTION2_TIMER->COUNT16.COUNT.reg, 0,
                                           MOTION2_TIMER->COUNT16.CC[0].reg, 0, freq);
    MOTION2_TIMER->COUNT16.CC[0].reg = freq;
    SYNC_TIMER(MOTION2_TIMER);
    MOTION2_TIMER->COUNT16.CTRLA.bit.ENABLE = 1; // enable timer
    SYNC_TIMER(MOTION2_TIMER);

    // Regular interrupts for heater control etc

    GCLK->PCHCTRL[PWM_TIMER_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}

    PWM_TIMER->COUNT16.CTRLA.bit.ENABLE = 0; // can only change if disabled

    // Use match mode so that the timer counter resets when the count matches the
    // compare register
    PWM_TIMER->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    SYNC_TIMER(PWM_TIMER);
    PWM_TIMER->COUNT16.INTENSET.reg = 0;
    PWM_TIMER->COUNT16.INTENSET.bit.MC0 = 1;
    getPrescaleFreq(F_CPU_TRUE / PWM_CLOCK_FREQ, prescale, freq);
    PWM_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
    PWM_TIMER->COUNT16.CTRLA.reg |= prescale;
    NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, 3);
    NVIC_EnableIRQ(PWM_TIMER_IRQ); // Enable IRQ for function call
    PWM_TIMER->COUNT16.COUNT.reg = map(PWM_TIMER->COUNT16.COUNT.reg, 0,
                                       PWM_TIMER->COUNT16.CC[0].reg, 0, freq);
    PWM_TIMER->COUNT16.CC[0].reg = freq;
    SYNC_TIMER(PWM_TIMER);
    PWM_TIMER->COUNT16.CTRLA.bit.ENABLE = 1; // enable timer
    SYNC_TIMER(PWM_TIMER);

    // Timer for stepper motor control

    GCLK->PCHCTRL[MOTION3_TIMER_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}

    MOTION3_TIMER->COUNT16.CTRLA.bit.ENABLE = 0; // can only change if disabled

    // Use match mode so that the timer counter resets when the count matches the
    // compare register
    MOTION3_TIMER->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    SYNC_TIMER(MOTION3_TIMER);
    MOTION3_TIMER->COUNT16.INTENSET.reg = 0;
    MOTION3_TIMER->COUNT16.INTENSET.bit.MC0 = 1;
    getPrescaleFreq(F_CPU_TRUE / STEPPER_FREQUENCY, prescale, freq);
    MOTION3_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
    MOTION3_TIMER->COUNT16.CTRLA.reg |= prescale;
    NVIC_SetPriority((IRQn_Type)MOTION3_TIMER_IRQ, 0);
    NVIC_EnableIRQ(MOTION3_TIMER_IRQ); // Enable IRQ for function call
    MOTION3_TIMER->COUNT16.COUNT.reg = map(MOTION3_TIMER->COUNT16.COUNT.reg, 0,
                                           MOTION3_TIMER->COUNT16.CC[0].reg, 0, freq);
    MOTION3_TIMER->COUNT16.CC[0].reg = freq;
    SYNC_TIMER(MOTION3_TIMER);
    MOTION3_TIMER->COUNT16.CTRLA.bit.ENABLE = 1; // enable timer
    SYNC_TIMER(MOTION3_TIMER);

    // Servo control
#if NUM_SERVOS > 0

    GCLK->PCHCTRL[SERVO_TIMER_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}

    SERVO_TIMER->COUNT16.CTRLA.bit.ENABLE = 0; // can only change if disabled

    // Use match mode so that the timer counter resets when the count matches the
    // compare register
    SERVO_TIMER->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    SYNC_TIMER(SERVO_TIMER);
    SERVO_TIMER->COUNT16.INTENSET.reg = 0;
    SERVO_TIMER->COUNT16.INTENSET.bit.MC0 = 1;
    SERVO_TIMER->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
    SERVO_TIMER->COUNT16.CTRLA.reg |= SERVO_PRESCALE_DIV;
    NVIC_SetPriority((IRQn_Type)SERVO_TIMER_IRQ, 1);
    NVIC_EnableIRQ(SERVO_TIMER_IRQ); // Enable IRQ for function call
    SERVO_TIMER->COUNT16.COUNT.reg = map(SERVO_TIMER->COUNT16.COUNT.reg, 0,
                                         SERVO_TIMER->COUNT16.CC[0].reg, 0, SERVO5000US);
    SERVO_TIMER->COUNT16.CC[0].reg = SERVO5000US;
    SYNC_TIMER(SERVO_TIMER);
    SERVO_TIMER->COUNT16.CTRLA.bit.ENABLE = 1; // enable timer
    SYNC_TIMER(SERVO_TIMER);

#endif
}

struct PWMChannel {
    bool used;
    const PinDescription* pwm; // timer
    uint32_t scale;
    int bits;
    int32_t divisor;
};

static PWMChannel pwm_channel[13] = {
    // 5 Tcc channel
    { false, nullptr, 0, TCC0_SIZE, -1 }, // Pins 2, 3, 4, 5, 24, 25
    { false, nullptr, 0, TCC1_SIZE, -1 }, // Pins 6, 7, 8, 62, 63
    { false, nullptr, 0, TCC2_SIZE, -1 }, // Pins 23,28
    { false, nullptr, 0, TCC3_SIZE, -1 }, // Pins 18,19
    { false, nullptr, 0, TCC4_SIZE, -1 }, // Pins 38,39
    // 8 Tc channels, 0-4 pre used by timers!
    // 0 MOTION3
    { true, nullptr, 0, 8, -1 }, // Pin 59
    // 1 PWM_TIMER
    { true, nullptr, 0, 8, -1 },  // Pins 60, 61
    { false, nullptr, 0, 8, -1 }, // Pins 26, 27
    // 3 Tone
    { true, nullptr, 0, 8, -1 }, // Pins 34, 35
    // 4 servo timer can be used if no servos present
    { NUM_SERVOS > 0, nullptr, 0, 8, -1 }, // Pins 30, 31
    // 5 MOTION2
    { true, nullptr, 0, 8, -1 },  // No Pins
    { false, nullptr, 0, 8, -1 }, // Pins 9, 14, 69
    { false, nullptr, 0, 8, -1 }  // Pins 10, 12, 13, 87
};

static void computePWMDivider(uint32_t frequency, uint32_t& div, uint32_t& scale, int bits) {
    uint32_t factor = 1;
    div = 0;
    uint32_t max_scale = 256;
    if (bits == 16)
        max_scale = 65536;
    else if (bits == 24)
        max_scale = 16777216;
    do {
        scale = VARIANT_MCK / (frequency * factor) - 1;
        if (factor != 32 && factor != 128 && factor != 512) {
            if (scale < max_scale) {
                return;
            }
            div++;
        }
        factor <<= 1;
    } while (factor <= 1024);
}
static int pinPeripheral(uint32_t ulPin, EPioType ulPeripheral) {
    // Handle the case the pin isn't usable as PIO
    if (g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN) {
        return -1;
    }

    switch (ulPeripheral) {
    case PIO_DIGITAL:
    case PIO_INPUT:
    case PIO_INPUT_PULLUP:
    case PIO_OUTPUT:
        // Disable peripheral muxing, done in pinMode
        //			PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.PMUXEN = 0 ;

        // Configure pin mode, if requested
        if (ulPeripheral == PIO_INPUT) {
            pinMode(ulPin, INPUT);
        } else {
            if (ulPeripheral == PIO_INPUT_PULLUP) {
                pinMode(ulPin, INPUT_PULLUP);
            } else {
                if (ulPeripheral == PIO_OUTPUT) {
                    pinMode(ulPin, OUTPUT);
                } else {
                    // PIO_DIGITAL, do we have to do something as all cases are covered?
                }
            }
        }
        break;

    case PIO_ANALOG:
    case PIO_SERCOM:
    case PIO_SERCOM_ALT:
    case PIO_TIMER:
    case PIO_TIMER_ALT:
    case PIO_EXTINT:
#if defined(__SAMD51__)
    case PIO_TCC_PDEC:
    case PIO_COM:
    case PIO_SDHC:
    case PIO_I2S:
    case PIO_PCC:
    case PIO_GMAC:
    case PIO_AC_CLK:
    case PIO_CCL:
#else
    case PIO_COM:
    case PIO_AC_CLK:
#endif
#if 0
      // Is the pio pin in the lower 16 ones?
      // The WRCONFIG register allows update of only 16 pin max out of 32
      if ( g_APinDescription[ulPin].ulPin < 16 )
      {
        PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin ) ;
      }
      else
      {
        PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
                                                                    PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin - 16 ) ;
      }
#else
        if (g_APinDescription[ulPin].ulPin & 1) // is pin odd?
        {
            uint32_t temp;

            // Get whole current setup for both odd and even pins and remove odd one
            temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE(0xF);
            // Set new muxing
            PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXO(ulPeripheral);
            // Enable port mux
            PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
        } else // even pin
        {
            uint32_t temp;

            temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO(0xF);
            PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXE(ulPeripheral);
            PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR; // Enable port mux
        }
#endif
        break;

    case PIO_NOT_A_PIN:
        return -1l;
        break;
    }

    return 0l;
}

// Try to initialize pinNumber as hardware PWM. Returns internal
// id if it succeeds or -1 if it fails. Typical reasons to fail
// are no pwm support for that pin or an other pin uses same PWM
// channel.
int HAL::initHardwarePWM(int pinNumber, uint32_t frequency) {
    if (pinNumber < 0) {
        return 255;
    }
    // Search pin mapping
    const PinDescription* pd = &g_APinDescription[pinNumber];
    uint32_t tcNum = GetTCNumber(pd->ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pd->ulPWMChannel);
    if (pd->ulPWMChannel == NOT_ON_PWM || pwm_channel[tcNum].used) { // pwm not possible
        return 255;
    }
    uint32_t attr = pd->ulPinAttribute;
    if (attr & PIN_ATTR_PWM_E) {
        pinPeripheral(pinNumber, PIO_TIMER);
    } else if (attr & PIN_ATTR_PWM_F) {
        pinPeripheral(pinNumber, PIO_TIMER_ALT);
    } else if (attr & PIN_ATTR_PWM_G) {
        pinPeripheral(pinNumber, PIO_TCC_PDEC);
    }
    PWMChannel& c = pwm_channel[tcNum];
    if (c.pwm != nullptr) { // First come sets the frequency, next just reuses it!
        setHardwarePWM(tcNum, 0);
        return tcNum;
    }
    c.used = true;
    c.pwm = pd;
    uint32_t div;
    computePWMDivider(frequency, div, c.scale, c.bits);
    uint32_t duty = 0;
    GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 0

    // Set PORT
    if (tcNum >= TCC_INST_NUM) {
        c.scale = 255;
        c.divisor = div;
        // -- Configure TC
        Tc* TCx = (Tc*)GetTC(pd->ulPWMChannel);

        //reset
        TCx->COUNT8.CTRLA.bit.SWRST = 1;
        while (TCx->COUNT8.SYNCBUSY.bit.SWRST) {}

        // Disable TCx
        TCx->COUNT8.CTRLA.bit.ENABLE = 0;
        while (TCx->COUNT8.SYNCBUSY.bit.ENABLE) {}
        // Set Timer counter Mode to 8 bits, normal PWM, prescaler dynamic
        TCx->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | (div << TC_CTRLA_PRESCALER_Pos);
        TCx->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;

        while (TCx->COUNT8.SYNCBUSY.bit.CC0) {}
        // Set the initial value
        TCx->COUNT8.CC[tcChannel].reg = (uint8_t)duty;
        while (TCx->COUNT8.SYNCBUSY.bit.CC0) {}
        // Set PER to maximum counter value (resolution : 0xFF)
        TCx->COUNT8.PER.reg = (uint8_t)0xff;
        while (TCx->COUNT8.SYNCBUSY.bit.PER) {}
        // Enable TCx
        TCx->COUNT8.CTRLA.bit.ENABLE = 1;
        while (TCx->COUNT8.SYNCBUSY.bit.ENABLE) {}
    } else {
        c.divisor = div;
        // -- Configure TCC
        Tcc* TCCx = (Tcc*)GetTC(pd->ulPWMChannel);

        TCCx->CTRLA.bit.SWRST = 1;
        while (TCCx->SYNCBUSY.bit.SWRST) {}

        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        while (TCCx->SYNCBUSY.bit.ENABLE) {}
        // Set prescaler to 1/256
        TCCx->CTRLA.reg = (div << TCC_CTRLA_PRESCALER_Pos) | TCC_CTRLA_PRESCSYNC_GCLK;
        // TCCx->CTRLA.bit.PRESCALER = div;
        // Set TCx as normal PWM
        TCCx->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
        while (TCCx->SYNCBUSY.bit.WAVE) {}

        while (TCCx->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3 | TCC_SYNCBUSY_CC4)) {}
        // Set the initial value
        TCCx->CC[tcChannel].reg = (uint32_t)duty;
        while (TCCx->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3 | TCC_SYNCBUSY_CC4)) {}
        // Set PER to maximum counter value (resolution : 0xFF)
        TCCx->PER.bit.PER = c.scale;
        while (TCCx->SYNCBUSY.bit.PER) {}
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        while (TCCx->SYNCBUSY.bit.ENABLE) {}
    }
    return tcNum;
}
// Set pwm output to value. id is id from initHardwarePWM.
void HAL::setHardwarePWM(int id, int value) {
    if (id < 0 || id >= 13) { // illegal id
        return;
    }
    PWMChannel& c = pwm_channel[id];
    uint32_t duty = (c.scale * value) / 255;
    uint8_t tcChannel = GetTCChannelNumber(c.pwm->ulPWMChannel);
    if (id >= TCC_INST_NUM) {
        Tc* TCx = (Tc*)GetTC(c.pwm->ulPWMChannel);
        TCx->COUNT16.CC[tcChannel].reg = (uint16_t)duty;
        // while (TCx->COUNT8.SYNCBUSY.bit.CC0 || TCx->COUNT8.SYNCBUSY.bit.CC1) {}
    } else {
        Tcc* TCCx = (Tcc*)GetTC(c.pwm->ulPWMChannel);
        // while (TCCx->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3 | TCC_SYNCBUSY_CC4)) {}
        TCCx->CCBUF[tcChannel].reg = (uint32_t)duty;
        while (TCCx->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3 | TCC_SYNCBUSY_CC4)) {}
        while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB) {}
        TCCx->CTRLBCLR.bit.LUPD = 1;
        while (TCCx->SYNCBUSY.bit.CTRLB) {}
    }
}

// Initialize ADC channels
#define ANALOG_PIN_TO_CHANNEL(p) (p < 62 ? p - 54 : p - 59)
int32_t analogValues[16];
const PinDescription* analogEnabled[16];
void reportAnalog() {
    for (int i = 0; i < 16; i++) {
        if (analogEnabled[i]) {
            Com::printF("Analog ", i);
            Com::printFLN(" = ", analogValues[i]);
        }
    }
}
Adc* analogAdcMap[16] = {
    ADC1,
    ADC1,
    ADC1,
    ADC1,
    ADC1,
    ADC0,
    ADC0,
    ADC0,
    ADC0,
    ADC0,
    ADC0,
    ADC1,
    ADC1,
    ADC1,
    ADC1,
    ADC1
};
static int analogConvertPos = -1;
void HAL::analogStart(void) {
    // Set ADC clock to 48MHz clock
    GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}
    GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}
    ADC0->CTRLA.bit.PRESCALER = 7; // prescaler 256
    ADC1->CTRLA.bit.PRESCALER = 7; // prescaler 256
    ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
    ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
    // Interrupt frequency will be around 8Khz and add a 3% CPU load.
    ADC0->INTENSET.bit.RESRDY = 1; // enable interrupt
    ADC1->INTENSET.bit.RESRDY = 1; // enable interrupt
    analogReference(AR_DEFAULT);   // 3.3V reference voltage
    for (int i = 0; i < 16; i++) {
        analogValues[i] = 0;
        analogEnabled[i] = nullptr;
    }
}
bool analogEven = false;
void analogISRFunction() {
#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 1);
#endif
    Adc* adc;
    analogEven = false;
    if (analogConvertPos >= 0 && !analogEven) { // store result
        adc = analogAdcMap[analogConvertPos];
        analogValues[analogConvertPos] = adc->RESULT.reg;
        adc->CTRLA.bit.ENABLE = 0x00;                      // Disable ADC
        while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE) {} //wait for sync
    }
    analogEven = !analogEven;
    // go to next active
    if (analogEven) {
        do {
            if (++analogConvertPos == 16) {
                // Process data
#undef IO_TARGET
#define IO_TARGET 11
#include "io/redefine.h"
                analogConvertPos = 0;
            }
        } while (analogEnabled[analogConvertPos] == nullptr);
    }

    // start new conversion
    adc = analogAdcMap[analogConvertPos];
    if (analogEven) {
        while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL) {}                            //wait for sync
        adc->INPUTCTRL.bit.MUXPOS = analogEnabled[analogConvertPos]->ulADCChannelNumber; // Selection for the positive ADC input
        adc->CTRLA.bit.ENABLE = 0x01;                                                    // Enable ADC
        while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE) {}                               //wait for sync
    }
    // Start conversion
    //adc->SWTRIG.bit.START = 1;
    adc->INTFLAG.reg = ADC_INTFLAG_RESRDY; // Clear the Data Ready flag
    // Start conversion again, since The first conversion after the reference is changed must not be used.
    adc->SWTRIG.bit.START = 1;

#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 0);
#endif
}
void ADC0_1_Handler(void) {
    analogISRFunction();
}
void ADC1_1_Handler(void) {
    analogISRFunction();
}
void HAL::analogEnable(int channel) {
    int cNum = ANALOG_PIN_TO_CHANNEL(channel);
    if (cNum < 0 || cNum >= 16) {
        return;
    }
    pinPeripheral(channel, PIO_ANALOG);
    {
        InterruptProtectedBlock ip;
        analogEnabled[cNum] = &g_APinDescription[channel];
    }
    if (analogConvertPos == -1) {
        // Attach analog conversion ready interrupts
        NVIC_SetPriority((IRQn_Type)ADC0_1_IRQn, 3);
        NVIC_EnableIRQ(ADC0_1_IRQn); // Enable IRQ for function call
        NVIC_SetPriority((IRQn_Type)ADC1_1_IRQn, 3);
        NVIC_EnableIRQ(ADC1_1_IRQn); // Enable IRQ for function call

        analogISRFunction(); // Start conversion loop
    }
}

int HAL::analogRead(int pin) {
    int cNum = ANALOG_PIN_TO_CHANNEL(pin);
    if (cNum < 0 || cNum >= 16) { // protect for config errors
        return 0;
    }
    return analogValues[cNum];
}

#if EEPROM_AVAILABLE == EEPROM_SDCARD

#if !SDSUPPORT
#error EEPROM using sd card requires SDCARSUPPORT
#endif

millis_t eprSyncTime = 0; // in sync
SdFile eepromFile;
void HAL::syncEEPROM() { // store to disk if changed
    millis_t time = millis();

    if (eprSyncTime && (time - eprSyncTime > 15000)) { // Buffer writes only every 15 seconds to pool writes
        eprSyncTime = 0;
        bool failed = false;
        if (!sd.sdactive) { // not mounted
            if (eepromFile.isOpen())
                eepromFile.close();
            Com::printErrorF("Could not write eeprom to sd card - no sd card mounted");
            Com::println();
            return;
        }

        if (!eepromFile.seekSet(0))
            failed = true;

        if (!failed && !eepromFile.write(virtualEeprom, EEPROM_BYTES) == EEPROM_BYTES)
            failed = true;

        if (failed) {
            Com::printErrorF("Could not write eeprom to sd card");
            Com::println();
        }
    }
}

void HAL::importEEPROM() {
    if (eepromFile.isOpen())
        eepromFile.close();
    if (!eepromFile.open("eeprom.bin", O_RDWR | O_CREAT | O_SYNC) || eepromFile.read(virtualEeprom, EEPROM_BYTES) != EEPROM_BYTES) {
        Com::printFLN(Com::tOpenFailedFile, "eeprom.bin");
    } else {
        Com::printFLN("EEPROM read from sd card.");
    }
    EEPROM::readDataFromEspi::beginEPROM();
}

#endif

// Print apparent cause of start/restart
void HAL::showStartReason() {
    uint8_t mcu = REG_RSTC_RCAUSE;
    if (mcu & RSTC_RCAUSE_POR) {
        Com::printInfoFLN(Com::tPowerUp);
    }
    // this is return from backup mode on SAM
    if (mcu & (RSTC_RCAUSE_BODCORE | RSTC_RCAUSE_BODVDD)) {
        Com::printInfoFLN(Com::tBrownOut);
    }
    if (mcu & RSTC_RCAUSE_WDT) {
        Com::printInfoFLN(Com::tWatchdog);
    }
    Com::printInfoFLN(Com::tSoftwareReset);
    if (mcu & RSTC_RCAUSE_EXT) {
        Com::printInfoFLN(Com::tExternalReset);
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
    NVIC_SystemReset();
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
#if NUM_EXTRUDER > 1
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

void HAL::i2cSetClockspeed(uint32_t clockSpeedHz)

{
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
void HAL::i2cStart(uint8_t address) {
    WIRE_PORT.beginTransmission(address);
}

void HAL::i2cStartRead(uint8_t address, uint8_t bytes) {
    WIRE_PORT.requestFrom(address, bytes);
}
/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 Also specifies internal address of device

 Input:   address and transfer direction of I2C device, internal address
*************************************************************************/
void HAL::i2cStartAddr(uint8_t address, unsigned int pos, uint8_t readBytes) {
    WIRE_PORT.beginTransmission(address);
    WIRE_PORT.write(pos >> 8);
    WIRE_PORT.write(pos & 255);
    if (readBytes) {
        WIRE_PORT.endTransmission();
        WIRE_PORT.requestFrom(address, readBytes);
    }
}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void) {
    WIRE_PORT.endTransmission();
}

/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
*************************************************************************/
void HAL::i2cWrite(uint8_t data) {
    WIRE_PORT.write(data);
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
int HAL::i2cRead(void) {
    if (WIRE_PORT.available()) {
        return WIRE_PORT.read();
    }
    return -1; // should never happen, but better then blocking
}

#if NUM_SERVOS > 0
unsigned int HAL::servoTimings[4] = { 0, 0, 0, 0 };
unsigned int servoAutoOff[4] = { 0, 0, 0, 0 };
static uint8_t servoIndex = 0;

void HAL::servoMicroseconds(uint8_t servo, int microsec, uint16_t autoOff) {
    servoTimings[servo] = (uint32_t)((((F_CPU_TRUE / SERVO_PRESCALE) / 1000) * microsec) / 1000);
    servoAutoOff[servo] = (microsec) ? (autoOff / 20) : 0;
}

// ================== Interrupt handling ======================

ServoInterface* analogServoSlots[4] = { nullptr, nullptr, nullptr, nullptr };
// Servo timer Interrupt handler
void SERVO_TIMER_VECTOR() {
    if (SERVO_TIMER->COUNT16.INTFLAG.bit.MC0 == 1) {
        SERVO_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
        static uint32_t interval;

        fast8_t servoId = servoIndex >> 1;
        ServoInterface* act = analogServoSlots[servoId];
        if (act == nullptr) {
            SERVO_TIMER->COUNT16.CC[0].reg = SERVO2500US;
            SYNC_TIMER(SERVO_TIMER);
        } else {
            if (servoIndex & 1) { // disable
                act->disable();
                SERVO_TIMER->COUNT16.CC[0].reg = SERVO5000US - interval;
                SYNC_TIMER(SERVO_TIMER);
                if (servoAutoOff[servoId]) {
                    servoAutoOff[servoId]--;
                    if (servoAutoOff[servoId] == 0)
                        HAL::servoTimings[servoId] = 0;
                }
            } else { // enable
                InterruptProtectedBlock noInt;
                if (HAL::servoTimings[servoId]) {
                    act->enable();
                    interval = HAL::servoTimings[servoId];
                    SERVO_TIMER->COUNT16.CC[0].reg = interval;
                    SYNC_TIMER(SERVO_TIMER);
                } else {
                    interval = SERVO2500US;
                    SERVO_TIMER->COUNT16.CC[0].reg = interval;
                    SYNC_TIMER(SERVO_TIMER);
                }
            }
        }
        servoIndex++;
        if (servoIndex > 7) {
            servoIndex = 0;
        }
    }
}
#endif

/** \brief Timer interrupt routine to drive the stepper motors.
*/
void MOTION3_TIMER_VECTOR() {
    if (MOTION3_TIMER->COUNT16.INTFLAG.bit.MC0 == 1) {
#if DEBUG_TIMING
        WRITE(DEBUG_ISR_STEPPER_PIN, 1);
#endif
        Motion3::timer();
#if DEBUG_TIMING
        WRITE(DEBUG_ISR_STEPPER_PIN, 0);
#endif
        MOTION3_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    }
}

fast8_t pwmSteps[] = { 1, 2, 4, 8, 16 };
fast8_t pwmMasks[] = { 255, 254, 252, 248, 240 };

#define pulseDensityModulate(pin, density, error, invert) \
    { \
        uint8_t carry; \
        carry = error + (invert ? 255 - density : density); \
        WRITE(pin, (carry < error)); \
        error = carry; \
    }

/**
This timer is called 5000 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/
void PWM_TIMER_VECTOR() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 1);
#endif
    //InterruptProtectedBlock noInt;
    if (PWM_TIMER->COUNT16.INTFLAG.bit.MC0 == 1) {
        PWM_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;

        static uint8_t pwm_count0 = 0; // Used my IO_PWM_SOFTWARE!
        static uint8_t pwm_count1 = 0;
        static uint8_t pwm_count2 = 0;
        static uint8_t pwm_count3 = 0;
        static uint8_t pwm_count4 = 0;

// Add all generated pwm handlers
#undef IO_TARGET
#define IO_TARGET 2
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
            if (!WDT->SYNCBUSY.bit.CLEAR) // Check if the WDT registers are synchronized
            {
                REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY; // Clear the watchdog timer
            }
            HAL::wdPinged = false;
        }
#endif
#if DEBUG_TIMING
        WRITE(DEBUG_ISR_TEMP_PIN, 0);
#endif
    }
}

// MOTION2_TIMER IRQ handler
void MOTION2_TIMER_VECTOR() {
    // static bool inside = false; // prevent double call when not finished
    if (MOTION2_TIMER->COUNT16.INTFLAG.bit.MC0 == 1) {
#if DEBUG_TIMING
        WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
        /*      if (inside) {
            return;
        }
        inside = true;*/
        Motion2::timer();
        //        inside = false;
        MOTION2_TIMER->COUNT16.INTFLAG.bit.MC0 = 1; // reenable interrupt
#if DEBUG_TIMING
        WRITE(DEBUG_ISR_MOTION_PIN, 0);
#endif
    }
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

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
RFDoubleSerial::RFDoubleSerial() {
}
void RFDoubleSerial::begin(unsigned long baud) {
    RFSERIAL.begin(baud);
    BT_SERIAL.begin(BLUETOOTH_BAUD);
}

void RFDoubleSerial::end() {
    RFSERIAL.end();
    BT_SERIAL.end();
}
int RFDoubleSerial::available(void) {
    int x = RFSERIAL.available();
    if (x > 0)
        return x;
    return BT_SERIAL.available();
}
int RFDoubleSerial::peek(void) {
    if (RFSERIAL.available())
        return RFSERIAL.peek();
    return BT_SERIAL.peek();
}
int RFDoubleSerial::read(void) {
    if (RFSERIAL.available())
        return RFSERIAL.read();
    return BT_SERIAL.read();
}
void RFDoubleSerial::flush(void) {
    RFSERIAL.flush();
    BT_SERIAL.flush();
}
size_t RFDoubleSerial::write(uint8_t c) {
    size_t r = RFSERIAL.write(c);
    BT_SERIAL.write(c);
    return r;
}
RFDoubleSerial BTAdapter;
#endif

// Dummy function to overload weak arduino function that always disables
// watchdog. We do not need that as we do this our self.
void watchdogSetup(void) {
}

#if BEEPER_PIN > -1
void TC3_Handler(void) {
    static bool toggle;
    WRITE(BEEPER_PIN, toggle);
    toggle = !toggle;
    TONE_TC->COUNT16.INTFLAG.bit.MC0 = 1; // Clear the interrupt
}
#endif

void HAL::tone(int frequency) {
#if BEEPER_PIN >= 0
    SET_OUTPUT(BEEPER_PIN);
    NVIC_SetPriority(TONE_TC_IRQn, 2); // don'r disturb stepper interrupt!
    GCLK->PCHCTRL[TONE_TC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0) {}

    TONE_TC->COUNT16.CTRLA.bit.ENABLE = 0; // can only change if disabled

    // Use match mode so that the timer counter resets when the count matches the
    // compare register
    TONE_TC->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    SYNC_TIMER(TONE_TC);
    TONE_TC->COUNT16.INTENSET.reg = 0;
    TONE_TC->COUNT16.INTENSET.bit.MC0 = 1;
    uint32_t prescale, freq;
    getPrescaleFreq(F_CPU_TRUE / (2 * frequency), prescale, freq);
    TONE_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
    TONE_TC->COUNT16.CTRLA.reg |= prescale;
    NVIC_EnableIRQ(TONE_TC_IRQn); // Enable IRQ for function call
    TONE_TC->COUNT16.COUNT.reg = map(TONE_TC->COUNT16.COUNT.reg, 0,
                                     TONE_TC->COUNT16.CC[0].reg, 0, freq);
    TONE_TC->COUNT16.CC[0].reg = freq;
    SYNC_TIMER(TONE_TC);
    TONE_TC->COUNT16.CTRLA.bit.ENABLE = 1; // enable timer
    SYNC_TIMER(TONE_TC);
#endif
}
void HAL::noTone() {
#if BEEPER_PIN >= 0
    // Disable TCx
    TONE_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TONE_TC->COUNT16.SYNCBUSY.bit.ENABLE) {}
    TONE_TC->COUNT16.CTRLA.reg = TC_CTRLA_SWRST; // Reset timer
    while (TONE_TC->COUNT16.SYNCBUSY.bit.ENABLE) {}
    while (TONE_TC->COUNT16.CTRLA.bit.SWRST) {}
    WRITE(BEEPER_PIN, 0);
#endif
}

#endif
