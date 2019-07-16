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
    pmc_enable_periph_clk(MOTION2_TIMER_IRQ); // enable power to timer
    //NVIC_SetPriority((IRQn_Type)EXTRUDER_TIMER_IRQ, NVIC_EncodePriority(4, 4, 1));
    NVIC_SetPriority((IRQn_Type)MOTION2_TIMER_IRQ, 15);

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

    // Regular interrupts for heater control etc
    pmc_enable_periph_clk(PWM_TIMER_IRQ);
    //NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, NVIC_EncodePriority(4, 6, 0));
    NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, 15);

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
    NVIC_SetPriority((IRQn_Type)MOTION3_TIMER_IRQ, 2); // highest priority - no surprises here wanted
    TC_Configure(MOTION3_TIMER, MOTION3_TIMER_CHANNEL,
                 TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_SetRC(MOTION3_TIMER, MOTION3_TIMER_CHANNEL, (F_CPU_TRUE / 2) / STEPPER_FREQUENCY);
    TC_Start(MOTION3_TIMER, MOTION3_TIMER_CHANNEL);

    MOTION3_TIMER->TC_CHANNEL[MOTION3_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    MOTION3_TIMER->TC_CHANNEL[MOTION3_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)MOTION3_TIMER_IRQ);

    // Servo control
#if NUM_SERVOS > 0
    pmc_enable_periph_clk(SERVO_TIMER_IRQ);
    //NVIC_SetPriority((IRQn_Type)SERVO_TIMER_IRQ, NVIC_EncodePriority(4, 5, 0));
    NVIC_SetPriority((IRQn_Type)SERVO_TIMER_IRQ, 4);

    TC_Configure(SERVO_TIMER, SERVO_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK1);

    TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, (F_CPU_TRUE / SERVO_PRESCALE) / SERVO_CLOCK_FREQ);
    TC_Start(SERVO_TIMER, SERVO_TIMER_CHANNEL);

    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)SERVO_TIMER_IRQ);
#endif
}

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
    do {
        scale = VARIANT_MCK / (frequency * factor);
        if (scale <= 65535) {
            return;
        }
        div = factor;
        factor <<= 1;
    } while (factor <= 1024);
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
    if (foundPin == -1) {
        return -1;
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
        if ((PWM_INTERFACE->PWM_SR & (1 << id)) == 0) { // disabled, set value
            PWM_INTERFACE->PWM_CH_NUM[id].PWM_CDTY = duty;
        } else { // just update
            PWM_INTERFACE->PWM_CH_NUM[id].PWM_CDTYUPD = duty;
        }
        return;
    }
    // TODO: timers can also produce PWM
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
        if (analogEnabled[i] == false) {
            continue;
        }
        adcEnable |= (0x1u << i);
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
    EEPROM::readDataFromEEPROM();
}

#endif

// Print apparent cause of start/restart
void HAL::showStartReason() {
    int mcu = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
    switch (mcu) {
    case 0:
        Com::printInfoFLN(Com::tPowerUp);
        break;
    case 1:
        // this is return from backup mode on SAM
        Com::printInfoFLN(Com::tBrownOut);
    case 2:
        Com::printInfoFLN(Com::tWatchdog);
        break;
    case 3:
        Com::printInfoFLN(Com::tSoftwareReset);
        break;
    case 4:
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
    if (readBytes != 0) {
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
    servoTimings[servo] = (unsigned int)(((F_CPU_TRUE / SERVO_PRESCALE) / 1000000) * microsec);
    servoAutoOff[servo] = (microsec) ? (autoOff / 20) : 0;
}

// ================== Interrupt handling ======================

ServoInterface* analogServoSlots[4] = { nullptr, nullptr, nullptr, nullptr };
// Servo timer Interrupt handler
void SERVO_TIMER_VECTOR() {
    static uint32_t interval;

    // apparently have to read status register
    TC_GetStatus(SERVO_TIMER, SERVO_TIMER_CHANNEL);

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
                if (servoAutoOff[servoId] == 0)
                    HAL::servoTimings[servoId] = 0;
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

#define pulseDensityModulate(pin, density, error, invert) \
    { \
        uint8_t carry; \
        carry = error + (invert ? 255 - density : density); \
        WRITE(pin, (carry < error)); \
        error = carry; \
    }

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
    TC_GetStatus(PWM_TIMER, PWM_TIMER_CHANNEL);

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
    // read analog values
    //#if ANALOG_INPUTS > 0
    // conversion finished?
    if ((ADC->ADC_ISR & adcEnable) == adcEnable) {
#undef IO_TARGET
#define IO_TARGET 11
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
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 0);
#endif
}

TcChannel* motion2Channel = (MOTION2_TIMER->TC_CHANNEL + MOTION2_TIMER_CHANNEL);

// MOTION2_TIMER IRQ handler
void MOTION2_TIMER_VECTOR() {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
    static bool inside = false; // prevent double call when not finished
    motion2Channel->TC_SR;      // faster replacement for above line!
    if (inside) {
        return;
    }
    inside = true;
    Motion2::timer();
    inside = false;
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 0);
#endif
}

// IRQ handler for tone generator
#if BEEPER_PIN > -1
void BEEPER_TIMER_VECTOR() {
    static bool toggle;

    TC_GetStatus(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);

    WRITE(BEEPER_PIN, toggle);
    toggle = !toggle;
}
#endif

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
#endif
