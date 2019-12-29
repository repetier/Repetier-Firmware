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
#ifdef STM32F4_BOARD
#include <stm32f4xx_hal.h>
#include <malloc.h>

// Create timer names from TIMER_NUM macro

#define _TIMER(num) TIM##num
#define _TIMER_CLK_ENABLE(num) __HAL_RCC_TIM##num##_CLK_ENABLE
#define _TIMER_IRQ(num) TIM##num##_IRQn
// #define _TIMER_VECTOR(num) TC##num##_Handler(stm32_timer_t* htim)
#define _TIMER_VECTOR(num) TC##num##_Handler()

#define TIMER(num) _TIMER(num)
#define TIMER_CLK_ENABLE(num) _TIMER_CLK_ENABLE(num)
#define TIMER_IRQ(num) _TIMER_IRQ(num)
#define TIMER_VECTOR(num) _TIMER_VECTOR(num)

// Adds output signals to pins below to measure interrupt timings
// with a logic analyser. Set to 0 for production!
#define DEBUG_TIMING 1
#define DEBUG_ISR_STEPPER_PIN 60 // PD12
#define DEBUG_ISR_MOTION_PIN 61  // PD13
#define DEBUG_ISR_TEMP_PIN 62    // PD14/PWM1
#define DEBUG_ISR_ANALOG_PIN 63  // PD15/PWM2

//extern "C" void __cxa_pure_virtual() { }
extern "C" char* sbrk(int i);

char HAL::virtualEeprom[EEPROM_BYTES] = { 0 };
bool HAL::wdPinged = true;
volatile uint8_t HAL::insideTimer1 = 0;

enum class TimerUsage {
    UNUSED,
    INTERRUPT_TIMER,
    PWM_TIMER
};

struct TimerFunction {
    TIM_TypeDef* tim;
    int num;
    TimerUsage usage;
    uint32_t frequency;
    uint32_t reserved;
    TIM_HandleTypeDef def;
};

struct PWMEntry {
    TimerFunction* function;
    const PinMap* map;
    HardwareTimer* ht;
};
static PWMEntry pwmEntries[40];
static int numPWMEntries = 0;

TimerFunction timerList[] = {
#ifdef TIM1
#define TIM1_COUNT 1
    { TIM1, 1, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM1_COUNT 2
#endif
#ifdef TIM2
#define TIM2_COUNT 1
    ,
    { TIM2, 2, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM2_COUNT 0
#endif
#ifdef TIM3
#define TIM3_COUNT 1
    ,
    { TIM3, 3, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM3_COUNT 0
#endif
#ifdef TIM4
#define TIM4_COUNT 1
    ,
    { TIM4, 4, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM4_COUNT 0
#endif
#ifdef TIM5
#define TIM5_COUNT 1
    ,
    { TIM5, 5, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM5_COUNT 0
#endif
#ifdef TIM6
#define TIM6_COUNT 1
    ,
    { TIM6, 6, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM6_COUNT 0
#endif
#ifdef TIM7
#define TIM7_COUNT 1
    ,
    { TIM7, 7, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM7_COUNT 0
#endif
#ifdef TIM8
#define TIM8_COUNT 1
    ,
    { TIM8, 8, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM8_COUNT 0
#endif
#ifdef TIM9
#define TIM9_COUNT 1
    ,
    { TIM9, 9, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM9_COUNT 0
#endif
#ifdef TIM10
#define TIM10_COUNT 1
    ,
    { TIM2, 10, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM10_COUNT 0
#endif
#ifdef TIM11
#define TIM11_COUNT 1
    ,
    { TIM11, 11, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM11_COUNT 0
#endif
#ifdef TIM12
#define TIM12_COUNT 1
    ,
    { TIM12, 12, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM12_COUNT 0
#endif
#ifdef TIM13
#define TIM13_COUNT 1
    ,
    { TIM13, 13, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM13_COUNT 0
#endif
#ifdef TIM14
#define TIM14_COUNT 1
    ,
    { TIM14, 14, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM14_COUNT 0
#endif
#ifdef TIM15
#define TIM15_COUNT 1
    ,
    { TIM15, 15, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM15_COUNT 0
#endif
#ifdef TIM16
#define TIM16_COUNT 1
    ,
    { TIM16, 16, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM16_COUNT 0
#endif
#ifdef TIM17
#define TIM17_COUNT 1
    ,
    { TIM17, 17, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM17_COUNT 0
#endif
#ifdef TIM18
#define TIM18_COUNT 1
    ,
    { TIM18, 18, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM18_COUNT 0
#endif
#ifdef TIM19
#define TIM19_COUNT 1
    ,
    { TIM19, 19, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM19_COUNT 0
#endif
#ifdef TIM20
#define TIM20_COUNT 1
    ,
    { TIM20, 20, TimerUsage::UNUSED, 0, 0 }
#else
#define TIM20_COUNT 0
#endif
};

#define NUM_TIMERS (TIM1_COUNT + TIM2_COUNT + TIM3_COUNT + TIM4_COUNT + TIM5_COUNT + TIM6_COUNT + TIM7_COUNT + TIM8_COUNT + TIM9_COUNT + TIM10_COUNT + TIM11_COUNT + TIM12_COUNT + TIM13_COUNT + TIM14_COUNT + TIM15_COUNT + TIM16_COUNT + TIM17_COUNT + TIM18_COUNT + TIM19_COUNT + TIM20_COUNT)

TimerFunction* reserveTimerInterrupt(int num) {
    for (int i = 0; i < NUM_TIMERS; i++) {
        if (timerList[i].num == num) {
            timerList[i].usage = TimerUsage::INTERRUPT_TIMER;
            return &timerList[i];
        }
    }
    return nullptr;
}
HAL::HAL() {
    //ctor
}

HAL::~HAL() {
    //dtor
}

/*
Get best prescaler to fit into 16 bit timer with 16bit prescaler
*/

void getPrescaleFreq(uint32_t ticks, uint32_t& prescale, uint32_t& freq) {
    prescale = 1;
    freq = ticks;
    while (freq > 65535ul) {
        freq >>= 1;
        prescale <<= 1;
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

    TimerFunction* motion2 = reserveTimerInterrupt(MOTION2_TIMER_NUM); // prevent pwm usage
    getPrescaleFreq(F_CPU_TRUE / PREPARE_FREQUENCY, prescale, freq);
    motion2->def.Instance = TIMER(MOTION2_TIMER_NUM);
    motion2->def.Init.Prescaler = prescale;
    motion2->def.Init.Period = freq;
    motion2->def.Init.RepetitionCounter = 0;
    motion2->def.Init.CounterMode = TIM_COUNTERMODE_UP;
    motion2->def.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&motion2->def) == HAL_OK) {
        HAL_TIM_Base_Start_IT(&motion2->def);
    }
    HAL_NVIC_SetPriority(TIMER_IRQ(MOTION2_TIMER_NUM), 2, 0);
    HAL_NVIC_EnableIRQ(TIMER_IRQ(MOTION2_TIMER_NUM)); // 1 = highest, 3 = lowest priority

    // Regular interrupts for heater control etc

    TimerFunction* pwm = reserveTimerInterrupt(PWM_TIMER_NUM); // prevent pwm usage
    getPrescaleFreq(F_CPU_TRUE / PWM_CLOCK_FREQ, prescale, freq);
    pwm->def.Instance = TIMER(PWM_TIMER_NUM);
    pwm->def.Init.Prescaler = prescale;
    pwm->def.Init.Period = freq;
    pwm->def.Init.RepetitionCounter = 0;
    pwm->def.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm->def.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&pwm->def) == HAL_OK) {
        HAL_TIM_Base_Start_IT(&pwm->def);
    }
    HAL_NVIC_SetPriority(TIMER_IRQ(PWM_TIMER_NUM), 2, 0);
    HAL_NVIC_EnableIRQ(TIMER_IRQ(PWM_TIMER_NUM)); // 1 = highest, 3 = lowest priority

    // Timer for stepper motor control

    TimerFunction* motion3 = reserveTimerInterrupt(MOTION3_TIMER_NUM); // prevent pwm usage
    getPrescaleFreq(F_CPU_TRUE / STEPPER_FREQUENCY, prescale, freq);
    motion3->def.Instance = TIMER(MOTION3_TIMER_NUM);
    motion3->def.Init.Prescaler = prescale;
    motion3->def.Init.Period = freq;
    motion3->def.Init.RepetitionCounter = 0;
    motion3->def.Init.CounterMode = TIM_COUNTERMODE_UP;
    motion3->def.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&motion2->def) == HAL_OK) {
        HAL_TIM_Base_Start_IT(&motion2->def);
    }
    HAL_NVIC_SetPriority(TIMER_IRQ(MOTION3_TIMER_NUM), 0, 0); // 1 = highest, 3 = lowest priority
    HAL_NVIC_EnableIRQ(TIMER_IRQ(MOTION3_TIMER_NUM));

    // Servo control
#if NUM_SERVOS > 0

    TimerFunction* servo = reserveTimerInterrupt(SERVO_TIMER_NUM); // prevent pwm usage
    // getPrescaleFreq(F_CPU_TRUE / 50, prescale, freq);
    servo->def.Instance = TIMER(SERVO_TIMER_NUM);
    servo->def.Init.Prescaler = SERVO_PRESCALE;
    servo->def.Init.Period = SERVO5000US;
    servo->def.Init.RepetitionCounter = 0;
    servo->def.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo->def.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&motion2->def) == HAL_OK) {
        HAL_TIM_Base_Start_IT(&motion2->def);
    }
    HAL_NVIC_SetPriority(TIMER_IRQ(SERVO_TIMER_NUM), 1, 0); // 1 = highest, 3 = lowest priority
    HAL_NVIC_EnableIRQ(TIMER_IRQ(SERVO_TIMER_NUM));
#endif
}

/* static void computePWMDivider(uint32_t frequency, uint32_t& div, uint32_t& scale, int bits) {
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
} */

// Try to initialize pinNumber as hardware PWM. Returns internal
// id if it succeeds or -1 if it fails. Typical reasons to fail
// are no pwm support for that pin or an other pin uses same PWM
// channel.
int HAL::initHardwarePWM(int pinNumber, uint32_t frequency) {
    if (pinNumber < 0) {
        return 255;
    }
    PinName p = digitalPinToPinName(pinNumber);
    if (p != NC) {
        const PinMap* map = PinMap_PWM;
        while (map->pin != NC) {
            if (map->pin == p) {
                break;
            }
            map++;
        }
        if (map->pin != NC) {
            TimerFunction* tf = nullptr;
            for (int i = 0; i < NUM_TIMERS; i++) {
                if (timerList[i].tim == map->peripheral) {
                    tf = &timerList[i];
                    break;
                }
            }
            if (tf == nullptr || tf->usage == TimerUsage::INTERRUPT_TIMER) {
                return -1; // already used
            }
            if (tf->usage == TimerUsage::UNUSED) {
                tf->usage = TimerUsage::PWM_TIMER;
                tf->frequency = frequency;
                tf->reserved = 1;
            } else {
                tf->reserved++;
            }
            pwmEntries[numPWMEntries].function = tf;
            pwmEntries[numPWMEntries].map = map;
            // Find timer in local usage list

            TIM_TypeDef* Instance = (TIM_TypeDef*)tf->tim;
            HardwareTimer* HT;
            uint32_t index = get_timer_index(tf->tim);
            if (HardwareTimer_Handle[index] == nullptr) {
                HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)map->peripheral);
            }

            HT = (HardwareTimer*)(HardwareTimer_Handle[index]->__this);
            pwmEntries[numPWMEntries].ht = HT;
            uint32_t channel = STM_PIN_CHANNEL(map->function);

            HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, p);
            HT->setOverflow(tf->frequency, HERTZ_FORMAT);
            HT->setCaptureCompare(channel, 0, RESOLUTION_12B_COMPARE_FORMAT); // set pwm 0
            HT->resume();
            return numPWMEntries++;
        }
    }
    return -1;
}
// Set pwm output to value. id is id from initHardwarePWM.
void HAL::setHardwarePWM(int id, int value) {
    if (id < 0 || id >= 50) { // illegal id
        return;
    }
    PWMEntry& entry = pwmEntries[id];
    uint32_t channel = STM_PIN_CHANNEL(entry.map->function);
    entry.ht->setCaptureCompare(channel, value, RESOLUTION_12B_COMPARE_FORMAT); // set pwm 0
}

ADC_HandleTypeDef AdcHandle = {};
struct AnalogFunction {
    bool enabled;
    int32_t channel;
    int32_t lastValue;
    ADC_TypeDef* def;
};

// Initialize ADC channels
static AnalogFunction analogValues[MAX_ANALOG_INPUTS] = { false, -1, 0, nullptr };
static AnalogFunction* analogMap[256]; // Map pin number to entry in analogValues
int numAnalogInputs = 0;
uint16_t adcData[16]; // Target for DMA adc transfer

void reportAnalog() {
    for (int i = 0; i < 256; i++) {
        if (analogMap[i]) {
            Com::printF("Analog ", i);
            Com::printFLN(" = ", static_cast<int32_t>(adcData[i]));
        }
    }
}

static int analogConvertPos = -1;
void HAL::analogStart(void) {
    // Analog channels being used are already enabled. Start conversion
    // only if we have any analog sources.
    if (numAnalogInputs == 0) {
        return;
    }
    analogConvertPos = 0;
    ADC_ChannelConfTypeDef sConfig = { 0 };
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE(); // Enable DMA2 clock
    AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    AdcHandle.Init.ScanConvMode = ENABLE;
    AdcHandle.Init.ContinuousConvMode = ENABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.NbrOfConversion = numAnalogInputs;
    AdcHandle.Init.DMAContinuousRequests = ENABLE;
    AdcHandle.Init.EOCSelection = DISABLE;
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        return;
    }
    for (int i = 0; i < numAnalogInputs; i++) {
        /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
        sConfig.Channel = analogValues[i].channel;
        sConfig.Rank = i + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
            Error_Handler();
        }
    }

    // HAL_ADC_Start_IT(&AdcHandle); // start samling over interrupt
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&adcData, numAnalogInputs);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 1);
#endif
    __HAL_ADC_DISABLE(AdcHandle);
    for (int i = 0; i < numAnalogInputs; i++) {
        analogValues[i].lastValue = adcData[i];
    }
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ANALOG_INPUT_LOOP
#include "io/redefine.h"
    analogConvertPos = 0;
    __HAL_ADC_ENABLE(AdcHandle);

    // start new conversion
#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 0);
#endif
}

void HAL::analogEnable(int pinId) {
    PinName pin = static_cast<PinName>(pinId);
    AnalogFunction* af = &analogValues[numAnalogInputs++];
    af->def = static_cast<ADC_TypeDef*>(pinmap_peripheral(pin, PinMap_ADC));
    if (af->def == nullptr) {
        analogMap[pinId] = nullptr;
        --numAnalogInputs;
        return;
    }
    af->enabled = true;
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    af->channel = 0;
    switch (STM_PIN_CHANNEL(function)) {
#ifdef ADC_CHANNEL_0
    case 0:
        af->channel = ADC_CHANNEL_0;
        break;
#endif
    case 1:
        af->channel = ADC_CHANNEL_1;
        break;
    case 2:
        af->channel = ADC_CHANNEL_2;
        break;
    case 3:
        af->channel = ADC_CHANNEL_3;
        break;
    case 4:
        af->channel = ADC_CHANNEL_4;
        break;
    case 5:
        af->channel = ADC_CHANNEL_5;
        break;
    case 6:
        af->channel = ADC_CHANNEL_6;
        break;
    case 7:
        af->channel = ADC_CHANNEL_7;
        break;
    case 8:
        af->channel = ADC_CHANNEL_8;
        break;
    case 9:
        af->channel = ADC_CHANNEL_9;
        break;
    case 10:
        af->channel = ADC_CHANNEL_10;
        break;
    case 11:
        af->channel = ADC_CHANNEL_11;
        break;
    case 12:
        af->channel = ADC_CHANNEL_12;
        break;
    case 13:
        af->channel = ADC_CHANNEL_13;
        break;
    case 14:
        af->channel = ADC_CHANNEL_14;
        break;
    case 15:
        af->channel = ADC_CHANNEL_15;
        break;
#ifdef ADC_CHANNEL_16
    case 16:
        af->channel = ADC_CHANNEL_16;
        break;
#endif
    case 17:
        af->channel = ADC_CHANNEL_17;
        break;
#ifdef ADC_CHANNEL_18
    case 18:
        af->channel = ADC_CHANNEL_18;
        break;
#endif
#ifdef ADC_CHANNEL_19
    case 19:
        af->channel = ADC_CHANNEL_19;
        break;
#endif
#ifdef ADC_CHANNEL_20
    case 20:
        af->channel = ADC_CHANNEL_20;
        break;
    case 21:
        af->channel = ADC_CHANNEL_21;
        break;
    case 22:
        af->channel = ADC_CHANNEL_22;
        break;
    case 23:
        af->channel = ADC_CHANNEL_23;
        break;
    case 24:
        af->channel = ADC_CHANNEL_24;
        break;
    case 25:
        af->channel = ADC_CHANNEL_25;
        break;
    case 26:
        af->channel = ADC_CHANNEL_26;
        break;
#ifdef ADC_CHANNEL_27
    case 27:
        af->channel = ADC_CHANNEL_27;
        break;
    case 28:
        af->channel = ADC_CHANNEL_28;
        break;
    case 29:
        af->channel = ADC_CHANNEL_29;
        break;
    case 30:
        af->channel = ADC_CHANNEL_30;
        break;
    case 31:
        af->channel = ADC_CHANNEL_31;
        break;
#endif
#endif
    default:
        af->channel = 0;
        break;
    }
}

int HAL::analogRead(int pin) {
    AnalogFunction* af = analogMap[pin];
    if (af == nullptr) { // protect for config errors
        return 0;
    }
    return af->lastValue;
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
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        Com::printInfoFLN(PSTR("Low power reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        Com::printInfoFLN(Com::tWatchdog);
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        Com::printInfoFLN(PSTR("Independent watchdog reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        Com::printInfoFLN(PSTR("Software reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        Com::printInfoFLN(Com::tPowerUp);
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        Com::printInfoFLN(PSTR("External reset pin reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
        Com::printInfoFLN(Com::tBrownOut);
    } else {
        Com::printInfoFLN(PSTR("Unknown reset reason"));
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();
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
extern "C" void TIMER_VECTOR(SERVO_TIMER_NUM) {
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
extern "C" void TIMER_VECTOR(MOTION3_TIMER_NUM) {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_STEPPER_PIN, 1);
#endif
    Motion3::timer();
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
This timer is called 5000 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/
extern "C" void TIMER_VECTOR(PWM_TIMER_NUM) {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 1);
#endif
    //InterruptProtectedBlock noInt;

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
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_TEMP_PIN, 0);
#endif
}

// MOTION2_TIMER IRQ handler
extern "C" void TIMER_VECTOR(MOTION2_TIMER_NUM) {
    // static bool inside = false; // prevent double call when not finished
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
    /*      if (inside) {
            return;
        }
        inside = true;*/
    Motion2::timer();
    //        inside = false;
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 0);
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
}
#endif

void HAL::tone(int frequency) {
#if BEEPER_PIN >= 0
/*    SET_OUTPUT(BEEPER_PIN);
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
    SYNC_TIMER(TONE_TC);*/
#endif
}
void HAL::noTone() {
#if BEEPER_PIN >= 0
    // Disable TCx
    /*   TONE_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TONE_TC->COUNT16.SYNCBUSY.bit.ENABLE) {}
    TONE_TC->COUNT16.CTRLA.reg = TC_CTRLA_SWRST; // Reset timer
    while (TONE_TC->COUNT16.SYNCBUSY.bit.ENABLE) {}
    while (TONE_TC->COUNT16.CTRLA.bit.SWRST) {}
    WRITE(BEEPER_PIN, 0);*/
#endif
}

#endif
