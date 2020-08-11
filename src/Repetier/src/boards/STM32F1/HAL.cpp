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

#include "Repetier.h"
#ifdef STM32F1_BOARD
#include <stm32f1xx_hal.h>
#include <malloc.h>

#include "PeripheralPins.h"
#if MOTHERBOARD == MOTHERBOARD_E3_MINI_V1_2
constexpr PinMap PinMap_PWM[] = {
    { PA_0, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0) }, // TIM2_CH1
// {PA_0,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_2, 1, 0)}, // TIM2_CH1
#ifdef STM32F103xE
// {PA_0,  TIM5,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0)}, // TIM5_CH1
#endif
    { PA_1, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0) }, // TIM2_CH2
// {PA_1,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_2, 2, 0)}, // TIM2_CH2
#ifdef STM32F103xE
// {PA_1,  TIM5,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0)}, // TIM5_CH2
#endif
    { PA_2, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0) }, // TIM2_CH3
// {PA_2,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_1, 3, 0)}, // TIM2_CH3
#ifdef STM32F103xE
// {PA_2,  TIM5,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0)}, // TIM5_CH3
#endif
#ifndef STM32F103xE
    { PA_3, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0) }, // TIM2_CH4
// {PA_3,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_1, 4, 0)}, // TIM2_CH4
#else
    { PA_3, TIM5, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0) }, // TIM5_CH4
#endif
    { PA_6, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0) }, // TIM3_CH1
#ifndef STM32F103xE
    { PA_7, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 1, 1) }, // TIM1_CH1N
// {PA_7,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0)}, // TIM3_CH2
#else
    { PA_7, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 1) }, // TIM8_CH1N
#endif
    { PA_8, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0) }, // TIM1_CH1
    // {PA_8,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 1, 0)}, // TIM1_CH1
    { PA_9, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0) }, // TIM1_CH2
    // {PA_9,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 2, 0)}, // TIM1_CH2
    { PA_10, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0) }, // TIM1_CH3
    // {PA_10, TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 3, 0)}, // TIM1_CH3
    { PA_11, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0) }, // TIM1_CH4
    // {PA_11, TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 4, 0)}, // TIM1_CH4
    { PA_15, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_1, 1, 0) }, // TIM2_CH1
    // {PA_15, TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_ENABLE, 1, 0)}, // TIM2_CH1
    // {PB_0,  TIM1,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 2, 1)}, // TIM1_CH2N
    // {PB_0,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0)}, // TIM3_CH3
    { PB_0, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_PARTIAL, 3, 0) }, // TIM3_CH3
#ifdef STM32F103xE
// {PB_0,  TIM8,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 1)}, // TIM8_CH2N
#endif
    { PB_1, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM1_PARTIAL, 3, 1) }, // TIM1_CH3N
// {PB_1,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0)}, // TIM3_CH4
// {PB_1,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_PARTIAL, 4, 0)}, // TIM3_CH4
#ifdef STM32F103xE
// {PB_1,  TIM8,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 1)}, // TIM8_CH3N
#endif
    { PB_3, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_1, 2, 0) }, // TIM2_CH2
    // {PB_3,  TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_ENABLE, 2, 0)}, // TIM2_CH2
    { PB_4, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_PARTIAL, 1, 0) },    // TIM3_CH1
    { PB_5, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_PARTIAL, 2, 0) },    // TIM3_CH2
    { PB_6, TIM4, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0) },            // TIM4_CH1
    { PB_7, TIM4, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0) },            // TIM4_CH2
    { PB_8, TIM4, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0) },            // TIM4_CH3
    { PB_9, TIM4, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0) },            // TIM4_CH4
    { PB_10, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_2, 3, 0) }, // TIM2_CH3
    // {PB_10, TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_ENABLE, 3, 0)}, // TIM2_CH3
    { PB_11, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_PARTIAL_2, 4, 0) }, // TIM2_CH4
    // {PB_11, TIM2,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM2_ENABLE, 4, 0)}, // TIM2_CH4
    { PB_13, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 1) },       // TIM1_CH1N
    { PB_14, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 1) },       // TIM1_CH2N
    { PB_15, TIM1, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 1) },       // TIM1_CH3N
    { PC_6, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_ENABLE, 1, 0) }, // TIM3_CH1
#ifdef STM32F103xE
// {PC_6,  TIM8,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 1, 0)}, // TIM8_CH1
#endif
    { PC_7, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_ENABLE, 2, 0) }, // TIM3_CH2
#ifdef STM32F103xE
// {PC_7,  TIM8,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 2, 0)}, // TIM8_CH2
#endif
#ifndef STM32F103xE
    { PC_8, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_ENABLE, 3, 0) }, // TIM3_CH3
#else
    { PC_8, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 3, 0) }, // TIM8_CH3
#endif
#if 0
  {PC_9,  TIM3,   STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_TIM3_ENABLE, 4, 0)}, // TIM3_CH4 Moses - switched over to use TIM8 instead.
#endif
    { PC_9, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, AFIO_NONE, 4, 0) }, // TIM8_CH4
    { NC, NP, 0 }
};

#endif

// Create timer names from TIMER_NUM macro

#define _TIMER(num) TIM##num
#define _TIMER_IRQ(num) TIM##num##_IRQn
#define _TIMER_VECTOR(num) RF_TC##num##_Handler(HardwareTimer*)
#define _TIMER_VECTOR_NAME(num) RF_TC##num##_Handler

#define TIMER(num) _TIMER(num)
#define TIMER_IRQ(num) _TIMER_IRQ(num)
#define TIMER_VECTOR(num) _TIMER_VECTOR(num)
#define TIMER_VECTOR_NAME(num) _TIMER_VECTOR_NAME(num)

// Adds output signals to pins below to measure interrupt timings
// with a logic analyser. Set to 0 for production!
#define DEBUG_TIMING 0
#define DEBUG_ISR_STEPPER_PIN 60 // PD12
#define DEBUG_ISR_MOTION_PIN 61  // PD13
#define DEBUG_ISR_TEMP_PIN 10    // PD14/PWM1
#define DEBUG_ISR_ANALOG_PIN 63  // PD15/PWM2

//extern "C" void __cxa_pure_virtual() { }
extern "C" char* sbrk(int i);

char HAL::virtualEeprom[EEPROM_BYTES] = { 0 };
bool HAL::wdPinged = true;
uint8_t HAL::i2cError = 0;

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
    HardwareTimer* timer;
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
    { TIM1, 1, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM1_COUNT 2
#endif
#ifdef TIM2
#define TIM2_COUNT 1
    ,
    { TIM2, 2, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM2_COUNT 0
#endif
#ifdef TIM3
#define TIM3_COUNT 1
    ,
    { TIM3, 3, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM3_COUNT 0
#endif
#ifdef TIM4
#define TIM4_COUNT 1
    ,
    { TIM4, 4, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM4_COUNT 0
#endif
#ifdef TIM5
#define TIM5_COUNT 1
    ,
    { TIM5, 5, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM5_COUNT 0
#endif
#ifdef TIM6
#define TIM6_COUNT 1
    ,
    { TIM6, 6, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM6_COUNT 0
#endif
#ifdef TIM7
#define TIM7_COUNT 1
    ,
    { TIM7, 7, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM7_COUNT 0
#endif
#ifdef TIM8
#define TIM8_COUNT 1
    ,
    { TIM8, 8, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM8_COUNT 0
#endif
#ifdef TIM9
#define TIM9_COUNT 1
    ,
    { TIM9, 9, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM9_COUNT 0
#endif
#ifdef TIM10
#define TIM10_COUNT 1
    ,
    { TIM2, 10, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM10_COUNT 0
#endif
#ifdef TIM11
#define TIM11_COUNT 1
    ,
    { TIM11, 11, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM11_COUNT 0
#endif
#ifdef TIM12
#define TIM12_COUNT 1
    ,
    { TIM12, 12, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM12_COUNT 0
#endif
#ifdef TIM13
#define TIM13_COUNT 1
    ,
    { TIM13, 13, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM13_COUNT 0
#endif
#ifdef TIM14
#define TIM14_COUNT 1
    ,
    { TIM14, 14, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM14_COUNT 0
#endif
#ifdef TIM15
#define TIM15_COUNT 1
    ,
    { TIM15, 15, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM15_COUNT 0
#endif
#ifdef TIM16
#define TIM16_COUNT 1
    ,
    { TIM16, 16, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM16_COUNT 0
#endif
#ifdef TIM17
#define TIM17_COUNT 1
    ,
    { TIM17, 17, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM17_COUNT 0
#endif
#ifdef TIM18
#define TIM18_COUNT 1
    ,
    { TIM18, 18, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM18_COUNT 0
#endif
#ifdef TIM19
#define TIM19_COUNT 1
    ,
    { TIM19, 19, TimerUsage::UNUSED, 0, 0, nullptr }
#else
#define TIM19_COUNT 0
#endif
#ifdef TIM20
#define TIM20_COUNT 1
    ,
    { TIM20, 20, TimerUsage::UNUSED, 0, 0, nullptr }
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
TimerFunction* motion2;
TimerFunction* motion3;
TimerFunction* pwm;
TimerFunction* servo;
TimerFunction* toneTimer = nullptr;
extern void TIMER_VECTOR(MOTION2_TIMER_NUM);
extern void TIMER_VECTOR(MOTION3_TIMER_NUM);
extern void TIMER_VECTOR(PWM_TIMER_NUM);
extern void TIMER_VECTOR(TONE_TIMER_NUM);

#if NUM_SERVOS > 0 || NUM_BEEPERS > 0
extern void servoOffTimer(HardwareTimer*);
extern void TIMER_VECTOR(SERVO_TIMER_NUM);
static uint32_t ServoPrescalerfactor = 20000;
static uint32_t Servo2500 = 2500;
#endif

void HAL::hwSetup(void) {
#if DEBUG_TIMING
    SET_OUTPUT(DEBUG_ISR_STEPPER_PIN);
    SET_OUTPUT(DEBUG_ISR_MOTION_PIN);
    SET_OUTPUT(DEBUG_ISR_TEMP_PIN);
    SET_OUTPUT(DEBUG_ISR_ANALOG_PIN);
#endif
    // Servo control
#if NUM_SERVOS > 0 || NUM_BEEPERS > 0
    servo = reserveTimerInterrupt(SERVO_TIMER_NUM); // prevent pwm usage
    servo->timer = new HardwareTimer(TIMER(SERVO_TIMER_NUM));
    servo->timer->setPWM(1, NC, 200, 50, TIMER_VECTOR_NAME(SERVO_TIMER_NUM), servoOffTimer);
    ServoPrescalerfactor = servo->tim->PSC + 1;
    Servo2500 = ((2500 * (servo->timer->getTimerClkFreq() / 1000000)) / ServoPrescalerfactor) - 1;
    HAL_NVIC_SetPriority(TIMER_IRQ(SERVO_TIMER_NUM), 1, 0);
#endif

#if defined(TWI_CLOCK_FREQ) && TWI_CLOCK_FREQ > 0 //init i2c if we have a frequency
    HAL::i2cInit(TWI_CLOCK_FREQ);
#endif
    // make debugging startup easier
    //Serial.begin(115200);

#if EEPROM_AVAILABLE && EEPROM_MODE != EEPROM_NONE && EEPROM_AVAILABLE != EEPROM_SDCARD && EEPROM_AVAILABLE != EEPROM_FLASH
    // Copy eeprom to ram for faster access
    int i;
    for (i = 0; i < EEPROM_BYTES; i += 4) {
        eeval_t v = eprGetValue(i, 4);
        memcopy4(&virtualEeprom[i], &v.i);
    }
#else
    memset(virtualEeprom, 0, EEPROM_BYTES);
#endif
#if EEPROM_AVAILABLE == EEPROM_FLASH && EEPROM_MODE != EEPROM_NONE
    FEInit();
#endif
}

// Set up all timer interrupts
void HAL::setupTimer() {
    motion2 = reserveTimerInterrupt(MOTION2_TIMER_NUM); // prevent pwm usage
    motion2->timer = new HardwareTimer(TIMER(MOTION2_TIMER_NUM));
    motion2->timer->setMode(2, TIMER_OUTPUT_COMPARE);
    motion2->timer->setOverflow(PREPARE_FREQUENCY, HERTZ_FORMAT);
    motion2->timer->attachInterrupt(TIMER_VECTOR_NAME(MOTION2_TIMER_NUM));
    motion2->timer->resume();
    HAL_NVIC_SetPriority(TIMER_IRQ(MOTION2_TIMER_NUM), 2, 0);

    // Regular interrupts for heater control etc

    pwm = reserveTimerInterrupt(PWM_TIMER_NUM); // prevent pwm usage
    pwm->timer = new HardwareTimer(TIMER(PWM_TIMER_NUM));
    pwm->timer->setMode(2, TIMER_OUTPUT_COMPARE);
    pwm->timer->setOverflow(PWM_CLOCK_FREQ, HERTZ_FORMAT);
    pwm->timer->attachInterrupt(TIMER_VECTOR_NAME(PWM_TIMER_NUM));
    pwm->timer->resume();
    HAL_NVIC_SetPriority(TIMER_IRQ(PWM_TIMER_NUM), 2, 0);

    // Timer for stepper motor control

    motion3 = reserveTimerInterrupt(MOTION3_TIMER_NUM); // prevent pwm usage
    motion3->timer = new HardwareTimer(TIMER(MOTION3_TIMER_NUM));
    motion3->timer->setMode(2, TIMER_OUTPUT_COMPARE);
    motion3->timer->setOverflow(STEPPER_FREQUENCY, HERTZ_FORMAT);
    motion3->timer->attachInterrupt(TIMER_VECTOR_NAME(MOTION3_TIMER_NUM));
    motion3->timer->resume();
    HAL_NVIC_SetPriority(TIMER_IRQ(MOTION3_TIMER_NUM), 0, 0); // highest priority required!

#if NUM_BEEPERS > 0
    for (int i = 0; i < NUM_BEEPERS; i++) {
        if (beepers[i]->getOutputType() == 1) {
            // If we have any SW beepers, enable the beeper IRQ
            toneTimer = reserveTimerInterrupt(TONE_TIMER_NUM); // prevent pwm usage
            toneTimer->timer = new HardwareTimer(TIMER(TONE_TIMER_NUM));
            toneTimer->timer->setMode(2, TIMER_OUTPUT_COMPARE);
            toneTimer->timer->setOverflow(0, HERTZ_FORMAT);
            toneTimer->timer->attachInterrupt(TIMER_VECTOR_NAME(TONE_TIMER_NUM));
            break;
        }
    }
#endif
}

// Try to initialize pinNumber as hardware PWM. Returns internal
// id if it succeeds or -1 if it fails. Typical reasons to fail
// are no pwm support for that pin or an other pin uses same PWM
// channel.
int HAL::initHardwarePWM(int pinNumber, uint32_t frequency) {
    if (pinNumber < 0) {
        return -1;
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
            pinMode(pinNumber, OUTPUT);
            digitalWrite(pinNumber, LOW);
            HT->setPWM(channel, p, tf->frequency, 0);
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
    entry.ht->pause();
    entry.ht->setCaptureCompare(channel, value, RESOLUTION_8B_COMPARE_FORMAT); // set pwm 0
    entry.ht->resume();
}

void HAL::setHardwareFrequency(int id, uint32_t frequency) {
    if (id < 0 || id >= 50) { // illegal id
        return;
    }
    PWMEntry& entry = pwmEntries[id];
    entry.ht->pause();
    if (frequency) {
        entry.ht->setOverflow(frequency, HERTZ_FORMAT);
        entry.ht->resume();
    }
}

ADC_HandleTypeDef AdcHandle = {};
struct AnalogFunction {
    bool enabled;
    int32_t channel;
    int32_t lastValue;
    ADC_TypeDef* def;
    int32_t pin;
};

// Initialize ADC channels
static AnalogFunction analogValues[MAX_ANALOG_INPUTS] = { false, -1, 0, nullptr, -1 };
static AnalogFunction* analogMap[256] = { nullptr }; // Map pin number to entry in analogValues
int numAnalogInputs = 0;
uint32_t adcData[16] = { 0 }; // Target for DMA adc transfer

void reportAnalog() {
    for (int i = 0; i < 256; i++) {
        if (analogMap[i]) {
            Com::printF("Analog ", i);
            Com::printFLN(" = ", static_cast<int32_t>(analogMap[i]->lastValue));
        }
    }
    for (int i = 0; i < numAnalogInputs; i++) {
        Com::printF("adc ", i);
        Com::printF(" channel ", analogValues[i].channel);
        Com::printF(" pin ", analogValues[i].pin);
        Com::printFLN(", val = ", static_cast<int32_t>(adcData[i]));
    }
}

static DMA_HandleTypeDef hdma_adc;
int dmaInitState, dmaInitError;
int adcerror = 0, dmaerror = 0;

void HAL::reportHALDebug() {
    reportAnalog();
    Com::printFLN("AdcHandle state:", AdcHandle.State);
    Com::printFLN("AdcHandle ErrorCode:", AdcHandle.ErrorCode);
    Com::printFLN("hdma_adc state:", hdma_adc.State);
    Com::printFLN("hdma_adc ErrorCode:", hdma_adc.ErrorCode);
    Com::printFLN("dmaInitState state:", dmaInitState);
    Com::printFLN("dmaInitError ErrorCode:", dmaInitError);
    Com::printFLN("numAnalogInputs:", numAnalogInputs);
    Com::printFLN("numPWMEntries:", numPWMEntries);
    Com::printFLN("adcerror:", adcerror);
    Com::printFLN("dmaerror:", dmaerror);
}

//#define USE_DMA_ADC_IRQ_CALLBACK 1
// ^^^ Use the HAL_ADC_ConvCpltCallback interrupt function instead of reading the dma
// buffer inside our PWM interrupt.

// Problem with that callback is it's processing overhead.
// At the /slowest/ setting that function gets called at ~37.5khz
// Even with the simplest interrupt, just accessing it from flash
// slows us down. 

void HAL::analogStart(void) {
    // Analog channels being used are already enabled. Start conversion
    // only if we have any analog sources.
    if (numAnalogInputs == 0) {
        return;
    }
    ADC_ChannelConfTypeDef sConfig = { 0 };

    // All of the following needs more tuning, but works decently for now.
    AdcHandle.Instance = ADC1;
    AdcHandle.State = 0;
    AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    AdcHandle.Init.ContinuousConvMode = ENABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion = 0;
    AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.NbrOfConversion = numAnalogInputs;
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        adcerror++;
        return;
    }
#ifndef USE_DMA_ADC_IRQ_CALLBACK
    sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
#else
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
#endif
    //ADC_SAMPLETIME_239CYCLES_5 : ADC_SAMPLETIME_71CYCLES_5; ADC_SAMPLETIME_41CYCLES_5
    for (int i = 0; i < numAnalogInputs; i++) {
        sConfig.Channel = analogValues[i].channel;
        sConfig.Rank = i + 1;
        if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
            adcerror++;
            // Error_Handler();
        }
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    dmaerror += HAL_DMA_Init(&hdma_adc);

    dmaInitState = hdma_adc.State;
    dmaInitError = hdma_adc.ErrorCode;
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc);

#ifdef USE_DMA_ADC_IRQ_CALLBACK
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#endif

    // Todo: fix/check if adcdata's size is correct for word
    //                                           v          v
    dmaerror += HAL_ADC_Start_DMA(&AdcHandle, adcData, numAnalogInputs);
#ifndef USE_DMA_ADC_IRQ_CALLBACK
    __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_HT | DMA_IT_TE | DMA_IT_TC);
#endif
}

#ifdef USE_DMA_ADC_IRQ_CALLBACK
extern "C" void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc);
}
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 1);
#endif
    for (int i = 0; i < numAnalogInputs; i++) {
        analogValues[i].lastValue = adcData[i];
    }
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ANALOG_INPUT_LOOP
#include "io/redefine.h"
#ifdef DEBUG_TIMING
    WRITE(DEBUG_ISR_ANALOG_PIN, 0);
#endif
}
#endif

void HAL::analogEnable(int pinId) {
    PinName pin = static_cast<PinName>(pinId);
    AnalogFunction* af = &analogValues[numAnalogInputs++];
    af->def = static_cast<ADC_TypeDef*>(pinmap_peripheral(pin, PinMap_ADC));
    if (af->def == nullptr) {
        analogMap[pinId] = nullptr;
        --numAnalogInputs;
        return;
    }
    analogMap[pinId] = af;
    af->enabled = true;
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    // Set pin as analog input
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = STM_LL_GPIO_PIN(pin);
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // TODO: initaliziing PA0 here completely resets port A for some reason.
    // shuts off our PWM timers etc.
    HAL_GPIO_Init(get_GPIO_Port(STM_PORT(pin)), &GPIO_InitStruct);
    af->pin = pinId;
    af->channel = STM_PIN_CHANNEL(function);
}

int HAL::analogRead(int pin) {
    AnalogFunction* af = analogMap[pin];
    if (af == nullptr) { // protect for config errors
        return 0;
    }
    return af->lastValue;
}
// Write any data type to EEPROM
void HAL::eprBurnValue(unsigned int pos, int size, union eeval_t newvalue) {
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
#elif EEPROM_AVAILABLE == EEPROM_SDCARD || EEPROM_AVAILABLE == EEPROM_FLASH
    if (pos >= EEPROM::reservedEnd) {
        eprSyncTime = 1UL; // enforce fast write to finish before power is lost
    } else {
        eprSyncTime = HAL::timeInMilliseconds() | 1UL;
    }
#endif
}

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

        if (!failed && eepromFile.write(virtualEeprom, EEPROM_BYTES) != EEPROM_BYTES)
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
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        Com::printInfoFLN(PSTR("Low power reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        Com::printInfoFLN(PSTR("Windowed watchdog reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        Com::printInfoFLN(Com::tWatchdog);
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        Com::printInfoFLN(PSTR("Software reset"));
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        Com::printInfoFLN(Com::tPowerUp);
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        Com::printInfoFLN(PSTR("External reset pin reset"));
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
static unsigned int servoAutoOff[4] = { 0, 0, 0, 0 };
static uint8_t servoId = 0;
static ServoInterface* actServo = nullptr;

void HAL::servoMicroseconds(uint8_t servoId, int microsec, uint16_t autoOff) {
    servoTimings[servoId] = microsec ? ((microsec * (servo->timer->getTimerClkFreq() / 1000000)) / ServoPrescalerfactor) - 1 : 0;
    servoAutoOff[servoId] = (microsec) ? (autoOff / 20) : 0;
}
#endif

// ================== Interrupt handling ======================

ServoInterface* analogServoSlots[4] = { nullptr, nullptr, nullptr, nullptr };
void servoOffTimer(HardwareTimer* timer) {
#if NUM_SERVOS > 0
    if (actServo) {
        actServo->disable();
        if (servoAutoOff[servoId]) {
            servoAutoOff[servoId]--;
            if (servoAutoOff[servoId] == 0) {
                HAL::servoTimings[servoId] = 0;
            }
        }
    }
    if (++servoId >= 4) {
        servoId = 0;
    }
    actServo = analogServoSlots[servoId];
    if (actServo == nullptr) {
        servo->tim->CCR1 = Servo2500;
    } else {
        // InterruptProtectedBlock noInt;
        unsigned int ms = HAL::servoTimings[servoId];
        if (ms) {
            servo->tim->CCR1 = ms;
        } else {
            servo->tim->CCR1 = Servo2500;
        }
    }
#endif
// Add all generated servo interrupt handlers
#undef IO_TARGET
#define IO_TARGET IO_TARGET_SERVO_INTERRUPT
#include "io/redefine.h"
}

// Servo timer Interrupt handler
void TIMER_VECTOR(SERVO_TIMER_NUM) {
#if NUM_SERVOS > 0
    if (actServo) {
        actServo->enable();
    }
#endif
}

/** \brief Timer interrupt routine to drive the stepper motors.
*/
void TIMER_VECTOR(MOTION3_TIMER_NUM) {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_STEPPER_PIN, 1);
#endif
    Motion3::timer();
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_STEPPER_PIN, 0);
#endif
}

ufast8_t pwmSteps[] = { 1, 2, 4, 8, 16 };
ufast8_t pwmMasks[] = { 255, 254, 252, 248, 240 };

/**
This timer is called 10000 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/

void TIMER_VECTOR(PWM_TIMER_NUM) {
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

#ifndef USE_DMA_ADC_IRQ_CALLBACK
    if (__HAL_DMA_GET_FLAG(&hdma_adc, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc))) {
        for (int i = 0; i < numAnalogInputs; i++) {
            analogValues[i].lastValue = adcData[i];
        }
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ANALOG_INPUT_LOOP
#include "io/redefine.h"
        __HAL_DMA_CLEAR_FLAG(&hdma_adc, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc));
    }
#endif

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
void TIMER_VECTOR(MOTION2_TIMER_NUM) {
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 1);
#endif
    Motion2::timer();
#if DEBUG_TIMING
    WRITE(DEBUG_ISR_MOTION_PIN, 0);
#endif
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

#if NUM_BEEPERS > 0
void TIMER_VECTOR(TONE_TIMER_NUM) {
#undef IO_TARGET
#define IO_TARGET IO_TARGET_BEEPER_LOOP
#include "io/redefine.h"
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
            playingBeepers[i - NUM_BEEPERS]->setFreqDiv((multiFreq / beeperCurFreq) - 1);
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
    toneTimer->timer->pause();
    toneTimer->timer->setOverflow(2 * frequency, HERTZ_FORMAT);
    toneTimer->timer->resume();
#endif
}
void HAL::noTone() {
#if NUM_BEEPERS > 0
#if NUM_BEEPERS > 1
    // If any IO beeper is still playing, we can't stop the timer yet.
    for (size_t i = 0; i < NUM_BEEPERS; i++) {
        if (beepers[i]->getOutputType() == 1 && beepers[i]->isPlaying()) {
            HAL::tone(0);
            return;
        }
    }
#endif
    if (toneTimer != nullptr) { // could be called before timer are initialized!
        toneTimer->timer->pause();
    }
#endif
}

typedef void (*pFunction)(void);
void init(void) {
    hw_config_init();
}

void HAL::switchToBootMode() {
    // Todo: can use this to immediately load a new firmware version
    // using BTT's bootloader.
}

#endif
