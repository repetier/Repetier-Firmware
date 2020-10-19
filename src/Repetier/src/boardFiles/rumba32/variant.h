/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#define PA0   0  //D0
#define PA1   1  //D1
#define PA2   2  //D2
#define PA3   A0 //D3
#define PA4   A1 //D4
#define PA5   5  //D5
#define PA6   6  //D6
#define PA7   7  //D7
#define PA8   8  //D8
#define PA9   9  //D9
#define PA10  10 //D10
#define PA11  11 //D11
#define PA12  12 //D12
#define PA13  13 //D13
#define PA14  14 //D14
#define PA15  15 //D15
#define PB0   16 //D16
#define PB1   17 //D17
#define PB2   18 //D18
#define PB3   19 //D19
#define PB4   20 //D20
#define PB5   21 //D21
#define PB6   22 //D22
#define PB7   23 //D23
#define PB8   24 //D24
#define PB9   25 //D25
#define PB10  26 //D26
#define PB11  27 //D27
#define PB12  28 //D28
#define PB13  29 //D29
#define PB14  30 //D30
#define PB15  31 //D31
#define PC0   A2 //D32
#define PC1   A3 //D33
#define PC2   A4 //D34
#define PC3   A5 //D35
#define PC4   A6 //D36
#define PC5   37 //D37
#define PC6   38 //D38
#define PC7   39 //D39
#define PC8   40 //D40
#define PC9   41 //D41
#define PC10  42 //D42
#define PC11  43 //D43
#define PC12  44 //D44
#define PC13  45 //D45
#define PC14  46 //D46
#define PC15  47 //D47
#define PD0   48 //D48
#define PD1   49 //D49
#define PD2   50 //D50
#define PD3   51 //D51
#define PD4   52 //D52
#define PD5   53 //D53
#define PD6   54 //D54
#define PD7   55 //D55
#define PD8   56 //D56
#define PD9   57 //D57
#define PD10  58 //D58
#define PD11  59 //D59
#define PD12  60 //D60
#define PD13  61 //D61
#define PD14  62 //D62
#define PD15  63 //D63
#define PE0   64 //D64
#define PE1   65 //D65
#define PE2   66 //D66
#define PE3   67 //D67
#define PE4   68 //D68
#define PE5   69 //D69
#define PE6   70 //D70
#define PE7   71 //D71
#define PE8   72 //D72
#define PE9   73 //D73
#define PE10  74 //D74
#define PE11  75 //D75
#define PE12  76 //D76
#define PE13  77 //D77
#define PE14  78 //D78
#define PE15  79 //D79

#define CMSIS_STARTUP_FILE      "rumba32_startup_stm32f446ve.s"

#define NUM_DIGITAL_PINS        80
#define NUM_ANALOG_INPUTS       7

#define PIN_SPI_SS              PA4
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5


#define PIN_WIRE_SDA            PB9
#define PIN_WIRE_SCL            PB8

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#define TIMER_TONE              TIM6
#define TIMER_SERVO             TIM7

// UART Definitions
#define SERIAL_UART_INSTANCE    1 // Connected to EXP3 header


#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9


#define HSE_VALUE               12000000U

//#define DEBUG_UART_BAUDRATE 115200
//#define DEBUG_UART PA_2
 


// Important note: Forgetting to override/define a IRQHandler function
// and then trying to enable the IRQ will cause an infinite loop. 
// This is because the "default handler" in the CMSIS startup 
// assembly file for IRQ's IS an infinite loop.

// If you have a debugger, you can detect a missing IRQHandler
// by looking at the last call stack and seeing the "WWDG_IRQHandler"
// as the previous function call right before the infinite loop. 

// See \framework-arduinoststm32\cores\arduino\stm32\timer.h
// for more timer function name redefinitions!

#define RAW_TIM1_IRQn           RAW_TIM1_UP_TIM10_IRQn
#define RAW_TIM1_IRQHandler     RAW_TIM1_UP_TIM10_IRQHandler

#define RAW_TIM6_IRQn           RAW_TIM6_DAC_IRQn
#define RAW_TIM6_IRQHandler     RAW_TIM6_DAC_IRQHandler

#define RAW_TIM8_IRQn           RAW_TIM8_UP_TIM13_IRQn
#define RAW_TIM8_IRQHandler     RAW_TIM8_UP_TIM13_IRQHandler

#define RAW_TIM9_IRQn           RAW_TIM1_BRK_TIM9_IRQn
#define RAW_TIM9_IRQHandler     RAW_TIM1_BRK_TIM9_IRQHandler

#define RAW_TIM11_IRQn          RAW_TIM1_TRG_COM_TIM11_IRQn
#define RAW_TIM11_IRQHandler    RAW_TIM1_TRG_COM_TIM11_IRQHandler

#define RAW_TIM12_IRQn          RAW_TIM8_BRK_TIM12_IRQn
#define RAW_TIM12_IRQHandler    RAW_TIM8_BRK_TIM12_IRQHandler

#define RAW_TIM10_IRQn          RAW_TIM1_UP_TIM10_IRQn
#define RAW_TIM10_IRQHandler    RAW_TIM1_IRQHandler
//TIM10_IRQHandler is mapped on TIM1_IRQHandler!!

#define SERIAL_PORT_MONITOR       Serial
#define SERIAL_PORT_HARDWARE_OPEN Serial
#endif /* _VARIANT_ARDUINO_STM32_ */
