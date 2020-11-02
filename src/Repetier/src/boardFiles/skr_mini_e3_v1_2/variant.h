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

#define PA0   0  //D0/A0
#define PA1   1  //D1/A1
#define PA2   2  //D2/A2
#define PA3   3  //D3/A3
#define PA4   4  //D4/A4
#define PA5   5  //D5/A5
#define PA6   6  //D6/A6
#define PA7   7  //D7/A7
#define PA8   8  //D8
#define PA9   9  //D9
#define PA10  10 //D10
#define PA11  11 //D11
#define PA12  12 //D12
#define PA13  13 //D13
#define PA14  14 //D14
#define PA15  15 //D15
#define PB0   16 //D16/A8
#define PB1   17 //D17/A9
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
#define PC0   32 //D32/A10
#define PC1   33 //D33/A11
#define PC2   34 //D34/A12
#define PC3   35 //D35/A13
#define PC4   36 //D36/A14
#define PC5   37 //D37/A15
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

#define CMSIS_STARTUP_FILE      "skr_startup_stm32f103xe.s"
#define VECT_TAB_OFFSET         0x7000 // vector table offset due to bootloader

#define TIMER_SERIAL            TIM7
#define TIMER_SERIAL_RAW_IRQ    RAW_TIM7_IRQHandler 

#define NUM_DIGITAL_PINS        51
#define NUM_ANALOG_INPUTS       16

#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5
#define PIN_SPI_SS              PA4

#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

#ifndef EXTI_IRQ_PRIO
#define EXTI_IRQ_PRIO 1 // Endstops
#endif

// Always have the TFT serial2 port defined for config.h changes and compiles. 
#define HAL_USART_MODULE_ENABLED
#define HAVE_HWSERIAL2

#define UART_IRQ_PRIO           3

//#define DEBUG_UART_BAUDRATE 115200
//#define DEBUG_UART PA_2

#define USBD_ATTACH_PIN         PC13
#define USBD_ATTACH_LEVEL       LOW

#define RAW_TIM8_IRQn           RAW_TIM8_UP_IRQn
#define RAW_TIM8_IRQHandler     RAW_TIM8_UP_IRQHandler

#define RAW_TIM1_IRQn           RAW_TIM1_UP_IRQn
#define RAW_TIM1_IRQHandler     RAW_TIM1_UP_IRQHandler
 
#define SYS_WKUP1               PA_0

#endif /* _VARIANT_ARDUINO_STM32_ */
