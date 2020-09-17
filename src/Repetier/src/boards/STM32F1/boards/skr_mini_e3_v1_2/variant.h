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
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/


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


#define TIMER_SERIAL            TIM7
#define TIMER_SERIAL_RAW_IRQ    TIM7_IRQHandler
// This must be a literal
#define NUM_DIGITAL_PINS        51
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       16
#define NUM_ANALOG_FIRST        35 

// Override default Arduino configuration
// SPI Definitions
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5
#define PIN_SPI_SS              PA4

// I2C Definitions
#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

// UART Definitions
///#define PIN_SERIAL_TX PA_2
//#define DEBUG_UART_BAUDRATE 115200
//#define DEBUG_UART PA_2

#define PIN_SERIAL3_TX          PC10
#define PIN_SERIAL3_RX          NC

#define USB_ENABLE_PIN          PC13
#define USBD_ATTACH_PIN         PC13
#define USBD_ATTACH_LEVEL       LOW

// ignore this, has nothing to do with repetier
#define TIMER_TONE              TIM2

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/


#endif /* _VARIANT_ARDUINO_STM32_ */
