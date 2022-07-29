/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#define PA0 0    //D0
#define PA1 1    //D1
#define PA2 2    //D2
#define PA3 3    //D3
#define PA4 4    //D4
#define PA5 5    //D5
#define PA6 6    //D6
#define PA7 7    //D7
#define PA8 8    //D8
#define PA9 9    //D9
#define PA10 10  //D10
#define PA11 11  //D11
#define PA12 12  //D12
#define PA13 13  //D13
#define PA14 14  //D14
#define PA15 15  //D15
#define PB0 16   //D16
#define PB1 17   //D17
#define PB2 18   //D18
#define PB3 19   //D19
#define PB4 20   //D20
#define PB5 21   //D21
#define PB6 22   //D22
#define PB7 23   //D23
#define PB8 24   //D24
#define PB9 25   //D25
#define PB10 26  //D26
#define PB11 27  //D27
#define PB12 28  //D28
#define PB13 29  //D29
#define PB14 30  //D30
#define PB15 31  //D31
#define PC0 32   //D32
#define PC1 33   //D33
#define PC2 34   //D34
#define PC3 35   //D35
#define PC4 36   //D36
#define PC5 37   //D37
#define PC6 38   //D38
#define PC7 39   //D39
#define PC8 40   //D40
#define PC9 41   //D41
#define PC10 42  //D42
#define PC11 43  //D43
#define PC12 44  //D44
#define PC13 45  //D45
#define PC14 46  //D46
#define PC15 47  //D47
#define PD0 48   //D48
#define PD1 49   //D49
#define PD2 50   //D50
#define PD3 51   //D51
#define PD4 52   //D52
#define PD5 53   //D53
#define PD6 54   //D54
#define PD7 55   //D55
#define PD8 56   //D56
#define PD9 57   //D57
#define PD10 58  //D58
#define PD11 59  //D59
#define PD12 60  //D60
#define PD13 61  //D61
#define PD14 62  //D62
#define PD15 63  //D63
#define PE0 64   //D64
#define PE1 65   //D65
#define PE2 66   //D66
#define PE3 67   //D67
#define PE4 68   //D68
#define PE5 69   //D69
#define PE6 70   //D70
#define PE7 71   //D71
#define PE8 72   //D72
#define PE9 73   //D73
#define PE10 74  //D74
#define PE11 75  //D75
#define PE12 76  //D76
#define PE13 77  //D77
#define PE14 78  //D78
#define PE15 79  //D79
#define PF0 80   //D80
#define PF1 81   //D81
#define PF2 82   //D82
#define PF3 83   //D83
#define PF4 84   //D84
#define PF5 85   //D85
#define PF6 86   //D86
#define PF7 87   //D87
#define PF8 88   //D88
#define PF9 89   //D89
#define PF10 90  //D90
#define PF11 91  //D91
#define PF12 92  //D92
#define PF13 93  //D93
#define PF14 94  //D94
#define PF15 95  //D95
#define PG0 96   //D96
#define PG1 97   //D97
#define PG2 98   //D98
#define PG3 99   //D99
#define PG4 100  //D100
#define PG5 101  //D101
#define PG6 102  //D102
#define PG7 103  //D103
#define PG8 104  //D104
#define PG9 105  //D105
#define PG10 106 //D106
#define PG11 107 //D107
#define PG12 108 //D108
#define PG13 109 //D109
#define PG14 110 //D110
#define PG15 111 //D111

#define NUM_DIGITAL_PINS 112
#define NUM_ANALOG_INPUTS 6

#define NUM_ANALOG_FIRST 112

// SPI Definitions
#define PIN_SPI_SS PA4
#define PIN_SPI_MOSI PA7
#define PIN_SPI_MISO PA6
#define PIN_SPI_SCK PA5

// Below SPI and I2C definitions already done in the core
// Could be redefined here if differs from the default one
// SPI Definitions
/*#define PIN_SPI_MOSI PB15
#define PIN_SPI_MISO PB14
#define PIN_SPI_SCK PB13
#define PIN_SPI_SS PB12 */
//#define PIN_SPI_SS1             PB0 // W25Q16 (on board flash)

// I2C Definitions
#define PIN_WIRE_SDA PB9
#define PIN_WIRE_SCL PB8

// Select timer to prevent conflict
#define TIMER_SERIAL TIM11

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE 1 //ex: 2 for Serial2 (USART2)
// DEBUG_UART could be redefined to print on another instance than 'Serial'
//#define DEBUG_UART              ((USART_TypeDef *) U(S)ARTX) // ex: USART3
// DEBUG_UART baudrate, default: 9600 if not defined
//#define DEBUG_UART_BAUDRATE     x
// DEBUG_UART Tx pin name, default: the first one found in PinMap_UART_TX for DEBUG_UART
//#define DEBUG_PINNAME_TX        PX_n // PinName used for TX

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX PA10
#define PIN_SERIAL_TX PA9

/* Extra HAL modules */
#define HAL_DAC_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR Serial
#define SERIAL_PORT_HARDWARE Serial1
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
