/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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
*/


#ifndef	FASTIO_H
#define	FASTIO_H

#include	<avr/io.h>

/*
	utility functions
*/

#ifndef		MASK
/// MASKING- returns \f$2^PIN\f$
	#define		MASK(PIN)				(1 << PIN)
#endif

/*
	magic I/O routines

	now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define		_READ(IO)			((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))

/// write to a pin
#define		_WRITE(IO, v)		do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

/// toggle a pin
#define		_TOGGLE(IO)			do {DIO ##  IO ## _RPORT = MASK(DIO ## IO ## _PIN); } while (0)

/// set pin as input
#define		_SET_INPUT(IO)		do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)

/// set pin as output
#define		_SET_OUTPUT(IO)		do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)

/// check if pin is an input
#define		_GET_INPUT(IO)		((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)

/// check if pin is an output
#define		_GET_OUTPUT(IO)		((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

//	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define		READ(IO)			_READ(IO)

/// Write to a pin wrapper
#define		WRITE(IO, v)		_WRITE(IO, v)
#define     PULLUP(IO,v)		_WRITE(IO, v)

/// toggle a pin wrapper
#define		TOGGLE(IO)			_TOGGLE(IO)

/// set pin as input wrapper
#define		SET_INPUT(IO)		_SET_INPUT(IO)

/// set pin as output wrapper
#define		SET_OUTPUT(IO)		_SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define		GET_INPUT(IO)		_GET_INPUT(IO)

/// check if pin is an output wrapper
#define		GET_OUTPUT(IO)		_GET_OUTPUT(IO)

/*
	ports and functions
	added as necessary or if I feel like it - not a comprehensive list!
*/


#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
// UART
#define	RXD					DIO0
#define	TXD					DIO1

// SPI
#define	SCK					52
#define	MISO				DIO50
#define	MOSI				DIO51
#define	SS					53

// TWI (I2C)
#define	SCL					DIO21
#define	SDA					DIO20

// timers and PWM
#define	OC0A				DIO13
#define	OC0B				DIO4
#define	OC1A				DIO11
#define	OC1B				DIO12
#define	OC2A				DIO10
#define	OC2B				DIO9
#define	OC3A				DIO5
#define	OC3B				DIO2
#define	OC3C				DIO3
#define	OC4A				DIO6
#define	OC4B				DIO7
#define	OC4C				DIO8
#define	OC5A				DIO46
#define	OC5B				DIO45
#define	OC5C				DIO44

#define	DEBUG_LED			DIO21

/*
pins
*/
#define	DIO0_PIN		PINE0
#define	DIO0_RPORT		PINE
#define	DIO0_WPORT		PORTE
#define	DIO0_DDR		DDRE
#define DIO0_PWM		NULL

#define	DIO1_PIN		PINE1
#define	DIO1_RPORT		PINE
#define	DIO1_WPORT		PORTE
#define	DIO1_DDR		DDRE
#define DIO1_PWM		NULL

#define	DIO2_PIN		PINE4
#define	DIO2_RPORT		PINE
#define	DIO2_WPORT		PORTE
#define	DIO2_DDR		DDRE
#define DIO2_PWM		&OCR3BL

#define	DIO3_PIN		PINE5
#define	DIO3_RPORT		PINE
#define	DIO3_WPORT		PORTE
#define	DIO3_DDR		DDRE
#define DIO3_PWM		&OCR3CL

#define	DIO4_PIN		PING5
#define	DIO4_RPORT		PING
#define	DIO4_WPORT		PORTG
#define	DIO4_DDR		DDRG
#define DIO4_PWM		&OCR0B

#define	DIO5_PIN		PINE3
#define	DIO5_RPORT		PINE
#define	DIO5_WPORT		PORTE
#define	DIO5_DDR		DDRE
#define DIO5_PWM		&OCR3AL

#define	DIO6_PIN		PINH3
#define	DIO6_RPORT		PINH
#define	DIO6_WPORT		PORTH
#define	DIO6_DDR		DDRH
#define DIO6_PWM		&OCR4AL

#define	DIO7_PIN		PINH4
#define	DIO7_RPORT		PINH
#define	DIO7_WPORT		PORTH
#define	DIO7_DDR		DDRH
#define DIO7_PWM		&OCR4BL

#define	DIO8_PIN		PINH5
#define	DIO8_RPORT		PINH
#define	DIO8_WPORT		PORTH
#define	DIO8_DDR		DDRH
#define DIO8_PWM		&OCR4CL

#define	DIO9_PIN		PINH6
#define	DIO9_RPORT		PINH
#define	DIO9_WPORT		PORTH
#define	DIO9_DDR		DDRH
#define DIO9_PWM		&OCR2B

#define	DIO10_PIN		PINB4
#define	DIO10_RPORT		PINB
#define	DIO10_WPORT		PORTB
#define	DIO10_DDR		DDRB
#define DIO10_PWM		&OCR2A

#define	DIO11_PIN		PINB5
#define	DIO11_RPORT		PINB
#define	DIO11_WPORT		PORTB
#define	DIO11_DDR		DDRB
#define DIO11_PWM		NULL

#define	DIO12_PIN		PINB6
#define	DIO12_RPORT		PINB
#define	DIO12_WPORT		PORTB
#define	DIO12_DDR		DDRB
#define DIO12_PWM		NULL

#define	DIO13_PIN		PINB7
#define	DIO13_RPORT		PINB
#define	DIO13_WPORT		PORTB
#define	DIO13_DDR		DDRB
#define DIO13_PWM		&OCR0A

#define	DIO14_PIN		PINJ1
#define	DIO14_RPORT		PINJ
#define	DIO14_WPORT		PORTJ
#define	DIO14_DDR		DDRJ
#define DIO14_PWM		NULL

#define	DIO15_PIN		PINJ0
#define	DIO15_RPORT		PINJ
#define	DIO15_WPORT		PORTJ
#define	DIO15_DDR		DDRJ
#define DIO15_PWM		NULL

#define	DIO16_PIN		PINH1
#define	DIO16_RPORT		PINH
#define	DIO16_WPORT		PORTH
#define	DIO16_DDR		DDRH
#define DIO16_PWM		NULL

#define	DIO17_PIN		PINH0
#define	DIO17_RPORT		PINH
#define	DIO17_WPORT		PORTH
#define	DIO17_DDR		DDRH
#define DIO17_PWM		NULL

#define	DIO18_PIN		PIND3
#define	DIO18_RPORT		PIND
#define	DIO18_WPORT		PORTD
#define	DIO18_DDR		DDRD
#define DIO18_PWM		NULL

#define	DIO19_PIN		PIND2
#define	DIO19_RPORT		PIND
#define	DIO19_WPORT		PORTD
#define	DIO19_DDR		DDRD
#define DIO19_PWM		NULL

#define	DIO20_PIN		PIND1
#define	DIO20_RPORT		PIND
#define	DIO20_WPORT		PORTD
#define	DIO20_DDR		DDRD
#define DIO20_PWM		NULL

#define	DIO21_PIN		PIND0
#define	DIO21_RPORT		PIND
#define	DIO21_WPORT		PORTD
#define	DIO21_DDR		DDRD
#define DIO21_PWM		NULL

#define	DIO22_PIN		PINA0
#define	DIO22_RPORT		PINA
#define	DIO22_WPORT		PORTA
#define	DIO22_DDR		DDRA
#define DIO22_PWM		NULL

#define	DIO23_PIN		PINA1
#define	DIO23_RPORT		PINA
#define	DIO23_WPORT		PORTA
#define	DIO23_DDR		DDRA
#define DIO23_PWM		NULL

#define	DIO24_PIN		PINA2
#define	DIO24_RPORT		PINA
#define	DIO24_WPORT		PORTA
#define	DIO24_DDR		DDRA
#define DIO24_PWM		NULL

#define	DIO25_PIN		PINA3
#define	DIO25_RPORT		PINA
#define	DIO25_WPORT		PORTA
#define	DIO25_DDR		DDRA
#define DIO25_PWM		NULL

#define	DIO26_PIN		PINA4
#define	DIO26_RPORT		PINA
#define	DIO26_WPORT		PORTA
#define	DIO26_DDR		DDRA
#define DIO26_PWM		NULL

#define	DIO27_PIN		PINA5
#define	DIO27_RPORT		PINA
#define	DIO27_WPORT		PORTA
#define	DIO27_DDR		DDRA
#define DIO27_PWM		NULL

#define	DIO28_PIN		PINA6
#define	DIO28_RPORT		PINA
#define	DIO28_WPORT		PORTA
#define	DIO28_DDR		DDRA
#define DIO28_PWM		NULL

#define	DIO29_PIN		PINA7
#define	DIO29_RPORT		PINA
#define	DIO29_WPORT		PORTA
#define	DIO29_DDR		DDRA
#define DIO29_PWM		NULL

#define	DIO30_PIN		PINC7
#define	DIO30_RPORT		PINC
#define	DIO30_WPORT		PORTC
#define	DIO30_DDR		DDRC
#define DIO30_PWM		NULL

#define	DIO31_PIN		PINC6
#define	DIO31_RPORT		PINC
#define	DIO31_WPORT		PORTC
#define	DIO31_DDR		DDRC
#define DIO31_PWM		NULL

#define	DIO32_PIN		PINC5
#define	DIO32_RPORT		PINC
#define	DIO32_WPORT		PORTC
#define	DIO32_DDR		DDRC
#define DIO32_PWM		NULL

#define	DIO33_PIN		PINC4
#define	DIO33_RPORT		PINC
#define	DIO33_WPORT		PORTC
#define	DIO33_DDR		DDRC
#define DIO33_PWM		NULL

#define	DIO34_PIN		PINC3
#define	DIO34_RPORT		PINC
#define	DIO34_WPORT		PORTC
#define	DIO34_DDR		DDRC
#define DIO34_PWM		NULL

#define	DIO35_PIN		PINC2
#define	DIO35_RPORT		PINC
#define	DIO35_WPORT		PORTC
#define	DIO35_DDR		DDRC
#define DIO35_PWM		NULL

#define	DIO36_PIN		PINC1
#define	DIO36_RPORT		PINC
#define	DIO36_WPORT		PORTC
#define	DIO36_DDR		DDRC
#define DIO36_PWM		NULL

#define	DIO37_PIN		PINC0
#define	DIO37_RPORT		PINC
#define	DIO37_WPORT		PORTC
#define	DIO37_DDR		DDRC
#define DIO37_PWM		NULL

#define	DIO38_PIN		PIND7
#define	DIO38_RPORT		PIND
#define	DIO38_WPORT		PORTD
#define	DIO38_DDR		DDRD
#define DIO38_PWM		NULL

#define	DIO39_PIN		PING2
#define	DIO39_RPORT		PING
#define	DIO39_WPORT		PORTG
#define	DIO39_DDR		DDRG
#define DIO39_PWM		NULL

#define	DIO40_PIN		PING1
#define	DIO40_RPORT		PING
#define	DIO40_WPORT		PORTG
#define	DIO40_DDR		DDRG
#define DIO40_PWM		NULL

#define	DIO41_PIN		PING0
#define	DIO41_RPORT		PING
#define	DIO41_WPORT		PORTG
#define	DIO41_DDR		DDRG
#define DIO41_PWM		NULL

#define	DIO42_PIN		PINL7
#define	DIO42_RPORT		PINL
#define	DIO42_WPORT		PORTL
#define	DIO42_DDR		DDRL
#define DIO42_PWM		NULL

#define	DIO43_PIN		PINL6
#define	DIO43_RPORT		PINL
#define	DIO43_WPORT		PORTL
#define	DIO43_DDR		DDRL
#define DIO43_PWM		NULL

#define	DIO44_PIN		PINL5
#define	DIO44_RPORT		PINL
#define	DIO44_WPORT		PORTL
#define	DIO44_DDR		DDRL
#define DIO44_PWM		&OCR5CL

#define	DIO45_PIN		PINL4
#define	DIO45_RPORT		PINL
#define	DIO45_WPORT		PORTL
#define	DIO45_DDR		DDRL
#define DIO45_PWM		&OCR5BL

#define	DIO46_PIN		PINL3
#define	DIO46_RPORT		PINL
#define	DIO46_WPORT		PORTL
#define	DIO46_DDR		DDRL
#define DIO46_PWM		&OCR5AL

#define	DIO47_PIN		PINL2
#define	DIO47_RPORT		PINL
#define	DIO47_WPORT		PORTL
#define	DIO47_DDR		DDRL
#define DIO47_PWM		NULL

#define	DIO48_PIN		PINL1
#define	DIO48_RPORT		PINL
#define	DIO48_WPORT		PORTL
#define	DIO48_DDR		DDRL
#define DIO48_PWM		NULL

#define	DIO49_PIN		PINL0
#define	DIO49_RPORT		PINL
#define	DIO49_WPORT		PORTL
#define	DIO49_DDR		DDRL
#define DIO49_PWM		NULL

#define	DIO50_PIN		PINB3
#define	DIO50_RPORT		PINB
#define	DIO50_WPORT		PORTB
#define	DIO50_DDR		DDRB
#define DIO50_PWM		NULL

#define	DIO51_PIN		PINB2
#define	DIO51_RPORT		PINB
#define	DIO51_WPORT		PORTB
#define	DIO51_DDR		DDRB
#define DIO51_PWM		NULL

#define	DIO52_PIN		PINB1
#define	DIO52_RPORT		PINB
#define	DIO52_WPORT		PORTB
#define	DIO52_DDR		DDRB
#define DIO52_PWM		NULL

#define	DIO53_PIN		PINB0
#define	DIO53_RPORT		PINB
#define	DIO53_WPORT		PORTB
#define	DIO53_DDR		DDRB
#define DIO53_PWM		NULL

#define DIO54_PIN		PINF0
#define DIO54_RPORT		PINF
#define DIO54_WPORT		PORTF
#define DIO54_DDR		DDRF
#define DIO54_PWM		NULL

#define DIO55_PIN		PINF1
#define DIO55_RPORT		PINF
#define DIO55_WPORT		PORTF
#define DIO55_DDR		DDRF
#define DIO55_PWM		NULL

#define DIO56_PIN		PINF2
#define DIO56_RPORT		PINF
#define DIO56_WPORT		PORTF
#define DIO56_DDR		DDRF
#define DIO56_PWM		NULL

#define DIO57_PIN		PINF3
#define DIO57_RPORT		PINF
#define DIO57_WPORT		PORTF
#define DIO57_DDR		DDRF
#define DIO57_PWM		NULL

#define DIO58_PIN		PINF4
#define DIO58_RPORT		PINF
#define DIO58_WPORT		PORTF
#define DIO58_DDR		DDRF
#define DIO58_PWM		NULL

#define DIO59_PIN		PINF5
#define DIO59_RPORT		PINF
#define DIO59_WPORT		PORTF
#define DIO59_DDR		DDRF
#define DIO59_PWM		NULL

#define DIO60_PIN		PINF6
#define DIO60_RPORT		PINF
#define DIO60_WPORT		PORTF
#define DIO60_DDR		DDRF
#define DIO60_PWM		NULL

#define DIO61_PIN		PINF7
#define DIO61_RPORT		PINF
#define DIO61_WPORT		PORTF
#define DIO61_DDR		DDRF
#define DIO61_PWM		NULL

#define DIO62_PIN		PINK0
#define DIO62_RPORT		PINK
#define DIO62_WPORT		PORTK
#define DIO62_DDR		DDRK
#define DIO62_PWM		NULL

#define DIO63_PIN		PINK1
#define DIO63_RPORT		PINK
#define DIO63_WPORT		PORTK
#define DIO63_DDR		DDRK
#define DIO63_PWM		NULL

#define DIO64_PIN		PINK2
#define DIO64_RPORT		PINK
#define DIO64_WPORT		PORTK
#define DIO64_DDR		DDRK
#define DIO64_PWM		NULL

#define DIO65_PIN		PINK3
#define DIO65_RPORT		PINK
#define DIO65_WPORT		PORTK
#define DIO65_DDR		DDRK
#define DIO65_PWM		NULL

#define DIO66_PIN		PINK4
#define DIO66_RPORT		PINK
#define DIO66_WPORT		PORTK
#define DIO66_DDR		DDRK
#define DIO66_PWM		NULL

#define DIO67_PIN		PINK5
#define DIO67_RPORT		PINK
#define DIO67_WPORT		PORTK
#define DIO67_DDR		DDRK
#define DIO67_PWM		NULL

#define DIO68_PIN		PINK6
#define DIO68_RPORT		PINK
#define DIO68_WPORT		PORTK
#define DIO68_DDR		DDRK
#define DIO68_PWM		NULL

#define DIO69_PIN		PINK7
#define DIO69_RPORT		PINK
#define DIO69_WPORT		PORTK
#define DIO69_DDR		DDRK
#define DIO69_PWM		NULL

#define DIO80_PIN		PINJ2
#define DIO80_RPORT		PINJ
#define DIO80_WPORT		PORTJ
#define DIO80_DDR		DDRJ
#define DIO80_PWM		NULL

#define DIO81_PIN		PINJ4
#define DIO81_RPORT		PINJ
#define DIO81_WPORT		PORTJ
#define DIO81_DDR		DDRJ
#define DIO81_PWM		NULL

#define DIO82_PIN		PINJ5
#define DIO82_RPORT		PINJ
#define DIO82_WPORT		PORTJ
#define DIO82_DDR		DDRJ
#define DIO82_PWM		NULL

#define DIO83_PIN		PINJ6
#define DIO83_RPORT		PINJ
#define DIO83_WPORT		PORTJ
#define DIO83_DDR		DDRJ
#define DIO83_PWM		NULL

#define DIO84_PIN		PINJ7
#define DIO84_RPORT		PINJ
#define DIO84_WPORT		PORTJ
#define DIO84_DDR		DDRJ
#define DIO84_PWM		NULL

#define DIO85_PIN		PINH7
#define DIO85_RPORT		PINH
#define DIO85_WPORT		PORTH
#define DIO85_DDR		DDRH
#define DIO85_PWM		NULL

#define DIO86_PIN		PINH2
#define DIO86_RPORT		PINH
#define DIO86_WPORT		PORTH
#define DIO86_DDR		DDRH
#define DIO86_PWM		NULL

#define DIO90_PIN		PINE7
#define DIO90_RPORT		PINE
#define DIO90_WPORT		PORTE
#define DIO90_DDR		DDRE
#define DIO90_PWM		NULL

#define DIO91_PIN		PINE2
#define DIO91_RPORT		PINE
#define DIO91_WPORT		PORTE
#define DIO91_DDR		DDRE
#define DIO91_PWM		NULL

#define DIO92_PIN		PIND4
#define DIO92_RPORT		PIND
#define DIO92_WPORT		PORTD
#define DIO92_DDR		DDRD
#define DIO92_PWM		NULL

#define DIO93_PIN		PIND5
#define DIO93_RPORT		PIND
#define DIO93_WPORT		PORTD
#define DIO93_DDR		DDRD
#define DIO93_PWM		NULL

#define DIO94_PIN		PIND6
#define DIO94_RPORT		PIND
#define DIO94_WPORT		PORTD
#define DIO94_DDR		DDRD
#define DIO94_PWM		NULL

#define DIO95_PIN		PING3
#define DIO95_RPORT		PING
#define DIO95_WPORT		PORTG
#define DIO95_DDR		DDRG
#define DIO95_PWM		NULL

#define DIO96_PIN		PING4
#define DIO96_RPORT		PING
#define DIO96_WPORT		PORTG
#define DIO96_DDR		DDRG
#define DIO96_PWM		NULL

#define DIO97_PIN		PINE6
#define DIO97_RPORT		PINE
#define DIO97_WPORT		PORTE
#define DIO97_DDR		DDRE
#define DIO97_PWM		NULL

#define DIO98_PIN		PINJ3
#define DIO98_RPORT		PINJ
#define DIO98_WPORT		PORTJ
#define DIO98_DDR		DDRJ
#define DIO98_PWM		NULL

#endif // defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)


#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please submit a pull request
#endif // DIO0_PIN


#endif // FASTIO_H

