/*
	This code contibuted by Triffid_Hunter and modified by Kliment
	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
        2012/3/10 AT90USB128x modified by lincomatic to match Teensyduino
		2019/5/2 SAMD51 Adafruit Grand Central Board by RAyWB/Robert Ayrenschmalz


	Pin mappings for analog pins A0-A15 to digital pin differ from due
	Analog Pin   Due Digital   MAGC Digital
	0            54            67
	1            55            68
	2            56            69
	3            57            70
	4            58            71
	5            59	           72
	6            60            73
	7            61            74
	8            62            54
	9            63            55
	10           64            56
	11           65            57
	12           66            58
	13           67            59
	14           68            60
	15           69            61

*/
#ifndef _FASTIO_H
#define _FASTIO_H

#include <samd.h>

#include <samd51/include/samd51.h>
//#include <samd51/include/pio/samd51p20a.h>

#if (MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_NO_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_RADDS)

#define ANALOG_PIN_TO_CHANNEL(p) (p < 62 ? p - 46 : p - 67)
#define ANALOG_TO_DIGITAL_PIN(p) ((p < 8) ? 67 + (p) : (p < 16) ? 54 + (p)-8 : (p < 18) ? 12 + (p)-16 : (p == 18) ? 9 : -1)

// 0 .. 53 - Digital pins
// ----------------------
// 0/1 - UART (Serial)
#define DIO0_PORT PORTB
#define DIO0_PIN PORT_PB25

#define DIO1_PORT PORTB
#define DIO1_PIN PORT_PB24

// 2
#define DIO2_PORT PORTC
#define DIO2_PIN PORT_PC18
#define DIO3_PORT PORTC
#define DIO3_PIN PORT_PC19
#define DIO4_PORT PORTC
#define DIO4_PIN PORT_PC20

// 5
#define DIO5_PORT PORTC
#define DIO5_PIN PORT_PC21
#define DIO6_PORT PORTD
#define DIO6_PIN PORT_PD20
#define DIO7_PORT PORTD
#define DIO7_PIN PORT_PD21
#define DIO8_PORT PORTB
#define DIO8_PIN PORT_PB18
#define DIO9_PORT PORTB
#define DIO9_PIN PORT_PB02
// 10
#define DIO10_PORT PORTB
#define DIO10_PIN PORT_PB22
#define DIO11_PORT PORTB
#define DIO11_PIN PORT_PB23
#define DIO12_PORT PORTB
#define DIO12_PIN PORT_PB00

// 13 - AMBER LED
#define DIO13_PORT PORTB
#define DIO13_PIN PORT_PB01

// 14/15 -(Serial3)
#define DIO14_PORT PORTB
#define DIO14_PIN PORT_PB16
#define DIO15_PORT PORTB
#define DIO15_PIN PORT_PB17

// 16/17 - (Serial2)
#define DIO16_PORT PORTC
#define DIO16_PIN PORT_PC22
#define DIO17_PORT PORTC
#define DIO17_PIN PORT_PC23

// 18/19 - (Serial1)
#define DIO18_PORT PORTB
#define DIO18_PIN PORT_PB12
#define DIO19_PORT PORTB
#define DIO19_PIN PORT_PB13

// 20/21  - (I2C)
#define DIO20_PORT PORTB
#define DIO20_PIN PORT_PB20
#define DIO21_PORT PORTB
#define DIO21_PIN PORT_PB21

// 22
#define DIO22_PORT PORTD
#define DIO22_PIN PORT_PD12
#define DIO23_PORT PORTA
#define DIO23_PIN PORT_PA15
#define DIO24_PORT PORTC
#define DIO24_PIN PORT_PC17
#define DIO25_PORT PORTC
#define DIO25_PIN PORT_PC16

// 26
#define DIO26_PORT PORTA
#define DIO26_PIN PORT_PA12
#define DIO27_PORT PORTA
#define DIO27_PIN PORT_PA13
#define DIO28_PORT PORTA
#define DIO28_PIN PORT_PA14
#define DIO29_PORT PORTB
#define DIO29_PIN PORT_PB19

// 30
#define DIO30_PORT PORTA
#define DIO30_PIN PORT_PA23
#define DIO31_PORT PORTA
#define DIO31_PIN PORT_PA22
#define DIO32_PORT PORTA
#define DIO32_PIN PORT_PA21
#define DIO33_PORT PORTA
#define DIO33_PIN PORT_PA20

// 34
#define DIO34_PORT PORTA
#define DIO34_PIN PORT_PA19
#define DIO35_PORT PORTA
#define DIO35_PIN PORT_PA18
#define DIO36_PORT PORTA
#define DIO36_PIN PORT_PA17
#define DIO37_PORT PORTA
#define DIO37_PIN PORT_PA16
// 38
#define DIO38_PORT PORTB
#define DIO38_PIN PORT_PB15
#define DIO39_PORT PORTB
#define DIO39_PIN PORT_PB14
#define DIO40_PORT PORTC
#define DIO40_PIN PORT_PC13
#define DIO41_PORT PORTC
#define DIO41_PIN PORT_PC12
// 42
#define DIO42_PORT PORTC
#define DIO42_PIN PORT_PC15
#define DIO43_PORT PORTC
#define DIO43_PIN PORT_PC14
#define DIO44_PORT PORTC
#define DIO44_PIN PORT_PC11
#define DIO45_PORT PORTC
#define DIO45_PIN PORT_PC10

// 46
#define DIO46_PORT PORTC
#define DIO46_PIN PORT_PC06
#define DIO47_PORT PORTC
#define DIO47_PIN PORT_PC07
#define DIO48_PORT PORTC
#define DIO48_PIN PORT_PC04
#define DIO49_PORT PORTC
#define DIO49_PIN PORT_PC05

// 50
#define DIO50_PORT PORTD
#define DIO50_PIN PORT_PD11
#define DIO51_PORT PORTD
#define DIO51_PIN PORT_PD08
#define DIO52_PORT PORTD
#define DIO52_PIN PORT_PD09
#define DIO53_PORT PORTD
#define DIO53_PIN PORT_PD10

// 54 .. 65 - Analog pins
// ----------------------
#define DIO54_PORT PORTB
#define DIO54_PIN PORT_PB05
#define DIO55_PORT PORTB
#define DIO55_PIN PORT_PB06
#define DIO56_PORT PORTB
#define DIO56_PIN PORT_PB07
#define DIO57_PORT PORTB
#define DIO57_PIN PORT_PB08
// 58
#define DIO58_PORT PORTB
#define DIO58_PIN PORT_PB09
#define DIO59_PORT PORTA
#define DIO59_PIN PORT_PA04
#define DIO60_PORT PORTA
#define DIO60_PIN PORT_PA06
#define DIO61_PORT PORTA
#define DIO61_PIN PORT_PA07
// 62
#define DIO62_PORT PORTB
#define DIO62_PIN PORT_PB20
#define DIO63_PORT PORTB
#define DIO63_PIN PORT_PB21
#define DIO64_PORT PORTD
#define DIO64_PIN PORT_PD11
#define DIO65_PORT PORTD
#define DIO65_PIN PORT_PD08

// 66-76
#define DIO66_PORT PORTD
#define DIO66_PIN PORT_PD09
#define DIO67_PORT PORTA
#define DIO67_PIN PORT_PA02
#define DIO68_PORT PORTA
#define DIO68_PIN PORT_PA05
#define DIO69_PORT PORTB
#define DIO69_PIN PORT_PB03
#define DIO70_PORT PORTC
#define DIO70_PIN PORT_PC00
#define DIO71_PORT PORTC
#define DIO71_PIN PORT_PC01
#define DIO72_PORT PORTC
#define DIO72_PIN PORT_PC02
#define DIO73_PORT PORTC
#define DIO73_PIN PORT_PC03
#define DIO74_PORT PORTB
#define DIO74_PIN PORT_PB04
#define DIO75_PORT PORTC
#define DIO75_PIN PORT_PC31
#define DIO76_PORT PORTC
#define DIO76_PIN PORT_PC30

// 77 - 79 USB
#define DIO77_PORT PORTA
#define DIO77_PIN PORT_PA27
#define DIO78_PORT PORTA
#define DIO78_PIN PORT_PA24
#define DIO79_PORT PORTA
#define DIO79_PIN PORT_PA25

// 80 - 83 ONBOARD SD PINS
#define DI80_PORT PORTB
#define DIO80_PIN PORT_PB29
#define DIO81_PORT PORTB
#define DIO81_PIN PORT_PB27
#define DIO82_PORT PORTB
#define DIO82_PIN PORT_PB26
#define DIO83_PORT PORTB
#define DIO83_PIN PORT_PB28

// 84 - AREF
#define DIO84_PORT PORTA
#define DIO84_PIN PORT_PA03

// 85 - DAC0
#define DIO85_PORT PORTA
#define DIO85_PIN PORT_PA02

// 86 - DAC1
#define DIO86_PORT PORTA
#define DIO86_PIN PORT_PA05

// 87 - LED#13
#define DIO87_PORT PORTB
#define DIO87_PIN PORT_PB01

// 88 - NEOPIXEL
#define DIO88_PORT PORTC
#define DIO88_PIN PORT_PC24

// 89-94 QSPI
#define DIO89_PORT PORTB
#define DIO89_PIN PORT_PB10
#define DIO90_PORT PORTB
#define DIO90_PIN PORT_PB11
#define DIO91_PORT PORTA
#define DIO91_PIN PORT_PA08
#define DIO92_PORT PORTA
#define DIO92_PIN PORT_PA09
#define DIO93_PORT PORTA
#define DIO93_PIN PORT_PA10
#define DIO94_PORT PORTA
#define DIO94_PIN PORT_PA11

// onboard SD detect
#define DIO95_PORT PORTB
#define DIO95_PIN PORT_PB31

#endif // (MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_NO_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_SMARTRAMPS_EEPROM || MOTHERBOARD == MOTHERBOARD_AGC_RADDS)

#ifndef DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please submit a pull request
#endif

#endif /* _FASTIO_H */
