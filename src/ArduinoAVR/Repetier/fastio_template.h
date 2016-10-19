/*
 * Some extensions to allow usage of fastio pins in C** Templates
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

*/
#ifndef _FASTIO_TEMPPLATE_h
#define _FASTIO_TEMPPLATE_h

#include "fastio.h"

template<uint8_t pin> struct DioPin {
};
// You must use arduino AVR board definitions and tools 1.6.12 or higher (gcc-4.9.x+)
// versions before are not able to resolve port addresses as constant expressions
#define _define_pin(PIN) \
template <> struct DioPin<PIN> { \
	static volatile uint8_t * const wport=&DIO ## PIN ## _WPORT;\
	static volatile uint8_t * const rport=&DIO ## PIN ## _RPORT;\
	static volatile uint8_t * const ddr=&DIO ## PIN ## _DDR;\
	static uint8_t const pin=DIO ## PIN ## _PIN;\
	static uint8_t const arduinoPin=PIN;\
	\
	static void setDirection(bool out) {if(out) *ddr |= _BV(pin); else *ddr &= ~_BV(pin);} \
	static void setValue(bool v) {if(v) *wport |= _BV(pin); else *wport &= ~_BV(pin);} \
	static bool getDirection() {return (*ddr & _BV(pin)) != 0;} \
	static bool getOutputState() {return (*wport & _BV(pin)) != 0;} \
	static bool getValue() {return (*rport & _BV(pin)) != 0;} \
};

// dummy pin
template<> struct DioPin<0xff> {
	static int const pin=0xff;
	static void setDirection(bool out) {
		(void) out;
	}
	static void setValue(bool v) {
		(void) v;
	}
	static bool getValue() {
		return false;
	}
};

typedef DioPin<0xff> DioDummyPin;

// generated with python one-liner:
// for i in xrange(0,100): print '#ifdef DIO%(i)d_PIN\n_define_pin(%(i)d)\n#endif'%{"i":i}

#ifdef DIO0_PIN
_define_pin(0)
#endif
#ifdef DIO1_PIN
_define_pin(1)
#endif
#ifdef DIO2_PIN
_define_pin(2)
#endif
#ifdef DIO3_PIN
_define_pin(3)
#endif
#ifdef DIO4_PIN
_define_pin(4)
#endif
#ifdef DIO5_PIN
_define_pin(5)
#endif
#ifdef DIO6_PIN
_define_pin(6)
#endif
#ifdef DIO7_PIN
_define_pin(7)
#endif
#ifdef DIO8_PIN
_define_pin(8)
#endif
#ifdef DIO9_PIN
_define_pin(9)
#endif
#ifdef DIO10_PIN
_define_pin(10)
#endif
#ifdef DIO11_PIN
_define_pin(11)
#endif
#ifdef DIO12_PIN
_define_pin(12)
#endif
#ifdef DIO13_PIN
_define_pin(13)
#endif
#ifdef DIO14_PIN
_define_pin(14)
#endif
#ifdef DIO15_PIN
_define_pin(15)
#endif
#ifdef DIO16_PIN
_define_pin(16)
#endif
#ifdef DIO17_PIN
_define_pin(17)
#endif
#ifdef DIO18_PIN
_define_pin(18)
#endif
#ifdef DIO19_PIN
_define_pin(19)
#endif
#ifdef DIO20_PIN
_define_pin(20)
#endif
#ifdef DIO21_PIN
_define_pin(21)
#endif
#ifdef DIO22_PIN
_define_pin(22)
#endif
#ifdef DIO23_PIN
_define_pin(23)
#endif
#ifdef DIO24_PIN
_define_pin(24)
#endif
#ifdef DIO25_PIN
_define_pin(25)
#endif
#ifdef DIO26_PIN
_define_pin(26)
#endif
#ifdef DIO27_PIN
_define_pin(27)
#endif
#ifdef DIO28_PIN
_define_pin(28)
#endif
#ifdef DIO29_PIN
_define_pin(29)
#endif
#ifdef DIO30_PIN
_define_pin(30)
#endif
#ifdef DIO31_PIN
_define_pin(31)
#endif
#ifdef DIO32_PIN
_define_pin(32)
#endif
#ifdef DIO33_PIN
_define_pin(33)
#endif
#ifdef DIO34_PIN
_define_pin(34)
#endif
#ifdef DIO35_PIN
_define_pin(35)
#endif
#ifdef DIO36_PIN
_define_pin(36)
#endif
#ifdef DIO37_PIN
_define_pin(37)
#endif
#ifdef DIO38_PIN
_define_pin(38)
#endif
#ifdef DIO39_PIN
_define_pin(39)
#endif
#ifdef DIO40_PIN
_define_pin(40)
#endif
#ifdef DIO41_PIN
_define_pin(41)
#endif
#ifdef DIO42_PIN
_define_pin(42)
#endif
#ifdef DIO43_PIN
_define_pin(43)
#endif
#ifdef DIO44_PIN
_define_pin(44)
#endif
#ifdef DIO45_PIN
_define_pin(45)
#endif
#ifdef DIO46_PIN
_define_pin(46)
#endif
#ifdef DIO47_PIN
_define_pin(47)
#endif
#ifdef DIO48_PIN
_define_pin(48)
#endif
#ifdef DIO49_PIN
_define_pin(49)
#endif
#ifdef DIO50_PIN
_define_pin(50)
#endif
#ifdef DIO51_PIN
_define_pin(51)
#endif
#ifdef DIO52_PIN
_define_pin(52)
#endif
#ifdef DIO53_PIN
_define_pin(53)
#endif
#ifdef DIO54_PIN
_define_pin(54)
#endif
#ifdef DIO55_PIN
_define_pin(55)
#endif
#ifdef DIO56_PIN
_define_pin(56)
#endif
#ifdef DIO57_PIN
_define_pin(57)
#endif
#ifdef DIO58_PIN
_define_pin(58)
#endif
#ifdef DIO59_PIN
_define_pin(59)
#endif
#ifdef DIO60_PIN
_define_pin(60)
#endif
#ifdef DIO61_PIN
_define_pin(61)
#endif
#ifdef DIO62_PIN
_define_pin(62)
#endif
#ifdef DIO63_PIN
_define_pin(63)
#endif
#ifdef DIO64_PIN
_define_pin(64)
#endif
#ifdef DIO65_PIN
_define_pin(65)
#endif
#ifdef DIO66_PIN
_define_pin(66)
#endif
#ifdef DIO67_PIN
_define_pin(67)
#endif
#ifdef DIO68_PIN
_define_pin(68)
#endif
#ifdef DIO69_PIN
_define_pin(69)
#endif
#ifdef DIO70_PIN
_define_pin(70)
#endif
#ifdef DIO71_PIN
_define_pin(71)
#endif
#ifdef DIO72_PIN
_define_pin(72)
#endif
#ifdef DIO73_PIN
_define_pin(73)
#endif
#ifdef DIO74_PIN
_define_pin(74)
#endif
#ifdef DIO75_PIN
_define_pin(75)
#endif
#ifdef DIO76_PIN
_define_pin(76)
#endif
#ifdef DIO77_PIN
_define_pin(77)
#endif
#ifdef DIO78_PIN
_define_pin(78)
#endif
#ifdef DIO79_PIN
_define_pin(79)
#endif
#ifdef DIO80_PIN
_define_pin(80)
#endif
#ifdef DIO81_PIN
_define_pin(81)
#endif
#ifdef DIO82_PIN
_define_pin(82)
#endif
#ifdef DIO83_PIN
_define_pin(83)
#endif
#ifdef DIO84_PIN
_define_pin(84)
#endif
#ifdef DIO85_PIN
_define_pin(85)
#endif
#ifdef DIO86_PIN
_define_pin(86)
#endif
#ifdef DIO87_PIN
_define_pin(87)
#endif
#ifdef DIO88_PIN
_define_pin(88)
#endif
#ifdef DIO89_PIN
_define_pin(89)
#endif
#ifdef DIO90_PIN
_define_pin(90)
#endif
#ifdef DIO91_PIN
_define_pin(91)
#endif
#ifdef DIO92_PIN
_define_pin(92)
#endif
#ifdef DIO93_PIN
_define_pin(93)
#endif
#ifdef DIO94_PIN
_define_pin(94)
#endif
#ifdef DIO95_PIN
_define_pin(95)
#endif
#ifdef DIO96_PIN
_define_pin(96)
#endif
#ifdef DIO97_PIN
_define_pin(97)
#endif
#ifdef DIO98_PIN
_define_pin(98)
#endif
#ifdef DIO99_PIN
_define_pin(99)
#endif

#undef _define_pin
#endif
