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


#ifndef UI_CONFIG_H
#define UI_CONFIG_H


/** While the ascii chars are all the same, the driver have different charsets
for special chars used in different countries. The charset allows to fix for
this problem. If characters look wrong, try a different charset. If nothing
works, use the ascii charset 0 as fallback. Not the nicest for everything but working!

0 = ASCII fallback
1 = Default works on most displays. This has some japanese chars in charset
2 = Alternative charset with more european chars

*/
#define UI_DISPLAY_CHARSET 2

/** Select type of beeper
0 = none
1 = Piezo connected to pin
2 = Piezo connected to a pin over I2C
*/
#ifndef BEEPER_TYPE
#define BEEPER_TYPE				1
#define BEEPER_TYPE_INVERTING	false
#endif // BEEPER_TYPE

#if BEEPER_TYPE==2
#define BEEPER_ADDRESS	0x40	// I2C address of the chip with the beeper pin
#define BEEPER_PIN		_BV(7)  // Bit value for pin 8
#define COMPILE_I2C_DRIVER		// We need the I2C driver as we are using i2c
#endif // BEEPER_TYPE==2


/* What type of chip is used for I2C communication
0 : PCF8574 or PCF8574A or compatible chips.
1 : MCP23017
*/
#define UI_DISPLAY_I2C_CHIPTYPE				0

// 0x40 till 0x4e for PCF8574, 0x40 for the adafruid RGB shield, 0x40 - 0x4e for MCP23017
// Official addresses have a value half as high!
#define UI_DISPLAY_I2C_ADDRESS				0x4e

// For MCP 23017 define which pins should be output
#define UI_DISPLAY_I2C_OUTPUT_PINS			65504

// Set the output mask that is or'd over the output data. This is needed to activate
// a backlight switched over the I2C.
// The adafruit RGB shields enables a light if the bit is not set. Bits 6-8 are used for backlight.
#define UI_DISPLAY_I2C_OUTPUT_START_MASK	0

// For MCP which inputs are with pullup. 31 = pins 0-4 for adafruid rgb shield buttons
#define UI_DISPLAY_I2C_PULLUP				31

/* How fast should the I2C clock go. The PCF8574 work only with the lowest setting 100000.
A MCP23017 can run also with 400000 Hz */
#define UI_I2C_CLOCKSPEED					100000L


/** Uncomment this, if you have keys connected via i2c to a PCF8574 chip. */
//#define UI_HAS_I2C_KEYS

// Do you have a I2C connected encoder?
#define UI_HAS_I2C_ENCODER					0

// Under which address can the key status requested. This is the address of your PCF8574 where the keys are connected.
// If you use a MCP23017 the address from display is used also for keys.
#define UI_I2C_KEY_ADDRESS					0x40

#endif // UI_CONFIG_H