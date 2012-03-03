/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

/* ===================== IMPORTANT ========================

The LCD and Key support is in development. It doesn't work yet so don't
try to enable it. This can only cause problems.

==============================================================*/

/**
Select the language to use.
0 = english
1 = german
*/
#define UI_LANGUAGE 1

/**
What display type do you use?
0 = No display
1 = LCD Display with 4 bit data bus
2 = LCD Display with 8 bit data bus
3 = LCD Display with I2C connection, 4 bit mode
*/
#define UI_DISPLAY_TYPE 0

/** Number of columns per row

Typical values are 16 and 20
*/
#define UI_COLS 16
/**
Rows of your display. 2 or 4
*/
#define UI_ROWS 4

/**
Define the pin
*/
#define UI_DISPLAY_RS_PIN _BV(4)
#define UI_DISPLAY_RW_PIN _BV(5)
#define UI_DISPLAY_ENABLE_PIN _BV(6)
#define UI_DISPLAY_D0_PIN _BV(0)
#define UI_DISPLAY_D1_PIN _BV(1)
#define UI_DISPLAY_D2_PIN _BV(2)
#define UI_DISPLAY_D3_PIN _BV(3)
#define UI_DISPLAY_D4_PIN _BV(0)
#define UI_DISPLAY_D5_PIN _BV(1)
#define UI_DISPLAY_D6_PIN _BV(2)
#define UI_DISPLAY_D7_PIN _BV(3)
#define UI_DISPLAY_I2C_ADDRESS 0x4e

#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 4x16

/** \brief Are some keys connected?

0 = No keys attached - disables also menu
1 = Some keys attached
*/
#define UI_HAS_KEYS 0
/** \brief Is a back key present.

If you have menus enabled, you need a method to leave it. If you have a back key, you can always go one level higher.
Without a back key, you need to navigate to the back entry in the menu. Setting this value to 1 removes the back entry.
*/
#define UI_HAS_BACK_KEY 0
