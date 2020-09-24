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
*/

#ifndef _DISPLAY_BASE
#define _DISPLAY_BASE

#define DRIVER_NONE 0
#define DRIVER_U8G2 1
#define DRIVER_20x4 2

#define LCD_CONNECTOR_HW_SPI 0
#define LCD_CONNECTOR_SW_SPI 1
#define LCD_CONNECTOR_4BIT_PARALLEL 2
#define LCD_CONNECTOR_I2C_4BIT_PARALLEL 3
#define LCD_CONNECTOR_SERIAL 4

#define LCD_CHAR_ENCODING_HD44870_KNJI 1
#define LCD_CHAR_ENCODING_HD44870_WESTERN 2
#define LCD_CHAR_ENCODING_HD44870_UTF8 3

#ifndef DISPLAY_DRIVER
#define DISPLAY_DRIVER DRIVER_NONE
#endif

#ifndef UI_DELAYPERCHAR
#define UI_DELAYPERCHAR 50
#endif

#ifndef UI_START_SCREEN_DELAY
#define UI_START_SCREEN_DELAY 3000
#endif

// Map controller types to drivers and set meningfull defaults if not predefined
#if FEATURE_CONTROLLER == CONTROLLER_SPARKLCD || FEATURE_CONTROLLER == CONTROLLER_SPARKLCD_ADAPTER \
    || FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD || FEATURE_CONTROLLER == CONTROLLER_FELIX_DUE \
    || FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2 || FEATURE_CONTROLLER == CONTROLLER_ENDER_3_12864
#define DISPLAY_ST7920_SW 1
#endif

#if FEATURE_CONTROLLER == CONTROLLER_REPRAPWORLD_GLCD
#define DISPLAY_ST7920_HW 1
#endif

#if FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864 || FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864_OLED
#if FEATURE_CONTROLLER == CONTROLLER_AZSMZ_12864
#define DISPLAY_ROTATION U8G2_R2
#endif
#if MOTHERBOARD == 408 || MOTHERBOARD == 413 // SMART RAMPS
#define U8GLIB_ST7565_NHD_C2832_SW_SPI
#else
// Hardware SPI creates artifacts on display, so we use software SPI
#define U8GLIB_ST7565_NHD_C2832_HW_SPI
#endif
#endif

#if FEATURE_CONTROLLER == CONTROLLER_RADDS || FEATURE_CONTROLLER == CONTROLLER_SMARTRAMPS
#undef DISPLAY_DRIVER
#define DISPLAY_DRIVER DRIVER_20x4
#define LCD_CONNECTOR LCD_CONNECTOR_4BIT_PARALLEL
#define UI_ROWS 4
#define UI_COLS 20
//#define UI_LINE_OFFSETS {0,0x20,0x40,0x60} // 4x20 with KS0073
#define UI_LINE_OFFSETS \
    { 0, 0x40, 0x14, 0x54 } // 4x20 with HD44780
#define LCD_CHAR_ENCODING LCD_CHAR_ENCODING_HD44870_KNJI
#endif

#ifndef ENCODER_DIRECTION
#if FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2 || FEATURE_CONTROLLER == CONTROLLER_FELIX_DUE \
    || FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD
#define ENCODER_DIRECTION -1
#else
#define ENCODER_DIRECTION 1
#endif
#endif

#ifndef ENCODER_SPEED
// Speeds vary from 0 = fastest to 2 = slowest.
#if FEATURE_CONTROLLER == CONTROLLER_ORCABOTXXLPRO2 || FEATURE_CONTROLLER == CONTROLLER_REPRAPWORLD_GLCD \
    || FEATURE_CONTROLLER == CONTROLLER_RADDS || FEATURE_CONTROLLER == CONTROLLER_REPRAPDISCOUNT_GLCD
#define ENCODER_SPEED 1
#else
#define ENCODER_SPEED 2
#endif
#endif

// It is always safe to assume there is no back key for a display
#ifndef UI_HAS_BACK_KEY
#define UI_HAS_BACK_KEY 0
#endif

#if ENABLED(DISPLAY_ST7920_SW) || ENABLED(DISPLAY_ST7920_HW) || ENABLED(U8GLIB_ST7565_NHD_C2832_SW_SPI) || ENABLED(U8GLIB_ST7565_NHD_C2832_HW_SPI)
#undef DISPLAY_DRIVER
#define DISPLAY_DRIVER DRIVER_U8G2
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#endif

#if FEATURE_CONTROLLER != NO_CONTROLLER && DISPLAY_DRIVER == DRIVER_NONE
#error The selected display is currently not assigned to a driver!
#endif
#endif
