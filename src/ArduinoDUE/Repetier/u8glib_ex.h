/*

  U8glib.cpp

  C++ Interface

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


The U8glib code (http://code.google.com/p/u8glib/) is licensed under the terms of
the new-bsd license (two-clause bsd license).
See also: http://www.opensource.org/licenses/bsd-license.php

The repository and optionally the releases contain icons, which are
derived from the WPZOOM Developer Icon Set:
http://www.wpzoom.com/wpzoom/new-freebie-wpzoom-developer-icon-set-154-free-icons/
WPZOOM Developer Icon Set by WPZOOM is licensed under a Creative Commons
Attribution-ShareAlike 3.0 Unported License.

Fonts are licensed under different conditions.
See http://code.google.com/p/u8glib/wiki/fontgroup for
detailed information on the licensing conditions for each font.

============ X11 Fonts COUR, HELV, NCEN, TIM, SYMB ============

For fonts derived from the following files, the license below applies.
COURB08.BDF COURB10.BDF COURB12.BDF COURB14.BDF COURB18.BDF
COURB24.BDF COURR08.BDF COURR10.BDF COURR12.BDF COURR14.BDF
COURR18.BDF COURR24.BDF HELVB08.BDF HELVB10.BDF HELVB12.BDF HELVB14.BDF
HELVB18.BDF HELVB24.BDF HELVR08.BDF HELVR10.BDF HELVR12.BDF HELVR14.BDF
HELVR18.BDF HELVR24.BDF NCENB08.BDF NCENB10.BDF NCENB12.BDF
NCENB14.BDF NCENB18.BDF NCENB24.BDF NCENR08.BDF NCENR10.BDF
NCENR12.BDF NCENR14.BDF NCENR18.BDF NCENR24.BDF SYMB08.BDF SYMB10.BDF
SYMB12.BDF SYMB14.BDF SYMB18.BDF SYMB24.BDF TIMB08.BDF TIMB10.BDF
TIMB12.BDF TIMB14.BDF TIMB18.BDF TIMB24.BDF TIMR08.BDF TIMR10.BDF
TIMR12.BDF TIMR14.BDF TIMR18.BDF TIMR24.BDF

Copyright 1984-1989, 1994 Adobe Systems Incorporated.
Copyright 1988, 1994 Digital Equipment Corporation.

Adobe is a trademark of Adobe Systems Incorporated which may be
registered in certain jurisdictions.
Permission to use these trademarks is hereby granted only in
association with the images described in this file.

Permission to use, copy, modify, distribute and sell this software
and its documentation for any purpose and without fee is hereby
granted, provided that the above copyright notices appear in all
copies and that both those copyright notices and this permission
notice appear in supporting documentation, and that the names of
Adobe Systems and Digital Equipment Corporation not be used in
advertising or publicity pertaining to distribution of the software
without specific, written prior permission.  Adobe Systems and
Digital Equipment Corporation make no representations about the
suitability of this software for any purpose.  It is provided "as
is" without express or implied warranty.


============ BSD License for U8glib Code ============

Universal 8bit Graphics Library (http://code.google.com/p/u8glib/)

Copyright (c) 2011, olikraus@gmail.com
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list
  of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// ============= u8g.h ====================

/*

  u8g.h

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _U8G_H
#define _U8G_H

/* uncomment the following line to support displays larger than 240x240 */
//#define U8G_16BIT 1

/* comment the following line to generate more compact but interrupt unsafe code */
#define U8G_INTERRUPT_SAFE 1


#include <stddef.h>

#ifdef __18CXX
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
#else
#include <stdint.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

/*
  use the com interface directly on any systems which are not AVR or ARDUINO
*/
#if defined(__AVR__) || defined(ARDUINO)
#define U8G_WITH_PINLIST
#endif


#ifdef __cplusplus
extern "C" {
#endif


/*===============================================================*/
#ifdef __GNUC__
#  define U8G_NOINLINE __attribute__((noinline))
#  define U8G_PURE  __attribute__ ((pure))
#  define U8G_NOCOMMON __attribute__ ((nocommon))
#  define U8G_SECTION(name) __attribute__ ((section (name)))
#  if defined(__MSPGCC__)
/* mspgcc does not have .progmem sections. Use -fdata-sections. */
#    define U8G_FONT_SECTION(name)
#  endif
#  if defined(__AVR__)
#    define U8G_FONT_SECTION(name) U8G_SECTION(".progmem." name)
#  endif
#else
#  define U8G_NOINLINE
#  define U8G_PURE
#  define U8G_NOCOMMON
#  define U8G_SECTION(name)
#endif

#ifndef U8G_FONT_SECTION
#  define U8G_FONT_SECTION(name)
#endif


/*===============================================================*/
/* flash memory access */

#if defined(__AVR__)
/* U8G_PROGMEM is used by the XBM example */
#define U8G_PROGMEM U8G_SECTION(".progmem.data")
typedef uint8_t PROGMEM u8g_pgm_uint8_t;
typedef uint8_t u8g_fntpgm_uint8_t;
#define u8g_pgm_read(adr) pgm_read_byte_near(adr)
#define U8G_PSTR(s) ((u8g_pgm_uint8_t *)PSTR(s))

#else

#define U8G_PROGMEM
#define PROGMEM
typedef uint8_t u8g_pgm_uint8_t;
typedef uint8_t u8g_fntpgm_uint8_t;
#define u8g_pgm_read(adr) (*(const u8g_pgm_uint8_t *)(adr))
#define U8G_PSTR(s) ((u8g_pgm_uint8_t *)(s))

#endif

/*===============================================================*/
/* interrupt safe code */
#if defined(U8G_INTERRUPT_SAFE)
#  if defined(__AVR__)
extern uint8_t global_SREG_backup;	/* u8g_state.c */
#    define U8G_ATOMIC_START()		do { global_SREG_backup = SREG; cli(); } while(0)
#    define U8G_ATOMIC_END()			SREG = global_SREG_backup
#    define U8G_ATOMIC_OR(ptr, val) 	do { uint8_t tmpSREG = SREG; cli(); (*(ptr) |= (val)); SREG = tmpSREG; } while(0)
#    define U8G_ATOMIC_AND(ptr, val) 	do { uint8_t tmpSREG = SREG; cli(); (*(ptr) &= (val)); SREG = tmpSREG; } while(0)
#  else
#    define U8G_ATOMIC_OR(ptr, val) (*(ptr) |= (val))
#    define U8G_ATOMIC_AND(ptr, val) (*(ptr) &= (val))
#    define U8G_ATOMIC_START()
#    define U8G_ATOMIC_END()
#  endif /* __AVR__ */
#else
#  define U8G_ATOMIC_OR(ptr, val) (*(ptr) |= (val))
#  define U8G_ATOMIC_AND(ptr, val) (*(ptr) &= (val))
#  define U8G_ATOMIC_START()
#  define U8G_ATOMIC_END()
#endif /* U8G_INTERRUPT_SAFE */


/*===============================================================*/
/* forward */
typedef struct _u8g_t u8g_t;
typedef struct _u8g_dev_t u8g_dev_t;

typedef struct _u8g_dev_arg_pixel_t u8g_dev_arg_pixel_t;
typedef struct _u8g_dev_arg_bbx_t u8g_dev_arg_bbx_t;
typedef struct _u8g_box_t u8g_box_t;
typedef struct _u8g_dev_arg_irgb_t u8g_dev_arg_irgb_t;


/*===============================================================*/
/* generic */
#if defined(U8G_16BIT)
typedef uint16_t u8g_uint_t;
typedef int16_t u8g_int_t;
#else
typedef uint8_t u8g_uint_t;
typedef int8_t u8g_int_t;
#endif

#ifdef OBSOLETE
struct _u8g_box_t
{
  u8g_uint_t x0, y0, x1, y1;
};
typedef struct _u8g_box_t u8g_box_t;
#endif /* OBSOLETE */


/*===============================================================*/
/* device structure */

#ifdef __XC8
/* device prototype */
typedef uint8_t (*u8g_dev_fnptr)(void *u8g, void *dev, uint8_t msg, void *arg);

/* com prototype */
typedef uint8_t (*u8g_com_fnptr)(void *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
#else
/* device prototype */
typedef uint8_t (*u8g_dev_fnptr)(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* com prototype */
typedef uint8_t (*u8g_com_fnptr)(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
#endif



struct _u8g_dev_t
{
  u8g_dev_fnptr dev_fn;         /* device procedure */
  void *dev_mem;                /* device memory */
  u8g_com_fnptr com_fn;         /* communication procedure */
};


/*===============================================================*/
/* device list */

/* Size: 128x64 SDL, u8g_dev_sdl.c */
extern u8g_dev_t u8g_dev_sdl_1bit;
extern u8g_dev_t u8g_dev_sdl_1bit_h;
extern u8g_dev_t u8g_dev_sdl_2bit;
extern u8g_dev_t u8g_dev_sdl_2bit_double_mem;
extern u8g_dev_t u8g_dev_sdl_8bit;
extern u8g_dev_t u8g_dev_sdl_hicolor;
extern u8g_dev_t u8g_dev_sdl_fullcolor;
int u8g_sdl_get_key(void);

/* Size: 70x30 monochrom, stdout */
extern u8g_dev_t u8g_dev_stdout;

/* Size: monochrom, writes "u8g.pbm" */
extern u8g_dev_t u8g_dev_pbm;
extern u8g_dev_t u8g_dev_pbm_8h1;
extern u8g_dev_t u8g_dev_pbm_8h2;	/* grayscale simulation */

/* Size: 128x64 monochrom, no output, used for performance measure */
extern u8g_dev_t u8g_dev_gprof;

/* Display: EA DOGS102, Size: 102x64 monochrom */
extern u8g_dev_t u8g_dev_uc1701_dogs102_sw_spi;
extern u8g_dev_t u8g_dev_uc1701_dogs102_hw_spi;

extern u8g_dev_t u8g_dev_uc1701_dogs102_2x_sw_spi;
extern u8g_dev_t u8g_dev_uc1701_dogs102_2x_hw_spi;

/* Display: Mini12864 (dealextreme), Size: 128x64 monochrom */
extern u8g_dev_t u8g_dev_uc1701_mini12864_sw_spi;
extern u8g_dev_t u8g_dev_uc1701_mini12864_hw_spi;

extern u8g_dev_t u8g_dev_uc1701_mini12864_2x_sw_spi;
extern u8g_dev_t u8g_dev_uc1701_mini12864_2x_hw_spi;

/* Display: EA DOGM132, Size: 128x32 monochrom */
extern u8g_dev_t u8g_dev_st7565_dogm132_sw_spi;
extern u8g_dev_t u8g_dev_st7565_dogm132_hw_spi;

/* Display: EA DOGM128, Size: 128x64 monochrom */
extern u8g_dev_t u8g_dev_st7565_dogm128_sw_spi;
extern u8g_dev_t u8g_dev_st7565_dogm128_hw_spi;
extern u8g_dev_t u8g_dev_st7565_dogm128_parallel;

extern u8g_dev_t u8g_dev_st7565_dogm128_2x_sw_spi;
extern u8g_dev_t u8g_dev_st7565_dogm128_2x_hw_spi;
extern u8g_dev_t u8g_dev_st7565_dogm128_2x_parallel;

/* Display: Topway LM6059 128x64 (Adafruit) */
extern u8g_dev_t u8g_dev_st7565_lm6059_sw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6059_hw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6059_2x_sw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6059_2x_hw_spi;
/* Display: Topway LM6063 128x64 */
extern u8g_dev_t u8g_dev_st7565_lm6063_sw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6063_hw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6063_2x_sw_spi;
extern u8g_dev_t u8g_dev_st7565_lm6063_2x_hw_spi;
/* Display: Newhaven NHD-C12864 */
extern u8g_dev_t u8g_dev_st7565_nhd_c12864_sw_spi;
extern u8g_dev_t u8g_dev_st7565_nhd_c12864_hw_spi;
extern u8g_dev_t u8g_dev_st7565_nhd_c12864_2x_sw_spi;
extern u8g_dev_t u8g_dev_st7565_nhd_c12864_2x_hw_spi;

/* Display: Newhaven NHD-C12832 */
extern u8g_dev_t u8g_dev_st7565_nhd_c12832_sw_spi;
extern u8g_dev_t u8g_dev_st7565_nhd_c12832_hw_spi;
extern u8g_dev_t u8g_dev_st7565_nhd_c12832_parallel;
extern u8g_dev_t u8g_dev_st7565_nhd_c12832_hw_usart_spi;

/* Display: Displaytech 64128N */
extern u8g_dev_t u8g_dev_st7565_64128n_sw_spi;
extern u8g_dev_t u8g_dev_st7565_64128n_hw_spi;
extern u8g_dev_t u8g_dev_st7565_64128n_parallel;

extern u8g_dev_t u8g_dev_st7565_64128n_2x_sw_spi;
extern u8g_dev_t u8g_dev_st7565_64128n_2x_hw_spi;
extern u8g_dev_t u8g_dev_st7565_64128n_2x_parallel;

/* Display: LCD-AG-C128032R-DIW W/KK E6 PBF */
extern u8g_dev_t u8g_dev_uc1601_c128032_sw_spi;
extern u8g_dev_t u8g_dev_uc1601_c128032_hw_spi;

extern u8g_dev_t u8g_dev_uc1601_c128032_2x_sw_spi;
extern u8g_dev_t u8g_dev_uc1601_c128032_2x_hw_spi;

/* dfrobot 128x64 Graphic LCD (SKU:FIT0021) */
extern u8g_dev_t u8g_dev_st7920_128x64_sw_spi;
extern u8g_dev_t u8g_dev_st7920_128x64_hw_spi;
extern u8g_dev_t u8g_dev_st7920_128x64_8bit;
extern u8g_dev_t u8g_dev_st7920_128x64_custom;

extern u8g_dev_t u8g_dev_st7920_128x64_4x_sw_spi;
extern u8g_dev_t u8g_dev_st7920_128x64_4x_hw_spi;
extern u8g_dev_t u8g_dev_st7920_128x64_4x_8bit;
extern u8g_dev_t u8g_dev_st7920_128x64_4x_custom;

/* NHD-19232WG */
extern u8g_dev_t u8g_dev_st7920_192x32_sw_spi;
extern u8g_dev_t u8g_dev_st7920_192x32_hw_spi;
extern u8g_dev_t u8g_dev_st7920_192x32_8bit;

extern u8g_dev_t u8g_dev_st7920_192x32_4x_sw_spi;
extern u8g_dev_t u8g_dev_st7920_192x32_4x_hw_spi;
extern u8g_dev_t u8g_dev_st7920_192x32_4x_8bit;

/* CrystalFontz CFAG20232 */
extern u8g_dev_t u8g_dev_st7920_202x32_sw_spi;
extern u8g_dev_t u8g_dev_st7920_202x32_hw_spi;
extern u8g_dev_t u8g_dev_st7920_202x32_8bit;

extern u8g_dev_t u8g_dev_st7920_202x32_4x_sw_spi;
extern u8g_dev_t u8g_dev_st7920_202x32_4x_hw_spi;
extern u8g_dev_t u8g_dev_st7920_202x32_4x_8bit;

/* LC7981 160x80 display */
extern u8g_dev_t u8g_dev_lc7981_160x80_8bit;
/* LC7981 240x64 display */
extern u8g_dev_t u8g_dev_lc7981_240x64_8bit;
/* LC7981 240x128 display */
extern u8g_dev_t u8g_dev_lc7981_240x128_8bit;
/* LC7981 320x64 display */
extern u8g_dev_t u8g_dev_lc7981_320x64_8bit;

/* T6963, all t6963 devices have double page (2x) */
extern u8g_dev_t u8g_dev_t6963_240x128_8bit;
extern u8g_dev_t u8g_dev_t6963_240x64_8bit;
extern u8g_dev_t u8g_dev_t6963_128x64_8bit;

/* Display: EA DOGXL160, Size: 160x104 monochrom & gray level */
extern u8g_dev_t u8g_dev_uc1610_dogxl160_bw_sw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_bw_hw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_gr_sw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_gr_hw_spi;

extern u8g_dev_t u8g_dev_uc1610_dogxl160_2x_bw_sw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_2x_bw_hw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_2x_gr_sw_spi;
extern u8g_dev_t u8g_dev_uc1610_dogxl160_2x_gr_hw_spi;

/* Display: Generic KS0108b, Size: 128x64 monochrom */
extern u8g_dev_t u8g_dev_ks0108_128x64;         /* official Arduino Library interface */
extern u8g_dev_t u8g_dev_ks0108_128x64_fast;    /* faster, but uses private tables from the Arduino Library */

/* Nokia 84x48 Display with PCD8544 */
extern u8g_dev_t u8g_dev_pcd8544_84x48_sw_spi;
extern u8g_dev_t u8g_dev_pcd8544_84x48_hw_spi;
extern u8g_dev_t u8g_dev_tls8204_84x48_sw_spi;

/* Nokia 96x65 Display with PCF8812 */
extern u8g_dev_t u8g_dev_pcf8812_96x65_sw_spi;
extern u8g_dev_t u8g_dev_pcf8812_96x65_hw_spi;

/* NHD-2.7-12864UCY3 OLED Display with SSD1325 Controller */
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_bw_sw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_bw_hw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_bw_parallel;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_gr_hw_spi;

extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_bw_sw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_bw_hw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_bw_parallel;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_gr_hw_spi;

/* LY120 OLED with SSD1327 Controller (tested with Seeedstudio module) */
extern u8g_dev_t u8g_dev_ssd1327_96x96_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1327_96x96_gr_hw_spi;
extern u8g_dev_t u8g_dev_ssd1327_96x96_gr_i2c;

extern u8g_dev_t u8g_dev_ssd1327_96x96_2x_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1327_96x96_2x_gr_hw_spi;
extern u8g_dev_t u8g_dev_ssd1327_96x96_2x_gr_i2c;

/* NHD-3.12-25664 OLED Display with SSD1322 Controller */
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_bw_sw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_bw_hw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_bw_parallel;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_2x_bw_sw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_2x_bw_hw_spi;

extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_gr_hw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_gr_parallel;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_2x_gr_sw_spi;
extern u8g_dev_t u8g_dev_ssd1322_nhd31oled_2x_gr_hw_spi;

/* OLED 128x64 Display with SSD1306 Controller */
extern u8g_dev_t u8g_dev_ssd1306_128x64_sw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x64_hw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x64_i2c;

extern u8g_dev_t u8g_dev_ssd1306_128x64_2x_sw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x64_2x_hw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x64_2x_i2c;

/* OLED 128x64 Display with SSD1309 Controller */
extern u8g_dev_t u8g_dev_ssd1309_128x64_sw_spi;
extern u8g_dev_t u8g_dev_ssd1309_128x64_hw_spi;
extern u8g_dev_t u8g_dev_ssd1309_128x64_i2c;

/* OLED 128x32 Display with SSD1306 Controller */
extern u8g_dev_t u8g_dev_ssd1306_128x32_sw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x32_hw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x32_i2c;

extern u8g_dev_t u8g_dev_ssd1306_128x32_2x_sw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x32_2x_hw_spi;
extern u8g_dev_t u8g_dev_ssd1306_128x32_2x_i2c;

/* experimental 65K TFT with st7687 controller */
extern u8g_dev_t u8g_dev_st7687_c144mvgd_sw_spi;
extern u8g_dev_t u8g_dev_st7687_c144mvgd_8bit;

/* SBN1661/SED1520 display with 122x32 */
extern u8g_dev_t u8g_dev_sbn1661_122x32;

/* flip disc matrix */
extern u8g_dev_t u8g_dev_flipdisc_2x7;
void u8g_SetFlipDiscCallback(u8g_t *u8g, void (*cb)(uint8_t id, uint8_t page, uint8_t width, uint8_t *row1, uint8_t *row2));

/* ILI9325D based TFT */
extern u8g_dev_t u8g_dev_ili9325d_320x240_8bit;


/* SSD1351 OLED (breakout board from http://www.kickstarter.com/projects/ilsoftltd/colour-oled-breakout-board) */
extern u8g_dev_t u8g_dev_ssd1351_128x128_332_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_332_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_4x_332_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_4x_332_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_idx_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_idx_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_hicolor_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_hicolor_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_4x_hicolor_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128_4x_hicolor_hw_spi;

/* SSD1351 OLED (Freetronics, GPIOs set to high level) */
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_332_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_332_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_4x_332_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_4x_332_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_hicolor_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_hicolor_hw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_4x_hicolor_sw_spi;
extern u8g_dev_t u8g_dev_ssd1351_128x128gh_4x_hicolor_hw_spi;

/* HT1632 */
extern u8g_dev_t u8g_dev_ht1632_24x16;

/* A2 Micro Printer */
extern u8g_dev_t u8g_dev_a2_micro_printer_384x240;
extern u8g_dev_t u8g_dev_a2_micro_printer_192x120_ds;

/* u8g_virtual_screen.c  */
extern u8g_dev_t u8g_dev_vs;


/*===============================================================*/
/* device messages */

struct _u8g_dev_arg_pixel_t
{
  u8g_uint_t x, y;              /* will be modified */
  uint8_t pixel;                  /* will be modified, pixel sequence or transparency value */
  uint8_t dir;
  uint8_t color;			/* color or index value, red value for true color mode */
  uint8_t hi_color;		/* high byte for 64K color mode, low byte is in "color", green value for true color mode */
  uint8_t blue;			/* blue value in true color mode */
};
/* typedef struct _u8g_dev_arg_pixel_t u8g_dev_arg_pixel_t; */ /* forward decl */

/* range for r,g,b: 0..255 */
#define U8G_GET_HICOLOR_BY_RGB(r,g,b) (((uint16_t)((r)&0x0f8))<<8)|(((uint16_t)((g)&0x0fc))<<3)|(((uint16_t)((b)>>3)))

struct _u8g_dev_arg_bbx_t
{
  u8g_uint_t x, y, w, h;
};
/* typedef struct _u8g_dev_arg_bbx_t u8g_dev_arg_bbx_t; */ /* forward decl */

struct _u8g_box_t
{
  u8g_uint_t x0, y0, x1, y1;
};
/* typedef struct _u8g_box_t u8g_box_t; */ /* forward decl */

struct _u8g_dev_arg_irgb_t
{
  u8g_uint_t idx, r, g, b;		/* index with rgb value */
};
/* typedef struct _u8g_dev_arg_irgb_t u8g_dev_arg_irgb_t; */ /* forward decl */



#define U8G_DEV_MSG_INIT                10
#define U8G_DEV_MSG_STOP                  11

/* arg: pointer to uint8_t, contranst value between 0 and 255 */
#define U8G_DEV_MSG_CONTRAST            15

#define U8G_DEV_MSG_SLEEP_ON            16
#define U8G_DEV_MSG_SLEEP_OFF            17

#define U8G_DEV_MSG_PAGE_FIRST                  20
#define U8G_DEV_MSG_PAGE_NEXT                    21

/* arg: u8g_dev_arg_bbx_t * */
/* new algorithm with U8G_DEV_MSG_GET_PAGE_BOX makes this msg obsolete */
/* #define U8G_DEV_MSG_IS_BBX_INTERSECTION 22 */

/* arg: u8g_box_t *, fill structure with current page properties */
#define U8G_DEV_MSG_GET_PAGE_BOX 23

/*
#define U8G_DEV_MSG_PRIMITIVE_START             30
#define U8G_DEV_MSG_PRIMITIVE_END               31
*/

/* arg: u8g_dev_arg_pixel_t * */
#define U8G_DEV_MSG_SET_TPIXEL				44
#define U8G_DEV_MSG_SET_4TPIXEL			45

#define U8G_DEV_MSG_SET_PIXEL                           50
#define U8G_DEV_MSG_SET_8PIXEL                          59

#define U8G_DEV_MSG_SET_COLOR_ENTRY                60

#define U8G_DEV_MSG_SET_XY_CB                           61

#define U8G_DEV_MSG_GET_WIDTH                           70
#define U8G_DEV_MSG_GET_HEIGHT                           71
#define U8G_DEV_MSG_GET_MODE                  72

/*===============================================================*/
/* device modes */
#define U8G_MODE(is_index_mode, is_color, bits_per_pixel) (((is_index_mode)<<6) | ((is_color)<<5)|(bits_per_pixel))

#define U8G_MODE_UNKNOWN     0
#define U8G_MODE_BW     U8G_MODE(0, 0, 1)
#define U8G_MODE_GRAY2BIT     U8G_MODE(0, 0, 2)
#define U8G_MODE_R3G3B2  U8G_MODE(0, 1, 8)
#define U8G_MODE_INDEX  U8G_MODE(1, 1, 8)
/* hicolor is R5G6B5 */
#define U8G_MODE_HICOLOR  U8G_MODE(0, 1, 16)
/* truecolor  */
#define U8G_MODE_TRUECOLOR  U8G_MODE(0, 1, 24)


#define U8G_MODE_GET_BITS_PER_PIXEL(mode) ((mode)&31)
#define U8G_MODE_IS_COLOR(mode) (((mode)&32)==0?0:1)
#define U8G_MODE_IS_INDEX_MODE(mode) (((mode)&64)==0?0:1)


/*===============================================================*/
/* com options */

/* uncomment the following line for Atmega HW SPI double speed, issue 89 */
/* #define U8G_HW_SPI_2X 1 */

/* com messages */

#define U8G_COM_MSG_STOP        0
#define U8G_COM_MSG_INIT        1

#define U8G_COM_MSG_ADDRESS 2

/* CHIP_SELECT argument: number of the chip which needs to be activated, so this is more like high active */
#define U8G_COM_MSG_CHIP_SELECT 3

#define U8G_COM_MSG_RESET 4

#define U8G_COM_MSG_WRITE_BYTE 5
#define U8G_COM_MSG_WRITE_SEQ 6
#define U8G_COM_MSG_WRITE_SEQ_P 7


/* com driver */
uint8_t u8g_com_null_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);               /* u8g_com_null.c */
uint8_t u8g_com_arduino_std_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);        /* u8g_com_arduino_std_sw_spi.c */
uint8_t u8g_com_arduino_hw_usart_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);      /* u8g_com_atmega_hw_usart_spi.c */
uint8_t u8g_com_arduino_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);        /* u8g_com_arduino_sw_spi.c */
uint8_t u8g_com_arduino_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);          /* u8g_com_arduino_hw_spi.c */
uint8_t u8g_com_arduino_ATtiny85_std_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);          /* u8g_arduino_ATTiny85_std_hw_spi.c */
uint8_t u8g_com_arduino_st7920_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);  /* u8g_com_arduino_st7920_spi.c */
uint8_t u8g_com_arduino_st7920_custom_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr); /* u8g_com_arduino_st7920_custom.c */
uint8_t u8g_com_arduino_st7920_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);  /* u8g_com_arduino_st7920_hw_spi.c */
uint8_t u8g_com_arduino_parallel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);           /* u8g_com_arduino_parallel.c */
uint8_t u8g_com_arduino_fast_parallel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);      /* u8g_com_arduino_fast_parallel.c */
uint8_t u8g_com_arduino_port_d_wr_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);       /* u8g_com_arduino_port_d_wr.c */
uint8_t u8g_com_arduino_no_en_parallel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);	/* u8g_com_arduino_no_en_parallel.c */
uint8_t u8g_com_arduino_ssd_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);		/* u8g_com_arduino_ssd_i2c.c */
uint8_t u8g_com_arduino_t6963_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);			/* u8g_com_arduino_t6963.c */


uint8_t u8g_com_atmega_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);      /* u8g_com_atmega_hw_spi.c */
uint8_t u8g_com_atmega_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);      /* u8g_com_atmega_sw_spi.c */
uint8_t u8g_com_atmega_st7920_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);	/* u8g_com_atmega_st7920_spi.c */
uint8_t u8g_com_atmega_st7920_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);
uint8_t u8g_com_atmega_parallel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);    /* u8g_com_atmega_parallel.c */


/*
  Translation of system specific com drives to generic com names
  At the moment, the following generic com drives are available
  U8G_COM_HW_SPI
  U8G_COM_SW_SPI
  U8G_COM_PARALLEL
  U8G_COM_T6963
  U8G_COM_FAST_PARALLEL
  U8G_COM_SSD_I2C

defined(__18CXX) || defined(__PIC32MX)

*/
/* ==== HW SPI, Arduino ====*/
#if defined(ARDUINO)
#if defined(__AVR__)

#if defined(__AVR_ATtiny85__)
#define U8G_COM_HW_SPI u8g_com_arduino_ATtiny85_std_hw_spi_fn
#define U8G_COM_ST7920_HW_SPI u8g_com_null_fn
#else

#define U8G_COM_HW_SPI u8g_com_arduino_hw_spi_fn
#if defined(__AVR_ATmega32U4__)
#define U8G_COM_HW_USART_SPI u8g_com_arduino_hw_usart_spi_fn
#endif /* __AVR_ATmega32U4__ */
#define U8G_COM_ST7920_HW_SPI u8g_com_arduino_st7920_hw_spi_fn
#endif /* __AVR_ATtiny85__ */

#elif defined(__18CXX) || defined(__PIC32MX)
#define U8G_COM_HW_SPI u8g_com_null_fn
#define U8G_COM_ST7920_HW_SPI u8g_com_null_fn
#elif defined(__arm__)   /* Arduino Due */
#define U8G_COM_HW_SPI u8g_com_arduino_hw_spi_fn
#define U8G_COM_ST7920_HW_SPI u8g_com_null_fn
#endif
#endif
/* ==== HW SPI, not Arduino ====*/
#ifndef U8G_COM_HW_SPI
#if defined(__AVR__)
#define U8G_COM_HW_SPI u8g_com_atmega_hw_spi_fn
#define U8G_COM_ST7920_HW_SPI u8g_com_atmega_st7920_hw_spi_fn
#endif
#endif
#ifndef U8G_COM_HW_SPI
#define U8G_COM_HW_SPI u8g_com_null_fn
#define U8G_COM_ST7920_HW_SPI u8g_com_null_fn
#endif

#ifndef U8G_COM_HW_USART_SPI
#define U8G_COM_HW_USART_SPI u8g_com_null_fn
#endif


/* ==== SW SPI, Arduino ====*/
#if defined(ARDUINO)
#if defined(__AVR__)
#define U8G_COM_SW_SPI u8g_com_arduino_sw_spi_fn
#define U8G_COM_ST7920_SW_SPI u8g_com_arduino_st7920_spi_fn
#elif defined(__18CXX) || defined(__PIC32MX)
#define U8G_COM_SW_SPI u8g_com_arduino_sw_spi_fn
#define U8G_COM_ST7920_SW_SPI u8g_com_arduino_st7920_spi_fn
#elif defined(__arm__)   /* Arduino Due */
//#define U8G_COM_SW_SPI u8g_com_arduino_std_sw_spi_fn
#define U8G_COM_SW_SPI u8g_com_arduino_sw_spi_fn
#define U8G_COM_ST7920_SW_SPI u8g_com_arduino_st7920_spi_fn
#endif
#endif

#ifndef U8G_COM_SW_SPI
/* ==== SW SPI, not Arduino ====*/
#if defined(__AVR__)
#define U8G_COM_SW_SPI u8g_com_atmega_sw_spi_fn
#define U8G_COM_ST7920_SW_SPI u8g_com_atmega_st7920_sw_spi_fn
#endif
#endif
#ifndef U8G_COM_SW_SPI
#define U8G_COM_SW_SPI u8g_com_null_fn
#define U8G_COM_ST7920_SW_SPI u8g_com_null_fn
#endif

/* ==== Parallel iinterface, Arduino ====*/
#if defined(ARDUINO)
#if defined(__AVR__)
#define U8G_COM_PARALLEL u8g_com_arduino_parallel_fn
#define U8G_COM_FAST_PARALLEL u8g_com_arduino_fast_parallel_fn
#define U8G_COM_T6963  u8g_com_arduino_t6963_fn
#else /* Arduino Due, Chipkit PIC32 */
#define U8G_COM_PARALLEL u8g_com_arduino_parallel_fn
#define U8G_COM_FAST_PARALLEL u8g_com_arduino_parallel_fn
#define U8G_COM_T6963  u8g_com_null_fn
#endif
#endif
#ifndef U8G_COM_PARALLEL
#if defined(__AVR__)
#define U8G_COM_PARALLEL u8g_com_atmega_parallel_fn
#define U8G_COM_FAST_PARALLEL u8g_com_atmega_parallel_fn
#define U8G_COM_T6963  u8g_com_null_fn
#endif
#endif
#ifndef U8G_COM_PARALLEL
#define U8G_COM_PARALLEL u8g_com_null_fn
#define U8G_COM_FAST_PARALLEL u8g_com_null_fn
#define U8G_COM_T6963  u8g_com_null_fn
#endif

#if defined(ARDUINO)
#if defined(__AVR__)
#define U8G_COM_SSD_I2C u8g_com_arduino_ssd_i2c_fn
#endif
#endif

#ifndef U8G_COM_SSD_I2C
#if defined(__AVR__)
/* AVR variant can use the arduino version at the moment */
#define U8G_COM_SSD_I2C u8g_com_arduino_ssd_i2c_fn
#endif
#endif
#ifndef U8G_COM_SSD_I2C
#define U8G_COM_SSD_I2C u8g_com_null_fn
#endif



/*===============================================================*/
/* com api */

#define U8G_SPI_CLK_CYCLE_50NS 1
#define U8G_SPI_CLK_CYCLE_300NS 2
#define U8G_SPI_CLK_CYCLE_400NS 3
#define U8G_SPI_CLK_CYCLE_NONE 255

uint8_t u8g_InitCom(u8g_t *u8g, u8g_dev_t *dev, uint8_t clk_cycle_time);
void u8g_StopCom(u8g_t *u8g, u8g_dev_t *dev);
void u8g_EnableCom(u8g_t *u8g, u8g_dev_t *dev);         /* obsolete */
void u8g_DisableCom(u8g_t *u8g, u8g_dev_t *dev);        /* obsolete */
void u8g_SetChipSelect(u8g_t *u8g, u8g_dev_t *dev, uint8_t cs);
void u8g_SetResetLow(u8g_t *u8g, u8g_dev_t *dev);
void u8g_SetResetHigh(u8g_t *u8g, u8g_dev_t *dev);
void u8g_SetAddress(u8g_t *u8g, u8g_dev_t *dev, uint8_t address);
uint8_t u8g_WriteByte(u8g_t *u8g, u8g_dev_t *dev, uint8_t val);
uint8_t u8g_WriteSequence(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, uint8_t *seq);
uint8_t u8g_WriteSequenceP(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, const uint8_t *seq);



#define U8G_ESC_DLY(x) 255, ((x) & 0x7f)
#define U8G_ESC_CS(x) 255, (0xd0 | ((x)&0x0f))
#define U8G_ESC_ADR(x) 255, (0xe0 | ((x)&0x0f))
#define U8G_ESC_RST(x) 255, (0xc0 | ((x)&0x0f))
#define U8G_ESC_VCC(x) 255, (0xbe | ((x)&0x01))
#define U8G_ESC_END 255, 254
#define U8G_ESC_255 255, 255
//uint8_t u8g_WriteEscSeqP(u8g_t *u8g, u8g_dev_t *dev, u8g_pgm_uint8_t *esc_seq);
uint8_t u8g_WriteEscSeqP(u8g_t *u8g, u8g_dev_t *dev, const uint8_t *esc_seq);


/* u8g_com_api_16gr.c */
uint8_t u8g_WriteByteBWTo16GrDevice(u8g_t *u8g, u8g_dev_t *dev, uint8_t b);
uint8_t u8g_WriteSequenceBWTo16GrDevice(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, uint8_t *ptr);
uint8_t u8g_WriteByte4LTo16GrDevice(u8g_t *u8g, u8g_dev_t *dev, uint8_t b);
uint8_t u8g_WriteSequence4LTo16GrDevice(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, uint8_t *ptr);


/*===============================================================*/
/* u8g_arduino_common.c */
void u8g_com_arduino_digital_write(u8g_t *u8g, uint8_t pin_index, uint8_t value);
void u8g_com_arduino_assign_pin_output_high(u8g_t *u8g);

/*===============================================================*/
/* u8g_com_io.c */

/* create internal number from port and pin */
uint8_t u8g_Pin(uint8_t port, uint8_t bitpos);
#define PN(port,bitpos) u8g_Pin(port,bitpos)

/* low level procedures */
void u8g_SetPinOutput(uint8_t internal_pin_number);
void u8g_SetPinLevel(uint8_t internal_pin_number, uint8_t level);
void u8g_SetPinInput(uint8_t internal_pin_number);
uint8_t u8g_GetPinLevel(uint8_t internal_pin_number);

/* u8g level procedures, expect U8G_PI_xxx macro */
void u8g_SetPIOutput(u8g_t *u8g, uint8_t pi);
void u8g_SetPILevel(u8g_t *u8g, uint8_t pi, uint8_t level);


/*===============================================================*/
/* page */
struct _u8g_page_t
{
  u8g_uint_t page_height;
  u8g_uint_t total_height;
  u8g_uint_t page_y0;
  u8g_uint_t page_y1;
  uint8_t page;
};
typedef struct _u8g_page_t u8g_page_t;

void u8g_page_First(u8g_page_t *p) U8G_NOINLINE;                                                                                        /* u8g_page.c */
void u8g_page_Init(u8g_page_t *p, u8g_uint_t page_height, u8g_uint_t total_height ) U8G_NOINLINE;            /* u8g_page.c */
uint8_t u8g_page_Next(u8g_page_t *p) U8G_NOINLINE;                                                                                   /* u8g_page.c */

/*===============================================================*/
/* page buffer (pb) */

struct _u8g_pb_t
{
  u8g_page_t p;
  u8g_uint_t width;		/* pixel width */
  void *buf;
};
typedef struct _u8g_pb_t u8g_pb_t;


/* u8g_pb.c */
void u8g_pb_Clear(u8g_pb_t *b);
uint8_t u8g_pb_IsYIntersection(u8g_pb_t *pb, u8g_uint_t v0, u8g_uint_t v1);
uint8_t u8g_pb_IsXIntersection(u8g_pb_t *b, u8g_uint_t v0, u8g_uint_t v1);
uint8_t u8g_pb_IsIntersection(u8g_pb_t *pb, u8g_dev_arg_bbx_t *bbx);
void u8g_pb_GetPageBox(u8g_pb_t *pb, u8g_box_t *box);
uint8_t u8g_pb_Is8PixelVisible(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel);
uint8_t u8g_pb_WriteBuffer(u8g_pb_t *b, u8g_t *u8g, u8g_dev_t *dev);

/*
  note on __attribute__ ((nocommon))
    AVR scripts often use  --gc-sections on the linker to remove unused section.
    This works fine for initialed data and text sections. In principle .bss is also
    handled, but the name##_pb definition is not removed. Reason is, that
    array definitions are placed in the COMMON section, by default
    The attribute "nocommon" removes this automatic assignment to the
    COMMON section and directly puts it into .bss. As a result, if more
    than one buffer is defined in one file, then it will be removed with --gc-sections

    .. not sure if Arduino IDE uses -fno-common... if yes, then the attribute is
    redundant.
*/
#define U8G_PB_DEV(name, width, height, page_height, dev_fn, com_fn) \
uint8_t name##_buf[width] U8G_NOCOMMON ; \
u8g_pb_t name##_pb = { {page_height, height, 0, 0, 0},  width, name##_buf}; \
u8g_dev_t name = { dev_fn, &name##_pb, com_fn }


void u8g_pb8v1_Init(u8g_pb_t *b, void *buf, u8g_uint_t width)   U8G_NOINLINE;
void u8g_pb8v1_Clear(u8g_pb_t *b) U8G_NOINLINE;

uint8_t u8g_pb8v1_IsYIntersection(u8g_pb_t *b, u8g_uint_t v0, u8g_uint_t v1);
uint8_t u8g_pb8v1_IsXIntersection(u8g_pb_t *b, u8g_uint_t v0, u8g_uint_t v1);
uint8_t u8g_pb8v1_WriteBuffer(u8g_pb_t *b, u8g_t *u8g, u8g_dev_t *dev);

uint8_t u8g_dev_pb8v1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb16v1.c */
uint8_t u8g_dev_pb16v1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb14v1.c */
uint8_t u8g_dev_pb14v1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb8v2.c */
uint8_t u8g_dev_pb8v2_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb16v2.c (double memory of pb8v2) */
uint8_t u8g_dev_pb16v2_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);


/* u8g_pb8h1.c */
uint8_t u8g_dev_pb8h1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb16h1.c */
uint8_t u8g_dev_pb16h1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb32h1.c */
uint8_t u8g_dev_pb32h1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);


/* u8g_pb8h2.c 8 pixel rows, byte has horzontal orientation */
uint8_t u8g_dev_pb8h2_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb16h2.c */
uint8_t u8g_dev_pb16h2_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);



/* u8g_pb8h1f.c */
uint8_t u8g_dev_pb8h1f_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pb8h8.c */
uint8_t u8g_dev_pb8h8_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pbxh16.c */
uint8_t u8g_dev_pbxh16_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

/* u8g_pbxh24.c */
uint8_t u8g_dev_pbxh24_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);


/*===============================================================*/
/* u8g_ll_api.c */

/* cursor draw callback */
typedef void (*u8g_draw_cursor_fn)(u8g_t *u8g);

/* vertical reference point calculation callback */
typedef u8g_uint_t (*u8g_font_calc_vref_fnptr)(u8g_t *u8g);

/* state backup and restore procedure */
typedef void (*u8g_state_cb)(uint8_t msg);


/* PI = Pin Index */

/* reset pin, usually optional */
#define U8G_PI_RESET 0

/* address / data or instruction */
#define U8G_PI_A0 1
#define U8G_PI_DI 1

/* chip select line */
#define U8G_PI_CS 2
#define U8G_PI_CS1 2
#define U8G_PI_CS2 3
/* Feb 2013: A0 state moved from 7 to 3 for t6963 controller*/
#define U8G_PI_A0_STATE 3

/* enable / clock signal */
#define U8G_PI_EN 4
#define U8G_PI_CS_STATE 4
#define U8G_PI_SCK 4
#define U8G_PI_SCL 4
#define U8G_PI_RD 4


/* data pins, shared with SPI and I2C pins */
#define U8G_PI_D0 5
#define U8G_PI_MOSI 5
#define U8G_PI_SDA 5
#define U8G_PI_D1 6
#define U8G_PI_MISO 6
#define U8G_PI_D2 7
#define U8G_PI_D3 8
#define U8G_PI_SET_A0 8
#define U8G_PI_D4 9
#define U8G_PI_D5 10
#define U8G_PI_I2C_OPTION 11
#define U8G_PI_D6 11
#define U8G_PI_D7 12

/* read/write pin, must be the last pin in the list, this means U8G_PIN_LIST_LEN =  U8G_PI_RW + 1*/
#define U8G_PI_WR 13
#define U8G_PI_RW 13

#define U8G_PIN_LIST_LEN 14


#define U8G_PIN_DUMMY 254
#define U8G_PIN_NONE 255

#define U8G_FONT_HEIGHT_MODE_TEXT 0
#define U8G_FONT_HEIGHT_MODE_XTEXT 1
#define U8G_FONT_HEIGHT_MODE_ALL 2

struct _u8g_t
{
  u8g_uint_t width;
  u8g_uint_t height;


  u8g_dev_t *dev;               /* first device in the device chain */
  const u8g_pgm_uint8_t *font;             /* regular font for all text procedures */
  const u8g_pgm_uint8_t *cursor_font;  /* special font for cursor procedures */
  uint8_t cursor_fg_color, cursor_bg_color;
  uint8_t cursor_encoding;
  uint8_t mode;                         /* display mode, one of U8G_MODE_xxx */
  u8g_uint_t cursor_x;
  u8g_uint_t cursor_y;
  u8g_draw_cursor_fn cursor_fn;

  int8_t glyph_dx;
  int8_t glyph_x;
  int8_t glyph_y;
  uint8_t glyph_width;
  uint8_t glyph_height;

  u8g_font_calc_vref_fnptr font_calc_vref;
  uint8_t font_height_mode;
  int8_t font_ref_ascent;
  int8_t font_ref_descent;
  uint8_t font_line_spacing_factor;     /* line_spacing = factor * (ascent - descent) / 64 */
  uint8_t line_spacing;

  u8g_dev_arg_pixel_t arg_pixel;
  /* uint8_t color_index; */

#ifdef U8G_WITH_PINLIST
  uint8_t pin_list[U8G_PIN_LIST_LEN];
#endif

  u8g_state_cb state_cb;

  u8g_box_t current_page;		/* current box of the visible page */

};

#define u8g_GetFontAscent(u8g) ((u8g)->font_ref_ascent)
#define u8g_GetFontDescent(u8g) ((u8g)->font_ref_descent)
#define u8g_GetFontLineSpacing(u8g) ((u8g)->line_spacing)

uint8_t u8g_call_dev_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);

uint8_t u8g_InitLL(u8g_t *u8g, u8g_dev_t *dev);
void u8g_FirstPageLL(u8g_t *u8g, u8g_dev_t *dev);
uint8_t u8g_NextPageLL(u8g_t *u8g, u8g_dev_t *dev);
uint8_t u8g_SetContrastLL(u8g_t *u8g, u8g_dev_t *dev, uint8_t contrast);
void u8g_DrawPixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y);
void u8g_Draw8PixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel);
void u8g_Draw4TPixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel);
uint8_t u8g_IsBBXIntersectionLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h);	/* obsolete */
u8g_uint_t u8g_GetWidthLL(u8g_t *u8g, u8g_dev_t *dev);
u8g_uint_t u8g_GetHeightLL(u8g_t *u8g, u8g_dev_t *dev);

void u8g_UpdateDimension(u8g_t *u8g);
uint8_t u8g_Begin(u8g_t *u8g);				/* reset device, put it into default state and call u8g_UpdateDimension() */
uint8_t u8g_Init(u8g_t *u8g, u8g_dev_t *dev);   /* only usefull if the device only as hardcoded ports */
uint8_t u8g_InitComFn(u8g_t *u8g, u8g_dev_t *dev, u8g_com_fnptr com_fn);	/* Init procedure for anything which is not Arduino or AVR (e.g. ARM, but not Due, which is Arduino) */
uint8_t u8g_InitSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t sck, uint8_t mosi, uint8_t cs, uint8_t a0, uint8_t reset);
uint8_t u8g_InitHWSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t cs, uint8_t a0, uint8_t reset);
uint8_t u8g_InitI2C(u8g_t *u8g, u8g_dev_t *dev, uint8_t options);	/* use U8G_I2C_OPT_NONE as options */
uint8_t u8g_Init8BitFixedPort(u8g_t *u8g, u8g_dev_t *dev, uint8_t en, uint8_t cs, uint8_t di, uint8_t rw, uint8_t reset);
uint8_t u8g_Init8Bit(u8g_t *u8g, u8g_dev_t *dev, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
  uint8_t en, uint8_t cs1, uint8_t cs2, uint8_t di, uint8_t rw, uint8_t reset);
uint8_t u8g_InitRW8Bit(u8g_t *u8g, u8g_dev_t *dev, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
  uint8_t cs, uint8_t a0, uint8_t wr, uint8_t rd, uint8_t reset);
void u8g_FirstPage(u8g_t *u8g);
uint8_t u8g_NextPage(u8g_t *u8g);
uint8_t u8g_SetContrast(u8g_t *u8g, uint8_t contrast);
void u8g_SleepOn(u8g_t *u8g);
void u8g_SleepOff(u8g_t *u8g);
void u8g_DrawPixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y);
void u8g_Draw8Pixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel);
void u8g_Draw4TPixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel);

uint8_t u8g_Stop(u8g_t *u8g);
void u8g_SetColorEntry(u8g_t *u8g, uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void u8g_SetColorIndex(u8g_t *u8g, uint8_t idx);
void u8g_SetHiColor(u8g_t *u8g, uint16_t rgb);
void u8g_SetHiColorByRGB(u8g_t *u8g, uint8_t r, uint8_t g, uint8_t b);
void u8g_SetRGB(u8g_t *u8g, uint8_t r, uint8_t g, uint8_t b);
uint8_t u8g_GetColorIndex(u8g_t *u8g);

uint8_t u8g_GetDefaultForegroundColor(u8g_t *u8g);
void u8g_SetDefaultForegroundColor(u8g_t *u8g);

uint8_t u8g_GetDefaultBackgroundColor(u8g_t *u8g);
void u8g_SetDefaultBackgroundColor(u8g_t *u8g);

uint8_t u8g_GetDefaultMidColor(u8g_t *u8g);
void u8g_SetDefaultMidColor(u8g_t *u8g);

#define u8g_GetWidth(u8g) ((u8g)->width)
#define u8g_GetHeight(u8g) ((u8g)->height)
#define u8g_GetMode(u8g) ((u8g)->mode)
/*
  U8G_MODE_GET_BITS_PER_PIXEL(u8g_GetMode(u8g))
  U8G_MODE_IS_COLOR(u8g_GetMode(u8g))
*/

/* u8g_state.c */
#define U8G_STATE_ENV_IDX 0
#define U8G_STATE_U8G_IDX 1
#define U8G_STATE_RESTORE 0
#define U8G_STATE_BACKUP 1
#define U8G_STATE_MSG_COMPOSE(cmd,idx) (((cmd)<<1) | (idx))

#define U8G_STATE_MSG_RESTORE_ENV U8G_STATE_MSG_COMPOSE(U8G_STATE_RESTORE,U8G_STATE_ENV_IDX)
#define U8G_STATE_MSG_BACKUP_ENV U8G_STATE_MSG_COMPOSE(U8G_STATE_BACKUP,U8G_STATE_ENV_IDX)
#define U8G_STATE_MSG_RESTORE_U8G U8G_STATE_MSG_COMPOSE(U8G_STATE_RESTORE,U8G_STATE_U8G_IDX)
#define U8G_STATE_MSG_BACKUP_U8G U8G_STATE_MSG_COMPOSE(U8G_STATE_BACKUP,U8G_STATE_U8G_IDX)

#define U8G_STATE_MSG_GET_IDX(msg) ((msg)&1)
#define U8G_STATE_MSG_IS_BACKUP(msg) ((msg)&2)



void u8g_state_dummy_cb(uint8_t msg);
void u8g_backup_spi(uint8_t msg);		/* backup SPI state controller */
/* backward compatible definition */
#define u8g_backup_avr_spi u8g_backup_spi

void u8g_SetHardwareBackup(u8g_t *u8g, u8g_state_cb backup_cb);

/* u8g_clip.c */

uint8_t u8g_IsBBXIntersection(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h);


/* u8g_rot.c */

void u8g_UndoRotation(u8g_t *u8g);
void u8g_SetRot90(u8g_t *u8g);
void u8g_SetRot180(u8g_t *u8g);
void u8g_SetRot270(u8g_t *u8g);

/* u8g_scale.c */

void u8g_UndoScale(u8g_t *u8g);
void u8g_SetScale2x2(u8g_t *u8g);


/* u8g_font.c */

size_t u8g_font_GetSize(const void *font);
uint8_t u8g_font_GetFontStartEncoding(const void *font) U8G_NOINLINE;
uint8_t u8g_font_GetFontEndEncoding(const void *font) U8G_NOINLINE;

void u8g_SetFont(u8g_t *u8g, const u8g_fntpgm_uint8_t *font);

uint8_t u8g_GetFontBBXWidth(u8g_t *u8g);
uint8_t u8g_GetFontBBXHeight(u8g_t *u8g);
int8_t u8g_GetFontBBXOffX(u8g_t *u8g);
int8_t u8g_GetFontBBXOffY(u8g_t *u8g);
uint8_t u8g_GetFontCapitalAHeight(u8g_t *u8g);

uint8_t u8g_IsGlyph(u8g_t *u8g, uint8_t requested_encoding);
int8_t u8g_GetGlyphDeltaX(u8g_t *u8g, uint8_t requested_encoding);

int8_t u8g_draw_glyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding); /* used by u8g_cursor.c */

int8_t u8g_DrawGlyphDir(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t encoding);
int8_t u8g_DrawGlyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding);
int8_t u8g_DrawGlyph90(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding);
int8_t u8g_DrawGlyph180(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding);
int8_t u8g_DrawGlyph270(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding);
int8_t u8g_DrawGlyphFontBBX(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t encoding);

u8g_uint_t u8g_DrawStr(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s);
u8g_uint_t u8g_DrawStr90(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s);
u8g_uint_t u8g_DrawStr180(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s);
u8g_uint_t u8g_DrawStr270(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s);

u8g_uint_t u8g_DrawStrDir(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, const char *s);


u8g_uint_t u8g_DrawStrP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s);
u8g_uint_t u8g_DrawStr90P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s);
u8g_uint_t u8g_DrawStr180P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s);
u8g_uint_t u8g_DrawStr270P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s);


void u8g_SetFontRefHeightText(u8g_t *u8g);
void u8g_SetFontRefHeightExtendedText(u8g_t *u8g);
void u8g_SetFontRefHeightAll(u8g_t *u8g);
void u8g_SetFontLineSpacingFactor(u8g_t *u8g, uint8_t factor);

u8g_uint_t u8g_font_calc_vref_font(u8g_t *u8g);
u8g_uint_t u8g_font_calc_vref_bottom(u8g_t *u8g);
u8g_uint_t u8g_font_calc_vref_top(u8g_t *u8g);
u8g_uint_t u8g_font_calc_vref_center(u8g_t *u8g);

void u8g_SetFontPosBaseline(u8g_t *u8g);
void u8g_SetFontPosBottom(u8g_t *u8g);
void u8g_SetFontPosCenter(u8g_t *u8g);
void u8g_SetFontPosTop(u8g_t *u8g);


u8g_uint_t u8g_GetStrPixelWidth(u8g_t *u8g, const char *s);
u8g_uint_t u8g_GetStrPixelWidthP(u8g_t *u8g, const u8g_pgm_uint8_t *s);
int8_t u8g_GetStrX(u8g_t *u8g, const char *s);
int8_t u8g_GetStrXP(u8g_t *u8g, const u8g_pgm_uint8_t *s);
u8g_uint_t u8g_GetStrWidth(u8g_t *u8g, const char *s);
u8g_uint_t u8g_GetStrWidthP(u8g_t *u8g, const u8g_pgm_uint8_t *s);

u8g_uint_t u8g_DrawStrFontBBX(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, const char *s);

void u8g_GetStrMinBox(u8g_t *u8g, const char *s, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height);
void u8g_GetStrAMinBox(u8g_t *u8g, const char *s, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height);


u8g_uint_t u8g_DrawAAStr(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s);

/* u8g_rect.c */

void u8g_draw_box(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h) U8G_NOINLINE;

void u8g_DrawHLine(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w);
void u8g_DrawVLine(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w);
void u8g_DrawFrame(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h);
void u8g_DrawBox(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h);

void u8g_DrawRFrame(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r);
void u8g_DrawRBox(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r);

/* u8g_bitmap.c */

void u8g_DrawHBitmap(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, const uint8_t *bitmap);
void u8g_DrawHBitmapP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, const u8g_pgm_uint8_t *bitmap);
void u8g_DrawBitmap(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, u8g_uint_t h, const uint8_t *bitmap);
void u8g_DrawBitmapP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, u8g_uint_t h, const u8g_pgm_uint8_t *bitmap);

void u8g_DrawXBM(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, const uint8_t *bitmap);
void u8g_DrawXBMP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, const u8g_pgm_uint8_t *bitmap);


/* u8g_line.c */
void u8g_DrawLine(u8g_t *u8g, u8g_uint_t x1, u8g_uint_t y1, u8g_uint_t x2, u8g_uint_t y2);


/* u8g_circle.c */

/* the following, commented code has been rewritten or is not yet finished
#define U8G_CIRC_UPPER_RIGHT 0x01
#define U8G_CIRC_UPPER_LEFT  0x02
#define U8G_CIRC_LOWER_LEFT 0x04
#define U8G_CIRC_LOWER_RIGHT  0x08
#define U8G_CIRC_ALL (U8G_CIRC_UPPER_RIGHT|U8G_CIRC_UPPER_LEFT|U8G_CIRC_LOWER_RIGHT|U8G_CIRC_LOWER_LEFT)
void u8g_DrawEmpCirc(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option);
void u8g_DrawFillCirc(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option);
void u8g_DrawEllipseRect(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t x1, u8g_uint_t y1);
*/

#define U8G_DRAW_UPPER_RIGHT 0x01
#define U8G_DRAW_UPPER_LEFT  0x02
#define U8G_DRAW_LOWER_LEFT 0x04
#define U8G_DRAW_LOWER_RIGHT  0x08
#define U8G_DRAW_ALL (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT)

void u8g_draw_circle(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option) U8G_NOINLINE;
void u8g_draw_disc(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option) U8G_NOINLINE;

void u8g_DrawCircle(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option);
void u8g_DrawDisc(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rad, uint8_t option);

/* u8g_ellipse.c */
void u8g_DrawEllipse(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rx, u8g_uint_t ry, uint8_t option);
void u8g_DrawFilledEllipse(u8g_t *u8g, u8g_uint_t x0, u8g_uint_t y0, u8g_uint_t rx, u8g_uint_t ry, uint8_t option);

/* u8g_clip.c */
uint8_t u8g_is_box_bbx_intersection(u8g_box_t *box, u8g_dev_arg_bbx_t *bbx);


/* u8g_cursor.c */
void u8g_SetCursorFont(u8g_t *u8g, const u8g_pgm_uint8_t *cursor_font);
void u8g_SetCursorStyle(u8g_t *u8g, uint8_t encoding);
void u8g_SetCursorPos(u8g_t *u8g, u8g_uint_t cursor_x, u8g_uint_t cursor_y);
void u8g_SetCursorColor(u8g_t *u8g, uint8_t fg, uint8_t bg);
void u8g_EnableCursor(u8g_t *u8g);
void u8g_DisableCursor(u8g_t *u8g);
void u8g_DrawCursor(u8g_t *u8g);



/*===============================================================*/
/* u8g_virtual_screen.c */
void u8g_SetVirtualScreenDimension(u8g_t *vs_u8g, u8g_uint_t width, u8g_uint_t height);
uint8_t u8g_AddToVirtualScreen(u8g_t *vs_u8g, u8g_uint_t x, u8g_uint_t y, u8g_t *child_u8g);

/*===============================================================*/
void st_Draw(uint8_t fps);
void st_Step(uint8_t player_pos, uint8_t is_auto_fire, uint8_t is_fire);

/*===============================================================*/
/* u8g_com_i2c.c */

/* options for u8g_i2c_init() */
#define U8G_I2C_OPT_NONE 0

/* retrun values from u8g_twi_get_error() */
#define U8G_I2C_ERR_NONE 0x00
/* the following values are bit masks */
#define U8G_I2C_ERR_TIMEOUT 0x01
#define U8G_I2C_ERR_BUS 0x02

void u8g_i2c_clear_error(void) U8G_NOINLINE;
uint8_t  u8g_i2c_get_error(void) U8G_NOINLINE;
uint8_t u8g_i2c_get_err_pos(void) U8G_NOINLINE;
void u8g_i2c_init(uint8_t options) U8G_NOINLINE;		/* use U8G_I2C_OPT_NONE as options */
uint8_t u8g_i2c_wait(uint8_t mask, uint8_t pos) U8G_NOINLINE;
uint8_t u8g_i2c_start(uint8_t sla) U8G_NOINLINE;
uint8_t u8g_i2c_send_byte(uint8_t data) U8G_NOINLINE;
void u8g_i2c_stop(void) U8G_NOINLINE;


/*===============================================================*/
/* u8g_u8toa.c */
/* v = value, d = number of digits */
const char *u8g_u8toa(uint8_t v, uint8_t d);

/* u8g_u8toa.c */
/* v = value, d = number of digits */
const char *u8g_u16toa(uint16_t v, uint8_t d);

/*===============================================================*/
/* u8g_delay.c */

/* delay by the specified number of milliseconds */
void u8g_Delay(uint16_t val);

/* delay by one microsecond */
void u8g_MicroDelay(void);

/* delay by 10 microseconds */
void u8g_10MicroDelay(void);

/*===============================================================*/
/* chessengine.c */
#define CHESS_KEY_NONE 0
#define CHESS_KEY_NEXT 1
#define CHESS_KEY_PREV 2
#define CHESS_KEY_SELECT 3
#define CHESS_KEY_BACK 4

void chess_Init(u8g_t *u8g, uint8_t empty_body_color);
void chess_Draw(void);
void chess_Step(uint8_t keycode);

/*===============================================================*/
/* font definitions */
extern const u8g_fntpgm_uint8_t u8g_font_m2icon_5[] U8G_FONT_SECTION("u8g_font_m2icon_5");
extern const u8g_fntpgm_uint8_t u8g_font_m2icon_7[] U8G_FONT_SECTION("u8g_font_m2icon_7");
extern const u8g_fntpgm_uint8_t u8g_font_m2icon_9[] U8G_FONT_SECTION("u8g_font_m2icon_9");

extern const u8g_fntpgm_uint8_t u8g_font_u8glib_4[] U8G_FONT_SECTION("u8g_font_u8glib_4");
extern const u8g_fntpgm_uint8_t u8g_font_u8glib_4r[] U8G_FONT_SECTION("u8g_font_u8glib_4r");


extern const u8g_fntpgm_uint8_t u8g_font_6x12_75r[] U8G_FONT_SECTION("u8g_font_6x12_75r");
extern const u8g_fntpgm_uint8_t u8g_font_6x13_75r[] U8G_FONT_SECTION("u8g_font_6x13_75r");
extern const u8g_fntpgm_uint8_t u8g_font_7x13_75r[] U8G_FONT_SECTION("u8g_font_7x13_75r");
extern const u8g_fntpgm_uint8_t u8g_font_8x13_75r[] U8G_FONT_SECTION("u8g_font_8x13_75r");
extern const u8g_fntpgm_uint8_t u8g_font_9x15_75r[] U8G_FONT_SECTION("u8g_font_9x15_75r");
extern const u8g_fntpgm_uint8_t u8g_font_9x18_75r[] U8G_FONT_SECTION("u8g_font_9x18_75r");
extern const u8g_fntpgm_uint8_t u8g_font_cu12_75r[] U8G_FONT_SECTION("u8g_font_cu12_75r");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_75r[] U8G_FONT_SECTION("u8g_font_unifont_75r");
extern const u8g_fntpgm_uint8_t u8g_font_10x20_75r[] U8G_FONT_SECTION("u8g_font_10x20_75r");

extern const u8g_fntpgm_uint8_t u8g_font_10x20_67_75[] U8G_FONT_SECTION("u8g_font_10x20_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_10x20_78_79[] U8G_FONT_SECTION("u8g_font_10x20_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_10x20[] U8G_FONT_SECTION("u8g_font_10x20");
extern const u8g_fntpgm_uint8_t u8g_font_10x20r[] U8G_FONT_SECTION("u8g_font_10x20r");
extern const u8g_fntpgm_uint8_t u8g_font_4x6[] U8G_FONT_SECTION("u8g_font_4x6");
extern const u8g_fntpgm_uint8_t u8g_font_4x6r[] U8G_FONT_SECTION("u8g_font_4x6r");
//extern const u8g_fntpgm_uint8_t u8g_font_4x6n[] U8G_FONT_SECTION("u8g_font_4x6n");
extern const u8g_fntpgm_uint8_t u8g_font_5x7[] U8G_FONT_SECTION("u8g_font_5x7");
extern const u8g_fntpgm_uint8_t u8g_font_5x7r[] U8G_FONT_SECTION("u8g_font_5x7r");
extern const u8g_fntpgm_uint8_t u8g_font_5x8[] U8G_FONT_SECTION("u8g_font_5x8");
extern const u8g_fntpgm_uint8_t u8g_font_5x8r[] U8G_FONT_SECTION("u8g_font_5x8r");
extern const u8g_fntpgm_uint8_t u8g_font_6x10[] U8G_FONT_SECTION("u8g_font_6x10");
extern const u8g_fntpgm_uint8_t u8g_font_6x10r[] U8G_FONT_SECTION("u8g_font_6x10r");
extern const u8g_fntpgm_uint8_t u8g_font_6x12_67_75[] U8G_FONT_SECTION("u8g_font_6x12_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_6x12_78_79[] U8G_FONT_SECTION("u8g_font_6x12_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_6x12[] U8G_FONT_SECTION("u8g_font_6x12");
extern const u8g_fntpgm_uint8_t u8g_font_6x12r[] U8G_FONT_SECTION("u8g_font_6x12r");
extern const u8g_fntpgm_uint8_t u8g_font_6x13_67_75[] U8G_FONT_SECTION("u8g_font_6x13_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_6x13_78_79[] U8G_FONT_SECTION("u8g_font_6x13_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_6x13B[] U8G_FONT_SECTION("u8g_font_6x13B");
extern const u8g_fntpgm_uint8_t u8g_font_6x13Br[] U8G_FONT_SECTION("u8g_font_6x13Br");
extern const u8g_fntpgm_uint8_t u8g_font_6x13[] U8G_FONT_SECTION("u8g_font_6x13");
extern const u8g_fntpgm_uint8_t u8g_font_6x13r[] U8G_FONT_SECTION("u8g_font_6x13r");
extern const u8g_fntpgm_uint8_t u8g_font_6x13O[] U8G_FONT_SECTION("u8g_font_6x13O");
extern const u8g_fntpgm_uint8_t u8g_font_6x13Or[] U8G_FONT_SECTION("u8g_font_6x13Or");
extern const u8g_fntpgm_uint8_t u8g_font_7x13_67_75[] U8G_FONT_SECTION("u8g_font_7x13_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_7x13_78_79[] U8G_FONT_SECTION("u8g_font_7x13_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_7x13B[] U8G_FONT_SECTION("u8g_font_7x13B");
extern const u8g_fntpgm_uint8_t u8g_font_7x13Br[] U8G_FONT_SECTION("u8g_font_7x13Br");
extern const u8g_fntpgm_uint8_t u8g_font_7x13[] U8G_FONT_SECTION("u8g_font_7x13");
extern const u8g_fntpgm_uint8_t u8g_font_7x13r[] U8G_FONT_SECTION("u8g_font_7x13r");
extern const u8g_fntpgm_uint8_t u8g_font_7x13O[] U8G_FONT_SECTION("u8g_font_7x13O");
extern const u8g_fntpgm_uint8_t u8g_font_7x13Or[] U8G_FONT_SECTION("u8g_font_7x13Or");
extern const u8g_fntpgm_uint8_t u8g_font_7x14B[] U8G_FONT_SECTION("u8g_font_7x14B");
extern const u8g_fntpgm_uint8_t u8g_font_7x14Br[] U8G_FONT_SECTION("u8g_font_7x14Br");
extern const u8g_fntpgm_uint8_t u8g_font_7x14[] U8G_FONT_SECTION("u8g_font_7x14");
extern const u8g_fntpgm_uint8_t u8g_font_7x14r[] U8G_FONT_SECTION("u8g_font_7x14r");
extern const u8g_fntpgm_uint8_t u8g_font_8x13_67_75[] U8G_FONT_SECTION("u8g_font_8x13_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_8x13B[] U8G_FONT_SECTION("u8g_font_8x13B");
extern const u8g_fntpgm_uint8_t u8g_font_8x13Br[] U8G_FONT_SECTION("u8g_font_8x13Br");
extern const u8g_fntpgm_uint8_t u8g_font_8x13[] U8G_FONT_SECTION("u8g_font_8x13");
extern const u8g_fntpgm_uint8_t u8g_font_8x13r[] U8G_FONT_SECTION("u8g_font_8x13r");
extern const u8g_fntpgm_uint8_t u8g_font_8x13O[] U8G_FONT_SECTION("u8g_font_8x13O");
extern const u8g_fntpgm_uint8_t u8g_font_8x13Or[] U8G_FONT_SECTION("u8g_font_8x13Or");

extern const u8g_fntpgm_uint8_t u8g_font_9x15_67_75[] U8G_FONT_SECTION("u8g_font_9x15_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_9x15_78_79[] U8G_FONT_SECTION("u8g_font_9x15_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_9x15B[] U8G_FONT_SECTION("u8g_font_9x15B");
extern const u8g_fntpgm_uint8_t u8g_font_9x15Br[] U8G_FONT_SECTION("u8g_font_9x15Br");
extern const u8g_fntpgm_uint8_t u8g_font_9x15[] U8G_FONT_SECTION("u8g_font_9x15");
extern const u8g_fntpgm_uint8_t u8g_font_9x15r[] U8G_FONT_SECTION("u8g_font_9x15r");

extern const u8g_fntpgm_uint8_t u8g_font_9x18_67_75[] U8G_FONT_SECTION("u8g_font_9x18_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_9x18_78_79[] U8G_FONT_SECTION("u8g_font_9x18_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_9x18B[] U8G_FONT_SECTION("u8g_font_9x18B");
extern const u8g_fntpgm_uint8_t u8g_font_9x18[] U8G_FONT_SECTION("u8g_font_9x18");
extern const u8g_fntpgm_uint8_t u8g_font_9x18Br[] U8G_FONT_SECTION("u8g_font_9x18Br");
extern const u8g_fntpgm_uint8_t u8g_font_9x18r[] U8G_FONT_SECTION("u8g_font_9x18r");

extern const u8g_fntpgm_uint8_t u8g_font_cursor[] U8G_FONT_SECTION("u8g_font_cursor");
extern const u8g_fntpgm_uint8_t u8g_font_cursorr[] U8G_FONT_SECTION("u8g_font_cursorr");
extern const u8g_fntpgm_uint8_t u8g_font_micro[] U8G_FONT_SECTION("u8g_font_micro");

extern const u8g_fntpgm_uint8_t u8g_font_cu12_67_75[] U8G_FONT_SECTION("u8g_font_cu12_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_cu12_78_79[] U8G_FONT_SECTION("u8g_font_cu12_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_cu12[] U8G_FONT_SECTION("u8g_font_cu12");

/*
  Free-Universal Bold
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_fub11[] U8G_FONT_SECTION("u8g_font_fub11");
extern const u8g_fntpgm_uint8_t u8g_font_fub11r[] U8G_FONT_SECTION("u8g_font_fub11r");
extern const u8g_fntpgm_uint8_t u8g_font_fub11n[] U8G_FONT_SECTION("u8g_font_fub11n");
extern const u8g_fntpgm_uint8_t u8g_font_fub14[] U8G_FONT_SECTION("u8g_font_fub14");
extern const u8g_fntpgm_uint8_t u8g_font_fub14r[] U8G_FONT_SECTION("u8g_font_fub14r");
extern const u8g_fntpgm_uint8_t u8g_font_fub14n[] U8G_FONT_SECTION("u8g_font_fub14n");
extern const u8g_fntpgm_uint8_t u8g_font_fub17[] U8G_FONT_SECTION("u8g_font_fub17");
extern const u8g_fntpgm_uint8_t u8g_font_fub17r[] U8G_FONT_SECTION("u8g_font_fub17r");
extern const u8g_fntpgm_uint8_t u8g_font_fub17n[] U8G_FONT_SECTION("u8g_font_fub17n");
extern const u8g_fntpgm_uint8_t u8g_font_fub20[] U8G_FONT_SECTION("u8g_font_fub20");
extern const u8g_fntpgm_uint8_t u8g_font_fub20r[] U8G_FONT_SECTION("u8g_font_fub20r");
extern const u8g_fntpgm_uint8_t u8g_font_fub20n[] U8G_FONT_SECTION("u8g_font_fub20n");
extern const u8g_fntpgm_uint8_t u8g_font_fub25[] U8G_FONT_SECTION("u8g_font_fub25");
extern const u8g_fntpgm_uint8_t u8g_font_fub25r[] U8G_FONT_SECTION("u8g_font_fub25r");
extern const u8g_fntpgm_uint8_t u8g_font_fub25n[] U8G_FONT_SECTION("u8g_font_fub25n");
extern const u8g_fntpgm_uint8_t u8g_font_fub30[] U8G_FONT_SECTION("u8g_font_fub30");
extern const u8g_fntpgm_uint8_t u8g_font_fub30r[] U8G_FONT_SECTION("u8g_font_fub30r");
extern const u8g_fntpgm_uint8_t u8g_font_fub30n[] U8G_FONT_SECTION("u8g_font_fub30n");
extern const u8g_fntpgm_uint8_t u8g_font_fub35n[] U8G_FONT_SECTION("u8g_font_fub35n");
extern const u8g_fntpgm_uint8_t u8g_font_fub42n[] U8G_FONT_SECTION("u8g_font_fub42n");
extern const u8g_fntpgm_uint8_t u8g_font_fub49n[] U8G_FONT_SECTION("u8g_font_fub49n");

/*
  Free-Universal Regular
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_fur11[] U8G_FONT_SECTION("u8g_font_fur11");
extern const u8g_fntpgm_uint8_t u8g_font_fur11r[] U8G_FONT_SECTION("u8g_font_fur11r");
extern const u8g_fntpgm_uint8_t u8g_font_fur11n[] U8G_FONT_SECTION("u8g_font_fur11n");
extern const u8g_fntpgm_uint8_t u8g_font_fur14[] U8G_FONT_SECTION("u8g_font_fur14");
extern const u8g_fntpgm_uint8_t u8g_font_fur14r[] U8G_FONT_SECTION("u8g_font_fur14r");
extern const u8g_fntpgm_uint8_t u8g_font_fur14n[] U8G_FONT_SECTION("u8g_font_fur14n");
extern const u8g_fntpgm_uint8_t u8g_font_fur17[] U8G_FONT_SECTION("u8g_font_fur17");
extern const u8g_fntpgm_uint8_t u8g_font_fur17r[] U8G_FONT_SECTION("u8g_font_fur17r");
extern const u8g_fntpgm_uint8_t u8g_font_fur17n[] U8G_FONT_SECTION("u8g_font_fur17n");
extern const u8g_fntpgm_uint8_t u8g_font_fur20[] U8G_FONT_SECTION("u8g_font_fur20");
extern const u8g_fntpgm_uint8_t u8g_font_fur20r[] U8G_FONT_SECTION("u8g_font_fur20r");
extern const u8g_fntpgm_uint8_t u8g_font_fur20n[] U8G_FONT_SECTION("u8g_font_fur20n");
extern const u8g_fntpgm_uint8_t u8g_font_fur25[] U8G_FONT_SECTION("u8g_font_fur25");
extern const u8g_fntpgm_uint8_t u8g_font_fur25r[] U8G_FONT_SECTION("u8g_font_fur25r");
extern const u8g_fntpgm_uint8_t u8g_font_fur25n[] U8G_FONT_SECTION("u8g_font_fur25n");
extern const u8g_fntpgm_uint8_t u8g_font_fur30[] U8G_FONT_SECTION("u8g_font_fur30");
extern const u8g_fntpgm_uint8_t u8g_font_fur30r[] U8G_FONT_SECTION("u8g_font_fur30r");
extern const u8g_fntpgm_uint8_t u8g_font_fur30n[] U8G_FONT_SECTION("u8g_font_fur30n");
extern const u8g_fntpgm_uint8_t u8g_font_fur35n[] U8G_FONT_SECTION("u8g_font_fur35n");
extern const u8g_fntpgm_uint8_t u8g_font_fur42n[] U8G_FONT_SECTION("u8g_font_fur42n");
extern const u8g_fntpgm_uint8_t u8g_font_fur49n[] U8G_FONT_SECTION("u8g_font_fur49n");

/*
  Gentium Bold
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_gdb11[] U8G_FONT_SECTION("u8g_font_gdb11");
extern const u8g_fntpgm_uint8_t u8g_font_gdb12[] U8G_FONT_SECTION("u8g_font_gdb12");
extern const u8g_fntpgm_uint8_t u8g_font_gdb14[] U8G_FONT_SECTION("u8g_font_gdb14");
extern const u8g_fntpgm_uint8_t u8g_font_gdb17[] U8G_FONT_SECTION("u8g_font_gdb17");
extern const u8g_fntpgm_uint8_t u8g_font_gdb20[] U8G_FONT_SECTION("u8g_font_gdb20");
extern const u8g_fntpgm_uint8_t u8g_font_gdb25[] U8G_FONT_SECTION("u8g_font_gdb25");
extern const u8g_fntpgm_uint8_t u8g_font_gdb30[] U8G_FONT_SECTION("u8g_font_gdb30");

extern const u8g_fntpgm_uint8_t u8g_font_gdb11r[] U8G_FONT_SECTION("u8g_font_gdb11r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb12r[] U8G_FONT_SECTION("u8g_font_gdb12r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb14r[] U8G_FONT_SECTION("u8g_font_gdb14r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb17r[] U8G_FONT_SECTION("u8g_font_gdb17r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb20r[] U8G_FONT_SECTION("u8g_font_gdb20r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb25r[] U8G_FONT_SECTION("u8g_font_gdb25r");
extern const u8g_fntpgm_uint8_t u8g_font_gdb30r[] U8G_FONT_SECTION("u8g_font_gdb30r");

extern const u8g_fntpgm_uint8_t u8g_font_gdb11n[] U8G_FONT_SECTION("u8g_font_gdb11n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb12n[] U8G_FONT_SECTION("u8g_font_gdb12n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb14n[] U8G_FONT_SECTION("u8g_font_gdb14n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb17n[] U8G_FONT_SECTION("u8g_font_gdb17n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb20n[] U8G_FONT_SECTION("u8g_font_gdb20n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb25n[] U8G_FONT_SECTION("u8g_font_gdb25n");
extern const u8g_fntpgm_uint8_t u8g_font_gdb30n[] U8G_FONT_SECTION("u8g_font_gdb30n");

/*
  Gentium Regular
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_gdr9[] U8G_FONT_SECTION("u8g_font_gdr9");
extern const u8g_fntpgm_uint8_t u8g_font_gdr10[] U8G_FONT_SECTION("u8g_font_gdr10");
extern const u8g_fntpgm_uint8_t u8g_font_gdr11[] U8G_FONT_SECTION("u8g_font_gdr11");
extern const u8g_fntpgm_uint8_t u8g_font_gdr12[] U8G_FONT_SECTION("u8g_font_gdr12");
extern const u8g_fntpgm_uint8_t u8g_font_gdr14[] U8G_FONT_SECTION("u8g_font_gdr14");
extern const u8g_fntpgm_uint8_t u8g_font_gdr17[] U8G_FONT_SECTION("u8g_font_gdr17");
extern const u8g_fntpgm_uint8_t u8g_font_gdr20[] U8G_FONT_SECTION("u8g_font_gdr20");
extern const u8g_fntpgm_uint8_t u8g_font_gdr25[] U8G_FONT_SECTION("u8g_font_gdr25");
extern const u8g_fntpgm_uint8_t u8g_font_gdr30[] U8G_FONT_SECTION("u8g_font_gdr30");

extern const u8g_fntpgm_uint8_t u8g_font_gdr9r[] U8G_FONT_SECTION("u8g_font_gdr9r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr10r[] U8G_FONT_SECTION("u8g_font_gdr10r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr11r[] U8G_FONT_SECTION("u8g_font_gdr11r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr12r[] U8G_FONT_SECTION("u8g_font_gdr12r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr14r[] U8G_FONT_SECTION("u8g_font_gdr14r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr17r[] U8G_FONT_SECTION("u8g_font_gdr17r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr20r[] U8G_FONT_SECTION("u8g_font_gdr20r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr25r[] U8G_FONT_SECTION("u8g_font_gdr25r");
extern const u8g_fntpgm_uint8_t u8g_font_gdr30r[] U8G_FONT_SECTION("u8g_font_gdr30r");

extern const u8g_fntpgm_uint8_t u8g_font_gdr9n[] U8G_FONT_SECTION("u8g_font_gdr9n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr10n[] U8G_FONT_SECTION("u8g_font_gdr10n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr11n[] U8G_FONT_SECTION("u8g_font_gdr11n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr12n[] U8G_FONT_SECTION("u8g_font_gdr12n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr14n[] U8G_FONT_SECTION("u8g_font_gdr14n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr17n[] U8G_FONT_SECTION("u8g_font_gdr17n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr20n[] U8G_FONT_SECTION("u8g_font_gdr20n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr25n[] U8G_FONT_SECTION("u8g_font_gdr25n");
extern const u8g_fntpgm_uint8_t u8g_font_gdr30n[] U8G_FONT_SECTION("u8g_font_gdr30n");

/*
  Old-Standard Bold
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_osb18[] U8G_FONT_SECTION("u8g_font_osb18");
extern const u8g_fntpgm_uint8_t u8g_font_osb21[] U8G_FONT_SECTION("u8g_font_osb21");
extern const u8g_fntpgm_uint8_t u8g_font_osb26[] U8G_FONT_SECTION("u8g_font_osb26");
extern const u8g_fntpgm_uint8_t u8g_font_osb29[] U8G_FONT_SECTION("u8g_font_osb29");
extern const u8g_fntpgm_uint8_t u8g_font_osb35[] U8G_FONT_SECTION("u8g_font_osb35");

extern const u8g_fntpgm_uint8_t u8g_font_osb18r[] U8G_FONT_SECTION("u8g_font_osb18r");
extern const u8g_fntpgm_uint8_t u8g_font_osb21r[] U8G_FONT_SECTION("u8g_font_osb21r");
extern const u8g_fntpgm_uint8_t u8g_font_osb26r[] U8G_FONT_SECTION("u8g_font_osb26r");
extern const u8g_fntpgm_uint8_t u8g_font_osb29r[] U8G_FONT_SECTION("u8g_font_osb29r");
extern const u8g_fntpgm_uint8_t u8g_font_osb35r[] U8G_FONT_SECTION("u8g_font_osb35r");

extern const u8g_fntpgm_uint8_t u8g_font_osb18n[] U8G_FONT_SECTION("u8g_font_osb18n");
extern const u8g_fntpgm_uint8_t u8g_font_osb21n[] U8G_FONT_SECTION("u8g_font_osb21n");
extern const u8g_fntpgm_uint8_t u8g_font_osb26n[] U8G_FONT_SECTION("u8g_font_osb26n");
extern const u8g_fntpgm_uint8_t u8g_font_osb29n[] U8G_FONT_SECTION("u8g_font_osb29n");
extern const u8g_fntpgm_uint8_t u8g_font_osb35n[] U8G_FONT_SECTION("u8g_font_osb35n");

/*
  Old-Standard Regular
  r: Reduced char set (codes 32 - 128)
  n: Numbers (codes 42 - 57)
  no char: Full set (codes 32 - 255)
*/

extern const u8g_fntpgm_uint8_t u8g_font_osr18[] U8G_FONT_SECTION("u8g_font_osr18");
extern const u8g_fntpgm_uint8_t u8g_font_osr21[] U8G_FONT_SECTION("u8g_font_osr21");
extern const u8g_fntpgm_uint8_t u8g_font_osr26[] U8G_FONT_SECTION("u8g_font_osr26");
extern const u8g_fntpgm_uint8_t u8g_font_osr29[] U8G_FONT_SECTION("u8g_font_osr29");
extern const u8g_fntpgm_uint8_t u8g_font_osr35[] U8G_FONT_SECTION("u8g_font_osr35");

extern const u8g_fntpgm_uint8_t u8g_font_osr18r[] U8G_FONT_SECTION("u8g_font_osr18r");
extern const u8g_fntpgm_uint8_t u8g_font_osr21r[] U8G_FONT_SECTION("u8g_font_osr21r");
extern const u8g_fntpgm_uint8_t u8g_font_osr26r[] U8G_FONT_SECTION("u8g_font_osr26r");
extern const u8g_fntpgm_uint8_t u8g_font_osr29r[] U8G_FONT_SECTION("u8g_font_osr29r");
extern const u8g_fntpgm_uint8_t u8g_font_osr35r[] U8G_FONT_SECTION("u8g_font_osr35r");

extern const u8g_fntpgm_uint8_t u8g_font_osr18n[] U8G_FONT_SECTION("u8g_font_osr18n");
extern const u8g_fntpgm_uint8_t u8g_font_osr21n[] U8G_FONT_SECTION("u8g_font_osr21n");
extern const u8g_fntpgm_uint8_t u8g_font_osr26n[] U8G_FONT_SECTION("u8g_font_osr26n");
extern const u8g_fntpgm_uint8_t u8g_font_osr29n[] U8G_FONT_SECTION("u8g_font_osr29n");
extern const u8g_fntpgm_uint8_t u8g_font_osr35n[] U8G_FONT_SECTION("u8g_font_osr35n");

//extern const u8g_fntpgm_uint8_t u8g_font_osr41[] U8G_FONT_SECTION("u8g_font_osr41");

/* GNU unifont */

extern const u8g_fntpgm_uint8_t u8g_font_unifont_18_19[] U8G_FONT_SECTION("u8g_font_unifont_18_19");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_72_73[] U8G_FONT_SECTION("u8g_font_unifont_72_73");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_67_75[] U8G_FONT_SECTION("u8g_font_unifont_67_75");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_76[] U8G_FONT_SECTION("u8g_font_unifont_76");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_77[] U8G_FONT_SECTION("u8g_font_unifont_77");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_78_79[] U8G_FONT_SECTION("u8g_font_unifont_78_79");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_86[] U8G_FONT_SECTION("u8g_font_unifont_86");
extern const u8g_fntpgm_uint8_t u8g_font_unifont[] U8G_FONT_SECTION("u8g_font_unifont");
extern const u8g_fntpgm_uint8_t u8g_font_unifontr[] U8G_FONT_SECTION("u8g_font_unifontr");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_0_8[] U8G_FONT_SECTION("u8g_font_unifont_0_8");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_2_3[] U8G_FONT_SECTION("u8g_font_unifont_2_3");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_4_5[] U8G_FONT_SECTION("u8g_font_unifont_4_5");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_8_9[] U8G_FONT_SECTION("u8g_font_unifont_8_9");
extern const u8g_fntpgm_uint8_t u8g_font_unifont_12_13[] U8G_FONT_SECTION("u8g_font_unifont_12_13");


/* 04b fonts */

extern const u8g_fntpgm_uint8_t u8g_font_04b_03b[] U8G_FONT_SECTION("u8g_font_04b_03b");
extern const u8g_fntpgm_uint8_t u8g_font_04b_03bn[] U8G_FONT_SECTION("u8g_font_04b_03bn");
extern const u8g_fntpgm_uint8_t u8g_font_04b_03br[] U8G_FONT_SECTION("u8g_font_04b_03br");
extern const u8g_fntpgm_uint8_t u8g_font_04b_03[] U8G_FONT_SECTION("u8g_font_04b_03");
extern const u8g_fntpgm_uint8_t u8g_font_04b_03n[] U8G_FONT_SECTION("u8g_font_04b_03n");
extern const u8g_fntpgm_uint8_t u8g_font_04b_03r[] U8G_FONT_SECTION("u8g_font_04b_03r");
extern const u8g_fntpgm_uint8_t u8g_font_04b_24[] U8G_FONT_SECTION("u8g_font_04b_24");
extern const u8g_fntpgm_uint8_t u8g_font_04b_24n[] U8G_FONT_SECTION("u8g_font_04b_24n");
extern const u8g_fntpgm_uint8_t u8g_font_04b_24r[] U8G_FONT_SECTION("u8g_font_04b_24r");

/* orgdot fonts */

extern const u8g_fntpgm_uint8_t u8g_font_orgv01[] U8G_FONT_SECTION("u8g_font_orgv01");
extern const u8g_fntpgm_uint8_t u8g_font_orgv01r[] U8G_FONT_SECTION("u8g_font_orgv01r");
extern const u8g_fntpgm_uint8_t u8g_font_orgv01n[] U8G_FONT_SECTION("u8g_font_orgv01n");

extern const u8g_fntpgm_uint8_t u8g_font_fixed_v0[] U8G_FONT_SECTION("u8g_font_fixed_v0");
extern const u8g_fntpgm_uint8_t u8g_font_fixed_v0r[] U8G_FONT_SECTION("u8g_font_fixed_v0r");
extern const u8g_fntpgm_uint8_t u8g_font_fixed_v0n[] U8G_FONT_SECTION("u8g_font_fixed_v0n");

extern const u8g_fntpgm_uint8_t u8g_font_tpssb[] U8G_FONT_SECTION("u8g_font_tpssb");
extern const u8g_fntpgm_uint8_t u8g_font_tpssbr[] U8G_FONT_SECTION("u8g_font_tpssbr");
extern const u8g_fntpgm_uint8_t u8g_font_tpssbn[] U8G_FONT_SECTION("u8g_font_tpssbn");

extern const u8g_fntpgm_uint8_t u8g_font_tpss[] U8G_FONT_SECTION("u8g_font_tpss");
extern const u8g_fntpgm_uint8_t u8g_font_tpssr[] U8G_FONT_SECTION("u8g_font_tpssr");
extern const u8g_fntpgm_uint8_t u8g_font_tpssn[] U8G_FONT_SECTION("u8g_font_tpssn");

/* contributed */

extern const u8g_fntpgm_uint8_t u8g_font_freedoomr25n[] U8G_FONT_SECTION("u8g_font_freedoomr25n");
extern const u8g_fntpgm_uint8_t u8g_font_freedoomr10r[] U8G_FONT_SECTION("u8g_font_freedoomr10r");

/* adobe X11 */
extern const u8g_fntpgm_uint8_t u8g_font_courB08[] U8G_FONT_SECTION("u8g_font_courB08");
extern const u8g_fntpgm_uint8_t u8g_font_courB08r[] U8G_FONT_SECTION("u8g_font_courB08r");
extern const u8g_fntpgm_uint8_t u8g_font_courB10[] U8G_FONT_SECTION("u8g_font_courB10");
extern const u8g_fntpgm_uint8_t u8g_font_courB10r[] U8G_FONT_SECTION("u8g_font_courB10r");
extern const u8g_fntpgm_uint8_t u8g_font_courB12[] U8G_FONT_SECTION("u8g_font_courB12");
extern const u8g_fntpgm_uint8_t u8g_font_courB12r[] U8G_FONT_SECTION("u8g_font_courB12r");
extern const u8g_fntpgm_uint8_t u8g_font_courB14[] U8G_FONT_SECTION("u8g_font_courB14");
extern const u8g_fntpgm_uint8_t u8g_font_courB14r[] U8G_FONT_SECTION("u8g_font_courB14r");
extern const u8g_fntpgm_uint8_t u8g_font_courB18[] U8G_FONT_SECTION("u8g_font_courB18");
extern const u8g_fntpgm_uint8_t u8g_font_courB18r[] U8G_FONT_SECTION("u8g_font_courB18r");
extern const u8g_fntpgm_uint8_t u8g_font_courB24[] U8G_FONT_SECTION("u8g_font_courB24");
extern const u8g_fntpgm_uint8_t u8g_font_courB24r[] U8G_FONT_SECTION("u8g_font_courB24r");
extern const u8g_fntpgm_uint8_t u8g_font_courB24n[] U8G_FONT_SECTION("u8g_font_courB24n");

extern const u8g_fntpgm_uint8_t u8g_font_courR08[] U8G_FONT_SECTION("u8g_font_courR08");
extern const u8g_fntpgm_uint8_t u8g_font_courR08r[] U8G_FONT_SECTION("u8g_font_courR08r");
extern const u8g_fntpgm_uint8_t u8g_font_courR10[] U8G_FONT_SECTION("u8g_font_courR10");
extern const u8g_fntpgm_uint8_t u8g_font_courR10r[] U8G_FONT_SECTION("u8g_font_courR10r");
extern const u8g_fntpgm_uint8_t u8g_font_courR12[] U8G_FONT_SECTION("u8g_font_courR12");
extern const u8g_fntpgm_uint8_t u8g_font_courR12r[] U8G_FONT_SECTION("u8g_font_courR12r");
extern const u8g_fntpgm_uint8_t u8g_font_courR14[] U8G_FONT_SECTION("u8g_font_courR14");
extern const u8g_fntpgm_uint8_t u8g_font_courR14r[] U8G_FONT_SECTION("u8g_font_courR14r");
extern const u8g_fntpgm_uint8_t u8g_font_courR18[] U8G_FONT_SECTION("u8g_font_courR18");
extern const u8g_fntpgm_uint8_t u8g_font_courR18r[] U8G_FONT_SECTION("u8g_font_courR18r");
extern const u8g_fntpgm_uint8_t u8g_font_courR24[] U8G_FONT_SECTION("u8g_font_courR24");
extern const u8g_fntpgm_uint8_t u8g_font_courR24r[] U8G_FONT_SECTION("u8g_font_courR24r");
extern const u8g_fntpgm_uint8_t u8g_font_courR24n[] U8G_FONT_SECTION("u8g_font_courR24n");

extern const u8g_fntpgm_uint8_t u8g_font_helvB08[] U8G_FONT_SECTION("u8g_font_helvB08");
extern const u8g_fntpgm_uint8_t u8g_font_helvB08r[] U8G_FONT_SECTION("u8g_font_helvB08r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB10[] U8G_FONT_SECTION("u8g_font_helvB10");
extern const u8g_fntpgm_uint8_t u8g_font_helvB10r[] U8G_FONT_SECTION("u8g_font_helvB10r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB12[] U8G_FONT_SECTION("u8g_font_helvB12");
extern const u8g_fntpgm_uint8_t u8g_font_helvB12r[] U8G_FONT_SECTION("u8g_font_helvB12r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB14[] U8G_FONT_SECTION("u8g_font_helvB14");
extern const u8g_fntpgm_uint8_t u8g_font_helvB14r[] U8G_FONT_SECTION("u8g_font_helvB14r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB18[] U8G_FONT_SECTION("u8g_font_helvB18");
extern const u8g_fntpgm_uint8_t u8g_font_helvB18r[] U8G_FONT_SECTION("u8g_font_helvB18r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB24[] U8G_FONT_SECTION("u8g_font_helvB24");
extern const u8g_fntpgm_uint8_t u8g_font_helvB24r[] U8G_FONT_SECTION("u8g_font_helvB24r");
extern const u8g_fntpgm_uint8_t u8g_font_helvB24n[] U8G_FONT_SECTION("u8g_font_helvB24n");

extern const u8g_fntpgm_uint8_t u8g_font_helvR08[] U8G_FONT_SECTION("u8g_font_helvR08");
extern const u8g_fntpgm_uint8_t u8g_font_helvR08r[] U8G_FONT_SECTION("u8g_font_helvR08r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR10[] U8G_FONT_SECTION("u8g_font_helvR10");
extern const u8g_fntpgm_uint8_t u8g_font_helvR10r[] U8G_FONT_SECTION("u8g_font_helvR10r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR12[] U8G_FONT_SECTION("u8g_font_helvR12");
extern const u8g_fntpgm_uint8_t u8g_font_helvR12r[] U8G_FONT_SECTION("u8g_font_helvR12r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR14[] U8G_FONT_SECTION("u8g_font_helvR14");
extern const u8g_fntpgm_uint8_t u8g_font_helvR14r[] U8G_FONT_SECTION("u8g_font_helvR14r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR18[] U8G_FONT_SECTION("u8g_font_helvR18");
extern const u8g_fntpgm_uint8_t u8g_font_helvR18r[] U8G_FONT_SECTION("u8g_font_helvR18r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR24[] U8G_FONT_SECTION("u8g_font_helvR24");
extern const u8g_fntpgm_uint8_t u8g_font_helvR24r[] U8G_FONT_SECTION("u8g_font_helvR24r");
extern const u8g_fntpgm_uint8_t u8g_font_helvR24n[] U8G_FONT_SECTION("u8g_font_helvR24n");

extern const u8g_fntpgm_uint8_t u8g_font_ncenB08[] U8G_FONT_SECTION("u8g_font_ncenB08");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB08r[] U8G_FONT_SECTION("u8g_font_ncenB08r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB10[] U8G_FONT_SECTION("u8g_font_ncenB10");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB10r[] U8G_FONT_SECTION("u8g_font_ncenB10r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB12[] U8G_FONT_SECTION("u8g_font_ncenB12");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB12r[] U8G_FONT_SECTION("u8g_font_ncenB12r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB14[] U8G_FONT_SECTION("u8g_font_ncenB14");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB14r[] U8G_FONT_SECTION("u8g_font_ncenB14r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB18[] U8G_FONT_SECTION("u8g_font_ncenB18");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB18r[] U8G_FONT_SECTION("u8g_font_ncenB18r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB24[] U8G_FONT_SECTION("u8g_font_ncenB24");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB24r[] U8G_FONT_SECTION("u8g_font_ncenB24r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenB24n[] U8G_FONT_SECTION("u8g_font_ncenB24n");

extern const u8g_fntpgm_uint8_t u8g_font_ncenR08[] U8G_FONT_SECTION("u8g_font_ncenR08");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR08r[] U8G_FONT_SECTION("u8g_font_ncenR08r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR10[] U8G_FONT_SECTION("u8g_font_ncenR10");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR10r[] U8G_FONT_SECTION("u8g_font_ncenR10r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR12[] U8G_FONT_SECTION("u8g_font_ncenR12");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR12r[] U8G_FONT_SECTION("u8g_font_ncenR12r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR14[] U8G_FONT_SECTION("u8g_font_ncenR14");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR14r[] U8G_FONT_SECTION("u8g_font_ncenR14r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR18[] U8G_FONT_SECTION("u8g_font_ncenR18");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR18r[] U8G_FONT_SECTION("u8g_font_ncenR18r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR24[] U8G_FONT_SECTION("u8g_font_ncenR24");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR24r[] U8G_FONT_SECTION("u8g_font_ncenR24r");
extern const u8g_fntpgm_uint8_t u8g_font_ncenR24n[] U8G_FONT_SECTION("u8g_font_ncenR24n");

extern const u8g_fntpgm_uint8_t u8g_font_symb08[] U8G_FONT_SECTION("u8g_font_symb08");
extern const u8g_fntpgm_uint8_t u8g_font_symb08r[] U8G_FONT_SECTION("u8g_font_symb08r");
extern const u8g_fntpgm_uint8_t u8g_font_symb10[] U8G_FONT_SECTION("u8g_font_symb10");
extern const u8g_fntpgm_uint8_t u8g_font_symb10r[] U8G_FONT_SECTION("u8g_font_symb10r");
extern const u8g_fntpgm_uint8_t u8g_font_symb12[] U8G_FONT_SECTION("u8g_font_symb12");
extern const u8g_fntpgm_uint8_t u8g_font_symb12r[] U8G_FONT_SECTION("u8g_font_symb12r");
extern const u8g_fntpgm_uint8_t u8g_font_symb14[] U8G_FONT_SECTION("u8g_font_symb14");
extern const u8g_fntpgm_uint8_t u8g_font_symb14r[] U8G_FONT_SECTION("u8g_font_symb14r");
extern const u8g_fntpgm_uint8_t u8g_font_symb18[] U8G_FONT_SECTION("u8g_font_symb18");
extern const u8g_fntpgm_uint8_t u8g_font_symb18r[] U8G_FONT_SECTION("u8g_font_symb18r");
extern const u8g_fntpgm_uint8_t u8g_font_symb24[] U8G_FONT_SECTION("u8g_font_symb24");
extern const u8g_fntpgm_uint8_t u8g_font_symb24r[] U8G_FONT_SECTION("u8g_font_symb24r");

extern const u8g_fntpgm_uint8_t u8g_font_timB08[] U8G_FONT_SECTION("u8g_font_timB08");
extern const u8g_fntpgm_uint8_t u8g_font_timB08r[] U8G_FONT_SECTION("u8g_font_timB08r");
extern const u8g_fntpgm_uint8_t u8g_font_timB10[] U8G_FONT_SECTION("u8g_font_timB10");
extern const u8g_fntpgm_uint8_t u8g_font_timB10r[] U8G_FONT_SECTION("u8g_font_timB10r");
extern const u8g_fntpgm_uint8_t u8g_font_timB12[] U8G_FONT_SECTION("u8g_font_timB12");
extern const u8g_fntpgm_uint8_t u8g_font_timB12r[] U8G_FONT_SECTION("u8g_font_timB12r");
extern const u8g_fntpgm_uint8_t u8g_font_timB14[] U8G_FONT_SECTION("u8g_font_timB14");
extern const u8g_fntpgm_uint8_t u8g_font_timB14r[] U8G_FONT_SECTION("u8g_font_timB14r");
extern const u8g_fntpgm_uint8_t u8g_font_timB18[] U8G_FONT_SECTION("u8g_font_timB18");
extern const u8g_fntpgm_uint8_t u8g_font_timB18r[] U8G_FONT_SECTION("u8g_font_timB18r");
extern const u8g_fntpgm_uint8_t u8g_font_timB24[] U8G_FONT_SECTION("u8g_font_timB24");
extern const u8g_fntpgm_uint8_t u8g_font_timB24r[] U8G_FONT_SECTION("u8g_font_timB24r");
extern const u8g_fntpgm_uint8_t u8g_font_timB24n[] U8G_FONT_SECTION("u8g_font_timB24n");

extern const u8g_fntpgm_uint8_t u8g_font_timR08[] U8G_FONT_SECTION("u8g_font_timR08");
extern const u8g_fntpgm_uint8_t u8g_font_timR08r[] U8G_FONT_SECTION("u8g_font_timR08r");
extern const u8g_fntpgm_uint8_t u8g_font_timR10[] U8G_FONT_SECTION("u8g_font_timR10");
extern const u8g_fntpgm_uint8_t u8g_font_timR10r[] U8G_FONT_SECTION("u8g_font_timR10r");
extern const u8g_fntpgm_uint8_t u8g_font_timR12[] U8G_FONT_SECTION("u8g_font_timR12");
extern const u8g_fntpgm_uint8_t u8g_font_timR12r[] U8G_FONT_SECTION("u8g_font_timR12r");
extern const u8g_fntpgm_uint8_t u8g_font_timR14[] U8G_FONT_SECTION("u8g_font_timR14");
extern const u8g_fntpgm_uint8_t u8g_font_timR14r[] U8G_FONT_SECTION("u8g_font_timR14r");
extern const u8g_fntpgm_uint8_t u8g_font_timR18[] U8G_FONT_SECTION("u8g_font_timR18");
extern const u8g_fntpgm_uint8_t u8g_font_timR18r[] U8G_FONT_SECTION("u8g_font_timR18r");
extern const u8g_fntpgm_uint8_t u8g_font_timR24[] U8G_FONT_SECTION("u8g_font_timR24");
extern const u8g_fntpgm_uint8_t u8g_font_timR24r[] U8G_FONT_SECTION("u8g_font_timR24r");
extern const u8g_fntpgm_uint8_t u8g_font_timR24n[] U8G_FONT_SECTION("u8g_font_timR24n");

/* fontstruct */

extern const u8g_fntpgm_uint8_t u8g_font_p01type[] U8G_FONT_SECTION("u8g_font_p01type");
extern const u8g_fntpgm_uint8_t u8g_font_p01typer[] U8G_FONT_SECTION("u8g_font_p01typer");
extern const u8g_fntpgm_uint8_t u8g_font_p01typen[] U8G_FONT_SECTION("u8g_font_p01typen");

extern const u8g_fntpgm_uint8_t u8g_font_lucasfont_alternate[] U8G_FONT_SECTION("u8g_font_lucasfont_alternate");
extern const u8g_fntpgm_uint8_t u8g_font_lucasfont_alternater[] U8G_FONT_SECTION("u8g_font_lucasfont_alternater");
extern const u8g_fntpgm_uint8_t u8g_font_lucasfont_alternaten[] U8G_FONT_SECTION("u8g_font_lucasfont_alternaten");

extern const u8g_fntpgm_uint8_t u8g_font_chikita[] U8G_FONT_SECTION("u8g_font_chikita");
extern const u8g_fntpgm_uint8_t u8g_font_chikitar[] U8G_FONT_SECTION("u8g_font_chikitar");
extern const u8g_fntpgm_uint8_t u8g_font_chikitan[] U8G_FONT_SECTION("u8g_font_chikitan");

extern const u8g_fntpgm_uint8_t u8g_font_pixelle_micro[] U8G_FONT_SECTION("u8g_font_pixelle_micro");
extern const u8g_fntpgm_uint8_t u8g_font_pixelle_micror[] U8G_FONT_SECTION("u8g_font_pixelle_micror");
extern const u8g_fntpgm_uint8_t u8g_font_pixelle_micron[] U8G_FONT_SECTION("u8g_font_pixelle_micron");

extern const u8g_fntpgm_uint8_t u8g_font_trixel_square[] U8G_FONT_SECTION("u8g_font_trixel_square");
extern const u8g_fntpgm_uint8_t u8g_font_trixel_squarer[] U8G_FONT_SECTION("u8g_font_trixel_squarer");
extern const u8g_fntpgm_uint8_t u8g_font_trixel_squaren[] U8G_FONT_SECTION("u8g_font_trixel_squaren");

extern const u8g_fntpgm_uint8_t u8g_font_robot_de_niro[] U8G_FONT_SECTION("u8g_font_robot_de_niro");
extern const u8g_fntpgm_uint8_t u8g_font_robot_de_niror[] U8G_FONT_SECTION("u8g_font_robot_de_niror");
extern const u8g_fntpgm_uint8_t u8g_font_robot_de_niron[] U8G_FONT_SECTION("u8g_font_robot_de_niron");

extern const u8g_fntpgm_uint8_t u8g_font_baby[] U8G_FONT_SECTION("u8g_font_baby");
extern const u8g_fntpgm_uint8_t u8g_font_babyr[] U8G_FONT_SECTION("u8g_font_babyr");
extern const u8g_fntpgm_uint8_t u8g_font_babyn[] U8G_FONT_SECTION("u8g_font_babyn");

extern const u8g_fntpgm_uint8_t u8g_font_blipfest_07[] U8G_FONT_SECTION("u8g_font_blipfest_07");
extern const u8g_fntpgm_uint8_t u8g_font_blipfest_07r[] U8G_FONT_SECTION("u8g_font_blipfest_07r");
extern const u8g_fntpgm_uint8_t u8g_font_blipfest_07n[] U8G_FONT_SECTION("u8g_font_blipfest_07n");



#ifdef __cplusplus
}
#endif

#endif /* _U8G_H */

/*
  Fontname: -Misc-Fixed-Medium-R-Normal--7-70-75-75-C-50-ISO10646-1
  Copyright: Public domain font.  Share and enjoy.
  Capital A Height: 6, '1' Height: 6
  Calculated Max Values w= 7 h= 8 x= 4 y= 5 dx= 9 dy= 0 ascent= 8 len= 8
  Font Bounding box     w= 5 h= 7 x= 0 y=-1
  Calculated Min Values           x= 0 y=-1 dx= 0 dy= 0
  Pure Font   ascent = 6 descent=-1
  X Font      ascent = 6 descent=-1
  Max Font    ascent = 8 descent=-1
*/
const u8g_fntpgm_uint8_t repetier_5x7[2405] U8G_FONT_SECTION("repetier_5x7") = {
  0,5,7,0,255,6,2,72,3,186,0,255,255,8,255,6,
  255,5,5,5,5,0,0,168,0,136,0,168,5,8,8,6,
  0,0,8,8,8,40,72,248,64,32,3,3,3,6,1,4,
  64,160,64,5,5,5,6,0,0,248,248,248,248,248,5,5,
  5,6,0,0,248,136,136,136,248,5,8,8,6,0,0,32,
  80,80,80,112,248,248,112,7,5,5,9,0,0,28,254,130,
  130,254,5,8,8,6,0,0,136,80,32,136,32,80,136,136,
  6,8,8,6,0,0,72,144,72,252,252,120,48,48,6,8,
  8,6,0,0,36,72,36,252,252,120,48,48,6,5,5,6,
  0,0,252,252,120,48,48,6,2,2,6,0,0,252,252,6,
  6,6,6,0,0,72,144,72,144,252,252,6,6,6,6,0,
  0,36,72,36,72,252,252,7,7,7,7,0,0,130,68,56,
  40,56,68,130,7,7,7,7,0,0,16,16,56,238,56,16,
  16,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,0,0,0,5,4,0,1,6,6,5,2,0,128,128,128,
  128,0,128,3,3,3,5,1,3,160,160,160,5,5,5,5,
  0,0,80,248,80,248,80,5,5,5,5,0,0,112,160,112,
  40,112,4,6,6,5,0,0,128,144,32,64,144,16,4,5,
  5,5,0,0,64,160,64,160,80,1,3,3,5,2,3,128,
  128,128,2,6,6,5,1,0,64,128,128,128,128,64,2,6,
  6,5,1,0,128,64,64,64,64,128,3,5,5,5,1,0,
  160,64,224,64,160,5,5,5,5,0,0,32,32,248,32,32,
  3,3,3,5,1,255,96,64,128,4,1,1,5,0,2,240,
  2,2,2,5,1,0,192,192,4,4,4,5,0,1,16,32,
  64,128,3,6,6,5,1,0,64,160,160,160,160,64,3,6,
  6,5,1,0,64,192,64,64,64,224,4,6,6,5,0,0,
  96,144,16,32,64,240,4,6,6,5,0,0,240,16,96,16,
  144,96,4,6,6,5,0,0,32,96,160,240,32,32,4,6,
  6,5,0,0,240,128,224,16,144,96,4,6,6,5,0,0,
  96,128,224,144,144,96,4,6,6,5,0,0,240,16,32,32,
  64,64,4,6,6,5,0,0,96,144,96,144,144,96,4,6,
  6,5,0,0,96,144,144,112,16,96,2,5,5,5,1,0,
  192,192,0,192,192,3,6,6,5,0,255,96,96,0,96,64,
  128,3,5,5,5,1,0,32,64,128,64,32,4,3,3,5,
  0,1,240,0,240,3,5,5,5,1,0,128,64,32,64,128,
  3,6,6,5,1,0,64,160,32,64,0,64,4,6,6,5,
  0,0,96,144,176,176,128,96,4,6,6,5,0,0,96,144,
  144,240,144,144,4,6,6,5,0,0,224,144,224,144,144,224,
  4,6,6,5,0,0,96,144,128,128,144,96,4,6,6,5,
  0,0,224,144,144,144,144,224,4,6,6,5,0,0,240,128,
  224,128,128,240,4,6,6,5,0,0,240,128,224,128,128,128,
  4,6,6,5,0,0,96,144,128,176,144,112,4,6,6,5,
  0,0,144,144,240,144,144,144,3,6,6,5,1,0,224,64,
  64,64,64,224,4,6,6,5,0,0,16,16,16,16,144,96,
  4,6,6,5,0,0,144,160,192,192,160,144,4,6,6,5,
  0,0,128,128,128,128,128,240,4,6,6,5,0,0,144,240,
  240,144,144,144,4,6,6,5,0,0,144,208,208,176,176,144,
  4,6,6,5,0,0,96,144,144,144,144,96,4,6,6,5,
  0,0,224,144,144,224,128,128,4,7,7,5,0,255,96,144,
  144,144,208,96,16,4,6,6,5,0,0,224,144,144,224,160,
  144,4,6,6,5,0,0,96,144,64,32,144,96,3,6,6,
  5,1,0,224,64,64,64,64,64,4,6,6,5,0,0,144,
  144,144,144,144,96,4,6,6,5,0,0,144,144,144,144,96,
  96,4,6,6,5,0,0,144,144,144,240,240,144,4,6,6,
  5,0,0,144,144,96,96,144,144,3,6,6,5,1,0,160,
  160,160,64,64,64,4,6,6,5,0,0,240,16,32,64,128,
  240,3,6,6,5,1,0,224,128,128,128,128,224,4,4,4,
  5,0,1,128,64,32,16,3,6,6,5,1,0,224,32,32,
  32,32,224,3,2,2,5,1,4,64,160,4,1,1,5,0,
  0,240,2,2,2,5,1,4,128,64,4,4,4,5,0,0,
  112,144,176,80,4,6,6,5,0,0,128,128,224,144,144,224,
  3,4,4,5,0,0,96,128,128,96,4,6,6,5,0,0,
  16,16,112,144,144,112,4,4,4,5,0,0,96,176,192,96,
  4,6,6,5,0,0,32,80,64,224,64,64,4,5,5,5,
  0,255,112,144,96,128,112,4,6,6,5,0,0,128,128,224,
  144,144,144,3,6,6,5,1,0,64,0,192,64,64,224,3,
  7,7,5,1,255,32,0,32,32,32,160,64,4,6,6,5,
  0,0,128,128,160,192,160,144,3,6,6,5,1,0,192,64,
  64,64,64,224,4,4,4,5,0,0,160,240,144,144,4,4,
  4,5,0,0,224,144,144,144,4,4,4,5,0,0,96,144,
  144,96,4,5,5,5,0,255,224,144,144,224,128,4,5,5,
  5,0,255,112,144,144,112,16,4,4,4,5,0,0,224,144,
  128,128,4,4,4,5,0,0,112,192,48,224,4,6,6,5,
  0,0,64,64,224,64,64,48,4,4,4,5,0,0,144,144,
  144,112,3,4,4,5,1,0,160,160,160,64,4,4,4,5,
  0,0,144,144,240,240,4,4,4,5,0,0,144,96,96,144,
  4,5,5,5,0,255,144,144,80,32,64,4,4,4,5,0,
  0,240,32,64,240,3,6,6,5,1,0,32,64,192,64,64,
  32,1,6,6,5,2,0,128,128,128,128,128,128,3,6,6,
  5,1,0,128,64,96,64,64,128,4,2,2,5,0,4,80,
  160,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,0,0,0,5,4,0,1,6,6,5,2,0,128,0,
  128,128,128,128,4,6,6,5,0,255,32,112,160,160,112,32,
  4,5,5,5,0,0,48,64,224,64,176,5,5,5,5,0,
  0,136,112,80,112,136,3,6,6,5,1,0,160,160,64,224,
  64,64,1,5,5,5,2,0,128,128,0,128,128,3,7,7,
  5,1,255,96,128,192,160,96,32,192,3,1,1,5,1,5,
  160,5,7,7,5,0,255,112,136,168,200,168,136,112,3,3,
  3,5,0,3,96,160,96,5,3,3,5,0,1,72,144,72,
  4,2,2,5,0,1,240,16,3,1,1,5,1,2,224,5,
  7,7,5,0,255,112,136,232,200,200,136,112,4,1,1,5,
  0,5,240,3,3,3,5,1,3,64,160,64,5,6,6,5,
  0,0,32,32,248,32,32,248,2,4,4,5,1,2,192,64,
  128,192,2,4,4,5,1,2,192,192,64,192,2,2,2,5,
  1,4,64,128,4,5,5,5,0,255,144,144,144,224,128,4,
  6,6,5,0,0,112,208,208,80,80,80,2,2,2,5,1,
  2,192,192,2,2,2,5,1,255,64,128,3,4,4,5,1,
  2,64,192,64,224,3,3,3,5,0,3,64,160,64,5,3,
  3,5,0,1,144,72,144,4,7,7,5,0,255,128,128,128,
  144,48,112,16,4,7,7,5,0,255,128,128,128,176,16,32,
  48,4,7,7,5,0,255,192,192,64,208,48,112,16,3,6,
  6,5,1,0,64,0,64,128,160,64,4,6,6,5,0,0,
  96,144,144,240,144,144,4,6,6,5,0,0,96,144,144,240,
  144,144,4,6,6,5,0,0,96,144,144,240,144,144,4,6,
  6,5,0,0,96,144,144,240,144,144,4,6,6,5,0,0,
  144,96,144,240,144,144,4,6,6,5,0,0,96,96,144,240,
  144,144,4,6,6,5,0,0,112,160,176,224,160,176,4,7,
  7,5,0,255,96,144,128,128,144,96,64,4,6,6,5,0,
  0,240,128,224,128,128,240,4,6,6,5,0,0,240,128,224,
  128,128,240,4,6,6,5,0,0,240,128,224,128,128,240,4,
  6,6,5,0,0,240,128,224,128,128,240,3,6,6,5,1,
  0,224,64,64,64,64,224,3,6,6,5,1,0,224,64,64,
  64,64,224,3,6,6,5,1,0,224,64,64,64,64,224,3,
  6,6,5,1,0,224,64,64,64,64,224,4,6,6,5,0,
  0,224,80,208,80,80,224,4,6,6,5,0,0,176,144,208,
  176,176,144,4,6,6,5,0,0,96,144,144,144,144,96,4,
  6,6,5,0,0,96,144,144,144,144,96,4,6,6,5,0,
  0,96,144,144,144,144,96,4,6,6,5,0,0,96,144,144,
  144,144,96,4,6,6,5,0,0,144,96,144,144,144,96,4,
  4,4,5,0,0,144,96,96,144,4,6,6,5,0,0,112,
  176,176,208,208,224,4,6,6,5,0,0,144,144,144,144,144,
  96,4,6,6,5,0,0,144,144,144,144,144,96,4,6,6,
  5,0,0,144,144,144,144,144,96,4,6,6,5,0,0,144,
  0,144,144,144,96,3,6,6,5,1,0,160,160,160,64,64,
  64,4,6,6,5,0,0,128,224,144,224,128,128,4,6,6,
  5,0,0,96,144,160,144,144,160,4,6,6,5,0,0,64,
  32,112,144,176,80,4,6,6,5,0,0,32,64,112,144,176,
  80,4,6,6,5,0,0,32,80,112,144,176,80,4,6,6,
  5,0,0,80,160,112,144,176,80,4,6,6,5,0,0,80,
  0,112,144,176,80,4,6,6,5,0,0,96,96,112,144,176,
  80,4,4,4,5,0,0,112,176,160,112,3,5,5,5,1,
  255,96,128,128,96,64,4,6,6,5,0,0,64,32,96,176,
  192,96,4,6,6,5,0,0,32,64,96,176,192,96,4,6,
  6,5,0,0,64,160,96,176,192,96,4,6,6,5,0,0,
  160,0,96,176,192,96,3,6,6,5,1,0,128,64,192,64,
  64,224,3,6,6,5,1,0,64,128,192,64,64,224,3,6,
  6,5,1,0,64,160,192,64,64,224,3,6,6,5,1,0,
  160,0,192,64,64,224,4,6,6,5,0,0,64,48,96,144,
  144,96,4,6,6,5,0,0,80,160,224,144,144,144,4,6,
  6,5,0,0,64,32,96,144,144,96,4,6,6,5,0,0,
  32,64,96,144,144,96,4,6,6,5,0,0,96,0,96,144,
  144,96,4,6,6,5,0,0,80,160,96,144,144,96,4,6,
  6,5,0,0,80,0,96,144,144,96,4,5,5,5,0,0,
  96,0,240,0,96,4,4,4,5,0,0,112,176,208,224,4,
  6,6,5,0,0,64,32,144,144,144,112,4,6,6,5,0,
  0,32,64,144,144,144,112,4,6,6,5,0,0,96,0,144,
  144,144,112,4,6,6,5,0,0,80,0,144,144,144,112,4,
  7,7,5,0,255,32,64,144,144,80,32,64,4,6,6,5,
  0,255,128,224,144,144,224,128,4,7,7,5,0,255,80,0,
  144,144,80,32,64};

/*
  Fontname: -Misc-Fixed-Medium-R-Normal--10-100-75-75-C-60-ISO10646-1
  Copyright: Public domain terminal emulator font.  Share and enjoy.
  Capital A Height: 7, '1' Height: 7
  Calculated Max Values w= 7 h= 9 x= 5 y= 7 dx= 9 dy= 0 ascent= 8 len= 9
  Font Bounding box     w= 7 h=10 x= 0 y=-2
  Calculated Min Values           x= 0 y=-2 dx= 0 dy= 0
  Pure Font   ascent = 7 descent=-2
  X Font      ascent = 7 descent=-2
  Max Font    ascent = 8 descent=-2
*/
const u8g_fntpgm_uint8_t repetier_6x10[2625] U8G_FONT_SECTION("repetier_6x10") = {
  0,7,10,0,254,7,2,105,3,251,0,255,254,8,254,7,
  254,5,7,7,6,0,0,168,0,136,0,136,0,168,5,8,
  8,6,0,0,8,8,8,40,72,248,64,32,3,3,3,6,
  1,4,64,160,64,5,5,5,6,0,0,248,248,248,248,248,
  5,5,5,6,0,0,248,136,136,136,248,5,8,8,6,0,
  0,32,80,80,80,112,248,248,112,7,5,5,9,0,0,28,
  254,130,130,254,5,8,8,6,0,0,136,80,32,136,32,80,
  136,136,6,8,8,6,0,0,72,144,72,252,252,120,48,48,
  6,8,8,6,0,0,36,72,36,252,252,120,48,48,6,5,
  5,6,0,0,252,252,120,48,48,6,2,2,6,0,0,252,
  252,6,6,6,6,0,0,72,144,72,144,252,252,6,6,6,
  6,0,0,36,72,36,72,252,252,7,7,7,7,0,0,130,
  68,56,40,56,68,130,7,7,7,7,0,0,16,16,56,238,
  56,16,16,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,0,0,0,6,5,255,1,7,7,6,2,0,128,
  128,128,128,128,0,128,3,3,3,6,1,4,160,160,160,5,
  7,7,6,0,0,80,80,248,80,248,80,80,5,7,7,6,
  0,0,32,112,160,112,40,112,32,6,8,8,6,0,0,68,
  164,72,16,32,72,148,136,5,7,7,6,0,0,64,160,160,
  64,168,144,104,1,3,3,6,2,4,128,128,128,3,7,7,
  6,1,0,32,64,128,128,128,64,32,3,7,7,6,1,0,
  128,64,32,32,32,64,128,5,5,5,6,0,1,136,80,248,
  80,136,5,5,5,6,0,1,32,32,248,32,32,3,3,3,
  6,1,255,96,64,128,4,1,1,6,1,3,240,2,2,2,
  6,1,255,192,192,5,7,7,6,0,0,8,8,16,32,64,
  128,128,5,7,7,6,0,0,32,80,136,136,136,80,32,5,
  7,7,6,0,0,32,96,160,32,32,32,248,5,7,7,6,
  0,0,112,136,8,48,64,128,248,5,7,7,6,0,0,248,
  8,16,48,8,136,112,5,7,7,6,0,0,16,48,80,144,
  248,16,16,5,7,7,6,0,0,248,128,176,200,8,136,112,
  5,7,7,6,0,0,48,64,128,176,200,136,112,5,7,7,
  6,0,0,248,8,16,16,32,64,64,5,7,7,6,0,0,
  112,136,136,112,136,136,112,5,7,7,6,0,0,112,136,152,
  104,8,16,96,2,5,5,6,2,1,192,192,0,192,192,3,
  7,7,6,1,255,64,224,64,0,96,64,128,4,7,7,6,
  1,0,16,32,64,128,64,32,16,5,3,3,6,0,2,248,
  0,248,4,7,7,6,1,0,128,64,32,16,32,64,128,5,
  7,7,6,0,0,112,136,16,32,32,0,32,5,7,7,6,
  0,0,112,136,152,168,176,128,112,5,7,7,6,0,0,32,
  80,136,136,248,136,136,5,7,7,6,0,0,240,72,72,112,
  72,72,240,5,7,7,6,0,0,112,136,128,128,128,136,112,
  5,7,7,6,0,0,240,72,72,72,72,72,240,5,7,7,
  6,0,0,248,128,128,240,128,128,248,5,7,7,6,0,0,
  248,128,128,240,128,128,128,5,7,7,6,0,0,112,136,128,
  128,152,136,112,5,7,7,6,0,0,136,136,136,248,136,136,
  136,3,7,7,6,1,0,224,64,64,64,64,64,224,5,7,
  7,6,0,0,56,16,16,16,16,144,96,5,7,7,6,0,
  0,136,144,160,192,160,144,136,5,7,7,6,0,0,128,128,
  128,128,128,128,248,5,7,7,6,0,0,136,136,216,168,136,
  136,136,5,7,7,6,0,0,136,136,200,168,152,136,136,5,
  7,7,6,0,0,112,136,136,136,136,136,112,5,7,7,6,
  0,0,240,136,136,240,128,128,128,5,8,8,6,0,255,112,
  136,136,136,136,168,112,8,5,7,7,6,0,0,240,136,136,
  240,160,144,136,5,7,7,6,0,0,112,136,128,112,8,136,
  112,5,7,7,6,0,0,248,32,32,32,32,32,32,5,7,
  7,6,0,0,136,136,136,136,136,136,112,5,7,7,6,0,
  0,136,136,136,80,80,80,32,5,7,7,6,0,0,136,136,
  136,168,168,216,136,5,7,7,6,0,0,136,136,80,32,80,
  136,136,5,7,7,6,0,0,136,136,80,32,32,32,32,5,
  7,7,6,0,0,248,8,16,32,64,128,248,3,7,7,6,
  1,0,224,128,128,128,128,128,224,5,7,7,6,0,0,128,
  128,64,32,16,8,8,3,7,7,6,1,0,224,32,32,32,
  32,32,224,5,3,3,6,0,4,32,80,136,5,1,1,6,
  0,255,248,2,2,2,6,2,6,128,64,5,5,5,6,0,
  0,112,8,120,136,120,5,7,7,6,0,0,128,128,176,200,
  136,200,176,5,5,5,6,0,0,112,136,128,136,112,5,7,
  7,6,0,0,8,8,104,152,136,152,104,5,5,5,6,0,
  0,112,136,248,128,112,5,7,7,6,0,0,48,72,64,240,
  64,64,64,5,7,7,6,0,254,120,136,136,120,8,136,112,
  5,7,7,6,0,0,128,128,176,200,136,136,136,3,7,7,
  6,1,0,64,0,192,64,64,64,224,4,9,9,6,1,254,
  16,0,48,16,16,16,144,144,96,5,7,7,6,0,0,128,
  128,136,144,224,144,136,3,7,7,6,1,0,192,64,64,64,
  64,64,224,5,5,5,6,0,0,208,168,168,168,136,5,5,
  5,6,0,0,176,200,136,136,136,5,5,5,6,0,0,112,
  136,136,136,112,5,7,7,6,0,254,176,200,136,200,176,128,
  128,5,7,7,6,0,254,104,152,136,152,104,8,8,5,5,
  5,6,0,0,176,200,128,128,128,5,5,5,6,0,0,112,
  128,112,8,240,5,7,7,6,0,0,64,64,240,64,64,72,
  48,5,5,5,6,0,0,136,136,136,152,104,5,5,5,6,
  0,0,136,136,80,80,32,5,5,5,6,0,0,136,136,168,
  168,80,5,5,5,6,0,0,136,80,32,80,136,5,7,7,
  6,0,254,136,136,152,104,8,136,112,5,5,5,6,0,0,
  248,16,32,64,248,4,7,7,6,1,0,48,64,32,192,32,
  64,48,1,7,7,6,2,0,128,128,128,128,128,128,128,4,
  7,7,6,1,0,192,32,64,48,64,32,192,5,3,3,6,
  0,4,72,168,144,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,1,7,7,6,2,0,128,0,128,
  128,128,128,128,5,7,7,6,0,255,32,120,160,160,160,120,
  32,5,7,7,6,0,0,48,72,64,224,64,72,176,5,5,
  5,6,0,0,136,112,80,112,136,5,8,8,6,0,255,136,
  136,80,32,248,32,32,32,1,7,7,6,2,0,128,128,128,
  0,128,128,128,5,8,8,6,0,255,112,128,224,144,72,56,
  8,112,3,1,1,6,1,7,160,5,7,7,6,0,0,112,
  136,168,200,168,136,112,4,6,6,6,1,1,112,144,176,80,
  0,240,6,5,5,6,0,0,36,72,144,72,36,4,2,2,
  6,1,2,240,16,255,5,7,7,6,0,0,112,136,232,200,
  200,136,112,5,1,1,6,0,7,248,3,3,3,6,1,4,
  64,160,64,5,6,6,6,0,0,32,32,248,32,32,248,4,
  5,5,6,1,3,96,144,32,64,240,4,5,5,6,1,3,
  224,16,96,16,224,2,2,2,6,2,6,64,128,255,5,7,
  7,6,0,0,120,232,232,104,40,40,40,1,1,1,6,2,
  3,128,2,2,2,6,2,254,64,128,3,5,5,6,1,3,
  64,192,64,64,224,4,6,6,6,1,1,96,144,144,96,0,
  240,6,5,5,6,0,0,144,72,36,72,144,6,9,9,6,
  0,255,64,192,64,64,228,12,20,60,4,6,9,9,6,0,
  255,64,192,64,64,232,20,4,8,28,5,9,9,6,0,255,
  192,32,64,32,200,24,40,120,8,5,7,7,6,0,0,32,
  0,32,32,64,136,112,5,8,8,6,0,0,64,32,112,136,
  136,248,136,136,5,8,8,6,0,0,16,32,112,136,136,248,
  136,136,5,8,8,6,0,0,32,80,112,136,136,248,136,136,
  5,8,8,6,0,0,72,176,112,136,136,248,136,136,5,8,
  8,6,0,0,80,0,112,136,136,248,136,136,5,8,8,6,
  0,0,32,80,112,136,136,248,136,136,6,7,7,6,0,0,
  60,80,144,156,240,144,156,5,9,9,6,0,254,112,136,128,
  128,128,136,112,32,64,5,8,8,6,0,0,64,248,128,128,
  240,128,128,248,5,8,8,6,0,0,16,248,128,128,240,128,
  128,248,5,8,8,6,0,0,32,248,128,128,240,128,128,248,
  5,8,8,6,0,0,80,248,128,128,240,128,128,248,3,8,
  8,6,1,0,128,64,224,64,64,64,64,224,3,8,8,6,
  1,0,32,64,224,64,64,64,64,224,3,8,8,6,1,0,
  64,160,224,64,64,64,64,224,3,8,8,6,1,0,160,0,
  224,64,64,64,64,224,5,7,7,6,0,0,240,72,72,232,
  72,72,240,5,8,8,6,0,0,40,80,136,200,168,152,136,
  136,5,8,8,6,0,0,64,32,112,136,136,136,136,112,5,
  8,8,6,0,0,16,32,112,136,136,136,136,112,5,8,8,
  6,0,0,32,80,112,136,136,136,136,112,5,8,8,6,0,
  0,40,80,112,136,136,136,136,112,5,8,8,6,0,0,80,
  0,112,136,136,136,136,112,5,5,5,6,0,0,136,80,32,
  80,136,5,7,7,6,0,0,112,152,152,168,200,200,112,5,
  8,8,6,0,0,64,32,136,136,136,136,136,112,5,8,8,
  6,0,0,16,32,136,136,136,136,136,112,5,8,8,6,0,
  0,32,80,0,136,136,136,136,112,5,8,8,6,0,0,80,
  0,136,136,136,136,136,112,5,8,8,6,0,0,16,32,136,
  136,80,32,32,32,5,7,7,6,0,0,128,240,136,240,128,
  128,128,5,7,7,6,0,0,112,136,144,160,144,136,176,5,
  8,8,6,0,0,64,32,0,112,8,120,136,120,5,8,8,
  6,0,0,16,32,0,112,8,120,136,120,5,8,8,6,0,
  0,32,80,0,112,8,120,136,120,5,8,8,6,0,0,40,
  80,0,112,8,120,136,120,5,7,7,6,0,0,80,0,112,
  8,120,136,120,5,8,8,6,0,0,32,80,32,112,8,120,
  136,120,6,5,5,6,0,0,120,20,124,144,124,5,7,7,
  6,0,254,112,136,128,136,112,32,64,5,8,8,6,0,0,
  64,32,0,112,136,248,128,112,5,8,8,6,0,0,16,32,
  0,112,136,248,128,112,5,8,8,6,0,0,32,80,0,112,
  136,248,128,112,5,7,7,6,0,0,80,0,112,136,248,128,
  112,3,8,8,6,1,0,128,64,0,192,64,64,64,224,3,
  8,8,6,1,0,64,128,0,192,64,64,64,224,3,8,8,
  6,1,0,64,160,0,192,64,64,64,224,3,7,7,6,1,
  0,160,0,192,64,64,64,224,5,7,7,6,0,0,192,48,
  112,136,136,136,112,5,8,8,6,0,0,40,80,0,176,200,
  136,136,136,5,8,8,6,0,0,64,32,0,112,136,136,136,
  112,5,8,8,6,0,0,16,32,0,112,136,136,136,112,5,
  8,8,6,0,0,32,80,0,112,136,136,136,112,5,8,8,
  6,0,0,40,80,0,112,136,136,136,112,5,7,7,6,0,
  0,80,0,112,136,136,136,112,5,5,5,6,0,1,32,0,
  248,0,32,5,5,5,6,0,0,120,152,168,200,240,5,8,
  8,6,0,0,64,32,0,136,136,136,152,104,5,8,8,6,
  0,0,16,32,0,136,136,136,152,104,5,8,8,6,0,0,
  32,80,0,136,136,136,152,104,5,7,7,6,0,0,80,0,
  136,136,136,152,104,5,9,9,6,0,254,16,32,136,136,152,
  104,8,136,112,5,8,8,6,0,254,128,240,136,136,136,240,
  128,128,5,9,9,6,0,254,80,0,136,136,152,104,8,136,
  112};
/*
  Fontname: -Misc-Fixed-Medium-R-SemiCondensed--12-110-75-75-C-60-ISO10646-1
  Copyright: Public domain terminal emulator font.  Share and enjoy.
  Capital A Height: 3, '1' Height: 8
  Calculated Max Values w= 6 h=12 x= 5 y= 8 dx= 6 dy= 0 ascent=10 len=12
  Font Bounding box     w= 6 h=12 x= 0 y=-2
  Calculated Min Values           x= 0 y=-2 dx= 0 dy= 0
  Pure Font   ascent = 3 descent= 0
  X Font      ascent = 8 descent= 0
  Max Font    ascent =10 descent=-2
*/
const u8g_fntpgm_uint8_t u8g_font_6x12_67_75[2382] U8G_FONT_SECTION("u8g_font_6x12_67_75") = {
  1,6,12,0,254,3,2,41,3,99,0,255,0,10,254,8,
  0,2,87,103,168,0,136,0,136,0,168,2,87,103,240,136,
  232,168,232,136,240,2,87,103,112,168,248,168,248,168,112,2,
  87,103,112,136,8,8,8,136,112,18,69,101,96,144,16,144,
  96,1,88,104,112,136,128,128,136,120,8,8,2,87,103,32,
  32,32,32,168,112,32,2,103,103,248,252,252,156,252,252,248,
  2,87,103,112,248,248,168,248,248,112,255,255,255,255,255,255,
  255,3,85,101,32,64,248,64,32,2,87,103,32,112,168,32,
  32,32,32,3,85,101,32,16,248,16,32,2,87,103,32,32,
  32,32,168,112,32,4,83,99,80,248,80,2,87,103,32,112,
  168,32,168,112,32,2,89,105,192,240,224,160,32,16,16,8,
  8,2,89,105,24,120,56,40,32,64,64,128,128,2,89,105,
  128,128,64,64,32,40,56,120,24,2,89,105,8,8,16,16,
  32,160,224,240,192,3,101,101,40,72,252,80,48,3,101,101,
  48,40,252,72,80,4,99,99,192,216,100,4,99,99,12,108,
  152,3,101,101,40,80,252,80,40,2,88,104,32,112,168,112,
  168,32,32,32,3,101,101,80,40,252,40,80,2,88,104,32,
  32,32,168,112,168,112,32,3,101,101,36,72,240,72,36,3,
  101,101,144,72,60,72,144,3,85,101,40,72,248,72,40,2,
  87,103,32,112,168,32,32,32,248,3,85,101,160,144,248,144,
  160,2,87,103,248,32,32,32,168,112,32,2,87,103,32,112,
  168,32,168,112,248,3,101,101,40,68,248,64,32,3,101,101,
  80,136,124,8,16,3,101,101,32,76,252,72,40,3,101,101,
  16,200,252,72,80,3,100,100,72,220,236,72,3,101,101,8,
  88,252,104,64,2,87,103,128,144,176,208,144,56,16,2,88,
  104,32,64,248,72,40,8,8,8,2,88,104,32,16,248,144,
  160,128,128,128,2,88,104,8,8,8,40,72,248,64,32,2,
  87,103,128,128,160,144,248,16,32,3,84,100,240,16,56,16,
  3,85,101,8,8,72,248,64,3,85,101,48,72,72,232,72,
  3,85,101,96,144,144,184,16,2,88,104,248,128,224,192,160,
  32,16,16,2,89,105,160,192,248,192,168,24,248,24,40,2,
  86,102,56,48,168,136,136,112,2,86,102,224,96,168,136,136,
  112,5,83,99,32,64,248,3,83,99,248,64,32,34,56,104,
  128,192,160,128,128,128,128,128,2,56,104,32,96,160,32,32,
  32,32,32,5,83,99,32,16,248,3,83,99,248,16,32,34,
  56,104,128,128,128,128,128,160,192,128,2,56,104,32,32,32,
  32,32,160,96,32,2,89,105,32,16,248,16,32,64,248,64,
  32,2,88,104,80,240,80,80,80,80,120,80,2,89,105,32,
  64,248,64,32,16,248,16,32,2,89,105,32,64,248,64,32,
  64,248,64,32,2,88,104,80,248,80,80,80,80,80,80,2,
  89,105,32,16,248,16,32,16,248,16,32,2,88,104,80,80,
  80,80,80,80,248,80,2,87,103,32,64,248,0,248,16,32,
  2,87,103,32,16,248,0,248,64,32,2,103,103,20,40,124,
  144,124,32,80,2,103,103,8,88,252,164,252,104,64,2,103,
  103,160,80,248,36,248,16,40,3,85,101,32,120,128,120,32,
  2,87,103,32,80,216,80,80,80,80,3,85,101,32,240,8,
  240,32,2,87,103,80,80,80,80,216,80,32,3,101,101,72,
  252,132,252,72,2,88,104,32,80,216,80,80,216,80,32,2,
  102,102,248,160,208,168,148,8,2,102,102,124,20,44,84,164,
  64,2,102,102,64,164,84,44,20,124,2,102,102,8,148,168,
  208,160,248,2,103,103,16,60,64,252,64,60,16,2,103,103,
  32,240,8,252,8,240,32,3,100,100,64,232,212,64,3,100,
  100,8,92,172,8,2,88,104,32,112,168,32,112,32,112,32,
  2,88,104,32,112,32,112,32,168,112,32,3,101,101,32,64,
  212,64,32,2,88,104,32,112,136,32,0,32,0,32,3,101,
  101,16,8,172,8,16,2,88,104,32,0,32,0,32,136,112,
  32,3,85,101,160,192,248,192,160,3,85,101,40,24,248,24,
  40,3,85,101,32,120,136,120,32,2,88,104,32,80,216,80,
  80,80,80,112,3,85,101,32,240,136,240,32,2,88,104,112,
  80,80,80,80,216,80,32,2,89,105,32,80,216,80,112,0,
  112,80,112,2,89,105,32,80,216,80,80,80,216,136,248,2,
  89,105,32,80,248,136,80,80,216,136,248,2,89,105,32,112,
  248,112,112,112,248,168,248,2,89,105,32,80,216,80,216,80,
  80,80,112,2,89,105,32,80,216,80,216,80,216,136,248,3,
  85,101,160,240,136,240,160,2,88,104,248,128,176,160,144,16,
  8,8,2,88,104,128,128,64,72,40,104,8,248,2,88,104,
  32,80,216,80,80,216,80,32,3,101,101,16,104,252,104,16,
  2,88,104,80,120,80,80,80,80,240,80,2,89,105,16,248,
  16,16,248,16,16,248,16,3,101,101,40,72,252,72,40,3,
  101,101,80,72,252,72,80,3,101,101,48,120,252,120,48,3,
  101,101,56,88,252,88,56,3,101,101,112,104,252,104,112,3,
  101,101,48,120,252,120,48,3,85,101,32,96,184,96,32,3,
  85,101,32,48,232,48,32,3,101,101,48,120,180,120,48,6,
  102,102,252,252,252,252,252,252,0,98,98,252,252,0,99,99,
  252,252,252,0,101,101,252,252,252,252,252,0,102,102,252,252,
  252,252,252,252,0,104,104,252,252,252,252,252,252,252,252,0,
  105,105,252,252,252,252,252,252,252,252,252,0,107,107,252,252,
  252,252,252,252,252,252,252,252,252,0,108,108,252,252,252,252,
  252,252,252,252,252,252,252,252,0,92,108,248,248,248,248,248,
  248,248,248,248,248,248,248,0,76,108,240,240,240,240,240,240,
  240,240,240,240,240,240,0,76,108,240,240,240,240,240,240,240,
  240,240,240,240,240,0,60,108,224,224,224,224,224,224,224,224,
  224,224,224,224,0,44,108,192,192,192,192,192,192,192,192,192,
  192,192,192,0,44,108,192,192,192,192,192,192,192,192,192,192,
  192,192,0,28,108,128,128,128,128,128,128,128,128,128,128,128,
  128,48,60,108,224,224,224,224,224,224,224,224,224,224,224,224,
  1,107,107,168,0,84,0,168,0,84,0,168,0,84,0,108,
  108,168,84,168,84,168,84,168,84,168,84,168,84,0,108,108,
  84,252,168,252,84,252,168,252,84,252,168,252,10,98,98,252,
  252,80,28,108,128,128,128,128,128,128,128,128,128,128,128,128,
  0,54,102,224,224,224,224,224,224,48,54,102,224,224,224,224,
  224,224,6,54,102,224,224,224,224,224,224,0,108,108,224,224,
  224,224,224,224,252,252,252,252,252,252,0,108,108,224,224,224,
  224,224,224,28,28,28,28,28,28,0,108,108,252,252,252,252,
  252,252,224,224,224,224,224,224,0,108,108,252,252,252,252,252,
  252,28,28,28,28,28,28,54,54,102,224,224,224,224,224,224,
  0,108,108,28,28,28,28,28,28,224,224,224,224,224,224,0,
  108,108,28,28,28,28,28,28,252,252,252,252,252,252,2,85,
  101,248,248,248,248,248,2,85,101,248,136,136,136,248,2,85,
  101,112,136,136,136,112,2,85,101,248,136,168,136,248,2,85,
  101,248,136,248,136,248,2,85,101,248,168,168,168,248,2,85,
  101,248,168,248,168,248,2,85,101,248,200,168,152,248,2,85,
  101,248,152,168,200,248,2,85,101,248,216,168,216,248,20,51,
  99,224,224,224,20,51,99,224,160,224,3,101,101,252,252,252,
  252,252,3,101,101,252,132,132,132,252,17,74,106,240,240,240,
  240,240,240,240,240,240,240,17,74,106,240,144,144,144,144,144,
  144,144,144,240,4,99,99,60,120,240,4,99,99,60,72,240,
  2,87,103,32,32,112,112,248,248,248,2,87,103,32,32,80,
  80,136,136,248,3,85,101,32,32,112,112,248,3,85,101,32,
  32,80,80,248,18,71,103,128,192,224,240,224,192,128,18,71,
  103,128,192,160,144,160,192,128,19,53,101,128,192,224,192,128,
  19,53,101,128,192,160,192,128,3,101,101,192,240,252,240,192,
  3,101,101,192,176,140,176,192,2,87,103,248,248,248,112,112,
  32,32,2,87,103,248,136,136,80,80,32,32,2,85,101,248,
  112,112,32,32,2,85,101,248,80,80,32,32,18,71,103,16,
  48,112,240,112,48,16,18,71,103,16,48,80,144,80,48,16,
  19,53,101,32,96,224,96,32,19,53,101,32,96,160,96,32,
  3,101,101,12,60,252,60,12,3,101,101,12,52,196,52,12,
  3,85,101,32,112,248,112,32,3,85,101,32,80,136,80,32,
  3,85,101,32,80,168,80,32,2,102,102,48,72,180,180,72,
  48,2,87,103,32,80,80,136,80,80,32,2,102,102,48,72,
  132,132,72,48,2,102,102,32,8,128,4,64,16,2,85,101,
  112,168,168,168,112,2,87,103,112,136,168,216,168,136,112,2,
  102,102,48,120,252,252,120,48,2,102,102,48,104,228,228,104,
  48,2,102,102,48,88,156,156,88,48,2,102,102,48,72,132,
  252,120,48,2,102,102,48,120,252,132,72,48,2,102,102,48,
  88,156,132,72,48,2,102,102,48,104,228,132,72,48,18,89,
  105,8,56,120,120,248,120,120,56,8,2,89,105,128,224,240,
  240,248,240,240,224,128,0,108,108,252,252,252,252,204,132,132,
  204,252,252,252,252,0,108,108,252,252,252,204,180,120,120,180,
  204,252,252,252,6,102,102,252,252,252,204,180,120,0,102,102,
  120,180,204,252,252,252,5,51,99,32,64,128,53,51,99,128,
  64,32,50,51,99,32,64,128,2,51,99,128,64,32,5,99,
  99,48,72,132,2,99,99,132,72,48,2,85,101,8,24,56,
  120,248,2,85,101,128,192,224,240,248,2,85,101,248,240,224,
  192,128,2,85,101,248,120,56,24,8,2,85,101,112,136,136,
  136,112,2,85,101,248,232,232,232,248,2,85,101,248,184,184,
  184,248,2,85,101,248,248,232,200,248,2,85,101,248,152,184,
  248,248,2,85,101,248,168,168,168,248,2,87,103,32,32,80,
  112,168,136,248,2,87,103,32,32,112,112,232,232,248,2,87,
  103,32,32,112,112,184,184,248,2,103,103,48,72,132,132,132,
  72,48,2,85,101,248,168,232,136,248,2,85,101,248,136,232,
  168,248,2,85,101,248,136,184,168,248,2,85,101,248,168,184,
  136,248,2,85,101,112,168,232,136,112,2,85,101,112,136,232,
  168,112,2,85,101,112,136,184,168,112,2,85,101,112,168,184,
  136,112,3,85,101,248,144,160,192,128,3,85,101,248,72,40,
  24,8,3,85,101,128,192,160,144,248,20,68,100,240,144,144,
  240,19,68,100,240,240,240,240,20,68,100,240,144,144,240,20,
  68,100,240,240,240,240,3,85,101,8,24,40,72,248};
// ======== u8g_dev_st7920_128x64.c ==============

/*

  u8g_dev_st7920_128x64.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


#define WIDTH 128
#define HEIGHT 64
#define PAGE_HEIGHT 8


/* init sequence from https://github.com/adafruit/ST7565-LCD/blob/master/ST7565/ST7565.cpp */
static const uint8_t u8g_dev_st7920_128x64_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_RST(15),           /* do reset low pulse with (15*16)+2 milliseconds (=maximum delay)*/
  U8G_ESC_DLY(100),         /* 8 Dez 2012: additional delay 100 ms because of reset*/
  U8G_ESC_CS(1),             /* enable chip */
  U8G_ESC_DLY(50),         /* delay 50 ms */

  0x038,                                /* 8 Bit interface (DL=1), basic instruction set (RE=0) */
  0x00c,                                /* display on, cursor & blink off; 0x08: all off */
  0x006,                                /* Entry mode: Cursor move to right ,DDRAM address counter (AC) plus 1, no shift */
  0x002,                                /* disable scroll, enable CGRAM adress */
  0x001,                                /* clear RAM, needs 1.6 ms */
  U8G_ESC_DLY(100),               /* delay 100 ms */

  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

uint8_t u8g_dev_st7920_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        uint8_t y, i;
        uint8_t *ptr;
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

        u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
        u8g_SetChipSelect(u8g, dev, 1);
        y = pb->p.page_y0;
        ptr = (uint8_t*)pb->buf;
        for( i = 0; i < 8; i ++ )
        {
          u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
          u8g_WriteByte(u8g, dev, 0x03e );      /* enable extended mode */

          if ( y < 32 )
          {
                  u8g_WriteByte(u8g, dev, 0x080 | y );      /* y pos  */
                  u8g_WriteByte(u8g, dev, 0x080  );      /* set x pos to 0*/
          }
          else
          {
                  u8g_WriteByte(u8g, dev, 0x080 | (y-32) );      /* y pos  */
                  u8g_WriteByte(u8g, dev, 0x080 | 8);      /* set x pos to 64*/
          }

          u8g_SetAddress(u8g, dev, 1);                  /* data mode */
          u8g_WriteSequence(u8g, dev, WIDTH/8, ptr);
          ptr += WIDTH/8;
          y++;
        }
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
  }
  return u8g_dev_pb8h1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_st7920_128x64_4x_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        uint8_t y, i;
        uint8_t *ptr;
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

        u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
        u8g_SetChipSelect(u8g, dev, 1);
        y = pb->p.page_y0;
        ptr = (uint8_t*)pb->buf;
        for( i = 0; i < 32; i ++ )
        {
          u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
          u8g_WriteByte(u8g, dev, 0x03e );      /* enable extended mode */

          if ( y < 32 )
          {
                  u8g_WriteByte(u8g, dev, 0x080 | y );      /* y pos  */
                  u8g_WriteByte(u8g, dev, 0x080  );      /* set x pos to 0*/
          }
          else
          {
                  u8g_WriteByte(u8g, dev, 0x080 | (y-32) );      /* y pos  */
                  u8g_WriteByte(u8g, dev, 0x080 | 8);      /* set x pos to 64*/
          }

          u8g_SetAddress(u8g, dev, 1);                  /* data mode */
          u8g_WriteSequence(u8g, dev, WIDTH/8, ptr);
          ptr += WIDTH/8;
          y++;
        }
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
  }
  return u8g_dev_pb32h1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_dev_st7920_128x64_sw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_fn, U8G_COM_ST7920_SW_SPI);
U8G_PB_DEV(u8g_dev_st7920_128x64_hw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_fn, U8G_COM_ST7920_HW_SPI);
U8G_PB_DEV(u8g_dev_st7920_128x64_8bit, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_fn, U8G_COM_FAST_PARALLEL);
U8G_PB_DEV(u8g_dev_st7920_128x64_custom, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_fn, u8g_com_arduino_st7920_custom_fn);



#define QWIDTH (WIDTH*4)
uint8_t u8g_dev_st7920_128x64_4x_buf[QWIDTH] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_st7920_128x64_4x_pb = { {32, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_st7920_128x64_4x_buf};
u8g_dev_t u8g_dev_st7920_128x64_4x_sw_spi = { u8g_dev_st7920_128x64_4x_fn, &u8g_dev_st7920_128x64_4x_pb, U8G_COM_ST7920_SW_SPI };
u8g_dev_t u8g_dev_st7920_128x64_4x_hw_spi = { u8g_dev_st7920_128x64_4x_fn, &u8g_dev_st7920_128x64_4x_pb, U8G_COM_ST7920_HW_SPI };
u8g_dev_t u8g_dev_st7920_128x64_4x_8bit = { u8g_dev_st7920_128x64_4x_fn, &u8g_dev_st7920_128x64_4x_pb, U8G_COM_FAST_PARALLEL };
u8g_dev_t u8g_dev_st7920_128x64_4x_custom = { u8g_dev_st7920_128x64_4x_fn, &u8g_dev_st7920_128x64_4x_pb, u8g_com_arduino_st7920_custom_fn };


// ================ u8g_font.c ===================

/*

  u8g_font.c

  U8G Font High Level Interface

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


/* font api */

/* pointer to the start adress of the glyph, points to progmem area */
typedef void * u8g_glyph_t;

/* size of the font data structure, there is no struct or class... */
#define U8G_FONT_DATA_STRUCT_SIZE 17

/*
  ... instead the fields of the font data structure are accessed directly by offset
  font information
  offset
  0             font format
  1             FONTBOUNDINGBOX width           unsigned
  2             FONTBOUNDINGBOX height          unsigned
  3             FONTBOUNDINGBOX x-offset         signed
  4             FONTBOUNDINGBOX y-offset        signed
  5             capital A height                                unsigned
  6             start 'A'
  8             start 'a'
  10            encoding start
  11            encoding end
  12            descent 'g'                     negative: below baseline
  13            font max ascent
  14            font min decent             negative: below baseline
  15            font xascent
  16            font xdecent             negative: below baseline

*/

/* use case: What is the width and the height of the minimal box into which string s fints? */
void u8g_font_GetStrSize(const void *font, const char *s, u8g_uint_t *width, u8g_uint_t *height);
void u8g_font_GetStrSizeP(const void *font, const char *s, u8g_uint_t *width, u8g_uint_t *height);

/* use case: lower left edge of a minimal box is known, what is the correct x, y position for the string draw procedure */
void u8g_font_AdjustXYToDraw(const void *font, const char *s, u8g_uint_t *x, u8g_uint_t *y);
void u8g_font_AdjustXYToDrawP(const void *font, const char *s, u8g_uint_t *x, u8g_uint_t *y);

/* use case: Baseline origin known, return minimal box */
void u8g_font_GetStrMinBox(u8g_t *u8g, const void *font, const char *s, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height);

/* procedures */

/*========================================================================*/
/* low level byte and word access */

/* removed NOINLINE, because it leads to smaller code, might also be faster */
//static uint8_t u8g_font_get_byte(const u8g_fntpgm_uint8_t *font, uint8_t offset) U8G_NOINLINE;
static uint8_t u8g_font_get_byte(const u8g_fntpgm_uint8_t *font, uint8_t offset)
{
  font += offset;
  return u8g_pgm_read( (u8g_pgm_uint8_t *)font );
}

static uint16_t u8g_font_get_word(const u8g_fntpgm_uint8_t *font, uint8_t offset) U8G_NOINLINE;
static uint16_t u8g_font_get_word(const u8g_fntpgm_uint8_t *font, uint8_t offset)
{
    uint16_t pos;
    font += offset;
    pos = u8g_pgm_read( (u8g_pgm_uint8_t *)font );
    font++;
    pos <<= 8;
    pos += u8g_pgm_read( (u8g_pgm_uint8_t *)font);
    return pos;
}

/*========================================================================*/
/* direct access on the font */

static uint8_t u8g_font_GetFormat(const u8g_fntpgm_uint8_t *font) U8G_NOINLINE;
static uint8_t u8g_font_GetFormat(const u8g_fntpgm_uint8_t *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 0);
}

static uint8_t u8g_font_GetFontGlyphStructureSize(const u8g_fntpgm_uint8_t *font) U8G_NOINLINE;
static uint8_t u8g_font_GetFontGlyphStructureSize(const u8g_fntpgm_uint8_t *font)
{
  switch(u8g_font_GetFormat((const u8g_fntpgm_uint8_t*)font))
  {
    case 0: return 6;
    case 1: return 3;
    case 2: return 6;
  }
  return 3;
}

static uint8_t u8g_font_GetBBXWidth(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 1);
}

static uint8_t u8g_font_GetBBXHeight(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 2);
}

static int8_t u8g_font_GetBBXOffX(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 3);
}

static int8_t u8g_font_GetBBXOffY(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 4);
}

uint8_t u8g_font_GetCapitalAHeight(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 5);
}

uint16_t u8g_font_GetEncoding65Pos(const void *font) U8G_NOINLINE;
uint16_t u8g_font_GetEncoding65Pos(const void *font)
{
    return u8g_font_get_word((const u8g_fntpgm_uint8_t*)font, 6);
}

uint16_t u8g_font_GetEncoding97Pos(const void *font) U8G_NOINLINE;
uint16_t u8g_font_GetEncoding97Pos(const void *font)
{
    return u8g_font_get_word((const u8g_fntpgm_uint8_t*)font, 8);
}

uint8_t u8g_font_GetFontStartEncoding(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 10);
}

uint8_t u8g_font_GetFontEndEncoding(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 11);
}

int8_t u8g_font_GetLowerGDescent(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 12);
}

int8_t u8g_font_GetFontAscent(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 13);
}

int8_t u8g_font_GetFontDescent(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 14);
}

int8_t u8g_font_GetFontXAscent(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 15);
}

int8_t u8g_font_GetFontXDescent(const void *font)
{
  return u8g_font_get_byte((const u8g_fntpgm_uint8_t*)font, 16);
}


/* return the data start for a font and the glyph pointer */
static uint8_t *u8g_font_GetGlyphDataStart(const void *font, u8g_glyph_t g)
{
  return ((u8g_fntpgm_uint8_t *)g) + u8g_font_GetFontGlyphStructureSize((const u8g_fntpgm_uint8_t*)font);
}

/* calculate the overall length of the font, only used to create the picture for the google wiki */
size_t u8g_font_GetSize(const void *font)
{
  uint8_t *p = (uint8_t *)(font);
  uint8_t font_format = u8g_font_GetFormat((const u8g_fntpgm_uint8_t*)font);
  uint8_t data_structure_size = u8g_font_GetFontGlyphStructureSize((const u8g_fntpgm_uint8_t*)font);
  uint8_t start, end;
  uint8_t i;
  uint8_t mask = 255;

  start = u8g_font_GetFontStartEncoding(font);
  end = u8g_font_GetFontEndEncoding(font);

  if ( font_format == 1 )
    mask = 15;

  p += U8G_FONT_DATA_STRUCT_SIZE;       /* skip font general information */

  i = start;
  for(;;)
  {
    if ( u8g_pgm_read((u8g_pgm_uint8_t *)(p)) == 255 )
    {
      p += 1;
    }
    else
    {
      p += u8g_pgm_read( ((u8g_pgm_uint8_t *)(p)) + 2 ) & mask;
      p += data_structure_size;
    }
    if ( i == end )
      break;
    i++;
  }

  return p - (uint8_t *)font;
}

/*========================================================================*/
/* u8g interface, font access */

uint8_t u8g_GetFontBBXWidth(u8g_t *u8g)
{
  return u8g_font_GetBBXWidth(u8g->font);
}

uint8_t u8g_GetFontBBXHeight(u8g_t *u8g)
{
  return u8g_font_GetBBXHeight(u8g->font);
}

int8_t u8g_GetFontBBXOffX(u8g_t *u8g) U8G_NOINLINE;
int8_t u8g_GetFontBBXOffX(u8g_t *u8g)
{
  return u8g_font_GetBBXOffX(u8g->font);
}

int8_t u8g_GetFontBBXOffY(u8g_t *u8g) U8G_NOINLINE;
int8_t u8g_GetFontBBXOffY(u8g_t *u8g)
{
  return u8g_font_GetBBXOffY(u8g->font);
}

uint8_t u8g_GetFontCapitalAHeight(u8g_t *u8g) U8G_NOINLINE;
uint8_t u8g_GetFontCapitalAHeight(u8g_t *u8g)
{
  return u8g_font_GetCapitalAHeight(u8g->font);
}

/*========================================================================*/
/* glyph handling */

static void u8g_CopyGlyphDataToCache(u8g_t *u8g, u8g_glyph_t g)
{
  uint8_t tmp;
  switch( u8g_font_GetFormat(u8g->font) )
  {
    case 0:
    case 2:
  /*
    format 0
    glyph information
    offset
    0             BBX width                                       unsigned
    1             BBX height                                      unsigned
    2             data size                                          unsigned    (BBX width + 7)/8 * BBX height
    3             DWIDTH                                          signed
    4             BBX xoffset                                    signed
    5             BBX yoffset                                    signed
  byte 0 == 255 indicates empty glyph
  */
      u8g->glyph_width =  u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 0 );
      u8g->glyph_height =  u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 1 );
      u8g->glyph_dx =  u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 3 );
      u8g->glyph_x =  u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 4 );
      u8g->glyph_y =  u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 5 );
      break;
    case 1:
    default:
      /*
format 1
  0             BBX xoffset                                    signed   --> upper 4 Bit
  0             BBX yoffset                                    signed --> lower 4 Bit
  1             BBX width                                       unsigned --> upper 4 Bit
  1             BBX height                                      unsigned --> lower 4 Bit
  2             data size                                           unsigned -(BBX width + 7)/8 * BBX height  --> lower 4 Bit
  2             DWIDTH                                          signed --> upper  4 Bit
  byte 0 == 255 indicates empty glyph
      */

      tmp = u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 0 );
      u8g->glyph_y =  tmp & 15;
      u8g->glyph_y-=2;
      tmp >>= 4;
      u8g->glyph_x =  tmp;

      tmp = u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 1 );
      u8g->glyph_height =  tmp & 15;
      tmp >>= 4;
      u8g->glyph_width =  tmp;

      tmp = u8g_pgm_read( ((u8g_pgm_uint8_t *)g) + 2 );
      tmp >>= 4;
      u8g->glyph_dx = tmp;


      break;
  }
}

//void u8g_FillEmptyGlyphCache(u8g_t *u8g) U8G_NOINLINE;
static void u8g_FillEmptyGlyphCache(u8g_t *u8g)
{
  u8g->glyph_dx = 0;
  u8g->glyph_width = 0;
  u8g->glyph_height = 0;
  u8g->glyph_x = 0;
  u8g->glyph_y = 0;
}

/*
  Find (with some speed optimization) and return a pointer to the glyph data structure
  Also uncompress (format 1) and copy the content of the data structure to the u8g structure
*/
u8g_glyph_t u8g_GetGlyph(u8g_t *u8g, uint8_t requested_encoding)
{
  uint8_t *p = (uint8_t *)(u8g->font);
  uint8_t font_format = u8g_font_GetFormat(u8g->font);
  uint8_t data_structure_size = u8g_font_GetFontGlyphStructureSize(u8g->font);
  uint8_t start, end;
  uint16_t pos;
  uint8_t i;
  uint8_t mask = 255;

  if ( font_format == 1 )
    mask = 15;

  start = u8g_font_GetFontStartEncoding(u8g->font);
  end = u8g_font_GetFontEndEncoding(u8g->font);

  pos = u8g_font_GetEncoding97Pos(u8g->font);
  if ( requested_encoding >= 97 && pos > 0 )
  {
    p+= pos;
    start = 97;
  }
  else
  {
    pos = u8g_font_GetEncoding65Pos(u8g->font);
    if ( requested_encoding >= 65 && pos > 0 )
    {
      p+= pos;
      start = 65;
    }
    else
      p += U8G_FONT_DATA_STRUCT_SIZE;       /* skip font general information */
  }

  if ( requested_encoding > end )
  {
    u8g_FillEmptyGlyphCache(u8g);
    return NULL;                      /* not found */
  }

  i = start;
  if ( i <= end )
  {
    for(;;)
    {
      if ( u8g_pgm_read((u8g_pgm_uint8_t *)(p)) == 255 )
      {
        p += 1;
      }
      else
      {
        if ( i == requested_encoding )
        {
          u8g_CopyGlyphDataToCache(u8g, p);
          return p;
        }
        p += u8g_pgm_read( ((u8g_pgm_uint8_t *)(p)) + 2 ) & mask;
        p += data_structure_size;
      }
      if ( i == end )
        break;
      i++;
    }
  }

  u8g_FillEmptyGlyphCache(u8g);

  return NULL;
}

uint8_t u8g_IsGlyph(u8g_t *u8g, uint8_t requested_encoding)
{
  if ( u8g_GetGlyph(u8g, requested_encoding) != NULL )
    return 1;
  return 0;
}

int8_t u8g_GetGlyphDeltaX(u8g_t *u8g, uint8_t requested_encoding)
{
  if ( u8g_GetGlyph(u8g, requested_encoding) == NULL )
    return 0;  /* should never happen, so return something */
  return u8g->glyph_dx;
}


/*========================================================================*/
/* glyph drawing procedures */

#ifdef OBSOLETE
/*
  Draw a glyph
  x,y: left baseline position of the glyph
*/
int8_t u8g_DrawGlyphDir(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t encoding)
{
  u8g_glyph_t g;
  uint8_t w, h, i, j;
  const u8g_pgm_uint8_t *data;
  uint8_t bytes_per_line;
  u8g_uint_t ix, iy;

  g = u8g_GetGlyph(u8g, encoding);
  if ( g == NULL  )
    return 0;
  w = u8g->glyph_width;
  h = u8g->glyph_height;

  bytes_per_line = w;
  bytes_per_line += 7;
  bytes_per_line /= 8;

  data = u8g_font_GetGlyphDataStart(u8g->font, g);

  switch(dir)
  {
    case 0:
      x += u8g->glyph_x;
      y -= u8g->glyph_y;
      y--;
      //u8g_DrawFrame(u8g, x, y-h+1, w, h);
      if ( u8g_IsBBXIntersection(u8g, x, y-h+1, w, h) == 0 )
        return u8g->glyph_dx;

      iy = y;
      iy -= h;
      iy++;

      for( j = 0; j < h; j++ )
      {
        ix = x;
        for( i = 0; i < bytes_per_line; i++ )
        {
          u8g_Draw8Pixel(u8g, ix, iy, dir, u8g_pgm_read(data));
          data++;
          ix+=8;
        }
        iy++;
      }
      break;
    case 1:
      x += u8g->glyph_y;
      x++;
      y += u8g->glyph_x;
      //printf("enc %d, dir %d, x %d, y %d, w %d, h %d\n", encoding, dir, x, y, w, h);
      //u8g_DrawFrame(u8g, x, y, h, w);
      if ( u8g_IsBBXIntersection(u8g, x, y, h, w) == 0 )
        return u8g->glyph_dx;

      ix = x;
      ix += h;
      ix--;
      for( j = 0; j < h; j++ )
      {
        iy = y;
        for( i = 0; i < bytes_per_line; i++ )
        {
          u8g_Draw8Pixel(u8g, ix, iy, dir, u8g_pgm_read(data));
          data++;
          iy+=8;
        }
        ix--;
      }
      break;
    case 2:
      x -= u8g->glyph_x;
      y += u8g->glyph_y;
      y++;
      if ( u8g_IsBBXIntersection(u8g, x-w-1, y, w, h) == 0 )
        return u8g->glyph_dx;

      iy = y;
      iy += h;
      iy--;
      for( j = 0; j < h; j++ )
      {
        ix = x;
        for( i = 0; i < bytes_per_line; i++ )
        {
          u8g_Draw8Pixel(u8g, ix, iy, dir, u8g_pgm_read(data));
          data++;
          ix-=8;
        }
        iy--;
      }
      break;
    case 3:
      x -= u8g->glyph_y;
      x--;
      y -= u8g->glyph_x;

      if ( u8g_IsBBXIntersection(u8g, x-h-1, y-w-1, h, w) == 0 )
        return u8g->glyph_dx;

      ix = x;
      ix -= h;
      ix++;

      for( j = 0; j < h; j++ )
      {
        iy = y;
        for( i = 0; i < bytes_per_line; i++ )
        {
          u8g_Draw8Pixel(u8g, ix, iy, dir, u8g_pgm_read(data));
          data++;
          iy-=8;
        }
        ix++;
      }
      break;
  }
  return u8g->glyph_dx;
}
#endif

int8_t u8g_draw_glyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  const u8g_pgm_uint8_t *data;
  uint8_t w, h;
  uint8_t i, j;
  u8g_uint_t ix, iy;

  {
    u8g_glyph_t g = u8g_GetGlyph(u8g, encoding);
    if ( g == NULL  )
      return 0;
    data = u8g_font_GetGlyphDataStart(u8g->font, g);
  }

  w = u8g->glyph_width;
  h = u8g->glyph_height;

  x += u8g->glyph_x;
  y -= u8g->glyph_y;
  y--;

  if ( u8g_IsBBXIntersection(u8g, x, y-h+1, w, h) == 0 )
    return u8g->glyph_dx;

  /* now, w is reused as bytes per line */
  w += 7;
  w /= 8;

  iy = y;
  iy -= h;
  iy++;

  for( j = 0; j < h; j++ )
  {
    ix = x;
    for( i = 0; i < w; i++ )
    {
      u8g_Draw8Pixel(u8g, ix, iy, 0, u8g_pgm_read(data));
      data++;
      ix+=8;
    }
    iy++;
  }
  return u8g->glyph_dx;
}

int8_t u8g_DrawGlyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  y += u8g->font_calc_vref(u8g);
  return u8g_draw_glyph(u8g, x, y, encoding);
}

int8_t u8g_draw_glyph90(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  const u8g_pgm_uint8_t *data;
  uint8_t w, h;
  uint8_t i, j;
  u8g_uint_t ix, iy;

  {
    u8g_glyph_t g = u8g_GetGlyph(u8g, encoding);
    if ( g == NULL  )
      return 0;
    data = u8g_font_GetGlyphDataStart(u8g->font, g);
  }

  w = u8g->glyph_width;
  h = u8g->glyph_height;

  x += u8g->glyph_y;
  x++;
  y += u8g->glyph_x;

  if ( u8g_IsBBXIntersection(u8g, x, y, h, w) == 0 )
    return u8g->glyph_dx;

  /* now, w is reused as bytes per line */
  w += 7;
  w /= 8;

  ix = x;
  ix += h;
  ix--;
  for( j = 0; j < h; j++ )
  {
    iy = y;
    for( i = 0; i < w; i++ )
    {
      u8g_Draw8Pixel(u8g, ix, iy, 1, u8g_pgm_read(data));
      data++;
      iy+=8;
    }
    ix--;
  }
  return u8g->glyph_dx;
}

int8_t u8g_DrawGlyph90(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  x -= u8g->font_calc_vref(u8g);
  return u8g_draw_glyph90(u8g, x, y, encoding);
}


int8_t u8g_draw_glyph180(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  const u8g_pgm_uint8_t *data;
  uint8_t w, h;
  uint8_t i, j;
  u8g_uint_t ix, iy;

  {
    u8g_glyph_t g = u8g_GetGlyph(u8g, encoding);
    if ( g == NULL  )
      return 0;
    data = u8g_font_GetGlyphDataStart(u8g->font, g);
  }

  w = u8g->glyph_width;
  h = u8g->glyph_height;

  x -= u8g->glyph_x;
  y += u8g->glyph_y;
  y++;

  if ( u8g_IsBBXIntersection(u8g, x-(w-1), y, w, h) == 0 )
    return u8g->glyph_dx;

  /* now, w is reused as bytes per line */
  w += 7;
  w /= 8;

  iy = y;
  iy += h;
  iy--;
  for( j = 0; j < h; j++ )
  {
    ix = x;
    for( i = 0; i < w; i++ )
    {
      u8g_Draw8Pixel(u8g, ix, iy, 2, u8g_pgm_read(data));
      data++;
      ix-=8;
    }
    iy--;
  }
  return u8g->glyph_dx;
}

int8_t u8g_DrawGlyph180(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  y -= u8g->font_calc_vref(u8g);
  return u8g_draw_glyph180(u8g, x, y, encoding);
}


int8_t u8g_draw_glyph270(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  const u8g_pgm_uint8_t *data;
  uint8_t w, h;
  uint8_t i, j;
  u8g_uint_t ix, iy;

  {
    u8g_glyph_t g = u8g_GetGlyph(u8g, encoding);
    if ( g == NULL  )
      return 0;
    data = u8g_font_GetGlyphDataStart(u8g->font, g);
  }

  w = u8g->glyph_width;
  h = u8g->glyph_height;

  x -= u8g->glyph_y;
  x--;
  y -= u8g->glyph_x;

  if ( u8g_IsBBXIntersection(u8g, x-(h-1), y-(w-1), h, w) == 0 )
    return u8g->glyph_dx;


  /* now, w is reused as bytes per line */
  w += 7;
  w /= 8;

  ix = x;
  ix -= h;
  ix++;

  for( j = 0; j < h; j++ )
  {
    iy = y;
    for( i = 0; i < w; i++ )
    {
      u8g_Draw8Pixel(u8g, ix, iy, 3, u8g_pgm_read(data));
      data++;
      iy-=8;
    }
    ix++;
  }
  return u8g->glyph_dx;
}

int8_t u8g_DrawGlyph270(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  x += u8g->font_calc_vref(u8g);
  return u8g_draw_glyph270(u8g, x, y, encoding);
}



#ifdef OBSOLETE
/*
  Draw a glyph
  x,y: lower left corner of the font bounding box
*/
int8_t u8g_DrawGlyphFontBBX(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t encoding)
{
  /* TODO: apply "dir" */
  x -= u8g_GetFontBBXOffX(u8g);
  y += u8g_GetFontBBXOffY(u8g);
  return u8g_DrawGlyphDir(u8g, x, y, dir, encoding);
}
#endif

/*========================================================================*/
/* string drawing procedures */


u8g_uint_t u8g_DrawStr(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  //u8g_uint_t u8g_GetStrWidth(u8g, s);
  //u8g_font_GetFontAscent(u8g->font)-u8g_font_GetFontDescent(u8g->font);

  y += u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_draw_glyph(u8g, x, y, *s);
    x += d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr90(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  x -= u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_draw_glyph90(u8g, x, y, *s);
    y += d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr180(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  y -= u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_draw_glyph180(u8g, x, y, *s);
    x -= d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr270(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  x += u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_draw_glyph270(u8g, x, y, *s);
    y -= d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStrDir(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, const char *s)
{
  switch(dir)
  {
    case 0:
      return u8g_DrawStr(u8g, x, y, s);
    case 1:
      return u8g_DrawStr90(u8g, x, y, s);
    case 2:
      return u8g_DrawStr180(u8g, x, y, s);
    case 3:
      return u8g_DrawStr270(u8g, x, y, s);
  }
  return 0;
}

u8g_uint_t u8g_DrawStrP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s)
{
  u8g_uint_t t = 0;
  int8_t d;
  uint8_t c;

  y += u8g->font_calc_vref(u8g);

  for(;;)
  {
    c = u8g_pgm_read(s);
    if ( c == '\0' )
      break;
    d = u8g_draw_glyph(u8g, x, y, c);
    x += d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr90P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  x -= u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_DrawGlyph90(u8g, x, y, u8g_pgm_read(s));
    y += d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr180P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  y -= u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_DrawGlyph180(u8g, x, y, u8g_pgm_read(s));
    x -= d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStr270P(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const u8g_pgm_uint8_t *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  x += u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_DrawGlyph270(u8g, x, y, u8g_pgm_read(s));
    y -= d;
    t += d;
    s++;
  }
  return t;
}

u8g_uint_t u8g_DrawStrFontBBX(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, const char *s)
{
  x -= u8g_GetFontBBXOffX(u8g);
  y += u8g_GetFontBBXOffY(u8g);
  return u8g_DrawStrDir(u8g, x, y, dir, s);
}

/* still used by picgen.c, dir argument is ignored */
int8_t u8g_DrawGlyphFontBBX(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t encoding)
{
  x -= u8g_GetFontBBXOffX(u8g);
  y += u8g_GetFontBBXOffY(u8g);
  u8g_draw_glyph(u8g, x, y, encoding);
  return 0;
}


/*========================================================================*/
/* set ascent/descent for reference point calculation */

void u8g_UpdateRefHeight(u8g_t *u8g)
{
  uint16_t ls;
  if ( u8g->font == NULL )
    return;
  if ( u8g->font_height_mode == U8G_FONT_HEIGHT_MODE_TEXT )
  {
    u8g->font_ref_ascent = u8g_font_GetCapitalAHeight(u8g->font);
    u8g->font_ref_descent = u8g_font_GetLowerGDescent(u8g->font);
  }
  else if ( u8g->font_height_mode == U8G_FONT_HEIGHT_MODE_XTEXT )
  {
    u8g->font_ref_ascent = u8g_font_GetFontXAscent(u8g->font);
    u8g->font_ref_descent = u8g_font_GetFontXDescent(u8g->font);
  }
  else
  {
    u8g->font_ref_ascent = u8g_font_GetFontAscent(u8g->font);
    u8g->font_ref_descent = u8g_font_GetFontDescent(u8g->font);
  }

  ls = u8g->font_ref_ascent - u8g->font_ref_descent;
  if ( u8g->font_line_spacing_factor != 64 )
  {
    ls &= 255;
    ls *= u8g->font_line_spacing_factor;
    ls >>= 6;
  }
  u8g->line_spacing = ls;
}

void u8g_SetFontRefHeightText(u8g_t *u8g)
{
  u8g->font_height_mode = U8G_FONT_HEIGHT_MODE_TEXT;
  u8g_UpdateRefHeight(u8g);
}

void u8g_SetFontRefHeightExtendedText(u8g_t *u8g)
{
  u8g->font_height_mode = U8G_FONT_HEIGHT_MODE_XTEXT;
  u8g_UpdateRefHeight(u8g);
}


void u8g_SetFontRefHeightAll(u8g_t *u8g)
{
  u8g->font_height_mode = U8G_FONT_HEIGHT_MODE_ALL;
  u8g_UpdateRefHeight(u8g);
}

/* factor = 64: linespaceing == ascent and descent */
void u8g_SetFontLineSpacingFactor(u8g_t *u8g, uint8_t  factor)
{
  u8g->font_line_spacing_factor = factor;
  u8g_UpdateRefHeight(u8g);
}



/*========================================================================*/
/* callback procedures to correct the y position */

u8g_uint_t u8g_font_calc_vref_font(u8g_t *u8g)
{
  return 0;
}

void u8g_SetFontPosBaseline(u8g_t *u8g)
{
  u8g->font_calc_vref = u8g_font_calc_vref_font;
}


u8g_uint_t u8g_font_calc_vref_bottom(u8g_t *u8g)
{
  /* y += (u8g_uint_t)(u8g_int_t)(u8g->font_ref_descent); */
  return (u8g_uint_t)(u8g_int_t)(u8g->font_ref_descent);
}

void u8g_SetFontPosBottom(u8g_t *u8g)
{
  u8g->font_calc_vref = u8g_font_calc_vref_bottom;
}

u8g_uint_t u8g_font_calc_vref_top(u8g_t *u8g)
{
  u8g_uint_t tmp;
  /* reference pos is one pixel above the upper edge of the reference glyph */

  /*
  y += (u8g_uint_t)(u8g_int_t)(u8g->font_ref_ascent);
  y++;
  */
  tmp = (u8g_uint_t)(u8g_int_t)(u8g->font_ref_ascent);
  tmp++;
  return tmp;
}

void u8g_SetFontPosTop(u8g_t *u8g)
{
  u8g->font_calc_vref = u8g_font_calc_vref_top;
}

u8g_uint_t u8g_font_calc_vref_center(u8g_t *u8g)
{
  int8_t tmp;
  tmp = u8g->font_ref_ascent;
  tmp -= u8g->font_ref_descent;
  tmp /= 2;
  tmp += u8g->font_ref_descent;
  /* y += (u8g_uint_t)(u8g_int_t)(tmp); */
  return tmp;
}

void u8g_SetFontPosCenter(u8g_t *u8g)
{
  u8g->font_calc_vref = u8g_font_calc_vref_center;
}

/*========================================================================*/
/* string pixel width calculation */

char u8g_font_get_char(const void *s)
{
  return *(const char *)(s);
}

char u8g_font_get_charP(const void *s)
{
  return u8g_pgm_read(s);
}

typedef char (*u8g_font_get_char_fn)(const void *s);


u8g_uint_t u8g_font_calc_str_pixel_width(u8g_t *u8g, const char *s, u8g_font_get_char_fn get_char )
{
  u8g_uint_t  w;
  uint8_t enc;

  /* reset the total minimal width to zero, this will be expanded during calculation */
  w = 0;

  enc = get_char(s);

  /* check for empty string, width is already 0 */
  if ( enc == '\0' )
  {
    return w;
  }

  /* get the glyph information of the first char. This must be valid, because we already checked for the empty string */
  /* if *s is not inside the font, then the cached parameters of the glyph are all zero */
  u8g_GetGlyph(u8g, enc);

  /* strlen(s) == 1:       width = width(s[0]) */
  /* strlen(s) == 2:       width = - offx(s[0]) + deltax(s[0]) + offx(s[1]) + width(s[1]) */
  /* strlen(s) == 3:       width = - offx(s[0]) + deltax(s[0]) + deltax(s[1]) + offx(s[2]) + width(s[2]) */

  /* assume that the string has size 2 or more, than start with negative offset-x */
  /* for string with size 1, this will be nullified after the loop */
  w = -u8g->glyph_x;
  for(;;)
  {

    /* check and stop if the end of the string is reached */
    s++;
    if ( get_char(s) == '\0' )
      break;

    /* if there are still more characters, add the delta to the next glyph */
    w += u8g->glyph_dx;

    /* store the encoding in a local variable, used also after the for(;;) loop */
    enc = get_char(s);

    /* load the next glyph information */
    u8g_GetGlyph(u8g, enc);
  }

  /* finally calculate the width of the last char */
  /* here is another exception, if the last char is a black, use the dx value instead */
  if ( enc != ' ' )
  {
    /* if g was not updated in the for loop (strlen() == 1), then the initial offset x gets removed */
    w += u8g->glyph_width;
    w += u8g->glyph_x;
  }
  else
  {
    w += u8g->glyph_dx;
  }


  return w;
}

u8g_uint_t u8g_GetStrPixelWidth(u8g_t *u8g, const char *s)
{
  return u8g_font_calc_str_pixel_width(u8g, s, u8g_font_get_char);
}

u8g_uint_t u8g_GetStrPixelWidthP(u8g_t *u8g, const u8g_pgm_uint8_t *s)
{
  return u8g_font_calc_str_pixel_width(u8g, (const char *)s, u8g_font_get_charP);
}

int8_t u8g_GetStrX(u8g_t *u8g, const char *s)
{
  u8g_GetGlyph(u8g, *s);
  return u8g->glyph_x;
}

int8_t u8g_GetStrXP(u8g_t *u8g, const u8g_pgm_uint8_t *s)
{
  u8g_GetGlyph(u8g, u8g_pgm_read(s));
  return u8g->glyph_x;
}

/*========================================================================*/
/* string width calculation */

u8g_uint_t u8g_GetStrWidth(u8g_t *u8g, const char *s)
{
  u8g_uint_t  w;
  uint8_t encoding;

  /* reset the total width to zero, this will be expanded during calculation */
  w = 0;

  for(;;)
  {
    encoding = *s;
    if ( encoding == 0 )
      break;

    /* load glyph information */
    u8g_GetGlyph(u8g, encoding);
    w += u8g->glyph_dx;

    /* goto next char */
    s++;
  }

  return w;
}


u8g_uint_t u8g_GetStrWidthP(u8g_t *u8g, const u8g_pgm_uint8_t *s)
{
  u8g_uint_t  w;
  uint8_t encoding;

  /* reset the total width to zero, this will be expanded during calculation */
  w = 0;

  for(;;)
  {
    encoding = u8g_pgm_read(s);
    if ( encoding == 0 )
      break;

    /* load glyph information */
    u8g_GetGlyph(u8g, encoding);
    w += u8g->glyph_dx;

    /* goto next char */
    s++;
  }

  return w;
}


/*========================================================================*/
/* calculation of font/glyph/string characteristics */


/*
  Description:
    Calculate parameter for the minimal bounding box on a given string
  Output
    buf->y_min          extend of the lower left edge if the string below (y_min<0) or above (y_min>0) baseline (descent)
    buf->y_max          extend of the upper left edge if the string below (y_min<0) or above (y_min>0) baseline (ascent)
    buf->w                 the width of the string
*/
struct u8g_str_size_struct
{
  int8_t y_min;         /* descent */
  int8_t y_max;         /* ascent */
  int8_t x, y;             /* the reference point of the font (negated!) */
  u8g_uint_t w;         /* width of the overall string */
};
typedef struct u8g_str_size_struct u8g_str_size_t;

static void u8g_font_calc_str_min_box(u8g_t *u8g, const char *s, u8g_str_size_t *buf)
{
  /* u8g_glyph_t g; */
  int8_t tmp;

  /* reset the total minimal width to zero, this will be expanded during calculation */
  buf->w = 0;

  /* check for empty string, width is already 0, but also reset y_min and y_max to 0 */
  if ( *s == '\0' )
  {
    buf->y_min = 0;
    buf->y_max = 0;
    buf->x = 0;
    buf->y = 0;
    return;
  }

  /* reset y_min to the largest possible value. Later we search for the smallest value */
  /* y_min contains the position [pixel] of the lower left edge of the glyph above (y_min>0) or below (y_min<0) baseline  */
  buf->y_min = 127;
  /* reset y_max to the smallest possible value. Later we search for the highest value */
  /* y_max contains the position [pixel] of the upper left edge of the glyph above (y_max>0) or below (y_max<0) baseline  */
  buf->y_max = -128;

  /* get the glyph information of the first char. This must be valid, because we already checked for the empty string */
  u8g_GetGlyph(u8g, *s);

  /* strlen(s) == 1:       width = width(s[0]) */
  /* strlen(s) == 2:       width = - offx(s[0]) + deltax(s[0]) + offx(s[1]) + width(s[1]) */
  /* strlen(s) == 3:       width = - offx(s[0]) + deltax(s[0]) + deltax(s[1]) + offx(s[2]) + width(s[2]) */

  /* assume that the string has size 2 or more, than start with negative offset-x */
  /* for string with size 1, this will be nullified after the loop */
  // buf->w = - u8g_font_GetGlyphBBXOffX(u8g->font, g);
  buf->w = - u8g->glyph_x;

  /* Also copy the position of the first glyph. This is the reference point of the string (negated) */
  buf->x = u8g->glyph_x;
  buf->y = u8g->glyph_y;

  for(;;)
  {

    /* calculated y position of the upper left corner (y_max) and lower left corner (y_min) of the string */
    /* relative to the base line */

    tmp = u8g->glyph_y;
    if ( buf->y_min > tmp )
      buf->y_min = tmp;

    tmp +=u8g->glyph_height;
    if ( buf->y_max < tmp )
      buf->y_max = tmp;

    /* check and stop if the end of the string is reached */
    s++;
    if ( *s == '\0' )
      break;

    /* if there are still more characters, add the delta to the next glyph */
    buf->w += u8g->glyph_dx;

    /* load the next glyph information */
    u8g_GetGlyph(u8g, *s);
  }

  /* finally calculate the width of the last char */
  /* if g was not updated in the for loop (strlen() == 1), then the initial offset x gets removed */
  buf->w += u8g->glyph_width;
  // buf->w += u8g_font_GetGlyphBBXOffX(u8g->font, g);

  buf->w += u8g->glyph_x;
}

/* calculate minimal box */
void u8g_font_box_min(u8g_t *u8g, const char *s, u8g_str_size_t *buf)
{
  u8g_font_calc_str_min_box(u8g, s, buf);
}

/* calculate gA box, but do not calculate the overall width */
void u8g_font_box_left_gA(u8g_t *u8g, const char *s, u8g_str_size_t *buf)
{

}

/* calculate gA box, including overall width */
void u8g_font_box_all_gA(u8g_t *u8g, const char *s, u8g_str_size_t *buf)
{

}


static void u8g_font_get_str_box_fill_args(u8g_t *u8g, const char *s, u8g_str_size_t *buf, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height)
{
  /*
  u8g_glyph_t g;
  g =
  */
  u8g_GetGlyph(u8g, *s);
  *x += u8g->glyph_x;
  *width = buf->w;
  *y -= buf->y_max;
  /* +1 because y_max is a height, this compensates the next step */
  //*y += 1;
  /* because the reference point is one below the string, this compensates the previous step */
  //*y -= 1;
  *height = buf->y_max;
  *height -= buf->y_min;
}


void u8g_GetStrMinBox(u8g_t *u8g, const char *s, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height)
{
  u8g_str_size_t buf;

  if ( *s == '\0' )
  {
    *width= 0;
    *height = 0;
    return;
  }

  u8g_font_calc_str_min_box(u8g, s, &buf);
  u8g_font_get_str_box_fill_args(u8g, s, &buf, x, y, width, height);
}


void u8g_GetStrAMinBox(u8g_t *u8g, const char *s, u8g_uint_t *x, u8g_uint_t *y, u8g_uint_t *width, u8g_uint_t *height)
{
  u8g_str_size_t buf;
  uint8_t cap_a;

  if ( *s == '\0' )
  {
    *width= 0;
    *height = 0;
    return;
  }

  cap_a = u8g_font_GetCapitalAHeight(u8g->font);
  u8g_font_calc_str_min_box(u8g, s, &buf);
  if ( buf.y_max < cap_a )
    buf.y_max = cap_a;
  u8g_font_get_str_box_fill_args(u8g, s, &buf, x, y, width, height);
}

void u8g_SetFont(u8g_t *u8g, const u8g_fntpgm_uint8_t  *font)
{
  if ( u8g->font != font )
  {
    u8g->font = font;
    u8g_UpdateRefHeight(u8g);
    u8g_SetFontPosBaseline(u8g);
  }
}

/*========================================================================*/
/* anti aliasing fonts */

int8_t u8g_draw_aa_glyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  const u8g_pgm_uint8_t *data;
  uint8_t w, h;
  uint8_t i, j;
  u8g_uint_t ix, iy;

  {
    u8g_glyph_t g = u8g_GetGlyph(u8g, encoding);
    if ( g == NULL  )
      return 0;
    data = u8g_font_GetGlyphDataStart(u8g->font, g);
  }

  w = u8g->glyph_width;
  h = u8g->glyph_height;

  x += u8g->glyph_x;
  y -= u8g->glyph_y;
  y--;

  if ( u8g_IsBBXIntersection(u8g, x, y-h+1, w, h) == 0 )
    return u8g->glyph_dx;

  /* now, w is reused as bytes per line */
  w += 3;
  w /= 4;

  iy = y;
  iy -= h;
  iy++;

  for( j = 0; j < h; j++ )
  {
    ix = x;
    for( i = 0; i < w; i++ )
    {
      u8g_Draw4TPixel(u8g, ix, iy, 0, u8g_pgm_read(data));
      data++;
      ix+=4;
    }
    iy++;
  }
  return u8g->glyph_dx;
}

int8_t u8g_DrawAAGlyph(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t encoding)
{
  y += u8g->font_calc_vref(u8g);
  return u8g_draw_aa_glyph(u8g, x, y, encoding);
}

u8g_uint_t u8g_DrawAAStr(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, const char *s)
{
  u8g_uint_t t = 0;
  int8_t d;

  if ( u8g_font_GetFormat(u8g->font)  != 2 )
    return 0;
  //u8g_uint_t u8g_GetStrWidth(u8g, s);
  //u8g_font_GetFontAscent(u8g->font)-u8g_font_GetFontDescent(u8g->font);

  y += u8g->font_calc_vref(u8g);

  while( *s != '\0' )
  {
    d = u8g_draw_aa_glyph(u8g, x, y, *s);
    x += d;
    t += d;
    s++;
  }
  return t;
}

// ============== u8g_page.c ================

/*

  u8g_page.c

  page helper functions, only called by the dev handler.

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/

/*
  setup page count structure
  conditions: page_height <= total_height
*/
void u8g_page_Init(u8g_page_t *p, u8g_uint_t page_height, u8g_uint_t total_height )
{
  p->page_height = page_height;
  p->total_height = total_height;
  p->page = 0;
  u8g_page_First(p);
}

void u8g_page_First(u8g_page_t *p)
{
  p->page_y0 = 0;
  p->page_y1 = p->page_height;
  p->page_y1--;
  p->page = 0;
}

uint8_t u8g_page_Next(u8g_page_t * p)
{
  register u8g_uint_t y1;
  p->page_y0 += p->page_height;
  if ( p->page_y0 >= p->total_height )
    return 0;
  p->page++;
  y1 = p->page_y1;
  y1 += p->page_height;
  if ( y1 >= p->total_height )
  {
    y1 = p->total_height;
    y1--;
  }
  p->page_y1 = y1;

  return 1;
}

// =============== u8g_clip.c ================

/*

  u8g_clip.c

  procedures for clipping
  taken over from procs in u8g_pb.c

  Universal 8bit Graphics Library

  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Notes

  This is one of the most critical parts of u8glib. It must be fast, but still reliable.
  Based on the intersection program (see tools folder), there is minimized version of
  the condition for the intersaction test:
    minimized version
    ---1----0 1             b1 <= a2 && b1 > b2
    -----1--0 1             b2 >= a1 && b1 > b2
    ---1-1--- 1             b1 <= a2 && b2 >= a1
  It includes the assumption, that a1 <= a2 is always true (correct, because
  a1, a2 are the page dimensions.

  The direct implementation of the above result is done in:
  uint8_t u8g_is_intersection_boolean(u8g_uint_t a0, u8g_uint_t a1, u8g_uint_t v0, u8g_uint_t v1)
  However, this is slower than a decision tree version:
  static uint8_t u8g_is_intersection_decision_tree(u8g_uint_t a0, u8g_uint_t a1, u8g_uint_t v0, u8g_uint_t v1)
  Also suprising is, that the macro implementation is slower than the inlined version.

  The decision tree is based on the expansion of the truth table.

*/


#ifdef __GNUC__
#define U8G_ALWAYS_INLINE __inline__ __attribute__((always_inline))
#else
#define U8G_ALWAYS_INLINE
 #endif

/*
  intersection assumptions:
    a1 <= a2 is always true

    minimized version
    ---1----0 1             b1 <= a2 && b1 > b2
    -----1--0 1             b2 >= a1 && b1 > b2
    ---1-1--- 1             b1 <= a2 && b2 >= a1
  */

#ifdef OLD_CODE_WHICH_IS_TOO_SLOW
static uint8_t u8g_is_intersection_boolean(u8g_uint_t a0, u8g_uint_t a1, u8g_uint_t v0, u8g_uint_t v1)
{
  uint8_t c1, c2, c3, tmp;
  c1 = v0 <= a1;
  c2 = v1 >= a0;
  c3 = v0 > v1;

  tmp = c1;
  c1 &= c2;
  c2 &= c3;
  c3 &= tmp;
  c1 |= c2;
  c1 |= c3;
  return c1 & 1;
}
#endif

#define U8G_IS_INTERSECTION_MACRO(a0,a1,v0,v1) ((uint8_t)( (v0) <= (a1) ) ? ( ( (v1) >= (a0) ) ? ( 1 ) : ( (v0) > (v1) ) ) : ( ( (v1) >= (a0) ) ? ( (v0) > (v1) ) : ( 0 ) ))

//static uint8_t u8g_is_intersection_decision_tree(u8g_uint_t a0, u8g_uint_t a1, u8g_uint_t v0, u8g_uint_t v1) U8G_ALWAYS_INLINE;
static uint8_t U8G_ALWAYS_INLINE u8g_is_intersection_decision_tree(u8g_uint_t a0, u8g_uint_t a1, u8g_uint_t v0, u8g_uint_t v1)
{
  /* surprisingly the macro leads to larger code */
  /* return U8G_IS_INTERSECTION_MACRO(a0,a1,v0,v1); */
  if ( v0 <= a1 )
  {
    if ( v1 >= a0 )
    {
      return 1;
    }
    else
    {
      if ( v0 > v1 )
      {
	return 1;
      }
      else
      {
	return 0;
      }
    }
  }
  else
  {
    if ( v1 >= a0 )
    {
      if ( v0 > v1 )
      {
	return 1;
      }
      else
      {
	return 0;
      }
    }
    else
    {
      return 0;
    }
  }
}


uint8_t u8g_IsBBXIntersection(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  register u8g_uint_t tmp;
  tmp = y;
  tmp += h;
  tmp--;
  if ( u8g_is_intersection_decision_tree(u8g->current_page.y0, u8g->current_page.y1, y, tmp) == 0 )
    return 0;

  tmp = x;
  tmp += w;
  tmp--;
  return u8g_is_intersection_decision_tree(u8g->current_page.x0, u8g->current_page.x1, x, tmp);
}

// ============= u8g_com_arduino_std_sw_spi.c =====================

/*

  u8g_arduino_std_sw_spi.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

void u8g_arduino_sw_spi_shift_out(uint8_t dataPin, uint8_t clockPin, uint8_t val)
{
  uint8_t i = 8;
  do
  {
    if ( val & 128 )
      digitalWrite(dataPin, HIGH);
    else
      digitalWrite(dataPin, LOW);
    val <<= 1;
    u8g_MicroDelay();		/* 23 Sep 2012 */
    //delay(1);
    digitalWrite(clockPin, HIGH);
    u8g_MicroDelay();		/* 23 Sep 2012 */
    //delay(1);
    digitalWrite(clockPin, LOW);
    u8g_MicroDelay();		/* 23 Sep 2012 */
    //delay(1);
    i--;
  } while( i != 0 );
}

uint8_t u8g_com_arduino_std_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      u8g_com_arduino_assign_pin_output_high(u8g);
      u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW);
      u8g_com_arduino_digital_write(u8g, U8G_PI_MOSI, LOW);
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
        u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      if ( arg_val == 0 )
      {
        /* disable */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, HIGH);
      }
      else
      {
        /* enable */
        u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, LOW);
      }
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ:
      {
        register uint8_t *ptr = (uint8_t*)arg_ptr;
        while( arg_val > 0 )
        {
          u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], *ptr++);
          arg_val--;
        }
      }
      break;

      case U8G_COM_MSG_WRITE_SEQ_P:
      {
        register uint8_t *ptr = (uint8_t*)arg_ptr;
        while( arg_val > 0 )
        {
          u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], u8g_pgm_read(ptr));
          ptr++;
          arg_val--;
        }
      }
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g_com_arduino_digital_write(u8g, U8G_PI_A0, arg_val);
      break;
  }
  return 1;
}


// ======== u8g_com_atmega_st7920_spi.c ==============

/*

  u8g_com_atmega_st7920_spi.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  A special SPI interface for ST7920 controller

*/

#if defined(__AVR__)

static void u8g_atmega_st7920_sw_spi_shift_out(u8g_t *u8g, uint8_t val) U8G_NOINLINE;
static void u8g_atmega_st7920_sw_spi_shift_out(u8g_t *u8g, uint8_t val)
{
  uint8_t i = 8;
  do
  {
    u8g_SetPILevel(u8g, U8G_PI_MOSI, val & 128 );
    val <<= 1;
    u8g_SetPILevel(u8g, U8G_PI_SCK, 1 );
    u8g_MicroDelay();		/* 15 Aug 2012: added for high speed uC */
    u8g_SetPILevel(u8g, U8G_PI_SCK, 0 );
    u8g_MicroDelay();		/* 15 Aug 2012: added for high speed uC */
    i--;
  } while( i != 0 );
}

static void u8g_com_atmega_st7920_write_byte(u8g_t *u8g, uint8_t rs, uint8_t val) U8G_NOINLINE;
static void u8g_com_atmega_st7920_write_byte(u8g_t *u8g, uint8_t rs, uint8_t val)
{
  uint8_t i;

  if ( rs == 0 )
  {
    /* command */
    u8g_atmega_st7920_sw_spi_shift_out(u8g, 0x0f8);
  }
  else if ( rs == 1 )
  {
    /* data */
    u8g_atmega_st7920_sw_spi_shift_out(u8g, 0x0fa);
  }

  u8g_atmega_st7920_sw_spi_shift_out(u8g, val & 0x0f0);
  u8g_atmega_st7920_sw_spi_shift_out(u8g, val << 4);

  for( i = 0; i < 4; i++ )
    u8g_10MicroDelay();
}


uint8_t u8g_com_atmega_st7920_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      u8g_SetPIOutput(u8g, U8G_PI_SCK);
      u8g_SetPIOutput(u8g, U8G_PI_MOSI);
      /* u8g_SetPIOutput(u8g, U8G_PI_A0); */
      u8g_SetPIOutput(u8g, U8G_PI_CS);
      u8g_SetPIOutput(u8g, U8G_PI_RESET);

      u8g_SetPILevel(u8g, U8G_PI_SCK, 0 );
      u8g_SetPILevel(u8g, U8G_PI_MOSI, 0 );
      u8g_SetPILevel(u8g, U8G_PI_CS, 0 );
      /* u8g_SetPILevel(u8g, U8G_PI_A0, 0); */

      u8g->pin_list[U8G_PI_A0_STATE] = 0;       /* inital RS state: command mode */
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      u8g_SetPILevel(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      if ( arg_val == 0 )
      {
        /* disable, note: the st7920 has an active high chip select */
        u8g_SetPILevel(u8g, U8G_PI_CS, 0);
      }
      else
      {
        /* u8g_SetPILevel(u8g, U8G_PI_SCK, 0 ); */
        /* enable */
        u8g_SetPILevel(u8g, U8G_PI_CS, 1); /* CS = 1 (high active) */
      }
      break;


    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_atmega_st7920_write_byte(u8g, u8g->pin_list[U8G_PI_A0_STATE], arg_val);
      u8g->pin_list[U8G_PI_A0_STATE] = 2;
      break;

    case U8G_COM_MSG_WRITE_SEQ:
      {
        register uint8_t *ptr = (uint8_t*)arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_atmega_st7920_write_byte(u8g, u8g->pin_list[U8G_PI_A0_STATE], *ptr++);
	  u8g->pin_list[U8G_PI_A0_STATE] = 2;
          arg_val--;
        }
      }
      break;

      case U8G_COM_MSG_WRITE_SEQ_P:
      {
        register uint8_t *ptr = (uint8_t*)arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_atmega_st7920_write_byte(u8g, u8g->pin_list[U8G_PI_A0_STATE], u8g_pgm_read(ptr));
	  u8g->pin_list[U8G_PI_A0_STATE] = 2;
          ptr++;
          arg_val--;
        }
      }
      break;
  }
  return 1;
}

#else


uint8_t u8g_com_atmega_st7920_sw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  return 1;
}


#endif

// ================ u8g_butmap.c ===============

/*

  u8g_bitmap.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


void u8g_DrawHBitmap(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, const uint8_t *bitmap)
{
  while( cnt > 0 )
  {
    u8g_Draw8Pixel(u8g, x, y, 0, *bitmap);
    bitmap++;
    cnt--;
    x+=8;
  }
}

void u8g_DrawBitmap(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, u8g_uint_t h, const uint8_t *bitmap)
{
  if ( u8g_IsBBXIntersection(u8g, x, y, cnt*8, h) == 0 )
    return;
  while( h > 0 )
  {
    u8g_DrawHBitmap(u8g, x, y, cnt, bitmap);
    bitmap += cnt;
    y++;
    h--;
  }
}


void u8g_DrawHBitmapP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, const u8g_pgm_uint8_t *bitmap)
{
  while( cnt > 0 )
  {
    u8g_Draw8Pixel(u8g, x, y, 0, u8g_pgm_read(bitmap));
    bitmap++;
    cnt--;
    x+=8;
  }
}

void u8g_DrawBitmapP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t cnt, u8g_uint_t h, const u8g_pgm_uint8_t *bitmap)
{
  if ( u8g_IsBBXIntersection(u8g, x, y, cnt*8, h) == 0 )
    return;
  while( h > 0 )
  {
    u8g_DrawHBitmapP(u8g, x, y, cnt, bitmap);
    bitmap += cnt;
    y++;
    h--;
  }
}

/*=========================================================================*/

static void u8g_DrawHXBM(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, const uint8_t *bitmap)
{
  uint8_t d;
  x+=7;
  while( w >= 8 )
  {
    u8g_Draw8Pixel(u8g, x, y, 2, *bitmap);
    bitmap++;
    w-= 8;
    x+=8;
  }
  if ( w > 0 )
  {
    d = *bitmap;
    x -= 7;
    do
    {
      if ( d & 1 )
        u8g_DrawPixel(u8g, x, y);
      x++;
      w--;
      d >>= 1;
    } while ( w > 0 );
  }
}

void u8g_DrawXBM(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, const uint8_t *bitmap)
{
  u8g_uint_t b;
  b = w;
  b += 7;
  b >>= 3;

  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;

  while( h > 0 )
  {
    u8g_DrawHXBM(u8g, x, y, w, bitmap);
    bitmap += b;
    y++;
    h--;
  }
}

static void u8g_DrawHXBMP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, const u8g_pgm_uint8_t *bitmap)
{
  uint8_t d;
  x+=7;
  while( w >= 8 )
  {
    u8g_Draw8Pixel(u8g, x, y, 2, u8g_pgm_read(bitmap));
    bitmap++;
    w-= 8;
    x+=8;
  }
  if ( w > 0 )
  {
    d = u8g_pgm_read(bitmap);
    x -= 7;
    do
    {
      if ( d & 1 )
        u8g_DrawPixel(u8g, x, y);
      x++;
      w--;
      d >>= 1;
    } while ( w > 0 );
  }
}

void u8g_DrawXBMP(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, const u8g_pgm_uint8_t *bitmap)
{
  u8g_uint_t b;
  b = w;
  b += 7;
  b >>= 3;

  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;
  while( h > 0 )
  {
    u8g_DrawHXBMP(u8g, x, y, w, bitmap);
    bitmap += b;
    y++;
    h--;
  }
}

// ============= u8g_api_ii.c ===============

/*

  u8g_ll_api.c

  low level api

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/

uint8_t u8g_call_dev_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  return dev->dev_fn(u8g, dev, msg, arg);
}

/*====================================================================*/

uint8_t u8g_InitLL(u8g_t *u8g, u8g_dev_t *dev)
{
  uint8_t r;
  u8g->state_cb(U8G_STATE_MSG_BACKUP_ENV);
  r =  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_INIT, NULL);
  u8g->state_cb(U8G_STATE_MSG_BACKUP_U8G);
  u8g->state_cb(U8G_STATE_MSG_RESTORE_ENV);
  return r;
}

void u8g_FirstPageLL(u8g_t *u8g, u8g_dev_t *dev)
{
  u8g->state_cb(U8G_STATE_MSG_BACKUP_ENV);
  u8g->state_cb(U8G_STATE_MSG_RESTORE_U8G);
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_PAGE_FIRST, NULL);
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_GET_PAGE_BOX, &(u8g->current_page));
  u8g->state_cb(U8G_STATE_MSG_RESTORE_ENV);
}

uint8_t u8g_NextPageLL(u8g_t *u8g, u8g_dev_t *dev)
{
  uint8_t r;
  u8g->state_cb(U8G_STATE_MSG_BACKUP_ENV);
  u8g->state_cb(U8G_STATE_MSG_RESTORE_U8G);
  r = u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_PAGE_NEXT, NULL);
  if ( r != 0 )
  {
    u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_GET_PAGE_BOX, &(u8g->current_page));
  }
  u8g->state_cb(U8G_STATE_MSG_RESTORE_ENV);
  return r;
}

uint8_t u8g_SetContrastLL(u8g_t *u8g, u8g_dev_t *dev, uint8_t contrast)
{
  return u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_CONTRAST, &contrast);
}

void u8g_DrawPixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y)
{
  u8g_dev_arg_pixel_t *arg = &(u8g->arg_pixel);
  arg->x = x;
  arg->y = y;
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_SET_PIXEL, arg);
}

void u8g_Draw8PixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel)
{
  u8g_dev_arg_pixel_t *arg = &(u8g->arg_pixel);
  arg->x = x;
  arg->y = y;
  arg->dir = dir;
  arg->pixel = pixel;
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_SET_8PIXEL, arg);
}

void u8g_Draw4TPixelLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel)
{
  u8g_dev_arg_pixel_t *arg = &(u8g->arg_pixel);
  arg->x = x;
  arg->y = y;
  arg->dir = dir;
  arg->pixel = pixel;
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_SET_4TPIXEL, arg);
}


#ifdef U8G_DEV_MSG_IS_BBX_INTERSECTION
uint8_t u8g_IsBBXIntersectionLL(u8g_t *u8g, u8g_dev_t *dev, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  return u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_IS_BBX_INTERSECTION, &arg);
}
#endif



u8g_uint_t u8g_GetWidthLL(u8g_t *u8g, u8g_dev_t *dev)
{
  u8g_uint_t r;
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_GET_WIDTH, &r);
  return r;
}

u8g_uint_t u8g_GetHeightLL(u8g_t *u8g, u8g_dev_t *dev)
{
  u8g_uint_t r;
  u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_GET_HEIGHT, &r);
  return r;
}

u8g_uint_t u8g_GetModeLL(u8g_t *u8g, u8g_dev_t *dev)
{
  return u8g_call_dev_fn(u8g, dev, U8G_DEV_MSG_GET_MODE, NULL);
}



/*====================================================================*/

void u8g_UpdateDimension(u8g_t *u8g)
{
  u8g->width = u8g_GetWidthLL(u8g, u8g->dev);
  u8g->height = u8g_GetHeightLL(u8g, u8g->dev);
  u8g->mode = u8g_GetModeLL(u8g, u8g->dev);
  /* 9 Dec 2012: u8g_scale.c requires update of current page */
  u8g_call_dev_fn(u8g, u8g->dev, U8G_DEV_MSG_GET_PAGE_BOX, &(u8g->current_page));
}

static void u8g_init_data(u8g_t *u8g)
{
  u8g->font = NULL;
  u8g->cursor_font = NULL;
  u8g->cursor_bg_color = 0;
  u8g->cursor_fg_color = 1;
  u8g->cursor_encoding = 34;
  u8g->cursor_fn = (u8g_draw_cursor_fn)0;

#if defined(U8G_WITH_PINLIST)
  {
    uint8_t i;
    for( i = 0; i < U8G_PIN_LIST_LEN; i++ )
      u8g->pin_list[i] = U8G_PIN_NONE;
  }
#endif

  u8g_SetColorIndex(u8g, 1);

  u8g_SetFontPosBaseline(u8g);

  u8g->font_height_mode = U8G_FONT_HEIGHT_MODE_XTEXT;
  u8g->font_ref_ascent = 0;
  u8g->font_ref_descent = 0;
  u8g->font_line_spacing_factor = 64;           /* 64 = 1.0, 77 = 1.2 line spacing factor */
  u8g->line_spacing = 0;

  u8g->state_cb = u8g_state_dummy_cb;

}

uint8_t u8g_Begin(u8g_t *u8g)
{
  /* call and init low level driver and com device */
  if ( u8g_InitLL(u8g, u8g->dev) == 0 )
    return 0;
  /* fetch width and height from the low level */
  u8g_UpdateDimension(u8g);
  return 1;
}

uint8_t u8g_Init(u8g_t *u8g, u8g_dev_t *dev)
{
  u8g_init_data(u8g);
  u8g->dev = dev;

  /* On the Arduino Environment this will lead to two calls to u8g_Begin(), the following line will be called first (by U8glib constructors) */
  /* if - in future releases - this is removed, then still call u8g_UpdateDimension() */
  /* if Arduino call u8g_UpdateDimension else u8g_Begin */
  /* issue 146 */
  return u8g_Begin(u8g);
}

/* special init for pure ARM systems */
uint8_t u8g_InitComFn(u8g_t *u8g, u8g_dev_t *dev, u8g_com_fnptr com_fn)
{
  u8g_init_data(u8g);

#if defined(U8G_WITH_PINLIST)
  {
    uint8_t i;
    for( i = 0; i < U8G_PIN_LIST_LEN; i++ )
      u8g->pin_list[i] = U8G_PIN_DUMMY;
  }
#endif

  u8g->dev = dev;

  /* replace the device procedure with a custom communication procedure */
  u8g->dev->com_fn = com_fn;

  /* On the Arduino Environment this will lead to two calls to u8g_Begin(), the following line will be called first (by U8glib constructors) */
  /* if - in future releases - this is removed, then still call u8g_UpdateDimension() */
  /* if Arduino call u8g_UpdateDimension else u8g_Begin */
  /* issue 146 */
  return u8g_Begin(u8g);
}


#if defined(U8G_WITH_PINLIST)
uint8_t u8g_InitSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t sck, uint8_t mosi, uint8_t cs, uint8_t a0, uint8_t reset)
{

  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;

  /* assign user pins */
  u8g->pin_list[U8G_PI_SCK] = sck;
  u8g->pin_list[U8G_PI_MOSI] = mosi;
  u8g->pin_list[U8G_PI_CS] = cs;
  u8g->pin_list[U8G_PI_A0] = a0;
  u8g->pin_list[U8G_PI_RESET] = reset;

  /* On the Arduino Environment this will lead to two calls to u8g_Begin(), the following line will be called first (by U8glib constructors) */
  /* if - in future releases - this is removed, then still call u8g_UpdateDimension() */
  /* if Arduino call u8g_UpdateDimension else u8g_Begin */
  /* issue 146 */
  return u8g_Begin(u8g);
}

uint8_t u8g_InitHWSPI(u8g_t *u8g, u8g_dev_t *dev, uint8_t cs, uint8_t a0, uint8_t reset)
{
  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;


  /* assign user pins */
  u8g->pin_list[U8G_PI_CS] = cs;
  u8g->pin_list[U8G_PI_A0] = a0;
  u8g->pin_list[U8G_PI_RESET] = reset;

  return u8g_Begin(u8g);
}

uint8_t u8g_InitI2C(u8g_t *u8g, u8g_dev_t *dev, uint8_t options)
{
  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;

  u8g->pin_list[U8G_PI_I2C_OPTION] = options;

  return u8g_Begin(u8g);
}


uint8_t u8g_Init8BitFixedPort(u8g_t *u8g, u8g_dev_t *dev, uint8_t en, uint8_t cs, uint8_t di, uint8_t rw, uint8_t reset)
{

  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;

  /* assign user pins */

  u8g->pin_list[U8G_PI_EN] = en;
  u8g->pin_list[U8G_PI_CS] = cs;
  u8g->pin_list[U8G_PI_DI] = di;
  u8g->pin_list[U8G_PI_RW] = rw;
  u8g->pin_list[U8G_PI_RESET] = reset;

  return u8g_Begin(u8g);
}

uint8_t u8g_Init8Bit(u8g_t *u8g, u8g_dev_t *dev, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
  uint8_t en, uint8_t cs1, uint8_t cs2, uint8_t di, uint8_t rw, uint8_t reset)
{

  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;

  /* assign user pins */

  u8g->pin_list[U8G_PI_D0] = d0;
  u8g->pin_list[U8G_PI_D1] = d1;
  u8g->pin_list[U8G_PI_D2] = d2;
  u8g->pin_list[U8G_PI_D3] = d3;
  u8g->pin_list[U8G_PI_D4] = d4;
  u8g->pin_list[U8G_PI_D5] = d5;
  u8g->pin_list[U8G_PI_D6] = d6;
  u8g->pin_list[U8G_PI_D7] = d7;

  u8g->pin_list[U8G_PI_EN] = en;
  u8g->pin_list[U8G_PI_CS1] = cs1;
  u8g->pin_list[U8G_PI_CS2] = cs2;
  u8g->pin_list[U8G_PI_DI] = di;
  u8g->pin_list[U8G_PI_RW] = rw;
  u8g->pin_list[U8G_PI_RESET] = reset;

  return u8g_Begin(u8g);
}

/*

  PIN_D0 8
  PIN_D1 9
  PIN_D2 10
  PIN_D3 11
  PIN_D4 4
  PIN_D5 5
  PIN_D6 6
  PIN_D7 7

  PIN_CS 14
  PIN_A0 15
  PIN_RESET 16
  PIN_WR 17
  PIN_RD 18

  u8g_InitRW8Bit(u8g, dev, d0, d1, d2, d3, d4, d5, d6, d7, cs, a0, wr, rd, reset)
  u8g_InitRW8Bit(u8g, dev,  8,  9, 10, 11,  4,  5,  6,  7, 14, 15, 17, 18, 16)

*/

uint8_t u8g_InitRW8Bit(u8g_t *u8g, u8g_dev_t *dev, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
  uint8_t cs, uint8_t a0, uint8_t wr, uint8_t rd, uint8_t reset)
{

  /* fill data structure with some suitable values */
  u8g_init_data(u8g);
  u8g->dev = dev;

  /* assign user pins */

  u8g->pin_list[U8G_PI_D0] = d0;
  u8g->pin_list[U8G_PI_D1] = d1;
  u8g->pin_list[U8G_PI_D2] = d2;
  u8g->pin_list[U8G_PI_D3] = d3;
  u8g->pin_list[U8G_PI_D4] = d4;
  u8g->pin_list[U8G_PI_D5] = d5;
  u8g->pin_list[U8G_PI_D6] = d6;
  u8g->pin_list[U8G_PI_D7] = d7;

  u8g->pin_list[U8G_PI_CS] = cs;
  u8g->pin_list[U8G_PI_A0] = a0;
  u8g->pin_list[U8G_PI_WR] = wr;
  u8g->pin_list[U8G_PI_RD] = rd;
  u8g->pin_list[U8G_PI_RESET] = reset;

  return u8g_Begin(u8g);
}
#endif /* defined(U8G_WITH_PINLIST)  */

void u8g_FirstPage(u8g_t *u8g)
{
  u8g_FirstPageLL(u8g, u8g->dev);
}

uint8_t u8g_NextPage(u8g_t *u8g)
{
  if  ( u8g->cursor_fn != (u8g_draw_cursor_fn)0 )
  {
    u8g->cursor_fn(u8g);
  }
  return u8g_NextPageLL(u8g, u8g->dev);
}

uint8_t u8g_SetContrast(u8g_t *u8g, uint8_t contrast)
{
  return u8g_SetContrastLL(u8g, u8g->dev, contrast);
}

void u8g_SleepOn(u8g_t *u8g)
{
  u8g_call_dev_fn(u8g, u8g->dev, U8G_DEV_MSG_SLEEP_ON, NULL);
}

void u8g_SleepOff(u8g_t *u8g)
{
  u8g_call_dev_fn(u8g, u8g->dev, U8G_DEV_MSG_SLEEP_OFF, NULL);
}


void u8g_DrawPixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y)
{
  u8g_DrawPixelLL(u8g, u8g->dev, x, y);
}

void u8g_Draw8Pixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel)
{
  u8g_Draw8PixelLL(u8g, u8g->dev, x, y, dir, pixel);
}

void u8g_Draw4TPixel(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, uint8_t dir, uint8_t pixel)
{
  u8g_Draw4TPixelLL(u8g, u8g->dev, x, y, dir, pixel);
}


/* u8g_IsBBXIntersection() has been moved to u8g_clip.c */
#ifdef OBSOLETE_CODE
uint8_t u8g_IsBBXIntersection(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  /* new code */
  u8g_dev_arg_bbx_t arg;
  arg.x = x;
  arg.y = y;
  arg.w = w;
  arg.h = h;
  return u8g_is_box_bbx_intersection(&(u8g->current_page), &arg);

  /* old code */
  //return u8g_IsBBXIntersectionLL(u8g, u8g->dev, x, y, w, h);
}
#endif

/*
  idx: index for the palette entry (0..255)
  r: value for red (0..255)
  g: value for green (0..255)
  b: value for blue (0..255)
*/
void u8g_SetColorEntry(u8g_t *u8g, uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
  u8g_dev_arg_irgb_t irgb;
  irgb.idx = idx;
  irgb.r = r;
  irgb.g = g;
  irgb.b = b;
  u8g_call_dev_fn(u8g, u8g->dev, U8G_DEV_MSG_SET_COLOR_ENTRY, &irgb);
}

void u8g_SetColorIndex(u8g_t *u8g, uint8_t idx)
{
  u8g->arg_pixel.color = idx;
  /*u8g->color_index = idx; */ /* must be removed */
}

void u8g_SetHiColor(u8g_t *u8g, uint16_t rgb)
{
  u8g->arg_pixel.color = rgb&255;
  u8g->arg_pixel.hi_color = rgb>>8;
  /*u8g->color_index = idx; */ /* must be removed */
}

void u8g_SetHiColorByRGB(u8g_t *u8g, uint8_t r, uint8_t g, uint8_t b)
{

  r &= ~7;
  g >>= 2;
  b >>= 3;
  u8g->arg_pixel.color = b;
  u8g->arg_pixel.color |= (g & 7) << 5;
  u8g->arg_pixel.hi_color = r;
  u8g->arg_pixel.hi_color |= (g>>3) & 7;

  //u8g_SetHiColor(u8g, U8G_GET_HICOLOR_BY_RGB(r,g,b));
}

void u8g_SetRGB(u8g_t *u8g, uint8_t r, uint8_t g, uint8_t b)
{
  if ( u8g->mode == U8G_MODE_R3G3B2 )
  {
    r &= 0x0e0;
    g &= 0x0e0;
    g >>= 3;
    b >>= 6;
    u8g->arg_pixel.color = r | g | b;
  }
  else if ( u8g->mode == U8G_MODE_HICOLOR )
  {
    u8g_SetHiColorByRGB(u8g, r,g,b);
  }
  else
  {
    u8g->arg_pixel.color = r;
    u8g->arg_pixel.hi_color = g;
    u8g->arg_pixel.blue = b;
  }
}


uint8_t u8g_GetColorIndex(u8g_t *u8g)
{
  return u8g->arg_pixel.color;
}

uint8_t u8g_GetDefaultForegroundColor(u8g_t *u8g)
{
  uint8_t mode;
  mode = u8g_GetMode(u8g);
  if ( mode == U8G_MODE_R3G3B2 )
    return 255;     /* white */
  else if ( u8g_GetMode(u8g) == U8G_MODE_GRAY2BIT )
    return 3;         /* max intensity */
  else /* if ( u8g.getMode() == U8G_MODE_BW ) */
    return 1;         /* pixel on */
  return 1;
}

void u8g_SetDefaultForegroundColor(u8g_t *u8g)
{
  if ( u8g->mode == U8G_MODE_HICOLOR )
  {
    u8g->arg_pixel.color = 0x0ff;
    u8g->arg_pixel.hi_color = 0x0ff;
  }
  else
  {
    u8g_SetColorIndex(u8g, u8g_GetDefaultForegroundColor(u8g));
  }
}

uint8_t u8g_GetDefaultBackgroundColor(u8g_t *u8g)
{
  return 0;
}

void u8g_SetDefaultBackgroundColor(u8g_t *u8g)
{
  u8g_SetColorIndex(u8g, u8g_GetDefaultBackgroundColor(u8g));         /* pixel on / black */
}

uint8_t u8g_GetDefaultMidColor(u8g_t *u8g)
{
  uint8_t mode;
  mode = u8g_GetMode(u8g);
  if ( mode == U8G_MODE_R3G3B2 )
    return 0x06d;     /* gray: 01101101 */
  else if ( u8g_GetMode(u8g) == U8G_MODE_GRAY2BIT )
    return 1;         /* low mid intensity */
  else /* if ( u8g.getMode() == U8G_MODE_BW ) */
    return 1;         /* pixel on */
  return 1;   /* default */
}

void u8g_SetDefaultMidColor(u8g_t *u8g)
{
  u8g_SetColorIndex(u8g, u8g_GetDefaultMidColor(u8g));
}

// ============== u8g_com_api.c ==================
/*

  u8g_com_api.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


uint8_t u8g_InitCom(u8g_t *u8g, u8g_dev_t *dev, uint8_t clk_cycle_time)
{
  return dev->com_fn(u8g, U8G_COM_MSG_INIT, clk_cycle_time, NULL);
}

void u8g_StopCom(u8g_t *u8g, u8g_dev_t *dev)
{
  dev->com_fn(u8g, U8G_COM_MSG_STOP, 0, NULL);
}

/* cs contains the chip number, which should be enabled */
void u8g_SetChipSelect(u8g_t *u8g, u8g_dev_t *dev, uint8_t cs)
{
  dev->com_fn(u8g, U8G_COM_MSG_CHIP_SELECT, cs, NULL);
}

void u8g_SetResetLow(u8g_t *u8g, u8g_dev_t *dev)
{
  dev->com_fn(u8g, U8G_COM_MSG_RESET, 0, NULL);
}

void u8g_SetResetHigh(u8g_t *u8g, u8g_dev_t *dev)
{
  dev->com_fn(u8g, U8G_COM_MSG_RESET, 1, NULL);
}


void u8g_SetAddress(u8g_t *u8g, u8g_dev_t *dev, uint8_t address)
{
  dev->com_fn(u8g, U8G_COM_MSG_ADDRESS, address, NULL);
}

uint8_t u8g_WriteByte(u8g_t *u8g, u8g_dev_t *dev, uint8_t val)
{
  return dev->com_fn(u8g, U8G_COM_MSG_WRITE_BYTE, val, NULL);
}

uint8_t u8g_WriteSequence(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, uint8_t *seq)
{
  return dev->com_fn(u8g, U8G_COM_MSG_WRITE_SEQ, cnt, seq);
}

uint8_t u8g_WriteSequenceP(u8g_t *u8g, u8g_dev_t *dev, uint8_t cnt, const uint8_t *seq)
{
  return dev->com_fn(u8g, U8G_COM_MSG_WRITE_SEQ_P, cnt, (void *)seq);
}

/*
  sequence := { direct_value | escape_sequence }
  direct_value := 0..254
  escape_sequence := value_255 | sequence_end | delay | adr | cs | not_used
  value_255 := 255 255
  sequence_end = 255 254
  delay := 255 0..127
  adr := 255 0x0e0 .. 0x0ef
  cs := 255 0x0d0 .. 0x0df
  not_used := 255 101..254

#define U8G_ESC_DLY(x) 255, ((x) & 0x7f)
#define U8G_ESC_CS(x) 255, (0xd0 | ((x)&0x0f))
#define U8G_ESC_ADR(x) 255, (0xe0 | ((x)&0x0f))
#define U8G_ESC_VCC(x) 255, (0xbe | ((x)&0x01))
#define U8G_ESC_END 255, 254
#define U8G_ESC_255 255, 255
#define U8G_ESC_RST(x) 255, (0xc0 | ((x)&0x0f))

*/
uint8_t u8g_WriteEscSeqP(u8g_t *u8g, u8g_dev_t *dev, const uint8_t *esc_seq)
{
  uint8_t is_escape = 0;
  uint8_t value;
  for(;;)
  {
    value = u8g_pgm_read(esc_seq);
    if ( is_escape == 0 )
    {
      if ( value != 255 )
      {
        if ( u8g_WriteByte(u8g, dev, value) == 0 )
          return 0;
      }
      else
      {
        is_escape = 1;
      }
    }
    else
    {
      if ( value == 255 )
      {
        if ( u8g_WriteByte(u8g, dev, value) == 0 )
          return 0;
      }
      else if ( value == 254 )
      {
        break;
      }
      else if ( value >= 0x0f0 )
      {
        /* not yet used, do nothing */
      }
      else if ( value >= 0xe0  )
      {
        u8g_SetAddress(u8g, dev, value & 0x0f);
      }
      else if ( value >= 0xd0 )
      {
        u8g_SetChipSelect(u8g, dev, value & 0x0f);
      }
      else if ( value >= 0xc0 )
      {
        u8g_SetResetLow(u8g, dev);
        value &= 0x0f;
        value <<= 4;
        value+=2;
        u8g_Delay(value);
        u8g_SetResetHigh(u8g, dev);
        u8g_Delay(value);
      }
      else if ( value >= 0xbe )
      {
	/* not yet implemented */
        /* u8g_SetVCC(u8g, dev, value & 0x01); */
      }
      else if ( value <= 127 )
      {
        u8g_Delay(value);
      }
      is_escape = 0;
    }
    esc_seq++;
  }
  return 1;
}


// ============ u8g_delay.c ===========
/*

  u8g_delay.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


  void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
  void u8g_MicroDelay(void)		Delay be one microsecond
  void u8g_10MicroDelay(void)	Delay by 10 microseconds


*/



/*==== Part 1: Derive suitable delay procedure ====*/

#if defined(ARDUINO)

#  if defined(__AVR__)
#    define USE_AVR_DELAY
#  elif defined(__PIC32MX)
#    define USE_PIC32_DELAY
#  elif defined(__arm__)		/* Arduino Due */
#    define USE_ARDUINO_DELAY
#  else
#    define USE_ARDUINO_DELAY
#  endif
#elif defined(__AVR__)
#  define USE_AVR_DELAY
#elif defined(__18CXX)
#  define USE_PIC18_DELAY
#elif defined(__arm__)
/* do not define anything, all procedures are expected to be defined outside u8glib */

/*
void u8g_Delay(uint16_t val);
void u8g_MicroDelay(void);
void u8g_10MicroDelay(void);
*/

#else
#  define USE_DUMMY_DELAY
#endif



/*==== Part 2: Definition of the delay procedures ====*/

/*== AVR Delay ==*/

#if defined(USE_AVR_DELAY)
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/*
  Delay by the provided number of milliseconds.
  Thus, a 16 bit value will allow a delay of 0..65 seconds
  Makes use of the _delay_loop_2

  _delay_loop_2 will do a delay of n * 4 prozessor cycles.
  with f = F_CPU cycles per second,
  n = f / (1000 * 4 )
  with f = 16000000 the result is 4000
  with f = 1000000 the result is 250

  the millisec loop, gcc requires the following overhead:
  - movev 1
  - subwi 2x2
  - bne i 2
  ==> 7 cycles
  ==> must be devided by 4, rounded up 7/4 = 2
*/
void u8g_Delay(uint16_t val)
{
  /* old version did a call to the arduino lib: delay(val); */
  while( val != 0 )
  {
    _delay_loop_2( (F_CPU / 4000 ) -2);
    val--;
  }
}

/* delay by one micro second */
void u8g_MicroDelay(void)
{
#if (F_CPU / 4000000 ) > 0
  _delay_loop_2( (F_CPU / 4000000 ) );
#endif
}

/* delay by 10 micro seconds */
void u8g_10MicroDelay(void)
{
#if (F_CPU / 400000 ) > 0
  _delay_loop_2( (F_CPU / 400000 ) );
#endif
}

#endif


/*== Delay for PIC18 (not tested) ==*/

#if defined(USE_PIC18_DELAY)
#include <delays.h>
#define GetSystemClock()		(64000000ul)      // Hz
#define GetInstructionClock()	(GetSystemClock()/4)

void u8g_Delay(uint16_t val)
{/*
	unsigned int _iTemp = (val);
	while(_iTemp--)
		Delay1KTCYx((GetInstructionClock()+999999)/1000000);
		*/
}
void u8g_MicroDelay(void)
{
  /* not implemented */
}
void u8g_10MicroDelay(void)
{
  /* not implemented */
}
#endif


/*== Arduino Delay ==*/
#if defined(USE_ARDUINO_DELAY)
void u8g_Delay(uint16_t val)
{
#if defined(__arm__)
	delayMicroseconds((uint32_t)val*(uint32_t)1000);
#else
	delay(val);
#endif
}
void u8g_MicroDelay(void)
{
	delayMicroseconds(1);
}
void u8g_10MicroDelay(void)
{
	delayMicroseconds(10);
}
#endif

#if defined(USE_PIC32_DELAY)
/*
  Assume chipkit here with F_CPU correctly defined
  The problem was, that u8g_Delay() is called within the constructor.
  It seems that the chipkit is not fully setup at this time, so a
  call to delay() will not work. So here is my own implementation.

*/
#define CPU_COUNTS_PER_SECOND (F_CPU/2UL)
#define TICKS_PER_MILLISECOND  (CPU_COUNTS_PER_SECOND/1000UL)
#include "plib.h"
void u8g_Delay(uint16_t val)
{
	uint32_t d;
	uint32_t s;
	d = val;
	d *= TICKS_PER_MILLISECOND;
	s = ReadCoreTimer();
	while ( (uint32_t)(ReadCoreTimer() - s) < d )
		;
}

void u8g_MicroDelay(void)
{
	uint32_t d;
	uint32_t s;
	d = TICKS_PER_MILLISECOND/1000;
	s = ReadCoreTimer();
	while ( (uint32_t)(ReadCoreTimer() - s) < d )
		;
}

void u8g_10MicroDelay(void)
{
	uint32_t d;
	uint32_t s;
	d = TICKS_PER_MILLISECOND/100;
	s = ReadCoreTimer();
	while ( (uint32_t)(ReadCoreTimer() - s) < d )
		;
}

#endif

/*== Any other systems: Dummy Delay ==*/
#if defined(USE_DUMMY_DELAY)
void u8g_Delay(uint16_t val)
{
	/* do not know how to delay... */
}
void u8g_MicroDelay(void)
{
}
void u8g_10MicroDelay(void)
{
}
#endif


// =============== u8g_state.c ================

/*

  u8g_state.c

  backup and restore hardware state

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


  state callback: backup env U8G_STATE_MSG_BACKUP_ENV
  device callback: DEV_MSG_INIT
  state callback: backup u8g U8G_STATE_MSG_BACKUP_U8G
  state callback: restore env U8G_STATE_MSG_RESTORE_ENV

  state callback: backup env U8G_STATE_MSG_BACKUP_ENV
  state callback: retore u8g U8G_STATE_MSG_RESTORE_U8G
  DEV_MSG_PAGE_FIRST or DEV_MSG_PAGE_NEXT
  state callback: restore env U8G_STATE_MSG_RESTORE_ENV

*/


void u8g_state_dummy_cb(uint8_t msg)
{
  /* the dummy procedure does nothing */
}

void u8g_SetHardwareBackup(u8g_t *u8g, u8g_state_cb backup_cb)
{
  u8g->state_cb = backup_cb;
  /* in most cases the init message was already sent, so this will backup the */
  /* current u8g state */
  backup_cb(U8G_STATE_MSG_BACKUP_U8G);
}


/*===============================================================*/
/* register variable for restoring interrupt state */

#if defined(__AVR__)
uint8_t global_SREG_backup;
#endif



/*===============================================================*/
/* AVR */

#if defined(__AVR__)
#define U8G_ATMEGA_HW_SPI

/* remove the definition for attiny */
#if __AVR_ARCH__ == 2
#undef U8G_ATMEGA_HW_SPI
#endif
#if __AVR_ARCH__ == 25
#undef U8G_ATMEGA_HW_SPI
#endif
#endif

#if defined(U8G_ATMEGA_HW_SPI)
#include <avr/interrupt.h>
static uint8_t u8g_state_avr_spi_memory[2];

void u8g_backup_spi(uint8_t msg)
{
  if ( U8G_STATE_MSG_IS_BACKUP(msg) )
  {
    u8g_state_avr_spi_memory[U8G_STATE_MSG_GET_IDX(msg)] = SPCR;
  }
  else
  {
    uint8_t tmp = SREG;
    cli();
    SPCR = 0;
    SPCR = u8g_state_avr_spi_memory[U8G_STATE_MSG_GET_IDX(msg)];
    SREG = tmp;
  }
}

#elif defined(ARDUINO) && defined(__arm__)		// Arduino Due, maybe we should better check for __SAM3X8E__

#include "sam.h"

struct sam_backup_struct
{
  uint32_t mr;
  uint32_t sr;
  uint32_t csr[4];
} sam_backup[2];

void u8g_backup_spi(uint8_t msg)
{
  uint8_t idx = U8G_STATE_MSG_GET_IDX(msg);
  if ( U8G_STATE_MSG_IS_BACKUP(msg) )
  {
    sam_backup[idx].mr = SPI0->SPI_MR;
    sam_backup[idx].sr = SPI0->SPI_SR;
    sam_backup[idx].csr[0] = SPI0->SPI_CSR[0];
    sam_backup[idx].csr[1] = SPI0->SPI_CSR[1];
    sam_backup[idx].csr[2] = SPI0->SPI_CSR[2];
    sam_backup[idx].csr[3] = SPI0->SPI_CSR[3];
  }
  else
  {
    SPI0->SPI_MR = sam_backup[idx].mr;
    SPI0->SPI_CSR[0] = sam_backup[idx].csr[0];
    SPI0->SPI_CSR[1] = sam_backup[idx].csr[1];
    SPI0->SPI_CSR[2] = sam_backup[idx].csr[2];
    SPI0->SPI_CSR[3] = sam_backup[idx].csr[3];
  }
}

#else

void u8g_backup_spi(uint8_t msg)
{
}

#endif

// =========== u8g_pb8h1.c ==============

/*

  u8g_pb8h1.c

  8bit height monochrom (1 bit) page buffer
  byte has horizontal orientation

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


  total buffer size is limited to 256 bytes because of the calculation inside the set pixel procedure

  23. Sep 2012: Bug with down procedure, see FPS 1st page --> fixed (bug located in u8g_clip.c)

*/

#include <string.h>

#ifdef __unix__
#include <assert.h>
#endif

/* NEW_CODE disabled, because the performance increase was too slow and not worth compared */
/* to the increase of code size */
/* #define NEW_CODE */

#ifdef __unix__
void *u8g_buf_lower_limit;
void *u8g_buf_upper_limit;
#endif

void u8g_pb8h1_Init(u8g_pb_t *b, void *buf, u8g_uint_t width) U8G_NOINLINE;
void u8g_pb8h1_set_pixel(u8g_pb_t *b, u8g_uint_t x, u8g_uint_t y, uint8_t color_index) U8G_NOINLINE;
void u8g_pb8h1_SetPixel(u8g_pb_t *b, const u8g_dev_arg_pixel_t * const arg_pixel) U8G_NOINLINE ;
void u8g_pb8h1_Set8PixelStd(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel) U8G_NOINLINE;
uint8_t u8g_dev_pb8h1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg);


#ifdef NEW_CODE
struct u8g_pb_h1_struct
{
  u8g_uint_t x;
  u8g_uint_t y;
  uint8_t *ptr;
  uint8_t mask;
  uint8_t line_byte_len;
  uint8_t cnt;
};

static uint8_t u8g_pb8h1_bitmask[8] = { 0x080, 0x040, 0x020, 0x010, 0x008, 0x004, 0x002, 0x001 };

static void u8g_pb8h1_state_right(struct u8g_pb_h1_struct *s)  U8G_NOINLINE;
static void u8g_pb8h1_state_right(struct u8g_pb_h1_struct *s)
{
  register u8g_uint_t x;
  x = s->x;
  x++;
  s->x = x;
  x &= 7;
  s->mask = u8g_pb8h1_bitmask[x];
  if ( x == 0 )
    s->ptr++;
}

static void u8g_pb8h1_state_left(struct u8g_pb_h1_struct *s)
{
  register u8g_uint_t x;
  x = s->x;
  x--;
  s->x = x;
  x &= 7;
  s->mask = u8g_pb8h1_bitmask[x];
  if ( x == 7 )
    s->ptr--;
}

static void u8g_pb8h1_state_down(struct u8g_pb_h1_struct *s)
{
  s->y++;
  s->ptr += s->line_byte_len;
}

static void u8g_pb8h1_state_up(struct u8g_pb_h1_struct *s)
{
  s->y--;
  s->ptr -= s->line_byte_len;
}

static void u8g_pb8h1_state_init(struct u8g_pb_h1_struct *s, u8g_pb_t *b, u8g_uint_t x, u8g_uint_t y) U8G_NOINLINE;
static void u8g_pb8h1_state_init(struct u8g_pb_h1_struct *s, u8g_pb_t *b, u8g_uint_t x, u8g_uint_t y)
{
  u8g_uint_t tmp;

  uint8_t *ptr = b->buf;

  s->x = x;
  s->y = y;

  y -= b->p.page_y0;

  tmp = b->width;
  tmp >>= 3;
  s->line_byte_len = tmp;

  /* assume negative y values, can be down to -7, subtract this from the pointer and add correction of 8 to y */
  ptr -= tmp*8;
  y+=8;
  /* it is important that the result of tmp*y can be 16 bit value also for 8 bit mode */
  ptr += tmp*y;

  s->mask = u8g_pb8h1_bitmask[x & 7];

  /* assume negative x values (to -7), subtract 8 pixel from the pointer and add 8 to x */
  ptr--;
  x += 8;
  x >>= 3;
  ptr += x;
  s->ptr = ptr;
}

static void u8g_pb8h1_state_set_pixel(struct u8g_pb_h1_struct *s, uint8_t color_index) U8G_NOINLINE;
static void u8g_pb8h1_state_set_pixel(struct u8g_pb_h1_struct *s, uint8_t color_index)
{

#ifdef __unix__
  assert( s->ptr >= u8g_buf_lower_limit );
  assert( s->ptr < u8g_buf_upper_limit );
#endif

  if ( color_index )
  {
    *s->ptr |= s->mask;
  }
  else
  {
    uint8_t mask = s->mask;
    mask ^=0xff;
    *s->ptr &= mask;
  }
}
#endif


void u8g_pb8h1_Init(u8g_pb_t *b, void *buf, u8g_uint_t width)
{
  b->buf = buf;
  b->width = width;
  u8g_pb_Clear(b);
}

/* limitation: total buffer must not exceed 256 bytes */
void u8g_pb8h1_set_pixel(u8g_pb_t *b, u8g_uint_t x, u8g_uint_t y, uint8_t color_index)
{
#ifdef NEW_CODE
  struct u8g_pb_h1_struct s;
  u8g_pb8h1_state_init(&s, b, x, y);
  u8g_pb8h1_state_set_pixel(&s, color_index);

//  u8g_pb8h1_state_up(&s);
//  if ( s.y > b->p.page_y1 )
//    return;
//  if ( s.x > b->width )
//    return;
//  u8g_pb8h1_state_set_pixel(&s, color_index);
#else
  register uint8_t mask;
  u8g_uint_t tmp;
  uint8_t *ptr = (uint8_t*)b->buf;

  y -= b->p.page_y0;
  tmp = b->width;
  tmp >>= 3;
  tmp *= (uint8_t)y;
  ptr += tmp;

  mask = 0x080;
  mask >>= x & 7;
  x >>= 3;
  ptr += x;
  if ( color_index )
  {
    *ptr |= mask;
  }
  else
  {
    mask ^=0xff;
    *ptr &= mask;
  }
#endif
}


void u8g_pb8h1_SetPixel(u8g_pb_t *b, const u8g_dev_arg_pixel_t * const arg_pixel)
{
  if ( arg_pixel->y < b->p.page_y0 )
    return;
  if ( arg_pixel->y > b->p.page_y1 )
    return;
  if ( arg_pixel->x >= b->width )
    return;
  u8g_pb8h1_set_pixel(b, arg_pixel->x, arg_pixel->y, arg_pixel->color);
}

void u8g_pb8h1_Set8PixelStd(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel)
{
  register uint8_t pixel = arg_pixel->pixel;
  do
  {
    if ( pixel & 128 )
    {
      u8g_pb8h1_SetPixel(b, arg_pixel);
    }
    switch( arg_pixel->dir )
    {
      case 0: arg_pixel->x++; break;
      case 1: arg_pixel->y++; break;
      case 2: arg_pixel->x--; break;
      case 3: arg_pixel->y--; break;
    }
    pixel <<= 1;
  } while( pixel != 0  );
}

void u8g_pb8h1_Set8PixelOpt2(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel)
{
  register uint8_t pixel = arg_pixel->pixel;
  u8g_uint_t dx = 0;
  u8g_uint_t dy = 0;

  switch( arg_pixel->dir )
  {
    case 0: dx++; break;
    case 1: dy++; break;
    case 2: dx--; break;
    case 3: dy--; break;
  }

  do
  {
    if ( pixel & 128 )
      u8g_pb8h1_SetPixel(b, arg_pixel);
    arg_pixel->x += dx;
    arg_pixel->y += dy;
    pixel <<= 1;
  } while( pixel != 0  );
}

#ifdef NEW_CODE
static void u8g_pb8h1_Set8PixelState(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel)
{
  register uint8_t pixel = arg_pixel->pixel;
  struct u8g_pb_h1_struct s;
  uint8_t cnt;
  u8g_pb8h1_state_init(&s, b, arg_pixel->x, arg_pixel->y);
  cnt = 8;
  switch( arg_pixel->dir )
  {
    case 0:
      do
      {
	if ( s.x < b->width )
	  if ( pixel & 128 )
	    u8g_pb8h1_state_set_pixel(&s, arg_pixel->color);
	u8g_pb8h1_state_right(&s);
	pixel <<= 1;
	cnt--;
      } while( cnt > 0 && pixel != 0  );
      break;
    case 1:
      do
      {
	if ( s.y >= b->p.page_y0 )
	  if ( s.y <= b->p.page_y1 )
	    if ( pixel & 128 )
	      u8g_pb8h1_state_set_pixel(&s, arg_pixel->color);
	u8g_pb8h1_state_down(&s);
	pixel <<= 1;
	cnt--;
      } while( cnt > 0 && pixel != 0  );
      break;
    case 2:
      do
      {
	if ( s.x < b->width )
	  if ( pixel & 128 )
	    u8g_pb8h1_state_set_pixel(&s, arg_pixel->color);
	u8g_pb8h1_state_left(&s);
	pixel <<= 1;
	cnt--;
      } while( cnt > 0 && pixel != 0 );
      break;
    case 3:
      do
      {
	if ( s.y >= b->p.page_y0 )
	  if ( s.y <= b->p.page_y1 )
	    if ( pixel & 128 )
	      u8g_pb8h1_state_set_pixel(&s, arg_pixel->color);
	u8g_pb8h1_state_up(&s);
	pixel <<= 1;
	cnt--;
      } while( cnt > 0 && pixel != 0  );
      break;
  }
}
#endif

uint8_t u8g_dev_pb8h1_base_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
  switch(msg)
  {
    case U8G_DEV_MSG_SET_8PIXEL:
#ifdef NEW_CODE
      if ( u8g_pb_Is8PixelVisible(pb, (u8g_dev_arg_pixel_t *)arg) )
        u8g_pb8h1_Set8PixelState(pb, (u8g_dev_arg_pixel_t *)arg);
#else
      if ( u8g_pb_Is8PixelVisible(pb, (u8g_dev_arg_pixel_t *)arg) )
        u8g_pb8h1_Set8PixelOpt2(pb, (u8g_dev_arg_pixel_t *)arg);
#endif
      break;
    case U8G_DEV_MSG_SET_PIXEL:
      u8g_pb8h1_SetPixel(pb, (u8g_dev_arg_pixel_t *)arg);
      break;
    case U8G_DEV_MSG_INIT:
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_FIRST:
      u8g_pb_Clear(pb);
      u8g_page_First(&(pb->p));
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      if ( u8g_page_Next(&(pb->p)) == 0 )
        return 0;
      u8g_pb_Clear(pb);
      break;
#ifdef U8G_DEV_MSG_IS_BBX_INTERSECTION
    case U8G_DEV_MSG_IS_BBX_INTERSECTION:
      return u8g_pb_IsIntersection(pb, (u8g_dev_arg_bbx_t *)arg);
#endif
    case U8G_DEV_MSG_GET_PAGE_BOX:
      u8g_pb_GetPageBox(pb, (u8g_box_t *)arg);
      break;
    case U8G_DEV_MSG_GET_WIDTH:
      *((u8g_uint_t *)arg) = pb->width;
      break;
    case U8G_DEV_MSG_GET_HEIGHT:
      *((u8g_uint_t *)arg) = pb->p.total_height;
      break;
    case U8G_DEV_MSG_SET_COLOR_ENTRY:
      break;
    case U8G_DEV_MSG_SET_XY_CB:
      break;
    case U8G_DEV_MSG_GET_MODE:
      return U8G_MODE_BW;
  }
  return 1;
}

// =========== u8g_com_arduino_st7920_spi.c ============
/*

  u8g_com_arduino_st7920_spi.c

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  A special SPI interface for ST7920 controller

  Update for ATOMIC operation done (01 Jun 2013)
    U8G_ATOMIC_OR(ptr, val)
    U8G_ATOMIC_AND(ptr, val)
    U8G_ATOMIC_START();
    U8G_ATOMIC_END();


*/


#if defined(ARDUINO)

#if ARDUINO < 100
#include <WProgram.h>
#include "wiring_private.h"
#include "pins_arduino.h"

#else
#include <Arduino.h>
#include "wiring_private.h"
#endif

#if defined(__AVR__)

uint8_t u8g_bitData, u8g_bitNotData;
uint8_t u8g_bitClock, u8g_bitNotClock;
volatile uint8_t *u8g_outData;
volatile uint8_t *u8g_outClock;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  u8g_outData = portOutputRegister(digitalPinToPort(dataPin));
  u8g_outClock = portOutputRegister(digitalPinToPort(clockPin));
  u8g_bitData = digitalPinToBitMask(dataPin);
  u8g_bitClock = digitalPinToBitMask(clockPin);

  u8g_bitNotClock = u8g_bitClock;
  u8g_bitNotClock ^= 0x0ff;

  u8g_bitNotData = u8g_bitData;
  u8g_bitNotData ^= 0x0ff;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val) U8G_NOINLINE;
static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t cnt = 8;
  uint8_t bitData = u8g_bitData;
  uint8_t bitNotData = u8g_bitNotData;
  uint8_t bitClock = u8g_bitClock;
  uint8_t bitNotClock = u8g_bitNotClock;
  volatile uint8_t *outData = u8g_outData;
  volatile uint8_t *outClock = u8g_outClock;


  U8G_ATOMIC_START();
  bitData |= *outData;
  bitNotData &= *outData;
  do
  {
    if ( val & 128 )
      *outData = bitData;
    else
      *outData = bitNotData;

    /*
    *outClock |= bitClock;
    val <<= 1;
    cnt--;
    *outClock &= bitNotClock;
    */

    val <<= 1;
    *outClock &= bitNotClock;
    cnt--;
    // removed micro delays, because AVRs are too slow and the delay is not required
    //u8g_MicroDelay();
    *outClock |= bitClock;
    //u8g_MicroDelay();
  } while( cnt != 0 );
  U8G_ATOMIC_END();
}

#elif defined(__18CXX) || defined(__PIC32MX)

uint16_t dog_bitData, dog_bitNotData;
uint16_t dog_bitClock, dog_bitNotClock;
volatile uint32_t *dog_outData;
volatile uint32_t *dog_outClock;
volatile uint32_t dog_pic32_spi_tmp;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  dog_outData = portOutputRegister(digitalPinToPort(dataPin));
  dog_outClock = portOutputRegister(digitalPinToPort(clockPin));
  dog_bitData = digitalPinToBitMask(dataPin);
  dog_bitClock = digitalPinToBitMask(clockPin);

  dog_bitNotClock = dog_bitClock;
  dog_bitNotClock ^= 0x0ffff;

  dog_bitNotData = dog_bitData;
  dog_bitNotData ^= 0x0ffff;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t cnt = 8;
  U8G_ATOMIC_START();
  do
  {
    if ( val & 128 )
	*dog_outData |= dog_bitData;
    else
	*dog_outData &= dog_bitNotData;
    val <<= 1;
    //u8g_MicroDelay();
    //*dog_outClock |= dog_bitClock;
    *dog_outClock &= dog_bitNotClock;
    cnt--;
    u8g_MicroDelay();
    //*dog_outClock &= dog_bitNotClock;
    *dog_outClock |= dog_bitClock;
    u8g_MicroDelay();

  } while( cnt != 0 );
  U8G_ATOMIC_END();
}

#else

/* default interface, Arduino DUE (__arm__) */

uint8_t u8g_data_pin;
uint8_t u8g_clock_pin;

static void u8g_com_arduino_init_shift_out(uint8_t dataPin, uint8_t clockPin)
{
  u8g_data_pin = dataPin;
  u8g_clock_pin = clockPin;
}

static void u8g_com_arduino_do_shift_out_msb_first(uint8_t val)
{
  uint8_t cnt = 8;
  do
  {
#ifdef UI_SPI_MOSI
    WRITE(UI_SPI_MOSI,val & 128);
#else
    if ( val & 128 )
	digitalWrite(u8g_data_pin, HIGH);
    else
	digitalWrite(u8g_data_pin, LOW);
#endif
    val <<= 1;
    //u8g_MicroDelay();
#ifdef UI_SPI_SCK
    WRITE(UI_SPI_SCK,LOW);
#else
    digitalWrite(u8g_clock_pin, LOW);
#endif
    cnt--;
    u8g_MicroDelay();
#ifdef UI_SPI_SCK
    WRITE(UI_SPI_SCK,HIGH);
#else
    digitalWrite(u8g_clock_pin, HIGH);
#endif
    u8g_MicroDelay();
  } while( cnt != 0 );
}

#endif


static void u8g_com_arduino_st7920_write_byte_seq(uint8_t rs, uint8_t *ptr, uint8_t len)
{
  uint8_t i;

  if ( rs == 0 )
  {
    /* command */
    u8g_com_arduino_do_shift_out_msb_first(0x0f8);
  }
  else if ( rs == 1 )
  {
    /* data */
    u8g_com_arduino_do_shift_out_msb_first(0x0fa);
  }

  while( len > 0 )
  {
    u8g_com_arduino_do_shift_out_msb_first(*ptr & 0x0f0);
    u8g_com_arduino_do_shift_out_msb_first(*ptr << 4);
    ptr++;
    len--;
    u8g_10MicroDelay();
  }

  for( i = 0; i < 4; i++ )
    u8g_10MicroDelay();
}

static void u8g_com_arduino_st7920_write_byte(uint8_t rs, uint8_t val)
{
  uint8_t i;

  if ( rs == 0 )
  {
    /* command */
    u8g_com_arduino_do_shift_out_msb_first(0x0f8);
  }
  else if ( rs == 1 )
  {
    /* data */
    u8g_com_arduino_do_shift_out_msb_first(0x0fa);
  }

  u8g_com_arduino_do_shift_out_msb_first(val & 0x0f0);
  u8g_com_arduino_do_shift_out_msb_first(val << 4);

  for( i = 0; i < 4; i++ )
    u8g_10MicroDelay();

}


uint8_t u8g_com_arduino_st7920_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      u8g_com_arduino_assign_pin_output_high(u8g);
#ifdef UI_SPI_CS
      WRITE(UI_SPI_CS,LOW);
#else
      u8g_com_arduino_digital_write(u8g, U8G_PI_CS, LOW);
#endif
      // u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, LOW);
#ifdef UI_SPI_SCK
        WRITE(UI_SPI_SCK,HIGH);
#else
      u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, HIGH);
#endif
#ifdef UI_SPI_MOSI
      WRITE(UI_SPI_MOSI,LOW);
#else
      u8g_com_arduino_digital_write(u8g, U8G_PI_MOSI, LOW);
#endif
      u8g_com_arduino_init_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK]);
      u8g->pin_list[U8G_PI_A0_STATE] = 0;       /* inital RS state: command mode */
      break;

    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
	u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_CHIP_SELECT:
#ifdef UI_SPI_CS
      WRITE(UI_SPI_CS,arg_val);
#else
      if ( arg_val == 0 )
      {
        /* disable, note: the st7920 has an active high chip select */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, LOW);
      }
      else
      {
        /* enable */
        //u8g_com_arduino_digital_write(u8g, U8G_PI_SCK, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS, HIGH);
      }
#endif
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_arduino_st7920_write_byte( u8g->pin_list[U8G_PI_A0_STATE], arg_val);
      //u8g->pin_list[U8G_PI_A0_STATE] = 2;
      //u8g_arduino_sw_spi_shift_out(u8g->pin_list[U8G_PI_MOSI], u8g->pin_list[U8G_PI_SCK], arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ:
      u8g_com_arduino_st7920_write_byte_seq(u8g->pin_list[U8G_PI_A0_STATE], (uint8_t *)arg_ptr, arg_val);
      break;

      case U8G_COM_MSG_WRITE_SEQ_P:
      {
        register uint8_t *ptr = (uint8_t*)arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_arduino_st7920_write_byte(u8g->pin_list[U8G_PI_A0_STATE], u8g_pgm_read(ptr) );
          //u8g->pin_list[U8G_PI_A0_STATE] = 2;
          ptr++;
          arg_val--;
        }
      }
      break;

    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      break;
  }
  return 1;
}

#else /* ARDUINO */

uint8_t u8g_com_arduino_st7920_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  return 1;
}

#endif /* ARDUINO */

// ============= u8g_com_arduino_common.c =============
/*

  u8g_com_arduino_common.c

  shared procedures for the arduino communication procedures

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


#if defined(ARDUINO)


void u8g_com_arduino_digital_write(u8g_t *u8g, uint8_t pin_index, uint8_t value)
{
  uint8_t pin;
  pin = u8g->pin_list[pin_index];
  if ( pin != U8G_PIN_NONE )
    digitalWrite(pin, value);
}

/* this procedure does not set the RW pin */
void u8g_com_arduino_assign_pin_output_high(u8g_t *u8g)
{
  uint8_t i;
  /* skip the RW pin, which is the last pin in the list */
  for( i = 0; i < U8G_PIN_LIST_LEN-1; i++ )
  {
    if ( u8g->pin_list[i] != U8G_PIN_NONE )
    {
      pinMode(u8g->pin_list[i], OUTPUT);
      digitalWrite(u8g->pin_list[i], HIGH);
    }
  }
}


#endif

/*

  u8g_pb.c

  common procedures for the page buffer

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


void u8g_pb_Clear(u8g_pb_t *b)
{
  uint8_t *ptr = (uint8_t *)b->buf;
  uint8_t *end_ptr = ptr;
  end_ptr += b->width;
  do
  {
    *ptr++ = 0;
  } while( ptr != end_ptr );
}

/* the following procedure does not work. why? Can be checked with descpic */
/*
void u8g_pb_Clear(u8g_pb_t *b)
{
  uint8_t *ptr = (uint8_t *)b->buf;
  uint8_t cnt = b->width;
  do
  {
    *ptr++ = 0;
    cnt--;
  } while( cnt != 0 );
}
*/

/*
  intersection assumptions:
    a1 <= a2 is always true
*/
  /*
    minimized version
    ---1----0 1             b1 <= a2 && b1 > b2
    -----1--0 1             b2 >= a1 && b1 > b2
    ---1-1--- 1             b1 <= a2 && b2 >= a1
  */
/*
uint8_t u8g_pb8v1_IsYIntersection___Old(u8g_pb_t *b, u8g_uint_t v0, u8g_uint_t v1)
{
  uint8_t c0, c1, c;
  c0 = v0 <= b->p.page_y1;
  c1 = v1 >= b->p.page_y0;
  c = v0 > v1;
  if ( c0 && c1 ) return 1;
  if ( c0 && c ) return 1;
  if ( c1 && c ) return 1;
  return 0;
}
*/

uint8_t u8g_pb_IsYIntersection(u8g_pb_t *pb, u8g_uint_t v0, u8g_uint_t v1)
{
  uint8_t c1, c2, c3, tmp;
  c1 = v0 <= pb->p.page_y1;
  c2 = v1 >= pb->p.page_y0;
  c3 = v0 > v1;
  /*
  if ( c1 && c2 )
    return 1;
  if ( c1 && c3 )
    return 1;
  if ( c2 && c3 )
    return 1;
  return 0;
  */

  tmp = c1;
  c1 &= c2;
  c2 &= c3;
  c3 &= tmp;
  c1 |= c2;
  c1 |= c3;
  return c1 & 1;
}


uint8_t u8g_pb_IsXIntersection(u8g_pb_t *b, u8g_uint_t v0, u8g_uint_t v1)
{
  uint8_t /*c0, c1, */ c2, c3;
  /*
    conditions: b->p.page_y0 < b->p.page_y1
    there are no restriction on v0 and v1. If v0 > v1, then warp around unsigned is assumed
  */
  /*
  c0 = v0 < 0;
  c1 = v1 < 0;
  */
  c2 = v0 > b->width;
  c3 = v1 > b->width;
  /*if ( c0 && c1 ) return 0;*/
  if ( c2 && c3 ) return 0;
  /*if ( c1 && c2 ) return 0;*/
  return 1;
}

uint8_t u8g_pb_IsIntersection(u8g_pb_t *pb, u8g_dev_arg_bbx_t *bbx)
{
  u8g_uint_t tmp;

  tmp = bbx->y;
  tmp += bbx->h;
  tmp--;

  if ( u8g_pb_IsYIntersection(pb, bbx->y, tmp) == 0 )
    return 0;

  /* maybe this one can be skiped... probability is very high to have an intersection, so it would be ok to always return 1 */
  tmp = bbx->x;
  tmp += bbx->w;
  tmp--;

  return u8g_pb_IsXIntersection(pb, bbx->x, tmp);
}

void u8g_pb_GetPageBox(u8g_pb_t *pb, u8g_box_t *box)
{
  box->x0 = 0;
  box->y0 = pb->p.page_y0;
  box->x1 = pb->width;
  box->x1--;
  box->y1 = pb->p.page_y1;
}


uint8_t u8g_pb_Is8PixelVisible(u8g_pb_t *b, u8g_dev_arg_pixel_t *arg_pixel)
{
  u8g_uint_t v0, v1;
  v0 = arg_pixel->y;
  v1 = v0;
  switch( arg_pixel->dir )
  {
    case 0:
      break;
    case 1:
      v1 += 8;          /* this is independent from the page height */
      break;
    case 2:
      break;
    case 3:
      v0 -= 8;
      break;
  }
  return u8g_pb_IsYIntersection(b, v0, v1);
}



uint8_t u8g_pb_WriteBuffer(u8g_pb_t *b, u8g_t *u8g, u8g_dev_t *dev)
{
  return u8g_WriteSequence(u8g, dev, b->width,(uint8_t*) b->buf);
}

/*

  u8g_rect.c

  U8G high level interface for horizontal and vertical things

  Universal 8bit Graphics Library

  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/

void u8g_draw_hline(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w)
{
  uint8_t pixel = 0x0ff;
  while( w >= 8 )
  {
    u8g_Draw8Pixel(u8g, x, y, 0, pixel);
    w-=8;
    x+=8;
  }
  if ( w != 0 )
  {
    w ^=7;
    w++;
    pixel <<= w&7;
    u8g_Draw8Pixel(u8g, x, y, 0, pixel);
  }
}

void u8g_draw_vline(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t h)
{
  uint8_t pixel = 0x0ff;
  while( h >= 8 )
  {
    u8g_Draw8Pixel(u8g, x, y, 1, pixel);
    h-=8;
    y+=8;
  }
  if ( h != 0 )
  {
    h ^=7;
    h++;
    pixel <<= h&7;
    u8g_Draw8Pixel(u8g, x, y, 1, pixel);
  }
}

void u8g_DrawHLine(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w)
{
  if ( u8g_IsBBXIntersection(u8g, x, y, w, 1) == 0 )
    return;
  u8g_draw_hline(u8g, x, y, w);
}

void u8g_DrawVLine(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w)
{
  if ( u8g_IsBBXIntersection(u8g, x, y, 1, w) == 0 )
    return;
  u8g_draw_vline(u8g, x, y, w);
}

/* restrictions: w > 0 && h > 0 */
void u8g_DrawFrame(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  u8g_uint_t xtmp = x;

  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;


  u8g_draw_hline(u8g, x, y, w);
  u8g_draw_vline(u8g, x, y, h);
  x+=w;
  x--;
  u8g_draw_vline(u8g, x, y, h);
  y+=h;
  y--;
  u8g_draw_hline(u8g, xtmp, y, w);
}

void u8g_draw_box(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  do
  {
    u8g_draw_hline(u8g, x, y, w);
    y++;
    h--;
  } while( h != 0 );
}

/* restrictions: h > 0 */
void u8g_DrawBox(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h)
{
  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;
  u8g_draw_box(u8g, x, y, w, h);
}


void u8g_DrawRFrame(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r)
{
  u8g_uint_t xl, yu;

  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;

  xl = x;
  xl += r;
  yu = y;
  yu += r;

  {
    u8g_uint_t yl, xr;

    xr = x;
    xr += w;
    xr -= r;
    xr -= 1;

    yl = y;
    yl += h;
    yl -= r;
    yl -= 1;

    u8g_draw_circle(u8g, xl, yu, r, U8G_DRAW_UPPER_LEFT);
    u8g_draw_circle(u8g, xr, yu, r, U8G_DRAW_UPPER_RIGHT);
    u8g_draw_circle(u8g, xl, yl, r, U8G_DRAW_LOWER_LEFT);
    u8g_draw_circle(u8g, xr, yl, r, U8G_DRAW_LOWER_RIGHT);
  }

  {
    u8g_uint_t ww, hh;

    ww = w;
    ww -= r;
    ww -= r;
    ww -= 2;
    hh = h;
    hh -= r;
    hh -= r;
    hh -= 2;

    xl++;
    yu++;
    h--;
    w--;
    u8g_draw_hline(u8g, xl, y, ww);
    u8g_draw_hline(u8g, xl, y+h, ww);
    u8g_draw_vline(u8g, x,         yu, hh);
    u8g_draw_vline(u8g, x+w, yu, hh);
  }
}

void u8g_DrawRBox(u8g_t *u8g, u8g_uint_t x, u8g_uint_t y, u8g_uint_t w, u8g_uint_t h, u8g_uint_t r)
{
  u8g_uint_t xl, yu;
    u8g_uint_t yl, xr;

  if ( u8g_IsBBXIntersection(u8g, x, y, w, h) == 0 )
    return;

  xl = x;
  xl += r;
  yu = y;
  yu += r;

  xr = x;
  xr += w;
  xr -= r;
  xr -= 1;

  yl = y;
  yl += h;
  yl -= r;
  yl -= 1;

  u8g_draw_disc(u8g, xl, yu, r, U8G_DRAW_UPPER_LEFT);
  u8g_draw_disc(u8g, xr, yu, r, U8G_DRAW_UPPER_RIGHT);
  u8g_draw_disc(u8g, xl, yl, r, U8G_DRAW_LOWER_LEFT);
  u8g_draw_disc(u8g, xr, yl, r, U8G_DRAW_LOWER_RIGHT);

  {
    u8g_uint_t ww, hh;

    ww = w;
    ww -= r;
    ww -= r;
    ww -= 2;
    hh = h;
    hh -= r;
    hh -= r;
    hh -= 2;

    xl++;
    yu++;
    h--;
    u8g_draw_box(u8g, xl, y, ww, r+1);
    u8g_draw_box(u8g, xl, yl, ww, r+1);
    //u8g_draw_hline(u8g, xl, y+h, ww);
    u8g_draw_box(u8g, x, yu, w, hh);
    //u8g_draw_vline(u8g, x+w, yu, hh);
  }
}
