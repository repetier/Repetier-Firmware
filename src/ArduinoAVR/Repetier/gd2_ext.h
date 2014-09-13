/*
 * Copyright (C) 2013 by James Bowman <jamesb@excamera.com>
 * Gameduino 2 library for Arduino, Raspberry Pi.

Copyright (c) 2013, James Bowman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the Excamera Labs nor the names of its contributors may be used to endorse
   or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
/*
This version was modified to get it working within the repetier firmware eco system.
Modifications in this file stay licenced under the BSD licence from the original library!
*/
#ifndef _GD2_H_INCLUDED
#define _GD2_H_INCLUDED

#if defined(RASPBERRY_PI)
#include "wiring.h"
#endif

#if defined(RASPBERRY_PI)
#define REPORT(VAR) fprintf(stderr, #VAR "=%d\n", (VAR))
#else
#define REPORT(VAR) (Serial.print(#VAR "="), Serial.println(VAR, DEC))
#endif

#include "Arduino.h"
#include <stdarg.h>

#define RGB(r, g, b)    ((uint32_t)((((r) & 0xffL) << 16) | (((g) & 0xffL) << 8) | ((b) & 0xffL)))
#define F8(x)           (int((x) * 256L))
#define F16(x)          ((int32_t)((x) * 65536L))

#define GD_CALIBRATE    1
#define GD_TRIM         2
#define GD_STORAGE      4

////////////////////////////////////////////////////////////////////////

class GDClass {
public:
  void begin(uint8_t options = (GD_CALIBRATE | GD_TRIM | GD_STORAGE));

  uint16_t random();
  uint16_t random(uint16_t n);
  void seed(uint16_t n);
  int16_t rsin(int16_t r, uint16_t th);
  int16_t rcos(int16_t r, uint16_t th);
  void polar(int &x, int &y, int16_t r, uint16_t th);
  uint16_t atan2(int16_t y, int16_t x);

  void copy(const PROGMEM prog_uchar *src, int count);
  void copyram(byte *src, int count);

  void self_calibrate(void);

  void swap(void);
  void flush(void);
  void finish(void);

  void play(uint8_t instrument, uint8_t note = 0);
  void sample(uint32_t start, uint32_t len, uint16_t freq, uint16_t format, int loop = 0);

  void get_inputs(void);
  void get_accel(int &x, int &y, int &z);
  struct {
    uint16_t track_tag;
    uint16_t track_val;
    uint16_t rz;
    uint16_t __dummy_1;
    int16_t y;
    int16_t x;
    int16_t tag_y;
    int16_t tag_x;
    uint8_t tag;
    uint8_t ptag;
  } inputs;

  void AlphaFunc(byte func, byte ref);
  void Begin(byte prim);
  void BitmapHandle(byte handle);
  void BitmapLayout(byte format, uint16_t linestride, uint16_t height);
  void BitmapSize(byte filter, byte wrapx, byte wrapy, uint16_t width, uint16_t height);
  void BitmapSource(uint32_t addr);
  void BitmapTransformA(int32_t a);
  void BitmapTransformB(int32_t b);
  void BitmapTransformC(int32_t c);
  void BitmapTransformD(int32_t d);
  void BitmapTransformE(int32_t e);
  void BitmapTransformF(int32_t f);
  void BlendFunc(byte src, byte dst);
  void Call(uint16_t dest);
  void Cell(byte cell);
  void ClearColorA(byte alpha);
  void ClearColorRGB(byte red, byte green, byte blue);
  void ClearColorRGB(uint32_t rgb);
  void Clear(byte c, byte s, byte t);
  void Clear(void);
  void ClearStencil(byte s);
  void ClearTag(byte s);
  void ColorA(byte alpha);
  void ColorMask(byte r, byte g, byte b, byte a);
  void ColorRGB(byte red, byte green, byte blue);
  void ColorRGB(uint32_t rgb);
  void Display(void);
  void End(void);
  void Jump(uint16_t dest);
  void LineWidth(uint16_t width);
  void Macro(byte m);
  void PointSize(uint16_t size);
  void RestoreContext(void);
  void Return(void);
  void SaveContext(void);
  void ScissorSize(uint16_t width, uint16_t height);
  void ScissorXY(uint16_t x, uint16_t y);
  void StencilFunc(byte func, byte ref, byte mask);
  void StencilMask(byte mask);
  void StencilOp(byte sfail, byte spass);
  void TagMask(byte mask);
  void Tag(byte s);
  void Vertex2f(int16_t x, int16_t y);
  void Vertex2ii(uint16_t x, uint16_t y, byte handle = 0, byte cell = 0);

  // Higher-level graphics commands

  void cmd_append(uint32_t ptr, uint32_t num);
  void cmd_bgcolor(uint32_t c);
  void cmd_button(int16_t x, int16_t y, uint16_t w, uint16_t h, byte font, uint16_t options, const char *s);
  void cmd_calibrate(void);
  void cmd_clock(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms);
  void cmd_coldstart(void);
  void cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val);
  void cmd_dlstart(void);
  void cmd_fgcolor(uint32_t c);
  void cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range);
  void cmd_getmatrix(void);
  void cmd_getprops(uint32_t &ptr, uint32_t &w, uint32_t &h);
  void cmd_getptr(void);
  void cmd_gradcolor(uint32_t c);
  void cmd_gradient(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1);
  void cmd_inflate(uint32_t ptr);
  void cmd_interrupt(uint32_t ms);
  void cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h, byte font, uint16_t options, const char*s);
  void cmd_loadidentity(void);
  void cmd_loadimage(uint32_t ptr, int32_t options);
  void cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num);
  void cmd_memset(uint32_t ptr, byte value, uint32_t num);
  uint32_t cmd_memcrc(uint32_t ptr, uint32_t num);
  void cmd_memwrite(uint32_t ptr, uint32_t num);
  void cmd_regwrite(uint32_t ptr, uint32_t val);
  void cmd_number(int16_t x, int16_t y, byte font, uint16_t options, uint32_t n);
  void cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range);
  void cmd_regread(uint32_t ptr);
  void cmd_rotate(int32_t a);
  void cmd_scale(int32_t sx, int32_t sy);
  void cmd_screensaver(void);
  void cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t size, uint16_t range);
  void cmd_setfont(byte font, uint32_t ptr);
  void cmd_setmatrix(void);
  void cmd_sketch(int16_t x, int16_t y, uint16_t w, uint16_t h, uint32_t ptr, uint16_t format);
  void cmd_slider(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range);
  void cmd_snapshot(uint32_t ptr);
  void cmd_spinner(int16_t x, int16_t y, byte style, byte scale);
  void cmd_stop(void);
  void cmd_swap(void);
  void cmd_text(int16_t x, int16_t y, byte font, uint16_t options, const char *s);
  void cmd_toggle(int16_t x, int16_t y, int16_t w, byte font, uint16_t options, uint16_t state, const char *s);
  void cmd_track(int16_t x, int16_t y, uint16_t w, uint16_t h, byte tag);
  void cmd_translate(int32_t tx, int32_t ty);

  byte rd(uint32_t addr);
  void wr(uint32_t addr, uint8_t v);
  uint16_t rd16(uint32_t addr);
  void wr16(uint32_t addr, uint16_t v);
  uint32_t rd32(uint32_t addr);
  void wr32(uint32_t addr, uint32_t v);
  void wr_n(uint32_t addr, byte *src, uint32_t n);

  void cmd32(uint32_t b);

  void bulkrd(uint32_t a);
  void resume(void);
  void __end(void);
  void reset(void);

  void dumpscreen(void);
  byte load(const char *filename, void (*progress)(long, long) = NULL);
  void safeload(const char *filename);

  sdcard SD;

  void storage(void);
  void tune(void);

private:
  static void cFFFFFF(byte v);
  static void cI(uint32_t);
  static void ci(int32_t);
  static void cH(uint16_t);
  static void ch(int16_t);
  static void cs(const char *);
  static void fmtcmd(const char *fmt, ...);

  static void align(byte n);
  void cmdbyte(uint8_t b);

  uint32_t measure_freq(void);

  uint16_t lfsr, lcg;
};

extern GDClass GD;

#if !defined(RASPBERRY_PI)
class Reader {
public:
  int openfile(const char *filename) {
    int i = 0;
    byte dosname[11];
    dirent de;

    dos83(dosname, filename);
    do {
      GD.SD.rdn((byte*)&de, GD.SD.o_root + i * 32, sizeof(de));
      // Serial.println(de.name);
      if (0 == memcmp(de.name, dosname, 11)) {
        begin(de);
        return 1;
      }
      i++;
    } while (de.name[0]);
    return 0;
  }
  void begin(dirent &de) {
    size = de.size;
    cluster = de.cluster;
    if (GD.SD.type == FAT32)
      cluster |= ((long)de.cluster_hi << 16);
    sector = 0;
    offset = 0;
  }
  void nextcluster() {
    if (GD.SD.type == FAT16)
      cluster = GD.SD.rd2(GD.SD.o_fat + 2 * cluster);
    else
      cluster = GD.SD.rd4(GD.SD.o_fat + 4 * cluster);
#if VERBOSE
    Serial.print("nextcluster=");
    Serial.println(cluster, DEC);
#endif
  }
  void skipcluster() {
    nextcluster();
    offset += GD.SD.cluster_size;
  }
  void skipsector() {
    if (sector == GD.SD.sectors_per_cluster) {
      sector = 0;
      nextcluster();
    }
    sector++;
    offset += 512;
  }
  void seek(uint32_t o) {
    while (offset < o) {
      if ((sector == GD.SD.sectors_per_cluster) && ((o - offset) > (long)GD.SD.cluster_size))
        skipcluster();
      else
        skipsector();
    }
  }
  void readsector() {
    if (sector == GD.SD.sectors_per_cluster) {
      sector = 0;
      nextcluster();
    }
    uint32_t off = GD.SD.o_data + ((long)GD.SD.cluster_size * cluster) + (512L * sector);
#if VERBOSE
    Serial.print("off=0x");
    Serial.println(off, HEX);
#endif
    GD.SD.cmd17(off & ~511L);
// Serial.println(2 * (micros() - t0), DEC);
    sector++;
    offset += 512;
  }
  void readsector(byte *dst) {
    readsector();
    for (int i = 0; i < 64; i++) {
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
      *dst++ = SPI.transfer(0xff);
    }
    SPI.transfer(0xff);   // consume CRC
    SPI.transfer(0xff);
    GD.SD.desel();
  }
  uint32_t cluster;
  uint32_t offset;
  uint32_t size;
  byte sector;
};
#endif

typedef struct {
  byte handle;
  uint16_t w, h;
  uint16_t size;
} shape_t;

// Convert degrees to Furmans
#define DEGREES(n) ((65536UL * (n)) / 360)

#define NEVER                0
#define LESS                 1
#define LEQUAL               2
#define GREATER              3
#define GEQUAL               4
#define EQUAL                5
#define NOTEQUAL             6
#define ALWAYS               7

#define ARGB1555             0
#define L1                   1
#define L4                   2
#define L8                   3
#define RGB332               4
#define ARGB2                5
#define ARGB4                6
#define RGB565               7
#define PALETTED             8
#define TEXT8X8              9
#define TEXTVGA              10
#define BARGRAPH             11

#define NEAREST              0
#define BILINEAR             1

#define BORDER               0
#define REPEAT               1

#define KEEP                 1
#define REPLACE              2
#define INCR                 3
#define DECR                 4
#define INVERT               5

#define DLSWAP_DONE          0
#define DLSWAP_LINE          1
#define DLSWAP_FRAME         2

#define INT_SWAP             1
#define INT_TOUCH            2
#define INT_TAG              4
#define INT_SOUND            8
#define INT_PLAYBACK         16
#define INT_CMDEMPTY         32
#define INT_CMDFLAG          64
#define INT_CONVCOMPLETE     128

#define TOUCHMODE_OFF        0
#define TOUCHMODE_ONESHOT    1
#define TOUCHMODE_FRAME      2
#define TOUCHMODE_CONTINUOUS 3

#define ZERO                 0
#define ONE                  1
#define SRC_ALPHA            2
#define DST_ALPHA            3
#define ONE_MINUS_SRC_ALPHA  4
#define ONE_MINUS_DST_ALPHA  5

#define BITMAPS              1
#define POINTS               2
#define LINES                3
#define LINE_STRIP           4
#define EDGE_STRIP_R         5
#define EDGE_STRIP_L         6
#define EDGE_STRIP_A         7
#define EDGE_STRIP_B         8
#define RECTS                9

#define OPT_MONO             1
#define OPT_NODL             2
#define OPT_FLAT             256
#define OPT_CENTERX          512
#define OPT_CENTERY          1024
#define OPT_CENTER           (OPT_CENTERX | OPT_CENTERY)
#define OPT_NOBACK           4096
#define OPT_NOTICKS          8192
#define OPT_NOHM             16384
#define OPT_NOPOINTER        16384
#define OPT_NOSECS           32768
#define OPT_NOHANDS          49152
#define OPT_RIGHTX           2048
#define OPT_SIGNED           256

#define LINEAR_SAMPLES       0
#define ULAW_SAMPLES         1
#define ADPCM_SAMPLES        2

// 'instrument' argument to GD.play()

#define SILENCE              0x00

#define SQUAREWAVE           0x01
#define SINEWAVE             0x02
#define SAWTOOTH             0x03
#define TRIANGLE             0x04

#define BEEPING              0x05
#define ALARM                0x06
#define WARBLE               0x07
#define CAROUSEL             0x08

#define PIPS(n)              (0x0f + (n))

#define HARP                 0x40
#define XYLOPHONE            0x41
#define TUBA                 0x42
#define GLOCKENSPIEL         0x43
#define ORGAN                0x44
#define TRUMPET              0x45
#define PIANO                0x46
#define CHIMES               0x47
#define MUSICBOX             0x48
#define BELL                 0x49

#define CLICK                0x50
#define SWITCH               0x51
#define COWBELL              0x52
#define NOTCH                0x53
#define HIHAT                0x54
#define KICKDRUM             0x55
#define POP                  0x56
#define CLACK                0x57
#define CHACK                0x58

#define MUTE                 0x60
#define UNMUTE               0x61

#define RAM_CMD              1081344UL
#define RAM_DL               1048576UL
#define RAM_PAL              1056768UL

#define REG_CLOCK            1057800UL
#define REG_CMD_DL           1058028UL
#define REG_CMD_READ         1058020UL
#define REG_CMD_WRITE        1058024UL
#define REG_CPURESET         1057820UL
#define REG_CSPREAD          1057892UL
#define REG_DITHER           1057884UL
#define REG_DLSWAP           1057872UL
#define REG_FRAMES           1057796UL
#define REG_FREQUENCY        1057804UL
#define REG_GPIO             1057936UL
#define REG_GPIO_DIR         1057932UL
#define REG_HCYCLE           1057832UL
#define REG_HOFFSET          1057836UL
#define REG_HSIZE            1057840UL
#define REG_HSYNC0           1057844UL
#define REG_HSYNC1           1057848UL
#define REG_ID               1057792UL
#define REG_INT_EN           1057948UL
#define REG_INT_FLAGS        1057944UL
#define REG_INT_MASK         1057952UL
#define REG_MACRO_0          1057992UL
#define REG_MACRO_1          1057996UL
#define REG_OUTBITS          1057880UL
#define REG_PCLK             1057900UL
#define REG_PCLK_POL         1057896UL
#define REG_PLAY             1057928UL
#define REG_PLAYBACK_FORMAT  1057972UL
#define REG_PLAYBACK_FREQ    1057968UL
#define REG_PLAYBACK_LENGTH  1057960UL
#define REG_PLAYBACK_LOOP    1057976UL
#define REG_PLAYBACK_PLAY    1057980UL
#define REG_PLAYBACK_READPTR 1057964UL
#define REG_PLAYBACK_START   1057956UL
#define REG_PWM_DUTY         1057988UL
#define REG_PWM_HZ           1057984UL
#define REG_ROTATE           1057876UL
#define REG_SOUND            1057924UL
#define REG_SWIZZLE          1057888UL
#define REG_TAG              1057912UL
#define REG_TAG_X            1057904UL
#define REG_TAG_Y            1057908UL
#define REG_TOUCH_ADC_MODE   1058036UL
#define REG_TOUCH_CHARGE     1058040UL
#define REG_TOUCH_DIRECT_XY  1058164UL
#define REG_TOUCH_DIRECT_Z1Z2 1058168UL
#define REG_TOUCH_MODE       1058032UL
#define REG_TOUCH_OVERSAMPLE 1058048UL
#define REG_TOUCH_RAW_XY     1058056UL
#define REG_TOUCH_RZ         1058060UL
#define REG_TOUCH_RZTHRESH   1058052UL
#define REG_TOUCH_SCREEN_XY  1058064UL
#define REG_TOUCH_SETTLE     1058044UL
#define REG_TOUCH_TAG        1058072UL
#define REG_TOUCH_TAG_XY     1058068UL
#define REG_TOUCH_TRANSFORM_A 1058076UL
#define REG_TOUCH_TRANSFORM_B 1058080UL
#define REG_TOUCH_TRANSFORM_C 1058084UL
#define REG_TOUCH_TRANSFORM_D 1058088UL
#define REG_TOUCH_TRANSFORM_E 1058092UL
#define REG_TOUCH_TRANSFORM_F 1058096UL
#define REG_TRACKER          1085440UL
#define REG_VCYCLE           1057852UL
#define REG_VOFFSET          1057856UL
#define REG_VOL_PB           1057916UL
#define REG_VOL_SOUND        1057920UL
#define REG_VSIZE            1057860UL
#define REG_VSYNC0           1057864UL
#define REG_VSYNC1           1057868UL

#define VERTEX2II(x, y, handle, cell) \
        ((2UL << 30) | (((x) & 511UL) << 21) | (((y) & 511UL) << 12) | (((handle) & 31) << 7) | (((cell) & 127) << 0))

#define ROM_PIXEL_FF        0xc0400UL

class Poly {
    int x0, y0, x1, y1;
    int x[8], y[8];
    byte n;
    void restart() {
      n = 0;
      x0 = 16 * 480;
      x1 = 0;
      y0 = 16 * 272;
      y1 = 0;
    }
    void perim() {
      for (byte i = 0; i < n; i++)
        GD.Vertex2f(x[i], y[i]);
      GD.Vertex2f(x[0], y[0]);
    }
  public:
    void begin() {
      restart();

      GD.ColorMask(0,0,0,0);
      GD.StencilOp(KEEP, INVERT);
      GD.StencilFunc(ALWAYS, 255, 255);
    }
    void v(int _x, int _y) {
      x0 = min(x0, _x >> 4);
      x1 = max(x1, _x >> 4);
      y0 = min(y0, _y >> 4);
      y1 = max(y1, _y >> 4);
      x[n] = _x;
      y[n] = _y;
      n++;
    }
    void paint() {
      x0 = max(0, x0);
      y0 = max(0, y0);
      x1 = min(16 * 480, x1);
      y1 = min(16 * 272, y1);
      GD.ScissorXY(x0, y0);
      GD.ScissorSize(x1 - x0 + 1, y1 - y0 + 1);
      GD.Begin(EDGE_STRIP_B);
      perim();
    }
    void finish() {
      GD.ColorMask(1,1,1,1);
      GD.StencilFunc(EQUAL, 255, 255);

      GD.Begin(EDGE_STRIP_B);
      GD.Vertex2ii(0, 0);
      GD.Vertex2ii(511, 0);
    }
    void draw() {
      paint();
      finish();
    }
    void outline() {
      GD.Begin(LINE_STRIP);
      perim();
    }
};

#if !defined(RASPBERRY_PI)
class Streamer {
public:
  void begin(const char *rawsamples,
             uint16_t freq = 44100,
             byte format = ADPCM_SAMPLES,
             uint32_t _base = (0x40000UL - 8192), uint16_t size = 8192) {
    r.openfile(rawsamples);

    base = _base;
    mask = size - 1;
    wp = 0;

    for (byte i = 10; i; i--)
      feed();

    GD.sample(base, size, freq, format, 1);
  }
  int feed() {
    uint16_t rp = GD.rd32(REG_PLAYBACK_READPTR) - base;
    uint16_t freespace = mask & ((rp - 1) - wp);
    if (freespace >= 512) {
      // REPORT(base);
      // REPORT(rp);
      // REPORT(wp);
      // REPORT(freespace);
      // Serial.println();
      byte buf[512];
      // uint16_t n = min(512, r.size - r.offset);
      // n = (n + 3) & ~3;   // force 32-bit alignment
      GD.__end();
      r.readsector(buf);
      GD.resume();
      GD.cmd_memwrite(base + wp, 512);
      GD.copyram(buf, 512);
      wp = (wp + 512) & mask;
    }
    return r.offset < r.size;
  }
  void progress(uint16_t &val, uint16_t &range) {
    uint32_t m = r.size;
    uint32_t p = min(r.offset, m);
    while (m > 0x10000) {
      m >>= 1;
      p >>= 1;
    }
    val = p;
    range = m;
  }
private:
  Reader r;
  uint32_t base;
  uint16_t mask;
  uint16_t wp;
};
#else
class Streamer {
public:
  void begin(const char *rawsamples,
             uint16_t freq = 44100,
             byte format = ADPCM_SAMPLES,
             uint32_t _base = (0x40000UL - 4096), uint16_t size = 4096) {}
  int feed() {}
  void progress(uint16_t &val, uint16_t &range) {}
};

#endif

static byte sinus(byte x)
{
  return 128 + GD.rsin(128, -16384 + (x << 7));
}


#endif

