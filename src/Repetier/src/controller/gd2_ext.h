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


#define RGB(r, g, b)    ((uint32_t)((((r) & 0xffL) << 16) | (((g) & 0xffL) << 8) | ((b) & 0xffL)))
#define F8(x)           (int((x) * 256L))
#define F16(x)          ((int32_t)((x) * 65536L))

#define GD_CALIBRATE    (EEPROM_MODE != 0 ? 1 : 0)
#define GD_TRIM         2
#define GD_STORAGE      0 // 4

////////////////////////////////////////////////////////////////////////

class GDClass
{
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
    struct
    {
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
    void cmd_textP(int16_t x, int16_t y, byte font, uint16_t options, PGM_P s);
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

typedef struct
{
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

class Poly
{
    int x0, y0, x1, y1;
    int x[8], y[8];
    byte n;
    void restart()
    {
        n = 0;
        x0 = 16 * 480;
        x1 = 0;
        y0 = 16 * 272;
        y1 = 0;
    }
    void perim()
    {
        for (byte i = 0; i < n; i++)
            GD.Vertex2f(x[i], y[i]);
        GD.Vertex2f(x[0], y[0]);
    }
public:
    void begin()
    {
        restart();

        GD.ColorMask(0,0,0,0);
        GD.StencilOp(KEEP, INVERT);
        GD.StencilFunc(ALWAYS, 255, 255);
    }
    void v(int _x, int _y)
    {
        x0 = RMath::min(x0, _x >> 4);
        x1 = RMath::max(x1, _x >> 4);
        y0 = RMath::min(y0, _y >> 4);
        y1 = RMath::max(y1, _y >> 4);
        x[n] = _x;
        y[n] = _y;
        n++;
    }
    void paint()
    {
        x0 = RMath::max(0, x0);
        y0 = RMath::max(0, y0);
        x1 = RMath::min(16 * 480, x1);
        y1 = RMath::min(16 * 272, y1);
        GD.ScissorXY(x0, y0);
        GD.ScissorSize(x1 - x0 + 1, y1 - y0 + 1);
        GD.Begin(EDGE_STRIP_B);
        perim();
    }
    void finish()
    {
        GD.ColorMask(1,1,1,1);
        GD.StencilFunc(EQUAL, 255, 255);

        GD.Begin(EDGE_STRIP_B);
        GD.Vertex2ii(0, 0);
        GD.Vertex2ii(511, 0);
    }
    void draw()
    {
        paint();
        finish();
    }
    void outline()
    {
        GD.Begin(LINE_STRIP);
        perim();
    }
};

static byte sinus(byte x)
{
    return 128 + GD.rsin(128, -16384 + (x << 7));
}


#define PROTO         1
#define STORAGE       0
#define CALIBRATION   1
#define DUMP_INPUTS   0
#define VERBOSE       0



class GDTransport
{
public:
    void begin()
    {
        SET_OUTPUT(UI_DISPLAY_CS);
        WRITE(UI_DISPLAY_CS, HIGH);

        HAL::spiInit(0);
        hostcmd(0x00);
#if PROTO == 0
        hostcmd(0x44); // from external crystal
#endif
        hostcmd(0x68);
        wp = 0;
        freespace = 4096 - 4;
        stream();
    }

    void cmd32(uint32_t x)
    {
        if (freespace < 4)
        {
            getfree(4);
        }
        wp += 4;
        freespace -= 4;
        union
        {
            uint32_t c;
            uint8_t b[4];
        };
        c = x;
        HAL::spiSend(b[0]);
        HAL::spiSend(b[1]);
        HAL::spiSend(b[2]);
        HAL::spiSend(b[3]);
    }
    void cmdbyte(byte x)
    {
        if (freespace == 0)
        {
            getfree(1);
        }
        wp++;
        freespace--;
        HAL::spiSend(x);
    }
    void cmd_n(byte *s, uint16_t n)
    {
        if (freespace < n)
        {
            getfree(n);
        }
        wp += n;
        freespace -= n;
        while (n > 8)
        {
            n -= 8;
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
            HAL::spiSend(*s++);
        }
        while (n--)
            HAL::spiSend(*s++);
    }

    inline void flush()
    {
        getfree(0);
    }
    uint16_t rp()
    {
        uint16_t r = __rd16(REG_CMD_READ);
        if (r == 0xfff)
        {
            Com::printErrorFLN(PSTR("Gameduino 2 signales defect state"));HAL::delayMilliseconds(100);
            //REPORT(/*EXCEPTION*/r);
            for (;;) ;
        }
        return r;
    }
    void finish()
    {
        wp &= 0xffc;
        __end();
        __wr16(REG_CMD_WRITE, wp);
        while (rp() != wp)
            ;
        stream();
    }

    byte rd(uint32_t addr)
    {
        __end(); // stop streaming
        __start(addr);
        HAL::spiSend(0);  // dummy
        byte r = HAL::spiReceive(0);
        stream();
        return r;
    }

    void wr(uint32_t addr, byte v)
    {
        __end(); // stop streaming
        __wstart(addr);
        HAL::spiSend(v);
        stream();
    }

    uint16_t rd16(uint32_t addr)
    {
        uint16_t r = 0;
        __end(); // stop streaming
        __start(addr);
        HAL::spiSend(0);
        r = HAL::spiReceive(0);
        r |= (HAL::spiReceive(0) << 8);
        stream();
        return r;
    }

    void wr16(uint32_t addr, uint32_t v)
    {
        __end(); // stop streaming
        __wstart(addr);
        HAL::spiSend(v);
        HAL::spiSend(v >> 8);
        stream();
    }

    uint32_t rd32(uint32_t addr)
    {
        __end(); // stop streaming
        __start(addr);
        HAL::spiSend(0);
        union
        {
            uint32_t c;
            uint8_t b[4];
        };
        b[0] = HAL::spiReceive(0);
        b[1] = HAL::spiReceive(0);
        b[2] = HAL::spiReceive(0);
        b[3] = HAL::spiReceive(0);
        stream();
        return c;
    }
    void rd_n(byte *dst, uint32_t addr, uint16_t n)
    {
        __end(); // stop streaming
        __start(addr);
        HAL::spiSend(0);
        while (n--)
            *dst++ = HAL::spiReceive(0);
        stream();
    }
    void wr_n(uint32_t addr, byte *src, uint16_t n)
    {
        __end(); // stop streaming
        __wstart(addr);
        while (n--)
            HAL::spiSend(*src++);
        stream();
    }

    void wr32(uint32_t addr, unsigned long v)
    {
        __end(); // stop streaming
        __wstart(addr);
        HAL::spiSend(v);
        HAL::spiSend(v >> 8);
        HAL::spiSend(v >> 16);
        HAL::spiSend(v >> 24);
        stream();
    }

    uint32_t getwp(void)
    {
        return RAM_CMD + (wp & 0xffc);
    }

    void bulk(uint32_t addr)
    {
        __end(); // stop streaming
        __start(addr);
    }
    inline void resume(void)
    {
        // REPORT(__rd16(REG_ID));
        stream();
    }

    static void __start(uint32_t addr) // start an SPI transaction to addr
    {
        WRITE(UI_DISPLAY_CS, LOW);
        HAL::spiSend(addr >> 16);
        HAL::spiSend(highByte(addr));
        HAL::spiSend(lowByte(addr));
    }

    static void __wstart(uint32_t addr) // start an SPI write transaction to addr
    {
        WRITE(UI_DISPLAY_CS, LOW);
        HAL::spiSend(0x80 | (addr >> 16));
        HAL::spiSend(highByte(addr));
        HAL::spiSend(lowByte(addr));
    }

    static inline void __end() // end the SPI transaction
    {
        WRITE(UI_DISPLAY_CS, HIGH);
        HAL::delayMicroseconds(1);
    }

    void stop() // end the SPI transaction
    {
        wp &= 0xffc;
        __end();
        __wr16(REG_CMD_WRITE, wp);
        // while (__rd16(REG_CMD_READ) != wp) ;
    }

    inline void stream(void)
    {
        __end();
        __wstart(RAM_CMD + (wp & 0xfff));
    }

    static unsigned int __rd16(uint32_t addr)
    {
        unsigned int r;

        __start(addr);
        HAL::spiSend(0);  // dummy
        r = HAL::spiReceive(0);
        r |= (((uint16_t)HAL::spiReceive(0)) << 8);
        __end();
        return r;
    }

    static void __wr16(uint32_t addr, unsigned int v)
    {
        __wstart(addr);
        HAL::spiSend(lowByte(v));
        HAL::spiSend(highByte(v));
        __end();
    }

    static void hostcmd(byte a)
    {
        WRITE(UI_DISPLAY_CS, LOW);
        HAL::spiSend(a);
        HAL::spiSend(0x00);
        HAL::spiSend(0x00);
        WRITE(UI_DISPLAY_CS, HIGH);
        HAL::delayMilliseconds(60);
    }

    void getfree(uint16_t n)
    {
        wp &= 0xfff;
        __end();
        __wr16(REG_CMD_WRITE, wp & 0xffc);
        do
        {
            uint16_t fullness = (wp - rp()) & 4095;
            freespace = (4096 - 4) - fullness;
        }
        while (freespace < n);
        stream();
    }

    byte streaming;
    uint16_t wp;
    uint16_t freespace;
};

static GDTransport GDTR;

GDClass GD;
void GDClass::flush(void)
{
    GDTR.flush();
}

void GDClass::swap(void)
{
    Display();
    cmd_swap();
    cmd_loadidentity();
    cmd_dlstart();
    GDTR.flush();
}

uint32_t GDClass::measure_freq(void)
{
    unsigned long t0 = GDTR.rd32(REG_CLOCK);
    HAL::delayMicroseconds(15625);
    unsigned long t1 = GDTR.rd32(REG_CLOCK);
    return (t1 - t0) << 6;
}

#define REG_TRIM        0x10256C
#define LOW_FREQ_BOUND  47040000UL

void GDClass::tune(void)
{
    uint32_t f;
    for (byte i = 0; (i < 31) && ((f = measure_freq()) < LOW_FREQ_BOUND); i++)
        GDTR.wr(REG_TRIM, i);
    GDTR.wr32(REG_FREQUENCY, f);
}

void GDClass::begin(uint8_t options)
{
    GDTR.begin();

#if VERBOSE
    Serial.println("ID REGISTER:");
    Serial.println(GDTR.rd(REG_ID), HEX);
#endif

    // Generate a blank screen
    cmd_dlstart();
    Clear();
    swap();
    finish();

    GDTR.wr(REG_PCLK_POL, 1);
    GDTR.wr(REG_PCLK, 5);
#if PROTO == 1
    GDTR.wr(REG_ROTATE, 1);
    GDTR.wr(REG_SWIZZLE, 3);
#endif
    GDTR.wr(REG_GPIO_DIR, 0x83);
    GDTR.wr(REG_GPIO, 0x80);

    if (options & GD_CALIBRATE)
    {
#if CALIBRATION
        if (HAL::eprGetByte(EPR_TOUCHSCREEN) != 0x7c)
        {
            self_calibrate();
            // for (int i = 0; i < 24; i++) Serial.println(GDTR.rd(REG_TOUCH_TRANSFORM_A + i), HEX);
            for (int i = 0; i < 24; i++)
                HAL::eprSetByte(EPR_TOUCHSCREEN + 1 + i, GDTR.rd(REG_TOUCH_TRANSFORM_A + i));
            HAL::eprSetByte(EPR_TOUCHSCREEN, 0x7c);  // is written!
            uint8_t newcheck = EEPROM::computeChecksum();
            if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
                HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);

        }
        else
        {
            for (int i = 0; i < 24; i++)
                GDTR.wr(REG_TOUCH_TRANSFORM_A + i, HAL::eprGetByte(EPR_TOUCHSCREEN + 1 + i));
        }
#endif


#if CALIBRATION && defined(RASPBERRY_PI)
        {
            uint8_t cal[24];
            FILE *calfile = fopen(".calibration", "r");
            if (calfile == NULL)
            {
                calfile = fopen(".calibration", "w");
                if (calfile != NULL)
                {
                    self_calibrate();
                    for (int i = 0; i < 24; i++)
                        cal[i] = GDTR.rd(REG_TOUCH_TRANSFORM_A + i);
                    fwrite(cal, 1, sizeof(cal), calfile);
                    fclose(calfile);
                }
            }
            else
            {
                fread(cal, 1, sizeof(cal), calfile);
                for (int i = 0; i < 24; i++)
                    GDTR.wr(REG_TOUCH_TRANSFORM_A + i, cal[i]);
                fclose(calfile);
            }
        }
#endif
    }

    GDTR.wr16(REG_TOUCH_RZTHRESH, 1200);

    lfsr = 0x5555;
    lcg = 0;

#if STORAGE && defined(ARDUINO)
    if (options & GD_STORAGE)
    {
        storage();
    }
#endif

    if (options & GD_TRIM)
    {
        tune();
    }
}

void GDClass::self_calibrate(void)
{
    cmd_dlstart();
    Clear();
    cmd_text(240, 100, 30, OPT_CENTERX, "please tap on the dot");
    cmd_calibrate();
    finish();
    cmd_loadidentity();
    cmd_dlstart();
    GDTR.flush();
}

void GDClass::seed(uint16_t n)
{
    lfsr = n | 1;
    lcg = n ^ 0x7921;
}
uint16_t GDClass::random()
{
    lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
    lcg = (lcg * 47) + 60497;
    return (lcg ^ lfsr);
}
uint16_t GDClass::random(uint16_t n)
{
    return GDClass::random() % n;
}


// >>> [int(65535*math.sin(math.pi * 2 * i / 1024)) for i in range(257)]
static const PROGMEM prog_int16_t sintab[257] =
{
    0, 402, 804, 1206, 1608, 2010, 2412, 2813, 3215, 3617, 4018, 4419, 4821, 5221, 5622, 6023, 6423, 6823, 7223, 7622, 8022, 8421, 8819, 9218, 9615, 10013, 10410, 10807, 11203, 11599, 11995, 12390, 12785, 13179, 13573, 13966, 14358, 14750, 15142, 15533, 15923, 16313, 16702, 17091, 17479, 17866, 18252, 18638, 19023, 19408, 19791, 20174, 20557, 20938, 21319, 21699, 22078, 22456, 22833, 23210, 23585, 23960, 24334, 24707, 25079, 25450, 25820, 26189, 26557, 26924, 27290, 27655, 28019, 28382, 28744, 29105, 29465, 29823, 30181, 30537, 30892, 31247, 31599, 31951, 32302, 32651, 32999, 33346, 33691, 34035, 34378, 34720, 35061, 35400, 35737, 36074, 36409, 36742, 37075, 37406, 37735, 38063, 38390, 38715, 39039, 39361, 39682, 40001, 40319, 40635, 40950, 41263, 41574, 41885, 42193, 42500, 42805, 43109, 43411, 43711, 44010, 44307, 44603, 44896, 45189, 45479, 45768, 46055, 46340, 46623, 46905, 47185, 47463, 47739, 48014, 48287, 48558, 48827, 49094, 49360, 49623, 49885, 50145, 50403, 50659, 50913, 51165, 51415, 51664, 51910, 52155, 52397, 52638, 52876, 53113, 53347, 53580, 53810, 54039, 54265, 54490, 54712, 54933, 55151, 55367, 55581, 55793, 56003, 56211, 56416, 56620, 56821, 57021, 57218, 57413, 57606, 57796, 57985, 58171, 58355, 58537, 58717, 58894, 59069, 59242, 59413, 59582, 59748, 59912, 60074, 60234, 60391, 60546, 60699, 60849, 60997, 61143, 61287, 61428, 61567, 61704, 61838, 61970, 62100, 62227, 62352, 62474, 62595, 62713, 62828, 62941, 63052, 63161, 63267, 63370, 63472, 63570, 63667, 63761, 63853, 63942, 64029, 64114, 64196, 64275, 64353, 64427, 64500, 64570, 64637, 64702, 64765, 64825, 64883, 64938, 64991, 65042, 65090, 65135, 65178, 65219, 65257, 65293, 65326, 65357, 65385, 65411, 65435, 65456, 65474, 65490, 65504, 65515, 65523, 65530, 65533, 65535
};

int16_t GDClass::rsin(int16_t r, uint16_t th)
{
    th >>= 6; // angle 0-123
    // return int(r * sin((2 * M_PI) * th / 1024.));
    int th4 = th & 511;
    if (th4 & 256)
        th4 = 512 - th4; // 256->256 257->255, etc
    uint16_t s = pgm_read_word_near(sintab + th4);
    int16_t p = ((uint32_t)s * r) >> 16;
    if (th & 512)
        p = -p;
    return p;
}

int16_t GDClass::rcos(int16_t r, uint16_t th)
{
    return rsin(r, th + 0x4000);
}

void GDClass::polar(int &x, int &y, int16_t r, uint16_t th)
{
    x = (int)(-GD.rsin(r, th));
    y = (int)( GD.rcos(r, th));
}

// >>> [int(round(1024 * math.atan(i / 256.) / math.pi)) for i in range(256)]
static const PROGMEM prog_uchar atan8[] =
{
    0,1,3,4,5,6,8,9,10,11,13,14,15,17,18,19,20,22,23,24,25,27,28,29,30,32,33,34,36,37,38,39,41,42,43,44,46,47,48,49,51,52,53,54,55,57,58,59,60,62,63,64,65,67,68,69,70,71,73,74,75,76,77,79,80,81,82,83,85,86,87,88,89,91,92,93,94,95,96,98,99,100,101,102,103,104,106,107,108,109,110,111,112,114,115,116,117,118,119,120,121,122,124,125,126,127,128,129,130,131,132,133,134,135,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,177,178,179,180,181,182,183,184,185,186,187,188,188,189,190,191,192,193,194,195,195,196,197,198,199,200,201,201,202,203,204,205,206,206,207,208,209,210,211,211,212,213,214,215,215,216,217,218,219,219,220,221,222,222,223,224,225,225,226,227,228,228,229,230,231,231,232,233,234,234,235,236,236,237,238,239,239,240,241,241,242,243,243,244,245,245,246,247,248,248,249,250,250,251,251,252,253,253,254,255,255
};

uint16_t GDClass::atan2(int16_t y, int16_t x)
{
    uint16_t a;
    uint16_t xx = 0;

    if ((x <= 0) ^ (y > 0))
    {
        int16_t t;
        t = x;
        x = y;
        y = t;
        xx ^= 0x4000;
    }
    if (x <= 0)
    {
        x = -x;
    }
    else
    {
        xx ^= 0x8000;
    }
    y = abs(y);
    if (x > y)
    {
        int16_t t;
        t = x;
        x = y;
        y = t;
        xx ^= 0x3fff;
    }
    while ((x | y) & 0xff80)
    {
        x >>= 1;
        y >>= 1;
    }
    if (y == 0)
    {
        a = 0;
    }
    else if (x == y)
    {
        a = 0x2000;
    }
    else
    {
        // assert(x <= y);
        int r = ((x << 8) / y);
        // assert(0 <= r);
        // assert(r < 256);
        a = pgm_read_byte(atan8 + r) << 5;
    }
    a ^= xx;
    return a;
}

void GDClass::align(byte n)
{
    while ((n++) & 3)
        GDTR.cmdbyte(0);
}

void GDClass::cH(uint16_t v)
{
    GDTR.cmdbyte(v & 0xff);
    GDTR.cmdbyte((v >> 8) & 0xff);
}

void GDClass::ch(int16_t v)
{
    cH((uint16_t)v);
}

void GDClass::cI(uint32_t v)
{
    GDTR.cmd32(v);
}

void GDClass::cFFFFFF(byte v)
{
    union
    {
        uint32_t c;
        uint8_t b[4];
    };
    b[0] = v;
    b[1] = 0xff;
    b[2] = 0xff;
    b[3] = 0xff;
    GDTR.cmd32(c);
}

void GDClass::ci(int32_t v)
{
    cI((uint32_t) v);
}

void GDClass::cs(const char *s)
{
    while (*s)
    {
        char c = *s++;
        GDTR.cmdbyte(c);
    }
    GDTR.cmdbyte(0);
}

void GDClass::copy(const PROGMEM prog_uchar *src, int count)
{
    byte a = count & 3;
    while (count--)
    {
        GDTR.cmdbyte(pgm_read_byte_near(src));
        src++;
    }
    align(a);
}

void GDClass::copyram(byte *src, int count)
{
    byte a = count & 3;
    GDTR.cmd_n(src, count);
    align(a);
}

void GDClass::AlphaFunc(byte func, byte ref)
{
    cI((9UL << 24) | ((func & 7L) << 8) | ((ref & 255L) << 0));
}
void GDClass::Begin(byte prim)
{
    cI((31UL << 24) | prim);
}
void GDClass::BitmapHandle(byte handle)
{
    cI((5UL << 24) | handle);
}
void GDClass::BitmapLayout(byte format, uint16_t linestride, uint16_t height)
{
    // cI((7UL << 24) | ((format & 31L) << 19) | ((linestride & 1023L) << 9) | ((height & 511L) << 0));
    union
    {
        uint32_t c;
        uint8_t b[4];
    };
    b[0] = height;
    b[1] = (1 & (height >> 8)) | (linestride << 1);
    b[2] = (7 & (linestride >> 7)) | (format << 3);
    b[3] = 7;
    cI(c);
}
void GDClass::BitmapSize(byte filter, byte wrapx, byte wrapy, uint16_t width, uint16_t height)
{
    byte fxy = (filter << 2) | (wrapx << 1) | (wrapy);
    // cI((8UL << 24) | ((uint32_t)fxy << 18) | ((width & 511L) << 9) | ((height & 511L) << 0));
    union
    {
        uint32_t c;
        uint8_t b[4];
    };
    b[0] = height;
    b[1] = (1 & (height >> 8)) | (width << 1);
    b[2] = (3 & (width >> 7)) | (fxy << 2);
    b[3] = 8;
    cI(c);
}
void GDClass::BitmapSource(uint32_t addr)
{
    cI((1UL << 24) | ((addr & 1048575L) << 0));
}
void GDClass::BitmapTransformA(int32_t a)
{
    cI((21UL << 24) | ((a & 131071L) << 0));
}
void GDClass::BitmapTransformB(int32_t b)
{
    cI((22UL << 24) | ((b & 131071L) << 0));
}
void GDClass::BitmapTransformC(int32_t c)
{
    cI((23UL << 24) | ((c & 16777215L) << 0));
}
void GDClass::BitmapTransformD(int32_t d)
{
    cI((24UL << 24) | ((d & 131071L) << 0));
}
void GDClass::BitmapTransformE(int32_t e)
{
    cI((25UL << 24) | ((e & 131071L) << 0));
}
void GDClass::BitmapTransformF(int32_t f)
{
    cI((26UL << 24) | ((f & 16777215L) << 0));
}
void GDClass::BlendFunc(byte src, byte dst)
{
    cI((11UL << 24) | ((src & 7L) << 3) | ((dst & 7L) << 0));
}
void GDClass::Call(uint16_t dest)
{
    cI((29UL << 24) | ((dest & 2047L) << 0));
}
void GDClass::Cell(byte cell)
{
    cI((6UL << 24) | ((cell & 127L) << 0));
}
void GDClass::ClearColorA(byte alpha)
{
    cI((15UL << 24) | ((alpha & 255L) << 0));
}
void GDClass::ClearColorRGB(byte red, byte green, byte blue)
{
    cI((2UL << 24) | ((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
}
void GDClass::ClearColorRGB(uint32_t rgb)
{
    cI((2UL << 24) | (rgb & 0xffffffL));
}
void GDClass::Clear(byte c, byte s, byte t)
{
    byte m = (c << 2) | (s << 1) | t;
    cI((38UL << 24) | m);
}
void GDClass::Clear(void)
{
    cI((38UL << 24) | 7);
}
void GDClass::ClearStencil(byte s)
{
    cI((17UL << 24) | ((s & 255L) << 0));
}
void GDClass::ClearTag(byte s)
{
    cI((18UL << 24) | ((s & 255L) << 0));
}
void GDClass::ColorA(byte alpha)
{
    cI((16UL << 24) | ((alpha & 255L) << 0));
}
void GDClass::ColorMask(byte r, byte g, byte b, byte a)
{
    cI((32UL << 24) | ((r & 1L) << 3) | ((g & 1L) << 2) | ((b & 1L) << 1) | ((a & 1L) << 0));
}
void GDClass::ColorRGB(byte red, byte green, byte blue)
{
    // cI((4UL << 24) | ((red & 255L) << 16) | ((green & 255L) << 8) | ((blue & 255L) << 0));
    union
    {
        uint32_t c;
        uint8_t b[4];
    };
    b[0] = blue;
    b[1] = green;
    b[2] = red;
    b[3] = 4;
    cI(c);
}
void GDClass::ColorRGB(uint32_t rgb)
{
    cI((4UL << 24) | (rgb & 0xffffffL));
}
void GDClass::Display(void)
{
    cI((0UL << 24));
}
void GDClass::End(void)
{
    cI((33UL << 24));
}
void GDClass::Jump(uint16_t dest)
{
    cI((30UL << 24) | ((dest & 2047L) << 0));
}
void GDClass::LineWidth(uint16_t width)
{
    cI((14UL << 24) | ((width & 4095L) << 0));
}
void GDClass::Macro(byte m)
{
    cI((37UL << 24) | ((m & 1L) << 0));
}
void GDClass::PointSize(uint16_t size)
{
    cI((13UL << 24) | ((size & 8191L) << 0));
}
void GDClass::RestoreContext(void)
{
    cI((35UL << 24));
}
void GDClass::Return(void)
{
    cI((36UL << 24));
}
void GDClass::SaveContext(void)
{
    cI((34UL << 24));
}
void GDClass::ScissorSize(uint16_t width, uint16_t height)
{
    cI((28UL << 24) | ((width & 1023L) << 10) | ((height & 1023L) << 0));
}
void GDClass::ScissorXY(uint16_t x, uint16_t y)
{
    cI((27UL << 24) | ((x & 511L) << 9) | ((y & 511L) << 0));
}
void GDClass::StencilFunc(byte func, byte ref, byte mask)
{
    cI((10UL << 24) | ((func & 7L) << 16) | ((ref & 255L) << 8) | ((mask & 255L) << 0));
}
void GDClass::StencilMask(byte mask)
{
    cI((19UL << 24) | ((mask & 255L) << 0));
}
void GDClass::StencilOp(byte sfail, byte spass)
{
    cI((12UL << 24) | ((sfail & 7L) << 3) | ((spass & 7L) << 0));
}
void GDClass::TagMask(byte mask)
{
    cI((20UL << 24) | ((mask & 1L) << 0));
}
void GDClass::Tag(byte s)
{
    cI((3UL << 24) | ((s & 255L) << 0));
}
void GDClass::Vertex2f(int16_t x, int16_t y)
{
    // x = int(16 * x);
    // y = int(16 * y);
    cI((1UL << 30) | ((x & 32767L) << 15) | ((y & 32767L) << 0));
}
void GDClass::Vertex2ii(uint16_t x, uint16_t y, byte handle, byte cell)
{
    // cI((2UL << 30) | ((x & 511L) << 21) | ((y & 511L) << 12) | ((handle & 31L) << 7) | ((cell & 127L) << 0));
    union
    {
        uint32_t c;
        uint8_t b[4];
    };
    b[0] = cell | ((handle & 1) << 7);
    b[1] = (handle >> 1) | (y << 4);
    b[2] = (y >> 4) | (x << 5);
    b[3] = (2 << 6) | (x >> 3);
    cI(c);
}

void GDClass::fmtcmd(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    byte sz = 0;  // Only the low 2 bits matter
    const char *s;

    while (*fmt)
        switch (*fmt++)
        {
        case 'i':
        case 'I':
            cI(va_arg(ap, uint32_t));
            break;
        case 'h':
        case 'H':
            cH(va_arg(ap, unsigned int));
            sz += 2;
            break;
        case 's':
            s = va_arg(ap, const char*);
            cs(s);
            sz += strlen(s) + 1;
            break;
        }
    align(sz);
}

void GDClass::cmd_append(uint32_t ptr, uint32_t num)
{
    cFFFFFF(0x1e);
    cI(ptr);
    cI(num);
}
void GDClass::cmd_bgcolor(uint32_t c)
{
    fmtcmd("II", 0xffffff09UL, c);
}
void GDClass::cmd_button(int16_t x, int16_t y, uint16_t w, uint16_t h, byte font, uint16_t options, const char *s)
{
    fmtcmd("IhhhhhHs", 0xffffff0dUL, x, y, w, h, font, options, s);
}
void GDClass::cmd_calibrate(void)
{
    cFFFFFF(0x15);
    cFFFFFF(0xff);
}
void GDClass::cmd_clock(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms)
{
    fmtcmd("IhhhHHHHH", 0xffffff14UL, x, y, r, options, h, m, s, ms);
}
void GDClass::cmd_coldstart(void)
{
    cFFFFFF(0x32);
}
void GDClass::cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val)
{
    fmtcmd("IhhhHH", 0xffffff2dUL, x, y, r, options, val);
}
void GDClass::cmd_dlstart(void)
{
    cFFFFFF(0x00);
}
void GDClass::cmd_fgcolor(uint32_t c)
{
    fmtcmd("II", 0xffffff0aUL, c);
}
void GDClass::cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range)
{
    fmtcmd("IhhhHHHHH", 0xffffff13UL, x, y, r, options, major, minor, val, range);
}
void GDClass::cmd_getmatrix(void)
{
    fmtcmd("Iiiiiii", 0xffffff33UL, 0, 0, 0, 0, 0, 0);
}
void GDClass::cmd_getprops(uint32_t &ptr, uint32_t &w, uint32_t &h)
{
    cFFFFFF(0x25);
    ptr = GDTR.getwp();
    cI(0);
    w = GDTR.getwp();
    cI(0);
    h = GDTR.getwp();
    cI(0);
}
void GDClass::cmd_getptr(void)
{
    fmtcmd("II", 0xffffff23UL, 0);
}
void GDClass::cmd_gradcolor(uint32_t c)
{
    fmtcmd("II", 0xffffff34UL, c);
}
void GDClass::cmd_gradient(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1)
{
    fmtcmd("IhhIhhI", 0xffffff0bUL, x0, y0, rgb0, x1, y1, rgb1);
}
void GDClass::cmd_inflate(uint32_t ptr)
{
    cFFFFFF(0x22);
    cI(ptr);
}
void GDClass::cmd_interrupt(uint32_t ms)
{
    fmtcmd("II", 0xffffff02UL, ms);
}
void GDClass::cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h, byte font, uint16_t options, const char*s)
{
    fmtcmd("IhhhhhHs", 0xffffff0eUL, x, y, w, h, font, options, s);
}
void GDClass::cmd_loadidentity(void)
{
    cFFFFFF(0x26);
}
void GDClass::cmd_loadimage(uint32_t ptr, int32_t options)
{
    fmtcmd("III", 0xffffff24UL, ptr, options);
}
void GDClass::cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num)
{
    fmtcmd("IIII", 0xffffff1dUL, dest, src, num);
}
void GDClass::cmd_memset(uint32_t ptr, byte value, uint32_t num)
{
    cFFFFFF(0x1b);
    cI(ptr);
    cI((uint32_t)value);
    cI(num);
}
uint32_t GDClass::cmd_memcrc(uint32_t ptr, uint32_t num)
{
    cFFFFFF(0x18);
    cI(ptr);
    cI(num);
    uint32_t r = GDTR.getwp();
    cI(0xFFFFFFFF);
    return r;
}
void GDClass::cmd_memwrite(uint32_t ptr, uint32_t num)
{
    fmtcmd("III", 0xffffff1aUL, ptr, num);
}
void GDClass::cmd_regwrite(uint32_t ptr, uint32_t val)
{
    cFFFFFF(0x1a);
    cI(ptr);
    cI(4UL);
    cI(val);
}
void GDClass::cmd_number(int16_t x, int16_t y, byte font, uint16_t options, uint32_t n)
{
    // fmtcmd("IhhhHi", 0xffffff2eUL, x, y, font, options, n);
    cFFFFFF(0x2e);
    ch(x);
    ch(y);
    ch(font);
    cH(options);
    ci(n);
}
void GDClass::cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range)
{
    fmtcmd("IhhhhHHH", 0xffffff0fUL, x, y, w, h, options, val, range);
}
void GDClass::cmd_regread(uint32_t ptr)
{
    fmtcmd("III", 0xffffff19UL, ptr, 0);
}
void GDClass::cmd_rotate(int32_t a)
{
    cFFFFFF(0x29);
    ci(a);
}
void GDClass::cmd_scale(int32_t sx, int32_t sy)
{
    cFFFFFF(0x28);
    ci(sx);
    ci(sy);
}
void GDClass::cmd_screensaver(void)
{
    cFFFFFF(0x2f);
}
void GDClass::cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t size, uint16_t range)
{
    fmtcmd("IhhhhHHHH", 0xffffff11UL, x, y, w, h, options, val, size, range);
}
void GDClass::cmd_setfont(byte font, uint32_t ptr)
{
    fmtcmd("III", 0xffffff2bUL, font, ptr);
}
void GDClass::cmd_setmatrix(void)
{
    cFFFFFF(0x2a);
}
void GDClass::cmd_sketch(int16_t x, int16_t y, uint16_t w, uint16_t h, uint32_t ptr, uint16_t format)
{
    cFFFFFF(0x30);
    ch(x);
    ch(y);
    cH(w);
    cH(h);
    cI(ptr);
    cI(format);
}
void GDClass::cmd_slider(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range)
{
    fmtcmd("IhhhhHHH", 0xffffff10UL, x, y, w, h, options, val, range);
}
void GDClass::cmd_snapshot(uint32_t ptr)
{
    fmtcmd("II", 0xffffff1fUL, ptr);
}
void GDClass::cmd_spinner(int16_t x, int16_t y, byte style, byte scale)
{
    cFFFFFF(0x16);
    ch(x);
    ch(y);
    cH(style);
    cH(scale);
}
void GDClass::cmd_stop(void)
{
    cFFFFFF(0x17);
}
void GDClass::cmd_swap(void)
{
    cFFFFFF(0x01);
}
void GDClass::cmd_text(int16_t x, int16_t y, byte font, uint16_t options, const char *s)
{
    // fmtcmd("IhhhHs", 0xffffff0cUL, x, y, font, options, s);
    cFFFFFF(0x0c);
    ch(x);
    ch(y);
    ch(font);
    cH(options);
    cs(s);
    align(strlen(s) + 1);
}
void GDClass::cmd_textP(int16_t x, int16_t y, byte font, uint16_t options, PGM_P sp) {
    char s[60];
    char cr;
    uint8_t len = 0;
    do {
        cr = HAL::readFlashByte(sp++);
        s[len++] = cr;
    } while(cr);
    len--;
    // fmtcmd("IhhhHs", 0xffffff0cUL, x, y, font, options, s);
    cFFFFFF(0x0c);
    ch(x);
    ch(y);
    ch(font);
    cH(options);
    cs(s);
    align(strlen(s) + 1);
}
void GDClass::cmd_toggle(int16_t x, int16_t y, int16_t w, byte font, uint16_t options, uint16_t state, const char *s)
{
    fmtcmd("IhhhhHHs", 0xffffff12UL, x, y, w, font, options, state, s);
}
void GDClass::cmd_track(int16_t x, int16_t y, uint16_t w, uint16_t h, byte tag)
{
    fmtcmd("Ihhhhh", 0xffffff2cUL, x, y, w, h, tag);
}
void GDClass::cmd_translate(int32_t tx, int32_t ty)
{
    cFFFFFF(0x27);
    ci(tx);
    ci(ty);
}

byte GDClass::rd(uint32_t addr)
{
    return GDTR.rd(addr);
}
void GDClass::wr(uint32_t addr, uint8_t v)
{
    GDTR.wr(addr, v);
}
uint16_t GDClass::rd16(uint32_t addr)
{
    return GDTR.rd16(addr);
}
void GDClass::wr16(uint32_t addr, uint16_t v)
{
    GDTR.wr16(addr, v);
}
uint32_t GDClass::rd32(uint32_t addr)
{
    return GDTR.rd32(addr);
}
void GDClass::wr32(uint32_t addr, uint32_t v)
{
    GDTR.wr32(addr, v);
}
void GDClass::wr_n(uint32_t addr, byte *src, uint32_t n)
{
    GDTR.wr_n(addr, src, n);
}

void GDClass::cmdbyte(uint8_t b)
{
    GDTR.cmdbyte(b);
}
void GDClass::cmd32(uint32_t b)
{
    GDTR.cmd32(b);
}
void GDClass::finish(void)
{
    GDTR.finish();
}
void GDClass::get_accel(int &x, int &y, int &z)
{
    static int f[3];

    for (byte i = 0; i < 3; i++)
    {
        int a = analogRead(A0 + i);
        // Serial.print(a, DEC); Serial.print(" ");
        int s = (-160 * (a - 376)) >> 6;
        f[i] = ((3 * f[i]) >> 2) + (s >> 2);
    }
    // Serial.println();
    x = f[2];
    y = f[1];
    z = f[0];
}
void GDClass::get_inputs(void)
{
    GDTR.finish();
    byte *bi = (byte*)&inputs;
    GDTR.rd_n(bi, REG_TRACKER, 4);
    GDTR.rd_n(bi + 4, REG_TOUCH_RZ, 13);
    GDTR.rd_n(bi + 17, REG_TAG, 1);
#if DUMP_INPUTS
    for (size_t i = 0; i < sizeof(inputs); i++)
    {
        Serial.print(bi[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif
}
void GDClass::bulkrd(uint32_t a)
{
    GDTR.bulk(a);
}
void GDClass::resume(void)
{
    GDTR.resume();
}
void GDClass::__end(void)
{
    GDTR.__end();
}
void GDClass::play(uint8_t instrument, uint8_t note)
{
    wr16(REG_SOUND, (note << 8) | instrument);
    wr(REG_PLAY, 1);
}
void GDClass::sample(uint32_t start, uint32_t len, uint16_t freq, uint16_t format, int loop)
{
    GD.wr32(REG_PLAYBACK_START, start);
    GD.wr32(REG_PLAYBACK_LENGTH, len);
    GD.wr16(REG_PLAYBACK_FREQ, freq);
    GD.wr(REG_PLAYBACK_FORMAT, format);
    GD.wr(REG_PLAYBACK_LOOP, loop);
    GD.wr(REG_PLAYBACK_PLAY, 1);
}
void GDClass::reset()
{
    GDTR.__end();
    GDTR.wr(REG_CPURESET, 1);
    GDTR.wr(REG_CPURESET, 0);
    GDTR.resume();
}

// Generated by mk_bsod.py. Blue screen with 'ERROR' text
static const PROGMEM prog_uchar __bsod[31] =
{
    0, 255, 255, 255, 255, 0, 0, 2, 7, 0, 0, 38, 12, 255, 255, 255, 240,
    0, 120, 0, 28, 0, 0, 6, 69, 82, 82, 79, 82, 33, 0
};
// "Cannot open file" text
static const PROGMEM prog_uchar __bsod_badfile[31] =
{
    12, 255, 255, 255, 240, 0, 148, 0, 28, 0, 0, 6, 67, 97, 110, 110, 111,
    116, 32, 111, 112, 101, 110, 32, 102, 105, 108, 101, 0, 0, 0
};

/*

Now come the images we developed for the firmware.
They are inserted here to not fill in another file.

*/
static const PROGMEM prog_uchar __assets[9797] = {
0, 0, 0, 5, 0, 0, 0, 1, 32, 64, 0, 8, 32, 128, 0, 7, 1, 0, 0, 5, 0,
24, 1, 1, 22, 116, 2, 8, 22, 232, 4, 7, 2, 0, 0, 5, 240, 131, 1, 1,
22, 116, 2, 8, 22, 232, 4, 7, 34, 255, 255, 255, 0, 0, 0, 0, 120, 156,
237, 125, 125, 112, 91, 215, 117, 39, 50, 105, 90, 36, 253, 162, 252,
71, 135, 78, 255, 176, 208, 52, 211, 65, 182, 155, 29, 82, 59, 187,
133, 211, 25, 203, 140, 63, 50, 204, 180, 211, 37, 213, 217, 25, 40,
153, 176, 97, 102, 147, 44, 181, 147, 164, 164, 166, 127, 192, 89,
147, 48, 34, 81, 10, 164, 208, 50, 72, 209, 43, 80, 20, 36, 67, 54,
233, 130, 46, 105, 3, 12, 161, 66, 148, 65, 23, 202, 80, 187, 160,
135, 114, 65, 133, 84, 64, 42, 160, 250, 228, 66, 50, 32, 193, 22,
100, 66, 18, 246, 158, 119, 241, 248, 190, 191, 0, 9, 138, 237, 115,
126, 67, 94, 188, 143, 223, 123, 247, 221, 119, 222, 61, 247, 156,
123, 223, 125, 22, 139, 190, 248, 11, 14, 166, 152, 246, 78, 182, 142,
21, 211, 74, 219, 203, 207, 198, 138, 0, 87, 206, 153, 117, 102, 29,
76, 99, 38, 181, 28, 78, 133, 83, 221, 73, 123, 208, 193, 192, 30,
174, 92, 235, 152, 165, 23, 32, 61, 130, 43, 231, 202, 253, 69, 166,
184, 12, 248, 215, 20, 176, 124, 139, 20, 13, 113, 216, 223, 51, 224,
202, 193, 94, 54, 15, 229, 91, 3, 122, 185, 141, 21, 27, 51, 225, 197,
182, 4, 199, 72, 12, 251, 11, 141, 25, 250, 219, 210, 219, 210, 111,
224, 130, 201, 21, 39, 134, 225, 220, 78, 198, 51, 64, 206, 57, 210,
152, 225, 242, 223, 54, 175, 197, 75, 23, 225, 10, 222, 88, 180, 7,
45, 189, 54, 79, 235, 152, 119, 210, 30, 248, 20, 97, 117, 133, 156,
89, 239, 100, 67, 188, 152, 142, 21, 245, 114, 15, 72, 45, 91, 3, 206,
44, 252, 10, 133, 225, 172, 77, 137, 88, 209, 95, 112, 229, 96, 141,
145, 252, 115, 199, 130, 92, 219, 60, 169, 148, 191, 192, 29, 219, 40,
23, 238, 105, 114, 6, 206, 157, 139, 49, 254, 112, 74, 111, 127, 127,
193, 153, 189, 149, 14, 147, 251, 150, 139, 133, 194, 222, 73, 235,
72, 167, 215, 230, 177, 121, 154, 226, 164, 196, 122, 115, 51, 225,
69, 216, 6, 247, 82, 75, 118, 102, 83, 169, 158, 100, 83, 188, 33, 30,
244, 193, 185, 189, 147, 233, 18, 45, 1, 10, 114, 61, 186, 146, 46,
158, 46, 238, 10, 89, 122, 59, 189, 192, 8, 167, 154, 18, 65, 31, 69,
98, 216, 183, 168, 196, 136, 21, 29, 12, 104, 172, 47, 217, 148, 128,
220, 183, 6, 233, 117, 231, 98, 80, 122, 68, 119, 76, 137, 191, 0,
121, 63, 233, 131, 210, 246, 78, 130, 198, 57, 25, 51, 108, 162, 225,
172, 214, 23, 211, 77, 241, 207, 6, 32, 39, 140, 223, 40, 59, 86, 4,
70, 167, 55, 188, 200, 105, 63, 57, 191, 103, 97, 198, 40, 255, 0,
155, 95, 82, 26, 41, 72, 91, 131, 13, 241, 198, 140, 113, 157, 217,
18, 135, 242, 162, 79, 56, 232, 170, 81, 30, 149, 135, 51, 244, 126,
153, 99, 241, 226, 202, 37, 124, 237, 154, 79, 39, 10, 10, 138, 88,
160, 158, 32, 245, 85, 245, 194, 218, 232, 123, 193, 111, 204, 64, 29,
221, 148, 168, 150, 79, 106, 14, 242, 155, 88, 139, 123, 192, 247, 45,
118, 133, 186, 66, 14, 35, 245, 182, 34, 159, 218, 44, 67, 117, 153,
46, 191, 59, 217, 16, 215, 40, 21, 93, 62, 180, 96, 8, 126, 67, 249,
208, 90, 116, 102, 225, 23, 180, 219, 26, 51, 180, 165, 2, 45, 54,
206, 242, 56, 179, 141, 25, 67, 119, 226, 62, 72, 106, 185, 117, 204,
172, 29, 19, 10, 148, 137, 246, 17, 154, 18, 13, 113, 117, 116, 133,
44, 108, 235, 81, 227, 8, 189, 70, 64, 142, 115, 159, 248, 53, 231,
95, 83, 244, 203, 79, 91, 106, 189, 127, 40, 40, 40, 198, 196, 149,
203, 197, 18, 195, 157, 222, 174, 16, 173, 195, 205, 73, 56, 69, 60,
243, 74, 109, 67, 188, 231, 138, 24, 245, 88, 137, 207, 33, 174, 177,
44, 96, 55, 236, 65, 26, 49, 80, 142, 90, 8, 197, 26, 16, 178, 193,
158, 185, 114, 212, 127, 164, 208, 182, 228, 36, 143, 162, 179, 131,
181, 166, 231, 230, 163, 14, 90, 158, 175, 148, 15, 54, 81, 90, 3, 19,
95, 84, 67, 196, 103, 115, 48, 224, 185, 138, 145, 24, 214, 226, 59,
179, 194, 171, 5, 143, 95, 202, 39, 219, 53, 197, 149, 19, 231, 193,
132, 5, 218, 148, 198, 140, 240, 204, 185, 152, 131, 113, 48, 77, 9,
104, 63, 216, 131, 234, 58, 229, 47, 192, 153, 61, 3, 77, 9, 216, 159,
191, 114, 198, 159, 139, 25, 241, 221, 169, 141, 82, 131, 61, 168, 23,
179, 16, 150, 28, 127, 102, 97, 201, 209, 246, 141, 154, 64, 164, 73,
122, 175, 73, 123, 71, 148, 7, 45, 190, 252, 94, 179, 237, 37, 209,
26, 173, 167, 200, 149, 227, 159, 60, 238, 108, 68, 131, 13, 243, 185,
150, 31, 119, 181, 80, 94, 149, 54, 95, 5, 250, 49, 187, 98, 154, 62,
129, 246, 96, 165, 172, 68, 103, 215, 127, 126, 65, 132, 117, 5, 175,
139, 45, 253, 213, 68, 20, 252, 133, 174, 80, 167, 55, 49, 156, 139,
233, 221, 125, 20, 148, 143, 139, 64, 140, 63, 49, 28, 244, 181, 142,
105, 215, 88, 74, 226, 47, 112, 17, 114, 90, 11, 152, 109, 3, 8, 217,
0, 229, 72, 173, 154, 200, 236, 127, 197, 98, 131, 253, 128, 24, 166,
30, 159, 70, 186, 121, 4, 217, 232, 45, 191, 86, 207, 131, 145, 214,
193, 197, 116, 186, 4, 177, 95, 30, 218, 145, 107, 177, 189, 239, 244,
166, 75, 228, 250, 123, 165, 199, 212, 58, 130, 240, 108, 140, 63, 86,
20, 215, 192, 0, 178, 135, 134, 208, 190, 2, 238, 252, 16, 179, 151,
66, 173, 5, 225, 47, 132, 83, 169, 101, 40, 175, 182, 121, 185, 45,
228, 161, 28, 37, 114, 48, 148, 147, 24, 110, 204, 164, 75, 177, 34,
237, 245, 161, 53, 119, 98, 88, 104, 153, 148, 173, 160, 176, 213,
225, 25, 176, 6, 248, 28, 180, 205, 199, 138, 233, 82, 106, 153, 150,
4, 68, 181, 149, 248, 242, 114, 18, 106, 129, 111, 17, 242, 212, 157,
108, 155, 87, 179, 3, 180, 159, 66, 140, 134, 56, 159, 111, 239, 164,
182, 230, 200, 91, 31, 158, 129, 88, 81, 216, 254, 74, 106, 70, 239,
83, 203, 82, 126, 119, 50, 93, 18, 223, 7, 237, 232, 137, 184, 221, 6,
254, 178, 244, 105, 34, 247, 68, 67, 104, 143, 5, 127, 181, 177, 162,
180, 69, 163, 221, 118, 5, 221, 75, 45, 39, 103, 184, 214, 154, 176,
53, 97, 68, 243, 165, 210, 157, 20, 179, 181, 91, 79, 114, 17, 183,
135, 180, 90, 158, 106, 210, 152, 1, 61, 12, 250, 186, 66, 228, 238,
160, 160, 124, 68, 36, 86, 12, 167, 186, 66, 137, 225, 196, 112, 235,
152, 121, 205, 21, 214, 3, 0, 179, 71, 144, 182, 0, 204, 61, 183, 82,
143, 199, 44, 95, 94, 143, 210, 252, 59, 179, 197, 180, 145, 214, 144,
244, 252, 80, 11, 59, 24, 174, 68, 140, 196, 68, 132, 117, 190, 53, 0,
150, 139, 239, 195, 133, 53, 122, 145, 144, 198, 12, 191, 127, 119,
210, 193, 8, 217, 0, 53, 63, 40, 181, 108, 15, 194, 200, 142, 116, 41,
156, 226, 56, 77, 9, 241, 189, 164, 215, 160, 196, 230, 106, 107, 155,
39, 156, 18, 94, 177, 28, 202, 237, 7, 161, 245, 177, 7, 187, 147, 82,
207, 147, 135, 114, 47, 149, 186, 253, 110, 136, 183, 205, 11, 115,
163, 172, 15, 74, 246, 155, 219, 223, 153, 77, 151, 120, 107, 162, 92,
126, 82, 173, 133, 59, 197, 181, 97, 90, 250, 83, 203, 80, 170, 214,
128, 53, 160, 86, 250, 82, 107, 101, 243, 56, 24, 190, 245, 5, 75,
202, 60, 78, 196, 109, 5, 122, 23, 132, 87, 173, 239, 123, 211, 248,
9, 128, 241, 67, 27, 76, 218, 242, 52, 166, 253, 225, 20, 205, 169,
218, 115, 100, 84, 228, 81, 36, 115, 190, 136, 84, 255, 140, 140, 24,
17, 138, 184, 245, 193, 248, 205, 91, 127, 223, 162, 53, 224, 25, 240,
12, 128, 54, 99, 223, 3, 202, 71, 73, 192, 255, 98, 252, 157, 94, 207,
64, 235, 152, 57, 15, 28, 196, 149, 19, 183, 252, 245, 218, 253, 82,
145, 62, 251, 54, 143, 185, 209, 110, 82, 139, 103, 233, 53, 243, 244,
146, 125, 101, 22, 192, 92, 254, 197, 182, 136, 218, 124, 24, 21, 169,
119, 21, 48, 178, 15, 82, 97, 20, 1, 70, 190, 249, 11, 201, 25, 234,
201, 181, 142, 169, 95, 9, 141, 82, 64, 77, 229, 47, 112, 126, 159,
205, 227, 47, 20, 211, 66, 47, 48, 49, 172, 156, 11, 190, 182, 7, 207,
175, 49, 67, 125, 159, 196, 176, 188, 253, 160, 60, 6, 65, 104, 255,
90, 199, 96, 4, 168, 252, 46, 104, 89, 34, 113, 153, 217, 60, 234,
246, 88, 57, 126, 160, 110, 255, 25, 191, 56, 39, 202, 237, 15, 121,
79, 7, 101, 129, 229, 117, 229, 132, 222, 185, 114, 251, 71, 218, 94,
105, 233, 119, 102, 233, 53, 180, 244, 131, 53, 229, 203, 71, 217, 14,
138, 237, 183, 103, 0, 206, 202, 229, 27, 162, 104, 180, 85, 109, 243,
168, 251, 224, 16, 171, 166, 163, 120, 26, 226, 254, 130, 48, 142,
195, 61, 129, 176, 86, 141, 205, 229, 130, 238, 33, 237, 197, 49, 59,
178, 71, 94, 154, 102, 158, 96, 242, 20, 200, 238, 133, 25, 190, 191,
32, 213, 61, 35, 189, 78, 66, 17, 199, 241, 170, 177, 254, 212, 131,
10, 250, 236, 65, 136, 63, 153, 101, 163, 160, 60, 88, 129, 104, 167,
53, 16, 244, 49, 254, 228, 140, 249, 81, 87, 210, 24, 128, 118, 236,
82, 46, 242, 190, 76, 115, 222, 131, 82, 95, 170, 25, 190, 220, 154,
24, 233, 61, 44, 166, 155, 18, 52, 159, 210, 22, 140, 94, 244, 23,
132, 187, 102, 40, 43, 161, 53, 97, 252, 70, 202, 95, 24, 119, 112,
102, 249, 22, 0, 188, 245, 0, 17, 121, 123, 208, 51, 16, 244, 37, 103,
212, 106, 50, 225, 232, 122, 136, 49, 112, 254, 27, 196, 244, 141,
180, 31, 196, 37, 158, 24, 78, 45, 83, 27, 144, 24, 150, 214, 197,
202, 22, 76, 43, 254, 46, 134, 81, 251, 173, 6, 229, 17, 12, 242, 158,
14, 122, 69, 65, 31, 119, 37, 28, 148, 227, 15, 98, 251, 221, 233, 13,
167, 184, 232, 121, 56, 21, 43, 10, 245, 65, 173, 255, 5, 90, 121, 65,
31, 244, 93, 52, 37, 224, 158, 113, 199, 107, 233, 135, 119, 42, 120,
190, 145, 40, 184, 51, 43, 124, 254, 160, 196, 57, 237, 50, 50, 46,
86, 218, 11, 70, 223, 59, 224, 181, 91, 91, 228, 158, 191, 222, 152,
17, 177, 200, 35, 63, 230, 90, 31, 242, 62, 64, 179, 99, 129, 107,
183, 254, 14, 38, 57, 195, 248, 161, 246, 68, 235, 143, 242, 209, 148,
88, 177, 41, 1, 49, 128, 196, 112, 40, 108, 190, 5, 16, 43, 10, 107,
32, 45, 191, 69, 89, 164, 241, 80, 53, 191, 83, 77, 228, 241, 88, 237,
145, 111, 32, 109, 243, 208, 231, 65, 223, 236, 147, 199, 175, 245,
248, 188, 5, 33, 121, 87, 56, 63, 172, 85, 23, 222, 219, 163, 30, 50,
223, 255, 77, 223, 32, 212, 107, 127, 8, 235, 44, 155, 7, 222, 191,
228, 143, 102, 100, 236, 146, 248, 122, 97, 228, 0, 109, 17, 192, 8,
8, 24, 77, 103, 13, 48, 126, 45, 95, 70, 234, 175, 67, 221, 71, 75,
77, 60, 254, 1, 142, 172, 196, 215, 30, 53, 40, 132, 242, 155, 151,
114, 125, 81, 131, 242, 91, 116, 98, 27, 13, 30, 35, 205, 63, 227,
151, 182, 37, 213, 172, 25, 180, 152, 91, 199, 160, 148, 96, 220, 30,
215, 251, 13, 35, 70, 196, 71, 214, 215, 67, 177, 255, 45, 140, 4, 0,
244, 218, 210, 225, 84, 235, 152, 116, 127, 94, 27, 65, 55, 180, 216,
242, 241, 38, 160, 141, 124, 52, 92, 239, 41, 84, 123, 110, 186, 147,
252, 179, 101, 142, 111, 46, 250, 86, 107, 189, 1, 207, 177, 176, 244,
212, 244, 85, 75, 156, 217, 228, 12, 140, 128, 102, 252, 240, 190,
174, 89, 54, 10, 202, 111, 130, 128, 253, 135, 8, 64, 181, 253, 87,
194, 152, 41, 227, 55, 98, 131, 252, 5, 232, 247, 165, 245, 147, 180,
21, 31, 244, 233, 181, 194, 93, 57, 106, 131, 232, 155, 199, 242, 8,
130, 182, 253, 21, 142, 86, 130, 104, 161, 124, 60, 150, 118, 11, 70,
232, 63, 130, 133, 17, 91, 61, 253, 30, 92, 113, 107, 201, 95, 16, 30,
47, 49, 108, 15, 234, 121, 125, 98, 255, 217, 59, 201, 197, 80, 187,
66, 198, 70, 46, 75, 253, 103, 90, 26, 16, 61, 232, 78, 38, 103, 66,
97, 189, 209, 147, 194, 246, 138, 248, 186, 133, 185, 82, 215, 35,
113, 188, 90, 13, 218, 246, 163, 152, 78, 206, 116, 133, 188, 147, 48,
74, 143, 234, 130, 180, 85, 162, 223, 11, 14, 194, 197, 251, 225, 62,
136, 115, 165, 231, 137, 22, 211, 208, 110, 16, 142, 191, 16, 199,
178, 181, 207, 47, 239, 107, 1, 207, 85, 216, 254, 208, 190, 126, 249,
61, 128, 88, 3, 223, 146, 80, 139, 221, 112, 34, 247, 188, 33, 106,
198, 249, 243, 116, 6, 14, 45, 241, 23, 164, 81, 28, 170, 51, 240, 62,
7, 196, 52, 180, 217, 32, 92, 159, 9, 167, 187, 250, 12, 169, 64, 172,
139, 122, 45, 13, 113, 180, 189, 40, 40, 181, 8, 140, 97, 9, 250, 172,
1, 99, 81, 172, 88, 49, 23, 19, 206, 174, 34, 172, 143, 18, 195, 122,
181, 135, 112, 212, 0, 88, 59, 97, 204, 147, 214, 229, 90, 53, 136,
56, 254, 10, 245, 166, 188, 5, 160, 229, 57, 137, 253, 6, 216, 83,
238, 137, 104, 213, 254, 98, 31, 15, 248, 242, 62, 8, 173, 249, 21,
196, 150, 10, 124, 38, 249, 249, 181, 90, 48, 210, 189, 187, 147, 92,
11, 6, 162, 254, 96, 193, 181, 99, 183, 106, 254, 179, 190, 199, 198,
137, 188, 188, 105, 29, 110, 13, 36, 134, 173, 1, 35, 189, 72, 124,
212, 84, 185, 45, 96, 188, 23, 73, 62, 118, 220, 200, 213, 52, 102,
96, 244, 57, 255, 238, 161, 61, 40, 245, 134, 181, 102, 224, 145, 159,
19, 246, 22, 183, 171, 180, 52, 72, 222, 131, 1, 250, 34, 141, 69,
171, 243, 229, 35, 38, 96, 111, 113, 155, 64, 203, 23, 150, 223, 63,
233, 83, 4, 177, 32, 117, 126, 172, 40, 205, 1, 88, 127, 190, 29, 239,
25, 208, 27, 193, 39, 30, 55, 196, 69, 237, 125, 139, 137, 97, 207,
64, 40, 108, 164, 253, 64, 173, 191, 103, 0, 227, 238, 40, 40, 230,
197, 153, 13, 133, 161, 159, 152, 175, 39, 225, 93, 128, 182, 121,
120, 175, 75, 159, 237, 19, 204, 54, 70, 219, 0, 14, 134, 171, 19,
160, 71, 84, 155, 45, 174, 63, 97, 188, 174, 244, 237, 53, 173, 145,
136, 177, 162, 248, 93, 39, 240, 124, 164, 254, 144, 205, 163, 222, 6,
145, 198, 27, 192, 214, 202, 107, 84, 245, 158, 68, 169, 253, 12, 167,
228, 227, 8, 181, 90, 0, 210, 94, 211, 92, 76, 218, 254, 1, 168, 199,
142, 165, 111, 91, 117, 122, 149, 70, 208, 107, 149, 160, 51, 43, 111,
177, 64, 173, 221, 16, 135, 185, 24, 67, 225, 174, 144, 222, 29, 132,
218, 87, 122, 140, 182, 121, 136, 95, 107, 243, 164, 71, 113, 229,
164, 165, 9, 185, 208, 211, 64, 152, 101, 178, 43, 212, 54, 15, 241,
10, 165, 247, 15, 97, 76, 157, 58, 91, 233, 77, 199, 238, 164, 88,
255, 180, 236, 159, 210, 120, 69, 103, 86, 106, 83, 213, 219, 96, 74,
124, 127, 65, 218, 254, 80, 191, 127, 210, 62, 123, 170, 43, 210, 114,
84, 143, 225, 72, 223, 254, 167, 109, 5, 113, 20, 12, 222, 72, 84, 47,
65, 254, 173, 17, 122, 118, 216, 87, 216, 142, 227, 102, 4, 84, 151,
98, 154, 43, 5, 198, 79, 219, 10, 124, 157, 0, 239, 80, 104, 179, 65,
96, 228, 170, 111, 81, 56, 187, 166, 111, 17, 122, 112, 240, 253, 125,
20, 148, 106, 69, 105, 86, 29, 53, 113, 48, 93, 33, 168, 57, 194, 169,
78, 111, 114, 134, 62, 117, 198, 249, 52, 90, 106, 243, 248, 22, 105,
77, 64, 235, 59, 51, 231, 23, 215, 216, 80, 227, 195, 8, 120, 35, 124,
240, 254, 186, 147, 226, 90, 48, 232, 131, 119, 32, 100, 35, 154, 20,
133, 143, 248, 195, 251, 190, 225, 20, 239, 207, 201, 234, 102, 69,
225, 108, 22, 140, 248, 7, 112, 254, 84, 75, 191, 204, 51, 83, 20,
110, 204, 23, 140, 54, 167, 160, 237, 31, 152, 179, 217, 88, 249, 81,
27, 8, 111, 0, 10, 249, 161, 176, 204, 179, 85, 20, 174, 181, 37, 205,
63, 235, 245, 26, 224, 243, 35, 16, 189, 147, 226, 242, 19, 142, 77,
212, 186, 126, 155, 71, 60, 83, 0, 88, 161, 182, 121, 54, 242, 97,
232, 250, 193, 195, 20, 183, 154, 232, 88, 61, 51, 250, 71, 243, 218,
58, 70, 75, 131, 182, 247, 100, 179, 56, 105, 72, 99, 6, 102, 156,
129, 231, 166, 43, 148, 139, 113, 62, 175, 48, 186, 99, 122, 102, 75,
81, 14, 96, 52, 161, 121, 62, 175, 221, 166, 231, 229, 172, 8, 29,
195, 160, 29, 65, 208, 22, 40, 209, 234, 103, 5, 134, 182, 129, 246,
204, 23, 40, 40, 40, 74, 130, 246, 223, 8, 31, 237, 191, 114, 254, 31,
188, 253, 55, 122, 253, 104, 255, 213, 5, 237, 63, 10, 202, 39, 83,
208, 254, 27, 225, 163, 253, 87, 206, 63, 218, 127, 180, 255, 104,
255, 81, 80, 126, 243, 197, 248, 91, 88, 202, 35, 153, 225, 157, 151,
214, 49, 14, 48, 130, 138, 251, 13, 71, 230, 183, 48, 126, 165, 239,
239, 65, 143, 57, 111, 189, 96, 54, 124, 87, 142, 91, 130, 109, 254,
2, 183, 4, 125, 227, 242, 30, 61, 176, 211, 48, 215, 43, 5, 180, 37,
248, 165, 116, 9, 250, 99, 249, 99, 195, 12, 105, 82, 62, 124, 33,
142, 223, 31, 70, 99, 243, 75, 233, 18, 124, 1, 142, 95, 130, 241,
197, 82, 62, 244, 150, 194, 183, 238, 56, 104, 47, 201, 71, 55, 195,
119, 143, 236, 65, 14, 240, 230, 60, 191, 100, 15, 130, 117, 227, 151,
96, 95, 49, 27, 174, 9, 230, 153, 227, 0, 111, 92, 9, 243, 207, 248,
97, 46, 66, 14, 180, 172, 132, 124, 152, 65, 128, 47, 239, 116, 9,
222, 254, 16, 242, 225, 189, 116, 126, 9, 238, 135, 184, 63, 189, 59,
9, 243, 21, 243, 200, 197, 90, 199, 132, 203, 96, 19, 133, 203, 157,
94, 241, 120, 198, 80, 216, 30, 20, 110, 135, 175, 7, 10, 151, 189,
147, 201, 25, 225, 178, 61, 40, 30, 15, 8, 54, 178, 165, 159, 7, 148,
178, 214, 178, 84, 135, 97, 158, 97, 152, 161, 143, 67, 75, 63, 188,
139, 207, 195, 51, 208, 210, 47, 92, 134, 55, 188, 120, 54, 104, 155,
131, 17, 151, 87, 219, 188, 112, 185, 41, 33, 190, 62, 40, 111, 222,
22, 67, 239, 185, 80, 91, 65, 195, 96, 214, 97, 225, 253, 100, 252,
194, 101, 120, 62, 248, 62, 241, 134, 184, 248, 110, 167, 75, 158, 1,
152, 3, 144, 71, 56, 37, 212, 110, 128, 240, 43, 91, 240, 141, 74,
241, 214, 150, 126, 190, 37, 72, 53, 70, 124, 127, 211, 37, 248, 54,
22, 175, 187, 66, 237, 0, 72, 203, 3, 230, 165, 20, 239, 1, 121, 222,
44, 192, 94, 97, 205, 33, 173, 61, 228, 53, 8, 173, 69, 200, 115, 84,
17, 181, 217, 166, 180, 193, 243, 81, 80, 62, 233, 146, 15, 185, 178,
48, 211, 152, 16, 174, 108, 222, 224, 91, 56, 205, 209, 116, 105, 215,
136, 35, 35, 253, 115, 25, 154, 183, 51, 79, 106, 159, 114, 121, 215,
200, 224, 188, 244, 207, 152, 191, 226, 202, 150, 203, 192, 143, 36,
165, 127, 198, 248, 254, 194, 174, 145, 178, 162, 212, 139, 175, 204,
70, 62, 242, 141, 240, 169, 254, 43, 137, 193, 231, 143, 60, 251, 35,
5, 57, 140, 63, 255, 40, 40, 40, 31, 85, 217, 53, 178, 48, 150, 15,
25, 195, 194, 216, 174, 17, 41, 127, 105, 81, 173, 254, 81, 146, 37,
217, 56, 230, 116, 209, 12, 63, 45, 139, 34, 214, 122, 254, 90, 175,
191, 86, 193, 242, 199, 242, 255, 36, 151, 63, 10, 10, 202, 131, 149,
212, 134, 245, 142, 237, 182, 197, 20, 152, 219, 169, 155, 148, 221,
182, 193, 220, 182, 223, 241, 220, 110, 49, 1, 246, 24, 87, 27, 254,
29, 206, 205, 220, 110, 187, 27, 186, 19, 36, 107, 59, 13, 243, 217,
220, 174, 91, 222, 74, 253, 155, 245, 67, 251, 29, 96, 195, 26, 83,
215, 112, 203, 178, 102, 121, 131, 249, 71, 219, 109, 207, 109, 150,
125, 75, 25, 80, 107, 42, 110, 201, 91, 222, 177, 188, 100, 235, 183,
112, 249, 145, 237, 97, 187, 213, 185, 201, 239, 188, 213, 34, 221,
126, 213, 242, 182, 5, 222, 153, 99, 175, 91, 137, 79, 153, 194, 255,
18, 254, 255, 229, 248, 45, 192, 126, 95, 10, 150, 35, 250, 47, 66,
173, 252, 43, 18, 126, 158, 71, 103, 190, 97, 195, 146, 103, 57, 162,
255, 34, 92, 177, 252, 66, 141, 47, 103, 234, 242, 225, 122, 54, 193,
238, 173, 240, 95, 132, 117, 194, 247, 213, 196, 127, 75, 159, 31,
222, 128, 255, 221, 55, 21, 248, 107, 34, 62, 92, 79, 5, 157, 87, 96,
239, 226, 6, 111, 181, 93, 236, 127, 126, 15, 22, 170, 252, 196, 21,
101, 219, 79, 183, 218, 8, 20, 248, 112, 61, 21, 88, 223, 85, 225,
147, 109, 204, 85, 238, 151, 229, 162, 229, 140, 50, 191, 85, 149,
223, 178, 206, 31, 73, 194, 135, 252, 84, 208, 122, 89, 153, 223, 122,
205, 199, 150, 35, 169, 119, 96, 191, 11, 102, 249, 254, 74, 106, 125,
183, 194, 255, 103, 115, 124, 42, 206, 82, 101, 63, 49, 31, 174, 167,
2, 235, 37, 45, 126, 231, 90, 101, 191, 119, 4, 252, 91, 66, 126, 98,
77, 157, 29, 122, 119, 115, 63, 224, 15, 108, 242, 33, 63, 21, 116,
94, 84, 99, 251, 75, 252, 94, 164, 246, 249, 185, 10, 255, 66, 90,
133, 239, 185, 160, 194, 127, 159, 205, 207, 38, 66, 138, 37, 152,
123, 87, 184, 15, 169, 61, 222, 80, 227, 91, 222, 73, 221, 144, 178,
137, 173, 120, 71, 145, 79, 234, 34, 219, 58, 155, 31, 17, 186, 46,
197, 54, 185, 190, 247, 58, 223, 145, 110, 39, 218, 255, 143, 150,
126, 91, 63, 3, 92, 56, 218, 47, 76, 225, 140, 229, 159, 224, 238, 89,
3, 169, 247, 200, 145, 206, 144, 123, 241, 115, 146, 31, 99, 248, 39,
130, 87, 216, 123, 207, 206, 119, 211, 176, 76, 150, 142, 145, 229, 1,
195, 240, 90, 216, 209, 78, 77, 149, 183, 245, 83, 191, 102, 94, 177,
245, 115, 176, 104, 246, 136, 113, 123, 89, 3, 252, 91, 185, 131, 75,
79, 143, 110, 125, 206, 28, 158, 30, 29, 90, 161, 236, 231, 207, 126,
119, 244, 123, 39, 158, 120, 225, 241, 189, 198, 1, 71, 56, 230, 206,
199, 225, 220, 255, 227, 200, 212, 185, 3, 211, 79, 188, 208, 242, 83,
99, 224, 248, 29, 238, 237, 125, 127, 180, 242, 244, 232, 247, 95, 6,
182, 217, 252, 175, 185, 183, 186, 143, 245, 50, 254, 173, 207, 61,
241, 2, 176, 45, 10, 120, 234, 200, 237, 194, 237, 194, 83, 71, 148,
182, 29, 115, 175, 245, 109, 237, 181, 121, 182, 62, 247, 248, 222,
150, 159, 42, 243, 223, 56, 15, 218, 247, 198, 121, 101, 254, 177, 62,
184, 31, 90, 252, 200, 10, 240, 35, 43, 74, 219, 122, 37, 252, 53, 55,
135, 75, 123, 126, 48, 251, 195, 185, 75, 123, 214, 220, 28, 31, 214,
253, 112, 238, 7, 179, 176, 142, 67, 175, 187, 87, 192, 183, 60, 119,
198, 77, 113, 105, 207, 207, 51, 192, 250, 209, 217, 51, 155, 252, 51,
238, 31, 157, 133, 95, 191, 100, 46, 237, 225, 246, 83, 227, 79, 175,
210, 103, 238, 133, 5, 33, 255, 133, 5, 186, 118, 122, 149, 219, 175,
195, 221, 161, 192, 255, 235, 83, 116, 191, 229, 108, 230, 224, 25,
247, 16, 203, 31, 34, 252, 204, 193, 229, 74, 156, 251, 175, 79, 41,
243, 73, 121, 186, 223, 220, 115, 135, 245, 166, 87, 114, 241, 253,
176, 204, 241, 225, 119, 124, 255, 74, 14, 150, 238, 20, 223, 220, 3,
203, 219, 69, 252, 53, 55, 172, 251, 143, 211, 244, 28, 204, 209, 99,
110, 41, 255, 152, 155, 57, 74, 183, 254, 249, 235, 148, 191, 93, 196,
39, 229, 81, 217, 127, 134, 233, 117, 247, 10, 150, 135, 86, 184, 229,
25, 6, 150, 7, 151, 224, 183, 144, 255, 248, 94, 182, 60, 221, 52,
135, 127, 51, 199, 237, 223, 205, 150, 90, 247, 2, 183, 252, 55, 115,
244, 234, 148, 248, 164, 60, 220, 119, 75, 176, 253, 247, 94, 239,
112, 83, 156, 56, 120, 120, 245, 240, 234, 137, 131, 220, 242, 239,
189, 14, 219, 239, 150, 224, 247, 86, 247, 86, 1, 255, 24, 187, 157,
150, 222, 239, 79, 115, 251, 75, 241, 251, 108, 249, 252, 170, 8, 191,
45, 18, 62, 201, 143, 59, 202, 222, 163, 182, 115, 219, 221, 202, 104,
59, 7, 219, 163, 89, 248, 173, 196, 111, 103, 175, 247, 98, 65, 141,
127, 145, 237, 43, 105, 95, 144, 243, 217, 242, 112, 191, 114, 156,
222, 161, 63, 56, 165, 196, 254, 131, 138, 110, 189, 114, 28, 150,
214, 250, 44, 34, 254, 86, 22, 209, 138, 150, 237, 61, 186, 213, 45,
198, 222, 163, 180, 116, 163, 89, 186, 44, 228, 111, 127, 174, 163,
178, 87, 127, 37, 7, 119, 75, 215, 34, 66, 246, 181, 8, 101, 151, 203,
253, 199, 229, 252, 173, 207, 193, 245, 80, 252, 225, 44, 103, 181,
46, 22, 154, 207, 93, 139, 92, 139, 52, 159, 187, 184, 217, 71, 244,
135, 179, 220, 126, 103, 250, 214, 122, 149, 248, 22, 119, 195, 92,
89, 69, 26, 230, 248, 189, 164, 252, 181, 62, 30, 251, 78, 158, 202,
73, 185, 167, 114, 251, 78, 10, 247, 57, 38, 226, 111, 21, 241, 1,
251, 199, 255, 118, 137, 30, 229, 84, 238, 111, 151, 246, 143, 75,
183, 247, 246, 157, 17, 241, 207, 244, 153, 3, 199, 167, 117, 217,
154, 73, 246, 153, 190, 237, 125, 29, 236, 56, 144, 189, 71, 161, 44,
122, 89, 116, 84, 82, 109, 192, 94, 91, 251, 142, 85, 70, 195, 253,
151, 149, 237, 100, 201, 66, 74, 227, 152, 9, 116, 84, 172, 49, 216,
224, 31, 198, 185, 37, 179, 224, 70, 241, 192, 151, 74, 132, 35, 204,
52, 237, 127, 101, 47, 161, 253, 71, 255, 95, 217, 255, 79, 220, 246,
19, 175, 33, 161, 28, 27, 208, 245, 255, 45, 183, 124, 119, 65, 255,
125, 119, 21, 249, 186, 254, 191, 229, 86, 138, 229, 167, 212, 248,
42, 254, 187, 237, 253, 134, 141, 134, 13, 27, 249, 181, 201, 23, 172,
211, 243, 255, 109, 239, 59, 216, 231, 189, 105, 67, 200, 111, 98,
189, 81, 71, 217, 166, 235, 255, 23, 239, 210, 250, 162, 123, 195,
146, 79, 177, 172, 20, 249, 213, 93, 241, 102, 139, 119, 181, 253,
255, 228, 7, 116, 63, 103, 185, 51, 47, 228, 119, 230, 157, 149, 122,
40, 249, 129, 186, 255, 223, 146, 167, 109, 126, 87, 185, 37, 15, 203,
155, 124, 118, 27, 245, 162, 99, 149, 109, 74, 254, 127, 168, 114,
118, 235, 117, 234, 163, 11, 249, 150, 171, 214, 235, 116, 171, 247,
3, 53, 255, 159, 238, 239, 40, 113, 62, 190, 152, 111, 185, 234, 40,
9, 150, 21, 252, 119, 154, 195, 134, 155, 156, 39, 79, 227, 6, 221,
155, 203, 13, 55, 233, 213, 169, 249, 255, 212, 111, 11, 93, 231, 246,
247, 92, 41, 110, 20, 55, 60, 155, 145, 129, 16, 123, 5, 105, 37, 62,
235, 191, 199, 56, 254, 186, 50, 40, 63, 166, 234, 255, 59, 217, 235,
107, 187, 169, 198, 111, 99, 243, 79, 60, 104, 21, 255, 159, 94, 175,
191, 76, 252, 185, 53, 57, 108, 235, 212, 131, 239, 86, 245, 255, 237,
149, 200, 67, 242, 154, 18, 63, 121, 173, 114, 119, 53, 252, 127, 103,
197, 74, 51, 178, 28, 48, 235, 180, 116, 181, 253, 127, 107, 197, 243,
77, 67, 30, 46, 242, 72, 94, 227, 124, 106, 235, 101, 109, 255, 63,
119, 141, 179, 184, 254, 82, 219, 141, 208, 187, 161, 119, 219, 110,
248, 75, 220, 186, 220, 53, 125, 255, 191, 233, 189, 178, 138, 52,
189, 103, 196, 255, 183, 92, 176, 95, 118, 149, 164, 92, 87, 201, 126,
217, 98, 208, 255, 39, 184, 208, 122, 57, 124, 131, 30, 197, 85, 10,
223, 104, 189, 204, 122, 217, 134, 253, 127, 3, 208, 241, 255, 117,
129, 254, 63, 250, 255, 58, 254, 191, 229, 185, 240, 82, 120, 73, 121,
139, 186, 255, 255, 133, 159, 13, 204, 125, 106, 47, 253, 253, 75,
230, 151, 12, 253, 245, 169, 189, 3, 115, 95, 248, 153, 17, 255, 255,
107, 39, 202, 229, 231, 207, 74, 249, 207, 19, 15, 252, 107, 39, 140,
248, 255, 212, 239, 255, 204, 97, 248, 245, 131, 217, 31, 204, 66,
250, 153, 195, 52, 14, 160, 239, 255, 3, 126, 235, 133, 229, 236, 159,
141, 10, 215, 252, 217, 232, 114, 246, 183, 94, 16, 174, 145, 243,
127, 103, 116, 112, 233, 183, 143, 112, 219, 127, 189, 255, 175, 166,
135, 86, 150, 179, 191, 100, 124, 231, 255, 195, 107, 60, 239, 183,
143, 248, 206, 255, 206, 168, 146, 255, 111, 61, 1, 254, 167, 239,
252, 155, 123, 32, 14, 0, 254, 210, 157, 226, 12, 185, 126, 248, 117,
177, 240, 217, 151, 207, 16, 255, 222, 119, 30, 214, 90, 79, 40, 251,
255, 241, 253, 63, 58, 11, 190, 63, 68, 43, 102, 24, 235, 9, 234, 183,
191, 185, 231, 207, 95, 7, 15, 234, 115, 175, 66, 12, 224, 71, 103,
105, 108, 64, 201, 255, 167, 184, 114, 156, 247, 249, 223, 58, 196,
28, 13, 236, 57, 230, 158, 59, 120, 177, 112, 167, 120, 124, 63, 191,
215, 118, 5, 255, 159, 162, 135, 228, 113, 238, 32, 253, 205, 28, 253,
251, 115, 209, 236, 149, 227, 189, 238, 223, 125, 13, 34, 7, 252, 94,
219, 37, 254, 251, 231, 94, 253, 242, 169, 47, 159, 250, 220, 171,
189, 238, 159, 103, 126, 85, 236, 21, 28, 239, 196, 65, 136, 32, 252,
203, 16, 141, 12, 112, 251, 73, 249, 212, 71, 188, 88, 232, 112, 15,
173, 220, 45, 245, 237, 17, 250, 237, 189, 228, 239, 23, 228, 170,
190, 124, 170, 99, 115, 63, 185, 255, 223, 183, 135, 178, 192, 203,
255, 79, 179, 82, 223, 255, 240, 106, 185, 28, 28, 234, 216, 220, 79,
201, 127, 167, 232, 96, 227, 28, 87, 167, 182, 11, 214, 129, 103, 47,
142, 11, 40, 243, 255, 110, 207, 203, 71, 183, 187, 191, 205, 198,
106, 224, 215, 246, 77, 207, 125, 247, 18, 252, 122, 249, 232, 223,
237, 81, 226, 179, 229, 233, 190, 58, 5, 87, 215, 121, 112, 187, 251,
39, 67, 160, 1, 116, 207, 206, 131, 119, 75, 167, 114, 143, 239, 129,
95, 112, 229, 52, 103, 114, 255, 255, 90, 4, 56, 175, 84, 252, 243,
230, 115, 92, 20, 0, 60, 242, 249, 113, 186, 246, 149, 227, 112, 109,
224, 219, 203, 253, 255, 150, 253, 28, 23, 240, 94, 180, 92, 126, 145,
217, 189, 180, 123, 9, 174, 101, 207, 255, 225, 183, 188, 114, 188,
101, 191, 186, 255, 207, 33, 42, 26, 215, 217, 124, 78, 186, 93, 221,
255, 167, 248, 135, 85, 33, 159, 143, 27, 24, 241, 255, 1, 98, 126,
238, 180, 116, 187, 158, 255, 255, 255, 34, 66, 254, 147, 67, 102,
253, 255, 63, 61, 196, 229, 224, 110, 233, 63, 159, 191, 244, 147,
106, 252, 255, 100, 52, 93, 250, 231, 194, 247, 142, 163, 255, 47,
176, 255, 232, 255, 155, 234, 255, 87, 133, 1, 255, 95, 19, 6, 252,
127, 29, 190, 78, 255, 189, 14, 106, 229, 107, 244, 255, 3, 90, 242,
201, 15, 83, 27, 206, 178, 163, 236, 219, 232, 250, 192, 150, 151,
110, 87, 239, 255, 183, 177, 113, 0, 240, 151, 98, 132, 237, 40, 195,
47, 127, 217, 254, 1, 221, 162, 223, 255, 223, 153, 135, 104, 133, 99,
211, 139, 183, 93, 245, 126, 0, 158, 99, 235, 7, 157, 121, 35, 253,
255, 224, 231, 167, 54, 108, 236, 111, 79, 158, 201, 195, 175, 206,
171, 126, 136, 28, 24, 234, 255, 247, 145, 179, 123, 42, 191, 153,
124, 219, 77, 39, 155, 147, 46, 114, 84, 239, 117, 35, 253, 255, 142,
82, 76, 212, 215, 239, 185, 90, 220, 176, 92, 9, 94, 37, 254, 219, 77,
193, 122, 85, 126, 106, 35, 93, 182, 137, 198, 10, 192, 146, 149, 248,
149, 201, 235, 170, 252, 171, 98, 47, 63, 39, 139, 1, 192, 120, 132,
224, 21, 193, 26, 213, 254, 127, 219, 58, 196, 57, 188, 215, 132, 108,
240, 236, 37, 113, 1, 213, 254, 127, 203, 90, 203, 58, 120, 142, 204,
230, 186, 28, 201, 81, 24, 216, 66, 143, 92, 181, 255, 31, 16, 92,
231, 61, 245, 78, 226, 185, 187, 74, 54, 105, 60, 64, 181, 255, 159,
162, 237, 6, 23, 5, 0, 143, 188, 245, 93, 89, 60, 65, 181, 255, 159,
243, 248, 33, 18, 20, 190, 25, 190, 9, 215, 146, 144, 109, 87, 239,
255, 167, 112, 138, 60, 240, 182, 27, 138, 124, 21, 255, 29, 80, 188,
41, 178, 223, 215, 164, 219, 181, 252, 127, 211, 124, 5, 255, 61, 36,
26, 5, 18, 148, 251, 247, 58, 254, 191, 103, 51, 7, 233, 178, 239,
134, 173, 42, 255, 63, 121, 57, 93, 246, 151, 236, 191, 66, 255, 95,
96, 255, 21, 252, 255, 116, 97, 117, 181, 188, 176, 122, 174, 124,
110, 245, 44, 1, 253, 175, 137, 209, 185, 175, 156, 117, 49, 28, 187,
156, 35, 88, 34, 124, 114, 12, 99, 248, 241, 220, 133, 169, 135, 78,
194, 104, 100, 150, 93, 46, 23, 86, 207, 127, 115, 238, 199, 115, 223,
52, 128, 111, 16, 124, 254, 212, 208, 196, 245, 224, 215, 95, 108, 92,
77, 175, 18, 126, 97, 100, 225, 66, 36, 50, 53, 61, 53, 100, 24, 59,
198, 223, 14, 88, 159, 111, 13, 146, 92, 47, 157, 38, 236, 158, 9,
115, 216, 54, 62, 49, 114, 210, 219, 226, 25, 157, 155, 61, 251, 151,
115, 228, 120, 19, 230, 208, 60, 126, 96, 196, 67, 238, 226, 81, 194,
255, 10, 240, 199, 205, 225, 161, 147, 148, 255, 227, 185, 163, 103,
63, 63, 187, 123, 162, 125, 220, 28, 30, 58, 185, 235, 197, 239, 0,
127, 246, 127, 179, 252, 109, 227, 20, 211, 113, 238, 153, 157, 142,
107, 173, 219, 82, 225, 127, 99, 246, 199, 115, 15, 207, 246, 108,
242, 133, 79, 189, 214, 58, 158, 255, 205, 185, 15, 163, 80, 30, 20,
34, 175, 71, 99, 221, 245, 224, 174, 23, 59, 9, 255, 47, 103, 191,
193, 242, 31, 26, 167, 16, 238, 171, 181, 46, 79, 180, 7, 248, 95, 57,
245, 141, 185, 91, 209, 246, 241, 106, 249, 127, 124, 234, 43, 179,
23, 162, 112, 61, 20, 194, 125, 181, 214, 229, 3, 148, 255, 249, 42,
249, 111, 3, 159, 60, 233, 27, 209, 207, 207, 78, 71, 160, 60, 40,
132, 251, 106, 173, 91, 8, 124, 125, 16, 248, 31, 70, 128, 15, 215,
67, 33, 220, 87, 107, 221, 66, 160, 181, 194, 255, 48, 26, 137, 108,
49, 205, 127, 45, 240, 37, 150, 127, 129, 240, 135, 34, 228, 122, 130,
20, 194, 125, 181, 214, 77, 108, 242, 111, 69, 7, 35, 112, 61, 20,
194, 125, 181, 214, 113, 252, 233, 169, 11, 209, 221, 83, 121, 243,
252, 145, 47, 13, 126, 149, 240, 35, 83, 75, 145, 106, 248, 7, 70,
236, 44, 127, 104, 106, 58, 210, 67, 248, 19, 1, 10, 225, 190, 90,
235, 14, 140, 124, 150, 229, 15, 78, 69, 34, 59, 166, 22, 54, 249,
237, 81, 110, 207, 246, 168, 214, 58, 142, 191, 123, 106, 40, 210, 62,
181, 16, 152, 48, 9, 47, 225, 183, 16, 126, 207, 196, 224, 212, 182,
9, 184, 30, 115, 216, 245, 162, 245, 121, 224, 239, 152, 216, 61, 213,
60, 49, 81, 5, 255, 202, 0, 240, 161, 46, 200, 7, 15, 152, 230, 183,
190, 152, 240, 218, 136, 55, 220, 28, 153, 96, 143, 246, 117, 83, 248,
210, 32, 51, 0, 218, 99, 13, 56, 86, 191, 244, 162, 125, 240, 179,
131, 214, 231, 175, 12, 24, 3, 67, 144, 240, 118, 122, 56, 251, 239,
155, 79, 120, 127, 210, 223, 233, 105, 49, 1, 27, 107, 203, 185, 175,
245, 20, 211, 246, 0, 157, 231, 68, 63, 2, 64, 247, 104, 65, 251, 143,
246, 95, 209, 254, 111, 27, 127, 116, 101, 103, 166, 39, 194, 47, 43,
67, 205, 254, 111, 27, 143, 65, 132, 185, 244, 240, 249, 109, 19, 198,
248, 98, 251, 223, 76, 249, 68, 98, 133, 72, 156, 95, 43, 133, 154,
253, 127, 104, 147, 15, 226, 98, 122, 162, 252, 22, 33, 212, 236, 191,
152, 15, 226, 88, 105, 158, 80, 231, 75, 237, 255, 150, 147, 82, 62,
188, 181, 183, 52, 255, 144, 96, 15, 45, 251, 175, 196, 7, 241, 103,
123, 162, 66, 190, 154, 253, 191, 30, 84, 230, 131, 60, 188, 194, 239,
165, 102, 255, 243, 154, 124, 126, 47, 53, 251, 175, 198, 247, 103,
219, 35, 252, 62, 234, 246, 255, 109, 5, 254, 233, 98, 56, 145, 15,
190, 45, 130, 154, 253, 95, 8, 72, 249, 183, 82, 249, 49, 126, 171,
158, 253, 23, 243, 119, 50, 205, 83, 114, 174, 150, 253, 231, 249,
177, 66, 207, 105, 101, 174, 150, 253, 159, 96, 249, 233, 210, 82,
114, 33, 168, 101, 127, 213, 236, 255, 68, 224, 86, 202, 177, 154, 15,
233, 217, 111, 180, 255, 104, 255, 209, 254, 127, 188, 237, 255, 96,
212, 159, 245, 103, 7, 163, 240, 251, 209, 149, 88, 46, 150, 115, 174,
234, 219, 127, 14, 145, 120, 154, 141, 65, 166, 75, 145, 211, 219,
198, 159, 97, 199, 174, 143, 228, 244, 237, 63, 197, 210, 60, 173, 3,
105, 109, 184, 52, 175, 206, 151, 218, 127, 192, 195, 75, 212, 102,
236, 152, 218, 49, 229, 103, 123, 66, 87, 75, 148, 175, 111, 255, 155,
199, 119, 178, 111, 48, 56, 87, 119, 68, 70, 114, 35, 185, 29, 17,
231, 42, 87, 159, 143, 228, 244, 236, 127, 123, 229, 124, 183, 22,
123, 162, 244, 221, 252, 116, 177, 39, 122, 107, 81, 141, 47, 182,
255, 59, 34, 244, 122, 35, 137, 72, 188, 204, 71, 112, 75, 145, 120,
36, 65, 203, 178, 61, 162, 110, 255, 7, 79, 67, 137, 179, 231, 75, 74,
109, 96, 49, 73, 243, 147, 46, 13, 158, 86, 182, 255, 244, 12, 177,
66, 123, 196, 177, 162, 100, 129, 29, 43, 237, 155, 185, 83, 178, 255,
143, 174, 194, 156, 65, 205, 83, 46, 70, 137, 13, 226, 34, 214, 16,
230, 27, 122, 116, 85, 201, 254, 95, 31, 91, 74, 230, 131, 234, 108,
122, 132, 124, 112, 41, 121, 125, 76, 221, 254, 251, 101, 35, 207,
133, 226, 207, 233, 217, 127, 61, 190, 158, 253, 55, 204, 87, 177,
255, 70, 249, 106, 246, 95, 143, 175, 103, 255, 205, 242, 165, 246,
127, 105, 49, 86, 84, 71, 42, 137, 246, 31, 237, 255, 39, 197, 254,
239, 158, 120, 38, 243, 76, 102, 119, 149, 246, 191, 103, 130, 218,
188, 103, 152, 158, 106, 236, 127, 133, 77, 143, 64, 24, 166, 236,
255, 142, 9, 113, 61, 236, 34, 71, 48, 234, 255, 3, 6, 35, 101, 137,
236, 150, 68, 4, 244, 236, 127, 79, 132, 180, 62, 114, 180, 206, 25,
140, 202, 227, 9, 218, 246, 159, 226, 25, 214, 18, 63, 147, 53, 238,
255, 55, 143, 11, 253, 125, 142, 207, 175, 233, 137, 54, 107, 216,
127, 218, 114, 232, 137, 168, 241, 123, 72, 169, 236, 204, 52, 171,
216, 255, 135, 42, 237, 14, 106, 159, 1, 220, 253, 231, 150, 7, 79,
195, 242, 206, 12, 196, 2, 228, 246, 159, 178, 161, 213, 179, 51, 195,
130, 225, 218, 31, 59, 25, 88, 218, 153, 241, 87, 198, 132, 237, 204,
200, 237, 63, 88, 111, 227, 242, 232, 170, 212, 254, 155, 227, 59, 86,
229, 246, 223, 89, 201, 255, 51, 92, 254, 51, 155, 249, 175, 44, 63,
83, 201, 191, 51, 163, 100, 255, 185, 35, 244, 68, 57, 27, 79, 117,
216, 197, 112, 203, 61, 81, 142, 173, 108, 255, 233, 17, 120, 143,
157, 227, 115, 203, 205, 83, 148, 173, 101, 255, 133, 254, 190, 148,
47, 220, 170, 238, 255, 107, 241, 141, 248, 255, 20, 91, 38, 182, 76,
113, 250, 183, 101, 106, 203, 132, 113, 255, 159, 178, 165, 119, 76,
122, 4, 61, 255, 159, 187, 155, 220, 29, 51, 235, 255, 11, 143, 0,
108, 243, 246, 159, 59, 194, 163, 10, 108, 131, 246, 159, 88, 103,
199, 170, 242, 54, 180, 255, 156, 253, 239, 78, 122, 39, 97, 70, 111,
41, 188, 147, 90, 95, 141, 231, 164, 59, 217, 148, 80, 171, 37, 154,
18, 250, 71, 240, 78, 106, 213, 51, 250, 95, 238, 232, 10, 105, 241,
229, 115, 237, 35, 31, 249, 247, 146, 95, 171, 254, 250, 22, 213, 159,
159, 134, 184, 145, 39, 208, 183, 88, 203, 243, 139, 130, 130, 242,
73, 150, 143, 163, 253, 247, 45, 182, 244, 59, 152, 106, 235, 127,
250, 221, 191, 150, 254, 234, 248, 240, 189, 19, 224, 135, 194, 198,
249, 240, 197, 23, 202, 134, 175, 15, 1, 155, 174, 55, 198, 135, 239,
12, 90, 217, 81, 66, 174, 28, 253, 14, 158, 189, 50, 102, 202, 24,
159, 114, 90, 199, 252, 5, 250, 197, 62, 248, 222, 140, 25, 62, 119,
197, 244, 63, 124, 235, 141, 43, 11, 163, 215, 207, 127, 105, 209, 51,
32, 156, 17, 223, 120, 249, 19, 77, 35, 108, 248, 130, 158, 240, 94,
152, 185, 127, 109, 243, 45, 253, 78, 201, 108, 250, 104, 255, 81, 80,
80, 62, 198, 34, 143, 17, 212, 118, 132, 218, 242, 80, 173, 212, 198,
166, 71, 168, 73, 164, 95, 229, 67, 62, 242, 145, 143, 252, 122, 241,
107, 174, 253, 106, 175, 125, 107, 175, 253, 107, 183, 62, 181, 90,
129, 79, 170, 244, 214, 200, 254, 168, 106, 15, 242, 145, 255, 209,
230, 127, 44, 90, 207, 40, 40, 40, 15, 74, 112, 254, 31, 156, 255, 71,
56, 127, 112, 106, 195, 89, 74, 136, 246, 78, 188, 239, 44, 165, 196,
243, 7, 171, 204, 255, 99, 203, 211, 47, 215, 196, 202, 228, 8, 149,
117, 137, 202, 188, 188, 197, 13, 155, 238, 252, 63, 225, 205, 239,
222, 144, 35, 136, 216, 32, 225, 13, 189, 249, 127, 28, 130, 121, 83,
224, 8, 65, 1, 91, 56, 55, 175, 218, 252, 63, 193, 171, 254, 178, 240,
8, 66, 182, 191, 28, 52, 48, 255, 143, 248, 8, 42, 108, 141, 249, 127,
96, 174, 31, 249, 17, 88, 182, 161, 239, 255, 176, 71, 184, 226, 151,
178, 197, 95, 15, 210, 152, 255, 7, 160, 200, 23, 207, 8, 164, 58,
255, 15, 203, 150, 205, 31, 235, 47, 73, 142, 160, 58, 255, 79, 112,
93, 206, 174, 28, 97, 93, 52, 255, 142, 34, 95, 204, 150, 220, 63,
225, 17, 84, 230, 255, 145, 232, 207, 122, 98, 93, 162, 63, 58, 243,
255, 132, 55, 191, 92, 19, 43, 37, 216, 111, 253, 144, 35, 108, 30,
51, 124, 195, 162, 51, 255, 143, 237, 34, 157, 247, 134, 156, 155,
251, 82, 208, 197, 196, 90, 229, 249, 185, 105, 51, 48, 255, 143, 237,
66, 234, 134, 179, 152, 16, 205, 232, 147, 88, 115, 22, 83, 55, 108,
38, 230, 255, 209, 133, 206, 252, 63, 134, 231, 239, 173, 149, 143,
243, 255, 214, 60, 255, 207, 244, 234, 174, 215, 159, 58, 242, 228,
225, 106, 96, 177, 248, 206, 239, 122, 253, 192, 244, 247, 95, 254,
238, 232, 119, 71, 159, 54, 141, 233, 213, 174, 215, 222, 56, 63, 48,
247, 63, 95, 133, 89, 132, 225, 24, 230, 208, 245, 218, 129, 105, 96,
67, 254, 171, 185, 134, 167, 142, 124, 255, 101, 96, 127, 241, 112,
117, 120, 242, 48, 228, 226, 201, 42, 217, 148, 255, 189, 19, 181,
240, 159, 102, 207, 255, 153, 42, 65, 249, 95, 124, 224, 252, 203, 67,
213, 129, 227, 175, 31, 170, 14, 247, 134, 255, 153, 195, 111, 29,
170, 14, 247, 138, 255, 210, 161, 234, 64, 249, 235, 85, 178, 239, 13,
255, 105, 194, 247, 84, 9, 202, 127, 235, 129, 242, 159, 58, 242, 210,
161, 239, 84, 137, 223, 12, 190, 231, 208, 19, 85, 2, 248, 79, 30,
246, 28, 250, 234, 193, 234, 80, 43, 31, 234, 144, 183, 72, 62, 190,
112, 176, 58, 204, 143, 127, 135, 112, 63, 125, 48, 115, 160, 58, 252,
247, 213, 185, 3, 199, 247, 185, 247, 125, 123, 95, 75, 85, 176, 88,
38, 19, 127, 178, 239, 77, 79, 159, 167, 163, 183, 26, 128, 253, 254,
163, 149, 175, 5, 30, 247, 24, 27, 255, 47, 106, 11, 144, 125, 141,
88, 200, 174, 215, 14, 175, 170, 197, 14, 244, 45, 60, 88, 229, 127,
25, 122, 232, 156, 50, 95, 223, 194, 3, 255, 165, 67, 115, 7, 254,
171, 98, 30, 140, 216, 248, 47, 18, 253, 106, 217, 247, 189, 128, 18,
223, 136, 141, 93, 39, 218, 121, 201, 243, 184, 167, 122, 254, 167,
15, 6, 60, 202, 49, 82, 35, 252, 183, 52, 248, 70, 108, 236, 91, 135,
50, 7, 30, 36, 255, 37, 13, 190, 17, 27, 15, 252, 62, 21, 190, 17, 27,
93, 43, 223, 67, 180, 79, 141, 111, 196, 70, 223, 79, 190, 17, 27, 91,
43, 255, 59, 135, 142, 239, 187, 95, 124, 35, 54, 182, 86, 254, 19,
53, 242, 191, 122, 240, 248, 190, 94, 149, 30, 18, 35, 54, 246, 171,
7, 221, 53, 241, 191, 112, 15, 248, 29, 42, 124, 35, 54, 94, 139, 111,
196, 70, 127, 250, 224, 183, 107, 226, 103, 14, 168, 243, 141, 216,
248, 185, 3, 127, 178, 15, 190, 213, 162, 196, 215, 183, 240, 115, 7,
90, 88, 237, 177, 42, 218, 15, 125, 11, 255, 169, 159, 244, 109, 190,
235, 39, 23, 125, 11, 191, 149, 181, 213, 255, 45, 161, 196, 182, 72,
251, 15, 101, 246, 93, 250, 174, 191, 146, 196, 138, 254, 130, 51,
235, 96, 170, 1, 176, 211, 165, 116, 201, 149, 115, 102, 171, 1, 176,
97, 86, 19, 103, 182, 152, 110, 204, 20, 211, 70, 193, 237, 235, 202,
193, 217, 29, 140, 111, 177, 109, 190, 26, 56, 24, 87, 206, 193, 116,
39, 27, 226, 213, 161, 49, 227, 96, 82, 203, 109, 243, 181, 241, 155,
18, 213, 242, 161, 36, 194, 169, 218, 249, 185, 88, 117, 128, 123, 16,
78, 53, 196, 171, 229, 167, 150, 107, 231, 251, 22, 107, 225, 167,
150, 129, 159, 156, 169, 14, 225, 20, 240, 115, 177, 90, 248, 221,
201, 234, 249, 190, 197, 112, 234, 94, 240, 67, 225, 234, 0, 252, 182,
249, 228, 76, 181, 252, 238, 36, 60, 185, 15, 150, 223, 148, 8, 133,
189, 147, 213, 161, 109, 30, 222, 128, 122, 112, 252, 166, 4, 212, 61,
181, 242, 149, 223, 32, 49, 130, 166, 4, 212, 61, 213, 243, 27, 226,
80, 247, 84, 207, 135, 103, 56, 20, 174, 150, 13, 231, 135, 255, 173,
99, 246, 32, 253, 51, 10, 110, 223, 198, 140, 61, 104, 13, 88, 3, 140,
191, 58, 192, 251, 59, 137, 97, 207, 64, 167, 183, 165, 191, 26, 128,
253, 46, 166, 237, 65, 126, 141, 49, 255, 159, 219, 87, 223, 194, 131,
165, 117, 102, 213, 218, 14, 218, 214, 29, 248, 197, 116, 119, 50, 57,
147, 90, 86, 230, 235, 91, 122, 176, 78, 201, 25, 123, 176, 49, 163,
196, 215, 183, 240, 160, 159, 161, 48, 227, 111, 29, 83, 226, 27, 177,
145, 160, 159, 65, 31, 45, 235, 234, 248, 93, 33, 207, 128, 114, 251,
211, 8, 63, 57, 211, 58, 86, 43, 191, 211, 171, 204, 55, 98, 35, 63,
234, 252, 80, 216, 30, 84, 227, 27, 177, 145, 148, 175, 236, 127, 124,
20, 248, 222, 73, 117, 190, 17, 27, 249, 224, 249, 214, 192, 253, 226,
27, 177, 145, 93, 161, 143, 47, 223, 136, 141, 108, 29, 99, 252, 106,
207, 143, 81, 62, 107, 151, 30, 16, 223, 30, 76, 12, 131, 31, 172,
196, 215, 179, 242, 96, 221, 19, 195, 80, 122, 246, 160, 18, 95, 219,
186, 39, 134, 19, 195, 65, 31, 148, 157, 205, 163, 236, 131, 235, 91,
120, 176, 231, 54, 15, 55, 215, 143, 76, 52, 188, 127, 238, 8, 246,
160, 186, 255, 15, 222, 115, 186, 68, 103, 119, 244, 23, 16, 136, 251,
3, 170, 97, 84, 219, 168, 198, 65, 204, 169, 152, 6, 255, 27, 129,
184, 31, 0, 191, 5, 52, 143, 211, 56, 87, 142, 248, 34, 243, 61, 167,
243, 99, 237, 17, 192, 219, 65, 154, 46, 4, 182, 77, 1, 48, 197, 180,
182, 180, 61, 210, 28, 105, 139, 67, 84, 26, 180, 14, 234, 56, 162,
113, 137, 221, 209, 29, 83, 249, 224, 142, 8, 0, 83, 76, 239, 121, 58,
213, 62, 181, 37, 90, 76, 83, 43, 235, 204, 146, 58, 46, 186, 35, 178,
59, 178, 229, 228, 238, 200, 110, 76, 49, 189, 47, 233, 142, 72, 251,
84, 83, 28, 122, 240, 98, 197, 98, 186, 39, 190, 99, 106, 119, 100,
40, 218, 60, 62, 20, 5, 96, 138, 233, 189, 79, 137, 214, 77, 109, 153,
42, 166, 193, 178, 166, 150, 161, 254, 3, 157, 219, 54, 49, 29, 5,
180, 99, 138, 233, 61, 78, 89, 157, 139, 92, 31, 75, 45, 131, 23, 139,
58, 135, 233, 253, 79, 81, 231, 48, 173, 119, 138, 58, 135, 105, 189,
83, 212, 57, 76, 235, 157, 162, 206, 97, 90, 239, 20, 117, 14, 211,
122, 167, 168, 115, 152, 214, 59, 69, 157, 195, 180, 222, 169, 84,
231, 218, 37, 58, 135, 41, 166, 247, 58, 21, 234, 28, 244, 125, 53,
71, 177, 239, 11, 211, 122, 244, 125, 229, 39, 161, 239, 11, 250, 248,
219, 18, 237, 83, 216, 199, 143, 233, 253, 238, 227, 223, 54, 209,
116, 154, 246, 241, 195, 88, 166, 45, 81, 162, 117, 56, 150, 9, 211,
251, 151, 78, 109, 155, 200, 135, 97, 44, 19, 63, 102, 179, 41, 222,
48, 117, 224, 197, 235, 99, 128, 255, 85, 73, 119, 13, 230, 131, 0,
76, 49, 173, 45, 189, 62, 150, 15, 53, 157, 230, 198, 108, 226, 216,
116, 68, 61, 32, 30, 155, 142, 239, 224, 32, 238, 63, 196, 239, 224,
160, 182, 33, 234, 5, 170, 119, 180, 53, 23, 78, 33, 16, 247, 31, 180,
69, 7, 241, 185, 71, 122, 31, 35, 248, 22, 139, 103, 9, 70, 89, 204,
18, 172, 18, 148, 89, 60, 210, 7, 120, 140, 224, 91, 4, 207, 178, 24,
37, 152, 101, 177, 74, 80, 102, 241, 136, 251, 17, 247, 99, 4, 223,
98, 241, 44, 193, 40, 139, 89, 130, 85, 22, 101, 130, 71, 158, 3, 60,
70, 240, 45, 22, 207, 18, 140, 178, 152, 37, 88, 37, 40, 111, 226, 46,
193, 29, 130, 219, 44, 74, 4, 27, 44, 62, 36, 184, 197, 162, 72, 112,
147, 224, 3, 22, 239, 19, 20, 88, 220, 32, 184, 206, 34, 79, 144, 99,
241, 30, 193, 53, 22, 87, 9, 178, 4, 255, 206, 226, 93, 130, 43, 44,
24, 130, 127, 99, 113, 153, 96, 157, 69, 134, 224, 215, 4, 151, 88,
172, 241, 216, 75, 240, 83, 22, 63, 35, 120, 129, 197, 97, 130, 35, 4,
163, 44, 78, 176, 120, 153, 224, 85, 130, 215, 88, 188, 78, 48, 205,
226, 20, 193, 44, 193, 28, 139, 179, 4, 231, 88, 44, 16, 156, 103,
177, 68, 176, 194, 98, 149, 32, 195, 130, 33, 200, 178, 200, 17, 20,
8, 138, 44, 74, 4, 101, 138, 51, 44, 142, 17, 208, 27, 216, 65, 176,
157, 197, 86, 2, 11, 193, 218, 93, 192, 25, 130, 99, 44, 122, 9, 58,
88, 108, 39, 216, 202, 194, 66, 176, 118, 103, 237, 206, 25, 22, 199,
8, 122, 89, 116, 16, 108, 103, 177, 149, 192, 194, 98, 237, 246, 218,
237, 51, 4, 199, 88, 244, 18, 116, 176, 216, 78, 176, 149, 5, 124, 87,
132, 142, 77, 15, 167, 80, 231, 80, 231, 234, 165, 115, 225, 20, 216,
88, 227, 58, 247, 24, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28,
234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234,
28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28,
234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234,
28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28, 234, 28,
234, 220, 125, 210, 57, 51, 99, 54, 81, 231, 80, 231, 106, 213, 57,
250, 78, 53, 142, 77, 71, 212, 11, 252, 23, 34, 240, 29, 28, 68, 125,
32, 125, 247, 11, 53, 15, 113, 255, 128, 223, 251, 66, 212, 27, 248,
189, 47, 76, 235, 155, 226, 247, 190, 48, 125, 0, 41, 126, 239, 11,
211, 186, 167, 248, 189, 47, 76, 31, 196, 156, 135, 248, 189, 47, 76,
235, 151, 226, 28, 214, 152, 214, 59, 69, 157, 195, 180, 222, 41, 234,
28, 166, 245, 78, 81, 231, 48, 173, 119, 138, 58, 135, 105, 189, 83,
212, 57, 76, 235, 157, 162, 206, 97, 90, 239, 20, 117, 14, 211, 122,
167, 248, 189, 47, 76, 235, 157, 226, 247, 190, 48, 173, 127, 138,
223, 251, 194, 180, 254, 125, 252, 248, 189, 47, 76, 235, 59, 150, 9,
191, 247, 133, 41, 126, 239, 11, 241, 241, 6, 126, 239, 11, 81, 111,
224, 247, 190, 16, 15, 6, 248, 189, 47, 68, 189, 129, 223, 251, 194,
185, 35, 30, 204, 220, 17, 248, 189, 47, 212, 57, 156, 151, 9, 117,
14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14,
117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117,
14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14,
117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117, 14, 117,
14, 117, 14, 117, 14, 117, 14, 117, 238, 147, 168, 115, 248, 189, 47,
212, 185, 122, 234, 28, 126, 239, 11, 81, 95, 224, 247, 190, 16, 245,
6, 125, 7, 231, 255, 3, 87, 66, 189, 33
};
#define LOAD_ASSETS()  GD.copy(__assets, sizeof(__assets))
#define ICONS32_HANDLE 0
#define ICONS32_WIDTH 32
#define ICONS32_HEIGHT 32
#define ICONS32_CELLS 35
#define SLIDER_HANDLE 1
#define SLIDER_WIDTH 314
#define SLIDER_HEIGHT 22
#define SLIDER_CELLS 2
#define NUMBERS_HANDLE 2
#define NUMBERS_WIDTH 314
#define NUMBERS_HEIGHT 22
#define NUMBERS_CELLS 2
#define ASSETS_END 126944UL
static const shape_t ICONS32_SHAPE = {0, 32, 32, 0};
static const shape_t SLIDER_SHAPE = {1, 314, 22, 0};
static const shape_t NUMBERS_SHAPE = {2, 314, 22, 0};

#endif

