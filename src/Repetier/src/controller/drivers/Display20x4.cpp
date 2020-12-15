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

#include "Repetier.h"

#if DISPLAY_DRIVER == DRIVER_20x4

// Menu up sign - code 1
// ..*.. 4
// .***. 14
// *.*.* 21
// ..*.. 4
// ..*.. 4
// ..*.. 4
// ***.. 28
// ..... 0
const uint8_t character_back[8] PROGMEM = { 4, 14, 21, 4, 4, 4, 28, 0 };
// Degrees sign - code 2
// ..*.. 4
// .*.*. 10
// ..*.. 4
// ..... 0
// ..... 0
// ..... 0
// ..... 0
// ..... 0
const uint8_t character_degree[8] PROGMEM = { 4, 10, 4, 0, 0, 0, 0, 0 };
// selected - code 3
// ..... 0
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_selected[8] PROGMEM = { 0, 31, 31, 31, 31, 31, 0, 0 };
// unselected - code 4
// ..... 0
// ***** 31
// *...* 17
// *...* 17
// *...* 17
// *...* 17
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_unselected[8] PROGMEM = { 0, 31, 17, 17, 17, 31, 0, 0 };
// unselected - code 5
// ..*.. 4
// .*.*. 10
// .*.*. 10
// .*.*. 10
// .*.*. 10
// .***. 14
// ***** 31
// ***** 31
// .***. 14
const uint8_t character_temperature[8] PROGMEM = { 4, 10, 10, 10, 14, 31, 31, 14 };
// unselected - code 6
// ..... 0
// ***.. 28
// ***** 31
// *...* 17
// *...* 17
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_folder[8] PROGMEM = { 0, 28, 31, 17, 17, 31, 0, 0 };

// printer ready - code 7
// *...* 17
// .*.*. 10
// ..*.. 4
// *...* 17
// ..*.. 4
// .*.*. 10
// *...* 17
// *...* 17
const byte character_ready[8] PROGMEM = { 17, 10, 4, 17, 4, 10, 17, 17 };

//Symbolic character values for specific symbols.
//May be overridden for different controllers, character sets, etc.
#define cUP 1
#define cDEG 2
#define cSEL 3
#define cUNSEL 4
#define cTEMP 5
#define cFOLD 6
#define bFOLD 6
#define cARROW 176

#define LCD_ENTRYMODE 0x04 /**< Set entrymode */

/** @name GENERAL COMMANDS */
/*@{*/
#define LCD_CLEAR 0x01 /**< Clear screen */
#define LCD_HOME 0x02  /**< Cursor move to first digit */
/*@}*/

/** @name ENTRYMODES */
/*@{*/
#define LCD_ENTRYMODE 0x04                       /**< Set entrymode */
#define LCD_INCREASE LCD_ENTRYMODE | 0x02        /**<    Set cursor move direction -- Increase */
#define LCD_DECREASE LCD_ENTRYMODE | 0x00        /**<    Set cursor move direction -- Decrease */
#define LCD_DISPLAYSHIFTON LCD_ENTRYMODE | 0x01  /**<    Display is shifted */
#define LCD_DISPLAYSHIFTOFF LCD_ENTRYMODE | 0x00 /**<    Display is not shifted */
/*@}*/

/** @name DISPLAYMODES */
/*@{*/
#define LCD_DISPLAYMODE 0x08                   /**< Set displaymode */
#define LCD_DISPLAYON LCD_DISPLAYMODE | 0x04   /**<    Display on */
#define LCD_DISPLAYOFF LCD_DISPLAYMODE | 0x00  /**<    Display off */
#define LCD_CURSORON LCD_DISPLAYMODE | 0x02    /**<    Cursor on */
#define LCD_CURSOROFF LCD_DISPLAYMODE | 0x00   /**<    Cursor off */
#define LCD_BLINKINGON LCD_DISPLAYMODE | 0x01  /**<    Blinking on */
#define LCD_BLINKINGOFF LCD_DISPLAYMODE | 0x00 /**<    Blinking off */
/*@}*/

/** @name SHIFTMODES */
/*@{*/
#define LCD_SHIFTMODE 0x10                    /**< Set shiftmode */
#define LCD_DISPLAYSHIFT LCD_SHIFTMODE | 0x08 /**<    Display shift */
#define LCD_CURSORMOVE LCD_SHIFTMODE | 0x00   /**<    Cursor move */
#define LCD_RIGHT LCD_SHIFTMODE | 0x04        /**<    Right shift */
#define LCD_LEFT LCD_SHIFTMODE | 0x00         /**<    Left shift */
/*@}*/

/** @name DISPLAY_CONFIGURATION */
/*@{*/
#define LCD_CONFIGURATION 0x20             /**< Set function */
#define LCD_8BIT LCD_CONFIGURATION | 0x10  /**<    8 bits interface */
#define LCD_4BIT LCD_CONFIGURATION | 0x00  /**<    4 bits interface */
#define LCD_2LINE LCD_CONFIGURATION | 0x08 /**<    2 line display */
#define LCD_1LINE LCD_CONFIGURATION | 0x00 /**<    1 line display */
#define LCD_5X10 LCD_CONFIGURATION | 0x04  /**<    5 X 10 dots */
#define LCD_5X7 LCD_CONFIGURATION | 0x00   /**<    5 X 7 dots */

#define LCD_SETCGRAMADDR 0x40

static const uint8_t LCDLineOffsets[] PROGMEM = UI_LINE_OFFSETS;

void createChar(uint8_t location, const uint8_t charmap[]);
void lcdWriteByte(uint8_t c, uint8_t rs);
#define lcdPutChar(value) lcdWriteByte(value, 1)
#define lcdCommand(value) lcdWriteByte(value, 0)

#if LCD_CONNECTOR == LCD_CONNECTOR_4BIT_PARALLEL
void lcdWriteNibble(uint8_t value) {
    WRITE(UI_DISPLAY_D4_PIN, value & 1);
    WRITE(UI_DISPLAY_D5_PIN, value & 2);
    WRITE(UI_DISPLAY_D6_PIN, value & 4);
    WRITE(UI_DISPLAY_D7_PIN, value & 8);
    DELAY1MICROSECOND;
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    HAL::delayMicroseconds(UI_DELAYPERCHAR);
}

void lcdWriteByte(uint8_t c, uint8_t rs) {
#if false && UI_DISPLAY_RW_PIN >= 0 // not really needed
    SET_INPUT(UI_DISPLAY_D4_PIN);
    SET_INPUT(UI_DISPLAY_D5_PIN);
    SET_INPUT(UI_DISPLAY_D6_PIN);
    SET_INPUT(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_RW_PIN, HIGH);
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    uint8_t busy;
    do {
        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        DELAY1MICROSECOND;
        busy = READ(UI_DISPLAY_D7_PIN);
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        DELAY2MICROSECOND;

        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        DELAY2MICROSECOND;

        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        DELAY2MICROSECOND;

    } while (busy);
    SET_OUTPUT(UI_DISPLAY_D4_PIN);
    SET_OUTPUT(UI_DISPLAY_D5_PIN);
    SET_OUTPUT(UI_DISPLAY_D6_PIN);
    SET_OUTPUT(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_RW_PIN, LOW);
#endif
    WRITE(UI_DISPLAY_RS_PIN, rs);

    WRITE(UI_DISPLAY_D4_PIN, c & 0x10);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x20);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x40);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x80);
#if FEATURE_CONTROLLER == CONTROLLER_RADDS
    HAL::delayMicroseconds(10);
#else
    HAL::delayMicroseconds(2);
#endif
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
#if FEATURE_CONTROLLER == CONTROLLER_RADDS
    HAL::delayMicroseconds(10);
#else
    HAL::delayMicroseconds(2);
#endif
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

    WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    HAL::delayMicroseconds(100);
}

#ifdef TRY_AUTOREPAIR_LCD_ERRORS
#define HAS_AUTOREPAIR
/* Fast repair function for displays loosing their settings.
  Do not call this if your display has no problems.
*/
void repairLCD() {
    // Now we pull both RS and R/W low to begin commands
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

    //put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    // at this point we are in 8 bit mode but of course in this
    // interface 4 pins are dangling unconnected and the values
    // on them don't matter for these instructions.
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    HAL::delayMicroseconds(20);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
    // second try
    //lcdWriteNibble(0x03);
    //HAL::delayMicroseconds(5000); // wait
    // third go!
    //lcdWriteNibble(0x03);
    //HAL::delayMicroseconds(160);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(160);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);              //- Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF); //- Display on
    createChar(1, character_back);
    createChar(2, character_degree);
    createChar(3, character_selected);
    createChar(4, character_unselected);
    createChar(5, character_temperature);
    createChar(6, character_folder);
    createChar(7, character_ready);
}
#endif

void initializeLCD() {
    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way before 4.5V.
    // is this delay long enough for all cases??

    if (GUI::curBootState != GUIBootState::DISPLAY_INIT) {
        return;
    }

    SET_OUTPUT(UI_DISPLAY_D4_PIN);
    SET_OUTPUT(UI_DISPLAY_D5_PIN);
    SET_OUTPUT(UI_DISPLAY_D6_PIN);
    SET_OUTPUT(UI_DISPLAY_D7_PIN);
    SET_OUTPUT(UI_DISPLAY_RS_PIN);
#if UI_DISPLAY_RW_PIN > -1
    SET_OUTPUT(UI_DISPLAY_RW_PIN);
#endif
    SET_OUTPUT(UI_DISPLAY_ENABLE_PIN);

    // Now we pull both RS and R/W low to begin commands
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

    //put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    // at this point we are in 8 bit mode but of course in this
    // interface 4 pins are dangling unconnected and the values
    // on them don't matter for these instructions.
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    HAL::delayMicroseconds(20);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
    // second try
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // wait
    // third go!
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(160);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(160);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);

    lcdCommand(LCD_CLEAR);                                       //- Clear Screen
    HAL::delayMilliseconds(3);                                   // clear is slow operation
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);              //- Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF); //- Display on
    createChar(1, character_back);
    createChar(2, character_degree);
    createChar(3, character_selected);
    createChar(4, character_unselected);
    createChar(5, character_temperature);
    createChar(6, character_folder);
    createChar(7, character_ready);
}
// ----------- end direct LCD driver
#endif

#if LCD_CONNECTOR == LCD_CONNECTOR_I2C_4BIT_PARALLEL
// ============= I2C LCD Display driver ================
inline void lcdStartWrite() {
    HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
#if UI_DISPLAY_I2C_CHIPTYPE == 1
    HAL::i2cWrite(0x14); // Start at port a
#endif
}
inline void lcdStopWrite() {
    HAL::i2cStop();
}
void lcdWriteNibble(uint8_t value) {
#if UI_DISPLAY_I2C_CHIPTYPE == 0
    value |= uid.outputMask;
#if UI_DISPLAY_D4_PIN == 1 && UI_DISPLAY_D5_PIN == 2 && UI_DISPLAY_D6_PIN == 4 && UI_DISPLAY_D7_PIN == 8
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#else
    uint8_t v = (value & 1 ? UI_DISPLAY_D4_PIN : 0) | (value & 2 ? UI_DISPLAY_D5_PIN : 0) | (value & 4 ? UI_DISPLAY_D6_PIN : 0) | (value & 8 ? UI_DISPLAY_D7_PIN : 0);
    HAL::i2cWrite((v) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(v);
#
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE == 1
    unsigned int v = (value & 1 ? UI_DISPLAY_D4_PIN : 0) | (value & 2 ? UI_DISPLAY_D5_PIN : 0) | (value & 4 ? UI_DISPLAY_D6_PIN : 0) | (value & 8 ? UI_DISPLAY_D7_PIN : 0) | uid.outputMask;
    unsigned int v2 = v | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(v2 & 255);
    HAL::i2cWrite(v2 >> 8);
    HAL::i2cWrite(v & 255);
    HAL::i2cWrite(v >> 8);
#endif
}
void lcdWriteByte(uint8_t c, uint8_t rs) {
#if UI_DISPLAY_I2C_CHIPTYPE == 0
    uint8_t mod = (rs ? UI_DISPLAY_RS_PIN : 0) | uid.outputMask; // | (UI_DISPLAY_RW_PIN);
#if UI_DISPLAY_D4_PIN == 1 && UI_DISPLAY_D5_PIN == 2 && UI_DISPLAY_D6_PIN == 4 && UI_DISPLAY_D7_PIN == 8
    uint8_t value = (c >> 4) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
    value = (c & 15) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#else
    uint8_t value = (c & 16 ? UI_DISPLAY_D4_PIN : 0) | (c & 32 ? UI_DISPLAY_D5_PIN : 0) | (c & 64 ? UI_DISPLAY_D6_PIN : 0) | (c & 128 ? UI_DISPLAY_D7_PIN : 0) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
    value = (c & 1 ? UI_DISPLAY_D4_PIN : 0) | (c & 2 ? UI_DISPLAY_D5_PIN : 0) | (c & 4 ? UI_DISPLAY_D6_PIN : 0) | (c & 8 ? UI_DISPLAY_D7_PIN : 0) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE == 1
    unsigned int mod = (rs ? UI_DISPLAY_RS_PIN : 0) | uid.outputMask; // | (UI_DISPLAY_RW_PIN);
    unsigned int value = (c & 16 ? UI_DISPLAY_D4_PIN : 0) | (c & 32 ? UI_DISPLAY_D5_PIN : 0) | (c & 64 ? UI_DISPLAY_D6_PIN : 0) | (c & 128 ? UI_DISPLAY_D7_PIN : 0) | mod;
    unsigned int value2 = (value) | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(value2 & 255);
    HAL::i2cWrite(value2 >> 8);
    HAL::i2cWrite(value & 255);
    HAL::i2cWrite(value >> 8);
    value = (c & 1 ? UI_DISPLAY_D4_PIN : 0) | (c & 2 ? UI_DISPLAY_D5_PIN : 0) | (c & 4 ? UI_DISPLAY_D6_PIN : 0) | (c & 8 ? UI_DISPLAY_D7_PIN : 0) | mod;
    value2 = (value) | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(value2 & 255);
    HAL::i2cWrite(value2 >> 8);
    HAL::i2cWrite(value & 255);
    HAL::i2cWrite(value >> 8);
#endif
}
void initializeLCD() {
    HAL::delayMilliseconds(235);
    lcdStartWrite();
    HAL::i2cWrite(uid.outputMask & 255);
#if UI_DISPLAY_I2C_CHIPTYPE == 1
    HAL::i2cWrite(uid.outputMask >> 8);
#endif
    HAL::delayMicroseconds(20);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(6000); // I have one LCD for which 4500 here was not long enough.
    // second try
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(180); // wait
    // third go!
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(180);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(180);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);
    lcdCommand(LCD_CLEAR);                                       //- Clear Screen
    HAL::delayMilliseconds(4);                                   // clear is slow operation
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);              //- Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF); //- Display on
    createChar(1, character_back);
    createChar(2, character_degree);
    createChar(3, character_selected);
    createChar(4, character_unselected);
    createChar(5, character_temperature);
    createChar(6, character_folder);
    createChar(7, character_ready);
    lcdStopWrite();
}
#endif

void createChar(uint8_t location, const uint8_t charmap[]) {
    location &= 0x7; // we only have 8 locations 0-7
    lcdCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcdPutChar(pgm_read_byte(&(charmap[i])));
    }
}

uint32_t nextRune(char** ptr) {
    unsigned char p1 = **ptr;
    (*ptr)++;
    if (p1 < 128) {
        return p1;
    }
    unsigned char p2 = **ptr;
    (*ptr)++;
    if ((p1 & 240) == 240) { // 4 byte value
        unsigned char p3 = **ptr;
        (*ptr)++;
        unsigned char p4 = **ptr;
        (*ptr)++;
        return (static_cast<uint32_t>(p1 & 0x7f) << 18) + (static_cast<uint32_t>(p2 & 0x3f) << 12) + (static_cast<uint32_t>(p3 & 0x3f) << 6) + (p4 & 0x3f);
    }
    if ((p1 & 224) == 224) { // 3 byte value
        unsigned char p3 = **ptr;
        (*ptr)++;
        return (static_cast<uint32_t>(p1 & 0x3f) << 12) + (static_cast<uint32_t>(p2 & 0x3f) << 6) + (p3 & 0x3f);
    }
    // 2 byte value
    return (static_cast<uint16_t>(p1 & 0x1f) << 6) + (p2 & 0x3f);
}
int32_t nextRuneSize(char** ptr, size_t& sz) {
    unsigned char p1 = **ptr;
    (*ptr)++;
    sz++;
    if (p1 < 128) {
        return p1;
    }
    unsigned char p2 = **ptr;
    (*ptr)++;
    if ((p1 & 240) == 240) { // 4 byte value
        unsigned char p3 = **ptr;
        (*ptr)++;
        unsigned char p4 = **ptr;
        (*ptr)++;
        uint32_t cp = (static_cast<uint32_t>(p1 & 0x7f) << 18) + (static_cast<uint32_t>(p2 & 0x3f) << 12) + (static_cast<uint32_t>(p3 & 0x3f) << 6) + (p4 & 0x3f);
#if LCD_CHAR_ENCODING == LCD_CHAR_ENCODING_HD44870_KNJI // HD44870 charset with knji chars
        if (cp == 0xc4 || cp == 0xd6 || cp == 0xdc) {
            sz++;
        }
#endif
        return cp;
    }
    if ((p1 & 224) == 224) { // 3 byte value
        unsigned char p3 = **ptr;
        (*ptr)++;
        return (static_cast<uint32_t>(p1 & 0x3f) << 12) + (static_cast<uint32_t>(p2 & 0x3f) << 6) + (p3 & 0x3f);
    }
    // 2 byte value
    return (static_cast<uint16_t>(p1 & 0x1f) << 6) + (p2 & 0x3f);
}
size_t utf8Length(char* text) {
    size_t len = 0;
    while (nextRuneSize(&text, len)) {
    }
    return --len;
}
uint8_t col = 0;

void putRune(uint32_t rune) {
#if LCD_CHAR_ENCODING == LCD_CHAR_ENCODING_HD44870_UTF8
    if (rune < 128) {
        lcdPutChar(static_cast<char>(rune));
        return;
    }
    if (rune < 2048) {
        lcdPutChar((static_cast<uint8_t>(rune >> 6) & 0x1f) | 0xc0);
        lcdPutChar((static_cast<uint8_t>(rune) & 0x3f) | 0x80);
        return;
    }
    if (rune < 65536) {
        lcdPutChar((static_cast<uint8_t>(rune >> 12) & 0x0f) | 0xe0);
        lcdPutChar((static_cast<uint8_t>(rune >> 6) & 0x3f) | 0x80);
        lcdPutChar((static_cast<uint8_t>(rune) & 0x3f) | 0x80);
        return;
    }
    lcdPutChar((static_cast<uint8_t>(rune >> 18) & 0x07) | 0xf0);
    lcdPutChar((static_cast<uint8_t>(rune >> 12) & 0x3f) | 0x80);
    lcdPutChar((static_cast<uint8_t>(rune >> 6) & 0x3f) | 0x80);
    lcdPutChar((static_cast<uint8_t>(rune) & 0x3f) | 0x80);
#else
    if (rune < 128) {
        lcdPutChar(static_cast<char>(rune));
        return;
    }
    switch (rune) {
    /* case 0x2103: // °C char
        lcdPutChar(cDEG);
        lcdPutChar('C');
        break; */
    case 0xb0: // ° char
        lcdPutChar(cDEG);
        break;
#if LCD_CHAR_ENCODING == LCD_CHAR_ENCODING_HD44870_KNJI // HD44870 charset with knji chars
    case 0xe4: // ä
        lcdPutChar(0xe1);
        break;
    case 0xc4: // Ä
        lcdPutChar('A');
        if (col < UI_COLS) {
            lcdPutChar('e');
            col++;
        }
        break;
    case 0xf6: // ö
        lcdPutChar(0xef);
        break;
    case 0xd6: // Ö
        lcdPutChar('O');
        if (col < UI_COLS) {
            lcdPutChar('e');
            col++;
        }
        break;
    case 0xfc: // ü
        lcdPutChar(0xf5);
        break;
    case 0xdc: // Ü
        lcdPutChar('U');
        if (col < UI_COLS) {
            lcdPutChar('e');
            col++;
        }
        break;
    case 0xdf: // ß
        lcdPutChar(0xe2);
        break;
#endif
#if LCD_CHAR_ENCODING == LCD_CHAR_ENCODING_HD44870_WESTERN // HD44870 charset with knji chars
    case 0xe4: // ä
        lcdPutChar(0x84);
        break;
    case 0xc4: // Ä
        lcdPutChar(0x8e);
        break;
    case 0xf6: // ö
        lcdPutChar(0x84);
        break;
    case 0xd6: // Ö
        lcdPutChar(0x89);
        break;
    case 0xfc: // ü
        lcdPutChar(0x81);
        break;
    case 0xdc: // Ü
        lcdPutChar(0x8a);
        break;
    case 0xdf: // ß
        lcdPutChar(0x70);
        break;
#endif
    }
#endif
}

void printRow(uint8_t r, char* txt) {
    // Set row
    col = 0;
    if (r >= UI_ROWS)
        return;
#if LCD_CONNECTOR == LCD_CONNECTOR_I2C_4BIT_PARALLEL
    lcdStartWrite();
#endif
    lcdWriteByte(128 + HAL::readFlashByte((const char*)&LCDLineOffsets[r]), 0); // Position cursor
    uint32_t c;
    while ((c = nextRune(&txt)) != 0 && col < UI_COLS) {
        putRune(c);
        col++;
    }
    while (col < UI_COLS) {
        lcdPutChar(' ');
        col++;
    }
#if LCD_CONNECTOR == LCD_CONNECTOR_I2C_4BIT_PARALLEL
    lcdStopWrite();
#endif
}

static fast8_t textScrollPos = 0;
static fast8_t textScrollWaits = 0;
static bool textScrollDir = false;

static char scrollBuf[sizeof(GUI::buf) + 1] = { 0 };
static void scrollSelectedText(int* row) {
    fast8_t lastCharPos = GUI::bufPos;
    if (lastCharPos > UI_COLS) {
        if (!GUI::textIsScrolling) {
            // Wait a few refresh ticks before starting scroll.
            textScrollWaits = 2;
            GUI::textIsScrolling = true;
        }
        if (textScrollWaits <= 0) {
            // Intentionally go one character over to show we're done.
            fast8_t maxScrollX = constrain(((lastCharPos + 1) - UI_COLS), 0, static_cast<fast8_t>(sizeof(scrollBuf) - 1));
            constexpr fast8_t scrollIncr = 1;
            if (!textScrollDir) {
                if ((textScrollPos += scrollIncr) >= maxScrollX) {
                    textScrollDir = true; // Left scroll
                    textScrollPos = maxScrollX;
                    textScrollWaits = 2;
                }
            } else {
                if ((textScrollPos -= scrollIncr) <= 0) {
                    textScrollDir = false; // Right scroll back
                    textScrollPos = 0;
                    textScrollWaits = 1;
                }
            }
        } else {
            textScrollWaits--;
        }
        memmove(&scrollBuf[0], &GUI::buf[textScrollPos], sizeof(scrollBuf) - 1);
        if (GUI::buf[0] == '>') {
            scrollBuf[0] = '>';
        }
        printRow((*row)++, scrollBuf);
    } else {
        printRow((*row)++, GUI::buf);
    }
}

void printRowCentered(uint8_t r, char* text) {
    unsigned int len = utf8Length(text);
    unsigned int extraSpaces = 0;
    if (len < UI_COLS) {
        extraSpaces = (UI_COLS - len) >> 1;
    }

    col = 0;
    // Set row
    if (r >= UI_ROWS)
        return;
#if LCD_CONNECTOR == LCD_CONNECTOR_I2C_4BIT_PARALLEL
    lcdStartWrite();
#endif
    lcdWriteByte(128 + HAL::readFlashByte((const char*)&LCDLineOffsets[r]), 0); // Position cursor
    uint32_t c;
    while (col < extraSpaces) {
        lcdPutChar(' ');
        col++;
    }
    while ((c = nextRune(&text)) != 0 && col < UI_COLS) {
        putRune(c);
        col++;
    }
    while (col < UI_COLS) {
        lcdPutChar(' ');
        col++;
    }
#if LCD_CONNECTOR == LCD_CONNECTOR_I2C_4BIT_PARALLEL
    lcdStopWrite();
#endif
}

millis_t init100msTicks = 0;
void GUI::init() {
    // Function called immediately at bootup
    init100msTicks = 0;
}
void GUI::processInit() {
    // Function called every 100ms (from GUI::update())
    // If our bootstate is still initalizing

    // A 200-300ms delay from inital power up -> to
    // sending commands seems to be safe for these little
    // character displays, otherwise you might get gibberish/
    // glitched characters etc.
    if (++init100msTicks < 3 || curBootState != GUIBootState::DISPLAY_INIT) { // 300 ms
        return;
    }

    initializeLCD();
    handleKeypress();
    nextAction = GUIAction::NONE;
    callbacks[0] = startScreen;
    pageType[0] = GUIPageType::FIXED_CONTENT;
    bufClear();
    setStatusP(PSTR("Ready"), GUIStatusLevel::REGULAR);
    bufClear();
    bufAddStringP(PSTR("Repetier-Firmware"));
    printRow(0, buf);
    bufClear();
    bufAddStringP(PSTR(REPETIER_VERSION));
    printRow(1, buf);
    bufClear();
    bufAddStringP(Com::tPrinterName);
    printRow(2, buf);
    bufClear();
    bufAddStringP(Com::tVendor);
    printRow(3, buf);
    lastRefresh = HAL::timeInMilliseconds() + UI_START_SCREEN_DELAY; // Show start screen 4s but will not delay start process
    curBootState = GUIBootState::IN_INTRO;                           // Switch to a skippable boot animation state
}

static fast8_t refresh_counter = 0;

void GUI::refresh() {
    refresh_counter++;
    callbacks[level](GUIAction::DRAW, data[level]);
}

// Draw top line status
void __attribute__((weak)) drawStatusLine() {
    // no place for status line
}

// Draw menu functions - driver specific

static bool npActionFound;
static int guiLine;
static int guiY;
static int guiSelIndex;

void GUI::menuStart(GUIAction action) {
    if (npActionFound) {
        if (GUI::textIsScrolling) {
            GUI::textIsScrolling = false;
            textScrollDir = false;
            textScrollPos = textScrollWaits = 0;
        }
    }
    npActionFound = false;
    guiLine = 0;
    guiSelIndex = cursorRow[level];
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        guiY = 0;
    }
}

void GUI::menuEnd(GUIAction action) {
    if (action == GUIAction::NEXT) {
        if (cursorRow[level] - topRow[level] >= UI_ROWS) {
            topRow[level] = cursorRow[level] + 1 - UI_ROWS;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (cursorRow[level] <= topRow[level]) {
            topRow[level] = RMath::max(static_cast<int32_t>(0), static_cast<int32_t>(cursorRow[level] - 1));
        }
    } else if (action == GUIAction::ANALYSE) {
        if (topRow[level] < 0) {
            topRow[level] = 0;
        }
        if (cursorRow[level] < 0) {
            cursorRow[level] = 0;
        }
    } else if (action == GUIAction::DRAW) {
        if (guiLine < UI_ROWS) {
            bufClear();
            const int count = (UI_ROWS - guiLine);
            for (int i = 0; i < count; i++) {
                printRow(guiLine++, buf);
            }
        }
    }
}

#define TEST_MENU_CLICK \
    if (guiLine == cursorRow[level]) { /* Actions for active line only!*/ \
        if (action == GUIAction::CLICK) { \
            if (tp == GUIPageType::POP) { /* Leave menu */ \
                pop(); \
            } else if (cb != nullptr && tp == GUIPageType::ACTION) { /* Execute a direct action */ \
                cb(action, cData); \
            } else if (cb) { /* Push new display function on stack */ \
                push(cb, cData, tp); \
            } \
            action = GUIAction::CLICK_PROCESSED; \
        } \
    }

void GUI::menuTextP(GUIAction& action, PGM_P text, bool highlight) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            bufAddStringP(text);
            if (guiLine == cursorRow[level]) {
                scrollSelectedText(&guiY);
            } else {
                printRow(guiY++, buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!highlight && !npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (!highlight && guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    }
    guiLine++;
}

void GUI::menuFloatP(GUIAction& action, PGM_P text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddStringP(text);
            bufAddFloat(val, 0, precision);
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuLongP(GUIAction& action, PGM_P text, long val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddStringP(text);
            bufAddLong(val, 0);
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuOnOffP(GUIAction& action, PGM_P text, bool val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddStringP(text);
            if (val) {
                bufAddStringP(PSTR("On"));
            } else {
                bufAddStringP(PSTR("Off"));
            }
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuSelectableP(GUIAction& action, PGM_P text, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        if (cursorRow[level] < 0) {
            cursorRow[level] = length[level];
        }
        maxCursorRow[level] = length[level];
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            GUI::bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            GUI::bufAddStringP(text);
            if (guiLine == cursorRow[level]) {
                scrollSelectedText(&guiY);
            } else {
                printRow(guiY++, buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuText(GUIAction& action, char* text, bool highlight) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            bufAddString(text);
            if (guiLine == cursorRow[level]) {
                scrollSelectedText(&guiY);
            } else {
                printRow(guiY++, buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!highlight && !npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (!highlight && guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    }
    guiLine++;
}

void GUI::menuFloat(GUIAction& action, char* text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddString(text);
            bufAddFloat(val, 0, precision);
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuLong(GUIAction& action, char* text, long val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddString(text);
            bufAddLong(val, 0);
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuOnOff(GUIAction& action, char* text, bool val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            bufAddString(text);
            if (val) {
                bufAddStringP(PSTR("On"));
            } else {
                bufAddStringP(PSTR("Off"));
            }
            printRow(guiY++, buf);
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuSelectable(GUIAction& action, char* text, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        if (cursorRow[level] < 0) {
            cursorRow[level] = length[level];
        }
        maxCursorRow[level] = length[level];
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + UI_ROWS) {
            GUI::bufClear();
            if (guiLine == cursorRow[level]) {
                bufAddChar('>');
            } else {
                bufAddChar(' ');
            }
            GUI::bufAddString(text);
            if (guiLine == cursorRow[level]) {
                scrollSelectedText(&guiY);
            } else {
                printRow(guiY++, buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

// Value modifyer display
// value is stored in GUI::buf
void GUI::showValueP(PGM_P text, PGM_P unit, char* value) {
    char valueCopy[MAX_COLS + 1];
    strncpy(valueCopy, value, MAX_COLS);
    GUI::bufClear();
    GUI::bufAddStringP(text);
    printRow(0, GUI::buf);
    GUI::bufClear();
    printRow(2, GUI::buf);
    GUI::bufAddString(valueCopy);
    GUI::bufAddChar(' ');
    GUI::bufAddStringP(unit);
    printRow(1, GUI::buf);
    GUI::bufClear();
    GUI::bufAddStringP(Com::tBtnOK);
    printRowCentered(3, GUI::buf);
}

void GUI::showValue(char* text, PGM_P unit, char* value) {
    char valueCopy[MAX_COLS + 1];
    strncpy(valueCopy, value, MAX_COLS);
    printRow(0, text);
    GUI::bufClear();
    printRow(2, GUI::buf);
    GUI::bufAddString(valueCopy);
    GUI::bufAddChar(' ');
    GUI::bufAddStringP(unit);
    printRow(1, GUI::buf);
    GUI::bufClear();
    GUI::bufAddStringP(Com::tBtnOK);
    printRowCentered(3, GUI::buf);
}

// No scrollbars for the 20x4's 
void GUI::showScrollbar(GUIAction& action) {
}
void GUI::showScrollbar(GUIAction& action, float percent, uint16_t min, uint16_t max) {
}
//extern void __attribute__((weak)) startScreen(GUIAction action, void* data);
//extern void __attribute__((weak)) printProgress(GUIAction action, void* data);
// extern void __attribute__((weak)) mainMenu(GUIAction action, void* data);
//void mainMenu(GUIAction action, void* data) __attribute__((weak, alias("WmainMenu")));
//void printProgress(GUIAction action, void* data) __attribute__((weak, alias("WprintProgress")));
//void startScreen(GUIAction action, void* data) __attribute__((weak, alias("WstartScreen")));

void __attribute__((weak)) startScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        // Extruder output
        // 12345678901234567890
        // E:123/200°C Z:999.00
        // B:123.1/80  FR:100%
        //

        int n = 0;

        Tool* tool = Tool::getActiveTool();
        GUI::bufClear();
        if (tool->getToolType() == ToolTypes::EXTRUDER) {
            GUI::bufAddChar('E');
            GUI::bufAddInt(tool->getToolId() + 1, 1);
            GUI::bufAddChar(':');
            GUI::bufAddHeaterTemp(tool->getHeater(), true);
        } else if (tool->getToolType() == ToolTypes::LASER) {

        } else if (tool->getToolType() == ToolTypes::MILL) {
        }
        GUI::bufAddStringP(PSTR(" Z:"));
        if (Motion1::isAxisHomed(Z_AXIS)) {
            if (Motion1::getShowPosition(Z_AXIS) < 1000) {
                GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 3, 2);
            } else {
                GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 4, 1);
            }
        } else {
            if (refresh_counter & 1) {
                GUI::bufAddChar('?');
                GUI::bufAddChar('?');
                GUI::bufAddChar('?');
                GUI::bufAddChar('.');
                GUI::bufAddChar('?');
            }
        }
        printRow(0, GUI::buf);

        GUI::bufClear();

#if NUM_HEATED_BEDS > 0 || NUM_HEATED_CHAMBERS > 0
        static fast8_t auxHeaterIndex = 0;
        if (auxHeaterIndex < NUM_HEATED_BEDS) {
            GUI::bufAddChar('B');
            if (NUM_HEATED_BEDS > 1) {
                GUI::bufAddInt(heatedBeds[auxHeaterIndex]->getIndex() + 1, 1);
            } else {
                GUI::bufAddChar(' ');
            }
            GUI::bufAddChar(':');
            GUI::bufAddHeaterTemp(heatedBeds[auxHeaterIndex], true);
        } else if (auxHeaterIndex >= NUM_HEATED_BEDS) {
            GUI::bufAddChar('C');
            if (NUM_HEATED_CHAMBERS > 1) {
                GUI::bufAddInt(heatedChambers[auxHeaterIndex - NUM_HEATED_BEDS]->getIndex() + 1, 1);
            } else {
                GUI::bufAddChar(' ');
            }
            GUI::bufAddChar(':');
            GUI::bufAddHeaterTemp(heatedChambers[auxHeaterIndex - NUM_HEATED_BEDS], true);
        }
        GUI::bufAddChar(' ');
        // change every four seconds.
        if (!(refresh_counter % 4)) {
            if (++auxHeaterIndex > (NUM_HEATED_BEDS + NUM_HEATED_CHAMBERS) - 1) {
                auxHeaterIndex = 0;
            }
        }
#endif
        GUI::bufAddStringP(PSTR("FR:"));
        GUI::bufAddInt(Printer::feedrateMultiply, 3);
        GUI::bufAddChar('%');
        printRow(1, GUI::buf);

        // 3rd line is depending on state
        GUI::bufClear();
        n = 0;
        if (tool->getToolType() == ToolTypes::EXTRUDER && tool->hasSecondary() && n < 2) {
            GUI::bufAddStringP("Fan:");
            GUI::bufAddInt(tool->secondaryPercent(), 3);
            GUI::bufAddChar('%');
            GUI::bufAddChar(' ');
            n++;
        }

        if (n < 2 && Printer::areAllSteppersDisabled()) {
            GUI::bufAddStringP("Motors off");
            GUI::bufAddChar(' ');
            n++;
        }

        if (tool->getToolType() == ToolTypes::EXTRUDER && n < 2) {
            GUI::bufAddStringP("Flow:");
            GUI::bufAddInt(Printer::extrudeMultiply, 3);
            GUI::bufAddChar('%');
            GUI::bufAddChar(' ');
            n++;
        }
        printRow(2, GUI::buf);

        // status info
        printRow(3, GUI::status);
    }
    if (Printer::isPrinting() || Printer::isZProbingActive()) {
        GuiCallback cb = Printer::isPrinting() ? printProgress : probeProgress;
        GUI::replaceOn(GUIAction::NEXT, cb, nullptr, GUIPageType::FIXED_CONTENT);
        GUI::replaceOn(GUIAction::PREVIOUS, cb, nullptr, GUIPageType::FIXED_CONTENT);
    }
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) printProgress(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
#if SDSUPPORT
        if (sd.state == SDState::SD_PRINTING) { // print from sd card
            Printer::progress = (static_cast<float>(sd.selectedFilePos) * 100.0) / static_cast<float>(sd.selectedFileSize);
        }
#endif
        static bool cycle = false;
        uint8_t row = 0u;
        GUI::bufClear();
        // Cycle name if not sd printing every 4 sec
        if ((Printer::maxLayer != -1) && !(refresh_counter % 4)) {
            cycle = !cycle;
        }

        if (!cycle) {
            GUI::bufAddStringP(PSTR("Progress: "));
            GUI::bufAddFloat(Printer::progress, 3, 1);
            GUI::bufAddStringP(Com::tUnitPercent);
        } else {
            GUI::bufAddString(Printer::printName);
        }

        printRow(row++, GUI::buf);
        GUI::bufClear();
        if (Printer::maxLayer != -1) {
            GUI::bufAddStringP(PSTR("Layer: "));
            GUI::bufAddInt(Printer::currentLayer, 3);
            GUI::bufAddStringP(Com::tSlash);
            GUI::bufAddInt(Printer::maxLayer, 3);
            printRow(row++, GUI::buf);
            GUI::bufClear();
        }
        GUI::bufAddStringP(PSTR("Height: "));
        GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 3, 2);
        GUI::bufAddStringP(Com::tUnitMM);

        if (Printer::maxLayer == -1) { // SD Printing, always show name
            printRow(row++, GUI::buf);
            GUI::bufClear();
            GUI::bufAddString(Printer::printName);
        }

        printRow(row++, GUI::buf);
        GUI::bufClear();
        printRow(3, GUI::status);
    }
    GUI::replaceOn(GUIAction::NEXT, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) probeProgress(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        if (Printer::isZProbingActive()) {
            GUI::bufClear();
            GUI::bufAddStringP(Com::tProbing);
            // Using our own counter instead of refresh_counter
            // since this screen might get updated faster than usual
            static ufast8_t dotCounter = 0;
            static millis_t lastDotTime = 0;
            if ((HAL::timeInMilliseconds() - lastDotTime) >= 1000ul) {
                dotCounter++;
                lastDotTime = HAL::timeInMilliseconds();
            }

            //...
            fast8_t len = dotCounter % 4;
            for (fast8_t i = 0; i < 3; i++) {
                GUI::bufAddChar(i < len ? '.' : ' ');
            }
            if (GUI::curProbingProgress) {

                GUI::bufAddChar(' ');
                GUI::bufAddChar(' ');
                GUI::bufAddLong((GUI::curProbingProgress->num * 100u) / (GUI::curProbingProgress->maxNum), 3);
                GUI::bufAddChar('%');
                printRow(0, GUI::buf);
                GUI::bufClear();
                // Xmm x Ymm
                GUI::bufAddStringP(Com::tXColon);
                GUI::bufAddLong(GUI::curProbingProgress->x, 3);
                GUI::bufAddStringP(Com::tUnitMM);
                GUI::bufAddChar(' ');
                GUI::bufAddStringP(Com::tYColon);
                GUI::bufAddLong(GUI::curProbingProgress->y, 3);
                GUI::bufAddStringP(Com::tUnitMM);
                printRowCentered(1, GUI::buf);
                GUI::bufClear();
                // (+0.000mm)
                float z = GUI::curProbingProgress->z;
                if (z != IGNORE_COORDINATE && z != ILLEGAL_Z_PROBE) {
                    GUI::bufAddChar('(');
                    if (z >= 0) {
                        GUI::bufAddChar('+');
                    }
                    GUI::bufAddFloat(z, 0, 3);
                    GUI::bufAddStringP(Com::tUnitMM);
                    GUI::bufAddChar(')');
                }
                printRowCentered(2, GUI::buf);
            } else {
                GUI::bufClear();
                printRow(0, GUI::buf);
                printRow(1, GUI::buf);
                printRow(2, GUI::buf);
            }
            printRow(3, GUI::status);
        }
    }
    GUI::replaceOn(GUIAction::NEXT, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}
void __attribute__((weak)) warningScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(Com::tWarning);
        printRow(0, GUI::buf);
        char* text = static_cast<char*>(data);
        printRowCentered(1, text);
        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) errorScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Error"));
        printRow(0, GUI::buf);
        char* text = static_cast<char*>(data);
        printRowCentered(1, text);
        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) infoScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Info"));
        printRow(0, GUI::buf);
        char* text = static_cast<char*>(data);
        printRowCentered(1, text);
        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) warningScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Warning"));
        printRow(0, GUI::buf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        printRowCentered(1, GUI::buf);

        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) errorScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Error"));
        printRow(0, GUI::buf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        printRowCentered(1, GUI::buf);
        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) infoScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Info"));
        printRow(0, GUI::buf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        printRowCentered(1, GUI::buf);
        GUI::bufClear();
        printRow(2, GUI::buf);
        GUI::bufAddStringP(Com::tBtnOK);
        printRowCentered(3, GUI::buf);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
#define SPIN_CENTER_X 10
#define SPIN_CENTER_Y 38

void waitScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        char* text = static_cast<char*>(data);

        char* newLine = strchr(text, '\n');
        if (newLine) {
            *newLine = '\0';
            printRowCentered(0, text);
            *newLine = '\n';
            printRowCentered(1, newLine + 1);
            GUI::bufClear();
        } else {
            printRowCentered(0, text);
            GUI::bufClear();
            printRow(1, GUI::buf);
        }
        printRow(2, GUI::buf);
        fast8_t len = refresh_counter % UI_COLS;
        for (fast8_t i = 0; i < len; i++) {
            GUI::bufAddChar('.');
        }
        printRow(3, GUI::buf);
    }
}

void waitScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        char* newLine = strchr(GUI::buf, '\n');
        if (newLine) {
            *newLine = '\0';
            printRowCentered(0, GUI::buf);
            *newLine = '\n';
            printRowCentered(1, newLine + 1);
            GUI::bufClear();
        } else {
            printRowCentered(0, GUI::buf);
            GUI::bufClear();
            printRow(1, GUI::buf);
        }
        printRow(2, GUI::buf);
        fast8_t len = refresh_counter % UI_COLS;
        for (fast8_t i = 0; i < len; i++) {
            GUI::bufAddChar('.');
        }
        printRow(3, GUI::buf);
    }
}

#endif
