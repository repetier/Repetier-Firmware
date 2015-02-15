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

*/

#define UI_MAIN 1
#include "Repetier.h"
// The uimenu.h declares static variables of menus, which must be declared only once.
// It does not define interfaces for other modules, so should never be included elsewhere
#include "uimenu.h"

extern const int8_t encoder_table[16] PROGMEM ;
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#if FEATURE_SERVO > 0 && UI_SERVO_CONTROL > 0
#if   UI_SERVO_CONTROL == 1 && defined(SERVO0_NEUTRAL_POS)
 uint16_t servoPosition = SERVO0_NEUTRAL_POS;
#elif UI_SERVO_CONTROL == 2 && defined(SERVO1_NEUTRAL_POS)
 uint16_t servoPosition = SERVO1_NEUTRAL_POS;
#elif UI_SERVO_CONTROL == 3 && defined(SERVO2_NEUTRAL_POS)
 uint16_t servoPosition = SERVO2_NEUTRAL_POS;
#elif UI_SERVO_CONTROL == 4 && defined(SERVO3_NEUTRAL_POS)
 uint16_t servoPosition = SERVO3_NEUTRAL_POS;
#else
 uint16_t servoPosition = 1500;
#endif
#endif

#if BEEPER_TYPE==2 && defined(UI_HAS_I2C_KEYS) && UI_I2C_KEY_ADDRESS!=BEEPER_ADDRESS
#error Beeper address and i2c key address must be identical
#else
#if BEEPER_TYPE==2
#define UI_I2C_KEY_ADDRESS BEEPER_ADDRESS
#endif
#endif

#if UI_AUTORETURN_TO_MENU_AFTER!=0
long ui_autoreturn_time=0;
#endif
#if FEATURE_BABYSTEPPING
 int zBabySteps = 0;
#endif

void beep(uint8_t duration,uint8_t count)
{
#if FEATURE_BEEPER
#if BEEPER_TYPE!=0
#if BEEPER_TYPE==1 && defined(BEEPER_PIN) && BEEPER_PIN>=0
    SET_OUTPUT(BEEPER_PIN);
#endif
#if BEEPER_TYPE==2
    HAL::i2cStartWait(BEEPER_ADDRESS+I2C_WRITE);
#if UI_DISPLAY_I2C_CHIPTYPE==1
    HAL::i2cWrite( 0x14); // Start at port a
#endif
#endif
    for(uint8_t i=0; i < count; i++)
    {
#if BEEPER_TYPE==1 && defined(BEEPER_PIN) && BEEPER_PIN>=0
#if defined(BEEPER_TYPE_INVERTING) && BEEPER_TYPE_INVERTING
        WRITE(BEEPER_PIN,LOW);
#else
        WRITE(BEEPER_PIN,HIGH);
#endif
#else
#if UI_DISPLAY_I2C_CHIPTYPE==0
#if BEEPER_ADDRESS == UI_DISPLAY_I2C_ADDRESS
        HAL::i2cWrite(uid.outputMask & ~BEEPER_PIN);
#else
        HAL::i2cWrite(~BEEPER_PIN);
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
        HAL::i2cWrite((BEEPER_PIN) | uid.outputMask);
        HAL::i2cWrite(((BEEPER_PIN) | uid.outputMask)>>8);
#endif
#endif
        HAL::delayMilliseconds(duration);
#if BEEPER_TYPE==1 && defined(BEEPER_PIN) && BEEPER_PIN>=0
#if defined(BEEPER_TYPE_INVERTING) && BEEPER_TYPE_INVERTING
        WRITE(BEEPER_PIN,HIGH);
#else
        WRITE(BEEPER_PIN,LOW);
#endif
#else
#if UI_DISPLAY_I2C_CHIPTYPE==0

#if BEEPER_ADDRESS == UI_DISPLAY_I2C_ADDRESS
        HAL::i2cWrite((BEEPER_PIN) | uid.outputMask);
#else
        HAL::i2cWrite(255);
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
        HAL::i2cWrite( uid.outputMask);
        HAL::i2cWrite(uid.outputMask>>8);
#endif
#endif
        HAL::delayMilliseconds(duration);
    }
#if BEEPER_TYPE==2
    HAL::i2cStop();
#endif
#endif
#endif
}

bool UIMenuEntry::showEntry() const
{
    bool ret = true;
    uint8_t f, f2;
    f = HAL::readFlashByte((PGM_P)&filter);
    if(f != 0)
        ret = (f & Printer::menuMode) != 0;
    if(ret && (f2 = HAL::readFlashByte((PGM_P)&nofilter)) != 0)
        ret = (f2 & Printer::menuMode) == 0;
    return ret;
}

#if UI_DISPLAY_TYPE != NO_DISPLAY
UIDisplay uid;
char displayCache[UI_ROWS][MAX_COLS+1];

// Menu up sign - code 1
// ..*.. 4
// .***. 14
// *.*.* 21
// ..*.. 4
// ***.. 28
// ..... 0
// ..... 0
// ..... 0
const uint8_t character_back[8] PROGMEM = {4,14,21,4,28,0,0,0};
// Degrees sign - code 2
// ..*.. 4
// .*.*. 10
// ..*.. 4
// ..... 0
// ..... 0
// ..... 0
// ..... 0
// ..... 0
const uint8_t character_degree[8] PROGMEM = {4,10,4,0,0,0,0,0};
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
const uint8_t character_selected[8] PROGMEM = {0,31,31,31,31,31,0,0};
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
const uint8_t character_unselected[8] PROGMEM = {0,31,17,17,17,31,0,0};
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
const uint8_t character_temperature[8] PROGMEM = {4,10,10,10,14,31,31,14};
// unselected - code 6
// ..... 0
// ***.. 28
// ***** 31
// *...* 17
// *...* 17
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_folder[8] PROGMEM = {0,28,31,17,17,31,0,0};

// printer ready - code 7
// *...* 17
// .*.*. 10
// ..*.. 4
// *...* 17
// ..*.. 4
// .*.*. 10
// *...* 17
// *...* 17
const byte character_ready[8] PROGMEM = {17,10,4,17,4,10,17,17};

const long baudrates[] PROGMEM = {9600,14400,19200,28800,38400,56000,57600,76800,111112,115200,128000,230400,250000,256000,
                                  460800,500000,921600,1000000,1500000,0
                                 };

#define LCD_ENTRYMODE			0x04			/**< Set entrymode */

/** @name GENERAL COMMANDS */
/*@{*/
#define LCD_CLEAR			0x01	/**< Clear screen */
#define LCD_HOME			0x02	/**< Cursor move to first digit */
/*@}*/

/** @name ENTRYMODES */
/*@{*/
#define LCD_ENTRYMODE			0x04			/**< Set entrymode */
#define LCD_INCREASE		LCD_ENTRYMODE | 0x02	/**<	Set cursor move direction -- Increase */
#define LCD_DECREASE		LCD_ENTRYMODE | 0x00	/**<	Set cursor move direction -- Decrease */
#define LCD_DISPLAYSHIFTON	LCD_ENTRYMODE | 0x01	/**<	Display is shifted */
#define LCD_DISPLAYSHIFTOFF	LCD_ENTRYMODE | 0x00	/**<	Display is not shifted */
/*@}*/

/** @name DISPLAYMODES */
/*@{*/
#define LCD_DISPLAYMODE			0x08			/**< Set displaymode */
#define LCD_DISPLAYON		LCD_DISPLAYMODE | 0x04	/**<	Display on */
#define LCD_DISPLAYOFF		LCD_DISPLAYMODE | 0x00	/**<	Display off */
#define LCD_CURSORON		LCD_DISPLAYMODE | 0x02	/**<	Cursor on */
#define LCD_CURSOROFF		LCD_DISPLAYMODE | 0x00	/**<	Cursor off */
#define LCD_BLINKINGON		LCD_DISPLAYMODE | 0x01	/**<	Blinking on */
#define LCD_BLINKINGOFF		LCD_DISPLAYMODE | 0x00	/**<	Blinking off */
/*@}*/

/** @name SHIFTMODES */
/*@{*/
#define LCD_SHIFTMODE			0x10			/**< Set shiftmode */
#define LCD_DISPLAYSHIFT	LCD_SHIFTMODE | 0x08	/**<	Display shift */
#define LCD_CURSORMOVE		LCD_SHIFTMODE | 0x00	/**<	Cursor move */
#define LCD_RIGHT		LCD_SHIFTMODE | 0x04	/**<	Right shift */
#define LCD_LEFT		LCD_SHIFTMODE | 0x00	/**<	Left shift */
/*@}*/

/** @name DISPLAY_CONFIGURATION */
/*@{*/
#define LCD_CONFIGURATION		0x20				/**< Set function */
#define LCD_8BIT		LCD_CONFIGURATION | 0x10	/**<	8 bits interface */
#define LCD_4BIT		LCD_CONFIGURATION | 0x00	/**<	4 bits interface */
#define LCD_2LINE		LCD_CONFIGURATION | 0x08	/**<	2 line display */
#define LCD_1LINE		LCD_CONFIGURATION | 0x00	/**<	1 line display */
#define LCD_5X10		LCD_CONFIGURATION | 0x04	/**<	5 X 10 dots */
#define LCD_5X7			LCD_CONFIGURATION | 0x00	/**<	5 X 7 dots */

#define LCD_SETCGRAMADDR 0x40

#define lcdPutChar(value) lcdWriteByte(value,1)
#define lcdCommand(value) lcdWriteByte(value,0)

static const uint8_t LCDLineOffsets[] PROGMEM = UI_LINE_OFFSETS;
static const char versionString[] PROGMEM = UI_VERSION_STRING;


#if UI_DISPLAY_TYPE == DISPLAY_I2C

// ============= I2C LCD Display driver ================
inline void lcdStartWrite()
{
    HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
#if UI_DISPLAY_I2C_CHIPTYPE == 1
    HAL::i2cWrite( 0x14); // Start at port a
#endif
}
inline void lcdStopWrite()
{
    HAL::i2cStop();
}
void lcdWriteNibble(uint8_t value)
{
#if UI_DISPLAY_I2C_CHIPTYPE==0
    value |= uid.outputMask;
#if UI_DISPLAY_D4_PIN==1 && UI_DISPLAY_D5_PIN==2 && UI_DISPLAY_D6_PIN==4 && UI_DISPLAY_D7_PIN==8
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#else
    uint8_t v=(value & 1?UI_DISPLAY_D4_PIN:0)|(value & 2?UI_DISPLAY_D5_PIN:0)|(value & 4?UI_DISPLAY_D6_PIN:0)|(value & 8?UI_DISPLAY_D7_PIN:0);
    HAL::i2cWrite((v) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(v);
#
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
    unsigned int v=(value & 1?UI_DISPLAY_D4_PIN:0)|(value & 2?UI_DISPLAY_D5_PIN:0)|(value & 4?UI_DISPLAY_D6_PIN:0)|(value & 8?UI_DISPLAY_D7_PIN:0) | uid.outputMask;
    unsigned int v2 = v | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(v2 & 255);
    HAL::i2cWrite(v2 >> 8);
    HAL::i2cWrite(v & 255);
    HAL::i2cWrite(v >> 8);
#endif
}
void lcdWriteByte(uint8_t c,uint8_t rs)
{
#if UI_DISPLAY_I2C_CHIPTYPE==0
    uint8_t mod = (rs?UI_DISPLAY_RS_PIN:0) | uid.outputMask; // | (UI_DISPLAY_RW_PIN);
#if UI_DISPLAY_D4_PIN==1 && UI_DISPLAY_D5_PIN==2 && UI_DISPLAY_D6_PIN==4 && UI_DISPLAY_D7_PIN==8
    uint8_t value = (c >> 4) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
    value = (c & 15) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#else
    uint8_t value = (c & 16?UI_DISPLAY_D4_PIN:0)|(c & 32?UI_DISPLAY_D5_PIN:0)|(c & 64?UI_DISPLAY_D6_PIN:0)|(c & 128?UI_DISPLAY_D7_PIN:0) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
    value = (c & 1?UI_DISPLAY_D4_PIN:0)|(c & 2?UI_DISPLAY_D5_PIN:0)|(c & 4?UI_DISPLAY_D6_PIN:0)|(c & 8?UI_DISPLAY_D7_PIN:0) | mod;
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#endif
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
    unsigned int mod = (rs?UI_DISPLAY_RS_PIN:0) | uid.outputMask; // | (UI_DISPLAY_RW_PIN);
    unsigned int value = (c & 16?UI_DISPLAY_D4_PIN:0)|(c & 32?UI_DISPLAY_D5_PIN:0)|(c & 64?UI_DISPLAY_D6_PIN:0)|(c & 128?UI_DISPLAY_D7_PIN:0) | mod;
    unsigned int value2 = (value) | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(value2 & 255);
    HAL::i2cWrite(value2 >>8);
    HAL::i2cWrite(value & 255);
    HAL::i2cWrite(value>>8);
    value = (c & 1?UI_DISPLAY_D4_PIN:0)|(c & 2?UI_DISPLAY_D5_PIN:0)|(c & 4?UI_DISPLAY_D6_PIN:0)|(c & 8?UI_DISPLAY_D7_PIN:0) | mod;
    value2 = (value) | UI_DISPLAY_ENABLE_PIN;
    HAL::i2cWrite(value2 & 255);
    HAL::i2cWrite(value2 >>8);
    HAL::i2cWrite(value & 255);
    HAL::i2cWrite(value>>8);
#endif
}
void initializeLCD()
{
    HAL::delayMilliseconds(235);
    lcdStartWrite();
    HAL::i2cWrite(uid.outputMask & 255);
#if UI_DISPLAY_I2C_CHIPTYPE==1
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
    lcdCommand(LCD_CLEAR);					//-	Clear Screen
    HAL::delayMilliseconds(4); // clear is slow operation
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);	//-	Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);	//-	Display on
    uid.lastSwitch = uid.lastRefresh = HAL::timeInMilliseconds();
    uid.createChar(1,character_back);
    uid.createChar(2,character_degree);
    uid.createChar(3,character_selected);
    uid.createChar(4,character_unselected);
    uid.createChar(5,character_temperature);
    uid.createChar(6,character_folder);
    uid.createChar(7,character_ready);
    lcdStopWrite();
}
#endif
#if UI_DISPLAY_TYPE == DISPLAY_4BIT || UI_DISPLAY_TYPE == DISPLAY_8BIT

void lcdWriteNibble(uint8_t value)
{
    WRITE(UI_DISPLAY_D4_PIN,value & 1);
    WRITE(UI_DISPLAY_D5_PIN,value & 2);
    WRITE(UI_DISPLAY_D6_PIN,value & 4);
    WRITE(UI_DISPLAY_D7_PIN,value & 8);
    DELAY1MICROSECOND;
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);// enable pulse must be >450ns
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    HAL::delayMicroseconds(UI_DELAYPERCHAR);
}

void lcdWriteByte(uint8_t c,uint8_t rs)
{
#if false && UI_DISPLAY_RW_PIN >= 0 // not really needed
    SET_INPUT(UI_DISPLAY_D4_PIN);
    SET_INPUT(UI_DISPLAY_D5_PIN);
    SET_INPUT(UI_DISPLAY_D6_PIN);
    SET_INPUT(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_RW_PIN, HIGH);
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    uint8_t busy;
    do
    {
        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        DELAY1MICROSECOND;
        busy = READ(UI_DISPLAY_D7_PIN);
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        DELAY2MICROSECOND;

        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        DELAY2MICROSECOND;

        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        DELAY2MICROSECOND;

    }
    while (busy);
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
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);   // enable pulse must be >450ns
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

    WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);   // enable pulse must be >450ns
    HAL::delayMicroseconds(2);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    HAL::delayMicroseconds(100);
}

void initializeLCD()
{
    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way before 4.5V.
    // is this delay long enough for all cases??
    HAL::delayMilliseconds(235);
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

    lcdCommand(LCD_CLEAR);					//-	Clear Screen
    HAL::delayMilliseconds(3); // clear is slow operation
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);	//-	Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);	//-	Display on
    uid.lastSwitch = uid.lastRefresh = HAL::timeInMilliseconds();
    uid.createChar(1, character_back);
    uid.createChar(2, character_degree);
    uid.createChar(3, character_selected);
    uid.createChar(4, character_unselected);
    uid.createChar(5, character_temperature);
    uid.createChar(6, character_folder);
    uid.createChar(7, character_ready);
}
// ----------- end direct LCD driver
#endif
#if UI_DISPLAY_TYPE < DISPLAY_ARDUINO_LIB
void UIDisplay::printRow(uint8_t r,char *txt,char *txt2,uint8_t changeAtCol)
{
    changeAtCol = RMath::min(UI_COLS,changeAtCol);
    uint8_t col=0;
// Set row
    if(r >= UI_ROWS) return;
#if UI_DISPLAY_TYPE == DISPLAY_I2C
    lcdStartWrite();
#endif
    lcdWriteByte(128 + HAL::readFlashByte((const char *)&LCDLineOffsets[r]), 0); // Position cursor
    char c;
    while((c = *txt) != 0x00 && col < changeAtCol)
    {
        txt++;
        lcdPutChar(c);
        col++;
    }
    while(col < changeAtCol)
    {
        lcdPutChar(' ');
        col++;
    }
    if(txt2 != NULL)
    {
        while((c = *txt2) != 0x00 && col < UI_COLS)
        {
            txt2++;
            lcdPutChar(c);
            col++;
        }
        while(col < UI_COLS)
        {
            lcdPutChar(' ');
            col++;
        }
    }
#if UI_DISPLAY_TYPE == DISPLAY_I2C
    lcdStopWrite();
#endif
#if UI_HAS_KEYS==1 && UI_HAS_I2C_ENCODER>0
    uiCheckSlowEncoder();
#endif
}
#endif

#if UI_DISPLAY_TYPE == DISPLAY_ARDUINO_LIB
// Use LiquidCrystal library instead
#include <LiquidCrystal.h>

LiquidCrystal lcd(UI_DISPLAY_RS_PIN, UI_DISPLAY_RW_PIN,UI_DISPLAY_ENABLE_PIN,UI_DISPLAY_D4_PIN,UI_DISPLAY_D5_PIN,UI_DISPLAY_D6_PIN,UI_DISPLAY_D7_PIN);

void UIDisplay::createChar(uint8_t location,const uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    uint8_t data[8];
    for (int i = 0; i < 8; i++)
    {
        data[i] = pgm_read_byte(&(charmap[i]));
    }
    lcd.createChar(location, data);
}
void UIDisplay::printRow(uint8_t r,char *txt,char *txt2,uint8_t changeAtCol)
{
    changeAtCol = RMath::min(UI_COLS,changeAtCol);
    uint8_t col = 0;
// Set row
    if(r >= UI_ROWS) return;
    lcd.setCursor(0,r);
    char c;
    while((c = *txt) != 0x00 && col < changeAtCol)
    {
        txt++;
        lcd.write(c);
        col++;
    }
    while(col < changeAtCol)
    {
        lcd.write(' ');
        col++;
    }
    if(txt2 != NULL)
    {
        while((c = *txt2) != 0x00 && col < UI_COLS)
        {
            txt2++;
            lcd.write(c);
            col++;
        }
        while(col < UI_COLS)
        {
            lcd.write(' ');
            col++;
        }
    }
#if UI_HAS_KEYS==1 && UI_HAS_I2C_ENCODER>0
    uiCheckSlowEncoder();
#endif
}

void initializeLCD()
{
    lcd.begin(UI_COLS,UI_ROWS);
    uid.lastSwitch = uid.lastRefresh = HAL::timeInMilliseconds();
    uid.createChar(1,character_back);
    uid.createChar(2,character_degree);
    uid.createChar(3,character_selected);
    uid.createChar(4,character_unselected);
}
// ------------------ End LiquidCrystal library as LCD driver
#endif // UI_DISPLAY_TYPE == DISPLAY_ARDUINO_LIB

#if UI_DISPLAY_TYPE == DISPLAY_U8G
//u8glib
#ifdef U8GLIB_ST7920
#define UI_SPI_SCK UI_DISPLAY_D4_PIN
#define UI_SPI_MOSI UI_DISPLAY_ENABLE_PIN
#define UI_SPI_CS UI_DISPLAY_RS_PIN
#endif
#include "u8glib_ex.h"
#include "logo.h"

u8g_t u8g;
u8g_uint_t u8_tx = 0, u8_ty = 0;

void u8PrintChar(char c)
{
    switch(c)
    {
    case 0x7E: // right arrow
        u8g_SetFont(&u8g, u8g_font_6x12_67_75);
        u8_tx += u8g_DrawGlyph(&u8g, u8_tx, u8_ty, 0x52);
        u8g_SetFont(&u8g, UI_FONT_DEFAULT);
        break;
    case CHAR_SELECTOR:
        u8g_SetFont(&u8g, u8g_font_6x12_67_75);
        u8_tx += u8g_DrawGlyph(&u8g, u8_tx, u8_ty, 0xb7);
        u8g_SetFont(&u8g, UI_FONT_DEFAULT);
        break;
    case CHAR_SELECTED:
        u8g_SetFont(&u8g, u8g_font_6x12_67_75);
        u8_tx += u8g_DrawGlyph(&u8g, u8_tx, u8_ty, 0xb6);
        u8g_SetFont(&u8g, UI_FONT_DEFAULT);
        break;
    default:
        u8_tx += u8g_DrawGlyph(&u8g, u8_tx, u8_ty, c);
    }
}
void printU8GRow(uint8_t x,uint8_t y,char *text)
{
    char c;
    while((c = *(text++)) != 0)
        x += u8g_DrawGlyph(&u8g,x,y,c);
}
void UIDisplay::printRow(uint8_t r,char *txt,char *txt2,uint8_t changeAtCol)
{
    changeAtCol = RMath::min(UI_COLS,changeAtCol);
    uint8_t col = 0;
// Set row
    if(r >= UI_ROWS) return;
    int y = r * UI_FONT_HEIGHT;
    if(!u8g_IsBBXIntersection(&u8g,0,y,UI_LCD_WIDTH,UI_FONT_HEIGHT+2)) return; // row not visible
    u8_tx = 0;
    u8_ty = y+UI_FONT_HEIGHT; //set position
    bool highlight = ((uint8_t)(*txt) == CHAR_SELECTOR) || ((uint8_t)(*txt) == CHAR_SELECTED);
    if(highlight)
    {
        u8g_SetColorIndex(&u8g,1);
        u8g_draw_box(&u8g, 0, y + 1, u8g_GetWidth(&u8g), UI_FONT_HEIGHT + 1);
        u8g_SetColorIndex(&u8g, 0);
    }
    char c;
    while((c = *(txt++)) != 0 && col < changeAtCol)
    {
        u8PrintChar(c);
        col++;
    }
    if(txt2 != NULL)
    {
        col = changeAtCol;
        u8_tx = col*UI_FONT_WIDTH; //set position
        while((c=*(txt2++)) != 0 && col < UI_COLS)
        {
            u8PrintChar(c);
            col++;
        }
    }
    if(highlight)
    {
        u8g_SetColorIndex(&u8g,1);
    }

#if UI_HAS_KEYS==1 && UI_HAS_I2C_ENCODER>0
    uiCheckSlowEncoder();
#endif
}

void initializeLCD()
{
#ifdef U8GLIB_ST7920
    u8g_InitSPI(&u8g,&u8g_dev_st7920_128x64_sw_spi,  UI_DISPLAY_D4_PIN, UI_DISPLAY_ENABLE_PIN, UI_DISPLAY_RS_PIN, U8G_PIN_NONE, U8G_PIN_NONE);
#endif
    u8g_Begin(&u8g);
    u8g_FirstPage(&u8g);
    do
    {
        u8g_SetColorIndex(&u8g, 0);
    }
    while( u8g_NextPage(&u8g) );

    u8g_SetFont(&u8g, UI_FONT_DEFAULT);
    u8g_SetColorIndex(&u8g, 1);
    uid.lastSwitch = uid.lastRefresh = HAL::timeInMilliseconds();
}
// ------------------ End u8GLIB library as LCD driver
#endif // UI_DISPLAY_TYPE == DISPLAY_U8G

#if UI_DISPLAY_TYPE == DISPLAY_GAMEDUINO2
#include "gameduino2.h"
#endif

UIDisplay::UIDisplay()
{
}
#if UI_ANIMATION
void slideIn(uint8_t row,FSTRINGPARAM(text))
{
    char *empty="";
    int8_t i = 0;
    uid.col=0;
    uid.addStringP(text);
    uid.printCols[uid.col]=0;
    for(i=UI_COLS-1; i>=0; i--)
    {
        uid.printRow(row,empty,uid.printCols,i);
        HAL::pingWatchdog();
        HAL::delayMilliseconds(10);
    }
}
#endif // UI_ANIMATION
void UIDisplay::initialize()
{
    oldMenuLevel = -2;
#ifdef COMPILE_I2C_DRIVER
    uid.outputMask = UI_DISPLAY_I2C_OUTPUT_START_MASK;
#if UI_DISPLAY_I2C_CHIPTYPE==0 && BEEPER_TYPE==2 && BEEPER_PIN>=0
#if BEEPER_ADDRESS == UI_DISPLAY_I2C_ADDRESS
    uid.outputMask |= BEEPER_PIN;
#endif
#endif
    HAL::i2cInit(UI_I2C_CLOCKSPEED);
#if UI_DISPLAY_I2C_CHIPTYPE==1
    // set direction of pins
    HAL::i2cStart(UI_DISPLAY_I2C_ADDRESS + I2C_WRITE);
    HAL::i2cWrite(0); // IODIRA
    HAL::i2cWrite(~(UI_DISPLAY_I2C_OUTPUT_PINS & 255));
    HAL::i2cWrite(~(UI_DISPLAY_I2C_OUTPUT_PINS >> 8));
    HAL::i2cStop();
    // Set pullups according to  UI_DISPLAY_I2C_PULLUP
    HAL::i2cStart(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
    HAL::i2cWrite(0x0C); // GPPUA
    HAL::i2cWrite(UI_DISPLAY_I2C_PULLUP & 255);
    HAL::i2cWrite(UI_DISPLAY_I2C_PULLUP >> 8);
    HAL::i2cStop();
#endif

#endif
    flags = 0;
    menuLevel = 0;
    shift = -2;
    menuPos[0] = 0;
    lastAction = 0;
    delayedAction = 0;
    lastButtonAction = 0;
    activeAction = 0;
    statusMsg[0] = 0;
    uiInitKeys();
    cwd[0] = '/';
    cwd[1] = 0;
    folderLevel = 0;
    UI_STATUS(UI_TEXT_PRINTER_READY);
#if UI_DISPLAY_TYPE != NO_DISPLAY
    initializeLCD();
#if defined(USER_KEY1_PIN) && USER_KEY1_PIN > -1
    UI_KEYS_INIT_BUTTON_LOW(USER_KEY1_PIN);
#endif
#if defined(USER_KEY2_PIN) && USER_KEY2_PIN > -1
    UI_KEYS_INIT_BUTTON_LOW(USER_KEY2_PIN);
#endif
#if defined(USER_KEY3_PIN) && USER_KEY3_PIN > -1
    UI_KEYS_INIT_BUTTON_LOW(USER_KEY3_PIN);
#endif
#if defined(USER_KEY4_PIN) && USER_KEY4_PIN > -1
    UI_KEYS_INIT_BUTTON_LOW(USER_KEY4_PIN);
#endif
#if UI_DISPLAY_TYPE == DISPLAY_I2C
    // I don't know why but after power up the lcd does not come up
    // but if I reinitialize i2c and the lcd again here it works.
    HAL::delayMilliseconds(10);
    HAL::i2cInit(UI_I2C_CLOCKSPEED);
    // set direction of pins
    HAL::i2cStart(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
    HAL::i2cWrite(0); // IODIRA
    HAL::i2cWrite(~(UI_DISPLAY_I2C_OUTPUT_PINS & 255));
    HAL::i2cWrite(~(UI_DISPLAY_I2C_OUTPUT_PINS >> 8));
    HAL::i2cStop();
    // Set pullups according to  UI_DISPLAY_I2C_PULLUP
    HAL::i2cStart(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
    HAL::i2cWrite(0x0C); // GPPUA
    HAL::i2cWrite(UI_DISPLAY_I2C_PULLUP & 255);
    HAL::i2cWrite(UI_DISPLAY_I2C_PULLUP >> 8);
    HAL::i2cStop();
    initializeLCD();
#endif
#if UI_DISPLAY_TYPE == DISPLAY_GAMEDUINO2
    GD2::startScreen();
#else
#if UI_ANIMATION==false || UI_DISPLAY_TYPE == DISPLAY_U8G
#if UI_DISPLAY_TYPE == DISPLAY_U8G
    //u8g picture loop
    u8g_FirstPage(&u8g);
    do
    {
        u8g_DrawBitmapP(&u8g, 128 - LOGO_WIDTH, 0, ((LOGO_WIDTH + 8) / 8), LOGO_HEIGHT, logo);
        for(uint8_t y = 0; y < UI_ROWS; y++) displayCache[y][0] = 0;
        printRowP(0, PSTR("Repetier"));
        printRowP(1, PSTR("Ver " REPETIER_VERSION));
        printRowP(3, PSTR("Machine:"));
        printRowP(4, PSTR(UI_PRINTER_NAME));
        printRowP(5, PSTR(UI_PRINTER_COMPANY));
    }
    while( u8g_NextPage(&u8g) );  //end picture loop
#else // not DISPLAY_U8G
    for(uint8_t y=0; y<UI_ROWS; y++) displayCache[y][0] = 0;
    printRowP(0, versionString);
    printRowP(1, PSTR(UI_PRINTER_NAME));
#if UI_ROWS>2
    printRowP(UI_ROWS-1, PSTR(UI_PRINTER_COMPANY));
#endif
#endif
#else
    slideIn(0, versionString);
    strcpy(displayCache[0], uid.printCols);
    slideIn(1, PSTR(UI_PRINTER_NAME));
    strcpy(displayCache[1], uid.printCols);
#if UI_ROWS>2
    slideIn(UI_ROWS-1, PSTR(UI_PRINTER_COMPANY));
    strcpy(displayCache[UI_ROWS-1], uid.printCols);
#endif
#endif
#endif // gameduino2
    HAL::delayMilliseconds(UI_START_SCREEN_DELAY);
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==0 && (BEEPER_TYPE==2 || defined(UI_HAS_I2C_KEYS))
    // Make sure the beeper is off
    HAL::i2cStartWait(UI_I2C_KEY_ADDRESS+I2C_WRITE);
    HAL::i2cWrite(255); // Disable beeper, enable read for other pins.
    HAL::i2cStop();
#endif
}
#if UI_DISPLAY_TYPE == DISPLAY_4BIT || UI_DISPLAY_TYPE == DISPLAY_8BIT || UI_DISPLAY_TYPE == DISPLAY_I2C
void UIDisplay::createChar(uint8_t location,const uint8_t PROGMEM charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    lcdCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i=0; i<8; i++)
    {
        lcdPutChar(pgm_read_byte(&(charmap[i])));
    }
}
#endif
void  UIDisplay::waitForKey()
{
    int nextAction = 0;

    lastButtonAction = 0;
    while(lastButtonAction==nextAction)
    {
        uiCheckSlowKeys(nextAction);
    }
}

void UIDisplay::printRowP(uint8_t r,PGM_P txt)
{
    if(r >= UI_ROWS) return;
    col = 0;
    addStringP(txt);
    uid.printCols[col] = 0;
    printRow(r,uid.printCols,NULL,UI_COLS);
}
void UIDisplay::addInt(int value,uint8_t digits,char fillChar)
{
    uint8_t dig=0,neg=0;
    if(value<0)
    {
        value = -value;
        neg=1;
        dig++;
    }
    char buf[7]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[6];
    buf[6]=0;
    do
    {
        unsigned int m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    }
    while(value);
    if(neg)
        uid.printCols[col++]='-';
    if(digits<6)
        while(dig<digits)
        {
            *--str = fillChar; //' ';
            dig++;
        }
    while(*str && col<MAX_COLS)
    {
        uid.printCols[col++] = *str;
        str++;
    }
}
void UIDisplay::addLong(long value,char digits)
{
    uint8_t dig = 0,neg=0;
    byte addspaces = digits>0;
    if (digits<0) digits = -digits;
    if(value<0)
    {
        neg=1;
        value = -value;
        dig++;
    }
    char buf[13]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[12];
    buf[12]=0;
    do
    {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    }
    while(value);
    if(neg)
        uid.printCols[col++]='-';
    if(addspaces && digits<=11)
        while(dig<digits)
        {
            *--str = ' ';
            dig++;
        }
    while(*str && col<MAX_COLS)
    {
        uid.printCols[col++] = *str;
        str++;
    }
}
const float roundingTable[] PROGMEM = {0.5,0.05,0.005,0.0005};
void UIDisplay::addFloat(float number, char fixdigits,uint8_t digits)
{
    // Handle negative numbers
    if (number < 0.0)
    {
        uid.printCols[col++]='-';
        if(col>=MAX_COLS) return;
        number = -number;
        fixdigits--;
    }
    number += pgm_read_float(&roundingTable[digits]); // for correct rounding

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    float remainder = number - (float)int_part;
    addLong(int_part,fixdigits);
    if(col>=UI_COLS) return;

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0)
    {
        uid.printCols[col++]='.';
    }

    // Extract digits from the remainder one at a time
    while (col<MAX_COLS && digits-- > 0)
    {
        remainder *= 10.0;
        uint8_t toPrint = uint8_t(remainder);
        uid.printCols[col++] = '0'+toPrint;
        remainder -= toPrint;
    }
}
void UIDisplay::addStringP(FSTRINGPARAM(text))
{
    while(col<MAX_COLS)
    {
        uint8_t c = HAL::readFlashByte(text++);
        if(c==0) return;
        uid.printCols[col++]=c;
    }
}
void UIDisplay::addChar(const char c)
{
    if(col<UI_COLS)
    {
        uid.printCols[col++]=c;
    }
}
void UIDisplay::addGCode(GCode *code)
{
    // assume volatile and make copy so we dont "see" multple code lines as we go.
    //GCode myCode = *code; insuffeicnet memory for this safety check
    //code = &myCode;
    addChar('#');
    addLong(code->N);
    if(code->hasM())
    {
        addChar('M');
        addLong((long)code->M);
    }
    if(code->hasG())
    {
        addChar('G');
        addLong((long)code->G);
    }
    if(code->hasT())
    {
        addChar('T');
        addLong((long)code->T);
    }
    if(code->hasX())
    {
        addChar('X');
        addFloat(code->X);
    }
    if(code->hasY())
    {
        addChar('Y');
        addFloat(code->Y);
    }
    if(code->hasZ())
    {
        addChar('Z');
        addFloat(code->Z);
    }
    if(code->hasE())
    {
        addChar('E');
        addFloat(code->E);
    }
    if(code->hasF())
    {
        addChar('F');
        addFloat(code->F);
    }
    if(code->hasS())
    {
        addChar('S');
        addLong(code->S);
    }
    if(code->hasP())
    {
        addChar('P');
        addLong(code->P);
    }
#ifdef ARC_SUPPORT
    if(code->hasI())
    {
        addChar('I');
        addFloat(code->I);
    }
    if(code->hasJ())
    {
        addChar('J');
        addFloat(code->J);
    }
    if(code->hasR())
    {
        addChar('R');
        addFloat(code->R);
    }
#endif
    // cannot print string, it isnt part of the gcode structure.
    //it points to temp memory in a buffer.
    //if(code->hasSTRING())
}

UI_STRING(ui_text_on,UI_TEXT_ON);
UI_STRING(ui_text_off,UI_TEXT_OFF);
UI_STRING(ui_text_na,UI_TEXT_NA);
UI_STRING(ui_yes,UI_TEXT_YES);
UI_STRING(ui_no,UI_TEXT_NO);
UI_STRING(ui_print_pos,UI_TEXT_PRINT_POS);
UI_STRING(ui_selected,UI_TEXT_SEL);
UI_STRING(ui_unselected,UI_TEXT_NOSEL);
UI_STRING(ui_action,UI_TEXT_STRING_ACTION);

void UIDisplay::parse(const char *txt,bool ram)
{
    int ivalue=0;
    float fvalue=0;
    while(col<MAX_COLS)
    {
        char c=(ram ? *(txt++) : pgm_read_byte(txt++));
        if(c==0) break; // finished
        if(c!='%')
        {
            uid.printCols[col++]=c;
            continue;
        }
        // dynamic parameter, parse meaning and replace
        char c1=(ram ? *(txt++) : pgm_read_byte(txt++));
        char c2=(ram ? *(txt++) : pgm_read_byte(txt++));
        switch(c1)
        {
        case '%':
        {
            // print % for input '%%' or '%%%'
            if(col<UI_COLS) uid.printCols[col++]='%'; // if data = '%%?' escaped percent, with left over ? char
            if (c2 != '%') txt--; // Be flexible and accept 2 or 3 chars
            break;
        } // case '%'

        case '?' : // conditional spacer or other char
        {
            // If something has been printed, check if the last char is c2.
            // if not, append c2.
            // otherwise do nothing.
            if (col>0 && col<UI_COLS)
            {
                if (uid.printCols[col-1] != c2) uid.printCols[col++]=c2;
            }
            break;
        }
        case 'a': // Acceleration settings
            if(c2=='x') addFloat(Printer::maxAccelerationMMPerSquareSecond[X_AXIS],5,0);
            else if(c2=='y') addFloat(Printer::maxAccelerationMMPerSquareSecond[Y_AXIS],5,0);
            else if(c2=='z') addFloat(Printer::maxAccelerationMMPerSquareSecond[Z_AXIS],5,0);
            else if(c2=='X') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS],5,0);
            else if(c2=='Y') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS],5,0);
            else if(c2=='Z') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS],5,0);
            else if(c2=='j') addFloat(Printer::maxJerk,3,1);
#if DRIVE_SYSTEM!=DELTA
            else if(c2=='J') addFloat(Printer::maxZJerk,3,1);
#endif
            break;

        case 'd':
            if(c2 == 'o') addStringP(Printer::debugEcho()?ui_text_on:ui_text_off);
            else if(c2 == 'i') addStringP(Printer::debugInfo()?ui_text_on:ui_text_off);
            else if(c2 == 'e') addStringP(Printer::debugErrors()?ui_text_on:ui_text_off);
            else if(c2 == 'd') addStringP(Printer::debugDryrun()?ui_text_on:ui_text_off);
            break;

        case 'e': // Extruder temperature
            if(c2 == 'I')
            {
                //give integer display
                char c2=(ram ? *(txt++) : pgm_read_byte(txt++));
                ivalue=0;
            }
            else ivalue = UI_TEMP_PRECISION;

            if(c2 == 'r')   // Extruder relative mode
            {
                addStringP(Printer::relativeExtruderCoordinateMode ? ui_yes : ui_no);
                break;
            }
            {
                uint8_t eid = NUM_EXTRUDER;    // default = BED if c2 not specified extruder number
                if(c2 == 'c') eid = Extruder::current->id;
                else if(c2 >= '0' && c2 <= '9') eid = c2 - '0';
                if(Printer::isAnyTempsensorDefect())
                {

                    if(tempController[eid]->isSensorDefect())
                    {
                        addStringP(PSTR(" def "));
                        break;
                    }
                    else if(tempController[eid]->isSensorDecoupled())
                    {
                        addStringP(PSTR(" dec "));
                        break;
                    }
                }
                if(tempController[eid]->isJammed())
                {
                    addStringP(PSTR(" jam "));
                    break;
                }
                if(c2=='c') fvalue=Extruder::current->tempControl.currentTemperatureC;
                else if(c2>='0' && c2<='9') fvalue=extruder[c2-'0'].tempControl.currentTemperatureC;
                else if(c2=='b') fvalue=Extruder::getHeatedBedTemperature();
                else if(c2=='B')
                {
                    ivalue=0;
                    fvalue=Extruder::getHeatedBedTemperature();
                }
                addFloat(fvalue,3,ivalue);
            }
            break;
        case 'E': // Target extruder temperature
            if(c2=='c') fvalue=Extruder::current->tempControl.targetTemperatureC;
            else if(c2>='0' && c2<='9') fvalue=extruder[c2-'0'].tempControl.targetTemperatureC;
#if HAVE_HEATED_BED
            else if(c2=='b') fvalue=heatedBedController.targetTemperatureC;
#endif
            addFloat(fvalue, 3, 0 /*UI_TEMP_PRECISION*/);
            break;
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
        case 'F': // FAN speed
            if(c2=='s') addInt(floor(Printer::getFanSpeed() * 100 / 255 + 0.5f), 3);
            break;
#endif
        case 'f':
            if(c2 == 'x') addFloat(Printer::maxFeedrate[X_AXIS], 5, 0);
            else if(c2 == 'y') addFloat(Printer::maxFeedrate[Y_AXIS], 5, 0);
            else if(c2 == 'z') addFloat(Printer::maxFeedrate[Z_AXIS], 5, 0);
            else if(c2 == 'X') addFloat(Printer::homingFeedrate[X_AXIS], 5, 0);
            else if(c2 == 'Y') addFloat(Printer::homingFeedrate[Y_AXIS], 5, 0);
            else if(c2 == 'Z') addFloat(Printer::homingFeedrate[Z_AXIS], 5, 0);
            break;
        case 'i':
            if(c2 == 's') addLong(stepperInactiveTime/1000,4);
            else if(c2 == 'p') addLong(maxInactiveTime/1000,4);
            break;
        case 'O': // ops related stuff
            break;
        case 'l':
            if(c2 == 'a') addInt(lastAction,4);
#if defined(CASE_LIGHTS_PIN) && CASE_LIGHTS_PIN >= 0
            else if(c2 == 'o') addStringP(READ(CASE_LIGHTS_PIN) ? ui_text_on : ui_text_off);        // Lights on/off
#endif
#if FEATURE_AUTOLEVEL
            else if(c2 == 'l') addStringP((Printer::isAutolevelActive()) ? ui_text_on : ui_text_off);        // Autolevel on/off
#endif
            break;
        case 'o':
            if(c2=='s')
            {
#if SDSUPPORT
                if(sd.sdactive && sd.sdmode)
                {
                    addStringP(PSTR( UI_TEXT_PRINT_POS));
                    float percent;
                    if(sd.filesize<2000000) percent=sd.sdpos*100.0/sd.filesize;
                    else percent = (sd.sdpos>>8)*100.0/(sd.filesize>>8);
                    addFloat(percent,3,1);
                    if(col<MAX_COLS)
                        uid.printCols[col++]='%';
                }
                else
#endif
                    parse(statusMsg,true);
                break;
            }
            if(c2=='c')
            {
                addLong(baudrate,6);
                break;
            }
            if(c2=='e')
            {
                if(errorMsg!=0)addStringP((char PROGMEM *)errorMsg);
                break;
            }
            if(c2=='B')
            {
                addInt((int)PrintLine::linesCount,2);
                break;
            }
            if(c2=='f')
            {
                addInt(Printer::extrudeMultiply,3);
                break;
            }
            if(c2=='m')
            {
                addInt(Printer::feedrateMultiply, 3);
                break;
            }
            if(c2=='n')
            {
                addInt(Extruder::current->id + 1, 1);
                break;
            }
#if FEATURE_SERVO > 0 && UI_SERVO_CONTROL > 0
            if(c2 == 'S')
            {
                addInt(servoPosition, 4);
                break;
            }
#endif
#if FEATURE_BABYSTEPPING
            if(c2=='Y')
            {
//                addInt(zBabySteps,0);
                addFloat((float)zBabySteps * Printer::invAxisStepsPerMM[Z_AXIS], 2, 2);
                break;
            }
#endif
            // Extruder output level
            if(c2>='0' && c2<='9') ivalue=pwm_pos[c2-'0'];
#if HAVE_HEATED_BED
            else if(c2=='b') ivalue=pwm_pos[heatedBedController.pwmIndex];
#endif
            else if(c2=='C') ivalue=pwm_pos[Extruder::current->id];
            ivalue=(ivalue*100)/255;
            addInt(ivalue,3);
            if(col<MAX_COLS)
                uid.printCols[col++]='%';
            break;
        case 's': // Endstop positions
            if(c2=='x')
            {
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
                addStringP(Printer::isXMinEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            }
            if(c2=='X')
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
                addStringP(Printer::isXMaxEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            if(c2=='y')
#if (Y_MIN_PIN > -1)&& MIN_HARDWARE_ENDSTOP_Y
                addStringP(Printer::isYMinEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            if(c2=='Y')
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
                addStringP(Printer::isYMaxEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            if(c2=='z')
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
                addStringP(Printer::isZMinEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            if(c2=='Z')
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
                addStringP(Printer::isZMaxEndstopHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            if(c2=='P')
#if (Z_PROBE_PIN > -1)
                addStringP(Printer::isZProbeHit()?ui_text_on:ui_text_off);
#else
                addStringP(ui_text_na);
#endif
            break;
        case 'S':
            if(c2=='x') addFloat(Printer::axisStepsPerMM[X_AXIS],3,1);
            if(c2=='y') addFloat(Printer::axisStepsPerMM[Y_AXIS],3,1);
            if(c2=='z') addFloat(Printer::axisStepsPerMM[Z_AXIS],3,1);
            if(c2=='e') addFloat(Extruder::current->stepsPerMM,3,1);
            break;

        case 'U':
            if(c2=='t')   // Printing time
            {
#if EEPROM_MODE
                bool alloff = true;
                for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
                    if(tempController[i]->targetTemperatureC>15) alloff = false;

                long seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
                long tmp = seconds / 86400;
                seconds -= tmp * 86400;
                addInt(tmp, 5);
                addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                tmp = seconds / 3600;
                addInt(tmp,2);
                addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                seconds -= tmp * 3600;
                tmp = seconds / 60;
                addInt(tmp,2,'0');
                addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
#endif
            }
            else if(c2 == 'f')     // Filament usage
            {
#if EEPROM_MODE
                float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
#else
                float dist = Printer::filamentPrinted * 0.001;
#endif
                addFloat(dist, 6, 1);
            }
            break;

        case 'x':
            if(c2>='0' && c2<='4')
            {
                if(c2=='4') // this sequence save 14 bytes of flash
                {
                    addFloat(Printer::filamentPrinted * 0.001,3,2);
                    break;
                }
                if(c2=='0')
                    fvalue = Printer::realXPosition();
                else if(c2=='1')
                    fvalue = Printer::realYPosition();
                else if(c2=='2')
                    fvalue = Printer::realZPosition();
                else
                    fvalue = (float)Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];
                addFloat(fvalue,4,2);
            }
            break;

        case 'X': // Extruder related
#if NUM_EXTRUDER>0
            if(c2>='0' && c2<='9')
            {
                addStringP(Extruder::current->id==c2-'0'?ui_selected:ui_unselected);
            }
#if TEMP_PID
            else if(c2=='i')
            {
                addFloat(Extruder::current->tempControl.pidIGain,4,2);
            }
            else if(c2=='p')
            {
                addFloat(Extruder::current->tempControl.pidPGain,4,2);
            }
            else if(c2=='d')
            {
                addFloat(Extruder::current->tempControl.pidDGain,4,2);
            }
            else if(c2=='m')
            {
                addInt(Extruder::current->tempControl.pidDriveMin,3);
            }
            else if(c2=='M')
            {
                addInt(Extruder::current->tempControl.pidDriveMax,3);
            }
            else if(c2=='D')
            {
                addInt(Extruder::current->tempControl.pidMax,3);
            }
#endif
            else if(c2=='w')
            {
                addInt(Extruder::current->watchPeriod,4);
            }
#if RETRACT_DURING_HEATUP
            else if(c2=='T')
            {
                addInt(Extruder::current->waitRetractTemperature,4);
            }
            else if(c2=='U')
            {
                addInt(Extruder::current->waitRetractUnits,2);
            }
#endif
            else if(c2=='h')
            {
                uint8_t hm = Extruder::current->tempControl.heatManager;
                if(hm == HTR_PID)
                    addStringP(PSTR(UI_TEXT_STRING_HM_PID));
                else if(hm == HTR_DEADTIME)
                    addStringP(PSTR(UI_TEXT_STRING_HM_DEADTIME));
                else if(hm == HTR_SLOWBANG)
                    addStringP(PSTR(UI_TEXT_STRING_HM_SLOWBANG));
                else
                    addStringP(PSTR(UI_TEXT_STRING_HM_BANGBANG));
            }
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
            else if(c2=='a')
            {
                addFloat(Extruder::current->advanceK,3,0);
            }
#endif
            else if(c2=='l')
            {
                addFloat(Extruder::current->advanceL,3,0);
            }
#endif
            else if(c2=='x')
            {
                addFloat(Extruder::current->xOffset,4,2);
            }
            else if(c2=='y')
            {
                addFloat(Extruder::current->yOffset,4,2);
            }
            else if(c2=='f')
            {
                addFloat(Extruder::current->maxStartFeedrate,5,0);
            }
            else if(c2=='F')
            {
                addFloat(Extruder::current->maxFeedrate,5,0);
            }
            else if(c2=='A')
            {
                addFloat(Extruder::current->maxAcceleration,5,0);
            }
#endif
            break;
        case 'y':
#if DRIVE_SYSTEM==DELTA
            if(c2>='0' && c2<='3') fvalue = (float)Printer::currentDeltaPositionSteps[c2-'0']*Printer::invAxisStepsPerMM[c2-'0'];
            addFloat(fvalue,3,2);
#endif
            break;
        }
    }
    uid.printCols[col] = 0;
}
void UIDisplay::setStatusP(PGM_P txt,bool error)
{
    if(!error && Printer::isUIErrorMessage()) return;
    uint8_t i=0;
    while(i<20)
    {
        uint8_t c = pgm_read_byte(txt++);
        if(!c) break;
        statusMsg[i++] = c;
    }
    statusMsg[i]=0;
    if(error)
        Printer::setUIErrorMessage(true);
}
void UIDisplay::setStatus(const char *txt,bool error)
{
    if(!error && Printer::isUIErrorMessage()) return;
    uint8_t i=0;
    while(*txt && i<20)
        statusMsg[i++] = *txt++;
    statusMsg[i]=0;
    if(error)
        Printer::setUIErrorMessage(true);
}

const UIMenu * const ui_pages[UI_NUM_PAGES] PROGMEM = UI_PAGES;
uint16_t nFilesOnCard;
void UIDisplay::updateSDFileCount()
{
#if SDSUPPORT
    dir_t* p = NULL;
    byte offset = menuTop[menuLevel];
    SdBaseFile *root = sd.fat.vwd();

    root->rewind();
    nFilesOnCard = 0;
    while ((p = root->getLongFilename(p, NULL, 0, NULL)))
    {
        if (! (DIR_IS_FILE(p) || DIR_IS_SUBDIR(p)))
            continue;
        if (folderLevel>=SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.'))
            continue;
        nFilesOnCard++;
        if (nFilesOnCard > 5000) // Arbitrary maximum, limited only by how long someone would scroll
            return;
    }
#endif
}

void getSDFilenameAt(uint16_t filePos,char *filename)
{
#if SDSUPPORT
    dir_t* p;
    SdBaseFile *root = sd.fat.vwd();

    root->rewind();
    while ((p = root->getLongFilename(p, tempLongFilename, 0, NULL)))
    {
        HAL::pingWatchdog();
        if (!DIR_IS_FILE(p) && !DIR_IS_SUBDIR(p)) continue;
        if(uid.folderLevel>=SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.')) continue;
        if (filePos--)
            continue;
        strcpy(filename, tempLongFilename);
        if(DIR_IS_SUBDIR(p)) strcat(filename, "/"); // Set marker for directory
        break;
    }
#endif
}

bool UIDisplay::isDirname(char *name)
{
    while(*name) name++;
    name--;
    return *name=='/';
}

void UIDisplay::goDir(char *name)
{
#if SDSUPPORT
    char *p = cwd;
    while(*p)p++;
    if(name[0]=='.' && name[1]=='.')
    {
        if(folderLevel==0) return;
        p--;
        p--;
        while(*p!='/') p--;
        p++;
        *p = 0;
        folderLevel--;
    }
    else
    {
        if(folderLevel>=SD_MAX_FOLDER_DEPTH) return;
        while(*name) *p++ = *name++;
        *p = 0;
        folderLevel++;
    }
    sd.fat.chdir(cwd);
    updateSDFileCount();
#endif
}

void sdrefresh(uint16_t &r,char cache[UI_ROWS][MAX_COLS+1])
{
#if SDSUPPORT
    dir_t* p = NULL;
    uint16_t offset = uid.menuTop[uid.menuLevel];
    SdBaseFile *root;
    uint16_t length, skip;

    sd.fat.chdir(uid.cwd);
    root = sd.fat.vwd();
    root->rewind();

    skip = (offset > 0 ? offset - 1 : 0);

    while (r + offset < nFilesOnCard + 1 && r < UI_ROWS && (p = root->getLongFilename(p, tempLongFilename, 0, NULL)))
    {
        HAL::pingWatchdog();
        // done if past last used entry
        // skip deleted entry and entries for . and  ..
        // only list subdirectories and files
        if ((DIR_IS_FILE(p) || DIR_IS_SUBDIR(p)))
        {
            if(uid.folderLevel >= SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.'))
                continue;
            if(skip > 0)
            {
                skip--;
                continue;
            }
            uid.col = 0;
            if(r + offset == uid.menuPos[uid.menuLevel])
                uid.printCols[uid.col++] = CHAR_SELECTOR;
            else
                uid.printCols[uid.col++] = ' ';
            // print file name with possible blank fill
            if(DIR_IS_SUBDIR(p))
                uid.printCols[uid.col++] = bFOLD; // Prepend folder symbol
            length = RMath::min((int)strlen(tempLongFilename), MAX_COLS - uid.col);
            memcpy(uid.printCols + uid.col, tempLongFilename, length);
            uid.col += length;
            uid.printCols[uid.col] = 0;
            strcpy(cache[r++],uid.printCols);
        }
    }
#endif
}

// Refresh current menu page
void UIDisplay::refreshPage()
{
#if  UI_DISPLAY_TYPE == DISPLAY_GAMEDUINO2
    GD2::refresh();
#else
    uint16_t r;
    uint8_t mtype;
    char cache[UI_ROWS][MAX_COLS+1];
    adjustMenuPos();
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    // Reset timeout on menu back when user active on menu
    if (uid.encoderLast != encoderStartScreen)
        ui_autoreturn_time=HAL::timeInMilliseconds()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
    encoderStartScreen = uid.encoderLast;

    // Copy result into cache
    if(menuLevel == 0) // Top level menu
    {
        UIMenu *men = (UIMenu*)pgm_read_word(&(ui_pages[menuPos[0]]));
        uint16_t nr = pgm_read_word_near(&(men->numEntries));
        UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        for(r = 0; r < nr && r < UI_ROWS; r++)
        {
            UIMenuEntry *ent = (UIMenuEntry *)pgm_read_word(&(entries[r]));
            col = 0;
            parse((char*)pgm_read_word(&(ent->text)),false);
            strcpy(cache[r],uid.printCols);
        }
    }
    else
    {
        UIMenu *men = (UIMenu*)menu[menuLevel];
        uint16_t nr = pgm_read_word_near((void*)&(men->numEntries));
        mtype = pgm_read_byte((void*)&(men->menuType));
        uint16_t offset = menuTop[menuLevel];
        UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        for(r = 0; r + offset < nr && r < UI_ROWS; )
        {
            UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[r+offset]));
            if(!ent->showEntry())
            {
                offset++;
                continue;
            }
            uint8_t entType = pgm_read_byte(&(ent->menuType));
            uint16_t entAction = pgm_read_word(&(ent->action));
            col = 0;
            if(entType >= 2 && entType <= 4)
            {
                if(r + offset == menuPos[menuLevel] && activeAction != entAction)
                    uid.printCols[col++] = CHAR_SELECTOR;
                else if(activeAction == entAction)
                    uid.printCols[col++] = CHAR_SELECTED;
                else
                    uid.printCols[col++]=' ';
            }
            parse((char*)pgm_read_word(&(ent->text)),false);
            if(entType == 2)   // Draw submenu marker at the right side
            {
                while(col<UI_COLS-1) uid.printCols[col++]=' ';
                if(col>UI_COLS)
                {
                    uid.printCols[RMath::min(UI_COLS-1,col)] = CHAR_RIGHT;
                }
                else
                    uid.printCols[col] = CHAR_RIGHT; // Arrow right
                uid.printCols[++col] = 0;
            }
            strcpy(cache[r],uid.printCols);
            r++;
        }
    }
#if SDSUPPORT
    if(mtype == UI_MENU_TYPE_FILE_SELECTOR)
    {
        sdrefresh(r,cache);
    }
#endif

    uid.printCols[0] = 0;
    while(r < UI_ROWS) // delete trailing empty rows
        strcpy(cache[r++],uid.printCols);
    // cache now contains the data to show
    // Compute transition
    uint8_t transition = 0; // 0 = display, 1 = up, 2 = down, 3 = left, 4 = right
#if UI_ANIMATION
    if(menuLevel != oldMenuLevel && !PrintLine::hasLines())
    {
        if(oldMenuLevel == 0 || oldMenuLevel == -2)
            transition = 1;
        else if(menuLevel == 0)
            transition = 2;
        else if(menuLevel>oldMenuLevel)
            transition = 3;
        else
            transition = 4;
    }
#endif
    uint8_t loops = 1;
    uint8_t dt = 1,y;
    if(transition == 1 || transition == 2) loops = UI_ROWS;
    else if(transition > 2)
    {
        dt = (UI_COLS + UI_COLS - 1) / 16;
        loops = UI_COLS + 1 / dt;
    }
    uint8_t off0 = (shift <= 0 ? 0 : shift);
    uint8_t scroll = dt;
    uint8_t off[UI_ROWS];
    if(transition == 0) // Copy cache to displayCache
    {
        for(y = 0; y < UI_ROWS; y++)
            strcpy(displayCache[y],cache[y]);
    }
    for(y = 0; y < UI_ROWS; y++)
    {
        uint8_t len = strlen(displayCache[y]); // length of line content
        off[y] = len > UI_COLS ? RMath::min(len - UI_COLS,off0) : 0;
        if(len > UI_COLS)
        {
            off[y] = RMath::min(len - UI_COLS,off0);
            if(transition == 0 && (mtype == UI_MENU_TYPE_FILE_SELECTOR || mtype == UI_MENU_TYPE_SUBMENU))  // Copy first char to front
            {
                //displayCache[y][off[y]] = displayCache[y][0];
                cache[y][off[y]] = cache[y][0];
            }
        }
        else off[y] = 0;
#if UI_ANIMATION
        if(transition == 3)
        {
            for(r = len; r < MAX_COLS; r++)
            {
                displayCache[y][r] = 32;
            }
            displayCache[y][MAX_COLS] = 0;
        }
        else if(transition == 4)
        {
            for(r = strlen(cache[y]); r < MAX_COLS; r++)
            {
                cache[y][r] = 32;
            }
            cache[y][MAX_COLS] = 0;
        }
#endif
    }
    for(uint8_t l = 0; l<loops; l++)
    {
        if(uid.encoderLast != encoderStartScreen)
        {
            scroll = 200;
        }
        scroll += dt;
#if UI_DISPLAY_TYPE == DISPLAY_U8G
#define drawHProgressBar(x,y,width,height,progress) \
     {u8g_DrawFrame(&u8g,x,y, width, height);  \
     int p = ceil((width-2) * progress / 100); \
     u8g_DrawBox(&u8g,x+1,y+1, p, height-2);}


#define drawVProgressBar(x,y,width,height,progress) \
     {u8g_DrawFrame(&u8g,x,y, width, height);  \
     int p = height-1 - ceil((height-2) * progress / 100); \
     u8g_DrawBox(&u8g,x+1,y+p, width-2, (height-p));}
#if UI_DISPLAY_TYPE == DISPLAY_U8G
#if SDSUPPORT
        unsigned long sdPercent;
#endif
        //fan
        int fanPercent;
        char fanString[2];
        if(menuLevel == 0 && menuPos[0] == 0 ) // Main menu with special graphics
        {
//ext1 and ext2 animation symbols
//            if(extruder[0].tempControl.targetTemperatureC > 0)
            if(pwm_pos[extruder[0].tempControl.pwmIndex] > 0)
                cache[0][0] = Printer::isAnimation()?'\x08':'\x09';
            else
                cache[0][0] = '\x0a'; //off
#if NUM_EXTRUDER>1
//            if(extruder[1].tempControl.targetTemperatureC > 0)
            if(pwm_pos[extruder[1].tempControl.pwmIndex] > 0)
                cache[1][0] = Printer::isAnimation()?'\x08':'\x09';
            else
#endif
                cache[1][0] = '\x0a'; //off
#if HAVE_HEATED_BED

            //heatbed animated icons
//            if(heatedBedController.targetTemperatureC > 0)
            if(pwm_pos[heatedBedController.pwmIndex] > 0)
                cache[2][0] = Printer::isAnimation()?'\x0c':'\x0d';
            else
                cache[2][0] = '\x0b';
#endif
            //fan
            fanPercent = Printer::getFanSpeed()*100/255;
            fanString[1]=0;
            if(fanPercent > 0)  //fan running anmation
            {
                fanString[0] = Printer::isAnimation() ? '\x0e' : '\x0f';
            }
            else
            {
                fanString[0] = '\x0e';
            }
#if SDSUPPORT
            //SD Card
            if(sd.sdactive)
            {
                if(sd.sdactive && sd.sdmode)
                {
                    if(sd.filesize<20000000) sdPercent=sd.sdpos*100/sd.filesize;
                    else sdPercent = (sd.sdpos>>8)*100/(sd.filesize>>8);
                }
                else
                {
                    sdPercent = 0;
                }
            }
#endif
        }
#endif
        //u8g picture loop
        u8g_FirstPage(&u8g);
        do
        {
#endif
            if(transition == 0)
            {
#if UI_DISPLAY_TYPE == DISPLAY_U8G

                if(menuLevel==0 && menuPos[0] == 0 )
                {
                    u8g_SetFont(&u8g,UI_FONT_SMALL);
                    uint8_t py = 8;
                    for(uint8_t r=0; r<3; r++)
                    {
                        if(u8g_IsBBXIntersection(&u8g, 0, py-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                            printU8GRow(0,py,cache[r]);
                        py+=10;
                    }
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
                    //fan
                    if(u8g_IsBBXIntersection(&u8g, 0, 30-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                        printU8GRow(117,30,fanString);
                    drawVProgressBar(116, 0, 9, 20, fanPercent);
                    if(u8g_IsBBXIntersection(&u8g, 0, 42-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                        printU8GRow(0,42,cache[3]); //mul + extruded
                    if(u8g_IsBBXIntersection(&u8g, 0, 52-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                        printU8GRow(0,52,cache[4]); //buf
#endif
#if SDSUPPORT
                    //SD Card
                    if(sd.sdactive && u8g_IsBBXIntersection(&u8g, 70, 52-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                    {
                        printU8GRow(70,52,"SD");
                        drawHProgressBar(83,46, 40, 6, sdPercent);
                    }
#endif
                    //Status
                    py = u8g_GetHeight(&u8g)-2;
                    if(u8g_IsBBXIntersection(&u8g, 70, py-UI_FONT_SMALL_HEIGHT, 1, UI_FONT_SMALL_HEIGHT))
                        printU8GRow(0,py,cache[5]);

                    //divider lines
                    u8g_DrawHLine(&u8g,0, 32, u8g_GetWidth(&u8g));
                    if ( u8g_IsBBXIntersection(&u8g, 55, 0, 1, 32) )
                    {
                        u8g_draw_vline(&u8g,112, 0, 32);
                        u8g_draw_vline(&u8g,62, 0, 32);
                    }
                    u8g_SetFont(&u8g, UI_FONT_DEFAULT);
                }
                else
                {
#endif
                    for(y = 0; y < UI_ROWS; y++)
                        printRow(y,&cache[y][off[y]],NULL,UI_COLS);
#if UI_DISPLAY_TYPE == DISPLAY_U8G
                }
#endif
            }
#if UI_ANIMATION
            else
            {
                if(transition == 1)   // up
                {
                    if(scroll > UI_ROWS)
                    {
                        scroll = UI_ROWS;
                        l = loops;
                    }
                    for(y=0; y<UI_ROWS-scroll; y++)
                    {
                        r = y+scroll;
                        printRow(y,&displayCache[r][off[r]],NULL,UI_COLS);
                    }
                    for(y=0; y<scroll; y++)
                    {
                        printRow(UI_ROWS-scroll+y,cache[y],NULL,UI_COLS);
                    }
                }
                else if(transition == 2)     // down
                {
                    if(scroll > UI_ROWS)
                    {
                        scroll = UI_ROWS;
                        l = loops;
                    }
                    for(y=0; y<scroll; y++)
                    {
                        printRow(y,cache[UI_ROWS-scroll+y],NULL,UI_COLS);
                    }
                    for(y=0; y<UI_ROWS-scroll; y++)
                    {
                        r = y+scroll;
                        printRow(y+scroll,&displayCache[y][off[y]],NULL,UI_COLS);
                    }
                }
                else if(transition == 3)     // left
                {
                    if(scroll > UI_COLS)
                    {
                        scroll = UI_COLS;
                        l = loops;
                    }
                    for(y=0; y<UI_ROWS; y++)
                    {
                        printRow(y,&displayCache[y][off[y]+scroll],cache[y],UI_COLS-scroll);
                    }
                }
                else     // right
                {
                    if(scroll > UI_COLS)
                    {
                        scroll = UI_COLS;
                        l = loops;
                    }
                    for(y=0; y<UI_ROWS; y++)
                    {
                        printRow(y,cache[y] + UI_COLS - scroll,&displayCache[y][off[y]],scroll);
                    }
                }
#if DISPLAY_TYPE != 5
                HAL::delayMilliseconds(transition<3 ? 200 : 70);
#endif
                HAL::pingWatchdog();
            }
#endif
#if UI_DISPLAY_TYPE == DISPLAY_U8G
        }
        while( u8g_NextPage(&u8g) );  //end picture loop
        Printer::toggleAnimation();
#endif
    } // for l
#if UI_ANIMATION
    // copy to last cache
    if(transition != 0)
        for(y=0; y<UI_ROWS; y++)
            strcpy(displayCache[y],cache[y]);
    oldMenuLevel = menuLevel;
#endif
#endif
}

void UIDisplay::pushMenu(const UIMenu *men,bool refresh)
{
    if(men == menu[menuLevel])
    {
        refreshPage();
        return;
    }
    if(menuLevel == 4) return; // Max. depth reached. No more memory to down further.
    menuLevel++;
    menu[menuLevel] = men;
    menuTop[menuLevel] = menuPos[menuLevel] = 0;
#if SDSUPPORT
    UIMenu *men2 = (UIMenu*)menu[menuLevel];
    if(pgm_read_byte(&(men2->menuType)) == 1)
    {
        // Menu is Open files list
        updateSDFileCount();
        // Keep menu positon in file list, more user friendly.
        // If file list changed, still need to reset position.
        if (menuPos[menuLevel] > nFilesOnCard)
        {
            //This exception can happen if the card was unplugged or modified.
            menuTop[menuLevel] = 0;
            menuPos[menuLevel] = UI_MENU_BACKCNT; // if top entry is back, default to next useful item
        }
    }
    else
#endif
    {
        // With or without SDCARD, being here means the menu is not a files list
        // Reset menu to top
        menuTop[menuLevel] = 0;
        menuPos[menuLevel] = UI_MENU_BACKCNT; // if top entry is back, default to next useful item
    }
    if(refresh)
        refreshPage();
}
void UIDisplay::popMenu(bool refresh)
{
    if(menuLevel > 0) menuLevel--;
    Printer::setAutomount(false);
    activeAction = 0;
    if(refresh)
        refreshPage();
}
int UIDisplay::okAction(bool allowMoves)
{
    if(Printer::isUIErrorMessage())
    {
        Printer::setUIErrorMessage(false);
        return 0;
    }
#if UI_HAS_KEYS == 1
    if(menuLevel == 0)   // Enter menu
    {
        menuLevel = 1;
        menuTop[1] = 0;
        menuPos[1] =  UI_MENU_BACKCNT; // if top entry is back, default to next useful item
        menu[1] = &ui_menu_main;
        BEEP_SHORT
        return 0;
    }
    UIMenu *men = (UIMenu*)menu[menuLevel];
    //uint8_t nr = pgm_read_word_near(&(menu->numEntries));
    uint8_t mtype = pgm_read_byte(&(men->menuType));
    UIMenuEntry **entries;
    UIMenuEntry *ent;
    unsigned char entType;
    int action;
#if SDSUPPORT
    if(mtype == UI_MENU_TYPE_FILE_SELECTOR)
    {
        if(menuPos[menuLevel] == 0)   // Selected back instead of file
        {
            return executeAction(UI_ACTION_BACK, allowMoves);
        }

        if(!sd.sdactive)
            return 0;
        uint8_t filePos = menuPos[menuLevel] - 1;
        char filename[LONG_FILENAME_LENGTH + 1];

        getSDFilenameAt(filePos, filename);
        if(isDirname(filename))   // Directory change selected
        {
            goDir(filename);
            menuTop[menuLevel] = 0;
            menuPos[menuLevel] = 1;
            refreshPage();
            oldMenuLevel = -1;
            return 0;
        }

        int16_t shortAction; // renamed to avoid scope confusion
        if (Printer::isAutomount())
            shortAction = UI_ACTION_SD_PRINT;
        else
        {
            men = menu[menuLevel-1];
            entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
            ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel-1]]));
            shortAction = pgm_read_word(&(ent->action));
        }
        sd.file.close();
        sd.fat.chdir(cwd);
        switch(shortAction)
        {
        case UI_ACTION_SD_PRINT:
            if (sd.selectFile(filename, false))
            {
                sd.startPrint();
                BEEP_LONG;
                menuLevel = 0;
            }
            break;
        case UI_ACTION_SD_DELETE:
            if(sd.sdactive)
            {
                sd.sdmode = 0;
                sd.file.close();
                if(sd.fat.remove(filename))
                {
                    Com::printFLN(Com::tFileDeleted);
                    BEEP_LONG
                }
                else
                {
                    Com::printFLN(Com::tDeletionFailed);
                }
            }
            break;
        }
        return 0;
    }
#endif
    entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
    entType = pgm_read_byte(&(ent->menuType));// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action
    action = pgm_read_word(&(ent->action));
    if(mtype == UI_MENU_TYPE_MODIFICATION_MENU)   // action menu
    {
        action = pgm_read_word(&(men->id));
        finishAction(action);
        return executeAction(UI_ACTION_BACK, true);
    }
    if(mtype == UI_MENU_TYPE_SUBMENU && entType == 4)   // Modify action
    {
        if(activeAction)   // finish action
        {
            finishAction(action);
            activeAction = 0;
        }
        else
            activeAction = action;
        return 0;
    }
    if(mtype == UI_MENU_TYPE_WIZARD)
    {
        action = pgm_read_word(&(men->id));
        switch(action)
        {
#if FEATURE_RETRACTION
        case UI_ACTION_WIZARD_FILAMENTCHANGE: // filament change is finished
            Extruder::current->retractDistance(EEPROM_FLOAT(RETRACTION_LENGTH));
#if FILAMENTCHANGE_REHOME
#if Z_HOME_DIR > 0
            Printer::homeAxis(true,true,FILAMENTCHANGE_REHOME == 2);
#else
            Printer::homeAxis(true,true,false);
#endif
#endif
            Printer::GoToMemoryPosition(true,true,false,false,Printer::homingFeedrate[X_AXIS]);
            Printer::GoToMemoryPosition(false,false,true,false,Printer::homingFeedrate[Z_AXIS]);
            Extruder::current->retractDistance(-EEPROM_FLOAT(RETRACTION_LENGTH));
            Printer::currentPositionSteps[E_AXIS] = Printer::popWizardVar().l; // set e to starting position
            popMenu(true);
            Printer::setBlockingReceive(false);
            break;
#endif
        }
        return 0;
    }
    if(entType == 2)   // Enter submenu
    {
        pushMenu((UIMenu*)action, false);
        BEEP_SHORT
#if FEATURE_BABYSTEPPING
        zBabySteps = 0;
#endif
        return 0;
    }
    if(entType == 3)
    {
        return executeAction(action, allowMoves);
    }
    return executeAction(UI_ACTION_BACK, allowMoves);
#endif
}

#define INCREMENT_MIN_MAX(a,steps,_min,_max) if ( (increment<0) && (_min>=0) && (a<_min-increment*steps) ) {a=_min;} else { a+=increment*steps; if(a<_min) a=_min; else if(a>_max) a=_max;};

void UIDisplay::adjustMenuPos()
{
    if(menuLevel == 0) return;
    UIMenu *men = (UIMenu*)menu[menuLevel];
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    uint8_t mtype = HAL::readFlashByte((PGM_P)&(men->menuType));
    int numEntries = pgm_read_word(&(men->numEntries));
    if(mtype != 2) return;
    UIMenuEntry *entry;
    while(menuPos[menuLevel] > 0) // Go up until we reach visible position
    {
        entry = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
        if(pgm_read_byte((void*)&(entry->menuType)) == 1) // skip headlines
            menuPos[menuLevel]--;
        else if(entry->showEntry())
            break;
        else
            menuPos[menuLevel]--;
    }

    // with bad luck the only visible option was in the opposite direction
    while(menuPos[menuLevel] < numEntries - 1) // Go down until we reach visible position
    {
        entry = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
        if(pgm_read_byte((void*)&(entry->menuType)) == 1) // skip headlines
            menuPos[menuLevel]++;
        else if(entry->showEntry())
            break;
        else
            menuPos[menuLevel]++;
    }

    uint8_t skipped = 0;
    bool modified;
    if(menuTop[menuLevel] > menuPos[menuLevel])
        menuTop[menuLevel] = menuPos[menuLevel];
    do
    {
        skipped = 0;
        modified = false;
        for(uint8_t r = menuTop[menuLevel]; r < menuPos[menuLevel]; r++)
        {
            UIMenuEntry *ent = (UIMenuEntry *)pgm_read_word(&(entries[r]));
            if(!ent->showEntry())
                skipped++;
        }
        if(menuTop[menuLevel] + skipped + UI_ROWS - 1 < menuPos[menuLevel])
        {
            menuTop[menuLevel] = menuPos[menuLevel] + 1 - UI_ROWS;
            modified = true;
        }
    }
    while(modified);
}

bool UIDisplay::isWizardActive()
{
    UIMenu *men = (UIMenu*)menu[menuLevel];
    return HAL::readFlashByte((PGM_P)&(men->menuType)) == 5;
}

bool UIDisplay::nextPreviousAction(int16_t next, bool allowMoves)
{
    if(Printer::isUIErrorMessage())
    {
        Printer::setUIErrorMessage(false);
        return true;
    }
    millis_t actTime = HAL::timeInMilliseconds();
    millis_t dtReal;
    millis_t dt = dtReal = actTime - lastNextPrev;
    lastNextPrev = actTime;
    if(dt < SPEED_MAX_MILLIS) dt = SPEED_MAX_MILLIS;
    if(dt > SPEED_MIN_MILLIS)
    {
        dt = SPEED_MIN_MILLIS;
        lastNextAccumul = 1;
    }
    float f = (float)(SPEED_MIN_MILLIS - dt) / (float)(SPEED_MIN_MILLIS - SPEED_MAX_MILLIS);
    lastNextAccumul = 1.0f + (float)SPEED_MAGNIFICATION * f * f * f;
#if UI_DYNAMIC_ENCODER_SPEED
    uint16_t dynSp = lastNextAccumul / 16;
    if(dynSp < 1)  dynSp = 1;
    if(dynSp > 30) dynSp = 30;
    next *= dynSp;
#endif

#if UI_HAS_KEYS == 1
    if(menuLevel == 0)
    {
        lastSwitch = HAL::timeInMilliseconds();
        if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0))
        {
            menuPos[0]++;
            if(menuPos[0] >= UI_NUM_PAGES)
                menuPos[0] = 0;
        }
        else
        {
            menuPos[0] = (menuPos[0] == 0 ? UI_NUM_PAGES - 1 : menuPos[0] - 1);
        }
        return true;
    }
    UIMenu *men = (UIMenu*)menu[menuLevel];
    uint8_t nr = pgm_read_word_near(&(men->numEntries));
    uint8_t mtype = HAL::readFlashByte((PGM_P)&(men->menuType));
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
    UIMenuEntry *testEnt;
    // 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command
    uint8_t entType = HAL::readFlashByte((PGM_P)&(ent->menuType));
    int action = pgm_read_word(&(ent->action));
    if(mtype == UI_MENU_TYPE_SUBMENU && activeAction == 0)   // browse through menu items
    {
        if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0))
        {
            while(menuPos[menuLevel] + 1 < nr)
            {
                menuPos[menuLevel]++;
                testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if(testEnt->showEntry())
                    break;
            }
        }
        else if(menuPos[menuLevel] > 0)
        {
            while(menuPos[menuLevel] > 0)
            {
                menuPos[menuLevel]--;
                testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if(testEnt->showEntry())
                    break;
            }
        }
        shift = -2; // reset shift position
        adjustMenuPos();
        return true;
    }
#if SDSUPPORT
    if(mtype == UI_MENU_TYPE_FILE_SELECTOR)   // SD listing
    {
        if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0))
        {
            menuPos[menuLevel] += abs(next);
            if(menuPos[menuLevel] > nFilesOnCard) menuPos[menuLevel] = nFilesOnCard;
        }
        else if(menuPos[menuLevel] > 0)
        {
            if(menuPos[menuLevel] > abs(next))
                menuPos[menuLevel] -= abs(next);
            else
                menuPos[menuLevel] = 0;
        }
        if(menuTop[menuLevel] > menuPos[menuLevel])
            menuTop[menuLevel] = menuPos[menuLevel];
        else if(menuTop[menuLevel] + UI_ROWS - 1 < menuPos[menuLevel])
            menuTop[menuLevel] = menuPos[menuLevel] + 1 - UI_ROWS;
        shift = -2; // reset shift position
        return true;
    }
#endif
    if(mtype == UI_MENU_TYPE_MODIFICATION_MENU || mtype == UI_MENU_TYPE_WIZARD) action = pgm_read_word(&(men->id));
    else action = activeAction;
    int8_t increment = next;
    switch(action)
    {
    case UI_ACTION_FANSPEED:
        Commands::setFanSpeed(Printer::getFanSpeed() + increment * 3,false);
        break;
    case UI_ACTION_XPOSITION:
        if(!allowMoves) return false;
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01*(float)increment * lastNextAccumul;
            if(fabs(d) * 2000 > Printer::maxFeedrate[X_AXIS] * dtReal)
                d *= Printer::maxFeedrate[X_AXIS]*dtReal / (2000 * fabs(d));
            long steps = (long)(d * Printer::axisStepsPerMM[X_AXIS]);
            steps = ( increment < 0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInStepsReal(steps,0,0,0,Printer::maxFeedrate[X_AXIS],true);
        }
#else
        PrintLine::moveRelativeDistanceInStepsReal(increment,0,0,0,Printer::homingFeedrate[X_AXIS],true);
#endif
        Commands::printCurrentPosition(PSTR("UI_ACTION_XPOSITION "));
        break;
    case UI_ACTION_YPOSITION:
        if(!allowMoves) return false;
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01 * (float)increment * lastNextAccumul;
            if(fabs(d) * 2000 > Printer::maxFeedrate[Y_AXIS] * dtReal)
                d *= Printer::maxFeedrate[Y_AXIS] * dtReal / (2000 * fabs(d));
            long steps = (long)(d * Printer::axisStepsPerMM[Y_AXIS]);
            steps = ( increment < 0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInStepsReal(0,steps,0,0,Printer::maxFeedrate[Y_AXIS],true);
        }
#else
        PrintLine::moveRelativeDistanceInStepsReal(0,increment,0,0,Printer::homingFeedrate[Y_AXIS],true);
#endif
        Commands::printCurrentPosition(PSTR("UI_ACTION_YPOSITION "));
        break;
    case UI_ACTION_ZPOSITION:
        if(!allowMoves) return false;
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01 * (float)increment * lastNextAccumul;
            if(fabs(d) * 2000 > Printer::maxFeedrate[Z_AXIS] * dtReal)
                d *= Printer::maxFeedrate[Z_AXIS] * dtReal / (2000 * fabs(d));
            long steps = (long)(d * Printer::axisStepsPerMM[Z_AXIS]);
            steps = ( increment<0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInStepsReal(0,0,steps,0,Printer::maxFeedrate[Z_AXIS],true);
        }
#else
        PrintLine::moveRelativeDistanceInStepsReal(0,0,increment,0,Printer::homingFeedrate[Z_AXIS],true);
#endif
        Commands::printCurrentPosition(PSTR("UI_ACTION_ZPOSITION "));
        break;
    case UI_ACTION_XPOSITION_FAST:
        if(!allowMoves) return false;
        PrintLine::moveRelativeDistanceInStepsReal(Printer::axisStepsPerMM[X_AXIS] * increment,0,0,0,Printer::homingFeedrate[X_AXIS],true);
        Commands::printCurrentPosition(PSTR("UI_ACTION_XPOSITION_FAST "));
        break;
    case UI_ACTION_YPOSITION_FAST:
        if(!allowMoves) return false;
        PrintLine::moveRelativeDistanceInStepsReal(0,Printer::axisStepsPerMM[Y_AXIS] * increment,0,0,Printer::homingFeedrate[Y_AXIS],true);
        Commands::printCurrentPosition(PSTR("UI_ACTION_YPOSITION_FAST "));
        break;
    case UI_ACTION_ZPOSITION_FAST:
        if(!allowMoves) return false;
        PrintLine::moveRelativeDistanceInStepsReal(0,0,Printer::axisStepsPerMM[Z_AXIS] * increment,0,Printer::homingFeedrate[Z_AXIS],true);
        Commands::printCurrentPosition(PSTR("UI_ACTION_ZPOSITION_FAST "));
        break;
    case UI_ACTION_EPOSITION:
        if(!allowMoves) return false;
        PrintLine::moveRelativeDistanceInSteps(0,0,0,Printer::axisStepsPerMM[E_AXIS]*increment / Printer::extrusionFactor,UI_SET_EXTRUDER_FEEDRATE,true,false);
        Commands::printCurrentPosition(PSTR("UI_ACTION_EPOSITION "));
        break;
#if FEATURE_RETRACTION
    case UI_ACTION_WIZARD_FILAMENTCHANGE: // filament change is finished
        Extruder::current->retractDistance(-increment);
        Commands::waitUntilEndOfAllMoves();
        Extruder::current->disableCurrentExtruderMotor();
        break;
#endif
    case UI_ACTION_ZPOSITION_NOTEST:
        if(!allowMoves) return false;
        Printer::setNoDestinationCheck(true);
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01 * (float)increment * lastNextAccumul;
            if(fabs(d) * 2000>Printer::maxFeedrate[Z_AXIS] * dtReal)
                d *= Printer::maxFeedrate[Z_AXIS] * dtReal / (2000 * fabs(d));
            long steps = (long)(d * Printer::axisStepsPerMM[Z_AXIS]);
            steps = ( increment < 0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[Z_AXIS], true);
        }
#else
        PrintLine::moveRelativeDistanceInStepsReal(0, 0, increment, 0, Printer::homingFeedrate[Z_AXIS], true);
#endif
        Commands::printCurrentPosition(PSTR("UI_ACTION_ZPOSITION_NOTEST "));
        Printer::setNoDestinationCheck(false);
        break;
    case UI_ACTION_ZPOSITION_FAST_NOTEST:
        Printer::setNoDestinationCheck(true);
        PrintLine::moveRelativeDistanceInStepsReal(0,0,Printer::axisStepsPerMM[Z_AXIS]*increment,0,Printer::homingFeedrate[Z_AXIS],true);
        Commands::printCurrentPosition(PSTR("UI_ACTION_ZPOSITION_FAST_NOTEST "));
        Printer::setNoDestinationCheck(false);
        break;
    case UI_ACTION_Z_BABYSTEPS:
#if FEATURE_BABYSTEPPING
    {
        previousMillisCmd = HAL::timeInMilliseconds();
        if((abs((int)Printer::zBabystepsMissing + (increment * BABYSTEP_MULTIPLICATOR))) < 127)
        {
            Printer::zBabystepsMissing += increment * BABYSTEP_MULTIPLICATOR;
            zBabySteps += increment * BABYSTEP_MULTIPLICATOR;
        }
    }
#endif
    break;
    case UI_ACTION_HEATED_BED_TEMP:
#if HAVE_HEATED_BED
    {
        int tmp = (int)heatedBedController.targetTemperatureC;
        if(tmp < UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
        if(tmp == 0 && increment > 0) tmp = UI_SET_MIN_HEATED_BED_TEMP;
        else tmp += increment;
        if(tmp < UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
        else if(tmp > UI_SET_MAX_HEATED_BED_TEMP) tmp = UI_SET_MAX_HEATED_BED_TEMP;
        Extruder::setHeatedBedTemperature(tmp);
    }
#endif
    break;

#if NUM_EXTRUDER>2
    case UI_ACTION_EXTRUDER2_TEMP:
#endif
#if NUM_EXTRUDER>1
    case UI_ACTION_EXTRUDER1_TEMP:
#endif
    case UI_ACTION_EXTRUDER0_TEMP:
        {
        int tmp = (int)extruder[action - UI_ACTION_EXTRUDER0_TEMP].tempControl.targetTemperatureC;
            if(tmp < UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
            if(tmp == 0 && increment > 0) tmp = UI_SET_MIN_EXTRUDER_TEMP;
            else tmp += increment;
                if(tmp < UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
                else if(tmp > UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp, action - UI_ACTION_EXTRUDER0_TEMP);
            }
            break;
    case UI_ACTION_FEEDRATE_MULTIPLY:
    {
        int fr = Printer::feedrateMultiply;
        INCREMENT_MIN_MAX(fr,1,25,500);
        Commands::changeFeedrateMultiply(fr);
    }
    break;
    case UI_ACTION_FLOWRATE_MULTIPLY:
    {
        INCREMENT_MIN_MAX(Printer::extrudeMultiply,1,25,500);
        Commands::changeFlowrateMultiply(Printer::extrudeMultiply);
    }
    break;
    case UI_ACTION_STEPPER_INACTIVE:
        stepperInactiveTime -= stepperInactiveTime % 1000;
        INCREMENT_MIN_MAX(stepperInactiveTime,60000UL,0,10080000UL);
        break;
    case UI_ACTION_MAX_INACTIVE:
        maxInactiveTime -= maxInactiveTime % 1000;
        INCREMENT_MIN_MAX(maxInactiveTime,60000UL,0,10080000UL);
        break;

    case UI_ACTION_PRINT_ACCEL_Z:
    case UI_ACTION_PRINT_ACCEL_Y:
    case UI_ACTION_PRINT_ACCEL_X:
#if DRIVE_SYSTEM!=DELTA
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[action - UI_ACTION_PRINT_ACCEL_X],((action == UI_ACTION_PRINT_ACCEL_Z) ? 1 : 100),0,10000);
#else
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[action - UI_ACTION_PRINT_ACCEL_X],100,0,10000);
#endif
        Printer::updateDerivedParameter();
        break;
            case UI_ACTION_MOVE_ACCEL_X:
            case UI_ACTION_MOVE_ACCEL_Y:
            case UI_ACTION_MOVE_ACCEL_Z:
        #if DRIVE_SYSTEM != DELTA
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[action - UI_ACTION_MOVE_ACCEL_X],((action == UI_ACTION_MOVE_ACCEL_Z) ? 1 : 100),0,10000);
        #else
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[action - UI_ACTION_MOVE_ACCEL_X],100,0,10000);
        #endif
                Printer::updateDerivedParameter();
                break;
    case UI_ACTION_MAX_JERK:
        INCREMENT_MIN_MAX(Printer::maxJerk,0.1,1,99.9);
        break;
#if DRIVE_SYSTEM != DELTA
    case UI_ACTION_MAX_ZJERK:
        INCREMENT_MIN_MAX(Printer::maxZJerk,0.1,0.1,99.9);
        break;
#endif
    case UI_ACTION_HOMING_FEEDRATE_X:
    case UI_ACTION_HOMING_FEEDRATE_Y:
    case UI_ACTION_HOMING_FEEDRATE_Z:
        INCREMENT_MIN_MAX(Printer::homingFeedrate[action - UI_ACTION_HOMING_FEEDRATE_X], 1, 1, 1000);
        break;

    case UI_ACTION_MAX_FEEDRATE_X:
    case UI_ACTION_MAX_FEEDRATE_Y:
    case UI_ACTION_MAX_FEEDRATE_Z:
        INCREMENT_MIN_MAX(Printer::maxFeedrate[action - UI_ACTION_MAX_FEEDRATE_X], 1, 1, 1000);
        break;

            case UI_ACTION_STEPS_X:
            case UI_ACTION_STEPS_Y:
            case UI_ACTION_STEPS_Z:
        INCREMENT_MIN_MAX(Printer::axisStepsPerMM[action - UI_ACTION_STEPS_X], 0.1, 0, 999);
                Printer::updateDerivedParameter();
                break;
    case UI_ACTION_BAUDRATE:
#if EEPROM_MODE != 0
    {
        char p = 0;
        int32_t rate;
        do
        {
            rate = pgm_read_dword(&(baudrates[p]));
            if(rate == baudrate) break;
            p++;
        }
        while(rate != 0);
        if(rate == 0) p -= 2;
        p += increment;
        if(p < 0) p = 0;
        rate = pgm_read_dword(&(baudrates[p]));
        if(rate == 0) p--;
        baudrate = pgm_read_dword(&(baudrates[p]));
    }
#endif
    break;
    case UI_ACTION_SERVOPOS:
#if FEATURE_SERVO > 0  && UI_SERVO_CONTROL > 0
        INCREMENT_MIN_MAX(servoPosition, 5, 500, 2500);
        HAL::servoMicroseconds(UI_SERVO_CONTROL - 1, servoPosition, 500);
#endif
        break;
#if TEMP_PID
    case UI_ACTION_PID_PGAIN:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidPGain,0.1,0,200);
        break;
    case UI_ACTION_PID_IGAIN:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidIGain,0.01,0,100);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_PID_DGAIN:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidDGain,0.1,0,200);
        break;
    case UI_ACTION_DRIVE_MIN:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidDriveMin,1,1,255);
        break;
    case UI_ACTION_DRIVE_MAX:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidDriveMax,1,1,255);
        break;
    case UI_ACTION_PID_MAX:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.pidMax,1,1,255);
        break;
#endif
    case UI_ACTION_X_OFFSET:
        INCREMENT_MIN_MAX(Extruder::current->xOffset,1,-99999,99999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_Y_OFFSET:
        INCREMENT_MIN_MAX(Extruder::current->yOffset,1,-99999,99999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_EXTR_STEPS:
        INCREMENT_MIN_MAX(Extruder::current->stepsPerMM,1,1,9999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_EXTR_ACCELERATION:
        INCREMENT_MIN_MAX(Extruder::current->maxAcceleration,10,10,99999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_EXTR_MAX_FEEDRATE:
        INCREMENT_MIN_MAX(Extruder::current->maxFeedrate,1,1,999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_EXTR_START_FEEDRATE:
        INCREMENT_MIN_MAX(Extruder::current->maxStartFeedrate,1,1,999);
        Extruder::selectExtruderById(Extruder::current->id);
        break;
    case UI_ACTION_EXTR_HEATMANAGER:
        INCREMENT_MIN_MAX(Extruder::current->tempControl.heatManager,1,0,3);
        break;
    case UI_ACTION_EXTR_WATCH_PERIOD:
        INCREMENT_MIN_MAX(Extruder::current->watchPeriod,1,0,999);
        break;
#if RETRACT_DURING_HEATUP
    case UI_ACTION_EXTR_WAIT_RETRACT_TEMP:
        INCREMENT_MIN_MAX(Extruder::current->waitRetractTemperature,1,100,UI_SET_MAX_EXTRUDER_TEMP);
        break;
    case UI_ACTION_EXTR_WAIT_RETRACT_UNITS:
        INCREMENT_MIN_MAX(Extruder::current->waitRetractUnits,1,0,99);
        break;
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    case UI_ACTION_ADVANCE_K:
        INCREMENT_MIN_MAX(Extruder::current->advanceK,1,0,200);
        break;
#endif
    case UI_ACTION_ADVANCE_L:
        INCREMENT_MIN_MAX(Extruder::current->advanceL,1,0,600);
        break;
#endif
    }
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
    return true;
}

void UIDisplay::finishAction(int action)
{
}
// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.
int UIDisplay::executeAction(int action, bool allowMoves)
{
    int ret = 0;
#if UI_HAS_KEYS == 1
    bool skipBeep = false;
    if(action & UI_ACTION_TOPMENU)   // Go to start menu
    {
        action -= UI_ACTION_TOPMENU;
        menuLevel = 0;
    }
    if(action >= 2000 && action < 3000)
    {
        setStatusP(ui_action);
    }
    else
        switch(action)
        {
        case UI_ACTION_OK:
            ret = okAction(allowMoves);
            skipBeep = true; // Prevent double beep
            break;
        case UI_ACTION_BACK:
            if(uid.isWizardActive()) break; // wizards can not exit before finished
            popMenu(false);
            break;
        case UI_ACTION_NEXT:
            if(!nextPreviousAction(1, allowMoves))
                ret = UI_ACTION_NEXT;
            break;
        case UI_ACTION_PREVIOUS:
            if(!nextPreviousAction(-1, allowMoves))
                ret = UI_ACTION_PREVIOUS;
            break;
        case UI_ACTION_MENU_UP:
            if(menuLevel > 0) menuLevel--;
            break;
        case UI_ACTION_TOP_MENU:
            menuLevel = 0;
            break;
        case UI_ACTION_EMERGENCY_STOP:
            Commands::emergencyStop();
            break;
        case UI_ACTION_HOME_ALL:
            if(!allowMoves) return UI_ACTION_HOME_ALL;
            Printer::homeAxis(true, true, true);
            Commands::printCurrentPosition(PSTR("UI_ACTION_HOMEALL "));
            break;
        case UI_ACTION_HOME_X:
            if(!allowMoves) return UI_ACTION_HOME_X;
            Printer::homeAxis(true, false, false);
            Commands::printCurrentPosition(PSTR("UI_ACTION_HOME_X "));
            break;
        case UI_ACTION_HOME_Y:
            if(!allowMoves) return UI_ACTION_HOME_Y;
            Printer::homeAxis(false, true, false);
            Commands::printCurrentPosition(PSTR("UI_ACTION_HOME_Y "));
            break;
        case UI_ACTION_HOME_Z:
            if(!allowMoves) return UI_ACTION_HOME_Z;
            Printer::homeAxis(false, false, true);
            Commands::printCurrentPosition(PSTR("UI_ACTION_HOME_Z "));
            break;
        case UI_ACTION_SET_ORIGIN:
            if(!allowMoves) return UI_ACTION_SET_ORIGIN;
            Printer::setOrigin(0, 0, 0);
            break;
        case UI_ACTION_DEBUG_ECHO:
            Printer::debugLevel ^= 1;
            break;
        case UI_ACTION_DEBUG_INFO:
            Printer::debugLevel ^= 2;
            break;
        case UI_ACTION_DEBUG_ERROR:
            Printer::debugLevel ^= 4;
            break;
        case UI_ACTION_DEBUG_DRYRUN:
            Printer::debugLevel ^= 8;
            if(Printer::debugDryrun())   // simulate movements without printing
            {
                Extruder::setTemperatureForExtruder(0, 0);
#if NUM_EXTRUDER > 1
                Extruder::setTemperatureForExtruder(0, 1);
#endif
#if NUM_EXTRUDER > 2
                Extruder::setTemperatureForExtruder(0, 2);
#endif
#if HAVE_HEATED_BED
                Extruder::setHeatedBedTemperature(0);
#endif
            }
            break;
        case UI_ACTION_POWER:
#if PS_ON_PIN >= 0 // avoid compiler errors when the power supply pin is disabled
            Commands::waitUntilEndOfAllMoves();
            SET_OUTPUT(PS_ON_PIN); //GND
            TOGGLE(PS_ON_PIN);
#endif
            break;
#if CASE_LIGHTS_PIN >= 0
        case UI_ACTION_LIGHTS_ONOFF:
            TOGGLE(CASE_LIGHTS_PIN);
            Printer::reportCaseLightStatus();
            UI_STATUS(UI_TEXT_LIGHTS_ONOFF);
            break;
#endif
        case UI_ACTION_PREHEAT_PLA:
            UI_STATUS(UI_TEXT_PREHEAT_PLA);
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,0);
#if NUM_EXTRUDER > 1
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,1);
#endif
#if NUM_EXTRUDER > 2
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,2);
#endif
#if HAVE_HEATED_BED
            Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_PLA);
#endif
            break;
        case UI_ACTION_PREHEAT_ABS:
            UI_STATUS(UI_TEXT_PREHEAT_ABS);
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,0);
#if NUM_EXTRUDER > 1
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,1);
#endif
#if NUM_EXTRUDER > 2
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,2);
#endif
#if HAVE_HEATED_BED
            Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_ABS);
#endif
            break;
        case UI_ACTION_COOLDOWN:
            UI_STATUS(UI_TEXT_COOLDOWN);
            Extruder::setTemperatureForExtruder(0, 0);
#if NUM_EXTRUDER > 1
            Extruder::setTemperatureForExtruder(0, 1);
#endif
#if NUM_EXTRUDER > 2
            Extruder::setTemperatureForExtruder(0, 2);
#endif
#if HAVE_HEATED_BED
            Extruder::setHeatedBedTemperature(0);
#endif
            break;
        case UI_ACTION_HEATED_BED_OFF:
#if HAVE_HEATED_BED
            Extruder::setHeatedBedTemperature(0);
#endif
            break;
        case UI_ACTION_EXTRUDER0_OFF:
            Extruder::setTemperatureForExtruder(0, 0);
            break;
        case UI_ACTION_EXTRUDER1_OFF:
#if NUM_EXTRUDER > 1
            Extruder::setTemperatureForExtruder(0, 1);
#endif
            break;
        case UI_ACTION_EXTRUDER2_OFF:
#if NUM_EXTRUDER>2
            Extruder::setTemperatureForExtruder(0, 2);
#endif
            break;
        case UI_ACTION_DISABLE_STEPPER:
            Printer::kill(true);
            break;
        case UI_ACTION_RESET_EXTRUDER:
            Printer::currentPositionSteps[E_AXIS] = 0;
            break;
        case UI_ACTION_EXTRUDER_RELATIVE:
            Printer::relativeExtruderCoordinateMode=!Printer::relativeExtruderCoordinateMode;
            break;
        case UI_ACTION_SELECT_EXTRUDER0:
#if NUM_EXTRUDER > 0
            if(!allowMoves) return UI_ACTION_SELECT_EXTRUDER0;
            Extruder::selectExtruderById(0);
#endif
            break;
        case UI_ACTION_SELECT_EXTRUDER1:
#if NUM_EXTRUDER > 1
            if(!allowMoves) return UI_ACTION_SELECT_EXTRUDER1;
            Extruder::selectExtruderById(1);
#endif
            break;
        case UI_ACTION_SELECT_EXTRUDER2:
#if NUM_EXTRUDER > 2
            if(!allowMoves) return UI_ACTION_SELECT_EXTRUDER2;
            Extruder::selectExtruderById(2);
#endif
            break;
#if EEPROM_MODE != 0
        case UI_ACTION_STORE_EEPROM:
            EEPROM::storeDataIntoEEPROM(false);
            pushMenu(&ui_menu_eeprom_saved, false);
            BEEP_LONG;
            skipBeep = true;
            break;
        case UI_ACTION_LOAD_EEPROM:
            EEPROM::readDataFromEEPROM();
            Extruder::selectExtruderById(Extruder::current->id);
            pushMenu(&ui_menu_eeprom_loaded, false);
            BEEP_LONG;
            skipBeep = true;
            break;
#endif
#if SDSUPPORT
        case UI_ACTION_SD_DELETE:
            if(sd.sdactive)
            {
                pushMenu(&ui_menu_sd_fileselector, false);
            }
            else
            {
                UI_ERROR(UI_TEXT_NOSDCARD);
            }
            break;
        case UI_ACTION_SD_PRINT:
            if(sd.sdactive)
            {
                pushMenu(&ui_menu_sd_fileselector, false);
            }
            break;
        case UI_ACTION_SD_PAUSE:
            if(!allowMoves)
                ret = UI_ACTION_SD_PAUSE;
            else
                sd.pausePrint(true);
            break;
        case UI_ACTION_SD_CONTINUE:
            if(!allowMoves) ret = UI_ACTION_SD_CONTINUE;
            else sd.continuePrint(true);
            break;
        case UI_ACTION_SD_PRI_PAU_CONT:
            if(!allowMoves) ret = UI_ACTION_SD_PRI_PAU_CONT;
            else
            {
                if(Printer::isMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_SD_PAUSED))
                    sd.continuePrint();
                else if(Printer::isMenuMode(MENU_MODE_SD_PRINTING))
                    sd.pausePrint(true);
                else if(sd.sdactive)
                    pushMenu(&ui_menu_sd_fileselector,false);
            }
            break;
        case UI_ACTION_SD_STOP:
            if(!allowMoves) ret = UI_ACTION_SD_STOP;
            else sd.stopPrint();
            break;
        case UI_ACTION_SD_UNMOUNT:
            sd.unmount();
            break;
        case UI_ACTION_SD_MOUNT:
            sd.mount();
            break;
        case UI_ACTION_MENU_SDCARD:
            pushMenu(&ui_menu_sd, false);
            break;
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
        case UI_ACTION_FAN_OFF:
        case UI_ACTION_FAN_25:
        case UI_ACTION_FAN_50:
        case UI_ACTION_FAN_75:
            Commands::setFanSpeed((action - UI_ACTION_FAN_OFF) * 64, false);
            break;
        case UI_ACTION_FAN_FULL:
            Commands::setFanSpeed(255, false);
            break;
        case UI_ACTION_FAN_SUSPEND:
        {
            static uint8_t lastFanSpeed = 255;
            if(Printer::getFanSpeed()==0)
                Commands::setFanSpeed(lastFanSpeed,false);
            else
            {
                lastFanSpeed = Printer::getFanSpeed();
                Commands::setFanSpeed(0,false);
            }
        }
        break;
#endif
        case UI_ACTION_MENU_XPOS:
            pushMenu(&ui_menu_xpos, false);
            break;
        case UI_ACTION_MENU_YPOS:
            pushMenu(&ui_menu_ypos, false);
            break;
        case UI_ACTION_MENU_ZPOS:
            pushMenu(&ui_menu_zpos, false);
            break;
        case UI_ACTION_MENU_XPOSFAST:
            pushMenu(&ui_menu_xpos_fast, false);
            break;
        case UI_ACTION_MENU_YPOSFAST:
            pushMenu(&ui_menu_ypos_fast, false);
            break;
        case UI_ACTION_MENU_ZPOSFAST:
            pushMenu(&ui_menu_zpos_fast, false);
            break;
        case UI_ACTION_MENU_QUICKSETTINGS:
            pushMenu(&ui_menu_quick, false);
            break;
        case UI_ACTION_MENU_EXTRUDER:
            pushMenu(&ui_menu_extruder, false);
            break;
        case UI_ACTION_MENU_POSITIONS:
            pushMenu(&ui_menu_positions, false);
            break;
#ifdef UI_USERMENU1
        case UI_ACTION_SHOW_USERMENU1:
            pushMenu(&UI_USERMENU1, false);
            break;
#endif
#ifdef UI_USERMENU2
        case UI_ACTION_SHOW_USERMENU2:
            pushMenu(&UI_USERMENU2, false);
            break;
#endif
#ifdef UI_USERMENU3
        case UI_ACTION_SHOW_USERMENU3:
            pushMenu(&UI_USERMENU3, false);
            break;
#endif
#ifdef UI_USERMENU4
        case UI_ACTION_SHOW_USERMENU4:
            pushMenu(&UI_USERMENU4, false);
            break;
#endif
#ifdef UI_USERMENU5
        case UI_ACTION_SHOW_USERMENU5:
            pushMenu(&UI_USERMENU5, false);
            break;
#endif
#ifdef UI_USERMENU6
        case UI_ACTION_SHOW_USERMENU6:
            pushMenu(&UI_USERMENU6, false);
            break;
#endif
#ifdef UI_USERMENU7
        case UI_ACTION_SHOW_USERMENU7:
            pushMenu(&UI_USERMENU7, false);
            break;
#endif
#ifdef UI_USERMENU8
        case UI_ACTION_SHOW_USERMENU8:
            pushMenu(&UI_USERMENU8, false);
            break;
#endif
#ifdef UI_USERMENU9
        case UI_ACTION_SHOW_USERMENU9:
            pushMenu(&UI_USERMENU9, false);
            break;
#endif
#ifdef UI_USERMENU10
        case UI_ACTION_SHOW_USERMENU10:
            pushMenu(&UI_USERMENU10, false);
            break;
#endif
#if FEATURE_RETRACTION
        case UI_ACTION_WIZARD_FILAMENTCHANGE:
        {
            Com::printFLN(PSTR("important: Filament change required!"));
            Printer::setBlockingReceive(true);
            pushMenu(&ui_wiz_filamentchange, true);
            Printer::resetWizardStack();
            Printer::pushWizardVar(Printer::currentPositionSteps[E_AXIS]);
            Printer::MemoryPosition();
            Extruder::current->retractDistance(FILAMENTCHANGE_SHORTRETRACT);
            float newZ = FILAMENTCHANGE_Z_ADD + Printer::currentPosition[Z_AXIS];
            Printer::currentPositionSteps[E_AXIS] = 0;
            Printer::moveToReal(Printer::currentPosition[X_AXIS], Printer::currentPosition[Y_AXIS], newZ, 0, Printer::homingFeedrate[Z_AXIS]);
            Printer::moveToReal(FILAMENTCHANGE_X_POS, FILAMENTCHANGE_Y_POS, newZ, 0, Printer::homingFeedrate[X_AXIS]);
            Extruder::current->retractDistance(FILAMENTCHANGE_LONGRETRACT);
            Extruder::current->disableCurrentExtruderMotor();
        }
        break;
#endif
        case UI_ACTION_X_UP:
        case UI_ACTION_X_DOWN:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP) ? 1.0 : -1.0) * Printer::axisStepsPerMM[X_AXIS], 0, 0, 0, Printer::homingFeedrate[X_AXIS], false);
            break;
        case UI_ACTION_Y_UP:
        case UI_ACTION_Y_DOWN:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS], 0, 0, Printer::homingFeedrate[Y_AXIS], false);
            break;
        case UI_ACTION_Z_UP:
        case UI_ACTION_Z_DOWN:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], false);
            break;
        case UI_ACTION_EXTRUDER_UP:
        case UI_ACTION_EXTRUDER_DOWN:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, 0, ((action == UI_ACTION_EXTRUDER_UP) ? 1.0 : -1.0) * Printer::axisStepsPerMM[E_AXIS], UI_SET_EXTRUDER_FEEDRATE, false);
            break;
/*
        case UI_ACTION_X_UP:
            if(!allowMoves) return UI_ACTION_X_UP;
            PrintLine::moveRelativeDistanceInStepsReal(Printer::axisStepsPerMM[X_AXIS], 0, 0, 0, Printer::homingFeedrate[X_AXIS], false);
            break;
        case UI_ACTION_X_DOWN:
            if(!allowMoves) return UI_ACTION_X_DOWN;
            PrintLine::moveRelativeDistanceInStepsReal(-Printer::axisStepsPerMM[X_AXIS], 0, 0, 0, Printer::homingFeedrate[X_AXIS], false);
            break;
        case UI_ACTION_Y_UP:
            if(!allowMoves) return UI_ACTION_Y_UP;
            PrintLine::moveRelativeDistanceInStepsReal(0, Printer::axisStepsPerMM[Y_AXIS], 0, 0, Printer::homingFeedrate[Y_AXIS], false);
            break;
        case UI_ACTION_Y_DOWN:
            if(!allowMoves) return UI_ACTION_Y_DOWN;
            PrintLine::moveRelativeDistanceInStepsReal(0, -Printer::axisStepsPerMM[Y_AXIS], 0, 0, Printer::homingFeedrate[Y_AXIS], false);
            break;
        case UI_ACTION_Z_UP:
            if(!allowMoves) return UI_ACTION_Z_UP;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], false);
            break;
        case UI_ACTION_Z_DOWN:
            if(!allowMoves) return UI_ACTION_Z_DOWN;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, -Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], false);
            break;
        case UI_ACTION_EXTRUDER_UP:
            if(!allowMoves) return UI_ACTION_EXTRUDER_UP;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, 0, Printer::axisStepsPerMM[E_AXIS],UI_SET_EXTRUDER_FEEDRATE, false);
            break;
        case UI_ACTION_EXTRUDER_DOWN:
            if(!allowMoves) return UI_ACTION_EXTRUDER_DOWN;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, 0, -Printer::axisStepsPerMM[E_AXIS], UI_SET_EXTRUDER_FEEDRATE, false);
            break;
*/
        case UI_ACTION_EXTRUDER_TEMP_UP:
        {
            int tmp = (int)(Extruder::current->tempControl.targetTemperatureC) + 1;
            if(tmp == 1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
            else if(tmp > UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
            Extruder::setTemperatureForExtruder(tmp, Extruder::current->id);
        }
        break;
        case UI_ACTION_EXTRUDER_TEMP_DOWN:
        {
            int tmp = (int)(Extruder::current->tempControl.targetTemperatureC) - 1;
            if(tmp < UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
            Extruder::setTemperatureForExtruder(tmp, Extruder::current->id);
        }
        break;
        case UI_ACTION_HEATED_BED_UP:
#if HAVE_HEATED_BED
        {
            int tmp = (int)heatedBedController.targetTemperatureC + 1;
            if(tmp == 1) tmp = UI_SET_MIN_HEATED_BED_TEMP;
            else if(tmp > UI_SET_MAX_HEATED_BED_TEMP) tmp = UI_SET_MAX_HEATED_BED_TEMP;
            Extruder::setHeatedBedTemperature(tmp);
        }
#endif
        break;
#if MAX_HARDWARE_ENDSTOP_Z
        case UI_ACTION_SET_MEASURED_ORIGIN:
        {
            Printer::updateCurrentPosition();
            Printer::zLength -= Printer::currentPosition[Z_AXIS];
            Printer::currentPositionSteps[Z_AXIS] = 0;
            Printer::updateDerivedParameter();
#if NONLINEAR_SYSTEM
            transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
#endif
            Printer::updateCurrentPosition(true);
            Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#if EEPROM_MODE != 0
            EEPROM::storeDataIntoEEPROM(false);
            Com::printFLN(Com::tEEPROMUpdated);
#endif
            Commands::printCurrentPosition(PSTR("UI_ACTION_SET_MEASURED_ORIGIN "));
        }
        break;
#endif
        case UI_ACTION_SET_P1:
#if SOFTWARE_LEVELING
            for (uint8_t i = 0; i < 3; i++)
            {
                Printer::levelingP1[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_SET_P2:
#if SOFTWARE_LEVELING
            for (uint8_t i = 0; i < 3; i++)
            {
                Printer::levelingP2[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_SET_P3:
#if SOFTWARE_LEVELING
            for (uint8_t i = 0; i < 3; i++)
            {
                Printer::levelingP3[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_CALC_LEVEL:
#if SOFTWARE_LEVELING
            int32_t factors[4];
            PrintLine::calculatePlane(factors, Printer::levelingP1, Printer::levelingP2, Printer::levelingP3);
            Com::printFLN(Com::tLevelingCalc);
            Com::printFLN(Com::tTower1, PrintLine::calcZOffset(factors, Printer::deltaAPosXSteps, Printer::deltaAPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
            Com::printFLN(Com::tTower2, PrintLine::calcZOffset(factors, Printer::deltaBPosXSteps, Printer::deltaBPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
            Com::printFLN(Com::tTower3, PrintLine::calcZOffset(factors, Printer::deltaCPosXSteps, Printer::deltaCPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
#endif
            break;
        case UI_ACTION_HEATED_BED_DOWN:
#if HAVE_HEATED_BED
        {
            int tmp = (int)heatedBedController.targetTemperatureC - 1;
            if(tmp < UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
            Extruder::setHeatedBedTemperature(tmp);
        }
#endif
        break;
        case UI_ACTION_FAN_UP:
            Commands::setFanSpeed(Printer::getFanSpeed() + 32,false);
            break;
        case UI_ACTION_FAN_DOWN:
            Commands::setFanSpeed(Printer::getFanSpeed() - 32,false);
            break;
        case UI_ACTION_KILL:
            Commands::emergencyStop();
            break;
        case UI_ACTION_RESET:
            HAL::resetHardware();
            break;
        case UI_ACTION_PAUSE:
            Com::printFLN(PSTR("RequestPause:"));
            break;
#if FEATURE_AUTOLEVEL
        case UI_ACTION_AUTOLEVEL_ONOFF:
            Printer::setAutolevelActive(!Printer::isAutolevelActive());
            break;
#endif
#ifdef DEBUG_PRINT
        case UI_ACTION_WRITE_DEBUG:
            Com::printF(PSTR("Buf. Read Idx:"),(int)GCode::bufferReadIndex);
            Com::printF(PSTR(" Buf. Write Idx:"),(int)GCode::bufferWriteIndex);
            Com::printF(PSTR(" Comment:"),(int)GCode::commentDetected);
            Com::printF(PSTR(" Buf. Len:"),(int)GCode::bufferLength);
            Com::printF(PSTR(" Wait resend:"),(int)GCode::waitingForResend);
            Com::printFLN(PSTR(" Recv. Write Pos:"),(int)GCode::commandsReceivingWritePosition);
            Com::printF(PSTR("Min. XY Speed:"),Printer::minimumSpeed);
            Com::printF(PSTR(" Min. Z Speed:"),Printer::minimumZSpeed);
            Com::printF(PSTR(" Buffer:"),PrintLine::linesCount);
            Com::printF(PSTR(" Lines pos:"),(int)PrintLine::linesPos);
            Com::printFLN(PSTR(" Write Pos:"),(int)PrintLine::linesWritePos);
            Com::printFLN(PSTR("Wait loop:"),debugWaitLoop);
            Com::printF(PSTR("sd mode:"),(int)sd.sdmode);
            Com::printF(PSTR(" pos:"),sd.sdpos);
            Com::printFLN(PSTR(" of "),sd.filesize);
            break;
#endif
        case UI_ACTION_TEMP_DEFECT:
            Printer::setAnyTempsensorDefect();
            break;
        }
    refreshPage();
    if(!skipBeep)
        BEEP_SHORT
#if UI_AUTORETURN_TO_MENU_AFTER!=0
        ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
    return ret;
}
void UIDisplay::mediumAction()
{
#if UI_HAS_I2C_ENCODER>0
    uiCheckSlowEncoder();
#endif
}

// Gets calles from main tread
void UIDisplay::slowAction(bool allowMoves)
{
    unsigned long time = HAL::timeInMilliseconds();
    uint8_t refresh = 0;
#if UI_HAS_KEYS==1
    // delayed action open?
    if(allowMoves && delayedAction != 0)
    {
        executeAction(delayedAction, true);
        delayedAction = 0;
    }

    // Update key buffer
    InterruptProtectedBlock noInts;
    if((flags & (UI_FLAG_FAST_KEY_ACTION + UI_FLAG_KEY_TEST_RUNNING)) == 0)
    {
        flags |= UI_FLAG_KEY_TEST_RUNNING;
        noInts.unprotect();
#if defined(UI_I2C_HOTEND_LED) || defined(UI_I2C_HEATBED_LED) || defined(UI_I2C_FAN_LED)
        {
            // check temps and set appropriate leds
            int led = 0;
#if NUM_EXTRUDER>0 && defined(UI_I2C_HOTEND_LED)
            led |= (tempController[Extruder::current->id]->targetTemperatureC > 0 ? UI_I2C_HOTEND_LED : 0);
#endif
#if HAVE_HEATED_BED && defined(UI_I2C_HEATBED_LED)
            led |= (heatedBedController.targetTemperatureC > 0 ? UI_I2C_HEATBED_LED : 0);
#endif
#if FAN_PIN>=0 && defined(UI_I2C_FAN_LED)
            led |= (Printer::getFanSpeed() > 0 ? UI_I2C_FAN_LED : 0);
#endif
            // update the leds
            uid.outputMask= ~led & (UI_I2C_HEATBED_LED | UI_I2C_HOTEND_LED | UI_I2C_FAN_LED);
        }
#endif
        int nextAction = 0;
        uiCheckSlowKeys(nextAction);
        if(lastButtonAction != nextAction)
        {
            lastButtonStart = time;
            lastButtonAction = nextAction;
            noInts.protect();
            flags |= UI_FLAG_SLOW_KEY_ACTION; // Mark slow action
        }
        noInts.protect();
        flags &= ~UI_FLAG_KEY_TEST_RUNNING;
    }
    noInts.protect();
    if((flags & UI_FLAG_SLOW_ACTION_RUNNING) == 0)
    {
        flags |= UI_FLAG_SLOW_ACTION_RUNNING;
        // Reset click encoder
        noInts.protect();
        int16_t encodeChange = encoderPos;
        encoderPos = 0;
        noInts.unprotect();
        int newAction;
        if(encodeChange) // encoder changed
        {
            nextPreviousAction(encodeChange, allowMoves);
            BEEP_SHORT
            refresh = 1;
        }
        if(lastAction != lastButtonAction)
        {
            if(lastButtonAction == 0)
            {
                if(lastAction >= 2000 && lastAction < 3000)
                    statusMsg[0] = 0;
                lastAction = 0;
                noInts.protect();
                flags &= ~(UI_FLAG_FAST_KEY_ACTION + UI_FLAG_SLOW_KEY_ACTION);
            }
            else if(time - lastButtonStart > UI_KEY_BOUNCETIME)     // New key pressed
            {
                lastAction = lastButtonAction;
                if((newAction = executeAction(lastAction, allowMoves)) == 0)
                {
                    nextRepeat = time + UI_KEY_FIRST_REPEAT;
                    repeatDuration = UI_KEY_FIRST_REPEAT;
                }
                else
                {
                    if(delayedAction == 0)
                        delayedAction = newAction;
                }
            }
        }
        else if(lastAction < 1000 && lastAction)     // Repeatable key
        {
            if(time - nextRepeat < 10000)
            {
                if(delayedAction == 0)
                    delayedAction = executeAction(lastAction, allowMoves);
                else
                    executeAction(lastAction, allowMoves);
                repeatDuration -= UI_KEY_REDUCE_REPEAT;
                if(repeatDuration < UI_KEY_MIN_REPEAT) repeatDuration = UI_KEY_MIN_REPEAT;
                nextRepeat = time + repeatDuration;
            }
        }
        noInts.protect();
        flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;
    }
    noInts.unprotect();
#endif
#if UI_AUTORETURN_TO_MENU_AFTER != 0
    if(menuLevel > 0 && ui_autoreturn_time < time && !uid.isWizardActive()) // Go to top menu after x seoonds
    {
        lastSwitch = time;
        menuLevel = 0;
        activeAction = 0;
    }
#endif
    if(uid.isWizardActive())
        previousMillisCmd = HAL::timeInMilliseconds(); // prevent stepper/heater disable from timeout during active wizard
    if(menuLevel == 0 && time > 4000) // Top menu refresh/switch
    {
        if(time - lastSwitch > UI_PAGES_DURATION)
        {
            lastSwitch = time;
#if !defined(UI_DISABLE_AUTO_PAGESWITCH) || !UI_DISABLE_AUTO_PAGESWITCH
            menuPos[0]++;
            if(menuPos[0] >= UI_NUM_PAGES)
                menuPos[0] = 0;
#endif
            refresh = 1;
        }
        else if(time - lastRefresh >= 1000) refresh = 1;
    }
    else if(time - lastRefresh >= 800)
    {
        UIMenu *men = (UIMenu*)menu[menuLevel];
        uint8_t mtype = pgm_read_byte((void*)&(men->menuType));
        //if(mtype!=1)
        refresh = 1;
    }
    if(refresh) // does lcd need a refresh?
    {
        if (menuLevel > 1 || Printer::isAutomount())
        {
            shift++;
            if(shift + UI_COLS > MAX_COLS + 1)
                shift = -2;
        }
        else
            shift = -2;

        refreshPage();
        lastRefresh = time;
    }
}


// Gets called from inside an interrupt with interrupts allowed!
void UIDisplay::fastAction()
{
#if UI_HAS_KEYS == 1
    // Check keys
    InterruptProtectedBlock noInts;
    if((flags & (UI_FLAG_KEY_TEST_RUNNING + UI_FLAG_SLOW_KEY_ACTION)) == 0)
    {
        flags |= UI_FLAG_KEY_TEST_RUNNING;
        int nextAction = 0;
        uiCheckKeys(nextAction);
        ui_check_Ukeys(nextAction);
        if(lastButtonAction != nextAction)
        {
            lastButtonStart = HAL::timeInMilliseconds();
            lastButtonAction = nextAction;
            flags |= UI_FLAG_FAST_KEY_ACTION;
        }
        flags &= ~UI_FLAG_KEY_TEST_RUNNING;
    }
#endif
}
#if UI_ENCODER_SPEED==0
const int8_t encoder_table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; // Full speed
#elif UI_ENCODER_SPEED==1
const int8_t encoder_table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0}; // Half speed
#else
//const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0}; // Quart speed
//const int8_t encoder_table[16] PROGMEM = {0,1,0,0,-1,0,0,0,0,0,0,0,0,0,0,0}; // Quart speed
const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,-1,0,0,1,0}; // Quart speed
#endif

#endif

