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

#define UI_MAIN
#include "Repetier.h"
extern const int8_t encoder_table[16] PROGMEM ;
#include "ui.h"
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include "Eeprom.h"
#include <ctype.h>

#if UI_ENCODER_SPEED==0
const int8_t encoder_table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; // Full speed
#elif UI_ENCODER_SPEED==1
const int8_t encoder_table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0}; // Half speed
#else
//const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0}; // Quart speed
//const int8_t encoder_table[16] PROGMEM = {0,1,0,0,-1,0,0,0,0,0,0,0,0,0,0,0}; // Quart speed
const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,-1,0,0,1,0}; // Quart speed
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

char printCols[MAX_COLS+2];
#if SDSUPPORT!=true
char tempLongFilename[MAX_COLS+3];
#else
char tempLongFilename[LONG_FILENAME_LENGTH+1];
#endif
byte oldOffset, oldMenuLevel, encoderStartScreen, iScreenTransition;

void beep(uint8_t duration,uint8_t count)
{
#if FEATURE_BEEPER
#if BEEPER_TYPE!=0
#if BEEPER_TYPE==1
    SET_OUTPUT(BEEPER_PIN);
#endif
#if BEEPER_TYPE==2
    HAL::i2cStartWait(BEEPER_ADDRESS+I2C_WRITE);
#if UI_DISPLAY_I2C_CHIPTYPE==1
    HAL::i2cWrite( 0x14); // Start at port a
#endif
#endif
    for(uint8_t i=0; i<count; i++)
    {
#if BEEPER_TYPE==1
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
#if BEEPER_TYPE==1
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
    uint8_t f,f2;
    f = HAL::readFlashByte((const prog_char*)&filter);
    if(f!=0)
        ret = (f & Printer::menuMode) != 0;
    f2 = HAL::readFlashByte((const prog_char*)&nofilter);
    if(ret && f2!=0)
    {
        ret = (f2 & Printer::menuMode) == 0;
    }
    return ret;
}

#if UI_DISPLAY_TYPE!=0
UIDisplay uid;

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

#if UI_DISPLAY_TYPE==3

// ============= I2C LCD Display driver ================
inline void lcdStartWrite()
{
    HAL::i2cStartWait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
#if UI_DISPLAY_I2C_CHIPTYPE==1
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
    value|=uid.outputMask;
#if UI_DISPLAY_D4_PIN==1 && UI_DISPLAY_D5_PIN==2 && UI_DISPLAY_D6_PIN==4 && UI_DISPLAY_D7_PIN==8
    HAL::i2cWrite((value) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(value);
#else
    uint8_t v=(value & 1?UI_DISPLAY_D4_PIN:0)|(value & 2?UI_DISPLAY_D5_PIN:0)|(value & 4?UI_DISPLAY_D6_PIN:0)|(value & 8?UI_DISPLAY_D7_PIN:0);
    HAL::i2cWrite((v) | UI_DISPLAY_ENABLE_PIN);
    HAL::i2cWrite(v);
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
    HAL::i2cWrite(uid.outputMask >> 16);
#endif
    HAL::delayMicroseconds(10);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
    // second try
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150); // wait
    // third go!
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(150);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);
    lcdCommand(LCD_CLEAR);					//-	Clear Screen
    HAL::delayMilliseconds(2); // clear is slow operation
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
#if UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2

void lcdWriteNibble(uint8_t value)
{
    WRITE(UI_DISPLAY_D4_PIN,value & 1);
    WRITE(UI_DISPLAY_D5_PIN,value & 2);
    WRITE(UI_DISPLAY_D6_PIN,value & 4);
    WRITE(UI_DISPLAY_D7_PIN,value & 8);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);// enable pulse must be >450ns
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}
void lcdWriteByte(uint8_t c,uint8_t rs)
{
#if UI_DISPLAY_RW_PIN<0
    HAL::delayMicroseconds(UI_DELAYPERCHAR);
#else
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
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
        busy = READ(UI_DISPLAY_D7_PIN);
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
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
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);   // enable pulse must be >450ns
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);   // enable pulse must be >450ns
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
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
#if UI_DISPLAY_RW_PIN>-1
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
    HAL::delayMicroseconds(10);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
    // second try
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150); // wait
    // third go!
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(150);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);

    lcdCommand(LCD_CLEAR);					//-	Clear Screen
    HAL::delayMilliseconds(2); // clear is slow operation
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
}
// ----------- end direct LCD driver
#endif

void  UIDisplay::waitForKey()
{
    int nextAction = 0;
    
    lastButtonAction = 0;
    while(lastButtonAction==nextAction)
        {
        ui_check_slow_keys(nextAction);
        }
}

void UIDisplay::transitionInRow(byte r, PGM_P txt, byte bProgMem)
{    
#if NO_SCREEN_ANIMATION==true
    printRow(r, (char *)txt);
#else
      // **** HELP ***** Without these lines in, always 1-2 secs into the screen animation it crashes.. only 0.90 not 0.80
      // When I add these lines works always, 100% of the time... but the delays don't get executed...
      // Something in the interrupt is conflicting with the screen not being painted quick enough....
//    HAL::forbidInterrupts();

    switch(iScreenTransition)
      {
        case 0:
          printRow(r, (char *)txt);
          break;
        case 1:
          scrollHorzRow(r,(PGM_P)txt, bProgMem, 1);
          break;
        case 2:
          scrollHorzRow(r, (PGM_P)txt, bProgMem, 0);
          break;
        case 3:
          scrollVertRow(r, txt, bProgMem, false);
          break;
        case 4:
          randomRow(r, txt, bProgMem);
          break;
      }
      // **** HELP ***** Without these lines in, always 1-2 secs into the screen animation it crashes.. only 0.90 not 0.80
//         HAL::allowInterrupts();
#endif
}

#if !defined(NOSCREEN_ANIMATION) || NO_SCREEN_ANIMATION!=true

void UIDisplay::scrollVertRow(byte r, PGM_P txt, byte bProgMem, int8_t bFromTop)
{
  char *spaceUsed = tempLongFilename;
  
  if (bProgMem)
    {
    col=0;
    addStringP(txt);
    }

  for(int8_t i=UI_ROWS-1;i>=r;i--)
    {
  // check for any encoder or key action and finish animation
    if (!bProgMem && uid.encoderLast != encoderStartScreen)
      {
      printRow(r, 0, printCols, 0);
      break;
      }      
    printRow(i, 0, printCols, 0);
    if (i<UI_ROWS-1)
      printRow(i+1, UI_COLS, printCols, 0);
    HAL::delayMilliseconds(125);
    }
}


void UIDisplay::randomRow(byte r, PGM_P txt, byte bProgMem)
{
  char *spaceUsed = tempLongFilename;
  
  if (bProgMem)
    {
    col=0;
    addStringP(txt);
    }
    
  memset(spaceUsed, '-', UI_COLS);
  for(byte i=0;i<UI_COLS;i++)
    {
    byte xRand = random(UI_COLS-i-1);

   // check for any encoder or key action and finish animation
   if (!bProgMem && uid.encoderLast != encoderStartScreen)
      {
      printRow(r, 0, printCols, 0);
      break;
      }
    
    for(byte ii=0;;ii++)
        {
        if (spaceUsed[ii] == '-')
          {
          if (xRand == 0)
            {
            spaceUsed[ii] = ii < col ? *(printCols+ii) : ' ';
            printRow(r, 0, spaceUsed, 0);
            HAL::delayMilliseconds(25);
            break;
            }
          xRand--;
          }
        }
    }
}

void UIDisplay::scrollHorzRow(byte r, PGM_P txt, byte bProgMem, int8_t bFromLeft)
{
  int8_t i;
  
  if (bFromLeft == -1)
    bFromLeft = random(10) > 4;
    
  if (bProgMem)
    {
    col=0;
    addStringP(txt);
    }

  memset(printCols+col, 0, RMath::max(1, MAX_COLS-col));
  for(i=UI_COLS;i>=0;--i)
    {
  // check for any encoder or key action and finish animation
    if (!bProgMem && uid.encoderLast != encoderStartScreen)
      {
      printRow(r, 0, printCols, 0);
      break;
      }      
    if (bFromLeft)
       printRow(r, 0, printCols, i);
    else
       printRow(r, i, printCols, 0);  
    HAL::delayMilliseconds(20);
    }
}
#endif

#if UI_DISPLAY_TYPE==4
// Use LiquidCrystal library instead
#include <LiquidCrystal.h>

LiquidCrystal lcd(UI_DISPLAY_RS_PIN, UI_DISPLAY_RW_PIN,UI_DISPLAY_ENABLE_PIN,UI_DISPLAY_D4_PIN,UI_DISPLAY_D5_PIN,UI_DISPLAY_D6_PIN,UI_DISPLAY_D7_PIN);

void UIDisplay::createChar(uint8_t location,const uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    uint8_t data[8];
    for (int i=0; i<8; i++)
    {
        data[i]=pgm_read_byte(&(charmap[i]));
    }
    lcd.createChar(location, data);
}


void UIDisplay::printRow(byte r, byte x, char *txt, byte xChar)
{    
 byte col=0;
 char c;

 // Set row
 if(r >= UI_ROWS) return;
 
 lcd.setCursor(0,r);
 
 while(col<x)
   {
   lcd.write(' ');
   col++; 
   }
   
 txt += xChar;
 while(col<UI_COLS && (c=*txt) != 0x00)
   {
   txt++;
   lcd.write(c);
   col++;
   }
 
 while(col<UI_COLS) 
   {
   lcd.write(' ');
   col++; 
   }
#if UI_HAS_KEYS==1
 mediumAction();
#endif
}

void UIDisplay::printRow(byte r,char *txt) { 
  printRow(r, 0, txt, 0);  
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
#endif
UIDisplay::UIDisplay()
{
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
#endif

#endif
    flags = 0;
    menuLevel = 0;
    shift = -2;
    menuPos[0] = 0;
    lastAction = 0;
    lastButtonAction = 0;
    activeAction = 0;
    statusMsg[0] = 0;
    ui_init_keys();
#if SDSUPPORT
    cwd[0]='/';
    cwd[1]=0;
    folderLevel=0;
#endif
    UI_STATUS(UI_TEXT_PRINTER_READY);
}
void UIDisplay::initialize()
{
#if UI_DISPLAY_TYPE>0
    initializeLCD();
#if UI_DISPLAY_TYPE==3
    // I don't know why but after power up the lcd does not come up
    // but if I reinitialize i2c and the lcd again here it works.
    HAL::delayMilliseconds(10);
    HAL::i2cInit(UI_I2C_CLOCKSPEED);
    initializeLCD();
#endif
    // SHOW STARTUP SCREEN HERE
    randomSeed(analogRead(0));
#if NO_SCREEN_ANIMATION==true
    printRowP(0, PSTR(UI_PRINTER_NAME));
    printRowP(1, PSTR(UI_PRINTER_COMPANY));
    printRowP(3, versionString);
#else    
    scrollVertRow(0, PSTR(UI_PRINTER_NAME), true, false);
    scrollHorzRow(1, PSTR(UI_PRINTER_COMPANY), true, random(4) > 2);
    randomRow(3, versionString, true);
#endif
    delay(2500);
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==0 && (BEEPER_TYPE==2 || defined(UI_HAS_I2C_KEYS))
    // Make sure the beeper is off
    HAL::i2cStartWait(UI_I2C_KEY_ADDRESS+I2C_WRITE);
    HAL::i2cWrite(255); // Disable beeper, enable read for other pins.
    HAL::i2cStop();
#endif
}
#if UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2 || UI_DISPLAY_TYPE==3
void UIDisplay::createChar(uint8_t location,const uint8_t PROGMEM charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    lcdCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i=0; i<8; i++)
    {
        lcdPutChar(pgm_read_byte(&(charmap[i])));
    }
}


void UIDisplay::printRow(byte r, byte x, char *txt, byte xChar)
{    
 byte cols=0;
 uint8_t len;
 char c;

// Com::print("Enter Row");
 // Set row
 if(r >= UI_ROWS) return;
 #if UI_DISPLAY_TYPE==3
  lcdStartWrite();
#endif
  lcdWriteByte(128 + HAL::readFlashByte((const char *)&LCDLineOffsets[r]),0); // Position cursor
 
// Com::print("shift:");
//Com::print(shift);

  if (shift>0 && (len = strlen(txt)) > UI_COLS) 
      txt += RMath::min(shift, len-UI_COLS);

 while(cols<x)
   {
   lcdPutChar(' ');
   cols++; 
   }
   
 txt += xChar;
// Com::print("name");
 while(cols<UI_COLS && (c=*txt) != 0x00)
   {
   lcdPutChar(c);
   txt++;
   cols++;
   }
// Com::print("name done");
 
 while(cols<UI_COLS) 
   {
   lcdPutChar(' ');
   cols++; 
   }
#if UI_DISPLAY_TYPE==3
  lcdStopWrite();
#endif
#if UI_HAS_KEYS==1 && UI_HAS_I2C_ENCODER>0
 ui_check_slow_encoder();
#endif

// Com::print("leave done\n\n");

}

void UIDisplay::printRow(byte r,char *txt) {
  printRow(r,0,txt,0);  
}

#endif

void UIDisplay::printRowP(uint8_t r,PGM_P txt)
{
    if(r >= UI_ROWS) return;
    col=0;
    addStringP(txt);
    printCols[col]=0;
    printRow(r,printCols);
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
        printCols[col++]='-';
    if(digits<6)
        while(dig<digits)
        {
            *--str = fillChar; //' ';
            dig++;
        }
    while(*str && col<MAX_COLS)
    {
        printCols[col++] = *str;
        str++;
    }
}
void UIDisplay::addLong(long value,char digits)
{
    uint8_t dig = 0,neg=0;
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
        printCols[col++]='-';
    if(digits<=11)
        while(dig<digits)
        {
            *--str = ' ';
            dig++;
        }
    while(*str && col<MAX_COLS)
    {
        printCols[col++] = *str;
        str++;
    }
}
const float roundingTable[] PROGMEM = {0.5,0.05,0.005,0.0005};
void UIDisplay::addFloat(float number, char fixdigits,uint8_t digits)
{
    // Handle negative numbers
    if (number < 0.0)
    {
        printCols[col++]='-';
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
        printCols[col++]='.';
    }

    // Extract digits from the remainder one at a time
    while (col<MAX_COLS && digits-- > 0)
    {
        remainder *= 10.0;
        uint8_t toPrint = uint8_t(remainder);
        printCols[col++] = '0'+toPrint;
        remainder -= toPrint;
    }
}
void UIDisplay::addStringP(FSTRINGPARAM(text))
{
    while(col<MAX_COLS)
    {
        uint8_t c = HAL::readFlashByte(text++);
        if(c==0) return;
        printCols[col++]=c;
    }
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

void UIDisplay::parse(char *txt,bool ram)
{
    int ivalue=0;
    float fvalue=0;
    while(col<MAX_COLS)
    {
        char c=(ram ? *(txt++) : pgm_read_byte(txt++));
        if(c==0) break; // finished
        if(c!='%')
        {
            printCols[col++]=c;
            continue;
        }
        // dynamic parameter, parse meaning and replace
        char c1=(ram ? *(txt++) : pgm_read_byte(txt++));
        char c2=(ram ? *(txt++) : pgm_read_byte(txt++));
        switch(c1)
        {
        case '%':
            if(c2=='%' && col<MAX_COLS)
                printCols[col++]='%';
            break;
        case 'a': // Acceleration settings
            if(c2=='x') addFloat(Printer::maxAccelerationMMPerSquareSecond[0],5,0);
            else if(c2=='y') addFloat(Printer::maxAccelerationMMPerSquareSecond[1],5,0);
            else if(c2=='z') addFloat(Printer::maxAccelerationMMPerSquareSecond[2],5,0);
            else if(c2=='X') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[0],5,0);
            else if(c2=='Y') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[1],5,0);
            else if(c2=='Z') addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[2],5,0);
            else if(c2=='j') addFloat(Printer::maxJerk,3,1);
#if DRIVE_SYSTEM!=3
            else if(c2=='J') addFloat(Printer::maxZJerk,3,1);
#endif
            break;

        case 'd':
            if(c2=='o') addStringP(Printer::debugEcho()?ui_text_on:ui_text_off);
            else if(c2=='i') addStringP(Printer::debugInfo()?ui_text_on:ui_text_off);
            else if(c2=='e') addStringP(Printer::debugErrors()?ui_text_on:ui_text_off);
            else if(c2=='d') addStringP(Printer::debugDryrun()?ui_text_on:ui_text_off);
            break;

        case 'e': // Extruder temperature
            if(c2=='r')   // Extruder relative mode
            {
                addStringP(Printer::relativeExtruderCoordinateMode?ui_yes:ui_no);
                break;
            }
            if(Printer::flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT)
            {
                addStringP(PSTR("def"));
                break;
            }
            ivalue = UI_TEMP_PRECISION;
            if(c2=='c') fvalue=Extruder::current->tempControl.currentTemperatureC;
            else if(c2>='0' && c2<='9') fvalue=extruder[c2-'0'].tempControl.currentTemperatureC;
            else if(c2=='b') fvalue=Extruder::getHeatedBedTemperature();
            else if(c2=='B')
            {
                ivalue=0;
                fvalue=Extruder::getHeatedBedTemperature();
            }
            addFloat(fvalue,3,ivalue);
            break;
        case 'E': // Target extruder temperature
            if(c2=='c') fvalue=Extruder::current->tempControl.targetTemperatureC;
            else if(c2>='0' && c2<='9') fvalue=extruder[c2-'0'].tempControl.targetTemperatureC;
#if HAVE_HEATED_BED
            else if(c2=='b') fvalue=heatedBedController.targetTemperatureC;
#endif
            addFloat(fvalue,3,0 /*UI_TEMP_PRECISION*/);
            break;
#if FAN_PIN > -1
        case 'F': // FAN speed
            if(c2=='s') addInt(Printer::getFanSpeed()*100/255,3);
            break;
#endif
        case 'f':
            if(c2=='x') addFloat(Printer::maxFeedrate[0],5,0);
            else if(c2=='y') addFloat(Printer::maxFeedrate[1],5,0);
            else if(c2=='z') addFloat(Printer::maxFeedrate[2],5,0);
            else if(c2=='X') addFloat(Printer::homingFeedrate[0],5,0);
            else if(c2=='Y') addFloat(Printer::homingFeedrate[1],5,0);
            else if(c2=='Z') addFloat(Printer::homingFeedrate[2],5,0);
            break;
        case 'i':
            if(c2=='s') addLong(stepperInactiveTime,4);
            else if(c2=='p') addLong(maxInactiveTime,4);
            break;
        case 'O': // ops related stuff
            break;
        case 'l':
            if(c2=='a') addInt(lastAction,4);
            else if(c2=='o') addStringP(Printer::debugEcho()?ui_text_on:ui_text_off);        // Lights on/off        
            break;
        case 'o':
            if(c2=='s')
            {
#if SDSUPPORT
                if(sd.sdactive && sd.sdmode)
                {
                    addStringP(PSTR( UI_TEXT_PRINT_POS));
                    unsigned long percent;
                    if(sd.filesize<20000000) percent=sd.sdpos*100/sd.filesize;
                    else percent = (sd.sdpos>>8)*100/(sd.filesize>>8);
                    addInt((int)percent,3);
                    if(col<MAX_COLS)
                        printCols[col++]='%';
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
                addInt(Printer::feedrateMultiply,3);
                break;
            }
            // Extruder output level
            if(c2>='0' && c2<='9') ivalue=pwm_pos[c2-'0'];
#if HAVE_HEATED_BED
            else if(c2=='b') ivalue=pwm_pos[heatedBedController.pwmIndex];
#endif
            else if(c2=='C') ivalue=pwm_pos[Extruder::current->id];
            ivalue=(ivalue*100)/255;
            addInt(ivalue,3);
            if(col<MAX_COLS)
                printCols[col++]='%';
            break;
        case 'x':
            if(c2>='0' && c2<='3')
                if(c2=='0')
                    fvalue = Printer::realXPosition();
                else if(c2=='1')
                    fvalue = Printer::realYPosition();
                else if(c2=='2')
                    fvalue = Printer::realZPosition();
                else
                    fvalue = (float)Printer::currentPositionSteps[3]*Printer::invAxisStepsPerMM[3];
            addFloat(fvalue,3,2);
            break;
        case 'y':
#if DRIVE_SYSTEM==3
            if(c2>='0' && c2<='3') fvalue = (float)Printer::currentDeltaPositionSteps[c2-'0']*Printer::invAxisStepsPerMM[c2-'0'];
            addFloat(fvalue,3,2);
#endif
            break;
        case 'X': // Extruder related
#if NUM_EXTRUDER>0
            if(c2>='0' && c2<='9')
            {
                addStringP(Extruder::current->id==c2-'0'?ui_selected:ui_unselected);
            }
#ifdef TEMP_PID
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
                if(hm == 1)
                    addStringP(PSTR(UI_TEXT_STRING_HM_PID));
                else if(hm == 3)
                    addStringP(PSTR(UI_TEXT_STRING_HM_DEADTIME));
                else if(hm == 2)
                    addStringP(PSTR(UI_TEXT_STRING_HM_SLOWBANG));
                else
                    addStringP(PSTR(UI_TEXT_STRING_HM_BANGBANG));
            }
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
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
            break;
        case 'S':
            if(c2=='x') addFloat(Printer::axisStepsPerMM[0],3,1);
            if(c2=='y') addFloat(Printer::axisStepsPerMM[1],3,1);
            if(c2=='z') addFloat(Printer::axisStepsPerMM[2],3,1);
            if(c2=='e') addFloat(Extruder::current->stepsPerMM,3,1);
            break;
        case 'P':
            if(c2=='N') addStringP(PSTR(UI_PRINTER_NAME));
            break;
        case 'U':
            if(c2=='t')   // Printing time
            {
#if EEPROM_MODE!=0
                bool alloff = true;
                for(uint8_t i=0; i<NUM_EXTRUDER; i++)
                    if(tempController[i]->targetTemperatureC>15) alloff = false;

                long seconds = (alloff ? 0 : (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME);
                long tmp = seconds/86400;
                seconds-=tmp*86400;
                addInt(tmp,5);
                addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                tmp=seconds/3600;
                addInt(tmp,2);
                addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                seconds-=tmp*3600;
                tmp = seconds/60;
                addInt(tmp,2,0);
                addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
#endif
            }
            else if(c2=='f')     // Filament usage
            {
#if EEPROM_MODE!=0
                float dist = Printer::filamentPrinted*0.001+HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
                addFloat(dist,6,1);
#endif
            }
        }
    }
    printCols[col] = 0;
}
void UIDisplay::setStatusP(PGM_P txt)
{
    uint8_t i=0;
    while(i<16)
    {
        uint8_t c = pgm_read_byte(txt++);
        if(!c) break;
        statusMsg[i++] = c;
    }
    statusMsg[i]=0;
}
void UIDisplay::setStatus(char *txt)
{
    uint8_t i=0;
    while(*txt && i<16)
        statusMsg[i++] = *txt++;
    statusMsg[i]=0;
}

const UIMenu * const ui_pages[UI_NUM_PAGES] PROGMEM = UI_PAGES;
#if SDSUPPORT
uint8_t nFilesOnCard;
void UIDisplay::updateSDFileCount() {
  dir_t* p = NULL;
  byte offset = menuTop[menuLevel];
  SdBaseFile *root = sd.fat.vwd();
  
  root->rewind();
  nFilesOnCard = 0;
  while ((p = root->getLongFilename(p, tempLongFilename))) {
      if (! (DIR_IS_FILE(p) || DIR_IS_SUBDIR(p))) 
        continue;
      if (folderLevel>=SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.')) 
        continue;
      nFilesOnCard++;
      if (nFilesOnCard==254) 
        return;
  }
}

void getSDFilenameAt(byte filePos,char *filename) {
  dir_t* p;
  byte c=0;
  SdBaseFile *root = sd.fat.vwd();
  root->rewind();

  while ((p = root->getLongFilename(p, tempLongFilename))) {
    if (!DIR_IS_FILE(p) && !DIR_IS_SUBDIR(p)) continue;
    if(uid.folderLevel>=SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.')) continue;
    if (filePos) {
      filePos--;
      continue;
    }
    for (uint8_t i = 0; i < 11; i++) 
      {
      if (i == 8)
         filename[c++]='.';
      if (p->name[i] == ' ')
        continue;
      filename[c++]=tolower(p->name[i]);
      }
    if(DIR_IS_SUBDIR(p)) filename[c++]='/'; // Set marker for directory
      break;
  }
  filename[c]=0;
}

bool UIDisplay::isDirname(char *name)
{
    while(*name) name++;
    name--;
    return *name=='/';
}

void UIDisplay::goDir(char *name)
{
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
}
void UIDisplay::sdrefresh(byte &r) {
  dir_t* p = NULL;
  byte offset = menuTop[menuLevel];
  SdBaseFile *root;
  byte length, skip;
    
  sd.fat.chdir(cwd);
  root = sd.fat.vwd();
  root->rewind();  
  skip = (offset>0?offset-1:0);
  
  while (r+offset<nFilesOnCard+1 && r<UI_ROWS && (p = root->getLongFilename(p, tempLongFilename))) {
    // done if past last used entry
      // skip deleted entry and entries for . and  ..
      // only list subdirectories and files
      if ((DIR_IS_FILE(p) || DIR_IS_SUBDIR(p)))
        {
      if(folderLevel>=SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.')) 
        continue;
      if(skip>0) {skip--;continue;}
      col=0;
      if(r+offset==menuPos[menuLevel])
         printCols[col++]='>';
      else
         printCols[col++]=' ';
      // print file name with possible blank fill
      if(DIR_IS_SUBDIR(p))
        printCols[col++] = 6; // Prepend folder symbol
      length =  RMath::min((int)strlen(tempLongFilename), MAX_COLS-col);
      memcpy(printCols+col, tempLongFilename, length);
      col += length;
      printCols[col] = 0;
      transitionInRow(r,(PGM_P)printCols, false);
      r++;
      }
  }
}
#endif
// Refresh current menu page
void UIDisplay::refreshPage()
{
    uint8_t r;
    uint8_t mtype;
    
#if HARDWARE_BED_LEVELING==true && HARDWARE_BED_LEVELING_BEFORE_USING==true
    if (!Printer::bBedHasBeenLeveled)
      {
      EEPROM::storeHardwareBedLeveled(true);        
      executeAction(UI_ACTION_LEVEL_BED);
      }
#endif

#if UI_AUTORETURN_TO_MENU_AFTER!=0
    // Reset timeout on menu back when user active on menu
    if (uid.encoderLast != encoderStartScreen)
      ui_autoreturn_time=HAL::timeInMilliseconds()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
    encoderStartScreen = uid.encoderLast;
    iScreenTransition = menuLevel == oldMenuLevel ? 0 : random(0, MAX_SCREEN_TRANSITIONS);
    
    if(menuLevel==0)
    {
        UIMenu *men = (UIMenu*)pgm_read_word(&(ui_pages[menuPos[0]]));
        uint8_t nr = pgm_read_word_near(&(men->numEntries));
        UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        for(r=0; r<nr && r<UI_ROWS; r++)
        {
            UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[r]));
            col=0;
            parse((char*)pgm_read_word(&(ent->text)),false);
            printRow(r,(char*)printCols);
        }
    }
    else
    {
        UIMenu *men = (UIMenu*)menu[menuLevel];
        uint8_t nr = pgm_read_word_near((void*)&(men->numEntries));
        mtype = pgm_read_byte((void*)&(men->menuType));
        uint8_t offset = menuTop[menuLevel];
        UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        for(r=0; r+offset<nr && r<UI_ROWS; )
        {
            UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[r+offset]));
            if(!ent->showEntry())
            {
                offset++;
                continue;
            }
            unsigned char entType = pgm_read_byte(&(ent->menuType));
            unsigned int entAction = pgm_read_word(&(ent->action));
            col=0;
            if(entType>=2 && entType<=4)
            {
                if(r+offset==menuPos[menuLevel] && activeAction!=entAction)
                    printCols[col++]=CHAR_SELECTOR;
                else if(activeAction==entAction)
                    printCols[col++]=CHAR_SELECTED;
                else
                    printCols[col++]=' ';
            }
            parse((char*)pgm_read_word(&(ent->text)),false);
            if(entType==2)   // Draw submenu marker at the right side
            {
                while(col<UI_COLS-1) printCols[col++]=' ';
                printCols[col++] = CHAR_RIGHT; // Arrow right
            }
            printCols[col] = 0;
            transitionInRow(r, (PGM_P)printCols, false);
            r++;
        }
    }
#if SDSUPPORT
    if(mtype==1)
    {
        sdrefresh(r);
    }
#endif
    oldMenuLevel = menuLevel;      
    printCols[0]=0;
    while(r<UI_ROWS)
        printRow(r++,printCols);
}
void UIDisplay::pushMenu(void *men,bool refresh)
{
    if(men==menu[menuLevel])
    {
        refreshPage();
        return;
    }
    if(menuLevel==4) return;
    menuLevel++;
    menu[menuLevel]=men;
    menuTop[menuLevel] = menuPos[menuLevel] = 0;
#if SDSUPPORT
    UIMenu *men2 = (UIMenu*)menu[menuLevel];
    if(pgm_read_byte(&(men2->menuType))==1) // Open files list
        updateSDFileCount();
#endif
    if(refresh)
        refreshPage();
}
void UIDisplay::okAction()
{
#if UI_HAS_KEYS==1
    if(menuLevel==0)   // Enter menu
    {
        menuLevel = 1;
        menuTop[1] = menuPos[1] = 0;
        menu[1] = (void*)&ui_menu_main;
        BEEP_SHORT
        return;
    }
    UIMenu *men = (UIMenu*)menu[menuLevel];
    //uint8_t nr = pgm_read_word_near(&(menu->numEntries));
    uint8_t mtype = pgm_read_byte(&(men->menuType));
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
    unsigned char entType = pgm_read_byte(&(ent->menuType));// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action
    int action = pgm_read_word(&(ent->action));
    if(mtype==3)   // action menu
    {
        action = pgm_read_word(&(men->id));
        finishAction(action);
        executeAction(UI_ACTION_BACK);
        return;
    }
    if(mtype==2 && entType==4)   // Modify action
    {
        if(activeAction)   // finish action
        {
            finishAction(action);
            activeAction = 0;
        }
        else
            activeAction = action;
        return;
    }
#if SDSUPPORT
    if(mtype==1)
    {
        if(menuPos[menuLevel]==0)   // Selected back instead of file
        {
            executeAction(UI_ACTION_BACK);
            return;
        }
        uint8_t filePos = menuPos[menuLevel]-1;
        char filename[14];
        getSDFilenameAt(filePos,filename);
        if(isDirname(filename))   // Directory change selected
        {
            goDir(filename);
            menuTop[menuLevel]=0;
            menuPos[menuLevel]=1;
            refreshPage();
            oldMenuLevel = -1;            
            return;
        }
        menuLevel--;
        men = (UIMenu*)menu[menuLevel];
        entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
        switch(pgm_read_word(&(ent->action)))
        {
        case UI_ACTION_SD_PRINT:
            if(sd.sdactive)
            {
                sd.sdmode = false;
                sd.file.close();
                if (sd.file.open(filename, O_READ))
                {
                    Com::printF(Com::tFileOpened,filename);
                    Com::printFLN(Com::tSpaceSizeColon,sd.file.fileSize());
                    sd.sdpos = 0;
                    sd.filesize = sd.file.fileSize();
                    Com::printFLN(Com::tFileSelected);
                    sd.sdmode = true; // Start print immediately
                    menuLevel = 0;
                    BEEP_LONG;
                }
                else
                {
                    Com::printFLN(Com::tFileOpenFailed);
                }
            }
            break;
        case UI_ACTION_SD_DELETE:
            if(sd.sdactive)
            {
                sd.sdmode = false;
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
    }
#endif
    if(entType==2)   // Enter submenu
    {
        pushMenu((void*)action,false);
        BEEP_SHORT
        return;
    }
    if(entType==3)
    {
        executeAction(action);
        return;
    }
    executeAction(UI_ACTION_BACK);
#endif
}
#define INCREMENT_MIN_MAX(a,steps,_min,_max) a+=increment*steps;if(a<(_min)) a=_min;else if(a>(_max)) a=_max;
void UIDisplay::nextPreviousAction(int8_t next)
{
    millis_t actTime = HAL::timeInMilliseconds();
    millis_t dtReal;
    millis_t dt = dtReal = actTime-lastNextPrev;
    lastNextPrev = actTime;
    if(dt<SPEED_MAX_MILLIS) dt = SPEED_MAX_MILLIS;
    if(dt>SPEED_MIN_MILLIS) {dt = SPEED_MIN_MILLIS;lastNextAccumul = 1;}
    float f = (float)(SPEED_MIN_MILLIS-dt)/(float)(SPEED_MIN_MILLIS-SPEED_MAX_MILLIS);
    lastNextAccumul = 1.0f+(float)SPEED_MAGNIFICATION*f*f*f;

#if UI_HAS_KEYS==1
    if(menuLevel==0)
    {
        lastSwitch = HAL::timeInMilliseconds();
        if((UI_INVERT_MENU_DIRECTION && next<0) || (!UI_INVERT_MENU_DIRECTION && next>0))
        {
            menuPos[0]++;
            if(menuPos[0]>=UI_NUM_PAGES)
                menuPos[0]=0;
        }
        else
        {
            if(menuPos[0]==0)
                menuPos[0]=UI_NUM_PAGES-1;
            else
                menuPos[0]--;
        }
        return;
    }
    UIMenu *men = (UIMenu*)menu[menuLevel];
    uint8_t nr = pgm_read_word_near(&(men->numEntries));
    uint8_t mtype = HAL::readFlashByte((const prog_char*)&(men->menuType));
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
    UIMenuEntry *testEnt;
    uint8_t entType = HAL::readFlashByte((const prog_char*)&(ent->menuType));// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command
    int action = pgm_read_word(&(ent->action));
    if(mtype==2 && activeAction==0)   // browse through menu items
    {
        if((UI_INVERT_MENU_DIRECTION && next<0) || (!UI_INVERT_MENU_DIRECTION && next>0))
        {
            while(menuPos[menuLevel]+1<nr)
            {
                menuPos[menuLevel]++;
                testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if(testEnt->showEntry())
                    break;
            }
        }
        else if(menuPos[menuLevel]>0)
        {
            while(menuPos[menuLevel]>0)
            {
                menuPos[menuLevel]--;
                testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if(testEnt->showEntry())
                    break;
            }
        }
        if(menuTop[menuLevel]>menuPos[menuLevel])
            menuTop[menuLevel]=menuPos[menuLevel];
        else if(menuTop[menuLevel]+UI_ROWS-1<menuPos[menuLevel])
            menuTop[menuLevel]=menuPos[menuLevel]+1-UI_ROWS;
        return;
    }
#if SDSUPPORT
    if(mtype==1)   // SD listing
    {
        if((UI_INVERT_MENU_DIRECTION && next<0) || (!UI_INVERT_MENU_DIRECTION && next>0))
        {
            if(menuPos[menuLevel]<nFilesOnCard) menuPos[menuLevel]++;
        }
        else if(menuPos[menuLevel]>0)
            menuPos[menuLevel]--;
        if(menuTop[menuLevel]>menuPos[menuLevel])
            menuTop[menuLevel]=menuPos[menuLevel];
        else if(menuTop[menuLevel]+UI_ROWS-1<menuPos[menuLevel])
            menuTop[menuLevel]=menuPos[menuLevel]+1-UI_ROWS;
        return;
    }
#endif
    if(mtype==3) action = pgm_read_word(&(men->id));
    else action=activeAction;
    int8_t increment = next;
    switch(action)
    {
    case UI_ACTION_FANSPEED:
        Commands::setFanSpeed(Printer::getFanSpeed()+increment*3,false);
        break;
    case UI_ACTION_XPOSITION:
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01*(float)increment*lastNextAccumul;
            if(fabs(d)*2000>Printer::maxFeedrate[X_AXIS]*dtReal)
                d *= Printer::maxFeedrate[X_AXIS]*dtReal/(2000*fabs(d));
            long steps = (long)(d*Printer::axisStepsPerMM[X_AXIS]);
            steps = ( increment<0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInSteps(steps,0,0,0,Printer::maxFeedrate[2],true,true);
        }
#else
        PrintLine::moveRelativeDistanceInSteps(increment,0,0,0,Printer::homingFeedrate[0],true,true);
#endif
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_YPOSITION:
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01*(float)increment*lastNextAccumul;
            if(fabs(d)*2000>Printer::maxFeedrate[Y_AXIS]*dtReal)
                d *= Printer::maxFeedrate[Y_AXIS]*dtReal/(2000*fabs(d));
            long steps = (long)(d*Printer::axisStepsPerMM[Y_AXIS]);
            steps = ( increment<0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInSteps(0,steps,0,0,Printer::maxFeedrate[2],true,true);
        }
#else
        PrintLine::moveRelativeDistanceInSteps(0,increment,0,0,Printer::homingFeedrate[1],true,true);
#endif
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_ZPOSITION:
#if UI_SPEEDDEPENDENT_POSITIONING
        {
            float d = 0.01*(float)increment*lastNextAccumul;
            if(fabs(d)*2000>Printer::maxFeedrate[Z_AXIS]*dtReal)
                d *= Printer::maxFeedrate[Z_AXIS]*dtReal/(2000*fabs(d));
            long steps = (long)(d*Printer::axisStepsPerMM[Z_AXIS]);
            steps = ( increment<0 ? RMath::min(steps,(long)increment) : RMath::max(steps,(long)increment));
            PrintLine::moveRelativeDistanceInSteps(0,0,steps,0,Printer::maxFeedrate[2],true,true);
        }
#else
        PrintLine::moveRelativeDistanceInSteps(0,0,increment,0,Printer::homingFeedrate[2],true,true);
#endif
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_XPOSITION_FAST:
        PrintLine::moveRelativeDistanceInSteps(Printer::axisStepsPerMM[0]*increment,0,0,0,Printer::homingFeedrate[0],true,true);
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_YPOSITION_FAST:
        PrintLine::moveRelativeDistanceInSteps(0,Printer::axisStepsPerMM[1]*increment,0,0,Printer::homingFeedrate[1],true,true);
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_ZPOSITION_FAST:
        PrintLine::moveRelativeDistanceInSteps(0,0,Printer::axisStepsPerMM[2]*increment,0,Printer::homingFeedrate[2],true,true);
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_EPOSITION:
        PrintLine::moveRelativeDistanceInSteps(0,0,0,Printer::axisStepsPerMM[3]*increment,UI_SET_EXTRUDER_FEEDRATE,true,false);
        Commands::printCurrentPosition();
        break;
    case UI_ACTION_HEATED_BED_TEMP:
#if HAVE_HEATED_BED==true
    {
        int tmp = (int)heatedBedController.targetTemperatureC;
        if(tmp<UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
        tmp+=increment;
        if(tmp==1) tmp = UI_SET_MIN_HEATED_BED_TEMP;
        if(tmp<UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
        else if(tmp>UI_SET_MAX_HEATED_BED_TEMP) tmp = UI_SET_MAX_HEATED_BED_TEMP;
        Extruder::setHeatedBedTemperature(tmp);
    }
#endif
    break;
    case UI_ACTION_EXTRUDER0_TEMP:
    {
        int tmp = (int)extruder[0].tempControl.targetTemperatureC;
        if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
        tmp+=increment;
        if(tmp==1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
        else if(tmp>UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp,0);
    }
    break;
    case UI_ACTION_EXTRUDER1_TEMP:
#if NUM_EXTRUDER>1
    {
        int tmp = (int)extruder[1].tempControl.targetTemperatureC;
        tmp+=increment;
        if(tmp==1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
        else if(tmp>UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp,1);
    }
#endif
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
        Com::printFLN(Com::tFlowMultiply,(int)Printer::extrudeMultiply);
    }
    break;
    case UI_ACTION_STEPPER_INACTIVE:
        INCREMENT_MIN_MAX(stepperInactiveTime,60,0,9999);
        break;
    case UI_ACTION_MAX_INACTIVE:
        INCREMENT_MIN_MAX(maxInactiveTime,60,0,9999);
        break;
    case UI_ACTION_PRINT_ACCEL_X:
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[0],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_PRINT_ACCEL_Y:
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[1],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_PRINT_ACCEL_Z:
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[2],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_MOVE_ACCEL_X:
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[0],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_MOVE_ACCEL_Y:
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[1],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_MOVE_ACCEL_Z:
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[2],100,0,10000);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_MAX_JERK:
        INCREMENT_MIN_MAX(Printer::maxJerk,0.1,1,99.9);
        break;
#if DRIVE_SYSTEM!=3
    case UI_ACTION_MAX_ZJERK:
        INCREMENT_MIN_MAX(Printer::maxZJerk,0.1,0.1,99.9);
        break;
#endif
    case UI_ACTION_HOMING_FEEDRATE_X:
        INCREMENT_MIN_MAX(Printer::homingFeedrate[0],1,5,1000);
        break;
    case UI_ACTION_HOMING_FEEDRATE_Y:
        INCREMENT_MIN_MAX(Printer::homingFeedrate[1],1,5,1000);
        break;
    case UI_ACTION_HOMING_FEEDRATE_Z:
        INCREMENT_MIN_MAX(Printer::homingFeedrate[2],1,1,1000);
        break;
    case UI_ACTION_MAX_FEEDRATE_X:
        INCREMENT_MIN_MAX(Printer::maxFeedrate[0],1,1,1000);
        break;
    case UI_ACTION_MAX_FEEDRATE_Y:
        INCREMENT_MIN_MAX(Printer::maxFeedrate[1],1,1,1000);
        break;
    case UI_ACTION_MAX_FEEDRATE_Z:
        INCREMENT_MIN_MAX(Printer::maxFeedrate[2],1,1,1000);
        break;
    case UI_ACTION_STEPS_X:
        INCREMENT_MIN_MAX(Printer::axisStepsPerMM[0],0.1,0,999);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_STEPS_Y:
        INCREMENT_MIN_MAX(Printer::axisStepsPerMM[1],0.1,0,999);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_STEPS_Z:
        INCREMENT_MIN_MAX(Printer::axisStepsPerMM[2],0.1,0,999);
        Printer::updateDerivedParameter();
        break;
    case UI_ACTION_BAUDRATE:
#if EEPROM_MODE!=0
    {
        char p=0;
        long rate;
        do
        {
            rate = pgm_read_dword(&(baudrates[p]));
            if(rate==baudrate) break;
            p++;
        }
        while(rate!=0);
        if(rate==0) p-=2;
        p+=increment;
        if(p<0) p = 0;
        rate = pgm_read_dword(&(baudrates[p]));
        if(rate==0) p--;
        baudrate = pgm_read_dword(&(baudrates[p]));
    }
#endif
    break;
#ifdef TEMP_PID
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
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
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
    ui_autoreturn_time=HAL::timeInMilliseconds()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
}

void UIDisplay::finishAction(int action)
{
}
// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.
void UIDisplay::executeAction(int action)
{
#if UI_HAS_KEYS==1
    bool skipBeep = false;
    if(action & UI_ACTION_TOPMENU)   // Go to start menu
    {
        action -= UI_ACTION_TOPMENU;
        menuLevel = 0;
    }
    if(action>=2000 && action<3000)
    {
        setStatusP(ui_action);
    }
    else
        switch(action)
        {
        case UI_ACTION_OK:
            okAction();
            skipBeep=true; // Prevent double beep
            break;
        case UI_ACTION_BACK:
            if(menuLevel>0) menuLevel--;
            activeAction = 0;
            break;
        case UI_ACTION_NEXT:
            nextPreviousAction(1);
            break;
        case UI_ACTION_PREVIOUS:
            nextPreviousAction(-1);
            break;
        case UI_ACTION_MENU_UP:
            if(menuLevel>0) menuLevel--;
            break;
        case UI_ACTION_TOP_MENU:
            menuLevel = 0;
            break;
        case UI_ACTION_EMERGENCY_STOP:
            Commands::emergencyStop();
            break;
        case UI_ACTION_HOME_ALL:
            Printer::homeAxis(true,true,true);
            Commands::printCurrentPosition();
            break;
        case UI_ACTION_HOME_X:
            Printer::homeAxis(true,false,false);
            Commands::printCurrentPosition();
            break;
        case UI_ACTION_HOME_Y:
            Printer::homeAxis(false,true,false);
            Commands::printCurrentPosition();
            break;
        case UI_ACTION_HOME_Z:
            Printer::homeAxis(false,false,true);
            Commands::printCurrentPosition();
            break;
        case UI_ACTION_SET_ORIGIN:
            Printer::setOrigin(0,0,0);
            break;
        case UI_ACTION_DEBUG_ECHO:
            if(Printer::debugEcho()) Printer::debugLevel-=1;
            else Printer::debugLevel+=1;
            break;
        case UI_ACTION_DEBUG_INFO:
            if(Printer::debugInfo()) Printer::debugLevel-=2;
            else Printer::debugLevel+=2;
            break;
        case UI_ACTION_DEBUG_ERROR:
            if(Printer::debugErrors()) Printer::debugLevel-=4;
            else Printer::debugLevel+=4;
            break;
        case UI_ACTION_DEBUG_DRYRUN:
            if(Printer::debugDryrun()) Printer::debugLevel-=8;
            else Printer::debugLevel+=8;
            if(Printer::debugDryrun())   // simulate movements without printing
            {
                Extruder::setTemperatureForExtruder(0,0);
#if NUM_EXTRUDER>1
                Extruder::setTemperatureForExtruder(0,1);
#endif
#if NUM_EXTRUDER>2
                Extruder::setTemperatureForExtruder(0,2);
#endif
#if HAVE_HEATED_BED==true
                Extruder::setHeatedBedTemperature(0);
#endif
            }
            break;
        case UI_ACTION_POWER:
            Commands::waitUntilEndOfAllMoves();
            SET_OUTPUT(PS_ON_PIN); //GND
            TOGGLE(PS_ON_PIN);
            break;

#if HARDWARE_BED_LEVELING==true
         case UI_ACTION_LEVEL_BED:
            printRow(2, "");
            GCode::executeFString(PSTR("G28\nG1 X10 Y10 E0\nG28 Z0\n"));
            printRow(0, "Adjust Bed Area 1");
            printRow(1, "Continue to Next?");
            waitForKey();
            GCode::executeFString(PSTR("G1 Z10\nG1 X290 Y10\nG28 Z0\n"));
            printRow(0, "Adjust Bed Area 2");
            printRow(1, "Continue to Next?");
            waitForKey();
            GCode::executeFString(PSTR("G1 Z10\nG1 X150 Y290\nG28 Z0\n"));
            printRow(0, "Adjust Bed Area 3");
            printRow(1, "Continue to Next?");
            waitForKey();
            GCode::executeFString(PSTR("G1 Z20\nG28 X0 Y0\n"));
            printRow(0, "Adjust Bed Area 4");
            printRow(1, "Done! Click to End");
            waitForKey();
            break;
#endif
        
#if CASE_LIGHTS_PIN > 0
        case UI_ACTION_LIGHTS_ONOFF:
            WRITE(CASE_LIGHTS_PIN, HIGH);
            UI_STATUS(UI_TEXT_LIGHTS_ONOFF);
          break;
#endif
            
        case UI_ACTION_PREHEAT_PLA:
            UI_STATUS(UI_TEXT_PREHEAT_PLA);
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,0);
#if NUM_EXTRUDER>1
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,1);
#endif
#if NUM_EXTRUDER>2
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA,2);
#endif
#if HAVE_HEATED_BED==true
            Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_PLA);
#endif
            break;
        case UI_ACTION_PREHEAT_ABS:
            UI_STATUS(UI_TEXT_PREHEAT_ABS);
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,0);
#if NUM_EXTRUDER>1
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,1);
#endif
#if NUM_EXTRUDER>2
            Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_ABS,2);
#endif
#if HAVE_HEATED_BED==true
            Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_ABS);
#endif
            break;
        case UI_ACTION_COOLDOWN:
            UI_STATUS(UI_TEXT_COOLDOWN);
            Extruder::setTemperatureForExtruder(0,0);
#if NUM_EXTRUDER>1
            Extruder::setTemperatureForExtruder(0,1);
#endif
#if NUM_EXTRUDER>2
            Extruder::setTemperatureForExtruder(0,2);
#endif
#if HAVE_HEATED_BED==true
            Extruder::setHeatedBedTemperature(0);
#endif
            break;
        case UI_ACTION_HEATED_BED_OFF:
#if HAVE_HEATED_BED==true
            Extruder::setHeatedBedTemperature(0);
#endif
            break;
        case UI_ACTION_EXTRUDER0_OFF:
            Extruder::setTemperatureForExtruder(0,0);
            break;
        case UI_ACTION_EXTRUDER1_OFF:
#if NUM_EXTRUDER>1
            Extruder::setTemperatureForExtruder(0,1);
#endif
            break;
        case UI_ACTION_EXTRUDER2_OFF:
#if NUM_EXTRUDER>2
            Extruder::setTemperatureForExtruder(0,2);
#endif
            break;
        case UI_ACTION_DISABLE_STEPPER:
            Printer::kill(true);
            break;
        case UI_ACTION_RESET_EXTRUDER:
            Printer::currentPositionSteps[3] = 0;
            break;
        case UI_ACTION_EXTRUDER_RELATIVE:
            Printer::relativeExtruderCoordinateMode=!Printer::relativeExtruderCoordinateMode;
            break;
        case UI_ACTION_SELECT_EXTRUDER0:
            Extruder::selectExtruderById(0);
            break;
        case UI_ACTION_SELECT_EXTRUDER1:
#if NUM_EXTRUDER>1
            Extruder::selectExtruderById(1);
#endif
            break;
        case UI_ACTION_SELECT_EXTRUDER2:
#if NUM_EXTRUDER>2
            Extruder::selectExtruderById(2);
#endif
            break;
#if EEPROM_MODE!=0
        case UI_ACTION_STORE_EEPROM:
            EEPROM::storeDataIntoEEPROM(false);
            pushMenu((void*)&ui_menu_eeprom_saved,false);
            BEEP_LONG;
            skipBeep = true;
            break;
        case UI_ACTION_LOAD_EEPROM:
            EEPROM::readDataFromEEPROM();
            Extruder::selectExtruderById(Extruder::current->id);
            pushMenu((void*)&ui_menu_eeprom_loaded,false);
            BEEP_LONG;
            skipBeep = true;
            break;
#endif
#if SDSUPPORT
        case UI_ACTION_SD_DELETE:
            if(sd.sdactive)
            {
                pushMenu((void*)&ui_menu_sd_fileselector,false);
            }
            else
            {
                UI_ERROR(UI_TEXT_NOSDCARD);
            }
            break;
        case UI_ACTION_SD_PRINT:
            if(sd.sdactive)
            {
                pushMenu((void*)&ui_menu_sd_fileselector,false);
            }
            break;
        case UI_ACTION_SD_PAUSE:
            if(sd.sdmode)
            {
                sd.sdmode = false;
            }
            break;
        case UI_ACTION_SD_CONTINUE:
            if(sd.sdactive)
            {
                sd.sdmode = true;
            }
            break;
        case UI_ACTION_SD_UNMOUNT:
            sd.unmount();
            break;
        case UI_ACTION_SD_MOUNT:
            sd.mount();
            break;
        case UI_ACTION_MENU_SDCARD:
            pushMenu((void*)&ui_menu_sd,false);
            break;
#endif
#if FAN_PIN>-1
        case UI_ACTION_FAN_OFF:
            Commands::setFanSpeed(0,false);
            break;
        case UI_ACTION_FAN_25:
            Commands::setFanSpeed(64,false);
            break;
        case UI_ACTION_FAN_50:
            Commands::setFanSpeed(128,false);
            break;
        case UI_ACTION_FAN_75:
            Commands::setFanSpeed(192,false);
            break;
        case UI_ACTION_FAN_FULL:
            Commands::setFanSpeed(255,false);
            break;
#endif
        case UI_ACTION_MENU_XPOS:
            pushMenu((void*)&ui_menu_xpos,false);
            break;
        case UI_ACTION_MENU_YPOS:
            pushMenu((void*)&ui_menu_ypos,false);
            break;
        case UI_ACTION_MENU_ZPOS:
            pushMenu((void*)&ui_menu_zpos,false);
            break;
        case UI_ACTION_MENU_XPOSFAST:
            pushMenu((void*)&ui_menu_xpos_fast,false);
            break;
        case UI_ACTION_MENU_YPOSFAST:
            pushMenu((void*)&ui_menu_ypos_fast,false);
            break;
        case UI_ACTION_MENU_ZPOSFAST:
            pushMenu((void*)&ui_menu_zpos_fast,false);
            break;
        case UI_ACTION_MENU_QUICKSETTINGS:
            pushMenu((void*)&ui_menu_quick,false);
            break;
        case UI_ACTION_MENU_EXTRUDER:
            pushMenu((void*)&ui_menu_extruder,false);
            break;
        case UI_ACTION_MENU_POSITIONS:
            pushMenu((void*)&ui_menu_positions,false);
            break;
#ifdef UI_USERMENU1
        case UI_ACTION_SHOW_USERMENU1:
            pushMenu((void*)&UI_USERMENU1,false);
            break;
#endif
#ifdef UI_USERMENU2
        case UI_ACTION_SHOW_USERMENU2:
            pushMenu((void*)&UI_USERMENU2,false);
            break;
#endif
#ifdef UI_USERMENU3
        case UI_ACTION_SHOW_USERMENU3:
            pushMenu((void*)&UI_USERMENU3,false);
            break;
#endif
#ifdef UI_USERMENU4
        case UI_ACTION_SHOW_USERMENU4:
            pushMenu((void*)&UI_USERMENU4,false);
            break;
#endif
#ifdef UI_USERMENU5
        case UI_ACTION_SHOW_USERMENU5:
            pushMenu((void*)&UI_USERMENU5,false);
            break;
#endif
#ifdef UI_USERMENU6
        case UI_ACTION_SHOW_USERMENU6:
            pushMenu((void*)&UI_USERMENU6,false);
            break;
#endif
#ifdef UI_USERMENU7
        case UI_ACTION_SHOW_USERMENU7:
            pushMenu((void*)&UI_USERMENU7,false);
            break;
#endif
#ifdef UI_USERMENU8
        case UI_ACTION_SHOW_USERMENU8:
            pushMenu((void*)&UI_USERMENU8,false);
            break;
#endif
#ifdef UI_USERMENU9
        case UI_ACTION_SHOW_USERMENU9:
            pushMenu((void*)&UI_USERMENU9,false);
            break;
#endif
#ifdef UI_USERMENU10
        case UI_ACTION_SHOW_USERMENU10:
            pushMenu((void*)&UI_USERMENU10,false);
            break;
#endif
        case UI_ACTION_X_UP:
            PrintLine::moveRelativeDistanceInSteps(Printer::axisStepsPerMM[0],0,0,0,Printer::homingFeedrate[0],false,true);
            break;
        case UI_ACTION_X_DOWN:
            PrintLine::moveRelativeDistanceInSteps(-Printer::axisStepsPerMM[0],0,0,0,Printer::homingFeedrate[0],false,true);
            break;
        case UI_ACTION_Y_UP:
            PrintLine::moveRelativeDistanceInSteps(0,Printer::axisStepsPerMM[1],0,0,Printer::homingFeedrate[1],false,true);
            break;
        case UI_ACTION_Y_DOWN:
            PrintLine::moveRelativeDistanceInSteps(0,-Printer::axisStepsPerMM[1],0,0,Printer::homingFeedrate[1],false,true);
            break;
        case UI_ACTION_Z_UP:
            PrintLine::moveRelativeDistanceInSteps(0,0,Printer::axisStepsPerMM[2],0,Printer::homingFeedrate[2],false,true);
            break;
        case UI_ACTION_Z_DOWN:
            PrintLine::moveRelativeDistanceInSteps(0,0,-Printer::axisStepsPerMM[2],0,Printer::homingFeedrate[2],false,true);
            break;
        case UI_ACTION_EXTRUDER_UP:
            PrintLine::moveRelativeDistanceInSteps(0,0,0,Printer::axisStepsPerMM[3],UI_SET_EXTRUDER_FEEDRATE,false,true);
            break;
        case UI_ACTION_EXTRUDER_DOWN:
            PrintLine::moveRelativeDistanceInSteps(0,0,0,-Printer::axisStepsPerMM[3],UI_SET_EXTRUDER_FEEDRATE,false,true);
            break;
        case UI_ACTION_EXTRUDER_TEMP_UP:
        {
            int tmp = (int)(Extruder::current->tempControl.targetTemperatureC)+1;
            if(tmp==1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
            else if(tmp>UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
            Extruder::setTemperatureForExtruder(tmp,Extruder::current->id);
        }
        break;
        case UI_ACTION_EXTRUDER_TEMP_DOWN:
        {
            int tmp = (int)(Extruder::current->tempControl.targetTemperatureC)-1;
            if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
            Extruder::setTemperatureForExtruder(tmp,Extruder::current->id);
        }
        break;
        case UI_ACTION_HEATED_BED_UP:
#if HAVE_HEATED_BED==true
        {
            int tmp = (int)heatedBedController.targetTemperatureC+1;
            if(tmp==1) tmp = UI_SET_MIN_HEATED_BED_TEMP;
            else if(tmp>UI_SET_MAX_HEATED_BED_TEMP) tmp = UI_SET_MAX_HEATED_BED_TEMP;
            Extruder::setHeatedBedTemperature(tmp);
        }
#endif
        break;
        case UI_ACTION_SET_MEASURED_ORIGIN:
        {
            Printer::zLength -= Printer::currentPosition[Z_AXIS];
            Printer::currentPositionSteps[Z_AXIS] = 0;
            Printer::updateDerivedParameter();
#if NONLINEAR_SYSTEM
            transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
#endif
            Printer::updateCurrentPosition();
            Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#if EEPROM_MODE!=0
            EEPROM::storeDataIntoEEPROM(false);
            Com::printFLN(Com::tEEPROMUpdated);
#endif
            Commands::printCurrentPosition();
        }
        case UI_ACTION_SET_P1:
#ifdef SOFTWARE_LEVELING
            for (uint8_t i=0; i<3; i++)
            {
                Printer::levelingP1[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_SET_P2:
#ifdef SOFTWARE_LEVELING
            for (uint8_t i=0; i<3; i++)
            {
                Printer::levelingP2[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_SET_P3:
#ifdef SOFTWARE_LEVELING
            for (uint8_t i=0; i<3; i++)
            {
                Printer::levelingP3[i] = Printer::currentPositionSteps[i];
            }
#endif
            break;
        case UI_ACTION_CALC_LEVEL:
#ifdef SOFTWARE_LEVELING
            long factors[4];
            PrintLine::calculatePlane(factors, Printer::levelingP1, Printer::levelingP2, Printer::levelingP3);
            Com::printFLN(Com::tLevelingCalc);
            Com::printFLN(Com::tTower1, PrintLine::calcZOffset(factors, Printer::deltaAPosXSteps, Printer::deltaAPosYSteps) * Printer::invAxisStepsPerMM[2]);
            Com::printFLN(Com::tTower2, PrintLine::calcZOffset(factors, Printer::deltaBPosXSteps, Printer::deltaBPosYSteps) * Printer::invAxisStepsPerMM[2]);
            Com::printFLN(Com::tTower3, PrintLine::calcZOffset(factors, Printer::deltaCPosXSteps, Printer::deltaCPosYSteps) * Printer::invAxisStepsPerMM[2]);
#endif
            break;
        case UI_ACTION_HEATED_BED_DOWN:
#if HAVE_HEATED_BED==true
        {
            int tmp = (int)heatedBedController.targetTemperatureC-1;
            if(tmp<UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
            Extruder::setHeatedBedTemperature(tmp);
        }
#endif
        break;
        case UI_ACTION_FAN_UP:
            Commands::setFanSpeed(Printer::getFanSpeed()+32,false);
            break;
        case UI_ACTION_FAN_DOWN:
            Commands::setFanSpeed(Printer::getFanSpeed()-32,false);
            break;
        case UI_ACTION_KILL:
            HAL::forbidInterrupts(); // Don't allow interrupts to do their work
            Printer::kill(false);
            Extruder::manageTemperatures();
            pwm_pos[0] = pwm_pos[1] = pwm_pos[2] = pwm_pos[3]=0;
#if EXT0_HEATER_PIN>-1
            WRITE(EXT0_HEATER_PIN,0);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
            WRITE(EXT1_HEATER_PIN,0);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
            WRITE(EXT2_HEATER_PIN,0);
#endif
#if FAN_PIN>-1
            WRITE(FAN_PIN,0);
#endif
            while(1) {}

            break;
        case UI_ACTION_RESET:
            HAL::resetHardware();
            break;
        case UI_ACTION_PAUSE:
            Com::printFLN(PSTR("RequestPause:"));
            break;
        case UI_ACTION_WRITE_DEBUG:
            Com::printF(Com::tDebug,(int)GCode::bufferReadIndex);
            Com::printF(Com::tComma,(int)GCode::bufferWriteIndex);
            Com::printF(Com::tComma,(int)GCode::commentDetected);
            Com::printF(Com::tComma,(int)GCode::bufferLength);
            Com::printF(Com::tComma,(int)GCode::waitingForResend);
            Com::printFLN(Com::tComma,(int)GCode::commandsReceivingWritePosition);
            Com::printF(Com::tDebug,Printer::minimumSpeed);
            Com::printF(Com::tComma,Printer::minimumZSpeed);
            Com::printF(Com::tComma,(int)PrintLine::linesPos);
            Com::printFLN(Com::tComma,(int)PrintLine::linesWritePos);
            break;
        }
    refreshPage();
    if(!skipBeep)
        BEEP_SHORT
#if UI_AUTORETURN_TO_MENU_AFTER!=0
        ui_autoreturn_time=HAL::timeInMilliseconds()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
}
void UIDisplay::mediumAction()
{
#if UI_HAS_I2C_ENCODER>0
    ui_check_slow_encoder();
#endif
}
void UIDisplay::slowAction()
{
    unsigned long time = HAL::timeInMilliseconds();
    uint8_t refresh=0;
#if UI_HAS_KEYS==1
    // Update key buffer
    HAL::forbidInterrupts();
    if((flags & 9)==0)
    {
        flags|=8;
        HAL::allowInterrupts();
#if FEATURE_CONTROLLER==5
        {
            // check temps and set appropriate leds
            int led= 0;
#if NUM_EXTRUDER>0
            led |= (tempController[Extruder::current->id]->targetTemperatureC > 0 ? UI_I2C_HOTEND_LED : 0);
#endif
#if HAVE_HEATED_BED
            led |= (heatedBedController.targetTemperatureC > 0 ? UI_I2C_HEATBED_LED : 0);
#endif
#if FAN_PIN>=0
            led |= (Printer::getFanSpeed() > 0 ? UI_I2C_FAN_LED : 0);
#endif
            // update the leds
            uid.outputMask= ~led&(UI_I2C_HEATBED_LED|UI_I2C_HOTEND_LED|UI_I2C_FAN_LED);
        }
#endif
        int nextAction = 0;
        ui_check_slow_keys(nextAction);
        if(lastButtonAction!=nextAction)
        {
            lastButtonStart = time;
            lastButtonAction = nextAction;
            HAL::forbidInterrupts();
            flags|=2; // Mark slow action
        }
        HAL::forbidInterrupts();
        flags-=8;
    }
    HAL::forbidInterrupts();
    if((flags & 4)==0)
    {
        flags |= 4;
        // Reset click encoder
        HAL::forbidInterrupts();
        int8_t epos = encoderPos;
        encoderPos=0;
        HAL::allowInterrupts();
        if(epos)
        {
            nextPreviousAction(epos);
            BEEP_SHORT
            refresh=1;
        }
        if(lastAction!=lastButtonAction)
        {
            if(lastButtonAction==0)
            {
                if(lastAction>=2000 && lastAction<3000)
                {
                    statusMsg[0] = 0;
                }
                lastAction = 0;
                HAL::forbidInterrupts();
                flags &= ~3;
            }
            else if(time-lastButtonStart>UI_KEY_BOUNCETIME)     // New key pressed
            {
                lastAction = lastButtonAction;
                executeAction(lastAction);
                nextRepeat = time+UI_KEY_FIRST_REPEAT;
                repeatDuration = UI_KEY_FIRST_REPEAT;
            }
        }
        else if(lastAction<1000 && lastAction)     // Repeatable key
        {
            if(time-nextRepeat<10000)
            {
                executeAction(lastAction);
                repeatDuration -=UI_KEY_REDUCE_REPEAT;
                if(repeatDuration<UI_KEY_MIN_REPEAT) repeatDuration = UI_KEY_MIN_REPEAT;
                nextRepeat = time+repeatDuration;
            }
        }
        HAL::forbidInterrupts();
        flags -=4;
    }
    HAL::allowInterrupts();
#endif
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    if(menuLevel>0 && ui_autoreturn_time<time)
    {
        lastSwitch = time;
        menuLevel=0;
        activeAction = 0;
    }
#endif
    if(menuLevel==0 && time>4000)
    {
        if(time-lastSwitch>UI_PAGES_DURATION)
        {
            lastSwitch = time;
#if !defined(UI_DISABLE_AUTO_PAGESWITCH) || !UI_DISABLE_AUTO_PAGESWITCH
            menuPos[0]++;
            if(menuPos[0]>=UI_NUM_PAGES)
                menuPos[0]=0;
#endif
            refresh = 1;
        }
        else if(time-lastRefresh>=1000) refresh=1;
    }
    else if(time-lastRefresh>=800)
    {
        UIMenu *men = (UIMenu*)menu[menuLevel];
        uint8_t mtype = pgm_read_byte((void*)&(men->menuType));
        //if(mtype!=1)
            refresh=1;
    }
    if(refresh)
    {
      if (menuLevel > 1 && iScreenTransition == 0)
        {
        shift++;
        if(shift+UI_COLS>MAX_COLS+1)
            shift = -2;
        }
     else
        shift = -2;
        
        refreshPage();
        lastRefresh = time;
    }
}
void UIDisplay::fastAction()
{
#if UI_HAS_KEYS==1
    // Check keys
    HAL::forbidInterrupts();
    if((flags & 10)==0)
    {
        flags |= 8;
        HAL::allowInterrupts();
        int nextAction = 0;
        ui_check_keys(nextAction);
        if(lastButtonAction!=nextAction)
        {
            lastButtonStart = HAL::timeInMilliseconds();
            lastButtonAction = nextAction;
            HAL::forbidInterrupts();
            flags|=1;
        }
        HAL::forbidInterrupts();
        flags-=8;
    }
    HAL::allowInterrupts();
#endif
}

#endif

