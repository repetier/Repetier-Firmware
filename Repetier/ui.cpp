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

#define UI_MAIN
#include <avr/pgmspace.h>
extern const int8_t encoder_table[16] PROGMEM ;
#include "ui.h"
#include "Reptier.h"
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include "Eeprom.h"
#include <ctype.h>

#if UI_ENCODER_SPEED==0
const int8_t encoder_table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; // Full speed
#elif UI_ENCODER_SPEED==1
const int8_t encoder_table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0}; // Half speed
#else
const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0}; // Quart speed
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

void beep(byte duration,byte count)
{
#if BEEPER_TYPE!=0
#if BEEPER_TYPE==1
  SET_OUTPUT(BEEPER_PIN);
#endif
#if BEEPER_TYPE==2
  i2c_start_wait(BEEPER_ADDRESS+I2C_WRITE);
#endif
  for(byte i=0;i<count;i++){
#if BEEPER_TYPE==1
    WRITE(BEEPER_PIN,HIGH);
#else
    i2c_write(~BEEPER_PIN);
#endif
    delay(duration);
#if BEEPER_TYPE==1
    WRITE(BEEPER_PIN,LOW);
#else
    i2c_write(255);
#endif
    delay(duration);
  }
#if BEEPER_TYPE==2
  i2c_stop();
#endif
#endif  
}

//=============================================================
//                          I2C driver
//=============================================================

#ifdef COMPILE_I2C_DRIVER
/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#include <inttypes.h>
#include <compat/twi.h>

/* I2C clock in Hz */
#define SCL_CLOCK  100000L


/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
inline void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;
    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
  /* send stop condition */
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);	
  // wait until stop condition is executed and bus released
  while(TWCR & (1<<TWSTO));
}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
  uint8_t   twst;
  // send data to the previously addressed device
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait until transmission completed
  while(!(TWCR & (1<<TWINT)));
  // check value of TWI Status Register. Mask prescaler bits
  twst = TW_STATUS & 0xF8;
  if( twst != TW_MT_DATA_ACK) return 1;
  return 0;
}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));    
    return TWDR;
}/* i2c_readAck */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));	
    return TWDR;
}/* i2c_readNak */

#endif


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
const byte character_back[8] PROGMEM = {4,14,21,4,28,0,0,0};
// Degrees sign - code 2
// ..*.. 4
// .*.*. 10
// ..*.. 4
// ..... 0
// ..... 0
// ..... 0
// ..... 0
// ..... 0
const byte character_degree[8] PROGMEM = {4,10,4,0,0,0,0,0};
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
const byte character_selected[8] PROGMEM = {0,31,31,31,31,31,0,0};
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
const byte character_unselected[8] PROGMEM = {0,31,17,17,17,31,0,0};
const long baudrates[] PROGMEM = {9600,14400,19200,28800,38400,56000,57600,76800,111112,115200,128000,230400,250000,256000,0};

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

static const byte LCDLineOffsets[] PROGMEM = UI_LINE_OFFSETS;
static const char versionString[] PROGMEM = UI_VERSION_STRING;
static const char versionString2[] PROGMEM = UI_VERSION_STRING2;


#if UI_DISPLAY_TYPE==3

// ============= I2C LCD Display driver ================
inline void lcdStartWrite() {
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
}
inline void lcdStopWrite() {
  i2c_stop();
}
void lcdWriteNibble(byte value) {
#if UI_DISPLAY_D4_PIN==1 && UI_DISPLAY_D5_PIN==2 && UI_DISPLAY_D6_PIN==4 && UI_DISPLAY_D7_PIN==8
  i2c_write((value) | UI_DISPLAY_ENABLE_PIN);
  i2c_write(value);
#else
  byte v=(value & 1?UI_DISPLAY_D4_PIN:0)|(value & 2?UI_DISPLAY_D5_PIN:0)|(value & 4?UI_DISPLAY_D6_PIN:0)|(value & 8?UI_DISPLAY_D7_PIN:0);
#endif
}
void lcdWriteByte(byte c,byte rs) {
  byte mod = (rs?UI_DISPLAY_RS_PIN:0); // | (UI_DISPLAY_RW_PIN);
#if UI_DISPLAY_D4_PIN==1 && UI_DISPLAY_D5_PIN==2 && UI_DISPLAY_D6_PIN==4 && UI_DISPLAY_D7_PIN==8
  byte value = (c >> 4) | mod;
  i2c_write((value) | UI_DISPLAY_ENABLE_PIN);
  i2c_write(value);  
  value = (c & 15) | mod;
  i2c_write((value) | UI_DISPLAY_ENABLE_PIN);
  i2c_write(value);  
#else
  byte value = (c & 16?UI_DISPLAY_D4_PIN:0)|(c & 32?UI_DISPLAY_D5_PIN:0)|(c & 64?UI_DISPLAY_D6_PIN:0)|(c & 128?UI_DISPLAY_D7_PIN:0) | mod;
  i2c_write((value) | UI_DISPLAY_ENABLE_PIN);
  i2c_write(value);  
  value = (c & 1?UI_DISPLAY_D4_PIN:0)|(c & 2?UI_DISPLAY_D5_PIN:0)|(c & 4?UI_DISPLAY_D6_PIN:0)|(c & 8?UI_DISPLAY_D7_PIN:0) | mod;
  i2c_write((value) | UI_DISPLAY_ENABLE_PIN);
  i2c_write(value);  
#endif
}
void initializeLCD() {
  delay(135);
  lcdStartWrite();
  i2c_write(0);  
  delayMicroseconds(10);
  lcdWriteNibble(0x03);
  delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
  // second try
  lcdWriteNibble(0x03);      
  delayMicroseconds(150); // wait 
  // third go!
  lcdWriteNibble(0x03); 
  delayMicroseconds(150);
  // finally, set to 4-bit interface
  lcdWriteNibble(0x02); 
  delayMicroseconds(150);
  // finally, set # lines, font size, etc.
  lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);  	
  lcdCommand(LCD_CLEAR);					//-	Clear Screen
  delay(2); // clear is slow operation
  lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);	//-	Entrymode (Display Shift: off, Increment Address Counter)
  lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);	//-	Display on
  uid.lastSwitch = uid.lastRefresh = millis();
  uid.createChar(1,character_back);
  uid.createChar(2,character_degree);
  uid.createChar(3,character_selected);
  uid.createChar(4,character_unselected);
  lcdStopWrite();
}
#endif
#if UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2

void lcdWriteNibble(byte value) {
  WRITE(UI_DISPLAY_D4_PIN,value & 1);
  WRITE(UI_DISPLAY_D5_PIN,value & 2);
  WRITE(UI_DISPLAY_D6_PIN,value & 4);
  WRITE(UI_DISPLAY_D7_PIN,value & 8);
  WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);// enable pulse must be >450ns
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}
void lcdWriteByte(byte c,byte rs) {
#if UI_DISPLAY_RW_PIN<0
  delayMicroseconds(UI_DELAYPERCHAR);
#else
  SET_INPUT(UI_DISPLAY_D4_PIN);
  SET_INPUT(UI_DISPLAY_D5_PIN);
  SET_INPUT(UI_DISPLAY_D6_PIN);
  SET_INPUT(UI_DISPLAY_D7_PIN);
  WRITE(UI_DISPLAY_RW_PIN, HIGH);
  WRITE(UI_DISPLAY_RS_PIN, LOW);
  uint8_t busy;
  do {
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    busy = READ(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
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
void initializeLCD() {

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V.
  // is this delay long enough for all cases??
  delay(135);
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
  delayMicroseconds(10);
  lcdWriteNibble(0x03);
  delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
  // second try
  lcdWriteNibble(0x03);      
  delayMicroseconds(150); // wait 
  // third go!
  lcdWriteNibble(0x03); 
  delayMicroseconds(150);
  // finally, set to 4-bit interface
  lcdWriteNibble(0x02); 
  delayMicroseconds(150);
  // finally, set # lines, font size, etc.
  lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);  
	
  lcdCommand(LCD_CLEAR);					//-	Clear Screen
  delay(2); // clear is slow operation
  lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);	//-	Entrymode (Display Shift: off, Increment Address Counter)
  lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);	//-	Display on
  uid.lastSwitch = uid.lastRefresh = millis();
  uid.createChar(1,character_back);
  uid.createChar(2,character_degree);
  uid.createChar(3,character_selected);
  uid.createChar(4,character_unselected);
}
// ----------- end direct LCD driver
#endif

#if UI_DISPLAY_TYPE==4||UI_DISPLAY_TYPE==5

#if UI_DISPLAY_TYPE==5
Adafruit_RGBLCDShield lcd;
#else
// Use LiquidCrystal library instead
#include <LiquidCrystal.h>

LiquidCrystal lcd(UI_DISPLAY_RS_PIN, UI_DISPLAY_RW_PIN,UI_DISPLAY_ENABLE_PIN,UI_DISPLAY_D4_PIN,UI_DISPLAY_D5_PIN,UI_DISPLAY_D6_PIN,UI_DISPLAY_D7_PIN);
#endif

void UIDisplay::createChar(byte location,const byte PROGMEM charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  byte data[8];
  for (int i=0; i<8; i++) {
    data[i]=pgm_read_byte(&(charmap[i]));
  }
  lcd.createChar(location, data);
}
void UIDisplay::printRow(byte r,char *txt) {    
 byte col=0;
 // Set row
 if(r >= UI_ROWS) return;
 lcd.setCursor(0,r);
 char c;
 while(col<UI_COLS && (c=*txt) != 0x00) {
   txt++;
   lcd.write(c);
   col++;
 }
 while(col<UI_COLS) {
   lcd.write(' ');
  col++; 
 }
#if UI_HAS_KEYS==1
 mediumAction();
#endif
}

void initializeLCD() {
  lcd.begin(UI_COLS,UI_ROWS);
  uid.lastSwitch = uid.lastRefresh = millis();
  uid.createChar(1,character_back);
  uid.createChar(2,character_degree);
  uid.createChar(3,character_selected);
  uid.createChar(4,character_unselected);  
}
// ------------------ End LiquidCrystal library as LCD driver
#endif

char printCols[UI_COLS+1];
UIDisplay::UIDisplay() {
#ifdef COMPILE_I2C_DRIVER
  i2c_init();
#endif
  flags = 0;
  menuLevel = 0;
  menuPos[0] = 0;
  lastAction = 0;
  lastButtonAction = 0;
  activeAction = 0;
  statusMsg[0] = 0;
  ui_init_keys();
}
void UIDisplay::initialize() {
#if UI_DISPLAY_TYPE>0
  initializeLCD();
  uid.printRowP(0,versionString);
  uid.printRowP(1,versionString2);
#endif
#if BEEPER_TYPE==2 || defined(UI_HAS_I2C_KEYS)
  // Make sure the beeper is off
  i2c_start_wait(UI_I2C_KEY_ADDRESS+I2C_WRITE);
  i2c_write(255); // Disable beeper, enable read for other pins.
  i2c_stop();
#endif
}
#if UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2 || UI_DISPLAY_TYPE==3
void UIDisplay::createChar(byte location,const byte PROGMEM charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  lcdCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcdPutChar(pgm_read_byte(&(charmap[i])));
  }
}
void UIDisplay::printRow(byte r,char *txt) {    
 byte col=0;
 // Set row
 if(r >= UI_ROWS) return;
#if UI_DISPLAY_TYPE==3
  lcdStartWrite();
#endif
  lcdWriteByte(128 + pgm_read_byte(&LCDLineOffsets[r]),0); // Position cursor
 char c;
 while(col<UI_COLS && (c=*txt) != 0x00) {
   txt++;
   lcdPutChar(c);
   col++;
 }
 while(col<UI_COLS) {
   lcdPutChar(' ');
  col++; 
 }
#if UI_DISPLAY_TYPE==3
  lcdStopWrite();
#endif
#if UI_HAS_KEYS==1
 ui_check_slow_encoder();
#endif
}
#endif

void UIDisplay::printRowP(byte r,PGM_P txt) {    
 if(r >= UI_ROWS) return;
 col=0;
 addStringP(txt);
 printCols[col]=0;
 printRow(r,printCols);
}
void UIDisplay::addInt(int value,byte digits) {
  byte dig=0,neg=0;
  if(value<0) {
    value = -value;
    neg=1;
    dig++;
  }
  char buf[7]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[6];
  buf[6]=0;
  do {
    unsigned int m = value;
    value /= 10;
    char c = m - 10 * value;
    *--str = c + '0';
    dig++;
  } while(value);
  if(neg)
      printCols[col++]='-';
  if(digits<6)
  while(dig<digits) {
    *--str = ' ';
    dig++;
  }
  while(*str && col<UI_COLS) {
    printCols[col++] = *str;
    str++;
  }
}
void UIDisplay::addLong(long value,char digits) {
  byte dig = 0,neg=0;
  if(value<0) {
    neg=1;
    value = -value;
    dig++;
  }
  char buf[13]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[12];
  buf[12]=0;
  do {
    unsigned long m = value;
    value /= 10;
    char c = m - 10 * value;
    *--str = c + '0';
    dig++;
  } while(value);
  if(neg)
    printCols[col++]='-';
  if(digits<=11)
  while(dig<digits) {
    *--str = ' ';
    dig++;
  }
  while(*str && col<UI_COLS) {
    printCols[col++] = *str;
    str++;
  }
}
const float roundingTable[] PROGMEM = {0.5,0.05,0.005,0.0005};
void UIDisplay::addFloat(float number, char fixdigits,byte digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     printCols[col++]='-';
     if(col>=UI_COLS) return;
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
  if (digits > 0) {
    printCols[col++]='.';
  }

  // Extract digits from the remainder one at a time
  while (col<UI_COLS && digits-- > 0)
  {
    remainder *= 10.0;
    byte toPrint = byte(remainder);
    printCols[col++] = '0'+toPrint;
    remainder -= toPrint; 
  } 
}
void UIDisplay::addStringP(PGM_P text) {
  while(col<UI_COLS) {
    byte c = pgm_read_byte(text++);
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

void UIDisplay::parse(char *txt,bool ram) {
  int ivalue;
  float fvalue;
  while(col<UI_COLS) {
    char c=(ram ? *(txt++) : pgm_read_byte(txt++));
    if(c==0) break; // finished
    if(c!='%') {
      printCols[col++]=c;
      continue;
    }
    // dynamic parameter, parse meaning and replace
    char c1=pgm_read_byte(txt++);
    char c2=pgm_read_byte(txt++);
    switch(c1) {
    case '%':
      if(c2=='%' && col<UI_COLS)
        printCols[col++]='%';
      break;
    case 'a': // Acceleration settings
      if(c2=='x') addFloat(max_acceleration_units_per_sq_second[0],5,0);
      else if(c2=='y') addFloat(max_acceleration_units_per_sq_second[1],5,0);
      else if(c2=='z') addFloat(max_acceleration_units_per_sq_second[2],5,0);
      else if(c2=='X') addFloat(max_travel_acceleration_units_per_sq_second[0],5,0);
      else if(c2=='Y') addFloat(max_travel_acceleration_units_per_sq_second[1],5,0);
      else if(c2=='Z') addFloat(max_travel_acceleration_units_per_sq_second[2],5,0);
      else if(c2=='j') addFloat(printer_state.maxJerk,3,1);
      else if(c2=='J') addFloat(printer_state.maxZJerk,3,1);
      break;

    case 'd':
      if(c2=='o') addStringP(DEBUG_ECHO?ui_text_on:ui_text_off);
      else if(c2=='i') addStringP(DEBUG_INFO?ui_text_on:ui_text_off);
      else if(c2=='e') addStringP(DEBUG_ERRORS?ui_text_on:ui_text_off);
      else if(c2=='d') addStringP(DEBUG_DRYRUN?ui_text_on:ui_text_off);
      break;

      case 'e': // Extruder temperature
        if(c2=='r') { // Extruder relative mode
          addStringP(relative_mode_e?ui_yes:ui_no);
          break;
        }
        if(c2=='c') ivalue=extruder_get_temperature();
        else if(c2>='0' && c2<='9') ivalue=extruder[c2-'0'].currentTemperatureC;
        else if(c2=='b') ivalue=heated_bed_get_temperature();
        addInt(ivalue>>CELSIUS_EXTRA_BITS,3);
        break;
      case 'E': // Target extruder temperature
        if(c2=='c') ivalue=current_extruder->targetTemperatureC;
        else if(c2>='0' && c2<='9') ivalue=extruder[c2-'0'].targetTemperatureC;
        else if(c2=='b') ivalue=target_bed_celsius;
        addInt(ivalue>>CELSIUS_EXTRA_BITS,3);
        break;
  
      case 'f':
        if(c2=='x') addFloat(max_feedrate[0],5,0);
        else if(c2=='y') addFloat(max_feedrate[1],5,0);
        else if(c2=='z') addFloat(max_feedrate[2],5,0);
        else if(c2=='X') addFloat(homing_feedrate[0],5,0);
        else if(c2=='Y') addFloat(homing_feedrate[1],5,0);
        else if(c2=='Z') addFloat(homing_feedrate[2],5,0);
        break;
      case 'i':
        if(c2=='s') addLong(stepper_inactive_time,4);
        else if(c2=='p') addLong(max_inactive_time,4);
        break;
      case 'O': // ops related stuff
 #if USE_OPS==1
        if(c2=='0') addStringP(printer_state.opsMode==0?ui_selected:ui_unselected);
        else if(c2=='1') addStringP(printer_state.opsMode==1?ui_selected:ui_unselected);
        else if(c2=='2') addStringP(printer_state.opsMode==2?ui_selected:ui_unselected);
        else if(c2=='r') addFloat(printer_state.opsRetractDistance,2,1);  
        else if(c2=='b') addFloat(printer_state.opsRetractBackslash,2,1);  
        else if(c2=='d') addFloat(printer_state.opsMinDistance,2,1);  
        else if(c2=='a') {
          addFloat(printer_state.opsMoveAfter,3,0);  
          if(col<UI_COLS)
            printCols[col++]='%';
        }
#endif
        break;
      case 'l':
        if(c2=='a') addInt(lastAction,4);
        break;
      case 'o': 
        if(c2=='s') {
#ifdef SDSUPPORT
          if(sdactive && sdmode) {
            addStringP(PSTR( UI_TEXT_PRINT_POS));
            unsigned long percent;
            if(filesize<20000000) percent=sdpos*100/filesize;
            else percent = (sdpos>>8)*100/(filesize>>8);
            addInt((int)percent,3);
            if(col<UI_COLS)
              printCols[col++]='%';              
          } else
#endif
          parse(statusMsg,true);
          break;
        }
        if(c2=='c') {addLong(baudrate,6);break;}
        if(c2=='e') {if(errorMsg!=0)addStringP((char PROGMEM *)errorMsg);break;}
        if(c2=='B') {addInt((int)lines_count,2);break;}
        if(c2=='m') {addInt(printer_state.feedrateMultiply,3);break;}
        // Extruder output level
        if(c2>='0' && c2<='9') ivalue=extruder[c2-'0'].output;
        else if(c2=='b') ivalue=heated_bed_output;
        else if(c2=='C') ivalue=current_extruder->output;
        ivalue=(ivalue*100)/255;
        addInt(ivalue,3);
        if(col<UI_COLS)
          printCols[col++]='%';
        break;
      case 'x':
        if(c2>='0' && c2<='3') fvalue = (float)printer_state.currentPositionSteps[c2-'0']*inv_axis_steps_per_unit[c2-'0'];
        addFloat(fvalue,3,2);
        break;
      case 'X': // Extruder related 
        if(c2>='0' && c2<='9') {addStringP(current_extruder->id==c2-'0'?ui_selected:ui_unselected);}
#ifdef TEMP_PID
        else if(c2=='i') {addLong(current_extruder->pidIGain,4);}
        else if(c2=='p') {addLong(current_extruder->pidPGain,4);}
        else if(c2=='d') {addLong(current_extruder->pidDGain,4);}
        else if(c2=='m') {addInt(current_extruder->pidDriveMin,3);}
        else if(c2=='M') {addInt(current_extruder->pidDriveMax,3);}
        else if(c2=='D') {addInt(current_extruder->pidMax,3);}
#endif
        else if(c2=='w') {addInt(current_extruder->watchPeriod,4);}
        else if(c2=='h') {addStringP(!current_extruder->heatManager?PSTR(UI_TEXT_STRING_HM_BANGBANG):PSTR(UI_TEXT_STRING_HM_PID));}
        else if(c2=='a') {addFloat(current_extruder->advanceK,3,1);}
        else if(c2=='x') {addFloat(current_extruder->xOffset,4,2);}
        else if(c2=='y') {addFloat(current_extruder->yOffset,4,2);}
        else if(c2=='f') {addFloat(current_extruder->maxStartFeedrate,5,0);}
        else if(c2=='F') {addFloat(current_extruder->maxFeedrate,5,0);}
        else if(c2=='A') {addFloat(current_extruder->maxAcceleration,5,0);}
        break;
      case 's': // Endstop positions
        if(c2=='x') {
        #if (X_MIN_PIN > -1)
          addStringP((READ(X_MIN_PIN)^ENDSTOP_X_MIN_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
        }
        if(c2=='X')
      	#if (X_MAX_PIN > -1)
          addStringP((READ(X_MAX_PIN)^ENDSTOP_X_MAX_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
        if(c2=='y')
      	#if (Y_MIN_PIN > -1)
          addStringP((READ(Y_MIN_PIN)^ENDSTOP_Y_MIN_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
        if(c2=='Y')
      	#if (Y_MAX_PIN > -1)
          addStringP((READ(Y_MAX_PIN)^ENDSTOP_Y_MAX_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
        if(c2=='z')
      	#if (Z_MIN_PIN > -1)
          addStringP((READ(Z_MIN_PIN)^ENDSTOP_Z_MIN_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
        if(c2=='Z')
      	#if (Z_MAX_PIN > -1)
          addStringP((READ(Z_MAX_PIN)^ENDSTOP_Z_MAX_INVERTING)?ui_text_on:ui_text_off);
        #else
          addStringP(ui_text_na);
      	#endif
      break;
    case 'S':
      if(c2=='x') addFloat(axis_steps_per_unit[0],3,1);
      if(c2=='y') addFloat(axis_steps_per_unit[1],3,1);
      if(c2=='z') addFloat(axis_steps_per_unit[2],3,1);
      if(c2=='e') addFloat(current_extruder->stepsPerMM,3,1);
      break;  
    }
  }
  printCols[col] = 0;
}
void UIDisplay::setStatusP(PGM_P txt) {
  byte i=0;
  while(i<16) {
    byte c = pgm_read_byte(txt++);
    if(!c) break;
    statusMsg[i++] = c;
  }
  statusMsg[i]=0;
}
void UIDisplay::setStatus(char *txt) {
  byte i=0;
  while(*txt && i<16)
    statusMsg[i++] = *txt++;
  statusMsg[i]=0;
}

const UIMenu * const ui_pages[UI_NUM_PAGES] PROGMEM = UI_PAGES;
#ifdef SDSUPPORT
byte nFilesOnCard;

void UIDisplay::updateSDFileCount() {
  dir_t* p;
  byte offset = menuTop[menuLevel];
  root.rewind();
  nFilesOnCard = 0;
  while ((p = root.readDirCache())) {
    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;

    // skip deleted entry and entries for . and  ..
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;

    // only list subdirectories and files
    if (!DIR_IS_FILE(p)) continue;
    nFilesOnCard++;
    if(nFilesOnCard==254) return;
  }
}
void getSDFilenameAt(byte filePos,char *filename) {
  dir_t* p;
  byte c=0;
  root.rewind();
  while ((p = root.readDirCache())) {
    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;
    // skip deleted entry and entries for . and  ..
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    // only list subdirectories and files
    if (!DIR_IS_FILE(p)) continue;
    if(filePos) {
      filePos--;
      continue;
    }
    for (uint8_t i = 0; i < 11; i++) {
      if (p->name[i] == ' ')continue;
      if (i == 8)
        filename[c++]='.';
      filename[c++]=tolower(p->name[i]);
    }
    break;
  }
  filename[c]=0;
}

void UIDisplay::sdrefresh(byte &r) {
  dir_t* p;
  byte offset = menuTop[menuLevel];
  root.rewind();
  byte skip = (offset>0?offset-1:0);
  while (r+offset<nFilesOnCard+1 && r<UI_ROWS && (p = root.readDirCache())) {
    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;
    // skip deleted entry and entries for . and  ..
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    // only list subdirectories and files
    if (!DIR_IS_FILE(p)) continue;
    if(skip>0) {skip--;continue;}
    col=0;
    if(r+offset==menuPos[menuLevel])
       printCols[col++]='>';
    else
       printCols[col++]=' ';
    // print file name with possible blank fill
    for (byte i = 0; i < 11; i++) {
      if (p->name[i] == ' ')continue;
      if (i == 8)
        printCols[col++]='.';
      printCols[col++]=tolower(p->name[i]);
    }
    printCols[col]=0;
    printRow(r,printCols);
    r++;
  }
}
#endif
// Refresh current menu page
void UIDisplay::refreshPage() {
  byte r;
  byte mtype;
  if(menuLevel==0) {
    UIMenu *men = (UIMenu*)pgm_read_word(&(ui_pages[menuPos[0]]));
    byte nr = pgm_read_word_near(&(men->numEntries));
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    for(r=0;r<nr && r<UI_ROWS;r++) {
      UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[r]));
      col=0;
      parse((char*)pgm_read_word(&(ent->text)),false);
      printRow(r,(char*)printCols);
    }
  } else {
    UIMenu *men = (UIMenu*)menu[menuLevel];
    byte nr = pgm_read_word_near((void*)&(men->numEntries));
    mtype = pgm_read_byte((void*)&(men->menuType));
    byte offset = menuTop[menuLevel];
    UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    for(r=0;r+offset<nr && r<UI_ROWS;r++) {
      UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[r+offset]));
      unsigned char entType = pgm_read_byte(&(ent->menuType));
      unsigned int entAction = pgm_read_word(&(ent->action));
      col=0;
      if(entType>=2 && entType<=4) {
        if(r+offset==menuPos[menuLevel] && activeAction!=entAction)
          printCols[col++]='>';
        else if(activeAction==entAction)
          printCols[col++]='*';
        else
          printCols[col++]=' ';
      }
      parse((char*)pgm_read_word(&(ent->text)),false);
      if(entType==2) { // Draw submenu marker at the right side
        while(col<UI_COLS) printCols[col++]=' ';
        printCols[UI_COLS-1]=0x7e; // Arrow right
      }
      printRow(r,(char*)printCols);
    }
  }
#ifdef SDSUPPORT
    if(mtype==1) {
      sdrefresh(r);
    }
#endif
  printCols[0]=0;
  while(r<UI_ROWS)
    printRow(r++,printCols);
}
void UIDisplay::pushMenu(void *men,bool refresh) {
  if(men==menu[menuLevel]) {
    refreshPage();
    return;
  }
  if(menuLevel==4) return;
  menuLevel++;
  menu[menuLevel]=men;
  menuTop[menuLevel] = menuPos[menuLevel] = 0;
#ifdef SDSUPPORT
  UIMenu *men2 = (UIMenu*)menu[menuLevel];
  if(pgm_read_byte(&(men2->menuType))==1) // Open files list
    updateSDFileCount();
#endif
  if(refresh)
    refreshPage();
}
void UIDisplay::okAction() {
#if UI_HAS_KEYS==1
  if(menuLevel==0) { // Enter menu
    menuLevel = 1;
    menuTop[1] = menuPos[1] = 0;
    menu[1] = (void*)&ui_menu_main;
    BEEP_SHORT
    return;
  }
  UIMenu *men = (UIMenu*)menu[menuLevel];
  //byte nr = pgm_read_word_near(&(menu->numEntries));
  byte mtype = pgm_read_byte(&(men->menuType));
  UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
  UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
  unsigned char entType = pgm_read_byte(&(ent->menuType));// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action
  int action = pgm_read_word(&(ent->action));
  if(mtype==3) { // action menu
      action = pgm_read_word(&(men->id));
      finishAction(action);
      executeAction(UI_ACTION_BACK);
      return;
  }
  if(mtype==2 && entType==4) { // Modify action
    if(activeAction) { // finish action
      finishAction(action);
      activeAction = 0;
    } else 
      activeAction = action;
    return;
  }
#ifdef SDSUPPORT
  if(mtype==1) {
    if(menuPos[menuLevel]==0) { // Selected back instead of file
      executeAction(UI_ACTION_BACK);
      return;
    }
    byte filePos = menuPos[menuLevel]-1;
    menuLevel--;
    men = (UIMenu*)menu[menuLevel];
    entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
    char filename[13];
    getSDFilenameAt(filePos,filename);
    switch(pgm_read_word(&(ent->action))) {
      case UI_ACTION_SD_PRINT:
        if(sdactive){
            sdmode = false;
            file.close();
            if (file.open(&root, filename, O_READ)) {
                out.print_P(PSTR("File opened:"));
                out.print(filename);
                out.print_P(PSTR(" Size:"));
                out.println(file.fileSize());
                sdpos = 0;
                filesize = file.fileSize();
                out.println_P(PSTR("File selected"));
                sdmode = true; // Start print immediately
                menuLevel = 0;
                BEEP_LONG;
            }
            else{
                out.println_P(PSTR("file.open failed"));
            }
        }
        break;
      case UI_ACTION_SD_DELETE:
       if(sdactive){
            sdmode = false;
            file.close();
            if(SdFile::remove(&root,filename)) {
              out.println_P(PSTR("File deleted"));
              BEEP_LONG
            } else {
              out.println_P(PSTR("Deletion failed"));
            }
        }
        break;
    }
  }
#endif
  if(entType==2) { // Enter submenu
    pushMenu((void*)action,false);
    BEEP_SHORT
    return;
  }
  if(entType==3) {
    executeAction(action);
    return;
  }  
  executeAction(UI_ACTION_BACK);
#endif
}
#define INCREMENT_MIN_MAX(a,steps,_min,_max) a+=increment*steps;if(a<(_min)) a=_min;else if(a>(_max)) a=_max;
void UIDisplay::nextPreviousAction(char next) {
#if UI_HAS_KEYS==1
  if(menuLevel==0) { 
    lastSwitch = millis();
    if(next>0) {
      menuPos[0]++;
      if(menuPos[0]>=UI_NUM_PAGES)
        menuPos[0]=0;
    } else {
      if(menuPos[0]==0)
        menuPos[0]=UI_NUM_PAGES-1;
      else
        menuPos[0]--;
    }
    return;
  }
  UIMenu *men = (UIMenu*)menu[menuLevel];
  byte nr = pgm_read_word_near(&(men->numEntries));
  byte mtype = pgm_read_byte(&(men->menuType));
  UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
  UIMenuEntry *ent =(UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
  unsigned char entType = pgm_read_byte(&(ent->menuType));// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command
  int action = pgm_read_word(&(ent->action));
  if(mtype==2 && activeAction==0) { // browse through menu items
    if(next>0) {
      if(menuPos[menuLevel]+1<nr) menuPos[menuLevel]++;
    } else if(menuPos[menuLevel]>0)
      menuPos[menuLevel]--;
    if(menuTop[menuLevel]>menuPos[menuLevel])
      menuTop[menuLevel]=menuPos[menuLevel];
    else if(menuTop[menuLevel]+UI_ROWS-1<menuPos[menuLevel])
      menuTop[menuLevel]=menuPos[menuLevel]+1-UI_ROWS;
    return;
  }
#ifdef SDSUPPORT
    if(mtype==1) { // SD listing
      if(next>0) {
        if(menuPos[menuLevel]<nFilesOnCard) menuPos[menuLevel]++;
      } else if(menuPos[menuLevel]>0)
        menuPos[menuLevel]--;
      if(menuTop[menuLevel]>menuPos[menuLevel])
        menuTop[menuLevel]=menuPos[menuLevel];
      else if(menuTop[menuLevel]+UI_ROWS-1<menuPos[menuLevel])
        menuTop[menuLevel]=menuPos[menuLevel]+1-UI_ROWS;
      return;
    }
#endif
  if(mtype==3) action = pgm_read_word(&(men->id)); else action=activeAction;
  char increment = next;
  switch(action) {
  case UI_ACTION_XPOSITION:
    move_steps(increment,0,0,0,homing_feedrate[0],true);
    printPosition();
    break;
  case UI_ACTION_YPOSITION:
    move_steps(0,increment,0,0,homing_feedrate[1],true);
    printPosition();
    break;
  case UI_ACTION_ZPOSITION:
    move_steps(0,0,increment,0,homing_feedrate[2],true);
    printPosition();
    break;
  case UI_ACTION_XPOSITION_FAST:
    move_steps(axis_steps_per_unit[0]*increment,0,0,0,homing_feedrate[0],true);
    printPosition();
    break;
  case UI_ACTION_YPOSITION_FAST:
    move_steps(0,axis_steps_per_unit[1]*increment,0,0,homing_feedrate[1],true);
    printPosition();
    break;
  case UI_ACTION_ZPOSITION_FAST:
    move_steps(0,0,axis_steps_per_unit[2]*increment,0,homing_feedrate[2],true);
    printPosition();
    break;
  case UI_ACTION_EPOSITION:
    move_steps(0,0,0,axis_steps_per_unit[3]*increment,UI_SET_EXTRUDER_FEEDRATE,true);
    printPosition();
    break;
  case UI_ACTION_HEATED_BED_TEMP:
#if HAVE_HEATED_BED==true
    {
       int tmp = target_bed_celsius>>CELSIUS_EXTRA_BITS;
       if(tmp<UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
       tmp+=increment;
       if(tmp==1) tmp = UI_SET_MIN_HEATED_BED_TEMP;
       if(tmp<UI_SET_MIN_HEATED_BED_TEMP) tmp = 0;
       else if(tmp>UI_SET_MAX_HEATED_BED_TEMP) tmp = UI_SET_MAX_HEATED_BED_TEMP;
       heated_bed_set_temperature(tmp<<CELSIUS_EXTRA_BITS);
    }
#endif
    break;
  case UI_ACTION_EXTRUDER0_TEMP:
    {
       int tmp = extruder[0].targetTemperatureC>>CELSIUS_EXTRA_BITS;
       if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
       tmp+=increment;
       if(tmp==1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
       if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
       else if(tmp>UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
       extruder_set_temperature(tmp<<CELSIUS_EXTRA_BITS,0);
    }
    break;
  case UI_ACTION_EXTRUDER1_TEMP:
 #if NUM_EXTRUDER>1
    {
       int tmp = extruder[1].targetTemperatureC>>CELSIUS_EXTRA_BITS;
       tmp+=increment;
       if(tmp==1) tmp = UI_SET_MIN_EXTRUDER_TEMP;
       if(tmp<UI_SET_MIN_EXTRUDER_TEMP) tmp = 0;
       else if(tmp>UI_SET_MAX_EXTRUDER_TEMP) tmp = UI_SET_MAX_EXTRUDER_TEMP;
       extruder_set_temperature(tmp<<CELSIUS_EXTRA_BITS,1);
    }
 #endif
    break;
#if USE_OPS==1
  case UI_ACTION_OPS_RETRACTDISTANCE:
    printer_state.opsRetractDistance+=increment*0.1;
    if(printer_state.opsRetractDistance<0) printer_state.opsRetractDistance=0;
    else if(printer_state.opsRetractDistance>10) printer_state.opsRetractDistance=10;
    extruder_select(current_extruder->id);
    break;
  case UI_ACTION_OPS_BACKSLASH:
    printer_state.opsRetractBackslash+=increment*0.1;
    if(printer_state.opsRetractBackslash<-5) printer_state.opsRetractBackslash=-5;
    else if(printer_state.opsRetractBackslash>5) printer_state.opsRetractBackslash=5;
    extruder_select(current_extruder->id);
    break;
  case UI_ACTION_OPS_MOVE_AFTER:
    printer_state.opsMoveAfter+=increment;
    if(printer_state.opsMoveAfter<0) printer_state.opsMoveAfter=0;
    else if(printer_state.opsMoveAfter>10) printer_state.opsMoveAfter=100;
    extruder_select(current_extruder->id);
    break;
  case UI_ACTION_OPS_MINDISTANCE:
    printer_state.opsMinDistance+=increment;
    if(printer_state.opsMinDistance<0) printer_state.opsMinDistance=0;
    else if(printer_state.opsMinDistance>10) printer_state.opsMinDistance=10;
    extruder_select(current_extruder->id);
    break;
#endif
  case UI_ACTION_FEEDRATE_MULTIPLY:
    {
      int fr = printer_state.feedrateMultiply;
      INCREMENT_MIN_MAX(fr,1,25,500);
      change_feedrate_multiply(fr);
    }
    break;
  case UI_ACTION_STEPPER_INACTIVE:
    INCREMENT_MIN_MAX(stepper_inactive_time,60,0,9999);
    break;
  case UI_ACTION_MAX_INACTIVE:
    INCREMENT_MIN_MAX(max_inactive_time,60,0,9999);
    break;
  case UI_ACTION_PRINT_ACCEL_X:
    INCREMENT_MIN_MAX(max_acceleration_units_per_sq_second[0],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_PRINT_ACCEL_Y:
    INCREMENT_MIN_MAX(max_acceleration_units_per_sq_second[1],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_PRINT_ACCEL_Z:
    INCREMENT_MIN_MAX(max_acceleration_units_per_sq_second[2],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_MOVE_ACCEL_X:
    INCREMENT_MIN_MAX(max_travel_acceleration_units_per_sq_second[0],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_MOVE_ACCEL_Y:
    INCREMENT_MIN_MAX(max_travel_acceleration_units_per_sq_second[1],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_MOVE_ACCEL_Z:
    INCREMENT_MIN_MAX(max_travel_acceleration_units_per_sq_second[2],100,0,10000);
    update_ramps_parameter();
    break;
  case UI_ACTION_MAX_JERK:
    INCREMENT_MIN_MAX(printer_state.maxJerk,0.1,1,99.9);
    break;
  case UI_ACTION_MAX_ZJERK:
    INCREMENT_MIN_MAX(printer_state.maxZJerk,0.1,0.1,99.9);
    break;
  case UI_ACTION_HOMING_FEEDRATE_X:
    INCREMENT_MIN_MAX(homing_feedrate[0],60,60,30000);
    break;
  case UI_ACTION_HOMING_FEEDRATE_Y:
    INCREMENT_MIN_MAX(homing_feedrate[1],60,60,30000);
    break;
  case UI_ACTION_HOMING_FEEDRATE_Z:
    INCREMENT_MIN_MAX(homing_feedrate[2],60,60,30000);
    break;
  case UI_ACTION_MAX_FEEDRATE_X:
    INCREMENT_MIN_MAX(max_feedrate[0],60,60,30000);
    break;
  case UI_ACTION_MAX_FEEDRATE_Y:
    INCREMENT_MIN_MAX(max_feedrate[1],60,60,30000);
    break;
  case UI_ACTION_MAX_FEEDRATE_Z:
    INCREMENT_MIN_MAX(max_feedrate[2],60,60,30000);
    break;
  case UI_ACTION_STEPS_X:
    INCREMENT_MIN_MAX(axis_steps_per_unit[0],0.1,0,999);
    update_ramps_parameter();
    break;
  case UI_ACTION_STEPS_Y:
    INCREMENT_MIN_MAX(axis_steps_per_unit[1],0.1,0,999);
    update_ramps_parameter();
    break;
  case UI_ACTION_STEPS_Z:
    INCREMENT_MIN_MAX(axis_steps_per_unit[2],0.1,0,999);
    update_ramps_parameter();
    break;
  case UI_ACTION_BAUDRATE:
#if EEPROM_MODE!=0
    {
      char p=0;
      long rate;
      do {
        rate = pgm_read_dword(&(baudrates[p]));
        if(rate==baudrate) break;
        p++;
      } while(rate!=0);
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
      INCREMENT_MIN_MAX(current_extruder->pidPGain,1,0,9999);
      break;
  case UI_ACTION_PID_IGAIN:
      INCREMENT_MIN_MAX(current_extruder->pidIGain,1,0,9999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_PID_DGAIN:
      INCREMENT_MIN_MAX(current_extruder->pidDGain,1,0,9999);
      break;
  case UI_ACTION_DRIVE_MIN:
      INCREMENT_MIN_MAX(current_extruder->pidDriveMin,1,1,255);
      break;
  case UI_ACTION_DRIVE_MAX:
      INCREMENT_MIN_MAX(current_extruder->pidDriveMax,1,1,255);
      break;
  case UI_ACTION_PID_MAX:
      INCREMENT_MIN_MAX(current_extruder->pidMax,1,1,255);
      break;
#endif
  case UI_ACTION_X_OFFSET:
      INCREMENT_MIN_MAX(current_extruder->xOffset,1,-99999,99999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_Y_OFFSET:
      INCREMENT_MIN_MAX(current_extruder->yOffset,1,-99999,99999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_EXTR_STEPS:
      INCREMENT_MIN_MAX(current_extruder->stepsPerMM,1,1,9999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_EXTR_ACCELERATION:
      INCREMENT_MIN_MAX(current_extruder->maxAcceleration,10,10,99999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_EXTR_MAX_FEEDRATE:
      INCREMENT_MIN_MAX(current_extruder->maxFeedrate,60,60,90000);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_EXTR_START_FEEDRATE:
      INCREMENT_MIN_MAX(current_extruder->maxStartFeedrate,1,1,999);
      extruder_select(current_extruder->id);
      break;
  case UI_ACTION_EXTR_HEATMANAGER:
      INCREMENT_MIN_MAX(current_extruder->heatManager,1,0,1);
      break;
  case UI_ACTION_EXTR_WATCH_PERIOD:
      INCREMENT_MIN_MAX(current_extruder->watchPeriod,1,0,999);
      break;
  case UI_ACTION_ADVANCE_K:
      INCREMENT_MIN_MAX(current_extruder->advanceK,0.5,0,200);
      break;
  }
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    ui_autoreturn_time=millis()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
}

void UIDisplay::finishAction(int action) {
}
// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.
void UIDisplay::executeAction(int action) {
#if UI_HAS_KEYS==1
  bool skipBeep = false;
  if(action>=2000 && action<3000)
  {
      setStatusP(ui_action);
  }
  else
  switch(action) {
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
      emergencyStop();
      break;
    case UI_ACTION_HOME_ALL:
      home_axis(true,true,true);
      printPosition();
      break;
    case UI_ACTION_HOME_X:
      home_axis(true,false,false);
      printPosition();
      break;
    case UI_ACTION_HOME_Y:
      home_axis(false,true,false);
      printPosition();
      break;
    case UI_ACTION_HOME_Z:
      home_axis(false,false,true);
      printPosition();
      break;
    case UI_ACTION_SET_ORIGIN:
      printer_state.currentPositionSteps[0] = -printer_state.offsetX;
      printer_state.currentPositionSteps[1] = -printer_state.offsetY;
      printer_state.currentPositionSteps[2] = 0;
      break;
    case UI_ACTION_DEBUG_ECHO:
      if(DEBUG_ECHO) debug_level-=1;else debug_level+=1;
      break;
    case UI_ACTION_DEBUG_INFO:
      if(DEBUG_INFO) debug_level-=2;else debug_level+=2;
      break;
    case UI_ACTION_DEBUG_ERROR:
      if(DEBUG_ERRORS) debug_level-=4;else debug_level+=4;
      break;
    case UI_ACTION_DEBUG_DRYRUN:
      if(DEBUG_DRYRUN) debug_level-=8;else debug_level+=8;
      if(DEBUG_DRYRUN) { // simulate movements without printing
          extruder_set_temperature(0,0);
#if NUM_EXTRUDER>1
          extruder_set_temperature(0,1);
#endif
#if HAVE_HEATED_BED==true
          target_bed_raw = 0;
#endif
      }
      break;
    case UI_ACTION_POWER:
      break;
    case UI_ACTION_PREHEAT:
      UI_STATUS(UI_TEXT_PREHEAT);
      extruder_set_temperature(UI_SET_PRESET_EXTRUDER0_TEMP<<CELSIUS_EXTRA_BITS,0);
#if NUM_EXTRUDER>1
      extruder_set_temperature(UI_SET_PRESET_EXTRUDER1_TEMP<<CELSIUS_EXTRA_BITS,1);
#endif
#if HAVE_HEATED_BED==true
      heated_bed_set_temperature(UI_SET_PRESET_HEATED_BED_TEMP<<CELSIUS_EXTRA_BITS);
#endif 
      break;
    case UI_ACTION_COOLDOWN:
      UI_STATUS(UI_TEXT_COOLDOWN);
      extruder_set_temperature(0,0);
#if NUM_EXTRUDER>1
      extruder_set_temperature(0,1);
#endif
#if HAVE_HEATED_BED==true
      heated_bed_set_temperature(0);
#endif 
      break;
    case UI_ACTION_HEATED_BED_OFF:
#if HAVE_HEATED_BED==true
      heated_bed_set_temperature(0);
#endif 
      break;
    case UI_ACTION_EXTRUDER0_OFF:
      extruder_set_temperature(0,0);
      break;
    case UI_ACTION_EXTRUDER1_OFF:
 #if NUM_EXTRUDER>1
      extruder_set_temperature(0,1);
 #endif
      break;
#if USE_OPS==1
    case UI_ACTION_OPS_OFF:
      printer_state.opsMode=0;
      break;
    case UI_ACTION_OPS_CLASSIC:
      printer_state.opsMode=1;
      break;
    case UI_ACTION_OPS_FAST:
      printer_state.opsMode=2;
      break;
 #endif
    case UI_ACTION_DISABLE_STEPPER:
      kill(true);
      break;
    case UI_ACTION_RESET_EXTRUDER:
      printer_state.currentPositionSteps[3] = 0;
      break;
    case UI_ACTION_EXTRUDER_RELATIVE:
      relative_mode_e=!relative_mode_e;
      break;
    case UI_ACTION_SELECT_EXTRUDER0:
      extruder_select(0);
      break;
    case UI_ACTION_SELECT_EXTRUDER1:
#if NUM_EXTRUDER>1
      extruder_select(1);
#endif
      break;
#if EEPROM_MODE!=0
    case UI_ACTION_STORE_EEPROM:
      epr_data_to_eeprom();
      pushMenu((void*)&ui_menu_eeprom_saved,false);
      BEEP_LONG;skipBeep = true;
      break;
    case UI_ACTION_LOAD_EEPROM:
      epr_eeprom_to_data();
      pushMenu((void*)&ui_menu_eeprom_loaded,false);
      BEEP_LONG;skipBeep = true;
      break;
#endif
#ifdef SDSUPPORT
    case UI_ACTION_SD_DELETE:
      if(sdactive){
        pushMenu((void*)&ui_menu_sd_fileselector,false);
      } else {
        UI_ERROR(UI_TEXT_NOSDCARD);
      }
      break;
    case UI_ACTION_SD_PRINT:
      if(sdactive){
        pushMenu((void*)&ui_menu_sd_fileselector,false);
      }
      break;
    case UI_ACTION_SD_PAUSE:
      if(sdmode){
          sdmode = false;
      }
      break;
    case UI_ACTION_SD_CONTINUE:
      if(sdactive){
          sdmode = true;
      }
      break;
    case UI_ACTION_SD_UNMOUNT:
      sdmode = false;
      sdactive = false;
      break;
    case UI_ACTION_SD_MOUNT:
      sdmode = false;
      initsd();
      break;
    case UI_ACTION_MENU_SDCARD:
      pushMenu((void*)&ui_menu_sd,false);
      break;
#endif
#if FAN_PIN>-1
    case UI_ACTION_FAN_OFF:
      set_fan_speed(0,false);
      out.println_P(PSTR("Fanspeed:0"));
      break;
    case UI_ACTION_FAN_25:
      set_fan_speed(64,false);
      out.println_P(PSTR("Fanspeed:64"));
      break;
    case UI_ACTION_FAN_50:
      set_fan_speed(128,false);
      out.println_P(PSTR("Fanspeed:128"));
      break;
    case UI_ACTION_FAN_75:
      set_fan_speed(192,false);
      out.println_P(PSTR("Fanspeed:192"));
      break;
    case UI_ACTION_FAN_FULL:
      set_fan_speed(255,false);
      out.println_P(PSTR("Fanspeed:255"));
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
  }
  refreshPage();
  if(!skipBeep)
    BEEP_SHORT
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    ui_autoreturn_time=millis()+UI_AUTORETURN_TO_MENU_AFTER;
#endif
#endif
}
void UIDisplay::mediumAction() {
  ui_check_slow_encoder();
}
void UIDisplay::slowAction() {
  unsigned long time = millis();
  byte refresh=0;
#if UI_HAS_KEYS==1
  // Update key buffer
  cli();
  if((flags & 9)==0) {
    flags|=8;
    sei();
    int nextAction = 0;
    ui_check_slow_keys(nextAction);
    if(lastButtonAction!=nextAction) {
      lastButtonStart = time;
      lastButtonAction = nextAction;
      cli();
      flags|=2; // Mark slow action
    }
    cli();
    flags-=8;
  }
  cli();
  if((flags & 4)==0) {
    flags |= 4; 
    // Reset click encoder
    cli();
    char epos = encoderPos;
    encoderPos=0;
    sei();
    if(epos) {
      nextPreviousAction(epos);
      BEEP_SHORT
      refresh=1;
    }
    if(lastAction!=lastButtonAction) {
      if(lastButtonAction==0) {
        if(lastAction>=2000 && lastAction<3000)
        {
          statusMsg[0] = 0;
        }
        lastAction = 0;
        cli();
        flags &= ~3;
      } else if(time-lastButtonStart>UI_KEY_BOUNCETIME) { // New key pressed
        lastAction = lastButtonAction;
        executeAction(lastAction);
        nextRepeat = time+UI_KEY_FIRST_REPEAT;
        repeatDuration = UI_KEY_FIRST_REPEAT;
      }
    } else if(lastAction<1000 && lastAction) { // Repeatable key
      if(time-nextRepeat<10000) {
        executeAction(lastAction);
        repeatDuration -=UI_KEY_REDUCE_REPEAT;
        if(repeatDuration<UI_KEY_MIN_REPEAT) repeatDuration = UI_KEY_MIN_REPEAT;
        nextRepeat = time+repeatDuration;      
      }
    }
    cli();
    flags -=4;
  }
  sei();
#endif
#if UI_AUTORETURN_TO_MENU_AFTER!=0
    if(menuLevel>0 && ui_autoreturn_time<time) {
      lastSwitch = time;
      menuLevel=0;
      activeAction = 0;
    }
#endif
  if(menuLevel==0 && time>4000) {
    if(time-lastSwitch>UI_PAGES_DURATION) {
      lastSwitch = time;
#ifndef UI_DISABLE_AUTO_PAGESWITCH
      menuPos[0]++;
      if(menuPos[0]>=UI_NUM_PAGES)
        menuPos[0]=0;
#endif
      refresh = 1;
    } else if(time-lastRefresh>=1000) refresh=1;
  } else if(time-lastRefresh>=1000) {
    UIMenu *men = (UIMenu*)menu[menuLevel];
    byte mtype = pgm_read_byte((void*)&(men->menuType));
    if(mtype!=1)
      refresh=1;
  }
  if(refresh) {
    refreshPage();
    lastRefresh = time;
  }
}
void UIDisplay::fastAction() {
#if UI_HAS_KEYS==1
  // Check keys
  cli();
  if((flags & 10)==0) {
    flags |= 8;
    sei();
    int nextAction = 0;
    ui_check_keys(nextAction);
    if(lastButtonAction!=nextAction) {
      lastButtonStart = millis();
      lastButtonAction = nextAction;
      cli();
      flags|=1;
    }
    cli();
    flags-=8;
  }
  sei();
#endif
}

#endif

