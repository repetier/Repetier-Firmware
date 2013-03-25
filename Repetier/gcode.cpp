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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Reptier.h"

#ifndef FEATURE_CHECKSUM_FORCED
#define FEATURE_CHECKSUM_FORCED true
#endif

#define bit_clear(x,y) x&= ~(1<<y) //cbi(x,y) 
#define bit_set(x,y)   x|= (1<<y)//sbi(x,y) 

#define MAX_CMD_SIZE 96
GCode gcode_buffer[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
byte gcode_rindex=0; ///< Read position in gcode_buffer.
byte gcode_windex=0; ///< Write position in gcode_buffer.
byte gcode_transbuffer[MAX_CMD_SIZE]; ///< Current received command. 
byte gcode_wpos=0; ///< Writing position in gcode_transbuffer.
byte gcode_binary; ///< Flags the command as binary input.
byte gcode_last_binary=0; ///< Was the last successful command in binary mode?
byte gcode_comment=false; ///< Flags true if we are reading the comment part of a command.
byte gcode_binary_size; ///< Expected size of the incoming binary command.
bool gcode_wait_all_parsed=false; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
long gcode_lastN=0; ///< Last line number received.
long gcode_actN; ///< Line number of current command.
char gcode_wait_resend=-1; ///< Waiting for line to be resend. -1 = no wait.
volatile byte gcode_buflen=0; ///< Number of commands stored in gcode_buffer
unsigned long gcode_lastdata=0; ///< Time, when we got the last data packet. Used to detect missing bytes.
SerialOutput out; ///< Instance used for serail write operations.

#ifndef EXTERNALSERIAL
// Implement serial communication for one stream only!
/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
  
  Modified to use only 1 queue with fixed length by Repetier
*/

ring_buffer rx_buffer = { { 0 }, 0, 0};
ring_buffer tx_buffer = { { 0 }, 0, 0};

inline void rf_store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) & SERIAL_BUFFER_MASK;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}
#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
  void rfSerialEvent() __attribute__((weak));
  void rfSerialEvent() {}
  #define serialEvent_implemented
#if defined(USART_RX_vect)
  SIGNAL(USART_RX_vect)
#elif defined(USART0_RX_vect)
  SIGNAL(USART0_RX_vect)
#else
#if defined(SIG_USART0_RECV)
  SIGNAL(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
  SIGNAL(SIG_UART0_RECV)
#elif defined(SIG_UART_RECV)
  SIGNAL(SIG_UART_RECV)
#else
  #error "Don't know what the Data Received vector is called for the first UART"
#endif
#endif
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;
  #elif defined(UDR)
    unsigned char c  =  UDR;
  #else
    #error UDR not defined
  #endif
    rf_store_char(c, &rx_buffer);
  }
#endif

#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
  #error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
  if (tx_buffer.head == tx_buffer.tail) {
	// Buffer empty, so disable interrupts
#if defined(UCSR0B)
    bit_clear(UCSR0B, UDRIE0);
#else
    bit_clear(UCSRB, UDRIE);
#endif
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    tx_buffer.tail = (tx_buffer.tail + 1) & SERIAL_BUFFER_MASK;
	
  #if defined(UDR0)
    UDR0 = c;
  #elif defined(UDR)
    UDR = c;
  #else
    #error UDR not defined
  #endif
  }
}
#endif
#endif


// Constructors ////////////////////////////////////////////////////////////////

RFHardwareSerial::RFHardwareSerial(ring_buffer *rx_buffer, ring_buffer *tx_buffer,
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *udr,
  uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x)
{
  _rx_buffer = rx_buffer;
  _tx_buffer = tx_buffer;
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _udr = udr;
  _rxen = rxen;
  _txen = txen;
  _rxcie = rxcie;
  _udrie = udrie;
  _u2x = u2x;
}

// Public Methods //////////////////////////////////////////////////////////////

void RFHardwareSerial::begin(unsigned long baud)
{
  uint16_t baud_setting;
  bool use_u2x = true;

#if F_CPU == 16000000UL
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    use_u2x = false;
  }
#endif

try_again:
  
  if (use_u2x) {
    *_ucsra = 1 << _u2x;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }
  
  if ((baud_setting > 4095) && use_u2x)
  {
    use_u2x = false;
    goto try_again;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  bit_set(*_ucsrb, _rxen);
  bit_set(*_ucsrb, _txen);
  bit_set(*_ucsrb, _rxcie);
  bit_clear(*_ucsrb, _udrie);
}

void RFHardwareSerial::end()
{
  // wait for transmission of outgoing data
  while (_tx_buffer->head != _tx_buffer->tail)
    ;

  bit_clear(*_ucsrb, _rxen);
  bit_clear(*_ucsrb, _txen);
  bit_clear(*_ucsrb, _rxcie);  
  bit_clear(*_ucsrb, _udrie);
  
  // clear a  ny received data
  _rx_buffer->head = _rx_buffer->tail;
}

int RFHardwareSerial::available(void)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & SERIAL_BUFFER_MASK;
}

int RFHardwareSerial::peek(void)
{
  if (_rx_buffer->head == _rx_buffer->tail) {
    return -1;
  } else {
    return _rx_buffer->buffer[_rx_buffer->tail];
  }
}

int RFHardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer->head == _rx_buffer->tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) & SERIAL_BUFFER_MASK;
    return c;
  }
}

void RFHardwareSerial::flush()
{
  while (_tx_buffer->head != _tx_buffer->tail)
    ;
}
#ifdef COMPAT_PRE1
  void 
#else
  size_t 
#endif
RFHardwareSerial::write(uint8_t c)
{
  int i = (_tx_buffer->head + 1) & SERIAL_BUFFER_MASK;
	
  // If the output buffer is full, there's nothing for it other than to 
  // wait for the interrupt handler to empty it a bit
  // ???: return 0 here instead?
  while (i == _tx_buffer->tail)
    ;
	
  _tx_buffer->buffer[_tx_buffer->head] = c;
  _tx_buffer->head = i;
	
  bit_set(*_ucsrb, _udrie);
#ifndef COMPAT_PRE1 
  return 1;
#endif
}

// Preinstantiate Objects //////////////////////////////////////////////////////

#if defined(UBRRH) && defined(UBRRL)
  RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
  RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
#elif defined(USBCON)
  // do nothing - Serial object and buffers are initialized in CDC code
#else
  #error no serial port defined  (port 0)
#endif

#endif

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcommings in the standard
RepRap communication method, while remaining backward compatible. To use the improved
features of this protocal, you need a host which speaks it. On Windows the recommended
host software is Repetier-Host. It is developed in parallel to this firmware and supports
all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce communication failures.
  The standard method is to transfer a checksum at the end of the line. This checksum is the
  XORd value of all characters send. The value is limited to a range between 0 and 127. It can 
  not detect two identical missing characters or a wrong order. Therefore the new protocol
  uses Fletchers checksum, which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to less then 50% and
  it speeds up decoding the command. No slow conversion from string to floats are needed.
  
*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 bytes define the
data. From this bitfield, this function computes the size of the command
including the 2 bytes of the bitfield and the 2 bytes for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned byte
- G : Bit 2 :  8-Bit unsigned byte
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more bytes following with Bits, only for future versions
- Int :Bit 14 : Marks it as internal command, 
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
*/
byte gcode_comp_binary_size(char *ptr) {// unsigned int bitfield) {
   byte s = 4; // include checksum and bitfield
   unsigned int bitfield = *(int*)ptr;
   if(bitfield & 1) s+=2;
   if(bitfield & 8) s+=4;
   if(bitfield & 16) s+=4;
   if(bitfield & 32) s+=4;
   if(bitfield & 64) s+=4;
   if(bitfield & 256) s+=4;
   if(bitfield & 512) s+=1;
   if(bitfield & 1024) s+=4;
   if(bitfield & 2048) s+=4;
   if(bitfield & 4096) { // Version 2 or later
     s+=2; // for bitfield 2
     unsigned int bitfield2 = *(int*)(ptr+2);
     if(bitfield & 2) s+=2;
     if(bitfield & 4) s+=2;
     if(bitfield2 & 1) s+= 4;
     if(bitfield2 & 2) s+= 4;
     if(bitfield2 & 4) s+= 4;
     if(bitfield & 32768) s+=(byte)ptr[4]+1;
     //OUT_P_I_LN("LenStr:",(int)ptr[4]);
     //OUT_P_I_LN("LenBinV2:",s);
   } else {
     if(bitfield & 2) s+=1;
     if(bitfield & 4) s+=1;
     if(bitfield & 32768) s+=16;
   }
   return s;
}

extern "C" void __cxa_pure_virtual() { }

SerialOutput::SerialOutput() {
}
#ifdef COMPAT_PRE1
void 
#else
size_t
#endif
SerialOutput::write(uint8_t value) {
  RFSERIAL.write(value);
#ifndef COMPAT_PRE1
  return 1;
#endif
}
void SerialOutput::printFloat(double number, uint8_t digits) 
{ 
  if (isnan(number)) {
	print_P(PSTR("NAN"));
    return;
  }

  if (isinf(number)) {
	print_P(PSTR("INF"));
    return;
  }
  
  // Handle negative numbers
  if (number < 0.0)
  {
     write('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    write('.'); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}

/**
  Print a string stored in program memory on serial console.
  
  Example: serial_print_pgm(PSTR("Dummy string"));
*/
void SerialOutput::print_P(PGM_P ptr) {
  char c;
  while ((c=pgm_read_byte(ptr++)) != 0x00) 
     write(c);
}

/**
  Print a string stored in program memory on serial console.
  
  Example: out.println_P(PSTR("Dummy string"));
*/
void SerialOutput::println_P(PGM_P ptr) {
  print_P(ptr);
  println();
}
void SerialOutput::print_long_P(PGM_P ptr,long value) {
  print_P(ptr);
  print(value);
}
void SerialOutput::print_int_P(PGM_P ptr,int value) {
  print_P(ptr);
  print(value);
}

void SerialOutput::print_float_P(PGM_P ptr,float value,uint8_t digits) {
  print_P(ptr);
  out.printFloat(value,digits);
}
void SerialOutput::println_long_P(PGM_P ptr,long value) {
  print_P(ptr);
  println(value);
}
void SerialOutput::println_int_P(PGM_P ptr,int value) {
  print_P(ptr);
  println(value);
}

void SerialOutput::println_float_P(PGM_P ptr,float value,uint8_t digits) {
  print_P(ptr);
  printFloat(value,digits);
  println();
}
void SerialOutput::print_error_P(PGM_P ptr,bool newline) {
  OUT_P("error:");
  print_P(ptr);
  if(newline)
    println();
}
/** \brief request resend of the expected line.
*/
void gcode_resend() {
  RFSERIAL.flush();
  gcode_wpos=0;
  if(gcode_binary)
    gcode_wait_resend = 30;
  else
    gcode_wait_resend = 14;
  OUT_LN;
  OUT_P_L_LN("Resend:",gcode_lastN+1);
  OUT_P_LN("ok");
}
void emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD==1
  resetFunc();
#else
     cli(); // Don't allow interrupts to do their work
     kill(false);
     manage_temperatures();
     pwm_pos[0] = pwm_pos[1] = pwm_pos[2] = pwm_pos[3]=0;
#if EXT0_HEATER_PIN>-1
    WRITE(EXT0_HEATER_PIN,0);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1
    WRITE(EXT1_HEATER_PIN,0);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1
    WRITE(EXT2_HEATER_PIN,0);
#endif
#if FAN_PIN>-1
    WRITE(FAN_PIN,0);
#endif
     while(1) {}
#endif
}
/**
  Check if result is plausible. If it is, an ok is send and the command is stored in queue.
  If not, a resend and ok is send.
*/
void gcode_checkinsert(GCode *act) {
  if(GCODE_HAS_M(act)) {
   if(act->M==110) { // Reset line number
     gcode_lastN = gcode_actN;
     OUT_P_LN("ok");
     return;
   }
   if(act->M==112) { // Emergency kill - freeze printer
     emergencyStop();
   }
  }
  if(GCODE_HAS_N(act)) {
    if((((gcode_lastN+1) & 0xffff)!=(gcode_actN&0xffff))) {
      if(gcode_wait_resend<0) { // after a resend, we have to skip the garbage in buffers, no message for this
        if(DEBUG_ERRORS) {
           OUT_P_L("Error: expected line ",gcode_lastN+1); 
           OUT_P_L_LN(" got ",gcode_actN);
        }
        gcode_resend(); // Line missing, force resend
      } else {
        --gcode_wait_resend;
        gcode_wpos = 0;
        OUT_P_L_LN("skip ",gcode_actN);
        OUT_P_LN("ok");
      }
      return;
    }
    gcode_lastN = gcode_actN;
  }
  gcode_windex = (gcode_windex+1) % GCODE_BUFFER_SIZE;
  gcode_buflen++;
#ifdef ACK_WITH_LINENUMBER  
  OUT_P_L_LN("ok ",gcode_actN);
#else
  OUT_P_LN("ok");
#endif
  gcode_last_binary = gcode_binary;
  gcode_wait_resend = -1; // everything is ok.
#ifndef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      OUT_P("Echo:");
      gcode_print_command(act);
      out.println();
  }
#endif
}
void gcode_silent_insert() {
  gcode_windex = (gcode_windex+1) % GCODE_BUFFER_SIZE;
  gcode_buflen++;
#ifndef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      out.print_P(PSTR("Echo:"));
      gcode_print_command(act);
      out.println();
  }
#endif
}
/**
  Get the next buffered command. Returns 0 if no more commands are buffered. For each
  returned command, the gcode_command_finished() function must be called.
*/
GCode *gcode_next_command() {
  if(gcode_buflen==0) return 0; // No more data
  byte idx = gcode_rindex;
  gcode_rindex = (idx+1) % GCODE_BUFFER_SIZE;
  return &gcode_buffer[idx];
}
/** \brief Removes the last returned command from cache.

*/
void gcode_command_finished(GCode *code) {
  if(!gcode_buflen) return; // Should not happen, but safety first
#ifdef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      OUT_P("Echo:");
      gcode_print_command(code);
      out.println();
  }
#endif
  gcode_buflen--;
}

/** \brief Execute commands in progmem stored string. Multiple commands are seperated by \n */
void gcode_execute_PString(PGM_P cmd) {
  char buf[80];
  byte buflen;
  char c;
  GCode code;
  do {
    // Wait for a free place in command buffer
    // Scan next command from string
    byte comment=0; 
    buflen = 0;   
    do {
      c = pgm_read_byte(cmd++);
      if(c == 0 || c == '\n') break;
      if(c == ';') comment = 1;
      if(comment) continue;
      buf[buflen++] = c;
    } while(buflen<79);
    if(buflen==0) { // empty line ignore
      continue;
    }
    buf[buflen]=0;
    // Send command into command buffer
    if(gcode_parse_ascii(&code,(char *)buf) && (code.params & 518)) { // Success
      process_command(&code,false);
      defaultLoopActions();
    }
  } while(c);
}
/** \brief Read from serial console or sdcard.

This function is the main function to read the commands from serial console or from sdcard.
It must be called frequently to empty the incoming buffer.
*/
void gcode_read_serial() {
  if(gcode_wait_all_parsed && gcode_buflen) return;
  gcode_wait_all_parsed=false;
  GCode *act;
  unsigned long time = millis();
  if(gcode_buflen>=GCODE_BUFFER_SIZE) return; // all buffers full
  if(RFSERIAL.available()==0) {
    if((gcode_wait_resend>=0 || gcode_wpos>0) && time-gcode_lastdata>200) {
      gcode_resend(); // Something is wrong, a started line was not continued in the last second 
      gcode_lastdata = time;
    }
#ifdef WAITING_IDENTIFIER
    else if(gcode_buflen == 0 && time-gcode_lastdata>1000) { // Don't do it if buffer is not empty. It may be a slow executing command.
      OUT_P_LN(WAITING_IDENTIFIER); // Unblock communication in case the last ok was not received correct.
      gcode_lastdata = time;
    }   
#endif
  }
  while(RFSERIAL.available() > 0 && gcode_wpos < MAX_CMD_SIZE) {  // consume data until no data or buffer full
    gcode_lastdata = millis();
    gcode_transbuffer[gcode_wpos++] = RFSERIAL.read();
    // first lets detect, if we got an old type ascii command
    if(gcode_wpos==1) {
      if(gcode_wait_resend>=0 && gcode_last_binary) {
         if(!gcode_transbuffer[0]) {gcode_wait_resend--;} // Skip 30 zeros to get in sync
         else gcode_wait_resend = 30;
         gcode_wpos = 0;
         continue;
      }
      if(!gcode_transbuffer[0]) {gcode_wpos = 0;continue;}
      gcode_binary = (gcode_transbuffer[0] & 128)!=0;
    }
    if(gcode_binary) {
      if(gcode_wpos < 2 ) continue;
      if(gcode_wpos == 5) gcode_binary_size = gcode_comp_binary_size((char*)gcode_transbuffer);
      if(gcode_wpos==gcode_binary_size) {
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_binary(act,gcode_transbuffer)) { // Success
          gcode_checkinsert(act);
        } else {
          gcode_resend();
        }
        gcode_wpos = 0;
        return;
      }
    } else { // Ascii command
      char ch = gcode_transbuffer[gcode_wpos-1];
      if(ch == '\n' || ch == '\r' || ch == ':' || gcode_wpos >= (MAX_CMD_SIZE - 1) ) {// complete line read
        gcode_transbuffer[gcode_wpos-1]=0;
        gcode_comment = false;
        if(gcode_wpos==1) { // empty line ignore
          gcode_wpos = 0;
          continue;
        }
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_ascii(act,(char *)gcode_transbuffer)) { // Success
          gcode_checkinsert(act);
        } else {
          gcode_resend();
        }
        gcode_wpos = 0;
        return;
      } else {
        if(ch == ';') gcode_comment = true; // ignore new data until lineend
        if(gcode_comment) gcode_wpos--;
      }           
    }
  }
  #if SDSUPPORT
  if(!sd.sdmode || gcode_wpos!=0) { // not reading or incoming serial command
    return;
  }
  while( sd.filesize > sd.sdpos && gcode_wpos < MAX_CMD_SIZE) {  // consume data until no data or buffer full
    gcode_lastdata = millis();
    int n = sd.file.read();
    if(n==-1) {
      OUT_P_LN("SD read error");
      break;
    }
    sd.sdpos++; // = file.curPosition();
    gcode_transbuffer[gcode_wpos++] = (byte)n;
  
    // first lets detect, if we got an old type ascii command
    if(gcode_wpos==1) {
      gcode_binary = (gcode_transbuffer[0] & 128)!=0;
    }
    if(gcode_binary) {
      if(gcode_wpos < 2 ) continue;
      if(gcode_wpos == 5) gcode_binary_size = gcode_comp_binary_size((char*)gcode_transbuffer);
      if(gcode_wpos==gcode_binary_size) {
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_binary(act,gcode_transbuffer)) { // Success
          gcode_silent_insert();
        } 
        gcode_wpos = 0;
        return;
      }
    } else {
      char ch = gcode_transbuffer[gcode_wpos-1];
      if(ch == '\n' || ch == '\r' || ch == ':' || gcode_wpos >= (MAX_CMD_SIZE - 1) ) {// complete line read
        gcode_transbuffer[gcode_wpos-1]=0;
        gcode_comment = false;
        if(gcode_wpos==1) { // empty line ignore
          gcode_wpos = 0;
          continue;
        }
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_ascii(act,(char *)gcode_transbuffer)) { // Success
          gcode_silent_insert();
        }
        gcode_wpos = 0;
        return;
      } else {
        if(ch == ';') gcode_comment = true; // ignore new data until lineend
        if(gcode_comment) gcode_wpos--;
      }
    }
  }
  sd.sdmode = false;
  OUT_P_LN("Done printing file");
  /*OUT_P_L("Printed bytes:",sd.sdpos);
  OUT_P_L_LN(" of ",sd.filesize);
  OUT_P_I_LN("WPOS:",gcode_wpos);*/
  gcode_wpos = 0;
#endif
}

/**
  Converts a binary bytefield containing one GCode line into a GCode structure.
  Returns true if checksum was correct.
*/
bool gcode_parse_binary(GCode *code,byte *buffer) {
   unsigned int sum1=0,sum2=0; // for fletcher-16 checksum
   // first do fletcher-16 checksum tests see
   // http://en.wikipedia.org/wiki/Fletcher's_checksum
   byte i=0;
   byte *p = buffer;
   byte len = gcode_binary_size-2;
   while (len) {
     byte tlen = len > 21 ? 21 : len;
     len -= tlen;
     do {
        sum1 += *p++;
        if(sum1>=255) sum1-=255;
        sum2 += sum1;
        if(sum2>=255) sum2-=255;
     } while (--tlen);
   }
   sum1 -= *p++;
   sum2 -= *p;
   if(sum1 | sum2) {
      if(DEBUG_ERRORS) {
        OUT_P_LN("Error:Binary cmd wrong checksum.");
      }
     return false;
   }
   p = buffer;
   code->params = *(unsigned int *)p;p+=2;
   byte textlen=16;
   if(GCODE_IS_V2(code)) {
     code->params2 = *(unsigned int *)p;p+=2;
     if(GCODE_HAS_STRING(code)) 
       textlen = *p++;
   } else code->params2 = 0;
   if(code->params & 1) {gcode_actN=code->N=*(unsigned int *)p;p+=2;}
   if(GCODE_IS_V2(code)) { // Read G,M as 16 bit value
     if(code->params & 2) {code->M=*(unsigned int *)p;p+=2;}
     if(code->params & 4) {code->G=*(unsigned int *)p;p+=2;}
   } else {
     if(code->params & 2) {code->M=*p++;}
     if(code->params & 4) {code->G=*p++;}
   }
   //if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
   if(code->params & 8) {code->X=*(float *)p;p+=4;}
   if(code->params & 16) {code->Y=*(float *)p;p+=4;}
   if(code->params & 32) {code->Z =*(float *)p;p+=4;}
   if(code->params & 64) {code->E=*(float *)p;p+=4;}
   if(code->params & 256) {code->F=*(float *)p;p+=4;}
   if(code->params & 512) {code->T=*p++;}
   if(code->params & 1024) {code->S=*(long int*)p;p+=4;}
   if(code->params & 2048) {code->P=*(long int*)p;p+=4;}
   if(GCODE_HAS_I(code)) {code->I=*(float *)p;p+=4;}
   if(GCODE_HAS_J(code)) {code->J=*(float *)p;p+=4;}
   if(GCODE_HAS_R(code)) {code->R=*(float *)p;p+=4;}
   if(GCODE_HAS_STRING(code)) { // set text pointer to string
     code->text = (char*)p;
     code->text[textlen] = 0; // Terminate string overwriting checksum
     gcode_wait_all_parsed=true; // Don't destroy string until executed
   }
   return true; 
}
inline float gcode_value(char *s) { return (strtod(s, NULL)); }
inline long gcode_value_long(char *s) { return (strtol(s, NULL, 10)); }

/**
  Converts a ascii GCode line into a GCode structure.
*/
bool gcode_parse_ascii(GCode *code,char *line) {
  bool has_checksum = false;
  char *pos;
  code->params = 0;
  code->params2 = 0;
  if((pos = strchr(line,'N'))!=0) { // Line number detected
     gcode_actN = gcode_value_long(++pos);
     code->params |=1;
     code->N = gcode_actN & 0xffff;
  }
  if((pos = strchr(line,'M'))!=0) { // M command
     code->M = gcode_value_long(++pos) & 0xffff;
     code->params |= 2;
     if(code->M>255) code->params |= 4096;
  }
  if(GCODE_HAS_M(code) && (code->M == 23 || code->M == 28 || code->M == 29 || code->M == 30 || code->M == 32 || code->M == 117)) {
     // after M command we got a filename for sd card management
     char *sp = line;
     while(*sp!='M') sp++; // Search M command
     while(*sp!=' ') sp++; // search next whitespace
     while(*sp==' ') sp++; // skip leading whitespaces
     code->text = sp;
     while(*sp) {
        if(code->M != 117 && (*sp==' ' || *sp=='*')) break; // end of filename reached
        sp++;
     }
     *sp = 0; // Removes checksum, but we don't care. Could also be part of the string.
     gcode_wait_all_parsed = true; // don't risk string be deleted
     code->params |= 32768;
  } else {
    if((pos = strchr(line,'G'))!=0) { // G command
       code->G = gcode_value_long(++pos) & 0xffff;
       code->params |= 4;
       if(code->G>255) code->params |= 4096;
    }
    if((pos = strchr(line,'X'))!=0) { 
       code->X = gcode_value(++pos);
       code->params |= 8;
    }
    if((pos = strchr(line,'Y'))!=0) { 
       code->Y = gcode_value(++pos);
       code->params |= 16;
    }
    if((pos = strchr(line,'Z'))!=0) { 
       code->Z = gcode_value(++pos);
       code->params |= 32;
    }
    if((pos = strchr(line,'E'))!=0) { 
       code->E = gcode_value(++pos);
       code->params |= 64;
    }
    if((pos = strchr(line,'F'))!=0) { 
       code->F = gcode_value(++pos);
       code->params |= 256;
    }
    if((pos = strchr(line,'T'))!=0) { // M command
       code->T = gcode_value_long(++pos) & 0xff;
       code->params |= 512;
    }
    if((pos = strchr(line,'S'))!=0) { // M command
       code->S = gcode_value_long(++pos);
       code->params |= 1024;
    }
    if((pos = strchr(line,'P'))!=0) { // M command
       code->P = gcode_value_long(++pos);
       code->params |= 2048;
    }
    if((pos = strchr(line,'I'))!=0) { 
       code->I = gcode_value(++pos);
       code->params2 |= 1;
       code->params |= 4096; // Needs V2 for saving
    }
    if((pos = strchr(line,'J'))!=0) { 
       code->J = gcode_value(++pos);
       code->params2 |= 2;
       code->params |= 4096; // Needs V2 for saving
    }
    if((pos = strchr(line,'R'))!=0) { 
       code->R = gcode_value(++pos);
       code->params2 |= 4;
       code->params |= 4096; // Needs V2 for saving
    }
  }
  if((pos = strchr(line,'*'))!=0) { // checksum
    byte checksum_given = gcode_value_long(pos+1);
    byte checksum = 0;
    while(line!=pos) checksum ^= *line++;
#if FEATURE_CHECKSUM_FORCED
    printer_state.flag0 |= PRINTER_FLAG0_FORCE_CHECKSUM;
#endif
    if(checksum!=checksum_given) {
      if(DEBUG_ERRORS) {
        out.println_int_P(PSTR("Error: Wrong checksum "),(int)checksum);
      }
      return false; // mismatch
    }
  } 
#if FEATURE_CHECKSUM_FORCED
  else {
    if(GCODE_HAS_M(code) && code->M == 117) return true;
      if(DEBUG_ERRORS) {
        OUT_P("Error: Missing checksum ");
      }
    return false;
  }
#endif
  return true;
}

/** \brief Print command on serial console */
void gcode_print_command(GCode *code) {
  if(GCODE_HAS_M(code)) {
    out.print('M');
    out.print((int)code->M);
    out.print(' ');
  }
  if(GCODE_HAS_G(code)) {
    out.print('G');
    out.print((int)code->G);
    out.print(' ');
  }
  if(GCODE_HAS_T(code)) {
    out.print('T');
    out.print((int)code->T);
    out.print(' ');
  }
  if(GCODE_HAS_X(code)) {
    out.print_float_P(PSTR(" X"),code->X);
  }
  if(GCODE_HAS_Y(code)) {
    out.print_float_P(PSTR(" Y"),code->Y);
  }
  if(GCODE_HAS_Z(code)) {
    out.print_float_P(PSTR(" Z"),code->Z);
  }
  if(GCODE_HAS_E(code)) {
    out.print_float_P(PSTR(" E"),code->E,4);
  }
  if(GCODE_HAS_F(code)) {
    out.print_float_P(PSTR(" F"),code->F);
  }
  if(GCODE_HAS_S(code)) {
    out.print_long_P(PSTR(" S"),code->S);
  }
  if(GCODE_HAS_P(code)) {
    out.print_long_P(PSTR(" P"),code->P);
  }
  if(GCODE_HAS_I(code)) {
    out.print_float_P(PSTR(" I"),code->I);
  }
  if(GCODE_HAS_J(code)) {
    out.print_float_P(PSTR(" J"),code->J);
  }
  if(GCODE_HAS_R(code)) {
    out.print_float_P(PSTR(" R"),code->R);
  }
  if(GCODE_HAS_STRING(code)) {
    out.print(code->text);
  }  
}
