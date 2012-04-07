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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Configuration.h"
#include "Reptier.h"

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
long gcode_lastN=0; ///< Last line number received.
long gcode_actN; ///< Line number of current command.
char gcode_wait_resend=-1; ///< Waiting for line to be resend. -1 = no wait.
volatile byte gcode_buflen=0; ///< Number of commands stored in gcode_buffer
unsigned long gcode_lastdata=0; ///< Time, when we got the last data packet. Used to detect missing bytes.
#ifndef COMPAT_PRE1
#undef USE_BUFFERED_OUTPUT
#endif
#ifdef USE_BUFFERED_OUTPUT 
byte out_buffer[OUTPUT_BUFFER_SIZE+4]; ///< Buffer for serial write operations.
#endif
SerialOutput out; ///< Instance used for serail write operations.

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
*/
byte gcode_comp_binary_size(unsigned int bitfield) {
   byte s = 4; // include checksum and bitfield
   if(bitfield & 1) s+=2;
   if(bitfield & 2) s+=1;
   if(bitfield & 4) s+=1;
   if(bitfield & 8) s+=4;
   if(bitfield & 16) s+=4;
   if(bitfield & 32) s+=4;
   if(bitfield & 64) s+=4;
   if(bitfield & 256) s+=4;
   if(bitfield & 512) s+=1;
   if(bitfield & 1024) s+=4;
   if(bitfield & 2048) s+=4;
   if(bitfield & 32768) s+=16;
   return s;
}

extern "C" void __cxa_pure_virtual() { }

#ifdef USE_BUFFERED_OUTPUT 
/* Initialisiert ein uint8 array zur Verwendung als fifo Buffer.
   buffer muss dazu bufferSize+4 Byte groß sein. */
void osFifoInit(uint8 *buffer,uint8 bufferSize) {
  buffer[0] = bufferSize;
  buffer[1] = buffer[2] = buffer[3] = 0;
}
/** \brief put byte in fofo buffer.

The function returns 0 if it was not successfull, because the buffer was full.
*/
uint8 osFifoPutByte(uint8 *buffer,uint8 value) {
  BEGIN_INTERRUPT_PROTECTED
  volatile uint8 *count = &buffer[1];
  if (*count >= buffer[0]) {
    ESCAPE_INTERRUPT_PROTECTED
	return 0;
  }
  buffer[4+buffer[3]++] = value;
  if(buffer[3]==buffer[0]) buffer[3] = 0;
  (*count)++;
  END_INTERRUPT_PROTECTED
  return 1;
}

int osFifoGetByte(uint8 *buffer) {
  uint8 val;
  BEGIN_INTERRUPT_PROTECTED
  volatile uint8 *count = &buffer[1];
  if(*count==0) {
    ESCAPE_INTERRUPT_PROTECTED
    return -1; // Buffer empty
  }
  val = buffer[4+buffer[2]++];
  (*count)--;
  if(buffer[2]==buffer[0]) buffer[2] = 0;
  END_INTERRUPT_PROTECTED
  return val;
}

uint8 osFifoGetWaitByte(uint8 *buffer) {
  int val;
  do {
    val = osFifoGetByte(buffer);
  } while(val==-1);
  return (uint8)val;
}
uint8 osFifoEmpty(uint8 *buffer) {
  volatile uint8 *count = &buffer[1];
  return (*count!=0?0:1);
}
uint8 osFifoFull(uint8 *buffer) {
  volatile uint8 *count = &buffer[1];
  return (*count==buffer[0]?1:0);
}
SerialOutput::SerialOutput() {
  osFifoInit(out_buffer,OUTPUT_BUFFER_SIZE);
}
void SerialOutput::write(uint8_t value) {
  while(!osFifoPutByte(out_buffer,value)) {} // Loop until buffer has free space
  UCSR0B |= (1 << UDRIE0);
}
/**
  Write next byte in ouput buffer or disable output interrupt.
*/
ISR (USART0_UDRE_vect) {
    if (osFifoEmpty(out_buffer))
        UCSR0B &= ~(1 << UDRIE0);
    else
       UDR0 = osFifoGetByte(out_buffer);
}
#else
SerialOutput::SerialOutput() {
}
#ifdef COMPAT_PRE1
void 
#else
size_t
#endif
SerialOutput::write(uint8_t value) {
  Serial.write(value);
#ifndef COMPAT_PRE1
  return 1;
#endif
}
#endif
void SerialOutput::printFloat(double number, uint8_t digits) 
{ 
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

/** \brief request resend of the expected line.
*/
void gcode_resend() {
  //Serial.flush();
  gcode_wpos=0;
  if(gcode_binary)
    gcode_wait_resend = 30;
  else
    gcode_wait_resend = 14;
  out.println_long_P(PSTR("Resend:"),gcode_lastN+1);
  out.println_P(PSTR("ok"));
}
void emergencyStop() {
     cli(); // Don't allow interrupts to do their work
     kill(false);
     manage_temperatures();
#ifdef SIMULATE_PWM
#if NUM_EXTRUDER==1
     WRITE(EXT0_HEATER_PIN,0 ); 
#else
  for(byte e=0;e<NUM_EXTRUDER;e++) {
    Extruder *ext = &extruder[e];
    digitalWrite(ext->heaterPin,off);
  }
#endif
#endif // SIMULATE_PWM
#ifdef SIMULATE_FAN_PWM
      WRITE(FAN_PIN,0 ); 
#endif
     while(1) {}
}
/**
  Check if result is plausible. If it is, an ok is send and the command is stored in queue.
  If not, a resend and ok is send.
*/
void gcode_checkinsert(GCode *act) {
  if(GCODE_HAS_M(act)) {
   if(act->M==110) { // Reset line number
     gcode_lastN = gcode_actN;
     out.println_P(PSTR("ok"));
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
           out.print_long_P(PSTR("Error: expected line "),gcode_lastN+1); 
           out.println_long_P(PSTR(" got "),gcode_actN);
        }
        gcode_resend(); // Line missing, force resend
      } else {
        --gcode_wait_resend;
        gcode_wpos = 0;
        out.println_long_P(PSTR("skip "),gcode_actN);
        out.println_P(PSTR("ok"));
      }
      return;
    }
    gcode_lastN = gcode_actN;
  }
  gcode_windex = (gcode_windex+1) % GCODE_BUFFER_SIZE;
  gcode_buflen++;
#ifdef ACK_WITH_LINENUMBER  
  out.println_long_P(PSTR("ok "),gcode_actN);
#else
  out.println_P(PSTR("ok"));
#endif
  gcode_last_binary = gcode_binary;
  gcode_wait_resend = -1; // everything is ok.
#ifndef ECHO_ON_EXECUTE
  if(DEBUG_ECHO) {
      out.print_P(PSTR("Echo:"));
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
void gcode_command_finished() {
  gcode_buflen--;
}
/** \brief Read from serial console or sdcard.

This function is the main function to read the commands from serial console or from sdcard.
It must be called frequently to empty the incoming buffer.
*/
void gcode_read_serial() {
  GCode *act;
  unsigned long time = millis();
  if(gcode_buflen>=GCODE_BUFFER_SIZE) return; // all buffers full
  if(Serial.available()==0) {
    if((gcode_wait_resend>=0 || gcode_wpos>0) && time-gcode_lastdata>200) {
      gcode_resend(); // Something is wrong, a started line was not continued in the last second 
       //  out.println_P(PSTR("Timeout"));   
      gcode_lastdata = time;
    }
#ifdef WAITING_IDENTIFIER
    else if(gcode_buflen == 0 && time-gcode_lastdata>1000) { // Don't do it if buffer is not empty. It may be a slow executing command.
      out.println_P(PSTR(WAITING_IDENTIFIER)); // Unblock communication in case the last ok was not received correct.
      gcode_lastdata = time;
    }   
#endif
  }
  while( Serial.available() > 0 && gcode_wpos < MAX_CMD_SIZE) {  // consume data until no data or buffer full
    gcode_lastdata = millis();
    gcode_transbuffer[gcode_wpos++] = Serial.read();
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
      if(gcode_wpos == 2) gcode_binary_size = gcode_comp_binary_size(*(unsigned int*)gcode_transbuffer);
      else if(gcode_wpos==gcode_binary_size) {
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_binary(act,gcode_transbuffer)) { // Success
          gcode_checkinsert(act);
        } else {
          gcode_resend();
        }
        gcode_wpos = 0;
        return;
      }
    } else {
      char ch = gcode_transbuffer[gcode_wpos-1];
      if(ch == '\n' || ch == '\r' || ch == ':' || gcode_wpos >= (MAX_CMD_SIZE - 1) ) {// complete line read
        gcode_transbuffer[gcode_wpos]=0;
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
        gcode_comment = false;
        return;
      } else {
        if(ch == ';') gcode_comment = true; // ignore new data until lineend
        if(gcode_comment) gcode_wpos--;
      }           
    }
  }
  #ifdef SDSUPPORT
  if(!sdmode || gcode_wpos!=0) { // not reading or incoming serial command
    return;
  }
  while( filesize > sdpos && gcode_wpos < MAX_CMD_SIZE) {  // consume data until no data or buffer full
    gcode_lastdata = millis();
    int n = file.read();
    if(n==-1) break;

    sdpos++; // = file.curPosition();
    gcode_transbuffer[gcode_wpos++] = (byte)n;
  
    // first lets detect, if we got an old type ascii command
    if(gcode_wpos==1) {
      gcode_binary = (gcode_transbuffer[0] & 128)!=0;
    }
    if(gcode_binary) {
      if(gcode_wpos < 2 ) continue;
      if(gcode_wpos == 2) gcode_binary_size = gcode_comp_binary_size(*(unsigned int*)gcode_transbuffer);
      else if(gcode_wpos==gcode_binary_size) {
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
        gcode_transbuffer[gcode_wpos]=0;
        if(gcode_wpos==1) { // empty line ignore
          gcode_wpos = 0;
          continue;
        }
        act = &gcode_buffer[gcode_windex];
        if(gcode_parse_ascii(act,(char *)gcode_transbuffer)) { // Success
          gcode_silent_insert();
        }
        gcode_wpos = 0;
        gcode_comment = false;
        return;
      } else {
        if(ch == ';') gcode_comment = true; // ignore new data until lineend
        if(gcode_comment) gcode_wpos--;
      }           
    }
  }
     sdmode = false;
     out.println_P(PSTR("Done printing file"));
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
        out.println_P(PSTR("Error:Binary cmd wrong checksum."));
      }
     return false;
   }
   p = buffer;
   code->params = *(unsigned int *)p;p+=2;
   if(code->params & 1) {gcode_actN=code->N=*(unsigned int *)p;p+=2;}
   if(code->params & 2) {code->M=*p++;}
   if(code->params & 4) {code->G=*p++;}
   //if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
   if(code->params & 8) {code->X=*(float *)p;p+=4;}
   if(code->params & 16) {code->Y=*(float *)p;p+=4;}
   if(code->params & 32) {code->Z =*(float *)p;p+=4;}
   if(code->params & 64) {code->E=*(float *)p;p+=4;}
   if(code->params & 256) {code->F=*(float *)p;p+=4;}
   if(code->params & 512) {code->T=*p++;}
   if(code->params & 1024) {code->S=*(long int*)p;p+=4;}
   if(code->params & 2048) {code->P=*(long int*)p;p+=4;}
   if(GCODE_HAS_STRING(code)) { // read 16 byte into string
     char *sp = code->text;
     for(i=0;i<16;++i) *sp++ = *p++;
     *sp = 0; // make sure 0 is at end of string
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
  if((pos = strchr(line,'N'))!=0) { // Line number detected
     gcode_actN = gcode_value_long(++pos);
     code->params |=1;
     code->N = gcode_actN & 0xffff;
  }
  if((pos = strchr(line,'M'))!=0) { // M command
     code->M = gcode_value_long(++pos) & 0xff;
     code->params |= 2;
  }
  if((pos = strchr(line,'G'))!=0) { // G command
     code->G = gcode_value_long(++pos) & 0xff;
     code->params |= 4;
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
  if(GCODE_HAS_M(code) && (code->M == 23 || code->M == 28 || code->M == 29 || code->M == 30 || code->M == 117)) {
     // after M command we got a filename for sd card management
     char *sp = line;
     while(*sp!='M') sp++; // Search M command
     while(*sp!=' ') sp++; // search next whitespace
     while(*sp==' ') sp++; // skip leading whitespaces
     char *wp = code->text;
     while(*sp) {
        if(*sp==' ' || *sp=='*') break; // end of filename reached
        *wp++ = *sp++;
     }
     *wp = 0; // ensure ending 0 
     code->params |= 32768;
  }
  if((pos = strchr(line,'*'))!=0) { // checksum
    byte checksum_given = gcode_value_long(pos+1);
    byte checksum = 0;
    while(line!=pos) checksum ^= *line++;
    if(checksum!=checksum_given) {
      if(DEBUG_ERRORS) {
        out.println_int_P(PSTR("Error: Wrong checksum "),(int)checksum);
      }
      return false; // mismatch
    }
  }
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
  if(GCODE_HAS_STRING(code)) {
    out.print(' ');
    out.print(code->text);
  }  
}
