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
#ifndef _GCODE_H
#define _GCODE_H

#include <avr/pgmspace.h>
// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734 
//#ifdef PROGMEM 
//#undef PROGMEM 
//#define PROGMEM __attribute__((section(".progmem.data"))) 
//#endif

typedef struct { // 52 bytes per command needed
   unsigned int params;
   unsigned int params2;
   unsigned int N; // Line number
   unsigned int M;
   unsigned int G;
   float X;
   float Y;
   float Z;
   float E;
   float F;
   byte T;
   long S;
   long P;
   float I;
   float J;
   float R;   
   char *text; //text[17];
} GCode;

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

#define SERIAL_BUFFER_SIZE 128
#define SERIAL_BUFFER_MASK 127

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};
class RFHardwareSerial : public Print
{
  public:
    ring_buffer *_rx_buffer;
    ring_buffer *_tx_buffer;
    volatile uint8_t *_ubrrh;
    volatile uint8_t *_ubrrl;
    volatile uint8_t *_ucsra;
    volatile uint8_t *_ucsrb;
    volatile uint8_t *_udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udrie;
    uint8_t _u2x;
  public:
    RFHardwareSerial(ring_buffer *rx_buffer, ring_buffer *tx_buffer,
      volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
      volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
      volatile uint8_t *udr,
      uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x);
    void begin(unsigned long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
#ifdef COMPAT_PRE1
    virtual void write(uint8_t);
#else
    virtual size_t write(uint8_t);
#endif
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool();
};
extern RFHardwareSerial RFSerial; 
#define RFSERIAL RFSerial
extern ring_buffer tx_buffer;
#define WAIT_OUT_EMPTY while(tx_buffer.head != tx_buffer.tail) {}
#else
#define RFSERIAL Serial
#endif

class SerialOutput : public Print {
public:
  SerialOutput();
#ifdef COMPAT_PRE1
  void write(uint8_t);
#else
  size_t write(uint8_t);
#endif
  void print_P(PGM_P ptr);
  void println_P(PGM_P ptr);
  void print_long_P(PGM_P ptr,long value);
  void print_int_P(PGM_P ptr,int value);
  void print_float_P(PGM_P ptr,float value,uint8_t digits = 2);
  void println_long_P(PGM_P ptr,long value);
  void println_int_P(PGM_P ptr,int value);
  void println_float_P(PGM_P ptr,float value,uint8_t digits = 2);
  void print_error_P(PGM_P ptr,bool newline);
  void printFloat(double number, uint8_t digits=2); 

};
#define OUT_P_I(p,i) out.print_int_P(PSTR(p),(int)(i))
#define OUT_P_I_LN(p,i) out.println_int_P(PSTR(p),(int)(i))
#define OUT_P_L(p,i) out.print_long_P(PSTR(p),(long)(i))
#define OUT_P_L_LN(p,i) out.println_long_P(PSTR(p),(long)(i))
#define OUT_P_F(p,i) out.print_float_P(PSTR(p),(float)(i))
#define OUT_P_F_LN(p,i) out.println_float_P(PSTR(p),(float)(i))
#define OUT_P_FX(p,i,x) out.print_float_P(PSTR(p),(float)(i),x)
#define OUT_P_FX_LN(p,i,x) out.println_float_P(PSTR(p),(float)(i),x)
#define OUT_P(p) out.print_P(PSTR(p))
#define OUT_P_LN(p) out.println_P(PSTR(p))
#define OUT_ERROR_P(p) out.print_error_P(PSTR(p),false)
#define OUT_ERROR_P_LN(p) out.print_error_P(PSTR(p),true)
#define OUT(v) out.print(v)
#define OUT_LN out.println()
extern SerialOutput out;
/** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
extern GCode *gcode_next_command();
/** Frees the cache used by the last command fetched. */ 
extern void gcode_command_finished(GCode *code);
// check for new commands
extern void gcode_read_serial();
extern void gcode_execute_PString(PGM_P cmd);
extern void gcode_print_command(GCode *code);
extern byte gcode_comp_binary_size(char *ptr);
extern bool gcode_parse_binary(GCode *code,byte *buffer);
extern bool gcode_parse_ascii(GCode *code,char *line);
extern void emergencyStop();

// Helper macros to detect, if parameter is stored in GCode struct
#define GCODE_HAS_N(a) ((a->params & 1)!=0)
#define GCODE_HAS_M(a) ((a->params & 2)!=0)
#define GCODE_HAS_G(a) ((a->params & 4)!=0)
#define GCODE_HAS_X(a) ((a->params & 8)!=0)
#define GCODE_HAS_Y(a) ((a->params & 16)!=0)
#define GCODE_HAS_Z(a) ((a->params & 32)!=0)
#define GCODE_HAS_NO_XYZ(a) ((a->params & 56)==0)
#define GCODE_HAS_E(a) ((a->params & 64)!=0)
#define GCODE_HAS_F(a) ((a->params & 256)!=0)
#define GCODE_HAS_T(a) ((a->params & 512)!=0)
#define GCODE_HAS_S(a) ((a->params & 1024)!=0)
#define GCODE_HAS_P(a) ((a->params & 2048)!=0)
#define GCODE_IS_V2(a) ((a->params & 4096)!=0)
#define GCODE_HAS_STRING(a) ((a->params & 32768)!=0)
#define GCODE_HAS_I(a) ((a->params2 & 1)!=0)
#define GCODE_HAS_J(a) ((a->params2 & 2)!=0)
#define GCODE_HAS_R(a) ((a->params2 & 4)!=0)

extern byte debug_level;
#define DEBUG_ECHO ((debug_level & 1)!=0)
#define DEBUG_INFO ((debug_level & 2)!=0)
#define DEBUG_ERRORS ((debug_level & 4)!=0)
#define DEBUG_DRYRUN ((debug_level & 8)!=0)
#define DEBUG_COMMUNICATION ((debug_level & 16)!=0)
#define DEBUG_NO_MOVES ((debug_level & 32)!=0)

#endif

