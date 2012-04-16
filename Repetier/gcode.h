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
   unsigned int N; // Line number
   byte M;
   byte G;
   float X;
   float Y;
   float Z;
   float E;
   float F;
   byte T;
   long S;
   long P;
   char text[17];
} GCode;

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
  void printFloat(double number, uint8_t digits=2); 

};

extern SerialOutput out;
/** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
extern GCode *gcode_next_command();
/** Frees the cache used by the last command fetched. */ 
extern void gcode_command_finished();
// check for new commands
extern void gcode_read_serial();
extern void gcode_print_command(GCode *code);
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
#define GCODE_HAS_STRING(a) ((a->params & 32768)!=0)

extern byte debug_level;
#define DEBUG_ECHO ((debug_level & 1)!=0)
#define DEBUG_INFO ((debug_level & 2)!=0)
#define DEBUG_ERRORS ((debug_level & 4)!=0)
#define DEBUG_DRYRUN ((debug_level & 8)!=0)
#define DEBUG_COMMUNICATION ((debug_level & 16)!=0)
#define DEBUG_NO_MOVES ((debug_level & 32)!=0)

#endif

