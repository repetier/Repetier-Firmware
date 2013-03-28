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

#ifndef _ui_h
#define _ui_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#include <avr/pgmspace.h>
#include "gcode.h"

// ----------------------------------------------------------------------------
//                          Action codes
// 1-999     : Autorepeat
// 1000-1999 : Execute
// 2000-2999 : Write code
// 4000-4999 : Show menu
// Add UI_ACTION_TOPMENU to show a menu as top menu
// ----------------------------------------------------------------------------

#define UI_ACTION_TOPMENU 8192

#define UI_ACTION_NEXT 1
#define UI_ACTION_PREVIOUS 2

#define UI_ACTION_X_UP                 100
#define UI_ACTION_X_DOWN               101
#define UI_ACTION_Y_UP                 102
#define UI_ACTION_Y_DOWN               103
#define UI_ACTION_Z_UP                 104
#define UI_ACTION_Z_DOWN               105
#define UI_ACTION_EXTRUDER_UP          106
#define UI_ACTION_EXTRUDER_DOWN        107
#define UI_ACTION_EXTRUDER_TEMP_UP     108
#define UI_ACTION_EXTRUDER_TEMP_DOWN   109
#define UI_ACTION_HEATED_BED_UP        110
#define UI_ACTION_HEATED_BED_DOWN      111
#define UI_ACTION_FAN_UP               112
#define UI_ACTION_FAN_DOWN             113

#define UI_ACTION_DUMMY 10000
#define UI_ACTION_BACK                  1000
#define UI_ACTION_OK                    1001
#define UI_ACTION_MENU_UP               1002
#define UI_ACTION_TOP_MENU              1003
#define UI_ACTION_EMERGENCY_STOP        1004
#define UI_ACTION_XPOSITION             1005
#define UI_ACTION_YPOSITION             1006
#define UI_ACTION_ZPOSITION             1007
#define UI_ACTION_EPOSITION             1008
#define UI_ACTION_BED_TEMP              1009
#define UI_ACTION_EXTRUDER_TEMP         1010
#define UI_ACTION_SD_DELETE             1012
#define UI_ACTION_SD_PRINT              1013
#define UI_ACTION_SD_PAUSE              1014
#define UI_ACTION_SD_CONTINUE           1015
#define UI_ACTION_SD_UNMOUNT            1016
#define UI_ACTION_SD_MOUNT              1017
#define UI_ACTION_XPOSITION_FAST        1018
#define UI_ACTION_YPOSITION_FAST        1019
#define UI_ACTION_ZPOSITION_FAST        1020
#define UI_ACTION_HOME_ALL              1021
#define UI_ACTION_HOME_X                1022
#define UI_ACTION_HOME_Y                1023
#define UI_ACTION_HOME_Z                1024
#define UI_ACTION_SELECT_EXTRUDER1      1025
#define UI_ACTION_OPS_RETRACTDISTANCE   1026
#define UI_ACTION_OPS_BACKLASH          1027
#define UI_ACTION_OPS_MOVE_AFTER        1028
#define UI_ACTION_OPS_MINDISTANCE       1029
#define UI_ACTION_STORE_EEPROM          1030
#define UI_ACTION_LOAD_EEPROM           1031
#define UI_ACTION_PRINT_ACCEL_X         1032
#define UI_ACTION_PRINT_ACCEL_Y         1033
#define UI_ACTION_PRINT_ACCEL_Z         1034
#define UI_ACTION_MOVE_ACCEL_X          1035
#define UI_ACTION_MOVE_ACCEL_Y          1036
#define UI_ACTION_MOVE_ACCEL_Z          1037
#define UI_ACTION_MAX_JERK              1038
#define UI_ACTION_MAX_ZJERK             1039
#define UI_ACTION_BAUDRATE              1040
#define UI_ACTION_HOMING_FEEDRATE_X     1041
#define UI_ACTION_HOMING_FEEDRATE_Y     1042
#define UI_ACTION_HOMING_FEEDRATE_Z     1043
#define UI_ACTION_MAX_FEEDRATE_X        1044
#define UI_ACTION_MAX_FEEDRATE_Y        1045
#define UI_ACTION_MAX_FEEDRATE_Z        1046
#define UI_ACTION_STEPS_X               1047
#define UI_ACTION_STEPS_Y               1048
#define UI_ACTION_STEPS_Z               1049
#define UI_ACTION_FAN_OFF               1050
#define UI_ACTION_FAN_25                1051
#define UI_ACTION_FAN_50                1052
#define UI_ACTION_FAN_75                1053
#define UI_ACTION_FAN_FULL              1054
#define UI_ACTION_FEEDRATE_MULTIPLY     1055
#define UI_ACTION_STEPPER_INACTIVE      1056
#define UI_ACTION_MAX_INACTIVE          1057
#define UI_ACTION_PID_PGAIN             1058
#define UI_ACTION_PID_IGAIN             1059
#define UI_ACTION_PID_DGAIN             1060
#define UI_ACTION_DRIVE_MIN             1061
#define UI_ACTION_DRIVE_MAX             1062
#define UI_ACTION_X_OFFSET              1063
#define UI_ACTION_Y_OFFSET              1064
#define UI_ACTION_EXTR_STEPS            1065
#define UI_ACTION_EXTR_ACCELERATION     1066
#define UI_ACTION_EXTR_MAX_FEEDRATE     1067
#define UI_ACTION_EXTR_START_FEEDRATE   1068
#define UI_ACTION_EXTR_HEATMANAGER      1069
#define UI_ACTION_EXTR_WATCH_PERIOD     1070
#define UI_ACTION_PID_MAX               1071
#define UI_ACTION_ADVANCE_K             1072
#define UI_ACTION_SET_ORIGIN            1073
#define UI_ACTION_DEBUG_ECHO            1074
#define UI_ACTION_DEBUG_INFO            1075
#define UI_ACTION_DEBUG_ERROR           1076
#define UI_ACTION_DEBUG_DRYRUN          1077
#define UI_ACTION_POWER                 1078
#define UI_ACTION_PREHEAT_PLA           1079
#define UI_ACTION_COOLDOWN              1080
#define UI_ACTION_HEATED_BED_OFF        1081
#define UI_ACTION_EXTRUDER0_OFF         1082
#define UI_ACTION_EXTRUDER1_OFF         1083
#define UI_ACTION_HEATED_BED_TEMP       1084
#define UI_ACTION_EXTRUDER0_TEMP        1085
#define UI_ACTION_EXTRUDER1_TEMP        1086
#define UI_ACTION_OPS_OFF               1087
#define UI_ACTION_OPS_CLASSIC           1088
#define UI_ACTION_OPS_FAST              1089
#define UI_ACTION_DISABLE_STEPPER       1090
#define UI_ACTION_RESET_EXTRUDER        1091
#define UI_ACTION_EXTRUDER_RELATIVE     1092
#define UI_ACTION_SELECT_EXTRUDER0      1093
#define UI_ACTION_ADVANCE_L             1094
#define UI_ACTION_PREHEAT_ABS           1095
#define UI_ACTION_FLOWRATE_MULTIPLY     1096
#define UI_ACTION_KILL                  1097
#define UI_ACTION_RESET                 1098
#define UI_ACTION_PAUSE                 1099
#define UI_ACTION_EXTR_WAIT_RETRACT_TEMP 1100
#define UI_ACTION_EXTR_WAIT_RETRACT_UNITS 1101

#define UI_ACTION_MENU_XPOS             4000
#define UI_ACTION_MENU_YPOS             4001
#define UI_ACTION_MENU_ZPOS             4002
#define UI_ACTION_MENU_XPOSFAST         4003
#define UI_ACTION_MENU_YPOSFAST         4004
#define UI_ACTION_MENU_ZPOSFAST         4005
#define UI_ACTION_MENU_SDCARD           4006
#define UI_ACTION_MENU_QUICKSETTINGS    4007
#define UI_ACTION_MENU_EXTRUDER         4008
#define UI_ACTION_MENU_POSITIONS        4009
#define UI_ACTION_SHOW_MEASUREMENT		4010
#define UI_ACTION_RESET_MEASUREMENT		4011
#define UI_ACTION_SET_MEASURED_ORIGIN	4012
#define UI_ACTION_SET_P1				4013
#define UI_ACTION_SET_P2				4014
#define UI_ACTION_SET_P3				4015
#define UI_ACTION_CALC_LEVEL			4016

#define UI_ACTION_SHOW_USERMENU1        4101
#define UI_ACTION_SHOW_USERMENU2        4102
#define UI_ACTION_SHOW_USERMENU3        4103
#define UI_ACTION_SHOW_USERMENU4        4104
#define UI_ACTION_SHOW_USERMENU5        4105
#define UI_ACTION_SHOW_USERMENU6        4106
#define UI_ACTION_SHOW_USERMENU7        4107
#define UI_ACTION_SHOW_USERMENU8        4108
#define UI_ACTION_SHOW_USERMENU9        4109
#define UI_ACTION_SHOW_USERMENU10       4110

// Load basic language definition to make sure all values are defined
#include "uilang.h"

#include "Configuration.h"
#include <avr/pgmspace.h>
#include "fastio.h"

typedef struct {
  const char *text; // Menu text 
  unsigned char menuType; // 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action command
  unsigned int action;
} const UIMenuEntry;

typedef struct {
  // 0 = info page
  // 1 = file selector
  // 2 = submenu
  // 3 = modififaction menu
  unsigned char menuType;
  int id; // Type of modification
  int numEntries;
  const UIMenuEntry * const * entries;
} const UIMenu;
extern const int8_t encoder_table[16] PROGMEM ;

//#ifdef COMPILE_I2C_DRIVER

/************************************************************************* 
* Title:    C include file for the I2C master interface 
*           (i2cmaster.S or twimaster.c)
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: i2cmaster.h,v 1.10 2005/03/06 22:39:57 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device
* Usage:    see Doxygen manual
**************************************************************************/

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

#include <avr/io.h>

/** defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start() */
#define I2C_WRITE   0

/** 
 @brief Terminates the data transfer and releases the I2C bus 
 @param void
 @return none
 */
extern void i2c_stop(void);
/** 
 @brief Issues a start condition and sends address and transfer direction 
  
 @param    addr address and transfer direction of I2C device
 @retval   0   device accessible 
 @retval   1   failed to access device 
 */
extern unsigned char i2c_start(unsigned char addr);
/**
 @brief Issues a start condition and sends address and transfer direction 
   
 If device is busy, use ack polling to wait until device ready 
 @param    addr address and transfer direction of I2C device
 @return   none
 */
extern void i2c_start_wait(unsigned char addr);
/**
 @brief Send one byte to I2C device
 @param    data  byte to be transfered
 @retval   0 write successful
 @retval   1 write failed
 */
extern unsigned char i2c_write(unsigned char data);
/**
 @brief    read one byte from the I2C device, request more data from device 
 @return   byte read from I2C device
 */
extern unsigned char i2c_readAck(void);
/**
 @brief    read one byte from the I2C device, read is followed by a stop condition 
 @return   byte read from I2C device
 */
extern unsigned char i2c_readNak(void);
/** 
 @brief    read one byte from the I2C device
 
 Implemented as a macro, which calls either i2c_readAck or i2c_readNak
 
 @param    ack 1 send ack, request more data from device<br>
               0 send nak, read is followed by a stop condition 
 @return   byte read from I2C device
 */
extern unsigned char i2c_read(unsigned char ack);
#define i2c_read(ack)  (ack) ? i2c_readAck() : i2c_readNak(); 
/**@}*/



//extern const int matrixActions[] PROGMEM;
// Key codes
#define UI_KEYS_INIT_CLICKENCODER_LOW(pinA,pinB) SET_INPUT(pinA);SET_INPUT(pinB); WRITE(pinA,HIGH);WRITE(pinB,HIGH);
#define UI_KEYS_INIT_BUTTON_LOW(pin) SET_INPUT(pin);WRITE(pin,HIGH);
#define UI_KEYS_INIT_CLICKENCODER_HIGH(pinA,pinB) SET_INPUT(pinA);SET_INPUT(pinB); WRITE(pinA,LOW);WRITE(pinB,LOW);
#define UI_KEYS_INIT_BUTTON_HIGH(pin) SET_INPUT(pin);WRITE(pin,LOW);

#define UI_KEYS_CLICKENCODER_LOW(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!READ(pinA)) uid.encoderLast |=2;if (!READ(pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_CLICKENCODER_LOW_REV(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!READ(pinA)) uid.encoderLast |=2;if (!READ(pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_BUTTON_LOW(pin,action_) if(READ(pin)==0) action=action_;
#define UI_KEYS_CLICKENCODER_HIGH(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (READ(pinA)) uid.encoderLast |=2;if (READ(pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_CLICKENCODER_HIGH_REV(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (READ(pinA)) uid.encoderLast |=2;if (READ(pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_BUTTON_HIGH(pin,action_) if(READ(pin)!=0) action=action_;
#define UI_KEYS_INIT_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4) if(c1>=0){SET_INPUT(c1);WRITE(c1,HIGH);}if(c2>=0){SET_INPUT(c2);WRITE(c2,HIGH);}if(c3>=0){SET_INPUT(c3);WRITE(c3,HIGH);}\
    if(c4>=0) {SET_INPUT(c4);WRITE(c4,HIGH);}if(r1>=0)SET_OUTPUT(r1);if(r2>=0)SET_OUTPUT(r2);if(r3>=0)SET_OUTPUT(r3);if(r4>=0)SET_OUTPUT(r4);\
    if(r1>=0)WRITE(r1,LOW);if(r2>=0)WRITE(r2,LOW);if(r3>=0)WRITE(r3,LOW);if(r4>=0)WRITE(r4,LOW);
//      out.print_int_P(PSTR("r4=>c1:"),READ(c1));out.print_int_P(PSTR(" c2:"),READ(c2));out.print_int_P(PSTR(" c3:"),READ(c3));out.println_int_P(PSTR(" c4:"),READ(c4));
#define UI_KEYS_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4) {byte r = (c1>=0?READ(c1):0) && (c2>=0?READ(c2):0) && (c3>=0?READ(c3):0) && (c4>=0?READ(c4):0);\
    if(!r) {\
      r = 255;\
      if(r2>=0)WRITE(r2,HIGH);if(r3>=0)WRITE(r3,HIGH);if(r4>=0)WRITE(r4,HIGH);\
      if(r1>=0) {\
        asm volatile ("nop\nnop\nnop\nnop\nnop");\
        if(!((c1>=0?READ(c1):1) && (c2>=0?READ(c2):1) && (c3>=0?READ(c3):1) && (c4>=0?READ(c4):1))) r = 0;\
        else WRITE(r1,HIGH);\
      }\
      if(r==255 && r2>=0) {\
        WRITE(r2,LOW);asm volatile ("nop\nnop\nnop\nnop\nnop");\
        if(!((c1>=0?READ(c1):1) && (c2>=0?READ(c2):1) && (c3>=0?READ(c3):1) && (c4>=0?READ(c4):1))) r = 4;\
        else WRITE(r2,HIGH);\
      }\
      if(r==255 && r3>=0) {\
        WRITE(r3,LOW);asm volatile ("nop\nnop\nnop\nnop\nnop");\
        if(!((c1>=0?READ(c1):0) && (c2>=0?READ(c2):1) && (c3>=0?READ(c3):1) && (c4>=0?READ(c4):1))) r = 8;\
        else WRITE(r3,HIGH);\
      }\
      if(r==255 && r4>=0) {\
        WRITE(r4,LOW);asm volatile ("nop\nnop\nnop\nnop\nnop");\
        if(!((c1>=0?READ(c1):1) && (c2>=0?READ(c2):1) && (c3>=0?READ(c3):1) && (c4>=0?READ(c4):1))) r = 12;\
        else WRITE(r4,HIGH);\
      }\
      if(c2>=0 && !READ(c2)) r+=1;\
      else if(c3>=0 && !READ(c3)) r+=2;\
      else if(c4>=0 && !READ(c4)) r+=3;\
      if(r<16) {action = pgm_read_word(&(matrixActions[r]));}\
    }if(r1>=0)WRITE(r1,LOW);if(r2>=0)WRITE(r2,LOW);if(r3>=0)WRITE(r3,LOW);if(r4>=0)WRITE(r4,LOW);}
// I2C keymask tests
#define UI_KEYS_I2C_CLICKENCODER_LOW(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!(keymask & pinA)) uid.encoderLast |=2;if (!(keymask & pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_CLICKENCODER_LOW_REV(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!(keymask & pinA)) uid.encoderLast |=2;if (!(keymask & pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_BUTTON_LOW(pin,action_) if((keymask & pin)==0) action=action_;
#define UI_KEYS_I2C_CLICKENCODER_HIGH(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (keymask & pinA) uid.encoderLast |=2;if (keymask & pinB) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_CLICKENCODER_HIGH_REV(pinA,pinB)  uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (keymask & pinA) uid.encoderLast |=2;if (keymask & pinB) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_BUTTON_HIGH(pin,action_) if((pin & keymask)!=0) action=action_;

#define UI_STRING(name,text) const char PROGMEM name[] = text;

#define UI_PAGE4(name,row1,row2,row3,row4) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {0,0,4,name ## _entries};
#define UI_PAGE2(name,row1,row2) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {0,0,2,name ## _entries};
#define UI_MENU_ACTION4C(name,action,rows) UI_MENU_ACTION4(name,action,rows)
#define UI_MENU_ACTION2C(name,action,rows) UI_MENU_ACTION2(name,action,rows)
#define UI_MENU_ACTION4(name,action,row1,row2,row3,row4) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {3,action,4,name ## _entries};
#define UI_MENU_ACTION2(name,action,row1,row2) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {3,action,2,name ## _entries};
#define UI_MENU_HEADLINE(name,text) UI_STRING(name ## _txt,text);UIMenuEntry name PROGMEM = {name ## _txt,1,0};
#define UI_MENU_CHANGEACTION(name,row,action) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,4,action};
#define UI_MENU_ACTIONCOMMAND(name,row,action) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,3,action};
#define UI_MENU_ACTIONSELECTOR(name,row,entries) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries};
#define UI_MENU_SUBMENU(name,row,entries) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries};
#define UI_MENU(name,items,itemsCnt) const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {2,0,itemsCnt,name ## _entries}
#define UI_MENU_FILESELECT(name,items,itemsCnt) const UIMenuEntry *name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {1,0,itemsCnt,name ## _entries}

#if FEATURE_CONTROLLER==2 // reprapdiscount smartcontroller has a sd card buildin 
#undef SDCARDDETECT
#define SDCARDDETECT 49
#undef SDCARDDETECTINVERTED
#define SDCARDDETECTINVERTED false
#undef SDSUPPORT
#define SDSUPPORT true
#endif

class UIDisplay {
  public:
    volatile byte flags; // 1 = fast key action, 2 = slow key action, 4 = slow action running, 8 = key test running
    byte col; // current col for buffer prefill
    byte menuLevel; // current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change
    byte menuPos[5]; // Positions in menu
    void *menu[5]; // Menus active
    byte menuTop[5]; // Top row in menu
    int pageDelay; // Counter. If 0 page is refreshed if menuLevel is 0.
    void *errorMsg;
    unsigned int activeAction; // action for ok/next/previous
    unsigned int lastAction;
    unsigned long lastSwitch; // Last time display switched pages
    unsigned long lastRefresh;
    unsigned int lastButtonAction;
    unsigned long lastButtonStart;
    unsigned long nextRepeat; // Time of next autorepeat
    unsigned int outputMask; // Output mask for backlight, leds etc.
    int repeatDuration; // Time beween to actions if autorepeat is enabled
    void addInt(int value,byte digits); // Print int into printCols
    void addLong(long value,char digits);
    void addFloat(float number, char fixdigits,byte digits);
    void addStringP(PGM_P text);
    void okAction();
    void nextPreviousAction(char next);
    char statusMsg[17];
    char encoderPos;
    int8_t encoderLast;
    PGM_P statusText;
    UIDisplay();
    void createChar(byte location,const byte charmap[]);
    void initialize(); // Initialize display and keys
    void printRow(byte r,char *txt); // Print row on display
    void printRowP(byte r,PGM_P txt);
    void parse(char *txt,bool ram); /// Parse output and write to printCols;
    void refreshPage();
    void executeAction(int action);
    void finishAction(int action);
    void slowAction();
    void fastAction();
    void mediumAction();
    void pushMenu(void *men,bool refresh);
    void setStatusP(PGM_P txt);
    void setStatus(char *txt);
    inline void setOutputMaskBits(unsigned int bits) {outputMask|=bits;}
    inline void unsetOutputMaskBits(unsigned int bits) {outputMask&=~bits;}
#if SDSUPPORT
    void updateSDFileCount();
    void sdrefresh(byte &r);
    void goDir(char *name);
    bool isDirname(char *name);
    char cwd[SD_MAX_FOLDER_DEPTH*13+2];
    byte folderLevel;
#endif
};
extern UIDisplay uid;


#if FEATURE_CONTROLLER==1
#include "uiconfig.h"
#endif
#if FEATURE_CONTROLLER==0 // No controller at all
#define UI_HAS_KEYS 0
#define UI_DISPLAY_TYPE 0
#ifdef UI_MAIN
void ui_init_keys() {}
void ui_check_keys(int &action) {}
inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif
#endif
#if FEATURE_CONTROLLER==2 // reprapdiscount smartcontroller
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 0
#define UI_DISPLAY_TYPE 1
#define UI_DISPLAY_CHARSET 1
#define BEEPER_TYPE 1
#define UI_COLS 20
#define UI_ROWS 4
#if MOTHERBOARD==80 // Rumba has different pins as RAMPS!
#define BEEPER_PIN             44
#define UI_DISPLAY_RS_PIN      19
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  42
#define UI_DISPLAY_D0_PIN      18
#define UI_DISPLAY_D1_PIN      38
#define UI_DISPLAY_D2_PIN      41
#define UI_DISPLAY_D3_PIN      40
#define UI_DISPLAY_D4_PIN      18
#define UI_DISPLAY_D5_PIN      38
#define UI_DISPLAY_D6_PIN      41
#define UI_DISPLAY_D7_PIN      40
#define UI_ENCODER_A           12
#define UI_ENCODER_B           11
#define UI_ENCODER_CLICK       43
#define UI_RESET_PIN           46
#else
#define BEEPER_PIN             37
#define UI_DISPLAY_RS_PIN      16
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  17
#define UI_DISPLAY_D0_PIN      23
#define UI_DISPLAY_D1_PIN      25
#define UI_DISPLAY_D2_PIN      27
#define UI_DISPLAY_D3_PIN      29
#define UI_DISPLAY_D4_PIN      23
#define UI_DISPLAY_D5_PIN      25
#define UI_DISPLAY_D6_PIN      27
#define UI_DISPLAY_D7_PIN      29
#define UI_ENCODER_A           33
#define UI_ENCODER_B           31
#define UI_ENCODER_CLICK       35
#define UI_RESET_PIN           41
#endif
#define UI_DELAYPERCHAR 320
#define UI_INVERT_MENU_DIRECTION false
#ifdef UI_MAIN
void ui_init_keys() {  
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A,UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(UI_RESET_PIN); // Kill pin
}
void ui_check_keys(int &action) {
 UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A,UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
 UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK,UI_ACTION_OK); // push button, connects gnd to pin
 UI_KEYS_BUTTON_LOW(UI_RESET_PIN,UI_ACTION_RESET); 
}
inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif
#endif // Controller 2

#if FEATURE_CONTROLLER==3 // Adafruit RGB controller
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE 3
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 16
#define UI_ROWS 2
#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 65504
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0
#define UI_DISPLAY_I2C_PULLUP 31
#define UI_I2C_CLOCKSPEED 400000L
#define UI_DISPLAY_RS_PIN _BV(15)
#define UI_DISPLAY_RW_PIN _BV(14)
#define UI_DISPLAY_ENABLE_PIN _BV(13)
#define UI_DISPLAY_D0_PIN _BV(12)
#define UI_DISPLAY_D1_PIN _BV(11)
#define UI_DISPLAY_D2_PIN _BV(10)
#define UI_DISPLAY_D3_PIN _BV(9)
#define UI_DISPLAY_D4_PIN _BV(12)
#define UI_DISPLAY_D5_PIN _BV(11)
#define UI_DISPLAY_D6_PIN _BV(10)
#define UI_DISPLAY_D7_PIN _BV(9)
#define UI_INVERT_MENU_DIRECTION true
#define UI_HAS_I2C_KEYS
#define UI_HAS_I2C_ENCODER 0
#define UI_I2C_KEY_ADDRESS 0x40
#ifdef UI_MAIN
void ui_init_keys() {}
void ui_check_keys(int &action) {}
inline void ui_check_slow_encoder() {
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
  i2c_write(0x12); // GIOA
  i2c_stop();
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_READ);
  unsigned int keymask = i2c_readAck();
  keymask = keymask + (i2c_readNak()<<8);
  i2c_stop();
}
void ui_check_slow_keys(int &action) {
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
  i2c_write(0x12); // GPIOA
  i2c_stop();
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_READ);
  unsigned int keymask = i2c_readAck();
  keymask = keymask + (i2c_readNak()<<8);
  i2c_stop();
  UI_KEYS_I2C_BUTTON_LOW(4,UI_ACTION_PREVIOUS); // Up button
  UI_KEYS_I2C_BUTTON_LOW(8,UI_ACTION_NEXT); // down button
  UI_KEYS_I2C_BUTTON_LOW(16,UI_ACTION_BACK); // left button
  UI_KEYS_I2C_BUTTON_LOW(2,UI_ACTION_OK); // right button
  UI_KEYS_I2C_BUTTON_LOW(1,UI_ACTION_MENU_QUICKSETTINGS);  //Select button
}
#endif
#endif // Controller 3

#if FEATURE_CONTROLLER==4 // Foltyn 3D Master
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE 1
#define UI_DISPLAY_CHARSET 2
#define UI_COLS 20
#define UI_ROWS 4
#define UI_DISPLAY_RS_PIN		63		// PINK.1, 88, D_RS
#define UI_DISPLAY_RW_PIN		-1
#define UI_DISPLAY_ENABLE_PIN	        65		// PINK.3, 86, D_E
#define UI_DISPLAY_D0_PIN		59		// PINF.5, 92, D_D4
#define UI_DISPLAY_D1_PIN		64		// PINK.2, 87, D_D5
#define UI_DISPLAY_D2_PIN		44		// PINL.5, 40, D_D6
#define UI_DISPLAY_D3_PIN		66		// PINK.4, 85, D_D7
#define UI_DISPLAY_D4_PIN		59		// PINF.5, 92, D_D4
#define UI_DISPLAY_D5_PIN		64		// PINK.2, 87, D_D5
#define UI_DISPLAY_D6_PIN		44		// PINL.5, 40, D_D6
#define UI_DISPLAY_D7_PIN		66		// PINK.4, 85, D_D7
#define UI_DELAYPERCHAR		   320
#define UI_INVERT_MENU_DIRECTION false
#ifdef UI_MAIN
void ui_init_keys() {  
  UI_KEYS_INIT_BUTTON_LOW(4); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(5);
  UI_KEYS_INIT_BUTTON_LOW(6);
  UI_KEYS_INIT_BUTTON_LOW(11);
  UI_KEYS_INIT_BUTTON_LOW(42);
}
void ui_check_keys(int &action) {
 UI_KEYS_BUTTON_LOW(4,UI_ACTION_OK); // push button, connects gnd to pin
 UI_KEYS_BUTTON_LOW(5,UI_ACTION_NEXT); // push button, connects gnd to pin
 UI_KEYS_BUTTON_LOW(6,UI_ACTION_PREVIOUS); // push button, connects gnd to pin
 UI_KEYS_BUTTON_LOW(11,UI_ACTION_BACK); // push button, connects gnd to pin
 UI_KEYS_BUTTON_LOW(42,UI_ACTION_SD_PRINT ); // push button, connects gnd to pin
}
inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif
#endif // Controller 4

#if FEATURE_CONTROLLER==5 // Viki Lcd
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE 3
#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4
#define UI_DISPLAY_I2C_CHIPTYPE 1
#define UI_DISPLAY_I2C_ADDRESS 0x40
#define UI_DISPLAY_I2C_OUTPUT_PINS 0xFFE0
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0x01C0 // bits that are high always, for now the 3 viki leds
#define UI_DISPLAY_I2C_PULLUP 0x001F
#define UI_I2C_CLOCKSPEED 100000L // Note with very long cables make this much smaller, for 2ft cables I found 80000 worked ok

#define UI_DISPLAY_RS_PIN _BV(15)
#define UI_DISPLAY_RW_PIN _BV(14)
#define UI_DISPLAY_ENABLE_PIN _BV(13)
#define UI_DISPLAY_D0_PIN _BV(12)
#define UI_DISPLAY_D1_PIN _BV(11)
#define UI_DISPLAY_D2_PIN _BV(10)
#define UI_DISPLAY_D3_PIN _BV(9)
#define UI_DISPLAY_D4_PIN _BV(12)
#define UI_DISPLAY_D5_PIN _BV(11)
#define UI_DISPLAY_D6_PIN _BV(10)
#define UI_DISPLAY_D7_PIN _BV(9)

#define BEEPER_PIN        _BV(5)
#define BEEPER_TYPE       2
#define BEEPER_ADDRESS    UI_DISPLAY_I2C_ADDRESS // I2C address of the chip with the beeper pin
#define UI_HEATBED_LED    _BV(8)
#define UI_HOTEND_LED     _BV(7)
#define UI_FAN_LED        _BV(6)

// RAMPS
#define UI_ENCODER_A      16 // pins the click encoder are connected to
#define UI_ENCODER_B      17 

// Azteeg
//#define UI_ENCODER_A      7 // pins the click encoder are connected to
//#define UI_ENCODER_B      22 

#define UI_INVERT_MENU_DIRECTION true
#define UI_HAS_I2C_KEYS
#define UI_HAS_I2C_ENCODER 0
#define UI_I2C_KEY_ADDRESS 0x40
#ifdef UI_MAIN
void ui_init_keys() {
	UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A,UI_ENCODER_B); // click encoder on real pins. Phase is connected with gnd for signals.
}
void ui_check_keys(int &action) {
	UI_KEYS_CLICKENCODER_LOW_REV(UI_ENCODER_A,UI_ENCODER_B); // click encoder on real pins
}
inline void ui_check_slow_encoder() { }// not used in Viki

void ui_check_slow_keys(int &action) {
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
  i2c_write(0x12); // GPIOA
  i2c_stop();
  i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_READ);
  unsigned int keymask = i2c_readAck();
  keymask = keymask + (i2c_readNak()<<8);
  i2c_stop();
  UI_KEYS_I2C_BUTTON_LOW(4,UI_ACTION_PREVIOUS); // Up button
  UI_KEYS_I2C_BUTTON_LOW(8,UI_ACTION_NEXT); // down button
  UI_KEYS_I2C_BUTTON_LOW(16,UI_ACTION_BACK); // left button
  UI_KEYS_I2C_BUTTON_LOW(2,UI_ACTION_OK); // right button
  UI_KEYS_I2C_BUTTON_LOW(1,UI_ACTION_MENU_QUICKSETTINGS);  //Select button
}
#endif
#endif // Controller 5

#if FEATURE_CONTROLLER>0

#if UI_ROWS==4
#if UI_COLS==16
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 4x16
#elif UI_COLS==20
//#define UI_LINE_OFFSETS {0,0x20,0x40,0x60} // 4x20 with KS0073
#define UI_LINE_OFFSETS {0,0x40,0x14,0x54} // 4x20 with HD44780
#else
#error Unknown combination off rows/columns - define UI_LINE_OFFSETS manually.
#endif
#else
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 2x16, 2x20, 2x24
#endif
#include "uilang.h"
#include "uimenu.h"
#endif

#define UI_VERSION_STRING "Repetier " REPETIER_VERSION

#ifdef UI_HAS_I2C_KEYS
#define COMPILE_I2C_DRIVER
#endif

#if UI_DISPLAY_TYPE!=0


#if UI_DISPLAY_TYPE==3
#define COMPILE_I2C_DRIVER
#endif

#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 1
#else
#define UI_TEMP_PRECISION 0
#endif
#endif

#define UI_INITIALIZE uid.initialize();
#define UI_FAST if(pwm_count & 4) {uid.fastAction();}
#define UI_MEDIUM uid.mediumAction();
#define UI_SLOW uid.slowAction();
#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_UPD(status) {uid.setStatusP(PSTR(status));uid.refreshPage();}
#define UI_STATUS_RAM(status) uid.setStatus(status);
#define UI_STATUS_UPD_RAM(status) {uid.setStatus(status);uid.refreshPage();}
#define UI_ERROR(msg) {uid.errorMsg=PSTR(msg);pushMenu((void*)&ui_menu_error,true);}
#define UI_CLEAR_STATUS {uid.statusMsg[0]=0;}
#else
#define UI_INITIALIZE {}
#define UI_FAST {}
#define UI_MEDIUM {}
#define UI_SLOW {}
#define UI_STATUS(status) {}
#define UI_STATUS_UPD(status) {}
#define UI_CLEAR_STATUS {}
#define UI_ERROR(msg) {}
#define UI_STATUS_UPD_RAM(status) {}
#endif  // Display

// Beeper methods
#if BEEPER_TYPE==0
#define BEEP_SHORT {}
#define BEEP_LONG {}
#else
#define BEEP_SHORT beep(BEEPER_SHORT_SEQUENCE);
#define BEEP_LONG beep(BEEPER_LONG_SEQUENCE);
#endif
extern void beep(byte duration,byte count);

#endif

