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

#include "gcode.h"

#define NO_DISPLAY  0
#define DISPLAY_4BIT 1
#define DISPLAY_8BIT 2
#define DISPLAY_I2C   3
#define DISPLAY_ARDUINO_LIB  4
#define DISPLAY_U8G  5
#define DISPLAY_GAMEDUINO2 6

/**
  What display type do you use?
  0 = No display
  1 = LCD Display with 4 bit data bus
  2 = LCD Display with 8 bit data bus (currently not implemented, fallback to 1)
  3 = LCD Display with I2C connection, 4 bit mode
  4 = Use the slower LiquiedCrystal library bundled with arduino.
    IMPORTANT: You need to uncomment the LiquidCrystal include in Repetier.pde for it to work.
               If you have Sanguino and want to use the library, you need to have Arduino 023 or older. (13.04.2012)
  5 = U8G supported display
*/

// ----------------------------------------------------------------------------
//                          Action codes
// 1-999     : Autorepeat
// 1000-1999 : Execute
// 2000-2999 : Write code
// 4000-4999 : Show menu
// 5000-5999 : Wizard pages
// Add UI_ACTION_TOPMENU to show a menu as top menu
// Add UI_ACTION_NO_AUTORETURN to prevent autoreturn to start display
// ----------------------------------------------------------------------------

#define UI_ACTION_TOPMENU 8192
#define UI_ACTION_NO_AUTORETURN 16384

#define UI_ACTION_NEXT 1
#define UI_ACTION_PREVIOUS 2

#define UI_ACTION_X_UP                   100
#define UI_ACTION_X_DOWN                 101
#define UI_ACTION_Y_UP                   102
#define UI_ACTION_Y_DOWN                 103
#define UI_ACTION_Z_UP                   104
#define UI_ACTION_Z_DOWN                 105
#define UI_ACTION_EXTRUDER_UP            106
#define UI_ACTION_EXTRUDER_DOWN          107
#define UI_ACTION_EXTRUDER_TEMP_UP       108
#define UI_ACTION_EXTRUDER_TEMP_DOWN     109
#define UI_ACTION_HEATED_BED_UP          110
#define UI_ACTION_HEATED_BED_DOWN        111
#define UI_ACTION_FAN_UP                 112
#define UI_ACTION_FAN_DOWN               113

#define UI_ACTION_DUMMY                 10000
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

#define UI_ACTION_POWER                 1078
#define UI_ACTION_PREHEAT_PLA           1079
#define UI_ACTION_COOLDOWN              1080
#define UI_ACTION_HEATED_BED_OFF        1081
#define UI_ACTION_EXTRUDER0_OFF         1082
#define UI_ACTION_EXTRUDER1_OFF         1083
#define UI_ACTION_EXTRUDER2_OFF         1084
#define UI_ACTION_EXTRUDER3_OFF         1085
#define UI_ACTION_EXTRUDER4_OFF         1086
#define UI_ACTION_EXTRUDER5_OFF         1087
#define UI_ACTION_OPS_OFF               1088
#define UI_ACTION_OPS_CLASSIC           1089
#define UI_ACTION_OPS_FAST              1090
#define UI_ACTION_DISABLE_STEPPER       1091
#define UI_ACTION_RESET_EXTRUDER        1092
#define UI_ACTION_EXTRUDER_RELATIVE     1093
#define UI_ACTION_ADVANCE_L             1094
#define UI_ACTION_PREHEAT_ABS           1095
#define UI_ACTION_FLOWRATE_MULTIPLY     1096
#define UI_ACTION_KILL                  1097
#define UI_ACTION_RESET                 1098
#define UI_ACTION_PAUSE                 1099
#define UI_ACTION_EXTR_WAIT_RETRACT_TEMP 1100
#define UI_ACTION_EXTR_WAIT_RETRACT_UNITS 1101
#define UI_ACTION_WRITE_DEBUG           1105
#define UI_ACTION_FANSPEED              1106
#define UI_ACTION_LIGHTS_ONOFF          1107
#define UI_ACTION_SD_STOP               1108
#define UI_ACTION_ZPOSITION_NOTEST      1109
#define UI_ACTION_ZPOSITION_FAST_NOTEST 1110
#define UI_ACTION_Z_BABYSTEPS           1111
#define UI_ACTION_MAX_INACTIVE          1112
#define UI_ACTION_TEMP_DEFECT           1113
#define UI_ACTION_BED_HEATMANAGER       1114
#define UI_ACTION_BED_PGAIN             1115
#define UI_ACTION_BED_IGAIN             1116
#define UI_ACTION_BED_DGAIN             1117
#define UI_ACTION_BED_DRIVE_MIN         1118
#define UI_ACTION_BED_DRIVE_MAX         1119
#define UI_ACTION_BED_MAX               1120
#define UI_ACTION_HEATED_BED_TEMP       1121
#define UI_ACTION_EXTRUDER0_TEMP        1122
#define UI_ACTION_EXTRUDER1_TEMP        1123
#define UI_ACTION_EXTRUDER2_TEMP        1124
#define UI_ACTION_EXTRUDER3_TEMP        1125
#define UI_ACTION_EXTRUDER4_TEMP        1126
#define UI_ACTION_EXTRUDER5_TEMP        1127
#define UI_ACTION_SELECT_EXTRUDER0      1128
#define UI_ACTION_SELECT_EXTRUDER1      1129
#define UI_ACTION_SELECT_EXTRUDER2      1130
#define UI_ACTION_SELECT_EXTRUDER3      1131
#define UI_ACTION_SELECT_EXTRUDER4      1132
#define UI_ACTION_SELECT_EXTRUDER5      1133
#define UI_DITTO_0                      1134
#define UI_DITTO_1                      1135
#define UI_DITTO_2                      1136
#define UI_DITTO_3                      1137

#define UI_ACTION_DEBUG_ECHO            1150
#define UI_ACTION_DEBUG_INFO            1151
#define UI_ACTION_DEBUG_ERROR           1152
#define UI_ACTION_DEBUG_DRYRUN          1153
#define UI_ACTION_DEBUG_ENDSTOP         1154

#define UI_ACTION_SD_PRI_PAU_CONT       1200
#define UI_ACTION_FAN_SUSPEND           1201
#define UI_ACTION_AUTOLEVEL_ONOFF       1202
#define UI_ACTION_SERVOPOS              1203
#define UI_ACTION_IGNORE_M106           1204

#define UI_ACTION_KAPTON                1205
#define UI_ACTION_BLUETAPE              1206
#define UI_ACTION_NOCOATING             1207
#define UI_ACTION_PETTAPE               1208
#define UI_ACTION_GLUESTICK             1209
#define UI_ACTION_RESET_MATRIX          1210
#define UI_ACTION_CALIBRATE             1211
#define UI_ACTION_BED_LED_CHANGE        1212
#define UI_ACTION_COATING_CUSTOM        1213
#define UI_ACTION_BUILDTAK              1214

// 1700-1956 language selectors

#define UI_ACTION_LANGUAGE_EN           1700
#define UI_ACTION_LANGUAGE_DE           1701
#define UI_ACTION_LANGUAGE_NL           1702
#define UI_ACTION_LANGUAGE_PT           1703
#define UI_ACTION_LANGUAGE_IT           1704
#define UI_ACTION_LANGUAGE_ES           1705
#define UI_ACTION_LANGUAGE_SE           1706
#define UI_ACTION_LANGUAGE_FR           1707
#define UI_ACTION_LANGUAGE_CZ           1708
#define UI_ACTION_LANGUAGE_PL           1709
#define UI_ACTION_LANGUAGE_TR           1710
#define UI_ACTION_LANGUAGE_FI           1711

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
//#define UI_ACTION_SHOW_MEASUREMENT    4010
//#define UI_ACTION_RESET_MEASUREMENT   4011
#define UI_ACTION_SET_MEASURED_ORIGIN   4012
#define UI_ACTION_SET_P1                4013
#define UI_ACTION_SET_P2                4014
#define UI_ACTION_SET_P3                4015
#define UI_ACTION_CALC_LEVEL            4016
#define UI_ACTION_XOFF                  4020
#define UI_ACTION_YOFF                  4021
#define UI_ACTION_ZOFF                  4022

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

#define UI_ACTION_WIZARD_FILAMENTCHANGE 5000
#define UI_ACTION_WIZARD_JAM_REHEAT     5001
#define UI_ACTION_WIZARD_JAM_WAITHEAT   5002
#define UI_ACTION_WIZARD_JAM_EOF        5003
#define UI_ACTION_WIZARD_BED_LEVEL      5004

// Load basic language definition to make sure all values are defined
//#include "uilang.h"

#define UI_MENU_TYPE_INFO                   0
#define UI_MENU_TYPE_FILE_SELECTOR          1
#define UI_MENU_TYPE_SUBMENU                2
#define UI_MENU_TYPE_MODIFICATION_MENU      3
#define UI_MENU_TYPE_WIZARD                 5

struct UIMenuEntry_s {
  const char *text; // Menu text
  uint8_t entryType; // 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action command,
  unsigned int action; // must be int so it gets 32 bit on arm!
  uint8_t filter; // allows dynamic menu filtering based on Printer::menuMode bits set.
  uint8_t nofilter; // Hide if one of these bits are set
  int translation; // Translation id
  bool showEntry() const;
} ;
typedef const UIMenuEntry_s UIMenuEntry;

struct UIMenu_s {
  // 0 = info page
  // 1 = file selector
  // 2 = submenu
  // 3 = modififaction menu
  // 5 = Wizard menu
  // +128 = sticky -> no autoreturn to main
  uint8_t menuType;
  int id; // Type of modification
  int numEntries;
  const UIMenuEntry * const * entries;
};
typedef const UIMenu_s UIMenu;

extern const int8_t encoder_table[16] PROGMEM ;

//#ifdef COMPILE_I2C_DRIVER

/*************************************************************************
  Title:    C include file for the I2C master interface
            (i2cmaster.S or twimaster.c)
  Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
  File:     $Id: i2cmaster.h,v 1.10 2005/03/06 22:39:57 Peter Exp $
  Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
  Target:   any AVR device
  Usage:    see Doxygen manual
**************************************************************************/

//extern const int matrixActions[] PROGMEM;
// Key codes
#define UI_KEYS_INIT_CLICKENCODER_LOW(pinA,pinB) SET_INPUT(pinA);SET_INPUT(pinB); PULLUP(pinA,HIGH);PULLUP(pinB,HIGH);
#define UI_KEYS_INIT_BUTTON_LOW(pin) SET_INPUT(pin);PULLUP(pin,HIGH);
#define UI_KEYS_INIT_CLICKENCODER_HIGH(pinA,pinB) SET_INPUT(pinA);SET_INPUT(pinB); PULLUP(pinA,LOW);PULLUP(pinB,LOW);
#define UI_KEYS_INIT_BUTTON_HIGH(pin) SET_INPUT(pin);PULLUP(pin,LOW);

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
#define UI_KEYS_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4) {uint8_t r = (c1>=0?READ(c1):0) && (c2>=0?READ(c2):0) && (c3>=0?READ(c3):0) && (c4>=0?READ(c4):0);\
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
#define UI_KEYS_I2C_BUTTON_HIGH(pin,action_) if((pin & keymask) != 0) action=action_;

#define UI_STRING(name,text) const char PROGMEM name[] = text

#define UI_PAGE6(name,row1,row2,row3,row4,row5,row6) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);UI_STRING(name ## _5txt,row5);UI_STRING(name ## _6txt,row6);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0,0};\
  UIMenuEntry name ## _5 PROGMEM ={name ## _5txt,0,0,0,0,0};\
  UIMenuEntry name ## _6 PROGMEM ={name ## _6txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4,&name ## _5,&name ## _6};\
  const UIMenu name PROGMEM = {0,0,6,name ## _entries};
#define UI_PAGE6_T(name,row1,row2,row3,row4,row5,row6) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  UIMenuEntry name ## _3 PROGMEM ={0,0,0,0,0,row3};\
  UIMenuEntry name ## _4 PROGMEM ={0,0,0,0,0,row4};\
  UIMenuEntry name ## _5 PROGMEM ={0,0,0,0,0,row5};\
  UIMenuEntry name ## _6 PROGMEM ={0,0,0,0,0,row6};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4,&name ## _5,&name ## _6};\
  const UIMenu name PROGMEM = {0,0,6,name ## _entries};
#define UI_PAGE4(name,row1,row2,row3,row4) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {0,0,4,name ## _entries};
#define UI_PAGE4_T(name,row1,row2,row3,row4) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  UIMenuEntry name ## _3 PROGMEM ={0,0,0,0,0,row3};\
  UIMenuEntry name ## _4 PROGMEM ={0,0,0,0,0,row4};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {0,0,4,name ## _entries};
#define UI_PAGE2(name,row1,row2) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {0,0,2,name ## _entries};
#define UI_PAGE2_T(name,row1,row2) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {0,0,2,name ## _entries};
#define UI_WIZARD4(name,action,row1,row2,row3,row4) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {5,action,4,name ## _entries};
#define UI_WIZARD4_T(name,action,row1,row2,row3,row4) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  UIMenuEntry name ## _3 PROGMEM ={0,0,0,0,0,row3};\
  UIMenuEntry name ## _4 PROGMEM ={0,0,0,0,0,row4};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {5,action,4,name ## _entries};
#define UI_BED_WIZARD_T(name,action,row1,row2,row3,row4) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  UIMenuEntry name ## _3 PROGMEM ={0,0,0,0,0,row3};\
  UIMenuEntry name ## _4 PROGMEM ={0,0,0,0,0,row4};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {5,action,4,name ## _entries};
#define UI_WIZARD2(name,action,row1,row2) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {5,action,2,name ## _entries};
#define UI_WIZARD2_T(name,action,row1,row2) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {5,action,2,name ## _entries};
#define UI_MENU_ACTION4C(name,action,rows) UI_MENU_ACTION4(name,action,rows)
#define UI_MENU_ACTION2C(name,action,rows) UI_MENU_ACTION2(name,action,rows)
#define UI_MENU_ACTION4C_T(name,action,rows) UI_MENU_ACTION4_T(name,action,rows)
#define UI_MENU_ACTION2C_T(name,action,rows) UI_MENU_ACTION2_T(name,action,rows)
#define UI_MENU_ACTION4(name,action,row1,row2,row3,row4) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {3,action,4,name ## _entries};
#define UI_MENU_ACTION4_T(name,action,row1,row2,row3,row4) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  UIMenuEntry name ## _3 PROGMEM ={0,0,0,0,0,row3};\
  UIMenuEntry name ## _4 PROGMEM ={0,0,0,0,0,row4};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {3,action,4,name ## _entries};
#define UI_MENU_ACTION2(name,action,row1,row2) UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {3,action,2,name ## _entries};
#define UI_MENU_ACTION2_T(name,action,row1,row2) \
  UIMenuEntry name ## _1 PROGMEM ={0,0,0,0,0,row1};\
  UIMenuEntry name ## _2 PROGMEM ={0,0,0,0,0,row2};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {3,action,2,name ## _entries};
#define UI_MENU_HEADLINE(name,text) UI_STRING(name ## _txt,text);UIMenuEntry name PROGMEM = {name ## _txt,1,0,0,0,0};
#define UI_MENU_HEADLINE_T(name,text) UIMenuEntry name PROGMEM = {0,1,0,0,0,text};
#define UI_MENU_CHANGEACTION(name,row,action) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,4,action,0,0,0};
#define UI_MENU_CHANGEACTION_T(name,row,action) UIMenuEntry name PROGMEM = {0,4,action,0,0,row};
#define UI_MENU_ACTIONCOMMAND(name,row,action) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,3,action,0,0,0};
#define UI_MENU_ACTIONCOMMAND_T(name,rowId,action) UIMenuEntry name PROGMEM = {0,3,action,0,0,rowId};
#define UI_MENU_ACTIONSELECTOR(name,row,entries) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,0,0,0};
#define UI_MENU_ACTIONSELECTOR_T(name,row,entries) UIMenuEntry name PROGMEM = {0,2,(unsigned int)&entries,0,0,row};
#define UI_MENU_SUBMENU(name,row,entries) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,0,0,0};
#define UI_MENU_SUBMENU_T(name,row,entries) UIMenuEntry name PROGMEM = {0,2,(unsigned int)&entries,0,0,row};
#define UI_MENU_WIZARD(name,row,entries) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,5,(unsigned int)&entries,0,0,0};
#define UI_MENU_WIZARD_T(name,row,entries) UIMenuEntry name PROGMEM = {0,5,(unsigned int)&entries,0,0,row};
#define UI_MENU_CHANGEACTION_FILTER(name,row,action,filter,nofilter) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,4,action,filter,nofilter,0};
#define UI_MENU_CHANGEACTION_FILTER_T(name,row,action,filter,nofilter) UIMenuEntry name PROGMEM = {0,4,action,filter,nofilter,row};
#define UI_MENU_ACTIONCOMMAND_FILTER(name,row,action,filter,nofilter) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,3,action,filter,nofilter,0};
#define UI_MENU_ACTIONCOMMAND_FILTER_T(name,row,action,filter,nofilter) UIMenuEntry name PROGMEM = {0,3,action,filter,nofilter,row};
#define UI_MENU_ACTIONSELECTOR_FILTER(name,row,entries,filter,nofilter) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,filter,nofilter,0};
#define UI_MENU_ACTIONSELECTOR_FILTER_T(name,row,entries,filter,nofilter) UIMenuEntry name PROGMEM = {0,2,(unsigned int)&entries,filter,nofilter,row};
#define UI_MENU_SUBMENU_FILTER(name,row,entries,filter,nofilter) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,filter,nofilter,0};
#define UI_MENU_SUBMENU_FILTER_T(name,row,entries,filter,nofilter) UIMenuEntry name PROGMEM = {0,2,(unsigned int)&entries,filter,nofilter,row};
#define UI_MENU(name,items,itemsCnt) const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {2,0,itemsCnt,name ## _entries};
#define UI_STICKYMENU(name,items,itemsCnt) const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {2+128,0,itemsCnt,name ## _entries};
#define UI_MENU_FILESELECT(name,items,itemsCnt) const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {1,0,itemsCnt,name ## _entries};

// Maximum size of a row - if row is larger, text gets scrolled
#if defined(UI_DISPLAY_TYPE) && UI_DISPLAY_TYPE == DISPLAY_GAMEDUINO2
#define MAX_COLS 50
#else
#define MAX_COLS 28
#endif
#define UI_MENU_MAXLEVEL 5

#define UI_FLAG_FAST_KEY_ACTION 1
#define UI_FLAG_SLOW_KEY_ACTION 2
#define UI_FLAG_SLOW_ACTION_RUNNING 4
#define UI_FLAG_KEY_TEST_RUNNING 8

class UIDisplay {
  public:
    volatile uint8_t flags; // 1 = fast key action, 2 = slow key action, 4 = slow action running, 8 = key test running
    uint8_t col; // current col for buffer prefill
    uint8_t menuLevel; // current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change
    uint16_t menuPos[UI_MENU_MAXLEVEL]; // Positions in menu
    const UIMenu *menu[UI_MENU_MAXLEVEL]; // Menus active
    uint16_t menuTop[UI_MENU_MAXLEVEL]; // Top row in menu
    int8_t shift; // Display shift for scrolling text
    int pageDelay; // Counter. If 0 page is refreshed if menuLevel is 0.
    void *errorMsg;
    uint16_t activeAction; // action for ok/next/previous
    uint16_t lastAction;
    uint16_t delayedAction;
    millis_t lastSwitch; // Last time display switched pages
    millis_t lastRefresh;
    uint16_t lastButtonAction;
    millis_t lastButtonStart;
    millis_t nextRepeat; // Time of next autorepeat
    millis_t lastNextPrev; // for increasing speed settings
    float lastNextAccumul; // Accumulated value
    unsigned int outputMask; // Output mask for backlight, leds etc.
    int repeatDuration; // Time beween to actions if autorepeat is enabled
    int8_t oldMenuLevel;
    uint8_t encoderStartScreen;
    char printCols[MAX_COLS + 1];
    void addInt(int value, uint8_t digits, char fillChar = ' '); // Print int into printCols
    void addLong(long value, int8_t digits);
    inline void addLong(long value) {
      addLong(value, -11);
    };
    void addFloat(float number, char fixdigits, uint8_t digits);
    inline void addFloat(float number) {
      addFloat(number, -9, 2);
    };
    void addStringP(PGM_P text);
    void addStringOnOff(uint8_t);
    void addChar(const char c);
    void addGCode(GCode *code);
    int okAction(bool allowMoves);
    bool nextPreviousAction(int16_t next, bool allowMoves);
    char statusMsg[21];
    int8_t encoderPos;
    int8_t encoderLast;
    UIDisplay();
    void createChar(uint8_t location, const uint8_t charmap[]);
    void initialize(); // Initialize display and keys
    void waitForKey();
    void printRow(uint8_t r, char *txt, char *txt2, uint8_t changeAtCol); // Print row on display
    void printRowP(uint8_t r, PGM_P txt);
    void parse(const char *txt, bool ram); /// Parse output and write to printCols;
    void refreshPage();
    int executeAction(unsigned int action, bool allowMoves);
    void finishAction(unsigned int action);
    void slowAction(bool allowMoves);
    void fastAction();
    void mediumAction();
    void pushMenu(const UIMenu *men, bool refresh);
    void popMenu(bool refresh);
    void adjustMenuPos();
    void setStatusP(PGM_P txt, bool error = false);
    void setStatus(const char *txt, bool error = false);
    inline void setOutputMaskBits(unsigned int bits) {
      outputMask |= bits;
    }
    inline void unsetOutputMaskBits(unsigned int bits) {
      outputMask &= ~bits;
    }
    void updateSDFileCount();
    void goDir(char *name);
    bool isDirname(char *name);
    bool isWizardActive();
    bool isSticky();
    void showLanguageSelectionWizard();
#if UI_BED_COATING
    void menuAdjustHeight(const UIMenu *men, float offset);
#endif
    char cwd[SD_MAX_FOLDER_DEPTH * LONG_FILENAME_LENGTH + 2];
    uint8_t folderLevel;
};
extern UIDisplay uid;

// RADDS controler configuration START
#undef SDSS
#define SDSS            10
#undef SPI_PIN
#define SPI_PIN         77
#undef SPI_CHAN
#define SPI_CHAN        0
#define UI_HAS_KEYS 1
#define UI_HAS_BACK_KEY 1
#define UI_DISPLAY_TYPE DISPLAY_4BIT
#define UI_DISPLAY_CHARSET 1
#define BEEPER_TYPE 1
#define UI_COLS 20
#define UI_ROWS 4
#undef BEEPER_PIN
#define BEEPER_PIN             41
#define UI_DISPLAY_RS_PIN      42
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  43
#define UI_DISPLAY_D0_PIN      44
#define UI_DISPLAY_D1_PIN      45
#define UI_DISPLAY_D2_PIN      46
#define UI_DISPLAY_D3_PIN      47
#define UI_DISPLAY_D4_PIN      44
#define UI_DISPLAY_D5_PIN      45
#define UI_DISPLAY_D6_PIN      46
#define UI_DISPLAY_D7_PIN      47
#define UI_ENCODER_A           50
#define UI_ENCODER_B           52
#define UI_ENCODER_CLICK       48
#define UI_RESET_PIN           -1
#define UI_DELAYPERCHAR        50
#define UI_INVERT_MENU_DIRECTION 0
#define UI_BUTTON_BACK         71
#ifdef UI_MAIN
void uiInitKeys() {
  UI_KEYS_INIT_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_INIT_BUTTON_LOW(UI_ENCODER_CLICK); // push button, connects gnd to pin
  UI_KEYS_INIT_BUTTON_LOW(UI_BUTTON_BACK);
}
void uiCheckKeys(uint16_t &action) {
  UI_KEYS_CLICKENCODER_LOW(UI_ENCODER_A, UI_ENCODER_B); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
  UI_KEYS_BUTTON_LOW(UI_ENCODER_CLICK, UI_ACTION_OK); // push button, connects gnd to pin
  UI_KEYS_BUTTON_LOW(UI_BUTTON_BACK, UI_ACTION_BACK);
}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(uint16_t &action) {}
#endif
// RADDS controler configuration END

#ifndef UI_HAS_I2C_ENCODER
#define UI_HAS_I2C_ENCODER 0
#endif

#if UI_ROWS==4
  #if UI_COLS==16
    #define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 4x16
  #elif UI_COLS==20
    #define UI_LINE_OFFSETS {0,0x40,0x14,0x54} // 4x20 with HD44780
  #else // UI_COLS
    #if UI_DISPLAY_TYPE!=DISPLAY_GAMEDUINO2
      #error Unknown combination off rows/columns - define UI_LINE_OFFSETS manually.
    #else // UI_DISPLAY_TYPE
      #define UI_LINE_OFFSETS {} // dummy never used
    #endif // UI_DISPLAY_TYPE
  #endif // UI_COLS
#else // UI_ROWS
  #define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 2x16, 2x20, 2x24
#endif // UI_ROWS
#include "uilang.h"

#define UI_VERSION_STRING "Repetier " REPETIER_VERSION

#ifdef UI_HAS_I2C_KEYS
#define COMPILE_I2C_DRIVER
#endif

#if UI_DISPLAY_TYPE != NO_DISPLAY


#if UI_DISPLAY_TYPE == DISPLAY_I2C
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
#define UI_FAST if((counterPeriodical & 3) == 3) {uid.fastAction();}
#define UI_MEDIUM uid.mediumAction();
#define UI_SLOW(allowMoves) uid.slowAction(allowMoves);
#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_F(status) uid.setStatusP(status);
#define UI_STATUS_UPD(status) {uid.setStatusP(PSTR(status));uid.refreshPage();}
#define UI_STATUS_UPD_F(status) {uid.setStatusP(status);uid.refreshPage();}
#define UI_STATUS_RAM(status) uid.setStatus(status);
#define UI_STATUS_UPD_RAM(status) {uid.setStatus(status);uid.refreshPage();}
#define UI_ERROR(status) uid.setStatusP(PSTR(status),true);
#define UI_ERROR_P(status) uid.setStatusP(status,true);
#define UI_ERROR_UPD(status) {uid.setStatusP(PSTR(status),true);uid.refreshPage();}
#define UI_ERROR_RAM(status) uid.setStatus(status,true);
#define UI_ERROR_UPD_RAM(status) {uid.setStatus(status,true);uid.refreshPage();}
//#define UI_ERROR(msg) {uid.errorMsg=(void*)PSTR(msg);pushMenu((void*)&ui_menu_error,true);}
#define UI_CLEAR_STATUS {uid.statusMsg[0]=0;}
#else
#define UI_INITIALIZE {}
#define UI_FAST {}
#define UI_MEDIUM {}
#define UI_SLOW(allowMoves) {}
#define UI_STATUS(status) {}
#define UI_STATUS_F(status) {}
#define UI_STATUS_RAM(status) {}
#define UI_STATUS_UPD(status) {}
#define UI_STATUS_UPD_F(status) {}
#define UI_STATUS_UPD_RAM(status) {}
#define UI_CLEAR_STATUS {}
#define UI_ERROR(msg) {}
#define UI_ERROR_P(status) {}
#define UI_ERROR_UPD(status) {}
#define UI_ERROR_RAM(status) {}
#define UI_ERROR_UPD_RAM(status) {}
#endif  // Display

// Beeper methods
#if BEEPER_TYPE==0
  #define BEEP_SHORT {}
  #define BEEP_LONG {}
#else
  #define BEEP_SHORT beep(BEEPER_SHORT_SEQUENCE);
  #define BEEP_LONG beep(BEEPER_LONG_SEQUENCE);
#endif


extern void beep(uint8_t duration, uint8_t count);
#if (defined(USER_KEY1_PIN) && USER_KEY1_PIN > -1 && defined(USER_KEY1_ACTION)) || (defined(USER_KEY2_PIN) && USER_KEY2_PIN > -1 && defined(USER_KEY2_ACTION)) || (defined(USER_KEY3_PIN) && USER_KEY3_PIN > -1 && defined(USER_KEY3_ACTION)) || (defined(USER_KEY4_PIN) && USER_KEY4_PIN > -1 && defined(USER_KEY4_ACTION))
#define HAS_USER_KEYS
static void ui_check_Ukeys(uint16_t &action) {
#if defined(USER_KEY1_PIN) && USER_KEY1_PIN > -1 && defined(USER_KEY1_ACTION)
  UI_KEYS_BUTTON_LOW(USER_KEY1_PIN, USER_KEY1_ACTION);
#endif
#if defined(USER_KEY2_PIN) && USER_KEY2_PIN > -1 && defined(USER_KEY2_ACTION)
  UI_KEYS_BUTTON_LOW(USER_KEY2_PIN, USER_KEY2_ACTION);
#endif
#if defined(USER_KEY3_PIN) && USER_KEY3_PIN > -1 && defined(USER_KEY3_ACTION)
  UI_KEYS_BUTTON_LOW(USER_KEY3_PIN, USER_KEY3_ACTION);
#endif
#if defined(USER_KEY4_PIN) && USER_KEY4_PIN > -1 && defined(USER_KEY4_ACTION)
  UI_KEYS_BUTTON_LOW(USER_KEY4_PIN, USER_KEY4_ACTION);
#endif
}
#endif

#endif
