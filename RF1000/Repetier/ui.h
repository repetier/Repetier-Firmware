/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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


#ifndef UI_H
#define UI_H

#include "gcode.h"

// ----------------------------------------------------------------------------
//                          Action codes
// 1-999     : Autorepeat
// 1000-1999 : Execute
// 2000-2999 : Write code
// 4000-4999 : Show menu
// Add UI_ACTION_TOPMENU to show a menu as top menu
// ----------------------------------------------------------------------------

#define UI_ACTION_TOPMENU					8192

#define UI_ACTION_NEXT						   1
#define UI_ACTION_PREVIOUS					   2

#define UI_ACTION_X_UP						 100
#define UI_ACTION_X_DOWN					 101
#define UI_ACTION_Y_UP						 102
#define UI_ACTION_Y_DOWN					 103
#define UI_ACTION_Z_UP						 104
#define UI_ACTION_Z_DOWN					 105
#define UI_ACTION_EXTRUDER_UP				 106
#define UI_ACTION_EXTRUDER_DOWN				 107
#define UI_ACTION_EXTRUDER_TEMP_UP			 108
#define UI_ACTION_EXTRUDER_TEMP_DOWN		 109
#define UI_ACTION_HEATED_BED_UP				 110
#define UI_ACTION_HEATED_BED_DOWN			 111
#define UI_ACTION_FAN_UP					 112
#define UI_ACTION_FAN_DOWN					 113

#define UI_ACTION_DUMMY					   10000
#define UI_ACTION_BACK						1000
#define UI_ACTION_OK						1001
#define UI_ACTION_MENU_UP					1002
#define UI_ACTION_TOP_MENU					1003
#define UI_ACTION_EMERGENCY_STOP			1004
#define UI_ACTION_XPOSITION					1005
#define UI_ACTION_YPOSITION					1006
#define UI_ACTION_ZPOSITION					1007
#define UI_ACTION_EPOSITION					1008
#define UI_ACTION_BED_TEMP					1009
#define UI_ACTION_EXTRUDER_TEMP				1010
#define UI_ACTION_SD_DELETE					1012
#define UI_ACTION_SD_PRINT					1013
#define UI_ACTION_SD_PAUSE					1014
#define UI_ACTION_SD_CONTINUE				1015
#define UI_ACTION_SD_UNMOUNT				1016
#define UI_ACTION_SD_MOUNT					1017
#define UI_ACTION_XPOSITION_FAST			1018
#define UI_ACTION_YPOSITION_FAST			1019
#define UI_ACTION_ZPOSITION_FAST			1020
#define UI_ACTION_HOME_ALL					1021
#define UI_ACTION_HOME_X					1022
#define UI_ACTION_HOME_Y					1023
#define UI_ACTION_HOME_Z					1024
#define UI_ACTION_SELECT_EXTRUDER1			1025
#define UI_ACTION_PRINT_ACCEL_X				1032
#define UI_ACTION_PRINT_ACCEL_Y				1033
#define UI_ACTION_PRINT_ACCEL_Z				1034
#define UI_ACTION_MOVE_ACCEL_X				1035
#define UI_ACTION_MOVE_ACCEL_Y				1036
#define UI_ACTION_MOVE_ACCEL_Z				1037
#define UI_ACTION_MAX_JERK					1038
#define UI_ACTION_MAX_ZJERK					1039
#define UI_ACTION_BAUDRATE					1040
#define UI_ACTION_HOMING_FEEDRATE_X			1041
#define UI_ACTION_HOMING_FEEDRATE_Y			1042
#define UI_ACTION_HOMING_FEEDRATE_Z			1043
#define UI_ACTION_MAX_FEEDRATE_X			1044
#define UI_ACTION_MAX_FEEDRATE_Y			1045
#define UI_ACTION_MAX_FEEDRATE_Z			1046
#define UI_ACTION_STEPS_X					1047
#define UI_ACTION_STEPS_Y					1048
#define UI_ACTION_STEPS_Z					1049
#define UI_ACTION_FAN_OFF					1050
#define UI_ACTION_FAN_25					1051
#define UI_ACTION_FAN_50					1052
#define UI_ACTION_FAN_75					1053
#define UI_ACTION_FAN_FULL					1054
#define UI_ACTION_FEEDRATE_MULTIPLY			1055
#define UI_ACTION_STEPPER_INACTIVE			1056
#define UI_ACTION_PID_PGAIN					1058
#define UI_ACTION_PID_IGAIN					1059
#define UI_ACTION_PID_DGAIN					1060
#define UI_ACTION_DRIVE_MIN					1061
#define UI_ACTION_DRIVE_MAX					1062
#define UI_ACTION_X_OFFSET					1063
#define UI_ACTION_Y_OFFSET					1064
#define UI_ACTION_EXTR_STEPS				1065
#define UI_ACTION_EXTR_ACCELERATION			1066
#define UI_ACTION_EXTR_MAX_FEEDRATE			1067
#define UI_ACTION_EXTR_START_FEEDRATE		1068
#define UI_ACTION_EXTR_HEATMANAGER			1069
#define UI_ACTION_EXTR_WATCH_PERIOD			1070
#define UI_ACTION_PID_MAX					1071
#define UI_ACTION_ADVANCE_K					1072
#define UI_ACTION_DEBUG_ECHO				1074
#define UI_ACTION_DEBUG_INFO				1075
#define UI_ACTION_DEBUG_ERROR				1076
#define UI_ACTION_DEBUG_DRYRUN				1077
#define UI_ACTION_POWER						1078
#define UI_ACTION_PREHEAT_PLA				1079
#define UI_ACTION_COOLDOWN					1080
#define UI_ACTION_HEATED_BED_OFF			1081
#define UI_ACTION_EXTRUDER0_OFF				1082
#define UI_ACTION_EXTRUDER1_OFF				1083
#define UI_ACTION_HEATED_BED_TEMP			1084
#define UI_ACTION_EXTRUDER0_TEMP			1085
#define UI_ACTION_EXTRUDER1_TEMP			1086
#define UI_ACTION_OPS_OFF					1087
#define UI_ACTION_OPS_CLASSIC				1088
#define UI_ACTION_OPS_FAST					1089
#define UI_ACTION_DISABLE_STEPPER			1090
#define UI_ACTION_SET_E_ORIGIN				1091
#define UI_ACTION_EXTRUDER_RELATIVE			1092
#define UI_ACTION_SELECT_EXTRUDER0			1093
#define UI_ACTION_ADVANCE_L					1094
#define UI_ACTION_PREHEAT_ABS				1095
#define UI_ACTION_FLOWRATE_MULTIPLY			1096
#define UI_ACTION_KILL						1097
#define UI_ACTION_RESET						1098
#define UI_ACTION_PAUSE						1099
#define UI_ACTION_EXTR_WAIT_RETRACT_TEMP	1100
#define UI_ACTION_EXTR_WAIT_RETRACT_UNITS	1101
#define UI_ACTION_EXTRUDER2_OFF				1102
#define UI_ACTION_EXTRUDER2_TEMP			1103
#define UI_ACTION_SELECT_EXTRUDER2			1104
#define UI_ACTION_WRITE_DEBUG				1105
#define UI_ACTION_FANSPEED					1106
#define UI_ACTION_LIGHTS_ONOFF				1107
#define UI_ACTION_SD_STOP					1108
#define UI_ACTION_SD_STOP_ACK				1109
#define UI_ACTION_ZPOSITION_NOTEST			1110
#define UI_ACTION_ZPOSITION_FAST_NOTEST		1111
#define UI_ACTION_MAX_INACTIVE				1113
#define UI_ACTION_BEEPER					1114
#define UI_ACTION_UNMOUNT_FILAMENT			1115
#define UI_ACTION_MOUNT_FILAMENT			1116
#define UI_ACTION_OPERATING_MODE			1117
#define UI_ACTION_SET_XY_ORIGIN				1118
#define UI_ACTION_Z_ENDSTOP_TYPE			1119
#define UI_ACTION_HOTEND_TYPE				1120
#define UI_ACTION_MILLER_TYPE				1121
#define	UI_ACTION_230V_OUTPUT				1122
#define UI_ACTION_RGB_LIGHT_MODE			1123
#define	UI_ACTION_EXTRUDER_OFFSET_X			1124
#define UI_ACTION_EXTRUDER_OFFSET_Y			1125
#define	UI_ACTION_RESTORE_DEFAULTS			1126
#define	UI_ACTION_ACTIVE_EXTRUDER			1127
#define UI_ACTION_ZOFFSET					1128
#define UI_ACTION_RIGHT						1129

#define UI_ACTION_MENU_XPOS					4000
#define UI_ACTION_MENU_YPOS					4001
#define UI_ACTION_MENU_ZPOS					4002
#define UI_ACTION_MENU_XPOSFAST				4003
#define UI_ACTION_MENU_YPOSFAST				4004
#define UI_ACTION_MENU_ZPOSFAST				4005
#define UI_ACTION_MENU_SDCARD				4006
#define UI_ACTION_MENU_QUICKSETTINGS		4007
#define UI_ACTION_MENU_EXTRUDER				4008
#define UI_ACTION_MENU_POSITIONS			4009
#define UI_ACTION_SET_Z_ORIGIN				4012
#define UI_ACTION_SET_P1					4013
#define UI_ACTION_SET_P2					4014
#define UI_ACTION_SET_P3					4015
#define UI_ACTION_CALC_LEVEL				4016

#define UI_ACTION_SHOW_USERMENU1			4101
#define UI_ACTION_SHOW_USERMENU2			4102
#define UI_ACTION_SHOW_USERMENU3			4103
#define UI_ACTION_SHOW_USERMENU4			4104
#define UI_ACTION_SHOW_USERMENU5			4105
#define UI_ACTION_SHOW_USERMENU6			4106
#define UI_ACTION_SHOW_USERMENU7			4107
#define UI_ACTION_SHOW_USERMENU8			4108
#define UI_ACTION_SHOW_USERMENU9			4109
#define UI_ACTION_SHOW_USERMENU10			4110

// Load basic language definition to make sure all values are defined
#include "uilang.h"


typedef struct
{
	const char*		text;		// Menu text
	uint8_t			menuType;	// 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action command
	unsigned int	action;		// must be int so it gets 32 bit on arm!
	uint8_t			filter;		// allows dynamic menu filtering based on Printer::menuMode bits set.
	uint8_t			nofilter;	// Hide if one of these bits are set

	bool showEntry() const;

}const UIMenuEntry;


typedef struct
{
	// 0 = info page
	// 1 = file selector
	// 2 = submenu
	// 3 = modification menu
	unsigned char	menuType;
	int				id;			// Type of modification
	int				numEntries;

	const UIMenuEntry * const * entries;

}const UIMenu;


extern const int8_t	encoder_table[16] PROGMEM;

extern	char	g_nYesNo;
extern	char	g_nContinueButtonPressed;
extern	char	g_nServiceRequest;


// Key codes
#define UI_KEYS_INIT_CLICKENCODER_LOW(pinA,pinB)		SET_INPUT(pinA);SET_INPUT(pinB); PULLUP(pinA,HIGH);PULLUP(pinB,HIGH);
#define UI_KEYS_INIT_BUTTON_LOW(pin)					SET_INPUT(pin);PULLUP(pin,HIGH);
#define UI_KEYS_INIT_CLICKENCODER_HIGH(pinA,pinB)		SET_INPUT(pinA);SET_INPUT(pinB); PULLUP(pinA,LOW);PULLUP(pinB,LOW);
#define UI_KEYS_INIT_BUTTON_HIGH(pin)					SET_INPUT(pin);PULLUP(pin,LOW);

#define UI_KEYS_CLICKENCODER_LOW(pinA,pinB)				uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!READ(pinA)) uid.encoderLast |=2;if (!READ(pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_CLICKENCODER_LOW_REV(pinA,pinB)			uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!READ(pinA)) uid.encoderLast |=2;if (!READ(pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_BUTTON_LOW(pin,action_)					if(READ(pin)==0) action=action_;
#define UI_KEYS_CLICKENCODER_HIGH(pinA,pinB)			uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (READ(pinA)) uid.encoderLast |=2;if (READ(pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_CLICKENCODER_HIGH_REV(pinA,pinB)		uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (READ(pinA)) uid.encoderLast |=2;if (READ(pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_BUTTON_HIGH(pin,action_)				if(READ(pin)!=0) action=action_;
#define UI_KEYS_INIT_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4)	if(c1>=0){SET_INPUT(c1);WRITE(c1,HIGH);}if(c2>=0){SET_INPUT(c2);WRITE(c2,HIGH);}if(c3>=0){SET_INPUT(c3);WRITE(c3,HIGH);}\
    if(c4>=0) {SET_INPUT(c4);WRITE(c4,HIGH);}if(r1>=0)SET_OUTPUT(r1);if(r2>=0)SET_OUTPUT(r2);if(r3>=0)SET_OUTPUT(r3);if(r4>=0)SET_OUTPUT(r4);\
    if(r1>=0)WRITE(r1,LOW);if(r2>=0)WRITE(r2,LOW);if(r3>=0)WRITE(r3,LOW);if(r4>=0)WRITE(r4,LOW);

#define UI_KEYS_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4)			{uint8_t r = (c1>=0?READ(c1):0) && (c2>=0?READ(c2):0) && (c3>=0?READ(c3):0) && (c4>=0?READ(c4):0);\
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
#define UI_KEYS_I2C_CLICKENCODER_LOW(pinA,pinB)			uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!(keymask & pinA)) uid.encoderLast |=2;if (!(keymask & pinB)) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_CLICKENCODER_LOW_REV(pinA,pinB)		uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (!(keymask & pinA)) uid.encoderLast |=2;if (!(keymask & pinB)) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_BUTTON_LOW(pin,action_)				if((keymask & pin)==0) action=action_;
#define UI_KEYS_I2C_CLICKENCODER_HIGH(pinA,pinB)		uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (keymask & pinA) uid.encoderLast |=2;if (keymask & pinB) uid.encoderLast |=1; uid.encoderPos += pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_CLICKENCODER_HIGH_REV(pinA,pinB)	uid.encoderLast = (uid.encoderLast << 2) & 0x0F;if (keymask & pinA) uid.encoderLast |=2;if (keymask & pinB) uid.encoderLast |=1; uid.encoderPos -= pgm_read_byte(&encoder_table[uid.encoderLast]);
#define UI_KEYS_I2C_BUTTON_HIGH(pin,action_)			if((pin & keymask)!=0) action=action_;

#define UI_STRING(name,text) const char PROGMEM name[] = text;

#define UI_PAGE6(name,row1,row2,row3,row4,row5,row6)	UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);UI_STRING(name ## _5txt,row5);UI_STRING(name ## _6txt,row6);\
   UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};\
   UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};\
   UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0};\
   UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0};\
   UIMenuEntry name ## _5 PROGMEM ={name ## _5txt,0,0,0,0};\
   UIMenuEntry name ## _6 PROGMEM ={name ## _6txt,0,0,0,0};\
   const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4,&name ## _5,&name ## _6};\
   const UIMenu name PROGMEM = {0,0,6,name ## _entries};

#define UI_PAGE4(name,row1,row2,row3,row4)				UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0};\
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {0,0,4,name ## _entries};

#define UI_PAGE2(name,row1,row2)						UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {0,0,2,name ## _entries};

#define UI_MENU_ACTION4C(name,action,rows)				UI_MENU_ACTION4(name,action,rows)
#define UI_MENU_ACTION2C(name,action,rows)				UI_MENU_ACTION2(name,action,rows)

#define UI_MENU_ACTION4(name,action,row1,row2,row3,row4)	UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);UI_STRING(name ## _3txt,row3);UI_STRING(name ## _4txt,row4);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};\
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0};\
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4};\
  const UIMenu name PROGMEM = {3,action,4,name ## _entries};

#define UI_MENU_ACTION2(name,action,row1,row2)				UI_STRING(name ## _1txt,row1);UI_STRING(name ## _2txt,row2);\
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};\
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};\
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2};\
  const UIMenu name PROGMEM = {3,action,2,name ## _entries};

#define UI_MENU_HEADLINE(name,text)						UI_STRING(name ## _txt,text);UIMenuEntry name PROGMEM = {name ## _txt,1,0,0,0};
#define UI_MENU_CHANGEACTION(name,row,action)			UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,4,action,0,0};
#define UI_MENU_ACTIONCOMMAND(name,row,action)			UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,3,action,0,0};
#define UI_MENU_ACTIONSELECTOR(name,row,entries)		UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,0,0};
#define UI_MENU_SUBMENU(name,row,entries)				UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,0,0};
#define UI_MENU_CHANGEACTION_FILTER(name,row,action,filter,nofilter)	UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,4,action,filter,nofilter};
#define UI_MENU_ACTIONCOMMAND_FILTER(name,row,action,filter,nofilter)	UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,3,action,filter,nofilter};
#define UI_MENU_ACTIONSELECTOR_FILTER(name,row,entries,filter,nofilter) UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,filter,nofilter};
#define UI_MENU_SUBMENU_FILTER(name,row,entries,filter,nofilter)		UI_STRING(name ## _txt,row);UIMenuEntry name PROGMEM = {name ## _txt,2,(unsigned int)&entries,filter,nofilter};
#define UI_MENU(name,items,itemsCnt)									const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {2,0,itemsCnt,name ## _entries}
#define UI_MENU_FILESELECT(name,items,itemsCnt)							const UIMenuEntry * const name ## _entries[] PROGMEM = items;const UIMenu name PROGMEM = {1,0,itemsCnt,name ## _entries}

// Maximum size of a row - if row is larger, text gets scrolled
#define MAX_COLS						28

#define	UI_FLAG_FAST_KEY_ACTION			1
#define UI_FLAG_SLOW_KEY_ACTION			2
#define UI_FLAG_SLOW_ACTION_RUNNING		4
#define	UI_FLAG_KEY_TEST_RUNNING		8


class UIDisplay
{
public:
    volatile uint8_t	flags;
    uint8_t				col;					// current col for buffer prefill
    uint8_t				menuLevel;				// current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change
    uint8_t				menuPos[5];				// Positions in menu
    void*				menu[5];				// Menus active
    uint8_t				menuTop[5];				// Top row in menu
    int8_t				shift;					// Display shift for scrolling text
    int					pageDelay;				// Counter. If 0 page is refreshed if menuLevel is 0.
    void*				errorMsg;
    uint16_t			activeAction;			// action for ok/next/previous
    uint16_t			lastAction;
    millis_t			lastSwitch;				// Last time display switched pages
    millis_t			lastRefresh;
    uint16_t			lastButtonAction;
    millis_t			lastButtonStart;
    millis_t			nextRepeat;				// Time of next autorepeat
    millis_t			lastNextPrev;			// for increasing speed settings
    float				lastNextAccumul;		// Accumulated value
    unsigned int		outputMask;				// Output mask for backlight, leds etc.
    int					repeatDuration;			// Time beween to actions if autorepeat is enabled
    int8_t				oldMenuLevel;
    uint8_t				encoderStartScreen;
    char				statusMsg[21];
    int8_t				encoderPos;
    int8_t				encoderLast;
    PGM_P				statusText;
	char				locked;

    void addInt(int value,uint8_t digits,char fillChar=' '); // Print int into printCols
    void addLong(long value,char digits);
    void addFloat(float number, char fixdigits,uint8_t digits);
    void addStringP(PGM_P text);
    void okAction();
    void nextPreviousAction(int8_t next);
    UIDisplay();
    void createChar(uint8_t location,const uint8_t charmap[]);
    void initialize(); // Initialize display and keys
    void waitForKey();
    void printRow(uint8_t r,char *txt,char *txt2,uint8_t changeAtCol); // Print row on display
    void printRowP(uint8_t r,PGM_P txt);
    void parse(char *txt,bool ram); /// Parse output and write to printCols;
    void refreshPage();
    void executeAction(int action);
    void finishAction(int action);
    void slowAction();
    void fastAction();
    void mediumAction();
    void pushMenu(void *men,bool refresh);
    void adjustMenuPos();
    void setStatusP(PGM_P txt,bool error = false);
    void setStatus(char *txt,bool error = false);

    inline void setOutputMaskBits(unsigned int bits)
	{
		outputMask|=bits;
	} // setOutputMaskBits

    inline void unsetOutputMaskBits(unsigned int bits)
	{
		outputMask&=~bits;
	} // unsetOutputMaskBits

#if SDSUPPORT
	char				cwd[SD_MAX_FOLDER_DEPTH*LONG_FILENAME_LENGTH+2];
    uint8_t				folderLevel;

	void updateSDFileCount();
    void goDir(char *name);
    bool isDirname(char *name);
#endif // SDSUPPORT

	void lock();
	void unlock();

};

extern UIDisplay uid;

// initializeLCD()
void initializeLCD();


#if MOTHERBOARD == DEVICE_TYPE_RF1000
#define UI_HAS_KEYS						  1		// 1 = Some keys attached
#define UI_HAS_BACK_KEY					  1
#define UI_DISPLAY_TYPE					  1		// 1 = LCD Display with 4 bit data bus
#define UI_DISPLAY_CHARSET				  1
#define UI_COLS							 16
#define UI_ROWS							  4
#define UI_DELAYPERCHAR					320
#define UI_INVERT_MENU_DIRECTION		false
#define UI_INVERT_INCREMENT_DIRECTION	true

#ifdef UI_MAIN
void ui_init_keys() 
{
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_1);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_2);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_3);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_4);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_5);	// push button, connects gnd to pin

#if FEATURE_EXTENDED_BUTTONS
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E1);	// PINJ.2, 80, X12.1 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E2);	// PINJ.4, 81, X12.2 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E3);	// PINJ.5, 82, X12.3 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E4);	// PINJ.6, 83, X12.4 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E5);	// PINH.7, 85, X12.6 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E6);	// PINH.2, 86, X12.7 - push button, connects gnd to pin
#endif // FEATURE_EXTENDED_BUTTONS

} // ui_init_keys


void ui_check_keys(int &action) 
{
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_1,UI_ACTION_OK);			// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_2,UI_ACTION_NEXT);		// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_5,UI_ACTION_PREVIOUS);	// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_4,UI_ACTION_BACK);		// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_3,UI_ACTION_RIGHT );		// push button, connects gnd to pin

#if FEATURE_EXTENDED_BUTTONS
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E1,UI_ACTION_RF_HEAT_BED_UP);			// PINJ.2, 80, X12.1 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E2,UI_ACTION_RF_HEAT_BED_DOWN);		// PINJ.4, 81, X12.2 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E3,UI_ACTION_RF_EXTRUDER_RETRACT);	// PINJ.5, 82, X12.3 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E4,UI_ACTION_RF_EXTRUDER_OUTPUT);		// PINJ.6, 83, X12.4 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E5,UI_ACTION_RF_CONTINUE);			// PINH.7, 85, X12.6 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E6,UI_ACTION_RF_PAUSE);				// PINH.2, 86, X12.7 - push button, connects gnd to pin
#endif // FEATURE_EXTENDED_BUTTONS

} // ui_check_keys

inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif // UI_MAIN
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000


#if MOTHERBOARD == DEVICE_TYPE_RF2000
#define UI_HAS_KEYS						  1		// 1 = Some keys attached
#define UI_HAS_BACK_KEY					  1
#define UI_DISPLAY_TYPE					  1		// 1 = LCD Display with 4 bit data bus
#define UI_DISPLAY_CHARSET				  1
#define UI_COLS							 20
#define UI_ROWS							  4
#define UI_DELAYPERCHAR					320
#define UI_INVERT_MENU_DIRECTION		false
#define UI_INVERT_INCREMENT_DIRECTION	true

#ifdef UI_MAIN
void ui_init_keys() 
{
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_1);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_2);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_3);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_4);	// push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_5);	// push button, connects gnd to pin

#if FEATURE_EXTENDED_BUTTONS
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E1);	// PINJ.2, 80, X12.1 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E2);	// PINJ.4, 81, X12.2 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E3);	// PINJ.5, 82, X12.3 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E4);	// PINJ.6, 83, X12.4 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E5);	// PINH.7, 85, X12.6 - push button, connects gnd to pin
	UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E6);	// PINH.2, 86, X12.7 - push button, connects gnd to pin
#endif // FEATURE_EXTENDED_BUTTONS

} // ui_init_keys


void ui_check_keys(int &action) 
{
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_1,UI_ACTION_OK);			// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_2,UI_ACTION_NEXT);		// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_5,UI_ACTION_PREVIOUS);	// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_4,UI_ACTION_BACK);		// push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_3,UI_ACTION_RIGHT );		// push button, connects gnd to pin

#if FEATURE_EXTENDED_BUTTONS
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E1,UI_ACTION_RF_HEAT_BED_UP);			// PINJ.2, 80, X12.1 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E2,UI_ACTION_RF_HEAT_BED_DOWN);		// PINJ.4, 81, X12.2 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E3,UI_ACTION_RF_EXTRUDER_RETRACT);	// PINJ.5, 82, X12.3 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E4,UI_ACTION_RF_EXTRUDER_OUTPUT);		// PINJ.6, 83, X12.4 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E5,UI_ACTION_RF_CONTINUE);			// PINH.7, 85, X12.6 - push button, connects gnd to pin
	UI_KEYS_BUTTON_LOW(ENABLE_KEY_E6,UI_ACTION_RF_PAUSE);				// PINH.2, 86, X12.7 - push button, connects gnd to pin
#endif // FEATURE_EXTENDED_BUTTONS

} // ui_check_keys

inline void ui_check_slow_encoder() {}
void ui_check_slow_keys(int &action) {}
#endif // UI_MAIN
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000


#if UI_ROWS==4
#if UI_COLS==16
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 4x16
#elif UI_COLS==20
//#define UI_LINE_OFFSETS {0,0x20,0x40,0x60} // 4x20 with KS0073
#define UI_LINE_OFFSETS {0,0x40,0x14,0x54} // 4x20 with HD44780
#else
#error Unknown combination of rows/columns - define UI_LINE_OFFSETS manually.
#endif // UI_COLS==16
#else
#define UI_LINE_OFFSETS {0,0x40,0x10,0x50} // 2x16, 2x20, 2x24
#endif // UI_ROWS==4

#include "uilang.h"
#include "uimenu.h"

#ifdef UI_HAS_I2C_KEYS
#define COMPILE_I2C_DRIVER
#endif // UI_HAS_I2C_KEYS

#if UI_DISPLAY_TYPE!=0
#if UI_DISPLAY_TYPE==3
#define COMPILE_I2C_DRIVER
#endif // UI_DISPLAY_TYPE==3

#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 1
#else
#define UI_TEMP_PRECISION 0
#endif // UI_COLS>16
#endif // UI_TEMP_PRECISION

#define UI_INITIALIZE uid.initialize();
#define UI_FAST if(pwm_count & 4) {uid.fastAction();}
#define UI_MEDIUM uid.mediumAction();
#define UI_SLOW uid.slowAction();
#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_UPD(status) {uid.setStatusP(PSTR(status));uid.refreshPage();}
#define UI_STATUS_RAM(status) uid.setStatus(status);
#define UI_STATUS_UPD_RAM(status) {uid.setStatus(status);uid.refreshPage();}
#define UI_ERROR(status) uid.setStatusP(PSTR(status),true);
#define UI_ERROR_UPD(status) {uid.setStatusP(PSTR(status),true);uid.refreshPage();}
#define UI_ERROR_RAM(status) uid.setStatus(status,true);
#define UI_ERROR_UPD_RAM(status) {uid.setStatus(status,true);uid.refreshPage();}
#define UI_CLEAR_STATUS {uid.statusMsg[0]=0;}
#else
#define UI_INITIALIZE {}
#define UI_FAST {}
#define UI_MEDIUM {}
#define UI_SLOW {}
#define UI_STATUS(status) {}
#define UI_STATUS_RAM(status) {}
#define UI_STATUS_UPD(status) {}
#define UI_STATUS_UPD_RAM(status) {}
#define UI_CLEAR_STATUS {}
#define UI_ERROR(msg) {}
#define UI_ERROR_UPD(status) {}
#define UI_ERROR_RAM(status) {}
#define UI_ERROR_UPD_RAM(status) {}
#endif  // UI_DISPLAY_TYPE!=0

// Beeper methods
#if BEEPER_TYPE==0
#define BEEP_SHORT {}
#define BEEP_LONG {}
#define BEEP_START_PRINTING {}
#define BEEP_ABORT_PRINTING {}
#define BEEP_STOP_PRINTING {}
#define BEEP_PAUSE {}
#define BEEP_CONTINUE {}
#define BEEP_START_HEAT_BED_SCAN {}
#define BEEP_ABORT_HEAT_BED_SCAN {}
#define BEEP_STOP_HEAT_BED_SCAN {}
#define BEEP_START_WORK_PART_SCAN {}
#define BEEP_ABORT_WORK_PART_SCAN {}
#define BEEP_STOP_WORK_PART_SCAN {}
#define BEEP_ABORT_SET_POSITION {}
#define BEEP_ACCEPT_SET_POSITION {}
#define	BEEP_SERVICE_INTERVALL {}
#define	BEEP_ALIGN_EXTRUDERS {}
#define BEEP_WRONG_FIRMWARE {}
#else
#define BEEP_SHORT beep(BEEPER_SHORT_SEQUENCE);
#define BEEP_LONG beep(BEEPER_LONG_SEQUENCE);
#define BEEP_START_PRINTING beep(BEEPER_START_PRINTING_SEQUENCE);
#define BEEP_ABORT_PRINTING beep(BEEPER_ABORT_PRINTING_SEQUENCE);
#define BEEP_STOP_PRINTING beep(BEEPER_STOP_PRINTING_SEQUENCE);
#define BEEP_PAUSE beep(BEEPER_PAUSE_SEQUENCE);
#define BEEP_CONTINUE beep(BEEPER_CONTINUE_SEQUENCE);
#define BEEP_START_HEAT_BED_SCAN beep(BEEPER_START_HEAT_BED_SCAN_SEQUENCE);
#define BEEP_ABORT_HEAT_BED_SCAN beep(BEEPER_ABORT_HEAT_BED_SCAN_SEQUENCE);
#define BEEP_STOP_HEAT_BED_SCAN beep(BEEPER_STOP_HEAT_BED_SCAN_SEQUENCE);
#define BEEP_START_WORK_PART_SCAN beep(BEEPER_START_WORK_PART_SCAN_SEQUENCE);
#define BEEP_ABORT_WORK_PART_SCAN beep(BEEPER_ABORT_WORK_PART_SCAN_SEQUENCE);
#define BEEP_STOP_WORK_PART_SCAN beep(BEEPER_STOP_WORK_PART_SCAN_SEQUENCE);
#define BEEP_ABORT_SET_POSITION beep(BEEPER_ABORT_SET_POSITION_SEQUENCE);
#define BEEP_ACCEPT_SET_POSITION beep(BEEPER_ACCEPT_SET_POSITION_SEQUENCE);
#define	BEEP_SERVICE_INTERVALL beep(BEEPER_SERVICE_INTERVALL_SEQUNCE);
#define	BEEP_ALIGN_EXTRUDERS beep(BEEPER_ALIGN_EXTRUDERS_SEQUNCE);
#define	BEEP_WRONG_FIRMWARE beep(BEEPER_WRONG_FIRMWARE_SEQUNCE);
#endif // BEEPER_TYPE==0

extern void beep(uint8_t duration,uint8_t count);

#endif // UI_H

