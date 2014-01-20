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

/* ===================== IMPORTANT ========================

The LCD and Key support is new. I tested everything as good as possible,
but some combinations may not work as supposed.
The I2C methods rely on a stable I2C connection. Noise may cause wrong signals
which can cause the firmware to freeze.

The ui adds quite some code, so AVRs with 64kB ram (Sanguino, Gen6) can not handle all features
of the firmware at the same time. You have to disable some features to gain the
ram needed. What should work:
- No sd card - the sd card code is quite large.
- No keys attached - The longest part is the menu handling.
- EEPROM_MODE 0 and USE_OPS 0. 

Currently supported hardware:

*** Displays ***

- direct connected lcd with 4 data lines
- connected via i2c

*** Keys ***

- rotary encoder
- push button
- key matrix up to 4x4
- rotary encoder via i2c (only slow turns are captured correct)
- push button via i2c

*** Buzzer ***

- directly connected, high = on
- connected via i2c, low = on

==============================================================*/

/**
Select the language to use.
0 = english
1 = german
*/
#define UI_LANGUAGE 1
#include "uilang.h"

/** Select type of beeper
0 = none
1 = Piezo connected to pin
2 = Piezo connected to a pin over I2C 
*/
#define BEEPER_TYPE 0

#if BEEPER_TYPE==1
#define BEEPER_PIN 42
#endif
#if BEEPER_TYPE==2
#define BEEPER_ADDRESS 0x40 // I2C address of the chip with the beeper pin
#define BEEPER_PIN _BV(7)  // Bit value for pin 8
#define COMPILE_I2C_DRIVER  // We need the I2C driver as we are using i2c
#endif

/**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 1,1
#define BEEPER_LONG_SEQUENCE 5,5

/**
What display type do you use?
0 = No display
1 = LCD Display with 4 bit data bus
2 = LCD Display with 8 bit data bus (currently not implemented, fallback to 1)
3 = LCD Display with I2C connection, 4 bit mode
4 = Use the slower LiquiedCrystal library bundled with arduino. 
    IMPORTANT: You need to uncomment the LiquidCrystal include in Repetier.pde for it to work.
               If you have Sanguino and want to use the library, you need to have Arduino 023 or older. (13.04.2012)
*/
#define UI_DISPLAY_TYPE 3 

// This is line 2 of the status display at startup
#define UI_VERSION_STRING2 "Orig. Mendel"

/** Number of columns per row

Typical values are 16 and 20
*/
#define UI_COLS 16
/**
Rows of your display. 2 or 4
*/
#define UI_ROWS 2

/* What type of chip is used for I2C communication
0 : PCF8574 or PCF8574A or compatible chips.
1 : MCP23017
*/
#define UI_DISPLAY_I2C_CHIPTYPE 1
// 0x40 till 0x4e for PCF8574, 0x40 for the adafruid RGB shield, 0x40 - 0x4e for MCP23017
#define UI_DISPLAY_I2C_ADDRESS 0x40
// For MCP 23017 define which pins should be output
#define UI_DISPLAY_I2C_OUTPUT_PINS 65504
// Set the output mask that is or'd over the output data. This is needed to activate
// a backlight switched over the I2C. 
// The adafruit RGB shields enables a light if the bit is not set. Bits 6-8 are used for backlight.
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0
// For MCP which inputs are with pullup. 31 = pins 0-4 for adafruid rgb shield buttons
#define UI_DISPLAY_I2C_PULLUP 31
/* How fast should the I2C clock go. The PCF8574 work only with the lowest setting 100000.
A MCP23017 can run also with 400000 Hz */
#define UI_I2C_CLOCKSPEED 400000L
/**
Define the pin
*/
#if UI_DISPLAY_TYPE==3 // I2C Pin configuration
/*#define UI_DISPLAY_RS_PIN _BV(4)
#define UI_DISPLAY_RW_PIN _BV(5)
#define UI_DISPLAY_ENABLE_PIN _BV(6)
#define UI_DISPLAY_D0_PIN _BV(0)
#define UI_DISPLAY_D1_PIN _BV(1)
#define UI_DISPLAY_D2_PIN _BV(2)
#define UI_DISPLAY_D3_PIN _BV(3)
#define UI_DISPLAY_D4_PIN _BV(0)
#define UI_DISPLAY_D5_PIN _BV(1)
#define UI_DISPLAY_D6_PIN _BV(2)
#define UI_DISPLAY_D7_PIN _BV(3)*/

// Pins for adafruid RGB shield
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

#else // Direct display connections
#define UI_DISPLAY_RS_PIN 16
#define UI_DISPLAY_RW_PIN 17
#define UI_DISPLAY_ENABLE_PIN 31
#define UI_DISPLAY_D0_PIN 23
#define UI_DISPLAY_D1_PIN 29
#define UI_DISPLAY_D2_PIN 25
#define UI_DISPLAY_D3_PIN 27
#define UI_DISPLAY_D4_PIN 23
#define UI_DISPLAY_D5_PIN 29
#define UI_DISPLAY_D6_PIN 25
#define UI_DISPLAY_D7_PIN 27
#define UI_DELAYPERCHAR 320
#endif

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

/** \brief Are some keys connected?

0 = No keys attached - disables also menu
1 = Some keys attached
*/
#define UI_HAS_KEYS 1

/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME 10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT 500
/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT 50
/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT 50

/** \brief Is a back key present.

If you have menus enabled, you need a method to leave it. If you have a back key, you can always go one level higher.
Without a back key, you need to navigate to the back entry in the menu. Setting this value to 1 removes the back entry.
*/
#define UI_HAS_BACK_KEY 1

/* Then you have the next/previous keys more like up/down keys, it may be more intuitive to change the direction you skip through the menus.
If you set it to true, next will go to previous menu instead of the next menu.

*/
#define UI_INVERT_MENU_DIRECTION true

/** Uncomment this, if you have keys connected via i2c to a PCF8574 chip. */
#define UI_HAS_I2C_KEYS

// Do you have a I2C connected encoder? 
#define UI_HAS_I2C_ENCODER 0

// Under which address can the key status requested. This is the address of your PCF8574 where the keys are connected.
// If you use a MCP23017 the address from display is used also for keys.
#define UI_I2C_KEY_ADDRESS 0x40

// ###############################################################################
// ##                         Values for menu settings                          ##
// ###############################################################################

// Values used for preheat
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA 60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA 170
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS 110
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS 240
// Extreme values 
#define UI_SET_MIN_HEATED_BED_TEMP 55
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP 160
#define UI_SET_MAX_EXTRUDER_TEMP 270
#define UI_SET_EXTRUDER_RETRACT_FEEDRATE 1000 // mm/min
#define UI_SET_EXTRUDER_FEEDRATE 120 // mm/min
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3 // mm

#ifdef UI_MAIN
/* #######################################################################
                      Key definitions

The firmware is very flexible regarding your input methods. You can use one
or more of the predefined key macros, to define a mapper. If no matching mapper
is available, you can add you c-code for mapping directly into the keyboard
routines. The predefined macros do the same, just hiding the code behind it.

For each key, two seperate parts must be defined. The first is the initialization
which must be added inside ui_init_keys() and the second ist a testing routine.
These come into ui_check_keys() or ui_check_slow_keys() depending on the time needed
for testing. If you are in doubt, put it in ui_check_slow_keys().
ui_init_keys() is called from an interrupt controlling the extruder, so only
fast tests should be put there.
The detect methods need an action identifier. A list of supported ids is found
at the beginning of ui.h It's best to use the symbol name, in case the value changes.

1. Simple push button connected to gnd if closed on a free arduino pin
    init -> UI_KEYS_INIT_BUTTON_LOW(pinNumber);
    detect -> UI_KEYS_BUTTON_LOW(pinNumber,action);
    
2. Simple push button connected to 5v if closed on a free arduino pin
    init -> UI_KEYS_INIT_BUTTON_HIGH(pinNumber);
    detect -> UI_KEYS_BUTTON_HIGH(pinNumber,action);
    
3. Click encoder, A/B connected to gnd if closed.
    init -> UI_KEYS_INIT_CLICKENCODER_LOW(pinA,pinB);
    detect -> UI_KEYS_CLICKENCODER_LOW(pinA,pinB);
         or   UI_KEYS_CLICKENCODER_LOW_REV(pinA,pinB); // reverse direction
    If you can move the menu cursor without a click, just be adding some force in one direction,
    toggle the _REV with non _REV and toggle pins.
    If the direction is wrong, toggle _REV with non _REV version.
    For the push button of the encoder use 1.
    
4. Click encoder, A/B connected to 5V if closed.
    init -> UI_KEYS_INIT_CLICKENCODER_HIGH(pinA,pinB);
    detect -> UI_KEYS_CLICKENCODER_HIGH(pinA,pinB);
         or   UI_KEYS_CLICKENCODER_HIGH_REV(pinA,pinB); // reverse direction
    If you can move the menu cursor without a click, just be adding some force in one direction,
    toggle the _REV with non _REV and toggle pins.
    If the direction is wrong, toggle _REV with non _REV version.
    For the push button of the encoder use 2.
    
5. Maxtrix keyboard with 1-4 rows and 1-4 columns.
    init -> UI_KEYS_INIT_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4);
    detect -> UI_KEYS_MATRIX(r1,r2,r3,r4,c1,c2,c3,c4);
    In addition you have to set UI_MATRIX_ACTIONS to match your desired actions.
    
------- Keys connected via I2C -------------

All keys and the buzzer if present must be on a connected to a single PCF8574 chip!
As all I2C request take time, they belong all in ui_check_slow_keys.
Dont use the pin ids but instead _BV(pinNumber0_7) as pin id. 0 = First pin

6. Click encoder, A/B connected to gnd if closed.
    init -> not needed, but make sure UI_HAS_I2C_KEY is not commented out.
    detect -> UI_KEYS_I2C_CLICKENCODER_LOW(pinA,pinB);
         or   UI_KEYS_I2C_CLICKENCODER_LOW_REV(pinA,pinB); // reverse direction
    If you can move the menu cursor without a click, just be adding some force in one direction,
    toggle the _REV with non _REV and toggle pins.
    If the direction is wrong, toggle _REV with non _REV version.
    For the push button of the encoder use 7.
    NOTICE: The polling frequency is limited, so only slow turns are captured correct!

7. Simple push button connected to gnd if closed via I2C on a PCF8574
    init -> not needed, but make sure UI_HAS_I2C_KEY is not commented out.
    detect -> UI_KEYS_I2C_BUTTON_LOW(pinNumber,action);

-------- Some notes on actions -------------

There are three kinds of actions. 

Type 1: Immediate actions - these are execute and forget actions like home/pre-heat
Type 2: Parameter change action - these change the mode for next/previous keys. They are valid
        until a new change action is initiated or the action is finished with ok button.
Type 3: Show menu action. These actions have a _MENU_ in their name. If they are executed, a new
        menu is pushed on the menu stack and you see the menu. If you assign these actions directly
        to a key, you might not want this pushing behaviour. In this case add UI_ACTION_TOPMENU to the
        action, like UI_ACTION_TOPMENU+UI_ACTION_MENU_XPOSFAST. That will show the menu as top-menu
        closing all othe submenus that were open.
        
   ####################################################################### */

// Use these codes for key detect. The main menu will show the pressed action in the lcd display.
// after that assign the desired codes.
//#define UI_MATRIX_ACTIONS {2000,2001,2002,2003,2004,2005,2006,2007,2008,2009,2010,2011,2012,2013,2014,2015}
// Define your matrix actions
#define UI_MATRIX_ACTIONS {UI_ACTION_HOME_ALL, UI_ACTION_TOP_MENU,       UI_ACTION_SET_ORIGIN,      UI_ACTION_NEXT,\
                           UI_ACTION_HOME_Z,   UI_ACTION_MENU_ZPOS,      UI_ACTION_COOLDOWN,        UI_ACTION_OK,\
                           UI_ACTION_HOME_Y,   UI_ACTION_MENU_YPOSFAST,  UI_ACTION_PREHEAT_ABS,     UI_ACTION_PREVIOUS,\
                           UI_ACTION_HOME_X,   UI_ACTION_MENU_XPOSFAST,  UI_ACTION_DISABLE_STEPPER, UI_ACTION_BACK}
#ifdef UI_MATRIX_ACTIONS
const int matrixActions[] PROGMEM = UI_MATRIX_ACTIONS;
#endif

/* Normally cou want a next/previous actions with every click of your encoder.
Unfotunately, the encoder have a different count of phase changes between clicks.
Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click.
*/
#define UI_ENCODER_SPEED 1

void ui_init_keys() {
//  UI_KEYS_INIT_CLICKENCODER_LOW(47,45); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
//  UI_KEYS_INIT_BUTTON_LOW(43); // push button, connects gnd to pin
//  UI_KEYS_INIT_MATRIX(32,47,45,43,41,39,37,35);
}
void ui_check_keys(int &action) {
//  UI_KEYS_CLICKENCODER_LOW_REV(47,45); // click encoder on pins 47 and 45. Phase is connected with gnd for signals.
//  UI_KEYS_BUTTON_LOW(43,UI_ACTION_OK); // push button, connects gnd to pin
}
inline void ui_check_slow_encoder() {
#ifdef UI_HAS_I2C_KEYS
#if UI_DISPLAY_I2C_CHIPTYPE==0
  i2c_start_wait(UI_I2C_KEY_ADDRESS+I2C_READ);
  byte keymask = i2c_readNak() & UI_I2C_KEYMASK; // Read current key mask
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
    i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
    i2c_write(0x12); // GIOA
    i2c_stop();
    i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_READ);
    unsigned int keymask = i2c_readAck();
    keymask = keymask + (i2c_readNak()<<8);
#endif
  i2c_stop();
  // Add I2C click encoder tests here, all other i2c tests and a copy of the encoder test belog in ui_check_slow_keys
  //UI_KEYS_I2C_CLICKENCODER_LOW_REV(_BV(2),_BV(0)); // click encoder on pins 0 and 2. Phase is connected with gnd for signals.
#endif
}
void ui_check_slow_keys(int &action) {
#ifdef UI_HAS_I2C_KEYS
#if UI_DISPLAY_I2C_CHIPTYPE==0
    i2c_start_wait(UI_I2C_KEY_ADDRESS+I2C_READ);
    byte keymask = i2c_readNak() & UI_I2C_KEYMASK; // Read current key mask
#endif
#if UI_DISPLAY_I2C_CHIPTYPE==1
    i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_WRITE);
    i2c_write(0x12); // GPIOA
    i2c_stop();
    i2c_start_wait(UI_DISPLAY_I2C_ADDRESS+I2C_READ);
    unsigned int keymask = i2c_readAck();
    keymask = keymask + (i2c_readNak()<<8);
#endif
    i2c_stop();
    // Add I2C key tests here
   /* UI_KEYS_I2C_CLICKENCODER_LOW_REV(_BV(2),_BV(0)); // click encoder on pins 0 and 2. Phase is connected with gnd for signals.
    UI_KEYS_I2C_BUTTON_LOW(_BV(1),UI_ACTION_OK); // push button, connects gnd to pin  
    UI_KEYS_I2C_BUTTON_LOW(_BV(3),UI_ACTION_BACK); // push button, connects gnd to pin  
    UI_KEYS_I2C_BUTTON_LOW(_BV(4),UI_ACTION_MENU_QUICKSETTINGS+UI_ACTION_TOPMENU); // push button, connects gnd to pin  
    UI_KEYS_I2C_BUTTON_LOW(_BV(5),UI_ACTION_MENU_EXTRUDER+UI_ACTION_TOPMENU); // push button, connects gnd to pin  
    UI_KEYS_I2C_BUTTON_LOW(_BV(6),UI_ACTION_MENU_POSITIONS+UI_ACTION_TOPMENU); // push button, connects gnd to pin  */
  // Button handling for the Adafruit RGB shild
    UI_KEYS_I2C_BUTTON_LOW(4,UI_ACTION_PREVIOUS); // Up button
    UI_KEYS_I2C_BUTTON_LOW(8,UI_ACTION_NEXT); // down button
    UI_KEYS_I2C_BUTTON_LOW(16,UI_ACTION_BACK); // left button
    UI_KEYS_I2C_BUTTON_LOW(2,UI_ACTION_OK); // right button
    UI_KEYS_I2C_BUTTON_LOW(1,UI_ACTION_MENU_QUICKSETTINGS);  //Select button
  // ----- End RGB shield ----------
#endif

  //UI_KEYS_MATRIX(32,47,45,43,41,39,37,35);
}
/*
The menu configuration uses dynamic strings. These dynamic strings can contain
a placeholder for special values. During print these placeholder are exchanged
by their current value. Everything else is printed exactly as written.

A placeholder always has 3 chars. It starts with a % followed by 2 characters
defining the value. You can use any placeholder in any position, also it doesn't
always make sense.

List of placeholder:
%ec : Current extruder temperature
%eb : Current heated bed temperature
%e0..9 : Temp. of extruder 0..9
%er : Extruder relative mode
%Ec : Target temperature of current extruder
%Eb : Target temperature of heated bed
%E0-9 : Target temperature of extruder 0..9
%os : Status message
%oe : Error message
%oB : Buffer length
%om : Speed multiplier
%of : flow multiplier
%oc : Connection baudrate
%o0..9 : Output level extruder 0..9 is % including %sign.
%oC : Output level current extruder
%ob : Output level heated bed
%%% : The % char
%x0 : X position
%x1 : Y position
%x2 : Z position
%x3 : Current extruder position
%sx : State of x min endstop.
%sX : State of x max endstop.
%sy : State of y min endstop.
%sY : State of y max endstop.
%sz : State of z min endstop.
%sZ : State of z max endstop.
%do : Debug echo state.
%di : Debug info state.
%de : Debug error state.
%dd : Debug dry run state.
%O0 : OPS mode = 0
%O1 : OPS mode = 1
%O2 : OPS mode = 2
%Or : OPS retract distance
%Ob : OPS backslash distance
%Od : OPS min distance
%Oa : OPS move after
%ax : X acceleration during print moves
%ay : Y acceleration during print moves
%az : Z acceleration during print moves
%aX : X acceleration during travel moves
%aY : Y acceleration during travel moves
%aZ : Z acceleration during travel moves
%aj : Max. jerk
%aJ : Max. Z-jerk
%fx : Max. feedrate x direction
%fy : Max. feedrate y direction
%fz : Max. feedrate z direction
%fe : Max. feedrate current extruder
%fX : Homing feedrate x direction
%fY : Homing feedrate y direction
%fZ : Homing feedrate z direction
%Sx : Steps per mm x direction
%Sy : Steps per mm y direction
%Sz : Steps per mm z direction
%Se : Steps per mm current extruder
%is : Stepper inactive time in seconds
%ip : Max. inactive time in seconds
%X0..9 : Extruder selected marker
%Xi : PID I gain
%Xp : PID P gain
%Xd : PID D gain
%Xm : PID drive min
%XM : PID drive max
%XD : PID max
%Xw : Extruder watch period in seconds
%Xh : Extruder heat manager (BangBang/PID)
%Xa : Advance K value
%Xx : x offset in steps
%Xy : y offset in steps
%Xf : Extruder max. start feedrate
%XF : Extruder max. feedrate
%XA : Extruder max. acceleration
*/

/* ============= PAGES DEFINITION =============

If you are not iside a menu, the firmware displays pages with information.
Especially if you have only a small display it is convenient to have
more then one information page.
*/

/** How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION 4000

/** Uncomment if you don't want automatic page switching. You can still switch the
info pages with next/previous button/click-encoder */
//#define UI_DISABLE_AUTO_PAGESWITCH

/** Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_AUTORETURN_TO_MENU_AFTER 30000

/* Define your pages using dynamic strings. To define a page use
UI_PAGE4(name,row1,row2,row3,row4);
for 4 row displays and
UI_PAGE2(name,row1,row2);
for 2 row displays. You can add additional pages or change the default pages like you want.
*/
#if UI_ROWS>=4
#if HAVE_HEATED_BED==true
UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,UI_TEXT_PAGE_BED,UI_TEXT_PAGE_BUFFER,"%os");
#else
UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,"Z:%x2 mm",UI_TEXT_PAGE_BUFFER,"%os");
#endif
UI_PAGE4(ui_page2,"X:%x0 mm","Y:%x1 mm","Z:%x2 mm","%os");
/*
Merge pages together. Use the following pattern:
#define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {&ui_page1,&ui_page2}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2
#else
#if HAVE_HEATED_BED==true
UI_PAGE2(ui_page1,UI_TEXT_PAGE_EXTRUDER,UI_TEXT_PAGE_BED);
#else
UI_PAGE2(ui_page1,UI_TEXT_PAGE_EXTRUDER,"%os");
#endif
UI_PAGE2(ui_page2,"X:%x0 Y:%x1","%os");
UI_PAGE2(ui_page3,"Z:%x2 mm","%os");
/*
Merge pages together. Use the following pattern:
#define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {&ui_page1,&ui_page2,&ui_page3}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 3
#endif
/* ============ MENU definition ================

The menu works the same as pages. In addion you need to define what the lines do
or where to jump to. For that reason, the menu structure needs to be entered in
reverse order. Starting from the leaves, the menu structure is defined.
*/

/*
At first define all actions available in the menu. The actions define, what the
next/previous button will do. All menu actions work the same:
next/previous changes the value
ok sets the value if not already done and goes back to previous menu.
*/

// Error menu

UI_MENU_ACTION2(ui_menu_error,UI_ACTION_DUMMY,UI_TEXT_ERROR,"%oe");

// **** Positions submenus

#if UI_ROWS>=4
UI_MENU_ACTION4C(ui_menu_xpos,UI_ACTION_XPOSITION,UI_TEXT_ACTION_XPOSITION4);
UI_MENU_ACTION4C(ui_menu_ypos,UI_ACTION_YPOSITION,UI_TEXT_ACTION_YPOSITION4);
UI_MENU_ACTION4C(ui_menu_zpos,UI_ACTION_ZPOSITION,UI_TEXT_ACTION_ZPOSITION4);
UI_MENU_ACTION4C(ui_menu_xpos_fast,UI_ACTION_XPOSITION_FAST,UI_TEXT_ACTION_XPOSITION_FAST4);
UI_MENU_ACTION4C(ui_menu_ypos_fast,UI_ACTION_YPOSITION_FAST,UI_TEXT_ACTION_YPOSITION_FAST4);
UI_MENU_ACTION4C(ui_menu_zpos_fast,UI_ACTION_ZPOSITION_FAST,UI_TEXT_ACTION_ZPOSITION_FAST4);
#else
UI_MENU_ACTION2C(ui_menu_xpos,UI_ACTION_XPOSITION,UI_TEXT_ACTION_XPOSITION2);
UI_MENU_ACTION2C(ui_menu_ypos,UI_ACTION_YPOSITION,UI_TEXT_ACTION_YPOSITION2);
UI_MENU_ACTION2C(ui_menu_zpos,UI_ACTION_ZPOSITION,UI_TEXT_ACTION_ZPOSITION2);
UI_MENU_ACTION2C(ui_menu_xpos_fast,UI_ACTION_XPOSITION_FAST,UI_TEXT_ACTION_XPOSITION_FAST2);
UI_MENU_ACTION2C(ui_menu_ypos_fast,UI_ACTION_YPOSITION_FAST,UI_TEXT_ACTION_YPOSITION_FAST2);
UI_MENU_ACTION2C(ui_menu_zpos_fast,UI_ACTION_ZPOSITION_FAST,UI_TEXT_ACTION_ZPOSITION_FAST2);
#endif
UI_MENU_ACTION2C(ui_menu_epos,UI_ACTION_EPOSITION,UI_TEXT_ACTION_EPOSITION_FAST2);

/*
Next step is to define submenus leading to the action.
*/

// **** Positionening menu
UI_MENU_ACTIONCOMMAND(ui_menu_back,UI_TEXT_BACK,UI_ACTION_BACK);
#if UI_HAS_BACK_KEY==0
#define UI_MENU_ADDCONDBACK &ui_menu_back,
#define UI_MENU_BACKCNT 1
#else
#define UI_MENU_ADDCONDBACK
#define UI_MENU_BACKCNT 0
#endif
UI_MENU_ACTIONCOMMAND(ui_menu_home_all,UI_TEXT_HOME_ALL,UI_ACTION_HOME_ALL);
UI_MENU_ACTIONCOMMAND(ui_menu_home_x,UI_TEXT_HOME_X,UI_ACTION_HOME_X);
UI_MENU_ACTIONCOMMAND(ui_menu_home_y,UI_TEXT_HOME_Y,UI_ACTION_HOME_Y);
UI_MENU_ACTIONCOMMAND(ui_menu_home_z,UI_TEXT_HOME_Z,UI_ACTION_HOME_Z);
UI_MENU_ACTIONSELECTOR(ui_menu_go_xpos,UI_TEXT_X_POSITION,ui_menu_xpos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_ypos,UI_TEXT_Y_POSITION,ui_menu_ypos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zpos,UI_TEXT_Z_POSITION,ui_menu_zpos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_epos,UI_TEXT_E_POSITION,ui_menu_epos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_xfast,UI_TEXT_X_POS_FAST,ui_menu_xpos_fast);
UI_MENU_ACTIONSELECTOR(ui_menu_go_yfast,UI_TEXT_Y_POS_FAST,ui_menu_ypos_fast);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zfast,UI_TEXT_Z_POS_FAST,ui_menu_zpos_fast);

#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z,&ui_menu_go_xfast,&ui_menu_go_xpos,&ui_menu_go_yfast,&ui_menu_go_ypos,&ui_menu_go_zfast,&ui_menu_go_zpos,&ui_menu_go_epos}
UI_MENU(ui_menu_positions,UI_MENU_POSITIONS,11+UI_MENU_BACKCNT);

// **** Extruder menu

UI_MENU_CHANGEACTION(ui_menu_ext_temp0,UI_TEXT_EXTR0_TEMP,UI_ACTION_EXTRUDER0_TEMP);
UI_MENU_CHANGEACTION(ui_menu_ext_temp1,UI_TEXT_EXTR1_TEMP,UI_ACTION_EXTRUDER1_TEMP);
UI_MENU_CHANGEACTION(ui_menu_bed_temp, UI_TEXT_BED_TEMP,UI_ACTION_HEATED_BED_TEMP);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_sel0,UI_TEXT_EXTR0_SELECT,UI_ACTION_SELECT_EXTRUDER0);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_sel1,UI_TEXT_EXTR1_SELECT,UI_ACTION_SELECT_EXTRUDER0);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off0,UI_TEXT_EXTR0_OFF,UI_ACTION_EXTRUDER0_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off1,UI_TEXT_EXTR1_OFF,UI_ACTION_EXTRUDER1_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_origin,UI_TEXT_EXTR_ORIGIN,UI_ACTION_RESET_EXTRUDER);

#if NUM_EXTRUDER>1
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_sel0,&ui_menu_ext_sel1,
#define UI_MENU_EXTCNT 6
#else
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_off0,
#define UI_MENU_EXTCNT 2
#endif
#if HAVE_HEATED_BED==true
#define UI_MENU_BEDCOND &ui_menu_bed_temp,
#define UI_MENU_BEDCNT 1
#else
#define UI_MENU_BEDCOND
#define UI_MENU_BEDCNT 0
#endif

#define UI_MENU_EXTRUDER {UI_MENU_ADDCONDBACK UI_MENU_BEDCOND UI_MENU_EXTCOND &ui_menu_go_epos,&ui_menu_ext_origin}
UI_MENU(ui_menu_extruder,UI_MENU_EXTRUDER,UI_MENU_BACKCNT+UI_MENU_BEDCNT+UI_MENU_EXTCNT+2);

// **** SD card menu

// **** Quick menu
UI_MENU_ACTIONCOMMAND(ui_menu_quick_preheat_pla,UI_TEXT_PREHEAT_PLA,UI_ACTION_PREHEAT_PLA);
UI_MENU_ACTIONCOMMAND(ui_menu_quick_preheat_abs,UI_TEXT_PREHEAT_ABS,UI_ACTION_PREHEAT_ABS);
UI_MENU_ACTIONCOMMAND(ui_menu_quick_cooldown,UI_TEXT_COOLDOWN,UI_ACTION_COOLDOWN);
UI_MENU_ACTIONCOMMAND(ui_menu_quick_origin,UI_TEXT_SET_TO_ORIGIN,UI_ACTION_SET_ORIGIN);
UI_MENU_ACTIONCOMMAND(ui_menu_quick_stopstepper,UI_TEXT_DISABLE_STEPPER,UI_ACTION_DISABLE_STEPPER);
UI_MENU_CHANGEACTION(ui_menu_quick_speedmultiply,UI_TEXT_SPEED_MULTIPLY,UI_ACTION_FEEDRATE_MULTIPLY);
UI_MENU_CHANGEACTION(ui_menu_quick_flowmultiply,UI_TEXT_FLOW_MULTIPLY,UI_ACTION_FLOWRATE_MULTIPLY);
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_quick_preheat_pla,&ui_menu_quick_preheat_abs,&ui_menu_quick_speedmultiply,&ui_menu_quick_flowmultiply,&ui_menu_quick_cooldown,&ui_menu_quick_origin,&ui_menu_quick_stopstepper}
UI_MENU(ui_menu_quick,UI_MENU_QUICK,8+UI_MENU_BACKCNT);

// **** Fan menu

#if FAN_PIN>-1
UI_MENU_ACTIONCOMMAND(ui_menu_fan_off,UI_TEXT_FAN_OFF,UI_ACTION_FAN_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_25,UI_TEXT_FAN_25,UI_ACTION_FAN_25);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_50,UI_TEXT_FAN_50,UI_ACTION_FAN_50);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_75,UI_TEXT_FAN_75,UI_ACTION_FAN_75);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_full,UI_TEXT_FAN_FULL,UI_ACTION_FAN_FULL);
#define UI_MENU_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_off,&ui_menu_fan_25,&ui_menu_fan_50,&ui_menu_fan_75,&ui_menu_fan_full}
UI_MENU(ui_menu_fan,UI_MENU_FAN,5+UI_MENU_BACKCNT);
UI_MENU_SUBMENU(ui_menu_fan_sub,UI_TEXT_FANSPEED,ui_menu_fan);
#define UI_MENU_FAN_COND &ui_menu_fan_sub,
#define UI_MENU_FAN_CNT 1
#else
#define UI_MENU_FAN_COND 
#define UI_MENU_FAN_CNT 0
#endif

// **** SD card menu

#ifdef SDSUPPORT
#define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
UI_MENU_FILESELECT(ui_menu_sd_fileselector,UI_MENU_SD_FILESELECTOR,1);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_printfile,UI_TEXT_PRINT_FILE,UI_ACTION_SD_PRINT);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_pause,UI_TEXT_PAUSE_PRINT,UI_ACTION_SD_PAUSE);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_continue,UI_TEXT_CONTINUE_PRINT,UI_ACTION_SD_CONTINUE);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_unmount,UI_TEXT_UNMOUNT_CARD,UI_ACTION_SD_UNMOUNT);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_mount,UI_TEXT_MOUNT_CARD,UI_ACTION_SD_MOUNT);
UI_MENU_ACTIONCOMMAND(ui_menu_sd_delete,UI_TEXT_DELETE_FILE,UI_ACTION_SD_DELETE);
#define UI_MENU_SD {UI_MENU_ADDCONDBACK &ui_menu_sd_printfile,&ui_menu_sd_pause,&ui_menu_sd_continue,&ui_menu_sd_unmount,&ui_menu_sd_mount,&ui_menu_sd_delete}
UI_MENU(ui_menu_sd,UI_MENU_SD,UI_MENU_BACKCNT+6);
UI_MENU_SUBMENU(ui_menu_sd_sub,UI_TEXT_SD_CARD,ui_menu_sd);

#define UI_MENU_SD_COND &ui_menu_sd_sub,
#define UI_MENU_SD_CNT 1
#else
#define UI_MENU_SD_COND
#define UI_MENU_SD_CNT 0
#endif


// **** Debugging menu
UI_MENU_ACTIONCOMMAND(ui_menu_debug_echo,UI_TEXT_DBG_ECHO,UI_ACTION_DEBUG_ECHO);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_info,UI_TEXT_DBG_INFO,UI_ACTION_DEBUG_INFO);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_error,UI_TEXT_DBG_ERROR,UI_ACTION_DEBUG_ERROR);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_dryrun,UI_TEXT_DBG_DRYRUN,UI_ACTION_DEBUG_DRYRUN);

#define UI_MENU_DEBUGGING {UI_MENU_ADDCONDBACK &ui_menu_debug_echo,&ui_menu_debug_info,&ui_menu_debug_error,&ui_menu_debug_dryrun}
UI_MENU(ui_menu_debugging,UI_MENU_DEBUGGING,4+UI_MENU_BACKCNT);

// **** OPS configuration
#if USE_OPS==1
UI_MENU_ACTIONCOMMAND(ui_menu_ops_off,UI_TEXT_OPS_OFF,UI_ACTION_OPS_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_ops_classic,UI_TEXT_OPS_CLASSIC,UI_ACTION_OPS_CLASSIC);
UI_MENU_ACTIONCOMMAND(ui_menu_ops_fast,UI_TEXT_OPS_FAST,UI_ACTION_OPS_FAST);
UI_MENU_CHANGEACTION(ui_menu_ops_retract,UI_TEXT_OPS_RETRACT,UI_ACTION_OPS_RETRACTDISTANCE);
UI_MENU_CHANGEACTION(ui_menu_ops_backslash,UI_TEXT_OPS_BACKSLASH,UI_ACTION_OPS_BACKSLASH);
UI_MENU_CHANGEACTION(ui_menu_ops_mindist,UI_TEXT_OPS_MINDIST,UI_ACTION_OPS_MINDISTANCE);
UI_MENU_CHANGEACTION(ui_menu_ops_moveafter,UI_TEXT_OPS_MOVE_AFTER,UI_ACTION_OPS_MOVE_AFTER);
#define UI_MENU_OPS {UI_MENU_ADDCONDBACK &ui_menu_ops_off,&ui_menu_ops_classic,&ui_menu_ops_fast,&ui_menu_ops_retract,&ui_menu_ops_backslash,&ui_menu_ops_mindist,&ui_menu_ops_moveafter}
UI_MENU(ui_menu_ops,UI_MENU_OPS,7+UI_MENU_BACKCNT);
UI_MENU_SUBMENU(ui_menu_ops_sub,UI_TEXT_ANTI_OOZE,ui_menu_ops);
#define UI_MENU_ADDCONDOPS &ui_menu_ops_sub,
#else
#define UI_MENU_ADDCONDOPS
#endif

// **** Acceleration settings
UI_MENU_CHANGEACTION(ui_menu_accel_printx,UI_TEXT_PRINT_X,UI_ACTION_PRINT_ACCEL_X);
UI_MENU_CHANGEACTION(ui_menu_accel_printy,UI_TEXT_PRINT_Y,UI_ACTION_PRINT_ACCEL_Y);
UI_MENU_CHANGEACTION(ui_menu_accel_printz,UI_TEXT_PRINT_Z,UI_ACTION_PRINT_ACCEL_Z);
UI_MENU_CHANGEACTION(ui_menu_accel_travelx,UI_TEXT_MOVE_X,UI_ACTION_MOVE_ACCEL_X);
UI_MENU_CHANGEACTION(ui_menu_accel_travely,UI_TEXT_MOVE_Y,UI_ACTION_MOVE_ACCEL_Y);
UI_MENU_CHANGEACTION(ui_menu_accel_travelz,UI_TEXT_MOVE_Z,UI_ACTION_MOVE_ACCEL_Z);
UI_MENU_CHANGEACTION(ui_menu_accel_jerk,UI_TEXT_JERK,UI_ACTION_MAX_JERK);
UI_MENU_CHANGEACTION(ui_menu_accel_zjerk,UI_TEXT_ZJERK,UI_ACTION_MAX_ZJERK);
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printx,&ui_menu_accel_printy,&ui_menu_accel_printz,&ui_menu_accel_travelx,&ui_menu_accel_travely,&ui_menu_accel_travelz,&ui_menu_accel_jerk,&ui_menu_accel_zjerk}
UI_MENU(ui_menu_accel,UI_MENU_ACCEL,8+UI_MENU_BACKCNT);

// **** Feedrates
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxx,UI_TEXT_FEED_MAX_X,UI_ACTION_MAX_FEEDRATE_X);
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxy,UI_TEXT_FEED_MAX_Y,UI_ACTION_MAX_FEEDRATE_Y);
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxz,UI_TEXT_FEED_MAX_Z,UI_ACTION_MAX_FEEDRATE_Z);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homex,UI_TEXT_FEED_HOME_X,UI_ACTION_HOMING_FEEDRATE_X);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homey,UI_TEXT_FEED_HOME_Y,UI_ACTION_HOMING_FEEDRATE_Y);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homez,UI_TEXT_FEED_HOME_Z,UI_ACTION_HOMING_FEEDRATE_Z);
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxx,&ui_menu_feedrate_maxy,&ui_menu_feedrate_maxz,&ui_menu_feedrate_homex,&ui_menu_feedrate_homey,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate,UI_MENU_ACCEL,6+UI_MENU_BACKCNT);

// **** General configuration settings

UI_MENU_ACTION2C(ui_menu_stepper2,UI_ACTION_STEPPER_INACTIVE,UI_TEXT_STEPPER_INACTIVE2);
UI_MENU_ACTION2C(ui_menu_maxinactive2,UI_ACTION_MAX_INACTIVE,UI_TEXT_POWER_INACTIVE2);
UI_MENU_CHANGEACTION(ui_menu_general_baud,UI_TEXT_BAUDRATE,UI_ACTION_BAUDRATE);
UI_MENU_ACTIONSELECTOR(ui_menu_general_stepper_inactive,UI_TEXT_STEPPER_INACTIVE,ui_menu_stepper2);
UI_MENU_ACTIONSELECTOR(ui_menu_general_max_inactive,UI_TEXT_POWER_INACTIVE,ui_menu_maxinactive2);
#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive}
UI_MENU(ui_menu_general,UI_MENU_GENERAL,3+UI_MENU_BACKCNT);

// **** Extruder configuration

UI_MENU_CHANGEACTION(ui_menu_cext_steps,UI_TEXT_EXTR_STEPS,UI_ACTION_EXTR_STEPS);
UI_MENU_CHANGEACTION(ui_menu_cext_start_feedrate,UI_TEXT_EXTR_START_FEED,UI_ACTION_EXTR_START_FEEDRATE);
UI_MENU_CHANGEACTION(ui_menu_cext_max_feedrate,UI_TEXT_EXTR_MAX_FEED,UI_ACTION_EXTR_MAX_FEEDRATE);
UI_MENU_CHANGEACTION(ui_menu_cext_acceleration,UI_TEXT_EXTR_ACCEL,UI_ACTION_EXTR_ACCELERATION);
UI_MENU_CHANGEACTION(ui_menu_cext_watch_period,UI_TEXT_EXTR_WATCH,UI_ACTION_EXTR_WATCH_PERIOD);
#define UI_MENU_ADV_CNT 0
#define UI_MENU_ADVANCE
#ifdef USE_ADVANCE
#define UI_MENU_ADV_CNT 1
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel
#ifdef ENABLE_QUADRATIC_ADVANCE
#define UI_MENU_ADV_CNT 2
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel,&ui_menu_cext_advancek
UI_MENU_CHANGEACTION(ui_menu_cext_advancek,UI_TEXT_EXTR_ADVANCE_K,UI_ACTION_ADVANCE_K);
#endif
UI_MENU_CHANGEACTION(ui_menu_cext_advancel,UI_TEXT_EXTR_ADVANCE_L,UI_ACTION_ADVANCE_L);
#endif
#ifdef TEMP_PID
UI_MENU_CHANGEACTION(ui_menu_cext_manager,UI_TEXT_EXTR_MANAGER,UI_ACTION_EXTR_HEATMANAGER);
UI_MENU_CHANGEACTION(ui_menu_cext_pgain,UI_TEXT_EXTR_PGAIN,UI_ACTION_PID_PGAIN);
UI_MENU_CHANGEACTION(ui_menu_cext_igain,UI_TEXT_EXTR_IGAIN,UI_ACTION_PID_IGAIN);
UI_MENU_CHANGEACTION(ui_menu_cext_dgain,UI_TEXT_EXTR_DGAIN,UI_ACTION_PID_DGAIN);
UI_MENU_CHANGEACTION(ui_menu_cext_dmin,UI_TEXT_EXTR_DMIN,UI_ACTION_DRIVE_MIN);
UI_MENU_CHANGEACTION(ui_menu_cext_dmax,UI_TEXT_EXTR_DMAX,UI_ACTION_DRIVE_MAX);
UI_MENU_CHANGEACTION(ui_menu_cext_pmax,UI_TEXT_EXTR_PMAX,UI_ACTION_PID_MAX);
#define UI_MENU_PIDCOND ,&ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax,&ui_menu_cext_pmax
#define UI_MENU_PIDCNT 7
#else
#define UI_MENU_PIDCOND
#define UI_MENU_PIDCNT 0
#endif
#if NUM_EXTRUDER>1
UI_MENU_CHANGEACTION(ui_menu_cext_xoffset,UI_TEXT_EXTR_XOFF,UI_ACTION_X_OFFSET);
UI_MENU_CHANGEACTION(ui_menu_cext_yoffset,UI_TEXT_EXTR_YOFF,UI_ACTION_Y_OFFSET);
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 4
#else
#define UI_MENU_CONFEXTCOND 
#define UI_MENU_CONFEXTCNT 0
#endif
#define UI_MENU_CEXTR {UI_MENU_ADDCONDBACK UI_MENU_CONFEXTCOND &ui_menu_cext_steps,&ui_menu_cext_start_feedrate,&ui_menu_cext_max_feedrate,&ui_menu_cext_acceleration,&ui_menu_cext_watch_period UI_MENU_ADVANCE UI_MENU_PIDCOND}
UI_MENU(ui_menu_cextr,UI_MENU_CEXTR,5+UI_MENU_BACKCNT+UI_MENU_PIDCNT+UI_MENU_CONFEXTCNT+UI_MENU_ADV_CNT);

// **** Configuration menu
UI_MENU_SUBMENU(ui_menu_conf_general,UI_TEXT_GENERAL,ui_menu_general);
UI_MENU_SUBMENU(ui_menu_conf_accel,UI_TEXT_ACCELERATION,ui_menu_accel);
UI_MENU_SUBMENU(ui_menu_conf_feed,UI_TEXT_FEEDRATE,ui_menu_feedrate);
UI_MENU_SUBMENU(ui_menu_conf_extr,UI_TEXT_EXTRUDER,ui_menu_cextr);
#if EEPROM_MODE!=0
UI_MENU_ACTIONCOMMAND(ui_menu_conf_to_eeprom,UI_TEXT_STORE_TO_EEPROM,UI_ACTION_STORE_EEPROM);
UI_MENU_ACTIONCOMMAND(ui_menu_conf_from_eeprom,UI_TEXT_LOAD_EEPROM,UI_ACTION_LOAD_EEPROM);
#define UI_MENU_EEPROM_COND ,&ui_menu_conf_to_eeprom,&ui_menu_conf_from_eeprom
#define UI_MENU_EEPROM_CNT 2
UI_MENU_ACTION2C(ui_menu_eeprom_saved,UI_ACTION_DUMMY,UI_TEXT_EEPROM_STORED);
UI_MENU_ACTION2C(ui_menu_eeprom_loaded,UI_ACTION_DUMMY,UI_TEXT_EEPROM_LOADED);
#else
#define UI_MENU_EEPROM_COND
#define UI_MENU_EEPROM_CNT 0
#endif
#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK &ui_menu_conf_general,UI_MENU_ADDCONDOPS &ui_menu_conf_accel,&ui_menu_conf_feed,&ui_menu_conf_extr UI_MENU_EEPROM_COND}
UI_MENU(ui_menu_configuration,UI_MENU_CONFIGURATION,UI_MENU_BACKCNT+USE_OPS+UI_MENU_EEPROM_CNT+4);
// Main menu
UI_MENU_SUBMENU(ui_menu_main1,UI_TEXT_QUICK_SETTINGS,ui_menu_quick);
UI_MENU_SUBMENU(ui_menu_main2, UI_TEXT_POSITION,ui_menu_positions);
UI_MENU_SUBMENU(ui_menu_main3,UI_TEXT_EXTRUDER,ui_menu_extruder);
UI_MENU_SUBMENU(ui_menu_main4,UI_TEXT_DEBUGGING,ui_menu_debugging);
UI_MENU_SUBMENU(ui_menu_main5,UI_TEXT_CONFIGURATION,ui_menu_configuration);
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK  &ui_menu_main1,&ui_menu_main2,&ui_menu_main3,UI_MENU_FAN_COND UI_MENU_SD_COND &ui_menu_main4,&ui_menu_main5}
UI_MENU(ui_menu_main,UI_MENU_MAIN,5+UI_MENU_BACKCNT+UI_MENU_SD_CNT+UI_MENU_FAN_CNT);

/* Define menus accessible by action commands

You can create up to 10 user menus which are accessible by the action commands UI_ACTION_SHOW_USERMENU1 until UI_ACTION_SHOW_USERMENU10
You this the same way as with the menus above or you use one of the above menus. Then add a define like

#define UI_USERMENU1 ui_menu_conf_feed

which assigns the menu stored in ui_menu_conf_feed to the action UI_ACTION_SHOW_USERMENU1. Make sure only to change the numbers and not the name of the define.

When do you need this? You might want a fast button to change the temperature. In the default menu you have no menu
to change the temperature and view it the same time. So you need to make an action menu for this like:
UI_MENU_ACTION4C(ui_menu_extrtemp,UI_ACTION_EXTRUDER0_TEMP,"Temp. 0  :%E0\002C","","","");
Then you assign this menu to a usermenu:
#define UI_USERMENU2 ui_menu_extrtemp

Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperture screen and allows
the change of temperature with the next/previous buttons.

*/


#endif
