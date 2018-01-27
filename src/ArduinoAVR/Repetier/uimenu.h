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
#if !defined(_UI_MENU_H)
#define _UI_MENU_H

/*moved to uilang.h
  #define cUP "\001"
  #define cDEG "\002"
  #define cSEL "\003"
  #define cUNSEL "\004"
  #define cTEMP "\005"
  #define cFOLD "\006"
  #define cARROW "\176"
*/
/*
  The menu configuration uses dynamic strings. These dynamic strings can contain
  a placeholder for special values. During print these placeholder are exchanged
  by their current value. Everything else is printed exactly as written.

  A placeholder always has 3 chars. It starts with a % followed by 2 characters
  defining the value. You can use any placeholder in any position, also it doesn't
  always make sense.

  Special Characters
  constant   description
  cUP        Folder up arrow
  cDEG       Degree mark
  cSEL       Selected
  cUNSEL     Unselected
  cTEMP      Thermometer symbol
  cFOLD      Folder symbol

  List of placeholder:
  %%% : The % char
  %% :  The % char (also)

  %?<c> : Conditional. Print c if current char is not c. Allows avoiding duplicate character, like space

  acceleration
  %ax : X acceleration during print moves
  %ay : Y acceleration during print moves
  %az : Z acceleration during print moves
  %aX : X acceleration during travel moves
  %aY : Y acceleration during travel moves
  %aZ : Z acceleration during travel moves
  %aj : Max. jerk
  %aJ : Max. Z-jerk

  debug
  %do : Debug echo state.
  %di : Debug info state.
  %de : Debug error state.
  %dd : Debug dry run state.
  %dp : Debug end stop state.
  endstop stats with only one char
  %dx : Debug min x endstop
  %dX : Debug max x endstop
  %dy : Debug min y endstop
  %dY : Debug max y endstop
  %dz : Debug min z endstop
  %dZ : Debug max z endstop
  
  extruder
  %ec : Current extruder temperature
  %ed : Number of copies for ditto mode
  %eIc : Current extruder temperature integer (shorter)
  %eb : Current heated bed temperature
  %e0..9 : Temp. of extruder 0..9
  %et : Thermo controlled fan temperature
  %er : Extruder relative mode
  %Ec : Target temperature of current extruder
  %Eb : Target temperature of heated bed
  %E0-9 : Target temperature of extruder 0..9
  %ej : Checkbox with jam enabled state
  %D0-3 : Ditto mode selected
  feed rate
  %fx : Max. feedrate x direction
  %fy : Max. feedrate y direction
  %fz : Max. feedrate z direction
  %fe : Max. feedrate current extruder
  %fX : Homing feedrate x direction
  %fY : Homing feedrate y direction
  %fZ : Homing feedrate z direction
  %Fs : Fan speed
  %FS : Fan2 speed
  %Fi : ignore M106 commands state

  inactivity
  %is : Stepper inactive time in minutes
  %ip : Max. inactive time in minutes

  random stuff
  %os : Status message
  %oe : Error message
  %oB : Buffer length
  %om : Speed multiplier
  %of : flow multiplier
  %oc : Connection baudrate
  %o0..9 : Output level extruder 0..9 is % including %sign.
  %oC : Output level current extruder
  %ob : Output level heated bed
  %PN : Printer name
  %on : current extruder number (1,2,3...)
  %oS : servo position
  %oY : baby steps counter
  %BC : Bed coating thickness
  %oP : PWM position Laser/Spindle

  Print status related
  %Pn : filename being printed
  %Pl : current layer
  %PL : num layer
  %pp : Progress in percent, one digit 

  stops
  %sx : State of x min endstop.
  %sX : State of x max endstop.
  %sy : State of y min endstop.
  %sY : State of y max endstop.
  %sz : State of z min endstop.
  %sZ : State of z max endstop.

  steps
  %Sx : Steps per mm x direction
  %Sy : Steps per mm y direction
  %Sz : Steps per mm z direction
  %Se : Steps per mm current extruder

  totals
  %Ut : Shows printing time in days hours:minutes
  %Uh : Shows printing time in hours
  %Uf : Shows total filament usage

  extruder position
  %x0 : X position
  %x1 : Y position
  %x2 : Z position
  %x3 : Current extruder position
  %x4 : Printed since temperature on in meters (for filament usage)
  
  %x5 : X position include offset
  %x6 : Y position include offset
  %x7 : Z position include offset  
 
  %xa-%xf : Extruder state icon 0x08 or 0x09 or 0x0a (off) - works only with graphic displays!
  %xB : Bed icon state 0x0c or 0x0d or 0x0b (off) Bed state - works only with graphic displays!
  
  Print offsets
  %T0 : X offset
  %T1 : Y offset
  %T2 : Z offset

  extruder parameters
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
  %Xl : Advance L value
  %Xx : x offset in mm
  %Xy : y offset in mm
  %Xz : z offset in mm
  %Xf : Extruder max. start feedrate
  %XF : Extruder max. feedrate
  %XA : Extruder max. acceleration

  delta stuff
  %y0-3 : same as %y0-3 back calculated from delta position steps
  %Y0-3 : raw delta position steps (no round off error to display it)
  %yD : delta printer low tower distance
  %YL : delta print envelope radius Limit
  %yx : low towers x offset mm
  %yy : low towers y offset mm
  %Yx : low towers x offset steps
  %Yy : low towers y offset steps
  %yX : high (Z) tower x offset mm
  %yY : high (Z) tower y offset mm
  %YX : high (Z) tower x offset steps
  %YY : high (Z) tower y offset steps

  Z-Probing
  %zh : z-probe height
  %ll : Autolevel correction enabled
  %De : Distortion correction enabled
  
  Wizard variables
  %w0-7 : Wizard variables 0-7 as integer
  %W0-7 : Wizard variables 0-7 as float 2 digit precision
  %WA : Wizard variable 0 as float 1 digit precision
  %WB : Wizard variable 0 as float 1 digit precision
*/

#if UI_DISPLAY_TYPE != NO_DISPLAY

#ifdef CUSTOM_MENU
#include "CustomMenu.h"
#else

// Define precision for temperatures. With small displays only integer values fit.
#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 1
#else
#define UI_TEMP_PRECISION 0
#endif
#endif

#if HAVE_HEATED_BED // make sure it is only 0 or 1 to be used in menu math
#undef HAVE_HEATED_BED
#define HAVE_HEATED_BED 1
#else
#undef HAVE_HEATED_BED
#define HAVE_HEATED_BED 0
#endif

/* ============= PAGES DEFINITION =============

  If you are not inside a menu, the firmware displays pages with information.
  Especially if you have only a small display it is convenient to have
  more then one information page.
*/

/* Define your pages using dynamic strings. To define a page use
  UI_PAGE6(name,row1,row2,row3,row4,row5,row6);
  UI_PAGE4(name,row1,row2,row3,row4);
  for 4 row displays and
  UI_PAGE2(name,row1,row2);
  for 2 row displays. You can add additional pages or change the default pages like you want.
*/

#if UI_ROWS>=5 && UI_DISPLAY_TYPE == DISPLAY_U8G

//graphic main status

UI_PAGE6_T(ui_page1, UI_TEXT_MAINPAGE6_1_ID, UI_TEXT_MAINPAGE6_2_ID, UI_TEXT_MAINPAGE6_3_ID, UI_TEXT_MAINPAGE6_4_ID, UI_TEXT_MAINPAGE6_5_ID, UI_TEXT_MAINPAGE6_6_ID)

#if EEPROM_MODE != 0 && NUM_EXTRUDER > 0
UI_PAGE6_T(ui_page2, UI_TEXT_PRINT_TIME_ID, UI_TEXT_PRINT_TIME_VALUE_ID, UI_TEXT_PRINT_FILAMENT_ID, UI_TEXT_PRINT_FILAMENT_VALUE_ID,UI_TEXT_EMPTY_ID,UI_TEXT_STATUS_ID)
#define UI_PRINTTIME_PAGES ,&ui_page2
#define UI_PRINTTIME_COUNT 1
#else
#define UI_PRINTTIME_PAGES
#define UI_PRINTTIME_COUNT 0
#endif

#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_PAGE6_T(ui_page3, UI_TEXT_EXTR0_TEMP_ID, UI_TEXT_EXTR1_TEMP_ID, 
#if NUM_EXTRUDER > 2
  UI_TEXT_EXTR2_TEMP_ID,
#endif
#if NUM_EXTRUDER > 3
           UI_TEXT_EXTR3_TEMP_ID,
#endif
#if NUM_EXTRUDER > 4
           UI_TEXT_EXTR4_TEMP_ID,
#endif
#if HAVE_HEATED_BED && NUM_EXTRUDER < 5
           UI_TEXT_BED_TEMP_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 5
            UI_TEXT_FLOW_MULTIPLY_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 3
           UI_TEXT_EMPTY_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 4
            UI_TEXT_EMPTY_ID,
#endif
           UI_TEXT_STATUS_ID
)
#define UI_EXTRUDERS_PAGES ,&ui_page3
#define UI_EXTRUDERS_PAGES_COUNT 1
#elif NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1
UI_PAGE6_T(ui_page3, UI_TEXT_EXTR0_TEMP_ID,
#if HAVE_HEATED_BED
UI_TEXT_BED_TEMP_ID,
#endif
 UI_TEXT_FLOW_MULTIPLY_ID,
 UI_TEXT_EMPTY_ID,
 UI_TEXT_EMPTY_ID,
#if !HAVE_HEATED_BED
 UI_TEXT_EMPTY_ID,
#endif  
 UI_TEXT_STATUS_ID
)
#define UI_EXTRUDERS_PAGES ,&ui_page3
#define UI_EXTRUDERS_PAGES_COUNT 1
#else
#define UI_EXTRUDERS_PAGES
#define UI_EXTRUDERS_PAGES_COUNT 0
#endif

UI_PAGE6_T(ui_page4, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4A_ID,UI_TEXT_PAGE_BUFFER_ID,UI_TEXT_SPEED_MULTIPLY_ID, UI_TEXT_STATUS_ID)

/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {&ui_page1 UI_PRINTTIME_PAGES ,&ui_page4 UI_EXTRUDERS_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2+UI_PRINTTIME_COUNT+UI_EXTRUDERS_PAGES_COUNT

#elif UI_ROWS >= 4
#if HAVE_HEATED_BED
#if NUM_EXTRUDER > 0
//   UI_PAGE4(ui_page1,cTEMP "%ec/%Ec" cDEG "B%eB/%Eb" cDEG,"Z:%x2  Buf : %oB","Mul: %om   Flow: %of","%os")
UI_PAGE4_T(ui_page1, UI_TEXT_MAINPAGE_TEMP_BED_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#else
//   UI_PAGE4(ui_page1,"B%eB/%Eb" cDEG,"Z:%x2  Buf : %oB","Mul: %om   Flow: %of","%os")
UI_PAGE4_T(ui_page1, UI_TEXT_MAINPAGE_BED_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#endif
//UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,UI_TEXT_PAGE_BED,UI_TEXT_PAGE_BUFFER,"%os");
#else // no bed
#if NUM_EXTRUDER > 0
UI_PAGE4_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#else
UI_PAGE4_T(ui_page1, UI_TEXT_EMPTY_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#endif
#endif
UI_PAGE4_T(ui_page2, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_STATUS_ID)
//UI_PAGE4(ui_page2,"dX:%y0 mm %sX","dY:%y1 mm %sY","dZ:%y2 mm %sZ","%os");
#if NUM_EXTRUDER > 0
#if NUM_EXTRUDER == 1 || (NUM_EXTRUDER > 1 && MIXING_EXTRUDER > 0)
UI_PAGE4_T(ui_page3, UI_TEXT_PAGE_EXTRUDER_ID
#else
UI_PAGE4_T(ui_page3, UI_TEXT_PAGE_EXTRUDER1_ID
#endif
#else
UI_PAGE4_T(ui_page3
#endif
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
           , UI_TEXT_PAGE_EXTRUDER2_ID
#endif
#if NUM_EXTRUDER>2 && MIXING_EXTRUDER == 0
           , UI_TEXT_PAGE_EXTRUDER3_ID
#endif
#if HAVE_HEATED_BED
           , UI_TEXT_PAGE_BED_ID
#endif
#if (NUM_EXTRUDER >= 3 && MIXING_EXTRUDER == 0 && !HAVE_HEATED_BED) || (NUM_EXTRUDER==2 && MIXING_EXTRUDER == 0 && HAVE_HEATED_BED==true)
           , UI_TEXT_STATUS_ID
#elif (NUM_EXTRUDER == 2 && MIXING_EXTRUDER == 0) || ((NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1) && HAVE_HEATED_BED)
           , UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#elif (NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1) || (NUM_EXTRUDER == 0 &&  HAVE_HEATED_BED)
           , UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#elif NUM_EXTRUDER == 0
           , UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#endif
          )
#if EEPROM_MODE != 0 && NUM_EXTRUDER > 0
UI_PAGE4_T(ui_page4, UI_TEXT_PRINT_TIME_ID, UI_TEXT_PRINT_TIME_VALUE_ID, UI_TEXT_PRINT_FILAMENT_ID, UI_TEXT_PRINT_FILAMENT_VALUE_ID)
#define UI_PRINTTIME_PAGES ,&ui_page4
#define UI_PRINTTIME_COUNT 1
#else
#define UI_PRINTTIME_PAGES
#define UI_PRINTTIME_COUNT 0
#endif
/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {&ui_page1, &ui_page2, &ui_page3 UI_PRINTTIME_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 3+UI_PRINTTIME_COUNT
#else
#if HAVE_HEATED_BED
UI_PAGE2_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_PAGE_BED_ID)
#else
UI_PAGE2_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_STATUS_ID)
#endif
UI_PAGE2_T(ui_page2, UI_TEXT_MAINPAGE_XY_ID, UI_TEXT_STATUS_ID)
UI_PAGE2_T(ui_page3, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_STATUS_ID)
/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {&ui_page1,&ui_page2,&ui_page3}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 3
#endif

/* ============ MENU definition ================

  The menu works the same as pages. In additon you need to define what the lines do
  or where to jump to. For that reason, the menu structure needs to be entered in
  reverse order. Starting from the leaves, the menu structure is defined.
*/

/*
  At first define all actions available in the menu. The actions define, what the
  next/previous button will do. All menu actions work the same:
  next/previous changes the value
  ok sets the value if not already done and goes back to previous menu.
*/

UI_MENU_ACTIONCOMMAND_T(ui_menu_back, UI_TEXT_BACK_ID, UI_ACTION_BACK)
#if UI_HAS_BACK_KEY == 0
#define UI_MENU_ADDCONDBACK &ui_menu_back,
#define UI_MENU_ADDCONDBACK_C ,&ui_menu_back
#define UI_MENU_BACKCNT 1
#else
#define UI_MENU_ADDCONDBACK
#define UI_MENU_ADDCONDBACK_C
#define UI_MENU_BACKCNT 0
#endif


// Language selection menu

#if EEPROM_MODE != 0
#define FIRSTLANG 1
#if LANGUAGE_EN_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_en, "English", UI_ACTION_LANGUAGE_EN | UI_ACTION_TOPMENU)
#define ADD_LANG_EN &ui_menu_setlang_en
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_EN
#endif // LANGUAGE_EN_ACTIVE
#if LANGUAGE_DE_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_de, "Deutsch", UI_ACTION_LANGUAGE_DE | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_DE &ui_menu_setlang_de
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_DE ,&ui_menu_setlang_de
#endif
#else
#define ADD_LANG_DE
#endif // LANGUAGE_DE_ACTIVE
#if LANGUAGE_ES_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_es, "Espanol", UI_ACTION_LANGUAGE_ES | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_ES &ui_menu_setlang_es
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_ES ,&ui_menu_setlang_es
#endif
#else
#define ADD_LANG_ES
#endif // LANGUAGE_ES_ACTIVE
#if LANGUAGE_PT_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_pt, "Portugues", UI_ACTION_LANGUAGE_PT | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_PT &ui_menu_setlang_pt
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_PT ,&ui_menu_setlang_pt
#endif
#else
#define ADD_LANG_PT
#endif // LANGUAGE_PT_ACTIVE
#if LANGUAGE_FR_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_fr, "Francais", UI_ACTION_LANGUAGE_FR | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_FR &ui_menu_setlang_fr
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_FR ,&ui_menu_setlang_fr
#endif
#else
#define ADD_LANG_FR
#endif // LANGUAGE_FR_ACTIVE
#if LANGUAGE_NL_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_nl, "Nederlandse", UI_ACTION_LANGUAGE_NL | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_NL &ui_menu_setlang_nl
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_NL ,&ui_menu_setlang_nl
#endif
#else
#define ADD_LANG_NL
#endif // LANGUAGE_NL_ACTIVE
#if LANGUAGE_IT_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_it, "Italiano", UI_ACTION_LANGUAGE_IT | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_IT &ui_menu_setlang_it
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_IT ,&ui_menu_setlang_it
#endif
#else
#define ADD_LANG_IT
#endif // LANGUAGE_IT_ACTIVE
#if LANGUAGE_FI_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_fi, "Suomi", UI_ACTION_LANGUAGE_FI | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_SE &ui_menu_setlang_fi
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_FI ,&ui_menu_setlang_fi
#endif
#else
#define ADD_LANG_FI
#endif // LANGUAGE_FI_ACTIVE
#if LANGUAGE_SE_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_se, "Svenska", UI_ACTION_LANGUAGE_SE | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_SE &ui_menu_setlang_se
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_SE ,&ui_menu_setlang_se
#endif
#else
#define ADD_LANG_SE
#endif // LANGUAGE_SE_ACTIVE
#if LANGUAGE_CZ_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_cz, "Cestina", UI_ACTION_LANGUAGE_CZ | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_CZ &ui_menu_setlang_cz
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_CZ ,&ui_menu_setlang_cz
#endif
#else
#define ADD_LANG_CZ
#endif // LANGUAGE_CZ_ACTIVE
#if LANGUAGE_PL_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_pl, "Polski", UI_ACTION_LANGUAGE_PL | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_PL &ui_menu_setlang_pl
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_PL ,&ui_menu_setlang_pl
#endif
#else
#define ADD_LANG_PL
#endif // LANGUAGE_PL_ACTIVE
#if LANGUAGE_TR_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_tr, "T" STR_uuml "rk", UI_ACTION_LANGUAGE_TR | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_TR &ui_menu_setlang_tr
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_TR ,&ui_menu_setlang_tr
#endif
#else
#define ADD_LANG_TR
#endif // LANGUAGE_TR_ACTIVE

#define UI_MENU_LANGUAGES {UI_MENU_ADDCONDBACK ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR}
#define UI_MENU_LANGUAGES_WIZ {ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR}
UI_MENU(ui_menu_languages, UI_MENU_LANGUAGES, UI_MENU_BACKCNT + LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE)
UI_MENU_SUBMENU_T(ui_menu_conf_lang, UI_TEXT_LANGUAGE_ID, ui_menu_languages)
UI_STICKYMENU(ui_menu_languages_wiz, UI_MENU_LANGUAGES_WIZ, LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE)
#define LANGMENU_ENTRY &ui_menu_conf_lang,
#define LANGMENU_COUNT 1
#else
#define LANGMENU_ENTRY
#define LANGMENU_COUNT 0
#endif

// Helper line

UI_MENU_HEADLINE_T(ui_menu_empty,UI_TEXT_EMPTY_ID)


// Error menu

UI_MENU_ACTION2_T(ui_menu_error, UI_ACTION_DUMMY, UI_TEXT_ERROR_ID, UI_TEXT_ERRORMSG_ID)

// Messages
#if UI_ROWS >= 4
UI_WIZARD4_T(ui_msg_decoupled, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_HEATER_DECOUPLED_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_defectsensor, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_TEMPSENSOR_DEFECT_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_slipping, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_SLIPPING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_leveling_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_LEVELING_ERROR_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_calibration_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_CALIBRATION_ERROR_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_clearbed1, UI_ACTION_AUTOLEVEL2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_clearbed2, UI_ACTION_MEASURE_DISTORTION2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_calibrating_bed, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_CALIBRATING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4_T(ui_msg_homing, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_HOMING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)

#else
UI_WIZARD2_T(ui_msg_decoupled, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_HEATER_DECOUPLED_ID)
UI_WIZARD2_T(ui_msg_defectsensor, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_TEMPSENSOR_DEFECT_ID)
UI_WIZARD2_T(ui_msg_slipping, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_SLIPPING_ID)
UI_WIZARD2_T(ui_msg_leveling_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_LEVELING_ERROR_ID)
UI_WIZARD2_T(ui_msg_calibration_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_CALIBRATION_ERROR_ID)
UI_WIZARD2_T(ui_msg_clearbed1, UI_ACTION_AUTOLEVEL2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID)
UI_WIZARD2_T(ui_msg_calibrating_bed, UI_ACTION_STATE, UI_TEXT_CALIBRATING_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD2_T(ui_msg_homing, UI_ACTION_STATE, UI_TEXT_HOMING_ID, UI_TEXT_PLEASE_WAIT_ID)
#endif

// Filament change wizard

#if FEATURE_RETRACTION
#if UI_ROWS >= 4
UI_WIZARD4_T(ui_wiz_filamentchange, UI_ACTION_WIZARD_FILAMENTCHANGE, UI_TEXT_WIZ_CH_FILAMENT1_ID, UI_TEXT_WIZ_CH_FILAMENT2_ID, UI_TEXT_WIZ_CH_FILAMENT3_ID, UI_TEXT_CLICK_DONE_ID)
UI_WIZARD4_T(ui_wiz_jamwaitheat, UI_ACTION_WIZARD_JAM_WAITHEAT, UI_TEXT_WIZ_WAITTEMP1_ID, UI_TEXT_WIZ_WAITTEMP2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_TEMP_SET_ID)
UI_WIZARD4_T(ui_wiz_jamreheat, UI_ACTION_WIZARD_JAM_REHEAT, UI_TEXT_WIZ_REHEAT1_ID, UI_TEXT_WIZ_REHEAT2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_CURRENT_TEMP_ID)
#else
UI_WIZARD2_T(ui_wiz_filamentchange, UI_ACTION_WIZARD_FILAMENTCHANGE, UI_TEXT_WIZ_CH_FILAMENT1_ID, UI_TEXT_CLICK_DONE_ID)
UI_WIZARD2_T(ui_wiz_jamwaitheat, UI_ACTION_WIZARD_JAM_WAITHEAT, UI_TEXT_WIZ_WAITTEMP1_ID, UI_TEXT_WIZ_WAITTEMP2_ID)
UI_WIZARD2_T(ui_wiz_jamreheat, UI_ACTION_WIZARD_JAM_REHEAT, UI_TEXT_WIZ_REHEAT1_ID, UI_TEXT_WIZ_REHEAT2_ID)
#endif
#endif

// **** Positions sub menus

#if UI_ROWS >= 4
UI_MENU_ACTION4_T(ui_menu_xpos, UI_ACTION_XPOSITION, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_XPOSITION4B_ID, UI_TEXT_ACTION_XPOSITION4C_ID, UI_TEXT_ACTION_XPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_ypos, UI_ACTION_YPOSITION, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4B_ID, UI_TEXT_ACTION_YPOSITION4C_ID, UI_TEXT_ACTION_YPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos, UI_ACTION_ZPOSITION, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4B_ID, UI_TEXT_ACTION_ZPOSITION4C_ID, UI_TEXT_ACTION_ZPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_notest, UI_ACTION_ZPOSITION_NOTEST, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4B_ID, UI_TEXT_ACTION_ZPOSITION4C_ID, UI_TEXT_ACTION_ZPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_xpos_fast, UI_ACTION_XPOSITION_FAST, UI_TEXT_ACTION_XPOSITION_FAST4A_ID, UI_TEXT_ACTION_XPOSITION_FAST4B_ID, UI_TEXT_ACTION_XPOSITION_FAST4C_ID, UI_TEXT_ACTION_XPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_ypos_fast, UI_ACTION_YPOSITION_FAST, UI_TEXT_ACTION_YPOSITION_FAST4A_ID, UI_TEXT_ACTION_YPOSITION_FAST4B_ID, UI_TEXT_ACTION_YPOSITION_FAST4C_ID, UI_TEXT_ACTION_YPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_fast, UI_ACTION_ZPOSITION_FAST, UI_TEXT_ACTION_ZPOSITION_FAST4A_ID, UI_TEXT_ACTION_ZPOSITION_FAST4B_ID, UI_TEXT_ACTION_ZPOSITION_FAST4C_ID, UI_TEXT_ACTION_ZPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_fast_notest, UI_ACTION_ZPOSITION_FAST_NOTEST, UI_TEXT_ACTION_ZPOSITION_FAST4A_ID, UI_TEXT_ACTION_ZPOSITION_FAST4B_ID, UI_TEXT_ACTION_ZPOSITION_FAST4C_ID, UI_TEXT_ACTION_ZPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_epos, UI_ACTION_EPOSITION, UI_TEXT_ACTION_EPOSITION_FAST2A_ID, UI_TEXT_ACTION_EPOSITION_FAST2B_ID, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_METER_PRINTED_ID)
#else
UI_MENU_ACTION2_T(ui_menu_xpos, UI_ACTION_XPOSITION, UI_TEXT_ACTION_XPOSITION2A_ID, UI_TEXT_ACTION_XPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_ypos, UI_ACTION_YPOSITION, UI_TEXT_ACTION_YPOSITION2A_ID, UI_TEXT_ACTION_YPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos, UI_ACTION_ZPOSITION, UI_TEXT_ACTION_ZPOSITION2A_ID, UI_TEXT_ACTION_ZPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_notest, UI_ACTION_ZPOSITION_NOTEST, UI_TEXT_ACTION_ZPOSITION2A_ID, UI_TEXT_ACTION_ZPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_xpos_fast, UI_ACTION_XPOSITION_FAST, UI_TEXT_ACTION_XPOSITION_FAST2A_ID, UI_TEXT_ACTION_XPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_ypos_fast, UI_ACTION_YPOSITION_FAST, UI_TEXT_ACTION_YPOSITION_FAST2A_ID, UI_TEXT_ACTION_YPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_fast, UI_ACTION_ZPOSITION_FAST, UI_TEXT_ACTION_ZPOSITION_FAST2A_ID, UI_TEXT_ACTION_ZPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_fast_notest, UI_ACTION_ZPOSITION_FAST_NOTEST, UI_TEXT_ACTION_ZPOSITION_FAST2A_ID, UI_TEXT_ACTION_ZPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_epos, UI_ACTION_EPOSITION, UI_TEXT_ACTION_EPOSITION_FAST2A_ID, UI_TEXT_ACTION_EPOSITION_FAST2B_ID)
#endif

/*
  Next step is to define sub menus leading to the action.
*/

// **** Positioning menu
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_all, UI_TEXT_HOME_ALL_ID, UI_ACTION_HOME_ALL, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_x, UI_TEXT_HOME_X_ID, UI_ACTION_HOME_X, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_y, UI_TEXT_HOME_Y_ID, UI_ACTION_HOME_Y, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_z, UI_TEXT_HOME_Z_ID, UI_ACTION_HOME_Z, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_xpos, UI_TEXT_X_POSITION_ID, ui_menu_xpos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_ypos, UI_TEXT_Y_POSITION_ID, ui_menu_ypos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zpos, UI_TEXT_Z_POSITION_ID, ui_menu_zpos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zpos_notest, UI_TEXT_Z_POSITION_ID, ui_menu_zpos_notest)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_epos, UI_TEXT_E_POSITION_ID, ui_menu_epos)
#if !UI_SPEEDDEPENDENT_POSITIONING
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_xfast, UI_TEXT_X_POS_FAST_ID, ui_menu_xpos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_yfast, UI_TEXT_Y_POS_FAST_ID, ui_menu_ypos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zfast, UI_TEXT_Z_POS_FAST_ID, ui_menu_zpos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zfast_notest, UI_TEXT_Z_POS_FAST_ID, ui_menu_zpos_fast_notest)
#define UI_SPEED 2
#define UI_SPEED_X ,&ui_menu_go_xfast,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_yfast,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zfast,&ui_menu_go_zpos
#define UI_SPEED_Z_NOTEST ,&ui_menu_go_zfast_notest,&ui_menu_go_zpos_notest
#else
#define UI_SPEED 1
#define UI_SPEED_X ,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zpos
#define UI_SPEED_Z_NOTEST ,&ui_menu_go_zpos_notest
#endif
#if FEATURE_SERVO > 0 && UI_SERVO_CONTROL > 0
UI_MENU_CHANGEACTION_T(ui_menu_servopos, UI_TEXT_SERVOPOS_ID, UI_ACTION_SERVOPOS)
#define SERVOPOS_COUNT 1
#define SERVOPOS_ENTRY ,&ui_menu_servopos
#else
#define SERVOPOS_COUNT 0
#define SERVOPOS_ENTRY
#endif
// Offsets menu
UI_MENU_CHANGEACTION_T(ui_menu_off_xpos, UI_TEXT_X_OFFSET_ID, UI_ACTION_XOFF)
UI_MENU_CHANGEACTION_T(ui_menu_off_ypos, UI_TEXT_Y_OFFSET_ID, UI_ACTION_YOFF)
UI_MENU_CHANGEACTION_T(ui_menu_off_zpos, UI_TEXT_Z_OFFSET_ID, UI_ACTION_ZOFF)
#define UI_MENU_OFFSETS {UI_MENU_ADDCONDBACK &ui_menu_off_xpos,&ui_menu_off_ypos,&ui_menu_off_zpos}
UI_MENU(ui_menu_offsets, UI_MENU_OFFSETS, UI_MENU_BACKCNT + 3)
UI_MENU_SUBMENU_T(ui_menu_go_offsets, UI_TEXT_OFFSETS_ID, ui_menu_offsets)

#if DRIVE_SYSTEM != DELTA     //Positioning menu for non-delta
#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z UI_SPEED_X UI_SPEED_Y UI_SPEED_Z ,&ui_menu_go_epos SERVOPOS_ENTRY,&ui_menu_go_offsets}
UI_MENU(ui_menu_positions, UI_MENU_POSITIONS, 6 + 3 * UI_SPEED + UI_MENU_BACKCNT + SERVOPOS_COUNT)
#else                   //Positioning menu for delta (removes individual x,y,z homing)
#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all  UI_SPEED_X UI_SPEED_Y UI_SPEED_Z ,&ui_menu_go_epos SERVOPOS_ENTRY,&ui_menu_go_offsets}
UI_MENU(ui_menu_positions, UI_MENU_POSITIONS, 3 + 3 * UI_SPEED + UI_MENU_BACKCNT + SERVOPOS_COUNT)
#endif

#if FEATURE_Z_PROBE
UI_MENU_HEADLINE_T(ui_menu_mzp_head,UI_TEXT_MEAS_ZP_HEIGHT_ID)
UI_MENU_CHANGEACTION_T(ui_menu_mzp_realz,UI_TEXT_REAL_Z_ID,UI_ACTION_MEASURE_ZP_REALZ)
UI_MENU_ACTIONCOMMAND_T(ui_menu_mzp_cont, UI_TEXT_CONTINUE_ID, UI_ACTION_MEASURE_ZPROBE_HEIGHT2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_mzp_close, UI_TEXT_CLOSE_ID, UI_ACTION_BACK)
#define UI_MENU_MZP_ITEMS {&ui_menu_mzp_head,&ui_menu_mzp_realz,&ui_menu_mzp_cont,&ui_menu_mzp_close}
UI_STICKYMENU(ui_menu_mzp,UI_MENU_MZP_ITEMS,4)
UI_MENU_ACTIONCOMMAND_T(ui_menu_measure_zprobe_height, UI_TEXT_MEAS_ZP_HEIGHT_ID, UI_ACTION_MEASURE_ZPROBE_HEIGHT)
#define UI_CAL_ZHEIGHT_ENTRY ,&ui_menu_measure_zprobe_height
#define UI_CAL_ZHEIGHT_CNT 1
#else
#define UI_CAL_ZHEIGHT_ENTRY
#define UI_CAL_ZHEIGHT_CNT 0
#endif

// **** Delta calibration menu
#if Z_HOME_DIR > 0
#if FEATURE_Z_PROBE
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_measured_origin, UI_TEXT_SET_MEASURED_ORIGIN_ID, UI_ACTION_SET_MEASURED_ORIGIN)
#define UI_MENU_DELTA {UI_MENU_ADDCONDBACK &ui_menu_home_all UI_SPEED_Z_NOTEST, &ui_menu_set_measured_origin, &ui_menu_measure_zprobe_height}
UI_MENU(ui_menu_delta, UI_MENU_DELTA, 3 + UI_SPEED + UI_MENU_BACKCNT)
#else
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_measured_origin, UI_TEXT_SET_MEASURED_ORIGIN_ID, UI_ACTION_SET_MEASURED_ORIGIN)
#define UI_MENU_DELTA {UI_MENU_ADDCONDBACK &ui_menu_home_all UI_SPEED_Z_NOTEST,&ui_menu_set_measured_origin}
UI_MENU(ui_menu_delta, UI_MENU_DELTA, 2 + UI_SPEED + UI_MENU_BACKCNT)
#endif
#endif

// **** Bed leveling menu
#if SOFTWARE_LEVELING
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p1, UI_TEXT_SET_P1_ID, UI_ACTION_SET_P1)
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p2, UI_TEXT_SET_P2_ID, UI_ACTION_SET_P2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p3, UI_TEXT_SET_P3_ID, UI_ACTION_SET_P3)
UI_MENU_ACTIONCOMMAND_T(ui_menu_calculate_leveling, UI_TEXT_CALCULATE_LEVELING_ID, UI_ACTION_CALC_LEVEL)
#define UI_MENU_LEVEL {UI_MENU_ADDCONDBACK &ui_menu_set_p1,&ui_menu_set_p2,&ui_menu_set_p3,&ui_menu_calculate_leveling UI_SPEED_X UI_SPEED_Y UI_SPEED_Z}
UI_MENU(ui_menu_level, UI_MENU_LEVEL, 4 + 3 * UI_SPEED + UI_MENU_BACKCNT)
#endif

// **** Extruder menu
#if NUM_EXTRUDER > 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp0, UI_TEXT_EXTR0_TEMP_ID, UI_ACTION_EXTRUDER0_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp0_printing, UI_TEXT_EXTR0_TEMP_ID, UI_ACTION_EXTRUDER0_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP0_PRINTING ,&ui_menu_ext_temp0_printing
#define UI_TEMP0_CNT 1
#else
#define UI_TEMP0_PRINTING
#define UI_TEMP0_CNT 0
#endif
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp1, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp1_printing, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP1_PRINTING ,&ui_menu_ext_temp1_printing
#define UI_TEMP1_CNT 1
#else
#define UI_TEMP1_PRINTING
#define UI_TEMP1_CNT 0
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp2, UI_TEXT_EXTR2_TEMP_ID, UI_ACTION_EXTRUDER2_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp2_printing, UI_TEXT_EXTR2_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP2_PRINTING ,&ui_menu_ext_temp2_printing
#define UI_TEMP2_CNT 1
#else
#define UI_TEMP2_PRINTING
#define UI_TEMP2_CNT 0
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp3, UI_TEXT_EXTR3_TEMP_ID, UI_ACTION_EXTRUDER3_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp3_printing, UI_TEXT_EXTR3_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP3_PRINTING ,&ui_menu_ext_temp3_printing
#define UI_TEMP3_CNT 1
#else
#define UI_TEMP3_PRINTING
#define UI_TEMP3_CNT 0
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp4, UI_TEXT_EXTR4_TEMP_ID, UI_ACTION_EXTRUDER4_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp4_printing, UI_TEXT_EXTR4_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP4_PRINTING ,&ui_menu_ext_temp4_printing
#define UI_TEMP4_CNT 1
#else
#define UI_TEMP4_PRINTING
#define UI_TEMP4_CNT 0
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp5, UI_TEXT_EXTR5_TEMP_ID, UI_ACTION_EXTRUDER5_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp5_printing, UI_TEXT_EXTR5_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP5_PRINTING ,&ui_menu_ext_temp5_printing
#define UI_TEMP5_CNT 1
#else
#define UI_TEMP5_PRINTING
#define UI_TEMP5_CNT 0
#endif
UI_MENU_CHANGEACTION_T(ui_menu_bed_temp, UI_TEXT_BED_TEMP_ID, UI_ACTION_HEATED_BED_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_bed_temp_printing, UI_TEXT_BED_TEMP_ID, UI_ACTION_HEATED_BED_TEMP,MENU_MODE_PRINTING,0)
#if HAVE_HEATED_BED
#define UI_BED_TEMP_PRINTING ,&ui_menu_bed_temp_printing
#else
#define UI_BED_TEMP_PRINTING
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel0, UI_TEXT_EXTR0_SELECT_ID, UI_ACTION_SELECT_EXTRUDER0)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel1, UI_TEXT_EXTR1_SELECT_ID, UI_ACTION_SELECT_EXTRUDER1)
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel2, UI_TEXT_EXTR2_SELECT_ID, UI_ACTION_SELECT_EXTRUDER2)
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel3, UI_TEXT_EXTR3_SELECT_ID, UI_ACTION_SELECT_EXTRUDER3)
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel4, UI_TEXT_EXTR4_SELECT_ID, UI_ACTION_SELECT_EXTRUDER4)
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel5, UI_TEXT_EXTR5_SELECT_ID, UI_ACTION_SELECT_EXTRUDER5)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off0, UI_TEXT_EXTR0_OFF_ID, UI_ACTION_EXTRUDER0_OFF)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off1, UI_TEXT_EXTR1_OFF_ID, UI_ACTION_EXTRUDER1_OFF)
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off2, UI_TEXT_EXTR2_OFF_ID, UI_ACTION_EXTRUDER2_OFF)
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off3, UI_TEXT_EXTR3_OFF_ID, UI_ACTION_EXTRUDER3_OFF)
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off4, UI_TEXT_EXTR4_OFF_ID, UI_ACTION_EXTRUDER4_OFF)
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off5, UI_TEXT_EXTR5_OFF_ID, UI_ACTION_EXTRUDER5_OFF)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_origin, UI_TEXT_EXTR_ORIGIN_ID, UI_ACTION_RESET_EXTRUDER)
#if FEATURE_DITTO_PRINTING
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto0, UI_TEXT_DITTO_0_ID, UI_DITTO_0)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto1, UI_TEXT_DITTO_1_ID, UI_DITTO_1)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto2, UI_TEXT_DITTO_2_ID, UI_DITTO_2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto3, UI_TEXT_DITTO_3_ID, UI_DITTO_3)
#if NUM_EXTRUDER == 3
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1,&ui_menu_ext_ditto2
#define UI_DITTO_COMMANDS_COUNT 3
#elif NUM_EXTRUDER == 4
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1,&ui_menu_ext_ditto2,&ui_menu_ext_ditto3
#define UI_DITTO_COMMANDS_COUNT 4
#else
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1
#define UI_DITTO_COMMANDS_COUNT 2
#endif
#else
#define UI_DITTO_COMMANDS
#define UI_DITTO_COMMANDS_COUNT 0
#endif
#if MIXING_EXTRUDER || NUM_EXTRUDER == 1
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_off0,
#define UI_MENU_EXTCNT 2
#define UI_MENU_EXTSEL
#define UI_MENU_EXTSEL_CNT 0
#elif NUM_EXTRUDER == 2
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_sel0,&ui_menu_ext_sel1,
#define UI_MENU_EXTCNT 6
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1
#define UI_MENU_EXTSEL_CNT 2
#elif NUM_EXTRUDER == 3
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,
#define UI_MENU_EXTCNT 9
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2
#define UI_MENU_EXTSEL_CNT 3
#elif NUM_EXTRUDER == 4
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,
#define UI_MENU_EXTCNT 12
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3
#define UI_MENU_EXTSEL_CNT 4
#elif NUM_EXTRUDER == 5
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_temp4,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_off4,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,
#define UI_MENU_EXTCNT 15
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4
#define UI_MENU_EXTSEL_CNT 5
#elif NUM_EXTRUDER == 6
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_temp4,&ui_menu_ext_temp5,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_off4,&ui_menu_ext_off5,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel5,
#define UI_MENU_EXTCNT 18
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel4
#define UI_MENU_EXTSEL_CNT 6
#elif NUM_EXTRUDER == 0
#define UI_MENU_EXTCOND
#define UI_MENU_EXTCNT 0
#define UI_MENU_EXTSEL
#define UI_MENU_EXTSEL_CNT 0
#endif
#if HAVE_HEATED_BED
#define UI_MENU_BEDCOND &ui_menu_bed_temp,
#define UI_MENU_BEDCNT 1
#else
#define UI_MENU_BEDCOND
#define UI_MENU_BEDCNT 0
#endif

#define UI_MENU_EXTRUDER {UI_MENU_ADDCONDBACK UI_MENU_BEDCOND UI_MENU_EXTCOND &ui_menu_go_epos,&ui_menu_ext_origin UI_DITTO_COMMANDS}
UI_MENU(ui_menu_extruder, UI_MENU_EXTRUDER, UI_MENU_BACKCNT + UI_MENU_BEDCNT + UI_MENU_EXTCNT + 2 + UI_DITTO_COMMANDS_COUNT)

// **** SD card menu

// **** Fan menu

#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
UI_MENU_CHANGEACTION_T(ui_menu_fan_fanspeed, UI_TEXT_ACTION_FANSPEED_ID, UI_ACTION_FANSPEED)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_fan_fanspeed_printing, UI_TEXT_ACTION_FANSPEED_ID, UI_ACTION_FANSPEED,MENU_MODE_PRINTING,0)
#define UI_FANSPEED_PRINTING ,&ui_menu_fan_fanspeed_printing
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_fan_off, UI_TEXT_FAN_OFF_ID, UI_ACTION_FAN_OFF, MENU_MODE_FAN_RUNNING, 0)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_25, UI_TEXT_FAN_25_ID, UI_ACTION_FAN_25)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_50, UI_TEXT_FAN_50_ID, UI_ACTION_FAN_50)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_75, UI_TEXT_FAN_75_ID, UI_ACTION_FAN_75)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_full, UI_TEXT_FAN_FULL_ID, UI_ACTION_FAN_FULL)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_ignoreM106, UI_TEXT_IGNORE_M106_ID, UI_ACTION_IGNORE_M106)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_fan_ignoreM106_printing, UI_TEXT_IGNORE_M106_ID, UI_ACTION_IGNORE_M106,MENU_MODE_PRINTING,0)
#define UI_IGNORE106_ENTRY ,&ui_menu_fan_ignoreM106
#define UI_IGNORE106_ENTRY_PRINTING ,&ui_menu_fan_ignoreM106_printing
#define UI_MENU_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_fanspeed,&ui_menu_fan_off,&ui_menu_fan_25,&ui_menu_fan_50,&ui_menu_fan_75,&ui_menu_fan_full,&ui_menu_fan_ignoreM106}
UI_MENU(ui_menu_fan, UI_MENU_FAN, 7 + UI_MENU_BACKCNT)
UI_MENU_SUBMENU_T(ui_menu_fan_sub, UI_TEXT_FANSPEED_ID, ui_menu_fan)
#define UI_MENU_FAN_COND &ui_menu_fan_sub,
#define UI_FANSPEED ,&ui_menu_fan_fanspeed
#define UI_MENU_FAN_CNT 1
#else
#define UI_MENU_FAN_COND
#define UI_FANSPEED
#define UI_MENU_FAN_CNT 0
#define UI_IGNORE106_ENTRY
#define UI_IGNORE106_ENTRY_PRINTING
#define UI_FANSPEED_PRINTING
#endif

#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_fan2_fanspeed_printing,UI_TEXT_ACTION_FAN2SPEED_ID, UI_ACTION_FAN2SPEED,MENU_MODE_PRINTING,0)
UI_MENU_CHANGEACTION_T(ui_menu_fan2_fanspeed,UI_TEXT_ACTION_FAN2SPEED_ID, UI_ACTION_FAN2SPEED)
#define UI_FAN2SPEED_PRINTING ,&ui_menu_fan2_fanspeed_printing
#define UI_FAN2SPEED ,&ui_menu_fan2_fanspeed
#define UI_MENU_FAN2_CNT 1
#else
#define UI_MENU_FAN2_CNT 0
#define UI_FAN2SPEED_PRINTING
#define UI_FAN2SPEED
#endif


// **** Quick menu
#if PS_ON_PIN > -1
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_power, UI_TEXT_POWER_ID, UI_ACTION_POWER)
#define MENU_PSON_COUNT 1
#define MENU_PSON_ENTRY ,&ui_menu_quick_power
#else
#define MENU_PSON_COUNT 0
#define MENU_PSON_ENTRY
#endif
#if CASE_LIGHTS_PIN >= 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_light, UI_TEXT_LIGHTS_ONOFF_ID, UI_ACTION_LIGHTS_ONOFF)
#define UI_TOOGLE_LIGHT_ENTRY ,&ui_menu_toggle_light
#define UI_TOGGLE_LIGHT_COUNT 1
#else
#define UI_TOOGLE_LIGHT_ENTRY
#define UI_TOGGLE_LIGHT_COUNT 0
#endif
#if NUM_EXTRUDER > 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_pla, UI_TEXT_PREHEAT_SINGLE_ID, UI_ACTION_PREHEAT_SINGLE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_abs, UI_TEXT_PREHEAT_ALL_ID, UI_ACTION_PREHEAT_ALL)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_cooldown, UI_TEXT_COOLDOWN_ID, UI_ACTION_COOLDOWN)
#define UI_MENU_PREHEAT_COND ,&ui_menu_quick_preheat_pla, &ui_menu_quick_preheat_abs, &ui_menu_quick_cooldown
#define UI_MENU_PREHEAT_CNT 3
#else
#define UI_MENU_PREHEAT_COND 
#define UI_MENU_PREHEAT_CNT 0
#endif
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_origin, UI_TEXT_SET_TO_ORIGIN_ID, UI_ACTION_SET_ORIGIN, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_stopstepper, UI_TEXT_DISABLE_STEPPER_ID, UI_ACTION_DISABLE_STEPPER, 0, MENU_MODE_PRINTING)
#if FEATURE_BABYSTEPPING
UI_MENU_CHANGEACTION_T(ui_menu_quick_zbaby, UI_TEXT_Z_BABYSTEPPING_ID, UI_ACTION_Z_BABYSTEPS)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_zbaby_printing, UI_TEXT_Z_BABYSTEPPING_ID, UI_ACTION_Z_BABYSTEPS,MENU_MODE_PRINTING,0)
#define BABY_CNT 1
#define BABY_ENTRY ,&ui_menu_quick_zbaby
#define BABY_ENTRY_PRINTING ,&ui_menu_quick_zbaby_printing
#else
#define BABY_CNT 0
#define BABY_ENTRY
#define BABY_ENTRY_PRINTING
#endif
UI_MENU_CHANGEACTION_T(ui_menu_quick_speedmultiply, UI_TEXT_SPEED_MULTIPLY_ID, UI_ACTION_FEEDRATE_MULTIPLY)
#if NUM_EXTRUDER > 0
UI_MENU_CHANGEACTION_T(ui_menu_quick_flowmultiply, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_flowmultiply_printing, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY,MENU_MODE_PRINTING,0)
#define UI_MENU_FLOWCONF_COND ,&ui_menu_quick_flowmultiply
#define UI_MENU_FLOWCONF_CNT 1
#define UI_MENU_FLOWPRINT_COND ,&ui_menu_quick_flowmultiply_printing
#define UI_MENU_FLOWPRINT_CNT 1
#else
#define UI_MENU_FLOWCONF_COND
#define UI_MENU_FLOWCONF_CNT 0
#define UI_MENU_FLOWPRINT_COND
#define UI_MENU_FLOWPRINT_CNT 0
#endif
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_speedmultiply_printing, UI_TEXT_SPEED_MULTIPLY_ID, UI_ACTION_FEEDRATE_MULTIPLY,MENU_MODE_PRINTING,0)
#ifdef DEBUG_PRINT
UI_MENU_ACTIONCOMMAND(ui_menu_quick_debug, "Write Debug", UI_ACTION_WRITE_DEBUG)
#define DEBUG_PRINT_COUNT 1
#define DEBUG_PRINT_EXTRA ,&ui_menu_quick_debug
#else
#define DEBUG_PRINT_COUNT 0
#define DEBUG_PRINT_EXTRA
#endif
#if FEATURE_RETRACTION
/* * Extruder 1
O Extruder 2
Temp. 170/180
Continue
Close */
UI_MENU_HEADLINE_T(ui_menu_chf_head,UI_TEXT_CHANGE_FILAMENT_ID)
UI_MENU_CHANGEACTION_T(ui_menu_chf_temp,UI_TEXT_CUR_TEMP_ID,UI_ACTION_EXTRUDER_TEMP)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_continue,UI_TEXT_CONTINUE_ID,UI_ACTION_WIZARD_FILAMENTCHANGE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_close,UI_TEXT_CLOSE_ID,UI_ACTION_BACK)
#define UI_MENU_CHF_ITEMS {&ui_menu_chf_head UI_MENU_EXTSEL ,&ui_menu_chf_temp,&ui_menu_chf_continue,&ui_menu_chf_close}
UI_STICKYMENU(ui_menu_chf,UI_MENU_CHF_ITEMS,4+UI_MENU_EXTSEL_CNT) 
UI_MENU_SUBMENU_T(ui_menu_quick_changefil,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_chf)   
UI_MENU_SUBMENU_FILTER_T(ui_menu_quick_changefil_printing,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_chf,MENU_MODE_PRINTING,0)
//UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_changefil, UI_TEXT_CHANGE_FILAMENT_ID, UI_ACTION_WIZARD_FILAMENTCHANGE)
//UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_changefil_printing, UI_TEXT_CHANGE_FILAMENT_ID, UI_ACTION_WIZARD_FILAMENTCHANGE,MENU_MODE_PRINTING,0)
#define UI_CHANGE_FIL_CNT 1
#define UI_CHANGE_FIL_ENT ,&ui_menu_quick_changefil
#define UI_CHANGE_FIL_ENT_PRINTING ,&ui_menu_quick_changefil_printing
#else
#define UI_CHANGE_FIL_CNT 0
#define UI_CHANGE_FIL_ENT
#define UI_CHANGE_FIL_ENT_PRINTING
#endif

#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_menu_home_all BABY_ENTRY ,&ui_menu_quick_speedmultiply UI_MENU_FLOWCONF_COND \
    UI_TOOGLE_LIGHT_ENTRY UI_FANSPEED UI_FAN2SPEED UI_CHANGE_FIL_ENT UI_MENU_PREHEAT_COND ,&ui_menu_quick_origin, \
    &ui_menu_quick_stopstepper MENU_PSON_ENTRY DEBUG_PRINT_EXTRA}
UI_MENU(ui_menu_quick, UI_MENU_QUICK, 4 + BABY_CNT + UI_MENU_BACKCNT + MENU_PSON_COUNT + DEBUG_PRINT_COUNT + UI_MENU_FLOWCONF_CNT + UI_TOGGLE_LIGHT_COUNT + UI_CHANGE_FIL_CNT + UI_MENU_PREHEAT_CNT + UI_MENU_FAN_CNT + UI_MENU_FAN2_CNT)

// **** Bed Coating Menu

#if UI_BED_COATING
UI_MENU_ACTION2_T(ui_menu_nocoating_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_NOCOATING_ID)
UI_MENU_ACTION2_T(ui_menu_buildtak_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_BUILDTAK_ID)
UI_MENU_ACTION2_T(ui_menu_kapton_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_KAPTON_ID)
UI_MENU_ACTION2_T(ui_menu_bluetape_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_BLUETAPE_ID)
UI_MENU_ACTION2_T(ui_menu_pettape_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_PETTAPE_ID)
UI_MENU_ACTION2_T(ui_menu_gluestick_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_GLUESTICK_ID)
UI_MENU_ACTION2_T(ui_menu_coating_custom, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_COATING_THICKNESS_ID)

UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_nocoating, UI_TEXT_NOCOATING_ID, UI_ACTION_NOCOATING, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_buildtak, UI_TEXT_BUILDTAK_ID, UI_ACTION_BUILDTAK, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_kapton, UI_TEXT_KAPTON_ID, UI_ACTION_KAPTON, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_bluetape, UI_TEXT_BLUETAPE_ID, UI_ACTION_BLUETAPE, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_pettape, UI_TEXT_PETTAPE_ID, UI_ACTION_PETTAPE, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_gluestick, UI_TEXT_GLUESTICK_ID, UI_ACTION_GLUESTICK, 0, MENU_MODE_PRINTING)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_adjust_custom, UI_TEXT_COATING_CUSTOM_ID, UI_ACTION_COATING_CUSTOM, 0, MENU_MODE_PRINTING)
#define UI_MENU_ADJUST {UI_MENU_ADDCONDBACK &ui_menu_adjust_nocoating,&ui_menu_adjust_buildtak,&ui_menu_adjust_kapton,&ui_menu_adjust_bluetape,&ui_menu_adjust_pettape,&ui_menu_adjust_gluestick,&ui_menu_adjust_custom}
UI_MENU(ui_menu_adjust, UI_MENU_ADJUST, 7 + UI_MENU_BACKCNT)
#define UI_MENU_COATING_CNT 1
#define UI_MENU_COATING_COND &ui_menu_prepare,
UI_MENU_SUBMENU_FILTER_T(ui_menu_prepare, UI_TEXT_BED_COATING_ID, ui_menu_adjust, 0, MENU_MODE_PRINTING)

#else
#define UI_MENU_COATING_CNT 0
#define UI_MENU_COATING_COND
#endif


UI_MENU_HEADLINE_T(ui_menu_askstop_head, UI_TEXT_STOP_PRINT_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_sd_askstop_no, UI_TEXT_NO_ID, UI_ACTION_BACK)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_askstop_yes,      UI_TEXT_YES_ID,     UI_ACTION_STOP_CONFIRMED | UI_ACTION_TOPMENU, MENU_MODE_PRINTING, 0)
#if UI_ROWS >= 5
#define UI_MENU_ASKSTOP {&ui_menu_empty,&ui_menu_askstop_head,&ui_menu_empty,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 5)
#elif UI_ROWS == 4
#define UI_MENU_ASKSTOP {&ui_menu_askstop_head,&ui_menu_empty,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 4)
#else
#define UI_MENU_ASKSTOP {&ui_menu_askstop_head,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 3)
#endif

// **** SD card menu

#if SDSUPPORT


#define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
UI_MENU_FILESELECT(ui_menu_sd_fileselector, UI_MENU_SD_FILESELECTOR, 1)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_printfile, UI_TEXT_PRINT_FILE_ID,     UI_ACTION_SD_PRINT,    MENU_MODE_SD_MOUNTED,  MENU_MODE_SD_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_pause,     UI_TEXT_PAUSE_PRINT_ID,    UI_ACTION_SD_PAUSE,    MENU_MODE_SD_PRINTING, MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_continue,  UI_TEXT_CONTINUE_PRINT_ID, UI_ACTION_SD_CONTINUE, MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED,   0)
// two versions of stop. Second is with security question since pausing can trigger stop with bad luck!
//UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_stop,      UI_TEXT_STOP_PRINT_ID,     UI_ACTION_SD_STOP,     MENU_MODE_SD_PRINTING, 0)
UI_MENU_SUBMENU_FILTER_T(ui_menu_sd_stop, UI_TEXT_STOP_PRINT_ID, ui_menu_askstop, MENU_MODE_SD_PRINTING, 0 )
#define SD_PRINTFILE_ENTRY ,&ui_menu_sd_printfile
#define SD_PRINTFILE_ENTRY_CNT 1
#if SDCARDDETECT > -1
#define UI_MOUNT_CNT 0
#define UI_MOUNT_CMD
#else
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_unmount, UI_TEXT_UNMOUNT_CARD_ID, UI_ACTION_SD_UNMOUNT, MENU_MODE_SD_MOUNTED, 0)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_mount, UI_TEXT_MOUNT_CARD_ID, UI_ACTION_SD_MOUNT, 0, MENU_MODE_SD_MOUNTED)
#define UI_MOUNT_CNT 2
#define UI_MOUNT_CMD ,&ui_menu_sd_unmount,&ui_menu_sd_mount
#endif
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_delete, UI_TEXT_DELETE_FILE_ID, UI_ACTION_SD_DELETE, MENU_MODE_SD_MOUNTED, MENU_MODE_SD_PRINTING)
#define UI_MENU_SD {UI_MENU_ADDCONDBACK &ui_menu_sd_printfile,&ui_menu_sd_pause,&ui_menu_sd_continue,&ui_menu_sd_stop UI_MOUNT_CMD ,&ui_menu_sd_delete}
UI_MENU(ui_menu_sd, UI_MENU_SD, UI_MENU_BACKCNT + 5 + UI_MOUNT_CNT)
UI_MENU_SUBMENU_T(ui_menu_sd_sub, UI_TEXT_SD_CARD_ID, ui_menu_sd)

#define UI_MENU_SD_COND &ui_menu_sd_sub,
#define UI_MENU_SD_CNT 1
#else
#define UI_MENU_SD_COND
#define UI_MENU_SD_CNT 0
#define SD_PRINTFILE_ENTRY
#define SD_PRINTFILE_ENTRY_CNT 0
#endif


// **** Debugging menu
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_echo,    UI_TEXT_DBG_ECHO_ID,    UI_ACTION_DEBUG_ECHO)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_info,    UI_TEXT_DBG_INFO_ID,    UI_ACTION_DEBUG_INFO)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_error,   UI_TEXT_DBG_ERROR_ID,   UI_ACTION_DEBUG_ERROR)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_dryrun,  UI_TEXT_DBG_DRYRUN_ID,  UI_ACTION_DEBUG_DRYRUN)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_endstop, UI_TEXT_DBG_ENDSTOP_ID, UI_ACTION_DEBUG_ENDSTOP)

#define UI_MENU_DEBUGGING {UI_MENU_ADDCONDBACK &ui_menu_debug_echo,&ui_menu_debug_info,&ui_menu_debug_error,&ui_menu_debug_dryrun,&ui_menu_debug_endstop}
UI_MENU(ui_menu_debugging, UI_MENU_DEBUGGING, 5 + UI_MENU_BACKCNT)

// **** Acceleration settings
#if DRIVE_SYSTEM != DELTA
UI_MENU_CHANGEACTION_T(ui_menu_accel_printx,  UI_TEXT_PRINT_X_ID, UI_ACTION_PRINT_ACCEL_X)
UI_MENU_CHANGEACTION_T(ui_menu_accel_printy,  UI_TEXT_PRINT_Y_ID, UI_ACTION_PRINT_ACCEL_Y)
UI_MENU_CHANGEACTION_T(ui_menu_accel_printz,  UI_TEXT_PRINT_Z_ID, UI_ACTION_PRINT_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelx, UI_TEXT_MOVE_X_ID, UI_ACTION_MOVE_ACCEL_X)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travely, UI_TEXT_MOVE_Y_ID, UI_ACTION_MOVE_ACCEL_Y)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelz, UI_TEXT_MOVE_Z_ID, UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_jerk,    UI_TEXT_JERK_ID, UI_ACTION_MAX_JERK)
UI_MENU_CHANGEACTION_T(ui_menu_accel_zjerk,   UI_TEXT_ZJERK_ID, UI_ACTION_MAX_ZJERK)
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printx,&ui_menu_accel_printy,&ui_menu_accel_printz,&ui_menu_accel_travelx,&ui_menu_accel_travely,&ui_menu_accel_travelz,&ui_menu_accel_jerk,&ui_menu_accel_zjerk}
UI_MENU(ui_menu_accel, UI_MENU_ACCEL, 8 + UI_MENU_BACKCNT)

// **** Feedrates
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxx,  UI_TEXT_FEED_MAX_X_ID,  UI_ACTION_MAX_FEEDRATE_X)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxy,  UI_TEXT_FEED_MAX_Y_ID,  UI_ACTION_MAX_FEEDRATE_Y)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxz,  UI_TEXT_FEED_MAX_Z_ID,  UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homex, UI_TEXT_FEED_HOME_X_ID, UI_ACTION_HOMING_FEEDRATE_X)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homey, UI_TEXT_FEED_HOME_Y_ID, UI_ACTION_HOMING_FEEDRATE_Y)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z_ID, UI_ACTION_HOMING_FEEDRATE_Z)
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxx,&ui_menu_feedrate_maxy,&ui_menu_feedrate_maxz,&ui_menu_feedrate_homex,&ui_menu_feedrate_homey,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate, UI_MENU_FEEDRATE, 6 + UI_MENU_BACKCNT)
#else
UI_MENU_CHANGEACTION_T(ui_menu_accel_printz, UI_TEXT_PRINT_Z_DELTA_ID, UI_ACTION_PRINT_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelz, UI_TEXT_MOVE_Z_DELTA_ID, UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_jerk, UI_TEXT_JERK_ID, UI_ACTION_MAX_JERK)
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printz,&ui_menu_accel_travelz,&ui_menu_accel_jerk}
UI_MENU(ui_menu_accel, UI_MENU_ACCEL, 3 + UI_MENU_BACKCNT)

// **** Feedrates
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxz, UI_TEXT_FEED_MAX_Z_DELTA_ID, UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z_DELTA_ID, UI_ACTION_HOMING_FEEDRATE_Z)
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxz,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate, UI_MENU_FEEDRATE, 2 + UI_MENU_BACKCNT)
#endif

// **** General configuration settings

UI_MENU_ACTION2_T(ui_menu_stepper2, UI_ACTION_STEPPER_INACTIVE, UI_TEXT_STEPPER_INACTIVE2A_ID, UI_TEXT_STEPPER_INACTIVE2B_ID)
UI_MENU_ACTION2_T(ui_menu_maxinactive2, UI_ACTION_MAX_INACTIVE, UI_TEXT_POWER_INACTIVE2A_ID, UI_TEXT_POWER_INACTIVE2B_ID)
UI_MENU_CHANGEACTION_T(ui_menu_general_baud, UI_TEXT_BAUDRATE_ID, UI_ACTION_BAUDRATE)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_stepper_inactive, UI_TEXT_STEPPER_INACTIVE_ID, ui_menu_stepper2)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_max_inactive, UI_TEXT_POWER_INACTIVE_ID, ui_menu_maxinactive2)
#if FEATURE_AUTOLEVEL
UI_MENU_ACTIONCOMMAND_T(ui_menu_autolevelbed,UI_TEXT_AUTOLEVEL_BED_ID,UI_ACTION_AUTOLEVEL);
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_autolevel, UI_TEXT_AUTOLEVEL_ONOFF_ID, UI_ACTION_AUTOLEVEL_ONOFF)
#define UI_TOOGLE_AUTOLEVEL_ENTRY ,&ui_menu_autolevelbed,&ui_menu_toggle_autolevel
#define UI_TOGGLE_AUTOLEVEL_COUNT 2
#else
#define UI_TOOGLE_AUTOLEVEL_ENTRY
#define UI_TOGGLE_AUTOLEVEL_COUNT 0
#endif
#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive UI_TOOGLE_AUTOLEVEL_ENTRY}
UI_MENU(ui_menu_general, UI_MENU_GENERAL, 3 + UI_MENU_BACKCNT + UI_TOGGLE_AUTOLEVEL_COUNT)

// **** Extruder configuration

UI_MENU_CHANGEACTION_T(ui_menu_cext_steps,          UI_TEXT_EXTR_STEPS_ID,              UI_ACTION_EXTR_STEPS)
UI_MENU_CHANGEACTION_T(ui_menu_cext_start_feedrate, UI_TEXT_EXTR_START_FEED_ID,         UI_ACTION_EXTR_START_FEEDRATE)
UI_MENU_CHANGEACTION_T(ui_menu_cext_max_feedrate,   UI_TEXT_EXTR_MAX_FEED_ID,           UI_ACTION_EXTR_MAX_FEEDRATE)
UI_MENU_CHANGEACTION_T(ui_menu_cext_acceleration,   UI_TEXT_EXTR_ACCEL_ID,              UI_ACTION_EXTR_ACCELERATION)
UI_MENU_CHANGEACTION_T(ui_menu_cext_watch_period,   UI_TEXT_EXTR_WATCH_ID,              UI_ACTION_EXTR_WATCH_PERIOD)
UI_MENU_CHANGEACTION_T(ui_menu_ext_wait_temp,       UI_TEXT_EXTR_WAIT_RETRACT_TEMP_ID,  UI_ACTION_EXTR_WAIT_RETRACT_TEMP)
UI_MENU_CHANGEACTION_T(ui_menu_ext_wait_units,      UI_TEXT_EXTR_WAIT_RETRACT_UNITS_ID, UI_ACTION_EXTR_WAIT_RETRACT_UNITS)
#define UI_MENU_ADV_CNT 0
#define UI_MENU_ADVANCE
#if USE_ADVANCE
#undef UI_MENU_ADV_CNT
#define UI_MENU_ADV_CNT 1
#undef UI_MENU_ADVANCE
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel
#if ENABLE_QUADRATIC_ADVANCE
#undef UI_MENU_ADV_CNT
#define UI_MENU_ADV_CNT 2
#undef UI_MENU_ADVANCE
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel,&ui_menu_cext_advancek
UI_MENU_CHANGEACTION_T(ui_menu_cext_advancek, UI_TEXT_EXTR_ADVANCE_K_ID, UI_ACTION_ADVANCE_K)
#endif
UI_MENU_CHANGEACTION_T(ui_menu_cext_advancel, UI_TEXT_EXTR_ADVANCE_L_ID, UI_ACTION_ADVANCE_L)
#endif
UI_MENU_CHANGEACTION_T(       ui_menu_cext_manager, UI_TEXT_EXTR_MANAGER_ID, UI_ACTION_EXTR_HEATMANAGER)
UI_MENU_CHANGEACTION_T(       ui_menu_cext_pmax,    UI_TEXT_EXTR_PMAX_ID,    UI_ACTION_PID_MAX)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_pgain,   UI_TEXT_EXTR_PGAIN_ID,   UI_ACTION_PID_PGAIN, MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_igain,   UI_TEXT_EXTR_IGAIN_ID,   UI_ACTION_PID_IGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dgain,   UI_TEXT_EXTR_DGAIN_ID,   UI_ACTION_PID_DGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmin,    UI_TEXT_EXTR_DMIN_ID,    UI_ACTION_DRIVE_MIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmax,    UI_TEXT_EXTR_DMAX_ID,    UI_ACTION_DRIVE_MAX,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_pgain_dt,   UI_TEXT_EXTR_DEADTIME_ID,   UI_ACTION_PID_PGAIN, MENU_MODE_DEADTIME, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmax_dt,    UI_TEXT_EXTR_DMAX_DT_ID,    UI_ACTION_DRIVE_MAX,  MENU_MODE_DEADTIME, 0)
#define UI_MENU_PIDCOND ,&ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax, &ui_menu_cext_pgain_dt,&ui_menu_cext_dmax_dt,&ui_menu_cext_pmax
#define UI_MENU_PIDCNT 9
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_zoffset, UI_TEXT_EXTR_ZOFF_ID, UI_ACTION_Z_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel5,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,&ui_menu_cext_zoffset,
#define UI_MENU_CONFEXTCNT 9
#elif NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_zoffset, UI_TEXT_EXTR_ZOFF_ID, UI_ACTION_Z_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,&ui_menu_cext_zoffset,
#define UI_MENU_CONFEXTCNT 8
#elif NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_zoffset, UI_TEXT_EXTR_ZOFF_ID, UI_ACTION_Z_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,&ui_menu_cext_zoffset,
#define UI_MENU_CONFEXTCNT 7
#elif NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_zoffset, UI_TEXT_EXTR_ZOFF_ID, UI_ACTION_Z_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,&ui_menu_cext_zoffset,
#define UI_MENU_CONFEXTCNT 6
#elif NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_zoffset, UI_TEXT_EXTR_ZOFF_ID, UI_ACTION_Z_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,&ui_menu_cext_zoffset,
#define UI_MENU_CONFEXTCNT 5
#else
#define UI_MENU_CONFEXTCOND
#define UI_MENU_CONFEXTCNT 0
#endif
#define UI_MENU_CEXTR {UI_MENU_ADDCONDBACK UI_MENU_CONFEXTCOND &ui_menu_cext_steps,&ui_menu_cext_start_feedrate,&ui_menu_cext_max_feedrate,&ui_menu_cext_acceleration,&ui_menu_cext_watch_period,&ui_menu_ext_wait_units,&ui_menu_ext_wait_temp UI_MENU_ADVANCE UI_MENU_PIDCOND}
UI_MENU(ui_menu_cextr, UI_MENU_CEXTR, 7 + UI_MENU_BACKCNT + UI_MENU_PIDCNT + UI_MENU_CONFEXTCNT + UI_MENU_ADV_CNT)

// HeatBed Configuration - use menu actions from extruder configuration
#if HAVE_HEATED_BED
#define UI_MENU_BEDCONF {UI_MENU_ADDCONDBACK &ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax,&ui_menu_cext_pgain_dt,&ui_menu_cext_pmax}
UI_MENU(ui_menu_bedconf, UI_MENU_BEDCONF, 8 + UI_MENU_BACKCNT)
#endif



// **** Configuration menu

UI_MENU_SUBMENU_T(ui_menu_conf_general, UI_TEXT_GENERAL_ID,      ui_menu_general)
UI_MENU_SUBMENU_T(ui_menu_conf_accel,   UI_TEXT_ACCELERATION_ID, ui_menu_accel)
UI_MENU_SUBMENU_T(ui_menu_conf_feed,    UI_TEXT_FEEDRATE_ID,     ui_menu_feedrate)
#if NUM_EXTRUDER > 0
UI_MENU_SUBMENU_T(ui_menu_conf_extr,    UI_TEXT_EXTRUDER_ID,     ui_menu_cextr)
#define UI_MENU_EXTCONF_COND ,&ui_menu_conf_extr 
#define UI_MENU_EXTCONF_CNT 1
#else
#define UI_MENU_EXTCONF_COND
#define UI_MENU_EXTCONF_CNT 0
#endif
#if HAVE_HEATED_BED
UI_MENU_SUBMENU_T(ui_menu_conf_bed,    UI_TEXT_HEATING_BED_ID,  ui_menu_bedconf)
#define UI_MENU_BEDCONF_COND ,&ui_menu_conf_bed
#define UI_MENU_BEDCONF_CNT 1
#else
#define UI_MENU_BEDCONF_COND
#define UI_MENU_BEDCONF_CNT 0
#endif
#if EEPROM_MODE!=0
UI_MENU_ACTIONCOMMAND_T(ui_menu_conf_to_eeprom, UI_TEXT_STORE_TO_EEPROM_ID, UI_ACTION_STORE_EEPROM)
UI_MENU_ACTIONCOMMAND_T(ui_menu_conf_from_eeprom, UI_TEXT_LOAD_EEPROM_ID, UI_ACTION_LOAD_EEPROM)
#define UI_MENU_EEPROM_COND ,&ui_menu_conf_to_eeprom,&ui_menu_conf_from_eeprom
#define UI_MENU_EEPROM_CNT 2
UI_MENU_ACTION2_T(ui_menu_eeprom_saved,  UI_ACTION_DUMMY, UI_TEXT_EEPROM_STOREDA_ID, UI_TEXT_EEPROM_STOREDB_ID)
UI_MENU_ACTION2_T(ui_menu_eeprom_loaded, UI_ACTION_DUMMY, UI_TEXT_EEPROM_LOADEDA_ID, UI_TEXT_EEPROM_LOADEDB_ID)
#else
#define UI_MENU_EEPROM_COND
#define UI_MENU_EEPROM_CNT 0
#endif
#if SOFTWARE_LEVELING
#define UI_MENU_SL_COND ,&ui_menu_conf_level
#define UI_MENU_SL_CNT 1
UI_MENU_SUBMENU_T(ui_menu_conf_level, UI_TEXT_LEVEL_ID, ui_menu_level)
#else
#define UI_MENU_SL_COND
#define UI_MENU_SL_CNT 0
#endif
#if Z_HOME_DIR > 0
#define UI_MENU_DELTA_COND ,&ui_menu_conf_delta
#define UI_MENU_DELTA_CNT 1
UI_MENU_SUBMENU_T(ui_menu_conf_delta, UI_TEXT_ZCALIB_ID, ui_menu_delta)
#else
#define UI_MENU_DELTA_COND
#define UI_MENU_DELTA_CNT 0
#endif
#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK LANGMENU_ENTRY &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive  ,&ui_menu_conf_accel,&ui_menu_conf_feed UI_MENU_EXTCONF_COND UI_MENU_BEDCONF_COND UI_MENU_EEPROM_COND UI_MENU_DELTA_COND UI_MENU_SL_COND}
UI_MENU(ui_menu_configuration, UI_MENU_CONFIGURATION, UI_MENU_BACKCNT + LANGMENU_COUNT + UI_MENU_EEPROM_CNT + UI_MENU_EXTCONF_CNT + UI_MENU_BEDCONF_CNT + UI_MENU_DELTA_CNT + UI_MENU_SL_CNT + 5)

// **** Preheat menu ****

#if HAVE_HEATED_BED + NUM_EXTRUDER > 0
UI_MENU_HEADLINE_T(ui_menu_preheat_hdl,UI_TEXT_PREHEAT_TEMPS_ID)
#if HAVE_HEATED_BED
UI_MENU_CHANGEACTION_T(ui_menu_preheat_bed, UI_TEXT_PREHEAT_BED_ID, UI_ACTION_BED_PREHEAT)
#define UI_MENU_PREHEAT_BED ,&ui_menu_preheat_bed
#else
#define UI_MENU_PREHEAT_BED
#endif
#if NUM_EXTRUDER > 0
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext0, UI_TEXT_PREHEAT_E0_ID, UI_ACTION_EXT0_PREHEAT)
#define UI_MENU_PREHEAT_EXT0 ,&ui_menu_preheat_ext0
#else
#define UI_MENU_PREHEAT_EXT0
#endif
#if NUM_EXTRUDER > 1
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext1, UI_TEXT_PREHEAT_E1_ID, UI_ACTION_EXT1_PREHEAT)
#define UI_MENU_PREHEAT_EXT1 ,&ui_menu_preheat_ext1
#else
#define UI_MENU_PREHEAT_EXT1
#endif
#if NUM_EXTRUDER > 2
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext2, UI_TEXT_PREHEAT_E2_ID, UI_ACTION_EXT2_PREHEAT)
#define UI_MENU_PREHEAT_EXT2 ,&ui_menu_preheat_ext2
#else
#define UI_MENU_PREHEAT_EXT2
#endif
#if NUM_EXTRUDER > 3
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext3, UI_TEXT_PREHEAT_E3_ID, UI_ACTION_EXT3_PREHEAT)
#define UI_MENU_PREHEAT_EXT3 ,&ui_menu_preheat_ext3
#else
#define UI_MENU_PREHEAT_EXT3
#endif
#if NUM_EXTRUDER > 4
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext4, UI_TEXT_PREHEAT_E4_ID, UI_ACTION_EXT4_PREHEAT)
#define UI_MENU_PREHEAT_EXT4 ,&ui_menu_preheat_ext4
#else
#define UI_MENU_PREHEAT_EXT4
#endif
#if NUM_EXTRUDER > 5
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext5, UI_TEXT_PREHEAT_E5_ID, UI_ACTION_EXT5_PREHEAT)
#define UI_MENU_PREHEAT_EXT5 ,&ui_menu_preheat_ext5
#else
#define UI_MENU_PREHEAT_EXT5
#endif

#define UI_MENU_PREHEAT_SUB {&ui_menu_preheat_hdl UI_MENU_ADDCONDBACK_C UI_MENU_PREHEAT_BED UI_MENU_PREHEAT_EXT0 UI_MENU_PREHEAT_EXT1 UI_MENU_PREHEAT_EXT2 UI_MENU_PREHEAT_EXT3 UI_MENU_PREHEAT_EXT4 UI_MENU_PREHEAT_EXT5}
UI_MENU(ui_menu_preheat_sub, UI_MENU_PREHEAT_SUB, UI_MENU_BACKCNT + 1 + HAVE_HEATED_BED + NUM_EXTRUDER)
UI_MENU_SUBMENU_T(ui_menu_preheat,UI_TEXT_PREHEAT_TEMPS_ID,ui_menu_preheat_sub)

#define UI_MENU_PREHEAT ,&ui_menu_preheat
#define UI_MENU_PREHEAT_CNT2 1
#else
#define UI_MENU_PREHEAT 
#define UI_MENU_PREHEAT_CNT2 0
#endif

// **** Setup Menu

/*
Debug
Preheat
Z Calibrate
Auto level on/off
Distortion map
Distortion on/off
Ignore M106 Cmd
*/
UI_MENU_SUBMENU_T(ui_debug, UI_TEXT_DEBUGGING_ID, ui_menu_debugging)

#if DISTORTION_CORRECTION
UI_MENU_ACTIONCOMMAND_T(ui_menu_measure_distortion,UI_TEXT_MEASURE_DISTORTION_ID, UI_ACTION_MEASURE_DISTORTION)
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_distortion,UI_TEXT_DISTORTION_CORR_ID, UI_ACTION_TOGGLE_DISTORTION)
#define UI_DISTORTION_ENTRY ,&ui_menu_measure_distortion,&ui_menu_toggle_distortion
#define UI_DISTORTION_COUNT 2
#else
#define UI_DISTORTION_ENTRY
#define UI_DISTORTION_COUNT 0
#endif
#define UI_MENU_SETUP {UI_MENU_ADDCONDBACK &ui_debug UI_MENU_PREHEAT UI_TOOGLE_AUTOLEVEL_ENTRY UI_DISTORTION_ENTRY UI_CAL_ZHEIGHT_ENTRY UI_IGNORE106_ENTRY}
UI_MENU(ui_menu_setup, UI_MENU_SETUP, UI_MENU_BACKCNT + 1 + UI_MENU_PREHEAT_CNT2 + UI_TOGGLE_AUTOLEVEL_COUNT + UI_DISTORTION_COUNT + UI_MENU_FAN_CNT + UI_CAL_ZHEIGHT_CNT)
UI_MENU_SUBMENU_FILTER_T(ui_setup, UI_TEXT_SETUP_ID, ui_menu_setup,0,MENU_MODE_PRINTING)

// Stop print security question


// Main menu

// stop/pause/continue entries

UI_MENU_ACTIONCOMMAND_FILTER_T(ui_pause,UI_TEXT_PAUSE_PRINT_ID,UI_ACTION_PAUSE,MENU_MODE_PRINTING,MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_continue,UI_TEXT_CONTINUE_PRINT_ID,UI_ACTION_CONTINUE,MENU_MODE_PAUSED,0)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_stop,UI_TEXT_STOP_PRINT_ID,UI_ACTION_STOP,MENU_MODE_PRINTING,MENU_MODE_PAUSED)


UI_MENU_SUBMENU_FILTER_T(ui_menu_control, UI_TEXT_QUICK_SETTINGS_ID, ui_menu_quick,0,MENU_MODE_PRINTING)
UI_MENU_SUBMENU_FILTER_T(ui_menu_move, UI_TEXT_POSITION_ID, ui_menu_positions,0,MENU_MODE_PRINTING)
#if NUM_EXTRUDER > 0
UI_MENU_SUBMENU_FILTER_T(ui_menu_extrudercontrol, UI_TEXT_EXTRUDER_ID, ui_menu_extruder,0,MENU_MODE_PRINTING)
#define UI_MENU_EXTCONT_COND &ui_menu_extrudercontrol,
#define UI_MENU_EXTCONT_CNT 1
#else
#define UI_MENU_EXTCONT_COND
#define UI_MENU_EXTCONT_CNT 0
#endif

UI_MENU_SUBMENU_FILTER_T(ui_menu_settings, UI_TEXT_CONFIGURATION_ID, ui_menu_configuration,0,MENU_MODE_PRINTING)
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK &ui_menu_control ,&ui_stop,&ui_pause,&ui_continue \
    UI_TEMP0_PRINTING UI_TEMP1_PRINTING UI_TEMP2_PRINTING UI_TEMP3_PRINTING UI_TEMP4_PRINTING UI_TEMP5_PRINTING \
    UI_BED_TEMP_PRINTING ,&ui_menu_quick_speedmultiply_printing UI_CHANGE_FIL_ENT_PRINTING UI_MENU_FLOWPRINT_COND UI_FANSPEED_PRINTING UI_FAN2SPEED_PRINTING UI_IGNORE106_ENTRY_PRINTING BABY_ENTRY_PRINTING SD_PRINTFILE_ENTRY \
    ,&ui_menu_move, UI_MENU_EXTCONT_COND \
    UI_MENU_COATING_COND UI_MENU_SD_COND &ui_setup, &ui_menu_settings}
    
UI_MENU(ui_menu_main, UI_MENU_MAIN, 8 + UI_MENU_FLOWPRINT_CNT + UI_MENU_EXTCONT_CNT + UI_MENU_BACKCNT + UI_MENU_SD_CNT + SD_PRINTFILE_ENTRY_CNT + UI_MENU_COATING_CNT +UI_TEMP0_CNT+UI_TEMP1_CNT+UI_TEMP2_CNT+UI_TEMP3_CNT+\
    UI_TEMP4_CNT+UI_TEMP5_CNT+ BABY_CNT+HAVE_HEATED_BED+2*UI_MENU_FAN_CNT+UI_MENU_FAN2_CNT+UI_CHANGE_FIL_CNT)

/* Define menus accessible by action commands

  You can create up to 10 user menus which are accessible by the action commands UI_ACTION_SHOW_USERMENU1 until UI_ACTION_SHOW_USERMENU10
  You this the same way as with the menus above or you use one of the above menus. Then add a define like

  #define UI_USERMENU1 ui_menu_conf_feed

  which assigns the menu stored in ui_menu_conf_feed to the action UI_ACTION_SHOW_USERMENU1. Make sure only to change the numbers and not the name of the define.

  When do you need this? You might want a fast button to change the temperature. In the default menu you have no menu
  to change the temperature and view it the same time. So you need to make an action menu for this like:
  UI_MENU_ACTION4C(ui_menu_extrtemp,UI_ACTION_EXTRUDER0_TEMP,"Temp. 0  :%E0" cDEG,"","","");
  Then you assign this menu to a user menu:
  #define UI_USERMENU2 ui_menu_extrtemp

  Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperature screen and allows
  the change of temperature with the next/previous buttons.

*/
#endif // else custom menu
#endif
#endif // __UI_MENU_H
