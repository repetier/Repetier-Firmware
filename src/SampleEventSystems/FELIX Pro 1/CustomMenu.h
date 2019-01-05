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
#if !defined(_UI_CUSTOM_MENU_H)
#define _UI_CUSTOM_MENU_H


#if UI_DISPLAY_TYPE != NO_DISPLAY

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

// new submenues and entries

UI_MENU_HEADLINE_T(ui_empty,UI_TEXT_EMPTY_ID)
UI_MENU_HEADLINE_T(ui_exy1a,UI_CTEXT_XY_P1_1_ID)
UI_MENU_HEADLINE_T(ui_exy1b,UI_CTEXT_XY_P1_2_ID)
UI_MENU_HEADLINE_T(ui_exy1c,UI_CTEXT_XY_P1_3_ID)
UI_MENU_ACTIONCOMMAND_T(ui_exy1d,UI_TEXT_BACK_ID,UI_ACTION_XY1_BACK)
UI_MENU_ACTIONCOMMAND_T(ui_exy1e,UI_CTEXT_CONTINUE_ID,UI_ACTION_XY1_CONT)

#define UI_EXY1_ITEMS {&ui_exy1a,&ui_exy1b,&ui_exy1c,&ui_exy1d,&ui_exy1e}
UI_STICKYMENU(ui_exy1,UI_EXY1_ITEMS,5)

UI_MENU_HEADLINE_T(ui_exy2a,UI_TEXT_CLEARBED1_ID)
UI_MENU_HEADLINE_T(ui_exy2b,UI_TEXT_CLEARBED2_ID)
UI_MENU_HEADLINE_T(ui_exy2c,UI_TEXT_CLEARBED3_ID)
UI_MENU_ACTIONCOMMAND_T(ui_exy2d,UI_TEXT_BACK_ID,UI_ACTION_XY2_BACK)
UI_MENU_ACTIONCOMMAND_T(ui_exy2e,UI_CTEXT_START_PRINTING_ID,UI_ACTION_XY2_CONT)

#define UI_EXY2_ITEMS {&ui_exy2a,&ui_exy2b,&ui_exy2c,&ui_exy2d,&ui_exy2e}
UI_STICKYMENU(ui_exy2,UI_EXY2_ITEMS,5)

UI_MENU_HEADLINE_T(ui_exy3a,UI_CTEXT_XY_P3_1_ID)
UI_MENU_HEADLINE_T(ui_exy3b,UI_CTEXT_XY_P3_2_ID)
UI_MENU_CHANGEACTION_T(ui_exy3c,UI_CTEXT_XY_P3_X_ID,UI_ACTION_EXY_XOFFSET)
UI_MENU_CHANGEACTION_T(ui_exy3d,UI_CTEXT_XY_P3_Y_ID,UI_ACTION_EXY_YOFFSET)
UI_MENU_ACTIONCOMMAND_T(ui_exy3e,UI_CTEXT_CONTINUE_ID,UI_ACTION_EXY_STORE)

#define UI_EXY3_ITEMS {&ui_exy3a,&ui_exy3b,&ui_exy3c,&ui_exy3d,&ui_exy3e}
UI_STICKYMENU(ui_exy3,UI_EXY3_ITEMS,5)

UI_MENU_ACTIONCOMMAND_T(ui_calex,UI_CTEXT_CALIBRATE_EXTRUDERS_ID,UI_ACTION_CALEX)
UI_MENU_ACTIONCOMMAND_T(ui_calex_xy,UI_CTEXT_CALIBRATE_XY_ID,UI_ACTION_CALEX_XY)
UI_MENU_ACTIONCOMMAND(ui_calex_xyv2,"Calibrate XY w. Card",UI_ACTION_EXTRXY_V2)
//added by FELIX
//====================
#ifdef TEC4
#else
UI_MENU_ACTIONCOMMAND_T(ui_calex_z,UI_CTEXT_CALIBRATE_Z_ID,UI_ACTION_CALEX_Z)
#endif

#ifdef TEC4
#define UI_CALEXTR_SUBITEMS {&ui_menu_back, &ui_calex_xy}
UI_MENU(ui_calextr_sub,UI_CALEXTR_SUBITEMS,2)
#else
#define UI_CALEXTR_SUBITEMS {&ui_menu_back, &ui_calex_z, &ui_calex_xy, &ui_calex_xyv2}
UI_MENU(ui_calextr_sub,UI_CALEXTR_SUBITEMS,4)
#endif
//====================
//added by FELIX

UI_WIZARD4_T(ui_msg_printxycal, UI_ACTION_STATE,UI_CTEXT_PRINTXYCAL1_ID, UI_CTEXT_PRINTXYCAL2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4_T(ui_msg_extzcalib, UI_ACTION_STATE,UI_CTEXT_EXTZCAL1_ID, UI_CTEXT_EXTZCAL2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD5_T(ui_msg_clearbed_zcalib, UI_ACTION_CALEX_Z2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)

UI_MENU_ACTIONCOMMAND_T(ui_preheatcool1,UI_CTEXT_PREHEATCOOL_ID,UI_ACTION_PRECOOL1)
#if NUM_EXTRUDER > 1
UI_MENU_ACTIONCOMMAND_T(ui_preheatcool2,UI_CTEXT_PREHEATCOOL2_ID,UI_ACTION_PRECOOL2)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_removebed,UI_CTEXT_REMOVEBED_ID,UI_ACTION_REMOVEBED)

UI_WIZARD4_T(cui_msg_measuring, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_MEASURING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4(cui_msg_ext_xy_1, UI_ACTION_EXTRXY_V2_1,"Place card at the", "marked spot.", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_error, UI_ACTION_MESSAGE,"Calibration failed", "", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_success, UI_ACTION_MESSAGE,"Extruder Offset", "Calibration Finished.", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_info, UI_ACTION_STATE,"XY Offset Calibration", "   Measuring ...", "", "   Please wait")
UI_WIZARD4(cui_msg_autolevel_failed, UI_ACTION_MESSAGE,"Autoleveling failed", "Plate is not leveled!", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_exzautolevel_failed, UI_ACTION_MESSAGE,"Autoleveling failed", "Extruder offsets", " not leveled!", "     >>> OK <<<")

#ifndef TEC4
UI_WIZARD4(cui_msg_preparing, UI_ACTION_STATE,"", "   Preparing ...", "", "   Please wait")
#else
UI_WIZARD4(cui_msg_preparing, UI_ACTION_STATE,"", "   Warming up...", "", "   Please wait")
#endif              
UI_WIZARD4(cui_calib_zprobe_info, UI_ACTION_CZREFH_INFO, "   Place the Felix","   calibration card"," between the Nozzle","   and Build PLT.")
UI_WIZARD4(cui_calib_zprobe, UI_ACTION_CZREFH, "  Rotate the button","   until you feel","   slight friction","    on the card.")
UI_WIZARD4(cui_calib_zprobe_succ, UI_ACTION_CZREFH_SUCC, "     Build PLT."," Leveled Correctly","","     >>> OK <<<")
UI_WIZARD4(cui_calib_zprobe_dist, UI_ACTION_STATE, "", "   now measuring", "    buildsurface", "")
UI_MENU_HEADLINE(ui_halfw," Max. 15" cDEG)
UI_MENU_HEADLINE(ui_halfa," Turn back: %bb")
UI_MENU_HEADLINE(ui_halfb," Turn front: %ba")
UI_MENU_ACTIONCOMMAND(ui_halfc,"Measure Again",UI_ACTION_HALFAUTO_LEV2)
UI_MENU_ACTIONCOMMAND(ui_halfd,"Cancel",UI_ACTION_HALFAUTO_LEV3)

#define UI_HALF_ITEMS {&ui_halfw,&ui_halfa,&ui_halfb,&ui_halfc,&ui_halfd}
UI_STICKYMENU(ui_half_show,UI_HALF_ITEMS,5)


// Define precision for temperatures. With small displays only integer values fit.
#undef UI_TEMP_PRECISION
#define UI_TEMP_PRECISION 0

// Define precision for temperatures. With small displays only integer values fit.
#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 0
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

#if EEPROM_MODE != 0  && 0
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
           UI_TEXT_EMPTY_ID, // UI_TEXT_FLOW_MULTIPLY_ID,
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

UI_PAGE6_T(ui_page4, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4A_ID,UI_TEXT_EMPTY_ID,UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID)

/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {/*&ui_page1 UI_PRINTTIME_PAGES ,*/&ui_page4 UI_EXTRUDERS_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 1/*+UI_PRINTTIME_COUNT*/+UI_EXTRUDERS_PAGES_COUNT

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
UI_PAGE4_T(ui_page3, UI_TEXT_PAGE_EXTRUDER1_ID
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
#if EEPROM_MODE != 0
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
#define UI_PAGES {/*&ui_page1,*/ &ui_page2, &ui_page3 UI_PRINTTIME_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2+UI_PRINTTIME_COUNT
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
#define UI_PAGES {/* &ui_page1,*/ &ui_page2,&ui_page3}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2
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
#if LANGUAGE_RU_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_ru, "Russian", UI_ACTION_LANGUAGE_RU | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_RU &ui_menu_setlang_ru
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_RU ,&ui_menu_setlang_ru
#endif
#else
#define ADD_LANG_RU
#endif // LANGUAGE_RU_ACTIVE

#define UI_MENU_LANGUAGES {UI_MENU_ADDCONDBACK ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR ADD_LANG_RU}
#define UI_MENU_LANGUAGES_WIZ {ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR ADD_LANG_RU}
UI_MENU(ui_menu_languages, UI_MENU_LANGUAGES, UI_MENU_BACKCNT + LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE + LANGUAGE_RU_ACTIVE)
UI_MENU_SUBMENU_T(ui_menu_conf_lang, UI_TEXT_LANGUAGE_ID, ui_menu_languages)
UI_STICKYMENU(ui_menu_languages_wiz, UI_MENU_LANGUAGES_WIZ, LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE + LANGUAGE_RU_ACTIVE)
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
UI_WIZARD5_T(ui_msg_clearbed1, UI_ACTION_AUTOLEVEL2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD5_T(ui_msg_clearbed2, UI_ACTION_MEASURE_DISTORTION2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID,  UI_TEXT_OK_ID)

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

#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z UI_SPEED_X UI_SPEED_Y UI_SPEED_Z SERVOPOS_ENTRY}
UI_MENU(ui_menu_positions, UI_MENU_POSITIONS, 4 + 3 * UI_SPEED + UI_MENU_BACKCNT + SERVOPOS_COUNT)

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
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_measured_origin, UI_TEXT_SET_MEASURED_ORIGIN_ID, UI_ACTION_SET_MEASURED_ORIGIN)
#define UI_MENU_DELTA {UI_MENU_ADDCONDBACK &ui_menu_home_all UI_SPEED_Z_NOTEST,&ui_menu_set_measured_origin}
UI_MENU(ui_menu_delta, UI_MENU_DELTA, 2 + UI_SPEED + UI_MENU_BACKCNT)
#endif

// **** Bed leveling menu
#ifdef SOFTWARE_LEVELING
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
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp1, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp1_printing, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
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
#define UI_FANSPEED_PRINTING
#endif

#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
UI_MENU_CHANGEACTION_FILTER(ui_menu_fan2_fanspeed_printing,"Fan 2 speed:%FS%%%" /* UI_TEXT_ACTION_FANSPEED_ID*/, UI_ACTION_FAN2SPEED,MENU_MODE_PRINTING,0)
UI_MENU_CHANGEACTION(ui_menu_fan2_fanspeed,"Fan 2 speed:%FS%%%" /* UI_TEXT_ACTION_FANSPEED_ID*/, UI_ACTION_FAN2SPEED)
#define UI_FAN2SPEED_PRINTING ,&ui_menu_fan2_fanspeed_printing
#define UI_FAN2SPEED ,&ui_menu_fan2_fanspeed
#define UI_MENU_FAN2_CNT 1
#else
#define UI_MENU_FAN2_CNT 0
#define UI_FAN2SPEED_PRINTING
#define UI_FAN2SPEED
#endif


// **** General configuration settings

UI_MENU_ACTION2_T(ui_menu_stepper2, UI_ACTION_STEPPER_INACTIVE, UI_TEXT_STEPPER_INACTIVE2A_ID, UI_TEXT_STEPPER_INACTIVE2B_ID)
UI_MENU_ACTION2_T(ui_menu_maxinactive2, UI_ACTION_MAX_INACTIVE, UI_TEXT_POWER_INACTIVE2A_ID, UI_TEXT_POWER_INACTIVE2B_ID)
UI_MENU_CHANGEACTION_T(ui_menu_general_baud, UI_TEXT_BAUDRATE_ID, UI_ACTION_BAUDRATE)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_stepper_inactive, UI_TEXT_STEPPER_INACTIVE_ID, ui_menu_stepper2)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_max_inactive, UI_TEXT_POWER_INACTIVE_ID, ui_menu_maxinactive2)
#ifdef ZPROBE_HEIGHT_ROUTINE
UI_MENU_ACTIONCOMMAND(ui_menu_calib_probe,"Calibrate Z-Probe",UI_ACTION_START_CZREFH);
#define UI_CALIB_PROBE_ENTRY ,&ui_menu_calib_probe
#define UI_CALIB_PROBE_COUNT 1
#else
#define UI_CALIB_PROBE_ENTRY
#define UI_CALIB_PROBE_COUNT 0
#endif

#if FEATURE_AUTOLEVEL
#ifdef HALFAUTOMATIC_LEVELING
UI_MENU_ACTIONCOMMAND_T(ui_menu_autolevelbed,UI_TEXT_AUTOLEVEL_BED_ID,UI_ACTION_HALFAUTO_LEV);
#define UI_TOOGLE_AUTOLEVEL_ENTRY ,&ui_menu_autolevelbed
#define UI_TOGGLE_AUTOLEVEL_COUNT 1
#else
UI_MENU_ACTIONCOMMAND_T(ui_menu_autolevelbed,UI_TEXT_AUTOLEVEL_BED_ID,UI_ACTION_AUTOLEVEL);
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_autolevel, UI_TEXT_AUTOLEVEL_ONOFF_ID, UI_ACTION_AUTOLEVEL_ONOFF)
#define UI_TOOGLE_AUTOLEVEL_ENTRY ,&ui_menu_autolevelbed,&ui_menu_toggle_autolevel
#define UI_TOGGLE_AUTOLEVEL_COUNT 2
#endif
#else
#define UI_TOOGLE_AUTOLEVEL_ENTRY
#define UI_TOGGLE_AUTOLEVEL_COUNT 0
#endif
#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive UI_TOOGLE_AUTOLEVEL_ENTRY UI_CALIB_PROBE_ENTRY}
UI_MENU(ui_menu_general, UI_MENU_GENERAL, 3 + UI_MENU_BACKCNT + UI_TOGGLE_AUTOLEVEL_COUNT + UI_CALIB_PROBE_COUNT)


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
#define UI_MENU_ADJUST {UI_MENU_ADDCONDBACK &ui_menu_adjust_buildtak,&ui_menu_adjust_kapton,&ui_menu_adjust_custom}
UI_MENU(ui_menu_adjust, UI_MENU_ADJUST, 3 + UI_MENU_BACKCNT)
#define UI_MENU_COATING_CNT 1
#define UI_MENU_COATING_COND &ui_menu_prepare,
UI_MENU_SUBMENU_FILTER_T(ui_menu_prepare, UI_TEXT_BED_COATING_ID, ui_menu_adjust, 0, MENU_MODE_PRINTING)

#else
#define UI_MENU_COATING_CNT 0
#define UI_MENU_COATING_COND
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
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_pla, UI_TEXT_PREHEAT_SINGLE_ID, UI_ACTION_PREHEAT_SINGLE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_abs, UI_TEXT_PREHEAT_ALL_ID, UI_ACTION_PREHEAT_ALL)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_cooldown, UI_TEXT_COOLDOWN_ID, UI_ACTION_COOLDOWN)
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
UI_MENU_CHANGEACTION_T(ui_menu_quick_flowmultiply, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_speedmultiply_printing, UI_TEXT_SPEED_MULTIPLY_ID, UI_ACTION_FEEDRATE_MULTIPLY,MENU_MODE_PRINTING,0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_flowmultiply_printing, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY,MENU_MODE_PRINTING,0)
#ifdef DEBUG_PRINT
UI_MENU_ACTIONCOMMAND(ui_menu_quick_debug, "Write Debug", UI_ACTION_WRITE_DEBUG)
#define DEBUG_PRINT_COUNT 1
#define DEBUG_PRINT_EXTRA ,&ui_menu_quick_debug
#else
#define DEBUG_PRINT_COUNT 0
#define DEBUG_PRINT_EXTRA
#endif

// Change Filament level 1 - extruder selection

UI_MENU_HEADLINE_T(ui_menu_chf_head,UI_TEXT_CHANGE_FILAMENT_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_cf_sel1,UI_CTEXT_SELECT_EX1_ID,UI_ACTION_FC_SELECT1)
#if NUM_EXTRUDER == 1
#define UI_MENU_CF1 {&ui_menu_chf_head, UI_MENU_ADDCONDBACK &ui_menu_cf_sel1}
UI_MENU(ui_menu_chf, UI_MENU_CF1 , 2+UI_MENU_BACKCNT)
#else
UI_MENU_ACTIONCOMMAND_T(ui_menu_cf_sel2,UI_CTEXT_SELECT_EX2_ID,UI_ACTION_FC_SELECT2)
#define UI_MENU_CF1 {&ui_menu_chf_head, UI_MENU_ADDCONDBACK &ui_menu_cf_sel1,&ui_menu_cf_sel2}
UI_MENU(ui_menu_chf, UI_MENU_CF1 , 3+UI_MENU_BACKCNT)
#endif

// Change filament level 2 - material selection

UI_MENU_ACTIONCOMMAND_T(ui_menu_ch2_back, UI_TEXT_BACK_ID, UI_ACTION_FC_BACK1)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_current,"Current: %eIc" cDEG "C",UI_ACTION_WIZARD_FILAMENTCHANGE)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_pla,"PLA",UI_ACTION_FC_PLA)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_petg,"PETG",UI_ACTION_FC_PETG)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_pva,"PVA",UI_ACTION_FC_PVA)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_flex,"FLEX",UI_ACTION_FC_FLEX)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_abs,"ABS",UI_ACTION_FC_ABS)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_glass,"GLASS",UI_ACTION_FC_GLASS)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_wood,"WOOD",UI_ACTION_FC_WOOD)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ch2_custom,UI_TEXT_CUSTOM_ID,UI_ACTION_FC_CUSTOM)
#define UI_MENU_CH2_ITEMS {&ui_menu_chf_head,&ui_menu_ch2_back,\
&ui_menu_ch2_current,&ui_menu_ch2_pla,&ui_menu_ch2_petg,&ui_menu_ch2_pva,&ui_menu_ch2_flex,&ui_menu_ch2_abs,&ui_menu_ch2_glass,&ui_menu_ch2_wood,\
&ui_menu_ch2_custom}
UI_MENU(ui_menu_ch2,UI_MENU_CH2_ITEMS,11) 

// Change filament - custom temperature

UI_MENU_HEADLINE_T(ui_menu_ch2b_1,UI_CTEXT_SELECT_TEMP_ID)
UI_MENU_HEADLINE(ui_menu_ch2b_2,"%w0\002C")
UI_MENU_HEADLINE_T(ui_menu_ch2b_3,UI_TEXT_CLICK_DONE_ID)
#define UI_MENU_CH2A {&ui_menu_ch2b_1,&ui_menu_ch2b_2,&ui_menu_ch2b_3}
//UI_STICKYMENU(ui_menu_ch2a,UI_MENU_CH2A,3)
UI_WIZARD4_T(ui_menu_ch2a, UI_ACTION_FC_CUSTOM_SET, UI_CTEXT_SELECT_TEMP_ID, UI_CTEXT_WIZ_TEMP_ID, UI_TEXT_EMPTY_ID, UI_TEXT_CLICK_DONE_ID)


// Change filament heating ... info

UI_MENU_HEADLINE_T(ui_menu_ch3_1,UI_CTEXT_HEATINGUP1_ID)
UI_MENU_HEADLINE_T(ui_menu_ch3_2,UI_CTEXT_HEATINGUP2_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ch3_back, UI_TEXT_BACK_ID, UI_ACTION_FC_BACK1)
#define UI_MENU_CH3 {&ui_menu_ch3_1,&ui_menu_ch3_2,&ui_menu_ch3_back}
UI_STICKYMENU(ui_menu_ch3,UI_MENU_CH3,3)


#if FEATURE_RETRACTION
/* * Extruder 1
O Extruder 2
Temp. 170/180
Continue
Close */

UI_MENU_CHANGEACTION_T(ui_menu_chf_temp,UI_TEXT_CUR_TEMP_ID,UI_ACTION_EXTRUDER_TEMP)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pla,"PLA",UI_ACTION_SPH_PLA_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_petg,"PETG",UI_ACTION_SPH_PETG_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pva,"PVA",UI_ACTION_SPH_PVA_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_flex,"FLEX",UI_ACTION_SPH_FLEX_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_abs,"ABS",UI_ACTION_SPH_ABS_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_glass,"GLASS",UI_ACTION_SPH_GLASS_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_wood,"WOOD",UI_ACTION_SPH_WOOD_ACTIVE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_continue,UI_TEXT_CONTINUE_ID,UI_ACTION_WIZARD_FILAMENTCHANGE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_close,UI_TEXT_CLOSE_ID,UI_ACTION_BACK)
#define UI_MENU_CHF_ITEMS {&ui_menu_chf_head UI_MENU_EXTSEL ,&ui_menu_chf_temp,\
&ui_menu_chf_sph_pla,&ui_menu_chf_sph_petg,&ui_menu_chf_sph_pva,&ui_menu_chf_sph_flex,&ui_menu_chf_sph_abs,&ui_menu_chf_sph_glass,&ui_menu_chf_sph_wood,\
&ui_menu_chf_continue,&ui_menu_chf_close}
//UI_STICKYMENU(ui_menu_chf,UI_MENU_CHF_ITEMS,11+UI_MENU_EXTSEL_CNT) 
#if NUM_EXTRUDER == 1
UI_MENU_SUBMENU_T(ui_menu_quick_changefil,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2)   
UI_MENU_SUBMENU_FILTER_T(ui_menu_quick_changefil_printing,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2,MENU_MODE_PRINTING,0)
#else
UI_MENU_SUBMENU_T(ui_menu_quick_changefil,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_chf)   
UI_MENU_SUBMENU_FILTER_T(ui_menu_quick_changefil_printing,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2,MENU_MODE_PRINTING,0)
#endif
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
UI_MENU_SUBMENU_FILTER_T(ui_menu_move, UI_TEXT_POSITION_ID, ui_menu_positions,0,MENU_MODE_PRINTING)
#if NUM_EXTRUDER > 1
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_preheatcool2,&ui_removebed UI_CHANGE_FIL_ENT ,&ui_menu_autolevelbed UI_CALIB_PROBE_ENTRY, &ui_calex ,&ui_menu_move \
      , &ui_menu_quick_stopstepper UI_FANSPEED, &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_bed_temp}
#else
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_preheatcool1,&ui_removebed UI_CHANGE_FIL_ENT ,&ui_menu_autolevelbed UI_CALIB_PROBE_ENTRY, &ui_calex ,&ui_menu_move \
      , &ui_menu_quick_stopstepper UI_FANSPEED, &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_bed_temp}
#endif      
UI_MENU(ui_menu_quick, UI_MENU_QUICK, 10 + UI_MENU_BACKCNT + UI_CHANGE_FIL_CNT+ UI_CALIB_PROBE_COUNT)

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

// Dummy print menu to show missing card
UI_MENU_HEADLINE(ui_menus_sd_pinsert, "Please insert SD card")
#define UI_MENU_SD_NOCARD {UI_MENU_ADDCONDBACK &ui_menus_sd_pinsert}
UI_MENU(ui_menu_sd_nocard, UI_MENU_SD_NOCARD, UI_MENU_BACKCNT + 1)
UI_MENU_SUBMENU_FILTER_T(ui_menu_sd_printfile2, UI_TEXT_PRINT_FILE_ID,     ui_menu_sd_nocard,    0, MENU_MODE_SD_MOUNTED + MENU_MODE_SD_PRINTING)

#define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
UI_MENU_FILESELECT(ui_menu_sd_fileselector, UI_MENU_SD_FILESELECTOR, 1)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_printfile, UI_TEXT_PRINT_FILE_ID,     UI_ACTION_SD_PRINT,    MENU_MODE_SD_MOUNTED,  MENU_MODE_SD_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_pause,     UI_TEXT_PAUSE_PRINT_ID,    UI_ACTION_SD_PAUSE,    MENU_MODE_SD_PRINTING, MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_continue,  UI_TEXT_CONTINUE_PRINT_ID, UI_ACTION_SD_CONTINUE, MENU_MODE_SD_PRINTING+MENU_MODE_PAUSED,   0)
// two versions of stop. Second is with security question since pausing can trigger stop with bad luck!
//UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_stop,      UI_TEXT_STOP_PRINT_ID,     UI_ACTION_SD_STOP,     MENU_MODE_SD_PRINTING, 0)
UI_MENU_SUBMENU_FILTER_T(ui_menu_sd_stop, UI_TEXT_STOP_PRINT_ID, ui_menu_askstop, MENU_MODE_SD_PRINTING, 0 )
#define SD_PRINTFILE_ENTRY &ui_menu_sd_printfile,&ui_menu_sd_printfile2,
#define SD_PRINTFILE_ENTRY_CNT 2
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

// ***** Info

UI_MENU_HEADLINE_T(ui_info1, UI_CTEXT_TOT_TIME_ID)
UI_MENU_HEADLINE_T(ui_info2, UI_CTEXT_TOT_FILAMENT_ID)
#define UI_MENU_INFO {&ui_info1,&ui_info2,&ui_menu_back}
UI_MENU(ui_menu_info, UI_MENU_INFO, 3)
UI_MENU_SUBMENU_T(ui_menu_sub_info, UI_CTEXT_INFO_ID, ui_menu_info)

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
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel5,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 8
#elif NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 7
#elif NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 6
#elif NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 5
#elif NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 4
#else
#define UI_MENU_CONFEXTCOND
#define UI_MENU_CONFEXTCNT 0
#endif
#define UI_MENU_CEXTR {UI_MENU_ADDCONDBACK UI_MENU_CONFEXTCOND &ui_menu_cext_steps,&ui_menu_cext_start_feedrate,&ui_menu_cext_max_feedrate,&ui_menu_cext_acceleration,&ui_menu_cext_watch_period,&ui_menu_ext_wait_units,&ui_menu_ext_wait_temp UI_MENU_ADVANCE UI_MENU_PIDCOND}
UI_MENU(ui_menu_cextr, UI_MENU_CEXTR, 7 + UI_MENU_BACKCNT + UI_MENU_PIDCNT + UI_MENU_CONFEXTCNT + UI_MENU_ADV_CNT)

// HeatBed Configuration - use menu actions from extruder configuration
#if HAVE_HEATED_BED
#define UI_MENU_BEDCONF {UI_MENU_ADDCONDBACK UI_MENU_COATING_COND &ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax,&ui_menu_cext_pgain_dt,&ui_menu_cext_pmax}
UI_MENU(ui_menu_bedconf, UI_MENU_BEDCONF, 8 + UI_MENU_BACKCNT + UI_MENU_COATING_CNT)
#endif



// **** Configuration menu

UI_MENU_SUBMENU_T(ui_menu_conf_general, UI_TEXT_GENERAL_ID,      ui_menu_general)
UI_MENU_SUBMENU_T(ui_menu_conf_accel,   UI_TEXT_ACCELERATION_ID, ui_menu_accel)
UI_MENU_SUBMENU_T(ui_menu_conf_feed,    UI_TEXT_FEEDRATE_ID,     ui_menu_feedrate)
UI_MENU_SUBMENU_T(ui_menu_conf_extr,    UI_TEXT_EXTRUDER_ID,     ui_menu_cextr)
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
UI_MENU_ACTIONCOMMAND(ui_menu_conf_reset_eeeprom, "Reset EEPROM", UI_ACTION_RESET_EEPROM)
#define UI_MENU_EEPROM_COND ,&ui_menu_conf_to_eeprom,&ui_menu_conf_from_eeprom, &ui_menu_conf_reset_eeeprom
#define UI_MENU_EEPROM_CNT 3
UI_MENU_ACTION2_T(ui_menu_eeprom_saved,  UI_ACTION_DUMMY, UI_TEXT_EEPROM_STOREDA_ID, UI_TEXT_EEPROM_STOREDB_ID)
UI_MENU_ACTION2_T(ui_menu_eeprom_loaded, UI_ACTION_DUMMY, UI_TEXT_EEPROM_LOADEDA_ID, UI_TEXT_EEPROM_LOADEDB_ID)
UI_MENU_ACTION2_T(ui_menu_eeprom_reset, UI_ACTION_DUMMY, UI_TEXT_EEPROM_RESETEDA_ID, UI_TEXT_EEPROM_RESETEDB_ID)
#else
#define UI_MENU_EEPROM_COND
#define UI_MENU_EEPROM_CNT 0
#endif
#if defined(SOFTWARE_LEVELING) && DRIVE_SYSTEM == DELTA
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
#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK &ui_menu_sub_info, LANGMENU_ENTRY &ui_menu_conf_extr UI_MENU_BEDCONF_COND ,&ui_menu_general_baud  UI_MENU_EEPROM_COND UI_MENU_DELTA_COND UI_MENU_SL_COND}
UI_MENU(ui_menu_configuration, UI_MENU_CONFIGURATION, UI_MENU_BACKCNT + LANGMENU_COUNT + UI_MENU_EEPROM_CNT + UI_MENU_BEDCONF_CNT + UI_MENU_DELTA_CNT + UI_MENU_SL_CNT + 3)


// **** Preheat menu ****

#if HAVE_HEATED_BED + NUM_EXTRUDER > 0
UI_MENU_HEADLINE_T(ui_menu_preheat_hdl,UI_TEXT_PREHEAT_TEMPS_ID)
#if HAVE_HEATED_BED
UI_MENU_CHANGEACTION_T(ui_menu_preheat_bed, UI_TEXT_PREHEAT_BED_ID, UI_ACTION_BED_PREHEAT)
#define UI_MENU_PREHEAT_BED ,&ui_menu_preheat_bed
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

UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pla_a,"PLA",UI_ACTION_SPH_PLA_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_petg_a,"PETG",UI_ACTION_SPH_PETG_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pva_a,"PVA",UI_ACTION_SPH_PVA_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_flex_a,"FLEX",UI_ACTION_SPH_FLEX_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_abs_a,"ABS",UI_ACTION_SPH_ABS_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_glass_a,"GLASS",UI_ACTION_SPH_GLASS_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_wood_a,"WOOD",UI_ACTION_SPH_WOOD_ALL)

#define UI_MENU_PREHEAT_SUB {&ui_menu_preheat_hdl UI_MENU_ADDCONDBACK_C UI_MENU_PREHEAT_BED UI_MENU_PREHEAT_EXT0 UI_MENU_PREHEAT_EXT1 UI_MENU_PREHEAT_EXT2 UI_MENU_PREHEAT_EXT3 UI_MENU_PREHEAT_EXT4 UI_MENU_PREHEAT_EXT5\
,&ui_menu_chf_sph_pla_a,&ui_menu_chf_sph_petg_a,&ui_menu_chf_sph_pva_a,&ui_menu_chf_sph_flex_a,&ui_menu_chf_sph_abs_a,&ui_menu_chf_sph_glass_a,&ui_menu_chf_sph_wood_a}
UI_MENU(ui_menu_preheat_sub, UI_MENU_PREHEAT_SUB, UI_MENU_BACKCNT + 8 + HAVE_HEATED_BED + NUM_EXTRUDER)
UI_MENU_SUBMENU_T(ui_menu_preheat,UI_TEXT_PREHEAT_TEMPS_ID,ui_menu_preheat_sub)

#define UI_MENU_PREHEAT ,&ui_menu_preheat
#define UI_MENU_PREHEAT_CNT 1
#else
#define UI_MENU_PREHEAT 
#define UI_MENU_PREHEAT_CNT 0
#endif

// Preheat info screen

UI_MENU_HEADLINE_T(ui_msh_ph1, UI_TEXT_PREHEAT_TEMPS_ID)
UI_MENU_HEADLINE_T(ui_msh_ph2, UI_TEXT_PREHEAT_BED_ID)
UI_MENU_HEADLINE_T(ui_msh_ph3, UI_TEXT_PREHEAT_E0_ID)
UI_MENU_HEADLINE_T(ui_msh_ph4, UI_TEXT_PREHEAT_E1_ID)
UI_MENU_ACTIONCOMMAND_T(ui_msg_ph_ok, UI_TEXT_OK_ID, UI_ACTION_MESSAGE)
#define UI_MENU_PREHEAT_SET {&ui_msh_ph1,&ui_msh_ph2,&ui_msh_ph3,&ui_msh_ph4,&ui_msg_ph_ok}
UI_STICKYMENU(ui_menu_preheatinfo,UI_MENU_PREHEAT_SET,5)

// **** Setup Menu

/*
Debug
Z Calibrate
Autlevel on/off
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

#define UI_MENU_SETUP {UI_MENU_ADDCONDBACK &ui_debug UI_MENU_PREHEAT UI_TOOGLE_AUTOLEVEL_ENTRY UI_CALIB_PROBE_ENTRY UI_DISTORTION_ENTRY UI_CAL_ZHEIGHT_ENTRY ,&ui_calex ,&ui_menu_fan_ignoreM106}
UI_MENU(ui_menu_setup, UI_MENU_SETUP, UI_MENU_BACKCNT + 3 + UI_MENU_PREHEAT_CNT + UI_TOGGLE_AUTOLEVEL_COUNT + UI_DISTORTION_COUNT + UI_CAL_ZHEIGHT_CNT + UI_CALIB_PROBE_COUNT)
UI_MENU_SUBMENU_FILTER_T(ui_setup, UI_TEXT_SETUP_ID, ui_menu_setup,0,MENU_MODE_PRINTING)

// Stop print security question


// Main menu

// stop/pause/continue entries

UI_MENU_ACTIONCOMMAND_FILTER_T(ui_pause,UI_TEXT_PAUSE_PRINT_ID,UI_ACTION_PAUSE,MENU_MODE_PRINTING,MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_continue,UI_TEXT_CONTINUE_PRINT_ID,UI_ACTION_CONTINUE,MENU_MODE_PAUSED,0)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_stop,UI_TEXT_STOP_PRINT_ID,UI_ACTION_STOP,MENU_MODE_PRINTING,MENU_MODE_PAUSED)


UI_MENU_SUBMENU_FILTER_T(ui_menu_control, UI_TEXT_QUICK_SETTINGS_ID, ui_menu_quick,0,MENU_MODE_PRINTING)
UI_MENU_SUBMENU_FILTER_T(ui_menu_extrudercontrol, UI_TEXT_EXTRUDER_ID, ui_menu_extruder,0,MENU_MODE_PRINTING)

UI_MENU_SUBMENU_FILTER_T(ui_menu_settings, UI_TEXT_CONFIGURATION_ID, ui_menu_configuration,0,MENU_MODE_PRINTING)
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK &ui_menu_control ,&ui_stop,&ui_pause,&ui_continue UI_CHANGE_FIL_ENT_PRINTING\
    BABY_ENTRY_PRINTING  ,&ui_menu_quick_speedmultiply_printing,&ui_menu_quick_flowmultiply_printing UI_TEMP0_PRINTING UI_TEMP1_PRINTING UI_TEMP2_PRINTING UI_TEMP3_PRINTING UI_TEMP4_PRINTING UI_TEMP5_PRINTING \
    UI_BED_TEMP_PRINTING  UI_FANSPEED_PRINTING UI_FAN2SPEED_PRINTING  , SD_PRINTFILE_ENTRY \
    &ui_menu_settings}
   // &ui_menu_move, &ui_menu_extrudercontrol, 
    
UI_MENU(ui_menu_main, UI_MENU_MAIN, 7 + UI_MENU_BACKCNT + SD_PRINTFILE_ENTRY_CNT +UI_TEMP0_CNT+UI_TEMP1_CNT+UI_TEMP2_CNT+UI_TEMP3_CNT+\
    UI_TEMP4_CNT+UI_TEMP5_CNT+ BABY_CNT+HAVE_HEATED_BED+UI_MENU_FAN_CNT+UI_MENU_FAN2_CNT + UI_CHANGE_FIL_CNT)

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

  Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperture screen and allows
  the change of temperature with the next/previous buttons.

*/
#endif
#endif // __UI_MENU_H
