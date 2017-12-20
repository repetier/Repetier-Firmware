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


#if !defined(UI_MENU_H) && defined(UI_MAIN)
#define UI_MENU_H

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
%oB : Buffer length
%om : Speed multiplier
%of : flow multiplier
%oc : Connection baudrate
%o0..9 : Output level extruder 0..9 is % including %sign
%oC : Output level current extruder
%ob : Output level heated bed
%%% : The % char
%x0 : X position
%x1 : Y position
%x2 : Z position
%x3 : Current extruder position
%sx : State of x min endstop
%sX : State of x max endstop
%sy : State of y min endstop
%sY : State of y max endstop
%sz : State of z min endstop
%sZ : State of z max endstop
%sC : State of the z compensation
%do : Debug echo state
%di : Debug info state
%de : Debug error state
%dd : Debug dry run state
%db : beeper state
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
%Fs : Fan speed
%PN : Printer name
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
%XT : Extruder wait retract temperature
%XU : Extruder wait retract unit
%Xh : Extruder heat manager (BangBang/PID)
%Xa : Advance K value
%Xl : Advance L value
%Xf : Extruder max. start feedrate
%XF : Extruder max. feedrate
%XA : Extruder max. acceleration

// RF specific:
%s1 : current value of the strain gauge
%Dx : scan step size x
%Dy : scan step size y
%lo : light state
%li : Light: White/Color
%ou : 230V output on/off
%OM : operating mode
%OZ : z endstop type
%z0 : Z Offset
%zm : Z Scale
%ht : hotend type
%mt : miller type
%mY : menu yes
%mN : menu no
%m1 : message line 1
%m2 : message line 2
%m3 : message line 3
%m4 : message line 4
%Oa : Active Extruder
%OE : Extruder offset X [mm]
%OF : Extruder offset Y [mm]
%px : mode of the Position X menu
%py : mode of the Position Y menu
%pz : mode of the Position Z menu
%HB : active heat bed z matrix
%WP : active work part z matrix
%Z1-Z4: Page5 service intervall
%Z5-Z8: Page4 printing/milling time
*/

// Define precision for temperatures. With small displays only integer values fit.
#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 1
#else
#define UI_TEMP_PRECISION 0
#endif // UI_COLS>16
#endif // UI_TEMP_PRECISION


/* ============= PAGES DEFINITION =============

If you are not iside a menu, the firmware displays pages with information.
Especially if you have only a small display it is convenient to have
more then one information page.
*/

/* Define your pages using dynamic strings. To define a page use
UI_PAGE4(name,row1,row2,row3,row4);
for 4 row displays and
UI_PAGE2(name,row1,row2);
for 2 row displays. You can add additional pages or change the default pages like you want.
*/

#if UI_ROWS>=6 && UI_DISPLAY_TYPE==5

	// graphic main status
	UI_PAGE6(ui_page1,"\xa %e0/%E0\xb0 X:%x0",

#if NUM_EXTRUDER>1
     "\xa %e1/%E1\xb0 Y:%x1",
#else
     "\xa -----/---\xb0 Y:%x1",
#endif // NUM_EXTRUDER>1

#if HAVE_HEATED_BED==true
     "\xe %eb/%Eb\xb0 Z:%x2",
#else
     "\xb -----/---\xb0 Z:%x2",
#endif // HAVE_HEATED_BED==true

	"Mul:%om", "Buf:%oB", "%os");

#if EEPROM_MODE!=0
    UI_PAGE4(ui_page2,"%Z5","%Z6","%Z7","%Z8");
    #define UI_PRINTTIME_PAGES ,&ui_page2
    #define UI_PRINTTIME_COUNT 1
#else
    #define UI_PRINTTIME_PAGES
    #define UI_PRINTTIME_COUNT 0
#endif // EEPROM_MODE!=0

	/* Merge pages together. Use the following pattern:
	#define UI_PAGES {&name1,&name2,&name3} */
	#define UI_PAGES {&ui_page1 UI_PRINTTIME_PAGES}

	// How many pages do you want to have. Minimum is 1.
	#define UI_NUM_PAGES 1+UI_PRINTTIME_COUNT

#elif UI_ROWS>=4
#if HAVE_HEATED_BED==true
#if UI_COLS<=16
	UI_PAGE4(ui_page1,"%U1%ec/%EcB%eB/%Eb","Z:%x2 mm %sC",UI_TEXT_STRAIN_GAUGE,"%os");
#else
	UI_PAGE4(ui_page1,"%U1%ec/%Ec\002 B%eB/%Eb\002","Z:%x2 mm %sC",UI_TEXT_STRAIN_GAUGE,"%os");
#endif // UI_COLS<=16
#else
	UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,"Z:%x2 mm %sC",UI_TEXT_STRAIN_GAUGE,"%os");
#endif // HAVE_HEATED_BED==true

	UI_PAGE4(ui_page2,"X:%x0 mm","Y:%x1 mm","Z:%x2 mm %sC","%os");
	UI_PAGE4(ui_page3,UI_TEXT_PAGE_EXTRUDER1, 

 #if NUM_EXTRUDER>1
	UI_TEXT_PAGE_EXTRUDER2
 #else
	""
 #endif // NUM_EXTRUDER>1
  
	,UI_TEXT_PAGE_BED,"%os");

#if EEPROM_MODE!=0
	UI_PAGE4(ui_page4,"%Z5","%Z6","%Z7","%Z8");
  
	#define UI_PRINTTIME_PAGES ,&ui_page4
	#define UI_PRINTTIME_COUNT 1
#else
	#define UI_PRINTTIME_PAGES
	#define UI_PRINTTIME_COUNT 0
#endif // EEPROM_MODE!=0

#if EEPROM_MODE!=0 && FEATURE_SERVICE_INTERVAL
	UI_PAGE4(ui_page5,"%Z1","%Z2","%Z3","%Z4");
	#define UI_SERVICE_PAGES ,&ui_page5
	#define UI_SERVICE_COUNT 1
#else
	#define UI_SERVICE_PAGES
	#define UI_SERVICE_COUNT 0
#endif // EEPROM_MODE && FEATURE_SERVICE_INTERVAL

	/* Merge pages together. Use the following pattern:
	#define UI_PAGES {&name1,&name2,&name3} */
	#define UI_PAGES {&ui_page1,&ui_page2,&ui_page3 UI_PRINTTIME_PAGES UI_SERVICE_PAGES}

	// How many pages do you want to have. Minimum is 1.
	#define UI_NUM_PAGES 3+UI_PRINTTIME_COUNT+UI_SERVICE_COUNT
#else
	UI_PAGE2(ui_page1,UI_TEXT_PAGE_EXTRUDER,UI_TEXT_PAGE_BED);
	UI_PAGE2(ui_page2,"X:%x0 Y:%x1","%os");
	UI_PAGE2(ui_page3,"Z:%x2 mm %sC","%os");

	/* Merge pages together. Use the following pattern:
	#define UI_PAGES {&name1,&name2,&name3} */
	#define UI_PAGES {&ui_page1,&ui_page2,&ui_page3}

	// How many pages do you want to have. Minimum is 1.
	#define UI_NUM_PAGES 3
#endif // UI_ROWS>=4

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


// error/warning/information message
UI_MENU_ACTION4(ui_menu_message,UI_ACTION_DUMMY,"%m1","%m2","%m3","%m4");

/** \brief Positions submenus */
#if UI_ROWS>=4
UI_MENU_ACTION4C(ui_menu_xpos,UI_ACTION_XPOSITION,UI_TEXT_ACTION_XPOSITION4);
UI_MENU_ACTION4C(ui_menu_ypos,UI_ACTION_YPOSITION,UI_TEXT_ACTION_YPOSITION4);
UI_MENU_ACTION4C(ui_menu_zpos,UI_ACTION_ZPOSITION,UI_TEXT_ACTION_ZPOSITION4);
UI_MENU_ACTION4C(ui_menu_zpos_notest,UI_ACTION_ZPOSITION,UI_TEXT_ACTION_ZPOSITION4);
UI_MENU_ACTION4C(ui_menu_xpos_fast,UI_ACTION_XPOSITION_FAST,UI_TEXT_ACTION_XPOSITION_FAST4);
UI_MENU_ACTION4C(ui_menu_ypos_fast,UI_ACTION_YPOSITION_FAST,UI_TEXT_ACTION_YPOSITION_FAST4);
UI_MENU_ACTION4C(ui_menu_zpos_fast,UI_ACTION_ZPOSITION_FAST,UI_TEXT_ACTION_ZPOSITION_FAST4);
UI_MENU_ACTION4C(ui_menu_zpos_fast_notest,UI_ACTION_ZPOSITION_FAST_NOTEST,UI_TEXT_ACTION_ZPOSITION_FAST4);
#endif // UI_ROWS>=4

UI_MENU_ACTION2C(ui_menu_epos,UI_ACTION_EPOSITION,UI_TEXT_ACTION_EPOSITION_FAST2);

/* Next step is to define submenus leading to the action. */

/** \brief Positionening menu */
UI_MENU_ACTIONCOMMAND(ui_menu_back,UI_TEXT_BACK,UI_ACTION_BACK);
#if UI_HAS_BACK_KEY==0
#define UI_MENU_ADDCONDBACK &ui_menu_back,
#define UI_MENU_BACKCNT 1
#else
#define UI_MENU_ADDCONDBACK
#define UI_MENU_BACKCNT 0
#endif // UI_HAS_BACK_KEY==0

UI_MENU_ACTIONCOMMAND(ui_menu_home_all,UI_TEXT_HOME_ALL,UI_ACTION_HOME_ALL);
UI_MENU_ACTIONCOMMAND(ui_menu_home_x,UI_TEXT_HOME_X,UI_ACTION_HOME_X);
UI_MENU_ACTIONCOMMAND(ui_menu_home_y,UI_TEXT_HOME_Y,UI_ACTION_HOME_Y);
UI_MENU_ACTIONCOMMAND(ui_menu_home_z,UI_TEXT_HOME_Z,UI_ACTION_HOME_Z);
UI_MENU_ACTIONSELECTOR(ui_menu_go_xpos,UI_TEXT_X_POSITION,ui_menu_xpos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_ypos,UI_TEXT_Y_POSITION,ui_menu_ypos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zpos,UI_TEXT_Z_POSITION,ui_menu_zpos);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zpos_notest,UI_TEXT_Z_POSITION,ui_menu_zpos_notest);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_go_epos,UI_TEXT_E_POSITION,ui_menu_epos,MENU_MODE_PRINTER,0);

#if !UI_SPEEDDEPENDENT_POSITIONING
UI_MENU_ACTIONSELECTOR(ui_menu_go_xfast,UI_TEXT_X_POS_FAST,ui_menu_xpos_fast);
UI_MENU_ACTIONSELECTOR(ui_menu_go_yfast,UI_TEXT_Y_POS_FAST,ui_menu_ypos_fast);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zfast,UI_TEXT_Z_POS_FAST,ui_menu_zpos_fast);
UI_MENU_ACTIONSELECTOR(ui_menu_go_zfast_notest,UI_TEXT_Z_POS_FAST,ui_menu_zpos_fast_notest);
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
#endif // UI_SPEEDDEPENDENT_POSITIONING

#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z UI_SPEED_X UI_SPEED_Y UI_SPEED_Z ,&ui_menu_go_epos}
UI_MENU(ui_menu_positions,UI_MENU_POSITIONS,5 + 3 * UI_SPEED + UI_MENU_BACKCNT);

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan,UI_TEXT_DO_HEAT_BED_SCAN,UI_ACTION_RF_SCAN_HEAT_BED,MENU_MODE_PRINTER,0);

#if FEATURE_PRECISE_HEAT_BED_SCAN

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan_pla,UI_TEXT_HEAT_BED_SCAN_PLA,UI_ACTION_RF_SCAN_HEAT_BED_PLA,MENU_MODE_PRINTER,0);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan_abs,UI_TEXT_HEAT_BED_SCAN_ABS,UI_ACTION_RF_SCAN_HEAT_BED_ABS,MENU_MODE_PRINTER,0);
#define UI_MENU_HEAT_BED_MODE_COND	 ,&ui_menu_heat_bed_scan_pla, &ui_menu_heat_bed_scan_abs
#define	UI_MENU_HEAT_BED_MODE_COUNT 2
#else
#define UI_MENU_HEAT_BED_MODE_COND	 
#define	UI_MENU_HEAT_BED_MODE_COUNT 0

#endif // FEATURE_PRECISE_HEAT_BED_SCAN

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_work_part_scan,UI_TEXT_DO_WORK_PART_SCAN,UI_ACTION_RF_SCAN_WORK_PART,0,MENU_MODE_PRINTER);

UI_MENU_ACTION4C(ui_menu_zoffset,UI_ACTION_ZOFFSET,UI_TEXT_ACTION_ZOFFSET);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_zoffset_z,UI_TEXT_Z_OFFSET,ui_menu_zoffset,MENU_MODE_PRINTER,0);

UI_MENU_CHANGEACTION_FILTER(ui_menu_set_z_matrix_heat_bed,UI_TEXT_SET_Z_MATRIX_HEAT_BED,UI_ACTION_RF_SET_Z_MATRIX_HEAT_BED,MENU_MODE_PRINTER,0);

/** \brief Z calibration menu */
#if FEATURE_MILLING_MODE

// in case the milling mode is enabled, we need the following menus
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_z_origin,UI_TEXT_SET_Z_ORIGIN,UI_ACTION_SET_Z_ORIGIN,0,MENU_MODE_PRINTER);

#if FEATURE_FIND_Z_ORIGIN
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_find_z_origin,UI_TEXT_FIND_Z_ORIGIN,UI_ACTION_RF_FIND_Z_ORIGIN,0,MENU_MODE_PRINTER);
#define UI_MENU_FIND_Z_COND	 ,&ui_menu_find_z_origin
#define	UI_MENU_FIND_Z_COUNT 1
#else
#define UI_MENU_FIND_Z_COND	 
#define	UI_MENU_FIND_Z_COUNT 0
#endif // FEATURE_FIND_Z_ORIGIN

UI_MENU_CHANGEACTION_FILTER(ui_menu_set_z_matrix_work_part,UI_TEXT_SET_Z_MATRIX_WORK_PART,UI_ACTION_RF_SET_Z_MATRIX_WORK_PART,0,MENU_MODE_PRINTER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_xy_start,UI_TEXT_SET_SCAN_XY_START,UI_ACTION_RF_SET_SCAN_XY_START,0,MENU_MODE_PRINTER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_xy_end,UI_TEXT_SET_SCAN_XY_END,UI_ACTION_RF_SET_SCAN_XY_END,0,MENU_MODE_PRINTER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_delta_x,UI_TEXT_SET_SCAN_DELTA_X,UI_ACTION_RF_SET_SCAN_DELTA_X,0,MENU_MODE_PRINTER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_delta_y,UI_TEXT_SET_SCAN_DELTA_Y,UI_ACTION_RF_SET_SCAN_DELTA_Y,0,MENU_MODE_PRINTER);

#define UI_MENU_Z {UI_MENU_ADDCONDBACK /*UI_MENU_ADJUST_HOTEND*/ &ui_menu_heat_bed_scan UI_MENU_HEAT_BED_MODE_COND, &ui_menu_zoffset_z, &ui_menu_work_part_scan UI_SPEED_Z /*UI_SPEED_Z_NOTEST*/,&ui_menu_set_z_origin UI_MENU_FIND_Z_COND, &ui_menu_set_z_matrix_heat_bed, &ui_menu_set_z_matrix_work_part, &ui_menu_set_xy_start, &ui_menu_set_xy_end, &ui_menu_set_delta_x, &ui_menu_set_delta_y}
UI_MENU(ui_menu_z,UI_MENU_Z,11 + UI_MENU_BACKCNT + UI_MENU_HEAT_BED_MODE_COUNT + UI_MENU_FIND_Z_COUNT/*+UI_MENU_ADJUST_HOTEND_COUNT*/);

#else

#define UI_MENU_Z {UI_MENU_ADDCONDBACK &ui_menu_heat_bed_scan UI_MENU_HEAT_BED_MODE_COND, &ui_menu_zoffset_z UI_SPEED_Z /*UI_SPEED_Z_NOTEST*/, &ui_menu_set_z_matrix_heat_bed}
UI_MENU(ui_menu_z,UI_MENU_Z,4 + UI_MENU_BACKCNT + UI_MENU_HEAT_BED_MODE_COUNT);

#endif // FEATURE_MILLING_MODE

/** \brief Extruder menu */

UI_MENU_CHANGEACTION(ui_menu_ext_temp0,UI_TEXT_EXTR0_TEMP,UI_ACTION_EXTRUDER0_TEMP);
UI_MENU_CHANGEACTION(ui_menu_ext_temp1,UI_TEXT_EXTR1_TEMP,UI_ACTION_EXTRUDER1_TEMP);
UI_MENU_CHANGEACTION(ui_menu_bed_temp, UI_TEXT_BED_TEMP,UI_ACTION_HEATED_BED_TEMP);

UI_MENU_ACTIONCOMMAND(ui_menu_active_extruder,UI_TEXT_ACTIVE_EXTRUDER,UI_ACTION_ACTIVE_EXTRUDER);

//UI_MENU_ACTIONCOMMAND(ui_menu_ext_sel1,UI_TEXT_EXTR1_SELECT,UI_ACTION_SELECT_EXTRUDER1);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off0,UI_TEXT_EXTR0_OFF,UI_ACTION_EXTRUDER0_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off1,UI_TEXT_EXTR1_OFF,UI_ACTION_EXTRUDER1_OFF);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_origin,UI_TEXT_SET_E_ORIGIN,UI_ACTION_SET_E_ORIGIN);

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_unmount_filament,UI_TEXT_UNMOUNT_FILAMENT,UI_ACTION_UNMOUNT_FILAMENT,MENU_MODE_PRINTER,0);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_mount_filament,UI_TEXT_MOUNT_FILAMENT,UI_ACTION_MOUNT_FILAMENT,MENU_MODE_PRINTER,0);

#if NUM_EXTRUDER>1
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_active_extruder,
#define UI_MENU_EXTCNT 5
#elif NUM_EXTRUDER>2
UI_MENU_CHANGEACTION(ui_menu_ext_temp2,UI_TEXT_EXTR2_TEMP,UI_ACTION_EXTRUDER2_TEMP);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_sel2,UI_TEXT_EXTR2_SELECT,UI_ACTION_SELECT_EXTRUDER2);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off2,UI_TEXT_EXTR2_OFF,UI_ACTION_EXTRUDER2_OFF);
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel1,
#define UI_MENU_EXTCNT 9
#else
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_off0,
#define UI_MENU_EXTCNT 2
#endif // NUM_EXTRUDER>1

#if HAVE_HEATED_BED==true
#define UI_MENU_BEDCOND &ui_menu_bed_temp,
#define UI_MENU_BEDCNT 1
#else
#define UI_MENU_BEDCOND
#define UI_MENU_BEDCNT 0
#endif // HAVE_HEATED_BED==true

#define UI_MENU_EXTRUDER {UI_MENU_ADDCONDBACK UI_MENU_BEDCOND UI_MENU_EXTCOND &ui_menu_go_epos,&ui_menu_extruder_mount_filament,&ui_menu_extruder_unmount_filament,&ui_menu_ext_origin}
UI_MENU(ui_menu_extruder,UI_MENU_EXTRUDER,UI_MENU_BACKCNT+UI_MENU_BEDCNT+UI_MENU_EXTCNT+4);

/** \brief Quick menu */
#if PS_ON_PIN>=0
UI_MENU_ACTIONCOMMAND(ui_menu_quick_power,UI_TEXT_POWER,UI_ACTION_POWER);
#define MENU_PSON_COUNT 1
#define MENU_PSON_ENTRY ,&ui_menu_quick_power
#else
#define MENU_PSON_COUNT 0
#define MENU_PSON_ENTRY
#endif // PS_ON_PIN>=0

UI_MENU_ACTION4C(ui_menu_quick_stop_print_ack, UI_ACTION_SD_STOP_ACK, UI_TEXT_STOP_PRINT_ACK);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_quick_stop_print, UI_TEXT_STOP_PRINT, ui_menu_quick_stop_print_ack, MENU_MODE_SD_PRINTING, MENU_MODE_MILLER);
UI_MENU_ACTION4C(ui_menu_quick_stop_mill_ack, UI_ACTION_SD_STOP_ACK, UI_TEXT_STOP_MILL_ACK);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_quick_stop_mill, UI_TEXT_STOP_MILL, ui_menu_quick_stop_mill_ack, MENU_MODE_SD_PRINTING, MENU_MODE_PRINTER);

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_preheat_pla,UI_TEXT_PREHEAT_PLA,UI_ACTION_PREHEAT_PLA,MENU_MODE_PRINTER,0);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_preheat_abs,UI_TEXT_PREHEAT_ABS,UI_ACTION_PREHEAT_ABS,MENU_MODE_PRINTER,0);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_cooldown,UI_TEXT_COOLDOWN,UI_ACTION_COOLDOWN,MENU_MODE_PRINTER,0);

#if FEATURE_SET_TO_XY_ORIGIN
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_xy_origin,UI_TEXT_SET_XY_ORIGIN,UI_ACTION_SET_XY_ORIGIN,0,MENU_MODE_PRINTER);
#define SET_TO_XY_ORIGIN_COUNT 1
#define SET_TO_XY_ORIGIN_ENTRY ,&ui_menu_quick_xy_origin
#else
#define SET_TO_XY_ORIGIN_COUNT 0
#define SET_TO_XY_ORIGIN_ENTRY 
#endif // FEATURE_SET_TO_XY_ORIGIN

UI_MENU_ACTIONCOMMAND(ui_menu_quick_stopstepper,UI_TEXT_DISABLE_STEPPER,UI_ACTION_DISABLE_STEPPER);

#ifdef FEATURE_BABYSTEPPING
UI_MENU_CHANGEACTION(ui_menu_quick_zbaby,UI_TEXT_Z_BABYSTEPPING,UI_ACTION_Z_BABYSTEPS);
#define BABY_COUNT 1
#define BABY_ENTRY ,&ui_menu_quick_zbaby
#else
#define BABY_COUNT 0
#define BABY_ENTRY
#endif // FEATURE_BABYSTEPPING

UI_MENU_CHANGEACTION(ui_menu_quick_speedmultiply,UI_TEXT_SPEED_MULTIPLY,UI_ACTION_FEEDRATE_MULTIPLY);
UI_MENU_CHANGEACTION_FILTER(ui_menu_quick_flowmultiply,UI_TEXT_FLOW_MULTIPLY,UI_ACTION_FLOWRATE_MULTIPLY,MENU_MODE_PRINTER,0);

#if FEATURE_OUTPUT_FINISHED_OBJECT
UI_MENU_ACTIONCOMMAND(ui_menu_quick_output_object,UI_TEXT_OUTPUT_OBJECT,UI_ACTION_RF_OUTPUT_OBJECT);
#define OUTPUT_OBJECT_COUNT 1
#define OUTPUT_OBJECT_ENTRY ,&ui_menu_quick_output_object
#else
#define OUTPUT_OBJECT_COUNT 0
#define OUTPUT_OBJECT_ENTRY 
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FEATURE_PARK
UI_MENU_ACTIONCOMMAND(ui_menu_quick_park,UI_TEXT_PARK_HEAT_BED,UI_ACTION_RF_PARK);
#define PARK_COUNT 1
#define PARK_ENTRY ,&ui_menu_quick_park
#else
#define PARK_COUNT 0
#define PARK_ENTRY 
#endif // FEATURE_PARK

#if FEATURE_230V_OUTPUT
UI_MENU_ACTIONCOMMAND(ui_menu_toggle_230V_output,UI_TEXT_230V_OUTPUT,UI_ACTION_230V_OUTPUT);
#define OUTPUT_230V_ENTRY ,&ui_menu_toggle_230V_output
#define OUTPUT_230V_COUNT 1
#else
#define OUTPUT_230V_ENTRY
#define OUTPUT_230V_COUNT 0
#endif // FEATURE_230V_OUTPUT

#if FEATURE_RESET_VIA_MENU
UI_MENU_ACTION4C(ui_menu_quick_reset_ack,UI_ACTION_RF_RESET_ACK,UI_TEXT_RESET_ACK);
UI_MENU_ACTIONSELECTOR(ui_menu_quick_reset,UI_TEXT_RESET,ui_menu_quick_reset_ack);
#define RESET_VIA_MENU_COUNT 1
#define RESET_VIA_MENU_ENTRY ,&ui_menu_quick_reset
#else
#define RESET_VIA_MENU_COUNT 0
#define RESET_VIA_MENU_ENTRY 
#endif // FEATURE_RESET_VIA_MENU

#if FEATURE_RGB_LIGHT_EFFECTS
UI_MENU_CHANGEACTION(ui_menu_toggle_rgb_light,UI_TEXT_RGB_LIGHT_MODE,UI_ACTION_RGB_LIGHT_MODE);
#define RGB_LIGHT_ENTRY ,&ui_menu_toggle_rgb_light
#define RGB_LIGHT_COUNT 1
#else
#define RGB_LIGHT_ENTRY
#define RGB_LIGHT_COUNT 0
#endif // FEATURE_RGB_LIGHT_EFFECTS

#ifdef DEBUG_PRINT
UI_MENU_ACTIONCOMMAND(ui_menu_quick_debug,"Write Debug",UI_ACTION_WRITE_DEBUG);
#define DEBUG_PRINT_COUNT 1
#define DEBUG_PRINT_EXTRA ,&ui_menu_quick_debug
#else
#define DEBUG_PRINT_COUNT 0
#define DEBUG_PRINT_EXTRA
#endif // DEBUG_PRINT

#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_quick_stop_print,&ui_menu_quick_stop_mill RGB_LIGHT_ENTRY BABY_ENTRY OUTPUT_OBJECT_ENTRY ,&ui_menu_quick_speedmultiply,&ui_menu_quick_flowmultiply,&ui_menu_quick_preheat_pla,&ui_menu_quick_preheat_abs,&ui_menu_quick_cooldown SET_TO_XY_ORIGIN_ENTRY, &ui_menu_quick_stopstepper PARK_ENTRY OUTPUT_230V_ENTRY RESET_VIA_MENU_ENTRY MENU_PSON_ENTRY DEBUG_PRINT_EXTRA }
UI_MENU(ui_menu_quick,UI_MENU_QUICK,9+UI_MENU_BACKCNT+MENU_PSON_COUNT+DEBUG_PRINT_COUNT+SET_TO_XY_ORIGIN_COUNT+BABY_COUNT+OUTPUT_OBJECT_COUNT+PARK_COUNT+OUTPUT_230V_COUNT+RESET_VIA_MENU_COUNT+RGB_LIGHT_COUNT);

/** \brief Fan menu */

#if FAN_PIN>-1
UI_MENU_CHANGEACTION(ui_menu_fan_fanspeed, UI_TEXT_ACTION_FANSPEED,UI_ACTION_FANSPEED);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_fan_off,UI_TEXT_FAN_OFF,UI_ACTION_FAN_OFF,MENU_MODE_FAN_RUNNING,0);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_25,UI_TEXT_FAN_25,UI_ACTION_FAN_25);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_50,UI_TEXT_FAN_50,UI_ACTION_FAN_50);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_75,UI_TEXT_FAN_75,UI_ACTION_FAN_75);
UI_MENU_ACTIONCOMMAND(ui_menu_fan_full,UI_TEXT_FAN_FULL,UI_ACTION_FAN_FULL);
#define UI_MENU_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_fanspeed,&ui_menu_fan_off,&ui_menu_fan_25,&ui_menu_fan_50,&ui_menu_fan_75,&ui_menu_fan_full}
UI_MENU(ui_menu_fan,UI_MENU_FAN,6+UI_MENU_BACKCNT);
UI_MENU_SUBMENU_FILTER(ui_menu_fan_sub,UI_TEXT_FANSPEED,ui_menu_fan,MENU_MODE_PRINTER,0);
#define UI_MENU_FAN_COND &ui_menu_fan_sub,
#define UI_MENU_FAN_CNT 1
#else
#define UI_MENU_FAN_COND
#define UI_MENU_FAN_CNT 0
#endif // FAN_PIN>-1

/** \brief SD card menu */

#ifdef SDSUPPORT
#define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
UI_MENU_FILESELECT(ui_menu_sd_fileselector,UI_MENU_SD_FILESELECTOR,1);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_print_file,		UI_TEXT_PRINT_FILE,     UI_ACTION_SD_PRINT,    MENU_MODE_SD_MOUNTED,  MENU_MODE_SD_PRINTING | MENU_MODE_MILLER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_pause_print,    UI_TEXT_PAUSE_PRINT,    UI_ACTION_SD_PAUSE,    MENU_MODE_SD_PRINTING, MENU_MODE_SD_PAUSED   | MENU_MODE_MILLER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_continue_print, UI_TEXT_CONTINUE_PRINT, UI_ACTION_SD_CONTINUE, MENU_MODE_SD_PAUSED,   MENU_MODE_MILLER);
UI_MENU_ACTION4C(ui_menu_sd_stop_print_ack, UI_ACTION_SD_STOP_ACK, UI_TEXT_STOP_PRINT_ACK);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_sd_stop_print, UI_TEXT_STOP_PRINT, ui_menu_sd_stop_print_ack, MENU_MODE_SD_PRINTING, MENU_MODE_MILLER);

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_mill_file,	   UI_TEXT_MILL_FILE,     UI_ACTION_SD_PRINT,    MENU_MODE_SD_MOUNTED,  MENU_MODE_SD_PRINTING | MENU_MODE_PRINTER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_pause_mill,    UI_TEXT_PAUSE_MILL,    UI_ACTION_SD_PAUSE,    MENU_MODE_SD_PRINTING, MENU_MODE_SD_PAUSED   | MENU_MODE_PRINTER);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_continue_mill, UI_TEXT_CONTINUE_MILL, UI_ACTION_SD_CONTINUE, MENU_MODE_SD_PAUSED,   MENU_MODE_PRINTER);
UI_MENU_ACTION4C(ui_menu_sd_stop_mill_ack, UI_ACTION_SD_STOP_ACK, UI_TEXT_STOP_MILL_ACK);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_sd_stop_mill, UI_TEXT_STOP_MILL, ui_menu_sd_stop_mill_ack, MENU_MODE_SD_PRINTING, MENU_MODE_PRINTER);

#if defined(SDCARDDETECT) && SDCARDDETECT>-1
#define UI_MOUNT_CNT 0
#define UI_MOUNT_CMD
#else
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_unmount,UI_TEXT_UNMOUNT_CARD,UI_ACTION_SD_UNMOUNT,MENU_MODE_SD_MOUNTED,0);
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_mount,UI_TEXT_MOUNT_CARD,UI_ACTION_SD_MOUNT,0,MENU_MODE_SD_MOUNTED);
#define UI_MOUNT_CNT 2
#define UI_MOUNT_CMD ,&ui_menu_sd_mount,&ui_menu_sd_unmount
#endif // (SDCARDDETECT) && SDCARDDETECT>-1
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_delete,UI_TEXT_DELETE_FILE,UI_ACTION_SD_DELETE,MENU_MODE_SD_MOUNTED,MENU_MODE_SD_PRINTING);
#define UI_MENU_SD {UI_MENU_ADDCONDBACK &ui_menu_sd_print_file,&ui_menu_sd_mill_file,&ui_menu_sd_pause_print,&ui_menu_sd_pause_mill,&ui_menu_sd_stop_print,&ui_menu_sd_stop_mill,&ui_menu_sd_continue_print,&ui_menu_sd_continue_mill,&ui_menu_sd_delete UI_MOUNT_CMD}
UI_MENU(ui_menu_sd,UI_MENU_SD,UI_MENU_BACKCNT+9+UI_MOUNT_CNT);
UI_MENU_SUBMENU(ui_menu_sd_sub,UI_TEXT_SD_CARD,ui_menu_sd);

#define UI_MENU_SD_COND &ui_menu_sd_sub,
#define UI_MENU_SD_CNT 1
#else
#define UI_MENU_SD_COND
#define UI_MENU_SD_CNT 0
#endif // SDSUPPORT

#if SHOW_DEBUGGING_MENU
/** \brief Debugging menu */
UI_MENU_ACTIONCOMMAND(ui_menu_debug_echo,   UI_TEXT_DBG_ECHO,   UI_ACTION_DEBUG_ECHO);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_info,   UI_TEXT_DBG_INFO,   UI_ACTION_DEBUG_INFO);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_error,  UI_TEXT_DBG_ERROR,  UI_ACTION_DEBUG_ERROR);
UI_MENU_ACTIONCOMMAND(ui_menu_debug_dryrun, UI_TEXT_DBG_DRYRUN, UI_ACTION_DEBUG_DRYRUN);

#define UI_MENU_DEBUGGING {UI_MENU_ADDCONDBACK &ui_menu_debug_echo,&ui_menu_debug_info,&ui_menu_debug_error,&ui_menu_debug_dryrun}
UI_MENU(ui_menu_debugging,UI_MENU_DEBUGGING,4+UI_MENU_BACKCNT);
#endif // SHOW_DEBUGGING_MENU

/** \brief Acceleration settings */
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printx,  UI_TEXT_PRINT_X, UI_ACTION_PRINT_ACCEL_X, 0, MENU_MODE_MILLER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printy,  UI_TEXT_PRINT_Y, UI_ACTION_PRINT_ACCEL_Y, 0, MENU_MODE_MILLER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printz,  UI_TEXT_PRINT_Z, UI_ACTION_PRINT_ACCEL_Z, 0, MENU_MODE_MILLER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_millx,   UI_TEXT_MILL_X,  UI_ACTION_PRINT_ACCEL_X, 0, MENU_MODE_PRINTER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_milly,   UI_TEXT_MILL_Y,  UI_ACTION_PRINT_ACCEL_Y, 0, MENU_MODE_PRINTER);
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_millz,   UI_TEXT_MILL_Z,  UI_ACTION_PRINT_ACCEL_Z, 0, MENU_MODE_PRINTER);
UI_MENU_CHANGEACTION(ui_menu_accel_travelx, UI_TEXT_MOVE_X,  UI_ACTION_MOVE_ACCEL_X);
UI_MENU_CHANGEACTION(ui_menu_accel_travely, UI_TEXT_MOVE_Y,  UI_ACTION_MOVE_ACCEL_Y);
UI_MENU_CHANGEACTION(ui_menu_accel_travelz, UI_TEXT_MOVE_Z,  UI_ACTION_MOVE_ACCEL_Z);
UI_MENU_CHANGEACTION(ui_menu_accel_jerk,    UI_TEXT_JERK,    UI_ACTION_MAX_JERK);
UI_MENU_CHANGEACTION(ui_menu_accel_zjerk,   UI_TEXT_ZJERK,   UI_ACTION_MAX_ZJERK);
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printx,&ui_menu_accel_millx,&ui_menu_accel_printy,&ui_menu_accel_milly,&ui_menu_accel_printz,&ui_menu_accel_millz,&ui_menu_accel_travelx,&ui_menu_accel_travely,&ui_menu_accel_travelz,&ui_menu_accel_jerk,&ui_menu_accel_zjerk}
UI_MENU(ui_menu_accel,UI_MENU_ACCEL,11+UI_MENU_BACKCNT);

/** \brief Feedrates */
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxx,  UI_TEXT_FEED_MAX_X,  UI_ACTION_MAX_FEEDRATE_X);
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxy,  UI_TEXT_FEED_MAX_Y,  UI_ACTION_MAX_FEEDRATE_Y);
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxz,  UI_TEXT_FEED_MAX_Z,  UI_ACTION_MAX_FEEDRATE_Z);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homex, UI_TEXT_FEED_HOME_X, UI_ACTION_HOMING_FEEDRATE_X);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homey, UI_TEXT_FEED_HOME_Y, UI_ACTION_HOMING_FEEDRATE_Y);
UI_MENU_CHANGEACTION(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z, UI_ACTION_HOMING_FEEDRATE_Z);
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxx,&ui_menu_feedrate_maxy,&ui_menu_feedrate_maxz,&ui_menu_feedrate_homex,&ui_menu_feedrate_homey,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate,UI_MENU_FEEDRATE,6+UI_MENU_BACKCNT);

/** \brief General configuration settings */

UI_MENU_ACTION2C(ui_menu_stepper2,UI_ACTION_STEPPER_INACTIVE,UI_TEXT_STEPPER_OFF2);
UI_MENU_ACTION2C(ui_menu_maxinactive2,UI_ACTION_MAX_INACTIVE,UI_TEXT_ALL_OFF2);
UI_MENU_CHANGEACTION(ui_menu_general_baud,UI_TEXT_BAUDRATE,UI_ACTION_BAUDRATE);
UI_MENU_ACTIONSELECTOR(ui_menu_general_stepper_inactive,UI_TEXT_STEPPER_OFF,ui_menu_stepper2);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_general_max_inactive,UI_TEXT_ALL_OFF,ui_menu_maxinactive2,MENU_MODE_PRINTER,0);

#if FEATURE_BEEPER
UI_MENU_ACTIONCOMMAND(ui_menu_general_beeper,UI_TEXT_BEEPER,UI_ACTION_BEEPER);
#define BEEPER_MODE_COUNT	1
#define BEEPER_MODE_ENTRY	,&ui_menu_general_beeper
#else
#define BEEPER_MODE_COUNT	0
#define BEEPER_MODE_ENTRY	
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
UI_MENU_ACTIONCOMMAND(ui_menu_light_mode,UI_TEXT_LIGHTS_ONOFF,UI_ACTION_LIGHTS_ONOFF);
#define LIGHT_MODE_ENTRY ,&ui_menu_light_mode
#define LIGHT_MODE_COUNT 1
#else
#define LIGHT_MODE_ENTRY
#define LIGHT_MODE_COUNT 0
#endif // FEATURE_CASE_LIGHT

UI_MENU_ACTIONCOMMAND(ui_menu_z_mode,UI_TEXT_Z_MODE,UI_ACTION_ZMODE);

#if FEATURE_MILLING_MODE
UI_MENU_ACTIONCOMMAND(ui_menu_operating_mode,UI_TEXT_OPERATING_MODE,UI_ACTION_OPERATING_MODE);
#define OPERATING_MODE_ENTRY ,&ui_menu_operating_mode
#define	OPERATING_MODE_COUNT 1

#if MOTHERBOARD == DEVICE_TYPE_RF1000
UI_MENU_ACTIONCOMMAND(ui_menu_z_endstop_type,UI_TEXT_Z_ENDSTOP_TYPE,UI_ACTION_Z_ENDSTOP_TYPE);
#define Z_ENDSTOP_TYPE_ENTRY ,&ui_menu_z_endstop_type
#define	Z_ENDSTOP_TYPE_COUNT 1
#else
#define Z_ENDSTOP_TYPE_ENTRY 
#define	Z_ENDSTOP_TYPE_COUNT 0
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if FEATURE_CONFIGURABLE_MILLER_TYPE
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_miller_type,UI_TEXT_MILLER_TYPE,UI_ACTION_MILLER_TYPE,MENU_MODE_MILLER,0);
#define MILLER_TYPE_ENTRY ,&ui_menu_miller_type
#define	MILLER_TYPE_COUNT 1
#else
#define MILLER_TYPE_ENTRY 
#define	MILLER_TYPE_COUNT 0
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if FEATURE_CONFIGURABLE_HOTEND_TYPE && MOTHERBOARD == DEVICE_TYPE_RF1000
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_hotend_type,UI_TEXT_HOTEND_TYPE,UI_ACTION_HOTEND_TYPE,MENU_MODE_PRINTER,0);
#define HOTEND_TYPE_ENTRY ,&ui_menu_hotend_type
#define	HOTEND_TYPE_COUNT 1
#else
#define HOTEND_TYPE_ENTRY 
#define	HOTEND_TYPE_COUNT 0
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE && MOTHERBOARD == DEVICE_TYPE_RF1000

#else

#define OPERATING_MODE_ENTRY 
#define	OPERATING_MODE_COUNT 0
#define Z_ENDSTOP_TYPE_ENTRY 
#define	Z_ENDSTOP_TYPE_COUNT 0
#define MILLER_TYPE_ENTRY 
#define	MILLER_TYPE_COUNT 0
#define HOTEND_TYPE_ENTRY 
#define	HOTEND_TYPE_COUNT 0

#endif // FEATURE_MILLING_MODE

#if NUM_EXTRUDER>1
UI_MENU_ACTION2C(ui_menu_extruder_offsetx2,UI_ACTION_EXTRUDER_OFFSET_X,UI_TEXT_EXTRUDER_OFFSET_X2);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_extruder_offset_x,UI_TEXT_EXTRUDER_OFFSET_X,ui_menu_extruder_offsetx2, MENU_MODE_PRINTER, MENU_MODE_MILLER);
UI_MENU_ACTION2C(ui_menu_extruder_offsety2,UI_ACTION_EXTRUDER_OFFSET_Y,UI_TEXT_EXTRUDER_OFFSET_Y2);
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_extruder_offset_y,UI_TEXT_EXTRUDER_OFFSET_Y,ui_menu_extruder_offsety2, MENU_MODE_PRINTER, MENU_MODE_MILLER);
#define EXTRUDER_OFFSET_TYPE_ENTRY_X ,&ui_menu_extruder_offset_x
#define EXTRUDER_OFFSET_TYPE_ENTRY_Y ,&ui_menu_extruder_offset_y
#define	EXTRUDER_OFFSET_TYPE_COUNT_XY 2
#else
#define EXTRUDER_OFFSET_TYPE_ENTRY_X
#define EXTRUDER_OFFSET_TYPE_ENTRY_Y
#define	EXTRUDER_OFFSET_TYPE_COUNT_XY 0
#endif // NUM_EXTRUDER>1

#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive BEEPER_MODE_ENTRY LIGHT_MODE_ENTRY OPERATING_MODE_ENTRY,&ui_menu_z_mode Z_ENDSTOP_TYPE_ENTRY HOTEND_TYPE_ENTRY MILLER_TYPE_ENTRY EXTRUDER_OFFSET_TYPE_ENTRY_X EXTRUDER_OFFSET_TYPE_ENTRY_Y}
UI_MENU(ui_menu_general,UI_MENU_GENERAL,4+UI_MENU_BACKCNT+BEEPER_MODE_COUNT+LIGHT_MODE_COUNT+OPERATING_MODE_COUNT+Z_ENDSTOP_TYPE_COUNT+HOTEND_TYPE_COUNT+MILLER_TYPE_COUNT+EXTRUDER_OFFSET_TYPE_COUNT_XY);

/** \brief Configuration menu */
UI_MENU_SUBMENU(ui_menu_conf_general, UI_TEXT_GENERAL,      ui_menu_general);
UI_MENU_SUBMENU(ui_menu_conf_accel,   UI_TEXT_ACCELERATION, ui_menu_accel);
UI_MENU_SUBMENU(ui_menu_conf_feed,    UI_TEXT_FEEDRATE,     ui_menu_feedrate);

#ifdef SOFTWARE_LEVELING
#define UI_MENU_SL_COND ,&ui_menu_conf_level
#define UI_MENU_SL_CNT 1
UI_MENU_SUBMENU(ui_menu_conf_level, UI_TEXT_LEVEL, ui_menu_level);
#else
#define UI_MENU_SL_COND
#define UI_MENU_SL_CNT 0
#endif // SOFTWARE_LEVELING

UI_MENU_SUBMENU(ui_menu_z_calibration, UI_TEXT_ZCALIB, ui_menu_z);
UI_MENU_ACTIONCOMMAND(ui_menu_restore_defaults,UI_TEXT_RESTORE_DEFAULTS,UI_ACTION_RESTORE_DEFAULTS);

#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK &ui_menu_conf_general,&ui_menu_conf_accel,&ui_menu_conf_feed, &ui_menu_z_calibration, &ui_menu_restore_defaults UI_MENU_SL_COND}
UI_MENU(ui_menu_configuration,UI_MENU_CONFIGURATION,UI_MENU_BACKCNT+UI_MENU_SL_CNT+5);

/** \brief Main menu */
UI_MENU_SUBMENU(ui_menu_main1, UI_TEXT_QUICK_SETTINGS, ui_menu_quick);
UI_MENU_SUBMENU(ui_menu_main2, UI_TEXT_POSITION,	   ui_menu_positions);
UI_MENU_SUBMENU_FILTER(ui_menu_main3, UI_TEXT_EXTRUDER,	ui_menu_extruder, MENU_MODE_PRINTER, 0);

#if SHOW_DEBUGGING_MENU
UI_MENU_SUBMENU(ui_menu_main4, UI_TEXT_DEBUGGING,	   ui_menu_debugging);
#define DEBUGGING_MENU_ENTRY	&ui_menu_main4,
#define DEBUGGING_MENU_COUNT	1
#else
#define DEBUGGING_MENU_ENTRY	
#define DEBUGGING_MENU_COUNT	0
#endif // SHOW_DEBUGGING_MENU

UI_MENU_SUBMENU(ui_menu_main5, UI_TEXT_CONFIGURATION,  ui_menu_configuration);

#if MOTHERBOARD == DEVICE_TYPE_RF2000
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK  &ui_menu_main1,&ui_menu_sd_print_file,&ui_menu_sd_mill_file,&ui_menu_main2,&ui_menu_main3,UI_MENU_FAN_COND UI_MENU_SD_COND DEBUGGING_MENU_ENTRY &ui_menu_main5}
UI_MENU(ui_menu_main,UI_MENU_MAIN,6+UI_MENU_BACKCNT+UI_MENU_SD_CNT+UI_MENU_FAN_CNT+DEBUGGING_MENU_COUNT);
#else
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK  &ui_menu_main1,&ui_menu_main2,&ui_menu_main3,UI_MENU_FAN_COND UI_MENU_SD_COND DEBUGGING_MENU_ENTRY &ui_menu_main5}
UI_MENU(ui_menu_main,UI_MENU_MAIN,5+UI_MENU_BACKCNT+UI_MENU_SD_CNT+UI_MENU_FAN_CNT+DEBUGGING_MENU_COUNT);
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000

/* Define menus accessible by action commands

You can create up to 10 user menus which are accessible by the action commands UI_ACTION_SHOW_USERMENU1 until UI_ACTION_SHOW_USERMENU10
Do this the same way as with the menus above or you use one of the above menus. Then add a define like

#define UI_USERMENU1 ui_menu_conf_feed

which assigns the menu stored in ui_menu_conf_feed to the action UI_ACTION_SHOW_USERMENU1. Make sure only to change the numbers and not the name of the define.

When do you need this? You might want a fast button to change the temperature. In the default menu you have no menu
to change the temperature and view it the same time. So you need to make an action menu for this like:
UI_MENU_ACTION4C(ui_menu_extrtemp,UI_ACTION_EXTRUDER0_TEMP,"Temp. 0  :%E0\002C","","","");
Then you assign this menu to a usermenu:
#define UI_USERMENU2 ui_menu_extrtemp

Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperture screen and allows
the change of temperature with the next/previous buttons. */

#endif // UI_MENU_H
