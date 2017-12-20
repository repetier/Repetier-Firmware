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


#ifndef CONSTANTS_H
#define CONSTANTS_H


#define REPETIER_VERSION					"RF.01.38"
#define UI_PRINTER_COMPANY					"Conrad SE"
#define UI_VERSION_STRING					"V " REPETIER_VERSION


// ##########################################################################################
// ##    basic definitions
// ##########################################################################################

#define	DEVICE_TYPE_RF1000					13
#define DEVICE_TYPE_RF2000					14

#define	OPERATING_MODE_PRINT				1	// the firmware works in mode "print"
#define OPERATING_MODE_MILL					2	// the firmware works in mode "mill"

#define	HOTEND_TYPE_1						1	// hotend V1 + messing ring
#define	HOTEND_TYPE_V1						2	// hotend V1
#define	HOTEND_TYPE_V2_SINGLE				3	// hotend V2 for single extruder
#define	HOTEND_TYPE_V2_DUAL					4	// hotend V2 for dual extruder

#define MILLER_TYPE_ONE_TRACK				1	// one track in x- and y-direction
#define MILLER_TYPE_TWO_TRACKS				2	// two tracks in x- and y-direction

#define	ENDSTOP_TYPE_SINGLE					1	// there is only one endstop attached (either the min- or the max-endstop)
#define	ENDSTOP_TYPE_CIRCUIT				2	// the min- and max-endstops are attached in a single circuit

#define	ENDSTOP_NOT_HIT						0
#define	ENDSTOP_IS_HIT						1
#define	ENDSTOP_WAS_HIT						2

#define X_AXIS								0
#define Y_AXIS								1
#define Z_AXIS								2
#define E_AXIS								3
#define VIRTUAL_AXIS						4

#define HOME_ORDER_XYZ						1
#define HOME_ORDER_XZY						2
#define HOME_ORDER_YXZ						3
#define HOME_ORDER_YZX						4
#define HOME_ORDER_ZXY						5
#define HOME_ORDER_ZYX						6

#define MENU_MODE_SD_MOUNTED				1
#define MENU_MODE_SD_PRINTING				2
#define MENU_MODE_SD_PAUSED					4
#define MENU_MODE_FAN_RUNNING				8
#define MENU_MODE_PRINTING					16
#define MENU_MODE_PRINTER					32	// we have to show the printer menu
#define	MENU_MODE_MILLER					64	// we have to show the miller menu

#define IGNORE_COORDINATE					99999

#define	TASK_NO_TASK						0
#define	TASK_ENABLE_Z_COMPENSATION			1
#define	TASK_DISABLE_Z_COMPENSATION			2
#define TASK_PAUSE_PRINT					4
#define TASK_PAUSE_PRINT_AND_MOVE			5
#define	TASK_MOVE_FROM_BUTTON				10

#define	PAUSE_STATUS_NONE					0	// we are not paused at the moment
#define PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE	1	// we are waiting until the last queue move is finished
#define	PAUSE_STATUS_PREPARE_PAUSE_1		2	// we are going to pause the print (= moving to the pause position)
#define	PAUSE_STATUS_PREPARE_PAUSE_2		3	// we are going to pause the print (= moving to the pause position)
#define	PAUSE_STATUS_PREPARE_PAUSE_3		4	// we are going to pause the print (= moving to the pause position)
#define PAUSE_STATUS_PAUSED					5	// we have paused the print (= pause position reached)
#define	PAUSE_STATUS_PREPARE_CONTINUE_1		6	// we are going to continue the print (= moving to the continue position)
#define	PAUSE_STATUS_PREPARE_CONTINUE_2		7	// we are going to continue the print (= moving to the continue position)

#define	PAUSE_MODE_NONE						0	// we are not paused at the moment
#define	PAUSE_MODE_PAUSED					1	// stopp at the last printing position
#define	PAUSE_MODE_PAUSED_AND_MOVED			2	// move away from the last printing position

#define	RGB_MODE_OFF						0
#define	RGB_MODE_WHITE						1
#define RGB_MODE_AUTOMATIC					2
#define RGB_MODE_MANUAL						3

#define	RGB_STATUS_NOT_AUTOMATIC			0
#define	RGB_STATUS_AUTOMATIC				1
#define	RGB_STATUS_PRINTING				    11
#define	RGB_STATUS_HEATING					12
#define	RGB_STATUS_COOLING					13
#define	RGB_STATUS_IDLE						14
#define	RGB_STATUS_COLOR_CHANGE				15

#define MOVE_MODE_SINGLE_STEPS				1
#define MOVE_MODE_SINGLE_MOVE				2
#define MOVE_MODE_1_MM						3
#define MOVE_MODE_10_MM						4
#define MOVE_MODE_50_MM						5

#define HEAT_BED_SCAN_MODE_PLA				1
#define HEAT_BED_SCAN_MODE_ABS				2

#define Z_VALUE_MODE_Z_MIN					1	// show the z-distance to z-min (print)
#define Z_VALUE_MODE_Z_ORIGIN				1	// show the z-distance to the z-origin (mill)
#define Z_VALUE_MODE_SURFACE				2	// show the z-distance to the surface of the heat bed (print) or work part (mill)

#define	STOP_BECAUSE_OF_Z_MIN				1
#define	STOP_BECAUSE_OF_Z_BLOCK				2


// ##########################################################################################
// ##    data types
// ##########################################################################################

#define uint								uint16_t
#define uint8								uint8_t
#define int8								int8_t
#define uint32								uint32_t
#define int32								int32_t


#endif // CONSTANTS_H