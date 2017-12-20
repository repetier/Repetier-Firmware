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


#ifndef RF_H
#define RF_H


// ##########################################################################################
// ##	RF specific UI Actions
// ##########################################################################################

#define UI_ACTION_RF_MIN_REPEATABLE			 500

#define UI_ACTION_RF_HEAT_BED_UP			 514
#define UI_ACTION_RF_HEAT_BED_DOWN			 515
#define UI_ACTION_RF_EXTRUDER_OUTPUT		 516
#define UI_ACTION_RF_EXTRUDER_RETRACT		 517
#define UI_ACTION_RF_SET_Z_MATRIX_HEAT_BED	 518
#define	UI_ACTION_RF_SET_Z_MATRIX_WORK_PART	 519
#define UI_ACTION_RF_SET_SCAN_DELTA_X		 520
#define UI_ACTION_RF_SET_SCAN_DELTA_Y		 521

#define UI_ACTION_RF_MAX_REPEATABLE			 600

#define UI_ACTION_RF_MIN_SINGLE				1500

#define UI_ACTION_RF_TEST_COMPENSATION		1513
#define UI_ACTION_RF_PAUSE					1518
#define UI_ACTION_RF_CONTINUE				1519
#define UI_ACTION_RF_SCAN_HEAT_BED			1520
#define	UI_ACTION_RF_SCAN_HEAT_BED_PLA		1521
#define	UI_ACTION_RF_SCAN_HEAT_BED_ABS		1522
#define UI_ACTION_RF_PARK					1523
#define UI_ACTION_RF_RESET					1524
#define UI_ACTION_RF_RESET_ACK				1525
#define UI_ACTION_RF_OUTPUT_OBJECT			1526
#define UI_ACTION_RF_FIND_Z_ORIGIN			1527
#define UI_ACTION_RF_SCAN_WORK_PART			1528
#define UI_ACTION_RF_SET_SCAN_XY_START		1529
#define UI_ACTION_RF_SET_SCAN_XY_END		1530

#define UI_ACTION_RF_MAX_SINGLE				1600


/*
// ##########################################################################################
// ##	RF specific M codes
// ##########################################################################################

// ##########################################################################################
// ##	the following M codes are for the heat bed scan in operating mode "print"
// ##########################################################################################

- M3000 - turn the z-compensation off
  - Examples:
  - M3000 ; turns the z-compensation off

- M3001 - turn the z-compensation on
  - Examples:
  - M3001 ; turns the z-compensation on

- M3002 [S] - configure the min z-compensation scope ( S - units are [steps] )
  - Examples:
  - M3002 S1280 ; sets the min z-compensation scope to 1280 [steps] (= 0.5 [mm] in case RF_MICRO_STEPS is 32)

- M3003 [S] - configure the max z-compensation scope ( S - units are [steps] )
  - Examples:
  - M3003 S12800 ; sets the max z-compensation scope to 12800 [steps] (= 5.0 [mm] in case RF_MICRO_STEPS is 32)

- M3006 [S] [Z] - configure the static z-offset ( S - units are [um], Z - units are [mm] )
  - Examples:
  - M3006 S-100 ; sets the static z-offset to -100 [um] (= -0.1 [mm])
  - M3006 Z-0.1 ; sets the static z-offset to -0.1 [mm]

- M3007 [S] [Z] - configure the min z-compensation scope ( S - units are [um], Z - units are [mm] )
  - Examples:
  - M3007 S500 ; sets the min z-compensation scope to 500 [um] (= 0.5 [mm])
  - M3007 Z0.5 ; sets the min z-compensation scope to 0.5 [mm]

- M3008 [S] [Z] - configure the max z-compensation scope ( S - units are [um], Z - units are [mm] )
  - Examples:
  - M3008 S5000 ; sets the max z-compensation scope to 5000 [um] (= 5.0 [mm])
  - M3008 Z5 ; sets the max z-compensation scope to 5.0 [mm]

- M3009 [S] - get/choose the active heat bed z-compensation matrix
  - Examples:
  - M3009 ; outputs a log entry which informs about the currently active heat bed z matrix
  - M3009 S3 ; loads the heat bed z matrix no. 3 and applies it as currenlty active heat bed z matrix

- M3010 [S] - start/abort the heat bed scan
  - Examples:
  - M3010 ; starts a standard head bed scan or aborts the currently performed heat bed scan
  - M3010 S1 ; starts a heat bed scan which is optimized for future PLA prints
  - M3010 S2 ; starts a heat bed scan which is optimized for future ABS prints

- M3011 [S] - clear the specified z-compensation matrix from the EEPROM
  - Examples:
  - M3011 ; removes the currently active heat bed z matrix from the EEPROM
  - M3011 S3 ; removes the heat bed z matrix no. 3 from the EEPROM

- M3012 - restore the default scan parameters
  - Examples:
  - M3012 ; restores the default scan parameters to the values which are defined within RF1000.h/RF2000.h

- M3013 [S] [P] - output the current heat bed z-compensation matrix
  - Examples:
  - M3013 ; outputs the currently active heat bed z matrix, the unit of the output z offset is [steps]
  - M3013 P1 ; outputs the currently active heat bed z matrix, the unit of the output z offset is [mm]
  - M3013 S3 ; outputs the heat bed z matrix no. 3, the unit of the output z offset is [steps]
  - M3013 S3 P1 ; outputs the heat bed z matrix no. 3, the unit of the output z offset is [mm]

- M3020 [S] - configure the x start position for the heat bed scan ( units are [mm] )
  - Examples:
  - M3020 S15 ; the next heat bed scan starts at an x-position 15 [mm] from the left border

- M3021 [S] - configure the y start position for the heat bed scan ( units are [mm] )
  - Examples:
  - M3021 S30 ; the next heat bed scan starts at a y-position 30 [mm] from the front border

- M3022 [S] - configure the x step size for the heat bed scan ( units are [mm] )
  - Examples:
  - M3022 S20 ; the next heat bed scan performs one measurement each 20 [mm] in x-direction

- M3023 [S] - configure the y step size for the heat bed scan ( units are [mm] )
  - Examples:
  - M3023 S20 ; the next heat bed scan performs one measurement each 20 [mm] in y-direction

- M3024 [S] - configure the x end position for the heat bed scan ( units are [mm] )
  - Examples:
  - M3024 S5 ; the next heat bed scan ends at an x-position 5 [mm] from the right border

- M3025 [S] - configure the y end position for the heat bed scan ( units are [mm] )
  - Examples:
  - M3025 S5 ; the next heat bed scan ends at a y-position 5 [mm] from the back border


// ##########################################################################################
// ##	the following M codes are for the general configuration
// ##########################################################################################

- M3005 [S] - enable custom debug outputs
  - Examples:
  - M3005 S1 ; sets the debug level to 1

- M3030 [S] - configure the fast step size for moving of the heat bed up during the heat bed/work part scan
  - Examples:
  - M3030 S64 ; the fast upwards movements use 64 [steps] between two checks about whether a contact is detected

- M3031 [S] - configure the slow step size for moving of the heat bed up during the heat bed/work part scan
  - Examples:
  - M3031 S16 ; the slow upwards movements use 16 [steps] between two checks about whether a contact is detected

- M3032 [S] - configure the fast step size for moving of the heat bed down during the heat bed/work part scan
  - Examples:
  - M3032 S640 ; the fast downwards movements use 640 [steps] between two checks about whether the extruder/tool is free again

- M3033 [S] - configure the slow step size for moving of the heat bed down during the heat bed/work part scan
  - Examples:
  - M3033 S32 ; the slow downwards movements use 32 [steps] between two checks about whether the extruder/tool is free again

- M3040 [S] - configure the delay ( in ms ) between two fast movements during the heat bed/work part scan
  - Examples:
  - M3040 S5 ; during the heat bed/work part scan, the firmware waits 5 [ms] between two fast movements

- M3041 [S] - configure the delay ( in ms ) between two slow movements during the heat bed/work part scan
  - Examples:
  - M3041 S100 ; during the heat bed/work part scan, the firmware waits 100 [ms] between two slow movements

- M3042 [S] - configure the delay ( in ms ) between reaching of a new x/y position and the test of the idle pressure
  - Examples:
  - M3042 S250 ; during the heat bed/work part scan, the firmware waits 250 [ms] between reaching of a new x/y position and the test of the idle pressure

- M3050 [S] - configure the contact pressure delta ( units are [digits] )
  - Examples:
  - M3050 S10 ; during the heat bed/work part scan, a pressure difference of 10 [digits] is detected as contact

- M3051 [S] - configure the retry pressure delta ( units are [digits] )
  - Examples:
  - M3051 S5 ; during the heat bed/work part scan, a pressure difference of 5 [digits] must be reached in order to switch from the slow downwards movement to the slow upwards movement

- M3052 [S] - configure the idle pressure tolerance ( units are [digits] )
  - Examples:
  - M3052 S5 ; during the heat bed/work part scan, the idle pressure must not drift more than 5 [digits]

- M3053 [S] - configure the number of A/D converter reads per pressure measurement
  - Examples:
  - M3053 S15 ; during the heat bed/work part scan, each presure value shall be build up as average of 15 measurements

- M3054 [S] - configure the delay between two A/D converter reads ( units are [ms] )
  - Examples:
  - M3054 S15 ; during the heat bed/work part scan, it shall be waited 15 [ms] between every two measurements of the pressure

- M3055 [S] - configure the pressure tolerance per pressure measurement ( units are [digits] )
  - Examples:
  - M3055 S15 ; during the heat bed/work part scan, the difference between the minimal measured pressure and the maximal measured pressure at one measurement point must not exceed 15 [digits]

- M3060 - output the device type and firmware version
  - Examples:
  - M3060 ; outputs the device type and firmware version

- M3070 [S] - pause the print as if the "Pause" button would have been pressed
  - Examples:
  - M3070 S1 ; pauses the printing
  - M3070 S2 ; pauses the printing and moves away

- M3071 - wait until the print has been continued via the "Continue" button
  - Examples:
  - M3071 ; waits until the "Continue" button has been pressed (does nothing in case the print is not paused at the moment)

- M3075 [S] [P] - configure the emergency pause digits
  - Examples:
  - M3075 S-5000 ; sets the min emergency pause digits to -5000 [digits]
  - M3075 P5000 ; sets the max emergency pause digits to 5000 [digits]
  - M3075 S-5000 P5000 ; sets the min emergency pause digits to -5000 [digits] and the max emergency pause digits to 5000 [digits]

- M3079 - output the printed object
  - Examples:
  - M3079 ; outputs the printed object

- M3080 - park the printer
  - Examples:
  - M3080 ; parks the printer

- M3090 - test the watchdog ( this command resets the firmware )
  - Examples:
  - M3090 ; tests the watchdog (and resets the firmware)

- M3091 - erase the external EEPROM
  - Examples:
  - M3091 ; erases the external EEPROM

- M3100 [S] - configure the number of manual z steps after the "Heat Bed up" or "Heat Bed down" button has been pressed
  - Examples:
  - M3100 S100 ; sets the number of manual z steps after the "Heat Bed up" or "Heat Bed down" button has been pressed to 100 [steps]

- M3101 [S] - configure the number of manual extruder steps after the "Extruder output" or "Extruder retract" button has been pressed
  - Examples:
  - M3101 S100 ; sets the number of manual extruder steps after the "Extruder output" or "Extruder retract" button has been pressed to 100 [steps]

- M3102 [X] [Y] [Z] [E] - configure the offset in x, y, z and e direction which shall be applied in case the "Pause Printing" button has been pressed ( units are [steps] )
  - Examples:
  - M3102 X100 ; sets the x-offset in case of "Pause Printing" has been pressed to 100 [steps]
  - M3102 Y100 ; sets the y-offset in case of "Pause Printing" has been pressed to 100 [steps]
  - M3102 Z100 ; sets the z-offset in case of "Pause Printing" has been pressed to 100 [steps]
  - M3102 E500 ; sets the e-offset in case of "Pause Printing" has been pressed to 500 [steps]
  - M3102 X100 Y100 Z100 E500 ; sets the x/y/z/e-offset in case of "Pause Printing" has been pressed to 100/100/100/500 [steps]

- M3103 [X] [Y] [Z] - configure the x, y and z position which shall set when the printer is parked ( units are [mm] )
  - Examples:
  - M3103 X10 ; sets the x-position when the printer is parked to 10 [mm]
  - M3103 Y10 ; sets the y-position when the printer is parked to 10 [mm]
  - M3103 Z10 ; sets the z-position when the printer is parked to 10 [mm]
  - M3103 X10 Y10 Z10 ; sets the x-/y-/z-position when the printer is parked to 10/10/10 [mm]

- M3105 [X] [Y] [Z] [E] - configure the offset in x, y, z and e direction which shall be applied in case the "Pause Printing" button has been pressed ( units are [mm] )
  - Examples:
  - M3105 X10 ; sets the x-offset in case of "Pause Printing" has been pressed to 10 [mm]
  - M3105 Y10 ; sets the y-offset in case of "Pause Printing" has been pressed to 10 [mm]
  - M3105 Z10 ; sets the z-offset in case of "Pause Printing" has been pressed to 10 [mm]
  - M3105 E20 ; sets the x-offset in case of "Pause Printing" has been pressed to 20 [mm]

- M3115 - set the x/y origin to the current x/y position
  - Examples:
  - M3115 ; sets the x/y origin to the current x/y position

- M3117 [message] - Set a status text which is not overwritten by M117
  - Examples:
  - M3117 My RF rocks ; displays the text "My RF rocks" in the 4. row of the display until it is overwritten through another M3117
  - M3117 ; frees the 4. row of the display so that it can be used by M117 and other status messages again

- M3120 - turn on the case fan
  - Examples:
  - M3120 ; turns the case fan on

- M3121 - turn off the case fan
  - Examples:
  - M3121 ; turns the case fan off


// ##########################################################################################
// ##	the following M codes are for the work part scan in operating mode "mill"
// ##########################################################################################

- M3130 - start/stop the search of the z-origin
  - Examples:
  - M3130 ; starts the search of the z-origin or aborts the currently performed search of the z-origin

- M3140 - turn the z-compensation off
  - Examples:
  - M3140 ; turns the z-compensation off

- M3141 - turn the z-compensation on
  - Examples:
  - M3141 ; turns the z-compensation on

- M3146 [S] [Z] - configure the static z-offset ( S - units are [um], Z - units are [mm] )
  - Examples:
  - M3146 S-100 ; sets the static z-offset to -100 [um] (= -0.1 [mm])
  - M3146 Z-0.1 ; sets the static z-offset to -0.1 [mm]

- M3149 [S] - get/choose the active work part z-compensation matrix
  - Examples:
  - M3149 ; outputs a log entry which informs about the currently active work part z-compensation matrix
  - M3149 S3 ; loads the work part z-compensation matrix no. 3 and applies it as currently active work part z-compensation matrix

- M3150 [S] - start/abort the work part scan
  - Examples:
  - M3130 ; starts the work part scan without homning of the z-axis or aborts the currently performed work part scan
  - M3130 S1 ; starts the work part scan with homing of the z-axis 

- M3151 [S] - clear the specified z-compensation matrix from the EEPROM
  - Examples:
  - M3151 ; removes the currently active work part z-compensation matrix from the EEPROM
  - M3151 S3 ; removes the work part z-compensation matrix no. 3 from the EEPROM

- M3152 - restore the default scan parameters
  - Examples:
  - M3152 ; restores the default scan parameters to the values which are defined within RF1000.h/RF2000.h

- M3153 [S] [P] - output the current work part z-compensation matrix
  - Examples:
  - M3153 ; outputs the currently active work part z-compensation matrix, the unit of the output z offset is [steps]
  - M3153 P1 ; outputs the currently active work part z-compensation matrix, the unit of the output z offset is [mm]
  - M3153 S3 ; outputs the work part z-compensation matrix no. 3, the unit of the output z offset is [steps]
  - M3153 S3 P1 ; outputs the work part z-compensation matrix no. 3, the unit of the output z offset is [mm]

- M3160 [S] - configure the x start position for the work part scan ( units are [mm] )
  - Examples:
  - M3160 S15 ; the next work part scan starts at an x-position 15 [mm] from the left border

- M3161 [S] - configure the y start position for the work part scan ( units are [mm] )
  - Examples:
  - M3161 S30 ; the next work part scan starts at a y-position 30 [mm] from the front border

- M3162 [S] - configure the x step size for the work part scan ( units are [mm] )
  - Examples:
  - M3162 S20 ; the next work part scan performs one measurement each 20 [mm] in x-direction

- M3163 [S] - configure the y step size for the work part scan ( units are [mm] )
  - Examples:
  - M3163 S20 ; the next work part scan performs one measurement each 20 [mm] in y-direction

- M3164 [S] - configure the x end position for the work part scan ( units are [mm] )
  - Examples:
  - M3164 S10 ; the next work part scan ends at an x-position 10 [mm] from the right border

- M3165 [S] - configure the y end position for the work part scan ( units are [mm] )
  - Examples:
  - M3165 S10 ; the next work part scan ends at a y-position 10 [mm] from the back border


// ##########################################################################################
// ##	other M codes
// ##########################################################################################

- M3190 - start/abort the test of the strain gauge
  - Examples:
  - M3190 ; starts the test of the strain gauge or aborts the currently performed test of the strain gauge

- M3200 [P] [S] - reserved for test and debug


// ##########################################################################################
// ##	the following M codes are supported only by the RF2000
// ##########################################################################################

- M3300 [P] [S] - configure the 24V FET outputs ( on/off )
  - Examples:
  - M3300 P1 S0 ; turns the first 24V-FET output off
  - M3300 P1 S1 ; turns the first 24V-FET output on
  - M3300 P2 S0 ; turns the second 24V-FET output off
  - M3300 P2 S1 ; turns the second 24V-FET output on
  - M3300 P3 S0 ; turns the third 24V-FET output off
  - M3300 P3 S1 ; turns the third 24V-FET output on

- M3301 [S] - configure the 230V output ( on/off )
  - Examples:
  - M3301 S0 ; turns the 230V output off
  - M3301 S1 ; turns the 230V output on

- M3303 [P] [S] - configure the RGB light effects for heating
  - Examples:
  - M3303 P1 S100 ; while the printer is heating, the red led shows a brightness of 100, allowed range = 0 (min) ... 255 (max)
  - M3303 P2 S50 ; while the printer is heating, the green led shows a brightness of 50, allowed range = 0 (min) ... 255 (max)
  - M3303 P3 S200 ; while the printer is heating, the blue led shows a brightness of 200, allowed range = 0 (min) ... 255 (max)

- M3304 [P] [S] - configure the RGB light effects for printing
  - Examples:
  - M3304 P1 S50 ; while the printer is printing, the red led shows a brightness of 50, allowed range = 0 (min) ... 255 (max)
  - M3304 P2 S100 ; while the printer is printing, the green led shows a brightness of 100, allowed range = 0 (min) ... 255 (max)
  - M3304 P3 S150 ; while the printer is printing, the blue led shows a brightness of 150, allowed range = 0 (min) ... 255 (max)

- M3305 [P] [S] - configure the RGB light effects for cooling
  - Examples:
  - M3305 P1 S150 ; while the printer is cooling, the red led shows a brightness of 150, allowed range = 0 (min) ... 255 (max)
  - M3305 P2 S20 ; while the printer is cooling, the green led shows a brightness of 20, allowed range = 0 (min) ... 255 (max)
  - M3305 P3 S220 ; while the printer is cooling, the blue led shows a brightness of 220, allowed range = 0 (min) ... 255 (max)

- M3306 [P] [S] - configure the RGB light effects for idle
  - Examples:
  - M3306 P1 S250 ; while the printer is idle, the red led shows a brightness of 250, allowed range = 0 (min) ... 255 (max)
  - M3306 P2 S200 ; while the printer is idle, the green led shows a brightness of 200, allowed range = 0 (min) ... 255 (max)
  - M3306 P3 S150 ; while the printer is idle, the blue led shows a brightness of 150, allowed range = 0 (min) ... 255 (max)

- M3307 [P] [S] - configure the manual RGB light colors
  - Examples:
  - M3307 P1 S0 ; when the RGB light mode is "Manual", the red led shows a brightness of 0, allowed range = 0 (min) ... 255 (max)
  - M3307 P2 S100 ; when the RGB light mode is "Manual", the green led shows a brightness of 100, allowed range = 0 (min) ... 255 (max)
  - M3307 P3 S200 ; when the RGB light mode is "Manual", the blue led shows a brightness of 200, allowed range = 0 (min) ... 255 (max)
  
- M3308 [P] - configure the RGB light mode
  - Examples:
  - M3308 P0 ; sets the RGB light mode to "off"
  - M3308 P1 ; sets the RGB light mode to "White"
  - M3308 P2 ; sets the RGB light mode to "Auto"
  - M3308 P3 ; sets the RGB light mode to "Manual"
*/


// ##########################################################################################
// ##	X/Y/Z movements
// ##########################################################################################
/*
The following variables are used for movements in x/y/z direction:

- Printer::queuePositionTargetSteps[x/y/z]
  - unit is [steps]
  - holds the position which shall be reached through the g-codes which are currently within the queue
  - this is not the position to which the printer is heading at the moment, because the printer is heading always to one position (from one g-code) and the queue can contain several g-codes

- Printer::queuePositionLastSteps[x/y/z]
  - unit is [steps]
  - holds the last position which has been calculated as queuePositionTargetSteps
  - in most cases, the value of queuePositionLastSteps is identical to the value of queuePositionTargetSteps, the values of these variables are different only while a new position is calculated

- Printer::queuePositionLastMM[x/y/z]
  - unit is [mm]
  - the rest is identical to queuePositionLastSteps

- Printer::queuePositionCurrentSteps[x/y/z]
  - unit is [steps]
  - holds the position which has been reached through the movements from the queue
  - the value of queuePositionCurrentSteps represents the current position of the printer in x, y and z direction

- Printer::stepperDirection[x/y/z]
  - holds the direction of the axes as it is requested by the currently processed movement from the queue

- Printer::queuePositionCommandMM[x/y/z]
  - unit is [mm]
  - in most cases, the value of queuePositionCommandMM is identical to the value of queuePositionLastMM, the values of these variables are different only while a new command is processed

- Printer::directPositionTargetSteps[x/y/z]
  - unit is [steps]
  - holds the position which shall be reached through direct movements, e.g. from the manual buttons or from the direct pause/continue functionality

- Printer::directPositionLastSteps[x/y/z]
  - unit is [steps]
  - holds the last position which has been calculated as directPositionTargetSteps
  - in most cases, the value of directPositionLastSteps is identical to the value of directPositionTargetSteps, the values of these variables are different only while a new position is calculated

- Printer::directPositionCurrentSteps[x/y/z]
  - unit is [steps]
  - holds the position which has been reached through direct movements, e.g. from the manual buttons or from the direct pause/continue functionality

- Printer::originOffsetMM[x/y/z]
  - unit is [mm]
  - holds the offset to the real origin of each axis

- Printer::staticCompensationZ
  - unit is [steps]
  - in case the z-compensation is used for a milling operation and the z-origin for the current milling operation has been determined at a different x/y position than the z-origin for the work part scan,
    this variable holds the z-difference between these two x/y points
  - the z-difference is calculated on base of the stored z compensation matrix

- Printer::compensatedPositionTargetStepsZ
  - unit is [steps]
  - holds the position which shall be reached through the z-compensation

- Printer::compensatedPositionCurrentStepsZ
  - unit is [steps]
  - holds the position which has been reached through the z-compensation

- the current x/y position of the printer in [steps] is:
  - Printer::queuePositionCurrentSteps[x/y] + directPositionCurrentSteps[x/y]
  - note that an additional, extruder-dependent origin can be used/set
  - see also:
    - Printer::currentXPosition()
	- Printer::currentYPosition()

- the current z position of the printer in [steps] is:
  - Printer::queuePositionCurrentSteps[z] + directPositionCurrentSteps[z] + compensatedPositionCurrentStepsZ
  - see also:
    - Printer::currentZPosition()

*/


extern const char	ui_text_error[]					PROGMEM;
extern const char	ui_text_warning[]				PROGMEM;
extern const char	ui_text_information[]			PROGMEM;
extern const char	ui_text_set_origin[]			PROGMEM;
extern const char	ui_text_heat_bed_scan[]			PROGMEM;
extern const char	ui_text_work_part_scan[]		PROGMEM;
extern const char	ui_text_find_z_origin[]			PROGMEM;
extern const char	ui_text_output_object[]			PROGMEM;
extern const char	ui_text_park_heat_bed[]			PROGMEM;
extern const char	ui_text_pause[]					PROGMEM;
extern const char	ui_text_home[]					PROGMEM;
extern const char	ui_text_delete_file[]			PROGMEM;
extern const char	ui_text_z_compensation[]		PROGMEM;
extern const char	ui_text_change_mode[]			PROGMEM;
extern const char	ui_text_change_z_type[]			PROGMEM;
extern const char	ui_text_change_hotend_type[]	PROGMEM;
extern const char	ui_text_change_miller_type[]	PROGMEM;
extern const char	ui_text_x_axis[]				PROGMEM;
extern const char	ui_text_y_axis[]				PROGMEM;
extern const char	ui_text_z_axis[]				PROGMEM;
extern const char	ui_text_extruder[]				PROGMEM;
extern const char	ui_text_autodetect_pid[]		PROGMEM;
extern const char	ui_text_temperature_manager[]	PROGMEM;
extern const char	ui_text_home_unknown[]			PROGMEM;
extern const char	ui_text_saving_failed[]			PROGMEM;
extern const char	ui_text_operation_denied[]		PROGMEM;
extern const char	ui_text_emergency_pause[]		PROGMEM;
extern const char	ui_text_emergency_stop[]		PROGMEM;
extern const char	ui_text_invalid_matrix[]		PROGMEM;
extern const char	ui_text_min_reached[]			PROGMEM;
extern const char	ui_text_max_reached[]			PROGMEM;
extern const char	ui_text_temperature_wrong[]		PROGMEM;
extern const char	ui_text_timeout[]				PROGMEM;
extern const char	ui_text_sensor_error[]			PROGMEM;


#if FEATURE_HEAT_BED_Z_COMPENSATION

// determine the maximal needed size for the heat bed compensation
// in case also FEATURE_WORK_PART_Z_COMPENSATION is enabled, only the defined dimensions for the heat bed scan count (so it must be ensured that the dimensions of the heat bed compensation matrix are at least of the size of the work part compensation matrix)
#define	COMPENSATION_MATRIX_MAX_X			long((X_MAX_LENGTH_PRINT - HEAT_BED_SCAN_X_START_MM - HEAT_BED_SCAN_X_END_MM) / HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM + 4)
#define	COMPENSATION_MATRIX_MAX_Y			long((Y_MAX_LENGTH		 - HEAT_BED_SCAN_Y_START_MM - HEAT_BED_SCAN_Y_END_MM) / HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM + 4)

#define	COMPENSATION_MATRIX_SIZE			long(COMPENSATION_MATRIX_MAX_X * COMPENSATION_MATRIX_MAX_Y * 2 + EEPROM_OFFSET_MAXTRIX_START)	// [bytes]

#elif FEATURE_WORK_PART_Z_COMPENSATION

// determine the maximal needed size for the work part compensation
#define	COMPENSATION_MATRIX_MAX_X			long((X_MAX_LENGTH_MILL - WORK_PART_SCAN_X_START_MM - WORK_PART_SCAN_X_END_MM) / WORK_PART_SCAN_X_STEP_SIZE_MIN_MM + 4)
#define	COMPENSATION_MATRIX_MAX_Y			long((Y_MAX_LENGTH		- WORK_PART_SCAN_Y_START_MM - WORK_PART_SCAN_Y_END_MM) / WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM + 4)

#define	COMPENSATION_MATRIX_SIZE			long(COMPENSATION_MATRIX_MAX_X * COMPENSATION_MATRIX_MAX_Y * 2 + EEPROM_OFFSET_MAXTRIX_START)	// [bytes]

#endif // FEATURE_HEAT_BED_Z_COMPENSATION && FEATURE_WORK_PART_Z_COMPENSATION


extern	unsigned long	g_uLastCommandLoop;
extern	unsigned long	g_uStartOfIdle;

#if FEATURE_HEAT_BED_Z_COMPENSATION
extern	long			g_offsetZCompensationSteps;	// this is the minimal distance between the heat bed and the extruder at the moment when the z-min endstop is hit
extern	long			g_minZCompensationSteps;
extern	long			g_maxZCompensationSteps;
extern	long			g_diffZCompensationSteps;
extern	unsigned char	g_nHeatBedScanStatus;
extern	char			g_nActiveHeatBed;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_WORK_PART_Z_COMPENSATION
extern	char			g_nWorkPartScanStatus;
extern	char			g_nWorkPartScanMode;		// 0 = do not home z-axis, 1 = home z-axis
extern	char			g_nActiveWorkPart;
#endif // FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
extern	char			g_abortZScan;
extern	short			g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
extern	unsigned char	g_uZMatrixMax[2];
extern	long			g_nZScanZPosition;

#if FEATURE_PRECISE_HEAT_BED_SCAN
extern	char			g_nHeatBedScanMode;			// 1 = PLA, 2 = ABS
#endif // FEATURE_PRECISE_HEAT_BED_SCAN

extern	long			g_nScanXStepSizeMm;
extern	long			g_nScanXStepSizeSteps;
extern	long			g_nScanYStepSizeMm;
extern	long			g_nScanYStepSizeSteps;

extern	unsigned short	g_nScanContactPressureDelta;
extern	unsigned short	g_nScanRetryPressureDelta;
#endif // #if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

extern	long			g_staticZSteps;
extern	char			g_debugLevel;
extern	char			g_debugLog;
//extern	long			g_debugCounter[20];
//extern	short			g_debugCounter[12];
//extern	short			g_debugCounter[6];
extern	unsigned long	g_uStopTime;
extern	unsigned long	g_uBlockSDCommands;
//extern	short			g_debugInt16;
//extern	unsigned short	g_debugUInt16;
//extern	long			g_debugInt32;

// other configurable parameters
#if FEATURE_EXTENDED_BUTTONS
extern	unsigned long	g_nManualSteps[4];
#endif // FEATURE_EXTENDED_BUTTONS


#if FEATURE_PAUSE_PRINTING
extern	long			g_nPauseSteps[4];
extern	long			g_nContinueSteps[4];
extern	char			g_pauseStatus;
extern	char			g_pauseMode;
extern	unsigned long	g_uPauseTime;
extern	char			g_pauseBeepDone;
#endif // FEATURE_PAUSE_PRINTING


#if FEATURE_FIND_Z_ORIGIN
extern	char			g_nFindZOriginStatus;
extern	long			g_nZOriginPosition[3];
extern	int				g_nZOriginSet;
#endif // FEATURE_FIND_Z_ORIGIN


#if FEATURE_TEST_STRAIN_GAUGE
extern	char			g_nTestStrainGaugeStatus;
#endif // FEATURE_TEST_STRAIN_GAUGE


#if DEBUG_HEAT_BED_Z_COMPENSATION || DEBUG_WORK_PART_Z_COMPENSATION
extern	long			g_nLastZCompensationPositionSteps[3];
extern	long			g_nLastZCompensationTargetStepsZ;
extern	long			g_nZCompensationUpdates;
extern	long			g_nDelta[2];
extern	long			g_nStepSize[2];
extern	long			g_nTempXFront;
extern	long			g_nTempXBack;
extern	long			g_nNeededZ;
extern	unsigned char	g_uIndex[4];
extern	short			g_nMatrix[4];
extern	long			g_nZDeltaMin;
extern	long			g_nZDeltaMax;
extern	long			g_nZCompensationUpdateTime;
extern	long			g_nZCompensationDelayMax;
extern	long			g_nTooFast;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION || DEBUG_WORK_PART_Z_COMPENSATION

#if FEATURE_RGB_LIGHT_EFFECTS

extern unsigned char	g_uRGBHeatingR;
extern unsigned char	g_uRGBHeatingG;
extern unsigned char	g_uRGBHeatingB;
extern unsigned char	g_uRGBPrintingR;
extern unsigned char	g_uRGBPrintingG;
extern unsigned char	g_uRGBPrintingB;
extern unsigned char	g_uRGBCoolingR;
extern unsigned char	g_uRGBCoolingG;
extern unsigned char	g_uRGBCoolingB;
extern unsigned char	g_uRGBIdleR;
extern unsigned char	g_uRGBIdleG;
extern unsigned char	g_uRGBIdleB;
extern unsigned char	g_uRGBManualR;
extern unsigned char	g_uRGBManualG;
extern unsigned char	g_uRGBManualB;
extern unsigned char	g_uRGBCurrentR;
extern unsigned char	g_uRGBCurrentG;
extern unsigned char	g_uRGBCurrentB;
extern unsigned char	g_uRGBTargetR;
extern unsigned char	g_uRGBTargetG;
extern unsigned char	g_uRGBTargetB;
#endif // FEATURE_RGB_LIGHT_EFFECTS


// initRF()
extern void initRF( void );

// initStrainGauge()
extern void initStrainGauge( void );

// readStrainGauge()
extern short readStrainGauge( unsigned char uAddress );

#if FEATURE_HEAT_BED_Z_COMPENSATION
// startHeatBedScan()
extern void startHeatBedScan( void );

// scanHeatBed()
extern void scanHeatBed( void );

// testExtruderTemperature()
extern short testExtruderTemperature( void );

// testHeatBedTemperature()
extern short testHeatBedTemperature( void );

// doHeatBedZCompensation()
extern void doHeatBedZCompensation( void );

// getHeatBedOffset()
extern long getHeatBedOffset( void );

// switchActiveHeatBed()
extern void switchActiveHeatBed( char newActiveHeatBed );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_WORK_PART_Z_COMPENSATION
// startWorkPartScan()
extern void startWorkPartScan( char nMode );

// scanWorkPart()
extern void scanWorkPart( void );

// doWorkPartZCompensation()
extern void doWorkPartZCompensation( void );

// getWorkPartOffset()
extern long getWorkPartOffset( void );

// determineStaticCompensationZ()
extern void determineStaticCompensationZ( void );
#endif // FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
// readIdlePressure()
extern short readIdlePressure( short* pnIdlePressure );

// testIdlePressure()
extern short testIdlePressure( void );

// readAveragePressure()
extern short readAveragePressure( short* pnAveragePressure );

// moveZUpFast()
extern short moveZUpFast( void );

// moveZDownSlow()
extern short moveZDownSlow( void );

// moveZUpSlow()
extern short moveZUpSlow( short* pnContactPressure, char* pnRetry );

// moveZDownFast()
extern short moveZDownFast( void );

// moveZ()
extern int moveZ( int nSteps );

// freeZ()
void freeZ( int nSteps );

// moveExtruder()
extern int moveExtruder( int nSteps );

// restoreDefaultScanParameters()
extern void restoreDefaultScanParameters( void );

// outputScanParameters()
extern void outputScanParameters( void );

// outputCompensationMatrix()
extern void outputCompensationMatrix( char format = 0 );

// initCompensationMatrix()
extern void initCompensationMatrix( void );

// prepareCompensationMatrix()
extern char prepareCompensationMatrix( void );

// determineCompensationOffsetZ()
extern char determineCompensationOffsetZ( void );

// adjustCompensationMatrix()
extern char adjustCompensationMatrix( short nZ );

// saveCompensationMatrix()
extern char saveCompensationMatrix( unsigned int uAddress );

// loadCompensationMatrix()
extern char loadCompensationMatrix( unsigned int uAddress );

// clearCompensationMatrix()
extern void clearCompensationMatrix( unsigned int uAddress );

// outputPressureMatrix()
extern void outputPressureMatrix( void );

#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

// clearExternalEEPROM()
extern char clearExternalEEPROM( void );

// writeByte24C256()
extern void writeByte24C256( int addressI2C, unsigned int addressEEPROM, unsigned char data );

// writeWord24C256()
extern void writeWord24C256( int addressI2C, unsigned int addressEEPROM, unsigned short data );

// readByte24C256()
extern unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM );

// readWord24C256()
extern unsigned short readWord24C256( int addressI2C, unsigned int addressEEPROM );

// doZCompensation()
extern void doZCompensation( void );

// loopRF()
extern void loopRF( void );

#if FEATURE_OUTPUT_FINISHED_OBJECT
// outputObject()
extern void outputObject( void );
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FEATURE_PARK
// parkPrinter()
extern void parkPrinter( void );
#endif // FEATURE_PARK

// pausePrint()
extern void pausePrint( void );

// continuePrint()
extern void continuePrint( void );

#if FEATURE_PAUSE_PRINTING
// determinePausePosition()
extern void determinePausePosition( void );

// determineZPausePositionForPrint()
extern void determineZPausePositionForPrint( void );

// determineZPausePositionForMill()
extern void determineZPausePositionForMill( void );

// waitUntilContinue
extern void waitUntilContinue( void );
#endif // FEATURE_PAUSE_PRINTING

// setExtruderCurrent()
extern void setExtruderCurrent( unsigned short level );

// processCommand()
extern void processCommand( GCode* pCommand );

// runStandardTasks()
extern void runStandardTasks( void );

// queueTask()
extern void queueTask( char task );

// processButton()
extern void processButton( int nAction );

// nextPreviousXAction()
extern void nextPreviousXAction( int8_t increment );

// nextPreviousYAction()
extern void nextPreviousYAction( int8_t increment );

// nextPreviousZAction()
extern void nextPreviousZAction( int8_t increment );


#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DRV8711
// setMotorCurrent()
extern void setMotorCurrent( unsigned char driver, unsigned short level );

// motorCurrentControlInit()
extern void motorCurrentControlInit( void );
#endif // CURRENT_CONTROL_DRV8711


// cleanupXPositions
extern void cleanupXPositions( void );

// cleanupYPositions
extern void cleanupYPositions( void );

// cleanupZPositions
extern void cleanupZPositions( void );

// cleanupEPositions
extern void cleanupEPositions( void );

// setZOrigin()
extern void setZOrigin( void );


#if FEATURE_FIND_Z_ORIGIN
// startFindZOrigin()
extern void startFindZOrigin( void );

// findZOrigin()
extern void findZOrigin( void );
#endif // FEATURE_FIND_Z_ORIGIN


#if FEATURE_TEST_STRAIN_GAUGE
// startTestStrainGauge()
extern void startTestStrainGauge( void );

// testStrainGauge()
extern void testStrainGauge( void );
#endif // FEATURE_TEST_STRAIN_GAUGE


#if FEATURE_MILLING_MODE
// switchOperatingMode()
extern void switchOperatingMode( char newOperatingMode );

// switchActiveWorkPart()
extern void switchActiveWorkPart( char newActiveWorkPart );

// setScanXYStart()
extern void setScanXYStart( void );

// setScanXYEnd()
extern void setScanXYEnd( void );
#endif // FEATURE_MILLING_MODE

#if FEATURE_RGB_LIGHT_EFFECTS
// setRGBTargetColors()
extern void setRGBTargetColors( uint8_t R, uint8_t G, uint8_t B );

// setRGBLEDs()
extern void setRGBLEDs( uint8_t R, uint8_t G, uint8_t B );

// updateRGBLightStatus()
extern void updateRGBLightStatus( void );
#endif // FEATURE_RGB_LIGHT_EFFECTS


// setupForPrinting
extern void setupForPrinting( void );

// setupForMilling()
extern void setupForMilling( void );

// prepareZCompensation()
extern void prepareZCompensation( void );

// resetZCompensation()
extern void resetZCompensation( void );

// isSupportedGCommand()
extern unsigned char isSupportedGCommand( unsigned int currentGCode, char neededMode, char outputLog = 1 );

// isSupportedMCommand()
extern unsigned char isSupportedMCommand( unsigned int currentMCode, char neededMode, char outputLog = 1 );

// isMovingAllowed()
extern unsigned char isMovingAllowed( const char* pszCommand, char outputLog = 1 );

// isHomingAllowed()
extern unsigned char isHomingAllowed( GCode* com, char outputLog = 1 );

// showInvalidSyntax()
extern void showInvalidSyntax( unsigned int currentMCode );

// addUInt32()
extern void addUInt32( char* pszString, uint32_t uNumber );

// addFloat()
extern void addFloat( char* pszString, float fNumber, uint8_t uDigits );


#if FEATURE_HEAT_BED_TEMP_COMPENSATION
// getHeatBedTemperatureOffset
extern float getHeatBedTemperatureOffset( float temperatureInCelsius );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION


#if FEATURE_TYPE_EEPROM
// determineHardwareType()
extern void determineHardwareType( void );

// notifyAboutWrongHardwareType()
extern void notifyAboutWrongHardwareType( unsigned char guessedHardwareType );
#endif // FEATURE_TYPE_EEPROM

// showIdle()
extern void showIdle( void );

// showError()
extern void showError( void* line2, void* line3 = NULL, void* line4 = NULL );

// showWarning()
extern void showWarning( void* line2, void* line3 = NULL, void* line4 = NULL );

// showInformation()
extern void showInformation( void* line2, void* line3 = NULL, void* line4 = NULL );

// dump()
extern void dump( char type, char from = 0 );

// doEmergencyStop()
void doEmergencyStop( char reason );


#endif // RF_H