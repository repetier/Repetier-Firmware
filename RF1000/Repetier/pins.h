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


#ifndef PINS_H
#define PINS_H


#define CURRENT_CONTROL_DRV8711		4
#define BEEPER_PIN_RF1000 			23
#define BEEPER_PIN_RF2000 			5


// ##########################################################################################
// ##	RF1000 pin assignment
// ##########################################################################################
#if MOTHERBOARD == DEVICE_TYPE_RF1000
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

// Definition for current control
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_DRV8711

// On board beeper, so define values already here
#define BEEPER_PIN		 	    23
#define BEEPER_TYPE			     1
#define SDCARDDETECT			-1

// digital pin mappings
#define ORIG_X_STEP_PIN         54	// PINF.0, 97, STP_DRV1
#define ORIG_X_DIR_PIN          55	// PINF.1, 96, DIR_DRV1
#define ORIG_X_ENABLE_PIN       38	// PIND.7, 50, ENA_DRV1
#define ORIG_X_MIN_PIN           3	// PINE.5,  7, ES1
#define ORIG_X_MAX_PIN          -1   // not installed

#define ORIG_Y_STEP_PIN         60	// PINF.6, 91, STP_DRV2
#define ORIG_Y_DIR_PIN          61	// PINF.7, 90, DIR_DRV2
#define ORIG_Y_ENABLE_PIN       56	// PINF.2, 95, ENA_DRV2
#define ORIG_Y_MIN_PIN           2	// PINE.4,  6, ES2
#define ORIG_Y_MAX_PIN          -1  // not installed

#define ORIG_Z_STEP_PIN         46	// PINL.3, 38, STP_DRV3
#define ORIG_Z_DIR_PIN          48	// PINL.1, 36, DIR_DRV3
#define ORIG_Z_ENABLE_PIN       62	// PINK.0, 89, ENA_DRV3

// the RF1000 with miller functionality can provide min and max endstops at the same pin
#define ORIG_Z_MIN_PIN          31	// PINC.6, 59, ES3
#define ORIG_Z_MAX_PIN          31	// PINC.6, 59, ES3

#define ORIG_E0_STEP_PIN        26	// PINA.4, 74, STP_DRV4
#define ORIG_E0_DIR_PIN         28	// PINA.6, 72, DIR_DRV4
#define ORIG_E0_ENABLE_PIN      24	// PINA.2, 76, ENA_DRV4

#if NUM_EXTRUDER == 2
#define ORIG_E1_STEP_PIN        36	// PINC.1, 54, STP_DRV5
#define ORIG_E1_DIR_PIN         34	// PINC.3, 56, DIR_DRV5
#define ORIG_E1_ENABLE_PIN      30	// PINC.7, 60, ENA_DRV5
#else
#define ORIG_E1_STEP_PIN        -1
#define ORIG_E1_DIR_PIN         -1
#define ORIG_E1_ENABLE_PIN      -1
#endif // NUM_EXTRUDER == 2

#define SDPOWER				    -1
#define SDSS				    53	// PINB.0, 19, SS
#define LED_PIN				    13	// PINB.7, 26, LED13

#if PROTOTYPE_PCB == 1 
  #define ORIG_FAN_PIN          25	// PINA.3, 75, OUT1
#else
  #define ORIG_FAN_PIN          27   // PINA.5, 73, OUT2  
#endif

#define PS_ON_PIN			    -1
#define WATCHDOG_PIN		    37	// PINC.0

#define HEATER_0_PIN		    10	// PINB.4, 23, HZ1  left X4

#if NUM_EXTRUDER == 2
#define HEATER_1_PIN		     9	// PINH.6, 18, HZ2  right X8
#else
#define HEATER_1_PIN		    -1
#endif // NUM_EXTRUDER == 2

#define HEATER_2_PIN		     8	// PINH.5, 17, HZ3

// analog pin mappings
#define TEMP_0_PIN			    13   // PINK.5, 84, TH1  X5
#define TEMP_1_PIN			    14   // PINK.6, 83, TH2  X6
#define TEMP_2_PIN			    15   // PINK.7, 82, TH3

#define E0_PINS					ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS					ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

// these pins are defined in the SD library if building with SD support  
#define SCK_PIN				    52	// PINB.1, 20, SCK
#define MISO_PIN			    50	// PINB.3, 22, MISO
#define MOSI_PIN			    51	// PINB.2, 21, MOSI
#define MAX6675_SS			    53	// PINB.0, 19, SS

// motor driver pin mappings
#define X_SCS_PIN			    49	// PINL.0, 35, SCS_1
#define X_STALL_PIN			    41	// PING.0, 51, STALLn_1

#define Y_SCS_PIN			    47	// PINL.2, 37, SCS_2
#define Y_STALL_PIN			    40	// PING.1, 52, STALLn_2

#define Z_SCS_PIN			    45	// PINL.4, 39, SCS_3
#define Z_STALL_PIN			    39	// PING.2, 70, STALLn_3

#define O0_SCS_PIN			    43	// PINL.6, 41, SCS_4
#define O0_STALL_PIN		    95	// PING.3, 28, STALLn_4

#define O1_SCS_PIN			    84	// PINJ.7, 79, SCS_5
#define O1_STALL_PIN		    96	// PING.4, 29, STALLn_5

#define	DRV_FAULT			    97	// PINE.6, 8, FAULTn
#define DRV_RESET1			    90	// PINE.7, 9, RESET_DRV
#define DRV_SCLK			    93	// PIND.5, 48, SCLK
#define DRV_SDATO			    92	// PIND.4, 47, SDATO
#define DRV_SDATI			    94	// PIND.6, 49, SDATI

// key pin mappings
#define ENABLE_KEY_E1		    80	// PINJ.2, 65, TAST_E1
#define ENABLE_KEY_E2		    81	// PINJ.4, 67, TAST_E2
#define ENABLE_KEY_E3		    82	// PINJ.5, 68, TAST_E3
#define ENABLE_KEY_E4		    83	// PINJ.6, 69, TAST_E4
#define ENABLE_KEY_E5		    85	// PINH.7, 27, TAST_E5
#define ENABLE_KEY_E6		    86	// PINH.2, 14, TAST_E6
#define ENABLE_KEY_1			 4	// PING.5,  1, TAST1
#define ENABLE_KEY_2			 5	// PINE.3,  5, TAST2
#define ENABLE_KEY_3			 6	// PINH.3, 15, TAST3 
#define ENABLE_KEY_4			11	// PINB.5, 24, TAST4
#define ENABLE_KEY_5			42	// PINL.7, 42, TAST5

// servo pin mapping
#define SERVO0_PIN				35	// PINC.2, 55, PC2
#define SERVO1_PIN				33	// PINC.4, 57, PC4
#define SERVO2_PIN				32	// PINC.5, 58, PC5

// case light pin mapping
#define CASE_LIGHT_PIN			25	// PINA.3, 75, OUT1

// case fan pin mapping
#define	CASE_FAN_PIN			 9	// PINH.6, 18, HZ2

// display pin mapping
#define UI_DISPLAY_RS_PIN		63	// PINK.1, 88, D_RS
#define UI_DISPLAY_RW_PIN		-1
#define UI_DISPLAY_ENABLE_PIN	65	// PINK.3, 86, D_E
#define UI_DISPLAY_D0_PIN		-1
#define UI_DISPLAY_D1_PIN		-1
#define UI_DISPLAY_D2_PIN		-1
#define UI_DISPLAY_D3_PIN		-1
#define UI_DISPLAY_D4_PIN		59	// PINF.5, 92, D_D4
#define UI_DISPLAY_D5_PIN		64	// PINK.2, 87, D_D5
#define UI_DISPLAY_D6_PIN		44	// PINL.5, 40, D_D6
#define UI_DISPLAY_D7_PIN		66	// PINK.4, 85, D_D7

#define OUTPUT_230V_PIN			-1	
#define FET1					-1	
#define	FET2					-1	
#define	FET3					-1	

#endif // MOTHERBOARD == DEVICE_TYPE_RF1000


// ##########################################################################################
// ##	RF2000 pin assignment
// ##########################################################################################
#if MOTHERBOARD == DEVICE_TYPE_RF2000
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

// Definition for current control
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_DRV8711

// On board beeper, so define values already here
#define BEEPER_PIN		 		 5	// PINE.3,  5, BUZ1
#define BEEPER_TYPE			     1
#define SDCARDDETECT			19	// PIND.2, 45, CARD

// digital pin mappings
#define ORIG_X_STEP_PIN         54	// PINF.0, 97, STP_DRV1
#define ORIG_X_DIR_PIN          55	// PINF.1, 96, DIR_DRV1
#define ORIG_X_ENABLE_PIN       38	// PIND.7, 50, ENA_DRV1
#define ORIG_X_MIN_PIN          35	// PINC.2, 55, ES1
#define ORIG_X_MAX_PIN          -1   // not installed

#define ORIG_Y_STEP_PIN         60	// PINF.6, 91, STP_DRV2
#define ORIG_Y_DIR_PIN          61	// PINF.7, 90, DIR_DRV2
#define ORIG_Y_ENABLE_PIN       56	// PINF.2, 95, ENA_DRV2
#define ORIG_Y_MIN_PIN          37	// PINC.0, 53, ES2
#define ORIG_Y_MAX_PIN          -1  // not installed

#define ORIG_Z_STEP_PIN         57	// PINF.3, 94, STP_DRV3
#define ORIG_Z_DIR_PIN          48	// PINL.1, 36, DIR_DRV3
#define ORIG_Z_ENABLE_PIN       62	// PINK.0, 89, ENA_DRV3

// the RF2000 with miller functionality can provide min and max endstops
#define ORIG_Z_MIN_PIN          31	// PINC.6, 59, ES3
#define ORIG_Z_MAX_PIN          16	// PINH.1, 13, ES4

#define ORIG_E0_STEP_PIN        26	// PINA.4, 74, STP_DRV4
#define ORIG_E0_DIR_PIN         28	// PINA.6, 72, DIR_DRV4
#define ORIG_E0_ENABLE_PIN      24	// PINA.2, 76, ENA_DRV4

#define ORIG_E1_STEP_PIN        36	// PINC.1, 54, STP_DRV5
#define ORIG_E1_DIR_PIN         34	// PINC.3, 56, DIR_DRV5
#define ORIG_E1_ENABLE_PIN      30	// PINC.7, 60, ENA_DRV5

#define SDPOWER				    -1
#define SDSS				    53	// PINB.0, 19, SS
#define LED_PIN				    -1	

#define ORIG_FAN_PIN			27  // PINA.5, 73, OUT2  

#define PS_ON_PIN			    -1
#define WATCHDOG_PIN		    29	// PINA.7, 71, WDI

#define HEATER_0_PIN		    10	// PINB.4, 23, HZ1
#define HEATER_1_PIN		     9	// PINH.6, 18, HZ2
#define HEATER_2_PIN		    91	// PINE.2,  4, HZ3

// analog pin mappings
#define TEMP_0_PIN			    13  // PINK.5, 84, TH1
#define TEMP_1_PIN			    14  // PINK.6, 83, TH2
#define TEMP_2_PIN			    15  // PINK.7, 82, TH3
#define	TEMP_3_PIN				12	// PINK.4, 85, TH4

#define E0_PINS					ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS					ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

// these pins are defined in the SD library if building with SD support  
#define SCK_PIN				    52	// PINB.1, 20, SCK
#define MISO_PIN			    50	// PINB.3, 22, MISO
#define MOSI_PIN			    51	// PINB.2, 21, MOSI
#define MAX6675_SS				53	// PINB.0, 19, SS

// motor driver pin mappings
#define X_SCS_PIN			    49	// PINL.0, 35, SCS_1
#define X_STALL_PIN			    41	// PING.0, 51, STALLn_1

#define Y_SCS_PIN			    47	// PINL.2, 37, SCS_2
#define Y_STALL_PIN			    40	// PING.1, 52, STALLn_2

#define Z_SCS_PIN			    12	// PINB.6, 25, SCS_3
#define Z_STALL_PIN			    39	// PING.2, 70, STALLn_3

#define O0_SCS_PIN			    43	// PINL.6, 41, SCS_4
#define O0_STALL_PIN		    95	// PING.3, 28, STALLn_4

#define O1_SCS_PIN			    84	// PINJ.7, 79, SCS_5
#define O1_STALL_PIN		    96	// PING.4, 29, STALLn_5

#define	DRV_FAULT			    97	// PINE.6,  8, FAULTn
#define DRV_RESET1			    90	// PINE.7,  9, RESET_DRV
#define DRV_SCLK			    93	// PIND.5, 48, SCLK
#define DRV_SDATO			    92	// PIND.4, 47, SDATO
#define DRV_SDATI			    94	// PIND.6, 49, SDATI

// key pin mappings
#define ENABLE_KEY_E1		    80	// PINJ.2, 65, TAST_E1
#define ENABLE_KEY_E2		    81	// PINJ.4, 67, TAST_E2
#define ENABLE_KEY_E3		    82	// PINJ.5, 68, TAST_E3
#define ENABLE_KEY_E4		    83	// PINJ.6, 69, TAST_E4
#define ENABLE_KEY_E5		    85	// PINH.7, 27, TAST_E5
#define ENABLE_KEY_E6		    86	// PINH.2, 14, TAST_E6
#define ENABLE_KEY_1			 4	// PING.5,  1, TAST1
#define ENABLE_KEY_2			13 	// PINB.7, 26, TAST2
#define ENABLE_KEY_3			17 	// PINH.0, 12, TAST3
#define ENABLE_KEY_4			11	// PINB.5, 24, TAST4
#define ENABLE_KEY_5			42	// PINL.7, 42, TAST5

// servo pin mapping
#define SERVO1_PIN				46	// PINL.3, 38, SERVO_1 
#define SERVO2_PIN				45	// PINL.4, 39, SERVO_2 
#define SERVO3_PIN				44	// PINL.5, 40, SERVO_3 

// 230V output pin mapping
#define OUTPUT_230V_PIN			3	// PINE.5,  7, P230

// 3x 24V FET output pin mapping
#define FET1					33	// PINC.4, 57, OUT3
#define	FET2					32	// PINC.5, 58, OUT4
#define	FET3					58	// PINF.4, 93, OUT5

// RGB lights pin mapping
#define RGB_LIGHT_R_PIN			6	// PINH.3, 15, L_RT
#define RGB_LIGHT_G_PIN			7	// PINH.4, 16, L_GN
#define RGB_LIGHT_B_PIN			8	// PINH.5, 17, L_BL

// case light pin mapping
#define CASE_LIGHT_PIN			25	// PINA.3, 75, OUT1

// case fan pin mapping
#define	CASE_FAN_PIN			58	// PINF.4, 93, OUT5

// display pin mapping
#define UI_DISPLAY_RS_PIN		63	// PINK.1, 88, D_RS
#define UI_DISPLAY_RW_PIN		-1
#define UI_DISPLAY_ENABLE_PIN	65	// PINK.3, 86, D_E
#define UI_DISPLAY_D0_PIN		-1
#define UI_DISPLAY_D1_PIN		-1
#define UI_DISPLAY_D2_PIN		-1
#define UI_DISPLAY_D3_PIN		-1
#define UI_DISPLAY_D4_PIN		59	// PINF.5, 92, D_D4
#define UI_DISPLAY_D5_PIN		64	// PINK.2, 87, D_D5
#define UI_DISPLAY_D6_PIN		23	// PINA.1, 77, D_D6
#define UI_DISPLAY_D7_PIN		98	// PINJ.3, 66, D_D7


#endif // MOTHERBOARD == DEVICE_TYPE_RF2000


#ifndef SDSSORIG
#define SDSSORIG				-1
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN			-1
#endif

#if NUM_EXTRUDER==1
#define E1_PINS
#endif

#if NUM_EXTRUDER<3
#define E2_PINS
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED	0
#endif

// Original pin assignmats to be used in configuration tool
#define X_STEP_PIN				ORIG_X_STEP_PIN
#define X_DIR_PIN				ORIG_X_DIR_PIN
#define X_ENABLE_PIN			ORIG_X_ENABLE_PIN
#define X_MIN_PIN				ORIG_X_MIN_PIN
#define X_MAX_PIN				ORIG_X_MAX_PIN

#define Y_STEP_PIN				ORIG_Y_STEP_PIN
#define Y_DIR_PIN				ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN			ORIG_Y_ENABLE_PIN
#define Y_MIN_PIN				ORIG_Y_MIN_PIN
#define Y_MAX_PIN				ORIG_Y_MAX_PIN

#define Z_STEP_PIN				ORIG_Z_STEP_PIN
#define Z_DIR_PIN				ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN			ORIG_Z_ENABLE_PIN
#define Z_MIN_PIN				ORIG_Z_MIN_PIN
#define Z_MAX_PIN				ORIG_Z_MAX_PIN

#define E0_STEP_PIN				ORIG_E0_STEP_PIN
#define E0_DIR_PIN				ORIG_E0_DIR_PIN
#define E0_ENABLE_PIN			ORIG_E0_ENABLE_PIN

#define E1_STEP_PIN				ORIG_E1_STEP_PIN
#define E1_DIR_PIN				ORIG_E1_DIR_PIN
#define E1_ENABLE_PIN			ORIG_E1_ENABLE_PIN

#define E2_STEP_PIN				ORIG_E2_STEP_PIN
#define E2_DIR_PIN				ORIG_E2_DIR_PIN
#define E2_ENABLE_PIN			ORIG_E2_ENABLE_PIN

#define FAN_PIN					ORIG_FAN_PIN
#define FAN2_PIN				ORIG_FAN2_PIN


#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, ORIG_X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, PS_ON_PIN, \
                        HEATER_0_PIN, HEATER_1_PIN, ORIG_FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN, SDSS }

#endif // PINS_H