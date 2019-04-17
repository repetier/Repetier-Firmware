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

/****************************************************************************************
* Ultimaker Shield pin assignment v1.5.7
*
****************************************************************************************/
#if MOTHERBOARD == 37
#define ULTIMAKER_157
#define KNOWN_BOARD 1

#define ORIG_X_STEP_PIN 25
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 27

#define ORIG_Y_STEP_PIN 31
#define ORIG_Y_DIR_PIN 33
#define ORIG_Y_MIN_PIN 26
#define ORIG_Y_MAX_PIN 28
#define ORIG_Y_ENABLE_PIN 29

#define ORIG_Z_STEP_PIN 37
#define ORIG_Z_DIR_PIN 39
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 32
#define ORIG_Z_ENABLE_PIN 35

// bed
#define HEATER_1_PIN 4
#define TEMP_1_PIN 10

#define HEATER_0_PIN 2
#define TEMP_0_PIN 8

#define HEATER_2_PIN 3
#define TEMP_2_PIN 9

#define HEATER_3_PIN -1
#define TEMP_3_PIN -1

#define ORIG_E0_STEP_PIN 43
#define ORIG_E0_DIR_PIN 45
#define ORIG_E0_ENABLE_PIN 41
#define E0_FAN_PIN -1
//  #define EXT1_EXTRUDER_COOLER_PIN E0_FAN_PIN

#define ORIG_E1_STEP_PIN 49
#define ORIG_E1_DIR_PIN 47
#define ORIG_E1_ENABLE_PIN 48
#define E1_FAN_PIN -1
//  #define EXT2_EXTRUDER_COOLER_PIN E1_FAN_PIN

#define LED_PIN 13
#define ORIG_FAN_PIN 7
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1
//PIN that has to be turned on right after start, to keep power flowing.
#define SUICIDE_PIN 54

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51
#define SDPOWER -1
#define SDSS 53
#define ORIG_SDCARDDETECT 38

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#endif
