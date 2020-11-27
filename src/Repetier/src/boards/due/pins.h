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

#ifndef PINS_H
#define PINS_H

#define CPU_ARCH ARCH_ARM
// Ensure right board selection
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#if MOTHERBOARD == 401

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN 2
#define ORIG_X_ENABLE_PIN 38

#define ORIG_Y_STEP_PIN 60
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN 15
#define ORIG_Y_ENABLE_PIN 56

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19
#define ORIG_Z_ENABLE_PIN 62

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 10
// Due analog pin #
#define TEMP_0_PIN 11
#define HEATER_1_PIN 8
// Due analog pin #
#define TEMP_1_PIN 12
#define HEATER_2_PIN 9
// Due analog pin #
#define TEMP_2_PIN 13

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
// was 40 but seemed to be wrong!
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1
// 10 if using HW SPI. 53 if using SW SPI
#define SDSS 53
#define LED_PIN 13
#define ORIG_FAN_PIN -1
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif
#endif

// RADDS Board
// http://www.dr-henschke.de/RADDS_due.html
#if MOTHERBOARD == 402
#include "boards/radds.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_RAMPS_FD_INVERTED_HEATER || MOTHERBOARD == MOTHERBOARD_RAMPS_FD
#include "boards/ramps-fd.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_FELIX
#include "boards/felix.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_BAM_AND_DICE
#include "boards/bam_and_dice.h"
#endif

#if MOTHERBOARD == 408 || MOTHERBOARD == 413
#include "boards/smart_ramps.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_ULTRATRONICS
#include "boards/ultratronics.h"
#endif

#if MOTHERBOARD == 410 || MOTHERBOARD == 411
#include "boards/due3dom.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_STACKER_3D_SUPERBOARD
#include "boards/stacker_3d_superboard.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_RURAMPS4D
#include "boards/ruramps4d.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_SHASTA
#include "boards/shasta.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_IKS3D
#include "boards/iks3d.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV1 || MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV2 || MOTHERBOARD == MOTHERBOARD_ALLIGATOR_REV3
#include "boards/alligator.h"
#endif

#if MOTHERBOARD == MOTHERBOARD_USER_DEFINED_DUE
#define KNOWN_BOARD
#include "../extra/userpins.h"
#endif

#ifndef SDSSORIG
#define SDSSORIG -1
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED 0
#endif

//Available chip select pins for HW SPI are 4 10 52
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 77)
#if (SDSS == 10)
#define SPI_PIN 77
#define SPI_CHAN 0
#else
#if (SDSS == 52)
#define SPI_PIN 86
#define SPI_CHAN 2
#else // SDSS == 4
#if (SDSS == 4)
#define SPI_PIN 87
#define SPI_CHAN 1
#else //SDSS == 77
#define SPI_PIN 77
#define SPI_CHAN 0
#endif
#endif
#endif
#define MOSI_PIN 75
#define MISO_PIN 74
#define SCK_PIN 76
//#define DUE_SOFTWARE_SPI
#else
#ifndef NONSTANDARD_SDSS
#define DUE_SOFTWARE_SPI
#define SPI_PIN SDSS
#endif
/* could be any pin with software */
#ifndef MOSI_PIN
#define MOSI_PIN 51
#endif
#ifndef MISO_PIN
#define MISO_PIN 50
#endif
#ifndef SCK_PIN
#define SCK_PIN 52
#endif

#endif

// Original pin assignmats to be used in configuration tool
#define X_STEP_PIN ORIG_X_STEP_PIN
#define X_DIR_PIN ORIG_X_DIR_PIN
#define X_ENABLE_PIN ORIG_X_ENABLE_PIN
#define X_MIN_PIN ORIG_X_MIN_PIN
#define X_MAX_PIN ORIG_X_MAX_PIN

#define Y_STEP_PIN ORIG_Y_STEP_PIN
#define Y_DIR_PIN ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN ORIG_Y_ENABLE_PIN
#define Y_MIN_PIN ORIG_Y_MIN_PIN
#define Y_MAX_PIN ORIG_Y_MAX_PIN

#define Z_STEP_PIN ORIG_Z_STEP_PIN
#define Z_DIR_PIN ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN ORIG_Z_ENABLE_PIN
#define Z_MIN_PIN ORIG_Z_MIN_PIN
#define Z_MAX_PIN ORIG_Z_MAX_PIN

#define E0_STEP_PIN ORIG_E0_STEP_PIN
#define E0_DIR_PIN ORIG_E0_DIR_PIN
#define E0_ENABLE_PIN ORIG_E0_ENABLE_PIN

#define E1_STEP_PIN ORIG_E1_STEP_PIN
#define E1_DIR_PIN ORIG_E1_DIR_PIN
#define E1_ENABLE_PIN ORIG_E1_ENABLE_PIN

#define E2_STEP_PIN ORIG_E2_STEP_PIN
#define E2_DIR_PIN ORIG_E2_DIR_PIN
#define E2_ENABLE_PIN ORIG_E2_ENABLE_PIN

#define E3_STEP_PIN ORIG_E3_STEP_PIN
#define E3_DIR_PIN ORIG_E3_DIR_PIN
#define E3_ENABLE_PIN ORIG_E3_ENABLE_PIN

#define E4_STEP_PIN ORIG_E4_STEP_PIN
#define E4_DIR_PIN ORIG_E4_DIR_PIN
#define E4_ENABLE_PIN ORIG_E4_ENABLE_PIN

#define E5_STEP_PIN ORIG_E5_STEP_PIN
#define E5_DIR_PIN ORIG_E5_DIR_PIN
#define E5_ENABLE_PIN ORIG_E5_ENABLE_PIN

#define FAN_PIN ORIG_FAN_PIN
#ifdef ORIG_FAN2_PIN
#define FAN2_PIN ORIG_FAN2_PIN
#endif

#define PS_ON_PIN ORIG_PS_ON_PIN

#ifndef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT -1
#endif
#define SDCARDDETECT ORIG_SDCARDDETECT

#ifndef BEEPER_PIN
#define BEEPER_PIN -1
#endif

#define SENSITIVE_PINS \
    { \
        0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, ORIG_PS_ON_PIN, \
            HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN, SDSS \
    }
#endif
