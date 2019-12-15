#ifndef PINS_H
#define PINS_H

/*
The board assignment defines the capabilities of the motherboard and the used
pins. Each board definition follows the following scheme:

CPU_ARCH
  ARCH_AVR for AVR based boards
  ARCH_ARM for all arm based boards

STEPPER_CURRENT_CONTROL
  CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
  CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
  CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master
  CURRENT_CONTROL_ALLIGATOR 4  //Use External DAC like Alligator
*/

#define ARCH_AVR 1
#define ARCH_ARM 2
#define CPU_ARCH ARCH_ARM
#define CURRENT_CONTROL_MANUAL 1    // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2   // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3   // Use LTC2600 like Foltyn 3D Master
#define CURRENT_CONTROL_ALLIGATOR 4 // Use External DAC like Alligator
#define CURRENT_CONTROL_TMC2130 5   // Trinamic TMC2130 configured via SPI

/*
  arm does not have a eeprom build in. Therefore boards can add a
  eeprom. Board definition must set the right type of eeprom
*/

#define EEPROM_NONE 0
#define EEPROM_I2C 1
#define EEPROM_SPI_ALLIGATOR 2
#define EEPROM_SDCARD 3

#if MOTHERBOARD == 401
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

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
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

// RADDS Board
// http://www.dr-henschke.de/RADDS_due.html
#if MOTHERBOARD == 402
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 24
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 28
#define ORIG_X_MAX_PIN 34
#define ORIG_X_ENABLE_PIN 26

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 30
#define ORIG_Y_MAX_PIN 36
#define ORIG_Y_ENABLE_PIN 22

#define ORIG_Z_STEP_PIN 2
#define ORIG_Z_DIR_PIN 3
#define ORIG_Z_MIN_PIN 32
#define ORIG_Z_MAX_PIN 38
#define ORIG_Z_ENABLE_PIN 15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 13
// Due analog pin #54
#define TEMP_0_PIN 7
#define HEATER_1_PIN 7
#define TEMP_1_PIN 3
// Due analog pin #58
#define HEATER_2_PIN 12
// Due analog pin #55
#define TEMP_2_PIN 6
#define HEATER_3_PIN 11
// Due analog pin #56
#define TEMP_3_PIN 5
// Due analog pin #57
#define TEMP_4_PIN 4

// Due analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 2
// There are no more analog pins freely available.
// You can use direction and enable pin from extruder 0 socket as they are also
// analog pins. Then you need to move the stepper driver to a different socket.

// Direction pin of extruder 0
#define THERMOCOUPLE_1_PIN 1
// Step pin of extruder 0
#define THERMOCOUPLE_2_PIN 0
// Enable pin of extruder 0
#define THERMOCOUPLE_3_PIN 10

#define ORIG_E0_STEP_PIN 61
#define ORIG_E0_DIR_PIN 60
#define ORIG_E0_ENABLE_PIN 62

#define ORIG_E1_STEP_PIN 64
#define ORIG_E1_DIR_PIN 63
#define ORIG_E1_ENABLE_PIN 65

#define ORIG_E2_STEP_PIN 51
#define ORIG_E2_DIR_PIN 53
#define ORIG_E2_ENABLE_PIN 49

// Extra driver on extension board
// Might require pin 66 high for some drivers!
#define ORIG_E3_STEP_PIN 35
#define ORIG_E3_DIR_PIN 33
#define ORIG_E3_ENABLE_PIN 37

// Extra driver on extension port
// Might require pin 25 high for some drivers!
#define ORIG_E4_STEP_PIN 29
#define ORIG_E4_DIR_PIN 27
#define ORIG_E4_ENABLE_PIN 31

#define ORIG_E5_STEP_PIN 67
#define ORIG_E5_DIR_PIN 66
#define ORIG_E5_ENABLE_PIN 68

#define EXTENSION_BOARD_MS1 67
#define EXTENSION_BOARD_MS2 68
#define EXTENSION_BOARD_MS3 69
// 66 -> not connected
// 25 -> not connected
// To set microstepping on startup set START_GCODE to e.g.
// "M42 P67 S255\nM42 P68 S255\nM42 P69 S255"

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN 8
#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

// Servo pins: 5,6 und 39

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,
#define E5_PINS ORIG_E5_STEP_PIN, ORIG_E5_DIR_PIN, ORIG_E5_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

/****************************************************************************/
// RAMPS-FD Board
//
#if MOTHERBOARD == 403 || MOTHERBOARD == 404
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#if MOTHERBOARD == 403
#define HEATER_PINS_INVERTED 1 // only old boards had the output inverted
#else
#define HEATER_PINS_INVERTED 0
#endif

/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 63
#define ORIG_X_DIR_PIN 62
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 48

#define ORIG_Y_STEP_PIN 65
#define ORIG_Y_DIR_PIN 64
#define ORIG_Y_MIN_PIN 24
#define ORIG_Y_MAX_PIN 38
#define ORIG_Y_ENABLE_PIN 46

#define ORIG_Z_STEP_PIN 67
#define ORIG_Z_DIR_PIN 66
#define ORIG_Z_MIN_PIN 26
#define ORIG_Z_MAX_PIN 34
#define ORIG_Z_ENABLE_PIN 44

// Caution - Heater 0 and 1 are likely reversed compared with other boards,
// so you might need to assign HEATER_0_PIN to the heated bed.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 8
// Due analog pin #54
#define TEMP_0_PIN 7

#define HEATER_1_PIN 9
// Due analog pin #55
#define TEMP_1_PIN 6

#define HEATER_2_PIN 10
// Due analog pin #56
#define TEMP_2_PIN 5

#define HEATER_3_PIN 11
// Due analog pin #57
#define TEMP_3_PIN 4

// Due analog pin #58
#define TEMP_4_PIN 3

#define ORIG_E0_STEP_PIN 36
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 42

#define ORIG_E1_STEP_PIN 43
#define ORIG_E1_DIR_PIN 41
#define ORIG_E1_ENABLE_PIN 39

#define ORIG_E2_STEP_PIN 32
#define ORIG_E2_DIR_PIN 47
#define ORIG_E2_ENABLE_PIN 45

//#define SDSUPPORT      false
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
//#define SDSS		   -1
//#define ORIG_SDCARDDETECT   -1
#define SDCARDDETECTINVERTED false
#define LED_PIN -1
#define ORIG_FAN_PIN 12
#define ORIG_FAN2_PIN 2
#define ORIG_PS_ON_PIN 53
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

/*****************************************************************
 * Felix Printers Due Board
 * http://www.felixprinters.com
 ******************************************************************/
#if MOTHERBOARD == 405
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 24
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 34
#define ORIG_X_MAX_PIN 34
#define ORIG_X_ENABLE_PIN 26

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 30
#define ORIG_Y_MAX_PIN 30
#define ORIG_Y_ENABLE_PIN 22

#define ORIG_Z_STEP_PIN 2
#define ORIG_Z_DIR_PIN 3
#define ORIG_Z_MIN_PIN 32
#define ORIG_Z_MAX_PIN 32
#define ORIG_Z_ENABLE_PIN 15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 8
#define TEMP_0_PIN 6
#define HEATER_1_PIN 7
#define TEMP_1_PIN 7
#define HEATER_2_PIN 9
#define TEMP_2_PIN 5
#define HEATER_3_PIN -1
#define TEMP_3_PIN -1 // Due analog pin #56
#define TEMP_4_PIN -1 // Due analog pin #57

#define THERMOCOUPLE_0_PIN -1 // Dua analog pin #59 = A5 -> AD 2

#define ORIG_E0_STEP_PIN 61
#define ORIG_E0_DIR_PIN 60
#define ORIG_E0_ENABLE_PIN 62

#define ORIG_E1_STEP_PIN 64
#define ORIG_E1_DIR_PIN 63
#define ORIG_E1_ENABLE_PIN 65

#define ORIG_E2_STEP_PIN -1
#define ORIG_E2_DIR_PIN -1
#define ORIG_E2_ENABLE_PIN -1

#define ORIG_E3_STEP_PIN -1
#define ORIG_E3_DIR_PIN -1
#define ORIG_E3_ENABLE_PIN -1

#define ORIG_E4_STEP_PIN -1
#define ORIG_E4_DIR_PIN -1
#define ORIG_E4_ENABLE_PIN -1

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 1
#define LED_PIN -1
#define ORIG_FAN_PIN 11
#define ORIG_FAN2_PIN -1
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

#define SDA_PIN 20 // 20 or 70
#define SCL_PIN 21 // 21 or 71

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

/*****************************************************************
 * BAM&DICE Due Board with Arduino Due
 * http://www.2printbeta.de
 ******************************************************************/

#if MOTHERBOARD == 406
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_MANUAL

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
#define ORIG_Y_MIN_PIN 43
#define ORIG_Y_MAX_PIN 45
#define ORIG_Y_ENABLE_PIN 56

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_MIN_PIN 40
#define ORIG_Z_MAX_PIN 42
#define ORIG_Z_ENABLE_PIN 62

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 10
// Due analog pin - SAM addressing (not arduino)
#define TEMP_0_PIN 11

#define HEATER_1_PIN 8
// Due analog pin - SAM addressing (not arduino)
#define TEMP_1_PIN 12

#define HEATER_2_PIN -1
#define TEMP_2_PIN -1
#define HEATER_3_PIN -1
#define TEMP_3_PIN -1
#define HEATER_4_PIN -1
#define TEMP_4_PIN -1

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDSUPPORT true
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 53
//#define SDSS		   -1
//#define SDCARDDETECT   -1
#define SDCARDDETECTINVERTED 0
#define LED_PIN 13
#define ORIG_FAN_PIN 9
#define ORIG_PS_ON_PIN -1
#define KILL_PIN 41
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ 100000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 128    // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

// Smart RAMPS without(408) and with EEPROM (413)
#if MOTHERBOARD == 408 || MOTHERBOARD == 413
#ifndef __SAM3X8E__
#erro oops !Be sure to have 'due Arduino' selected from \
    the 'tools-> Boards menu'.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due pin assignments
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

// Note that in due A0 pins on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 10
// Because analog pin #
#define TEMP_0_PIN 11
#define HEATER_1_PIN 8
// Because analog pin #
#define TEMP_1_PIN 12
#define HEATER_2_PIN 9
// Because analog pin #
#define TEMP_2_PIN 13

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1

// 10 if using HW spi. 53 if using SW SPI
#define SDSS 53
#define LED_PIN 13
#define ORIG_FAN_PIN 9
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1
// Pin that has to be turned right after the start, to keep the power  flowing.
#define SUICIDE_PIN -1

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#if MOTHERBOARD == 408
#define TWI_CLOCK_FREQ 100000
//	20 or 70
#define SDA_PIN -1
// 21 or 71
#define SCL_PIN -1
#define EEPROM_AVAILABLE EEPROM_NONE
#else
// this board supports eeprom
#define SDA_PIN 20 // 20 or 70
#define SCL_PIN 21 // 21 or 71

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif
#endif

// Ultratronics Board
// http://www.reprapworld.com
#if MOTHERBOARD == 409
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 35
#define ORIG_X_DIR_PIN 34
#define ORIG_X_MIN_PIN 31
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 37

#define ORIG_Y_STEP_PIN 22
#define ORIG_Y_DIR_PIN 23
#define ORIG_Y_MIN_PIN 12
#define ORIG_Y_MAX_PIN 11
#define ORIG_Y_ENABLE_PIN 33

#define ORIG_Z_STEP_PIN 25
#define ORIG_Z_DIR_PIN 26
#define ORIG_Z_MIN_PIN 29
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 24

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 3
// Due analog pin A0 = channel 7
#define TEMP_0_PIN 7

#define HEATER_1_PIN 2
// Due analog pin A1 = channel 6
#define TEMP_1_PIN 6
// Due analog pin #58

#define HEATER_2_PIN 8
// Due analog pin A2 = channel 5
#define TEMP_2_PIN 5

#define HEATER_3_PIN 7
// Due analog pin A3 = channel 4
#define TEMP_3_PIN 4

#define HEATER_4_PIN 9
// Due analog pin A4 = channel 3
#define TEMP_4_PIN 3

// Dua analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 65
#define THERMOCOUPLE_1_PIN 52
#define THERMOCOUPLE_2_PIN 51
#define THERMOCOUPLE_3_PIN 50

#define ORIG_E0_STEP_PIN 47
#define ORIG_E0_DIR_PIN 46
#define ORIG_E0_ENABLE_PIN 48

#define ORIG_E1_STEP_PIN 44
#define ORIG_E1_DIR_PIN 36
#define ORIG_E1_ENABLE_PIN 45

#define ORIG_E2_STEP_PIN 42
#define ORIG_E2_DIR_PIN 41
#define ORIG_E2_ENABLE_PIN 43

#define ORIG_E3_STEP_PIN 39
#define ORIG_E3_DIR_PIN 38
#define ORIG_E3_ENABLE_PIN 40

#define SDSUPPORT -1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 59
#define SPI_PIN 78
#define NONSTANDARD_SDSS
// --- Use software SPI since display does otherwise not work, remove to use hardware SPI
#define DUE_SOFTWARE_SPI
#define ENABLE_SOFTWARE_SPI_CLASS 1
#define SD_SOFT_MISO_PIN 74
#define SD_SOFT_MOSI_PIN 75
#define SD_SOFT_SCK_PIN 76
// --- end switch to software SPI
#define MOSI_PIN 75
#define MISO_PIN 74
#define SCK_PIN 76

#define BEEPER_PIN 27

#define ORIG_SDCARDDETECT 60
#define SDCARDDETECTINVERTED 0
#define LED_PIN 13
#define ORIG_FAN_PIN 6
#define ORIG_FAN2_PIN 5
#define ORIG_PS_ON_PIN 32
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.
#define ENC424_SS 61

// 20 or 70
#define SDA_PIN 70
// 21 or 71
#define SCL_PIN 71

// Servo pins: 5,6 und 39

#define E0_PINS \
    ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN, TEMP_0_PIN,
#define E1_PINS \
    ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN, TEMP_2_PIN,
#define E2_PINS \
    ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN, TEMP_3_PIN,
#define E3_PINS \
    ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN, TEMP_4_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
// Ultronics has no eeprom for storing changeable data
// as a solution you can use sd card. But this requires always
// the same sd card when powering up the printer
//#define EEPROM_AVAILABLE EEPROM_NONE
#define EEPROM_AVAILABLE EEPROM_SDCARD

#define MB_SETUP \
    SET_OUTPUT(ORIG_FAN_PIN); \
    WRITE(ORIG_FAN_PIN, LOW); \
    SET_OUTPUT(ORIG_FAN2_PIN); \
    WRITE(ORIG_FAN2_PIN, LOW); \
    SET_OUTPUT(HEATER_0_PIN); \
    WRITE(HEATER_0_PIN, LOW); \
    SET_OUTPUT(HEATER_1_PIN); \
    WRITE(HEATER_1_PIN, LOW); \
    SET_OUTPUT(HEATER_2_PIN); \
    WRITE(HEATER_2_PIN, LOW); \
    SET_OUTPUT(HEATER_3_PIN); \
    WRITE(HEATER_3_PIN, LOW); \
    SET_OUTPUT(THERMOCOUPLE_0_PIN); \
    WRITE(THERMOCOUPLE_0_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_1_PIN); \
    WRITE(THERMOCOUPLE_1_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_2_PIN); \
    WRITE(THERMOCOUPLE_2_PIN, HIGH); \
    SET_OUTPUT(THERMOCOUPLE_3_PIN); \
    WRITE(THERMOCOUPLE_3_PIN, HIGH); \
    SET_OUTPUT(ENC424_SS); \
    WRITE(ENC424_SS, HIGH); \
    SET_OUTPUT(SDSS); \
    WRITE(SDSS, HIGH)

#endif

/*****************************************************************
 * DUE3DOM Board
 * http://www.due3dom.pl
 ******************************************************************/
#if MOTHERBOARD == 410
#ifndef __SAM3X8E__
#error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 2
#define ORIG_X_DIR_PIN 3
#define ORIG_X_MIN_PIN 38
#define ORIG_X_MAX_PIN 36
#define ORIG_X_ENABLE_PIN 22

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 34
#define ORIG_Y_MAX_PIN 32
#define ORIG_Y_ENABLE_PIN 26

#define ORIG_Z_STEP_PIN 61
#define ORIG_Z_DIR_PIN 60
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 15

#define ORIG_E0_STEP_PIN 64
#define ORIG_E0_DIR_PIN 63
#define ORIG_E0_ENABLE_PIN 62

#define ORIG_E1_STEP_PIN 51
#define ORIG_E1_DIR_PIN 53
#define ORIG_E1_ENABLE_PIN 65

#define ORIG_E2_STEP_PIN 24
#define ORIG_E2_DIR_PIN 23
#define ORIG_E2_ENABLE_PIN 49

// hotend1 heater
#define HEATER_0_PIN 7
// bed heater
#define HEATER_1_PIN 39
// hotend2 heater
#define HEATER_2_PIN 8

// hotend1 thermistor
#define TEMP_0_PIN 7
// bed thermistor
#define TEMP_1_PIN 6
// hotend2 thermistor
#define TEMP_2_PIN 5
// thermo fan thermistor
#define TEMP_3_PIN 2
#define THERMOCOUPLE_0_PIN 3
#define THERMOCOUPLE_1_PIN 4

// print fan
#define ORIG_FAN_PIN 11
// hotend1 cooler
#define ORIG_FAN2_PIN 9
// hotend2 cooler / thermo fan / board fan
#define FAN_THERMO_PIN 12

#define SDSUPPORT 1
#define SDPOWER -1
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1

#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN -1

#define SDA_PIN 20
#define SCL_PIN 21

// servo pins 5, 6, 13

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif
// End DUE3DOM Board

/*****************************************************************
 * DUE3DOM MINI Board
 * http://www.due3dom.pl
 ******************************************************************/
#if MOTHERBOARD == 411
#ifndef __SAM3X8E__
#error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define ORIG_X_STEP_PIN 17
#define ORIG_X_DIR_PIN 16
#define ORIG_X_MIN_PIN 38
// on expansion port
#define ORIG_X_MAX_PIN 36
#define ORIG_X_ENABLE_PIN 22

#define ORIG_Y_STEP_PIN 2
#define ORIG_Y_DIR_PIN 3
#define ORIG_Y_MIN_PIN 34
// on expansion port
#define ORIG_Y_MAX_PIN 32
#define ORIG_Y_ENABLE_PIN 26

#define ORIG_Z_STEP_PIN 64
#define ORIG_Z_DIR_PIN 63
#define ORIG_Z_MIN_PIN 30
// on expansion port
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 15

#define ORIG_E0_STEP_PIN 61
#define ORIG_E0_DIR_PIN 60
#define ORIG_E0_ENABLE_PIN 62

// on expansion port
#define ORIG_E1_STEP_PIN -1
// on expansion port
#define ORIG_E1_DIR_PIN -1
// on expansion port
#define ORIG_E1_ENABLE_PIN -1

// on expansion port
#define ORIG_E2_STEP_PIN -1
// on expansion port
#define ORIG_E2_DIR_PIN -1
// on expansion port
#define ORIG_E2_ENABLE_PIN -1

// hotend1 heater
#define HEATER_0_PIN 13
// bed heater
#define HEATER_1_PIN 7
// on expansion port
#define HEATER_2_PIN -1

// hotend1 thermistor
#define TEMP_0_PIN 7
// bed thermistor
#define TEMP_1_PIN 6
// thermo fan thermistor
#define TEMP_2_PIN 5
// onboard thermistor NTC100K Beta3950
#define TEMP_3_PIN 2
// on expansion port
#define THERMOCOUPLE_0_PIN 3
// on expansion port
#define THERMOCOUPLE_1_PIN 4

#define SDSUPPORT 1
#define SDPOWER -1
#define SDSS 4
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1

// hotend1 cooler
#define ORIG_FAN_PIN 9
// print fan
#define ORIG_FAN2_PIN 11
// thermo fan
#define FAN_THERMO_PIN 12
// 4-pin header FAN0 - only for 4-pin fans !!!
#define FAN_BOARD_PIN 8

#define ORIG_PS_ON_PIN 40
#define KILL_PIN -1
#define SUICIDE_PIN -1

#define SDA_PIN 20
#define SCL_PIN 21

// servo pins 5, 6, 13

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif
// End DUE3DOM MINI Board

// STACKER 3D Superboard
#if MOTHERBOARD == 412
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 24
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 28
#define ORIG_X_MAX_PIN 34
#define ORIG_X_ENABLE_PIN 26

#define ORIG_Y_STEP_PIN 17
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 30
#define ORIG_Y_MAX_PIN 36
#define ORIG_Y_ENABLE_PIN 22

#define ORIG_Z_STEP_PIN 2
#define ORIG_Z_DIR_PIN 3
#define ORIG_Z_MIN_PIN 32
#define ORIG_Z_MAX_PIN 38
#define ORIG_Z_ENABLE_PIN 15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
/*
Input Pins Channel Number
AD0 CH0
AD1 CH1
AD2 CH2
AD3 CH3
AD4 CH4
AD5 CH5
AD6 CH6
AD7 CH7
AD8 CH8
AD9 CH9
AD10 CH10
AD11 CH11
AD12 CH12
AD13 CH13
AD14 CH14
AD15 CH15
*/
#define HEATER_0_PIN 7
#define TEMP_0_PIN 7
// These pins are for bed !
#define HEATER_1_PIN 98
#define TEMP_1_PIN 6
#define HEATER_2_PIN 8
#define TEMP_2_PIN 5
#define HEATER_3_PIN 9
#define TEMP_3_PIN 4
// D7
#define HEATER_4_PIN 11
#define TEMP_4_PIN 3
// D8
#define HEATER_5_PIN 12
#define HEATER_6_PIN 13
#define HEATER_7_PIN 100
#define HEATER_8_PIN 72
#define TEMP_5_PIN 0
#define TEMP_6_PIN 1
#define TEMP_7_PIN 2

#define THERMOCOUPLE_0_PIN 10
#define THERMOCOUPLE_1_PIN 11
#define THERMOCOUPLE_2_PIN 12
#define THERMOCOUPLE_3_PIN 13
#define THERMOCOUPLE_4_PIN 14
#define THERMOCOUPLE_5_PIN 0
#define THERMOCOUPLE_6_PIN 1
#define THERMOCOUPLE_7_PIN 2

#define ORIG_E0_STEP_PIN 105
#define ORIG_E0_DIR_PIN 106
#define ORIG_E0_ENABLE_PIN 107

#define ORIG_E1_STEP_PIN 102
#define ORIG_E1_DIR_PIN 101
#define ORIG_E1_ENABLE_PIN 103

#define ORIG_E2_STEP_PIN 51
#define ORIG_E2_DIR_PIN 53
#define ORIG_E2_ENABLE_PIN 49

#define ORIG_E3_STEP_PIN 41
#define ORIG_E3_DIR_PIN 40
#define ORIG_E3_ENABLE_PIN 99

#define ORIG_E4_STEP_PIN 5
#define ORIG_E4_DIR_PIN 4
#define ORIG_E4_ENABLE_PIN 31

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 77
#define ORIG_SDCARDDETECT 14
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 97
// Same as heater 8, alias
#define ORIG_FAN2_PIN 72
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

// Servo pins: 5,6 und 39

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN, ORIG_E4_DIR_PIN, ORIG_E4_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

/****************************
 *     RURAMPS4D 1.0-1.1 Board
 ****************************/

#if MOTHERBOARD == 414
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 36
#define ORIG_X_MIN_PIN 45
#define ORIG_X_MAX_PIN 39
#define ORIG_X_ENABLE_PIN 38

#define ORIG_Y_STEP_PIN 32
#define ORIG_Y_DIR_PIN 35
#define ORIG_Y_MIN_PIN 46
#define ORIG_Y_MAX_PIN 41
#define ORIG_Y_ENABLE_PIN 34

#define ORIG_Z_STEP_PIN 30
#define ORIG_Z_DIR_PIN 2
#define ORIG_Z_MIN_PIN 47
#define ORIG_Z_MAX_PIN 43
#define ORIG_Z_ENABLE_PIN 33

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 13
// Due analog pin #54
#define TEMP_0_PIN 7
#define HEATER_1_PIN 7
#define TEMP_1_PIN 6
// Due analog pin #58
#define HEATER_2_PIN 12
// Due analog pin #55
#define TEMP_2_PIN 5
#define HEATER_3_PIN 11
// Due analog pin #56
#define TEMP_3_PIN 4
// Due analog pin #57
#define TEMP_4_PIN 3

// Due analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 2
#define THERMOCOUPLE_1_PIN 1

#define ORIG_E0_STEP_PIN 29
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 31

#define ORIG_E1_STEP_PIN 22
#define ORIG_E1_DIR_PIN 24
#define ORIG_E1_ENABLE_PIN 26

#define ORIG_E2_STEP_PIN 14
#define ORIG_E2_DIR_PIN 15
#define ORIG_E2_ENABLE_PIN 61

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
#define ORIG_SDCARDDETECT 51
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN 8
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

// servo pins 3, 5

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

/****************************
 *     RURAMPS4D 1.3 Board
 ****************************/

#if MOTHERBOARD == 415
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 36
#define ORIG_X_MIN_PIN 45
#define ORIG_X_MAX_PIN 39
#define ORIG_X_ENABLE_PIN 31

#define ORIG_Y_STEP_PIN 32
#define ORIG_Y_DIR_PIN 35
#define ORIG_Y_MIN_PIN 46
#define ORIG_Y_MAX_PIN 41
#define ORIG_Y_ENABLE_PIN 31

#define ORIG_Z_STEP_PIN 30
#define ORIG_Z_DIR_PIN 2
#define ORIG_Z_MIN_PIN 47
#define ORIG_Z_MAX_PIN 43
#define ORIG_Z_ENABLE_PIN 31

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 13
// Due analog pin #54
#define TEMP_0_PIN 7
#define HEATER_1_PIN 7
#define TEMP_1_PIN 6
// Due analog pin #58
#define HEATER_2_PIN 12
// Due analog pin #55
#define TEMP_2_PIN 5
#define HEATER_3_PIN 11
// Due analog pin #56
#define TEMP_3_PIN 4
// Due analog pin #57
#define TEMP_4_PIN 3

// Due analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN 2
#define THERMOCOUPLE_1_PIN 1

#define ORIG_E0_STEP_PIN 29
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 33

#define ORIG_E1_STEP_PIN 22
#define ORIG_E1_DIR_PIN 24
#define ORIG_E1_ENABLE_PIN 26

#define ORIG_E2_STEP_PIN 25
#define ORIG_E2_DIR_PIN 23
#define ORIG_E2_ENABLE_PIN 27

#define SDSUPPORT 1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 4
#define ORIG_SDCARDDETECT 51
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1
#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN 8
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 20
// 21 or 71
#define SCL_PIN 21

// servo pins 3, 5

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

// *****************************************************************
// Shasta - dua based printer board used in 3ntr printers
// *****************************************************************

#if MOTHERBOARD == 416
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
// 1
#define ORIG_X_STEP_PIN 18
#define ORIG_X_DIR_PIN 19
#define ORIG_X_MIN_PIN 31
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 22
// 2
#define ORIG_Y_STEP_PIN 23
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 12
#define ORIG_Y_MAX_PIN 11
#define ORIG_Y_ENABLE_PIN 17
// 3
#define ORIG_Z_STEP_PIN 25
#define ORIG_Z_DIR_PIN 70
#define ORIG_Z_MIN_PIN 29
#define ORIG_Z_MAX_PIN 28
#define ORIG_Z_ENABLE_PIN 24

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 9
// Due analog pin A0 = channel 7
#define TEMP_0_PIN -1

#define HEATER_1_PIN 5
// Due analog pin A1 = channel 6
#define TEMP_1_PIN -1
// Due analog pin #58

#define HEATER_2_PIN 8
// Due analog pin A2 = channel 5
#define TEMP_2_PIN -1

#define HEATER_3_PIN 7
// Due analog pin A3 = channel 4
#define TEMP_3_PIN -1

#define HEATER_4_PIN 6
// Due analog pin A4 = channel 3
#define TEMP_4_PIN -1

// PB6
#define THERMOCOUPLE_0_PIN 102
// PB4
#define THERMOCOUPLE_1_PIN 101
// PB3
#define THERMOCOUPLE_2_PIN 111
// PB2
#define THERMOCOUPLE_3_PIN 110
// PB6
#define THERMOCOUPLE_4_PIN 103
// PB7
#define THERMOCOUPLE_5_PIN 104
// PB8
#define THERMOCOUPLE_6_PIN 105
// PB9
#define THERMOCOUPLE_7_PIN 106

// PB0
#define THERMOCOUPLE_CS 108
// PB1
#define THERMOCOUPLE_SCK 109

// 4
#define ORIG_E0_STEP_PIN 28
#define ORIG_E0_DIR_PIN 27
#define ORIG_E0_ENABLE_PIN 26
// 5
#define ORIG_E1_STEP_PIN 29
#define ORIG_E1_DIR_PIN 15
#define ORIG_E1_ENABLE_PIN 14
// 6
#define ORIG_E2_STEP_PIN 68
#define ORIG_E2_DIR_PIN 69
#define ORIG_E2_ENABLE_PIN 30
// 7
#define ORIG_E3_STEP_PIN 33
#define ORIG_E3_DIR_PIN 32
#define ORIG_E3_ENABLE_PIN 31

#define SDSUPPORT -1
#define SDPOWER -1
// 4,10,52 if using HW SPI.
#define SDSS 66
//#define NONSTANDARD_SDSS
#define MOSI_PIN 75
#define MISO_PIN 74
#define SCK_PIN 76

#define ORIG_SDCARDDETECT 60
#define SDCARDDETECTINVERTED 0
#define LED_PIN 59
#define ORIG_FAN_PIN 2
#define ORIG_FAN2_PIN 3
#define ORIG_PS_ON_PIN 32
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.
#define ENC424_SS 61

// 20 or 70
#define SDA_PIN 70
// 21 or 71
#define SCL_PIN 71

// Servo pins: 5,6 und 39

#define E0_PINS \
    ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN, TEMP_0_PIN,
#define E1_PINS \
    ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN, TEMP_2_PIN,
#define E2_PINS \
    ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN, TEMP_3_PIN,
#define E3_PINS \
    ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN, TEMP_4_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    7 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
// Ultronics has no eeprom for storing changeable data
// as a solution you can use sd card. But this requires always
// the same sd card when powering up the printer
//#define EEPROM_AVAILABLE EEPROM_NONE
#define EEPROM_AVAILABLE EEPROM_SDCARD

#define MB_SETUP \
    SET_OUTPUT(ORIG_FAN_PIN); \
    WRITE(ORIG_FAN_PIN, LOW); \
    SET_OUTPUT(ORIG_FAN2_PIN); \
    WRITE(ORIG_FAN2_PIN, LOW); \
    SET_OUTPUT(HEATER_0_PIN); \
    WRITE(HEATER_0_PIN, LOW); \
    SET_OUTPUT(HEATER_1_PIN); \
    WRITE(HEATER_1_PIN, LOW); \
    SET_OUTPUT(HEATER_2_PIN); \
    WRITE(HEATER_2_PIN, LOW); \
    SET_OUTPUT(HEATER_3_PIN); \
    WRITE(HEATER_3_PIN, LOW); \
    SET_OUTPUT(ENC424_SS); \
    WRITE(ENC424_SS, HIGH); \
    SET_OUTPUT(SDSS); \
    WRITE(SDSS, HIGH)

#endif

/*****************************************************************
 * Alligator Board rev3
 * http://www.3dartists.org/
 ******************************************************************/
//
#if MOTHERBOARD == 502
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Alligator 3D Printer Board R3' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
#define SPI_CHAN_DAC 1
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_ALLIGATOR
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

#define ORIG_X_STEP_PIN 96
#define ORIG_X_DIR_PIN 2
#define ORIG_X_MIN_PIN 34
#define ORIG_X_MAX_PIN 33
#define ORIG_X_ENABLE_PIN 24
#define X_MS1_PIN 99
#define X_MS2_PIN -1

#define ORIG_Y_STEP_PIN 94
#define ORIG_Y_DIR_PIN 95
#define ORIG_Y_MIN_PIN 37
#define ORIG_Y_MAX_PIN 35
#define ORIG_Y_ENABLE_PIN 3
#define Y_MS1_PIN 10
#define Y_MS2_PIN -1

#define ORIG_Z_STEP_PIN 65
#define ORIG_Z_DIR_PIN 52
#define ORIG_Z_MIN_PIN 39
#define ORIG_Z_MAX_PIN 38
#define ORIG_Z_ENABLE_PIN 4
#define Z_MS1_PIN 44
#define Z_MS2_PIN -1

// Motor Fault onboard stepper driver (X/Y/Z/E0)
#define MOTOR_FAULT_PIN 22
// Motor Fault on piggy module (E1/E2/E3)
#define MOTOR_FAULT_PIGGY_PIN 7

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 69
#define TEMP_0_PIN 6
#define HEATER_1_PIN 42
#define TEMP_1_PIN 5

#define HEATER_2_PIN 8
#define TEMP_2_PIN 4
#define HEATER_3_PIN 9
#define TEMP_3_PIN 3
#define HEATER_4_PIN 97
#define TEMP_4_PIN 2

#define ORIG_E0_STEP_PIN 63
#define ORIG_E0_DIR_PIN 64
#define ORIG_E0_ENABLE_PIN 98
#define E0_MS1_PIN 45
#define E0_MS2_PIN -1

#define ORIG_E1_STEP_PIN 62
#define ORIG_E1_DIR_PIN 53
#define ORIG_E1_ENABLE_PIN 29
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1

#define ORIG_E2_STEP_PIN 20
#define ORIG_E2_DIR_PIN 21
#define ORIG_E2_ENABLE_PIN 12
#define E2_MS_PIN -1

#define ORIG_E3_STEP_PIN 66
#define ORIG_E3_DIR_PIN 67
#define ORIG_E3_ENABLE_PIN 30
#define E3_MS_PIN -1

#define SDSUPPORT true
#define SDPOWER -1
#define SDSS 77
#define ORIG_SDCARDDETECT 87
#define SDCARDDETECTINVERTED 0

#define LED_PIN -1
#define ORIG_FAN_PIN 92
#define ORIG_FAN2_PIN 31

#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN -1

#define CASE_LIGHTS_PIN -1

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,

//** DAC for onboard motor vfref current
#define DAC0_SYNC 27
//** DAC for piggy module motor vfref current
#define DAC1_SYNC 6

//** Onboard addictional memory
// 64K EEPROM
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS 25
// 2K EEPROM
#define SPI_EEPROM2_CS 26
// 32Mb FLASH
#define SPI_FLASH_CS 23

// EXP Use internal USART-3
#define ESP_WIFI_MODULE_COM 3
#define ESP_WIFI_MODULE_RESET_PIN 43
#define PIGGY_GPIO_PIN 11

#define FTDI_COM_RESET_PIN 32

/*** Expansions ports ***/

/** RaspberryPi Expasnion port J15 **/
#define EXP_RPI_J15_12 60
#define EXP_RPI_J15_13 61
#define EXP_RPI_J15_16 100
#define EXP_RPI_J15_18 51
#define EXP_RPI_J15_22 50

/** Expansion#1 port J14  3V3/5V voltage level firmware selectable**/
#define EXP1_OUT_ENABLE_PIN 49
#define EXP1_VOLTAGE_SELECT 5
#define EXP1_J14_4 48
#define EXP1_J14_6 47
#define EXP1_J14_8 46
// USART-0 RX
#define EXP1_J14_5 19
// USART-0 TX
#define EXP1_J14_3 18
#define EXP1_J14_9 104
#define EXP1_J14_10 105

//** Expansion#2 port J5
// USART-1 RX
#define EXP2_J5_5 17
// USART-1 TX
#define EXP2_J5_1 16
// A0
#define EXP2_J5_3 54
#define EXP1_J5_4 36
#define EXP1_J5_8 40
#define EXP1_J5_6 41
/* i2c on EXP2 , SDA J5-7 , SCL J5-9*/
#define SDA_PIN 70
#define SCL_PIN 71

//** Expansion#3 port J19
#define EXP3_J19_5 73
#define EXP3_J19_9 108
#define EXP3_J19_7 109
#define EXP3_J19_3 110
#define EXP3_J19_14 111
#define EXP3_J19_12 101
#define EXP3_J19_10 102
#define EXP3_J19_8 103
#define EXP3_J19_6 106
#define EXP3_J19_4 13
#define EXP3_J19_2 72
#define EXP3_J19_1 28

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    10 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR // EEPROM_SPI_ALLIGATOR
#endif
// End Alligator Board

/*****************************************************************
 * Alligator Board rev2
 * http://www.3dartists.org/
 ******************************************************************/
//
#if MOTHERBOARD == 501
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Alligator 3D Printer Board R2' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
#define SPI_CHAN_DAC 1
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_ALLIGATOR
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

// PB24
#define ORIG_X_STEP_PIN 96
// PB25
#define ORIG_X_DIR_PIN 2
// PC2
#define ORIG_X_MIN_PIN 34
// PC1
#define ORIG_X_MAX_PIN 33
// PA15, motor RESET pin
#define ORIG_X_ENABLE_PIN 24
// PC10
#define X_MS1_PIN 99
#define X_MS2_PIN -1

// PB22
#define ORIG_Y_STEP_PIN 94
// PB23
#define ORIG_Y_DIR_PIN 95
// PC5
#define ORIG_Y_MIN_PIN 37
// PC3
#define ORIG_Y_MAX_PIN 35
// PA15, motor RESET pin
#define ORIG_Y_ENABLE_PIN 24
// PC29
#define Y_MS1_PIN 10
#define Y_MS2_PIN -1

// PC27
#define ORIG_Z_STEP_PIN 98
// PC28
#define ORIG_Z_DIR_PIN 3
// PC7
#define ORIG_Z_MIN_PIN 39
// PC6
#define ORIG_Z_MAX_PIN 38
// PA15, motor RESET pin
#define ORIG_Z_ENABLE_PIN 24
// PC19
#define Z_MS1_PIN 44
#define Z_MS2_PIN -1

// PB26 , motor X-Y-Z-E0 motor FAULT
#define MOTOR_FAULT_PIN 22

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
// PA1
#define HEATER_0_PIN 68
// PA24, analog pin
#define TEMP_0_PIN 6
// PA0
#define HEATER_1_PIN 69
// PA16
#define TEMP_1_PIN 7

// PC22 on piggy
#define HEATER_2_PIN 8
// PA6, analog on piggy
#define TEMP_2_PIN 3
// PC21 on piggy
#define HEATER_3_PIN 9
// PA22, analog pin on piggy
#define TEMP_3_PIN 4
// PC20 on piggy
#define HEATER_4_PIN 97
// PA23 analog pin on piggy
#define TEMP_4_PIN 5

// PA15, motor RESET pin
#define ORIG_MOTOR_RESET 24

// PC25
#define ORIG_E0_STEP_PIN 5
// PC26
#define ORIG_E0_DIR_PIN 4
// PA15, motor RESET pin
#define ORIG_E0_ENABLE_PIN 24
// PC18
#define E0_MS1_PIN 45
#define E0_MS2_PIN -1

// PD3 on piggy
#define ORIG_E1_STEP_PIN 28
// PD2 on piggy
#define ORIG_E1_DIR_PIN 27
// PA15, motor RESET pin
#define ORIG_E1_ENABLE_PIN 24
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1

// PD7 on piggy
#define ORIG_E2_STEP_PIN 11
// PD6 on piggy
#define ORIG_E2_DIR_PIN 29
// PA15, motor RESET pin
#define ORIG_E2_ENABLE_PIN 24
#define E2_MS_PIN -1

// PD9 on piggy
#define ORIG_E3_STEP_PIN 30
// PD8 on piggy
#define ORIG_E3_DIR_PIN 12
// PA15, motor RESET pin
#define ORIG_E3_ENABLE_PIN 24
#define E3_MS_PIN -1

#define SDSUPPORT true
#define SDPOWER -1
// PA28
#define SDSS 77
// PA29
#define ORIG_SDCARDDETECT 87
#define SDCARDDETECTINVERTED 0
#define LED_PIN -1

// PA5
#define ORIG_FAN_PIN 92
// PA7
#define ORIG_FAN2_PIN 31
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.

#define SDA_PIN -1 // i2c not used
#define SCL_PIN -1 // i2c not used

// PC4
#define CASE_LIGHTS_PIN 36

// PB20
#define EXP_VOLTAGE_LEVEL_PIN 65

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,

//** DAC for motor vfref current
// PB14
#define DAC0_SYNC 53
// PC24
#define DAC1_SYNC 6

//** EEPROM **

// 64K SPI
#define SPI_CHAN_EEPROM1 2
// PD0
#define SPI_EEPROM1_CS 25

// 2K SPI
// PD1
#define SPI_EEPROM2_CS 26

//** FLASH SPI**/
// 32Mb
// PA14
#define SPI_FLASH_CS 23

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    10 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR // EEPROM_SPI_ALLIGATOR
#endif
// End Alligator Board

/*****************************************************************
 * Alligator Board rev1
 * http://www.3dartists.org/
 ******************************************************************/
//
#if MOTHERBOARD == 500
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Alligator 3D Printer Board R1' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
#define SPI_CHAN_DAC 1
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_DAC
/*****************************************************************
 * Arduino Due Pin Assignments
 ******************************************************************/

// PB24
#define ORIG_X_STEP_PIN 96
// PB25
#define ORIG_X_DIR_PIN 2
// PC2
#define ORIG_X_MIN_PIN 34
// PC1
#define ORIG_X_MAX_PIN 33
// PA15, motor RESET pin
#define ORIG_X_ENABLE_PIN 24

// PB22
#define ORIG_Y_STEP_PIN 94
// PB23
#define ORIG_Y_DIR_PIN 95
// PC5
#define ORIG_Y_MIN_PIN 37
// PC3
#define ORIG_Y_MAX_PIN 35
// PA15, motor RESET pin
#define ORIG_Y_ENABLE_PIN 24

// PC27
#define ORIG_Z_STEP_PIN 98
// PC28
#define ORIG_Z_DIR_PIN 3
// PC7
#define ORIG_Z_MIN_PIN 39
// PC6
#define ORIG_Z_MAX_PIN 38
// PA15, motor RESET pin
#define ORIG_Z_ENABLE_PIN 24

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
// PA1
#define HEATER_0_PIN 68
// PA4, analog pin
#define TEMP_0_PIN 2
// PA0
#define HEATER_1_PIN 69
// PA6, analog pn
#define TEMP_1_PIN 3
#define HEATER_2_PIN -1 // PC22 on piggy
#define TEMP_2_PIN -1   // PA3 analog pin on piggy
#define HEATER_3_PIN -1 // PC21 on piggy
#define TEMP_3_PIN -1   // PA2, analog pin on piggy
#define HEATER_4_PIN -1 // PC20 on piggy
#define TEMP_4_PIN -1   // PB12, analog pin on piggy

#define ORIG_ENABLE_PIN 24

// PC25
#define ORIG_E0_STEP_PIN 5
// PC26
#define ORIG_E0_DIR_PIN 4
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN -1 // PD3 on piggy
#define ORIG_E1_DIR_PIN -1  // PD2 on piggy
#define ORIG_E1_ENABLE_PIN -1

#define ORIG_E2_STEP_PIN -1 // PD7 on piggy
#define ORIG_E2_DIR_PIN -1  // PD6 on piggy
#define ORIG_E2_ENABLE_PIN -1

#define ORIG_E3_STEP_PIN -1 // PD9 on piggy
#define ORIG_E3_DIR_PIN -1  // PD8 on piggy
#define ORIG_E3_ENABLE_PIN -1

#define SDSUPPORT true
#define SDPOWER -1
// PA28
#define SDSS 77
// PA29
#define ORIG_SDCARDDETECT 87
#define SDCARDDETECTINVERTED false
#define LED_PIN -1

// 92(orig) // PA5
#define ORIG_FAN_PIN 92
// 31(orig) // PA7
#define ORIG_FAN2_PIN 31
#define ORIG_PS_ON_PIN -1
#define KILL_PIN ORIG_X_MIN_PIN
#define SUICIDE_PIN \
    -1 // PIN that has to be turned on right after start, to keep power flowing.
#define HEAT_OFF_INT_PIN 50

#define SDA_PIN -1 // i2c not used
#define SCL_PIN -1 // i2c not used

// PC9
#define CASE_LIGHTS_PIN 41

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,

//** DAC for motor vfref current
#define DAC0_SYNC 53 // PB14
#define DAC1_SYNC 53 // PB14

//** EEPROM **

// 64K SPI
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS 25 // PD0

// 2K SPI
#define SPI_EEPROM2_CS 26 // PD1

//** FLASH SPI**/
// 32Mb
#define SPI_FLASH_CS 23 // PA14

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50 // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME \
    10 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR
#endif

#if MOTHERBOARD == 999
#define KNOWN_BOARD
#include "userpins.h"
#endif

#ifndef SDSSORIG
#define SDSSORIG -1
#endif

// Set default stepper current control if not set yet.
#ifndef STEPPER_CURRENT_CONTROL
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_MANUAL
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#if NUM_EXTRUDER < 2
#undef E1_PINS
#define E1_PINS
#endif

#if NUM_EXTRUDER < 3
#undef E2_PINS
#define E2_PINS
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED 0
#endif

// Available chip select pins for HW SPI are 4 10 52
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
#else // SDSS == 77
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

#define SENSITIVE_PINS \
    { \
        0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
            Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, \
            Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, \
            ORIG_PS_ON_PIN, HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, \
            E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN, SDSS \
    }
#endif
