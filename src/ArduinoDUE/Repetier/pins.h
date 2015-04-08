#ifndef PINS_H
#define PINS_H


/*
The board assignment defines the capabilities of the motherboard and the used pins.
Each board definition follows the following scheme:

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
#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master
#define CURRENT_CONTROL_ALLIGATOR 4  //Use External DAC like Alligator

/*
  arm does not have a eeprom build in. Therefore boards can add a
  eeprom. Board definition must set the right type of eeprom
*/

#define EEPROM_NONE 0
#define EEPROM_I2C  1
#define EEPROM_SPI_ALLIGATOR 2

#if MOTHERBOARD == 401
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN     54  // A0
#define ORIG_X_DIR_PIN      55  // A1
#define ORIG_X_MIN_PIN      3
#define ORIG_X_MAX_PIN      2
#define ORIG_X_ENABLE_PIN   38

#define ORIG_Y_STEP_PIN     60  // A6 
#define ORIG_Y_DIR_PIN      61  // A7
#define ORIG_Y_MIN_PIN      14
#define ORIG_Y_MAX_PIN      15
#define ORIG_Y_ENABLE_PIN   56  // A2

#define ORIG_Z_STEP_PIN     46
#define ORIG_Z_DIR_PIN      48
#define ORIG_Z_MIN_PIN      18
#define ORIG_Z_MAX_PIN      19
#define ORIG_Z_ENABLE_PIN   62  // A8

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN   10
#define TEMP_0_PIN     11  // Due analog pin #
#define HEATER_1_PIN   8
#define TEMP_1_PIN     12  // Due analog pin #
#define HEATER_2_PIN   9
#define TEMP_2_PIN     13  // Due analog pin #

#define ORIG_E0_STEP_PIN    26
#define ORIG_E0_DIR_PIN     28
#define ORIG_E0_ENABLE_PIN  24

#define ORIG_E1_STEP_PIN    36
#define ORIG_E1_DIR_PIN     34
#define ORIG_E1_ENABLE_PIN  40

#define SDPOWER 	   -1
#define SDSS		   53 // 10 if using HW SPI. 53 if using SW SPI
#define LED_PIN 	   13
#define ORIG_FAN_PIN 	   -1
#define ORIG_PS_ON_PIN      12
#define KILL_PIN	   -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.


#define SDA_PIN 				20  	// 20 or 70
#define SCL_PIN 				21  	// 21 or 71


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
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

#define ORIG_X_STEP_PIN     24
#define ORIG_X_DIR_PIN      23
#define ORIG_X_MIN_PIN      28
#define ORIG_X_MAX_PIN      34
#define ORIG_X_ENABLE_PIN   26

#define ORIG_Y_STEP_PIN     17 
#define ORIG_Y_DIR_PIN      16
#define ORIG_Y_MIN_PIN      30
#define ORIG_Y_MAX_PIN      36
#define ORIG_Y_ENABLE_PIN   22

#define ORIG_Z_STEP_PIN     2
#define ORIG_Z_DIR_PIN      3
#define ORIG_Z_MIN_PIN      32
#define ORIG_Z_MAX_PIN      38
#define ORIG_Z_ENABLE_PIN   15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     13
#define TEMP_0_PIN       7 // Due analog pin #54
#define HEATER_1_PIN     7 
#define TEMP_1_PIN       3 // Due analog pin #58
#define HEATER_2_PIN     12
#define TEMP_2_PIN       6 // Due analog pin #55
#define HEATER_3_PIN     11
#define TEMP_3_PIN       5 // Due analog pin #56
#define TEMP_4_PIN       4 // Due analog pin #57

#define ORIG_E0_STEP_PIN    61 // A7
#define ORIG_E0_DIR_PIN     60 // A6
#define ORIG_E0_ENABLE_PIN  62 // A8

#define ORIG_E1_STEP_PIN    64 // A10
#define ORIG_E1_DIR_PIN     63 // A9
#define ORIG_E1_ENABLE_PIN  65 // A11

#define ORIG_E2_STEP_PIN    51
#define ORIG_E2_DIR_PIN     53
#define ORIG_E2_ENABLE_PIN  49

#define ORIG_E3_STEP_PIN    35
#define ORIG_E3_DIR_PIN     33
#define ORIG_E3_ENABLE_PIN  37

#define ORIG_E4_STEP_PIN    29
#define ORIG_E4_DIR_PIN     27
#define ORIG_E4_ENABLE_PIN  31

#define SDSUPPORT      1
#define SDPOWER 	   -1
#define SDSS		    4// 4,10,52 if using HW SPI.
#define ORIG_SDCARDDETECT       14
#define SDCARDDETECTINVERTED 0
#define LED_PIN 	   -1
#define ORIG_FAN_PIN 	   9 
#define ORIG_FAN2_PIN           8 
#define ORIG_PS_ON_PIN          40
#define KILL_PIN	   -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

#define SDA_PIN 				20  	// 20 or 70
#define SCL_PIN 				21  	// 21 or 71


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN,ORIG_E4_DIR_PIN,ORIG_E4_ENABLE_PIN,

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
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
#define HEATER_PINS_INVERTED 1  // only old boards had the output inverted
#else
#define HEATER_PINS_INVERTED 0
#endif

/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN     63
#define ORIG_X_DIR_PIN      62
#define ORIG_X_MIN_PIN      22
#define ORIG_X_MAX_PIN      30
#define ORIG_X_ENABLE_PIN   48

#define ORIG_Y_STEP_PIN     65 
#define ORIG_Y_DIR_PIN      64
#define ORIG_Y_MIN_PIN      24
#define ORIG_Y_MAX_PIN      38
#define ORIG_Y_ENABLE_PIN   46

#define ORIG_Z_STEP_PIN     67
#define ORIG_Z_DIR_PIN      66
#define ORIG_Z_MIN_PIN      26
#define ORIG_Z_MAX_PIN      34
#define ORIG_Z_ENABLE_PIN   44

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     8
#define TEMP_0_PIN       7 // Due analog pin #54

#define HEATER_1_PIN     9 
#define TEMP_1_PIN       6 // Due analog pin #55

#define HEATER_2_PIN     10
#define TEMP_2_PIN       5 // Due analog pin #56

#define HEATER_3_PIN     11
#define TEMP_3_PIN       4 // Due analog pin #57

#define TEMP_4_PIN       3 // Due analog pin #58

#define ORIG_E0_STEP_PIN    36
#define ORIG_E0_DIR_PIN     28
#define ORIG_E0_ENABLE_PIN  42

#define ORIG_E1_STEP_PIN    43
#define ORIG_E1_DIR_PIN     41
#define ORIG_E1_ENABLE_PIN  39

#define ORIG_E2_STEP_PIN    32
#define ORIG_E2_DIR_PIN     47
#define ORIG_E2_ENABLE_PIN  45

//#define SDSUPPORT      false
#define SDPOWER 	   -1
#define SDSS		   4 // 4,10,52 if using HW SPI.
//#define SDSS		   -1
//#define ORIG_SDCARDDETECT   -1
#define SDCARDDETECTINVERTED false
#define LED_PIN 	   -1
#define ORIG_FAN_PIN 	   12 
#define ORIG_FAN2_PIN       2
#define ORIG_PS_ON_PIN      53
#define KILL_PIN	   -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

#define SDA_PIN 				20  	// 20 or 70
#define SCL_PIN 				21  	// 21 or 71


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif

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

#define ORIG_X_STEP_PIN     96 // PB24
#define ORIG_X_DIR_PIN       2 // PB25
#define ORIG_X_MIN_PIN      34 // PC2
#define ORIG_X_MAX_PIN      33 // PC1
#define ORIG_X_ENABLE_PIN   24 // PA15, motor RESET pin
#define X_MS1_PIN           99 // PC10
#define X_MS2_PIN           -1

#define ORIG_Y_STEP_PIN     94 // PB22
#define ORIG_Y_DIR_PIN      95 // PB23
#define ORIG_Y_MIN_PIN      37 // PC5
#define ORIG_Y_MAX_PIN      35 // PC3
#define ORIG_Y_ENABLE_PIN   24 // PA15, motor RESET pin
#define Y_MS1_PIN           10 // PC29
#define Y_MS2_PIN           -1

#define ORIG_Z_STEP_PIN     98 // PC27
#define ORIG_Z_DIR_PIN       3 // PC28
#define ORIG_Z_MIN_PIN      39 // PC7
#define ORIG_Z_MAX_PIN      38 // PC6
#define ORIG_Z_ENABLE_PIN   24 // PA15, motor RESET pin
#define Z_MS1_PIN           44 // PC19
#define Z_MS2_PIN           -1

#define MOTOR_FAULT_PIN 22 // PB26 , motor X-Y-Z-E0 motor FAULT

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     68 // PA1
#define TEMP_0_PIN       6  // PA24, analog pin
#define HEATER_1_PIN     69 // PA0
#define TEMP_1_PIN       7  // PA16

#define HEATER_2_PIN     8  // PC22 on piggy
#define TEMP_2_PIN       5  // PA23 analog pin on piggy
#define HEATER_3_PIN     9  // PC21 on piggy
#define TEMP_3_PIN       4  // PA22, analog pin on piggy
#define HEATER_4_PIN     97 // PC20 on piggy
#define TEMP_4_PIN       3  // PA6, analog on piggy

#define ORIG_MOTOR_RESET  24  // PA15, motor RESET pin

#define ORIG_E0_STEP_PIN    5  // PC25
#define ORIG_E0_DIR_PIN     4  // PC26
#define ORIG_E0_ENABLE_PIN  24 // PA15, motor RESET pin
#define E0_MS1_PIN          45 // PC18
#define E0_MS2_PIN           -1

#define ORIG_E1_STEP_PIN    28 // PD3 on piggy
#define ORIG_E1_DIR_PIN     27 // PD2 on piggy
#define ORIG_E1_ENABLE_PIN  -1
#define E1_MS1_PIN          -1
#define E1_MS2_PIN          -1

#define ORIG_E2_STEP_PIN    11 // PD7 on piggy
#define ORIG_E2_DIR_PIN     29 // PD6 on piggy
#define ORIG_E2_ENABLE_PIN  -1
#define E2_MS_PIN         -1

#define ORIG_E3_STEP_PIN    30 // PD9 on piggy
#define ORIG_E3_DIR_PIN     12 // PD8 on piggy
#define ORIG_E3_ENABLE_PIN  -1
#define E3_MS_PIN         -1

#define SDSUPPORT      true
#define SDPOWER 	   -1
#define SDSS		    77 // PA28
#define ORIG_SDCARDDETECT    87 // PA29
#define SDCARDDETECTINVERTED false
#define LED_PIN 	   -1

#define ORIG_FAN_PIN 	   92 // PA5
#define ORIG_FAN2_PIN      31 // PA7
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1
#define SUICIDE_PIN        -1 //PIN that has to be turned on right after start, to keep power flowing.

#define SDA_PIN 	-1  // i2c not used
#define SCL_PIN 	-1  // i2c not used

#define CASE_LIGHTS_PIN 36 // PC4

#define EXP_VOLTAGE_LEVEL_PIN 65 // PB20

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,

//** DAC for motor vfref current
#define DAC_SYNC   53 // PB14

//** EEPROM **

//64K SPI
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS 25 // PD0

//2K SPI
#define SPI_EEPROM2_CS 26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS 23 //PA14

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        32     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  10      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR //EEPROM_SPI_ALLIGATOR
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

#define ORIG_X_STEP_PIN     96 // PB24
#define ORIG_X_DIR_PIN       2 // PB25
#define ORIG_X_MIN_PIN      34 // PC2 
#define ORIG_X_MAX_PIN      33 // PC1
#define ORIG_X_ENABLE_PIN   24 // PA15, motor RESET pin

#define ORIG_Y_STEP_PIN      94 // PB22
#define ORIG_Y_DIR_PIN       95 // PB23
#define ORIG_Y_MIN_PIN      37 // PC5
#define ORIG_Y_MAX_PIN      35 // PC3
#define ORIG_Y_ENABLE_PIN   24 // PA15, motor RESET pin

#define ORIG_Z_STEP_PIN     98 // PC27
#define ORIG_Z_DIR_PIN       3 // PC28
#define ORIG_Z_MIN_PIN      39 // PC7
#define ORIG_Z_MAX_PIN      38 // PC6
#define ORIG_Z_ENABLE_PIN   24 // PA15, motor RESET pin

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     68 // PA1 
#define TEMP_0_PIN       2// PA4, analog pin 
#define HEATER_1_PIN     69 // PA0 
#define TEMP_1_PIN       3// // PA6, analog pn
#define HEATER_2_PIN     -1 // PC22 on piggy
#define TEMP_2_PIN       -1 // PA3 analog pin on piggy
#define HEATER_3_PIN     -1 // PC21 on piggy
#define TEMP_3_PIN       -1 // PA2, analog pin on piggy
#define HEATER_4_PIN    -1 // PC20 on piggy
#define TEMP_4_PIN      -1 //PB12, analog pin on piggy

#define ORIG_ENABLE_PIN 24

#define ORIG_E0_STEP_PIN    5 // PC25
#define ORIG_E0_DIR_PIN     4 // PC26
#define ORIG_E0_ENABLE_PIN  24

#define ORIG_E1_STEP_PIN    -1 // PD3 on piggy
#define ORIG_E1_DIR_PIN     -1 // PD2 on piggy
#define ORIG_E1_ENABLE_PIN  -1

#define ORIG_E2_STEP_PIN    -1 // PD7 on piggy
#define ORIG_E2_DIR_PIN     -1 // PD6 on piggy
#define ORIG_E2_ENABLE_PIN  -1

#define ORIG_E3_STEP_PIN    -1 // PD9 on piggy
#define ORIG_E3_DIR_PIN     -1 // PD8 on piggy
#define ORIG_E3_ENABLE_PIN  -1

#define SDSUPPORT      true
#define SDPOWER 	   -1
#define SDSS		    77 // PA28
#define ORIG_SDCARDDETECT        87 // PA29
#define SDCARDDETECTINVERTED false
#define LED_PIN 	   -1

#define ORIG_FAN_PIN 	   92 //92(orig) // PA5
#define ORIG_FAN2_PIN      31//31(orig) // PA7
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN	   ORIG_X_MIN_PIN
#define SUICIDE_PIN    -1 //PIN that has to be turned on right after start, to keep power flowing.
#define HEAT_OFF_INT_PIN 50  

#define SDA_PIN 	-1  // i2c not used
#define SCL_PIN 	-1  // i2c not used

#define CASE_LIGHTS_PIN 41//PC9

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,

//** DAC for motor vfref current
#define DAC_SYNC   53 // PB14

//** EEPROM **

//64K SPI
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS 25 // PD0

//2K SPI
#define SPI_EEPROM2_CS 26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS 23 //PA14

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        32     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  10      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR
#endif


#if MOTHERBOARD == 999
#define KNOWN_BOARD
#include "userpins.h"
#endif



#ifndef SDSSORIG
#define SDSSORIG -1
#endif

#ifndef STEPPER_CURRENT_CONTROL // Set default stepper current control if not set yet.
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_MANUAL
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#if NUM_EXTRUDER==1
#define E1_PINS
#endif

#if NUM_EXTRUDER<3
#define E2_PINS
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED 0
#endif

//Available chip select pins for HW SPI are 4 10 52 
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 77) 
#if (SDSS == 10)
#define SPI_PIN         77
#define SPI_CHAN        0
#else
#if (SDSS == 52) 
#define SPI_PIN         86
#define SPI_CHAN        2
#else // SDSS == 4
#if (SDSS == 4)
#define SPI_PIN         87
#define SPI_CHAN        1
#else //SDSS == 77
#define SPI_PIN         77
#define SPI_CHAN        0
  #endif
#endif
#endif
#define MOSI_PIN        75
#define MISO_PIN        74
#define SCK_PIN         76
//#define DUE_SOFTWARE_SPI
#else
#define DUE_SOFTWARE_SPI
#define MOSI_PIN		51
#define MISO_PIN		50
#define SCK_PIN 		52
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
#define FAN2_PIN ORIG_FAN2_PIN
#define PS_ON_PIN ORIG_PS_ON_PIN

#ifndef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT -1
#endif
#define SDCARDDETECT ORIG_SDCARDDETECT

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, ORIG_PS_ON_PIN, \
						HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN,SDSS }
#endif

