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

#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master
#define CURRENT_CONTROL_ALLIGATOR 4  //Use External DAC like Alligator
#define CURRENT_CONTROL_MCP4728 5  // Use an i2c DAC as a digipot like PrintrBoard Rev. F
#define CURRENT_CONTROL_TMC2130 6  // Trinamic TMC2130 configured via SPI

/****************************************************************************************
* Arduino pin assignment
*
*                  ATMega168
*                   +-\/-+
*             PC6  1|    |28  PC5 (AI 5 / D19)
*       (D 0) PD0  2|    |27  PC4 (AI 4 / D18)
*       (D 1) PD1  3|    |26  PC3 (AI 3 / D17)
*       (D 2) PD2  4|    |25  PC2 (AI 2 / D16)
*  PWM+ (D 3) PD3  5|    |24  PC1 (AI 1 / D15)
*       (D 4) PD4  6|    |23  PC0 (AI 0 / D14)
*             VCC  7|    |22  GND
*             GND  8|    |21  AREF
*             PB6  9|    |20  AVCC
*             PB7 10|    |19  PB5 (D 13)
*  PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
*  PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
*       (D 7) PD7 13|    |16  PB2 (D 10) PWM
*       (D 8) PB0 14|    |15  PB1 (D 9)  PWM
*                   +----+
****************************************************************************************/
#if MOTHERBOARD == 0
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega168__
#error Oops!  Make sure you have 'Arduino Diecimila' selected from the boards menu.
#endif

#define ORIG_X_STEP_PIN          2
#define ORIG_X_DIR_PIN           3
#define ORIG_X_ENABLE_PIN       -1
#define ORIG_X_MIN_PIN           4
#define ORIG_X_MAX_PIN           9

#define ORIG_Y_STEP_PIN         10
#define ORIG_Y_DIR_PIN           7
#define ORIG_Y_ENABLE_PIN       -1
#define ORIG_Y_MIN_PIN           8
#define ORIG_Y_MAX_PIN          13

#define ORIG_Z_STEP_PIN         19
#define ORIG_Z_DIR_PIN          18
#define ORIG_Z_ENABLE_PIN        5
#define ORIG_Z_MIN_PIN          17
#define ORIG_Z_MAX_PIN          16

#define ORIG_E0_STEP_PIN         11
#define ORIG_E0_DIR_PIN          12
#define ORIG_E0_ENABLE_PIN       -1

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN            -1
#define ORIG_FAN_PIN            -1
#define ORIG_PS_ON_PIN          15

#define HEATER_0_PIN        6
// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN          0    


#endif



/****************************************************************************************
* Sanguino/RepRap Motherboard with direct-drive extruders
*
*                        ATMega644P
*
*                        +---\/---+
*            (D 0) PB0  1|        |40  PA0 (AI 0 / D31)
*            (D 1) PB1  2|        |39  PA1 (AI 1 / D30)
*       INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D29)
*        PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D28)
*        PWM (D 4) PB4  5|        |36  PA4 (AI 4 / D27)
*       MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D26)
*       MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D25)
*        SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D24)
*                  RST  9|        |32  AREF
*                  VCC 10|        |31  GND
*                  GND 11|        |30  AVCC
*                XTAL2 12|        |29  PC7 (D 23)
*                XTAL1 13|        |28  PC6 (D 22)
*       RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
*       TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
*  INT0 RX1 (D 10) PD2 16|        |25  PC3 (D 19) TMS
*  INT1 TX1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
*       PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
*       PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
*       PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
*                        +--------+
*
****************************************************************************************/
#if MOTHERBOARD == 1
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         15
#define ORIG_X_DIR_PIN          18
#define ORIG_X_ENABLE_PIN       19
#define ORIG_X_MIN_PIN          20
#define ORIG_X_MAX_PIN          21

#define ORIG_Y_STEP_PIN         23
#define ORIG_Y_DIR_PIN          22
#define ORIG_Y_ENABLE_PIN       19
#define ORIG_Y_MIN_PIN          25
#define ORIG_Y_MAX_PIN          26

#define ORIG_Z_STEP_PIN         29
#define ORIG_Z_DIR_PIN          30
#define ORIG_Z_ENABLE_PIN       31
#define ORIG_Z_MIN_PIN           2
#define ORIG_Z_MAX_PIN           1

#define ORIG_E0_STEP_PIN         12
#define ORIG_E0_DIR_PIN          16
#define ORIG_E0_ENABLE_PIN        3

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN             0
#define ORIG_FAN_PIN            -1
#define ORIG_PS_ON_PIN          -1

#define HEATER_0_PIN       14
//D27   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN          4 
#define HEATER_1_PIN   -1
#define TEMP_1_PIN     -1
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27) 28 (29) (30) (31)  */

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS


#endif

#if MOTHERBOARD == 91
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284__)
#error Oops!  Make sure you have 'OMC with Atmega644 at 20 Mhz' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          25
#define ORIG_X_ENABLE_PIN       10
#define ORIG_X_MIN_PIN          0
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         28
#define ORIG_Y_DIR_PIN          27
#define ORIG_Y_ENABLE_PIN       10
#define ORIG_Y_MIN_PIN          1
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         23
#define ORIG_Z_DIR_PIN          22
#define ORIG_Z_ENABLE_PIN       10
#define ORIG_Z_MIN_PIN           2
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN        24
#define ORIG_E0_DIR_PIN         21
#define ORIG_E0_ENABLE_PIN      10

#define PROBE_PIN               13

#define SDPOWER                 -1
#define SDSS                    -1
#define LED_PIN                 -1
#define ORIG_FAN_PIN            14
#define ORIG_PS_ON_PIN          -1

#define ORIG_SDCARDDETECT 	    -1

#define HEATER_0_PIN             3
#define TEMP_0_PIN               0
#define HEATER_1_PIN             4
#define TEMP_1_PIN               1
#define HEATER_2_PIN            -1
#define TEMP_2_PIN               2

#define SCK_PIN          7
#define MISO_PIN         6
#define MOSI_PIN         5

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,

#endif

/****************************************************************************************
* RepRap Motherboard  ****---NOOOOOO RS485/EXTRUDER CONTROLLER!!!!!!!!!!!!!!!!!---*******
*
****************************************************************************************/
#if MOTHERBOARD == 2
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN      15
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    19
#define ORIG_X_MIN_PIN       20
#define ORIG_X_MAX_PIN       21

#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define ORIG_Y_MIN_PIN       25
#define ORIG_Y_MAX_PIN       26

#define ORIG_Z_STEP_PINN     27
#define ORIG_Z_DIR_PINN      28
#define ORIG_Z_ENABLE_PIN    29
#define ORIG_Z_MIN_PIN       30
#define ORIG_Z_MAX_PIN       31

#define ORIG_E0_STEP_PIN      17
#define ORIG_E0_DIR_PIN       16
#define ORIG_E0_ENABLE_PIN    -1

#define SDPOWER          -1
#define SDSS          4
#define LED_PIN          0

#define SD_CARD_WRITE    2
#define SD_CARD_DETECT   3
#define SD_CARD_SELECT   4

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

//pin for controlling the PSU.
#define ORIG_PS_ON_PIN       14

#define ORIG_FAN_PIN         -1

#define HEATER_0_PIN    -1
// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN      -1    


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS


#endif

/****************************************************************************************
* Gen3 PLUS for RepRap Motherboard V1.2
*
****************************************************************************************/
#if MOTHERBOARD == 21
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega644P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN      15
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    19
#define ORIG_X_MIN_PIN       20
#define ORIG_X_MAX_PIN       -1

//y axis pins
#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define ORIG_Y_MIN_PIN       25
#define ORIG_Y_MAX_PIN       -1

//z axis pins
#define ORIG_Z_STEP_PIN      27
#define ORIG_Z_DIR_PIN       28
#define ORIG_Z_ENABLE_PIN    29
#define ORIG_Z_MIN_PIN       30
#define ORIG_Z_MAX_PIN       -1

#define ORIG_E0_DIR_PIN       21
#define ORIG_E0_STEP_PIN  17
#define ORIG_E0_ENABLE_PIN  13

//heaters
// hot end heater
#define HEATER_0_PIN  12    
// heated bed heater
#define HEATER_1_PIN   16    
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

//pin for debugging.
#define DEBUG_PIN        -1
//SD card pin
#define SDSS      4
#define SDPOWER          -1
#define ORIG_FAN_PIN          -1
#define TEMP_0_PIN        0
#define TEMP_1_PIN        5
#define LED_PIN          -1

//pin for controlling the PSU.
#define ORIG_PS_ON_PIN       14
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#endif
//----------end Gen3 PLUS for RepRap Motherboard V1.2--------------

/****************************************************************************************
* Arduino Mega pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 33
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#elif MOTHERBOARD == 34
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define AZTEEG_X3
#elif MOTHERBOARD == 35
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define AZTEEG_X3_PRO
#elif MOTHERBOARD == 39
#define KNOWN_BOARD 1
#define RAMPS_V_1_3
#define ZRIB_V2
#elif MOTHERBOARD == 38
#define RAMPS_V_1_3
#define MPX3
#endif
#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 39 || MOTHERBOARD == 38
#define KNOWN_BOARD 1

#if !(defined (__AVR_ATmega1280__ ) || defined (__AVR_ATmega2560__ ))
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#ifdef RAMPS_V_1_3

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN         26
#define ORIG_E0_DIR_PIN          28
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN         36
#define ORIG_E1_DIR_PIN          34
#define ORIG_E1_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define ORIG_SDCARDDETECT 	    49

#define LED_PIN            13
#define ORIG_FAN_PIN            9
#define ORIG_PS_ON_PIN          12

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define HEATER_2_PIN       9
// ANALOG NUMBERING
#define TEMP_0_PIN         13   
#define TEMP_1_PIN         14
#define TEMP_2_PIN         15
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,


#else // RAMPS_V_1_1 or RAMPS_V_1_2 as default

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          28
#define ORIG_X_ENABLE_PIN       24
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN          -1    //2

#define ORIG_Y_STEP_PIN         38
#define ORIG_Y_DIR_PIN          40
#define ORIG_Y_ENABLE_PIN       36
#define ORIG_Y_MIN_PIN          16
#define ORIG_Y_MAX_PIN          -1    //17

#define ORIG_Z_STEP_PIN         44
#define ORIG_Z_DIR_PIN          46
#define ORIG_Z_ENABLE_PIN       42
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1    //19

#define ORIG_E0_STEP_PIN         32
#define ORIG_E0_DIR_PIN          34
#define ORIG_E0_ENABLE_PIN       30

#define SDPOWER            48
#define SDSS               53
#define LED_PIN            13
#define ORIG_PS_ON_PIN          -1
//#define SCL                21
//#define SDA                20

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS


#ifdef RAMPS_V_1_0 // RAMPS_V_1_0
#define HEATER_0_PIN     12    
#define HEATER_1_PIN     -1    
#define ORIG_FAN_PIN          11

#else // RAMPS_V_1_1 or RAMPS_V_1_2
#define HEATER_0_PIN     10    
#define HEATER_1_PIN      8    
#define ORIG_FAN_PIN      9
#endif

// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN          2    
#define TEMP_1_PIN          1
#endif

// SPI for Max6675 Thermocouple

// these pins are defined in the SD library if building with SD support
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define MAX6675_SS       53

#ifdef AZTEEG_X3
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED 0
#define ORIG_SDCARDDETECT 49
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN           4
#define ORIG_FAN2_PIN          5
#define LIGHT_PIN         6
// Activate beeper on extension shield
#define BEEPER_PIN        33  
#define BEEPER_TYPE        1

// Only available with X3 shield
#define ORIG_E2_STEP_PIN         27  
#define ORIG_E2_DIR_PIN          29 
#define ORIG_E2_ENABLE_PIN       41 
// Only available with X3 shield
#define ORIG_E3_STEP_PIN         23 
#define ORIG_E3_DIR_PIN          25 
#define ORIG_E3_ENABLE_PIN       40 
// Only available with X3 shield
#define HEATER_3_PIN        17 
#define TEMP_3_PIN          12 
#define HEATER_4_PIN        16 
#define TEMP_4_PIN          5 


#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS E3_STEP_PIN,E3_DIR_PIN,E3_ENABLE_PIN,

#endif

#ifdef AZTEEG_X3_PRO
#define SDSUPPORT 1
#define SDCARDDETECTINVERTED false
#define ORIG_SDCARDDETECT 49
#define SDSS               53
#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN           5
#define ORIG_FAN2_PIN          6
#define LIGHT_PIN         11
// Activate beeper on extension shield
#define BEEPER_PIN        33  
#define BEEPER_TYPE        1

#define ORIG_E2_STEP_PIN         23
#define ORIG_E2_DIR_PIN          25
#define ORIG_E2_ENABLE_PIN       40
#define ORIG_E3_STEP_PIN         27
#define ORIG_E3_DIR_PIN          29
#define ORIG_E3_ENABLE_PIN       41
#define ORIG_E4_STEP_PIN         43
#define ORIG_E4_DIR_PIN          37
#define ORIG_E4_ENABLE_PIN       42
#define HEATER_0_PIN       10
// bed
#define HEATER_1_PIN       8  
#define HEATER_2_PIN       9
#define HEATER_3_PIN       16
#define HEATER_4_PIN       17
#define HEATER_5_PIN       4
// ANALOG NUMBERING
#define TEMP_0_PIN         13   
// BED , ANALOG NUMBERING
#define TEMP_1_PIN         14   
#define TEMP_2_PIN         15
#define TEMP_3_PIN         12 
#define TEMP_4_PIN         11 
#define TEMP_5_PIN         10

// Thermocouple 1 and 2
#define TEMP_6_PIN         4   
#define TEMP_7_PIN         5 
#define THERMOCOUPLE_0_PIN         4  
#define THERMOCOUPLE_1_PIN         5  

// Special extension board for x3 pro allows 2 more extruders

#define ORIG_E5_STEP_PIN         12
#define ORIG_E5_DIR_PIN          47
#define ORIG_E5_ENABLE_PIN       63
#define ORIG_E6_STEP_PIN         39
#define ORIG_E6_DIR_PIN          57
#define ORIG_E6_ENABLE_PIN       31

#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN,ORIG_E4_DIR_PIN,ORIG_E4_ENABLE_PIN,
#define E5_PINS ORIG_E5_STEP_PIN,ORIG_E5_DIR_PIN,ORIG_E5_ENABLE_PIN,
#define E6_PINS ORIG_E6_STEP_PIN,ORIG_E6_DIR_PIN,ORIG_E6_ENABLE_PIN,

#endif

// Zonestar ZRIB V2.1 Board
#ifdef ZRIB_V2
#undef HEATER_2_PIN
#define HEATER_2_PIN       7
#define ORIG_FAN2_PIN    6
#define SD_DETECT_PIN     49
#define LCD_PINS_RS     16
#define LCD_PINS_ENABLE   17
#define LCD_PINS_D4     23
#define LCD_PINS_D5     25
#define LCD_PINS_D6     27
#define LCD_PINS_D7     29
#define BEEPER_PIN      37
#endif

#ifdef MPX3
#undef HEATER_1_PIN
#define HEATER_1_PIN    8

#undef FAN_PIN
#define FAN_PIN           9

#undef HEATER_0_PIN
#define HEATER_0_PIN     10

#undef HEATER_2_PIN
#define HEATER_2_PIN      7
#endif

#endif

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

#define HEATER_0_PIN  2
#define TEMP_0_PIN 8

#define HEATER_2_PIN 3
#define TEMP_2_PIN 9

#define HEATER_3_PIN -1
#define TEMP_3_PIN -1

#define ORIG_E0_STEP_PIN         43
#define ORIG_E0_DIR_PIN          45
#define ORIG_E0_ENABLE_PIN       41
#define E0_FAN_PIN           -1
//  #define EXT1_EXTRUDER_COOLER_PIN E0_FAN_PIN

#define ORIG_E1_STEP_PIN         49
#define ORIG_E1_DIR_PIN          47
#define ORIG_E1_ENABLE_PIN       48
#define E1_FAN_PIN           -1
//  #define EXT2_EXTRUDER_COOLER_PIN E1_FAN_PIN

#define LED_PIN            13
#define ORIG_FAN_PIN            7
#define ORIG_PS_ON_PIN          12
#define KILL_PIN           -1
//PIN that has to be turned on right after start, to keep power flowing.
#define SUICIDE_PIN        54  

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define SDPOWER          -1
#define SDSS             53
#define ORIG_SDCARDDETECT	    38

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#endif

/****************************************************************************************
* RUMBA pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 80
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         17
#define ORIG_X_DIR_PIN          16
#define ORIG_X_ENABLE_PIN       48
#define ORIG_X_MIN_PIN          37
#define ORIG_X_MAX_PIN          36

#define ORIG_Y_STEP_PIN         54
#define ORIG_Y_DIR_PIN          47
#define ORIG_Y_ENABLE_PIN       55
#define ORIG_Y_MIN_PIN          35
#define ORIG_Y_MAX_PIN          34

#define ORIG_Z_STEP_PIN         57
#define ORIG_Z_DIR_PIN          56
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          33
#define ORIG_Z_MAX_PIN          32

#define ORIG_E0_STEP_PIN         23
#define ORIG_E0_DIR_PIN          22
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN        26
#define ORIG_E1_DIR_PIN         25
#define ORIG_E1_ENABLE_PIN      27

#define ORIG_E2_STEP_PIN        29
#define ORIG_E2_DIR_PIN         28
#define ORIG_E2_ENABLE_PIN      39

#define LED_PIN            13

#define ORIG_FAN_PIN            7
#define ORIG_FAN2_PIN     8 // (e.g. useful for electronics fan or light on/off) on PIN 8

#define ORIG_PS_ON_PIN          45

 // EXTRUDER 1
#define HEATER_0_PIN       2   
// EXTRUDER 2
#define HEATER_2_PIN       3    
// EXTRUDER 3
#define HEATER_3_PIN       6    
//optional FAN1 can be used as 4th heater output: #define HEATER_4_PIN       8    // EXTRUDER 4
 // BED
#define HEATER_1_PIN       9   

// ANALOG NUMBERING
#define TEMP_0_PIN         15  
#define TEMP_2_PIN         14  
#define TEMP_3_PIN         13  
//optional for extruder 4 or chamber: #define TEMP_2_PIN         12   // ANALOG NUMBERING
#define TEMP_1_PIN       11

#define SDPOWER            -1
#define SDSS               53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,

#endif //MOTHERBOARD==80

/****************************************************************************************
* Duemilanove w/ ATMega328P pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 4
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega328P__
#error Oops!  Make sure you have 'Arduino Duemilanove w/ ATMega328' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         19
#define ORIG_X_DIR_PIN          18
#define ORIG_X_ENABLE_PIN       -1
#define ORIG_X_MIN_PIN          17
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         10
#define ORIG_Y_DIR_PIN           7
#define ORIG_Y_ENABLE_PIN       -1
#define ORIG_Y_MIN_PIN           8
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         13
#define ORIG_Z_DIR_PIN           3
#define ORIG_Z_ENABLE_PIN        2
#define ORIG_Z_MIN_PIN           4
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         11
#define ORIG_E0_DIR_PIN          12
#define ORIG_E0_ENABLE_PIN       -1

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN            -1
#define ORIG_FAN_PIN             5
#define ORIG_PS_ON_PIN          -1

#define HEATER_0_PIN        6
// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_0_PIN          0    
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS


#endif

/****************************************************************************************
* Gen6 pin assignment (5) and Gen6 deluxe assignment (51)
*
****************************************************************************************/
#if MOTHERBOARD == 5 || MOTHERBOARD == 51
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN      15
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    19
#define ORIG_X_MIN_PIN       20
#define ORIG_X_MAX_PIN       -1

//y axis pins
#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define ORIG_Y_MIN_PIN       25
#define ORIG_Y_MAX_PIN       -1

//z axis pins
#define ORIG_Z_STEP_PIN      27
#define ORIG_Z_DIR_PIN       28
#define ORIG_Z_ENABLE_PIN    29
#define ORIG_Z_MIN_PIN       30
#define ORIG_Z_MAX_PIN       -1

//extruder pins
#define ORIG_E0_STEP_PIN      4  
#define ORIG_E0_DIR_PIN       2  
#define ORIG_E0_ENABLE_PIN    3  
#define TEMP_0_PIN      5     
#define HEATER_0_PIN    14    
#if MOTHERBOARD == 5
#define HEATER_1_PIN  -1    
#define TEMP_1_PIN    -1    
#else
#define HEATER_1_PIN   1    
#define TEMP_1_PIN     0    
#endif
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1


#define SDPOWER          -1
// SCL pin of I2C header
#define SDSS          16 
#define LED_PIN         -1   
#define ORIG_FAN_PIN         -1
#define ORIG_PS_ON_PIN       -1
//our pin for debugging.

#define DEBUG_PIN        0

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

#define SCK_PIN          7
#define MISO_PIN         6
#define MOSI_PIN         5

// #define SCL 16
// #define SDA 17

#endif
/****************************************************************************************
* Sanguinololu pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 62
#define SANGUINOLOLU_V_1_2
#endif
#if MOTHERBOARD == 65
#define AZTEEG_X1
#define SANGUINOLOLU_V_1_2
#endif

#if MOTHERBOARD == 6 || MOTHERBOARD == 62 || MOTHERBOARD == 65
#define KNOWN_BOARD 1
//#ifndef __AVR_ATmega644P__
#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         15
#define ORIG_X_DIR_PIN          21
#define ORIG_X_MIN_PIN          18
#define ORIG_X_MAX_PIN           -2

#define ORIG_Y_STEP_PIN         22
#define ORIG_Y_DIR_PIN          23
#define ORIG_Y_MIN_PIN          19
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         3
#define ORIG_Z_DIR_PIN          2
#define ORIG_Z_MIN_PIN          20
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         1
#define ORIG_E0_DIR_PIN          0

#define LED_PIN            -1

#define ORIG_FAN_PIN            -1

#define ORIG_PS_ON_PIN          -1

// (extruder)
#define HEATER_0_PIN       13 

#ifdef SANGUINOLOLU_V_1_2

// (bed)
#define HEATER_1_PIN       12 
#define ORIG_X_ENABLE_PIN       14
#define ORIG_Y_ENABLE_PIN       14
#define ORIG_Z_ENABLE_PIN       26
#define ORIG_E0_ENABLE_PIN       14

#else

#define HEATER_1_PIN       14
#define ORIG_X_ENABLE_PIN       -1
#define ORIG_Y_ENABLE_PIN       -1
#define ORIG_Z_ENABLE_PIN       -1
#define ORIG_E0_ENABLE_PIN       -1

#endif

// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_0_PIN          7   
// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define TEMP_1_PIN          6   
#define SDPOWER          -1
#define SDSS          31
#define SCK_PIN          7
#define MISO_PIN         6
#define MOSI_PIN         5
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#ifdef AZTEEG_X1
#define ORIG_FAN_PIN            4
#endif

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#define E2_PINS

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

#endif

/****************************************************************************************
* Melzi pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 63
#define KNOWN_BOARD 1
#ifndef __AVR_ATmega644P__
#ifndef __AVR_ATmega1284P__
//#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
#endif

#define ORIG_X_STEP_PIN         15
#define ORIG_X_DIR_PIN          21
#define ORIG_X_MIN_PIN          18
#define ORIG_X_MAX_PIN           -2

#define ORIG_Y_STEP_PIN         22
#define ORIG_Y_DIR_PIN          23
#define ORIG_Y_MIN_PIN          19
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         3
#define ORIG_Z_DIR_PIN          2
#define ORIG_Z_MIN_PIN          20
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         1
#define ORIG_E0_DIR_PIN          0
#define ORIG_E0_ENABLE_PIN      14

//29 on Melzi1284p A2
#define PROBE_PIN          -1    

#define LED_PIN            27

#define ORIG_FAN_PIN            4

#define ORIG_PS_ON_PIN          -1

// (extruder)
#define HEATER_0_PIN       13 
#define HEATER_2_PIN       -1
// bed was 10 in older versions,but 12 seems to be correct
#define HEATER_1_PIN     12 

#define ORIG_X_ENABLE_PIN       14
#define ORIG_Y_ENABLE_PIN       14
#define ORIG_Z_ENABLE_PIN       26

// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_0_PIN          7   
// MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define TEMP_1_PIN          6   
#define TEMP_2_PIN         -1
#define SDPOWER            -1
// 31 http://reprap.org/wiki/Melzi#Melzi_Arduino_Pin_Numbers says 31, schematic show pin 37 = PA0 which is arduino pin 31!
#define SDSS               31 
#define SCK_PIN          7
#define MISO_PIN         6
#define MOSI_PIN         5
#define SDSUPPORT 1  // sd card reader on board
#define ORIG_SDCARDDETECT -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

#endif

#if MOTHERBOARD == 66
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// X/Y/Z Steppers and MIN endstops verified

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       63
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN        26 
#define ORIG_E0_DIR_PIN         28 
#define ORIG_E0_ENABLE_PIN      24 

#define ORIG_E1_STEP_PIN        -1
#define ORIG_E1_DIR_PIN         -1
#define ORIG_E1_ENABLE_PIN      -1

#define SDPOWER                 -1
#define SDSS                    25 
#define ORIG_SDCARDDETECT       -1

#define LED_PIN                 13 
#define ORIG_FAN_PIN             8 
#define ORIG_PS_ON_PIN          -1

// schematic: HEATER1 (Extruder)
#define HEATER_0_PIN            10 
// schematic: HEATER2 (Heated Bed)
#define HEATER_1_PIN             9 
#define HEATER_2_PIN            -1

// ANALOG NUMBERING
// schematic: THERM1 (Extruder)
#define TEMP_0_PIN              13 
// schematic: THERM2 (Heated Bed)
#define TEMP_1_PIN              14 
#define TEMP_2_PIN              -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

// following pins (LCD, ENCODER, SDCARD) reverse engineered from schematic diagram:

#ifdef ULTRA_LCD
#ifdef NEWPANEL

#define LCD_PINS_RS            27 
#define LCD_PINS_ENABLE        29 
#define LCD_PINS_D4            37 
#define LCD_PINS_D5            35 
#define LCD_PINS_D6            33 
#define LCD_PINS_D7            31 

#define BTN_EN1                16 
#define BTN_EN2                17 
#define BTN_ENC                23 

#endif
#endif //ULTRA_LCD

#define SCK_PIN                52
#define MISO_PIN               50
#define MOSI_PIN               51
#define MAX6675_SS             53
#endif

/****************************************************************************************
* Gen7 1.1 and above pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 7
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
#error Oops!  Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN      19
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    24
#define ORIG_X_MIN_PIN       7
#define ORIG_X_MAX_PIN       6

//y axis pins
#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define ORIG_Y_MIN_PIN       5
#define ORIG_Y_MAX_PIN       2

//z axis pins
#define ORIG_Z_STEP_PIN      26
#define ORIG_Z_DIR_PIN       25
#define ORIG_Z_ENABLE_PIN    24
#define ORIG_Z_MIN_PIN       1
#define ORIG_Z_MAX_PIN       0

//extruder pins
#define ORIG_E0_STEP_PIN      28
#define ORIG_E0_DIR_PIN       27
#define ORIG_E0_ENABLE_PIN    24
#define TEMP_0_PIN      1
#define TEMP_1_PIN      2
#define HEATER_0_PIN    4
#define HEATER_1_PIN    3
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1


#define SDPOWER          -1
#define SDSS          -1 // SCL pin of I2C header
#define LED_PIN         -1

#define ORIG_FAN_PIN         31
#define ORIG_PS_ON_PIN       15
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

//our pin for debugging.

#define DEBUG_PIN        0

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

#define SCK_PIN          7
#define SDSSORIG         4
#define MISO_PIN         6
#define MOSI_PIN         5

#endif
/****************************************************************************************
* Gen7 1.4.1 pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 71
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
#error Oops!  Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN      29
#define ORIG_X_DIR_PIN       28
#define ORIG_X_ENABLE_PIN    25
#define ORIG_X_MIN_PIN       0
#define ORIG_X_MAX_PIN       -1

//y axis pins
#define ORIG_Y_STEP_PIN      27
#define ORIG_Y_DIR_PIN       26
#define ORIG_Y_ENABLE_PIN    25
#define ORIG_Y_MIN_PIN       1
#define ORIG_Y_MAX_PIN       -1

//z axis pins
#define ORIG_Z_STEP_PIN      23
#define ORIG_Z_DIR_PIN       22
#define ORIG_Z_ENABLE_PIN    25
#define ORIG_Z_MIN_PIN       2
#define ORIG_Z_MAX_PIN       -1

//extruder pins
#define ORIG_E0_STEP_PIN      19
#define ORIG_E0_DIR_PIN       18
#define ORIG_E0_ENABLE_PIN    25
#define TEMP_0_PIN      1
#define TEMP_1_PIN      0
#define HEATER_0_PIN    4
#define HEATER_1_PIN    3
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1


#define SDPOWER          -1
#define SDSS            -1
#define LED_PIN         -1

#define ORIG_FAN_PIN         -1
#define ORIG_PS_ON_PIN       15
//our pin for debugging.

#define DEBUG_PIN        0

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

#define SDPOWER          -1
#define SDSS          -1
// Needs to set this to output to enable SPI even if other SS is used!
#define SDSSORIG         4  

#define SCK_PIN          7
#define MISO_PIN         6
#define MOSI_PIN         5
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#endif

/****************************************************************************************
* Sethi 3D_1 Extruder
*
****************************************************************************************/
#if MOTHERBOARD == 72
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
#error Oops!  Make sure you have 'Sethi' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN      19
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    24
#define ORIG_X_MIN_PIN       2
#define ORIG_X_MAX_PIN       6

//y axis pins
#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define ORIG_Y_MIN_PIN       0
#define ORIG_Y_MAX_PIN       2

//z axis pins
#define ORIG_Z_STEP_PIN      26
#define ORIG_Z_DIR_PIN       25
#define ORIG_Z_ENABLE_PIN    24
#define ORIG_Z_MIN_PIN       1
#define ORIG_Z_MAX_PIN       0

//extruder pins
#define ORIG_E0_STEP_PIN      28
#define ORIG_E0_DIR_PIN       27
#define ORIG_E0_ENABLE_PIN    24
#define TEMP_0_PIN      1
#define TEMP_1_PIN      2
#define HEATER_0_PIN    4
#define HEATER_1_PIN    3


#define SDPOWER          -1
#define SDSS          -1 // SCL pin of I2C header
#define LED_PIN         -1

#define ORIG_FAN_PIN         31
#define ORIG_PS_ON_PIN       15
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

//our pin for debugging.

#define DEBUG_PIN        0

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

#define SCK_PIN          7
#define SDSSORIG         4
#define MISO_PIN         6
#define MOSI_PIN         5

#endif

/****************************************************************************************
    OpenHardware.co.za FrontPrint Controller 1.0
****************************************************************************************/
#if MOTHERBOARD == 73
#define KNOWN_BOARD 1

#if !defined(AVR_ATmega644P) && !defined(AVR_ATmega644) && !defined(AVR_ATmega1284P)
#error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN 29
#define ORIG_X_DIR_PIN 28
#define ORIG_X_ENABLE_PIN 25
#define ORIG_X_MIN_PIN 0
#define ORIG_X_MAX_PIN -1

//y axis pins
#define ORIG_Y_STEP_PIN 27
#define ORIG_Y_DIR_PIN 26
#define ORIG_Y_ENABLE_PIN 25
#define ORIG_Y_MIN_PIN 1
#define ORIG_Y_MAX_PIN -1

//z axis pins
#define ORIG_Z_STEP_PIN 23
#define ORIG_Z_DIR_PIN 22
#define ORIG_Z_ENABLE_PIN 25
#define ORIG_Z_MIN_PIN 2
#define ORIG_Z_MAX_PIN -1

//First extruder pins
#define ORIG_E0_STEP_PIN 19
#define ORIG_E0_DIR_PIN 18
#define ORIG_E0_ENABLE_PIN 25
#define TEMP_0_PIN 1
#define HEATER_0_PIN 4

//Second Extruder pins
#define ORIG_E1_STEP_PIN 20
#define ORIG_E1_DIR_PIN 18
#define ORIG_E1_ENABLE_PIN 25

#define TEMP_2_PIN 7
#define HEATER_2_PIN 13

//Heated Bed Pins
#define HEATER_1_PIN 3
#define TEMP_1_PIN 0

//SD Card Pins
#define SDPOWER -1
#define SDSS 14
#define SCK_PIN 7
#define MISO_PIN 6
#define MOSI_PIN 5

//FAN and ATX Power Supply Control Pins
#define ORIG_FAN_PIN 21 
#define ORIG_PS_ON_PIN 15

#define LED_PIN -1
#define SDSUPPORT 1  // sd card reader on board
#define ORIG_SDCARDDETECT -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#endif

/****************************************************************************************
* Teensylu 0.7 pin assingments (ATMEGA90USB)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
****************************************************************************************/
#if MOTHERBOARD == 8
#define KNOWN_BOARD 1

#define ORIG_X_STEP_PIN         28
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       19
#define ORIG_X_MIN_PIN          25
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         30
#define ORIG_Y_DIR_PIN          31
#define ORIG_Y_ENABLE_PIN       20
#define ORIG_Y_MIN_PIN          26
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         32
#define ORIG_Z_DIR_PIN          33
#define ORIG_Z_ENABLE_PIN       17
#define ORIG_Z_MIN_PIN          27
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          35
#define ORIG_E0_ENABLE_PIN       13

// Extruder - ANALOG PIN NUMBER!
#define TEMP_0_PIN          7 
// Bed - ANALOG PIN NUMBER!
#define TEMP_1_PIN          6 
// Extruder
#define HEATER_0_PIN       15 
// bed
#define HEATER_1_PIN       14 
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#define SDPOWER            -1
#define SDSS                20
#define LED_PIN            -1

#define ORIG_FAN_PIN            16 // Fan
#define ORIG_PS_ON_PIN          -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

#if !SDSUPPORT
// these pins are defined in the SD library if building with SD support
#define SCK_PIN          21
#define MISO_PIN         23
#define MOSI_PIN         22
#endif

#endif

/****************************************************************************************
* Unique One rev. A pin assingments (ATMEGA90USB)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
****************************************************************************************/
#if MOTHERBOARD == 88
#define KNOWN_BOARD 1

#define ORIG_X_STEP_PIN         28
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       19
#define ORIG_X_MIN_PIN          25
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         30
#define ORIG_Y_DIR_PIN          31
#define ORIG_Y_ENABLE_PIN       18
#define ORIG_Y_MIN_PIN          26
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         32
#define ORIG_Z_DIR_PIN          33
#define ORIG_Z_ENABLE_PIN       17
#define ORIG_Z_MIN_PIN          27
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          35
#define ORIG_E0_ENABLE_PIN       12
// Extruder
#define HEATER_0_PIN         8 
// Extruder - ANALOG PIN NUMBER!
#define TEMP_0_PIN           5 

#define ORIG_E1_STEP_PIN         14
#define ORIG_E1_DIR_PIN          13
#define ORIG_E1_ENABLE_PIN       11
// Extruder
#define HEATER_2_PIN         9 
// Extruder - ANALOG PIN NUMBER!
#define TEMP_2_PIN           6 

// bed
#define HEATER_1_PIN       10 
// Bed - ANALOG PIN NUMBER!
#define TEMP_1_PIN          7 

#define SDPOWER            -1
#define SDSS                20
#define LED_PIN            -1

// Fan
#define ORIG_FAN_PIN            16 
#define ORIG_PS_ON_PIN          -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#if !SDSUPPORT
// these pins are defined in the SD library if building with SD support
#define SCK_PIN          21
#define MISO_PIN         23
#define MOSI_PIN         22
#endif

#endif

/****************************************************************************************
* Printrboard Rev. F pin assingments (ATMEGA90USB1286)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
* See http://reprap.org/wiki/Printrboard for more info
*
* Rev. F uses an MCP4728 DAC to generate the Reference Voltage used to determine the
* Stepper Driver's maximum current.
*
* On PrintrBoard, with Sense Resistors = 0.11 Ohms, and 2 Amps maximum current rating,
* the Maximum VRef to send is calculated as:
*
*   2.00 Amps Maximum Output * (8 * 0.11 Ohms) = 1.76 Maximum VRef from MCP4728.
*
****************************************************************************************/
#if MOTHERBOARD == 92
#define KNOWN_BOARD 1

// Definition for current control
#define STEPPER_CURRENT_CONTROL   CURRENT_CONTROL_MCP4728

#define MCP4728_I2C_ADDRESS	0x60 << 1 // Base Address (0x60); Pre-Shifted Left 1 bit for Repetier HAL.
#define MCP4728_GENERALCALL_ADDRESS  0x00 // General Call Address. Weird, but OK...
#define MCP4728_CMD_MULTI_WRITE   0B01000000 // Writes DAC Settings, Does not update EEPROM.
#define MCP4728_CMD_SEQ_WRITE     0B01010000 // Writes DAC Settings, also persists to EEPROM.
#define MCP4728_CMD_GC_UPDATE     0B00001000 // General Call Update - Update all DAC Outputs (Only way to update DAC Outputs on PrintrBoard Rev F because they tied /LDAC to VDD.
#define MCP4728_CMD_GC_RESET      0B00000110 // General Call Reset
#define MCP4728_VREF 		1 // From DataSheet. We will use MCP4728's internal 2.048V as Vref
#define MCP4728_GAIN		0 // From DataSheet. Use 1x Gain Multiplier (0V - 2.048V);
#define MCP4728_NUM_CHANNELS    4 // Duh. Specified here in case there's a beefier chip used on some other board someday.
#define MCP4728_STEPPER_ORDER 	{3,2,1,0} // PrintrBoard wired 'em up backwards. SMH.  X, Y, Z, E
#define MCP4728_VOUT_MAX	3520 // 1.76 Volts * 2000. See DataSheets for the math. Value should be between 0-4095

#define ORIG_X_STEP_PIN         28
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       19
#define ORIG_X_MIN_PIN          47
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         30
#define ORIG_Y_DIR_PIN          31
#define ORIG_Y_ENABLE_PIN       18
// (Was Pin 20 on Rev B-E); Don't use this if you want to use SD card. Use 37 and put the endstop in the e-stop slot!!!
#define ORIG_Y_MIN_PIN          24 
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         32
#define ORIG_Z_DIR_PIN          33
#define ORIG_Z_ENABLE_PIN       17
#define ORIG_Z_MIN_PIN          36
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          35
#define ORIG_E0_ENABLE_PIN       13
// Extruder - ANALOG PIN NUMBER!
#define TEMP_0_PIN          1 
// Bed - ANALOG PIN NUMBER!
#define TEMP_1_PIN          0 
// Extruder
#define HEATER_0_PIN       15 
// bed
#define HEATER_1_PIN       14 
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#define SDPOWER            -1
// (Was Pin 26 on Rev. B-E);  old value 2
#define SDSS               20 
#define LED_PIN            -1

// Fan
#define ORIG_FAN_PIN            16 
#define ORIG_PS_ON_PIN          -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#if !SDSUPPORT
// these pins are defined in the SD library if building with SD support
#define SCK_PIN          21
#define MISO_PIN         23
#define MOSI_PIN         22
#endif

#endif


/****************************************************************************************
* Printrboard Rev. B pin assingments (ATMEGA90USB1286)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/
#if MOTHERBOARD == 9
#define KNOWN_BOARD 1

#define ORIG_X_STEP_PIN         28
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       19
#define ORIG_X_MIN_PIN          47
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         30
#define ORIG_Y_DIR_PIN          31
#define ORIG_Y_ENABLE_PIN       18
// Don't use this if you want to use SD card. Use 37 and put the endstop in the e-stop slot!!!
#define ORIG_Y_MIN_PIN           20 
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         32
#define ORIG_Z_DIR_PIN          33
#define ORIG_Z_ENABLE_PIN       17
#define ORIG_Z_MIN_PIN          36
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          35
#define ORIG_E0_ENABLE_PIN       13
// Extruder - ANALOG PIN NUMBER!
#define TEMP_0_PIN          1 
// Bed - ANALOG PIN NUMBER!
#define TEMP_1_PIN          0 
// Extruder
#define HEATER_0_PIN       15 
// bed
#define HEATER_1_PIN       14 
#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#define SDPOWER            -1
// old value 2
#define SDSS                26 
#define LED_PIN            -1

#define ORIG_FAN_PIN            16
#define ORIG_PS_ON_PIN          -1

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#if !SDSUPPORT
// these pins are defined in the SD library if building with SD support
#define SCK_PIN          21
#define MISO_PIN         23
#define MOSI_PIN         22
#endif

#endif

/****************************************************************************************
* 3D Master pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 12
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// Definition for current control
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_LTC2600
#define LTC2600_CHANNELS {0x30,0x31,0x32,0x33,0x34}
#define LTC2600_NUM_CHANNELS 5
#define	LTC2600_CS_PIN		   92	// PIND.4, 47, DA_CS
#define LTC2600_SCK_PIN		   93	// PIND.5, 48, DA_SCK
#define LTC2600_SDI_PIN		   94	// PIND.6, 49, DA_SDI

// On board beeper, so define values already here
#define BEEPER_PIN 23
#define BEEPER_TYPE 1
#define SDSUPPORT 1  // sd card reader on board
#define ORIG_SDCARDDETECT -1

// digital pin mappings
// PINF.0, 97, STP_DRV1
#define ORIG_X_STEP_PIN         54	
// PINF.1, 96, DIR_DRV1
#define ORIG_X_DIR_PIN          55	
// PIND.7, 50, ENA_DRV1
#define ORIG_X_ENABLE_PIN       38	
// PINE.5,  7, OPTO1
#define ORIG_X_MIN_PIN           3	
#define ORIG_X_MAX_PIN          -1   // PINJ.0, 63, OPTO4 (would be "15", -1 = disabled)

// PINF.6, 91, STP_DRV2
#define ORIG_Y_STEP_PIN         60	
// PINF.7, 90, DIR_DRV2
#define ORIG_Y_DIR_PIN          61	
// PINF.2, 95, ENA_DRV2
#define ORIG_Y_ENABLE_PIN       56	
// PINE.4,  6, OPTO2
#define ORIG_Y_MIN_PIN           2	
// PIND.3, 46, OPTO5 (would be "18", -1 = disabled
#define ORIG_Y_MAX_PIN          -1   

// PINL.3, 38, STP_DRV3
#define ORIG_Z_STEP_PIN         46	
// PINL.1, 36, DIR_DRV3
#define ORIG_Z_DIR_PIN          48	
// PINK.0, 89, ENA_DRV3
#define ORIG_Z_ENABLE_PIN       62	
// PINJ.1, 64, OPTO3
#define ORIG_Z_MIN_PIN          14	
#define ORIG_Z_MAX_PIN          -1   // PIND.2, 45, OPTO6 (would be "19", -1 = disabled)

// PINA.4, 74, STP_DRV4
#define ORIG_E0_STEP_PIN         26	
// PINA.6, 72, DIR_DRV4
#define ORIG_E0_DIR_PIN          28	
// PINA.2, 76 ENA_DRV4
#define ORIG_E0_ENABLE_PIN       24	

// PINC.1, 54, STP_DRV5
#define ORIG_E1_STEP_PIN       36	
// PINC.3, 56, DIR_DRV5
#define ORIG_E1_DIR_PIN        34	
// PINC.7, 60, ENA_DRV5
#define ORIG_E1_ENABLE_PIN     30	

#define SDPOWER            -1
// PINB.0, 19, SS
#define SDSS               53	
// PINB.7, 26, LED13
#define LED_PIN            13	
// OUT1 PINA.3, 75, OUT1
#define ORIG_FAN_PIN            25	
// OUT2
#define FAN_BOARD_PIN      27 
#define ORIG_PS_ON_PIN          -1

// PINB.4, 23, HZ1
#define HEATER_0_PIN       10	
// PINH.6, 18, HZ2
#define HEATER_1_PIN        9	
// PINH.5, 17, HZ3
#define HEATER_2_PIN	    8	

// analog pin mappings
// PINK.5, 84, TH1
#define TEMP_0_PIN         13  
 // PINK.6, 83, TH2 
#define TEMP_1_PIN         14  
// PINK.7, 82, TH3
#define TEMP_2_PIN         15   

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

// these pins are defined in the SD library if building with SD support
// PINB.1, 20, SCK
#define SCK_PIN          52	
// PINB.3, 22, MISO
#define MISO_PIN         50	
// PINB.2, 21, MOSI
#define MOSI_PIN         51	
// PINB.0, 19, SS
#define MAX6675_SS       53	

#endif // MOTHERBOARD == 12


/****************************************************************************************
* MegaTronics
*
****************************************************************************************/
#if MOTHERBOARD == 70
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          28
#define ORIG_X_ENABLE_PIN       24
#define ORIG_X_MIN_PIN          41
 //2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN          37  

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       22
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         54
#define ORIG_Z_DIR_PIN          55
#define ORIG_Z_ENABLE_PIN       56
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN         31
#define ORIG_E0_DIR_PIN          32
#define ORIG_E0_ENABLE_PIN       38

#define ORIG_E1_STEP_PIN        34
#define ORIG_E1_DIR_PIN         36
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13


// IO pin. Buffer needed
#define ORIG_FAN_PIN            7 
#define ORIG_PS_ON_PIN          12

// EXTRUDER 1
#define HEATER_0_PIN       9    
// EXTRUDER 2 (FAN On Sprinter)
#define HEATER_1_PIN       8    
// Heated bed
#define HEATER_2_PIN       10   

#define THERMOCOUPLE_0_PIN 8
// Thermocouple 0 ANALOG NUMBERING
#define TEMP_3_PIN         8   
// ANALOG NUMBERING
#define TEMP_0_PIN         13   
// ANALOG NUMBERING
#define TEMP_1_PIN         15   
#define TEMP_2_PIN         -1   // ANALOG NUMBERING
 // BED
#define HEATER_BED_PIN     10  
// ANALOG NUMBERING
#define TEMP_BED_PIN       14   

// Beeper on AUX-4
#define BEEPER_PIN 33			
#define BEEPER_TYPE 1
#define SDSUPPORT 1  // sd card reader on board
#define ORIG_SDCARDDETECT -1


#ifdef ULTRA_LCD

#ifdef NEWPANEL
#define LCD_PINS_RS 16
#define LCD_PINS_ENABLE 17
#define LCD_PINS_D4 23
#define LCD_PINS_D5 25
#define LCD_PINS_D6 27
#define LCD_PINS_D7 29

//buttons are directly attached using AUX-2
#define BTN_EN1 37
#define BTN_EN2 35
#define BTN_ENC 43 

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#endif
#endif //ULTRA_LCD

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS

#endif

/****************************************************************************************
* FELIXprinters
*
****************************************************************************************/
#if MOTHERBOARD == 101
#define KNOWN_BOARD 1


//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
#endif

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          -1

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          -1

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN         26
#define ORIG_E0_DIR_PIN          28
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN         36
#define ORIG_E1_DIR_PIN          34
#define ORIG_E1_ENABLE_PIN       30



#define LED_PIN            13
#define ORIG_FAN_PIN            9
#define ORIG_PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
//BED
#define HEATER_1_PIN       8 
#define HEATER_2_PIN       7

// ANALOG NUMBERING
#define TEMP_0_PIN         13
   // BED,ANALOG NUMBERING   
#define TEMP_1_PIN         14
#define TEMP_2_PIN         15

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#define SDPOWER            1
#define SDSS               53
#define ORIG_SDCARDDETECT 	   6
#define SDSUPPORT 1            // already defined in config.h
#define SDCARDDETECTINVERTED 1 // already defined in config.h

// these pins are defined in the SD library if building with SD support
// PINB.1, 20, SCK
#define SCK_PIN          52
// PINB.3, 22, MISO
#define MISO_PIN         50
// PINB.2, 21, MOSI
#define MOSI_PIN         51
//53	// PINB.0, 19, SS
#define MAX6675_SS       -1

#define BEEPER_PIN        -1  // Activate beeper on extension shield
#define BEEPER_TYPE        1

#endif//MOTHERBOARD == 101


/****************************************************************************************
* MegaTronics v2.0
*
****************************************************************************************/
#if MOTHERBOARD == 701
#define KNOWN_BOARD 1


#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif


#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          27
#define ORIG_X_ENABLE_PIN       25
#define ORIG_X_MIN_PIN          37
 //2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN          40  

#define ORIG_Y_STEP_PIN         4
#define ORIG_Y_DIR_PIN          54
#define ORIG_Y_ENABLE_PIN       5
#define ORIG_Y_MIN_PIN          41
#define ORIG_Y_MAX_PIN          38

#define ORIG_Z_STEP_PIN         56
#define ORIG_Z_DIR_PIN          60
#define ORIG_Z_ENABLE_PIN       55
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN         35
#define ORIG_E0_DIR_PIN          36
#define ORIG_E0_ENABLE_PIN       34

#define ORIG_E1_STEP_PIN         29
#define ORIG_E1_DIR_PIN          39
#define ORIG_E1_ENABLE_PIN       28

#define ORIG_E2_STEP_PIN         23
#define ORIG_E2_DIR_PIN          24
#define ORIG_E2_ENABLE_PIN       22

#define ORIG_SDCARDDETECT -1		// Ramps does not use this port
#define SDPOWER            -1
#define SDSS               53

#define LED_PIN            13


#define ORIG_FAN_PIN            7
#define ORIG_FAN2_PIN           6
#define ORIG_PS_ON_PIN          12

 // EXTRUDER 1
#define HEATER_0_PIN       9   
// Heated bed
#define HEATER_2_PIN       8    
// EXTRUDER 2
#define HEATER_1_PIN       10   

// Thermistor 0 ANALOG NUMBERING
#define TEMP_0_PIN         13   
// Thermistor 1 ANALOG NUMBERING
#define TEMP_2_PIN         15  
 // Thermistor 2 for heated bed ANALOG NUMBERING 
#define TEMP_1_PIN         14  
 // Thermocouple 0
#define TEMP_3_PIN         8   
 // Thermocouple 1
#define TEMP_4_PIN         4   
#define THERMOCOUPLE_0_PIN 8
#define THERMOCOUPLE_1_PIN 4

// Beeper on AUX-4
#define BEEPER_PIN 64			

#define LCD_PINS_RS 14
#define LCD_PINS_ENABLE 15
#define LCD_PINS_D4 30
#define LCD_PINS_D5 31
#define LCD_PINS_D6 32
#define LCD_PINS_D7 33


//buttons are directly attached using AUX-2
#define BTN_EN1 59
#define BTN_EN2 64
#define BTN_ENC 43 

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,

#endif

/****************************************************************************************
* Minitronics v1.0
*
****************************************************************************************/
#if MOTHERBOARD == 702
#define KNOWN_BOARD 1


#ifndef __AVR_ATmega1281__
#error Oops! Make sure you have 'Minitronics ' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN 48
#define ORIG_X_DIR_PIN 47
#define ORIG_X_ENABLE_PIN 49
#define ORIG_X_MIN_PIN 5
//2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN -1 

#define ORIG_Y_STEP_PIN 39
#define ORIG_Y_DIR_PIN 40 
#define ORIG_Y_ENABLE_PIN 38
#define ORIG_Y_MIN_PIN 2
#define ORIG_Y_MAX_PIN -1 

#define ORIG_Z_STEP_PIN 42
#define ORIG_Z_DIR_PIN 43 
#define ORIG_Z_ENABLE_PIN 41
#define ORIG_Z_MIN_PIN 6
#define ORIG_Z_MAX_PIN -1

#define ORIG_E0_STEP_PIN 45
#define ORIG_E0_DIR_PIN 44
#define ORIG_E0_ENABLE_PIN 27

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 35
#define ORIG_E1_ENABLE_PIN 37

#define ORIG_E2_STEP_PIN -1
#define ORIG_E2_DIR_PIN -1
#define ORIG_E2_ENABLE_PIN -1

#define SDPOWER -1
#define SDSS 16
#define SCK_PIN 10
#define MISO_PIN 12
#define MOSI_PIN 11
#define LED_PIN 46

#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN -1
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1

// EXTRUDER 1
#define HEATER_0_PIN 7 
// BED
#define HEATER_1_PIN 3 
// EXTRUDER 2
#define HEATER_2_PIN 8 
#define HEATER_3_PIN -1


// ANALOG NUMBERING
#define TEMP_0_PIN 7
// BED SENSOR ANALOG NUMBERING
#define TEMP_1_PIN 6 
// ANALOG NUMBERING
#define TEMP_2_PIN 6 
// ANALOG NUMBERING
#define TEMP_3_PIN -1 


#define BEEPER_PIN -1

#define ORIG_SDCARDDETECT -1 // Megatronics does not use this port
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS

#endif


/****************************************************************************************
* MegaTronics v3.0
*
****************************************************************************************/
#if MOTHERBOARD == 703
#define KNOWN_BOARD 1


#ifndef __AVR_ATmega2560__
#error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif


#define ORIG_X_STEP_PIN 58
#define ORIG_X_DIR_PIN 57
#define ORIG_X_ENABLE_PIN 59
#define ORIG_X_MIN_PIN 37
//2 //Max endstops default to disabled "-1", set to commented value to enable.
#define ORIG_X_MAX_PIN 40 

#define ORIG_Y_STEP_PIN 5 
#define ORIG_Y_DIR_PIN 17
#define ORIG_Y_ENABLE_PIN 4
#define ORIG_Y_MIN_PIN 41
#define ORIG_Y_MAX_PIN 38

#define ORIG_Z_STEP_PIN 16
#define ORIG_Z_DIR_PIN 11 
#define ORIG_Z_ENABLE_PIN 3
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19

#define ORIG_E0_STEP_PIN 28
#define ORIG_E0_DIR_PIN 27
#define ORIG_E0_ENABLE_PIN 29

#define ORIG_E1_STEP_PIN 25
#define ORIG_E1_DIR_PIN 24
#define ORIG_E1_ENABLE_PIN 26

#define ORIG_E2_STEP_PIN 22
#define ORIG_E2_DIR_PIN 60
#define ORIG_E2_ENABLE_PIN 23

#define ORIG_SDCARDDETECT -1	 // Ramps does not use this port
#define SDPOWER -1
#define SDSS 53

#define LED_PIN 13

#define ORIG_FAN_PIN 6
#define ORIG_FAN2_PIN 7

#define ORIG_PS_ON_PIN 12
//#define KILL_PIN -1

// EXTRUDER 0 - changed 10-9-2015
#define HEATER_0_PIN 2
// EXTRUDER 1 - changed 10-9-2015
#define HEATER_2_PIN 9 
// EXTRUDER 2 - changed 10-9-2015
#define HEATER_3_PIN 8 
// heater bed
#define HEATER_1_PIN 10 

/*
Temperature sensors
ANALOG NUMBERING!

Thermistors
T0=15
T1=14
T2=13
T3=12

Thermocouple
S0=11
S1=10
S2(ext)=8
S3(ext)=9
*/

// Extruder 1 - Thermistor 1
#define TEMP_0_PIN 15
// Extruder 2 - Thermistor 2
#define TEMP_2_PIN 14
// Extruder 3 - Thermistor 3
#define TEMP_3_PIN 13 
// Heated bed - Thermistor 4
#define TEMP_1_PIN 12 

#define THERMOCOUPLE_0_PIN 11
#define THERMOCOUPLE_1_PIN 10
#define THERMOCOUPLE_2_PIN 8
#define THERMOCOUPLE_3_PIN 9

// Beeper on AUX-4
#define BEEPER_PIN 61	 
#define SDSUPPORT 1 // sd card reader on board

// #define UI_DISPLAY_RS_PIN 32
// #define UI_DISPLAY_ENABLE_PIN 31
// #define UI_DISPLAY_D4_PIN 14
// #define UI_DISPLAY_D5_PIN 30
// #define UI_DISPLAY_D6_PIN 39
// #define UI_DISPLAY_D7_PIN 15


//buttons are directly attached using AUX-2
////encoder A 59
////encoder B 64
////encoder click 33 //the click

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,


#endif


#if MOTHERBOARD == 301
#define KNOWN_BOARD
/*****************************************************************
* RAMBo Pin Assignments
******************************************************************/

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN     37
#define ORIG_X_DIR_PIN      48
#define ORIG_X_MIN_PIN      12
#define ORIG_X_MAX_PIN      24
#define ORIG_X_ENABLE_PIN   29
#define X_MS1_PIN      40
#define X_MS2_PIN      41

#define ORIG_Y_STEP_PIN     36
#define ORIG_Y_DIR_PIN      49
#define ORIG_Y_MIN_PIN      11
#define ORIG_Y_MAX_PIN      23
#define ORIG_Y_ENABLE_PIN   28
#define Y_MS1_PIN      69
#define Y_MS2_PIN      39

#define ORIG_Z_STEP_PIN     35
#define ORIG_Z_DIR_PIN      47
#define ORIG_Z_MIN_PIN      10
#define ORIG_Z_MAX_PIN      30
#define ORIG_Z_ENABLE_PIN   27
#define Z_MS1_PIN      68
#define Z_MS2_PIN      67

#define HEATER_0_PIN   9
#define TEMP_0_PIN     0

#define HEATER_1_PIN   3
// This is T2 on the board!
#define TEMP_1_PIN     2

#define HEATER_2_PIN   7
// This is T1 on the board!
#define TEMP_2_PIN     1
// T3 on board
#define TEMP_3_PIN     7

#define ORIG_E0_STEP_PIN    34
#define ORIG_E0_DIR_PIN     43
#define ORIG_E0_ENABLE_PIN  26
#define E0_MS1_PIN     65
#define E0_MS2_PIN     66

#define ORIG_E1_STEP_PIN    33
#define ORIG_E1_DIR_PIN     42
#define ORIG_E1_ENABLE_PIN  25
#define E1_MS1_PIN     63
#define E1_MS2_PIN     64

#define DIGIPOTSS_PIN  38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER        -1
#define SDSS           53
#define LED_PIN        13
#define ORIG_FAN_PIN        8
#define ORIG_FAN2_PIN    6
#define ORIG_FAN3_PIN    2
#define ORIG_PS_ON_PIN      4
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,E0_MS1_PIN,E0_MS2_PIN,
#define E1_PINS

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define MAX6675_SS       53
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_DIGIPOT

#endif


/***************************************************************************************
*PiBot for Repetier pins assignment
*illustration :
* PiBot for Repetier V1.0-1.3       =314
* PiBot for Repetier V1.4-1.6       =315
* PiBot Controller Rev2.0           =316
***************************************************************************************/

#if MOTHERBOARD == 316
#define MOTHERBOARD 314
#define PiBot_V_2_0 true
#define PiBot_HD_VERSION "Rev2.0"
#ifndef Thermistor_Solution
#define Thermistor_Solution   0
#endif
// define for temperature senser chip  connection
// temperature sensor port in Rev 2.0 for max6675
// #define MAX6675_TEMP_Senser false   ///*** canceled hardware integration
// Temperature sensor port in Rev 2.0 for AD595
#define AD595_TEMP_Senser false    /////*** you can input at port 59 60 61 ///Analoge Pin 8  9 10
#endif

#if MOTHERBOARD == 315
#define MOTHERBOARD 314
#define PiBot_V_1_6 true
#define PiBot_HD_VERSION "Rev1.6"
#endif

#if MOTHERBOARD == 314
#define KNOWN_BOARD 1
#define PiBot true

#if PiBot_V_1_4==true || PiBot_V_1_6==true || PiBot_V_2_0==true
#define PiBot_V_1_0 false
#else
#define PiBot_V_1_0 true
#define PiBot_HD_VERSION "Rev1.0"
#endif

#ifndef PiBotSemitec
#define PiBotSemitec false    // for semitec NTC 100K  b=4230(test value) default b=4267
#endif

#ifndef PI_PRUSA_I3
#define PI_PRUSA_I3 false
#endif

#ifndef PiBotMachine
#define PiBotMachine false  // if use for pibot 3D printer uncomment this line.
#endif

#if PI_PRUSA_I3==true
#define PiBotMachine true  // if use for pibot 3D printer uncomment this line.
#endif

// define in AVR public files, when you finish the chip select.
#ifndef __AVR_ATmega1280__
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
#endif

#if PiBot_V_1_0
#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN         26
#define ORIG_E0_DIR_PIN          28
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN         36
#define ORIG_E1_DIR_PIN          34
#define ORIG_E1_ENABLE_PIN       30

#define SDPOWER            -1
#define ORIG_SDCARDDETECT 	    49

#define LED_PIN            13
#define ORIG_FAN_PIN       7        
#define ORIG_PS_ON_PIN     12
#define KILL_PIN           -1

// Extuder1
#define HEATER_0_PIN       8   
// Bed
#define HEATER_1_PIN       10  
// Extuder2  
#define HEATER_2_PIN       9   
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         13   
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         15   
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN         14   

// ISP for TFcard
#define SDSS             53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#endif     //  end PiBot for Repetier V1.0


#if PiBot_V_1_4==true || PiBot_V_1_6==true
#define ORIG_X_STEP_PIN         4
#define ORIG_X_DIR_PIN          17
#define ORIG_X_ENABLE_PIN       16
#define ORIG_X_MIN_PIN          37
#define ORIG_X_MAX_PIN          34

#define ORIG_Y_STEP_PIN         56
#define ORIG_Y_DIR_PIN          55
#define ORIG_Y_ENABLE_PIN       54
#define ORIG_Y_MIN_PIN          36
#define ORIG_Y_MAX_PIN          33

#define ORIG_Z_STEP_PIN         59
#define ORIG_Z_DIR_PIN          58
#define ORIG_Z_ENABLE_PIN       57
#define ORIG_Z_MIN_PIN          35
#define ORIG_Z_MAX_PIN          32

#define ORIG_E0_STEP_PIN         24
#define ORIG_E0_DIR_PIN          23
#define ORIG_E0_ENABLE_PIN       22

#define ORIG_E1_STEP_PIN         27
#define ORIG_E1_DIR_PIN          26
#define ORIG_E1_ENABLE_PIN       25

//uncomment when use 3rd extruder
#define ORIG_E2_STEP_PIN        15
#define ORIG_E2_DIR_PIN          14
#define ORIG_E2_ENABLE_PIN       39

//uncomment when use 4th extruder
#define ORIG_E3_STEP_PIN         41
#define ORIG_E3_DIR_PIN          38
#define ORIG_E3_ENABLE_PIN       13

#define SDPOWER            -1
#define ORIG_SDCARDDETECT 	   10

#define LED_PIN            30
#define ORIG_FAN_PIN       7      
//uncomment when the 2nd fan used - works only without heated bed!
#define ORIG_FAN2_PIN      2
#define PS_ON_PIN          40
#define KILL_PIN           -1

// Extuder1
#define HEATER_0_PIN       3    
// Bed
#define HEATER_1_PIN       12   
// Extuder2
#define HEATER_2_PIN       6    
 // Extuder3
#define HEATER_3_PIN       9   
// Extuder4
#define HEATER_4_PIN       11   

// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         14   
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         15   
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN         13   
// ANALOG NUMBERING  Extuder3
#define TEMP_3_PIN         12   
// ANALOG NUMBERING  Extuder4
#define TEMP_4_PIN         11   

//PiBot use this pin as Z-Probing pin
#define PiBot_Z_PROBE_PIN  64   

// ISP for TFcard
#define SDSS             53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,

#endif     // end PiBot for Repetier V1.4 or V1.6


#if PiBot_V_2_0==true
#define ORIG_X_STEP_PIN         24
#define ORIG_X_DIR_PIN          23
#define ORIG_X_ENABLE_PIN       22
#define ORIG_X_MIN_PIN          62
#define ORIG_X_MAX_PIN          63

#define ORIG_Y_STEP_PIN         27
#define ORIG_Y_DIR_PIN          26
#define ORIG_Y_ENABLE_PIN       25
#define ORIG_Y_MIN_PIN          64
#define ORIG_Y_MAX_PIN          65

#define ORIG_Z_STEP_PIN         15
#define ORIG_Z_DIR_PIN          14
#define ORIG_Z_ENABLE_PIN       39
#define ORIG_Z_MIN_PIN          66
#define ORIG_Z_MAX_PIN          67

#define ORIG_E0_STEP_PIN        32
#define ORIG_E0_DIR_PIN         31
#define ORIG_E0_ENABLE_PIN      30

#define ORIG_E1_STEP_PIN        35
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      33

// =========================================
#define PiBot_Z_PROBE_PIN  68   // PiBot use this pin as Z-Probing pin

#define LED_PIN            -1

// PWM6 fan1
#define ORIG_FAN_PIN        6    
 // PWM7 fan2 
#define ORIG_FAN2_PIN       7    

// have hardware in PiBot HDV2.0
#define ORIG_PS_ON_PIN          17   
#define KILL_PIN           -1

// PWM5 Extuder1
#define HEATER_0_PIN       5     
// PWM4 Bed
#define HEATER_1_PIN       4     
// PWM2 Extuder2
#define HEATER_2_PIN       2     
// for Pibot   PWM Extuder3
#define HEATER_3_PIN       -1    
// for Pibot   PWM Extuder4
#define HEATER_4_PIN       -1    

#if Thermistor_Solution==0       // 000    0 2 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         2     
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         0     
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN         4     
#endif
#if Thermistor_Solution==1       // 001  1 2 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         2     
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         1    
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         4     
#endif
#if Thermistor_Solution==2       // 010  0 3 4
#define TEMP_0_PIN         3     
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         0    
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         4    
#endif
#if Thermistor_Solution==3       // 011  1 3 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         3    
#define TEMP_1_PIN         1     
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         4     
#endif
#if Thermistor_Solution==4       // 100  0 2 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         2
// ANALOG NUMBERING  Bed     
#define TEMP_1_PIN         0    
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         5     
#endif
#if Thermistor_Solution==5       // 101 1 2 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         2    
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         1    
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         5   
#endif
#if Thermistor_Solution==6       // 110  0 3 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         3  
// ANALOG NUMBERING  Bed  
#define TEMP_1_PIN         0     
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         5    
#endif
#if Thermistor_Solution==7       // 111  1 3 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         3     
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         1     
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         5    
#endif
#if !defined(TEMP_0_PIN) || !defined(TEMP_1_PIN) || !defined(TEMP_2_PIN) ||!defined(Thermistor_Solution)
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         2     
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN         0     
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         4    
#endif

// ad595 temp senser
#if AD595_TEMP_Senser==true
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         8   
// ANALOG NUMBERING  Bed 
#define TEMP_1_PIN         9   
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN         10   
#endif

// max6675 ISP port   temp->ISP-ENABLE
// these enable pins have been isolated by capacitor
/*#if MAX6675_TEMP_Senser==true
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN         14   // ANALOG NUMBERING   Extuder1
#define TEMP_1_PIN         15   // ANALOG NUMBERING   Bed
#define TEMP_2_PIN         13   // ANALOG NUMBERING   Extuder2
//uncomment when 3 extruder used
#define TEMP_3_PIN         12   // ANALOG NUMBERING   Extuder3
#endif*/

// ISP for TFcard
#define SDPOWER           -1
#define ORIG_SDCARDDETECT 	  40
#define SDSS              53
#define SCK_PIN           52
#define MISO_PIN          50
#define MOSI_PIN          51

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#endif  // end PiBot Controller Rev 2.0

#endif   // end  PiBot for Repetier

/****************************************************************************************
* Sanguish Beta pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 501
#define KNOWN_BOARD

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
#error Oops! Make sure you have 'Your MCU/Bootloader' selected from the 'Tools -> Boards' menu.
#endif

//x axis pins
#define ORIG_X_STEP_PIN 28
#define ORIG_X_DIR_PIN 27
#define ORIG_X_ENABLE_PIN 29
#define ORIG_X_MIN_PIN 2
#define ORIG_X_MAX_PIN -1

//y axis pins
#define ORIG_Y_STEP_PIN 25
#define ORIG_Y_DIR_PIN 24
#define ORIG_Y_ENABLE_PIN 26
#define ORIG_Y_MIN_PIN 5
#define ORIG_Y_MAX_PIN -1

//z axis pins
#define ORIG_Z_STEP_PIN 22
#define ORIG_Z_DIR_PIN 21
#define ORIG_Z_ENABLE_PIN 23
#define ORIG_Z_MIN_PIN 1
#define ORIG_Z_MAX_PIN -1

//extruder pins
#define ORIG_E0_STEP_PIN 19
#define ORIG_E0_DIR_PIN 18
#define ORIG_E0_ENABLE_PIN 20
#define TEMP_0_PIN 1
#define TEMP_1_PIN 0
#define HEATER_0_PIN 3
#define HEATER_1_PIN 4


#define SDPOWER -1
#define SDSS -1
#define LED_PIN -1

#define ORIG_FAN_PIN -1
#define ORIG_PS_ON_PIN 0
//our pin for debugging.

#define DEBUG_PIN -1

//our RS485 pins
#define TX_ENABLE_PIN	12
#define RX_ENABLE_PIN	13

#define SDPOWER -1
#define SDSS -1

#define SCK_PIN 7
#define MISO_PIN 6
#define MOSI_PIN 5
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS
#endif

/****************************************************************************************
 * SAV MK1 (ATMEGA90USB)
 * Requires the Teensyduino software!
 * See how to install software etc. here: http://reprap.org/wiki/SAV_MKI#Setting-up_the_environment
 ****************************************************************************************/
 #if MOTHERBOARD == 89
 #define KNOWN_BOARD 1
 
 #define ORIG_X_STEP_PIN         28
 #define ORIG_X_DIR_PIN          29
 #define ORIG_X_ENABLE_PIN       19
 #define ORIG_X_MIN_PIN          25
 #define ORIG_X_MAX_PIN          -1
 
 #define ORIG_Y_STEP_PIN         30
 #define ORIG_Y_DIR_PIN          31
 #define ORIG_Y_ENABLE_PIN       18
 #define ORIG_Y_MIN_PIN          26
 #define ORIG_Y_MAX_PIN          -1
 
 #define ORIG_Z_STEP_PIN         32
 #define ORIG_Z_DIR_PIN          33
 #define ORIG_Z_ENABLE_PIN       17
 #define ORIG_Z_MIN_PIN          -1
 #define ORIG_Z_MAX_PIN          27
 
 #define ORIG_E0_STEP_PIN         34
 #define ORIG_E0_DIR_PIN          35
 #define ORIG_E0_ENABLE_PIN       13
 
 // Extruder - ANALOG PIN NUMBER!
 #define TEMP_0_PIN          7
 // Bed - ANALOG PIN NUMBER!
 #define TEMP_1_PIN          6
 // Extruder
 #define HEATER_0_PIN       15
 // bed
 #define HEATER_1_PIN       14
 #define HEATER_2_PIN   -1
 #define TEMP_2_PIN     -1
 
 #define SDPOWER            -1
 #define SDSS                20
 #define LED_PIN            -1
 // Fan
 #define ORIG_FAN_PIN            16
 #define ORIG_PS_ON_PIN          -1
 
 #define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
 #define E1_PINS
 
 #if !SDSUPPORT
 // these pins are defined in the SD library if building with SD support
 #define SCK_PIN          21
 #define MISO_PIN         23
 #define MOSI_PIN         22
 #endif
 
 #endif

/****************************************************************************************
* MJRice Pica Rev B * 
****************************************************************************************/
#if MOTHERBOARD == 183
#define KNOWN_BOARD 1

#define ORIG_X_DIR_PIN 54
#define ORIG_X_STEP_PIN 55
#define ORIG_Y_DIR_PIN 56
#define ORIG_Y_STEP_PIN 57
#define ORIG_Z_DIR_PIN 58
#define ORIG_Z_STEP_PIN 59
#define ORIG_X_ENABLE_PIN 60
#define ORIG_Y_ENABLE_PIN 61
#define ORIG_Z_ENABLE_PIN 62
#define ORIG_X_MIN_PIN 14
#define ORIG_X_MAX_PIN 15
#define ORIG_Y_MIN_PIN 16
#define ORIG_Y_MAX_PIN 17
#define ORIG_Z_MIN_PIN 23
#define ORIG_Z_MAX_PIN 22

#define ORIG_E0_STEP_PIN 67
#define ORIG_E0_DIR_PIN 24
#define ORIG_E0_ENABLE_PIN 26

#define ORIG_E1_STEP_PIN 68
#define ORIG_E1_DIR_PIN 28
#define ORIG_E1_ENABLE_PIN 27

#define SSR_PIN 6

#define SDPOWER -1
#define SDSS 53
#define ORIG_SDCARDDETECT 49

#define LED_PIN -1
#define ORIG_FAN_PIN 11
#define ORIG_FAN2_PIN 12
#define ORIG_PS_ON_PIN -1

#define HEATER_0_PIN 10
#define HEATER_1_PIN 8
#define HEATER_2_PIN 2
// ANALOG NUMBERING
#define TEMP_0_PIN 9

#define TEMP_1_PIN 10
#define TEMP_2_PIN 11
#define TEMP_3_PIN 12
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

// LCD interface pins
#define LCD_PINS_RS 33
#define LCD_PINS_ENABLE 30
#define LCD_PINS_D4 35
#define LCD_PINS_D5 32
#define LCD_PINS_D6 37
#define LCD_PINS_D7 36
#define BTN_EN1 47
#define BTN_EN2 48
#define BTN_ENC 31 //the click
#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#endif

/****************************************************************************************
* MJRice Pica Rev C * 
****************************************************************************************/
#if MOTHERBOARD == 184
#define KNOWN_BOARD 1

#define ORIG_X_DIR_PIN 54
#define ORIG_X_STEP_PIN 55
#define ORIG_Y_DIR_PIN 56
#define ORIG_Y_STEP_PIN 57
#define ORIG_Z_DIR_PIN 58
#define ORIG_Z_STEP_PIN 59
#define ORIG_X_ENABLE_PIN 60
#define ORIG_Y_ENABLE_PIN 61
#define ORIG_Z_ENABLE_PIN 62
#define ORIG_X_MIN_PIN 14
#define ORIG_X_MAX_PIN 15
#define ORIG_Y_MIN_PIN 16
#define ORIG_Y_MAX_PIN 17
#define ORIG_Z_MIN_PIN 23
#define ORIG_Z_MAX_PIN 22

#define ORIG_E0_STEP_PIN 67
#define ORIG_E0_DIR_PIN 24
#define ORIG_E0_ENABLE_PIN 26

#define ORIG_E1_STEP_PIN 68
#define ORIG_E1_DIR_PIN 28
#define ORIG_E1_ENABLE_PIN 27

#define SSR_PIN 6

#define SDPOWER -1
#define SDSS 53
#define ORIG_SDCARDDETECT 49

#define LED_PIN -1
#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN 7
#define ORIG_PS_ON_PIN -1

#define HEATER_0_PIN 10
#define HEATER_1_PIN 8
#define HEATER_2_PIN 2
// ANALOG NUMBERING
#define TEMP_0_PIN 9

#define TEMP_1_PIN 10
#define TEMP_2_PIN 11
#define TEMP_3_PIN 12
#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

// LCD interface pins
#define LCD_PINS_RS 33
#define LCD_PINS_ENABLE 30
#define LCD_PINS_D4 35
#define LCD_PINS_D5 32
#define LCD_PINS_D6 37
#define LCD_PINS_D7 36
#define BTN_EN1 47
#define BTN_EN2 48
#define BTN_ENC 31
#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

// these pins are defined in the SD library if building with SD support
#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51
#define MAX6675_SS 53

#endif


#if MOTHERBOARD == 999
#define KNOWN_BOARD
#include "userpins.h"
#endif



#ifndef CPU_ARCH  // Set default architecture
#define CPU_ARCH ARCH_AVR
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

#ifndef E2_PINS
#define E2_PINS
#endif

#if NUM_EXTRUDER==1
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

#define E6_STEP_PIN ORIG_E6_STEP_PIN
#define E6_DIR_PIN ORIG_E6_DIR_PIN
#define E6_ENABLE_PIN ORIG_E6_ENABLE_PIN

#define FAN_PIN ORIG_FAN_PIN
#ifdef ORIG_FAN2_PIN
#define FAN2_PIN ORIG_FAN2_PIN
#endif

#define PS_ON_PIN ORIG_PS_ON_PIN

#ifndef ORIG_SDCARDDETECT
#define ORIG_SDCARDDETECT -1
#endif
#define SDCARDDETECT ORIG_SDCARDDETECT

#define SENSITIVE_PINS {0, 1, ORIG_X_STEP_PIN, ORIG_X_DIR_PIN, ORIG_X_ENABLE_PIN, ORIG_X_MIN_PIN, ORIG_X_MAX_PIN, \
        ORIG_Y_STEP_PIN, ORIG_Y_DIR_PIN, ORIG_Y_ENABLE_PIN, ORIG_Y_MIN_PIN, ORIG_Y_MAX_PIN, ORIG_Z_STEP_PIN,\
        ORIG_Z_DIR_PIN, ORIG_Z_ENABLE_PIN, ORIG_Z_MIN_PIN, ORIG_Z_MAX_PIN, LED_PIN, ORIG_PS_ON_PIN, \
        HEATER_0_PIN, HEATER_1_PIN, /*ORIG_FAN_PIN,*/ E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN,SDSS }
#endif

