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

*/

#define ARCH_AVR 1
#define ARCH_ARM 2

#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master

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

#define X_STEP_PIN          2
#define X_DIR_PIN           3
#define X_ENABLE_PIN       -1
#define X_MIN_PIN           4
#define X_MAX_PIN           9

#define Y_STEP_PIN         10
#define Y_DIR_PIN           7
#define Y_ENABLE_PIN       -1
#define Y_MIN_PIN           8
#define Y_MAX_PIN          13

#define Z_STEP_PIN         19
#define Z_DIR_PIN          18
#define Z_ENABLE_PIN        5
#define Z_MIN_PIN          17
#define Z_MAX_PIN          16

#define E0_STEP_PIN         11
#define E0_DIR_PIN          12
#define E0_ENABLE_PIN       -1

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN            -1
#define FAN_PIN            -1
#define PS_ON_PIN          15
#define KILL_PIN           -1

#define HEATER_0_PIN        6
#define TEMP_0_PIN          0    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!


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

#define X_STEP_PIN         15
#define X_DIR_PIN          18
#define X_ENABLE_PIN       19
#define X_MIN_PIN          20
#define X_MAX_PIN          21

#define Y_STEP_PIN         23
#define Y_DIR_PIN          22
#define Y_ENABLE_PIN       19
#define Y_MIN_PIN          25
#define Y_MAX_PIN          26

#define Z_STEP_PIN         29
#define Z_DIR_PIN          30
#define Z_ENABLE_PIN       31
#define Z_MIN_PIN           2
#define Z_MAX_PIN           1

#define E0_STEP_PIN         12
#define E0_DIR_PIN          16
#define E0_ENABLE_PIN        3

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN             0
#define FAN_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       14
#define TEMP_0_PIN          4 //D27   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!

/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27) 28 (29) (30) (31)  */

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS


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

#define X_STEP_PIN      15
#define X_DIR_PIN       18
#define X_ENABLE_PIN    19
#define X_MIN_PIN       20
#define X_MAX_PIN       21

#define Y_STEP_PIN      23
#define Y_DIR_PIN       22
#define Y_ENABLE_PIN    24
#define Y_MIN_PIN       25
#define Y_MAX_PIN       26

#define Z_STEP_PINN     27
#define Z_DIR_PINN      28
#define Z_ENABLE_PIN    29
#define Z_MIN_PIN       30
#define Z_MAX_PIN       31

#define E0_STEP_PIN      17
#define E0_DIR_PIN       16
#define E0_ENABLE_PIN    -1

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
#define PS_ON_PIN       14

#define FAN_PIN         -1
#define KILL_PIN        -1

#define HEATER_0_PIN    -1
#define TEMP_0_PIN      -1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!


#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
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
    #define X_STEP_PIN      15
    #define X_DIR_PIN       18
    #define X_ENABLE_PIN    19
    #define X_MIN_PIN       20
    #define X_MAX_PIN       -1

    //y axis pins
    #define Y_STEP_PIN      23
    #define Y_DIR_PIN       22
    #define Y_ENABLE_PIN    24
    #define Y_MIN_PIN       25
    #define Y_MAX_PIN       -1

    //z axis pins
    #define Z_STEP_PIN      27
    #define Z_DIR_PIN       28
    #define Z_ENABLE_PIN    29
    #define Z_MIN_PIN       30
    #define Z_MAX_PIN       -1

    #define E0_DIR_PIN       21
    #define E0_STEP_PIN  17
    #define E0_ENABLE_PIN  13

    //heaters
    #define HEATER_0_PIN  12    // hot end heater
    #define HEATER_1_PIN   16    // heated bed heater

    //pin for debugging.
    #define DEBUG_PIN        -1
    //SD card pin
    #define SDSS      4
    #define SDPOWER          -1
    #define FAN_PIN          -1
    #define TEMP_0_PIN        0
    #define TEMP_1_PIN        5
    #define LED_PIN          -1

    //pin for controlling the PSU.
    #define PS_ON_PIN       14
    #define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
    #define E1_PINS
#endif
    //----------end Gen3 PLUS for RepRap Motherboard V1.2--------------

/****************************************************************************************
* Arduino Mega pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 33
  #define MOTHERBOARD 3
  #define RAMPS_V_1_3
#endif
#if MOTHERBOARD == 3
  #define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
  #ifndef __AVR_ATmega1280__
    #ifndef __AVR_ATmega2560__
     #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
    #endif
  #endif

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#ifdef RAMPS_V_1_3

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN          3
#define X_MAX_PIN          2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN         26
#define E0_DIR_PIN          28
#define E0_ENABLE_PIN       24

#define E1_STEP_PIN         36
#define E1_DIR_PIN          34
#define E1_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT 	    49

#define LED_PIN            13
#define FAN_PIN            9
#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define HEATER_2_PIN       9
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING
#define TEMP_2_PIN         15
#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,


#else // RAMPS_V_1_1 or RAMPS_V_1_2 as default

#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define X_MIN_PIN           3
#define X_MAX_PIN          -1    //2

#define Y_STEP_PIN         38
#define Y_DIR_PIN          40
#define Y_ENABLE_PIN       36
#define Y_MIN_PIN          16
#define Y_MAX_PIN          -1    //17

#define Z_STEP_PIN         44
#define Z_DIR_PIN          46
#define Z_ENABLE_PIN       42
#define Z_MIN_PIN          18
#define Z_MAX_PIN          -1    //19

#define E0_STEP_PIN         32
#define E0_DIR_PIN          34
#define E0_ENABLE_PIN       30

#define SDPOWER            48
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN          -1
#define KILL_PIN           -1
//#define SCL                21
//#define SDA                20

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS


#ifdef RAMPS_V_1_0 // RAMPS_V_1_0
  #define HEATER_0_PIN     12    // RAMPS 1.0
  #define HEATER_1_PIN     -1    // RAMPS 1.0
  #define FAN_PIN          11    // RAMPS 1.0

#else // RAMPS_V_1_1 or RAMPS_V_1_2
  #define HEATER_0_PIN     10    // RAMPS 1.1
  #define HEATER_1_PIN      8    // RAMPS 1.1
  #define FAN_PIN           9    // RAMPS 1.1
#endif

#define TEMP_0_PIN          2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_1_PIN          1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#endif

// SPI for Max6675 Thermocouple 

// these pins are defined in the SD library if building with SD support  
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define MAX6675_SS       53


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

#define X_STEP_PIN         17
#define X_DIR_PIN          16
#define X_ENABLE_PIN       48
#define X_MIN_PIN          37
#define X_MAX_PIN          36   //Max endstops default to disabled "-1"

#define Y_STEP_PIN         54
#define Y_DIR_PIN          47 
#define Y_ENABLE_PIN       55
#define Y_MIN_PIN          35
#define Y_MAX_PIN          34 

#define Z_STEP_PIN         57 
#define Z_DIR_PIN          56
#define Z_ENABLE_PIN       62 
#define Z_MIN_PIN          33
#define Z_MAX_PIN          32

#define E0_STEP_PIN         23
#define E0_DIR_PIN          22
#define E0_ENABLE_PIN       24

#define E1_STEP_PIN        26
#define E1_DIR_PIN         25
#define E1_ENABLE_PIN      27

#define E2_STEP_PIN        29
#define E2_DIR_PIN         28
#define E2_ENABLE_PIN      39

#define LED_PIN            13

#define FAN_PIN            7 
//additional FAN1 PIN (e.g. useful for electronics fan or light on/off) on PIN 8

#define PS_ON_PIN          45
#define KILL_PIN           46

#define HEATER_0_PIN       2    // EXTRUDER 1
#define HEATER_1_PIN       3    // EXTRUDER 2
#define HEATER_2_PIN       6    // EXTRUDER 3
//optional FAN1 can be used as 4th heater output: #define HEATER_3_PIN       8    // EXTRUDER 4
#define HEATER_BED_PIN     9    // BED

#define TEMP_0_PIN         15   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING
#define TEMP_2_PIN         13   // ANALOG NUMBERING
//optional for extruder 4 or chamber: #define TEMP_2_PIN         12   // ANALOG NUMBERING
#define TEMP_BED_PIN       11   // ANALOG NUMBERING

#define SDPOWER            -1
#define SDSS               53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,
#define E2_PINS E2_STEP_PIN,E2_DIR_PIN,E2_ENABLE_PIN,

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

#define X_STEP_PIN         19
#define X_DIR_PIN          18
#define X_ENABLE_PIN       -1
#define X_MIN_PIN          17
#define X_MAX_PIN          -1

#define Y_STEP_PIN         10
#define Y_DIR_PIN           7
#define Y_ENABLE_PIN       -1
#define Y_MIN_PIN           8
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         13
#define Z_DIR_PIN           3
#define Z_ENABLE_PIN        2
#define Z_MIN_PIN           4
#define Z_MAX_PIN          -1

#define E0_STEP_PIN         11
#define E0_DIR_PIN          12
#define E0_ENABLE_PIN       -1

#define SDPOWER          -1
#define SDSS          -1
#define LED_PIN            -1
#define FAN_PIN             5
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN        6
#define TEMP_0_PIN          0    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
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
    #define X_STEP_PIN      15
    #define X_DIR_PIN       18
    #define X_ENABLE_PIN    19
    #define X_MIN_PIN       20
    #define X_MAX_PIN       -1
    
    //y axis pins
    #define Y_STEP_PIN      23
    #define Y_DIR_PIN       22
    #define Y_ENABLE_PIN    24
    #define Y_MIN_PIN       25
    #define Y_MAX_PIN       -1
    
    //z axis pins
    #define Z_STEP_PIN      27
    #define Z_DIR_PIN       28
    #define Z_ENABLE_PIN    29
    #define Z_MIN_PIN       30
    #define Z_MAX_PIN       -1
    
    //extruder pins
    #define E0_STEP_PIN      4     //Edited @ EJE Electronics 20100715
    #define E0_DIR_PIN       2     //Edited @ EJE Electronics 20100715
    #define E0_ENABLE_PIN    3     //Added @ EJE Electronics 20100715
    #define TEMP_0_PIN      5     //changed @ rkoeppl 20110410
    #define HEATER_0_PIN    14    //changed @ rkoeppl 20110410
#if MOTHERBOARD == 5
    #define HEATER_1_PIN  -1    //changed @ rkoeppl 20110410
    #define TEMP_1_PIN    -1    //changed @ rkoeppl 20110410
#else
    #define HEATER_1_PIN   1    //changed @ rkoeppl 20110410
    #define TEMP_1_PIN     0    //changed @ rkoeppl 20110410
#endif
    
    
    #define SDPOWER          -1
    #define SDSS          16 // SCL pin of I2C header
    #define LED_PIN         -1    //changed @ rkoeppl 20110410
    #define TEMP_1_PIN      -1    //changed @ rkoeppl 20110410
    #define FAN_PIN         -1    //changed @ rkoeppl 20110410
    #define PS_ON_PIN       -1    //changed @ rkoeppl 20110410
    //our pin for debugging.
    
    #define DEBUG_PIN        0
    
    //our RS485 pins
    #define TX_ENABLE_PIN	12
    #define RX_ENABLE_PIN	13
    #define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
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
    #define MOTHERBOARD 6
    #define SANGUINOLOLU_V_1_2 
#endif
#if MOTHERBOARD == 6
    #define KNOWN_BOARD 1
    //#ifndef __AVR_ATmega644P__
    #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
      #error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
    #endif

    #define X_STEP_PIN         15
    #define X_DIR_PIN          21
    #define X_MIN_PIN          18
    #define X_MAX_PIN           -2

    #define Y_STEP_PIN         22
    #define Y_DIR_PIN          23
    #define Y_MIN_PIN          19
    #define Y_MAX_PIN          -1

    #define Z_STEP_PIN         3
    #define Z_DIR_PIN          2
    #define Z_MIN_PIN          20
    #define Z_MAX_PIN          -1

    #define E0_STEP_PIN         1
    #define E0_DIR_PIN          0

    #define LED_PIN            -1

    #define FAN_PIN            -1 

    #define PS_ON_PIN          -1
    #define KILL_PIN           -1

    #define HEATER_0_PIN       13 // (extruder)

    #ifdef SANGUINOLOLU_V_1_2

      #define HEATER_1_PIN       12 // (bed)
      #define X_ENABLE_PIN       14
      #define Y_ENABLE_PIN       14
      #define Z_ENABLE_PIN       26
      #define E0_ENABLE_PIN       14

    #else

      #define HEATER_1_PIN       14  // (bed)
      #define X_ENABLE_PIN       -1
      #define Y_ENABLE_PIN       -1
      #define Z_ENABLE_PIN       -1
      #define E0_ENABLE_PIN       -1

    #endif

    #define TEMP_0_PIN          7   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
    #define TEMP_1_PIN          6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
    #define SDPOWER          -1
    #define SDSS          31
    
    #define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
    #define E1_PINS

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

#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_MIN_PIN          18
#define X_MAX_PIN           -2

#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_MIN_PIN          19
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         3
#define Z_DIR_PIN          2
#define Z_MIN_PIN          20
#define Z_MAX_PIN          -1

#define E0_STEP_PIN         1
#define E0_DIR_PIN          0
#define E0_ENABLE_PIN      14

#define PROBE_PIN          -1    //29 on Melzi1284p A2

#define LED_PIN            27

#define FAN_PIN            4 

#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       13 // (extruder)
#define HEATER_2_PIN       -1
#ifdef REPRAPPRO_HUXLEY
  #define HEATER_1_PIN     10 // bed (change to 10 for gate pin of MOSFET on heated bed)
#else
  #define HEATER_1_PIN     12
#endif
#define X_ENABLE_PIN       14
#define Y_ENABLE_PIN       14
#define Z_ENABLE_PIN       26

#define TEMP_0_PIN          7   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_1_PIN          6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define TEMP_2_PIN         -1
#define SDPOWER            -1
#define SDSS               31

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS

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
#define X_STEP_PIN      19
#define X_DIR_PIN       18
#define X_ENABLE_PIN    24
#define X_MIN_PIN       7
#define X_MAX_PIN       6
    
//y axis pins
#define Y_STEP_PIN      23
#define Y_DIR_PIN       22
#define Y_ENABLE_PIN    24
#define Y_MIN_PIN       5
#define Y_MAX_PIN       2
    
//z axis pins
#define Z_STEP_PIN      26
#define Z_DIR_PIN       25
#define Z_ENABLE_PIN    24
#define Z_MIN_PIN       1
#define Z_MAX_PIN       0
    
//extruder pins
#define E0_STEP_PIN      28     
#define E0_DIR_PIN       27     
#define E0_ENABLE_PIN    24     
#define TEMP_0_PIN      1 
#define TEMP_1_PIN      2    
#define HEATER_0_PIN    4    
#define HEATER_1_PIN    3    
    
    
#define SDPOWER          -1
#define SDSS          -1 // SCL pin of I2C header
#define LED_PIN         -1    
       
#define FAN_PIN         31    
#define PS_ON_PIN       15    
#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
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
  #define X_STEP_PIN      29
  #define X_DIR_PIN       28
  #define X_ENABLE_PIN    25
  #define X_MIN_PIN       0
  #define X_MAX_PIN       -1
    
  //y axis pins
  #define Y_STEP_PIN      27
  #define Y_DIR_PIN       26
  #define Y_ENABLE_PIN    25
  #define Y_MIN_PIN       1
  #define Y_MAX_PIN       -1
    
  //z axis pins
  #define Z_STEP_PIN      23
  #define Z_DIR_PIN       22
  #define Z_ENABLE_PIN    25
  #define Z_MIN_PIN       2
  #define Z_MAX_PIN       -1
    
  //extruder pins
  #define E0_STEP_PIN      19     
  #define E0_DIR_PIN       18     
  #define E0_ENABLE_PIN    25     
  #define TEMP_0_PIN      0 
  #define TEMP_1_PIN      1    
  #define HEATER_0_PIN    4    
  #define HEATER_1_PIN    3    
    
    
  #define SDPOWER          -1
  #define SDSS            -1 
  #define LED_PIN         -1    
       
  #define FAN_PIN         -1    
  #define PS_ON_PIN       15    
    //our pin for debugging.
    
  #define DEBUG_PIN        0
    
    //our RS485 pins
  #define TX_ENABLE_PIN	12
  #define RX_ENABLE_PIN	13

  #define SDPOWER          -1
  #define SDSS          -1
  #define SDSSORIG         4  // Needs to set this to output to enable SPI even if other SS is used!

  #define SCK_PIN          7
  #define MISO_PIN         6
  #define MOSI_PIN         5
  #define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
  #define E1_PINS
#endif

/****************************************************************************************
* Teensylu 0.7 pin assingments (ATMEGA90USB)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
****************************************************************************************/
#if MOTHERBOARD == 8
#define MOTHERBOARD 8
#define KNOWN_BOARD 1

#define X_STEP_PIN         28
#define X_DIR_PIN          29
#define X_ENABLE_PIN       19
#define X_MIN_PIN          25
#define X_MAX_PIN          -1

#define Y_STEP_PIN         30
#define Y_DIR_PIN          31
#define Y_ENABLE_PIN       20 //26
#define Y_MIN_PIN          26 // 20
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         32
#define Z_DIR_PIN          33
#define Z_ENABLE_PIN       17
#define Z_MIN_PIN          27
#define Z_MAX_PIN          -1

#define E0_STEP_PIN         34
#define E0_DIR_PIN          35
#define E0_ENABLE_PIN       13

#define TEMP_0_PIN          7 // Extruder - ANALOG PIN NUMBER!
#define TEMP_1_PIN          6 // Bed - ANALOG PIN NUMBER! 
#define HEATER_0_PIN       15 // Extruder
#define HEATER_1_PIN       14 // bed

#define SDPOWER            -1
#define SDSS                20
#define LED_PIN            -1

#define FAN_PIN            16 // Fan
#define PS_ON_PIN          -1

#define KILL_PIN           -1
#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN          21
  #define MISO_PIN         22
  #define MOSI_PIN         23
#endif

#endif


/****************************************************************************************
* Printrboard Rev. B pin assingments (ATMEGA90USB1286)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/
#if MOTHERBOARD == 9
#define MOTHERBOARD 9
#define KNOWN_BOARD 1

#define X_STEP_PIN         28
#define X_DIR_PIN          29
#define X_ENABLE_PIN       19
#define X_MIN_PIN          47
#define X_MAX_PIN          -1

#define Y_STEP_PIN         30
#define Y_DIR_PIN          31
#define Y_ENABLE_PIN       18
#define Y_MIN_PIN           20 // Don't use this if you want to use SD card. Use 37 and put the endstop in the e-stop slot!!!
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         32
#define Z_DIR_PIN          33
#define Z_ENABLE_PIN       17
#define Z_MIN_PIN          36
#define Z_MAX_PIN          -1

#define E0_STEP_PIN         34
#define E0_DIR_PIN          35
#define E0_ENABLE_PIN       13
#define TEMP_0_PIN          1 // Extruder - ANALOG PIN NUMBER!
#define TEMP_1_PIN          0 // Bed - ANALOG PIN NUMBER! 
#define HEATER_0_PIN       15 // Extruder
#define HEATER_1_PIN       14 // bed

#define SDPOWER            -1
#define SDSS                26 // old value 2
#define LED_PIN            -1

#define FAN_PIN            16 // Fan
#define PS_ON_PIN          -1

#define KILL_PIN           -1
#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS
#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN          21
  #define MISO_PIN         22
  #define MOSI_PIN         23
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
#define SDSUPPORT true  // sd card reader on board
#define SDCARDDETECT -1

// digital pin mappings
#define X_STEP_PIN         54	// PINF.0, 97, STP_DRV1
#define X_DIR_PIN          55	// PINF.1, 96, DIR_DRV1
#define X_ENABLE_PIN       38	// PIND.7, 50, ENA_DRV1
#define X_MIN_PIN           3	// PINE.5,  7, OPTO1
#define X_MAX_PIN          -1   // PINJ.0, 63, OPTO4 (would be "15", -1 = disabled)

#define Y_STEP_PIN         60	// PINF.6, 91, STP_DRV2
#define Y_DIR_PIN          61	// PINF.7, 90, DIR_DRV2
#define Y_ENABLE_PIN       56	// PINF.2, 95, ENA_DRV2
#define Y_MIN_PIN           2	// PINE.4,  6, OPTO2
#define Y_MAX_PIN          -1   // PIND.3, 46, OPTO5 (would be "18", -1 = disabled

#define Z_STEP_PIN         46	// PINL.3, 38, STP_DRV3
#define Z_DIR_PIN          48	// PINL.1, 36, DIR_DRV3
#define Z_ENABLE_PIN       62	// PINK.0, 89, ENA_DRV3
#define Z_MIN_PIN          14	// PINJ.1, 64, OPTO3
#define Z_MAX_PIN          -1   // PIND.2, 45, OPTO6 (would be "19", -1 = disabled)

#define E0_STEP_PIN         26	// PINA.4, 74, STP_DRV4
#define E0_DIR_PIN          28	// PINA.6, 72, DIR_DRV4
#define E0_ENABLE_PIN       24	// PINA.2, 76 ENA_DRV4

#define E1_STEP_PIN       36	// PINC.1, 54, STP_DRV5
#define E1_DIR_PIN        34	// PINC.3, 56, DIR_DRV5
#define E1_ENABLE_PIN     30	// PINC.7, 60, ENA_DRV5

#define SDPOWER            -1
#define SDSS               53	// PINB.0, 19, SS
#define LED_PIN            13	// PINB.7, 26, LED13
#define FAN_PIN            25	// OUT1 PINA.3, 75, OUT1
#define FAN_BOARD_PIN      27   // OUT2
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       10	// PINB.4, 23, HZ1
#define HEATER_1_PIN        9	// PINH.6, 18, HZ2
#define HEATER_2_PIN	    8	// PINH.5, 17, HZ3

// analog pin mappings
#define TEMP_0_PIN         13   // PINK.5, 84, TH1
#define TEMP_1_PIN         14   // PINK.6, 83, TH2
#define TEMP_2_PIN         15   // PINK.7, 82, TH3

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,

// these pins are defined in the SD library if building with SD support  
#define SCK_PIN          52	// PINB.1, 20, SCK
#define MISO_PIN         50	// PINB.3, 22, MISO
#define MOSI_PIN         51	// PINB.2, 21, MOSI
#define MAX6675_SS       53	// PINB.0, 19, SS

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




#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define X_MIN_PIN          41
#define X_MAX_PIN          37   //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define Y_STEP_PIN         60 // A6
#define Y_DIR_PIN          61 // A7
#define Y_ENABLE_PIN       22
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15   //15

#define Z_STEP_PIN         54 // A0
#define Z_DIR_PIN          55 // A1
#define Z_ENABLE_PIN       56 // A2
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN         31
#define E0_DIR_PIN          32
#define E0_ENABLE_PIN       38

#define E1_STEP_PIN        34
#define E1_DIR_PIN         36
#define E1_ENABLE_PIN      30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13


#define FAN_PIN            7 // IO pin. Buffer needed
#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       9    // EXTRUDER 1
#define HEATER_1_PIN       8    // EXTRUDER 2 (FAN On Sprinter)
#define HEATER_2_PIN       10   // Heated bed  

#define TEMP_3_PIN         8   // Thermocouple 0 ANALOG NUMBERING
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         15   // ANALOG NUMBERING
#define TEMP_2_PIN         -1   // ANALOG NUMBERING
#define HEATER_BED_PIN     10   // BED
#define TEMP_BED_PIN       14   // ANALOG NUMBERING

#define BEEPER_PIN 33			// Beeper on AUX-4
#define BEEPER_TYPE 1
#define SDSUPPORT true  // sd card reader on board
#define SDCARDDETECT -1


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
    #define BTN_ENC 43  //the click
    
    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0
    
#endif
#endif //ULTRA_LCD

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS

#endif

/****************************************************************************************
* MegaTronics v2.0
*
****************************************************************************************/
#if MOTHERBOARD == 701
 #define KNOWN_BOARD 1
 
 
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
 
 
 #define X_STEP_PIN         26
 #define X_DIR_PIN          27
 #define X_ENABLE_PIN       25
 #define X_MIN_PIN          37
 #define X_MAX_PIN          40   //2 //Max endstops default to disabled "-1", set to commented value to enable.
 
 #define Y_STEP_PIN         4 // A6
 #define Y_DIR_PIN          54 // A0
 #define Y_ENABLE_PIN       5
 #define Y_MIN_PIN          41
 #define Y_MAX_PIN          38   //15
 
 #define Z_STEP_PIN         56 // A2
 #define Z_DIR_PIN          60 // A6
 #define Z_ENABLE_PIN       55 // A1
 #define Z_MIN_PIN          18
 #define Z_MAX_PIN          19
 
 #define E0_STEP_PIN         35
 #define E0_DIR_PIN          36
 #define E0_ENABLE_PIN       34
 
 #define E1_STEP_PIN         29
 #define E1_DIR_PIN          39
 #define E1_ENABLE_PIN       28
 
 #define E2_STEP_PIN         23
 #define E2_DIR_PIN          24
 #define E2_ENABLE_PIN       22
 
 #define SDCARDDETECT -1		// Ramps does not use this port
 #define SDPOWER            -1
 #define SDSS               53

 #define LED_PIN            13
 
 
 #define FAN_PIN            7 
 #define FAN2_PIN           6
 #define PS_ON_PIN          12
 #define KILL_PIN           -1
 
 #define HEATER_0_PIN       9    // EXTRUDER 1
 #define HEATER_1_PIN       8    // Heated bed 
 #define HEATER_2_PIN       10   // EXTRUDER 2  
 
 #define TEMP_0_PIN         13   // Thermistor 0 ANALOG NUMBERING   
 #define TEMP_1_PIN         15   // Thermistor 1 ANALOG NUMBERING   
 #define TEMP_2_PIN         14   // Thermistor 2 for heated bed ANALOG NUMBERING
 #define TEMP_3_PIN         8    // Thermocouple 0
 #define TEMP_4_PIN         4    // Thermocouple 1
  
 #define BEEPER_PIN 64			// Beeper on AUX-4
 
 #define LCD_PINS_RS 14 
 #define LCD_PINS_ENABLE 15
 #define LCD_PINS_D4 30
 #define LCD_PINS_D5 31 
 #define LCD_PINS_D6 32
 #define LCD_PINS_D7 33
 
 
 //buttons are directly attached using AUX-2
 #define BTN_EN1 59
 #define BTN_EN2 64
 #define BTN_ENC 43  //the click
 
 #define BLEN_C 2
 #define BLEN_B 1
 #define BLEN_A 0

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,
#define E2_PINS E2_STEP_PIN,E2_DIR_PIN,E2_ENABLE_PIN,

#endif


#if MOTHERBOARD == 301
#define KNOWN_BOARD
/*****************************************************************
* Rambo Pin Assignments
******************************************************************/

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN     37
#define X_DIR_PIN      48
#define X_MIN_PIN      12
#define X_MAX_PIN      24
#define X_ENABLE_PIN   29
#define X_MS1_PIN      40
#define X_MS2_PIN      41

#define Y_STEP_PIN     36
#define Y_DIR_PIN      49
#define Y_MIN_PIN      11
#define Y_MAX_PIN      23
#define Y_ENABLE_PIN   28
#define Y_MS1_PIN      69
#define Y_MS2_PIN      39

#define Z_STEP_PIN     35
#define Z_DIR_PIN      47
#define Z_MIN_PIN      10
#define Z_MAX_PIN      30
#define Z_ENABLE_PIN   27
#define Z_MS1_PIN      68
#define Z_MS2_PIN      67

#define HEATER_BED_PIN 3
#define TEMP_BED_PIN   2 

#define HEATER_0_PIN   9
#define TEMP_0_PIN     0

#define HEATER_1_PIN   7
#define TEMP_1_PIN     1

#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#define E0_STEP_PIN    34
#define E0_DIR_PIN     43
#define E0_ENABLE_PIN  26
#define E0_MS1_PIN     65
#define E0_MS2_PIN     66

#define E1_STEP_PIN    33
#define E1_DIR_PIN     42
#define E1_ENABLE_PIN  25
#define E1_MS1_PIN     63
#define E1_MS2_PIN     64

#define DIGIPOTSS_PIN  38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER        -1
#define SDSS           53
#define LED_PIN        13
#define FAN_PIN        8
#define PS_ON_PIN      4
#define KILL_PIN       -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,E0_MS1_PIN,E0_MS2_PIN,
#define E1_PINS

#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define MAX6675_SS       53
#define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_DIGIPOT

#endif

#if MOTHERBOARD == 401
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define X_STEP_PIN     36
#define X_DIR_PIN      48
#define X_MIN_PIN      12
#define X_MAX_PIN      19
#define X_ENABLE_PIN   29
#define X_MS1_PIN      40
#define X_MS2_PIN      41

#define Y_STEP_PIN     36
#define Y_DIR_PIN      49
#define Y_MIN_PIN      11
#define Y_MAX_PIN      18
#define Y_ENABLE_PIN   28
#define Y_MS1_PIN      69
#define Y_MS2_PIN      39

#define Z_STEP_PIN     35
#define Z_DIR_PIN      47
#define Z_MIN_PIN      10
#define Z_MAX_PIN      15
#define Z_ENABLE_PIN   27
#define Z_MS1_PIN      68
#define Z_MS2_PIN      67

#define HEATER_BED_PIN 3
#define TEMP_BED_PIN   65 

#define HEATER_0_PIN   9
#define TEMP_0_PIN     63

#define HEATER_1_PIN   7
#define TEMP_1_PIN     64

#define HEATER_2_PIN   -1
#define TEMP_2_PIN     -1

#define E0_STEP_PIN    34
#define E0_DIR_PIN     43
#define E0_ENABLE_PIN  26
#define E0_MS1_PIN     65
#define E0_MS2_PIN     66

#define E1_STEP_PIN    33
#define E1_DIR_PIN     42
#define E1_ENABLE_PIN  25
#define E1_MS1_PIN     63
#define E1_MS2_PIN     64

#define SDPOWER        -1
#define SDSS           53
#define LED_PIN        13
#define FAN_PIN        8
#define PS_ON_PIN      4
#define KILL_PIN       -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,E0_MS1_PIN,E0_MS2_PIN,
#define E1_PINS

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

#if NUM_EXTRUDER==1
#define E1_PINS
#endif

#if NUM_EXTRUDER<3
#define E2_PINS
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, PS_ON_PIN, \
                        HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN,SDSS }
#endif

