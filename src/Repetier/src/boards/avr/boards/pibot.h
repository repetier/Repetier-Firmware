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
#define Thermistor_Solution 0
#endif
// define for temperature senser chip  connection
// temperature sensor port in Rev 2.0 for max6675
// #define MAX6675_TEMP_Senser false   ///*** canceled hardware integration
// Temperature sensor port in Rev 2.0 for AD595
#define AD595_TEMP_Senser false /////*** you can input at port 59 60 61 ///Analoge Pin 8  9 10
#endif

#if MOTHERBOARD == 315
#define MOTHERBOARD 314
#define PiBot_V_1_6 true
#define PiBot_HD_VERSION "Rev1.6"
#endif

#if MOTHERBOARD == 314
#define KNOWN_BOARD 1
#define PiBot true

#if PiBot_V_1_4 == true || PiBot_V_1_6 == true || PiBot_V_2_0 == true
#define PiBot_V_1_0 false
#else
#define PiBot_V_1_0 true
#define PiBot_HD_VERSION "Rev1.0"
#endif

#ifndef PiBotSemitec
#define PiBotSemitec false // for semitec NTC 100K  b=4230(test value) default b=4267
#endif

#ifndef PI_PRUSA_I3
#define PI_PRUSA_I3 false
#endif

#ifndef PiBotMachine
#define PiBotMachine false // if use for pibot 3D printer uncomment this line.
#endif

#if PI_PRUSA_I3 == true
#define PiBotMachine true // if use for pibot 3D printer uncomment this line.
#endif

// define in AVR public files, when you finish the chip select.
#ifndef __AVR_ATmega1280__
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
#endif

#if PiBot_V_1_0
#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_ENABLE_PIN 38
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN 2

#define ORIG_Y_STEP_PIN 60
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_ENABLE_PIN 56
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN 15

#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_ENABLE_PIN 62
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19

#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1
#define ORIG_SDCARDDETECT 49

#define LED_PIN 13
#define ORIG_FAN_PIN 7
#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1

// Extuder1
#define HEATER_0_PIN 8
// Bed
#define HEATER_1_PIN 10
// Extuder2
#define HEATER_2_PIN 9
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 13
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 15
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN 14

// ISP for TFcard
#define SDSS 53
#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#endif //  end PiBot for Repetier V1.0

#if PiBot_V_1_4 == true || PiBot_V_1_6 == true
#define ORIG_X_STEP_PIN 4
#define ORIG_X_DIR_PIN 17
#define ORIG_X_ENABLE_PIN 16
#define ORIG_X_MIN_PIN 37
#define ORIG_X_MAX_PIN 34

#define ORIG_Y_STEP_PIN 56
#define ORIG_Y_DIR_PIN 55
#define ORIG_Y_ENABLE_PIN 54
#define ORIG_Y_MIN_PIN 36
#define ORIG_Y_MAX_PIN 33

#define ORIG_Z_STEP_PIN 59
#define ORIG_Z_DIR_PIN 58
#define ORIG_Z_ENABLE_PIN 57
#define ORIG_Z_MIN_PIN 35
#define ORIG_Z_MAX_PIN 32

#define ORIG_E0_STEP_PIN 24
#define ORIG_E0_DIR_PIN 23
#define ORIG_E0_ENABLE_PIN 22

#define ORIG_E1_STEP_PIN 27
#define ORIG_E1_DIR_PIN 26
#define ORIG_E1_ENABLE_PIN 25

//uncomment when use 3rd extruder
#define ORIG_E2_STEP_PIN 15
#define ORIG_E2_DIR_PIN 14
#define ORIG_E2_ENABLE_PIN 39

//uncomment when use 4th extruder
#define ORIG_E3_STEP_PIN 41
#define ORIG_E3_DIR_PIN 38
#define ORIG_E3_ENABLE_PIN 13

#define SDPOWER -1
#define ORIG_SDCARDDETECT 10

#define LED_PIN 30
#define ORIG_FAN_PIN 7
//uncomment when the 2nd fan used - works only without heated bed!
#define ORIG_FAN2_PIN 2
#define PS_ON_PIN 40
#define KILL_PIN -1

// Extuder1
#define HEATER_0_PIN 3
// Bed
#define HEATER_1_PIN 12
// Extuder2
#define HEATER_2_PIN 6
// Extuder3
#define HEATER_3_PIN 9
// Extuder4
#define HEATER_4_PIN 11

// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 14
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 15
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN 13
// ANALOG NUMBERING  Extuder3
#define TEMP_3_PIN 12
// ANALOG NUMBERING  Extuder4
#define TEMP_4_PIN 11

//PiBot use this pin as Z-Probing pin
#define PiBot_Z_PROBE_PIN 64

// ISP for TFcard
#define SDSS 53
#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,

#endif // end PiBot for Repetier V1.4 or V1.6

#if PiBot_V_2_0 == true
#define ORIG_X_STEP_PIN 24
#define ORIG_X_DIR_PIN 23
#define ORIG_X_ENABLE_PIN 22
#define ORIG_X_MIN_PIN 62
#define ORIG_X_MAX_PIN 63

#define ORIG_Y_STEP_PIN 27
#define ORIG_Y_DIR_PIN 26
#define ORIG_Y_ENABLE_PIN 25
#define ORIG_Y_MIN_PIN 64
#define ORIG_Y_MAX_PIN 65

#define ORIG_Z_STEP_PIN 15
#define ORIG_Z_DIR_PIN 14
#define ORIG_Z_ENABLE_PIN 39
#define ORIG_Z_MIN_PIN 66
#define ORIG_Z_MAX_PIN 67

#define ORIG_E0_STEP_PIN 32
#define ORIG_E0_DIR_PIN 31
#define ORIG_E0_ENABLE_PIN 30

#define ORIG_E1_STEP_PIN 35
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 33

// =========================================
#define PiBot_Z_PROBE_PIN 68 // PiBot use this pin as Z-Probing pin

#define LED_PIN -1

// PWM6 fan1
#define ORIG_FAN_PIN 6
// PWM7 fan2
#define ORIG_FAN2_PIN 7

// have hardware in PiBot HDV2.0
#define ORIG_PS_ON_PIN 17
#define KILL_PIN -1

// PWM5 Extuder1
#define HEATER_0_PIN 5
// PWM4 Bed
#define HEATER_1_PIN 4
// PWM2 Extuder2
#define HEATER_2_PIN 2
// for Pibot   PWM Extuder3
#define HEATER_3_PIN -1
// for Pibot   PWM Extuder4
#define HEATER_4_PIN -1

#if Thermistor_Solution == 0 // 000    0 2 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 2
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 0
// ANALOG NUMBERING  Extuder2
#define TEMP_2_PIN 4
#endif
#if Thermistor_Solution == 1 // 001  1 2 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 2
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 1
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 4
#endif
#if Thermistor_Solution == 2 // 010  0 3 4
#define TEMP_0_PIN 3
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 0
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 4
#endif
#if Thermistor_Solution == 3 // 011  1 3 4
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 3
#define TEMP_1_PIN 1
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 4
#endif
#if Thermistor_Solution == 4 // 100  0 2 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 2
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 0
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 5
#endif
#if Thermistor_Solution == 5 // 101 1 2 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 2
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 1
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 5
#endif
#if Thermistor_Solution == 6 // 110  0 3 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 3
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 0
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 5
#endif
#if Thermistor_Solution == 7 // 111  1 3 5
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 3
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 1
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 5
#endif
#if !defined(TEMP_0_PIN) || !defined(TEMP_1_PIN) || !defined(TEMP_2_PIN) || !defined(Thermistor_Solution)
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 2
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 0
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 4
#endif

// ad595 temp senser
#if AD595_TEMP_Senser == true
// ANALOG NUMBERING  Extuder1
#define TEMP_0_PIN 8
// ANALOG NUMBERING  Bed
#define TEMP_1_PIN 9
// ANALOG NUMBERING   Extuder2
#define TEMP_2_PIN 10
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
#define SDPOWER -1
#define ORIG_SDCARDDETECT 40
#define SDSS 53
#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

#endif // end PiBot Controller Rev 2.0

#endif // end  PiBot for Repetier
