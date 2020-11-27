#if MOTHERBOARD == 301
#define KNOWN_BOARD
/*****************************************************************
* RAMBo Pin Assignments
******************************************************************/

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 48
#define ORIG_X_MIN_PIN 12
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define ORIG_Y_STEP_PIN 36
#define ORIG_Y_DIR_PIN 49
#define ORIG_Y_MIN_PIN 11
#define ORIG_Y_MAX_PIN 23
#define ORIG_Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define ORIG_Z_STEP_PIN 35
#define ORIG_Z_DIR_PIN 47
#define ORIG_Z_MIN_PIN 10
#define ORIG_Z_MAX_PIN 30
#define ORIG_Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define HEATER_0_PIN 9
#define TEMP_0_PIN 0

#define HEATER_1_PIN 3
// This is T2 on the board!
#define TEMP_1_PIN 2

#define HEATER_2_PIN 7
// This is T1 on the board!
#define TEMP_2_PIN 1
// T3 on board
#define TEMP_3_PIN 7

#define ORIG_E0_STEP_PIN 34
#define ORIG_E0_DIR_PIN 43
#define ORIG_E0_ENABLE_PIN 26
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66

#define ORIG_E1_STEP_PIN 33
#define ORIG_E1_DIR_PIN 42
#define ORIG_E1_ENABLE_PIN 25
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64

#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS \
    { 4, 5, 3, 0, 1 } // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER -1
#define SDSS 53
#define LED_PIN 13
#define ORIG_FAN_PIN 8
#define ORIG_FAN2_PIN 6
#define ORIG_FAN3_PIN 2
#define ORIG_PS_ON_PIN 4
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN, E0_MS1_PIN, E0_MS2_PIN,
#define E1_PINS

#define SCK_PIN 52
#define MISO_PIN 50
#define MOSI_PIN 51
#define MAX6675_SS 53
// #define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_DIGIPOT

#endif
