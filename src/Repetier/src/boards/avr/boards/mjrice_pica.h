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
#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

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
#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,

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
