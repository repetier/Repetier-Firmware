// Shasta - dua based printer board used in 3ntr printers
// Info: The board uses pins not available on stock Arduino Due therefore
// it doe snot work 100% with due board. Use Stacker 3d board instead
// The variant is part of the firmware distribution and just needs to be added.

#if MOTHERBOARD == 416
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
//1
#define ORIG_X_STEP_PIN 18
#define ORIG_X_DIR_PIN 19
#define ORIG_X_MIN_PIN 31
#define ORIG_X_MAX_PIN 30
#define ORIG_X_ENABLE_PIN 22
//2
#define ORIG_Y_STEP_PIN 23
#define ORIG_Y_DIR_PIN 16
#define ORIG_Y_MIN_PIN 12
#define ORIG_Y_MAX_PIN 11
#define ORIG_Y_ENABLE_PIN 17
//3
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

//4
#define ORIG_E0_STEP_PIN 28
#define ORIG_E0_DIR_PIN 27
#define ORIG_E0_ENABLE_PIN 26
//5
#define ORIG_E1_STEP_PIN 29
#define ORIG_E1_DIR_PIN 15
#define ORIG_E1_ENABLE_PIN 14
//6
#define ORIG_E2_STEP_PIN 68
#define ORIG_E2_DIR_PIN 69
#define ORIG_E2_ENABLE_PIN 30
//7
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
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.
#define ENC424_SS 61

// 20 or 70
#define SDA_PIN 70
//21 or 71
#define SCL_PIN 71

// Servo pins: 5,6 und 39

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN, TEMP_0_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN, TEMP_2_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN, TEMP_3_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN, TEMP_4_PIN,

#define TWI_CLOCK_FREQ 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
// Ultronics has no eeprom for storing changeable data
// as a solution you can use sd card. But this requires always
// the same sd card when powering up the printer
//#define EEPROM_AVAILABLE EEPROM_NONE
#define EEPROM_AVAILABLE EEPROM_SDCARD

// Use second I2C by default
#ifndef WIRE_PORT
#define WIRE_PORT Wire1
#endif
#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 2
#endif

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