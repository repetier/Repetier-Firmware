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
// #define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_ALLIGATOR
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
//A0
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

#define TWI_CLOCK_FREQ 0 // disabled since E2 uses pin 20/21! // 400000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32       // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 10 // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR //EEPROM_SPI_ALLIGATOR

#define WIRE_PORT Wire1
#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 2
#endif

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
// #define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_ALLIGATOR
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
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.

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

//64K SPI
#define SPI_CHAN_EEPROM1 2
// PD0
#define SPI_EEPROM1_CS 25

//2K SPI
// PD1
#define SPI_EEPROM2_CS 26

//** FLASH SPI**/
//32Mb
//PA14
#define SPI_FLASH_CS 23

#define TWI_CLOCK_FREQ 0
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50               // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32                   // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 10             // page write time in milliseconds (docs say 5ms but that is too short)
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR //EEPROM_SPI_ALLIGATOR

#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 0
#endif

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
// #define STEPPER_CURRENT_CONTROL CURRENT_CONTROL_DAC
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
#define TEMP_4_PIN -1   //PB12, analog pin on piggy

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

//92(orig) // PA5
#define ORIG_FAN_PIN 92
//31(orig) // PA7
#define ORIG_FAN2_PIN 31
#define ORIG_PS_ON_PIN -1
#define KILL_PIN ORIG_X_MIN_PIN
#define SUICIDE_PIN -1 //PIN that has to be turned on right after start, to keep power flowing.
#define HEAT_OFF_INT_PIN 50

#define SDA_PIN -1 // i2c not used
#define SCL_PIN -1 // i2c not used

//PC9
#define CASE_LIGHTS_PIN 41

#define E0_PINS ORIG_E0_STEP_PIN, ORIG_E0_DIR_PIN, ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN, ORIG_E1_DIR_PIN, ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN, ORIG_E2_DIR_PIN, ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN, ORIG_E3_DIR_PIN, ORIG_E3_ENABLE_PIN,

//** DAC for motor vfref current
#define DAC0_SYNC 53 // PB14
#define DAC1_SYNC 53 // PB14

//** EEPROM **

//64K SPI
#define SPI_CHAN_EEPROM1 2
#define SPI_EEPROM1_CS 25 // PD0

//2K SPI
#define SPI_EEPROM2_CS 26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS 23 //PA14

#define TWI_CLOCK_FREQ 0
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 32       // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 10 // page write time in milliseconds (docs say 5ms but that is too short)
#define EEPROM_AVAILABLE EEPROM_SPI_ALLIGATOR

#ifndef MAX_WIRE_INTERFACES
#define MAX_WIRE_INTERFACES 0
#endif

#endif
