/*
The Stacker3D SuperMini board is used for the Stacker3d F1 printers.
The differ depending on the printer type so not all pins/drivers may be present
in your model. That is no error - it is by design. But we define here
all pins possible.

Used timers for pin sused typically with hardware pwm:
Timer 1 (PE13, PE14), 2 (PB8, PB9), 3 (PB0, PC6, PC7), 4 (PD12), 9 (PE5, PE6)

*/

#pragma once

#ifndef STM32F4
#error "Oops! Select RUMBA32 in platformio.ini -> default_envs"
#endif

// Users expect Serial to be usb port!
#undef Serial
#define Serial SerialUSB

#define CPU_ARCH ARCH_ARM
#define MAX_RAM 131072 // Rumba32

// Default is 9 but that is used by fan pins, so map servo to timer 13
#define SERVO_TIMER_NUM 13

// Steppers
#define ORIG_X_STEP_PIN PD2
#define ORIG_X_DIR_PIN PD3
#define ORIG_X_ENABLE_PIN PD0
#define ORIG_X_CS_PIN PD1
#define ORIG_X_DIAG0_PIN PD4
#define ORIG_X_MIN_PIN PB12
#define ORIG_X_MAX_PIN -1

#define ORIG_Y_STEP_PIN PE2
#define ORIG_Y_DIR_PIN PE3
#define ORIG_Y_ENABLE_PIN PE0
#define ORIG_Y_CS_PIN PE1
#define ORIG_Y_DIAG0_PIN PE4
#define ORIG_Y_MIN_PIN PB14
#define ORIG_Y_MAX_PIN -1

#define ORIG_Y2_STEP_PIN PF13
#define ORIG_Y2_DIR_PIN PF14
#define ORIG_Y2_ENABLE_PIN PF11
#define ORIG_Y2_CS_PIN PF12
#define ORIG_Y2_DIAG0 PF15
#define ORIG_Y2_MIN_PIN -1
#define ORIG_Y2_MAX_PIN -1

#define ORIG_Z_STEP_PIN PE9
#define ORIG_Z_DIR_PIN PE10
#define ORIG_Z_ENABLE_PIN PE7
#define ORIG_Z_CS_PIN PE8
#define ORIG_Z_DIAG0_PIN PE11
#define ORIG_Z_MIN_PIN PB15
#define ORIG_Z_MAX_PIN PB2

#define ORIG_Z2_STEP_PIN PG2
#define ORIG_Z2_DIR_PIN PG3
#define ORIG_Z2_ENABLE_PIN PG0
#define ORIG_Z2_CS_PIN PG1
#define ORIG_Z2_DIAG0_PIN PG4
#define ORIG_Z2_MIN_PIN -1
#define ORIG_Z2_MAX_PIN PB1

#define ORIG_A_STEP_PIN PD7
#define ORIG_A_DIR_PIN PD8
#define ORIG_A_ENABLE_PIN PD6
#define ORIG_A_CS_PIN PD5
#define ORIG_A_DIAG0_PIN PD9
#define ORIG_A_MIN_PIN -1
#define ORIG_A_MAX_PIN PB13 // used as max pin, in data sheet it is a min

#define ORIG_E0_STEP_PIN PG7
#define ORIG_E0_DIR_PIN PG8
#define ORIG_E0_ENABLE_PIN PG5
#define ORIG_E0_CS_PIN PG6
#define ORIG_E0_DIAG0_PIN PG9

#define ORIG_E1_STEP_PIN PG12
#define ORIG_E1_DIR_PIN PG13
#define ORIG_E1_ENABLE_PIN PG10
#define ORIG_E1_CS_PIN PG11
#define ORIG_E1_DIAG0_PIN PG14

// Temperature Sensors
#define THERMOCOUPLE_0_PIN static_cast<int>(PA_0)
#define THERMOCOUPLE_1_PIN static_cast<int>(PA_1)
#define THERMOCOUPLE_2_PIN static_cast<int>(PA_2)

#define FILAMENT_SENSOR0 PE12
#define FILAMENT_SENSOR1 PE15

// Heaters / Fans
#define HEATER_0_PIN PE5  // E0 Timer 9 Channel 1
#define HEATER_2_PIN PE6  // E1 Timer 9 Channel 2
#define HEATER_1_PIN PD12 // bed Timer 4 Channel 1

// Note: In schematic fan0 goes to fan1 and fan1 to fan0. We use numbering printed on board.
#define ORIG_FAN_PIN PE13  // Timer 1 Channel 3
#define ORIG_FAN2_PIN PE14 // Timer 1 Channel 4
#define ORIG_FAN3_PIN PB8  // Timer 2 Channel 1
#define ORIG_FAN4_PIN PB9  // Timer 2 Channel 2

#ifndef TWI_CLOCK_FREQ
#define TWI_CLOCK_FREQ 400000
#endif

// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR 0x50  // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE 64      // page write buffer size
#define EEPROM_PAGE_WRITE_TIME 7 // page write time in milliseconds (docs say 5ms but that is too short)
#define FLASH_START 0x08000000ul
#define FLASH_SIZE 0x0080000ul // flash size excluding bootloader
#ifndef FLASH_EEPROM_SIZE
#define FLASH_EEPROM_SIZE 0x20000ul // use 128kb flash to prevent early destruction of flash but leave room for firmware
#define FLASH_SECTOR 7              // sector 7 is last 128kb
#endif
#ifndef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_FLASH
#endif

// I2C
#define SCK_PIN PA5
#define MISO_PIN PA6
#define MOSI_PIN PA7

//
// Misc. Functions
//
#define LED_PIN PB0  // blue - Timer 3 channel 3
#define LED_PIN2 PC6 // red - Timer 3 channel 1
#define LED_PIN3 PC7 // green - Timer 3 channel 2
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1

#ifndef SDSS
#define SDSS PC10
#endif
#define SDPOWER -1
#define ORIG_SDCARDDETECT

// LCD / Controller
#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#define UI_DISPLAY_RS_PIN PF2
#define UI_DISPLAY_RW_PIN -1
#define UI_DISPLAY_ENABLE_PIN PF0 // data
#define UI_DISPLAY_D4_PIN PF1     // sck
#define UI_DISPLAY_D5_PIN -1
#define UI_DISPLAY_D6_PIN -1
#define UI_DISPLAY_D7_PIN -1
#define UI_ENCODER_A PF3
#define UI_ENCODER_B PF4
#define UI_ENCODER_CLICK PF5
#ifndef BEEPER_PIN
#define BEEPER_PIN -1
#endif
#ifndef SDCARDDETECT
#define SDCARDDETECT PC9
#endif

#endif
#endif
