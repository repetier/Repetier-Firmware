#ifndef _pins_h_main
#define _pins_h_main

#define ARCH_AVR 1
#define ARCH_ARM 2
#define ARCH_DUE 2

#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master
#define CURRENT_CONTROL_ALLIGATOR 4  //Use External DAC like Alligator
#define CURRENT_CONTROL_MCP4728 5  // Use an i2c DAC as a digipot like PrintrBoard Rev. F

/*
  arm does not have a eeprom build in. Therefore boards can add a
  eeprom. Board definition must set the right type of eeprom
*/

#define EEPROM_NONE 0
#define EEPROM_I2C  1
#define EEPROM_SPI_ALLIGATOR 2
#define EEPROM_SDCARD 3

inline void memcopy2(void *dest,void *source) {
	*((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void *dest,void *source) {
	*((int32_t*)dest) = *((int32_t*)source);
}

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2       = 3
// RAMPS 1.3/RAMPS 1.4        = 33
// Azteeg X3                  = 34
// Azteeg X3 Pro              = 35
// MPX3  (mainly RAMPS compatible) = 38
// Ultimaker Shield 1.5.7     = 37
// Open Motion Controller     = 91
// Azteeg X1                  = 65
// 3Drag/Velleman K8200 (experimental) = 66
// Sethi 3D_1                 = 72
// Foltyn 3D Master           = 12
// MegaTronics 1.0            = 70
// Megatronics 2.0            = 701
// Megatronics 3.0            = 703 // Thermistors predefined not thermocouples
// Minitronics 1.0            = 702
// RUMBA                      = 80  // Get it from reprapdiscount
// FELIXprinters              = 101
// Rambo                      = 301
// PiBot for Repetier V1.0-1.3= 314
// PiBot for Repetier V1.4    = 315
// PiBot Controller V2.0      = 316
// Sanguish Beta              = 501
// Unique One rev. A          = 88
// SAV MK1                    = 89
// MJRice Pica Rev B          = 183
// MJRice Pica Rev C          = 184
// Zonestar ZRIB 2.1          = 39
// User layout defined in userpins.h = 999

#if MOTHERBOARD == 3 || MOTHERBOARD == 33 || MOTHERBOARD == 34 || MOTHERBOARD == 35 || MOTHERBOARD == 38 || \
MOTHERBOARD == 37 || MOTHERBOARD == 66 || MOTHERBOARD == 91 || MOTHERBOARD == 65 || MOTHERBOARD == 72 || \
MOTHERBOARD == 12 || MOTHERBOARD == 70 || MOTHERBOARD == 701 || MOTHERBOARD == 702 || MOTHERBOARD == 703 || \
MOTHERBOARD == 80 || MOTHERBOARD == 101 || MOTHERBOARD == 301 || MOTHERBOARD == 314 || MOTHERBOARD == 315 || \
MOTHERBOARD == 316 || MOTHERBOARD == 501 || MOTHERBOARD == 88 || MOTHERBOARD == 89 || MOTHERBOARD == 183 || \
MOTHERBOARD == 184 || MOTHERBOARD == 39

#define AVR_BOARD 1
#include "src/avr/pins.h"

// Due boards

// Arduino Due with RADDS       = 402
// Arduino Due with RAMPS-FD    = 403
// Arduino Due with RAMPS-FD V2 = 404
// Felix Printers for arm       = 405
// DAM&DICE DUE                 = 406
// Smart RAMPS for Due          = 408
// Smart RAMPS for Due with EEPROM = 413
// Ultratronics Board           = 409
// DUE3DOM                      = 410
// DUE3DOM MINI                 = 411
// STACKER 3D Superboard        = 412
// RURAMPS4D                    = 414
// Alligator Board rev1         = 500
// Alligator Board rev2         = 501
// Alligator Board rev3         = 502
// User defined due board       = 998

#elif MOTHERBOARD == 402 || MOTHERBOARD == 403 || MOTHERBOARD == 404 || MOTHERBOARD == 405 || MOTHERBOARD == 406 || \
MOTHERBOARD == 408 || MOTHERBOARD == 413 || MOTHERBOARD == 409 || MOTHERBOARD == 410 || MOTHERBOARD == 411 || \
MOTHERBOARD == 412 || MOTHERBOARD == 414 || MOTHERBOARD == 500 || MOTHERBOARD == 501 || MOTHERBOARD == 502|| MOTHERBOARD == 998

#define DUE_BOARD 1
#include "src/due/pins.h"

#else

#error Motherboard with set ID is not available!

#endif


#ifdef AVR_BOARD
#include "src/avr/HAL.h"
#endif
#ifdef DUE_BOARD
#include "src/due/HAL.h"
#endif

#include "src/communication/Communication.h"

#endif