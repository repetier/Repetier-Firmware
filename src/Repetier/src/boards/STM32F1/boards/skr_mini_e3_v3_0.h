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

#pragma once

#ifndef skr_mini_e3_v3_0
#error "Oops! Select skr_mini_e3_v3_0 in platformio.ini -> default_envs"
#endif
#define skr_mini_e3_v1_2 // Prevent error message in included file

// SKR E3 Mini V2 has a small onboard AT24C32 I2C EEPROM. Powered @ 3.3V.
#define TWI_CLOCK_FREQ 1000000

#undef EEPROM_AVAILABLE
#define EEPROM_AVAILABLE EEPROM_I2C

#define EEPROM_SERIAL_ADDR 0x50
#define EEPROM_PAGE_WRITE_TIME 10

/*
    SKR E3 Mini V2 - We use V1.2 as a base since it shares almost all the same pins.
    We just replace/add any new V2 pins here.
*/
#include "skr_mini_e3_v1_2.h"

#undef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32

#undef ORIG_E0_ENABLE_PIN
#define ORIG_E0_ENABLE_PIN PD1

#undef ORIG_FAN_PIN
#define ORIG_FAN_PIN PC6   // "FAN0"
#define ORIG_FAN1_PIN PC7  // "FAN1"
#define ORIG_FAN2_PIN PB15 // "FAN2"

#undef NEOPIXEL_PIN
#define NEOPIXEL_PIN PA8 // LED driving pin

#undef ORIG_PS_ON_PIN
#define ORIG_PS_ON_PIN PC13

#undef STATUS_LED_PIN
#define STATUS_LED_PIN PD8

#undef LED_PIN
#define LED_PIN STATUS_LED_PIN

#undef TEMP_1_PIN
#define TEMP_1_PIN PC4 // BED

#define FILAMENT_SENSOR0 PC15 // E0-STOP

// Default TMC slave addresses
#ifndef X_SLAVE_ADDRESS
#define X_SLAVE_ADDRESS 0
#endif
#ifndef Y_SLAVE_ADDRESS
#define Y_SLAVE_ADDRESS 2
#endif
#ifndef Z_SLAVE_ADDRESS
#define Z_SLAVE_ADDRESS 1
#endif
#ifndef E0_SLAVE_ADDRESS
#define E0_SLAVE_ADDRESS 3
#endif

#undef FLASH_START
#define FLASH_START 0x08002000ul

// LCD / Controller

/**
 *              SKR Mini E3 V3.0
 *                  ------
 *  (BEEPER)  PB5  |10  9 | PA15 (BTN_ENC)
 *  (BTN_EN1) PA9  | 8  7 | RESET
 *  (BTN_EN2) PA10   6  5 | PB9  (LCD_D4)
 *  (LCD_RS)  PB8  | 4  3 | PD6  (LCD_EN)
 *             GND | 2  1 | 5V
 *                  ------
 *                   EXP1
 */

#ifndef CUSTOM_CONTROLLER_PINS
#if FEATURE_CONTROLLER != CONTROLLER_NONE

#undef UI_ENCODER_CLICK
#define UI_ENCODER_CLICK PA15
#undef UI_DISPLAY_ENABLE_PIN
#define UI_DISPLAY_ENABLE_PIN PD6

#endif
#endif

#if 0


//#define BOARD_CUSTOM_BUILD_FLAGS -DTONE_CHANNEL=4 -DTONE_TIMER=4 -DTIMER_TONE=4

#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

// Onboard I2C EEPROM
#if EITHER(NO_EEPROM_SELECTED, I2C_EEPROM)
#undef NO_EEPROM_SELECTED
#define I2C_EEPROM
#define SOFT_I2C_EEPROM // Force the use of Software I2C
#define I2C_SCL_PIN PB6
#define I2C_SDA_PIN PB7
#define MARLIN_EEPROM_SIZE 0x1000 // 4K
#endif


//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN PC14 // PROBE

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
#define FIL_RUNOUT_PIN PC15 // E0-STOP
#endif

//
// Power-loss Detection
//
#ifndef POWER_LOSS_PIN
#define POWER_LOSS_PIN PC12 // Power Loss Detection: PWR-DET
#endif

#ifndef PS_ON_PIN
#define PS_ON_PIN PC13 // Power Supply Control
#endif

#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
   * Hardware serial communication ports
   */
#define X_HARDWARE_SERIAL MSerial4
#define Y_HARDWARE_SERIAL MSerial4
#define Z_HARDWARE_SERIAL MSerial4
#define E0_HARDWARE_SERIAL MSerial4

// Default TMC slave addresses
#ifndef X_SLAVE_ADDRESS
#define X_SLAVE_ADDRESS 0
#endif
#ifndef Y_SLAVE_ADDRESS
#define Y_SLAVE_ADDRESS 2
#endif
#ifndef Z_SLAVE_ADDRESS
#define Z_SLAVE_ADDRESS 1
#endif
#ifndef E0_SLAVE_ADDRESS
#define E0_SLAVE_ADDRESS 3
#endif
#endif


/**
 *              SKR Mini E3 V3.0
 *                  ------
 *  (BEEPER)  PB5  |10  9 | PA15 (BTN_ENC)
 *  (BTN_EN1) PA9  | 8  7 | RESET
 *  (BTN_EN2) PA10   6  5 | PB9  (LCD_D4)
 *  (LCD_RS)  PB8  | 4  3 | PD6  (LCD_EN)
 *             GND | 2  1 | 5V
 *                  ------
 *                   EXP1
 */
#define EXP1_09_PIN PA15
#define EXP1_03_PIN PD6

#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  /**
   *        ------                ------                ------
   * (ENT) |10  9 | (BEEP)       |10  9 |              |10  9 |
   *  (RX) | 8  7 |         (RX) | 8  7 | (TX)      RX | 8  7 | TX
   *  (TX)   6  5 |        (ENT)   6  5 | (BEEP)   ENT | 6  5 | BEEP
   *   (B) | 4  3 | (A)      (B) | 4  3 | (A)        B | 4  3 | A
   *   GND | 2  1 | (VCC)    GND | 2  1 | VCC      GND | 2  1 | VCC
   *        ------                ------                ------
   *         EXP1                  DWIN               DWIN (plug)
   *
   * All pins are labeled as printed on DWIN PCB. Connect TX-TX, A-A and so on.
   */

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
#error "CAUTION! DWIN_CREALITY_LCD requires a custom cable, see diagram above this line. (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
#endif

#define BEEPER_PIN EXP1_09_PIN
#define BTN_EN1 EXP1_03_PIN
#define BTN_EN2 PB8
#define BTN_ENC PB5

#elif HAS_WIRED_LCD

#if ENABLED(CR10_STOCKDISPLAY)

#define BEEPER_PIN PB5
#define BTN_ENC EXP1_09_PIN

#define BTN_EN1 PA9
#define BTN_EN2 PA10

#define LCD_PINS_RS PB8
#define LCD_PINS_ENABLE EXP1_03_PIN
#define LCD_PINS_D4 PB9

#elif ENABLED(ZONESTAR_LCD) // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
#error "CAUTION! ZONESTAR_LCD requires wiring modifications. See 'pins_BTT_SKR_MINI_E3_common.h' for details. (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
#endif

#define LCD_PINS_RS PB9
#define LCD_PINS_ENABLE EXP1_09_PIN
#define LCD_PINS_D4 PB8
#define LCD_PINS_D5 PA10
#define LCD_PINS_D6 PA9
#define LCD_PINS_D7 PB5
#define ADC_KEYPAD_PIN PA1 // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

#elif EITHER(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

#define BTN_ENC EXP1_09_PIN
#define BTN_EN1 PA9
#define BTN_EN2 PA10

#define DOGLCD_CS PB8
#define DOGLCD_A0 PB9
#define DOGLCD_SCK PB5
#define DOGLCD_MOSI EXP1_03_PIN

#define FORCE_SOFT_SPI
#define LCD_BACKLIGHT_PIN -1

#elif IS_TFTGLCD_PANEL

#if ENABLED(TFTGLCD_PANEL_SPI)

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
#error "CAUTION! TFTGLCD_PANEL_SPI requires wiring modifications. See 'pins_BTT_SKR_MINI_E3_common.h' for details. (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
#endif

      /**
       * TFTGLCD_PANEL_SPI display pinout
       *
       *                 Board                               Display
       *                 ------                               ------
       * (BEEPER) PB6  |10  9 | PB5  (SD_DET)             5V |10  9 | GND
       *         RESET | 8  7 | PA9  (MOD_RESET)          -- | 8  7 | (SD_DET)
       *          PB9    6  5 | PA10 (SD_CS)         (MOSI)  | 6  5 | --
       *          PB7  | 4  3 | PB8  (LCD_CS)        (SD_CS) | 4  3 | (LCD_CS)
       *           GND | 2  1 | 5V                   (SCK)   | 2  1 | (MISO)
       *                 ------                               ------
       *                  EXP1                                 EXP1
       *
       * Needs custom cable:
       *
       *    Board             Display
       *
       *   EXP1-1 ----------- EXP1-10
       *   EXP1-2 ----------- EXP1-9
       *   SPI1-4 ----------- EXP1-6
       *   EXP1-4 ----------- FREE
       *   SPI1-3 ----------- EXP1-2
       *   EXP1-6 ----------- EXP1-4
       *   EXP1-7 ----------- FREE
       *   EXP1-8 ----------- EXP1-3
       *   SPI1-1 ----------- EXP1-1
       *  EXP1-10 ----------- EXP1-7
       */

#define TFTGLCD_CS PA9

#endif

#elif ENABLED(FYSETC_MINI_12864_2_1)

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
#error "CAUTION! FYSETC_MINI_12864_2_1 and clones require wiring modifications. See 'pins_BTT_SKR_MINI_E3_V3_0.h' for details. Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning"
#endif

      /**
       *
       *                 Board                               Display
       *                 ------                               ------
       *    (EN2)  PB5  |10  9 | PA15(BTN_ENC)            5V |10  9 | GND
       *  (LCD_CS) PA9  | 8  7 | RST (RESET)              -- | 8  7 | --
       *  (LCD_A0) PA10 |#6  5 | PB9 (EN1)            (DIN)  | 6  5#| (RESET)
       *  (LCD_SCK)PB8  | 4  3 | PD6 (MOSI)         (LCD_A0) | 4  3 | (LCD_CS)
       *            GND | 2  1 | 5V                (BTN_ENC) | 2  1 | --
       *                 ------                               ------
       *                  EXP1                                 EXP1
       *
       *                                                      ------
       *                                                  -- |10  9 | --
       *                   ---                       (RESET) | 8  7 | --
       *                  | 3 |                      (MOSI)  | 6  5#| (EN2)
       *                  | 2 | (DIN)                     -- | 4  3 | (EN1)
       *                  | 1 |                     (LCD_SCK)| 2  1 | --
       *                   ---                                ------
       *                Neopixel                               EXP2
       *
       * Needs custom cable. Connect EN2-EN2, LCD_CS-LCD_CS and so on.
       *
       * Check twice index position!!! (marked as # here)
       * On BTT boards pins from IDC10 connector are numbered in unusual order.
       */
#define BTN_ENC EXP1_09_PIN
#define BTN_EN1 PB9
#define BTN_EN2 PB5
#define BEEPER_PIN -1

#define DOGLCD_CS PA9
#define DOGLCD_A0 PA10
#define DOGLCD_SCK PB8
#define DOGLCD_MOSI PD6

#define FORCE_SOFT_SPI
#define LCD_BACKLIGHT_PIN -1

#else
#error "Only CR10_STOCKDISPLAY, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, FYSETC_MINI_12864_2_1, and TFTGLCD_PANEL_(SPI|I2C) are currently supported on the BIGTREE_SKR_MINI_E3."
#endif

#endif // HAS_WIRED_LCD

#if BOTH(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
#error "CAUTION! LCD_FYSETC_TFT81050 requires wiring modifications. See 'pins_BTT_SKR_MINI_E3_common.h' for details. (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
#endif

  /**
   * FYSETC TFT TFT81050 display pinout
   *
   *                   Board                             Display
   *                   ------                            ------
   * (SD_DET)    PB5  |10  9 | PB6 (BEEPER)          5V |10  9 | GND
   * (MOD_RESET) PA9  | 8  7 | RESET            (RESET) | 8  7 | (SD_DET)
   * (SD_CS)     PA10   6  5 | PB9 (FREE)       (MOSI)  | 6  5 | (LCD_CS)
   * (LCD_CS)    PB8  | 4  3 | PB7 (FREE)       (SD_CS) | 4  3 | (MOD_RESET)
   *               5V | 2  1 | GND              (SCK)   | 2  1 | (MISO)
   *                   ------                            ------
   *                    EXP1                              EXP1
   *
   * Needs custom cable:
   *
   *    Board   Adapter   Display
   *           _________
   *   EXP1-1 ----------- EXP1-10
   *   EXP1-2 ----------- EXP1-9
   *   SPI1-4 ----------- EXP1-6
   *   EXP1-4 ----------- EXP1-5
   *   SPI1-3 ----------- EXP1-2
   *   EXP1-6 ----------- EXP1-4
   *   EXP1-7 ----------- EXP1-8
   *   EXP1-8 ----------- EXP1-3
   *   SPI1-1 ----------- EXP1-1
   *  EXP1-10 ----------- EXP1-7
   */

#define CLCD_SPI_BUS 1 // SPI1 connector

#define BEEPER_PIN EXP1_09_PIN

#define CLCD_MOD_RESET PA9
#define CLCD_SPI_CS PB8

#endif // TOUCH_UI_FTDI_EVE && LCD_FYSETC_TFT81050

//
// SD Support
//

#ifndef SDCARD_CONNECTION
#define SDCARD_CONNECTION ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
#define SD_DETECT_PIN PC3
#elif SD_CONNECTION_IS(LCD) && (BOTH(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050) || IS_TFTGLCD_PANEL)
#define SD_DETECT_PIN PB5
#define SD_SS_PIN PA10
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
#error "SD CUSTOM_CABLE is not compatible with SKR Mini E3."
#endif

#define ONBOARD_SPI_DEVICE 1  // SPI1 -> used only by HAL/STM32F1...
#define ONBOARD_SD_CS_PIN PA4 // Chip select for "System" SD card

#define ENABLE_SPI1
#define SDSS ONBOARD_SD_CS_PIN
#define SD_SS_PIN ONBOARD_SD_CS_PIN
#define SD_SCK_PIN PA5
#define SD_MISO_PIN PA6
#define SD_MOSI_PIN PA7

#endif