/*
	This code contibuted by Triffid_Hunter and modified by Kliment
	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
        2012/3/10 AT90USB128x modified by lincomatic to match Teensyduino
*/
#ifndef	_FASTIO_H
#define	_FASTIO_H

#include <sam.h>
#include <sam3xa/include/pio/pio_sam3x8h.h>

// 0 .. 53 - Digital pins
  // ----------------------
  // 0/1 - UART (Serial)
#define DIO0_PORT PIOA
#define DIO0_PIN PIO_PA8A_URXD  // URXD

#define DIO1_PORT PIOA
#define DIO1_PIN PIO_PA9A_UTXD // UTXD

  // 2
#define DIO2_PORT  PIOB
#define DIO2_PIN PIO_PB25B_TIOA0 // TIOA0
#define DIO3_PORT PIOC
#define DIO3_PIN PIO_PC28B_TIOA7 // TIOA7
#define DIO4_PORT PIOC
#define DIO4_PIN PIO_PC26B_TIOB6 // TIOB6

  // 5
#define DIO5_PORT  PIOC
#define DIO5_PIN PIO_PC25B_TIOA6 // TIOA6
#define DIO6_PORT PIOC
#define DIO6_PIN PIO_PC24B_PWML7 // PWML7
#define DIO7_PORT PIOC
#define DIO7_PIN PIO_PC23B_PWML6 // PWML6
#define DIO8_PORT PIOC
#define DIO8_PIN PIO_PC22B_PWML5 // PWML5
#define DIO9_PORT PIOC
#define DIO9_PIN PIO_PC21B_PWML4 // PWML4
  // 10
#define DIO10_PORT PIOC
#define DIO10_PIN PIO_PC29B_TIOB7 // TIOB7
#define DIO11_PORT PIOD
#define DIO11_PIN PIO_PD7B_TIOA8 // TIOA8
#define DIO12_PORT PIOD
#define DIO12_PIN PIO_PD8B_TIOB8 // TIOB8

  // 13 - AMBER LED
#define DIO13_PORT PIOB
#define DIO13_PIN PIO_PB27B_TIOB0 // TIOB0

  // 14/15 - USART3 (Serial3)
#define DIO14_PORT PIOD
#define DIO14_PIN PIO_PD4B_TXD3 // TXD3
#define DIO15_PORT PIOD
#define DIO15_PIN PIO_PD5B_RXD3 // RXD3

  // 16/17 - USART1 (Serial2)
#define DIO16_PORT PIOA
#define DIO16_PIN PIO_PA13A_TXD1 // TXD1
#define DIO17_PORT PIOA
#define DIO17_PIN PIO_PA12A_RXD1 // RXD1

  // 18/19 - USART0 (Serial1)
#define DIO18_PORT PIOA
#define DIO18_PIN PIO_PA11A_TXD0 // TXD0
#define DIO19_PORT PIOA
#define DIO19_PIN PIO_PA10A_RXD0 // RXD0

  // 20/21 - TWI1
#define DIO20_PORT PIOB
#define DIO20_PIN PIO_PB12A_TWD1 // TWD1 - SDA0
#define DIO21_PORT PIOB
#define DIO21_PIN PIO_PB13A_TWCK1 // TWCK1 - SCL0

  // 22
#define DIO22_PORT PIOB
#define DIO22_PIN PIO_PB26 // PIN 22
#define DIO23_PORT PIOA
#define DIO23_PIN PIO_PA14 // PIN 23
#define DIO24_PORT PIOA
#define DIO24_PIN PIO_PA15 // PIN 24
#define DIO25_PORT PIOD
#define DIO25_PIN PIO_PD0 // PIN 25

  // 26
#define DIO26_PORT PIOD
#define DIO26_PIN PIO_PD1 // PIN 26
#define DIO27_PORT PIOD
#define DIO27_PIN PIO_PD2 // PIN 27
#define DIO28_PORT PIOD
#define DIO28_PIN PIO_PD3 // PIN 28
#define DIO29_PORT PIOD
#define DIO29_PIN PIO_PD6 // PIN 29

  // 30
#define DIO30_PORT PIOD
#define DIO30_PIN PIO_PD9 // PIN 30
#define DIO31_PORT PIOA
#define DIO31_PIN PIO_PA7 // PIN 31
#define DIO32_PORT PIOD
#define DIO32_PIN PIO_PD10 // PIN 32
#define DIO33_PORT PIOC
#define DIO33_PIN PIO_PC1 // PIN 33

  // 34
#define DIO34_PORT PIOC
#define DIO34_PIN PIO_PC2 // PIN 34
#define DIO35_PORT PIOC
#define DIO35_PIN PIO_PC3 // PIN 35
#define DIO36_PORT PIOC
#define DIO36_PIN PIO_PC4 // PIN 36
#define DIO37_PORT PIOC
#define DIO37_PIN PIO_PC5 // PIN 37

  // 38
#define DIO38_PORT PIOC
#define DIO38_PIN PIO_PC6 // PIN 38
#define DIO39_PORT PIOC
#define DIO39_PIN PIO_PC7 // PIN 39
#define DIO40_PORT PIOC
#define DIO40_PIN PIO_PC8 // PIN 40
#define DIO41_PORT PIOC
#define DIO41_PIN PIO_PC9 // PIN 41

  // 42
#define DIO42_PORT PIOA
#define DIO42_PIN PIO_PA19 // PIN 42
#define DIO43_PORT PIOA
#define DIO43_PIN PIO_PA20 // PIN 43
#define DIO44_PORT PIOC
#define DIO44_PIN PIO_PC19 // PIN 44
#define DIO45_PORT PIOC
#define DIO45_PIN PIO_PC18 // PIN 45

  // 46
#define DIO46_PORT PIOC
#define DIO46_PIN PIO_PC17 // PIN 46
#define DIO47_PORT PIOC
#define DIO47_PIN PIO_PC16 // PIN 47
#define DIO48_PORT PIOC
#define DIO48_PIN PIO_PC15 // PIN 48
#define DIO49_PORT PIOC
#define DIO49_PIN PIO_PC14 // PIN 49

  // 50
#define DIO50_PORT PIOC
#define DIO50_PIN PIO_PC13 // PIN 50
#define DIO51_PORT PIOC
#define DIO51_PIN PIO_PC12 // PIN 51
#define DIO52_PORT PIOB
#define DIO52_PIN PIO_PB21 // PIN 52
#define DIO53_PORT PIOB
#define DIO53_PIN PIO_PB14 // PIN 53


  // 54 .. 65 - Analog pins
  // ----------------------
#define DIO54_PORT PIOA
#define DIO54_PIN PIO_PA16X1_AD7 // AD0
#define DIO55_PORT PIOA
#define DIO55_PIN PIO_PA24X1_AD6 // AD1
#define DIO56_PORT PIOA
#define DIO56_PIN PIO_PA23X1_AD5 // AD2
#define DIO57_PORT PIOA
#define DIO57_PIN PIO_PA22X1_AD4 // AD3
  // 58
#define DIO58_PORT PIOA
#define DIO58_PIN PIO_PA6X1_AD3 // AD4
#define DIO59_PORT PIOA
#define DIO59_PIN PIO_PA4X1_AD2 // AD5
#define DIO60_PORT PIOA
#define DIO60_PIN PIO_PA3X1_AD1 // AD6
#define DIO61_PORT PIOA
#define DIO61_PIN PIO_PA2X1_AD0 // AD7
  // 62
#define DIO62_PORT PIOB
#define DIO62_PIN PIO_PB17X1_AD10 // AD8
#define DIO63_PORT PIOB
#define DIO63_PIN PIO_PB18X1_AD11 // AD9
#define DIO64_PORT PIOB
#define DIO64_PIN PIO_PB19X1_AD12 // AD10
#define DIO65_PORT PIOB
#define DIO65_PIN PIO_PB20X1_AD13 // AD11

  // 66/67 - DAC0/DAC1
#define DIO66_PORT PIOB
#define DIO66_PIN PIO_PB15X1_DAC0 // DAC0
#define DIO67_PORT PIOB
#define DIO67_PIN PIO_PB16X1_DAC1 // DAC1

  // 68/69 - CANRX0/CANTX0
#define DIO68_PORT PIOA
#define DIO68_PIN PIO_PA1A_CANRX0 // CANRX
#define DIO69_PORT PIOA
#define DIO69_PIN PIO_PA0A_CANTX0 // CANTX

  // 70/71 - TWI0
#define DIO70_PORT PIOA
#define DIO70_PIN PIO_PA17A_TWD0 // TWD0 - SDA1
#define DIO71_PORT PIOA
#define DIO71_PIN PIO_PA18A_TWCK0 // TWCK0 - SCL1

  // 72/73 - LEDs
#define DIO72_PORT PIOC
#define DIO72_PIN PIO_PC30 // LED AMBER RXL
#define DIO73_PORT PIOA
#define DIO73_PIN PIO_PA21 // LED AMBER TXL

  // 74/75/76 - SPI
#define DIO74_PORT PIOA
#define DIO74_PIN PIO_PA25A_SPI0_MISO // MISO
#define DIO75_PORT PIOA
#define DIO75_PIN PIO_PA26A_SPI0_MOSI // MOSI
#define DIO76_PORT PIOA
#define DIO76_PIN PIO_PA27A_SPI0_SPCK // SPCK

  // 77 - SPI CS0
#define DIO77_PORT PIOA
#define DIO77_PIN PIO_PA28A_SPI0_NPCS0 // NPCS0

  // 78 - SPI CS3 (unconnected)
#define DIO78_PORT PIOB
#define DIO78_PIN PIO_PB23B_SPI0_NPCS3 // NPCS3

  // 79 .. 84 - "All pins" masks

  // 79 - TWI0 all pins
#define DIO79_PORT PIOA
#define DIO79_PIN PIO_PA17A_TWD0|PIO_PA18A_TWCK0
 // 80 - TWI1 all pins
#define DI80_PORT PIOB
#define DIO80_PIN PIO_PB12A_TWD1|PIO_PB13A_TWCK1
  // 81 - UART (Serial) all pins
#define DIO81_PORT PIOA
#define DIO81_PIN PIO_PA8A_URXD|PIO_PA9A_UTXD
  // 82 - USART0 (Serial1) all pins
#define DIO82_PORT PIOA
#define DIO82_PIN PIO_PA11A_TXD0|PIO_PA10A_RXD0
  // 83 - USART1 (Serial2) all pins
#define DIO83_PORT PIOA
#define DIO83_PIN PIO_PA13A_TXD1|PIO_PA12A_RXD1
  // 84 - USART3 (Serial3) all pins
#define DIO84_PORT PIOD
#define DIO84_PIN PIO_PD4B_TXD3|PIO_PD5B_RXD3

  // 85 - USB
#define DIO85_PORT PIOB
#define DIO85_PIN PIO_PB11A_UOTGID|PIO_PB10A_UOTGVBOF // ID - VBOF

  // 86 - SPI CS2
#define DIO86_PORT PIOB
#define DIO86_PIN PIO_PB21B_SPI0_NPCS2 // NPCS2

  // 87 - SPI CS1
#define DIO87_PORT PIOA
#define DIO87_PIN PIO_PA29A_SPI0_NPCS1 // NPCS1

  // 88/89 - CANRX1/CANTX1 (same physical pin for 66/53)
#define DIO88_PORT PIOB
#define DIO88_PIN PIO_PB15A_CANRX1 // CANRX1
#define DIO89_PORT PIOB
#define DIO89_PIN PIO_PB14A_CANTX1 // CANTX1

  // 90 .. 91 - "All CAN pins" masks
  // 90 - CAN0 all pins
#define DIO90_PORT PIOA
#define DIO90_PIN PIO_PA1A_CANRX0|PIO_PA0A_CANTX0
  // 91 - CAN1 all pins
#define DIO91_PORT PIOB
#define DIO91_PIN PIO_PB15A_CANRX1|PIO_PB14A_CANTX1

// Additional Pins for Alligator board
#if (MOTHERBOARD == 500) || (MOTHERBOARD == 501)
//92
#define DIO92_PORT PIOA
#define DIO92_PIN PIO_PA5
//93
#define DIO93_PORT PIOB
#define DIO93_PIN PIO_PB12X1_AD8
//94
#define DIO94_PORT PIOB
#define DIO94_PIN PIO_PB22
//95
#define DIO95_PORT PIOB
#define DIO95_PIN PIO_PB23
//96
#define DIO96_PORT PIOB
#define DIO96_PIN PIO_PB24
//97
#define DIO97_PORT PIOC
#define DIO97_PIN PIO_PC20
//98
#define DIO98_PORT PIOC
#define DIO98_PIN PIO_PC27
//99
#define DIO99_PORT PIOC
#define DIO99_PIN PIO_PC10
//100
#define DIO100_PORT PIOC
#define DIO100_PIN PIO_PC11
#endif




#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please submit a pull request
#endif

#endif /* _FASTIO_H */

