/* Arduino SdFat Library
 * Copyright (C) 2012 by William Greiman
 *
 * This file is part of the Arduino SdFat Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdFat Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/* Arduino SdFat Library
 * Copyright (C) 2012 by William Greiman
 *
 * This file is part of the Arduino SdFat Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdFat Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
\mainpage Arduino %SdFat Library
<CENTER>Copyright &copy; 2012 by William Greiman
</CENTER>

\section Intro Introduction
The Arduino %SdFat Library is a minimal implementation of FAT16 and FAT32
file systems on SD flash memory cards. Standard SD and high capacity SDHC
cards are supported.

Experimental support for FAT12 can be enabled by setting FAT12_SUPPORT
nonzero in SdFatConfig.h.

The %SdFat library only supports short 8.3 names.

The main classes in %SdFat are SdFat, SdFile, \ref fstream, \ref ifstream,
and \ref ofstream.

The SdFat class maintains a volume working directories, a current working
directory, and simplifies initialization of other classes.

The SdFile class provides binary file access functions such as open(), read(),
remove(), write(), close() and sync(). This class supports access to the root
directory and subdirectories.

The \ref fstream class implements C++ iostreams for both reading and writing
text files.

The \ref ifstream class implements the C++ iostreams for reading text files.

The \ref ofstream class implements the C++ iostreams for writing text files.

The classes \ref ibufstream and \ref obufstream format and parse character
 strings in memory buffers.

the classes ArduinoInStream and ArduinoOutStream provide iostream functions
for Serial, LiquidCrystal, and other devices.

The SdVolume class supports FAT16 and FAT32 partitions.  Most applications
will not need to call SdVolume member function.

The Sd2Card class supports access to standard SD cards and SDHC cards.  Most
applications will not need to call Sd2Card functions.  The Sd2Card class can
be used for raw access to the SD card.

A number of example are provided in the %SdFat/examples folder.  These were
developed to test %SdFat and illustrate its use.

%SdFat was developed for high speed data recording.  %SdFat was used to
implement an audio record/play class, WaveRP, for the Adafruit Wave Shield.
This application uses special Sd2Card calls to write to contiguous files in
raw mode.  These functions reduce write latency so that audio can be
recorded with the small amount of RAM in the Arduino.

\section SDcard SD\SDHC Cards

Arduinos access SD cards using the cards SPI protocol.  PCs, Macs, and
most consumer devices use the 4-bit parallel SD protocol.  A card that
functions well on A PC or Mac may not work well on the Arduino.

Most cards have good SPI read performance but cards vary widely in SPI
write performance.  Write performance is limited by how efficiently the
card manages internal erase/remapping operations.  The Arduino cannot
optimize writes to reduce erase operations because of its limit RAM.

SanDisk cards generally have good write performance.  They seem to have
more internal RAM buffering than other cards and therefore can limit
the number of flash erase operations that the Arduino forces due to its
limited RAM.

\section Hardware Hardware Configuration

%SdFat was developed using an
<A HREF = "http://www.adafruit.com/"> Adafruit Industries</A>
<A HREF = "http://www.ladyada.net/make/waveshield/"> Wave Shield</A>.

The hardware interface to the SD card should not use a resistor based level
shifter.  %SdFat sets the SPI bus frequency to 8 MHz which results in signal
rise times that are too slow for the edge detectors in many newer SD card
controllers when resistor voltage dividers are used.

The 5 to 3.3 V level shifter for 5 V Arduinos should be IC based like the
74HC4050N based circuit shown in the file SdLevel.png.  The Adafruit Wave Shield
uses a 74AHC125N.  Gravitech sells SD and MicroSD Card Adapters based on the
74LCX245.

If you are using a resistor based level shifter and are having problems try
setting the SPI bus frequency to 4 MHz.  This can be done by using
card.init(SPI_HALF_SPEED) to initialize the SD card.

\section comment Bugs and Comments

If you wish to report bugs or have comments, send email to fat16lib@sbcglobal.net.

\section SdFatClass SdFat Usage

%SdFat uses a slightly restricted form of short names.
Only printable ASCII characters are supported. No characters with code point
values greater than 127 are allowed.  Space is not allowed even though space
was allowed in the API of early versions of DOS.

Short names are limited to 8 characters followed by an optional period (.)
and extension of up to 3 characters.  The characters may be any combination
of letters and digits.  The following special characters are also allowed:

$ % ' - _ @ ~ ` ! ( ) { } ^ # &

Short names are always converted to upper case and their original case
value is lost.

\note
  The Arduino Print class uses character
at a time writes so it was necessary to use a \link SdFile::sync() sync() \endlink
function to control when data is written to the SD card.

\par
An application which writes to a file using print(), println() or
\link SdFile::write write() \endlink must call \link SdFile::sync() sync() \endlink
at the appropriate time to force data and directory information to be written
to the SD Card.  Data and directory information are also written to the SD card
when \link SdFile::close() close() \endlink is called.

\par
Applications must use care calling \link SdFile::sync() sync() \endlink
since 2048 bytes of I/O is required to update file and
directory information.  This includes writing the current data block, reading
the block that contains the directory entry for update, writing the directory
block back and reading back the current data block.

It is possible to open a file with two or more instances of SdFile.  A file may
be corrupted if data is written to the file by more than one instance of SdFile.

\section HowTo How to format SD Cards as FAT Volumes

You should use a freshly formatted SD card for best performance.  FAT
file systems become slower if many files have been created and deleted.
This is because the directory entry for a deleted file is marked as deleted,
but is not deleted.  When a new file is created, these entries must be scanned
before creating the file, a flaw in the FAT design.  Also files can become
fragmented which causes reads and writes to be slower.

A formatter sketch, SdFormatter.pde, is included in the
%SdFat/examples/SdFormatter directory.  This sketch attempts to
emulate SD Association's SDFormatter.

The best way to restore an SD card's format on a PC is to use SDFormatter
which can be downloaded from:

http://www.sdcard.org/consumers/formatter/

SDFormatter aligns flash erase boundaries with file
system structures which reduces write latency and file system overhead.

SDFormatter does not have an option for FAT type so it may format
small cards as FAT12.

After the MBR is restored by SDFormatter you may need to reformat small
cards that have been formatted FAT12 to force the volume type to be FAT16.

If you reformat the SD card with an OS utility, choose a cluster size that
will result in:

4084 < CountOfClusters && CountOfClusters < 65525

The volume will then be FAT16.

If you are formatting an SD card on OS X or Linux, be sure to use the first
partition. Format this partition with a cluster count in above range for FAT16.
SDHC cards should be formatted FAT32 with a cluster size of 32 KB.

Microsoft operating systems support removable media formatted with a
Master Boot Record, MBR, or formatted as a super floppy with a FAT Boot Sector
in block zero.

Microsoft operating systems expect MBR formatted removable media
to have only one partition. The first partition should be used.

Microsoft operating systems do not support partitioning SD flash cards.
If you erase an SD card with a program like KillDisk, Most versions of
Windows will format the card as a super floppy.

\section  References References

Adafruit Industries:

http://www.adafruit.com/

http://www.ladyada.net/make/waveshield/

The Arduino site:

http://www.arduino.cc/

For more information about FAT file systems see:

http://www.microsoft.com/whdc/system/platform/firmware/fatgen.mspx

For information about using SD cards as SPI devices see:

http://www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf

The ATmega328 datasheet:

http://www.atmel.com/dyn/resources/prod_documents/doc8161.pdf


 */
#ifndef SdFat_h
#define SdFat_h
/**
 * \file
 * \brief SdFat class
 */
//------------------------------------------------------------------------------
/** SdFat version YYYYMMDD */
#define SD_FAT_VERSION 20130629
//------------------------------------------------------------------------------
/** error if old IDE */
#if !defined(ARDUINO) || ARDUINO < 100
#error Arduino IDE must be 1.0 or greater
#endif  // ARDUINO < 100
//------------------------------------------------------------------------------
#include <stdint.h>
// Based on the document:
//
// SD Specifications
// Part 1
// Physical Layer
// Simplified Specification
// Version 3.01
// May 18, 2010
//
// http://www.sdcard.org/developers/tech/sdcard/pls/simplified_specs
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
uint8_t const CMD0 = 0X00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
uint8_t const CMD8 = 0X08;
/** SEND_CSD - read the Card Specific Data (CSD register) */
uint8_t const CMD9 = 0X09;
/** SEND_CID - read the card identification information (CID register) */
uint8_t const CMD10 = 0X0A;
/** STOP_TRANSMISSION - end multiple block read sequence */
uint8_t const CMD12 = 0X0C;
/** SEND_STATUS - read the card status register */
uint8_t const CMD13 = 0X0D;
/** READ_SINGLE_BLOCK - read a single data block from the card */
uint8_t const CMD17 = 0X11;
/** READ_MULTIPLE_BLOCK - read a multiple data blocks from the card */
uint8_t const CMD18 = 0X12;
/** WRITE_BLOCK - write a single data block to the card */
uint8_t const CMD24 = 0X18;
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
uint8_t const CMD25 = 0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
uint8_t const CMD32 = 0X20;
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
uint8_t const CMD33 = 0X21;
/** ERASE - erase all previously selected blocks */
uint8_t const CMD38 = 0X26;
/** APP_CMD - escape for application specific command */
uint8_t const CMD55 = 0X37;
/** READ_OCR - read the OCR register of a card */
uint8_t const CMD58 = 0X3A;
/** CRC_ON_OFF - enable or disable CRC checking */
uint8_t const CMD59 = 0X3B;
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
uint8_t const ACMD23 = 0X17;
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
uint8_t const ACMD41 = 0X29;
//------------------------------------------------------------------------------
/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0X05;
//------------------------------------------------------------------------------
/** Card IDentification (CID) register */
typedef struct CID {
  // byte 0
  /** Manufacturer ID */
  unsigned char mid;
  // byte 1-2
  /** OEM/Application ID */
  char oid[2];
  // byte 3-7
  /** Product name */
  char pnm[5];
  // byte 8
  /** Product revision least significant digit */
  unsigned char prv_m : 4;
  /** Product revision most significant digit */
  unsigned char prv_n : 4;
  // byte 9-12
  /** Product serial number */
  uint32_t psn;
  // byte 13
  /** Manufacturing date year low digit */
  unsigned char mdt_year_high : 4;
  /** not used */
  unsigned char reserved : 4;
  // byte 14
  /** Manufacturing date month */
  unsigned char mdt_month : 4;
  /** Manufacturing date year low digit */
  unsigned char mdt_year_low :4;
  // byte 15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** CRC7 checksum */
  unsigned char crc : 7;
}__attribute__((packed)) cid_t;
//------------------------------------------------------------------------------
/** CSD for version 1.00 cards */
typedef struct CSDV1 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  unsigned char taac;
  // byte 2
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  unsigned char c_size_high : 2;
  unsigned char reserved2 : 2;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign :1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7
  unsigned char c_size_mid;
  // byte 8
  unsigned char vdd_r_curr_max : 3;
  unsigned char vdd_r_curr_min : 3;
  unsigned char c_size_low :2;
  // byte 9
  unsigned char c_size_mult_high : 2;
  unsigned char vdd_w_cur_max : 3;
  unsigned char vdd_w_curr_min : 3;
  // byte 10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char c_size_mult_low : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved3 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved4 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved5: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Indicates the file format on the card */
  unsigned char file_format_grp : 1;
  // byte 15
  unsigned char always1 : 1;
  unsigned char crc : 7;
}__attribute__((packed)) csd1_t;
//------------------------------------------------------------------------------
/** CSD for version 2.00 cards */
typedef struct CSDV2 {
  // byte 0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1
  /** fixed to 0X0E */
  unsigned char taac;
  // byte 2
  /** fixed to 0 */
  unsigned char nsac;
  // byte 3
  unsigned char tran_speed;
  // byte 4
  unsigned char ccc_high;
  // byte 5
  /** This field is fixed to 9h, which indicates READ_BL_LEN=512 Byte */
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6
  /** not used */
  unsigned char reserved2 : 4;
  unsigned char dsr_imp : 1;
  /** fixed to 0 */
  unsigned char read_blk_misalign :1;
  /** fixed to 0 */
  unsigned char write_blk_misalign : 1;
  /** fixed to 0 - no partial read */
  unsigned char read_bl_partial : 1;
  // byte 7
  /** high part of card size */
  unsigned char c_size_high : 6;
  /** not used */
  unsigned char reserved3 : 2;
  // byte 8
  /** middle part of card size */
  unsigned char c_size_mid;
  // byte 9
  /** low part of card size */
  unsigned char c_size_low;
  // byte 10
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_high : 6;
  /** fixed to 1 - erase single is supported */
  unsigned char erase_blk_en : 1;
  /** not used */
  unsigned char reserved4 : 1;
  // byte 11
  unsigned char wp_grp_size : 7;
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_low : 1;
  // byte 12
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_high : 2;
  /** fixed value of 2 */
  unsigned char r2w_factor : 3;
  /** not used */
  unsigned char reserved5 : 2;
  /** fixed value of 0 - no write protect groups */
  unsigned char wp_grp_enable : 1;
  // byte 13
  unsigned char reserved6 : 5;
  /** always zero - no partial block read*/
  unsigned char write_partial : 1;
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_low : 2;
  // byte 14
  unsigned char reserved7: 2;
  /** Do not use always 0 */
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Do not use always 0 */
  unsigned char file_format_grp : 1;
  // byte 15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** checksum */
  unsigned char crc : 7;
}__attribute__((packed)) csd2_t;
//------------------------------------------------------------------------------
/** union of old and new style CSD register */
union csd_t {
  csd1_t v1;
  csd2_t v2;
};

//------------------------------------------------------------------------------
/**
 * To enable SD card CRC checking set USE_SD_CRC nonzero.
 *
 * Set USE_SD_CRC to 1 to use a smaller slower CRC-CCITT function.
 *
 * Set USE_SD_CRC to 2 to used a larger faster table driven CRC-CCITT function.
 */
#define USE_SD_CRC 2
//------------------------------------------------------------------------------
/**
 * To use multiple SD cards set USE_MULTIPLE_CARDS nonzero.
 *
 * Using multiple cards costs 400 - 500  bytes of flash.
 *
 * Each card requires about 550 bytes of SRAM so use of a Mega is recommended.
 */
#define USE_MULTIPLE_CARDS 0
//------------------------------------------------------------------------------
/**
 * Set DESTRUCTOR_CLOSES_FILE nonzero to close a file in its destructor.
 *
 * Causes use of lots of heap in ARM.
 */
#define DESTRUCTOR_CLOSES_FILE 0
//------------------------------------------------------------------------------

/**
 * Set USE_SEPARATE_FAT_CACHE nonzero to use a second 512 byte cache
 * for FAT table entries.  Improves performance for large writes that
 * are not a multiple of 512 bytes.
 */
#ifdef __arm__
#define USE_SEPARATE_FAT_CACHE 1
#else  // __arm__
#define USE_SEPARATE_FAT_CACHE 0
#endif  // __arm__
//------------------------------------------------------------------------------
/**
 * Don't use mult-block read/write on small AVR boards
 */
#if defined(RAMEND) && (RAMEND < 3000 || (NONLINEAR_SYSTEM && RAMEND<8000))
#define USE_MULTI_BLOCK_SD_IO 0
#else
#define USE_MULTI_BLOCK_SD_IO 1
#endif
//------------------------------------------------------------------------------
/**
 *  Force use of Arduino Standard SPI library if USE_ARDUINO_SPI_LIBRARY
 * is nonzero.
 */
#define USE_ARDUINO_SPI_LIBRARY 0

//------------------------------------------------------------------------------

/**
 * Use native SPI on Teensy 3.0 if USE_NATIVE_MK20DX128-SPI is nonzero.
 */
#if defined(__arm__) && defined(CORE_TEENSY)
#define USE_NATIVE_MK20DX128_SPI 1
#else
#define USE_NATIVE_MK20DX128_SPI 0
#endif
//------------------------------------------------------------------------------
/**
 * Use fast SAM3X SPI library if USE_NATIVE_SAM3X_SPI is nonzero.
 */
#if defined(__arm__) && !defined(CORE_TEENSY)
#define USE_NATIVE_SAM3X_SPI 1
#else
#define USE_NATIVE_SAM3X_SPI 0
#endif
//------------------------------------------------------------------------------
/**
 * Set nonzero to use Serial (the HardwareSerial class) for error messages
 * and output from print functions like ls().
 *
 * If USE_SERIAL_FOR_STD_OUT is zero, a small non-interrupt driven class
 * is used to output messages to serial port zero.  This allows an alternate
 * Serial library like SerialPort to be used with SdFat.
 *
 * You can redirect stdOut with SdFat::setStdOut(Print* stream) and
 * get the current stream with SdFat::stdOut().
 */
#define USE_SERIAL_FOR_STD_OUT 0
//------------------------------------------------------------------------------
/**
 * Call flush for endl if ENDL_CALLS_FLUSH is nonzero
 *
 * The standard for iostreams is to call flush.  This is very costly for
 * SdFat.  Each call to flush causes 2048 bytes of I/O to the SD.
 *
 * SdFat has a single 512 byte buffer for SD I/O so it must write the current
 * data block to the SD, read the directory block from the SD, update the
 * directory entry, write the directory block to the SD and read the data
 * block back into the buffer.
 *
 * The SD flash memory controller is not designed for this many rewrites
 * so performance may be reduced by more than a factor of 100.
 *
 * If ENDL_CALLS_FLUSH is zero, you must call flush and/or close to force
 * all data to be written to the SD.
 */
#define ENDL_CALLS_FLUSH 0
//------------------------------------------------------------------------------
/**
 * Allow use of deprecated functions if ALLOW_DEPRECATED_FUNCTIONS is nonzero
 */
#define ALLOW_DEPRECATED_FUNCTIONS 0
//------------------------------------------------------------------------------
/**
 * Allow FAT12 volumes if FAT12_SUPPORT is nonzero.
 * FAT12 has not been well tested.
 */
#define FAT12_SUPPORT 0
//------------------------------------------------------------------------------
/**
 * SPI init rate for SD initialization commands. Must be 10 (F_CPU/64)
 * or greater
 */
#define SPI_SD_INIT_RATE 11
//------------------------------------------------------------------------------
/**
 * Set the SS pin high for hardware SPI.  If SS is chip select for another SPI
 * device this will disable that device during the SD init phase.
 */
#define SET_SPI_SS_HIGH 1
//------------------------------------------------------------------------------
/**
 * Define MEGA_SOFT_SPI nonzero to use software SPI on Mega Arduinos.
 * Default pins used are SS 10, MOSI 11, MISO 12, and SCK 13.
 * Edit Software Spi pins to change pin numbers.
 *
 * MEGA_SOFT_SPI allows an unmodified Adafruit GPS Shield to be used
 * on Mega Arduinos.  Software SPI works well with GPS Shield V1.1
 * but many SD cards will fail with GPS Shield V1.0.
 */
#define MEGA_SOFT_SPI 0
//------------------------------------------------------------------------------
/**
 * Define LEONARDO_SOFT_SPI nonzero to use software SPI on Leonardo Arduinos.
 * Derfault pins used are SS 10, MOSI 11, MISO 12, and SCK 13.
 * Edit Software Spi pins to change pin numbers.
 *
 * LEONARDO_SOFT_SPI allows an unmodified Adafruit GPS Shield to be used
 * on Leonardo Arduinos.  Software SPI works well with GPS Shield V1.1
 * but many SD cards will fail with GPS Shield V1.0.
 */
#define LEONARDO_SOFT_SPI 0
//------------------------------------------------------------------------------
/**
 * Set USE_SOFTWARE_SPI nonzero to always use software SPI.
 */
#define USE_SOFTWARE_SPI 0
// define software SPI pins so Mega can use unmodified 168/328 shields
/** Default Software SPI chip select pin */
uint8_t const SOFT_SPI_CS_PIN = 10;
/** Software SPI Master Out Slave In pin */
uint8_t const SOFT_SPI_MOSI_PIN = 11;
/** Software SPI Master In Slave Out pin */
uint8_t const SOFT_SPI_MISO_PIN = 12;
/** Software SPI Clock pin */
uint8_t const SOFT_SPI_SCK_PIN = 13;
//------------------------------------------------------------------------------
// SPI speed is F_CPU/2^(1 + index), 0 <= index <= 6
/** Set SCK to max rate of F_CPU/2. See Sd2Card::setSckRate(). */
uint8_t const SPI_FULL_SPEED = 0;
/** Set SCK rate to F_CPU/3 for Due */
uint8_t const SPI_DIV3_SPEED = 1;
/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate(). */
uint8_t const SPI_HALF_SPEED = 2;
/** Set SCK rate to F_CPU/6 for Due */
uint8_t const SPI_DIV6_SPEED = 3;
/** Set SCK rate to F_CPU/8. See Sd2Card::setSckRate(). */
uint8_t const SPI_QUARTER_SPEED = 4;
/** Set SCK rate to F_CPU/16. See Sd2Card::setSckRate(). */
uint8_t const SPI_EIGHTH_SPEED = 6;
/** Set SCK rate to F_CPU/32. See Sd2Card::setSckRate(). */
uint8_t const SPI_SIXTEENTH_SPEED = 8;
/** MAX rate test - see spiInit for a given chip for details */
const uint8_t MAX_SCK_RATE_ID = 14;
//------------------------------------------------------------------------------
/** init timeout ms */
uint16_t const SD_INIT_TIMEOUT = 2000;
/** erase timeout ms */
uint16_t const SD_ERASE_TIMEOUT = 10000;
/** read timeout ms */
uint16_t const SD_READ_TIMEOUT = 300;
/** write time out ms */
uint16_t const SD_WRITE_TIMEOUT = 600;
//------------------------------------------------------------------------------
// SD card errors
/** timeout error for command CMD0 (initialize card in SPI mode) */
uint8_t const SD_CARD_ERROR_CMD0 = 0X1;
/** CMD8 was not accepted - not a valid SD card*/
uint8_t const SD_CARD_ERROR_CMD8 = 0X2;
/** card returned an error response for CMD12 (write stop) */
uint8_t const SD_CARD_ERROR_CMD12 = 0X3;
/** card returned an error response for CMD17 (read block) */
uint8_t const SD_CARD_ERROR_CMD17 = 0X4;
/** card returned an error response for CMD18 (read multiple block) */
uint8_t const SD_CARD_ERROR_CMD18 = 0X5;
/** card returned an error response for CMD24 (write block) */
uint8_t const SD_CARD_ERROR_CMD24 = 0X6;
/**  WRITE_MULTIPLE_BLOCKS command failed */
uint8_t const SD_CARD_ERROR_CMD25 = 0X7;
/** card returned an error response for CMD58 (read OCR) */
uint8_t const SD_CARD_ERROR_CMD58 = 0X8;
/** SET_WR_BLK_ERASE_COUNT failed */
uint8_t const SD_CARD_ERROR_ACMD23 = 0X9;
/** ACMD41 initialization process timeout */
uint8_t const SD_CARD_ERROR_ACMD41 = 0XA;
/** card returned a bad CSR version field */
uint8_t const SD_CARD_ERROR_BAD_CSD = 0XB;
/** erase block group command failed */
uint8_t const SD_CARD_ERROR_ERASE = 0XC;
/** card not capable of single block erase */
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0XD;
/** Erase sequence timed out */
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0XE;
/** card returned an error token instead of read data */
uint8_t const SD_CARD_ERROR_READ = 0XF;
/** read CID or CSD failed */
uint8_t const SD_CARD_ERROR_READ_REG = 0X10;
/** timeout while waiting for start of read data */
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X11;
/** card did not accept STOP_TRAN_TOKEN */
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X12;
/** card returned an error token as a response to a write operation */
uint8_t const SD_CARD_ERROR_WRITE = 0X13;
/** attempt to write protected block zero */
uint8_t const SD_CARD_ERROR_WRITE_BLOCK_ZERO = 0X14;  // REMOVE - not used
/** card did not go ready for a multiple block write */
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X15;
/** card returned an error to a CMD13 status check after a write */
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X16;
/** timeout occurred during write programming */
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X17;
/** incorrect rate selected */
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X18;
/** init() not called */
uint8_t const SD_CARD_ERROR_INIT_NOT_CALLED = 0X19;
/** card returned an error for CMD59 (CRC_ON_OFF) */
uint8_t const SD_CARD_ERROR_CMD59 = 0X1A;
/** invalid read CRC */
uint8_t const SD_CARD_ERROR_READ_CRC = 0X1B;
/** SPI DMA error */
uint8_t const SD_CARD_ERROR_SPI_DMA = 0X1C;
//------------------------------------------------------------------------------
// card types
/** Standard capacity V1 SD card */
uint8_t const SD_CARD_TYPE_SD1  = 1;
/** Standard capacity V2 SD card */
uint8_t const SD_CARD_TYPE_SD2  = 2;
/** High Capacity SD card */
uint8_t const SD_CARD_TYPE_SDHC = 3;
/**
 * define SOFTWARE_SPI to use bit-bang SPI
 */
//------------------------------------------------------------------------------
#if LEONARDO_SOFT_SPI && defined(__AVR_ATmega32U4__) && !defined(CORE_TEENSY)
#define SOFTWARE_SPI
#elif MEGA_SOFT_SPI&&(defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__))
#define SOFTWARE_SPI
#elif USE_SOFTWARE_SPI
#define SOFTWARE_SPI
#endif  // LEONARDO_SOFT_SPI
//------------------------------------------------------------------------------
// define default chip select pin
//
#ifndef SOFTWARE_SPI
// hardware pin defs
/** The default chip select pin for the SD card is SS. */
uint8_t const  SD_CHIP_SELECT_PIN = SDSS;
#else  // SOFTWARE_SPI
/** SPI chip select pin */
uint8_t const SD_CHIP_SELECT_PIN = SOFT_SPI_CS_PIN;
#endif  // SOFTWARE_SPI
//------------------------------------------------------------------------------
/**
 * \class Sd2Card
 * \brief Raw access to SD and SDHC flash memory cards.
 */
class Sd2Card {
 public:
  /** Construct an instance of Sd2Card. */
  Sd2Card() : errorCode_(SD_CARD_ERROR_INIT_NOT_CALLED), type_(0) {}
  uint32_t cardSize();
  bool erase(uint32_t firstBlock, uint32_t lastBlock);
  bool eraseSingleBlockEnable();
  /**
   *  Set SD error code.
   *  \param[in] code value for error code.
   */
  void error(uint8_t code) {errorCode_ = code;}
  /**
   * \return error code for last error. See Sd2Card.h for a list of error codes.
   */
  int errorCode() const {return errorCode_;}
  /** \return error data for last error. */
  int errorData() const {return status_;}
  /**
   * Initialize an SD flash memory card with default clock rate and chip
   * select pin.  See sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin).
   *
   * \return true for success or false for failure.
   */
  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
    uint8_t chipSelectPin = SD_CHIP_SELECT_PIN);
  bool readBlock(uint32_t block, uint8_t* dst);
  /**
   * Read a card's CID register. The CID contains card identification
   * information such as Manufacturer ID, Product name, Product serial
   * number and Manufacturing date.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readCID(cid_t* cid) {
    return readRegister(CMD10, cid);
  }
  /**
   * Read a card's CSD register. The CSD contains Card-Specific Data that
   * provides information regarding access to the card's contents.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readCSD(csd_t* csd) {
    return readRegister(CMD9, csd);
  }
  bool readData(uint8_t *dst);
  bool readStart(uint32_t blockNumber);
  bool readStop();
  bool setSckRate(uint8_t sckRateID);
  /** Return the card type: SD V1, SD V2 or SDHC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC.
   */
  int type() const {return type_;}
  bool writeBlock(uint32_t blockNumber, const uint8_t* src);
  bool writeData(const uint8_t* src);
  bool writeStart(uint32_t blockNumber, uint32_t eraseCount);
  bool writeStop();

 private:
  //----------------------------------------------------------------------------
  uint8_t chipSelectPin_;
  uint8_t errorCode_;
  uint8_t spiRate_;
  uint8_t status_;
  uint8_t type_;
  // private functions
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  bool readData(uint8_t* dst, size_t count);
  bool readRegister(uint8_t cmd, void* buf);
  void chipSelectHigh();
  void chipSelectLow();
  void type(uint8_t value) {type_ = value;}
  bool waitNotBusy(uint16_t timeoutMillis);
  bool writeData(uint8_t token, const uint8_t* src);
};

/**
 * \file
 * \brief FAT file structures
 */
/*
 * mostly from Microsoft document fatgen103.doc
 * http://www.microsoft.com/whdc/system/platform/firmware/fatgen.mspx
 */
//------------------------------------------------------------------------------
/** Value for byte 510 of boot block or MBR */
uint8_t const BOOTSIG0 = 0X55;
/** Value for byte 511 of boot block or MBR */
uint8_t const BOOTSIG1 = 0XAA;
/** Value for bootSignature field int FAT/FAT32 boot sector */
uint8_t const EXTENDED_BOOT_SIG = 0X29;
//------------------------------------------------------------------------------
/**
 * \struct partitionTable
 * \brief MBR partition table entry
 *
 * A partition table entry for a MBR formatted storage device.
 * The MBR partition table has four entries.
 */
struct partitionTable {
          /**
           * Boot Indicator . Indicates whether the volume is the active
           * partition.  Legal values include: 0X00. Do not use for booting.
           * 0X80 Active partition.
           */
  uint8_t  boot;
          /**
            * Head part of Cylinder-head-sector address of the first block in
            * the partition. Legal values are 0-255. Only used in old PC BIOS.
            */
  uint8_t  beginHead;
          /**
           * Sector part of Cylinder-head-sector address of the first block in
           * the partition. Legal values are 1-63. Only used in old PC BIOS.
           */
  unsigned beginSector : 6;
           /** High bits cylinder for first block in partition. */
  unsigned beginCylinderHigh : 2;
          /**
           * Combine beginCylinderLow with beginCylinderHigh. Legal values
           * are 0-1023.  Only used in old PC BIOS.
           */
  uint8_t  beginCylinderLow;
          /**
           * Partition type. See defines that begin with PART_TYPE_ for
           * some Microsoft partition types.
           */
  uint8_t  type;
          /**
           * head part of cylinder-head-sector address of the last sector in the
           * partition.  Legal values are 0-255. Only used in old PC BIOS.
           */
  uint8_t  endHead;
          /**
           * Sector part of cylinder-head-sector address of the last sector in
           * the partition.  Legal values are 1-63. Only used in old PC BIOS.
           */
  unsigned endSector : 6;
           /** High bits of end cylinder */
  unsigned endCylinderHigh : 2;
          /**
           * Combine endCylinderLow with endCylinderHigh. Legal values
           * are 0-1023.  Only used in old PC BIOS.
           */
  uint8_t  endCylinderLow;
           /** Logical block address of the first block in the partition. */
  uint32_t firstSector;
           /** Length of the partition, in blocks. */
  uint32_t totalSectors;
} PACK;
/** Type name for partitionTable */
typedef struct partitionTable part_t;
//------------------------------------------------------------------------------
/**
 * \struct masterBootRecord
 *
 * \brief Master Boot Record
 *
 * The first block of a storage device that is formatted with a MBR.
 */
struct masterBootRecord {
           /** Code Area for master boot program. */
  uint8_t  codeArea[440];
           /** Optional Windows NT disk signature. May contain boot code. */
  uint32_t diskSignature;
           /** Usually zero but may be more boot code. */
  uint16_t usuallyZero;
           /** Partition tables. */
  part_t   part[4];
           /** First MBR signature byte. Must be 0X55 */
  uint8_t  mbrSig0;
           /** Second MBR signature byte. Must be 0XAA */
  uint8_t  mbrSig1;
} PACK;
/** Type name for masterBootRecord */
typedef struct masterBootRecord mbr_t;
//------------------------------------------------------------------------------
/**
 * \struct fat_boot
 *
 * \brief Boot sector for a FAT12/FAT16 volume.
 *
 */
struct fat_boot {
         /**
          * The first three bytes of the boot sector must be valid,
          * executable x 86-based CPU instructions. This includes a
          * jump instruction that skips the next nonexecutable bytes.
          */
  uint8_t jump[3];
         /**
          * This is typically a string of characters that identifies
          * the operating system that formatted the volume.
          */
  char    oemId[8];
          /**
           * The size of a hardware sector. Valid decimal values for this
           * field are 512, 1024, 2048, and 4096. For most disks used in
           * the United States, the value of this field is 512.
           */
  uint16_t bytesPerSector;
          /**
           * Number of sectors per allocation unit. This value must be a
           * power of 2 that is greater than 0. The legal values are
           * 1, 2, 4, 8, 16, 32, 64, and 128.  128 should be avoided.
           */
  uint8_t  sectorsPerCluster;
          /**
           * The number of sectors preceding the start of the first FAT,
           * including the boot sector. The value of this field is always 1.
           */
  uint16_t reservedSectorCount;
          /**
           * The number of copies of the FAT on the volume.
           * The value of this field is always 2.
           */
  uint8_t  fatCount;
          /**
           * For FAT12 and FAT16 volumes, this field contains the count of
           * 32-byte directory entries in the root directory. For FAT32 volumes,
           * this field must be set to 0. For FAT12 and FAT16 volumes, this
           * value should always specify a count that when multiplied by 32
           * results in a multiple of bytesPerSector.  FAT16 volumes should
           * use the value 512.
           */
  uint16_t rootDirEntryCount;
          /**
           * This field is the old 16-bit total count of sectors on the volume.
           * This count includes the count of all sectors in all four regions
           * of the volume. This field can be 0; if it is 0, then totalSectors32
           * must be nonzero.  For FAT32 volumes, this field must be 0. For
           * FAT12 and FAT16 volumes, this field contains the sector count, and
           * totalSectors32 is 0 if the total sector count fits
           * (is less than 0x10000).
           */
  uint16_t totalSectors16;
          /**
           * This dates back to the old MS-DOS 1.x media determination and is
           * no longer usually used for anything.  0xF8 is the standard value
           * for fixed (nonremovable) media. For removable media, 0xF0 is
           * frequently used. Legal values are 0xF0 or 0xF8-0xFF.
           */
  uint8_t  mediaType;
          /**
           * Count of sectors occupied by one FAT on FAT12/FAT16 volumes.
           * On FAT32 volumes this field must be 0, and sectorsPerFat32
           * contains the FAT size count.
           */
  uint16_t sectorsPerFat16;
           /** Sectors per track for interrupt 0x13. Not used otherwise. */
  uint16_t sectorsPerTrack;
           /** Number of heads for interrupt 0x13.  Not used otherwise. */
  uint16_t headCount;
          /**
           * Count of hidden sectors preceding the partition that contains this
           * FAT volume. This field is generally only relevant for media
           * visible on interrupt 0x13.
           */
  uint32_t hidddenSectors;
          /**
           * This field is the new 32-bit total count of sectors on the volume.
           * This count includes the count of all sectors in all four regions
           * of the volume.  This field can be 0; if it is 0, then
           * totalSectors16 must be nonzero.
           */
  uint32_t totalSectors32;
           /**
            * Related to the BIOS physical drive number. Floppy drives are
            * identified as 0x00 and physical hard disks are identified as
            * 0x80, regardless of the number of physical disk drives.
            * Typically, this value is set prior to issuing an INT 13h BIOS
            * call to specify the device to access. The value is only
            * relevant if the device is a boot device.
            */
  uint8_t  driveNumber;
           /** used by Windows NT - should be zero for FAT */
  uint8_t  reserved1;
           /** 0X29 if next three fields are valid */
  uint8_t  bootSignature;
           /**
            * A random serial number created when formatting a disk,
            * which helps to distinguish between disks.
            * Usually generated by combining date and time.
            */
  uint32_t volumeSerialNumber;
           /**
            * A field once used to store the volume label. The volume label
            * is now stored as a special file in the root directory.
            */
  char     volumeLabel[11];
           /**
            * A field with a value of either FAT, FAT12 or FAT16,
            * depending on the disk format.
            */
  char     fileSystemType[8];
           /** X86 boot code */
  uint8_t  bootCode[448];
           /** must be 0X55 */
  uint8_t  bootSectorSig0;
           /** must be 0XAA */
  uint8_t  bootSectorSig1;
} PACK;
/** Type name for FAT Boot Sector */
typedef struct fat_boot fat_boot_t;
//------------------------------------------------------------------------------
/**
 * \struct fat32_boot
 *
 * \brief Boot sector for a FAT32 volume.
 *
 */
struct fat32_boot {
         /**
          * The first three bytes of the boot sector must be valid,
          * executable x 86-based CPU instructions. This includes a
          * jump instruction that skips the next nonexecutable bytes.
          */
  uint8_t jump[3];
         /**
          * This is typically a string of characters that identifies
          * the operating system that formatted the volume.
          */
  char    oemId[8];
          /**
           * The size of a hardware sector. Valid decimal values for this
           * field are 512, 1024, 2048, and 4096. For most disks used in
           * the United States, the value of this field is 512.
           */
  uint16_t bytesPerSector;
          /**
           * Number of sectors per allocation unit. This value must be a
           * power of 2 that is greater than 0. The legal values are
           * 1, 2, 4, 8, 16, 32, 64, and 128.  128 should be avoided.
           */
  uint8_t  sectorsPerCluster;
          /**
           * The number of sectors preceding the start of the first FAT,
           * including the boot sector. Must not be zero
           */
  uint16_t reservedSectorCount;
          /**
           * The number of copies of the FAT on the volume.
           * The value of this field is always 2.
           */
  uint8_t  fatCount;
          /**
           * FAT12/FAT16 only. For FAT32 volumes, this field must be set to 0.
           */
  uint16_t rootDirEntryCount;
          /**
           * For FAT32 volumes, this field must be 0.
           */
  uint16_t totalSectors16;
          /**
           * This dates back to the old MS-DOS 1.x media determination and is
           * no longer usually used for anything.  0xF8 is the standard value
           * for fixed (nonremovable) media. For removable media, 0xF0 is
           * frequently used. Legal values are 0xF0 or 0xF8-0xFF.
           */
  uint8_t  mediaType;
          /**
           * On FAT32 volumes this field must be 0, and sectorsPerFat32
           * contains the FAT size count.
           */
  uint16_t sectorsPerFat16;
           /** Sectors per track for interrupt 0x13. Not used otherwise. */
  uint16_t sectorsPerTrack;
           /** Number of heads for interrupt 0x13.  Not used otherwise. */
  uint16_t headCount;
          /**
           * Count of hidden sectors preceding the partition that contains this
           * FAT volume. This field is generally only relevant for media
           * visible on interrupt 0x13.
           */
  uint32_t hidddenSectors;
          /**
           * Contains the total number of sectors in the FAT32 volume.
           */
  uint32_t totalSectors32;
         /**
           * Count of sectors occupied by one FAT on FAT32 volumes.
           */
  uint32_t sectorsPerFat32;
          /**
           * This field is only defined for FAT32 media and does not exist on
           * FAT12 and FAT16 media.
           * Bits 0-3 -- Zero-based number of active FAT.
           *             Only valid if mirroring is disabled.
           * Bits 4-6 -- Reserved.
           * Bit 7	-- 0 means the FAT is mirrored at runtime into all FATs.
	         *        -- 1 means only one FAT is active; it is the one referenced
	         *             in bits 0-3.
           * Bits 8-15 	-- Reserved.
           */
  uint16_t fat32Flags;
          /**
           * FAT32 version. High byte is major revision number.
           * Low byte is minor revision number. Only 0.0 define.
           */
  uint16_t fat32Version;
          /**
           * Cluster number of the first cluster of the root directory for FAT32.
           * This usually 2 but not required to be 2.
           */
  uint32_t fat32RootCluster;
          /**
           * Sector number of FSINFO structure in the reserved area of the
           * FAT32 volume. Usually 1.
           */
  uint16_t fat32FSInfo;
          /**
           * If nonzero, indicates the sector number in the reserved area
           * of the volume of a copy of the boot record. Usually 6.
           * No value other than 6 is recommended.
           */
  uint16_t fat32BackBootBlock;
          /**
           * Reserved for future expansion. Code that formats FAT32 volumes
           * should always set all of the bytes of this field to 0.
           */
  uint8_t  fat32Reserved[12];
           /**
            * Related to the BIOS physical drive number. Floppy drives are
            * identified as 0x00 and physical hard disks are identified as
            * 0x80, regardless of the number of physical disk drives.
            * Typically, this value is set prior to issuing an INT 13h BIOS
            * call to specify the device to access. The value is only
            * relevant if the device is a boot device.
            */
  uint8_t  driveNumber;
           /** used by Windows NT - should be zero for FAT */
  uint8_t  reserved1;
           /** 0X29 if next three fields are valid */
  uint8_t  bootSignature;
           /**
            * A random serial number created when formatting a disk,
            * which helps to distinguish between disks.
            * Usually generated by combining date and time.
            */
  uint32_t volumeSerialNumber;
           /**
            * A field once used to store the volume label. The volume label
            * is now stored as a special file in the root directory.
            */
  char     volumeLabel[11];
           /**
            * A text field with a value of FAT32.
            */
  char     fileSystemType[8];
           /** X86 boot code */
  uint8_t  bootCode[420];
           /** must be 0X55 */
  uint8_t  bootSectorSig0;
           /** must be 0XAA */
  uint8_t  bootSectorSig1;
} PACK;
/** Type name for FAT32 Boot Sector */
typedef struct fat32_boot fat32_boot_t;
//------------------------------------------------------------------------------
/** Lead signature for a FSINFO sector */
uint32_t const FSINFO_LEAD_SIG = 0x41615252;
/** Struct signature for a FSINFO sector */
uint32_t const FSINFO_STRUCT_SIG = 0x61417272;
/**
 * \struct fat32_fsinfo
 *
 * \brief FSINFO sector for a FAT32 volume.
 *
 */
struct fat32_fsinfo {
           /** must be 0X52, 0X52, 0X61, 0X41 */
  uint32_t  leadSignature;
           /** must be zero */
  uint8_t  reserved1[480];
           /** must be 0X72, 0X72, 0X41, 0X61 */
  uint32_t  structSignature;
          /**
           * Contains the last known free cluster count on the volume.
           * If the value is 0xFFFFFFFF, then the free count is unknown
           * and must be computed. Any other value can be used, but is
           * not necessarily correct. It should be range checked at least
           * to make sure it is <= volume cluster count.
           */
  uint32_t freeCount;
          /**
           * This is a hint for the FAT driver. It indicates the cluster
           * number at which the driver should start looking for free clusters.
           * If the value is 0xFFFFFFFF, then there is no hint and the driver
           * should start looking at cluster 2.
           */
  uint32_t nextFree;
           /** must be zero */
  uint8_t  reserved2[12];
           /** must be 0X00, 0X00, 0X55, 0XAA */
  uint8_t  tailSignature[4];
} PACK;
/** Type name for FAT32 FSINFO Sector */
typedef struct fat32_fsinfo fat32_fsinfo_t;
//------------------------------------------------------------------------------
// End Of Chain values for FAT entries
/** FAT12 end of chain value used by Microsoft. */
uint16_t const FAT12EOC = 0XFFF;
/** Minimum value for FAT12 EOC.  Use to test for EOC. */
uint16_t const FAT12EOC_MIN = 0XFF8;
/** FAT16 end of chain value used by Microsoft. */
uint16_t const FAT16EOC = 0XFFFF;
/** Minimum value for FAT16 EOC.  Use to test for EOC. */
uint16_t const FAT16EOC_MIN = 0XFFF8;
/** FAT32 end of chain value used by Microsoft. */
uint32_t const FAT32EOC = 0X0FFFFFFF;
/** Minimum value for FAT32 EOC.  Use to test for EOC. */
uint32_t const FAT32EOC_MIN = 0X0FFFFFF8;
/** Mask a for FAT32 entry. Entries are 28 bits. */
uint32_t const FAT32MASK = 0X0FFFFFFF;

// Reuse directory entries from deleted files
#define SD_CARD_REUSE_FAT_ENTRIES true
//------------------------------------------------------------------------------
/**
 * \struct directoryEntry
 * \brief FAT short directory entry
 *
 * Short means short 8.3 name, not the entry size.
 *
 * Date Format. A FAT directory entry date stamp is a 16-bit field that is
 * basically a date relative to the MS-DOS epoch of 01/01/1980. Here is the
 * format (bit 0 is the LSB of the 16-bit word, bit 15 is the MSB of the
 * 16-bit word):
 *
 * Bits 9-15: Count of years from 1980, valid value range 0-127
 * inclusive (1980-2107).
 *
 * Bits 5-8: Month of year, 1 = January, valid value range 1-12 inclusive.
 *
 * Bits 0-4: Day of month, valid value range 1-31 inclusive.
 *
 * Time Format. A FAT directory entry time stamp is a 16-bit field that has
 * a granularity of 2 seconds. Here is the format (bit 0 is the LSB of the
 * 16-bit word, bit 15 is the MSB of the 16-bit word).
 *
 * Bits 11-15: Hours, valid value range 0-23 inclusive.
 *
 * Bits 5-10: Minutes, valid value range 0-59 inclusive.
 *
 * Bits 0-4: 2-second count, valid value range 0-29 inclusive (0 - 58 seconds).
 *
 * The valid time range is from Midnight 00:00:00 to 23:59:58.
 */
struct directoryEntry {
           /** Short 8.3 name.
            *
            * The first eight bytes contain the file name with blank fill.
            * The last three bytes contain the file extension with blank fill.
            */
  uint8_t  name[11];
          /** Entry attributes.
           *
           * The upper two bits of the attribute byte are reserved and should
           * always be set to 0 when a file is created and never modified or
           * looked at after that.  See defines that begin with DIR_ATT_.
           */
  uint8_t  attributes;
          /**
           * Reserved for use by Windows NT. Set value to 0 when a file is
           * created and never modify or look at it after that.
           */
  uint8_t  reservedNT;
          /**
           * The granularity of the seconds part of creationTime is 2 seconds
           * so this field is a count of tenths of a second and its valid
           * value range is 0-199 inclusive. (WHG note - seems to be hundredths)
           */
  uint8_t  creationTimeTenths;
           /** Time file was created. */
  uint16_t creationTime;
           /** Date file was created. */
  uint16_t creationDate;
          /**
           * Last access date. Note that there is no last access time, only
           * a date.  This is the date of last read or write. In the case of
           * a write, this should be set to the same date as lastWriteDate.
           */
  uint16_t lastAccessDate;
          /**
           * High word of this entry's first cluster number (always 0 for a
           * FAT12 or FAT16 volume).
           */
  uint16_t firstClusterHigh;
           /** Time of last write. File creation is considered a write. */
  uint16_t lastWriteTime;
           /** Date of last write. File creation is considered a write. */
  uint16_t lastWriteDate;
           /** Low word of this entry's first cluster number. */
  uint16_t firstClusterLow;
           /** 32-bit unsigned holding this file's size in bytes. */
  uint32_t fileSize;
} PACK;

// LONG FILENAME FAT ENTRY
struct directoryVFATEntry {
  /**
   * Sequence number. Consists of 2 parts:
   *  bit 6:   indicates first long filename block for the next file
   *  bit 0-4: the position of this long filename block (first block is 1)
   */
  uint8_t  sequenceNumber;
  /** First set of UTF-16 characters */
  uint16_t name1[5];//UTF-16
  /** attributes (at the same location as in directoryEntry), always 0x0F */
  uint8_t  attributes;
  /** Reserved for use by Windows NT. Always 0. */
  uint8_t  reservedNT;
  /** Checksum of the short 8.3 filename, can be used to checked if the file system as modified by a not-long-filename aware implementation. */
  uint8_t  checksum;
  /** Second set of UTF-16 characters */
  uint16_t name2[6];//UTF-16
  /** firstClusterLow is always zero for longFilenames */
  uint16_t firstClusterLow;
  /** Third set of UTF-16 characters */
  uint16_t name3[2];//UTF-16
} PACK;
typedef struct directoryVFATEntry vfat_t;


//------------------------------------------------------------------------------
// Definitions for directory entries
//
/** Type name for directoryEntry */
typedef struct directoryEntry dir_t;
/** escape for name[0] = 0XE5 */
uint8_t const DIR_NAME_0XE5 = 0X05;
/** name[0] value for entry that is free after being "deleted" */
uint8_t const DIR_NAME_DELETED = 0XE5;
/** name[0] value for entry that is free and no allocated entries follow */
uint8_t const DIR_NAME_FREE = 0X00;
/** file is read-only */
uint8_t const DIR_ATT_READ_ONLY = 0X01;
/** File should hidden in directory listings */
uint8_t const DIR_ATT_HIDDEN = 0X02;
/** Entry is for a system file */
uint8_t const DIR_ATT_SYSTEM = 0X04;
/** Directory entry contains the volume label */
uint8_t const DIR_ATT_VOLUME_ID = 0X08;
/** Entry is for a directory */
uint8_t const DIR_ATT_DIRECTORY = 0X10;
/** Old DOS archive bit for backup support */
uint8_t const DIR_ATT_ARCHIVE = 0X20;
/** Test value for long name entry.  Test is
  (d->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME. */
uint8_t const DIR_ATT_LONG_NAME = 0X0F;
/** Test mask for long name entry */
uint8_t const DIR_ATT_LONG_NAME_MASK = 0X3F;
/** defined attribute bits */
uint8_t const DIR_ATT_DEFINED_BITS = 0X3F;
/** Directory entry is part of a long name
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for part of a long name else false.
 */
static inline uint8_t DIR_IS_LONG_NAME(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME;
}
/** Mask for file/subdirectory tests */
uint8_t const DIR_ATT_FILE_TYPE_MASK = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY);
/** Directory entry is for a file
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a normal file else false.
 */
static inline uint8_t DIR_IS_FILE(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == 0;
}
/** Directory entry is for a subdirectory
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a subdirectory else false.
 */
static inline uint8_t DIR_IS_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == DIR_ATT_DIRECTORY;
}
/** Directory entry is for a file or subdirectory
 * \param[in] dir Pointer to a directory entry.
 *
 * \return true if the entry is for a normal file or subdirectory else false.
 */
static inline uint8_t DIR_IS_FILE_OR_SUBDIR(const dir_t* dir) {
  return (dir->attributes & DIR_ATT_VOLUME_ID) == 0;
}

//==============================================================================
// SdVolume class
/**
 * \brief Cache for an SD data block
 */
union cache_t {
           /** Used to access cached file data blocks. */
  uint8_t  data[512];
           /** Used to access cached FAT16 entries. */
  uint16_t fat16[256];
           /** Used to access cached FAT32 entries. */
  uint32_t fat32[128];
           /** Used to access cached directory entries. */
  dir_t    dir[16];
           /** Used to access a cached Master Boot Record. */
  mbr_t    mbr;
           /** Used to access to a cached FAT boot sector. */
  fat_boot_t fbs;
           /** Used to access to a cached FAT32 boot sector. */
  fat32_boot_t fbs32;
           /** Used to access to a cached FAT32 FSINFO sector. */
  fat32_fsinfo_t fsinfo;
} PACK;
//------------------------------------------------------------------------------
/**
 * \class SdVolume
 * \brief Access FAT16 and FAT32 volumes on SD and SDHC cards.
 */
class SdVolume {
 public:
  /** Create an instance of SdVolume */
  SdVolume() : fatType_(0) {}
  /** Clear the cache and returns a pointer to the cache.  Used by the WaveRP
   * recorder to do raw write to the SD card.  Not for normal apps.
   * \return A pointer to the cache buffer or zero if an error occurs.
   */
  cache_t* cacheClear() {
    if (!cacheSync()) return 0;
    cacheBlockNumber_ = 0XFFFFFFFF;
    return &cacheBuffer_;
  }
  /** Initialize a FAT volume.  Try partition one first then try super
   * floppy format.
   *
   * \param[in] dev The Sd2Card where the volume is located.
   *
   * \return The value one, true, is returned for success and
   * the value zero, false, is returned for failure.  Reasons for
   * failure include not finding a valid partition, not finding a valid
   * FAT file system or an I/O error.
   */
  bool init(Sd2Card* dev) { return init(dev, 1) ? true : init(dev, 0);}
  bool init(Sd2Card* dev, uint8_t part);

  // inline functions that return volume info
  /** \return The volume's cluster size in blocks. */
  uint8_t blocksPerCluster() const {return blocksPerCluster_;}
  /** \return The number of blocks in one FAT. */
  uint32_t blocksPerFat()  const {return blocksPerFat_;}
  /** \return The total number of clusters in the volume. */
  uint32_t clusterCount() const {return clusterCount_;}
  /** \return The shift count required to multiply by blocksPerCluster. */
  uint8_t clusterSizeShift() const {return clusterSizeShift_;}
  /** \return The logical block number for the start of file data. */
  uint32_t dataStartBlock() const {return dataStartBlock_;}
  /** \return The number of FAT structures on the volume. */
  uint8_t fatCount() const {return fatCount_;}
  /** \return The logical block number for the start of the first FAT. */
  uint32_t fatStartBlock() const {return fatStartBlock_;}
  /** \return The FAT type of the volume. Values are 12, 16 or 32. */
  uint8_t fatType() const {return fatType_;}
  int32_t freeClusterCount();
  /** \return The number of entries in the root directory for FAT16 volumes. */
  uint32_t rootDirEntryCount() const {return rootDirEntryCount_;}
  /** \return The logical block number for the start of the root directory
       on FAT16 volumes or the first cluster number on FAT32 volumes. */
  uint32_t rootDirStart() const {return rootDirStart_;}
  /** Sd2Card object for this volume
   * \return pointer to Sd2Card object.
   */
  Sd2Card* sdCard() {return sdCard_;}
  /** Debug access to FAT table
   *
   * \param[in] n cluster number.
   * \param[out] v value of entry
   * \return true for success or false for failure
   */
  bool dbgFat(uint32_t n, uint32_t* v) {return fatGet(n, v);}
//------------------------------------------------------------------------------
 private:
  // Allow SdBaseFile access to SdVolume private data.
  friend class SdBaseFile;
//------------------------------------------------------------------------------
  uint32_t allocSearchStart_;   // start cluster for alloc search
  uint8_t blocksPerCluster_;    // cluster size in blocks
  uint32_t blocksPerFat_;       // FAT size in blocks
  uint32_t clusterCount_;       // clusters in one FAT
  uint8_t clusterSizeShift_;    // shift to convert cluster count to block count
  uint32_t dataStartBlock_;     // first data block number
  uint8_t fatCount_;            // number of FATs on volume
  uint32_t fatStartBlock_;      // start block for first FAT
  uint8_t fatType_;             // volume type (12, 16, OR 32)
  uint16_t rootDirEntryCount_;  // number of entries in FAT16 root dir
  uint32_t rootDirStart_;       // root start block for FAT16, cluster for FAT32
//------------------------------------------------------------------------------
// block caches
// use of static functions save a bit of flash - maybe not worth complexity
//
  static const uint8_t CACHE_STATUS_DIRTY = 1;
  static const uint8_t CACHE_STATUS_FAT_BLOCK = 2;
  static const uint8_t CACHE_STATUS_MASK
     = CACHE_STATUS_DIRTY | CACHE_STATUS_FAT_BLOCK;
  static const uint8_t CACHE_OPTION_NO_READ = 4;
  // value for option argument in cacheFetch to indicate read from cache
  static uint8_t const CACHE_FOR_READ = 0;
  // value for option argument in cacheFetch to indicate write to cache
  static uint8_t const CACHE_FOR_WRITE = CACHE_STATUS_DIRTY;
  // reserve cache block with no read
  static uint8_t const CACHE_RESERVE_FOR_WRITE
     = CACHE_STATUS_DIRTY | CACHE_OPTION_NO_READ;
#if USE_MULTIPLE_CARDS
  cache_t cacheBuffer_;        // 512 byte cache for device blocks
  uint32_t cacheBlockNumber_;  // Logical number of block in the cache
  uint32_t cacheFatOffset_;    // offset for mirrored FAT
  Sd2Card* sdCard_;            // Sd2Card object for cache
  uint8_t cacheStatus_;        // status of cache block
#if USE_SEPARATE_FAT_CACHE
  cache_t cacheFatBuffer_;       // 512 byte cache for FAT
  uint32_t cacheFatBlockNumber_;  // current Fat block number
  uint8_t  cacheFatStatus_;       // status of cache Fatblock
#endif  // USE_SEPARATE_FAT_CACHE
#else  // USE_MULTIPLE_CARDS
  static cache_t cacheBuffer_;        // 512 byte cache for device blocks
  static uint32_t cacheBlockNumber_;  // Logical number of block in the cache
  static uint32_t cacheFatOffset_;    // offset for mirrored FAT
  static uint8_t cacheStatus_;        // status of cache block
#if USE_SEPARATE_FAT_CACHE
  static cache_t cacheFatBuffer_;       // 512 byte cache for FAT
  static uint32_t cacheFatBlockNumber_;  // current Fat block number
  static uint8_t  cacheFatStatus_;       // status of cache Fatblock
#endif  // USE_SEPARATE_FAT_CACHE
  static Sd2Card* sdCard_;            // Sd2Card object for cache
#endif  // USE_MULTIPLE_CARDS

  cache_t *cacheAddress() {return &cacheBuffer_;}
  uint32_t cacheBlockNumber() {return cacheBlockNumber_;}
#if USE_MULTIPLE_CARDS
  cache_t* cacheFetch(uint32_t blockNumber, uint8_t options);
  cache_t* cacheFetchData(uint32_t blockNumber, uint8_t options);
  cache_t* cacheFetchFat(uint32_t blockNumber, uint8_t options);
  void cacheInvalidate();
  bool cacheSync();
  bool cacheWriteData();
  bool cacheWriteFat();
#else  // USE_MULTIPLE_CARDS
  static cache_t* cacheFetch(uint32_t blockNumber, uint8_t options);
  static cache_t* cacheFetchData(uint32_t blockNumber, uint8_t options);
  static cache_t* cacheFetchFat(uint32_t blockNumber, uint8_t options);
  static void cacheInvalidate();
  static bool cacheSync();
  static bool cacheWriteData();
  static bool cacheWriteFat();
#endif  // USE_MULTIPLE_CARDS
//------------------------------------------------------------------------------
  bool allocContiguous(uint32_t count, uint32_t* curCluster);
  uint8_t blockOfCluster(uint32_t position) const {
          return (position >> 9) & (blocksPerCluster_ - 1);}
  uint32_t clusterStartBlock(uint32_t cluster) const;
  bool fatGet(uint32_t cluster, uint32_t* value);
  bool fatPut(uint32_t cluster, uint32_t value);
  bool fatPutEOC(uint32_t cluster) {
    return fatPut(cluster, 0x0FFFFFFF);
  }
  bool freeChain(uint32_t cluster);
  bool isEOC(uint32_t cluster) const {
    if (FAT12_SUPPORT && fatType_ == 12) return  cluster >= FAT12EOC_MIN;
    if (fatType_ == 16) return cluster >= FAT16EOC_MIN;
    return  cluster >= FAT32EOC_MIN;
  }
  bool readBlock(uint32_t block, uint8_t* dst) {
    return sdCard_->readBlock(block, dst);}
  bool writeBlock(uint32_t block, const uint8_t* dst) {
    return sdCard_->writeBlock(block, dst);
  }
//------------------------------------------------------------------------------
  // Deprecated functions  - suppress cpplint warnings with NOLINT comment
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)

 public:
  /** \deprecated Use: bool SdVolume::init(Sd2Card* dev);
   * \param[in] dev The SD card where the volume is located.
   * \return true for success or false for failure.
   */
  bool init(Sd2Card& dev) {return init(&dev);}  // NOLINT
  /** \deprecated Use: bool SdVolume::init(Sd2Card* dev, uint8_t vol);
   * \param[in] dev The SD card where the volume is located.
   * \param[in] part The partition to be used.
   * \return true for success or false for failure.
   */
  bool init(Sd2Card& dev, uint8_t part) {  // NOLINT
    return init(&dev, part);
  }
#endif  // ALLOW_DEPRECATED_FUNCTIONS
};

//------------------------------------------------------------------------------
/**
 * \struct FatPos_t
 * \brief internal type for istream
 * do not use in user apps
 */
struct FatPos_t {
  /** stream position */
  uint32_t position;
  /** cluster for position */
  uint32_t cluster;
  FatPos_t() : position(0), cluster(0) {}
};

// use the gnu style oflag in open()
/** open() oflag for reading */
uint8_t const O_READ = 0X01;
/** open() oflag - same as O_IN */
uint8_t const O_RDONLY = O_READ;
/** open() oflag for write */
uint8_t const O_WRITE = 0X02;
/** open() oflag - same as O_WRITE */
uint8_t const O_WRONLY = O_WRITE;
/** open() oflag for reading and writing */
uint8_t const O_RDWR = (O_READ | O_WRITE);
/** open() oflag mask for access modes */
uint8_t const O_ACCMODE = (O_READ | O_WRITE);
/** The file offset shall be set to the end of the file prior to each write. */
uint8_t const O_APPEND = 0X04;
/** synchronous writes - call sync() after each write */
uint8_t const O_SYNC = 0X08;
/** truncate the file to zero length */
uint8_t const O_TRUNC = 0X10;
/** set the initial position at the end of the file */
uint8_t const O_AT_END = 0X20;
/** create the file if nonexistent */
uint8_t const O_CREAT = 0X40;
/** If O_CREAT and O_EXCL are set, open() shall fail if the file exists */
uint8_t const O_EXCL = 0X80;

// SdBaseFile class static and const definitions
// flags for ls()
/** ls() flag to print modify date */
uint8_t const LS_DATE = 1;
/** ls() flag to print file size */
uint8_t const LS_SIZE = 2;
/** ls() flag for recursive list of subdirectories */
uint8_t const LS_R = 4;


// flags for timestamp
/** set the file's last access date */
uint8_t const T_ACCESS = 1;
/** set the file's creation date and time */
uint8_t const T_CREATE = 2;
/** Set the file's write date and time */
uint8_t const T_WRITE = 4;
// values for type_
/** This file has not been opened. */
uint8_t const FAT_FILE_TYPE_CLOSED = 0;
/** A normal file */
uint8_t const FAT_FILE_TYPE_NORMAL = 1;
/** A FAT12 or FAT16 root directory */
uint8_t const FAT_FILE_TYPE_ROOT_FIXED = 2;
/** A FAT32 root directory */
uint8_t const FAT_FILE_TYPE_ROOT32 = 3;
/** A subdirectory file*/
uint8_t const FAT_FILE_TYPE_SUBDIR = 4;
/** Test value for directory type */
uint8_t const FAT_FILE_TYPE_MIN_DIR = FAT_FILE_TYPE_ROOT_FIXED;

/** date field for FAT directory entry
 * \param[in] year [1980,2107]
 * \param[in] month [1,12]
 * \param[in] day [1,31]
 *
 * \return Packed date for dir_t entry.
 */
static inline uint16_t FAT_DATE(uint16_t year, uint8_t month, uint8_t day) {
  return (year - 1980) << 9 | month << 5 | day;
}
/** year part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted year [1980,2107]
 */
static inline uint16_t FAT_YEAR(uint16_t fatDate) {
  return 1980 + (fatDate >> 9);
}
/** month part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted month [1,12]
 */
static inline uint8_t FAT_MONTH(uint16_t fatDate) {
  return (fatDate >> 5) & 0XF;
}
/** day part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted day [1,31]
 */
static inline uint8_t FAT_DAY(uint16_t fatDate) {
  return fatDate & 0X1F;
}
/** time field for FAT directory entry
 * \param[in] hour [0,23]
 * \param[in] minute [0,59]
 * \param[in] second [0,59]
 *
 * \return Packed time for dir_t entry.
 */
static inline uint16_t FAT_TIME(uint8_t hour, uint8_t minute, uint8_t second) {
  return hour << 11 | minute << 5 | second >> 1;
}
/** hour part of FAT directory time field
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted hour [0,23]
 */
static inline uint8_t FAT_HOUR(uint16_t fatTime) {
  return fatTime >> 11;
}
/** minute part of FAT directory time field
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted minute [0,59]
 */
static inline uint8_t FAT_MINUTE(uint16_t fatTime) {
  return(fatTime >> 5) & 0X3F;
}
/** second part of FAT directory time field
 * Note second/2 is stored in packed time.
 *
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted second [0,58]
 */
static inline uint8_t FAT_SECOND(uint16_t fatTime) {
  return 2*(fatTime & 0X1F);
}
/** Default date for file timestamps is 1 Jan 2000 */
uint16_t const FAT_DEFAULT_DATE = ((2000 - 1980) << 9) | (1 << 5) | 1;
/** Default time for file timestamp is 1 am */
uint16_t const FAT_DEFAULT_TIME = (1 << 11);
//------------------------------------------------------------------------------
/**
 * \class SdBaseFile
 * \brief Base class for SdFile with Print and C++ streams.
 */
class SdBaseFile {
 public:
  /** Create an instance. */
  SdBaseFile() : writeError(false), type_(FAT_FILE_TYPE_CLOSED) {}
  SdBaseFile(const char* path, uint8_t oflag);
  #if DESTRUCTOR_CLOSES_FILE
  ~SdBaseFile() {if(isOpen()) close();}
  #endif
  /**
   * writeError is set to true if an error occurs during a write().
   * Set writeError to false before calling print() and/or write() and check
   * for true after calls to print() and/or write().
   */
  bool writeError;
  /** \return value of writeError */
  bool getWriteError() {return writeError;}
  /** Set writeError to zero */
  void clearWriteError() {writeError = 0;}
  //----------------------------------------------------------------------------
  // helpers for stream classes
  /** get position for streams
   * \param[out] pos struct to receive position
   */
  void getpos(FatPos_t* pos);
  /** set position for streams
   * \param[out] pos struct with value for new position
   */
  void setpos(FatPos_t* pos);
  //----------------------------------------------------------------------------
  /** \return number of bytes available from yhe current position to EOF */
  uint32_t available() {return fileSize() - curPosition();}
  bool close();
  bool contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock);
  bool createContiguous(SdBaseFile* dirFile,
          const char* path, uint32_t size);
  /** \return The current cluster number for a file or directory. */
  uint32_t curCluster() const {return curCluster_;}
  /** \return The current position for a file or directory. */
  uint32_t curPosition() const {return curPosition_;}
  /** \return Current working directory */
  static SdBaseFile* cwd() {return cwd_;}
  /** Set the date/time callback function
   *
   * \param[in] dateTime The user's call back function.  The callback
   * function is of the form:
   *
   * \code
   * void dateTime(uint16_t* date, uint16_t* time) {
   *   uint16_t year;
   *   uint8_t month, day, hour, minute, second;
   *
   *   // User gets date and time from GPS or real-time clock here
   *
   *   // return date using FAT_DATE macro to format fields
   *   *date = FAT_DATE(year, month, day);
   *
   *   // return time using FAT_TIME macro to format fields
   *   *time = FAT_TIME(hour, minute, second);
   * }
   * \endcode
   *
   * Sets the function that is called when a file is created or when
   * a file's directory entry is modified by sync(). All timestamps,
   * access, creation, and modify, are set when a file is created.
   * sync() maintains the last access date and last modify date/time.
   *
   * See the timestamp() function.
   */
  static void dateTimeCallback(
    void (*dateTime)(uint16_t* date, uint16_t* time)) {
    dateTime_ = dateTime;
  }
  /**  Cancel the date/time callback function. */
  static void dateTimeCallbackCancel() {dateTime_ = 0;}
  bool dirEntry(dir_t* dir);
  static void dirName(const dir_t& dir, char* name);
  bool exists(const char* name);
  int16_t fgets(char* str, int16_t num, char* delim = 0);
  /** \return The total number of bytes in a file or directory. */
  uint32_t fileSize() const {return fileSize_;}
  /** \return The first cluster number for a file or directory. */
  uint32_t firstCluster() const {return firstCluster_;}
  bool getFilename(char* name);
  uint8_t lfn_checksum(const unsigned char *pFCBName);
  bool openParentReturnFile(SdBaseFile* dirFile, const char* path, uint8_t *dname, SdBaseFile *newParent, boolean bMakeDirs);

  
  /** \return True if this is a directory else false. */
  bool isDir() const {return type_ >= FAT_FILE_TYPE_MIN_DIR;}
  /** \return True if this is a normal file else false. */
  bool isFile() const {return type_ == FAT_FILE_TYPE_NORMAL;}
  /** \return True if this is an open file/directory else false. */
  bool isOpen() const {return type_ != FAT_FILE_TYPE_CLOSED;}
  /** \return True if this is a subdirectory else false. */
  bool isSubDir() const {return type_ == FAT_FILE_TYPE_SUBDIR;}
  /** \return True if this is the root directory. */
  bool isRoot() const {
    return type_ == FAT_FILE_TYPE_ROOT_FIXED || type_ == FAT_FILE_TYPE_ROOT32;
  }
  void ls(uint8_t flags = 0, uint8_t indent = 0);
  void ls(uint8_t flags = 0);
  bool mkdir(SdBaseFile* dir, const char* path, bool pFlag = true);
  // alias for backward compactability
  bool makeDir(SdBaseFile* dir, const char* path) {
    return mkdir(dir, path, false);
  }
  bool open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag);
  bool open(SdBaseFile* dirFile, const char* path, uint8_t oflag);
  bool open(const char* path, uint8_t oflag = O_READ);
  bool openNext(SdBaseFile* dirFile, uint8_t oflag);
  bool openRoot(SdVolume* vol);
  int8_t readDir(dir_t& dir, char *longfilename) {return readDir(&dir, longfilename);}
  int peek();
  bool printCreateDateTime();
  static void printFatDate(uint16_t fatDate);
  static void printFatTime(uint16_t fatTime);
  bool printModifyDateTime();
  int printField(int16_t value, char term);
  int printField(uint16_t value, char term);
  int printField(int32_t value, char term);
  int printField(uint32_t value, char term);
  bool printName();
  int16_t read();
  int read(void* buf, size_t nbyte);
  int8_t readDir(dir_t* dir, char *longfilename);

  static bool remove(SdBaseFile* dirFile, const char* path);
  bool remove();
  /** Set the file's current position to zero. */
  void rewind() {seekSet(0);}
  bool rename(SdBaseFile* dirFile, const char* newPath);
  bool rmdir();
  // for backward compatibility
  bool rmDir() {return rmdir();}
  bool rmRfStar();
  /** Set the files position to current position + \a pos. See seekSet().
   * \param[in] offset The new position in bytes from the current position.
   * \return true for success or false for failure.
   */
  bool seekCur(int32_t offset) {
    return seekSet(curPosition_ + offset);
  }
  /** Set the files position to end-of-file + \a offset. See seekSet().
   * \param[in] offset The new position in bytes from end-of-file.
   * \return true for success or false for failure.
   */
  bool seekEnd(int32_t offset = 0) {return seekSet(fileSize_ + offset);}
  bool seekSet(uint32_t pos);
  bool sync();
  bool timestamp(SdBaseFile* file);
  bool timestamp(uint8_t flag, uint16_t year, uint8_t month, uint8_t day,
          uint8_t hour, uint8_t minute, uint8_t second);
  /** Type of file.  You should use isFile() or isDir() instead of type()
   * if possible.
   *
   * \return The file or directory type.
   */
  uint8_t type() const {return type_;}
  bool truncate(uint32_t size);
  /** \return SdVolume that contains this file. */
  SdVolume* volume() const {return vol_;}
  int write(const void* buf, size_t nbyte);
//------------------------------------------------------------------------------
 public:
  // allow SdFat to set cwd_
  friend class SdFat;
  // global pointer to cwd dir
  static SdBaseFile* cwd_;
  // data time callback function
  static void (*dateTime_)(uint16_t* date, uint16_t* time);
  // bits defined in flags_
  // should be 0X0F
  static uint8_t const F_OFLAG = (O_ACCMODE | O_APPEND | O_SYNC);
  // sync of directory entry required
  static uint8_t const F_FILE_DIR_DIRTY = 0X80;

  // private data
  uint8_t   flags_;         // See above for definition of flags_ bits
  uint8_t   fstate_;        // error and eof indicator
  uint8_t   type_;          // type of file see above for values
  uint8_t   dirIndex_;      // index of directory entry in dirBlock
  SdVolume* vol_;           // volume where file is located
  uint32_t  curCluster_;    // cluster for current file position
  uint32_t  curPosition_;   // current file position in bytes from beginning
  uint32_t  dirBlock_;      // block for this files directory entry
  uint32_t  fileSize_;      // file size in bytes
  uint32_t  firstCluster_;  // first cluster of file
  char *pathend;



  /** experimental don't use */
  bool openParent(SdBaseFile* dir);
  // private functions
  bool addCluster();
  cache_t* addDirCluster();
  dir_t* cacheDirEntry(uint8_t action);
  int8_t lsPrintNext(uint8_t flags, uint8_t indent);
  static bool make83Name(const char* str, uint8_t* name, const char** ptr);
  bool mkdir(SdBaseFile* parent, const uint8_t *dname);
  bool open(SdBaseFile* dirFile, const uint8_t *dname, uint8_t oflag, bool bDir);
  bool openCachedEntry(uint8_t cacheIndex, uint8_t oflags);
  dir_t* readDirCache();
  dir_t* readDirCacheSpecial();
  dir_t *getLongFilename(dir_t *dir, char *longFilename, int8_t cVFATNeeded, uint32_t *pwIndexPos);
  bool findSpace(dir_t *dir, int8_t cVFATNeeded, int8_t *pcVFATFound, uint32_t *pwIndexPos);
  uint8_t lsRecursive(SdBaseFile *parent, uint8_t level, char *findFilename, SdBaseFile *pParentFound);

  bool setDirSize();
//------------------------------------------------------------------------------
// to be deleted
  static void printDirName(const dir_t& dir,
    uint8_t width, bool printSlash);
//------------------------------------------------------------------------------
// Deprecated functions  - suppress cpplint warnings with NOLINT comment
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)

 public:
  /** \deprecated Use:
   * bool contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock);
   * \param[out] bgnBlock the first block address for the file.
   * \param[out] endBlock the last  block address for the file.
   * \return true for success or false for failure.
   */
  bool contiguousRange(uint32_t& bgnBlock, uint32_t& endBlock) {  // NOLINT
    return contiguousRange(&bgnBlock, &endBlock);
  }
 /** \deprecated Use:
   * bool createContiguous(SdBaseFile* dirFile,
   *   const char* path, uint32_t size)
   * \param[in] dirFile The directory where the file will be created.
   * \param[in] path A path with a valid DOS 8.3 file name.
   * \param[in] size The desired file size.
   * \return true for success or false for failure.
   */
  bool createContiguous(SdBaseFile& dirFile,  // NOLINT
    const char* path, uint32_t size) {
    return createContiguous(&dirFile, path, size);
  }
  /** \deprecated Use:
   * static void dateTimeCallback(
   *   void (*dateTime)(uint16_t* date, uint16_t* time));
   * \param[in] dateTime The user's call back function.
   */
  static void dateTimeCallback(
    void (*dateTime)(uint16_t& date, uint16_t& time)) {  // NOLINT
    oldDateTime_ = dateTime;
    dateTime_ = dateTime ? oldToNew : 0;
  }
  /** \deprecated Use: bool dirEntry(dir_t* dir);
   * \param[out] dir Location for return of the file's directory entry.
   * \return true for success or false for failure.
   */
  bool dirEntry(dir_t& dir) {return dirEntry(&dir);}  // NOLINT
  /** \deprecated Use:
   * bool mkdir(SdBaseFile* dir, const char* path);
   * \param[in] dir An open SdFat instance for the directory that will contain
   * the new directory.
   * \param[in] path A path with a valid 8.3 DOS name for the new directory.
   * \return true for success or false for failure.
   */
  bool mkdir(SdBaseFile& dir, const char* path) {  // NOLINT
    return mkdir(&dir, path);
  }
  /** \deprecated Use:
   * bool open(SdBaseFile* dirFile, const char* path, uint8_t oflag);
   * \param[in] dirFile An open SdFat instance for the directory containing the
   * file to be opened.
   * \param[in] path A path with a valid 8.3 DOS name for the file.
   * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
   * OR of flags O_READ, O_WRITE, O_TRUNC, and O_SYNC.
   * \return true for success or false for failure.
   */
  bool open(SdBaseFile& dirFile, // NOLINT
    const char* path, uint8_t oflag) {
    return open(&dirFile, path, oflag);
  }
  /** \deprecated  Do not use in new apps
   * \param[in] dirFile An open SdFat instance for the directory containing the
   * file to be opened.
   * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
   * \return true for success or false for failure.
   */
  bool open(SdBaseFile& dirFile, const char* path) {  // NOLINT
    return open(dirFile, path, O_RDWR);
  }
  /** \deprecated Use:
   * bool open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag);
   * \param[in] dirFile An open SdFat instance for the directory.
   * \param[in] index The \a index of the directory entry for the file to be
   * opened.  The value for \a index is (directory file position)/32.
   * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
   * OR of flags O_READ, O_WRITE, O_TRUNC, and O_SYNC.
   * \return true for success or false for failure.
   */
  bool open(SdBaseFile& dirFile, uint16_t index, uint8_t oflag) {  // NOLINT
    return open(&dirFile, index, oflag);
  }
  /** \deprecated Use: bool openRoot(SdVolume* vol);
   * \param[in] vol The FAT volume containing the root directory to be opened.
   * \return true for success or false for failure.
   */
  bool openRoot(SdVolume& vol) {return openRoot(&vol);}  // NOLINT
  /** \deprecated Use: int8_t readDir(dir_t* dir);
   * \param[out] dir The dir_t struct that will receive the data.
   * \return bytes read for success zero for eof or -1 for failure.
   */
  int8_t readDir(dir_t& dir) {return readDir(&dir);}  // NOLINT
  /** \deprecated Use:
   * static uint8_t remove(SdBaseFile* dirFile, const char* path);
   * \param[in] dirFile The directory that contains the file.
   * \param[in] path The name of the file to be removed.
   * \return true for success or false for failure.
   */
  static bool remove(SdBaseFile& dirFile, const char* path) {  // NOLINT
    return remove(&dirFile, path);
  }
//------------------------------------------------------------------------------
// rest are private
 private:
  static void (*oldDateTime_)(uint16_t& date, uint16_t& time);  // NOLINT
  static void oldToNew(uint16_t* date, uint16_t* time) {
    uint16_t d;
    uint16_t t;
    oldDateTime_(d, t);
    *date = d;
    *time = t;
  }
#elif !defined(DOXYGEN)  // ALLOW_DEPRECATED_FUNCTIONS
 public:
  bool contiguousRange(uint32_t& bgnBlock, uint32_t& endBlock)  // NOLINT
    __attribute__((error("use contiguousRange(&bgnBlock, &endBlock)")));
  bool createContiguous(SdBaseFile& dirFile,  // NOLINT
    const char* path, uint32_t size)
    __attribute__((error("use createContiguous(&bgnBlock, &endBlock)")));
  static void dateTimeCallback(  // NOLINT
    void (*dateTime)(uint16_t& date, uint16_t& time))  // NOLINT
    __attribute__((error("use void dateTimeCallback("
     "void (*dateTime)(uint16_t* date, uint16_t* time))")));
  bool dirEntry(dir_t& dir)  // NOLINT
    __attribute__((error("use dirEntry(&dir)")));
  bool mkdir(SdBaseFile& dir, const char* path)  // NOLINT
    __attribute__((error("use mkdir(&dir, path)")));
  bool open(SdBaseFile& dirFile, // NOLINT
    const char* path, uint8_t oflag)
    __attribute__((error("use open(&dirFile, path, oflag)")));
  bool open(SdBaseFile& dirFile, const char* path)  // NOLINT
    __attribute__((error("use open(&dirFile, path, O_RDWR)")));
  bool open(SdBaseFile& dirFile, uint16_t index, uint8_t oflag) // NOLINT
    __attribute__((error("use open(&dirFile, index, oflag)")));
  bool openRoot(SdVolume& vol)  // NOLINT
    __attribute__((error("use openRoot(&vol)")));
  int8_t readDir(dir_t& dir)  // NOLINT
    __attribute__((error("use readDir(&dir)")));
  static bool remove(SdBaseFile& dirFile, const char* path)  // NOLINT
    __attribute__((error("use remove(&dirFile, path)")));
#endif  // ALLOW_DEPRECATED_FUNCTIONS
};
//------------------------------------------------------------------------------
/**
 * \class SdFile
 * \brief SdBaseFile with Print.
 */
class SdFile : public SdBaseFile {
 public:
  SdFile() {}
  SdFile(const char* name, uint8_t oflag);
#if DESTRUCTOR_CLOSES_FILE
  ~SdFile() {}
#endif  // DESTRUCTOR_CLOSES_FILE
  /** \return value of writeError */
  bool getWriteError() {return SdBaseFile::getWriteError();}
  /** Set writeError to zero */
  void clearWriteError() {SdBaseFile::clearWriteError();}
#ifdef COMPAT_PRE1
  void write(uint8_t b);
#else
  size_t write(uint8_t b);
#endif
  int write(const char* str);
  int write(const void* buf, size_t nbyte);
  void write_P(FSTRINGPARAM(str));
  void writeln_P(FSTRINGPARAM(str));
};
/** Store and print a string in flash memory.*/
#define PgmPrint(x) SerialPrint_P(PSTR(x))
/** Store and print a string in flash memory followed by a CR/LF.*/
#define PgmPrintln(x) SerialPrintln_P(PSTR(x))

namespace SdFatUtil {
  int FreeRam();
  void SerialPrint_P(FSTRINGPARAM(str));
  void SerialPrintln_P(FSTRINGPARAM(str));
}

using namespace SdFatUtil;  // NOLINT

//#include <SdStream.h>
//#include <ArduinoStream.h>
//------------------------------------------------------------------------------
/**
 * \class SdFat
 * \brief Integration class for the %SdFat library.
 */
class SdFat {
 public:
  SdFat() {}
#if ALLOW_DEPRECATED_FUNCTIONS && !defined(DOXYGEN)
 /**
   * Initialize an SdFat object.
   *
   * Initializes the SD card, SD volume, and root directory.
   *
   * \param[in] sckRateID value for SPI SCK rate. See Sd2Card::init().
   * \param[in] chipSelectPin SD chip select pin. See Sd2Card::init().
   *
   * \return The value one, true, is returned for success and
   * the value zero, false, is returned for failure.
   */

  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
    uint8_t chipSelectPin = SD_CHIP_SELECT_PIN) {
    return begin(chipSelectPin, sckRateID);
  }
#elif  !defined(DOXYGEN)  // ALLOW_DEPRECATED_FUNCTIONS
  bool init() __attribute__((error("use sd.begin()")));
  bool init(uint8_t sckRateID)
    __attribute__((error("use sd.begin(chipSelect, sckRate)")));
  bool init(uint8_t sckRateID, uint8_t chipSelectPin)
    __attribute__((error("use sd.begin(chipSelect, sckRate)")));
#endif  // ALLOW_DEPRECATED_FUNCTIONS
  /** \return a pointer to the Sd2Card object. */
  Sd2Card* card() {return &card_;}
  bool chdir(bool set_cwd = false);
  bool chdir(const char* path, bool set_cwd = false);
  void chvol();
  void errorHalt();
  void errorHalt_P(FSTRINGPARAM(msg));
  void errorHalt(char const *msg);
  void errorPrint();
  void errorPrint_P(FSTRINGPARAM(msg));
  void errorPrint(char const *msg);
  bool exists(const char* name);
  bool begin(uint8_t chipSelectPin = SD_CHIP_SELECT_PIN,
    uint8_t sckRateID = SPI_FULL_SPEED);
  void initErrorHalt();
  void initErrorHalt(char const *msg);
  void initErrorHalt_P(FSTRINGPARAM(msg));
  void initErrorPrint();
  void initErrorPrint(char const *msg);
  void initErrorPrint_P(FSTRINGPARAM(msg));
  void ls(uint8_t flags = 0);
  bool mkdir(const char* path, bool pFlag = true);
  bool remove(const char* path);
  bool rename(const char *oldPath, const char *newPath);
  bool rmdir(const char* path);
  bool truncate(const char* path, uint32_t length);
  /** \return a pointer to the SdVolume object. */
  SdVolume* vol() {return &vol_;}
  /** \return a pointer to the volume working directory. */
  SdBaseFile* vwd() {return &vwd_;}
  //----------------------------------------------------------------------------
  // static functions for stdOut

 private:
  Sd2Card card_;
  SdVolume vol_;
  SdBaseFile vwd_;
};
#endif  // SdFat_h
