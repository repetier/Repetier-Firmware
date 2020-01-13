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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.



    Main author: repetier
*/

/*
  Simulates EEPROM by storing data directly in flahs memory instead of a EEPROM.
  STM sais flahs can be written around 10000 times. To prevent early degradation
  we use a log like structur ethat fills flash with changes. That way with 128KB flash
  usage we can at worst case store 160 million value changes. Since only changes
  to previous values get logged to flash the solution is very efficient.

  Log structure:
  Each entry starts with 4 bytes of control structure containing 
  eeprom position, length and crc8 checksum.
  This is followed by length 4 byte uint32_t data entries.

  
*/

#include "Repetier.h"
#if defined(STM32F4_BOARD) && EEPROM_AVAILABLE == EEPROM_FLASH

#ifndef FLASH_EEPROM_START
#define FLASH_EEPROM_START (FLASH_START + FLASH_SIZE - FLASH_EEPROM_SIZE)
#endif
#define UNLOCK_FLASH() \
    HAL_FLASH_Unlock(); \
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

#define LOCK_FLASH() HAL_FLASH_Lock();
#define FLASH_MAX_IDX (EEPROM_BYTES / 4)
#define FLASH_MAX_WRITE_POS (FLASH_EEPROM_SIZE / 4)

static uint32_t flashEeprom[EEPROM_BYTES / 4];
static uint32_t* origEeprom = reinterpret_cast<uint32_t*>(HAL::virtualEeprom);

struct FlashControl {
    uint32_t address : 12, length : 12, checksum : 8;
};
union FlashControlUnion {
    FlashControl control;
    uint32_t data;
    bool isUnset() {
        return data == 0xfffffffful;
    }
};
static FlashControlUnion* flashArray = reinterpret_cast<FlashControlUnion*>(FLASH_EEPROM_START);
static uint32_t flashIndex = 0;

uint8_t FEComputeCRC8(uint8_t crc, uint8_t nextData) {
    const uint8_t generator = 0x1D;
    crc ^= nextData;

    for (int i = 0; i < 8; i++) {
        if ((crc & 0x80) != 0) {
            crc = (uint8_t)((crc << 1) ^ generator);
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

uint8_t FEComputeCRC8(uint8_t crc, uint32_t nextData) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&nextData);
    crc = FEComputeCRC8(crc, *ptr);
    ptr++;
    crc = FEComputeCRC8(crc, *ptr);
    ptr++;
    crc = FEComputeCRC8(crc, *ptr);
    ptr++;
    return FEComputeCRC8(crc, *ptr);
}

/** Clear log result */
void FEClear(void) {
    for (int i = 0; i < FLASH_MAX_IDX; i++) {
        flashEeprom[i] = 0;
    }
}

/** Erase flash content with 0xff */
void FEReset(void) {
    bool cleared = true;
    for (uint32_t i = 0; i < FLASH_MAX_WRITE_POS; i++) {
        if (flashArray[i].data != 0xfffffffful) {
            cleared = false;
            break;
        }
    }
    if (cleared) { // was cleared from upload, no need to torture flash twice
        return;
    }
    UNLOCK_FLASH();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    LOCK_FLASH();
    if (status != HAL_OK) {
        Com::printErrorF("Flash erase failed:");
        Com::print(HAL_FLASH_GetError());
        Com::println();
    }
}

void FEInit(void) {
    bool errors = false;
    uint8_t crc;
    FEClear();
    // build array from modification list
    flashIndex = 0;
    do {
        FlashControlUnion& c = flashArray[flashIndex];
        if (c.isUnset()) {
            break;
        }
        crc = 0;
        if (c.control.address >= FLASH_MAX_IDX || c.control.address + c.control.length >= FLASH_MAX_IDX) {
            errors = true; // illegal values, must be from some errors/old content
            break;
        }
        uint32_t* posWrite = &origEeprom[c.control.address];
        uint32_t* posWrite2 = &flashEeprom[c.control.address];
        flashIndex++;
        uint32_t n = c.control.length;
        while (n--) {
            crc = FEComputeCRC8(crc, flashArray[flashIndex++].data);
        }
        errors = crc != c.control.checksum;
        if (!errors) {
            n = c.control.length;
            flashIndex -= n;
            while (n--) {
                *posWrite2 = *posWrite = flashArray[flashIndex++].data;
                crc = FEComputeCRC8(crc, *posWrite);
                posWrite++;
                posWrite2++;
            }
        }
    } while (!errors && flashIndex < FLASH_MAX_WRITE_POS);
    if (!errors && flashIndex + 1 < FLASH_MAX_IDX && !flashArray[flashIndex + 1].isUnset()) {
        errors = true;
    }
    if (errors) {
        // first usage or something has gone bad on updating
        // delete the flash and reinit as if it was first usage
        FEReset();
        FEClear();
        flashIndex = 0;
        if (flashIndex < 100) { // not initialized
            for (int i = 0; i < FLASH_MAX_IDX; i++) {
                origEeprom[i] = 0;
            }
        } else { // last write failed, reinit with previous values
            FEUpdateChanges();
        }
    }
}

/**
 Compares origEeprom with flashEeprom and saves differences to flash
 */
void FEUpdateChanges(void) {
    HAL_StatusTypeDef status;
    UNLOCK_FLASH();
    for (int start = 0; start < FLASH_MAX_IDX;) {
        if (origEeprom[start] == flashEeprom[start]) {
            start++;
            continue;
        }
        FlashControlUnion entry { .data = 0 };
        entry.control.address = start;
        uint32_t end = start;
        uint8_t crc = 0;
        while (origEeprom[end] != flashEeprom[end]) {
            crc = FEComputeCRC8(crc, origEeprom[end]);
            end++;
        }
        entry.control.checksum = crc;
        entry.control.length = end - start;
        if (flashIndex + entry.control.length + 1 >= FLASH_MAX_WRITE_POS) { // flash full, start at beginning
            Com::printInfoFLN("EEPROM flash full - restarting");
            LOCK_FLASH();
            FEReset();
            FEClear();
            flashIndex = 0;
            FEUpdateChanges();
            return;
        }
        // write data first and then control data so we fail on tests if we loose power during write
        for (int i = 0; i < entry.control.length; i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_EEPROM_START + 4 * (flashIndex + 1 + i), origEeprom[start + i]);
            if (status != HAL_OK) {
                Com::printErrorF("Flash write failed:");
                Com::print(HAL_FLASH_GetError());
                Com::println();
                LOCK_FLASH();
                return;
            }
            flashEeprom[start + i] = origEeprom[start + i];
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_EEPROM_START + 4 * flashIndex, entry.data);
        if (status != HAL_OK) {
            Com::printErrorF("Flash write failed:");
            Com::print(HAL_FLASH_GetError());
            Com::println();
            LOCK_FLASH();
            return;
        }
        flashIndex += entry.control.length + 1;
        start = end;
    }
    LOCK_FLASH();
}

#endif
