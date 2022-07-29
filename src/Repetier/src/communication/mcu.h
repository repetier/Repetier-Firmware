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

#pragma pack(push, 1)
union MCUDataHeader {
    uint32_t header;
    struct {
        uint8_t mcuId : 4;
        uint16_t command : 12;
        uint8_t length : 8;
        uint8_t crc : 8;
    };
};
#pragma pack(pop)

/**
 * @brief Compute J1850 style CRC8 checksums.
 * 
 * Sequence: init-> add/addArray + -> finish
 */
class CRC8_J1850 {
public:
    uint8_t crc;
    CRC8_J1850();
    void init();
    void add(uint8_t val);
    void addArray(uint8_t* data, int length);
    uint8_t finish();
};

class MCUData {
    uint8_t pos; // read/write position
    CRC8_J1850 crc;

public:
    uint8_t data[128];
    MCUDataHeader header;

    void prepareSending(uint8_t mcuId, uint16_t functionId);
    void dataComplete();
    void appendBytes(void* ptr, int length);
    void appendI8(int8_t x);
    void appendU8(uint8_t x);
    void appendI16(int16_t x);
    void appendU16(uint16_t x);
    void appendI32(int32_t x);
    void appendU32(uint32_t x);
    void appendF(float x);
    void appendS(char* s);

    bool prepareRead();
    void readBytes(void* ptr, int length);
    int8_t readI8();
    uint8_t readU8();
    int16_t readI16();
    uint16_t readU16();
    int32_t readI32();
    uint32_t readU32();
    float readFloat();
    char* readS();
};