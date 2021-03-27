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
#pragma once

/**
 * RepetierSerialUSB_Due.h - Hardware Serial over USB (CDC) library for Arduino DUE
 * Copyright (c) 2017 Eduardo José Tagle. All right reserved
 * Modified for repetier 2021 Roland Littwin
 */

#include <WString.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class RepetierSerialUSB : public Stream {

public:
    RepetierSerialUSB() {};
    void begin(const long);
    void end();
    int peek() override;
    int read() override;
    void flush() override;

    int available() override;
    size_t write(const uint8_t c) override;

    size_t write(const uint8_t* buffer, size_t size) {
        size_t written = 0;
        while (size--) {
            written += write(*buffer++);
        }
        return written;
    }
};

extern RepetierSerialUSB SerialUSB;
