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
*/

#include "Repetier.h"

#if NEW_FILE_HANDLING == 1

#ifndef MOUNT_DEBOUNCE_TIME_MS
#define MOUNT_DEBOUNCE_TIME_MS 5
#endif

#ifndef MOUNT_RETRIES
#define MOUNT_RETRIES 0
#endif

FileSource* fileSources[4] = { nullptr };
FilePrintManager filePrintManager;

void FilePrintManager::selectSource(FileSource* source) {
    activeSource = source;
}
void FilePrintManager::writeCommand(GCode* code) {
    if (activeSource) {
        activeSource->writeCommand(code);
    }
}

FileSource* FilePrintManager::getMountedSource(uint8_t pos) {
    if (pos < 4) {
        if (fileSources[pos] == nullptr || fileSources[pos].isMounted() == false) {
            return nullptr;
        }
        return fileSources[pos];
    }
    for (pos = 0; pos < 4; pos++) {
        if (fileSources[pos] != nullptr && fileSources[pos].isMounted()) {
            return fileSources[pos];
        }
    }
    return nullptr;
}

void FilePrintManager::handleAutomount() {
    for (int i = 0; i < 4; i++) {
        auto s = fileSources[i];
        if (s != nullptr && s->usesAutomount()) {
            s->handleAutomount();
        }
    }
}

// ------------ FileSourceSPI ---------------

template <class Detect, int cs>
FileSourceSPI::FileSourceSPI(PGM_P name, int pos, SPIClass* _spi):FileSource(name) {


}

template <class Detect, int cs>
bool FileSourceSPI::isMounted() {

}

template <class Detect, int cs>
void FileSourceSPI::handleAutomount() {
    if (!usesAutomount()) {
        return;
    }
    bool pinLevel = Detect::get();
    if (pinLevel && state == SDState::SD_UNMOUNTED) {
        if (!mountDebounceTimeMS) {
            mountDebounceTimeMS = HAL::timeInMilliseconds();
            mountRetries = 0ul;
        } else {
            if ((HAL::timeInMilliseconds() - mountDebounceTimeMS) > 250ul) {
                mount(false);
            }
        }
    } else if (!pinLevel) {
        if (state == SDState::SD_SAFE_EJECTED) {
            state = SDState::SD_UNMOUNTED;
        }
        if (state != SDState::SD_UNMOUNTED) {
            unmount(false);
        }
        mountDebounceTimeMS = 0ul;
    }
}

template <class Detect, int cs>
void FileSourceSPI::writeCommand(GCode* code) {


}
#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"

#endif