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
char tempLongFilename[LONG_FILENAME_LENGTH + 1u];
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1u];
SDCardGCodeSource sdSource;

template <typename T>
bool doForDirectory(sd_file_t& dir, T&& action, const bool recursive = false, size_t depth = 0) {
    if (!dir.isDir()) {
        return false;
    }
    sd_file_t file;
    dir.rewind();
    while (file.openNext(&dir)) {
        HAL::pingWatchdog();
        if (!action(file, dir, depth)) {
            file.close();
            return false;
        }
        if (recursive && file.isDir() && (depth + 1u) < SD_MAX_FOLDER_DEPTH) {
            depth++;
            if (!doForDirectory(file, action, recursive, depth)) {
                file.close();
                return false;
            }
            depth--;
        }
        file.close();
    }
    return true;
}
// Shorter than writing getname(templongfilename, sizeof(templongfilename)) etc
// Will always inline return pointer to either tempLongFilename or the input buffer.
template <size_t N>
inline char* getFN(sd_file_t* file, char (&buf)[N]) {
    file->getName(buf, N);
    return buf;
}
template <size_t N>
inline char* getFN(sd_file_t& file, char (&buf)[N]) {
    file.getName(buf, N);
    return buf;
}
inline char* getFN(sd_file_t* file, char* buf, const uint8_t size) {
    file->getName(buf, size);
    return buf;
}
inline char* getFN(sd_file_t& file, char* buf, const uint8_t size) {
    file.getName(buf, size);
    return buf;
}
inline char* getFN(sd_file_t* file) {
    return getFN(file, tempLongFilename);
}
inline char* getFN(sd_file_t& file) {
    return getFN(file, tempLongFilename);
}

FilePrintManager::FilePrintManager() {
    flags = 0;
    selectedSource = nullptr;
    scheduledPause = SDScheduledPause::NO_PAUSE;
    scheduledStop = false;
    printingSilent = false;
}

void FilePrintManager::selectSource(FileSource* source) {
    selectedSource = source;
}

void FilePrintManager::setPrinting(bool printing) {
    if (printing) {
        flags |= 1;
    } else {
        flags &= ~1;
    }
}

void FilePrintManager::setWriting(bool writing) {
    if (writing) {
        flags |= 2;
    } else {
        flags &= ~2;
    }
}

int FilePrintManager::posFor(FileSource* s) {
    for (int i = 0; i < 4; i++) {
        if (fileSources[i] == s) {
            return i;
        }
    }
    return 255;
}

void FilePrintManager::mountAll() {
    for (int i = 0; i < 4; i++) {
        if (fileSources[i]) {
            fileSources[i]->mount(false);
            if (fileSources[i]->isMounted()) {
                selectedSource = fileSources[i];
                break;
            }
        }
    }
}

void FilePrintManager::unmount(bool manual, int pos) {
    if (pos < 4) {
        FileSource* s = fileSources[pos];
        if (s == nullptr) {
            return;
        }
        if (s != selectedSource) {
            s->unmount(manual);
            return;
        }
    }
    if (selectedSource) {
        selectedSource->unmount(manual);
        selectedSource = nullptr;
    }
}

void FilePrintManager::mount(bool manual, int pos) {
    ///< pos = 255 for first available
    // Com::printFLN(PSTR("Mounting device "), static_cast<int32_t>(pos));
    if (isPrinting() || isWriting()) {
        if (getMountedSource(pos) == selectedSource) {
            Com::printWarningFLN(PSTR("SD mount skipped! Active device still busy."));
            return;
        }
    }
    if (pos < 4) {
        if (fileSources[pos] == nullptr) {
            Com::printWarningFLN(PSTR("SD mount skipped! Non existing device."));
            return;
        }
        fileSources[pos]->mount(manual);
        if (fileSources[pos]->isMounted() && (manual || selectedSource == nullptr || !selectedSource->isMounted())) {
            selectedSource = fileSources[pos];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (fileSources[i]) {
                fileSources[i]->mount(manual);
                if (fileSources[i]->isMounted() && (manual || selectedSource == nullptr || !selectedSource->isMounted())) {
                    selectedSource = fileSources[i];
                    break;
                }
            }
        }
    }
}

void FilePrintManager::startWrite(const char* filename) {
    if (selectedSource == nullptr) {
        return;
    }
    if (selectedSource->isMounted() == false) {
        return;
    }
    if (selectedSource->startWrite(filename)) {
        setWriting(true);
        writtenBytes = 0ul;
        GUI::setStatusP(PSTR("Receiving file..."), GUIStatusLevel::BUSY);
        Com::printFLN(Com::tWritingToFile, filename);
        lastWriteTimeMS = HAL::timeInMilliseconds();
        Com::printFLN(Com::tOk); // Needed for octoprint
    } else {
        Com::printFLN(Com::tOpenFailedFile, filename);
    }
}

void FilePrintManager::writeCommand(GCode* code) {
    if (selectedSource) {
        unsigned int sum1 = 0u, sum2 = 0u; // for fletcher-16 checksum
        uint8_t buf[100u];
        uint8_t p = 2u;
        uint16_t params = 128u | (code->params & ~1);
        memcopy2(buf, &params);
        if (code->isV2()) { // Read G,M as 16 bit value
            memcopy2(&buf[p], &code->params2);
            p += 2u;
            if (code->hasString()) {
                buf[p++] = strlen(code->text);
            }
            if (code->hasM()) {
                memcopy2(&buf[p], &code->M);
                p += 2u;
            }
            if (code->hasG()) {
                memcopy2(&buf[p], &code->G);
                p += 2u;
            }
        } else {
            if (code->hasM()) {
                buf[p++] = (uint8_t)code->M;
            }
            if (code->hasG()) {
                buf[p++] = (uint8_t)code->G;
            }
        }
        if (code->hasX()) {
            memcopy4(&buf[p], &code->X);
            p += 4u;
        }
        if (code->hasY()) {
            memcopy4(&buf[p], &code->Y);
            p += 4u;
        }
        if (code->hasZ()) {
            memcopy4(&buf[p], &code->Z);
            p += 4u;
        }
        if (code->hasE()) {
            memcopy4(&buf[p], &code->E);
            p += 4u;
        }
        if (code->hasF()) {
            memcopy4(&buf[p], &code->F);
            p += 4u;
        }
        if (code->hasT()) {
            buf[p++] = code->T;
        }
        if (code->hasS()) {
            memcopy4(&buf[p], &code->S);
            p += 4u;
        }
        if (code->hasP()) {
            memcopy4(&buf[p], &code->P);
            p += 4u;
        }
        if (code->hasI()) {
            memcopy4(&buf[p], &code->I);
            p += 4u;
        }
        if (code->hasJ()) {
            memcopy4(&buf[p], &code->J);
            p += 4u;
        }
        if (code->hasR()) {
            memcopy4(&buf[p], &code->R);
            p += 4u;
        }
        if (code->hasD()) {
            memcopy4(&buf[p], &code->D);
            p += 4u;
        }
        if (code->hasC()) {
            memcopy4(&buf[p], &code->C);
            p += 4u;
        }
        if (code->hasH()) {
            memcopy4(&buf[p], &code->H);
            p += 4u;
        }
        if (code->hasA()) {
            memcopy4(&buf[p], &code->A);
            p += 4u;
        }
        if (code->hasB()) {
            memcopy4(&buf[p], &code->B);
            p += 4u;
        }
        if (code->hasK()) {
            memcopy4(&buf[p], &code->K);
            p += 4u;
        }
        if (code->hasL()) {
            memcopy4(&buf[p], &code->L);
            p += 4u;
        }
        if (code->hasO()) {
            memcopy4(&buf[p], &code->O);
            p += 4u;
        }
        if (code->hasU()) {
            memcopy4(&buf[p], &code->U);
            p += 4u;
        }
        if (code->hasV()) {
            memcopy4(&buf[p], &code->V);
            p += 4u;
        }
        if (code->hasW()) {
            memcopy4(&buf[p], &code->W);
            p += 4u;
        }
        if (code->hasString()) { // read 16 uint8_t into string
            char* sp = code->text;
            if (code->isV2()) {
                uint8_t i = strlen(code->text);
                for (; i; i--) {
                    buf[p++] = *sp++;
                }
            } else {
                for (uint8_t i = 0u; i < 16u; ++i) {
                    buf[p++] = *sp++;
                }
            }
        }
        uint8_t* ptr = buf;
        uint8_t len = p;
        while (len) {
            uint8_t tlen = len > 21u ? 21u : len;
            len -= tlen;
            do {
                sum1 += *ptr++;
                if (sum1 >= 255) {
                    sum1 -= 255;
                }
                sum2 += sum1;
                if (sum2 >= 255) {
                    sum2 -= 255;
                }
            } while (--tlen);
        }
        buf[p++] = sum1;
        buf[p++] = sum2;
        if (params == 128u) { // Todo: What is this?
            Com::printErrorFLN(Com::tAPIDFinished);
            return;
        }

        if (!selectedSource->write(reinterpret_cast<char*>(buf), p)) {
            Com::printFLN(Com::tErrorWritingToFile);
        }
        if ((HAL::timeInMilliseconds() - lastWriteTimeMS) > 1000ul && writtenBytes) {
            if (GUI::statusLevel == GUIStatusLevel::BUSY) {
                float kB = writtenBytes / 1000.0f;
                GUI::flashToStringFloat(GUI::status, PSTR("Received @ B"), kB > 1000.0f ? kB / 1000.0f : kB, 1);
                size_t len = strlen(GUI::status);
                GUI::status[len - 2] = kB > 1000.0f ? 'M' : 'k';
                static float lastKBytes = 0.0f;
                if (kB < lastKBytes) {
                    lastKBytes = 0.0f;
                }
                GUI::flashToStringFloat(GUI::tmpString, PSTR("\n @kB/s"), kB - lastKBytes, 1);
                lastKBytes = kB;
                strncat(GUI::status, GUI::tmpString, sizeof(GUI::status) - len);
            }
            lastWriteTimeMS = HAL::timeInMilliseconds();
        }
    }
}

void FilePrintManager::finishWriting() {
    if (!isWriting()) {
        return;
    }
    selectedSource->finishWriting();
    Com::printFLN(Com::tDoneSavingFile);
    GUI::pop();
    setWriting(false);
}

int FilePrintManager::read() {
    if (isPrinting()) {
        return selectedSource->read();
    }
    return -1;
}

void FilePrintManager::setPosition(uint32_t newPos) {
    if (selectedSource) {
        selectedSource->setPosition(newPos);
    }
}

void FilePrintManager::printCardInfo(bool json) {
    if (selectedSource) {
        selectedSource->printCardInfo(json);
    }
}

int8_t rfstrncasecmp_P(const char* s1, PGM_P s2, size_t n) {
    if (!n) {
        return 0;
    }
    int8_t res = 0;
    do {
        char c = tolower(pgm_read_byte(s2));
        res = tolower(*s1++) - c;
        if (res || !c) {
            break;
        }
        s2++;
    } while (--n);
    return res;
}

bool FilePrintManager::validGCodeExtension(const char* filename) {
    if (filename[0u] != '\0'
        && strlen(filename) <= LONG_FILENAME_LENGTH) {
        char* extPtr = strrchr(filename, '.');
        if (extPtr
            && (!rfstrncasecmp_P(extPtr, PSTR(".gcode"), 7u)
                || !rfstrncasecmp_P(extPtr, PSTR(".gco"), 5u)
                || !rfstrncasecmp_P(extPtr, PSTR(".gc"), 4u)
                || !rfstrncasecmp_P(extPtr, PSTR(".g"), 3u)
                || !rfstrncasecmp_P(extPtr, PSTR(".nc"), 4u))) {
            return true;
        }
    }
    return false;
}

bool FilePrintManager::startRead(const char* filename, const bool silent) {
    if (!validGCodeExtension(filename)) {
        if (!silent) {
            Com::printFLN(Com::tInvalidFiletype);
        }
        return false;
    }
    if (!selectedSource || !selectedSource->isMounted()) {
        if (!silent) {
            Com::printFLN(Com::tNoMountedCard);
        }
        return false;
    }
    bool ret = selectedSource->startRead(filename, silent);
    if (ret) {
        setPrinting(true);
        Printer::maxLayer = -1;
        selectedFilePos = 0ul;
        printingSilent = silent;
        if (!silent) {
            Com::printF(Com::tFileOpened, filename);
            Com::printFLN(Com::tSpaceSizeColon, selectedFileSize);
            Com::printFLN(Com::tFileSelected);
        }
    }
    return ret;
}

void FilePrintManager::finishPrint() {
    if (!isPrinting()) {
        return;
    }
    GCodeSource::removeSource(&sdSource);
    selectedSource->close();
    setPrinting(false);
    Printer::setPrinting(false);
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, false);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    if (!printingSilent) {
        Com::writeToAll = true; // tell all listeners that we are finished
        Com::printFLN(Com::tDonePrinting);
    }
    printingSilent = false;
}

#if JSON_OUTPUT
void FilePrintManager::lsJSON(const char* filename) {
    if (selectedSource) {
        selectedSource->lsJSON(filename);
    }
}

void FilePrintManager::JSONFileInfo(const char* filename) {
    if (selectedSource) {
        selectedSource->JSONFileInfo(filename);
    }
}
#endif

void FilePrintManager::ls(const char* lsDir, const bool json) {
    if (selectedSource) {
        selectedSource->ls(lsDir, json);
    }
}

FileSource* FilePrintManager::getMountedSource(uint8_t pos) {
    if (pos < 4) {
        if (fileSources[pos] == nullptr || fileSources[pos]->isMounted() == false) {
            return nullptr;
        }
        return fileSources[pos];
    }
    for (pos = 0; pos < 4; pos++) {
        if (fileSources[pos] != nullptr && fileSources[pos]->isMounted()) {
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

bool FilePrintManager::writingTimedOut(millis_t time) {
    return (time - lastWriteTimeMS) > (5ul * 60000ul);
}

void FilePrintManager::deleteFile(const char* filename) {
    if (selectedSource) {
        selectedSource->deleteFile(filename);
    }
}

void FilePrintManager::makeDirectory(const char* filename) {
    if (selectedSource) {
        selectedSource->makeDirectory(filename);
    }
}

void FilePrintManager::printStatus(const bool getFilename) {
    if (getFilename) {
        Com::printF(Com::tCurrentOpenFile);
        if (isPrinting()) {
            Com::printFLN(Printer::printName);
        } else {
            Com::printFLN(PSTR("(no file)"));
        }
    } else {
        if (isPrinting()) {
            Com::printF(Com::tSDPrintingByte, selectedFilePos);
            Com::printFLN(Com::tSlash, selectedFileSize);
        } else if (Printer::isMenuMode(MENU_MODE_PAUSED)) {
            Com::printFLN(Com::tSDPrintingPaused);
        } else {
            Com::printFLN(Com::tNotSDPrinting);
        }
    }
}

void FilePrintManager::startPrint() {
    if (!selectedSource || selectedSource->isMounted() == false
        || !selectedSource->isFileOpened()
        || Printer::failedMode) {
        return;
    }
    scheduledStop = false;
    scheduledPause = SDScheduledPause::NO_PAUSE;
    setPrinting(true);
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    Printer::setPrinting(true);
    Printer::maxLayer = -1;
    Printer::currentLayer = 0;
    GUI::clearStatus();
    GCodeSource::registerSource(&sdSource);
}

void FilePrintManager::pausePrint(const bool internal) {
    if (isPrinting() || Printer::isMenuMode(MENU_MODE_PAUSED)) {
        return;
    }
    Printer::setMenuMode(MENU_MODE_PAUSED, true);
#if !defined(DISABLE_PRINTMODE_ON_PAUSE) || DISABLE_PRINTMODE_ON_PAUSE == 1
    Printer::setPrinting(false);
#endif
    GCodeSource::removeSource(&sdSource);
    scheduledPause = internal ? SDScheduledPause::PARKING_PLANNED : SDScheduledPause::NO_PAUSE;
    if (!internal) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
        EVENT_SD_PAUSE_START(internal);
        EVENT_SD_PAUSE_END(internal);
#pragma GCC diagnostic pop
    }
}

void FilePrintManager::printFullyPaused() {
    if (scheduledPause != SDScheduledPause::PARKING_PLANNED) {
        return;
    }
    scheduledPause = SDScheduledPause::NO_PAUSE;
    if (EVENT_SD_PAUSE_START(false)) {
        Commands::waitUntilEndOfAllBuffers();
        Motion1::pushToMemory();
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                            Motion1::currentPosition[E_AXIS] - RETRACT_ON_PAUSE,
                            Motion1::maxFeedrate[E_AXIS] / 2.0f);
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            tool->afterPause();
        }
        Motion1::moveToParkPosition();
        GCode::executeFString(PSTR(PAUSE_START_COMMANDS));
        scheduledPause = SDScheduledPause::PARKED;
    }
    EVENT_SD_PAUSE_END(false);
}

void FilePrintManager::continuePrint() {
    if (!isPrinting() || !Printer::isMenuMode(MENU_MODE_PAUSED)) {
        return;
    }
    GUI::setStatusP(PSTR("Continuing..."), GUIStatusLevel::BUSY);
    MCode_513(nullptr); // Reset jam marker
    if (EVENT_SD_CONTINUE_START(scheduledPause)) {
        if (scheduledPause == SDScheduledPause::PARKED) {
            Tool* tool = Tool::getActiveTool();
            if (tool) {
                tool->beforeContinue();
            }
            // Com::printFLN(PSTR("Before M601"));
            GCode::executeFString(PSTR(PAUSE_END_COMMANDS));
            // Com::printFLN(PSTR("After M601"));
            float pos[NUM_AXES];
            if (Motion1::popFromMemory(pos)) {
                float z = pos[Z_AXIS];
                float e = pos[E_AXIS];
                pos[Z_AXIS] = IGNORE_COORDINATE;
                pos[E_AXIS] = IGNORE_COORDINATE;
                Motion1::moveByOfficial(pos, Motion1::moveFeedrate[X_AXIS], false);
                FOR_ALL_AXES(i) {
                    pos[i] = IGNORE_COORDINATE;
                }
                pos[Z_AXIS] = z;
                Motion1::moveByOfficial(pos, Motion1::moveFeedrate[Z_AXIS], false);
                pos[E_AXIS] = e;
                Motion1::moveByOfficial(pos, Motion1::maxFeedrate[E_AXIS], false);
            }
            /*
            Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE);
            Motion1::popFromMemory();
            Motion1::pushToMemory();
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[X_AXIS], false);
            Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE);
            Motion1::popFromMemory();
            Motion1::pushToMemory();
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[Z_AXIS], false);
            Motion1::setTmpPositionXYZE(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE);
            Motion1::popFromMemory();
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[E_AXIS], false); */
        }
    }
    EVENT_SD_CONTINUE_END(scheduledPause);
    GCodeSource::registerSource(&sdSource);
    Printer::setPrinting(true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    scheduledPause = SDScheduledPause::NO_PAUSE;
    GUI::pop();
    GUI::setStatusP(Com::tEmpty, GUIStatusLevel::REGULAR);
}

void FilePrintManager::stopPrint(const bool silent) {
    if (!isPrinting() || scheduledStop || !selectedSource) {
        return;
    }
    scheduledStop = true;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED, false);
    Printer::setPrinting(false);
    selectedSource->close();
    if (!silent) {
        Com::printFLN(PSTR("SD print stopped by user."));
        GUI::clearStatus();
    }
    Printer::breakLongCommand = true; // stop waiting heatup if running
}

void FilePrintManager::printFullyStopped() {
    if (!scheduledStop || !selectedSource) {
        return;
    }
    scheduledStop = false;
    Motion1::moveToParkPosition();
    GCodeSource::removeSource(&sdSource);
    selectedSource->close();
    if (EVENT_SD_STOP_START) {
        GCode::executeFString(PSTR(SD_RUN_ON_STOP));
        if (SD_STOP_HEATER_AND_MOTORS_ON_STOP) {
            Commands::waitUntilEndOfAllMoves();
            Printer::kill(false, true);
        }
    }
    EVENT_SD_STOP_END;
}

// ------------ FileSource -------------

FileSource::FileSource(PGM_P _identifier)
    : identifier(_identifier) {
}

void FileSource::printEscapeChars(const char* s) {
    const size_t len = strlen(s);
    for (size_t i = 0; i < len; ++i) {
        switch (s[i]) {
        case '"':
        case '/':
        case '\b':
        case '\f':
        case '\n':
        case '\r':
        case '\t':
        case '\\':
            Com::print('\\');
            break;
        }
        Com::print(s[i]);
    }
}

// ------------ FileSourceSPI ---------------

template <class Detect, int cs>
FileSourceSPI<Detect, cs>::FileSourceSPI(PGM_P name, int pos, SPIClass* _spi, uint32_t speed)
    : FileSourceSdFatBase(name, pos)
    , spiConfig(cs, SHARED_SPI, SD_SCK_MHZ(speed), _spi) {
    pinMode(cs, OUTPUT);
    WRITE_VAR(cs, true); // deactivated by default
    this->speed = speed;
    //SdSpiConfig spiConfig(cs, SHARED_SPI, SD_SCK_MHZ(speed), _spi);
    //card.begin(spiConfig);
}

template <class Detect, int cs>
void FileSourceSPI<Detect, cs>::mount(const bool manual) {
    if (state > SDState::SD_SAFE_EJECTED) {
        if (manual && state >= SDState::SD_MOUNTED) {
            // Some hosts (BTT TFT's) require a response after M21 or they'll hang.
            Com::writeToAll = true;
            Com::printFLN(PSTR("SD card ok"));
        }
        return;
    } else {
        Com::printFLN(PSTR("SD card already mounted"));
    }

    if (Detect::pin() != 255 && !Detect::get()) {
        return;
    }

    // if (!fileSystem.begin(spiConfig)) {
    card.begin(spiConfig);
    if (!fileSystem.begin(&card)) {
        if (mountRetries < 3u) {
            mountRetries++;
            if (manual) {
                mount(true); // Try recursively remounting 3 times if manually mounted
            }
            return;
        }
        state = SDState::SD_HAS_ERROR;
        mountRetries = 0ul;
        Com::printFLN(Com::tSDInitFail);
        UI_STATUS_UPD("SD Card read error.");
        if (printIfCardErrCode()) {
            if (fileSystem.card()->errorCode() == SD_CARD_ERROR_CMD0) {
                Com::printFLN(PSTR("Card reset failed, check chip select pin (SDSS)."));
            }
            if (Detect::pin() != 255) {
                HAL::delayMilliseconds(35ul); // wait a little more before reporting the pin state
                Com::printF(PSTR("Card detect pin:"));
                Com::printFLN(Detect::get() ? Com::tH : Com::tL);
            }
        } else if (!fileSystem.fatType()) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32/exFAT partition."));
        } else if (!fileSystem.chdir()) {
            Com::printFLN(PSTR("Can't open root directory."));
        }
        return;
    }

    mountRetries = 0ul;
    state = SDState::SD_MOUNTED;
    Com::printFLN(PSTR("SD card ok"));
    Com::printFLN(PSTR("SD card inserted"));

    filePrintManager.printCardInfo(false); // <- collects our volumeLabel too.

    if (!volumeLabel[0u] || strncmp_P(volumeLabel, PSTR("NO NAME"), sizeof(volumeLabel)) == 0u) {
        volumeLabel[0u] = '\0'; // There's no volume label at all.
        UI_STATUS_UPD("SD Card mounted.");
    } else {
        GUI::flashToStringString(GUI::tmpString, PSTR("@ mounted."), volumeLabel);
        UI_STATUS_UPD_RAM(GUI::tmpString);
    }

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED, true);
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::importEEPROM();
#endif
}

template <class Detect, int cs>
void FileSourceSPI<Detect, cs>::printCardInfo(bool json) {
    if (state < SDState::SD_MOUNTED) {
        if (!json) {
            if (state != SDState::SD_HAS_ERROR) {
                Com::printFLN(Com::tNoMountedCard);
            } else {
                Com::printFLN(Com::tSDInitFail);
            }
        } else {
#if JSON_OUTPUT
            Com::printF(Com::tJSONSDInfo);
            Com::printFLN(PSTR("{\"slot\":0,\"present\":0}}"));
#endif
        }
        return;
    }
    // Print out some basic statistics on successful mount. We also store the volume label.
    uint64_t volumeSectors = 0ul, usageBytes = 0ul;
    uint16_t fileCount = 0u;
    uint8_t folderCount = 0u;
    getCardInfo(volumeLabel, sizeof(volumeLabel), &volumeSectors, &usageBytes, &fileCount, &folderCount);

    if (!json) {
        Com::printF(PSTR("Label: "), volumeLabel);
        Com::printF(PSTR(" | "));

        if (fileSystem.fatType() == FAT_TYPE_EXFAT) {
            Com::printF(PSTR("exFAT"));
        } else {
            Com::printF(PSTR("FAT"), fileSystem.fatType());
        }

        Com::printF(PSTR(" SD"));
        uint8_t type = fileSystem.card()->type();
        if (type == SD_CARD_TYPE_SD1) {
            Com::printF(PSTR("V1"));
        } else if (type == SD_CARD_TYPE_SD2) {
            Com::printF(PSTR("V2"));
        } else if (type == SD_CARD_TYPE_SDHC) {
            if (fileSystem.sectorsPerCluster() > 64ul) {
                Com::printF(PSTR("XC")); // Cards > 32gb are XC.
            } else {
                Com::printF(PSTR("HC"));
            }
        }

        // Print out volume size
        bool gb = false;
        float size = ((0.000001f * 512.0f) * static_cast<float>(volumeSectors));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.f;
        }
        Com::printF(PSTR(" | Volume Size: "), size);
        Com::printF(gb ? PSTR(" GB") : PSTR(" MB"));
        // Print out current estimated usage
        gb = false;
        size = (0.000001f * static_cast<float>(usageBytes));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.0f;
        }
        Com::printF(PSTR(" | Usage: "), size);
        Com::printF(gb ? PSTR(" GB (") : PSTR(" MB ("), static_cast<int32_t>(fileCount));
        Com::printF(PSTR(" files, "), folderCount);
        Com::printFLN(PSTR(" folders found.)"));
    } else {
#if JSON_OUTPUT
        //{"SDinfo":{"slot":0,"present":1,"capacity":4294967296,"free":2147485184,"speed":20971520,"clsize":32768}}

        uint64_t sizeBytes = 512ull * volumeSectors;
        Com::printF(Com::tJSONSDInfo);
        Com::printF(PSTR("{\"slot\":0,\"present\":1,\"capacity\":"), sizeBytes);
        Com::printF(PSTR(",\"free\":"), (sizeBytes - usageBytes));
        Com::printF(PSTR(",\"speed\":"), (SD_SCK_MHZ(constrain(speed, 1ul, 50ul)) / 8ul));
        Com::printF(PSTR(",\"clsize\":"), static_cast<uint32_t>(fileSystem.sectorsPerCluster() * 512ul)); // bytesPerCluster isn't working properly for some reason.
        // Extra
        if (volumeLabel[0]) {
            Com::printF(PSTR(",\"label\":\""));
            FileSource::printEscapeChars(volumeLabel);
            Com::print('"');
        }
        Com::printF(PSTR(",\"files\":"), fileCount);
        Com::printF(PSTR(",\"folders\":"), folderCount);
        Com::printFLN(PSTR("}}"));
#endif
    }
}

template <class Detect, int cs>
void FileSourceSPI<Detect, cs>::handleAutomount() {
    if (!usesAutomount()) {
        return;
    }
    bool pinLevel = Detect::get();
    if (pinLevel && state == SDState::SD_UNMOUNTED) {
        Com::writeToAll = true; // tell all listeners that we are finished
        if (!mountDebounceTimeMS) {
            mountDebounceTimeMS = HAL::timeInMilliseconds();
            mountRetries = 0ul;
        } else {
            if ((HAL::timeInMilliseconds() - mountDebounceTimeMS) > 250ul) {
                filePrintManager.mount(false, filePrintManager.posFor(this));
            }
        }
    } else if (!pinLevel) {
        if (state == SDState::SD_SAFE_EJECTED) {
            state = SDState::SD_UNMOUNTED;
        }
        if (state != SDState::SD_UNMOUNTED) {
            Com::writeToAll = true; // tell all listeners that we are finished
            filePrintManager.unmount(false, filePrintManager.posFor(this));
        }
        mountDebounceTimeMS = 0ul;
    }
}

// ------------- FileSourceSDIO --------------

#if defined(HAS_SDIO_SD_SLOT) && HAS_SDIO_SD_SLOT == 1

template <class Detect>
FileSourceSDIO<Detect>::FileSourceSDIO(PGM_P name, int pos)
    : FileSourceSdFatBase(name, pos) {
    //SdSpiConfig spiConfig(cs, SHARED_SPI, SD_SCK_MHZ(speed), _spi);
    //card.begin(spiConfig);
}

template <class Detect>
void FileSourceSDIO<Detect>::mount(const bool manual) {
    if (state > SDState::SD_SAFE_EJECTED) {
        if (manual && state >= SDState::SD_MOUNTED) {
            // Some hosts (BTT TFT's) require a response after M21 or they'll hang.
            Com::writeToAll = true;
            Com::printFLN(PSTR("SD card ok"));
        }
        return;
    } else {
        Com::printFLN(PSTR("SD card already mounted"));
    }

    if (Detect::pin() != 255 && !Detect::get()) {
        return;
    }

    if (!card.init()) {
        if (mountRetries < 3u) {
            mountRetries++;
            if (manual) {
                mount(true); // Try recursively remounting 3 times if manually mounted
            }
            return;
        }
    }
    if (!fileSystem.begin(&card)) {
        if (mountRetries < 3u) {
            mountRetries++;
            if (manual) {
                mount(true); // Try recursively remounting 3 times if manually mounted
            }
            return;
        }
        state = SDState::SD_HAS_ERROR;
        mountRetries = 0ul;
        Com::printFLN(Com::tSDInitFail);
        UI_STATUS_UPD("SD Card read error.");
        if (printIfCardErrCode()) {
            if (fileSystem.card()->errorCode() == SD_CARD_ERROR_CMD0) {
                Com::printFLN(PSTR("Card reset failed, check chip select pin (SDSS)."));
            }
            if (Detect::pin() != 255) {
                HAL::delayMilliseconds(35ul); // wait a little more before reporting the pin state
                Com::printF(PSTR("Card detect pin:"));
                Com::printFLN(Detect::get() ? Com::tH : Com::tL);
            }
        } else if (!fileSystem.fatType()) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32/exFAT partition."));
        } else if (!fileSystem.chdir()) {
            Com::printFLN(PSTR("Can't open root directory."));
        }
        return;
    }

    mountRetries = 0ul;
    state = SDState::SD_MOUNTED;
    Com::printFLN(PSTR("SD card ok"));
    Com::printFLN(PSTR("SD card inserted"));

    filePrintManager.printCardInfo(false); // <- collects our volumeLabel too.

    if (!volumeLabel[0u] || strncmp_P(volumeLabel, PSTR("NO NAME"), sizeof(volumeLabel)) == 0u) {
        volumeLabel[0u] = '\0'; // There's no volume label at all.
        UI_STATUS_UPD("SD Card mounted.");
    } else {
        GUI::flashToStringString(GUI::tmpString, PSTR("@ mounted."), volumeLabel);
        UI_STATUS_UPD_RAM(GUI::tmpString);
    }

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED, true);
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::importEEPROM();
#endif
}

template <class Detect>
void FileSourceSDIO<Detect>::printCardInfo(bool json) {
    if (state < SDState::SD_MOUNTED) {
        if (!json) {
            if (state != SDState::SD_HAS_ERROR) {
                Com::printFLN(Com::tNoMountedCard);
            } else {
                Com::printFLN(Com::tSDInitFail);
            }
        } else {
#if JSON_OUTPUT
            Com::printF(Com::tJSONSDInfo);
            Com::printFLN(PSTR("{\"slot\":0,\"present\":0}}"));
#endif
        }
        return;
    }
    // Print out some basic statistics on successful mount. We also store the volume label.
    uint64_t volumeSectors = 0ul, usageBytes = 0ul;
    uint16_t fileCount = 0u;
    uint8_t folderCount = 0u;
    getCardInfo(volumeLabel, sizeof(volumeLabel), &volumeSectors, &usageBytes, &fileCount, &folderCount);

    if (!json) {
        Com::printF(PSTR("Label: "), volumeLabel);
        Com::printF(PSTR(" | "));

        if (fileSystem.fatType() == FAT_TYPE_EXFAT) {
            Com::printF(PSTR("exFAT"));
        } else {
            Com::printF(PSTR("FAT"), fileSystem.fatType());
        }

        Com::printF(PSTR(" SD"));
        uint8_t type = fileSystem.card()->type();
        if (type == SD_CARD_TYPE_SD1) {
            Com::printF(PSTR("V1"));
        } else if (type == SD_CARD_TYPE_SD2) {
            Com::printF(PSTR("V2"));
        } else if (type == SD_CARD_TYPE_SDHC) {
            if (fileSystem.sectorsPerCluster() > 64ul) {
                Com::printF(PSTR("XC")); // Cards > 32gb are XC.
            } else {
                Com::printF(PSTR("HC"));
            }
        }

        // Print out volume size
        bool gb = false;
        float size = ((0.000001f * 512.0f) * static_cast<float>(volumeSectors));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.f;
        }
        Com::printF(PSTR(" | Volume Size: "), size);
        Com::printF(gb ? PSTR(" GB") : PSTR(" MB"));
        // Print out current estimated usage
        gb = false;
        size = (0.000001f * static_cast<float>(usageBytes));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.0f;
        }
        Com::printF(PSTR(" | Usage: "), size);
        Com::printF(gb ? PSTR(" GB (") : PSTR(" MB ("), static_cast<int32_t>(fileCount));
        Com::printF(PSTR(" files, "), folderCount);
        Com::printFLN(PSTR(" folders found.)"));
    } else {
#if JSON_OUTPUT
        //{"SDinfo":{"slot":0,"present":1,"capacity":4294967296,"free":2147485184,"speed":20971520,"clsize":32768}}

        uint64_t sizeBytes = 512ull * volumeSectors;
        Com::printF(Com::tJSONSDInfo);
        Com::printF(PSTR("{\"slot\":0,\"present\":1,\"capacity\":"), sizeBytes);
        Com::printF(PSTR(",\"free\":"), (sizeBytes - usageBytes));
        Com::printF(PSTR(",\"speed\":"), (SD_SCK_MHZ(constrain(18, 1ul, 50ul)) / 8ul));
        Com::printF(PSTR(",\"clsize\":"), static_cast<uint32_t>(fileSystem.sectorsPerCluster() * 512ul)); // bytesPerCluster isn't working properly for some reason.
        // Extra
        if (volumeLabel[0]) {
            Com::printF(PSTR(",\"label\":\""));
            FileSource::printEscapeChars(volumeLabel);
            Com::print('"');
        }
        Com::printF(PSTR(",\"files\":"), fileCount);
        Com::printF(PSTR(",\"folders\":"), folderCount);
        Com::printFLN(PSTR("}}"));
#endif
    }
}

template <class Detect>
void FileSourceSDIO<Detect>::handleAutomount() {
    if (!usesAutomount()) {
        return;
    }
    bool pinLevel = Detect::get();
    if (pinLevel && state == SDState::SD_UNMOUNTED) {
        Com::writeToAll = true; // tell all listeners that we are finished
        if (!mountDebounceTimeMS) {
            mountDebounceTimeMS = HAL::timeInMilliseconds();
            mountRetries = 0ul;
        } else {
            if ((HAL::timeInMilliseconds() - mountDebounceTimeMS) > 250ul) {
                filePrintManager.mount(false, filePrintManager.posFor(this));
            }
        }
    } else if (!pinLevel) {
        if (state == SDState::SD_SAFE_EJECTED) {
            state = SDState::SD_UNMOUNTED;
        }
        if (state != SDState::SD_UNMOUNTED) {
            Com::writeToAll = true; // tell all listeners that we are finished
            filePrintManager.unmount(false, filePrintManager.posFor(this));
        }
        mountDebounceTimeMS = 0ul;
    }
}

#endif

// ------------- FileSourceUSB --------------

#if USB_HOST_SUPPORT == 1

FileSourceUSB::FileSourceUSB(PGM_P name, int pos)
    : FileSourceSdFatBase(name, pos) {
}

void FileSourceUSB::mount(const bool manual) {
    Com::printFLN(PSTR("mount 1")); // TODO
    HAL::delayMilliseconds(20);     // TODO
    if (state > SDState::SD_SAFE_EJECTED) {
        if (manual && state >= SDState::SD_MOUNTED) {
            // Some hosts (BTT TFT's) require a response after M21 or they'll hang.
            Com::writeToAll = true;
            Com::printFLN(PSTR("SD card ok"));
        }
        return;
    } else {
        Com::printFLN(PSTR("SD card already mounted"));
    }
    Com::printFLN(PSTR("mount 2")); // TODO
    HAL::delayMilliseconds(20);     // TODO

    if (!card.driveAvailable()) {
        Com::printFLN(PSTR("No USB memory stick inserted!"));
        return;
    }
    Com::printFLN(PSTR("mount 3")); // TODO
    HAL::delayMilliseconds(20);     // TODO

    if (!card.init()) {
        if (mountRetries < 3u) {
            mountRetries++;
            if (manual) {
                mount(true); // Try recursively remounting 3 times if manually mounted
            }
            return;
        }
    }
    Com::printFLN(PSTR("mount 4")); // TODO
    HAL::delayMilliseconds(20);     // TODO
    if (!fileSystem.begin(&card)) {
        if (mountRetries < 3u) {
            mountRetries++;
            if (manual) {
                mount(true); // Try recursively remounting 3 times if manually mounted
            }
            return;
        }
        state = SDState::SD_HAS_ERROR;
        mountRetries = 0ul;
        Com::printFLN(Com::tSDInitFail);
        UI_STATUS_UPD("SD Card read error.");
        if (printIfCardErrCode()) {
        } else if (!fileSystem.fatType()) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32/exFAT partition."));
        } else if (!fileSystem.chdir()) {
            Com::printFLN(PSTR("Can't open root directory."));
        }
        return;
    }

    mountRetries = 0ul;
    state = SDState::SD_MOUNTED;
    Com::printFLN(PSTR("SD card ok"));
    Com::printFLN(PSTR("SD card inserted"));

    filePrintManager.printCardInfo(false); // <- collects our volumeLabel too.

    if (!volumeLabel[0u] || strncmp_P(volumeLabel, PSTR("NO NAME"), sizeof(volumeLabel)) == 0u) {
        volumeLabel[0u] = '\0'; // There's no volume label at all.
        UI_STATUS_UPD("SD Card mounted.");
    } else {
        GUI::flashToStringString(GUI::tmpString, PSTR("@ mounted."), volumeLabel);
        UI_STATUS_UPD_RAM(GUI::tmpString);
    }

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED, true);
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::importEEPROM();
#endif
}

void FileSourceUSB::printCardInfo(bool json) {
    if (state < SDState::SD_MOUNTED) {
        if (!json) {
            if (state != SDState::SD_HAS_ERROR) {
                Com::printFLN(Com::tNoMountedCard);
            } else {
                Com::printFLN(Com::tSDInitFail);
            }
        } else {
#if JSON_OUTPUT
            Com::printF(Com::tJSONSDInfo);
            Com::printFLN(PSTR("{\"slot\":0,\"present\":0}}"));
#endif
        }
        return;
    }
    // Print out some basic statistics on successful mount. We also store the volume label.
    uint64_t volumeSectors = 0ul, usageBytes = 0ul;
    uint16_t fileCount = 0u;
    uint8_t folderCount = 0u;
    getCardInfo(volumeLabel, sizeof(volumeLabel), &volumeSectors, &usageBytes, &fileCount, &folderCount);

    if (!json) {
        Com::printF(PSTR("Label: "), volumeLabel);
        Com::printF(PSTR(" | "));

        if (fileSystem.fatType() == FAT_TYPE_EXFAT) {
            Com::printF(PSTR("exFAT"));
        } else {
            Com::printF(PSTR("FAT"), fileSystem.fatType());
        }

        Com::printF(PSTR(" SD"));
        uint8_t type = fileSystem.card()->type();
        if (type == SD_CARD_TYPE_SD1) {
            Com::printF(PSTR("V1"));
        } else if (type == SD_CARD_TYPE_SD2) {
            Com::printF(PSTR("V2"));
        } else if (type == SD_CARD_TYPE_SDHC) {
            if (fileSystem.sectorsPerCluster() > 64ul) {
                Com::printF(PSTR("XC")); // Cards > 32gb are XC.
            } else {
                Com::printF(PSTR("HC"));
            }
        }

        // Print out volume size
        bool gb = false;
        float size = ((0.000001f * 512.0f) * static_cast<float>(volumeSectors));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.f;
        }
        Com::printF(PSTR(" | Volume Size: "), size);
        Com::printF(gb ? PSTR(" GB") : PSTR(" MB"));
        // Print out current estimated usage
        gb = false;
        size = (0.000001f * static_cast<float>(usageBytes));
        if (size > 1000.0f) {
            gb = true;
            size /= 1000.0f;
        }
        Com::printF(PSTR(" | Usage: "), size);
        Com::printF(gb ? PSTR(" GB (") : PSTR(" MB ("), static_cast<int32_t>(fileCount));
        Com::printF(PSTR(" files, "), folderCount);
        Com::printFLN(PSTR(" folders found.)"));
    } else {
#if JSON_OUTPUT
        //{"SDinfo":{"slot":0,"present":1,"capacity":4294967296,"free":2147485184,"speed":20971520,"clsize":32768}}

        uint64_t sizeBytes = 512ull * volumeSectors;
        Com::printF(Com::tJSONSDInfo);
        Com::printF(PSTR("{\"slot\":0,\"present\":1,\"capacity\":"), sizeBytes);
        Com::printF(PSTR(",\"free\":"), (sizeBytes - usageBytes));
        Com::printF(PSTR(",\"speed\":"), (SD_SCK_MHZ(constrain(18, 1ul, 50ul)) / 8ul));
        Com::printF(PSTR(",\"clsize\":"), static_cast<uint32_t>(fileSystem.sectorsPerCluster() * 512ul)); // bytesPerCluster isn't working properly for some reason.
        // Extra
        if (volumeLabel[0]) {
            Com::printF(PSTR(",\"label\":\""));
            FileSource::printEscapeChars(volumeLabel);
            Com::print('"');
        }
        Com::printF(PSTR(",\"files\":"), fileCount);
        Com::printF(PSTR(",\"folders\":"), folderCount);
        Com::printFLN(PSTR("}}"));
#endif
    }
}

void FileSourceUSB::handleAutomount() {
    bool pinLevel = card.driveAvailable();
    return; // TODO block automount until it does not crash

    if (pinLevel && state == SDState::SD_UNMOUNTED) {
        Com::writeToAll = true; // tell all listeners that we are finished
        if (!mountDebounceTimeMS) {
            mountDebounceTimeMS = HAL::timeInMilliseconds();
            mountRetries = 0ul;
        } else {
            if ((HAL::timeInMilliseconds() - mountDebounceTimeMS) > 250ul) {
                filePrintManager.mount(false, filePrintManager.posFor(this));
            }
        }
    } else if (!pinLevel) {
        if (state == SDState::SD_SAFE_EJECTED) {
            state = SDState::SD_UNMOUNTED;
        }
        if (state != SDState::SD_UNMOUNTED) {
            Com::writeToAll = true; // tell all listeners that we are finished
            filePrintManager.unmount(false, filePrintManager.posFor(this));
        }
        mountDebounceTimeMS = 0ul;
    }
}

#endif

// ------------ FileSourceSdFatBase ---------

FileSourceSdFatBase::FileSourceSdFatBase(PGM_P _identifier, int pos)
    : FileSource(_identifier) {
    if (pos >= 0 && pos < 4) {
        fileSources[pos] = this;
    }
    state = SDState::SD_UNMOUNTED;
    mountRetries = 0;
    mountDebounceTimeMS = 0;
    GUI::popAll();
}

void FileSourceSdFatBase::unmount(const bool manual) {
    if (state == SDState::SD_SAFE_EJECTED || state == SDState::SD_UNMOUNTED) {
        // in case we're unmounting manually due to sd error
        if (manual) {
            Com::printFLN(PSTR("SD Card already unmounted."));
        }
        return;
    }
    mountRetries = 0u;

    if (!volumeLabel[0u]) {
        UI_STATUS_UPD("SD Card removed.");
    } else {
        GUI::flashToStringString(GUI::tmpString, PSTR("@ removed."), volumeLabel);
        UI_STATUS_UPD_RAM(GUI::tmpString);
    }
    Com::printFLN(PSTR("SD card removed")); // Needed for hosts

#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    eepromFile.close();
#endif

    selectedFile.close();

    GUI::cwd[0u] = '/';
    GUI::cwd[1u] = '\0';
    GUI::folderLevel = 0u;
    GUI::cwdFile.close();

#if SDFAT_FILE_TYPE == 3 // When using FSVolume (ExFAT & FAT)
    fileSystem.end();
#endif

    if (this == filePrintManager.selectedSource) {
        Printer::setMenuMode(MENU_MODE_SD_MOUNTED + MENU_MODE_PAUSED + MENU_MODE_SD_PRINTING, false);

        if (filePrintManager.isPrinting()) { // unmounted while printing!
            filePrintManager.stopPrint(true);
            if (filePrintManager.selectedFilePos > 0) {
                GUI::setStatusP(PSTR("SD Card removed!"), GUIStatusLevel::ERROR);
            }
        }
        filePrintManager.setPrinting(false);
        filePrintManager.setWriting(false);
        filePrintManager.scheduledPause = SDScheduledPause::NO_PAUSE; // cancel any scheduled pauses. only scheduled stops survive
    }
    state = manual ? SDState::SD_SAFE_EJECTED : SDState::SD_UNMOUNTED;
}

bool FileSourceSdFatBase::getCardInfo(char* volumeLabelBuf, uint8_t volumeLabelSize, uint64_t* volumeSizeSectors, uint64_t* usageBytes, uint16_t* fileCount, uint8_t* folderCount) {
    if (state < SDState::SD_MOUNTED
        || (!fileCount && !folderCount && !usageBytes && !volumeSizeSectors
            && (!volumeLabelBuf || volumeLabelSize <= 1u))) {
        return false;
    }

    fileSystem.chdir();
    if (volumeSizeSectors) { // Multiply by 512 to convert into bytes
        *volumeSizeSectors = (fileSystem.clusterCount() * fileSystem.sectorsPerCluster());
    }
    if (!fileCount && !folderCount && !usageBytes && (!volumeLabelBuf || volumeLabelSize <= 1u)) {
        return true;
    }
    sd_file_t root = fileSystem.open(Com::tSlash);
    if (fileCount || folderCount || usageBytes) {
        if (fileCount) {
            *fileCount = 0u;
        }
        if (folderCount) {
            *folderCount = 0u;
        }
        if (usageBytes) {
            *usageBytes = 0ul;
        }

        doForDirectory(
            root, [&](sd_file_t& file, sd_file_t& dir, size_t depth) {
                if (!file.isHidden() && !dir.isHidden()) {
                    if (usageBytes) {
                        *usageBytes += file.fileSize();
                    }
                    if (folderCount && file.isDir()) {
                        (*folderCount)++;
                    } else if (fileCount && !file.isDir()) {
                        (*fileCount)++;
                    }
                }
                return true;
            },
            true);
    }

    // try finding a volume label in the root directory or the mbr (fat16/32)
    if (volumeLabelBuf && volumeLabelSize > 1u) {
        root.rewind();
        volumeLabelBuf[0u] = '\0';
        uint8_t buf[32u] = { 0u };
        if (fileSystem.fatType() == FAT_TYPE_EXFAT) {
            while (root.read(buf, 32ul) == 32ul) {
                // Special exFat volume label directory entry in the root directory
                DirLabel_t* exFatDir = reinterpret_cast<DirLabel_t*>(buf);
                if (!exFatDir->type) {
                    break;
                } else if (exFatDir->type == EXFAT_TYPE_LABEL) {
                    volumeLabelSize = (volumeLabelSize > exFatDir->labelLength + 1u)
                        ? exFatDir->labelLength + 1u
                        : volumeLabelSize;
                    for (size_t i = 0u; i < volumeLabelSize; i++) {
                        volumeLabelBuf[i] = static_cast<const char>(exFatDir->unicode[i * 2u]);
                    }
                    break;
                }
            }
        } else {
            while (root.read(buf, 32ul) == 32ul) {
                // FAT32/16's own label directory entry in the root directory.
                DirFat_t* fatDir = reinterpret_cast<DirFat_t*>(buf);
                if (fatDir->name[0u] == FAT_NAME_FREE) {
                    break;
                } else if (fatDir->name[0u] != FAT_NAME_DELETED && fatDir->attributes == FAT_ATTRIB_LABEL) {
                    volumeLabelSize = volumeLabelSize > 11u ? 11u : volumeLabelSize;
                    memcpy(volumeLabelBuf, fatDir->name, volumeLabelSize);
                    volumeLabelBuf[volumeLabelSize - 1u] = '\0';
                    break;
                }
            }
        }
        // For FAT16/32, if we never found a label at the magic root directory, try finding
        // one at the boot sector. (this label is only created during formats, though)
        if (fileSystem.fatType() != FAT_TYPE_EXFAT && !volumeLabelBuf[0u]) {
            volumeLabelSize = volumeLabelSize > 11u ? 11u : volumeLabelSize;
            fileSystem.getVolumeLabel(volumeLabelBuf, volumeLabelSize);
            volumeLabelBuf[volumeLabelSize - 1u] = '\0';
            // returns "NO NAME" for missing labels.
        }

        if (volumeLabelBuf[0u] != '\0') {
            char* ptr = &volumeLabelBuf[volumeLabelSize - 1u];
            // trim any whitespace
            while (isspace(*--ptr))
                ;
            *(++ptr) = 0u;
        } else {
            root.close();
            return false;
        }
    }
    root.close();
    return true;
}

bool FileSourceSdFatBase::printIfCardErrCode() {
    if (fileSystem.card()->errorCode()) {
        Com::printF(Com::tSDErrorCode, "0X");
        char buf[8u];
        buf[7u] = '\0';
        char* ptr = fmtHex(&buf[7u], fileSystem.card()->errorCode());
        Com::printF(ptr);
        Com::print(' ');
        printSdErrorText(fileSystem.card()->errorCode());
        Com::println();
        return true;
    }
    return false;
}

bool FileSourceSdFatBase::isMounted() {
    return state == SDState::SD_MOUNTED;
}

bool FileSourceSdFatBase::startWrite(const char* filename) {
    // TODO: a way to cancel host filewrites from the fw side
    fileSystem.chdir();
    if (selectedFile.open(fileSystem.vol(), filename, O_CREAT | O_APPEND | O_WRONLY | O_TRUNC)) {
        return true;
    }
    return false;
}

bool FileSourceSdFatBase::write(char* buf, uint8_t pos) {
    selectedFile.clearWriteError();
    filePrintManager.writtenBytes += selectedFile.write(buf, pos);
    return !selectedFile.getWriteError();
}

void FileSourceSdFatBase::finishWriting() {
    filePrintManager.writtenBytes = selectedFile.fileSize();
    selectedFile.sync(); // exFat seems to require a manual sync for now
    selectedFile.close();
    state = SDState::SD_MOUNTED;
}

#if JSON_OUTPUT
void FileSourceSdFatBase::lsJSON(const char* filename) {
    sd_file_t dir;
    fileSystem.chdir();
    if (*filename == 0) {
        dir.open(Com::tSlash);
    } else {
        if (!dir.open(fileSystem.vol(), filename) || !dir.isDir()) {
            dir.close();
            Com::printF(Com::tJSONErrorStart);
            Com::printF(Com::tFileOpenFailed);
            Com::printFLN(Com::tJSONErrorEnd);
            return;
        }
    }
    Com::printF(Com::tJSONDir);
    FileSource::printEscapeChars(filename);
    Com::printF(Com::tJSONFiles);
    ls(dir, true); // ls in JSON mode
    dir.close();
    Com::printFLN(Com::tJSONArrayEnd);
}

void FileSourceSdFatBase::JSONFileInfo(const char* filename) {
    sd_file_t targetFile;
    GCodeFileInfo *info, tmpInfo;
    if (strlen(filename) == 0) {
        targetFile = selectedFile;
        info = &filePrintManager.fileInfo;
    } else {
        if (!targetFile.open(fileSystem.vol(), filename, O_RDONLY) || targetFile.isDir()) {
            targetFile.close();
            Com::printF(Com::tJSONErrorStart);
            Com::printF(Com::tFileOpenFailed);
            Com::printFLN(Com::tJSONErrorEnd);
            return;
        }
        info = &tmpInfo;
        info->init(targetFile);
    }
    if (!targetFile.isOpen()) {
        Com::printF(Com::tJSONErrorStart);
        Com::printF(Com::tNotSDPrinting);
        Com::printFLN(Com::tJSONErrorEnd);
        return;
    }

    // {"err":0,"size":457574,"height":4.00,"layerHeight":0.25,"filament":[6556.3],"generatedBy":"Slic3r 1.1.7 on 2014-11-09 at 17:11:32"}
    Com::printF(Com::tJSONFileInfoStart);
    Com::print(info->fileSize);
    Com::printF(Com::tJSONFileInfoHeight);
    Com::print(info->objectHeight);
    Com::printF(Com::tJSONFileInfoLayerHeight);
    Com::print(info->layerHeight);
    Com::printF(Com::tJSONFileInfoFilament);
    Com::print(info->filamentNeeded);
    Com::printF(Com::tJSONFileInfoGeneratedBy);
    Com::print(info->generatedBy);
    Com::print('"');
    if (strlen(filename) == 0) {
        Com::printF(Com::tJSONFileInfoName);
        Com::printF(getFN(targetFile));
        Com::print('"');
    }
    Com::print('}');
    Com::println();
    targetFile.close();
}
#endif

void FileSourceSdFatBase::ls(const char* lsDir, const bool json) {
    fileSystem.chdir();
    sd_file_t dir = fileSystem.open(lsDir);
    ls(dir, json);
}

void FileSourceSdFatBase::ls(sd_file_t& rootDir, const bool json) {
    if (state < SDState::SD_MOUNTED || !rootDir.isDir()) {
        return;
    }
    if (!json) {
        Com::printFLN(Com::tBeginFileList);
    }
    fullName[0u] = '\0'; // Used to keep track of long nested directory names
    size_t lastDepth = 0u;
#if JSON_OUTPUT
    bool firstFile = true;
#endif
    auto action = [&](sd_file_t& file, sd_file_t& dir, size_t depth) {
        if (!dir.isHidden() && !file.isHidden()) {
            if (!json) {
                if (depth > lastDepth) {
                    if (lastDepth) {
                        fullName[strlen(fullName)] = '/';
                    }
                    strcat(fullName, getFN(dir));
                } else if (depth < lastDepth) {
                    char* p = strrchr(fullName, '/');
                    if (p) {
                        *(++p) = '\0';
                    } else {
                        fullName[0u] = '\0';
                    }
                }
                lastDepth = depth;
                if (depth) {
                    Com::print(fullName);
                    Com::print('/');
                }
                Com::print(getFN(file));
                if (file.isDir()) {
                    Com::print('/');
                } else {
                    Com::print(' ');
                    Com::print(static_cast<int32_t>(file.fileSize()));
                }
                Com::println();
            } else {
#if JSON_OUTPUT
                if (!firstFile) {
                    Com::print(',');
                }
                firstFile = false;
                Com::print('"');
                if (file.isDir()) {
                    Com::print('*');
                }
                FileSource::printEscapeChars(getFN(file));
                Com::print('"');
#endif
            }
        }
        return true;
    };

    doForDirectory(rootDir, action, true);
    rootDir.close();

    if (!json) {
        Com::printFLN(Com::tEndFileList);
    }
}

bool FileSourceSdFatBase::startRead(const char* filename, const bool silent) {
    if (state != SDState::SD_MOUNTED || Printer::failedMode) {
        if (!silent && (state < SDState::SD_MOUNTED)) {
            Com::printFLN(Com::tNoMountedCard);
        }
        return false;
    }
    if (selectedFile.open(fileSystem.vol(), filename) && (filePrintManager.selectedFileSize = selectedFile.fileSize())) {
#if JSON_OUTPUT
        filePrintManager.fileInfo.init(selectedFile);
#endif
        return true;
    }
    selectedFile.close();
    if (!silent) {
        Com::printFLN(Com::tFileOpenFailed);
    }
    return false;
}

int FileSourceSdFatBase::read() {
    int n = selectedFile.read();
    if (n == -1) {
        Com::printFLN(Com::tSDReadError);

        // Second try in case of recoverable errors
        selectedFile.seekSet(filePrintManager.selectedFilePos);
        n = selectedFile.read();
        if (n == -1) {
            return -1;
        }
    }
    ++filePrintManager.selectedFilePos;
    return n;
}

void FileSourceSdFatBase::setPosition(uint32_t newPos) {
    if (state < SDState::SD_MOUNTED || !selectedFile.isOpen()) {
        return;
    }
    filePrintManager.selectedFilePos = newPos;
    selectedFile.seekSet(filePrintManager.selectedFilePos);
}

void FileSourceSdFatBase::close() {
    selectedFile.close();
    state = SDState::SD_MOUNTED;
}

void FileSourceSdFatBase::deleteFile(const char* filename) {
    if (state != SDState::SD_MOUNTED) {
        return;
    }
    fileSystem.chdir();
    if (fileSystem.rmdir(filename)) {
        Com::printFLN(Com::tFileDeleted);
        return;
    }

#if EEPROM_AVAILABLE == EEPROM_SDCARD // allow deleting eeprom.bin, but make sure we close the handler too.
    if (eepromFile.isOpen() && !strcmp(filename, getFN(eepromFile))) {
        eepromFile.remove();
        Com::printFLN(Com::tFileDeleted);
        return;
    }
#endif

    if (!(selectedFile.isOpen() && !strcmp(filename, getFN(selectedFile)))) {
        if (fileSystem.remove(filename)) {
            Com::printFLN(Com::tFileDeleted);
            return;
        }
    }
    Com::printFLN(Com::tDeletionFailed);
}

void FileSourceSdFatBase::makeDirectory(const char* filename) {
    if (state != SDState::SD_MOUNTED) {
        return;
    }
    fileSystem.chdir();
    if (fileSystem.mkdir(filename)) {
        Com::printFLN(Com::tDirectoryCreated);
    } else {
        Com::printFLN(Com::tCreationFailed);
    }
}

bool FileSourceSdFatBase::isFileOpened() {
    return selectedFile.isOpen();
}

bool FileSourceSdFatBase::listDirectory(sd_file_t& dir, bool recursive, listDirectoryCallback callback, int depth) {
    if (!dir.isDir()) {
        return false;
    }
    sd_file_t file;
    dir.rewind();
    while (file.openNext(&dir)) {
        HAL::pingWatchdog();
        file.getName(tempLongFilename, sizeof(tempLongFilename));
        if (file.isDir()) { // append / as directory marker
            char* p = &tempLongFilename[strlen(tempLongFilename)];
            *p = '/';
            p++;
            *p = 0;
        }
        if (!callback(tempLongFilename, file.dirIndex(), depth)) {
            file.close();
            return false;
        }
        if (recursive && file.isDir() && (depth + 1u) < SD_MAX_FOLDER_DEPTH) {
            depth++;
            if (!listDirectory(file, recursive, callback, depth)) {
                file.close();
                return false;
            }
            depth--;
        }
        file.close();
    }
    return true;
}

bool FileSourceSdFatBase::listDirectory(char* dir, bool recursive, listDirectoryCallback callback, int depth) {
    sd_file_t dirF;
    dirF.open(fileSystem.vol(), dir, O_RDONLY);
    return listDirectory(dirF, recursive, callback, depth);
}

char* FileSourceSdFatBase::filenameAtIndex(char* dir, int idx) {
    sd_file_t dirF;
    dirF.open(fileSystem.vol(), dir, O_RDONLY);
    sd_file_t file;
    dirF.rewind();
    while (file.openNext(&dirF)) {
        HAL::pingWatchdog();
        if (file.dirIndex() == idx) {
            file.getName(tempLongFilename, sizeof(tempLongFilename));
            if (file.isDir()) {
                char* p = &tempLongFilename[strlen(tempLongFilename)];
                *p = '/';
                p++;
                *p = 0;
            }
            file.close();
            return tempLongFilename;
        }
        file.close();
    }
    return nullptr;
}

#undef IO_TARGET
#define IO_TARGET IO_TARGET_TEMPLATES
#include "../io/redefine.h"

#endif