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

#if SDSUPPORT

#ifndef SD_SPI_SPEED_MHZ
#define SD_SPI_SPEED_MHZ 4
#endif

char tempLongFilename[LONG_FILENAME_LENGTH + 1];
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];
SDCardGCodeSource sdSource;
SDCard sd;

SDCard::SDCard()
    : mountDebounceTime(0ul)
    , state(SDState::SD_UNMOUNTED) {
    Printer::setAutomount(true);
}

void SDCard::automount() {
    bool pinLevel = !HAL::digitalRead(SDCARDDETECT);
    if (SDCARDDETECTINVERTED) {
        pinLevel = !pinLevel;
    }
    if (pinLevel && state == SDState::SD_UNMOUNTED) {
        if (!mountDebounceTime) {
            mountDebounceTime = HAL::timeInMilliseconds();
        } else {
            if ((HAL::timeInMilliseconds() - mountDebounceTime) > 100ul) {
                mount();
            }
        }
    } else if (!pinLevel) {
        if (state != SDState::SD_UNMOUNTED) {
            unmount(false);
        }
        mountDebounceTime = 0;
    }
}

void SDCard::mount() {
    if (state != SDState::SD_UNMOUNTED) {
        return;
    }
    bool pinLevel = !HAL::digitalRead(SDCARDDETECT);
    if (SDCARDDETECTINVERTED) {
        pinLevel = !pinLevel;
    }
    if (!pinLevel) {
        return;
    }

#if (ENABLE_SOFTWARE_SPI_CLASS || (SPI_DRIVER_SELECT == 2))
    SdSpiConfig spiConfig = SdSpiConfig(SDSS, DEDICATED_SPI, SD_SCK_MHZ(constrain(SD_SPI_SPEED_MHZ, 1, 50)), &softSpi);
#else
    SdSpiConfig spiConfig = SdSpiConfig(SDSS, DEDICATED_SPI, SD_SCK_MHZ(constrain(SD_SPI_SPEED_MHZ, 1, 50)));
#endif
    if (!fileSystem.begin(spiConfig)) {
        state = SDState::SD_HAS_ERROR;
        Com::printFLN(Com::tSDInitFail);
        UI_STATUS_UPD("SD Card read error.");
        if (fileSystem.card()->errorCode()) {
            Com::printFLN(PSTR(
                "SD initialization failed.\n"
                "Do not reformat the card!\n"
                "Is the card correctly inserted?\n"
                "Is chipSelect set to the correct value?\n"
                "Does another SPI device need to be disabled?\n"
                "Is there a wiring/soldering problem?"));
            Com::printFLN(PSTR("errorCode: "), fileSystem.card()->errorCode());
            return;
        }
        if (!fileSystem.vol()->fatType()) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32/exFAT partition."));
            return;
        }
        if (!fileSystem.chdir()) {
            Com::printFLN(PSTR("Can't open root directory."));
            return;
        }
        return;
    }
    state = SDState::SD_MOUNTED;
    
    // DEBUGGING
    uint32_t volumeSize;
    uint32_t usageBytes = 0;
    uint16_t fileCount = 0;
    uint8_t folderCount = 0;
    getCardInfo(&volumeSize, &usageBytes, &fileCount, &folderCount);

    if (fileSystem.fatType() > 32) {
        Com::printF(PSTR("exFAT"));
    } else {
        Com::printF(PSTR("FAT"), fileSystem.fatType());
    }

    Com::printF(PSTR(" SD"));
    switch (fileSystem.card()->type()) {
    case 0:
        Com::printF(PSTR(" V1"));
        break;
    case 1:
        Com::printF(PSTR(" V2"));
        break;
    case 2:
        Com::printF(PSTR("HC"));
        break;
    default:
        Com::printF(PSTR("XC"));
        break;
    }

    bool gb = false;
    float showVolSize = (0.000512f * static_cast<float>(volumeSize));
    if (showVolSize > 1000.0f) {
        gb = true;
        showVolSize /= 1000.f;
    }
    Com::printF(PSTR(" | Volume Size: "), showVolSize);
    Com::printF(gb ? PSTR(" GB") : PSTR(" MB"));
    gb = false;
    float showUseSize = ((1.0f / (1024.0f * 1024.0f)) * static_cast<float>(usageBytes));
    if (showUseSize > 1000.0f) {
        gb = true;
        showUseSize /= 1000.0f;
    }
    Com::printF(PSTR(" | In use: "), showUseSize);
    Com::printF(gb ? PSTR(" GB (") : PSTR(" MB ("), fileCount);
    Com::printF(PSTR(" files, "), folderCount);
    Com::printFLN(PSTR(" folders found.)"));

#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
    HAL::importEEPROM();
#endif
}

bool SDCard::getCardInfo(uint32_t* volumeSizeBytes, uint32_t* usageBytes, uint16_t* fileCount, uint8_t* folderCount) {
    if (state != SDState::SD_MOUNTED
        || (!fileCount && !folderCount && !usageBytes && !volumeSizeBytes)) {
        return false;
    }

    fileSystem.chdir();
    if (volumeSizeBytes) {
        *volumeSizeBytes = fileSystem.clusterCount() * fileSystem.sectorsPerCluster();
    }
    if (!fileCount && !folderCount && !usageBytes) {
        return true;
    }

    if (fileCount) {
        *fileCount = 0u;
    }
    if (folderCount) {
        *folderCount = 0u;
    }
    if (usageBytes) {
        *usageBytes = 0ul;
    }

    sd_file_t root = fileSystem.open(Com::tSlash);
    doForDirectory(
        root, [&](sd_file_t& file, sd_file_t& dir, size_t depth) {
            if (!file.isHidden() && !dir.isHidden()) {
                if (usageBytes) {
                    *usageBytes += file.size();
                }
                if (folderCount && file.isDir()) {
                    (*folderCount)++;
                } else if (fileCount && !file.isDir()) {
                    (*fileCount)++;
                }
            }
        },
        true);
    root.close();

    return true;
}

extern sd_file_t eepromFile; // TODO handle properly, will crash after ejecting exFAT cards and then mounting new cards, if not closed
void SDCard::unmount(bool manual) {
    if (state == SDState::SD_UNMOUNTED) {
        return;
    }
    state = SDState::SD_UNMOUNTED;
#if FEATURE_CONTROLLER != CONTROLLER_NONE
    GUI::cwd[0] = '/';
    GUI::cwd[1] = 0;
    GUI::folderLevel = 0;
#endif
    eepromFile.close(); // TODO!!
    selectedFile.close();
    fileSystem.end();
    Com::printFLN(PSTR("Card unmounted"));
}

bool SDCard::selectFile(const char* filename, bool silent) {
    if (state != SDState::SD_MOUNTED) {
        return false;
    }
    if (!strstr(filename, PSTR(".gcode"))) {
        if (!silent) {
            Com::printFLN(Com::tInvalidFiletype);
        }
        return false;
    }
    if (selectedFile.open(filename, O_RDONLY)) {
        // Filename for progress view
        selectedFile.getName(Printer::printName, sizeof(Printer::printName));
        Printer::printName[sizeof(Printer::printName) - 1u] = '\0';
        Printer::maxLayer = -1;
        sdpos = 0;
        filesize = selectedFile.fileSize();
        if (!silent) {
            Com::printF(Com::tFileOpened, filename);
            float showSize = static_cast<float>(filesize);
            uint8_t unit = 0u; // Bytes
            if (showSize > 1000.0f) {
                showSize /= 1000.0f;
                unit = 1u; // kB
            }
            if (showSize > 1000.0f) {
                showSize /= 1000.0f;
                unit = 2u; // MB
            }

            Com::printF(Com::tSpaceSizeColon, showSize, 0);
            if (unit == 1u) {
                Com::printFLN(PSTR("kB"));
            } else if (unit == 2u) {
                Com::printFLN(PSTR("MB"));
            } else {
                Com::printFLN(PSTR(" bytes"));
            }

            Com::printFLN(Com::tFileSelected);
        }
#if JSON_OUTPUT
        fileInfo.init(file);
#endif
        return true;
    }

    if (!silent) {
        Com::printFLN(Com::tFileOpenFailed);
    }
    return false;
}
void SDCard::startPrint() {
    if (state != SDState::SD_MOUNTED || !selectedFile.isOpen()) {
        return;
    }
    state = SDState::SD_PRINTING;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    Printer::setPrinting(true);
    Printer::maxLayer = -1;
    Printer::currentLayer = 0;
    GCodeSource::registerSource(&sdSource);
}

void SDCard::pausePrint(bool intern) {
    if (state != SDState::SD_PRINTING) {
        return;
    }
    Printer::setMenuMode(MENU_MODE_PAUSED, true);
#if !defined(DISABLE_PRINTMODE_ON_PAUSE) || DISABLE_PRINTMODE_ON_PAUSE == 1
    Printer::setPrinting(false);
#endif
    GCodeSource::removeSource(&sdSource);
    if (intern) { // TODO
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
        EVENT_SD_PAUSE_START(intern);
        EVENT_SD_PAUSE_END(intern);
#pragma GCC diagnostic pop
    }
}

void SDCard::pausePrintPart2() {
    if (EVENT_SD_PAUSE_START(true)) {
        Commands::waitUntilEndOfAllBuffers();
        Motion1::pushToMemory();
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                            Motion1::currentPosition[E_AXIS] - RETRACT_ON_PAUSE,
                            Motion1::maxFeedrate[E_AXIS] / 2);
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            tool->afterPause();
        }
        Motion1::moveToParkPosition();
        GCode::executeFString(PSTR(PAUSE_START_COMMANDS));
    }
    EVENT_SD_PAUSE_END(intern);
}

void SDCard::continuePrint(bool intern) {
    if (state != SDState::SD_PRINTING) {
        return;
    }
    if (EVENT_SD_CONTINUE_START(intern)) {
        if (intern) {
            Tool* tool = Tool::getActiveTool();
            if (tool) {
                tool->beforeContinue();
            }
            GCode::executeFString(PSTR(PAUSE_END_COMMANDS));
            Motion1::popFromMemory();
            Motion1::pushToMemory();
            Motion1::tmpPosition[Z_AXIS] = IGNORE_COORDINATE;
            Motion1::tmpPosition[E_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[X_AXIS], false);
            Motion1::popFromMemory();
            Motion1::pushToMemory();
            Motion1::tmpPosition[X_AXIS] = IGNORE_COORDINATE;
            Motion1::tmpPosition[Y_AXIS] = IGNORE_COORDINATE;
            Motion1::tmpPosition[E_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[Z_AXIS], false);
            Motion1::popFromMemory();
            Motion1::tmpPosition[X_AXIS] = IGNORE_COORDINATE;
            Motion1::tmpPosition[Y_AXIS] = IGNORE_COORDINATE;
            Motion1::tmpPosition[Z_AXIS] = IGNORE_COORDINATE;
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[E_AXIS], false);
        }
    }
    EVENT_SD_CONTINUE_END(intern);
    GCodeSource::registerSource(&sdSource);
    Printer::setPrinting(true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
}

void SDCard::stopPrint() {
    if (state != SDState::SD_PRINTING) {
        return;
    }
    state = SDState::SD_MOUNTED;
    Com::printFLN(PSTR("SD print stopped by user."));
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, false);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    Printer::setPrinting(0);
    Printer::breakLongCommand = true; // stop waiting heatup if running
}

void SDCard::stopPrintPart2() {
    Motion1::moveToParkPosition();
    GCodeSource::removeSource(&sdSource);
    if (EVENT_SD_STOP_START) {
        GCode::executeFString(PSTR(SD_RUN_ON_STOP));
        if (SD_STOP_HEATER_AND_MOTORS_ON_STOP) {
            Commands::waitUntilEndOfAllMoves();
            Printer::kill(false);
        }
    }
    EVENT_SD_STOP_END;
}

void SDCard::writeCommand(GCode* code) {
    unsigned int sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    uint8_t buf[100];
    uint8_t p = 2;
    selectedFile.clearWriteError();
    uint16_t params = 128 | (code->params & ~1);
    memcopy2(buf, &params);
    if (code->isV2()) // Read G,M as 16 bit value
    {
        memcopy2(&buf[p], &code->params2);
        p += 2;
        if (code->hasString())
            buf[p++] = strlen(code->text);
        if (code->hasM()) {
            memcopy2(&buf[p], &code->M);
            p += 2;
        }
        if (code->hasG()) {
            memcopy2(&buf[p], &code->G);
            p += 2;
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
        p += 4;
    }
    if (code->hasY()) {
        memcopy4(&buf[p], &code->Y);
        p += 4;
    }
    if (code->hasZ()) {
        memcopy4(&buf[p], &code->Z);
        p += 4;
    }
    if (code->hasE()) {
        memcopy4(&buf[p], &code->E);
        p += 4;
    }
    if (code->hasF()) {
        memcopy4(&buf[p], &code->F);
        p += 4;
    }
    if (code->hasT()) {
        buf[p++] = code->T;
    }
    if (code->hasS()) {
        memcopy4(&buf[p], &code->S);
        p += 4;
    }
    if (code->hasP()) {
        memcopy4(&buf[p], &code->P);
        p += 4;
    }
    if (code->hasI()) {
        memcopy4(&buf[p], &code->I);
        p += 4;
    }
    if (code->hasJ()) {
        memcopy4(&buf[p], &code->J);
        p += 4;
    }
    if (code->hasR()) {
        memcopy4(&buf[p], &code->R);
        p += 4;
    }
    if (code->hasD()) {
        memcopy4(&buf[p], &code->D);
        p += 4;
    }
    if (code->hasC()) {
        memcopy4(&buf[p], &code->C);
        p += 4;
    }
    if (code->hasH()) {
        memcopy4(&buf[p], &code->H);
        p += 4;
    }
    if (code->hasA()) {
        memcopy4(&buf[p], &code->A);
        p += 4;
    }
    if (code->hasB()) {
        memcopy4(&buf[p], &code->B);
        //*(float*)&buf[p] = code->B;
        p += 4;
    }
    if (code->hasK()) {
        memcopy4(&buf[p], &code->K);
        p += 4;
    }
    if (code->hasL()) {
        memcopy4(&buf[p], &code->L);
        p += 4;
    }
    if (code->hasO()) {
        memcopy4(&buf[p], &code->O);
        //*(float*)&buf[p] = code->O;
        p += 4;
    }
    if (code->hasU()) {
        memcopy4(&buf[p], &code->U);
        p += 4;
    }
    if (code->hasV()) {
        memcopy4(&buf[p], &code->V);
        p += 4;
    }
    if (code->hasW()) {
        memcopy4(&buf[p], &code->W);
        p += 4;
    }
    if (code->hasString()) // read 16 uint8_t into string
    {
        char* sp = code->text;
        if (code->isV2()) {
            uint8_t i = strlen(code->text);
            for (; i; i--)
                buf[p++] = *sp++;
        } else {
            for (uint8_t i = 0; i < 16; ++i)
                buf[p++] = *sp++;
        }
    }
    uint8_t* ptr = buf;
    uint8_t len = p;
    while (len) {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *ptr++;
            if (sum1 >= 255)
                sum1 -= 255;
            sum2 += sum1;
            if (sum2 >= 255)
                sum2 -= 255;
        } while (--tlen);
    }
    buf[p++] = sum1;
    buf[p++] = sum2;
    if (params == 128) {
        Com::printErrorFLN(Com::tAPIDFinished);
    } else {
        selectedFile.write(buf, p);
    }
    if (selectedFile.getWriteError()) {
        Com::printFLN(Com::tErrorWritingToFile);
    }
}

template <typename T>
bool SDCard::doForDirectory(sd_file_t& dir, T&& action, const bool recursive) {
    if (!dir.isDir()) {
        return false;
    }
    static size_t depth = 0;
    sd_file_t file;
    dir.rewind();
    while (file.openNext(&dir)) {
        if (file.isDir() && recursive) {
            depth++;
            if (!doForDirectory(file, action, true)) {
                file.close();
                depth = 0;
                return false;
            }
            depth--;
        }
        action(file, dir, depth);
        file.close();
    }
    file.close();
    return true;
}
void SDCard::ls() {
    if (state != SDState::SD_MOUNTED) {
        return;
    }
    Com::printFLN(Com::tBeginFileList);
    fileSystem.chdir();
    sd_file_t rootDir = fileSystem.open(Com::tSlash);
    //static size_t lastDepth = 0u;
    auto act = [&](sd_file_t& file, sd_file_t& dir, size_t depth) {
        if (!file.isHidden() && !dir.isHidden()) {
            /* // ascii directory tree
            Com::print((!depth && file.isDir()) ? '+' : '|');
            if (depth) {
                for (size_t i = 0u; i < (depth * 2u); i++) {
                    if (i && !(i % 2u)) {
                        Com::print("|");
                    } else {
                        Com::print(' ');
                    }
                }
                if (!file.isDir()) {
                    if (depth > 1u) {
                        Com::print(' ');
                    }
                    Com::print((depth != lastDepth) ? '/' : '|');
                } else {
                    Com::print('+');
                }
            }
            if (file.isDir()) { // Extra for directory at depth 0
                Com::print('-');
            }
            Com::print('-');
            lastDepth = depth;
            */
            if (depth) {
                dir.getName(tempLongFilename, sizeof(tempLongFilename));
                Com::print(tempLongFilename);
                Com::print('/');
            }
            file.getName(tempLongFilename, sizeof(tempLongFilename));
            Com::print(tempLongFilename);
            if (file.isDir()) {
                Com::print('/');
            } else {
                Com::print(' ');
                Com::print(static_cast<int32_t>(file.size()));
            }
            Com::println();
        }
    };
    //lastDepth = 0;
    doForDirectory(rootDir, act, true);
    rootDir.close();

    Com::printFLN(Com::tEndFileList);
}

void SDCard::printStatus(bool getFilename) {
    if (getFilename) {
        Com::printF(Com::tCurrentOpenFile);
        if (state == SDState::SD_PRINTING) {
            Com::printFLN(Printer::printName);
        } else {
            Com::printFLN(PSTR("(no file)"));
        }
    } else {
        if (state == SDState::SD_PRINTING) {
            Com::printF(Com::tSDPrintingByte, sdpos);
            Com::printFLN(Com::tSlash, filesize);
        } else {
            Com::printFLN(Com::tNotSDPrinting);
        }
    }
}

void SDCard::startWrite(const char* filename) {
    if (state != SDState::SD_MOUNTED) {
        return;
    }
    fileSystem.chdir();
    if (!selectedFile.open(filename, O_CREAT | O_APPEND | O_WRONLY | O_TRUNC)) {
        Com::printFLN(Com::tOpenFailedFile, filename);
    } else {
        GUI::setStatusP(PSTR("Uploading..."), GUIStatusLevel::INFO);
        Com::printFLN(Com::tWritingToFile, filename);
        state = SDState::SD_WRITING;
    }
}

void SDCard::finishWrite() {
    if (state != SDState::SD_WRITING) {
        return;
    }
    selectedFile.close();
    state = SDState::SD_MOUNTED;
    Com::printFLN(Com::tDoneSavingFile);
    GUI::pop();
}

void SDCard::deleteFile(const char* filename) {
    if (state != SDState::SD_MOUNTED) {
        return;
    }

    fileSystem.chdir();
    if (fileSystem.remove(filename)) {
        Com::printFLN(Com::tFileDeleted);
    } else {
        if (fileSystem.rmdir(filename)) {
            Com::printFLN(Com::tFileDeleted);
        } else {
            Com::printFLN(Com::tDeletionFailed);
        }
    }
}

void SDCard::makeDirectory(const char* filename) {
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

#if JSON_OUTPUT
void SDCard::lsJSON(const char* filename) {
    SdBaseFile dir;
    fileSystem.chdir();
    if (*filename == 0) {
        dir.openRoot(fileSystem.vol());
    } else {
        if (!dir.open(fileSystem.vwd(), filename, O_RDONLY) || !dir.isDir()) {
            Com::printF(Com::tJSONErrorStart);
            Com::printF(Com::tFileOpenFailed);
            Com::printFLN(Com::tJSONErrorEnd);
            return;
        }
    }

    Com::printF(Com::tJSONDir);
    SDCard::printEscapeChars(filename);
    Com::printF(Com::tJSONFiles);
    dir.lsJSON();
    Com::printFLN(Com::tJSONArrayEnd);
}

void SDCard::printEscapeChars(const char* s) {
    for (unsigned int i = 0; i < strlen(s); ++i) {
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

void SDCard::JSONFileInfo(const char* filename) {
    sd_file_t targetFile;
    GCodeFileInfo *info, tmpInfo;
    if (strlen(filename) == 0) {
        targetFile = file;
        info = &fileInfo;
    } else {
        if (!targetFile.open(fileSystem.vwd(), filename, O_RDONLY) || targetFile.isDir()) {
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
        file.printName();
        Com::print('"');
    }
    Com::print('}');
    Com::println();
};

#endif
#ifdef GLENN_DEBUG
void SDCard::writeToFile() {
    size_t nbyte;
    char szName[10];

    strcpy(szName, "Testing\r\n");
    nbyte = file.write(szName, strlen(szName));
    Com::print("L=");
    Com::print((long)nbyte);
    Com::println();
}

#endif

#endif

#if JSON_OUTPUT

// --------------------------------------------------------------- //
// Code that gets gcode information is adapted from RepRapFirmware //
// Originally licensed under GPL                                   //
// Authors: reprappro, dc42, dcnewman, others                      //
// Source: https://github.com/dcnewman/RepRapFirmware              //
// Copy date: 15 Nov 2015                                          //
// --------------------------------------------------------------- //

void GCodeFileInfo::init(SdFile& file) {
    this->fileSize = file.fileSize();
    this->filamentNeeded = 0.0;
    this->objectHeight = 0.0;
    this->layerHeight = 0.0;
    if (!file.isOpen()) {
        return;
    }
    bool genByFound = false, layerHeightFound = false, filamentNeedFound = false;
#if CPU_ARCH == ARCH_AVR
#define GCI_BUF_SIZE 120
#else
#define GCI_BUF_SIZE 1024
#endif
    // READ 4KB FROM THE BEGINNING
    char buf[GCI_BUF_SIZE];
    for (int i = 0; i < 4096; i += GCI_BUF_SIZE - 50) {
        if (!file.seekSet(i))
            break;
        file.read(buf, GCI_BUF_SIZE);
        if (!genByFound && findGeneratedBy(buf, this->generatedBy)) {
            genByFound = true;
        }
        if (!layerHeightFound && findLayerHeight(buf, this->layerHeight)) {
            layerHeightFound = true;
        }
        if (!filamentNeedFound && findFilamentNeed(buf, this->filamentNeeded)) {
            filamentNeedFound = true;
        }
        if (genByFound && layerHeightFound && filamentNeedFound) {
            goto get_objectHeight;
        }
    }

    // READ 4KB FROM END
    for (int i = 0; i < 4096; i += GCI_BUF_SIZE - 50) {
        if (!file.seekEnd(-4096 + i)) {
            break;
        }
        file.read(buf, GCI_BUF_SIZE);
        if (!genByFound && findGeneratedBy(buf, this->generatedBy)) {
            genByFound = true;
        }
        if (!layerHeightFound && findLayerHeight(buf, this->layerHeight)) {
            layerHeightFound = true;
        }
        if (!filamentNeedFound && findFilamentNeed(buf, this->filamentNeeded)) {
            filamentNeedFound = true;
        }
        if (genByFound && layerHeightFound && filamentNeedFound) {
            goto get_objectHeight;
        }
    }

get_objectHeight:
    // MOVE FROM END UP IN 1KB BLOCKS UP TO 30KB
    for (int i = GCI_BUF_SIZE; i < 30000; i += GCI_BUF_SIZE - 50) {
        if (!file.seekEnd(-i)) {
            break;
        }
        file.read(buf, GCI_BUF_SIZE);
        if (findTotalHeight(buf, this->objectHeight)) {
            break;
        }
    }
    file.seekSet(0);
}

bool GCodeFileInfo::findGeneratedBy(char* buf, char* genBy) {
    // Slic3r & S3D
    const char* generatedByString = PSTR("generated by ");
    char* pos = strstr_P(buf, generatedByString);
    if (pos) {
        pos += strlen_P(generatedByString);
        size_t i = 0;
        while (i < GENBY_SIZE - 1 && *pos >= ' ') {
            char c = *pos++;
            if (c == '"' || c == '\\') {
                // Need to escape the quote-mark for JSON
                if (i > GENBY_SIZE - 3)
                    break;
                genBy[i++] = '\\';
            }
            genBy[i++] = c;
        }
        genBy[i] = 0;
        return true;
    }

    // CURA
    const char* slicedAtString = PSTR(";Sliced at: ");
    pos = strstr_P(buf, slicedAtString);
    if (pos) {
        strcpy_P(genBy, PSTR("Cura"));
        return true;
    }

    // UNKNOWN
    strcpy_P(genBy, PSTR("Unknown"));
    return false;
}

bool GCodeFileInfo::findLayerHeight(char* buf, float& layerHeight) {
    // SLIC3R
    layerHeight = 0;
    const char* layerHeightSlic3r = PSTR("; layer_height ");
    char* pos = strstr_P(buf, layerHeightSlic3r);
    if (pos) {
        pos += strlen_P(layerHeightSlic3r);
        while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
            ++pos;
        }
        layerHeight = strtod(pos, NULL);
        return true;
    }

    // CURA
    const char* layerHeightCura = PSTR("Layer height: ");
    pos = strstr_P(buf, layerHeightCura);
    if (pos) {
        pos += strlen_P(layerHeightCura);
        while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
            ++pos;
        }
        layerHeight = strtod(pos, NULL);
        return true;
    }

    return false;
}

bool GCodeFileInfo::findFilamentNeed(char* buf, float& filament) {
    const char* filamentUsedStr = PSTR("filament used");
    const char* pos = strstr_P(buf, filamentUsedStr);
    filament = 0;
    if (pos != NULL) {
        pos += strlen_P(filamentUsedStr);
        while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
            ++pos; // this allows for " = " from default slic3r comment and ": " from default Cura comment
        }
        if (isDigit(*pos)) {
            char* q;
            filament += strtod(pos, &q);
            if (*q == 'm' && *(q + 1) != 'm') {
                filament *= 1000.0; // Cura outputs filament used in metres not mm
            }
        }
        return true;
    }
    return false;
}

bool GCodeFileInfo::findTotalHeight(char* buf, float& height) {
    int len = 1024;
    bool inComment, inRelativeMode = false;
    unsigned int zPos;
    for (int i = len - 5; i > 0; i--) {
        if (inRelativeMode) {
            inRelativeMode = !(buf[i] == 'G' && buf[i + 1] == '9' && buf[i + 2] == '1' && buf[i + 3] <= ' ');
        } else if (buf[i] == 'G') {
            // Ignore G0/G1 codes if absolute mode was switched back using G90 (typical for Cura files)
            if (buf[i + 1] == '9' && buf[i + 2] == '0' && buf[i + 3] <= ' ') {
                inRelativeMode = true;
            } else if ((buf[i + 1] == '0' || buf[i + 1] == '1') && buf[i + 2] == ' ') {
                // Look for last "G0/G1 ... Z#HEIGHT#" command as generated by common slicers
                // Looks like we found a controlled move, however it could be in a comment, especially when using slic3r 1.1.1
                inComment = false;
                size_t j = i;
                while (j != 0) {
                    --j;
                    char c = buf[j];
                    if (c == '\n' || c == '\r')
                        break;
                    if (c == ';') {
                        // It is in a comment, so give up on this one
                        inComment = true;
                        break;
                    }
                }
                if (inComment)
                    continue;

                // Find 'Z' position and grab that value
                zPos = 0;
                for (int j = i + 3; j < len - 2; j++) {
                    char c = buf[j];
                    if (c < ' ') {
                        // Skip all white spaces...
                        while (j < len - 2 && c <= ' ') {
                            c = buf[++j];
                        }
                        // ...to make sure ";End" doesn't follow G0 .. Z#HEIGHT#
                        if (zPos != 0) {
                            //debugPrintf("Found at offset %u text: %.100s\n", zPos, &buf[zPos + 1]);
                            height = strtod(&buf[zPos + 1], NULL);
                            return true;
                        }
                        break;
                    } else if (c == ';')
                        break;
                    else if (c == 'Z')
                        zPos = j;
                }
            }
        }
    }
    return false;
}
#endif // JSON_OUTPUT
