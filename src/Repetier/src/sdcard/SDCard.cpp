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

#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
extern sd_file_t eepromFile; // eepromFile has to always be closed on unmounts.
                             // otherwise it'll cause a freeze when we try opening
                             // it again with a new sdcard of a different type.
#endif

char tempLongFilename[LONG_FILENAME_LENGTH + 1u];
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1u];
SDCardGCodeSource sdSource;
SDCard sd;
SDCard::SDCard()
    : selectedFileSize(0ul)
    , selectedFilePos(0ul)
    , state(SDState::SD_UNMOUNTED)
    , volumeLabel { 0u }
    , scheduledPause(false)
    , scheduledStop(false)
    , lastWriteTimeMS(0ul)
    , writtenBytes(0ul)
    , mountRetries(0ul)
    , mountDebounceTimeMS(0ul)
    , printingSilent(true) {
    Printer::setAutomount((SDCARDDETECT > -1));
}

void SDCard::automount() {
#if SDCARDDETECT > -1
    bool pinLevel = SDCARDDETECTINVERTED
        ? HAL::digitalRead(SDCARDDETECT)
        : !HAL::digitalRead(SDCARDDETECT);

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
#endif
}

void SDCard::mount(const bool manual) {
#if SDSS > -1
    if (state > SDState::SD_SAFE_EJECTED) {
        if (manual && state >= SDState::SD_MOUNTED) {
            // Some hosts (BTT TFT's) require a response after M21 or they'll hang.
            Com::writeToAll = true;
            Com::printFLN(PSTR("SD card ok"));
        }
        return;
    }
#if SDCARDDETECT > -1
    if (HAL::digitalRead(SDCARDDETECT) != SDCARDDETECTINVERTED) {
        return;
    }
#endif

#if defined(ENABLE_SOFTWARE_SPI_CLASS) && ENABLE_SOFTWARE_SPI_CLASS
    SdSpiConfig spiConfig = SdSpiConfig(SDSS, ENABLE_DEDICATED_SPI ? DEDICATED_SPI : SHARED_SPI, SD_SCK_MHZ(constrain(SD_SPI_SPEED_MHZ, 1ul, 50ul)), &softSpi);
#else
    SdSpiConfig spiConfig = SdSpiConfig(SDSS, ENABLE_DEDICATED_SPI ? DEDICATED_SPI : SHARED_SPI, SD_SCK_MHZ(constrain(SD_SPI_SPEED_MHZ, 1ul, 50ul)));
#endif
    if (!fileSystem.begin(spiConfig)) {
        if (mountRetries < 2u) {
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
#if SDCARDDETECT > -1
            HAL::delayMilliseconds(35ul); // wait a little more before reporting the pin state
            Com::printFLN(PSTR("Card detect pin:"),
                          HAL::digitalRead(SDCARDDETECT) ? Com::tH : Com::tL);
#endif
        } else if (!fileSystem.fatType()) {
#if SDFAT_FILE_TYPE == 3
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32/exFAT partition."));
#elif SDFAT_FILE_TYPE == 1
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32 partition."));
#else
            Com::printFLN(PSTR("Can't find a valid exFAT partition."));
#endif
        } else if (!fileSystem.chdir()) {
            Com::printFLN(PSTR("Can't open root directory."));
        }
        return;
    }

    mountRetries = 0ul;
    state = SDState::SD_MOUNTED;
    Com::printFLN(PSTR("SD card ok"));
    Com::printFLN(PSTR("SD card inserted"));

    printCardStats(); // <- collects our volumeLabel too.

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
#endif
}

void SDCard::printCardStats() {
    if (state < SDState::SD_MOUNTED) {
        return;
    }
    // Print out some basic statistics on successful mount. We also store the volume label.
    uint64_t volumeSectors = 0ul, usageBytes = 0ul;
    uint16_t fileCount = 0u;
    uint8_t folderCount = 0u;
    getCardInfo(volumeLabel, sizeof(volumeLabel), &volumeSectors, &usageBytes, &fileCount, &folderCount);

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
    Com::printF(gb ? PSTR(" GB (") : PSTR(" MB ("), fileCount);
    Com::printF(PSTR(" files, "), folderCount);
    Com::printFLN(PSTR(" folders found.)"));
}

void SDCard::unmount(const bool manual) {
    if (state <= SDState::SD_SAFE_EJECTED) {
        // in case we're unmounting manually due to sd error
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
    fileSystem.card()->spiStop();
#if SDFAT_FILE_TYPE == 3 // When using FSVolume (ExFAT & FAT)
    fileSystem.end();
#endif

    GUI::cwd[0u] = '/';
    GUI::cwd[1u] = '\0';
    GUI::folderLevel = 0u;

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED + MENU_MODE_PAUSED + MENU_MODE_SD_PRINTING, false);

    if (state == SDState::SD_PRINTING) { // unmounted while printing!
        stopPrint(true);
        GUI::setStatusP(PSTR("SD Card removed!"), GUIStatusLevel::ERROR);
    }

    scheduledPause = false; // cancel any scheduled pauses. only scheduled stops survive
    state = manual ? SDState::SD_SAFE_EJECTED : SDState::SD_UNMOUNTED;
}

bool SDCard::getCardInfo(char* volumeLabelBuf, uint8_t volumeLabelSize, uint64_t* volumeSizeSectors, uint64_t* usageBytes, uint16_t* fileCount, uint8_t* folderCount) {
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

bool SDCard::validGCodeExtension(const char* filename) {
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

bool SDCard::selectFile(const char* filename, const bool silent) {
    if (state != SDState::SD_MOUNTED || Printer::failedMode) {
        if (!silent && (state < SDState::SD_MOUNTED)) {
            Com::printFLN(Com::tNoMountedCard);
        }
        return false;
    }
    if (!validGCodeExtension(filename)) {
        if (!silent) {
            Com::printFLN(Com::tInvalidFiletype);
        }
        return false;
    }
    if (selectedFile.open(filename) && (selectedFileSize = selectedFile.fileSize())) {
        // Filename for progress view
        getFN(selectedFile, Printer::printName);
        Printer::printName[sizeof(Printer::printName) - 1u] = '\0';
        Printer::maxLayer = -1;
        selectedFilePos = 0ul;
        printingSilent = silent;
        if (!silent) {
            Com::printF(Com::tFileOpened, filename);
            Com::printFLN(Com::tSpaceSizeColon, selectedFileSize);
            Com::printFLN(Com::tFileSelected);
        }
#if JSON_OUTPUT
        fileInfo.init(selectedFile);
#endif
        return true;
    }
    selectedFile.close();
    if (!silent) {
        Com::printFLN(Com::tFileOpenFailed);
    }
    return false;
}

bool SDCard::printIfCardErrCode() {
    if (fileSystem.card()->errorCode()) {
        Com::printF(Com::tSDErrorCode, "0X");
        char buf[8u];
        buf[7u] = '\0';
        char* ptr = fmtHex(&buf[7u], fileSystem.card()->errorCode());
        Com::printFLN(ptr);
        return true;
    }
    return false;
}

void SDCard::startPrint() {
    if (state != SDState::SD_MOUNTED
        || !selectedFile.isOpen()
        || Printer::failedMode) {
        return;
    }
    scheduledStop = scheduledPause = false;
    state = SDState::SD_PRINTING;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    Printer::setPrinting(true);
    Printer::maxLayer = -1;
    Printer::currentLayer = 0;
    GUI::clearStatus();
    GCodeSource::registerSource(&sdSource);
}

void SDCard::pausePrint(const bool internal) {
    if (state != SDState::SD_PRINTING
        || Printer::isMenuMode(MENU_MODE_PAUSED)) {
        return;
    }
    Printer::setMenuMode(MENU_MODE_PAUSED, true);
#if !defined(DISABLE_PRINTMODE_ON_PAUSE) || DISABLE_PRINTMODE_ON_PAUSE == 1
    Printer::setPrinting(false);
#endif
    GCodeSource::removeSource(&sdSource);
    scheduledPause = internal;
    if (!internal) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
        EVENT_SD_PAUSE_START(internal);
        EVENT_SD_PAUSE_END(internal);
#pragma GCC diagnostic pop
    }
}

void SDCard::printFullyPaused() {
    if (!scheduledPause) {
        return;
    }
    scheduledPause = false;
    if (EVENT_SD_PAUSE_START(true)) {
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
    }
    EVENT_SD_PAUSE_END(internal);
}

void SDCard::continuePrint(const bool internal) {
    if (state != SDState::SD_PRINTING
        && Printer::isMenuMode(MENU_MODE_PAUSED)) {
        return;
    }
    if (EVENT_SD_CONTINUE_START(internal)) {
        if (internal) {
            Tool* tool = Tool::getActiveTool();
            if (tool) {
                tool->beforeContinue();
            }
            GCode::executeFString(PSTR(PAUSE_END_COMMANDS));
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
            Motion1::moveByOfficial(Motion1::tmpPosition, Motion1::maxFeedrate[E_AXIS], false);
        }
    }
    EVENT_SD_CONTINUE_END(internal);
    GCodeSource::registerSource(&sdSource);
    Printer::setPrinting(true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
}

void SDCard::stopPrint(const bool silent) {
    if (state != SDState::SD_PRINTING
        || scheduledStop) {
        return;
    }
    scheduledStop = true;
    state = SDState::SD_MOUNTED;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING + MENU_MODE_PAUSED, false);
    Printer::setPrinting(false);
    if (!silent) {
        Com::printFLN(PSTR("SD print stopped by user."));
        GUI::clearStatus();
    }
    Printer::breakLongCommand = true; // stop waiting heatup if running
}

void SDCard::printFullyStopped() {
    if (!scheduledStop) {
        return;
    }
    scheduledStop = false;
    Motion1::moveToParkPosition();
    GCodeSource::removeSource(&sdSource);
    selectedFile.close();
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
    unsigned int sum1 = 0u, sum2 = 0u; // for fletcher-16 checksum
    uint8_t buf[100u];
    uint8_t p = 2u;
    selectedFile.clearWriteError();
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
    } else {
        writtenBytes += selectedFile.write(buf, p);

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
    if (selectedFile.getWriteError()) {
        Com::printFLN(Com::tErrorWritingToFile);
    }
}

void SDCard::ls(const char* lsDir, const bool json) {
    fileSystem.chdir();
    sd_file_t dir = fileSystem.open(lsDir);
    ls(dir, json);
}

void SDCard::ls(sd_file_t& rootDir, const bool json) {
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
                SDCard::printEscapeChars(getFN(file));
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

void SDCard::printStatus(const bool getFilename) {
    if (getFilename) {
        Com::printF(Com::tCurrentOpenFile);
        if (state == SDState::SD_PRINTING) {
            Com::printFLN(Printer::printName);
        } else {
            Com::printFLN(PSTR("(no file)"));
        }
    } else {
        if (state == SDState::SD_PRINTING) {
            Com::printF(Com::tSDPrintingByte, selectedFilePos);
            Com::printFLN(Com::tSlash, selectedFileSize);
        } else {
            Com::printFLN(Com::tNotSDPrinting);
        }
    }
}

void SDCard::startWrite(const char* filename) {
    if (state != SDState::SD_MOUNTED) {
        return;
    }
    // TODO: a way to cancel host filewrites from the fw side
    fileSystem.chdir();
    if (!selectedFile.open(filename, O_CREAT | O_APPEND | O_WRONLY | O_TRUNC)) {
        Com::printFLN(Com::tOpenFailedFile, filename);
    } else {
        writtenBytes = 0ul;
        GUI::setStatusP(PSTR("Receiving file..."), GUIStatusLevel::BUSY);
        Com::printFLN(Com::tWritingToFile, filename);
        lastWriteTimeMS = HAL::timeInMilliseconds();
        state = SDState::SD_WRITING;
    }
}

void SDCard::finishWrite() {
    if (state != SDState::SD_WRITING) {
        return;
    }
    writtenBytes = selectedFile.fileSize();
    selectedFile.sync(); // exFat seems to require a manual sync for now
    selectedFile.close();
    state = SDState::SD_MOUNTED;
    Com::printFLN(Com::tDoneSavingFile);
    GUI::pop();
}

void SDCard::finishPrint() {
    if (sd.state != SDState::SD_PRINTING) {
        return;
    }
    GCodeSource::removeSource(&sdSource);
    selectedFile.close();
    sd.state = SDState::SD_MOUNTED;
    Printer::setPrinting(false);
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, false);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    if (!printingSilent) {
        Com::writeToAll = true; // tell all listeners that we are finished
        Com::printFLN(Com::tDonePrinting);
    }
    printingSilent = false;
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
    sd_file_t dir;
    fileSystem.chdir();
    if (*filename == 0) {
        dir.open(Com::tSlash);
    } else {
        if (!dir.open(filename) || !dir.isDir()) {
            dir.close();
            Com::printF(Com::tJSONErrorStart);
            Com::printF(Com::tFileOpenFailed);
            Com::printFLN(Com::tJSONErrorEnd);
            return;
        }
    }
    Com::printF(Com::tJSONDir);
    SDCard::printEscapeChars(filename);
    Com::printF(Com::tJSONFiles);
    ls(dir, true); // ls in JSON mode
    dir.close();
    Com::printFLN(Com::tJSONArrayEnd);
}

void SDCard::printEscapeChars(const char* s) {
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

void SDCard::JSONFileInfo(const char* filename) {
    sd_file_t targetFile;
    GCodeFileInfo *info, tmpInfo;
    if (strlen(filename) == 0) {
        targetFile = selectedFile;
        info = &fileInfo;
    } else {
        if (!targetFile.open(filename, O_RDONLY) || targetFile.isDir()) {
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

void GCodeFileInfo::init(sd_file_t& file) {
    this->fileSize = file.fileSize();
    this->filamentNeeded = 0.0f;
    this->objectHeight = 0.0f;
    this->layerHeight = 0.0f;
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
