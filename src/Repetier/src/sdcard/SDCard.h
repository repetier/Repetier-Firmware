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

#pragma once
#include "Repetier.h"

#if NEW_FILE_HANDLING == 0
#if SDSUPPORT

class SDCard {
public:
    SDCard();
    void writeCommand(GCode* code);
    void startPrint();
    void continuePrint();

    void pausePrint(const bool internal = false);
    void printFullyPaused();

    void stopPrint(const bool silent = false);
    void printFullyStopped();
    static bool validGCodeExtension(const char* filename);

    inline void setIndex(uint32_t newpos) {
        if (state < SDState::SD_MOUNTED || !selectedFile.isOpen()) {
            return;
        }
        selectedFilePos = newpos;
        selectedFile.seekSet(selectedFilePos);
    }
    void printStatus(const bool getFilename = false);
#if JSON_OUTPUT
    void lsJSON(const char* filename);
    void JSONFileInfo(const char* filename);
    static void printEscapeChars(const char* s);
#endif

    void ls(const char* lsDir = Com::tSlash, const bool json = false);
    void ls(sd_file_t& rootDir, const bool json = false); // Will auto close rootdir

    void mount(const bool manual);
    void unmount(const bool manual);
    void automount();

    bool selectFile(const char* filename, const bool silent = false);
    void startWrite(const char* filename);
    void deleteFile(const char* filename);

    void makeDirectory(const char* filename);

    void finishWrite();
    void finishPrint();

    void printCardInfo(bool json = false);
    bool printIfCardErrCode(); // Just prints a small hex errorcode for the lookup table
    bool getCardInfo(char* volumeLabelBuf = nullptr, uint8_t volumeLabelSize = 0, uint64_t* volumeSizeSectors = nullptr, uint64_t* usageBytes = nullptr, uint16_t* fileCount = nullptr, uint8_t* folderCount = nullptr);

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
#ifdef GLENN_DEBUG
    void writeToFile();
#endif

    uint32_t selectedFileSize;
    uint32_t selectedFilePos;

    sd_file_t selectedFile;
    sd_fsys_t fileSystem;
    SDState state;

    char volumeLabel[16u];

    SDScheduledPause scheduledPause;
    bool scheduledStop;
    millis_t lastWriteTimeMS;
    uint32_t writtenBytes;

#if JSON_OUTPUT
    GCodeFileInfo fileInfo;
#endif

private:
#if defined(ENABLE_SOFTWARE_SPI_CLASS) && ENABLE_SOFTWARE_SPI_CLASS
    SoftSpiDriver<SD_SOFT_MISO_PIN, SD_SOFT_MOSI_PIN, SD_SOFT_SCK_PIN> softSpi;
#endif

    size_t mountRetries;
    millis_t mountDebounceTimeMS;
    bool printingSilent; // Will not report when starting or finishing printing
};

extern SDCard sd;
#endif

#endif