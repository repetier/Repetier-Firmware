#pragma once
#include "Repetier.h"

#if SDSUPPORT
typedef SdFat sd_fsys_t;
typedef SdBaseFile sd_file_t;
enum class SDState {
    SD_UNMOUNTED,    // No SD Card detected/mounted
    SD_SAFE_EJECTED, // Manually ejected by M22
    SD_HAS_ERROR,    // Rare, when has error but left SD card in.
    SD_MOUNTED,      // When not printing/idle
    SD_PRINTING,     // While printing. Includes while paused.
    SD_WRITING       // While writing to a file. 5 minute timeout from last write.
};

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

    bool scheduledPause;
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