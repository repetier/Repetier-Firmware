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

#include <functional>

#ifndef USB_HOST_SUPPORT // fallback is not available
#define USB_HOST_SUPPORT 0
#endif

typedef SdFat sd_fsys_t;
typedef SdBaseFile sd_file_t;
enum class SDState {
    SD_UNMOUNTED = 0,    // No SD Card detected/mounted
    SD_SAFE_EJECTED = 1, // Manually ejected by M22
    SD_HAS_ERROR = 2,    // Rare, when has error but left SD card in.
    SD_MOUNTED = 3,      // When not printing/idle
    SD_PRINTING = 4,     // While printing. Includes while paused.
    SD_WRITING = 5       // While writing to a file. 5 minute timeout from last write.
};
enum class SDScheduledPause {
    NO_PAUSE,
    PARKING_PLANNED,
    PARKED
};

typedef std::function<int(char*, int, int)> listDirectoryCallback;
// typedef int (*listDirectoryCallback)(char*, int, int);

#if USB_HOST_SUPPORT == 1
class RFUSBDrive : public SdCardInterface {
    uint8_t lastErrorCode = SD_CARD_ERROR_INIT_NOT_CALLED;
    uint32_t lastErrorLine = 0;
#if defined(STM32F4_BOARD) || defined(STM32F1_BOARD)
private:
#endif
public:
    RFUSBDrive();
    virtual ~RFUSBDrive() { }
    bool init();
    /** end use of device */
    virtual void end() override;
    /**
   * Check for FsBlockDevice busy.
   *
   * \return true if busy else false.
   */
    bool isBusy() override;
    /**
   * Read a sector.
   *
   * \param[in] sector Logical sector to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
    bool readSector(uint32_t sector, uint8_t* dst) override;

    /**
   * Read multiple sectors.
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
    bool readSectors(uint32_t sector, uint8_t* dst, size_t ns) override;

    /** \return device size in sectors. */
    uint32_t sectorCount() override;

    /** End multi-sector transfer and go to idle state.
   * \return true for success or false for failure.
   */
    bool syncDevice() override;

    /**
   * Writes a sector.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
    bool writeSector(uint32_t sector, const uint8_t* src) override;

    /**
   * Write multiple sectors.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] ns Number of sectors to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
    bool writeSectors(uint32_t sector, const uint8_t* src, size_t ns) override;

    /** CMD6 Switch mode: Check Function Set Function.
   * \param[in] arg CMD6 argument.
   * \param[out] status return status data.
   *
   * \return true for success or false for failure.
   */
    bool cardCMD6(uint32_t arg, uint8_t* status) override;

    /** Erase a range of sectors.
   *
   * \param[in] firstSector The address of the first sector in the range.
   * \param[in] lastSector The address of the last sector in the range.
   *
   * \return true for success or false for failure.
   */
    bool erase(uint32_t firstSector, uint32_t lastSector) override;
    /** \return error code. */
    uint8_t errorCode() const override;
    /** \return error data. */
    uint32_t errorData() const override;
    /** \return false by default */
    bool hasDedicatedSpi() override;
    /** \return false by default */
    bool isDedicatedSpi() override;
    /** Set SPI sharing state
   * \param[in] value desired state.
   * \return false by default.
   */
    bool setDedicatedSpi(bool value) override;
    /**
   * Read a card's CID register.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
    bool readCID(cid_t* cid) override;
    /**
   * Read a card's CSD register.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
    bool readCSD(csd_t* csd) override;
    /** Read OCR register.
   *
   * \param[out] ocr Value of OCR register.
   * \return true for success or false for failure.
   */
    bool readOCR(uint32_t* ocr) override;
    /** Read SCR register.
   *
   * \param[out] scr Value of SCR register.
   * \return true for success or false for failure.
   */
    bool readSCR(scr_t* scr) override;
    /** \return card status. */
    uint32_t status() override;
    /** Return the card type: SD V1, SD V2 or SDHC/SDXC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC.
   */
    uint8_t type() const override;
    /** Write one data sector in a multiple sector write sequence.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */

    bool writeData(const uint8_t* src) override;
    /** Start a write multiple sectors sequence.
   *
   * \param[in] sector Address of first sector in sequence.
   *
   * \return true for success or false for failure.
   */
    bool writeStart(uint32_t sector) override;
    /** End a write multiple sectors sequence.
   * \return true for success or false for failure.
   */
    bool writeStop() override;
    bool driveAvailable(); // true if mass storage device is connected
};
#endif

#if defined(HAS_SDIO_SD_SLOT) && HAS_SDIO_SD_SLOT == 1
class RFSDSdio : public SdCardInterface {
    uint8_t lastErrorCode = SD_CARD_ERROR_INIT_NOT_CALLED;
    uint32_t lastErrorLine = 0;
#if defined(STM32F4_BOARD) || defined(STM32F1_BOARD)
public:
    static SD_HandleTypeDef hsd;
    static DMA_HandleTypeDef hdma_sdio;

private:
    void goToTransferSpeed();
    void SDLowLevelInit();
    bool SDIOReadWriteBlockDMA(uint32_t block, const uint8_t* src, uint8_t* dst);
    bool SDIOInit();
    bool SDIOReadBlock(uint32_t block, uint8_t* dst);
    bool SDIOWriteBlock(uint32_t block, const uint8_t* src);
    bool SDIOIsReady();
    bool SDIOIsBusy();
    uint32_t SDIOGetCardSize();
    uint32_t SDIOGetBlocks();
#endif
public:
    virtual ~RFSDSdio() { }
    bool init();
    /** end use of device */
    virtual void end() override;
    /**
   * Check for FsBlockDevice busy.
   *
   * \return true if busy else false.
   */
    bool isBusy() override;
    /**
   * Read a sector.
   *
   * \param[in] sector Logical sector to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
    bool readSector(uint32_t sector, uint8_t* dst) override;

    /**
   * Read multiple sectors.
   *
   * \param[in] sector Logical sector to be read.
   * \param[in] ns Number of sectors to be read.
   * \param[out] dst Pointer to the location that will receive the data.
   * \return true for success or false for failure.
   */
    bool readSectors(uint32_t sector, uint8_t* dst, size_t ns) override;

    /** \return device size in sectors. */
    uint32_t sectorCount() override;

    /** End multi-sector transfer and go to idle state.
   * \return true for success or false for failure.
   */
    bool syncDevice() override;

    /**
   * Writes a sector.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
    bool writeSector(uint32_t sector, const uint8_t* src) override;

    /**
   * Write multiple sectors.
   *
   * \param[in] sector Logical sector to be written.
   * \param[in] ns Number of sectors to be written.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */
    bool writeSectors(uint32_t sector, const uint8_t* src, size_t ns) override;

    /** CMD6 Switch mode: Check Function Set Function.
   * \param[in] arg CMD6 argument.
   * \param[out] status return status data.
   *
   * \return true for success or false for failure.
   */
    bool cardCMD6(uint32_t arg, uint8_t* status) override;

    /** Erase a range of sectors.
   *
   * \param[in] firstSector The address of the first sector in the range.
   * \param[in] lastSector The address of the last sector in the range.
   *
   * \return true for success or false for failure.
   */
    bool erase(uint32_t firstSector, uint32_t lastSector) override;
    /** \return error code. */
    uint8_t errorCode() const override;
    /** \return error data. */
    uint32_t errorData() const override;
    /** \return false by default */
    bool hasDedicatedSpi() override;
    /** \return false by default */
    bool isDedicatedSpi() override;
    /** Set SPI sharing state
   * \param[in] value desired state.
   * \return false by default.
   */
    bool setDedicatedSpi(bool value) override;
    /**
   * Read a card's CID register.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
    bool readCID(cid_t* cid) override;
    /**
   * Read a card's CSD register.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
    bool readCSD(csd_t* csd) override;
    /** Read OCR register.
   *
   * \param[out] ocr Value of OCR register.
   * \return true for success or false for failure.
   */
    bool readOCR(uint32_t* ocr) override;
    /** Read SCR register.
   *
   * \param[out] scr Value of SCR register.
   * \return true for success or false for failure.
   */
    bool readSCR(scr_t* scr) override;
    /** \return card status. */
    uint32_t status() override;
    /** Return the card type: SD V1, SD V2 or SDHC/SDXC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC.
   */
    uint8_t type() const override;
    /** Write one data sector in a multiple sector write sequence.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */

    bool writeData(const uint8_t* src) override;
    /** Start a write multiple sectors sequence.
   *
   * \param[in] sector Address of first sector in sequence.
   *
   * \return true for success or false for failure.
   */
    bool writeStart(uint32_t sector) override;
    /** End a write multiple sectors sequence.
   * \return true for success or false for failure.
   */
    bool writeStop() override;
};
#endif

class FileSource {
public:
    FileSource(PGM_P _identifier);
    static void printEscapeChars(const char* s);
    virtual bool removable() = 0;     ///< Is data source removeable
    virtual bool writeable() = 0;     ///< Can we write to source
    virtual bool usesAutomount() = 0; ///< Does it detect insertion on it's own?
    virtual bool isMounted() = 0;
    virtual bool isFileOpened() = 0;
    virtual void handleAutomount() = 0;
    virtual void mount(const bool manual) = 0;
    virtual void unmount(const bool manual) = 0;
    virtual bool startWrite(const char* filename) = 0;
    virtual bool write(char* buf, uint8_t pos) = 0;
    virtual void finishWriting() = 0;
    virtual bool startRead(const char* filename, const bool silent) = 0;
    virtual int read() = 0;
    virtual void close() = 0;
    virtual void setPosition(uint32_t newPos) = 0;
    virtual void deleteFile(const char* filename) = 0;
    virtual void makeDirectory(const char* filename) = 0;
    virtual void printCardInfo(bool json) = 0;
#if JSON_OUTPUT
    virtual void lsJSON(const char* filename) = 0;
    virtual void JSONFileInfo(const char* filename) = 0;
#endif
    virtual void ls(const char* lsDir = Com::tSlash, const bool json = false) = 0;
    virtual bool listDirectory(char* dir, bool recursive, listDirectoryCallback callback, int depth = 0) = 0;
    virtual char* filenameAtIndex(char* dir, int idx) = 0;
    PGM_P identifier;
};

class FileSourceSdFatBase : public FileSource {
protected:
    SdFat fileSystem;
    sd_file_t selectedFile;
    SDState state;
    char volumeLabel[16u];

    size_t mountRetries;
    millis_t mountDebounceTimeMS;

    bool getCardInfo(char* volumeLabelBuf = nullptr, uint8_t volumeLabelSize = 0, uint64_t* volumeSizeSectors = nullptr, uint64_t* usageBytes = nullptr, uint16_t* fileCount = nullptr, uint8_t* folderCount = nullptr);
    bool printIfCardErrCode();
    void ls(sd_file_t& rootDir, const bool json);
    bool listDirectory(sd_file_t& dir, bool recursive, listDirectoryCallback callback, int depth);

public:
    FileSourceSdFatBase(PGM_P _identifier, int pos);
    bool removable() override { return true; } ///< Is data source removeable
    bool writeable() override { return true; } ///< Can we write to source
    bool isMounted() override;
    bool isFileOpened() override;
    bool startWrite(const char* filename);
    bool write(char* buf, uint8_t pos) override;
    void finishWriting() override;
    bool startRead(const char* filename, const bool silent) override;
    void close() override;
    int read() override;
    void setPosition(uint32_t newPos) override;
    void deleteFile(const char* filename) override;
    void makeDirectory(const char* filename) override;
    void unmount(const bool manual) override;
#if JSON_OUTPUT
    void lsJSON(const char* filename) override;
    void JSONFileInfo(const char* filename) override;
#endif
    void ls(const char* lsDir = Com::tSlash, const bool json = false) override;
    bool listDirectory(char* dir, bool recursive, listDirectoryCallback callback, int depth) override;
    char* filenameAtIndex(char* dir, int idx) override;
};

template <class Detect, int cs>
class FileSourceSPI : public FileSourceSdFatBase {
    int speed;
    SdSpiCard card;
    SdSpiConfig spiConfig;

public:
    FileSourceSPI(PGM_P name, int pos, SPIClass* _spi, uint32_t speed);
    void mount(const bool manual) override;
    bool usesAutomount() override { return Detect::pin() != 255; } ///< Does it detect insertion on it's own?
    void handleAutomount() override;

    void printCardInfo(bool json) override;
};

#if defined(HAS_SDIO_SD_SLOT) && HAS_SDIO_SD_SLOT == 1

template <class Detect>
class FileSourceSDIO : public FileSourceSdFatBase {
    int speed;
    RFSDSdio card;

public:
    FileSourceSDIO(PGM_P name, int pos);
    void mount(const bool manual) override;
    bool usesAutomount() override { return Detect::pin() != 255; } ///< Does it detect insertion on it's own?
    void handleAutomount() override;

    void printCardInfo(bool json) override;
};

#endif

#if USB_HOST_SUPPORT == 1
class FileSourceUSB : public FileSourceSdFatBase {
    RFUSBDrive card;

public:
    FileSourceUSB(PGM_P name, int pos);
    void mount(const bool manual) override;
    bool usesAutomount() override { return true; } ///< Does it detect insertion on it's own?
    void handleAutomount() override;

    void printCardInfo(bool json) override;
};

#endif

extern FileSource* fileSources[4];

class SDCardGCodeSource;
class Printer;
class FilePrintManager {
    friend class SDCardGCodeSource;
    friend class FileSource;
    friend class FileSourceSdFatBase;
    template <class Detect, int cs>
    friend class FileSourceSPI;
    friend class Printer;
    FileSource* selectedSource; // file being written to or being read
    uint32_t selectedFileSize;
    uint32_t selectedFilePos;

    millis_t lastWriteTimeMS;
    uint32_t writtenBytes;

    uint32_t flags; // 1 = printing, 2 = writing
    SDScheduledPause scheduledPause;
    bool scheduledStop;
    bool printingSilent; // Will not report when starting or finishing printing
#if JSON_OUTPUT
    GCodeFileInfo fileInfo;
#endif

public:
    FilePrintManager();
    bool isPrinting() { return (flags & 1) == 1; }
    bool isWriting() { return (flags & 2) == 2; }
    void setPrinting(bool printing);
    void setWriting(bool writing);
    void selectSource(FileSource* source);
    void startWrite(const char* filename);
    void writeCommand(GCode* code);
    void finishWriting();
    bool writingTimedOut(millis_t time);

#if JSON_OUTPUT
    void lsJSON(const char* filename);
    void JSONFileInfo(const char* filename);
#endif

    void ls(const char* lsDir = Com::tSlash, const bool json = false);

    SDScheduledPause getScheduledPause() { return scheduledPause; }
    bool isStopScheduled() { return scheduledStop; }
    void setScheduledStop(bool s) { scheduledStop = s; }
    /**
     * @brief Get the Mounted Source object
     * 
     * Returns the first mounted sd card if no Tx is given.
     * If Tx is given returns that source if it is mounted and available, otherwise nullptr.
     * 
     * @param pos Position, 255 = autodetect first 
     * @return FileSource* Found source or nullptr
     */
    FileSource* getMountedSource(uint8_t pos);
    /**
     * @brief Handles mounting of storage systems if supported,
     */
    void handleAutomount();
    void startPrint();
    void continuePrint();
    void printStatus(const bool getFilename = false);

    void pausePrint(const bool internal = false);
    void printFullyPaused();

    void stopPrint(const bool silent = false);
    void printFullyStopped();
    bool validGCodeExtension(const char* filename);
    void mountAll();
    void finishPrint();
    bool endReached() { return selectedFilePos == selectedFileSize; }
    int read();
    bool startRead(const char* filename, const bool silent);
    void close();
    void deleteFile(const char* filename);
    void makeDirectory(const char* filename);
    float getProgress() { return (static_cast<float>(selectedFilePos) * 100.0) / static_cast<float>(selectedFileSize); }
    void unmount(bool manual, int pos = 255);
    void mount(bool manual, int pos); ///< pos = 255 for first available
    void setPosition(uint32_t newPos);
    void printCardInfo(bool json);
    int posFor(FileSource* s);
    void setSource(FileSource* source) {
        selectedSource = source;
    }
    FileSource* getSelectedSource() { return selectedSource; }
};
extern FilePrintManager filePrintManager;
