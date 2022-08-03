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
enum class SDScheduledPause {
    NO_PAUSE,
    PARKING_PLANNED,
    PARKED
};

class FileSource {
public:
    FileSource(PGM_P _identifier);
    virtual bool removable() = 0;     ///< Is data source removeable
    virtual bool writeable() = 0;     ///< Can we write to source
    virtual bool usesAutomount() = 0; ///< Does it detect insertion on it's own?
    virtual bool isMounted() = 0;
    virtual void handleAutomount() = 0;
    virtual void writeCommand(GCode* code) = 0;
    PGM_P identifier;
};

template <class Detect, int cs>
class FileSourceSPI : public FileSource {
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
    size_t mountRetries;
    millis_t mountDebounceTimeMS;

#if JSON_OUTPUT
    GCodeFileInfo fileInfo;
#endif
public:
    FileSourceSPI(PGM_P name, int pos, SPIClass* _spi);
    bool removable() override { return true; }                     ///< Is data source removeable
    bool writeable() override { return true; }                     ///< Can we write to source
    bool usesAutomount() override { return Detect::pin() != 255; } ///< Does it detect insertion on it's own?
    bool isMounted() override;
    void handleAutomount() override;
    void writeCommand(GCode* code) override;
};

class FilePrintManager {
    FileSource* activeSource; // file being written to or being read
    uint32_t flags;           // 1 = printing, 2 = writing
public:
    bool isPrinting() { return (flags & 1) == 1; }
    bool isWriting() { return (flags & 2) == 2; }
    void setPrinting(bool printing);
    void setWriting(bool printing);
    void selectSource(FileSource* source);
    void writeCommand(GCode* code);
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

    void pausePrint(const bool internal = false);
    void printFullyPaused();

    void stopPrint(const bool silent = false);
    void printFullyStopped();
    bool validGCodeExtension(const char* filename);
    void mountAll();
};
extern FilePrintManager filePrintManager;
