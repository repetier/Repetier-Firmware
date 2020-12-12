/* 
    CSV Parser Library for Repetier-Firmware V2.0/SdFat
    Copyright (C) 2020 AbsoluteCatalyst (moses.github@outlook.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef _CSVParser_H
#define _CSVParser_H


enum class CSVDir {
    ABOVE = 0,
    BELOW = 1,
    PREV = 2,
    NEXT = 3,
    EXISTS = -1
};

class CSVParser {
private:
    constexpr static size_t maxCellSize = 50;
    // 50 seems to be a decent max cell character limit.
    char cellBuf[maxCellSize] {};

    fast8_t curReadColumn = 0;
    fast8_t curReadRow = 0;

    ufast8_t nextDelimType = 0; // 1 = new line 0 = comma
    uint32_t nextDelimPos = 0;  // Used to also tell if we already have a valid cell buffed

    sd_file_t* csvFile;

    /**
     * \brief Internal use.
     * Returns a pointer to the internal cell buffer.
     * 
     * \return const char* Cell buffer pointer.
     */
    inline const char* cellBufRaw() {
        return cellBuf;
    }

    /**
     * \brief Internal use.
     * Resets the current read position in the file and
     * our current cell position. Clears cell buffer.
     */
    void resetReadPos() {
        curReadColumn = curReadRow = nextDelimPos = nextDelimType = 0;
        memset(cellBuf, 0, maxCellSize);
        csvFile->rewind();
    }

    /**
     * \brief Internal use.
     * Resets our read position and incrementally scans 
     * for a field \param key string. If succesful then attempts 
     * to find the neighboring cell based on the 
     * input \param valDir direction.
     * 
     * \param[in] key String to scan for.
     * \param[in] valDir Direction of the neighboring cell to get.
     * 
     * \return true Found the field and neighboring cell. 
     * \return false Failed to find the field or neighboring cell.
     */
    bool getFieldRaw(FSTRINGPARAM(key), CSVDir valDir) {
        fast8_t foundRow = -1, foundColumn = -1;
        resetReadPos();
        while (readCurCellPos()) {
            if (foundRow == -1 && foundColumn == -1) {
                if (strcmp_P(cellBufRaw(), key) == 0) {
                    if (valDir == CSVDir::EXISTS) {
                        // Key exists, no need to do anything else.
                        return true;
                    }
                    foundRow = curReadRow;
                    foundColumn = curReadColumn;
                    if (valDir == CSVDir::ABOVE || valDir == CSVDir::PREV) {
                        resetReadPos();
                        continue;
                    }
                }
            } else {
                if (curReadColumn == foundColumn) { //vertical
                    if (valDir == CSVDir::ABOVE) {
                        if (curReadRow == (foundRow - 1)) {
                            return true;
                        }
                    } else if (valDir == CSVDir::BELOW) {
                        if (curReadRow == (foundRow + 1)) {
                            return true;
                        }
                    }
                }

                if (curReadRow == foundRow) { // Horiz
                    if (valDir == CSVDir::PREV) {
                        if (curReadColumn == (foundColumn - 1)) {
                            return true;
                        }
                    } else if (valDir == CSVDir::NEXT) {
                        if (curReadColumn == (foundColumn + 1)) {
                            return true;
                        }
                    }
                }
            }
            nextReadPos();
        }
        resetReadPos();
        //Nothing found.
        return false;
    }

    /**
     * \brief Internal use.
     * fgets to the next comma delimiter or newline
     * and saves to the internal cell buffer. 
     * Next delimiter position saved as the start of the next
     * cell.
     * 
     * \return true Next delimiter exists and we're in a valid 
     * cell.
     * \return false Unable to read or reached EOL and
     * no further cells exist.
     * 
     */
    bool readCurCellPos() {
        if (nextDelimPos) { // Cell already read
            return true;
        }

        uint32_t startPos = csvFile->curPosition();
        int bytes = csvFile->fgets(cellBuf, maxCellSize, (char*)",\n");

        if (bytes > 0) {
            nextDelimPos = csvFile->curPosition();
            // Is the next delimiter a new row or just a new cell?
            nextDelimType = cellBuf[bytes - 1] == '\n' ? 1 : 0;

            cellBuf[bytes - 1] = '\0';
            csvFile->seekSet(startPos);
            return true;
        }

        nextDelimPos = nextDelimType = 0;
        csvFile->seekSet(startPos);

        return false;
    }

    /**
     * \brief Internal use.
     * Increments to the next saved cell start delimiter.
     * Only increments if the current cell was read.
     * 
     * \return true Was able to step to the next unread cell. 
     * cell left unread.
     * \return false Unable to step, current cell unread or no
     * further cells exist.
     */
    bool nextReadPos() {
        if (!nextDelimPos) {
            return false;
        }

        if (!csvFile->seekSet(nextDelimPos)) {
            nextDelimPos = 0;
            return false;
        }

        if (nextDelimType) {
            curReadColumn = 0;
            curReadRow++;
        } else {
            curReadColumn++;
        }

        nextDelimPos = nextDelimType = 0;
        return true;
    }
    /**
     * \brief Internal use. 
     * Increments AND reads the next cell.
     * 
     * \return true Sucessfully read the next unread cell.
     * \return false Failed to read next unread cell.
     */
    bool getNextCellRaw() {
        nextReadPos();
        if (readCurCellPos()) {
            return true;
        }
        return false;
    }

public:
    /**
     * \brief Construct a new CSVParser object
     * 
     * \param[in] file CSV File to parse.
     */
    CSVParser(sd_file_t* file)
        : csvFile(file) {}
    ~CSVParser() {};

    /**
     * \brief Returns the current cell row
     * 
     * \return fast8_t Current cell row
     */
    inline fast8_t getCurRow() { return curReadRow; }

    /**
     * \brief Returns the current cell column
     * 
     * \return fast8_t Current cell column
     */
    inline fast8_t getCurCol() { return curReadColumn; }

    /**
     * \brief Attempts to find and move to the unread cell at 
     * the target row and column in the CSV file. 
     * 
     * \param[in] row Target Row.
     * \param[in] col Target Column.
     * 
     * \return true Found and moved to the row/column cell.
     * \return false Returns to previous column row if 
     * failed to find the target row/column cell.
     */
    bool seekGridPos(fast8_t row, fast8_t col) {

        if (curReadRow == row && curReadColumn == col) {
            return readCurCellPos();
        }

        int origRow = curReadRow;
        int origColumn = curReadColumn;
        uint32_t startPos = csvFile->curPosition();

        resetReadPos();

        while (readCurCellPos()) {
            nextReadPos();
            if (curReadColumn == col && curReadRow == row) {
                return true;
            }
        }

        // we failed to find a row so just go back and mark
        // cell unread.
        csvFile->seekSet(startPos);
        nextDelimPos = 0;
        curReadColumn = origColumn;
        curReadRow = origRow;

        return false;
    }

    /**
     * \brief Attempts to retrieve the current 
     * row/column (cell) value in float.
     * 
     * \param[out] output Cell float value.
     * 
     * \return true Succesfully read cell. 
     * Output set.
     * \return false Failed to read current cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type
    getCurCell(T& output) {
        if (!readCurCellPos()) {
            return false;
        }
        output = static_cast<T>(strtof(cellBufRaw(), nullptr));
        return true;
    }

    /**
     * \brief Attempts to retrieve the current 
     * row/column (cell) value in long.
     * 
     * \param[out] output Cell long value.
     * 
     * \return true Succesfully read cell. 
     * Output set.
     * \return false Failed to read current cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_integral<T>::value, bool>::type
    getCurCell(T& output) {
        if (!readCurCellPos()) {
            return false;
        }
        output = static_cast<T>(strtol(cellBufRaw(), nullptr, 10));
        return true;
    }

    /**
     * \brief Attempts to retrieve the current  
     * raw value row/column (cell) to \param output 
     * char buffer.
     * 
     * \param[out] output Cell raw char data. 
     * Uses passed char buf's length up to a 
     * limit of maxCellSize.
     * 
     * \return true Succesfully read cell. 
     * Output set.
     * \return false Failed to read current cell. 
     * Output unchanged.
     */
    template <size_t N>
    bool getCurCell(char (&output)[N]) {
        if (!readCurCellPos()) {
            return false;
        }
        memcpy(output, cellBufRaw(), N > maxCellSize ? maxCellSize : N);
        return true;
    }

    /**
     * \brief Seeks to the next unread row/column (cell)
     * and returns it's value in float.
     * Will wrap around any line endings (new rows).
     * 
     * \param[out] output Cell float value.
     * 
     * \return true Succesfully read next cell. 
     * Output set.
     * \return false Failed to read next cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type
    getNextCell(T& output) {
        if (!getNextCellRaw()) {
            return false;
        }
        output = static_cast<T>(strtof(cellBufRaw(), nullptr));
        return true;
    }

    /**
     * \brief Seeks to the next unread row/column (cell)
     * and returns it's value in long.
     * Will wrap around any line endings (new rows).
     * 
     * \param[out] output Cell long value.
     * 
     * \return true Succesfully read next cell. 
     * Output set.
     * \return false Failed to read next cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_integral<T>::value, bool>::type
    getNextCell(T& output) {
        if (!getNextCellRaw()) {
            return false;
        }
        output = static_cast<T>(strtol(cellBufRaw(), nullptr, 10));
        return true;
    }

    /**
     * \brief Seeks to the next unread row/column (cell)
     * and returns it's raw value to \param output char
     * buffer.
     * Will wrap around any line endings (new rows).
     * 
     * \param[out] output Cell raw char data. 
     * 
     * \return true Succesfully read next cell. 
     * Output set.
     * \return false Failed to read next cell. 
     * Output unchanged.
     */
    template <size_t N>
    bool getNextCell(char (&output)[N]) {
        if (!getNextCellRaw()) {
            return false;
        }
        memcpy(output, cellBufRaw(), N > maxCellSize ? maxCellSize : N);
        return true;
    }

    /**
     * \brief Checks to see if the key field 
     * exists.
     * 
     * \param[in] key Cell string to search for.
     * 
     * \return true Field exists.
     * \return false No field by key exists.
     */
    inline bool getField(FSTRINGPARAM(key)) {
        return getFieldRaw(key, CSVDir::EXISTS);
    }

    /**
     * \brief Searches for a cell matching a key string 
     * and returns the neighboring cell's value in float.
     * Neighboring cell determined by \param dir direction.
     * 
     * \param[in] key Cell string to search for.
     * \param[out] output Neighboring cell float value.
     * \param[in] dir Direction of desired neighboring cell.
     * 
     * \return true Succesfully found neighboring cell. 
     * Output set.
     * \return false Failed to find neighboring cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type
    getField(FSTRINGPARAM(key), T& output, CSVDir dir) {
        if (!getFieldRaw(key, dir)) {
            return false;
        }
        output = static_cast<T>(strtof(cellBufRaw(), nullptr));
        return true;
    }

    /**
     * \brief Searches for a cell matching a key string 
     * and returns the neighboring cell's value in long.
     * Neighboring cell determined by \param dir direction.
     * 
     * \param[in] key Cell string to search for.
     * \param[out] output Neighboring cell long value.
     * \param[in] dir Direction of desired neighboring cell.
     * 
     * \return true Succesfully found neighboring cell. 
     * Output set.
     * \return false Failed to find neighboring cell. 
     * Output unchanged.
     */
    template <typename T>
    typename std::enable_if<std::is_integral<T>::value, bool>::type
    getField(FSTRINGPARAM(key), T& output, CSVDir dir) {
        if (!getFieldRaw(key, dir)) {
            return false;
        }
        output = static_cast<T>(strtol(cellBufRaw(), nullptr, 10));
        return true;
    }

    /**
     * \brief Searches for a cell matching a key string 
     * and returns the neighboring cell's raw value to 
     * \param output char buffer. 
     * Neighboring cell determined by \param dir direction.
     * 
     * \param[in] key Cell string to search for.
     * \param[out] output Neighboring cell raw char data.
     * \param[in] dir Direction of desired neighboring cell.
     * 
     * \return true Succesfully found neighboring cell. 
     * Output set.
     * \return false Failed to find neighboring cell. 
     * Output unchanged.
     */
    template <size_t N>
    bool getField(FSTRINGPARAM(key), char (&output)[N], CSVDir dir) {
        if (!getFieldRaw(key, dir)) {
            return false;
        }
        memcpy(output, cellBufRaw(), N > maxCellSize ? maxCellSize : N);
        return true;
    }
    /**
     * \brief Small helper function to validate a CSV
     * file name extension or alternatively add 
     * the extension to the filename string 
     * automatically. 
     * 
     * Doesn't catch all bad inputs.
     * 
     * \param[out] filename
     * \return true Valid extension or added.
     * \return false Invalid name 
     * (includes dot character but no csv extension)
     */
    static bool validCSVExt(char* filename) { 
        if (strchr(filename, '.') != nullptr) {
            if (strstr_P(filename, PSTR(".csv")) == nullptr) {
                return false;
            }
        } else {
            strcat_P(filename, PSTR(".csv"));
        }
        return true;
    }
};

#endif
