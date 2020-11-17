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

*/

#ifndef _EEPROM_H
#define _EEPROM_H

#define EEPROM_SIGNATURE_MOTION 1
#define EEPROM_SIGNATURE_EXTRUDER 2
#define EEPROM_SIGNATURE_DELTA 3
#define EEPROM_SIGNATURE_Z_PROBE 4
#define EEPROM_SIGNATURE_HEAT_MANAGER 5
#define EEPROM_SIGNATURE_DUAL_X 6
#define EEPROM_SIGNATURE_JAM 7
#define EEPROM_SIGNATURE_LASER 8
#define EEPROM_SIGNATURE_CNC 9
#define EEPROM_SIGNATURE_CARTESIAN 10
#define EEPROM_SIGNATURE_GRID_LEVELING 11
#define EEPROM_SIGNATURE_STEPPER 12

#define EPR_MAGIC_BYTE 0
#define EPR_INTEGRITY_BYTE 1         // Here the xored sum over eeprom is stored
#define EPR_VARIATION1 2             // Used to detect eeprom layout changes
#define EPR_VARIATION2 3             // Used to detect eeprom layout changes
#define EPR_BAUDRATE 4               // Connection baudrate
#define EPR_MAX_INACTIVE_TIME 8      // When to disable heaters and setppers
#define EPR_STEPPER_INACTIVE_TIME 12 // When to disable steppers
#define EPR_PRINTING_TIME 16         // Time in seconds printing
#define EPR_PRINTING_DISTANCE 20     // Filament length printed
#define EPR_BAUDRATE2 24             // Connection baudrate for second connector
#define EPR_SELECTED_LANGUAGE 25     // Active language
#define EPR_VERSION 26               // Version id for updates in EEPROM storage
#define EPR_TONE_VOLUME 27           // Tone volume, off if 0.
#define EEPROM_PROTOCOL_VERSION 1    // Protocol version

#define EPR_START_RESERVE 40

union EEPROMVar {
    float f;
    int32_t l;
    int16_t i;
    uint8_t c;
};

enum class EEPROMType {
    FLOAT = 1,
    LONG = 2,
    INT = 3,
    BYTE = 4
};

enum class EEPROMMode {
    REPORT = 0,
    SET_VAR = 1,
    STORE = 2,
    READ = 3
};

class EEPROM {
    friend class HAL;
    static uint storePos; // where does M206 want to store
    static bool silent;   // if true it will not write it out
    static EEPROMType storeType;
    static EEPROMVar storeVar;
    static void callHandle();
    static char prefix[20];
    static void updateVariation(fast8_t data);
    static void updateVariationRecover(fast8_t data);
    static uint reservedEnd;        // Last position for reserved data
    static uint reservedRecoverEnd; // Last position for reserved data
    static unsigned int variation1, variation2;
    static unsigned int variationRecover1, variationRecover2;
    static fast8_t changedTimer;
    static uint8_t computeChecksum();
    static void updateChecksum();

public:
    static EEPROMMode mode;

    static void setSilent(bool s) { silent = s; }
    static void init();
    static void markChanged();
    static void initBaudrate();
    static void setBaudrate(int32_t val);
    static void updateDerived();
    static void timerHandler(); // gets aclled every 100ms
    /** Reserve memory in eeprom to store data. sig and version
     * are used to compute a variation checksum to reinit eeprom
     * on configuration changes.
     * sig = 1: Motion1, 2: Tool Extruder, 3: Tool Lase, 4: Tool cnc
     *       5: Heater
     *  */
    static uint reserve(uint8_t sig, uint8_t version, uint length);
    static uint reserveRecover(uint8_t sig, uint8_t version, uint length);
    static void storeDataIntoEEPROM(uint8_t corrupted = 0);
    static void readDataFromEEPROM();
    static void restoreEEPROMSettingsFromConfiguration();
    static void writeSettings();
    static void update(GCode* com);
    static void updatePrinterUsage();
    static void handleFloat(uint pos, PGM_P text, uint8_t digits, float& var);
    static void handleLong(uint pos, PGM_P text, int32_t& var);
    static void handleLong(uint pos, PGM_P text, uint32_t& var);
    static void handleInt(uint pos, PGM_P text, int16_t& var);
    static void handleByte(uint pos, PGM_P text, uint8_t& var);
    static void handleByte(uint pos, PGM_P text, int8_t& var);
    static void handleByte(uint pos, PGM_P text, int32_t& var);
    static void handleByte(uint pos, PGM_P text, bool& var);
    static void handlePrefix(PGM_P text);
    static void handlePrefix(PGM_P text, int id);
    static void removePrefix();
#if EEPROM_MODE != 0
    static float getRecoverFloat(uint pos);
    static int32_t getRecoverLong(uint pos);
    static int16_t getRecoverInt(uint pos);
    static uint8_t getRecoverByte(uint pos);
    static void setRecoverFloat(uint pos, float val);
    static void setRecoverLong(uint pos, int32_t val);
    static void setRecoverInt(uint pos, int16_t val);
    static void setRecoverByte(uint pos, uint8_t val);
    static void resetRecover();
#endif

    static inline void setVersion(uint8_t v) {
#if EEPROM_MODE != 0
        HAL::eprSetByte(EPR_VERSION, v);
        HAL::eprSetByte(EPR_INTEGRITY_BYTE, computeChecksum());
#endif
    }
    static inline uint8_t getStoredLanguage() {
#if EEPROM_MODE != 0
        return HAL::eprGetByte(EPR_SELECTED_LANGUAGE);
#else
        return 0;
#endif
    }
};
#endif
