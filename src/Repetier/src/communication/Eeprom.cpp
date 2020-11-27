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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

EEPROMMode EEPROM::mode;
uint EEPROM::storePos; // where does M206 want to store
EEPROMType EEPROM::storeType;
EEPROMVar EEPROM::storeVar;
char EEPROM::prefix[20];
uint EEPROM::reservedEnd = EPR_START_RESERVE; // Last position for reserved data
unsigned int EEPROM::variation1 = 0, EEPROM::variation2 = 0;
uint EEPROM::reservedRecoverEnd = 2; // Last position for recover data
unsigned int EEPROM::variationRecover1 = 0, EEPROM::variationRecover2 = 0;
fast8_t EEPROM::changedTimer = 0;
bool EEPROM::silent;

uint EEPROM::reserve(uint8_t sig, uint8_t version, uint length) {
    uint r = reservedEnd;
    reservedEnd += length;
    updateVariation(sig);
    updateVariation(version);
    updateVariation(static_cast<uint8_t>(length & 255));
    return r;
}

uint EEPROM::reserveRecover(uint8_t sig, uint8_t version, uint length) {
    uint r = reservedRecoverEnd;
    reservedRecoverEnd += length;
    updateVariationRecover(sig);
    updateVariationRecover(version);
    updateVariationRecover(static_cast<uint8_t>(length & 255));
    return r;
}

void EEPROM::timerHandler() {
#if EEPROM_MODE != 0
    if (changedTimer == 0) {
        return;
    }
    changedTimer--;
    if (changedTimer == 0) {
        storeDataIntoEEPROM();
    }
#endif
}

void EEPROM::markChanged() {
#if EEPROM_MODE != 0
    changedTimer = 10;
#endif
}

void EEPROM::update(GCode* com) {
    if (com->hasT() && com->hasP()) {
        storePos = com->P;
        mode = EEPROMMode::SET_VAR;
        switch (com->T) {
        case 0:
            if (com->hasS()) {
                storeVar.c = static_cast<uint8_t>(com->S);
                storeType = EEPROMType::BYTE;
            }
            break;
        case 1:
            if (com->hasS()) {
                storeVar.i = static_cast<int16_t>(com->S);
                storeType = EEPROMType::INT;
            }
            break;
        case 2:
            if (com->hasS()) {
                storeVar.l = static_cast<uint32_t>(com->S);
                storeType = EEPROMType::LONG;
            }
            break;
        case 3:
            if (com->hasX()) {
                storeVar.f = com->X;
                storeType = EEPROMType::FLOAT;
            }
            break;
        }
        callHandle();
    }
}

void EEPROM::callHandle() {
    static bool firstImport = true;
#if FEATURE_CONTROLLER != NO_CONTROLLER
    handleByte(EPR_SELECTED_LANGUAGE, Com::tLanguage, Com::selectedLanguage);
#endif
    setSilent(Printer::isNativeUSB());
    handleLong(EPR_BAUDRATE, Com::tEPRBaudrate, baudrate);
    setSilent(false);
    handleLong(EPR_PRINTING_TIME, Com::tEPRPrinterActive, Printer::printingTime);
    handleLong(EPR_MAX_INACTIVE_TIME, Com::tEPRMaxInactiveTime, maxInactiveTime);
    handleLong(EPR_STEPPER_INACTIVE_TIME, Com::tEPRStopAfterInactivty, stepperInactiveTime);
#if NUM_TOOLS > 0
    handleFloat(EPR_PRINTING_DISTANCE, Com::tEPRFilamentPrinted, 3, Printer::filamentPrintedTotal);
#endif

#if NUM_BEEPERS > 0
    handleByte(EPR_TONE_VOLUME, Com::tEPRToneVolume,
               reinterpret_cast<uint8_t&>(Printer::toneVolume = constrain(Printer::toneVolume, 0, 100)));
    BeeperSourceBase::muteAll((Printer::toneVolume <= MINIMUM_TONE_VOLUME));
#endif

    Motion1::eepromHandle(firstImport);
    ZProbeHandler::eepromHandle();
    LevelingCorrector::handleEeprom();
    Leveling::handleEeprom();
    PrinterType::eepromHandle();
    Tool::eepromHandleTools();
    // Add generic eepromHandle calls
#undef IO_TARGET
#define IO_TARGET IO_TARGET_EEPROM
#include "../io/redefine.h"
    if (mode == EEPROMMode::SET_VAR || mode == EEPROMMode::READ) {
        // Update derived data if needed
        Motion1::updateDerived();
        PrinterType::updateDerived();
        Tool::updateDerivedTools();
        // Add generic updateDerived calls
#undef IO_TARGET
#define IO_TARGET IO_TARGET_UPDATE_DERIVED
#include "../io/redefine.h"
    }
    int eCnt = 1, bCnt = 1, cCnt = 1, oCnt = 1;
    for (int i = 0; i < NUM_HEATERS; i++) {
        HeatManager* h = heaters[i];
        if (h->isExtruderHeater()) {
            EEPROM::handlePrefix(PSTR("Extr."), eCnt++);
        }
        if (h->isBedHeater()) {
            EEPROM::handlePrefix(PSTR("Bed "), bCnt++);
        }
        if (h->isChamberHeater()) {
            EEPROM::handlePrefix(PSTR("Chamber "), cCnt++);
        }
        if (h->isOtherHeater()) {
            EEPROM::handlePrefix(PSTR("Misc Heater "), oCnt++);
        }
        h->eepromHandle();
        EEPROM::removePrefix();
    }
    if (mode == EEPROMMode::READ) {
        firstImport = false;
    }
}

void EEPROM::restoreEEPROMSettingsFromConfiguration() {
#undef IO_TARGET
#define IO_TARGET IO_TARGET_RESTORE_FROM_CONFIG
#include "../io/redefine.h"
    PrinterType::restoreFromConfiguration();
    Motion1::setFromConfig();
    ZProbeHandler::eepromReset();
    LevelingCorrector::resetEeprom();
    Leveling::resetEeprom();
    markChanged();
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted) {
#if EEPROM_MODE != 0
    Com::printFLN(PSTR("Storing data to eeprom"));
    mode = EEPROMMode::STORE;
    callHandle();
    // Save version and build checksum
    HAL::eprSetByte(EPR_VERSION, EEPROM_PROTOCOL_VERSION);
    HAL::eprSetByte(EPR_MAGIC_BYTE, EEPROM_MODE);                      // Make data change permanent
    HAL::eprSetByte(EPR_VARIATION1, static_cast<uint8_t>(variation1)); // Make data change permanent
    HAL::eprSetByte(EPR_VARIATION2, static_cast<uint8_t>(variation2)); // Make data change permanent
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, computeChecksum());
#endif
}

void EEPROM::readDataFromEEPROM() {
#if EEPROM_MODE != 0
    mode = EEPROMMode::READ;
    callHandle();
    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data nor set it to older versions!
    Com::printFLN(PSTR("Detected EEPROM version:"), (int)version);
#endif
}

void EEPROM::initBaudrate() {
    // Invariant - baudrate is initialized with or without eeprom!
    baudrate = BAUDRATE;
#if EEPROM_MODE != 0
    if (HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE) {
        baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
#endif
}

void EEPROM::setBaudrate(int32_t val) {
    storePos = EPR_BAUDRATE;
    mode = EEPROMMode::SET_VAR;
    storeVar.l = val;
    storeType = EEPROMType::LONG;
    callHandle();
    storeDataIntoEEPROM();
}

void EEPROM::updateVariation(fast8_t data) {
    variation1 += data;
    if (variation1 >= 255) {
        variation1 -= 255;
    }
    variation2 += variation1;
    if (variation2 >= 255) {
        variation2 -= 255;
    }
}

void EEPROM::updateVariationRecover(fast8_t data) {
    variationRecover1 += data;
    if (variationRecover1 >= 255) {
        variationRecover1 -= 255;
    }
    variationRecover2 += variationRecover1;
    if (variationRecover2 >= 255) {
        variationRecover2 -= 255;
    }
}

#ifndef USE_CONFIGURATION_BAUD_RATE
#define USE_CONFIGURATION_BAUD_RATE 0
#endif // USE_CONFIGURATION_BAUD_RATE
void EEPROM::init() {
    silent = false;
#if EEPROM_MODE != 0
    uint8_t check = computeChecksum();
    uint8_t storedcheck = HAL::eprGetByte(EPR_INTEGRITY_BYTE);
    uint8_t var1, var2;
    prefix[0] = 0;
    var1 = HAL::eprGetByte(EPR_VARIATION1);
    var2 = HAL::eprGetByte(EPR_VARIATION2);
    bool resetRec = false;
    if (HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check && var1 == variation1 && var2 == variation2 && HAL::eprGetByte(EPR_VERSION) == EEPROM_PROTOCOL_VERSION) {
        readDataFromEEPROM();
        if (USE_CONFIGURATION_BAUD_RATE) {
            // Used if eeprom gets unusable baud rate set and communication wont work at all.
            if (HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE) {
                HAL::eprSetInt32(EPR_BAUDRATE, BAUDRATE);
                baudrate = BAUDRATE;
                uint8_t newcheck = computeChecksum();
                if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
                    HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
            }
            Com::printFLN(PSTR("EEPROM baud rate restored from configuration."));
            Com::printFLN(PSTR("RECOMPILE WITH USE_CONFIGURATION_BAUD_RATE == 0 to alter baud rate via EEPROM"));
        }
    } else {
        resetRec = true;
        storeDataIntoEEPROM(storedcheck != check);
    }
    var1 = getRecoverByte(0);
    var2 = getRecoverByte(1);
    if (resetRec || var1 != variationRecover1 || var2 != variationRecover2) {
        resetRecover();
    }
#endif
}

void EEPROM::updatePrinterUsage() {
#if EEPROM_MODE != 0
    if (Printer::filamentPrinted == 0 || (Printer::flag2 & PRINTER_FLAG2_RESET_FILAMENT_USAGE) != 0)
        return; // No miles only enabled
    uint32_t seconds = (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000;
    Printer::printingTime += seconds;
    HAL::eprSetInt32(EPR_PRINTING_TIME, seconds);
    Printer::filamentPrintedTotal += Printer::filamentPrinted * 0.001;
    Printer::filamentPrinted = 0;
    Printer::flag2 |= PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    Printer::msecondsPrinting = HAL::timeInMilliseconds();
    markChanged();
    // updateChecksum();
    Commands::reportPrinterUsage();
#endif
}

/** \brief Writes all eeprom settings to serial console.

For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void EEPROM::writeSettings() {
    mode = EEPROMMode::REPORT;
    callHandle();
}

#if EEPROM_MODE != 0

uint8_t EEPROM::computeChecksum() {
    unsigned int i;
    uint8_t checksum = 0;
    for (i = 0; i < reservedEnd; i++) {
        if (i == EPR_INTEGRITY_BYTE)
            continue;
        checksum += HAL::eprGetByte(i);
    }
    return checksum;
}

void EEPROM::updateChecksum() {
    uint8_t newcheck = computeChecksum();
    if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
        HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
}

#endif

void EEPROM::handlePrefix(PGM_P text) {
    if (mode == EEPROMMode::REPORT) {
        uint8_t i = 0;
        while (i < 19) {
            uint8_t c = pgm_read_byte(text++);
            if (!c)
                break;
            prefix[i++] = c;
        }
        prefix[i++] = 32;
        prefix[i] = 0;
    }
}

void EEPROM::handlePrefix(PGM_P text, int id) {
    if (mode == EEPROMMode::REPORT) {
        uint8_t i = 0;
        while (i < 17) {
            uint8_t c = pgm_read_byte(text++);
            if (!c)
                break;
            prefix[i++] = c;
        }
        itoa(id, &prefix[i], 10);
        i = strlen(prefix);
        prefix[i++] = 32;
        prefix[i] = 0;
    }
}

void EEPROM::removePrefix() {
    prefix[0] = 0;
}

void EEPROM::handleFloat(uint pos, PGM_P text, uint8_t digits, float& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR3, static_cast<int>(pos));
        Com::print(' ');
        Com::printFloat(var, digits);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::FLOAT) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type float"));
            } else {
                if (storeVar.f != var) {
                    var = storeVar.f;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetFloat(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetFloat(pos);
    }
#endif
}

void EEPROM::handleLong(uint pos, PGM_P text, int32_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR2, static_cast<int>(pos));
        Com::print(' ');
        Com::print(var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::LONG) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type long"));
            } else {
                if (var != storeVar.l) {
                    var = storeVar.l;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetInt32(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetInt32(pos);
    }
#endif
}

void EEPROM::handleLong(uint pos, PGM_P text, uint32_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR2, static_cast<int>(pos));
        Com::print(' ');
        Com::print(static_cast<int32_t>(var));
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::LONG) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type long"));
            } else {
                if (static_cast<int32_t>(var) != storeVar.l) {
                    var = storeVar.l;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetInt32(pos, static_cast<int32_t>(var));
    } else if (mode == EEPROMMode::READ) {
        var = static_cast<uint32_t>(HAL::eprGetInt32(pos));
    }
#endif
}

void EEPROM::handleInt(uint pos, PGM_P text, int16_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR1, static_cast<int>(pos));
        Com::print(' ');
        Com::print(var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::INT) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type int"));
            } else {
                if (var != storeVar.i) {
                    var = storeVar.i;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetInt16(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetInt16(pos);
    }
#endif
}

void EEPROM::handleByte(uint pos, PGM_P text, uint8_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR0, static_cast<int>(pos));
        Com::print(' ');
        Com::print((int)var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::BYTE) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type byte"));
            } else {
                if (var != storeVar.c) {
                    var = storeVar.c;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetByte(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetByte(pos);
    }
#endif
}

void EEPROM::handleByte(uint pos, PGM_P text, int8_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR0, static_cast<int>(pos));
        Com::print(' ');
        Com::print((int)var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::BYTE) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type byte"));
            } else {
                if (var != storeVar.c) {
                    var = storeVar.c;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetByte(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetByte(pos);
    }
#endif
}

void EEPROM::handleByte(uint pos, PGM_P text, bool& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR0, static_cast<int>(pos));
        Com::print(' ');
        Com::print((int)var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::BYTE) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type byte"));
            } else {
                if (var != storeVar.c) {
                    var = storeVar.c;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetByte(pos, var);
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetByte(pos);
    }
#endif
}

void EEPROM::handleByte(uint pos, PGM_P text, int32_t& var) {
    if (mode == EEPROMMode::REPORT && !silent && text != nullptr) {
        Com::printF(Com::tEPR0, static_cast<int>(pos));
        Com::print(' ');
        Com::print((int)var);
        Com::print(' ');
        Com::print(prefix);
        Com::printFLN(text);
        HAL::delayMilliseconds(4); // reduces somehow transmission errors
    } else if (mode == EEPROMMode::SET_VAR) {
        if (pos == storePos) {
            if (storeType != EEPROMType::BYTE) {
                Com::printErrorFLN(PSTR("Storing variable called for wrong type byte"));
            } else {
                if (static_cast<uint8_t>(var) != storeVar.c) {
                    var = storeVar.c;
                    markChanged();
                }
            }
        }
    }
#if EEPROM_MODE != 0
    else if (mode == EEPROMMode::STORE) {
        HAL::eprSetByte(pos, static_cast<uint8_t>(var));
    } else if (mode == EEPROMMode::READ) {
        var = HAL::eprGetByte(pos);
    }
#endif
}
#if EEPROM_MODE != 0
void EEPROM::resetRecover() {
    Com::printFLN(PSTR("Resetting rescue data block"));
    for (uint i = 2; i < reservedRecoverEnd; i++) {
        setRecoverByte(i, 0);
    }
    setRecoverByte(0, variationRecover1);
    setRecoverByte(1, variationRecover2);
}
float EEPROM::getRecoverFloat(uint pos) {
    return HAL::eprGetFloat(pos + reservedEnd);
}
int32_t EEPROM::getRecoverLong(uint pos) {
    return HAL::eprGetInt32(pos + reservedEnd);
}
int16_t EEPROM::getRecoverInt(uint pos) {
    return HAL::eprGetInt16(pos + reservedEnd);
}
uint8_t EEPROM::getRecoverByte(uint pos) {
    return HAL::eprGetByte(pos + reservedEnd);
}
void EEPROM::setRecoverFloat(uint pos, float val) {
    HAL::eprSetFloat(pos + reservedEnd, val);
}
void EEPROM::setRecoverLong(uint pos, int32_t val) {
    HAL::eprSetInt32(pos + reservedEnd, val);
}
void EEPROM::setRecoverInt(uint pos, int16_t val) {
    HAL::eprSetInt16(pos + reservedEnd, val);
}
void EEPROM::setRecoverByte(uint pos, uint8_t val) {
    HAL::eprSetByte(pos + reservedEnd, val);
}
#endif