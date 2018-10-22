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

int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;
volatile uint8_t executePeriodical = 0;
unsigned int counterPeriodical = 0;
uint8_t counter500ms = 5;

void Commands::commandLoop() {
//while(true) {
#ifdef DEBUG_PRINT
    debugWaitLoop = 1;
#endif
    if (!Printer::isBlockingReceive()) {
        GCode::readFromSerial();
        GCode* code = GCode::peekCurrentCommand();
        //UI_SLOW; // do longer timed user interface action
        UI_MEDIUM; // do check encoder
        if (code) {
#if SDSUPPORT
            if (sd.savetosd) {
                if (!(code->hasM() && code->M == 29)) // still writing to file
                    sd.writeCommand(code);
                else
                    sd.finishWrite();
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }
    } else {
        GCode::keepAlive(Paused);
        UI_MEDIUM;
    }
    Printer::defaultLoopActions();
    //}
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {
    Printer::handleInterruptEvent();
    FirmwareEvent::handleEvents();
#if EMERGENCY_PARSER
    GCodeSource::prefetchAll();
#endif
    EVENT_PERIODICAL;
#if defined(DOOR_PIN) && DOOR_PIN > -1
    if (Printer::updateDoorOpen()) {
        Tool* tool = Tool::getActiveTool();
        if (tool) {
            tool->shutdown();
        }
    }
#endif
#undef IO_TARGET
#define IO_TARGET 15
#include "../io/redefine.h"

    if (!executePeriodical)
        return; // gets true every 100ms
    executePeriodical = 0;
    EEPROM::timerHandler(); // store changes after timeout
    // include generic 100ms calls
#undef IO_TARGET
#define IO_TARGET 3
#include "../io/redefine.h"
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

    // Report temperatures every second, so we do not need to send M105
    if (Printer::isAutoreportTemp()) {
        millis_t now = HAL::timeInMilliseconds();
        if (now - Printer::lastTempReport > 1000) {
            Printer::lastTempReport = now;
            Commands::printTemperatures();
        }
    }

    EVENT_TIMER_100MS;
    // Extruder::manageTemperatures();
    if (--counter500ms == 0) {
        counter500ms = 5;
#undef IO_TARGET
#define IO_TARGET 12
#include "../io/redefine.h"
        EVENT_TIMER_500MS;
    }
    // If called from queueDelta etc. it is an error to start a new move since it
    // would invalidate old computation resulting in unpredicted behavior.
    // lcd controller can start new moves, so we disallow it if called from within
    // a move command.
    UI_SLOW(allowNewMoves);
}

/** \brief Waits until movement cache is empty.

Some commands expect no movement, before they can execute. This function
waits, until the steppers are stopped. In the meanwhile it buffers incoming
commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves() {
#ifdef DEBUG_PRINT
    debugWaitLoop = 8;
#endif
    while (Motion1::length) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(Processing, 3);
        UI_MEDIUM;
    }
}

void Commands::waitMS(uint32_t wait) {
    millis_t end = HAL::timeInMilliseconds() + wait;
    while (static_cast<int32_t>(end - HAL::timeInMilliseconds()) > 0) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(Processing, 3);
        UI_MEDIUM;
    }
}

void Commands::waitUntilEndOfAllBuffers() {
    GCode* code = NULL;
#ifdef DEBUG_PRINT
    debugWaitLoop = 9;
#endif
    while (Motion1::length || (code != NULL)) {
        //GCode::readFromSerial();
        code = GCode::peekCurrentCommand();
        UI_MEDIUM; // do check encoder
        if (code) {
#if SDSUPPORT
            if (sd.savetosd) {
                if (!(code->hasM() && code->M == 29)) // still writing to file
                    sd.writeCommand(code);
                else
                    sd.finishWrite();
#if ECHO_ON_EXECUTE
                code->echoCommand();
#endif
            } else
#endif
                Commands::executeGCode(code);
            code->popCurrentCommand();
        }
        Commands::checkForPeriodicalActions(false); // only called from memory
        UI_MEDIUM;
    }
}

void Commands::printTemperatures(bool showRaw) {
    int error;
#if NUM_TOOLS > 0
    Tool* t = Tool::getActiveTool();
    if (t != nullptr && t->supportsTemperatures()) {
        t->getHeater()->reportTemperature('T', -1);
    }
#if NUM_TOOLS > 1
    for (int i = 0; i < NUM_TOOLS; i++) {
        t = Tool::getTool(i);
        if (t != nullptr && t->supportsTemperatures()) {
            t->getHeater()->reportTemperature('T', i);
        }
    }
#endif
#endif
#if NUM_HEATED_BEDS > 0
    for (int i = 0; i < NUM_HEATED_BEDS; i++) {
        heatedBeds[i]->reportTemperature('B', i > 0 ? i : -1);
    }
#endif

#ifdef FAKE_CHAMBER
    Com::printF(PSTR(" C:"), extruder[0].tempControl.currentTemperatureC);
    Com::printF(Com::tSpaceSlash, extruder[0].tempControl.targetTemperatureC, 0);
    Com::printF(PSTR(" @:"), (pwm_pos[extruder[0].tempControl.pwmIndex]));
#endif
    Com::println();
}
void Commands::changeFeedrateMultiply(int factor) {
    if (factor < 25)
        factor = 25;
    if (factor > 500)
        factor = 500;
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

void Commands::changeFlowrateMultiply(int factor) {
    if (factor < 25)
        factor = 25;
    if (factor > 200)
        factor = 200;
    Printer::extrudeMultiply = factor;
    // TODO: volumetric extrusion
    //if (Extruder::current->diameter <= 0)
    Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
    //else
    //    Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
    Com::printFLN(Com::tFlowMultiply, factor);
}

void Commands::reportPrinterUsage() {
#if EEPROM_MODE != 0
    float dist = Printer::filamentPrinted * 0.001 + Printer::filamentPrintedTotal;
    Com::printF(Com::tPrintedFilament, dist, 2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
    for (uint8_t i = 0; i < NUM_TOOLS; i++) {
        Tool* t = Tool::getTool(i);
        if (t->getHeater() != nullptr && t->getHeater()->isEnabled()) {
            alloff = false;
        }
    }
    int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + Printer::printingTime;
    int32_t tmp = seconds / 86400;
    seconds -= tmp * 86400;
    Com::printF(Com::tPrintingTime, tmp);
    tmp = seconds / 3600;
    Com::printF(Com::tSpaceDaysSpace, tmp);
    seconds -= tmp * 3600;
    tmp = seconds / 60;
    Com::printF(Com::tSpaceHoursSpace, tmp);
    Com::printFLN(Com::tSpaceMin);
#endif
}
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_DIGIPOT
// Digipot methods for controling current and microstepping

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
int digitalPotWrite(int address, uint16_t value) { // From Arduino DigitalPotControl example
    if (value > 255)
        value = 255;
    WRITE(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    HAL::spiSend(address);     //  send in the address and value via SPI:
    HAL::spiSend(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void setMotorCurrent(uint8_t driver, uint16_t current) {
    if (driver > 4)
        return;
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}

void setMotorCurrentPercent(uint8_t channel, float level) {
    uint16_t raw_level = (level * 255 / 100);
    setMotorCurrent(channel, raw_level);
}
#endif

void motorCurrentControlInit() { //Initialize Digipot Motor Current
#if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    HAL::spiInit(0); //SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for (int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;
    for (int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrent(i, digipot_motor_current[i]);
#endif
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_LTC2600

void setMotorCurrent(uint8_t channel, unsigned short level) {
    if (channel >= LTC2600_NUM_CHANNELS)
        return;
    const uint8_t ltc_channels[] = LTC2600_CHANNELS;
    if (channel > LTC2600_NUM_CHANNELS)
        return;
    uint8_t address = ltc_channels[channel];
    fast8_t i;

    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    // configure the pins
    WRITE(LTC2600_CS_PIN, HIGH);
    SET_OUTPUT(LTC2600_CS_PIN);
    WRITE(LTC2600_SCK_PIN, LOW);
    SET_OUTPUT(LTC2600_SCK_PIN);
    WRITE(LTC2600_SDI_PIN, LOW);
    SET_OUTPUT(LTC2600_SDI_PIN);

    // enable the command interface of the LTC2600
    WRITE(LTC2600_CS_PIN, LOW);

    // transfer command and address
    for (i = 7; i >= 0; i--) {
        WRITE(LTC2600_SDI_PIN, address & (0x01 << i));
        WRITE(LTC2600_SCK_PIN, 1);
        WRITE(LTC2600_SCK_PIN, 0);
    }

    // transfer the data word
    for (i = 15; i >= 0; i--) {
        WRITE(LTC2600_SDI_PIN, level & (0x01 << i));
        WRITE(LTC2600_SCK_PIN, 1);
        WRITE(LTC2600_SCK_PIN, 0);
    }

    // disable the ommand interface of the LTC2600 -
    // this carries out the specified command
    WRITE(LTC2600_CS_PIN, HIGH);

} // setLTC2600
void setMotorCurrentPercent(uint8_t channel, float level) {
    if (level > 100.0f)
        level = 100.0f;
    uint16_t raw_level = static_cast<uint16_t>((long)level * 65535L / 100L);
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() { //Initialize LTC2600 Motor Current
    uint8_t i;
#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for (int i = 0; i < LTC2600_NUM_CHANNELS; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const unsigned int ltc_current[] = MOTOR_CURRENT;
    for (i = 0; i < LTC2600_NUM_CHANNELS; i++) {
        setMotorCurrent(i, ltc_current[i]);
    }
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_ALLIGATOR
void setMotorCurrent(uint8_t channel, unsigned short value) {
    if (channel >= 7) // max channel (X,Y,Z,E0,E1,E2,E3)
        return;
    if (value > 255)
        value = 255;

    uint8_t externalDac_buf[2] = { 0x10, 0x00 };

    if (channel > 3)
        externalDac_buf[0] |= (7 - channel << 6);
    else
        externalDac_buf[0] |= (3 - channel << 6);

    externalDac_buf[0] |= (value >> 4);
    externalDac_buf[1] |= (value << 4);

    // All SPI chip-select HIGH
    WRITE(DAC0_SYNC, HIGH);
    WRITE(DAC1_SYNC, HIGH);
    WRITE(SPI_EEPROM1_CS, HIGH);
    WRITE(SPI_EEPROM2_CS, HIGH);
    WRITE(SPI_FLASH_CS, HIGH);
    WRITE(SDSS, HIGH);

    if (channel > 3) { // DAC Piggy E1,E2,E3
        WRITE(DAC1_SYNC, LOW);
        HAL::delayMicroseconds(2);
        WRITE(DAC1_SYNC, HIGH);
        HAL::delayMicroseconds(2);
        WRITE(DAC1_SYNC, LOW);
    } else { // DAC onboard X,Y,Z,E0
        WRITE(DAC0_SYNC, LOW);
        HAL::delayMicroseconds(2);
        WRITE(DAC0_SYNC, HIGH);
        HAL::delayMicroseconds(2);
        WRITE(DAC0_SYNC, LOW);
    }

    HAL::delayMicroseconds(2);
    HAL::spiSend(SPI_CHAN_DAC, externalDac_buf, 2);
}

void setMotorCurrentPercent(uint8_t channel, float level) {
    uint16_t raw_level = (level * 255 / 100);
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() {                 //Initialize Motor Current
    uint8_t externalDac_buf[2] = { 0x20, 0x00 }; //all off

    // All SPI chip-select HIGH
    WRITE(DAC0_SYNC, HIGH);
    WRITE(DAC1_SYNC, HIGH);
    WRITE(SPI_EEPROM1_CS, HIGH);
    WRITE(SPI_EEPROM2_CS, HIGH);
    WRITE(SPI_FLASH_CS, HIGH);
    WRITE(SDSS, HIGH);

    // init onboard DAC
    WRITE(DAC0_SYNC, LOW);
    HAL::delayMicroseconds(2);
    WRITE(DAC0_SYNC, HIGH);
    HAL::delayMicroseconds(2);
    WRITE(DAC0_SYNC, LOW);

    HAL::spiSend(SPI_CHAN_DAC, externalDac_buf, 2);
    WRITE(DAC0_SYNC, HIGH);

#if NUM_EXTRUDER > 1
    // init Piggy DAC
    WRITE(DAC1_SYNC, LOW);
    HAL::delayMicroseconds(2);
    WRITE(DAC1_SYNC, HIGH);
    HAL::delayMicroseconds(2);
    WRITE(DAC1_SYNC, LOW);

    HAL::spiSend(SPI_CHAN_DAC, externalDac_buf, 2);
    WRITE(DAC1_SYNC, HIGH);
#endif

#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for (int i = 0; i < NUM_EXTRUDER + 3; i++)
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;
    for (uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
        setMotorCurrent(i, digipot_motor_current[i]);
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
uint8_t _intVref[] = { MCP4728_VREF, MCP4728_VREF, MCP4728_VREF, MCP4728_VREF };
uint8_t _gain[] = { MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN };
uint8_t _powerDown[] = { 0, 0, 0, 0 };
int16_t dac_motor_current[] = { 0, 0, 0, 0 };

uint8_t _intVrefEp[] = { MCP4728_VREF, MCP4728_VREF, MCP4728_VREF, MCP4728_VREF };
uint8_t _gainEp[] = { MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN };
uint8_t _powerDownEp[] = { 0, 0, 0, 0 };
int16_t _valuesEp[] = { 0, 0, 0, 0 };

uint8_t dac_stepper_channel[] = MCP4728_STEPPER_ORDER;

int dacSimpleCommand(uint8_t simple_command) {
    HAL::i2cStartWait(MCP4728_GENERALCALL_ADDRESS + I2C_WRITE);
    HAL::i2cWrite(simple_command);
    HAL::i2cStop();
}

void dacReadStatus() {
    HAL::delayMilliseconds(500);
    HAL::i2cStartWait(MCP4728_I2C_ADDRESS | I2C_READ);

    for (int i = 0; i < 8; i++) { // 2 sets of 4 Channels (1 EEPROM, 1 Runtime)
        uint8_t deviceID = HAL::i2cReadAck();
        uint8_t hiByte = HAL::i2cReadAck();
        uint8_t loByte = ((i < 7) ? HAL::i2cReadAck() : HAL::i2cReadNak());

        uint8_t isEEPROM = (deviceID & 0B00001000) >> 3;
        uint8_t channel = (deviceID & 0B00110000) >> 4;
        if (isEEPROM == 1) {
            _intVrefEp[channel] = (hiByte & 0B10000000) >> 7;
            _gainEp[channel] = (hiByte & 0B00010000) >> 4;
            _powerDownEp[channel] = (hiByte & 0B01100000) >> 5;
            _valuesEp[channel] = word((hiByte & 0B00001111), loByte);
        } else {
            _intVref[channel] = (hiByte & 0B10000000) >> 7;
            _gain[channel] = (hiByte & 0B00010000) >> 4;
            _powerDown[channel] = (hiByte & 0B01100000) >> 5;
            dac_motor_current[channel] = word((hiByte & 0B00001111), loByte);
        }
    }

    HAL::i2cStop();
}

void dacAnalogUpdate(bool saveEEPROM = false) {
    uint8_t dac_write_cmd = MCP4728_CMD_SEQ_WRITE;

    HAL::i2cStartWait(MCP4728_I2C_ADDRESS + I2C_WRITE);
    if (saveEEPROM)
        HAL::i2cWrite(dac_write_cmd);

    for (int i = 0; i < MCP4728_NUM_CHANNELS; i++) {
        uint16_t level = dac_motor_current[i];

        uint8_t highbyte = (_intVref[i] << 7 | _gain[i] << 4 | (uint8_t)((level) >> 8));
        uint8_t lowbyte = ((uint8_t)((level)&0xff));
        dac_write_cmd = MCP4728_CMD_MULTI_WRITE | (i << 1);

        if (!saveEEPROM)
            HAL::i2cWrite(dac_write_cmd);
        HAL::i2cWrite(highbyte);
        HAL::i2cWrite(lowbyte);
    }

    HAL::i2cStop();

    // Instruct the MCP4728 to reflect our updated value(s) on its DAC Outputs
    dacSimpleCommand((uint8_t)MCP4728_CMD_GC_UPDATE); // MCP4728 General Command Software Update (Update all DAC Outputs to reflect settings)

    // if (saveEEPROM) dacReadStatus(); // Not necessary, just a read-back sanity check.
}

void dacCommitEeprom() {
    dacAnalogUpdate(true);
    dacReadStatus(); // Refresh EEPROM Values with values actually stored in EEPROM. .
}

void dacPrintSet(int dacChannelSettings[], const char* dacChannelPrefixes[]) {
    for (int i = 0; i < MCP4728_NUM_CHANNELS; i++) {
        uint8_t dac_channel = dac_stepper_channel[i]; // DAC Channel is a mapped lookup.
        Com::printF(dacChannelPrefixes[i], ((float)dacChannelSettings[dac_channel] * 100 / MCP4728_VOUT_MAX));
        Com::printF(Com::tSpaceRaw);
        Com::printFLN(Com::tColon, dacChannelSettings[dac_channel]);
    }
}

void dacPrintValues() {
    const char* dacChannelPrefixes[] = { Com::tSpaceXColon, Com::tSpaceYColon, Com::tSpaceZColon, Com::tSpaceEColon };

    Com::printFLN(Com::tMCPEpromSettings);
    dacPrintSet(_valuesEp, dacChannelPrefixes); // Once for the EEPROM set

    Com::printFLN(Com::tMCPCurrentSettings);
    dacPrintSet(dac_motor_current, dacChannelPrefixes); // And another for the RUNTIME set
}

void setMotorCurrent(uint8_t xyz_channel, uint16_t level) {
    if (xyz_channel >= MCP4728_NUM_CHANNELS)
        return;
    uint8_t stepper_channel = dac_stepper_channel[xyz_channel];
    dac_motor_current[stepper_channel] = level < MCP4728_VOUT_MAX ? level : MCP4728_VOUT_MAX;
    dacAnalogUpdate();
}

void setMotorCurrentPercent(uint8_t channel, float level) {
    uint16_t raw_level = (level * MCP4728_VOUT_MAX / 100);
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() {                     //Initialize MCP4728 Motor Current
    HAL::i2cInit(400000);                            // Initialize the i2c bus.
    dacSimpleCommand((uint8_t)MCP4728_CMD_GC_RESET); // MCP4728 General Command Reset
    dacReadStatus();                                 // Load Values from EEPROM.

    for (int i = 0; i < MCP4728_NUM_CHANNELS; i++) {
        setMotorCurrent(dac_stepper_channel[i], _valuesEp[i]); // This is not strictly necessary, but serves as a good sanity check to ensure we're all on the same page.
    }
}
#endif

#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstepMS(uint8_t driver, int8_t ms1, int8_t ms2) {
    if (ms1 > -1)
        switch (driver) {
        case 0:
#if X_MS1_PIN > -1
            WRITE(X_MS1_PIN, ms1);
#endif
            break;
        case 1:
#if Y_MS1_PIN > -1
            WRITE(Y_MS1_PIN, ms1);
#endif
            break;
        case 2:
#if Z_MS1_PIN > -1
            WRITE(Z_MS1_PIN, ms1);
#endif
            break;
        case 3:
#if E0_MS1_PIN > -1
            WRITE(E0_MS1_PIN, ms1);
#endif
            break;
        case 4:
#if E1_MS1_PIN > -1
            WRITE(E1_MS1_PIN, ms1);
#endif
            break;
        }
    if (ms2 > -1)
        switch (driver) {
        case 0:
#if X_MS2_PIN > -1
            WRITE(X_MS2_PIN, ms2);
#endif
            break;
        case 1:
#if Y_MS2_PIN > -1
            WRITE(Y_MS2_PIN, ms2);
#endif
            break;
        case 2:
#if Z_MS2_PIN > -1
            WRITE(Z_MS2_PIN, ms2);
#endif
            break;
        case 3:
#if E0_MS2_PIN > -1
            WRITE(E0_MS2_PIN, ms2);
#endif
            break;
        case 4:
#if E1_MS2_PIN > -1
            WRITE(E1_MS2_PIN, ms2);
#endif
            break;
        }
}

void microstepMode(uint8_t driver, uint8_t stepping_mode) {
    switch (stepping_mode) {
    case 1:
        microstepMS(driver, MICROSTEP1);
        break;
    case 2:
        microstepMS(driver, MICROSTEP2);
        break;
    case 4:
        microstepMS(driver, MICROSTEP4);
        break;
    case 8:
        microstepMS(driver, MICROSTEP8);
        break;
    case 16:
        microstepMS(driver, MICROSTEP16);
        break;
    case 32:
        microstepMS(driver, MICROSTEP32);
        break;
    }
}

void microstepReadings() {
    Com::printFLN(Com::tMS1MS2Pins);
#if X_MS1_PIN > -1 && X_MS2_PIN > -1
    Com::printF(Com::tXColon, READ(X_MS1_PIN));
    Com::printFLN(Com::tComma, READ(X_MS2_PIN));
#elif X_MS1_PIN > -1
    Com::printFLN(Com::tXColon, READ(X_MS1_PIN));
#endif
#if Y_MS1_PIN > -1 && Y_MS2_PIN > -1
    Com::printF(Com::tYColon, READ(Y_MS1_PIN));
    Com::printFLN(Com::tComma, READ(Y_MS2_PIN));
#elif Y_MS1_PIN > -1
    Com::printFLN(Com::tYColon, READ(Y_MS1_PIN));
#endif
#if Z_MS1_PIN > -1 && Z_MS2_PIN > -1
    Com::printF(Com::tZColon, READ(Z_MS1_PIN));
    Com::printFLN(Com::tComma, READ(Z_MS2_PIN));
#elif Z_MS1_PIN > -1
    Com::printFLN(Com::tZColon, READ(Z_MS1_PIN));
#endif
#if E0_MS1_PIN > -1 && E0_MS2_PIN > -1
    Com::printF(Com::tE0Colon, READ(E0_MS1_PIN));
    Com::printFLN(Com::tComma, READ(E0_MS2_PIN));
#elif E0_MS1_PIN > -1
    Com::printFLN(Com::tE0Colon, READ(E0_MS1_PIN));
#endif
#if E1_MS1_PIN > -1 && E1_MS2_PIN > -1
    Com::printF(Com::tE1Colon, READ(E1_MS1_PIN));
    Com::printFLN(Com::tComma, READ(E1_MS2_PIN));
#elif E1_MS1_PIN > -1
    Com::printFLN(Com::tE1Colon, READ(E1_MS1_PIN));
#endif
}
#endif

void microstepInit() {
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
#if X_MS1_PIN > -1
    SET_OUTPUT(X_MS1_PIN);
#endif
#if Y_MS1_PIN > -1
    SET_OUTPUT(Y_MS1_PIN);
#endif
#if Z_MS1_PIN > -1
    SET_OUTPUT(Z_MS1_PIN);
#endif
#if E0_MS1_PIN > -1
    SET_OUTPUT(E0_MS1_PIN);
#endif
#if E1_MS1_PIN > -1
    SET_OUTPUT(E1_MS1_PIN);
#endif
#if X_MS2_PIN > -1
    SET_OUTPUT(X_MS2_PIN);
#endif
#if Y_MS2_PIN > -1
    SET_OUTPUT(Y_MS2_PIN);
#endif
#if Z_MS2_PIN > -1
    SET_OUTPUT(Z_MS2_PIN);
#endif
#if E0_MS2_PIN > -1
    SET_OUTPUT(E0_MS2_PIN);
#endif
#if E1_MS2_PIN > -1
    SET_OUTPUT(E1_MS2_PIN);
#endif
    for (int i = 0; i <= 4; i++)
        microstepMode(i, microstep_modes[i]);
#endif
}

/**
\brief Execute the Arc command stored in com.
*/
#if ARC_SUPPORT
void Commands::processArc(GCode* com) {
    float position[Z_AXIS_ARRAY];
    Printer::realPosition(position[X_AXIS], position[Y_AXIS], position[Z_AXIS]);
    // TODO: Make Arc work again
    //if(!Printer::setDestinationStepsFromGCode(com)) return; // For X Y Z E F
    float offset[2] = { Printer::convertToMM(com->hasI() ? com->I : 0), Printer::convertToMM(com->hasJ() ? com->J : 0) };
    float target[E_AXIS_ARRAY] = { Printer::realXPosition(), Printer::realYPosition(), Printer::realZPosition(), Printer::destinationSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS] };
    float r;
    if (com->hasR()) {
        /*
        We need to calculate the center of the circle that has the designated radius and passes
        through both the current position and the target position. This method calculates the following
        set of equations where [x,y] is the vector from current to target position, d == magnitude of
        that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
        the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
        length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
        [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

        d^2 == x^2 + y^2
        h^2 == r^2 - (d/2)^2
        i == x/2 - y/d*h
        j == y/2 + x/d*h

        O <- [i,j]
        -  |
        r      -     |
        -        |
        -           | h
        -              |
        [0,0] ->  C -----------------+--------------- T  <- [x,y]
        | <------ d/2 ---->|

        C - Current position
        T - Target position
        O - center of circle that pass through both C and T
        d - distance from C to T
        r - designated radius
        h - distance from center of CT to O

        Expanding the equations:

        d -> sqrt(x^2 + y^2)
        h -> sqrt(4 * r^2 - x^2 - y^2)/2
        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

        Which can be written:

        i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
        j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

        Which we for size and speed reasons optimize to:

        h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
        i = (x - (y * h_x2_div_d))/2
        j = (y + (x * h_x2_div_d))/2

        */
        r = Printer::convertToMM(com->R);
        // Calculate the change in position along each selected axis
        double x = target[X_AXIS] - position[X_AXIS];
        double y = target[Y_AXIS] - position[Y_AXIS];

        double h_x2_div_d = -sqrt(4 * r * r - x * x - y * y) / hypot(x, y); // == -(h * 2 / d)
        // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
        // real CNC, and thus - for practical reasons - we will terminate promptly:
        if (isnan(h_x2_div_d)) {
            Com::printErrorFLN(Com::tInvalidArc);
            return;
        }
        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
        if (com->G == 3) {
            h_x2_div_d = -h_x2_div_d;
        }

        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
        the left hand circle will be generated - when it is negative the right hand circle is generated.


        T  <-- Target position

        ^
        Clockwise circles with this center         |          Clockwise circles with this center will have
        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
        \         |          /
        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
        |
        |

        C  <-- Current position                                 */

        // Negative R is g-code-alias for "I want a circle with more than 180 degrees of travel" (go figure!),
        // even though it is advised against ever generating such circles in a single line of g-code. By
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the inadvisable long arcs as prescribed.
        if (r < 0) {
            h_x2_div_d = -h_x2_div_d;
            r = -r; // Finished with r. Set to positive for mc_arc
        }
        // Complete the operation by calculating the actual center of the arc
        offset[0] = 0.5 * (x - (y * h_x2_div_d));
        offset[1] = 0.5 * (y + (x * h_x2_div_d));

    } else {                             // Offset mode specific computations
        r = hypot(offset[0], offset[1]); // Compute arc radius for arc
    }
    // Set clockwise/counter-clockwise sign for arc computations
    uint8_t isclockwise = com->G == 2;
    // Trace the arc
    PrintLine::arc(position, target, offset, r, isclockwise);
}
#endif

/**
\brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode* com) {
    if (EVENT_UNHANDLED_G_CODE(com)) {
        previousMillisCmd = HAL::timeInMilliseconds();
        return;
    }
    switch (com->G) {
    case 0: // G0 -> G1
    case 1: // G1
        GCode_0_1(com);
        break;
    case 2: // CW Arc
    case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        GCode_2_3(com);
        break;
    case 4: // G4 dwell
        GCode_4(com);
        break;
    case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
        GCode_10(com);
        break;
    case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
        GCode_11(com);
        break;
    case 20: // G20 Units to inches
        GCode_20(com);
        break;
    case 21: // G21 Units to mm
        GCode_21(com);
        break;
    case 28:
        GCode_28(com);
        break;
    case 29:
        GCode_29(com);
        break;
    case 30:
        GCode_30(com);
        break;
    case 31:
        GCode_31(com);
        break;
    case 32: // G32 Auto-Bed leveling
        GCode_32(com);
        break;
    case 33:
        GCode_33(com);
        break;
    case 90: // G90
        GCode_90(com);
        break;
    case 91:
        GCode_91(com);
        break;
    case 92:
        GCode_92(com);
        break;
    case 100:
        GCode_134(com);
        break;
    case 131:
        GCode_131(com);
        break;
    case 132:
        GCode_132(com);
        break;
    case 133:
        GCode_133(com);
        break;
    case 134:
        GCode_134(com);
        break;
    case 135: // G135
        GCode_135(com);
        break;
    case 201:
        GCode_201(com);
        break;
    case 202:
        GCode_202(com);
        break;
    case 203:
        GCode_203(com);
        break;
    case 204:
        GCode_204(com);
        break;
    case 205:
        GCode_205(com);
        break;
    default:
        if (Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
    previousMillisCmd = HAL::timeInMilliseconds();
}
/**
\brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode* com) {
    if (EVENT_UNHANDLED_M_CODE(com)) {
        return;
    }
    switch (com->M) {
    case 3: // Spindle/laser
        MCode_3(com);
        break;
    case 4: // Spindle CCW
        MCode_4(com);
        break;
    case 5: // Spindle/laser off
        MCode_5(com);
        break;
    case 20: // M20 - list SD card
        MCode_20(com);
        break;
    case 21: // M21 - init SD card
        MCode_21(com);
        break;
    case 22: //M22 - release SD card
        MCode_22(com);
        break;
    case 23: //M23 - Select file
        MCode_23(com);
        break;
    case 24: //M24 - Start SD print
        MCode_24(com);
        break;
    case 25: //M25 - Pause SD print
        MCode_25(com);
        break;
    case 26: //M26 - Set SD index
        MCode_26(com);
        break;
    case 27: //M27 - Get SD status
        MCode_27(com);
        break;
    case 28: //M28 - Start SD write
        MCode_28(com);
        break;
    case 29: //M29 - Stop SD write
        MCode_29(com);
        break;
    case 30: // M30 filename - Delete file
        MCode_30(com);
        break;
    case 32: // M32 directoryname
        MCode_32(com);
        break;
    case 36: // M36 JSON File Info
        MCode_36(com);
        break;
    case 42: //M42 -Change pin status via gcode
        MCode_42(com);
        break;
    case 80: // M80 - ATX Power On
        MCode_80(com);
        break;
    case 81: // M81 - ATX Power Off
        MCode_81(com);
        break;
    case 82: // M82
        MCode_82(com);
        break;
    case 83: // M83
        MCode_83(com);
        break;
    case 18: // M18 is to disable named axis
        MCode_18(com);
        break;
    case 84: // M84
        MCode_84(com);
        break;
    case 85: // M85
        MCode_85(com);
        break;
    case 92: // M92
        MCode_92(com);
        break;
    case 99: // M99 S<time>
        MCode_99(com);
        break;

    case 104: // M104 T<extr> S<temp> O<offset> F0 (beep)temperature
        MCode_104(com);
        break;
    case 140: // M140 set bed temp
        MCode_140(com);
        break;
    case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
        MCode_105(com);
        break;
    case 108: // Break long commands
        MCode_108(com);
        break;
    case 109: // M109 - Wait for extruder heater to reach target.
        MCode_109(com);
        break;
    case 190: // M190 - Wait bed for heater to reach target.
        MCode_190(com);
        break;
    case 155: // M155 S<1/0> Enable/disable auto report temperatures. When enabled firmware will emit temperatures every second.
        MCode_155(com);
        break;
    case 116: // Wait for temperatures to reach target temperature
        MCode_116(com);
        break;
    case 106: // M106 Fan On
        MCode_106(com);
        break;
    case 107: // M107 Fan Off
        MCode_107(com);
        break;
    case 111: // M111 enable/disable run time debug flags
        MCode_111(com);
        break;
    case 115: // M115
        MCode_115(com);
        break;
    case 114: // M114
        MCode_114(com);
        break;
    case 117: // M117 message to lcd
        MCode_117(com);
        break;
    case 119: // M119
        MCode_119(com);
        break;
    case 120: // M120 Test beeper function
        MCode_120(com);
        break;
    case 163: // M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
        MCode_163(com);
        break;
    case 164: /// M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights as virtual extruder S
        MCode_164(com);
        break;
    case 170: // preheat temperatures
        MCode_170(com);
        break;
    case 200: // M200 T<extruder> D<diameter>
        MCode_200(com);
        break;
    case 201: // M201
        MCode_201(com);
        break;
    case 202: // M202 travel acceleration, but no difference atm
        MCode_202(com);
        break;
    case 203: // M203
        MCode_203(com);
        break;
    case 204: // M204
        MCode_204(com);
        break;
    case 205: // M205 Show EEPROM settings
        MCode_205(com);
        break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
        MCode_206(com);
        break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
        MCode_207(com);
        break;
    case 209: // M209 S<0/1> Enable/disable autoretraction
        MCode_209(com);
        break;
    case 220: // M220 S<Feedrate multiplier in percent>
        MCode_220(com);
        break;
    case 221: // M221 S<Extrusion flow multiplier in percent>
        MCode_221(com);
        break;
    case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
        MCode_226(com);
        break;
    case 232: // M232
        MCode_232(com);
        break;
    case 233: // M233 now use M900 like Marlin
        MCode_900(com);
        break;
    case 251: // M251
        MCode_251(com);
        break;
    case 280: // M280
        MCode_280(com);
        break;
    case 281: // Trigger watchdog
        MCode_281(com);
        break;
    case 290: // M290 Z<babysteps> - Correct by adding baby steps for Z mm
#if EMERGENCY_PARSER == 0
        MCode_290(com);
#endif
        break;
    case 300: // M300
        MCode_300(com);
        break;
    case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will allow, S0 will disallow.
        MCode_302(com);
        break;
    case 303: // M303
        MCode_303(com);
        break;
    case 320: // M320 Activate autolevel
        MCode_320(com);
        break;
    case 321: // M321 Deactivate autoleveling
        MCode_321(com);
        break;
    case 322: // M322 Reset auto leveling matrix
        MCode_322(com);
        break;
    case 323: // M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
        MCode_323(com);
        break;
    case 340: // M340
        MCode_340(com);
        break;
    case 350: // M350 Set micro stepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
        MCode_350(com);
        break;
    case 355: // M355 S<0/1> - Turn case light on/off, no S = report status
        MCode_355(com);
        break;
    case 360: // M360 - show configuration
        MCode_360(com);
        break;
    case 400: // M400 Finish all moves
        MCode_400(com);
        break;
    case 401: // M401 Memory position
        MCode_401(com);
        break;
    case 402: // M402 Go to stored position
        MCode_402(com);
        break;
    case 408:
        MCode_408(com);
        break;
    case 460: // M460 X<minTemp> Y<maxTemp> : Set temperature range for thermo controlled fan
        MCode_460(com);
        break;
    case 500: // M500 store to eeprom
        MCode_500(com);
        break;
    case 501: // M501 read from eeprom
        MCode_501(com);
        break;
    case 502: // M502 restore from configuration
        MCode_502(com);
        break;
    case 513:
        MCode_513(com);
        break;
    //- M530 S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
    case 530:
        MCode_530(com);
        break;
        //- M531 filename - Define filename being printed
    case 531:
        MCode_531(com);
        break;
        //- M532 X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
    case 532:
        MCode_532(com);
        break;
    case 539:
        MCode_539(com);
        break;
    case 540: // report motion buffers
        MCode_540(com);
        break;
    case 600:
        MCode_600(com);
        break;
    case 601:
        MCode_601(com);
        break;
    case 602: // set jam control
        MCode_602(com);
        break;
    case 604:
        MCode_604(com);
        break;
    case 606: // Park extruder
        MCode_606(com);
        break;
    case 900: // M233 now use M900 like Marlin
        MCode_900(com);
        break;
    case 907: // M907 Set digital trimpot/DAC motor current using axis codes.
        MCode_907(com);
        break;
    case 908: // M908 Control digital trimpot directly.
        MCode_908(com);
        break;
    case 909: // M909 Read digital trimpot settings.
        MCode_909(com);
        break;
    case 910: // M910 - Commit digipot/DAC value to external EEPROM
        MCode_910(com);
        break;
#if 0 && UI_DISPLAY_TYPE != NO_DISPLAY
    // some debugging commands normally disabled
    case 888:
        Com::printFLN(PSTR("Selected language:"), (int)Com::selectedLanguage);
        Com::printF(PSTR("Translation:"));
        Com::printFLN(Com::translatedF(0));
        break;
    case 889:
        uid.showLanguageSelectionWizard();
        break;
    case 891:
        if(com->hasS())
            EEPROM::setVersion(com->S);
        break;
#endif
    case 890:
        MCode_890(com);
        break;
    case 998:
        MCode_998(com);
        break;
    case 999: // Stop fatal error take down
        MCode_999(com);
        break;
    default:
        if (Printer::debugErrors()) {
            Com::writeToAll = false;
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    } // switch
}

/**
\brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode* com) {
    // Set return channel for private commands. By default all commands send to all receivers.
    GCodeSource* actSource = GCodeSource::activeSource;
    GCodeSource::activeSource = com->source;
    Com::writeToAll = true;
    if (INCLUDE_DEBUG_COMMUNICATION) {
        if (Printer::debugCommunication()) {
            if (com->hasG() || (com->hasM() && com->M != 111)) {
                previousMillisCmd = HAL::timeInMilliseconds();
                GCodeSource::activeSource = actSource;
                return;
            }
        }
    }
    if (com->hasG())
        processGCode(com);
    else if (com->hasM())
        processMCode(com);
    else if (com->hasT()) { // Process T code
        //com->printCommand(); // for testing if this the source of extruder switches
        Motion1::waitForEndOfMoves();
        Tool::selectTool(com->T);
        // Extruder::selectExtruderById(com->T);
    } else {
        if (Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#ifdef DEBUG_DRYRUN_ERROR
    if (Printer::debugDryrun()) {
        Com::printFLN("Dryrun was enabled");
        com->printCommand();
        Printer::debugReset(8);
    }
#endif
    GCodeSource::activeSource = actSource;
}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
    HAL::resetHardware();
#else
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Printer::kill(false);
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_KILLED_ID));
    Commands::checkForPeriodicalActions(false);
    HAL::delayMilliseconds(200);
    Commands::checkForPeriodicalActions(false);
    InterruptProtectedBlock noInts;
    while (1) {
    }
#endif
}

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if (newfree < lowestRAMValue)
        lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
    if (lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM, lowestRAMValue);
    }
}
