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

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
    //while(true) {
#ifdef DEBUG_PRINT
    debugWaitLoop = 1;
#endif
    if(!Printer::isBlockingReceive()) {
        GCode::readFromSerial();
        GCode *code = GCode::peekCurrentCommand();
        //UI_SLOW; // do longer timed user interface action
        UI_MEDIUM; // do check encoder
        if(code) {
#if SDSUPPORT
            if(sd.savetosd) {
                if(!(code->hasM() && code->M == 29))   // still writing to file
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
    EVENT_PERIODICAL;
#if defined(DOOR_PIN) && DOOR_PIN > -1
    if(Printer::updateDoorOpen()) {
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        if(Printer::mode == PRINTER_MODE_LASER) {
            LaserDriver::changeIntensity(0);
        }
#endif
    }
#endif
    if(!executePeriodical) return; // gets true every 100ms
    executePeriodical = 0;
    EVENT_TIMER_100MS;
    Extruder::manageTemperatures();
    if(--counter500ms == 0) {
        if(manageMonitor)
            writeMonitor();
        counter500ms = 5;
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
    while(PrintLine::hasLines()) {
        //GCode::readFromSerial();
        checkForPeriodicalActions(false);
        GCode::keepAlive(Processing);
        UI_MEDIUM;
    }
}

void Commands::waitUntilEndOfAllBuffers() {
    GCode *code = NULL;
#ifdef DEBUG_PRINT
    debugWaitLoop = 9;
#endif
    while(PrintLine::hasLines() || (code != NULL)) {
        //GCode::readFromSerial();
        code = GCode::peekCurrentCommand();
        UI_MEDIUM; // do check encoder
        if(code) {
#if SDSUPPORT
            if(sd.savetosd) {
                if(!(code->hasM() && code->M == 29))   // still writing to file
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

void Commands::printCurrentPosition() {
    float x, y, z;
    Printer::realPosition(x, y, z);
    x += Printer::coordinateOffset[X_AXIS];
    y += Printer::coordinateOffset[Y_AXIS];
    z += Printer::coordinateOffset[Z_AXIS];
    Com::printF(Com::tXColon, x * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceZColon, z * (Printer::unitIsInches ? 0.03937 : 1), 3);
    Com::printFLN(Com::tSpaceEColon, Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 4);
#ifdef DEBUG_POS
    Com::printF(PSTR("OffX:"), Printer::offsetX); // to debug offset handling
    Com::printF(PSTR(" OffY:"), Printer::offsetY);
    Com::printF(PSTR(" OffZ:"), Printer::offsetZ);
    Com::printF(PSTR(" OffZ2:"), Printer::offsetZ2);
    Com::printF(PSTR(" XS:"), Printer::currentPositionSteps[X_AXIS]);
    Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
    Com::printFLN(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);

#endif
}

void Commands::printTemperatures(bool showRaw) {
    int error;
#if NUM_EXTRUDER > 0
    float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE == 0
    Com::printF(Com::tTColon, temp);
    Com::printF(Com::tSpaceSlash, Extruder::current->tempControl.targetTemperatureC, 0);
#else
    Com::printF(Com::tTColon, temp);
    Com::printF(Com::tSpaceSlash, Extruder::current->tempControl.targetTemperatureC, 0);
#if HAVE_HEATED_BED
    Com::printF(Com::tSpaceBColon, Extruder::getHeatedBedTemperature());
    Com::printF(Com::tSpaceSlash, heatedBedController.targetTemperatureC, 0);
    if((error = heatedBedController.errorState()) > 0) {
        Com::printF(PSTR(" DB:"), error);
    }
    if(showRaw) {
        Com::printF(Com::tSpaceRaw, (int)NUM_EXTRUDER);
        Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - heatedBedController.currentTemperature);
    }
    Com::printF(Com::tSpaceBAtColon, (pwm_pos[heatedBedController.pwmIndex])); // Show output of auto tune when tuning!
#endif
#endif
    Com::printF(Com::tSpaceAtColon, (autotuneIndex == 255 ? pwm_pos[Extruder::current->id] : pwm_pos[autotuneIndex])); // Show output of auto tune when tuning!
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        Com::printF(Com::tSpaceT, (int)i);
        Com::printF(Com::tColon, extruder[i].tempControl.currentTemperatureC);
        Com::printF(Com::tSpaceSlash, extruder[i].tempControl.targetTemperatureC, 0);
        Com::printF(Com::tSpaceAt, (int)i);
        Com::printF(Com::tColon, (pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of auto tune when tuning!
        if((error = extruder[i].tempControl.errorState()) > 0) {
            Com::printF(PSTR(" D"), (int)i);
            Com::printF(Com::tColon, error);
        }
        if(showRaw) {
            Com::printF(Com::tSpaceRaw, (int)i);
            Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[i].tempControl.currentTemperature);
        }
    }
#elif NUM_EXTRUDER == 1 || MIXING_EXTRUDER
    if((error = extruder[0].tempControl.errorState()) > 0) {
        Com::printF(PSTR(" D0:"), error);
    }
    if(showRaw) {
        Com::printF(Com::tSpaceRaw, (int)0);
        Com::printF(Com::tColon, (1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[0].tempControl.currentTemperature);
    }
#endif
#ifdef FAKE_CHAMBER
    Com::printF(PSTR(" C:"), extruder[0].tempControl.currentTemperatureC);
    Com::printF(Com::tSpaceSlash, extruder[0].tempControl.targetTemperatureC, 0);
    Com::printF(PSTR(" @:"), (pwm_pos[extruder[0].tempControl.pwmIndex]));
#endif
    Com::println();
#endif
}
void Commands::changeFeedrateMultiply(int factor) {
    if(factor < 25) factor = 25;
    if(factor > 500) factor = 500;
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

void Commands::changeFlowrateMultiply(int factor) {
    if(factor < 25) factor = 25;
    if(factor > 200) factor = 200;
    Printer::extrudeMultiply = factor;
    if(Extruder::current->diameter <= 0)
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
    else
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
    Com::printFLN(Com::tFlowMultiply, factor);
}

#if FEATURE_FAN_CONTROL
uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
uint8_t fan2Kickstart;
#endif

void Commands::setFanSpeed(int speed, bool immediately) {
#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL
    if(Printer::fanSpeed == speed)
        return;
    speed = constrain(speed, 0, 255);
    Printer::setMenuMode(MENU_MODE_FAN_RUNNING, speed != 0);
    Printer::fanSpeed = speed;
    if(PrintLine::linesCount == 0 || immediately) {
        if(Printer::mode == PRINTER_MODE_FFF) {
            for(fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++)
                PrintLine::lines[i].secondSpeed = speed;         // fill all printline buffers with new fan speed value
        }
        Printer::setFanSpeedDirectly(speed);
    }
    Com::printFLN(Com::tFanspeed, speed); // send only new values to break update loops!
#endif
}
void Commands::setFan2Speed(int speed) {
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
    speed = constrain(speed, 0, 255);
    Printer::setFan2SpeedDirectly(speed);
    Com::printFLN(Com::tFan2speed, speed); // send only new values to break update loops!
#endif
}

void Commands::reportPrinterUsage() {
#if EEPROM_MODE != 0
    float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
    Com::printF(Com::tPrintedFilament, dist, 2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
#if NUM_EXTRUDER > 0
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;
#endif
    int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
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
    if(value > 255)
        value = 255;
    WRITE(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    HAL::spiSend(address); //  send in the address and value via SPI:
    HAL::spiSend(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void setMotorCurrent(uint8_t driver, uint16_t current) {
    if(driver > 4) return;
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}

void setMotorCurrentPercent( uint8_t channel, float level) {
    uint16_t raw_level = ( level * 255 / 100 );
    setMotorCurrent(channel, raw_level);
}
#endif

void motorCurrentControlInit() { //Initialize Digipot Motor Current
#if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    HAL::spiInit(0); //SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for(int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;
    for(int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrent(i, digipot_motor_current[i]);
#endif
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_LTC2600

void setMotorCurrent( uint8_t channel, unsigned short level ) {
    if(channel >= LTC2600_NUM_CHANNELS) return;
    const uint8_t ltc_channels[] =  LTC2600_CHANNELS;
    if(channel > LTC2600_NUM_CHANNELS) return;
    uint8_t address = ltc_channels[channel];
    fast8_t i;


    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    // configure the pins
    WRITE( LTC2600_CS_PIN, HIGH );
    SET_OUTPUT( LTC2600_CS_PIN );
    WRITE( LTC2600_SCK_PIN, LOW );
    SET_OUTPUT( LTC2600_SCK_PIN );
    WRITE( LTC2600_SDI_PIN, LOW );
    SET_OUTPUT( LTC2600_SDI_PIN );

    // enable the command interface of the LTC2600
    WRITE( LTC2600_CS_PIN, LOW );

    // transfer command and address
    for( i = 7; i >= 0; i-- ) {
        WRITE( LTC2600_SDI_PIN, address & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // transfer the data word
    for( i = 15; i >= 0; i-- ) {
        WRITE( LTC2600_SDI_PIN, level & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // disable the ommand interface of the LTC2600 -
    // this carries out the specified command
    WRITE( LTC2600_CS_PIN, HIGH );

} // setLTC2600
void setMotorCurrentPercent( uint8_t channel, float level) {
    if(level > 100.0f) level = 100.0f;
    uint16_t raw_level = static_cast<uint16_t>( (long)level * 65535L / 100L );
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() { //Initialize LTC2600 Motor Current
    uint8_t i;
#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for(int i = 0; i < LTC2600_NUM_CHANNELS; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const unsigned int ltc_current[] =  MOTOR_CURRENT;
    for(i = 0; i < LTC2600_NUM_CHANNELS; i++) {
        setMotorCurrent(i, ltc_current[i] );
    }
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_ALLIGATOR
void setMotorCurrent(uint8_t channel, unsigned short value) {
    if(channel >= 7) // max channel (X,Y,Z,E0,E1,E2,E3)
        return;
    if(value > 255)
        value = 255;

    uint8_t externalDac_buf[2] = {0x10, 0x00};

    if(channel > 3)
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

    if(channel > 3) { // DAC Piggy E1,E2,E3
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

void setMotorCurrentPercent( uint8_t channel, float level) {
    uint16_t raw_level = ( level * 255 / 100 );
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() { //Initialize Motor Current
    uint8_t externalDac_buf[2] = {0x20, 0x00};//all off

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
    for(int i = 0; i < NUM_EXTRUDER + 3; i++)
        setMotorCurrentPercent(i, digipot_motor_current[i]);
#else
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;
    for(uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
        setMotorCurrent(i, digipot_motor_current[i]);
#endif
}
#endif

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_TMC2130
TMC2130Stepper* tmcDriverByIndex(uint8_t index) {
    switch(index) {
#if TMC2130_ON_X
        case 0: // X Axis
            return Printer::tmc_driver_x;
        break;
#endif
#if TMC2130_ON_Y
        case 1: // Y Axis
            return Printer::tmc_driver_y;
        break;
#endif
#if TMC2130_ON_Z
        case 2: // Z Axis
            return Printer::tmc_driver_z;
        break;
#endif
#if TMC2130_ON_EXT0
        case 3: // E0 Axis
            return Printer::tmc_driver_e0;
        break;
#endif
#if TMC2130_ON_EXT1
        case 4: // E1 Axis
            return Printer::tmc_driver_e1;
        break;
#endif
#if TMC2130_ON_EXT2
        case 5: // E2 Axis
            return Printer::tmc_driver_e2;
        break;
#endif
        default:
            return NULL;
        break;
    }
}

void setMotorCurrent( uint8_t driver, uint16_t level ) {
    TMC2130Stepper* tmc_driver = tmcDriverByIndex(driver);
    if(tmc_driver) {
#if MOTHERBOARD == 310
		tmc_driver->rms_current(level, 0.5, 0.22);
#else
        tmc_driver->rms_current(level);
#endif
    }
}

void setMotorCurrentPercent( uint8_t channel, float level ) {
    const uint16_t tmc_motor_current[] = MOTOR_CURRENT;
    uint16_t raw_level = ( level * tmc_motor_current[channel] / 100 );
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() {
    const uint16_t tmc_motor_current[] = MOTOR_CURRENT;
    for(uint8_t i = 0; i < (sizeof(tmc_motor_current) / sizeof(uint16_t)); i++) {
        setMotorCurrent(i, tmc_motor_current[i]);
    }
}

void motorCurrentReadings() {
    Com::printF(Com::tTrinamicMotorCurrent);
#if TMC2130_ON_X
    Com::printF(Com::tSpaceXColon, (uint32_t)(Printer::tmc_driver_x->rms_current()));
#endif
#if TMC2130_ON_Y
    Com::printF(Com::tSpaceYColon, (uint32_t)(Printer::tmc_driver_y->rms_current()));
#endif
#if TMC2130_ON_Z
    Com::printF(Com::tSpaceZColon, (uint32_t)(Printer::tmc_driver_z->rms_current()));
#endif
#if TMC2130_ON_EXT0
    Com::printF(Com::tSpaceEColon, (uint32_t)(Printer::tmc_driver_e0->rms_current()));
#endif
#if TMC2130_ON_EXT1
    Com::printF(PSTR(" E1:"), (uint32_t)(Printer::tmc_driver_e1->rms_current()));
#endif
#if TMC2130_ON_EXT2
    Com::printF(PSTR(" E2:"), (uint32_t)(Printer::tmc_driver_e2->rms_current()));
#endif
    Com::printFLN(Com::tSpace);
}
#endif // CURRENT_CONTROL_TMC2130

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
uint8_t   _intVref[]     = {MCP4728_VREF, MCP4728_VREF, MCP4728_VREF, MCP4728_VREF};
uint8_t   _gain[]        = {MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN};
uint8_t   _powerDown[]   = {0, 0, 0, 0};
int16_t   dac_motor_current[] =  {0, 0, 0, 0};

uint8_t   _intVrefEp[]   = {MCP4728_VREF, MCP4728_VREF, MCP4728_VREF, MCP4728_VREF};
uint8_t   _gainEp[]      = {MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN, MCP4728_GAIN};
uint8_t   _powerDownEp[] = {0, 0, 0, 0};
int16_t    _valuesEp[]   = {0, 0, 0, 0};

uint8_t   dac_stepper_channel[] = MCP4728_STEPPER_ORDER;

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
        uint8_t  hiByte  = HAL::i2cReadAck();
        uint8_t  loByte  = ((i < 7) ? HAL::i2cReadAck() : HAL::i2cReadNak());

        uint8_t isEEPROM = (deviceID & 0B00001000) >> 3;
        uint8_t channel  = (deviceID & 0B00110000) >> 4;
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
    if (saveEEPROM) HAL::i2cWrite(dac_write_cmd);

    for (int i = 0; i < MCP4728_NUM_CHANNELS; i++) {
        uint16_t level = dac_motor_current[i];

        uint8_t highbyte = ( _intVref[i] << 7 | _gain[i] << 4 | (uint8_t)((level) >> 8) );
        uint8_t lowbyte =  ( (uint8_t) ((level) & 0xff) );
        dac_write_cmd = MCP4728_CMD_MULTI_WRITE | (i << 1);

        if (!saveEEPROM) HAL::i2cWrite(dac_write_cmd);
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
    const char* dacChannelPrefixes[] = {Com::tSpaceXColon, Com::tSpaceYColon, Com::tSpaceZColon, Com::tSpaceEColon};

    Com::printFLN(Com::tMCPEpromSettings);
    dacPrintSet(_valuesEp, dacChannelPrefixes); // Once for the EEPROM set

    Com::printFLN(Com::tMCPCurrentSettings);
    dacPrintSet(dac_motor_current, dacChannelPrefixes); // And another for the RUNTIME set
}

void setMotorCurrent( uint8_t xyz_channel, uint16_t level ) {
    if (xyz_channel >= MCP4728_NUM_CHANNELS) return;
    uint8_t stepper_channel = dac_stepper_channel[xyz_channel];
    dac_motor_current[stepper_channel] = level < MCP4728_VOUT_MAX ? level : MCP4728_VOUT_MAX;
    dacAnalogUpdate();
}

void setMotorCurrentPercent( uint8_t channel, float level) {
    uint16_t raw_level = ( level * MCP4728_VOUT_MAX / 100 );
    setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() { //Initialize MCP4728 Motor Current
    HAL::i2cInit(400000); // Initialize the i2c bus.
    dacSimpleCommand((uint8_t)MCP4728_CMD_GC_RESET); // MCP4728 General Command Reset
    dacReadStatus(); // Load Values from EEPROM.

    for(int i = 0; i < MCP4728_NUM_CHANNELS; i++) {
        setMotorCurrent(dac_stepper_channel[i], _valuesEp[i] ); // This is not strictly necessary, but serves as a good sanity check to ensure we're all on the same page.
    }
}
#endif

#if defined(DRV_TMC2130)
void microstepMode(uint8_t driver, uint16_t stepping_mode) {
    TMC2130Stepper* tmc_driver;
    if(tmc_driver = tmcDriverByIndex(driver))
        tmc_driver->microsteps(stepping_mode);
}

void microstepInit() {
    const uint16_t microstep_modes[] = MICROSTEP_MODES;
    for(uint8_t i = 0; i < (sizeof(microstep_modes) / sizeof(uint16_t)); i++) {
        microstepMode(i, microstep_modes[i]);
    }
}

void microstepReadings() {
    Com::printF(Com::tTrinamicMicrostepMode);
#if TMC2130_ON_X
    Com::printF(Com::tSpaceXColon, (uint32_t)(Printer::tmc_driver_x->microsteps()));
#endif
#if TMC2130_ON_Y
    Com::printF(Com::tSpaceYColon, (uint32_t)(Printer::tmc_driver_y->microsteps()));
#endif
#if TMC2130_ON_Z
    Com::printF(Com::tSpaceZColon, (uint32_t)(Printer::tmc_driver_z->microsteps()));
#endif
#if TMC2130_ON_EXT0
    Com::printF(Com::tSpaceEColon, (uint32_t)(Printer::tmc_driver_e0->microsteps()));
#endif
#if TMC2130_ON_EXT1
    Com::printF(PSTR(" E1:"), (uint32_t)(Printer::tmc_driver_e1->microsteps()));
#endif
#if TMC2130_ON_EXT2
    Com::printF(PSTR(" E1:"), (uint32_t)(Printer::tmc_driver_e2->microsteps()));
#endif
    Com::printFLN(Com::tSpace);
}
#else
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
void microstepMS(uint8_t driver, int8_t ms1, int8_t ms2) {
    if(ms1 > -1) switch(driver) {
        case 0:
#if X_MS1_PIN > -1
            WRITE( X_MS1_PIN, ms1);
#endif
            break;
        case 1:
#if Y_MS1_PIN > -1
            WRITE( Y_MS1_PIN, ms1);
#endif
            break;
        case 2:
#if Z_MS1_PIN > -1
            WRITE( Z_MS1_PIN, ms1);
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
    if(ms2 > -1) switch(driver) {
        case 0:
#if X_MS2_PIN > -1
            WRITE( X_MS2_PIN, ms2);
#endif
            break;
        case 1:
#if Y_MS2_PIN > -1
            WRITE( Y_MS2_PIN, ms2);
#endif
            break;
        case 2:
#if Z_MS2_PIN > -1
            WRITE( Z_MS2_PIN, ms2);
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
    switch(stepping_mode) {
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
    for(int i = 0; i <= 4; i++) microstepMode(i, microstep_modes[i]);
#endif
}
#endif
/**
\brief Execute the Arc command stored in com.
*/
#if ARC_SUPPORT
void Commands::processArc(GCode *com) {
    float position[Z_AXIS_ARRAY];
    Printer::realPosition(position[X_AXIS], position[Y_AXIS], position[Z_AXIS]);
    if(!Printer::setDestinationStepsFromGCode(com)) return; // For X Y Z E F
    float offset[2] = {Printer::convertToMM(com->hasI() ? com->I : 0), Printer::convertToMM(com->hasJ() ? com->J : 0)};
    float target[E_AXIS_ARRAY] = {Printer::realXPosition(), Printer::realYPosition(), Printer::realZPosition(), Printer::destinationSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]};
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
        if(isnan(h_x2_div_d)) {
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

    } else { // Offset mode specific computations
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
void Commands::processGCode(GCode *com) {
    if(EVENT_UNHANDLED_G_CODE(com)) {
        previousMillisCmd = HAL::timeInMilliseconds();
        return;
    }
    uint32_t codenum; //throw away variable
    switch(com->G) {
    case 0: // G0 -> G1
    case 1: // G1
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    {
        // disable laser for G0 moves
        bool laserOn = LaserDriver::laserOn;
        if(Printer::mode == PRINTER_MODE_LASER) {
            if(com->G == 0) {
                LaserDriver::laserOn = false;
                LaserDriver::firstMove = true; //set G1 flag for Laser
            } else {
#if LASER_WARMUP_TIME > 0
                uint8_t power = (com->hasX() || com->hasY()) && (LaserDriver::laserOn || com->hasE()) ? LaserDriver::intensity : 0;
                if(power > 0 && LaserDriver::firstMove) {
                    PrintLine::waitForXFreeLines(1, true);
                    PrintLine::LaserWarmUp(LASER_WARMUP_TIME);
                    LaserDriver::firstMove = false;
                }
#endif
            }
        }
#endif // defined
        if(com->hasS()) Printer::setNoDestinationCheck(com->S != 0);
        if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if NONLINEAR_SYSTEM
            if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, true)) {
                Com::printWarningFLN(PSTR("executeGCode / queueDeltaMove returns error"));
            }
#else
            PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
#endif
#if UI_HAS_KEYS
        // ui can only execute motion commands if we are not waiting inside a move for an
        // old move to finish. For normal response times, we always leave one free after
        // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
        // gets filled while waiting, the lost is neglectable.
        PrintLine::waitForXFreeLines(1, true);
#endif // UI_HAS_KEYS
#ifdef DEBUG_QUEUE_MOVE
        {

            InterruptProtectedBlock noInts;
            int lc = (int)PrintLine::linesCount;
            int lp = (int)PrintLine::linesPos;
            int wp = (int)PrintLine::linesWritePos;
            int n = (wp - lp);
            if(n < 0) n += PRINTLINE_CACHE_SIZE;
            noInts.unprotect();
            if(n != lc)
                Com::printFLN(PSTR("Buffer corrupted"));
        }
#endif

#if defined(SUPPORT_LASER) && SUPPORT_LASER
        LaserDriver::laserOn = laserOn;
    }
#endif // defined
    break;
#if ARC_SUPPORT
    case 2: // CW Arc
    case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
    {
        bool laserOn = LaserDriver::laserOn;
#if LASER_WARMUP_TIME > 0
        if(Printer::mode == PRINTER_MODE_LASER && LaserDriver::firstMove && (LaserDriver::laserOn || com->hasE())) {
            PrintLine::waitForXFreeLines(1, true);
            PrintLine::LaserWarmUp(LASER_WARMUP_TIME);
            LaserDriver::firstMove = false;
        }
#endif
#endif // defined
        processArc(com);
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        LaserDriver::laserOn = laserOn;
    }
#endif // defined                
    break;
#endif
    case 4: // G4 dwell
        Commands::waitUntilEndOfAllMoves();
        codenum = 0;
        if(com->hasP()) codenum = com->P; // milliseconds to wait
        if(com->hasS()) codenum = com->S * 1000; // seconds to wait
        codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting
        while((uint32_t)(codenum - HAL::timeInMilliseconds())  < 2000000000 ) {
            GCode::keepAlive(Processing);
            Commands::checkForPeriodicalActions(true);
        }
        break;
#if FEATURE_RETRACTION && NUM_EXTRUDER > 0
    case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
#if NUM_EXTRUDER > 1
        Extruder::current->retract(true, com->hasS() && com->S > 0);
#else
        Extruder::current->retract(true, false);
#endif
        break;
    case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
#if NUM_EXTRUDER > 1
        Extruder::current->retract(false, com->hasS() && com->S > 0);
#else
        Extruder::current->retract(false, false);
#endif
        break;
#endif // FEATURE_RETRACTION
    case 20: // G20 Units to inches
        Printer::unitIsInches = 1;
        break;
    case 21: // G21 Units to mm
        Printer::unitIsInches = 0;
        break;
    case 28: { //G28 Home all Axis one at a time
        uint8_t homeAllAxis = (com->hasNoXYZ() && !com->hasE());
        if(com->hasE())
            Printer::currentPositionSteps[E_AXIS] = 0;
        if(homeAllAxis || !com->hasNoXYZ())
            Printer::homeAxis(homeAllAxis || com->hasX(), homeAllAxis || com->hasY(), homeAllAxis || com->hasZ());
    }
    break;
#if FEATURE_Z_PROBE
    case 29: { // G29 3 points, build average or distortion compensation
        Printer::prepareForProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
        float actTemp[NUM_EXTRUDER];
        for(int i = 0; i < NUM_EXTRUDER; i++)
            actTemp[i] = extruder[i].tempControl.targetTemperatureC;
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::max(EEPROM::zProbeHeight(), static_cast<float>(ZHOME_HEAT_HEIGHT)), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
        Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
        }
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
        }
#else
        if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
        bool ok = true;
        Printer::startProbing(true);
        bool oldAutolevel = Printer::isAutolevelActive();
        Printer::setAutolevelActive(false);
        float sum = 0, last, oldFeedrate = Printer::feedrate;
        Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
        sum = Printer::runZProbe(true, false, Z_PROBE_REPETITIONS, false);
        if(sum == ILLEGAL_Z_PROBE) ok = false;
        if(ok) {
            Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
            last = Printer::runZProbe(false, false);
            if(last == ILLEGAL_Z_PROBE) ok = false;
            sum += last;
        }
        if(ok) {
            Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
            last = Printer::runZProbe(false, true);
            if(last == ILLEGAL_Z_PROBE) ok = false;
            sum += last;
        }
        if(ok) {
            sum *= 0.33333333333333;
            Com::printFLN(Com::tZProbeAverage, sum);
            if(com->hasS() && com->S) {
#if MAX_HARDWARE_ENDSTOP_Z
#if DRIVE_SYSTEM == DELTA
                Printer::updateCurrentPosition();
                Printer::zLength += sum - Printer::currentPosition[Z_AXIS];
                Printer::updateDerivedParameter();
                Printer::homeAxis(true, true, true);
#else
                Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                float zup = Printer::runZMaxProbe();
                if(zup == ILLEGAL_Z_PROBE) {
                    ok = false;
                } else
                    Printer::zLength = zup + sum - ENDSTOP_Z_BACK_ON_HOME;
#endif // DELTA
                Com::printInfoFLN(Com::tZProbeZReset);
                Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#else
                Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
                Com::printFLN(PSTR("Adjusted z origin"));
#endif // max z endstop
            }
            Printer::feedrate = oldFeedrate;
            Printer::setAutolevelActive(oldAutolevel);
            if(ok && com->hasS() && com->S == 2)
                EEPROM::storeDataIntoEEPROM();
        }
        Printer::updateCurrentPosition(true);
        printCurrentPosition();
        Printer::finishProbing();
        Printer::feedrate = oldFeedrate;
        if(!ok) {
            GCode::fatalError(PSTR("G29 leveling failed!"));
            break;
        }
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
        }
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
        }
#else
        if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
    }
    break;
    case 30: {
        // G30 [Pn] [S]
        // G30 (the same as G30 P3) single probe set Z0
        // G30 S1 Z<real_z_pos> - measures probe height (P is ignored) assuming we are at real height Z
        // G30 H<height> R<offset> Make probe define new Z and z offset (R) at trigger point assuming z-probe measured an object of H height.
        if (com->hasS()) {
            Printer::measureZProbeHeight(com->hasZ() ? com->Z : Printer::currentPosition[Z_AXIS]);
        } else {
            uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
            float z = Printer::runZProbe(p & 1, p & 2, Z_PROBE_REPETITIONS, true, false);
            if(z == ILLEGAL_Z_PROBE) {
                GCode::fatalError(PSTR("G30 probing failed!"));
                break;
            }
            if(com->hasR() || com->hasH()) {
                float h = Printer::convertToMM(com->hasH() ? com->H : 0);
                float o = Printer::convertToMM(com->hasR() ? com->R : h);
#if DISTORTION_CORRECTION
				// Undo z distortion correction contained in z
                float zCorr = 0;
                if(Printer::distortion.isEnabled()) {
                    zCorr = Printer::distortion.correct(Printer::currentPositionSteps[X_AXIS], Printer::currentPositionSteps[Y_AXIS], Printer::zMinSteps) * Printer::invAxisStepsPerMM[Z_AXIS];
                    z -= zCorr;
                }
#endif
                Printer::coordinateOffset[Z_AXIS] = o - h;
                Printer::currentPosition[Z_AXIS] = Printer::lastCmdPos[Z_AXIS] = z + h + Printer::zMin;
                Printer::updateCurrentPositionSteps();
                Printer::setZHomed(true);
#if NONLINEAR_SYSTEM
                transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
#endif
            } else {
                Printer::updateCurrentPosition(p & 1);
            }
        }
    }
    break;
    case 31:  // G31 display hall sensor output
        Endstops::update();
        Endstops::update();
        Com::printF(Com::tZProbeState);
        Com::printF(Endstops::zProbe() ? Com::tHSpace : Com::tLSpace);
        Com::println();
        break;
#if FEATURE_AUTOLEVEL
    case 32: // G32 Auto-Bed leveling
        if(!runBedLeveling(com->hasS() ? com->S : -1)) {
            GCode::fatalError(PSTR("G32 leveling failed!"));
        }
        break;
#endif
#if DISTORTION_CORRECTION
    case 33: {
        if(com->hasL()) { // G33 L0 - List distortion matrix
            Printer::distortion.showMatrix();
        } else if(com->hasR()) { // G33 R0 - Reset distortion matrix
            Printer::distortion.resetCorrection();
        } else if(com->hasX() || com->hasY() || com->hasZ()) { // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
            if(com->hasX() && com->hasY() && com->hasZ()) {
                Printer::distortion.set(com->X, com->Y, com->Z);
            } else {
                Com::printErrorFLN(PSTR("You need to define X, Y and Z to set a point!"));
            }
        } else { // G33
            Printer::measureDistortion();
        }
    }
    break;
#endif
#endif
    case 90: // G90
        Printer::relativeCoordinateMode = false;
        if(com->internalCommand)
            Com::printInfoFLN(PSTR("Absolute positioning"));
        break;
    case 91: // G91
        Printer::relativeCoordinateMode = true;
        if(com->internalCommand)
            Com::printInfoFLN(PSTR("Relative positioning"));
        break;
    case 92: { // G92
        float xOff = Printer::coordinateOffset[X_AXIS];
        float yOff = Printer::coordinateOffset[Y_AXIS];
        float zOff = Printer::coordinateOffset[Z_AXIS];
        if(com->hasX()) xOff = Printer::convertToMM(com->X) - Printer::currentPosition[X_AXIS];
        if(com->hasY()) yOff = Printer::convertToMM(com->Y) - Printer::currentPosition[Y_AXIS];
        if(com->hasZ()) zOff = Printer::convertToMM(com->Z) - Printer::currentPosition[Z_AXIS];
        Printer::setOrigin(xOff, yOff, zOff);
        if(com->hasE()) {
            Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
        }
		if(com->hasX() || com->hasY() || com->hasZ()) {
			Com::printF(PSTR("X_OFFSET:"), Printer::coordinateOffset[X_AXIS], 3);
			Com::printF(PSTR(" Y_OFFSET:"), Printer::coordinateOffset[Y_AXIS], 3);
			Com::printFLN(PSTR(" Z_OFFSET:"), Printer::coordinateOffset[Z_AXIS], 3);
		}
    }
    break;
#if DRIVE_SYSTEM == DELTA
    case 100: { // G100 Calibrate floor or rod radius
        // Using manual control, adjust hot end to contact floor.
        // G100 <no arguments> No action. Avoid accidental floor reset.
        // G100 [X] [Y] [Z] set floor for argument passed in. Number ignored and may be absent.
        // G100 R with X Y or Z flag error, sets only floor or radius, not both.
        // G100 R[n] Add n to radius. Adjust to be above floor if necessary
        // G100 R[0] set radius based on current z measurement. Moves to (0,0,0)
        float currentZmm = Printer::currentPosition[Z_AXIS];
        if (currentZmm / Printer::zLength > 0.1) {
            Com::printErrorFLN(PSTR("Calibration code is limited to bottom 10% of Z height"));
            break;
        }
        if (com->hasR()) {
            if (com->hasX() || com->hasY() || com->hasZ())
                Com::printErrorFLN(PSTR("Cannot set radius and floor at same time."));
            else if (com->R != 0) {
                //add r to radius
                if (abs(com->R) <= 10) EEPROM::incrementRodRadius(com->R);
                else Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
            } else {
                // auto set radius. Head must be at 0,0 and touching
                // Z offset will be corrected for.
                if (Printer::currentPosition[X_AXIS] == 0
                        && Printer::currentPosition[Y_AXIS] == 0) {
                    if(Printer::isLargeMachine()) {
                        // calculate radius assuming we are at surface
                        // If Z is greater than 0 it will get calculated out for correct radius
                        // Use either A or B tower as they anchor x Cartesian axis and always have
                        // Radius distance to center in simplest set up.
                        float h = Printer::deltaDiagonalStepsSquaredB.f;
                        unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
                        // The correct Rod Radius would put us here at z==0 and B height is
                        // square root (rod length squared minus rod radius squared)
                        // Reverse that to get calculated Rod Radius given B height
                        h -= RMath::sqr((float)bSteps);
                        h = sqrt(h);
                        EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
                    } else {
                        // calculate radius assuming we are at surface
                        // If Z is greater than 0 it will get calculated out for correct radius
                        // Use either A or B tower as they anchor x Cartesian axis and always have
                        // Radius distance to center in simplest set up.
                        unsigned long h = Printer::deltaDiagonalStepsSquaredB.l;
                        unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
                        // The correct Rod Radius would put us here at z==0 and B height is
                        // square root (rod length squared minus rod radius squared)
                        // Reverse that to get calculated Rod Radius given B height
                        h -= RMath::sqr(bSteps);
                        h = SQRT(h);
                        EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
                    }
                } else
                    Com::printErrorFLN(PSTR("First move to touch at x,y=0,0 to auto-set radius."));
            }
        } else {
            bool tooBig = false;
            if (com->hasX()) {
                if (abs(com->X) <= 10)
                    EEPROM::setTowerXFloor(com->X + currentZmm + Printer::xMin);
                else tooBig = true;
            }
            if (com->hasY()) {
                if (abs(com->Y) <= 10)
                    EEPROM::setTowerYFloor(com->Y + currentZmm + Printer::yMin);
                else tooBig = true;
            }
            if (com->hasZ()) {
                if (abs(com->Z) <= 10)
                    EEPROM::setTowerZFloor(com->Z + currentZmm + Printer::zMin);
                else tooBig = true;
            }
            if (tooBig)
                Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
        }
        // after adjusting zero, physical position is out of sync with memory position
        // this could cause jerky movement or push head into print surface.
        // moving gets back into safe zero'ed position with respect to newle set floor or Radius.
        Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, 12.0, IGNORE_COORDINATE, IGNORE_COORDINATE);
        break;
    }
    case 131: { // G131 Remove offset
        float cx, cy, cz;
        Printer::realPosition(cx, cy, cz);
        float oldfeedrate = Printer::feedrate;
        Printer::offsetX = 0;
        Printer::offsetY = 0;
        Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
        Printer::feedrate = oldfeedrate;
        Printer::updateCurrentPosition();
    }
    break;
    case 132: { // G132 Calibrate endstop offsets
        // This has the probably unintended side effect of turning off leveling.
        Printer::setAutolevelActive(false); // don't let transformations change result!
        Printer::coordinateOffset[X_AXIS] = 0;
        Printer::coordinateOffset[Y_AXIS] = 0;
        Printer::coordinateOffset[Z_AXIS] = 0;
        // I think this is coded incorrectly, as it depends on the start position of the
        // of the hot end, and so should first move to x,y,z= 0,0,0, but as that may not
        // be possible if the printer is not in the homes/zeroed state, the printer
        // cannot safely move to 0 z coordinate without crashing into the print surface.
        // so other than commenting, I'm not meddling.
        // but you will always get different counts from different positions.
        Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
        int32_t m = RMath::max(Printer::stepsRemainingAtXHit, RMath::max(Printer::stepsRemainingAtYHit, Printer::stepsRemainingAtZHit));
        int32_t offx = m - Printer::stepsRemainingAtXHit;
        int32_t offy = m - Printer::stepsRemainingAtYHit;
        int32_t offz = m - Printer::stepsRemainingAtZHit;
        Com::printFLN(Com::tTower1, offx);
        Com::printFLN(Com::tTower2, offy);
        Com::printFLN(Com::tTower3, offz);
#if EEPROM_MODE != 0
        if(com->hasS() && com->S > 0) {
            EEPROM::setDeltaTowerXOffsetSteps(offx);
            EEPROM::setDeltaTowerYOffsetSteps(offy);
            EEPROM::setDeltaTowerZOffsetSteps(offz);
        }
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, -5 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);
        Printer::homeAxis(true, true, true);
    }
    break;
    case 133: { // G133 Measure steps to top
        bool oldAuto = Printer::isAutolevelActive();
        Printer::setAutolevelActive(false); // don't let transformations change result!
        Printer::currentPositionSteps[X_AXIS] = 0;
        Printer::currentPositionSteps[Y_AXIS] = 0;
        Printer::currentPositionSteps[Z_AXIS] = 0;
        Printer::coordinateOffset[X_AXIS] = 0;
        Printer::coordinateOffset[Y_AXIS] = 0;
        Printer::coordinateOffset[Z_AXIS] = 0;
        Printer::currentNonlinearPositionSteps[A_TOWER] = 0;
        Printer::currentNonlinearPositionSteps[B_TOWER] = 0;
        Printer::currentNonlinearPositionSteps[C_TOWER] = 0;
        // similar to comment above, this will get a different answer from any different starting point
        // so it is unclear how this is helpful. It must start at a well defined point.
        Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
        int32_t offx = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtXHit;
        int32_t offy = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtYHit;
        int32_t offz = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtZHit;
        Com::printFLN(Com::tTower1, offx);
        Com::printFLN(Com::tTower2, offy);
        Com::printFLN(Com::tTower3, offz);
        Printer::setAutolevelActive(oldAuto);
        PrintLine::moveRelativeDistanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        Printer::homeAxis(true, true, true);
    }
    break;
    case 135: // G135
        Com::printF(PSTR("CompDelta:"), Printer::currentNonlinearPositionSteps[A_TOWER]);
        Com::printF(Com::tComma, Printer::currentNonlinearPositionSteps[B_TOWER]);
        Com::printFLN(Com::tComma, Printer::currentNonlinearPositionSteps[C_TOWER]);
#ifdef DEBUG_REAL_POSITION
        Com::printF(PSTR("RealDelta:"), Printer::realDeltaPositionSteps[A_TOWER]);
        Com::printF(Com::tComma, Printer::realDeltaPositionSteps[B_TOWER]);
        Com::printFLN(Com::tComma, Printer::realDeltaPositionSteps[C_TOWER]);
#endif
        Printer::updateCurrentPosition();
        Com::printF(PSTR("PosFromSteps:"));
        printCurrentPosition();
        break;

#endif // DRIVE_SYSTEM
#if FEATURE_Z_PROBE && NUM_EXTRUDER > 1
    case 134: {
        // - G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!) Px = reference extruder, Sx = only measure extrude x against reference, Zx = add to measured z distance for Sx for correction.
        float z = com->hasZ() ? com->Z : 0;
        int p = com->hasP() ? com->P : 0;
        int s = com->hasS() ? com->S : -1;
        int startExtruder = Extruder::current->id;
        extruder[p].zOffset = 0;
        float mins[NUM_EXTRUDER], maxs[NUM_EXTRUDER], avg[NUM_EXTRUDER];
        for(int i = 0; i < NUM_EXTRUDER; i++) { // silence unnecessary compiler warning
            avg[i] = 0;
        }
        bool bigError = false;

#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
        float actTemp[NUM_EXTRUDER];
        for(int i = 0; i < NUM_EXTRUDER; i++)
            actTemp[i] = extruder[i].tempControl.targetTemperatureC;
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
        Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
        }
        for(int i = 0; i < NUM_EXTRUDER; i++) {
            if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
                Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
        }
#else
        if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
            Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif

#ifndef G134_REPETITIONS
#define G134_REPETITIONS 3
#endif
#ifndef G134_PRECISION
#define G134_PRECISION 0.05
#endif
        Printer::startProbing(true);
        for(int r = 0; r < G134_REPETITIONS && !bigError; r++) {
            Extruder::selectExtruderById(p);
            float refHeight = Printer::runZProbe(false, false);
            if(refHeight == ILLEGAL_Z_PROBE) {
                bigError = true;
                break;
            }
            for(int i = 0; i < NUM_EXTRUDER && !bigError; i++) {
                if(i == p) continue;
                if(s >= 0 && i != s) continue;
                extruder[i].zOffset = 0;
                Extruder::selectExtruderById(i);
                float height = Printer::runZProbe(false, false);
                if(height == ILLEGAL_Z_PROBE) {
                    bigError = true;
                    break;
                }
                float off = (height - refHeight + z);
                if(r == 0) {
                    avg[i] = mins[i] = maxs[i] = off;
                } else {
                    avg[i] += off;
                    if(off < mins[i]) mins[i] = off;
                    if(off > maxs[i]) maxs[i] = off;
                    if(maxs[i] - mins[i] > G134_PRECISION) {
                        Com::printWarningFLN(PSTR("Deviation between measurements were too big, please repeat."));
						Com::printFLN(PSTR("Z Offset not computed due to errors"));
                        bigError = true;
                        break;
                    }
                }
            }
        }
        if(!bigError) {
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                if(s >= 0 && i != s) continue;
                extruder[i].zOffset = avg[i] * Printer::axisStepsPerMM[Z_AXIS] / G134_REPETITIONS;
            }
#if EEPROM_MODE != 0
            EEPROM::storeDataIntoEEPROM(0);
#endif
			Com::printFLN(PSTR("Z Offset stored"));
        } else {
			Com::printFLN(PSTR("Z Offset not computed due to errors"));
		}
        Extruder::selectExtruderById(startExtruder);
        Printer::finishProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
#if ZHOME_HEAT_ALL
        for(int i = 0; i < NUM_EXTRUDER; i++)
            Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
        for(int i = 0; i < NUM_EXTRUDER; i++)
            Extruder::setTemperatureForExtruder(actTemp[i], i, false, actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
        Extruder::setTemperatureForExtruder(actTemp[Extruder::current->id], Extruder::current->id, false, actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
    }
    break;
#endif
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    case 201:
        commandG201(*com);
        break;
    case 202:
        commandG202(*com);
        break;
    case 203:
        commandG203(*com);
        break;
    case 204:
        commandG204(*com);
        break;
    case 205:
        commandG205(*com);
        break;
#endif // defined
    default:
        if(Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
    previousMillisCmd = HAL::timeInMilliseconds();
}
/**
\brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode *com) {
    if(EVENT_UNHANDLED_M_CODE(com))
        return;
    switch( com->M ) {
    case 3: // Spindle/laser on
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        if(Printer::mode == PRINTER_MODE_LASER) {
            if(com->hasS())
                LaserDriver::intensity = constrain(com->S, 0, LASER_PWM_MAX);
            LaserDriver::laserOn = true;
            Com::printFLN(PSTR("LaserOn:"), (int)LaserDriver::intensity);
        }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOnCW(com->hasS() ? com->S : CNC_RPM_MAX);
        }
#endif // defined
        break;
    case 4: // Spindle CCW
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOnCCW(com->hasS() ? com->S : CNC_RPM_MAX);
        }
#endif // defined
        break;
    case 5: // Spindle/laser off
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        if(Printer::mode == PRINTER_MODE_LASER) {
            LaserDriver::laserOn = false;
        }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOff();
        }
#endif // defined
        break;
#if SDSUPPORT
    case 20: // M20 - list SD card
#if JSON_OUTPUT
        if (com->hasString() && com->text[1] == '2') { // " S2 P/folder"
            if (com->text[3] == 'P') {
                sd.lsJSON(com->text + 4);
            }
        } else sd.ls();
#else
        sd.ls();
#endif
        break;
    case 21: // M21 - init SD card
        sd.mount();
        break;
    case 22: //M22 - release SD card
        sd.unmount();
        break;
    case 23: //M23 - Select file
        if(com->hasString()) {
            sd.fat.chdir();
            sd.selectFile(com->text);
        }
        break;
    case 24: //M24 - Start SD print
        sd.startPrint();
        break;
    case 25: //M25 - Pause SD print
        sd.pausePrint();
        break;
    case 26: //M26 - Set SD index
        if(com->hasS())
            sd.setIndex(com->S);
        break;
    case 27: //M27 - Get SD status
        sd.printStatus();
        break;
    case 28: //M28 - Start SD write
        if(com->hasString())
            sd.startWrite(com->text);
        break;
    case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
    case 30: // M30 filename - Delete file
        if(com->hasString()) {
            sd.fat.chdir();
            sd.deleteFile(com->text);
        }
        break;
    case 32: // M32 directoryname
        if(com->hasString()) {
            sd.fat.chdir();
            sd.makeDirectory(com->text);
        }
        break;
#endif
#if JSON_OUTPUT && SDSUPPORT
    case 36: // M36 JSON File Info
        if (com->hasString()) {
            sd.JSONFileInfo(com->text);
        }
        break;
#endif
    case 42: //M42 -Change pin status via gcode
        if (com->hasP()) {
            int pin_number = com->P;
            for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
                if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
                    pin_number = -1;
                    break;
                }
            }
            if (pin_number > -1) {
                if(com->hasS()) {
                    if(com->S >= 0 && com->S <= 255) {
                        pinMode(pin_number, OUTPUT);
                        digitalWrite(pin_number, com->S);
                        analogWrite(pin_number, com->S);
                        Com::printF(Com::tSetOutputSpace, pin_number);
                        Com::printFLN(Com::tSpaceToSpace, (int)com->S);
                    } else
                        Com::printErrorFLN(PSTR("Illegal S value for M42"));
                } else {
                    pinMode(pin_number, INPUT_PULLUP);
                    Com::printF(Com::tSpaceToSpace, pin_number);
                    Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
                }
            } else {
                Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
            }
        }
        break;
    case 80: // M80 - ATX Power On
#if PS_ON_PIN > -1
        Commands::waitUntilEndOfAllMoves();
        previousMillisCmd = HAL::timeInMilliseconds();
        SET_OUTPUT(PS_ON_PIN); //GND
        Printer::setPowerOn(true);
        WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif
        break;
    case 81: // M81 - ATX Power Off
#if PS_ON_PIN > -1
        Commands::waitUntilEndOfAllMoves();
        SET_OUTPUT(PS_ON_PIN); //GND
        Printer::setPowerOn(false);
        WRITE(PS_ON_PIN, (POWER_INVERTING ? LOW : HIGH));
#endif
        break;
    case 82: // M82
        Printer::relativeExtruderCoordinateMode = false;
        break;
    case 83: // M83
        Printer::relativeExtruderCoordinateMode = true;
        break;
	case 18: // M18 is to disable named axis
        {
			Commands::waitUntilEndOfAllMoves();
			bool named = false;
			if(com->hasX()) {
				named = true;
				Printer::disableXStepper();
			}
			if(com->hasY()) {
				named = true;
				Printer::disableYStepper();
			}
			if(com->hasZ()) {
				named = true;
				Printer::disableZStepper();
			}
			if(com->hasE()) {
				named = true;
				Extruder::disableCurrentExtruderMotor();
			}
			if(!named) {
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();
				Extruder::disableAllExtruderMotors();
			}
		}
		break;
    case 84: // M84
        if(com->hasS()) {
            stepperInactiveTime = com->S * 1000;
        } else {
            Commands::waitUntilEndOfAllMoves();
            Printer::kill(true);
        }
        break;
    case 85: // M85
        if(com->hasS())
            maxInactiveTime = (int32_t)com->S * 1000;
        else
            maxInactiveTime = 0;
        break;
    case 92: // M92
        if(com->hasX()) Printer::axisStepsPerMM[X_AXIS] = com->X;
        if(com->hasY()) Printer::axisStepsPerMM[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::axisStepsPerMM[Z_AXIS] = com->Z;
        Printer::updateDerivedParameter();
        if(com->hasE()) {
            Extruder::current->stepsPerMM = com->E;
            Extruder::selectExtruderById(Extruder::current->id);
        }
        break;
    case 99: { // M99 S<time>
        millis_t wait = 10000;
        if(com->hasS())
            wait = 1000 * com->S;
        if(com->hasX())
            Printer::disableXStepper();
        if(com->hasY())
            Printer::disableYStepper();
        if(com->hasZ())
            Printer::disableZStepper();
        wait += HAL::timeInMilliseconds();
#ifdef DEBUG_PRINT
        debugWaitLoop = 2;
#endif
        while(wait - HAL::timeInMilliseconds() < 100000) {
            Printer::defaultLoopActions();
        }
        if(com->hasX())
            Printer::enableXStepper();
        if(com->hasY())
            Printer::enableYStepper();
        if(com->hasZ())
            Printer::enableZStepper();
    }
    break;

    case 104: // M104 temperature
#if NUM_EXTRUDER > 0
        if(reportTempsensorError()) break;
        previousMillisCmd = HAL::timeInMilliseconds();
        if(Printer::debugDryrun()) break;
#ifdef EXACT_TEMPERATURE_TIMING
        Commands::waitUntilEndOfAllMoves();
#else
        if(com->hasP() || (com->hasS() && com->S == 0))
            Commands::waitUntilEndOfAllMoves();
#endif
        if (com->hasS()) {
            if(com->hasT() && com->T < NUM_EXTRUDER)
                Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), com->T, com->hasF() && com->F > 0);
            else
                Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), Extruder::current->id, com->hasF() && com->F > 0);
        } else if(com->hasH()) {
            if(com->hasT() && com->T < NUM_EXTRUDER)
                Extruder::setTemperatureForExtruder(extruder[com->T].tempControl.preheatTemperature + (com->hasO() ? com->O : 0), com->T, com->hasF() && com->F > 0);
            else
                Extruder::setTemperatureForExtruder(Extruder::current->tempControl.preheatTemperature + (com->hasO() ? com->O : 0), Extruder::current->id, com->hasF() && com->F > 0);
        }
#endif
        break;
    case 140: // M140 set bed temp
#if HAVE_HEATED_BED
        if(reportTempsensorError()) break;
        previousMillisCmd = HAL::timeInMilliseconds();
        if(Printer::debugDryrun()) break;
        if (com->hasS()) Extruder::setHeatedBedTemperature(com->S + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
        else if(com->hasH()) Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
#endif
        break;
    case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
        Com::writeToAll = false;
        printTemperatures(com->hasX());
        break;
    case 109: // M109 - Wait for extruder heater to reach target.
#if NUM_EXTRUDER > 0
    {
        if(reportTempsensorError()) break;
        previousMillisCmd = HAL::timeInMilliseconds();
        if(Printer::debugDryrun()) break;
        Commands::waitUntilEndOfAllMoves();
        Extruder *actExtruder = Extruder::current;
        if(com->hasT() && com->T < NUM_EXTRUDER) actExtruder = &extruder[com->T];
        if (com->hasS()) Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), actExtruder->id, com->hasF() && com->F > 0, true);
        else if(com->hasH())  Extruder::setTemperatureForExtruder(actExtruder->tempControl.preheatTemperature + (com->hasO() ? com->O : 0), actExtruder->id, com->hasF() && com->F > 0, true);
    }
#endif
    previousMillisCmd = HAL::timeInMilliseconds();
    break;
    case 190: { // M190 - Wait bed for heater to reach target.
#if HAVE_HEATED_BED
        if(Printer::debugDryrun()) break;
        UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_HEATING_BED_ID));
        Commands::waitUntilEndOfAllMoves();
        if (com->hasS()) Extruder::setHeatedBedTemperature(com->S + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
        else if(com->hasH())  Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN > 0
        if(abs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) < SKIP_M190_IF_WITHIN) break;
#endif
        EVENT_WAITING_HEATER(-1);
        tempController[HEATED_BED_INDEX]->waitForTargetTemperature();
        EVENT_HEATING_FINISHED(-1);
#endif
        UI_CLEAR_STATUS;
        previousMillisCmd = HAL::timeInMilliseconds();
    }
    break;
    case 155: // M155 S<1/0> Enable/disable auto report temperatures. When enabled firmware will emit temperatures every second.
        Printer::setAutoreportTemp((com->hasS() && com->S != 0) || !com->hasS() );
        Printer::lastTempReport = HAL::timeInMilliseconds();
        break;
#if NUM_TEMPERATURE_LOOPS > 0
    case 116: // Wait for temperatures to reach target temperature
        for(fast8_t h = 0; h <= HEATED_BED_INDEX; h++) {
            EVENT_WAITING_HEATER(h < NUM_EXTRUDER ? h : -1);
            tempController[h]->waitForTargetTemperature();
            EVENT_HEATING_FINISHED(h < NUM_EXTRUDER ? h : -1);
        }
        break;
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    case 106: // M106 Fan On
        if(com->hasI()) {
            if(com->I != 0)
                Printer::flag2 |= PRINTER_FLAG2_IGNORE_M106_COMMAND;
            else
                Printer::flag2 &= ~PRINTER_FLAG2_IGNORE_M106_COMMAND;
        }
        if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
            if(com->hasP() && com->P == 1)
                setFan2Speed(com->hasS() ? com->S : 255);
            else
                setFanSpeed(com->hasS() ? com->S : 255);
        }
        break;
    case 107: // M107 Fan Off
        if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
            if(com->hasP() && com->P == 1)
                setFan2Speed(0);
            else
                setFanSpeed(0);
        }
        break;
#endif
    case 111: // M111 enable/disable run time debug flags
        if(com->hasS()) Printer::setDebugLevel(static_cast<uint8_t>(com->S));
        if(com->hasP()) {
            if (com->P > 0) Printer::debugSet(static_cast<uint8_t>(com->P));
            else Printer::debugReset(static_cast<uint8_t>(-com->P));
        }
        if(Printer::debugDryrun()) { // simulate movements without printing
#if NUM_EXTRUDER > 1
            for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
                Extruder::setTemperatureForExtruder(0, i);
#else
            Extruder::setTemperatureForExtruder(0, 0);
#endif
#if HAVE_HEATED_BED != 0
            Extruder::setHeatedBedTemperature(0, false);
#endif
        }
        break;
    case 115: // M115
        Com::writeToAll = false;
        Com::printFLN(Com::tFirmware);
#if FEATURE_CONTROLLER != NO_CONTROLLER
        Com::cap(PSTR("PROGRESS:1"));
#else
        Com::cap(PSTR("PROGRESS:0"));
#endif
        Com::cap(PSTR("AUTOREPORT_TEMP:1"));
#if EEPROM_MODE != 0
        Com::cap(PSTR("EEPROM:1"));
#else
        Com::cap(PSTR("EEPROM:0"));
#endif
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
        Com::cap(PSTR("AUTOLEVEL:1"));
#else
        Com::cap(PSTR("AUTOLEVEL:0"));
#endif
#if FEATURE_Z_PROBE
        Com::cap(PSTR("Z_PROBE:1"));
#else
        Com::cap(PSTR("Z_PROBE:0"));
#endif
#if PS_ON_PIN>-1
        Com::cap(PSTR("SOFTWARE_POWER:1"));
#else
        Com::cap(PSTR("SOFTWARE_POWER:0"));
#endif
#if CASE_LIGHTS_PIN > -1
        Com::cap(PSTR("TOGGLE_LIGHTS:1"));
#else
        Com::cap(PSTR("TOGGLE_LIGHTS:0"));
#endif
        Com::cap(PSTR("PAUSESTOP:1"));
        Com::cap(PSTR("PREHEAT:1"));
        reportPrinterUsage();
        Printer::reportPrinterMode();
        break;
    case 114: // M114
        Com::writeToAll = false;
        printCurrentPosition();
        if(com->hasS() && com->S) {
            Com::printF(PSTR("XS:"), Printer::currentPositionSteps[X_AXIS]);
            Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
            Com::printFLN(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);
        }
        break;
    case 117: // M117 message to lcd
        if(com->hasString()) {
            UI_STATUS_UPD_RAM(com->text);
#if JSON_OUTPUT && defined(WRITE_MESSAGES_To_JSON)
            Com::printF(PSTR("{\"message\":\""), com->text);
            Com::printFLN(PSTR("\"}"));
#endif
        }
        break;
    case 119: // M119
        Com::writeToAll = false;
        Commands::waitUntilEndOfAllMoves();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
        Endstops::report();
        break;
#if BEEPER_TYPE>0
    case 120: // M120 Test beeper function
        if(com->hasS() && com->hasP())
            beep(com->S, com->P); // Beep test
        break;
#endif
#if MIXING_EXTRUDER > 0
    case 163: // M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
        if(com->hasS() && com->hasP() && com->S < NUM_EXTRUDER && com->S >= 0)
            Extruder::setMixingWeight(com->S, com->P);
        Extruder::recomputeMixingExtruderSteps();
        break;
    case 164: /// M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights as virtual extruder S
        if(!com->hasS() || com->S < 0 || com->S >= VIRTUAL_EXTRUDER) break; // ignore illegal values
        for(uint8_t i = 0; i < NUM_EXTRUDER; i++) {
            extruder[i].virtualWeights[com->S] = extruder[i].mixingW;
        }
#if EEPROM_MODE != 0
        if(com->hasP() && com->P != 0)  // store permanently to eeprom
            EEPROM::storeMixingRatios();
#endif
        break;
#endif // MIXING_EXTRUDER
    case 170: // preheat temperatures
        /* M170 - Set or retrieve preheat temperatures
        Parameter:
        B<bedPreheat> : Sets bed preheat temperature
        C<chamberPreheat> : Sets heated chamber temperature
        T<extruder> S<preheatTemp> : Sets preheat temperature for given extruder
        L0 : List preheat temperatures. Returns
        PREHEAT_BED:temp PREHEAT0:extr0 PREHEAT1:extr1 PREHEAT_CHAMBER:temp
        */
    {
        bool mod = false;
#if HAVE_HEATED_BED
        if(com->hasB()) {
            mod |= heatedBedController.preheatTemperature != static_cast<int16_t>(com->B);
            heatedBedController.preheatTemperature = com->B;
        }
#endif
#if NUM_EXTRUDER > 0
        if(com->hasT() && com->hasS() && com->T < NUM_EXTRUDER) {
            mod |= extruder[com->T].tempControl.preheatTemperature != static_cast<int16_t>(com->S);
            extruder[com->T].tempControl.preheatTemperature = com->S;
        }
#endif
        if(com->hasL()) {
#if HAVE_HEATED_BED
            Com::printF(PSTR("PREHEAT_BED:"), heatedBedController.preheatTemperature);
#endif
#if NUM_EXTRUDER > 0
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                Com::printF(PSTR(" PREHEAT"), i);
                Com::printF(Com::tColon, extruder[i].tempControl.preheatTemperature);
            }
#endif
            Com::println();
        }
        if(mod) {
#if EEPROM_MODE != 0
#if HAVE_HEATED_BED
            HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP, heatedBedController.preheatTemperature);
#endif
#if NUM_EXTRUDER > 0
            for(int i = 0; i < NUM_EXTRUDER; i++) {
                int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
                Extruder *e = &extruder[i];
                HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT, e->tempControl.preheatTemperature);
            }
#endif
            EEPROM::updateChecksum();
#endif
        }
    }
    break;
    case 200: { // M200 T<extruder> D<diameter>
        uint8_t extruderId = Extruder::current->id;
        if(com->hasT() && com->T < NUM_EXTRUDER)
            extruderId = com->T;
        float d = 0;
        if(com->hasR())
            d = com->R;
        if(com->hasD())
            d = com->D;
        extruder[extruderId].diameter = d;
        if(extruderId == Extruder::current->id)
            changeFlowrateMultiply(Printer::extrudeMultiply);
        if(d == 0) {
            Com::printFLN(PSTR("Disabled volumetric extrusion for extruder "), static_cast<int>(extruderId));
        } else {
            Com::printF(PSTR("Set volumetric extrusion for extruder "), static_cast<int>(extruderId));
            Com::printFLN(PSTR(" to "), d);
        }
    }
    break;
#if RAMP_ACCELERATION
    case 201: // M201
        if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
        if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
        if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
    case 202: // M202
        if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
        if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
        if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
#endif
    case 203: // M203 Temperature monitor
		if(com->hasX()) {
			Printer::maxFeedrate[X_AXIS] = com->X / 60.0f;
		}
		if(com->hasY()) {
			Printer::maxFeedrate[Y_AXIS] = com->Y / 60.0f;
		}
		if(com->hasZ()) {
			Printer::maxFeedrate[Z_AXIS] = com->Z / 60.0f;
		}
		if(com->hasE()) {
			Printer::maxFeedrate[E_AXIS] = com->E / 60.0f;
		}
        if(com->hasS())
            manageMonitor = com->S != 255;
        else
            manageMonitor = 0;
        break;
    case 204: { // M204
        TemperatureController *temp = &Extruder::current->tempControl;
        if(com->hasS()) {
            if(com->S < 0) break;
            if(com->S < NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
#if HAVE_HEATED_BED
            else temp = &heatedBedController;
#else
            else break;
#endif
        }
        if(com->hasX()) temp->pidPGain = com->X;
        if(com->hasY()) temp->pidIGain = com->Y;
        if(com->hasZ()) temp->pidDGain = com->Z;
        temp->updateTempControlVars();
    }
    break;
    case 205: // M205 Show EEPROM settings
        Com::writeToAll = false;
        EEPROM::writeSettings();
        break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
        Com::writeToAll = false;
        EEPROM::update(com);
        break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
        if(com->hasX())
            Printer::maxJerk = com->X;
        if(com->hasE()) {
            Extruder::current->maxStartFeedrate = com->E;
            Extruder::selectExtruderById(Extruder::current->id);
        }
#if DRIVE_SYSTEM != DELTA
        if(com->hasZ())
            Printer::maxZJerk = com->Z;
        Com::printF(Com::tJerkColon, Printer::maxJerk);
        Com::printFLN(Com::tZJerkColon, Printer::maxZJerk);
#else
        Com::printFLN(Com::tJerkColon, Printer::maxJerk);
#endif
        break;
    case 209: // M209 S<0/1> Enable/disable autoretraction
        if(com->hasS())
            Printer::setAutoretract(com->S != 0);
        break;
	case 218:
		{
		 int extId = 0;
		 if(com->hasT()) extId = com->T;
		 if(extId >= 0 && extId < NUM_EXTRUDER) {
			if(com->hasX()) {
				extruder[extId].xOffset = com->X * Printer::axisStepsPerMM[X_AXIS];
			}
			if(com->hasY()) {
				 extruder[extId].yOffset = com->Y * Printer::axisStepsPerMM[Y_AXIS];
			}
			if(com->hasZ()) {
				extruder[extId].zOffset = com->Z * Printer::axisStepsPerMM[Z_AXIS];
			}
#if EEPROM_MODE > 0
			if(com->hasS() && com->S > 0) {
				EEPROM::storeDataIntoEEPROM(false);
			}
#endif
		  }
		}
		break;
    case 220: // M220 S<Feedrate multiplier in percent>
        changeFeedrateMultiply(com->getS(100));
        break;
    case 221: // M221 S<Extrusion flow multiplier in percent>
        changeFlowrateMultiply(com->getS(100));
        break;
    case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
        if(!com->hasS() || !com->hasP())
            break;
        {
            bool comp = com->S;
            if(com->hasX()) {
                if(com->X == 0)
                    HAL::pinMode(com->P, INPUT);
                else
                    HAL::pinMode(com->P, INPUT_PULLUP);
            }
            do {
                Commands::checkForPeriodicalActions(true);
                GCode::keepAlive(WaitHeater);
            } while(HAL::digitalRead(com->P) != comp);
        }
        break;
#if USE_ADVANCE
    case 223: // M223 Extruder interrupt test
        if(com->hasS()) {
            InterruptProtectedBlock noInts;
            Printer::extruderStepsNeeded += com->S;
        }
        break;
    case 232: // M232
        Com::printF(Com::tLinearStepsColon, maxadv2);
#if ENABLE_QUADRATIC_ADVANCE
        Com::printF(Com::tQuadraticStepsColon, maxadv);
#endif
        Com::printFLN(Com::tCommaSpeedEqual, maxadvspeed);
#if ENABLE_QUADRATIC_ADVANCE
        maxadv = 0;
#endif
        maxadv2 = 0;
        maxadvspeed = 0;
        break;
#endif
#if USE_ADVANCE
    case 233: // M233
        if(com->hasY())
            Extruder::current->advanceL = com->Y;
        Com::printF(Com::tLinearLColon, Extruder::current->advanceL);
#if ENABLE_QUADRATIC_ADVANCE
        if(com->hasX())
            Extruder::current->advanceK = com->X;
        Com::printF(Com::tQuadraticKColon, Extruder::current->advanceK);
#endif
        Com::println();
        Printer::updateAdvanceFlags();
        break;
#endif
#if Z_HOME_DIR > 0 && MAX_HARDWARE_ENDSTOP_Z
    case 251: // M251
        Printer::zLength -= Printer::currentPosition[Z_AXIS];
        Printer::currentPositionSteps[Z_AXIS] = 0;
        Printer::updateDerivedParameter();
#if NONLINEAR_SYSTEM
        transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
#endif
        Printer::updateCurrentPosition();
        Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
        Com::printFLN(Com::tEEPROMUpdated);
#endif
        Commands::printCurrentPosition();
        break;
#endif
#if FEATURE_DITTO_PRINTING
    case 280: // M280
#if DUAL_X_AXIS
        Extruder::dittoMode = 0;
        if(Extruder::current->id != 0)
            Extruder::selectExtruderById(0);
        Printer::homeXAxis();
        if(com->hasS() && com->S > 0) {
#if LAZY_DUAL_X_AXIS
            PrintLine::moveRelativeDistanceInSteps(-Extruder::current->xOffset, 0, 0, 0, EXTRUDER_SWITCH_XY_SPEED, true, true);
#endif
            Extruder::current = &extruder[1];
            PrintLine::moveRelativeDistanceInSteps(-Extruder::current->xOffset + static_cast<int32_t>(Printer::xLength * 0.5 * Printer::axisStepsPerMM[X_AXIS]), 0, 0, 0, EXTRUDER_SWITCH_XY_SPEED, true, true);
            Printer::currentPositionSteps[X_AXIS] = Printer::xMinSteps;
            Extruder::current = &extruder[0];
            Extruder::dittoMode = 1;
        }
        Printer::updateCurrentPosition(true);
#else
        if(com->hasS()) { // Set ditto mode S: 0 = off, 1 = 1 extra extruder, 2 = 2 extra extruder, 3 = 3 extra extruders
            Extruder::dittoMode = com->S;
        }
#endif
        break;
#endif
    case 281: // Trigger watchdog
#if FEATURE_WATCHDOG
    {
        if(com->hasX()) {
            HAL::stopWatchdog();
            Com::printFLN(PSTR("Watchdog disabled"));
            break;
        }
        Com::printInfoFLN(PSTR("Triggering watchdog. If activated, the printer will reset."));
        Printer::kill(false);
        HAL::delayMilliseconds(200); // write output, make sure heaters are off for safety
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
        InterruptProtectedBlock noInts;         // don't disable interrupts on mega2560 and mega1280 because of bootloader bug
#endif
        while(1) {} // Endless loop
    }
#else
    Com::printInfoFLN(PSTR("Watchdog feature was not compiled into this version!"));
#endif
    break;
#if FEATURE_BABYSTEPPING
    case 290: // M290 Z<babysteps> - Correct by adding baby steps for Z mm
        if(com->hasZ()) {
            if(abs(com->Z) < (32700 - labs(Printer::zBabystepsMissing)) * Printer::axisStepsPerMM[Z_AXIS])
                Printer::zBabystepsMissing += com->Z * Printer::axisStepsPerMM[Z_AXIS];
        }
        break;
#endif
#if defined(BEEPER_PIN) && BEEPER_PIN>=0
    case 300: { // M300
        int beepS = 1;
        int beepP = 1000;
        if(com->hasS()) beepS = com->S;
        if(com->hasP()) beepP = com->P;
        HAL::tone(BEEPER_PIN, beepS);
        HAL::delayMilliseconds(beepP);
        HAL::noTone(BEEPER_PIN);
    }
    break;
#endif
    case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will allow, S0 will disallow.
        Printer::setColdExtrusionAllowed(!com->hasS() || (com->hasS() && com->S != 0));
        break;
    case 303: { // M303
#if NUM_TEMPERATURE_LOOPS > 0
        int temp = 150;
        int cont = 0;
        int cycles = 5;
        int method = 0;
        if(com->hasS()) temp = com->S;
        if(com->hasP()) cont = com->P;
        if(com->hasR()) cycles = static_cast<int>(com->R);
        if(com->hasC()) method = static_cast<int>(com->C);
        if(cont >= HEATED_BED_INDEX) cont = HEATED_BED_INDEX;
        if(cont < 0) cont = 0;
        tempController[cont]->autotunePID(temp, cont, cycles, com->hasX(), method);
#endif
    }
    break;

#if FEATURE_AUTOLEVEL
    case 320: // M320 Activate autolevel
        Printer::setAutolevelActive(true);
        if(com->hasS() && com->S) {
            EEPROM::storeDataIntoEEPROM();
        }
        break;
    case 321: // M321 Deactivate autoleveling
        Printer::setAutolevelActive(false);
        if(com->hasS() && com->S) {
            if(com->S == 3)
                Printer::resetTransformationMatrix(false);
            EEPROM::storeDataIntoEEPROM();
        }
        break;
    case 322: // M322 Reset auto leveling matrix
        Printer::resetTransformationMatrix(false);
        if(com->hasS() && com->S) {
            EEPROM::storeDataIntoEEPROM();
        }
        break;
#endif // FEATURE_AUTOLEVEL
#if DISTORTION_CORRECTION
    case 323: // M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
        if(com->hasS()) {
            if(com->S > 0)
                Printer::distortion.enable(com->hasP() && com->P == 1);
            else
                Printer::distortion.disable(com->hasP() && com->P == 1);
        } else {
            Printer::distortion.reportStatus();
        }
        break;
#endif // DISTORTION_CORRECTION
#if FEATURE_SERVO
    case 340: // M340
        if(com->hasP() && com->P < 4 && com->P >= 0) {
			ENSURE_POWER
            int s = 0;
            if(com->hasS())
                s = com->S;
            uint16_t r = 0;
            if(com->hasR())    // auto off time in ms
                r = com->R;
            HAL::servoMicroseconds(com->P, s, r);
        }
        break;
#endif // FEATURE_SERVO
    case 350: { // M350 Set micro stepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
#if (defined(X_MS1_PIN) && X_MS1_PIN > -1) || defined(DRV_TMC2130)
        if(com->hasS()) for(int i = 0; i <= 4; i++) microstepMode(i, com->S);
        if(com->hasX()) microstepMode(0, (uint8_t)com->X);
        if(com->hasY()) microstepMode(1, (uint8_t)com->Y);
        if(com->hasZ()) microstepMode(2, (uint8_t)com->Z);
        if(com->hasE()) microstepMode(3, (uint8_t)com->E);
        if(com->hasP()) microstepMode(4, (uint8_t)com->P); // Original B but is not supported here
        if(com->hasR()) microstepMode(5, (uint8_t)com->R);
        microstepReadings();
#endif
    }
    break;
    case 355: // M355 S<0/1> - Turn case light on/off, no S = report status
        if(com->hasS()) {
            Printer::setCaseLight(com->S);
        } else
            Printer::reportCaseLightStatus();
        break;
    case 360: // M360 - show configuration
        Com::writeToAll = false;
        Printer::showConfiguration();
        break;
    case 400: // M400 Finish all moves
        Commands::waitUntilEndOfAllMoves();
        break;
    case 401: // M401 Memory position
        Printer::MemoryPosition();
        break;
    case 402: // M402 Go to stored position
        Printer::GoToMemoryPosition(com->hasX(), com->hasY(), com->hasZ(), com->hasE(), (com->hasF() ? com->F : Printer::feedrate));
        break;
#if JSON_OUTPUT
    case 408:
        Printer::showJSONStatus(com->hasS() ? static_cast<int>(com->S) : 0);
        break;
#endif
    case 450:
        Printer::reportPrinterMode();
        break;
    case 451:
        waitUntilEndOfAllMoves();
        Printer::mode = PRINTER_MODE_FFF;
        Printer::reportPrinterMode();
        break;
    case 452:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        waitUntilEndOfAllMoves();
        Printer::mode = PRINTER_MODE_LASER;
#endif
        Printer::reportPrinterMode();
        break;
    case 453:
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        waitUntilEndOfAllMoves();
        Printer::mode = PRINTER_MODE_CNC;
#endif
        Printer::reportPrinterMode();
        break;
#if FAN_THERMO_PIN > -1
    case 460: // M460 X<minTemp> Y<maxTemp> : Set temperature range for thermo controlled fan
        if(com->hasX())
            Printer::thermoMinTemp = com->X;
        if(com->hasY())
            Printer::thermoMaxTemp = com->Y;
        break;
#endif
    case 500: { // M500
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
        Com::printInfoFLN(Com::tConfigStoredEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 501: { // M501
#if EEPROM_MODE != 0
        EEPROM::readDataFromEEPROM(true);
        Extruder::selectExtruderById(Extruder::current->id);
        Com::printInfoFLN(Com::tConfigLoadedEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 502: // M502
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        break;
#if EXTRUDER_JAM_CONTROL
#ifdef DEBUG_JAM
    case 512:
        Com::printFLN(PSTR("Jam signal:"), (int16_t)READ(EXT0_JAM_PIN));
        break;
#endif // DEBUG_JAM
    case 513:
        Extruder::markAllUnjammed();
        break;
#endif // EXTRUDER_JAM_CONTROL
//- M530 S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
    case 530:
        if(com->hasL())
            Printer::maxLayer = static_cast<int>(com->L);
        if(com->hasS())
            Printer::setPrinting(static_cast<uint8_t>(com->S));
        else {
            Printer::setPrinting(0);
        }
        Printer::setMenuMode(MENU_MODE_PAUSED, false);
        UI_RESET_MENU
        break;
//- M531 filename - Define filename being printed
    case 531:
        strncpy(Printer::printName, com->text, 20);
        Printer::printName[20] = 0;
        break;
//- M532 X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
    case 532:
        if(com->hasX())
            Printer::progress = com->X;
        if(Printer::progress > 100.0)
            Printer::progress = 100.0;
        else if(Printer::progress < 0)
            Printer::progress = 0;
        if(com->hasL())
            Printer::currentLayer = static_cast<int>(com->L);
        break;
#ifdef DEBUG_QUEUE_MOVE
    case 533: { // M533 Write move data
        InterruptProtectedBlock noInts;
        int lc = (int)PrintLine::linesCount;
        int lp = (int)PrintLine::linesPos;
        int wp = (int)PrintLine::linesWritePos;
        int n = (wp - lp);
        if(n < 0) n += PRINTLINE_CACHE_SIZE;
        noInts.unprotect();
        if(n != lc)
            Com::printFLN(PSTR("Buffer corrupted"));
        Com::printF(PSTR("Buf:"), lc);
        Com::printF(PSTR(",LP:"), lp);
        Com::printFLN(PSTR(",WP:"), wp);
        if(PrintLine::cur == NULL) {
            Com::printFLN(PSTR("No move"));
            if(PrintLine::linesCount > 0) {
                PrintLine &cur = PrintLine::lines[PrintLine::linesPos];
                Com::printF(PSTR("JFlags:"), (int)cur.joinFlags);
                Com::printFLN(PSTR(" Flags:"), (int)cur.flags);
                if(cur.isWarmUp()) {
                    Com::printFLN(PSTR(" warmup:"), (int)cur.getWaitForXLinesFilled());
                }
            }
        } else {
            Com::printF(PSTR("Rem:"), PrintLine::cur->stepsRemaining);
            Com::printFLN(PSTR(" Int:"), Printer::interval);
        }
    }
    break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
    case 534: // M534
        Com::printFLN(PSTR("Max. segment size:"), Printer::maxRealSegmentLength);
        if(com->hasS())
            Printer::maxRealSegmentLength = 0;
        break;
#endif
#ifdef DEBUG_REAL_JERK
        Com::printFLN(PSTR("Max. jerk measured:"), Printer::maxRealJerk);
        if(com->hasS())
            Printer::maxRealJerk = 0;
        break;
#endif
    /*      case 535:  // M535
    Com::printF(PSTR("Last commanded position:"),Printer::lastCmdPos[X_AXIS]);
    Com::printF(Com::tComma,Printer::lastCmdPos[Y_AXIS]);
    Com::printFLN(Com::tComma,Printer::lastCmdPos[Z_AXIS]);
    Com::printF(PSTR("Current position:"),Printer::currentPosition[X_AXIS]);
    Com::printF(Com::tComma,Printer::currentPosition[Y_AXIS]);
    Com::printFLN(Com::tComma,Printer::currentPosition[Z_AXIS]);
    Com::printF(PSTR("Position steps:"),Printer::currentPositionSteps[X_AXIS]);
    Com::printF(Com::tComma,Printer::currentPositionSteps[Y_AXIS]);
    Com::printFLN(Com::tComma,Printer::currentPositionSteps[Z_AXIS]);
    #if NONLINEAR_SYSTEM
    Com::printF(PSTR("Nonlin. position steps:"),Printer::currentDeltaPositionSteps[A_TOWER]);
    Com::printF(Com::tComma,Printer::currentDeltaPositionSteps[B_TOWER]);
    Com::printFLN(Com::tComma,Printer::currentDeltaPositionSteps[C_TOWER]);
    #endif // NONLINEAR_SYSTEM
    break;*/
    /* case 700: // M700 test new square root function
    if(com->hasS())
    Com::printFLN(Com::tInfo,(int32_t)HAL::integerSqrt(com->S));
    break;*/
    case 539:
        if(com->hasS()) {
            Printer::setSupportStartStop(com->S != 0);
        }
        if(com->hasP()) {
            if(com->P) {
                Printer::setMenuMode(MENU_MODE_PAUSED, true);
            } else {
                Printer::setMenuMode(MENU_MODE_PAUSED, false);
                UI_RESET_MENU
            }
        }
        break;
#if FEATURE_CONTROLLER != NO_CONTROLLER && FEATURE_RETRACTION
    case 600:
        uid.executeAction(UI_ACTION_WIZARD_FILAMENTCHANGE, true);
        break;
#endif
    case 601:
        if(com->hasS() && com->S > 0)
            Extruder::pauseExtruders(com->hasB() && com->B != 0);
        else
            Extruder::unpauseExtruders(com->hasP() && com->P != 1);
        break;
#if EXTRUDER_JAM_CONTROL && NUM_EXTRUDER > 0
    case 602:
        Commands::waitUntilEndOfAllMoves();
        if(com->hasS()) Printer::setDebugJam(com->S > 0);
        if(com->hasP()) Printer::setJamcontrolDisabled(com->P > 0);
        break;
    case 603:
        Printer::setInterruptEvent(PRINTER_INTERRUPT_EVENT_JAM_DETECTED, true);
        break;
    case 604: {
        uint8_t extId = Extruder::current->id;
        if(com->hasT()) extId = com->T;
        if(extId >= NUM_EXTRUDER)
            break;
        Extruder &ext = extruder[extId];
        if(com->hasX())
            ext.jamSlowdownSteps = static_cast<int32_t>(com->X);
        if(com->hasY())
            ext.jamErrorSteps = static_cast<int32_t>(com->Y);
        if(com->hasZ())
            ext.jamSlowdownTo = static_cast<uint8_t>(com->Z);
    }
    break;
#endif
    case 670:
#if EEPROM_MODE != 0
        if(com->hasS()) {
            HAL::eprSetByte(EPR_VERSION, static_cast<uint8_t>(com->S));
            HAL::eprSetByte(EPR_INTEGRITY_BYTE, EEPROM::computeChecksum());
        }
#endif
        break;
    case 907: { // M907 Set digital trimpot/DAC motor current using axis codes.
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
        // If "S" is specified, use that as initial default value, then update each axis w/ specific values as found later.
        if(com->hasS()) {
            for(int i = 0; i < 10; i++) {
                setMotorCurrentPercent(i, com->S);
            }
        }

        if(com->hasX()) setMotorCurrentPercent(0, (float)com->X);
        if(com->hasY()) setMotorCurrentPercent(1, (float)com->Y);
        if(com->hasZ()) setMotorCurrentPercent(2, (float)com->Z);
        if(com->hasE()) setMotorCurrentPercent(3, (float)com->E);
#endif
    }
    break;
    case 908: { // M908 Control digital trimpot directly.
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
        uint8_t channel, current;
        if(com->hasP() && com->hasS()) {
            setMotorCurrent((uint8_t)com->P, (unsigned int)com->S);
        }
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_TMC2130
        motorCurrentReadings();
#endif
#endif
    }
    break;
    case 909: { // M909 Read digital trimpot settings.
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
        dacPrintValues();
#endif
    }
    break;
    case 910: // M910 - Commit digipot/DAC value to external EEPROM
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
        dacCommitEeprom();
#endif
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
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    case 890: {
        if(com->hasX() && com->hasY()) {
            float c = Printer::bendingCorrectionAt(com->X, com->Y);
            Com::printF(PSTR("Bending at ("), com->X);
            Com::printF(PSTR(","), com->Y);
            Com::printFLN(PSTR(") = "), c);
        }
    }
    break;
#endif
	case 998:
		UI_MESSAGE(com->S);
		break;
    case 999: // Stop fatal error take down
        if(com->hasS())
            GCode::fatalError(PSTR("Testing fatal error"));
        else
            GCode::resetFatalError();
        break;
#if defined(DRV_TMC2130)
    case 914: // Configure stallguard threshold on Trinamic TMC2130
        Com::printF(PSTR("Trinamic SGT"));
        if(com->hasNoXYZ()) {
#if TMC2130_ON_X
            Com::printF(PSTR(" X:"), Printer::tmc_driver_x->sg_stall_value());
#endif
#if TMC2130_ON_Y
            Com::printF(PSTR(" Y:"), Printer::tmc_driver_y->sg_stall_value());
#endif
#if TMC2130_ON_Z
            Com::printF(PSTR(" Z:"), Printer::tmc_driver_z->sg_stall_value());
#endif
#if TMC2130_ON_EXT0
            Com::printF(PSTR(" E0:"), Printer::tmc_driver_e0->sg_stall_value());
#endif
#if TMC2130_ON_EXT1
            Com::printF(PSTR(" E1:"), Printer::tmc_driver_e1->sg_stall_value());
#endif
#if TMC2130_ON_EXT2
            Com::printF(PSTR(" E2:"), Printer::tmc_driver_e2->sg_stall_value());
#endif
        }
#if TMC2130_ON_X
        if(com->hasX()) {
            Com::printF(PSTR(" X:"), com->X);
            Printer::tmc_driver_x->sg_stall_value(com->X);
        }
#endif
#if TMC2130_ON_Y
        if(com->hasY()) {
            Com::printF(PSTR(" Y:"), com->Y);
            Printer::tmc_driver_y->sg_stall_value(com->Y);
        }
#endif
#if TMC2130_ON_Z
        if(com->hasZ()) {
            Com::printF(PSTR(" Z:"), com->Z);
            Printer::tmc_driver_z->sg_stall_value(com->Z);
        }
#endif
#if TMC2130_ON_EXT0
        if(com->hasE()) {
            Com::printF(PSTR(" E0:"), com->E);
            Printer::tmc_driver_e0->sg_stall_value(com->E);
        }
#endif
#if TMC2130_ON_EXT1
        if(com->hasE()) {
            Com::printF(PSTR(" E1:"), com->F);
            Printer::tmc_driver_e1->sg_stall_value(com->F);
        }
#endif
#if TMC2130_ON_EXT2
        if(com->hasE()) {
            Com::printF(PSTR(" E2:"), com->G);
            Printer::tmc_driver_e2->sg_stall_value(com->G);
        }
#endif
        Com::println();
    break;
    case 915: // Configure StealthChop on Trinamic TMC2130
        Com::printF(PSTR("Trinamic StealthChop"));
        if(com->hasNoXYZ()) {
#if TMC2130_ON_X
            Com::printF(PSTR(" X:"), Printer::tmc_driver_x->stealthChop());
#endif
#if TMC2130_ON_Y
            Com::printF(PSTR(" Y:"), Printer::tmc_driver_y->stealthChop());
#endif
#if TMC2130_ON_Z
            Com::printF(PSTR(" Z:"), Printer::tmc_driver_z->stealthChop());
#endif
#if TMC2130_ON_EXT0
            Com::printF(PSTR(" E0:"), Printer::tmc_driver_e0->stealthChop());
#endif
#if TMC2130_ON_EXT1
            Com::printF(PSTR(" E1:"), Printer::tmc_driver_e1->stealthChop());
#endif
#if TMC2130_ON_EXT2
            Com::printF(PSTR(" E2:"), Printer::tmc_driver_e2->stealthChop());
#endif
        }
#if TMC2130_ON_X
        if(com->hasX()) {
            Com::printF(PSTR(" X:"), com->X);
            Printer::tmc_driver_x->stealthChop((bool)(com->X));
        }
#endif
#if TMC2130_ON_Y
        if(com->hasY()) {
            Com::printF(PSTR(" Y:"), com->Y);
           Printer::tmc_driver_y->stealthChop((bool)(com->Y));
        }
#endif
#if TMC2130_ON_Z
        if(com->hasZ()) {
            Com::printF(PSTR(" Z:"), com->Z);
            Printer::tmc_driver_z->stealthChop((bool)(com->Z));
        }
#endif
#if TMC2130_ON_EXT0
        if(com->hasE()) {
            Com::printF(PSTR(" E0:"), com->E);
            Printer::tmc_driver_e0->stealthChop((bool)(com->E));
        }
#endif
#if TMC2130_ON_EXT1
        if(com->hasE()) {
            Com::printF(PSTR(" E1:"), com->F);
            Printer::tmc_driver_e1->stealthChop((bool)(com->F));
        }
#endif
#if TMC2130_ON_EXT2
        if(com->hasE()) {
            Com::printF(PSTR(" E2:"), com->G);
            Printer::tmc_driver_e2->stealthChop((bool)(com->G));
        }
#endif
        Com::println();
    break;
#endif
    default:
        if(Printer::debugErrors()) {
            Com::writeToAll = false;
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}

/**
\brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com) {
#if NEW_COMMUNICATION
    // Set return channel for private commands. By default all commands send to all receivers.
    GCodeSource *actSource = GCodeSource::activeSource;
    GCodeSource::activeSource = com->source;
    Com::writeToAll = true;
#endif
    if (INCLUDE_DEBUG_COMMUNICATION) {
        if(Printer::debugCommunication()) {
            if(com->hasG() || (com->hasM() && com->M != 111)) {
                previousMillisCmd = HAL::timeInMilliseconds();
#if NEW_COMMUNICATION
                GCodeSource::activeSource = actSource;
#endif
                return;
            }
        }
    }
    if(com->hasG()) processGCode(com);
    else if(com->hasM()) processMCode(com);
    else if(com->hasT()) {    // Process T code
        //com->printCommand(); // for testing if this the source of extruder switches
        Commands::waitUntilEndOfAllMoves();
        Extruder::selectExtruderById(com->T);
    } else {
        if(Printer::debugErrors()) {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#ifdef DEBUG_DRYRUN_ERROR
    if(Printer::debugDryrun()) {
        Com::printFLN("Dryrun was enabled");
        com->printCommand();
        Printer::debugReset(8);
    }
#endif
#if NEW_COMMUNICATION
    GCodeSource::activeSource = actSource;
#endif
}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
    HAL::resetHardware();
#else
    //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
    Printer::kill(false);
    Extruder::manageTemperatures();
    for(uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
        pwm_pos[i] = 0;
#if EXT0_HEATER_PIN > -1 && NUM_EXTRUDER > 0
    WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN > -1 && NUM_EXTRUDER > 1
    WRITE(EXT1_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN > -1 && NUM_EXTRUDER > 2
    WRITE(EXT2_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN > -1 && NUM_EXTRUDER > 3
    WRITE(EXT3_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN > -1 && NUM_EXTRUDER > 4
    WRITE(EXT4_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN > -1 && NUM_EXTRUDER > 5
    WRITE(EXT5_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    WRITE(FAN_PIN, 0);
#endif
#if HAVE_HEATED_BED && HEATED_BED_HEATER_PIN > -1
    WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);
#endif
    UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_KILLED_ID));
    HAL::delayMilliseconds(200);
    InterruptProtectedBlock noInts;
    while(1) {}
#endif
}

void Commands::checkFreeMemory() {
    int newfree = HAL::getFreeRam();
    if(newfree < lowestRAMValue)
        lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
    if(lowestRAMValueSend > lowestRAMValue) {
        lowestRAMValueSend = lowestRAMValue;
        Com::printFLN(Com::tFreeRAM, lowestRAMValue);
    }
}
