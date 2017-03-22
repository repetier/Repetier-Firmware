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

#ifndef FEATURE_CHECKSUM_FORCED
#define FEATURE_CHECKSUM_FORCED false
#endif

GCode    GCode::commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
uint8_t  GCode::bufferReadIndex = 0; ///< Read position in gcode_buffer.
uint8_t  GCode::bufferWriteIndex = 0; ///< Write position in gcode_buffer.
uint8_t  GCode::commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
uint8_t  GCode::commandsReceivingWritePosition = 0; ///< Writing position in gcode_transbuffer.
uint8_t  GCode::sendAsBinary; ///< Flags the command as binary input.
uint8_t  GCode::wasLastCommandReceivedAsBinary = 0; ///< Was the last successful command in binary mode?
uint8_t  GCode::commentDetected = false; ///< Flags true if we are reading the comment part of a command.
uint8_t  GCode::binaryCommandSize; ///< Expected size of the incoming binary command.
bool     GCode::waitUntilAllCommandsAreParsed = false; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
uint32_t GCode::lastLineNumber = 0; ///< Last line number received.
uint32_t GCode::actLineNumber; ///< Line number of current command.
int8_t   GCode::waitingForResend = -1; ///< Waiting for line to be resend. -1 = no wait.
volatile uint8_t GCode::bufferLength = 0; ///< Number of commands stored in gcode_buffer
millis_t GCode::timeOfLastDataPacket = 0; ///< Time, when we got the last data packet. Used to detect missing uint8_ts.
uint8_t  GCode::formatErrors = 0;
PGM_P GCode::fatalErrorMsg = NULL; ///< message unset = no fatal error 
millis_t GCode::lastBusySignal = 0; ///< When was the last busy signal
uint32_t GCode::keepAliveInterval = KEEP_ALIVE_INTERVAL;

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcommings in the standard
RepRap communication method, while remaining backward compatible. To use the improved
features of this protocal, you need a host which speaks it. On Windows the recommended
host software is Repetier-Host. It is developed in parallel to this firmware and supports
all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce communication failures.
  The standard method is to transfer a checksum at the end of the line. This checksum is the
  XORd value of all characters send. The value is limited to a range between 0 and 127. It can
  not detect two identical missing characters or a wrong order. Therefore the new protocol
  uses Fletchers checksum, which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to less then 50% and
  it speeds up decoding the command. No slow conversion from string to floats are needed.

*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 uint8_ts define the
data. From this bitfield, this function computes the size of the command
including the 2 uint8_ts of the bitfield and the 2 uint8_ts for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned uint8_t
- G : Bit 2 :  8-Bit unsigned uint8_t
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more uint8_ts following with Bits, only for future versions
- Int :Bit 14 : Marks it as internal command,
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
- D : Bit 3 : 32-Bit float
- C : Bit 4 : 32-Bit float
- H : Bit 5 : 32-Bit float
- A : Bit 6 : 32-Bit float
- B : Bit 7 : 32-Bit float
- K : Bit 8 : 32-Bit float
- L : Bit 9 : 32-Bit float
- O : Bit 0 : 32-Bit float
*/
uint8_t GCode::computeBinarySize(char *ptr)  // unsigned int bitfield) {
{
    uint8_t s = 4; // include checksum and bitfield
    uint16_t bitfield = *(uint16_t*)ptr;
    if(bitfield & 1) s += 2;
    if(bitfield & 8) s += 4;
    if(bitfield & 16) s += 4;
    if(bitfield & 32) s += 4;
    if(bitfield & 64) s += 4;
    if(bitfield & 256) s += 4;
    if(bitfield & 512) s += 1;
    if(bitfield & 1024) s += 4;
    if(bitfield & 2048) s += 4;
    if(bitfield & 4096)   // Version 2 or later
    {
        s += 2; // for bitfield 2
        uint16_t bitfield2 = *(uint16_t*)(ptr + 2);
        if(bitfield & 2) s += 2;
        if(bitfield & 4) s += 2;
        if(bitfield2 & 1) s += 4;
        if(bitfield2 & 2) s += 4;
        if(bitfield2 & 4) s += 4;
        if(bitfield2 & 8) s += 4;
        if(bitfield2 & 16) s += 4;
        if(bitfield2 & 32) s += 4;
        if(bitfield2 & 64) s += 4;
        if(bitfield2 & 128) s += 4;
        if(bitfield2 & 256) s += 4;
        if(bitfield2 & 512) s += 4;
        if(bitfield2 & 1024) s += 4;
        if(bitfield2 & 2048) s += 4;
        if(bitfield2 & 4096) s += 4;
        if(bitfield2 & 8192) s += 4;
        if(bitfield2 & 16384) s += 4;
        if(bitfield2 & 32768) s += 4;
        if(bitfield & 32768) s += RMath::min(80,(uint8_t)ptr[4] + 1);
    }
    else
    {
        if(bitfield & 2) s += 1;
        if(bitfield & 4) s += 1;
        if(bitfield & 32768) s += 16;
    }
    return s;
}

void GCode::keepAlive(enum FirmwareState state) {
	millis_t now = HAL::timeInMilliseconds();
	
	if(state != NotBusy && keepAliveInterval != 0) {
		if(now - lastBusySignal < keepAliveInterval)
			return;
		if(state == Paused) {
			Com::printFLN(PSTR("busy:paused for user interaction"));	
		} else if(state == WaitHeater) {
			Com::printFLN(PSTR("busy:heating"));	
		} else { // processing and uncaught cases
			Com::printFLN(PSTR("busy:processing"));
		}
	}
	lastBusySignal = now;
}

void GCode::requestResend()
{
    HAL::serialFlush();
    commandsReceivingWritePosition = 0;
    if(sendAsBinary)
        waitingForResend = 30;
    else
        waitingForResend = 14;
    Com::println();
    Com::printFLN(Com::tResend,lastLineNumber + 1);
    Com::printFLN(Com::tOk);
}

/**
  Check if result is plausible. If it is, an ok is send and the command is stored in queue.
  If not, a resend and ok is send.
*/
void GCode::checkAndPushCommand()
{
    if(hasM())
    {
        if(M == 110)   // Reset line number
        {
            lastLineNumber = actLineNumber;
            Com::printFLN(Com::tOk);
            waitingForResend = -1;
            return;
        }
        if(M == 112)   // Emergency kill - freeze printer
        {
            Commands::emergencyStop();
        }
#ifdef DEBUG_COM_ERRORS
        if(M == 666) // force an communication error
        {
            lastLineNumber++;
            return;
        } else if(M == 668) {
            lastLineNumber = 0;  // simulate a reset so lines are out of resend buffer
        }
#endif // DEBUG_COM_ERRORS
    }
    if(hasN())
    {
        if((((lastLineNumber + 1) & 0xffff) != (actLineNumber & 0xffff)))
        {
            if(static_cast<uint16_t>(lastLineNumber - actLineNumber) < 40)
            {
                // we have seen that line already. So we assume it is a repeated resend and we ignore it
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip,actLineNumber);
                Com::printFLN(Com::tOk);
            }
            else if(waitingForResend < 0)  // after a resend, we have to skip the garbage in buffers, no message for this
            {
                if(Printer::debugErrors())
                {
                    Com::printF(Com::tExpectedLine, lastLineNumber + 1);
                    Com::printFLN(Com::tGot, actLineNumber);
                }
                requestResend(); // Line missing, force resend
            }
            else
            {
                --waitingForResend;
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip, actLineNumber);
                Com::printFLN(Com::tOk);
            }
            return;
        }
        lastLineNumber = actLineNumber;
    } /*
	This test is not compatible with all hosts. Replaced by forbidding backward switch of protocols.
	else if(lastLineNumber && !(hasM() && M == 117)) { // once line number always line number!
		if(Printer::debugErrors())
        {
			Com::printErrorFLN(PSTR("Missing linenumber"));
		}
		requestResend();
		return;
	}*/
	if(GCode::hasFatalError() && !(hasM() && M==999)) {
		GCode::reportFatalError();
	} else {
		pushCommand();
	}
#ifdef DEBUG_COM_ERRORS
    if(hasM() && M == 667)
        return; // omit ok
#endif
#if ACK_WITH_LINENUMBER
    Com::printFLN(Com::tOkSpace, actLineNumber);
#else
    Com::printFLN(Com::tOk);
#endif
    wasLastCommandReceivedAsBinary = sendAsBinary;
	keepAlive(NotBusy);
    waitingForResend = -1; // everything is ok.
}

void GCode::pushCommand()
{
#if !ECHO_ON_EXECUTE
    commandsBuffered[bufferWriteIndex].echoCommand();
#endif
    if(++bufferWriteIndex >= GCODE_BUFFER_SIZE) bufferWriteIndex = 0;
    bufferLength++;
}

/**
  Get the next buffered command. Returns 0 if no more commands are buffered. For each
  returned command, the gcode_command_finished() function must be called.
*/
GCode *GCode::peekCurrentCommand()
{
    if(bufferLength == 0) return NULL; // No more data
    return &commandsBuffered[bufferReadIndex];
}

/** \brief Removes the last returned command from cache. */
void GCode::popCurrentCommand()
{
    if(!bufferLength) return; // Should not happen, but safety first
#if ECHO_ON_EXECUTE
    echoCommand();
#endif
    if(++bufferReadIndex == GCODE_BUFFER_SIZE) bufferReadIndex = 0;
    bufferLength--;
}

void GCode::echoCommand()
{
    if(Printer::debugEcho())
    {
        Com::printF(Com::tEcho);
        printCommand();
    }
}

void GCode::debugCommandBuffer()
{
    Com::printF(PSTR("CommandBuffer"));
    for(int i = 0; i < commandsReceivingWritePosition; i++)
        Com::printF(Com::tColon,(int)commandReceiving[i]);
    Com::println();
    Com::printFLN(PSTR("Binary:"), (int)sendAsBinary);
    if(!sendAsBinary)
    {
        Com::print((char*)commandReceiving);
        Com::println();
    }
}

/** \brief Execute commands in progmem stored string. Multiple commands are seperated by \n */
void GCode::executeFString(FSTRINGPARAM(cmd))
{
    char buf[80];
    uint8_t buflen;
    char c;
    GCode code;
    do
    {
        // Wait for a free place in command buffer
        // Scan next command from string
        uint8_t comment = 0;
        buflen = 0;
        do
        {
            c = HAL::readFlashByte(cmd++);
            if(c == 0 || c == '\n') break;
            if(c == ';') comment = 1;
            if(comment) continue;
            buf[buflen++] = c;
        }
        while(buflen < 79);
        if(buflen == 0)   // empty line ignore
            continue;
        buf[buflen] = 0;
        // Send command into command buffer
        if(code.parseAscii((char *)buf,false) && (code.params & 518))   // Success
        {
#ifdef DEBUG_PRINT
            debugWaitLoop = 7;
#endif

            Commands::executeGCode(&code);
            Printer::defaultLoopActions();
        }
    }
    while(c);
}

/** \brief Read from serial console or sdcard.

This function is the main function to read the commands from serial console or from sdcard.
It must be called frequently to empty the incoming buffer.
*/
void GCode::readFromSerial()
{
    if(bufferLength >= GCODE_BUFFER_SIZE || (waitUntilAllCommandsAreParsed && bufferLength)) {
		keepAlive(Processing);
		return; // all buffers full
	}
    waitUntilAllCommandsAreParsed = false;
    millis_t time = HAL::timeInMilliseconds();
    if(!HAL::serialByteAvailable())
    {
        if((waitingForResend >= 0 || commandsReceivingWritePosition > 0) && time - timeOfLastDataPacket > 200)
        {
            // Com::printF(PSTR("WFR:"),waitingForResend);Com::printF(PSTR(" CRWP:"),commandsReceivingWritePosition);commandReceiving[commandsReceivingWritePosition] = 0;Com::printFLN(PSTR(" GOT:"),(char*)commandReceiving);
            requestResend(); // Something is wrong, a started line was not continued in the last second
            timeOfLastDataPacket = time;
        }
#ifdef WAITING_IDENTIFIER
        else if(bufferLength == 0 && time - timeOfLastDataPacket > 1000)   // Don't do it if buffer is not empty. It may be a slow executing command.
        {
            Com::printFLN(Com::tWait); // Unblock communication in case the last ok was not received correct.
            timeOfLastDataPacket = time;
        }
#endif
    }
    while(HAL::serialByteAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE)    // consume data until no data or buffer full
    {
        timeOfLastDataPacket = time; //HAL::timeInMilliseconds();
        commandReceiving[commandsReceivingWritePosition++] = HAL::serialReadByte();
        // first lets detect, if we got an old type ascii command
        if(commandsReceivingWritePosition == 1)
        {
            if(waitingForResend >= 0 && wasLastCommandReceivedAsBinary)
            {
                if(!commandReceiving[0])
                    waitingForResend--;   // Skip 30 zeros to get in sync
                else
                    waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if(!commandReceiving[0]) // Ignore zeros
            {
                commandsReceivingWritePosition = 0;
                continue;
            }
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        }
        if(sendAsBinary)
        {
            if(commandsReceivingWritePosition < 2 ) continue;
            if(commandsReceivingWritePosition == 5 || commandsReceivingWritePosition == 4)
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if(commandsReceivingWritePosition == binaryCommandSize)
            {
                GCode *act = &commandsBuffered[bufferWriteIndex];
                if(act->parseBinary(commandReceiving, true))   // Success
                    act->checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            }
        }
        else     // Ascii command
        {
            char ch = commandReceiving[commandsReceivingWritePosition - 1];
            if(ch == 0 || ch == '\n' || ch == '\r' || (!commentDetected && ch == ':'))  // complete line read
            {
                commandReceiving[commandsReceivingWritePosition - 1] = 0;
                //Com::printF(PSTR("Parse ascii:"));Com::print((char*)commandReceiving);Com::println();
                commentDetected = false;
                if(commandsReceivingWritePosition == 1)   // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode *act = &commandsBuffered[bufferWriteIndex];
                if(act->parseAscii((char *)commandReceiving, true))   // Success
                    act->checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            }
            else
            {
                if(ch == ';') commentDetected = true; // ignore new data until line end
                if(commentDetected) commandsReceivingWritePosition--;
            }
        }
        if(commandsReceivingWritePosition == MAX_CMD_SIZE)
        {
            requestResend();
            return;
        }
    }
#if SDSUPPORT
    if(sd.sdmode == 0 || sd.sdmode >= 100 || commandsReceivingWritePosition != 0)   // not reading or incoming serial command
        return;
    while( sd.filesize > sd.sdpos && commandsReceivingWritePosition < MAX_CMD_SIZE)    // consume data until no data or buffer full
    {
        timeOfLastDataPacket = HAL::timeInMilliseconds();
        int n = sd.file.read();
        if(n == -1)
        {
            Com::printFLN(Com::tSDReadError);
            UI_ERROR("SD Read Error");

            // Second try in case of recoverable errors
            sd.file.seekSet(sd.sdpos);
            n = sd.file.read();
            if(n == -1)
            {
                Com::printErrorFLN(PSTR("SD error did not recover!"));
                sd.sdmode = 0;
                break;
            }
            UI_ERROR("SD error fixed");
        }
        sd.sdpos++; // = file.curPosition();
        commandReceiving[commandsReceivingWritePosition++] = (uint8_t)n;

        // first lets detect, if we got an old type ascii command
        if(commandsReceivingWritePosition == 1)
        {
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        }
        if(sendAsBinary)
        {
            if(commandsReceivingWritePosition < 2 ) continue;
            if(commandsReceivingWritePosition == 4 || commandsReceivingWritePosition == 5)
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if(commandsReceivingWritePosition == binaryCommandSize)
            {
                GCode *act = &commandsBuffered[bufferWriteIndex];
                if(act->parseBinary(commandReceiving, false))   // Success, silently ignore illegal commands
                    pushCommand();
                commandsReceivingWritePosition = 0;
                if(sd.sdmode == 2)
                    sd.sdmode = 0;
                return;
            }
        }
        else
        {
            char ch = commandReceiving[commandsReceivingWritePosition-1];
            bool returnChar = ch == '\n' || ch == '\r';
            if(returnChar || sd.filesize == sd.sdpos || (!commentDetected && ch == ':') || commandsReceivingWritePosition >= (MAX_CMD_SIZE - 1) )  // complete line read
            {
                if(returnChar || ch == ':')
                    commandReceiving[commandsReceivingWritePosition - 1] = 0;
                else
                    commandReceiving[commandsReceivingWritePosition] = 0;
                commentDetected = false;
                if(commandsReceivingWritePosition == 1)   // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode *act = &commandsBuffered[bufferWriteIndex];
                if(act->parseAscii((char *)commandReceiving, false))   // Success
                    pushCommand();
                commandsReceivingWritePosition = 0;
                if(sd.sdmode == 2)
                    sd.sdmode = 0;
                return;
            }
            else
            {
                if(ch == ';') commentDetected = true; // ignore new data until line end
                if(commentDetected) commandsReceivingWritePosition--;
            }
        }
    }
    sd.sdmode = 0;
    Com::printFLN(Com::tDonePrinting);
    commandsReceivingWritePosition = 0;
    commentDetected = false;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, false);
#endif
}

/**
  Converts a binary uint8_tfield containing one GCode line into a GCode structure.
  Returns true if checksum was correct.
*/
bool GCode::parseBinary(uint8_t *buffer,bool fromSerial)
{
    internalCommand = !fromSerial;
    unsigned int sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    // first do fletcher-16 checksum tests see
    // http://en.wikipedia.org/wiki/Fletcher's_checksum
    uint8_t *p = buffer;
    uint8_t len = binaryCommandSize - 2;
    while (len)
    {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do
        {
            sum1 += *p++;
            if(sum1 >= 255) sum1 -= 255;
            sum2 += sum1;
            if(sum2 >= 255) sum2 -= 255;
        }
        while (--tlen);
    }
    sum1 -= *p++;
    sum2 -= *p;
    if(sum1 | sum2)
    {
        if(Printer::debugErrors())
        {
            Com::printErrorFLN(Com::tWrongChecksum);
        }
        return false;
    }
    p = buffer;
    params = *(uint16_t *)p;
    p += 2;
    uint8_t textlen = 16;
    if(isV2())
    {
        params2 = *(uint16_t *)p;
        p += 2;
        if(hasString())
            textlen = *p++;
    }
    else params2 = 0;
    if(params & 1)
    {
        actLineNumber = N = *(uint16_t *)p;
        p += 2;
    }
    if(isV2())   // Read G,M as 16 bit value
    {
        if(hasM())
        {
            M = *(uint16_t *)p;
            p += 2;
        }
        if(hasG())
        {
            G = *(uint16_t *)p;
            p += 2;
        }
    }
    else
    {
        if(hasM())
        {
            M = *p++;
        }
        if(hasG())
        {
            G = *p++;
        }
    }
    //if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
    if(hasX())
    {
        X = *(float *)p;
        p += 4;
    }
    if(hasY())
    {
        Y = *(float *)p;
        p += 4;
    }
    if(hasZ())
    {
        Z = *(float *)p;
        p += 4;
    }
    if(hasE())
    {
        E = *(float *)p;
        p += 4;
    }
    if(hasF())
    {
        F = *(float *)p;
        p += 4;
    }
    if(hasT())
    {
        T = *p++;
    }
    if(hasS())
    {
        S = *(int32_t*)p;
        p += 4;
    }
    if(hasP())
    {
        P = *(int32_t*)p;
        p += 4;
    }
    if(hasI())
    {
        I = *(float *)p;
        p += 4;
    }
    if(hasJ())
    {
        J = *(float *)p;
        p += 4;
    }
    if(hasR())
    {
        R = *(float *)p;
        p += 4;
    }
    if(hasD())
    {
        D = *(float *)p;
        p += 4;
    }
    if(hasC())
    {
        C = *(float *)p;
        p += 4;
    }
    if(hasH())
    {
        H = *(float *)p;
        p += 4;
    }
    if(hasA())
    {
        A = *(float *)p;
        p += 4;
    }
    if(hasB())
    {
        B = *(float *)p;
        p += 4;
    }
    if(hasK())
    {
        K = *(float *)p;
        p += 4;
    }
    if(hasL())
    {
        L = *(float *)p;
        p += 4;
    }
    if(hasO())
    {
        O = *(float *)p;
        p += 4;
    }
    if(hasString())   // set text pointer to string
    {
        text = (char*)p;
        text[textlen] = 0; // Terminate string overwriting checksum
        waitUntilAllCommandsAreParsed = true; // Don't destroy string until executed
    }
    formatErrors = 0;
    return true;
}

/**
  Converts a ascii GCode line into a GCode structure.
*/
bool GCode::parseAscii(char *line,bool fromSerial)
{
    char *pos = line;
    params = 0;
    params2 = 0;
    internalCommand = !fromSerial;
	bool hasChecksum = false;
    char c;
    while ( (c = *(pos++)) )
    {
        if(c == '(' || c == '%') break; // alternative comment or program block
        switch(c)
        {
        case 'N':
        case 'n':
        {
            actLineNumber = parseLongValue(pos);
            params |=1;
            N = actLineNumber;
            break;
        }
        case 'G':
        case 'g':
        {
            G = parseLongValue(pos) & 0xffff;
            params |= 4;
            if(G > 255) params |= 4096;
            break;
        }
        case 'M':
        case 'm':
        {
            M = parseLongValue(pos) & 0xffff;
            params |= 2;
            if(M > 255) params |= 4096;
            // handle non standard text arguments that some M codes have
            if (M == 20 || M == 23 || M == 28 || M == 29 || M == 30 || M == 32 || M == 36 || M == 117)
            {
                // after M command we got a filename or text
                char digit;
                while( (digit = *pos) )
                {
                    if (digit < '0' || digit > '9') break;
                    pos++;
                }
                while( (digit = *pos) )
                {
                    if (digit != ' ') break;
                    pos++;
                    // skip leading whitespaces (may be no white space)
                }
                text = pos;
                while (*pos)
                {
                    if((M != 117 && M != 20 && *pos==' ') || *pos=='*') break;
                    pos++; // find a space as file name end
                }
                *pos = 0; // truncate filename by erasing space with nul, also skips checksum
                waitUntilAllCommandsAreParsed = true; // don't risk string be deleted
                params |= 32768;
            }
            break;
        }
        case 'X':
        case 'x':
        {
            X = parseFloatValue(pos);
            params |= 8;
            break;
        }
        case 'Y':
        case 'y':
        {
            Y = parseFloatValue(pos);
            params |= 16;
            break;
        }
        case 'Z':
        case 'z':
        {
            Z = parseFloatValue(pos);
            params |= 32;
            break;
        }
        case 'E':
        case 'e':
        {
            E = parseFloatValue(pos);
            params |= 64;
            break;
        }
        case 'F':
        case 'f':
        {
            F = parseFloatValue(pos);
            params |= 256;
            break;
        }
        case 'T':
        case 't':
        {
            T = parseLongValue(pos) & 0xff;
            params |= 512;
            break;
        }
        case 'S':
        case 's':
        {
            S = parseLongValue(pos);
            params |= 1024;
            break;
        }
        case 'P':
        case 'p':
        {
            P = parseLongValue(pos);
            params |= 2048;
            break;
        }
        case 'I':
        case 'i':
        {
            I = parseFloatValue(pos);
            params2 |= 1;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'J':
        case 'j':
        {
            J = parseFloatValue(pos);
            params2 |= 2;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'R':
        case 'r':
        {
            R = parseFloatValue(pos);
            params2 |= 4;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'D':
        case 'd':
        {
            D = parseFloatValue(pos);
            params2 |= 8;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'C':
        case 'c':
        {
	        C = parseFloatValue(pos);
	        params2 |= 16;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'H':
        case 'h':
        {
	        H = parseFloatValue(pos);
	        params2 |= 32;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'A':
        case 'a':
        {
	        A = parseFloatValue(pos);
	        params2 |= 64;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'B':
        case 'b':
        {
	        B = parseFloatValue(pos);
	        params2 |= 128;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'K':
        case 'k':
        {
	        K = parseFloatValue(pos);
	        params2 |= 256;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'L':
        case 'l':
        {
	        L = parseFloatValue(pos);
	        params2 |= 512;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case 'O':
        case 'o':
        {
	        O = parseFloatValue(pos);
	        params2 |= 1024;
	        params |= 4096; // Needs V2 for saving
	        break;
        }
        case '*' : //checksum
        {
            uint8_t checksum_given = parseLongValue(pos);
            uint8_t checksum = 0;
            while(line != (pos - 1)) checksum ^= *line++;
#if FEATURE_CHECKSUM_FORCED
            Printer::flag0 |= PRINTER_FLAG0_FORCE_CHECKSUM;
#endif
            if(checksum != checksum_given)
            {
                if(Printer::debugErrors())
                {
                    Com::printErrorFLN(Com::tWrongChecksum);
                }
                return false; // mismatch
            }
			hasChecksum = true;
            break;
        }
        default:
            break;
        }// end switch
    }// end while
	if(wasLastCommandReceivedAsBinary && !hasChecksum && fromSerial) {
		Com::printErrorFLN("Checksum required when switching back to ASCII protocol.");
		return false;
	}
    if(hasFormatError() || (params & 518) == 0)   // Must contain G, M or T command and parameter need to have variables!
    {
        formatErrors++;
        if(Printer::debugErrors())
            Com::printErrorFLN(Com::tFormatError);
        if(formatErrors < 3) return false;
    }
    else formatErrors = 0;
    return true;
}

/** \brief Print command on serial console */
void GCode::printCommand()
{
    if(hasN()) {
        Com::print('N');
        Com::print((int32_t)N);
        Com::print(' ');
    }
    if(hasM())
    {
        Com::print('M');
        Com::print((int)M);
        Com::print(' ');
    }
    if(hasG())
    {
        Com::print('G');
        Com::print((int)G);
        Com::print(' ');
    }
    if(hasT())
    {
        Com::print('T');
        Com::print((int)T);
        Com::print(' ');
    }
    if(hasX())
    {
        Com::printF(Com::tX,X);
    }
    if(hasY())
    {
        Com::printF(Com::tY,Y);
    }
    if(hasZ())
    {
        Com::printF(Com::tZ,Z);
    }
    if(hasE())
    {
        Com::printF(Com::tE,E,4);
    }
    if(hasF())
    {
        Com::printF(Com::tF,F);
    }
    if(hasS())
    {
        Com::printF(Com::tS,S);
    }
    if(hasP())
    {
        Com::printF(Com::tP,P);
    }
    if(hasI())
    {
        Com::printF(Com::tI,I);
    }
    if(hasJ())
    {
        Com::printF(Com::tJ,J);
    }
    if(hasR())
    {
        Com::printF(Com::tR,R);
    }
    if(hasString())
    {
        Com::print(text);
    }
    Com::println();
}

void GCode::fatalError(FSTRINGPARAM(message)) {
	fatalErrorMsg = message;
#if SDSUPPORT
	if(sd.sdmode != 0)	{ // stop sd print to prevent damage
		sd.stopPrint();
	}
#endif	
	if(Printer::currentPosition[Z_AXIS] < Printer::zMin + Printer::zLength - 15)
		PrintLine::moveRelativeDistanceInStepsReal(0,0,10*Printer::axisStepsPerMM[Z_AXIS],0,Printer::homingFeedrate[Z_AXIS],true,true);
	EVENT_FATAL_ERROR_OCCURED		
	Commands::waitUntilEndOfAllMoves();
	Printer::kill(true);		
	reportFatalError();
}

void GCode::reportFatalError() {
	Com::printF(Com::tFatal);
	Com::printF(fatalErrorMsg);
	Com::printFLN(PSTR(" Printer stopped and heaters disabled due to this error. Fix error and restart with M999."));
	UI_ERROR_P(fatalErrorMsg)
}

void GCode::resetFatalError() {
	TemperatureController::resetAllErrorStates();
	fatalErrorMsg = NULL;
	UI_ERROR("");
	EVENT_CONTINUE_FROM_FATAL_ERROR
	Com::printFLN(PSTR("info:Continue from fatal state"));
}

#if JSON_OUTPUT

// --------------------------------------------------------------- //
// Code that gets gcode information is adapted from RepRapFirmware //
// Originally licensed under GPL                                   //
// Authors: reprappro, dc42, dcnewman, others                      //
// Source: https://github.com/dcnewman/RepRapFirmware              //
// Copy date: 15 Nov 2015                                          //
// --------------------------------------------------------------- //

void GCodeFileInfo::init(SdBaseFile &file) {
	this->fileSize = file.fileSize();
	this->filamentNeeded = 0.0;
	this->objectHeight = 0.0;
	this->layerHeight = 0.0;
	if (!file.isOpen()) return;
	bool genByFound = false, layerHeightFound = false, filamentNeedFound = false;
	#if CPU_ARCH==ARCH_AVR
	#define GCI_BUF_SIZE 120
	#else
	#define GCI_BUF_SIZE 1024
	#endif
	// READ 4KB FROM THE BEGINNING
	char buf[GCI_BUF_SIZE];
	for (int i = 0; i < 4096; i += GCI_BUF_SIZE-50) {
		if(!file.seekSet(i)) break;
		file.read(buf, GCI_BUF_SIZE);
		if (!genByFound && findGeneratedBy(buf, this->generatedBy)) genByFound = true;
		if (!layerHeightFound && findLayerHeight(buf, this->layerHeight)) layerHeightFound = true;
		if (!filamentNeedFound && findFilamentNeed(buf, this->filamentNeeded)) filamentNeedFound = true;
		if(genByFound && layerHeightFound && filamentNeedFound) goto get_objectHeight;
	}

	// READ 4KB FROM END
	for (int i = 0; i < 4096; i += GCI_BUF_SIZE-50) {
		if(!file.seekEnd(-4096 + i)) break;
		file.read(buf, GCI_BUF_SIZE);
		if (!genByFound && findGeneratedBy(buf, this->generatedBy)) genByFound = true;
		if (!layerHeightFound && findLayerHeight(buf, this->layerHeight)) layerHeightFound = true;
		if (!filamentNeedFound && findFilamentNeed(buf, this->filamentNeeded)) filamentNeedFound = true;
		if(genByFound && layerHeightFound && filamentNeedFound) goto get_objectHeight;
	}
	
	get_objectHeight:
	// MOVE FROM END UP IN 1KB BLOCKS UP TO 30KB
	for (int i = GCI_BUF_SIZE; i < 30000; i += GCI_BUF_SIZE-50) {
		if(!file.seekEnd(-i)) break;
		file.read(buf, GCI_BUF_SIZE);
		if (findTotalHeight(buf, this->objectHeight)) break;
	}
	file.seekSet(0);
}

bool GCodeFileInfo::findGeneratedBy(char *buf, char *genBy) {
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
                if (i > GENBY_SIZE - 3) break;
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

bool GCodeFileInfo::findLayerHeight(char *buf, float &layerHeight) {
    // SLIC3R
	layerHeight = 0;
    const char* layerHeightSlic3r = PSTR("; layer_height ");
    char *pos = strstr_P(buf, layerHeightSlic3r);
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

bool GCodeFileInfo::findFilamentNeed(char *buf, float &filament) {
    const char* filamentUsedStr = PSTR("filament used");
    const char* pos = strstr_P(buf, filamentUsedStr);
	filament = 0;
    if (pos != NULL) {
        pos += strlen_P(filamentUsedStr);
        while (*pos == ' ' || *pos == 't' || *pos == '=' || *pos == ':') {
            ++pos;    // this allows for " = " from default slic3r comment and ": " from default Cura comment
        }
        if (isDigit(*pos)) {
            char *q;
            filament += strtod(pos, &q);
            if (*q == 'm' && *(q + 1) != 'm') {
                filament *= 1000.0;        // Cura outputs filament used in metres not mm
            }
        }
        return true;
    }
    return false;
}

bool GCodeFileInfo::findTotalHeight(char *buf, float &height) {
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
                    if (c == '\n' || c == '\r') break;
                    if (c == ';') {
                        // It is in a comment, so give up on this one
                        inComment = true;
                        break;
                    }
                }
                if (inComment) continue;

                // Find 'Z' position and grab that value
                zPos = 0;
                for (int j = i + 3; j < len - 2; j++) {
                    char c = buf[j];
                    if (c < ' ') {
                        // Skip all whitespaces...
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
                    } else if (c == ';') break;
                    else if (c == 'Z') zPos = j;
                }
            }
        }
    }
    return false;
}
#endif // JSON_OUTPUT
