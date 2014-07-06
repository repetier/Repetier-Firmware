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
#ifndef _GCODE_H
#define _GCODE_H

#define MAX_CMD_SIZE 96
class SDCard;
class GCode   // 52 uint8_ts per command needed
{
    unsigned int params;
    unsigned int params2;
public:
    unsigned int N; // Line number
    unsigned int M;
    unsigned int G;
    float X;
    float Y;
    float Z;
    float E;
    float F;
    uint8_t T;
    long S;
    long P;
    float I;
    float J;
    float R;
    char *text; //text[17];
    inline bool hasM()
    {
        return ((params & 2)!=0);
    }
    inline bool hasN()
    {
        return ((params & 1)!=0);
    }
    inline bool hasG()
    {
        return ((params & 4)!=0);
    }
    inline bool hasX()
    {
        return ((params & 8)!=0);
    }
    inline bool hasY()
    {
        return ((params & 16)!=0);
    }
    inline bool hasZ()
    {
        return ((params & 32)!=0);
    }
    inline bool hasNoXYZ()
    {
        return ((params & 56)==0);
    }
    inline bool hasE()
    {
        return ((params & 64)!=0);
    }
    inline bool hasF()
    {
        return ((params & 256)!=0);
    }
    inline bool hasT()
    {
        return ((params & 512)!=0);
    }
    inline bool hasS()
    {
        return ((params & 1024)!=0);
    }
    inline bool hasP()
    {
        return ((params & 2048)!=0);
    }
    inline bool isV2()
    {
        return ((params & 4096)!=0);
    }
    inline bool hasString()
    {
        return ((params & 32768)!=0);
    }
    inline bool hasI()
    {
        return ((params2 & 1)!=0);
    }
    inline bool hasJ()
    {
        return ((params2 & 2)!=0);
    }
    inline bool hasR()
    {
        return ((params2 & 4)!=0);
    }
    inline long getS(long def)
    {
        return (hasS() ? S : def);
    }
    inline long getP(long def)
    {
        return (hasP() ? P : def);
    }
    inline void setFormatError() {
        params2 |= 32768;
    }
    inline bool hasFormatError() {
        return ((params2 & 32768)!=0);
    }
    void printCommand();
    bool parseBinary(uint8_t *buffer,bool fromSerial);
    bool parseAscii(char *line,bool fromSerial);
    void popCurrentCommand();
    void echoCommand();
    /** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
    static GCode *peekCurrentCommand();
    /** Frees the cache used by the last command fetched. */
    static void readFromSerial();
    static void pushCommand();
    static void executeFString(FSTRINGPARAM(cmd));
    static uint8_t computeBinarySize(char *ptr);

    friend class SDCard;
    friend class UIDisplay;
private:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s)
    {
        char *endPtr;
        float f = (strtod(s, &endPtr));
        if(s == endPtr) setFormatError();
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) setFormatError();
        return l;
    }

    static GCode commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
    static uint8_t bufferReadIndex; ///< Read position in gcode_buffer.
    static uint8_t bufferWriteIndex; ///< Write position in gcode_buffer.
    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
    static uint8_t sendAsBinary; ///< Flags the command as binary input.
    static uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
    static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
    static bool waitUntilAllCommandsAreParsed; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
    static uint32_t lastLineNumber; ///< Last line number received.
    static uint32_t actLineNumber; ///< Line number of current command.
    static int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.
    static volatile uint8_t bufferLength; ///< Number of commands stored in gcode_buffer
    static millis_t timeOfLastDataPacket; ///< Time, when we got the last data packet. Used to detect missing uint8_ts.
    static uint8_t formatErrors; ///< Number of sequential format errors
};


#endif

