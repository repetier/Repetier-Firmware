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
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))

enum FirmwareState {NotBusy=0,Processing,Paused,WaitHeater,DoorOpen};

class SDCard;
class Commands;





class SerialGCodeSource: public GCodeSource {
    Stream *stream;
public:    
    SerialGCodeSource(Stream *p);
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();
};
//#pragma message "Sd support: " XSTR(SDSUPPORT)  
#if SDSUPPORT
class SDCardGCodeSource: public GCodeSource {
    public:
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();
};
#endif

class FlashGCodeSource: public GCodeSource {
    public:
    FSTRINGPARAM(pointer);
    volatile bool finished;
    int actionOnFinish;
    
    FlashGCodeSource();
    virtual bool isOpen();
    virtual bool supportsWrite(); ///< true if write is a non dummy function
    virtual bool closeOnError(); // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable(); // would read return a new byte?
    virtual int readByte();
    virtual void writeByte(uint8_t byte);
    virtual void close();
    
    /** Execute the commands at the given memory. If already an other string is
    running, the command will wait until that command finishes. If wait is true it
    will also wait for given command to be enqueued completely. */
    void executeCommands(FSTRINGPARAM(data),bool waitFinish,int action);
};

#if NEW_COMMUNICATION
extern FlashGCodeSource flashSource;
extern SerialGCodeSource serial0Source;
#if BLUETOOTH_SERIAL > 0
extern SerialGCodeSource serial1Source;
#endif
#if SDSUPPORT
extern SDCardGCodeSource sdSource;
#endif
#endif

class GCode   // 52 uint8_ts per command needed
{
    uint16_t params;
    uint16_t params2;
public:
    uint16_t N; ///< Line number reduced to 16 bit
    uint16_t M; ///< G-code M value if set
    uint16_t G; ///< G-code G value if set
    float X; ///< G-code X value if set
    float Y; ///< G-code Y value if set
    float Z; ///< G-code Z value if set
    float E; ///< G-code E value if set
    float F; ///< G-code F value if set
    int32_t S; ///< G-code S value if set
    int32_t P; ///< G-code P value if set
    float I; ///< G-code I value if set
    float J; ///< G-code J value if set
    float R; ///< G-code R value if set
    float D; ///< G-code D value if set
    float C; ///< G-code C value if set
    float H; ///< G-code H value if set
    float A; ///< G-code A value if set
    float B; ///< G-code B value if set
    float K; ///< G-code K value if set
    float L; ///< G-code L value if set
    float O; ///< G-code O value if set

    char *text; ///< Text message of g-code if present.
    //moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt
    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;
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
	inline void unsetX() {
		params &= ~8;
	}
    inline bool hasY()
    {
        return ((params & 16)!=0);
    }
	inline void unsetY() {
		params &= ~16;
	}
    inline bool hasZ()
    {
        return ((params & 32)!=0);
    }
	inline void unsetZ() {
		params &= ~32;
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
    inline bool hasD()
    {
        return ((params2 & 8)!=0);
    }
    inline bool hasC()
    {
        return ((params2 & 16)!=0);
    }
    inline bool hasH()
    {
        return ((params2 & 32)!=0);
    }
    inline bool hasA()
    {
        return ((params2 & 64)!=0);
    }
    inline bool hasB()
    {
        return ((params2 & 128)!=0);
    }
    inline bool hasK()
    {
        return ((params2 & 256)!=0);
    }
    inline bool hasL()
    {
        return ((params2 & 512)!=0);
    }
    inline bool hasO()
    {
        return ((params2 & 1024)!=0);
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
	static void fatalError(FSTRINGPARAM(message));
	static void reportFatalError();
	static void resetFatalError();
	inline static bool hasFatalError() {
		return fatalErrorMsg != NULL;
	}
	static void keepAlive(enum FirmwareState state);
	static uint32_t keepAliveInterval;
    friend class SDCard;
    friend class UIDisplay;
	static FSTRINGPARAM(fatalErrorMsg);
    friend class GCodeSource;    
protected:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        float f = (strtod(s, &endPtr));
        if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
        return l;
    }

    static GCode commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
    static uint8_t bufferReadIndex; ///< Read position in gcode_buffer.
    static uint8_t bufferWriteIndex; ///< Write position in gcode_buffer.
    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
    static uint8_t sendAsBinary; ///< Flags the command as binary input.
    static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
    static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
    static bool waitUntilAllCommandsAreParsed; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
    static uint32_t actLineNumber; ///< Line number of current command.
    static volatile uint8_t bufferLength; ///< Number of commands stored in gcode_buffer
    static uint8_t formatErrors; ///< Number of sequential format errors
	static millis_t lastBusySignal; ///< When was the last busy signal
#if NEW_COMMUNICATION == 0
    static int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.
    static uint32_t lastLineNumber; ///< Last line number received.
    static millis_t timeOfLastDataPacket; ///< Time, when we got the last data packet. Used to detect missing uint8_ts.
    static uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
#else
public:
    GCodeSource *source;    
#endif    
};


#if JSON_OUTPUT
#include "SdFat.h"
// Struct to hold Gcode file information 32 bytes
#define GENBY_SIZE 16
class GCodeFileInfo {
public:
    void init(SdBaseFile &file);

    unsigned long fileSize;
    float objectHeight;
    float layerHeight;
    float filamentNeeded;
    char generatedBy[GENBY_SIZE];

    bool findGeneratedBy(char *buf, char *genBy);
    bool findLayerHeight(char *buf, float &layerHeight);
    bool findFilamentNeed(char *buf, float &filament);
    bool findTotalHeight(char *buf, float &objectHeight);
};
#endif

#endif

