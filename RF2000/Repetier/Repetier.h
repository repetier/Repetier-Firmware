/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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


#ifndef REPETIER_H
#define REPETIER_H

#include "Constants.h"
#include "Configuration.h"


#if MOTHERBOARD == DEVICE_TYPE_RF1000
#include "RF1000.h"
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if MOTHERBOARD == DEVICE_TYPE_RF2000
#include "RF2000.h"
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000


#include "pins.h"
#include "HAL.h"
#include "gcode.h"
#include "RF.h"
#include "ui.h"
#include "Communication.h"

#if SDSUPPORT
#include "SdFat.h"
#endif // SDSUPPORT

#undef min
#undef max


class RMath {
public:
    static inline float min(float a,float b)
	{
        if(a<b) return a;
        return b;
    } // min

    static inline float max(float a,float b)
	{
        if(a<b) return b;
        return a;
    } // max

    static inline long min(long a,long b)
	{
        if(a<b) return a;
        return b;
    } // min

    static inline long max(long a,long b)
	{
        if(a<b) return b;
        return a;
    } // max

    static inline int min(int a,int b)
	{
        if(a<b) return a;
        return b;
    } // min

    static inline int max(int a,int b)
	{
        if(a<b) return b;
        return a;
    } // max

    static inline unsigned int min(unsigned int a,unsigned int b)
	{
        if(a<b) return a;
        return b;
    } // min

    static inline unsigned int max(unsigned int a,unsigned int b)
	{
        if(a<b) return b;
        return a;
    } // max

    static inline long sqr(long a)
	{
		return a*a;
	} // sqr

    static inline float sqr(float a)
	{
		return a*a;
	} // sqr

};

extern const uint8		osAnalogInputChannels[] PROGMEM;
extern uint8			osAnalogInputCounter[ANALOG_INPUTS];
extern uint				osAnalogInputBuildup[ANALOG_INPUTS];
extern uint8			osAnalogInputPos; // Current sampling position
extern volatile uint	osAnalogInputValues[ANALOG_INPUTS];
extern uint8_t			pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
extern int				maxadv;
#endif // ENABLE_QUADRATIC_ADVANCE

extern int				maxadv2;
extern float			maxadvspeed;
#endif // USE_ADVANCE


#include "Extruder.h"

void manage_inactivity(uint8_t debug);

extern void finishNextSegment();
extern void linear_move(long steps_remaining[]);

#ifndef FEATURE_DITTO_PRINTING
#define FEATURE_DITTO_PRINTING false
#endif // FEATURE_DITTO_PRINTING

#if FEATURE_DITTO_PRINTING && NUM_EXTRUDER!=2
#error Ditto printing requires exactly 2 extruder.
#endif // FEATURE_DITTO_PRINTING && NUM_EXTRUDER!=2


extern millis_t previousMillisCmd;
extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;

extern void setupTimerInterrupt();
extern void motorCurrentControlInit();

#include "Printer.h"
#include "motion.h"


extern long baudrate;
#if OS_ANALOG_INPUTS>0
// Get last result for pin x
extern volatile uint osAnalogInputValues[OS_ANALOG_INPUTS];
#endif // OS_ANALOG_INPUTS>0

#include "HAL.h"


extern unsigned int			counterPeriodical;
extern volatile uint8_t		executePeriodical;
extern uint8_t				counter250ms;


extern void writeMonitor();

#if SDSUPPORT
extern char					tempLongFilename[LONG_FILENAME_LENGTH+1];
extern char					fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];

#define SHORT_FILENAME_LENGTH 14
#include "SdFat.h"

enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};

class SDCard
{
public:
	SdFat		fat;
	SdFile		file;
	uint32_t	filesize;
	uint32_t	sdpos;
	char*		shortname;	// Pointer to start of filename itself
	char*		pathend;	// File to char where pathname in fullname ends
	bool		sdmode;		// true if we are printing from sd card
	bool		sdactive;
	bool		savetosd;
	SdBaseFile	parentFound;

	SDCard();
	void initsd();
	void writeCommand(GCode *code);
	bool selectFile(char *filename,bool silent=false);
	void mount();
	void unmount();
	void startPrint();
	void abortPrint();

	inline void setIndex(uint32_t  newpos)
	{
		if(!sdactive) return;
		sdpos = newpos;
		file.seekSet(sdpos);

	} // setIndex

	void printStatus();
	void ls();
	void startWrite(char *filename);
	void deleteFile(char *filename);
	void finishWrite();
	char *createFilename(char *buffer,const dir_t &p);
	void makeDirectory(char *filename);
	bool showFilename(const uint8_t *name);
	void automount();

#ifdef GLENN_DEBUG
	void writeToFile();
#endif // GLENN_DEBUG

private:
	uint8_t lsRecursive(SdBaseFile *parent,uint8_t level,char *findFilename);

};

extern SDCard sd;
#endif // SDSUPPORT

extern volatile int			waitRelax; // Delay filament relax at the end of print, could be a simple timeout

extern void updateStepsParameter(PrintLine *p);

#ifdef DEBUG_PRINT
extern int					debugWaitLoop;
#endif // DEBUG_PRINT

#define STR(s)		#s
#define XSTR(s)		STR(s)

#include "Commands.h"
#include "Eeprom.h"

#endif // REPETIER_H
