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


#include "Repetier.h"
#include "ui.h"

#if SDSUPPORT

char tempLongFilename[LONG_FILENAME_LENGTH+1];
char fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];

SDCard sd;

SDCard::SDCard()
{
    sdmode = false;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);

    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
#endif // SDPOWER > -1

#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    SET_INPUT(SDCARDDETECT);
    WRITE(SDCARDDETECT,HIGH);
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

} /// SDCard


void SDCard::automount()
{
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED)
    {
        if(sdactive)   // Card removed
        {
			if( Printer::debugInfo() )
			{
	            Com::printFLN(PSTR("SD card removed"));
			}
#if UI_DISPLAY_TYPE!=0
            uid.executeAction(UI_ACTION_TOP_MENU);
#endif // UI_DISPLAY_TYPE!=0

            unmount();
            UI_STATUS(UI_TEXT_SD_REMOVED);
        }
    }
    else
    {
        if(!sdactive)
        {
            UI_STATUS(UI_TEXT_SD_INSERTED);
			if( Printer::debugInfo() )
			{
	            Com::printFLN(PSTR("SD card inserted"));
			}
            Printer::setMenuMode(MENU_MODE_SD_MOUNTED,true);
            initsd();
#if UI_DISPLAY_TYPE!=0
            if(sdactive) {
                Printer::setAutomount(true);
                uid.executeAction(UI_ACTION_SD_PRINT+UI_ACTION_TOPMENU);
            }
#endif // UI_DISPLAY_TYPE!=0
        }
    }
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

} // automount


void SDCard::initsd()
{
    sdactive = false;

#if SDSS >- 1
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED)
        return;
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

    if(!fat.begin(SDSS,SPI_FULL_SPEED))
    {
		if( Printer::debugErrors() )
		{
			Com::printFLN(Com::tSDInitFail);
		}
        return;
    }
    sdactive = true;

	if( uid.menuPos[uid.menuLevel] == 9 && uid.menuLevel == 2 )
	{
		// we are within the SD card menu at the moment - after the successful mounting, the menu shall point to the "print"/"mill" item and not to the "delete" item
		uid.menuPos[uid.menuLevel] = 0;
	}

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED,true);

    fat.chdir();
    if(selectFile("init.g",true))
    {
        startPrint();
    }
#endif // SDSS >- 1

} // initsd


void SDCard::mount()
{
    sdmode = false;
    initsd();

} // mount


void SDCard::unmount()
{
    sdmode = false;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);
    Printer::setMenuMode(MENU_MODE_SD_MOUNTED+MENU_MODE_SD_PAUSED+MENU_MODE_SD_PRINTING,false);

#if UI_DISPLAY_TYPE!=0
    uid.cwd[0]='/';
    uid.cwd[1]=0;
    uid.folderLevel=0;
#endif // UI_DISPLAY_TYPE!=0

} // unmount


void SDCard::startPrint()
{
    if(!sdactive) return;
    sdmode = true;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING,true);
    Printer::setMenuMode(MENU_MODE_SD_PAUSED,false);

} // startPrint


void SDCard::abortPrint()
{
    if( !sd.sdactive )
	{
		return;
	}

#if FEATURE_WATCHDOG
	HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	g_uStopTime = millis();

	Printer::setMenuMode(MENU_MODE_SD_PRINTING,false);
    Printer::setMenuMode(MENU_MODE_SD_PAUSED,false);
    
	if( Printer::debugInfo() )
	{
		Com::printFLN(PSTR("SD print aborted."));
	}

	HAL::delayMilliseconds( 250 );

	HAL::forbidInterrupts();

	sdmode	 = false;
	sdpos	 = 0;
	filesize = 0;

	if( Printer::debugInfo() )
	{
	    Com::printFLN(PSTR("G-Code buffer reset"));
	}
	GCode::resetBuffer();

	if( Printer::debugInfo() )
	{
		Com::printFLN(PSTR("Path planner reset"));
	}
	PrintLine::resetPathPlanner();

	if( Printer::debugInfo() )
	{
	    Com::printFLN(PSTR("Line buffer reset"));
	}
	PrintLine::resetLineBuffer();

	HAL::allowInterrupts();

	BEEP_ABORT_PRINTING

#if FEATURE_PAUSE_PRINTING
	if( g_pauseStatus != PAUSE_STATUS_NONE )
	{
		// the printing is paused at the moment
		HAL::forbidInterrupts();

		g_uPauseTime  = 0;
		g_pauseStatus = PAUSE_STATUS_NONE;
		g_pauseMode	  = PAUSE_MODE_NONE;

        g_nContinueSteps[X_AXIS] = 0;
        g_nContinueSteps[Y_AXIS] = 0;
        g_nContinueSteps[Z_AXIS] = 0;
		g_nContinueSteps[E_AXIS] = 0;

		HAL::allowInterrupts();
	}
#endif // FEATURE_PAUSE_PRINTING

	// wait until all moves are done
	while( PrintLine::linesCount )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		HAL::delayMilliseconds( 1 );
		Commands::checkForPeriodicalActions();
	}

	if( Printer::debugInfo() )
	{
	    Com::printFLN(PSTR("Abort complete"));
	}

	HAL::forbidInterrupts();
    Printer::stepperDirection[X_AXIS]	 = 0;
    Printer::stepperDirection[Y_AXIS]	 = 0;
    Printer::stepperDirection[Z_AXIS]	 = 0;
    Extruder::current->stepperDirection = 0;

	// we have to tell the firmware about its real current position
    Printer::queuePositionLastSteps[X_AXIS] = Printer::queuePositionCurrentSteps[X_AXIS];
    Printer::queuePositionLastSteps[Y_AXIS] = Printer::queuePositionCurrentSteps[Y_AXIS];
    Printer::queuePositionLastSteps[Z_AXIS] = Printer::queuePositionCurrentSteps[Z_AXIS];
	Printer::updateCurrentPosition();

	HAL::allowInterrupts();

} // abortPrint


void SDCard::writeCommand(GCode *code)
{
    unsigned int	sum1=0, sum2=0; // for fletcher-16 checksum
    uint8_t			buf[100];
    uint8_t			p=2;
    int				params = 128 | (code->params & ~1);


    file.writeError = false;
    *(int*)buf = params;

    if(code->isV2())   // Read G,M as 16 bit value
    {
        *(int*)&buf[p] = code->params2;
        p+=2;
        if(code->hasString())
            buf[p++] = strlen(code->text);
        if(code->hasM())
        {
            *(int*)&buf[p] = code->M;
            p+=2;
        }
        if(code->hasG())
        {
            *(int*)&buf[p]= code->G;
            p+=2;
        }
    }
    else
    {
        if(code->hasM())
        {
            buf[p++] = (uint8_t)code->M;
        }
        if(code->hasG())
        {
            buf[p++] = (uint8_t)code->G;
        }
    }
    if(code->hasX())
    {
        *(float*)&buf[p] = code->X;
        p+=4;
    }
    if(code->hasY())
    {
        *(float*)&buf[p] = code->Y;
        p+=4;
    }
    if(code->hasZ())
    {
        *(float*)&buf[p] = code->Z;
        p+=4;
    }
    if(code->hasE())
    {
        *(float*)&buf[p] = code->E;
        p+=4;
    }
    if(code->hasF())
    {
        *(float*)&buf[p] = code->F;
        p+=4;
    }
    if(code->hasT())
    {
        buf[p++] = code->T;
    }
    if(code->hasS())
    {
        *(long int*)&buf[p] = code->S;
        p+=4;
    }
    if(code->hasP())
    {
        *(long int*)&buf[p] = code->P;
        p+=4;
    }
    if(code->hasI())
    {
        *(float*)&buf[p] = code->I;
        p+=4;
    }
    if(code->hasJ())
    {
        *(float*)&buf[p] = code->J;
        p+=4;
    }
    if(code->hasString())   // read 16 uint8_t into string
    {
        char *sp = code->text;
        if(code->isV2())
        {
            uint8_t i = strlen(code->text);
            for(; i; i--) buf[p++] = *sp++;
        }
        else
        {
            for(uint8_t i=0; i<16; ++i) buf[p++] = *sp++;
        }
    }
    uint8_t *ptr = buf;
    uint8_t len = p;
    while (len)
    {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do
        {
            sum1 += *ptr++;
            if(sum1>=255) sum1-=255;
            sum2 += sum1;
            if(sum2>=255) sum2-=255;
        }
        while (--tlen);
    }
    buf[p++] = sum1;
    buf[p++] = sum2;
    if(params == 128)
    {
		if( Printer::debugErrors() )
		{
	        Com::printErrorFLN(Com::tAPIDFinished);
		}
    }
    else
        file.write(buf,p);

    if (file.writeError)
    {
		if( Printer::debugErrors() )
		{
	        Com::printFLN(Com::tErrorWritingToFile);
		}
    }
} // writeCommand


char *SDCard::createFilename(char *buffer,const dir_t &p)
{
    char *pos = buffer,*src = (char*)p.name;


    for (uint8_t i = 0; i < 11; i++,src++)
    {
        if (*src == ' ') continue;
        if (i == 8)
            *pos++ = '.';
        *pos++ = *src;
    }
    *pos = 0;
    return pos;

} // createFilename


bool SDCard::showFilename(const uint8_t *name)
{
    if (*name == DIR_NAME_DELETED || *name == '.') return false;
    return true;

} // showFilename


int8_t RFstricmp(const char* s1, const char* s2)
{
    while(*s1 && (tolower(*s1) == tolower(*s2)))
        s1++,s2++;
    return (const uint8_t)tolower(*s1)-(const uint8_t)tolower(*s2);

} // RFstricmp


int8_t RFstrnicmp(const char* s1, const char* s2, size_t n)
{
    while(n--)
    {
        if(tolower(*s1)!=tolower(*s2))
            return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
        s1++;
        s2++;
    }
    return 0;

} // RFstrnicmp


void SDCard::ls()
{
    SdBaseFile file;


	if( Printer::debugInfo() )
	{
	    Com::printFLN(Com::tBeginFileList);
	}
    fat.chdir();

    file.openRoot(fat.vol());
    file.ls(0, 0);

	if( Printer::debugInfo() )
	{
	    Com::printFLN(Com::tEndFileList);
	}

} // ls


bool SDCard::selectFile(char *filename, bool silent)
{
    SdBaseFile	parent;
    char*		oldP = filename;
    boolean		bFound;


    if(!sdactive) return false;
    sdmode = false;

    file.close();

    parent = *fat.vwd();
    if (file.open(&parent, filename, O_READ))
	{
		if ((oldP = strrchr(filename, '/')) != NULL)
			oldP++;
		else
			oldP = filename;

        if(!silent)
        {
			if( Printer::debugInfo() )
			{
				Com::printF(Com::tFileOpened, oldP);
				Com::printFLN(Com::tSpaceSizeColon,file.fileSize());
			}
        }
        sdpos = 0;
        filesize = file.fileSize();

		if( Printer::debugInfo() )
		{
			Com::printFLN(Com::tFileSelected);
		}
        return true;
    }
    else
    {
        if(!silent)
		{
			if( Printer::debugInfo() )
			{
	            Com::printFLN(Com::tFileOpenFailed);
			}
		}
        return false;
    }

} // selectFile


void SDCard::printStatus()
{
	if( Printer::debugInfo() )
	{
		if(sdactive)
		{
			Com::printF(Com::tSDPrintingByte,sdpos);
			Com::printFLN(Com::tSlash,filesize);
		}
		else
		{
			Com::printFLN(Com::tNotSDPrinting);
		}
	}

} // printStatus


void SDCard::startWrite(char *filename)
{
    if(!sdactive) return;
    file.close();
    sdmode = false;
    fat.chdir();

    if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
    {
		if( Printer::debugErrors() )
		{
	        Com::printFLN(Com::tOpenFailedFile,filename);
		}
    }
    else
    {
        UI_STATUS(UI_TEXT_UPLOADING);
        savetosd = true;

		if( Printer::debugInfo() )
		{
			Com::printFLN(Com::tWritingToFile,filename);
		}
    }

} // startWrite


void SDCard::finishWrite()
{
    if(!savetosd) return; // already closed or never opened
    file.sync();
    file.close();
    savetosd = false;

	if( Printer::debugInfo() )
	{
		Com::printFLN(Com::tDoneSavingFile);
	}

    UI_CLEAR_STATUS;

} // finishWrite


void SDCard::deleteFile(char *filename)
{
    if(!sdactive) return;

	if(Printer::isMenuMode(MENU_MODE_SD_PRINTING))
	{
		// we do not allow to delete a file while we are printing/milling from the SD card
		if( Printer::debugErrors() )
		{
			Com::printFLN(PSTR("It is not possible to delete a file from the SD card until the current processing has finished."));
		}
		return;
	}

	sdmode = false;
    file.close();
    if(fat.remove(filename))
    {
		if( Printer::debugInfo() )
		{
	        Com::printFLN(Com::tFileDeleted);
		}
    }
    else
    {
        if(fat.rmdir(filename))
		{
			if( Printer::debugInfo() )
			{
	            Com::printFLN(Com::tFileDeleted);
			}
		}
        else
		{
			if( Printer::debugErrors() )
			{
	            Com::printFLN(Com::tDeletionFailed);
			}
		}
    }

} // deleteFile


void SDCard::makeDirectory(char *filename)
{
    if(!sdactive) return;
    sdmode = false;
    file.close();

    if(fat.mkdir(filename))
    {
		if( Printer::debugInfo() )
		{
	        Com::printFLN(Com::tDirectoryCreated);
		}
    }
    else
    {
		if( Printer::debugErrors() )
		{
	        Com::printFLN(Com::tCreationFailed);
		}
    }

} // makeDirectory


#ifdef GLENN_DEBUG
void SDCard::writeToFile()
{
	size_t	nbyte;
	char	szName[10];

	strcpy(szName, "Testing\r\n");
	nbyte = file.write(szName, strlen(szName));

	if( Printer::debugInfo() )
	{
		Com::print("L=");
		Com::print((long)nbyte);
		Com::println();
	}

} // writeToFile
#endif // GLENN_DEBUG

#endif // SDSUPPORT