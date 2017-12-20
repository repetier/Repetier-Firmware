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

FSTRINGVALUE(Com::tFirmware,"FIRMWARE_NAME:Repetier_" REPETIER_VERSION " FIRMWARE_URL:https://github.com/RF1000/Repetier-Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:" XSTR(NUM_EXTRUDER) " REPETIER_PROTOCOL:2")

FSTRINGVALUE(Com::tDebug,"Debug:");
FSTRINGVALUE(Com::tOk,"ok")
FSTRINGVALUE(Com::tNewline,"\r\n")
FSTRINGVALUE(Com::tNAN,"NAN")
FSTRINGVALUE(Com::tINF,"INF")
FSTRINGVALUE(Com::tError,"Error:")
FSTRINGVALUE(Com::tInfo,"Info:")
FSTRINGVALUE(Com::tWarning,"Warning:")
FSTRINGVALUE(Com::tResend,"Resend:")
FSTRINGVALUE(Com::tEcho,"Echo:")
FSTRINGVALUE(Com::tOkSpace,"ok ")
FSTRINGVALUE(Com::tWrongChecksum,"Wrong checksum")
FSTRINGVALUE(Com::tMissingChecksum,"Missing checksum")
FSTRINGVALUE(Com::tFormatError,"Format error")
FSTRINGVALUE(Com::tDonePrinting,"Done printing file")
FSTRINGVALUE(Com::tX," X")
FSTRINGVALUE(Com::tY," Y")
FSTRINGVALUE(Com::tZ," Z")
FSTRINGVALUE(Com::tE," E")
FSTRINGVALUE(Com::tF," F")
FSTRINGVALUE(Com::tS," S")
FSTRINGVALUE(Com::tP," P")
FSTRINGVALUE(Com::tI," I")
FSTRINGVALUE(Com::tJ," J")
FSTRINGVALUE(Com::tR," R")
FSTRINGVALUE(Com::tSDReadError,"SD read error")
FSTRINGVALUE(Com::tExpectedLine,"Error:expected line ")
FSTRINGVALUE(Com::tGot," got ")
FSTRINGVALUE(Com::tSkip,"skip ")
FSTRINGVALUE(Com::tBLK,"BLK ")
FSTRINGVALUE(Com::tStart,"start")					// do not change "start" to "Start" because some applications might not be able to detect "Start" as "start"
FSTRINGVALUE(Com::tStartWatchdog,"start Watchdog")
FSTRINGVALUE(Com::tPowerUp,"PowerUp")
FSTRINGVALUE(Com::tExternalReset,"External Reset")
FSTRINGVALUE(Com::tBrownOut,"Brown out Reset")
FSTRINGVALUE(Com::tWatchdog,"Watchdog Reset")
FSTRINGVALUE(Com::tSoftwareReset,"Software Reset")
FSTRINGVALUE(Com::tUnknownCommand,"Unknown command:")
FSTRINGVALUE(Com::tFreeRAM,"Free RAM:")
FSTRINGVALUE(Com::tXColon,"X:")
FSTRINGVALUE(Com::tSpaceXColon," X:")
FSTRINGVALUE(Com::tSpaceYColon," Y:")
FSTRINGVALUE(Com::tSpaceZColon," Z:")
FSTRINGVALUE(Com::tSpaceEColon," E:")
FSTRINGVALUE(Com::tTColon,"T:")
FSTRINGVALUE(Com::tSpaceBColon," B:")
FSTRINGVALUE(Com::tSpaceAtColon," @:")
FSTRINGVALUE(Com::tSpaceT," T")
FSTRINGVALUE(Com::tSpaceAt," @")
FSTRINGVALUE(Com::tSpaceBAtColon," B@:")
FSTRINGVALUE(Com::tSpaceRaw," RAW")
FSTRINGVALUE(Com::tColon,":")
FSTRINGVALUE(Com::tSlash,"/")
FSTRINGVALUE(Com::tSpaceSlash," /")
FSTRINGVALUE(Com::tSpeedMultiply,"SpeedMultiply:")
FSTRINGVALUE(Com::tFlowMultiply,"FlowMultiply:")
FSTRINGVALUE(Com::tFanspeed,"Fanspeed:")
FSTRINGVALUE(Com::tPrintedFilament,"Printed filament:")
FSTRINGVALUE(Com::tPrintingTime,"Printing time:")
FSTRINGVALUE(Com::tMillingTime,"Milling time:")
FSTRINGVALUE(Com::tSpacem,"m ")
FSTRINGVALUE(Com::tSpaceDaysSpace," days ")
FSTRINGVALUE(Com::tSpaceHoursSpace," hours ")
FSTRINGVALUE(Com::tSpaceMin," min")
FSTRINGVALUE(Com::tInvalidArc,"Invalid arc")
FSTRINGVALUE(Com::tComma,",")
FSTRINGVALUE(Com::tSpace," ")
FSTRINGVALUE(Com::tYColon,"Y:")
FSTRINGVALUE(Com::tZColon,"Z:")
FSTRINGVALUE(Com::tE0Colon,"E0:")
FSTRINGVALUE(Com::tE1Colon,"E1:")
FSTRINGVALUE(Com::tMS1MS2Pins,"MS1,MS2 Pins")
FSTRINGVALUE(Com::tSetOutputSpace,"Set output ")
FSTRINGVALUE(Com::tSpaceToSpace," to ")
FSTRINGVALUE(Com::tHSpace,"H ")
FSTRINGVALUE(Com::tLSpace,"L ")
FSTRINGVALUE(Com::tXMinColon,"x_min:")
FSTRINGVALUE(Com::tXMaxColon,"x_max:")
FSTRINGVALUE(Com::tYMinColon,"y_min:")
FSTRINGVALUE(Com::tYMaxColon,"y_max:")
FSTRINGVALUE(Com::tZMinColon,"z_min:")
FSTRINGVALUE(Com::tZMaxColon,"z_max:")
FSTRINGVALUE(Com::tJerkColon,"Jerk:")
FSTRINGVALUE(Com::tZJerkColon," ZJerk:")
FSTRINGVALUE(Com::tLinearStepsColon," linear steps:")
FSTRINGVALUE(Com::tQuadraticStepsColon," quadratic steps:")
FSTRINGVALUE(Com::tCommaSpeedEqual,", speed=")
FSTRINGVALUE(Com::tEEPROMUpdated,"EEPROM updated")
FSTRINGVALUE(Com::tLinearLColon,"linear L:")
FSTRINGVALUE(Com::tQuadraticKColon," quadratic K:")

#if FEATURE_SERVICE_INTERVAL
FSTRINGVALUE(Com::tPrintedFilamentService,"Printed filament since last Service:")
FSTRINGVALUE(Com::tPrintingTimeService,"Printing time since last Service:")
FSTRINGVALUE(Com::tMillingTimeService,"Milling time since last Service:")
#endif // FEATURE_SERVICE_INTERVAL

#ifdef DEBUG_GENERIC
FSTRINGVALUE(Com::tGenTemp,"GenTemp:")
#endif // DEBUG_GENERIC

FSTRINGVALUE(Com::tTargetExtr,"TargetExtr")
FSTRINGVALUE(Com::tTargetBedColon,"TargetBed:")
FSTRINGVALUE(Com::tPIDAutotuneStart,"PID Autotune start")
FSTRINGVALUE(Com::tAPIDBias," bias: ")
FSTRINGVALUE(Com::tAPIDD," d: ")
FSTRINGVALUE(Com::tAPIDMin," min: ")
FSTRINGVALUE(Com::tAPIDMax," max: ")
FSTRINGVALUE(Com::tAPIDKu," Ku: ")
FSTRINGVALUE(Com::tAPIDTu," Tu: ")
FSTRINGVALUE(Com::tAPIDClassic," Classic PID")
FSTRINGVALUE(Com::tAPIDKp," Kp: ")
FSTRINGVALUE(Com::tAPIDKi," Ki: ")
FSTRINGVALUE(Com::tAPIDKd," Kd: ")
FSTRINGVALUE(Com::tAPIDFailedHigh,"PID Autotune failed! Temperature to high")
FSTRINGVALUE(Com::tAPIDFailedTimeout,"PID Autotune failed! timeout")
FSTRINGVALUE(Com::tAPIDFinished,"PID Autotune finished ! Place the Kp, Ki and Kd constants in the Configuration.h or EEPROM")
FSTRINGVALUE(Com::tMTEMPColon,"MTEMP:")
FSTRINGVALUE(Com::tHeatedBed,"heated bed")
FSTRINGVALUE(Com::tExtruderSpace,"extruder ")
FSTRINGVALUE(Com::tTempSensorDefect,": temp sensor defect")
FSTRINGVALUE(Com::tTempSensorWorking,": working")
FSTRINGVALUE(Com::tDryModeUntilRestart,"Printer set into dry run mode until restart!")

#if (DEBUG_QUEUE_MOVE || DEBUG_DIRECT_MOVE)
FSTRINGVALUE(Com::tDBGId,"ID:")
FSTRINGVALUE(Com::tDBGVStartEnd,"vStart/End:")
FSTRINGVALUE(Com::tDBAccelSteps,"accel/decel steps:")
FSTRINGVALUE(Com::tDBGStartEndSpeed,"st./end speed:")
FSTRINGVALUE(Com::tDBGFlags,"Flags:")
FSTRINGVALUE(Com::tDBGJoinFlags,"joinFlags:")
FSTRINGVALUE(Com::tDBGDelta,"Delta")
FSTRINGVALUE(Com::tDBGDir,"Dir:")
FSTRINGVALUE(Com::tDBGFullSpeed,"fullSpeed:")
FSTRINGVALUE(Com::tDBGVMax,"vMax:")
FSTRINGVALUE(Com::tDBGAcceleration,"Acceleration:")
FSTRINGVALUE(Com::tDBGAccelerationPrim,"Acceleration Prim:")
FSTRINGVALUE(Com::tDBGRemainingSteps,"Remaining steps:")
FSTRINGVALUE(Com::tDBGAdvanceFull,"advanceFull:")
FSTRINGVALUE(Com::tDBGAdvanceRate,"advanceRate:")
FSTRINGVALUE(Com::tDBGLimitInterval,"LimitInterval:")
FSTRINGVALUE(Com::tDBGMoveDistance,"Move distance on the XYZ space:")
FSTRINGVALUE(Com::tDBGCommandedFeedrate,"Commanded feedrate:")
FSTRINGVALUE(Com::tDBGConstFullSpeedMoveTime,"Constant full speed move time:")
#endif // DEBUG_QUEUE_MOVE || DEBUG_DIRECT_MOVE

#ifdef DEBUG_SPLIT
FSTRINGVALUE(Com::tDBGDeltaSeconds,"Seconds:")
FSTRINGVALUE(Com::tDBGDeltaZDelta,"Z delta:")
FSTRINGVALUE(Com::tDBGDeltaSegments,"Segments:")
FSTRINGVALUE(Com::tDBGDeltaNumLines,"Num lines:")
FSTRINGVALUE(Com::tDBGDeltaSegmentsPerLine,"segments_per_line:")
FSTRINGVALUE(Com::tDBGDeltaMaxDS,"Max DS:")
FSTRINGVALUE(Com::tDBGDeltaStepsPerSegment,"Steps Per Segment:")
FSTRINGVALUE(Com::tDBGDeltaVirtualAxisSteps,"Virtual axis steps:")
#endif // DEBUG_SPLIT

#ifdef DEBUG_STEPCOUNT
FSTRINGVALUE(Com::tDBGMissedSteps,"Missed steps:")
#endif // DEBUG_STEPCOUNT

#ifdef WAITING_IDENTIFIER
FSTRINGVALUE(Com::tWait,WAITING_IDENTIFIER)
#endif // WAITING_IDENTIFIER

#if EEPROM_MODE==0
FSTRINGVALUE(Com::tNoEEPROMSupport,"No EEPROM support compiled.\r\n")
#else
FSTRINGVALUE(Com::tConfigStoredEEPROM,"Configuration stored to EEPROM.")
FSTRINGVALUE(Com::tConfigLoadedEEPROM,"Configuration loaded from EEPROM.")
FSTRINGVALUE(Com::tEPRConfigResetDefaults,"Configuration reset to defaults.")
FSTRINGVALUE(Com::tEPRProtocolChanged,"Protocol version changed, upgrading")
FSTRINGVALUE(Com::tExtrDot,"Extr.")
FSTRINGVALUE(Com::tEPR0,"EPR:0 ")
FSTRINGVALUE(Com::tEPR1,"EPR:1 ")
FSTRINGVALUE(Com::tEPR2,"EPR:2 ")
FSTRINGVALUE(Com::tEPR3,"EPR:3 ")
FSTRINGVALUE(Com::tEPRBaudrate,"Baudrate")
FSTRINGVALUE(Com::tEPRFilamentPrinted,"Filament printed [m]")
FSTRINGVALUE(Com::tEPRFilamentPrintedService,"Filament printed since last Service [m]")
FSTRINGVALUE(Com::tEPRPrinterActive,"Printer active [s]")
FSTRINGVALUE(Com::tEPRPrinterActiveService,"Printer active since last Service [s]")
FSTRINGVALUE(Com::tEPRMillerActive,"Miller active [s]")
FSTRINGVALUE(Com::tEPRMillerActiveService,"Miller active since last Service [s]")
FSTRINGVALUE(Com::tEPRMaxInactiveTime,"Max. inactive time [ms,0=off]")
FSTRINGVALUE(Com::tEPRStopAfterInactivty,"Stop stepper after inactivity [ms,0=off]")
FSTRINGVALUE(Com::tEPRXHomePos,"X home pos [mm]")
FSTRINGVALUE(Com::tEPRYHomePos,"Y home pos [mm]")
FSTRINGVALUE(Com::tEPRZHomePos,"Z home pos [mm]")
FSTRINGVALUE(Com::tEPRXMaxLength,"X max length [mm]")
FSTRINGVALUE(Com::tEPRYMaxLength,"Y max length [mm]")
FSTRINGVALUE(Com::tEPRZMaxLength,"Z max length [mm]")
FSTRINGVALUE(Com::tEPRXBacklash,"X backlash [mm]")
FSTRINGVALUE(Com::tEPRYBacklash,"Y backlash [mm]")
FSTRINGVALUE(Com::tEPRZBacklash,"Z backlash [mm]")
FSTRINGVALUE(Com::tEPRMaxJerk,"Max. jerk [mm/s]")
FSTRINGVALUE(Com::tEPRMaxZJerk,"Max. Z-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRXStepsPerMM,"X-axis steps per mm")
FSTRINGVALUE(Com::tEPRYStepsPerMM,"Y-axis steps per mm")
FSTRINGVALUE(Com::tEPRZStepsPerMM,"Z-axis steps per mm")
FSTRINGVALUE(Com::tEPRXMaxFeedrate,"X-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYMaxFeedrate,"Y-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZMaxFeedrate,"Z-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXHomingFeedrate,"X-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYHomingFeedrate,"Y-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZHomingFeedrate,"Z-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXAcceleration,"X-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYAcceleration,"Y-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZAcceleration,"Z-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRXTravelAcceleration,"X-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYTravelAcceleration,"Y-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZTravelAcceleration,"Z-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZOffset,"Z-Offset [um]")
FSTRINGVALUE(Com::tEPRZMode,"Z Scale")
FSTRINGVALUE(Com::tEPROPSMode,"OPS operation mode [0=Off,1=Classic,2=Fast]")
FSTRINGVALUE(Com::tEPROPSMoveAfter,"OPS move after x% retract [%]")
FSTRINGVALUE(Com::tEPROPSMinDistance,"OPS min. distance for fil. retraction [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionLength,"OPS retraction length [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionBacklash,"OPS retraction backlash [mm]")
FSTRINGVALUE(Com::tEPRBedHeatManager,"Bed Heat Manager [0-3]")
FSTRINGVALUE(Com::tEPRBedPIDDriveMax,"Bed PID drive max")
FSTRINGVALUE(Com::tEPRBedPIDDriveMin,"Bed PID drive min")
FSTRINGVALUE(Com::tEPRBedPGain,"Bed PID P-gain")
FSTRINGVALUE(Com::tEPRBedIGain,"Bed PID I-gain")
FSTRINGVALUE(Com::tEPRBedDGain,"Bed PID D-gain")
FSTRINGVALUE(Com::tEPRBedPISMaxValue,"Bed PID max value [0-255]")
FSTRINGVALUE(Com::tEPRStepsPerMM,"steps per mm")
FSTRINGVALUE(Com::tEPRMaxFeedrate,"max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRStartFeedrate,"start feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRAcceleration,"acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRHeatManager,"heat manager [0-3]")
FSTRINGVALUE(Com::tEPRDriveMax,"PID drive max")
FSTRINGVALUE(Com::tEPRDriveMin,"PID drive min")
FSTRINGVALUE(Com::tEPRPGain,"PID P-gain/dead-time")
FSTRINGVALUE(Com::tEPRIGain,"PID I-gain")
FSTRINGVALUE(Com::tEPRDGain,"PID D-gain")
FSTRINGVALUE(Com::tEPRPIDMaxValue,"PID max value [0-255]")
FSTRINGVALUE(Com::tEPRXOffset,"X-offset [mm]")
FSTRINGVALUE(Com::tEPRYOffset,"Y-offset [mm]")
FSTRINGVALUE(Com::tEPRStabilizeTime,"temp. stabilize time [s]")
FSTRINGVALUE(Com::tEPRRetractionWhenHeating,"temp. for retraction when heating [C]")
FSTRINGVALUE(Com::tEPRDistanceRetractHeating,"distance to retract when heating [mm]")
FSTRINGVALUE(Com::tEPRExtruderCoolerSpeed,"extruder cooler speed [0-255]")
FSTRINGVALUE(Com::tEPRAdvanceK,"advance K [0=off]")
FSTRINGVALUE(Com::tEPRAdvanceL,"advance L [0=off]")
FSTRINGVALUE(Com::tEPRBeeperMode,"beeper mode [0=off]")
FSTRINGVALUE(Com::tEPRCaseLightsMode,"case lights mode [0=off, 1=on]")
FSTRINGVALUE(Com::tEPR230VOutputMode,"230V output mode [0=off, 1=on]")
FSTRINGVALUE(Com::tEPROperatingMode,"operating mode [1=print, 2=mill]")
FSTRINGVALUE(Com::tEPRZEndstopType,"Z endstop type [1=single, 2=circuit]")
FSTRINGVALUE(Com::tEPRHotendType,"Hotend type [2=V1, 3=V2 single, 4=V2 dual]")
FSTRINGVALUE(Com::tEPRMillerType,"Miller type [1=one track, 2=two tracks]")
FSTRINGVALUE(Com::tEPRRGBLightMode,"RGB Light mode [0=off, 1=white, 2=color, 3=manual]")
FSTRINGVALUE(Com::tEPRFET1Mode,"FET1 mode [0=off, 1=on]")
FSTRINGVALUE(Com::tEPRFET2Mode,"FET2 mode [0=off, 1=on]")
FSTRINGVALUE(Com::tEPRFET3Mode,"FET3 mode [0=off, 1=on]")
#endif // EEPROM_MODE

#if SDSUPPORT
FSTRINGVALUE(Com::tSDRemoved,UI_TEXT_SD_REMOVED)
FSTRINGVALUE(Com::tSDInserted,UI_TEXT_SD_INSERTED)
FSTRINGVALUE(Com::tSDInitFail,"SD init fail")
FSTRINGVALUE(Com::tErrorWritingToFile,"error writing to file")
FSTRINGVALUE(Com::tBeginFileList,"Begin file list")
FSTRINGVALUE(Com::tEndFileList,"End file list")
FSTRINGVALUE(Com::tFileOpened,"File opened:")
FSTRINGVALUE(Com::tSpaceSizeColon," Size:")
FSTRINGVALUE(Com::tFileSelected,"File selected")
FSTRINGVALUE(Com::tFileOpenFailed,"file.open failed")
FSTRINGVALUE(Com::tSDPrintingByte,"SD printing byte ")
FSTRINGVALUE(Com::tNotSDPrinting,"Not SD printing")
FSTRINGVALUE(Com::tOpenFailedFile,"open failed, File: ")
FSTRINGVALUE(Com::tWritingToFile,"Writing to file: ")
FSTRINGVALUE(Com::tDoneSavingFile,"Done saving file.")
FSTRINGVALUE(Com::tFileDeleted,"File deleted")
FSTRINGVALUE(Com::tDeletionFailed,"Deletion failed")
FSTRINGVALUE(Com::tDirectoryCreated,"Directory created")
FSTRINGVALUE(Com::tCreationFailed,"Creation failed")
FSTRINGVALUE(Com::tSDErrorCode,"SD errorCode:")
#endif // SDSUPPORT

#if FEATURE_OUTPUT_FINISHED_OBJECT 
FSTRINGVALUE(Com::tOutputObjectPrint,OUTPUT_OBJECT_SCRIPT_PRINT)
FSTRINGVALUE(Com::tOutputObjectMill,OUTPUT_OBJECT_SCRIPT_MILL)
#endif // FEATURE_OUTPUT_FINISHED_OBJECT 

FSTRINGVALUE(Com::tUnmountFilamentWithHeating,UNMOUNT_FILAMENT_SCRIPT_WITH_HEATING)
FSTRINGVALUE(Com::tUnmountFilamentWithoutHeating,UNMOUNT_FILAMENT_SCRIPT_WITHOUT_HEATING)
FSTRINGVALUE(Com::tMountFilamentWithHeating,MOUNT_FILAMENT_SCRIPT_WITH_HEATING)
FSTRINGVALUE(Com::tMountFilamentWithoutHeating,MOUNT_FILAMENT_SCRIPT_WITHOUT_HEATING)

#if FEATURE_FIND_Z_ORIGIN
FSTRINGVALUE(Com::tFindZOrigin,FIND_Z_ORIGIN_SCRIPT)
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_TEST_STRAIN_GAUGE
FSTRINGVALUE(Com::tTestStrainGauge,TEST_STRAIN_GAUGE_SCRIPT)
#endif // FEATURE_TEST_STRAIN_GAUGE


void Com::printWarningF(FSTRINGPARAM(text))
{
    printF(tWarning);
    printF(text);
} // printWarningF


void Com::printWarningFLN(FSTRINGPARAM(text))
{
    printF(tWarning);
    printFLN(text);
} // printWarningFLN


void Com::printInfoF(FSTRINGPARAM(text))
{
    printF(tInfo);
    printF(text);
} // printInfoF


void Com::printInfoFLN(FSTRINGPARAM(text))
{
    printF(tInfo);
    printFLN(text);
} // printInfoFLN

void Com::printErrorF(FSTRINGPARAM(text))
{
    printF(tError);
    printF(text);
} // printErrorF


void Com::printErrorFLN(FSTRINGPARAM(text))
{
    printF(tError);
    printFLN(text);
} // printErrorFLN


void Com::printFLN(FSTRINGPARAM(text))
{
    printF(text);
    println();
} // printFLN


void Com::printFLN(FSTRINGPARAM(text),const char *msg)
{
    printF(text);
    print(msg);
    println();
} // printFLN


void Com::printF(FSTRINGPARAM(ptr))
{
  char c;
  while ((c=HAL::readFlashByte(ptr++)) != 0)
     HAL::serialWriteByte(c);
} // printF


void Com::printF(FSTRINGPARAM(text),const char *msg)
{
    printF(text);
    print(msg);
} // printF


void Com::printF(FSTRINGPARAM(text),int value)
{
    printF(text);
    print(value);
} // printF


void Com::printF(FSTRINGPARAM(text),int32_t value)
{
    printF(text);
    print(value);
} // printF


void Com::printF(FSTRINGPARAM(text),uint32_t value)
{
    printF(text);
    printNumber(value);
} // printF


void Com::printFLN(FSTRINGPARAM(text),int value)
{
    printF(text);
    print(value);
    println();
} // printFLN


void Com::printFLN(FSTRINGPARAM(text),int32_t value)
{
    printF(text);
    print(value);
    println();
} // printFLN


void Com::printFLN(FSTRINGPARAM(text),uint32_t value)
{
    printF(text);
    printNumber(value);
    println();
} // printFLN


void Com::printFLN(FSTRINGPARAM(text),float value,uint8_t digits)
{
    printF(text);
    printFloat(value,digits);
    println();
} // printFLN


void Com::printF(FSTRINGPARAM(text),float value,uint8_t digits)
{
    printF(text);
    printFloat(value,digits);
} // printF


void Com::print(const char *text)
{
	while(*text)
	{
		HAL::serialWriteByte(*text++);
	}
} // print


void Com::print(long value)
{
    if(value<0)
	{
        HAL::serialWriteByte('-');
        value = -value;
    }
    printNumber(value);
} // print


void Com::printNumber(uint32_t n)
{
	char buf[11]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[10];


	*str = '\0';
	do
	{
		unsigned long m = n;
		n /= 10;
		*--str = '0'+(m - 10 * n);
	}while(n);

	print(str);
} // printNumber


void Com::printArrayFLN(FSTRINGPARAM(text),float *arr,uint8_t n,uint8_t digits)
{
    printF(text);
    for(uint8_t i=0; i<n; i++)
        printF(Com::tSpace,arr[i],digits);
    println();
} // printArrayFLN


void Com::printArrayFLN(FSTRINGPARAM(text),int32_t *arr,uint8_t n)
{
    printF(text);
    for(uint8_t i=0; i<n; i++)
        printF(Com::tSpace,arr[i]);
    println();
} // printArrayFLN


void Com::printFloat(float number, uint8_t digits)
{
	if (isnan(number))
	{
		printF(tNAN);
		return;
	}
	if (isinf(number))
	{
		printF(tINF);
		return;
	}

	// Handle negative numbers
	if (number < 0.0)
	{
		print('-');
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	float rounding = 0.5;
	for (uint8_t i=0; i<digits; ++i)
		rounding /= 10.0;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long	int_part  = (unsigned long)number;
	float			remainder = number - (float)int_part;
  
	
	printNumber(int_part);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0)
		print('.');

	// Extract digits from the remainder one at a time
	while (digits-- > 0)
	{
		remainder *= 10.0;
		int toPrint = int(remainder);
		print(toPrint);
		remainder -= toPrint;
	}
} // printFloat
