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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifndef MAX_DATA_SOURCES
#define MAX_DATA_SOURCES 4
#endif

/** This class defines the general interface to handle gcode communication with the firmware. This
allows it to connect to different data sources and handle them all inside the same data structure.
If several readers are active, the first one sending a byte pauses all other inputs until the command
is complete. Only then the next reader will be queried. New queries are started in round robin fashion
so every channel gets the same chance to send commands.

Available source types are:
- serial communication port
- sd card
- flash memory
*/
class GCodeSource {
    static fast8_t numSources; ///< Number of data sources available
    static fast8_t numWriteSources;
    static GCodeSource *sources[MAX_DATA_SOURCES];
    static GCodeSource *writeableSources[MAX_DATA_SOURCES];
    public:
    static GCodeSource *activeSource;
    static void registerSource(GCodeSource *newSource);
    static void removeSource(GCodeSource *delSource);
    static void rotateSource(); ///< Move active to next source
    static void writeToAll(uint8_t byte); ///< Write to all listening sources
    static void printAllFLN(FSTRINGPARAM(text) );
    static void printAllFLN(FSTRINGPARAM(text), int32_t v);
    uint32_t lastLineNumber;
    uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    millis_t timeOfLastDataPacket;
    int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.

    GCodeSource();
    virtual ~GCodeSource() {}
    virtual bool isOpen() = 0;
    virtual bool supportsWrite() = 0; ///< true if write is a non dummy function
    virtual bool closeOnError() = 0; // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable() = 0; // would read return a new byte?
    virtual int readByte() = 0;
    virtual void close() = 0;
    virtual void writeByte(uint8_t byte) = 0;
};

class Com
{
    public:
FSTRINGVAR(tDebug)
FSTRINGVAR(tFirmware)
FSTRINGVAR(tOk)
FSTRINGVAR(tNewline)
FSTRINGVAR(tNAN)
FSTRINGVAR(tINF)
FSTRINGVAR(tError)
FSTRINGVAR(tInfo)
FSTRINGVAR(tWarning)
FSTRINGVAR(tResend)
FSTRINGVAR(tEcho)
FSTRINGVAR(tCap)
FSTRINGVAR(tOkSpace)
FSTRINGVAR(tWrongChecksum)
FSTRINGVAR(tMissingChecksum)
FSTRINGVAR(tFormatError)
FSTRINGVAR(tDonePrinting)
FSTRINGVAR(tX)
FSTRINGVAR(tY)
FSTRINGVAR(tZ)
FSTRINGVAR(tE)
FSTRINGVAR(tF)
FSTRINGVAR(tS)
FSTRINGVAR(tP)
FSTRINGVAR(tI)
FSTRINGVAR(tJ)
FSTRINGVAR(tR)
FSTRINGVAR(tD)
FSTRINGVAR(tC)
FSTRINGVAR(tH)
FSTRINGVAR(tA)
FSTRINGVAR(tB)
FSTRINGVAR(tK)
FSTRINGVAR(tL)
FSTRINGVAR(tO)
FSTRINGVAR(tSDReadError)
FSTRINGVAR(tExpectedLine)
FSTRINGVAR(tGot)
FSTRINGVAR(tSkip)
FSTRINGVAR(tBLK)
FSTRINGVAR(tStart)
FSTRINGVAR(tPowerUp)
FSTRINGVAR(tExternalReset)
FSTRINGVAR(tBrownOut)
FSTRINGVAR(tWatchdog)
FSTRINGVAR(tSoftwareReset)
FSTRINGVAR(tUnknownCommand)
FSTRINGVAR(tFreeRAM)
FSTRINGVAR(tXColon)
FSTRINGVAR(tSlash)
FSTRINGVAR(tSpaceSlash)
FSTRINGVAR(tFatal)
FSTRINGVAR(tDoorOpen)
#if JSON_OUTPUT
FSTRINGVAR(tJSONDir)
FSTRINGVAR(tJSONFiles)
FSTRINGVAR(tJSONArrayEnd)
FSTRINGVAR(tJSONErrorStart)
FSTRINGVAR(tJSONErrorEnd)
FSTRINGVAR(tJSONFileInfoStart)
FSTRINGVAR(tJSONFileInfoHeight)
FSTRINGVAR(tJSONFileInfoLayerHeight)
FSTRINGVAR(tJSONFileInfoFilament)
FSTRINGVAR(tJSONFileInfoGeneratedBy)
FSTRINGVAR(tJSONFileInfoName)
#endif
FSTRINGVAR(tSpaceXColon)
FSTRINGVAR(tSpaceYColon)
FSTRINGVAR(tSpaceZColon)
FSTRINGVAR(tSpaceEColon)
FSTRINGVAR(tTColon)
FSTRINGVAR(tSpaceBColon)
FSTRINGVAR(tSpaceAtColon)
FSTRINGVAR(tSpaceT)
FSTRINGVAR(tSpaceRaw)
FSTRINGVAR(tSpaceAt)
FSTRINGVAR(tSpaceBAtColon)
FSTRINGVAR(tColon)
FSTRINGVAR(tSpeedMultiply)
FSTRINGVAR(tFlowMultiply)
FSTRINGVAR(tFanspeed)
FSTRINGVAR(tFan2speed)
FSTRINGVAR(tPrintedFilament)
FSTRINGVAR(tPrintingTime)
FSTRINGVAR(tSpacem)
FSTRINGVAR(tSpaceDaysSpace)
FSTRINGVAR(tSpaceHoursSpace)
FSTRINGVAR(tSpaceMin)
FSTRINGVAR(tInvalidArc)
FSTRINGVAR(tComma)
FSTRINGVAR(tSpace)
FSTRINGVAR(tYColon)
FSTRINGVAR(tZColon)
FSTRINGVAR(tE0Colon)
FSTRINGVAR(tE1Colon)
FSTRINGVAR(tMS1MS2Pins)
FSTRINGVAR(tSetOutputSpace)
FSTRINGVAR(tGetInputSpace)
FSTRINGVAR(tSpaceToSpace)
FSTRINGVAR(tSpaceIsSpace)
FSTRINGVAR(tHSpace)
FSTRINGVAR(tLSpace)
FSTRINGVAR(tXMinColon)
FSTRINGVAR(tXMaxColon)
FSTRINGVAR(tYMinColon)
FSTRINGVAR(tYMaxColon)
FSTRINGVAR(tZMinColon)
FSTRINGVAR(tZ2MinMaxColon)
FSTRINGVAR(tZMaxColon)
FSTRINGVAR(tJerkColon)
FSTRINGVAR(tZJerkColon)
FSTRINGVAR(tLinearStepsColon)
FSTRINGVAR(tQuadraticStepsColon)
FSTRINGVAR(tCommaSpeedEqual)
FSTRINGVAR(tLinearLColon)
FSTRINGVAR(tQuadraticKColon)
FSTRINGVAR(tEEPROMUpdated)
FSTRINGVAR(tFilamentSlipping)
FSTRINGVAR(tPauseCommunication)
FSTRINGVAR(tContinueCommunication)
#if NONLINEAR_SYSTEM
FSTRINGVAR(tInvalidDeltaCoordinate)
FSTRINGVAR(tDBGDeltaNoMoveinDSegment)
#endif
#if DRIVE_SYSTEM == DELTA
FSTRINGVAR(tMeasurementReset)
FSTRINGVAR(tMeasureDeltaSteps)
FSTRINGVAR(tMeasureDelta)
FSTRINGVAR(tMeasureOriginReset)
FSTRINGVAR(tMeasurementAbortedOrigin)
FSTRINGVAR(tLevelingCalc)
FSTRINGVAR(tTower1)
FSTRINGVAR(tTower2)
FSTRINGVAR(tTower3)
FSTRINGVAR(tDeltaAlphaA)
FSTRINGVAR(tDeltaAlphaB)
FSTRINGVAR(tDeltaAlphaC)
FSTRINGVAR(tDeltaRadiusCorrectionA)
FSTRINGVAR(tDeltaRadiusCorrectionB)
FSTRINGVAR(tDeltaRadiusCorrectionC)
FSTRINGVAR(tDeltaDiagonalCorrectionA)
FSTRINGVAR(tDeltaDiagonalCorrectionB)
FSTRINGVAR(tDeltaDiagonalCorrectionC)
FSTRINGVAR(tEPRDeltaMaxRadius)
#endif // DRIVE_SYSTEM
#if DRIVE_SYSTEM==TUGA
FSTRINGVAR(tEPRDiagonalRodLength)
#endif
#ifdef DEBUG_GENERIC
FSTRINGVAR(tGenTemp)
#endif // DEBUG_GENERICFSTRINGVALUE(Com::,"")
FSTRINGVAR(tTargetExtr)
FSTRINGVAR(tTargetBedColon)
FSTRINGVAR(tPIDAutotuneStart)
FSTRINGVAR(tAPIDBias)
FSTRINGVAR(tAPIDD)
FSTRINGVAR(tAPIDMin)
FSTRINGVAR(tAPIDMax)
FSTRINGVAR(tAPIDKu)
FSTRINGVAR(tAPIDTu)
FSTRINGVAR(tAPIDClassic)
FSTRINGVAR(tAPIDSome)
FSTRINGVAR(tAPIDNone)
FSTRINGVAR(tAPIDPessen)
FSTRINGVAR(tAPIDTyreusLyben)
FSTRINGVAR(tAPIDKp)
FSTRINGVAR(tAPIDKi)
FSTRINGVAR(tAPIDKd)
FSTRINGVAR(tAPIDFailedHigh)
FSTRINGVAR(tAPIDFailedTimeout)
FSTRINGVAR(tAPIDFinished)
FSTRINGVAR(tMTEMPColon)
FSTRINGVAR(tHeatedBed)
FSTRINGVAR(tExtruderSpace)
FSTRINGVAR(tTempSensorDefect)
FSTRINGVAR(tTempSensorWorking)
FSTRINGVAR(tDryModeUntilRestart)
#ifdef DEBUG_QUEUE_MOVE
FSTRINGVAR(tDBGId)
FSTRINGVAR(tDBGVStartEnd)
FSTRINGVAR(tDBAccelSteps)
FSTRINGVAR(tDBGStartEndSpeed)
FSTRINGVAR(tDBGFlags)
FSTRINGVAR(tDBGJoinFlags)
FSTRINGVAR(tDBGDelta)
FSTRINGVAR(tDBGDir)
FSTRINGVAR(tDBGFullSpeed)
FSTRINGVAR(tDBGVMax)
FSTRINGVAR(tDBGAcceleration)
FSTRINGVAR(tDBGAccelerationPrim)
FSTRINGVAR(tDBGRemainingSteps)
FSTRINGVAR(tDBGAdvanceFull)
FSTRINGVAR(tDBGAdvanceRate)
FSTRINGVAR(tDBGLimitInterval)
FSTRINGVAR(tDBGMoveDistance)
FSTRINGVAR(tDBGCommandedFeedrate)
FSTRINGVAR(tDBGConstFullSpeedMoveTime)
#endif // DEBUG_QUEUE_MOVEFSTRINGVALUE(Com::,"")
#ifdef DEBUG_DELTA_OVERFLOW
FSTRINGVAR(tDBGDeltaOverflow)
#endif // DEBUG_DELTA_OVERFLOW
#ifdef DEBUG_SPLIT
FSTRINGVAR(tDBGDeltaSeconds)
FSTRINGVAR(tDBGDeltaZDelta)
FSTRINGVAR(tDBGDeltaSegments)
FSTRINGVAR(tDBGDeltaNumLines)
FSTRINGVAR(tDBGDeltaSegmentsPerLine)
FSTRINGVAR(tDBGDeltaMaxDS)
FSTRINGVAR(tDBGDeltaStepsPerSegment)
FSTRINGVAR(tDBGDeltaVirtualAxisSteps)
#endif
#ifdef DEBUG_STEPCOUNT
FSTRINGVAR(tDBGMissedSteps)
#endif
#if FEATURE_Z_PROBE
FSTRINGVAR(tZProbe)
FSTRINGVAR(tZProbeState)
FSTRINGVAR(tZProbeStartScript)
FSTRINGVAR(tZProbeEndScript)
FSTRINGVAR(tHitZProbe)
FSTRINGVAR(tZProbeAverage)
FSTRINGVAR(tZProbeZReset)
FSTRINGVAR(tZProbeBedDitance)
#endif
FSTRINGVAR(tAutolevelReset)
FSTRINGVAR(tAutolevelEnabled)
FSTRINGVAR(tAutolevelDisabled)
FSTRINGVAR(tTransformationMatrix)
FSTRINGVAR(tZProbeFailed)
FSTRINGVAR(tZProbeMax)
FSTRINGVAR(tZProbePrinterHeight)

#ifdef WAITING_IDENTIFIER
FSTRINGVAR(tWait)
#endif // WAITING_IDENTIFIER

#if EEPROM_MODE==0
FSTRINGVAR(tNoEEPROMSupport)
#else
FSTRINGVAR(tZProbeOffsetZ)
#if FEATURE_Z_PROBE
FSTRINGVAR(tZProbeHeight)
FSTRINGVAR(tZProbeOffsetX)
FSTRINGVAR(tZProbeOffsetY)
FSTRINGVAR(tZProbeSpeed)
FSTRINGVAR(tZProbeSpeedXY)
FSTRINGVAR(tZProbeX1)
FSTRINGVAR(tZProbeY1)
FSTRINGVAR(tZProbeX2)
FSTRINGVAR(tZProbeY2)
FSTRINGVAR(tZProbeX3)
FSTRINGVAR(tZProbeY3)
FSTRINGVAR(zZProbeBendingCorA)
FSTRINGVAR(zZProbeBendingCorB)
FSTRINGVAR(zZProbeBendingCorC)
#endif
#if FEATURE_AUTOLEVEL
FSTRINGVAR(tAutolevelActive)
#endif
#if FEATURE_AXISCOMP
FSTRINGVAR(tAxisCompTanXY)
FSTRINGVAR(tAxisCompTanYZ)
FSTRINGVAR(tAxisCompTanXZ)
#endif
FSTRINGVAR(tConfigStoredEEPROM)
FSTRINGVAR(tConfigLoadedEEPROM)
FSTRINGVAR(tEPRConfigResetDefaults)
FSTRINGVAR(tEPRProtocolChanged)
FSTRINGVAR(tEPR0)
FSTRINGVAR(tEPR1)
FSTRINGVAR(tEPR2)
FSTRINGVAR(tEPR3)
FSTRINGVAR(tLanguage)
FSTRINGVAR(tEPRBaudrate)
FSTRINGVAR(tEPRFilamentPrinted)
FSTRINGVAR(tEPRPrinterActive)
FSTRINGVAR(tEPRMaxInactiveTime)
FSTRINGVAR(tEPRStopAfterInactivty)
FSTRINGVAR(tEPRMaxJerk)
FSTRINGVAR(tEPRXHomePos)
FSTRINGVAR(tEPRYHomePos)
FSTRINGVAR(tEPRZHomePos)
FSTRINGVAR(tEPRXMaxLength)
FSTRINGVAR(tEPRYMaxLength)
FSTRINGVAR(tEPRZMaxLength)
FSTRINGVAR(tEPRXBacklash)
FSTRINGVAR(tEPRYBacklash)
FSTRINGVAR(tEPRZBacklash)
FSTRINGVAR(tEPRZAcceleration)
FSTRINGVAR(tEPRZTravelAcceleration)
FSTRINGVAR(tEPRAccelerationFactorAtTop)
FSTRINGVAR(tEPRZStepsPerMM)
FSTRINGVAR(tEPRZMaxFeedrate)
FSTRINGVAR(tEPRZHomingFeedrate)
#if DRIVE_SYSTEM != DELTA
FSTRINGVAR(tEPRMaxZJerk)
FSTRINGVAR(tEPRXStepsPerMM)
#if DUAL_X_RESOLUTION
FSTRINGVAR(tEPRX2StepsPerMM)
#endif
FSTRINGVAR(tEPRYStepsPerMM)
FSTRINGVAR(tEPRXMaxFeedrate)
FSTRINGVAR(tEPRYMaxFeedrate)
FSTRINGVAR(tEPRXHomingFeedrate)
FSTRINGVAR(tEPRYHomingFeedrate)
FSTRINGVAR(tEPRXAcceleration)
FSTRINGVAR(tEPRYAcceleration)
FSTRINGVAR(tEPRXTravelAcceleration)
FSTRINGVAR(tEPRYTravelAcceleration)
#else
FSTRINGVAR(tEPRDiagonalRodLength)
FSTRINGVAR(tEPRHorizontalRadius)
FSTRINGVAR(tEPRTowerXOffset)
FSTRINGVAR(tEPRTowerYOffset)
FSTRINGVAR(tEPRTowerZOffset)
#endif
FSTRINGVAR(tEPROPSMode)
FSTRINGVAR(tEPROPSMoveAfter)
FSTRINGVAR(tEPROPSMinDistance)
FSTRINGVAR(tEPROPSRetractionLength)
FSTRINGVAR(tEPROPSRetractionBacklash)
FSTRINGVAR(tEPRBedHeatManager)
FSTRINGVAR(tEPRBedPIDDriveMax)
FSTRINGVAR(tEPRBedPIDDriveMin)
FSTRINGVAR(tEPRBedPGain)
FSTRINGVAR(tEPRBedIGain)
FSTRINGVAR(tEPRBedDGain)
FSTRINGVAR(tEPRBedPISMaxValue)
FSTRINGVAR(tEPRStepsPerMM)
FSTRINGVAR(tEPRMaxFeedrate)
FSTRINGVAR(tEPRStartFeedrate)
FSTRINGVAR(tEPRAcceleration)
FSTRINGVAR(tEPRHeatManager)
FSTRINGVAR(tEPRDriveMax)
FSTRINGVAR(tEPRDriveMin)
FSTRINGVAR(tEPRPGain)
FSTRINGVAR(tEPRDead)
FSTRINGVAR(tEPRUnused)
FSTRINGVAR(tEPRIGain)
FSTRINGVAR(tEPRDGain)
FSTRINGVAR(tEPRPIDMaxValue)
FSTRINGVAR(tEPRXOffset)
FSTRINGVAR(tEPRYOffset)
FSTRINGVAR(tEPRZOffset)
FSTRINGVAR(tEPRStabilizeTime)
FSTRINGVAR(tEPRRetractionWhenHeating)
FSTRINGVAR(tEPRDistanceRetractHeating)
FSTRINGVAR(tEPRExtruderCoolerSpeed)
FSTRINGVAR(tEPRAdvanceK)
FSTRINGVAR(tEPRAdvanceL)
FSTRINGVAR(tEPRPreheatTemp)
FSTRINGVAR(tEPRPreheatBedTemp)
#endif
#if SDSUPPORT
//FSTRINGVAR(tSDRemoved)
//FSTRINGVAR(tSDInserted)
FSTRINGVAR(tSDInitFail)
FSTRINGVAR(tErrorWritingToFile)
FSTRINGVAR(tBeginFileList)
FSTRINGVAR(tEndFileList)
FSTRINGVAR(tFileOpened)
FSTRINGVAR(tSpaceSizeColon)
FSTRINGVAR(tFileSelected)
FSTRINGVAR(tFileOpenFailed)
FSTRINGVAR(tSDPrintingByte)
FSTRINGVAR(tNotSDPrinting)
FSTRINGVAR(tOpenFailedFile)
FSTRINGVAR(tWritingToFile)
FSTRINGVAR(tDoneSavingFile)
FSTRINGVAR(tFileDeleted)
FSTRINGVAR(tDeletionFailed)
FSTRINGVAR(tDirectoryCreated)
FSTRINGVAR(tCreationFailed)
FSTRINGVAR(tSDErrorCode)
#endif // SDSUPPORT
FSTRINGVAR(tHeaterDecoupled)
FSTRINGVAR(tHeaterDecoupledWarning)
#if DISTORTION_CORRECTION
FSTRINGVAR(tZCorrectionEnabled)
FSTRINGVAR(tZCorrectionDisabled)
#endif
#if FEATURE_RETRACTION
FSTRINGVAR(tEPRAutoretractEnabled)
FSTRINGVAR(tEPRRetractionLength)
FSTRINGVAR(tEPRRetractionLongLength)
FSTRINGVAR(tEPRRetractionSpeed)
FSTRINGVAR(tEPRRetractionZLift)
FSTRINGVAR(tEPRRetractionUndoExtraLength)
FSTRINGVAR(tEPRRetractionUndoExtraLongLength)
FSTRINGVAR(tEPRRetractionUndoSpeed)
#endif
FSTRINGVAR(tConfig)
FSTRINGVAR(tExtrDot)

#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_MCP4728
FSTRINGVAR(tMCPEpromSettings)
FSTRINGVAR(tMCPCurrentSettings)
#endif
FSTRINGVAR(tPrinterModeFFF)
FSTRINGVAR(tPrinterModeLaser)
FSTRINGVAR(tPrinterModeCNC)
#ifdef STARTUP_GCODE
FSTRINGVAR(tStartupGCode)
#endif
#if NONLINEAR_SYSTEM
FSTRINGVAR(tEPRSegmentsPerSecondPrint)
FSTRINGVAR(tEPRSegmentsPerSecondTravel)
#endif
#ifdef DRV_TMC2130
FSTRINGVAR(tTrinamicMotorCurrent)
FSTRINGVAR(tTrinamicMicrostepMode)
#endif

static void cap(FSTRINGPARAM(text));
static void config(FSTRINGPARAM(text));
static void config(FSTRINGPARAM(text),int value);
static void config(FSTRINGPARAM(text),const char *msg);
static void config(FSTRINGPARAM(text),int32_t value);
static void config(FSTRINGPARAM(text),uint32_t value);
static void config(FSTRINGPARAM(text),float value,uint8_t digits=2);
static void printNumber(uint32_t n);
static void printWarningF(FSTRINGPARAM(text));
static void printInfoF(FSTRINGPARAM(text));
static void printErrorF(FSTRINGPARAM(text));
static void printWarningFLN(FSTRINGPARAM(text));
static void printInfoFLN(FSTRINGPARAM(text));
static void printErrorFLN(FSTRINGPARAM(text));
static void printFLN(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text),int value);
static void printF(FSTRINGPARAM(text),const char *msg);
static void printF(FSTRINGPARAM(text),int32_t value);
static void printF(FSTRINGPARAM(text),uint32_t value);
static void printF(FSTRINGPARAM(text),float value,uint8_t digits=2);
static void printFLN(FSTRINGPARAM(text),int value);
static void printFLN(FSTRINGPARAM(text),int32_t value);
static void printFLN(FSTRINGPARAM(text),uint32_t value);
static void printFLN(FSTRINGPARAM(text),const char *msg);
static void printFLN(FSTRINGPARAM(text),float value,uint8_t digits=2);
static void printArrayFLN(FSTRINGPARAM(text),float *arr,uint8_t n=4,uint8_t digits=2);
static void printArrayFLN(FSTRINGPARAM(text),long *arr,uint8_t n=4);
static void print(long value);
static inline void print(uint32_t value) {printNumber(value);}
static inline void print(int value) {print((int32_t)value);}
static void print(const char *text);
static inline void print(char c) {GCodeSource::writeToAll(c);}
static void printFloat(float number, uint8_t digits);
static inline void print(float number) {printFloat(number, 6);}
static inline void println() {GCodeSource::writeToAll('\r');GCodeSource::writeToAll('\n');}
static bool writeToAll;    
#if FEATURE_CONTROLLER != NO_CONTROLLER
static const char* translatedF(int textId);
static void selectLanguage(fast8_t lang);
static uint8_t selectedLanguage;
#endif
    protected:
    private:
};

#ifdef DEBUG
#define SHOW(x) {Com::printF(PSTR(" " #x "=")); Com::print(x); Com::println();}
#define SHOWS(x) {Com::printF(PSTR(" " #x "=")); Com::print(x); Com::print(" steps  "); Com::print(x/80); Com::printFLN(PSTR(" mm"));}
#define SHOWM(x) {Com::printF(PSTR(" " #x "=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printFLN(PSTR(" mm"));}
#define SHOT(x) Com::printF(PSTR(x " "))
#define SHOWA(t,a,n) {SHOT(t); for (int i=0;i<n;i++) SHOWS(a[i]);}
#define SHOWAM(t,a,n) {SHOT(t); for (int i=0;i<n;i++) SHOWM(a[i]);}

#else
#define SHOW(x)
#define SHOT(x)
#define SHOWS(x)
#define SHOWM(x)
#define SHOWA(t,a,n)
#define SHOWAM(t,a,n)
#endif

#endif // COMMUNICATION_H
