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

#pragma once

#ifndef MAX_DATA_SOURCES
#define MAX_DATA_SOURCES 4
#endif

typedef void (*PromptDialogCallback)(int selection);
enum class BoolFormat { TRUEFALSE,
                        ONOFF,
                        YESNO };

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
    static GCodeSource* sources[MAX_DATA_SOURCES];
    static GCodeSource* writeableSources[MAX_DATA_SOURCES];

public:
    static GCodeSource* activeSource;
    static GCodeSource* usbHostSource;
    static void registerSource(GCodeSource* newSource);
    static void removeSource(GCodeSource* delSource);
    static void rotateSource();           ///< Move active to next source
    static void writeToAll(uint8_t byte); ///< Write to all listening sources
    static void prefetchAll();
    static void printAllFLN(FSTRINGPARAM(text));
    static void printAllFLN(FSTRINGPARAM(text), int32_t v);
    static bool hasBaudSources();
    uint32_t lastLineNumber;
    uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    millis_t timeOfLastDataPacket;
    int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.
    bool outOfOrder;         ///< True if out of order execution is allowed
    GCodeSource();
    virtual ~GCodeSource() { }
    virtual bool isOpen() = 0;
    virtual bool supportsWrite() = 0; ///< true if write is a non dummy function
    virtual bool closeOnError() = 0;  // return true if the channel can not interactively correct errors.
    virtual bool dataAvailable() = 0; // would read return a new byte?
    virtual int readByte() = 0;
    virtual void close() = 0;
    virtual void writeByte(uint8_t byte) = 0;
    virtual void prefetchContent() { } // Used for emergency parsing to read ahaed
};

class Com {
public:
    FSTRINGVAR(tFirmwareVersion)
    FSTRINGVAR(tFirmwareCompiled)
    FSTRINGVAR(tVendor)
    FSTRINGVAR(tPrinterName)
    FSTRINGVAR(tXAxis)
    FSTRINGVAR(tYAxis)
    FSTRINGVAR(tZAxis)
    FSTRINGVAR(tEAxis)
    FSTRINGVAR(tAAxis)
    FSTRINGVAR(tBAxis)
    FSTRINGVAR(tCAxis)

    FSTRINGVAR(tEmpty)
    FSTRINGVAR(tDebug)
    FSTRINGVAR(tFirmware)
    FSTRINGVAR(tOk)
    FSTRINGVAR(tNewline)
    FSTRINGVAR(tNAN)
    FSTRINGVAR(tINF)
    FSTRINGVAR(tError)
    FSTRINGVAR(tLog)
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
    FSTRINGVAR(tU)
    FSTRINGVAR(tV)
    FSTRINGVAR(tW)
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
    FSTRINGVAR(tBtnOK)
    FSTRINGVAR(tM999)
    FSTRINGVAR(tTestM999)
    FSTRINGVAR(tColdExtrusionPrevented)
    // Units
    FSTRINGVAR(tUnitMM)
    FSTRINGVAR(tUnitMMPS)
    FSTRINGVAR(tUnitMMPS2)
    FSTRINGVAR(tUnitMMPS3)
    FSTRINGVAR(tUnitBaud)
    FSTRINGVAR(tUnitPercent)
    FSTRINGVAR(tUnitDegCelsius)
    FSTRINGVAR(tUnitStepsPerMM)
    FSTRINGVAR(tUnitSteps)
    FSTRINGVAR(tUnitSeconds)
    FSTRINGVAR(tUnitMilliSeconds)
    FSTRINGVAR(tUnitMilliWatt)
    FSTRINGVAR(tUnitPWM)
    FSTRINGVAR(tUnitRPM)
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
    FSTRINGVAR(tSpaceXColon)
    FSTRINGVAR(tSpaceYColon)
    FSTRINGVAR(tSpaceZColon)
    FSTRINGVAR(tSpaceEColon)
    FSTRINGVAR(tSpaceAColon)
    FSTRINGVAR(tSpaceCColon)
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
    FSTRINGVAR(tAMinColon)
    FSTRINGVAR(tAMaxColon)
    FSTRINGVAR(tBMinColon)
    FSTRINGVAR(tBMaxColon)
    FSTRINGVAR(tCMinColon)
    FSTRINGVAR(tCMaxColon)
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
    FSTRINGVAR(tInvalidDeltaCoordinate)
    FSTRINGVAR(tDBGDeltaNoMoveinDSegment)
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
    FSTRINGVAR(tProbing)
    FSTRINGVAR(tZProbe)
    FSTRINGVAR(tZProbeStartScript)
    FSTRINGVAR(tZProbeEndScript)
    FSTRINGVAR(tHitZProbe)
    FSTRINGVAR(tZProbeAverage)
    FSTRINGVAR(tZProbeZReset)
    FSTRINGVAR(tZProbeBedDitance)
    FSTRINGVAR(tZProbeState)
    FSTRINGVAR(tAutolevelReset)
    FSTRINGVAR(tAutolevelEnabled)
    FSTRINGVAR(tAutolevelDisabled)

    FSTRINGVAR(tBumpCSVHeader)
    FSTRINGVAR(tErrorImportBump)
    FSTRINGVAR(tErrorExportBump)
    FSTRINGVAR(tNoGridLeveling)
    FSTRINGVAR(tNoDistortionData)
    FSTRINGVAR(tTransformationMatrix)
    FSTRINGVAR(tZProbeFailed)
    FSTRINGVAR(tZProbeMax)
    FSTRINGVAR(tZProbePrinterHeight)

    FSTRINGVAR(tWait)

    FSTRINGVAR(tNoEEPROMSupport)
    FSTRINGVAR(tZProbeOffsetZ)
    FSTRINGVAR(tAxisCompTanXY)
    FSTRINGVAR(tAxisCompTanYZ)
    FSTRINGVAR(tAxisCompTanXZ)
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
    FSTRINGVAR(tEPRMaxZJerk)
    FSTRINGVAR(tEPRXStepsPerMM)
    FSTRINGVAR(tEPRYStepsPerMM)
    FSTRINGVAR(tEPRXMaxFeedrate)
    FSTRINGVAR(tEPRYMaxFeedrate)
    FSTRINGVAR(tEPRXHomingFeedrate)
    FSTRINGVAR(tEPRYHomingFeedrate)
    FSTRINGVAR(tEPRXAcceleration)
    FSTRINGVAR(tEPRYAcceleration)
    FSTRINGVAR(tEPRXTravelAcceleration)
    FSTRINGVAR(tEPRYTravelAcceleration)
    FSTRINGVAR(tEPRDiagonalRodLength)
    FSTRINGVAR(tEPRHorizontalRadius)
    FSTRINGVAR(tEPRTowerXOffset)
    FSTRINGVAR(tEPRTowerYOffset)
    FSTRINGVAR(tEPRTowerZOffset)
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
    FSTRINGVAR(tEPRToneVolume)


    // SDCARD RELATED
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
    FSTRINGVAR(tInvalidFiletype)
    FSTRINGVAR(tInvalidFilename)
    FSTRINGVAR(tNoMountedCard)
    FSTRINGVAR(tSDPrintingByte)
    FSTRINGVAR(tNotSDPrinting)
    FSTRINGVAR(tCurrentOpenFile)
    FSTRINGVAR(tOpenFailedFile)
    FSTRINGVAR(tWritingToFile)
    FSTRINGVAR(tDoneSavingFile)
    FSTRINGVAR(tFileDeleted)
    FSTRINGVAR(tDeletionFailed)
    FSTRINGVAR(tDirectoryCreated)
    FSTRINGVAR(tCreationFailed)
    FSTRINGVAR(tSDErrorCode)
    // END SDCARD RELATED


    FSTRINGVAR(tHeaterDecoupled)
    FSTRINGVAR(tHeaterDecoupledWarning)
#if FEATURE_RETRACTION
    FSTRINGVAR(tEPRAutoretractEnabled)
    FSTRINGVAR(tEPRRetractionLength)
    FSTRINGVAR(tEPRRetractionSpeed)
    FSTRINGVAR(tEPRRetractionZLift)
    FSTRINGVAR(tEPRRetractionUndoExtraLength)
    FSTRINGVAR(tEPRRetractionLongLength)
    FSTRINGVAR(tEPRRetractionUndoExtraLongLength)
    FSTRINGVAR(tEPRRetractionUndoSpeed)
#endif
    FSTRINGVAR(tConfig)
    FSTRINGVAR(tExtrDot)

    FSTRINGVAR(tPrinterModeFFF)
    FSTRINGVAR(tPrinterModeLaser)
    FSTRINGVAR(tPrinterModeCNC)
#ifdef STARTUP_GCODE
    FSTRINGVAR(tStartupGCode)
#endif
    FSTRINGVAR(tColonSpace)
    FSTRINGVAR(tI2CError)
    // motor diver strings
    FSTRINGVAR(tMotorMotorSpace)
    FSTRINGVAR(tMotorResolutionColon)
    FSTRINGVAR(tMotorResolutionSteps)
    FSTRINGVAR(tMotorMicrosteps)
    FSTRINGVAR(tMotorRMSCurrentMA)
    FSTRINGVAR(tMotorRMSCurrentMAColon)
    FSTRINGVAR(tMotorHybridTresholdMMS)
    FSTRINGVAR(tMotorStealthOnOff)
    FSTRINGVAR(tMotorStallSensitivity255)
    FSTRINGVAR(tMotorStallSensitivity64)
    FSTRINGVAR(tMotorStatusColon)
    FSTRINGVAR(tMotorNoConnection)
    FSTRINGVAR(tMotorNoPower)
    FSTRINGVAR(tMotorEnabledColon)
    FSTRINGVAR(tMotorSpaceSetColonSpace)
    FSTRINGVAR(tMotorMaxCurrentMA)
    FSTRINGVAR(tMotorMicrostepsColon)
    FSTRINGVAR(tMotorSpaceMresColon)
    FSTRINGVAR(tMotorStealthChopColon)
    FSTRINGVAR(tMotorHybridTresholdMMSColon)
    FSTRINGVAR(tMotorHybridModeDisabled)
    FSTRINGVAR(tMotorStallguardSensitivityColon)
    FSTRINGVAR(tMotorTStep)
    FSTRINGVAR(tMax)
    FSTRINGVAR(tMotorTPWMTHRS)
    FSTRINGVAR(tMotorTPOWERDOWN)
    FSTRINGVAR(tMotorIRUN)
    FSTRINGVAR(tMotorSlash31)
    FSTRINGVAR(tMotorIHOLD)
    FSTRINGVAR(tMotorCSActual)
    FSTRINGVAR(tMotorVSense)
    FSTRINGVAR(tMotorTOff)
    FSTRINGVAR(tMotorHStart)
    FSTRINGVAR(tMotorHEnd)
    FSTRINGVAR(tMotorBlankTime)
    FSTRINGVAR(tMotorTBLColon)
    FSTRINGVAR(tMotorIOIN)
    FSTRINGVAR(tMotorGSTAT)
    FSTRINGVAR(tMotorDriverShort)
    FSTRINGVAR(tMotorDriverOvertempCurrent)
    FSTRINGVAR(tMotorDriverOvertempWarningCurrent)
    FSTRINGVAR(tMotorCurrentDecreasedTo)
    FSTRINGVAR(tMotorSpaceStealthChopColon)
    FSTRINGVAR(tMotorTempPrewarnTriggered)
    FSTRINGVAR(tMotorPrewarnFlagCleared)
    FSTRINGVAR(tMotorSpaceHybridTresholdColor)
    FSTRINGVAR(tMotorSpaceStallguardSensitivityColon)
    FSTRINGVAR(tMotorStallguardResult)
    FSTRINGVAR(tMotorSpaceRMSCurrentMAColon)

    static void cap(FSTRINGPARAM(text));
    static void config(FSTRINGPARAM(text));
    static void config(FSTRINGPARAM(text), int value);
    static void config(FSTRINGPARAM(text), const char* msg);
    static void config(FSTRINGPARAM(text), int32_t value);
    static void config(FSTRINGPARAM(text), uint32_t value);
    static void config(FSTRINGPARAM(text), float value, uint8_t digits = 2);
    static void printNumber(uint32_t n);
    static void printWarningF(FSTRINGPARAM(text));
    static void printInfoF(FSTRINGPARAM(text));
    static void printErrorF(FSTRINGPARAM(text));
    static void printWarningFLN(FSTRINGPARAM(text));
    static void printInfoFLN(FSTRINGPARAM(text));
    static void printErrorFLN(FSTRINGPARAM(text));
    static void printLogFLN(FSTRINGPARAM(text));
    static void printLogF(FSTRINGPARAM(text));
    static void printFLN(FSTRINGPARAM(text));
    static void printF(FSTRINGPARAM(text));
    static void printF(FSTRINGPARAM(text), bool value, BoolFormat format = BoolFormat::TRUEFALSE);
    static void printF(FSTRINGPARAM(text), int value);
    static void printF(FSTRINGPARAM(text), const char* msg);
    static void printF(FSTRINGPARAM(text), int32_t value);
    static void printF(FSTRINGPARAM(text), uint32_t value);
    static void printF(FSTRINGPARAM(text), float value, uint8_t digits = 2);
    static void printFLN(FSTRINGPARAM(text), bool value, BoolFormat format = BoolFormat::TRUEFALSE);
    static void printFLN(FSTRINGPARAM(text), int value);
    static void printFLN(FSTRINGPARAM(text), int32_t value);
    static void printFLN(FSTRINGPARAM(text), uint32_t value);
    static void printFLN(FSTRINGPARAM(text), const char* msg);
    static void printFLN(FSTRINGPARAM(text), float value, uint8_t digits = 2);
    static void printArrayFLN(FSTRINGPARAM(text), float* arr, uint8_t n = 4, uint8_t digits = 2);
    static void printArrayFLN(FSTRINGPARAM(text), long* arr, uint8_t n = 4);
    static void print(bool value, BoolFormat format = BoolFormat::TRUEFALSE);
    static void print(long value);
    static inline void print(uint32_t value) { printNumber(value); }
    static inline void print(int value) { print((int32_t)value); }
    static void print(const char* text);
    static inline void print(char c) { GCodeSource::writeToAll(c); }
    static void printFloat(float number, uint8_t digits);
    static inline void print(float number) { printFloat(number, 6); }
    static inline void println() {
        GCodeSource::writeToAll('\r');
        GCodeSource::writeToAll('\n');
    }
    static void promptStart(PromptDialogCallback cb, FSTRINGPARAM(text), bool blocking = true);
    static void promptStart(PromptDialogCallback cb, char* text, bool blocking = true);
    static void promptStart(PromptDialogCallback cb, FSTRINGPARAM(prefix), FSTRINGPARAM(text), bool blocking = true);
    static void promptStart(PromptDialogCallback cb, FSTRINGPARAM(prefix), char* text, bool blocking = true);
    static void promptEnd(bool blocking = true);
    static void promptButton(FSTRINGPARAM(text));
    static void promptShow();

    template <typename T>
    static void printBinaryFLN(FSTRINGPARAM(text), T n, bool grouping = false);

    static bool writeToAll;
#if FEATURE_CONTROLLER != NO_CONTROLLER
    static const char* translatedF(int textId);
    static void selectLanguage(fast8_t lang);
    static uint8_t selectedLanguage;
#endif
protected:
private:
};

template <typename T>
void Com::printBinaryFLN(FSTRINGPARAM(text), T n, bool grouping) {
    printF(text);
    size_t i = (sizeof(n) * 8u) - 1u;
    do {
        print(((1u << i) & n) ? '1' : '0');
        if (grouping && i && !(i % 8u)) {
            print(' ');
        }
    } while (i--);
    println();
}

#ifdef DEBUG
#define SHOW(x) \
    { \
        Com::printF(PSTR(" " #x "=")); \
        Com::print(x); \
        Com::println(); \
    }
#define SHOWS(x) \
    { \
        Com::printF(PSTR(" " #x "=")); \
        Com::print(x); \
        Com::print(" steps  "); \
        Com::print(x / 80); \
        Com::printFLN(PSTR(" mm")); \
    }
#define SHOWM(x) \
    { \
        Com::printF(PSTR(" " #x "=")); \
        Com::print((long)x * 80); \
        Com::print(" steps  "); \
        Com::print(x); \
        Com::printFLN(PSTR(" mm")); \
    }
#define SHOT(x) Com::printF(PSTR(x " "))
#define SHOWA(t, a, n) \
    { \
        SHOT(t); \
        for (int i = 0; i < n; i++) \
            SHOWS(a[i]); \
    }
#define SHOWAM(t, a, n) \
    { \
        SHOT(t); \
        for (int i = 0; i < n; i++) \
            SHOWM(a[i]); \
    }

#else
#define SHOW(x)
#define SHOT(x)
#define SHOWS(x)
#define SHOWM(x)
#define SHOWA(t, a, n)
#define SHOWAM(t, a, n)
#endif
