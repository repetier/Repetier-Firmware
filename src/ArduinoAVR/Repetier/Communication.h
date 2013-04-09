#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class Com
{
    public:
FSTRINGVAR(tOk)
FSTRINGVAR(tNAN)
FSTRINGVAR(tINF)
FSTRINGVAR(tError)
FSTRINGVAR(tInfo)
FSTRINGVAR(tWarning)
FSTRINGVAR(tResend)
FSTRINGVAR(tEcho)
FSTRINGVAR(tOkSpace)
FSTRINGVAR(tNoEEPROMSupport)
FSTRINGVAR(tWrongChecksum)
FSTRINGVAR(tMissingChecksum)
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
FSTRINGVAR(tSpaceYColon)
FSTRINGVAR(tSpaceZColon)
FSTRINGVAR(tSpaceEColon)
FSTRINGVAR(tTColon)
FSTRINGVAR(tSpaceBColon)
FSTRINGVAR(tSpaceAtColon)
FSTRINGVAR(tSpaceT)
FSTRINGVAR(tSpaceAt)
FSTRINGVAR(tColon)
FSTRINGVAR(tSpeedMultiply);
FSTRINGVAR(tFlowMultiply);
FSTRINGVAR(tFanspeed);
FSTRINGVAR(tPrintedFilament)
FSTRINGVAR(tPrintingTime)
FSTRINGVAR(tSpacem)
FSTRINGVAR(tSpaceDaysSpace)
FSTRINGVAR(tSpaceHoursSpace)
FSTRINGVAR(tSpaceMin)
FSTRINGVAR(tInvalidArc)
FSTRINGVAR(tComma)
FSTRINGVAR(tYColon)
FSTRINGVAR(tZColon)
FSTRINGVAR(tE0Colon)
FSTRINGVAR(tE1Colon)
FSTRINGVAR(tMS1MS2Pins)

#ifdef WAITING_IDENTIFIER
FSTRINGVAR(tWait)
#endif // WAITING_IDENTIFIER

#if EEPROM_MODE==0
FSTRINGVAR(tNoEEPROMSupport,"No EEPROM support compiled.\r\n")
#else
FSTRINGVAR(tConfigStoredEEPROM)
FSTRINGVAR(tConfigLoadedEEPROM)
FSTRINGVAR(tEPRConfigResetDefaults)
FSTRINGVAR(tEPRProtocolChanged)
FSTRINGVAR(tExtrDot)
FSTRINGVAR(tEPR0)
FSTRINGVAR(tEPR1)
FSTRINGVAR(tEPR2)
FSTRINGVAR(tEPR3)
FSTRINGVAR(tEPRBaudrate)
FSTRINGVAR(tEPRFilamentPrinted)
FSTRINGVAR(tEPRPrinterActive)
FSTRINGVAR(tEPRMaxInactiveTime)
FSTRINGVAR(tEPRStopAfterInactivty)
FSTRINGVAR(tEPRXStepsPerMM)
FSTRINGVAR(tEPRYStepsPerMM)
FSTRINGVAR(tEPRZStepsPerMM)
FSTRINGVAR(tEPRXMaxFeedrate)
FSTRINGVAR(tEPRYMaxFeedrate)
FSTRINGVAR(tEPRZMaxFeedrate)
FSTRINGVAR(tEPRXHomingFeedrate)
FSTRINGVAR(tEPRYHomingFeedrate)
FSTRINGVAR(tEPRZHomingFeedrate)
FSTRINGVAR(tEPRMaxJerk)
FSTRINGVAR(tEPRMaxZJerk)
FSTRINGVAR(tEPRXHomePos)
FSTRINGVAR(tEPRYHomePos)
FSTRINGVAR(tEPRZHomePos)
FSTRINGVAR(tEPRXMaxLength)
FSTRINGVAR(tEPRYMaxLength)
FSTRINGVAR(tEPRZMaxLength)
FSTRINGVAR(tEPRXBacklash)
FSTRINGVAR(tEPRYBacklash)
FSTRINGVAR(tEPRZBacklash)
FSTRINGVAR(tEPRXAcceleration)
FSTRINGVAR(tEPRYAcceleration)
FSTRINGVAR(tEPRZAcceleration)
FSTRINGVAR(tEPRXTravelAcceleration)
FSTRINGVAR(tEPRYTravelAcceleration)
FSTRINGVAR(tEPRZTravelAcceleration)
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
FSTRINGVAR(tEPRIGain)
FSTRINGVAR(tEPRDGain)
FSTRINGVAR(tEPRPIDMaxValue)
FSTRINGVAR(tEPRXOffset)
FSTRINGVAR(tEPRYOffset)
FSTRINGVAR(tEPRStabilizeTime)
FSTRINGVAR(tEPRRetractionWhenHeating)
FSTRINGVAR(tEPRDistanceRetractHeating)
FSTRINGVAR(tEPRExtruderCoolerSpeed)
FSTRINGVAR(tEPRAdvanceK)
FSTRINGVAR(tEPRAdvanceL)
#endif



static void printNumber(unsigned long n);
static void printWarningF(FSTRINGPARAM(text));
static void printInfoF(FSTRINGPARAM(text));
static void printErrorF(FSTRINGPARAM(text));
static void printWarningFLN(FSTRINGPARAM(text));
static void printInfoFLN(FSTRINGPARAM(text));
static void printErrorFLN(FSTRINGPARAM(text));
static void printFLN(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text),int value);
static void printF(FSTRINGPARAM(text),long value);
static void printF(FSTRINGPARAM(text),float value,byte digits=2);
static void printFLN(FSTRINGPARAM(text),int value);
static void printFLN(FSTRINGPARAM(text),long value);
static void print(long value);
static inline void print(int value) {print((long)value);}
static void print(const char *text);
static inline void print(char c) {HAL::serialWriteByte(c);}
static void printFloat(float number, byte digits);
static inline void println() {HAL::serialWriteByte('\r');HAL::serialWriteByte('\n');}
    protected:
    private:
};

#endif // COMMUNICATION_H
