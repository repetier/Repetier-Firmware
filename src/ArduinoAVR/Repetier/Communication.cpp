#include "Repetier.h"

FSTRINGVALUE(Com::tOk,"ok")
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
FSTRINGVALUE(Com::tStart,"start")
FSTRINGVALUE(Com::tPowerUp,"PowerUp")
FSTRINGVALUE(Com::tExternalReset,"External Reset")
FSTRINGVALUE(Com::tBrownOut,"Brown out Reset")
FSTRINGVALUE(Com::tWatchdog,"Watchdog Reset")
FSTRINGVALUE(Com::tSoftwareReset,"Software Reset")
FSTRINGVALUE(Com::tUnknownCommand,"Unknown command:")
FSTRINGVALUE(Com::tFreeRAM,"Free RAM:")

#ifdef WAITING_IDENTIFIER
FSTRINGVALUE(Com::tWait,WAITING_IDENTIFIER)
#endif // WAITING_IDENTIFIER
#if EEPROM_MODE==0
FSTRINGVALUE(Com::tNoEEPROMSupport,"No EEPROM support compiled.\r\n")
#else
FSTRINGVALUE(Com::tEPRConfigResetDefaults,"Configuration reset to defaults.")
FSTRINGVALUE(Com::tEPRProtocolChanged,"Protocol version changed, upgrading")
FSTRINGVALUE(Com::tExtrDot,"Extr.")
FSTRINGVALUE(Com::tEPR0,"EPR:0 ")
FSTRINGVALUE(Com::tEPR1,"EPR:1 ")
FSTRINGVALUE(Com::tEPR2,"EPR:2 ")
FSTRINGVALUE(Com::tEPR3,"EPR:3 ")
FSTRINGVALUE(Com::tEPRBaudrate,"Baudrate")
FSTRINGVALUE(Com::tEPRFilamentPrinted,"Filament printed [m]")
FSTRINGVALUE(Com::tEPRPrinterActive,"Printer active [s]")
FSTRINGVALUE(Com::tEPRMaxInactiveTime,"Max. inactive time [ms,0=off]")
FSTRINGVALUE(Com::tEPRStopAfterInactivty,"Stop stepper after inactivity [ms,0=off]")
FSTRINGVALUE(Com::tEPRXStepsPerMM,"X-axis steps per mm")
FSTRINGVALUE(Com::tEPRYStepsPerMM,"Y-axis steps per mm")
FSTRINGVALUE(Com::tEPRZStepsPerMM,"Z-axis steps per mm")
FSTRINGVALUE(Com::tEPRXMaxFeedrate,"X-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYMaxFeedrate,"Y-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZMaxFeedrate,"Z-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXHomingFeedrate,"X-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYHomingFeedrate,"Y-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZHomingFeedrate,"Z-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRMaxJerk,"Max. jerk [mm/s]")
FSTRINGVALUE(Com::tEPRMaxZJerk,"Max. Z-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRXHomePos,"X home pos [mm]")
FSTRINGVALUE(Com::tEPRYHomePos,"Y home pos [mm]")
FSTRINGVALUE(Com::tEPRZHomePos,"Z home pos [mm]")
FSTRINGVALUE(Com::tEPRXMaxLength,"X max length [mm]")
FSTRINGVALUE(Com::tEPRYMaxLength,"Y max length [mm]")
FSTRINGVALUE(Com::tEPRZMaxLength,"Z max length [mm]")
FSTRINGVALUE(Com::tEPRXBacklash,"X backlash [mm]")
FSTRINGVALUE(Com::tEPRYBacklash,"Y backlash [mm]")
FSTRINGVALUE(Com::tEPRZBacklash,"Z backlash [mm]")
FSTRINGVALUE(Com::tEPRXAcceleration,"X-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYAcceleration,"Y-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZAcceleration,"Z-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRXTravelAcceleration,"X-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYTravelAcceleration,"Y-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZTravelAcceleration,"Z-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPROPSMode,"OPS operation mode [0=Off,1=Classic,2=Fast]")
FSTRINGVALUE(Com::tEPROPSMoveAfter,"OPS move after x% retract [%]")
FSTRINGVALUE(Com::tEPROPSMinDistance,"OPS min. distance for fil. retraction [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionLength,"OPS retraction length [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionBacklash,"OPS retraction backlash [mm]")
FSTRINGVALUE(Com::tEPRBedHeatManager,"Bed Heat Manager [0-2]")
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
FSTRINGVALUE(Com::tEPRHeatManager,"heat manager [0-1]")
FSTRINGVALUE(Com::tEPRDriveMax,"PID drive max")
FSTRINGVALUE(Com::tEPRDriveMin,"PID drive min")
FSTRINGVALUE(Com::tEPRPGain,"PID P-gain")
FSTRINGVALUE(Com::tEPRIGain,"PID I-gain")
FSTRINGVALUE(Com::tEPRDGain,"PID D-gain")
FSTRINGVALUE(Com::tEPRPIDMaxValue,"PID max value [0-255]")
FSTRINGVALUE(Com::tEPRXOffset,"X-offset [steps]")
FSTRINGVALUE(Com::tEPRYOffset,"Y-offset [steps]")
FSTRINGVALUE(Com::tEPRStabilizeTime,"temp. stabilize time [s]")
FSTRINGVALUE(Com::tEPRRetractionWhenHeating,"temp. for retraction when heating [C]")
FSTRINGVALUE(Com::tEPRDistanceRetractHeating,"distance to retract when heating [mm]")
FSTRINGVALUE(Com::tEPRExtruderCoolerSpeed,"extruder cooler speed [0-255]")
FSTRINGVALUE(Com::tEPRAdvanceK,"advance K [0=off]")
FSTRINGVALUE(Com::tEPRAdvanceL,"advance L [0=off]")

#endif


void Com::printWarningF(FSTRINGPARAM(text)) {
    printF(tWarning);
    printF(text);
}
void Com::printWarningFLN(FSTRINGPARAM(text)) {
    printF(tWarning);
    printFLN(text);
}
void Com::printInfoF(FSTRINGPARAM(text)) {
    printF(tInfo);
    printF(text);
}
void Com::printInfoFLN(FSTRINGPARAM(text)) {
    printF(tInfo);
    printFLN(text);
}

void Com::printErrorF(FSTRINGPARAM(text)) {
    printF(tError);
    printF(text);
}
void Com::printErrorFLN(FSTRINGPARAM(text)) {
    printF(tError);
    printF(text);
}
void Com::printFLN(FSTRINGPARAM(text)) {
    printF(text);
    println();
}

void Com::printF(FSTRINGPARAM(ptr)) {
  char c;
  while ((c=HAL::readFlashByte(ptr++)) != 0)
     HAL::serialWriteByte(c);
}
void Com::printF(FSTRINGPARAM(text),int value) {
    printF(text);
    print(value);
}
void Com::printF(FSTRINGPARAM(text),long value) {
    printF(text);
    print(value);
}
void Com::printFLN(FSTRINGPARAM(text),int value) {
    printF(text);
    print(value);
    println();
}
void Com::printFLN(FSTRINGPARAM(text),long value) {
    printF(text);
    print(value);
    println();
}
void Com::printF(FSTRINGPARAM(text),float value,byte digits) {
    printF(text);
    printFloat(value,digits);
}

void Com::print(const char *text) {
  while(*text) {
    HAL::serialWriteByte(*text++);
  }
}
void Com::print(long value) {
    if(value<0) {
        HAL::serialWriteByte('-');
        value = -value;
    }
    printNumber(value);
}

void Com::printNumber(unsigned long n) {
  char buf[11]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[10];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    *--str = '0'+(m - 10 * n);
  } while(n);

  print(str);
}
void Com::printFloat(float number, uint8_t digits)
{
  if (isnan(number)) {
	printF(tNAN);
    return;
  }
  if (isinf(number)) {
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
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
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
}
