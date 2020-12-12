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

#include "Repetier.h"

#if FEATURE_CONTROLLER != NO_CONTROLLER
uint8_t Com::selectedLanguage;
#endif

#ifndef MACHINE_TYPE
#define MACHINE_TYPE "Unknown"
#endif
#ifndef FIRMWARE_URL
#define FIRMWARE_URL "https://github.com/repetier/Repetier-Firmware/"
#endif // FIRMWARE_URL

FSTRINGVALUE(Com::tFirmware, "FIRMWARE_NAME:Repetier_" REPETIER_VERSION " COMPILED:" __DATE__ " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:1.0 MACHINE_TYPE:" MACHINE_TYPE " EXTRUDER_COUNT:" XSTR(NUM_EXTRUDER) " REPETIER_PROTOCOL:4")
FSTRINGVALUE(Com::tEmpty, "")
FSTRINGVALUE(Com::tFirmwareVersion, REPETIER_VERSION)
FSTRINGVALUE(Com::tFirmwareCompiled, "Compiled:" __DATE__)
FSTRINGVALUE(Com::tVendor, UI_PRINTER_COMPANY)
FSTRINGVALUE(Com::tPrinterName, UI_PRINTER_NAME)
FSTRINGVALUE(Com::tDebug, "Debug:")
FSTRINGVALUE(Com::tOk, "ok")
FSTRINGVALUE(Com::tNewline, "\r\n")
FSTRINGVALUE(Com::tNAN, "NAN")
FSTRINGVALUE(Com::tINF, "INF")
FSTRINGVALUE(Com::tError, "Error:")
FSTRINGVALUE(Com::tLog, "Log:")
FSTRINGVALUE(Com::tInfo, "Info:")
FSTRINGVALUE(Com::tWarning, "Warning:")
FSTRINGVALUE(Com::tResend, "Resend:")
FSTRINGVALUE(Com::tEcho, "Echo:")
FSTRINGVALUE(Com::tCap, "Cap:")
FSTRINGVALUE(Com::tOkSpace, "ok ")
FSTRINGVALUE(Com::tWrongChecksum, "Wrong checksum")
FSTRINGVALUE(Com::tMissingChecksum, "Missing checksum")
FSTRINGVALUE(Com::tFormatError, "Format error")
FSTRINGVALUE(Com::tDonePrinting, "Done printing file")
FSTRINGVALUE(Com::tXAxis, "X axis")
FSTRINGVALUE(Com::tYAxis, "Y axis")
FSTRINGVALUE(Com::tZAxis, "Z axis")
FSTRINGVALUE(Com::tEAxis, "E axis")
FSTRINGVALUE(Com::tAAxis, "A axis")
FSTRINGVALUE(Com::tBAxis, "B axis")
FSTRINGVALUE(Com::tCAxis, "C axis")
FSTRINGVALUE(Com::tX, " X")
FSTRINGVALUE(Com::tY, " Y")
FSTRINGVALUE(Com::tZ, " Z")
FSTRINGVALUE(Com::tE, " E")
FSTRINGVALUE(Com::tF, " F")
FSTRINGVALUE(Com::tS, " S")
FSTRINGVALUE(Com::tP, " P")
FSTRINGVALUE(Com::tI, " I")
FSTRINGVALUE(Com::tJ, " J")
FSTRINGVALUE(Com::tR, " R")
FSTRINGVALUE(Com::tD, " D")
FSTRINGVALUE(Com::tC, " C")
FSTRINGVALUE(Com::tH, " H")
FSTRINGVALUE(Com::tA, " A")
FSTRINGVALUE(Com::tB, " B")
FSTRINGVALUE(Com::tK, " K")
FSTRINGVALUE(Com::tL, " L")
FSTRINGVALUE(Com::tO, " O")
FSTRINGVALUE(Com::tU, " U")
FSTRINGVALUE(Com::tV, " V")
FSTRINGVALUE(Com::tW, " W")
FSTRINGVALUE(Com::tExpectedLine, "Error:expected line ")
FSTRINGVALUE(Com::tGot, " got ")
FSTRINGVALUE(Com::tSkip, "skip ")
FSTRINGVALUE(Com::tBLK, "BLK ")
FSTRINGVALUE(Com::tStart, "start")
FSTRINGVALUE(Com::tPowerUp, "PowerUp")
FSTRINGVALUE(Com::tExternalReset, "External Reset")
FSTRINGVALUE(Com::tBrownOut, "Brown out Reset")
FSTRINGVALUE(Com::tWatchdog, "Watchdog Reset")
FSTRINGVALUE(Com::tSoftwareReset, "Software Reset")
FSTRINGVALUE(Com::tUnknownCommand, "Unknown command:")
FSTRINGVALUE(Com::tFreeRAM, "Free RAM:")
FSTRINGVALUE(Com::tXColon, "X:")
FSTRINGVALUE(Com::tSpaceXColon, " X:")
FSTRINGVALUE(Com::tSpaceYColon, " Y:")
FSTRINGVALUE(Com::tSpaceZColon, " Z:")
FSTRINGVALUE(Com::tSpaceEColon, " E:")
FSTRINGVALUE(Com::tSpaceAColon, " A:")
FSTRINGVALUE(Com::tSpaceCColon, " C:")
FSTRINGVALUE(Com::tTColon, "T:")
FSTRINGVALUE(Com::tSpaceBColon, " B:")
FSTRINGVALUE(Com::tSpaceAtColon, " @:")
FSTRINGVALUE(Com::tSpaceT, " T")
FSTRINGVALUE(Com::tSpaceAt, " @")
FSTRINGVALUE(Com::tSpaceBAtColon, " B@:")
FSTRINGVALUE(Com::tSpaceRaw, " RAW")
FSTRINGVALUE(Com::tColon, ":")
FSTRINGVALUE(Com::tSlash, "/")
FSTRINGVALUE(Com::tSpaceSlash, " /")
FSTRINGVALUE(Com::tFatal, "fatal:")
FSTRINGVALUE(Com::tDoorOpen, "Door open")
FSTRINGVALUE(Com::tBtnOK, " OK ")
FSTRINGVALUE(Com::tUnitMM, "mm")
FSTRINGVALUE(Com::tUnitMMPS, "mm/s")
FSTRINGVALUE(Com::tUnitMMPS2, "mm/s^2")
FSTRINGVALUE(Com::tUnitMMPS3, "mm/s^3")
FSTRINGVALUE(Com::tUnitBaud, "baud")
FSTRINGVALUE(Com::tUnitPercent, "%")
FSTRINGVALUE(Com::tUnitDegCelsius, "°C")
FSTRINGVALUE(Com::tUnitStepsPerMM, "steps/mm")
FSTRINGVALUE(Com::tUnitSteps, "steps")
FSTRINGVALUE(Com::tUnitSeconds, "s")
FSTRINGVALUE(Com::tUnitMilliSeconds, "ms")
FSTRINGVALUE(Com::tUnitMilliWatt, "mW")
FSTRINGVALUE(Com::tUnitPWM, "0-255")
FSTRINGVALUE(Com::tUnitRPM, "1/min")
FSTRINGVALUE(Com::tM999, "Fail mode active. Send M999 to disable failed mode!")
FSTRINGVALUE(Com::tTestM999, "Testing fatal error")
FSTRINGVALUE(Com::tColdExtrusionPrevented, "Cold extrusion prevented")
#if JSON_OUTPUT
FSTRINGVALUE(Com::tJSONDir, "{\"dir\":\"")
FSTRINGVALUE(Com::tJSONFiles, "\",\"files\":[")
FSTRINGVALUE(Com::tJSONArrayEnd, "]}")
FSTRINGVALUE(Com::tJSONErrorStart, "{\"err\":\"")
FSTRINGVALUE(Com::tJSONErrorEnd, "\"}")
FSTRINGVALUE(Com::tJSONFileInfoStart, "{\"err\":0,\"size\":");
FSTRINGVALUE(Com::tJSONFileInfoHeight, ",\"height\":");
FSTRINGVALUE(Com::tJSONFileInfoLayerHeight, ",\"layerHeight\":");
FSTRINGVALUE(Com::tJSONFileInfoFilament, ",\"filament\":[");
FSTRINGVALUE(Com::tJSONFileInfoGeneratedBy, "],\"generatedBy\":\"");
FSTRINGVALUE(Com::tJSONFileInfoName, ",\"fileName\":\"");
#endif // JSON_OUTPUT
FSTRINGVALUE(Com::tSpeedMultiply, "SpeedMultiply:")
FSTRINGVALUE(Com::tFlowMultiply, "FlowMultiply:")
FSTRINGVALUE(Com::tPrintedFilament, "Printed filament:")
FSTRINGVALUE(Com::tPrintingTime, "Printing time:")
FSTRINGVALUE(Com::tSpacem, "m ")
FSTRINGVALUE(Com::tSpaceDaysSpace, " days ")
FSTRINGVALUE(Com::tSpaceHoursSpace, " hours ")
FSTRINGVALUE(Com::tSpaceMin, " min")
FSTRINGVALUE(Com::tInvalidArc, "Invalid arc")
FSTRINGVALUE(Com::tComma, ",")
FSTRINGVALUE(Com::tSpace, " ")
FSTRINGVALUE(Com::tYColon, "Y:")
FSTRINGVALUE(Com::tZColon, "Z:")
FSTRINGVALUE(Com::tE0Colon, "E0:")
FSTRINGVALUE(Com::tE1Colon, "E1:")
FSTRINGVALUE(Com::tMS1MS2Pins, "MS1,MS2 Pins")
FSTRINGVALUE(Com::tSetOutputSpace, "Set output: ")
FSTRINGVALUE(Com::tGetInputSpace, "Get Input: ")
FSTRINGVALUE(Com::tSpaceToSpace, " to ")
FSTRINGVALUE(Com::tSpaceIsSpace, " is ")
FSTRINGVALUE(Com::tHSpace, "H ")
FSTRINGVALUE(Com::tLSpace, "L ")
FSTRINGVALUE(Com::tXMinColon, "x_min:")
FSTRINGVALUE(Com::tXMaxColon, "x_max:")
FSTRINGVALUE(Com::tYMinColon, "y_min:")
FSTRINGVALUE(Com::tYMaxColon, "y_max:")
FSTRINGVALUE(Com::tZMinColon, "z_min:")
FSTRINGVALUE(Com::tZMaxColon, "z_max:")
FSTRINGVALUE(Com::tAMinColon, "a_min:")
FSTRINGVALUE(Com::tAMaxColon, "a_max:")
FSTRINGVALUE(Com::tBMinColon, "b_min:")
FSTRINGVALUE(Com::tBMaxColon, "b_max:")
FSTRINGVALUE(Com::tCMinColon, "c_min:")
FSTRINGVALUE(Com::tCMaxColon, "c_max:")
FSTRINGVALUE(Com::tZ2MinMaxColon, "z2_minmax:")
FSTRINGVALUE(Com::tJerkColon, "Jerk:")
FSTRINGVALUE(Com::tZJerkColon, " ZJerk:")
FSTRINGVALUE(Com::tLinearStepsColon, " linear steps:")
FSTRINGVALUE(Com::tQuadraticStepsColon, " quadratic steps:")
FSTRINGVALUE(Com::tCommaSpeedEqual, ", speed=")
FSTRINGVALUE(Com::tEEPROMUpdated, "EEPROM updated")

FSTRINGVALUE(Com::tLinearLColon, "linear L:")
FSTRINGVALUE(Com::tQuadraticKColon, " quadratic K:")
FSTRINGVALUE(Com::tFilamentSlipping, "Filament slipping")
FSTRINGVALUE(Com::tPauseCommunication, "// action:pause")
FSTRINGVALUE(Com::tContinueCommunication, "// action:resume")
FSTRINGVALUE(Com::tMeasurementReset, "Measurement reset.")
FSTRINGVALUE(Com::tMeasureDeltaSteps, "Measure/delta (Steps) =")
FSTRINGVALUE(Com::tMeasureDelta, "Measure/delta =")
FSTRINGVALUE(Com::tMeasureOriginReset, "Measured origin set. Measurement reset.")
FSTRINGVALUE(Com::tMeasurementAbortedOrigin, "Origin measurement cannot be set.  Use only Z-Cartesian (straight up and down) movements and try again.")
FSTRINGVALUE(Com::tLevelingCalc, "Leveling calc:")
FSTRINGVALUE(Com::tTower1, "Tower 1:")
FSTRINGVALUE(Com::tTower2, "Tower 2:")
FSTRINGVALUE(Com::tTower3, "Tower 3:")
FSTRINGVALUE(Com::tDeltaAlphaA, "Alpha A(210):")
FSTRINGVALUE(Com::tDeltaAlphaB, "Alpha B(330):")
FSTRINGVALUE(Com::tDeltaAlphaC, "Alpha C(90):")
FSTRINGVALUE(Com::tDeltaRadiusCorrectionA, "Delta Radius A(0):")
FSTRINGVALUE(Com::tDeltaRadiusCorrectionB, "Delta Radius B(0):")
FSTRINGVALUE(Com::tDeltaRadiusCorrectionC, "Delta Radius C(0):")
FSTRINGVALUE(Com::tInvalidDeltaCoordinate, "Invalid delta coordinate - move ignored")
FSTRINGVALUE(Com::tDBGDeltaNoMoveinDSegment, "No move in delta segment with > 1 segment. This should never happen and may cause a problem!")

#ifdef DEBUG_GENERIC
FSTRINGVALUE(Com::tGenTemp, "GenTemp:")
#endif // DEBUG_GENERICFSTRINGVALUE(Com::,"")
FSTRINGVALUE(Com::tTargetExtr, "TargetExtr")
FSTRINGVALUE(Com::tTargetBedColon, "TargetBed:")
FSTRINGVALUE(Com::tPIDAutotuneStart, "PID Autotune start")
FSTRINGVALUE(Com::tAPIDBias, " bias: ")
FSTRINGVALUE(Com::tAPIDD, " d: ")
FSTRINGVALUE(Com::tAPIDMin, " min: ")
FSTRINGVALUE(Com::tAPIDMax, " max: ")
FSTRINGVALUE(Com::tAPIDKu, " Ku: ")
FSTRINGVALUE(Com::tAPIDTu, " Tu: ")
FSTRINGVALUE(Com::tAPIDClassic, " Classic PID")
FSTRINGVALUE(Com::tAPIDSome, " Some Overshoot PID")
FSTRINGVALUE(Com::tAPIDNone, " No Overshoot PID")
FSTRINGVALUE(Com::tAPIDPessen, " Pessen Integral Rule PID")
FSTRINGVALUE(Com::tAPIDTyreusLyben, " Tyreus-Lyben PID")
FSTRINGVALUE(Com::tAPIDKp, " Kp: ")
FSTRINGVALUE(Com::tAPIDKi, " Ki: ")
FSTRINGVALUE(Com::tAPIDKd, " Kd: ")
FSTRINGVALUE(Com::tAPIDFailedHigh, "PID Autotune failed! Temperature too high")
FSTRINGVALUE(Com::tAPIDFailedTimeout, "PID Autotune failed! timeout")
FSTRINGVALUE(Com::tAPIDFinished, "PID Autotune finished ! Place the Kp, Ki and Kd constants in the Configuration.h or EEPROM")
FSTRINGVALUE(Com::tMTEMPColon, "MTEMP:")
FSTRINGVALUE(Com::tHeatedBed, "heated bed")
FSTRINGVALUE(Com::tExtruderSpace, "extruder ")
FSTRINGVALUE(Com::tTempSensorDefect, ": temp sensor defect")
FSTRINGVALUE(Com::tTempSensorWorking, ": working")
FSTRINGVALUE(Com::tDryModeUntilRestart, "Printer set into dry run mode until restart!")
#ifdef DEBUG_QUEUE_MOVE
FSTRINGVALUE(Com::tDBGId, "ID:")
FSTRINGVALUE(Com::tDBGVStartEnd, "vStart/End:")
FSTRINGVALUE(Com::tDBAccelSteps, "accel/decel steps:")
FSTRINGVALUE(Com::tDBGStartEndSpeed, "st./end speed:")
FSTRINGVALUE(Com::tDBGFlags, "Flags:")
FSTRINGVALUE(Com::tDBGJoinFlags, "joinFlags:")
FSTRINGVALUE(Com::tDBGDelta, "Delta")
FSTRINGVALUE(Com::tDBGDir, "Dir:")
FSTRINGVALUE(Com::tDBGFullSpeed, "fullSpeed:")
FSTRINGVALUE(Com::tDBGVMax, "vMax:")
FSTRINGVALUE(Com::tDBGAcceleration, "Acceleration:")
FSTRINGVALUE(Com::tDBGAccelerationPrim, "Acceleration Prim:")
FSTRINGVALUE(Com::tDBGRemainingSteps, "Remaining steps:")
FSTRINGVALUE(Com::tDBGAdvanceFull, "advanceFull:")
FSTRINGVALUE(Com::tDBGAdvanceRate, "advanceRate:")
FSTRINGVALUE(Com::tDBGLimitInterval, "LimitInterval:")
FSTRINGVALUE(Com::tDBGMoveDistance, "Move distance on the XYZ space:")
FSTRINGVALUE(Com::tDBGCommandedFeedrate, "Commanded feedrate:")
FSTRINGVALUE(Com::tDBGConstFullSpeedMoveTime, "Constant full speed move time:")
#endif // DEBUG_QUEUE_MOVEFSTRINGVALUE(Com::,"")
#ifdef DEBUG_DELTA_OVERFLOW
FSTRINGVALUE(Com::tDBGDeltaOverflow, "Delta overflow:")
#endif // DEBUG_DELTA_OVERFLOW
#ifdef DEBUG_SPLIT
FSTRINGVALUE(Com::tDBGDeltaSeconds, "Seconds:")
FSTRINGVALUE(Com::tDBGDeltaZDelta, "Z delta:")
FSTRINGVALUE(Com::tDBGDeltaSegments, "Segments:")
FSTRINGVALUE(Com::tDBGDeltaNumLines, "Num lines:")
FSTRINGVALUE(Com::tDBGDeltaSegmentsPerLine, "segments_per_line:")
FSTRINGVALUE(Com::tDBGDeltaMaxDS, "Max DS:")
FSTRINGVALUE(Com::tDBGDeltaStepsPerSegment, "Steps Per Segment:")
FSTRINGVALUE(Com::tDBGDeltaVirtualAxisSteps, "Virtual axis steps:")
#endif
#ifdef DEBUG_STEPCOUNT
FSTRINGVALUE(Com::tDBGMissedSteps, "Missed steps:")
#endif // DEBUG_STEPCOUNT
FSTRINGVALUE(Com::tProbing, "Probing")
FSTRINGVALUE(Com::tZProbe, "Z-probe:")
FSTRINGVALUE(Com::tZProbeAverage, "Z-probe average height:")
FSTRINGVALUE(Com::tZProbeZReset, "Reset Z height")
FSTRINGVALUE(Com::tZProbeStartScript, Z_PROBE_START_SCRIPT)
FSTRINGVALUE(Com::tZProbeEndScript, Z_PROBE_FINISHED_SCRIPT)
FSTRINGVALUE(Com::tHitZProbe, "Hit z-probe")
FSTRINGVALUE(Com::tZProbeState, "Z-probe state:")
FSTRINGVALUE(Com::tAutolevelReset, "Autolevel matrix reset")
FSTRINGVALUE(Com::tAutolevelEnabled, "Autoleveling enabled")
FSTRINGVALUE(Com::tAutolevelDisabled, "Autoleveling disabled")
FSTRINGVALUE(Com::tBumpCSVHeader, "Repetier V2 Bump-Matrix File")
FSTRINGVALUE(Com::tErrorImportBump, "Error importing bump matrix:")
FSTRINGVALUE(Com::tErrorExportBump, "Error exporting bump matrix:")
FSTRINGVALUE(Com::tNoGridLeveling, "Not compiled with grid-based leveling.")
FSTRINGVALUE(Com::tNoDistortionData, "No distortion matrix data stored!")
FSTRINGVALUE(Com::tTransformationMatrix, "Transformation matrix:")
FSTRINGVALUE(Com::tZProbeFailed, "Z-probe failed")
FSTRINGVALUE(Com::tZProbeMax, "Z-probe max:")
FSTRINGVALUE(Com::tZProbePrinterHeight, "Printer height:")
//FSTRINGVALUE(Com::,"")
FSTRINGVALUE(Com::tWait, WAITING_IDENTIFIER)
FSTRINGVALUE(Com::tNoEEPROMSupport, "No EEPROM support compiled.\r\n")
FSTRINGVALUE(Com::tZProbeOffsetZ, "Coating thickness [mm]")
#if FEATURE_AXISCOMP
FSTRINGVALUE(Com::tAxisCompTanXY, "tanXY Axis Compensation")
FSTRINGVALUE(Com::tAxisCompTanYZ, "tanYZ Axis Compensation")
FSTRINGVALUE(Com::tAxisCompTanXZ, "tanXZ Axis Compensation")
#endif

FSTRINGVALUE(Com::tConfigStoredEEPROM, "Configuration stored to EEPROM.")
FSTRINGVALUE(Com::tConfigLoadedEEPROM, "Configuration loaded from EEPROM.")
FSTRINGVALUE(Com::tEPRConfigResetDefaults, "Configuration reset to defaults.")
FSTRINGVALUE(Com::tEPRProtocolChanged, "Protocol version changed, upgrading")
FSTRINGVALUE(Com::tEPR0, "EPR:0 ")
FSTRINGVALUE(Com::tEPR1, "EPR:1 ")
FSTRINGVALUE(Com::tEPR2, "EPR:2 ")
FSTRINGVALUE(Com::tEPR3, "EPR:3 ")
FSTRINGVALUE(Com::tEPRBaudrate, "Baudrate [baud]")
FSTRINGVALUE(Com::tLanguage, "Language")
FSTRINGVALUE(Com::tEPRFilamentPrinted, "Filament printed [m]")
FSTRINGVALUE(Com::tEPRPrinterActive, "Printer active [s]")
FSTRINGVALUE(Com::tEPRMaxInactiveTime, "Max. inactive time [ms,0=off]")
FSTRINGVALUE(Com::tEPRStopAfterInactivty, "Stop stepper after inactivity [ms,0=off]")
FSTRINGVALUE(Com::tEPRXHomePos, "X min pos [mm]")
FSTRINGVALUE(Com::tEPRYHomePos, "Y min pos [mm]")
FSTRINGVALUE(Com::tEPRZHomePos, "Z min pos [mm]")
FSTRINGVALUE(Com::tEPRXMaxLength, "X max length [mm]")
FSTRINGVALUE(Com::tEPRYMaxLength, "Y max length [mm]")
FSTRINGVALUE(Com::tEPRZMaxLength, "Z max length [mm]")
FSTRINGVALUE(Com::tEPRXBacklash, "X backlash [mm]")
FSTRINGVALUE(Com::tEPRYBacklash, "Y backlash [mm]")
FSTRINGVALUE(Com::tEPRZBacklash, "Z backlash [mm]")
FSTRINGVALUE(Com::tEPRMaxJerk, "Max. jerk [mm/s]")
FSTRINGVALUE(Com::tEPRAccelerationFactorAtTop, "Acceleration factor at top [%,100=like bottom]")
// FSTRINGVALUE(Com::tEPRZAcceleration, "Acceleration [mm/s^2]")
// FSTRINGVALUE(Com::tEPRZTravelAcceleration, "Travel acceleration [mm/s^2]")
// FSTRINGVALUE(Com::tEPRZStepsPerMM, "Steps per mm")
// FSTRINGVALUE(Com::tEPRZMaxFeedrate, "Max. feedrate [mm/s]")
// FSTRINGVALUE(Com::tEPRZHomingFeedrate, "Homing feedrate [mm/s]")

FSTRINGVALUE(Com::tEPRDiagonalRodLength, "Diagonal rod length [mm]")
FSTRINGVALUE(Com::tEPRHorizontalRadius, "Horizontal rod radius at 0,0 [mm]")

FSTRINGVALUE(Com::tEPRTowerXOffset, "Tower X endstop offset [steps]")
FSTRINGVALUE(Com::tEPRTowerYOffset, "Tower Y endstop offset [steps]")
FSTRINGVALUE(Com::tEPRTowerZOffset, "Tower Z endstop offset [steps]")

FSTRINGVALUE(Com::tEPRDeltaMaxRadius, "Max printable radius [mm]")
FSTRINGVALUE(Com::tDeltaDiagonalCorrectionA, "Corr. diagonal A [mm]")
FSTRINGVALUE(Com::tDeltaDiagonalCorrectionB, "Corr. diagonal B [mm]")
FSTRINGVALUE(Com::tDeltaDiagonalCorrectionC, "Corr. diagonal C [mm]")

FSTRINGVALUE(Com::tEPRMaxZJerk, "Max. Z-jerk [mm/s]")
FSTRINGVALUE(Com::tEPRXStepsPerMM, "X-axis steps per mm")
FSTRINGVALUE(Com::tEPRYStepsPerMM, "Y-axis steps per mm")
FSTRINGVALUE(Com::tEPRZStepsPerMM, "Z-axis steps per mm")
FSTRINGVALUE(Com::tEPRXMaxFeedrate, "X-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYMaxFeedrate, "Y-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZMaxFeedrate, "Z-axis max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXHomingFeedrate, "X-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRYHomingFeedrate, "Y-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRZHomingFeedrate, "Z-axis homing feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRXAcceleration, "X-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYAcceleration, "Y-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZAcceleration, "Z-axis acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRXTravelAcceleration, "X-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRYTravelAcceleration, "Y-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRZTravelAcceleration, "Z-axis travel acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPROPSMode, "OPS operation mode [0=Off,1=Classic,2=Fast]")
FSTRINGVALUE(Com::tEPROPSMoveAfter, "OPS move after x% retract [%]")
FSTRINGVALUE(Com::tEPROPSMinDistance, "OPS min. distance for fil. retraction [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionLength, "OPS retraction length [mm]")
FSTRINGVALUE(Com::tEPROPSRetractionBacklash, "OPS retraction backlash [mm]")
FSTRINGVALUE(Com::tEPRBedHeatManager, "Bed Heat Manager [0-3]")
FSTRINGVALUE(Com::tEPRBedPIDDriveMax, "Bed PID drive max")
FSTRINGVALUE(Com::tEPRBedPIDDriveMin, "Bed PID drive min")
FSTRINGVALUE(Com::tEPRBedPGain, "Bed PID P-gain")
FSTRINGVALUE(Com::tEPRBedIGain, "Bed PID I-gain")
FSTRINGVALUE(Com::tEPRBedDGain, "Bed PID D-gain")
FSTRINGVALUE(Com::tEPRBedPISMaxValue, "Bed PID max value [0-255]")
FSTRINGVALUE(Com::tEPRStepsPerMM, "steps per mm [steps/mm]")
FSTRINGVALUE(Com::tEPRMaxFeedrate, "max. feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRStartFeedrate, "start feedrate [mm/s]")
FSTRINGVALUE(Com::tEPRAcceleration, "acceleration [mm/s^2]")
FSTRINGVALUE(Com::tEPRHeatManager, "heat manager [0-3]")
FSTRINGVALUE(Com::tEPRDriveMax, "PID drive max")
FSTRINGVALUE(Com::tEPRDriveMin, "PID drive min")
FSTRINGVALUE(Com::tEPRPGain, "PID P-gain/dead-time")
FSTRINGVALUE(Com::tEPRDead, "Heater dead-time")
FSTRINGVALUE(Com::tEPRUnused, "na for dead time ctrl")
FSTRINGVALUE(Com::tEPRIGain, "PID I-gain")
FSTRINGVALUE(Com::tEPRDGain, "PID D-gain")
FSTRINGVALUE(Com::tEPRPIDMaxValue, "PID max value [0-255]")
FSTRINGVALUE(Com::tEPRXOffset, "X-offset [steps]")
FSTRINGVALUE(Com::tEPRYOffset, "Y-offset [steps]")
FSTRINGVALUE(Com::tEPRZOffset, "Z-offset [steps]")
FSTRINGVALUE(Com::tEPRStabilizeTime, "temp. stabilize time [s]")
FSTRINGVALUE(Com::tEPRRetractionWhenHeating, "temp. for retraction when heating [C]")
FSTRINGVALUE(Com::tEPRDistanceRetractHeating, "distance to retract when heating [mm]")
FSTRINGVALUE(Com::tEPRExtruderCoolerSpeed, "extruder cooler speed [0-255]")
FSTRINGVALUE(Com::tEPRAdvanceK, "advance K [0=off]")
FSTRINGVALUE(Com::tEPRAdvanceL, "advance L [0=off]")
FSTRINGVALUE(Com::tEPRPreheatTemp, "Preheat temp. [�C]")
FSTRINGVALUE(Com::tEPRPreheatBedTemp, "Bed Preheat temp. [�C]")
#if NUM_BEEPERS > 0
FSTRINGVALUE(Com::tEPRToneVolume, "Tone volume [%]")
#endif
//FSTRINGVALUE(Com::tSDRemoved,UI_TEXT_SD_REMOVED)
//FSTRINGVALUE(Com::tSDInserted,UI_TEXT_SD_INSERTED)
FSTRINGVALUE(Com::tSDInitFail, "SD init fail")
FSTRINGVALUE(Com::tSDReadError, "SD read error")
FSTRINGVALUE(Com::tErrorWritingToFile, "Error writing to file")
FSTRINGVALUE(Com::tBeginFileList, "Begin file list")
FSTRINGVALUE(Com::tEndFileList, "End file list")
FSTRINGVALUE(Com::tFileOpened, "File opened:")
FSTRINGVALUE(Com::tSpaceSizeColon, " Size:")
FSTRINGVALUE(Com::tFileSelected, "File selected")
FSTRINGVALUE(Com::tFileOpenFailed, "File open failed")
FSTRINGVALUE(Com::tInvalidFiletype, "Invalid file type")
FSTRINGVALUE(Com::tInvalidFilename, "Invalid file name")
FSTRINGVALUE(Com::tNoMountedCard, "No SD card mounted")
FSTRINGVALUE(Com::tSDPrintingByte, "SD printing byte ")
FSTRINGVALUE(Com::tNotSDPrinting, "Not SD printing")
FSTRINGVALUE(Com::tCurrentOpenFile, "Current file: ") // Compat with hosts that use M27 C
FSTRINGVALUE(Com::tOpenFailedFile, "open failed, File: ")
FSTRINGVALUE(Com::tWritingToFile, "Writing to file: ")
FSTRINGVALUE(Com::tDoneSavingFile, "Done saving file.")
FSTRINGVALUE(Com::tFileDeleted, "File deleted")
FSTRINGVALUE(Com::tDeletionFailed, "Deletion failed")
FSTRINGVALUE(Com::tDirectoryCreated, "Directory created")
FSTRINGVALUE(Com::tCreationFailed, "Creation failed")
FSTRINGVALUE(Com::tSDErrorCode, "SD errorCode:")
FSTRINGVALUE(Com::tHeaterDecoupled, "Heater decoupled")
FSTRINGVALUE(Com::tHeaterDecoupledWarning, "One heater seems decoupled from thermistor - disabling all for safety!")
#if FEATURE_RETRACTION
FSTRINGVALUE(Com::tEPRAutoretractEnabled, "Enable retraction conversion [0/1]")
FSTRINGVALUE(Com::tEPRRetractionLength, "Retraction length [mm]")
FSTRINGVALUE(Com::tEPRRetractionSpeed, "Retraction speed [mm/s]")
FSTRINGVALUE(Com::tEPRRetractionZLift, "Retraction z-lift [mm]")
FSTRINGVALUE(Com::tEPRRetractionUndoExtraLength, "Extra extrusion on undo retract [mm]")
FSTRINGVALUE(Com::tEPRRetractionLongLength, "Retraction length extruder switch [mm]")
FSTRINGVALUE(Com::tEPRRetractionUndoExtraLongLength, "Extra extrusion on undo switch retract [mm]")
FSTRINGVALUE(Com::tEPRRetractionUndoSpeed, "Retraction undo speed [mm/s]")
#endif
FSTRINGVALUE(Com::tConfig, "Config:")
FSTRINGVALUE(Com::tExtrDot, "Extr.")

FSTRINGVALUE(Com::tPrinterModeFFF, "PrinterMode:FFF")
FSTRINGVALUE(Com::tPrinterModeLaser, "PrinterMode:Laser")
FSTRINGVALUE(Com::tPrinterModeCNC, "PrinterMode:CNC")
#ifdef STARTUP_GCODE
FSTRINGVALUE(Com::tStartupGCode, STARTUP_GCODE)
#endif
FSTRINGVALUE(Com::tI2CError, "I2C communication error occured")
FSTRINGVALUE(Com::tColonSpace, ": ")
FSTRINGVALUE(Com::tMotorMotorSpace, "Motor ")
FSTRINGVALUE(Com::tMotorResolutionColon, "Resolution:")
FSTRINGVALUE(Com::tMotorResolutionSteps, "Resolution [steps/mm]")
FSTRINGVALUE(Com::tMotorMicrosteps, "Microsteps")
FSTRINGVALUE(Com::tMotorRMSCurrentMA, "RMS Current [mA]")
FSTRINGVALUE(Com::tMotorRMSCurrentMAColon, "RMS Current [mA]:")
FSTRINGVALUE(Com::tMotorHybridTresholdMMS, "Hybrid Treshold [mm/s]")
FSTRINGVALUE(Com::tMotorStealthOnOff, "Stealth [0/1]")
FSTRINGVALUE(Com::tMotorStallSensitivity255, "Stall Sensitivity [0..255]")
FSTRINGVALUE(Com::tMotorStallSensitivity64, "Stall Sensitivity [-64..63]")
FSTRINGVALUE(Com::tMotorStatusColon, "Status: ")
FSTRINGVALUE(Com::tMotorNoConnection, "No connection")
FSTRINGVALUE(Com::tMotorNoPower, "No power")
FSTRINGVALUE(Com::tMotorEnabledColon, "Enabled: ")
FSTRINGVALUE(Com::tMotorSpaceSetColonSpace, " set: ")
FSTRINGVALUE(Com::tMotorMaxCurrentMA, "Max. current [mA]: ")
FSTRINGVALUE(Com::tMotorMicrostepsColon, "Microsteps: ")
FSTRINGVALUE(Com::tMotorSpaceMresColon, " mres: ")
FSTRINGVALUE(Com::tMotorStealthChopColon, "StealthChop: ")
FSTRINGVALUE(Com::tMotorHybridTresholdMMSColon, "Hybrid treshold [mm/s]: ")
FSTRINGVALUE(Com::tMotorHybridModeDisabled, "Hybrid mode disabled")
FSTRINGVALUE(Com::tMotorStallguardSensitivityColon, "Stallguard sensitivity: ")
FSTRINGVALUE(Com::tMotorTStep, "TSTEP: ")
FSTRINGVALUE(Com::tMax, "max")
FSTRINGVALUE(Com::tMotorTPWMTHRS, "TPWMTHRS: ")
FSTRINGVALUE(Com::tMotorTPOWERDOWN, "TPOWERDOWN: ")
FSTRINGVALUE(Com::tMotorIRUN, "IRUN: ")
FSTRINGVALUE(Com::tMotorSlash31, "/31")
FSTRINGVALUE(Com::tMotorIHOLD, "IHOLD: ")
FSTRINGVALUE(Com::tMotorCSActual, "CS Actual: ")
FSTRINGVALUE(Com::tMotorVSense, "vsense: ")
FSTRINGVALUE(Com::tMotorTOff, "toff: ")
FSTRINGVALUE(Com::tMotorHStart, "hstart: ")
FSTRINGVALUE(Com::tMotorHEnd, "hend: ")
FSTRINGVALUE(Com::tMotorBlankTime, "Blank time: ")
FSTRINGVALUE(Com::tMotorTBLColon, " tbl: ")
FSTRINGVALUE(Com::tMotorIOIN, "IOIN: ")
FSTRINGVALUE(Com::tMotorGSTAT, "GSTAT: ")
FSTRINGVALUE(Com::tMotorDriverShort, " driver short circuit ")
FSTRINGVALUE(Com::tMotorDriverOvertempCurrent, " driver overtemperature! Current: ")
FSTRINGVALUE(Com::tMotorDriverOvertempWarningCurrent, " driver overtemperature warning! Current: ")
FSTRINGVALUE(Com::tMotorCurrentDecreasedTo, " current decreased to ")
FSTRINGVALUE(Com::tMotorSpaceStealthChopColon, " stealthChop: ")
FSTRINGVALUE(Com::tMotorTempPrewarnTriggered, " temperature prewarn triggered: ")
FSTRINGVALUE(Com::tMotorPrewarnFlagCleared, " prewarn flag cleared")
FSTRINGVALUE(Com::tMotorSpaceHybridTresholdColor, " hybrid treshold: ")
FSTRINGVALUE(Com::tMotorSpaceStallguardSensitivityColon, " stallguard sensitivity: ")
FSTRINGVALUE(Com::tMotorStallguardResult, "StallGuard Result: ")
FSTRINGVALUE(Com::tMotorSpaceRMSCurrentMAColon, " RMS Current [mA]: ")

bool Com::writeToAll = true; // transmit start messages to all devices!

void Com::cap(FSTRINGPARAM(text)) {
    printF(tCap);
    printFLN(text);
}

void Com::config(FSTRINGPARAM(text)) {
    printF(tConfig);
    printFLN(text);
}

void Com::config(FSTRINGPARAM(text), int value) {
    printF(tConfig);
    printFLN(text, value);
}

void Com::config(FSTRINGPARAM(text), const char* msg) {
    printF(tConfig);
    printF(text);
    print(msg);
    println();
}

void Com::config(FSTRINGPARAM(text), int32_t value) {
    printF(tConfig);
    printFLN(text, value);
}

void Com::config(FSTRINGPARAM(text), uint32_t value) {
    printF(tConfig);
    printFLN(text, value);
}

void Com::config(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(tConfig);
    printFLN(text, value, digits);
}

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
    printFLN(text);
}

void Com::printLogFLN(FSTRINGPARAM(text)) {
    printF(tLog);
    printFLN(text);
}

void Com::printLogF(FSTRINGPARAM(text)) {
    printF(tLog);
    printF(text);
}

void Com::printFLN(FSTRINGPARAM(text)) {
    printF(text);
    println();
}

void Com::printFLN(FSTRINGPARAM(text), const char* msg) {
    printF(text);
    print(msg);
    println();
}

void Com::printF(FSTRINGPARAM(ptr)) {
    char c;
    while ((c = HAL::readFlashByte(ptr++)) != 0) {
        GCodeSource::writeToAll(c);
    }
}

void Com::printF(FSTRINGPARAM(text), const char* msg) {
    printF(text);
    print(msg);
}

void Com::printF(FSTRINGPARAM(text), bool value, BoolFormat format) {
    printF(text);
    print(value, format);
}

void Com::printF(FSTRINGPARAM(text), int value) {
    printF(text);
    print(value);
}

void Com::printF(FSTRINGPARAM(text), int32_t value) {
    printF(text);
    print(value);
}

void Com::printF(FSTRINGPARAM(text), uint32_t value) {
    printF(text);
    printNumber(value);
}

void Com::printFLN(FSTRINGPARAM(text), int value) {
    printF(text);
    print(value);
    println();
}

void Com::printFLN(FSTRINGPARAM(text), int32_t value) {
    printF(text);
    print(value);
    println();
}

void Com::printFLN(FSTRINGPARAM(text), uint32_t value) {
    printF(text);
    printNumber(value);
    println();
}

void Com::printFLN(FSTRINGPARAM(text), bool value, BoolFormat format) {
    printF(text);
    print(value, format);
    println();
}

void Com::printFLN(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(text);
    printFloat(value, digits);
    println();
}

void Com::printF(FSTRINGPARAM(text), float value, uint8_t digits) {
    printF(text);
    printFloat(value, digits);
}

void Com::print(const char* text) {
    while (*text) {
        GCodeSource::writeToAll(*text++);
    }
}
void Com::print(long value) {
    if (value < 0) {
        GCodeSource::writeToAll('-');
        value = -value;
    }
    printNumber(value);
}
void Com::print(bool value, BoolFormat format) {
    switch (format) {
    case BoolFormat::TRUEFALSE:
        if (value) {
            printF(PSTR("True"));
        } else {
            printF(PSTR("False"));
        }
        break;
    case BoolFormat::YESNO:
        if (value) {
            printF(PSTR("Yes"));
        } else {
            printF(PSTR("No"));
        }
        break;
    case BoolFormat::ONOFF:
        if (value) {
            printF(PSTR("On"));
        } else {
            printF(PSTR("Off"));
        }
        break;
    }
}
void Com::printNumber(uint32_t n) {
    char buf[11]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[10];
    *str = '\0';
    do {
        unsigned long m = n;
        n /= 10;
        *--str = '0' + (m - 10 * n);
    } while (n);

    print(str);
}
void Com::printArrayFLN(FSTRINGPARAM(text), float* arr, uint8_t n, uint8_t digits) {
    printF(text);
    for (uint8_t i = 0; i < n; i++)
        printF(Com::tSpace, arr[i], digits);
    println();
}
void Com::printArrayFLN(FSTRINGPARAM(text), int32_t* arr, uint8_t n) {
    printF(text);
    for (uint8_t i = 0; i < n; i++)
        printF(Com::tSpace, arr[i]);
    println();
}

void Com::printFloat(float number, uint8_t digits) {
    if (isnan(number)) {
        printF(tNAN);
        return;
    }
    if (isinf(number)) {
        printF(tINF);
        return;
    }
    // Handle negative numbers
    if (number < 0.0) {
        print('-');
        number = -number;
    }
    // Round correctly so that print(1.999, 2) prints as "2.00"
    float rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
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
    while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
    }
}

void Com::promptStart(PromptDialogCallback cb, FSTRINGPARAM(text), bool blocking) {
    if (blocking) {
        Com::printFLN(PSTR("//action:paused"));
    }
    if (Printer::promptSupported) {
        Com::printF(PSTR("//action:prompt_begin "));
        Com::printFLN(text);
    }
    Printer::activePromptDialog = cb;
}

void Com::promptStart(PromptDialogCallback cb, char* text, bool blocking) {
    if (blocking) {
        Com::printFLN(PSTR("//action:paused"));
    }
    if (Printer::promptSupported) {
        Com::printFLN(PSTR("//action:prompt_begin "), text);
    }
    Printer::activePromptDialog = cb;
}

void Com::promptStart(PromptDialogCallback cb, FSTRINGPARAM(prefix), FSTRINGPARAM(text), bool blocking) {
    if (blocking) {
        Com::printFLN(PSTR("//action:paused"));
    }
    if (Printer::promptSupported) {
        Com::printF(PSTR("//action:prompt_begin "));
        Com::printF(prefix);
        Com::print(' ');
        Com::printFLN(text);
    }
    Printer::activePromptDialog = cb;
}

void Com::promptStart(PromptDialogCallback cb, FSTRINGPARAM(prefix), char* text, bool blocking) {
    if (blocking) {
        Com::printFLN(PSTR("//action:paused"));
    }
    if (Printer::promptSupported) {
        Com::printF(PSTR("//action:prompt_begin "));
        Com::printF(prefix);
        Com::print(' ');
        Com::print(text);
        Com::println();
    }
    Printer::activePromptDialog = cb;
}

void Com::promptEnd(bool blocking) {
    writeToAll = true;
    if (blocking) {
        Com::printFLN(PSTR("//action:resumed"));
    }
    if (Printer::promptSupported) {
        Com::printFLN(PSTR("//action:prompt_end"));
    }
    Printer::activePromptDialog = nullptr;
}

void Com::promptButton(FSTRINGPARAM(text)) {
    if (Printer::promptSupported) {
        Com::printF(PSTR("//action:prompt_button "));
        Com::printFLN(text);
    }
}

void Com::promptShow() {
    if (Printer::promptSupported) {
        Com::printFLN(PSTR("//action:prompt_show"));
    }
}
