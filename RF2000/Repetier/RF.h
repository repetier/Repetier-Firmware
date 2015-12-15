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


#ifndef RF_H
#define RF_H


// ##########################################################################################
// ##	RF specific UI Actions
// ##########################################################################################

#define UI_ACTION_RF_MIN_REPEATABLE			 500

#define UI_ACTION_RF_HEAT_BED_UP			 514
#define UI_ACTION_RF_HEAT_BED_DOWN			 515
#define UI_ACTION_RF_EXTRUDER_OUTPUT		 516
#define UI_ACTION_RF_EXTRUDER_RETRACT		 517
#define	UI_ACTION_RF_SET_WORK_PART			 518
#define UI_ACTION_RF_SET_SCAN_DELTA_X		 519
#define UI_ACTION_RF_SET_SCAN_DELTA_Y		 520

#define UI_ACTION_RF_MAX_REPEATABLE			 600

#define UI_ACTION_RF_MIN_SINGLE				1500

#define UI_ACTION_RF_SCAN_HEAT_BED			1512
#define UI_ACTION_RF_TEST_COMPENSATION		1513
#define UI_ACTION_RF_PAUSE					1518
#define UI_ACTION_RF_CONTINUE				1519
#define UI_ACTION_RF_DO_HEAT_BED_SCAN		1520
#define UI_ACTION_RF_PARK					1521
#define UI_ACTION_RF_RESET					1522
#define UI_ACTION_RF_RESET_ACK				1523
#define UI_ACTION_RF_OUTPUT_OBJECT			1524
#define UI_ACTION_RF_FIND_Z_ORIGIN			1525
#define UI_ACTION_RF_DO_WORK_PART_SCAN		1526
#define UI_ACTION_RF_SET_SCAN_XY_START		1527
#define UI_ACTION_RF_SET_SCAN_XY_END		1528

#define UI_ACTION_RF_MAX_SINGLE				1600


/*
// ##########################################################################################
// ##	RF specific M codes
// ##########################################################################################

// ##########################################################################################
// ##	the following M codes are for the heat bed scan in operating mode "print"
// ##########################################################################################

- M3000 - turn the z-compensation off
- M3001 - turn the z-compensation on
- M3002 - configure the min z-compensation offset ( units are [steps] )
- M3003 - configure the max z-compensation offset ( units are [steps] )
- M3006 - configure the static z-offset ( units are [um] )
- M3007 - configure the min z-compensation offset ( units are [um] )
- M3008 - configure the max z-compensation offset ( units are [um] )

- M3010 - start/abort the heat bed scan
- M3011 - clear the z-compensation matrix from the EEPROM
- M3012 - restore the default scan parameters
- M3013 - output the current compensation matrix

- M3020 - configure the x start position for the heat bed scan ( units are [mm] )
- M3021 - configure the y start position for the heat bed scan ( units are [mm] )
- M3022 - configure the x step size for the heat bed scan ( units are [mm] )
- M3023 - configure the y step size for the heat bed scan ( units are [mm] )
- M3024 - configure the x end position for the heat bed scan ( units are [mm] )
- M3025 - configure the y end position for the heat bed scan ( units are [mm] )

- M3030 - configure the fast step size for moving of the heat bed up during the heat bed scan
- M3031 - configure the slow step size for moving of the heat bed up during the heat bed scan
- M3032 - configure the fast step size for moving of the heat bed down during the heat bed scan
- M3033 - configure the slow step size for moving of the heat bed down during the heat bed scan

- M3040 - configure the delay ( in ms ) between two fast movements during the heat bed scan
- M3041 - configure the delay ( in ms ) between two slow movements during the heat bed scan
- M3042 - configure the delay ( in ms ) between reaching of a new x/y position and the test of the idle pressure

- M3050 - configure the contact pressure delta ( units are [digits] )
- M3051 - configure the retry pressure delta ( units are [digits] )
- M3052 - configure the idle pressure tolerance ( units are [digits] )
- M3053 - configure the number of A/D converter reads per pressure measurement
- M3054 - configure the delay between two A/D converter reads ( units are [ms] )
- M3055 - configure the pressure tolerance per pressure measurement ( units are [digits] )


// ##########################################################################################
// ##	the following M codes are for the general configuration
// ##########################################################################################

- M3005 - enable custom debug outputs

- M3060 - output the device type and firmware version

- M3070 - pause the print as if the "Pause" button would have been pressed
- M3071 - wait until the print has been continued via the "Continue" button

- M3075 - configure the emergency pause digits

- M3079 - output the printed object
- M3080 - park the printer

- M3090 - test the watchdog ( this command resets the firmware )
- M3091 - erase the external EEPROM

- M3100 - configure the number of manual z steps after the "Heat Bed up" or "Heat Bed down" button has been pressed
- M3101 - configure the number of manual extruder steps after the "Extruder output" or "Extruder retract" button has been pressed
- M3102 - configure the offset in x, y and z direction as well as the extruder retract which shall be applied in case the "Pause Printing" button has been pressed (units are [steps])
- M3103 - configure the x, y and z position which shall set when the printer is parked
- M3105 - configure the offset in x, y and z direction as well as the extruder retract which shall be applied in case the "Pause Printing" button has been pressed (units are [mm])

- M3110 - lock the current status text
- M3115 - set the x/y origin to the current x/y position

- M3120 - turn on the case fan
- M3121 - turn off the case fan


// ##########################################################################################
// ##	the following M codes are for the work part scan in operating mode "mill"
// ##########################################################################################

- M3130 - start/stop the search of the z-origin

- M3140 - turn the z-compensation off
- M3141 - turn the z-compensation on
- M3146 - configure the static z-offset ( units are [um] )
- M3149 - get/choose the active work part z-compensation matrix

- M3150 - start/abort the work part scan
- M3151 - clear the specified z-compensation matrix from the EEPROM
- M3152 - restore the default scan parameters
- M3153 - output the specified work part z-compensation matrix

- M3160 - configure the x start position for the work part scan ( units are [mm] )
- M3161 - configure the y start position for the work part scan ( units are [mm] )
- M3162 - configure the x step size for the work part scan ( units are [mm] )
- M3163 - configure the y step size for the work part scan ( units are [mm] )
- M3164 - configure the x end position for the work part scan ( units are [mm] )
- M3165 - configure the y end position for the work part scan ( units are [mm] )


// ##########################################################################################
// ##	other M codes
// ##########################################################################################

- M3200 - reserved for test and debug


// ##########################################################################################
// ##	the following M codes are supported only by the RF2000
// ##########################################################################################

- M3300 - configure the 24V FET-Outputs ( on/off )
- M3301 - configure the 230V output ( on/off )
- M3303 - configure the RGB light effects for heating
- M3304 - configure the RGB light effects for printing
- M3305 - configure the RGB light effects for cooling
- M3306 - configure the RGB light effects for idle
- M3307 - configure the manual RGB light colors
- M3308 - configure the RGB light mode
*/


// ##########################################################################################
// ##	X/Y/Z movements
// ##########################################################################################
/*
The following variables are used for movements in x/y/z direction:

- Printer::queuePositionTargetSteps[x/y/z]
  - unit is [steps]
  - holds the position which shall be reached through the g-codes which are currently within the queue
  - this is not the position to which the printer is heading at the moment, because the printer is heading always to one position (from one g-code) and the queue can contain several g-codes

- Printer::queuePositionLastSteps[x/y/z]
  - unit is [steps]
  - holds the last position which has been calculated as queuePositionTargetSteps
  - in most cases, the value of queuePositionLastSteps is identical to the value of queuePositionTargetSteps, the values of these variables are different only while a new position is calculated

- Printer::queuePositionLastMM[x/y/z]
  - unit is [mm]
  - the rest is identical to queuePositionLastSteps

- Printer::queuePositionCurrentSteps[x/y/z]
  - unit is [steps]
  - holds the position which has been reached through the movements from the queue
  - the value of queuePositionCurrentSteps represents the current position of the printer in x, y and z direction

- Printer::stepperDirection[x/y/z]
  - holds the direction of the axes as it is requested by the currently processed movement from the queue

- Printer::queuePositionCommandMM[x/y/z]
  - unit is [mm]
  - in most cases, the value of queuePositionCommandMM is identical to the value of queuePositionLastMM, the values of these variables are different only while a new command is processed

- Printer::directPositionTargetSteps[x/y/z]
  - unit is [steps]
  - holds the position which shall be reached through direct movements, e.g. from the manual buttons or from the direct pause/continue functionality

- Printer::directPositionLastSteps[x/y/z]
  - unit is [steps]
  - holds the last position which has been calculated as directPositionTargetSteps
  - in most cases, the value of directPositionLastSteps is identical to the value of directPositionTargetSteps, the values of these variables are different only while a new position is calculated

- Printer::directPositionCurrentSteps[x/y/z]
  - unit is [steps]
  - holds the position which has been reached through direct movements, e.g. from the manual buttons or from the direct pause/continue functionality

- Printer::originOffsetMM[x/y/z]
  - unit is [mm]
  - holds the offset to the real origin of each axis

- Printer::compensatedPositionTargetStepsZ
  - unit is [steps]
  - holds the position which shall be reached through the z-compensation

- Printer::compensatedPositionCurrentStepsZ
  - unit is [steps]
  - holds the position which has been reached through the z-compensation

- the current x/y position of the printer in [steps] is:
  - Printer::queuePositionCurrentSteps[x/y] + directPositionCurrentSteps[x/y]
  - note that an additional, extruder-dependent origin can be used/set

- the current z position of the printer in [steps] is:
  - Printer::queuePositionCurrentSteps[z] + directPositionCurrentSteps[z] + compensatedPositionCurrentStepsZ

*/


#if FEATURE_HEAT_BED_Z_COMPENSATION

// determine the maximal needed size for the heat bed compensation
// in case also FEATURE_WORK_PART_Z_COMPENSATION is enabled, only the defined dimensions for the heat bed scan count (so it must be ensured that the dimensions of the heat bed compensation matrix are at least of the size of the work part compensation matrix)
#define	COMPENSATION_MATRIX_MAX_X			long((X_MAX_LENGTH - HEAT_BED_SCAN_X_START_MM - HEAT_BED_SCAN_X_END_MM) / HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM + 4)
#define	COMPENSATION_MATRIX_MAX_Y			long((Y_MAX_LENGTH - HEAT_BED_SCAN_Y_START_MM - HEAT_BED_SCAN_Y_END_MM) / HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM + 4)

#define	COMPENSATION_MATRIX_SIZE			long(COMPENSATION_MATRIX_MAX_X * COMPENSATION_MATRIX_MAX_Y * 2 + EEPROM_OFFSET_MAXTRIX_START)	// [bytes]

#elif FEATURE_WORK_PART_Z_COMPENSATION

// determine the maximal needed size for the work part compensation
#define	COMPENSATION_MATRIX_MAX_X			long((X_MAX_LENGTH - WORK_PART_SCAN_X_START_MM - WORK_PART_SCAN_X_END_MM) / WORK_PART_SCAN_X_STEP_SIZE_MIN_MM + 4)
#define	COMPENSATION_MATRIX_MAX_Y			long((Y_MAX_LENGTH - WORK_PART_SCAN_Y_START_MM - WORK_PART_SCAN_Y_END_MM) / WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM + 4)

#define	COMPENSATION_MATRIX_SIZE			long(COMPENSATION_MATRIX_MAX_X * COMPENSATION_MATRIX_MAX_Y * 2 + EEPROM_OFFSET_MAXTRIX_START)	// [bytes]

#endif // FEATURE_HEAT_BED_Z_COMPENSATION && FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION
extern	long			g_offsetZCompensationSteps;	// this is the minimal distance between the heat bed and the extruder at the moment when the z-min endstop is hit
extern	long			g_minZCompensationSteps;
extern	long			g_maxZCompensationSteps;
extern	long			g_diffZCompensationSteps;
extern	unsigned char	g_nHeatBedScanStatus;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_WORK_PART_Z_COMPENSATION
extern	char			g_nWorkPartScanStatus;
extern	char			g_nWorkPartScanMode;		// 0 = do not home z-axis, 1 = home z-axis
extern	char			g_nActiveWorkPart;
#endif // FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
extern	char			g_abortZScan;
extern	short			g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
extern	unsigned char	g_uZMatrixMaxX;
extern	unsigned char	g_uZMatrixMaxY;
extern	long			g_nZScanZPosition;

extern	long			g_nScanXStepSizeMm;
extern	long			g_nScanXStepSizeSteps;
extern	long			g_nScanYStepSizeMm;
extern	long			g_nScanYStepSizeSteps;

extern	unsigned short	g_nScanContactPressureDelta;
extern	unsigned short	g_nScanRetryPressureDelta;
#endif // #if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

extern	long			g_staticZSteps;
extern	char			g_debugLevel;
extern	char			g_debugLog;
//extern	long			g_debugCounter[20];
extern	unsigned long	g_uStopTime;
extern	unsigned long	g_uBlockCommands;
extern	short			g_debugInt16;
extern	unsigned short	g_debugUInt16;
extern	long			g_debugInt32;

// other configurable parameters
#if FEATURE_EXTENDED_BUTTONS
extern	unsigned long	g_nManualZSteps;
extern	unsigned long	g_nManualExtruderSteps;
#endif // FEATURE_EXTENDED_BUTTONS


#if FEATURE_PAUSE_PRINTING
extern	long			g_nPauseSteps[4];
extern	long			g_nContinueSteps[4];
extern	char			g_pauseStatus;
extern	char			g_pauseMode;
extern	unsigned long	g_uPauseTime;
extern	char			g_pauseBeepDone;
extern	int				g_ContinueButtonPressed;
#endif // FEATURE_PAUSE_PRINTING


#if FEATURE_FIND_Z_ORIGIN
extern	char			g_nFindZOriginStatus;
extern	long			g_nZOriginPosition[3];
extern	int				g_nZOriginSet;
#endif // FEATURE_FIND_Z_ORIGIN


#if DEBUG_HEAT_BED_Z_COMPENSATION
extern	long			g_nLastZCompensationPositionSteps[3];
extern	long			g_nLastZCompensationTargetStepsZ;
extern	long			g_nZCompensationUpdates;
extern	long			g_nDelta[2];
extern	long			g_nStepSize[2];
extern	long			g_nTempXFront;
extern	long			g_nTempXBack;
extern	long			g_nNeededZ;
extern	unsigned char	g_uIndex[4];
extern	short			g_nMatrix[4];
extern	long			g_nZDeltaMin;
extern	long			g_nZDeltaMax;
extern	long			g_nZCompensationUpdateTime;
extern	long			g_nZCompensationDelayMax;
extern	long			g_nTooFast;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION

#if FEATURE_RGB_LIGHT_EFFECTS

extern unsigned char	g_uRGBHeatingR;
extern unsigned char	g_uRGBHeatingG;
extern unsigned char	g_uRGBHeatingB;
extern unsigned char	g_uRGBPrintingR;
extern unsigned char	g_uRGBPrintingG;
extern unsigned char	g_uRGBPrintingB;
extern unsigned char	g_uRGBCoolingR;
extern unsigned char	g_uRGBCoolingG;
extern unsigned char	g_uRGBCoolingB;
extern unsigned char	g_uRGBIdleR;
extern unsigned char	g_uRGBIdleG;
extern unsigned char	g_uRGBIdleB;
extern unsigned char	g_uRGBManualR;
extern unsigned char	g_uRGBManualG;
extern unsigned char	g_uRGBManualB;
extern unsigned char	g_uRGBCurrentR;
extern unsigned char	g_uRGBCurrentG;
extern unsigned char	g_uRGBCurrentB;
extern unsigned char	g_uRGBTargetR;
extern unsigned char	g_uRGBTargetG;
extern unsigned char	g_uRGBTargetB;
#endif // FEATURE_RGB_LIGHT_EFFECTS


// initRF()
extern void initRF( void );

// initStrainGauge()
extern void initStrainGauge( void );

// readStrainGauge()
extern short readStrainGauge( unsigned char uAddress );

#if FEATURE_HEAT_BED_Z_COMPENSATION
// startHeatBedScan()
extern void startHeatBedScan( void );

// scanHeatBed()
extern void scanHeatBed( void );

// testExtruderTemperature()
extern short testExtruderTemperature( void );

// testHeatBedTemperature()
extern short testHeatBedTemperature( void );

// doHeatBedZCompensation()
extern void doHeatBedZCompensation( void );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_WORK_PART_Z_COMPENSATION
// startWorkPartScan()
extern void startWorkPartScan( char nMode );

// scanWorkPart()
extern void scanWorkPart( void );

// doWorkPartZCompensation()
extern void doWorkPartZCompensation( void );

// determineStaticCompensationZ()
extern void determineStaticCompensationZ( void );
#endif // FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
// readIdlePressure()
extern short readIdlePressure( short* pnIdlePressure );

// testIdlePressure()
extern short testIdlePressure( void );

// readAveragePressure()
extern short readAveragePressure( short* pnAveragePressure );

// moveZUpFast()
extern short moveZUpFast( void );

// moveZDownSlow()
extern short moveZDownSlow( void );

// moveZUpSlow()
extern short moveZUpSlow( short* pnContactPressure, char* pnRetry );

// moveZDownFast()
extern short moveZDownFast( void );

// moveZ()
extern int moveZ( int nSteps );

// moveExtruder()
extern int moveExtruder( int nSteps );

// restoreDefaultScanParameters()
extern void restoreDefaultScanParameters( void );

// outputScanParameters()
extern void outputScanParameters( void );

// outputCompensationMatrix()
extern void outputCompensationMatrix( void );

// initCompensationMatrix()
extern void initCompensationMatrix( void );

// prepareCompensationMatrix()
extern char prepareCompensationMatrix( void );

// convertCompensationMatrix()
extern char convertCompensationMatrix( void );

// saveCompensationMatrix()
extern char saveCompensationMatrix( unsigned int uAddress );

// loadCompensationMatrix()
extern char loadCompensationMatrix( unsigned int uAddress );

// clearCompensationMatrix()
extern void clearCompensationMatrix( unsigned int uAddress );

// outputPressureMatrix()
extern void outputPressureMatrix( void );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

// clearExternalEEPROM()
extern char clearExternalEEPROM( void );

// writeByte24C256()
extern void writeByte24C256( int addressI2C, unsigned int addressEEPROM, unsigned char data );

// writeWord24C256()
extern void writeWord24C256( int addressI2C, unsigned int addressEEPROM, unsigned short data );

// readByte24C256()
extern unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM );

// readWord24C256()
extern unsigned short readWord24C256( int addressI2C, unsigned int addressEEPROM );

// doZCompensation()
extern void doZCompensation( void );

// loopRF()
extern void loopRF( void );

#if FEATURE_OUTPUT_FINISHED_OBJECT
// outputObject()
extern void outputObject( void );
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FEATURE_PARK
// parkPrinter()
extern void parkPrinter( void );
#endif // FEATURE_PARK

// pausePrint()
extern void pausePrint( void );

// continuePrint()
extern void continuePrint( void );

#if FEATURE_PAUSE_PRINTING
// determinePausePosition()
extern void determinePausePosition( void );

// determineZPausePositionForPrint()
extern void determineZPausePositionForPrint( void );

// determineZPausePositionForMill()
extern void determineZPausePositionForMill( void );

// waitUntilContinue
extern void waitUntilContinue( void );
#endif // FEATURE_PAUSE_PRINTING

// setExtruderCurrent()
extern void setExtruderCurrent( unsigned short level );

// processCommand()
extern void processCommand( GCode* pCommand );

// runStandardTasks()
extern void runStandardTasks( void );

// queueTask()
extern void queueTask( char task );

// processButton()
extern void processButton( int nAction );

// nextPreviousZAction()
extern void nextPreviousZAction( int8_t next );


#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DRV8711
// setMotorCurrent()
extern void setMotorCurrent( unsigned char driver, unsigned short level );

// motorCurrentControlInit()
extern void motorCurrentControlInit( void );
#endif // CURRENT_CONTROL_DRV8711


// cleanupXPositions
extern void cleanupXPositions( void );

// cleanupYPositions
extern void cleanupYPositions( void );

// cleanupZPositions
extern void cleanupZPositions( void );

// cleanupEPositions
extern void cleanupEPositions( void );

// setZOrigin()
extern void setZOrigin( void );


#if FEATURE_FIND_Z_ORIGIN
// startFindZOrigin()
extern void startFindZOrigin( void );

// findZOrigin()
extern void findZOrigin( void );
#endif // FEATURE_FIND_Z_ORIGIN


#if FEATURE_MILLING_MODE
// switchOperatingMode()
extern void switchOperatingMode( char newOperatingMode );

// switchActiveWorkPart()
extern void switchActiveWorkPart( char newActiveWorkPart );

// setScanXYStart()
extern void setScanXYStart( void );

// setScanXYEnd()
extern void setScanXYEnd( void );
#endif // FEATURE_MILLING_MODE


#if FEATURE_RGB_LIGHT_EFFECTS
// setRGBTargetColors()
extern void setRGBTargetColors( uint8_t R, uint8_t G, uint8_t B );

// setRGBLEDs()
extern void setRGBLEDs( uint8_t R, uint8_t G, uint8_t B );

// updateRGBLightStatus()
extern void updateRGBLightStatus( void );
#endif // FEATURE_RGB_LIGHT_EFFECTS


// setupForPrinting
extern void setupForPrinting( void );

// setupForMilling()
extern void setupForMilling( void );

// prepareZCompensation()
extern void prepareZCompensation( void );

// resetZCompensation()
extern void resetZCompensation( void );

// isSupportedGCommand()
extern unsigned char isSupportedGCommand( unsigned int currentGCode, char neededMode, char outputLog = 1 );

// isSupportedMCommand()
extern unsigned char isSupportedMCommand( unsigned int currentMCode, char neededMode, char outputLog = 1 );

// isMovingAllowed()
extern unsigned char isMovingAllowed( const char* pszCommand, char outputLog = 1 );

// isHomingAllowed()
extern unsigned char isHomingAllowed( GCode* com, char outputLog = 1 );

// showInvalidSyntax()
extern void showInvalidSyntax( unsigned int currentMCode );

// addUInt32()
extern void addUInt32( char* pszString, uint32_t uNumber );

// addFloat()
extern void addFloat( char* pszString, float fNumber, uint8_t uDigits );


#if FEATURE_HEAT_BED_TEMP_COMPENSATION
// getHeatBedTemperatureOffset
float getHeatBedTemperatureOffset( float temperatureInCelsius );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION


#if FEATURE_TYPE_EEPROM
// determineHardwareType()
void determineHardwareType( void );

// notifyAboutWrongHardwareType()
void notifyAboutWrongHardwareType( unsigned char guessedHardwareType );
#endif // FEATURE_TYPE_EEPROM


#endif // RF_H