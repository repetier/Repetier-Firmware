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


#include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#include "Repetier.h"
#include <Wire.h>


unsigned long	g_lastTime				   = 0;

#if FEATURE_HEAT_BED_Z_COMPENSATION
long			g_offsetZCompensationSteps = 0;
long			g_minZCompensationSteps	   = HEAT_BED_Z_COMPENSATION_MIN_STEPS;
long			g_maxZCompensationSteps	   = HEAT_BED_Z_COMPENSATION_MAX_STEPS;
long			g_diffZCompensationSteps   = HEAT_BED_Z_COMPENSATION_MAX_STEPS - HEAT_BED_Z_COMPENSATION_MIN_STEPS;
unsigned char	g_nHeatBedScanStatus	   = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
char			g_nWorkPartScanStatus = 0;
char			g_nWorkPartScanMode	  = 0;
char			g_nActiveWorkPart	  = 1;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
unsigned long	g_lastScanTime		  = 0;
unsigned long	g_scanStartTime		  = 0;
char			g_scanRetries		  = 0;
char			g_retryZScan		  = 0;

char			g_abortZScan		  = 0;
short			g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
unsigned char	g_uZMatrixMaxX		  = 0;
unsigned char	g_uZMatrixMaxY		  = 0;
long			g_nZScanZPosition	  = 0;
long			g_nLastZScanZPosition = 0;

short			g_nMaxPressureContact;
short			g_nMaxPressureRetry;
short			g_nMaxPressureIdle;
short			g_nMinPressureContact;
short			g_nMinPressureRetry;
short			g_nMinPressureIdle;
short			g_nFirstIdlePressure;
short			g_nCurrentIdlePressure;
char			g_nTempDirectionZ			 = 0;	// this is the current z-direction during operations like the bed scan or finding of the z-origin

// configurable scan parameters - the proper default values are set by restoreDefaultScanParameters()
long			g_nScanXStartSteps			 = 0;
long			g_nScanXStepSizeMm			 = 0;
long			g_nScanXStepSizeSteps		 = 0;
long			g_nScanXEndSteps			 = 0;
long			g_nScanXMaxPositionSteps	 = 0;
long			g_nScanYStartSteps			 = 0;
long			g_nScanYStepSizeMm			 = 0;
long			g_nScanYStepSizeSteps		 = 0;
long			g_nScanYEndSteps			 = 0;
long			g_nScanYMaxPositionSteps	 = 0;
short			g_nScanHeatBedUpFastSteps	 = 0;
short			g_nScanHeatBedUpSlowSteps	 = 0;
short			g_nScanHeatBedDownFastSteps	 = 0;
short			g_nScanHeatBedDownSlowSteps	 = 0;
long			g_nScanZMaxCompensationSteps = 0;
unsigned short	g_nScanFastStepDelay		 = 0;
unsigned short	g_nScanSlowStepDelay		 = 0;
unsigned short	g_nScanIdleDelay			 = 0;
unsigned short	g_nScanContactPressureDelta	 = 0;
unsigned short	g_nScanRetryPressureDelta	 = 0;
unsigned short	g_nScanIdlePressureDelta	 = 0;
short			g_nScanIdlePressureMin		 = 0;
short			g_nScanIdlePressureMax		 = 0;
char			g_nScanPressureReads		 = 0;
unsigned short	g_nScanPressureReadDelay	 = 0;
short			g_nScanPressureTolerance	 = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if DEBUG_REMEMBER_SCAN_PRESSURE
short			g_ScanPressure[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

long			g_staticZSteps				= 0;
char			g_debugLevel				= 0;
char			g_debugLog					= 0;
//long			g_debugCounter[20]			= { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long	g_uStopTime					= 0;
unsigned long	g_uBlockCommands			= 0;
short			g_debugInt16				= 0;
unsigned short	g_debugUInt16				= 0;
long			g_debugInt32				= 0;

// other configurable parameters
unsigned long	g_nManualZSteps				= DEFAULT_MANUAL_Z_STEPS;
unsigned long	g_nManualExtruderSteps		= DEFAULT_MANUAL_EXTRUDER_STEPS;

#if FEATURE_PAUSE_PRINTING
long			g_nPauseSteps[4]			= { DEFAULT_PAUSE_STEPS_X, DEFAULT_PAUSE_STEPS_Y, DEFAULT_PAUSE_STEPS_Z, DEFAULT_PAUSE_STEPS_EXTRUDER };
long			g_nContinueSteps[4]			= { 0, 0, 0, 0 };
char			g_pauseStatus				= PAUSE_STATUS_NONE;
char			g_pauseMode					= PAUSE_MODE_NONE;
unsigned long	g_uPauseTime				= 0;
char			g_pauseBeepDone				= 0;
int				g_ContinueButtonPressed		= 0;
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_PARK
long			g_nParkPosition[3]			= { PARK_POSITION_X, PARK_POSITION_Y, PARK_POSITION_Z };
#endif // FEATURE_PARK

#if FEATURE_EMERGENCY_PAUSE
unsigned long	g_uLastPressureTime			= 0;
long			g_nPressureSum				= 0;
char			g_nPressureChecks			= 0;
long			g_nEmergencyPauseDigitsMin	= 0;
long			g_nEmergencyPauseDigitsMax	= 0;
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_EMERGENCY_Z_STOP
unsigned long	g_uLastZPressureTime		= 0;
long			g_nZPressureSum				= 0;
char			g_nZPressureChecks			= 0;
#endif // FEATURE_EMERGENCY_Z_STOP

#if FEATURE_FIND_Z_ORIGIN
char			g_nFindZOriginStatus		= 0;
long			g_nZOriginPosition[3]		= { 0, 0, 0 };
int				g_nZOriginSet				= 0;
char			g_abortSearch				= 0;
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_RGB_LIGHT_EFFECTS
unsigned char	g_uRGBHeatingR				= RGB_HEATING_R;
unsigned char	g_uRGBHeatingG				= RGB_HEATING_G;
unsigned char	g_uRGBHeatingB				= RGB_HEATING_B;
unsigned char	g_uRGBPrintingR				= RGB_PRINTING_R;
unsigned char	g_uRGBPrintingG				= RGB_PRINTING_G;
unsigned char	g_uRGBPrintingB				= RGB_PRINTING_B;
unsigned char	g_uRGBCoolingR				= RGB_COOLING_R;
unsigned char	g_uRGBCoolingG				= RGB_COOLING_G;
unsigned char	g_uRGBCoolingB				= RGB_COOLING_B;
unsigned char	g_uRGBIdleR					= RGB_IDLE_R;
unsigned char	g_uRGBIdleG					= RGB_IDLE_G;
unsigned char	g_uRGBIdleB					= RGB_IDLE_B;
unsigned char	g_uRGBManualR				= RGB_MANUAL_R;
unsigned char	g_uRGBManualG				= RGB_MANUAL_G;
unsigned char	g_uRGBManualB				= RGB_MANUAL_B;
unsigned char	g_uRGBCurrentR				= 0;
unsigned char	g_uRGBCurrentG				= 0;
unsigned char	g_uRGBCurrentB				= 0;
unsigned char	g_uRGBTargetR				= 0;
unsigned char	g_uRGBTargetG				= 0;
unsigned char	g_uRGBTargetB				= 0;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if DEBUG_HEAT_BED_Z_COMPENSATION
long			g_nLastZCompensationPositionSteps[3] = { 0, 0, 0 };
long			g_nLastZCompensationTargetStepsZ	 = 0;
long			g_nZCompensationUpdates				 = 0;
long			g_nDelta[2]							 = { 0, 0 };
long			g_nStepSize[2]						 = { 0, 0 };
long			g_nTempXFront						 = 0;
long			g_nTempXBack						 = 0;
long			g_nNeededZ							 = 0;
unsigned char	g_uIndex[4]							 = { 0, 0, 0, 0 };
short			g_nMatrix[4]						 = { 0, 0, 0, 0 };
long			g_nZDeltaMin						 = 100000;
long			g_nZDeltaMax						 = -100000;
long			g_nZCompensationUpdateTime			 = 0;
long			g_nZCompensationDelayMax			 = 0;
long			g_nTooFast							 = 0;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION

#if FEATURE_SERVICE_INTERVAL
unsigned long	g_nlastServiceTime	= 0;
int				g_nEnteredService	= 0;
#endif // FEATURE_SERVICE_INTERVAL


void initRF( void )
{
	// initialize the strain gauge
	initStrainGauge();

#if FEATURE_MILLING_MODE
	switchOperatingMode( Printer::operatingMode );
#else
	setupForPrinting();
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	if( Printer::ZEndstopType != ENDSTOP_TYPE_SINGLE )
	{
		if( Printer::isZMinEndstopHit() || Printer::isZMaxEndstopHit() )
		{
			// a z-endstop is active at the moment of the startup of the firmware, but both z-endstops are within one circuit so we do not know which one is the pressed one
			// in this situation we do not allow any moving into z-direction before a z-homing has been performed
			Printer::ZEndstopUnknown = 1;
		}
	}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_EMERGENCY_PAUSE
	g_nEmergencyPauseDigitsMin = EMERGENCY_PAUSE_DIGITS_MIN;
	g_nEmergencyPauseDigitsMax = EMERGENCY_PAUSE_DIGITS_MAX;
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_SERVICE_INTERVAL
	float	fDistanceService	 = Printer::filamentPrinted*0.001+HAL::eprGetFloat(EPR_PRINTING_DISTANCE_SERVICE);
	int32_t uSecondsServicePrint = ((HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME_SERVICE);
	int32_t uHoursServicePrint	 = uSecondsServicePrint/3600;
	int32_t uSecondsServiceMill	 = ((HAL::timeInMilliseconds()-Printer::msecondsMilling)/1000)+HAL::eprGetInt32(EPR_MILLING_TIME_SERVICE);
	int32_t uHoursServiceMill	 = uSecondsServiceMill/3600;
	
	if ( fDistanceService >= FILAMENT_PRINTED_UNTIL_SERVICE || uHoursServicePrint >= HOURS_PRINTED_UNTIL_SERVICE || uHoursServiceMill >= HOURS_MILLED_UNTIL_SERVICE )
	{
		UI_STATUS(UI_TEXT_SERVICE);
		BEEP_SERVICE_INTERVALL
		g_nlastServiceTime = HAL::timeInMilliseconds();
	}

#endif // FEATURE_SERVICE_INTERVAL

	return;

} // initRF


void initStrainGauge( void )
{
	// configure DMS #1 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( I2C_ADDRESS_STRAIN_GAUGE );
	Wire.write( 0x8C );
	Wire.endTransmission();
	return;

} // initStrainGauge


short readStrainGauge( unsigned char uAddress )
{
	unsigned char	Register;
	short			Result;


	Wire.beginTransmission( uAddress );
	Wire.requestFrom( (uint8_t)uAddress, (uint8_t)3 );
        
	Result =  Wire.read();
    Result =  Result << 8;
	Result += Wire.read();
        
	Register = Wire.read();
	Wire.endTransmission();

	return Result;

} // readStrainGauge


#if FEATURE_HEAT_BED_Z_COMPENSATION
void startHeatBedScan( void )
{
	if( g_nHeatBedScanStatus )
	{
		// abort the heat bed scan
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "startHeatBedScan(): the scan has been cancelled" ) );
		}
		g_abortZScan = 1;
	}
	else
	{
		if( PrintLine::linesCount )
		{
			// there is some printing in progress at the moment - do not start the heat bed scan in this case
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "startHeatBedScan(): the scan can not be started while the printing is in progress" ) );
			}
		}
		else
		{
			// start the heat bed scan
			g_nHeatBedScanStatus = 1;
			BEEP_START_HEAT_BED_SCAN

			// when the heat bed is scanned, the z-compensation must be disabled
			if( Printer::doHeatBedZCompensation )
			{
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "startHeatBedScan(): the z compensation has been disabled" ) );
				}
				resetZCompensation();
			}
		}
	}

	return;

} // startHeatBedScan


void scanHeatBed( void )
{
	static unsigned char	nIndexX;
	static unsigned char	nIndexY;
	static char				nIndexYDirection;
	static char				nRetry;
	static long				nX;
	static long				nY;
	static long				nZ;
	static long				nYDirection;
	static short			nContactPressure;
	unsigned char			nLastHeatBedScanStatus = g_nHeatBedScanStatus;
	short					nTempPressure;
	long					nTempPosition;


	// directions:
	// +x = to the right
	// -x = to the left
	// +y = heat bed moves to the front
	// -y = heat bed moves to the back
	// +z = heat bed moves down
	// -z = heat bed moves up

	if( g_abortZScan )
	{
		// the scan has been aborted
		g_abortZScan = 0;

		// avoid to crash the extruder against the heat bed during the following homing
		g_nZScanZPosition += moveZ( ZAXIS_STEPS_PER_MM *5 );

		// start at the home position
		Printer::homeAxis( true, true, true );

		// turn off the engines
		Printer::disableXStepper();
		Printer::disableYStepper();
		Printer::disableZStepper();

		// disable all heaters
		Extruder::setHeatedBedTemperature( 0, false );
		Extruder::setTemperatureForExtruder( 0, 0, false );

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "scanHeatBed(): the scan has been aborted" ) );
		}

		UI_STATUS_UPD( UI_TEXT_HEAT_BED_SCAN_ABORTED );
		BEEP_ABORT_HEAT_BED_SCAN

		// restore the compensation values from the EEPROM
		if( loadCompensationMatrix( EEPROM_SECTOR_SIZE ) )
		{
			// there is no valid compensation matrix available
			initCompensationMatrix();
		}

		g_nHeatBedScanStatus  = 0;
		g_nZScanZPosition	  = 0;
		g_nLastZScanZPosition = 0;
		return;
	}

	// show that we are active
	previousMillisCmd = HAL::timeInMilliseconds();

	if( g_nHeatBedScanStatus )
	{
		if( g_nHeatBedScanStatus != 130 )
		{
			// do not change the status text in case it shows "Align Extruders" at the moment
			UI_STATUS( UI_TEXT_HEAT_BED_SCAN );
		}

		if( g_retryZScan )
		{
			// we have to retry to scan the current position
			g_nHeatBedScanStatus = 45;
			g_retryZScan		 = 0;
		}

		switch( g_nHeatBedScanStatus )
		{
			case 1:
			{
				g_scanStartTime    = HAL::timeInMilliseconds();
				g_abortZScan	   = 0;
				nContactPressure   = 0;
				g_nTempDirectionZ  = 0;

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): the scan has been started" ) );
				}

				// clear all fields of the heat bed compensation matrix
				initCompensationMatrix();

				g_uZMatrixMaxX = 0;
				g_uZMatrixMaxY = 0;

				// output the currently used scan parameters
				outputScanParameters();

				g_nHeatBedScanStatus = 10;
				break;
			}
			case 10:
			{
				// the scan is performed with the left extruder
				Extruder::selectExtruderById( 0 );

				// start at the home position
				Printer::homeAxis( true, true, true );

				g_nHeatBedScanStatus = 15;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 15:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY )
				{
					// do not check too often
					break;
				}

				if( testExtruderTemperature() )
				{
					// we did not reach the proper temperature
					g_lastScanTime = HAL::timeInMilliseconds();
					break;
				}
				g_nHeatBedScanStatus = 20;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 20:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY )
				{
					// do not check too often
					break;
				}

				if( testHeatBedTemperature() )
				{
					// we did not reach the proper temperature
					g_lastScanTime = HAL::timeInMilliseconds();
					break;
				}
				g_nHeatBedScanStatus = 25;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 25:
			{
				// move to the first position
				Printer::enableZStepper();
				Printer::unsetAllSteppersDisabled();

				PrintLine::moveRelativeDistanceInSteps( g_nScanXStartSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				PrintLine::moveRelativeDistanceInSteps( 0, g_nScanYStartSteps, 0, 0, MAX_FEEDRATE_Y, true, true );

				g_nHeatBedScanStatus = 30;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 30:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY )
				{
					// do not check too early
					break;
				}

				if( readIdlePressure( &g_nFirstIdlePressure ) )
				{
					// we were unable to determine the idle pressure
					break;
				}

				g_nHeatBedScanStatus = 35;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 35:
			{
				nX				 = g_nScanXStartSteps;
				nY				 = g_nScanYStartSteps;
				nZ				 = 0;
				nYDirection		 = g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
				nIndexYDirection = 1;
				nIndexX			 = 2;
				nIndexY			 = 2;

				g_nMinPressureContact = g_nFirstIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nFirstIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nFirstIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nFirstIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nFirstIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nFirstIdlePressure + g_nScanIdlePressureDelta;

				// store also the version of this heat bed compensation matrix
#if DEBUG_REMEMBER_SCAN_PRESSURE
				g_ScanPressure[0][0]		= EEPROM_FORMAT;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

				g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;

				g_nHeatBedScanStatus = 40;
				break;
			}
			case 39:
			{
				nTempPosition = nX + g_nScanXStepSizeSteps;
				if( nTempPosition > g_nScanXMaxPositionSteps )
				{
					// we end up here when the scan is complete
					g_nHeatBedScanStatus = 60;
					break;
				}

				// move to the next x-position
				PrintLine::moveRelativeDistanceInSteps( g_nScanXStepSizeSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				nX += g_nScanXStepSizeSteps;
				nIndexX ++;

				if( nIndexX > COMPENSATION_MATRIX_MAX_X )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanHeatBed(): the x-dimension of the compensation matrix became too big: " ), nIndexX );
					}
					g_abortZScan = 1;
					break;
				}

				if( nIndexX > g_uZMatrixMaxX )
				{
					g_uZMatrixMaxX = nIndexX;
				}

				if( nYDirection > 0 )
				{
					// we were moving from the front to the back during this column, so we have to move from the back to the front during the next column
					nYDirection		 = -g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
					nIndexYDirection = -1;
				}
				else
				{
					// we were moving from the back to the front during this column, so we have to move from the front to the back during the next column
					nYDirection		 = g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
					nIndexYDirection = 1;
				}

				g_nHeatBedScanStatus = 40;
				break;
			}
			case 40:
			{
				// safety checks
				if( nX <= g_nScanXMaxPositionSteps )
				{
					// remember also the exact x-position of this row/column
#if DEBUG_REMEMBER_SCAN_PRESSURE
					g_ScanPressure[nIndexX][0] = nX;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

					g_ZCompensationMatrix[nIndexX][0] = (short)((float)nX / XAXIS_STEPS_PER_MM);	// convert to mm

					g_nHeatBedScanStatus = 49;
					g_lastScanTime		 = HAL::timeInMilliseconds();
					break;
				}

				// we end up here when the scan is complete
				g_nHeatBedScanStatus = 60;
				break;
			}
			case 45:
			{
				// home the z-axis in order to find the starting point again
				Printer::homeAxis( false, false, true );

				g_scanRetries		 --;
				g_nZScanZPosition	 = 0;
				nZ					 = 0;
				g_nHeatBedScanStatus = 50;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 49:
			{
				g_scanRetries		 = HEAT_BED_SCAN_RETRIES;
				g_nHeatBedScanStatus = 50;
				break;
			}
			case 50:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay )
				{
					// do not check too early
					break;
				}

				// scan this point
				if( testIdlePressure() )
				{
					// the current idle pressure is not plausible
					// break;
				}

				// we should consider that the idle presse can change slightly
				g_nMinPressureContact = g_nCurrentIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nCurrentIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nCurrentIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nCurrentIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nCurrentIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nCurrentIdlePressure + g_nScanIdlePressureDelta;

				g_nHeatBedScanStatus = 51;
				break;
			}
			case 51:
			{
				// move fast to the surface
				nZ += moveZUpFast();

				g_nHeatBedScanStatus = 52;
				break;
			}
			case 52:
			{
				// move a little bit away from the surface
				nZ += moveZDownSlow();

				g_nHeatBedScanStatus = 53;
				break;
			}
			case 53:
			{
				// move slowly to the surface
				nZ += moveZUpSlow( &nTempPressure, &nRetry );
				nContactPressure = nTempPressure;

				g_nHeatBedScanStatus = 54;
				break;
			}
			case 54:
			{
#if DEBUG_HEAT_BED_SCAN
				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( ";" ), nX );
					Com::printF( PSTR( ";" ), nY );
					Com::printF( PSTR( ";" ), nZ );
					Com::printF( PSTR( ";" ), nContactPressure );

					// output the non compensated position values
					Com::printF( PSTR( ";;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";" ), Printer::compensatedPositionCurrentStepsZ );

					Com::printFLN( PSTR( " " ) );
				}
#endif // DEBUG_HEAT_BED_SCAN

				// remember the z-position and the exact y-position of this row/column
				g_ZCompensationMatrix[nIndexX][nIndexY] = (short)nZ;
				g_ZCompensationMatrix[0][nIndexY]		= (short)((float)nY / YAXIS_STEPS_PER_MM);	// convert to mm

#if DEBUG_REMEMBER_SCAN_PRESSURE
				// remember the pressure and the exact y-position of this row/column
				g_ScanPressure[nIndexX][nIndexY] = nContactPressure;
				g_ScanPressure[0][nIndexY]		 = nY;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

				g_nHeatBedScanStatus = 55;
				break;
			}
			case 55:
			{
				// move away from the surface
				nZ += moveZDownFast();

				if( nYDirection > 0 )
				{
					nTempPosition = nY+nYDirection;

					if( nTempPosition > g_nScanYMaxPositionSteps )
					{
						// we have reached the end of this column
						g_nHeatBedScanStatus = 39;
						break;
					}
				}
				else
				{
					nTempPosition = nY+nYDirection;

					if( nTempPosition < g_nScanYStartSteps )
					{
						// we have reached the end of this column
						g_nHeatBedScanStatus = 39;
						break;
					}
				}

				// move to the next y-position
				PrintLine::moveRelativeDistanceInSteps( 0, nYDirection, 0, 0, MAX_FEEDRATE_Y, true, true );
				nY		+= nYDirection;
				nIndexY += nIndexYDirection;

				if( nIndexY > COMPENSATION_MATRIX_MAX_Y )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): the y-dimension of the compensation matrix became too big: " ), nIndexY );
					}
					g_abortZScan = 1;
					break;
				}

				if( nIndexY > g_uZMatrixMaxY )
				{
					g_uZMatrixMaxY = nIndexY;
				}
		
				g_nHeatBedScanStatus = 49;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 60:
			{
				// avoid to crash the extruder against the heat bed during the following homing
				g_nZScanZPosition += moveZ( ZAXIS_STEPS_PER_MM *5 );

				// move back to the home position
				Printer::homeAxis( true, true, true);

				// disable all heaters
				Extruder::setHeatedBedTemperature( 0, false );
				Extruder::setTemperatureForExtruder( 0, 0, false );

				g_nHeatBedScanStatus = 65;
				break;
			}
			case 65:
			{
				if( Printer::debugInfo() )
				{
					// output the determined compensation
					Com::printFLN( PSTR( "scanHeatBed(): raw heat bed compensation matrix: " ) );
					outputCompensationMatrix();
				}

				g_nHeatBedScanStatus = 70;
				break;
			}
			case 70:
			{
				// output the determined pressure
				outputPressureMatrix();

				g_nHeatBedScanStatus = 75;
				break;
			}
			case 75:
			{
				if( Printer::debugInfo() )
				{
					// output the pure scan time
					Com::printF( PSTR( "scanHeatBed(): total scan time: " ), long((HAL::timeInMilliseconds() - g_scanStartTime) / 1000) );
					Com::printFLN( PSTR( " [s]" ) );
				}

				// prepare the heat bed compensation matrix for fast usage during the actual printing
				prepareCompensationMatrix();

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): g_uZMatrixMaxY.1 = " ), (int)g_uZMatrixMaxY );
				}

				// convert the heat bed compensation matrix for fast usage during the actual printing
				convertCompensationMatrix();

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): g_uZMatrixMaxY.2 = " ), (int)g_uZMatrixMaxY );

					// output the converted heat bed compensation matrix
					Com::printFLN( PSTR( "scanHeatBed(): converted heat bed compensation matrix: " ) );
					outputCompensationMatrix();
				}

				// save the determined values to the EEPROM
				if( saveCompensationMatrix( EEPROM_SECTOR_SIZE ) )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanHeatBed(): the heat bed compensation matrix could not be saved" ) );
					}
				}
				else
				{
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "scanHeatBed(): the heat bed compensation matrix has been saved" ) );
					}
				}

				g_nHeatBedScanStatus = 80;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 80:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < HEAT_BED_SCAN_DELAY )
				{
					// do not check too early
					break;
				}

				// compare the idle pressure at the beginning and at the end
				readAveragePressure( &nTempPressure );

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): idle pressure at start: " ), g_nFirstIdlePressure );
					Com::printFLN( PSTR( "scanHeatBed(): idle pressure at stop: " ), nTempPressure );
				}

#if NUM_EXTRUDER == 2
				g_nHeatBedScanStatus = 100;
#else
				g_nHeatBedScanStatus = 150;
#endif // NUM_EXTRUDER == 2
				break;
			}
#if NUM_EXTRUDER == 2
			case 100:
			{
				// we are homed at the moment - move to the position where both extruders shall be calibrated to the same z position
				PrintLine::moveRelativeDistanceInSteps( HEAT_BED_SCAN_X_CALIBRATION_POINT_STEPS, HEAT_BED_SCAN_Y_CALIBRATION_POINT_STEPS, 0, 0, MAX_FEEDRATE_X, true, true );

				g_lastScanTime		 = HAL::timeInMilliseconds();
				g_nHeatBedScanStatus = 110;
				break;
			}
			case 110:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay )
				{
					// do not check too early
					break;
				}

				// scan this point
				if( testIdlePressure() )
				{
					// the current idle pressure is not plausible
					// break;
				}

				// we should consider that the idle presse can change slightly
				g_nMinPressureContact = g_nCurrentIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nCurrentIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nCurrentIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nCurrentIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nCurrentIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nCurrentIdlePressure + g_nScanIdlePressureDelta;

				g_nHeatBedScanStatus = 120;
				break;
			}
			case 120:
			{
				// move to the surface
				moveZUpFast();

				// the left extruder is at the surface now - show that the user must move also the right extruder to the surface in order to get them to the same z-height
				UI_STATUS_UPD( UI_TEXT_ALIGN_EXTRUDERS );
				BEEP_ALIGN_EXTRUDERS

				g_nContinueButtonPressed = 0;
				g_nHeatBedScanStatus	   = 130;
				break;
			}
			case 130:
			{
				// wait until the continue button has been pressed
				if( !g_nContinueButtonPressed )
				{
					break;
				}

				// avoid to crash the extruder against the heat bed during the following homing
				g_nZScanZPosition += moveZ( ZAXIS_STEPS_PER_MM *5 );

				// move back to the home position
				Printer::homeAxis( true, true, true);
				g_nHeatBedScanStatus = 150;
				break;
			}
#endif // NUM_EXTRUDER == 2

			case 150:
			{
				// turn off the engines
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): the scan has been completed" ) );
				}
				UI_STATUS_UPD(UI_TEXT_HEAT_BED_SCAN_DONE);
				BEEP_STOP_HEAT_BED_SCAN

				g_nHeatBedScanStatus = 0;
				break;
			}
		}
	}

	return;

} // scanHeatBed


short testExtruderTemperature( void )
{
	if( Extruder::current->tempControl.targetTemperatureC > 40 )
	{
		// we have to wait until the target temperature is reached
		if( (Extruder::current->tempControl.currentTemperatureC + 2) < Extruder::current->tempControl.targetTemperatureC )
		{
			// wait until the extruder has reached its target temperature
			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "testExtruderTemperature(): heating: " ), Extruder::current->tempControl.currentTemperatureC, 1 );
				Com::printF( PSTR( " C / " ), Extruder::current->tempControl.targetTemperatureC, 1 );
				Com::printFLN( PSTR( " C" ) );
			}

			return -1;
		}
	}
	else
	{
		// we have to wait until the current temperatur is below something which would be too warm
		if( Extruder::current->tempControl.currentTemperatureC > 65 )
		{
			// wait until the extruder has reached its target temperature
			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "testExtruderTemperature(): cooling: " ),Extruder::current->tempControl.currentTemperatureC, 1 );
				Com::printFLN( PSTR( " C" ) );
			}

			return -1;
		}
	}

	// at this point we have reached the proper temperature
	return 0;

} // testExtruderTemperature


short testHeatBedTemperature( void )
{
#if HAVE_HEATED_BED
	if( heatedBedController.targetTemperatureC > 40 )
	{
		// we have to wait until the target temperature is reached
		if( (Extruder::getHeatedBedTemperature() + TEMP_TOLERANCE) < heatedBedController.targetTemperatureC )
		{
			// wait until the heat bed has reached its target temperature
			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "testHeatBedTemperature(): heating: " ), Extruder::getHeatedBedTemperature(), 1 );
				Com::printF( PSTR( " C / " ), heatedBedController.targetTemperatureC, 1 );
				Com::printFLN( PSTR( " C" ) );
			}

			return -1;
		}
	}
	else
	{
		// we have to wait until the current temperatur is below something which would be too warm
		if( Extruder::getHeatedBedTemperature() > 50 )
		{
			// wait until the heat bed has reached its target temperature
			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "testHeatBedTemperature(): cooling: " ), Extruder::getHeatedBedTemperature(), 1 );
				Com::printFLN( PSTR( " C" ) );
			}

			return -1;
		}
	}
#endif // HAVE_HEATED_BED

	// at this point we have reached the proper temperature
	return 0;

} // testHeatBedTemperature


void doHeatBedZCompensation( void )
{
	long			nCurrentPositionSteps[3];
	unsigned char	nXLeftIndex;
	unsigned char	nXRightIndex;
	unsigned char	nYFrontIndex;
	unsigned char	nYBackIndex;
	long			nXLeftSteps;
	long			nXRightSteps;
	long			nYFrontSteps;
	long			nYBackSteps;
	long			nTemp;
	long			nDeltaX;
	long			nDeltaY;
	long			nDeltaZ;
	long			nStepSizeX;
	long			nStepSizeY;
	long			nNeededZCompensation;
	long			nTempXFront;
	long			nTempXBack;
	long			nTempZ;
	long			i;


	if( !Printer::doHeatBedZCompensation || (g_pauseStatus != PAUSE_STATUS_NONE && g_pauseStatus != PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE) )
	{
		// there is nothing to do at the moment
		return;
	}

	HAL::forbidInterrupts();
	nCurrentPositionSteps[X_AXIS] = Printer::queuePositionCurrentSteps[X_AXIS];
	nCurrentPositionSteps[Y_AXIS] = Printer::queuePositionCurrentSteps[Y_AXIS];
	nCurrentPositionSteps[Z_AXIS] = Printer::queuePositionCurrentSteps[Z_AXIS];
	HAL::allowInterrupts();

#if DEBUG_HEAT_BED_Z_COMPENSATION
	g_nLastZCompensationPositionSteps[X_AXIS] = nCurrentPositionSteps[X_AXIS];
	g_nLastZCompensationPositionSteps[Y_AXIS] = nCurrentPositionSteps[Y_AXIS];
	g_nLastZCompensationPositionSteps[Z_AXIS] = nCurrentPositionSteps[Z_AXIS];
#endif // DEBUG_HEAT_BED_Z_COMPENSATION
	
	if( nCurrentPositionSteps[Z_AXIS] > 0 )
	{
		// check whether we have to perform a compensation in z-direction
		if( nCurrentPositionSteps[Z_AXIS] < g_maxZCompensationSteps )
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			// find the rectangle which covers the current position of the extruder
			nXLeftIndex = 1;
			nXLeftSteps = (long)((float)g_ZCompensationMatrix[1][0] * XAXIS_STEPS_PER_MM);

			for( i=1; i<g_uZMatrixMaxX; i++ )
			{
				nTemp = g_ZCompensationMatrix[i][0];
				nTemp = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
				if( nCurrentPositionSteps[X_AXIS] <= nTemp )
				{
					nXRightIndex = i;
					nXRightSteps = nTemp;
					break;
				}
				nXLeftIndex = i;
				nXLeftSteps = nTemp;
			}
					
			nYFrontIndex = 1;
			nYFrontSteps = (long)((float)g_ZCompensationMatrix[0][1] * YAXIS_STEPS_PER_MM);

			for( i=1; i<g_uZMatrixMaxY; i++ )
			{
				nTemp = g_ZCompensationMatrix[0][i];
				nTemp = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
				if( nCurrentPositionSteps[Y_AXIS] <= nTemp )
				{
					nYBackIndex = i;
					nYBackSteps = nTemp;
					break;
				}
				nYFrontIndex = i;
				nYFrontSteps = nTemp;
			}

			nDeltaX    = nCurrentPositionSteps[X_AXIS] - nXLeftSteps;
			nDeltaY	   = nCurrentPositionSteps[Y_AXIS] - nYFrontSteps;
			nStepSizeX = nXRightSteps - nXLeftSteps;
			nStepSizeY = nYBackSteps - nYFrontSteps;

			// we do a linear interpolation in order to find our exact place within the current rectangle
			nTempXFront = g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex] +
						  (g_ZCompensationMatrix[nXRightIndex][nYFrontIndex] - g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex]) * nDeltaX / nStepSizeX;
			nTempXBack  = g_ZCompensationMatrix[nXLeftIndex][nYBackIndex] +
						  (g_ZCompensationMatrix[nXRightIndex][nYBackIndex] - g_ZCompensationMatrix[nXLeftIndex][nYBackIndex]) * nDeltaX / nStepSizeX;
			nNeededZCompensation = nTempXFront +
								   (nTempXBack - nTempXFront) * nDeltaY / nStepSizeY;

#if DEBUG_HEAT_BED_Z_COMPENSATION
			g_nDelta[X_AXIS]	= nDeltaX;
			g_nDelta[Y_AXIS]	= nDeltaY;
			g_nStepSize[X_AXIS] = nStepSizeX;
			g_nStepSize[Y_AXIS] = nStepSizeY;
			g_nTempXFront		= nTempXFront;
			g_nTempXBack		= nTempXBack;
			g_nNeededZ			= nNeededZCompensation;
			g_uIndex[0]			= nXLeftIndex;
			g_uIndex[1]			= nXRightIndex;
			g_uIndex[2]			= nYFrontIndex;
			g_uIndex[3]			= nYBackIndex;
			g_nMatrix[0]		= g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex];
			g_nMatrix[1]		= g_ZCompensationMatrix[nXRightIndex][nYFrontIndex];
			g_nMatrix[2]		= g_ZCompensationMatrix[nXLeftIndex][nYBackIndex];
			g_nMatrix[3]		= g_ZCompensationMatrix[nXRightIndex][nYBackIndex];
#endif // DEBUG_HEAT_BED_Z_COMPENSATION

			if( nCurrentPositionSteps[Z_AXIS] <= g_minZCompensationSteps )
			{
				// the printer is very close to the surface - we shall print a layer of exactly the desired thickness
				nNeededZCompensation += g_staticZSteps;
			}
			else
			{
				// the printer is already a bit away from the surface - do the actual compensation
				nDeltaZ = g_maxZCompensationSteps - nCurrentPositionSteps[Z_AXIS];
				nNeededZCompensation = g_offsetZCompensationSteps + 
									   (nNeededZCompensation - g_offsetZCompensationSteps) * nDeltaZ / (g_maxZCompensationSteps - g_minZCompensationSteps);
				nNeededZCompensation += g_staticZSteps;
			}
		}
		else
		{	
			// after the first layers, only the static offset to the surface must be compensated
			nNeededZCompensation = g_offsetZCompensationSteps + g_staticZSteps;
		}
	}
	else
	{
		// we do not perform a compensation in case the z-position from the G-code is 0 (because this would drive the extruder against the heat bed)
		nNeededZCompensation = g_staticZSteps;
	}

#if DEBUG_HEAT_BED_Z_COMPENSATION
	long	nZDelta = Printer::compensatedPositionTargetStepsZ - nNeededZCompensation;

	if( nZDelta < g_nZDeltaMin )		g_nZDeltaMin = nZDelta;
	if( nZDelta > g_nZDeltaMax )		g_nZDeltaMax = nZDelta;

	g_nZCompensationUpdateTime = micros();

	if( Printer::compensatedPositionTargetStepsZ != Printer::compensatedPositionCurrentStepsZ )
	{
		g_nTooFast ++;
	}
#endif // DEBUG_HEAT_BED_Z_COMPENSATION

	HAL::forbidInterrupts();
	Printer::compensatedPositionTargetStepsZ = nNeededZCompensation;
	HAL::allowInterrupts();

#if DEBUG_HEAT_BED_Z_COMPENSATION
	g_nLastZCompensationTargetStepsZ  =  nNeededZCompensation;
	g_nZCompensationUpdates ++;
#endif // DEBUG_HEAT_BED_Z_COMPENSATION

	return;

} // doHeatBedZCompensation
#endif // FEATURE_HEAT_BED_Z_COMPENSATION


#if FEATURE_WORK_PART_Z_COMPENSATION
void startWorkPartScan( char nMode )
{
	if( g_nWorkPartScanStatus )
	{
		// abort the work part scan
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "startWorkPartScan(): the scan has been cancelled" ) );
		}
		g_abortZScan = 1;
	}
	else
	{
		if( PrintLine::linesCount )
		{
			// there is some printing in progress at the moment - do not start the heat bed scan in this case
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "startWorkPartScan(): the scan can not be started while the milling is in progress" ) );
			}
		}
		else
		{
			// start the work part scan
			g_nWorkPartScanStatus = 1;
			g_nWorkPartScanMode	  = nMode;
			BEEP_START_WORK_PART_SCAN

			// when the work part is scanned, the z-compensation must be disabled
			if( Printer::doWorkPartZCompensation )
			{
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "startWorkPartScan(): the z compensation has been disabled" ) );
				}
				resetZCompensation();
			}
		}
	}

	return;

} // startWorkPartScan


void scanWorkPart( void )
{
	static unsigned char	nIndexX;
	static unsigned char	nIndexY;
	static char				nIndexYDirection;
	static char				nRetry;
	static long				nX;
	static long				nY;
	static long				nZ;
	static long				nYDirection;
	static short			nContactPressure;
	char					nLastWorkPartScanStatus = g_nWorkPartScanStatus;
	short					nTempPressure;
	long					nTempPosition;


	// directions:
	// +x = to the right
	// -x = to the left
	// +y = work part moves to the front
	// -y = work part moves to the back
	// +z = work part moves down
	// -z = work part moves up

	if( g_abortZScan )
	{
		// the scan has been aborted
		g_abortZScan = 0;

		// start at the home position
		if( g_nWorkPartScanMode )
		{
			// also the z-axis shall be homed
			Printer::homeAxis( true, true, true );
		}
		else
		{
			// the z-axis shall not be homed - in this case we must ensure that the tool does not crash against the limit stops at the front/left of the bed
			PrintLine::moveRelativeDistanceInSteps( 0, 0, WORK_PART_SCAN_Z_START_STEPS, 0, MAX_FEEDRATE_Z, true, true );
			Printer::homeAxis( true, true, false );
		}

		// turn off the engines
		Printer::disableXStepper();
		Printer::disableYStepper();
		Printer::disableZStepper();

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "scanWorkPart(): the scan has been aborted" ) );
		}

		UI_STATUS_UPD( UI_TEXT_WORK_PART_SCAN_ABORTED );
		BEEP_ABORT_WORK_PART_SCAN

		// restore the compensation values from the EEPROM
		if( loadCompensationMatrix( 0 ) )
		{
			// there is no valid compensation matrix available
			initCompensationMatrix();
		}

		g_nWorkPartScanStatus = 0;
		g_nZScanZPosition	  = 0;
		g_nLastZScanZPosition = 0;
		return;
	}

	// show that we are active
	previousMillisCmd = HAL::timeInMilliseconds();

	if( g_nWorkPartScanStatus )
	{
		UI_STATUS( UI_TEXT_WORK_PART_SCAN );

		if( g_retryZScan )
		{
			// we have to retry to scan the current position
			g_nWorkPartScanStatus = 45;
			g_retryZScan		  = 0;
		}

		switch( g_nWorkPartScanStatus )
		{
			case 1:
			{
				g_scanStartTime    = HAL::timeInMilliseconds();
				g_abortZScan	   = 0;
				nContactPressure   = 0;
				g_nTempDirectionZ  = 0;

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): the scan has been started" ) );
				}

				// clear all fields of the work part compensation matrix
				initCompensationMatrix();

				g_uZMatrixMaxX = 0;
				g_uZMatrixMaxY = 0;

				// output the currently used scan parameters
				outputScanParameters();

				g_nWorkPartScanStatus = 10;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 1 -> 10" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 10:
			{
				// start at the home position
				if( g_nWorkPartScanMode )
				{
					// also the z-axis shall be homed
					Printer::homeAxis( true, true, true );
				}
				else
				{
					// the z-axis shall not be homed - in this case we must ensure that the tool does not crash against the limit stops at the front/left of the bed
					PrintLine::moveRelativeDistanceInSteps( 0, 0, WORK_PART_SCAN_Z_START_STEPS, 0, MAX_FEEDRATE_Z, true, true );
					Printer::homeAxis( true, true, false );
				}

				g_nWorkPartScanStatus = 25;
				g_lastScanTime		  = HAL::timeInMilliseconds();

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 10 -> 25" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 25:
			{
				// move to the first position
				Printer::enableZStepper();
				Printer::unsetAllSteppersDisabled();

				PrintLine::moveRelativeDistanceInSteps( g_nScanXStartSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				PrintLine::moveRelativeDistanceInSteps( 0, g_nScanYStartSteps, 0, 0, MAX_FEEDRATE_Y, true, true );

				g_nWorkPartScanStatus = 30;
				g_lastScanTime		  = HAL::timeInMilliseconds();

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 25 -> 30" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 30:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < WORK_PART_SCAN_DELAY )
				{
					// do not check too early
					break;
				}

				if( readIdlePressure( &g_nFirstIdlePressure ) )
				{
					// we were unable to determine the idle pressure
					break;
				}

				g_nMinPressureContact = g_nFirstIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nFirstIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nFirstIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nFirstIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nFirstIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nFirstIdlePressure + g_nScanIdlePressureDelta;

				// prepare the direction of the z-axis (we have to move the milling bed up)
				prepareBedUp();
				g_nTempDirectionZ = -1;

				nX				 = g_nScanXStartSteps;
				nY				 = g_nScanYStartSteps;
				nZ				 = 0;
				nYDirection		 = g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
				nIndexYDirection = 1;
				nIndexX			 = 2;
				nIndexY			 = 2;

				// store also the version of this heat bed compensation matrix
#if DEBUG_REMEMBER_SCAN_PRESSURE
				g_ScanPressure[0][0] = EEPROM_FORMAT;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

				g_ZCompensationMatrix[0][0] = EEPROM_FORMAT;

				g_nWorkPartScanStatus = 32;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 30 -> 32" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 32:
			{
				short	nCurrentPressure;


				// move the heat bed up until we detect the contact pressure
				g_lastScanTime = HAL::timeInMilliseconds();
				while( 1 )
				{
#if FEATURE_WATCHDOG
					HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

					nCurrentPressure = readStrainGauge( ACTIVE_STRAIN_GAUGE );

					if( nCurrentPressure > g_nMaxPressureContact || nCurrentPressure < g_nMinPressureContact )
					{
						// we have reached the target pressure
						g_nWorkPartScanStatus = 33;

#if DEBUG_WORK_PART_SCAN
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): 32 -> 33" ) );
						}
#endif // DEBUG_WORK_PART_SCAN
						return;
					}

					if( Printer::isZMinEndstopHit() )
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): the z-min endstop has been reached" ) );
						}
						g_abortZScan = 1;
						return;
					}

					g_nZScanZPosition += moveZ( g_nScanHeatBedUpFastSteps );

					if( (HAL::timeInMilliseconds() - g_lastScanTime) > SEARCH_Z_ORIGIN_BREAKOUT_DELAY )
					{
						// do not stay within this loop forever
						return;
					}

					if( g_abortZScan )
					{
						break;
					}
				}

				// we should never end up here
				break;
			}
			case 33:
			{
				short	nCurrentPressure;


				// move the heat bed down again until we do not detect any contact anymore
				g_lastScanTime = HAL::timeInMilliseconds();
				while( 1 )
				{
#if FEATURE_WATCHDOG
					HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

					nCurrentPressure = readStrainGauge( ACTIVE_STRAIN_GAUGE );

					if( nCurrentPressure > g_nMinPressureContact && nCurrentPressure < g_nMaxPressureContact )
					{
						// we have reached the target pressure / we have found the z-origin
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): the z-origin has been determined" ) );
						}

						setZOrigin();

						// move away from the surface
						g_nZScanZPosition = nZ = moveZDownFast();
						g_nWorkPartScanStatus = 35;

						// ensure that we do not remember any previous z-position at this moment
						g_nLastZScanZPosition = 0;

#if DEBUG_WORK_PART_SCAN
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): 33 -> 35 > " ), nZ );
						}
#endif // DEBUG_WORK_PART_SCAN
						return;
					}

					if( Printer::isZMaxEndstopHit() )
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): the z-max endstop has been reached" ) );
						}
						g_abortZScan = 1;
						return;
					}

					g_nZScanZPosition += moveZ( g_nScanHeatBedDownSlowSteps );

					if( (HAL::timeInMilliseconds() - g_lastScanTime) > SEARCH_Z_ORIGIN_BREAKOUT_DELAY )
					{
						// do not stay within this loop forever
						return;
					}

					if( g_abortZScan )
					{
						break;
					}
				}

				// we should never end up here
				break;
			}
			case 35:
			{
				g_nWorkPartScanStatus = 40;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 35 -> 40" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 39:
			{
				nTempPosition = nX + g_nScanXStepSizeSteps;
				if( nTempPosition > g_nScanXMaxPositionSteps )
				{
					// we end up here when the scan is complete
					g_nWorkPartScanStatus = 60;

#if DEBUG_WORK_PART_SCAN
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): 39 -> 60" ) );
					}
#endif // DEBUG_WORK_PART_SCAN
					break;
				}

				// move to the next x-position
				PrintLine::moveRelativeDistanceInSteps( g_nScanXStepSizeSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				nX += g_nScanXStepSizeSteps;
				nIndexX ++;

				if( nIndexX > COMPENSATION_MATRIX_MAX_X )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): the x-dimension of the compensation matrix became too big: " ), nIndexX );
					}
					g_abortZScan = 1;
					break;
				}

				if( nIndexX > g_uZMatrixMaxX )
				{
					g_uZMatrixMaxX = nIndexX;
				}

				if( nYDirection > 0 )
				{
					// we were moving from the front to the back during this column, so we have to move from the back to the front during the next column
					nYDirection		 = -g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
					nIndexYDirection = -1;
				}
				else
				{
					// we were moving from the back to the front during this column, so we have to move from the front to the back during the next column
					nYDirection		 = g_nScanYStepSizeSteps;	// we start to move the heat bed from the back to the front
					nIndexYDirection = 1;
				}

				g_nWorkPartScanStatus = 40;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 39 -> 40" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 40:
			{
				// safety checks
				if( nX <= g_nScanXMaxPositionSteps )
				{
					// remember also the exact x-position of this row/column
#if DEBUG_REMEMBER_SCAN_PRESSURE
					g_ScanPressure[nIndexX][0] = nX;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

					g_ZCompensationMatrix[nIndexX][0] = (short)((float)nX / XAXIS_STEPS_PER_MM);	// convert to mm

					g_nWorkPartScanStatus = 49;
					g_lastScanTime		  = HAL::timeInMilliseconds();

#if DEBUG_WORK_PART_SCAN
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): 40 -> 49" ) );
					}
#endif // DEBUG_WORK_PART_SCAN
					break;
				}

				// we end up here when the scan is complete
				g_nWorkPartScanStatus = 60;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 40 -> 60" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 45:
			{
				// move away from the surface
				nZ += moveZ( g_nScanHeatBedDownFastSteps );
				g_nZScanZPosition = nZ;

				g_scanRetries		  --;
				g_nWorkPartScanStatus = 46;
				g_lastScanTime		  = HAL::timeInMilliseconds();

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 45 -> 46 > " ), nZ );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 46:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < WORK_PART_SCAN_DELAY )
				{
					// do not check too early
					break;
				}

				// try to determine the idle pressure again
				if( readIdlePressure( &g_nFirstIdlePressure ) )
				{
					// we were unable to determine the idle pressure
					break;
				}

				g_nMinPressureContact = g_nFirstIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nFirstIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nFirstIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nFirstIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nFirstIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nFirstIdlePressure + g_nScanIdlePressureDelta;

				g_nWorkPartScanStatus = 50;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 46 -> 50" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 49:
			{
				g_scanRetries		  = WORK_PART_SCAN_RETRIES;
				g_nWorkPartScanStatus = 50;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 49 -> 50" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 50:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < g_nScanIdleDelay )
				{
					// do not check too early
					break;
				}

				// scan this point
				if( testIdlePressure() )
				{
					// the current idle pressure is not plausible
					// break;
				}

				// we should consider that the idle presse can change slightly
				g_nMinPressureContact = g_nCurrentIdlePressure - g_nScanContactPressureDelta;
				g_nMaxPressureContact = g_nCurrentIdlePressure + g_nScanContactPressureDelta;
				g_nMinPressureRetry	  = g_nCurrentIdlePressure - g_nScanRetryPressureDelta;
				g_nMaxPressureRetry   = g_nCurrentIdlePressure + g_nScanRetryPressureDelta;
				g_nMinPressureIdle	  = g_nCurrentIdlePressure - g_nScanIdlePressureDelta;
				g_nMaxPressureIdle	  = g_nCurrentIdlePressure + g_nScanIdlePressureDelta;

				g_nWorkPartScanStatus = 51;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 50 -> 51" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 51:
			{
				// move fast to the surface
				nZ += moveZUpFast();

				g_nWorkPartScanStatus = 52;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 51 -> 52 > " ), nZ );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 52:
			{
				// move a little bit away from the surface
				nZ += moveZDownSlow();

				g_nWorkPartScanStatus = 53;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 52 -> 53 > " ), nZ );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 53:
			{
				// move slowly to the surface
				nZ += moveZUpSlow( &nTempPressure, &nRetry );

				nContactPressure	  = nTempPressure;
				g_nWorkPartScanStatus = 54;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 53 -> 54 > " ), nZ );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 54:
			{
#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( ";" ), nX );
					Com::printF( PSTR( ";" ), nY );
					Com::printF( PSTR( ";" ), nZ );
					Com::printF( PSTR( ";" ), nContactPressure );

					// output the non compensated position values
					Com::printF( PSTR( ";;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printFLN( PSTR( ";" ), Printer::compensatedPositionCurrentStepsZ );
				}
#endif // DEBUG_WORK_PART_SCAN

				// remember the z-position and the exact y-position of this row/column
				g_ZCompensationMatrix[nIndexX][nIndexY] = (short)nZ;
				g_ZCompensationMatrix[0][nIndexY]		= (short)((float)nY / YAXIS_STEPS_PER_MM);	// convert to mm

#if DEBUG_REMEMBER_SCAN_PRESSURE
				// remember the pressure and the exact y-position of this row/column
				g_ScanPressure[nIndexX][nIndexY] = nContactPressure;
				g_ScanPressure[0][nIndexY]		 = nY;
#endif // DEBUG_REMEMBER_SCAN_PRESSURE

				g_nWorkPartScanStatus = 55;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 54 -> 55 > " ), nZ );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 55:
			{
				// move away from the surface
				nZ += moveZDownFast();

				if( nYDirection > 0 )
				{
					nTempPosition = nY+nYDirection;

					if( nTempPosition > g_nScanYMaxPositionSteps )
					{
						// we have reached the end of this column
						g_nWorkPartScanStatus = 39;

#if DEBUG_WORK_PART_SCAN
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): 55 -> 39" ) );
						}
#endif // DEBUG_WORK_PART_SCAN
						break;
					}
				}
				else
				{
					nTempPosition = nY+nYDirection;

					if( nTempPosition < g_nScanYStartSteps )
					{
						// we have reached the end of this column
						g_nWorkPartScanStatus = 39;

#if DEBUG_WORK_PART_SCAN
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "scanWorkPart(): 55 -> 39" ) );
						}
#endif // DEBUG_WORK_PART_SCAN
						break;
					}
				}

				// move to the next y-position
				PrintLine::moveRelativeDistanceInSteps( 0, nYDirection, 0, 0, MAX_FEEDRATE_Y, true, true );
				nY		+= nYDirection;
				nIndexY += nIndexYDirection;

				if( nIndexY > COMPENSATION_MATRIX_MAX_Y )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): the y-dimension of the compensation matrix became too big: " ), nIndexY );
					}
					g_abortZScan = 1;
					break;
				}

				if( nIndexY > g_uZMatrixMaxY )
				{
					g_uZMatrixMaxY = nIndexY;
				}
		
				g_nWorkPartScanStatus = 49;
				g_lastScanTime		  = HAL::timeInMilliseconds();

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 55 -> 49" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 60:
			{
				// move back to the home position
				if( g_nWorkPartScanMode )
				{
					// also the z-axis shall be homed
					Printer::homeAxis( true, true, true );
				}
				else
				{
					// the z-axis shall not be homed - in this case we must ensure that the tool does not crash against the limit stops at the front/left of the bed
					PrintLine::moveRelativeDistanceInSteps( 0, 0, WORK_PART_SCAN_Z_START_STEPS, 0, MAX_FEEDRATE_Z, true, true );
					Printer::homeAxis( true, true, false );
				}

				// turn off the engines
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();

				g_nWorkPartScanStatus = 65;

#if DEBUG_WORK_PART_SCAN
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): 60 -> 65" ) );
				}
#endif // DEBUG_WORK_PART_SCAN
				break;
			}
			case 65:
			{
				if( Printer::debugInfo() )
				{
					// output the determined compensation
					Com::printFLN( PSTR( "scanWorkPart(): raw work part compensation matrix: " ) );
					outputCompensationMatrix();
				}

				g_nWorkPartScanStatus = 70;
				break;
			}
			case 70:
			{
				// output the determined pressure
				outputPressureMatrix();

				g_nWorkPartScanStatus = 75;
				break;
			}
			case 75:
			{
				if( Printer::debugInfo() )
				{
					// output the pure scan time
					Com::printF( PSTR( "scanWorkPart(): total scan time: " ), long((HAL::timeInMilliseconds() - g_scanStartTime) / 1000) );
					Com::printFLN( PSTR( " [s]" ) );
				}

				// prepare the work part compensation matrix for fast usage during the actual milling
				prepareCompensationMatrix();

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): g_uZMatrixMaxY.1 = " ), (int)g_uZMatrixMaxY );
				}

				// convert the work part compensation matrix for fast usage during the actual printing
				convertCompensationMatrix();

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): g_uZMatrixMaxY.2 = " ), (int)g_uZMatrixMaxY );

					// output the converted work part compensation matrix
					Com::printFLN( PSTR( "scanWorkPart(): converted work part compensation matrix: " ) );
					outputCompensationMatrix();
				}

				// save the determined values to the EEPROM
				if( saveCompensationMatrix( EEPROM_SECTOR_SIZE + (unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveWorkPart) ) )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): the work part compensation matrix could not be saved" ) );
					}
				}
				else
				{
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "scanWorkPart(): the work part compensation matrix has been saved > " ), g_nActiveWorkPart );
					}
				}

				g_nWorkPartScanStatus = 80;
				g_lastScanTime		  = HAL::timeInMilliseconds();
				break;
			}
			case 80:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < WORK_PART_SCAN_DELAY )
				{
					// do not check too early
					break;
				}

				// compare the idle pressure at the beginning and at the end
				readAveragePressure( &nTempPressure );

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): idle pressure at start: " ), g_nFirstIdlePressure );
					Com::printFLN( PSTR( "scanWorkPart(): idle pressure at stop: " ), nTempPressure );
				}

				g_nWorkPartScanStatus = 100;
				break;
			}
			case 100:
			{
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanWorkPart(): the scan has been completed" ) );
				}
				UI_STATUS_UPD(UI_TEXT_WORK_PART_SCAN_DONE);
				BEEP_STOP_WORK_PART_SCAN

				g_nWorkPartScanStatus = 0;
				break;
			}
		}
	}

	return;

} // scanWorkPart


void doWorkPartZCompensation( void )
{
	long			nCurrentPositionSteps[3];
	unsigned char	nXLeftIndex;
	unsigned char	nXRightIndex;
	unsigned char	nYFrontIndex;
	unsigned char	nYBackIndex;
	long			nXLeftSteps;
	long			nXRightSteps;
	long			nYFrontSteps;
	long			nYBackSteps;
	long			nTemp;
	long			nDeltaX;
	long			nDeltaY;
	long			nDeltaZ;
	long			nStepSizeX;
	long			nStepSizeY;
	long			nNeededZCompensation;
	long			nTempXFront;
	long			nTempXBack;
	long			nTempZ;
	long			i;


	if( !Printer::doWorkPartZCompensation || (g_pauseStatus != PAUSE_STATUS_NONE && g_pauseStatus != PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE) )
	{
		// there is nothing to do at the moment
		return;
	}

	HAL::forbidInterrupts();
	nCurrentPositionSteps[X_AXIS] = Printer::queuePositionCurrentSteps[X_AXIS];
	nCurrentPositionSteps[Y_AXIS] = Printer::queuePositionCurrentSteps[Y_AXIS];
	nCurrentPositionSteps[Z_AXIS] = Printer::queuePositionCurrentSteps[Z_AXIS];
	HAL::allowInterrupts();

	if( nCurrentPositionSteps[Z_AXIS] )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		// find the rectangle which covers the current position of the miller
		nXLeftIndex = 1;
		nXLeftSteps = (long)((float)g_ZCompensationMatrix[1][0] * XAXIS_STEPS_PER_MM);
		for( i=1; i<g_uZMatrixMaxX; i++ )
		{
			nTemp = g_ZCompensationMatrix[i][0];
			nTemp = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
			if( nCurrentPositionSteps[X_AXIS] <= nTemp )
			{
				nXRightIndex = i;
				nXRightSteps = nTemp;
				break;
			}
			nXLeftIndex = i;
			nXLeftSteps = nTemp;
		}
					
		nYFrontIndex = 1;
		nYFrontSteps = (long)((float)g_ZCompensationMatrix[0][1] * YAXIS_STEPS_PER_MM);
		for( i=1; i<g_uZMatrixMaxY; i++ )
		{
			nTemp = g_ZCompensationMatrix[0][i];
			nTemp = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
			if( nCurrentPositionSteps[Y_AXIS] <= nTemp )
			{
				nYBackIndex = i;
				nYBackSteps = nTemp;
				break;
			}
			nYFrontIndex = i;
			nYFrontSteps = nTemp;
		}

		nDeltaX    = nCurrentPositionSteps[X_AXIS] - nXLeftSteps;
		nDeltaY	   = nCurrentPositionSteps[Y_AXIS] - nYFrontSteps;
		nStepSizeX = nXRightSteps - nXLeftSteps;
		nStepSizeY = nYBackSteps - nYFrontSteps;

		// we do a linear interpolation in order to find our exact place within the current rectangle
		nTempXFront = g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex] +
					  (g_ZCompensationMatrix[nXRightIndex][nYFrontIndex] - g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex]) * nDeltaX / nStepSizeX;
		nTempXBack  = g_ZCompensationMatrix[nXLeftIndex][nYBackIndex] +
					  (g_ZCompensationMatrix[nXRightIndex][nYBackIndex] - g_ZCompensationMatrix[nXLeftIndex][nYBackIndex]) * nDeltaX / nStepSizeX;
		nNeededZCompensation = nTempXFront +
							   (nTempXBack - nTempXFront) * nDeltaY / nStepSizeY;
					
		nNeededZCompensation += 
#if FEATURE_FIND_Z_ORIGIN
								Printer::staticCompensationZ +
#endif // FEATURE_FIND_Z_ORIGIN
								g_staticZSteps;
	}
	else
	{
		// we do not perform a compensation in case the z-position from the G-code is 0 (because this would drive the tool against the work part)
		nNeededZCompensation = g_staticZSteps;
	}

	HAL::forbidInterrupts();
	Printer::compensatedPositionTargetStepsZ = nNeededZCompensation;
	HAL::allowInterrupts();

	return;

} // doWorkPartZCompensation


void determineStaticCompensationZ( void )
{
	long	nXLeftIndex;
	long	nXRightIndex;
	long	nYFrontIndex;
	long	nYBackIndex;
	long	nXLeftSteps;
	long	nXRightSteps;
	long	nYFrontSteps;
	long	nYBackSteps;
	long	nTemp;
	long	nDeltaX;
	long	nDeltaY;
	long	nDeltaZ;
	long	nStepSizeX;
	long	nStepSizeY;
	long	nTempXFront;
	long	nTempXBack;
	long	i;


	// find the rectangle which covers the current position of the miller
	nXLeftIndex = 1;
	nXLeftSteps = (long)((float)g_ZCompensationMatrix[1][0] * XAXIS_STEPS_PER_MM);
	for( i=1; i<g_uZMatrixMaxX; i++ )
	{
		nTemp = g_ZCompensationMatrix[i][0];
		nTemp = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
		if( Printer::queuePositionCurrentSteps[X_AXIS] <= nTemp )
		{
			nXRightIndex = i;
			nXRightSteps = nTemp;
			break;
		}
		nXLeftIndex = i;
		nXLeftSteps = nTemp;
	}
					
	nYFrontIndex = 1;
	nYFrontSteps = (long)((float)g_ZCompensationMatrix[0][1] * YAXIS_STEPS_PER_MM);
	for( i=1; i<g_uZMatrixMaxY; i++ )
	{
		nTemp = g_ZCompensationMatrix[0][i];
		nTemp = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
		if( Printer::queuePositionCurrentSteps[Y_AXIS] <= nTemp )
		{
			nYBackIndex = i;
			nYBackSteps = nTemp;
			break;
		}
		nYFrontIndex = i;
		nYFrontSteps = nTemp;
	}

	nDeltaX    = Printer::queuePositionCurrentSteps[X_AXIS] - nXLeftSteps;
	nDeltaY	   = Printer::queuePositionCurrentSteps[Y_AXIS] - nYFrontSteps;
	nStepSizeX = nXRightSteps - nXLeftSteps;
	nStepSizeY = nYBackSteps - nYFrontSteps;

	// we do a linear interpolation in order to find our exact place within the current rectangle
	nTempXFront = g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex] +
			 	  (g_ZCompensationMatrix[nXRightIndex][nYFrontIndex] - g_ZCompensationMatrix[nXLeftIndex][nYFrontIndex]) * nDeltaX / nStepSizeX;
	nTempXBack  = g_ZCompensationMatrix[nXLeftIndex][nYBackIndex] +
				  (g_ZCompensationMatrix[nXRightIndex][nYBackIndex] - g_ZCompensationMatrix[nXLeftIndex][nYBackIndex]) * nDeltaX / nStepSizeX;
		
	Printer::staticCompensationZ = nTempXFront +
								   (nTempXBack - nTempXFront) * nDeltaY / nStepSizeY;
	
	g_debugLog = 1;
	return;

} // determineStaticCompensationZ

#endif // FEATURE_WORK_PART_Z_COMPENSATION


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
short readIdlePressure( short* pnIdlePressure )
{
	short	nTempPressure;


	// determine the pressure when the heat bed is far away - wait until the measured pressure is rather stable
	nTempPressure	= 0;
	if( readAveragePressure( pnIdlePressure ) )
	{
		// we were unable to determine the pressure
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "readIdlePressure(): the pressure could not be determined" ) );
		}
		return -1;
	}

	while( abs( nTempPressure - *pnIdlePressure) > 5 )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if( Printer::debugInfo() )
		{
			Com::printF( PSTR( "readIdlePressure(): pressure calibration: " ), nTempPressure );
			Com::printFLN( PSTR( " / " ), *pnIdlePressure );
		}

		HAL::delayMilliseconds( 500 );

		nTempPressure = *pnIdlePressure;
		if( readAveragePressure( pnIdlePressure ) )
		{
			// we were unable to determine the pressure
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "readIdlePressure(): the pressure could not be determined" ) );
			}
			return -1;
		}
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "readIdlePressure(): idle pressure: " ), *pnIdlePressure );
	}

	if( *pnIdlePressure < g_nScanIdlePressureMin || *pnIdlePressure > g_nScanIdlePressureMax )
	{
		// the idle pressure is out of range
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "readIdlePressure(): the idle pressure is out of range" ) );
		}
		return -1;
	}

	// at this point we know the idle pressure
	return 0;

} // readIdlePressure


short testIdlePressure( void )
{
	short	nTempPressure;
	short	nTemp;


	if( readAveragePressure( &nTempPressure ) )
	{
		// some error has occurred
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "testIdlePressure(): the pressure could not be determined" ) );
		}
		return -1;
	}
	g_nCurrentIdlePressure = nTempPressure;
	return 0;

	nTemp = 0;
	while( 1 )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		HAL::delayMilliseconds( g_nScanSlowStepDelay );
		if( readAveragePressure( &nTempPressure ) )
		{
			// some error has occurred
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "testIdlePressure(): the pressure could not be determined" ) );
			}
			return -1;
		}

		if( nTempPressure < g_nMaxPressureIdle && nTempPressure > g_nMinPressureIdle )
		{
			// we have reached the target pressure
			g_nCurrentIdlePressure = nTempPressure;
			return 0;
		}

		nTemp ++;
		if( nTemp > 3 )
		{
			// it does not make sense to try this forever
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "testIdlePressure(): the idle pressure went out of range: " ), nTempPressure );
			}
			g_abortZScan = 1;
			return -1;
		}
	}

	// we should never end up here
	g_abortZScan = 1;
	return -1;

} // testIdlePressure


short readAveragePressure( short* pnAveragePressure )
{
	short	i;
	short	nTempPressure;
	short	nMinPressure;
	short	nMaxPressure;
	long	nPressureSum;
	char	nTemp;


	nTemp = 0;
	while( 1 )
	{
		// we read the strain gauge multiple times and check the variance
		nPressureSum = 0;
		nMinPressure = 32000;
		nMaxPressure = -32000;
		for( i=0; i<g_nScanPressureReads; i++)
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			HAL::delayMilliseconds( g_nScanPressureReadDelay );
			nTempPressure =  readStrainGauge( ACTIVE_STRAIN_GAUGE );
			nPressureSum  += nTempPressure;
			if( nTempPressure < nMinPressure )	nMinPressure = nTempPressure;
			if( nTempPressure > nMaxPressure )	nMaxPressure = nTempPressure;
		}
		nTempPressure = (short)(nPressureSum / g_nScanPressureReads);

		if( (nMaxPressure - nMinPressure) < g_nScanPressureTolerance )
		{
			// we have good results
			*pnAveragePressure = nTempPressure;
			return 0;
		}

		nTemp ++;
		if( nTemp >= 5 )
		{
			// we are unable to receive stable values - do not hang here forever
			if( Printer::debugErrors() )
			{
				Com::printF( PSTR( "readAveragePressure(): the pressure is not constant: " ), nMinPressure );
				Com::printF( PSTR( " / " ), nTempPressure );
				Com::printFLN( PSTR( " / " ), nMaxPressure );
			}
			break;
		}
	
		// wait some extra amount of time in case our results were not constant enough
		HAL::delayMilliseconds( 100 );
		
		//runStandardTasks();
		Commands::checkForPeriodicalActions(); 
	}

	if( Printer::debugErrors() )
	{
		Com::printFLN( PSTR( "readAveragePressure(): the pressure is not plausible" ) );
	}
	g_abortZScan	   = 1;
	*pnAveragePressure = 0;
	return -1;

} // readAveragePressure


short moveZUpFast( void )
{
	short	nTempPressure;
	short	nZ = 0;
	short	nSteps;


	// move the heat bed up until we detect the contact pressure (fast speed)
	while( 1 )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		HAL::delayMilliseconds( g_nScanFastStepDelay );
		if( readAveragePressure( &nTempPressure ) )
		{
			// some error has occurred
			break;
		}

		if( nTempPressure > g_nMaxPressureContact || nTempPressure < g_nMinPressureContact )
		{
			// we have reached the target pressure
			break;
		}

		nSteps			  =  moveZ( g_nScanHeatBedUpFastSteps );
		nZ				  += nSteps;
		g_nZScanZPosition += nSteps;

		runStandardTasks();

		if( g_abortZScan )
		{
			break;
		}

		if( g_nZScanZPosition < -g_nScanZMaxCompensationSteps || g_nZScanZPosition > g_nScanZMaxCompensationSteps )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZUpFast(): the z position went out of range, retries = " ), (int)g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryZScan = 1;
			else				g_abortZScan = 1;
			break;
		}
	}

	return nZ;

} // moveZUpFast


short moveZDownSlow( void )
{
	short	nTempPressure;
	short	nZ = 0;
	short	nSteps;


	// move the heat bed down until we detect the retry pressure (slow speed)
	while( 1 )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		HAL::delayMilliseconds( g_nScanSlowStepDelay );
		if( readAveragePressure( &nTempPressure ) )
		{
			// some error has occurred
			break;
		}

		if( nTempPressure < g_nMaxPressureRetry && nTempPressure > g_nMinPressureRetry )
		{
			// we have reached the target pressure
			break;
		}

		nSteps			  =  moveZ( g_nScanHeatBedDownSlowSteps );
		nZ				  += nSteps;
		g_nZScanZPosition += nSteps;
		runStandardTasks();

		if( g_abortZScan )
		{
			break;
		}

		if( g_nZScanZPosition < -g_nScanZMaxCompensationSteps || g_nZScanZPosition > g_nScanZMaxCompensationSteps )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZDownSlow(): the z position went out of range, retries = " ), g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryZScan = 1;
			else				g_abortZScan = 1;
			break;
		}
		if( g_nLastZScanZPosition )
		{
			if( (g_nZScanZPosition > g_nLastZScanZPosition && (g_nZScanZPosition - g_nLastZScanZPosition) > g_nScanHeatBedDownFastSteps) ||
				(g_nZScanZPosition < g_nLastZScanZPosition && (g_nLastZScanZPosition - g_nZScanZPosition) > g_nScanHeatBedDownFastSteps) )
			{
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "moveZDownSlow(): the z position delta went out of range, retries = " ), g_scanRetries );
				}
			
				if( g_scanRetries )	g_retryZScan = 1;
				else				g_abortZScan = 1;
				break;
			}
		}
	}

	return nZ;

} // moveZDownSlow


short moveZUpSlow( short* pnContactPressure, char* pnRetry )
{
	short	nTempPressure;
	short	nZ = 0;
	short	nSteps;


	// move the heat bed up until we detect the contact pressure (slow speed)
	while( 1 )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		HAL::delayMilliseconds( g_nScanSlowStepDelay );
		if( readAveragePressure( &nTempPressure ) )
		{
			// some error has occurred
			break;
		}

		if( nTempPressure > g_nMaxPressureContact || nTempPressure < g_nMinPressureContact )
		{
			// we have found the proper pressure
			break;
		}

		nSteps			  =  moveZ( g_nScanHeatBedUpSlowSteps );
		nZ			  	  += nSteps;
		g_nZScanZPosition += nSteps;
		runStandardTasks();

		if( g_abortZScan )
		{
			break;
		}

		if( g_nZScanZPosition < -g_nScanZMaxCompensationSteps || g_nZScanZPosition > g_nScanZMaxCompensationSteps )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZUpSlow(): the z position went out of range, retries = " ), g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryZScan = 1;
			else				g_abortZScan = 1;
			break;
		}
	}

	*pnContactPressure = nTempPressure;
	return nZ;

} // moveZUpSlow


short moveZDownFast( void )
{
	short	nTempPressure;
	short	nZ = 0;
	short	nSteps;


	// move the heat bed down so that we won't hit it when we move to the next position
	g_nLastZScanZPosition = g_nZScanZPosition;
	HAL::delayMilliseconds( g_nScanFastStepDelay );

	nSteps			  =  moveZ( g_nScanHeatBedDownFastSteps );
	nZ				  += nSteps;
	g_nZScanZPosition += nSteps;
	runStandardTasks();

	if( readAveragePressure( &nTempPressure ) )
	{
		// some error has occurred
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "moveZDownFast(): the pressure could not be determined" ) );
		}
		g_abortZScan = 1;
		return nZ;
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "moveZDownFast(): " ), (int)nTempPressure );
	}
	return nZ;

} // moveZDownFast


int moveZ( int nSteps )
{
	int		i;
	int		nMaxLoops;
	char	bBreak;
	char	nRun = 1;
	

	// Warning: this function does not check any end stops
	// choose the direction
	if( nSteps >= 0 )
	{
		nMaxLoops = nSteps;

		if( g_nTempDirectionZ != 1 )
		{
			prepareBedDown();

			HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
			g_nTempDirectionZ = 1;
		}
	}
	else
	{
		nMaxLoops = -nSteps;

		if( g_nTempDirectionZ != -1 )
		{
			prepareBedUp();

			HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
			g_nTempDirectionZ = -1;
		}
	}
	
	// perform the steps
	for( i=0; i<nMaxLoops; i++ )
	{
		bBreak = 0;
		nRun   --;

		if( !nRun )
		{
			// process the standard commands from time to time also while the moving in z-direction is in progress
			// runStandardTasks();
			nRun = 10;
//			Com::printF( PSTR( "moveZ(): " ), i );
//			Com::printFLN( PSTR( " / " ), nMaxLoops );
		}

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
		if( g_abortZScan )
		{
			bBreak = 1;
		}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
		
#if FEATURE_FIND_Z_ORIGIN
		if( g_abortSearch )
		{
			bBreak = 1;
		}
#endif // FEATURE_FIND_Z_ORIGIN

		if( bBreak )
		{
			// do not continue here in case the current operation has been cancelled
			if( nSteps > 0 )	nSteps = nMaxLoops;
			else				nSteps = -nMaxLoops;
			break;
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::delayMicroseconds( XYZ_STEPPER_HIGH_DELAY );
		startZStep( g_nTempDirectionZ );

        HAL::delayMicroseconds( XYZ_STEPPER_LOW_DELAY );
		endZStep();
	}

	return nSteps;

} // moveZ


int moveExtruder( int nSteps )
{
	int		i;
	int		nMaxLoops;
	
	
	HAL::forbidInterrupts();
	Extruder::enable();
    HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);

	// choose the direction
	if( nSteps >= 0 )
	{
		nMaxLoops = nSteps;
		Extruder::setDirection(true);
	}
	else
	{
		nMaxLoops = -nSteps;
		Extruder::setDirection(false);
	}

	HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);

	// perform the steps
	for( i=0; i<nMaxLoops; i++ )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::delayMicroseconds(EXTRUDER_STEPPER_HIGH_DELAY);
		Extruder::step();

        HAL::delayMicroseconds(EXTRUDER_STEPPER_LOW_DELAY);
		Extruder::unstep();
	}

	HAL::allowInterrupts();
	return nSteps;
	
} // moveExtruder


void restoreDefaultScanParameters( void )
{
#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
#if FEATURE_HEAT_BED_Z_COMPENSATION
		// we must restore the default scan parameters for the heat bed scan
		g_nScanXStartSteps			 = HEAT_BED_SCAN_X_START_STEPS;
		g_nScanXStepSizeMm			 = HEAT_BED_SCAN_X_STEP_SIZE_MM;
		g_nScanXStepSizeSteps		 = HEAT_BED_SCAN_X_STEP_SIZE_STEPS;
		g_nScanXEndSteps			 = HEAT_BED_SCAN_X_END_STEPS;
		g_nScanXMaxPositionSteps	 = HEAT_BED_SCAN_X_MAX_POSITION_STEPS;

		g_nScanYStartSteps			 = HEAT_BED_SCAN_Y_START_STEPS;
		g_nScanYStepSizeMm			 = HEAT_BED_SCAN_Y_STEP_SIZE_MM;
		g_nScanYStepSizeSteps		 = HEAT_BED_SCAN_Y_STEP_SIZE_STEPS;
		g_nScanYEndSteps			 = HEAT_BED_SCAN_Y_END_STEPS;
		g_nScanYMaxPositionSteps	 = HEAT_BED_SCAN_Y_MAX_POSITION_STEPS;

		g_nScanHeatBedUpFastSteps	 = HEAT_BED_SCAN_UP_FAST_STEPS;
		g_nScanHeatBedUpSlowSteps	 = HEAT_BED_SCAN_UP_SLOW_STEPS;
		g_nScanHeatBedDownFastSteps	 = HEAT_BED_SCAN_DOWN_FAST_STEPS;
		g_nScanHeatBedDownSlowSteps	 = HEAT_BED_SCAN_DOWN_SLOW_STEPS;
		g_nScanZMaxCompensationSteps = HEAT_BED_Z_COMPENSATION_MAX_STEPS;
		g_nScanFastStepDelay		 = HEAT_BED_SCAN_FAST_STEP_DELAY_MS;
		g_nScanSlowStepDelay		 = HEAT_BED_SCAN_SLOW_STEP_DELAY_MS;
		g_nScanIdleDelay			 = HEAT_BED_SCAN_IDLE_DELAY_MS;

		g_nScanContactPressureDelta	 = HEAT_BED_SCAN_CONTACT_PRESSURE_DELTA;
		g_nScanRetryPressureDelta	 = HEAT_BED_SCAN_RETRY_PRESSURE_DELTA;
		g_nScanIdlePressureDelta	 = HEAT_BED_SCAN_IDLE_PRESSURE_DELTA;
		g_nScanIdlePressureMin		 = HEAT_BED_SCAN_IDLE_PRESSURE_MIN;
		g_nScanIdlePressureMax		 = HEAT_BED_SCAN_IDLE_PRESSURE_MAX;

		g_nScanPressureReads		 = HEAT_BED_SCAN_PRESSURE_READS;
		g_nScanPressureTolerance	 = HEAT_BED_SCAN_PRESSURE_TOLERANCE;
		g_nScanPressureReadDelay	 = HEAT_BED_SCAN_PRESSURE_READ_DELAY_MS;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
	}
	else
	{
#if FEATURE_WORK_PART_Z_COMPENSATION
		// we must restore the default scan parameters for the work part scan
		g_nScanXStartSteps			 = WORK_PART_SCAN_X_START_STEPS;
		g_nScanXStepSizeMm			 = WORK_PART_SCAN_X_STEP_SIZE_MM;
		g_nScanXStepSizeSteps		 = WORK_PART_SCAN_X_STEP_SIZE_STEPS;
		g_nScanXEndSteps			 = WORK_PART_SCAN_X_END_STEPS;
		g_nScanXMaxPositionSteps	 = WORK_PART_SCAN_X_MAX_POSITION_STEPS;

		g_nScanYStartSteps			 = WORK_PART_SCAN_Y_START_STEPS;
		g_nScanYStepSizeMm			 = WORK_PART_SCAN_Y_STEP_SIZE_MM;
		g_nScanYStepSizeSteps		 = WORK_PART_SCAN_Y_STEP_SIZE_STEPS;
		g_nScanYEndSteps			 = WORK_PART_SCAN_Y_END_STEPS;
		g_nScanYMaxPositionSteps	 = WORK_PART_SCAN_Y_MAX_POSITION_STEPS;

		g_nScanHeatBedUpFastSteps	 = WORK_PART_SCAN_UP_FAST_STEPS;
		g_nScanHeatBedUpSlowSteps	 = WORK_PART_SCAN_UP_SLOW_STEPS;
		g_nScanHeatBedDownFastSteps	 = WORK_PART_SCAN_DOWN_FAST_STEPS;
		g_nScanHeatBedDownSlowSteps	 = WORK_PART_SCAN_DOWN_SLOW_STEPS;
		g_nScanZMaxCompensationSteps = WORK_PART_Z_COMPENSATION_MAX_STEPS;
		g_nScanFastStepDelay		 = WORK_PART_SCAN_FAST_STEP_DELAY_MS;
		g_nScanSlowStepDelay		 = WORK_PART_SCAN_SLOW_STEP_DELAY_MS;
		g_nScanIdleDelay			 = WORK_PART_SCAN_IDLE_DELAY_MS;

#if FEATURE_CONFIGURABLE_MILLER_TYPE
		if( Printer::MillerType == MILLER_TYPE_ONE_TRACK )
		{
			g_nScanContactPressureDelta	= MT1_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA;
			g_nScanRetryPressureDelta	= MT1_WORK_PART_SCAN_RETRY_PRESSURE_DELTA;
		}
		else
		{
			g_nScanContactPressureDelta	= MT2_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA;
			g_nScanRetryPressureDelta	= MT2_WORK_PART_SCAN_RETRY_PRESSURE_DELTA;
		}
#else
		g_nScanContactPressureDelta	= WORK_PART_SCAN_CONTACT_PRESSURE_DELTA;
		g_nScanRetryPressureDelta	= WORK_PART_SCAN_RETRY_PRESSURE_DELTA;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

		g_nScanIdlePressureDelta	= WORK_PART_SCAN_IDLE_PRESSURE_DELTA;
		g_nScanIdlePressureMin		= WORK_PART_SCAN_IDLE_PRESSURE_MIN;
		g_nScanIdlePressureMax		= WORK_PART_SCAN_IDLE_PRESSURE_MAX;

		g_nScanPressureReads		= WORK_PART_SCAN_PRESSURE_READS;
		g_nScanPressureTolerance	= WORK_PART_SCAN_PRESSURE_TOLERANCE;
		g_nScanPressureReadDelay	= WORK_PART_SCAN_PRESSURE_READ_DELAY_MS;
#endif // FEATURE_WORK_PART_Z_COMPENSATION
	}
#else
#if FEATURE_HEAT_BED_Z_COMPENSATION
	// we must restore the default scan parameters for the heat bed scan
	g_nScanXStartSteps			 = HEAT_BED_SCAN_X_START_STEPS;
	g_nScanXStepSizeMm			 = HEAT_BED_SCAN_X_STEP_SIZE_MM;
	g_nScanXStepSizeSteps		 = HEAT_BED_SCAN_X_STEP_SIZE_STEPS;
	g_nScanXEndSteps			 = HEAT_BED_SCAN_X_END_STEPS;
	g_nScanXMaxPositionSteps	 = HEAT_BED_SCAN_X_MAX_POSITION_STEPS;

	g_nScanYStartSteps			 = HEAT_BED_SCAN_Y_START_STEPS;
	g_nScanYStepSizeMm			 = HEAT_BED_SCAN_Y_STEP_SIZE_MM;
	g_nScanYStepSizeSteps		 = HEAT_BED_SCAN_Y_STEP_SIZE_STEPS;
	g_nScanYEndSteps			 = HEAT_BED_SCAN_Y_END_STEPS;
	g_nScanYMaxPositionSteps	 = HEAT_BED_SCAN_Y_MAX_POSITION_STEPS;

	g_nScanHeatBedUpFastSteps	 = HEAT_BED_SCAN_UP_FAST_STEPS;
	g_nScanHeatBedUpSlowSteps	 = HEAT_BED_SCAN_UP_SLOW_STEPS;
	g_nScanHeatBedDownFastSteps	 = HEAT_BED_SCAN_DOWN_FAST_STEPS;
	g_nScanHeatBedDownSlowSteps	 = HEAT_BED_SCAN_DOWN_SLOW_STEPS;
	g_nScanZMaxCompensationSteps = HEAT_BED_Z_COMPENSATION_MAX_STEPS;
	g_nScanFastStepDelay		 = HEAT_BED_SCAN_FAST_STEP_DELAY_MS;
	g_nScanSlowStepDelay		 = HEAT_BED_SCAN_SLOW_STEP_DELAY_MS;
	g_nScanIdleDelay			 = HEAT_BED_SCAN_IDLE_DELAY_MS;

	g_nScanContactPressureDelta	 = HEAT_BED_SCAN_CONTACT_PRESSURE_DELTA;
	g_nScanRetryPressureDelta	 = HEAT_BED_SCAN_RETRY_PRESSURE_DELTA;
	g_nScanIdlePressureDelta	 = HEAT_BED_SCAN_IDLE_PRESSURE_DELTA;
	g_nScanIdlePressureMin		 = HEAT_BED_SCAN_IDLE_PRESSURE_MIN;
	g_nScanIdlePressureMax		 = HEAT_BED_SCAN_IDLE_PRESSURE_MAX;

	g_nScanPressureReads		 = HEAT_BED_SCAN_PRESSURE_READS;
	g_nScanPressureTolerance	 = HEAT_BED_SCAN_PRESSURE_TOLERANCE;
	g_nScanPressureReadDelay	 = HEAT_BED_SCAN_PRESSURE_READ_DELAY_MS;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
#endif // FEATURE_MILLING_MODE

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "restoreDefaultScanParameters(): the default scan parameters have been restored" ) );
	}
	return;

} // restoreDefaultScanParameters


void outputScanParameters( void )
{
	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "outputScanParameters(): current scan parameters:" ) );

		Com::printF( PSTR( "" ), ZAXIS_STEPS_PER_MM );					Com::printFLN( PSTR( ";[steps];ZAXIS_STEPS_PER_MM" ) );

		Com::printF( PSTR( "" ), g_nScanXStartSteps );					Com::printFLN( PSTR( ";[steps];g_nScanXStartSteps" ) );
		Com::printF( PSTR( "" ), g_nScanXStepSizeSteps );				Com::printFLN( PSTR( ";[steps];g_nScanXStepSizeSteps" ) );
		Com::printF( PSTR( "" ), g_nScanXEndSteps );					Com::printFLN( PSTR( ";[steps];g_nScanXEndSteps" ) );
		Com::printF( PSTR( "" ), g_nScanXMaxPositionSteps );			Com::printFLN( PSTR( ";[steps];g_nScanXMaxPositionSteps" ) );

		Com::printF( PSTR( "" ), g_nScanYStartSteps );					Com::printFLN( PSTR( ";[steps];g_nScanYStartSteps" ) );
		Com::printF( PSTR( "" ), g_nScanYStepSizeSteps );				Com::printFLN( PSTR( ";[steps];g_nScanYStepSizeSteps" ) );
		Com::printF( PSTR( "" ), g_nScanYEndSteps );					Com::printFLN( PSTR( ";[steps];g_nScanYEndSteps" ) );
		Com::printF( PSTR( "" ), g_nScanYMaxPositionSteps );			Com::printFLN( PSTR( ";[steps];g_nScanYMaxPositionSteps" ) );

		Com::printF( PSTR( "" ), (int)g_nScanHeatBedUpFastSteps );		Com::printFLN( PSTR( ";[steps];g_nScanHeatBedUpFastSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanHeatBedUpSlowSteps );		Com::printFLN( PSTR( ";[steps];g_nScanHeatBedUpSlowSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanHeatBedDownFastSteps );	Com::printFLN( PSTR( ";[steps];g_nScanHeatBedDownFastSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanHeatBedDownSlowSteps );	Com::printFLN( PSTR( ";[steps];g_nScanHeatBedDownSlowSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanFastStepDelay );			Com::printFLN( PSTR( ";[ms];g_nScanFastStepDelay" ) );
		Com::printF( PSTR( "" ), (int)g_nScanSlowStepDelay );			Com::printFLN( PSTR( ";[ms];g_nScanSlowStepDelay" ) );
		Com::printF( PSTR( "" ), (int)g_nScanIdleDelay );				Com::printFLN( PSTR( ";[ms];g_nScanIdleDelay" ) );

		Com::printF( PSTR( "" ), (int)g_nScanContactPressureDelta );	Com::printFLN( PSTR( ";[digits];g_nScanContactPressureDelta" ) );
		Com::printF( PSTR( "" ), (int)g_nScanRetryPressureDelta );		Com::printFLN( PSTR( ";[digits];g_nScanRetryPressureDelta" ) );
		Com::printF( PSTR( "" ), (int)g_nScanIdlePressureDelta );		Com::printFLN( PSTR( ";[digits];g_nScanIdlePressureDelta" ) );

		Com::printF( PSTR( "" ), (int)g_nScanPressureReads );			Com::printFLN( PSTR( ";[-];g_nScanPressureReads" ) );
		Com::printF( PSTR( "" ), (int)g_nScanPressureTolerance );		Com::printFLN( PSTR( ";[digits];g_nScanPressureTolerance" ) );
		Com::printF( PSTR( "" ), (int)g_nScanPressureReadDelay );		Com::printFLN( PSTR( ";[ms];g_nScanPressureReadDelay" ) );
	}
	return;

} // outputScanParameters


void initCompensationMatrix( void )
{
	// clear all fields of the compensation matrix
	memset( g_ZCompensationMatrix, 0, COMPENSATION_MATRIX_MAX_X*COMPENSATION_MATRIX_MAX_Y*2 );
	return;

} // initCompensationMatrix


void outputCompensationMatrix( void )
{
	if( Printer::debugInfo() )
	{
		short	x;
		short	y;


//		Com::printFLN( PSTR( "z compensation matrix:" ) );
		Com::printFLN( PSTR( "front left ... front right" ) );
		Com::printFLN( PSTR( "...        ...         ..." ) );
		Com::printFLN( PSTR( "back left  ...  back right" ) );

#if FEATURE_HEAT_BED_Z_COMPENSATION
		g_offsetZCompensationSteps = -32000;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

		for( y=0; y<g_uZMatrixMaxY; y++ )
		{
			for( x=0; x<g_uZMatrixMaxX; x++ )
			{
				Com::printF( PSTR( ";" ), g_ZCompensationMatrix[x][y] );

#if FEATURE_HEAT_BED_Z_COMPENSATION
				if( x>0 && y>0 && g_ZCompensationMatrix[x][y] > g_offsetZCompensationSteps )
				{
					g_offsetZCompensationSteps = g_ZCompensationMatrix[x][y];
				}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
			}
			Com::printFLN( PSTR( " " ) );
		}

#if FEATURE_HEAT_BED_Z_COMPENSATION
		Com::printF( PSTR( "offset = " ), g_offsetZCompensationSteps );
		Com::printFLN( PSTR( " [steps]" ), g_offsetZCompensationSteps );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

		Com::printFLN( PSTR( "g_uZMatrixMaxX = " ), g_uZMatrixMaxX );
		Com::printFLN( PSTR( "g_uZMatrixMaxY = " ), g_uZMatrixMaxY );

#if FEATURE_WORK_PART_Z_COMPENSATION && FEATURE_MILLING_MODE
		if( Printer::operatingMode == OPERATING_MODE_MILL )
		{
			Com::printFLN( PSTR( "g_nActiveWorkPart = " ), g_nActiveWorkPart );
			Com::printF( PSTR( "scan start: x = " ), (float)g_nScanXStartSteps / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)g_nScanYStartSteps / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
			Com::printF( PSTR( "scan steps: x = " ), (float)g_nScanXStepSizeMm );
			Com::printF( PSTR( ", y = " ), (float)g_nScanYStepSizeMm );
			Com::printFLN( PSTR( " [mm]" ) );
			Com::printF( PSTR( "scan end: x = " ), (float)g_nScanXMaxPositionSteps / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)g_nScanYMaxPositionSteps / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
		}
#endif // FEATURE_WORK_PART_Z_COMPENSATION && FEATURE_MILLING_MODE
	}

	return;

} // outputCompensationMatrix


char prepareCompensationMatrix( void )
{
	short	x;
	short	y;


	// perform some safety checks first
	if( g_ZCompensationMatrix[0][0] != EEPROM_FORMAT )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid version detected: " ), g_ZCompensationMatrix[0][0] );
			Com::printF( PSTR( " (expected: " ), EEPROM_FORMAT );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}
	
	if( g_uZMatrixMaxX > COMPENSATION_MATRIX_MAX_X || g_uZMatrixMaxX < 2 )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid x dimension detected: " ), g_uZMatrixMaxX );
			Com::printF( PSTR( " (max expected: " ), COMPENSATION_MATRIX_MAX_X );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	if( g_uZMatrixMaxY > COMPENSATION_MATRIX_MAX_Y || g_uZMatrixMaxY < 2 )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid y dimension detected: " ), g_uZMatrixMaxY );
			Com::printF( PSTR( " (max expected: " ), COMPENSATION_MATRIX_MAX_Y );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	if( g_ZCompensationMatrix[2][0] > 0 )
	{
		// we have to fill x[1] with the values of x[2]
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] > 0" ) );
		}
*/		g_ZCompensationMatrix[1][0] = 0;
		for( y=1; y<g_uZMatrixMaxY; y++ )
		{
			g_ZCompensationMatrix[1][y] = g_ZCompensationMatrix[2][y];
		}
	}
	else
	{
		// we have to shift all x columns one index
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] = 0" ) );
		}
*/		for( x=1; x<g_uZMatrixMaxX-1; x++ )
		{
			for( y=0; y<g_uZMatrixMaxY; y++ )
			{
				g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x+1][y];
			}
		}

		// we have one x column less now
		g_uZMatrixMaxX --;
	}

	if( g_ZCompensationMatrix[g_uZMatrixMaxX-1][0] < (short)X_MAX_LENGTH )
	{
		// we have to fill x[g_uZMatrixMaxX] with the values of x[g_uZMatrixMaxX-1]
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[g_uZMatrixMaxX-1] < X_MAX_LENGTH" ) );
		}
*/		g_ZCompensationMatrix[g_uZMatrixMaxX][0] = short(X_MAX_LENGTH);
		for( y=1; y<g_uZMatrixMaxY; y++ )
		{
			g_ZCompensationMatrix[g_uZMatrixMaxX][y] = g_ZCompensationMatrix[g_uZMatrixMaxX-1][y];
		}

		// we have one x column more now
		g_uZMatrixMaxX ++;
	}
	else
	{
		// there is nothing else to do here
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[g_uZMatrixMaxX-1] = X_MAX_LENGTH" ) );
		}
*/	}

	if( g_ZCompensationMatrix[0][2] > 0 )
	{
		// we have to fill y[1] with the values of y[2]
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[2] > 0" ) );
		}
*/		g_ZCompensationMatrix[0][1] = 0;
		for( x=1; x<g_uZMatrixMaxX; x++ )
		{
			g_ZCompensationMatrix[x][1] = g_ZCompensationMatrix[x][2];
		}
	}
	else
	{
		// we have to shift all y columns one index
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[2] = 0" ) );
		}
*/		for( x=0; x<g_uZMatrixMaxX; x++ )
		{
			for( y=1; y<g_uZMatrixMaxY-1; y++ )
			{
				g_ZCompensationMatrix[x][y] = g_ZCompensationMatrix[x][y+1];
			}
		}

		// we have one y column less now
		g_uZMatrixMaxY --;
	}

	if( g_ZCompensationMatrix[0][g_uZMatrixMaxY-1] < short(Y_MAX_LENGTH) )
	{
		// we have to fill y[g_uZMatrixMaxY] with the values of y[g_uZMatrixMaxY-1]
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[g_uZMatrixMaxY-1] < Y_MAX_LENGTH" ) );
		}
*/		g_ZCompensationMatrix[0][g_uZMatrixMaxY] = short(Y_MAX_LENGTH);
		for( x=1; x<g_uZMatrixMaxX; x++ )
		{
			g_ZCompensationMatrix[x][g_uZMatrixMaxY] = g_ZCompensationMatrix[x][g_uZMatrixMaxY-1];
		}

		// we have one y column more now
		g_uZMatrixMaxY ++;
	}
	else
	{
		// there is nothing else to do here
/*		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[g_uZMatrixMaxY-1] = Y_MAX_LENGTH" ) );
		}
*/	}

	return 0;

} // prepareCompensationMatrix


char convertCompensationMatrix( void )
{
#if FEATURE_HEAT_BED_Z_COMPENSATION
	short	x;
	short	y;


	g_offsetZCompensationSteps = -32000;

	for( x=1; x<g_uZMatrixMaxX-1; x++ )
	{
		for( y=1; y<g_uZMatrixMaxY-1; y++ )
		{
			if( x>0 && y>0 && g_ZCompensationMatrix[x][y] > g_offsetZCompensationSteps )
			{
				g_offsetZCompensationSteps = g_ZCompensationMatrix[x][y];
			}
		}
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

	return 0;

} // convertCompensationMatrix


char saveCompensationMatrix( unsigned int uAddress )
{
	unsigned int	uOffset;
	short			uTemp;
	short			uMax = -32000;
	short			x;
	short			y;


	if( g_ZCompensationMatrix[0][0] && g_uZMatrixMaxX && g_uZMatrixMaxY )
	{
		// we have valid compensation values
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "saveCompensationMatrix(): valid data detected" ) );
		}

		// write the current header version
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT );
		
		// write the current sector version
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, EEPROM_FORMAT );
		
		// write the current x dimension
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, g_uZMatrixMaxX );

		// write the current y dimension
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, g_uZMatrixMaxY );

		// write the current micro steps
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS, RF_MICRO_STEPS );

		// write some information about the scanning area - note that this information is read only in case of work part z-compensation matrixes later
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_START_MM, (short)(g_nScanXStartSteps / XAXIS_STEPS_PER_MM) );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_START_MM, (short)(g_nScanYStartSteps / YAXIS_STEPS_PER_MM) );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_STEP_MM, (short)g_nScanXStepSizeMm );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_STEP_MM, (short)g_nScanYStepSizeMm );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_END_MM, (short)(g_nScanXMaxPositionSteps / XAXIS_STEPS_PER_MM) );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_END_MM, (short)(g_nScanYMaxPositionSteps / YAXIS_STEPS_PER_MM) );

		uOffset = uAddress + EEPROM_OFFSET_MAXTRIX_START;
		for( x=0; x<g_uZMatrixMaxX; x++ )
		{
			for( y=0; y<g_uZMatrixMaxY; y++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				uTemp = g_ZCompensationMatrix[x][y];
				writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, uTemp );
				uOffset += 2;

				if( x>0 && y>0 )
				{
					// the first column and row is used for version and position information
					if( uTemp > uMax )	uMax = uTemp;
				}
			}
		}
	}
	else
	{
		// we do not have valid heat bed compensation values - clear the EEPROM data
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "saveCompensationMatrix(): invalid data detected ( " ), g_ZCompensationMatrix[0][0] );
			Com::printF( PSTR( " / " ), g_uZMatrixMaxX );
			Com::printF( PSTR( " / " ), g_uZMatrixMaxY );
			Com::printFLN( PSTR( " )" ), g_uZMatrixMaxY );
		}

		// write the current version
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT );
		
		// write the current sector version
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT, 0 );
		
		// write the current x dimension
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X, 0 );

		// write the current y dimension
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y, 0 );

		// write the current micro steps
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS, 0 );

		// write some information about the scanning area - note that this information is read only in case of work part z-compensation matrixes later
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_START_MM, 0 );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_START_MM, 0 );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_STEP_MM, 0 );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_STEP_MM, 0 );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_END_MM, 0 );
		writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_END_MM, 0 );

		uOffset = uAddress + EEPROM_OFFSET_MAXTRIX_START;
		for( x=0; x<COMPENSATION_MATRIX_MAX_X; x++ )
		{
			for( y=0; y<COMPENSATION_MATRIX_MAX_Y; y++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uOffset, 0 );
				uOffset += 2;
			}
		}
	}

#if FEATURE_HEAT_BED_Z_COMPENSATION
	g_offsetZCompensationSteps = uMax;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

	return 0;

} // saveCompensationMatrix


char loadCompensationMatrix( unsigned int uAddress )
{
	unsigned short	uTemp;
	unsigned short	uDimensionX;
	unsigned short	uDimensionY;
	unsigned short	uMicroSteps;
	unsigned int	uOffset;
	short			nTemp;
	short			uMax = -32000;
	short			x;
	short			y;
	float			fMicroStepCorrection;


	// check the stored header format
	uTemp = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_HEADER_FORMAT );

	if( uTemp != EEPROM_FORMAT )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "loadCompensationMatrix(): invalid header format detected: " ), (int)uTemp );
			Com::printF( PSTR( " (expected: " ), EEPROM_FORMAT );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	if( !uAddress )
	{
		// we have to detect the to-be-loaded compensation matrix automatically
#if FEATURE_MILLING_MODE
		if( Printer::operatingMode == OPERATING_MODE_PRINT )
		{
#if FEATURE_HEAT_BED_Z_COMPENSATION
			// load the heat bed compensation matrix
			uAddress = EEPROM_SECTOR_SIZE;
#else
			// we do not support the heat bed compensation
			return -1;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
		}
		else
		{
#if FEATURE_WORK_PART_Z_COMPENSATION
			// load the currently active work part compensation matrix
			uTemp = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_WORK_PART );

			if( uTemp < 1 || uTemp > EEPROM_MAX_WORK_PART_SECTORS )
			{
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "loadCompensationMatrix(): invalid active work part detected: " ), (int)uTemp );
				}
				return -1;
			}

			g_nActiveWorkPart = (char)uTemp;
			uAddress		  = EEPROM_SECTOR_SIZE + (unsigned int)(EEPROM_SECTOR_SIZE * uTemp);
#else
			// we do not support the work part compensation
			return -1;
#endif // FEATURE_WORK_PART_Z_COMPENSATION
		}
#else
#if FEATURE_HEAT_BED_Z_COMPENSATION
		// load the heat bed compensation matrix
		uAddress = EEPROM_SECTOR_SIZE;
#else
		// we do not support the heat bed compensation
		return -1;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
#endif // FEATURE_MILLING_MODE
	}

#if FEATURE_WORK_PART_Z_COMPENSATION && FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_MILL )
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "loadCompensationMatrix(): active work part: " ), (int)g_nActiveWorkPart );
		}
	}
#endif // FEATURE_WORK_PART_Z_COMPENSATION && FEATURE_MILLING_MODE

	// check the stored sector format
	uTemp = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_SECTOR_FORMAT );

	if( uTemp != EEPROM_FORMAT )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "loadCompensationMatrix(): invalid sector format detected: " ), (int)uTemp );
			Com::printF( PSTR( " (expected: " ), EEPROM_FORMAT );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	// check the stored x dimension
	uDimensionX = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_X );

	if( uDimensionX > COMPENSATION_MATRIX_MAX_X )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "loadCompensationMatrix(): invalid x dimension detected: " ), (int)uDimensionX );
			Com::printF( PSTR( " (max expected: " ), COMPENSATION_MATRIX_MAX_X );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	// check the stored y dimension
	uDimensionY = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_DIMENSION_Y );

	if( uDimensionY > COMPENSATION_MATRIX_MAX_Y )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "loadCompensationMatrix(): invalid y dimension detected: " ), (int)uDimensionY );
			Com::printF( PSTR( " (max expected: " ), COMPENSATION_MATRIX_MAX_Y );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	g_uZMatrixMaxX = (unsigned char)uDimensionX;
	g_uZMatrixMaxY = (unsigned char)uDimensionY;

	// check the stored microsteps
	uMicroSteps = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_MICRO_STEPS );

	if( uMicroSteps == RF_MICRO_STEPS )
	{
		// the current z-compensation matrix has been determined with the current micro step setting, there is nothing to recalculate
		fMicroStepCorrection = 1.0;
	}
	else if( uMicroSteps > RF_MICRO_STEPS )
	{
		// the current z-compensation matrix has been determined with a higher than the current micro step setting, we must divide all z-correction values
		fMicroStepCorrection = (float)RF_MICRO_STEPS / (float)uMicroSteps;
	}
	else
	{
		// the current z-compensation matrix has been determined with a smaller than the current micro step setting, we must multiply all z-correction values
		fMicroStepCorrection = (float)RF_MICRO_STEPS / (float)uMicroSteps;
	}

	if( uAddress != EEPROM_SECTOR_SIZE )
	{
		// in case we are reading a work part z-compensation matrix, we have to read out some information about the scanning area
		g_nScanXStartSteps		 = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_START_MM ) * XAXIS_STEPS_PER_MM;
		g_nScanYStartSteps		 = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_START_MM ) * YAXIS_STEPS_PER_MM;
		g_nScanXStepSizeMm		 = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_STEP_MM );
		g_nScanYStepSizeMm		 = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_STEP_MM );
		g_nScanXMaxPositionSteps = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_X_END_MM ) * XAXIS_STEPS_PER_MM;
		g_nScanYMaxPositionSteps = (long)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uAddress + EEPROM_OFFSET_Y_END_MM ) * YAXIS_STEPS_PER_MM;
		g_nScanXStepSizeSteps	 = g_nScanXStepSizeMm * XAXIS_STEPS_PER_MM;
		g_nScanYStepSizeSteps	 = g_nScanYStepSizeMm * YAXIS_STEPS_PER_MM;
	}

	// read out the actual compensation values
	uOffset = uAddress + EEPROM_OFFSET_MAXTRIX_START;
	for( x=0; x<g_uZMatrixMaxX; x++ )
	{
		for( y=0; y<g_uZMatrixMaxY; y++ )
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			nTemp = readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, uOffset );

			if( x == 0 || y == 0 )
			{
				// we must not modify our header row/column
				g_ZCompensationMatrix[x][y] = nTemp;
			}
			else
			{
				// we may have to update all z-compensation values
				g_ZCompensationMatrix[x][y] = (short)((float)nTemp * fMicroStepCorrection);
			}
			uOffset += 2;

			if( x>0 && y>0 )
			{
				// the first column and row is used for version and position information
				if( nTemp > uMax )	uMax = nTemp;
			}
		}
	}

#if FEATURE_HEAT_BED_Z_COMPENSATION
	g_offsetZCompensationSteps = uMax;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

	resetZCompensation();
	return 0;

} // loadCompensationMatrix


void clearCompensationMatrix( unsigned int uAddress )
{
	// clear all fields of the compensation matrix
	initCompensationMatrix();

	// store the cleared compensation matrix to the EEPROM
	saveCompensationMatrix( uAddress );

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "clearCompensationMatrix(): the compensation matrix has been cleared" ) );
	}
	return;

} // clearCompensationMatrix


void outputPressureMatrix( void )
{
	if( Printer::debugInfo() )
	{
#if DEBUG_REMEMBER_SCAN_PRESSURE
		short	i;
		short	j;


		Com::printFLN( PSTR( "Pressure matrix:" ) );
		Com::printFLN( PSTR( "front left ... front right" ) );
		Com::printFLN( PSTR( "...        ...         ..." ) );
		Com::printFLN( PSTR( "back left  ...  back right" ) );
		for( i=0; i<COMPENSATION_MATRIX_MAX_Y; i++ )
		{
			for( j=0; j<COMPENSATION_MATRIX_MAX_X; j++ )
			{
				Com::printF( PSTR( ";" ), g_ScanPressure[j][i] );
			}
			Com::printFLN( PSTR( " " ) );
		}
#endif // DEBUG_REMEMBER_SCAN_PRESSURE
	}

	return;

} // outputPressureMatrix
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION


char clearExternalEEPROM( void )
{
	unsigned short	i;
	unsigned short	uMax = 32768;
	unsigned short	uTemp;
	unsigned short	uLast = 0;


	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "clearExternalEEPROM(): erasing external memory ..." ) );
	}

	// the external EEPROM is able to store 262.144 kBit (= 32.768 kByte)
	for( i=0; i<uMax; i++ )
	{
		writeByte24C256( I2C_ADDRESS_EXTERNAL_EEPROM, i, 0 );
		Commands::checkForPeriodicalActions();

		if( Printer::debugInfo() )
		{
			uTemp = i / 100;
			if( uTemp != uLast )
			{
				Com::printF( PSTR( "clearExternalEEPROM(): " ), (int)i );
				Com::printFLN( PSTR( " / " ), (long)uMax );
				uLast = uTemp;
			}
		}
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "clearExternalEEPROM(): erasing complete" ) );
	}
	return 0;

} // clearExternalEEPROM


void writeByte24C256( int addressI2C, unsigned int addressEEPROM, unsigned char data )
{
	HAL::delayMilliseconds( EEPROM_DELAY );
    Wire.beginTransmission( addressI2C );
    Wire.write( int(addressEEPROM >> 8));		// MSB
    Wire.write( int(addressEEPROM & 0xFF));		// LSB
    Wire.write( data );
    Wire.endTransmission();
	return;
    
} // writeByte24C256


void writeWord24C256( int addressI2C, unsigned int addressEEPROM, unsigned short data )
{
	unsigned short	Temp;


	Temp = byte(data >> 8);
	writeByte24C256( addressI2C, addressEEPROM, Temp );
	Temp = byte(data & 0x00FF);
	writeByte24C256( addressI2C, addressEEPROM+1, Temp );
	return;

} // writeWord24C256


unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM )
{
	HAL::delayMilliseconds( EEPROM_DELAY );
    Wire.beginTransmission( addressI2C );
    Wire.write( int(addressEEPROM >> 8));		// MSB
    Wire.write( int(addressEEPROM & 0xFF));		// LSB
    Wire.endTransmission();
    Wire.requestFrom( addressI2C, 1 );
    
    return Wire.read();
    
} // readByte24C256


unsigned short readWord24C256( int addressI2C, unsigned int addressEEPROM )
{
	unsigned short	data;
	byte			Temp;


	Temp = readByte24C256( addressI2C, addressEEPROM );
	data = Temp;
	data = data << 8;
	Temp = readByte24C256( addressI2C, addressEEPROM+1 );

	return data + Temp;

} // readWord24C256


void doZCompensation( void )
{
#if FEATURE_MILLING_MODE

	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
#if FEATURE_HEAT_BED_Z_COMPENSATION
		doHeatBedZCompensation();
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
	}
	else
	{
#if FEATURE_WORK_PART_Z_COMPENSATION
		doWorkPartZCompensation();
#endif // FEATURE_WORK_PART_Z_COMPENSATION
	}

#else

#if FEATURE_HEAT_BED_Z_COMPENSATION
	doHeatBedZCompensation();
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#endif // FEATURE_MILLING_MODE

} // doZCompensation


void loopRF( void )
{
	static char		nEntered = 0;
	unsigned long	uTime;
	short			nPressure;


#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	if( nEntered )
	{
		// do not enter more than once
		return;
	}
	nEntered ++;

	uTime = HAL::timeInMilliseconds();

#if FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON
	if( Printer::prepareFanOff )
	{
		if( (uTime - Printer::prepareFanOff) > Printer::fanOffDelay )
		{
			// it is time to turn the case fan off
			Printer::prepareFanOff = 0;
			WRITE( CASE_FAN_PIN, 0 );
		}
	}
#endif // FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON

#if FEATURE_MILLING_MODE

	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
#if FEATURE_HEAT_BED_Z_COMPENSATION
		if( g_nHeatBedScanStatus )
		{
			scanHeatBed();
		}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
	}
	else
	{
#if FEATURE_WORK_PART_Z_COMPENSATION
		if( g_nWorkPartScanStatus )
		{
			scanWorkPart();
		}
#endif // FEATURE_WORK_PART_Z_COMPENSATION
	}

#else

#if FEATURE_HEAT_BED_Z_COMPENSATION
	if( g_nHeatBedScanStatus )
	{
		scanHeatBed();
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#endif // FEATURE_MILLING_MODE

#if FEATURE_FIND_Z_ORIGIN
	if( g_nFindZOriginStatus )
	{
		findZOrigin();
	}
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_PAUSE_PRINTING
	if( g_uPauseTime )
	{
		if( !g_pauseBeepDone )
		{
			BEEP_PAUSE
			g_pauseBeepDone = 1;
		}

		if( g_pauseStatus == PAUSE_STATUS_PAUSED )
		{
#if EXTRUDER_CURRENT_PAUSE_DELAY
			if( (uTime - g_uPauseTime) > EXTRUDER_CURRENT_PAUSE_DELAY )
			{
				char	nProcessExtruder = 0;


#if FEATURE_MILLING_MODE
				if( Printer::operatingMode == OPERATING_MODE_PRINT )
				{
					// process the extruder only in case we are in mode "print"
					nProcessExtruder = 1;
				}
#else
				nProcessExtruder = 1;
#endif // FEATURE_MILLING_MODE

				if( nProcessExtruder )
				{
					// we have paused a few moments ago - reduce the current of the extruder motor in order to avoid unwanted heating of the filament for use cases where the printing is paused for several minutes
/*					Com::printF( PSTR( "loopRF(): PauseTime = " ), g_uPauseTime );
					Com::printF( PSTR( ", Time = " ), uTime );
					Com::printFLN( PSTR( ", Diff = " ), uTime - g_uPauseTime );
*/
					setExtruderCurrent( EXTRUDER_CURRENT_PAUSED );
				}
				g_uPauseTime = 0;
			}
#endif // EXTRUDER_CURRENT_PAUSE_DELAY
		}
		else
		{
			// we are not paused any more
			g_uPauseTime = 0;
		}
	}
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_EMERGENCY_PAUSE
	if( g_nEmergencyPauseDigitsMin || g_nEmergencyPauseDigitsMax )
	{
		if( (uTime - g_uLastPressureTime) > EMERGENCY_PAUSE_INTERVAL )
		{
			g_uLastPressureTime = uTime;

			if( (g_pauseStatus == PAUSE_STATUS_NONE || g_pauseStatus == PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE) && PrintLine::linesCount > 5 )
			{
				// this check shall be done only during the printing (for example, it shall not be done in case filament is extruded manually)
				g_nPressureSum	  += readStrainGauge( ACTIVE_STRAIN_GAUGE );
				g_nPressureChecks += 1;

				if( g_nPressureChecks == EMERGENCY_PAUSE_CHECKS )
				{
					nPressure		 = g_nPressureSum / g_nPressureChecks;

	//				Com::printF( PSTR( "loopRF(): average = " ), nPressure );
	//				Com::printFLN( PSTR( " / " ), g_nPressureChecks );

					g_nPressureSum	  = 0;
					g_nPressureChecks = 0;

					if( (nPressure < g_nEmergencyPauseDigitsMin) ||
						(nPressure > g_nEmergencyPauseDigitsMax) )
					{
						// the pressure is outside the allowed range, we must perform the emergency pause
						if( Printer::debugErrors() )
						{
							Com::printF( PSTR( "loopRF(): emergency pause: " ), nPressure );
							Com::printFLN( PSTR( " / " ), PrintLine::linesCount );
						}

						pausePrint();
					}
				}
			}
			else
			{
				g_nPressureSum	  = 0;
				g_nPressureChecks = 0;
			}
		}
	}
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_EMERGENCY_Z_STOP
	if( (uTime - g_uLastZPressureTime) > EMERGENCY_Z_STOP_INTERVAL )
	{
		g_uLastZPressureTime = uTime;

		if( Printer::stepperDirection[Z_AXIS] && !Extruder::current->stepperDirection )
		{
			// this check shall be done only when there is some moving into z-direction in progress and the extruder is not doing anything
			g_nZPressureSum	   += readStrainGauge( ACTIVE_STRAIN_GAUGE );
			g_nZPressureChecks += 1;

			if( g_nZPressureChecks == EMERGENCY_Z_STOP_CHECKS )
			{
				nPressure		 = g_nZPressureSum / g_nZPressureChecks;

				g_nZPressureSum	   = 0;
				g_nZPressureChecks = 0;

				if( (nPressure < EMERGENCY_Z_STOP_DIGITS_MIN) ||
					(nPressure > EMERGENCY_Z_STOP_DIGITS_MAX) )
				{
					// the pressure is outside the allowed range, we must perform the emergency z-stop
					HAL::forbidInterrupts();
					moveZ( ZAXIS_STEPS_PER_MM *5 );

					// block any further movement into the z-direction
					Printer::blockZ					  = 1;
					Printer::stepperDirection[Z_AXIS] = 0;
					HAL::allowInterrupts();

					Com::printFLN( PSTR( "loopRF(): block Z" ) );
				}
			}
		}
		else
		{
			g_nZPressureSum	   = 0;
			g_nZPressureChecks = 0;
		}
	}
#endif // FEATURE_EMERGENCY_Z_STOP

	if( g_uStopTime )
	{
		if( (uTime - g_uStopTime) > CLEAN_UP_DELAY_AFTER_STOP_PRINT )
		{
			// we have stopped the printing a few moments ago, output the object now
			if( PrintLine::linesCount )
			{
				// wait until all moves are done
				g_uStopTime = uTime;
			}
			else
			{
				// there is no printing in progress any more, do all clean-up now
				g_uStopTime	= 0;

				// disable all heaters
				Extruder::setHeatedBedTemperature( 0, false );
				Extruder::setTemperatureForExtruder( 0, 0, false );

#if NUM_EXTRUDER == 2
				Extruder::setTemperatureForExtruder( 0, 1, false );
#endif // #if NUM_EXTRUDER == 2

#if FEATURE_MILLING_MODE
				if ( Printer::operatingMode == OPERATING_MODE_MILL )
				{
					EEPROM::updatePrinterUsage();
				}
#endif // FEATURE_MILLING_MODE

#if FEATURE_OUTPUT_FINISHED_OBJECT
				// output the object
				outputObject();
#else
				// disable all steppers
				Printer::setAllSteppersDisabled();
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();
				Extruder::disableAllExtruders();
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FAN_PIN>-1
				// disable the fan
				Commands::setFanSpeed(0,false);
#endif // FAN_PIN>-1

				uTime = millis();
				g_uBlockCommands = uTime;
			}
		}
	}

	if( g_uBlockCommands )
	{
		if( (uTime - g_uBlockCommands) > COMMAND_BLOCK_DELAY )
		{
			g_uBlockCommands = 0;
		}
	}
	
#if FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR
	if( Printer::isAnyTempsensorDefect() && sd.sdmode && PrintLine::linesCount )
	{
		// we are printing from the SD card and a temperature sensor got defect - abort the current printing
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "loopRF(): aborting print because of a temperature sensor defect" ) );
		}

		sd.abortPrint();
	}
#endif // FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR

	if( (uTime - g_lastTime) > LOOP_INTERVAL )
	{
		// it is time for another processing
		g_lastTime = uTime;

#if FEATURE_HEAT_BED_Z_COMPENSATION
//		if( g_debugLevel && Printer::debugInfo() )
		if( g_debugLevel && Printer::debugInfo() && PrintLine::linesCount )
		{
#if DEBUG_HEAT_BED_Z_COMPENSATION
			switch( g_debugLevel )
			{
				case 1:
				{
					Com::printF( PSTR( "tcZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( ";ccZ;" ), Printer::compensatedPositionCurrentStepsZ );
					break;
				}
				case 2:
				{
					Com::printF( PSTR( "tpsZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( ";cpsZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					break;
				}
				case 4:
				{
					Com::printF( PSTR( "tpsX;" ), Printer::directPositionTargetSteps[X_AXIS] );
					Com::printF( PSTR( ";cpsX;" ), Printer::directPositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";tpsY;" ), Printer::directPositionTargetSteps[Y_AXIS] );
					Com::printF( PSTR( ";cpsY;" ), Printer::directPositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";tpsZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( ";cpsZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";tpsE;" ), Printer::directPositionTargetSteps[E_AXIS] );
					Com::printF( PSTR( ";cpsE;" ), Printer::directPositionCurrentSteps[E_AXIS] );
					break;
				}
				case 5:
				{
					Com::printF( PSTR( "nCPS X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";nCPS Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";nCPS Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";t Z;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( ";c Z;" ), Printer::compensatedPositionCurrentStepsZ );
					Com::printF( PSTR( ";tPS Z;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( ";cPS Z;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";DirZ;" ), Printer::stepperDirection[Z_AXIS] );
					break;
				}
				case 6:
				{
					Com::printF( PSTR( "nCPS X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( "; nCPS Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( "; nCPS Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( "; MDZ;" ), Printer::stepperDirection[Z_AXIS] );
					Com::printF( PSTR( "; tCZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( "; cCZ;" ), Printer::compensatedPositionCurrentStepsZ );
					Com::printF( PSTR( "; tPSZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cPSZ;" ), Printer::queuePositionLastSteps[Z_AXIS] );
					Com::printF( PSTR( "; dZ;" ), Printer::queuePositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cZ;" ), Printer::queuePositionLastSteps[Z_AXIS] );
					Com::printF( PSTR( "; Int32;" ), g_debugInt32 );
					Com::printF( PSTR( "; Int16;" ), g_debugInt16 );
					Com::printF( PSTR( "; RAM;" ), Commands::lowestRAMValue );
					Com::printF( PSTR( "; doZC;" ), Printer::doHeatBedZCompensation );
					break;
				}
				case 10:
				{
					Com::printF( PSTR( "Start;" ), g_nZCompensationUpdates );
					Com::printF( PSTR( "; nCPS X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";" ), g_nLastZCompensationPositionSteps[X_AXIS] );
					Com::printF( PSTR( "; nCPS Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";" ), g_nLastZCompensationPositionSteps[Y_AXIS] );
					Com::printF( PSTR( "; nCPS Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";" ), g_nLastZCompensationPositionSteps[Z_AXIS] );
					Com::printF( PSTR( "; tCZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( "; cCZ;" ), Printer::compensatedPositionCurrentStepsZ );
					Com::printF( PSTR( "; tPSZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cPSZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( "; dZ;" ), Printer::queuePositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cZ;" ), Printer::queuePositionLastSteps[Z_AXIS] );
					Com::printF( PSTR( "; doZC;" ), Printer::doHeatBedZCompensation );
					Com::printF( PSTR( "; Delta;" ), g_nDelta[X_AXIS] );
					Com::printF( PSTR( ";" ), g_nDelta[Y_AXIS] );
					Com::printF( PSTR( "; StepSize;" ), g_nStepSize[X_AXIS] );
					Com::printF( PSTR( ";" ), g_nStepSize[Y_AXIS] );
					Com::printF( PSTR( "; TempX;" ), g_nTempXFront );
					Com::printF( PSTR( ";" ), g_nTempXBack );
					Com::printF( PSTR( "; NeededZ;" ), g_nNeededZ );
					Com::printF( PSTR( "; Index;" ), g_uIndex[0] );
					Com::printF( PSTR( ";" ), g_uIndex[1] );
					Com::printF( PSTR( ";" ), g_uIndex[2] );
					Com::printF( PSTR( ";" ), g_uIndex[3] );
					Com::printF( PSTR( "; Matrix;" ), g_nMatrix[0] );
					Com::printF( PSTR( ";" ), g_nMatrix[1] );
					Com::printF( PSTR( ";" ), g_nMatrix[2] );
					Com::printF( PSTR( ";" ), g_nMatrix[3] );
					Com::printF( PSTR( "; Z Delta Min;" ), g_nZDeltaMin );
					Com::printF( PSTR( "; Z Delta Max;" ), g_nZDeltaMax );
					Com::printF( PSTR( "; Z Delay Max;" ), g_nZCompensationDelayMax );
					Com::printF( PSTR( "; Too fast;" ), g_nTooFast );
					Com::printF( PSTR( "; End;" ), g_nZCompensationUpdates );

					g_nZDeltaMin			 = 100000;
					g_nZDeltaMax			 = -100000;
					g_nZCompensationDelayMax = 0;
					g_nTooFast				 = 0;
					break;
				}

				default:
				{
					Com::printF( PSTR( "unsupported debug level: " ), g_debugLevel );
					break;
				}
			}
			Com::printFLN( PSTR( " " ) );

#endif // DEBUG_HEAT_BED_Z_COMPENSATION
		}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
		if( g_debugLevel && Printer::debugInfo() && PrintLine::linesCount )
		{
#if DEBUG_WORK_PART_Z_COMPENSATION
			switch( g_debugLevel )
			{
				case 1:
				{
					Com::printF( PSTR( "tcZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( ";ccZ;" ), Printer::compensatedPositionCurrentStepsZ );
					break;
				}
				case 2:
				{
					Com::printF( PSTR( "tpsZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( ";cpsZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					break;
				}
				case 4:
				{
					Com::printF( PSTR( "tpsX;" ), Printer::directPositionTargetSteps[X_AXIS] );
					Com::printF( PSTR( ";cpsX;" ), Printer::directPositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( ";tpsY;" ), Printer::directPositionTargetSteps[Y_AXIS] );
					Com::printF( PSTR( ";cpsY;" ), Printer::directPositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( ";tpsZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( ";cpsZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( ";tpsE;" ), Printer::directPositionTargetSteps[E_AXIS] );
					Com::printF( PSTR( ";cpsE;" ), Printer::directPositionCurrentSteps[E_AXIS] );
					break;
				}
				case 5:
				{
					Com::printF( PSTR( "X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( "; Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( "; Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( "; tZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( "; cZ;" ), Printer::compensatedPositionCurrentStepsZ );
					break;
				}
				case 6:
				{
					Com::printF( PSTR( "nCPS X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
					Com::printF( PSTR( "; nCPS Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
					Com::printF( PSTR( "; nCPS Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( "; tCZ;" ), Printer::compensatedPositionTargetStepsZ );
					Com::printF( PSTR( "; cCZ;" ), Printer::compensatedPositionCurrentStepsZ );
					Com::printF( PSTR( "; tPSZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cPSZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
					Com::printF( PSTR( "; dZ;" ), Printer::queuePositionTargetSteps[Z_AXIS] );
					Com::printF( PSTR( "; cZ;" ), Printer::queuePositionLastSteps[Z_AXIS] );
					Com::printF( PSTR( "; Int32;" ), g_debugInt32 );
					Com::printF( PSTR( "; RAM;" ), Commands::lowestRAMValue );
					break;
				}

				default:
				{
					Com::printF( PSTR( "unsupported debug level: " ), g_debugLevel );
					break;
				}
			}
			Com::printFLN( PSTR( " " ) );
#endif // DEBUG_WORK_PART_Z_COMPENSATION
		}
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
		if( g_debugLevel && Printer::debugInfo() && g_nFindZOriginStatus )
		{
#if DEBUG_FIND_Z_ORIGIN
			switch( g_debugLevel )
			{
				case 5:
				{
					Com::printF( PSTR( "OriginZ;" ), g_nZOriginPosition[Z_AXIS] );
					Com::printF( PSTR( "; Dir;" ), READ( Z_DIR_PIN ) );
					Com::printF( PSTR( "; TempDir;" ), g_nTempDirectionZ );
					Com::printF( PSTR( "; Status;" ), g_nFindZOriginStatus );
					//Com::printF( PSTR( "; Pressure;" ), g_debugInt16 );
					break;
				}

				default:
				{
					Com::printF( PSTR( "unsupported debug level: " ), g_debugLevel );
					break;
				}
			}
			Com::printFLN( PSTR( " " ) );
#endif // DEBUG_FIND_Z_ORIGIN
		}
#endif // FEATURE_FIND_Z_ORIGIN
	}

	if( g_debugLog )
	{
		switch( g_debugLog )
		{
#if FEATURE_FIND_Z_ORIGIN

			case 1:
			{
				Com::printF( PSTR( "Z-Origin X: " ), g_nZOriginPosition[X_AXIS] );
				Com::printF( PSTR( "; Z-Origin Y: " ), g_nZOriginPosition[Y_AXIS] );
				Com::printFLN( PSTR( "; Z-Origin Z: " ), Printer::staticCompensationZ );
				break;
			}

#endif // FEATURE_FIND_Z_ORIGIN
		}

		g_debugLog = 0;
	}

#if FEATURE_SERVICE_INTERVAL
	if ( !g_nEnteredService )
	{
		if ( ( HAL::timeInMilliseconds() - g_nlastServiceTime ) > 5000 )
		{
			char	mode = OPERATING_MODE_PRINT;


#if FEATURE_MILLING_MODE
			mode = Printer::operatingMode;
#endif // FEATURE_MILLING_MODE

			if( mode == OPERATING_MODE_PRINT )   
			{
				UI_STATUS(UI_TEXT_PRINTER_READY);

				if( READ(5) == 0 && READ(11) == 0 && READ(42) == 0 )
				{
					if ( g_nServiceRequest == 1 )
					{
						HAL::eprSetInt32(EPR_PRINTING_TIME_SERVICE,0);
						EEPROM::updateChecksum();
						HAL::eprSetFloat(EPR_PRINTING_DISTANCE_SERVICE,0);
						EEPROM::updateChecksum();
						Com::printF( PSTR( "Service Reset" ) );
					}
				}
				else
				{
					g_nServiceRequest = 0;
				}
			}
			else
			{
				UI_STATUS(UI_TEXT_MILLER_READY);

				if( READ(5) == 0 && READ(11) == 0 && READ(42) == 0 )
				{
					if ( g_nServiceRequest == 1 )
					{
						HAL::eprSetInt32(EPR_MILLING_TIME_SERVICE,0);
						EEPROM::updateChecksum();
						Com::printF( PSTR( " Service Reset = OK " ) );
					}
				}
				else
				{
					g_nServiceRequest = 0;
				}
			}
			g_nEnteredService  = 1;
		}
	}
#endif // FEATURE_SERVICE_INTERVAL

#if FEATURE_RGB_LIGHT_EFFECTS
	updateRGBLightStatus();
#endif // FEATURE_RGB_LIGHT_EFFECTS

	nEntered --;
	return;

} // loopRF


#if FEATURE_OUTPUT_FINISHED_OBJECT
void outputObject( void )
{
	if( PrintLine::linesCount )
	{
		// there is some printing in progress at the moment - do not park the printer in this case
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "outputObject(): the object can not be output while the printing is in progress" ) );
		}
		return;
	}

	if( !Printer::isHomed() )
	{
		// the printer does not know its home position, thus we can not output the object
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "outputObject(): the object can not be output because the home position is unknown" ) );
		}
		return;
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "outputObject()" ) );
	}

#if FAN_PIN>-1
	// disable the fan
    Commands::setFanSpeed(0,false);
#endif // FAN_PIN>-1

	Commands::printCurrentPosition();

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_MILL )
	{
		GCode::executeFString(Com::tOutputObjectMill);
	}
	else
	{
		GCode::executeFString(Com::tOutputObjectPrint);
	}
#else
	GCode::executeFString(Com::tOutputObjectPrint);
#endif // FEATURE_MILLING_MODE


	Commands::waitUntilEndOfAllMoves();
    Commands::printCurrentPosition();
	
	// disable all steppers
	Printer::setAllSteppersDisabled();
	Printer::disableXStepper();
	Printer::disableYStepper();
	Printer::disableZStepper();
	Extruder::disableAllExtruders();

} // outputObject
#endif // FEATURE_OUTPUT_FINISHED_OBJECT


#if FEATURE_PARK
void parkPrinter( void )
{
	if( PrintLine::linesCount )
	{
		// there is some printing in progress at the moment - do not park the printer in this case
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "parkPrinter(): the printer can not be parked while the printing is in progress" ) );
		}
		return;
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "parkPrinter()" ) );
	}

	Printer::homeAxis( true, true, true );

	Printer::moveToReal( g_nParkPosition[X_AXIS], g_nParkPosition[Y_AXIS], g_nParkPosition[Z_AXIS], IGNORE_COORDINATE, Printer::homingFeedrate[0]);

} // parkPrinter
#endif // FEATURE_PARK


#if FEATURE_PAUSE_PRINTING
void pausePrint( void )
{
	long	Temp;


	if( Printer::debugErrors() )
	{
		Com::printFLN( PSTR( "pausePrint()" ) );
	}

	if( g_pauseMode == PAUSE_MODE_NONE )
	{
		// the printing is not paused at the moment
		if( !Printer::isHomed() )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "pausePrint(): pause is not available at the moment because the home position is unknown" ) );
			}
			return;
		}

		if( PrintLine::linesCount )
		{
			g_pauseStatus = PAUSE_STATUS_WAIT_FOR_QUEUE_MOVE;
			g_pauseMode	  = PAUSE_MODE_PAUSED;

			// wait until the current move is completed
			while( g_pauseStatus != PAUSE_STATUS_PAUSED )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				HAL::delayMilliseconds( 1 );
				Commands::checkForPeriodicalActions();
			}

			g_uPauseTime	= HAL::timeInMilliseconds();
			g_pauseBeepDone	= 0;

			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "pausePrint(): the printing has been paused" ) );
			}

		    UI_STATUS(UI_TEXT_PAUSED);
		    Printer::setMenuMode(MENU_MODE_SD_PAUSED,true);

			g_nContinueSteps[X_AXIS] = 0;
			g_nContinueSteps[Y_AXIS] = 0;
			g_nContinueSteps[Z_AXIS] = 0;


#if FEATURE_MILLING_MODE
			if( Printer::operatingMode == OPERATING_MODE_PRINT )
			{
				// we do not process the extruder in case we are not in operating mode "print"
#endif // FEATURE_MILLING_MODE
				if( g_nPauseSteps[E_AXIS] )
				{
					Printer::directPositionTargetSteps[E_AXIS] -= g_nPauseSteps[E_AXIS];
					g_nContinueSteps[E_AXIS]				   =  g_nPauseSteps[E_AXIS];
					PrintLine::prepareDirectMove();
				}
#if FEATURE_MILLING_MODE
			}
#endif // FEATURE_MILLING_MODE
		}
		else
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "pausePrint(): pause is not available at the moment because nothing is printed" ) );
			}
			return;
		}

		g_pauseStatus = PAUSE_STATUS_PAUSED;
		return;
	}

	if( g_pauseMode == PAUSE_MODE_PAUSED )
	{
		// in case the print is paused already, we move the printer head to the pause position
		HAL::forbidInterrupts();
		g_pauseMode	  = PAUSE_MODE_PAUSED_AND_MOVED;
		g_pauseStatus = PAUSE_STATUS_PREPARE_PAUSE_2;

		determinePausePosition();
		PrintLine::prepareDirectMove();
		HAL::allowInterrupts();

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): moving to the pause position" ) );

			Com::printF( PSTR( "x;" ), Printer::directPositionTargetSteps[X_AXIS] );
			Com::printF( PSTR( ";y;" ), Printer::directPositionTargetSteps[Y_AXIS] );
			Com::printFLN( PSTR( ";z;" ), Printer::directPositionTargetSteps[Z_AXIS] );
		}

		// wait until the pause position has been reached
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): waiting for the pause position" ) );
		}

		while( g_pauseStatus != PAUSE_STATUS_PAUSED )
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			HAL::delayMilliseconds( 1 );
			Commands::checkForPeriodicalActions();

			// NOTE: do not run runStandardTasks() within this loop
			//runStandardTasks();
		}

		if( Printer::debugInfo() )
		{
			Com::printF( PSTR( "g_nPauseSteps[X_AXIS] = " ), g_nPauseSteps[X_AXIS] );
			Com::printF( PSTR( ", g_nPauseSteps[Y_AXIS] = " ), g_nPauseSteps[X_AXIS] );
			Com::printFLN( PSTR( ", g_nPauseSteps[Z_AXIS] = " ), g_nPauseSteps[X_AXIS] );
			Com::printFLN( PSTR( "pausePrint(): the pause position has been reached" ) );
		}
		return;
	}

#if FEATURE_EMERGENCY_STOP_VIA_PAUSE
	if( g_pauseMode == PAUSE_MODE_PAUSED_AND_MOVED )
	{
		// in case the print is paused and the extruder is moved away already, we kill the printing
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): emergency stop" ) );
		}
		HAL::delayMilliseconds( 100 );
		Commands::emergencyStop();
		return;
	}
#endif // FEATURE_EMERGENCY_STOP_VIA_PAUSE

	return;

} // pausePrint


void continuePrint( void )
{
	const unsigned short	uMotorCurrent[] = MOTOR_CURRENT;
	char					nPrinting		 = 0;


	if( g_pauseStatus == PAUSE_STATUS_PAUSED )
	{
		BEEP_CONTINUE

#if FEATURE_MILLING_MODE
		if( Printer::operatingMode == OPERATING_MODE_PRINT )
		{
			nPrinting = 1;
		}
#else
		nPrinting = 1;
#endif // FEATURE_MILLING_MODE

		if( g_pauseMode == PAUSE_MODE_PAUSED_AND_MOVED )
		{
			// move to the continue position
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "continuePrint(): moving to the continue position" ) );
			}

#if EXTRUDER_CURRENT_PAUSE_DELAY
			if( nPrinting )
			{
				// process the extruder only in case we are in mode "print"
				setExtruderCurrent( uMotorCurrent[E_AXIS] );
			}
#endif // EXTRUDER_CURRENT_PAUSE_DELAY

			HAL::forbidInterrupts();
			if( nPrinting )
			{
				if( g_nContinueSteps[X_AXIS] )		Printer::directPositionTargetSteps[X_AXIS] += g_nContinueSteps[X_AXIS];
				if( g_nContinueSteps[Y_AXIS] )		Printer::directPositionTargetSteps[Y_AXIS] += g_nContinueSteps[Y_AXIS];
				if( g_nContinueSteps[Z_AXIS] )		Printer::directPositionTargetSteps[Z_AXIS] += g_nContinueSteps[Z_AXIS];
				if( g_nContinueSteps[E_AXIS] )		Printer::directPositionTargetSteps[E_AXIS] += g_nContinueSteps[E_AXIS];
			}
			else
			{
				// in operating mode mill, we have 2 continue positions because we have to move into x/y direction before we shall enter the work part
				if( g_nContinueSteps[X_AXIS] )		Printer::directPositionTargetSteps[X_AXIS] += g_nContinueSteps[X_AXIS];
				if( g_nContinueSteps[Y_AXIS] )		Printer::directPositionTargetSteps[Y_AXIS] += g_nContinueSteps[Y_AXIS];
			}
			PrintLine::prepareDirectMove();
			HAL::allowInterrupts();

			// wait until the continue position has been reached
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "continuePrint(): waiting for the continue position 1" ) );
			}

			g_pauseStatus = PAUSE_STATUS_PREPARE_CONTINUE_1;

			while( (Printer::directPositionTargetSteps[X_AXIS] != Printer::directPositionCurrentSteps[X_AXIS]) ||
				   (Printer::directPositionTargetSteps[Y_AXIS] != Printer::directPositionCurrentSteps[Y_AXIS]) ||
				   (Printer::directPositionTargetSteps[Z_AXIS] != Printer::directPositionCurrentSteps[Z_AXIS]) ||
				   (Printer::directPositionTargetSteps[E_AXIS] != Printer::directPositionCurrentSteps[E_AXIS]) )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				HAL::delayMilliseconds( 1 );
				Commands::checkForPeriodicalActions();

				// NOTE: do not run runStandardTasks() within this loop
				//runStandardTasks();
			}

			if( !nPrinting )
			{
				// we are in operating mode mill - get back into the work part now
				g_pauseStatus = PAUSE_STATUS_PREPARE_CONTINUE_2;

				if( g_nContinueSteps[Z_AXIS] )
				{
					Printer::directPositionTargetSteps[Z_AXIS] += g_nContinueSteps[Z_AXIS];

					while( (Printer::directPositionTargetSteps[Z_AXIS] != Printer::directPositionCurrentSteps[Z_AXIS]) )
					{
#if FEATURE_WATCHDOG
						HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

						HAL::delayMilliseconds( 1 );
						Commands::checkForPeriodicalActions();

						// NOTE: do not run runStandardTasks() within this loop
						//runStandardTasks();
					}
				}
			}
		}
		else if( g_pauseMode == PAUSE_MODE_PAUSED )
		{
			if( nPrinting )
			{
				// process the extruder only in case we are in mode "print"
#if EXTRUDER_CURRENT_PAUSE_DELAY
				setExtruderCurrent( uMotorCurrent[E_AXIS] );
#endif // EXTRUDER_CURRENT_PAUSE_DELAY

				HAL::forbidInterrupts();
				if( g_nContinueSteps[E_AXIS] )	Printer::directPositionTargetSteps[E_AXIS] += g_nContinueSteps[E_AXIS];
				PrintLine::prepareDirectMove();
				HAL::allowInterrupts();

				// wait until the continue position has been reached
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "continuePrint(): waiting for the continue position" ) );
				}

				g_pauseStatus = PAUSE_STATUS_PREPARE_CONTINUE_1;

				while( Printer::directPositionTargetSteps[E_AXIS] != Printer::directPositionCurrentSteps[E_AXIS] )
				{
#if FEATURE_WATCHDOG
					HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

					HAL::delayMilliseconds( 1 );
					Commands::checkForPeriodicalActions();

					// NOTE: do not run runStandardTasks() within this loop
					//runStandardTasks();
				}
			}
			else
			{
				g_pauseStatus = PAUSE_STATUS_PREPARE_CONTINUE_1;
			}
		}

		// wait until the next move is started
		g_pauseMode	  = PAUSE_MODE_NONE;
		g_pauseStatus = PAUSE_STATUS_NONE;
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "continuePrint(): waiting for the next move" ) );
		}

		while( !PrintLine::cur )
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			if( !PrintLine::linesCount )
			{
				// the printing won't continue in case there is nothing else to do
				break;
			}
			HAL::delayMilliseconds( 1 );
			Commands::checkForPeriodicalActions();

			// NOTE: do not run runStandardTasks() within this loop
			//runStandardTasks();
		}

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "continuePrint(): the printing has been continued" ) );
		}

		if( nPrinting )
		{
			UI_STATUS(UI_TEXT_PRINT_POS);
		}
		else
		{
			UI_STATUS(UI_TEXT_MILL_POS);
		}

	    Printer::setMenuMode(MENU_MODE_SD_PAUSED,false);
	}
	else
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "continuePrint(): continue is not available at the moment" ) );
		}
	}
	return;

} // continuePrint


void determinePausePosition( void )
{
	long	Temp;


#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		determineZPausePositionForPrint();
	}
	else
	{
		// in operating mode "mill", we must move only into z direction first in order to get the tool out of the work part
		if( g_pauseStatus == PAUSE_STATUS_PREPARE_PAUSE_2 )
		{
			determineZPausePositionForMill();
			return;
		}
	}
#else
	determineZPausePositionForPrint();
#endif // FEATURE_MILLING_MODE

    if( g_nPauseSteps[X_AXIS] )
    {
		Temp = g_nPauseSteps[X_AXIS];
		Temp += Printer::queuePositionCurrentSteps[X_AXIS];
		Temp += Printer::directPositionTargetSteps[X_AXIS];

        if( g_nPauseSteps[X_AXIS] < 0 )
		{
			if( Temp < PAUSE_X_MIN )
			{
				// we can move only partially
				Temp = PAUSE_X_MIN - Printer::directPositionTargetSteps[X_AXIS];
				Temp -= Printer::queuePositionCurrentSteps[X_AXIS];

				Printer::directPositionTargetSteps[X_AXIS] += Temp;
				g_nContinueSteps[X_AXIS]				   =  -Temp;
			}
			else
			{
				Printer::directPositionTargetSteps[X_AXIS] += g_nPauseSteps[X_AXIS];
				g_nContinueSteps[X_AXIS]				   =  -g_nPauseSteps[X_AXIS];
			}
		}
        else if( g_nPauseSteps[X_AXIS] > 0 )
		{
			if( Temp > PAUSE_X_MAX )
			{
				// we can move only partially
				Temp =  PAUSE_X_MAX - Printer::directPositionTargetSteps[X_AXIS];
				Temp -= Printer::queuePositionCurrentSteps[X_AXIS];

				Printer::directPositionTargetSteps[X_AXIS] += Temp;
				g_nContinueSteps[X_AXIS]				   =  -Temp;
			}
			else
			{
				Printer::directPositionTargetSteps[X_AXIS] += g_nPauseSteps[X_AXIS];
				g_nContinueSteps[X_AXIS]				   =  -g_nPauseSteps[X_AXIS];
			}
		}
    }

    if( g_nPauseSteps[Y_AXIS] )
    {
		Temp =  g_nPauseSteps[Y_AXIS];
		Temp += Printer::queuePositionCurrentSteps[Y_AXIS];
		Temp += Printer::directPositionTargetSteps[Y_AXIS];

		if( g_nPauseSteps[Y_AXIS] < 0 )
		{
			if( Temp < PAUSE_Y_MIN )
			{
				// we can move only partially
				Temp =  PAUSE_Y_MIN - Printer::directPositionTargetSteps[Y_AXIS];
				Temp -= Printer::queuePositionCurrentSteps[Y_AXIS];

				Printer::directPositionTargetSteps[Y_AXIS] += Temp;
				g_nContinueSteps[Y_AXIS]				   =  -Temp;
			}
			else
			{
				Printer::directPositionTargetSteps[Y_AXIS] += g_nPauseSteps[Y_AXIS];
				g_nContinueSteps[Y_AXIS]				   =  -g_nPauseSteps[Y_AXIS];
			}
		}
        else if( g_nPauseSteps[Y_AXIS] > 0 )
		{
			if( Temp > PAUSE_Y_MAX )
			{
				// we can move only partially
				Temp =  PAUSE_Y_MAX - Printer::directPositionTargetSteps[Y_AXIS];
				Temp -= Printer::queuePositionCurrentSteps[Y_AXIS];

				Printer::directPositionTargetSteps[Y_AXIS] += Temp;
				g_nContinueSteps[Y_AXIS]				   =  -Temp;
			}
			else
			{
				Printer::directPositionTargetSteps[Y_AXIS] += g_nPauseSteps[Y_AXIS];
				g_nContinueSteps[Y_AXIS]				   =  -g_nPauseSteps[Y_AXIS];
			}
		}
    }
	return;

} // determinePausePosition


void determineZPausePositionForPrint( void )
{
	long	Temp;


	// in operating mode "print", pausing drives from the current position downwards the specified g_nPauseSteps[Z_AXIS]
	if( g_nPauseSteps[Z_AXIS] )
    {
		Temp =  g_nPauseSteps[Z_AXIS];
		Temp += Printer::queuePositionCurrentSteps[Z_AXIS];

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
		Temp += Printer::compensatedPositionCurrentStepsZ;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

		Temp += Printer::directPositionTargetSteps[Z_AXIS];

        if( Temp <= PAUSE_Z_MAX )
        {
            Printer::directPositionTargetSteps[Z_AXIS] += g_nPauseSteps[Z_AXIS];
            g_nContinueSteps[Z_AXIS]				   =  -g_nPauseSteps[Z_AXIS];
        }
		else
		{
			// we can move only partially
			Temp =  PAUSE_Z_MAX - Printer::directPositionTargetSteps[Z_AXIS];
			Temp -= Printer::queuePositionCurrentSteps[Z_AXIS];

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
			Temp -= Printer::compensatedPositionCurrentStepsZ;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

			Printer::directPositionTargetSteps[Z_AXIS] += Temp;
            g_nContinueSteps[Z_AXIS]				   =  -Temp;
		}
    }
	return;

} // determineZPausePositionForPrint


void determineZPausePositionForMill( void )
{
	long	Temp;


	// in operating mode "mill", pausing drives from the current position downwards the specified g_nPauseSteps[Z_AXIS] + queuePositionCurrentSteps[Z_AXIS] because we must drive the tool out of the work part before we can move into x or y direction
	Temp =  g_nPauseSteps[Z_AXIS];
	Temp -= Printer::queuePositionCurrentSteps[Z_AXIS];	// in operating mode "mill", the bed/work part moves upwards while the milling is in progress - Printer::queuePositionCurrentSteps[Z_AXIS] is negative

    Printer::directPositionTargetSteps[Z_AXIS] += Temp;
    g_nContinueSteps[Z_AXIS]				   =  -Temp;
	return;

} // determineZPausePositionForMill


void waitUntilContinue( void )
{
	if( g_pauseStatus == PAUSE_STATUS_NONE )
	{
		// we are not paused at the moment
		return;
	}

	UI_STATUS( UI_TEXT_START_MILL );
	Printer::msecondsMilling = HAL::timeInMilliseconds();

	while( g_pauseStatus != PAUSE_STATUS_NONE )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
        UI_MEDIUM;
	}
	g_ContinueButtonPressed = 1;

} // waitUntilContinue
#endif // FEATURE_PAUSE_PRINTING


void setExtruderCurrent( unsigned short level )
{
#if FEATURE_MILLING_MODE
	if( Printer::operatingMode != OPERATING_MODE_PRINT )
	{
		// we have no extruder when we are not in print mode
		return;
	}
#endif // FEATURE_MILLING_MODE

	// set the current for the extruder motor
	setMotorCurrent( 4, level );

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "setExtruderCurrent(): new extruder current level: " ), (unsigned long)level );
	}
	return;

} // setExtruderCurrent


void processCommand( GCode* pCommand )
{
  	long	nTemp;


	if( pCommand->hasM() )
	{
		switch( pCommand->M )
		{
#if FEATURE_HEAT_BED_Z_COMPENSATION
			case 3000: // M3000 - turn the z-compensation off
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3000: disabling z compensation" ) );
					}
					queueTask( TASK_DISABLE_Z_COMPENSATION );
				}
				break;
			}
			case 3001: // M3001 - turn the z-compensation on
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( Printer::isHomed() )
					{
						if( g_ZCompensationMatrix[0][0] != EEPROM_FORMAT )
						{
							// we load the z compensation matrix before its first usage because this can take some time
							prepareZCompensation();
						}

						if( g_ZCompensationMatrix[0][0] == EEPROM_FORMAT )
						{
							// enable the z compensation only in case we have valid compensation values
							if( Printer::debugInfo() )
							{
								Com::printFLN( PSTR( "M3001: enabling z compensation" ) );
							}
							queueTask( TASK_ENABLE_Z_COMPENSATION );
						}
						else
						{
							if( Printer::debugErrors() )
							{
								Com::printF( PSTR( "M3001: the z compensation can not be enabled because the heat bed compensation matrix is not valid ( " ), g_ZCompensationMatrix[0][0] );
								Com::printF( PSTR( " / " ), EEPROM_FORMAT );
								Com::printFLN( PSTR( " )" ) );
							}
						}
					}
					else
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "M3001: the z compensation can not be enabled because the home position is unknown" ) );
						}
					}
				}
				break;
			}
			case 3002: // M3002 - configure the min z-compensation offset (units are [steps])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < (ZAXIS_STEPS_PER_MM /10) )	nTemp = ZAXIS_STEPS_PER_MM /10;
						if( nTemp > (ZAXIS_STEPS_PER_MM *10) )	nTemp = ZAXIS_STEPS_PER_MM *10;

						if( nTemp > g_maxZCompensationSteps )
						{
							// the minimal z-compensation offset can not be bigger than the maximal z-compensation offset
							nTemp = g_maxZCompensationSteps;
						}

						g_minZCompensationSteps = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3002: new min z-compensation offset: " ), g_minZCompensationSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_diffZCompensationSteps = g_maxZCompensationSteps - g_minZCompensationSteps;
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3003: // M3003 - configure the max z-compensation offset (units are [steps])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < (ZAXIS_STEPS_PER_MM /10) )	nTemp = ZAXIS_STEPS_PER_MM /10;
						if( nTemp > (ZAXIS_STEPS_PER_MM *10) )	nTemp = ZAXIS_STEPS_PER_MM *10;

						if( nTemp < g_minZCompensationSteps )
						{
							// the maximal z-compensation offset can not be smaller than the minimal z-compensation offset
							nTemp = g_minZCompensationSteps;
						}

						g_maxZCompensationSteps = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3003: new max z-compensation offset: " ), g_maxZCompensationSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_diffZCompensationSteps = g_maxZCompensationSteps - g_minZCompensationSteps;
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3007: // M3007 - configure the min z-compensation offset (units are [um])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 100 )		nTemp = 100;
						if( nTemp > 10000 )		nTemp = 10000;

						g_minZCompensationSteps = (nTemp * ZAXIS_STEPS_PER_MM) / 1000;
						if( g_minZCompensationSteps > g_maxZCompensationSteps )
						{
							// the minimal z-compensation offset can not be bigger than the maximal z-compensation offset
							g_minZCompensationSteps = g_maxZCompensationSteps;
						}

						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3007: new min z-compensation offset: " ), nTemp );
							Com::printF( PSTR( " [um]" ) );
							Com::printF( PSTR( " / " ), g_minZCompensationSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_diffZCompensationSteps = g_maxZCompensationSteps - g_minZCompensationSteps;
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3008: // M3008 - configure the max z-compensation offset (units are [um])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 100 )		nTemp = 100;
						if( nTemp > 10000 )		nTemp = 10000;

						g_maxZCompensationSteps = (nTemp * ZAXIS_STEPS_PER_MM) / 1000;;
						if( g_maxZCompensationSteps < g_minZCompensationSteps )
						{
							// the maximal z-compensation offset can not be smaller than the minimal z-compensation offset
							g_maxZCompensationSteps = g_minZCompensationSteps;
						}

						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3008: new max z-compensation offset: " ), nTemp );
							Com::printF( PSTR( " [um]" ) );
							Com::printF( PSTR( " / " ), g_maxZCompensationSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_diffZCompensationSteps = g_maxZCompensationSteps - g_minZCompensationSteps;
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

			case 3005: // M3005 - enable custom debug outputs
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 0 )	nTemp = 0;

					g_debugLevel = (char)nTemp;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3005: new debug level: " ), g_debugLevel );
					}
				}
				else
				{
					showInvalidSyntax( pCommand->M );
				}
				break;
			}

#if FEATURE_HEAT_BED_Z_COMPENSATION
			case 3006: // M3006 - configure the static z-offset (units are [um])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;

						if( nTemp < -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000) )	nTemp = -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
						if( nTemp > (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000) )	nTemp = (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);

						g_staticZSteps = (nTemp * ZAXIS_STEPS_PER_MM) / 1000;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3006: new static z-offset: " ), nTemp );
							Com::printF( PSTR( " [um]" ) );
							Com::printF( PSTR( " / " ), g_staticZSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
						Printer::ZOffset = nTemp;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
						if( HAL::eprGetInt32( EPR_RF_Z_OFFSET ) != Printer::ZOffset )
						{
							HAL::eprSetInt32( EPR_RF_Z_OFFSET, Printer::ZOffset );
							EEPROM::updateChecksum();
						}
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3010: // M3010 - start/abort the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					startHeatBedScan();
				}
				break;
			}
			case 3011: // M3011 - clear the z-compensation matrix from the EEPROM
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					clearCompensationMatrix( EEPROM_SECTOR_SIZE );
				}
				break;
			}
			case 3012: // M3012 - restore the default scan parameters
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					restoreDefaultScanParameters();
				}
				break;
			}
			case 3013: // M3013 - output the current heat bed z-compensation matrix
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( g_ZCompensationMatrix[0][0] != EEPROM_FORMAT )
					{
						// we load the z compensation matrix before its first usage because this can take some time
						prepareZCompensation();
					}

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3013: current heat bed compensation matrix: " ) );
					}
					outputCompensationMatrix();
				}
				break;
			}
			case 3020: // M3020 - configure the x start position for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

						g_nScanXStartSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3020: new x start position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXStartSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}				
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3021: // M3021 - configure the y start position for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

						g_nScanYStartSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3021: new y start position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYStartSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3022: // M3022 - configure the x step size for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM )	nTemp = HEAT_BED_SCAN_X_STEP_SIZE_MIN_MM;
						if( nTemp > 100 )								nTemp = 100;

						g_nScanXStepSizeSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3022: new x step size: " ), (int)nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXStepSizeSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3023: // M3023 - configure the y step size for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM )	nTemp = HEAT_BED_SCAN_Y_STEP_SIZE_MIN_MM;
						if( nTemp > 100 )								nTemp = 100;

						g_nScanYStepSizeSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3023: new y step size: " ), (int)nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYStepSizeSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3024: // M3024 - configure the x end position for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

						g_nScanXEndSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3024: new x end position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXEndSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_nScanXMaxPositionSteps = long(X_MAX_LENGTH * XAXIS_STEPS_PER_MM - g_nScanXEndSteps);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3024: new x max position: " ), (int)g_nScanXMaxPositionSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3025: // M3025 - configure the y end position for the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

						g_nScanYEndSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3025: new y end position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYEndSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_nScanYMaxPositionSteps = long(Y_MAX_LENGTH * YAXIS_STEPS_PER_MM - g_nScanYEndSteps);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3025: new y max position: " ), (int)g_nScanYMaxPositionSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3030: // M3030 - configure the fast step size for moving of the heat bed up during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp > 50 )	nTemp = 50;
						if( nTemp < 1 )		nTemp = 1;

						g_nScanHeatBedUpFastSteps = -nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3030: new fast step size for moving of the heat bed up: " ), (int)g_nScanHeatBedUpFastSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3031: // M3031 - configure the slow step size for moving of the heat bed up during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp > 50 )	nTemp = 50;
						if( nTemp < 1 )		nTemp = 1;

						g_nScanHeatBedUpSlowSteps = -nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3031: new slow step size for moving of the heat bed up: " ), (int)g_nScanHeatBedUpSlowSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3032: // M3032 - configure the fast step size for moving of the heat bed down during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < ZAXIS_STEPS_PER_MM /20 )	nTemp = ZAXIS_STEPS_PER_MM /20;
						if( nTemp > ZAXIS_STEPS_PER_MM )		nTemp = ZAXIS_STEPS_PER_MM;

						g_nScanHeatBedDownFastSteps = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3032: new fast step size for moving of the heat bed down: " ), (int)g_nScanHeatBedDownFastSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3033: // M3033 - configure the slow step size for moving of the heat bed down during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 50 )	nTemp = 50;

						g_nScanHeatBedDownSlowSteps = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3033: new slow step size for moving of the heat bed down: " ), (int)g_nScanHeatBedDownSlowSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3040: // M3040 - configure the delay (in ms) between two fast movements during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanFastStepDelay = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3040: new delay between two fast movements: " ), (int)g_nScanFastStepDelay );
							Com::printFLN( PSTR( " [ms]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3041: // M3041 - configure the delay (in ms) between two slow movements during the heat bed scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanSlowStepDelay = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3041: new delay between two slow movements: " ), (int)g_nScanSlowStepDelay );
							Com::printFLN( PSTR( " [ms]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3042: // M3042 - configure the delay (in ms) between reaching of a new x/y position and the test of the idle pressure
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 10000 )	nTemp = 10000;

						g_nScanIdleDelay = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3042: new idle delay: " ), (int)g_nScanIdleDelay );
							Com::printFLN( PSTR( " [ms]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3050: // M3050 - configure the contact pressure delta (in digits)
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanContactPressureDelta = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3050: new contact pressure delta: " ), (int)g_nScanContactPressureDelta );
							Com::printFLN( PSTR( " [digits]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3051: // M3051 - configure the retry pressure delta (in digits)
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanRetryPressureDelta = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3051: new retry pressure delta: " ), (int)g_nScanRetryPressureDelta );
							Com::printFLN( PSTR( " [digits]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3052: // M3052 - configure the idle pressure tolerance (in digits)
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanIdlePressureDelta = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3052: new idle pressure delta: " ), (int)g_nScanIdlePressureDelta );
							Com::printFLN( PSTR( " [digits]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3053: // M3053 - configure the number of A/D converter reads per pressure measurement
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 100 )	nTemp = 100;

						g_nScanPressureReads = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3053: new pressure reads per measurement: " ), (int)g_nScanPressureReads );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3054: // M3054 - configure the delay (in ms) between two A/D converter reads
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanPressureReadDelay = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3054: new delay between two pressure reads: " ), (int)g_nScanPressureReadDelay );
							Com::printFLN( PSTR( " [ms]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3055: // M3055 - configure the pressure tolerance (in digits) per pressure measurement
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )		nTemp = 1;
						if( nTemp > 1000 )	nTemp = 1000;

						g_nScanPressureTolerance = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3055: new scan pressure tolerance: " ), (int)g_nScanPressureTolerance );
							Com::printFLN( PSTR( " [digits]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

			case 3060:	// M3060 - output the device type and firmware version
			{
				Com::printFLN( PSTR( "Device Type: " ), UI_PRINTER_NAME );
				Com::printFLN( PSTR( "Firmware Version: " ), UI_VERSION_STRING );
				break;
			}

#if FEATURE_PAUSE_PRINTING
			case 3070: // M3070 - pause the print as if the "Pause" button would have been pressed
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 1 )		nTemp = 1;
					if( nTemp > 2 )		nTemp = 2;

					if( nTemp == 1 )
					{
						// we shall pause the printing
						queueTask( TASK_PAUSE_PRINT );
					}
					if( nTemp == 2 )
					{
						// we shall pause the printing and we shall move away
						queueTask( TASK_PAUSE_PRINT_AND_MOVE );
					}
				}
				else
				{
					showInvalidSyntax( pCommand->M );
				}

				break;
			}
			case 3071: // M3071 - wait until the print has been continued via the "Continue" button
			{
				waitUntilContinue();
				break;
			}
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_EMERGENCY_PAUSE
			case 3075: // M3075 - configure the emergency pause digits
			{
				long	nMin = g_nEmergencyPauseDigitsMin;
				long	nMax = g_nEmergencyPauseDigitsMax;

				if( pCommand->hasS() )
				{
					// test and take over the specified value - this is our new min value
					nMin = pCommand->S;
				}
				if( pCommand->hasP() )
				{
					// test and take over the specified value - this is our new max value
					nMax = pCommand->P;
				}

				if( nMin == 0 && nMax == 0 )
				{
					g_nEmergencyPauseDigitsMin = 0;
					g_nEmergencyPauseDigitsMax = 0;

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3075: the emergency pause has been disabled" ) );
					}
				}
				else if( nMin < nMax )
				{
					g_nEmergencyPauseDigitsMin = nMin;
					g_nEmergencyPauseDigitsMax = nMax;

					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3075: new min: " ), (int)g_nEmergencyPauseDigitsMin );
						Com::printF( PSTR( " [digits], new max: " ), (int)g_nEmergencyPauseDigitsMax );
						Com::printFLN( PSTR( " [digits]" ) );
					}
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printF( PSTR( "M3075: min is not smaller than max (" ), (int)g_nEmergencyPauseDigitsMin );
						Com::printF( PSTR( "/" ), (int)g_nEmergencyPauseDigitsMax );
						Com::printFLN( PSTR( " [digits])" ) );
					}
				}

				break;
			}
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_OUTPUT_FINISHED_OBJECT
			case 3079: // M3079 - output the printed object
			{
				outputObject();
				break;
			}
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FEATURE_PARK
			case 3080: // M3080 - park the printer
			{
				parkPrinter();
				break;
			}
#endif // FEATURE_PARK

#if FEATURE_WATCHDOG
			case 3090: // M3090 - test the watchdog (this command resets the firmware)
			{
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "M3090: the watchdog is going to reset the firmware" ) );
				}
				HAL::delayMilliseconds( 100 );
				HAL::testWatchdog();
				break;
			}
#endif // FEATURE_WATCHDOG

			case 3091: // M3091 - erase the external EEPROM
			{
				clearExternalEEPROM();
				break;
			}

#if FEATURE_EXTENDED_BUTTONS
			case 3100: // M3100 - configure the number of manual z steps after the "Z up" or "Z down" button has been pressed
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 1 )							nTemp = 1;
					if( nTemp > MAXIMAL_MANUAL_Z_STEPS )	nTemp = MAXIMAL_MANUAL_Z_STEPS;

					g_nManualZSteps = nTemp;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3100: new manual z steps: " ), (int)g_nManualZSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}
				}
				else
				{
					showInvalidSyntax( pCommand->M );
				}
				break;
			}
			case 3101: // M3101 - configure the number of manual extruder steps after the "Extruder up" or "Extruder down" button has been pressed
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_PRINT ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 1 )							nTemp = 1;
						if( nTemp > (EXT0_STEPS_PER_MM *10) )	nTemp = EXT0_STEPS_PER_MM *10;

						g_nManualExtruderSteps = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3101: new manual extruder steps: " ), (int)g_nManualExtruderSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
#endif // FEATURE_EXTENDED_BUTTONS

#if FEATURE_PAUSE_PRINTING
			case 3102: // M3102 - configure the offset in x, y and z direction which shall be applied in case the "Pause" button has been pressed (units are [steps])
			{
				if( pCommand->hasNoXYZ() && !pCommand->hasE() )
				{
					showInvalidSyntax( pCommand->M );
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < -(XAXIS_STEPS_PER_MM * X_MAX_LENGTH) )		nTemp = -(XAXIS_STEPS_PER_MM * X_MAX_LENGTH);
						if( nTemp > (XAXIS_STEPS_PER_MM * X_MAX_LENGTH) )		nTemp = (XAXIS_STEPS_PER_MM * X_MAX_LENGTH);

						g_nPauseSteps[X_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new x pause offset: " ), g_nPauseSteps[X_AXIS] );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < -(YAXIS_STEPS_PER_MM * Y_MAX_LENGTH) )		nTemp = -(YAXIS_STEPS_PER_MM * Y_MAX_LENGTH);
						if( nTemp > (YAXIS_STEPS_PER_MM * Y_MAX_LENGTH) )		nTemp = (YAXIS_STEPS_PER_MM * Y_MAX_LENGTH);

						g_nPauseSteps[Y_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new y pause offset: " ), g_nPauseSteps[Y_AXIS] );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )											nTemp = 0;
						if( nTemp > (ZAXIS_STEPS_PER_MM * Z_MAX_LENGTH) )		nTemp = (ZAXIS_STEPS_PER_MM * Z_MAX_LENGTH);

						g_nPauseSteps[Z_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new z pause offset: " ), g_nPauseSteps[Z_AXIS] );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasE() )
					{
						// test and take over the specified value
						nTemp = pCommand->E;
						if( nTemp < 0 )								nTemp = 0;
						if( nTemp > (EXT0_STEPS_PER_MM *5) )		nTemp = EXT0_STEPS_PER_MM *5;

						g_nPauseSteps[E_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new extruder pause offset: " ), g_nPauseSteps[E_AXIS] );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
				}
				break;
			}
			case 3105: // M3105 - configure the offset in x, y and z direction which shall be applied in case the "Pause" button has been pressed (units are [mm])
			{
				if( pCommand->hasNoXYZ() && !pCommand->hasE() )
				{
					showInvalidSyntax( pCommand->M );
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < -X_MAX_LENGTH )		nTemp = -X_MAX_LENGTH;
						if( nTemp > X_MAX_LENGTH )		nTemp = X_MAX_LENGTH;

						g_nPauseSteps[X_AXIS] = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3105: new x pause offset: " ), nTemp );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < -Y_MAX_LENGTH )		nTemp = -Y_MAX_LENGTH;
						if( nTemp > Y_MAX_LENGTH )		nTemp = Y_MAX_LENGTH;

						g_nPauseSteps[Y_AXIS] = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3105: new y pause offset: " ), nTemp );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )					nTemp = 0;
						if( nTemp > Z_MAX_LENGTH )		nTemp = Z_MAX_LENGTH;

						g_nPauseSteps[Z_AXIS] = (long)((float)nTemp * ZAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3105: new z pause offset: " ), nTemp );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasE() )
					{
						// test and take over the specified value
						nTemp = pCommand->E;
						if( nTemp < 0 )		nTemp = 0;
						if( nTemp > 5 )		nTemp = 5;

						g_nPauseSteps[E_AXIS] = (long)((float)nTemp * EXT0_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3105: new extruder pause offset: " ), nTemp );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
				}
				break;
			}
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_PARK
			case 3103: // M3103 - configure the x, y and z position which shall set when the printer is parked
			{
				if( pCommand->hasNoXYZ() )
				{
					showInvalidSyntax( pCommand->M );
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > X_MAX_LENGTH )	nTemp = X_MAX_LENGTH;

						g_nParkPosition[X_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new x park position: " ), g_nParkPosition[X_AXIS] );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Y_MAX_LENGTH )	nTemp = Y_MAX_LENGTH;

						g_nParkPosition[Y_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new y park position: " ), g_nParkPosition[Y_AXIS] );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Z_MAX_LENGTH )	nTemp = Z_MAX_LENGTH;

						g_nParkPosition[Z_AXIS] = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new z park position: " ), g_nParkPosition[Z_AXIS] );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
				}
				break;
			}
#endif // FEATURE_PARK

			case 3110:	// M3110 - force a status text
			{
				if( pCommand->hasS() )
				{
					// take over the specified value
					if( pCommand->S )
					{
						// ensure that the current text won't be overwritten
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "M3110: lock" ) );
						}
						uid.lock();
					}
					else
					{
						// allow to overwrite the current string again
						uid.unlock();
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "M3110: unlock" ) );
						}
					}
				}
				break;
			}

			case 3115:	// M3115 - set the x/y origin to the current x/y position
			{
	            Printer::setOrigin(-Printer::queuePositionLastMM[X_AXIS],-Printer::queuePositionLastMM[Y_AXIS],Printer::originOffsetMM[Z_AXIS]);
				break;
			}

#if FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON
			case 3120:	// M3120 - turn on the case fan
			{
				// enable the case fan
				Printer::prepareFanOff = 0;
				WRITE( CASE_FAN_PIN, 1 );

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "M3120: fan on" ) );
				}
				break;
			}

			case 3121:	// M3121 - turn off the case fan
			{
				// disable the case fan
				if( pCommand->hasS() )
				{
					// we shall set a new case fan off delay
					Printer::fanOffDelay =  pCommand->S;
					Printer::fanOffDelay *= 1000;	// convert from [s] to [ms]
				}

				if( Printer::fanOffDelay )
				{
					// we are going to disable the case fan after the delay
					Printer::prepareFanOff = HAL::timeInMilliseconds();

					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3121: fan off in " ), pCommand->S );
						Com::printFLN( PSTR( " [s]" ) );
					}
				}
				else
				{
					// we are going to disable the case fan now
					Printer::prepareFanOff = 0;
					WRITE(CASE_FAN_PIN, 0);

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3121: fan off" ) );
					}
				}
				break;
			}
#endif // FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON

#if FEATURE_FIND_Z_ORIGIN
			case 3130: // M3130 - start/stop the search of the z-origin
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					startFindZOrigin();
				}
				break;
			}
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_WORK_PART_Z_COMPENSATION
			case 3140: // M3140 - turn the z-compensation off
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3140: disabling z compensation" ) );
					}
					queueTask( TASK_DISABLE_Z_COMPENSATION );
				}
				break;
			}
			case 3141: // M3141 - turn the z-compensation on
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( Printer::isHomed() )
					{
						if( g_ZCompensationMatrix[0][0] != EEPROM_FORMAT )
						{
							// we load the z compensation matrix before its first usage because this can take some time
							prepareZCompensation();
						}

						if( g_ZCompensationMatrix[0][0] == EEPROM_FORMAT )
						{
							// enable the z compensation only in case we have valid compensation values
							if( Printer::debugInfo() )
							{
								Com::printFLN( PSTR( "M3141: enabling z compensation" ) );
							}

/*							if( g_nZOriginPosition[X_AXIS] && g_nZOriginPosition[Y_AXIS] )
							{
								Com::printF( PSTR( "g_nZOriginPosition[X_AXIS] = " ), g_nZOriginPosition[X_AXIS] );
								Com::printFLN( PSTR( ", g_nZOriginPosition[Y_AXIS] = " ), g_nZOriginPosition[Y_AXIS] );
							}
*/							queueTask( TASK_ENABLE_Z_COMPENSATION );
						}
						else
						{
							if( Printer::debugErrors() )
							{
								Com::printF( PSTR( "M3141: the z compensation can not be enabled because the work part compensation matrix is not valid ( " ), g_ZCompensationMatrix[0][0] );
								Com::printF( PSTR( " / " ), EEPROM_FORMAT );
								Com::printFLN( PSTR( " )" ) );
							}
						}
					}
					else
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "M3141: the z compensation can not be enabled because the home position is unknown" ) );
						}
					}
				}
				break;
			}

			case 3146: // M3146 - configure the static z-offset (units are [um])
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;

						if( nTemp < -(WORK_PART_MAX_STATIC_Z_OFFSET_MM * 1000) )	nTemp = -(WORK_PART_MAX_STATIC_Z_OFFSET_MM * 1000);
						if( nTemp > (WORK_PART_MAX_STATIC_Z_OFFSET_MM * 1000) )		nTemp = (WORK_PART_MAX_STATIC_Z_OFFSET_MM * 1000);

						g_staticZSteps = (nTemp * ZAXIS_STEPS_PER_MM) / 1000;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3146: new static z-offset: " ), nTemp );
							Com::printF( PSTR( " [um]" ) );
							Com::printF( PSTR( " / " ), g_staticZSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}

			case 3149: // M3149 - get/choose the active work part z-compensation matrix
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						nTemp = pCommand->S;

						if( nTemp < 1 || nTemp > EEPROM_MAX_WORK_PART_SECTORS )
						{
							if( Printer::debugErrors() )
							{
								Com::printF( PSTR( "M3149: invalid work part (" ), nTemp );
								Com::printFLN( PSTR( ")" ) );
							}
							break;
						}

						if( g_nActiveWorkPart != nTemp )
						{
							// we have to switch to another work part
							switchActiveWorkPart( (char)nTemp );
						}
					}

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3149: currently active work part: " ), g_nActiveWorkPart );
					}
				}
				break;
			}
			case 3150: // M3150 - start/abort the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						nTemp = pCommand->S;
					}
					else
					{
						nTemp = 0;
					}

					startWorkPartScan( (char)nTemp );
				}
				break;
			}
			case 3151: // M3151 - clear the specified z-compensation matrix from the EEPROM
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						nTemp = pCommand->S;
					}
					else
					{
						// we clear the current z-compensation matrix in case no other z-compensation matrix is specified
						nTemp = g_nActiveWorkPart;
					}

					if( nTemp < 1 || nTemp > EEPROM_MAX_WORK_PART_SECTORS )
					{
						if( Printer::debugErrors() )
						{
							Com::printF( PSTR( "M3151: invalid work part (" ), nTemp );
							Com::printFLN( PSTR( ")" ) );
						}
						break;
					}

					// switch to the specified work part
					g_nActiveWorkPart = (char)nTemp;
					writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_WORK_PART, g_nActiveWorkPart );
					clearCompensationMatrix( EEPROM_SECTOR_SIZE + (unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveWorkPart) );

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3151: cleared z-compensation matrix: " ), nTemp );
					}
				}
				break;
			}
			case 3152: // M3152 - restore the default scan parameters
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					restoreDefaultScanParameters();
				}
				break;
			}
			case 3153: // M3153 - output the specified work part z-compensation matrix
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						nTemp = pCommand->S;
					}
					else
					{
						// we output the current z-compensation matrix in case no other z-compensation matrix is specified
						nTemp = g_nActiveWorkPart;
					}

					if( nTemp < 0 || nTemp > EEPROM_MAX_WORK_PART_SECTORS )
					{
						if( Printer::debugErrors() )
						{
							Com::printF( PSTR( "M3153: invalid work part (" ), nTemp );
							Com::printFLN( PSTR( ")" ) );
						}
						break;
					}

					if( g_nActiveWorkPart != nTemp )
					{
						// we have to switch to another work part
						switchActiveWorkPart( (char)nTemp );
					}

					if( g_ZCompensationMatrix[0][0] != EEPROM_FORMAT )
					{
						// we load the z compensation matrix before its first usage because this can take some time
						prepareZCompensation();
					}

					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "M3153: current work part compensation matrix: " ) );
					}
					outputCompensationMatrix();
				}
				break;
			}
			case 3160: // M3160 - configure the x start position for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

						g_nScanXStartSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3160: new x start position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXStartSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}				
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3161: // M3161 - configure the y start position for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

						g_nScanYStartSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3161: new y start position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYStartSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3162: // M3162 - configure the x step size for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < WORK_PART_SCAN_X_STEP_SIZE_MIN_MM )	nTemp = WORK_PART_SCAN_X_STEP_SIZE_MIN_MM;
						if( nTemp > 100 )								nTemp = 100;

						g_nScanXStepSizeMm	  = nTemp;
						g_nScanXStepSizeSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3162: new x step size: " ), (int)g_nScanXStepSizeMm );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXStepSizeSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3163: // M3163 - configure the y step size for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM )	nTemp = WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM;
						if( nTemp > 100 )								nTemp = 100;

						g_nScanYStepSizeMm	  = nTemp;
						g_nScanYStepSizeSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3163: new y step size: " ), (int)g_nScanYStepSizeMm );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYStepSizeSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3164: // M3164 - configure the x end position for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

						g_nScanXEndSteps = (long)((float)nTemp * XAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3164: new x end position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanXEndSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_nScanXMaxPositionSteps = long(X_MAX_LENGTH * XAXIS_STEPS_PER_MM - g_nScanXEndSteps);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3164: new x max position: " ), (int)g_nScanXMaxPositionSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
			case 3165: // M3165 - configure the y end position for the work part scan
			{
				if( isSupportedMCommand( pCommand->M, OPERATING_MODE_MILL ) )
				{
					if( pCommand->hasS() )
					{
						// test and take over the specified value
						nTemp = pCommand->S;
						if( nTemp < 5 )						nTemp = 5;
						if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

						g_nScanYEndSteps = (long)((float)nTemp * YAXIS_STEPS_PER_MM);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3165: new y end position: " ), nTemp );
							Com::printF( PSTR( " [mm], " ), (int)g_nScanYEndSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}

						g_nScanYMaxPositionSteps = long(Y_MAX_LENGTH * YAXIS_STEPS_PER_MM - g_nScanYEndSteps);
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3165: new y max position: " ), (int)g_nScanYMaxPositionSteps );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					else
					{
						showInvalidSyntax( pCommand->M );
					}
				}
				break;
			}
#endif // FEATURE_WORK_PART_Z_COMPENSATION

			case 3200: // M3200 - reserved for test and debug
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1:
						{
							Com::printFLN( PSTR( "lowest free RAM: " ), Commands::lowestRAMValue );

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
							Com::printFLN( PSTR( "z-compensation matrix x: " ), COMPENSATION_MATRIX_MAX_X );
							Com::printFLN( PSTR( "z-compensation matrix y: " ), COMPENSATION_MATRIX_MAX_Y );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
							break;
						}
						case 2:
						{
							Com::printFLN( PSTR( "Homing feedrates:" ) );
							Com::printF( PSTR( "x = " ), Printer::homingFeedrate[X_AXIS] );
							Com::printF( PSTR( ", y = " ), Printer::homingFeedrate[Y_AXIS] );
							Com::printFLN( PSTR( ", z = " ), Printer::homingFeedrate[Z_AXIS] );
							break;
						}
						case 3:
						{
							if( pCommand->hasS() )
							{
								switch( pCommand->S )
								{
									case  1:	BEEP_SHORT					break;
									case  2:	BEEP_LONG					break;
									case  3:	BEEP_START_PRINTING			break;
									case  4:	BEEP_ABORT_PRINTING			break;
									case  5:	BEEP_STOP_PRINTING			break;
									case  6:	BEEP_PAUSE					break;
									case  7:	BEEP_CONTINUE				break;
									case  8:	BEEP_START_HEAT_BED_SCAN	break;
									case  9:	BEEP_ABORT_HEAT_BED_SCAN	break;
									case 10:	BEEP_STOP_HEAT_BED_SCAN		break;
									case 11:	BEEP_SERVICE_INTERVALL		break;
									case 12:	BEEP_ALIGN_EXTRUDERS		break;
									case 13:	BEEP_WRONG_FIRMWARE			break;
								}
							}
							break;
						}
						case 4:
						{
							// simulate blocking of the z-axis
							Com::printFLN( PSTR( "M3200: block Z" ) );
							Printer::blockZ = 1;
							break;
						}
						case 5:
						{
							// simulate a temp sensor error
							Com::printFLN( PSTR( "M3200: simulating a defect temperature sensor" ) );
							Printer::flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
							reportTempsensorError();
							break;
						}
						case 6:
						{
							Com::printF( PSTR( "nCPS X;" ), Printer::queuePositionCurrentSteps[X_AXIS] );
							Com::printF( PSTR( "; nCPS Y;" ), Printer::queuePositionCurrentSteps[Y_AXIS] );
							Com::printF( PSTR( "; nCPS Z;" ), Printer::queuePositionCurrentSteps[Z_AXIS] );
							Com::printF( PSTR( "; tCZ;" ), Printer::compensatedPositionTargetStepsZ );
							Com::printF( PSTR( "; cCZ;" ), Printer::compensatedPositionCurrentStepsZ );
							Com::printF( PSTR( "; tPSZ;" ), Printer::directPositionTargetSteps[Z_AXIS] );
							Com::printF( PSTR( "; cPSZ;" ), Printer::directPositionCurrentSteps[Z_AXIS] );
							Com::printF( PSTR( "; dZ;" ), Printer::queuePositionTargetSteps[Z_AXIS] );
							Com::printF( PSTR( "; cZ;" ), Printer::queuePositionLastSteps[Z_AXIS] );
							Com::printFLN( PSTR( "; Int32;" ), g_debugInt32 );
							break;
						}

#if FEATURE_FIND_Z_ORIGIN
						case 7:
						{
							Com::printF( PSTR( "Z-Origin X: " ), g_nZOriginPosition[X_AXIS] );
							Com::printF( PSTR( "; Z-Origin Y: " ), g_nZOriginPosition[Y_AXIS] );
							Com::printFLN( PSTR( "; Z-Origin Z: " ), Printer::staticCompensationZ );
							break;
						}
#endif // FEATURE_FIND_Z_ORIGIN

						case 8:
						{
							HAL::forbidInterrupts();
							if( PrintLine::cur )
							{
								Com::printF( PSTR( "Current command: " ) );
								PrintLine::cur->logLine2();
							}
							else
							{
								Com::printFLN( PSTR( "Current command = NULL " ) );
							}
							HAL::allowInterrupts();
							break;
						}
						case 9:
						{
							Com::printFLN( PSTR( "debug level: "), Printer::debugLevel );
							break;
						}
						case 10:
						{
							Com::printF( PSTR( "g_debugLevel= "), g_debugLevel );
							Com::printF( PSTR( ", g_debugLog= "), g_debugLog );
							Com::printF( PSTR( ", g_debugInt16= "), g_debugInt16 );
							Com::printF( PSTR( ", g_debugUInt16= "), (unsigned long)g_debugUInt16 );
							Com::printFLN( PSTR( ", g_debugInt32= "), g_debugInt32 );
							break;
						}
						case 11:
						{
#if FEATURE_MILLING_MODE
							Com::printF( PSTR( "operating mode= "), Printer::operatingMode );
							Com::printFLN( PSTR( "") );
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
							Com::printF( PSTR( "Z endstop type= "), Printer::ZEndstopType );
							Com::printF( PSTR( ", Z-Min= "), Printer::isZMinEndstopHit() );
							Com::printF( PSTR( ", Z-Max= "), Printer::isZMaxEndstopHit() );
							Com::printF( PSTR( ", lastZDirection= "), Printer::lastZDirection );
							Com::printF( PSTR( ", endstopZMinHit= "), Printer::endstopZMinHit );
							Com::printF( PSTR( ", endstopZMaxHit= "), Printer::endstopZMaxHit );
							Com::printF( PSTR( ", stepsSinceZMinEndstop= "), Printer::stepsSinceZMinEndstop );
							Com::printF( PSTR( ", stepsSinceZMaxEndstop= "), Printer::stepsSinceZMaxEndstop );
							Com::printF( PSTR( ", ZEndstopUnknown= "), Printer::ZEndstopUnknown );
							Com::printFLN( PSTR( "") );
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
							break;
						}
						case 12:
						{
							Com::printF( PSTR( "extruder= "), Extruder::current->id );
							Com::printF( PSTR( ", g_nHeatBedScanStatus= "), g_nHeatBedScanStatus );
							break;
						}
						case 13:
						{
							Com::printFLN( PSTR( "LCD re-initialization") );
							initializeLCD();
						}
						case 14:
						{
							Com::printF( PSTR( "target = " ), Printer::directPositionTargetSteps[X_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionTargetSteps[Y_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionTargetSteps[Z_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionTargetSteps[E_AXIS] );
							Com::printF( PSTR( "; current = " ), Printer::directPositionCurrentSteps[X_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionCurrentSteps[Y_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionCurrentSteps[Z_AXIS] );
							Com::printF( PSTR( "; " ), Printer::directPositionCurrentSteps[E_AXIS] );
							Com::printFLN( PSTR( "" ) );
							break;
						}
					}
				}

				break;
			}

#if FEATURE_24V_FET_OUTPUTS
			case 3300: // M3300 - configure the 24V FET outputs ( on/off )
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1:
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S == 0 )
								{
									Printer::enableFET1 = 0;
									WRITE(FET1, Printer::enableFET1);
									Com::printFLN( PSTR( " 24V FET1-Output = off ") );
								}
								else
								{
									Printer::enableFET1 = 1;
									WRITE(FET1, Printer::enableFET1);
									Com::printFLN( PSTR( " 24V FET1-Output = on ") );
								}

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
								HAL::eprSetByte( EPR_RF_FET1_MODE, Printer::enableFET1 );
								EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							}
							break;
						}
						case 2:
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S == 0 )
								{
									Printer::enableFET2 = 0;
									WRITE(FET2, Printer::enableFET2);
									Com::printFLN( PSTR( " 24V FET2-Output = off ") );
								}
								else
								{
									Printer::enableFET2 = 1;
									WRITE(FET2, Printer::enableFET2);
									Com::printFLN( PSTR( " 24V FET2-Output = on ") );
								}

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
								HAL::eprSetByte( EPR_RF_FET2_MODE, Printer::enableFET2 );
								EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							}
							break;
						}
						case 3:
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S == 0 )
								{
									Printer::enableFET3 = 0;
									WRITE(FET3, Printer::enableFET3);
									Com::printFLN( PSTR( " 24V FET3-Output = off ") );
								}
								else
								{
									Printer::enableFET3 = 1;
									WRITE(FET3, Printer::enableFET3);
									Com::printFLN( PSTR( " 24V FET3-Output = on ") );
								}

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
								HAL::eprSetByte( EPR_RF_FET3_MODE, Printer::enableFET3 );
								EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							}
							break;
						}
					}
				}
				else
				{
					showInvalidSyntax( pCommand->M );
				}

				break;
			}
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_230V_OUTPUT
			case 3301: // M3301 - configure the 230V output ( on/off )
			{
				if( pCommand->hasS() )
				{
					if( pCommand->S == 0 )
					{
						Printer::enable230VOutput = 0;
						WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
						Com::printFLN( PSTR( "230V output = off") );
					}
					else
					{
						Printer::enable230VOutput = 1;
						WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);
						Com::printFLN( PSTR( "230V output = on") );
					}

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
					HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, Printer::enable230VOutput );
					EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
				}
				else
				{
					showInvalidSyntax( pCommand->M );
				}

				break;
			}
#endif // FEATURE_230V_OUTPUT

#if FEATURE_RGB_LIGHT_EFFECTS
			case 3303: // M3303 - configure the RGB light effects for heating
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1: // red
						{
							if( pCommand->hasS() )
							{
								if( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBHeatingR = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_HEATING_R, g_uRGBHeatingR );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING )
									{
										setRGBTargetColors( g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB );
									}
									Com::printFLN( PSTR( "RGB_HEATING_R = "), g_uRGBHeatingR );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_HEATING_R out of range ") );
								}
							}
							break;
						}
						case 2: // green
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBHeatingG = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_HEATING_G, g_uRGBHeatingG );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING )
									{
										setRGBTargetColors( g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB );
									}
									Com::printFLN( PSTR( "RGB_HEATING_G = "), g_uRGBHeatingG );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_HEATING_G out of range ") );
								}
							}
							break;
						}
						case 3: // blue
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBHeatingB = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_HEATING_B, g_uRGBHeatingB );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_HEATING )
									{
										setRGBTargetColors( g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB );
									}
									Com::printFLN( PSTR( "RGB_HEATING_B = "), g_uRGBHeatingB );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_HEATING_B out of range ") );
								}
							}
							break;
						}
					}
				}

				break;
			}
			case 3304: // M3304 - configure the RGB light effects for printing
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1: // red
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBPrintingR = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_PRINTING_R, g_uRGBPrintingR );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING )
									{
										setRGBTargetColors( g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB );
									}
									Com::printFLN( PSTR( "RGB_PRINTING_R = "), g_uRGBPrintingR );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_PRINTING_R out of range ") );
								}
							}
							break;
						}
						case 2: // green
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBPrintingG = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_PRINTING_G, g_uRGBPrintingG );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING )
									{
										setRGBTargetColors( g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB );
									}
									Com::printFLN( PSTR( "RGB_PRINTING_G = "), g_uRGBPrintingG );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_PRINTING_G out of range ") );
								}
							}
							break;
						}
						case 3: // blue
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBPrintingB = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_PRINTING_B, g_uRGBPrintingB );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_PRINTING )
									{
										setRGBTargetColors( g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB );
									}
									Com::printFLN( PSTR( "RGB_PRINTING_B = "), g_uRGBPrintingB );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_PRINTING_B out of range ") );
								}
							}
							break;
						}
					}
				}

				break;
			}
			case 3305: // M3305 - configure the RGB light effects for cooling
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1: // red
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBCoolingR = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_COOLING_R, g_uRGBCoolingR );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING )
									{
										setRGBTargetColors( g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB );
									}
									Com::printFLN( PSTR( "RGB_COOLING_R = "), g_uRGBCoolingR );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_COOLING_R out of range ") );
								}
							}
							break;
						}
						case 2: // green
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBCoolingG = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_COOLING_G, g_uRGBCoolingG );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING )
									{
										setRGBTargetColors( g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB );
									}
									Com::printFLN( PSTR( "RGB_COOLING_G = "), g_uRGBCoolingG );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_COOLING_G out of range ") );
								}
							}
							break;
						}
						case 3: // B
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBCoolingB = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_COOLING_B, g_uRGBCoolingB );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_COOLING )
									{
										setRGBTargetColors( g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB );
									}
									Com::printFLN( PSTR( "RGB_COOLING_B = "), g_uRGBCoolingB );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_COOLING_B out of range ") );
								}
							}
							break;
						}
					}
				}

				break;
			}
			case 3306: // M3306 - configure the RGB light effects for idle
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1: // red
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBIdleR = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_IDLE_R, g_uRGBIdleR );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE )
									{
										setRGBTargetColors( g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB );
									}
									Com::printFLN( PSTR( "RGB_IDLE_R = "), g_uRGBIdleR );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_IDLE_R out of range ") );
								}
							}
							break;
						}
						case 2: // green
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBIdleG = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_IDLE_G, g_uRGBIdleG );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE )
									{
										setRGBTargetColors( g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB );
									}
									Com::printFLN( PSTR( "RGB_IDLE_G = "), g_uRGBIdleG );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_IDLE_G out of range ") );
								}
							}
							break;
						}
						case 3: // blue
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBIdleB = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_IDLE_B, g_uRGBIdleB );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_AUTOMATIC && Printer::RGBLightStatus == RGB_STATUS_IDLE )
									{
										setRGBTargetColors( g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB );
									}
									Com::printFLN( PSTR( "RGB_IDLE_B = "), g_uRGBIdleB );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_IDLE_B out of range ") );
								}
							}
							break;
						}
					}
				}

				break;
			}
			case 3307: // M3307 - configure the manual RGB light colors
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case 1: // red
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBManualR = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_MANUAL_R, g_uRGBManualR );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_MANUAL )
									{
										setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );
									}
									Com::printFLN( PSTR( "RGB_MANUAL_R = "), g_uRGBManualR );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_MANUAL_R out of range ") );
								}
							}
							break;
						}
						case 2: // G
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBManualG = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_MANUAL_G, g_uRGBManualG );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_MANUAL )
									{
										setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );
									}
									Com::printFLN( PSTR( "RGB_MANUAL_G = "), g_uRGBManualG );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_MANUAL_G out of range ") );
								}
							}
							break;
						}
						case 3: // B
						{
							if( pCommand->hasS() )
							{
								if ( pCommand->S >= 0 && pCommand->S <= 255 )
								{
									g_uRGBManualB = pCommand->S;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
									HAL::eprSetByte( EPR_RF_RGB_MANUAL_B, g_uRGBManualB );
									EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE

									if( Printer::RGBLightMode == RGB_MODE_MANUAL )
									{
										setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );
									}
									Com::printFLN( PSTR( "RGB_MANUAL_B = "), g_uRGBManualB );
								}
								else
								{
									Com::printFLN( PSTR( "RGB_MANUAL_B out of range ") );
								}
							}
							break;
						}
					}
				}

				break;
			}
			case 3308: // M3308 - configure the RGB light mode
			{
				if( pCommand->hasP() )
				{
					switch( pCommand->P )
					{
						case RGB_MODE_OFF:
						{
							Printer::RGBLightStatus			= RGB_STATUS_NOT_AUTOMATIC;
							Printer::RGBLightMode			= RGB_MODE_OFF;
							Printer::RGBLightModeForceWhite = 0;

							setRGBTargetColors( 0, 0, 0 );

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
							HAL::eprSetByte( EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode );
							EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							break;
						}
						case RGB_MODE_WHITE:
						{
							Printer::RGBLightStatus			= RGB_STATUS_NOT_AUTOMATIC;
							Printer::RGBLightMode			= RGB_MODE_WHITE;
							Printer::RGBLightModeForceWhite = 0;

							setRGBTargetColors( 255, 255, 255 );

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
							HAL::eprSetByte( EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode );
							EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							break;
						}
						case RGB_MODE_AUTOMATIC:
						{
							Printer::RGBLightStatus			= RGB_STATUS_AUTOMATIC;
							Printer::RGBLightMode			= RGB_MODE_AUTOMATIC;
							Printer::RGBLightModeForceWhite = 0;

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
							HAL::eprSetByte( EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode );
							EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							break;
						}
						case RGB_MODE_MANUAL:
						{
							Printer::RGBLightStatus			= RGB_STATUS_NOT_AUTOMATIC;
							Printer::RGBLightMode			= RGB_MODE_MANUAL;
							Printer::RGBLightModeForceWhite = 0;

							setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
							HAL::eprSetByte( EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode );
							EEPROM::updateChecksum();
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
							break;
						}
					}
				}

				break;
			}
#endif // FEATURE_RGB_LIGHT_EFFECTS
		}
	}

	return;

} // processCommand


void runStandardTasks( void )
{
	GCode*	pCode;


#if FEATURE_WATCHDOG
	HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	GCode::readFromSerial();
	pCode = GCode::peekCurrentCommand();
	if( pCode )
	{
#if DEBUG_COMMAND_PEEK
		Com::printFLN( PSTR( "runStandardTasks(): peek" ) );
#endif // DEBUG_COMMAND_PEEK

		Commands::executeGCode( pCode );
		pCode->popCurrentCommand();
	}
    Commands::checkForPeriodicalActions(); 
	return;

} // runStandardTasks


void queueTask( char task )
{
	while( PrintLine::linesCount >= MOVE_CACHE_SIZE )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		// wait for a free entry in movement cache
		GCode::readFromSerial();
		Commands::checkForPeriodicalActions();
	}
  
	PrintLine::queueTask( task );
	return;

} // queueTask


extern void processButton( int nAction )
{
	long	Temp;


	switch( nAction )
	{
#if FEATURE_EXTENDED_BUTTONS
		case UI_ACTION_RF_HEAT_BED_UP:
		{
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
			if( Printer::ZEndstopUnknown )
			{
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): heat bed up: moving aborted (perform a z-homing first)" ) );
				}
				break;
			}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

			// show that we are active
			previousMillisCmd = HAL::timeInMilliseconds();

			if( Printer::processAsDirectSteps() )
			{
				// we are printing at the moment, the hardware buttons allow to configure an additional, manual offset
#if FEATURE_ENABLE_MANUAL_Z_SAFETY
				HAL::forbidInterrupts();
				Temp =  Printer::directPositionTargetSteps[Z_AXIS] - g_nManualZSteps;
				Temp += Printer::queuePositionCurrentSteps[Z_AXIS];

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
				Temp += Printer::compensatedPositionCurrentStepsZ;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

				HAL::allowInterrupts();
				if( Temp < -MANUAL_Z_OVERRIDE_MAX )
				{
					// do not allow to drive the heat bed into the extruder
					if( Printer::debugErrors() )
					{
						Com::printF( PSTR( "processButton(): heat bed up: moving aborted (directPositionTargetSteps[Z_AXIS] = " ), Printer::directPositionTargetSteps[Z_AXIS] );
						Com::printF( PSTR( ", g_nManualZSteps = " ), g_nManualZSteps );
						Com::printF( PSTR( ", queuePositionCurrentSteps[Z_AXIS] = " ), Printer::queuePositionCurrentSteps[Z_AXIS] );

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
						Com::printF( PSTR( ", compensatedPositionCurrentStepsZ = " ), Printer::compensatedPositionCurrentStepsZ );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

						Com::printFLN( PSTR( "" ) );
					}
					break;
				}
				else
#endif // FEATURE_ENABLE_MANUAL_Z_SAFETY
				{
					Printer::unsetAllSteppersDisabled();
					Printer::enableZStepper();

					HAL::forbidInterrupts();
					Printer::directPositionTargetSteps[Z_AXIS] -= g_nManualZSteps;
					if( Printer::directPositionTargetSteps[Z_AXIS] < EXTENDED_BUTTONS_Z_MIN )
					{
						Printer::directPositionTargetSteps[Z_AXIS] = EXTENDED_BUTTONS_Z_MIN;
					}
					HAL::allowInterrupts();

					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "processButton(): current manual Z steps: " ), Printer::directPositionTargetSteps[Z_AXIS] );
						Com::printFLN( PSTR( " [steps]" ) );
					}
				}
			}
			else
			{
				// we are not printing at the moment, the hardware buttons behave like the standard position menu
				nextPreviousZAction( -1 );
			}
			break;
		}
		case UI_ACTION_RF_HEAT_BED_DOWN:
		{
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
			if( Printer::ZEndstopUnknown )
			{
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): heat bed down: moving aborted (perform a z-homing first)" ) );
				}
				break;
			}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

			// show that we are active
			previousMillisCmd = HAL::timeInMilliseconds();

			if( Printer::processAsDirectSteps() )
			{
				// we are printing at the moment, the hardware buttons allow to configure an additional, manual offset
				Printer::unsetAllSteppersDisabled();
				Printer::enableZStepper();

				HAL::forbidInterrupts();
				Printer::directPositionTargetSteps[Z_AXIS] += g_nManualZSteps;
				if( Printer::directPositionTargetSteps[Z_AXIS] > EXTENDED_BUTTONS_Z_MAX )
				{
					Printer::directPositionTargetSteps[Z_AXIS] = EXTENDED_BUTTONS_Z_MAX;
				}
				HAL::allowInterrupts();

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): current manual Z steps: " ), Printer::directPositionTargetSteps[Z_AXIS] );
					Com::printFLN( PSTR( " [steps]" ) );
				}
			}
			else
			{
				// we are not printing at the moment, the hardware buttons behave like the standard position menu
				nextPreviousZAction( 1 );
			}
			break;
		}
		case UI_ACTION_RF_EXTRUDER_OUTPUT:
		{
#if !EXTRUDER_ALLOW_COLD_MOVE
			if( Extruder::current->tempControl.currentTemperatureC < UI_SET_MIN_EXTRUDER_TEMP )
			{
				// we do not allow to move the extruder in case it is not heated up enough
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): extruder output: aborted" ) );
				}
				break;
			}
#endif // !EXTRUDER_ALLOW_COLD_MOVE

			// show that we are active
			previousMillisCmd = HAL::timeInMilliseconds();

			//if( Printer::processAsDirectSteps() )
			if( true )
			{
				// we are printing at the moment - use direct steps
				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): extruder output: " ), (int)g_nManualExtruderSteps );
					Com::printFLN( PSTR( " [steps]" ) );
				}

#if DEBUG_DIRECT_MOVE
				Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_OUTPUT: direct step" ) );
#endif // DEBUG_DIRECT_MOVE

				HAL::forbidInterrupts();
				Extruder::enable();
				Printer::directPositionTargetSteps[E_AXIS] += g_nManualExtruderSteps;
				HAL::allowInterrupts();

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): current manual E steps: " ), (int)Printer::directPositionTargetSteps[E_AXIS] );
					Com::printFLN( PSTR( " [steps]" ) );
				}
			}
			else
			{
				// we are not printing at the moment - use direct moves
				if( PrintLine::direct.stepsRemaining )
				{
					// we are moving already, there is nothing more to do
#if DEBUG_DIRECT_MOVE
					Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_OUTPUT: busy" ) );
#endif // DEBUG_DIRECT_MOVE
					break;
				}

#if DEBUG_DIRECT_MOVE
				Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_OUTPUT: direct move" ) );
#endif // DEBUG_DIRECT_MOVE

				HAL::forbidInterrupts();
				float	fTemp = Printer::feedrate;
				Printer::feedrate = 5;
				Printer::directPositionTargetSteps[E_AXIS] = Printer::directPositionCurrentSteps[E_AXIS] + (long)(float(EXTRUDE_MAXLENGTH) * Extruder::current->stepsPerMM);
				PrintLine::prepareDirectMove();
				PrintLine::direct.task = TASK_MOVE_FROM_BUTTON;
				Printer::feedrate = fTemp;
				HAL::allowInterrupts();
			}

#if DEBUG_DIRECT_MOVE
			Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_OUTPUT: end" ) );
#endif // DEBUG_DIRECT_MOVE
			break;
		}
		case UI_ACTION_RF_EXTRUDER_RETRACT:
		{
#if !EXTRUDER_ALLOW_COLD_MOVE
			if( Extruder::current->tempControl.currentTemperatureC < UI_SET_MIN_EXTRUDER_TEMP )
			{
				// we do not allow to move the extruder in case it is not heated up enough
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): extruder retract: aborted" ) );
				}
				break;
			}
#endif // !EXTRUDER_ALLOW_COLD_MOVE

			// show that we are active
			previousMillisCmd = HAL::timeInMilliseconds();

			//if( Printer::processAsDirectSteps() )
			if( true )
			{
				// we are printing at the moment - use direct steps
				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): extruder retract: " ), (int)g_nManualExtruderSteps );
					Com::printFLN( PSTR( " [steps]" ) );
				}

#if DEBUG_DIRECT_MOVE
				Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_RETRACT: direct step" ) );
#endif // DEBUG_DIRECT_MOVE

				HAL::forbidInterrupts();
				Extruder::enable();
				Printer::directPositionTargetSteps[E_AXIS] -= g_nManualExtruderSteps;
				HAL::allowInterrupts();

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): current manual E steps: " ), (int)Printer::directPositionTargetSteps[E_AXIS] );
					Com::printFLN( PSTR( " [steps]" ) );
				}
			}
			else
			{
				// we are not printing at the moment - use direct moves
				if( PrintLine::direct.stepsRemaining )
				{
					// we are moving already, there is nothing more to do
#if DEBUG_DIRECT_MOVE
					Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_RETRACT: busy" ) );
#endif // DEBUG_DIRECT_MOVE
					break;
				}

#if DEBUG_DIRECT_MOVE
				Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_RETRACT: direct move" ) );
#endif // DEBUG_DIRECT_MOVE

				HAL::forbidInterrupts();
				float	fTemp = Printer::feedrate;
				Printer::feedrate = 5;
				Printer::directPositionTargetSteps[E_AXIS] = Printer::directPositionCurrentSteps[E_AXIS] - (long)(float(EXTRUDE_MAXLENGTH) * Extruder::current->stepsPerMM);
				PrintLine::prepareDirectMove();
				PrintLine::direct.task = TASK_MOVE_FROM_BUTTON;
				Printer::feedrate = fTemp;
				HAL::allowInterrupts();
			}

#if DEBUG_DIRECT_MOVE
			Com::printFLN( PSTR( "UI_ACTION_RF_EXTRUDER_RETRACT: end" ) );
#endif // DEBUG_DIRECT_MOVE
			break;
		}
#if FEATURE_PAUSE_PRINTING
		case UI_ACTION_RF_PAUSE:
		{
			pausePrint();
			break;
		}
		case UI_ACTION_RF_CONTINUE:
		{
			continuePrint();
			break;
		}
#endif // FEATURE_PAUSE_PRINTING

#endif // FEATURE_EXTENDED_BUTTONS

#if FEATURE_HEAT_BED_Z_COMPENSATION
		case UI_ACTION_RF_DO_HEAT_BED_SCAN:
		{
			startHeatBedScan();
			break;
		}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
		case UI_ACTION_RF_DO_WORK_PART_SCAN:
		{
			startWorkPartScan( 0 );
			break;
		}
		case UI_ACTION_RF_SET_SCAN_XY_START:
		{
			setScanXYStart();
			break;
		}
		case UI_ACTION_RF_SET_SCAN_XY_END:
		{
			setScanXYEnd();
			break;
		}
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_OUTPUT_FINISHED_OBJECT
		case UI_ACTION_RF_OUTPUT_OBJECT:
		{
			outputObject();
			break;
		}
#endif // FEATURE_OUTPUT_FINISHED_OBJECT

#if FEATURE_FIND_Z_ORIGIN
		case UI_ACTION_RF_FIND_Z_ORIGIN:
		{
			startFindZOrigin();
			break;
		}
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_PARK
		case UI_ACTION_RF_PARK:
		{
			parkPrinter();
			break;
		}
#endif // FEATURE_PARK

	}
	return;

} // processButton


void nextPreviousZAction( int8_t next )
{
	long		steps;


	if( PrintLine::direct.stepsRemaining )
	{
		// we are moving already, there is nothing more to do
		return;
	}

	if( next < 0 )	steps = -(long)((Z_MAX_LENGTH + 5) * ZAXIS_STEPS_PER_MM);
	else			steps =  (long)((Z_MAX_LENGTH + 5) * ZAXIS_STEPS_PER_MM);

#if	!FEATURE_ALLOW_UNKNOWN_POSITIONS
	if(!Printer::isHomed())
	{
		// we do not allow unknown positions and the printer is not homed, thus we do not move
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "nextPreviousZAction(): moving z aborted (not homed)" ) );
		}
		return;
	}
#endif // !FEATURE_ALLOW_UNKNOWN_POSITIONS

	if(steps<0 && Printer::isZMinEndstopHit())
	{
		// we shall move upwards but the z-min-endstop is hit already, so we do nothing
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "nextPreviousZAction(): moving z aborted (min reached)" ) );
		}
		return;
	}

	if(steps>0 && Printer::isZMaxEndstopHit())
	{
		// we shall move downwards but the z-max-endstop is hit already, so we do nothing
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "nextPreviousZAction(): moving z aborted (max reached)" ) );
		}

		PrintLine::moveRelativeDistanceInStepsReal(0,0,-Printer::axisStepsPerMM[Z_AXIS]*5,0,Printer::maxFeedrate[Z_AXIS],true);
		Commands::printCurrentPosition();

		return;
	}

	if(steps>0 && Printer::targetZPosition() >= Z_MAX_LENGTH)
	{
		// we shall move downwards but the end of the z-axis has been reached already, so we do nothing
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "nextPreviousZAction(): moving z aborted (z-max reached)" ) );
		}
		return;
	}

	// PrintLine::moveRelativeDistanceInStepsReal(0,0,steps,0,Printer::maxFeedrate[Z_AXIS],true);

	HAL::forbidInterrupts();
	Printer::directPositionTargetSteps[Z_AXIS] = Printer::directPositionCurrentSteps[Z_AXIS] + steps;
	if( Printer::processAsDirectSteps() )
	{
		// we are printing at the moment - use direct steps"
#if DEBUG_DIRECT_MOVE
		Com::printFLN( PSTR( "nextPreviousZAction(): direct step" ) );
#endif // DEBUG_DIRECT_MOVE

		Printer::unsetAllSteppersDisabled();
		Printer::enableZStepper();
	}
	else
	{
		// we are not printing at the moment - use direct moves
#if DEBUG_DIRECT_MOVE
		Com::printFLN( PSTR( "nextPreviousZAction(): direct move" ) );
#endif // DEBUG_DIRECT_MOVE

		PrintLine::prepareDirectMove();
		PrintLine::direct.task = TASK_MOVE_FROM_BUTTON;
	}
	HAL::allowInterrupts();
//	Commands::printCurrentPosition();
/*
#if DEBUG_DIRECT_MOVE
	Com::printF( PSTR( "steps=" ), steps );
	Com::printF( PSTR( ", x=" ), Printer::directPositionCurrentSteps[X_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionLastSteps[X_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionTargetSteps[X_AXIS] );
	Com::printF( PSTR( ", y=" ), Printer::directPositionCurrentSteps[Y_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionLastSteps[Y_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionTargetSteps[Y_AXIS] );
	Com::printF( PSTR( ", z=" ), Printer::directPositionCurrentSteps[Z_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionLastSteps[Z_AXIS] );
	Com::printF( PSTR( "/" ), Printer::directPositionTargetSteps[Z_AXIS] );
	Com::printFLN( PSTR( "" ) );
#endif // DEBUG_DIRECT_MOVE
*/
} // nextPreviousZAction


#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DRV8711
void drv8711Transmit( unsigned short command )
{
	char	i;


	// transfer the command (= direction, address and data)
	HAL::forbidInterrupts();
	for( i=15; i>=0; i-- )
	{
		WRITE( DRV_SDATI, command & (0x01 << i));
		HAL::delayMicroseconds( 1 );
		WRITE( DRV_SCLK, 1 );
		HAL::delayMicroseconds( 5 );
		WRITE( DRV_SCLK, 0 );
	}
	HAL::allowInterrupts();

} // drv8711Transmit


unsigned short drv8711Receive( unsigned char address )
{
	unsigned short	acknowledge = 0;
	unsigned short	temp;
	char				i;


	if( address > 7 )	return 0;

	acknowledge =  address;
	acknowledge =  acknowledge << 12;
	acknowledge |= 0x8000;

	// transfer the read request plus the register address (4 bits)
	HAL::forbidInterrupts();
	for( i=15; i>=12; i-- )
	{
		WRITE( DRV_SDATI, acknowledge & (0x01 << i));
		HAL::delayMicroseconds( 1 );
		WRITE( DRV_SCLK, 1 );
		HAL::delayMicroseconds( 5 );
		WRITE( DRV_SCLK, 0 );
	}

	HAL::delayMicroseconds( 20 );
  
	// read the acknowledge (12 bits)
	for( i=11; i>=0; i-- )
	{
		temp = READ( DRV_SDATO );
		acknowledge = acknowledge | (temp << i);
		WRITE( DRV_SCLK, 1 );
		HAL::delayMicroseconds( 25 );
		WRITE( DRV_SCLK, 0 );
		HAL::delayMicroseconds( 25 );
	}
	HAL::allowInterrupts();

	return acknowledge;

} // drv8711Receive


void drv8711EnableAll( void )
{
	// enable the chip select of all present DRV8711
	switch( DRV8711_NUM_CHANNELS )
	{
		case 5:	 {  WRITE( O1_SCS_PIN, LOW ); SET_OUTPUT( O1_SCS_PIN ); WRITE( O1_SCS_PIN, HIGH ); }  // fall through
		case 4:  {  WRITE( O0_SCS_PIN, LOW ); SET_OUTPUT( O0_SCS_PIN ); WRITE( O0_SCS_PIN, HIGH ); }  // fall through
		case 3:  {  WRITE( Z_SCS_PIN, LOW );  SET_OUTPUT( Z_SCS_PIN );  WRITE( Z_SCS_PIN, HIGH );  }  // fall through
		case 2:  {  WRITE( Y_SCS_PIN, LOW );  SET_OUTPUT( Y_SCS_PIN );  WRITE( Y_SCS_PIN, HIGH );  }  // fall through
		case 1:  {  WRITE( X_SCS_PIN, LOW );  SET_OUTPUT( X_SCS_PIN );  WRITE( X_SCS_PIN, HIGH );  }
	}

} // drv8711EnableAll


void drv8711DisableAll( void )
{
	// disable the chip select of all present DRV8711
	switch( DRV8711_NUM_CHANNELS )
	{
		case 5:  {  WRITE( O1_SCS_PIN, LOW ); }  // fall through
		case 4:  {  WRITE( O0_SCS_PIN, LOW ); }  // fall through
		case 3:  {  WRITE( Z_SCS_PIN, LOW );  }  // fall through
		case 2:  {  WRITE( Y_SCS_PIN, LOW );  }  // fall through
		case 1:  {  WRITE( X_SCS_PIN, LOW );  }
	}

} // drv8711DisableAll


void drv8711Enable( unsigned char driver )
{
	// enable the chip select of the DRV8711
	switch( driver )
	{
		case 5:  {  WRITE( O1_SCS_PIN, LOW ); SET_OUTPUT( O1_SCS_PIN ); WRITE( O1_SCS_PIN, HIGH ); break;  }
		case 4:  {  WRITE( O0_SCS_PIN, LOW ); SET_OUTPUT( O0_SCS_PIN ); WRITE( O0_SCS_PIN, HIGH ); break;  }
		case 3:  {  WRITE( Z_SCS_PIN, LOW );  SET_OUTPUT( Z_SCS_PIN );  WRITE( Z_SCS_PIN, HIGH );  break;  }
		case 2:  {  WRITE( Y_SCS_PIN, LOW );  SET_OUTPUT( Y_SCS_PIN );  WRITE( Y_SCS_PIN, HIGH );  break;  }
		case 1:  {  WRITE( X_SCS_PIN, LOW );  SET_OUTPUT( X_SCS_PIN );  WRITE( X_SCS_PIN, HIGH );  break;  }
	}

} // drv8711Enable


void drv8711Disable( unsigned char driver )
{
	// disable the chip select of the DRV8711
	switch( driver )
	{
		case 5:  {  WRITE( O1_SCS_PIN, LOW ); break;  }
		case 4:  {  WRITE( O0_SCS_PIN, LOW ); break;  }
		case 3:  {  WRITE( Z_SCS_PIN, LOW );  break;  }
		case 2:  {  WRITE( Y_SCS_PIN, LOW );  break;  }
		case 1:  {  WRITE( X_SCS_PIN, LOW );  break;  }
	}

} // drv8711Disable


void drv8711Init( void )
{
	char	i;
  

	// configure the pins
	WRITE( DRV_RESET1, LOW );
	SET_OUTPUT( DRV_RESET1 );

#if DRV_RESET2
	WRITE( DRV_RESET2, LOW );
	SET_OUTPUT( DRV_RESET2 );
#endif // DRV_RESET2

	WRITE( DRV_SCLK, LOW );
	SET_OUTPUT( DRV_SCLK );
	WRITE( DRV_SDATI, LOW );
	SET_OUTPUT( DRV_SDATI );

	// configure the following inputs as pullup
	WRITE( DRV_SDATO, HIGH );
	WRITE( DRV_FAULT, HIGH );
	WRITE( X_STALL_PIN, HIGH );
	WRITE( Y_STALL_PIN, HIGH );
	WRITE( Z_STALL_PIN, HIGH );
	WRITE( O0_STALL_PIN, HIGH );
	WRITE( O1_STALL_PIN, HIGH );

	// reset all DRV8711 (active high)
	WRITE( DRV_RESET1, HIGH );

#if DRV_RESET2
	WRITE( DRV_RESET2, HIGH );
#endif // DRV_RESET2

	HAL::delayMicroseconds( 5000 );
	WRITE( DRV_RESET1, LOW );

#if DRV_RESET2
	WRITE( DRV_RESET2, LOW );
#endif // DRV_RESET2

	HAL::delayMicroseconds( 5000 );
  
	// configure all registers except the motor current (= register 01)
	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_00 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_02 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_03 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_04 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_05 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_06 );
	drv8711DisableAll();
	HAL::delayMicroseconds( 1 );

	drv8711EnableAll();
	drv8711Transmit( DRV8711_REGISTER_07 );
	drv8711DisableAll();

} // drv8711Init


void setMotorCurrent( unsigned char driver, unsigned short level )
{
	unsigned short	command;
	char			i;
	
	
	// NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
	// When the saturation is reached, more current causes more heating and more power loss.
	// In case of engines with lower quality, the saturation current may be reached before the nominal current.

	// configure the pins
	WRITE( DRV_SCLK, LOW );
	SET_OUTPUT( DRV_SCLK );
	WRITE( DRV_SDATI, LOW );
	SET_OUTPUT( DRV_SDATI );
		
	drv8711Enable( driver);

	// we have to write to register 01
	command = 0x1100 + level;
	drv8711Transmit( command );

	drv8711Disable( driver );

} // setMotorCurrent


void motorCurrentControlInit( void )
{
	const unsigned short	uMotorCurrent[] =  MOTOR_CURRENT;
	unsigned char			i;

	// configure all DRV8711
	drv8711Init();

	// set all motor currents
	for(i=0;i<DRV8711_NUM_CHANNELS;i++)
	{
		setMotorCurrent( i+1, uMotorCurrent[i] );
	}

} // motorCurrentControlInit

#endif // CURRENT_CONTROL_DRV8711


void cleanupXPositions( void )
{
	HAL::forbidInterrupts();

    Printer::queuePositionCurrentSteps[X_AXIS] = 0;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	Printer::directPositionTargetSteps[X_AXIS] = Printer::directPositionCurrentSteps[X_AXIS] = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	
#if FEATURE_PAUSE_PRINTING
	g_nContinueSteps[X_AXIS] = 0;
	g_pauseStatus			 = PAUSE_STATUS_NONE;
	g_pauseMode				 = PAUSE_MODE_NONE;
	g_uPauseTime			 = 0;
	g_pauseBeepDone			 = 0;
	g_ContinueButtonPressed	 = 0;
#endif // FEATURE_PAUSE_PRINTING

	HAL::allowInterrupts();

} // cleanupXPositions


void cleanupYPositions( void )
{
	HAL::forbidInterrupts();

    Printer::queuePositionCurrentSteps[Y_AXIS] = 0;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	Printer::directPositionTargetSteps[Y_AXIS] = Printer::directPositionCurrentSteps[Y_AXIS] = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	
#if FEATURE_PAUSE_PRINTING
	g_nContinueSteps[Y_AXIS] = 0;
	g_pauseStatus			 = PAUSE_STATUS_NONE;
	g_pauseMode				 = PAUSE_MODE_NONE;
	g_uPauseTime			 = 0;
	g_pauseBeepDone			 = 0;
	g_ContinueButtonPressed  = 0;
#endif // FEATURE_PAUSE_PRINTING

	HAL::allowInterrupts();

} // cleanupYPositions


void cleanupZPositions( void )
{
	HAL::forbidInterrupts();

#if FEATURE_HEAT_BED_Z_COMPENSATION
    Printer::doHeatBedZCompensation = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    Printer::doWorkPartZCompensation = 0;
	Printer::staticCompensationZ	 = 0;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

    Printer::queuePositionCurrentSteps[Z_AXIS] = 0;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    Printer::compensatedPositionTargetStepsZ  =
    Printer::compensatedPositionCurrentStepsZ =
	g_nZScanZPosition			  = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
	g_nZOriginPosition[X_AXIS] = 0;
	g_nZOriginPosition[Y_AXIS] = 0;
	g_nZOriginPosition[Z_AXIS] = 0;
	g_nZOriginSet		= 0;
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	Printer::directPositionTargetSteps[Z_AXIS] = Printer::directPositionCurrentSteps[Z_AXIS] = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
	
#if FEATURE_PAUSE_PRINTING
	g_nContinueSteps[Z_AXIS] = 0;
	g_pauseStatus			 = PAUSE_STATUS_NONE;
	g_pauseMode				 = PAUSE_MODE_NONE;
	g_uPauseTime			 = 0;
	g_pauseBeepDone			 = 0;
	g_ContinueButtonPressed  = 0;
#endif // FEATURE_PAUSE_PRINTING

	HAL::allowInterrupts();

} // cleanupZPositions


void cleanupEPositions( void )
{
	HAL::forbidInterrupts();

#if FEATURE_EXTENDED_BUTTONS
	Printer::directPositionTargetSteps[E_AXIS] = Printer::directPositionCurrentSteps[E_AXIS] = 0;
#endif // FEATURE_EXTENDED_BUTTONS
	
#if FEATURE_PAUSE_PRINTING
	g_nContinueSteps[E_AXIS] = 0;
	g_pauseStatus			 = PAUSE_STATUS_NONE;
	g_pauseMode				 = PAUSE_MODE_NONE;
	g_uPauseTime			 = 0;
	g_pauseBeepDone			 = 0;
	g_ContinueButtonPressed  = 0;
#endif // FEATURE_PAUSE_PRINTING

	HAL::allowInterrupts();

} // cleanupEPositions


void setZOrigin( void )
{
#if FEATURE_FIND_Z_ORIGIN
	g_nZOriginPosition[X_AXIS] = Printer::queuePositionLastSteps[X_AXIS];
	g_nZOriginPosition[Y_AXIS] = Printer::queuePositionLastSteps[Y_AXIS];
	g_nZOriginPosition[Z_AXIS] = 0;
	g_nZOriginSet = 1;
#endif // FEATURE_FIND_Z_ORIGIN

	Printer::updateCurrentPosition();

	// it does not make sense to change the length here because Printer::queuePositionLastMM[Z_AXIS] can be a more or less random value
	//Printer::lengthMM[Z_AXIS] -= Printer::queuePositionLastMM[Z_AXIS];

    Printer::queuePositionLastSteps[Z_AXIS] = 0;
	Printer::originOffsetMM[Z_AXIS]			= 0; 
    Printer::updateDerivedParameter();
    Printer::updateCurrentPosition(true);
    
#if EEPROM_MODE!=0
    EEPROM::storeDataIntoEEPROM(false);

	if( Printer::debugInfo() )
	{
		Com::printFLN(Com::tEEPROMUpdated);
	}
#endif // EEPROM_MODE!=0
    
	Commands::printCurrentPosition();

	Printer::setZOriginSet(true);

	BEEP_ACCEPT_SET_POSITION

} // setZOrigin


#if FEATURE_FIND_Z_ORIGIN
void startFindZOrigin( void )
{
	if( g_nFindZOriginStatus )
	{
		if( !g_abortSearch )
		{
			// abort the finding of the z-origin
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "startFindZOrigin(): the search has been cancelled" ) );
			}
			g_abortSearch = 1;
		}
	}
	else
	{
		if( Printer::operatingMode != OPERATING_MODE_MILL )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "startFindZOrigin(): finding of the z-origin is not supported in this mode" ) );
				return;
			}
		}

/*		if( PrintLine::linesCount )
		{
			// there is some printing in progress at the moment - do not start the search in this case
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "startFindZOrigin(): the search can not be started while the milling is in progress" ) );
				return;
			}
		}
*/
		// start the search
		g_nFindZOriginStatus = 1;

#if FEATURE_HEAT_BED_Z_COMPENSATION
		// when the search is running, the z-compensation must be disabled
		if( Printer::doHeatBedZCompensation )
		{
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "startFindZOrigin(): the z compensation has been disabled" ) );
			}
			resetZCompensation();
		}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
		// when the search is running, the z-compensation must be disabled
		if( Printer::doWorkPartZCompensation )
		{
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "startFindZOrigin(): the z compensation has been disabled" ) );
			}
			resetZCompensation();
		}
#endif // FEATURE_WORK_PART_Z_COMPENSATION
	}

	return;

} // startFindZOrigin


void findZOrigin( void )
{
	static short	nMaxPressureContact;
	static short	nMinPressureContact;
	short			nCurrentPressure;
	unsigned long	uStartTime;
	unsigned long	uCurrentTime;


	if( g_abortSearch )
	{
		// the search has been aborted
		g_abortSearch		= 0;
		g_nZOriginPosition[Z_AXIS] = 0;
		g_nZOriginSet		= 0;

		// turn off the engines
		Printer::disableZStepper();

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "findZOrigin(): the search has been aborted" ) );
		}

		UI_STATUS_UPD( UI_TEXT_FIND_Z_ORIGIN_ABORTED );
		g_nFindZOriginStatus = 0;
		return;
	}

	// show that we are active
	previousMillisCmd = HAL::timeInMilliseconds();

	if( g_nFindZOriginStatus )
	{
		UI_STATUS( UI_TEXT_FIND_Z_ORIGIN );

		//HAL::delayMilliseconds( 2000 );

		switch( g_nFindZOriginStatus )
		{
			case 1:
			{
				g_abortSearch		= 0;
				g_nZOriginPosition[Z_AXIS] = 0;

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "findZOrigin(): the search has been started" ) );
				}

				if( readAveragePressure( &nCurrentPressure ) )
				{
					// some error has occurred
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "findZOrigin(): the start pressure could not be determined" ) );
					}
					g_abortSearch = 1;
					return;
				}

				nMinPressureContact = nCurrentPressure - SEARCH_Z_ORIGIN_CONTACT_PRESSURE_DELTA;
				nMaxPressureContact = nCurrentPressure + SEARCH_Z_ORIGIN_CONTACT_PRESSURE_DELTA;

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "findZOrigin(): nMinPressureContact = " ), nMinPressureContact );
					Com::printFLN( PSTR( ", nMaxPressureContact = " ), nMaxPressureContact );
				}

				Printer::enableZStepper();
				Printer::unsetAllSteppersDisabled();

				// prepare the direction of the z-axis (we have to move the milling bed up)
				prepareBedUp();

				g_nTempDirectionZ	 = -1;
				g_nFindZOriginStatus = 10;

#if DEBUG_FIND_Z_ORIGIN
				Com::printFLN( PSTR( "findZOrigin(): 1 -> 10" ) );
#endif // DEBUG_FIND_Z_ORIGIN
				break;
			}
			case 10:
			{
				// move the heat bed up until we detect the contact pressure
				uStartTime = HAL::timeInMilliseconds();
				while( 1 )
				{
#if FEATURE_WATCHDOG
					HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

					nCurrentPressure = readStrainGauge( ACTIVE_STRAIN_GAUGE );
					//g_debugInt16	 = nCurrentPressure;

					if( nCurrentPressure > nMaxPressureContact || nCurrentPressure < nMinPressureContact )
					{
						// we have reached the target pressure
						g_nFindZOriginStatus = 20;

#if DEBUG_FIND_Z_ORIGIN
						Com::printFLN( PSTR( "findZOrigin(): 10 -> 20" ) );
#endif // DEBUG_FIND_Z_ORIGIN
						return;
					}

					if( Printer::isZMinEndstopHit() )
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "findZOrigin(): the z-min endstop has been reached" ) );
						}
						g_abortSearch = 1;
						return;
					}

					g_nZOriginPosition[Z_AXIS] += moveZ( SEARCH_Z_ORIGIN_BED_UP_STEPS );

					uCurrentTime = HAL::timeInMilliseconds();
					if( (uCurrentTime - uStartTime) > SEARCH_Z_ORIGIN_BREAKOUT_DELAY )
					{
						// do not stay within this loop forever
						return;
					}

					if( g_abortSearch )
					{
						break;
					}
				}

				// we should never end up here
				break;
			}
			case 20:
			{
				// move the heat bed down again until we do not detect any contact anymore
				uStartTime = HAL::timeInMilliseconds();
				while( 1 )
				{
#if FEATURE_WATCHDOG
					HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

					nCurrentPressure = readStrainGauge( ACTIVE_STRAIN_GAUGE );
					//g_debugInt16	 = nCurrentPressure;

					if( nCurrentPressure > nMinPressureContact && nCurrentPressure < nMaxPressureContact )
					{
						// we have reached the target pressure
						g_nFindZOriginStatus = 30;

#if DEBUG_FIND_Z_ORIGIN
						Com::printFLN( PSTR( "findZOrigin(): 20 -> 30" ) );
#endif // DEBUG_FIND_Z_ORIGIN
						return;
					}

					if( Printer::isZMaxEndstopHit() )
					{
						if( Printer::debugErrors() )
						{
							Com::printFLN( PSTR( "findZOrigin(): the z-max endstop has been reached" ) );
						}
						g_abortSearch = 1;
						return;
					}

					g_nZOriginPosition[Z_AXIS] += moveZ( SEARCH_Z_ORIGIN_BED_DOWN_STEPS );

					uCurrentTime = HAL::timeInMilliseconds();
					if( (uCurrentTime - uStartTime) > SEARCH_Z_ORIGIN_BREAKOUT_DELAY )
					{
						// do not stay within this loop forever
						return;
					}

					if( g_abortSearch )
					{
						break;
					}
				}

				// we should never end up here
				break;
			}
			case 30:
			{
				// we have found the z-origin
				setZOrigin();

				g_ContinueButtonPressed = 0;
				GCode::executeFString( Com::tFindZOrigin );
				g_nFindZOriginStatus = 40;

#if DEBUG_FIND_Z_ORIGIN
				Com::printFLN( PSTR( "findZOrigin(): 30 -> 40" ) );
#endif // DEBUG_FIND_Z_ORIGIN
				break;
			}
			case 40:
			{
				if( PrintLine::linesCount )
				{
					// wait until all moves have been done
					break;
				}

				g_nFindZOriginStatus = 0;
				UI_STATUS( UI_TEXT_FIND_Z_ORIGIN_DONE );

#if DEBUG_FIND_Z_ORIGIN
				Com::printFLN( PSTR( "findZOrigin(): 40 -> 0" ) );
#endif // DEBUG_FIND_Z_ORIGIN
				break;
			}
		}
	}

	// we should never end up here
	return;

} // findZOrigin
#endif // FEATURE_FIND_Z_ORIGIN


#if FEATURE_MILLING_MODE
void switchOperatingMode( char newOperatingMode )
{
	if( newOperatingMode != OPERATING_MODE_PRINT && newOperatingMode != OPERATING_MODE_MILL )
	{
		// do not allow not-supported operating modes
		return;
	}

	Printer::operatingMode = newOperatingMode;
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		setupForPrinting();
	}
	else
	{
		setupForMilling();
	}

	g_staticZSteps = 0;
	return;

} // switchOperatingMode


void switchActiveWorkPart( char newActiveWorkPart )
{
	if( newActiveWorkPart < 1 || newActiveWorkPart > EEPROM_MAX_WORK_PART_SECTORS )
	{
		// do not allow not-supported active work parts
		return;
	}

	g_nActiveWorkPart = newActiveWorkPart;
	writeWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_WORK_PART, g_nActiveWorkPart );

	if( loadCompensationMatrix( EEPROM_SECTOR_SIZE + (unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveWorkPart) ) )
	{
		// there is no valid compensation matrix available
		initCompensationMatrix();
	}
	return;

} // switchActiveWorkPart


void setScanXYStart( void )
{
/*	if( Printer::queuePositionLastSteps[X_AXIS] > g_nScanXMaxPositionSteps ||
		Printer::queuePositionLastSteps[Y_AXIS] > g_nScanYMaxPositionSteps )
	{
		// we can not take over the new x/y start position in case it is bigger than the current x/y end position
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "setScanXYStart(): the new x/y start position can not be set because it is bigger than the current x/y end position" ) );
			Com::printF( PSTR( "current: x = " ), (float)Printer::queuePositionLastSteps[X_AXIS] / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)Printer::queuePositionLastSteps[Y_AXIS] / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
			Com::printF( PSTR( "end: x = " ), (float)g_nScanXMaxPositionSteps / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)g_nScanYMaxPositionSteps / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
		}
		BEEP_ABORT_SET_POSITION
		return;
	}
*/
	// round to Integer [mm]
	g_nScanXStartSteps = (float)Printer::queuePositionLastSteps[X_AXIS] / XAXIS_STEPS_PER_MM;
	g_nScanXStartSteps *= XAXIS_STEPS_PER_MM;
	g_nScanYStartSteps = (float)Printer::queuePositionLastSteps[Y_AXIS] / YAXIS_STEPS_PER_MM;
	g_nScanYStartSteps *= YAXIS_STEPS_PER_MM;

	if( g_nScanXStartSteps > g_nScanXMaxPositionSteps ||
		g_nScanYStartSteps > g_nScanYMaxPositionSteps )
	{
		// the new start position would be bigger than the current end position - we set the end position to the start position in this case
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "setScanXYStart(): the new x/y start position is bigger than the current x/y end position, the x/y end position will be set to the new x/y start position" ) );
			g_nScanXMaxPositionSteps = g_nScanXStartSteps;
			g_nScanYMaxPositionSteps = g_nScanYStartSteps;
		}
	}
	
	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "setScanXYStart(): the new x/y start position has been set" ) );
		Com::printF( PSTR( "start: x = " ), (float)g_nScanXStartSteps / XAXIS_STEPS_PER_MM );
		Com::printF( PSTR( ", y = " ), (float)g_nScanYStartSteps / YAXIS_STEPS_PER_MM );
		Com::printFLN( PSTR( " [mm]" ) );
		Com::printF( PSTR( "end: x = " ), (float)g_nScanXMaxPositionSteps / XAXIS_STEPS_PER_MM );
		Com::printF( PSTR( ", y = " ), (float)g_nScanYMaxPositionSteps / YAXIS_STEPS_PER_MM );
		Com::printFLN( PSTR( " [mm]" ) );
	}
	BEEP_ACCEPT_SET_POSITION
	return;

} // setScanXYStart


void setScanXYEnd( void )
{
/*	if( Printer::queuePositionLastSteps[X_AXIS] < g_nScanXStartSteps ||
		Printer::queuePositionLastSteps[Y_AXIS] < g_nScanYStartSteps )
	{
		// we can not take over the new x/y end position in case it is smaller than the current x/y start position
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "setScanXYEnd(): the new x/y end position can not be set because it is smaller than the current x/y start position" ) );
			Com::printF( PSTR( "start: x = " ), (float)g_nScanXStartSteps / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)g_nScanYStartSteps / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
			Com::printF( PSTR( "current: x = " ), (float)Printer::queuePositionLastSteps[X_AXIS] / XAXIS_STEPS_PER_MM );
			Com::printF( PSTR( ", y = " ), (float)Printer::queuePositionLastSteps[Y_AXIS] / YAXIS_STEPS_PER_MM );
			Com::printFLN( PSTR( " [mm]" ) );
		}
		BEEP_ABORT_SET_POSITION
		return;
	}
*/
	// round to Integer [mm]
	g_nScanXMaxPositionSteps =  (float)Printer::queuePositionLastSteps[X_AXIS] / XAXIS_STEPS_PER_MM;
	g_nScanXMaxPositionSteps *= XAXIS_STEPS_PER_MM;
	g_nScanYMaxPositionSteps =  (float)Printer::queuePositionLastSteps[Y_AXIS] / YAXIS_STEPS_PER_MM;
	g_nScanYMaxPositionSteps *= YAXIS_STEPS_PER_MM;

	if( g_nScanXMaxPositionSteps < g_nScanXStartSteps ||
		g_nScanYMaxPositionSteps < g_nScanYStartSteps )
	{
		// the new end position would be smaller than the current start position - we set the start position to the end position in this case
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "setScanXYEnd(): the new x/y end position is smaller than the current x/y start position, the x/y start position will be set to the new x/y end position" ) );
			g_nScanXStartSteps = g_nScanXMaxPositionSteps;
			g_nScanYStartSteps = g_nScanYMaxPositionSteps;
		}
	}
	
	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "setScanXYEnd(): the new x/y end position has been set" ) );
		Com::printF( PSTR( "start: x = " ), (float)g_nScanXStartSteps / XAXIS_STEPS_PER_MM );
		Com::printF( PSTR( ", y = " ), (float)g_nScanYStartSteps / YAXIS_STEPS_PER_MM );
		Com::printFLN( PSTR( " [mm]" ) );
		Com::printF( PSTR( "end: x = " ), (float)g_nScanXMaxPositionSteps / XAXIS_STEPS_PER_MM );
		Com::printF( PSTR( ", y = " ), (float)g_nScanYMaxPositionSteps / YAXIS_STEPS_PER_MM );
		Com::printFLN( PSTR( " [mm]" ) );
	}
	BEEP_ACCEPT_SET_POSITION
	return;

} // setScanXYEnd
#endif // FEATURE_MILLING_MODE


#if FEATURE_RGB_LIGHT_EFFECTS
void setRGBTargetColors( uint8_t R, uint8_t G, uint8_t B )
{
	g_uRGBTargetR = R;
	g_uRGBTargetG = G;
	g_uRGBTargetB = B;

} // setRGBTargetColors


void setRGBLEDs( uint8_t R, uint8_t G, uint8_t B )
{
	if( R > 0 )
	{
		TCCR4A |= (1<<COM4A1);						// R > 0 PWM start
		OCR4A  =  R*80;
	}						
	else 
	{											
		TCCR4A &= ~(1<<COM4A1);						// R = 0 PWM stop
	}

	if( G > 0 )
	{
		TCCR4A |= (1<<COM4B1);						// G > 0 PWM start
		OCR4B  =  G*80;
	}							
	else
	{
		TCCR4A &= ~(1<<COM4B1);						// G = 0 PWM stop
	}							

	if( B > 0 )
	{
		TCCR4A |= (1<<COM4C1);						// B > 0 PWM start
		OCR4C  =  B*80;
	}						
	else
	{
		TCCR4A &= ~(1<<COM4C1);						// B = 0 PWM stop
	}						

	g_uRGBCurrentR = R;
	g_uRGBCurrentG = G;
	g_uRGBCurrentB = B;

} // setRGBLEDs


void updateRGBLightStatus( void )
{
	char	newStatus = RGB_STATUS_IDLE;
	char	mode;


	if( Printer::RGBButtonBackPressed )
	{
		// toggle the white light
		Printer::RGBLightModeForceWhite = !Printer::RGBLightModeForceWhite;

		if( Printer::RGBLightModeForceWhite )
		{
			setRGBTargetColors( 255, 255, 255 );
		}
		else
		{
			if ( Printer::RGBLightMode == RGB_MODE_OFF )
			{
				setRGBTargetColors( 0, 0, 0 );
			}
			else if ( Printer::RGBLightMode == RGB_MODE_AUTOMATIC )
			{
				Printer::RGBLightStatus	= RGB_STATUS_AUTOMATIC;
			}
			else if ( Printer::RGBLightMode == RGB_MODE_MANUAL )
			{
				setRGBTargetColors( g_uRGBManualR, g_uRGBManualG, g_uRGBManualB );
			}
		}

		Printer::RGBButtonBackPressed = 0;
	}

	if( Printer::RGBLightModeForceWhite )
	{
		// there is nothing to do in case we shall display the white light
		return;
	}

	if( Printer::RGBLightStatus == RGB_STATUS_NOT_AUTOMATIC )
	{
		// there is nothing to do in case we shall not change the RGB colors automatically
		return;
	}

#if FEATURE_MILLING_MODE
	mode = Printer::operatingMode;
#else
	mode = OPERATING_MODE_PRINT;
#endif // FEATURE_MILLING_MODE

	if( mode == OPERATING_MODE_PRINT )
	{
		// operating mode print
#if NUM_EXTRUDER >= 1
		if( extruder[0].tempControl.targetTemperatureC > EXTRUDER_MIN_TEMP )
		{
			if( abs( extruder[0].tempControl.targetTemperatureC - extruder[0].tempControl.currentTemperatureC ) < RGB_LIGHT_TEMP_TOLERANCE )
			{
				// we have reached the target temperature
				newStatus = RGB_STATUS_PRINTING;
			}
			else if( extruder[0].tempControl.targetTemperatureC > extruder[0].tempControl.currentTemperatureC )
			{
				// we are still heating
				newStatus = RGB_STATUS_HEATING;
			}
			else
			{
				// we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
			}
		}
#endif // NUM_EXTRUDER >= 1

#if NUM_EXTRUDER == 2
		if( extruder[1].tempControl.targetTemperatureC > EXTRUDER_MIN_TEMP )
		{
			if( abs( extruder[1].tempControl.targetTemperatureC - extruder[1].tempControl.currentTemperatureC ) < RGB_LIGHT_TEMP_TOLERANCE )
			{
				if( newStatus == RGB_STATUS_IDLE )
				{
					// we have reached the target temperature
					newStatus = RGB_STATUS_PRINTING;
				}
			}
			else if( extruder[1].tempControl.targetTemperatureC > extruder[1].tempControl.currentTemperatureC )
			{
				// we are still heating
				newStatus = RGB_STATUS_HEATING;
			}
			else
			{
				// we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
			}
		}
#endif // NUM_EXTRUDER == 2

		if( heatedBedController.targetTemperatureC > HEATED_BED_MIN_TEMP )
		{
			if( abs( heatedBedController.targetTemperatureC - heatedBedController.currentTemperatureC ) < RGB_LIGHT_TEMP_TOLERANCE )
			{
				// we have reached the target temperature
				if( newStatus == RGB_STATUS_IDLE )
				{
					newStatus = RGB_STATUS_PRINTING;
				}
			}
			else if( heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC )
			{
				// we are still heating
				newStatus = RGB_STATUS_HEATING;
			}
					else
			{
				// we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
			}
		}

#if NUM_EXTRUDER >= 1
		if( (extruder[0].tempControl.currentTemperatureC - extruder[0].tempControl.targetTemperatureC) > COOLDOWN_THRESHOLD )
		{
			// we shall cool down
			if( newStatus == RGB_STATUS_IDLE )
			{
				newStatus = RGB_STATUS_COOLING;
			}
		}
#endif // NUM_EXTRUDER >= 1

#if NUM_EXTRUDER == 2
		if( (extruder[1].tempControl.currentTemperatureC - extruder[1].tempControl.targetTemperatureC) > COOLDOWN_THRESHOLD )
		{
			// we shall cool down
			if( newStatus == RGB_STATUS_IDLE )
			{
				newStatus = RGB_STATUS_COOLING;
			}
		}
#endif // NUM_EXTRUDER == 2

		if( (heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > COOLDOWN_THRESHOLD )
		{
			// we shall cool down
			if( newStatus == RGB_STATUS_IDLE )
			{
				newStatus = RGB_STATUS_COOLING;
			}
		}
	}
	else
	{
		// operating mode mill
		if( PrintLine::linesCount )
		{
			newStatus = RGB_STATUS_PRINTING;
		}
		else
		{
			newStatus = RGB_STATUS_IDLE;
		}
	}

	if( newStatus != Printer::RGBLightStatus )
	{
		if( newStatus == RGB_STATUS_IDLE && Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE )
		{
			// when we are in color change mode already we shall not switch back to idle
		}
		else
		{
			Printer::RGBLightStatus = newStatus;
//			Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );

			switch( Printer::RGBLightStatus )
			{
				case RGB_STATUS_PRINTING:
				{
					setRGBTargetColors( g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB );
					break;
				}
				case RGB_STATUS_HEATING:
				{
					setRGBTargetColors( g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB );
					break;
				}
				case RGB_STATUS_COOLING:
				{
					setRGBTargetColors( g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB );
					break;
				}
				case RGB_STATUS_IDLE:	// fall through
				default:
				{
					Printer::RGBLightIdleStart = HAL::timeInMilliseconds();
					Printer::RGBLightStatus	   = RGB_STATUS_IDLE;
					setRGBTargetColors( g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB );
					break;
				}
			}
		}
	}

	if( Printer::RGBLightStatus == RGB_STATUS_IDLE && Printer::RGBLightIdleStart )
	{
		if( (HAL::timeInMilliseconds() - Printer::RGBLightIdleStart) / 1000 > RGB_LIGHT_COLOR_CHANGE_DELAY )
		{
			Printer::RGBLightStatus	   = RGB_STATUS_COLOR_CHANGE;
			Printer::RGBLightIdleStart = 0;

//			Com::printFLN( PSTR( "new RGB light status: " ), Printer::RGBLightStatus );
			setRGBTargetColors( 255, 0, 0 );
		}
	}

	if( Printer::RGBLightStatus == RGB_STATUS_COLOR_CHANGE )
	{
		if( g_uRGBTargetR == g_uRGBCurrentR && 
			g_uRGBTargetG == g_uRGBCurrentG &&
			g_uRGBTargetB == g_uRGBCurrentB )
		{
			if( g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 0 )
			{
				g_uRGBTargetG = 255;
			}
			else if( g_uRGBTargetR == 255 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0 )
			{
				g_uRGBTargetR = 0;
			}
			else if( g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0 )
			{
				g_uRGBTargetB = 255;
			}
			else if( g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 255 )
			{
				g_uRGBTargetG = 0;
			}
			else if( g_uRGBTargetR == 0 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255 )
			{
				g_uRGBTargetR = 255;
			}
			else if( g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255 )
			{
				g_uRGBTargetB = 0;
			}
		}
	}

	return;

} // updateRGBLightStatus
#endif // FEATURE_RGB_LIGHT_EFFECTS


void setupForPrinting( void )
{
	Printer::flag0 &= ~PRINTER_FLAG0_TEMPSENSOR_DEFECT;

#if FEATURE_HEAT_BED_Z_COMPENSATION
	// restore the default scan parameters
	restoreDefaultScanParameters();
	
/*	// restore the last known compensation matrix
	// this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
	if( loadCompensationMatrix( EEPROM_SECTOR_SIZE ) )
	{
		// there is no valid compensation matrix available
		initCompensationMatrix();

		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "setupForPrinting(): the compensation matrix is not available" ) );
		}
	}
*/
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if EEPROM_MODE
	// read the settings from the EEPROM
	Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT);
	Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT);
	Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT);
#else
	// read the settings from Configuration.h
	Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
	Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
	Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
#endif // EEPROM_MODE

	Printer::setMenuMode( MENU_MODE_MILLER, false );
	Printer::setMenuMode( MENU_MODE_PRINTER, true );
	UI_STATUS( UI_TEXT_PRINTER_READY );
	return;

} // setupForPrinting


void setupForMilling( void )
{

#if FEATURE_WORK_PART_Z_COMPENSATION
	// we must restore the default work part scan parameters
	restoreDefaultScanParameters();

	g_nActiveWorkPart = (char)readWord24C256( I2C_ADDRESS_EXTERNAL_EEPROM, EEPROM_OFFSET_ACTIVE_WORK_PART );

	if( g_nActiveWorkPart < 1 || g_nActiveWorkPart > EEPROM_MAX_WORK_PART_SECTORS )
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "setupForMilling(): invalid active work part detected: " ), (int)g_nActiveWorkPart );
		}

		// continue with the default work part
		g_nActiveWorkPart = 1;
	}

/*	// we must restore the work part z-compensation matrix
	// this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
	if( loadCompensationMatrix( 0 ) )
	{
		// there is no valid compensation matrix available
		initCompensationMatrix();

		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "setupForMilling(): the compensation matrix is not available" ) );
		}
	}
*/
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if EEPROM_MODE
	// read the settings from the EEPROM
	Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_MILL);
	Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_MILL);
	Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_MILL);
#else
	// read the settings from Configuration.h
	Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_MILL;
	Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_MILL;
	Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_MILL;
#endif // EEPROM_MODE

	// disable all heaters
	Extruder::setHeatedBedTemperature( 0, false );
	Extruder::setTemperatureForExtruder( 0, 0, false );

	Printer::setMenuMode( MENU_MODE_PRINTER, false );
	Printer::setMenuMode( MENU_MODE_MILLER, true );
	UI_STATUS( UI_TEXT_MILLER_READY );
	return;

} // setupForMilling


void prepareZCompensation( void )
{
	char	mode;


#if FEATURE_MILLING_MODE
	mode = Printer::operatingMode;
#else
	mode = OPERATING_MODE_PRINT;
#endif // FEATURE_MILLING_MODE

#if FEATURE_HEAT_BED_Z_COMPENSATION
	if( mode == OPERATING_MODE_PRINT )
	{
		// restore the default scan parameters
		restoreDefaultScanParameters();
	
		// restore the last known compensation matrix
		// this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
		if( loadCompensationMatrix( EEPROM_SECTOR_SIZE ) )
		{
			// there is no valid compensation matrix available
			initCompensationMatrix();

			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "prepareZCompensation(): the compensation matrix is not available" ) );
			}
		}
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
	if( mode == OPERATING_MODE_MILL )
	{
		// we must restore the default work part scan parameters
		restoreDefaultScanParameters();

		// we must restore the work part z-compensation matrix
		// this operation must be performed after restoring of the default scan parameters because the values from the EEPROM can overwrite some scan parameters
		if( loadCompensationMatrix( 0 ) )
		{
			// there is no valid compensation matrix available
			initCompensationMatrix();

			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "prepareZCompensation(): the compensation matrix is not available" ) );
			}
		}
	}
#endif // FEATURE_WORK_PART_Z_COMPENSATION

	if( COMPENSATION_MATRIX_SIZE > EEPROM_SECTOR_SIZE )
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "prepareZCompensation(): the size of the compensation matrix is too big" ) );
		}
	}

} // prepareZCompensation


void resetZCompensation( void )
{
	HAL::forbidInterrupts();

	// disable and reset the z-compensation

#if FEATURE_HEAT_BED_Z_COMPENSATION
	Printer::doHeatBedZCompensation = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
	Printer::doWorkPartZCompensation = 0;
	Printer::staticCompensationZ	 = 0;
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
	Printer::compensatedPositionTargetStepsZ  = 0;
	Printer::compensatedPositionCurrentStepsZ = 0;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

	HAL::allowInterrupts();
	return;

} // resetZCompensation


unsigned char isSupportedGCommand( unsigned int currentGCode, char neededMode, char outputLog )
{
	char	currentMode = OPERATING_MODE_PRINT;


#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_MILL )
	{
		currentMode = OPERATING_MODE_MILL;
	}
#endif // FEATURE_MILLING_MODE

	if( currentMode == neededMode )
	{
		return 1;
	}

	if( Printer::debugErrors() && outputLog )
	{
		Com::printF( PSTR( "G" ), (int)currentGCode );
		Com::printFLN( PSTR( ": this command is not supported in this mode" ) );
	}
	return 0;

} // isSupportedGCommand


unsigned char isSupportedMCommand( unsigned int currentMCode, char neededMode, char outputLog )
{
	char	currentMode = OPERATING_MODE_PRINT;


#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_MILL )
	{
		currentMode = OPERATING_MODE_MILL;
	}
#endif // FEATURE_MILLING_MODE

	if( currentMode == neededMode )
	{
		return 1;
	}

	if( Printer::debugErrors() && outputLog )
	{
		Com::printF( PSTR( "M" ), (int)currentMCode );
		Com::printFLN( PSTR( ": this command is not supported in this mode" ) );
	}
	return 0;

} // isSupportedMCommand


unsigned char isMovingAllowed( const char* pszCommand, char outputLog )
{
#if FEATURE_HEAT_BED_Z_COMPENSATION
	if( g_nHeatBedScanStatus )
	{
		// do not allow manual movements while the heat bed scan is in progress
		if( Printer::debugErrors() && outputLog )
		{
			Com::printF( pszCommand );
			Com::printFLN( PSTR( ": this command can not be used while the heat bed scan is in progress" ) );
		}
		return 0;
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
		
#if FEATURE_WORK_PART_Z_COMPENSATION
	if( g_nWorkPartScanStatus )
	{
		// do not allow manual movements while the work part scan is in progress
		if( Printer::debugErrors() && outputLog )
		{
			Com::printF( pszCommand );
			Com::printFLN( PSTR( ": this command can not be used while the work part scan is in progress" ) );
		}
		return 0;
	}
#endif // FEATURE_WORK_PART_Z_COMPENSATION
		
#if FEATURE_FIND_Z_ORIGIN
	if( g_nFindZOriginStatus && g_nFindZOriginStatus != 30 )
	{
		// do not allow manual movements while the z-origin is searched
		if( Printer::debugErrors() && outputLog )
		{
			Com::printF( pszCommand );
			Com::printFLN( PSTR( ": this command can not be used while the z-origin is searched" ) );
		}
		return 0;
	}
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	if( Printer::ZEndstopUnknown )
	{
		// in case we do not know which Z-endstop is active at the moment, we do not allow to move until a z-homing has been performed
		if( Printer::debugErrors() && outputLog )
		{
			Com::printF( pszCommand );
			Com::printFLN( PSTR( ": this command can not be used until a z-homing has been performed" ) );
		}
		return 0;
	}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

	// we allow the manual movements at the moment
	return 1;

} // isMovingAllowed


unsigned char isHomingAllowed( GCode* com, char outputLog )
{
#if FEATURE_HEAT_BED_Z_COMPENSATION
	if( g_nHeatBedScanStatus )
	{
		// do not allow homing while the heat bed scan is in progress
		if( Printer::debugErrors() && outputLog )
		{
			Com::printFLN( PSTR( "G28: homing can not be performed while the heat bed scan is in progress" ) );
		}
		return 0;
	}
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
		
#if FEATURE_WORK_PART_Z_COMPENSATION
	if( g_nWorkPartScanStatus )
	{
		// do not allow homing while the work part scan is in progress
		if( Printer::debugErrors() && outputLog )
		{
			Com::printFLN( PSTR( "G28: homing can not be performed while the work part scan is in progress" ) );
		}
		return 0;
	}
#endif // FEATURE_WORK_PART_Z_COMPENSATION
		
#if FEATURE_FIND_Z_ORIGIN
	if( g_nFindZOriginStatus )
	{
		// do not allow homing while the z-origin is searched
		if( Printer::debugErrors() && outputLog )
		{
			Com::printFLN( PSTR( "G28: homing can not be performed while the z-origin is searched" ) );
		}
		return 0;
	}
#endif // FEATURE_FIND_Z_ORIGIN

	if( !com )
	{
		// there is nothing more which we can check
		return 1;
	}

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	if( Printer::ZEndstopUnknown && (com->hasX() || com->hasY() || com->hasNoXYZ()) )
	{
		// in case we do not know which Z-endstop is active at the moment, we do not allow any homing except the z-homing
		if( Printer::debugErrors() && outputLog )
		{
			Com::printFLN( PSTR( "G28: x/y homing can not be performed until a z-homing has been performed" ) );
		}

		// turn off the homing in x- and y-direction
		com->setX( 0 );
		com->setY( 0 );

		if( com->hasZ() || com->hasNoXYZ() )
		{
			com->setZ( 1 );

			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "isHomingAllowed(): x=" ), com->hasX() );
				Com::printF( PSTR( ", y=" ), com->hasY() );
				Com::printFLN( PSTR( ", z=" ), com->hasZ() );
			}
			return 1;
		}

		return 0;
	}
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

	// we allow the homing at the moment
	return 1;

} // isHomingAllowed


void showInvalidSyntax( unsigned int currentMCode )
{
	if( Printer::debugErrors() )
	{
		Com::printF( PSTR( "M" ), (int)currentMCode );
		Com::printFLN( PSTR( ": invalid syntax" ) );
	}
	return;

} // showInvalidSyntax


void addUInt32( char* pszString, uint32_t uNumber )
{
	char		szTemp[11];
	char*		pszTemp = &szTemp[10];
	uint32_t	uTemp;
	
	
	*pszTemp = '\0';
	do
	{
		uTemp	   =  uNumber;
		uNumber	   /= 10;
		*--pszTemp =  '0' + (uTemp - 10 * uNumber);
	}while(uNumber);

	strcpy( pszString, pszTemp );

} // addUInt32


void addFloat( char* pszString, float fNumber, uint8_t uDigits )
{
	float		fRounding = 0.5;
	uint32_t	uNumber;
	float		fRemainder;
	uint8_t		uRemainder;
	uint8_t		i;


	// add at the end of the string
	while( *pszString )
	{
		pszString ++;
	}

	if( isnan( fNumber ) )
	{
		strcpy( pszString, "NAN" );
		return;
	}
	if( isinf( fNumber ) )
	{
		strcpy( pszString, "INF" );
		return;
	}

	// Handle negative numbers
	if( fNumber < 0.0 )
	{
		*pszString = '-';
		pszString ++;
		*pszString = '\0';
		fNumber = -fNumber;
	}

	for( i=0; i<uDigits; i++ )
	{
		fRounding /= 10.0;
	}

	fNumber += fRounding;

	uNumber	   = (uint32_t)fNumber;
	fRemainder = fNumber - (float)uNumber;
	addUInt32( pszString, uNumber );

	if( uDigits > 0 )
	{
		while( *pszString )
		{
			pszString ++;
		}

		*pszString = '.';
		pszString ++;

		while( uDigits-- > 0 )
		{
			fRemainder *= 10.0;
			uRemainder =  (uint8_t)fRemainder;
			*pszString =  '0' + uRemainder;
			pszString ++;
			fRemainder -= uRemainder;
		}

		*pszString = '\0';
	}

} // addFloat


#if FEATURE_HEAT_BED_TEMP_COMPENSATION
float getHeatBedTemperatureOffset( float temperatureInCelsius )
{
	const unsigned char		setpointTemperatures[] = BED_SETPOINT_TEMPERATURES;
	const unsigned char		measuredTemperatures[] = BED_MEASURED_TEMPERATURES;
	float					deltaLow, deltaHigh;
	float					temp;
	char					i;


	if( temperatureInCelsius <= setpointTemperatures[0] )
	{
		// the specified temperature is below our known, measured values
		deltaLow  = 0;
		deltaHigh = measuredTemperatures[0]	- setpointTemperatures[0];

		temp =  (deltaHigh - deltaLow) / float(setpointTemperatures[0]);
		temp *= temperatureInCelsius;

#if DEBUG_HEAT_BED_TEMP_COMPENSATION
		if( temperatureInCelsius > 50 )
		{
			Com::printF( PSTR( "getHeatBedTemperatureOffset(): " ), temperatureInCelsius );
			Com::printFLN( PSTR( ", " ), temp );
		}
#endif // DEBUG_HEAT_BED_TEMP_COMPENSATION

		return temp;
	}

	if( temperatureInCelsius >= setpointTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX] )
	{
		// the specified temperature is above our known, measured values
		temp = measuredTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX] - setpointTemperatures[BED_TEMP_COMPENSATION_INDEX_MAX];

#if DEBUG_HEAT_BED_TEMP_COMPENSATION
		Com::printF( PSTR( "getHeatBedTemperatureOffset(): " ), temperatureInCelsius );
		Com::printFLN( PSTR( ", " ), temp );
#endif // DEBUG_HEAT_BED_TEMP_COMPENSATION

		return temp;
	}

	for( i=1; i<=BED_TEMP_COMPENSATION_INDEX_MAX; i++ )
	{
		if( temperatureInCelsius < setpointTemperatures[i] )
		{
			deltaLow  = measuredTemperatures[i-1] - setpointTemperatures[i-1];
			deltaHigh = measuredTemperatures[i]	  - setpointTemperatures[i];

			temp =  (deltaHigh - deltaLow) / float(setpointTemperatures[i] - setpointTemperatures[i-1]);
			temp *= (temperatureInCelsius - setpointTemperatures[i-1]);

#if DEBUG_HEAT_BED_TEMP_COMPENSATION
			Com::printF( PSTR( "getHeatBedTemperatureOffset(): " ), temperatureInCelsius );
			Com::printFLN( PSTR( ", " ), temp + deltaLow );
#endif // DEBUG_HEAT_BED_TEMP_COMPENSATION

			return (temp + deltaLow);
		}
	}

	// we should never end up here
	return 0;

} // getHeatBedTemperatureOffset
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION


#if FEATURE_TYPE_EEPROM
void determineHardwareType( void )
{
	unsigned short	uTemp;


	Printer::wrongType = 0;

	// check the stored header format
	uTemp = readWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT );

	if( uTemp != TYPE_EEPROM_FORMAT )
	{
		// we could not read the header format or the header format is wrong
		writeWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT, TYPE_EEPROM_FORMAT );
		writeWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE,	MOTHERBOARD );

		uTemp = readWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_HEADER_FORMAT );

		if( uTemp != TYPE_EEPROM_FORMAT )
		{
			if( MOTHERBOARD == DEVICE_TYPE_RF1000 )
			{
				// the RF1000 does not provide the type EEPROM, thus we should run at an RF1000 board when we end up here
				return;
			}

			// we end up here in case this firmware is for the RF2000, but the current board does not seem to be an RF2000 board
			notifyAboutWrongHardwareType( DEVICE_TYPE_RF1000 );

			Printer::wrongType = 1;
			return;
		}

		if( MOTHERBOARD == DEVICE_TYPE_RF1000 )
		{
			// we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
			notifyAboutWrongHardwareType( DEVICE_TYPE_RF2000 );

			Printer::wrongType = 1;
			return;
		}

		// when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
		return;
	}

	if( MOTHERBOARD == DEVICE_TYPE_RF1000 )
	{
		// we end up here in case this firmware is for the RF1000, but the current board seems to be an RF2000 board
		notifyAboutWrongHardwareType( DEVICE_TYPE_RF2000 );

		Printer::wrongType = 1;
		return;
	}

	// when we end up here this firmware is for the RF2000 and the current board seems to be an RF2000 board
	if( readWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE ) != MOTHERBOARD )
	{
		writeWord24C256( I2C_ADDRESS_TYPE_EEPROM, TYPE_EEPROM_OFFSET_BOARD_TYPE, MOTHERBOARD );
	}

	return;

} // determineHardwareType


void notifyAboutWrongHardwareType( unsigned char guessedHardwareType )
{
	uint16_t	duration = 1000;
	uint8_t		count	 = 4;


	switch( guessedHardwareType )
	{
		case DEVICE_TYPE_RF1000:
		{
			// we try to beep via the beeper pin of the RF1000 hardware
			SET_OUTPUT( BEEPER_PIN_RF1000 );

			for( uint8_t i=0; i<count; i++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				WRITE( BEEPER_PIN_RF1000, HIGH );
				HAL::delayMilliseconds( duration );
				WRITE( BEEPER_PIN_RF1000, LOW );
				HAL::delayMilliseconds( duration );
			}
			break;
		}
		case DEVICE_TYPE_RF2000:
		{
			// we try to beep via the beeper pin of the RF2000 hardware
			SET_OUTPUT( BEEPER_PIN_RF2000 );

			for( uint8_t i=0; i<count; i++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				WRITE( BEEPER_PIN_RF2000, HIGH );
				HAL::delayMilliseconds( duration );
				WRITE( BEEPER_PIN_RF2000, LOW );
				HAL::delayMilliseconds( duration );
			}
			break;
		}
	}
	return;

} // notifyAboutWrongHardwareType
#endif // FEATURE_TYPE_EEPROM
