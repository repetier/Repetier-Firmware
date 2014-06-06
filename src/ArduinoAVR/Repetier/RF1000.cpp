#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#else
 #include <WProgram.h>
#endif

//#include <avr/pgmspace.h>
#include "Repetier.h"
#include <Wire.h>


char			g_nDirectionX	= 0;
char			g_nDirectionY	= 0;
char			g_nDirectionZ	= 0;
char			g_nBlockZ		= 0;
char			g_nDirectionE	= 0;
unsigned long	g_lastTime		= 0;

#if FEATURE_Z_COMPENSATION
unsigned long	g_lastScanTime  = 0;
unsigned long	g_scanStartTime = 0;
char			g_abortScan		= 0;
char			g_retryScan		= 0;
char			g_scanRetries	= 0;
short			g_HeatBedCompensation[HEAT_BED_COMPENSATION_X][HEAT_BED_COMPENSATION_Y];
#endif // FEATURE_Z_COMPENSATION

#if REMEMBER_PRESSURE
short			g_HeatBedPressure[HEAT_BED_COMPENSATION_X][HEAT_BED_COMPENSATION_Y];
#endif // REMEMBER_PRESSURE

short			g_nMaxPressureContact;
short			g_nMaxPressureRetry;
short			g_nMaxPressureIdle;
short			g_nMinPressureContact;
short			g_nMinPressureRetry;
short			g_nMinPressureIdle;
short			g_nFirstIdlePressure;
short			g_nCurrentIdlePressure;
char			g_nLastZDirection			= 0;
short			g_noZCompensationSteps		= Z_COMPENSATION_NO_STEPS;
short			g_maxZCompensationSteps		= Z_COMPENSATION_MAX_STEPS;
short			g_diffZCompensationSteps	= Z_COMPENSATION_MAX_STEPS - Z_COMPENSATION_NO_STEPS;
short			g_manualCompensationSteps	= 0;
short			g_offsetHeatBedCompensation = 0;
char			g_nHeatBedScanStatus		= 0;
unsigned char	g_uHeatBedMaxX				= 0;
unsigned char	g_uHeatBedMaxY				= 0;
long			g_minX						= 0;
long			g_maxX						= 0;
long			g_minY						= 0;
long			g_maxY						= 0;
long			g_recalculatedCompensation	= 0;
char			g_debugLevel				= 0;
//short			g_debugCounter[10]			= { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
short			g_nHeatBedScanZ				= 0;
short			g_nLastHeatBedScanZ			= 0;
unsigned long	g_uStopTime					= 0;

// configurable scan parameters - the proper default values are set by restoreDefaultScanParameters()
unsigned short	g_nScanXStartSteps			= 0;
unsigned short	g_nScanXStepSizeSteps		= 0;
unsigned short	g_nScanXEndSteps			= 0;
unsigned short	g_nScanXMaxPositionSteps	= 0;
unsigned short	g_nScanYStartSteps			= 0;
unsigned short	g_nScanYStepSizeSteps		= 0;
unsigned short	g_nScanYEndSteps			= 0;
unsigned short	g_nScanYMaxPositionSteps	= 0;
char			g_nScanHeatBedUpFastSteps	= 0;
char			g_nScanHeatBedUpSlowSteps	= 0;
short			g_nScanHeatBedDownFastSteps	= 0;
char			g_nScanHeatBedDownSlowSteps	= 0;
unsigned short	g_nScanFastStepDelay		= 0;
unsigned short	g_nScanSlowStepDelay		= 0;
unsigned short	g_nScanIdleDelay			= 0;
unsigned short	g_nScanContactPressureDelta	= 0;
unsigned short	g_nScanRetryPressureDelta	= 0;
unsigned short	g_nScanIdlePressureDelta	= 0;
char			g_nScanPressureReads		= 0;
unsigned short	g_nScanPressureReadDelay	= 0;
short			g_nScanPressureTolerance	= 0;

// other configurable parameters
unsigned short	g_nManualZSteps				= DEFAULT_MANUAL_Z_STEPS;
unsigned short	g_nManualExtruderSteps		= DEFAULT_MANUAL_EXTRUDER_STEPS;

#if FEATURE_PAUSE_PRINTING
short			g_nPauseStepsX				= DEFAULT_PAUSE_STEPS_X;
short			g_nPauseStepsY				= DEFAULT_PAUSE_STEPS_Y;
short			g_nPauseStepsZ				= DEFAULT_PAUSE_STEPS_Z;
short			g_nPauseStepsExtruder		= DEFAULT_PAUSE_STEPS_EXTRUDER;
short			g_nContinueStepsX			= 0;
short			g_nContinueStepsY			= 0;
short			g_nContinueStepsZ			= 0;
short			g_nContinueStepsExtruder	= 0;
char			g_pausePrint				= 0;
char			g_printingPaused			= 0;
unsigned long	g_uPauseTime				= 0;
char			g_pauseBeepDone				= 0;
#endif // FEATURE_PAUSE_PRINTING

unsigned short	g_nNextZSteps				= DEFAULT_MANUAL_Z_STEPS;
unsigned long	g_uLastHeatBedUpTime		= 0;
unsigned long	g_uLastHeadBedDownTime		= 0;

#if FEATURE_PARK
short			g_nParkPositionX			= PARK_POSITION_X;
short			g_nParkPositionY			= PARK_POSITION_Y;
short			g_nParkPositionZ			= PARK_POSITION_Z;
#endif // FEATURE_PARK

#if FEATURE_OUTPUT_PRINTED_OBJECT
long			g_nOutputOffsetX			= OUTPUT_OFFSET_X;
long			g_nOutputOffsetY			= OUTPUT_OFFSET_Y;
long			g_nOutputOffsetZ			= OUTPUT_OFFSET_Z;
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

#if FEATURE_EMERGENCY_PAUSE
unsigned long	g_uLastPressureTime			= 0;
long			g_nPressureSum				= 0;
char			g_nPressureChecks			= 0;
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_EMERGENCY_Z_STOP
unsigned long	g_uLastZPressureTime		= 0;
long			g_nZPressureSum				= 0;
char			g_nZPressureChecks			= 0;
#endif // FEATURE_EMERGENCY_Z_STOP


void initRF1000( void )
{
	// initialize the two strain gauges
	Wire.begin();
	initStrainGauge();

#if FEATURE_Z_COMPENSATION
	// restore the default scan parameters
	restoreDefaultScanParameters();

	// restore the last known compensation matrix
	if( restoreCompensationMatrix() )
	{
		// there is no valid compensation matrix available
		initCompensationMatrix();

		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "initRF1000(): the compensation matrix is not available" ) );
		}
	}
	else
	{
		// we have restored a valid compensation matrix
		outputCompensationMatrix();
	}
#endif // FEATURE_Z_COMPENSATION

#if defined ENABLE_OUT1 && ENABLE_OUT1
	SET_OUTPUT( RF1000_OUT1_PIN );
	WRITE( RF1000_OUT1_PIN, SET_OUT1 );
#endif // ENABLE_OUT1

#if defined ENABLE_HZ3 && ENABLE_HZ3
	SET_OUTPUT( RF1000_HZ3_PIN );
	WRITE( RF1000_HZ3_PIN, SET_HZ3 );
#endif // ENABLE_HZ3

	return;

} // initRF1000


void initStrainGauge( void )
{
#if MOTHERBOARD == 12
	// configure DMS #1 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( I2C_ADDRESS_STRAIN_GAUGE_1 );
	Wire.write( 0x8C );
	Wire.endTransmission();

	// configure DMS #2 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( I2C_ADDRESS_STRAIN_GAUGE_2 );
	Wire.write( 0x8C );
	Wire.endTransmission();
#endif // MOTHERBOARD == 12

#if MOTHERBOARD == 13
	// configure DMS #1 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( I2C_ADDRESS_STRAIN_GAUGE );
	Wire.write( 0x8C );
	Wire.endTransmission();
#endif // MOTHERBOARD == 13

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


#if FEATURE_Z_COMPENSATION
void scanHeatBed( void )
{
	static unsigned char	nIndexX;
	static unsigned char	nIndexY;
	static char				nIndexYDirection;
	static char				nRetry;
	static long				nX;
	static long				nY;
	static long				nZ;
	static short			nYDirection;
	static short			nContactPressure;
	char					nLastHeatBedScanStatus = g_nHeatBedScanStatus;
	short					nTempPressure;


	// directions:
	// +x = to the right
	// -x = to the left
	// +y = heat bed moves to the front
	// -y = heat bed moves to the back
	// +z = heat bed moves down
	// -z = heat bed moves up

	if( g_abortScan )
	{
		// the scan has been aborted
		g_abortScan			 = 0;
		g_nHeatBedScanStatus = 0;
		g_nHeatBedScanZ		 = 0;
		g_nLastHeatBedScanZ	 = 0;

		// start at the home position (move the z-axis first for safety reasons)
		Printer::homeAxis( false, false, true );
		Printer::homeAxis( true, true, false );

		// turn off the engines (for cases where the heating needs some longer amount of time)
		Printer::disableXStepper();
		Printer::disableYStepper();
		Printer::disableZStepper();

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "scanHeatBed(): the scan has been aborted" ) );
		}

		UI_STATUS_UPD(UI_TEXT_HEAT_BED_SCAN_ABORTED);
		BEEP_ABORT_HEAT_BED_SCAN

		// restore the compensation values from the EEPROM
		restoreCompensationMatrix();
		return;
	}

	// show that we are active
	previousMillisCmd = HAL::timeInMilliseconds();

	if( g_nHeatBedScanStatus )
	{
		UI_STATUS(UI_TEXT_HEAT_BED_SCAN);

		if( g_retryScan )
		{
			// we have to retry to scan the current position
			g_nHeatBedScanStatus = 45;
			g_retryScan			 = 0;
		}

		switch( g_nHeatBedScanStatus )
		{
			case 1:
			{
				g_scanStartTime   = HAL::timeInMilliseconds();
				g_abortScan		  = 0;
				nContactPressure  = 0;
				g_nLastZDirection = 0;

				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "scanHeatBed(): the scan has been started" ) );
				}

				// clear all fields of the compensation matrix
				initCompensationMatrix();

				g_uHeatBedMaxX = 0;
				g_uHeatBedMaxY = 0;

				// output the currently used scan parameters
				outputScanParameters();

				g_nHeatBedScanStatus = 10;
				break;
			}
			case 10:
			{
				// start at the home position (move the z-axis first for safety reasons)
				Printer::homeAxis( false, false, true );
				Printer::homeAxis( true, true, false );

				// turn off the engines (for cases where the heating needs some longer amount of time)
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();
				g_nHeatBedScanStatus = 15;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 15:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < 1000 )
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
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < 1000 )
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
				PrintLine::moveRelativeDistanceInSteps( g_nScanXStartSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				PrintLine::moveRelativeDistanceInSteps( 0, g_nScanYStartSteps, 0, 0, MAX_FEEDRATE_Y, true, true );

				g_nHeatBedScanStatus = 30;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 30:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < 1000 )
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

				// store also the version of this compensation matrix
#if REMEMBER_PRESSURE
				g_HeatBedPressure[0][0]		= COMPENSATION_VERSION;
#endif // REMEMBER_PRESSURE

				g_HeatBedCompensation[0][0] = COMPENSATION_VERSION;

				g_nHeatBedScanStatus = 40;
				break;
			}
			case 39:
			{
				// move to the next x-position
				PrintLine::moveRelativeDistanceInSteps( g_nScanXStepSizeSteps, 0, 0, 0, MAX_FEEDRATE_X, true, true );
				nX += g_nScanXStepSizeSteps;
				nIndexX ++;

				if( nIndexX > g_uHeatBedMaxX )
				{
					g_uHeatBedMaxX = nIndexX;
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
				// move one interval in x-direction
				if( nX <= g_nScanXMaxPositionSteps )
				{
					// remember also the exact x-position of this row/column
#if REMEMBER_PRESSURE
					g_HeatBedPressure[nIndexX][0]	  = nX;
#endif // REMEMBER_PRESSURE

					g_HeatBedCompensation[nIndexX][0] = nX;

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
				g_nHeatBedScanZ		 = 0;
				nZ					 = 0;
				g_nHeatBedScanStatus = 50;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 49:
			{
				g_scanRetries		 = SCAN_RETRIES;
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
				// move away from the surface
				nZ += moveZDownFast();

				g_nHeatBedScanStatus = 55;
				break;
			}
			case 55:
			{
				if( Printer::debugInfo() )
				{
#if DEBUG_HEAT_BED_SCAN
					Com::printF( nX );
					Com::printF( PSTR( ";" ) );
					Com::printF( nY );
					Com::printF( PSTR( ";" ) );
					Com::printF( nZ );
					Com::printF( PSTR( ";" ) );
					Com::printF( nContactPressure );

#if FEATURE_Z_COMPENSATION
					// output the non compensated position values
					Com::printF( PSTR( ";;" ) );
					Com::printF( Printer::nonCompensatedPositionStepsX );
					Com::printF( PSTR( ";" ) );
					Com::printF( Printer::nonCompensatedPositionStepsY );
					Com::printF( PSTR( ";" ) );
					Com::printF( Printer::nonCompensatedPositionStepsZ );
					Com::printF( PSTR( ";" ) );
					Com::printF( Printer::currentCompensationZ );
#endif // FEATURE_Z_COMPENSATION

					Com::printFLN( PSTR( " " ) );
#endif // DEBUG_HEAT_BED_SCAN
				}

				// remember the z-position and the exact y-position of this row/column
				g_HeatBedCompensation[nIndexX][nIndexY] = (short)nZ;
				g_HeatBedCompensation[0][nIndexY]		= nY;

#if REMEMBER_PRESSURE
				// remember the pressure and the exact y-position of this row/column
				g_HeatBedPressure[nIndexX][nIndexY]		= nContactPressure;
				g_HeatBedPressure[0][nIndexY]			= nY;
#endif // REMEMBER_PRESSURE

				if( nYDirection > 0 )
				{
					if( (nY+nYDirection) > g_nScanYMaxPositionSteps )
					{
						// we have reached the end of this column
						g_nHeatBedScanStatus = 39;
						break;
					}
				}
				else
				{
					if( (nY+nYDirection) < g_nScanYStartSteps )
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

				if( nIndexY > g_uHeatBedMaxY )
				{
					g_uHeatBedMaxY = nIndexY;
				}
		
				g_nHeatBedScanStatus = 49;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 60:
			{
				// move back to the home position (move the z-axis first for safety reasons)
				Printer::homeAxis( false, false, true );
				Printer::homeAxis( true, true, false );

				// turn off the engines
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();

				g_nHeatBedScanStatus = 65;
				break;
			}
			case 65:
			{
				if( Printer::debugInfo() )
				{
					// output the determined compensation
					Com::printFLN( PSTR( "scanHeatBed(): raw compensation matrix: " ) );
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

				// prepare the compensation matrix for fast usage during the actual printing
				prepareCompensationMatrix();

				Com::printFLN( PSTR( "scanHeatBed(): g_uHeatBedMaxY.1 = " ), (int)g_uHeatBedMaxY );

				// convert the compensation matrix for fast usage during the actual printing
				convertCompensationMatrix();

				Com::printFLN( PSTR( "scanHeatBed(): g_uHeatBedMaxY.2 = " ), (int)g_uHeatBedMaxY );

				if( Printer::debugInfo() )
				{
					// output the converted compensation matrix
					Com::printFLN( PSTR( "scanHeatBed(): converted compensation matrix: " ) );
					outputCompensationMatrix();
				}

				// save the determined values to the EEPROM
				if( saveCompensationMatrix() )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "scanHeatBed(): the compensation matrix could not be saved" ) );
					}
				}
				else
				{
					if( Printer::debugInfo() )
					{
						Com::printFLN( PSTR( "scanHeatBed(): the compensation matrix has been saved" ) );
					}
				}

				g_nHeatBedScanStatus = 80;
				g_lastScanTime		 = HAL::timeInMilliseconds();
				break;
			}
			case 80:
			{
				if( (HAL::timeInMilliseconds() - g_lastScanTime) < 1000 )
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

				g_nHeatBedScanStatus = 100;
				break;
			}
			case 100:
			{
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
#endif // FEATURE_Z_COMPENSATION


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
		if( (Extruder::getHeatedBedTemperature() + 2) < heatedBedController.targetTemperatureC )
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


#if FEATURE_Z_COMPENSATION
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

	if( *pnIdlePressure < SCAN_IDLE_PRESSURE_MIN || *pnIdlePressure > SCAN_IDLE_PRESSURE_MAX )
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
			g_abortScan = 1;
			return -1;
		}
	}

	// we should never end up here
	g_abortScan = 1;
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
			nTempPressure =  readStrainGauge( SCAN_STRAIN_GAUGE );
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
		runStandardTasks();
	}

	if( Printer::debugErrors() )
	{
		Com::printFLN( PSTR( "readAveragePressure(): the pressure is not plausible" ) );
	}
	g_abortScan		   = 1;
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

		nSteps			=  moveZ( g_nScanHeatBedUpFastSteps );
		nZ				+= nSteps;
		g_nHeatBedScanZ += nSteps;

		runStandardTasks();

		if( g_abortScan )
		{
			break;
		}

		if( g_nHeatBedScanZ < -Z_COMPENSATION_MAX_STEPS || g_nHeatBedScanZ > Z_COMPENSATION_MAX_STEPS )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZUpFast(): the z position went out of range, retries = " ), (int)g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryScan = 1;
			else				g_abortScan = 1;
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

		nSteps			=  moveZ( g_nScanHeatBedDownSlowSteps );
		nZ				+= nSteps;
		g_nHeatBedScanZ += nSteps;
		runStandardTasks();

		if( g_abortScan )
		{
			break;
		}

		if( g_nHeatBedScanZ < -Z_COMPENSATION_MAX_STEPS || g_nHeatBedScanZ > Z_COMPENSATION_MAX_STEPS )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZDownSlow(): the z position went out of range, retries = " ), g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryScan = 1;
			else				g_abortScan = 1;
			break;
		}
		if( g_nLastHeatBedScanZ )
		{
			if( (g_nHeatBedScanZ > g_nLastHeatBedScanZ && (g_nHeatBedScanZ - g_nLastHeatBedScanZ) > g_nScanHeatBedDownFastSteps) ||
				(g_nHeatBedScanZ < g_nLastHeatBedScanZ && (g_nLastHeatBedScanZ - g_nHeatBedScanZ) > g_nScanHeatBedDownFastSteps) )
			{
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "moveZDownSlow(): the z position delta went out of range, retries = " ), g_scanRetries );
				}
			
				if( g_scanRetries )	g_retryScan = 1;
				else				g_abortScan = 1;
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

		nSteps			=  moveZ( g_nScanHeatBedUpSlowSteps );
		nZ				+= nSteps;
		g_nHeatBedScanZ += nSteps;
		runStandardTasks();

		if( g_abortScan )
		{
			break;
		}

		if( g_nHeatBedScanZ < -Z_COMPENSATION_MAX_STEPS || g_nHeatBedScanZ > Z_COMPENSATION_MAX_STEPS )
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "moveZUpSlow(): the z position went out of range, retries = " ), g_scanRetries );
			}
			
			if( g_scanRetries )	g_retryScan = 1;
			else				g_abortScan = 1;
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
	g_nLastHeatBedScanZ = g_nHeatBedScanZ;
	HAL::delayMilliseconds( g_nScanFastStepDelay );

	nSteps			=  moveZ( g_nScanHeatBedDownFastSteps );
	nZ				+= nSteps;
	g_nHeatBedScanZ += nSteps;
	runStandardTasks();

	if( readAveragePressure( &nTempPressure ) )
	{
		// some error has occurred
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "moveZDownFast(): the pressure could not be determined" ) );
		}
		g_abortScan = 1;
		return nZ;
	}

	if( Printer::debugInfo() )
	{
		Com::printFLN( PSTR( "moveZDownFast(): " ), (int)nTempPressure );
	}
	return nZ;

} // moveZDownFast


int moveX( int nSteps )
{
	int		i;
	int		nMaxLoops;
	

	// Warning: this function does not check any end stops

	// choose the direction
	if( nSteps >= 0 )
	{
		HAL::forbidInterrupts();
		nMaxLoops = nSteps;

		WRITE( X_DIR_PIN, !INVERT_X_DIR );
		HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
	}
	else
	{
		HAL::forbidInterrupts();
		nMaxLoops = -nSteps;

		WRITE( X_DIR_PIN, INVERT_X_DIR );
		HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
	}
	
	// perform the steps
	for( i=0; i<nMaxLoops; i++ )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::delayMicroseconds( XYZ_STEPPER_HIGH_DELAY );
		WRITE( X_STEP_PIN, HIGH );

        HAL::delayMicroseconds( XYZ_STEPPER_LOW_DELAY );
		WRITE( X_STEP_PIN, LOW );
	}

	HAL::allowInterrupts();
	return nSteps;

} // moveX


int moveY( int nSteps )
{
	int		i;
	int		nMaxLoops;
	

	// Warning: this function does not check any end stops

	// choose the direction
	if( nSteps >= 0 )
	{
		HAL::forbidInterrupts();
		nMaxLoops = nSteps;

		WRITE( Y_DIR_PIN, !INVERT_Y_DIR );
		HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
	}
	else
	{
		HAL::forbidInterrupts();
		nMaxLoops = -nSteps;

		WRITE( Y_DIR_PIN, INVERT_Y_DIR );
		HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
	}
	
	// perform the steps
	for( i=0; i<nMaxLoops; i++ )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::delayMicroseconds( XYZ_STEPPER_HIGH_DELAY );
		WRITE( Y_STEP_PIN, HIGH );

        HAL::delayMicroseconds( XYZ_STEPPER_LOW_DELAY );
		WRITE( Y_STEP_PIN, LOW );
	}

	HAL::allowInterrupts();
	return nSteps;

} // moveY


int moveZ( int nSteps )
{
	int		i;
	int		nMaxLoops;
	

	// Warning: this function does not check any end stops
/*	if( g_nDirectionZ )
	{
		// this function must not move the z-axis in case the "main" interrupt (bresenhamStep()) is running at the moment
		return 0;
	}			
*/
	// choose the direction
	if( nSteps >= 0 )
	{
		HAL::forbidInterrupts();
		nMaxLoops = nSteps;

		if( g_nLastZDirection != 1 )
		{
			WRITE( Z_DIR_PIN, !INVERT_Z_DIR );
			HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
			g_nLastZDirection = 1;
		}
	}
	else
	{
		HAL::forbidInterrupts();
		nMaxLoops = -nSteps;

		if( g_nLastZDirection != -1 )
		{
			WRITE( Z_DIR_PIN, INVERT_Z_DIR );
			HAL::delayMicroseconds( XYZ_DIRECTION_CHANGE_DELAY );
			g_nLastZDirection = -1;
		}
	}
	
	// perform the steps
	for( i=0; i<nMaxLoops; i++ )
	{
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::delayMicroseconds( XYZ_STEPPER_HIGH_DELAY );
		WRITE( Z_STEP_PIN, HIGH );

        HAL::delayMicroseconds( XYZ_STEPPER_LOW_DELAY );
		WRITE( Z_STEP_PIN, LOW );
	}

	HAL::allowInterrupts();
	return nSteps;

} // moveZ


int moveExtruder( int nSteps )
{
	int		i;
	int		nMaxLoops;
	
	
	// choose the direction
	if( nSteps >= 0 )
	{
		HAL::forbidInterrupts();
		Extruder::enable();
        HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);

		nMaxLoops = nSteps;
		Extruder::setDirection(true);
        HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);
	}
	else
	{
		HAL::forbidInterrupts();
		Extruder::enable();
        HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);

		nMaxLoops = -nSteps;
		Extruder::setDirection(false);
        HAL::delayMicroseconds(EXTRUDER_DIRECTION_CHANGE_DELAY);
	}
	
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
	g_nScanXStartSteps			= SCAN_X_START_STEPS;
	g_nScanXStepSizeSteps		= SCAN_X_STEP_SIZE_STEPS;
	g_nScanXEndSteps			= SCAN_X_END_STEPS;
	g_nScanXMaxPositionSteps	= SCAN_X_MAX_POSITION_STEPS;

	g_nScanYStartSteps			= SCAN_Y_START_STEPS;
	g_nScanYStepSizeSteps		= SCAN_Y_STEP_SIZE_STEPS;
	g_nScanYEndSteps			= SCAN_Y_END_STEPS;
	g_nScanYMaxPositionSteps	= SCAN_Y_MAX_POSITION_STEPS;

	g_nScanHeatBedUpFastSteps	= SCAN_HEAT_BED_UP_FAST_STEPS;
	g_nScanHeatBedUpSlowSteps	= SCAN_HEAT_BED_UP_SLOW_STEPS;
	g_nScanHeatBedDownFastSteps	= SCAN_HEAT_BED_DOWN_FAST_STEPS;
	g_nScanHeatBedDownSlowSteps	= SCAN_HEAT_BED_DOWN_SLOW_STEPS;
	g_nScanFastStepDelay		= SCAN_FAST_STEP_DELAY_MS;
	g_nScanSlowStepDelay		= SCAN_SLOW_STEP_DELAY_MS;
	g_nScanIdleDelay			= SCAN_IDLE_DELAY_MS;

	g_nScanContactPressureDelta	= SCAN_CONTACT_PRESSURE_DELTA;
	g_nScanRetryPressureDelta	= SCAN_RETRY_PRESSURE_DELTA;
	g_nScanIdlePressureDelta	= SCAN_IDLE_PRESSURE_DELTA;

	g_nScanPressureReads		= SCAN_PRESSURE_READS;
	g_nScanPressureTolerance	= SCAN_PRESSURE_TOLERANCE;
	g_nScanPressureReadDelay	= SCAN_PRESSURE_READ_DELAY_MS;

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

		Com::printF( PSTR( "" ), (int)g_nScanXStartSteps );				Com::printFLN( PSTR( ";[steps];g_nScanXStartSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanXStepSizeSteps );			Com::printFLN( PSTR( ";[steps];g_nScanXStepSizeSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanXEndSteps );				Com::printFLN( PSTR( ";[steps];g_nScanXEndSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanXMaxPositionSteps );		Com::printFLN( PSTR( ";[steps];g_nScanXMaxPositionSteps" ) );

		Com::printF( PSTR( "" ), (int)g_nScanYStartSteps );				Com::printFLN( PSTR( ";[steps];g_nScanYStartSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanYStepSizeSteps );			Com::printFLN( PSTR( ";[steps];g_nScanYStepSizeSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanYEndSteps );				Com::printFLN( PSTR( ";[steps];g_nScanYEndSteps" ) );
		Com::printF( PSTR( "" ), (int)g_nScanYMaxPositionSteps );		Com::printFLN( PSTR( ";[steps];g_nScanYMaxPositionSteps" ) );

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
	memset( g_HeatBedCompensation, 0, HEAT_BED_COMPENSATION_X*HEAT_BED_COMPENSATION_Y*2 );
	return;

} // initCompensationMatrix


void outputCompensationMatrix( void )
{
	if( Printer::debugInfo() )
	{
		short	x;
		short	y;


		Com::printFLN( PSTR( "Compensation matrix:" ) );
		Com::printFLN( PSTR( "front left ... front right" ) );
		Com::printFLN( PSTR( "...        ...         ..." ) );
		Com::printFLN( PSTR( "back left  ...  back right" ) );

		g_offsetHeatBedCompensation = -32000;
		for( y=0; y<g_uHeatBedMaxY; y++ )
		{
			for( x=0; x<g_uHeatBedMaxX; x++ )
			{
				Com::printF( PSTR( ";" ), g_HeatBedCompensation[x][y] );
				if( x>0 && y>0 && g_HeatBedCompensation[x][y] > g_offsetHeatBedCompensation )
				{
					g_offsetHeatBedCompensation = g_HeatBedCompensation[x][y];
				}
			}
			Com::printFLN( PSTR( " " ) );
		}
		Com::printFLN( PSTR( "offset = " ), g_offsetHeatBedCompensation );
		Com::printFLN( PSTR( "g_uHeatBedMaxX = " ), g_uHeatBedMaxX );
		Com::printFLN( PSTR( "g_uHeatBedMaxY = " ), g_uHeatBedMaxY );
	}

	return;

} // outputCompensationMatrix


char prepareCompensationMatrix( void )
{
	short	x;
	short	y;


	// perform some safety checks first
	if( g_HeatBedCompensation[0][0] != COMPENSATION_VERSION )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid version detected: " ), g_HeatBedCompensation[0][0] );
			Com::printF( PSTR( " (expected: " ), COMPENSATION_VERSION );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}
	
	if( g_uHeatBedMaxX > HEAT_BED_COMPENSATION_X || g_uHeatBedMaxX < 2 )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid x dimension detected: " ), g_uHeatBedMaxX );
			Com::printF( PSTR( " (max expected: " ), HEAT_BED_COMPENSATION_X );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	if( g_uHeatBedMaxY > HEAT_BED_COMPENSATION_Y || g_uHeatBedMaxY < 2 )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "prepareCompensationMatrix(): invalid y dimension detected: " ), g_uHeatBedMaxY );
			Com::printF( PSTR( " (max expected: " ), HEAT_BED_COMPENSATION_Y );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	if( g_HeatBedCompensation[2][0] > 0 )
	{
		// we have to fill x[1] with the values of x[2]
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] > 0" ) );
		}
		g_HeatBedCompensation[1][0] = 0;
		for( y=1; y<g_uHeatBedMaxY; y++ )
		{
			g_HeatBedCompensation[1][y] = g_HeatBedCompensation[2][y];
		}
	}
	else
	{
		// we have to shift all x columns one index
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[2] = 0" ) );
		}
		for( x=1; x<g_uHeatBedMaxX-1; x++ )
		{
			for( y=0; y<g_uHeatBedMaxY; y++ )
			{
				g_HeatBedCompensation[x][y] = g_HeatBedCompensation[x+1][y];
			}
		}

		// we have one x column less now
		g_uHeatBedMaxX --;
	}

	if( g_HeatBedCompensation[g_uHeatBedMaxX-1][0] < short(X_MAX_LENGTH * XAXIS_STEPS_PER_MM) )
	{
		// we have to fill x[g_uHeatBedMaxX] with the values of x[g_uHeatBedMaxX-1]
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[g_uHeatBedMaxX-1] < X_MAX_LENGTH" ) );
		}
		g_HeatBedCompensation[g_uHeatBedMaxX][0] = g_HeatBedCompensation[g_uHeatBedMaxX-1][0] + g_nScanXStepSizeSteps;
		for( y=1; y<g_uHeatBedMaxY; y++ )
		{
			g_HeatBedCompensation[g_uHeatBedMaxX][y] = g_HeatBedCompensation[g_uHeatBedMaxX-1][y];
		}

		// we have one x column more now
		g_uHeatBedMaxX ++;
	}
	else
	{
		// there is nothing else to do here
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): x[g_uHeatBedMaxX-1] = X_MAX_LENGTH" ) );
		}
	}

	if( g_HeatBedCompensation[0][2] > 0 )
	{
		// we have to fill y[1] with the values of y[2]
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[2] > 0" ) );
		}
		g_HeatBedCompensation[0][1] = 0;
		for( x=1; x<g_uHeatBedMaxX; x++ )
		{
			g_HeatBedCompensation[x][1] = g_HeatBedCompensation[x][2];
		}
	}
	else
	{
		// we have to shift all y columns one index
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[2] = 0" ) );
		}
		for( x=0; x<g_uHeatBedMaxX; x++ )
		{
			for( y=1; y<g_uHeatBedMaxY-1; y++ )
			{
				g_HeatBedCompensation[x][y] = g_HeatBedCompensation[x][y+1];
			}
		}

		// we have one y column less now
		g_uHeatBedMaxY --;
	}

	if( g_HeatBedCompensation[0][g_uHeatBedMaxY-1] < short(Y_MAX_LENGTH * YAXIS_STEPS_PER_MM) )
	{
		// we have to fill y[g_uHeatBedMaxY] with the values of y[g_uHeatBedMaxY-1]
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[g_uHeatBedMaxY-1] < Y_MAX_LENGTH" ) );
		}
		g_HeatBedCompensation[0][g_uHeatBedMaxY] = g_HeatBedCompensation[0][g_uHeatBedMaxY-1] + g_nScanYStepSizeSteps;
		for( x=1; x<g_uHeatBedMaxX; x++ )
		{
			g_HeatBedCompensation[x][g_uHeatBedMaxY] = g_HeatBedCompensation[x][g_uHeatBedMaxY-1];
		}

		// we have one y column more now
		g_uHeatBedMaxY ++;
	}
	else
	{
		// there is nothing else to do here
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "prepareCompensationMatrix(): y[g_uHeatBedMaxY-1] = Y_MAX_LENGTH" ) );
		}
	}

	return 0;

} // prepareCompensationMatrix


char convertCompensationMatrix( void )
{
	long	nSum;
	short	x;
	short	y;


	g_offsetHeatBedCompensation = -32000;
	for( x=1; x<g_uHeatBedMaxX-1; x++ )
	{
		for( y=1; y<g_uHeatBedMaxY-1; y++ )
		{
			// we calculate the average of each rectangle m[x][y], m[x+1][y], m[x][y+1], m[x+1][y+1] and store it to m[x][y]
			nSum = g_HeatBedCompensation[x][y] +
				   g_HeatBedCompensation[x+1][y] +
				   g_HeatBedCompensation[x][y+1] + 
				   g_HeatBedCompensation[x+1][y+1];
			g_HeatBedCompensation[x][y] = short(nSum / 4);

			if( x>0 && y>0 && g_HeatBedCompensation[x][y] > g_offsetHeatBedCompensation )
			{
				g_offsetHeatBedCompensation = g_HeatBedCompensation[x][y];
			}
		}
	}
	return 0;

} // convertCompensationMatrix


char saveCompensationMatrix( void )
{
	unsigned int	uOffset;
	short			uTemp;
	short			uMax = -32000;
	short			x;
	short			y;


	if( g_HeatBedCompensation[0][0] && g_uHeatBedMaxX && g_uHeatBedMaxY )
	{
		// we have valid compensation values
		// write the current version
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeByte24C256( 0x50, EEPROM_OFFSET_VERSION, COMPENSATION_VERSION );
		
		// write the current x dimension
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_X, g_uHeatBedMaxX );

		// write the current y dimension
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_Y, g_uHeatBedMaxY );

		uOffset = EEPROM_OFFSET_Z_START;
		for( x=0; x<g_uHeatBedMaxX; x++ )
		{
			for( y=0; y<g_uHeatBedMaxY; y++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				HAL::delayMilliseconds( EEPROM_DELAY );
				uTemp = g_HeatBedCompensation[x][y];
				writeWord24C256( 0x50, uOffset, uTemp );
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
		// we do not have valid compensation values - clear the EEPROM data
		// write the current version
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeByte24C256( 0x50, EEPROM_OFFSET_VERSION, 0 );
		
		// write the current x dimension
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_X, 0 );

		// write the current y dimension
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_Y, 0 );

		uOffset = EEPROM_OFFSET_Z_START;
		uTemp	= 0;
		for( x=0; x<HEAT_BED_COMPENSATION_X; x++ )
		{
			for( y=0; y<HEAT_BED_COMPENSATION_Y; y++ )
			{
#if FEATURE_WATCHDOG
				HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

				HAL::delayMilliseconds( EEPROM_DELAY );
				writeWord24C256( 0x50, uOffset, uTemp );
				uOffset += 2;

				if( x>0 && y>0 )
				{
					// the first column and row is used for version and position information
					if( uTemp > uMax )	uMax = uTemp;
				}
			}
		}
	}

	g_offsetHeatBedCompensation = uMax;
	CalculateAllowedZStepsAfterEndStop();
	return 0;

} // saveCompensationMatrix


char restoreCompensationMatrix( void )
{
	unsigned char	uVersion;
	unsigned short	uDimensionX;
	unsigned short	uDimensionY;
	unsigned int	uOffset;
	short			uTemp;
	short			uMax = -32000;
	short			x;
	short			y;


	// check the stored version
	HAL::delayMilliseconds( EEPROM_DELAY );
	uVersion = readByte24C256( 0x50, EEPROM_OFFSET_VERSION );

	if( uVersion != COMPENSATION_VERSION )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "restoreCompensationMatrix(): invalid version detected: " ), (int)uVersion );
			Com::printF( PSTR( " (expected: " ), COMPENSATION_VERSION );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	// check the stored x dimension
	HAL::delayMilliseconds( EEPROM_DELAY );
	uDimensionX = readWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_X );

	if( uDimensionX > HEAT_BED_COMPENSATION_X )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "restoreCompensationMatrix(): invalid x dimension detected: " ), (int)uDimensionX );
			Com::printF( PSTR( " (max expected: " ), HEAT_BED_COMPENSATION_X );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	// check the stored y dimension
	HAL::delayMilliseconds( EEPROM_DELAY );
	uDimensionY = readWord24C256( 0x50, EEPROM_OFFSET_DIMENSION_Y );

	if( uDimensionY > HEAT_BED_COMPENSATION_Y )
	{
		if( Printer::debugErrors() )
		{
			Com::printF( PSTR( "restoreCompensationMatrix(): invalid y dimension detected: " ), (int)uDimensionY );
			Com::printF( PSTR( " (max expected: " ), HEAT_BED_COMPENSATION_Y );
			Com::printFLN( PSTR( ")" ) );
		}
		return -1;
	}

	g_uHeatBedMaxX = (unsigned char)uDimensionX;
	g_uHeatBedMaxY = (unsigned char)uDimensionY;

	// read out the actual compensation values
	uOffset = EEPROM_OFFSET_Z_START;
	for( x=0; x<g_uHeatBedMaxX; x++ )
	{
		for( y=0; y<g_uHeatBedMaxY; y++ )
		{
#if FEATURE_WATCHDOG
			HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

			HAL::delayMilliseconds( EEPROM_DELAY );
			uTemp = readWord24C256( 0x50, uOffset );
			g_HeatBedCompensation[x][y] = uTemp;
			uOffset += 2;

			if( x>0 && y>0 )
			{
				// the first column and row is used for version and position information
				if( uTemp > uMax )	uMax = uTemp;
			}
		}
	}

	g_offsetHeatBedCompensation = uMax;
	CalculateAllowedZStepsAfterEndStop();
	return 0;

} // restoreCompensationMatrix


void clearCompensationMatrix( void )
{
	// clear all fields of the compensation matrix
	initCompensationMatrix();

	// store the cleared compensation matrix to the EEPROM
	saveCompensationMatrix();

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
#if REMEMBER_PRESSURE
		short	i;
		short	j;


		Com::printFLN( PSTR( "Pressure matrix:" ) );
		Com::printFLN( PSTR( "front left ... front right" ) );
		Com::printFLN( PSTR( "...        ...         ..." ) );
		Com::printFLN( PSTR( "back left  ...  back right" ) );
		for( i=0; i<HEAT_BED_COMPENSATION_Y; i++ )
		{
			for( j=0; j<HEAT_BED_COMPENSATION_X; j++ )
			{
				Com::printF( PSTR( ";" ), g_HeatBedPressure[j][i] );
			}
			Com::printFLN( PSTR( " " ) );
		}
#endif // REMEMBER_PRESSURE
	}

	return;

} // outputPressureMatrix
#endif // FEATURE_Z_COMPENSATION


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

	for( i=0; i<uMax; i++ )
	{
		HAL::delayMilliseconds( EEPROM_DELAY );
		writeByte24C256( 0x50, i, 0 );

		if( Printer::debugInfo() )
		{
			uTemp = i / 1000;
			if( uTemp != uLast )
			{
				Com::printF( PSTR( "clearExternalEEPROM(): " ), (int)i );
				Com::printFLN( PSTR( " / " ), (int)uMax );
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
	HAL::delayMilliseconds( EEPROM_DELAY );	// 1 is too little here
	writeByte24C256( addressI2C, addressEEPROM+1, Temp );
	return;

} // writeWord24C256


unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM )
{
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
	HAL::delayMilliseconds( EEPROM_DELAY ); // 1 is too little here
	Temp = readByte24C256( addressI2C, addressEEPROM+1 );

	return data + Temp;

} // readWord24C256


void loopRF1000( void )
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

#if FEATURE_Z_COMPENSATION
	if( g_nHeatBedScanStatus )
	{
		scanHeatBed();
	}
#endif // FEATURE_Z_COMPENSATION

#if FEATURE_PAUSE_PRINTING
	if( g_uPauseTime )
	{
		if( !g_pauseBeepDone )
		{
			BEEP_PAUSE
			g_pauseBeepDone = 1;
		}

		if( g_pausePrint )
		{
			if( (uTime - g_uPauseTime) > EXTRUDER_CURRENT_PAUSE_DELAY )
			{
				// we have paused a few moments ago - reduce the current of the extruder motor in order to avoid unwanted heating of the filament for use cases where the printing is paused for several minutes
/*				Com::printF( PSTR( "loopRF1000(): PauseTime = " ), g_uPauseTime );
				Com::printF( PSTR( ", Time = " ), uTime );
				Com::printFLN( PSTR( ", Diff = " ), uTime - g_uPauseTime );
*/
				setExtruderCurrent( EXTRUDER_CURRENT_PAUSED );
				g_uPauseTime = 0;
			}
		}
		else
		{
			// we are not paused any more
			g_uPauseTime = 0;
		}
	}
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_EMERGENCY_PAUSE
	if( (uTime - g_uLastPressureTime) > EMERGENCY_PAUSE_INTERVAL )
	{
		g_uLastPressureTime = uTime;

		if( g_pausePrint == 0 && PrintLine::linesCount > 5 )
		{
			// this check shall be done only during the printing (for example, it shall not be done in case filament is extruded manually)
			g_nPressureSum	  += readStrainGauge( SCAN_STRAIN_GAUGE );
			g_nPressureChecks += 1;

			if( g_nPressureChecks == EMERGENCY_PAUSE_CHECKS )
			{
				nPressure		 = g_nPressureSum / g_nPressureChecks;

//				Com::printF( PSTR( "loopRF1000(): average = " ), nPressure );
//				Com::printFLN( PSTR( " / " ), g_nPressureChecks );

				g_nPressureSum	  = 0;
				g_nPressureChecks = 0;

				if( (nPressure < EMERGENCY_PAUSE_DIGITS_MIN) ||
					(nPressure > EMERGENCY_PAUSE_DIGITS_MAX) )
				{
					// the pressure is outside the allowed range, we must perform the emergency pause
					if( Printer::debugErrors() )
					{
						Com::printF( PSTR( "loopRF1000(): emergency pause: " ), nPressure );
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
#endif // FEATURE_EMERGENCY_PAUSE

#if FEATURE_EMERGENCY_Z_STOP
	if( (uTime - g_uLastZPressureTime) > EMERGENCY_Z_STOP_INTERVAL )
	{
		g_uLastZPressureTime = uTime;

		if( g_nDirectionZ && !g_nDirectionE )
		{
			// this check shall be done only when there is some moving into z-direction in progress and the extruder is not doing anything
			g_nZPressureSum	   += readStrainGauge( SCAN_STRAIN_GAUGE );
			g_nZPressureChecks += 1;

			if( g_nZPressureChecks == EMERGENCY_Z_STOP_CHECKS )
			{
				nPressure		 = g_nZPressureSum / g_nZPressureChecks;

//				Com::printF( PSTR( "loopRF1000(): average = " ), nPressure );
//				Com::printFLN( PSTR( " / " ), g_nZPressureChecks );

				g_nZPressureSum	   = 0;
				g_nZPressureChecks = 0;

				if( (nPressure < EMERGENCY_Z_STOP_DIGITS_MIN) ||
					(nPressure > EMERGENCY_Z_STOP_DIGITS_MAX) )
				{
					// the pressure is outside the allowed range, we must perform the emergency z-stop
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "loopRF1000(): emergency z-stop: " ), nPressure );
					}

					// block any further movement into the z-direction
					g_nBlockZ	  = 1;
					g_nDirectionZ = 0;
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
				g_uStopTime = 0;

				// disable all heaters
				Extruder::setHeatedBedTemperature(0,false);
				Extruder::setTemperatureForExtruder(0,0,false);

#if FEATURE_OUTPUT_PRINTED_OBJECT
				// output the object
				outputObject();
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

#if FAN_PIN>-1
				// disable the fan
				Commands::setFanSpeed(0,false);
#endif // FAN_PIN>-1

				// disable all steppers
				Printer::setAllSteppersDisabled();
				Printer::disableXStepper();
				Printer::disableYStepper();
				Printer::disableZStepper();
				Extruder::disableCurrentExtruderMotor();
			}
		}
	}
	
#if FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR
	if( Printer::isAnyTempsensorDefect() && sd.sdmode && PrintLine::linesCount )
	{
		// we are printing from the SD card and a temperature sensor got defect - abort the current printing
		Com::printFLN( PSTR( "loopRF1000(): aborting print because of a temperature sensor defect" ) );

		sd.abortPrint();
	}
#endif // FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR

	if( (uTime - g_lastTime) > LOOP_INTERVAL )
	{
		// it is time for another processing
		g_lastTime = uTime;

#if FEATURE_Z_COMPENSATION
		if( g_debugLevel && Printer::debugInfo() )
		{
#if DEBUG_Z_COMPENSATION
			switch( g_debugLevel )
			{
				case 1:
				{
					Com::printF( PSTR( "tcZ;" ), Printer::targetCompensationZ );
					Com::printF( PSTR( ";ccZ;" ), Printer::currentCompensationZ );
					break;
				}
				case 2:
				{
					Com::printF( PSTR( "tpsZ;" ), Printer::targetPositionStepsZ );
					Com::printF( PSTR( ";cpsZ;" ), Printer::currentPositionStepsZ );
					break;
				}
				case 3:
				{
					Com::printF( PSTR( "asaeZ;" ), Printer::allowedZStepsAfterEndstop );
					Com::printF( PSTR( ";csaeZ;" ), Printer::currentZStepsAfterEndstop );
					break;
				}
				case 4:
				{
					Com::printF( PSTR( "tpsX;" ), Printer::targetPositionStepsX );
					Com::printF( PSTR( ";cpsX;" ), Printer::currentPositionStepsX );
					Com::printF( PSTR( ";tpsY;" ), Printer::targetPositionStepsY );
					Com::printF( PSTR( ";cpsY;" ), Printer::currentPositionStepsY );
					Com::printF( PSTR( ";tpsZ;" ), Printer::targetPositionStepsZ );
					Com::printF( PSTR( ";cpsZ;" ), Printer::currentPositionStepsZ );
					Com::printF( PSTR( ";tpsE;" ), Printer::targetPositionStepsE );
					Com::printF( PSTR( ";cpsE;" ), Printer::currentPositionStepsE );
					break;
				}
				default:
				{
					Com::printF( PSTR( "unsupported debug level: " ), g_debugLevel );
					break;
				}
			}
			Com::printFLN( PSTR( " " ) );

/*			Com::printF( PSTR( "" ), Printer::nonCompensatedPositionStepsX );
			Com::printF( PSTR( ";" ), Printer::nonCompensatedPositionStepsY );
			Com::printF( PSTR( ";" ), Printer::nonCompensatedPositionStepsZ );
			Com::printF( PSTR( ";t Z;" ), Printer::targetCompensationZ );
			Com::printF( PSTR( ";c Z;" ), Printer::currentCompensationZ );
			Com::printF( PSTR( ";r C;" ), g_recalculatedCompensation );
			Com::printF( PSTR( ";tPS Z;" ), Printer::targetPositionStepsZ );
			Com::printF( PSTR( ";cPS Z;" ), Printer::currentPositionStepsZ );
			Com::printF( PSTR( ";a ZsaE;" ), Printer::allowedZStepsAfterEndstop );
			Com::printF( PSTR( ";c ZsaE;" ), Printer::currentZStepsAfterEndstop );
			Com::printFLN( PSTR( ";" ) );
*/
#endif // DEBUG_Z_COMPENSATION

			// NOTE: there is no need to turn off the z compensation automatically
/*			if( Printer::nonCompensatedPositionStepsZ > g_maxZCompensationSteps &&
				Printer::targetCompensationZ == Printer::currentCompensationZ )
			{
				// turn off the z compensation in case we are far away from the surface
				Printer::doZCompensation = 0;

				// NOTE: at this place we have to continue with the constant offset in z direction - do not set the target correction to 0
			    //Printer::targetCompensationZ = 0;
				Com::printFLN( PSTR( "loopRF1000(): The z compensation has been disabled (z)." ) );
			}
*/		}
#endif // FEATURE_Z_COMPENSATION

	}

	nEntered --;
	return;

} // loopRF1000


#if FEATURE_Z_COMPENSATION
void startHeatBedScan( void )
{
	if( g_nHeatBedScanStatus )
	{
		// abort the heat bed scan
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "startHeatBedScan(): the scan has been cancelled" ) );
		}
		g_abortScan = 1;
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
			if( Printer::doZCompensation )
			{
				if( Printer::debugInfo() )
				{
					Com::printFLN( PSTR( "startHeatBedScan(): the z compensation has been disabled" ) );
				}
				Printer::doZCompensation = 0;
			}
		}
	}

	return;

} // startHeatBedScan
#endif // FEATURE_Z_COMPENSATION

#if FEATURE_OUTPUT_PRINTED_OBJECT
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

#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	PrintLine::moveRelativeDistanceInSteps( 0,
											0,
											g_nOutputOffsetZ * ZAXIS_STEPS_PER_MM,
											0,
											Printer::homingFeedrate[Z_AXIS],
											true,
											ALWAYS_CHECK_ENDSTOPS);

#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	PrintLine::moveRelativeDistanceInSteps( g_nOutputOffsetX * XAXIS_STEPS_PER_MM,
											g_nOutputOffsetY * YAXIS_STEPS_PER_MM,
											0,
											0,
											Printer::homingFeedrate[X_AXIS],
											true,
											ALWAYS_CHECK_ENDSTOPS);

	// disable all steppers
	Printer::setAllSteppersDisabled();
	Printer::disableXStepper();
	Printer::disableYStepper();
	Printer::disableZStepper();
	Extruder::disableCurrentExtruderMotor();

} // outputObject
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

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

#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

	Printer::moveToReal( g_nParkPositionX, g_nParkPositionY, g_nParkPositionZ, IGNORE_COORDINATE, Printer::homingFeedrate[0]);

} // parkPrinter
#endif // FEATURE_PARK

void pausePrint( void )
{
#if FEATURE_PAUSE_PRINTING
	long	Temp;


	if( g_pausePrint == 0 )
	{
		// the printing is not paused at the moment
		if( PrintLine::linesCount )
		{
			g_pausePrint = 1;

			// wait until the current move is completed
			while( !g_printingPaused )
			{
				HAL::delayMilliseconds( 1 );
				Commands::checkForPeriodicalActions();
			}

#if EXTRUDER_CURRENT_PAUSE_DELAY
			// remember the pause time only in case we shall lower the extruder current
			g_uPauseTime	= HAL::timeInMilliseconds();
			g_pauseBeepDone	= 0;
#endif // EXTRUDER_CURRENT_PAUSE_DELAY

			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "pausePrint(): the printing has been paused" ) );
			}
		    UI_STATUS(UI_TEXT_PAUSED);

			g_nContinueStepsX = 0;
			g_nContinueStepsY = 0;
			g_nContinueStepsZ = 0;

			if( g_nPauseStepsExtruder )
			{
				Printer::targetPositionStepsE += g_nPauseStepsExtruder;
				g_nContinueStepsExtruder	  =  g_nPauseStepsExtruder;
			}
		}
		else
		{
			if( Printer::debugErrors() )
			{
				Com::printFLN( PSTR( "pausePrint(): pause is not available at the moment because nothing is printed" ) );
			}
		}
		return;
	}

	if( g_pausePrint == 1 )
	{
		// in case the print is paused already, we move the printer head to the pause position
		g_pausePrint = 2;

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): moving to the pause position" ) );
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if( g_nPauseStepsZ )
		{
			Temp = g_nPauseStepsZ + Printer::targetPositionStepsZ;

#if FEATURE_Z_COMPENSATION
			Temp += Printer::nonCompensatedPositionStepsZ;
			Temp += Printer::currentCompensationZ;
#endif // FEATURE_Z_COMPENSATION

			if( Temp > (Z_MAX_LENGTH * ZAXIS_STEPS_PER_MM - ZAXIS_STEPS_PER_MM) )
			{
				// do not allow to drive the heat bed into the bottom
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "pausePrint(): moving z aborted" ) );
				}
			}
			else
			{
				Printer::targetPositionStepsZ += g_nPauseStepsZ;
				g_nContinueStepsZ			  =  -g_nPauseStepsZ;

				CalculateAllowedZStepsAfterEndStop();
			}
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if( g_nPauseStepsX )
		{
			Temp = g_nPauseStepsX;

#if FEATURE_Z_COMPENSATION
			Temp += Printer::nonCompensatedPositionStepsX;
#endif // FEATURE_Z_COMPENSATION

			if( g_nPauseStepsX < 0 &&
				(Temp < (XAXIS_STEPS_PER_MM *5)) )
			{
				// do not allow to drive the heat bed into the left border
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "pausePrint(): moving x aborted" ) );
				}
			}
			else if( g_nPauseStepsX > 0 &&
					 (Temp > (X_MAX_LENGTH * XAXIS_STEPS_PER_MM - XAXIS_STEPS_PER_MM *5)) )
			{
				// do not allow to drive the heat bed into the right border
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "pausePrint(): moving x aborted" ) );
				}
			}
			else
			{
				Printer::targetPositionStepsX += g_nPauseStepsX;
				g_nContinueStepsX			  =  -g_nPauseStepsX;
			}
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if( g_nPauseStepsY )
		{
			Temp = g_nPauseStepsY;

#if FEATURE_Z_COMPENSATION
			Temp += Printer::nonCompensatedPositionStepsY;
#endif // FEATURE_Z_COMPENSATION

			if( g_nPauseStepsY < 0 &&
				(Temp < (YAXIS_STEPS_PER_MM *5)) )
			{
				// do not allow to drive the heat bed into the front border
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "pausePrint(): moving y aborted" ) );
				}
			}
			else if( g_nPauseStepsY > 0 &&
					 (Temp > (Y_MAX_LENGTH * YAXIS_STEPS_PER_MM - YAXIS_STEPS_PER_MM *5)) )
			{
				// do not allow to drive the heat bed into the back border
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "pausePrint(): moving y aborted" ) );
				}
			}
			else
			{
				Printer::targetPositionStepsY += g_nPauseStepsY;
				g_nContinueStepsY			  =  -g_nPauseStepsY;
			}
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		// wait until the pause position has been reached
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): waiting for the pause position" ) );
		}

		while( (Printer::targetPositionStepsX != Printer::currentPositionStepsX) ||
			   (Printer::targetPositionStepsY != Printer::currentPositionStepsY) ||
			   (Printer::targetPositionStepsZ != Printer::currentPositionStepsZ) ||
			   (Printer::targetPositionStepsE != Printer::currentPositionStepsE) )
		{
			HAL::delayMilliseconds( 1 );
			loopRF1000();
			Commands::checkForPeriodicalActions();

			// NOTE: do not run runStandardTasks() within this loop
			//runStandardTasks();
		}

#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "pausePrint(): the pause position has been reached" ) );
		}
		return;
	}

#if FEATURE_EMERGENCY_STOP_VIA_PAUSE
	if( g_pausePrint == 2 )
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

#else
	// there is nothing to do in case the pause printing feature is disabled
#endif // FEATURE_PAUSE_PRINTING

	return;

} // pausePrint


void continuePrint( void )
{
#if FEATURE_PAUSE_PRINTING
	const unsigned short	uMotorCurrents[] =  MOTOR_CURRENT;


	if( g_pausePrint )
	{
		BEEP_CONTINUE

		if( g_pausePrint == 2 )
		{
			// move the the continue position
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "continuePrint(): moving to the continue position" ) );
			}

			setExtruderCurrent( uMotorCurrents[E_AXIS] );

			if( g_nContinueStepsY )			Printer::targetPositionStepsY += g_nContinueStepsY;
			if( g_nContinueStepsX )			Printer::targetPositionStepsX += g_nContinueStepsX;
			if( g_nContinueStepsZ )			Printer::targetPositionStepsZ += g_nContinueStepsZ;
			if( g_nContinueStepsExtruder )	Printer::targetPositionStepsE -= g_nContinueStepsExtruder;

			CalculateAllowedZStepsAfterEndStop();

			// wait until the continue position has been reached
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "continuePrint(): waiting for the continue position" ) );
			}

			while( (Printer::targetPositionStepsX != Printer::currentPositionStepsX) ||
				   (Printer::targetPositionStepsY != Printer::currentPositionStepsY) ||
				   (Printer::targetPositionStepsZ != Printer::currentPositionStepsZ) ||
				   (Printer::targetPositionStepsE != Printer::currentPositionStepsE) )
			{
				HAL::delayMilliseconds( 1 );
				loopRF1000();
				Commands::checkForPeriodicalActions();

				// NOTE: do not run runStandardTasks() within this loop
				//runStandardTasks();
			}
		}
		else if( g_pausePrint == 1 )
		{
			setExtruderCurrent( uMotorCurrents[E_AXIS] );

			if( g_nContinueStepsExtruder )	Printer::targetPositionStepsE -= g_nContinueStepsExtruder;

			// wait until the continue position has been reached
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "continuePrint(): waiting for the continue position" ) );
			}

			while( Printer::targetPositionStepsE != Printer::currentPositionStepsE )
			{
				HAL::delayMilliseconds( 1 );
				loopRF1000();
				Commands::checkForPeriodicalActions();

				// NOTE: do not run runStandardTasks() within this loop
				//runStandardTasks();
			}
		}

		// wait until the next move is started
		g_pausePrint = 0;
		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "continuePrint(): waiting for the next move" ) );
		}
		while( g_printingPaused )
		{
			if( !PrintLine::linesCount )
			{
				// the printing won't continue in case there is nothing else to do
				break;
			}
			HAL::delayMilliseconds( 1 );
			loopRF1000();
			Commands::checkForPeriodicalActions();

			// NOTE: do not run runStandardTasks() within this loop
			//runStandardTasks();
		}

		if( Printer::debugInfo() )
		{
			Com::printFLN( PSTR( "continuePrint(): the printing has been continued" ) );
		}
		UI_STATUS(UI_TEXT_PRINTING);
	}
	else
	{
		if( Printer::debugErrors() )
		{
			Com::printFLN( PSTR( "continuePrint(): continue is not available at the moment" ) );
		}
	}
#endif // FEATURE_PAUSE_PRINTING

	return;

} // continuePrint


void setExtruderCurrent( unsigned short level )
{
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
#if FEATURE_Z_COMPENSATION
			case 3000: // M3000 - turn off the Z compensation
			{
				Com::printFLN( PSTR( "M3000: disabling z compensation" ) );
				PrintLine::queueTask( TASK_DISABLE_Z_COMPENSATION );
				break;
			}
			case 3001: // M3001 - turn on the Z compensation
			{
				if( g_HeatBedCompensation[0][0] == COMPENSATION_VERSION )
				{
					// enable the z compensation only in case we have valid compensation values
					Com::printFLN( PSTR( "M3001: enabling z compensation" ) );
					PrintLine::queueTask( TASK_ENABLE_Z_COMPENSATION );
				}
				else
				{
					Com::printF( PSTR( "M3001: the z compensation can not be enabled because the compensation matrix is not valid ( " ), g_HeatBedCompensation[0][0] );
					Com::printF( PSTR( " / " ), COMPENSATION_VERSION );
					Com::printFLN( PSTR( " )" ) );
				}
				break;
			}
			case 3002: // M3002 - configure the no compensation steps
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < (ZAXIS_STEPS_PER_MM /10) )	nTemp = ZAXIS_STEPS_PER_MM /10;
					if( nTemp > (ZAXIS_STEPS_PER_MM *10) )	nTemp = ZAXIS_STEPS_PER_MM *10;

					if( nTemp > g_maxZCompensationSteps )
					{
						// the no compensation steps can not be bigger than the maximal compensation steps
						nTemp = g_maxZCompensationSteps;
					}

					g_noZCompensationSteps = nTemp;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3002: new no compensation steps: " ), g_noZCompensationSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}

					g_diffZCompensationSteps = g_maxZCompensationSteps - g_noZCompensationSteps;
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3002: invalid syntax" ) );
					}
				}
				break;
			}
			case 3003: // M3003 - configure the max compensation steps
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < (ZAXIS_STEPS_PER_MM /10) )	nTemp = ZAXIS_STEPS_PER_MM /10;
					if( nTemp > (ZAXIS_STEPS_PER_MM *10) )	nTemp = ZAXIS_STEPS_PER_MM *10;

					if( nTemp < g_noZCompensationSteps )
					{
						// the maximal compensation steps can not be smaller than the no compensation steps
						nTemp = g_noZCompensationSteps;
					}

					g_maxZCompensationSteps = nTemp;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3003: new max compensation steps: " ), g_maxZCompensationSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}

					g_diffZCompensationSteps = g_maxZCompensationSteps - g_noZCompensationSteps;
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3003: invalid syntax" ) );
					}
				}
				break;
			}
#endif // FEATURE_Z_COMPENSATION

			case 3004: // M3004 - configure the manual compensation steps
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < -Z_COMPENSATION_MAX_STEPS )	nTemp = -Z_COMPENSATION_MAX_STEPS;
					if( nTemp > Z_COMPENSATION_MAX_STEPS )	nTemp = Z_COMPENSATION_MAX_STEPS;

					g_manualCompensationSteps = nTemp;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3004: new manual compensation steps: " ), g_manualCompensationSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}

					CalculateAllowedZStepsAfterEndStop();
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3004: invalid syntax" ) );
					}
				}
				break;
			}
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3005: invalid syntax" ) );
					}
				}
				break;
			}

#if FEATURE_Z_COMPENSATION
			case 3010: // M3010 - start/abort the heat bed scan
			{
				startHeatBedScan();
				break;
			}
			case 3011: // M3011 - clear the compensation matrix from the EEPROM
			{
				clearCompensationMatrix();
				break;
			}
			case 3012: // M3012 - restore the default scan parameters
			{
				restoreDefaultScanParameters();
				break;
			}
			case 3013: // M3013 - output the current compensation matrix
			{
				outputCompensationMatrix();
				break;
			}
			case 3020: // M3020 - configure the x start position for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 5 )						nTemp = 5;
					if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

					g_nScanXStartSteps = nTemp * XAXIS_STEPS_PER_MM;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3020: new x start position: " ), nTemp );
						Com::printF( PSTR( " [mm], " ), (int)g_nScanXStartSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3020: invalid syntax" ) );
					}
				}
				break;
			}
			case 3021: // M3021 - configure the y start position for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 5 )						nTemp = 5;
					if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

					g_nScanYStartSteps = nTemp * YAXIS_STEPS_PER_MM;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3021: new y start position: " ), nTemp );
						Com::printF( PSTR( " [mm], " ), (int)g_nScanYStartSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3021: invalid syntax" ) );
					}
				}
				break;
			}
			case 3022: // M3022 - configure the x step size for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < SCAN_X_STEP_SIZE_MM )	nTemp = SCAN_X_STEP_SIZE_MM;
					if( nTemp > 100 )					nTemp = 100;

					g_nScanXStepSizeSteps = nTemp * XAXIS_STEPS_PER_MM;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3022: new x step size: " ), (int)nTemp );
						Com::printF( PSTR( " [mm], " ), (int)g_nScanXStepSizeSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3022: invalid syntax" ) );
					}
				}
				break;
			}
			case 3023: // M3023 - configure the y step size for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < SCAN_Y_STEP_SIZE_MM )	nTemp = SCAN_Y_STEP_SIZE_MM;
					if( nTemp > 100 )					nTemp = 100;

					g_nScanYStepSizeSteps = nTemp * YAXIS_STEPS_PER_MM;
					if( Printer::debugInfo() )
					{
						Com::printF( PSTR( "M3023: new y step size: " ), (int)nTemp );
						Com::printF( PSTR( " [mm], " ), (int)g_nScanYStepSizeSteps );
						Com::printFLN( PSTR( " [steps]" ) );
					}
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3023: invalid syntax" ) );
					}
				}
				break;
			}
			case 3024: // M3024 - configure the x end position for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 5 )						nTemp = 5;
					if( nTemp > (X_MAX_LENGTH -5) )		nTemp = X_MAX_LENGTH -5;

					g_nScanXEndSteps = nTemp * XAXIS_STEPS_PER_MM;
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3024: invalid syntax" ) );
					}
				}
				break;
			}
			case 3025: // M3025 - configure the y end position for the heat bed scan
			{
				if( pCommand->hasS() )
				{
					// test and take over the specified value
					nTemp = pCommand->S;
					if( nTemp < 5 )						nTemp = 5;
					if( nTemp > (Y_MAX_LENGTH -5 ) )	nTemp = Y_MAX_LENGTH -5;

					g_nScanYEndSteps = nTemp * YAXIS_STEPS_PER_MM;
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3025: invalid syntax" ) );
					}
				}
				break;
			}
			case 3030: // M3030 - configure the fast step size for moving of the heat bed up during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3030: invalid syntax" ) );
					}
				}
				break;
			}
			case 3031: // M3031 - configure the slow step size for moving of the heat bed up during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3031: invalid syntax" ) );
					}
				}
				break;
			}
			case 3032: // M3032 - configure the fast step size for moving of the heat bed down during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3032: invalid syntax" ) );
					}
				}
				break;
			}
			case 3033: // M3033 - configure the slow step size for moving of the heat bed down during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3033: invalid syntax" ) );
					}
				}
				break;
			}
			case 3040: // M3040 - configure the delay (in ms) between two fast movements during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3040: invalid syntax" ) );
					}
				}
				break;
			}
			case 3041: // M3041 - configure the delay (in ms) between two slow movements during the heat bed scan
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3041: invalid syntax" ) );
					}
				}
				break;
			}
			case 3042: // M3042 - configure the delay (in ms) between reaching of a new x/y position and the test of the idle pressure
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3042: invalid syntax" ) );
					}
				}
				break;
			}
			case 3050: // M3050 - configure the contact pressure delta (in digits)
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3050: invalid syntax" ) );
					}
				}
				break;
			}
			case 3051: // M3051 - configure the retry pressure delta (in digits)
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3051: invalid syntax" ) );
					}
				}
				break;
			}
			case 3052: // M3052 - configure the idle pressure tolerance (in digits)
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3052: invalid syntax" ) );
					}
				}
				break;
			}
			case 3053: // M3053 - configure the number of A/D converter reads per pressure measurement
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3053: invalid syntax" ) );
					}
				}
				break;
			}
			case 3054: // M3054 - configure the delay (in ms) between two A/D converter reads
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3054: invalid syntax" ) );
					}
				}
				break;
			}
			case 3055: // M3055 - configure the pressure tolerance (in digits) per pressure measurement
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3055: invalid syntax" ) );
					}
				}
				break;
			}
#endif // FEATURE_Z_COMPENSATION

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
						PrintLine::queueTask( TASK_PAUSE_PRINT_1 );
					}
					if( nTemp == 2 )
					{
						// we shall pause the printing and we shall move away
						PrintLine::queueTask( TASK_PAUSE_PRINT_2 );
					}
				}
				else
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3070: invalid syntax" ) );
					}
				}

				break;
			}
#endif // FEATURE_PAUSE_PRINTING

#if FEATURE_OUTPUT_PRINTED_OBJECT
			case 3079: // M3079 - output the printed object
			{
				outputObject();
				break;
			}
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3100: invalid syntax" ) );
					}
				}
				break;
			}
			case 3101: // M3101 - configure the number of manual extruder steps after the "Extruder up" or "Extruder down" button has been pressed
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3101: invalid syntax" ) );
					}
				}
				break;
			}
#endif // FEATURE_EXTENDED_BUTTONS

#if FEATURE_PAUSE_PRINTING
			case 3102: // M3102 - configure the offset in x, y and z direction which shall be applied in case the "Pause" button has been pressed
			{
				if( pCommand->hasNoXYZ() && !pCommand->hasE() )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3102: invalid syntax" ) );
					}
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < -(XAXIS_STEPS_PER_MM *10) )		nTemp = -(XAXIS_STEPS_PER_MM *10);
						if( nTemp > (XAXIS_STEPS_PER_MM *10) )		nTemp = (XAXIS_STEPS_PER_MM *10);

						g_nPauseStepsX = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new x pause offset: " ), (int)g_nPauseStepsX );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < -(YAXIS_STEPS_PER_MM *10) )		nTemp = -(YAXIS_STEPS_PER_MM *10);
						if( nTemp > (YAXIS_STEPS_PER_MM *10) )		nTemp = (YAXIS_STEPS_PER_MM *10);

						g_nPauseStepsY = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new y pause offset: " ), (int)g_nPauseStepsY );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )								nTemp = 0;
						if( nTemp > ZAXIS_STEPS_PER_MM )			nTemp = ZAXIS_STEPS_PER_MM;

						g_nPauseStepsZ = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new z pause offset: " ), (int)g_nPauseStepsZ );
							Com::printFLN( PSTR( " [steps]" ) );
						}
					}
					if( pCommand->hasE() )
					{
						// test and take over the specified value
						nTemp = pCommand->E;
						if( nTemp < 0 )								nTemp = 0;
						if( nTemp > (EXT0_STEPS_PER_MM *5) )		nTemp = EXT0_STEPS_PER_MM *5;

						g_nPauseStepsExtruder = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3102: new extruder pause offset: " ), (int)g_nPauseStepsExtruder );
							Com::printFLN( PSTR( " [steps]" ) );
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
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3103: invalid syntax" ) );
					}
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > X_MAX_LENGTH )	nTemp = X_MAX_LENGTH;

						g_nParkPositionX = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new x park position: " ), (int)g_nParkPositionX );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Y_MAX_LENGTH )	nTemp = Y_MAX_LENGTH;

						g_nParkPositionY = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new y park position: " ), (int)g_nParkPositionY );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Z_MAX_LENGTH )	nTemp = Y_MAX_LENGTH;

						g_nParkPositionZ = (short)nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3103: new z park position: " ), (int)g_nParkPositionZ );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
				}
				break;
			}
#endif // FEATURE_PARK

#if FEATURE_OUTPUT_PRINTED_OBJECT
			case 3104: // M3104 - configure the x, y and z position which shall set when the printed object is output
			{
				if( pCommand->hasNoXYZ() )
				{
					if( Printer::debugErrors() )
					{
						Com::printFLN( PSTR( "M3104: invalid syntax" ) );
					}
				}
				else
				{
					if( pCommand->hasX() )
					{
						// test and take over the specified value
						nTemp = pCommand->X;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > X_MAX_LENGTH )	nTemp = X_MAX_LENGTH;

						g_nOutputOffsetX = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3104: new x output offset: " ), (int)g_nOutputOffsetX );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasY() )
					{
						// test and take over the specified value
						nTemp = pCommand->Y;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Y_MAX_LENGTH )	nTemp = Y_MAX_LENGTH;

						g_nOutputOffsetY = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3104: new y output offset: " ), (int)g_nOutputOffsetY );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
					if( pCommand->hasZ() )
					{
						// test and take over the specified value
						nTemp = pCommand->Z;
						if( nTemp < 0 )				nTemp = 0;
						if( nTemp > Z_MAX_LENGTH )	nTemp = Z_MAX_LENGTH;

						g_nOutputOffsetZ = nTemp;
						if( Printer::debugInfo() )
						{
							Com::printF( PSTR( "M3104: new z output offset: " ), (int)g_nOutputOffsetZ );
							Com::printFLN( PSTR( " [mm]" ) );
						}
					}
				}
				break;
			}
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

#if FEATURE_CONTROLLER == 4 || FEATURE_CONTROLLER == 33
			case 3110:	// M3110 - force a status text
			{
				if( pCommand->hasS() )
				{
					// take over the specified value
					if( pCommand->S )
					{
						// ensure that the current text won't be overwritten
						Com::printFLN( PSTR( "M3110: lock" ) );
						uid.lock();
					}
					else
					{
						// allow to overwrite the current string again
						uid.unlock();
						Com::printFLN( PSTR( "M3110: unlock" ) );
					}
				}
				break;
			}
#endif // FEATURE_CONTROLLER == 4 || FEATURE_CONTROLLER == 33

			case 3200: // M3200 - reserved for test and debug
			{
/*				// simulate a temp sensor error
				Com::printFLN( PSTR( "M3200: simulating a defect temperature sensor" ) );
                Printer::flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
                reportTempsensorError();
*/
/*				if( pCommand->hasS() )
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

						case 11:
						{
							Com::printFLN( PSTR( "M3200: block" ) );
							g_nBlockZ = 1;
							break;
						}
					}
				}
*/
				break;
			}
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
		Commands::executeGCode( pCode );
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
#if FEATURE_Z_COMPENSATION
		case UI_ACTION_RF1000_SCAN_HEAT_BED:
		{
			startHeatBedScan();
			break;
		}
#endif // FEATURE_Z_COMPENSATION

#if FEATURE_EXTENDED_BUTTONS
		case UI_ACTION_RF1000_HEAT_BED_UP:
		{
#if FEATURE_ENABLE_MANUAL_Z_SAFETY
			if( (Printer::nonCompensatedPositionStepsZ + Printer::currentCompensationZ + Printer::currentPositionStepsZ - g_nManualZSteps) < ZAXIS_STEPS_PER_MM )
			{
				// do not allow to drive the heat bed into the extruder
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): heat bed up: moving aborted" ) );
				}
			}
			else
#endif // FEATURE_ENABLE_MANUAL_Z_SAFETY
			{
				// in case the user moves the z-axis manually, we also must allow the endstop violation for the rest of the firmware
				if( g_uLastHeadBedDownTime )
				{
					g_uLastHeadBedDownTime = 0;
					g_nNextZSteps = g_nManualZSteps;
				}

				if( g_uLastHeatBedUpTime )
				{
					if( (HAL::timeInMilliseconds() - g_uLastHeatBedUpTime) < 500 )
					{
						// the user keeps the button pressed - we shall move faster and faster
						g_nNextZSteps *= 2;
						if( g_nNextZSteps > MAXIMAL_MANUAL_Z_STEPS )	g_nNextZSteps = MAXIMAL_MANUAL_Z_STEPS;
					}
					else
					{
						// this is the first movement after some time
						g_nNextZSteps = g_nManualZSteps;
					}
				}
				g_uLastHeatBedUpTime = HAL::timeInMilliseconds();

				if( (long(Printer::targetPositionStepsZ) - long(g_nNextZSteps)) < -32767 )
				{
					if( Printer::targetPositionStepsZ == -32767 )
					{
						// we can not allow to move up forever
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "processButton(): heat bed up: aborted (the topmost manual position has been reached)" ) );
						}
						break;
					}

					// move the last path until the topmost manual position
					g_nNextZSteps = 32767 + Printer::targetPositionStepsZ;
				}

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): heat bed up: " ), (int)g_nNextZSteps );
					Com::printFLN( PSTR( " [steps]" ) );
				}

				Printer::enableZStepper();
				Printer::targetPositionStepsZ -= g_nNextZSteps;

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): current manual Z steps: " ), (int)Printer::targetPositionStepsZ );
					Com::printFLN( PSTR( " [steps]" ) );
				}

				CalculateAllowedZStepsAfterEndStop();
			}
			break;
		}
		case UI_ACTION_RF1000_HEAT_BED_DOWN:
		{
#if FEATURE_ENABLE_MANUAL_Z_SAFETY
			if( (Printer::nonCompensatedPositionStepsZ + Printer::currentCompensationZ + Printer::targetPositionStepsZ + g_nManualZSteps) > (Z_MAX_LENGTH * ZAXIS_STEPS_PER_MM - ZAXIS_STEPS_PER_MM) )
			{
				// do not allow to drive the heat bed into the bottom
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): heat bed down: moving aborted" ) );
				}
			}
			else
#endif // FEATURE_ENABLE_MANUAL_Z_SAFETY
			{
				// in case the user moves the z-axis manually, we also must allow the endstop violation for the rest of the firmware
				if( g_uLastHeatBedUpTime )
				{
					g_uLastHeatBedUpTime = 0;
					g_nNextZSteps = g_nManualZSteps;
				}

				if( g_uLastHeadBedDownTime )
				{
					if( (HAL::timeInMilliseconds() - g_uLastHeadBedDownTime) < 500 )
					{
						// the user keeps the button pressed - we shall move faster and faster
						g_nNextZSteps *= 2;
						if( g_nNextZSteps > MAXIMAL_MANUAL_Z_STEPS )	g_nNextZSteps = MAXIMAL_MANUAL_Z_STEPS;
					}
					else
					{
						// this is the first movement after some time
						g_nNextZSteps = g_nManualZSteps;
					}
				}
				g_uLastHeadBedDownTime = HAL::timeInMilliseconds();

				if( (long(Printer::targetPositionStepsZ) + long(g_nNextZSteps)) > 32767 )
				{
					if( Printer::targetPositionStepsZ == 32767 )
					{
						// we can not allow to move down forever
						if( Printer::debugInfo() )
						{
							Com::printFLN( PSTR( "processButton(): heat bed down: moving aborted (the bottommost manual position has been reached)" ) );
						}
						break;
					}

					// move the last path until the bottommost manual position
					g_nNextZSteps = 32767 - Printer::targetPositionStepsZ;
				}

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): heat bed down: " ), (int)g_nNextZSteps );
					Com::printFLN( PSTR( " [steps]" ) );
				}

				Printer::enableZStepper();
				Printer::targetPositionStepsZ += g_nNextZSteps;

				if( Printer::debugInfo() )
				{
					Com::printF( PSTR( "processButton(): current manual Z steps: " ), (int)Printer::targetPositionStepsZ );
					Com::printFLN( PSTR( " [steps]" ) );
				}

				CalculateAllowedZStepsAfterEndStop();
			}
			break;
		}
		case UI_ACTION_RF1000_EXTRUDER_OUTPUT:
		{
			if( Extruder::current->tempControl.targetTemperatureC < UI_SET_MIN_EXTRUDER_TEMP )
			{
				// we do not allow to move the extruder in case it is not heated up enough
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): extruder output: aborted" ) );
				}
				break;
			}

			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "processButton(): extruder output: " ), (int)g_nManualExtruderSteps );
				Com::printFLN( PSTR( " [steps]" ) );
			}

			Extruder::enable();
			Printer::targetPositionStepsE -= g_nManualExtruderSteps;

			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "processButton(): current manual E steps: " ), (int)Printer::targetPositionStepsE );
				Com::printFLN( PSTR( " [steps]" ) );
			}
			break;
		}
		case UI_ACTION_RF1000_EXTRUDER_RETRACT:
		{
			if( Extruder::current->tempControl.targetTemperatureC < UI_SET_MIN_EXTRUDER_TEMP )
			{
				// we do not allow to move the extruder in case it is not heated up enough
				if( Printer::debugErrors() )
				{
					Com::printFLN( PSTR( "processButton(): extruder retract: aborted" ) );
				}
				break;
			}

			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "processButton(): extruder retract: " ), (int)g_nManualExtruderSteps );
				Com::printFLN( PSTR( " [steps]" ) );
			}

			Extruder::enable();
			Printer::targetPositionStepsE += g_nManualExtruderSteps;

			if( Printer::debugInfo() )
			{
				Com::printF( PSTR( "processButton(): current manual E steps: " ), (int)Printer::targetPositionStepsE );
				Com::printFLN( PSTR( " [steps]" ) );
			}
			break;
		}
		case UI_ACTION_RF1000_PAUSE:
		{
			pausePrint();
			break;
		}
		case UI_ACTION_RF1000_CONTINUE:
		{
			continuePrint();
			break;
		}
#endif // FEATURE_EXTENDED_BUTTONS

#if FEATURE_Z_COMPENSATION
		case UI_ACTION_RF1000_DO_HEAT_BED_SCAN:
		{
			startHeatBedScan();
			break;
		}
#endif // FEATURE_Z_COMPENSATION

#if FEATURE_OUTPUT_PRINTED_OBJECT
		case UI_ACTION_RF1000_OUTPUT_OBJECT:
		{
			outputObject();
			break;
		}
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

#if FEATURE_PARK
		case UI_ACTION_RF1000_PARK:
		{
			parkPrinter();
			break;
		}
#endif // FEATURE_PARK

#if FEATURE_RESET_VIA_MENU
		case UI_ACTION_RF1000_RESET:
		{
			if( Printer::debugInfo() )
			{
				Com::printFLN( PSTR( "processButton(): restart" ) );
			}
			HAL::delayMilliseconds( 100 );
			Commands::emergencyStop();
			break;
		}
#endif // FEATURE_RESET_VIA_MENU
	}
	return;

} // processButton


void CalculateAllowedZStepsAfterEndStop( void )
{
	short	nTemp = 0;


	if( g_manualCompensationSteps < 0 )				nTemp -= g_manualCompensationSteps;
	if( g_offsetHeatBedCompensation < 0 )			nTemp -= g_offsetHeatBedCompensation;

#if FEATURE_EXTENDED_BUTTONS
	nTemp -= Printer::targetPositionStepsZ;
#endif // FEATURE_EXTENDED_BUTTONS

	Printer::allowedZStepsAfterEndstop = nTemp;

} // CalculateAllowedZStepsAfterEndStop


#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_LTC2600

void setMotorCurrent( uint8_t channel, unsigned short level )
{
    const uint8_t ltc_channels[] =  LTC2600_CHANNELS;
    if(channel>LTC2600_NUM_CHANNELS) return;
    uint8_t address = ltc_channels[channel];
    char i;


    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    // configure the pins
    WRITE( LTC2600_CS_PIN, HIGH );
    SET_OUTPUT( LTC2600_CS_PIN );
    WRITE( LTC2600_SCK_PIN, LOW );
    SET_OUTPUT( LTC2600_SCK_PIN );
    WRITE( LTC2600_SDI_PIN, LOW );
    SET_OUTPUT( LTC2600_SDI_PIN );

    // enable the command interface of the LTC2600
    WRITE( LTC2600_CS_PIN, LOW );

    // transfer command and address
    for( i=7; i>=0; i-- )
    {
        WRITE( LTC2600_SDI_PIN, address & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // transfer the data word
    for( i=15; i>=0; i-- )
    {
        WRITE( LTC2600_SDI_PIN, level & (0x01 << i));
        WRITE( LTC2600_SCK_PIN, 1 );
        WRITE( LTC2600_SCK_PIN, 0 );
    }

    // disable the command interface of the LTC2600 -
    // this carries out the specified command
    WRITE( LTC2600_CS_PIN, HIGH );

} // setMotorCurrent


void motorCurrentControlInit( void )
{
    const unsigned int ltc_current[] =  MOTOR_CURRENT;
    uint8_t i;
    for(i=0; i<LTC2600_NUM_CHANNELS; i++)
    {
        setMotorCurrent(i, ltc_current[i] );
    }
}

#endif // CURRENT_CONTROL_LTC2600


#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DRV8711

void drv8711Transmit( unsigned short command )
{
  char	i;

  // transfer the command (= direction, address and data)
  HAL::forbidInterrupts();
  for( i=15; i>=0; i-- )	{
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
  for( i=15; i>=12; i-- )	{
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
  char				i;
	
	
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
  const unsigned short	drv_current[] =  MOTOR_CURRENT;
  unsigned char			i;

  // configure all DRV8711
  drv8711Init();

  // set all motor currents
  for(i=0;i<DRV8711_NUM_CHANNELS;i++)
  {
    setMotorCurrent( i+1, drv_current[i] );
  }

} // motorCurrentControlInit

#endif // CURRENT_CONTROL_DRV8711


