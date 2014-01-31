// Custom UI Actions

#define UI_ACTION_RF1000_MIN_REPEATABLE			 500

#define UI_ACTION_RF1000_HEAT_BED_UP			 514
#define UI_ACTION_RF1000_HEAT_BED_DOWN			 515
#define UI_ACTION_RF1000_EXTRUDER_OUTPUT		 516
#define UI_ACTION_RF1000_EXTRUDER_RETRACT		 517

#define UI_ACTION_RF1000_MAX_REPEATABLE			 600

#define UI_ACTION_RF1000_MIN_SINGLE				1500

#define UI_ACTION_RF1000_SCAN_HEAT_BED			1512
#define UI_ACTION_RF1000_TEST_COMPENSATION		1513
#define UI_ACTION_RF1000_PAUSE					1518
#define UI_ACTION_RF1000_CONTINUE				1519
#define UI_ACTION_RF1000_DO_HEAT_BED_SCAN		1520
#define UI_ACTION_RF1000_PARK					1521
#define UI_ACTION_RF1000_RESET					1522
#define UI_ACTION_RF1000_OUTPUT_OBJECT			1523

#define UI_ACTION_RF1000_MAX_SINGLE				1600

/* Custom M Codes

- M3000 - turn off the Z compensation
- M3001 - turn on the Z compensation
- M3002 - configure the no compensation steps
- M3003 - configure the max compensation steps
- M3004 - configure the manual compensation steps
- M3005 - enable custom debug outputs

- M3010 - start/abort the heat bed scan
- M3011 - clear the compensation matrix from the EEPROM
- M3012 - restore the default scan parameters
- M3013 - output the current compensation matrix

- M3020 - configure the x start position for the heat bed scan
- M3021 - configure the y start position for the heat bed scan
- M3022 - configure the x step size for the heat bed scan
- M3023 - configure the y step size for the heat bed scan
- M3024 - configure the x end position for the heat bed scan
- M3025 - configure the y end position for the heat bed scan

- M3030 - configure the fast step size for moving of the heat bed up during the heat bed scan
- M3031 - configure the slow step size for moving of the heat bed up during the heat bed scan
- M3032 - configure the fast step size for moving of the heat bed down during the heat bed scan
- M3033 - configure the slow step size for moving of the heat bed down during the heat bed scan

- M3040 - configure the delay (in ms) between two fast movements during the heat bed scan
- M3041 - configure the delay (in ms) between two slow movements during the heat bed scan
- M3042 - configure the delay (in ms) between reaching of a new x/y position and the test of the idle pressure

- M3050 - configure the contact pressure delta (in digits)
- M3051 - configure the retry pressure delta (in digits)
- M3052 - configure the idle pressure tolerance (in digits)
- M3053 - configure the number of A/D converter reads per pressure measurement
- M3054 - configure the delay (in ms) between two A/D converter reads
- M3055 - configure the pressure tolerance (in digits) per pressure measurement

- M3070 - pause the print as if the "Pause" button would have been pressed

- M3079 - output the printed object
- M3080 - park the printer

- M3090 - test the watchdog (this command resets the firmware)

- M3100 - configure the number of manual z steps after the "Heat Bed up" or "Heat Bed down" button has been pressed
- M3101 - configure the number of manual extruder steps after the "Extruder output" or "Extruder retract" button has been pressed
- M3102 - configure the offset in x, y and z direction as well as the extruder retract which shall be applied in case the "Pause Printing" button has been pressed
- M3103 - configure the x, y and z position which shall set when the printer is parked
- M3104 - configure the x, y and z position which shall set when the printed object is output
*/


#define SCAN_X_START_MM					15																// [mm]
#define SCAN_X_START_STEPS				long(XAXIS_STEPS_PER_MM * SCAN_X_START_MM)						// [steps]
#define SCAN_X_END_MM					5																// [mm]
#define SCAN_X_END_STEPS				long(XAXIS_STEPS_PER_MM * SCAN_X_END_MM)						// [steps]
#define SCAN_X_STEP_SIZE_MM				20																// [mm]
#define SCAN_X_STEP_SIZE_STEPS			long(XAXIS_STEPS_PER_MM * SCAN_X_STEP_SIZE_MM)					// [steps]
#define SCAN_X_MAX_POSITION_STEPS		long(X_MAX_LENGTH * XAXIS_STEPS_PER_MM - SCAN_X_END_STEPS)		// [steps]

#define	SCAN_Y_START_MM					20																// [mm]
#define	SCAN_Y_START_STEPS				long(YAXIS_STEPS_PER_MM * SCAN_Y_START_MM)						// [steps]
#define	SCAN_Y_END_MM					5																// [mm]
#define	SCAN_Y_END_STEPS				long(YAXIS_STEPS_PER_MM * SCAN_Y_END_MM)						// [steps]
#define SCAN_Y_STEP_SIZE_MM				20																// [mm]
#define	SCAN_Y_STEP_SIZE_STEPS			long(YAXIS_STEPS_PER_MM * SCAN_Y_STEP_SIZE_MM)					// [steps]
#define SCAN_Y_MAX_POSITION_STEPS		long(Y_MAX_LENGTH * YAXIS_STEPS_PER_MM - SCAN_Y_END_STEPS)		// [steps]

#define SCAN_HEAT_BED_UP_FAST_STEPS		-20																// [steps]
#define SCAN_HEAT_BED_UP_SLOW_STEPS		-4																// [steps]
#define SCAN_HEAT_BED_DOWN_SLOW_STEPS	10																// [steps]
#define SCAN_HEAT_BED_DOWN_FAST_STEPS	long(ZAXIS_STEPS_PER_MM / 4)									// [steps]
#define	SCAN_FAST_STEP_DELAY_MS			5																// [ms]
#define	SCAN_SLOW_STEP_DELAY_MS			100																// [ms]
#define SCAN_IDLE_DELAY_MS				250																// [ms]

#define SCAN_CONTACT_PRESSURE_DELTA		10																// [digits]
#define SCAN_RETRY_PRESSURE_DELTA		5																// [digits]
#define SCAN_IDLE_PRESSURE_DELTA		0																// [digits]

#define SCAN_RETRIES					3																// [-]
#define	SCAN_PRESSURE_READS				15																// [-]
#define SCAN_PRESSURE_TOLERANCE			15																// [digits]
#define SCAN_PRESSURE_READ_DELAY_MS		15																// [ms]

#define	SCAN_STRAIN_GAUGE				ACTIVE_STRAIN_GAUGE
#define	HEAT_BED_COMPENSATION_X			long((X_MAX_LENGTH - SCAN_X_START_MM - SCAN_X_END_MM) / SCAN_X_STEP_SIZE_MM + 4)
#define	HEAT_BED_COMPENSATION_Y			long((Y_MAX_LENGTH - SCAN_Y_START_MM - SCAN_Y_END_MM) / SCAN_Y_STEP_SIZE_MM + 4)
#define COMPENSATION_VERSION			2
#define EEPROM_OFFSET_VERSION			0
#define EEPROM_OFFSET_DIMENSION_X		2
#define EEPROM_OFFSET_DIMENSION_Y		4
#define EEPROM_OFFSET_Z_START			6
#define EEPROM_DELAY					2																// [ms]
#define XYZ_DIRECTION_CHANGE_DELAY		250																// [탎]
#define XYZ_STEPPER_HIGH_DELAY			250																// [탎]
#define XYZ_STEPPER_LOW_DELAY			250																// [탎]
#define EXTRUDER_DIRECTION_CHANGE_DELAY	250																// [탎]
#define EXTRUDER_STEPPER_HIGH_DELAY		40000															// [탎]
#define EXTRUDER_STEPPER_LOW_DELAY		250																// [탎]
#define	LOOP_INTERVAL					1000															// [ms]
#define REMEMBER_PRESSURE				0
#define	Z_COMPENSATION_INTERVAL			18

#define DEFAULT_MANUAL_Z_STEPS			16
#define DEFAULT_MANUAL_EXTRUDER_STEPS	(EXT0_STEPS_PER_MM /2)
#define	DEFAULT_PAUSE_STEPS_X			(XAXIS_STEPS_PER_MM *50)
#define	DEFAULT_PAUSE_STEPS_Y			(YAXIS_STEPS_PER_MM *50)
#define DEFAULT_PAUSE_STEPS_Z			(ZAXIS_STEPS_PER_MM *2)
#define	DEFAULT_PAUSE_STEPS_EXTRUDER	(EXT0_STEPS_PER_MM *10)

#define	TASK_ENABLE_Z_COMPENSATION		1
#define	TASK_DISABLE_Z_COMPENSATION		2
#define	TASK_INIT_Z_COMPENSATION		3
#define TASK_PAUSE_PRINT_1				4
#define TASK_PAUSE_PRINT_2				5


extern	char			g_nDirectionX;
extern	char			g_nDirectionY;
extern	char			g_nDirectionZ;
extern	char			g_nDirectionE;

#if FEATURE_Z_COMPENSATION
extern	char			g_abortScan;
extern	short			g_HeatBedCompensation[HEAT_BED_COMPENSATION_X][HEAT_BED_COMPENSATION_Y];
#endif // FEATURE_Z_COMPENSATION

extern	short			g_noZCompensationSteps;
extern	short			g_maxZCompensationSteps;
extern	short			g_diffZCompensationSteps;
extern	short			g_manualCompensationSteps;
extern	short			g_offsetHeatBedCompensation;
extern	char			g_nHeatBedScanStatus;
extern	unsigned char	g_uHeatBedMaxX;
extern	unsigned char	g_uHeatBedMaxY;
extern	long			g_minX;
extern	long			g_maxX;
extern	long			g_minY;
extern	long			g_maxY;
extern	long			g_recalculatedCompensation;
extern	char			g_debugLevel;
//extern	short			g_debugCounter[10];
extern	short			g_nHeatBedScanZ;

// other configurable parameters
#if FEATURE_EXTENDED_BUTTONS
extern	unsigned short	g_nManualZSteps;
extern	unsigned short	g_nManualExtruderSteps;
#endif // FEATURE_EXTENDED_BUTTONS

#if FEATURE_PAUSE_PRINTING
extern	short			g_nPauseStepsX;
extern	short			g_nPauseStepsY;
extern	short			g_nPauseStepsZ;
extern	short			g_nPauseStepsExtruder;
extern	short			g_nContinueStepsX;
extern	short			g_nContinueStepsY;
extern	short			g_nContinueStepsZ;
extern	short			g_nContinueStepsExtruder;
extern	char			g_pausePrint;
extern	char			g_printingPaused;
#endif // FEATURE_PAUSE_PRINTING


// initRF1000()
extern void initRF1000( void );

// initStrainGauge()
extern void initStrainGauge( void );

// readStrainGauge()
extern short readStrainGauge( unsigned char uAddress );

#if FEATURE_Z_COMPENSATION
// scanHeatBed()
extern void scanHeatBed( void );
#endif // FEATURE_Z_COMPENSATION

// testExtruderTemperature()
extern short testExtruderTemperature( void );

// testHeatBedTemperature()
extern short testHeatBedTemperature( void );

#if FEATURE_Z_COMPENSATION
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

// moveX()
extern int moveX( int nSteps );

// moveY()
extern int moveY( int nSteps );

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
extern char saveCompensationMatrix( void );

// restoreCompensationMatrix()
extern char restoreCompensationMatrix( void );

// clearCompensationMatrix()
extern void clearCompensationMatrix( void );

// outputPressureMatrix()
extern void outputPressureMatrix( void );
#endif // FEATURE_Z_COMPENSATION

// writeByte24C256()
extern void writeByte24C256( int addressI2C, unsigned int addressEEPROM, unsigned char data );

// writeWord24C256()
extern void writeWord24C256( int addressI2C, unsigned int addressEEPROM, unsigned short data );

// readByte24C256()
extern unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM );

// readWord24C256()
extern unsigned short readWord24C256( int addressI2C, unsigned int addressEEPROM );

// loopRF1000()
extern void loopRF1000( void );

#if FEATURE_Z_COMPENSATION
// startHeatBedScan()
extern void startHeatBedScan( void );
#endif // FEATURE_Z_COMPENSATION

#if FEATURE_OUTPUT_PRINTED_OBJECT
// outputObject()
extern void outputObject( void );
#endif // FEATURE_OUTPUT_PRINTED_OBJECT

#if FEATURE_PARK
// parkPrinter()
extern void parkPrinter( void );
#endif // FEATURE_PARK

// pausePrint()
extern void pausePrint( void );

// continuePrint()
extern void continuePrint( void );

// processCommand()
extern void processCommand( GCode* pCommand );

// runStandardTasks()
extern void runStandardTasks( void );

// queueTask()
extern void queueTask( char task );

// processButton()
extern void processButton( int nAction );

// CalculateAllowedZStepsAfterEndStop()
extern void CalculateAllowedZStepsAfterEndStop( void );
#if STEPPER_CURRENT_CONTROL==CURRENT_CONTROL_DRV8711

// setMotorCurrent()
void setMotorCurrent( unsigned char driver, unsigned short level );

// motorCurrentControlInit()
void motorCurrentControlInit();

#endif // CURRENT_CONTROL_DRV8711
