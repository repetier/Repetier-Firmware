/*

This file defines io solutions used. This is the lowest level and is the base
for all higher level functions using io operations. At several places we need
subsets of these list of operations. To make configuration easy and easy to 
understand, we use a technique called "x macros". This requires that only
predefined macro names for IO are used here. Do NOT add anything else here
or compilation/functionality will break.

Rules:
1. Each definition will create a class that is named like the first parameter.
This class is later used as input to templates building higher functions. By
convention the names should start with IO followed by something that helps you
identify the function. 
2. Do not use a semicolon at the end. Macro definition gets different meanings
and will add the semicolon if required.

*/

/* Define motor pins here. Each motor needs a setp, dir and enable pin. */

ENDSTOP_NONE(endstopNone)

// X Motor

IO_OUTPUT_FULL(IOX1Step, ORIG_X_STEP_PIN)
IO_OUTPUT_INVERTED_FULL(IOX1Dir, ORIG_X_DIR_PIN)
IO_OUTPUT_FULL(IOX1Enable, ORIG_X_ENABLE_PIN)

// Y Motor

IO_OUTPUT_FULL(IOY1Step, ORIG_Y_STEP_PIN)
IO_OUTPUT_FULL(IOY1Dir, ORIG_Y_DIR_PIN)
IO_OUTPUT_FULL(IOY1Enable, ORIG_Y_ENABLE_PIN)

// Z Motor

IO_OUTPUT_FULL(IOZ1Step, ORIG_Z_STEP_PIN)
IO_OUTPUT_INVERTED_FULL(IOZ1Dir, ORIG_Z_DIR_PIN)
IO_OUTPUT_FULL(IOZ1Enable, ORIG_Z_ENABLE_PIN)

// E0 Motor

IO_OUTPUT_FULL(IOE0Step, ORIG_E0_STEP_PIN)
IO_OUTPUT_INVERTED_FULL(IOE0Dir, ORIG_E0_DIR_PIN)
IO_OUTPUT_INVERTED_FULL(IOE0Enable, ORIG_E0_ENABLE_PIN)

// E1 Motor

IO_OUTPUT_FULL(IOE1Step, ORIG_E1_STEP_PIN)
IO_OUTPUT_FULL(IOE1Dir, ORIG_E1_DIR_PIN)
IO_OUTPUT_FULL(IOE1Enable, ORIG_E1_ENABLE_PIN)

// Define your endstops inputs

IO_INPUT(IOEndstopXMin, ORIG_X_MIN_PIN)
IO_INPUT(IOEndstopYMax, ORIG_Y_MAX_PIN)
IO_INPUT(IOEndstopZMin, ORIG_Z_MIN_PIN)

// Define our endstops solutions

ENDSTOP_SWITCH(endstopXMin, IOEndstopXMin)
ENDSTOP_SWITCH(endstopYMax, IOEndstopYMax)
ENDSTOP_SWITCH(endstopZMin, IOEndstopZMin)

// Define fans

IO_OUTPUT_FULL(IOFan1, ORIG_FAN_PIN)
IO_PWM_SOFTWARE(Fan1NoKSPWM, IOFan1, 0)
// IO_PDM_SOFTWARE(Fan1NoKSPWM, IOFan1) // alternative to PWM signals
IO_PWM_KICKSTART(Fan1PWM, Fan1NoKSPWM, 20)

// Define temperature sensors

// Typically they requre an analog input (12 bit) so define
// them first.

IO_ANALOG_INPUT(IOAnalogBed0, TEMP_1_PIN, 5)
IO_ANALOG_INPUT(IOAnalogExt0, TEMP_0_PIN, 5)
IO_ANALOG_INPUT(IOAnalogExt1, TEMP_2_PIN, 5)

// Need a conversion table for epcos NTC
IO_TEMP_TABLE_NTC(TempTableEpcos, Epcos_B57560G0107F000)

// Now create the temperature inputs

IO_TEMPERATURE_TABLE(TempBed, IOAnalogBed0, TempTableEpcos)
IO_TEMPERATURE_TABLE(TempExt0, IOAnalogExt0, TempTableEpcos)
IO_TEMPERATURE_TABLE(TempExt1, IOAnalogExt1, TempTableEpcos)
