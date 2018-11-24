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

// X Motor left

IO_OUTPUT(IOX1Step, ORIG_X_STEP_PIN)
IO_OUTPUT(IOX1Dir, ORIG_X_DIR_PIN)
IO_OUTPUT(IOX1Enable, ORIG_X_ENABLE_PIN)

// X Motor right

IO_OUTPUT(IOAStep, ORIG_E2_STEP_PIN)
IO_OUTPUT(IOADir, ORIG_E2_DIR_PIN)
IO_OUTPUT(IOAEnable, ORIG_E2_ENABLE_PIN)

// Y Motor

IO_OUTPUT(IOY1Step, ORIG_Y_STEP_PIN)
IO_OUTPUT(IOY1Dir, ORIG_Y_DIR_PIN)
IO_OUTPUT(IOY1Enable, ORIG_Y_ENABLE_PIN)

// Z Motor

IO_OUTPUT(IOZ1Step, ORIG_Z_STEP_PIN)
IO_OUTPUT_INVERTED(IOZ1Dir, ORIG_Z_DIR_PIN)
IO_OUTPUT(IOZ1Enable, ORIG_Z_ENABLE_PIN)

// E0 Motor

IO_OUTPUT(IOE1Step, ORIG_E0_STEP_PIN)
IO_OUTPUT(IOE1Dir, ORIG_E0_DIR_PIN)
IO_OUTPUT(IOE1Enable, ORIG_E0_ENABLE_PIN)

// E1 Motor

IO_OUTPUT(IOE2Step, ORIG_E1_STEP_PIN)
IO_OUTPUT(IOE2Dir, ORIG_E1_DIR_PIN)
IO_OUTPUT(IOE2Enable, ORIG_E1_ENABLE_PIN)

// Controller input pins

#if UI_ENCODER_CLICK >= 0
IO_INPUT_INVERTED_PULLUP(ControllerClick, UI_ENCODER_CLICK)
#else
IO_INPUT_DUMMY(ControllerClick, false)
#endif
#if UI_ENCODER_A >= 0
IO_INPUT_INVERTED_PULLUP(ControllerEncA, UI_ENCODER_A)
#else
IO_INPUT_DUMMY(ControllerEncA, false)
#endif
#if UI_ENCODER_B >= 0
IO_INPUT_INVERTED_PULLUP(ControllerEncB, UI_ENCODER_B)
#else
IO_INPUT_DUMMY(ControllerEncB, false)
#endif
#if UI_BACK_PIN >= 0
IO_INPUT_PULLUP(ControllerBack, UI_BACK_PIN)
#else
IO_INPUT_DUMMY(ControllerBack, false)
#endif
#if UI_RESET_PIN >= 0
IO_INPUT_PULLUP(ControllerReset, UI_RESET_PIN)
#else
IO_INPUT_DUMMY(ControllerReset, false)
#endif

// Define your endstops inputs

//IO_INPUT_PULLUP(IOEndstopXMax, ORIG_X_MAX_PIN)
IO_INPUT_PULLUP(IOEndstopXMin, ORIG_X_MIN_PIN)
IO_INPUT_PULLUP(IOEndstopYMin, ORIG_Y_MIN_PIN)
IO_INPUT_PULLUP(IOEndstopZMin, ORIG_Z_MIN_PIN)

// Define our endstops solutions
// You need to define all min and max endstops for all
// axes except E even if you have none!

ENDSTOP_SWITCH_HW(endstopXMin, IOEndstopXMin, X_AXIS, false)
ENDSTOP_SWITCH_HW(endstopYMin, IOEndstopYMin, Y_AXIS, false)
ENDSTOP_SWITCH_HW(endstopZMin, IOEndstopZMin, Z_AXIS, false)
ENDSTOP_NONE(endstopAMin)
ENDSTOP_NONE(endstopXMax)
ENDSTOP_NONE(endstopYMax)
ENDSTOP_NONE(endstopZMax)
ENDSTOP_SWITCH(endstopAMax, IOEndstopXMin)
// Set to nullptr for no zprobe or &endstopName for a switch
#undef ZPROBE_ADDRESS
#define ZPROBE_ADDRESS nullptr

// Define fans

IO_OUTPUT(IOFan1, ORIG_FAN_PIN)
IO_OUTPUT(IOFan2, ORIG_FAN2_PIN)
IO_OUTPUT(IOCoolerFan1, HEATER_3_PIN)

IO_PWM_SOFTWARE(Fan1NoKSPWM, IOFan1, 0)
IO_PWM_SOFTWARE(Fan2NoKSPWM, IOFan2, 0)
IO_PWM_SOFTWARE(CoolerFan, IOCoolerFan1, 0)
// IO_PWM_HARDWARE(Fan1PWM, 37,5000)
// IO_PDM_SOFTWARE(Fan1NoKSPWM, IOFan1) // alternative to PWM signals
IO_PWM_KICKSTART(Fan1PWM, Fan1NoKSPWM, 20)
IO_PWM_KICKSTART(Fan2PWM, Fan2NoKSPWM, 20)

// Define temperature sensors

// Typically they require an analog input (12 bit) so define
// them first.

IO_ANALOG_INPUT(IOAnalogBed1, TEMP_1_PIN, 5)
IO_ANALOG_INPUT(IOAnalogExt1, TEMP_0_PIN, 5)
IO_ANALOG_INPUT(IOAnalogExt2, TEMP_2_PIN, 5)

// Need a conversion table for epcos NTC
IO_TEMP_TABLE_NTC(TempTablePT100, PT100_3_3V)

// Now create the temperature inputs

IO_TEMPERATURE_TABLE(TempBed1, IOAnalogBed1, TempTablePT100)
IO_TEMPERATURE_TABLE(TempExt1, IOAnalogExt1, TempTablePT100)
IO_TEMPERATURE_TABLE(TempExt2, IOAnalogExt2, TempTablePT100)
IO_HOTTEST_OF_2(TempHottestExtruder, TempExt1, TempExt2)

// Use PWM outputs to heat. If using hardware PWM make sure
// that the selected pin can be used as hardware pwm otherwise
// select a software pwm model whcih works on all pins.

IO_OUTPUT(IOExtr1, HEATER_0_PIN)
IO_OUTPUT(IOExtr2, HEATER_2_PIN)
IO_OUTPUT(IOBed1, HEATER_1_PIN)
IO_PWM_SOFTWARE(PWMExtruder1, IOExtr1, 1)
IO_PWM_SOFTWARE(PWMExtruder2, IOExtr2, 1)
IO_PWM_SOFTWARE(PWMBed1, IOBed1, 1)

// IO_OUTPUT(IOCooler1, FAN2_PIN)
// IO_PWM_SOFTWARE(PWMCoolerExt1, FAN2_PIN, 0)

// Define all stepper motors used
// For deltas the top is the minumum position in motor coordinates
// therefore the max endstops of a delta need to be entered as
// minimum endstop here!
STEPPER_SIMPLE(XMotor, IOX1Step, IOX1Dir, IOX1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(AMotor, IOAStep, IOADir, IOAEnable, endstopNone, endstopNone)
STEPPER_SIMPLE(YMotor, IOY1Step, IOY1Dir, IOY1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(ZMotor, IOZ1Step, IOZ1Dir, IOZ1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(E1Motor, IOE1Step, IOE1Dir, IOE1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(E2Motor, IOE2Step, IOE2Dir, IOE2Enable, endstopNone, endstopNone)
// Heat manages are used for every component that needs to
// control temperature. Higher level classes take these as input
// and simple heater like a heated bed use it directly.

HEAT_MANAGER_PID(HeatedBed1, 'B', 0, TempBed1, PWMBed1, 70, 255, 10, 300000, 131.1, 3.76, 1143, 80, 255, false)
HEAT_MANAGER_PID(HeaterExtruder1, 'E', 0, TempExt1, PWMExtruder1, 260, 255, 20, 20000, 18.3, 2.13, 39, 40, 235, false)
HEAT_MANAGER_PID(HeaterExtruder2, 'E', 1, TempExt2, PWMExtruder2, 260, 255, 20, 20000, 18.3, 2.13, 39, 40, 235, false)
COOLER_MANAGER_SENSOR(ExtruderCooler, TempHottestExtruder, CoolerFan, 40, 100, 200, 255)

// Coolers are stand alone functions that allow it to control
// a fan with external sensors. Many extruders require a cooling
// fan pointer to the extruder to prevent heat rising up.
// These can be controlled by the cooler. Since it is
// independent you just tell what part needs cooling.
// Other use cases are board cooling and heated chambers.

// Define tools. They get inserted into a tool array in configuration.h
// Typical tools are:
// TOOL_EXTRUDER(name, offx, offy, offz, heater, stepper, resolution, yank, maxSpeed, acceleration, advance, startScript, endScript)

TOOL_EXTRUDER(ToolExtruder1, 0, 0, 0, HeaterExtruder1, E1Motor, 1.75, 2230, 5, 30, 5000, 40, "M117 Extruder 1", "", &Fan1PWM)
TOOL_EXTRUDER(ToolExtruder2, 0, 0, 0, HeaterExtruder2, E2Motor, 1.75, 2230, 5, 30, 5000, 40, "M117 Extruder 2", "", &Fan2PWM)
