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
IO_OUTPUT_FAKE(fakeOut)

// X

IO_OUTPUT(IOX1Step, ORIG_X_STEP_PIN)
IO_OUTPUT(IOX1Dir, ORIG_X_DIR_PIN)
IO_OUTPUT_INVERTED(IOX1Enable, ORIG_X_ENABLE_PIN)

// Y Motor

IO_OUTPUT(IOY1Step, ORIG_Y_STEP_PIN)
IO_OUTPUT_INVERTED(IOY1Dir, ORIG_Y_DIR_PIN)
IO_OUTPUT_INVERTED(IOY1Enable, ORIG_Y_ENABLE_PIN)

// Z Motor

IO_OUTPUT(IOZ1Step, ORIG_Z_STEP_PIN)
IO_OUTPUT_INVERTED(IOZ1Dir, ORIG_Z_DIR_PIN)
IO_OUTPUT_INVERTED(IOZ1Enable, ORIG_Z_ENABLE_PIN)

// E0 Motor

IO_OUTPUT(IOE1Step, ORIG_E0_STEP_PIN)
IO_OUTPUT_INVERTED(IOE1Dir, ORIG_E0_DIR_PIN)
IO_OUTPUT_INVERTED(IOE1Enable, ORIG_E0_ENABLE_PIN)

// E1 Motor

IO_OUTPUT(IOE2Step, ORIG_E1_STEP_PIN)
IO_OUTPUT_INVERTED(IOE2Dir, ORIG_E1_DIR_PIN)
IO_OUTPUT_INVERTED(IOE2Enable, ORIG_E1_ENABLE_PIN)

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

IO_INPUT_INVERTED(IOEndstopXMax, ORIG_X_MIN_PIN)
IO_INPUT_INVERTED(IOEndstopYMax, ORIG_Y_MAX_PIN)
IO_INPUT_INVERTED(IOEndstopZMax, ORIG_Z_MAX_PIN)
// IO_INPUT_INVERTED_PULLUP(IOEndstopYMin, ORIG_Y_MIN_PIN)
IO_INPUT_PULLUP(IOEndstopZProbe, PD13) // EXP3 left back pin

// Define our endstops solutions
// You need to define all min and max endstops for all
// axes except E even if you have none!

ENDSTOP_SWITCH_HW(endstopMotorXMax, IOEndstopXMax, NO_AXIS, true)
ENDSTOP_SWITCH_HW(endstopMotorYMax, IOEndstopYMax, NO_AXIS, true)
ENDSTOP_SWITCH_HW(endstopMotorZMax, IOEndstopZMax, NO_AXIS, true)
ENDSTOP_NONE(endstopXMin)
ENDSTOP_NONE(endstopYMin)
ENDSTOP_NONE(endstopZMin)
ENDSTOP_NONE(endstopXMax)
ENDSTOP_NONE(endstopYMax)
ENDSTOP_MERGE3(endstopZMax, endstopMotorXMax, endstopMotorYMax, endstopMotorZMax, Z_AXIS, true)
ENDSTOP_SWITCH_HW(endstopZProbe, IOEndstopZProbe, ZPROBE_AXIS, false)

// Servo for z-probe
IO_OUTPUT(Servo1Pin, 4)
SERVO_ANALOG(ZProbeServo, 0, Servo1Pin, 500, 2500, 1473)
// Set to nullptr for no zprobe or &endstopName for a switch
#undef ZPROBE_ADDRESS
#define ZPROBE_ADDRESS &endstopZProbe

// Define fans

IO_PWM_HARDWARE(CoolerFan, ORIG_FAN2_PIN, 1000)
//IO_PWM_HARDWARE(Fan1PWMnoKS, ORIG_FAN_PIN, 5000)
//IO_PWM_KICKSTART(Fan1PWM, Fan1PWMnoKS, 10)
// IO_PWM_HARDWARE(Fan1PWMNoMin, ORIG_FAN_PIN, 1000)
IO_OUTPUT(IOFan1, ORIG_FAN_PIN)
IO_PWM_SOFTWARE(Fan1PWMNoMin, IOFan1, 0)
IO_PWM_MIN_SPEED(Fan1PWM, Fan1PWMNoMin, 128, false)

// Define temperature sensors

// Typically they require an analog input (12 bit) so define
// them first.

IO_ANALOG_INPUT(IOAnalogExt1, TEMP_0_PIN, 5)
IO_ANALOG_INPUT(IOAnalogExt2, TEMP_2_PIN, 5)
IO_ANALOG_INPUT(IOAnalogBed1, TEMP_1_PIN, 5)

// Need a conversion table for epcos NTC
IO_TEMP_TABLE_NTC(TempTableEpcos, Epcos_B57560G0107F000)
// Now create the temperature inputs

// IO_TEMPERATURE_TABLE(TempBed1, IOAnalogBed1, TempTableEpcos)
IO_TEMPERATURE_FAKE(TempBed1, 24.5)
IO_TEMPERATURE_TABLE(TempExt1, IOAnalogExt1, TempTableEpcos)
IO_TEMPERATURE_TABLE(TempExt2, IOAnalogExt2, TempTableEpcos)
IO_HOTTEST_OF_2(TempHottestExtruder, TempExt1, TempExt2)

// Use PWM outputs to heat. If using hardware PWM make sure
// that the selected pin can be used as hardware pwm otherwise
// select a software pwm model whcih works on all pins.

IO_PWM_HARDWARE(PWMExtruder1_12V, HEATER_2_PIN, 1000)
IO_PWM_HARDWARE(PWMExtruder2_12V, HEATER_0_PIN, 1000)
IO_PWM_SCALEDOWN(PWMExtruder1, PWMExtruder1_12V, 63)
IO_PWM_SCALEDOWN(PWMExtruder2, PWMExtruder2_12V, 63)
IO_PWM_HARDWARE(PWMBed1, HEATER_1_PIN, 1000)

// Define all stepper motors used
// For deltas the top is the minumum position in motor coordinates
// therefore the max endstops of a delta need to be entered as
// minimum endstop here!
STEPPER_TMC2130_HW_SPI(XMotor, IOX1Step, IOX1Dir, IOX1Enable, ORIG_X_CS_PIN, 0.11, 1, 64, 900, true, 100, 8, 12500000, endstopNone, endstopMotorXMax)
STEPPER_TMC2130_HW_SPI(YMotor, IOY1Step, IOY1Dir, IOY1Enable, ORIG_Y_CS_PIN, 0.11, 1, 64, 900, true, 100, 8, 12500000, endstopNone, endstopMotorYMax)
STEPPER_TMC2130_HW_SPI(ZMotor, IOZ1Step, IOZ1Dir, IOZ1Enable, ORIG_Z_CS_PIN, 0.11, 1, 64, 900, true, 100, 8, 12500000, endstopNone, endstopMotorZMax)
STEPPER_TMC2130_HW_SPI(E1Motor, IOE1Step, IOE1Dir, IOE1Enable, ORIG_E0_CS_PIN, 0.11, 1, 32, 900, true, 100, 8, 12500000, endstopNone, endstopNone)
STEPPER_TMC2130_HW_SPI(E2Motor, IOE2Step, IOE2Dir, IOE2Enable, ORIG_E1_CS_PIN, 0.11, 1, 32, 900, true, 100, 8, 12500000, endstopNone, endstopNone)

// Heat manages are used for every component that needs to
// control temperature. Higher level classes take these as input
// and simple heater like a heated bed use it directly.
// HEAT_MANAGER_PID(HeatedBed1, 'B', 0, TempBed1, PWMBed1, 120, 255, 1000, 10, 300000, 131.1, 3.76, 1143, 80, 255, false)
HEAT_MANAGER_BANG_BANG(HeatedBed1, 'B', 0, TempBed1, PWMBed1, 120, 255, 5, 1000, true)
HEAT_MANAGER_DEFINE_HYSTERESIS(HeatedBed1, 0.0, 20000, 0)
HEAT_MANAGER_PID(HeaterExtruder1, 'E', 0, TempExt1, PWMExtruder1, 310, 255, 1000, 20, 20000, 18.3, 2.13, 39, 40, 235, false)
HEAT_MANAGER_DEFINE_HYSTERESIS(HeaterExtruder1, 2.0, 20000, 300000)
HEAT_MANAGER_PID(HeaterExtruder2, 'E', 1, TempExt2, PWMExtruder2, 310, 255, 1000, 20, 20000, 18.3, 2.13, 39, 40, 235, false)
HEAT_MANAGER_DEFINE_HYSTERESIS(HeaterExtruder2, 2.0, 20000, 300000)
COOLER_MANAGER_SENSOR(ExtruderCooler, TempHottestExtruder, CoolerFan, 70, 200, 150, 255)

// Coolers are stand alone functions that allow it to control
// a fan with external sensors. Many extruders require a cooling
// fan pointer to the extruder to prevent heat rising up.
// These can be controlled by the cooler. Since it is
// independent you just tell what part needs cooling.
// Other use cases are board cooling and heated chambers.

// Define tools. They get inserted into a tool array in configuration.h
// Typical tools are:

TOOL_EXTRUDER(ToolExtruder1, 0, -13, 0, HeaterExtruder1, E1Motor, 1.75, 500, 5, 30, 5000, 177, "M117 Extruder 1", "", &Fan1PWM)
TOOL_EXTRUDER(ToolExtruder2, 0, 13, 0, HeaterExtruder2, E2Motor, 1.75, 500, 5, 30, 5000, 177, "M117 Extruder 2", "", &Fan1PWM)
// Define beeper output
#if BEEPER_PIN > -1
IO_OUTPUT(IOBeeperMain, BEEPER_PIN)
BEEPER_SOURCE_IO(MainBeeper, IOBeeperMain)
#endif
