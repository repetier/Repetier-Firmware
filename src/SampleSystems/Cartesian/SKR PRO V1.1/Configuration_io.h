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

/* 
    Work in progress sample configuration for the SKR E3 Mini v1.2.
    Configured for a Creality Ender 3 or similar bedslinger type printer,
    with an installed BLTouch and a generic Creality Ender 3 128x64 graphics
    display.

    Check out https://docfirmwarev2.repetier.com/config/introduction for more thorough 
    information on configuring RFW V2!
*/

// For use when no output is wanted, but possible
ENDSTOP_NONE(endstopNone)
IO_OUTPUT_FAKE(IOOutFake)
IO_PWM_FAKE(PWMFake)

/* Define motor pins here. Each motor needs a step, dir and enable pin. */

// X Motor

IO_OUTPUT(IOX1Step, ORIG_X_STEP_PIN)
IO_OUTPUT_INVERTED(IOX1Dir, ORIG_X_DIR_PIN)
IO_OUTPUT_INVERTED(IOX1Enable, ORIG_X_ENABLE_PIN)

// Y Motor

IO_OUTPUT(IOY1Step, ORIG_Y_STEP_PIN)
IO_OUTPUT_INVERTED(IOY1Dir, ORIG_Y_DIR_PIN)
IO_OUTPUT_INVERTED(IOY1Enable, ORIG_Y_ENABLE_PIN)

// Z Motor

IO_OUTPUT(IOZ1Step, ORIG_Z_STEP_PIN)
IO_OUTPUT(IOZ1Dir, ORIG_Z_DIR_PIN)
IO_OUTPUT_INVERTED(IOZ1Enable, ORIG_Z_ENABLE_PIN)

// E0 Motor

IO_OUTPUT(IOE1Step, ORIG_E0_STEP_PIN)
IO_OUTPUT_INVERTED(IOE1Dir, ORIG_E0_DIR_PIN)
IO_OUTPUT_INVERTED(IOE1Enable, ORIG_E0_ENABLE_PIN)

// Controller input pins

// Define your endstops inputs

IO_INPUT_PULLUP(IOEndstopXMin, ORIG_X_MIN_PIN)
IO_INPUT_PULLUP(IOEndstopYMin, ORIG_Y_MIN_PIN)

// SKR E3 Mini - Using a bltouch by default. 
// White and black wired connector to the "Probe" port.
IO_INPUT_PULLUP(IOEndstopZProbe, BLTOUCH_Z_MIN)

//IO_INPUT_PULLUP(IOEndstopZMin, ORIG_Z_MIN_PIN)

// Define our endstops solutions
// You need to define all min and max endstops for all
// axes except E even if you have none!

ENDSTOP_SWITCH_HW(endstopXMin, IOEndstopXMin, X_AXIS, false)
ENDSTOP_SWITCH_HW(endstopYMin, IOEndstopYMin, Y_AXIS, false)
//ENDSTOP_SWITCH_HW(endstopZMin, IOEndstopZMin, Z_AXIS, false)
ENDSTOP_SWITCH_HW(endstopZProbe, IOEndstopZProbe, ZPROBE_AXIS, false)

ENDSTOP_NONE(endstopXMax)
ENDSTOP_NONE(endstopYMax)
ENDSTOP_NONE(endstopZMax)
ENDSTOP_NONE(endstopZMin)

IO_OUTPUT(Servo1Pin, SERVO_1_PIN)
SERVO_ANALOG(ZProbeServo, 0, Servo1Pin, 500, 2500, 1473)

IO_PWM_HARDWARE(Fan1PWMRaw, FAN_PIN, 80)
IO_PWM_KICKSTART(Fan1PWM, Fan1PWMRaw, 2, 50)


// SKR E3 Mini - The following is meant as an example of the LIGHT_SOURCE module
// and for debugging. The led is fairly small and obscure. 
// Remove the little /* and */ 's to enable the functionality.
/*
IO_OUTPUT(IOStatusLED, STATUS_LED_PIN)
IO_PWM_SOFTWARE(PWMPcbStatusLED, IOStatusLED, 1)

LIGHT_STATE_PWM(PWMPcbLEDState)
LIGHT_COND(PWMPcbLEDState, true, LIGHT_STATE_OFF, 255, 255, 255, 100)

// Glow status light if any movement is happening.
LIGHT_COND(
    PWMPcbLEDState,
    Motion1::buffersUsed() > 0,
    LIGHT_STATE_ON, 255, 255, 255, 100)

// Advanced usage of the light conditions using a C++ lambda function
// to loop through our heaters and return true if any is turned on and
// then slowly blink. 
LIGHT_COND(
    PWMPcbLEDState, 
    [] {
        for (fast8_t i = 0; i < NUM_HEATERS; i++) {
            if (!heaters[i]->isOff()) {
                return true;
            }
        }
        return false;
    }(),
    LIGHT_STATE_BLINK_SLOW, 255, 255, 255, 100)

// "Burst" LED blink pattern if we have any sort of fatal error (decouple etc)
LIGHT_COND(PWMPcbLEDState, GCode::hasFatalError(), LIGHT_STATE_BURST, 255, 255, 255, 100)
LIGHT_SOURCE_PWM(PCBStatusLED, PWMPcbStatusLED, PWMPcbLEDState)
*/

// SKR E3 Mini - Using the creality 128x64 graphics display by default which uses an
// active buzzer. Using high-freq HW-PWM we can still run it like a passive buzzer anyway.
#if (FEATURE_CONTROLLER != NO_CONTROLLER) && (BEEPER_PIN > -1)
IO_PWM_HARDWARE(PWMBeeperMain, BEEPER_PIN, 100)
BEEPER_SOURCE_PWM(MainBeeper, PWMBeeperMain)
#endif


// Define temperature sensors

// Typically they require an analog input (12 bit) so define
// them first.
IO_ANALOG_INPUT(IOAnalogExt1, TEMP_0_PIN, 5)
IO_ANALOG_INPUT(IOAnalogBed1, TEMP_1_PIN, 5)

// Need a conversion table for 100k epcos NTC
IO_TEMP_TABLE_NTC(TempTableEpcos, Epcos_B57560G0107F000) 

IO_TEMPERATURE_TABLE(TempBed1, IOAnalogBed1, TempTableEpcos)
IO_TEMPERATURE_TABLE(TempExt1, IOAnalogExt1, TempTableEpcos)

// Controller input pins

#if defined(UI_ENCODER_CLICK) && UI_ENCODER_CLICK >= 0
IO_INPUT_INVERTED_PULLUP(ControllerClick, UI_ENCODER_CLICK)
#else
IO_INPUT_DUMMY(ControllerClick, false)
#endif
#if defined(UI_ENCODER_A) && UI_ENCODER_A >= 0
IO_INPUT_INVERTED_PULLUP(ControllerEncA, UI_ENCODER_A)
#else
IO_INPUT_DUMMY(ControllerEncA, false)
#endif
#if defined(UI_ENCODER_B) && UI_ENCODER_B >= 0
IO_INPUT_INVERTED_PULLUP(ControllerEncB, UI_ENCODER_B)
#else
IO_INPUT_DUMMY(ControllerEncB, false)
#endif

#if defined(UI_BACK_PIN) && UI_BACK_PIN >= 0
IO_INPUT_PULLUP(ControllerBack, UI_BACK_PIN)
#else
IO_INPUT_DUMMY(ControllerBack, false)
#endif
#if defined(UI_RESET_PIN) && UI_RESET_PIN >= 0
IO_INPUT_PULLUP(ControllerReset, UI_RESET_PIN)
#else
IO_INPUT_DUMMY(ControllerReset, false)
#endif

// Use PWM outputs to heat. If using hardware PWM make sure
// that the selected pin can be used as hardware pwm otherwise
// select a software pwm model whcih works on all pins.

// PWM_SOFTWARE available frequencies (STM32F1): 
// 0       1       2        3        4
// 39.2Hz, 78.7Hz, 158.7Hz, 322.5Hz, 666.6Hz

//IO_OUTPUT(IOExtruder, HEATER_0_PIN)
//IO_PWM_SOFTWARE(PWMExtruder, IOExtruder, 1)

// SKR E3 Mini - Both extruder and bed heaters are on the same HW timer (PWM),
// you CAN use HW PWM for both but they'll share the same frequency of the 
// first one you define! The bed heater CANNOT be run at such a high frequency. 
IO_PWM_HARDWARE(PWMExtruder, HEATER_0_PIN, 2000)
// Instead, just set the bed to software PWM using a slower speed which should
// work just fine.
IO_OUTPUT(IOBed, HEATER_1_PIN)
IO_PWM_SOFTWARE(PWMBed, IOBed, 0)



// Define all stepper motors used
// For deltas the top is the minumum position in motor coordinates
// therefore the max endstops of a delta need to be entered as
// minimum endstop here!

// SKR E3 Mini - X, Y, Z set to 550mA, E to 650mA, 16 microsteps by default.
STEPPER_TMC2209_SW_UART(XMotor, IOX1Step, IOX1Dir, IOX1Enable, X_SW_SERIAL_TMC_PIN, X_SW_SERIAL_TMC_PIN, 0.1f, 16, 550, 1, 0, 0, 0, 12, endstopNone, endstopNone)
STEPPER_TMC2209_SW_UART(YMotor, IOY1Step, IOY1Dir, IOY1Enable, Y_SW_SERIAL_TMC_PIN, Y_SW_SERIAL_TMC_PIN, 0.1f, 16, 550, 1, 0, 0, 0, 12, endstopNone, endstopNone)
STEPPER_TMC2209_SW_UART(ZMotor, IOZ1Step, IOZ1Dir, IOZ1Enable, Z_SW_SERIAL_TMC_PIN, Z_SW_SERIAL_TMC_PIN, 0.1f, 16, 550, 1, 0, 0, 0, 12, endstopNone, endstopNone)
STEPPER_TMC2209_SW_UART(E1MotorBase, IOE1Step, IOE1Dir, IOE1Enable, E0_SW_SERIAL_TMC_PIN, E0_SW_SERIAL_TMC_PIN, 0.1f, 16, 650, 1, 0, 0, 0, 12, endstopNone, endstopNone)

/*
// Following definitions were just used for debugging
STEPPER_SIMPLE(XMotor, IOX1Step, IOX1Dir, IOX1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(YMotor, IOY1Step, IOY1Dir, IOY1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(ZMotor, IOZ1Step, IOZ1Dir, IOZ1Enable, endstopNone, endstopNone)
STEPPER_SIMPLE(E1MotorBase, IOE1Step, IOE1Dir, IOE1Enable, endstopNone, endstopNone)
*/

STEPPER_OBSERVEABLE(E1Motor, E1MotorBase)

// Heat manages are used for every component that needs to
// control temperature. Higher level classes take these as input
// and simple heater like a heated bed use it directly.

HEAT_MANAGER_PID(HeaterExtruder1, 'E', 0, TempExt1, PWMExtruder, 280, 255, 1000, 25, 45000, 21.73, 1.54, 76.55, 0, 235, false)
HEAT_MANAGER_PID(HeatedBed1, 'B', 0, TempBed1, PWMBed, 120, 255, 1000, 20, 0, 131.1, 3.76, 1143, 80, 255, false)


// Coolers are stand alone functions that allow it to control
// a fan with external sensors. Many extruders require a cooling
// fan pointer to the extruder to prevent heat rising up.
// These can be controlled by the cooler. Since it is
// independent you just tell what part needs cooling.
// Other use cases are board cooling and heated chambers.
// eg. COOLER_MANAGER_MOTORS(BoardFanController, BoardFan, 0, 192, 10)
// SKR E3 Mini - No coolers available by default.



// Define tools. They get inserted into a tool array in configuration.h
// Typical tools are:
TOOL_EXTRUDER(ToolExtruder1, 0, 0, 0, HeaterExtruder1, E1Motor, 1.75, 93.0, 20, 30, 5000, 40, "", "", &Fan1PWM) 


// SKR E3 Mini - Since the creality ender 3 128x64 display has an active buzzer, 
// we need to use higher frequencies to avoid turning on it's internal oscillator.
// (If you want to mute a specific theme, just set it to {0, 0} and it'll never play.)
#if FEATURE_CONTROLLER == CONTROLLER_ENDER_3_12864
#define CUSTOM_DEFAULT_THEMES 1 
TONE_THEME(ThemeButtonNextPrev, TONES({ 12500, 10 }))
TONE_THEME(ThemeButtonOk, TONES({ 12000, 15 }, { 9500, 15 }))
TONE_THEME(ThemeButtonReset, TONES({ 6250, 20 }, { 5500, 30 }, { 0, 50 }, { 6500, 50 }))
TONE_THEME(ThemeNotifyError, TONES({ 8500, 100 }, { 9750, 150 }, { 8500, 100 }, { 9750, 150 }, { 0, 500 }))
TONE_THEME(ThemeNotifyConfirm, TONES({ 9250, 70 }, { 9500, 100 }))
TONE_THEME(ThemeNotifyWarning, TONES({ 5800, 150 }, { 6100, 30 }, { 5300, 100 }))
#endif 