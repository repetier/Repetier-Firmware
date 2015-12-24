#ifndef EVENTS_H_INCLUDED
#define EVENTS_H_INCLUDED

/*
Event system in a nutshell:

All printers are different and my need additions in th eone or other place.
It is not very convenient to add these code parts across the firmware. For this
reason repetier-firmware uses a simple event system that comes at no cost if
a event is not used.

- simple: Only one subscriber is possible
- cost effective: Macros work as event caller. By default all macros are empty

How to use the system:

1. In Configuration.h add
#define CUSTOM_EVENTS
2. Add a file "CustomEvents.h" which overrides all event macros you need.
   It shoudl also include the function declarations used.
3. Add a file "CustomEventsImpl.h" which includes all function definitions.
   Also it is named .h it will be included inside a cpp file only once.
   This is to compile only when selected and still keep ArduinoIDE happy.

Each of the following events describe the parameter and when it is called.
*/

// Catch heating events. id is extruder id or -1 for heated bed.
#define EVENT_WAITING_HEATER(id) {}
#define EVENT_HEATING_FINISHED(id) {}

// This gets called every 0.1 second
#define EVENT_TIMER_100MS {}
// This gets called every 0.5 second
#define EVENT_TIMER_500MS {}
// Gets called on a regular basis as time allows
#define EVENT_PERIODICAL {}
// Gets called when kill gets called. only_steppes = true -> we only want to disable steppers, not everything.
#define EVENT_KILL(only_steppers) {}
// Gets called when a jam was detected.
#define EVENT_JAM_DETECTED {}
// Gets called everytime the jam detection signal switches. Steps are the extruder steps since last change.
#define EVENT_JAM_SIGNAL_CHANGED(extruderId,steps) {}
// Gets called if a heater decoupling is detected.
#define EVENT_HEATER_DECOUPLED(id) {}
// Gets called if a missing/shorted thermistor is detected.
#define EVENT_HEATER_DEFECT(id) {}
// Gets called if a action in ui.cpp okAction gets executed.
#define EVENT_START_UI_ACTION(shortAction) {}
// Gets called if a nextPrevius actions gets executed.
#define EVENT_START_NEXTPREVIOUS(action,increment) {}

// Called to initalize laser pins. Return false to prevent default initalization.
#define EVENT_INITALIZE_LASER true
// Set laser to intensity level 0 = off, 255 = full. Return false if you have overridden the setting routine.
// with true the default solution will set it as digital value.
#define EVENT_SET_LASER(intensity) true

// Called to initalize cnc pins. Return false to prevent default initalization.
#define EVENT_INITALIZE_CNC true
// Turn off spindle
#define EVENT_SPINDLE_OFF true
// Turn spindle clockwise
#define EVENT_SPINDLE_CW(rpm) true
// Turn spindle counter clockwise
#define EVENT_SPINDLE_CCW(rpm) true

// Allow adding new G and M codes. To implement it create a function
// bool eventUnhandledGCode(GCode *com)
// that returns true if it handled the code, otherwise false.
// Event define would then be
// #define EVENT_UNHANDLED_G_CODE(c) eventUnhandledGCode(c)
#define EVENT_UNHANDLED_G_CODE(c) false
#define EVENT_UNHANDLED_M_CODE(c) false

// This gets called every time the user has saved a value to eeprom
// or any other reason why dependent values may need recomputation.
#define EVENT_UPDATE_DERIVED {}

// This gets called after the basic firmware functions have initialized.
// Use this to initalize your hardware etc.
#define EVENT_INITIALIZE {}

#endif // EVENTS_H_INCLUDED
