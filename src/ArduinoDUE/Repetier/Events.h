#ifndef EVENTS_H_INCLUDED
#define EVENTS_H_INCLUDED

/*
Event system in a nutshell:

All printers are different and my need additions in th eone or other place.
It is not very convenient to add these code parts across the firmware. For this
reason repetier-firmware uses a simple event system that comes at no cost if
an event is not used.

- simple: Only one subscriber is possible
- cost effective: Macros work as event caller. By default all macros are empty

How to use the system:

1. In Configuration.h add
#define CUSTOM_EVENTS
2. Add a file "CustomEvents.h" which overrides all event macros you need.
   It should also include the function declarations used.
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
// Gets called every time the jam detection signal switches. Steps are the extruder steps since last change.
#define EVENT_JAM_SIGNAL_CHANGED(extruderId,steps) {}
// Gets called if a heater decoupling is detected.
#define EVENT_HEATER_DECOUPLED(id) {}
// Gets called if a missing/shorted thermistor is detected.
#define EVENT_HEATER_DEFECT(id) {}
// Gets called if a action in ui.cpp okAction gets executed.
#define EVENT_START_UI_ACTION(shortAction) {}
// Gets called if a nextPrevius actions gets executed.
#define EVENT_START_NEXTPREVIOUS(action,increment) {}
// Gets called before a move is queued. Gives the ability to limit moves.
#define EVENT_CONTRAIN_DESTINATION_COORDINATES
// Gets called when a fatal error occurs and all actions should be stopped
#define EVENT_FATAL_ERROR_OCCURED
// Gets called after a M999 to continue from fatal errors
#define EVENT_CONTINUE_FROM_FATAL_ERROR

// Allow adding new G and M codes. To implement it create a function
// bool eventUnhandledGCode(GCode *com);
// that returns true if it handled the code, otherwise false.
// Event define would then be
// #define EVENT_UNHANDLED_G_CODE(c) eventUnhandledGCode(c)
#define EVENT_UNHANDLED_G_CODE(c) false
#define EVENT_UNHANDLED_M_CODE(c) false

// Called when bed temperature is set
#define EVENT_SET_BED_TEMP(temp,boop)

// This gets called every time the user has saved a value to eeprom
// or any other reason why dependent values may need recomputation.
#define EVENT_UPDATE_DERIVED {}

// This gets called after the basic firmware functions have initialized.
// Use this to initalize your hardware etc.
#define EVENT_INITIALIZE {}

#endif // EVENTS_H_INCLUDED
