/*
Custom macro definition for usage in Configuration_io.h

How it works:
Modules get defined by calling a macro in Configuration_io.h
To make a module work it needs to add code at several stategic
position of the firmware. YOu might need to initialize a hardware component
to use it later or you want some parameters to appear in the eeprom editor.

This is done by a simple trick, once you have understood it. At all these
stategic places we add

#undef IO_TARGET
#define IO_TARGET <number 1-12>
#include "/src/io/redefine.h"

redefine.h includes now all .h files in src/io which define all the 
standard modules and then this file gets included and then
Configuration_io.h gets included.

Now we have the possibility to add up to 12 code parts, that get added
into the firmware code at these strategic places. To make it
easier to understand we provide here a demo module SAMPLE_MODULE
that writes a message to the console every 500ms. In Configuration_io.h
the integration would be

SAMPLE_MODULE("Ping")

As you see, at the beginning we create a dummy definition for all
targets where we do not need to add code. In our example we
only need target 12 which gets called every 500ms. Here we undef the old
macro definition (to get no warnings) and define a new version
that simply writes the line. In all othe rcases our empty start
defintion takes care about not causing any compile errors and just adds nothing.

For some complex problems the firmware has specialized base classes
that have many hooks. E.g. the Tool class has many hooks that add
tests and functions at several key time points. 

Hints:
- Make sure always to use same parameter list!
- Macros can go over multiple lines if they end with \, but make sure nothing
  even a white space follows!
- If you add an alternative way e.g. for output pin check the src/io files for
  the base class and how they are normally integrated.
- Unused modules will not use any memory, so we can include any number of modules.
  If you hav ewritten and tested a module that might be useful to others, please
  consider sending it to us, so we can include it in the official version.
  The more modules we have the more flexible the firmware gets.
- Don't put long or slow code into fast interrupts (PWM interrupt, Endstop update, analog input loop)
  and do not create output in the interrupts,
  that might break the motion planner or crash firmware. 100ms call is run in
  main thread and has no real limits on what you can do.
*/

// First add dummy

#undef SAMPLE_MODULE
#define SAMPLE_MODULE(msg)

// Now only add code where you really need it.

#if IO_TARGET == 1    // Init at firmware start
#elif IO_TARGET == 2  // PWM Interrupt
#elif IO_TARGET == 3  // 100ms call
#elif IO_TARGET == 4  // Define class
#elif IO_TARGET == 5  // Endstop update
#elif IO_TARGET == 6  // define variables
#elif IO_TARGET == 7  // Visualization for config
#elif IO_TARGET == 8  // eepromHandle calls
#elif IO_TARGET == 9  // updateDerived calls
#elif IO_TARGET == 10 // restore from config
#elif IO_TARGET == 11 // analog input loop
#elif IO_TARGET == 12 // 500ms timer

#undef SAMPLE_MODULE
#define SAMPLE_MODULE(msg) \
    Com::printFLN(PSTR(msg));

#endif
