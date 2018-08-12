# Repetier-Firmware - the fast and user friendly firmware

## Notes for developers/pull requests

This software is open source licensed under the GPL V3. As any free project we
like contributions from other sources, especially since the firmware is very
hardware related and many things can only be implemented/tested with the right
hardware. To allow easy integration of new features you should follow some simple
principals.
1. Only send pull requests against development version. This is where we add
and test new features and bug fixes. From time to time we push these to master
as a new version.
2. Do not include your personal configuration files. If you need new configuration
options, add them to the official configuration.h file.
3. We have two folders for different processor architectures. So for most
changes modifications need to be in both folders. The general files are identical.
In fact we develop on avr and just copy them to due. Only pins.h/fastio.h/hal.*
and configuration.h are architecture dependent, so changes there need to be made
twice and not copied.
4. Document what your pull request will change/fix/introduce. Please also mention
new configurations since we need to add them also to our online tool, so users
can set them correctly.

.0 will be the last 1 version. Fixes for this will be done in the dev branch and
moved to stable after a while as 1.0.x patch updates. With the release of the
official 1.0 version we will start developing version 2.0 in the branch dev2
where all the new features will be added.

### Version 2.0 information

Version 2 will be a incompatible refactoring of version 1.0. We will try to keep
commands and communication identical to 1.0, but for the configuration we see
a much more flexible way that makes it much easier to adjust the firmware to
nowadays needs. But that requires a different configuration at several parts.

Planned stages:
1. Merge AVR and Due into one fileset.
2. Split long files like boards, displays, languages in several files into a subfolder.
    That way it gets much faster to search the right place.
3. Update used libraries.
4. Change configuration.
5. Change motion planner.
6. Create a config tool.
7. Public testing with more users. We assume until a config tool is available the configuration
will get several changes and only users with programming skills will do it manually
to benefit already from improvements achieved.

Until 6. the version should be considered alpha stage. We do expect errors from all
the changes and there may be bigger mods with an update. Of course we will try
to keep every release workable, but as work in progress there is no guarantee. 

## Installation

Please use your configuration tool at 
[http://www.repetier.com/firmware/v100](http://www.repetier.com/firmware/v100)
or for latest 1.0.x development version at
[http://www.repetier.com/firmware/dev](http://www.repetier.com/firmware/dev)
for easy and fast configuration. You get the complete sources you need to compile from the online configurator.
This system also allows it to upload configurations created with this tool and modify the configuration. This is handy for updates as you get all newly introduced parameter just by uploading the old version and downloading the
latest version. New parameter are initalized with default values.

## Version 1.0.0
* Fixed many autoleveling bugs.
* More supported boards and displays.
* Added CNC/Laser modes.
* Improved event system to extend firmware without modification of files.
* Dual X axis support (2 separate x axis)
* New gcode handler for more flexible support of different inputs.
* Improved menu.
* Jam detection.
* More stable temperature control with PID.
* Per axis homing flag.
* Keep alive signals for hosts.
* Support capabilities protocol.
* Many bug fixes.

## Version 0.92.8 
* Cleaner code base.
* Pulse dense modulation for heater and fans.
* Bed bump correction for delta printer.
* Correction of parallelogram distortions.
* Decoupling test for heater and sensor for more safety.
* Mixing extruder support.
* Test for watchdog.
* Allow cold extrusion.
* Fixed pause sd print issues.
* Commands on sd stop.
* Disable heaters/extruders on sd stop.
* Safety question for sd stop.
* Many minor corrections and improvements.
* Extra motor drivers.
* Event system for lights etc.
* New homing sequence with preheat for nozzle based z sensors.
* Language selectable on runtime.
* Fix structure for Arduino 1.6.7
* New bed leveling.
* Fatal error handling added.

## Version 0.91 released 2013-12-30

WARNING: This version only compiles with older Arduino IDE 1.0.x, for
compilation with newest version use 0.92

Improvements over old code:
* Better readable code.
* Long filename support (from Glenn Kreisel).
* Animated menu changes.
* Separation of logic and hardware access to allow different processor architectures
  by changing the hardware related files.
* z-leveling support.
* Mirroring of x,y and z motor.
* Ditto printing.
* Faster and better delta printing.
* New heat manager (dead time control).
* Removed OPS handling.
* Full graphic display support.
* Many bug fixes.
* many other changes.

## Documentation

For documentation please visit [http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/)

## Developer

The sources are managed by the Hot-World GmbH & Co. KG
It was initially based on the Sprinter firmware from Kliment, but the code has run
through many changes since them.
Other developers:
- Glenn Kreisel (long filename support)
- Martin Croome (first delta implementation)
- John Silvia (Arduino Due port)
- sdavi (first u8glib code implementation)
- plus several small contributions from other users.

## Introduction

Repetier-Firmware is a firmware for RepRap like 3d-printer powered with
an arduino compatible controller.
This firmware is a nearly complete rewrite of the sprinter firmware by kliment
which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
Some ideas were also taken from Teacup, Grbl and Marlin.

## Features

- Supports cartesian, delta and core xy/yz printers.
- RAMP acceleration support.
- Path planning for higher print speeds.
- Trajectory smoothing for smoother lines.
- Nozzle pressure control for improved print quality with RAMPS.
- Fast - 40000 Hz and more stepper frequency is possible with a 16 MHz AVR.
- Support for Arduino Due based boards allowing much faster speeds. 
- Multiple extruder supported (max. 6 extruder).
- Standard ASCII and improved binary (Repetier protocol) communication.
- Autodetect the command protocol, so it will work with any host software.
- Important parameters are stored in EEPROM and can easily be modified without
  recompilation of the firmware.
- Automatic bed leveling.
- Mixed extruder.
- Detection of heater/thermistor decoupling.
- 2 fans plus thermistor controlled fan.
- Multi-Language support, switchable at runtime.
- Stepper control is handled in an interrupt routine, leaving time for
  filling caches for next move.
- PID control for extruder/heated bed temperature.
- Interrupt based sending buffer (Arduino library normally waits for the
  recipient to receive written data)
- Small RAM memory print, resulting in large caches.
- Supports SD-cards.
- mm and inches can be used for G0/G1
- Arc support
- Dry run : Execute yout GCode without using the extruder. This way you can
  test for non-extruder related failures without actually printing.

## Controlling firmware

Also you can control the firmware with any reprap compatible host, you will only get
the full benefits with the following products, which have special code for this
firmware:

* [Repetier-Host for Windows/Linux](http://www.repetier.com/download/)
* [Repetier-Host for Mac](http://www.repetier.com/download/)
* [Repetier-Server](http://www.repetier.com/repetier-server-download/)

## Installation

For documentation and installation please visit 
[http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/).

## Changelog

See changelog.txt
