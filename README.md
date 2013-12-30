# Repetier-Firmware - the fast and user friendly firmware

## Installation

Please use your new at [http://www.repetier.com/firmware/v091](http://www.repetier.com/firmware/v091)
for easy and fast configuration. You get the complete sources you need to compile back.
This system also allows it to upload configurations created with this tool and modify
the configuration.

## Version 0.91 released 2013-12-30

Improvements over old code:
* Works with CodeBlocks for Arduino http://www.arduinodev.com/codeblocks/#download
  which can replace the ArduinoIDE with a much better one on windows systems. Load the
  Repetier.cdb project file for this.
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

Supported boards:

The following boards are supported by setting the proper motherboard type.Other boards
require a matching pin definition.

* MEGA/RAMPS up to 1.2       = 3
* RAMPS 1.3/RAMPS 1.4        = 33
* Azteeg X3                  = 34
* Gen6                       = 5 
* Gen6 deluxe                = 51
* Sanguinololu up to 1.1     = 6
* Sanguinololu 1.2 and above = 62
* Melzi board                = 63
* Gen7 1.1 till 1.3.x        = 7
* Gen7 1.4.1 and later       = 71
* Teensylu (at90usb)         = 8 // requires Teensyduino
* Printrboard (at90usb)      = 9 // requires Teensyduino
* Foltyn 3D Master           = 12
* MegaTronics                = 70
* Megatronics 2.0            = 701
* RUMBA                      = 80  // Get it from reprapdiscount
* Rambo                      = 301

## Features

- RAMP acceleration support.
- Path planning for higher print speeds.
- Trajectory smoothing for smoother lines.
- Ooze prevention system for faster anti ooze then slicer can do,
- Nozzle pressure control for improved print quality with RAMPS.
- Fast - 40000 Hz and more stepper frequency is possible with a 16 MHz AVR. 
- Multiple extruder supported (max. 6 extruder).
- Standard ASCII and improved binary (Repetier protocol) communication.
- Autodetect the command protocol, so it will work with any host software.
- Continuous monitoring of one temperature.
- Important parameters are stored in EEPROM and can easily be modified without
  recompilation of the firmware.
- Stepper control is handled in an interrupt routine, leaving time for
  filling caches for next move.
- PID control for extruder/heated bed temperature.
- Interrupt based sending buffer (Arduino library normally waits for the
  recipient to receive written data)
- Small RAM memory print, resulting in large caches.
- Supports SD-cards.
- mm and inches can be used for G0/G1
- Arc support
- Works with Skeinforge 41, all unknown commands are ignored.
- Dry run : Execute yout GCode without using the extruder. This way you can
  test for non-extruder related failures without actually printing.

## Controlling firmware

Also you can control the firmware with any reprap compatible host, you will only get
the full benefits with the following products, which have special code for this
firmware:

* [Repetier-Host for Windos/Linux](http://www.repetier.com/download/)
* [Repetier-Host for Mac](http://www.repetier.com/download/)
* [Repetier-Server](http://www.repetier.com/repetier-server-download/)

## Installation

For documentation and installation please visit 
[http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/).

## Changelog

See changelog.txt
