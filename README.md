# Repetier-Firmware - the fast and user friendly firmware

## Important notice of this development version

This development version is under heavy code change. Im reorganizing and refactoring
this code. Planned achievements for this release:

* Works with CodeBlocks for Arduino http://www.arduinodev.com/codeblocks/#download
  which can replace the ArduinoIDE with a much better one on windows systems. Load the
  Repetier.cdb project file for this.
* Better readable code.
* Separation of logic and hardware access to allow different processor architectures
  by changing the hardware related files.
* z-leveling support.
* Modified OPS handling.

## Documentation

by repetier  (repetierdev@gmail.com)

For documentation please visit [http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/)

Beta version with added delta move and homing calculations and several other features for delta printers.

To set rod height on a delta printer use command M251 which will allow you to measure 
the rod length or, if you have an LCD panel then you can select delta calibration in the
configuration menu.

Version 0.81  03.02.2013

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
