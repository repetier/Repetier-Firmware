# RF Firmware for RF1000 and RF2000 devices
Based on Repetier-Firmware - the fast and user friendly firmware.

## Installation

The firmware is compiled and downloaded with Arduino V 1.6.5.

## Version RF.01.38 (2017-12-19)

* Current stable release.

## Documentation

For documentation please visit [http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/)

## Introduction

This variant of the Repetier-firmware has been optimized for the use with the
Renkforce RF1000 and Renkforce RF2000 3D printers of Conrad Electronic SE.
The firmware adds functionality which is not available within the standard
Repetier-firmware and uses the settings which match the available hardware.

The main differences to the standard Repetier-firmware are:

Common
* Support for motor current control via the TI DRV8711.
* Support for an external watchdog via the TI TPS382x.
* Printing can be paused/continued via a hardware button.
* Heat bed scan and Z-compensation via the built-in strain gauge of the extruder.
* Automatic emergency pause in case the strain gauge delivers too high measurements.

RF1000
* Support for the RF1000 motherboard.
* Support for the RF1000 feature controller (= 6 additional hardware buttons).
* Additional M-codes have been defined in order to control the RF1000-specific functionality.

RF2000
* Support for the RF2000 motherboard.
* Support for the RF2000 feature controller (= 6 additional hardware buttons).
* Additional M-codes have been defined in order to control the RF2000-specific functionality.

## Changelog

See changelog.txt
