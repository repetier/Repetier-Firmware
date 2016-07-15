There are some printers sold under different names like 'Zonestar P802M',
'Prusa i3 P802M DIY kit', 'Anet A8-B', etc, which have LCD 20x4 with 5 keys controller
connected to Melzi V2.0 board via 10 wires cable. Keys are connected to a
single analog input using resistive divider.

Despite GPL license, manufacturers ignore the requirement to share modified
code. So this is a community effort to reimplement the ADC keypad support.

More information on the issue:
    http://forum.repetier.com/discussion/1105/melzi-v2-0-with-lcd2004-and-5-keys

All credits to original modifications and schematics should go to their authors:
 - Hally for the original modified code
 - Mixanoid, Mekanofreak, axelsp for their work on this

Since now the Zonestar support is merged into the official code, this firmware
can be compiled using normal process. You may use the sample Configuration.h
provided as a starting point.

TO BUILD THE FIRMWARE:
- Install Sanguino board support to the Arduino IDE via board manager options:
  https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
- Optionally flash new bootloader via ISP programmer (using Arduino IDE options).
  Originally supplied bootloader uses 57600 baud rate instead of 115200 and has
  some issues. This update is optional.
- Upload sample Configuration.h to the web configurator and make any changes
  if desired. Download new Configuration.h if changed, or firmware zip bundle.
- Build and flash the firmware.

IMPORTANT NOTES:
- It seems that some Zonestar printers have different steppers and/or
  Z mechanics. Some of them require 400 Z-steps/mm, some 1600. The latter
  value is used by default, but if you have issues with Z-steps - change
  them to 400.
- At least some Zonestar printers have issues with Watchdog enabled.
  Particularly, if no SD card inserted, such printers with watchdog enabled
  cannot boot and constantly restart. So by default the watchdog option is
  now disabled. Enable it if desired.

Do not forget about EEPROM modes, so you use this firmware settings and not
previously stored. M502 (restore firmware defaults), M500 (save to EEPROM)
commands may help.

Oleg Semyonov (osnwt@forum.repetier.com)
