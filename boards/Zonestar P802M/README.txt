There are some printers sold under different names like 'Zonestar P802M',
'Prusa i3 P802M DIY kit', etc, which have LCD 20x4 with 5 keys controller
connected to Melzi V2.0 board via 10 wires cable. Keys are connected to a
single analog input using resitive divider.

Despite GPL license, manufacturers ignore the requirement to share modified
code. So this is a community effort to reimplement the ADC keypad support.

More information on the issue:
    http://forum.repetier.com/discussion/1105/melzi-v2-0-with-lcd2004-and-5-keys

All credits to original modifications and schematics should go to their authors:
 - Hally for the original modified code
 - Mixanoid, Mekanofreak, axelsp for their work on this

This is the attempt to rework the code and possibly merge it into the official
firmware repository.

Using this version, normal web configuration is fully supported. Some options
are overridden by the Manual config section of web UI. Read comments there
for details. There are two sample configurations supplied. The basic one
uses mostly default values for the printer. The normal one has faster settings
(including better keypad response and autorepeat). It is recommended as a
starting point.

TO BUILD THE FIRMWARE:
- Install Sanguino board support to the Arduino IDE via board manager options:
  https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
- Optionally flash new bootloader via ISP programmer (using Arduino IDE options).
  Supplied uses 57600 baud rate and has some issues. This update is optional.
- Upload one of supplied configurations to the web configurator and make any
  cnahges if desired. Download new Configuration.h if changed.
- Build and flash the firmware with this Configuration.h file.

Do not forget about EEPROM modes, so you use this firmware settings and not
previously stored.

Oleg Semyonov (osnwt@forum.repetier.com)
