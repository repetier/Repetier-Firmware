#!/bin/bash
# Copies hardware independent files from due to avr version

cp Repetier/Repetier.h  ../ArduinoAVR/Repetier
cp Repetier/Repetier.ino  ../ArduinoAVR/Repetier
cp Repetier/Commands.*  ../ArduinoAVR/Repetier
cp Repetier/Communication.*  ../ArduinoAVR/Repetier
cp Repetier/Eeprom.*  ../ArduinoAVR/Repetier
cp Repetier/Extruder.*  ../ArduinoAVR/Repetier
cp Repetier/gcode.*  ../ArduinoAVR/Repetier
cp Repetier/motion.*  ../ArduinoAVR/Repetier
cp Repetier/Printer.*  ../ArduinoAVR/Repetier
cp Repetier/SDCard.cpp  ../ArduinoAVR/Repetier
cp Repetier/ui.*  ../ArduinoAVR/Repetier
cp Repetier/Drivers.*  ../ArduinoAVR/Repetier
cp Repetier/uiconfig.h  ../ArduinoAVR/Repetier
cp Repetier/uilang.*  ../ArduinoAVR/Repetier
cp Repetier/uimenu.h  ../ArduinoAVR/Repetier
cp Repetier/u8*.h  ../ArduinoAVR/Repetier
cp Repetier/logo.h  ../ArduinoAVR/Repetier
cp Repetier/Events.h  ../ArduinoAVR/Repetier
cp Repetier/BedLeveling.*  ../ArduinoAVR/Repetier
cp Repetier/DisplayList.*  ../ArduinoAVR/Repetier
cp Repetier/Endstops.*  ../ArduinoAVR/Repetier
cp Repetier/Distortion.*  ../ArduinoAVR/Repetier
cp -r Repetier/src  ../ArduinoAVR/Repetier

echo Copying finished. AVR tree is now up to date.
