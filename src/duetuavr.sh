#!/bin/bash
# Copies hardware independent files from due to avr version

cp ArduinoDue/Repetier/Repetier.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Repetier.ino  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Commands.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Communication.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Eeprom.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Extruder.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/gcode.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/motion.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Printer.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/SDCard.cpp  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/ui.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Drivers.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/uiconfig.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/uilang.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/uimenu.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/u8*.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/logo.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Events.h  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/BedLeveling.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/DisplayList.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Endstops.*  ArduinoAVR/Repetier
cp ArduinoDue/Repetier/Distortion.*  ArduinoAVR/Repetier
cp -r ArduinoDue/Repetier/src  ArduinoAVR/Repetier/src  

echo copying finished. AVR tree is now up to date.

