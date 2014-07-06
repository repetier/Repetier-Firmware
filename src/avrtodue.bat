REM Copies hardware independent files from avr to due version

copy ArduinoAVR\Repetier\Repetier.h  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Repetier.ino  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Commands.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Communication.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Eeprom.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Extruder.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\FatStructs.h  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\gcode.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\motion.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\Printer.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\SDCard.cpp  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\SdFat.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\ui.*  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\uiconfig.h  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\uilang.h  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\uimenu.h  ArduinoDue\Repetier
copy ArduinoAVR\Repetier\u8*.h  ArduinoDue\Repetier

echo Copying finished. DUE tree is now up to date.
pause
