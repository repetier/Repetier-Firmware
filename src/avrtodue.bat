REM Copies hardware independent files from avr to due version

copy ArduinoAVR\Repetier\Repetier.h  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Repetier.ino  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Commands.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Communication.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Eeprom.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Extruder.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\FatStructs.h  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\gcode.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\motion.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\Printer.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\SDCard.cpp  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\SdFat.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\ui.*  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\uiconfig.h  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\uilang.h  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\uimenu.h  ArduinoDUE\Repetier
copy ArduinoAVR\Repetier\u8*.h  ArduinoDUE\Repetier

echo Copying finished. DUE tree is now up to date.
pause
