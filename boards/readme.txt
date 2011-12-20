The serial implementation of the arduino is not optimal. In the subfolders
Sanguino and arduino are faster implementations of these. Copy the files
to the proper places of your arduino installations to replace the existing ones.
You find them at

Arduino Pre 1.0 version:
<Installpath>/hardware/arduino/cores/arduino

Arduino 1.0+ version:
<Installpath>/hardware/arduino/cores/arduino

Sanguino version:
<Installpath>/hardware/Sanguino/cores/arduino
Make sure to copy the Arduino.h which is needed to compile on Arduino 1.0.

They are fully compatible with the original files. Only the slow modulo
operator % is replaced by a fast "and" operation.
 