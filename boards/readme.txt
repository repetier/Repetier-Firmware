The serial implementation of the arduino is not optimal. In the subfolders
Sanguino and arduino are faster implementations of these. Copy the files
to the proper places of your arduino installations to replace the existing ones.
You find them at

Arduino Pre 1.0 version:
------------------------
<Installpath>/hardware/arduino/cores/arduino

Arduino 1.0+ version:
---------------------
<Installpath>/hardware/arduino/cores/arduino

The input buffer length is 64 bytes with this version. Make sure to reduce
the buffer length in Repetier-Host or you will get overflows with unwanted
results!

Arduino 1.0.3 changed the handling a bit to allows setting stop bits/parity/data-bits.
If you mind losing this functionality, dont copy the files!

Sanguino pre 1.0 version:
-------------------------
<Installpath>/hardware/Sanguino/cores/arduino

Sanguino 1.0+ version:
----------------------
<Installpath>/hardware/Sanguino/cores/arduino
Make sure to copy the Arduino.h which is needed to compile on Arduino 1.0.
Replace the original boards.txt with the boards.txt so uploading files works.

gen7 pre 1.0 version:
---------------------
<Installpath>/hardware/Gen7/cores/arduino

gen7 1.0+ version:
------------------
<Installpath>/hardware/Gen7/cores/arduino
Make sure to copy the Arduino.h which is needed to compile on Arduino 1.0.
Replace the original boards.txt with the boards.txt so uploading files works.

============================

The versions are the Arduino IDE versions not the board versions!

The wiring files are fully compatible with the original files. Only the slow modulo
operator % is replaced by a fast "and" operation.
 