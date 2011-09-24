The serial implementation of the arduino is not optimal. In the subfolders
Sanguino and arduino are faster implementations of these. Copy the files
to the proper places of your arduino installations to replace the existing ones.
You find them at

Arduino version:
<Installpath>/hardware/arduino/cores/arduino
Sanguino version:
<Installpath>/hardware/Sanguino/cores/arduino

They are fully compatible with the original files. Only the slow modulo
operator % is replaced by a fast "and" operation.
 