The firmware now allows hardware abstraction. That way it is possible to
port the code to other boards not using Arduino/AVR.

If you have a normal 8 bit Arduino board, use the ArduinoAVR folder to compile.

If you have a Arduino Due based board, use the ArduinoDUE folder. It contains the
adjusted HAL files from John Silvia. It requires Arduino 1.5 or higher to compile.
Upload and connect through the programming port near the power jack.
Status: Beta and work in progress.