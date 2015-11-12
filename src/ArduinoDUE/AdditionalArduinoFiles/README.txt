The Due implementation of the default arduino disables the watchdog feature at
startup. Due to ARM limitation, it is not possible to enable a watchdog once it
is disabled. For that reason, the watchdog feature does not work with default
Arduino installation. 

Also some other boards like the Alligator board need their own variants, which can be
copied and installed the same way.

To solve the problem copy the modified version as follows:

---------------------------------

For Arduino 1.6 and newer

Install latest Arduino IDE and in Tools->Board->Boards Manager download the 
Arduino Sam Boards to get basic structure for Due base boards.

boards.txt needts to to to C:\Users\<your user name>\AppData\Roaming\Arduino15\packages\arduino\hardware\sam\1.6.4

It contains the 2 original entries plus a new one called Arduino Due for Repetier.
If you have modifed the boards.txt before, add only the last entry in boards.txt.

Copy the folder arduino_due_repetier to
C:\Users\<YourUserName>\AppData\Roaming\Arduino15\packages\arduino\hardware\sam\1.6.4\variants

After a restart you have a new board in your list. Use it only if you
compile with watchdog feature enabled (which is the preferred way for increased safety)

----------------------------------

For Arduin 1.5.0 - 1.5.8


boards.txt needts to to to <ArduinoInstallPath>/hardware/arduino/sam

It contains the 2 original entries plus a new one called Arduino Due for Repetier.
If you have modifed the boards.txt before, add only the last entry in boards.txt.

Copy the folder arduino_due_repetier to
<ArduinoInstallPath>/hardware/arduino/sam/variants

After a restart you have a new board in your list. Use it only if you
compile with watchdog feature enabled (which is the preferred way for increased safety)

------------ License -----------

The files contained here are part of the Arduino package and their
license terms stay valid regardless of the license of the core firmware.  