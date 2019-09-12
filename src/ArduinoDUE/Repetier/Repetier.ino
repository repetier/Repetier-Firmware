/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm
   firmware.

    Main author: repetier

*/
/**
\mainpage Repetier-Firmware for Arduino based RepRaps
<CENTER>Copyright &copy; 2011-2013 by repetier
</CENTER>

\section Intro Introduction


\section GCodes Implemented GCodes

 look here for descriptions of gcodes:
http://linuxcnc.org/handbook/gcode/g-code.html and
http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

Implemented Codes

- G0  -> G1
- G1  - Coordinated Movement X Y Z E, S1 disables boundary check, S0 enables it
- G2 - Clockwise arc  X,Y,E = end position, R = Radius or I,J = center
- G3 - Counterclockwise arc   X,Y,E = end position, R = Radius or I,J = center
- G4  - Dwell S<seconds> or P<milliseconds>
- G10 S<1 = long retract, 0 = short retract = default> retracts filament
according to stored setting
- G11 S<1 = long retract, 0 = short retract = default> = Undo retraction
according to stored setting
- G20 - Units for G0/G1 are inches.
- G21 - Units for G0/G1 are mm.
- G28 - Home all axis or named axis.
- G29 S<0..2> - Z-Probe at the 3 defined probe points. S = 1 measure avg.
zHeight, S = 2 store avg zHeight
- G30 P<0..3> - Single z-probe at current position P = 1 first measurement, P =
2 Last measurement P = 0 or 3 first and last measurement
- G30 H<height> R<offset> Make probe define new Z and z offset (R) at trigger
point assuming z-probe measured an object of H height.
- G31 - Write signal of probe sensor
- G32 S<0..2> P<0..1> - Autolevel print bed. S = 1 measure zLength, S = 2
Measure and store new zLength
- G33 - Measure distortion map
- G33 R0 - delete distortion map
- G33 L0 - List distortion map
- G33 X<xpos> Y<ypos> Z<newdistortioncorrection> - Set new distortion for
nearest distortion point.
- G90 - Use absolute coordinates
- G91 - Use relative coordinates
- G92 - Set current position to coordinates given
- G131 - set extruder offset position to 0 - needed for calibration with G132
- G132 - calibrate endstop positions. Call this, after calling G131 and after
centering the extruder holder.
- G133 - measure steps until max endstops for deltas. Can be used to detect lost
steps within tolerances of endstops.
- G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!)
Px = reference extruder, Sx = only measure extrude x against reference, Zx = add
to measured z distance for Sx for correction.
- G201 P<motorId> X<pos>     - Go to position X with motor X
- G202 P<motorId> X<setpos>  - Mark current position as X
- G203 P<motorId>            - Report current motor position
- G204 P<motorId> S<0/1>     - Enable/disable motor
- G205 P<motorId> S<0/1> E<0/1> - Home motor, S1 = go back to stored position,
E1 = home only if endstop was never met, meaning it was never homed with motor.

RepRap M Codes

- M104 - Set extruder target temp
- M105 - Read current temp
- M106 S<speed> P<fan> I<ignore> - Fan on speed = 0..255, P = 0 or 1, 0 is
default and can be omitted, I1 = Ignore M106/M107 from now on, I0 = disable
ignore
- M107 P<fan> - Fan off, P = 0 or 1, 0 is default and can be omitted
- M109 - Wait for extruder current temp to reach target temp. Same params as
M104
- M114 S1 - Display current position, S1 = also write position in steps

Custom M Codes

- M3 Sx - Spindle on, Clockwise or Laser on during G1 moves. Sx = laser
intensity 0-255 if driver supports this (default ignores it)
- M4 - Spindle on, Counterclockwise.
- M5 - Spindle off, Laser off.
- M17 - Enable all motors or only named motors
- M18 - Disable all motors or named motors
- M20  - List SD card
- M21  - Initialize SD card
- M22  - Release SD card
- M23  - Select SD file (M23 filename.g)
- M24  - Start/resume SD print
- M25  - Pause SD print
- M26  - Set SD position in bytes (M26 S12345)
- M27  - Report SD print status
- M28  - Start SD write (M28 filename.g)
- M29  - Stop SD write
- M30 <filename> - Delete file on sd card
- M32 <dirname> create subdirectory
- M42 P<pin number> S<value 0..255> - Change output of pin P to S. Does not work
on most important pins.
- M80  - Turn on power supply
- M81  - Turn off power supply
- M82  - Set E codes absolute (default)
- M83  - Set E codes relative while in Absolute Coordinates (G90) mode
- M84  - Disable steppers until next move,
        or use S<seconds> to specify an inactivity timeout, after which the
steppers will be disabled.  S0 to disable the timeout.
- M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set
zero (default)
- M92  - Set axisStepsPerMM - same syntax as G92
- M99 S<delayInSec> X0 Y0 Z0 - Disable motors for S seconds (default 10) for
given axis.
- M104 S<temp> T<extruder> P1 F1 H1 O<offset>- Set temperature without wait. P1
= wait for moves to finish, F1 = beep when temp. reached first time O add offset
to temperature in S, H1 use preheat temperature instead of S value.
- M105 X0 - Get temperatures. If X0 is added, the raw analog values are also
written.
- M111 S<debugflags> - Set debugging option. Add values for wanted options:
            1 = echo commands, 2 = info, 4 = errors, 8 = dry run mode, 16 = only
communication, no actions
- M112 - Emergency kill
- M115- Capabilities string
- M116 - Wait for all temperatures in a +/- 1 degree range
- M117 <message> - Write message in status row on lcd
- M118 <message> - Write message to host
- M119 - Report endstop status
- M122 - Output TMC2130 diagnostic readings
- M140 S<temp> H1 O<offset> F1 - Set bed target temp, F1 makes a beep when
temperature is reached the first time
- M155 S<1/0> Enable/disable auto report temperatures. When enabled firmware
will emit temperatures every second.
- M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
- M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights
as virtual extruder S
- M170 B<bedtemp> T<extruderid> S<extrudertemp> L0 - Set preset temperatures for
extruder (T+S) or bed (B) or list settings (L0)
- M190 - Wait for bed current temp to reach target temp. Same params as M109
- M200 T<extruder> D<diameter> - Use volumetric extrusion. Set D0 or omit D to
disable volumetric extr. Omit T for current extruder.
- M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
- M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)
- M203 - Set temperature monitor to Sx
- M204 - Set PID parameter X => Kp Y => Ki Z => Kd S<extruder> Default is
current extruder. NUM_EXTRUDER=Heated bed
- M205 - Output EEPROM settings
- M206 - Set EEPROM value
- M207 X<XY jerk> Z<Z Jerk> E<ExtruderJerk> - Changes current jerk values, but
do not store them in eeprom.
- M209 S<0/1> - Enable/disable auto retraction
- M218 T<extruderId> X<offset> Y<offset> Z<offset> S<0/1> - Set extruder offset.
S1 = Save to eeprom.
- M220 S<Feedrate multiplier in percent> - Increase/decrease given feedrate
- M221 S<Extrusion flow multiplier in percent> - Increase/decrease given flow
rate
- M226 P<pin> S<state 0/1> - Wait for pin getting state S. Add X0 to init as
input without pull-up and X1 for input with pull-up.
- M231 S<OPS_MODE> X<Min_Distance> Y<Retract> Z<Backlash> F<ReatrctMove> - Set
OPS parameter
- M232 - Read and reset max. advance values
- M233 X<AdvanceK> Y<AdvanceL> - Set temporary advance K-value to X and linear
term advanceL to Y
- M251 Measure Z steps from homing stop (Delta printers). S0 - Reset, S1 -
Print, S2 - Store to Z length (also EEPROM if enabled)
- M280 S<mode> - Set ditto printing mode. mode: 0 = off, 1 = 1 extra extruder, 2
= 2 extra extruder, 3 = 3 extra extruders
- M281 Test if watchdog is running and working. Use M281 X0 to disable watchdog
on AVR boards. Sometimes needed for boards with old bootloaders to allow
reflashing.
- M290 Z<babysteps> - Correct by adding baby steps for Z mm
- M300 S<Frequency> P<DurationMillis> play frequency
- M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1
will allow, S0 will disallow.
- M303 P<extruder/bed> S<printTemerature> X0 R<Repetitions> C<method>- Auto
detect pid values. Use P<NUM_EXTRUDER> for heated bed. X0 saves result in
EEPROM. R is number of cycles. method 0 = classic, 1 = some overshoot, 2 = no
overshoot, 3 = pessen, 4 = Tyreus-Lyben
- M320 S<0/1> - Activate auto level, S1 stores it in eeprom
- M321 S<0/1> - Deactivate auto level, S1 stores it in eeprom
- M322 - Reset auto level matrix
- M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 =
permanent = default
- M340 P<servoId> S<pulseInUS> R<autoOffIn ms>: servoID = 0..3, Servos are
controlled by a pulse with normally between 500 and 2500 with 1500ms in center
position. 0 turns servo off. R allows automatic disabling after a while.
- M350 S<mstepsAll> X<mstepsX> Y<mstepsY> Z<mstepsZ> E<mstepsE0> P<mstespE1> :
Set micro stepping on RAMBO board
- M355 S<0/1> - Turn case light on/off, no S = report status
- M360 - show configuration
- M400 - Wait until move buffers empty.
- M401 - Store x, y and z position.
- M402 - Go to stored position. If X, Y or Z is specified, only these
coordinates are used. F changes feedrate for that move.
- M408 S<0-5> - Return status as json string (requires matching feature) for
PanelDue
- M415 S<0/1> Z<zpos> - Enables (S1) or disables (S0) rescue system. Zx can
define current Z as homed Z position.
- M416 - Simulate a power loss
- M450 - Reports printer mode
- M451 - Set printer mode to FFF
- M452 - Set printer mode to laser
- M453 - Set printer mode to CNC
- M460 X<minTemp> Y<maxTemp> : Set temperature range for thermistor controlled
fan
- M500 Store settings to EEPROM
- M501 Load settings from EEPROM
- M502 Reset settings to the one in configuration.h. Does not store values in
EEPROM!
- M513 - Clear all jam marker.
- M530 S<printing> L<layer> - Enables explicit printing mode (S1) or disables it
(S0). L can set layer count
- M531 filename - Define filename being printed
- M532 X<percent> L<curLayer> - update current print state progress (X=0..100)
and layer L
- M600 Change filament
- M601 S<1/0> B<1/0> P<1/0> - Pause extruders. B1 also pauses heated bed. Paused
extrudes disable heaters and motor. Continue (S0) reheats extruder to old temp.
P0 does not wait for target temperature.
- M602 S<1/0> P<1/0>- Debug jam control (S) Disable jam control (P). If enabled
it will log signal changes and will not trigger jam errors!
- M603 - Simulate a jam
- M604 X<slowdownSteps> Y<errorSteps> Z<slowdownTo> T<extruderId> - Set jam
detection values on a per extruder basis. If not set it uses defaults from
Configuration.h
- M606 X<distance> F<speed> - In DUAL_X_AXIS_MODE = 1 moves the current extruder
to the specified distance relative to the carriage's home position. Default X is
0 (home position).
- M666 - force communication error, required DEBUG_COM_ERRORS
- M668 - set line number 0 without notice to simulate error
- M670 S<version> - Set eeprom version to a value for testing eeprom upgrade
path.
- M908 P<address> S<value> : Set stepper current for digipot (RAMBO board)
- M999 - Continue from fatal error. M999 S1 will create a fatal error for
testing.
- M914 X<sg_value> Y<sg_value> Z<sg_value> E<sg_value> A<sg_value> B<sg_value>
Stall detection sensitivity for Trinamic stepper drivers. E = Ext0, A = Ext1, B
= Ext2
- M915 X<0/1> Y<0/1> Z<0/1> E<0/1> A<0/1> B<0/1> Turn StealthChop mode ON or OFF
on Trinamic stepper drivers. E = Ext0, A = Ext1, B = Ext2
*/

#include "Repetier.h"
#include <SPI.h>

#if UI_DISPLAY_TYPE == DISPLAY_ARDUINO_LIB
//#include <LiquidCrystal.h> // Uncomment this if you are using liquid crystal
//library
#endif

void setup() { Printer::setup(); }

void loop() { Commands::commandLoop(); }
