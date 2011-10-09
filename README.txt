Repetier-Firmware - the fast and user friendly firmware
by repetier  (repetierdev@googlemail.com)

Version 0.35  9.10.2011

1) Introduction

Repetier-Firmware is a firmware for RepRap like 3d-printer powered with
an arduino compatible controller.
This firmware is a nearly complete rewrite of the sprinter firmware
by kliment (https://github.com/kliment/Sprinter)
which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
Some ideas were also taken from Teacup, Grbl and Marlin.
While I changed nearly all parts, I kept the sprinter logic as far intact as
possible. As a result, all boards with at least 64kB RAM working with Sprinter
should work with this firmware. This includes Saguino/GEN 6/Mega/RAMPS Boards.

You may ask, why did I rewrite a functioning firmware. First, let me say, there
is nothing wrong with the original Sprinter software. After studying the code,
I know how much effort was put into it, to get it running on many different
platforms. At first I yust wanted to test
a different communication protocol, which is more reliable and faster
compared to the ascii method. So I invented the Repetier-Protocol, which sends
values in binary format. This reduces the size of data to less then 50% and
no conversion from ascii to float/int is needed. An improved checksum method
should make mistakes nearly impossible. While programming this, I saw some
parts that could be improved, so I did it. Then I read in the forum about
the idea of using the EEPROM for parameter. 

Help wanted:

There are many kinds of RepRaps and Controller around. I can only test hardware
I have at home, so it would be great if you could tell me, if there are problems
with your combination. If you have a code patch, I will include it. If not I
will try to find a solution and send you a patch. This needs a good description
of the error, otherwise I won't find it:-(

I'd like to include working configurations for many Printer/Controller/Extruder
combinations. This way a novice can copy the best matching configuration data.
It's likely, that the Configuration.h file will change in the future and there
are quite a lot of possible combinations. As a solution, I will include a
file for each machine/controller/extruder, so everyone can lookup the parameters
coming from this. E.g. for a mendel this would contain:

For Stepper controller with 1/8 step, metric system
#define XAXIS_STEPS_PER_MM 40
#define YAXIS_STEPS_PER_MM 40
#define ZAXIS_STEPS_PER_MM 3360

which are all parameter affected by the printer type. If you have some 
parameters, which are not included, please send them 
to repetierdev@googlemail.com   

2) Features

- RAMP acceleration support.
- Path planning for higher print speeds.
- Ooze prevention system for faster anti ooze then slicer can do,
- Trajectory smoothing for smoother lines.
- Nozzle pressure control for improved print quality with RAMPS.
- Fast - 20000 Hz and more stepper frequency is possible with a 16 MHz AVR. 
- Multiple extruder supported (experimental).
- Standard ASCII and improved binary (Repetier protocol) communication.
- Autodetect the command protocol, so it will work with any host software.
- Continuous monitoring of one temperature.
- Important parameters are stored in EEPROM and can easyly modified without
  recompilation of the firmware.
- Stepper control is handeled in an interrupt routine, leaving time for
  filling caches for next move.
- PID control for extruder temperature.
- Interrupt based sending buffer (Arduino library normally waits for the
  recipient to receive written data)
- Small RAM memory print, resulting in large caches.
- Supports SD-cards (experimental, don't have a card to test it)
- mm and inches can be used for G0/G1
- Works with Skeinforge 41, all unknown commands are ignored.
- Dry run : Execute yout GCode without using the extruder. This way you can
  test for non-extruder related failures without actually printing.

Should work with any host software around using the ascii commands. To get the
most out of the firmware, I have written an optimized host software 
(repetier-host https://github.com/repetier/Repetier-Host), with the following
main features:
- ASCII and repetier protocol
- Temperature monitor (for Repetier-Firmware)
- EEPROM parameter configuration (for Repetier-Firmware)
- STL composer, so you can arrange your STL files and slice them all together.
- Skeinforge integration. You can load a STL file and it will be converted into
  GCode by Skeinforge. 
- Estimation of printing time.
- Control panel to move your printer head, view/set temperature etc.

3) Installation

The software was developed with Ardiuno 0022. Older versions should work, if
your board is supported. Start the Arduino-IDE and load the Repetier.pde file.
The IDE will open all other files in that directory, too. Select the file
"Configuration.h" and edit the paramaters, to fir your RepRap. All parameters
are documented in the file, so no further explanation is required.
Connect your board, select the correct COM-Port under Tools->Serial Port and
the correct Board under Tools->Board. Klick the "Upload" button. After a while
your new firmware should be uploaded and ready to start. If the upload doesn't
start, after the files are compiled, try pressing the reset button on your
board.

Update from older version:

Version 0.32 and 0.34 changed the EEPROM usage, because some parameter aren't 
used any more and new parameter were added. Check the settings before using
the new version. Watch out for new values, tellung you they are 0 - they are 
probably not. Illegal numbers are returned as 0, too.

4) Things you should know

4.1 Continuous temperature monitoring

Send
M203 S0    for temperature of extruder 0
M203 S1    for temperature of extruder 1
M203 S100  for temperature of heated bed
M203 S255  disable temperture monitoring

Afterwards, on every temperature event (approx. 4 times per second) you get
a response line like
MTEMP:time current_temperature target_temperature output

The main advatage against M105 is, that no command needs to be send. In addition
it will give some informations, you won't get with M105. In the Repetier-Host
this is used, to monitor a temperature continuously and draw a graph with
temperature/output (go Windows->Temperature monitor). This helps adjusting the 
PID parameters.

4.2 Changing EEPROM variables

If you compiled the firmware with EEPROM support, you can change all important
parameters, like start speed, acceleration, steps per mm using two commands.
Send "M205" to your RepRap and it will show all parameters with position, type
and value. The output will look like this:
EPR:2 75 76800 Baudrate
EPR:2 79 0 Max. inactive time [ms,0=off]
EPR:2 83 60000 Stop stepper afer inactivity [ms,0=off]
EPR:3 3 40.00 X-axis steps per mm
EPR:3 7 40.00 Y-axis steps per mm
EPR:3 11 3333.59 Z-axis steps per mm
EPR:3 15 20000.00 X-axis max. feedrate [mm/min]
EPR:3 19 20000.00 Y-axis max. feedrate [mm/min]
EPR:3 23 2.00 Z-axis max. feedrate [mm/min]
EPR:3 27 1500.00 X-axis homing feedrate [mm/min]
EPR:3 31 1500.00 Y-axis homing feedrate [mm/min]
EPR:3 35 100.00 Z-axis homing feedrate [mm/min]
EPR:3 39 20.00 X-axis start speed [mm/s]
EPR:3 43 20.00 Y-axis start speed [mm/s]
EPR:3 47 1.00 Z-axis start speed [mm/s]
EPR:3 51 750.00 X-axis acceleration [mm/s^2]
EPR:3 55 750.00 Y-axis acceleration [mm/s^2]
EPR:3 59 50.00 Z-axis acceleration [mm/s^2]
EPR:3 63 750.00 X-axis travel acceleration [mm/s^2]
EPR:3 67 750.00 Y-axis travel acceleration [mm/s^2]
EPR:3 71 50.00 Z-axis travel acceleration [mm/s^2]
EPR:3 150 373.00 Extr. steps per mm
EPR:3 154 1200.00 Extr. max. feedrate [mm/min]
EPR:3 158 10.00 Extr. start feedrate [mm/s]
EPR:3 162 10000.00 Extr. acceleration [mm/s^2]
EPR:0 166 1 Heat manager [0-1]
EPR:0 167 130 PID drive max
EPR:2 168 300 PID P-gain [*0.01]
EPR:2 172 2 PID I-gain [*0.01]
EPR:2 176 2000 PID D-gain [*0.01]
EPR:0 180 200 PID max value [0-255]
EPR:2 181 0 X-offset [steps]
EPR:2 185 0 Y-offset [steps]
EPR:2 189 40 Temp. stabilize time [s]
The first value is the data type (0=byte, 1=16 bit int,2=32 bit int,3 = float).
The second parameter is the position in EEPROM, the value is stored.
The third parameter is the current value. The rest of the line is a short
description of the parameter.
To change one of these parameters, just send
M206 type position new_value
Type and position must match the values returned by M205, otherwise you will
overwrite some other data with undeterminable results.
Changes are best done with a host software supporting this directly. Currently
only Repetier-Host supports this natively.

4.3 SD card

Not only a RepRap can have an defect, but also your Computer may do things
causing your print to stop (power loss, reboot after update, ...). If your
board has a SD card, you can copy the gcode to the card and just tell your
RepRap to print this file. The printer will happyly ignore your computer crash
until your print is finished:-)

Caution: Only remove the card, if it is unmounted (M22)
         Use it only, if mounted (on startup or M21)

You start by copying the file to your card. You can do this by copying it to the
card, while inserted in your computer, or by sending it to the RepRap. Select
only filenames with 8.3 notion. Longer filenames are not supported. To push a
file to your printer, execute the following steps:
M28 filename
<Send your gcode file>
M29

Printing from SD card:
M20  ! Will list all files on card
M23 filename ! selects the file, you want to print
M24 ! Starts/resumes the print

You can send commands during print. But if you don't want to ruin your print,
send no print commands! Only monitoring commands like M105 for temperature
reading. 

4.4 PID temperature control

For PID control, you have to modify the following parameters:

/** Type of heat manager for this extruder. 
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better, but needs an output with PWM, which doesn't
      use Timer 0 and 1
*/
#define EXT0_HEAT_MANAGER 1
/** The maximum value, I-gain can contribute to the output. */
#define EXT0_PID_INTEGRAL_DRIVE_MAX 130
/** P-gain in 0,01 units */
#define EXT0_PID_PGAIN   300
/** I-gain in 0,01 units */
#define EXT0_PID_IGAIN   2
/** Dgain in 0,01 units. */
#define EXT0_PID_DGAIN 2000
// maximum time the heater is can be switched on. Max = 255
#define EXT0_PID_MAX 200

The first EXT0_HEAT_MANAGER is easy, set it to 1. For the rest, you need some
background information, how PID works. I will give here a simple explanation
sufficient for tweaking the parameter. 
If we only enable the heater, if our temperature is below target temperature,
we will get a sine curve around the target temperature of about 10°. The exact
value depends on the latency of your hot end. This can be avoided by setting
the heater not to full power, but to some value, which is normally unknown and
can even change, if you increase/decrease your printing speed. The PID
algorithm tries to iterate to this value, resulting in a nearly constant
temperature. To tweak this, you have 3 parameters P, I and D. At every iteration
an error is computed:
error = targetTemp - currentTemp
from this, we get the P term
P = PGAIN*error
You see, the P term increases the power, if the error is large and decreases it,
if we are coming near our target. It is used, to get quickly into the range we
want, but it can't reach the wanted temperature alone.
For this, we have an integral part, the I term. It uses a help variable Istate,
which adds all previous errors:
Istate = Istate+error
from this, we get the I term
I = IGAIN*Istate
At the worming up phase, we have a long perod of high errors, which can cause
the Istate to get quite high. This is prevented by the 
EXT0_PID_INTEGRAL_DRIVE_MAX parameter, which limits the I term to+/- it's value
by reducing the error. Set this to 10+needed output for target temperature.
After some time, the I term is the dominant value, determining the output.
The last part is the damping term D, which tries to prevent an overshoot.
This value needs a smooth temperature reading to work correct. Unfortunately,
many sensors scatter some degrees. The firmware smoothes the temperature, to
compensate this, but it may still be visible. As a result, high dampings are
no good idea. I use 20% (2000) for my hot end, which works. The D term is
computed as:
D = DGain * (oldTemp-currentTemperature)
As you can see, it will alwys slow down the movement into the needed direction.
The faster you are moving onto the target temperature, the more it will damp.

Adding the values gives the PID value:
PID = P+I+D
This value is written as PWM signal to the heater. As you see, it is essential,
that the output pin has a timer attached, which is not timer 0 and not timer 1
(they are used for other functions). Valid values are 0-255. You can limit the
output with the EXT0_PID_MAX value.  

5. Changelog

4.9.2011 First public version
