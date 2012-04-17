Repetier-Firmware - the fast and user friendly firmware
by repetier  (repetierdev@googlemail.com)

Version 0.62  17.04.2012

1) Introduction

Repetier-Firmware is a firmware for RepRap like 3d-printer powered with
an arduino compatible controller.
This firmware is a nearly complete rewrite of the sprinter firmware by kliment
which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
Some ideas were also taken from Teacup, Grbl and Marlin.
While I changed nearly all parts, I kept the sprinter logic as far intact as
possible. As a result, all boards with at least 64kB RAM working with Sprinter
should work with this firmware. This includes Sanguino/GEN 6/Mega/RAMPS Boards.

You may ask, why did I rewrite a functioning firmware. First, let me say, there
is nothing wrong with the original Sprinter software. After studying the code,
I know how much effort was put into it, to get it running on many different
platforms. At first I just wanted to test
a different communication protocol, which is more reliable and faster
compared to the ascii method. So I invented the Repetier-Protocol, which sends
values in binary format. This reduces the size of data to less then 50% and
no conversion from ascii to float/int is needed. An improved checksum method
should make mistakes nearly impossible. While programming this, I saw some
parts that could be improved, so I did it. Then I read in the forum about
the idea of using the EEPROM for parameter. 

Supported boards:

The following boards should work by changing the configuration settings. If the 
extruder heater pin uses Timer 0 or 1, no PID control should be compiled. These
interrupts are needed for internal purposes. Other boards can made running by
adding a pin definition in pins.h

Gen 6 : Full support
Ramps 1.0 : Full support with PWM simulation (uncomment SIMULATE_PWM)
Ramps 1.1/1.2/1.3/1.4 : Full Support
Sanguinololu : Full support  with PWM simulation
Gen 7: Full support

Help wanted:

There are many kinds of RepRaps and Controller around. I can only test hardware
I have at home, so it would be great if you could tell me, if there are problems
with your combination. If you have a code patch, I will include it. If not I
will try to find a solution and send you a patch. This needs a good description
of the error, otherwise I won't find it:-(

repetierdev@googlemail.com   


2) Features

- RAMP acceleration support.
- Path planning for higher print speeds.
- Ooze prevention system for faster anti ooze then slicer can do,
- Trajectory smoothing for smoother lines.
- Nozzle pressure control for improved print quality with RAMPS.
- Fast - 30000 Hz and more stepper frequency is possible with a 16 MHz AVR. 
- Multiple extruder supported (experimental).
- Standard ASCII and improved binary (Repetier protocol) communication.
- Autodetect the command protocol, so it will work with any host software.
- Continuous monitoring of one temperature.
- Important parameters are stored in EEPROM and can easily be modified without
  recompilation of the firmware.
- Stepper control is handeled in an interrupt routine, leaving time for
  filling caches for next move.
- PID control for extruder temperature.
- Interrupt based sending buffer (Arduino library normally waits for the
  recipient to receive written data)
- Small RAM memory print, resulting in large caches.
- Supports SD-cards.
- mm and inches can be used for G0/G1
- Works with Skeinforge 41, all unknown commands are ignored.
- Dry run : Execute yout GCode without using the extruder. This way you can
  test for non-extruder related failures without actually printing.

Should work with any host software around using the ascii commands. To get the
most out of the firmware, I have written an optimized host software 
(Repetier-Host https://github.com/repetier/Repetier-Host), with the following
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

Read the wiki at https://github.com/repetier/Repetier-Firmware/wiki

especially the basic installation instructions at 

https://github.com/repetier/Repetier-Firmware/wiki/Installation

Update from older version:

Version 0.32 and 0.34 changed the EEPROM usage, because some parameter aren't 
used any more and new parameter were added. Check the settings before using
the new version. Watch out for new values, telling you they are 0 - they are 
probably not. Illegal numbers are returned as 0, too.

Version 0.40 changed the temperature tables for higher precision.

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
board has a SD card, you can copy the G-Code to the card and just tell your
RepRap to print this file. The printer will happily ignore your computer crash
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
M20          ; Will list all files on card
M23 filename ; selects the file, you want to print
M24          ; Starts/resumes the print
M30 filename ; Deletes file from sd-card.

You can send commands during print. But if you don't want to ruin your print,
send no print commands! Only monitoring commands like M105 for temperature
reading. 

5. Changelog

See changelog.txt
