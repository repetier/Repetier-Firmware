# Repetier-Firmware - the fast and user friendly firmware

## Configuration and Installation

Please read [our documentation](https://docfirmwarev2.repetier.com/config/introduction/)
for detailed help.

## The new Dimension of 3d printing. The ultimate upgrade for your printer!

With Repetier-Firmware 2.0 you will get these benefits you'll never want to miss again:
* Better prints with smoother surface
* Faster printing speed
* Extreme vibration and printing noise reduction
* Simpler firmware configuration
* Easily expandable due to modular design
* Code is much easier to understand
* Same handling for all printer types - no difference between linear and nonlinear systems
* Less memory usage compared to old nonlinear handling
* Easier to adjust z correction, babystepping and advance
* No more delays for slower drivers wasting CPU time
* Most special cases can now be catched by the module system
Support of up to 7 axes (X, Y, Z, E, A, B, C)

**The significant improvement of your printing experience is made possible by our new printing algorithm, which means that the print head is no longer using linear velocity profiles. Now we use ultra smooth 5th order polynominals to get a smooth velocity curve. This reduces printer vibrations and gives a better surface.**

<div style="max-width:505px;margin: 0 auto;"><object data="https://www.repetier.com/images/polynomalvelocityprofile.svg" type="image/svg+xml"><img src="https://www.repetier.com/images/polynomalvelocityprofile.svg"></object></div>

Although the development and testing of our firmware takes thousands of hours of development time and requires a lot of expensive hardware, we offer this freely accessible as open source software. However, this is only possible if you support us. If you are enthusiastic about our firmware and want to have it updated in the future, support us with an amount you think is appropriate.

Many open source projects are discontinued or do not offer support, because the financing is not sufficient. We do not want to belong to this group! We think this is especially important for manufacturers, who deploy their printers with our firmware. Imagine at one point you get no more updates or help for the basic OS of your printer! So if you use our free software to generate good sales, it would only be fair and clever if you regularly support us. If you are interested in supporting us, you can donate via <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=UL49NW2JVQ98S" target="_blank">PayPal</a> or <a href="https://www.repetier.com/about-us/" target="_blank">contact us</a>.

## Notes for developers/pull requests

This software is open source licensed under the GPL V3. As any free project we
like contributions from other sources, especially since the firmware is very
hardware related and many things can only be implemented/tested with the right
hardware. To allow easy integration of new features you should follow some simple
principals.
1. Only send pull requests against development version. This is where we add
and test new features and bug fixes. From time to time we push these to master
as a new version.
2. Do not include your personal configuration files. If you need new configuration
options, add them to the official configuration.h file.
3. We have two folders for different processor architectures. So for most
changes modifications need to be in both folders. The general files are identical.
In fact we develop on avr and just copy them to due. Only pins.h/fastio.h/hal.*
and configuration.h are architecture dependend, so chnages there need to be made
twice and not copied.
4. Document what your pull request will change/fix/introduce. Please also mention
new configurations since we need to add them also to our online tool, so users
can set them correctly.

1.0.x Is in maintenence mode. It will only get important changes and bug fixes.
Fixes for this will be done in the dev branch and
moved to stable after a while as 1.0.x patch updates. 
2.0 in the branch dev2 is where all the new features will be added. This is a complete overhauled
firmware and works completely different to 1.x version. 

### Version 2.0 information

#### Status

The version works already very stable in many parts. But is an ongoing big change so
between updates there might be new features and configuration changes required.
The supported features documented should work without problems. So if your printer
only needs them, you can already switch. Just make sure to test the update before using
in production.

Supported CPU types:
- Due based boards (SAM3x8E)
- SAMD51 (Adafruit Grand Central Metro)
- STM32F4xxx (RUMBA32, stacker3d_super_mini)
- STM32F1xxx (SKR Mini E3 V1.2 and V2.0)

Done:
- New motion system and planner for better flexibility.
- Pure cartesian printer system.
- Core XYZ printer system.
- Dual X axis printer type
- Delta printer type
- Linear, cubic and quintic velocity shape.
- No minimum speed required any more.
- Homing.
- new analog value reading.
- Hardware PWM.
- Endstop detection via interrupts.
- Tools instead of extruders.
- Linear advance
- Upgraded to SdFat 1.0.14 for better name handling.
- Ditto printing/ ditto mirror
- Mirror motors (2 - 4).
- Display support for 20x4 char and 128x64 pixel displays.
- New light system.
- Buffered tones.
- Automatic delta leveling

ToDo:
- Add more preconfigured boards and displays
- Port AVR HAL to new system
- Add LPC1768 support
- Add SAMD21 HAL
- LCD solution for filament change
- recheck event system
- Multi language support
- New config tool

#### Benefits of new system

We tested a cartesian printer on a Arduino DUE. The limit for the motion planner
was at 250000 Steps/s with precomputed segments of 1ms length using 5th grade
velocity profiles. At this speed it makes a big difference if endstops get tested
or not. 250kHz is including end stop tests. Without we could go even higher.

#### What does module system mean

The idea is that all required basic functions become a own classes with
base classes or alternative implementations with the same function/variable
names.

Imagine the old system with the inputs. Digital inputs come in 4 versions depending
on the hardware used. High signal as high or low and with or without pullup. So
every function had used many define and if conditions to always us ethe right one.

With the new system a input pin becomes a class with inline get function that
returns a normalized state, has proper initialization. So instead of every function
handling all 4 cases we have now 4 different classes that to the user all
behave identical.

Same for output pins which might need to be inverted or not. Now we have 2 classes
handling the inversion internally.

Now think about the stepper driver. That is a base class whcih gets consumed
by motion routines. They normally require 3 output pins but some are inverted
depending on connector and driver. No problem, just input the proper output class,
stepper driver does not need to know about this.

You want 2 motors for Z axis? No problem use a stepper driver that mirrors signals
to 2 drivers you configured. Motion function just need to know about one of them.

You see where this leads. We split the firmware into many blocks and you can
simply combine modules to match your printer. And because all parts are normalized
it becomes easy to add new function. You need a new stepper driver that can set
microsteps? Just add a new derivative and use that instead. No big fiddeling
at dozens of points where you might need extra cases.

The complete firmware becomes more compact and easier to maintain and easier
to extend.

![Building Blocks](doc/images/BuildingBlocks.png)

#### Developing

If you want to start developing on V2, you will not get happy with the
Arduino IDE. It will not show most files at all, also it will still compile
all files. In fact our solution does not even compile using the 
 Arduino IDE.  Best solution for developing is to use Visual Studio Code with
the following plugins:
- Platform IO
- Clang-Format (for formatting, style guid is included in sources) 

Platform IO also works stand alone or with other editors, but we will not discuss
these solutions.

If you have clang-format installed (requires llvm compile to be installed)
it will format the sources according to our styling definition on save.

To test compile press Ctrl+Alt+R and it will compile the firmware.
In the bottom right you see selected port and board. Select the right one
and upload your new firmware with Ctrl+Alt+U.

If you have once setup the system, you will never look back to the Arduino-IDE.

## Installation

For version 2.0 there is currently no configuration tool, so you need to edit
Configuration.h and COnfiguration_io.h manually.
You can compile it using Visual Studio Code with PlatformIO extension.

Find the full documentation for the new version here:
[https://docfirmwarev2.repetier.com/](https://docfirmwarev2.repetier.com/)


## Controlling firmware

Also you can control the firmware with any reprap compatible host, you will only get
the full benefits with the following products, which have special code for this
firmware:

* [Repetier-Host for Windows/Linux](http://www.repetier.com/download/)
* [Repetier-Host for Mac](http://www.repetier.com/download/)
* [Repetier-Server](http://www.repetier.com/repetier-server-download/)

## Installation

For documentation and installation please visit 
[http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/).

## Changelog

See changelog.txt
