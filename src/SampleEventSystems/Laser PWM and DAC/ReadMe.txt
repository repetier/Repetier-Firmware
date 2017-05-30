

files content following implementations :

----------------------------------------------------


- I2C Portexpander MCP23017

  single Pin write/read

  port write /read

-----------------------------------------------------

- External PWM controller PCA9685
  
  selectable PWM frequency from 24 Hz to 1526 Hz ( all Pins use same frequency)

  single Pin write (value 0...4095 ) logic inversion possible

-----------------------------------------------------

- external single channel DAC MAP4725

  write value (o...4095) corresponding to 0...VDD

-----------------------------------------------------

- Number of components limited by I2C addresses
  multiple components possible

------------------------------------------------------

-  control Laser via PWM 

-  control Analog Laser via DAC

------------------------------------------------------

Custom G-Codes :

G0/G1

modified to use feedrate of fastest axis for G0 moves

so there is no need to add Feedrate for G0.

after G0 move feedrate is restored to the old G1 feedrate 



ensure your maximum feedrates are set correct!!


-------------------------------------------------------
Custom M-codes :

M3

- added capability for intensity correction via Gamma curve in Laser mode
  useful for grayscale engraving 

- added base offset for Laser (this gives the ability to set a value where
  the material starts to change colour , useful for grayscale engraving

- rescaling the values to match original limits


M452

- added possibility to set Parameters used in M3

- added Gamma correction and base offset to start at where material begins to change colour


   use:  M452 C0<1> for Gamma off/on 
              K<value> for gamma curve(should be around 1...2.2) 
              P<value> for start Power
L<value> for limiting Power


 ##!!   my tests showed optimum for P at arund 10..15% of LASER_PWM MAX, as it's quite sensitive
 ##!!   i use the value, not percentage.

    example: Gamma on , Gamma 1.4 , start value 500, Limitvalue 3000

    Gcode command for this example is 

    M452 C1 K1.4 P500 L3000


--------------------------------------------------------------
Custom_Execute

  some ui-actions as example for self defined actions
  to use in  user Interface.

--------------------------------------------------------------  
Custom_CheckSlowKeys

  reading ports of  MCP23017 and assigning buttons/button combinations to actions.

  65535 different commands can be assigned to 16 buttons.

  (intended use is for example to move one axis in steps of 0.1 , 1 , 10 mm)
  
  
  some actions are written out of user interface , so no definition in UI-actions is necessary.

  also use of self defined macros is possible, define the macros following examples in CustomEvents.h

