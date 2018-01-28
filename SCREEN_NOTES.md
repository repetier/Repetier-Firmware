# Wiring Melzi screen to RAMPS

I located the pinout of the Melzi screen connector and found that the Ramps has a 10-pin header spot labeled Aux-2 which is nearly perfect.
The only issue is that GND and +5 are swapped, so to make this work you have to make a custom ribbon connector which reverses the two on the ramps side.
What that means is that on the display side, the cable looks normal and on the other side the red stripe is the second connector, not the first.
The remainder of the pinout is resolved in the firmware configuration, and everthing operates very well.

## Screen pinout

```
	  1 LCD CS    2 Encoder B
	  3 LCD DATA  4 Encoder A
KEY	5 LCD SCLK  6 Enc Pushbutton
    7 ESTOP     8 Beeper
    9 VCC (5v) 10 GND
             |
            \|/
         Red stripe
```		  

## Screen to RAMPS conversion 
```
Screen  Melzi  Ramps		Mega 2560 Description
1       D17    A11/D65  Chip Select CS / CSE / RS	
2       A1/30  A12/D66  Data 1 / ENC B				
3       D16    D42      DN / MOSI / DATA / Display Enable
4       A2/29  D44      Data 2 / ENC A					
5       D11    D40      Clock / CLK / SCLK / D4 PIN		
6       A3/28  A10/D64  Data 3 / Button / Click			
7       D10    A9/D63   ESTOP / Reset Pin				
8       A4/27  A5/D59   Beep							
9       +5vdc  GND      SOLUTION: Create a 10-pin cable which swaps the GND and +5V pins on one side
10      GND    +5vdc
```

## Misc notes on motor wire order:
Melzi: Black Green Blue Red
Ramps: Red Blue Green Black
