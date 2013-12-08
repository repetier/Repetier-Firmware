#!/bin/sh

BDF2U8G=~/Downloads/u8glib-fd2f25626a10/u8glib-fd2f25626a10/tools/font/bdf2u8g/bdf2u8g
FONT_PATH=../src/ArduinoAVR/Repetier


#5x7 font
$BDF2U8G -b 0 -e 255 Repetier-5x7-7.bdf repetier_5x7 $FONT_PATH/Font_Repetier_5x7.h
#6x10 font
$BDF2U8G -b 0 -e 255 Repetier-6x10-10.bdf repetier_6x10 $FONT_PATH/Font_Repetier_6x10.h

#update header file
perl -pi -e 's/\"u8g.h\"/<u8g.h>/g' $FONT_PATH/Font_Repetier_5x7.h
perl -pi -e 's/\"u8g.h\"/<u8g.h>/g' $FONT_PATH/Font_Repetier_6x10.h

