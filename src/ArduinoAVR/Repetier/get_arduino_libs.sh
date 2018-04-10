#!/bin/bash
ARD_VER="1.0.5"
LAST_OLD="1.0.6"

#Which struct we use

# This is to get the files we need from Arduino to build.
URL="https://github.com/arduino/Arduino.git"
# backup
# URL="https://github.com/rickyrockrat/Arduino.git"
odir="arduino.lib"
idir="ard.all"
# Grab the huge repo
clone_arduino () {
	git clone $URL ard.all
	if [ $? -ne 0 ]; then
		echo "Clone of $URL failed"
		exit 1
	fi
	cd ard.all
	git checkout $ARD_VER
	cd ..
}
# Grab the 236M beastie
get_arduino_tag () {
	wget -O ard.zip "https://github.com/arduino/Arduino/archive/$ARD_VER.zip"
	# unzips to Arduino-1.0.5
	unzip ard.zip
	idir="Arduino-$ARD_VER"
}
# Just get the few files we need. 
get_short_arduino_libs() {
	URL="https://github.com/rickyrockrat/arduino_short.git"
	clone_arduino
}
get_short_arduino_libs
mkdir $odir
#latest version paths:
#cp ard.all/hardware/arduino/avr/cores/arduino/* $odir
#cp ard.all/hardware/arduino/avr/variants/mega/* $odir
#cp ard.all/hardware/arduino/avr/libraries/SPI/src/* $odir
L=$(echo "$LAST_OLD"|tr -d '.')
A=$(echo "$ARD_VER"|tr -d '.')
let l=L
let a=A
if [ $a -gt $l ]; then
	PA="ard.all/hardware/arduino/avr/cores/arduino"
	PM="ard.all/hardware/arduino/avr/variants/mega"
	PS="ard.all/hardware/arduino/avr/libraries/SPI/src/SPI.h"
else
	PA="ard.all/hardware/arduino/cores/arduino"
	PM="ard.all/hardware/arduino/variants/mega"
	PS="ard.all/libraries/SPI/SPI.h"
fi

cp -a $PA/* $odir
cp $PM/* $odir
cp $PS $odir 
echo '#include "pins_arduino.h"' > $odir/pins_arduino.c

