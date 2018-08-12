#!/bin/bash
# Version of Arduino we want to build against. 1.0.5 is known to work.
ARD_VER="1.0.6"

# Do not change the variables below. 
# These mark the point where the directory structure changed.
LAST_OLD="1.0.6"
SPI_SRC="1.6.8"

if [ -n "$1" ]; then
	ARD_VER="$1"
fi
RE_CHECK=0
if [ -n "$2" ]; then # they want to check out another tag, just do the copy.
	RE_CHECK=1
fi
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
	#URL="https://github.com/rickyrockrat/arduino_short.git"
	URL="https://github.com/rickyrockrat/Arduino.hardware.git"
	clone_arduino
}
if [ "1" = "$RE_CHECK" ]; then
	rm -rf "$odir"
	if [ ! -e "$idir" ]; then
		get_short_arduino_libs
	else
		cd ard.all
		git checkout $ARD_VER
		cd ..
	fi
else
	get_short_arduino_libs
fi
mkdir $odir
#latest version paths:
#cp ard.all/hardware/arduino/avr/cores/arduino/* $odir
#cp ard.all/hardware/arduino/avr/variants/mega/* $odir
#cp ard.all/hardware/arduino/avr/libraries/SPI/src/* $odir
L=$(echo "$LAST_OLD"|tr -d '.')
A=$(echo "$ARD_VER"|tr -d '.')
S=$(echo "$SPI_SRC"|tr -d '.')

let l=L
let a=A
let s=S

if [ $a -gt $l ]; then
	PA="ard.all/hardware/arduino/avr/cores/arduino"
	PM="ard.all/hardware/arduino/avr/variants/mega"
	if [ $a -ge $s ]; then 
		PS="ard.all/hardware/arduino/avr/libraries/SPI/src/SPI.h"
	else
		PS="ard.all/hardware/arduino/avr/libraries/SPI/SPI.h"
	fi
else
	PA="ard.all/hardware/arduino/cores/arduino"
	PM="ard.all/hardware/arduino/variants/mega"
	PS="ard.all/libraries/SPI/SPI.h"
fi

cp -a $PA/* $odir
cp $PM/* $odir
cp $PS $odir 
echo '#include "pins_arduino.h"' > $odir/pins_arduino.c

