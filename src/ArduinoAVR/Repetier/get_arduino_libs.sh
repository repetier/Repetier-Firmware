#!/bin/bash
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
	git checkout 1.0.5
	cd ..
}
# Grab the 236M beastie
get_arduino_tag () {
	wget -O ard.zip 'https://github.com/arduino/Arduino/archive/1.0.5.zip'
	# unzips to Arduino-1.0.5
	unzip ard.zip
	idir="Arduino-1.0.5"
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

cp -a ard.all/hardware/arduino/cores/arduino/* $odir
cp ard.all/hardware/arduino/variants/mega/* $odir
cp ard.all/libraries/SPI/SPI.h $odir 
echo '#include "pins_arduino.h"' > $odir/pins_arduino.c

