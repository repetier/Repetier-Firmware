# Repetier-Firmware Arduino Project Makefile
# 
# Makefile Based on:
# Arduino 0011 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
#
# This has been tested with Arduino 0022.
# 
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# Detailed instructions for using the makefile:
#
#  1. Modify the line containg "INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under Mac OS X, this
#     might be /Applications/arduino-0012).
#
#  2. Modify the line containing "PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wildcard (e.g. PORT = /dev/tty.usb*).
#
#  3. Set the line containing "MCU" to match your board's processor. 
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000.
#
#  4. Type "make" and press enter to compile/verify your program.
#
#  5. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# $Id$

ARDUINO_TAG?=1.0.5
#TARGET = $(notdir $(CURDIR))
FORMAT:=ihex
TARGET = Repetier
PDEFILE=$(TARGET).ino
#INSTALL_DIR = D:/arduino/arduino-0022
#UPLOAD_RATE = 38400
AVRDUDE_PROGRAMMER = stk500v2
#AVRDUDE_PROGRAMMER = 2232HIO
#PORT = COM10
PORT?=/dev/ttyUSB4
MCU = atmega2560
#For "old" Arduino Mega
#MCU = atmega1280
#For Sanguinololu
#MCU = atmega644p 
F_CPU = 16000000


############################################################################
# Below here nothing should be changed...

#ARDUINO = (INSTALL_DIR)/hardware/arduino/cores/arduino.lib
# apt-get download  arduino-core
# dpkg -x arduino-core2_1.0.5.deb  core
# mv core/usr/share/arduino/hardware/arduino/cores/arduino arduino.lib
# cp core/usr/share/arduino/hardware/arduino/variants/mega/pins_arduino.h arduino.lib
# echo '#include "pins_arduino.h"' > $(ARDUINO)/pins_arduino.c
# cp -a /core/usr/share/arduino/libraries/SPI/SPI.h $(ARDUINO)

# look in core/usr/share/arduino/revisions.txt, use ARDUINO def from there
ARDUINO_VER = ARDUINO=105
ARDUINO = arduino.lib

HW_SER1:=$(shell echo "$(ARDUINO_TAG)"|tr -d '.'|sed 's!$$!>165!'|bc)
ifeq ($(HW_SER1),1)
HW_SER1_SRC:=$(ARDUINO)/HardwareSerial1.cpp $(ARDUINO)/HardwareSerial0.cpp
HW_C_SRC:=$(ARDUINO)/hooks.c
endif

AVR_TOOLS_PATH = /usr/bin
#AVR_TOOLS_PATH = d:/WinAVR/bin
SRC =  $(ARDUINO)/pins_arduino.c $(ARDUINO)/wiring.c \
$(ARDUINO)/wiring_analog.c $(ARDUINO)/wiring_digital.c \
$(ARDUINO)/wiring_pulse.c $(HW_C_SRC)\
$(ARDUINO)/wiring_shift.c $(ARDUINO)/WInterrupts.c 
CXXSRC = $(ARDUINO)/HardwareSerial.cpp $(ARDUINO)/WMath.cpp \
$(ARDUINO)/Print.cpp  $(shell ls *.cpp) \
$(ARDUINO)/WString.cpp $(HW_SER1_SRC)
#$(ARDUINO)/Print.cpp ./SdFile.cpp ./SdVolume.cpp ./Sd2Card.cpp ./gcode.cpp \


# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile
GCC_VER=$(shell $(CC) --version|grep GCC|sed 's!.* !!'|tr -d '.' )
# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)
CXXDEFS = -DF_CPU=$(F_CPU)

# Place -I options here
CINCS = -I$(ARDUINO) 
CXXINCS = -I$(ARDUINO)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)

CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA) -MD -MP -D$(ARDUINO_VER)
CXXFLAGS = $(CDEFS) $(CINCS) -O$(OPT) -MD -MP -D$(ARDUINO_VER)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
#LDFLAGS = -lm -lC -Wl,-Map=applet/$(TARGET).map

# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:applet/$(TARGET).hex:i
AVRDUDE_FLAGS = -D -C $(AVRDUDE_CONF) \
-p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) 
#-b $(UPLOAD_RATE)

# Program settings
CC := $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
#AVRDUDE = $(INSTALL_DIR)/hardware/tools/avrdude
AVRDUDE = /usr/bin/avrdude
AVRDUDE_CONF = /etc/avrdude.conf
REMOVE = rm -f
MV = mv -f

VPATH=.

GCC_WORKAROUND:=$(shell $(CC) --version|grep GCC|sed 's!.* !!'|tr -d '.'|sed 's!$$!<480!'|bc)
ifeq ($(GCC_WORKAROUND),1)
LDFLAGS = -lm -lc -Wl,-Map=applet/$(TARGET).map -lc
else
LDFLAGS = -lm -Wl,-Map=applet/$(TARGET).map
endif
# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)
RELIB?=0
help:
	@echo "Targets are build, clean, cleanall, sizeafter, and upload"
	@echo "for upload, set PORT for the serial terminal"
	@echo " ex: make PORT=$(PORT) upload"
	@echo " to use a different ARDUINO tag, set ARDUINO_TAG to tag, i.e. ARDUINO_TAG=$(ARDUINO_TAG)"
	@echo " Set RELIB to just check out a differene ARDUINO tag and rebuild"

# Default target.
all:  build sizeafter
ifeq ($(RELIB),1)
build: cleanlib
endif
build: applet/$(TARGET).cpp elf hex lss

applet:
	mkdir $@

$(ARDUINO):
ifeq ($(RELIB),1)
	./get_arduino_libs.sh $(ARDUINO_TAG) 1
else
	./get_arduino_libs.sh $(ARDUINO_TAG) 
endif

fake_new_delete.c fake_new_delete.h:
	./build_fake_cplus.sh 

applet/core.a applet/$(TARGET).cpp (OBJ): | $(ARDUINO)

applet/$(TARGET).cpp: $(PDEFILE) | applet
	# Here is the "preprocessing".
	# It creates a .cpp file based with the same name as the .pde file.
	# On top of the new .cpp file comes the WProgram.h header.
	# At the end there is a generic main() function attached.
	# Then the .cpp file will be compiled. Errors during compile will
	# refer to this new, automatically generated, file. 
	# Not the original .pde file you actually edit...
	#echo '#include "WProgram.h"' > applet/$(TARGET).cpp
	cat $(PDEFILE) >> applet/$(TARGET).cpp
	cat $(ARDUINO)/main.cpp >> applet/$(TARGET).cpp

elf: applet/$(TARGET).elf
hex: applet/$(TARGET).hex
eep: applet/$(TARGET).eep
lss: applet/$(TARGET).lss 
sym: applet/$(TARGET).sym

# Program the device.  
upload: applet/$(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


	# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) applet/$(TARGET).hex
ELFSIZE = $(SIZE) -C applet/$(TARGET).elf
sizebefore:
	@if [ -f applet/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@if [ -f applet/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: applet/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr applet/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr applet/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym

%.hex:%.elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep:%.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss:%.elf
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym:%.elf
	$(NM) -n $< > $@

fake_new_delete.o: fake_new_delete.c
	$(CXX) -c -mmcu=$(MCU) -Wall --pedantic  -fno-exceptions  -Wl,--gc-sections -ffunction-sections -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ansi -o $@ $<

	# Link: create ELF output file from library.
applet/$(TARGET).elf: $(PDEFILE) applet/core.a fake_new_delete.o
	@echo "GCC_VER=$(GCC_VER) GCC_W=$(GCC_WORKAROUND)"
	$(CXX) $(ALL_CXXFLAGS) -o $@ applet/$(TARGET).cpp -L. applet/core.a fake_new_delete.o $(LDFLAGS) 
	$(ELFSIZE)

applet/core.a: $(OBJ) | applet
	@for i in $(OBJ); do echo $(AR) rcs applet/core.a $$i; $(AR) rcs applet/core.a $$i; done



# Compile: create object files from C++ source files.
%.o:%.cpp
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

# Compile: create object files from C source files.
%.o:%.c
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
%.s:%.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o:%S
	$(CC) -c $(ALL_ASFLAGS) $< -o $@



# Target: clean project.
clean:
	$(REMOVE) -r applet $(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d) 
	$(REMOVE) fake_new_delete*

cleanlib: clean
	$(REMOVE) -r arduino.lib

cleanall: clean cleanlib
	$(REMOVE) -r   ard.all

depend:
	if grep '^# DO NOT DELETE' $(MAKEFILE) >/dev/null; \
	then \
		sed -e '/^# DO NOT DELETE/,$$d' $(MAKEFILE) > \
			$(MAKEFILE).$$$$ && \
		$(MV) $(MAKEFILE).$$$$ $(MAKEFILE); \
	fi
	echo '# DO NOT DELETE THIS LINE -- make depend depends on it.' \
		>> $(MAKEFILE); \
	$(CC) -M -mmcu=$(MCU) $(CDEFS) $(CINCS) $(SRC) $(ASRC) >> $(MAKEFILE)

.PHONY:	all build elf hex eep lss sym program coff extcoff clean cleanlib cleanall depend applet_files sizebefore sizeafter
