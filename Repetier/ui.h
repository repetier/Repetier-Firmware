/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Foobar is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _ui_h
#define _ui_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#include <avr/pgmspace.h>

#include "uiconfig.h"

#define UI_VERSION_STRING "Repetier 0.50"

#if UI_DISPLAY_TYPE!=0

// Load basic language definition to make sure all values are defined
#include "uilang.h"

class UIDisplay {
  private:
    byte printCols[UI_COLS];
    byte actCol; // current col printing, 255 if row is printed
    byte menuLevel; // current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change
    byte menuPos[4]; // Positions in menu
    int pageDelay; // Counter. If 0 page is refreshed if menuLevel is 0.
    byte actRow; // Current row printing, 255 if finished
    void positionAtRow(byte r);
    void writeNibble(byte value);
    void writeByte(byte c,byte rs,byte rw);
    void lcdCommand(byte c);
    void command();
  public:
    UIDisplay();
    void initialize();
    void clear();
    void printRow(byte r,PGM_P txt);
    void slowAction();
    void fastAction();
};
extern UIDisplay uid;
#define UI_INITIALIZE uid.initialize();
#define UI_FAST uid.fastAction();
#define UI_SLOW uid.slowAction();

#else
#define UI_INITIALIZE
#define UI_FAST
#define UI_SLOW
#endif
#endif

