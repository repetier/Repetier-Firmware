
#include "Repetier.h"

uint8_t currentExtruder = -1;

bool EventUnhandledGCode(GCode *com)
{
    if (com->hasG())
    {
        switch (com->G)
        {
        case 28:
            if (com->hasX() || com->hasY())
            {
                GCode::executeFString(PSTR("G50028 Y\n"));
            }
            if (com->hasX())
            {
                GCode::executeFString(PSTR("G50028 X\n"));
            }
            if (com->hasZ())
            {
                GCode::executeFString(PSTR("G50028 Z\n"));
            }
            previousMillisCmd = HAL::timeInMilliseconds();
            return true;
            break;
        default:
            return false;
            break;
        }
        return false;
    }
    return true;
}

void SelectExtruder500XL(uint8_t t) {
    if (currentExtruder == t)
    {
        return;
    }

    currentExtruder = t;

    switch (t)
    {
    case 0:
        GCode::executeFString(PSTR("G1 X574 F6000\n"));
        GCode::executeFString(PSTR("G1 Y506 S1\n"));
        GCode::executeFString(PSTR("G1 X564\n"));
        GCode::executeFString(PSTR("G1 Y496\n"));
        GCode::executeFString(PSTR("G1 X0 S0\n"));
        GCode::executeFString(PSTR("G92 E0\n"));
        break;
    case 1:
        GCode::executeFString(PSTR("G0 X0 F6000\n"));
        GCode::executeFString(PSTR("G0 Y506 S1\n"));
        GCode::executeFString(PSTR("G0 X10\n"));
        GCode::executeFString(PSTR("G0 Y496\n"));
        GCode::executeFString(PSTR("G0 X574 S0\n"));
        GCode::executeFString(PSTR("G92 E0\n"));
        break;
    default:
        break;
    }
}
