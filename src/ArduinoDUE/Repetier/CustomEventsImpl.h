
#include "Repetier.h"

int currentExtruder = -1;

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
    // if (com->hasM())
    // {
    //     switch (com->M)
    //     {
    //     case -1:
    //         //  do something
    //         return true;
    //         break;
    //     default:
    //         return false;
    //         break;
    //     }
    //     return false;
    // }
    if (com->hasT())
    {
        if (currentExtruder == (int)com->T)
        {
            return true;
        }
        else
        {
            currentExtruder = (int)com->T;
        }
        switch (com->T)
        {
        case 0:
            GCode::executeFString(PSTR("G1 X574 F6000\n"));
            GCode::executeFString(PSTR("G1 Y506 S1\n"));
            GCode::executeFString(PSTR("G1 X564\n"));
            GCode::executeFString(PSTR("G1 Y496\n"));
            GCode::executeFString(PSTR("G1 X0 S0\n"));
            GCode::executeFString(PSTR("G92 E0\n"));
            return true;
            break;
        case 1:
            GCode::executeFString(PSTR("G0 X0 F6000\n"));
            GCode::executeFString(PSTR("G0 Y506 S1\n"));
            GCode::executeFString(PSTR("G0 X10\n"));
            GCode::executeFString(PSTR("G0 Y496\n"));
            GCode::executeFString(PSTR("G0 X574 S0\n"));
            GCode::executeFString(PSTR("G92 E0\n"));
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
