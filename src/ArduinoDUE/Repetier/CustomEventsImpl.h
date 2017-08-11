
#include "Repetier.h"


void ParkExtruder1() {
	//TODO:RM: Move positions to config
	GCode::executeFString(PSTR("G1 X574 Y496 F6000\n"));
	GCode::executeFString(PSTR("G1 Y506 F2000 S1\n"));
	GCode::executeFString(PSTR("G1 X564 F2000\n"));
	GCode::executeFString(PSTR("G1 Y0 F6000 S0\n"));
}

void ParkExtruder2() {
	//TODO:RM: Move positions to config
	GCode::executeFString(PSTR("G1 X0 Y496 F6000\n"));
	GCode::executeFString(PSTR("G1 Y506 F2000 S1\n"));
	GCode::executeFString(PSTR("G1 X10 F2000\n"));
	GCode::executeFString(PSTR("G1 Y0 F6000 S0\n"));
}

bool ParkExtruder(uint8_t extruderId) {
	switch (extruderId)
	{
	case 1: ParkExtruder1();
		return true;
	case 2: ParkExtruder2();
		return true;
	default:
		return false;
	}
}

uint8_t GetExtruderId(GCode *com) {
	if (com->hasT() && com->T < NUM_EXTRUDER)
		return com->T;
	return 0;
}

bool CustomMCodeHandler(GCode *com)
{
  uint8_t extruderId;

	switch (com->M)
	{
	case 700:
		// Example. To park extruder 1: M700 T1
		extruderId = GetExtruderId(com);
		if (extruderId == 0)
			return false;
		return ParkExtruder(extruderId);
	default:
		return false;
	}
}






