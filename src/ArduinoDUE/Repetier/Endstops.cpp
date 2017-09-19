/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

flag8_t Endstops::lastState = 0;
flag8_t Endstops::lastRead = 0;
flag8_t Endstops::accumulator = 0;
#ifdef EXTENDED_ENDSTOPS
flag8_t Endstops::lastState2 = 0;
flag8_t Endstops::lastRead2 = 0;
flag8_t Endstops::accumulator2 = 0;
#endif

void Endstops::update() {
	flag8_t newRead = 0;
	#ifdef EXTENDED_ENDSTOPS
	flag8_t newRead2 = 0;
	#endif
	#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
	if(READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING)
	newRead |= ENDSTOP_Y_MIN_ID;
	#endif
	#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
	if(READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING)
	newRead |= ENDSTOP_Y_MAX_ID;
	#endif
	#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
	if(READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING) {
		newRead |= ENDSTOP_X_MIN_ID;
	}
	#endif
	#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
	if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING)
	newRead |= ENDSTOP_X_MAX_ID;
	#endif
	#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
	if(READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING)
	newRead |= ENDSTOP_Z_MIN_ID;
	#endif
	#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
	if(READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING)
	newRead |= ENDSTOP_Z_MAX_ID;
	#endif
	#if (Z2_MINMAX_PIN > -1) && MINMAX_HARDWARE_ENDSTOP_Z2
	if(READ(Z2_MINMAX_PIN) != ENDSTOP_Z2_MINMAX_INVERTING)
	newRead |= ENDSTOP_Z2_MINMAX_ID;
	#endif
	#if FEATURE_Z_PROBE
	#if Z_PROBE_PIN == Z_MIN_PIN && MIN_HARDWARE_ENDSTOP_Z
	if(newRead & ENDSTOP_Z_MIN_ID) // prevent different results causing confusion
	newRead |= ENDSTOP_Z_PROBE_ID;
	if(!Printer::isHoming())
	newRead &= ~ENDSTOP_Z_MIN_ID; // could cause wrong signals depending on probe position
	#else
	if(Z_PROBE_ON_HIGH ? READ(Z_PROBE_PIN) : !READ(Z_PROBE_PIN))
	newRead |= ENDSTOP_Z_PROBE_ID;
	#endif
	#endif
	#ifdef EXTENDED_ENDSTOPS
	#if HAS_PIN(Y2_MIN_PIN) && MIN_HARDWARE_ENDSTOP_Y2
	if(READ(Y2_MIN_PIN) != ENDSTOP_Y2_MIN_INVERTING)
	newRead2 |= ENDSTOP_Y2_MIN_ID;
	#endif
	#if HAS_PIN(Y2_MAX_PIN) && MAX_HARDWARE_ENDSTOP_Y2
	if(READ(Y2_MAX_PIN) != ENDSTOP_Y2_MAX_INVERTING)
	newRead2 |= ENDSTOP_Y2_MAX_ID;
	#endif
	#if HAS_PIN(X2_MIN_PIN) && MIN_HARDWARE_ENDSTOP_X2
	if(READ(X2_MIN_PIN) != ENDSTOP_X2_MIN_INVERTING) {
		newRead2 |= ENDSTOP_X2_MIN_ID;
	}
	#endif
	#if HAS_PIN(X2_MAX_PIN) && MAX_HARDWARE_ENDSTOP_X2
	if(READ(X2_MAX_PIN) != ENDSTOP_X2_MAX_INVERTING)
	newRead2 |= ENDSTOP_X2_MAX_ID;
	#endif
	#if HAS_PIN(Z2_MAX_PIN) && MAX_HARDWARE_ENDSTOP_Z2
	if(READ(Z2_MAX_PIN) != ENDSTOP_Z2_MAX_INVERTING)
	newRead2 |= ENDSTOP_Z2_MAX_ID;
	#endif
	#if HAS_PIN(Z3_MAX_PIN) && MAX_HARDWARE_ENDSTOP_Z3
	if(READ(Z3_MAX_PIN) != ENDSTOP_Z3_MAX_INVERTING)
	newRead2 |= ENDSTOP_Z3_MAX_ID;
	#endif
	#if HAS_PIN(Z3_MIN_PIN) && MIN_HARDWARE_ENDSTOP_Z3
	if(READ(Z3_MIN_PIN) != ENDSTOP_Z3_MIN_INVERTING)
	newRead2 |= ENDSTOP_Z3_MIN_ID;
	#endif

	#endif
	InterruptProtectedBlock noInts; // bad idea to run this from different interrupts at once!
	lastRead &= newRead;
	#ifdef EXTENDED_ENDSTOPS
	lastRead2 &= newRead2;
	#endif // EXTENDED_ENDSTOPS
	if(lastRead != lastState
	#ifdef EXTENDED_ENDSTOPS
	|| (lastState2 != lastRead2)
	#endif
	) { // Report endstop hit changes
		lastState = lastRead;
		accumulator |= lastState;
		#ifdef EXTENDED_ENDSTOPS
		lastState2 = lastRead2;
		accumulator2 |= lastState2;
		#endif
		if (Printer::debugEndStop())  Endstops::report();
		} else {
		lastState = lastRead;
		#ifdef EXTENDED_ENDSTOPS
		lastState2 = lastRead2;
		#endif
	}
	lastRead = newRead;
	#ifdef EXTENDED_ENDSTOPS
	lastRead2 = newRead2;
	#endif
}

void Endstops::report() {
	Com::printF(PSTR("endstops hit: "));
	#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
	Com::printF(Com::tXMinColon);
	Com::printF(xMin() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
	Com::printF(Com::tXMaxColon);
	Com::printF(xMax() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
	Com::printF(Com::tYMinColon);
	Com::printF(yMin() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
	Com::printF(Com::tYMaxColon);
	Com::printF(yMax() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
	Com::printF(Com::tZMinColon);
	Com::printF(zMin() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
	Com::printF(Com::tZMaxColon);
	Com::printF(zMax() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if (Z2_MINMAX_PIN > -1) && MINMAX_HARDWARE_ENDSTOP_Z2
	Com::printF(PSTR("z2_minmax:"));
	Com::printF(z2MinMax() ? Com::tHSpace : Com::tLSpace);
	#endif
	#if FEATURE_Z_PROBE
	Com::printF(Com::tZProbeState);
	Com::printF(zProbe() ? Com::tHSpace : Com::tLSpace);
	#endif
	Com::println();
}
