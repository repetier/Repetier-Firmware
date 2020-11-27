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

#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

class Commands {
public:
    static void commandLoop();
    static void checkForPeriodicalActions(bool allowNewMoves);
    static void processArc(GCode* com);
    static void processGCode(GCode* com);
    static void processMCode(GCode* com);
    static void executeGCode(GCode* com);
    static void waitUntilEndOfAllMoves();
    static void waitUntilEndOfAllBuffers();
    static void waitMS(uint32_t wait);
    static void printTemperatures();
    static void changeFeedrateMultiply(int factorInPercent);
    static void changeFlowrateMultiply(int factorInPercent);
    static void reportPrinterUsage();
    static void emergencyStop();
    static void checkFreeMemory();
    static void writeLowestFreeRAM();

private:
    static int lowestRAMValue;
    static int lowestRAMValueSend;
};

// #include "Repetier.h"

void GCode_0_1(GCode* com);
void GCode_2_3(GCode* com);
void GCode_4(GCode* com);
void GCode_10(GCode* com);
void GCode_11(GCode* com);
void GCode_20(GCode* com);
void GCode_21(GCode* com);
void GCode_28(GCode* com);
void GCode_29(GCode* com);
void GCode_30(GCode* com);
void GCode_31(GCode* com);
void GCode_32(GCode* com);
void GCode_33(GCode* com);
void GCode_90(GCode* com);
void GCode_91(GCode* com);
void GCode_92(GCode* com);
void GCode_100(GCode* com);
void GCode_131(GCode* com);
void GCode_132(GCode* com);
void GCode_133(GCode* com);
void GCode_134(GCode* com);
void GCode_135(GCode* com);
void GCode_201(GCode* com);
void GCode_202(GCode* com);
void GCode_203(GCode* com);
void GCode_204(GCode* com);
void GCode_205(GCode* com);

void MCode_3(GCode* com);
void MCode_4(GCode* com);
void MCode_5(GCode* com);
void MCode_6(GCode* com);
void MCode_7(GCode* com);
void MCode_8(GCode* com);
void MCode_9(GCode* com);
void MCode_17(GCode* com);
void MCode_18(GCode* com);
void MCode_20(GCode* com);
void MCode_21(GCode* com);
void MCode_22(GCode* com);
void MCode_23(GCode* com);
void MCode_24(GCode* com);
void MCode_25(GCode* com);
void MCode_26(GCode* com);
void MCode_27(GCode* com);
void MCode_28(GCode* com);
void MCode_29(GCode* com);
void MCode_30(GCode* com);
void MCode_32(GCode* com);
void MCode_36(GCode* com);
void MCode_42(GCode* com);
void MCode_48(GCode* com);
void MCode_80(GCode* com);
void MCode_81(GCode* com);
void MCode_82(GCode* com);
void MCode_83(GCode* com);
void MCode_84(GCode* com);
void MCode_85(GCode* com);
void MCode_92(GCode* com);
void MCode_99(GCode* com);
void MCode_104(GCode* com);
void MCode_105(GCode* com);
void MCode_106(GCode* com);
void MCode_107(GCode* com);
void MCode_108(GCode* com);
void MCode_109(GCode* com);
void MCode_111(GCode* com);
void MCode_114(GCode* com);
void MCode_115(GCode* com);
void MCode_116(GCode* com);
void MCode_117(GCode* com);
void MCode_118(GCode* com);
void MCode_119(GCode* com);
void MCode_120(GCode* com);
void MCode_140(GCode* com);
void MCode_141(GCode* com);
void MCode_155(GCode* com);
void MCode_163(GCode* com);
void MCode_164(GCode* com);
void MCode_170(GCode* com);
void MCode_190(GCode* com);
void MCode_191(GCode* com);
void MCode_200(GCode* com);
void MCode_201(GCode* com);
void MCode_202(GCode* com);
void MCode_203(GCode* com);
void MCode_204(GCode* com);
void MCode_205(GCode* com);
void MCode_206(GCode* com);
void MCode_207(GCode* com);
void MCode_209(GCode* com);
void MCode_218(GCode* com);
void MCode_220(GCode* com);
void MCode_221(GCode* com);
void MCode_226(GCode* com);
void MCode_232(GCode* com);
void MCode_251(GCode* com);
void MCode_280(GCode* com);
void MCode_281(GCode* com);
// M290 is defined in PrinterType
void MCode_300(GCode* com);
void MCode_302(GCode* com);
void MCode_303(GCode* com);
void MCode_320(GCode* com);
void MCode_321(GCode* com);
void MCode_322(GCode* com);
void MCode_323(GCode* com);
void MCode_340(GCode* com);
void MCode_350(GCode* com);
void MCode_355(GCode* com);
void MCode_360(GCode* com);
void MCode_400(GCode* com);
void MCode_401(GCode* com);
void MCode_402(GCode* com);
void MCode_408(GCode* com);
void MCode_415(GCode* com);
void MCode_416(GCode* com);
void MCode_460(GCode* com);
void MCode_500(GCode* com);
void MCode_501(GCode* com);
void MCode_502(GCode* com);
void MCode_513(GCode* com);
void MCode_524(GCode* com);
void MCode_530(GCode* com);
void MCode_531(GCode* com);
void MCode_532(GCode* com);
void MCode_539(GCode* com);
void MCode_540(GCode* com);
void MCode_575(GCode* com);
void MCode_576(GCode* com);
void MCode_600(GCode* com);
void MCode_601(GCode* com);
void MCode_602(GCode* com);
void MCode_604(GCode* com);
void MCode_606(GCode* com);
void MCode_669(GCode* com);
void MCode_876(GCode* com);
void MCode_890(GCode* com);
void MCode_900(GCode* com);
void MCode_907(GCode* com);
void MCode_908(GCode* com);
void MCode_909(GCode* com);
void MCode_910(GCode* com);
void MCode_998(GCode* com);
void MCode_999(GCode* com);
void MCode_Stepper(GCode* com);
#endif // COMMANDS_H_INCLUDED
