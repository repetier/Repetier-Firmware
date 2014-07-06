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

*/

#ifndef _EEPROM_H
#define _EEPROM_H

// Id to distinguish version changes
#define EEPROM_PROTOCOL_VERSION 7

/** Where to start with our datablock in memory. Can be moved if you
have problems with other modules using the eeprom */


#define EPR_MAGIC_BYTE              0
#define EPR_ACCELERATION_TYPE       1
#define EPR_XAXIS_STEPS_PER_MM      3
#define EPR_YAXIS_STEPS_PER_MM      7
#define EPR_ZAXIS_STEPS_PER_MM     11
#define EPR_X_MAX_FEEDRATE         15
#define EPR_Y_MAX_FEEDRATE         19
#define EPR_Z_MAX_FEEDRATE         23
#define EPR_X_HOMING_FEEDRATE      27
#define EPR_Y_HOMING_FEEDRATE      31
#define EPR_Z_HOMING_FEEDRATE      35
#define EPR_MAX_JERK               39
//#define EPR_OPS_MIN_DISTANCE       43
#define EPR_MAX_ZJERK              47
#define EPR_X_MAX_ACCEL            51
#define EPR_Y_MAX_ACCEL            55
#define EPR_Z_MAX_ACCEL            59
#define EPR_X_MAX_TRAVEL_ACCEL     63
#define EPR_Y_MAX_TRAVEL_ACCEL     67
#define EPR_Z_MAX_TRAVEL_ACCEL     71
#define EPR_BAUDRATE               75
#define EPR_MAX_INACTIVE_TIME      79
#define EPR_STEPPER_INACTIVE_TIME  83
//#define EPR_OPS_RETRACT_DISTANCE   87
//#define EPR_OPS_RETRACT_BACKLASH   91
#define EPR_EXTRUDER_SPEED         95
//#define EPR_OPS_MOVE_AFTER         99
//#define EPR_OPS_MODE              103
#define EPR_INTEGRITY_BYTE        104   // Here the xored sum over eeprom is stored
#define EPR_VERSION               105   // Version id for updates in EEPROM storage
#define EPR_BED_HEAT_MANAGER      106
#define EPR_BED_DRIVE_MAX         107
#define EPR_BED_PID_PGAIN         108
#define EPR_BED_PID_IGAIN         112
#define EPR_BED_PID_DGAIN         116
#define EPR_BED_PID_MAX           120
#define EPR_BED_DRIVE_MIN         124
#define EPR_PRINTING_TIME         125  // Time in seconds printing
#define EPR_PRINTING_DISTANCE     129  // Filament length printed
#define EPR_X_HOME_OFFSET         133
#define EPR_Y_HOME_OFFSET         137
#define EPR_Z_HOME_OFFSET         141
#define EPR_X_LENGTH              145
#define EPR_Y_LENGTH              149
#define EPR_Z_LENGTH              153
#define EPR_BACKLASH_X            157
#define EPR_BACKLASH_Y            161
#define EPR_BACKLASH_Z            165

#define EPR_Z_PROBE_X_OFFSET      800
#define EPR_Z_PROBE_Y_OFFSET      804
#define EPR_Z_PROBE_HEIGHT        808
#define EPR_Z_PROBE_SPEED         812
#define EPR_Z_PROBE_X1            816
#define EPR_Z_PROBE_Y1            820
#define EPR_Z_PROBE_X2            824
#define EPR_Z_PROBE_Y2            828
#define EPR_Z_PROBE_X3            832
#define EPR_Z_PROBE_Y3            836
#define EPR_Z_PROBE_XY_SPEED      840
#define EPR_AUTOLEVEL_MATRIX      844
#define EPR_AUTOLEVEL_ACTIVE      880
#define EPR_DELTA_DIAGONAL_ROD_LENGTH 881
#define EPR_DELTA_HORIZONTAL_RADIUS 885
#define EPR_DELTA_SEGMENTS_PER_SECOND_PRINT 889
#define EPR_DELTA_SEGMENTS_PER_SECOND_MOVE 891
#define EPR_DELTA_TOWERX_OFFSET_STEPS 893
#define EPR_DELTA_TOWERY_OFFSET_STEPS 895
#define EPR_DELTA_TOWERZ_OFFSET_STEPS 897
#define EPR_DELTA_ALPHA_A         901
#define EPR_DELTA_ALPHA_B         905
#define EPR_DELTA_ALPHA_C         909
#define EPR_DELTA_RADIUS_CORR_A   913
#define EPR_DELTA_RADIUS_CORR_B   917
#define EPR_DELTA_RADIUS_CORR_C   921
#define EPR_DELTA_MAX_RADIUS      925
#define EPR_Z_PROBE_BED_DISTANCE  929
#define EPR_DELTA_DIAGONAL_CORR_A 933
#define EPR_DELTA_DIAGONAL_CORR_B 937
#define EPR_DELTA_DIAGONAL_CORR_C 941

#define EEPROM_EXTRUDER_OFFSET 200
// bytes per extruder needed, leave some space for future development
#define EEPROM_EXTRUDER_LENGTH 100
// Extruder positions relative to extruder start
#define EPR_EXTRUDER_STEPS_PER_MM        0
#define EPR_EXTRUDER_MAX_FEEDRATE        4
// Feedrate from halted extruder in mm/s
#define EPR_EXTRUDER_MAX_START_FEEDRATE  8
// Acceleration in mm/s^2
#define EPR_EXTRUDER_MAX_ACCELERATION   12
#define EPR_EXTRUDER_HEAT_MANAGER       16
#define EPR_EXTRUDER_DRIVE_MAX          17
#define EPR_EXTRUDER_PID_PGAIN          18
#define EPR_EXTRUDER_PID_IGAIN          22
#define EPR_EXTRUDER_PID_DGAIN          26
#define EPR_EXTRUDER_PID_MAX            30
#define EPR_EXTRUDER_X_OFFSET           31
#define EPR_EXTRUDER_Y_OFFSET           35
#define EPR_EXTRUDER_WATCH_PERIOD       39
#define EPR_EXTRUDER_ADVANCE_K          41
#define EPR_EXTRUDER_DRIVE_MIN          45
#define EPR_EXTRUDER_ADVANCE_L          46
#define EPR_EXTRUDER_WAIT_RETRACT_TEMP 50
#define EPR_EXTRUDER_WAIT_RETRACT_UNITS 52
#define EPR_EXTRUDER_COOLER_SPEED       54

#ifndef Z_PROBE_BED_DISTANCE
#define Z_PROBE_BED_DISTANCE 5.0
#endif

class EEPROM
{
#if EEPROM_MODE!=0
    static uint8_t computeChecksum();
    static void writeExtruderPrefix(uint pos);
    static void writeFloat(uint pos,PGM_P text,uint8_t digits=3);
    static void writeLong(uint pos,PGM_P text);
    static void writeInt(uint pos,PGM_P text);
    static void writeByte(uint pos,PGM_P text);
#endif
public:

    static void init();
    static void initBaudrate();
    static void storeDataIntoEEPROM(uint8_t corrupted=0);
    static void readDataFromEEPROM();
    static void restoreEEPROMSettingsFromConfiguration();
    static void writeSettings();
    static void update(GCode *com);
    static void updatePrinterUsage();

    static inline float zProbeSpeed() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_SPEED);
#else
        return Z_PROBE_SPEED;
#endif
    }
    static inline float zProbeXYSpeed() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_XY_SPEED);
#else
        return Z_PROBE_XY_SPEED;
#endif
    }
    static inline float zProbeXOffset() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_X_OFFSET);
#else
        return Z_PROBE_X_OFFSET;
#endif
    }
    static inline float zProbeYOffset() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_Y_OFFSET);
#else
        return Z_PROBE_Y_OFFSET;
#endif
    }
    static inline float zProbeHeight() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_HEIGHT);
#else
        return Z_PROBE_HEIGHT;
#endif
    }
    static inline float zProbeX1() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_X1);
#else
        return Z_PROBE_X1;
#endif
    }
    static inline float zProbeY1() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_Y1);
#else
        return Z_PROBE_Y1;
#endif
    }
    static inline float zProbeX2() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_X2);
#else
        return Z_PROBE_X2;
#endif
    }
    static inline float zProbeY2() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_Y2);
#else
        return Z_PROBE_Y2;
#endif
    }
    static inline float zProbeX3() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_X3);
#else
        return Z_PROBE_X3;
#endif
    }
    static inline float zProbeY3() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_Y3);
#else
        return Z_PROBE_Y3;
#endif
    }
    static inline float zProbeBedDistance() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_Z_PROBE_BED_DISTANCE);
#else
        return Z_PROBE_BED_DISTANCE;
#endif
    }
#if NONLINEAR_SYSTEM
    static inline int16_t deltaSegmentsPerSecondMove() {
#if EEPROM_MODE!=0
        return HAL::eprGetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE);
#else
        return DELTA_SEGMENTS_PER_SECOND_MOVE;
#endif
    }
    static inline float deltaDiagonalRodLength() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH);
#else
        return DELTA_DIAGONAL_ROD;
#endif
    }
    static inline int16_t deltaSegmentsPerSecondPrint() {
#if EEPROM_MODE!=0
        return HAL::eprGetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT);
#else
        return DELTA_SEGMENTS_PER_SECOND_PRINT;
#endif
    }
#endif
#if DRIVE_SYSTEM==3
    static inline float deltaHorizontalRadius() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_HORIZONTAL_RADIUS);
#else
        return DELTA_RADIUS;
#endif
    }
    static inline int16_t deltaTowerXOffsetSteps() {
#if EEPROM_MODE!=0
        return HAL::eprGetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS);
#else
        return DELTA_X_ENDSTOP_OFFSET_STEPS;
#endif
    }
    static inline int16_t deltaTowerYOffsetSteps() {
#if EEPROM_MODE!=0
        return HAL::eprGetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS);
#else
        return DELTA_Y_ENDSTOP_OFFSET_STEPS;
#endif
    }
    static inline int16_t deltaTowerZOffsetSteps() {
#if EEPROM_MODE!=0
        return HAL::eprGetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS);
#else
        return DELTA_Z_ENDSTOP_OFFSET_STEPS;
#endif
    }
    static inline void setDeltaTowerXOffsetSteps(int16_t steps) {
#if EEPROM_MODE!=0
        HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,steps);
        uint8_t newcheck = computeChecksum();
        if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
            HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
#endif
    }
    static inline void setDeltaTowerYOffsetSteps(int16_t steps) {
#if EEPROM_MODE!=0
        HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,steps);
        uint8_t newcheck = computeChecksum();
        if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
            HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
#endif
    }
    static inline void setDeltaTowerZOffsetSteps(int16_t steps) {
#if EEPROM_MODE!=0
        HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,steps);
        uint8_t newcheck = computeChecksum();
        if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
            HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
#endif
    }
    static inline float deltaAlphaA() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_ALPHA_A);
#else
        return DELTA_ALPHA_A;
#endif
    }
    static inline float deltaAlphaB() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_ALPHA_B);
#else
        return DELTA_ALPHA_B;
#endif
    }
    static inline float deltaAlphaC() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_ALPHA_C);
#else
        return DELTA_ALPHA_C;
#endif
    }
    static inline float deltaRadiusCorrectionA() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_A);
#else
        return DELTA_RADIUS_CORRECTION_A;
#endif
    }
    static inline float deltaRadiusCorrectionB() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_B);
#else
        return DELTA_RADIUS_CORRECTION_B;
#endif
    }
    static inline float deltaRadiusCorrectionC() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_C);
#else
        return DELTA_RADIUS_CORRECTION_C;
#endif
    }
    static inline float deltaDiagonalCorrectionA() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_DIAGONAL_CORR_A);
#else
        return DELTA_DIAGONAL_CORRECTION_A;
#endif
    }
    static inline float deltaDiagonalCorrectionB() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_DIAGONAL_CORR_B);
#else
        return DELTA_DIAGONAL_CORRECTION_B;
#endif
    }
    static inline float deltaDiagonalCorrectionC() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_DIAGONAL_CORR_C);
#else
        return DELTA_DIAGONAL_CORRECTION_C;
#endif
    }
    static inline float deltaMaxRadius() {
#if EEPROM_MODE!=0
        return HAL::eprGetFloat(EPR_DELTA_MAX_RADIUS);
#else
        return DELTA_MAX_RADIUS;
#endif
    }

#endif
    static void initalizeUncached();
};
#endif
