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

#if EEPROM_MODE!=0

byte EEPROM::computeChecksum()
{
    int i;
    byte checksum=0;
    for(i=0; i<2048; i++)
    {
        if(i==EEPROM_OFFSET+EPR_INTEGRITY_BYTE) continue;
        checksum += eeprom_read_byte ((unsigned char *)(i));
    }
    return checksum;
}



void EEPROM::writeExtruderPrefix(uint pos)
{
    if(pos<EEPROM_EXTRUDER_OFFSET) return;
    int n = (pos-EEPROM_EXTRUDER_OFFSET)/EEPROM_EXTRUDER_LENGTH+1;
    OUT_P_I("Extr.",n);
    out.print(' ');
}
void EEPROM::writeFloat(uint pos,PGM_P text)
{
    OUT_P("EPR:3 ");
    OUT(pos);
    OUT(' ');
    OUT(HAL::epr_get_float(pos));
    OUT(' ');
    writeExtruderPrefix(pos);
    out.println_P(text);
}
void EEPROM::writeLong(uint pos,PGM_P text)
{
    OUT_P("EPR:2 ");
    OUT(pos);
    OUT(' ');
    OUT(HAL::epr_get_long(pos));
    OUT(' ');
    writeExtruderPrefix(pos);
    out.println_P(text);
}
void EEPROM::writeInt(uint pos,PGM_P text)
{
    OUT_P("EPR:1 ");
    OUT(pos);
    OUT(' ');
    OUT(HAL::epr_get_int(pos));
    OUT(' ');
    writeExtruderPrefix(pos);
    out.println_P(text);
}
void EEPROM::writeByte(uint pos,PGM_P text)
{
    OUT_P("EPR:0 ");
    OUT(pos);
    OUT(' ');
    OUT((int)HAL::epr_get_byte(pos));
    OUT(' ');
    writeExtruderPrefix(pos);
    out.println_P(text);
}
#endif

void EEPROM::update(GCode *com)
{
#if EEPROM_MODE!=0
    if(com->hasT() && com->hasP()) switch(com->T)
        {
        case 0:
            if(com->hasS()) HAL::epr_set_byte(com->P,(byte)com->S);
            break;
        case 1:
            if(com->hasS()) HAL::epr_set_int(com->P,(int)com->S);
            break;
        case 2:
            if(com->hasS()) HAL::epr_set_long(com->P,(long)com->S);
            break;
        case 3:
            if(com->hasX()) HAL::epr_set_float(com->P,com->X);
            break;
        }
    byte newcheck = computeChecksum();
    if(newcheck!=HAL::epr_get_byte(EPR_INTEGRITY_BYTE))
        HAL::epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
    readDataFromEEPROM();
#else
    OUT_P_LN("Error: No EEPROM support compiled.");
#endif
}

/** \brief Copy data from EEPROM to variables.
*/
void EEPROM::restoreEEPROMSettingsFromConfiguration()
{
#if EEPROM_MODE!=0
    byte version = EEPROM_PROTOCOL_VERSION;
    baudrate = BAUDRATE;
    max_inactive_time = MAX_INACTIVE_TIME*1000L;
    stepper_inactive_time = STEPPER_INACTIVE_TIME*1000L;
    axis_steps_per_unit[0] = XAXIS_STEPS_PER_MM;
    axis_steps_per_unit[1] = YAXIS_STEPS_PER_MM;
    axis_steps_per_unit[2] = ZAXIS_STEPS_PER_MM;
    axis_steps_per_unit[3] = 1;
    max_feedrate[0] = MAX_FEEDRATE_X;
    max_feedrate[1] = MAX_FEEDRATE_Y;
    max_feedrate[2] = MAX_FEEDRATE_Z;
    homing_feedrate[0] = HOMING_FEEDRATE_X;
    homing_feedrate[1] = HOMING_FEEDRATE_Y;
    homing_feedrate[2] = HOMING_FEEDRATE_Z;
    printer.maxJerk = MAX_JERK;
    printer.maxZJerk = MAX_ZJERK;
#ifdef RAMP_ACCELERATION
    max_acceleration_units_per_sq_second[0] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    max_acceleration_units_per_sq_second[1] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    max_acceleration_units_per_sq_second[2] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    max_travel_acceleration_units_per_sq_second[0] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    max_travel_acceleration_units_per_sq_second[1] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    max_travel_acceleration_units_per_sq_second[2] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
#endif
#if USE_OPS==1
    printer.opsMode = OPS_MODE;
    printer.opsMinDistance = OPS_MIN_DISTANCE;
    printer.opsRetractDistance = OPS_RETRACT_DISTANCE;
    printer.opsRetractBacklash = OPS_RETRACT_BACKLASH;
    printer.opsMoveAfter = OPS_MOVE_AFTER;
#endif
#if HAVE_HEATED_BED
    heatedBedController.heatManager= HEATED_BED_HEAT_MANAGER;
#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
    heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
    heatedBedController.pidPGain = HEATED_BED_PID_PGAIN;
    heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
    heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
    heatedBedController.pidMax = HEATED_BED_PID_MAX;
#endif
#endif
    printer.xLength = X_MAX_LENGTH;
    printer.yLength = Y_MAX_LENGTH;
    printer.zLength = Z_MAX_LENGTH;
    printer.xMin = X_MIN_POS;
    printer.yMin = Y_MIN_POS;
    printer.zMin = Z_MIN_POS;
#if ENABLE_BACKLASH_COMPENSATION
    printer.backlashX = X_BACKLASH;
    printer.backlashY = Y_BACKLASH;
    printer.backlashZ = Z_BACKLASH;
#endif
    Extruder *e;
#if NUM_EXTRUDER>0
    e = &extruder[0];
    e->stepsPerMM = EXT0_STEPS_PER_MM;
    e->maxFeedrate = EXT0_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT0_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT0_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT0_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT0_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT0_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT0_PID_P;
    e->tempControl.pidIGain = EXT0_PID_I;
    e->tempControl.pidDGain = EXT0_PID_D;
    e->tempControl.pidMax = EXT0_PID_MAX;
#endif
    e->yOffset = EXT0_Y_OFFSET;
    e->xOffset = EXT0_X_OFFSET;
    e->watchPeriod = EXT0_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT0_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT0_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT0_ADVANCE_K;
#endif
    e->advanceL = EXT0_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER>0
#if NUM_EXTRUDER>1
    e = &extruder[1];
    e->stepsPerMM = EXT1_STEPS_PER_MM;
    e->maxFeedrate = EXT1_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT1_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT1_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT1_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT1_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT1_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT1_PID_P;
    e->tempControl.pidIGain = EXT1_PID_I;
    e->tempControl.pidDGain = EXT1_PID_D;
    e->tempControl.pidMax = EXT1_PID_MAX;
#endif
    e->yOffset = EXT1_Y_OFFSET;
    e->xOffset = EXT1_X_OFFSET;
    e->watchPeriod = EXT1_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT1_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT1_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT1_ADVANCE_K;
#endif
    e->advanceL = EXT1_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 1
#if NUM_EXTRUDER>2
    e = &extruder[2];
    e->stepsPerMM = EXT2_STEPS_PER_MM;
    e->maxFeedrate = EXT2_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT2_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT2_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT2_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT2_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT2_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT2_PID_P;
    e->tempControl.pidIGain = EXT2_PID_I;
    e->tempControl.pidDGain = EXT2_PID_D;
    e->tempControl.pidMax = EXT2_PID_MAX;
#endif
    e->yOffset = EXT2_Y_OFFSET;
    e->xOffset = EXT2_X_OFFSET;
    e->watchPeriod = EXT2_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT2_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT2_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT2_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT2_ADVANCE_K;
#endif
    e->advanceL = EXT2_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 2
#if NUM_EXTRUDER>3
    e = &extruder[3];
    e->stepsPerMM = EXT3_STEPS_PER_MM;
    e->maxFeedrate = EXT3_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT3_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT3_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT3_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT3_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT3_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT3_PID_P;
    e->tempControl.pidIGain = EXT3_PID_I;
    e->tempControl.pidDGain = EXT3_PID_D;
    e->tempControl.pidMax = EXT3_PID_MAX;
#endif
    e->yOffset = EXT3_Y_OFFSET;
    e->xOffset = EXT3_X_OFFSET;
    e->watchPeriod = EXT3_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT3_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT3_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT3_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT3_ADVANCE_K;
#endif
    e->advanceL = EXT3_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 3
#if NUM_EXTRUDER>4
    e = &extruder[4];
    e->stepsPerMM = EXT4_STEPS_PER_MM;
    e->maxFeedrate = EXT4_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT4_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT4_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT4_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT4_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT4_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT4_PID_P;
    e->tempControl.pidIGain = EXT4_PID_I;
    e->tempControl.pidDGain = EXT4_PID_D;
    e->tempControl.pidMax = EXT4_PID_MAX;
#endif
    e->yOffset = EXT4_Y_OFFSET;
    e->xOffset = EXT4_X_OFFSET;
    e->watchPeriod = EXT4_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT4_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT4_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT4_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT4_ADVANCE_K;
#endif
    e->advanceL = EXT4_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 4
#if NUM_EXTRUDER>5
    e = &extruder[5];
    e->stepsPerMM = EXT5_STEPS_PER_MM;
    e->maxFeedrate = EXT5_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT5_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT5_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT5_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT5_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT5_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT5_PID_P;
    e->tempControl.pidIGain = EXT5_PID_I;
    e->tempControl.pidDGain = EXT5_PID_D;
    e->tempControl.pidMax = EXT5_PID_MAX;
#endif
    e->yOffset = EXT5_Y_OFFSET;
    e->xOffset = EXT5_X_OFFSET;
    e->watchPeriod = EXT5_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT5_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT5_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT5_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT5_ADVANCE_K;
#endif
    e->advanceL = EXT5_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 5
    Extruder::selectExtruderById(current_extruder->id);
    update_ramps_parameter();
    Extruder::initHeatedBed();
    OUT_P_LN("Configuration reset to defaults.");
#else
    OUT_P_LN("Error: No EEPROM support compiled.");
#endif
}
/** \brief Moves current settings to EEPROM.

The values the are currently set are used to fill the eeprom.*/
void EEPROM::storeDataIntoEEPROM(byte corrupted)
{
#if EEPROM_MODE!=0
    HAL::epr_set_long(EPR_BAUDRATE,baudrate);
    HAL::epr_set_long(EPR_MAX_INACTIVE_TIME,max_inactive_time);
    HAL::epr_set_long(EPR_STEPPER_INACTIVE_TIME,stepper_inactive_time);
//#define EPR_ACCELERATION_TYPE 1
    HAL::epr_set_float(EPR_XAXIS_STEPS_PER_MM,axis_steps_per_unit[0]);
    HAL::epr_set_float(EPR_YAXIS_STEPS_PER_MM,axis_steps_per_unit[1]);
    HAL::epr_set_float(EPR_ZAXIS_STEPS_PER_MM,axis_steps_per_unit[2]);
    HAL::epr_set_float(EPR_X_MAX_FEEDRATE,max_feedrate[0]);
    HAL::epr_set_float(EPR_Y_MAX_FEEDRATE,max_feedrate[1]);
    HAL::epr_set_float(EPR_Z_MAX_FEEDRATE,max_feedrate[2]);
    HAL::epr_set_float(EPR_X_HOMING_FEEDRATE,homing_feedrate[0]);
    HAL::epr_set_float(EPR_Y_HOMING_FEEDRATE,homing_feedrate[1]);
    HAL::epr_set_float(EPR_Z_HOMING_FEEDRATE,homing_feedrate[2]);
    HAL::epr_set_float(EPR_MAX_JERK,printer.maxJerk);
    HAL::epr_set_float(EPR_MAX_ZJERK,printer.maxZJerk);
#ifdef RAMP_ACCELERATION
    HAL::epr_set_float(EPR_X_MAX_ACCEL,max_acceleration_units_per_sq_second[0]);
    HAL::epr_set_float(EPR_Y_MAX_ACCEL,max_acceleration_units_per_sq_second[1]);
    HAL::epr_set_float(EPR_Z_MAX_ACCEL,max_acceleration_units_per_sq_second[2]);
    HAL::epr_set_float(EPR_X_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[0]);
    HAL::epr_set_float(EPR_Y_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[1]);
    HAL::epr_set_float(EPR_Z_MAX_TRAVEL_ACCEL,max_travel_acceleration_units_per_sq_second[2]);
#endif
#if USE_OPS==1
    HAL::epr_set_float(EPR_OPS_MIN_DISTANCE,printer.opsMinDistance);
    HAL::epr_set_byte(EPR_OPS_MODE,printer.opsMode);
    HAL::epr_set_float(EPR_OPS_MOVE_AFTER,printer.opsMoveAfter);
    HAL::epr_set_float(EPR_OPS_RETRACT_DISTANCE,printer.opsRetractDistance);
    HAL::epr_set_float(EPR_OPS_RETRACT_BACKLASH,printer.opsRetractBacklash);
#else
    HAL::epr_set_float(EPR_OPS_MIN_DISTANCE,OPS_MIN_DISTANCE);
    HAL::epr_set_byte(EPR_OPS_MODE,OPS_MODE);
    HAL::epr_set_float(EPR_OPS_MOVE_AFTER,OPS_MOVE_AFTER);
    HAL::epr_set_float(EPR_OPS_RETRACT_DISTANCE,OPS_RETRACT_DISTANCE);
    HAL::epr_set_float(EPR_OPS_RETRACT_BACKLASH,OPS_RETRACT_BACKLASH);
#endif
#if HAVE_HEATED_BED
    HAL::epr_set_byte(EPR_BED_HEAT_MANAGER,heatedBedController.heatManager);
#else
    HAL::epr_set_byte(EPR_BED_HEAT_MANAGER,HEATED_BED_HEAT_MANAGER);
#endif
#if defined(TEMP_PID) && HAVE_HEATED_BED
    HAL::epr_set_byte(EPR_BED_DRIVE_MAX,heatedBedController.pidDriveMax);
    HAL::epr_set_byte(EPR_BED_DRIVE_MIN,heatedBedController.pidDriveMin);
    HAL::epr_set_float(EPR_BED_PID_PGAIN,heatedBedController.pidPGain);
    HAL::epr_set_float(EPR_BED_PID_IGAIN,heatedBedController.pidIGain);
    HAL::epr_set_float(EPR_BED_PID_DGAIN,heatedBedController.pidDGain);
    HAL::epr_set_byte(EPR_BED_PID_MAX,heatedBedController.pidMax);
#else
    HAL::epr_set_byte(EPR_BED_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MAX);
    HAL::epr_set_byte(EPR_BED_DRIVE_MIN,HEATED_BED_PID_INTEGRAL_DRIVE_MIN);
    HAL::epr_set_float(EPR_BED_PID_PGAIN,HEATED_BED_PID_PGAIN);
    HAL::epr_set_float(EPR_BED_PID_IGAIN,HEATED_BED_PID_IGAIN);
    HAL::epr_set_float(EPR_BED_PID_DGAIN,HEATED_BED_PID_DGAIN);
    HAL::epr_set_byte(EPR_BED_PID_MAX,HEATED_BED_PID_MAX);
#endif
    HAL::epr_set_float(EPR_X_HOME_OFFSET,printer.xMin);
    HAL::epr_set_float(EPR_Y_HOME_OFFSET,printer.yMin);
    HAL::epr_set_float(EPR_Z_HOME_OFFSET,printer.zMin);
    HAL::epr_set_float(EPR_X_LENGTH,printer.xLength);
    HAL::epr_set_float(EPR_Y_LENGTH,printer.yLength);
    HAL::epr_set_float(EPR_Z_LENGTH,printer.zLength);
#if ENABLE_BACKLASH_COMPENSATION
    HAL::epr_set_float(EPR_BACKLASH_X,printer.backlashX);
    HAL::epr_set_float(EPR_BACKLASH_Y,printer.backlashY);
    HAL::epr_set_float(EPR_BACKLASH_Z,printer.backlashZ);
#else
    HAL::epr_set_float(EPR_BACKLASH_X,0);
    HAL::epr_set_float(EPR_BACKLASH_Y,0);
    HAL::epr_set_float(EPR_BACKLASH_Z,0);
#endif

    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        HAL::epr_set_float(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
        HAL::epr_set_byte(o+EPR_EXTRUDER_HEAT_MANAGER,e->tempControl.heatManager);
#ifdef TEMP_PID
        HAL::epr_set_byte(o+EPR_EXTRUDER_DRIVE_MAX,e->tempControl.pidDriveMax);
        HAL::epr_set_byte(o+EPR_EXTRUDER_DRIVE_MIN,e->tempControl.pidDriveMin);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_PGAIN,e->tempControl.pidPGain);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_IGAIN,e->tempControl.pidIGain);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_DGAIN,e->tempControl.pidDGain);
        HAL::epr_set_byte(o+EPR_EXTRUDER_PID_MAX,e->tempControl.pidMax);
#endif
        HAL::epr_set_long(o+EPR_EXTRUDER_X_OFFSET,e->xOffset);
        HAL::epr_set_long(o+EPR_EXTRUDER_Y_OFFSET,e->yOffset);
        HAL::epr_set_int(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);
#if RETRACT_DURING_HEATUP
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,e->waitRetractTemperature);
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,e->waitRetractUnits);
#else
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_TEMP);
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,EXT0_WAIT_RETRACT_UNITS);
#endif
        HAL::epr_set_byte(o+EPR_EXTRUDER_COOLER_SPEED,e->coolerSpeed);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
#else
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,0);
#endif
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);
#else
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,0);
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_L,0);
#endif
    }
    if(corrupted)
    {
        HAL::epr_set_long(EPR_PRINTING_TIME,0);
        HAL::epr_set_float(EPR_PRINTING_DISTANCE,0);
    }
    // Save version and build checksum
    HAL::epr_set_byte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
    HAL::epr_set_byte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif
}
/** \brief Copy data from EEPROM to variables.
*/
void EEPROM::readDataFromEEPROM()
{
#if EEPROM_MODE!=0
    byte version = HAL::epr_get_byte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
    baudrate = HAL::epr_get_long(EPR_BAUDRATE);
    max_inactive_time = HAL::epr_get_long(EPR_MAX_INACTIVE_TIME);
    stepper_inactive_time = HAL::epr_get_long(EPR_STEPPER_INACTIVE_TIME);
//#define EPR_ACCELERATION_TYPE 1
    axis_steps_per_unit[0] = HAL::epr_get_float(EPR_XAXIS_STEPS_PER_MM);
    axis_steps_per_unit[1] = HAL::epr_get_float(EPR_YAXIS_STEPS_PER_MM);
    axis_steps_per_unit[2] = HAL::epr_get_float(EPR_ZAXIS_STEPS_PER_MM);
    max_feedrate[0] = HAL::epr_get_float(EPR_X_MAX_FEEDRATE);
    max_feedrate[1] = HAL::epr_get_float(EPR_Y_MAX_FEEDRATE);
    max_feedrate[2] = HAL::epr_get_float(EPR_Z_MAX_FEEDRATE);
    homing_feedrate[0] = HAL::epr_get_float(EPR_X_HOMING_FEEDRATE);
    homing_feedrate[1] = HAL::epr_get_float(EPR_Y_HOMING_FEEDRATE);
    homing_feedrate[2] = HAL::epr_get_float(EPR_Z_HOMING_FEEDRATE);
    printer.maxJerk = HAL::epr_get_float(EPR_MAX_JERK);
    printer.maxZJerk = HAL::epr_get_float(EPR_MAX_ZJERK);
#ifdef RAMP_ACCELERATION
    max_acceleration_units_per_sq_second[0] = HAL::epr_get_float(EPR_X_MAX_ACCEL);
    max_acceleration_units_per_sq_second[1] = HAL::epr_get_float(EPR_Y_MAX_ACCEL);
    max_acceleration_units_per_sq_second[2] = HAL::epr_get_float(EPR_Z_MAX_ACCEL);
    max_travel_acceleration_units_per_sq_second[0] = HAL::epr_get_float(EPR_X_MAX_TRAVEL_ACCEL);
    max_travel_acceleration_units_per_sq_second[1] = HAL::epr_get_float(EPR_Y_MAX_TRAVEL_ACCEL);
    max_travel_acceleration_units_per_sq_second[2] = HAL::epr_get_float(EPR_Z_MAX_TRAVEL_ACCEL);
#endif
#if USE_OPS==1
    printer.opsMode = HAL::epr_get_byte(EPR_OPS_MODE);
    printer.opsMoveAfter = HAL::epr_get_float(EPR_OPS_MOVE_AFTER);
    printer.opsMinDistance = HAL::epr_get_float(EPR_OPS_MIN_DISTANCE);
    printer.opsRetractDistance = HAL::epr_get_float(EPR_OPS_RETRACT_DISTANCE);
    printer.opsRetractBacklash = HAL::epr_get_float(EPR_OPS_RETRACT_BACKLASH);
#endif
#if HAVE_HEATED_BED
    heatedBedController.heatManager= HAL::epr_get_byte(EPR_BED_HEAT_MANAGER);
#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HAL::epr_get_byte(EPR_BED_DRIVE_MAX);
    heatedBedController.pidDriveMin = HAL::epr_get_byte(EPR_BED_DRIVE_MIN);
    heatedBedController.pidPGain = HAL::epr_get_float(EPR_BED_PID_PGAIN);
    heatedBedController.pidIGain = HAL::epr_get_float(EPR_BED_PID_IGAIN);
    heatedBedController.pidDGain = HAL::epr_get_float(EPR_BED_PID_DGAIN);
    heatedBedController.pidMax = HAL::epr_get_byte(EPR_BED_PID_MAX);
#endif
#endif
    printer.xMin = HAL::epr_get_float(EPR_X_HOME_OFFSET);
    printer.yMin = HAL::epr_get_float(EPR_Y_HOME_OFFSET);
    printer.zMin = HAL::epr_get_float(EPR_Z_HOME_OFFSET);
    printer.xLength = HAL::epr_get_float(EPR_X_LENGTH);
    printer.yLength = HAL::epr_get_float(EPR_Y_LENGTH);
    printer.zLength = HAL::epr_get_float(EPR_Z_LENGTH);
#if ENABLE_BACKLASH_COMPENSATION
    printer.backlashX = HAL::epr_get_float(EPR_BACKLASH_X);
    printer.backlashY = HAL::epr_get_float(EPR_BACKLASH_Y);
    printer.backlashZ = HAL::epr_get_float(EPR_BACKLASH_Z);
#endif
    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        e->stepsPerMM = HAL::epr_get_float(o+EPR_EXTRUDER_STEPS_PER_MM);
        e->maxFeedrate = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_FEEDRATE);
        e->maxStartFeedrate = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
        e->maxAcceleration = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_ACCELERATION);
        e->tempControl.heatManager = HAL::epr_get_byte(o+EPR_EXTRUDER_HEAT_MANAGER);
#ifdef TEMP_PID
        e->tempControl.pidDriveMax = HAL::epr_get_byte(o+EPR_EXTRUDER_DRIVE_MAX);
        e->tempControl.pidDriveMin = HAL::epr_get_byte(o+EPR_EXTRUDER_DRIVE_MIN);
        e->tempControl.pidPGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_PGAIN);
        e->tempControl.pidIGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_IGAIN);
        e->tempControl.pidDGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_DGAIN);
        e->tempControl.pidMax = HAL::epr_get_byte(o+EPR_EXTRUDER_PID_MAX);
#endif
        e->xOffset = HAL::epr_get_long(o+EPR_EXTRUDER_X_OFFSET);
        e->yOffset = HAL::epr_get_long(o+EPR_EXTRUDER_Y_OFFSET);
        e->watchPeriod = HAL::epr_get_int(o+EPR_EXTRUDER_WATCH_PERIOD);
#if RETRACT_DURING_HEATUP
        e->waitRetractTemperature = HAL::epr_get_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP);
        e->waitRetractUnits = HAL::epr_get_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        e->advanceK = HAL::epr_get_float(o+EPR_EXTRUDER_ADVANCE_K);
#endif
        e->advanceL = HAL::epr_get_float(o+EPR_EXTRUDER_ADVANCE_L);
#endif
        if(version>1)
            e->coolerSpeed = HAL::epr_get_byte(o+EPR_EXTRUDER_COOLER_SPEED);
    }
    if(version!=EEPROM_PROTOCOL_VERSION)
    {
        OUT_P_LN("Protocol version changed, upgrading");
        storeDataIntoEEPROM(false); // Store new fields for changed version
    }
    Extruder::selectExtruderById(current_extruder->id);
    update_ramps_parameter();
    Extruder::initHeatedBed();
#endif
}

void EEPROM::initBaudrate()
{
#if EEPROM_MODE!=0
    if(HAL::epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE)
    {
        baudrate = HAL::epr_get_long(EPR_BAUDRATE);
    }
#endif
}
void EEPROM::init()
{
#if EEPROM_MODE!=0
    byte check = computeChecksum();
    byte storedcheck = HAL::epr_get_byte(EPR_INTEGRITY_BYTE);
    if(HAL::epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE && storedcheck==check)
    {
        readDataFromEEPROM();
    }
    else
    {
        HAL::epr_set_byte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make datachange permanent
        storeDataIntoEEPROM(storedcheck!=check);
    }
#endif
}
/**
*/
void EEPROM::updatePrinterUsage()
{
#if EEPROM_MODE!=0
    if(printer.filamentPrinted==0) return; // No miles only enabled
    unsigned long seconds = (HAL::timeInMilliseconds()-printer.msecondsPrinting)/1000;
    seconds += HAL::epr_get_long(EPR_PRINTING_TIME);
    HAL::epr_set_long(EPR_PRINTING_TIME,seconds);
    HAL::epr_set_float(EPR_PRINTING_DISTANCE,HAL::epr_get_float(EPR_PRINTING_DISTANCE)+printer.filamentPrinted*0.001);
    printer.filamentPrinted = 0;
    printer.msecondsPrinting = HAL::timeInMilliseconds();
    byte newcheck = computeChecksum();
    if(newcheck!=HAL::epr_get_byte(EPR_INTEGRITY_BYTE))
        HAL::epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
    Commands::reportPrinterUsage();
#endif
}

/** \brief Writes all eeprom settings to serial console.

For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void EEPROM::writeSettings()
{
#if EEPROM_MODE!=0
    writeLong(EPR_BAUDRATE,PSTR("Baudrate"));
    writeFloat(EPR_PRINTING_DISTANCE,PSTR("Filament printed [m]"));
    writeLong(EPR_PRINTING_TIME,PSTR("Printer active [s]"));
    writeLong(EPR_MAX_INACTIVE_TIME,PSTR("Max. inactive time [ms,0=off]"));
    writeLong(EPR_STEPPER_INACTIVE_TIME,PSTR("Stop stepper after inactivity [ms,0=off]"));
//#define EPR_ACCELERATION_TYPE 1
    writeFloat(EPR_XAXIS_STEPS_PER_MM,PSTR("X-axis steps per mm"));
    writeFloat(EPR_YAXIS_STEPS_PER_MM,PSTR("Y-axis steps per mm"));
    writeFloat(EPR_ZAXIS_STEPS_PER_MM,PSTR("Z-axis steps per mm"));
    writeFloat(EPR_X_MAX_FEEDRATE,PSTR("X-axis max. feedrate [mm/s]"));
    writeFloat(EPR_Y_MAX_FEEDRATE,PSTR("Y-axis max. feedrate [mm/s]"));
    writeFloat(EPR_Z_MAX_FEEDRATE,PSTR("Z-axis max. feedrate [mm/s]"));
    writeFloat(EPR_X_HOMING_FEEDRATE,PSTR("X-axis homing feedrate [mm/s]"));
    writeFloat(EPR_Y_HOMING_FEEDRATE,PSTR("Y-axis homing feedrate [mm/s]"));
    writeFloat(EPR_Z_HOMING_FEEDRATE,PSTR("Z-axis homing feedrate [mm/s]"));
    writeFloat(EPR_MAX_JERK,PSTR("Max. jerk [mm/s]"));
    writeFloat(EPR_MAX_ZJERK,PSTR("Max. Z-jerk [mm/s]"));
    writeFloat(EPR_X_HOME_OFFSET,PSTR("X home pos [mm]"));
    writeFloat(EPR_Y_HOME_OFFSET,PSTR("Y home pos [mm]"));
    writeFloat(EPR_Z_HOME_OFFSET,PSTR("Z home pos [mm]"));
    writeFloat(EPR_X_LENGTH,PSTR("X max length [mm]"));
    writeFloat(EPR_Y_LENGTH,PSTR("Y max length [mm]"));
    writeFloat(EPR_Z_LENGTH,PSTR("Z max length [mm]"));
#if ENABLE_BACKLASH_COMPENSATION
    writeFloat(EPR_BACKLASH_X,PSTR("X backlash [mm]"));
    writeFloat(EPR_BACKLASH_Y,PSTR("Y backlash [mm]"));
    writeFloat(EPR_BACKLASH_Z,PSTR("Z backlash [mm]"));
#endif

#ifdef RAMP_ACCELERATION
    //epr_out_float(EPR_X_MAX_START_SPEED,PSTR("X-axis start speed [mm/s]"));
    //epr_out_float(EPR_Y_MAX_START_SPEED,PSTR("Y-axis start speed [mm/s]"));
    //epr_out_float(EPR_Z_MAX_START_SPEED,PSTR("Z-axis start speed [mm/s]"));
    writeFloat(EPR_X_MAX_ACCEL,PSTR("X-axis acceleration [mm/s^2]"));
    writeFloat(EPR_Y_MAX_ACCEL,PSTR("Y-axis acceleration [mm/s^2]"));
    writeFloat(EPR_Z_MAX_ACCEL,PSTR("Z-axis acceleration [mm/s^2]"));
    writeFloat(EPR_X_MAX_TRAVEL_ACCEL,PSTR("X-axis travel acceleration [mm/s^2]"));
    writeFloat(EPR_Y_MAX_TRAVEL_ACCEL,PSTR("Y-axis travel acceleration [mm/s^2]"));
    writeFloat(EPR_Z_MAX_TRAVEL_ACCEL,PSTR("Z-axis travel acceleration [mm/s^2]"));
#endif
#if USE_OPS==1
    writeByte(EPR_OPS_MODE,PSTR("OPS operation mode [0=Off,1=Classic,2=Fast]"));
    writeFloat(EPR_OPS_MOVE_AFTER,PSTR("OPS move after x% retract [%]"));
    writeFloat(EPR_OPS_MIN_DISTANCE,PSTR("OPS min. distance for fil. retraction [mm]"));
    writeFloat(EPR_OPS_RETRACT_DISTANCE,PSTR("OPS retraction length [mm]"));
    writeFloat(EPR_OPS_RETRACT_BACKLASH,PSTR("OPS retraction backlash [mm]"));
#endif
#if HAVE_HEATED_BED
    writeByte(EPR_BED_HEAT_MANAGER,PSTR("Bed Heat Manager [0-2]"));
#ifdef TEMP_PID
    writeByte(EPR_BED_DRIVE_MAX,PSTR("Bed PID drive max"));
    writeByte(EPR_BED_DRIVE_MIN,PSTR("Bed PID drive min"));
    writeFloat(EPR_BED_PID_PGAIN,PSTR("Bed PID P-gain"));
    writeFloat(EPR_BED_PID_IGAIN,PSTR("Bed PID I-gain"));
    writeFloat(EPR_BED_PID_DGAIN,PSTR("Bed PID D-gain"));
    writeByte(EPR_BED_PID_MAX,PSTR("Bed PID max value [0-255]"));
#endif
#endif
    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        writeFloat(o+EPR_EXTRUDER_STEPS_PER_MM,PSTR("steps per mm"));
        writeFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,PSTR("max. feedrate [mm/s]"));
        writeFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,PSTR("start feedrate [mm/s]"));
        writeFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,PSTR("acceleration [mm/s^2]"));
        writeByte(o+EPR_EXTRUDER_HEAT_MANAGER,PSTR("heat manager [0-1]"));
#ifdef TEMP_PID
        writeByte(o+EPR_EXTRUDER_DRIVE_MAX,PSTR("PID drive max"));
        writeByte(o+EPR_EXTRUDER_DRIVE_MIN,PSTR("PID drive min"));
        writeFloat(o+EPR_EXTRUDER_PID_PGAIN,PSTR("PID P-gain"));
        writeFloat(o+EPR_EXTRUDER_PID_IGAIN,PSTR("PID I-gain"));
        writeFloat(o+EPR_EXTRUDER_PID_DGAIN,PSTR("PID D-gain"));
        writeByte(o+EPR_EXTRUDER_PID_MAX,PSTR("PID max value [0-255]"));
#endif
        writeLong(o+EPR_EXTRUDER_X_OFFSET,PSTR("X-offset [steps]"));
        writeLong(o+EPR_EXTRUDER_Y_OFFSET,PSTR("Y-offset [steps]"));
        writeInt(o+EPR_EXTRUDER_WATCH_PERIOD,PSTR("temp. stabilize time [s]"));
#if RETRACT_DURING_HEATUP
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,PSTR("temp. for retraction when heating [C]"));
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,PSTR("distance to retract when heating [mm]"));
#endif
        writeByte(o+EPR_EXTRUDER_COOLER_SPEED,PSTR("extruder cooler speed [0-255]"));
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        writeFloat(o+EPR_EXTRUDER_ADVANCE_K,PSTR("advance K [0=off]"));
#endif
        writeFloat(o+EPR_EXTRUDER_ADVANCE_L,PSTR("advance L [0=off]"));
#endif
    }
#else
    OUT_P_LN("Error: No EEPROM support compiled.");
#endif
}

