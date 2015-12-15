/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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


#include "Repetier.h"


void EEPROM::update(GCode *com)
{
#if EEPROM_MODE!=0
    if(com->hasT() && com->hasP()) switch(com->T)
    {
        case 0:
		{
            if(com->hasS()) HAL::eprSetByte(com->P,(uint8_t)com->S);
            break;
		}
        case 1:
		{
            if(com->hasS()) HAL::eprSetInt16(com->P,(int)com->S);
            break;
		}
        case 2:
		{
            if(com->hasS()) HAL::eprSetInt32(com->P,(int32_t)com->S);
            break;
		}
        case 3:
		{
            if(com->hasX()) HAL::eprSetFloat(com->P,com->X);
            break;
		}
	}
    uint8_t newcheck = computeChecksum();
    if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
        HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);

    readDataFromEEPROM();
    Extruder::selectExtruderById(Extruder::current->id);
#else
	if( Printer::debugErrors() )
	{
	    Com::printErrorF(Com::tNoEEPROMSupport);
	}
#endif // EEPROM_MODE!=0

} // update


void EEPROM::restoreEEPROMSettingsFromConfiguration()
{
#if EEPROM_MODE!=0
    uint8_t version = EEPROM_PROTOCOL_VERSION;
    baudrate = BAUDRATE;
    maxInactiveTime = MAX_INACTIVE_TIME*1000L;
    stepperInactiveTime = STEPPER_INACTIVE_TIME*1000L;
    Printer::axisStepsPerMM[X_AXIS] = XAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[Y_AXIS] = YAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[Z_AXIS] = ZAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[E_AXIS] = 1;
    Printer::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
    Printer::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
    Printer::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
		Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
		Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
	}
	else
	{
		Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_MILL;
		Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_MILL;
		Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_MILL;
	}
#else
    Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X_PRINT;
    Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y_PRINT;
    Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z_PRINT;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
#if MOTHERBOARD == DEVICE_TYPE_RF1000
		Printer::HotendType = HOTEND_TYPE_V2_SINGLE;
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000
#if MOTHERBOARD == DEVICE_TYPE_RF2000
		Printer::HotendType = HOTEND_TYPE_V2_DUAL;
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

	Printer::maxJerk = MAX_JERK;
    Printer::maxZJerk = MAX_ZJERK;

#ifdef RAMP_ACCELERATION
    Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
    Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
    Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
    Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
#endif // RAMP_ACCELERATION

#if HAVE_HEATED_BED
    heatedBedController.heatManager= HEATED_BED_HEAT_MANAGER;

#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
    heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
    heatedBedController.pidPGain = HEATED_BED_PID_PGAIN;
    heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
    heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
    heatedBedController.pidMax = HEATED_BED_PID_MAX;
#endif // TEMP_PID
#endif // HAVE_HEATED_BED

    Printer::lengthMM[X_AXIS] = X_MAX_LENGTH;
    Printer::lengthMM[Y_AXIS] = Y_MAX_LENGTH;
    Printer::lengthMM[Z_AXIS] = Z_MAX_LENGTH;
    Printer::minMM[X_AXIS] = X_MIN_POS;
    Printer::minMM[Y_AXIS] = Y_MIN_POS;
    Printer::minMM[Z_AXIS] = Z_MIN_POS;

#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlash[X_AXIS] = X_BACKLASH;
    Printer::backlash[Y_AXIS] = Y_BACKLASH;
    Printer::backlash[Z_AXIS] = Z_BACKLASH;
#endif // ENABLE_BACKLASH_COMPENSATION

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
#endif // TEMP_PID

    e->yOffset = EXT0_Y_OFFSET;
    e->xOffset = EXT0_X_OFFSET;
    e->watchPeriod = EXT0_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT0_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT0_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT0_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT0_ADVANCE_L;
#endif // USE_ADVANCE
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
#endif // TEMP_PID

    e->yOffset = EXT1_Y_OFFSET;
    e->xOffset = EXT1_X_OFFSET;
    e->watchPeriod = EXT1_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT1_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT1_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT1_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT1_ADVANCE_L;
#endif // USE_ADVANCE
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
#endif // TEMP_PID

    e->yOffset = EXT2_Y_OFFSET;
    e->xOffset = EXT2_X_OFFSET;
    e->watchPeriod = EXT2_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT2_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT2_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT2_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT2_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT2_ADVANCE_L;
#endif // USE_ADVANCE
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
#endif // TEMP_PID

    e->yOffset = EXT3_Y_OFFSET;
    e->xOffset = EXT3_X_OFFSET;
    e->watchPeriod = EXT3_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT3_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT3_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT3_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT3_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT3_ADVANCE_L;
#endif // USE_ADVANCE
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
#endif // TEMP_PID

    e->yOffset = EXT4_Y_OFFSET;
    e->xOffset = EXT4_X_OFFSET;
    e->watchPeriod = EXT4_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT4_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT4_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT4_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT4_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT4_ADVANCE_L;
#endif // USE_ADVANCE
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
#endif // TEMP_PID

    e->yOffset = EXT5_Y_OFFSET;
    e->xOffset = EXT5_X_OFFSET;
    e->watchPeriod = EXT5_WATCHPERIOD;

#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT5_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT5_WAIT_RETRACT_UNITS;
#endif // RETRACT_DURING_HEATUP

    e->coolerSpeed = EXT5_EXTRUDER_COOLER_SPEED;

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT5_ADVANCE_K;
#endif // ENABLE_QUADRATIC_ADVANCE

    e->advanceL = EXT5_ADVANCE_L;
#endif // USE_ADVANCE
#endif // NUM_EXTRUDER > 5

    Printer::updateDerivedParameter();
    Extruder::selectExtruderById(Extruder::current->id);
    Extruder::initHeatedBed();

	if( Printer::debugInfo() )
	{
		Com::printInfoF(Com::tEPRConfigResetDefaults);
	}
#else
	if( Printer::debugErrors() )
	{
	    Com::printErrorF(Com::tNoEEPROMSupport);
	}
#endif // EEPROM_MODE!=0

} // restoreEEPROMSettingsFromConfiguration


void EEPROM::clearEEPROM()
{
    unsigned int i;
    for(i=0; i<2048; i++)
    {
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        HAL::eprSetByte(i,0);
    }

} // clearEEPROM


void EEPROM::storeDataIntoEEPROM(uint8_t corrupted)
{
#if EEPROM_MODE!=0
	HAL::eprSetByte(EPR_MAGIC_BYTE,EEPROM_MODE);
    HAL::eprSetInt32(EPR_BAUDRATE,baudrate);
    HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME,maxInactiveTime);
    HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME,stepperInactiveTime);
    HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[X_AXIS]);
    HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Y_AXIS]);
    HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Z_AXIS]);
    HAL::eprSetFloat(EPR_X_MAX_FEEDRATE,Printer::maxFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE,Printer::maxFeedrate[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE,Printer::maxFeedrate[Z_AXIS]);
	HAL::eprSetInt32(EPR_RF_Z_OFFSET,Printer::ZOffset);

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
	    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[X_AXIS]);
		HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[Y_AXIS]);
		HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[Z_AXIS]);
	}
	else
	{
	    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_MILL,Printer::homingFeedrate[X_AXIS]);
		HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_MILL,Printer::homingFeedrate[Y_AXIS]);
		HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_MILL,Printer::homingFeedrate[Z_AXIS]);
	}
#else
    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT,Printer::homingFeedrate[Z_AXIS]);
#endif // FEATURE_MILLING_MODE

	HAL::eprSetFloat(EPR_MAX_JERK,Printer::maxJerk);
    HAL::eprSetFloat(EPR_MAX_ZJERK,Printer::maxZJerk);

#ifdef RAMP_ACCELERATION
    HAL::eprSetFloat(EPR_X_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
    HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#endif // RAMP_ACCELERATION

#if HAVE_HEATED_BED
    HAL::eprSetByte(EPR_BED_HEAT_MANAGER,heatedBedController.heatManager);
#else
    HAL::eprSetByte(EPR_BED_HEAT_MANAGER,HEATED_BED_HEAT_MANAGER);
#endif // HAVE_HEATED_BED

#if defined(TEMP_PID) && HAVE_HEATED_BED
    HAL::eprSetByte(EPR_BED_DRIVE_MAX,heatedBedController.pidDriveMax);
    HAL::eprSetByte(EPR_BED_DRIVE_MIN,heatedBedController.pidDriveMin);
    HAL::eprSetFloat(EPR_BED_PID_PGAIN,heatedBedController.pidPGain);
    HAL::eprSetFloat(EPR_BED_PID_IGAIN,heatedBedController.pidIGain);
    HAL::eprSetFloat(EPR_BED_PID_DGAIN,heatedBedController.pidDGain);
    HAL::eprSetByte(EPR_BED_PID_MAX,heatedBedController.pidMax);
#else
    HAL::eprSetByte(EPR_BED_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MAX);
    HAL::eprSetByte(EPR_BED_DRIVE_MIN,HEATED_BED_PID_INTEGRAL_DRIVE_MIN);
    HAL::eprSetFloat(EPR_BED_PID_PGAIN,HEATED_BED_PID_PGAIN);
    HAL::eprSetFloat(EPR_BED_PID_IGAIN,HEATED_BED_PID_IGAIN);
    HAL::eprSetFloat(EPR_BED_PID_DGAIN,HEATED_BED_PID_DGAIN);
    HAL::eprSetByte(EPR_BED_PID_MAX,HEATED_BED_PID_MAX);
#endif // defined(TEMP_PID) && HAVE_HEATED_BED

    HAL::eprSetFloat(EPR_X_HOME_OFFSET,Printer::minMM[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_HOME_OFFSET,Printer::minMM[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_HOME_OFFSET,Printer::minMM[Z_AXIS]);
    HAL::eprSetFloat(EPR_X_LENGTH,Printer::lengthMM[X_AXIS]);
    HAL::eprSetFloat(EPR_Y_LENGTH,Printer::lengthMM[Y_AXIS]);
    HAL::eprSetFloat(EPR_Z_LENGTH,Printer::lengthMM[Z_AXIS]);

#if ENABLE_BACKLASH_COMPENSATION
    HAL::eprSetFloat(EPR_BACKLASH_X,Printer::backlash[X_AXIS]);
    HAL::eprSetFloat(EPR_BACKLASH_Y,Printer::backlash[Y_AXIS]);
    HAL::eprSetFloat(EPR_BACKLASH_Z,Printer::backlash[Z_AXIS]);
#else
    HAL::eprSetFloat(EPR_BACKLASH_X,0);
    HAL::eprSetFloat(EPR_BACKLASH_Y,0);
    HAL::eprSetFloat(EPR_BACKLASH_Z,0);
#endif // ENABLE_BACKLASH_COMPENSATION

    // now the extruder
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        HAL::eprSetFloat(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
        HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
        HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
        HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
        HAL::eprSetByte(o+EPR_EXTRUDER_HEAT_MANAGER,e->tempControl.heatManager);

#ifdef TEMP_PID
        HAL::eprSetByte(o+EPR_EXTRUDER_DRIVE_MAX,e->tempControl.pidDriveMax);
        HAL::eprSetByte(o+EPR_EXTRUDER_DRIVE_MIN,e->tempControl.pidDriveMin);
        HAL::eprSetFloat(o+EPR_EXTRUDER_PID_PGAIN,e->tempControl.pidPGain);
        HAL::eprSetFloat(o+EPR_EXTRUDER_PID_IGAIN,e->tempControl.pidIGain);
        HAL::eprSetFloat(o+EPR_EXTRUDER_PID_DGAIN,e->tempControl.pidDGain);
        HAL::eprSetByte(o+EPR_EXTRUDER_PID_MAX,e->tempControl.pidMax);
#endif // TEMP_PID

        HAL::eprSetFloat(o+EPR_EXTRUDER_X_OFFSET,e->xOffset/XAXIS_STEPS_PER_MM);
        HAL::eprSetFloat(o+EPR_EXTRUDER_Y_OFFSET,e->yOffset/YAXIS_STEPS_PER_MM);
        HAL::eprSetInt16(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);

#if RETRACT_DURING_HEATUP
        HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,e->waitRetractTemperature);
        HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,e->waitRetractUnits);
#else
        HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_TEMP);
        HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,EXT0_WAIT_RETRACT_UNITS);
#endif // RETRACT_DURING_HEATUP

        HAL::eprSetByte(o+EPR_EXTRUDER_COOLER_SPEED,e->coolerSpeed);

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
#else
        HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,0);
#endif // ENABLE_QUADRATIC_ADVANCE

        HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);
#else
        HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,0);
        HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_L,0);
#endif // USE_ADVANCE
    }

    if(corrupted)
    {
#if FEATURE_MILLING_MODE
		if( Printer::operatingMode == OPERATING_MODE_PRINT )
		{
			HAL::eprSetInt32(EPR_PRINTING_TIME,0);
			HAL::eprSetFloat(EPR_PRINTING_DISTANCE,0);
#if FEATURE_SERVICE_INTERVAL
			HAL::eprSetInt32(EPR_PRINTING_TIME_SERVICE,0);
			HAL::eprSetFloat(EPR_PRINTING_DISTANCE_SERVICE,0);
#endif // FEATURE_SERVICE_INTERVAL
		}
		else
		{
			HAL::eprSetInt32(EPR_MILLING_TIME,0);
#if FEATURE_SERVICE_INTERVAL
			HAL::eprSetInt32(EPR_MILLING_TIME_SERVICE,0);
#endif // FEATURE_SERVICE_INTERVAL
		}
#else
		HAL::eprSetInt32(EPR_PRINTING_TIME,0);
		HAL::eprSetFloat(EPR_PRINTING_DISTANCE,0);
#if FEATURE_SERVICE_INTERVAL
		HAL::eprSetInt32(EPR_PRINTING_TIME_SERVICE,0);
		HAL::eprSetFloat(EPR_PRINTING_DISTANCE_SERVICE,0);
#endif // FEATURE_SERVICE_INTERVAL
#endif // FEATURE_MILLING_MODE
	}

#if FEATURE_BEEPER
	HAL::eprSetByte( EPR_RF_BEEPER_MODE, Printer::enableBeeper );
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
	HAL::eprSetByte( EPR_RF_CASE_LIGHT_MODE, Printer::enableCaseLight );
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
	HAL::eprSetByte( EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode );
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_24V_FET_OUTPUTS
	HAL::eprSetByte( EPR_RF_FET1_MODE, Printer::enableFET1 );
	HAL::eprSetByte( EPR_RF_FET2_MODE, Printer::enableFET2 );
	HAL::eprSetByte( EPR_RF_FET3_MODE, Printer::enableFET3 );
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_230V_OUTPUT
	HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, Printer::enable230VOutput );
#endif // FEATURE_230V_OUTPUT

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	HAL::eprSetByte( EPR_RF_Z_ENDSTOP_TYPE, Printer::ZEndstopType );
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_MILLING_MODE
	HAL::eprSetByte( EPR_RF_OPERATING_MODE, Printer::operatingMode );
#endif // FEATURE_MILLING_MODE > 0

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
	HAL::eprSetByte( EPR_RF_HOTEND_TYPE, Printer::HotendType );
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

#if FEATURE_CONFIGURABLE_MILLER_TYPE
	HAL::eprSetByte( EPR_RF_MILLER_TYPE, Printer::MillerType );
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

	// Save version and build checksum
    HAL::eprSetByte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
    HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif // EEPROM_MODE!=0

} // storeDataIntoEEPROM


void EEPROM::updateChecksum()
{
#if EEPROM_MODE!=0
	HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif // EEPROM_MODE!=0

} // updateChecksum

void EEPROM::initializeAllOperatingModes()
{
#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		// initialize the EEPROM values of the operating mode which is not active at the moment
	    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_MILL,HOMING_FEEDRATE_X_MILL);
		HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_MILL,HOMING_FEEDRATE_Y_MILL);
		HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_MILL,HOMING_FEEDRATE_Z_MILL);
	}
	else
	{
		// initialize the EEPROM values of the operating mode which is not active at the moment
	    HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_X_PRINT);
		HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_Y_PRINT);
		HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT,HOMING_FEEDRATE_Z_PRINT);
	}
#endif // FEATURE_MILLING_MODE

} // initializeAllOperatingModes


void EEPROM::readDataFromEEPROM()
{
#if EEPROM_MODE!=0
    uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
    baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    maxInactiveTime = HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME);
    stepperInactiveTime = HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
    Printer::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);
    Printer::maxFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE);
    Printer::maxFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_FEEDRATE);
    Printer::maxFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_FEEDRATE);
	Printer::ZOffset = HAL::eprGetInt32(EPR_RF_Z_OFFSET);

    Printer::maxJerk = HAL::eprGetFloat(EPR_MAX_JERK);
    Printer::maxZJerk = HAL::eprGetFloat(EPR_MAX_ZJERK);

#ifdef RAMP_ACCELERATION
    Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL);
#endif // RAMP_ACCELERATION

#if HAVE_HEATED_BED
    heatedBedController.heatManager= HAL::eprGetByte(EPR_BED_HEAT_MANAGER);

#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HAL::eprGetByte(EPR_BED_DRIVE_MAX);
    heatedBedController.pidDriveMin = HAL::eprGetByte(EPR_BED_DRIVE_MIN);
    heatedBedController.pidPGain = HAL::eprGetFloat(EPR_BED_PID_PGAIN);
    heatedBedController.pidIGain = HAL::eprGetFloat(EPR_BED_PID_IGAIN);
    heatedBedController.pidDGain = HAL::eprGetFloat(EPR_BED_PID_DGAIN);
    heatedBedController.pidMax = HAL::eprGetByte(EPR_BED_PID_MAX);
#endif // TEMP_PID
#endif // HAVE_HEATED_BED

    Printer::minMM[X_AXIS] = HAL::eprGetFloat(EPR_X_HOME_OFFSET);
    Printer::minMM[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOME_OFFSET);
    Printer::minMM[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOME_OFFSET);
    Printer::lengthMM[X_AXIS] = HAL::eprGetFloat(EPR_X_LENGTH);
    Printer::lengthMM[Y_AXIS] = HAL::eprGetFloat(EPR_Y_LENGTH);
    Printer::lengthMM[Z_AXIS] = HAL::eprGetFloat(EPR_Z_LENGTH);

#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlash[X_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_X);
    Printer::backlash[Y_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Y);
    Printer::backlash[Z_AXIS] = HAL::eprGetFloat(EPR_BACKLASH_Z);
#endif // ENABLE_BACKLASH_COMPENSATION

    // now the extruder
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        e->stepsPerMM = HAL::eprGetFloat(o+EPR_EXTRUDER_STEPS_PER_MM);
        e->maxFeedrate = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE);
        e->maxStartFeedrate = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
        e->maxAcceleration = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION);
        e->tempControl.heatManager = HAL::eprGetByte(o+EPR_EXTRUDER_HEAT_MANAGER);

#ifdef TEMP_PID
        e->tempControl.pidDriveMax = HAL::eprGetByte(o+EPR_EXTRUDER_DRIVE_MAX);
        e->tempControl.pidDriveMin = HAL::eprGetByte(o+EPR_EXTRUDER_DRIVE_MIN);
        e->tempControl.pidPGain    = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_PGAIN);
        e->tempControl.pidIGain	   = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_IGAIN);
        e->tempControl.pidDGain	   = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_DGAIN);
        e->tempControl.pidMax	   = HAL::eprGetByte(o+EPR_EXTRUDER_PID_MAX);
#endif // TEMP_PID

        e->xOffset = int32_t(HAL::eprGetFloat(o+EPR_EXTRUDER_X_OFFSET)*XAXIS_STEPS_PER_MM);
        e->yOffset = int32_t(HAL::eprGetFloat(o+EPR_EXTRUDER_Y_OFFSET)*YAXIS_STEPS_PER_MM);
        e->watchPeriod = HAL::eprGetInt16(o+EPR_EXTRUDER_WATCH_PERIOD);

#if RETRACT_DURING_HEATUP
        e->waitRetractTemperature = HAL::eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP);
        e->waitRetractUnits = HAL::eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif // RETRACT_DURING_HEATUP

#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        e->advanceK = HAL::eprGetFloat(o+EPR_EXTRUDER_ADVANCE_K);
#endif // ENABLE_QUADRATIC_ADVANCE
        e->advanceL = HAL::eprGetFloat(o+EPR_EXTRUDER_ADVANCE_L);
#endif // USE_ADVANCE

        if(version>1)
            e->coolerSpeed = HAL::eprGetByte(o+EPR_EXTRUDER_COOLER_SPEED);
    }

#if FEATURE_BEEPER
	Printer::enableBeeper = HAL::eprGetByte( EPR_RF_BEEPER_MODE );
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
	Printer::enableCaseLight = HAL::eprGetByte( EPR_RF_CASE_LIGHT_MODE );
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
	Printer::RGBLightMode = HAL::eprGetByte( EPR_RF_RGB_LIGHT_MODE );
	if ( Printer::RGBLightMode == RGB_MODE_AUTOMATIC)
	{
		Printer::RGBLightStatus	   = RGB_STATUS_AUTOMATIC;
		Printer::RGBLightIdleStart = 0;
	}
	else
	{
		Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
	}
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_24V_FET_OUTPUTS
	Printer::enableFET1 = HAL::eprGetByte( EPR_RF_FET1_MODE );
	Printer::enableFET2 = HAL::eprGetByte( EPR_RF_FET2_MODE );
	Printer::enableFET3 = HAL::eprGetByte( EPR_RF_FET3_MODE );
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_230V_OUTPUT
	Printer::enable230VOutput = HAL::eprGetByte( EPR_RF_230V_OUTPUT_MODE );
#endif // FEATURE_230V_OUTPUT

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	Printer::ZEndstopType  = HAL::eprGetByte( EPR_RF_Z_ENDSTOP_TYPE ) == ENDSTOP_TYPE_CIRCUIT ? ENDSTOP_TYPE_CIRCUIT : ENDSTOP_TYPE_SINGLE;
#endif //FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_MILLING_MODE
	Printer::operatingMode = HAL::eprGetByte( EPR_RF_OPERATING_MODE ) == OPERATING_MODE_MILL ? OPERATING_MODE_MILL : OPERATING_MODE_PRINT;
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT);
		Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT);
		Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT);
	}
	else
	{
		Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_MILL);
		Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_MILL);
		Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_MILL);
	}
#else
    Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE_PRINT);
    Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE_PRINT);
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
	Printer::HotendType = HAL::eprGetByte( EPR_RF_HOTEND_TYPE );
	if( Printer::HotendType < HOTEND_TYPE_1 || HOTEND_TYPE_V2_DUAL )
	{
#if MOTHERBOARD == DEVICE_TYPE_RF1000
		Printer::HotendType = HOTEND_TYPE_V2_SINGLE;
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if MOTHERBOARD == DEVICE_TYPE_RF2000
		Printer::HotendType = HOTEND_TYPE_V2_DUAL;
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000
	}
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

#if FEATURE_CONFIGURABLE_MILLER_TYPE
	Printer::MillerType = HAL::eprGetByte( EPR_RF_MILLER_TYPE ) == MILLER_TYPE_ONE_TRACK ? MILLER_TYPE_ONE_TRACK : MILLER_TYPE_TWO_TRACKS;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

	if(version!=EEPROM_PROTOCOL_VERSION)
    {
		if( Printer::debugInfo() )
		{
	        Com::printInfoFLN(Com::tEPRProtocolChanged);
		}

		storeDataIntoEEPROM(false); // Store new fields for changed version
    }
    Printer::updateDerivedParameter();
    Extruder::initHeatedBed();
#endif // EEPROM_MODE!=0

} // readDataFromEEPROM


void EEPROM::initBaudrate()
{
#if EEPROM_MODE!=0
    if(HAL::eprGetByte(EPR_MAGIC_BYTE)==EEPROM_MODE)
    {
        baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
#endif // EEPROM_MODE!=0

} // initBaudrate


void EEPROM::init()
{
#if EEPROM_MODE!=0
    uint8_t check = computeChecksum();
    uint8_t storedcheck = HAL::eprGetByte(EPR_INTEGRITY_BYTE);

    if(HAL::eprGetByte(EPR_MAGIC_BYTE)==EEPROM_MODE && storedcheck==check)
    {
        readDataFromEEPROM();
    }
    else
    {
#if FEATURE_FULL_EEPROM_RESET
		clearEEPROM();
		restoreEEPROMSettingsFromConfiguration();
#endif // FEATURE_FULL_EEPROM_RESET

        storeDataIntoEEPROM(storedcheck!=check);
		initializeAllOperatingModes();
    }
#endif // EEPROM_MODE!=0

} // init


void EEPROM::updatePrinterUsage()
{
#if EEPROM_MODE!=0
#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		if(Printer::filamentPrinted==0) return; // No miles only enabled
		uint32_t seconds = (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000;
		seconds += HAL::eprGetInt32(EPR_PRINTING_TIME);
		HAL::eprSetInt32(EPR_PRINTING_TIME,seconds);
		HAL::eprSetFloat(EPR_PRINTING_DISTANCE,HAL::eprGetFloat(EPR_PRINTING_DISTANCE)+Printer::filamentPrinted*0.001);

#if FEATURE_SERVICE_INTERVAL
		uint32_t uSecondsServicePrint = (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000;
		uSecondsServicePrint += HAL::eprGetInt32(EPR_PRINTING_TIME_SERVICE);
		HAL::eprSetInt32(EPR_PRINTING_TIME_SERVICE,uSecondsServicePrint);
		HAL::eprSetFloat(EPR_PRINTING_DISTANCE_SERVICE,HAL::eprGetFloat(EPR_PRINTING_DISTANCE_SERVICE)+Printer::filamentPrinted*0.001);
#endif // FEATURE_SERVICE_INTERVAL

		Printer::filamentPrinted = 0;
		Printer::msecondsPrinting = HAL::timeInMilliseconds();
		uint8_t newcheck = computeChecksum();
		if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
		{
			HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
		}
		Commands::reportPrinterUsage();
	}
	if( Printer::operatingMode == OPERATING_MODE_MILL )
	{
		uint32_t seconds = (HAL::timeInMilliseconds()-Printer::msecondsMilling)/1000;
		seconds += HAL::eprGetInt32(EPR_MILLING_TIME);
		HAL::eprSetInt32(EPR_MILLING_TIME,seconds);
#if FEATURE_SERVICE_INTERVAL
		uint32_t uSecondsServicePrint = (HAL::timeInMilliseconds()-Printer::msecondsMilling)/1000;
		uSecondsServicePrint += HAL::eprGetInt32(EPR_MILLING_TIME_SERVICE);
		HAL::eprSetInt32(EPR_MILLING_TIME_SERVICE,uSecondsServicePrint);
#endif // FEATURE_SERVICE_INTERVAL

		Printer::msecondsMilling = HAL::timeInMilliseconds();
		uint8_t newcheck = computeChecksum();
		if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
		{
			HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
		}
		Commands::reportPrinterUsage();
	}
#else
	if(Printer::filamentPrinted==0) return; // No miles only enabled
    uint32_t seconds = (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000;
    seconds += HAL::eprGetInt32(EPR_PRINTING_TIME);
    HAL::eprSetInt32(EPR_PRINTING_TIME,seconds);
    HAL::eprSetFloat(EPR_PRINTING_DISTANCE,HAL::eprGetFloat(EPR_PRINTING_DISTANCE)+Printer::filamentPrinted*0.001);

#if FEATURE_SERVICE_INTERVAL
	uint32_t uSecondsServicePrint = (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000;
	uSecondsServicePrint += HAL::eprGetInt32(EPR_PRINTING_TIME_SERVICE);
	HAL::eprSetInt32(EPR_PRINTING_TIME_SERVICE,uSecondsServicePrint);
	HAL::eprSetFloat(EPR_PRINTING_DISTANCE_SERVICE,HAL::eprGetFloat(EPR_PRINTING_DISTANCE_SERVICE)+Printer::filamentPrinted*0.001);
#endif // FEATURE_SERVICE_INTERVAL

	Printer::filamentPrinted = 0;
    Printer::msecondsPrinting = HAL::timeInMilliseconds();
    uint8_t newcheck = computeChecksum();
    if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
	{
        HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
	}
    Commands::reportPrinterUsage();

#endif // FEATURE_MILLING_MODE
#endif // EEPROM_MODE

} // updatePrinterUsage


int EEPROM::getExtruderOffset(uint8_t extruder)
{
	return extruder * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;

} // getExtruderOffset


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
    writeLong(EPR_BAUDRATE,Com::tEPRBaudrate);
    writeLong(EPR_MAX_INACTIVE_TIME,Com::tEPRMaxInactiveTime);
    writeLong(EPR_STEPPER_INACTIVE_TIME,Com::tEPRStopAfterInactivty);
    writeFloat(EPR_XAXIS_STEPS_PER_MM,Com::tEPRXStepsPerMM,4);
    writeFloat(EPR_YAXIS_STEPS_PER_MM,Com::tEPRYStepsPerMM,4);
	writeFloat(EPR_ZAXIS_STEPS_PER_MM,Com::tEPRZStepsPerMM,4);
	writeFloat(EPR_X_MAX_FEEDRATE,Com::tEPRXMaxFeedrate);
    writeFloat(EPR_Y_MAX_FEEDRATE,Com::tEPRYMaxFeedrate);
    writeFloat(EPR_Z_MAX_FEEDRATE,Com::tEPRZMaxFeedrate);
	writeLong(EPR_RF_Z_OFFSET,Com::tEPRZOffset);

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
		writeFloat(EPR_PRINTING_DISTANCE,Com::tEPRFilamentPrinted);
		writeLong(EPR_PRINTING_TIME,Com::tEPRPrinterActive);

#if FEATURE_SERVICE_INTERVAL
		writeFloat(EPR_PRINTING_DISTANCE_SERVICE,Com::tEPRFilamentPrintedService);
		writeLong(EPR_PRINTING_TIME_SERVICE,Com::tEPRPrinterActiveService);
#endif // FEATURE_SERVICE_INTERVAL
	}
	else
	{
		writeLong(EPR_MILLING_TIME,Com::tEPRMillerActive);
#if FEATURE_SERVICE_INTERVAL
		writeLong(EPR_MILLING_TIME_SERVICE,Com::tEPRMillerActiveService);
#endif // FEATURE_SERVICE_INTERVAL
	}
#else
	writeFloat(EPR_PRINTING_DISTANCE,Com::tEPRFilamentPrinted);
	writeLong(EPR_PRINTING_TIME,Com::tEPRPrinterActive);

#if FEATURE_SERVICE_INTERVAL
	writeFloat(EPR_PRINTING_DISTANCE_SERVICE,Com::tEPRFilamentPrintedService);
	writeLong(EPR_PRINTING_TIME_SERVICE,Com::tEPRPrinterActiveService);
#endif // FEATURE_SERVICE_INTERVAL
#endif // FEATURE_MILLING_MODE

#if FEATURE_MILLING_MODE
	if( Printer::operatingMode == OPERATING_MODE_PRINT )
	{
	    writeFloat(EPR_X_HOMING_FEEDRATE_PRINT,Com::tEPRXHomingFeedrate);
		writeFloat(EPR_Y_HOMING_FEEDRATE_PRINT,Com::tEPRYHomingFeedrate);
		writeFloat(EPR_Z_HOMING_FEEDRATE_PRINT,Com::tEPRZHomingFeedrate);
	}
	else
	{
		writeFloat(EPR_X_HOMING_FEEDRATE_MILL,Com::tEPRXHomingFeedrate);
		writeFloat(EPR_Y_HOMING_FEEDRATE_MILL,Com::tEPRYHomingFeedrate);
		writeFloat(EPR_Z_HOMING_FEEDRATE_MILL,Com::tEPRZHomingFeedrate);
	}
#else
    writeFloat(EPR_X_HOMING_FEEDRATE_PRINT,Com::tEPRXHomingFeedrate);
    writeFloat(EPR_Y_HOMING_FEEDRATE_PRINT,Com::tEPRYHomingFeedrate);
	writeFloat(EPR_Z_HOMING_FEEDRATE_PRINT,Com::tEPRZHomingFeedrate);
#endif // FEATURE_MILLING_MODE

	writeFloat(EPR_MAX_JERK,Com::tEPRMaxJerk);
    writeFloat(EPR_MAX_ZJERK,Com::tEPRMaxZJerk);
    writeFloat(EPR_X_HOME_OFFSET,Com::tEPRXHomePos);
    writeFloat(EPR_Y_HOME_OFFSET,Com::tEPRYHomePos);
    writeFloat(EPR_Z_HOME_OFFSET,Com::tEPRZHomePos);
    writeFloat(EPR_X_LENGTH,Com::tEPRXMaxLength);
    writeFloat(EPR_Y_LENGTH,Com::tEPRYMaxLength);
    writeFloat(EPR_Z_LENGTH,Com::tEPRZMaxLength);

#if ENABLE_BACKLASH_COMPENSATION
    writeFloat(EPR_BACKLASH_X,Com::tEPRXBacklash);
    writeFloat(EPR_BACKLASH_Y,Com::tEPRYBacklash);
    writeFloat(EPR_BACKLASH_Z,Com::tEPRZBacklash);
#endif // ENABLE_BACKLASH_COMPENSATION

#ifdef RAMP_ACCELERATION
    writeFloat(EPR_X_MAX_ACCEL,Com::tEPRXAcceleration);
    writeFloat(EPR_Y_MAX_ACCEL,Com::tEPRYAcceleration);
    writeFloat(EPR_Z_MAX_ACCEL,Com::tEPRZAcceleration);
    writeFloat(EPR_X_MAX_TRAVEL_ACCEL,Com::tEPRXTravelAcceleration);
    writeFloat(EPR_Y_MAX_TRAVEL_ACCEL,Com::tEPRYTravelAcceleration);
    writeFloat(EPR_Z_MAX_TRAVEL_ACCEL,Com::tEPRZTravelAcceleration);
#endif // RAMP_ACCELERATION

#if HAVE_HEATED_BED
    writeByte(EPR_BED_HEAT_MANAGER,Com::tEPRBedHeatManager);

#ifdef TEMP_PID
    writeByte(EPR_BED_DRIVE_MAX,Com::tEPRBedPIDDriveMax);
    writeByte(EPR_BED_DRIVE_MIN,Com::tEPRBedPIDDriveMin);
    writeFloat(EPR_BED_PID_PGAIN,Com::tEPRBedPGain);
    writeFloat(EPR_BED_PID_IGAIN,Com::tEPRBedIGain);
    writeFloat(EPR_BED_PID_DGAIN,Com::tEPRBedDGain);
    writeByte(EPR_BED_PID_MAX,Com::tEPRBedPISMaxValue);
#endif // TEMP_PID
#endif // HAVE_HEATED_BED

    // now the extruder
    for(uint8_t i=0; i<NUM_EXTRUDER; i++)
    {
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        writeFloat(o+EPR_EXTRUDER_STEPS_PER_MM,Com::tEPRStepsPerMM);
        writeFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,Com::tEPRMaxFeedrate);
        writeFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,Com::tEPRStartFeedrate);
        writeFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,Com::tEPRAcceleration);
        writeByte(o+EPR_EXTRUDER_HEAT_MANAGER,Com::tEPRHeatManager);

#ifdef TEMP_PID
        writeByte(o+EPR_EXTRUDER_DRIVE_MAX,Com::tEPRDriveMax);
        writeByte(o+EPR_EXTRUDER_DRIVE_MIN,Com::tEPRDriveMin);
        writeFloat(o+EPR_EXTRUDER_PID_PGAIN,Com::tEPRPGain,4);
        writeFloat(o+EPR_EXTRUDER_PID_IGAIN,Com::tEPRIGain,4);
        writeFloat(o+EPR_EXTRUDER_PID_DGAIN,Com::tEPRDGain,4);
        writeByte(o+EPR_EXTRUDER_PID_MAX,Com::tEPRPIDMaxValue);
#endif // TEMP_PID

        writeFloat(o+EPR_EXTRUDER_X_OFFSET,Com::tEPRXOffset);
        writeFloat(o+EPR_EXTRUDER_Y_OFFSET,Com::tEPRYOffset);
        writeInt(o+EPR_EXTRUDER_WATCH_PERIOD,Com::tEPRStabilizeTime);

#if RETRACT_DURING_HEATUP
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,Com::tEPRRetractionWhenHeating);
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,Com::tEPRDistanceRetractHeating);
#endif // RETRACT_DURING_HEATUP

        writeByte(o+EPR_EXTRUDER_COOLER_SPEED,Com::tEPRExtruderCoolerSpeed);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        writeFloat(o+EPR_EXTRUDER_ADVANCE_K,Com::tEPRAdvanceK);
#endif // ENABLE_QUADRATIC_ADVANCE
        writeFloat(o+EPR_EXTRUDER_ADVANCE_L,Com::tEPRAdvanceL);
#endif // USE_ADVANCE
    }

	// RF specific
#if FEATURE_BEEPER
	writeByte(EPR_RF_BEEPER_MODE,Com::tEPRBeeperMode);
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
	writeByte(EPR_RF_CASE_LIGHT_MODE,Com::tEPRCaseLightsMode);
#endif // FEATURE_CASE_LIGHT

#if FEATURE_MILLING_MODE
	writeByte(EPR_RF_OPERATING_MODE,Com::tEPROperatingMode);
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
	writeByte(EPR_RF_Z_ENDSTOP_TYPE,Com::tEPRZEndstopType);
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_RGB_LIGHT_EFFECTS
	writeByte(EPR_RF_RGB_LIGHT_MODE,Com::tEPRRGBLightMode);
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_24V_FET_OUTPUTS
	writeByte(EPR_RF_FET1_MODE,Com::tEPRFET1Mode);
	writeByte(EPR_RF_FET2_MODE,Com::tEPRFET2Mode);
	writeByte(EPR_RF_FET3_MODE,Com::tEPRFET3Mode);
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_230V_OUTPUT
	writeByte(EPR_RF_230V_OUTPUT_MODE,Com::tEPR230VOutputMode);
#endif // FEATURE_230V_OUTPUT

#if FEATURE_CONFIGURABLE_HOTEND_TYPE
	writeByte(EPR_RF_HOTEND_TYPE,Com::tEPRHotendType);
#endif // FEATURE_CONFIGURABLE_HOTEND_TYPE

#if FEATURE_CONFIGURABLE_MILLER_TYPE
	writeByte(EPR_RF_MILLER_TYPE,Com::tEPRMillerType);
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#else
	if( Printer::debugErrors() )
	{
	    Com::printErrorF(Com::tNoEEPROMSupport);
	}
#endif // EEPROM_MODE!=0

} // writeSettings


#if EEPROM_MODE!=0
uint8_t EEPROM::computeChecksum()
{
    unsigned int	i;
    uint8_t			checksum=0;


    for(i=0; i<2048; i++)
    {
#if FEATURE_WATCHDOG
		HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

		if(i==EEPROM_OFFSET+EPR_INTEGRITY_BYTE) continue;
        checksum += HAL::eprGetByte(i);
    }
    return checksum;

} // computeChecksum


void EEPROM::writeExtruderPrefix(uint pos)
{
    if(pos<EEPROM_EXTRUDER_OFFSET || pos>=800) return;
    int n = (pos-EEPROM_EXTRUDER_OFFSET)/EEPROM_EXTRUDER_LENGTH+1;
	Com::printF(Com::tExtrDot,n);
    Com::print(' ');

} // writeExtruderPrefix


void EEPROM::writeFloat(uint pos,PGM_P text,uint8_t digits)
{
    Com::printF(Com::tEPR3,(int)pos);
    Com::print(' ');
    Com::printFloat(HAL::eprGetFloat(pos),digits);
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);

} // writeFloat


void EEPROM::writeLong(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR2,(int)pos);
    Com::print(' ');
    Com::print(HAL::eprGetInt32(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);

} // writeLong


void EEPROM::writeInt(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR1,(int)pos);
    Com::print(' ');
    Com::print(HAL::eprGetInt16(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);

} // writeInt


void EEPROM::writeByte(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR0,(int)pos);
    Com::print(' ');
    Com::print((int)HAL::eprGetByte(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);

} // writeByte
#endif // EEPROM_MODE!=0

