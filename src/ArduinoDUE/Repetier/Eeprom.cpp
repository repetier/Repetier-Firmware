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
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm
  firmware.

  Functions in this file are used to communicate using ascii or repetier
  protocol.
*/

#include "Repetier.h"

void EEPROM::update(GCode *com) {
#if EEPROM_MODE != 0
  if (com->hasT() && com->hasP())
    switch (com->T) {
    case 0:
      if (com->hasS())
        HAL::eprSetByte(com->P, (uint8_t)com->S);
      break;
    case 1:
      if (com->hasS())
        HAL::eprSetInt16(com->P, (int16_t)com->S);
      break;
    case 2:
      if (com->hasS())
        HAL::eprSetInt32(com->P, (int32_t)com->S);
      break;
    case 3:
      if (com->hasX())
        HAL::eprSetFloat(com->P, com->X);
      break;
    }
  uint8_t newcheck = computeChecksum();
  if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
  bool includesEeprom =
      com->P >= EEPROM_EXTRUDER_OFFSET &&
      com->P < EEPROM_EXTRUDER_OFFSET + 6 * EEPROM_EXTRUDER_LENGTH;
  readDataFromEEPROM(includesEeprom);
#if MIXING_EXTRUDER
  Extruder::selectExtruderById(Extruder::activeMixingExtruder);
#else
  Extruder::selectExtruderById(Extruder::current->id);
#endif
#else
  Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::restoreEEPROMSettingsFromConfiguration() {
  // can only be done right if we also update permanent values not cached!
#if EEPROM_MODE != 0
  EEPROM::initalizeUncached();
  uint8_t newcheck = computeChecksum();
  if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
  baudrate = BAUDRATE;
  maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
  stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;
  Printer::axisStepsPerMM[X_AXIS] = XAXIS_STEPS_PER_MM;
  Printer::axisStepsPerMM[Y_AXIS] = YAXIS_STEPS_PER_MM;
  Printer::axisStepsPerMM[Z_AXIS] = ZAXIS_STEPS_PER_MM;
  Printer::axisStepsPerMM[E_AXIS] = 1;
  Printer::maxFeedrate[X_AXIS] = MAX_FEEDRATE_X;
  Printer::maxFeedrate[Y_AXIS] = MAX_FEEDRATE_Y;
  Printer::maxFeedrate[Z_AXIS] = MAX_FEEDRATE_Z;
  Printer::homingFeedrate[X_AXIS] = HOMING_FEEDRATE_X;
  Printer::homingFeedrate[Y_AXIS] = HOMING_FEEDRATE_Y;
  Printer::homingFeedrate[Z_AXIS] = HOMING_FEEDRATE_Z;
  Printer::maxJerk = MAX_JERK;
#if DRIVE_SYSTEM != DELTA
  Printer::maxZJerk = MAX_ZJERK;
#endif
#if RAMP_ACCELERATION
  Printer::maxAccelerationMMPerSquareSecond[X_AXIS] =
      MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X;
  Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] =
      MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
  Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] =
      MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
  Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] =
      MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X;
  Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] =
      MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y;
  Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] =
      MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z;
#endif
#if HAVE_HEATED_BED
  heatedBedController.heatManager = HEATED_BED_HEAT_MANAGER;
  heatedBedController.preheatTemperature = HEATED_BED_PREHEAT_TEMP;
  heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
  heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
  heatedBedController.pidPGain = HEATED_BED_PID_PGAIN_OR_DEAD_TIME;
  heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
  heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
  heatedBedController.pidMax = HEATED_BED_PID_MAX;
#if ENABLED(TEMP_GAIN)
  heatedBedController.tempGain = 1.0;
  heatedBedController.tempBias = 0.0;
#endif
#endif
  Printer::xLength = X_MAX_LENGTH;
  Printer::yLength = Y_MAX_LENGTH;
  Printer::zLength = Z_MAX_LENGTH;
  Printer::xMin = X_MIN_POS;
  Printer::yMin = Y_MIN_POS;
  Printer::zMin = Z_MIN_POS;
#if DUAL_X_AXIS_MODE > 0 && LAZY_DUAL_X_AXIS == 0
  Printer::x1Min = Printer::xMin;
  Printer::x1Length = Printer::xLength;
#endif
#if NONLINEAR_SYSTEM
#ifdef ROD_RADIUS
  Printer::radius0 = ROD_RADIUS;
#else
  Printer::radius0 = 0;
#endif
#endif
#if ENABLE_BACKLASH_COMPENSATION
  Printer::backlashX = X_BACKLASH;
  Printer::backlashY = Y_BACKLASH;
  Printer::backlashZ = Z_BACKLASH;
#endif
  Extruder *e;
#if NUM_EXTRUDER > 0
  e = &extruder[0];
  e->stepsPerMM = EXT0_STEPS_PER_MM;
  e->maxFeedrate = EXT0_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT0_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT0_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT0_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT0_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT0_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT0_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT0_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT0_PID_I;
  e->tempControl.pidDGain = EXT0_PID_D;
  e->tempControl.pidMax = EXT0_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT0_Y_OFFSET;
  e->xOffset = EXT0_X_OFFSET;
  e->watchPeriod = EXT0_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT0_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT0_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT0_ADVANCE_K;
#endif
  e->advanceL = EXT0_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER>0
#if NUM_EXTRUDER > 1
  e = &extruder[1];
  e->stepsPerMM = EXT1_STEPS_PER_MM;
  e->maxFeedrate = EXT1_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT1_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT1_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT1_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT1_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT1_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT1_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT1_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT1_PID_I;
  e->tempControl.pidDGain = EXT1_PID_D;
  e->tempControl.pidMax = EXT1_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT1_Y_OFFSET;
  e->xOffset = EXT1_X_OFFSET;
  e->watchPeriod = EXT1_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT1_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT1_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT1_ADVANCE_K;
#endif
  e->advanceL = EXT1_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 1
#if NUM_EXTRUDER > 2
  e = &extruder[2];
  e->stepsPerMM = EXT2_STEPS_PER_MM;
  e->maxFeedrate = EXT2_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT2_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT2_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT2_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT2_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT2_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT2_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT2_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT2_PID_I;
  e->tempControl.pidDGain = EXT2_PID_D;
  e->tempControl.pidMax = EXT2_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT2_Y_OFFSET;
  e->xOffset = EXT2_X_OFFSET;
  e->watchPeriod = EXT2_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT2_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT2_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT2_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT2_ADVANCE_K;
#endif
  e->advanceL = EXT2_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 2
#if NUM_EXTRUDER > 3
  e = &extruder[3];
  e->stepsPerMM = EXT3_STEPS_PER_MM;
  e->maxFeedrate = EXT3_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT3_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT3_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT3_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT3_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT3_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT3_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT3_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT3_PID_I;
  e->tempControl.pidDGain = EXT3_PID_D;
  e->tempControl.pidMax = EXT3_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT3_Y_OFFSET;
  e->xOffset = EXT3_X_OFFSET;
  e->watchPeriod = EXT3_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT3_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT3_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT3_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT3_ADVANCE_K;
#endif
  e->advanceL = EXT3_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 3
#if NUM_EXTRUDER > 4
  e = &extruder[4];
  e->stepsPerMM = EXT4_STEPS_PER_MM;
  e->maxFeedrate = EXT4_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT4_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT4_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT4_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT4_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT4_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT4_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT4_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT4_PID_I;
  e->tempControl.pidDGain = EXT4_PID_D;
  e->tempControl.pidMax = EXT4_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT4_Y_OFFSET;
  e->xOffset = EXT4_X_OFFSET;
  e->watchPeriod = EXT4_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT4_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT4_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT4_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT4_ADVANCE_K;
#endif
  e->advanceL = EXT4_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 4
#if NUM_EXTRUDER > 5
  e = &extruder[5];
  e->stepsPerMM = EXT5_STEPS_PER_MM;
  e->maxFeedrate = EXT5_MAX_FEEDRATE;
  e->maxStartFeedrate = EXT5_MAX_START_FEEDRATE;
  e->maxAcceleration = EXT5_MAX_ACCELERATION;
  e->tempControl.heatManager = EXT5_HEAT_MANAGER;
  e->tempControl.preheatTemperature = EXT5_PREHEAT_TEMP;
  e->tempControl.pidDriveMax = EXT5_PID_INTEGRAL_DRIVE_MAX;
  e->tempControl.pidDriveMin = EXT5_PID_INTEGRAL_DRIVE_MIN;
  e->tempControl.pidPGain = EXT5_PID_PGAIN_OR_DEAD_TIME;
  e->tempControl.pidIGain = EXT5_PID_I;
  e->tempControl.pidDGain = EXT5_PID_D;
  e->tempControl.pidMax = EXT5_PID_MAX;
#if ENABLED(TEMP_GAIN)
  e->tempControl.tempGain = 1.0;
  e->tempControl.tempBias = 0.0;
#endif
  e->yOffset = EXT5_Y_OFFSET;
  e->xOffset = EXT5_X_OFFSET;
  e->watchPeriod = EXT5_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
  e->waitRetractTemperature = EXT5_WAIT_RETRACT_TEMP;
  e->waitRetractUnits = EXT5_WAIT_RETRACT_UNITS;
#endif
  e->coolerSpeed = EXT5_EXTRUDER_COOLER_SPEED;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  e->advanceK = EXT5_ADVANCE_K;
#endif
  e->advanceL = EXT5_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 5
#if FEATURE_AUTOLEVEL
  Printer::setAutolevelActive(false);
  Printer::resetTransformationMatrix(true);
#endif
#if MIXING_EXTRUDER
  restoreMixingRatios();
#endif
  initalizeUncached();
  Printer::updateDerivedParameter();
#if MIXING_EXTRUDER
  Extruder::selectExtruderById(Extruder::activeMixingExtruder);
#else
  Extruder::selectExtruderById(Extruder::current->id);
#endif
  Extruder::initHeatedBed();
  Com::printInfoFLN(Com::tEPRConfigResetDefaults);
#else
  Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted) {
#if EEPROM_MODE != 0
  HAL::eprSetInt32(EPR_BAUDRATE, baudrate);
  HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, maxInactiveTime);
  HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, stepperInactiveTime);
//#define EPR_ACCELERATION_TYPE 1
#if DUAL_X_RESOLUTION
  HAL::eprSetFloat(EPR_X2AXIS_STEPS_PER_MM, Printer::axisX2StepsPerMM);
  HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM, Printer::axisX1StepsPerMM);
#else
  HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[X_AXIS]);
#endif
  HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[Y_AXIS]);
  HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM, Printer::axisStepsPerMM[Z_AXIS]);
  HAL::eprSetFloat(EPR_X_MAX_FEEDRATE, Printer::maxFeedrate[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE, Printer::maxFeedrate[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE, Printer::maxFeedrate[Z_AXIS]);
  HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE, Printer::homingFeedrate[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE, Printer::homingFeedrate[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE, Printer::homingFeedrate[Z_AXIS]);
  HAL::eprSetFloat(EPR_MAX_JERK, Printer::maxJerk);
#if DRIVE_SYSTEM != DELTA
  HAL::eprSetFloat(EPR_MAX_ZJERK, Printer::maxZJerk);
#endif
#if RAMP_ACCELERATION
  HAL::eprSetFloat(EPR_X_MAX_ACCEL,
                   Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_MAX_ACCEL,
                   Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_MAX_ACCEL,
                   Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
  HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL,
                   Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL,
                   Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL,
                   Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#endif
#if HAVE_HEATED_BED
  HAL::eprSetByte(EPR_BED_HEAT_MANAGER, heatedBedController.heatManager);
  HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP,
                   heatedBedController.preheatTemperature);
#else
  HAL::eprSetByte(EPR_BED_HEAT_MANAGER, HEATED_BED_HEAT_MANAGER);
#endif
#if HAVE_HEATED_BED
  HAL::eprSetByte(EPR_BED_DRIVE_MAX, heatedBedController.pidDriveMax);
  HAL::eprSetByte(EPR_BED_DRIVE_MIN, heatedBedController.pidDriveMin);
  HAL::eprSetFloat(EPR_BED_PID_PGAIN, heatedBedController.pidPGain);
  HAL::eprSetFloat(EPR_BED_PID_IGAIN, heatedBedController.pidIGain);
  HAL::eprSetFloat(EPR_BED_PID_DGAIN, heatedBedController.pidDGain);
  HAL::eprSetByte(EPR_BED_PID_MAX, heatedBedController.pidMax);
#if ENABLED(TEMP_GAIN)
  HAL::eprSetFloat(EPR_EXTRUDER_GAIN, heatedBedController.tempGain);
  HAL::eprSetFloat(EPR_EXTRUDER_BIAS, heatedBedController.tempBias);
#endif
#else
  HAL::eprSetByte(EPR_BED_DRIVE_MAX, HEATED_BED_PID_INTEGRAL_DRIVE_MAX);
  HAL::eprSetByte(EPR_BED_DRIVE_MIN, HEATED_BED_PID_INTEGRAL_DRIVE_MIN);
  HAL::eprSetFloat(EPR_BED_PID_PGAIN, HEATED_BED_PID_PGAIN_OR_DEAD_TIME);
  HAL::eprSetFloat(EPR_BED_PID_IGAIN, HEATED_BED_PID_IGAIN);
  HAL::eprSetFloat(EPR_BED_PID_DGAIN, HEATED_BED_PID_DGAIN);
  HAL::eprSetByte(EPR_BED_PID_MAX, HEATED_BED_PID_MAX);
#if ENABLED(TEMP_GAIN)
  HAL::eprSetFloat(EPR_HEATED_BED_GAIN, 1.0);
  HAL::eprSetFloat(EPR_HEATED_BED_BIAS, 0.0);
#endif
#endif
  // SHOT("storeDataIntoEEPROM");
  // SHOWM(Printer::xMin);SHOWM(Printer::yMin);SHOWM(Printer::zMin);
  HAL::eprSetFloat(EPR_X_HOME_OFFSET, Printer::xMin);
  HAL::eprSetFloat(EPR_Y_HOME_OFFSET, Printer::yMin);
  HAL::eprSetFloat(EPR_Z_HOME_OFFSET, Printer::zMin);
  HAL::eprSetFloat(EPR_X_LENGTH, Printer::xLength);
  HAL::eprSetFloat(EPR_Y_LENGTH, Printer::yLength);
  HAL::eprSetFloat(EPR_Z_LENGTH, Printer::zLength);
#if NONLINEAR_SYSTEM
  HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, Printer::radius0);
#endif
#if ENABLE_BACKLASH_COMPENSATION
  HAL::eprSetFloat(EPR_BACKLASH_X, Printer::backlashX);
  HAL::eprSetFloat(EPR_BACKLASH_Y, Printer::backlashY);
  HAL::eprSetFloat(EPR_BACKLASH_Z, Printer::backlashZ);
#else
  HAL::eprSetFloat(EPR_BACKLASH_X, 0);
  HAL::eprSetFloat(EPR_BACKLASH_Y, 0);
  HAL::eprSetFloat(EPR_BACKLASH_Z, 0);
#endif
#if FEATURE_AUTOLEVEL
  HAL::eprSetByte(EPR_AUTOLEVEL_ACTIVE, Printer::isAutolevelActive());
  for (uint8_t i = 0; i < 9; i++)
    HAL::eprSetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2),
                     Printer::autolevelTransformation[i]);
#endif
#if UI_DISPLAY_TYPE != NO_DISPLAY
  HAL::eprSetByte(EPR_SELECTED_LANGUAGE, Com::selectedLanguage);
#endif
  // now the extruder
  for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
#if FEATURE_WATCHDOG
    HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

    int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
    Extruder *e = &extruder[i];
    HAL::eprSetFloat(o + EPR_EXTRUDER_STEPS_PER_MM, e->stepsPerMM);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, e->maxFeedrate);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, e->maxStartFeedrate);
    HAL::eprSetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, e->maxAcceleration);
    HAL::eprSetByte(o + EPR_EXTRUDER_HEAT_MANAGER, e->tempControl.heatManager);
    HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT,
                     e->tempControl.preheatTemperature);
    HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MAX, e->tempControl.pidDriveMax);
    HAL::eprSetByte(o + EPR_EXTRUDER_DRIVE_MIN, e->tempControl.pidDriveMin);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_PGAIN, e->tempControl.pidPGain);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_IGAIN, e->tempControl.pidIGain);
    HAL::eprSetFloat(o + EPR_EXTRUDER_PID_DGAIN, e->tempControl.pidDGain);
    HAL::eprSetByte(o + EPR_EXTRUDER_PID_MAX, e->tempControl.pidMax);
    HAL::eprSetInt32(o + EPR_EXTRUDER_X_OFFSET, e->xOffset);
    HAL::eprSetInt32(o + EPR_EXTRUDER_Y_OFFSET, e->yOffset);
    HAL::eprSetInt32(o + EPR_EXTRUDER_Z_OFFSET, e->zOffset);
    HAL::eprSetInt16(o + EPR_EXTRUDER_WATCH_PERIOD, e->watchPeriod);
#if RETRACT_DURING_HEATUP
    HAL::eprSetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_TEMP,
                     e->waitRetractTemperature);
    HAL::eprSetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_UNITS, e->waitRetractUnits);
#else
    HAL::eprSetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_TEMP,
                     EXT0_WAIT_RETRACT_TEMP);
    HAL::eprSetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_UNITS,
                     EXT0_WAIT_RETRACT_UNITS);
#endif
    HAL::eprSetByte(o + EPR_EXTRUDER_COOLER_SPEED, e->coolerSpeed);
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_K, e->advanceK);
#else
    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_K, 0);
#endif
    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_L, e->advanceL);
#else
    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_K, 0);
    HAL::eprSetFloat(o + EPR_EXTRUDER_ADVANCE_L, 0);
#endif
#if ENABLED(TEMP_GAIN)
    HAL::eprSetFloat(o + EPR_EXTRUDER_GAIN, e->tempControl.tempGain);
    HAL::eprSetFloat(o + EPR_EXTRUDER_BIAS, e->tempControl.tempBias);
#endif
  }
#if MIXING_EXTRUDER
  storeMixingRatios(false);
#endif
  if (corrupted) {
    HAL::eprSetInt32(EPR_PRINTING_TIME, 0);
    HAL::eprSetFloat(EPR_PRINTING_DISTANCE, 0);
    initalizeUncached();
  }
  // Save version and build checksum
  HAL::eprSetByte(EPR_VERSION, EEPROM_PROTOCOL_VERSION);
  HAL::eprSetByte(EPR_INTEGRITY_BYTE, computeChecksum());
#endif
}
void EEPROM::initalizeUncached() {
  HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, Z_PROBE_HEIGHT);
  HAL::eprSetFloat(EPR_Z_PROBE_SPEED, Z_PROBE_SPEED);
  HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED, Z_PROBE_XY_SPEED);
  HAL::eprSetFloat(EPR_Z_PROBE_X_OFFSET, Z_PROBE_X_OFFSET);
  HAL::eprSetFloat(EPR_Z_PROBE_Y_OFFSET, Z_PROBE_Y_OFFSET);
  HAL::eprSetFloat(EPR_Z_PROBE_Z_OFFSET, Z_PROBE_Z_OFFSET);
  HAL::eprSetFloat(EPR_Z_PROBE_X1, Z_PROBE_X1);
  HAL::eprSetFloat(EPR_Z_PROBE_Y1, Z_PROBE_Y1);
  HAL::eprSetFloat(EPR_Z_PROBE_X2, Z_PROBE_X2);
  HAL::eprSetFloat(EPR_Z_PROBE_Y2, Z_PROBE_Y2);
  HAL::eprSetFloat(EPR_Z_PROBE_X3, Z_PROBE_X3);
  HAL::eprSetFloat(EPR_Z_PROBE_Y3, Z_PROBE_Y3);
  HAL::eprSetFloat(EPR_AXISCOMP_TANXY, AXISCOMP_TANXY);
  HAL::eprSetFloat(EPR_AXISCOMP_TANYZ, AXISCOMP_TANYZ);
  HAL::eprSetFloat(EPR_AXISCOMP_TANXZ, AXISCOMP_TANXZ);
  HAL::eprSetFloat(EPR_Z_PROBE_BED_DISTANCE, Z_PROBE_BED_DISTANCE);
  Printer::zBedOffset = HAL::eprGetFloat(EPR_Z_PROBE_Z_OFFSET);
#if NONLINEAR_SYSTEM
  HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,
                   DELTA_SEGMENTS_PER_SECOND_PRINT);
  HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,
                   DELTA_SEGMENTS_PER_SECOND_MOVE);
#endif
#if DRIVE_SYSTEM == DELTA
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH, DELTA_DIAGONAL_ROD);
  HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, ROD_RADIUS);
  HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS, DELTA_X_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS, DELTA_Y_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS, DELTA_Z_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_A, DELTA_ALPHA_A);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_B, DELTA_ALPHA_B);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_C, DELTA_ALPHA_C);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A, DELTA_RADIUS_CORRECTION_A);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B, DELTA_RADIUS_CORRECTION_B);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C, DELTA_RADIUS_CORRECTION_C);
  HAL::eprSetFloat(EPR_DELTA_MAX_RADIUS, DELTA_MAX_RADIUS);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,
                   DELTA_DIAGONAL_CORRECTION_A);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,
                   DELTA_DIAGONAL_CORRECTION_B);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,
                   DELTA_DIAGONAL_CORRECTION_C);
#endif
  HAL::eprSetFloat(EPR_AXISCOMP_TANXY, AXISCOMP_TANXY);
  HAL::eprSetFloat(EPR_AXISCOMP_TANYZ, AXISCOMP_TANYZ);
  HAL::eprSetFloat(EPR_AXISCOMP_TANXZ, AXISCOMP_TANXZ);
  HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);

  HAL::eprSetFloat(EPR_RETRACTION_LENGTH, RETRACTION_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_LONG_LENGTH, RETRACTION_LONG_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_SPEED, RETRACTION_SPEED);
  HAL::eprSetFloat(EPR_RETRACTION_Z_LIFT, RETRACTION_Z_LIFT);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,
                   RETRACTION_UNDO_EXTRA_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,
                   RETRACTION_UNDO_EXTRA_LONG_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_SPEED, RETRACTION_UNDO_SPEED);
  HAL::eprSetByte(EPR_AUTORETRACT_ENABLED, AUTORETRACT_ENABLED);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_A, BENDING_CORRECTION_A);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_B, BENDING_CORRECTION_B);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_C, BENDING_CORRECTION_C);
  HAL::eprSetFloat(EPR_ACCELERATION_FACTOR_TOP, ACCELERATION_FACTOR_TOP);
  HAL::eprSetFloat(EPR_PARK_X, PARK_POSITION_X);
  HAL::eprSetFloat(EPR_PARK_Y, PARK_POSITION_Y);
  HAL::eprSetFloat(EPR_PARK_Z, PARK_POSITION_Z_RAISE);
}

void EEPROM::readDataFromEEPROM(bool includeExtruder) {
#if EEPROM_MODE != 0
  uint8_t version =
      HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy
                                    // data nor set it to older versions!
  Com::printFLN(PSTR("Detected EEPROM version:"), (int)version);
  baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
  maxInactiveTime = HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME);
  stepperInactiveTime = HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
//#define EPR_ACCELERATION_TYPE 1
#if DUAL_X_RESOLUTION
  Printer::axisX1StepsPerMM = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
  Printer::axisX2StepsPerMM = HAL::eprGetFloat(EPR_X2AXIS_STEPS_PER_MM);
#else
  Printer::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
#endif
  Printer::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
  Printer::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);
  Printer::maxFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_FEEDRATE);
  Printer::maxFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_FEEDRATE);
  Printer::maxFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_FEEDRATE);
  Printer::homingFeedrate[X_AXIS] = HAL::eprGetFloat(EPR_X_HOMING_FEEDRATE);
  Printer::homingFeedrate[Y_AXIS] = HAL::eprGetFloat(EPR_Y_HOMING_FEEDRATE);
  Printer::homingFeedrate[Z_AXIS] = HAL::eprGetFloat(EPR_Z_HOMING_FEEDRATE);
  Printer::maxJerk = HAL::eprGetFloat(EPR_MAX_JERK);
#if DRIVE_SYSTEM != DELTA
  Printer::maxZJerk = HAL::eprGetFloat(EPR_MAX_ZJERK);
#endif
#if RAMP_ACCELERATION
  Printer::maxAccelerationMMPerSquareSecond[X_AXIS] =
      HAL::eprGetFloat(EPR_X_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] =
      HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] =
      HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] =
      HAL::eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] =
      HAL::eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] =
      HAL::eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL);
#endif
#if HAVE_HEATED_BED
  heatedBedController.heatManager = HAL::eprGetByte(EPR_BED_HEAT_MANAGER);
  heatedBedController.preheatTemperature =
      HAL::eprGetInt16(EPR_BED_PREHEAT_TEMP);
  heatedBedController.pidDriveMax = HAL::eprGetByte(EPR_BED_DRIVE_MAX);
  heatedBedController.pidDriveMin = HAL::eprGetByte(EPR_BED_DRIVE_MIN);
  heatedBedController.pidPGain = HAL::eprGetFloat(EPR_BED_PID_PGAIN);
  heatedBedController.pidIGain = HAL::eprGetFloat(EPR_BED_PID_IGAIN);
  heatedBedController.pidDGain = HAL::eprGetFloat(EPR_BED_PID_DGAIN);
  heatedBedController.pidMax = HAL::eprGetByte(EPR_BED_PID_MAX);
#if ENABLED(TEMP_GAIN)
  if (version < 20) {
    HAL::eprSetFloat(EPR_HEATED_BED_GAIN, 1.0);
    HAL::eprSetFloat(EPR_HEATED_BED_BIAS, 0.0);
  }
  heatedBedController.tempGain = HAL::eprGetFloat(EPR_HEATED_BED_GAIN);
  heatedBedController.tempBias = HAL::eprGetFloat(EPR_HEATED_BED_BIAS);
#endif
#endif
  Printer::xMin = HAL::eprGetFloat(EPR_X_HOME_OFFSET);
  Printer::yMin = HAL::eprGetFloat(EPR_Y_HOME_OFFSET);
  Printer::zMin = HAL::eprGetFloat(EPR_Z_HOME_OFFSET);
  Printer::xLength = HAL::eprGetFloat(EPR_X_LENGTH);
  Printer::yLength = HAL::eprGetFloat(EPR_Y_LENGTH);
  Printer::zLength = HAL::eprGetFloat(EPR_Z_LENGTH);
#if DUAL_X_AXIS_MODE > 0 && LAZY_DUAL_X_AXIS == 0
  Printer::x1Min = Printer::xMin;
  Printer::x1Length = Printer::xLength;
#endif
#if NONLINEAR_SYSTEM
  Printer::radius0 = HAL::eprGetFloat(EPR_DELTA_HORIZONTAL_RADIUS);
#endif
#if ENABLE_BACKLASH_COMPENSATION
  Printer::backlashX = HAL::eprGetFloat(EPR_BACKLASH_X);
  Printer::backlashY = HAL::eprGetFloat(EPR_BACKLASH_Y);
  Printer::backlashZ = HAL::eprGetFloat(EPR_BACKLASH_Z);
#endif
#if FEATURE_AUTOLEVEL
  if (version > 2) {
    float sum = 0;
    for (uint8_t i = 0; i < 9; i++)
      Printer::autolevelTransformation[i] =
          HAL::eprGetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2));
    if (isnan(Printer::autolevelTransformation
                  [0])) // a bug caused storage of matrix at the wrong place.
                        // Read from old position instead.
    {
      for (uint8_t i = 0; i < 9; i++)
        Printer::autolevelTransformation[i] =
            HAL::eprGetFloat((EPR_AUTOLEVEL_MATRIX + (int)i) << 2);
    }
    for (uint8_t i = 0; i < 9; i++) {
      if (isnan(Printer::autolevelTransformation[i]))
        sum += 10;
      else
        sum += RMath::sqr(Printer::autolevelTransformation[i]);
    }
    if (sum < 2.7 || sum > 3.3)
      Printer::resetTransformationMatrix(false);
    Printer::setAutolevelActive(HAL::eprGetByte(EPR_AUTOLEVEL_ACTIVE));
    Com::printArrayFLN(Com::tTransformationMatrix,
                       Printer::autolevelTransformation, 9, 6);
  }
#endif
  if (includeExtruder) {
#if MIXING_EXTRUDER
    readMixingRatios();
#endif
    // now the extruder
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
#if FEATURE_WATCHDOG
      HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

      int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
      Extruder *e = &extruder[i];
      e->stepsPerMM = HAL::eprGetFloat(o + EPR_EXTRUDER_STEPS_PER_MM);
      e->maxFeedrate = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_FEEDRATE);
      e->maxStartFeedrate =
          HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE);
      e->maxAcceleration = HAL::eprGetFloat(o + EPR_EXTRUDER_MAX_ACCELERATION);
      e->tempControl.heatManager =
          HAL::eprGetByte(o + EPR_EXTRUDER_HEAT_MANAGER);
      e->tempControl.preheatTemperature =
          HAL::eprGetInt16(o + EPR_EXTRUDER_PREHEAT);
      e->tempControl.pidDriveMax = HAL::eprGetByte(o + EPR_EXTRUDER_DRIVE_MAX);
      e->tempControl.pidDriveMin = HAL::eprGetByte(o + EPR_EXTRUDER_DRIVE_MIN);
      e->tempControl.pidPGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_PGAIN);
      e->tempControl.pidIGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_IGAIN);
      e->tempControl.pidDGain = HAL::eprGetFloat(o + EPR_EXTRUDER_PID_DGAIN);
      e->tempControl.pidMax = HAL::eprGetByte(o + EPR_EXTRUDER_PID_MAX);
      e->xOffset = HAL::eprGetInt32(o + EPR_EXTRUDER_X_OFFSET);
      e->yOffset = HAL::eprGetInt32(o + EPR_EXTRUDER_Y_OFFSET);
      e->watchPeriod = HAL::eprGetInt16(o + EPR_EXTRUDER_WATCH_PERIOD);
#if RETRACT_DURING_HEATUP
      e->waitRetractTemperature =
          HAL::eprGetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_TEMP);
      e->waitRetractUnits =
          HAL::eprGetInt16(o + EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
      e->advanceK = HAL::eprGetFloat(o + EPR_EXTRUDER_ADVANCE_K);
#endif
      e->advanceL = HAL::eprGetFloat(o + EPR_EXTRUDER_ADVANCE_L);
#endif
#if ENABLED(TEMP_GAIN)
      if (version < 20) {
        HAL::eprSetFloat(o + EPR_EXTRUDER_GAIN, 1.0);
        HAL::eprSetFloat(o + EPR_EXTRUDER_BIAS, 0.0);
      }
      e->tempControl.tempGain = HAL::eprGetFloat(o + EPR_EXTRUDER_GAIN);
      e->tempControl.tempBias = HAL::eprGetFloat(o + EPR_EXTRUDER_BIAS);
#endif
      if (version > 1)
        e->coolerSpeed = HAL::eprGetByte(o + EPR_EXTRUDER_COOLER_SPEED);
      if (version < 13) {
        HAL::eprSetInt32(o + EPR_EXTRUDER_Z_OFFSET, e->zOffset);
      }
      e->zOffset = HAL::eprGetInt32(o + EPR_EXTRUDER_Z_OFFSET);
    }
  }
  if (version != EEPROM_PROTOCOL_VERSION) {
    Com::printInfoFLN(Com::tEPRProtocolChanged);
    if (version < 3) {
      HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, Z_PROBE_HEIGHT);
      HAL::eprSetFloat(EPR_Z_PROBE_SPEED, Z_PROBE_SPEED);
      HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED, Z_PROBE_XY_SPEED);
      HAL::eprSetFloat(EPR_Z_PROBE_X_OFFSET, Z_PROBE_X_OFFSET);
      HAL::eprSetFloat(EPR_Z_PROBE_Y_OFFSET, Z_PROBE_Y_OFFSET);
      HAL::eprSetFloat(EPR_Z_PROBE_X1, Z_PROBE_X1);
      HAL::eprSetFloat(EPR_Z_PROBE_Y1, Z_PROBE_Y1);
      HAL::eprSetFloat(EPR_Z_PROBE_X2, Z_PROBE_X2);
      HAL::eprSetFloat(EPR_Z_PROBE_Y2, Z_PROBE_Y2);
      HAL::eprSetFloat(EPR_Z_PROBE_X3, Z_PROBE_X3);
      HAL::eprSetFloat(EPR_Z_PROBE_Y3, Z_PROBE_Y3);
    }
    if (version < 4) {
#if NONLINEAR_SYSTEM
      HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,
                       DELTA_SEGMENTS_PER_SECOND_PRINT);
      HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,
                       DELTA_SEGMENTS_PER_SECOND_MOVE);
#endif
#if DRIVE_SYSTEM == DELTA
      HAL::eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH, DELTA_DIAGONAL_ROD);
      HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, ROD_RADIUS);
      HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,
                       DELTA_X_ENDSTOP_OFFSET_STEPS);
      HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,
                       DELTA_Y_ENDSTOP_OFFSET_STEPS);
      HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,
                       DELTA_Z_ENDSTOP_OFFSET_STEPS);
#endif
    }
#if DRIVE_SYSTEM == DELTA
    if (version < 5) {
      HAL::eprSetFloat(EPR_DELTA_ALPHA_A, DELTA_ALPHA_A);
      HAL::eprSetFloat(EPR_DELTA_ALPHA_B, DELTA_ALPHA_B);
      HAL::eprSetFloat(EPR_DELTA_ALPHA_C, DELTA_ALPHA_C);
    }
    if (version < 6) {
      HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A, DELTA_RADIUS_CORRECTION_A);
      HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B, DELTA_RADIUS_CORRECTION_B);
      HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C, DELTA_RADIUS_CORRECTION_C);
    }
    if (version < 7) {
      HAL::eprSetFloat(EPR_DELTA_MAX_RADIUS, DELTA_MAX_RADIUS);
      HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,
                       DELTA_DIAGONAL_CORRECTION_A);
      HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,
                       DELTA_DIAGONAL_CORRECTION_B);
      HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,
                       DELTA_DIAGONAL_CORRECTION_C);
    }
#endif
    if (version < 8) {
      HAL::eprSetFloat(EPR_Z_PROBE_BED_DISTANCE, Z_PROBE_BED_DISTANCE);
    }
    if (version < 9) {
#if MIXING_EXTRUDER
      storeMixingRatios(false);
#endif
    }
    if (version < 10) {
      HAL::eprSetFloat(EPR_AXISCOMP_TANXY, AXISCOMP_TANXY);
      HAL::eprSetFloat(EPR_AXISCOMP_TANYZ, AXISCOMP_TANYZ);
      HAL::eprSetFloat(EPR_AXISCOMP_TANXZ, AXISCOMP_TANXZ);
    }
    if (version < 11) {
      HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);
    }
    if (version < 12) {
      HAL::eprSetFloat(EPR_RETRACTION_LENGTH, RETRACTION_LENGTH);
      HAL::eprSetFloat(EPR_RETRACTION_LONG_LENGTH, RETRACTION_LONG_LENGTH);
      HAL::eprSetFloat(EPR_RETRACTION_SPEED, RETRACTION_SPEED);
      HAL::eprSetFloat(EPR_RETRACTION_Z_LIFT, RETRACTION_Z_LIFT);
      HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,
                       RETRACTION_UNDO_EXTRA_LENGTH);
      HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,
                       RETRACTION_UNDO_EXTRA_LONG_LENGTH);
      HAL::eprSetFloat(EPR_RETRACTION_UNDO_SPEED, RETRACTION_UNDO_SPEED);
      HAL::eprSetByte(EPR_AUTORETRACT_ENABLED, AUTORETRACT_ENABLED);
    }
    if (version < 14) {
      HAL::eprSetFloat(EPR_Z_PROBE_Z_OFFSET, Z_PROBE_Z_OFFSET);
    }
    if (version < 15) {
      HAL::eprSetByte(EPR_SELECTED_LANGUAGE,
                      254); // activate selector on startup
#if UI_DISPLAY_TYPE != NO_DISPLAY
      Com::selectedLanguage = 254;
#endif
    }
    if (version < 16) {
      HAL::eprSetFloat(EPR_BENDING_CORRECTION_A, BENDING_CORRECTION_A);
      HAL::eprSetFloat(EPR_BENDING_CORRECTION_B, BENDING_CORRECTION_B);
      HAL::eprSetFloat(EPR_BENDING_CORRECTION_C, BENDING_CORRECTION_C);
      HAL::eprSetFloat(EPR_ACCELERATION_FACTOR_TOP, ACCELERATION_FACTOR_TOP);
    }
    if (version < 17) {
#if HAVE_HEATED_BED
      heatedBedController.preheatTemperature = HEATED_BED_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 0
      extruder[0].tempControl.preheatTemperature = EXT0_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 1
      extruder[1].tempControl.preheatTemperature = EXT1_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 2
      extruder[2].tempControl.preheatTemperature = EXT2_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 3
      extruder[3].tempControl.preheatTemperature = EXT3_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 4
      extruder[4].tempControl.preheatTemperature = EXT4_PREHEAT_TEMP;
#endif
#if NUM_EXTRUDER > 5
      extruder[5].tempControl.preheatTemperature = EXT5_PREHEAT_TEMP;
#endif
    }
#if DUAL_X_RESOLUTION
    if (version < 18) {
      Printer::axisX2StepsPerMM = X2AXIS_STEPS_PER_MM;
    }
#endif
    if (version < 19) {
      HAL::eprSetFloat(EPR_PARK_X, PARK_POSITION_X);
      HAL::eprSetFloat(EPR_PARK_Y, PARK_POSITION_Y);
      HAL::eprSetFloat(EPR_PARK_Z, PARK_POSITION_Z_RAISE);
    }
    /*        if (version<8) {
    #if DRIVE_SYSTEM==DELTA
              // Prior to version 8, the Cartesian max was stored in the zmax
              // Now, x,y and z max are used for tower a, b and c
              // Of tower min are all set at 0, tower max is larger than
    cartesian max
              // by the height of any tower for coordinate 0,0,0
              long cart[Z_AXIS_ARRAY], delta[TOWER_ARRAY];
              cart[X_AXIS] = cart[Y_AXIS] = cart[Z_AXIS] = 0;
              transformCartesianStepsToDeltaSteps(cart, delta);
              // We can only count on ZLENGTH being set correctly, as it was
    used for all towers Printer::xLength = Printer::zLength + delta[X_AXIS];
              Printer::yLength = Printer::zLength + delta[Y_AXIS];
              Printer::zLength += delta[Z_AXIS];
    #endif
            }*/
    HAL::eprSetByte(EPR_RESCUE_MODE, 0);
    storeDataIntoEEPROM(false); // Store new fields for changed version
  }
  Printer::zBedOffset = HAL::eprGetFloat(EPR_Z_PROBE_Z_OFFSET);
#if UI_DISPLAY_TYPE != NO_DISPLAY
  Com::selectLanguage(HAL::eprGetByte(EPR_SELECTED_LANGUAGE));
#endif
  Printer::updateDerivedParameter();
  Extruder::initHeatedBed();
#endif
}

void EEPROM::initBaudrate() {
  // Invariant - baudrate is initialized with or without eeprom!
  baudrate = BAUDRATE;
#if EEPROM_MODE != 0
  if (HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE) {
    baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
  }
#endif
}

#ifndef USE_CONFIGURATION_BAUD_RATE
#define USE_CONFIGURATION_BAUD_RATE 0
#endif // USE_CONFIGURATION_BAUD_RATE
void EEPROM::init() {
#if EEPROM_MODE != 0
  uint8_t check = computeChecksum();
  uint8_t storedcheck = HAL::eprGetByte(EPR_INTEGRITY_BYTE);
  if (HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check) {
    readDataFromEEPROM(true);
    if (USE_CONFIGURATION_BAUD_RATE) {
      // Used if eeprom gets unusable baud rate set and communication wont work
      // at all.
      if (HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE) {
        HAL::eprSetInt32(EPR_BAUDRATE, BAUDRATE);
        baudrate = BAUDRATE;
        uint8_t newcheck = computeChecksum();
        if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
          HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
      }
      Com::printFLN(PSTR("EEPROM baud rate restored from configuration."));
      Com::printFLN(PSTR("RECOMPILE WITH USE_CONFIGURATION_BAUD_RATE == 0 to "
                         "alter baud rate via EEPROM"));
    }
  } else {
    HAL::eprSetByte(EPR_MAGIC_BYTE, EEPROM_MODE); // Make data change permanent
    initalizeUncached();
    storeDataIntoEEPROM(storedcheck != check);
    HAL::eprSetByte(EPR_RESCUE_MODE, 0);
  }
#endif
}

void EEPROM::updatePrinterUsage() {
#if EEPROM_MODE != 0
  if (Printer::filamentPrinted == 0 ||
      (Printer::flag2 & PRINTER_FLAG2_RESET_FILAMENT_USAGE) != 0)
    return; // No miles only enabled
  uint32_t seconds =
      (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000;
  seconds += HAL::eprGetInt32(EPR_PRINTING_TIME);
  HAL::eprSetInt32(EPR_PRINTING_TIME, seconds);
  HAL::eprSetFloat(EPR_PRINTING_DISTANCE,
                   HAL::eprGetFloat(EPR_PRINTING_DISTANCE) +
                       Printer::filamentPrinted * 0.001);
  Printer::flag2 |= PRINTER_FLAG2_RESET_FILAMENT_USAGE;
  Printer::msecondsPrinting = HAL::timeInMilliseconds();
  updateChecksum();
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
void EEPROM::writeSettings() {
#if EEPROM_MODE != 0
  writeByte(EPR_SELECTED_LANGUAGE, Com::tLanguage);
  writeLong(EPR_BAUDRATE, Com::tEPRBaudrate);
#if NUM_EXTRUDER > 0
  writeFloat(EPR_PRINTING_DISTANCE, Com::tEPRFilamentPrinted);
#endif
  writeLong(EPR_PRINTING_TIME, Com::tEPRPrinterActive);
  writeLong(EPR_MAX_INACTIVE_TIME, Com::tEPRMaxInactiveTime);
  writeLong(EPR_STEPPER_INACTIVE_TIME, Com::tEPRStopAfterInactivty);
//#define EPR_ACCELERATION_TYPE 1
#if DRIVE_SYSTEM != DELTA
  writeFloat(EPR_XAXIS_STEPS_PER_MM, Com::tEPRXStepsPerMM, 4);
#if DUAL_X_RESOLUTION
  writeFloat(EPR_X2AXIS_STEPS_PER_MM, Com::tEPRX2StepsPerMM, 4);
#endif
  writeFloat(EPR_YAXIS_STEPS_PER_MM, Com::tEPRYStepsPerMM, 4);
#endif
  writeFloat(EPR_ZAXIS_STEPS_PER_MM, Com::tEPRZStepsPerMM, 4);
#if DRIVE_SYSTEM != DELTA
  writeFloat(EPR_X_MAX_FEEDRATE, Com::tEPRXMaxFeedrate);
  writeFloat(EPR_Y_MAX_FEEDRATE, Com::tEPRYMaxFeedrate);
#endif
  writeFloat(EPR_Z_MAX_FEEDRATE, Com::tEPRZMaxFeedrate);
#if DRIVE_SYSTEM != DELTA
  writeFloat(EPR_X_HOMING_FEEDRATE, Com::tEPRXHomingFeedrate);
  writeFloat(EPR_Y_HOMING_FEEDRATE, Com::tEPRYHomingFeedrate);
#endif
  writeFloat(EPR_Z_HOMING_FEEDRATE, Com::tEPRZHomingFeedrate);
  writeFloat(EPR_MAX_JERK, Com::tEPRMaxJerk);
#if DRIVE_SYSTEM != DELTA
  writeFloat(EPR_MAX_ZJERK, Com::tEPRMaxZJerk);
#endif
  writeFloat(EPR_X_HOME_OFFSET, Com::tEPRXHomePos);
  writeFloat(EPR_Y_HOME_OFFSET, Com::tEPRYHomePos);
  writeFloat(EPR_Z_HOME_OFFSET, Com::tEPRZHomePos);
  writeFloat(EPR_X_LENGTH, Com::tEPRXMaxLength);
  writeFloat(EPR_Y_LENGTH, Com::tEPRYMaxLength);
  writeFloat(EPR_Z_LENGTH, Com::tEPRZMaxLength);
  writeFloat(EPR_PARK_X, PSTR("Park position X [mm]"));
  writeFloat(EPR_PARK_Y, PSTR("Park position Y [mm]"));
  writeFloat(EPR_PARK_Z, PSTR("Park position Z raise [mm]"));

#if ENABLE_BACKLASH_COMPENSATION
  writeFloat(EPR_BACKLASH_X, Com::tEPRXBacklash);
  writeFloat(EPR_BACKLASH_Y, Com::tEPRYBacklash);
  writeFloat(EPR_BACKLASH_Z, Com::tEPRZBacklash);
#endif
#if NONLINEAR_SYSTEM
  writeInt(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,
           Com::tEPRSegmentsPerSecondTravel);
  writeInt(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,
           Com::tEPRSegmentsPerSecondPrint);
#endif
#if RAMP_ACCELERATION
  // epr_out_float(EPR_X_MAX_START_SPEED,PSTR("X-axis start speed [mm/s]"));
  // epr_out_float(EPR_Y_MAX_START_SPEED,PSTR("Y-axis start speed [mm/s]"));
  // epr_out_float(EPR_Z_MAX_START_SPEED,PSTR("Z-axis start speed [mm/s]"));
#if DRIVE_SYSTEM == TUGA
  writeFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH, Com::tEPRDiagonalRodLength);
#endif
#if DRIVE_SYSTEM == DELTA
  writeFloat(EPR_Z_MAX_ACCEL, Com::tEPRZAcceleration);
  writeFloat(EPR_Z_MAX_TRAVEL_ACCEL, Com::tEPRZTravelAcceleration);
#if defined(INTERPOLATE_ACCELERATION_WITH_Z) &&                                \
    INTERPOLATE_ACCELERATION_WITH_Z != 0
  writeFloat(EPR_ACCELERATION_FACTOR_TOP, Com::tEPRAccelerationFactorAtTop);
#endif
  writeFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH, Com::tEPRDiagonalRodLength);
  writeFloat(EPR_DELTA_HORIZONTAL_RADIUS, Com::tEPRHorizontalRadius);
  writeFloat(EPR_DELTA_MAX_RADIUS, Com::tEPRDeltaMaxRadius);
  writeInt(EPR_DELTA_TOWERX_OFFSET_STEPS, Com::tEPRTowerXOffset);
  writeInt(EPR_DELTA_TOWERY_OFFSET_STEPS, Com::tEPRTowerYOffset);
  writeInt(EPR_DELTA_TOWERZ_OFFSET_STEPS, Com::tEPRTowerZOffset);
  writeFloat(EPR_DELTA_ALPHA_A, Com::tDeltaAlphaA);
  writeFloat(EPR_DELTA_ALPHA_B, Com::tDeltaAlphaB);
  writeFloat(EPR_DELTA_ALPHA_C, Com::tDeltaAlphaC);
  writeFloat(EPR_DELTA_RADIUS_CORR_A, Com::tDeltaRadiusCorrectionA);
  writeFloat(EPR_DELTA_RADIUS_CORR_B, Com::tDeltaRadiusCorrectionB);
  writeFloat(EPR_DELTA_RADIUS_CORR_C, Com::tDeltaRadiusCorrectionC);
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_A, Com::tDeltaDiagonalCorrectionA);
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_B, Com::tDeltaDiagonalCorrectionB);
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_C, Com::tDeltaDiagonalCorrectionC);
#else
  writeFloat(EPR_X_MAX_ACCEL, Com::tEPRXAcceleration);
  writeFloat(EPR_Y_MAX_ACCEL, Com::tEPRYAcceleration);
  writeFloat(EPR_Z_MAX_ACCEL, Com::tEPRZAcceleration);
  writeFloat(EPR_X_MAX_TRAVEL_ACCEL, Com::tEPRXTravelAcceleration);
  writeFloat(EPR_Y_MAX_TRAVEL_ACCEL, Com::tEPRYTravelAcceleration);
  writeFloat(EPR_Z_MAX_TRAVEL_ACCEL, Com::tEPRZTravelAcceleration);
#if defined(INTERPOLATE_ACCELERATION_WITH_Z) &&                                \
    INTERPOLATE_ACCELERATION_WITH_Z != 0
  writeFloat(EPR_ACCELERATION_FACTOR_TOP, Com::tEPRAccelerationFactorAtTop);
#endif
#endif
#endif
  writeFloat(EPR_Z_PROBE_Z_OFFSET, Com::tZProbeOffsetZ);
#if FEATURE_Z_PROBE
  writeFloat(EPR_Z_PROBE_HEIGHT, Com::tZProbeHeight);
  writeFloat(EPR_Z_PROBE_BED_DISTANCE, Com::tZProbeBedDitance);
  writeFloat(EPR_Z_PROBE_SPEED, Com::tZProbeSpeed);
  writeFloat(EPR_Z_PROBE_XY_SPEED, Com::tZProbeSpeedXY);
  writeFloat(EPR_Z_PROBE_X_OFFSET, Com::tZProbeOffsetX);
  writeFloat(EPR_Z_PROBE_Y_OFFSET, Com::tZProbeOffsetY);
  writeFloat(EPR_Z_PROBE_X1, Com::tZProbeX1);
  writeFloat(EPR_Z_PROBE_Y1, Com::tZProbeY1);
  writeFloat(EPR_Z_PROBE_X2, Com::tZProbeX2);
  writeFloat(EPR_Z_PROBE_Y2, Com::tZProbeY2);
  writeFloat(EPR_Z_PROBE_X3, Com::tZProbeX3);
  writeFloat(EPR_Z_PROBE_Y3, Com::tZProbeY3);
  writeFloat(EPR_BENDING_CORRECTION_A, Com::zZProbeBendingCorA);
  writeFloat(EPR_BENDING_CORRECTION_B, Com::zZProbeBendingCorB);
  writeFloat(EPR_BENDING_CORRECTION_C, Com::zZProbeBendingCorC);
#endif
#if FEATURE_AUTOLEVEL
  writeByte(EPR_AUTOLEVEL_ACTIVE, Com::tAutolevelActive);
#endif

#if FEATURE_AXISCOMP
  writeFloat(EPR_AXISCOMP_TANXY, Com::tAxisCompTanXY, 6);
  writeFloat(EPR_AXISCOMP_TANYZ, Com::tAxisCompTanYZ, 6);
  writeFloat(EPR_AXISCOMP_TANXZ, Com::tAxisCompTanXZ, 6);
#endif

#if HAVE_HEATED_BED
  writeInt(EPR_BED_PREHEAT_TEMP, Com::tEPRPreheatBedTemp);
  writeByte(EPR_BED_HEAT_MANAGER, Com::tEPRBedHeatManager);
  writeByte(EPR_BED_DRIVE_MAX, Com::tEPRBedPIDDriveMax);
  writeByte(EPR_BED_DRIVE_MIN, Com::tEPRBedPIDDriveMin);
  writeFloat(EPR_BED_PID_PGAIN, Com::tEPRBedPGain);
  writeFloat(EPR_BED_PID_IGAIN, Com::tEPRBedIGain);
  writeFloat(EPR_BED_PID_DGAIN, Com::tEPRBedDGain);
  writeByte(EPR_BED_PID_MAX, Com::tEPRBedPISMaxValue);
#if ENABLED(TEMP_GAIN)
  writeFloat(EPR_HEATED_BED_GAIN, PSTR("Bed Temp. Gain"), 4);
  writeFloat(EPR_HEATED_BED_BIAS, PSTR("Bed Temp. Bias [degC]"), 4);
#endif
#endif
#if FEATURE_RETRACTION
  writeByte(EPR_AUTORETRACT_ENABLED, Com::tEPRAutoretractEnabled);
  writeFloat(EPR_RETRACTION_LENGTH, Com::tEPRRetractionLength);
#if NUM_EXTRUDER > 1
  writeFloat(EPR_RETRACTION_LONG_LENGTH, Com::tEPRRetractionLongLength);
#endif
  writeFloat(EPR_RETRACTION_SPEED, Com::tEPRRetractionSpeed);
  writeFloat(EPR_RETRACTION_Z_LIFT, Com::tEPRRetractionZLift);
  writeFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,
             Com::tEPRRetractionUndoExtraLength);
#if NUM_EXTRUDER > 1
  writeFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,
             Com::tEPRRetractionUndoExtraLongLength);
#endif
  writeFloat(EPR_RETRACTION_UNDO_SPEED, Com::tEPRRetractionUndoSpeed);
#endif
  // now the extruder
  for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
    int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
    writeFloat(o + EPR_EXTRUDER_STEPS_PER_MM, Com::tEPRStepsPerMM);
    writeFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, Com::tEPRMaxFeedrate);
    writeFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, Com::tEPRStartFeedrate);
    writeFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, Com::tEPRAcceleration);
    writeInt(o + EPR_EXTRUDER_PREHEAT, Com::tEPRPreheatTemp);
    writeByte(o + EPR_EXTRUDER_HEAT_MANAGER, Com::tEPRHeatManager);
    writeByte(o + EPR_EXTRUDER_DRIVE_MAX, Com::tEPRDriveMax);
    writeByte(o + EPR_EXTRUDER_DRIVE_MIN, Com::tEPRDriveMin);
    writeFloat(o + EPR_EXTRUDER_PID_PGAIN, Com::tEPRPGain, 4);
    writeFloat(o + EPR_EXTRUDER_PID_IGAIN, Com::tEPRIGain, 4);
    writeFloat(o + EPR_EXTRUDER_PID_DGAIN, Com::tEPRDGain, 4);
#if ENABLED(TEMP_GAIN)
    writeFloat(o + EPR_EXTRUDER_GAIN, PSTR("Temp. Gain"), 4);
    writeFloat(o + EPR_EXTRUDER_BIAS, PSTR("Temp. Bias [degC]"), 4);
#endif
    writeByte(o + EPR_EXTRUDER_PID_MAX, Com::tEPRPIDMaxValue);
    writeLong(o + EPR_EXTRUDER_X_OFFSET, Com::tEPRXOffset);
    writeLong(o + EPR_EXTRUDER_Y_OFFSET, Com::tEPRYOffset);
    writeLong(o + EPR_EXTRUDER_Z_OFFSET, Com::tEPRZOffset);
    writeInt(o + EPR_EXTRUDER_WATCH_PERIOD, Com::tEPRStabilizeTime);
#if RETRACT_DURING_HEATUP
    writeInt(o + EPR_EXTRUDER_WAIT_RETRACT_TEMP,
             Com::tEPRRetractionWhenHeating);
    writeInt(o + EPR_EXTRUDER_WAIT_RETRACT_UNITS,
             Com::tEPRDistanceRetractHeating);
#endif
    writeByte(o + EPR_EXTRUDER_COOLER_SPEED, Com::tEPRExtruderCoolerSpeed);
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    writeFloat(o + EPR_EXTRUDER_ADVANCE_K, Com::tEPRAdvanceK);
#endif
    writeFloat(o + EPR_EXTRUDER_ADVANCE_L, Com::tEPRAdvanceL);
#endif
#if MIXING_EXTRUDER
    for (uint8_t v = 0; v < VIRTUAL_EXTRUDER; v++) {
      uint16_t pos = o + EPR_EXTRUDER_MIXING_RATIOS + 2 * v;
      Com::printF(Com::tEPR1, (int)pos);
      Com::print(' ');
      Com::print(HAL::eprGetInt16(pos));
      Com::print(' ');
      writeExtruderPrefix(pos);
      Com::printFLN(PSTR("Weight "), (int)(v + 1));
    }
#endif
  }
#else
  Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

#if EEPROM_MODE != 0

uint8_t EEPROM::computeChecksum() {
  unsigned int i;
  uint8_t checksum = 0;
  for (i = 0; i < 2048; i++) {
    if (i == EEPROM_OFFSET + EPR_INTEGRITY_BYTE)
      continue;
    checksum += HAL::eprGetByte(i);
  }
  return checksum;
}

void EEPROM::updateChecksum() {
  uint8_t newcheck = computeChecksum();
  if (newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
}

void EEPROM::writeExtruderPrefix(uint pos) {
  if (pos < EEPROM_EXTRUDER_OFFSET || pos >= 800)
    return;
  int n = (pos - EEPROM_EXTRUDER_OFFSET) / EEPROM_EXTRUDER_LENGTH + 1;
  Com::printF(Com::tExtrDot, n);
  Com::print(' ');
}

void EEPROM::writeFloat(uint pos, PGM_P text, uint8_t digits) {
  Com::printF(Com::tEPR3, static_cast<int>(pos));
  Com::print(' ');
  Com::printFloat(HAL::eprGetFloat(pos), digits);
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
  HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeLong(uint pos, PGM_P text) {
  Com::printF(Com::tEPR2, static_cast<int>(pos));
  Com::print(' ');
  Com::print(HAL::eprGetInt32(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
  HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeInt(uint pos, PGM_P text) {
  Com::printF(Com::tEPR1, static_cast<int>(pos));
  Com::print(' ');
  Com::print(HAL::eprGetInt16(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
  HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeByte(uint pos, PGM_P text) {
  Com::printF(Com::tEPR0, static_cast<int>(pos));
  Com::print(' ');
  Com::print((int)HAL::eprGetByte(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
  HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

#if MIXING_EXTRUDER
void EEPROM::storeMixingRatios(bool _updateChecksum) {
  for (uint8_t e = 0; e < NUM_EXTRUDER; e++) {
    for (uint8_t i = 0; i < VIRTUAL_EXTRUDER; i++) {
      HAL::eprSetInt16(EEPROM_EXTRUDER_OFFSET + e * EEPROM_EXTRUDER_LENGTH +
                           EPR_EXTRUDER_MIXING_RATIOS + 2 * i,
                       extruder[e].virtualWeights[i]);
    }
  }
  if (_updateChecksum)
    updateChecksum();
}

void EEPROM::readMixingRatios() {
  for (uint8_t e = 0; e < NUM_EXTRUDER; e++) {
    for (uint8_t i = 0; i < VIRTUAL_EXTRUDER; i++) {
      extruder[e].virtualWeights[i] =
          HAL::eprGetInt16(EEPROM_EXTRUDER_OFFSET + e * EEPROM_EXTRUDER_LENGTH +
                           EPR_EXTRUDER_MIXING_RATIOS + 2 * i);
    }
  }
}

void EEPROM::restoreMixingRatios() {
  for (uint8_t e = 0; e < NUM_EXTRUDER; e++) {
    for (uint8_t i = 0; i < VIRTUAL_EXTRUDER; i++) {
      extruder[e].virtualWeights[i] = 10;
    }
  }
}

#endif

void EEPROM::setZCorrection(int32_t c, int index) {
  HAL::eprSetInt32(2048 + (index << 2), c);
}

#endif
