extern const char extzCalibGCode[] PROGMEM;
extern const char extzCalibGCode2[] PROGMEM;
extern const char calibrationGCode[] PROGMEM;
extern const char removeBedGCode[] PROGMEM;

void Felix100MS() {
#ifndef TEC4
  if (PrintLine::linesCount == 0) {
    TemperatureController *bed = tempController[NUM_TEMPERATURE_LOOPS - 1];
    if (bed->currentTemperatureC < MIN_DEFECT_TEMPERATURE) {
      bed->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
      if (!Printer::isAnyTempsensorDefect()) {
        Printer::setAnyTempsensorDefect();
        reportTempsensorError();
        UI_MESSAGE(2); // show message
      }
    }

    // Test if bed is back
    if ((bed->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) ==
            TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT &&
        bed->currentTemperatureC > 0) {
      GCode::resetFatalError();
      bed->flags &= ~TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
      Printer::unsetAnyTempsensorDefect();
      UI_RESET_MENU
#ifndef TEC4
      Printer::unsetHomedAll();
#endif
    }
  }
#endif
}

bool probeValueNew = false;
bool probeValueOld;
char probeMessageOld[22];
bool changeFilWaitTarget = false;

void reportPrintStatus() {
  int idle = 1;
#if NUM_TEMPERATURE_LOOPS > 0
  for (uint8_t i = 0; i <= HEATED_BED_INDEX; i++) {
    TemperatureController *c = tempController[i];
    if (c->targetTemperatureC > 0) {
      idle = 0;
    }
  }
#endif
  if (PrintLine::hasLines()) {
    idle = 0;
  }
  Com::printFLN(PSTR("PrinterIdle:"), idle);
}

void Felix500MS() {
#ifndef TEC4
  if (PrintLine::linesCount == 0) {
    Endstops::update(); // need to update endstops
    Endstops::update();
    probeValueNew = Endstops::zProbe();
    // check if something is changed with probe
    if (probeValueNew != probeValueOld) {
      probeValueOld = probeValueNew;
      // probe is triggered, take action
      if (probeValueNew) {
        // store old message
        strncpy(probeMessageOld, uid.statusMsg, 21);
        // make screen update with new message
        UI_STATUS_UPD_F(PSTR("Z sensor triggered!"));
      } else {
        // probe is not triggered;
        // restore old message
        UI_STATUS_RAM(probeMessageOld);
      }
    }
  }
#endif
  if (changeFilWaitTarget) {
    if (Extruder::current->tempControl.currentTemperatureC >=
        Extruder::current->tempControl.targetTemperatureC
#ifndef TEC4
            - 15
#else
            - 2
#endif
    ) {
      changeFilWaitTarget = false;
      uid.executeAction(UI_ACTION_WIZARD_FILAMENTCHANGE, true);
    }
  }
}

void FelixContainCoordinates() {
  TemperatureController *bed = tempController[NUM_TEMPERATURE_LOOPS - 1];
  if ((bed->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) ==
      TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) {
    Printer::destinationSteps[X_AXIS] = Printer::currentPositionSteps[X_AXIS];
    Printer::destinationSteps[Y_AXIS] = Printer::currentPositionSteps[Y_AXIS];
    Printer::destinationSteps[Z_AXIS] = Printer::currentPositionSteps[Z_AXIS];
    Printer::currentPositionSteps[E_AXIS] =
        Printer::destinationSteps[E_AXIS]; // to prevent fast e move when
                                           // reactivated
    Printer::updateCurrentPosition(true);
  }
}

void cstmCooldown() {
  for (int i = 0; i < NUM_EXTRUDER; i++)
    Extruder::setTemperatureForExtruder(0, i);
#if HAVE_HEATED_BED
  Extruder::setHeatedBedTemperature(0);
#endif
}
bool cstmIsHeating() {
  return extruder[0].tempControl.targetTemperatureC > 0 ||
#if NUM_EXTRUDER > 1
         extruder[1].tempControl.targetTemperatureC > 0 ||
#endif
         heatedBedController.targetTemperatureC > 0;
}
void setPreheatTemps(int16_t extr, int16_t bed, bool all,
                     bool showMenu = true) {
  bool mod = false;
  mod |= heatedBedController.preheatTemperature != bed;
  heatedBedController.preheatTemperature = bed;
  if (all) {
    mod |= extruder[0].tempControl.preheatTemperature != extr;
    extruder[0].tempControl.preheatTemperature = extr;
#if NUM_EXTRUDER > 1
    mod |= extruder[1].tempControl.preheatTemperature != extr;
    extruder[1].tempControl.preheatTemperature = extr;
#endif
  } else {
    mod |= Extruder::current->tempControl.preheatTemperature != extr;
    Extruder::current->tempControl.preheatTemperature = extr;
  }
  if (mod) {
#if EEPROM_MODE != 0
    HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP,
                     heatedBedController.preheatTemperature);
    for (int i = 0; i < NUM_EXTRUDER; i++) {
      int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
      Extruder *e = &extruder[i];
      HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT,
                       e->tempControl.preheatTemperature);
    }
    EEPROM::updateChecksum();
#endif
  }
  if (showMenu)
    uid.pushMenu(&ui_menu_preheatinfo, true);
}

// Preheat to preselected temperature for active extruder and continue to
// filamentchange when finished.
void preheatFCActive() {

  Extruder::setTemperatureForExtruder(
      Extruder::current->tempControl.preheatTemperature, Extruder::current->id,
      false);
  changeFilWaitTarget = true;
  uid.popMenu(false);
  uid.pushMenu(&ui_menu_ch3, true);
}

#ifdef HALFAUTOMATIC_LEVELING
// Measure fixed point height and P1
void halfautomaticLevel2() {
  uid.popMenu(false);
  uid.pushMenu(&cui_msg_measuring, true);
  PlaneBuilder planeBuilder;
  Printer::moveToReal(HALF_FIX_X, HALF_FIX_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float halfRefHeight = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_FIX_X, HALF_FIX_Y, halfRefHeight);
  Printer::moveToReal(HALF_P1_X, HALF_P1_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float p1 = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_P1_X, HALF_P1_Y, p1);
  Printer::moveToReal(HALF_P2_X, HALF_P2_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float p2 = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_P2_X, HALF_P2_Y, p2);
  Plane plane;
  planeBuilder.createPlane(plane);
  // float z1 = p1 + (p2 - p1) / (HALF_P2_Y - HALF_P1_Y) * (HALF_WHEEL_P1 -
  // HALF_P1_Y) - halfRefHeight; float z2 = p1 + (p2 - p1) / (HALF_P2_Y -
  // HALF_P1_Y) * (HALF_WHEEL_P2 - HALF_P1_Y) - halfRefHeight;
#ifdef TEC4
  float z1 = (plane.z(HALF_P1_X, HALF_WHEEL_P1) - halfRefHeight - .15) * 360 /
             HALF_PITCH; // added by FELIX extra offset of 0.1
  float z2 = (plane.z(HALF_P1_X, HALF_WHEEL_P2) - halfRefHeight - .15) * 360 /
             HALF_PITCH; // added by FELIX extra offset of 0.1
#else
  float z1 =
      (plane.z(HALF_P1_X, HALF_WHEEL_P1) - halfRefHeight) * 360 / HALF_PITCH;
  float z2 =
      (plane.z(HALF_P1_X, HALF_WHEEL_P2) - halfRefHeight) * 360 / HALF_PITCH;
#endif
  Printer::wizardStack[0].f = z2;
  Printer::wizardStack[1].f = z1;
  uid.popMenu(false);
  if (fabs(z1) <= 15 && fabs(z2) <= 15) {
    Printer::finishProbing();
    uid.pushMenu(&cui_calib_zprobe_dist, true);
    Printer::measureDistortion();
    uid.popMenu(false);
    uid.pushMenu(&cui_calib_zprobe_succ, true);
  } else {
    uid.pushMenu(&ui_half_show, true);
  }
}
// Finish leveling
void halfautomaticLevel3() {
  Printer::finishProbing();
  uid.popMenu(false);
}
/* Start autoleveling */
void halfautomaticLevel1() {
  uid.pushMenu(&cui_msg_measuring, true);
  Printer::distortion.resetCorrection();
  Printer::distortion.disable(false);
#ifndef TEC4
  Extruder::setTemperatureForExtruder(170, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(170, 1, false);
  }
#else
  Extruder::setTemperatureForExtruder(120, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(120, 1, false);
  }
#endif
  Extruder::setHeatedBedTemperature(55, true);
  EVENT_WAITING_HEATER(-1);
  tempController[HEATED_BED_INDEX]->waitForTargetTemperature();
  EVENT_HEATING_FINISHED(-1);
  Extruder::setTemperatureForExtruder(0, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(0, 1, false);
  }
  Printer::homeAxis(true, true, true);
  Extruder::setTemperatureForExtruder(0, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(0, 1, false);
  }
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, HALF_Z,
                      IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  Printer::moveToReal(HALF_FIX_X, HALF_FIX_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Printer::startProbing(true);
  uid.popMenu(false);
  halfautomaticLevel2();
}

// Measure fixed point height and P1
void halfautomaticLevel2GC() {
  Com::printFLN(PSTR("ZLeveling:measuring"));
  PlaneBuilder planeBuilder;
  Printer::moveToReal(HALF_FIX_X, HALF_FIX_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float halfRefHeight = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_FIX_X, HALF_FIX_Y, halfRefHeight);
  Printer::moveToReal(HALF_P1_X, HALF_P1_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float p1 = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_P1_X, HALF_P1_Y, p1);
  Printer::moveToReal(HALF_P2_X, HALF_P2_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Commands::waitUntilEndOfAllMoves();
  float p2 = Printer::runZProbe(false, false);
  planeBuilder.addPoint(HALF_P2_X, HALF_P2_Y, p2);
  Plane plane;
  planeBuilder.createPlane(plane);
  // float z1 = p1 + (p2 - p1) / (HALF_P2_Y - HALF_P1_Y) * (HALF_WHEEL_P1 -
  // HALF_P1_Y) - halfRefHeight; float z2 = p1 + (p2 - p1) / (HALF_P2_Y -
  // HALF_P1_Y) * (HALF_WHEEL_P2 - HALF_P1_Y) - halfRefHeight;
#ifdef TEC4
  float z1 = (plane.z(HALF_P1_X, HALF_WHEEL_P1) - halfRefHeight - .15) * 360 /
             HALF_PITCH; // added by FELIX extra offset of 0.1
  float z2 = (plane.z(HALF_P1_X, HALF_WHEEL_P2) - halfRefHeight - .15) * 360 /
             HALF_PITCH; // added by FELIX extra offset of 0.1
#else
  float z1 =
      (plane.z(HALF_P1_X, HALF_WHEEL_P1) - halfRefHeight) * 360 / HALF_PITCH;
  float z2 =
      (plane.z(HALF_P1_X, HALF_WHEEL_P2) - halfRefHeight) * 360 / HALF_PITCH;
#endif
  // z1 = back, z2 = front
  Com::printF(PSTR("ZLeveling:result back:"), z1, 2);
  Com::printFLN(PSTR(" front:"), z2, 2);
}
// Finish leveling
void halfautomaticLevel3GC() {
  Printer::finishProbing();
  Com::printFLN(PSTR("ZLeveling:finished"));
}
/* Start autoleveling */
void halfautomaticLevel1GC() {
  Com::printFLN(PSTR("ZLeveling:warmup"));
  Printer::distortion.resetCorrection();
  Printer::distortion.disable(false);
#ifndef TEC4
  Extruder::setTemperatureForExtruder(170, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(170, 1, false);
  }
#else
  Extruder::setTemperatureForExtruder(120, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(120, 1, false);
  }
#endif
  Extruder::setHeatedBedTemperature(55, true);
  EVENT_WAITING_HEATER(-1);
  tempController[HEATED_BED_INDEX]->waitForTargetTemperature();
  EVENT_HEATING_FINISHED(-1);
  Extruder::setTemperatureForExtruder(0, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(0, 1, false);
  }
  Printer::homeAxis(true, true, true);
  Extruder::setTemperatureForExtruder(0, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(0, 1, false);
  }
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, HALF_Z,
                      IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  Printer::moveToReal(HALF_FIX_X, HALF_FIX_Y, IGNORE_COORDINATE,
                      IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  Printer::startProbing(true);
  halfautomaticLevel2GC();
}

#endif

#ifdef ZPROBE_HEIGHT_ROUTINE
/*
- Click on function
- Message preparing
- Home, go to P1 from z leveling
- Probe position for reference height.
- Go to your preset z value.
- Message to adjust hight with wheel
Change Z with wheel
until card fits with
no force below and
press the button
- Compute new z probe height from this.
- Back to menu.


*/
float refZ;
bool distEnabled;

void cZPHeight1(bool menu = true) {
  if (menu)
    uid.pushMenu(&cui_msg_preparing, true);
#if DISTORTION_CORRECTION
  distEnabled = Printer::distortion.isEnabled();
  Printer::distortion.disable(
      false); // if level has changed, distortion is also invalid
#endif
  // Extruder::disableAllHeater();
#ifndef TEC4
  Extruder::setTemperatureForExtruder(170, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(170, 1, false);
  }
#else
  Extruder::setTemperatureForExtruder(120, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(120, 1, false);
  }
#endif
  Extruder::setHeatedBedTemperature(55, true);
  EVENT_WAITING_HEATER(-1);
  tempController[HEATED_BED_INDEX]->waitForTargetTemperature();
  EVENT_HEATING_FINISHED(-1);
  Extruder::setTemperatureForExtruder(0, 0, false);
  if (NUM_EXTRUDER > 1) {
    Extruder::setTemperatureForExtruder(0, 1, false);
  }

  Printer::homeAxis(true, true, true);
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,
                      EEPROM::zProbeBedDistance(), IGNORE_COORDINATE,
                      Printer::homingFeedrate[Z_AXIS]);
  //  Printer::moveToReal(HALF_FIX_X, HALF_FIX_Y, IGNORE_COORDINATE,
  //  IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
  // Printer::moveToReal(ZHOME_X_POS, ZHOME_Y_POS, IGNORE_COORDINATE,
  // IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED); //added by FELIXprinters
  Printer::moveToReal(100, 100, IGNORE_COORDINATE, IGNORE_COORDINATE,
                      EXTRUDER_SWITCH_XY_SPEED); // added by FELIXprinters
  // refZ = Printer::runZProbe(true, true) - EEPROM::zProbeBedDistance() -
  // Printer::zBedOffset;
  refZ = 0;
  // Com::printF(PSTR("
  // cur:"),Printer::currentPosition[Z_AXIS],3);Com::printF(PSTR(" refZ:"),
  // refZ, 3);Com::printFLN(PSTR(" atZ:"), EEPROM::zProbeBedDistance(), 3);
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,
                      ZPROBE_REF_HEIGHT - refZ, IGNORE_COORDINATE,
                      Printer::homingFeedrate[Z_AXIS]);
  Printer::updateCurrentPosition(true);
  Extruder::disableAllHeater(); // added by FELIX, ensure all heaters are off
                                // for correct reading of induction sensor
  Com::printFLN(PSTR("Z-Probe calibration started"));
  if (menu) {
    uid.popMenu(false);
    uid.pushMenu(&cui_calib_zprobe_info, true);
  }
}

void cZPHeight2(bool menu = true) {
#if FEATURE_Z_PROBE
  // float diff = refZ + Printer::currentPosition[Z_AXIS] - ZPROBE_REF_HEIGHT;
  Commands::printCurrentPosition();
  float diff =
      (Printer::lastCmdPos[Z_AXIS] - refZ) - (ZPROBE_REF_HEIGHT - refZ);
  // Com::printF(PSTR("oldZPH:"),EEPROM::zProbeHeight(),3);Com::printF(PSTR("
  // diff:"), diff,3);Com::printF(PSTR("
  // cur:"),Printer::currentPosition[Z_AXIS],3); Com::printFLN(PSTR("
  // REFH:"),(float)ZPROBE_REF_HEIGHT, 2);
  if (diff > 1)
    diff = 1;
  if (diff < -1)
    diff = -1;
  Printer::currentPositionSteps[Z_AXIS] =
      ZPROBE_REF_HEIGHT * Printer::axisStepsPerMM[Z_AXIS];
  Printer::updateCurrentPosition(true);
  float zProbeHeight = EEPROM::zProbeHeight() - diff;
#if EEPROM_MODE != 0 // Com::tZProbeHeight is not declared when EEPROM_MODE is 0
  EEPROM::setZProbeHeight(zProbeHeight); // will also report on output
#else
  Com::printFLN(PSTR("Z-probe height [mm]:"), zProbeHeight);
#endif
  Com::printFLN(PSTR("Probe calibrated"));
  if (menu) {
    uid.popMenu(false);
    uid.menuLevel = 0;
  }
  UI_STATUS_UPD("Probe calibrated");
#endif
#if DISTORTION_CORRECTION
  if (distEnabled)
    Printer::distortion.enable(false);
#endif
  Extruder::disableAllHeater(); // commented by FELIX
}
#endif

bool cCustomParser(char c1, char c2) {
  if (c1 == 'b') {
    if (c2 == 'a') {
      float x = Printer::wizardStack[0].f;
      uid.addFloat(fabs(x), 0, 0);
      uid.addChar(2);
      uid.addChar(x < 0 ? 'R' : 'L');
      return true;
    }
    if (c2 == 'b') {
      float x = Printer::wizardStack[1].f;
      uid.addFloat(fabs(x), 0, 0);
      uid.addChar(2);
      uid.addChar(x < 0 ? 'R' : 'L');
      return true;
    }
  }
  return false;
}

bool cExecuteOverride(int action, bool allowMoves) {
  switch (action) {
  case UI_ACTION_LOAD_EEPROM:
    EEPROM::readDataFromEEPROM(true);
    EEPROM::storeDataIntoEEPROM(false);
    Extruder::selectExtruderById(Extruder::current->id);
    uid.pushMenu(&ui_menu_eeprom_loaded, false);
    BEEP_LONG;
    return true;
#if FEATURE_AUTOLEVEL & FEATURE_Z_PROBE
  case UI_ACTION_AUTOLEVEL2:
    uid.popMenu(false);
    uid.pushMenu(&ui_msg_calibrating_bed, true);
    Printer::homeAxis(true, true, true);
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, 3,
                        IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    if (!runBedLeveling(2)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_autolevel_failed, true);
    } else {
#if DISTORTION_CORRECTION     
	    Printer::measureDistortion(); // add for G33 distortion correction
#endif	    
      uid.popMenu(true);
    }
    Extruder::disableAllHeater();
    return true;
#endif
  }
  return false;
}

#ifndef TEC4
bool zDiffCalib() { // G134 P0 S1\n
  // - G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in
  // nozzle!) Px = reference extruder, Sx = only measure extrude x against
  // reference, Zx = add to measured z distance for Sx for correction.
  float z = 0;
  int p = 0;
  int s = 1;
  int startExtruder = Extruder::current->id;
  extruder[p].zOffset = 0;
  float mins[NUM_EXTRUDER], maxs[NUM_EXTRUDER], avg[NUM_EXTRUDER];
  for (int i = 0; i < NUM_EXTRUDER;
       i++) { // silence unnecessary compiler warning
    avg[i] = 0;
  }
  bool bigError = false;

#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
  float actTemp[NUM_EXTRUDER];
  for (int i = 0; i < NUM_EXTRUDER; i++)
    actTemp[i] = extruder[i].tempControl.targetTemperatureC;
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZHOME_HEAT_HEIGHT,
                      IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  Commands::waitUntilEndOfAllMoves();
#if ZHOME_HEAT_ALL
  for (int i = 0; i < NUM_EXTRUDER; i++) {
    Extruder::setTemperatureForExtruder(
        RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i,
        false, false);
  }
  for (int i = 0; i < NUM_EXTRUDER; i++) {
    if (extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
      Extruder::setTemperatureForExtruder(
          RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i,
          false, true);
  }
#else
  if (extruder[Extruder::current->id].tempControl.currentTemperatureC <
      ZPROBE_MIN_TEMPERATURE)
    Extruder::setTemperatureForExtruder(
        RMath::max(actTemp[Extruder::current->id],
                   static_cast<float>(ZPROBE_MIN_TEMPERATURE)),
        Extruder::current->id, false, true);
#endif
#endif

#ifndef G134_REPETITIONS
#define G134_REPETITIONS 3
#endif
#ifndef G134_PRECISION
#define G134_PRECISION 0.05
#endif
  Printer::startProbing(true);
  for (int r = 0; r < G134_REPETITIONS && !bigError; r++) {
    Extruder::selectExtruderById(p);
    float refHeight = Printer::runZProbe(false, false);
    if (refHeight == ILLEGAL_Z_PROBE) {
      bigError = true;
      break;
    }
    for (int i = 0; i < NUM_EXTRUDER && !bigError; i++) {
      if (i == p)
        continue;
      if (s >= 0 && i != s)
        continue;
      extruder[i].zOffset = 0;
      Extruder::selectExtruderById(i);
      float height = Printer::runZProbe(false, false);
      if (height == ILLEGAL_Z_PROBE) {
        bigError = true;
        break;
      }
      float off = (height - refHeight + z);
      if (r == 0) {
        avg[i] = mins[i] = maxs[i] = off;
      } else {
        avg[i] += off;
        if (off < mins[i])
          mins[i] = off;
        if (off > maxs[i])
          maxs[i] = off;
        if (maxs[i] - mins[i] > G134_PRECISION) {
          Com::printErrorFLN(PSTR(
              "Deviation between measurements were too big, please repeat."));
          bigError = true;
          break;
        }
      }
    }
  }
  if (!bigError) {
    for (int i = 0; i < NUM_EXTRUDER; i++) {
      if (s >= 0 && i != s)
        continue;
      extruder[i].zOffset =
          avg[i] * Printer::axisStepsPerMM[Z_AXIS] / G134_REPETITIONS;
    }
#if EEPROM_MODE != 0
    EEPROM::storeDataIntoEEPROM(0);
#endif
  }
  Extruder::selectExtruderById(startExtruder);
  Printer::finishProbing();
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE
#if ZHOME_HEAT_ALL
  for (int i = 0; i < NUM_EXTRUDER; i++)
    Extruder::setTemperatureForExtruder(actTemp[i], i, false, false);
  for (int i = 0; i < NUM_EXTRUDER; i++)
    Extruder::setTemperatureForExtruder(actTemp[i], i, false,
                                        actTemp[i] > MAX_ROOM_TEMPERATURE);
#else
  Extruder::setTemperatureForExtruder(
      actTemp[Extruder::current->id], Extruder::current->id, false,
      actTemp[Extruder::current->id] > MAX_ROOM_TEMPERATURE);
#endif
#endif
  return bigError;
}
#endif

void cExecute(int action, bool allowMoves) {
  switch (action) {
#ifdef HALFAUTOMATIC_LEVELING
  case UI_ACTION_HALFAUTO_LEV:
    halfautomaticLevel1();
    break;
  case UI_ACTION_HALFAUTO_LEV2:
    halfautomaticLevel2();
    break;
  case UI_ACTION_HALFAUTO_LEV3:
    halfautomaticLevel3();
    break;
#endif
  case UI_ACTION_XY1_BACK:
    uid.popMenu(true);
    break;
  case UI_ACTION_XY1_CONT:
    uid.popMenu(false);
    uid.pushMenu(&ui_exy2, true);
    break;
  case UI_ACTION_XY2_BACK:
    uid.popMenu(true);
    break;
  case UI_ACTION_XY2_CONT:
    uid.popMenu(false);
    uid.pushMenu(&ui_msg_printxycal, true);
    flashSource.executeCommands(calibrationGCode, false, UI_ACTION_CALEX_XY3);
    break;
  case UI_ACTION_CALEX_XY3:
    Printer::wizardStackPos = 0;
    Printer::wizardStack[0].l = 5;
    Printer::wizardStack[1].l = 5;
    uid.popMenu(false);
    uid.pushMenu(&ui_exy3, true);
    break;
  case UI_ACTION_EXY_STORE: {
    int32_t xcor = static_cast<int32_t>(
        (Printer::axisStepsPerMM[X_AXIS] * (Printer::wizardStack[0].l - 5)) /
        10);
    int32_t ycor = static_cast<int32_t>(
        (Printer::axisStepsPerMM[Y_AXIS] * (Printer::wizardStack[1].l - 5)) /
        10);
    Com::printF(PSTR(" xOffset_before:"), extruder[1].xOffset, 3);
#ifdef TEC4
    extruder[1].xOffset += xcor;
#else
    extruder[1].xOffset -= xcor;
#endif
    Com::printF(PSTR(" xCor:"), xcor, 3);
    Com::printF(PSTR(" xOffset_after:"), extruder[1].xOffset, 3);
    Com::printF(PSTR(" yOffset_before:"), extruder[1].yOffset, 3);
    extruder[1].yOffset += ycor;
    Com::printF(PSTR(" yCor:"), ycor, 3);
    Com::printFLN(PSTR(" yOffset_after:"), extruder[1].yOffset, 3);
    if (xcor != 0 || ycor != 0)
      EEPROM::storeDataIntoEEPROM(false);
    uid.popMenu(true);
  } break;
  case UI_ACTION_CALEX:
    uid.pushMenu(&ui_calextr_sub, true);
    break;
  case UI_ACTION_CALEX_XY:
    Printer::wizardStackPos = 0;
    Printer::wizardStack[0].l = 5;
    Printer::wizardStack[1].l = 5;
    uid.pushMenu(&ui_exy1, true);
    break;
  case UI_ACTION_CALEX_Z:
    uid.pushMenu(&ui_msg_clearbed_zcalib, true);
    break;
  case UI_ACTION_CALEX_Z3:
    uid.popMenu(true);
    break;
  case UI_ACTION_PRECOOL1:
    if (cstmIsHeating()) {
      cstmCooldown();
    } else {
      Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature);
      Extruder::setTemperatureForExtruder(
          extruder[0].tempControl.preheatTemperature, 0, false);
#if NUM_EXTRUDER > 1
      Extruder::setTemperatureForExtruder(0, 1, false);
#endif
    }
    break;
  case UI_ACTION_PRECOOL2:
    if (cstmIsHeating()) {
      cstmCooldown();
    } else {
      Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature);
      Extruder::setTemperatureForExtruder(
          extruder[0].tempControl.preheatTemperature, 0, false);
#if NUM_EXTRUDER > 0
      Extruder::setTemperatureForExtruder(
          extruder[1].tempControl.preheatTemperature, 1, false);
#endif
    }
    break;
  case UI_ACTION_REMOVEBED: {
    flashSource.executeCommands(removeBedGCode, false, 0);
    /* TemperatureController *bed = tempController[NUM_TEMPERATURE_LOOPS-1];
    Extruder::setHeatedBedTemperature(0);
    if(!Printer::isZHomed())
      Printer::homeAxis(true,true,true);
    else
      Printer::homeAxis(false,true,false);
    Printer::moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE, 200,
    IGNORE_COORDINATE, 30,false); */
  } break;
  case UI_ACTION_SPH_PLA_ACTIVE: // 190/55
    setPreheatTemps(190, 55, false);
    break;
  case UI_ACTION_SPH_PETG_ACTIVE:
    setPreheatTemps(215, 70, false);
    break;
  case UI_ACTION_SPH_PVA_ACTIVE:
    setPreheatTemps(185, 55, false);
    break;
  case UI_ACTION_SPH_FLEX_ACTIVE:
    setPreheatTemps(185, 40, false);
    break;
  case UI_ACTION_SPH_ABS_ACTIVE:
    setPreheatTemps(225, 85, false);
    break;
  case UI_ACTION_SPH_GLASS_ACTIVE:
    setPreheatTemps(225, 85, false);
    break;
  case UI_ACTION_SPH_WOOD_ACTIVE:
    setPreheatTemps(185, 55, false);
    break;
  case UI_ACTION_SPH_PLA_ALL:
    setPreheatTemps(190, 55, true);
    break;
  case UI_ACTION_SPH_PETG_ALL:
    setPreheatTemps(215, 70, true);
    break;
  case UI_ACTION_SPH_PVA_ALL:
    setPreheatTemps(185, 55, true);
    break;
  case UI_ACTION_SPH_FLEX_ALL:
    setPreheatTemps(185, 40, true);
    break;
  case UI_ACTION_SPH_ABS_ALL:
    setPreheatTemps(225, 85, true);
    break;
  case UI_ACTION_SPH_GLASS_ALL:
    setPreheatTemps(225, 85, true);
    break;
  case UI_ACTION_SPH_WOOD_ALL:
    setPreheatTemps(185, 55, true);
    break;
  // Filament change related functions
  case UI_ACTION_FC_SELECT1:
    Extruder::selectExtruderById(0);
    uid.popMenu(false);
    uid.pushMenu(&ui_menu_ch2, true);
    break;
  case UI_ACTION_FC_SELECT2:
#ifndef TEC4
    if (!Printer::isHomedAll()) { // added by FELIX
      // Printer::homeAxis(true,true,false);
      Extruder::selectExtruderById(1);
    } else {
      Extruder::selectExtruderById(1);
    }
#else
    Extruder::selectExtruderById(1);
#endif
    uid.popMenu(false);
    uid.pushMenu(&ui_menu_ch2, true);
    break;
  case UI_ACTION_FC_PLA:
    setPreheatTemps(190, 55, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_PETG:
    setPreheatTemps(215, 70, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_PVA:
    setPreheatTemps(210, 55, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_FLEX:
    setPreheatTemps(185, 40, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_ABS:
    setPreheatTemps(225, 85, false, false);
    preheatFCActive();                                      /*
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
#if !defined(_UI_CUSTOM_MENU_H)
#define _UI_CUSTOM_MENU_H


#if UI_DISPLAY_TYPE != NO_DISPLAY

UI_MENU_ACTIONCOMMAND_T(ui_menu_back, UI_TEXT_BACK_ID, UI_ACTION_BACK)
#if UI_HAS_BACK_KEY == 0
#define UI_MENU_ADDCONDBACK &ui_menu_back,
#define UI_MENU_ADDCONDBACK_C ,&ui_menu_back
#define UI_MENU_BACKCNT 1
#else
#define UI_MENU_ADDCONDBACK
#define UI_MENU_ADDCONDBACK_C
#define UI_MENU_BACKCNT 0
#endif

// new submenues and entries

UI_MENU_HEADLINE_T(ui_empty,UI_TEXT_EMPTY_ID)
UI_MENU_HEADLINE_T(ui_exy1a,UI_CTEXT_XY_P1_1_ID)
UI_MENU_HEADLINE_T(ui_exy1b,UI_CTEXT_XY_P1_2_ID)
UI_MENU_HEADLINE_T(ui_exy1c,UI_CTEXT_XY_P1_3_ID)
UI_MENU_ACTIONCOMMAND_T(ui_exy1d,UI_TEXT_BACK_ID,UI_ACTION_XY1_BACK)
UI_MENU_ACTIONCOMMAND_T(ui_exy1e,UI_CTEXT_CONTINUE_ID,UI_ACTION_XY1_CONT)

#define UI_EXY1_ITEMS {&ui_exy1a,&ui_exy1b,&ui_exy1c,&ui_exy1d,&ui_exy1e}
UI_STICKYMENU(ui_exy1,UI_EXY1_ITEMS,5)

UI_MENU_HEADLINE_T(ui_exy2a,UI_TEXT_CLEARBED1_ID)
UI_MENU_HEADLINE_T(ui_exy2b,UI_TEXT_CLEARBED2_ID)
UI_MENU_HEADLINE_T(ui_exy2c,UI_TEXT_CLEARBED3_ID)
UI_MENU_ACTIONCOMMAND_T(ui_exy2d,UI_TEXT_BACK_ID,UI_ACTION_XY2_BACK)
UI_MENU_ACTIONCOMMAND_T(ui_exy2e,UI_CTEXT_START_PRINTING_ID,UI_ACTION_XY2_CONT)

#define UI_EXY2_ITEMS {&ui_exy2a,&ui_exy2b,&ui_exy2c,&ui_exy2d,&ui_exy2e}
UI_STICKYMENU(ui_exy2,UI_EXY2_ITEMS,5)

UI_MENU_HEADLINE_T(ui_exy3a,UI_CTEXT_XY_P3_1_ID)
UI_MENU_HEADLINE_T(ui_exy3b,UI_CTEXT_XY_P3_2_ID)
UI_MENU_CHANGEACTION_T(ui_exy3c,UI_CTEXT_XY_P3_X_ID,UI_ACTION_EXY_XOFFSET)
UI_MENU_CHANGEACTION_T(ui_exy3d,UI_CTEXT_XY_P3_Y_ID,UI_ACTION_EXY_YOFFSET)
UI_MENU_ACTIONCOMMAND_T(ui_exy3e,UI_CTEXT_CONTINUE_ID,UI_ACTION_EXY_STORE)

#define UI_EXY3_ITEMS {&ui_exy3a,&ui_exy3b,&ui_exy3c,&ui_exy3d,&ui_exy3e}
UI_STICKYMENU(ui_exy3,UI_EXY3_ITEMS,5)

UI_MENU_ACTIONCOMMAND_T(ui_calex,UI_CTEXT_CALIBRATE_EXTRUDERS_ID,UI_ACTION_CALEX)
UI_MENU_ACTIONCOMMAND_T(ui_calex_xy,UI_CTEXT_CALIBRATE_XY_ID,UI_ACTION_CALEX_XY)
UI_MENU_ACTIONCOMMAND(ui_calex_xyv2,"Calibrate XY w. Card",UI_ACTION_EXTRXY_V2)
//added by FELIX
//====================
#ifdef TEC4
#else
UI_MENU_ACTIONCOMMAND_T(ui_calex_z,UI_CTEXT_CALIBRATE_Z_ID,UI_ACTION_CALEX_Z)
#endif

#ifdef TEC4
#define UI_CALEXTR_SUBITEMS {&ui_menu_back, &ui_calex_xy}
UI_MENU(ui_calextr_sub,UI_CALEXTR_SUBITEMS,2)
#else
#define UI_CALEXTR_SUBITEMS {&ui_menu_back, &ui_calex_z, &ui_calex_xy, &ui_calex_xyv2}
UI_MENU(ui_calextr_sub,UI_CALEXTR_SUBITEMS,4)
#endif
//====================
//added by FELIX

UI_WIZARD4_T(ui_msg_printxycal, UI_ACTION_STATE,UI_CTEXT_PRINTXYCAL1_ID, UI_CTEXT_PRINTXYCAL2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4_T(ui_msg_extzcalib, UI_ACTION_STATE,UI_CTEXT_EXTZCAL1_ID, UI_CTEXT_EXTZCAL2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD5_T(ui_msg_clearbed_zcalib, UI_ACTION_CALEX_Z2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)

UI_MENU_ACTIONCOMMAND_T(ui_preheatcool1,UI_CTEXT_PREHEATCOOL_ID,UI_ACTION_PRECOOL1)
#if NUM_EXTRUDER > 1
UI_MENU_ACTIONCOMMAND_T(ui_preheatcool2,UI_CTEXT_PREHEATCOOL2_ID,UI_ACTION_PRECOOL2)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_removebed,UI_CTEXT_REMOVEBED_ID,UI_ACTION_REMOVEBED)

UI_WIZARD4_T(cui_msg_measuring, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_MEASURING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4(cui_msg_ext_xy_1, UI_ACTION_EXTRXY_V2_1,"Place card at the", "marked spot.", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_error, UI_ACTION_MESSAGE,"Calibration failed", "", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_success, UI_ACTION_MESSAGE,"Extruder Offset", "Calibration Finished.", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_ext_xy_info, UI_ACTION_STATE,"XY Offset Calibration", "   Measuring ...", "", "   Please wait")
UI_WIZARD4(cui_msg_autolevel_failed, UI_ACTION_MESSAGE,"Autoleveling failed", "Plate is not leveled!", "", "     >>> OK <<<")
UI_WIZARD4(cui_msg_exzautolevel_failed, UI_ACTION_MESSAGE,"Autoleveling failed", "Extruder offsets", " not leveled!", "     >>> OK <<<")

#ifndef TEC4
UI_WIZARD4(cui_msg_preparing, UI_ACTION_STATE,"", "   Preparing ...", "", "   Please wait")
#else
UI_WIZARD4(cui_msg_preparing, UI_ACTION_STATE,"", "   Warming up...", "", "   Please wait")
#endif              
UI_WIZARD4(cui_calib_zprobe_info, UI_ACTION_CZREFH_INFO, "   Place the Felix","   calibration card"," between the Nozzle","   and Build PLT.")
UI_WIZARD4(cui_calib_zprobe, UI_ACTION_CZREFH, "  Rotate the button","   until you feel","   slight friction","    on the card.")
UI_WIZARD4(cui_calib_zprobe_succ, UI_ACTION_CZREFH_SUCC, "     Build PLT."," Leveled Correctly","","     >>> OK <<<")
UI_WIZARD4(cui_calib_zprobe_dist, UI_ACTION_STATE, "", "   now measuring", "    buildsurface", "")
UI_MENU_HEADLINE(ui_halfw," Max. 15" cDEG)
UI_MENU_HEADLINE(ui_halfa," Turn back: %bb")
UI_MENU_HEADLINE(ui_halfb," Turn front: %ba")
UI_MENU_ACTIONCOMMAND(ui_halfc,"Measure Again",UI_ACTION_HALFAUTO_LEV2)
UI_MENU_ACTIONCOMMAND(ui_halfd,"Cancel",UI_ACTION_HALFAUTO_LEV3)

#define UI_HALF_ITEMS {&ui_halfw,&ui_halfa,&ui_halfb,&ui_halfc,&ui_halfd}
UI_STICKYMENU(ui_half_show,UI_HALF_ITEMS,5)


// Define precision for temperatures. With small displays only integer values fit.
#undef UI_TEMP_PRECISION
#define UI_TEMP_PRECISION 0

// Define precision for temperatures. With small displays only integer values fit.
#ifndef UI_TEMP_PRECISION
#if UI_COLS>16
#define UI_TEMP_PRECISION 0
#else
#define UI_TEMP_PRECISION 0
#endif
#endif

#if HAVE_HEATED_BED // make sure it is only 0 or 1 to be used in menu math
#undef HAVE_HEATED_BED
#define HAVE_HEATED_BED 1
#else
#undef HAVE_HEATED_BED
#define HAVE_HEATED_BED 0
#endif

/* ============= PAGES DEFINITION =============

  If you are not inside a menu, the firmware displays pages with information.
  Especially if you have only a small display it is convenient to have
  more then one information page.
*/

/* Define your pages using dynamic strings. To define a page use
  UI_PAGE6(name,row1,row2,row3,row4,row5,row6);
  UI_PAGE4(name,row1,row2,row3,row4);
  for 4 row displays and
  UI_PAGE2(name,row1,row2);
  for 2 row displays. You can add additional pages or change the default pages like you want.
*/

#if UI_ROWS>=5 && UI_DISPLAY_TYPE == DISPLAY_U8G

//graphic main status

UI_PAGE6_T(ui_page1, UI_TEXT_MAINPAGE6_1_ID, UI_TEXT_MAINPAGE6_2_ID, UI_TEXT_MAINPAGE6_3_ID, UI_TEXT_MAINPAGE6_4_ID, UI_TEXT_MAINPAGE6_5_ID, UI_TEXT_MAINPAGE6_6_ID)

#if EEPROM_MODE != 0  && 0
UI_PAGE6_T(ui_page2, UI_TEXT_PRINT_TIME_ID, UI_TEXT_PRINT_TIME_VALUE_ID, UI_TEXT_PRINT_FILAMENT_ID, UI_TEXT_PRINT_FILAMENT_VALUE_ID,UI_TEXT_EMPTY_ID,UI_TEXT_STATUS_ID)
#define UI_PRINTTIME_PAGES ,&ui_page2
#define UI_PRINTTIME_COUNT 1
#else
#define UI_PRINTTIME_PAGES
#define UI_PRINTTIME_COUNT 0
#endif

#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_PAGE6_T(ui_page3, UI_TEXT_EXTR0_TEMP_ID, UI_TEXT_EXTR1_TEMP_ID, 
#if NUM_EXTRUDER > 2
  UI_TEXT_EXTR2_TEMP_ID,
#endif
#if NUM_EXTRUDER > 3
           UI_TEXT_EXTR3_TEMP_ID,
#endif
#if NUM_EXTRUDER > 4
           UI_TEXT_EXTR4_TEMP_ID,
#endif
#if HAVE_HEATED_BED && NUM_EXTRUDER < 5
           UI_TEXT_BED_TEMP_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 5
           UI_TEXT_EMPTY_ID, // UI_TEXT_FLOW_MULTIPLY_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 3
           UI_TEXT_EMPTY_ID,
#endif
#if NUM_EXTRUDER + HAVE_HEATED_BED < 4
            UI_TEXT_EMPTY_ID,
#endif
           UI_TEXT_STATUS_ID
)
#define UI_EXTRUDERS_PAGES ,&ui_page3
#define UI_EXTRUDERS_PAGES_COUNT 1
#elif NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1
UI_PAGE6_T(ui_page3, UI_TEXT_EXTR0_TEMP_ID,
#if HAVE_HEATED_BED
UI_TEXT_BED_TEMP_ID,
#endif
 UI_TEXT_FLOW_MULTIPLY_ID,
 UI_TEXT_EMPTY_ID,
 UI_TEXT_EMPTY_ID,
#if !HAVE_HEATED_BED
 UI_TEXT_EMPTY_ID,
#endif  
 UI_TEXT_STATUS_ID
)
#define UI_EXTRUDERS_PAGES ,&ui_page3
#define UI_EXTRUDERS_PAGES_COUNT 1
#else
#define UI_EXTRUDERS_PAGES
#define UI_EXTRUDERS_PAGES_COUNT 0
#endif

UI_PAGE6_T(ui_page4, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4A_ID,UI_TEXT_EMPTY_ID,UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID)

/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {/*&ui_page1 UI_PRINTTIME_PAGES ,*/&ui_page4 UI_EXTRUDERS_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 1/*+UI_PRINTTIME_COUNT*/+UI_EXTRUDERS_PAGES_COUNT

#elif UI_ROWS >= 4
#if HAVE_HEATED_BED
#if NUM_EXTRUDER > 0
//   UI_PAGE4(ui_page1,cTEMP "%ec/%Ec" cDEG "B%eB/%Eb" cDEG,"Z:%x2  Buf : %oB","Mul: %om   Flow: %of","%os")
UI_PAGE4_T(ui_page1, UI_TEXT_MAINPAGE_TEMP_BED_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#else
//   UI_PAGE4(ui_page1,"B%eB/%Eb" cDEG,"Z:%x2  Buf : %oB","Mul: %om   Flow: %of","%os")
UI_PAGE4_T(ui_page1, UI_TEXT_MAINPAGE_BED_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#endif
//UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,UI_TEXT_PAGE_BED,UI_TEXT_PAGE_BUFFER,"%os");
#else // no bed
#if NUM_EXTRUDER > 0
UI_PAGE4_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#else
UI_PAGE4_T(ui_page1, UI_TEXT_EMPTY_ID, UI_TEXT_MAINPAGE_Z_BUF_ID, UI_TEXT_MAINPAGE_MUL_EUSAGE_ID, UI_TEXT_STATUS_ID)
#endif
#endif
UI_PAGE4_T(ui_page2, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_STATUS_ID)
//UI_PAGE4(ui_page2,"dX:%y0 mm %sX","dY:%y1 mm %sY","dZ:%y2 mm %sZ","%os");
#if NUM_EXTRUDER > 0
UI_PAGE4_T(ui_page3, UI_TEXT_PAGE_EXTRUDER1_ID
#else
UI_PAGE4_T(ui_page3
#endif
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
           , UI_TEXT_PAGE_EXTRUDER2_ID
#endif
#if NUM_EXTRUDER>2 && MIXING_EXTRUDER == 0
           , UI_TEXT_PAGE_EXTRUDER3_ID
#endif
#if HAVE_HEATED_BED
           , UI_TEXT_PAGE_BED_ID
#endif
#if (NUM_EXTRUDER >= 3 && MIXING_EXTRUDER == 0 && !HAVE_HEATED_BED) || (NUM_EXTRUDER==2 && MIXING_EXTRUDER == 0 && HAVE_HEATED_BED==true)
           , UI_TEXT_STATUS_ID
#elif (NUM_EXTRUDER == 2 && MIXING_EXTRUDER == 0) || ((NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1) && HAVE_HEATED_BED)
           , UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#elif (NUM_EXTRUDER == 1 || MIXING_EXTRUDER == 1) || (NUM_EXTRUDER == 0 &&  HAVE_HEATED_BED)
           , UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#elif NUM_EXTRUDER == 0
           , UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_EMPTY_ID, UI_TEXT_STATUS_ID
#endif
          )
#if EEPROM_MODE != 0
UI_PAGE4_T(ui_page4, UI_TEXT_PRINT_TIME_ID, UI_TEXT_PRINT_TIME_VALUE_ID, UI_TEXT_PRINT_FILAMENT_ID, UI_TEXT_PRINT_FILAMENT_VALUE_ID)
#define UI_PRINTTIME_PAGES ,&ui_page4
#define UI_PRINTTIME_COUNT 1
#else
#define UI_PRINTTIME_PAGES
#define UI_PRINTTIME_COUNT 0
#endif
/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {/*&ui_page1,*/ &ui_page2, &ui_page3 UI_PRINTTIME_PAGES}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2+UI_PRINTTIME_COUNT
#else
#if HAVE_HEATED_BED
UI_PAGE2_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_PAGE_BED_ID)
#else
UI_PAGE2_T(ui_page1, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_STATUS_ID)
#endif
UI_PAGE2_T(ui_page2, UI_TEXT_MAINPAGE_XY_ID, UI_TEXT_STATUS_ID)
UI_PAGE2_T(ui_page3, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_STATUS_ID)
/*
  Merge pages together. Use the following pattern:
  #define UI_PAGES {&name1,&name2,&name3}
*/
#define UI_PAGES {/* &ui_page1,*/ &ui_page2,&ui_page3}
// How many pages do you want to have. Minimum is 1.
#define UI_NUM_PAGES 2
#endif

/* ============ MENU definition ================

  The menu works the same as pages. In additon you need to define what the lines do
  or where to jump to. For that reason, the menu structure needs to be entered in
  reverse order. Starting from the leaves, the menu structure is defined.
*/

/*
  At first define all actions available in the menu. The actions define, what the
  next/previous button will do. All menu actions work the same:
  next/previous changes the value
  ok sets the value if not already done and goes back to previous menu.
*/



// Language selection menu

#if EEPROM_MODE != 0
#define FIRSTLANG 1
#if LANGUAGE_EN_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_en, "English", UI_ACTION_LANGUAGE_EN | UI_ACTION_TOPMENU)
#define ADD_LANG_EN &ui_menu_setlang_en
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_EN
#endif // LANGUAGE_EN_ACTIVE
#if LANGUAGE_DE_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_de, "Deutsch", UI_ACTION_LANGUAGE_DE | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_DE &ui_menu_setlang_de
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_DE ,&ui_menu_setlang_de
#endif
#else
#define ADD_LANG_DE
#endif // LANGUAGE_DE_ACTIVE
#if LANGUAGE_ES_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_es, "Espanol", UI_ACTION_LANGUAGE_ES | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_ES &ui_menu_setlang_es
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_ES ,&ui_menu_setlang_es
#endif
#else
#define ADD_LANG_ES
#endif // LANGUAGE_ES_ACTIVE
#if LANGUAGE_PT_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_pt, "Portugues", UI_ACTION_LANGUAGE_PT | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_PT &ui_menu_setlang_pt
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_PT ,&ui_menu_setlang_pt
#endif
#else
#define ADD_LANG_PT
#endif // LANGUAGE_PT_ACTIVE
#if LANGUAGE_FR_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_fr, "Francais", UI_ACTION_LANGUAGE_FR | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_FR &ui_menu_setlang_fr
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_FR ,&ui_menu_setlang_fr
#endif
#else
#define ADD_LANG_FR
#endif // LANGUAGE_FR_ACTIVE
#if LANGUAGE_NL_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_nl, "Nederlandse", UI_ACTION_LANGUAGE_NL | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_NL &ui_menu_setlang_nl
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_NL ,&ui_menu_setlang_nl
#endif
#else
#define ADD_LANG_NL
#endif // LANGUAGE_NL_ACTIVE
#if LANGUAGE_IT_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_it, "Italiano", UI_ACTION_LANGUAGE_IT | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_IT &ui_menu_setlang_it
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_IT ,&ui_menu_setlang_it
#endif
#else
#define ADD_LANG_IT
#endif // LANGUAGE_IT_ACTIVE
#if LANGUAGE_FI_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_fi, "Suomi", UI_ACTION_LANGUAGE_FI | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_SE &ui_menu_setlang_fi
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_FI ,&ui_menu_setlang_fi
#endif
#else
#define ADD_LANG_FI
#endif // LANGUAGE_FI_ACTIVE
#if LANGUAGE_SE_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_se, "Svenska", UI_ACTION_LANGUAGE_SE | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_SE &ui_menu_setlang_se
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_SE ,&ui_menu_setlang_se
#endif
#else
#define ADD_LANG_SE
#endif // LANGUAGE_SE_ACTIVE
#if LANGUAGE_CZ_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_cz, "Cestina", UI_ACTION_LANGUAGE_CZ | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_CZ &ui_menu_setlang_cz
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_CZ ,&ui_menu_setlang_cz
#endif
#else
#define ADD_LANG_CZ
#endif // LANGUAGE_CZ_ACTIVE
#if LANGUAGE_PL_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_pl, "Polski", UI_ACTION_LANGUAGE_PL | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_PL &ui_menu_setlang_pl
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_PL ,&ui_menu_setlang_pl
#endif
#else
#define ADD_LANG_PL
#endif // LANGUAGE_PL_ACTIVE
#if LANGUAGE_TR_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_tr, "T" STR_uuml "rk", UI_ACTION_LANGUAGE_TR | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_TR &ui_menu_setlang_tr
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_TR ,&ui_menu_setlang_tr
#endif
#else
#define ADD_LANG_TR
#endif // LANGUAGE_TR_ACTIVE
#if LANGUAGE_RU_ACTIVE
UI_MENU_ACTIONCOMMAND(ui_menu_setlang_ru, "Russian", UI_ACTION_LANGUAGE_RU | UI_ACTION_TOPMENU)
#if FIRSTLANG
#define ADD_LANG_RU &ui_menu_setlang_ru
#undef FIRSTLANG
#define FIRSTLANG 0
#else
#define ADD_LANG_RU ,&ui_menu_setlang_ru
#endif
#else
#define ADD_LANG_RU
#endif // LANGUAGE_RU_ACTIVE

#define UI_MENU_LANGUAGES {UI_MENU_ADDCONDBACK ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR ADD_LANG_RU}
#define UI_MENU_LANGUAGES_WIZ {ADD_LANG_EN ADD_LANG_DE ADD_LANG_ES ADD_LANG_PT ADD_LANG_FR ADD_LANG_NL ADD_LANG_IT ADD_LANG_FI ADD_LANG_SE ADD_LANG_CZ ADD_LANG_PL ADD_LANG_TR ADD_LANG_RU}
UI_MENU(ui_menu_languages, UI_MENU_LANGUAGES, UI_MENU_BACKCNT + LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE + LANGUAGE_RU_ACTIVE)
UI_MENU_SUBMENU_T(ui_menu_conf_lang, UI_TEXT_LANGUAGE_ID, ui_menu_languages)
UI_STICKYMENU(ui_menu_languages_wiz, UI_MENU_LANGUAGES_WIZ, LANGUAGE_EN_ACTIVE + LANGUAGE_DE_ACTIVE + LANGUAGE_ES_ACTIVE + LANGUAGE_PT_ACTIVE + LANGUAGE_FR_ACTIVE + LANGUAGE_NL_ACTIVE + LANGUAGE_IT_ACTIVE + LANGUAGE_FI_ACTIVE + LANGUAGE_SE_ACTIVE + LANGUAGE_CZ_ACTIVE + LANGUAGE_PL_ACTIVE + LANGUAGE_TR_ACTIVE + LANGUAGE_RU_ACTIVE)
#define LANGMENU_ENTRY &ui_menu_conf_lang,
#define LANGMENU_COUNT 1
#else
#define LANGMENU_ENTRY
#define LANGMENU_COUNT 0
#endif

// Helper line

UI_MENU_HEADLINE_T(ui_menu_empty,UI_TEXT_EMPTY_ID)


// Error menu

UI_MENU_ACTION2_T(ui_menu_error, UI_ACTION_DUMMY, UI_TEXT_ERROR_ID, UI_TEXT_ERRORMSG_ID)

// Messages
#if UI_ROWS >= 4
UI_WIZARD4_T(ui_msg_decoupled, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_HEATER_DECOUPLED_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_defectsensor, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_TEMPSENSOR_DEFECT_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_slipping, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_SLIPPING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_leveling_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_LEVELING_ERROR_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD4_T(ui_msg_calibration_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_CALIBRATION_ERROR_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD5_T(ui_msg_clearbed1, UI_ACTION_AUTOLEVEL2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID, UI_TEXT_OK_ID)
UI_WIZARD5_T(ui_msg_clearbed2, UI_ACTION_MEASURE_DISTORTION2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID, UI_TEXT_CLEARBED3_ID, UI_TEXT_EMPTY_ID,  UI_TEXT_OK_ID)

UI_WIZARD4_T(ui_msg_calibrating_bed, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_CALIBRATING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD4_T(ui_msg_homing, UI_ACTION_STATE,UI_TEXT_EMPTY_ID, UI_TEXT_HOMING_ID, UI_TEXT_EMPTY_ID, UI_TEXT_PLEASE_WAIT_ID)
#else
UI_WIZARD2_T(ui_msg_decoupled, UI_ACTION_MESSAGE,  UI_TEXT_NOTIFICATION_ID, UI_TEXT_HEATER_DECOUPLED_ID)
UI_WIZARD2_T(ui_msg_defectsensor, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_TEMPSENSOR_DEFECT_ID)
UI_WIZARD2_T(ui_msg_slipping, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_SLIPPING_ID)
UI_WIZARD2_T(ui_msg_leveling_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_LEVELING_ERROR_ID)
UI_WIZARD2_T(ui_msg_calibration_error, UI_ACTION_MESSAGE, UI_TEXT_NOTIFICATION_ID, UI_TEXT_CALIBRATION_ERROR_ID)
UI_WIZARD2_T(ui_msg_clearbed1, UI_ACTION_AUTOLEVEL2, UI_TEXT_CLEARBED1_ID, UI_TEXT_CLEARBED2_ID)

UI_WIZARD2_T(ui_msg_calibrating_bed, UI_ACTION_STATE, UI_TEXT_CALIBRATING_ID, UI_TEXT_PLEASE_WAIT_ID)
UI_WIZARD2_T(ui_msg_homing, UI_ACTION_STATE, UI_TEXT_HOMING_ID, UI_TEXT_PLEASE_WAIT_ID)
#endif

// Filament change wizard

#if FEATURE_RETRACTION
#if UI_ROWS >= 4
UI_WIZARD4_T(ui_wiz_filamentchange, UI_ACTION_WIZARD_FILAMENTCHANGE, UI_TEXT_WIZ_CH_FILAMENT1_ID, UI_TEXT_WIZ_CH_FILAMENT2_ID, UI_TEXT_WIZ_CH_FILAMENT3_ID, UI_TEXT_CLICK_DONE_ID)
UI_WIZARD4_T(ui_wiz_jamwaitheat, UI_ACTION_WIZARD_JAM_WAITHEAT, UI_TEXT_WIZ_WAITTEMP1_ID, UI_TEXT_WIZ_WAITTEMP2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_TEMP_SET_ID)
UI_WIZARD4_T(ui_wiz_jamreheat, UI_ACTION_WIZARD_JAM_REHEAT, UI_TEXT_WIZ_REHEAT1_ID, UI_TEXT_WIZ_REHEAT2_ID, UI_TEXT_EMPTY_ID, UI_TEXT_CURRENT_TEMP_ID)
#else
UI_WIZARD2_T(ui_wiz_filamentchange, UI_ACTION_WIZARD_FILAMENTCHANGE, UI_TEXT_WIZ_CH_FILAMENT1_ID, UI_TEXT_CLICK_DONE_ID)
UI_WIZARD2_T(ui_wiz_jamwaitheat, UI_ACTION_WIZARD_JAM_WAITHEAT, UI_TEXT_WIZ_WAITTEMP1_ID, UI_TEXT_WIZ_WAITTEMP2_ID)
UI_WIZARD2_T(ui_wiz_jamreheat, UI_ACTION_WIZARD_JAM_REHEAT, UI_TEXT_WIZ_REHEAT1_ID, UI_TEXT_WIZ_REHEAT2_ID)
#endif
#endif

// **** Positions sub menus

#if UI_ROWS >= 4
UI_MENU_ACTION4_T(ui_menu_xpos, UI_ACTION_XPOSITION, UI_TEXT_ACTION_XPOSITION4A_ID, UI_TEXT_ACTION_XPOSITION4B_ID, UI_TEXT_ACTION_XPOSITION4C_ID, UI_TEXT_ACTION_XPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_ypos, UI_ACTION_YPOSITION, UI_TEXT_ACTION_YPOSITION4A_ID, UI_TEXT_ACTION_YPOSITION4B_ID, UI_TEXT_ACTION_YPOSITION4C_ID, UI_TEXT_ACTION_YPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos, UI_ACTION_ZPOSITION, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4B_ID, UI_TEXT_ACTION_ZPOSITION4C_ID, UI_TEXT_ACTION_ZPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_notest, UI_ACTION_ZPOSITION_NOTEST, UI_TEXT_ACTION_ZPOSITION4A_ID, UI_TEXT_ACTION_ZPOSITION4B_ID, UI_TEXT_ACTION_ZPOSITION4C_ID, UI_TEXT_ACTION_ZPOSITION4D_ID)
UI_MENU_ACTION4_T(ui_menu_xpos_fast, UI_ACTION_XPOSITION_FAST, UI_TEXT_ACTION_XPOSITION_FAST4A_ID, UI_TEXT_ACTION_XPOSITION_FAST4B_ID, UI_TEXT_ACTION_XPOSITION_FAST4C_ID, UI_TEXT_ACTION_XPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_ypos_fast, UI_ACTION_YPOSITION_FAST, UI_TEXT_ACTION_YPOSITION_FAST4A_ID, UI_TEXT_ACTION_YPOSITION_FAST4B_ID, UI_TEXT_ACTION_YPOSITION_FAST4C_ID, UI_TEXT_ACTION_YPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_fast, UI_ACTION_ZPOSITION_FAST, UI_TEXT_ACTION_ZPOSITION_FAST4A_ID, UI_TEXT_ACTION_ZPOSITION_FAST4B_ID, UI_TEXT_ACTION_ZPOSITION_FAST4C_ID, UI_TEXT_ACTION_ZPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_zpos_fast_notest, UI_ACTION_ZPOSITION_FAST_NOTEST, UI_TEXT_ACTION_ZPOSITION_FAST4A_ID, UI_TEXT_ACTION_ZPOSITION_FAST4B_ID, UI_TEXT_ACTION_ZPOSITION_FAST4C_ID, UI_TEXT_ACTION_ZPOSITION_FAST4D_ID)
UI_MENU_ACTION4_T(ui_menu_epos, UI_ACTION_EPOSITION, UI_TEXT_ACTION_EPOSITION_FAST2A_ID, UI_TEXT_ACTION_EPOSITION_FAST2B_ID, UI_TEXT_PAGE_EXTRUDER_ID, UI_TEXT_METER_PRINTED_ID)
#else
UI_MENU_ACTION2_T(ui_menu_xpos, UI_ACTION_XPOSITION, UI_TEXT_ACTION_XPOSITION2A_ID, UI_TEXT_ACTION_XPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_ypos, UI_ACTION_YPOSITION, UI_TEXT_ACTION_YPOSITION2A_ID, UI_TEXT_ACTION_YPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos, UI_ACTION_ZPOSITION, UI_TEXT_ACTION_ZPOSITION2A_ID, UI_TEXT_ACTION_ZPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_notest, UI_ACTION_ZPOSITION_NOTEST, UI_TEXT_ACTION_ZPOSITION2A_ID, UI_TEXT_ACTION_ZPOSITION2B_ID)
UI_MENU_ACTION2_T(ui_menu_xpos_fast, UI_ACTION_XPOSITION_FAST, UI_TEXT_ACTION_XPOSITION_FAST2A_ID, UI_TEXT_ACTION_XPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_ypos_fast, UI_ACTION_YPOSITION_FAST, UI_TEXT_ACTION_YPOSITION_FAST2A_ID, UI_TEXT_ACTION_YPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_fast, UI_ACTION_ZPOSITION_FAST, UI_TEXT_ACTION_ZPOSITION_FAST2A_ID, UI_TEXT_ACTION_ZPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_zpos_fast_notest, UI_ACTION_ZPOSITION_FAST_NOTEST, UI_TEXT_ACTION_ZPOSITION_FAST2A_ID, UI_TEXT_ACTION_ZPOSITION_FAST2B_ID)
UI_MENU_ACTION2_T(ui_menu_epos, UI_ACTION_EPOSITION, UI_TEXT_ACTION_EPOSITION_FAST2A_ID, UI_TEXT_ACTION_EPOSITION_FAST2B_ID)
#endif

/*
  Next step is to define sub menus leading to the action.
*/

// **** Positioning menu
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_all, UI_TEXT_HOME_ALL_ID, UI_ACTION_HOME_ALL, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_x, UI_TEXT_HOME_X_ID, UI_ACTION_HOME_X, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_y, UI_TEXT_HOME_Y_ID, UI_ACTION_HOME_Y, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_home_z, UI_TEXT_HOME_Z_ID, UI_ACTION_HOME_Z, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_xpos, UI_TEXT_X_POSITION_ID, ui_menu_xpos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_ypos, UI_TEXT_Y_POSITION_ID, ui_menu_ypos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zpos, UI_TEXT_Z_POSITION_ID, ui_menu_zpos)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zpos_notest, UI_TEXT_Z_POSITION_ID, ui_menu_zpos_notest)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_epos, UI_TEXT_E_POSITION_ID, ui_menu_epos)
#if !UI_SPEEDDEPENDENT_POSITIONING
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_xfast, UI_TEXT_X_POS_FAST_ID, ui_menu_xpos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_yfast, UI_TEXT_Y_POS_FAST_ID, ui_menu_ypos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zfast, UI_TEXT_Z_POS_FAST_ID, ui_menu_zpos_fast)
UI_MENU_ACTIONSELECTOR_T(ui_menu_go_zfast_notest, UI_TEXT_Z_POS_FAST_ID, ui_menu_zpos_fast_notest)
#define UI_SPEED 2
#define UI_SPEED_X ,&ui_menu_go_xfast,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_yfast,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zfast,&ui_menu_go_zpos
#define UI_SPEED_Z_NOTEST ,&ui_menu_go_zfast_notest,&ui_menu_go_zpos_notest
#else
#define UI_SPEED 1
#define UI_SPEED_X ,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zpos
#define UI_SPEED_Z_NOTEST ,&ui_menu_go_zpos_notest
#endif
#if FEATURE_SERVO > 0 && UI_SERVO_CONTROL > 0
UI_MENU_CHANGEACTION_T(ui_menu_servopos, UI_TEXT_SERVOPOS_ID, UI_ACTION_SERVOPOS)
#define SERVOPOS_COUNT 1
#define SERVOPOS_ENTRY ,&ui_menu_servopos
#else
#define SERVOPOS_COUNT 0
#define SERVOPOS_ENTRY
#endif
// Offsets menu
UI_MENU_CHANGEACTION_T(ui_menu_off_xpos, UI_TEXT_X_OFFSET_ID, UI_ACTION_XOFF)
UI_MENU_CHANGEACTION_T(ui_menu_off_ypos, UI_TEXT_Y_OFFSET_ID, UI_ACTION_YOFF)
UI_MENU_CHANGEACTION_T(ui_menu_off_zpos, UI_TEXT_Z_OFFSET_ID, UI_ACTION_ZOFF)
#define UI_MENU_OFFSETS {UI_MENU_ADDCONDBACK &ui_menu_off_xpos,&ui_menu_off_ypos,&ui_menu_off_zpos}
UI_MENU(ui_menu_offsets, UI_MENU_OFFSETS, UI_MENU_BACKCNT + 3)
UI_MENU_SUBMENU_T(ui_menu_go_offsets, UI_TEXT_OFFSETS_ID, ui_menu_offsets)

#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z UI_SPEED_X UI_SPEED_Y UI_SPEED_Z SERVOPOS_ENTRY}
UI_MENU(ui_menu_positions, UI_MENU_POSITIONS, 4 + 3 * UI_SPEED + UI_MENU_BACKCNT + SERVOPOS_COUNT)

#if FEATURE_Z_PROBE
UI_MENU_HEADLINE_T(ui_menu_mzp_head,UI_TEXT_MEAS_ZP_HEIGHT_ID)
UI_MENU_CHANGEACTION_T(ui_menu_mzp_realz,UI_TEXT_REAL_Z_ID,UI_ACTION_MEASURE_ZP_REALZ)
UI_MENU_ACTIONCOMMAND_T(ui_menu_mzp_cont, UI_TEXT_CONTINUE_ID, UI_ACTION_MEASURE_ZPROBE_HEIGHT2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_mzp_close, UI_TEXT_CLOSE_ID, UI_ACTION_BACK)
#define UI_MENU_MZP_ITEMS {&ui_menu_mzp_head,&ui_menu_mzp_realz,&ui_menu_mzp_cont,&ui_menu_mzp_close}
UI_STICKYMENU(ui_menu_mzp,UI_MENU_MZP_ITEMS,4)
UI_MENU_ACTIONCOMMAND_T(ui_menu_measure_zprobe_height, UI_TEXT_MEAS_ZP_HEIGHT_ID, UI_ACTION_MEASURE_ZPROBE_HEIGHT)
#define UI_CAL_ZHEIGHT_ENTRY ,&ui_menu_measure_zprobe_height
#define UI_CAL_ZHEIGHT_CNT 1
#else
#define UI_CAL_ZHEIGHT_ENTRY
#define UI_CAL_ZHEIGHT_CNT 0
#endif

// **** Delta calibration menu
#if Z_HOME_DIR > 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_measured_origin, UI_TEXT_SET_MEASURED_ORIGIN_ID, UI_ACTION_SET_MEASURED_ORIGIN)
#define UI_MENU_DELTA {UI_MENU_ADDCONDBACK &ui_menu_home_all UI_SPEED_Z_NOTEST,&ui_menu_set_measured_origin}
UI_MENU(ui_menu_delta, UI_MENU_DELTA, 2 + UI_SPEED + UI_MENU_BACKCNT)
#endif

// **** Bed leveling menu
#ifdef SOFTWARE_LEVELING
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p1, UI_TEXT_SET_P1_ID, UI_ACTION_SET_P1)
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p2, UI_TEXT_SET_P2_ID, UI_ACTION_SET_P2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_set_p3, UI_TEXT_SET_P3_ID, UI_ACTION_SET_P3)
UI_MENU_ACTIONCOMMAND_T(ui_menu_calculate_leveling, UI_TEXT_CALCULATE_LEVELING_ID, UI_ACTION_CALC_LEVEL)
#define UI_MENU_LEVEL {UI_MENU_ADDCONDBACK &ui_menu_set_p1,&ui_menu_set_p2,&ui_menu_set_p3,&ui_menu_calculate_leveling UI_SPEED_X UI_SPEED_Y UI_SPEED_Z}
UI_MENU(ui_menu_level, UI_MENU_LEVEL, 4 + 3 * UI_SPEED + UI_MENU_BACKCNT)
#endif

// **** Extruder menu
#if NUM_EXTRUDER > 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp0, UI_TEXT_EXTR0_TEMP_ID, UI_ACTION_EXTRUDER0_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp0_printing, UI_TEXT_EXTR0_TEMP_ID, UI_ACTION_EXTRUDER0_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP0_PRINTING ,&ui_menu_ext_temp0_printing
#define UI_TEMP0_CNT 1
#else
#define UI_TEMP0_PRINTING
#define UI_TEMP0_CNT 0
#endif
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp1, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp1_printing, UI_TEXT_EXTR1_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
#define UI_TEMP1_PRINTING ,&ui_menu_ext_temp1_printing
#define UI_TEMP1_CNT 1
#else
#define UI_TEMP1_PRINTING
#define UI_TEMP1_CNT 0
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp2, UI_TEXT_EXTR2_TEMP_ID, UI_ACTION_EXTRUDER2_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp2_printing, UI_TEXT_EXTR2_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP2_PRINTING ,&ui_menu_ext_temp2_printing
#define UI_TEMP2_CNT 1
#else
#define UI_TEMP2_PRINTING
#define UI_TEMP2_CNT 0
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp3, UI_TEXT_EXTR3_TEMP_ID, UI_ACTION_EXTRUDER3_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp3_printing, UI_TEXT_EXTR3_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP3_PRINTING ,&ui_menu_ext_temp3_printing
#define UI_TEMP3_CNT 1
#else
#define UI_TEMP3_PRINTING
#define UI_TEMP3_CNT 0
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp4, UI_TEXT_EXTR4_TEMP_ID, UI_ACTION_EXTRUDER4_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp4_printing, UI_TEXT_EXTR4_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP4_PRINTING ,&ui_menu_ext_temp4_printing
#define UI_TEMP4_CNT 1
#else
#define UI_TEMP4_PRINTING
#define UI_TEMP4_CNT 0
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_ext_temp5, UI_TEXT_EXTR5_TEMP_ID, UI_ACTION_EXTRUDER5_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_ext_temp5_printing, UI_TEXT_EXTR5_TEMP_ID, UI_ACTION_EXTRUDER1_TEMP,MENU_MODE_PRINTING,0)
#define UI_TEMP5_PRINTING ,&ui_menu_ext_temp5_printing
#define UI_TEMP5_CNT 1
#else
#define UI_TEMP5_PRINTING
#define UI_TEMP5_CNT 0
#endif
UI_MENU_CHANGEACTION_T(ui_menu_bed_temp, UI_TEXT_BED_TEMP_ID, UI_ACTION_HEATED_BED_TEMP)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_bed_temp_printing, UI_TEXT_BED_TEMP_ID, UI_ACTION_HEATED_BED_TEMP,MENU_MODE_PRINTING,0)
#if HAVE_HEATED_BED
#define UI_BED_TEMP_PRINTING ,&ui_menu_bed_temp_printing
#else
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel0, UI_TEXT_EXTR0_SELECT_ID, UI_ACTION_SELECT_EXTRUDER0)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel1, UI_TEXT_EXTR1_SELECT_ID, UI_ACTION_SELECT_EXTRUDER1)
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel2, UI_TEXT_EXTR2_SELECT_ID, UI_ACTION_SELECT_EXTRUDER2)
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel3, UI_TEXT_EXTR3_SELECT_ID, UI_ACTION_SELECT_EXTRUDER3)
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel4, UI_TEXT_EXTR4_SELECT_ID, UI_ACTION_SELECT_EXTRUDER4)
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_sel5, UI_TEXT_EXTR5_SELECT_ID, UI_ACTION_SELECT_EXTRUDER5)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off0, UI_TEXT_EXTR0_OFF_ID, UI_ACTION_EXTRUDER0_OFF)
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off1, UI_TEXT_EXTR1_OFF_ID, UI_ACTION_EXTRUDER1_OFF)
#endif
#if NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off2, UI_TEXT_EXTR2_OFF_ID, UI_ACTION_EXTRUDER2_OFF)
#endif
#if NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off3, UI_TEXT_EXTR3_OFF_ID, UI_ACTION_EXTRUDER3_OFF)
#endif
#if NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off4, UI_TEXT_EXTR4_OFF_ID, UI_ACTION_EXTRUDER4_OFF)
#endif
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_off5, UI_TEXT_EXTR5_OFF_ID, UI_ACTION_EXTRUDER5_OFF)
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_origin, UI_TEXT_EXTR_ORIGIN_ID, UI_ACTION_RESET_EXTRUDER)
#if FEATURE_DITTO_PRINTING
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto0, UI_TEXT_DITTO_0_ID, UI_DITTO_0)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto1, UI_TEXT_DITTO_1_ID, UI_DITTO_1)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto2, UI_TEXT_DITTO_2_ID, UI_DITTO_2)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ext_ditto3, UI_TEXT_DITTO_3_ID, UI_DITTO_3)
#if NUM_EXTRUDER == 3
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1,&ui_menu_ext_ditto2
#define UI_DITTO_COMMANDS_COUNT 3
#elif NUM_EXTRUDER == 4
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1,&ui_menu_ext_ditto2,&ui_menu_ext_ditto3
#define UI_DITTO_COMMANDS_COUNT 4
#else
#define UI_DITTO_COMMANDS ,&ui_menu_ext_ditto0,&ui_menu_ext_ditto1
#define UI_DITTO_COMMANDS_COUNT 2
#endif
#else
#define UI_DITTO_COMMANDS
#define UI_DITTO_COMMANDS_COUNT 0
#endif
#if MIXING_EXTRUDER || NUM_EXTRUDER == 1
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_off0,
#define UI_MENU_EXTCNT 2
#define UI_MENU_EXTSEL
#define UI_MENU_EXTSEL_CNT 0
#elif NUM_EXTRUDER == 2
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_sel0,&ui_menu_ext_sel1,
#define UI_MENU_EXTCNT 6
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1
#define UI_MENU_EXTSEL_CNT 2
#elif NUM_EXTRUDER == 3
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,
#define UI_MENU_EXTCNT 9
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2
#define UI_MENU_EXTSEL_CNT 3
#elif NUM_EXTRUDER == 4
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,
#define UI_MENU_EXTCNT 12
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3
#define UI_MENU_EXTSEL_CNT 4
#elif NUM_EXTRUDER == 5
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_temp4,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_off4,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,
#define UI_MENU_EXTCNT 15
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4
#define UI_MENU_EXTSEL_CNT 5
#elif NUM_EXTRUDER == 6
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_temp2,&ui_menu_ext_temp3,&ui_menu_ext_temp4,&ui_menu_ext_temp5,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_ext_off2,&ui_menu_ext_off3,&ui_menu_ext_off4,&ui_menu_ext_off5,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel5,
#define UI_MENU_EXTCNT 18
#define UI_MENU_EXTSEL ,&ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel4
#define UI_MENU_EXTSEL_CNT 6
#elif NUM_EXTRUDER == 0
#define UI_MENU_EXTCOND
#define UI_MENU_EXTCNT 0
#define UI_MENU_EXTSEL
#define UI_MENU_EXTSEL_CNT 0
#endif
#if HAVE_HEATED_BED
#define UI_MENU_BEDCOND &ui_menu_bed_temp,
#define UI_MENU_BEDCNT 1
#else
#define UI_MENU_BEDCOND
#define UI_MENU_BEDCNT 0
#endif

#define UI_MENU_EXTRUDER {UI_MENU_ADDCONDBACK UI_MENU_BEDCOND UI_MENU_EXTCOND &ui_menu_go_epos,&ui_menu_ext_origin UI_DITTO_COMMANDS}
UI_MENU(ui_menu_extruder, UI_MENU_EXTRUDER, UI_MENU_BACKCNT + UI_MENU_BEDCNT + UI_MENU_EXTCNT + 2 + UI_DITTO_COMMANDS_COUNT)

// **** SD card menu

// **** Fan menu

#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
UI_MENU_CHANGEACTION_T(ui_menu_fan_fanspeed, UI_TEXT_ACTION_FANSPEED_ID, UI_ACTION_FANSPEED)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_fan_fanspeed_printing, UI_TEXT_ACTION_FANSPEED_ID, UI_ACTION_FANSPEED,MENU_MODE_PRINTING,0)
#define UI_FANSPEED_PRINTING ,&ui_menu_fan_fanspeed_printing
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_fan_off, UI_TEXT_FAN_OFF_ID, UI_ACTION_FAN_OFF, MENU_MODE_FAN_RUNNING, 0)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_25, UI_TEXT_FAN_25_ID, UI_ACTION_FAN_25)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_50, UI_TEXT_FAN_50_ID, UI_ACTION_FAN_50)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_75, UI_TEXT_FAN_75_ID, UI_ACTION_FAN_75)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_full, UI_TEXT_FAN_FULL_ID, UI_ACTION_FAN_FULL)
UI_MENU_ACTIONCOMMAND_T(ui_menu_fan_ignoreM106, UI_TEXT_IGNORE_M106_ID, UI_ACTION_IGNORE_M106)
#define UI_MENU_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_fanspeed,&ui_menu_fan_off,&ui_menu_fan_25,&ui_menu_fan_50,&ui_menu_fan_75,&ui_menu_fan_full,&ui_menu_fan_ignoreM106}
UI_MENU(ui_menu_fan, UI_MENU_FAN, 7 + UI_MENU_BACKCNT)
UI_MENU_SUBMENU_T(ui_menu_fan_sub, UI_TEXT_FANSPEED_ID, ui_menu_fan)
#define UI_MENU_FAN_COND &ui_menu_fan_sub,
#define UI_FANSPEED ,&ui_menu_fan_fanspeed
#define UI_MENU_FAN_CNT 1
#else
#define UI_MENU_FAN_COND
#define UI_FANSPEED
#define UI_MENU_FAN_CNT 0
#define UI_FANSPEED_PRINTING
#endif

#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
UI_MENU_CHANGEACTION_FILTER(ui_menu_fan2_fanspeed_printing,"Fan 2 speed:%FS%%%" /* UI_TEXT_ACTION_FANSPEED_ID*/, UI_ACTION_FAN2SPEED,MENU_MODE_PRINTING,0)
UI_MENU_CHANGEACTION(ui_menu_fan2_fanspeed,"Fan 2 speed:%FS%%%" /* UI_TEXT_ACTION_FANSPEED_ID*/, UI_ACTION_FAN2SPEED)
#define UI_FAN2SPEED_PRINTING ,&ui_menu_fan2_fanspeed_printing
#define UI_FAN2SPEED ,&ui_menu_fan2_fanspeed
#define UI_MENU_FAN2_CNT 1
#else
#define UI_MENU_FAN2_CNT 0
#define UI_FAN2SPEED_PRINTING
#define UI_FAN2SPEED
#endif


// **** General configuration settings

UI_MENU_ACTION2_T(ui_menu_stepper2, UI_ACTION_STEPPER_INACTIVE, UI_TEXT_STEPPER_INACTIVE2A_ID, UI_TEXT_STEPPER_INACTIVE2B_ID)
UI_MENU_ACTION2_T(ui_menu_maxinactive2, UI_ACTION_MAX_INACTIVE, UI_TEXT_POWER_INACTIVE2A_ID, UI_TEXT_POWER_INACTIVE2B_ID)
UI_MENU_CHANGEACTION_T(ui_menu_general_baud, UI_TEXT_BAUDRATE_ID, UI_ACTION_BAUDRATE)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_stepper_inactive, UI_TEXT_STEPPER_INACTIVE_ID, ui_menu_stepper2)
UI_MENU_ACTIONSELECTOR_T(ui_menu_general_max_inactive, UI_TEXT_POWER_INACTIVE_ID, ui_menu_maxinactive2)
#ifdef ZPROBE_HEIGHT_ROUTINE
UI_MENU_ACTIONCOMMAND(ui_menu_calib_probe,"Calibrate Z-Probe",UI_ACTION_START_CZREFH);
#define UI_CALIB_PROBE_ENTRY ,&ui_menu_calib_probe
#define UI_CALIB_PROBE_COUNT 1
#else
#define UI_CALIB_PROBE_ENTRY
#define UI_CALIB_PROBE_COUNT 0
#endif

#if FEATURE_AUTOLEVEL
#ifdef HALFAUTOMATIC_LEVELING
UI_MENU_ACTIONCOMMAND_T(ui_menu_autolevelbed,UI_TEXT_AUTOLEVEL_BED_ID,UI_ACTION_HALFAUTO_LEV);
#define UI_TOOGLE_AUTOLEVEL_ENTRY ,&ui_menu_autolevelbed
#define UI_TOGGLE_AUTOLEVEL_COUNT 1
#else
UI_MENU_ACTIONCOMMAND_T(ui_menu_autolevelbed,UI_TEXT_AUTOLEVEL_BED_ID,UI_ACTION_AUTOLEVEL);
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_autolevel, UI_TEXT_AUTOLEVEL_ONOFF_ID, UI_ACTION_AUTOLEVEL_ONOFF)
#define UI_TOOGLE_AUTOLEVEL_ENTRY ,&ui_menu_autolevelbed,&ui_menu_toggle_autolevel
#define UI_TOGGLE_AUTOLEVEL_COUNT 2
#endif
#else
#define UI_TOOGLE_AUTOLEVEL_ENTRY
#define UI_TOGGLE_AUTOLEVEL_COUNT 0
#endif
#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive UI_TOOGLE_AUTOLEVEL_ENTRY UI_CALIB_PROBE_ENTRY}
UI_MENU(ui_menu_general, UI_MENU_GENERAL, 3 + UI_MENU_BACKCNT + UI_TOGGLE_AUTOLEVEL_COUNT + UI_CALIB_PROBE_COUNT)


// **** Bed Coating Menu

#if UI_BED_COATING
UI_MENU_ACTION2_T(ui_menu_nocoating_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_NOCOATING_ID)
UI_MENU_ACTION2_T(ui_menu_buildtak_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_BUILDTAK_ID)
UI_MENU_ACTION2_T(ui_menu_kapton_action,  UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_KAPTON_ID)
UI_MENU_ACTION2_T(ui_menu_bluetape_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_BLUETAPE_ID)
UI_MENU_ACTION2_T(ui_menu_pettape_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_PETTAPE_ID)
UI_MENU_ACTION2_T(ui_menu_gluestick_action, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_GLUESTICK_ID)
UI_MENU_ACTION2_T(ui_menu_coating_custom, UI_ACTION_DUMMY, UI_TEXT_BED_COATING_SET1_ID, UI_TEXT_COATING_THICKNESS_ID)

UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_nocoating, UI_TEXT_NOCOATING_ID, UI_ACTION_NOCOATING, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_buildtak, UI_TEXT_BUILDTAK_ID, UI_ACTION_BUILDTAK, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_kapton, UI_TEXT_KAPTON_ID, UI_ACTION_KAPTON, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_bluetape, UI_TEXT_BLUETAPE_ID, UI_ACTION_BLUETAPE, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_pettape, UI_TEXT_PETTAPE_ID, UI_ACTION_PETTAPE, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_adjust_gluestick, UI_TEXT_GLUESTICK_ID, UI_ACTION_GLUESTICK, 0, MENU_MODE_PRINTING)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_adjust_custom, UI_TEXT_COATING_CUSTOM_ID, UI_ACTION_COATING_CUSTOM, 0, MENU_MODE_PRINTING)
#define UI_MENU_ADJUST {UI_MENU_ADDCONDBACK &ui_menu_adjust_buildtak,&ui_menu_adjust_kapton,&ui_menu_adjust_custom}
UI_MENU(ui_menu_adjust, UI_MENU_ADJUST, 3 + UI_MENU_BACKCNT)
#define UI_MENU_COATING_CNT 1
#define UI_MENU_COATING_COND &ui_menu_prepare,
UI_MENU_SUBMENU_FILTER_T(ui_menu_prepare, UI_TEXT_BED_COATING_ID, ui_menu_adjust, 0, MENU_MODE_PRINTING)

#else
#define UI_MENU_COATING_CNT 0
#define UI_MENU_COATING_COND
#endif


// **** Quick menu
#if PS_ON_PIN > -1
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_power, UI_TEXT_POWER_ID, UI_ACTION_POWER)
#define MENU_PSON_COUNT 1
#define MENU_PSON_ENTRY ,&ui_menu_quick_power
#else
#define MENU_PSON_COUNT 0
#define MENU_PSON_ENTRY
#endif
#if CASE_LIGHTS_PIN >= 0
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_light, UI_TEXT_LIGHTS_ONOFF_ID, UI_ACTION_LIGHTS_ONOFF)
#define UI_TOOGLE_LIGHT_ENTRY ,&ui_menu_toggle_light
#define UI_TOGGLE_LIGHT_COUNT 1
#else
#define UI_TOOGLE_LIGHT_ENTRY
#define UI_TOGGLE_LIGHT_COUNT 0
#endif
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_pla, UI_TEXT_PREHEAT_SINGLE_ID, UI_ACTION_PREHEAT_SINGLE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_preheat_abs, UI_TEXT_PREHEAT_ALL_ID, UI_ACTION_PREHEAT_ALL)
UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_cooldown, UI_TEXT_COOLDOWN_ID, UI_ACTION_COOLDOWN)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_origin, UI_TEXT_SET_TO_ORIGIN_ID, UI_ACTION_SET_ORIGIN, 0, MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_stopstepper, UI_TEXT_DISABLE_STEPPER_ID, UI_ACTION_DISABLE_STEPPER, 0, MENU_MODE_PRINTING)
#if FEATURE_BABYSTEPPING
UI_MENU_CHANGEACTION_T(ui_menu_quick_zbaby, UI_TEXT_Z_BABYSTEPPING_ID, UI_ACTION_Z_BABYSTEPS)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_zbaby_printing, UI_TEXT_Z_BABYSTEPPING_ID, UI_ACTION_Z_BABYSTEPS,MENU_MODE_PRINTING,0)
#define BABY_CNT 1
#define BABY_ENTRY ,&ui_menu_quick_zbaby
#define BABY_ENTRY_PRINTING ,&ui_menu_quick_zbaby_printing
#else
#define BABY_CNT 0
#define BABY_ENTRY
#define BABY_ENTRY_PRINTING
#endif
UI_MENU_CHANGEACTION_T(ui_menu_quick_speedmultiply, UI_TEXT_SPEED_MULTIPLY_ID, UI_ACTION_FEEDRATE_MULTIPLY)
UI_MENU_CHANGEACTION_T(ui_menu_quick_flowmultiply, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_speedmultiply_printing, UI_TEXT_SPEED_MULTIPLY_ID, UI_ACTION_FEEDRATE_MULTIPLY,MENU_MODE_PRINTING,0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_quick_flowmultiply_printing, UI_TEXT_FLOW_MULTIPLY_ID, UI_ACTION_FLOWRATE_MULTIPLY,MENU_MODE_PRINTING,0)
#ifdef DEBUG_PRINT
UI_MENU_ACTIONCOMMAND(ui_menu_quick_debug, "Write Debug", UI_ACTION_WRITE_DEBUG)
#define DEBUG_PRINT_COUNT 1
#define DEBUG_PRINT_EXTRA ,&ui_menu_quick_debug
#else
#define DEBUG_PRINT_COUNT 0
#define DEBUG_PRINT_EXTRA
#endif

// Change Filament level 1 - extruder selection

UI_MENU_HEADLINE_T(ui_menu_chf_head,UI_TEXT_CHANGE_FILAMENT_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_cf_sel1,UI_CTEXT_SELECT_EX1_ID,UI_ACTION_FC_SELECT1)
#if NUM_EXTRUDER == 1
#define UI_MENU_CF1 {&ui_menu_chf_head, UI_MENU_ADDCONDBACK &ui_menu_cf_sel1}
UI_MENU(ui_menu_chf, UI_MENU_CF1 , 2+UI_MENU_BACKCNT)
#else
UI_MENU_ACTIONCOMMAND_T(ui_menu_cf_sel2,UI_CTEXT_SELECT_EX2_ID,UI_ACTION_FC_SELECT2)
#define UI_MENU_CF1 {&ui_menu_chf_head, UI_MENU_ADDCONDBACK &ui_menu_cf_sel1,&ui_menu_cf_sel2}
UI_MENU(ui_menu_chf, UI_MENU_CF1 , 3+UI_MENU_BACKCNT)
#endif

// Change filament level 2 - material selection

UI_MENU_ACTIONCOMMAND_T(ui_menu_ch2_back, UI_TEXT_BACK_ID, UI_ACTION_FC_BACK1)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_current,"Current: %eIc" cDEG "C",UI_ACTION_WIZARD_FILAMENTCHANGE)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_pla,"PLA",UI_ACTION_FC_PLA)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_petg,"PETG",UI_ACTION_FC_PETG)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_pva,"PVA",UI_ACTION_FC_PVA)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_flex,"FLEX",UI_ACTION_FC_FLEX)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_abs,"ABS",UI_ACTION_FC_ABS)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_glass,"GLASS",UI_ACTION_FC_GLASS)
UI_MENU_ACTIONCOMMAND(ui_menu_ch2_wood,"WOOD",UI_ACTION_FC_WOOD)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ch2_custom,UI_TEXT_CUSTOM_ID,UI_ACTION_FC_CUSTOM)
#define UI_MENU_CH2_ITEMS {&ui_menu_chf_head,&ui_menu_ch2_back,\
&ui_menu_ch2_current,&ui_menu_ch2_pla,&ui_menu_ch2_petg,&ui_menu_ch2_pva,&ui_menu_ch2_flex,&ui_menu_ch2_abs,&ui_menu_ch2_glass,&ui_menu_ch2_wood,\
&ui_menu_ch2_custom}
UI_MENU(ui_menu_ch2,UI_MENU_CH2_ITEMS,11) 

// Change filament - custom temperature

UI_MENU_HEADLINE_T(ui_menu_ch2b_1,UI_CTEXT_SELECT_TEMP_ID)
UI_MENU_HEADLINE(ui_menu_ch2b_2,"%w0\002C")
UI_MENU_HEADLINE_T(ui_menu_ch2b_3,UI_TEXT_CLICK_DONE_ID)
#define UI_MENU_CH2A {&ui_menu_ch2b_1,&ui_menu_ch2b_2,&ui_menu_ch2b_3}
//UI_STICKYMENU(ui_menu_ch2a,UI_MENU_CH2A,3)
UI_WIZARD4_T(ui_menu_ch2a, UI_ACTION_FC_CUSTOM_SET, UI_CTEXT_SELECT_TEMP_ID, UI_CTEXT_WIZ_TEMP_ID, UI_TEXT_EMPTY_ID, UI_TEXT_CLICK_DONE_ID)


// Change filament heating ... info

UI_MENU_HEADLINE_T(ui_menu_ch3_1,UI_CTEXT_HEATINGUP1_ID)
UI_MENU_HEADLINE_T(ui_menu_ch3_2,UI_CTEXT_HEATINGUP2_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_ch3_back, UI_TEXT_BACK_ID, UI_ACTION_FC_BACK1)
#define UI_MENU_CH3 {&ui_menu_ch3_1,&ui_menu_ch3_2,&ui_menu_ch3_back}
UI_STICKYMENU(ui_menu_ch3,UI_MENU_CH3,3)


#if FEATURE_RETRACTION
/* * Extruder 1
O Extruder 2
Temp. 170/180
Continue
Close */

UI_MENU_CHANGEACTION_T(ui_menu_chf_temp,UI_TEXT_CUR_TEMP_ID,UI_ACTION_EXTRUDER_TEMP)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pla,"PLA",UI_ACTION_SPH_PLA_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_petg,"PETG",UI_ACTION_SPH_PETG_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pva,"PVA",UI_ACTION_SPH_PVA_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_flex,"FLEX",UI_ACTION_SPH_FLEX_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_abs,"ABS",UI_ACTION_SPH_ABS_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_glass,"GLASS",UI_ACTION_SPH_GLASS_ACTIVE)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_wood,"WOOD",UI_ACTION_SPH_WOOD_ACTIVE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_continue,UI_TEXT_CONTINUE_ID,UI_ACTION_WIZARD_FILAMENTCHANGE)
UI_MENU_ACTIONCOMMAND_T(ui_menu_chf_close,UI_TEXT_CLOSE_ID,UI_ACTION_BACK)
#define UI_MENU_CHF_ITEMS {&ui_menu_chf_head UI_MENU_EXTSEL ,&ui_menu_chf_temp,\
&ui_menu_chf_sph_pla,&ui_menu_chf_sph_petg,&ui_menu_chf_sph_pva,&ui_menu_chf_sph_flex,&ui_menu_chf_sph_abs,&ui_menu_chf_sph_glass,&ui_menu_chf_sph_wood,\
&ui_menu_chf_continue,&ui_menu_chf_close}
//UI_STICKYMENU(ui_menu_chf,UI_MENU_CHF_ITEMS,11+UI_MENU_EXTSEL_CNT) 
#if NUM_EXTRUDER == 1
UI_MENU_SUBMENU_T(ui_menu_quick_changefil,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2)   
UI_MENU_SUBMENU_FILTER_T(ui_menu_quick_changefil_printing,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2,MENU_MODE_PRINTING,0)
#else
UI_MENU_SUBMENU_T(ui_menu_quick_changefil,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_chf)   
UI_MENU_SUBMENU_FILTER_T(ui_menu_quick_changefil_printing,UI_TEXT_CHANGE_FILAMENT_ID,ui_menu_ch2,MENU_MODE_PRINTING,0)
#endif
//UI_MENU_ACTIONCOMMAND_T(ui_menu_quick_changefil, UI_TEXT_CHANGE_FILAMENT_ID, UI_ACTION_WIZARD_FILAMENTCHANGE)
//UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_quick_changefil_printing, UI_TEXT_CHANGE_FILAMENT_ID, UI_ACTION_WIZARD_FILAMENTCHANGE,MENU_MODE_PRINTING,0)
#define UI_CHANGE_FIL_CNT 1
#define UI_CHANGE_FIL_ENT ,&ui_menu_quick_changefil
#define UI_CHANGE_FIL_ENT_PRINTING ,&ui_menu_quick_changefil_printing
#else
#define UI_CHANGE_FIL_CNT 0
#define UI_CHANGE_FIL_ENT
#define UI_CHANGE_FIL_ENT_PRINTING
#endif
UI_MENU_SUBMENU_FILTER_T(ui_menu_move, UI_TEXT_POSITION_ID, ui_menu_positions,0,MENU_MODE_PRINTING)
#if NUM_EXTRUDER > 1
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_preheatcool2,&ui_removebed UI_CHANGE_FIL_ENT ,&ui_menu_autolevelbed UI_CALIB_PROBE_ENTRY, &ui_calex ,&ui_menu_move \
      , &ui_menu_quick_stopstepper UI_FANSPEED, &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_bed_temp}
#else
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_preheatcool1,&ui_removebed UI_CHANGE_FIL_ENT ,&ui_menu_autolevelbed UI_CALIB_PROBE_ENTRY, &ui_calex ,&ui_menu_move \
      , &ui_menu_quick_stopstepper UI_FANSPEED, &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_bed_temp}
#endif      
UI_MENU(ui_menu_quick, UI_MENU_QUICK, 10 + UI_MENU_BACKCNT + UI_CHANGE_FIL_CNT+ UI_CALIB_PROBE_COUNT)

UI_MENU_HEADLINE_T(ui_menu_askstop_head, UI_TEXT_STOP_PRINT_ID)
UI_MENU_ACTIONCOMMAND_T(ui_menu_sd_askstop_no, UI_TEXT_NO_ID, UI_ACTION_BACK)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_askstop_yes,      UI_TEXT_YES_ID,     UI_ACTION_STOP_CONFIRMED | UI_ACTION_TOPMENU, MENU_MODE_PRINTING, 0)
#if UI_ROWS >= 5
#define UI_MENU_ASKSTOP {&ui_menu_empty,&ui_menu_askstop_head,&ui_menu_empty,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 5)
#elif UI_ROWS == 4
#define UI_MENU_ASKSTOP {&ui_menu_askstop_head,&ui_menu_empty,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 4)
#else
#define UI_MENU_ASKSTOP {&ui_menu_askstop_head,&ui_menu_sd_askstop_no,&ui_menu_sd_askstop_yes}
UI_MENU(ui_menu_askstop, UI_MENU_ASKSTOP, 3)
#endif

// **** SD card menu

#if SDSUPPORT

// Dummy print menu to show missing card
UI_MENU_HEADLINE(ui_menus_sd_pinsert, "Please insert SD card")
#define UI_MENU_SD_NOCARD {UI_MENU_ADDCONDBACK &ui_menus_sd_pinsert}
UI_MENU(ui_menu_sd_nocard, UI_MENU_SD_NOCARD, UI_MENU_BACKCNT + 1)
UI_MENU_SUBMENU_FILTER_T(ui_menu_sd_printfile2, UI_TEXT_PRINT_FILE_ID,     ui_menu_sd_nocard,    0, MENU_MODE_SD_MOUNTED + MENU_MODE_SD_PRINTING)

#define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
UI_MENU_FILESELECT(ui_menu_sd_fileselector, UI_MENU_SD_FILESELECTOR, 1)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_printfile, UI_TEXT_PRINT_FILE_ID,     UI_ACTION_SD_PRINT,    MENU_MODE_SD_MOUNTED,  MENU_MODE_SD_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_pause,     UI_TEXT_PAUSE_PRINT_ID,    UI_ACTION_SD_PAUSE,    MENU_MODE_SD_PRINTING, MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_continue,  UI_TEXT_CONTINUE_PRINT_ID, UI_ACTION_SD_CONTINUE, MENU_MODE_SD_PRINTING+MENU_MODE_PAUSED,   0)
// two versions of stop. Second is with security question since pausing can trigger stop with bad luck!
//UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_stop,      UI_TEXT_STOP_PRINT_ID,     UI_ACTION_SD_STOP,     MENU_MODE_SD_PRINTING, 0)
UI_MENU_SUBMENU_FILTER_T(ui_menu_sd_stop, UI_TEXT_STOP_PRINT_ID, ui_menu_askstop, MENU_MODE_SD_PRINTING, 0 )
#define SD_PRINTFILE_ENTRY &ui_menu_sd_printfile,&ui_menu_sd_printfile2,
#define SD_PRINTFILE_ENTRY_CNT 2
#if SDCARDDETECT > -1
#define UI_MOUNT_CNT 0
#define UI_MOUNT_CMD
#else
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_unmount, UI_TEXT_UNMOUNT_CARD_ID, UI_ACTION_SD_UNMOUNT, MENU_MODE_SD_MOUNTED, 0)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_mount, UI_TEXT_MOUNT_CARD_ID, UI_ACTION_SD_MOUNT, 0, MENU_MODE_SD_MOUNTED)
#define UI_MOUNT_CNT 2
#define UI_MOUNT_CMD ,&ui_menu_sd_unmount,&ui_menu_sd_mount
#endif
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_menu_sd_delete, UI_TEXT_DELETE_FILE_ID, UI_ACTION_SD_DELETE, MENU_MODE_SD_MOUNTED, MENU_MODE_SD_PRINTING)
#define UI_MENU_SD {UI_MENU_ADDCONDBACK &ui_menu_sd_printfile,&ui_menu_sd_pause,&ui_menu_sd_continue,&ui_menu_sd_stop UI_MOUNT_CMD ,&ui_menu_sd_delete}
UI_MENU(ui_menu_sd, UI_MENU_SD, UI_MENU_BACKCNT + 5 + UI_MOUNT_CNT)
UI_MENU_SUBMENU_T(ui_menu_sd_sub, UI_TEXT_SD_CARD_ID, ui_menu_sd)

#define UI_MENU_SD_COND &ui_menu_sd_sub,
#define UI_MENU_SD_CNT 1
#else
#define UI_MENU_SD_COND
#define UI_MENU_SD_CNT 0
#define SD_PRINTFILE_ENTRY
#define SD_PRINTFILE_ENTRY_CNT 0
#endif


// **** Debugging menu
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_echo,    UI_TEXT_DBG_ECHO_ID,    UI_ACTION_DEBUG_ECHO)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_info,    UI_TEXT_DBG_INFO_ID,    UI_ACTION_DEBUG_INFO)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_error,   UI_TEXT_DBG_ERROR_ID,   UI_ACTION_DEBUG_ERROR)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_dryrun,  UI_TEXT_DBG_DRYRUN_ID,  UI_ACTION_DEBUG_DRYRUN)
UI_MENU_ACTIONCOMMAND_T(ui_menu_debug_endstop, UI_TEXT_DBG_ENDSTOP_ID, UI_ACTION_DEBUG_ENDSTOP)

#define UI_MENU_DEBUGGING {UI_MENU_ADDCONDBACK &ui_menu_debug_echo,&ui_menu_debug_info,&ui_menu_debug_error,&ui_menu_debug_dryrun,&ui_menu_debug_endstop}
UI_MENU(ui_menu_debugging, UI_MENU_DEBUGGING, 5 + UI_MENU_BACKCNT)

// **** Acceleration settings
#if DRIVE_SYSTEM != DELTA
UI_MENU_CHANGEACTION_T(ui_menu_accel_printx,  UI_TEXT_PRINT_X_ID, UI_ACTION_PRINT_ACCEL_X)
UI_MENU_CHANGEACTION_T(ui_menu_accel_printy,  UI_TEXT_PRINT_Y_ID, UI_ACTION_PRINT_ACCEL_Y)
UI_MENU_CHANGEACTION_T(ui_menu_accel_printz,  UI_TEXT_PRINT_Z_ID, UI_ACTION_PRINT_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelx, UI_TEXT_MOVE_X_ID, UI_ACTION_MOVE_ACCEL_X)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travely, UI_TEXT_MOVE_Y_ID, UI_ACTION_MOVE_ACCEL_Y)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelz, UI_TEXT_MOVE_Z_ID, UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_jerk,    UI_TEXT_JERK_ID, UI_ACTION_MAX_JERK)
UI_MENU_CHANGEACTION_T(ui_menu_accel_zjerk,   UI_TEXT_ZJERK_ID, UI_ACTION_MAX_ZJERK)
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printx,&ui_menu_accel_printy,&ui_menu_accel_printz,&ui_menu_accel_travelx,&ui_menu_accel_travely,&ui_menu_accel_travelz,&ui_menu_accel_jerk,&ui_menu_accel_zjerk}
UI_MENU(ui_menu_accel, UI_MENU_ACCEL, 8 + UI_MENU_BACKCNT)

// **** Feedrates
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxx,  UI_TEXT_FEED_MAX_X_ID,  UI_ACTION_MAX_FEEDRATE_X)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxy,  UI_TEXT_FEED_MAX_Y_ID,  UI_ACTION_MAX_FEEDRATE_Y)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxz,  UI_TEXT_FEED_MAX_Z_ID,  UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homex, UI_TEXT_FEED_HOME_X_ID, UI_ACTION_HOMING_FEEDRATE_X)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homey, UI_TEXT_FEED_HOME_Y_ID, UI_ACTION_HOMING_FEEDRATE_Y)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z_ID, UI_ACTION_HOMING_FEEDRATE_Z)
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxx,&ui_menu_feedrate_maxy,&ui_menu_feedrate_maxz,&ui_menu_feedrate_homex,&ui_menu_feedrate_homey,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate, UI_MENU_FEEDRATE, 6 + UI_MENU_BACKCNT)
#else
UI_MENU_CHANGEACTION_T(ui_menu_accel_printz, UI_TEXT_PRINT_Z_DELTA_ID, UI_ACTION_PRINT_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_travelz, UI_TEXT_MOVE_Z_DELTA_ID, UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION_T(ui_menu_accel_jerk, UI_TEXT_JERK_ID, UI_ACTION_MAX_JERK)
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printz,&ui_menu_accel_travelz,&ui_menu_accel_jerk}
UI_MENU(ui_menu_accel, UI_MENU_ACCEL, 3 + UI_MENU_BACKCNT)

// **** Feedrates
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_maxz, UI_TEXT_FEED_MAX_Z_DELTA_ID, UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION_T(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z_DELTA_ID, UI_ACTION_HOMING_FEEDRATE_Z)
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_feedrate_maxz,&ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate, UI_MENU_FEEDRATE, 2 + UI_MENU_BACKCNT)
#endif

// ***** Info

UI_MENU_HEADLINE_T(ui_info1, UI_CTEXT_TOT_TIME_ID)
UI_MENU_HEADLINE_T(ui_info2, UI_CTEXT_TOT_FILAMENT_ID)
#define UI_MENU_INFO {&ui_info1,&ui_info2,&ui_menu_back}
UI_MENU(ui_menu_info, UI_MENU_INFO, 3)
UI_MENU_SUBMENU_T(ui_menu_sub_info, UI_CTEXT_INFO_ID, ui_menu_info)

// **** Extruder configuration

UI_MENU_CHANGEACTION_T(ui_menu_cext_steps,          UI_TEXT_EXTR_STEPS_ID,              UI_ACTION_EXTR_STEPS)
UI_MENU_CHANGEACTION_T(ui_menu_cext_start_feedrate, UI_TEXT_EXTR_START_FEED_ID,         UI_ACTION_EXTR_START_FEEDRATE)
UI_MENU_CHANGEACTION_T(ui_menu_cext_max_feedrate,   UI_TEXT_EXTR_MAX_FEED_ID,           UI_ACTION_EXTR_MAX_FEEDRATE)
UI_MENU_CHANGEACTION_T(ui_menu_cext_acceleration,   UI_TEXT_EXTR_ACCEL_ID,              UI_ACTION_EXTR_ACCELERATION)
UI_MENU_CHANGEACTION_T(ui_menu_cext_watch_period,   UI_TEXT_EXTR_WATCH_ID,              UI_ACTION_EXTR_WATCH_PERIOD)
UI_MENU_CHANGEACTION_T(ui_menu_ext_wait_temp,       UI_TEXT_EXTR_WAIT_RETRACT_TEMP_ID,  UI_ACTION_EXTR_WAIT_RETRACT_TEMP)
UI_MENU_CHANGEACTION_T(ui_menu_ext_wait_units,      UI_TEXT_EXTR_WAIT_RETRACT_UNITS_ID, UI_ACTION_EXTR_WAIT_RETRACT_UNITS)
#define UI_MENU_ADV_CNT 0
#define UI_MENU_ADVANCE
#if USE_ADVANCE
#undef UI_MENU_ADV_CNT
#define UI_MENU_ADV_CNT 1
#undef UI_MENU_ADVANCE
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel
#if ENABLE_QUADRATIC_ADVANCE
#undef UI_MENU_ADV_CNT
#define UI_MENU_ADV_CNT 2
#undef UI_MENU_ADVANCE
#define UI_MENU_ADVANCE ,&ui_menu_cext_advancel,&ui_menu_cext_advancek
UI_MENU_CHANGEACTION_T(ui_menu_cext_advancek, UI_TEXT_EXTR_ADVANCE_K_ID, UI_ACTION_ADVANCE_K)
#endif
UI_MENU_CHANGEACTION_T(ui_menu_cext_advancel, UI_TEXT_EXTR_ADVANCE_L_ID, UI_ACTION_ADVANCE_L)
#endif
UI_MENU_CHANGEACTION_T(       ui_menu_cext_manager, UI_TEXT_EXTR_MANAGER_ID, UI_ACTION_EXTR_HEATMANAGER)
UI_MENU_CHANGEACTION_T(       ui_menu_cext_pmax,    UI_TEXT_EXTR_PMAX_ID,    UI_ACTION_PID_MAX)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_pgain,   UI_TEXT_EXTR_PGAIN_ID,   UI_ACTION_PID_PGAIN, MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_igain,   UI_TEXT_EXTR_IGAIN_ID,   UI_ACTION_PID_IGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dgain,   UI_TEXT_EXTR_DGAIN_ID,   UI_ACTION_PID_DGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmin,    UI_TEXT_EXTR_DMIN_ID,    UI_ACTION_DRIVE_MIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmax,    UI_TEXT_EXTR_DMAX_ID,    UI_ACTION_DRIVE_MAX,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_pgain_dt,   UI_TEXT_EXTR_DEADTIME_ID,   UI_ACTION_PID_PGAIN, MENU_MODE_DEADTIME, 0)
UI_MENU_CHANGEACTION_FILTER_T(ui_menu_cext_dmax_dt,    UI_TEXT_EXTR_DMAX_DT_ID,    UI_ACTION_DRIVE_MAX,  MENU_MODE_DEADTIME, 0)
#define UI_MENU_PIDCOND ,&ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax, &ui_menu_cext_pgain_dt,&ui_menu_cext_dmax_dt,&ui_menu_cext_pmax
#define UI_MENU_PIDCNT 9
#if NUM_EXTRUDER > 5 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_ext_sel5,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 8
#elif NUM_EXTRUDER > 4 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_ext_sel4,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 7
#elif NUM_EXTRUDER > 3 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_ext_sel3,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 6
#elif NUM_EXTRUDER > 2 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_ext_sel2,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 5
#elif NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0
UI_MENU_CHANGEACTION_T(ui_menu_cext_xoffset, UI_TEXT_EXTR_XOFF_ID, UI_ACTION_X_OFFSET)
UI_MENU_CHANGEACTION_T(ui_menu_cext_yoffset, UI_TEXT_EXTR_YOFF_ID, UI_ACTION_Y_OFFSET)
#define UI_MENU_CONFEXTCOND &ui_menu_ext_sel0,&ui_menu_ext_sel1,&ui_menu_cext_xoffset,&ui_menu_cext_yoffset,
#define UI_MENU_CONFEXTCNT 4
#else
#define UI_MENU_CONFEXTCOND
#define UI_MENU_CONFEXTCNT 0
#endif
#define UI_MENU_CEXTR {UI_MENU_ADDCONDBACK UI_MENU_CONFEXTCOND &ui_menu_cext_steps,&ui_menu_cext_start_feedrate,&ui_menu_cext_max_feedrate,&ui_menu_cext_acceleration,&ui_menu_cext_watch_period,&ui_menu_ext_wait_units,&ui_menu_ext_wait_temp UI_MENU_ADVANCE UI_MENU_PIDCOND}
UI_MENU(ui_menu_cextr, UI_MENU_CEXTR, 7 + UI_MENU_BACKCNT + UI_MENU_PIDCNT + UI_MENU_CONFEXTCNT + UI_MENU_ADV_CNT)

// HeatBed Configuration - use menu actions from extruder configuration
#if HAVE_HEATED_BED
#define UI_MENU_BEDCONF {UI_MENU_ADDCONDBACK UI_MENU_COATING_COND &ui_menu_cext_manager,&ui_menu_cext_pgain,&ui_menu_cext_igain,&ui_menu_cext_dgain,&ui_menu_cext_dmin,&ui_menu_cext_dmax,&ui_menu_cext_pgain_dt,&ui_menu_cext_pmax}
UI_MENU(ui_menu_bedconf, UI_MENU_BEDCONF, 8 + UI_MENU_BACKCNT + UI_MENU_COATING_CNT)
#endif



// **** Configuration menu

UI_MENU_SUBMENU_T(ui_menu_conf_general, UI_TEXT_GENERAL_ID,      ui_menu_general)
UI_MENU_SUBMENU_T(ui_menu_conf_accel,   UI_TEXT_ACCELERATION_ID, ui_menu_accel)
UI_MENU_SUBMENU_T(ui_menu_conf_feed,    UI_TEXT_FEEDRATE_ID,     ui_menu_feedrate)
UI_MENU_SUBMENU_T(ui_menu_conf_extr,    UI_TEXT_EXTRUDER_ID,     ui_menu_cextr)
#if HAVE_HEATED_BED
UI_MENU_SUBMENU_T(ui_menu_conf_bed,    UI_TEXT_HEATING_BED_ID,  ui_menu_bedconf)
#define UI_MENU_BEDCONF_COND ,&ui_menu_conf_bed
#define UI_MENU_BEDCONF_CNT 1
#else
#define UI_MENU_BEDCONF_COND
#define UI_MENU_BEDCONF_CNT 0
#endif
#if EEPROM_MODE!=0
UI_MENU_ACTIONCOMMAND_T(ui_menu_conf_to_eeprom, UI_TEXT_STORE_TO_EEPROM_ID, UI_ACTION_STORE_EEPROM)
UI_MENU_ACTIONCOMMAND_T(ui_menu_conf_from_eeprom, UI_TEXT_LOAD_EEPROM_ID, UI_ACTION_LOAD_EEPROM)
UI_MENU_ACTIONCOMMAND(ui_menu_conf_reset_eeeprom, "Reset EEPROM", UI_ACTION_RESET_EEPROM)
#define UI_MENU_EEPROM_COND ,&ui_menu_conf_to_eeprom,&ui_menu_conf_from_eeprom, &ui_menu_conf_reset_eeeprom
#define UI_MENU_EEPROM_CNT 3
UI_MENU_ACTION2_T(ui_menu_eeprom_saved,  UI_ACTION_DUMMY, UI_TEXT_EEPROM_STOREDA_ID, UI_TEXT_EEPROM_STOREDB_ID)
UI_MENU_ACTION2_T(ui_menu_eeprom_loaded, UI_ACTION_DUMMY, UI_TEXT_EEPROM_LOADEDA_ID, UI_TEXT_EEPROM_LOADEDB_ID)
UI_MENU_ACTION2_T(ui_menu_eeprom_reset, UI_ACTION_DUMMY, UI_TEXT_EEPROM_RESETEDA_ID, UI_TEXT_EEPROM_RESETEDB_ID)
#else
#define UI_MENU_EEPROM_COND
#define UI_MENU_EEPROM_CNT 0
#endif
#if defined(SOFTWARE_LEVELING) && DRIVE_SYSTEM == DELTA
#define UI_MENU_SL_COND ,&ui_menu_conf_level
#define UI_MENU_SL_CNT 1
UI_MENU_SUBMENU_T(ui_menu_conf_level, UI_TEXT_LEVEL_ID, ui_menu_level)
#else
#define UI_MENU_SL_COND
#define UI_MENU_SL_CNT 0
#endif
#if Z_HOME_DIR > 0
#define UI_MENU_DELTA_COND ,&ui_menu_conf_delta
#define UI_MENU_DELTA_CNT 1
UI_MENU_SUBMENU_T(ui_menu_conf_delta, UI_TEXT_ZCALIB_ID, ui_menu_delta)
#else
#define UI_MENU_DELTA_COND
#define UI_MENU_DELTA_CNT 0
#endif
#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK &ui_menu_sub_info, LANGMENU_ENTRY &ui_menu_conf_extr UI_MENU_BEDCONF_COND ,&ui_menu_general_baud  UI_MENU_EEPROM_COND UI_MENU_DELTA_COND UI_MENU_SL_COND}
UI_MENU(ui_menu_configuration, UI_MENU_CONFIGURATION, UI_MENU_BACKCNT + LANGMENU_COUNT + UI_MENU_EEPROM_CNT + UI_MENU_BEDCONF_CNT + UI_MENU_DELTA_CNT + UI_MENU_SL_CNT + 3)


// **** Preheat menu ****

#if HAVE_HEATED_BED + NUM_EXTRUDER > 0
UI_MENU_HEADLINE_T(ui_menu_preheat_hdl,UI_TEXT_PREHEAT_TEMPS_ID)
#if HAVE_HEATED_BED
UI_MENU_CHANGEACTION_T(ui_menu_preheat_bed, UI_TEXT_PREHEAT_BED_ID, UI_ACTION_BED_PREHEAT)
#define UI_MENU_PREHEAT_BED ,&ui_menu_preheat_bed
#endif
#if NUM_EXTRUDER > 0
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext0, UI_TEXT_PREHEAT_E0_ID, UI_ACTION_EXT0_PREHEAT)
#define UI_MENU_PREHEAT_EXT0 ,&ui_menu_preheat_ext0
#else
#define UI_MENU_PREHEAT_EXT0
#endif
#if NUM_EXTRUDER > 1
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext1, UI_TEXT_PREHEAT_E1_ID, UI_ACTION_EXT1_PREHEAT)
#define UI_MENU_PREHEAT_EXT1 ,&ui_menu_preheat_ext1
#else
#define UI_MENU_PREHEAT_EXT1
#endif
#if NUM_EXTRUDER > 2
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext2, UI_TEXT_PREHEAT_E2_ID, UI_ACTION_EXT2_PREHEAT)
#define UI_MENU_PREHEAT_EXT2 ,&ui_menu_preheat_ext2
#else
#define UI_MENU_PREHEAT_EXT2
#endif
#if NUM_EXTRUDER > 3
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext3, UI_TEXT_PREHEAT_E3_ID, UI_ACTION_EXT3_PREHEAT)
#define UI_MENU_PREHEAT_EXT3 ,&ui_menu_preheat_ext3
#else
#define UI_MENU_PREHEAT_EXT3
#endif
#if NUM_EXTRUDER > 4
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext4, UI_TEXT_PREHEAT_E4_ID, UI_ACTION_EXT4_PREHEAT)
#define UI_MENU_PREHEAT_EXT4 ,&ui_menu_preheat_ext4
#else
#define UI_MENU_PREHEAT_EXT4
#endif
#if NUM_EXTRUDER > 5
UI_MENU_CHANGEACTION_T(ui_menu_preheat_ext5, UI_TEXT_PREHEAT_E5_ID, UI_ACTION_EXT5_PREHEAT)
#define UI_MENU_PREHEAT_EXT5 ,&ui_menu_preheat_ext5
#else
#define UI_MENU_PREHEAT_EXT5
#endif

UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pla_a,"PLA",UI_ACTION_SPH_PLA_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_petg_a,"PETG",UI_ACTION_SPH_PETG_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_pva_a,"PVA",UI_ACTION_SPH_PVA_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_flex_a,"FLEX",UI_ACTION_SPH_FLEX_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_abs_a,"ABS",UI_ACTION_SPH_ABS_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_glass_a,"GLASS",UI_ACTION_SPH_GLASS_ALL)
UI_MENU_ACTIONCOMMAND(ui_menu_chf_sph_wood_a,"WOOD",UI_ACTION_SPH_WOOD_ALL)

#define UI_MENU_PREHEAT_SUB {&ui_menu_preheat_hdl UI_MENU_ADDCONDBACK_C UI_MENU_PREHEAT_BED UI_MENU_PREHEAT_EXT0 UI_MENU_PREHEAT_EXT1 UI_MENU_PREHEAT_EXT2 UI_MENU_PREHEAT_EXT3 UI_MENU_PREHEAT_EXT4 UI_MENU_PREHEAT_EXT5\
,&ui_menu_chf_sph_pla_a,&ui_menu_chf_sph_petg_a,&ui_menu_chf_sph_pva_a,&ui_menu_chf_sph_flex_a,&ui_menu_chf_sph_abs_a,&ui_menu_chf_sph_glass_a,&ui_menu_chf_sph_wood_a}
UI_MENU(ui_menu_preheat_sub, UI_MENU_PREHEAT_SUB, UI_MENU_BACKCNT + 8 + HAVE_HEATED_BED + NUM_EXTRUDER)
UI_MENU_SUBMENU_T(ui_menu_preheat,UI_TEXT_PREHEAT_TEMPS_ID,ui_menu_preheat_sub)

#define UI_MENU_PREHEAT ,&ui_menu_preheat
#define UI_MENU_PREHEAT_CNT 1
#else
#define UI_MENU_PREHEAT 
#define UI_MENU_PREHEAT_CNT 0
#endif

// Preheat info screen

UI_MENU_HEADLINE_T(ui_msh_ph1, UI_TEXT_PREHEAT_TEMPS_ID)
UI_MENU_HEADLINE_T(ui_msh_ph2, UI_TEXT_PREHEAT_BED_ID)
UI_MENU_HEADLINE_T(ui_msh_ph3, UI_TEXT_PREHEAT_E0_ID)
UI_MENU_HEADLINE_T(ui_msh_ph4, UI_TEXT_PREHEAT_E1_ID)
UI_MENU_ACTIONCOMMAND_T(ui_msg_ph_ok, UI_TEXT_OK_ID, UI_ACTION_MESSAGE)
#define UI_MENU_PREHEAT_SET {&ui_msh_ph1,&ui_msh_ph2,&ui_msh_ph3,&ui_msh_ph4,&ui_msg_ph_ok}
UI_STICKYMENU(ui_menu_preheatinfo,UI_MENU_PREHEAT_SET,5)

// **** Setup Menu

/*
Debug
Z Calibrate
Autlevel on/off
Distortion map
Distortion on/off
Ignore M106 Cmd
*/
UI_MENU_SUBMENU_T(ui_debug, UI_TEXT_DEBUGGING_ID, ui_menu_debugging)

#if DISTORTION_CORRECTION
UI_MENU_ACTIONCOMMAND_T(ui_menu_measure_distortion,UI_TEXT_MEASURE_DISTORTION_ID, UI_ACTION_MEASURE_DISTORTION)
UI_MENU_ACTIONCOMMAND_T(ui_menu_toggle_distortion,UI_TEXT_DISTORTION_CORR_ID, UI_ACTION_TOGGLE_DISTORTION)
#define UI_DISTORTION_ENTRY ,&ui_menu_measure_distortion,&ui_menu_toggle_distortion
#define UI_DISTORTION_COUNT 2
#else
#define UI_DISTORTION_ENTRY
#define UI_DISTORTION_COUNT 0
#endif

#define UI_MENU_SETUP {UI_MENU_ADDCONDBACK &ui_debug UI_MENU_PREHEAT UI_TOOGLE_AUTOLEVEL_ENTRY UI_CALIB_PROBE_ENTRY UI_DISTORTION_ENTRY UI_CAL_ZHEIGHT_ENTRY ,&ui_calex ,&ui_menu_fan_ignoreM106}
UI_MENU(ui_menu_setup, UI_MENU_SETUP, UI_MENU_BACKCNT + 3 + UI_MENU_PREHEAT_CNT + UI_TOGGLE_AUTOLEVEL_COUNT + UI_DISTORTION_COUNT + UI_CAL_ZHEIGHT_CNT + UI_CALIB_PROBE_COUNT)
UI_MENU_SUBMENU_FILTER_T(ui_setup, UI_TEXT_SETUP_ID, ui_menu_setup,0,MENU_MODE_PRINTING)

// Stop print security question


// Main menu

// stop/pause/continue entries

UI_MENU_ACTIONCOMMAND_FILTER_T(ui_pause,UI_TEXT_PAUSE_PRINT_ID,UI_ACTION_PAUSE,MENU_MODE_PRINTING,MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_continue,UI_TEXT_CONTINUE_PRINT_ID,UI_ACTION_CONTINUE,MENU_MODE_PAUSED,0)
UI_MENU_ACTIONCOMMAND_FILTER_T(ui_stop,UI_TEXT_STOP_PRINT_ID,UI_ACTION_STOP,MENU_MODE_PRINTING,MENU_MODE_PAUSED)


UI_MENU_SUBMENU_FILTER_T(ui_menu_control, UI_TEXT_QUICK_SETTINGS_ID, ui_menu_quick,0,MENU_MODE_PRINTING)
UI_MENU_SUBMENU_FILTER_T(ui_menu_extrudercontrol, UI_TEXT_EXTRUDER_ID, ui_menu_extruder,0,MENU_MODE_PRINTING)

UI_MENU_SUBMENU_FILTER_T(ui_menu_settings, UI_TEXT_CONFIGURATION_ID, ui_menu_configuration,0,MENU_MODE_PRINTING)
#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK &ui_menu_control ,&ui_stop,&ui_pause,&ui_continue UI_CHANGE_FIL_ENT_PRINTING\
    BABY_ENTRY_PRINTING  ,&ui_menu_quick_speedmultiply_printing,&ui_menu_quick_flowmultiply_printing UI_TEMP0_PRINTING UI_TEMP1_PRINTING UI_TEMP2_PRINTING UI_TEMP3_PRINTING UI_TEMP4_PRINTING UI_TEMP5_PRINTING \
    UI_BED_TEMP_PRINTING  UI_FANSPEED_PRINTING UI_FAN2SPEED_PRINTING  , SD_PRINTFILE_ENTRY \
    &ui_menu_settings}
   // &ui_menu_move, &ui_menu_extrudercontrol, 
    
UI_MENU(ui_menu_main, UI_MENU_MAIN, 7 + UI_MENU_BACKCNT + SD_PRINTFILE_ENTRY_CNT +UI_TEMP0_CNT+UI_TEMP1_CNT+UI_TEMP2_CNT+UI_TEMP3_CNT+\
    UI_TEMP4_CNT+UI_TEMP5_CNT+ BABY_CNT+HAVE_HEATED_BED+UI_MENU_FAN_CNT+UI_MENU_FAN2_CNT + UI_CHANGE_FIL_CNT)

/* Define menus accessible by action commands

  You can create up to 10 user menus which are accessible by the action commands UI_ACTION_SHOW_USERMENU1 until UI_ACTION_SHOW_USERMENU10
  You this the same way as with the menus above or you use one of the above menus. Then add a define like

  #define UI_USERMENU1 ui_menu_conf_feed

  which assigns the menu stored in ui_menu_conf_feed to the action UI_ACTION_SHOW_USERMENU1. Make sure only to change the numbers and not the name of the define.

  When do you need this? You might want a fast button to change the temperature. In the default menu you have no menu
  to change the temperature and view it the same time. So you need to make an action menu for this like:
  UI_MENU_ACTION4C(ui_menu_extrtemp,UI_ACTION_EXTRUDER0_TEMP,"Temp. 0  :%E0" cDEG,"","","");
  Then you assign this menu to a user menu:
  #define UI_USERMENU2 ui_menu_extrtemp

  Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperture screen and allows
  the change of temperature with the next/previous buttons.

*/
#endif
#endif // __UI_MENU_H

    break;
  case UI_ACTION_FC_GLASS:
    setPreheatTemps(225, 85, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_WOOD:
    setPreheatTemps(185, 55, false, false);
    preheatFCActive();
    break;
  case UI_ACTION_FC_CUSTOM:
    Printer::wizardStack[0].l = 190;
    uid.popMenu(false);
    uid.pushMenu(&ui_menu_ch2a, true);
    break;
  case UI_ACTION_FC_WAITHEAT:
    changeFilWaitTarget = false;
    Extruder::setTemperatureForExtruder(0, Extruder::current->id, false);
    uid.popMenu(true);
    break;
  case UI_ACTION_FC_BACK1:
    uid.popMenu(false);
    uid.pushMenu(&ui_menu_chf, true);
    break;
  case UI_ACTION_FC_BACK2:
    uid.popMenu(false);
    uid.pushMenu(&ui_menu_ch2, true);
    break;
#ifdef ZPROBE_HEIGHT_ROUTINE
  case UI_ACTION_START_CZREFH:
    cZPHeight1();
    break;
#endif
  case UI_ACTION_EXTRXY_V2:
    uid.pushMenu(&cui_msg_preparing, true);
    if (!Printer::isHomedAll()) {
      Printer::homeAxis(true, true, true);
    }
    Extruder::selectExtruderById(0);
    Printer::moveToReal(0, 240, 40, IGNORE_COORDINATE, 100);
    uid.popMenu(false);
    uid.pushMenu(&cui_msg_ext_xy_1, true);
    break;
  }
}

void cNextPrevious(int action, bool allowMoves, int increment) {
  switch (action) {
  case UI_ACTION_EXY_XOFFSET:
    Printer::wizardStack[0].l += increment;
    if (Printer::wizardStack[0].l < 1)
      Printer::wizardStack[0].l = 1;
    if (Printer::wizardStack[0].l > 9)
      Printer::wizardStack[0].l = 9;
    break;
  case UI_ACTION_EXY_YOFFSET:
    Printer::wizardStack[1].l += increment;
    if (Printer::wizardStack[1].l < 1)
      Printer::wizardStack[1].l = 1;
    if (Printer::wizardStack[1].l > 9)
      Printer::wizardStack[1].l = 9;
    break;
  case UI_ACTION_FC_CUSTOM_SET:
    Printer::wizardStack[0].l += increment;
    if (Printer::wizardStack[0].l < 150)
      Printer::wizardStack[0].l = 150;
    if (Printer::wizardStack[0].l > 275)
      Printer::wizardStack[0].l = 275;
    break;
  case UI_ACTION_CZREFH: {
    bool old = Printer::isNoDestinationCheck();
    Printer::setNoDestinationCheck(true);
    PrintLine::moveRelativeDistanceInStepsReal(
        0, 0, ((long)increment * Printer::axisStepsPerMM[Z_AXIS]) / 100, 0,
        Printer::homingFeedrate[Z_AXIS], false, false);
    Printer::setNoDestinationCheck(old);
  } break;
  }
}

bool measureXEdge(float x, float y, float width, float treshhold,
                  float &result) {
  float xleft = x; // - width;
  float xright = x + width;
  width *= 0.5;
  float zleft, zright, zcenter, zopt;
  Printer::moveToReal(xleft, y, 2, IGNORE_COORDINATE, 100);
  zleft = Printer::runZProbe(true, true, 1, true, false);
  if (zleft == ILLEGAL_Z_PROBE) {
    return false;
  }
  Printer::moveToReal(xright, y, 2, IGNORE_COORDINATE, 100);
  zright = Printer::runZProbe(true, true, 1, true, false);
  if (zright == ILLEGAL_Z_PROBE) {
    return false;
  }
  if (fabs(zright - zleft) < treshhold) {
    Com::printFLN(PSTR("No card detected, aborting."));
    return false;
  }
  zopt = 0.5 * (zleft + zright);
  bool leftSide = zleft > zright;
  do {
    Printer::moveToReal(xleft + width, y, 2, IGNORE_COORDINATE, 40);
    zcenter = Printer::runZProbe(true, true, 1, true, false);
    if (zcenter == ILLEGAL_Z_PROBE) {
      return false;
    }
    if (leftSide) {
      if (zcenter < zopt) {
        xright = xleft + width;
      } else {
        xleft += width;
      }
    } else {
      if (zcenter > zopt) {
        xright = xleft + width;
      } else {
        xleft += width;
      }
    }
    width *= 0.5;
  } while (width > 0.005);
  result = xleft + width;
  return true;
}

bool measureYEdge(float x, float y, float width, float treshhold,
                  float &result) {
  float xleft = y; // - width;
  float xright = y + width;
  width *= 0.5;
  float zleft, zright, zcenter, zopt;
  Printer::moveToReal(x, xleft, 2, IGNORE_COORDINATE, 100);
  zleft = Printer::runZProbe(true, true, 1, true, false);
  if (zleft == ILLEGAL_Z_PROBE) {
    return false;
  }
  Printer::moveToReal(x, xright, 2, IGNORE_COORDINATE, 100);
  zright = Printer::runZProbe(true, true, 1, true, false);
  if (zright == ILLEGAL_Z_PROBE) {
    return false;
  }
  if (fabs(zright - zleft) < treshhold) {
    Com::printFLN(PSTR("No card detected, aborting."));
    return false;
  }
  zopt = 0.5 * (zleft + zright);
  bool leftSide = zleft > zright;
  do {
    Printer::moveToReal(x, xleft + width, 2, IGNORE_COORDINATE, 100);
    zcenter = Printer::runZProbe(true, true, 1, true, false);
    if (zcenter == ILLEGAL_Z_PROBE) {
      return false;
    }
    if (leftSide) {
      if (zcenter < zopt) {
        xright = xleft + width;
      } else {
        xleft += width;
      }
    } else {
      if (zcenter > zopt) {
        xright = xleft + width;
      } else {
        xleft += width;
      }
    }
    width *= 0.5;
  } while (width > 0.005);
  result = xleft + width;
  return true;
}

void cOkWizard(int action) {
  switch (action) {
  case UI_ACTION_EXTRXY_V2_1: {
    float x1, x2, x3, x4, y1, y2, y3, y4;
    uid.popMenu(false);
    uid.pushMenu(&cui_msg_ext_xy_info, true);
    extruder[1].xOffset =
        static_cast<int32_t>(Printer::axisStepsPerMM[X_AXIS] * 16);
    extruder[1].yOffset = 0;
    Printer::setBlockingReceive(true);
    if (!measureXEdge(CARD_CENTER_X, CARD_CENTER_Y, CARD_WIDTH, CARD_TRESHHOLD,
                      x1)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 1 X Edge:"), x1);
    if (!measureXEdge(CARD_CENTER_X - CARD_WIDTH, CARD_CENTER_Y, CARD_WIDTH,
                      CARD_TRESHHOLD, x3)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 1 X Edge 2:"), x3);
    Extruder::selectExtruderById(1);
    if (!measureXEdge(CARD_CENTER_X, CARD_CENTER_Y, CARD_WIDTH, CARD_TRESHHOLD,
                      x2)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 2 X Edge:"), x2);
    if (!measureXEdge(CARD_CENTER_X - CARD_WIDTH, CARD_CENTER_Y, CARD_WIDTH,
                      CARD_TRESHHOLD, x4)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 2 X Edge 2:"), x4);
    Extruder::selectExtruderById(0);
    if (!measureYEdge(CARD_CENTER_X, CARD_CENTER_Y, CARD_HEIGHT, CARD_TRESHHOLD,
                      y1)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 1 Y Edge:"), y1);
    if (!measureYEdge(CARD_CENTER_X, CARD_CENTER_Y - CARD_HEIGHT, CARD_HEIGHT,
                      CARD_TRESHHOLD, y3)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 1 Y Edge 2:"), y3);
    Extruder::selectExtruderById(1);
    if (!measureYEdge(CARD_CENTER_X, CARD_CENTER_Y, CARD_HEIGHT, CARD_TRESHHOLD,
                      y2)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 2 Y Edge:"), y2);
    if (!measureYEdge(CARD_CENTER_X, CARD_CENTER_Y - CARD_HEIGHT, CARD_HEIGHT,
                      CARD_TRESHHOLD, y4)) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_ext_xy_error, true);
      Printer::setBlockingReceive(false);
      break;
    }
    Com::printFLN(PSTR("Extruder 2 Y Edge 2:"), y4);
    Extruder::selectExtruderById(0);
    Printer::moveToReal(0, 240, 40, IGNORE_COORDINATE, 100);

    int32_t xcor = static_cast<int32_t>(Printer::axisStepsPerMM[X_AXIS] * -0.5 *
                                        (x2 + x4 - x1 - x3));
    int32_t ycor = static_cast<int32_t>(Printer::axisStepsPerMM[Y_AXIS] * -0.5 *
                                        (y2 + y4 - y1 - y3));
    extruder[1].xOffset += xcor;
    extruder[1].yOffset += ycor;
    Com::printF(PSTR("Calibration result xcorr:"), xcor);
    Com::printFLN(PSTR(" ycorr:"), ycor);
    Com::printF(PSTR("Calibration result xOffset_after:"), extruder[1].xOffset);
    Com::printFLN(PSTR(" yOffset_after:"), extruder[1].yOffset);
    if (xcor != 0 || ycor != 0)
      EEPROM::storeDataIntoEEPROM(false);
    uid.popMenu(false);
    uid.pushMenu(&cui_msg_ext_xy_success, true);
    Printer::setBlockingReceive(false);
  } break;
  case UI_ACTION_CZREFH_SUCC:
    uid.menuLevel = 0;
    break;
#ifndef TEC4
  case UI_ACTION_CALEX_Z2: {
    uid.popMenu(false);
    uid.pushMenu(&ui_msg_extzcalib, true);
    flashSource.executeCommands(extzCalibGCode, true, 0);
    bool err = zDiffCalib();
    flashSource.executeCommands(extzCalibGCode2, false,
                                err ? 0 : UI_ACTION_CALEX_Z3);
    if (err) {
      uid.popMenu(false);
      uid.pushMenu(&cui_msg_exzautolevel_failed, true);
    }
  } break;
#endif
  case UI_ACTION_FC_CUSTOM_SET:
    setPreheatTemps(Printer::wizardStack[0].l, 55, false, false);
    preheatFCActive();
    break;
#ifdef ZPROBE_HEIGHT_ROUTINE
  case UI_ACTION_CZREFH_INFO:
    uid.popMenu(false);
    uid.pushMenu(&cui_calib_zprobe, true);
    break;
  case UI_ACTION_CZREFH:
    cZPHeight2();
    break;
#endif
  }
}

void cRelaxExtruderEndstop() {
#ifndef NO_RELAX_ENDSTOPS
  uint8_t oldJam = Printer::isJamcontrolDisabled();
  Printer::setJamcontrolDisabled(
      true); // prevent jam message when no filament is inserted
  bool nocheck = Printer::isNoDestinationCheck();
  Printer::setNoDestinationCheck(false);
  int activeExtruder = Extruder::current->id;
  Printer::setColdExtrusionAllowed(true);
  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = 0;
  Printer::destinationPositionTransformed[E_AXIS] =
      Printer::currentPositionTransformed[E_AXIS] = 0;
  Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, 0.25,
                  10);
  Extruder::selectExtruderById(1 - activeExtruder);
  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = 0;
  Printer::destinationPositionTransformed[E_AXIS] =
      Printer::currentPositionTransformed[E_AXIS] = 0;
  Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, 0.25,
                  10);
  Printer::setColdExtrusionAllowed(false);
  Extruder::selectExtruderById(activeExtruder);
  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = 0;
  Printer::destinationPositionTransformed[E_AXIS] =
      Printer::currentPositionTransformed[E_AXIS] = 0;
  Printer::setNoDestinationCheck(nocheck);
  Printer::setJamcontrolDisabled(oldJam);
#endif
}

bool cRefreshPage() {
  if (uid.menuLevel != 0)
    return false;
  if (uid.menuPos[0] == 0 && Printer::isPrinting())
    return false;
  // Use big chars, skip 5th line
  uint16_t r;
  uint8_t mtype = UI_MENU_TYPE_INFO;
  char cache[UI_ROWS + UI_ROWS_EXTRA][MAX_COLS + 1];
  uid.adjustMenuPos();
#if defined(UI_HEAD) && UI_DISPLAY_TYPE == DISPLAY_U8G
  char head[MAX_COLS + 1];
  uid.col = 0;
  uid.parse(uiHead, false);
  strcpy(head, uid.printCols);
#endif
  char *text;

  UIMenu *men = (UIMenu *)pgm_read_word(&(ui_pages[uid.menuPos[0]]));
  uint16_t nr = pgm_read_word_near(&(men->numEntries));
  UIMenuEntry **entries = (UIMenuEntry **)pgm_read_word(&(men->entries));
  for (r = 0; r < nr && r < UI_ROWS + UI_ROWS_EXTRA; r++) {
    UIMenuEntry *ent = (UIMenuEntry *)pgm_read_word(&(entries[r]));
    uid.col = 0;
    text = (char *)pgm_read_word(&(ent->text));
    if (text == NULL)
      text = (char *)Com::translatedF(pgm_read_word(&(ent->translation)));
    uid.parse(text, false);
    strcpy(cache[r], uid.printCols);
  }

  uid.printCols[0] = 0;
  while (r < UI_ROWS) // delete trailing empty rows
    strcpy(cache[r++], uid.printCols);
  // compute line scrolling values
  uint8_t off0 = (uid.shift <= 0 ? 0 : uid.shift), y;
  uint8_t off[UI_ROWS + UI_ROWS_EXTRA];
  for (y = 0; y < UI_ROWS + UI_ROWS_EXTRA; y++) {
    uint8_t len = strlen(cache[y]); // length of line content
    off[y] = len > UI_COLS ? RMath::min(len - UI_COLS, off0) : 0;
    if (len > UI_COLS) {
      off[y] = RMath::min(len - UI_COLS, off0);
    } else
      off[y] = 0;
  }
#if UI_DISPLAY_TYPE == DISPLAY_U8G
#define drawHProgressBar(x, y, width, height, progress)                        \
  {                                                                            \
    u8g_DrawFrame(&u8g, x, y, width, height);                                  \
    int p = ceil((width - 2) * progress / 100);                                \
    u8g_DrawBox(&u8g, x + 1, y + 1, p, height - 2);                            \
  }

#define drawVProgressBar(x, y, width, height, progress)                        \
  {                                                                            \
    u8g_DrawFrame(&u8g, x, y, width, height);                                  \
    int p = height - 1 - ceil((height - 2) * progress / 100);                  \
    u8g_DrawBox(&u8g, x + 1, y + p, width - 2, (height - p));                  \
  }

  // u8g picture loop
  u8g_FirstPage(&u8g);
  do {

#endif //  UI_DISPLAY_TYPE == DISPLAY_U8G
#if defined(UI_HEAD) && UI_DISPLAY_TYPE == DISPLAY_U8G
    // Show status line
    u8g_SetColorIndex(&u8g, 1);
    u8g_draw_box(&u8g, 0, 0, u8g_GetWidth(&u8g), UI_FONT_SMALL_HEIGHT + 1);
    u8g_SetColorIndex(&u8g, 0);
#if LANGUAGE_RU_ACTIVE // Switch font
    if (Com::selectedLanguage != LANGUAGE_RU_ID) {
      u8g_SetFont(&u8g, UI_FONT_SMALL);
    } else {
      u8g_SetFont(&u8g, UI_FONT_SMALL_RU);
    }
#else
    u8g_SetFont(&u8g, UI_FONT_SMALL);
#endif
    if (u8g_IsBBXIntersection(&u8g, 0, 1, 1, UI_FONT_SMALL_HEIGHT + 1))
      printU8GRow(1, UI_FONT_SMALL_HEIGHT, head);
#if LANGUAGE_RU_ACTIVE // Switch font
    if (Com::selectedLanguage != LANGUAGE_RU_ID) {
      u8g_SetFont(&u8g, UI_FONT_DEFAULT);
    } else {
      u8g_SetFont(&u8g, UI_FONT_DEFAULT_RU);
    }
#else
    u8g_SetFont(&u8g, UI_FONT_DEFAULT);
#endif
    u8g_SetColorIndex(&u8g, 1);

#endif
    for (y = 0; y < UI_ROWS + UI_ROWS_EXTRA; y++) {
      uint8_t l = y;
      if (y == 4)
        continue;
      if (y == 5)
        l = 4;
      uid.printRow(l, &cache[y][off[y]], NULL, UI_COLS);
    }
#if UI_DISPLAY_TYPE == DISPLAY_U8G
  } while (u8g_NextPage(&u8g)); // end picture loop
#endif
  Printer::toggleAnimation();
  return true;
}

FSTRINGVALUE(removeBedGCode, "M140 S0\n"
                             "G28 Y0\n"
                             "M84\n");

FSTRINGVALUE(extzCalibGCode, "M104 S190 T0\n"
                             "M104 S190 T1\n"
                             "M109 S190 T0\n"
                             "M109 S190 T1\n"
                             "G28\n"
                             "G1 X137 Y45 Z10 F9000\n"
             //"G134 P0 S1\n" //G134 Px Sx Zx - Calibrate nozzle height
             // difference (need z probe in nozzle!) Px = reference extruder, Sx
             //= only measure extrude x against reference, Zx = add to measured
             // z distance for Sx for correction. "M104 S0 T0\n" "M104 S0 T1\n"
             //"M400"
);
FSTRINGVALUE(extzCalibGCode2, "M104 S0 T0\n"
                              "M104 S0 T1\n"
                              "M400");

FSTRINGVALUE(calibrationGCode,
#ifdef TEC4
             "M80 \n"
             "M107\n"
             "M117 Homing\n"
             "G91\n"
             "G1 Z5 F7800\n"
             "G90\n"
             "M140 S0\n"
             "M104 T0 S0\n"
             "M104 T1 S0\n"
             "G28\n"
             "M140 S55\n"
             "M104 T0 S195\n"
             "M104 T1 S195\n"
             "M190 S55 \n"
             "T0\n"
             "M109 T0 S195\n"
             "M117 Purge right printhead\n"
             "G92 E0\n"
             "G1 X1.0 Y0.1 Z0.3 F7800.0\n"
             "G1 X128.5 Y0.1 Z0.3 F1500.0 E15\n"
             "G1 E14.9 F3000\n"
             "G92 E0\n"
             "M104 T0 S140\n"
             "G92 E0\n"
             "G1 X236.0 Y0.8 Z5 F7800.0\n"
             "T1\n"
             "M117 Purge Left printhead\n"
             "M109 S195\n"
             "G1 X236.0 Y0.8 Z0.3 F3000.0\n"
             "G1 X108.5 Y0.8 Z0.3 F1500.0 E15\n"
             "G1 E14.9 F3000\n"
             "G92 E0\n"
             "M106 S64\n"
             "M109 T0 S160\n"
             "M104 T0 S140\n"
             "M107\n"
             "M117 FELIXprinting...\n"
             "G92 E0\n"
             "T1\n"
             "G1 X120 Y0.5 F8000\n"
             "M117 Heating Extruder\n"
             "M109 S195\n"
             "G92 E0  \n"
             "G1 E1.5 F100\n"
             "G92 E0\n"
             "G92 E0\n"
             "G1 X60.36 Y175.8 Z0.55 E0 F9000\n"
             "G1 X60.36 Y175.8 Z0.25 E0 F210\n"
             "G1 E1.25 F900\n"
             "G1 X60.54 Y175.79 E1.2557 F1800\n"
             "G1 X60.64 Y175.75 E1.2593\n"
             "G1 X60.68 Y175.62 E1.2637\n"
             "G1 X60.68 Y146.02 E2.2273\n"
             "G1 X60.69 Y145.91 E2.2308\n"
             "G1 X60.74 Y145.84 E2.2337\n"
             "G1 X60.89 Y145.8 E2.2388\n"
             "G1 X61.49 Y145.8 E2.2582\n"
             "G1 X61.64 Y145.84 E2.2632\n"
             "G1 X61.68 Y146 E2.2688\n"
             "G1 X61.68 Y175.53 E3.2301\n"
             "G1 X61.7 Y175.77 E3.2378\n"
             "G1 X61.86 Y175.8 E3.243\n"
             "G1 X68.04 Y175.8 E3.4441\n"
             "G1 X68.21 Y175.75 E3.4499\n"
             "G1 X68.25 Y175.62 E3.4544\n"
             "G1 X68.25 Y145.99 E4.419\n"
             "G1 X68.29 Y145.85 E4.4237\n"
             "G1 X68.45 Y145.8 E4.429\n"
             "G1 X69.06 Y145.8 E4.449\n"
             "G1 X69.17 Y145.82 E4.4525\n"
             "G1 X69.23 Y145.87 E4.4553\n"
             "G1 X69.25 Y146.04 E4.4608\n"
             "G1 X69.25 Y175.58 E5.4226\n"
             "G1 X69.29 Y175.76 E5.4285\n"
             "G1 X69.44 Y175.8 E5.4336\n"
             "G1 X75.62 Y175.8 E5.6349\n"
             "G1 X75.79 Y175.76 E5.6406\n"
             "G1 X75.83 Y175.61 E5.6458\n"
             "G1 X75.83 Y145.99 E6.6097\n"
             "G1 X75.87 Y145.85 E6.6145\n"
             "G1 X76.03 Y145.8 E6.6202\n"
             "G1 X76.66 Y145.8 E6.6407\n"
             "G1 X76.8 Y145.84 E6.6452\n"
             "G1 X76.82 Y145.99 E6.6503\n"
             "G1 X76.83 Y175.61 E7.6145\n"
             "G1 X76.87 Y175.76 E7.6196\n"
             "G1 X77.04 Y175.8 E7.6254\n"
             "G1 X83.19 Y175.8 E7.8255\n"
             "G1 X83.32 Y175.77 E7.8301\n"
             "G1 X83.36 Y175.74 E7.8317\n"
             "G1 X83.4 Y175.61 E7.8361\n"
             "G1 X83.4 Y145.99 E8.8003\n"
             "G1 X83.43 Y145.84 E8.8053\n"
             "G1 X83.6 Y145.8 E8.811\n"
             "G1 X84.23 Y145.8 E8.8314\n"
             "G1 X84.37 Y145.85 E8.8362\n"
             "G1 X84.4 Y146 E8.8413\n"
             "G1 X84.4 Y175.62 E9.8055\n"
             "G1 X84.43 Y175.77 E9.8106\n"
             "G1 X84.64 Y175.8 E9.8176\n"
             "G1 X90.74 Y175.8 E10.0162\n"
             "G1 X90.85 Y175.79 E10.0196\n"
             "G1 X90.94 Y175.75 E10.0229\n"
             "G1 X90.98 Y175.61 E10.0273\n"
             "G1 X90.98 Y146.01 E10.9909\n"
             "G1 X90.99 Y145.91 E10.9944\n"
             "G1 X91.04 Y145.84 E10.9972\n"
             "G1 X91.19 Y145.8 E11.0023\n"
             "G1 X91.79 Y145.8 E11.0218\n"
             "G1 X91.94 Y145.84 E11.0268\n"
             "G1 X91.98 Y146.02 E11.033\n"
             "G1 X91.98 Y180.62 E12.1591\n"
             "G1 X91.93 Y180.76 E12.1639\n"
             "G1 X91.75 Y180.8 E12.17\n"
             "G1 X30.56 Y180.8 E14.1621\n"
             "G1 X30.47 Y180.79 E14.1652\n"
             "G1 X30.42 Y180.76 E14.1668\n"
             "G1 X30.4 Y180.78 E14.1679\n"
             "G1 X30.38 Y180.46 E14.1782\n"
             "G1 X30.38 Y146.01 E15.2997\n"
             "G1 X30.39 Y145.91 E15.303\n"
             "G1 X30.44 Y145.84 E15.3058\n"
             "G1 X30.57 Y145.8 E15.3103\n"
             "G1 X31.2 Y145.8 E15.3306\n"
             "G1 X31.34 Y145.83 E15.3353\n"
             "G1 X31.38 Y146.01 E15.3413\n"
             "G1 X31.38 Y175.56 E16.3031\n"
             "G1 X31.39 Y175.68 E16.3073\n"
             "G1 X31.43 Y175.77 E16.3105\n"
             "G1 X31.56 Y175.8 E16.315\n"
             "G1 X37.73 Y175.8 E16.5158\n"
             "G1 X37.88 Y175.77 E16.5209\n"
             "G1 X37.94 Y175.7 E16.5237\n"
             "G1 X37.95 Y175.6 E16.5271\n"
             "G1 X37.95 Y145.99 E17.4908\n"
             "G1 X37.99 Y145.86 E17.4954\n"
             "G1 X38.15 Y145.8 E17.5009\n"
             "G1 X38.78 Y145.8 E17.5215\n"
             "G1 X38.89 Y145.83 E17.5251\n"
             "G1 X38.95 Y145.94 E17.5292\n"
             "G1 X38.95 Y175.59 E18.4945\n"
             "G1 X38.99 Y175.73 E18.499\n"
             "G1 X39.02 Y175.77 E18.5006\n"
             "G1 X39.15 Y175.8 E18.505\n"
             "G1 X45.32 Y175.8 E18.7062\n"
             "G1 X45.49 Y175.76 E18.7118\n"
             "G1 X45.53 Y175.61 E18.7169\n"
             "G1 X45.52 Y146 E19.6809\n"
             "G1 X45.56 Y145.87 E19.6854\n"
             "G1 X45.6 Y145.84 E19.6869\n"
             "G1 X45.74 Y145.8 E19.6915\n"
             "G1 X46.36 Y145.8 E19.7119\n"
             "G1 X46.5 Y145.84 E19.7164\n"
             "G1 X46.53 Y146 E19.7216\n"
             "G1 X46.53 Y175.61 E20.6856\n"
             "G1 X46.57 Y175.77 E20.6908\n"
             "G1 X46.74 Y175.8 E20.6966\n"
             "G1 X52.87 Y175.8 E20.8961\n"
             "G1 X53.02 Y175.77 E20.9011\n"
             "G1 X53.06 Y175.74 E20.9029\n"
             "G1 X53.1 Y175.61 E20.9073\n"
             "G1 X53.1 Y175.49 E20.9111\n"
             "G1 X53.1 Y145.99 E21.8715\n"
             "G1 X53.13 Y145.84 E21.8765\n"
             "G1 X53.3 Y145.8 E21.8822\n"
             "G1 X53.9 Y145.8 E21.9015\n"
             "G1 X53.99 Y145.81 E21.9044\n"
             "G1 X54.07 Y145.85 E21.9074\n"
             "G1 X54.1 Y146 E21.9125\n"
             "G1 X54.1 Y175.62 E22.8766\n"
             "G1 X54.13 Y175.75 E22.8809\n"
             "G1 X54.13 Y175.77 E22.8818\n"
             "G1 X54.31 Y175.8 E22.8879\n"
             "G1 X60.33 Y175.8 E23.0837\n"
             "G1 E21.8337 F2400\n"
             "G1 X60.33 Y175.8 Z0.55 E21.8337 F210\n"
             "G1 X131.41 Y142.93 E21.8337 F9000\n"
             "G1 X131.41 Y142.93 Z0.25 E21.8337 F210\n"
             "G1 E23.0837 F900\n"
             "G1 X132.19 Y142.93 E23.1092 F1800\n"
             "G1 X132.29 Y142.91 E23.1123\n"
             "G1 X132.34 Y142.86 E23.1148\n"
             "G1 X132.38 Y142.76 E23.1182\n"
             "G1 X132.38 Y142.13 E23.1387\n"
             "G1 X132.34 Y142 E23.1432\n"
             "G1 X132.31 Y141.96 E23.1447\n"
             "G1 X132.18 Y141.93 E23.1491\n"
             "G1 X102.57 Y141.93 E24.1127\n"
             "G1 X102.41 Y141.89 E24.1178\n"
             "G1 X102.38 Y141.72 E24.1236\n"
             "G1 X102.38 Y135.54 E24.3246\n"
             "G1 X102.42 Y135.39 E24.3298\n"
             "G1 X102.6 Y135.35 E24.3358\n"
             "G1 X132.16 Y135.35 E25.2978\n"
             "G1 X132.31 Y135.32 E25.3027\n"
             "G1 X132.36 Y135.26 E25.3053\n"
             "G1 X132.38 Y135.16 E25.3088\n"
             "G1 X132.37 Y134.55 E25.3287\n"
             "G1 X132.34 Y134.41 E25.3332\n"
             "G1 X132.22 Y134.36 E25.3375\n"
             "G1 X132.16 Y134.35 E25.3394\n"
             "G1 X102.57 Y134.35 E26.3021\n"
             "G1 X102.46 Y134.33 E26.3057\n"
             "G1 X102.41 Y134.28 E26.308\n"
             "G1 X102.38 Y134.13 E26.313\n"
             "G1 X102.38 Y127.96 E26.5137\n"
             "G1 X102.4 Y127.81 E26.5189\n"
             "G1 X102.41 Y127.81 E26.5192\n"
             "G1 X102.71 Y127.78 E26.529\n"
             "G1 X132.17 Y127.78 E27.4877\n"
             "G1 X132.34 Y127.73 E27.4934\n"
             "G1 X132.38 Y127.6 E27.498\n"
             "G1 X132.38 Y126.98 E27.5181\n"
             "G1 X132.33 Y126.81 E27.5238\n"
             "G1 X132.18 Y126.78 E27.5285\n"
             "G1 X102.58 Y126.78 E28.4918\n"
             "G1 X102.42 Y126.74 E28.4972\n"
             "G1 X102.38 Y126.57 E28.5028\n"
             "G1 X102.38 Y120.46 E28.7016\n"
             "G1 X102.41 Y120.23 E28.7093\n"
             "G1 X102.42 Y120.24 E28.7098\n"
             "G1 X102.56 Y120.2 E28.7144\n"
             "G1 X132.18 Y120.2 E29.6781\n"
             "G1 X132.33 Y120.17 E29.6833\n"
             "G1 X132.38 Y120.03 E29.6882\n"
             "G1 X132.38 Y119.41 E29.7081\n"
             "G1 X132.36 Y119.32 E29.7112\n"
             "G1 X132.33 Y119.23 E29.7142\n"
             "G1 X132.2 Y119.2 E29.7185\n"
             "G1 X97.58 Y119.2 E30.845\n"
             "G1 X97.42 Y119.24 E30.8503\n"
             "G1 X97.38 Y119.43 E30.8565\n"
             "G1 X97.38 Y180.6 E32.8472\n"
             "G1 X97.41 Y180.77 E32.8528\n"
             "G1 X97.57 Y180.8 E32.858\n"
             "G1 X132.18 Y180.8 E33.9844\n"
             "G1 X132.34 Y180.77 E33.9894\n"
             "G1 X132.38 Y180.63 E33.9942\n"
             "G1 X132.38 Y179.96 E34.016\n"
             "G1 X132.33 Y179.83 E34.0202\n"
             "G1 X132.21 Y179.8 E34.0245\n"
             "G1 X102.57 Y179.8 E34.989\n"
             "G1 X102.41 Y179.76 E34.9941\n"
             "G1 X102.37 Y179.58 E35\n"
             "G1 X102.37 Y173.43 E35.2002\n"
             "G1 X102.41 Y173.3 E35.2047\n"
             "G1 X102.45 Y173.26 E35.2064\n"
             "G1 X102.58 Y173.23 E35.2108\n"
             "G1 X132.12 Y173.23 E36.1722\n"
             "G1 X132.29 Y173.22 E36.1775\n"
             "G1 X132.35 Y173.16 E36.1801\n"
             "G1 X132.38 Y173.04 E36.1841\n"
             "G1 X132.38 Y172.44 E36.2039\n"
             "G1 X132.34 Y172.3 E36.2084\n"
             "G1 X132.31 Y172.26 E36.2099\n"
             "G1 X132.18 Y172.23 E36.2144\n"
             "G1 X102.55 Y172.23 E37.1784\n"
             "G1 X102.42 Y172.19 E37.183\n"
             "G1 X102.38 Y172.02 E37.1888\n"
             "G1 X102.38 Y165.84 E37.3898\n"
             "G1 X102.41 Y165.7 E37.3944\n"
             "G1 X102.4 Y165.69 E37.395\n"
             "G1 X102.6 Y165.65 E37.4014\n"
             "G1 X132.15 Y165.65 E38.363\n"
             "G1 X132.3 Y165.63 E38.3679\n"
             "G1 X132.36 Y165.56 E38.3711\n"
             "G1 X132.38 Y165.45 E38.3745\n"
             "G1 X132.37 Y164.85 E38.3942\n"
             "G1 X132.33 Y164.71 E38.3989\n"
             "G1 X132.22 Y164.66 E38.403\n"
             "G1 X132.16 Y164.65 E38.405\n"
             "G1 X102.6 Y164.65 E39.3669\n"
             "G1 X102.42 Y164.61 E39.3727\n"
             "G1 X102.37 Y164.43 E39.3787\n"
             "G1 X102.38 Y158.26 E39.5795\n"
             "G1 X102.4 Y158.1 E39.5847\n"
             "G1 X102.57 Y158.08 E39.59\n"
             "G1 X132.18 Y158.08 E40.5535\n"
             "G1 X132.34 Y158.03 E40.5591\n"
             "G1 X132.38 Y157.9 E40.5636\n"
             "G1 X132.38 Y157.28 E40.5837\n"
             "G1 X132.33 Y157.11 E40.5895\n"
             "G1 X132.19 Y157.08 E40.5942\n"
             "G1 X102.62 Y157.08 E41.5562\n"
             "G1 X102.54 Y157.07 E41.5588\n"
             "G1 X102.42 Y157.04 E41.563\n"
             "G1 X102.38 Y156.86 E41.5689\n"
             "G1 X102.38 Y150.74 E41.7681\n"
             "G1 X102.4 Y150.59 E41.773\n"
             "G1 X102.43 Y150.53 E41.7752\n"
             "G1 X102.57 Y150.5 E41.7797\n"
             "G1 X132.18 Y150.5 E42.7432\n"
             "G1 X132.33 Y150.47 E42.7483\n"
             "G1 X132.38 Y150.33 E42.7531\n"
             "G1 X132.38 Y149.72 E42.7727\n"
             "G1 X132.34 Y149.53 E42.779\n"
             "G1 X132.2 Y149.5 E42.7835\n"
             "G1 X102.56 Y149.5 E43.7479\n"
             "G1 X102.41 Y149.46 E43.7531\n"
             "G1 X102.38 Y149.28 E43.759\n"
             "G1 X102.37 Y143.13 E43.9591\n"
             "G1 X102.41 Y143 E43.9637\n"
             "G1 X102.44 Y142.96 E43.9652\n"
             "G1 X102.58 Y142.93 E43.9697\n"
             "G1 X131.37 Y142.93 E44.9067\n"
             "G1 E43.6567 F2400\n"
             "G1 X131.37 Y142.93 Z0.55 E43.6567 F210\n"
             "G92 E0 \n"
             "G1 E-1.4 F3000\n"
             "G92 E0\n"
             "M104 S0\n"
             "T0\n"
             "G1 X120 Y0.5 F8000\n"
             "M117 Heating Extruder\n"
             "M109 S195\n"
             "G92 E0 \n"
             "G1 E1.5 F100\n"
             "G92 E0\n"
             "G92 E0\n"
             "G1 X30.3 Y107.46 Z0.55 E0 F9000\n"
             "G1 X30.3 Y107.46 Z0.25 E0 F210\n"
             "G1 E1.25 F900\n"
             "G1 X30.42 Y107.48 E1.2542 F1800\n"
             "G1 X30.68 Y107.27 E1.2663\n"
             "G1 X31.09 Y107.08 E1.283\n"
             "G1 X31.39 Y107.02 E1.2942\n"
             "G1 X31.95 Y107.02 E1.3147\n"
             "G1 X32.39 Y107.1 E1.331\n"
             "G1 X32.73 Y107.24 E1.3444\n"
             "G1 X32.99 Y107.4 E1.3555\n"
             "G1 X33.21 Y107.6 E1.3662\n"
             "G1 X33.37 Y107.83 E1.3767\n"
             "G1 X33.57 Y108.29 E1.3947\n"
             "G1 X33.68 Y108.72 E1.4112\n"
             "G1 X33.69 Y109.2 E1.4286\n"
             "G1 X33.63 Y109.61 E1.4437\n"
             "G1 X33.54 Y109.88 E1.454\n"
             "G1 X33.37 Y110.18 E1.4666\n"
             "G1 X33.14 Y110.44 E1.4792\n"
             "G1 X32.86 Y110.63 E1.4916\n"
             "G1 X32.52 Y110.78 E1.5053\n"
             "G1 X32.04 Y110.88 E1.5229\n"
             "G1 X31.48 Y110.89 E1.5433\n"
             "G1 X31.01 Y110.83 E1.5609\n"
             "G1 X30.58 Y110.71 E1.5769\n"
             "G1 X30.22 Y110.53 E1.5917\n"
             "G1 X29.96 Y110.34 E1.6034\n"
             "G1 X29.75 Y110.14 E1.6141\n"
             "G1 X29.71 Y109.98 E1.6199\n"
             "G1 X29.75 Y109.39 E1.6418\n"
             "G1 X29.83 Y108.79 E1.6638\n"
             "G1 X29.98 Y108.23 E1.6849\n"
             "G1 X30.17 Y107.81 E1.7018\n"
             "G1 X30.25 Y107.68 E1.7074\n"
             "G1 X30.4 Y107.5 E1.7156\n"
             "G1 X30.37 Y107.39 E1.7198\n"
             "G1 X30.14 Y107.43 E1.7198 F9000\n"
             "G1 X30.11 Y107.31 E1.7241 F1800\n"
             "G1 X30.36 Y107.1 E1.736\n"
             "G1 X30.68 Y106.91 E1.7495\n"
             "G1 X31.12 Y106.78 E1.7663\n"
             "G1 X31.58 Y106.73 E1.7834\n"
             "G1 X32.11 Y106.75 E1.8026\n"
             "G1 X32.51 Y106.82 E1.8174\n"
             "G1 X32.92 Y106.97 E1.8335\n"
             "G1 X33.28 Y107.19 E1.8487\n"
             "G1 X33.52 Y107.42 E1.861\n"
             "G1 X33.74 Y107.75 E1.8754\n"
             "G1 X33.94 Y108.16 E1.892\n"
             "G1 X34.05 Y108.59 E1.9083\n"
             "G1 X34.09 Y109.04 E1.9246\n"
             "G1 X34.05 Y109.48 E1.9408\n"
             "G1 X33.97 Y109.82 E1.9535\n"
             "G1 X33.85 Y110.11 E1.9652\n"
             "G1 X33.65 Y110.41 E1.9783\n"
             "G1 X33.35 Y110.7 E1.9935\n"
             "G1 X32.98 Y110.93 E2.0094\n"
             "G1 X32.53 Y111.09 E2.0265\n"
             "G1 X32.02 Y111.17 E2.0456\n"
             "G1 X31.42 Y111.17 E2.0674\n"
             "G1 X30.98 Y111.11 E2.0835\n"
             "G1 X30.61 Y111 E2.0978\n"
             "G1 X30.38 Y110.9 E2.1069\n"
             "G1 X30.07 Y110.72 E2.1201\n"
             "G1 X29.94 Y110.71 E2.1248\n"
             "G1 X29.83 Y110.78 E2.1294\n"
             "G1 X29.77 Y110.89 E2.1338\n"
             "G1 X29.8 Y111.13 E2.1429\n"
             "G1 X29.89 Y111.62 E2.1609\n"
             "G1 X30.04 Y112.06 E2.1778\n"
             "G1 X30.29 Y112.52 E2.1971\n"
             "G1 X30.58 Y112.9 E2.2144\n"
             "G1 X30.88 Y113.17 E2.2291\n"
             "G1 X31.22 Y113.38 E2.2437\n"
             "G1 X31.7 Y113.57 E2.2625\n"
             "G1 X32.15 Y113.67 E2.2793\n"
             "G1 X32.67 Y113.7 E2.2982\n"
             "G1 X33.09 Y113.69 E2.3137\n"
             "G1 X33.45 Y113.64 E2.3268\n"
             "G1 X33.47 Y113.67 E2.3281\n"
             "G1 X33.47 Y114.91 E2.3736\n"
             "G1 X33.32 Y115.1 E2.3824\n"
             "G1 X33.28 Y115.1 E2.384\n"
             "G1 X33.11 Y114.92 E2.393\n"
             "G1 X33.11 Y114.21 E2.4187\n"
             "G1 X33.05 Y114.07 E2.4245\n"
             "G1 X32.94 Y113.98 E2.4295\n"
             "G1 X32.4 Y113.98 E2.449\n"
             "G1 X31.95 Y113.95 E2.4655\n"
             "G1 X31.55 Y113.85 E2.4805\n"
             "G1 X31.03 Y113.64 E2.5011\n"
             "G1 X30.66 Y113.43 E2.5165\n"
             "G1 X30.33 Y113.16 E2.5322\n"
             "G1 X30.06 Y112.85 E2.5471\n"
             "G1 X29.8 Y112.4 E2.5659\n"
             "G1 X29.56 Y111.79 E2.5899\n"
             "G1 X29.42 Y111.26 E2.6099\n"
             "G1 X29.34 Y110.62 E2.6336\n"
             "G1 X29.32 Y109.86 E2.6614\n"
             "G1 X29.38 Y109.08 E2.6898\n"
             "G1 X29.5 Y108.46 E2.713\n"
             "G1 X29.66 Y107.98 E2.7312\n"
             "G1 X29.88 Y107.59 E2.7477\n"
             "G1 X30.09 Y107.34 E2.7597\n"
             "G1 X30.21 Y107.36 E2.764\n"
             "G1 E1.514 F2400\n"
             "G1 X30.21 Y107.36 Z0.55 E1.514 F210\n"
             "G1 X33.12 Y115.22 E1.514 F9000\n"
             "G1 X33.12 Y115.22 Z0.25 E1.514 F210\n"
             "G1 E2.764 F900\n"
             "G1 X33.07 Y115.26 E2.7663 F1800\n"
             "G1 X32.87 Y115.3 E2.7729\n"
             "G1 X32.38 Y115.3 E2.7888\n"
             "G1 X32.22 Y115.34 E2.7941\n"
             "G1 X32.18 Y115.5 E2.7995\n"
             "G1 X32.18 Y145.1 E3.7635\n"
             "G1 X32.15 Y145.28 E3.7694\n"
             "G1 X32.14 Y145.26 E3.7701\n"
             "G1 X31.97 Y145.3 E3.7759\n"
             "G1 X31.44 Y145.3 E3.7932\n"
             "G1 X31.21 Y145.26 E3.8007\n"
             "G1 X31.2 Y145.27 E3.8012\n"
             "G1 X31.18 Y145.12 E3.8063\n"
             "G1 X31.18 Y115.5 E4.7707\n"
             "G1 X31.13 Y115.34 E4.7763\n"
             "G1 X31 Y115.3 E4.7808\n"
             "G1 X24.43 Y115.3 E4.9945\n"
             "G1 X24.3 Y115.29 E4.999\n"
             "G1 X24.22 Y115.25 E5.0018\n"
             "G1 X24.2 Y115.27 E5.0027\n"
             "G1 X24.18 Y115.12 E5.0077\n"
             "G1 X24.18 Y85.47 E5.9731\n"
             "G1 X24.21 Y85.34 E5.9776\n"
             "G1 X24.2 Y85.33 E5.9784\n"
             "G1 X24.41 Y85.3 E5.9852\n"
             "G1 X55.16 Y85.3 E6.9866\n"
             "G1 X55.25 Y85.32 E6.9898\n"
             "G1 X55.35 Y85.38 E6.9936\n"
             "G1 E5.7436 F2400\n"
             "G1 X55.35 Y85.38 Z0.55 E5.7436 F210\n"
             "G1 X55.55 Y85.61 E5.7436 F9000\n"
             "G1 X55.55 Y85.61 Z0.25 E5.7436 F210\n"
             "G1 E6.9936 F900\n"
             "G1 X55.52 Y85.5 E6.9979 F1800\n"
             "G1 X55.54 Y85.47 E6.9991\n"
             "G1 X55.59 Y85.47 E7.0008\n"
             "G1 X55.72 Y85.66 E7.009\n"
             "G1 X55.72 Y87.96 E7.0929\n"
             "G1 X55.58 Y88.14 E7.1014\n"
             "G1 X55.5 Y88.1 E7.1043\n"
             "G1 X55.36 Y87.92 E7.1129\n"
             "G1 X55.36 Y85.69 E7.1941\n"
             "G1 X55.47 Y85.54 E7.2009\n"
             "G1 X55.5 Y85.52 E7.2022\n"
             "G1 X55.76 Y85.35 E7.2022 F9000\n"
             "G1 X55.96 Y85.3 E7.2087 F1800\n"
             "G1 X66.42 Y85.3 E7.55\n"
             "G1 X66.6 Y85.35 E7.5563\n"
             "G1 E6.3063 F2400\n"
             "G1 X66.6 Y85.35 Z0.55 E6.3063 F210\n"
             "G1 X55.35 Y88.23 E6.3063 F9000\n"
             "G1 X55.35 Y88.23 Z0.25 E6.3063 F210\n"
             "G1 E7.5563 F900\n"
             "G1 X55.19 Y88.3 E7.5619 F1800\n"
             "G1 X54.62 Y88.3 E7.5802\n"
             "G1 X54.53 Y88.31 E7.5831\n"
             "G1 X54.35 Y88.38 E7.5891\n"
             "G1 X54.36 Y88.38 E7.5893\n"
             "G1 X54.45 Y88.64 E7.5982\n"
             "G1 X60.3 Y97.79 E7.9447\n"
             "G1 X60.35 Y97.91 E7.9487\n"
             "G1 X60.3 Y98.02 E7.9528\n"
             "G1 X54.2 Y107.56 E8.314\n"
             "G1 X54.14 Y107.68 E8.3183\n"
             "G1 X54.1 Y107.84 E8.3237\n"
             "G1 X54.31 Y107.9 E8.3307\n"
             "G1 X55.44 Y107.9 E8.3667\n"
             "G1 X55.55 Y107.89 E8.3703\n"
             "G1 X55.65 Y107.78 E8.3749\n"
             "G1 X61 Y99.43 E8.6913\n"
             "G1 X61.06 Y99.35 E8.6945\n"
             "G1 X61.18 Y99.26 E8.6992\n"
             "G1 X61.31 Y99.37 E8.7046\n"
             "G1 X61.39 Y99.48 E8.709\n"
             "G1 X66.68 Y107.76 E9.0225\n"
             "G1 X66.77 Y107.87 E9.027\n"
             "G1 X66.87 Y107.9 E9.0305\n"
             "G1 X67.97 Y107.9 E9.0654\n"
             "G1 X68.11 Y107.88 E9.0702\n"
             "G1 X68.24 Y107.82 E9.0745\n"
             "G1 X68.28 Y107.86 E9.0763\n"
             "G1 X68.36 Y107.88 E9.0791\n"
             "G1 X68.41 Y107.93 E9.0813\n"
             "G1 X68.5 Y107.96 E9.0841\n"
             "G1 X68.59 Y108.03 E9.0878\n"
             "G1 X68.67 Y108.06 E9.0906\n"
             "G1 E7.8406 F2400\n"
             "G1 X68.67 Y108.06 Z0.55 E7.8406 F210\n"
             "G1 X91.28 Y106.28 E7.8406 F9000\n"
             "G1 X91.28 Y106.28 Z0.25 E7.8406 F210\n"
             "G1 E9.0906 F900\n"
             "G1 X91.22 Y106.16 E9.0956 F1800\n"
             "G1 X91.94 Y106.15 E9.122\n"
             "G1 X93.28 Y107.02 E9.1801\n"
             "G1 X93.3 Y107.06 E9.182\n"
             "G1 X93.3 Y107.5 E9.198\n"
             "G1 X93.27 Y107.54 E9.1997\n"
             "G1 X91.99 Y106.68 E9.2559\n"
             "G1 X91.9 Y106.66 E9.2593\n"
             "G1 X91.79 Y106.68 E9.2635\n"
             "G1 X91.69 Y106.79 E9.2688\n"
             "G1 X91.67 Y106.88 E9.2724\n"
             "G1 X91.67 Y114.91 E9.5651\n"
             "G1 X91.54 Y115.06 E9.5724\n"
             "G1 X91.5 Y115.06 E9.5739\n"
             "G1 X91.37 Y114.91 E9.581\n"
             "G1 X91.31 Y114.9 E9.5831\n"
             "G1 X91.31 Y114.14 E9.611\n"
             "G1 X91.28 Y114.02 E9.6153\n"
             "G1 X91.2 Y113.91 E9.6205\n"
             "G1 X91.2 Y106.18 E9.9025\n"
             "G1 X91.33 Y106.22 E9.9075\n"
             "G1 X93.08 Y107.18 E9.9075 F9000\n"
             "G1 X93.08 Y107.15 E9.9081 F1800\n"
             "G1 X92.11 Y106.51 E9.9292\n"
             "G1 X91.99 Y106.44 E9.9318\n"
             "G1 X91.86 Y106.4 E9.9342\n"
             "G1 X91.63 Y106.44 E9.9385\n"
             "G1 X91.52 Y106.48 E9.9405\n"
             "G1 X91.48 Y106.6 E9.9428\n"
             "G1 X91.44 Y106.87 E9.9478\n"
             "G1 X91.44 Y113.78 E10.074\n"
             "G1 X91.46 Y113.9 E10.0761\n"
             "G1 X91.36 Y115.22 E10.0761 F9000\n"
             "G1 X91.24 Y115.3 E10.0811 F1800\n"
             "G1 X91.18 Y115.47 E10.0871\n"
             "G1 X91.17 Y145.09 E11.0514\n"
             "G1 X91.13 Y145.24 E11.0564\n"
             "G1 X91 Y145.3 E11.0611\n"
             "G1 X90.35 Y145.3 E11.0824\n"
             "G1 X90.25 Y145.27 E11.0857\n"
             "G1 X90.2 Y145.23 E11.0879\n"
             "G1 X90.17 Y145.06 E11.0935\n"
             "G1 X90.18 Y115.51 E12.0554\n"
             "G1 X90.14 Y115.38 E12.06\n"
             "G1 X90.11 Y115.34 E12.0615\n"
             "G1 X89.98 Y115.3 E12.0661\n"
             "G1 X84.02 Y115.3 E12.2601\n"
             "G1 X83.86 Y115.33 E12.2654\n"
             "G1 X83.81 Y115.43 E12.2689\n"
             "G1 X83.8 Y115.5 E12.2712\n"
             "G1 X83.8 Y145.1 E13.2348\n"
             "G1 X83.77 Y145.23 E13.2393\n"
             "G1 X83.74 Y145.27 E13.2408\n"
             "G1 X83.6 Y145.3 E13.2453\n"
             "G1 X82.99 Y145.3 E13.2652\n"
             "G1 X82.86 Y145.27 E13.2695\n"
             "G1 X82.8 Y145.16 E13.2737\n"
             "G1 X82.8 Y115.5 E14.2392\n"
             "G1 X82.76 Y115.37 E14.2436\n"
             "G1 X82.73 Y115.34 E14.2451\n"
             "G1 X82.6 Y115.3 E14.2496\n"
             "G1 X76.62 Y115.3 E14.4442\n"
             "G1 X76.51 Y115.32 E14.448\n"
             "G1 X76.44 Y115.38 E14.4511\n"
             "G1 X76.43 Y115.49 E14.4547\n"
             "G1 X76.43 Y145.09 E15.4181\n"
             "G1 X76.39 Y145.26 E15.4239\n"
             "G1 X76.24 Y145.3 E15.4291\n"
             "G1 X75.63 Y145.3 E15.4487\n"
             "G1 X75.49 Y145.27 E15.4536\n"
             "G1 X75.43 Y145.16 E15.4576\n"
             "G1 X75.43 Y145.09 E15.4599\n"
             "G1 X75.43 Y115.49 E16.4234\n"
             "G1 X75.38 Y115.35 E16.4282\n"
             "G1 X75.23 Y115.3 E16.4336\n"
             "G1 X69.24 Y115.3 E16.6286\n"
             "G1 X69.14 Y115.32 E16.6316\n"
             "G1 X69.1 Y115.35 E16.6333\n"
             "G1 X69.08 Y115.33 E16.6343\n"
             "G1 X69.05 Y115.53 E16.6409\n"
             "G1 X69.05 Y145.11 E17.604\n"
             "G1 X69.01 Y145.27 E17.6094\n"
             "G1 X68.86 Y145.3 E17.6146\n"
             "G1 X68.26 Y145.3 E17.634\n"
             "G1 X68.11 Y145.26 E17.6391\n"
             "G1 X68.06 Y145.15 E17.6429\n"
             "G1 X68.05 Y145.08 E17.6455\n"
             "G1 X68.05 Y115.48 E18.6089\n"
             "G1 X68.01 Y115.35 E18.6136\n"
             "G1 X67.85 Y115.3 E18.619\n"
             "G1 X61.85 Y115.3 E18.8142\n"
             "G1 X61.72 Y115.34 E18.8187\n"
             "G1 X61.7 Y115.33 E18.8194\n"
             "G1 X61.68 Y115.56 E18.8271\n"
             "G1 X61.68 Y145.1 E19.7889\n"
             "G1 X61.65 Y145.24 E19.7934\n"
             "G1 X61.58 Y145.29 E19.796\n"
             "G1 X61.47 Y145.3 E19.7996\n"
             "G1 X60.86 Y145.3 E19.8195\n"
             "G1 X60.73 Y145.26 E19.8241\n"
             "G1 X60.68 Y145.11 E19.8291\n"
             "G1 X60.68 Y115.48 E20.794\n"
             "G1 X60.64 Y115.34 E20.7986\n"
             "G1 X60.47 Y115.3 E20.8042\n"
             "G1 X54.48 Y115.3 E20.999\n"
             "G1 X54.34 Y115.34 E21.0038\n"
             "G1 X54.3 Y115.52 E21.0099\n"
             "G1 X54.3 Y145.09 E21.9725\n"
             "G1 X54.27 Y145.27 E21.9784\n"
             "G1 X54.13 Y145.3 E21.983\n"
             "G1 X53.49 Y145.3 E22.0037\n"
             "G1 X53.35 Y145.26 E22.0085\n"
             "G1 X53.3 Y145.1 E22.0139\n"
             "G1 X53.3 Y115.51 E22.9773\n"
             "G1 X53.26 Y115.34 E22.983\n"
             "G1 X53.13 Y115.3 E22.9876\n"
             "G1 X47.11 Y115.3 E23.1834\n"
             "G1 X47.01 Y115.33 E23.1868\n"
             "G1 X46.96 Y115.37 E23.1889\n"
             "G1 X46.92 Y115.5 E23.1934\n"
             "G1 X46.93 Y145.14 E24.1581\n"
             "G1 X46.89 Y145.26 E24.1625\n"
             "G1 X46.76 Y145.3 E24.1669\n"
             "G1 X46.11 Y145.3 E24.1881\n"
             "G1 X45.97 Y145.26 E24.1928\n"
             "G1 X45.93 Y145.09 E24.1984\n"
             "G1 X45.93 Y115.52 E25.1612\n"
             "G1 X45.89 Y115.36 E25.1664\n"
             "G1 X45.78 Y115.31 E25.1704\n"
             "G1 X45.72 Y115.3 E25.1725\n"
             "G1 X39.92 Y115.3 E25.3614\n"
             "G1 X39.66 Y115.31 E25.3697\n"
             "G1 X39.59 Y115.36 E25.3725\n"
             "G1 X39.55 Y115.49 E25.3769\n"
             "G1 X39.55 Y145.06 E26.3397\n"
             "G1 X39.54 Y145.17 E26.3432\n"
             "G1 X39.51 Y145.26 E26.3464\n"
             "G1 X39.37 Y145.3 E26.351\n"
             "G1 X38.73 Y145.3 E26.372\n"
             "G1 X38.59 Y145.26 E26.3766\n"
             "G1 X38.57 Y145.28 E26.3775\n"
             "G1 X38.55 Y144.83 E26.3921\n"
             "G1 X38.55 Y115.51 E27.3467\n"
             "G1 X38.5 Y115.34 E27.3525\n"
             "G1 X38.37 Y115.3 E27.3572\n"
             "G1 X33.7 Y115.3 E27.509\n"
             "G1 X33.52 Y115.27 E27.515\n"
             "G1 X33.47 Y115.22 E27.5173\n"
             "G1 E26.2673 F2400\n"
             "G1 X33.47 Y115.22 Z0.55 E26.2673 F210\n"
             "G1 X68.24 Y107.78 E26.2673 F9000\n"
             "G1 X68.24 Y107.78 Z0.25 E26.2673 F210\n"
             "G1 E27.5173 F900\n"
             "G1 X68.19 Y107.61 E27.5228 F1800\n"
             "G1 X62.04 Y98.01 E27.8855\n"
             "G1 X61.99 Y97.91 E27.8894\n"
             "G1 X62.04 Y97.8 E27.8931\n"
             "G1 X67.92 Y88.61 E28.2403\n"
             "G1 X67.96 Y88.5 E28.244\n"
             "G1 X67.99 Y88.36 E28.2486\n"
             "G1 X68.03 Y88.36 E28.2499\n"
             "G1 X67.9 Y88.35 E28.2542\n"
             "G1 X67.78 Y88.3 E28.2583\n"
             "G1 X67.21 Y88.3 E28.2765\n"
             "G1 X67.06 Y88.27 E28.2813\n"
             "G1 X66.97 Y88.2 E28.285\n"
             "G1 E27.035 F2400\n"
             "G1 X66.97 Y88.2 Z0.55 E27.035 F210\n"
             "G1 X66.79 Y85.62 E27.035 F9000\n"
             "G1 X66.79 Y85.62 Z0.25 E27.035 F210\n"
             "G1 E28.285 F900\n"
             "G1 X66.79 Y85.48 E28.2897 F1800\n"
             "G1 X66.86 Y85.52 E28.2923\n"
             "G1 X66.99 Y85.69 E28.3004\n"
             "G1 X66.99 Y87.91 E28.3812\n"
             "G1 X66.95 Y87.93 E28.3829\n"
             "G1 X66.82 Y88.07 E28.3901\n"
             "G1 X66.77 Y88.07 E28.3918\n"
             "G1 X66.63 Y87.9 E28.4001\n"
             "G1 X66.63 Y85.67 E28.4813\n"
             "G1 X66.77 Y85.5 E28.4894\n"
             "G1 X66.87 Y85.58 E28.4942\n"
             "G1 E27.2442 F2400\n"
             "G1 X66.87 Y85.58 Z0.55 E27.2442 F210\n"
             "G1 X66.64 Y88.24 E27.2442 F9000\n"
             "G1 X66.64 Y88.24 Z0.25 E27.2442 F210\n"
             "G1 E28.4942 F900\n"
             "G1 X66.46 Y88.4 E28.5019 F1800\n"
             "G1 X61.36 Y96.37 E28.804\n"
             "G1 X61.24 Y96.52 E28.8098\n"
             "G1 X61.2 Y96.52 E28.8112\n"
             "G1 X61.18 Y96.56 E28.8125\n"
             "G1 X61.06 Y96.45 E28.8177\n"
             "G1 X60.99 Y96.37 E28.8211\n"
             "G1 X55.88 Y88.38 E29.1237\n"
             "G1 X55.75 Y88.26 E29.1293\n"
             "G1 E27.8793 F2400\n"
             "G1 X55.75 Y88.26 Z0.55 E27.8793 F210\n"
             "G1 X66.99 Y85.36 E27.8793 F9000\n"
             "G1 X66.99 Y85.36 Z0.25 E27.8793 F210\n"
             "G1 E29.1293 F900\n"
             "G1 X67.12 Y85.31 E29.1335 F1800\n"
             "G1 X67.29 Y85.3 E29.139\n"
             "G1 X97.94 Y85.3 E30.1367\n"
             "G1 X98.05 Y85.32 E30.1403\n"
             "G1 X98.14 Y85.36 E30.1435\n"
             "G1 X98.18 Y85.49 E30.1481\n"
             "G1 X98.18 Y115.07 E31.1108\n"
             "G1 X98.17 Y115.14 E31.1133\n"
             "G1 X98.14 Y115.26 E31.1173\n"
             "G1 X97.95 Y115.3 E31.1233\n"
             "G1 X91.91 Y115.3 E31.3202\n"
             "G1 X91.71 Y115.26 E31.3268\n"
             "G1 X91.66 Y115.21 E31.329\n"
             "G1 E30.079 F2400\n"
             "G1 X91.66 Y115.21 Z0.55 E30.079 F210\n"
             "G1 X168.27 Y118.72 E30.079 F9000\n"
             "G1 X168.27 Y118.72 Z0.25 E30.079 F210\n"
             "G1 E31.329 F900\n"
             "G1 X168.38 Y118.75 E31.3331 F1800\n"
             "G1 X168.51 Y118.69 E31.3382\n"
             "G1 X168.88 Y118.7 E31.3516\n"
             "G1 X169.47 Y118.74 E31.3735\n"
             "G1 X170.05 Y118.85 E31.3949\n"
             "G1 X170.66 Y119.05 E31.4183\n"
             "G1 X171 Y119.21 E31.4318\n"
             "G1 X171.3 Y119.45 E31.446\n"
             "G1 X171.51 Y119.73 E31.4587\n"
             "G1 X171.72 Y120.15 E31.476\n"
             "G1 X171.78 Y120.48 E31.488\n"
             "G1 X171.78 Y121.07 E31.5097\n"
             "G1 X171.71 Y121.49 E31.5252\n"
             "G1 X171.6 Y121.83 E31.5383\n"
             "G1 X171.43 Y122.14 E31.5511\n"
             "G1 X171.2 Y122.42 E31.5645\n"
             "G1 X170.92 Y122.63 E31.577\n"
             "G1 X170.46 Y122.83 E31.5956\n"
             "G1 X170.05 Y122.94 E31.6109\n"
             "G1 X169.61 Y122.99 E31.6272\n"
             "G1 X169.22 Y122.96 E31.6416\n"
             "G1 X168.78 Y122.86 E31.658\n"
             "G1 X168.41 Y122.67 E31.6731\n"
             "G1 X168.14 Y122.44 E31.6857\n"
             "G1 X167.91 Y122.14 E31.6997\n"
             "G1 X167.74 Y121.78 E31.7143\n"
             "G1 X167.63 Y121.33 E31.7312\n"
             "G1 X167.6 Y120.86 E31.7484\n"
             "G1 X167.62 Y120.34 E31.7675\n"
             "G1 X167.73 Y119.83 E31.7863\n"
             "G1 X167.88 Y119.43 E31.8017\n"
             "G1 X168.08 Y119.08 E31.8165\n"
             "G1 X168.36 Y118.77 E31.832\n"
             "G1 X168.34 Y118.65 E31.8361\n"
             "G1 E30.5861 F2400\n"
             "G1 X168.34 Y118.65 Z0.55 E30.5861 F210\n"
             "G1 X165.19 Y119.37 E30.5861 F9000\n"
             "G1 X165.19 Y119.37 Z0.25 E30.5861 F210\n"
             "G1 E31.8361 F900\n"
             "G1 X165.17 Y119.25 E31.8404 F1800\n"
             "G1 X165.49 Y118.98 E31.8559\n"
             "G1 X165.94 Y118.73 E31.8746\n"
             "G1 X166.62 Y118.47 E31.901\n"
             "G1 X167.25 Y118.31 E31.9249\n"
             "G1 X167.93 Y118.23 E31.9497\n"
             "G1 X168.74 Y118.21 E31.9793\n"
             "G1 X169.5 Y118.27 E32.0073\n"
             "G1 X170.06 Y118.37 E32.028\n"
             "G1 X170.57 Y118.51 E32.0472\n"
             "G1 X170.96 Y118.69 E32.0628\n"
             "G1 X171.31 Y118.91 E32.0779\n"
             "G1 X171.65 Y119.24 E32.0951\n"
             "G1 X171.86 Y119.56 E32.1093\n"
             "G1 X172.01 Y119.9 E32.1227\n"
             "G1 X172.1 Y120.29 E32.1375\n"
             "G1 X172.14 Y120.83 E32.157\n"
             "G1 X172.1 Y121.4 E32.1781\n"
             "G1 X172 Y121.83 E32.1943\n"
             "G1 X171.84 Y122.23 E32.2098\n"
             "G1 X171.65 Y122.55 E32.2234\n"
             "G1 X171.39 Y122.82 E32.237\n"
             "G1 X171.02 Y123.08 E32.2536\n"
             "G1 X170.65 Y123.25 E32.2684\n"
             "G1 X170.17 Y123.4 E32.2868\n"
             "G1 X169.74 Y123.45 E32.3027\n"
             "G1 X169.29 Y123.44 E32.319\n"
             "G1 X168.87 Y123.37 E32.3344\n"
             "G1 X168.46 Y123.21 E32.3505\n"
             "G1 X168.09 Y122.97 E32.3665\n"
             "G1 X167.81 Y122.69 E32.3811\n"
             "G1 X167.57 Y122.33 E32.3969\n"
             "G1 X167.4 Y121.93 E32.4128\n"
             "G1 X167.28 Y121.46 E32.4304\n"
             "G1 X167.24 Y120.88 E32.4515\n"
             "G1 X167.27 Y120.35 E32.471\n"
             "G1 X167.37 Y119.83 E32.49\n"
             "G1 X167.53 Y119.41 E32.5065\n"
             "G1 X167.76 Y119.03 E32.5228\n"
             "G1 X167.76 Y118.9 E32.5277\n"
             "G1 X167.68 Y118.78 E32.5329\n"
             "G1 X167.54 Y118.74 E32.5379\n"
             "G1 X167.11 Y118.81 E32.554\n"
             "G1 X166.65 Y118.92 E32.5714\n"
             "G1 X166.25 Y119.07 E32.5868\n"
             "G1 X165.77 Y119.33 E32.6068\n"
             "G1 X165.38 Y119.63 E32.6248\n"
             "G1 X165.09 Y119.98 E32.6413\n"
             "G1 X164.85 Y120.39 E32.6587\n"
             "G1 X164.68 Y120.85 E32.6767\n"
             "G1 X164.56 Y121.36 E32.6956\n"
             "G1 X164.53 Y121.9 E32.7154\n"
             "G1 X164.55 Y122.32 E32.7309\n"
             "G1 X164.6 Y122.78 E32.7477\n"
             "G1 X163.26 Y122.78 E32.7966\n"
             "G1 X163.24 Y122.75 E32.7982\n"
             "G1 X163.08 Y122.61 E32.8058\n"
             "G1 X163.05 Y122.54 E32.8086\n"
             "G1 X163.21 Y122.42 E32.8158\n"
             "G1 X163.97 Y122.42 E32.8437\n"
             "G1 X164.06 Y122.4 E32.847\n"
             "G1 X164.16 Y122.31 E32.852\n"
             "G1 X164.19 Y122.16 E32.8574\n"
             "G1 X164.18 Y121.54 E32.88\n"
             "G1 X164.24 Y120.98 E32.9004\n"
             "G1 X164.39 Y120.5 E32.9188\n"
             "G1 X164.65 Y119.93 E32.9417\n"
             "G1 X164.9 Y119.55 E32.9583\n"
             "G1 X165.15 Y119.27 E32.9719\n"
             "G1 X165.26 Y119.3 E32.9762\n"
             "G1 E31.7262 F2400\n"
             "G1 X165.26 Y119.3 Z0.55 E31.7262 F210\n"
             "G1 X168.01 Y122.59 E31.7262 F9000\n"
             "G1 X168.01 Y122.59 Z0.25 E31.7262 F210\n"
             "G1 E32.9762 F900\n"
             "G1 X168.26 Y122.83 E32.9825 F1800\n"
             "G1 X168.57 Y123.01 E32.9889\n"
             "G1 X168.91 Y123.14 E32.9956\n"
             "G1 X169.35 Y123.21 E33.0037\n"
             "G1 X169.84 Y123.21 E33.0126\n"
             "G1 X170.28 Y123.13 E33.0208\n"
             "G1 X170.72 Y122.98 E33.0293\n"
             "G1 X171.1 Y122.77 E33.0372\n"
             "G1 X171.31 Y122.6 E33.0421\n"
             "G1 X171.44 Y122.46 E33.0457\n"
             "G1 E31.7957 F2400\n"
             "G1 X171.44 Y122.46 Z0.55 E31.7957 F210\n"
             "G1 X171.25 Y119.14 E31.7957 F9000\n"
             "G1 X171.25 Y119.14 Z0.25 E31.7957 F210\n"
             "G1 E33.0457 F900\n"
             "G1 X170.97 Y118.94 E33.052 F1800\n"
             "G1 X170.52 Y118.74 E33.0609\n"
             "G1 X169.97 Y118.59 E33.0713\n"
             "G1 X169.25 Y118.49 E33.0846\n"
             "G1 X168.58 Y118.45 E33.0968\n"
             "G1 X168.31 Y118.48 E33.1017\n"
             "G1 X168.08 Y118.56 E33.1062\n"
             "G1 X168.04 Y118.59 E33.1071\n"
             "G1 X167.97 Y118.59 E33.1084\n"
             "G1 X167.92 Y118.56 E33.1094\n"
             "G1 X167.75 Y118.52 E33.1128\n"
             "G1 X167.49 Y118.51 E33.1175\n"
             "G1 X166.9 Y118.62 E33.1284\n"
             "G1 X166.36 Y118.79 E33.1388\n"
             "G1 X165.86 Y119.02 E33.1488\n"
             "G1 X165.47 Y119.27 E33.1573\n"
             "G1 X165.24 Y119.48 E33.163\n"
             "G1 X165.05 Y119.7 E33.1683\n"
             "G1 E31.9183 F2400\n"
             "G1 X165.05 Y119.7 Z0.55 E31.9183 F210\n"
             "G1 X162.97 Y122.77 E31.9183 F9000\n"
             "G1 X162.97 Y122.77 Z0.25 E31.9183 F210\n"
             "G1 E33.1683 F900\n"
             "G1 X162.93 Y122.82 E33.1705 F1800\n"
             "G1 X162.88 Y123 E33.1767\n"
             "G1 X162.88 Y127.19 E33.3132\n"
             "G1 X162.83 Y127.34 E33.3182\n"
             "G1 X162.68 Y127.38 E33.3231\n"
             "G1 X133.06 Y127.38 E34.2877\n"
             "G1 X132.92 Y127.42 E34.2924\n"
             "G1 X132.88 Y127.58 E34.2978\n"
             "G1 X132.88 Y128.15 E34.3164\n"
             "G1 X132.9 Y128.35 E34.3229\n"
             "G1 X133.15 Y128.38 E34.3309\n"
             "G1 X162.71 Y128.38 E35.2934\n"
             "G1 X162.85 Y128.42 E35.2981\n"
             "G1 X162.88 Y128.56 E35.3029\n"
             "G1 X162.88 Y134.54 E35.4974\n"
             "G1 X162.86 Y134.63 E35.5004\n"
             "G1 X162.82 Y134.71 E35.5035\n"
             "G1 X162.66 Y134.75 E35.5087\n"
             "G1 X133.09 Y134.75 E36.4719\n"
             "G1 X133.01 Y134.76 E36.4744\n"
             "G1 X132.91 Y134.8 E36.4777\n"
             "G1 X132.88 Y134.96 E36.4831\n"
             "G1 X132.88 Y135.57 E36.5029\n"
             "G1 X132.9 Y135.73 E36.5082\n"
             "G1 X132.91 Y135.71 E36.5088\n"
             "G1 X133.05 Y135.75 E36.5135\n"
             "G1 X162.6 Y135.75 E37.4755\n"
             "G1 X162.72 Y135.75 E37.4795\n"
             "G1 X162.84 Y135.8 E37.4838\n"
             "G1 X162.88 Y135.95 E37.4889\n"
             "G1 X162.87 Y141.91 E37.683\n"
             "G1 X162.86 Y142.02 E37.6864\n"
             "G1 X162.81 Y142.09 E37.6894\n"
             "G1 X162.66 Y142.13 E37.6944\n"
             "G1 X133.07 Y142.13 E38.6581\n"
             "G1 X132.89 Y142.14 E38.6637\n"
             "G1 X132.91 Y142.16 E38.6646\n"
             "G1 X132.88 Y142.3 E38.6691\n"
             "G1 X132.88 Y142.93 E38.6896\n"
             "G1 X132.9 Y143.11 E38.6955\n"
             "G1 X132.91 Y143.09 E38.6962\n"
             "G1 X133.12 Y143.13 E38.7033\n"
             "G1 X162.67 Y143.13 E39.6655\n"
             "G1 X162.82 Y143.16 E39.6704\n"
             "G1 X162.87 Y143.26 E39.674\n"
             "G1 X162.88 Y143.34 E39.6764\n"
             "G1 X162.88 Y149.32 E39.8712\n"
             "G1 X162.84 Y149.46 E39.8759\n"
             "G1 X162.66 Y149.5 E39.882\n"
             "G1 X133.11 Y149.5 E40.8441\n"
             "G1 X132.99 Y149.52 E40.8479\n"
             "G1 X132.91 Y149.55 E40.8507\n"
             "G1 X132.87 Y149.68 E40.8552\n"
             "G1 X132.87 Y150.32 E40.876\n"
             "G1 X132.91 Y150.46 E40.8807\n"
             "G1 X133.08 Y150.5 E40.8862\n"
             "G1 X162.67 Y150.5 E41.8498\n"
             "G1 X162.84 Y150.54 E41.8555\n"
             "G1 X162.88 Y150.68 E41.86\n"
             "G1 X162.88 Y156.68 E42.0556\n"
             "G1 X162.83 Y156.84 E42.0609\n"
             "G1 X162.65 Y156.88 E42.0669\n"
             "G1 X133.1 Y156.88 E43.0293\n"
             "G1 X132.94 Y156.91 E43.0346\n"
             "G1 X132.88 Y157.03 E43.039\n"
             "G1 X132.88 Y157.68 E43.06\n"
             "G1 X132.9 Y157.79 E43.0637\n"
             "G1 X132.95 Y157.84 E43.066\n"
             "G1 X133.07 Y157.88 E43.0702\n"
             "G1 X162.63 Y157.88 E44.0328\n"
             "G1 X162.75 Y157.89 E44.0368\n"
             "G1 X162.84 Y157.92 E44.0398\n"
             "G1 X162.88 Y158.05 E44.0441\n"
             "G1 X162.88 Y164.07 E44.2401\n"
             "G1 X162.85 Y164.23 E44.2454\n"
             "G1 X162.84 Y164.22 E44.246\n"
             "G1 X162.7 Y164.25 E44.2506\n"
             "G1 X133.09 Y164.25 E45.2151\n"
             "G1 X132.94 Y164.29 E45.2201\n"
             "G1 X132.89 Y164.36 E45.2228\n"
             "G1 X132.88 Y164.47 E45.2263\n"
             "G1 X132.88 Y165.07 E45.2459\n"
             "G1 X132.91 Y165.21 E45.2505\n"
             "G1 X133.06 Y165.25 E45.2555\n"
             "G1 X162.69 Y165.25 E46.2203\n"
             "G1 X162.85 Y165.27 E46.2257\n"
             "G1 X162.84 Y165.28 E46.2261\n"
             "G1 X162.88 Y165.58 E46.2357\n"
             "G1 X162.88 Y171.37 E46.4245\n"
             "G1 X162.85 Y171.6 E46.432\n"
             "G1 X162.83 Y171.59 E46.4327\n"
             "G1 X162.7 Y171.63 E46.4373\n"
             "G1 X133.08 Y171.63 E47.4016\n"
             "G1 X132.93 Y171.66 E47.4068\n"
             "G1 X132.88 Y171.81 E47.4118\n"
             "G1 X132.88 Y172.43 E47.432\n"
             "G1 X132.91 Y172.58 E47.4372\n"
             "G1 X133.05 Y172.63 E47.4419\n"
             "G1 X162.68 Y172.63 E48.4069\n"
             "G1 X162.85 Y172.65 E48.4123\n"
             "G1 X162.84 Y172.66 E48.4125\n"
             "G1 X162.88 Y172.96 E48.4223\n"
             "G1 X162.88 Y178.76 E48.6115\n"
             "G1 X162.84 Y178.96 E48.618\n"
             "G1 X162.69 Y179 E48.6232\n"
             "G1 X133.16 Y179 E49.5846\n"
             "G1 X132.92 Y179.03 E49.5927\n"
             "G1 X132.88 Y179.18 E49.5977\n"
             "G1 X132.88 Y179.83 E49.6189\n"
             "G1 X132.91 Y179.96 E49.6233\n"
             "G1 X133.07 Y180 E49.6286\n"
             "G1 X162.69 Y180 E50.5933\n"
             "G1 X162.84 Y180.05 E50.5984\n"
             "G1 E49.3484 F2400\n"
             "G1 X162.84 Y180.05 Z0.55 E49.3484 F210\n"
             "G1 X190.02 Y156.94 E49.3484 F9000\n"
             "G1 X190.02 Y156.94 Z0.25 E49.3484 F210\n"
             "G1 E50.5984 F900\n"
             "G1 X189.99 Y156.82 E50.6028 F1800\n"
             "G1 X190.13 Y156.73 E50.6089\n"
             "G1 X192.56 Y156.73 E50.6978\n"
             "G1 X192.74 Y156.87 E50.7062\n"
             "G1 X192.71 Y156.95 E50.7092\n"
             "G1 X192.52 Y157.09 E50.7179\n"
             "G1 X190.16 Y157.09 E50.8037\n"
             "G1 X190.11 Y157.08 E50.8056\n"
             "G1 X189.9 Y156.94 E50.8148\n"
             "G1 X189.97 Y156.85 E50.8192\n"
             "G1 X190.08 Y156.87 E50.8235\n"
             "G1 X189.69 Y156.81 E50.8235 F9000\n"
             "G1 X189.59 Y156.78 E50.827 F1800\n"
             "G1 X179.83 Y150.81 E51.1947\n"
             "G1 X179.63 Y150.7 E51.2019\n"
             "G1 X179.56 Y150.7 E51.2042\n"
             "G1 X170.48 Y150.7 E51.4961\n"
             "G1 X170.31 Y150.67 E51.5017\n"
             "G1 X170.28 Y150.52 E51.5065\n"
             "G1 X170.27 Y149.48 E51.5401\n"
             "G1 X170.32 Y149.34 E51.5446\n"
             "G1 X170.45 Y149.3 E51.5491\n"
             "G1 X179.6 Y149.3 E51.8432\n"
             "G1 X179.67 Y149.29 E51.8454\n"
             "G1 X189.58 Y143.23 E52.2189\n"
             "G1 X189.73 Y143.18 E52.224\n"
             "G1 E50.974 F2400\n"
             "G1 X189.73 Y143.18 Z0.55 E50.974 F210\n"
             "G1 X189.88 Y156.62 E50.974 F9000\n"
             "G1 X189.88 Y156.62 Z0.25 E50.974 F210\n"
             "G1 E52.224 F900\n"
             "G1 X189.88 Y155.49 E52.2598 F1800\n"
             "G1 X189.86 Y155.39 E52.2633\n"
             "G1 X189.74 Y155.29 E52.2682\n"
             "G1 X181.44 Y150.21 E52.5782\n"
             "G1 X181.32 Y150.12 E52.5828\n"
             "G1 X181.21 Y150 E52.5881\n"
             "G1 X181.41 Y149.81 E52.5968\n"
             "G1 X189.7 Y144.75 E52.906\n"
             "G1 X189.84 Y144.65 E52.9114\n"
             "G1 X189.88 Y144.56 E52.9147\n"
             "G1 X189.88 Y143.54 E52.9472\n"
             "G1 X189.89 Y143.36 E52.9529\n"
             "G1 E51.7029 F2400\n"
             "G1 X189.89 Y143.36 Z0.55 E51.7029 F210\n"
             "G1 X192.83 Y156.68 E51.7029 F9000\n"
             "G1 X192.83 Y156.68 Z0.25 E51.7029 F210\n"
             "G1 E52.9529 F900\n"
             "G1 X192.88 Y156.53 E52.958 F1800\n"
             "G1 X192.88 Y143.47 E53.3824\n"
             "G1 X192.84 Y143.33 E53.3873\n"
             "G1 X192.77 Y143.24 E53.391\n"
             "G1 E52.141 F2400\n"
             "G1 X192.77 Y143.24 Z0.55 E52.141 F210\n"
             "G1 X190.05 Y143.12 E52.141 F9000\n"
             "G1 X190.05 Y143.12 Z0.25 E52.141 F210\n"
             "G1 E53.391 F900\n"
             "G1 X189.94 Y143.04 E53.3961 F1800\n"
             "G1 X190.15 Y142.91 E53.405\n"
             "G1 X192.55 Y142.91 E53.4927\n"
             "G1 X192.68 Y143.01 E53.4987\n"
             "G1 X192.68 Y143.05 E53.5002\n"
             "G1 X192.5 Y143.22 E53.5091\n"
             "G1 X192.49 Y143.27 E53.5111\n"
             "G1 X190.15 Y143.27 E53.5965\n"
             "G1 X190.02 Y143.18 E53.6024\n"
             "G1 X189.93 Y143.06 E53.6079\n"
             "G1 X190.07 Y143.05 E53.6129\n"
             "G1 E52.3629 F2400\n"
             "G1 X190.07 Y143.05 Z0.55 E52.3629 F210\n"
             "G1 X192.85 Y142.82 E52.3629 F9000\n"
             "G1 X192.85 Y142.82 Z0.25 E52.3629 F210\n"
             "G1 E53.6129 F900\n"
             "G1 X192.88 Y142.68 E53.6177 F1800\n"
             "G1 X192.88 Y113.22 E54.5771\n"
             "G1 X192.84 Y113.03 E54.5831\n"
             "G1 X192.7 Y113 E54.5877\n"
             "G1 X163.07 Y113 E55.5526\n"
             "G1 X162.93 Y113.04 E55.5574\n"
             "G1 X162.88 Y113.21 E55.563\n"
             "G1 X162.88 Y119.8 E55.7777\n"
             "G1 X162.85 Y119.94 E55.7822\n"
             "G1 X162.78 Y119.99 E55.7849\n"
             "G1 X162.67 Y120 E55.7886\n"
             "G1 X133.07 Y120 E56.7523\n"
             "G1 X132.94 Y120.04 E56.7568\n"
             "G1 X132.91 Y120.07 E56.7583\n"
             "G1 X132.88 Y120.21 E56.7628\n"
             "G1 X132.88 Y120.79 E56.7817\n"
             "G1 X132.9 Y120.94 E56.7866\n"
             "G1 X132.97 Y120.99 E56.7893\n"
             "G1 X133.08 Y121 E56.7929\n"
             "G1 X162.67 Y121 E57.7565\n"
             "G1 X162.81 Y121.04 E57.761\n"
             "G1 X162.84 Y121.07 E57.7624\n"
             "G1 X162.88 Y121.2 E57.767\n"
             "G1 X162.88 Y122.16 E57.7981\n"
             "G1 X162.93 Y122.39 E57.8057\n"
             "G1 E56.5557 F2400\n"
             "G1 X162.93 Y122.39 Z0.55 E56.5557 F210\n"
             "G1 X163.16 Y180.2 E56.5557 F9000\n"
             "G1 X163.16 Y180.2 Z0.25 E56.5557 F210\n"
             "G1 E57.8057 F900\n"
             "G1 X163.08 Y180.1 E57.8105 F1800\n"
             "G1 X163.17 Y180.03 E57.8147\n"
             "G1 X164.04 Y180.03 E57.8463\n"
             "G1 X164.17 Y180.01 E57.851\n"
             "G1 X164.27 Y179.93 E57.8558\n"
             "G1 X171.98 Y179.93 E58.1371\n"
             "G1 X172.03 Y179.95 E58.1391\n"
             "G1 X172.03 Y180.67 E58.1653\n"
             "G1 X171.16 Y182.01 E58.2236\n"
             "G1 X171.11 Y182.02 E58.2254\n"
             "G1 X170.64 Y182.02 E58.2426\n"
             "G1 X170.65 Y181.98 E58.2443\n"
             "G1 X171.46 Y180.78 E58.297\n"
             "G1 X171.51 Y180.67 E58.3016\n"
             "G1 X171.51 Y180.56 E58.3056\n"
             "G1 X171.44 Y180.45 E58.3102\n"
             "G1 X171.32 Y180.39 E58.3152\n"
             "G1 X171.27 Y180.4 E58.3168\n"
             "G1 X163.29 Y180.4 E58.6078\n"
             "G1 X163.28 Y180.35 E58.6097\n"
             "G1 X163.13 Y180.22 E58.6169\n"
             "G1 X163.07 Y180.13 E58.6209\n"
             "G1 X163.2 Y180.13 E58.6257\n"
             "G1 X164.32 Y180.17 E58.6257 F9000\n"
             "G1 X164.39 Y180.16 E58.6269 F1800\n"
             "G1 X171.35 Y180.16 E58.7538\n"
             "G1 X171.56 Y180.2 E58.7579\n"
             "G1 X171.7 Y180.27 E58.7606\n"
             "G1 X171.75 Y180.43 E58.7638\n"
             "G1 X171.77 Y180.6 E58.7668\n"
             "G1 X171.63 Y180.89 E58.7727\n"
             "G1 X171.03 Y181.81 E58.7927\n"
             "G1 E57.5427 F2400\n"
             "G1 X171.03 Y181.81 Z0.55 E57.5427 F210\n"
             "G1 X162.96 Y180.35 E57.5427 F9000\n"
             "G1 X162.96 Y180.35 Z0.25 E57.5427 F210\n"
             "G1 E58.7927 F900\n"
             "G1 X162.9 Y180.5 E58.7981 F1800\n"
             "G1 X162.88 Y180.64 E58.8027\n"
             "G1 X162.88 Y186.77 E59.0025\n"
             "G1 X162.91 Y186.98 E59.0092\n"
             "G1 X162.92 Y186.97 E59.0098\n"
             "G1 X163.06 Y187 E59.0145\n"
             "G1 X192.64 Y187 E59.9781\n"
             "G1 X192.73 Y187 E59.9813\n"
             "G1 X192.84 Y186.96 E59.9849\n"
             "G1 X192.85 Y186.98 E59.9857\n"
             "G1 X192.88 Y186.82 E59.991\n"
             "G1 X192.88 Y157.31 E60.9522\n"
             "G1 X192.83 Y157.12 E60.9587\n"
             "G1 E59.7087 F2400\n"
             "G1 X192.83 Y157.12 Z1.05 E59.7087 F210\n"
             "G92 E0 \n"
             "G1 E-1.4 F3000\n"
             "G92 E0\n"
             "M104 S0\n"
             "G91\n"
             "G1 Z2 F5000\n"
             "G90\n"
             "G1 X5 Y5 F7800\n"
             "T0\n"
             "M104 T0 S0 \n"
             "G92 E0\n"
             "G1 E-8\n"
             "G1 E-5\n"
             "G92 E0\n"
             "T1\n"
             "M104 T1 S0\n"
             "G92 E0\n"
             "G1 E-8\n"
             "G1 E-5\n"
             "G92 E0\n"
             "T0\n"
             "M140 S0\n"
             "M107\n"
             "M84\n"
             "M4202\n"
#else
             "M140 S55\n"
             "M104 T0 S190\n"
             "M104 T1 S190\n"
             "M117 Homing\n"
             "G91\n"
             "G1 Z5 F7800\n"
             "G90\n"
             "G28 Z\n"
             "M190 S55\n"
             "T0\n"
             "M109 T1 S190\n"
             "G92 E0\n"
             "G1 X1.0 Y243.8 Z0.3 F7800.0\n"
             "G1 X128.5 Y243.8 Z0.3 F1500.0 E15\n"
             "G1 E14.5 F3000\n"
             "G92 E0\n"
             "M104 T1 S160\n"
             "M117 Purge Extruder 1\n"
             "G92 E0\n"
             "G1 X236.0 Y243.2 Z5 F7800.0\n"
             "T1\n"
             "M109 S190\n"
             "G1 X236.0 Y243.2 Z0.3 F3000.0\n"
             "G1 X108.5 Y243.2 Z0.3 F1500.0 E15\n"
             "G1 E14.5 F3000\n"
             "G92 E0\n"
             "G4 P100\n"
             "T1\n"
             "G92 E0\n"
             "G1 X120 Y243.5 F8000\n"
             "M109 S190\n"
             "G92 E0  \n"
             "G1 E2 F100\n"
             "G92 E0\n"
             "G1 X179.04 Y44.16 Z0.45 E0 F9000\n"
             "G1 X179.04 Y44.16 Z0.25 E0 F210\n"
             "G1 E1 F900\n"
             "G1 X178.86 Y44.17 E1.006 F1800\n"
             "G1 X178.76 Y44.21 E1.0098\n"
             "G1 X178.72 Y44.35 E1.0145\n"
             "G1 X178.72 Y73.94 E2.0316\n"
             "G1 X178.71 Y74.05 E2.0353\n"
             "G1 X178.66 Y74.12 E2.0383\n"
             "G1 X178.51 Y74.16 E2.0437\n"
             "G1 X177.91 Y74.16 E2.0642\n"
             "G1 X177.76 Y74.13 E2.0695\n"
             "G1 X177.72 Y73.96 E2.0754\n"
             "G1 X177.72 Y44.43 E3.0902\n"
             "G1 X177.7 Y44.19 E3.0983\n"
             "G1 X177.54 Y44.16 E3.1037\n"
             "G1 X171.36 Y44.16 E3.316\n"
             "G1 X171.19 Y44.21 E3.3221\n"
             "G1 X171.15 Y44.34 E3.3269\n"
             "G1 X171.15 Y73.97 E4.3451\n"
             "G1 X171.11 Y74.11 E4.35\n"
             "G1 X170.95 Y74.16 E4.3556\n"
             "G1 X170.34 Y74.16 E4.3767\n"
             "G1 X170.23 Y74.15 E4.3804\n"
             "G1 X170.17 Y74.09 E4.3834\n"
             "G1 X170.15 Y73.92 E4.3892\n"
             "G1 X170.15 Y44.38 E5.4044\n"
             "G1 X170.11 Y44.2 E5.4106\n"
             "G1 X169.96 Y44.16 E5.4161\n"
             "G1 X163.78 Y44.16 E5.6285\n"
             "G1 X163.61 Y44.2 E5.6345\n"
             "G1 X163.57 Y44.36 E5.64\n"
             "G1 X163.57 Y73.97 E6.6575\n"
             "G1 X163.53 Y74.11 E6.6625\n"
             "G1 X163.37 Y74.16 E6.6685\n"
             "G1 X162.74 Y74.16 E6.6902\n"
             "G1 X162.6 Y74.12 E6.695\n"
             "G1 X162.58 Y73.97 E6.7003\n"
             "G1 X162.57 Y44.35 E7.718\n"
             "G1 X162.53 Y44.2 E7.7235\n"
             "G1 X162.36 Y44.16 E7.7296\n"
             "G1 X156.21 Y44.16 E7.9408\n"
             "G1 X156.08 Y44.19 E7.9456\n"
             "G1 X156.04 Y44.22 E7.9474\n"
             "G1 X156 Y44.35 E7.952\n"
             "G1 X156 Y73.97 E8.9697\n"
             "G1 X155.97 Y74.12 E8.9751\n"
             "G1 X155.8 Y74.16 E8.9811\n"
             "G1 X155.17 Y74.16 E9.0026\n"
             "G1 X155.03 Y74.12 E9.0076\n"
             "G1 X155 Y73.96 E9.013\n"
             "G1 X155 Y44.34 E10.0308\n"
             "G1 X154.97 Y44.19 E10.0362\n"
             "G1 X154.76 Y44.16 E10.0435\n"
             "G1 X148.66 Y44.16 E10.2533\n"
             "G1 X148.55 Y44.17 E10.2568\n"
             "G1 X148.46 Y44.22 E10.2603\n"
             "G1 X148.42 Y44.35 E10.265\n"
             "G1 X148.42 Y73.95 E11.2821\n"
             "G1 X148.41 Y74.05 E11.2858\n"
             "G1 X148.36 Y74.12 E11.2887\n"
             "G1 X148.21 Y74.16 E11.2941\n"
             "G1 X147.61 Y74.16 E11.3147\n"
             "G1 X147.46 Y74.13 E11.3199\n"
             "G1 X147.42 Y73.94 E11.3265\n"
             "G1 X147.42 Y39.35 E12.5152\n"
             "G1 X147.47 Y39.21 E12.5202\n"
             "G1 X147.65 Y39.16 E12.5267\n"
             "G1 X208.84 Y39.16 E14.6294\n"
             "G1 X208.93 Y39.18 E14.6327\n"
             "G1 X208.98 Y39.21 E14.6344\n"
             "G1 X209 Y39.18 E14.6356\n"
             "G1 X209.02 Y39.5 E14.6465\n"
             "G1 X209.02 Y73.95 E15.8303\n"
             "G1 X209.01 Y74.05 E15.8337\n"
             "G1 X208.96 Y74.12 E15.8366\n"
             "G1 X208.83 Y74.16 E15.8414\n"
             "G1 X208.2 Y74.16 E15.8629\n"
             "G1 X208.06 Y74.13 E15.8678\n"
             "G1 X208.02 Y73.95 E15.8742\n"
             "G1 X208.02 Y44.41 E16.8894\n"
             "G1 X208.01 Y44.28 E16.8938\n"
             "G1 X207.97 Y44.19 E16.8972\n"
             "G1 X207.84 Y44.16 E16.902\n"
             "G1 X201.67 Y44.16 E17.1139\n"
             "G1 X201.52 Y44.2 E17.1193\n"
             "G1 X201.46 Y44.26 E17.1222\n"
             "G1 X201.45 Y44.37 E17.1259\n"
             "G1 X201.45 Y73.97 E18.1431\n"
             "G1 X201.41 Y74.11 E18.1479\n"
             "G1 X201.25 Y74.16 E18.1537\n"
             "G1 X200.62 Y74.16 E18.1755\n"
             "G1 X200.51 Y74.13 E18.1793\n"
             "G1 X200.45 Y74.02 E18.1836\n"
             "G1 X200.45 Y44.37 E19.2025\n"
             "G1 X200.41 Y44.24 E19.2073\n"
             "G1 X200.38 Y44.2 E19.209\n"
             "G1 X200.25 Y44.16 E19.2136\n"
             "G1 X194.08 Y44.16 E19.4259\n"
             "G1 X193.91 Y44.2 E19.4319\n"
             "G1 X193.87 Y44.35 E19.4372\n"
             "G1 X193.88 Y73.96 E20.4548\n"
             "G1 X193.84 Y74.1 E20.4595\n"
             "G1 X193.8 Y74.13 E20.4612\n"
             "G1 X193.66 Y74.16 E20.466\n"
             "G1 X193.04 Y74.16 E20.4875\n"
             "G1 X192.9 Y74.12 E20.4923\n"
             "G1 X192.87 Y73.97 E20.4978\n"
             "G1 X192.87 Y44.35 E21.5154\n"
             "G1 X192.83 Y44.2 E21.5209\n"
             "G1 X192.66 Y44.16 E21.527\n"
             "G1 X186.53 Y44.16 E21.7375\n"
             "G1 X186.38 Y44.19 E21.7429\n"
             "G1 X186.34 Y44.22 E21.7447\n"
             "G1 X186.3 Y44.35 E21.7494\n"
             "G1 X186.3 Y44.47 E21.7534\n"
             "G1 X186.3 Y73.97 E22.7671\n"
             "G1 X186.27 Y74.12 E22.7724\n"
             "G1 X186.1 Y74.16 E22.7784\n"
             "G1 X185.5 Y74.16 E22.7988\n"
             "G1 X185.41 Y74.15 E22.8019\n"
             "G1 X185.33 Y74.11 E22.805\n"
             "G1 X185.3 Y73.96 E22.8104\n"
             "G1 X185.3 Y44.34 E23.8281\n"
             "G1 X185.27 Y44.22 E23.8326\n"
             "G1 X185.27 Y44.19 E23.8335\n"
             "G1 X185.09 Y44.16 E23.84\n"
             "G1 X179.07 Y44.16 E24.0467\n"
             "G1 E23.0467 F2400\n"
             "G1 X179.07 Y44.16 Z0.45 E23.0467 F210\n"
             "G1 X107.99 Y77.04 E23.0467 F9000\n"
             "G1 X107.99 Y77.04 Z0.25 E23.0467 F210\n"
             "G1 E24.0467 F900\n"
             "G1 X107.21 Y77.04 E24.0736 F1800\n"
             "G1 X107.11 Y77.05 E24.0769\n"
             "G1 X107.06 Y77.1 E24.0795\n"
             "G1 X107.02 Y77.2 E24.0831\n"
             "G1 X107.02 Y77.83 E24.1047\n"
             "G1 X107.06 Y77.96 E24.1095\n"
             "G1 X107.09 Y78 E24.1111\n"
             "G1 X107.22 Y78.04 E24.1158\n"
             "G1 X136.83 Y78.04 E25.1328\n"
             "G1 X136.99 Y78.07 E25.1382\n"
             "G1 X137.02 Y78.25 E25.1443\n"
             "G1 X137.02 Y84.42 E25.3565\n"
             "G1 X136.98 Y84.58 E25.362\n"
             "G1 X136.8 Y84.61 E25.3684\n"
             "G1 X107.24 Y84.61 E26.3838\n"
             "G1 X107.09 Y84.64 E26.3889\n"
             "G1 X107.04 Y84.7 E26.3917\n"
             "G1 X107.02 Y84.81 E26.3954\n"
             "G1 X107.03 Y85.42 E26.4164\n"
             "G1 X107.06 Y85.55 E26.4211\n"
             "G1 X107.18 Y85.61 E26.4256\n"
             "G1 X107.24 Y85.61 E26.4277\n"
             "G1 X136.83 Y85.61 E27.4439\n"
             "G1 X136.94 Y85.64 E27.4477\n"
             "G1 X136.99 Y85.68 E27.4501\n"
             "G1 X137.02 Y85.83 E27.4554\n"
             "G1 X137.02 Y92 E27.6673\n"
             "G1 X137 Y92.16 E27.6727\n"
             "G1 X136.99 Y92.15 E27.673\n"
             "G1 X136.69 Y92.19 E27.6834\n"
             "G1 X107.23 Y92.19 E28.6954\n"
             "G1 X107.06 Y92.23 E28.7014\n"
             "G1 X107.02 Y92.36 E28.7062\n"
             "G1 X107.02 Y92.98 E28.7274\n"
             "G1 X107.07 Y93.15 E28.7334\n"
             "G1 X107.22 Y93.19 E28.7384\n"
             "G1 X136.82 Y93.19 E29.7553\n"
             "G1 X136.98 Y93.23 E29.761\n"
             "G1 X137.02 Y93.39 E29.7669\n"
             "G1 X137.02 Y99.5 E29.9767\n"
             "G1 X136.99 Y99.74 E29.9848\n"
             "G1 X136.98 Y99.73 E29.9854\n"
             "G1 X136.84 Y99.76 E29.9902\n"
             "G1 X107.22 Y99.76 E31.0075\n"
             "G1 X107.07 Y99.79 E31.0129\n"
             "G1 X107.02 Y99.94 E31.0181\n"
             "G1 X107.02 Y100.55 E31.0391\n"
             "G1 X107.04 Y100.64 E31.0424\n"
             "G1 X107.07 Y100.73 E31.0456\n"
             "G1 X107.2 Y100.76 E31.0501\n"
             "G1 X141.82 Y100.76 E32.2392\n"
             "G1 X141.98 Y100.72 E32.2448\n"
             "G1 X142.02 Y100.54 E32.2513\n"
             "G1 X142.02 Y39.36 E34.3526\n"
             "G1 X141.99 Y39.19 E34.3585\n"
             "G1 X141.83 Y39.16 E34.364\n"
             "G1 X107.22 Y39.16 E35.5529\n"
             "G1 X107.06 Y39.19 E35.5582\n"
             "G1 X107.02 Y39.34 E35.5633\n"
             "G1 X107.02 Y40.01 E35.5863\n"
             "G1 X107.07 Y40.13 E35.5908\n"
             "G1 X107.19 Y40.16 E35.5953\n"
             "G1 X136.83 Y40.16 E36.6134\n"
             "G1 X136.99 Y40.2 E36.6188\n"
             "G1 X137.03 Y40.38 E36.625\n"
             "G1 X137.03 Y46.53 E36.8363\n"
             "G1 X136.99 Y46.66 E36.8411\n"
             "G1 X136.95 Y46.7 E36.8428\n"
             "G1 X136.82 Y46.74 E36.8475\n"
             "G1 X107.28 Y46.74 E37.8623\n"
             "G1 X107.11 Y46.75 E37.8679\n"
             "G1 X107.05 Y46.8 E37.8707\n"
             "G1 X107.02 Y46.92 E37.8749\n"
             "G1 X107.02 Y47.53 E37.8957\n"
             "G1 X107.06 Y47.66 E37.9006\n"
             "G1 X107.09 Y47.7 E37.9021\n"
             "G1 X107.22 Y47.74 E37.9068\n"
             "G1 X136.85 Y47.74 E38.9244\n"
             "G1 X136.98 Y47.77 E38.9293\n"
             "G1 X137.02 Y47.94 E38.9354\n"
             "G1 X137.02 Y54.12 E39.1476\n"
             "G1 X136.99 Y54.26 E39.1525\n"
             "G1 X137 Y54.27 E39.1531\n"
             "G1 X136.8 Y54.31 E39.1598\n"
             "G1 X107.25 Y54.31 E40.1748\n"
             "G1 X107.1 Y54.33 E40.18\n"
             "G1 X107.04 Y54.4 E40.1833\n"
             "G1 X107.02 Y54.51 E40.187\n"
             "G1 X107.03 Y55.11 E40.2077\n"
             "G1 X107.07 Y55.25 E40.2127\n"
             "G1 X107.18 Y55.31 E40.2171\n"
             "G1 X107.24 Y55.31 E40.2191\n"
             "G1 X136.8 Y55.31 E41.2345\n"
             "G1 X136.98 Y55.35 E41.2406\n"
             "G1 X137.03 Y55.53 E41.247\n"
             "G1 X137.02 Y61.7 E41.4589\n"
             "G1 X137 Y61.86 E41.4644\n"
             "G1 X136.83 Y61.89 E41.47\n"
             "G1 X107.22 Y61.89 E42.4871\n"
             "G1 X107.06 Y61.93 E42.493\n"
             "G1 X107.02 Y62.06 E42.4977\n"
             "G1 X107.02 Y62.68 E42.5189\n"
             "G1 X107.07 Y62.85 E42.525\n"
             "G1 X107.21 Y62.89 E42.53\n"
             "G1 X136.78 Y62.89 E43.5455\n"
             "G1 X136.86 Y62.89 E43.5482\n"
             "G1 X136.98 Y62.93 E43.5526\n"
             "G1 X137.02 Y63.11 E43.5589\n"
             "G1 X137.02 Y69.23 E43.7691\n"
             "G1 X137 Y69.38 E43.7743\n"
             "G1 X136.97 Y69.43 E43.7766\n"
             "G1 X136.83 Y69.46 E43.7814\n"
             "G1 X107.22 Y69.46 E44.7984\n"
             "G1 X107.07 Y69.5 E44.8037\n"
             "G1 X107.02 Y69.64 E44.8088\n"
             "G1 X107.02 Y70.24 E44.8295\n"
             "G1 X107.06 Y70.43 E44.8362\n"
             "G1 X107.2 Y70.46 E44.8409\n"
             "G1 X136.84 Y70.46 E45.8589\n"
             "G1 X136.99 Y70.5 E45.8643\n"
             "G1 X137.02 Y70.68 E45.8706\n"
             "G1 X137.03 Y76.83 E46.0818\n"
             "G1 X136.99 Y76.96 E46.0866\n"
             "G1 X136.96 Y77 E46.0883\n"
             "G1 X136.82 Y77.04 E46.093\n"
             "G1 X108.03 Y77.04 E47.0821\n"
             "G1 E46.0821 F2400\n"
             "G1 X108.03 Y77.04 Z0.45 E46.0821 F210\n"
             "G92 E0 \n"
             "G92 E0\n"
             "M104 S0\n"
             "G1 X120 Y243.5 F8000\n"
             "T0\n"
             "M117 Heating Extruder\n"
             "M109 S190\n"
             "G92 E0\n"
             "G1 E1 F900\n"
             "G1 X209.1 Y112.5 Z0.45 E0 F9000\n"
             "G1 X209.1 Y112.5 Z0.25 E0 F210\n"
             "G1 E1 F900\n"
             "G1 X208.98 Y112.48 E1.0045 F1800\n"
             "G1 X208.72 Y112.69 E1.0172\n"
             "G1 X208.31 Y112.89 E1.0348\n"
             "G1 X208.01 Y112.94 E1.0466\n"
             "G1 X207.45 Y112.94 E1.0682\n"
             "G1 X207.01 Y112.86 E1.0855\n"
             "G1 X206.67 Y112.73 E1.0997\n"
             "G1 X206.41 Y112.56 E1.1114\n"
             "G1 X206.19 Y112.37 E1.1226\n"
             "G1 X206.03 Y112.13 E1.1338\n"
             "G1 X205.83 Y111.68 E1.1527\n"
             "G1 X205.72 Y111.24 E1.1701\n"
             "G1 X205.71 Y110.76 E1.1885\n"
             "G1 X205.77 Y110.35 E1.2045\n"
             "G1 X205.86 Y110.09 E1.2153\n"
             "G1 X206.03 Y109.78 E1.2286\n"
             "G1 X206.26 Y109.53 E1.242\n"
             "G1 X206.54 Y109.33 E1.2551\n"
             "G1 X206.88 Y109.18 E1.2695\n"
             "G1 X207.36 Y109.08 E1.2881\n"
             "G1 X207.92 Y109.07 E1.3096\n"
             "G1 X208.39 Y109.13 E1.3282\n"
             "G1 X208.82 Y109.25 E1.3451\n"
             "G1 X209.18 Y109.43 E1.3607\n"
             "G1 X209.44 Y109.62 E1.3731\n"
             "G1 X209.65 Y109.82 E1.3843\n"
             "G1 X209.69 Y109.98 E1.3905\n"
             "G1 X209.65 Y110.58 E1.4135\n"
             "G1 X209.57 Y111.17 E1.4367\n"
             "G1 X209.42 Y111.73 E1.459\n"
             "G1 X209.23 Y112.16 E1.4769\n"
             "G1 X209.15 Y112.29 E1.4828\n"
             "G1 X209 Y112.46 E1.4915\n"
             "G1 X209.03 Y112.57 E1.4959\n"
             "G1 X209.26 Y112.54 E1.4959 F9000\n"
             "G1 X209.29 Y112.65 E1.5005 F1800\n"
             "G1 X209.04 Y112.87 E1.513\n"
             "G1 X208.72 Y113.05 E1.5273\n"
             "G1 X208.28 Y113.19 E1.545\n"
             "G1 X207.82 Y113.23 E1.563\n"
             "G1 X207.29 Y113.22 E1.5833\n"
             "G1 X206.89 Y113.14 E1.5989\n"
             "G1 X206.48 Y112.99 E1.6159\n"
             "G1 X206.12 Y112.78 E1.632\n"
             "G1 X205.88 Y112.54 E1.6449\n"
             "G1 X205.66 Y112.21 E1.6601\n"
             "G1 X205.46 Y111.8 E1.6777\n"
             "G1 X205.35 Y111.37 E1.6949\n"
             "G1 X205.31 Y110.93 E1.712\n"
             "G1 X205.35 Y110.48 E1.7292\n"
             "G1 X205.43 Y110.14 E1.7426\n"
             "G1 X205.55 Y109.85 E1.7549\n"
             "G1 X205.75 Y109.55 E1.7687\n"
             "G1 X206.05 Y109.26 E1.7848\n"
             "G1 X206.42 Y109.03 E1.8016\n"
             "G1 X206.87 Y108.88 E1.8197\n"
             "G1 X207.38 Y108.79 E1.8398\n"
             "G1 X207.98 Y108.79 E1.8628\n"
             "G1 X208.42 Y108.85 E1.8798\n"
             "G1 X208.79 Y108.96 E1.8949\n"
             "G1 X209.02 Y109.06 E1.9045\n"
             "G1 X209.33 Y109.25 E1.9184\n"
             "G1 X209.46 Y109.25 E1.9234\n"
             "G1 X209.57 Y109.18 E1.9282\n"
             "G1 X209.63 Y109.08 E1.9329\n"
             "G1 X209.6 Y108.83 E1.9425\n"
             "G1 X209.51 Y108.34 E1.9615\n"
             "G1 X209.36 Y107.91 E1.9793\n"
             "G1 X209.11 Y107.44 E1.9997\n"
             "G1 X208.82 Y107.06 E2.018\n"
             "G1 X208.52 Y106.8 E2.0334\n"
             "G1 X208.18 Y106.59 E2.049\n"
             "G1 X207.7 Y106.4 E2.0688\n"
             "G1 X207.25 Y106.29 E2.0865\n"
             "G1 X206.73 Y106.26 E2.1064\n"
             "G1 X206.31 Y106.28 E2.1228\n"
             "G1 X205.95 Y106.32 E2.1366\n"
             "G1 X205.93 Y106.3 E2.138\n"
             "G1 X205.93 Y105.05 E2.186\n"
             "G1 X206.08 Y104.86 E2.1953\n"
             "G1 X206.12 Y104.86 E2.197\n"
             "G1 X206.29 Y105.05 E2.2065\n"
             "G1 X206.29 Y105.75 E2.2336\n"
             "G1 X206.35 Y105.9 E2.2398\n"
             "G1 X206.46 Y105.98 E2.245\n"
             "G1 X207 Y105.98 E2.2656\n"
             "G1 X207.45 Y106.02 E2.2831\n"
             "G1 X207.85 Y106.11 E2.2989\n"
             "G1 X208.37 Y106.33 E2.3206\n"
             "G1 X208.74 Y106.54 E2.3369\n"
             "G1 X209.07 Y106.81 E2.3535\n"
             "G1 X209.34 Y107.11 E2.3692\n"
             "G1 X209.6 Y107.56 E2.389\n"
             "G1 X209.84 Y108.17 E2.4144\n"
             "G1 X209.98 Y108.7 E2.4355\n"
             "G1 X210.06 Y109.35 E2.4605\n"
             "G1 X210.08 Y110.11 E2.4898\n"
             "G1 X210.02 Y110.88 E2.5198\n"
             "G1 X209.9 Y111.51 E2.5442\n"
             "G1 X209.74 Y111.98 E2.5635\n"
             "G1 X209.52 Y112.38 E2.5809\n"
             "G1 X209.31 Y112.63 E2.5935\n"
             "G1 X209.19 Y112.61 E2.5981\n"
             "G1 E1.5981 F2400\n"
             "G1 X209.19 Y112.61 Z0.45 E1.5981 F210\n"
             "G1 X206.28 Y104.74 E1.5981 F9000\n"
             "G1 X206.28 Y104.74 Z0.25 E1.5981 F210\n"
             "G1 E2.5981 F900\n"
             "G1 X206.33 Y104.7 E2.6006 F1800\n"
             "G1 X206.53 Y104.66 E2.6075\n"
             "G1 X207.02 Y104.66 E2.6243\n"
             "G1 X207.18 Y104.63 E2.6299\n"
             "G1 X207.22 Y104.47 E2.6356\n"
             "G1 X207.22 Y74.87 E3.6531\n"
             "G1 X207.25 Y74.68 E3.6594\n"
             "G1 X207.26 Y74.7 E3.6602\n"
             "G1 X207.43 Y74.66 E3.6662\n"
             "G1 X207.96 Y74.66 E3.6845\n"
             "G1 X208.19 Y74.7 E3.6924\n"
             "G1 X208.2 Y74.69 E3.6929\n"
             "G1 X208.22 Y74.85 E3.6984\n"
             "G1 X208.22 Y104.46 E4.7163\n"
             "G1 X208.27 Y104.63 E4.7222\n"
             "G1 X208.4 Y104.66 E4.7269\n"
             "G1 X214.97 Y104.66 E4.9526\n"
             "G1 X215.1 Y104.67 E4.9573\n"
             "G1 X215.18 Y104.71 E4.9602\n"
             "G1 X215.2 Y104.69 E4.9611\n"
             "G1 X215.22 Y104.84 E4.9665\n"
             "G1 X215.22 Y134.49 E5.9855\n"
             "G1 X215.19 Y134.62 E5.9903\n"
             "G1 X215.2 Y134.64 E5.991\n"
             "G1 X214.99 Y134.66 E5.9983\n"
             "G1 X184.24 Y134.66 E7.0553\n"
             "G1 X184.15 Y134.64 E7.0587\n"
             "G1 X184.05 Y134.59 E7.0626\n"
             "G1 E6.0626 F2400\n"
             "G1 X184.05 Y134.59 Z0.45 E6.0626 F210\n"
             "G1 X183.86 Y134.35 E6.0626 F9000\n"
             "G1 X183.86 Y134.35 Z0.25 E6.0626 F210\n"
             "G1 E7.0626 F900\n"
             "G1 X183.88 Y134.47 E7.0672 F1800\n"
             "G1 X183.86 Y134.49 E7.0685\n"
             "G1 X183.81 Y134.49 E7.0702\n"
             "G1 X183.68 Y134.31 E7.079\n"
             "G1 X183.68 Y132.01 E7.1675\n"
             "G1 X183.82 Y131.83 E7.1765\n"
             "G1 X183.9 Y131.86 E7.1795\n"
             "G1 X184.04 Y132.05 E7.1886\n"
             "G1 X184.04 Y134.27 E7.2744\n"
             "G1 X183.93 Y134.42 E7.2815\n"
             "G1 X183.9 Y134.44 E7.2828\n"
             "G1 X183.64 Y134.62 E7.2828 F9000\n"
             "G1 X183.44 Y134.66 E7.2897 F1800\n"
             "G1 X172.98 Y134.66 E7.65\n"
             "G1 X172.8 Y134.61 E7.6566\n"
             "G1 E6.6566 F2400\n"
             "G1 X172.8 Y134.61 Z0.45 E6.6566 F210\n"
             "G1 X183.65 Y131.7 E6.6566 F9000\n"
             "G1 X183.65 Y131.7 Z0.25 E6.6566 F210\n"
             "G1 E7.6566 F900\n"
             "G1 X183.48 Y131.51 E7.6651 F1800\n"
             "G1 X178.41 Y123.6 E7.9818\n"
             "G1 X178.35 Y123.52 E7.9852\n"
             "G1 X178.22 Y123.4 E7.9909\n"
             "G1 X178.21 Y123.43 E7.9919\n"
             "G1 X178.13 Y123.48 E7.9951\n"
             "G1 X178.04 Y123.6 E8\n"
             "G1 X172.96 Y131.54 E8.3177\n"
             "G1 X172.88 Y131.64 E8.3218\n"
             "G1 X172.76 Y131.72 E8.3268\n"
             "G1 E7.3268 F2400\n"
             "G1 X172.76 Y131.72 Z0.45 E7.3268 F210\n"
             "G1 X148.12 Y113.68 E7.3268 F9000\n"
             "G1 X148.12 Y113.68 Z0.25 E7.3268 F210\n"
             "G1 E8.3268 F900\n"
             "G1 X148.18 Y113.8 E8.3321 F1800\n"
             "G1 X147.46 Y113.81 E8.3599\n"
             "G1 X146.12 Y112.95 E8.4213\n"
             "G1 X146.1 Y112.9 E8.4233\n"
             "G1 X146.1 Y112.46 E8.4401\n"
             "G1 X146.13 Y112.42 E8.442\n"
             "G1 X147.41 Y113.29 E8.5013\n"
             "G1 X147.5 Y113.31 E8.5049\n"
             "G1 X147.61 Y113.28 E8.5093\n"
             "G1 X147.71 Y113.17 E8.515\n"
             "G1 X147.73 Y113.08 E8.5187\n"
             "G1 X147.73 Y105.05 E8.8277\n"
             "G1 X147.86 Y104.91 E8.8354\n"
             "G1 X147.9 Y104.91 E8.837\n"
             "G1 X148.03 Y105.05 E8.8445\n"
             "G1 X148.09 Y105.06 E8.8467\n"
             "G1 X148.09 Y105.83 E8.8762\n"
             "G1 X148.12 Y105.94 E8.8807\n"
             "G1 X148.2 Y106.06 E8.8861\n"
             "G1 X148.2 Y113.79 E9.1838\n"
             "G1 X148.07 Y113.74 E9.1891\n"
             "G1 X146.32 Y112.78 E9.1891 F9000\n"
             "G1 X146.32 Y112.81 E9.1897 F1800\n"
             "G1 X147.29 Y113.45 E9.2121\n"
             "G1 X147.41 Y113.52 E9.2148\n"
             "G1 X147.54 Y113.56 E9.2173\n"
             "G1 X147.77 Y113.52 E9.2218\n"
             "G1 X147.88 Y113.48 E9.224\n"
             "G1 X147.92 Y113.37 E9.2264\n"
             "G1 X147.96 Y113.1 E9.2316\n"
             "G1 X147.96 Y106.18 E9.3648\n"
             "G1 X147.94 Y106.06 E9.3671\n"
             "G1 X148.04 Y104.75 E9.3671 F9000\n"
             "G1 X148.16 Y104.66 E9.3723 F1800\n"
             "G1 X148.22 Y104.49 E9.3787\n"
             "G1 X148.23 Y74.87 E10.3966\n"
             "G1 X148.27 Y74.72 E10.4018\n"
             "G1 X148.4 Y74.66 E10.4067\n"
             "G1 X149.05 Y74.66 E10.4293\n"
             "G1 X149.15 Y74.69 E10.4328\n"
             "G1 X149.2 Y74.74 E10.4351\n"
             "G1 X149.23 Y74.9 E10.441\n"
             "G1 X149.22 Y104.45 E11.4563\n"
             "G1 X149.26 Y104.59 E11.4612\n"
             "G1 X149.29 Y104.62 E11.4627\n"
             "G1 X149.42 Y104.66 E11.4676\n"
             "G1 X155.38 Y104.66 E11.6724\n"
             "G1 X155.54 Y104.63 E11.678\n"
             "G1 X155.59 Y104.53 E11.6817\n"
             "G1 X155.6 Y104.46 E11.6842\n"
             "G1 X155.6 Y74.87 E12.7013\n"
             "G1 X155.63 Y74.73 E12.706\n"
             "G1 X155.66 Y74.7 E12.7076\n"
             "G1 X155.8 Y74.66 E12.7124\n"
             "G1 X156.41 Y74.66 E12.7333\n"
             "G1 X156.54 Y74.69 E12.7379\n"
             "G1 X156.6 Y74.81 E12.7424\n"
             "G1 X156.6 Y104.46 E13.7614\n"
             "G1 X156.64 Y104.59 E13.7662\n"
             "G1 X156.67 Y104.62 E13.7677\n"
             "G1 X156.8 Y104.66 E13.7724\n"
             "G1 X162.78 Y104.66 E13.9778\n"
             "G1 X162.89 Y104.65 E13.9818\n"
             "G1 X162.96 Y104.58 E13.9851\n"
             "G1 X162.97 Y104.47 E13.9889\n"
             "G1 X162.97 Y74.88 E15.0058\n"
             "G1 X163.01 Y74.7 E15.012\n"
             "G1 X163.16 Y74.66 E15.0175\n"
             "G1 X163.77 Y74.66 E15.0382\n"
             "G1 X163.91 Y74.7 E15.0433\n"
             "G1 X163.97 Y74.81 E15.0476\n"
             "G1 X163.97 Y74.88 E15.05\n"
             "G1 X163.97 Y104.47 E16.067\n"
             "G1 X164.02 Y104.61 E16.072\n"
             "G1 X164.17 Y104.66 E16.0777\n"
             "G1 X170.16 Y104.66 E16.2836\n"
             "G1 X170.26 Y104.64 E16.2868\n"
             "G1 X170.3 Y104.62 E16.2886\n"
             "G1 X170.32 Y104.64 E16.2896\n"
             "G1 X170.35 Y104.43 E16.2966\n"
             "G1 X170.35 Y74.85 E17.3132\n"
             "G1 X170.39 Y74.69 E17.3188\n"
             "G1 X170.54 Y74.66 E17.3243\n"
             "G1 X171.14 Y74.66 E17.3448\n"
             "G1 X171.29 Y74.7 E17.3503\n"
             "G1 X171.34 Y74.81 E17.3543\n"
             "G1 X171.35 Y74.89 E17.357\n"
             "G1 X171.35 Y104.48 E18.374\n"
             "G1 X171.39 Y104.62 E18.3789\n"
             "G1 X171.55 Y104.66 E18.3846\n"
             "G1 X177.55 Y104.66 E18.5906\n"
             "G1 X177.68 Y104.62 E18.5954\n"
             "G1 X177.7 Y104.64 E18.5962\n"
             "G1 X177.72 Y104.4 E18.6043\n"
             "G1 X177.72 Y74.86 E19.6195\n"
             "G1 X177.75 Y74.73 E19.6242\n"
             "G1 X177.82 Y74.67 E19.627\n"
             "G1 X177.93 Y74.66 E19.6307\n"
             "G1 X178.54 Y74.66 E19.6518\n"
             "G1 X178.67 Y74.7 E19.6566\n"
             "G1 X178.72 Y74.85 E19.6619\n"
             "G1 X178.72 Y104.49 E20.6804\n"
             "G1 X178.76 Y104.62 E20.6853\n"
             "G1 X178.93 Y104.66 E20.6912\n"
             "G1 X184.92 Y104.66 E20.8968\n"
             "G1 X185.06 Y104.62 E20.9019\n"
             "G1 X185.1 Y104.44 E20.9083\n"
             "G1 X185.1 Y74.87 E21.9244\n"
             "G1 X185.13 Y74.69 E21.9306\n"
             "G1 X185.27 Y74.66 E21.9355\n"
             "G1 X185.91 Y74.66 E21.9573\n"
             "G1 X186.05 Y74.7 E21.9624\n"
             "G1 X186.1 Y74.86 E21.9681\n"
             "G1 X186.1 Y104.45 E22.985\n"
             "G1 X186.14 Y104.62 E22.991\n"
             "G1 X186.27 Y104.66 E22.9959\n"
             "G1 X192.29 Y104.66 E23.2025\n"
             "G1 X192.39 Y104.63 E23.2061\n"
             "G1 X192.44 Y104.59 E23.2083\n"
             "G1 X192.48 Y104.46 E23.2131\n"
             "G1 X192.47 Y74.83 E24.2314\n"
             "G1 X192.51 Y74.7 E24.236\n"
             "G1 X192.64 Y74.66 E24.2407\n"
             "G1 X193.29 Y74.66 E24.263\n"
             "G1 X193.43 Y74.7 E24.268\n"
             "G1 X193.47 Y74.87 E24.274\n"
             "G1 X193.47 Y104.44 E25.2902\n"
             "G1 X193.51 Y104.6 E25.2957\n"
             "G1 X193.62 Y104.66 E25.2999\n"
             "G1 X193.68 Y104.66 E25.3021\n"
             "G1 X199.48 Y104.66 E25.5016\n"
             "G1 X199.74 Y104.65 E25.5103\n"
             "G1 X199.81 Y104.6 E25.5133\n"
             "G1 X199.85 Y104.47 E25.5179\n"
             "G1 X199.85 Y74.9 E26.5342\n"
             "G1 X199.86 Y74.79 E26.5379\n"
             "G1 X199.89 Y74.7 E26.5413\n"
             "G1 X200.03 Y74.66 E26.5461\n"
             "G1 X200.67 Y74.66 E26.5683\n"
             "G1 X200.81 Y74.7 E26.5732\n"
             "G1 X200.83 Y74.68 E26.5741\n"
             "G1 X200.85 Y75.13 E26.5896\n"
             "G1 X200.85 Y104.45 E27.5972\n"
             "G1 X200.9 Y104.63 E27.6033\n"
             "G1 X201.03 Y104.66 E27.6082\n"
             "G1 X205.7 Y104.66 E27.7685\n"
             "G1 X205.88 Y104.7 E27.7748\n"
             "G1 X205.93 Y104.74 E27.7772\n"
             "G1 E26.7772 F2400\n"
             "G1 X205.93 Y104.74 Z0.45 E26.7772 F210\n"
             "G1 X170.73 Y111.91 E26.7772 F9000\n"
             "G1 X170.73 Y111.91 Z0.25 E26.7772 F210\n"
             "G1 E27.7772 F900\n"
             "G1 X170.81 Y111.94 E27.7801 F1800\n"
             "G1 X170.9 Y112.01 E27.784\n"
             "G1 X170.99 Y112.03 E27.787\n"
             "G1 X171.04 Y112.08 E27.7893\n"
             "G1 X171.12 Y112.11 E27.7922\n"
             "G1 X171.16 Y112.14 E27.7941\n"
             "G1 X171.29 Y112.08 E27.7987\n"
             "G1 X171.44 Y112.06 E27.8038\n"
             "G1 X172.5 Y112.06 E27.8398\n"
             "G1 X172.6 Y112.08 E27.8432\n"
             "G1 X172.7 Y112.18 E27.8477\n"
             "G1 X178.02 Y120.5 E28.1807\n"
             "G1 X178.16 Y120.67 E28.1879\n"
             "G1 X178.2 Y120.67 E28.1893\n"
             "G1 X178.22 Y120.71 E28.1906\n"
             "G1 X178.4 Y120.54 E28.199\n"
             "G1 X183.75 Y112.19 E28.5328\n"
             "G1 X183.83 Y112.08 E28.5373\n"
             "G1 X183.95 Y112.06 E28.5415\n"
             "G1 X185.09 Y112.06 E28.5799\n"
             "G1 X185.3 Y112.12 E28.5872\n"
             "G1 X185.26 Y112.28 E28.5929\n"
             "G1 X185.2 Y112.4 E28.5973\n"
             "G1 X179.1 Y121.94 E28.9787\n"
             "G1 X179.05 Y122.06 E28.9831\n"
             "G1 X179.1 Y122.17 E28.9873\n"
             "G1 X184.96 Y131.34 E29.3539\n"
             "G1 X185 Y131.41 E29.3563\n"
             "G1 X185.04 Y131.59 E29.3625\n"
             "G1 X184.87 Y131.65 E29.3684\n"
             "G1 X184.78 Y131.66 E29.3715\n"
             "G1 X184.24 Y131.66 E29.3896\n"
             "G1 X184.15 Y131.68 E29.3929\n"
             "G1 X184.05 Y131.74 E29.3968\n"
             "G1 E28.3968 F2400\n"
             "G1 X184.05 Y131.74 Z0.45 E28.3968 F210\n"
             "G1 X172.61 Y134.35 E28.3968 F9000\n"
             "G1 X172.61 Y134.35 Z0.25 E28.3968 F210\n"
             "G1 E29.3968 F900\n"
             "G1 X172.61 Y134.48 E29.4018 F1800\n"
             "G1 X172.54 Y134.45 E29.4045\n"
             "G1 X172.41 Y134.27 E29.4131\n"
             "G1 X172.41 Y132.06 E29.4983\n"
             "G1 X172.45 Y132.04 E29.5002\n"
             "G1 X172.58 Y131.89 E29.5078\n"
             "G1 X172.63 Y131.89 E29.5095\n"
             "G1 X172.77 Y132.07 E29.5183\n"
             "G1 X172.77 Y134.29 E29.604\n"
             "G1 X172.63 Y134.47 E29.6126\n"
             "G1 X172.53 Y134.38 E29.6176\n"
             "G1 X172.41 Y134.61 E29.6176 F9000\n"
             "G1 X172.28 Y134.65 E29.6221 F1800\n"
             "G1 X172.11 Y134.66 E29.6279\n"
             "G1 X141.46 Y134.66 E30.681\n"
             "G1 X141.35 Y134.65 E30.6848\n"
             "G1 X141.26 Y134.61 E30.6882\n"
             "G1 X141.22 Y134.47 E30.693\n"
             "G1 X141.22 Y104.9 E31.7092\n"
             "G1 X141.23 Y104.82 E31.7118\n"
             "G1 X141.26 Y104.7 E31.7161\n"
             "G1 X141.45 Y104.66 E31.7225\n"
             "G1 X147.49 Y104.66 E31.9303\n"
             "G1 X147.69 Y104.71 E31.9372\n"
             "G1 X147.74 Y104.75 E31.9395\n"
             "G1 E30.9395 F2400\n"
             "G1 X147.74 Y104.75 Z0.45 E30.9395 F210\n"
             "G1 X172.43 Y131.77 E30.9395 F9000\n"
             "G1 X172.43 Y131.77 Z0.25 E30.9395 F210\n"
             "G1 E31.9395 F900\n"
             "G1 X172.34 Y131.69 E31.9434 F1800\n"
             "G1 X172.2 Y131.66 E31.9484\n"
             "G1 X171.64 Y131.66 E31.9671\n"
             "G1 X171.56 Y131.65 E31.97\n"
             "G1 X171.5 Y131.61 E31.9722\n"
             "G1 X171.37 Y131.6 E31.9767\n"
             "G1 X171.41 Y131.6 E31.9781\n"
             "G1 X171.43 Y131.46 E31.9829\n"
             "G1 X171.48 Y131.35 E31.9868\n"
             "G1 X177.36 Y122.16 E32.3534\n"
             "G1 X177.41 Y122.06 E32.3573\n"
             "G1 X177.36 Y121.95 E32.3613\n"
             "G1 X171.21 Y112.35 E32.7442\n"
             "G1 X171.16 Y112.18 E32.75\n"
             "G1 E31.75 F2400\n"
             "G1 X171.16 Y112.18 Z0.45 E31.75 F210\n"
             "G1 X76.25 Y100.37 E31.75 F9000\n"
             "G1 X76.25 Y100.37 Z0.25 E31.75 F210\n"
             "G1 E32.75 F900\n"
             "G1 X76.33 Y100.47 E32.7551 F1800\n"
             "G1 X76.15 Y100.62 E32.7642\n"
             "G1 X75.31 Y100.62 E32.7965\n"
             "G1 X75.09 Y100.73 E32.8059\n"
             "G1 X67.41 Y100.73 E33.1018\n"
             "G1 X67.37 Y100.69 E33.1037\n"
             "G1 X67.38 Y100.01 E33.13\n"
             "G1 X67.38 Y99.97 E33.1314\n"
             "G1 X68.25 Y98.63 E33.1929\n"
             "G1 X68.75 Y98.63 E33.212\n"
             "G1 X68.76 Y98.66 E33.2133\n"
             "G1 X67.9 Y99.94 E33.2729\n"
             "G1 X67.88 Y100.05 E33.277\n"
             "G1 X67.9 Y100.14 E33.2808\n"
             "G1 X67.99 Y100.24 E33.2859\n"
             "G1 X68.08 Y100.26 E33.2894\n"
             "G1 X76.15 Y100.26 E33.6001\n"
             "G1 X76.34 Y100.41 E33.6098\n"
             "G1 X76.34 Y100.44 E33.6109\n"
             "G1 X76.21 Y100.45 E33.6159\n"
             "G1 E32.6159 F2400\n"
             "G1 X76.21 Y100.45 Z0.45 E32.6159 F210\n"
             "G1 X68.39 Y98.83 E32.6159 F9000\n"
             "G1 X68.39 Y98.83 Z0.25 E32.6159 F210\n"
             "G1 E33.6159 F900\n"
             "G1 X67.71 Y99.85 E33.6394 F1800\n"
             "G1 X67.62 Y100.05 E33.6436\n"
             "G1 X67.64 Y100.22 E33.647\n"
             "G1 X67.7 Y100.39 E33.6504\n"
             "G1 X67.77 Y100.44 E33.6521\n"
             "G1 X68.12 Y100.49 E33.6589\n"
             "G1 X75.03 Y100.49 E33.7919\n"
             "G1 X75.11 Y100.47 E33.7936\n"
             "G1 X76.47 Y100.24 E33.7936 F9000\n"
             "G1 X76.48 Y100.21 E33.7948 F1800\n"
             "G1 X76.56 Y100.02 E33.8018\n"
             "G1 X76.62 Y99.97 E33.8044\n"
             "G1 X76.73 Y99.96 E33.8084\n"
             "G1 X106.33 Y99.96 E34.8256\n"
             "G1 X106.46 Y99.92 E34.8304\n"
             "G1 X106.49 Y99.89 E34.832\n"
             "G1 X106.52 Y99.75 E34.8368\n"
             "G1 X106.52 Y99.17 E34.8568\n"
             "G1 X106.5 Y99.03 E34.8619\n"
             "G1 X106.43 Y98.97 E34.8647\n"
             "G1 X106.32 Y98.96 E34.8685\n"
             "G1 X76.73 Y98.96 E35.8858\n"
             "G1 X76.59 Y98.92 E35.8905\n"
             "G1 X76.56 Y98.89 E35.892\n"
             "G1 X76.52 Y98.76 E35.8969\n"
             "G1 X76.52 Y92.8 E36.1018\n"
             "G1 X76.53 Y92.72 E36.1046\n"
             "G1 X76.57 Y92.62 E36.108\n"
             "G1 X76.72 Y92.59 E36.1132\n"
             "G1 X106.34 Y92.59 E37.1314\n"
             "G1 X106.48 Y92.54 E37.1363\n"
             "G1 X106.52 Y92.38 E37.1421\n"
             "G1 X106.52 Y91.81 E37.1617\n"
             "G1 X106.5 Y91.62 E37.1685\n"
             "G1 X106.25 Y91.59 E37.177\n"
             "G1 X76.69 Y91.59 E38.193\n"
             "G1 X76.56 Y91.55 E38.1979\n"
             "G1 X76.52 Y91.4 E38.203\n"
             "G1 X76.52 Y85.43 E38.4084\n"
             "G1 X76.54 Y85.34 E38.4115\n"
             "G1 X76.58 Y85.25 E38.4148\n"
             "G1 X76.74 Y85.21 E38.4203\n"
             "G1 X106.31 Y85.21 E39.437\n"
             "G1 X106.39 Y85.2 E39.4397\n"
             "G1 X106.49 Y85.17 E39.4432\n"
             "G1 X106.52 Y85.01 E39.4488\n"
             "G1 X106.52 Y84.4 E39.4698\n"
             "G1 X106.5 Y84.24 E39.4753\n"
             "G1 X106.49 Y84.25 E39.476\n"
             "G1 X106.35 Y84.21 E39.4809\n"
             "G1 X76.8 Y84.21 E40.4965\n"
             "G1 X76.68 Y84.21 E40.5006\n"
             "G1 X76.56 Y84.17 E40.5051\n"
             "G1 X76.52 Y84.01 E40.5105\n"
             "G1 X76.53 Y78.05 E40.7155\n"
             "G1 X76.54 Y77.95 E40.7191\n"
             "G1 X76.59 Y77.87 E40.7222\n"
             "G1 X76.74 Y77.84 E40.7275\n"
             "G1 X106.33 Y77.84 E41.7448\n"
             "G1 X106.51 Y77.82 E41.7507\n"
             "G1 X106.49 Y77.8 E41.7516\n"
             "G1 X106.52 Y77.67 E41.7563\n"
             "G1 X106.52 Y77.04 E41.778\n"
             "G1 X106.5 Y76.86 E41.7842\n"
             "G1 X106.49 Y76.87 E41.785\n"
             "G1 X106.28 Y76.84 E41.7925\n"
             "G1 X76.73 Y76.84 E42.8081\n"
             "G1 X76.58 Y76.8 E42.8133\n"
             "G1 X76.53 Y76.7 E42.8171\n"
             "G1 X76.52 Y76.63 E42.8197\n"
             "G1 X76.52 Y70.65 E43.0253\n"
             "G1 X76.56 Y70.51 E43.0303\n"
             "G1 X76.74 Y70.46 E43.0367\n"
             "G1 X106.29 Y70.46 E44.0523\n"
             "G1 X106.41 Y70.45 E44.0563\n"
             "G1 X106.49 Y70.41 E44.0593\n"
             "G1 X106.53 Y70.28 E44.064\n"
             "G1 X106.53 Y69.64 E44.086\n"
             "G1 X106.49 Y69.5 E44.091\n"
             "G1 X106.32 Y69.46 E44.0968\n"
             "G1 X76.73 Y69.46 E45.1139\n"
             "G1 X76.56 Y69.42 E45.1199\n"
             "G1 X76.52 Y69.29 E45.1247\n"
             "G1 X76.52 Y63.28 E45.3311\n"
             "G1 X76.57 Y63.12 E45.3367\n"
             "G1 X76.75 Y63.09 E45.3431\n"
             "G1 X106.3 Y63.09 E46.359\n"
             "G1 X106.46 Y63.05 E46.3645\n"
             "G1 X106.52 Y62.93 E46.3692\n"
             "G1 X106.52 Y62.29 E46.3913\n"
             "G1 X106.5 Y62.17 E46.3953\n"
             "G1 X106.45 Y62.12 E46.3977\n"
             "G1 X106.33 Y62.09 E46.4022\n"
             "G1 X76.77 Y62.09 E47.4183\n"
             "G1 X76.65 Y62.08 E47.4225\n"
             "G1 X76.56 Y62.04 E47.4257\n"
             "G1 X76.52 Y61.92 E47.4302\n"
             "G1 X76.52 Y55.9 E47.6371\n"
             "G1 X76.55 Y55.73 E47.6427\n"
             "G1 X76.56 Y55.75 E47.6433\n"
             "G1 X76.7 Y55.71 E47.6481\n"
             "G1 X106.31 Y55.71 E48.6663\n"
             "G1 X106.46 Y55.67 E48.6716\n"
             "G1 X106.51 Y55.6 E48.6745\n"
             "G1 X106.52 Y55.5 E48.6781\n"
             "G1 X106.52 Y54.9 E48.6988\n"
             "G1 X106.49 Y54.76 E48.7037\n"
             "G1 X106.34 Y54.71 E48.7089\n"
             "G1 X76.71 Y54.71 E49.7274\n"
             "G1 X76.55 Y54.69 E49.7331\n"
             "G1 X76.56 Y54.68 E49.7335\n"
             "G1 X76.52 Y54.39 E49.7437\n"
             "G1 X76.52 Y48.59 E49.9429\n"
             "G1 X76.55 Y48.36 E49.9509\n"
             "G1 X76.57 Y48.37 E49.9516\n"
             "G1 X76.7 Y48.34 E49.9564\n"
             "G1 X106.32 Y48.34 E50.9744\n"
             "G1 X106.47 Y48.3 E50.9799\n"
             "G1 X106.52 Y48.16 E50.9851\n"
             "G1 X106.52 Y47.54 E51.0064\n"
             "G1 X106.49 Y47.38 E51.0119\n"
             "G1 X106.35 Y47.34 E51.0169\n"
             "G1 X76.72 Y47.34 E52.0356\n"
             "G1 X76.55 Y47.31 E52.0412\n"
             "G1 X76.56 Y47.31 E52.0415\n"
             "G1 X76.52 Y47.01 E52.0518\n"
             "G1 X76.52 Y41.2 E52.2515\n"
             "G1 X76.56 Y41 E52.2584\n"
             "G1 X76.71 Y40.96 E52.2638\n"
             "G1 X106.24 Y40.96 E53.2787\n"
             "G1 X106.48 Y40.93 E53.2872\n"
             "G1 X106.52 Y40.78 E53.2926\n"
             "G1 X106.52 Y40.13 E53.3149\n"
             "G1 X106.49 Y40 E53.3195\n"
             "G1 X106.33 Y39.96 E53.3252\n"
             "G1 X76.71 Y39.96 E54.3433\n"
             "G1 X76.59 Y39.93 E54.3479\n"
             "G1 X76.56 Y39.93 E54.3488\n"
             "G1 X76.52 Y39.74 E54.3555\n"
             "G1 X76.52 Y38.63 E54.3938\n"
             "G1 X76.47 Y38.44 E54.4003\n"
             "G1 E53.4003 F2400\n"
             "G1 X76.47 Y38.44 Z0.45 E53.4003 F210\n"
             "G1 X49.38 Y63.02 E53.4003 F9000\n"
             "G1 X49.38 Y63.02 Z0.25 E53.4003 F210\n"
             "G1 E54.4003 F900\n"
             "G1 X49.41 Y63.14 E54.4049 F1800\n"
             "G1 X49.27 Y63.23 E54.4114\n"
             "G1 X46.84 Y63.23 E54.5052\n"
             "G1 X46.66 Y63.09 E54.5141\n"
             "G1 X46.69 Y63.02 E54.5173\n"
             "G1 X46.88 Y62.87 E54.5265\n"
             "G1 X49.24 Y62.87 E54.617\n"
             "G1 X49.29 Y62.89 E54.619\n"
             "G1 X49.5 Y63.02 E54.6287\n"
             "G1 X49.43 Y63.12 E54.6333\n"
             "G1 X49.32 Y63.09 E54.6379\n"
             "G1 X49.71 Y63.16 E54.6379 F9000\n"
             "G1 X49.81 Y63.18 E54.6416 F1800\n"
             "G1 X59.57 Y69.15 E55.0297\n"
             "G1 X59.77 Y69.26 E55.0374\n"
             "G1 X59.84 Y69.26 E55.0397\n"
             "G1 X68.92 Y69.26 E55.3479\n"
             "G1 X69.09 Y69.3 E55.3538\n"
             "G1 X69.12 Y69.44 E55.3588\n"
             "G1 X69.13 Y70.49 E55.3943\n"
             "G1 X69.08 Y70.62 E55.399\n"
             "G1 X68.95 Y70.66 E55.4038\n"
             "G1 X59.8 Y70.66 E55.7143\n"
             "G1 X59.73 Y70.67 E55.7166\n"
             "G1 X49.82 Y76.73 E56.1109\n"
             "G1 X49.67 Y76.78 E56.1162\n"
             "G1 E55.1162 F2400\n"
             "G1 X49.67 Y76.78 Z0.45 E55.1162 F210\n"
             "G1 X49.52 Y63.34 E55.1162 F9000\n"
             "G1 X49.52 Y63.34 Z0.25 E55.1162 F210\n"
             "G1 E56.1162 F900\n"
             "G1 X49.52 Y64.47 E56.154 F1800\n"
             "G1 X49.54 Y64.58 E56.1577\n"
             "G1 X49.66 Y64.68 E56.1629\n"
             "G1 X57.96 Y69.76 E56.4901\n"
             "G1 X58.08 Y69.84 E56.495\n"
             "G1 X58.19 Y69.97 E56.5005\n"
             "G1 X57.99 Y70.16 E56.5097\n"
             "G1 X49.7 Y75.22 E56.8361\n"
             "G1 X49.56 Y75.31 E56.8418\n"
             "G1 X49.52 Y75.4 E56.8453\n"
             "G1 X49.52 Y76.43 E56.8796\n"
             "G1 X49.51 Y76.6 E56.8856\n"
             "G1 E55.8856 F2400\n"
             "G1 X49.51 Y76.6 Z0.45 E55.8856 F210\n"
             "G1 X46.57 Y63.28 E55.8856 F9000\n"
             "G1 X46.57 Y63.28 Z0.25 E55.8856 F210\n"
             "G1 E56.8856 F900\n"
             "G1 X46.52 Y63.43 E56.891 F1800\n"
             "G1 X46.52 Y76.49 E57.3389\n"
             "G1 X46.56 Y76.64 E57.3441\n"
             "G1 X46.63 Y76.73 E57.3481\n"
             "G1 E56.3481 F2400\n"
             "G1 X46.63 Y76.73 Z0.45 E56.3481 F210\n"
             "G1 X49.35 Y76.84 E56.3481 F9000\n"
             "G1 X49.35 Y76.84 Z0.25 E56.3481 F210\n"
             "G1 E57.3481 F900\n"
             "G1 X49.46 Y76.93 E57.3534 F1800\n"
             "G1 X49.25 Y77.05 E57.3628\n"
             "G1 X46.85 Y77.05 E57.4554\n"
             "G1 X46.72 Y76.95 E57.4617\n"
             "G1 X46.72 Y76.91 E57.4633\n"
             "G1 X46.9 Y76.74 E57.4727\n"
             "G1 X46.91 Y76.69 E57.4748\n"
             "G1 X49.25 Y76.69 E57.565\n"
             "G1 X49.38 Y76.78 E57.5712\n"
             "G1 X49.47 Y76.9 E57.5769\n"
             "G1 X49.33 Y76.91 E57.5823\n"
             "G1 X46.55 Y77.14 E57.5823 F9000\n"
             "G1 X46.52 Y77.28 E57.5874 F1800\n"
             "G1 X46.52 Y106.75 E58.6005\n"
             "G1 X46.56 Y106.93 E58.6069\n"
             "G1 X46.7 Y106.96 E58.6117\n"
             "G1 X76.33 Y106.96 E59.6307\n"
             "G1 X76.47 Y106.92 E59.6358\n"
             "G1 X76.52 Y106.76 E59.6417\n"
             "G1 X76.52 Y100.81 E59.8461\n"
             "G1 X76.47 Y100.63 E59.8525\n"
             "G1 E58.8525 F2400\n"
             "G1 X76.47 Y100.63 Z0.45 E58.8525 F210\n"
             "G1 X71.15 Y42.12 E58.8525 F9000\n"
             "G1 X71.15 Y42.12 Z0.25 E58.8525 F210\n"
             "G1 E59.8525 F900\n"
             "G1 X71.04 Y42.09 E59.8569 F1800\n"
             "G1 X70.92 Y42.17 E59.8624\n"
             "G1 X70.53 Y42.16 E59.8773\n"
             "G1 X69.9 Y42.11 E59.9017\n"
             "G1 X69.38 Y42.01 E59.9223\n"
             "G1 X68.8 Y41.84 E59.9457\n"
             "G1 X68.42 Y41.66 E59.9616\n"
             "G1 X68.11 Y41.42 E59.977\n"
             "G1 X67.88 Y41.12 E59.9913\n"
             "G1 X67.68 Y40.7 E60.0091\n"
             "G1 X67.62 Y40.39 E60.0214\n"
             "G1 X67.62 Y39.78 E60.0448\n"
             "G1 X67.68 Y39.39 E60.0602\n"
             "G1 X67.79 Y39.05 E60.0738\n"
             "G1 X67.95 Y38.74 E60.0873\n"
             "G1 X68.19 Y38.45 E60.1016\n"
             "G1 X68.46 Y38.24 E60.1149\n"
             "G1 X68.98 Y38.01 E60.1367\n"
             "G1 X69.39 Y37.91 E60.1529\n"
             "G1 X69.83 Y37.88 E60.17\n"
             "G1 X70.24 Y37.91 E60.1859\n"
             "G1 X70.65 Y38.02 E60.2021\n"
             "G1 X70.99 Y38.19 E60.2169\n"
             "G1 X71.26 Y38.42 E60.2306\n"
             "G1 X71.49 Y38.72 E60.2449\n"
             "G1 X71.66 Y39.06 E60.2598\n"
             "G1 X71.77 Y39.52 E60.2777\n"
             "G1 X71.8 Y40.02 E60.297\n"
             "G1 X71.77 Y40.54 E60.3173\n"
             "G1 X71.67 Y41.03 E60.3366\n"
             "G1 X71.52 Y41.44 E60.3534\n"
             "G1 X71.31 Y41.79 E60.3689\n"
             "G1 X71.06 Y42.07 E60.3835\n"
             "G1 X71.08 Y42.19 E60.3879\n"
             "G1 X74.23 Y41.47 E60.3879 F9000\n"
             "G1 X74.25 Y41.59 E60.3924 F1800\n"
             "G1 X73.95 Y41.85 E60.4078\n"
             "G1 X73.47 Y42.13 E60.4293\n"
             "G1 X72.82 Y42.38 E60.4564\n"
             "G1 X72.33 Y42.51 E60.4758\n"
             "G1 X71.53 Y42.63 E60.5067\n"
             "G1 X70.69 Y42.65 E60.5392\n"
             "G1 X69.9 Y42.59 E60.5695\n"
             "G1 X69.39 Y42.51 E60.5896\n"
             "G1 X68.85 Y42.35 E60.6111\n"
             "G1 X68.44 Y42.17 E60.6285\n"
             "G1 X68.08 Y41.95 E60.6447\n"
             "G1 X67.76 Y41.63 E60.6622\n"
             "G1 X67.53 Y41.29 E60.6778\n"
             "G1 X67.39 Y40.94 E60.6926\n"
             "G1 X67.29 Y40.51 E60.7093\n"
             "G1 X67.26 Y40.01 E60.7287\n"
             "G1 X67.3 Y39.47 E60.7496\n"
             "G1 X67.39 Y39.05 E60.7664\n"
             "G1 X67.55 Y38.64 E60.7831\n"
             "G1 X67.77 Y38.29 E60.7991\n"
             "G1 X68.03 Y38.02 E60.8135\n"
             "G1 X68.42 Y37.76 E60.8314\n"
             "G1 X68.82 Y37.58 E60.8484\n"
             "G1 X69.26 Y37.46 E60.866\n"
             "G1 X69.75 Y37.41 E60.8849\n"
             "G1 X70.21 Y37.43 E60.9027\n"
             "G1 X70.63 Y37.52 E60.9191\n"
             "G1 X71.02 Y37.69 E60.9355\n"
             "G1 X71.33 Y37.91 E60.9502\n"
             "G1 X71.59 Y38.18 E60.9645\n"
             "G1 X71.84 Y38.54 E60.9814\n"
             "G1 X71.99 Y38.92 E60.9972\n"
             "G1 X72.11 Y39.35 E61.0145\n"
             "G1 X72.16 Y39.91 E61.0361\n"
             "G1 X72.14 Y40.48 E61.0581\n"
             "G1 X72.06 Y40.91 E61.0748\n"
             "G1 X71.94 Y41.28 E61.0899\n"
             "G1 X71.84 Y41.51 E61.0993\n"
             "G1 X71.67 Y41.77 E61.1114\n"
             "G1 X71.65 Y41.88 E61.1156\n"
             "G1 X71.68 Y41.99 E61.1199\n"
             "G1 X71.79 Y42.09 E61.1257\n"
             "G1 X71.92 Y42.12 E61.1307\n"
             "G1 X72.5 Y42.02 E61.1535\n"
             "G1 X72.97 Y41.87 E61.1725\n"
             "G1 X73.47 Y41.63 E61.1938\n"
             "G1 X73.85 Y41.37 E61.2115\n"
             "G1 X74.15 Y41.1 E61.2271\n"
             "G1 X74.42 Y40.71 E61.2452\n"
             "G1 X74.62 Y40.3 E61.2631\n"
             "G1 X74.78 Y39.81 E61.2828\n"
             "G1 X74.85 Y39.35 E61.3007\n"
             "G1 X74.86 Y38.73 E61.3246\n"
             "G1 X74.81 Y38.11 E61.3487\n"
             "G1 X74.84 Y38.08 E61.3503\n"
             "G1 X76.15 Y38.08 E61.4009\n"
             "G1 X76.34 Y38.23 E61.4102\n"
             "G1 X76.34 Y38.28 E61.4121\n"
             "G1 X76.14 Y38.44 E61.4221\n"
             "G1 X75.4 Y38.44 E61.4505\n"
             "G1 X75.3 Y38.48 E61.4544\n"
             "G1 X75.23 Y38.55 E61.4584\n"
             "G1 X75.2 Y38.65 E61.4625\n"
             "G1 X75.22 Y39.26 E61.486\n"
             "G1 X75.16 Y39.83 E61.508\n"
             "G1 X75.03 Y40.31 E61.5273\n"
             "G1 X74.81 Y40.82 E61.5487\n"
             "G1 X74.58 Y41.21 E61.5661\n"
             "G1 X74.28 Y41.56 E61.584\n"
             "G1 X74.16 Y41.54 E61.5885\n"
             "G1 X71.36 Y38.24 E61.5885 F9000\n"
             "G1 X71.11 Y38.01 E61.5949 F1800\n"
             "G1 X70.79 Y37.83 E61.602\n"
             "G1 X70.43 Y37.71 E61.6094\n"
             "G1 X70 Y37.65 E61.6176\n"
             "G1 X69.51 Y37.66 E61.6271\n"
             "G1 X69.05 Y37.75 E61.6361\n"
             "G1 X68.62 Y37.91 E61.6451\n"
             "G1 X68.27 Y38.11 E61.6527\n"
             "G1 X68.07 Y38.27 E61.6577\n"
             "G1 X67.92 Y38.45 E61.6623\n"
             "G1 E60.6623 F2400\n"
             "G1 X67.92 Y38.45 Z0.45 E60.6623 F210\n"
             "G1 X76.47 Y38.06 E60.6623 F9000\n"
             "G1 X76.47 Y38.06 Z0.25 E60.6623 F210\n"
             "G1 E61.6623 F900\n"
             "G1 X76.52 Y37.88 E61.6687 F1800\n"
             "G1 X76.52 Y33.19 E61.8299\n"
             "G1 X76.49 Y32.99 E61.837\n"
             "G1 X76.48 Y33 E61.8376\n"
             "G1 X76.34 Y32.96 E61.8426\n"
             "G1 X46.77 Y32.96 E62.8595\n"
             "G1 X46.67 Y32.97 E62.8628\n"
             "G1 X46.56 Y33 E62.8666\n"
             "G1 X46.55 Y32.99 E62.8674\n"
             "G1 X46.52 Y33.15 E62.873\n"
             "G1 X46.52 Y62.65 E63.8873\n"
             "G1 X46.57 Y62.84 E63.8941\n"
             "G1 E62.8941 F2400\n"
             "G1 X46.57 Y62.84 Z0.45 E62.8941 F210\n"
             "G1 X74.31 Y41.21 E62.8941 F9000\n"
             "G1 X74.31 Y41.21 Z0.25 E62.8941 F210\n"
             "G1 E63.8941 F900\n"
             "G1 X74.01 Y41.52 E63.9025 F1800\n"
             "G1 X73.63 Y41.79 E63.9114\n"
             "G1 X73.13 Y42.04 E63.9222\n"
             "G1 X72.59 Y42.22 E63.9331\n"
             "G1 X72.1 Y42.32 E63.9428\n"
             "G1 X71.83 Y42.35 E63.9479\n"
             "G1 X71.62 Y42.33 E63.9521\n"
             "G1 X71.44 Y42.26 E63.9558\n"
             "G1 X71.39 Y42.26 E63.9568\n"
             "G1 X71.34 Y42.28 E63.9578\n"
             "G1 X71.08 Y42.38 E63.9633\n"
             "G1 X70.89 Y42.41 E63.967\n"
             "G1 X70.21 Y42.38 E63.9801\n"
             "G1 X69.62 Y42.31 E63.9915\n"
             "G1 X69.04 Y42.17 E64.0029\n"
             "G1 X68.6 Y42 E64.012\n"
             "G1 X68.32 Y41.85 E64.0183\n"
             "G1 X68.11 Y41.69 E64.0232\n"
             "G92 E0\n"
             "G1 E-1.0000 F3000\n"
             "G91\n"
             "G1 Z2 F5000\n"
             "G90\n"
             "G1 X220 Y243 F7800\n"
             "T0\n"
             "M104 T0 S0\n"
             "G92 E0\n"
             "G1 E-8\n"
             "G1 E-5\n"
             "G92 E0\n"
             "T1\n"
             "M104 T1 S0\n"
             "G92 E0\n"
             "G1 E-8\n"
             "G1 E-5\n"
             "G92 E0\n"
             "T0\n"
             "M140 S0\n"
             "M107\n"
             "M84\n"
             "M117 Print Complete\n"
             "M4202\n"
#endif
);
bool customMCode(GCode *com) {
  switch (com->M) {
  case 4200: // send calibration gcode
    flashSource.executeCommands(calibrationGCode, false, 0);
    break;
  case 4201:
    uid.pushMenu(&ui_exy3, true);
    break;
  case 4202:
    Com::printFLN(PSTR("Calibration Printed"));
    break;
  case 4203: // M4203 X<offset> Y<offset> - Store xy calibration offset manually
    if (com->hasX() && com->hasY()) {
      int32_t xcor = static_cast<int32_t>(
          (Printer::axisStepsPerMM[X_AXIS] * (com->X - 5)) / 10);
      int32_t ycor = static_cast<int32_t>(
          (Printer::axisStepsPerMM[Y_AXIS] * (com->Y - 5)) / 10);
      Com::printF(PSTR(" xOffset_before:"), extruder[1].xOffset, 3);
#ifdef TEC4
      extruder[1].xOffset += xcor;
#else
      extruder[1].xOffset -= xcor;
#endif
      Com::printF(PSTR(" xCor:"), xcor, 3);
      Com::printF(PSTR(" xOffset_after:"), extruder[1].xOffset, 3);
      Com::printF(PSTR(" yOffset_before:"), extruder[1].yOffset, 3);
      extruder[1].yOffset += ycor;
      Com::printF(PSTR(" yCor:"), ycor, 3);
      Com::printFLN(PSTR(" yOffset_after:"), extruder[1].yOffset, 3);
      if (xcor != 0 || ycor != 0)
        EEPROM::storeDataIntoEEPROM(false);
    }
    break;
#ifdef HALFAUTOMATIC_LEVELING
  case 4204:
    halfautomaticLevel1GC();
    break;
  case 4205:
    halfautomaticLevel2GC();
    break;
  case 4206:
    halfautomaticLevel3GC();
    break;
#endif
#ifdef ZPROBE_HEIGHT_ROUTINE
  case 4207:
    cZPHeight1(false);
    break;
  case 4208:
    cZPHeight2(false);
    break;
#endif
  case 4210:
    reportPrintStatus(); // Report if printer is moving or not and if all
                         // heaters are off.
    break;
  case 4211: // M4211 Z0.2 change second extruder offset by x mm
  {
    if (com->hasZ()) {
      extruder[1].zOffset += com->Z * Printer::axisStepsPerMM[Z_AXIS];
#if EEPROM_MODE != 0
      EEPROM::storeDataIntoEEPROM(false);
#endif
    }
  } break;
  default:
    return false;
  }
  return true;
}
