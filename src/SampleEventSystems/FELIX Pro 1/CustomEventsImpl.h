void Felix100MS() {
  if(PrintLine::linesCount == 0) {
    TemperatureController *bed = tempController[NUM_TEMPERATURE_LOOPS-1];
    if(bed->currentTemperatureC < MIN_DEFECT_TEMPERATURE) {
      bed->flags |= TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
      if(!Printer::isAnyTempsensorDefect())
      {
          Printer::setAnyTempsensorDefect();
          reportTempsensorError();
      }
    }
    
    // Test if bed is back
    if((bed->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) == TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT && bed->currentTemperatureC > 0) {
      bed->flags &= ~TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
      Printer::debugReset(8);
      Printer::unsetAnyTempsensorDefect();
    }
  }
}

bool probeValueNew = false;
bool probeValueOld;
char probeMessageOld[22];

void Felix500MS() {
  if(PrintLine::linesCount == 0) {
     Endstops::update(); // need to update endstops
     Endstops::update();
     probeValueNew = Endstops::zProbe();
     // check if something is changed with probe
     if (probeValueNew != probeValueOld){
        probeValueOld = probeValueNew;
        // probe is triggered, take action
        if(probeValueNew) {
          // store old message
          strncpy(probeMessageOld,uid.statusMsg,21);
          // make screen update with new message
          UI_STATUS_UPD_F(PSTR("Z sensor triggered!"));
        } else{ 
          //probe is not triggered;
          // restore old message
           UI_STATUS_RAM(probeMessageOld);
        }
     }
  }
}

void FelixContainCoordinates() {
  TemperatureController *bed = tempController[NUM_TEMPERATURE_LOOPS-1];
  if((bed->flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) == TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT) {
    Printer::destinationSteps[X_AXIS] = Printer::currentPositionSteps[X_AXIS];
    Printer::destinationSteps[Y_AXIS] = Printer::currentPositionSteps[Y_AXIS];
    Printer::destinationSteps[Z_AXIS] = Printer::currentPositionSteps[Z_AXIS];
    Printer::currentPositionSteps[E_AXIS] = Printer::destinationSteps[E_AXIS];  // to prevent fast e move when reactivated
    Printer::updateCurrentPosition(true);
  }
}
 
