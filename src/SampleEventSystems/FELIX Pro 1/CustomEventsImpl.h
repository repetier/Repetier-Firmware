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
