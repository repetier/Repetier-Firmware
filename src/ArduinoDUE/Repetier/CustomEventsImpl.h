void ReportDestinationCheck() {
  if (Printer::isNoDestinationCheck() == 0)
    Com::printInfoFLN(PSTR("Destination check enabled."));
  else
    Com::printInfoFLN(PSTR("Destination check disabled."));
}

void DisableDestinationCheck() {
  Printer::setNoDestinationCheck(true);
  ReportDestinationCheck();
}

void EnableDestinationCheck() {
  Printer::setNoDestinationCheck(false);
  ReportDestinationCheck();
}

bool CustomMCodeHandler(GCode *com)
{
  switch (com->M)
  {
    case 700:
      ReportDestinationCheck();
      return true;
    case 701:
      DisableDestinationCheck();
      return true;
    case 702:
      EnableDestinationCheck();
      return true;
    default:
      return false;
  }
}
