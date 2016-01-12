#include "Lighting.h"
#include "Extruder.h"
#include "Printer.h"

Lighting::Lighting()
{
	CurrentShow		= 0;
	CurrentShowStep	= 0;
	UpdateNeeded	= false;
	BedTarget		= 70; //we initialize this to some temerature to be considered hot, to detect hot-to-touch bed right after boot
	BedCurrent		= 0;
	ExtruderTarget	= 160;
	ExtruderCurrent	= 0;
	ThisStep = LED_LOOP_DEVIDER+1;
}
void Lighting::init()
{
	//factoryTest();
	LED.setOutput(BED_LED_PIN); 
	LED.setColorOrderGRB(); 

	//smooth fade in to blue to avoid instant turn-on. total time of this blocking code is 250ms. worth it.
	//slower/longer fade would cause problems to boot and/or connect host software
	SetAllLeds(0, 0, 0);
	if (EEPROM_MODE > 0)
		LedBrightness = EEPROM::bedLedBrightness();
	else
		LedBrightness = LED_MAX_RELATIVE_BRIGHTNESS;
	if (LedBrightness>0.0)
	for (int i = 0; i < 255; i++)
	{
		SetAllLeds(0, 0, i);
		delay(1);
	}
}

void Lighting::factoryTest(){
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLedInstantly(i, 0, 0, 0);
		delay(20); // Wait (ms)
	}
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLedInstantly(i, 255, 0, 0);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLedInstantly(i, 0, 255, 0);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLedInstantly(i, 0, 0, 255);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLedInstantly(i, 255, 255, 255);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	SetAllLeds(0, 0, 0);
}
void Lighting::loop()
{
	if(PrintLine::hasLines())
		return;
	///===This part is ment to avoid pausing interrupts when it could cause problems
	ThisStep++;
	if (ThisStep <LED_LOOP_DEVIDER) return; //only update leds every x loops
	ThisStep = 0;

	if (LastPositionHash != Printer::stepNumber)//do not update leds if there has been any head movement since last loop
	{
		LastPositionHash = Printer::stepNumber;
		return;
	}
	///===
	if (Printer::isZProbingActive())
		return;
	//Update EEPROM
	if (LedBrightness != EEPROM::bedLedBrightness())
	HAL::eprSetFloat(EPR_BED_LED_BRIGHTNESS, LedBrightness);
	
	if (!(LedBrightness>0.0)) //avoid processing if relative brightness set to 0
	{
		SetAllLeds(0, 0, 0);
		return;
	}	
	//Avoid interruptions
	Printer::setZProbingActive(true);
		
	SetShowType(ShowTemperatures); //temorary - to test bed heating with leds
	
	switch (CurrentShow) {
	case Off:
		SetAllLeds(0, 0, 0);
		break;
	case SolidRed:
		SetAllLeds(255, 0, 0);
		break;
	case SolidGreen:
		SetAllLeds(0, 255, 0);
		break;
	case SolidBlue:
		SetAllLeds(0, 0, 255);
		break;
	case ShowTemperatures:
		ShowTemps();
		break;
	}
	Printer::setZProbingActive(false);
}
void Lighting::ShowTemps()
{
	if(PrintLine::hasLines()) return;
	BedCurrent			= Extruder::getHeatedBedTemperature();
	if (BedCurrent < 35 && Extruder::getHeatedBedTargetTemperature() < 35) BedCurrent = 0;
	ExtruderCurrent		= Extruder::current->tempControl.currentTemperatureC;
	if (ExtruderCurrent < 40 && Extruder::current->tempControl.targetTemperatureC < 40) ExtruderCurrent = 0;

	//these checks for 0 enable non-interupted correct-colored lighting throughout cooldown process
	if (Extruder::getHeatedBedTargetTemperature()>0) 
		BedTarget		= Extruder::getHeatedBedTargetTemperature();
	else
		BedTarget		= 80;
	if (Extruder::current->tempControl.targetTemperatureC>0)
		ExtruderTarget	= Extruder::current->tempControl.targetTemperatureC;
	else
		ExtruderTarget	= 170;

	//bed leds (all except middle one (5th)
	int b = (BedCurrent) * 255 / (BedTarget);
	const uint8_t reductor = 3;
	if (b>255) b = 255;
	if (b<0) b = 0;
	int bg = 0;
	int bb = 0;
	//make blue more visible
	if (b < 115) {
		bg = (255 - b) / reductor;
		bb = 255 - b;
		b = b / reductor * 0.5;
	}
	else  {
		bg = 255 - b;
		bb = (255 - b) / reductor;
	}

	//extruder led (5th)
	int e = (ExtruderCurrent)* 255 / (ExtruderTarget);
	if (e>255) e = 255;
	if (e<0) e = 0;
	int eg = 0;
	int eb = 0;
	//make blue more visible
	if (e < 100) {
		eg = (255 - e) / reductor;
		eb = 255 - e;
		e = e / reductor * 0.5;
	}
	else  {
		eg = 255 - e;
		eb = (255 - e) / reductor;
	}

	ary[LED_EXTRUDER][0] = 255;
	ary[LED_EXTRUDER][1] = 255;
	ary[LED_EXTRUDER][2] = 255;
	if (b < 1)
		SetAllBedLeds(255, 255, 255);
	else
		SetAllBedLeds(b, bg, bb);
		ary[LED_EXTRUDER][0] = b;
		ary[LED_EXTRUDER][1] = bg;
		ary[LED_EXTRUDER][2] = bb;
	
	if (e <  1)	 {
		SetLed(LED_EXTRUDER, 255, 255, 255);
	}
	else 	   {
		SetLed(LED_EXTRUDER, e, eg, eb);
		ary[LED_EXTRUDER][0] = e;
		ary[LED_EXTRUDER][1] = eg;
		ary[LED_EXTRUDER][2] = eb;
	}
		
	CommitLeds();

}
void Lighting::SetShowType(ShowType SType)
{
	CurrentShow = SType;
	CurrentShowStep = 0;
	UpdateNeeded = true;
}

void Lighting::SetAllLeds(uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		SetLed(i, r, g, b);
		ary[i][0] = r;
		ary[i][1] = g;
		ary[i][2] = b;
	}
	CommitLeds();
}
void Lighting::SetAllBedLeds(uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		if (!(i==LED_EXTRUDER)) {
			SetLed(i, r, g, b);
			ary[i][0] = r;
			ary[i][1] = g;
			ary[i][2] = b;
		}
	}
}

//Low level wrappers
void Lighting::SetLed(uint8_t i, uint8_t r, uint8_t g, uint8_t b)
{
	LED.set_crgb_at(i,
		r*LedBrightness,
		g*LedBrightness,
		b*LedBrightness);
}
void Lighting::SetLedInstantly(uint8_t i, uint8_t r, uint8_t g, uint8_t b)
{
	SetLed(i, r, g, b);
	CommitLeds();
}
void Lighting::CommitLeds()
{
	LED.sync(); // Sends the data to the LEDs
}

Lighting Light = Lighting();




