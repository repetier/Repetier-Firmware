#include "Lighting.h"
#include "Extruder.h"

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
	///===This part is ment to avoid pausing interrupts when it could cause problems
	ThisStep++;
	if (ThisStep <LED_LOOP_DEVIDER) return; //only update leds every x loops
	ThisStep = 0;

	if (LastPositionHash != Printer::stepNumber)//do not update leds if there has been any head movement since last loop
	{
		LastPositionHash != Printer::stepNumber;
		return;
	}
	///===
	if (EEPROM_MODE > 0)
		LedBrightness = EEPROM::bedLedBrightness();
	else
		LedBrightness = LED_MAX_RELATIVE_BRIGHTNESS;
		
	if (!(LedBrightness>0.0)) //avoid processing if relative brightness set to 0
	{
		SetAllLeds(0, 0, 0);
		return;
	}

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
}
void Lighting::ShowTemps()
{
	BedCurrent			= Extruder::getHeatedBedTemperature();
	ExtruderCurrent		= Extruder::current->tempControl.currentTemperatureC;

	//these checks for 0 enable non-interupted correct-colored lighting throughout cooldown process
	if (Extruder::getHeatedBedTargetTemperature()>0) 
		BedTarget		= Extruder::getHeatedBedTargetTemperature();
	if (Extruder::current->tempControl.targetTemperatureC>0)
		ExtruderTarget	= Extruder::current->tempControl.targetTemperatureC;

	//bed leds (all except middle one (5th)
	int b = (BedCurrent) * 255 / (BedTarget);
	if (b>255) b = 255;
	if (b<0) b = 0;

	//extruder led (5th)
	int e = (ExtruderCurrent)* 255 / (ExtruderTarget);
	if (e>255) e = 255;
	if (e<0) e = 0;

	SetAllBedLeds(b, 0, (255 - b));
	SetLed(LED_EXTRUDER, e, 0, (255 - e));
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
	}
	CommitLeds();
}
void Lighting::SetAllBedLeds(uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		if (!(i==LED_EXTRUDER))SetLed(i, r, g, b);
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




