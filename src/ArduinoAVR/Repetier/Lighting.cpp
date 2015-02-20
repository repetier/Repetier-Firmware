#include "Lighting.h"
#include "Extruder.h"

Lighting::Lighting()
{
	//LED =  WS2812();
	CurrentShow=0;
	CurrentShowStep=0;
	UpdateNeeded=false;
	BedTarget=0;
	BedCurrent=0;
}
//enum ShowType {
//	Off,
//	SolidRed,
//	SolidBlue,
//	SolidGreen,
//	FixedRGB,
//	BedTempDynamic
//};
void Lighting::init()
{
	
	//factoryTest();
	
	LED.setOutput(BED_LED_PIN); 
	LED.setColorOrderGRB(); 


	SetAllLeds(0, 0, 0);
}
void Lighting::SetAllLeds(uint8_t r, uint8_t g, uint8_t b)
{
	WS2812 LED = WS2812(LED_COUNT); LED.setOutput(BED_LED_PIN); LED.setColorOrderGRB(); //todo: fix this asap

	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, r, g, b);
		LED.sync(); // Sends the data to the LEDs
	}
}
void Lighting::factoryTest(){
	WS2812 LED = WS2812(LED_COUNT); LED.setOutput(BED_LED_PIN); LED.setColorOrderGRB(); //todo: fix this asap

	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 0, 0, 0);
		delay(20); // Wait (ms)
		LED.sync(); // Sends the data to the LEDs
	}
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 255, 0, 0);
		delay(20); // Wait (ms)
		LED.sync(); // Sends the data to the LEDs
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 0, 255, 0);
		delay(20); // Wait (ms)
		LED.sync(); // Sends the data to the LEDs
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 0, 0, 255);
		delay(20); // Wait (ms)
		LED.sync(); // Sends the data to the LEDs
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 255, 255, 255);
		delay(20); // Wait (ms)
		LED.sync(); // Sends the data to the LEDs
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, 0, 0, 0);

	}
	LED.sync(); // Sends the data to the LEDs
}
void Lighting::loop()
{
	SetShowType(BedTempDynamic); //temorary - to test bed heating with leds
	
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
	case BedTempDynamic:
		//ShowBedTemp(Extruder::getHeatedBedTargetTemperature(), Extruder::getHeatedBedTemperature());
		//ShowBedTemp();
		break;
	}
}
void Lighting::ShowBedTemp()
{
	BedCurrent = Extruder::getHeatedBedTemperature();
	

	int r = (BedCurrent) * 255 / (BedTarget);
	if (r>255) r = 255;
	for (int i = 0; i < LED_COUNT; i++)
	{
		LED.set_crgb_at(i, r*LED_MAX_RELATIVE_BRIGHTNESS, 0, (255 - r)*LED_MAX_RELATIVE_BRIGHTNESS);
		LED.sync(); // Sends the data to the LEDs
	}



}
void Lighting::SetShowType(ShowType SType)
{
	CurrentShow = SType;
	CurrentShowStep = 0;
	UpdateNeeded = true;
}

Lighting Light = Lighting();




