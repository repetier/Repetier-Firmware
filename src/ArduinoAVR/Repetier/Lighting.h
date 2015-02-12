//Module for managing lighting
//Not being included for now due to errors



#ifndef _LightingClass_h
#define _LightingClass_h
#include <Adafruit_NeoPixel\Adafruit_NeoPixel.h>
class Adafruit_NeoPixel;
//#include "Repetier.h"
#define PIXEL_PIN	46    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 12



class LightingClass : public Adafruit_NeoPixel
{
 public:
	 LightingClass() : Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800){}
	 enum ShowType {
		 Off,
		 SolidRed,
		 SolidBlue,
		 SolidGreen,
		 FixedRGB,
		 BedTempDynamic
	 };
	 //Adafruit_NeoPixel strip;
	 int  CurrentShow;
	 int  CurrentShowStep;
	 bool UpdateNeeded;
	 void init()
	 {
		 //strip = s;
		 this->begin();
		 this->show(); // Initialize all pixels to 'off'
		 for (size_t i = 0; i < 255; i++)
		 {
			 ShowBedTemp(255, i);
			 delay(10);
		 }

	 }
	 void loop()
	 {
		 switch (this->CurrentShow) {
		 case Off:
			 this->show();
			 break;
		 case SolidRed:
			 SetAllPixelColor(255, 0, 0);
			 break;
		 case SolidGreen:
			 SetAllPixelColor(0, 255, 0);
			 break;
		 case SolidBlue:
			 SetAllPixelColor(0, 0, 255);
			 break;
		 case BedTempDynamic:
			 //ShowBedTemp(heatedBedController.targetTemperatureC, heatedBedController.currentTemperatureC);
			 break;
		 }
	 }
	 void ShowBedTemp(int TargetTemp, int FactTemp)
	 {
		 for (int i = 0; i < this->numPixels(); i = i + 1)
		 {
			 //SetAllPixelColor(FactTemp / TargetTemp * 255, 0, (TargetTemp - FactTemp) / TargetTemp * 255);
		 }
	 }
	 void SetShowType(ShowType SType)
	 {
		 CurrentShow = SType;
		 CurrentShowStep = 0;
		 UpdateNeeded = true;
	 }
	 void SetAllPixelColor(float r, float g, float b)
	 {
		 for (int i = 0; i < this->numPixels(); i = i + 3)
		 {
			 this->setPixelColor(i, r, g, b);
		 }
	 }
};

extern LightingClass Light = LightingClass() ;


#endif
