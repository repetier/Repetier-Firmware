//Module for managing lighting

#ifndef _Lighting_h
#define _Lighting_h

#include "Repetier.h"
#include "WS2812.h"
//#include "Extruder.h"

//#define LED_PIN	46    // Digital IO pin connected to the NeoPixels.
#define LED_COUNT 12
#define LED_MAX_RELATIVE_BRIGHTNESS 0.3
#define LED_BASE_TEMP 30

class Lighting
{
 public:
	 Lighting();

	 WS2812 LED;
	 enum ShowType {
		 Off,
		 SolidRed,
		 SolidBlue,
		 SolidGreen,
		 FixedRGB,
		 BedTempDynamic
	 };
	 int  CurrentShow;
	 int  CurrentShowStep;
	 bool UpdateNeeded;
	 float BedTarget;
	 float BedCurrent;
	 void init();
	 void SetAllLeds(uint8_t r, uint8_t g, uint8_t b);
	  void factoryTest();
	 void loop();
	 void ShowBedTemp();
	 void SetShowType(ShowType SType);
	
};

extern Lighting Light;

#endif
