//Module for managing lighting

#ifndef _Lighting_h
#define _Lighting_h

#include "Repetier.h"
#include "WS2812.h"
//#include "Extruder.h"

#define LED_COUNT 12
#define LED_LOOP_DEVIDER 30
#define LED_EXTRUDER 5

#define LED_MAX_RELATIVE_BRIGHTNESS 0.25
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
		 ShowTemperatures
	 };
	 int  ThisStep;
	 int  CurrentShow;
	 int  CurrentShowStep;
	 bool UpdateNeeded;
	 float BedTarget;
	 float BedCurrent;
	 float ExtruderTarget;
	 float ExtruderCurrent;
	 void init();
	 void SetAllLeds(uint8_t r, uint8_t g, uint8_t b);
	 void SetAllBedLeds(uint8_t r, uint8_t g, uint8_t b);
	 void SetLed(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
	 void SetLedInstantly(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
	 void CommitLeds();
	 void factoryTest();
	 void loop();
	 void ShowTemps();
	 void SetShowType(ShowType SType);
	
};

extern Lighting Light;

#endif
