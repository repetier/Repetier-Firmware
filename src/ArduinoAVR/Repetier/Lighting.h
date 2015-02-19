//Module for managing lighting
//Not being included for now due to errors
//http://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/

//#include "Repetier.h"

#ifndef _Lighting_h
#define _Lighting_h

#define LED_PIN	46    // Digital IO pin connected to the NeoPixels.
#define NUM_LEDS 12
#define LedBrightness 0.3
#define BaseTemp 30

//extern Adafruit_NeoPixel Pixels;

class Lighting
{
#define PIXELS NUM_LEDS // Number of pixels in the string

	// These values depend on which pin your string is connected to and what board you are using
	// More info on how to find these at http://www.arduino.cc/en/Reference/PortManipulation

	// These values are for digital pin 8 on an Arduino Yun or digital pin 12 on a DueMilinove
	// Note that you could also include the DigitalWriteFast header file to not need to to this lookup.

#define PIXEL_PORT PORTL // Port of the pin the pixels are connected to
#define PIXEL_DDR DDRL // Port of the pin the pixels are connected to
#define PIXEL_BIT 3 // Bit of the pin the pixels are connected to

#define T1H  2000    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  350    // Width of a 0 bit in ns
#define T0L  800    // Width of a 0 bit in ns

#define RES 50000    // Width of the low gap between bits to cause a frame to latch

	// Here are some convenience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L) // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

#define DELAY_CYCLES(n) ( ((n)>0) ? __builtin_avr_delay_cycles( n ) : __builtin_avr_delay_cycles( 0 ) ) // Make sure we never have a delay less than zero

	// Actually send a bit to the string. We turn off optimizations to make sure the compile does
	// not reorder things and make it so the delay happens in the wrong place.

	

	static void sendBit(bool bitVal)__attribute__((optimize(0))) {

		if (bitVal) {      // 1-bit

			bitSet(PIXEL_PORT, PIXEL_BIT);

			DELAY_CYCLES(NS_TO_CYCLES(T1H) - 2); // 1-bit width less overhead for the actual bit setting
			// Note that this delay could be longer and everything would still work
			bitClear(PIXEL_PORT, PIXEL_BIT);

			DELAY_CYCLES(NS_TO_CYCLES(T1L) - 10); // 1-bit gap less the overhead of the loop

		}
		else {             // 0-bit

			cli();                                       // We need to protect this bit from being made wider by an interrupt 

			bitSet(PIXEL_PORT, PIXEL_BIT);

			DELAY_CYCLES(NS_TO_CYCLES(T0H) - 2); // 0-bit width less overhead
			// **************************************************************************
			// This line is really the only tight goldilocks timing in the whole program!
			// **************************************************************************
			bitClear(PIXEL_PORT, PIXEL_BIT);

			sei();

			DELAY_CYCLES(NS_TO_CYCLES(T0L) - 10); // 0-bit gap less overhead of the loop

		}

		// Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
		// Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
		// This has thenice side effect of avoid glitches on very long strings becuase

	}
	static void sendByte(unsigned char byte)__attribute__((optimize(0))) {

		for (unsigned char bit = 0; bit < 8; bit++) {

			sendBit(bitRead(byte, 7)); // Neopixel wants bit in highest-to-lowest order
			// so send highest bit (bit #7 in an 8-bit byte since they start at 0)
			byte <<= 1; // and then shift left so bit 6 moves into 7, 5 moves into 6, etc

		}
	}

	/*

	The following three functions are the public API:
	ledSetup() - set up the pin that is connected to the string. Call once at the beginning of the program.
	sendPixel( r , g , b ) - send a single pixel to the string. Call this once for each pixel in a frame.
	show() - latch the recently sent pixels on the LEDs . Call once per frame.
	*/

	// Set the specified pin up as digital out

	static void ledsetup()__attribute__((optimize(0))) {

		bitSet(PIXEL_DDR, PIXEL_BIT);

	}

	static void sendPixel(unsigned char r, unsigned char g, unsigned char b)__attribute__((optimize(0))) {

		sendByte(g); // Neopixel wants colors in green-then-red-then-blue order
		sendByte(r);
		sendByte(b);

	}

	// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame

	static void show()__attribute__((optimize(0))) {
		DELAY_CYCLES(NS_TO_CYCLES(RES));
	}

 public:

	 enum ShowType {
		 Off,
		 SolidRed,
		 SolidBlue,
		 SolidGreen,
		 FixedRGB,
		 BedTempDynamic
	 };

	 static int  CurrentShow;
	 static int  CurrentShowStep;
	 static bool UpdateNeeded;

	 static void init() 
	 {
		 ledsetup();
		 //factoryTest();
		 for (int k = 0; k < NUM_LEDS; k++)
		 {
			 sendPixel(0, 0, 0);
		 }
		 show();
		 //int i = 0;

		 //for (int k = 0; k < 100; k++)
		 //{
			// while (i < 255){
			//	 i++;
			//	 sendPixel(i, 0, 0);
			//	 sendPixel(0, i, 0);
			//	 sendPixel(0, 0, i);
			//	 sendPixel(255 - i, 255 - i, 255 - i);
			//	 sendPixel(255 - i, i, 255 - i);
			//	 sendPixel(i, 255 - i, i);
			//	 sendPixel(i, 0, 0);
			//	 sendPixel(i, 255 - i, 0);
			//	 show();
			//	 delay(10);
			// }
			// while (i >1 ){
			//	 i--;
			//	 sendPixel(i, 0, 0);
			//	 sendPixel(0, i, 0);
			//	 sendPixel(0, 0, i);
			//	 sendPixel(255 - i, 255 - i, 255 - i);
			//	 sendPixel(255 - i, i, 255 - i);
			//	 sendPixel(i, 255 - i, i);
			//	 sendPixel(i, 0, 0);
			//	 sendPixel(i, 255 - i, 0);
			//	 show();
			//	 delay(10);
			// }
		 //}
		 //

		 //
		 //

		 //for (size_t i = 0; i < 255; i++)
		 //{
			// ShowBedTemp(255, i);
			// delay(10);
		 //}*/

	 }
	 static void factoryTest(){
		 for (int i = 0; i < 250; i += 1)
		 {
			 for (int k = 0; k < NUM_LEDS; k++)
			 {
				 sendPixel(i, 0, 0);
			 }
			 show();
			 delay(10);
		 }
		 for (int i = 0; i < 250; i+=1)
		 {
			 for (int k = 0; k < NUM_LEDS; k++)
			 {
				 sendPixel(0,i,0);
			 }
			 show();
			 delay(10);
		 }
		 for (int i = 0; i < 250; i ++)
		 {
			 for (int k = 0; k < NUM_LEDS; k++)
			 {
				 sendPixel(0,0,i);
			 }
			 show();
			 delay(10);
		 }
		 for (int i = 0; i < 250; i += 1)
		 {
			 for (int k = 0; k < NUM_LEDS; k++)
			 {
				 sendPixel(i,i,i);
			 }
			 show();
			 delay(10);
		 }
		 
	 }
	 //void loop()
	 //{
		// switch (CurrentShow) {
		// case Off:
		//	 FastLED.showColor(CRGB(0, 0, 0));
		//	 break;
		// case SolidRed:
		//	 FastLED.showColor(CRGB(255, 0, 0));
		//	 break;
		// case SolidGreen:
		//	 FastLED.showColor(CRGB(0, 255, 0));
		//	 break;
		// case SolidBlue:
		//	 FastLED.showColor(CRGB(0, 0, 255));
		//	 break;
		// case BedTempDynamic:
		//	 //ShowBedTemp(heatedBedController.targetTemperatureC ,  Extruder::getHeatedBedTemperature());
		//	 break;
		// }
	 //}
	 //static void ShowBedTemp(float TargetTemp, float FactTemp)
	 //{

		// //int r = FactTemp * 255 / (TargetTemp - BaseTemp);
		// //if (r>255) r = 255;
		// //FastLED.showColor(CRGB(r*LedBrightness, 0, (255 - r)*LedBrightness));
	 //}
	 //void SetShowType(ShowType SType)
	 //{
		// CurrentShow = SType;
		// CurrentShowStep = 0;
		// UpdateNeeded = true;
	 //}
	
};

#endif
