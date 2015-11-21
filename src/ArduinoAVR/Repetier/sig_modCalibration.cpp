#include "Repetier.h"
#include "ui.h"
#include "HAL.h"

// Inlcude custom headers
#include "sig_modCalibration.h"
#include "sig_modEeprom.h"


#include <avr/wdt.h>

/* The rest file is working as soon as display exists */
#if UI_DISPLAY_TYPE != NO_DISPLAY
UIDisplay uid;


/* An einai ok na metaferthei sto header (sig_modui.h) */
/* Dokimi mallon einai axrhsth */
extern uint8_t EEMEM bed_calibration_is_ok;

/*===================================================================================*/

#define drawHProgressBar1(x,y,width,height,progress) \
     {u8g_DrawFrame(&u8g,x,y, width, height);  \
     int p = ceil((width-2) * progress / 100); \
     u8g_DrawBox(&u8g,x+1,y+1, p, height-2);}

/*===================================================================================*/

volatile int8_t left_right = 0;
volatile uint8_t state = 0;
volatile uint8_t temp_disable_lcd_refresh = 0;

/*===================================================================================*/

void bedCalibration()
{
		
	static volatile uint8_t loop = 1;
	// static int8_t left_right = 0;
	static int8_t left_right_last = 0;
	static int position = 0;
	
	int current_temp = 0;
	bool temp_changed = true;
	int temp_last;
	static uint8_t material = 0;
	unsigned int iii=0;
		
	while(loop){
		
		HAL::pingWatchdog();
		
		switch(state){
			case WELCOME:
				u8g_FirstPage(&u8g);
				do
				{
					uid.printRowP(3, PSTR("      Welcome!      "));
				}
				while( u8g_NextPage(&u8g) );  //end of welcome
				HAL::delayMilliseconds(1500);
				state = CHOOSE_NORMAL_FIRST_START_A;
			break;
			case CHOOSE_NORMAL_FIRST_START_A:
				u8g_FirstPage(&u8g);
				do
				{
					uid.printRowP(0, PSTR("  a) First start up."));
					uid.printRowP(1, PSTR("  b) Start up."));
					// uid.printRowP(2, PSTR("--------------------"));
					uid.printRowP(3, PSTR("[Choose a or b and ]"));
					uid.printRowP(4, PSTR("[press the knob to ]"));
					uid.printRowP(5, PSTR("[     Continue.    ]"));
					
				}
				while( u8g_NextPage(&u8g) );  
				// HAL::delayMilliseconds(1000);
				state = CHOOSE_NORMAL_FIRST_START_B;
				
			break;
			case CHOOSE_NORMAL_FIRST_START_B:
				
				cli();
				
				if (left_right<=-1 && left_right_last!=-1){				
					left_right_last = -1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("->a) First start up."));
						uid.printRowP(1, PSTR("  b) Start up."));
						// uid.printRowP(2, PSTR("--------------------"));
						uid.printRowP(3, PSTR("[Choose a or b and ]"));
						uid.printRowP(4, PSTR("[press the knob to ]"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
					}
					while( u8g_NextPage(&u8g) ); 
					// HAL::delayMilliseconds(1000);
				}
				else if (left_right>=1 && left_right_last!=1){
					left_right_last = 1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("  a) First start up."));
						uid.printRowP(1, PSTR("->b) Start up."));
						// uid.printRowP(2, PSTR("--------------------"));
						uid.printRowP(3, PSTR("[Choose a or b and ]"));
						uid.printRowP(4, PSTR("[press the knob to ]"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
					}
					while( u8g_NextPage(&u8g) );
					// HAL::delayMilliseconds(1000);
				}
				
				sei();
				
				if (buttonPressed() && left_right_last!=0){
				
					if (left_right_last == -1){
						state = FIRST_START1;
						left_right_last = 0;
						temp_disable_lcd_refresh = 1;
					}
					else {
						state = NORMAL_START;
					}
				}
				
			break;
			case NORMAL_START:
				loop = 0; 			// continue to normal program flow
				uid.encoderPos=0;
				uid.encoderLast=0;
				uid.menuLevel = 0;
				uid.activeAction = 0;
				// eeprom_update_byte(&bed_calibration_is_ok, 1 );
				HAL::eprSetByte(EPR_BED_CALIBRATION_OK,1);
				
			break;
			case FIRST_START1:
				
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("This is the first"));
						uid.printRowP(1, PSTR("start up of your"));
						uid.printRowP(2, PSTR("printer."));
						// uid.printRowP(3, PSTR("--------------------"));
						// uid.printRowP(3, PSTR("[                  ]"));
						uid.printRowP(4, PSTR("[Press the knob to ]"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
						
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
						state = FIRST_START2;
						left_right_last = 0;
						
				}
				
			break;
			case  FIRST_START2:
				
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("I will guide you to"));
						uid.printRowP(1, PSTR("adjust the printer"));
						uid.printRowP(2, PSTR("for your first"));
						uid.printRowP(3, PSTR("print."));
						// uid.printRowP(3, PSTR("[                  ]"));
						uid.printRowP(4, PSTR("[Press the knob to ]"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
						
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START3;
					// state = FIRST_START12;
					left_right_last = 0;
					
					/* home ALL */
					Printer::homeAxis(true,true,true);
											
				}
						
			break;
			
			case FIRST_START3:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Now you need to"));
						uid.printRowP(1, PSTR("make some "));
						uid.printRowP(2, PSTR("adjustments."));
						// uid.printRowP(3, PSTR("--------------------"));
						// uid.printRowP(3, PSTR("[                  ]"));
						uid.printRowP(4, PSTR("[Press the knob to ]"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
						
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START4;
					left_right_last = 0;
						
				}
			break;
			
			case FIRST_START4:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("You should adjust"));
						uid.printRowP(1, PSTR("the build plate to"));
						uid.printRowP(2, PSTR("refrain about 0.1 mm"));
						uid.printRowP(3, PSTR("from the nozzle."));
						// uid.printRowP(4, PSTR("--------------------"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
												
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START5;
					left_right_last = 0;
					
					/* O extruder παει περιπου πανω απο το κουμπι ρυθμισης στο πισω μερος */
					Printer::moveToReal(125.0, 250.0, 250.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
					Printer::moveToReal(125.0, 225.0, 250.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
					Printer::moveToReal(125.0, 225.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				}
			break;
			
			case FIRST_START5:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Cut a small piece of"));
						uid.printRowP(1, PSTR("an ordinary paper"));
						uid.printRowP(2, PSTR("sheet and place it"));
						uid.printRowP(3, PSTR("between the extruder"));
						uid.printRowP(4, PSTR("and the build plate."));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START6;
					left_right_last = 0;
						
				}
								
			break;
			
			case FIRST_START6:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Adjust the plate by"));
						uid.printRowP(1, PSTR("turning the knurled"));
						uid.printRowP(2, PSTR("knob under the plate"));
						uid.printRowP(3, PSTR("left or right."));
						// uid.printRowP(4, PSTR("--------------------"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START7;
					left_right_last = 0;
						
				}
				
			break;
			
			case FIRST_START7:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("The right hight is"));
						uid.printRowP(1, PSTR("when you just feel"));
						uid.printRowP(2, PSTR("the paper touching"));
						uid.printRowP(3, PSTR("the nozzle of the"));
						uid.printRowP(4, PSTR("extruder."));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START8;
					left_right_last = 0;
					
					/* To build plate κατεβαινει 15mm, παει μπρος δεξια και ξανανεβαινει */						
					Printer::moveToReal(125.0, 225.0, 15.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);		// 15mm down
					Printer::moveToReal(25.0, 25.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());	// move x,y
					Printer::moveToReal(25.0, 25.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);	// return z to +15mm
				}
				
			break;

			case FIRST_START8:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Again, adjust the")); 
						uid.printRowP(1, PSTR("build plate as"));
						uid.printRowP(2, PSTR("needed."));
						// uid.printRowP(3, PSTR(" "));
						// uid.printRowP(4, PSTR("--------------------"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START9;
					left_right_last = 0;
					
					/* To build plate κατεβαινει 15mm, παει μπρος αριστερα και ξανανεβαινει */						
					Printer::moveToReal(25.0, 25.0, 15.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);		// 15mm down
					Printer::moveToReal(225.0, 25.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());	// move x
					Printer::moveToReal(225.0, 25.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);	// return z to +15mm
				}
				
			break;
			
			
			case FIRST_START9:
				
				if (buttonPressed() ){
				
						state = FIRST_START10;
						// left_right_last = 0;
						
					/* O extruder παει παλι πανω απο το κουμπι ρυθμισης στο πισω μερος */					
					Printer::moveToReal(225.0, 25.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
					Printer::moveToReal(125.0, 225.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
					Printer::moveToReal(125.0, 225.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				}
			break;
			
			case FIRST_START10:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Correct if needed.")); 
						// uid.printRowP(1, PSTR(" "));
						// uid.printRowP(2, PSTR(" "));
						// uid.printRowP(3, PSTR(" "));
						// uid.printRowP(4, PSTR("--------------------"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
					state = FIRST_START11;
					left_right_last = 0;
						
					/* To build plate κατεβαινει 15mm, παει μπρος δεξια και ξανανεβαινει */						
					Printer::moveToReal(125.0, 225.0, 15.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);		// 15mm down
					Printer::moveToReal(25.0, 25.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());	// move x,y
					Printer::moveToReal(25.0, 25.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);	// return z to +15mm
				}
			break;
			
			case FIRST_START11:
				if (buttonPressed()){
				
					state = FIRST_START12;
					left_right_last = 0;
						
					/* To build plate κατεβαινει 15mm, παει μπρος αριστερα και ξανανεβαινει */						
					Printer::moveToReal(25.0, 25.0, 15.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);		// 15mm down
					Printer::moveToReal(225.0, 25.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());	// move x
					Printer::moveToReal(225.0, 25.0, 0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);	// return z to +15mm
				}
			break;
			case FIRST_START12:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Plate calibration")); 
						uid.printRowP(1, PSTR("has completed with"));
						uid.printRowP(2, PSTR("success.Next step is"));
						uid.printRowP(3, PSTR("to insert the"));
						uid.printRowP(4, PSTR("filament."));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
						state = FIRST_START13;
						left_right_last = 0;
						
						/* Ο extruder παει στη μεση και το build plate  κατεβαινει κατω */
						Printer::moveToReal(225.0, 25.0, 15.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);						
						Printer::moveToReal(125.0, 125.0, 15.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());		// 	move x,y					
						Printer::moveToReal(125.0, 125.0, 100.0, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);		// 	move z
						
						// Printer::moveToReal(125.0, 125.0, 250.0, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());						
						Commands::waitUntilEndOfAllMoves();

				}
			break;

			case FIRST_START13:
				if( left_right_last == 0 ){
				u8g_FirstPage(&u8g);
				do
				{
					uid.printRowP(0, PSTR("Please select your"));
					uid.printRowP(1, PSTR("material and press"));
					uid.printRowP(2, PSTR("continue."));
					uid.printRowP(3, PSTR("   PLA"));
					uid.printRowP(4, PSTR("   ABS"));
					uid.printRowP(5, PSTR("[     Continue.    ]"));
					
				}
				while( u8g_NextPage(&u8g) );  
				
				state = FIRST_START14;
				cli();
				left_right = 0;
				sei();
				}
			break;
			
			case FIRST_START14:
				

				cli();
				
				if (left_right<=-1 && left_right_last!=-1){				
					left_right_last = -1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Please select your"));
						uid.printRowP(1, PSTR("material and press"));
						uid.printRowP(2, PSTR("continue."));
						uid.printRowP(3, PSTR("-> PLA"));
						uid.printRowP(4, PSTR("   ABS"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
					}
					while( u8g_NextPage(&u8g) ); 
				}
				else if (left_right>=1 && left_right_last!=1){
					left_right_last = 1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Please select your"));
						uid.printRowP(1, PSTR("material and press"));
						uid.printRowP(2, PSTR("continue."));
						uid.printRowP(3, PSTR("   PLA"));
						uid.printRowP(4, PSTR("-> ABS"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
					}
					while( u8g_NextPage(&u8g) );
				}
				
				sei();
				
				if (buttonPressed() && left_right_last!=0){
				
					if (left_right_last == -1){
						material = PLA;
					}
					else {
						material = ABS;
					}
					
					state = FIRST_START15;
					left_right_last = 0;
				}
			break;
			
			case FIRST_START15:
				u8g_FirstPage(&u8g);
				do
				{
					uid.printRowP(0, PSTR("You have selected"));
					if(material == PLA)
						uid.printRowP(1, PSTR("PLA."));
					else
						uid.printRowP(1, PSTR("ABS."));
					uid.printRowP(2, PSTR("Continue?"));
					uid.printRowP(3, PSTR("   Yes"));
					uid.printRowP(4, PSTR("   No"));
					uid.printRowP(5, PSTR("[     Continue.    ]"));
					
				}
				while( u8g_NextPage(&u8g) );  

				state = FIRST_START16;
				
				cli();
				left_right = 0;
				sei();
				
			break;
			case FIRST_START16:
				
				cli();
				
				if (left_right<=-1 && left_right_last!=-1){				
					left_right_last = -1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("You have selected"));
						if(material == PLA)
							uid.printRowP(1, PSTR("PLA."));
						else
							uid.printRowP(1, PSTR("ABS."));
						uid.printRowP(2, PSTR("Continue?"));
						uid.printRowP(3, PSTR("-> Yes"));
						uid.printRowP(4, PSTR("   No"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
						
					}
					while( u8g_NextPage(&u8g) ); 
				}
				else if (left_right>=1 && left_right_last!=1){
					left_right_last = 1;
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("You have selected"));
						if(material == PLA)
							uid.printRowP(1, PSTR("PLA."));
						else
							uid.printRowP(1, PSTR("ABS."));
						uid.printRowP(2, PSTR("Continue?"));
						uid.printRowP(3, PSTR("   Yes"));
						uid.printRowP(4, PSTR("-> No"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
						
					}
					while( u8g_NextPage(&u8g) );
				}
				
				sei();
				
				if (buttonPressed() && left_right_last!=0){
				
					if (left_right_last == -1){
						state = FIRST_START17;
					}
					else {
						state = FIRST_START13;
					}
					
					left_right_last = 0;
				}
				
			break;
			
			case FIRST_START17:
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("The next step is to"));  
						uid.printRowP(1, PSTR("insert the filament."));
						// uid.printRowP(2, PSTR(" "));
						// uid.printRowP(3, PSTR(" "));
						// uid.printRowP(4, PSTR("--------------------"));
						uid.printRowP(5, PSTR("[     Continue.    ]"));
					}
					while( u8g_NextPage(&u8g) );
					left_right_last = 1;
				}
				
				if (buttonPressed() && left_right_last!=0){
				
						state = FIRST_START18;
						left_right_last = 0;
						
				}
			break;
			
			case FIRST_START18:
				
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("The filament loads"));         
						uid.printRowP(1, PSTR("in the back of the"));
						uid.printRowP(2, PSTR("printer where the"));
						uid.printRowP(3, PSTR("arrow shows."));
						// uid.printRowP(4, PSTR(""));
						uid.printRowP(5, PSTR("[     Continue.     ]"));
					}
					while( u8g_NextPage(&u8g) );
				
					left_right_last = 1;
				}
					
				if (buttonPressed() && left_right_last!=0){

					state = FIRST_START19;
					left_right_last = 0;
					
				}
				
			break;

			case FIRST_START19:
							
				if( left_right_last == 0 ){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Push until you see"));            
						uid.printRowP(1, PSTR("filament inserting"));
						uid.printRowP(2, PSTR("the clear tube."));
						// uid.printRowP(3, PSTR(""));
						// uid.printRowP(4, PSTR(""));
						uid.printRowP(5, PSTR("[     Continue.     ]"));
					}
					while( u8g_NextPage(&u8g) );
					
					left_right_last = 1;
				}
					
				if (buttonPressed() && left_right_last!=0){

					state = FIRST_START20;
					left_right_last = 0;
					
					temp_last = 0;
					// cli();
					WRITE(EXT0_HEATER_PIN,!HEATER_PINS_INVERTED); //turn on extruder
					temporary_stop_isr_for_extruder = 0;
				}
				
			break;


			case FIRST_START20:
				
				// cli();
				current_temp = Temperature_GetTemperature();
				// char output[20];
				// snprintf(output,20,"%f",current_temp);
				if(temp_last < current_temp){
					temp_last = current_temp;
					temp_changed = true;
				}
				// Com::printFloat(current_temp,2);
				// sei();
				// Com::print(current_temp);
				// Com::print('\n');
				// cli();
				// WRITE(EXT0_HEATER_PIN,!HEATER_PINS_INVERTED); //turn on extruder
				if(temp_changed){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Please wait while"));   
						uid.printRowP(1, PSTR("the printhead is"));
						uid.printRowP(2, PSTR("reaching the proper"));
						uid.printRowP(3, PSTR("temperature."));
						// uid.printRow(4, output,NULL,UI_COLS);
						// uid.printRowP(4, PSTR(""));
						// uid.printRowP(5, PSTR("[     Continue.     ]"));
					
						drawHProgressBar1(14,48,100,7,(100*temp_last)/((material == PLA) ? PLA_TEMPERATURE : ABS_TEMPERATURE));
					}
					while( u8g_NextPage(&u8g) );
					
					temp_changed = false;
				}
				
				if(current_temp >= ((material == PLA) ? PLA_TEMPERATURE : ABS_TEMPERATURE)){
					state = FIRST_START21;
					extrudeFillament_inCM(30);
				} 
				// sei();
				_delay_ms(1000);
				
				
				
			break;

			case FIRST_START21:
				u8g_FirstPage(&u8g);
				do
				{
					uid.printRowP(0, PSTR("Now,rotate the knob"));      
					uid.printRowP(1, PSTR("until filament is"));
					uid.printRowP(2, PSTR("being extruded from"));
					uid.printRowP(3, PSTR("the printhead."));
					// uid.printRowP(4, PSTR(""));
					uid.printRowP(5, PSTR("[     Continue.     ]"));
				
				}
				while( u8g_NextPage(&u8g) );
				
				state = FIRST_START22;
				

			break;
				
			case FIRST_START22:

				current_temp = Temperature_GetTemperature();
				
				if(current_temp < ((material == PLA) ? PLA_TEMPERATURE : ABS_TEMPERATURE))
					WRITE(EXT0_HEATER_PIN,!HEATER_PINS_INVERTED);
				else 
					WRITE(EXT0_HEATER_PIN,HEATER_PINS_INVERTED);
				
				cli();
							
				if (left_right<-1 ){
					left_right_last = 1;
					extrudeFillament_inMM();
				}
				
				sei();
				
				if (buttonPressed() && left_right_last!=0){
				
					left_right_last = 0;
					state = FIRST_START23;
					HAL::eprSetByte(EPR_BED_CALIBRATION_OK,1);		// initial setup completed
					iii = 0;
					temporary_stop_isr_for_extruder = 1;
				}

			break;

			case FIRST_START23:

				while(1){
					u8g_FirstPage(&u8g);
					do
					{
						uid.printRowP(0, PSTR("Initial setup has"));              
						uid.printRowP(1, PSTR("been completed!"));
						// uid.printRowP(2, PSTR(""));
						uid.printRowP(3, PSTR("    Printer will    "));
						uid.printRowP(4, PSTR("      Restart!      "));
						// uid.printRowP(5, PSTR(""));
						
						drawHProgressBar1(14,56,100,7,iii);
					}
					while( u8g_NextPage(&u8g) );
					
					iii += 10;
					
					_delay_ms(1000);
					
					if(iii < 90){
						HAL::pingWatchdog();
					}
					else if(iii>100){
						WDTCSR = (1<<WDCE) | (1<<WDE);					// unlock wd
						WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1);		// 1sec wd reset
						for(;;){}
					}
					
					// Printer::kill(uint8_t only_steppers)
					// if(iii >= 100){
						// HAL::resetHardware();
					// }
				}
				
				
			break;
			
			// static void initExtruder();
    // static void initHeatedBed();
    // static void setHeatedBedTemperature(float temp_celsius,bool beep = false);
    // static float getHeatedBedTemperature();
	
	// Extruder::setTemperatureForExtruder(float temperatureInCelsius,uint8_t extr,bool beep)		MAXTEMP
	
	// static void step();
    // static void unstep();
    // static void setDirection(uint8_t dir);
    // static void enable();
	
	// bool UIDisplay::nextPreviousAction(int16_t next, bool allowMoves)
	
	        // case UI_ACTION_PREHEAT_PLA: 


			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			// case FIRST_START:
				
			// break;
			
			
			
					// cli();
				
				// if (left_right<=-1 && position!=0){
					// while(left_right != position){
						// /*	move down millimeters */
						// position--;
					// }
				// }
				// else if(left_right>=1 && position!=0){
					// while(left_right != position){
						// /*	move up millimeters */
						// position++;
					// }
				// }
				
				// sei();
			default:
				state = CHOOSE_NORMAL_FIRST_START_A;
				left_right_last = 0;
			break;
			
			
		}	/* End of switch(state) */
	}	/* End of while(loop) */
	
	
	// u8g_FirstPage(&u8g);
    // do
    // {
        // uid.printRowP(3, PSTR("123456789abcdefghigklm"));
    // }
    // while( u8g_NextPage(&u8g) );  //end picture loop
	
	// HAL::delayMilliseconds(2000);
	
	// u8g_FirstPage(&u8g);
    // do
    // {
        // uid.printRowP(1, PSTR("test1"));
    // }
    // while( u8g_NextPage(&u8g) );  //end picture loop
	
	// HAL::delayMilliseconds(2000);

	// while(READ(BTN_ENC)){};
	
}

/*===================================================================================*/

// The following variables are long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long start_time = 0;       // the last time the output pin was toggled
long debounceTime = 50;    // the debounce time; increase if the output flickers
int reading_last=1;        // the current reading from the input pin

/*===================================================================================*/

bool buttonPressed(void)
{
	
  // read the state of the switch into a local variable:
  int reading = 1;
  
  reading = READ(UI_ENCODER_CLICK);
	  
  
  if(reading == 0)
  {
		if(reading != reading_last)
		{
			// reset the debouncing timer
			reading_last = reading;
			start_time = millis();
		}
 
	    if ((millis() - start_time) > debounceTime)
	    {
			while(READ(UI_ENCODER_CLICK)==0)
			{
			    HAL::pingWatchdog();
			}
		    start_time = 0;
		    reading_last = 1;
			
			return true;
		}
  }
     
  return false;
}

/*===================================================================================*/

// int8_t encoderRotation(void){

	// static uint8_t encoderLast1 = 0;
	
	// encoderLast1 = (encoderLast1 << 2) & 0x0F;
	
	// if (!READ(UI_ENCODER_A)) 
		// encoderLast1 |=0x02;
	
	// if (!READ(UI_ENCODER_B)) 
		// encoderLast1 |=0x01;
	
	// return pgm_read_byte(&encoder_table[encoderLast1]);
// }

/*===================================================================================*/

void encoderRotation(void){

	static uint8_t encoderLast1 = 0;
	
	encoderLast1 = (encoderLast1 << 2) & 0x0F;
	
	if (!READ(UI_ENCODER_A)) 
		encoderLast1 |=0x02;
	
	if (!READ(UI_ENCODER_B)) 
		encoderLast1 |=0x01;
	
	if(state == CHOOSE_NORMAL_FIRST_START_B || state == FIRST_START14 || state == FIRST_START22){
		if(left_right<-1) left_right=-1;
		if(left_right>1) left_right=1;
	}
	
	left_right += pgm_read_byte(&encoder_table[encoderLast1]);
}

/*===================================================================================*/

inline void extrudeFillament_inCM(unsigned int i){

	for(unsigned int j=0;j<(i*10);j++)
		extrudeFillament_inMM();
}

/*===================================================================================*/

void extrudeFillament_inMM(void){
	
	unsigned int i=200;	// number of loops for 1mm of fillament
	
	cli();		//disable interrupts
	
	WRITE(EXT0_ENABLE_PIN, EXT0_ENABLE_ON );	// enable extruder
	
	WRITE(EXT0_DIR_PIN,!EXT0_INVERSE);		// set forward movement
	// RESET_EXTRUDER_JAM(0, dir);
	
	while(i--)
	{
		WRITE(EXT0_STEP_PIN, HIGH);		
		// _delay_ms(1);
		_delay_us(100);
		WRITE(EXT0_STEP_PIN, LOW);
		// _delay_ms(1);
		_delay_us(100);
		
		HAL::pingWatchdog();
	}
	
	WRITE(EXT0_ENABLE_PIN, !EXT0_ENABLE_ON );		// disable extruder
	
	sei();	// enable interrupts
}


/*===================================================================================*/

static const uint16_t PROGMEM Temperature_Lookup[TEMP_TABLE_SIZE] =
{
	1023,1022,1021,1019,1016,1010,
	1001,987 ,966 ,937 ,897 ,847,
	785 ,713 ,635 ,554 ,475 ,402,
	336 ,278 ,229 ,189 ,155 ,127,
	105 ,87  ,72  ,60  ,51  ,43,
	36  ,31  ,26  ,22  ,19  ,17
};

/*===================================================================================*/

static inline uint16_t ADC_GetChannelReading(const uint16_t MUXMask)
{
	// while ADC reading is complete
	while (!(((ADCSRA & (1 << ADIF)) ? true : false)));	
	
	// ADC Start Reading
	ADMUX = MUXMask;

	#if defined(ADCSRB) && defined(MUX5)
	if (MUXMask & (1 << 8))
	  ADCSRB |=  (1 << MUX5);
	else
	  ADCSRB &= ~(1 << MUX5);
	#endif

	ADCSRA |= (1 << ADSC);

	// while ADC reading is complete
	while (!(((ADCSRA & (1 << ADIF)) ? true : false)));
	
	// ADC Get Result
	ADCSRA |= (1 << ADIF);

	return ADC;
}

/*===================================================================================*/

int Temperature_GetTemperature(void)
{
	uint16_t Temp_ADC = ADC_GetChannelReading(ADC_REFERENCE_AVCC | ADC_RIGHT_ADJUSTED | TEMP_ADC_CHANNEL_MASK);

	if (Temp_ADC > pgm_read_word(&Temperature_Lookup[0]))
	  return TEMP_MIN_TEMP;

	for (uint16_t Index = 0; Index < TEMP_TABLE_SIZE; Index++)
	{
		if (Temp_ADC > pgm_read_word(&Temperature_Lookup[Index]))
		  return ((Index*10) + TEMP_TABLE_OFFSET_DEGREES);
	}

	return TEMP_MAX_TEMP;
}

/*===================================================================================*/

static void SigMod::calibration(void)
{
    if(READ(UI_ENCODER_CLICK)==0 && HAL::eprGetByte(EPR_BED_CALIBRATION_OK)==1)
        HAL::eprSetByte(EPR_BED_CALIBRATION_OK, 0);
		
    if(! HAL::eprGetByte(EPR_BED_CALIBRATION_OK))
        bedCalibration();

    HAL::delayMilliseconds(200);
}

#endif
