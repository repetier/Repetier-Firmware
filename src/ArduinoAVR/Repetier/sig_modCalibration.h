#ifndef _sig_modui_h
#define _sig_modui_h

#if UI_DISPLAY_TYPE != NO_DISPLAY

/* Definitions */
/*===================================================================================*/
#define WELCOME						0
#define CHOOSE_NORMAL_FIRST_START_A	1
#define CHOOSE_NORMAL_FIRST_START_B	2
#define NORMAL_START				3
#define FIRST_START1				4
#define FIRST_START2				5
#define FIRST_START3				6
#define FIRST_START4				7
#define FIRST_START5				8
#define FIRST_START6				9
#define FIRST_START7				10
#define FIRST_START8				11
#define FIRST_START9				12
#define FIRST_START10				13
#define FIRST_START11				14
#define FIRST_START12				15
#define FIRST_START13				16
#define FIRST_START14				17
#define FIRST_START15				18
#define FIRST_START16				19
#define FIRST_START17				20
#define FIRST_START18				21
#define FIRST_START19				22
#define FIRST_START20				23
#define FIRST_START21				24
#define FIRST_START22				25
#define FIRST_START23				26



#define PLA	1
#define ABS	2
// #define PLA_TEMPERATURE	210
// #define ABS_TEMPERATURE	290
// #define PLA_TEMPERATURE	110
// #define ABS_TEMPERATURE	120


/*===================================================================================*/

#define TEMP_TABLE_OFFSET_DEGREES   -50
#define TEMP_MIN_TEMP          TEMP_TABLE_OFFSET_DEGREES
#define TEMP_TABLE_SIZE        36
#define TEMP_MAX_TEMP          (((TEMP_TABLE_SIZE-1) * 10) + TEMP_TABLE_OFFSET_DEGREES)

/** ADC channel number for the temperature sensor. */
#define ADC_CHANNEL13               ((1 << 8) | (0x05 << MUX0))
#define TEMP_ADC_CHANNEL             ADC_CHANNEL13  

/** ADC channel MUX mask for the temperature sensor. */
#define TEMP_ADC_CHANNEL_MASK  			TEMP_ADC_CHANNEL
#define ADC_REFERENCE_AVCC              (1 << REFS0)
#define ADC_RIGHT_ADJUSTED              (0 << ADLAR)


// Functions
/*===================================================================================*/
void bedCalibration(void);

bool buttonPressed(void);

void encoderRotation(void);   /* int8_t encoderRotation(void);*/

inline void extrudeFillament_inCM(unsigned int i);

void extrudeFillament_inMM(void);

static inline uint16_t ADC_GetChannelReading(const uint16_t MUXMask);

int Temperature_GetTemperature(void);

// Functions
/*===================================================================================*/

class SigMod
{
    public:
        static void calibration();
};


// Variables
extern uint8_t temporary_stop_isr_for_extruder;


#endif
#endif
