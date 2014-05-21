// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#define DS1307_ADDRESS 0x68
#define SECONDS_PER_DAY 86400L

#define SECONDS_FROM_1970_TO_2000 946684800

#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
#else
 #include <WProgram.h>
#endif

#include "configuration.h"
#include "gcode.h"
#include "fastio.h"
#include <Wire.h>
#include <avr/pgmspace.h>
#include "Reptier.h"
#include "use3DMaster.h"


int i = 0; //The new wire library needs to take an int when you are sending for the zero register
////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 }; //has to be const or compiler compaints

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

uint8_t DateTime::dayOfWeek() const {    
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

uint8_t RTC_DS1307::begin(void) {
  return 1;
}


#if (ARDUINO >= 100)

uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(i);	
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.read();
  return !(ss>>7);
}

void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(i);
    Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.write(i);
    Wire.endTransmission();
}

DateTime RTC_DS1307::now() {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(i);	
  Wire.endTransmission();
  
  Wire.requestFrom(DS1307_ADDRESS, 7);
  uint8_t ss = bcd2bin(Wire.read() & 0x7F);
  uint8_t mm = bcd2bin(Wire.read());
  uint8_t hh = bcd2bin(Wire.read());
  Wire.read();
  uint8_t d = bcd2bin(Wire.read());
  uint8_t m = bcd2bin(Wire.read());
  uint16_t y = bcd2bin(Wire.read()) + 2000;
  
  return DateTime (y, m, d, hh, mm, ss);
}

#else

uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.send(i);	
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.receive();
  return !(ss>>7);
}

void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.send(i);
    Wire.send(bin2bcd(dt.second()));
    Wire.send(bin2bcd(dt.minute()));
    Wire.send(bin2bcd(dt.hour()));
    Wire.send(bin2bcd(0));
    Wire.send(bin2bcd(dt.day()));
    Wire.send(bin2bcd(dt.month()));
    Wire.send(bin2bcd(dt.year() - 2000));
    Wire.send(i);
    Wire.endTransmission();
}

DateTime RTC_DS1307::now() {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.send(i);	
  Wire.endTransmission();
  
  Wire.requestFrom(DS1307_ADDRESS, 7);
  uint8_t ss = bcd2bin(Wire.receive() & 0x7F);
  uint8_t mm = bcd2bin(Wire.receive());
  uint8_t hh = bcd2bin(Wire.receive());
  Wire.receive();
  uint8_t d = bcd2bin(Wire.receive());
  uint8_t m = bcd2bin(Wire.receive());
  uint16_t y = bcd2bin(Wire.receive()) + 2000;
  
  return DateTime (y, m, d, hh, mm, ss);
}

#endif


////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
  return (uint32_t)(offset + millis() / 1000);
}

////////////////////////////////////////////////////////////////////////////////

RTC_DS1307 RTC;

////////////////////////////////////////////////////////////////////////////////


void init3DMaster( void )
{

	// initialize the RTC
	initRTC();

	// initialize the two strain gauges
	initStrainGauge();

	// initialize the TMP100
	initTMP100();

	// initialize the 24C256
	init24C256();

	// initialize the additional outputs
	initOutputs();

} // init3DMaster


void test3DMaster( void )
{
	// read the current value of the RTC
	readRTC();

	// read the current values of the strain gauges
	readStrainGauge();

	// read the current value from the TMP100
	readTMP100();

	// read the test value from the 24C256
	read24C256();

	// set all outputs to 0
	writeOutputs( 0 );

	// read the end switches
	readEndSwitches();

} // test3DMaster
		


void initRTC( void )
{
	Wire.begin();
	RTC.begin();

	if( !RTC.isrunning() )
	{
		// the following line sets the RTC to the date/time when this sketch was compiled
		RTC.adjust( DateTime( __DATE__, __TIME__ ) );
	}

} // initRTC


void readRTC( void )
{
    DateTime now = RTC.now();
    
	Serial.println( "*** readRTC() ***" );
    Serial.print( now.year(), DEC );
    Serial.print( '/');
    Serial.print( now.month(), DEC );
    Serial.print( '/');
    Serial.print( now.day(), DEC );
    Serial.print( ' ');
    Serial.print( now.hour(), DEC );
    Serial.print( ':');
    Serial.print( now.minute(), DEC );
    Serial.print( ':');
    Serial.print( now.second(), DEC );
    Serial.println();
    
} // readRTC
	

void initStrainGauge( void )
{
	// configure DMS #1 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( 0x49 );
	Wire.write( 0x8C );
	Wire.endTransmission();

	// configure DMS #2 (0x8C = single mode, 16 bits, gain = *1)
	Wire.beginTransmission( 0x4A );
	Wire.write( 0x8C );
	Wire.endTransmission();

} // initStrainGauge


void readStrainGauge( void )
{
	unsigned char	Register;
	short			Result;

	// read DMS #1
	Wire.beginTransmission( 0x49 );
	Wire.requestFrom( 0x49, 3 );
        
	Result =  Wire.read();
    Result =  Result << 8;
	Result += Wire.read();
        
	Register = Wire.read();
	Wire.endTransmission();
     
	Serial.println( "*** readStrainGauge() ***" );
	Serial.print( "DMS #1: " );
	Serial.println( Result, DEC );

	// read DMS #2
	Wire.beginTransmission( 0x4A );
	Wire.requestFrom( 0x4A, 3 );
        
	Result =  Wire.read();
    Result =  Result << 8;
	Result += Wire.read();
        
	Register = Wire.read();
	Wire.endTransmission();
     
	Serial.print( "DMS #2: " );
	Serial.println( Result, DEC );

} // readStrainGauge


void initTMP100( void )
{
  Wire.beginTransmission( 0x4C );
  
  // configuration register
  Wire.write( 0x01 );
  
  // resolution bits (we want to have 12 bits)
  Wire.write( 3 << 5 );
  Wire.endTransmission();
 
  Wire.beginTransmission( 0x4C );
  Wire.write( 0x00 );
  Wire.endTransmission();

} // initTMP100


void readTMP100( void )
{
	short	Result;
	float	Temperature;
	
	
	Wire.requestFrom( 0x4C, 2 );
	Result = Wire.read();
	Result = Result << 8;
	Result += Wire.read();
	Result = Result >> 4;

	Temperature = Result * 0.0625;
 
	Serial.println( "*** readTMP100() ***" );
	Serial.print( "Temperature: " );
	Serial.println( Temperature );
  
} // readTMP100


void init24C256( void )
{
	// write some test byte
	writeByte24C256( 0x50, 0, 0xA5 );
	
} // init24C256


void read24C256( void )
{
	unsigned char	Test;
	
	
	// read our test byte
	Test = readByte24C256( 0x50, 0 );
	
	Serial.println( "*** read24C256() ***" );
	if( Test == 0xA5 )	Serial.println( "EEPROM success" );
	else				Serial.println( "EEPROM error" );
	
} // read24C256


void writeByte24C256( int addressI2C, unsigned int addressEEPROM, unsigned char data )
{
    Wire.beginTransmission( addressI2C );
    Wire.write( (int)(addressEEPROM >> 8));	// MSB
    Wire.write( (int)(addressEEPROM & 0xFF)); // LSB
    Wire.write( data );
    Wire.endTransmission();
    
} // writeByte24C256


unsigned char readByte24C256( int addressI2C, unsigned int addressEEPROM )
{
    Wire.beginTransmission( addressI2C );
    Wire.write( (int)(addressEEPROM >> 8));	// MSB
    Wire.write( (int)(addressEEPROM & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom( addressI2C, 1 );
    
    return Wire.read();
    
} // readByte24C256


void initOutputs( void )
{
	SET_OUTPUT( 25 );	// OUT1
	SET_OUTPUT( 27 );	// OUT2
	SET_OUTPUT( 29 );	// OUT3
	SET_OUTPUT( 8 );	// HZ3
	
} // initOutputs


void writeOutputs( unsigned char on )
{
	// OUT1
	WRITE( 25, on );

	// OUT2
	WRITE( 27, on );
	
	// OUT3
	WRITE( 29, on );
	
	// HZ1
	WRITE( 10, on );
	
	// HZ2
	WRITE( 9, on );
	
	// HZ3
	WRITE( 8, on );
	
} // writeOutputs


void readEndSwitches( void )
{
    Serial.println( "*** readEndSwitches() ***" );
    
	out.print_int_P( PSTR( "Xmin: "), READ( X_MIN_PIN ) );
	out.print_int_P( PSTR( " Xmax: "), READ( 15 ) );
	Serial.println( "" );
	out.print_int_P( PSTR( "Ymin: "), READ( Y_MIN_PIN ) );
	out.print_int_P( PSTR( " Ymax: "), READ( 18 ) );
	Serial.println( "" );
	out.print_int_P( PSTR( "Zmin: "), READ( Z_MIN_PIN ) );
	out.print_int_P( PSTR( " Zmax: "), READ( 19 ) );
	Serial.println( "" );

} // readEndSwitches

