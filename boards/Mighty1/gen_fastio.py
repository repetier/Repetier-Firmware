# currently unused

n=100

print """
#include <Arduino.h>
#include "fastio.h"


void FastIO::digitalWrite(uint8_t pin,uint8_t value) {
    switch(pin) {
"""

for i in xrange(0,n): 
    print """
#ifdef DIO%(i)d_PIN
        case %(i)d:
            if(value) DIO%(i)d_WPORT |= _BV(DIO%(i)d_PIN); else DIO%(i)d_WPORT &= ~_BV(DIO%(i)d_PIN);
            break;
#endif"""%{"i":i}

print """
        default:
            break;
    }
}
"""
print """
uint8_t FastIO::digitalRead(uint8_t pin) {
    switch(pin) {
"""
for i in xrange(0,n): 
    print """
#ifdef DIO%(i)d_PIN
        case %(i)d:
            return (DIO%(i)d_RPORT & _BV(DIO%(i)d_PIN)) ? HIGH : LOW;
#endif"""%{"i":i}
print """
        default:
            return LOW;
    }
}
"""

print """
void FastIO::pinMode(uint8_t pin,uint8_t mode) {
    switch(pin) {
"""
for i in xrange(0,n): 
    print """
#ifdef DIO%(i)d_PIN
        case %(i)d:
            switch(mode) {
                case INPUT:
                    DIO%(i)d_DDR &= ~_BV(DIO%(i)d_PIN);
                    DIO%(i)d_WPORT &= ~_BV(DIO%(i)d_PIN);
                    break;
                case INPUT_PULLUP:
                    DIO%(i)d_DDR &= ~_BV(DIO%(i)d_PIN);
                    DIO%(i)d_WPORT |= _BV(DIO%(i)d_PIN);
                    break;
                default:
                    DIO%(i)d_DDR |= _BV(DIO%(i)d_PIN);
            }
            break;
#endif"""%{"i":i}
print """
        default:
            break;
    }
}
"""


