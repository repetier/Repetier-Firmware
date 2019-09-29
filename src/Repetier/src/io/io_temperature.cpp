#include "Repetier.h"

#define CELSIUS_EXTRA_BITS 3
#define TEMP_INT_TO_FLOAT(temp) ((float)(temp) / (float)(1 << CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp) * (1 << CELSIUS_EXTRA_BITS)))

float IOTemperatureTable::interpolateNTC(int value, fast8_t num, const short* temptable) {
    num <<= 1;
    fast8_t i = 2;
    int oldraw = pgm_read_word(&temptable[0]);
    int oldtemp = pgm_read_word(&temptable[1]);
    int newraw, newtemp = 0;
    if (value < oldraw) {
        return TEMP_INT_TO_FLOAT(oldtemp);
    }
    while (i < num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newraw > value) {
            return TEMP_INT_TO_FLOAT(oldtemp + (float)(value - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
        }
        oldtemp = newtemp;
        oldraw = newraw;
    }
    // Overflow: Set to last value in the table
    return TEMP_INT_TO_FLOAT(newtemp);
}

float IOTemperatureTable::interpolatePTC(int value, fast8_t num, const short* temptable) {
    num <<= 1;
    fast8_t i = 2;
    int oldraw = pgm_read_word(&temptable[0]);
    int oldtemp = pgm_read_word(&temptable[1]);
    int newraw, newtemp = 0;
    // value = 4095 - value;
    if (value < oldraw) {
        return TEMP_INT_TO_FLOAT(oldtemp);
    }
    while (i < num) {
        newraw = pgm_read_word(&temptable[i++]);
        newtemp = pgm_read_word(&temptable[i++]);
        if (newraw > value) {
            return TEMP_INT_TO_FLOAT(oldtemp + (float)(value - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
        }
        oldtemp = newtemp;
        oldraw = newraw;
    }
    // Overflow: Set to last value in the table
    return TEMP_INT_TO_FLOAT(newtemp);
}