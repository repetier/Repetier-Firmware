#include "Repetier.h"

void LightStoreBase::reset() {
    mode = LIGHT_STATE_OFF;
}
bool LightStoreBase::on() {
    counter++;
    if (counter == 12) {
        counter = 0;
    }
    switch (mode) {
    case LIGHT_STATE_OFF:
        return false;
    case LIGHT_STATE_ON:
        return true;
    case LIGHT_STATE_BURST:
        return counter == 0 || counter == 6;
    case LIGHT_STATE_BLINK_FAST:
        return counter < 3 || (counter >= 6 && counter <= 8);
    case LIGHT_STATE_BLINK_SLOW:
        return counter < 6;
    default:
        return false;
    }
}

LightStoreMonochrome::LightStoreMonochrome()
    : LightStoreBase() {}

void LightStoreMonochrome::set(uint8_t _mode, uint8_t red, uint8_t green, uint8_t blue) {
    mode = _mode;
}

void LightStoreRGB::reset() {
    mode = LIGHT_STATE_OFF;
    redVal = greenVal = blueVal = 255;
}

LightStoreRGB::LightStoreRGB()
    : LightStoreBase() {}

void LightStoreRGB::set(uint8_t _mode, uint8_t _red, uint8_t _green, uint8_t _blue) {
    mode = _mode;
    redVal = _red;
    greenVal = _green;
    blueVal = _blue;
}
