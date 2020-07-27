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

void LightStoreMonochrome::set(uint8_t _mode, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
    mode = _mode;
}

void LightStoreRGB::reset() {
    mode = LIGHT_STATE_OFF;
    redVal = greenVal = blueVal = 255;
}

LightStoreRGB::LightStoreRGB()
    : LightStoreBase() {}

void LightStoreRGB::set(uint8_t _mode, uint8_t _red, uint8_t _green, uint8_t _blue, uint8_t brightness) {
    mode = _mode;
    redVal = _red;
    greenVal = _green;
    blueVal = _blue;
}

void LightStorePWM::set(uint8_t _mode, uint8_t _red, uint8_t _green, uint8_t _blue, uint8_t brightness) {
    finalSetBrightness = brightness;
    finalSetMode = _mode;
}

void LightStorePWM::reset() {
    if (mode != finalSetMode || finalSetBrightness != lastBrightness) {
        // new mode or change in brightness needs a step recalculation
        switch (mode = finalSetMode) {
        case LIGHT_STATE_BLINK_SLOW:
            // Each fade takes 800ms..
            fadeStep = computePWMStep(800, finalSetBrightness);
            break;
        case LIGHT_STATE_BURST:
            // Two fade pulses which take 100ms each, with ~600ms inbetween them.
            fadeStep = computePWMStep(100, finalSetBrightness);
            break;
        default:
            // 400ms by default for light off/on/blink fast
            fadeStep = computePWMStep(400, finalSetBrightness);
        }
        lastBrightness = finalSetBrightness;
    }

    if (mode == LIGHT_STATE_OFF) {
        targetPWM = 0;
    } else if (mode == LIGHT_STATE_ON) {
        targetPWM = finalSetBrightness;
    } else if (mode == LIGHT_STATE_BURST) {
        if (curPWM == targetPWM) {
            if (counter == 5 || counter == 11) {
                targetPWM = finalSetBrightness;
            } else {
                targetPWM = 0;
            }

            if (++counter > 11) {
                counter = 0;
            }
        }
    } else if (mode == LIGHT_STATE_BLINK_FAST) {
        if (curPWM == targetPWM) {
            if (targetPWM) {
                targetPWM = 0;
            } else {
                targetPWM = finalSetBrightness;
            }
        }
    } else if (mode == LIGHT_STATE_BLINK_SLOW) {
        if (curPWM == targetPWM) {
            if (targetPWM) {
                targetPWM = 0;
            } else {
                targetPWM = finalSetBrightness;
            }
        }
    }
}

fast8_t LightStorePWM::updatePWM() {
    millis_t curTime = HAL::timeInMilliseconds();

    if ((curTime - lastUpdate) < refreshRateMS) {
        return curPWM;
    }
    lastUpdate = curTime;

    if (curPWM < targetPWM) {
        curPWM = ((curPWM += rolloverCheck(fadeStep, true)) > targetPWM) ? targetPWM : curPWM;
    } else if (curPWM > targetPWM) {
        curPWM = ((curPWM -= rolloverCheck(fadeStep, false)) < targetPWM) ? targetPWM : curPWM;
    }
    return curPWM;
}
