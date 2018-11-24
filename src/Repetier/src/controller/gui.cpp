#include "Repetier.h"

int GUI::level = 0;                        // Menu level for back handling
int GUI::topRow[GUI_MAX_LEVEL];            ///< First visible row
int GUI::cursorRow[GUI_MAX_LEVEL];         ///< Selected row
int GUI::maxCursorRow[GUI_MAX_LEVEL];      ///< Maximum selectable row
int GUI::length[GUI_MAX_LEVEL];            ///< Rows for complete content
GuiCallback GUI::callbacks[GUI_MAX_LEVEL]; ///< Content handler
void* GUI::data[GUI_MAX_LEVEL];            ///< menu pointer to custom data
GUIPageType GUI::pageType[GUI_MAX_LEVEL];  ///< page type
millis_t GUI::lastRefresh = 0;             ///< Last refresh time
millis_t GUI::lastAction = 0;              ///< Last action time for autoreturn to display
GUIStatusLevel GUI::statusLevel = GUIStatusLevel::REGULAR;
bool GUI::contentChanged = false;            ///< set to true if forced refresh is wanted
GUIAction GUI::nextAction = GUIAction::NONE; ///< Next action to execute on opdate
int GUI::nextActionRepeat = 0;               ///< Increment for next/previous
char GUI::status[MAX_COLS + 1];              ///< Status Line
char GUI::buf[MAX_COLS + 1];                 ///< Buffer to build strings
fast8_t GUI::bufPos;                         ///< Pos for appending data

#if DISPLAY_DRIVER == DRIVER_NONE
void GUI::init() { ///< Initialize display
    level = 0;
}

void GUI::refresh() {
}

void GUI::resetMenu() {} ///< Go to start page

void __attribute__((weak)) startScreen(GUIAction action, void* data) {}
void __attribute__((weak)) waitScreen(GUIAction action, void* data) {}
void __attribute__((weak)) infoScreen(GUIAction action, void* data) {}
void __attribute__((weak)) warningScreen(GUIAction action, void* data) {}
void __attribute__((weak)) errorScreen(GUIAction action, void* data) {}
#endif

#if DISPLAY_DRIVER != DRIVER_NONE
void GUI::resetMenu() { ///< Go to start page
    level = 0;
    replace(startScreen, nullptr, GUIPageType::TOPLEVEL);
}
#endif

void GUI::update() {
#if DISPLAY_DRIVER != DRIVER_NONE
    millis_t timeDiff = HAL::timeInMilliseconds() - lastRefresh;

    handleKeypress(); // Test for new keys

    if (nextAction != GUIAction::NONE && nextAction != GUIAction::CLICK_PROCESSED) {
        Com::printFLN(PSTR("Action:"), (int32_t)nextAction);
        callbacks[level](nextAction, data[level]);
        nextAction = nextAction == GUIAction::CLICK ? GUIAction::CLICK_PROCESSED : GUIAction::NONE;
        contentChanged = true;
        lastAction = HAL::timeInMilliseconds();
    }

    if (level > 0 && !isStickyPageType(pageType[level]) && (HAL::timeInMilliseconds() - lastAction) > UI_AUTORETURN_TO_MENU_AFTER) {
        level = 0;
    }
#if DISPLAY_DRIVER == DRIVER_U8G2
    if (statusLevel == GUIStatusLevel::BUSY && timeDiff > 500) {
        contentChanged = true; // for faster spinning icon
    }
#endif
    if ((timeDiff > 1000 && timeDiff < 60000) || contentChanged) {
        // Com::printFLN(PSTR("upd:"), (int32_t)timeDiff);
        refresh();
        lastRefresh = HAL::timeInMilliseconds();
        contentChanged = false;
    }
#endif
}

bool GUI::isStickyPageType(GUIPageType t) {
    return t == GUIPageType::WIZARD_FIXED || t == GUIPageType::WIZARD_MENU || t == GUIPageType::ACK_MESSAGE || t == GUIPageType::BUSY || t == GUIPageType::STATUS;
}

void GUI::pop() {
    if (level > 0) {
        if (pageType[level] == GUIPageType::BUSY || pageType[level] == GUIPageType::STATUS) {
            status[0] = 0;
            statusLevel = GUIStatusLevel::REGULAR;
        }
        level--;
        contentChanged = true;
    }
}

void GUI::popBusy() {
    if (level > 0 && pageType[level] == GUIPageType::BUSY) {
        status[0] = 0;
        statusLevel = GUIStatusLevel::REGULAR;
        level--;
        contentChanged = true;
    }
}

void GUI::push(GuiCallback cb, void* cData, GUIPageType tp) {
    if (tp == GUIPageType::STATUS || tp == GUIPageType::BUSY) {
        if (pageType[level] == GUIPageType::STATUS || pageType[level] == GUIPageType::BUSY) {
            level--; // Replace as they all share status for data
        }
    }
    if (level == GUI_MAX_LEVEL - 1) {
        return;
    }
    if (tp == GUIPageType::BUSY) {
        contentChanged = true; // Show directly being busy
    }
    level++;
    replace(cb, cData, tp);
}

void GUI::replace(GuiCallback cb, void* cData, GUIPageType tp) {
    callbacks[level] = cb;
    data[level] = cData;
    pageType[level] = tp;
    topRow[level] = 0;
    length[level] = 0;
    cursorRow[level] = -1;
    maxCursorRow[level] = -1;
    if (tp == GUIPageType::MENU || tp == GUIPageType::WIZARD_MENU) {
        cb(GUIAction::ANALYSE, cData);
    }
    contentChanged = true;
}

void GUI::replaceOn(GUIAction a, GuiCallback cb, void* cData, GUIPageType tp) {
    if (nextAction == a) {
        replace(cb, cData, tp);
    }
}

void GUI::pushOn(GUIAction a, GuiCallback cb, void* cData, GUIPageType tp) {
    if (nextAction == a) {
        push(cb, cData, tp);
    }
}

// Run action for key press
void GUI::backKey() {
    pop();
    contentChanged = true;
}
void GUI::nextKey() {
    if (nextAction != GUIAction::NEXT) {
        nextActionRepeat = 0;
    }
    nextAction = GUIAction::NEXT;
    nextActionRepeat++;
    contentChanged = true;
}
void GUI::previousKey() {
    if (nextAction != GUIAction::PREVIOUS) {
        nextActionRepeat = 0;
    }
    nextAction = GUIAction::PREVIOUS;
    nextActionRepeat++;
    contentChanged = true;
}
void GUI::okKey() {
    nextAction = GUIAction::CLICK;
    contentChanged = true;
}

void GUI::handleKeypress() {
    setEncoderA(ControllerEncA::get());
    setEncoderB(ControllerEncB::get());
    if (ControllerClick::get()) {
        if (nextAction != GUIAction::CLICK_PROCESSED) {
            okKey();
        }
    } else if (nextAction == GUIAction::CLICK_PROCESSED) {
        nextAction = GUIAction::NONE;
    }
    if (ControllerBack::get()) {
        backKey();
    }
}

#if ENCODER_SPEED == 0
const int8_t encoder_table[16] PROGMEM = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 }; // Full speed
#elif ENCODER_SPEED == 1
const int8_t encoder_table[16] PROGMEM = { 0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0 }; // Half speed
#else
//const int8_t encoder_table[16] PROGMEM = {0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0}; // Quart speed
//const int8_t encoder_table[16] PROGMEM = {0,1,0,0,-1,0,0,0,0,0,0,0,0,0,0,0}; // Quart speed
const int8_t encoder_table[16] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0 }; // Quart speed
#endif

static fast8_t aEnc = 0;
static fast8_t bEnc = 0;
static fast8_t encoderLast = 0;

void GUI::setEncoderA(fast8_t state) {
    if (state == aEnc) {
        return;
    }
    aEnc = state;
    encoderLast = (encoderLast << 2) & 0x0F;
    if (aEnc) {
        encoderLast += 2;
    }
    if (bEnc) {
        encoderLast++;
    }
    int8_t mod = pgm_read_byte(&encoder_table[encoderLast]) * ENCODER_DIRECTION;
    if (mod > 0) {
        nextKey();
    }
    if (mod < 0) {
        previousKey();
    }
}

void GUI::setEncoderB(fast8_t state) {
    if (state == bEnc) {
        return;
    }
    bEnc = state;
    encoderLast = (encoderLast << 2) & 0x0F;
    if (aEnc) {
        encoderLast += 2;
    }
    if (bEnc) {
        encoderLast++;
    }
    int8_t mod = pgm_read_byte(&encoder_table[encoderLast]) * ENCODER_DIRECTION;
    if (mod > 0) {
        nextKey();
    }
    if (mod < 0) {
        previousKey();
    }
}

const float roundingTable[] PROGMEM = { 0.5, 0.05, 0.005, 0.0005, 0.00005, 0.000005 };

void GUI::bufClear() {
    bufPos = 0;
    buf[0] = 0;
}

void GUI::bufAddInt(int value, uint8_t digits, char fillChar) {
    uint8_t dig = 0, neg = 0;
    byte addspaces = digits > 0;
    if (digits < 0)
        digits = -digits;
    if (value < 0) {
        neg = 1;
        value = -value;
        dig++;
    }
    char buf2[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf2[12];
    buf2[12] = 0;
    do {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);
    if (neg) {
        *--str = '-';
        dig++;
    }
    if (addspaces && digits <= 11) {
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    }
    while (*str && bufPos < MAX_COLS) {
        buf[bufPos++] = *str;
        str++;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddLong(long value, int8_t digits) {
    uint8_t dig = 0, neg = 0;
    byte addspaces = digits > 0;
    if (digits < 0)
        digits = -digits;
    if (value < 0) {
        neg = 1;
        value = -value;
        dig++;
    }
    char buf2[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf2[12];
    buf2[12] = 0;
    do {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);
    if (neg) {
        *--str = '-';
        dig++;
    }
    if (addspaces && digits <= 11) {
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    }
    while (*str && bufPos < MAX_COLS) {
        buf[bufPos++] = *str;
        str++;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddFloat(float value, int8_t fixdigits, int8_t digits) {
    if (bufPos >= MAX_COLS) {
        return;
    }
    // Handle negative numbers
    if (value < 0.0) {
        bufAddChar('-');
        value = -value;
        fixdigits--;
    }
    value += pgm_read_float(&roundingTable[digits]); // for correct rounding

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)value;
    float remainder = value - (float)int_part;
    bufAddLong(int_part, fixdigits);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        bufAddChar('.');
    }

    // Extract digits from the remainder one at a time
    while (bufPos < MAX_COLS && digits-- > 0) {
        remainder *= 10.0;
        uint8_t toPrint = uint8_t(remainder);
        buf[bufPos++] = '0' + toPrint;
        remainder -= toPrint;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddString(char* value) {
    while (bufPos < MAX_COLS) {
        uint8_t c = *value;
        if (c == 0)
            break;
        buf[bufPos++] = c;
        value++;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddStringP(FSTRINGPARAM(value)) {
    while (bufPos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(value++);
        if (c == 0)
            break;
        buf[bufPos++] = c;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddChar(char value) {
    if (bufPos < MAX_COLS) {
        buf[bufPos++] = value;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddHeaterTemp(HeatManager* hm, bool target) {
    HeaterError err = hm->getError();
    if (err == HeaterError::NO_ERROR) {
        if (hm->isUnplugged()) {
            GUI::bufAddStringP(PSTR("---"));
        } else if (hm->isPaused()) {
            GUI::bufAddStringP(PSTR("Off"));
        } else {
            GUI::bufAddFloat(hm->getCurrentTemperature(), 3, 0);
            if (target) {
                GUI::bufAddChar('/');
                GUI::bufAddFloat(hm->getTargetTemperature(), 3, 0);
            }
            GUI::bufAddStringP(PSTR("°C"));
        }
    } else {
        GUI::bufAddStringP(PSTR("Defect"));
    }
}

void GUI::bufToStatus(GUIStatusLevel lvl) {
    setStatus(buf, lvl);
}

void GUI::clearStatus() {
    if (statusLevel == GUIStatusLevel::REGULAR) {
        status[0] = 0;
    }
}

void GUI::setStatusP(FSTRINGPARAM(text), GUIStatusLevel lvl) {
    if (lvl >= statusLevel) {
        fast8_t pos = 0;
        while (pos < MAX_COLS) {
            uint8_t c = HAL::readFlashByte(text++);
            if (c == 0)
                break;
            status[pos++] = c;
        }
        status[pos] = 0;
        statusLevel = lvl;
        if (lvl == GUIStatusLevel::BUSY) {
            push(waitScreen, status, GUIPageType::BUSY);
        }
        if (lvl == GUIStatusLevel::INFO) {
            push(infoScreen, status, GUIPageType::STATUS);
        }
        if (lvl == GUIStatusLevel::WARNING) {
            push(warningScreen, status, GUIPageType::STATUS);
        }
        if (lvl == GUIStatusLevel::ERROR) {
            push(errorScreen, status, GUIPageType::STATUS);
        }
    }
}

void GUI::setStatus(char* text, GUIStatusLevel lvl) {
    if (lvl >= statusLevel) {
        fast8_t pos = 0;
        while (pos < MAX_COLS) {
            uint8_t c = *text;
            text++;
            if (c == 0)
                break;
            status[pos++] = c;
        }
        status[pos] = 0;
        statusLevel = lvl;
        if (lvl == GUIStatusLevel::BUSY) {
            push(waitScreen, status, GUIPageType::BUSY);
        }
        if (lvl == GUIStatusLevel::INFO) {
            push(infoScreen, status, GUIPageType::STATUS);
        }
        if (lvl == GUIStatusLevel::WARNING) {
            push(warningScreen, status, GUIPageType::STATUS);
        }
        if (lvl == GUIStatusLevel::ERROR) {
            push(errorScreen, status, GUIPageType::STATUS);
        }
    }
}
