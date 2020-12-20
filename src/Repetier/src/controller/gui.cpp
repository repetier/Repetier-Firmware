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
char GUI::tmpString[MAX_COLS + 1];           ///< Buffer to build strings
fast8_t GUI::bufPos;                         ///< Pos for appending data
GUIBootState GUI::curBootState = GUIBootState::DISPLAY_INIT;
bool GUI::textIsScrolling = false; ///< Our selected row/text is now scrolling/anim
probeProgInfo* GUI::curProbingProgress = nullptr;
#if SDSUPPORT
char GUI::cwd[SD_MAX_FOLDER_DEPTH * LONG_FILENAME_LENGTH + 2] = { '/', 0 };
uint8_t GUI::folderLevel = 0;
#endif

#if DISPLAY_DRIVER == DRIVER_NONE
void GUI::init() { ///< Initialize display
    level = 0;
}
void GUI::processInit() { ///< Function repeatedly called if curBootState isn't at least IN_INTRO
}

void GUI::refresh() {
}

void GUI::resetMenu() { } ///< Go to start page

void __attribute__((weak)) probeProgress(GUIAction action, void* data) { }
void __attribute__((weak)) startScreen(GUIAction action, void* data) { }
void __attribute__((weak)) waitScreen(GUIAction action, void* data) { }
void __attribute__((weak)) infoScreen(GUIAction action, void* data) { }
void __attribute__((weak)) warningScreen(GUIAction action, void* data) { }
void __attribute__((weak)) errorScreen(GUIAction action, void* data) { }
void __attribute__((weak)) waitScreenP(GUIAction action, void* data) { }
void __attribute__((weak)) infoScreenP(GUIAction action, void* data) { }
void __attribute__((weak)) warningScreenP(GUIAction action, void* data) { }
void __attribute__((weak)) errorScreenP(GUIAction action, void* data) { }
#endif

#if DISPLAY_DRIVER != DRIVER_NONE
void GUI::resetMenu() { ///< Go to start page
    level = 0;
    replace(Printer::isPrinting() ? printProgress : Printer::isZProbingActive() ? probeProgress : startScreen, nullptr, GUIPageType::TOPLEVEL);
}
#endif

void GUI::update() {
#if DISPLAY_DRIVER != DRIVER_NONE
    millis_t timeDiff = HAL::timeInMilliseconds() - lastRefresh;
    handleKeypress();                                 // Test for new keys
    if (curBootState == GUIBootState::DISPLAY_INIT) { // Small delay before we start processing
        processInit();
        nextAction = GUIAction::NONE;
        contentChanged = false;
        return;
    } else if (curBootState == GUIBootState::IN_INTRO) { // Boot screen
        if (contentChanged || (timeDiff < 60000 && timeDiff > 1000)) {
            // Skip if any key presses or timeout.
            curBootState = GUIBootState::READY;
            lastRefresh = HAL::timeInMilliseconds();
            if (HAL::startReason == BootReason::WATCHDOG_RESET) {
                push(warningScreenP, (void*)PSTR("Reset by Watchdog!"), GUIPageType::STATUS);
                Printer::playDefaultSound(DefaultSounds::WARNING);
            } else if (HAL::startReason == BootReason::BROWNOUT) {
#ifdef ALWAYS_SHOW_BROWNOUT_WARNING
                push(warningScreenP, (void*)PSTR("Brownout reset!"), GUIPageType::STATUS);
                Printer::playDefaultSound(DefaultSounds::WARNING);
#else
                if (Printer::isRescueRequired()) {
                    push(warningScreenP, (void*)PSTR("Brownout reset!"), GUIPageType::STATUS);
                    Printer::playDefaultSound(DefaultSounds::WARNING);
                }
#endif
            }
        }
    }

    if (nextAction == GUIAction::BACK) {
        Printer::playDefaultSound(DefaultSounds::OK);
        pop();
        nextAction = GUIAction::BACK_PROCESSED;
        lastAction = HAL::timeInMilliseconds();
        nextActionRepeat = 0;
        contentChanged = true;
    } else if (nextAction != GUIAction::NONE && nextAction != GUIAction::CLICK_PROCESSED && nextAction != GUIAction::BACK_PROCESSED) {
        if (nextAction == GUIAction::NEXT || nextAction == GUIAction::PREVIOUS) {
            Printer::playDefaultSound(DefaultSounds::NEXT_PREV);
        } else if (nextAction == GUIAction::CLICK) {
            Printer::playDefaultSound(DefaultSounds::OK);
        }
        // Com::printFLN(PSTR("Action:"), (int32_t)nextAction);
        lastAction = HAL::timeInMilliseconds();
        callbacks[level](nextAction, data[level]); // Execute action
        nextAction = nextAction == GUIAction::CLICK ? GUIAction::CLICK_PROCESSED : (nextAction == GUIAction::BACK ? GUIAction::BACK_PROCESSED : GUIAction::NONE);
        nextActionRepeat = 0;
        contentChanged = true;
    }

    if (level > 0 && !isStickyPageType(pageType[level]) && (HAL::timeInMilliseconds() - lastAction) > UI_AUTORETURN_TO_MENU_AFTER) {
        level = 0;
    }
    if ((statusLevel == GUIStatusLevel::BUSY || GUI::textIsScrolling) && timeDiff > 500) {
        contentChanged = true; // for faster spinning icon
    }
    if (timeDiff < 60000 && (timeDiff > 1000 || contentChanged)) {
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

void GUI::pop(int selection) {
    GUI::pop();
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
    nextAction = GUIAction::BACK;
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

/** Check for button and store result in nextAction. */
void GUI::handleKeypress() {
    if (!ControllerClick::get()) {
        setEncoderA(ControllerEncA::get());
        setEncoderB(ControllerEncB::get());
    }
    // debounce clicks
    if (nextAction == GUIAction::CLICK_PROCESSED || nextAction == GUIAction::BACK_PROCESSED) {
        millis_t timeDiff = HAL::timeInMilliseconds() - lastAction;
        if (timeDiff < 200) { // wait 200ms until next click counts
            return;
        }
    }
    // action sequence: CLICK -> execute operation -> CLICK_PROCESSED -> ok key up -> NONE
    if (ControllerClick::get()) {
        if (nextAction != GUIAction::CLICK_PROCESSED) {
            okKey();
        }
    } else if (nextAction == GUIAction::CLICK_PROCESSED) {
        nextAction = GUIAction::NONE;
    }
#if ENABLED(UI_HAS_BACK_KEY)
    // action sequence: BACK -> execute operation -> BACK_PROCESSED -> back key up -> NONE
    if (ControllerBack::get()) {
        if (nextAction != GUIAction::BACK_PROCESSED) {
            backKey();
        }
    } else if (nextAction == GUIAction::BACK_PROCESSED) {
        nextAction = GUIAction::NONE;
    }
#endif
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
    } else if (mod < 0) {
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
    } else if (mod < 0) {
        previousKey();
    }
}

const float roundingTable[] PROGMEM = { 0.5, 0.05, 0.005, 0.0005, 0.00005, 0.000005 };

void GUI::bufClear() {
    bufPos = 0;
    buf[0] = 0;
}

void GUI::bufAddInt(int value, int8_t digits, char fillChar) {
    uint8_t dig = 0, neg = 0;
    byte addspaces = digits > 0;
    if (digits < 0) {
        digits = -digits;
    }
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
        if (c == 0) {
            break;
        }
        buf[bufPos++] = c;
        value++;
    }
    buf[bufPos] = 0;
}

void GUI::bufAddStringP(FSTRINGPARAM(value)) {
    while (bufPos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(value++);
        if (c == 0) {
            break;
        }
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
            GUI::bufAddFloat(hm->getCurrentTemperature(), 3, 0);
            if (target) {
                GUI::bufAddChar('/');
                GUI::bufAddStringP(PSTR("Off"));
            }
            GUI::bufAddStringP(PSTR("°C"));
        } else {
            GUI::bufAddFloat(hm->getCurrentTemperature(), 3, 0);
            if (target) {
                if (hm->isOff()) {
                    GUI::bufAddStringP(PSTR("°C/Off"));
                } else {
                    GUI::bufAddChar('/');
                    GUI::bufAddFloat(hm->getTargetTemperature(), 3, 0);
                    GUI::bufAddStringP(PSTR("°C"));
                }
            } else {
                GUI::bufAddStringP(PSTR("°C"));
            }
        }
    } else {
        GUI::bufAddStringP(PSTR("Defect"));
    }
}

void GUI::bufToStatus(GUIStatusLevel lvl) {
    setStatus(buf, lvl);
}

void GUI::flashToString(char* dest, FSTRINGPARAM(text)) {
    fast8_t pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0)
            break;
        dest[pos++] = c;
    }
    dest[pos] = 0;
}

void GUI::flashToStringLong(char* dest, FSTRINGPARAM(text), int32_t value) {
    fast8_t pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0) {
            break;
        }
        if (c == '@') {
            uint8_t dig = 0;
            if (value < 0) {
                dest[pos++] = '-';
                value = -value;
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
            while (*str && pos < MAX_COLS) {
                dest[pos++] = *str;
                str++;
            }
        } else {
            dest[pos++] = c;
        }
    }
    dest[pos] = 0;
}

void GUI::flashToStringFloat(char* dest, FSTRINGPARAM(text), float value, int digits) {
    fast8_t pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0) {
            break;
        }
        if (c == '@') {
            // Handle negative numbers
            if (value < 0.0) {
                dest[pos++] = '-';
                value = -value;
            }
            value += pgm_read_float(&roundingTable[digits]); // for correct rounding

            // Extract the integer part of the number and print it
            uint32_t int_part = static_cast<uint32_t>(value);
            float remainder = value - static_cast<float>(int_part);
            uint8_t dig = 0;
            char buf2[13]; // Assumes 8-bit chars plus zero byte.
            char* str = &buf2[12];
            buf2[12] = 0;
            do {
                unsigned long m = int_part;
                int_part /= 10;
                char c = m - 10 * int_part;
                *--str = c + '0';
                dig++;
            } while (int_part);
            while (*str && pos < MAX_COLS) {
                dest[pos++] = *str;
                str++;
            }

            // Print the decimal point, but only if there are digits beyond
            if (digits > 0) {
                dest[pos++] = '.';
                // Extract digits from the remainder one at a time
                while (digits-- > 0) {
                    remainder *= 10.0;
                    uint8_t toPrint = static_cast<uint8_t>(remainder);
                    dest[pos++] = '0' + toPrint;
                    remainder -= toPrint;
                }
            }
        } else {
            dest[pos++] = c;
        }
    }
    dest[pos] = 0;
}

void GUI::flashToStringFlash(char* dest, FSTRINGPARAM(text), FSTRINGPARAM(val)) {
    fast8_t pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0) {
            break;
        }
        if (c == '@') {
            while (pos < MAX_COLS) {
                uint8_t c = HAL::readFlashByte(val++);
                if (c == 0)
                    break;
                dest[pos++] = c;
            }
        } else {
            dest[pos++] = c;
        }
    }
    dest[pos] = 0;
}

void GUI::flashToStringString(char* dest, FSTRINGPARAM(text), char* val) {
    fast8_t pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0) {
            break;
        }
        if (c == '@') {
            while (pos < MAX_COLS) {
                uint8_t c = *val;
                val++;
                if (c == 0)
                    break;
                dest[pos++] = c;
            }
        } else {
            dest[pos++] = c;
        }
    }
    dest[pos] = 0;
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
            Printer::playDefaultSound(DefaultSounds::WARNING);
            push(warningScreen, status, GUIPageType::STATUS);
            Com::promptStart(GUI::pop, Com::tWarning, status, false);
            Com::promptButton(Com::tOk);
            Com::promptShow();
        }
        if (lvl == GUIStatusLevel::ERROR) {
            Printer::playDefaultSound(DefaultSounds::ERROR);
            push(errorScreen, status, GUIPageType::STATUS);
            Com::promptStart(GUI::pop, Com::tError, status, false);
            Com::promptButton(Com::tOk);
            Com::promptShow();
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
            Printer::playDefaultSound(DefaultSounds::WARNING);
            push(warningScreen, status, GUIPageType::STATUS);
            Com::promptStart(GUI::pop, Com::tWarning, status, false);
            Com::promptButton(Com::tOk);
            Com::promptShow();
        }
        if (lvl == GUIStatusLevel::ERROR) {
            push(errorScreen, status, GUIPageType::STATUS);
            Com::promptStart(GUI::pop, Com::tError, status, false);
            Com::promptButton(Com::tOk);
            Com::promptShow();
        }
    }
}

bool GUI::handleFloatValueAction(GUIAction& action, float& value, float min, float max, float increment) {
    if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
        return false;
    }
    float orig = value;
    if (action == GUIAction::NEXT) {
        float calc = (nextActionRepeat * increment);
        value = (value == min) ? increment * ::floorf((value + calc) / increment) : (value + calc);
        contentChanged = true;
    } else if (action == GUIAction::PREVIOUS) {
        float calc = (nextActionRepeat * increment);
        value = (value == max) ? increment * ::ceilf((value - calc) / increment) : (value - calc);
        contentChanged = true;
    }
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    } else if (std::signbit(orig) != std::signbit(value)) {
        value = increment * std::roundf(value / increment);
    }
    return orig != value;
}

bool GUI::handleFloatValueAction(GUIAction& action, float& value, float increment) {
    if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
        return false;
    }
    float orig = value;
    if (action == GUIAction::NEXT) {
        float calc = (nextActionRepeat * increment);
        value = increment * ::floorf((value + calc) / increment);
        contentChanged = true;
    } else if (action == GUIAction::PREVIOUS) {
        float calc = (nextActionRepeat * increment);
        value = increment * ::ceilf((value - calc) / increment);
        contentChanged = true;
    }
    if (std::signbit(orig) != std::signbit(value)) {
        value = increment * std::roundf(value / increment);
    }
    return orig != value;
}

bool GUI::handleLongValueAction(GUIAction& action, int32_t& value, int32_t min, int32_t max, int32_t increment) {
    if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
        return false;
    }
    int32_t orig = value;
    if (action == GUIAction::NEXT) {
        int32_t calc = value + (nextActionRepeat * increment);
        value = (value == min) ? increment * ((calc - std::signbit(calc) * (increment - 1)) / increment) : calc;
        contentChanged = true;
    } else if (action == GUIAction::PREVIOUS) {
        int32_t calc = value - (nextActionRepeat * increment);
        value = (value == max) ? increment * ((calc + !std::signbit(calc) * (increment - 1)) / increment) : calc;
        contentChanged = true;
    }
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    } else if (std::signbit(orig) != std::signbit(value)) {
        int32_t calc = (std::labs(value) + (increment / 2));
        value = (calc - (calc % increment)) * (std::signbit(value) ? -1 : 1);
    }
    return orig != value;
}

void GUI::menuBack(GUIAction& action) {
#if DISABLED(UI_HAS_BACK_KEY)
    GUI::menuSelectableP(action, PSTR("Back"), nullptr, nullptr, GUIPageType::POP);
#else
    if (action == GUIAction::ANALYSE) {
        if (cursorRow[level] < 0) {
            cursorRow[level] = length[level];
        }
        maxCursorRow[level] = length[level];
        length[level]++;
    }
#endif
}

void directAction(GUIAction action, void* data) {
    int opt = reinterpret_cast<int>(data);
    switch (opt) {
    case GUI_DIRECT_ACTION_HOME_ALL:
        if (!Printer::isHoming()) {
            Motion1::homeAxes(0);
        }
        break;
    case GUI_DIRECT_ACTION_HOME_X:
    case GUI_DIRECT_ACTION_HOME_Y:
    case GUI_DIRECT_ACTION_HOME_Z:
    case GUI_DIRECT_ACTION_HOME_E:
    case GUI_DIRECT_ACTION_HOME_A:
    case GUI_DIRECT_ACTION_HOME_B:
    case GUI_DIRECT_ACTION_HOME_C:
        if (!Printer::isHoming()) {
            Motion1::homeAxes(axisBits[opt - GUI_DIRECT_ACTION_HOME_X]);
        }
        break;
    case GUI_DIRECT_ACTION_FACTORY_RESET:
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        GUI::setStatusP(PSTR("Factory Setting Act."), GUIStatusLevel::INFO);
        break;
    case GUI_DIRECT_ACTION_STORE_EEPROM:
#if EEPROM_MODE > 0
        EEPROM::markChanged();
        GUI::setStatusP(PSTR("Settings Saved"), GUIStatusLevel::INFO);
#endif
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_ECHO:
        Printer::toggleEcho();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_INFO:
        Printer::toggleInfo();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_ERRORS:
        Printer::toggleErrors();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_DRYRUN:
        Printer::toggleDryRun();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_NO_MOVES:
        Printer::toggleNoMoves();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_DEBUG_COMMUNICATION:
        Printer::toggleCommunication();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_LIGHT:
        Printer::caseLightMode = Printer::caseLightMode ? 0 : 1;
        break;
    case GUI_DIRECT_ACTION_DISABLE_MOTORS:
        Motion1::waitForEndOfMoves();
        Printer::kill(true);
        break;
    case GUI_DIRECT_ACTION_MOUNT_SD_CARD:
#if SDSUPPORT  
        if (sd.state == SDState::SD_HAS_ERROR) { 
            sd.unmount(true);
        }
        sd.mount(true);
#endif
        break;
    case GUI_DIRECT_ACTION_STOP_PRINT:
        Printer::stopPrint();
        break;
    case GUI_DIRECT_ACTION_PAUSE_PRINT:
        Printer::pausePrint();
        break;
    case GUI_DIRECT_ACTION_CONTINUE_PRINT:
        Printer::continuePrint();
        break;
    case GUI_DIRECT_ACTION_POWERLOSS:
        Printer::handlePowerLoss();
        break;
    case GUI_DIRECT_ACTION_DITTO_OFF:
        PrinterType::setDittoMode(0, false);
        GUI::pop();
        break;
    case GUI_DIRECT_ACTION_DITTO_MIRROR:
        PrinterType::setDittoMode(1, true);
        GUI::pop();
        break;
    case GUI_DIRECT_ACTION_DITTO_2:
    case GUI_DIRECT_ACTION_DITTO_3:
    case GUI_DIRECT_ACTION_DITTO_4:
    case GUI_DIRECT_ACTION_DITTO_5:
    case GUI_DIRECT_ACTION_DITTO_6:
    case GUI_DIRECT_ACTION_DITTO_7:
    case GUI_DIRECT_ACTION_DITTO_8:
        PrinterType::setDittoMode(1 + opt - GUI_DIRECT_ACTION_DITTO_2, false);
        GUI::pop();
        break;
    case GUI_DIRECT_ACTION_TOGGLE_PROBE_PAUSE:
        ZProbeHandler::setHeaterPause(!ZProbeHandler::getHeaterPause());
        break;
    case GUI_DIRECT_ACTION_TOGGLE_AUTORETRACTIONS:
        Printer::setAutoretract(!Printer::isAutoretract(), true);
        break;
    }
}

void selectToolAction(GUIAction action, void* data) {
    int id = reinterpret_cast<int>(data);
    if (!Printer::failedMode) {
        Motion1::waitForEndOfMoves();
        Tool::selectTool(id);
    }
}
