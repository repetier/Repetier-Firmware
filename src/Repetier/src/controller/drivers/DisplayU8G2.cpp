/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#include "Repetier.h"

#if DISPLAY_DRIVER == DRIVER_U8G2
#include "controller/u8g2/cppsrc/U8g2lib.h"
#define UI_SPI_SCK UI_DISPLAY_D4_PIN
#define UI_SPI_MOSI UI_DISPLAY_ENABLE_PIN
#define UI_SPI_CS UI_DISPLAY_RS_PIN
#define UI_DC UI_DISPLAY_D5_PIN
#ifndef DISPLAY_ROTATION
#define DISPLAY_ROTATION U8G2_R0
#endif

#include "controller/images.h"

#if ENABLED(DISPLAY_ST7920_SW)
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_ST7920_128X64_F_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS);
#else
U8G2_ST7920_128X64_2_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS);
#endif
#endif

#if ENABLED(DISPLAY_ST7920_HW)
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_ST7920_128X64_F_HW_SPI lcd(DISPLAY_ROTATION, UI_SPI_CS);
#else
U8G2_ST7920_128X64_2_HW_SPI lcd(DISPLAY_ROTATION, UI_SPI_CS);
#endif
#endif

#if ENABLED(U8GLIB_ST7565_NHD_C2832_HW_SPI)
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_ST7565_NHD_C12864_F_4W_HW_SPI(DISPLAY_ROTATION, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#else
U8G2_ST7565_NHD_C12864_2_4W_HW_SPI(DISPLAY_ROTATION, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#endif
#endif

#if ENABLED(U8GLIB_ST7565_NHD_C2832_SW_SPI)
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_ST7565_NHD_C12864_F_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#else
U8G2_ST7565_NHD_C12864_2_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#endif
#endif

#ifdef U8GLIB_SSD1306_SW_SPI
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_2_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#endif
#endif

#ifdef U8GLIB_SH1106_SW_SPI
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#else
U8G2_SH1106_128X64_NONAME_2_4W_SW_SPI lcd(DISPLAY_ROTATION, UI_SPI_SCK, UI_SPI_MOSI, UI_SPI_CS, UI_DC, U8X8_PIN_NONE);
#endif
#endif

// Pin names differ to u8g so double check when adding this
#ifdef U8GLIB_KS0108
#if ENABLED(DISPLAY_FULL_BUFFER)
U8G2_KS0108_128X64_F lcd(DISPLAY_ROTATION, UI_DISPLAY_D0_PIN, UI_DISPLAY_D1_PIN, UI_DISPLAY_D2_PIN, UI_DISPLAY_D3_PIN, UI_DISPLAY_D4_PIN, UI_DISPLAY_D5_PIN, UI_DISPLAY_D6_PIN, UI_DISPLAY_D7_PIN, UI_DISPLAY_ENABLE_PIN, UI_DISPLAY_DI, UI_DISPLAY_CS0, UI_DISPLAY_CS1, UI_DISPLAY_CS2, UI_DISPLAY_RESET_PIN);
#else
U8G2_KS0108_128X64_2 lcd(DISPLAY_ROTATION, UI_DISPLAY_D0_PIN, UI_DISPLAY_D1_PIN, UI_DISPLAY_D2_PIN, UI_DISPLAY_D3_PIN, UI_DISPLAY_D4_PIN, UI_DISPLAY_D5_PIN, UI_DISPLAY_D6_PIN, UI_DISPLAY_D7_PIN, UI_DISPLAY_ENABLE_PIN, UI_DISPLAY_DI, UI_DISPLAY_CS0, UI_DISPLAY_CS1, UI_DISPLAY_CS2, UI_DISPLAY_RESET_PIN);
#endif
#endif

millis_t init100msTicks = 0;
void GUI::init() {
    init100msTicks = 0;
}
void GUI::processInit() {
    if (++init100msTicks < 1 || curBootState != GUIBootState::DISPLAY_INIT) { // 100 ms
        return;
    }

    lcd.begin();
    handleKeypress();
    nextAction = GUIAction::NONE;
    callbacks[0] = startScreen;
    pageType[0] = GUIPageType::FIXED_CONTENT;
    bufClear();
    setStatusP(PSTR("Ready"), GUIStatusLevel::REGULAR);
    HAL::delayMilliseconds(10);
    lcd.firstPage();
    do {
        lcd.setFont(u8g2_font_6x10_mf);
        lcd.drawXBM(0, 0, LOGO_width, LOGO_height, LOGO_bits);
        bufClear();
        bufAddStringP(PSTR(REPETIER_VERSION));
        lcd.drawUTF8(20, 55, buf);
    } while (lcd.nextPage());
    lastRefresh = HAL::timeInMilliseconds() + UI_START_SCREEN_DELAY; // Show start screen 4s but will not delay start process
    curBootState = GUIBootState::IN_INTRO;
}

static fast8_t refresh_counter = 0;

void GUI::refresh() {
    /* draw something on the display with the `firstPage()`/`nextPage()` loop*/
    refresh_counter++;
    lcd.firstPage();
    do {
        lcd.setFontMode(0);
        callbacks[level](GUIAction::DRAW, data[level]);
    } while (lcd.nextPage());
}

// Draw top line status
void __attribute__((weak)) drawStatusLine() {
    GUI::bufClear();
    Tool* t = Tool::getActiveTool();
    HeatManager* hm = t->getHeater();
    if (hm != nullptr) { // E1:210/210°C
        GUI::bufAddChar('E');
        GUI::bufAddInt(t->getToolId() + 1, 1);
        GUI::bufAddChar(':');
        GUI::bufAddHeaterTemp(hm, true);
        GUI::bufAddChar(' ');
    }
#if NUM_HEATED_BEDS > 0
    GUI::bufAddStringP(PSTR("B:"));
    GUI::bufAddHeaterTemp(heatedBeds[0], true);
#endif
    lcd.setFont(u8g2_font_5x7_mf);
    lcd.drawUTF8(1, 7, GUI::buf);
    lcd.drawHLine(0, 8, 128);
}

// Draw menu functions - driver specific

static bool npActionFound;
static int guiLine;
static int guiY;
static int guiSelIndex;

static fast8_t textScrollWaits = 0;
static fast8_t textScrollPos = 0;
static bool textScrollDir = false;

static void scrollSelectedText(fast8_t x, fast8_t y) {
    fast8_t charWidth = lcd.getMaxCharWidth();
    fast8_t lastCharPos = (GUI::bufPos * charWidth);
    if (lastCharPos > (128 - x)) {
        if (!GUI::textIsScrolling) {
            // Wait a few refresh ticks before starting scroll.
            textScrollWaits = 2;
            GUI::textIsScrolling = true;
        }
        if (textScrollWaits <= 0) {
            // One extra charWidth/2 move at the end to show more.
            fast8_t maxScrollX = ((128 - lastCharPos) - x) - (charWidth / 2);
            constexpr fast8_t scrollIncr = 4;
            if (!textScrollDir) {
                if ((textScrollPos -= scrollIncr) <= maxScrollX) {
                    textScrollDir = true; // Left scroll
                    textScrollWaits = 2;
                }
            } else {
                if ((textScrollPos += scrollIncr) >= (charWidth / 2)) {
                    textScrollDir = false; // Right scroll back
                    textScrollWaits = 1;
                }
            }
        } else {
            --textScrollWaits;
        }
        lcd.drawUTF8(x + textScrollPos, y, GUI::buf);
    } else {
        lcd.drawUTF8(x, y, GUI::buf);
    }
}
void GUI::menuStart(GUIAction action) {
    if (npActionFound) {
        if (GUI::textIsScrolling) {
            GUI::textIsScrolling = false;
            textScrollDir = false;
            textScrollPos = textScrollWaits = 0;
        }
    }
    npActionFound = false;
    guiLine = 0;
    guiSelIndex = cursorRow[level];
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_6x10_mf);
        guiY = 10;
    }
}

void GUI::menuEnd(GUIAction action) {
    if (action == GUIAction::NEXT) {
        if (cursorRow[level] - topRow[level] > 4) {
            topRow[level] = cursorRow[level] - 4;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (cursorRow[level] <= topRow[level]) {
            topRow[level] = RMath::max(static_cast<int32_t>(0), static_cast<int32_t>(cursorRow[level] - 1));
        }
    } else if (action == GUIAction::ANALYSE) {
        if (topRow[level] < 0) {
            topRow[level] = 0;
        }
        if (cursorRow[level] < 0) {
            cursorRow[level] = 0;
        }
    }
}

#define TEST_MENU_CLICK \
    if (guiLine == cursorRow[level]) { /* Actions for active line only!*/ \
        if (action == GUIAction::CLICK) { \
            if (tp == GUIPageType::POP) { /* Leave menu */ \
                pop(); \
            } else if (cb != nullptr && tp == GUIPageType::ACTION) { /* Execute a direct action */ \
                cb(action, cData); \
            } else if (cb) { /* Push new display function on stack */ \
                push(cb, cData, tp); \
            } \
            action = GUIAction::CLICK_PROCESSED; \
        } \
    }

void GUI::menuTextP(GUIAction& action, PGM_P text, bool highlight) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddStringP(text);
            if (highlight) {
                guiY += 12;
                lcd.drawBox(0, guiY - 9, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(1, guiY - 1, GUI::buf);
                lcd.setDrawColor(1);
            } else {
                guiY += 10;
                if (guiLine == cursorRow[level]) {
                    lcd.drawBox(0, guiY - 8, 128, 10);
                    lcd.setDrawColor(0);
                    scrollSelectedText(1, guiY);
                    lcd.setDrawColor(1);
                } else {
                    lcd.drawUTF8(1, guiY, GUI::buf);
                }
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!highlight && !npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (!highlight && guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    }
    guiLine++;
}

void GUI::menuFloatP(GUIAction& action, PGM_P text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddStringP(text);
            bufAddFloat(val, 0, precision);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuLongP(GUIAction& action, PGM_P text, long val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddStringP(text);
            bufAddLong(val, 0);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuOnOffP(GUIAction& action, PGM_P text, bool val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddStringP(text);
            if (val) {
                bufAddStringP(PSTR("On"));
            } else {
                bufAddStringP(PSTR("Off"));
            }
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuSelectableP(GUIAction& action, PGM_P text, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        if (cursorRow[level] < 0) {
            cursorRow[level] = length[level];
        }
        maxCursorRow[level] = length[level];
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            GUI::bufClear();
            GUI::bufAddStringP(text);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                scrollSelectedText(10, guiY);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuText(GUIAction& action, char* text, bool highlight) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddString(text);
            if (highlight) {
                guiY += 12;
                lcd.drawBox(0, guiY - 9, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(1, guiY - 1, GUI::buf);
                lcd.setDrawColor(1);
            } else {
                guiY += 10;
                if (guiLine == cursorRow[level]) {
                    lcd.drawBox(0, guiY - 8, 128, 10);
                    lcd.setDrawColor(0);
                    scrollSelectedText(1, guiY);
                    lcd.setDrawColor(1);
                } else {
                    lcd.drawUTF8(1, guiY, GUI::buf);
                }
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!highlight && !npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (!highlight && guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    }
    guiLine++;
}

void GUI::menuFloat(GUIAction& action, char* text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddString(text);
            bufAddFloat(val, 0, precision);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuLong(GUIAction& action, char* text, long val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddString(text);
            bufAddLong(val, 0);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuOnOff(GUIAction& action, char* text, bool val, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            bufClear();
            bufAddString(text);
            if (val) {
                bufAddStringP(PSTR("On"));
            } else {
                bufAddStringP(PSTR("Off"));
            }
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                lcd.drawUTF8(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void GUI::menuSelectable(GUIAction& action, char* text, GuiCallback cb, void* cData, GUIPageType tp) {
    if (action == GUIAction::ANALYSE) {
        if (cursorRow[level] < 0) {
            cursorRow[level] = length[level];
        }
        maxCursorRow[level] = length[level];
        length[level]++;
    } else if (action == GUIAction::DRAW) {
        if (guiLine >= topRow[level] && guiLine < topRow[level] + 5) {
            GUI::bufClear();
            GUI::bufAddString(text);
            guiY += 10;
            if (guiLine == cursorRow[level]) {
                lcd.drawBox(0, guiY - 8, 128, 10);
                lcd.setDrawColor(0);
                scrollSelectedText(10, guiY);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawUTF8(10, guiY, GUI::buf);
            }
        }
    } else if (action == GUIAction::NEXT) {
        if (!npActionFound && guiLine > cursorRow[level]) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else if (action == GUIAction::PREVIOUS) {
        if (guiLine < guiSelIndex) {
            cursorRow[level] = guiLine;
            npActionFound = true;
            contentChanged = true;
        }
    } else {
        TEST_MENU_CLICK
    }
    guiLine++;
}

void drawPStr(PGM_P flash, int x, int y, int dxLen = 0) {
    char text[MAX_COLS + 1];
    int pos = 0;
    while (pos < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(flash++);
        if (c == 0)
            break;
        text[pos++] = c;
    }
    x -= dxLen * pos;
    text[pos] = 0;
    lcd.drawUTF8(x, y, text);
}

// Value modifyer display
void GUI::showValueP(PGM_P text, PGM_P unit, char* value) {
    // Value y 24 - 48, x 0 - 127

    lcd.setFont(u8g2_font_10x20_mf);
    int len = strlen(value);
    lcd.drawUTF8(125 - 10 * len, 39, value);
    lcd.drawFrame(0, 22, 128, 21);

    // changes buf so use after drawing value!
    drawStatusLine();
    // Head
    lcd.setFont(u8g2_font_6x10_mf);
    drawPStr(text, 0, 19);

    // Unit
    lcd.setFont(u8g2_font_5x7_mf);
    drawPStr(unit, 125, 51, 5);
    // Ok hint
    lcd.setFont(u8g2_font_6x10_mf);
    lcd.setDrawColor(0);
    GUI::bufClear();
    GUI::bufAddStringP(Com::tBtnOK);
    lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
    lcd.setDrawColor(1);
}

void GUI::showValue(char* text, PGM_P unit, char* value) {
    // Value y 24 - 48, x 0 - 127

    lcd.setFont(u8g2_font_10x20_mf);
    int len = strlen(value);
    lcd.drawUTF8(125 - 10 * len, 39, value);
    lcd.drawFrame(0, 22, 128, 21);

    // changes buf so use after drawing value!
    drawStatusLine();
    // Head
    lcd.setFont(u8g2_font_6x10_mf);
    lcd.drawUTF8(1, 19, text);

    // Unit
    lcd.setFont(u8g2_font_5x7_mf);
    drawPStr(unit, 125, 51, 5);
    // Ok hint
    lcd.setFont(u8g2_font_6x10_mf);
    lcd.setDrawColor(0);
    GUI::bufClear();
    GUI::bufAddStringP(Com::tBtnOK);
    lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
    lcd.setDrawColor(1);
}

void GUI::showScrollbar(GUIAction& action, float percent, uint16_t min, uint16_t max) {
    if (action == GUIAction::DRAW) {
        if (min == max) {
            return;
        }
        static float lastPos = 0.0f;
        static millis_t lastUpdate = 0ul;
        uint16_t pxSize = static_cast<uint16_t>((static_cast<float>(min) / static_cast<float>(max)) * 35.0f);
        if (pxSize < 5.0f) {
            pxSize = 5.0f;
        }
        uint16_t pxPos = static_cast<uint16_t>(percent * (63.0f - pxSize - 10.0f));
        if (percent != lastPos) {
            lastUpdate = HAL::timeInMilliseconds();
        } else if ((HAL::timeInMilliseconds() - lastUpdate) > 750ul) {
            return;
        }
        lastPos = percent;
        lcd.setDrawColor(0u);
        lcd.drawBox(128u - 8u + 3u, 9u, 8u, 64u - 9u);
        lcd.setDrawColor(1u);
        lcd.drawVLine(128u - 9u + 3u, 9u, 64u - 9u);
        lcd.drawBox(128u - 7u + 3u, 10u + pxPos, 3u, pxSize);
    }
}
void GUI::showScrollbar(GUIAction& action) {
    if (action == GUIAction::DRAW) {
        if (length[level] > 5) {
            float percent = static_cast<float>(topRow[level]) / static_cast<float>(length[level] - 5);
            showScrollbar(action, percent, 5u, static_cast<uint16_t>(length[level]));
        }
    }
}
void __attribute__((weak)) probeProgress(GUIAction action, void* data) {

    if (action == GUIAction::DRAW) {
        if (Printer::isZProbingActive() && GUI::curProbingProgress) {
            drawStatusLine();

            static ufast8_t dotCounter = 0;
            static millis_t lastDotTime = 0;
            if ((HAL::timeInMilliseconds() - lastDotTime) >= 1000ul) {
                dotCounter++;
                lastDotTime = HAL::timeInMilliseconds();
            }

            constexpr size_t barWidth = 116u;                   // width across entire screen left/right
            constexpr size_t barLeft = (64u - (barWidth / 2u)); // margin from left in order to center the bar
            constexpr size_t barTop = 21u;                      // margin from top of screen
            constexpr size_t barHeight = 14u;                   // height amount expanded downwards

            constexpr size_t probeTxtBase = 3u; // margin above bar
            constexpr size_t probeTxtLeft = 2u; // margin from bar left

            constexpr size_t percentTxtRight = 5u; // margin from bar right

            constexpr size_t countTxtBase = 2u;  // margin above bar
            constexpr size_t countTxtRight = 3u; // margin from bar right

            constexpr size_t zHitTxtBase = 6u;  // margin under bar
            constexpr size_t zHitTxtRight = 4u; // margin from bar right

            constexpr size_t coordTxtBase = 2u; // margin under bar
            constexpr size_t coordTxtLeft = 4u; // margin from bar left
            constexpr size_t coordTxtGap = 3u;  // margin vertical gap between X/Y coords strings

            constexpr size_t vLineSeperatorLeft = 10u; // margin away from coordtxt. (from final coord char)

            const float progress = ((static_cast<float>(GUI::curProbingProgress->num) * 100.0f) / static_cast<float>(GUI::curProbingProgress->maxNum));

            // DRAW PROBING TITLE & ANIM
            GUI::bufClear();
            GUI::bufAddStringP(Com::tProbing);
            size_t len = dotCounter % 4u;
            for (size_t i = 0u; i < 3u; i++) {
                GUI::bufAddChar(i < len ? '.' : ' ');
            }
            lcd.drawUTF8(barLeft + probeTxtLeft, barTop - probeTxtBase, GUI::buf);
            // END DRAW PROBING TITLE & ANIM

            // DRAW FRAME & BOX
            lcd.drawFrame(barLeft, barTop, barWidth, barHeight);
            lcd.drawBox(barLeft, barTop, static_cast<u8g2_uint_t>(barWidth * progress * 0.01f), barHeight);
            // END DRAW FRAME & BOX

            // DRAW POINTS OUT OF POINTS REMAINING
            GUI::bufClear();
            GUI::bufAddInt(GUI::curProbingProgress->num, 3);
            GUI::bufAddStringP(Com::tSlash);
            GUI::bufAddInt(GUI::curProbingProgress->maxNum, 3);

            len = (GUI::bufPos * 5u);
            lcd.drawUTF8(((barLeft + barWidth) - len) - countTxtRight, barTop - countTxtBase, GUI::buf);
            // END DRAW POINTS OUT OF POINTS REMAINING

            // DRAW X AND Y COORDINATES
            GUI::bufClear();
            lcd.setFontPosTop(); // Must offset by -1 to the font height from now on.
            constexpr u8g2_uint_t yPx = barTop + barHeight + coordTxtBase;

            GUI::bufAddStringP(Com::tXColon);
            GUI::bufAddLong(GUI::curProbingProgress->x, 3);
            GUI::bufAddStringP(Com::tUnitMM);

            len = (GUI::bufPos * 5u);
            lcd.drawUTF8(barLeft + coordTxtLeft, yPx, GUI::buf);

            // DRAW V SEPERATOR
            lcd.drawVLine((barLeft + coordTxtLeft) + len + vLineSeperatorLeft, yPx + 1u, (((7u - 1u) * 2u) + coordTxtGap));
            // END DRAW V SEPERATOR

            GUI::bufClear();
            GUI::bufAddStringP(Com::tYColon);
            GUI::bufAddLong(GUI::curProbingProgress->y, 3);
            GUI::bufAddStringP(Com::tUnitMM);
            lcd.drawUTF8(barLeft + coordTxtLeft, (yPx + (7u - 1u)) + coordTxtGap, GUI::buf);
            // END DRAW X AND Y COORDINATES

            // DRAW STATUS EARLY
            GUI::bufClear();
            lcd.drawUTF8(1u, 61u - (7u - 1u), GUI::status);
            // END DRAW STATUS EARLY

            // Reduce font changes, process these after everything else:
            // DRAW PROGRESS PERCENTAGE - CHANGE TO 6x10
            lcd.setFont(u8g2_font_6x10_mf);
            lcd.setFontMode(1u);
            lcd.setDrawColor(2u);
            GUI::bufClear();

            GUI::bufAddFloat(progress, 3, 1);
            GUI::bufAddStringP(Com::tUnitPercent);
            len = (GUI::bufPos * 6u);

            constexpr u8g2_uint_t center = barTop + ((barHeight - (10u - 1u)) / 2u);
            lcd.drawUTF8(((barLeft + barWidth) - len) - percentTxtRight, center, GUI::buf);
            lcd.setDrawColor(1u);
            lcd.setFontMode(0u);
            // END DRAW PROGRESS PERCENTAGE

            // DRAW PROBE HIT POINT
            GUI::bufClear();
            if (GUI::curProbingProgress->z != IGNORE_COORDINATE) {
                GUI::bufAddChar('(');
                if (GUI::curProbingProgress->z != ILLEGAL_Z_PROBE) {
                    if (GUI::curProbingProgress->z >= 0.0f) {
                        GUI::bufAddChar('+');
                    }
                    GUI::bufAddFloat(GUI::curProbingProgress->z, 1, 2);
                    GUI::bufAddStringP(Com::tUnitMM);
                } else {
                    GUI::bufAddStringP(PSTR("Illegal"));
                }
                GUI::bufAddChar(')');

                len = (GUI::bufPos * 6u);
                lcd.drawUTF8(((barLeft + barWidth) - len) - zHitTxtRight, (barTop + barHeight) + zHitTxtBase, GUI::buf);
            }
            // END DRAW PROBE HIT POINT

            lcd.setFontPosBaseline(); // Revert font pos to default, otherwise it affects all other gui calls.
        } else {
            lcd.setFontPosBaseline();
            startScreen(action, data);
        }
    }
    GUI::replaceOn(GUIAction::NEXT, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}
//extern void __attribute__((weak)) startScreen(GUIAction action, void* data);
//extern void __attribute__((weak)) printProgress(GUIAction action, void* data);
// extern void __attribute__((weak)) mainMenu(GUIAction action, void* data);
//void mainMenu(GUIAction action, void* data) __attribute__((weak, alias("WmainMenu")));
//void printProgress(GUIAction action, void* data) __attribute__((weak, alias("WprintProgress")));
//void startScreen(GUIAction action, void* data) __attribute__((weak, alias("WstartScreen")));

void __attribute__((weak)) startScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        // lcd.setFont(u8g2_font_6x10_mf);
        drawStatusLine();
        lcd.drawVLine(64, 8, 46);
        lcd.setFont(u8g2_font_5x7_mf);

        // X position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("X:"));
        if (Motion1::isAxisHomed(X_AXIS) || (refresh_counter & 1) == 0) {
            GUI::bufAddFloat(Motion1::getShowPosition(X_AXIS), 4, 2);
        } else {
            GUI::bufAddStringP(PSTR("????.??"));
        }
        lcd.drawUTF8(66, 16, GUI::buf);
        // Y position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Y:"));
        if (Motion1::isAxisHomed(Y_AXIS) || (refresh_counter & 1) == 0) {
            GUI::bufAddFloat(Motion1::getShowPosition(Y_AXIS), 4, 2);
        } else {
            GUI::bufAddStringP(PSTR("????.??"));
        }
        lcd.drawUTF8(66, 23, GUI::buf);
        // Z position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Z:"));
        if (Motion1::isAxisHomed(Z_AXIS) || (refresh_counter & 1) == 0) {
            GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 4, 2);
        } else {
            GUI::bufAddStringP(PSTR("????.??"));
        }
        lcd.drawUTF8(66, 30, GUI::buf);
        // FeedRate multiplier
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("FR:"));
        GUI::bufAddInt(Printer::feedrateMultiply, 3);
        GUI::bufAddChar('%');
        lcd.drawUTF8(66, 37, GUI::buf);

        // Left take first 6 that make sense

        int n = 0;

        Tool* tool = Tool::getActiveTool();
        if (NUM_TOOLS > 1 && NUM_TOOLS <= 4 && tool->getToolType() == ToolTypes::EXTRUDER) {
            for (int e = 0; e < NUM_TOOLS; e++) {
                Tool* at = Tool::getTool(e);
                if (at->getToolType() == ToolTypes::EXTRUDER && n < 6) {
                    GUI::bufClear();
                    GUI::bufAddChar('E');
                    GUI::bufAddInt(at->getToolId() + 1, 1);
                    GUI::bufAddChar(':');
                    GUI::bufAddHeaterTemp(at->getHeater(), true);
                    lcd.drawUTF8(1, 16 + n * 7, GUI::buf);
                    n++;
                }
            }
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && tool->hasSecondary() && tool->secondaryIsFan() && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Fan:");
            GUI::bufAddInt(tool->secondaryPercent(), 3);
            GUI::bufAddChar('%');
            lcd.drawUTF8(1, 16 + n * 7, GUI::buf);
            n++;
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Ditto:");
            GUI::bufAddInt(Motion1::dittoMode, 1);
            if (Motion1::dittoMirror) {
                GUI::bufAddChar('M');
            }
            lcd.drawUTF8(1, 16 + n * 7, GUI::buf);
            n++;
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Flow:");
            GUI::bufAddInt(Printer::extrudeMultiply, 3);
            GUI::bufAddChar('%');
            lcd.drawUTF8(1, 16 + n * 7, GUI::buf);
            n++;
        }
        if (n < 6 && Printer::areAllSteppersDisabled()) {
            GUI::bufClear();
            GUI::bufAddStringP("Motors off");
            lcd.drawUTF8(1, 16 + n * 7, GUI::buf);
            n++;
        }
        lcd.setFont(u8g2_font_6x10_mf);
        lcd.drawUTF8(1, 61, GUI::status);
    }
    if (Printer::isPrinting() || Printer::isZProbingActive()) {
        GuiCallback cb = Printer::isPrinting() ? printProgress : probeProgress;
        GUI::replaceOn(GUIAction::NEXT, cb, nullptr, GUIPageType::FIXED_CONTENT);
        GUI::replaceOn(GUIAction::PREVIOUS, cb, nullptr, GUIPageType::FIXED_CONTENT);
    }
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) printProgress(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        if (Printer::isPrinting()) {
#if SDSUPPORT
            if (sd.state == SDState::SD_PRINTING) { // print from sd card
                Printer::progress = (static_cast<float>(sd.selectedFilePos) * 100.0) / static_cast<float>(sd.selectedFileSize);
            }
#endif
            lcd.setFont(u8g2_font_6x10_mf);
            int len = strlen(Printer::printName);
            lcd.drawUTF8(64 - 3 * len, 8, Printer::printName); // Draw centered
            lcd.drawHLine(0, 9, 128);
            lcd.drawVLine(64, 9, 45);
            lcd.setFont(u8g2_font_5x7_mf);
            int y0 = 17;

            GUI::bufClear();
            GUI::bufAddStringP(PSTR("Prog:"));
            GUI::bufAddFloat(Printer::progress, 3, 1);
            GUI::bufAddStringP(PSTR(" %"));
            lcd.drawUTF8(1, y0 + 1, GUI::buf);
            lcd.drawFrame(1, y0 + 5, 62, 9);
            lcd.drawBox(1, y0 + 6, static_cast<u8g2_uint_t>(62.0 * Printer::progress * 0.01), 7);

            if (Printer::maxLayer > 0) {
                GUI::bufClear();
                GUI::bufAddStringP(PSTR("Layer:"));
                lcd.drawUTF8(1, 42, GUI::buf);
                GUI::bufClear();
                GUI::bufAddInt(Printer::currentLayer, 5);
                GUI::bufAddStringP(Com::tSlash);
                GUI::bufAddInt(Printer::maxLayer, 5);
                lcd.drawUTF8(1, 50, GUI::buf);
            }
            // Right side, try to show Z, E, B, Fan, Chamber, Multiplier
            int n = 0;

            // Z-Position
            GUI::bufClear();
            GUI::bufAddStringP(PSTR("Z :"));
            GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 4, 2);
            lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
            n++;

            // Active extruder
            Tool* tool = Tool::getActiveTool();
            if (tool->getToolType() == ToolTypes::EXTRUDER) {
                if (tool->getToolType() == ToolTypes::EXTRUDER && n < 6) {
                    GUI::bufClear();
                    GUI::bufAddChar('E');
                    GUI::bufAddInt(tool->getToolId() + 1, 1);
                    GUI::bufAddChar(':');
                    GUI::bufAddHeaterTemp(tool->getHeater(), true);
                    lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                    n++;
                }
#if NUM_HEATED_BEDS
                GUI::bufClear();
                GUI::bufAddStringP(PSTR("B :"));
                GUI::bufAddHeaterTemp(heatedBeds[0], true);
                lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                n++;
#endif
#if NUM_HEATED_CHAMBERS
                GUI::bufClear();
                GUI::bufAddChar('C');
                GUI::bufAddChar(':');
                GUI::bufAddHeaterTemp(heatedChambers[0], true);
                lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                n++;
#endif
            }
            if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && tool->hasSecondary() && tool->secondaryIsFan() && n < 6) {
                GUI::bufClear();
                GUI::bufAddStringP("Fan:");
                GUI::bufAddInt(tool->secondaryPercent(), 3);
                GUI::bufAddChar('%');
                lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                n++;
            }
            if (n < 6) {
                GUI::bufClear();
                GUI::bufAddStringP(PSTR("FR :"));
                GUI::bufAddInt(Printer::feedrateMultiply, 3);
                GUI::bufAddChar('%');
                lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                n++;
            }
            if (n < 6) {
                GUI::bufClear();
                GUI::bufAddStringP(PSTR("Buf:"));
                GUI::bufAddInt(static_cast<int>(Motion1::length), 3);
                lcd.drawUTF8(66, y0 + n * 7, GUI::buf);
                n++;
            }
            lcd.setFont(u8g2_font_6x10_mf);
            lcd.drawUTF8(1, 61, GUI::status);
        } else {
            startScreen(action, data); // print is finished!
        }
    }
    GUI::replaceOn(GUIAction::NEXT, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) warningScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, WARNING_width, WARNING_height, WARNING_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Warning"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}

void __attribute__((weak)) errorScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, ERROR_width, ERROR_height, ERROR_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Error"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}

void __attribute__((weak)) infoScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, INFO_width, INFO_height, INFO_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Info"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}

void __attribute__((weak)) warningScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, WARNING_width, WARNING_height, WARNING_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Warning"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        int len = strlen(static_cast<char*>(GUI::buf));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(GUI::buf));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}
void __attribute__((weak)) errorScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, ERROR_width, ERROR_height, ERROR_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Error"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        int len = strlen(static_cast<char*>(GUI::buf));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(GUI::buf));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}

void __attribute__((weak)) infoScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, INFO_width, INFO_height, INFO_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Info"));
        lcd.drawUTF8(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        GUI::bufClear();
        GUI::bufAddStringP((const char*)data);
        int len = strlen(static_cast<char*>(GUI::buf));
        lcd.drawUTF8(64 - 3 * len, 50, static_cast<char*>(GUI::buf));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(Com::tBtnOK);
        lcd.drawUTF8(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
        lcd.setDrawColor(1);
    } else if (action == GUIAction::CLICK || action == GUIAction::BACK) {
        GUI::pop();
    }
}

#define SPIN_CENTER_X 10
#define SPIN_CENTER_Y 38

void waitScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        switch (refresh_counter % 9) {
        case 0:
            lcd.drawXBM(SPIN_CENTER_X - SPIN1_width / 2, SPIN_CENTER_Y - SPIN1_height / 2, SPIN1_width, SPIN1_height, SPIN1_bits);
            break;
        case 1:
            lcd.drawXBM(SPIN_CENTER_X - SPIN2_width / 2, SPIN_CENTER_Y - SPIN2_height / 2, SPIN2_width, SPIN2_height, SPIN2_bits);
            break;
        case 2:
            lcd.drawXBM(SPIN_CENTER_X - SPIN3_width / 2, SPIN_CENTER_Y - SPIN3_height / 2, SPIN3_width, SPIN3_height, SPIN3_bits);
            break;
        case 3:
            lcd.drawXBM(SPIN_CENTER_X - SPIN4_width / 2, SPIN_CENTER_Y - SPIN4_height / 2, SPIN4_width, SPIN4_height, SPIN4_bits);
            break;
        case 4:
            lcd.drawXBM(SPIN_CENTER_X - SPIN5_width / 2, SPIN_CENTER_Y - SPIN5_height / 2, SPIN5_width, SPIN5_height, SPIN5_bits);
            break;
        case 5:
            lcd.drawXBM(SPIN_CENTER_X - SPIN6_width / 2, SPIN_CENTER_Y - SPIN6_height / 2, SPIN6_width, SPIN6_height, SPIN6_bits);
            break;
        case 6:
            lcd.drawXBM(SPIN_CENTER_X - SPIN7_width / 2, SPIN_CENTER_Y - SPIN7_height / 2, SPIN7_width, SPIN7_height, SPIN7_bits);
            break;
        case 7:
            lcd.drawXBM(SPIN_CENTER_X - SPIN8_width / 2, SPIN_CENTER_Y - SPIN8_height / 2, SPIN8_width, SPIN8_height, SPIN8_bits);
            break;
        case 8:
            lcd.drawXBM(SPIN_CENTER_X - SPIN9_width / 2, SPIN_CENTER_Y - SPIN9_height / 2, SPIN9_width, SPIN9_height, SPIN9_bits);
            refresh_counter = 8;
            break;
        }
        char* str = static_cast<char*>(data); 
        int len = strlen(str);
        if (len <= 10) {
            lcd.setFont(u8g2_font_10x20_mf);
            lcd.drawUTF8(73 - len * 5, SPIN_CENTER_Y + 7, str);
        } else { 
            char* newLine = strchr(str, '\n');
            lcd.setFont(u8g2_font_6x10_mf);
            if (newLine) { 
                *newLine = '\0';
                uint8_t newLineLen = (newLine - str);
                lcd.drawUTF8(73 - newLineLen * 3, (SPIN_CENTER_Y + 3) - 5, str);
                *newLine = '\n';
                lcd.drawUTF8(73 - (len - newLineLen) * 3, (SPIN_CENTER_Y + 3) + 5, str + newLineLen + 1);
            } else {
                lcd.drawUTF8(73 - len * 3, SPIN_CENTER_Y + 3, str); 
            }
        }
    }
}

void waitScreenP(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        switch (refresh_counter % 9) {
        case 0:
            lcd.drawXBM(SPIN_CENTER_X - SPIN1_width / 2, SPIN_CENTER_Y - SPIN1_height / 2, SPIN1_width, SPIN1_height, SPIN1_bits);
            break;
        case 1:
            lcd.drawXBM(SPIN_CENTER_X - SPIN2_width / 2, SPIN_CENTER_Y - SPIN2_height / 2, SPIN2_width, SPIN2_height, SPIN2_bits);
            break;
        case 2:
            lcd.drawXBM(SPIN_CENTER_X - SPIN3_width / 2, SPIN_CENTER_Y - SPIN3_height / 2, SPIN3_width, SPIN3_height, SPIN3_bits);
            break;
        case 3:
            lcd.drawXBM(SPIN_CENTER_X - SPIN4_width / 2, SPIN_CENTER_Y - SPIN4_height / 2, SPIN4_width, SPIN4_height, SPIN4_bits);
            break;
        case 4:
            lcd.drawXBM(SPIN_CENTER_X - SPIN5_width / 2, SPIN_CENTER_Y - SPIN5_height / 2, SPIN5_width, SPIN5_height, SPIN5_bits);
            break;
        case 5:
            lcd.drawXBM(SPIN_CENTER_X - SPIN6_width / 2, SPIN_CENTER_Y - SPIN6_height / 2, SPIN6_width, SPIN6_height, SPIN6_bits);
            break;
        case 6:
            lcd.drawXBM(SPIN_CENTER_X - SPIN7_width / 2, SPIN_CENTER_Y - SPIN7_height / 2, SPIN7_width, SPIN7_height, SPIN7_bits);
            break;
        case 7:
            lcd.drawXBM(SPIN_CENTER_X - SPIN8_width / 2, SPIN_CENTER_Y - SPIN8_height / 2, SPIN8_width, SPIN8_height, SPIN8_bits);
            break;
        case 8:
            lcd.drawXBM(SPIN_CENTER_X - SPIN9_width / 2, SPIN_CENTER_Y - SPIN9_height / 2, SPIN9_width, SPIN9_height, SPIN9_bits);
            refresh_counter = 8;
            break;
        }
        GUI::bufClear();
        GUI::bufAddStringP(static_cast<const char*>(data));
        int len = GUI::bufPos;
        if (len <= 10) {
            lcd.setFont(u8g2_font_10x20_mf);
            lcd.drawUTF8(73 - len * 5, SPIN_CENTER_Y + 7, static_cast<char*>(GUI::buf));
        } else {
            char* newLine = strchr(GUI::buf, '\n');
            lcd.setFont(u8g2_font_6x10_mf);
            if (newLine) { 
                *newLine = '\0';
                uint8_t newLineLen = (newLine - GUI::buf);
                lcd.drawUTF8(73 - newLineLen * 3, (SPIN_CENTER_Y + 3) - 5, GUI::buf);
                *newLine = '\n';
                lcd.drawUTF8(73 - (len - newLineLen) * 3, (SPIN_CENTER_Y + 3) + 5, GUI::buf + newLineLen + 1);
            } else {
                lcd.drawUTF8(73 - len * 3, SPIN_CENTER_Y + 3, GUI::buf); 
            }
        }
    }
}
#endif
