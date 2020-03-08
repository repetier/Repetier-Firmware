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
#include "u8g2/U8g2lib.h"
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

void GUI::init() {
    HAL::delayMilliseconds(50);
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
    lcd.drawUTF8(0, 6, GUI::buf);
    lcd.drawHLine(0, 8, 128);
}

// Draw menu functions - driver specific

static bool npActionFound;
static int guiLine;
static int guiY;
static int guiSelIndex;

void GUI::menuStart(GUIAction action) {
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
            GUI::nextAction = GUIAction::CLICK_PROCESSED; \
            if (tp == GUIPageType::POP) { /* Leave menu */ \
                pop(); \
            } else if (cb != nullptr && tp == GUIPageType::ACTION) { /* Execute a direct action */ \
                cb(action, cData); \
            } else if (cb) { /* Push new display function on stack */ \
                push(cb, cData, tp); \
            } \
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
                lcd.drawUTF8(0, guiY - 1, GUI::buf);
                lcd.setDrawColor(1);
            } else {
                guiY += 10;
                if (guiLine == cursorRow[level]) {
                    lcd.drawBox(0, guiY - 8, 128, 10);
                    lcd.setDrawColor(0);
                }
                lcd.drawUTF8(0, guiY, GUI::buf);
                if (guiLine == cursorRow[level]) {
                    lcd.setDrawColor(1);
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
                lcd.drawUTF8(0, guiY - 1, GUI::buf);
                lcd.setDrawColor(1);
            } else {
                guiY += 10;
                if (guiLine == cursorRow[level]) {
                    lcd.drawBox(0, guiY - 8, 128, 10);
                    lcd.setDrawColor(0);
                }
                lcd.drawUTF8(0, guiY, GUI::buf);
                if (guiLine == cursorRow[level]) {
                    lcd.setDrawColor(1);
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
    } else
        TEST_MENU_CLICK
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
    lcd.drawUTF8(0, 19, text);

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
        GUI::bufAddFloat(Motion1::getShowPosition(X_AXIS), 4, 2);
        lcd.drawUTF8(66, 16, GUI::buf);
        // Y position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Y:"));
        GUI::bufAddFloat(Motion1::getShowPosition(Y_AXIS), 4, 2);
        lcd.drawUTF8(66, 23, GUI::buf);
        // Z position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Z:"));
        GUI::bufAddFloat(Motion1::getShowPosition(Z_AXIS), 4, 2);
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
                    lcd.drawUTF8(0, 16 + n * 7, GUI::buf);
                    n++;
                }
            }
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && tool->hasSecondary() && tool->secondaryIsFan() && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Fan:");
            GUI::bufAddInt(tool->secondaryPercent(), 3);
            GUI::bufAddChar('%');
            lcd.drawUTF8(0, 16 + n * 7, GUI::buf);
            n++;
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Ditto:");
            GUI::bufAddInt(Motion1::dittoMode, 1);
            if (Motion1::dittoMirror) {
                GUI::bufAddChar('M');
            }
            lcd.drawUTF8(0, 16 + n * 7, GUI::buf);
            n++;
        }
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && n < 6) {
            GUI::bufClear();
            GUI::bufAddStringP("Flow:");
            GUI::bufAddInt(Printer::extrudeMultiply, 3);
            GUI::bufAddChar('%');
            lcd.drawUTF8(0, 16 + n * 7, GUI::buf);
            n++;
        }
        if (n < 6 && Printer::areAllSteppersDisabled()) {
            GUI::bufClear();
            GUI::bufAddStringP("Motors off");
            lcd.drawUTF8(0, 16 + n * 7, GUI::buf);
            n++;
        }
        lcd.setFont(u8g2_font_6x10_mf);
        lcd.drawUTF8(0, 61, GUI::status);
    }
    if (Printer::isPrinting()) {
        GUI::replaceOn(GUIAction::NEXT, printProgress, nullptr, GUIPageType::FIXED_CONTENT);
        GUI::replaceOn(GUIAction::PREVIOUS, printProgress, nullptr, GUIPageType::FIXED_CONTENT);
    }
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) printProgress(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        if (Printer::isPrinting()) {
#if SDSUPPORT
            if (sd.sdactive && sd.sdmode) { // print from sd card
                Printer::progress = (static_cast<float>(sd.sdpos) * 100.0) / static_cast<float>(sd.filesize);
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
            lcd.drawUTF8(0, y0 + 1, GUI::buf);
            lcd.drawFrame(0, y0 + 5, 62, 9);
            lcd.drawBox(0, y0 + 6, static_cast<u8g2_uint_t>(62.0 * Printer::progress * 0.01), 7);

            if (Printer::maxLayer > 0) {
                GUI::bufClear();
                GUI::bufAddStringP(PSTR("Layer:"));
                lcd.drawUTF8(0, 42, GUI::buf);
                GUI::bufClear();
                GUI::bufAddInt(Printer::currentLayer, 5);
                GUI::bufAddStringP(PSTR("/"));
                GUI::bufAddInt(Printer::maxLayer, 5);
                lcd.drawUTF8(0, 50, GUI::buf);
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
            lcd.drawUTF8(0, 61, GUI::status);
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
        int len = strlen(static_cast<char*>(data));
        if (len <= 10) {
            lcd.setFont(u8g2_font_10x20_mf);
            lcd.drawUTF8(73 - len * 5, SPIN_CENTER_Y + 7, static_cast<char*>(data));
        } else {
            lcd.setFont(u8g2_font_6x10_mf);
            lcd.drawUTF8(73 - len * 3, SPIN_CENTER_Y + 3, static_cast<char*>(data));
        }
    }
}

#endif
