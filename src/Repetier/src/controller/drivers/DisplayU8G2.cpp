#include "Repetier.h"

#if DISPLAY_DRIVER == DRIVER_U8G2
#include <U8g2lib.h>
#define UI_SPI_SCK UI_DISPLAY_D4_PIN
#define UI_SPI_MOSI UI_DISPLAY_ENABLE_PIN
#define UI_SPI_CS UI_DISPLAY_RS_PIN
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
        lcd.drawStr(20, 55, buf);
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

void GUI::menuText(GUIAction action, PGM_P text, bool highlight) {
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
                lcd.drawStr(0, guiY - 1, GUI::buf);
                lcd.setDrawColor(1);
            } else {
                guiY += 10;
                lcd.drawStr(0, guiY, GUI::buf);
            }
        }
    }
    guiLine++;
}

void GUI::menuSelectable(GUIAction action, PGM_P text, GuiCallback cb, void* cData, GUIPageType tp) {
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
                lcd.drawStr(10, guiY, GUI::buf);
                lcd.drawGlyph(0, guiY, '>');
                lcd.setDrawColor(1);
            } else {
                lcd.drawStr(10, guiY, GUI::buf);
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
    } else if (guiLine == cursorRow[level]) {
        if (action == GUIAction::CLICK) {
            if (tp == GUIPageType::POP) {
                pop();
            } else {
                push(cb, cData, tp);
            }
        }
    }
    guiLine++;
}

// Value modifyer display

void GUI::modInt(PGM_P text, int32_t& value, int32_t min, int32_t max, int32_t step) {
}

extern void __attribute__((weak)) startScreen(GUIAction action, void* data);
extern void __attribute__((weak)) printProgress(GUIAction action, void* data);
extern void __attribute__((weak)) mainMenu(GUIAction action, void* data);
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
#if PRINTER_TYPE == 3
        // GUI::bufAddFloat(Motion1::currentPosition[PrinterType::getActiveAxis() ? A_AXIS : X_AXIS], 4, 2);
        GUI::bufAddFloat(Motion1::currentPosition[X_AXIS], 4, 2);
#else
        GUI::bufAddFloat(Motion1::currentPosition[X_AXIS], 4, 2);
#endif
        lcd.drawStr(66, 16, GUI::buf);
        // Y position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Y:"));
        GUI::bufAddFloat(Motion1::currentPosition[Y_AXIS], 4, 2);
        lcd.drawStr(66, 23, GUI::buf);
        // Z position
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Z:"));
        GUI::bufAddFloat(Motion1::currentPosition[Z_AXIS], 4, 2);
        lcd.drawStr(66, 30, GUI::buf);
        // FeedRate multiplier
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("FR:"));
        GUI::bufAddInt(Printer::feedrateMultiply, 3);
        GUI::bufAddChar('%');
        lcd.drawStr(66, 37, GUI::buf);

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
        if (NUM_TOOLS > 1 && tool->getToolType() == ToolTypes::EXTRUDER && tool->hasSecondary() && n < 6) {
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
            lcd.drawStr(0, 16 + n * 7, GUI::buf);
            n++;
        }
        if (n < 6 && Printer::areAllSteppersDisabled()) {
            GUI::bufClear();
            GUI::bufAddStringP("Motors off");
            lcd.drawStr(0, 16 + n * 7, GUI::buf);
            n++;
        }
        lcd.setFont(u8g2_font_6x10_mf);
        lcd.drawStr(0, 61, GUI::status);
    }
    GUI::replaceOn(GUIAction::NEXT, printProgress, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, printProgress, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) printProgress(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        lcd.setFont(u8g2_font_6x10_mf);
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Progress:"));
        GUI::bufAddFloat(Motion1::currentPosition[Y_AXIS], 4, 2);
        lcd.drawStr(0, 10, GUI::buf);

        lcd.setFont(u8g2_font_6x10_mf);
        lcd.drawStr(0, 61, GUI::status);
    }
    GUI::replaceOn(GUIAction::NEXT, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::replaceOn(GUIAction::PREVIOUS, startScreen, nullptr, GUIPageType::FIXED_CONTENT);
    GUI::pushOn(GUIAction::CLICK, mainMenu, nullptr, GUIPageType::MENU);
}

void __attribute__((weak)) mainMenu(GUIAction action, void* data) {
    GUI::menuStart(action);
    GUI::menuText(action, PSTR("Main Menu"), true);
#if UI_HAS_BACK_KEY == 0
    GUI::menuSelectable(action, PSTR("Back"), nullptr, nullptr, GUIPageType::POP);
#endif
    GUI::menuSelectable(action, PSTR("Warning 1"), warningScreen, (void*)"Test Warning", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectable(action, PSTR("Info 1"), infoScreen, (void*)"Test Info", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectable(action, PSTR("Error 1"), errorScreen, (void*)"Test Error", GUIPageType::WIZARD_FIXED);
    GUI::menuSelectable(action, PSTR("Spin"), waitScreen, (void*)"Computing", GUIPageType::BUSY);
    GUI::menuSelectable(action, PSTR("Spin 2"), waitScreen, (void*)"Computing slowly", GUIPageType::BUSY);
    GUI::menuSelectable(action, PSTR("Item 2"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 3"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 4"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 5"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 6"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 7"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 8"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 9"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuSelectable(action, PSTR("Item 10"), nullptr, nullptr, GUIPageType::POP);
    GUI::menuEnd(action);
}

void __attribute__((weak)) warningScreen(GUIAction action, void* data) {
    if (action == GUIAction::DRAW) {
        drawStatusLine();
        lcd.setFont(u8g2_font_10x20_mf);
        lcd.drawXBM(2, 13, WARNING_width, WARNING_height, WARNING_bits); // 26x26 pixel
        GUI::bufClear();
        GUI::bufAddStringP(PSTR("Warning"));
        lcd.drawStr(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawStr(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(PSTR(" OK "));
        lcd.drawStr(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
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
        lcd.drawStr(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawStr(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(PSTR(" OK "));
        lcd.drawStr(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
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
        lcd.drawStr(79 - 5 * strlen(GUI::buf), 33, GUI::buf);
        lcd.setFont(u8g2_font_6x10_mf);
        int len = strlen(static_cast<char*>(data));
        lcd.drawStr(64 - 3 * len, 50, static_cast<char*>(data));
        lcd.setDrawColor(0);
        GUI::bufClear();
        GUI::bufAddStringP(PSTR(" OK "));
        lcd.drawStr(64 - 3 * strlen(GUI::buf), 62, GUI::buf);
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
