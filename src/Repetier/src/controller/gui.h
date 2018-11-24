#ifndef _GUI_H
#define _GUI_H

#define GUI_MAX_LEVEL 5
#define MAX_COLS 40

enum class GUIAction {
    NONE = 0,
    DRAW = 1,
    NEXT = 2,
    PREVIOUS = 3,
    CLICK = 4,
    ANALYSE = 5,
    BACK = 6,
    CLICK_PROCESSED = 7
};

enum class GUIPageType {
    TOPLEVEL = 1,      // Top menu
    MENU = 2,          // Scrollable menu
    FIXED_CONTENT = 3, // Fixed content height - gets next/previous events
    SD_CARD = 4,       // Select file for printing
    WIZARD_FIXED = 5,  // like FIXED_CONTENT without timeout,
    WIZARD_MENU = 6,   // like MENU without timeout
    ACK_MESSAGE = 7,   // Sticky message until ok is clicked
    POP = 8,           // Pop if clicked
    STATUS = 9,        // Info, Warning or Error page
    BUSY = 10          // Busy status
};

enum class GUIStatusLevel {
    REGULAR = 0, // Normal info in status bar
    BUSY = 1,    // Waiting view
    INFO = 2,    // Popup info
    WARNING = 3, // Popup warning
    ERROR = 4    // Popup error
};

typedef void (*GuiCallback)(GUIAction action, void* data);
extern void startScreen(GUIAction action, void* data);
extern void warningScreen(GUIAction action, void* data);
extern void errorScreen(GUIAction action, void* data);
extern void infoScreen(GUIAction action, void* data);
extern void waitScreen(GUIAction action, void* data);

#include "drivers/DisplayBase.h"

#define UI_STATUS(status) \
    GUI::setStatusP(PSTR(status), GUIStatusLevel::REGULAR);
#define UI_STATUS_F(status) \
    GUI::setStatusP(status, GUIStatusLevel::REGULAR);
#define UI_STATUS_UPD(status) \
    { \
        GUI::setStatusP(PSTR(status), GUIStatusLevel::REGULAR); \
        GUI::contentChanged = true; \
    }
#define UI_STATUS_UPD_F(status) \
    { \
        GUI::setStatusP(PSTR(status), GUIStatusLevel::REGULAR); \
        GUI::contentChanged = true; \
    }
#define UI_STATUS_RAM(status) \
    { \
        GUI::setStatus(status, GUIStatusLevel::REGULAR); \
    }
#define UI_STATUS_UPD_RAM(status) \
    { \
        GUI::setStatus(status, GUIStatusLevel::REGULAR); \
        GUI::contentChanged = true; \
    }
#define UI_ERROR(status) \
    {}
#define UI_ERROR_P(status) \
    {}
#define UI_ERROR_UPD(status) \
    {}
#define UI_ERROR_RAM(status) \
    {}
#define UI_ERROR_UPD_RAM(status) \
    {}

#define UI_CLEAR_STATUS \
    { \
        GUI::clearStatus(); \
    }

class GUI {
public:
    static int level;                            // Menu level for back handling
    static int topRow[GUI_MAX_LEVEL];            ///< First visible row
    static int cursorRow[GUI_MAX_LEVEL];         ///< Selected row
    static int maxCursorRow[GUI_MAX_LEVEL];      ///< Maximum selectable row
    static int length[GUI_MAX_LEVEL];            ///< Rows for complete content
    static GuiCallback callbacks[GUI_MAX_LEVEL]; ///< Content handler
    static void* data[GUI_MAX_LEVEL];            ///< menu pointer to custom data
    static GUIPageType pageType[GUI_MAX_LEVEL];  ///< page type
    static millis_t lastRefresh;                 ///< Last refresh time
    static millis_t lastAction;                  ///< Last action time for autoreturn to display
    static bool contentChanged;                  ///< set to true if forced refresh is wanted
    static char status[MAX_COLS + 1];            ///< Status Line
    static char buf[MAX_COLS + 1];               ///< Buffer to build strings
    static fast8_t bufPos;                       ///< Pos for appending data
    static GUIAction nextAction;                 ///< Next action to execute on opdate
    static int nextActionRepeat;                 ///< Increment for next/previous
    static GUIStatusLevel statusLevel;
    static void bufClear();
    static void bufAddInt(int value, uint8_t digits, char fillChar = ' ');
    static void bufAddLong(long value, int8_t digits);
    static void bufAddFloat(float value, int8_t fixdigits, int8_t digits);
    static void bufAddString(char* value);
    static void bufAddStringP(FSTRINGPARAM(text));
    static void bufAddChar(char value);
    static void bufAddHeaterTemp(HeatManager* hm, bool target);

    static void bufToStatus(GUIStatusLevel lvl);
    static void clearStatus();
    static void setStatusP(FSTRINGPARAM(text), GUIStatusLevel lvl);
    static void setStatus(char* text, GUIStatusLevel lvl);

    static void resetMenu(); ///< Go to start page
    static void init();      ///< Initialize display
    static void refresh();   ///< Refresh display
    static void update();    ///< Calls refresh, checks buttons
    static void pop();       ///< Go 1 level higher if possible
    static void popBusy();   ///< Pop if waiting is on top
    static void push(GuiCallback cb, void* cData, GUIPageType tp);
    static void replace(GuiCallback cb, void* cData, GUIPageType tp);
    static bool isStickyPageType(GUIPageType t);

    // Run action for key press
    static void backKey();
    static void nextKey();
    static void previousKey();
    static void okKey();
    static void setEncoderA(fast8_t state);
    static void setEncoderB(fast8_t state);
    static void handleKeypress();
    static void replaceOn(GUIAction a, GuiCallback cb, void* cData, GUIPageType tp);
    static void pushOn(GUIAction a, GuiCallback cb, void* cData, GUIPageType tp);

    // Draw menu functions - driver specific

    static void menuStart(GUIAction action);
    static void menuEnd(GUIAction action);
    static void menuText(GUIAction action, PGM_P text, bool highlight);
    static void menuSelectable(GUIAction action, PGM_P text, GuiCallback cb, void* cData, GUIPageType tp);

    // Value modifyer display

    static void modInt(PGM_P text, int32_t& value, int32_t min, int32_t max, int32_t step);
};

#endif
