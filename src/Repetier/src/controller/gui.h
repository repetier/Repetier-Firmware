#ifndef _GUI_H
#define _GUI_H

#define GUI_MAX_LEVEL 5
#define MAX_COLS 40

#define GUI_DIRECT_ACTION_HOME_ALL 1
#define GUI_DIRECT_ACTION_HOME_X 2
#define GUI_DIRECT_ACTION_HOME_Y 3
#define GUI_DIRECT_ACTION_HOME_Z 4
#define GUI_DIRECT_ACTION_HOME_E 5
#define GUI_DIRECT_ACTION_HOME_A 6
#define GUI_DIRECT_ACTION_HOME_B 7
#define GUI_DIRECT_ACTION_HOME_C 8
#define GUI_DIRECT_ACTION_FACTORY_RESET 9
#define GUI_DIRECT_ACTION_STORE_EEPROM 10
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_ECHO 11
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_INFO 12
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_ERRORS 13
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_DRYRUN 14
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_NO_MOVES 15
#define GUI_DIRECT_ACTION_TOGGLE_DEBUG_COMMUNICATION 16
#define GUI_DIRECT_ACTION_TOGGLE_LIGHT 17
#define GUI_DIRECT_ACTION_DISABLE_MOTORS 18
#define GUI_DIRECT_ACTION_MOUNT_SD_CARD 19
#define GUI_DIRECT_ACTION_STOP_PRINT 20
#define GUI_DIRECT_ACTION_PAUSE_PRINT 21
#define GUI_DIRECT_ACTION_CONTINUE_PRINT 22
#define GUI_DIRECT_ACTION_POWERLOSS 23
#define GUI_DIRECT_ACTION_DITTO_OFF 24
#define GUI_DIRECT_ACTION_DITTO_MIRROR 25
#define GUI_DIRECT_ACTION_DITTO_2 26
#define GUI_DIRECT_ACTION_DITTO_3 27
#define GUI_DIRECT_ACTION_DITTO_4 28
#define GUI_DIRECT_ACTION_DITTO_5 29
#define GUI_DIRECT_ACTION_DITTO_6 30
#define GUI_DIRECT_ACTION_DITTO_7 31
#define GUI_DIRECT_ACTION_DITTO_8 32
#define GUI_DIRECT_ACTION_TOGGLE_PROBE_PAUSE 33
#define GUI_DIRECT_ACTION_TOGGLE_AUTORETRACTIONS 34

enum class GUIBootState {
    DISPLAY_INIT = 0,
    IN_INTRO = 1,
    READY = 2
};

enum class GUIAction {
    NONE = 0,
    DRAW = 1,
    NEXT = 2,
    PREVIOUS = 3,
    CLICK = 4,
    ANALYSE = 5,
    BACK = 6,
    CLICK_PROCESSED = 7,
    BACK_PROCESSED = 8
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
    BUSY = 10,         // Busy status
    ACTION = 11        // Action command that does not need a new entry
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
extern void printProgress(GUIAction action, void* data);
extern void probeProgress(GUIAction action, void* data);
extern void mainMenu(GUIAction action, void* data);
extern void startScreen(GUIAction action, void* data);
extern void warningScreen(GUIAction action, void* data);
extern void warningScreenP(GUIAction action, void* data);
extern void errorScreen(GUIAction action, void* data);
extern void errorScreenP(GUIAction action, void* data);
extern void infoScreen(GUIAction action, void* data);
extern void infoScreenP(GUIAction action, void* data);
extern void waitScreen(GUIAction action, void* data);
extern void waitScreenP(GUIAction action, void* data);
extern void directAction(GUIAction action, void* data);
extern void selectToolAction(GUIAction action, void* data);

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
    { }
#define UI_ERROR_P(status) \
    { }
#define UI_ERROR_UPD(status) \
    { }
#define UI_ERROR_RAM(status) \
    { }
#define UI_ERROR_UPD_RAM(status) \
    { }

#define UI_CLEAR_STATUS \
    { \
        GUI::clearStatus(); \
    }

struct probeProgInfo;
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
    static GUIBootState curBootState;            ///< GUI boot sequence state
    static bool contentChanged;                  ///< set to true if forced refresh is wanted
    static char status[MAX_COLS + 1];            ///< Status Line
    static char buf[MAX_COLS + 1];               ///< Buffer to build strings
    static char tmpString[MAX_COLS + 1];         ///< Buffer to build strings
    static fast8_t bufPos;                       ///< Pos for appending data
    static GUIAction nextAction;                 ///< Next action to execute on opdate
    static int nextActionRepeat;                 ///< Increment for next/previous
    static GUIStatusLevel statusLevel;
    static bool textIsScrolling;
    static probeProgInfo* curProbingProgress;    ///< Pointer to a valid current probing datastruct
#if SDSUPPORT
    static char cwd[SD_MAX_FOLDER_DEPTH * LONG_FILENAME_LENGTH + 2];
    static uint8_t folderLevel;
#endif

    static void bufClear();
    static void bufAddInt(int value, int8_t digits, char fillChar = ' ');
    static void bufAddLong(long value, int8_t digits);
    static void bufAddFloat(float value, int8_t fixdigits, int8_t digits);
    static void bufAddString(char* value);
    static void bufAddStringP(FSTRINGPARAM(text));
    static void bufAddChar(char value);
    static void bufAddHeaterTemp(HeatManager* hm, bool target);

    static void bufToStatus(GUIStatusLevel lvl);

    static void flashToString(char* dest, FSTRINGPARAM(text));
    static void flashToStringLong(char* dest, FSTRINGPARAM(text), int32_t val);
    static void flashToStringFloat(char* dest, FSTRINGPARAM(text), float val, int prec = 0);
    static void flashToStringFlash(char* dest, FSTRINGPARAM(text), FSTRINGPARAM(val));
    static void flashToStringString(char* dest, FSTRINGPARAM(text), char* val);

    static void clearStatus();
    static void setStatusP(FSTRINGPARAM(text), GUIStatusLevel lvl);
    static void setStatus(char* text, GUIStatusLevel lvl);

    static void resetMenu();        ///< Go to start page
    static void init();             ///< Initialize display
    static void processInit();      ///< Continue initializing display if not ready
    static void refresh();          ///< Refresh display
    static void update();           ///< Calls refresh, checks buttons
    static void pop();              ///< Go 1 level higher if possible
    static void pop(int selection); // For prompt callbacks!
    static void popBusy();          ///< Pop if waiting is on top
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
    static void menuTextP(GUIAction& action, PGM_P text, bool highlight = false);
    static void menuFloatP(GUIAction& action, PGM_P text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuLongP(GUIAction& action, PGM_P text, long val, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuOnOffP(GUIAction& action, PGM_P text, bool val, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuSelectableP(GUIAction& action, PGM_P text, GuiCallback cb, void* cData, GUIPageType tp);

    static void menuText(GUIAction& action, char* text, bool highlight = false);
    static void menuFloat(GUIAction& action, char* text, float val, int precision, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuLong(GUIAction& action, char* text, long val, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuOnOff(GUIAction& action, char* text, bool val, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuSelectable(GUIAction& action, char* text, GuiCallback cb, void* cData, GUIPageType tp);
    static void menuBack(GUIAction& action);

    static void showScrollbar(GUIAction& action);
    static void showScrollbar(GUIAction& action, float percent, uint16_t min, uint16_t max);

    // Value modifyer display

    // Draw display with content for a value given as string
    static void showValueP(PGM_P text, PGM_P unit, char* value);
    static void showValue(char* text, PGM_P unit, char* value);
    static bool handleFloatValueAction(GUIAction& action, float& value, float min, float max, float increment);
    static bool handleFloatValueAction(GUIAction& action, float& value, float increment);
    static bool handleLongValueAction(GUIAction& action, int32_t& value, int32_t min, int32_t max, int32_t increment);
};

struct probeProgInfo {
    explicit probeProgInfo(const float& _x, const float& _y, const float& _z, const uint16_t& _num, const uint16_t _maxNum)
        : x(_x)
        , y(_y)
        , z(_z)
        , num(_num)
        , maxNum(_maxNum) {
        GUI::curProbingProgress = this;
    }
    const float &x, &y, &z;
    const uint16_t &num, maxNum;
    ~probeProgInfo() {
        GUI::curProbingProgress = nullptr;
    }
};

#define DRAW_FLOAT_P(text, unit, val, prec) \
    float v = val; \
    if (action == GUIAction::DRAW) { \
        GUI::bufClear(); \
        GUI::bufAddFloat(v, 0, prec); \
        GUI::showValueP(text, unit, GUI::buf); \
    }

#define DRAW_FLOAT(text, unit, val, prec) \
    float v = val; \
    if (action == GUIAction::DRAW) { \
        GUI::bufClear(); \
        GUI::bufAddFloat(v, 0, prec); \
        GUI::showValue(text, unit, GUI::buf); \
    }

#define DRAW_LONG_P(text, unit, val) \
    int32_t v = val; \
    if (action == GUIAction::DRAW) { \
        GUI::bufClear(); \
        GUI::bufAddLong(v, 0); \
        GUI::showValueP(text, unit, GUI::buf); \
    }

#define DRAW_LONG(text, unit, val) \
    int32_t v = val; \
    if (action == GUIAction::DRAW) { \
        GUI::bufClear(); \
        GUI::bufAddLong(v, 0); \
        GUI::showValue(text, unit, GUI::buf); \
    }

#include "menu.h"

#endif
