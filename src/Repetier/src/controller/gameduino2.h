/*

Implementation of the gameduino2 parts. Since they require a complete new
handling of everything gui related, we moved it into a separate file.

*/

#include "gd2_ext.h"

// Initalize display
void initializeLCD()
{
    HAL::delayMilliseconds(100);
    HAL::spiBegin();
    HAL::delayMilliseconds(100);
    GD.begin();
    HAL::delayMilliseconds(100);
}

#define CELL_32_SPEED 0
#define CELL_32_FLOW 1
#define CELL_32_FAN 2
#define CELL_32_EXTRUDER 11
#define CELL_32_BED 14
#define CELL_32_SDCARD 15
#define CELL_32_MENU 16
#define CELL_32_EMERGENCY 17
#define CELL_32_POWER_ON 18
#define CELL_32_POWER_OFF 19
#define CELL_32_LIGHT_ON 20
#define CELL_32_LIGHT_OFF 21
#define CELL_32_PAUSE 22
#define CELL_32_PRINT 23
#define CELL_32_HOME 24
#define CELL_32_MOVE 25
#define CELL_32_CHECK_OFF 26
#define CELL_32_CHECK_ON 27
#define CELL_32_TARGET_POS 28
#define CELL_32_CURRENT_POS 29
#define CELL_32_CLOSE 30
#define CELL_32_BIGBTN_SEL_A 31
#define CELL_32_BIGBTN_SEL_B 32
#define CELL_32_BIGBTN_A 33
#define CELL_32_BIGBTN_B 34

#define TAG_NONE 0
#define TAG_EMERGENCY 1
#define TAG_POWER 2
#define TAG_LIGHT 3
#define TAG_FAN 4
#define TAG_FAN_SLIDER 5
#define TAG_BED 6
#define TAG_BED_SLIDER 7
#define TAG_EXT0 8
#define TAG_EXT0_SLIDER 9
#define TAG_EXT1 10
#define TAG_EXT1_SLIDER 11
#define TAG_SPEED_SLIDER 12
#define TAG_FLOW_SLIDER 13
#define TAG_MENU 14

class GD2
{
public:
    static uint8_t screens[4];
    static uint8_t screenPos;
    static void startScreen()
    {
        HAL::delayMilliseconds(100);
        GD.ClearColorRGB(0x103000);
        GD.Clear();
        HAL::delayMilliseconds(100);
        GD.cmd_textP(240, 30, 31, OPT_CENTER, versionString);
        GD.cmd_textP(240, 130, 31, OPT_CENTER, PSTR(UI_PRINTER_NAME));
        GD.swap();
        HAL::delayMilliseconds(100);
        LOAD_ASSETS();
        HAL::delayMilliseconds(100);
    }
    static void refresh()
    {
        switch(screens[screenPos]) {
        case 0:
        default:
            renderMainScreen();
            break;
        }
        GD.swap();
    }
    static void parse(FSTRINGPARAM(text)) {
        uid.col = 0;
        uid.parse(text,false);
    }
    static void renderMainScreen() {
//        GD.ClearColorRGB(0xf0f0f0L);
        GD.ClearColorRGB(0xffffffL);
        GD.Clear();
        // Grid
        GD.ColorRGB(0xabadb3L);
        GD.Begin(RECTS);
        GD.Vertex2ii(0,251);GD.Vertex2ii(480,272);
        GD.Begin(LINES);
        GD.Vertex2ii(0,58);GD.Vertex2ii(419, 58);
        GD.Vertex2ii(419,0);GD.Vertex2ii(419, 250);
        GD.Vertex2ii(0,250);GD.Vertex2ii(479, 250);
        GD.Vertex2ii(444,250);GD.Vertex2ii(444, 272);
        GD.ColorRGB(0x3b5e67);
        GD.Begin(BITMAPS);
        GD.Tag(TAG_MENU);
        GD.Vertex2ii( 451, 253, ICONS32_HANDLE, CELL_32_MENU);
        GD.Tag(TAG_EMERGENCY);
        GD.Vertex2ii( 432, 11, ICONS32_HANDLE, CELL_32_EMERGENCY);
        int iy = 51;
        GD.Tag(TAG_POWER);
        GD.Vertex2ii( 432, iy, ICONS32_HANDLE, CELL_32_POWER_OFF);
        iy += 40;
        GD.Tag(TAG_LIGHT);
        GD.Vertex2ii( 432, iy, ICONS32_HANDLE, CELL_32_LIGHT_OFF);
        iy += 40;

        GD.Vertex2ii( 5, 74, ICONS32_HANDLE, CELL_32_SPEED);
        GD.Vertex2ii( 5, 104, ICONS32_HANDLE, CELL_32_FLOW);
        GD.Vertex2ii( 5, 134, ICONS32_HANDLE, CELL_32_FAN);
        GD.Vertex2ii( 5, 164, ICONS32_HANDLE, CELL_32_EXTRUDER);
        GD.Vertex2ii( 5, 194, ICONS32_HANDLE, CELL_32_EXTRUDER);
        GD.Vertex2ii( 5, 224, ICONS32_HANDLE, CELL_32_EXTRUDER);

        GD.Vertex2ii( 46, 74, SLIDER_HANDLE, 0);
        GD.Vertex2ii( 46, 104, SLIDER_HANDLE, 0);
        GD.Vertex2ii( 46, 134, SLIDER_HANDLE, 0);
        GD.Vertex2ii( 46, 164, SLIDER_HANDLE, 1);
        GD.Vertex2ii( 46, 194, SLIDER_HANDLE, 1);
        GD.Vertex2ii( 46, 224, SLIDER_HANDLE, 1);
        // Text
        GD.ColorRGB(0);
        parse(PSTR(UI_TEXT_PAGE_BUFFER));
        GD.cmd_text(9, 8, 26, 0, uid.printCols);
        parse(PSTR(UI_TEXT_PRINT_FILAMENT  ":%Uf m"));
        GD.cmd_text(9, 24, 26, 0, uid.printCols);
        parse(PSTR(UI_TEXT_PRINT_TIME  ":%Ut"));
        GD.cmd_text(9, 40, 26, 0, uid.printCols);
        parse(PSTR("X:%x0 mm"));
        GD.cmd_text(330, 8, 26, 0, uid.printCols);
        parse(PSTR("Y:%x1 mm"));
        GD.cmd_text(330, 24, 26, 0, uid.printCols);
        parse(PSTR("Z:%x2 mm"));
        GD.cmd_text(330, 40, 26, 0, uid.printCols);
        GD.ColorRGB(0xffffffL);
        parse(PSTR("%os"));
        GD.cmd_text(9, 250, 27, 0, uid.printCols);
    }
    // Check toush screen for new actions
    static void checkTouch()
    {
        GD.get_inputs();
        switch(GD.inputs.tag) {
        case 1: // Emergency
            break;
        }
    }
};
uint8_t GD2screens[4] = {0,0,0,0};
uint8_t screenPos = 0;

void uiInitKeys() {}
void uiCheckKeys(int &action) {}
inline void uiCheckSlowEncoder() {}
void uiCheckSlowKeys(int &action)
{
    GD2::checkTouch();
}
