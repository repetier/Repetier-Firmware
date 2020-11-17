#ifndef _MENU_H
#define _MENU_H

enum class GUIAction;
extern const char* const axisNames[] PROGMEM;
extern const int32_t baudrates[] PROGMEM;

extern void menuMove(GUIAction action, void* data);
extern void menuHome(GUIAction action, void* data);
extern void menuConfig(GUIAction action, void* data);

extern void menuMoveAxis(GUIAction action, void* data);
extern void menuTune(GUIAction action, void* data);
extern void menuConfig(GUIAction action, void* data);
extern void menuMoveE(GUIAction action, void* data);
extern void menuFans(GUIAction action, void* data);
extern void menuConfigAxis(GUIAction action, void* data);
#endif
