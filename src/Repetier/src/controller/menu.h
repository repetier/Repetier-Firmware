#ifndef _MENU_H
#define _MENU_H

enum class GUIAction;

extern void menuMove(GUIAction action, void* data);
extern void menuHome(GUIAction action, void* data);
extern void menuConfig(GUIAction action, void* data);

extern void menuMoveX(GUIAction action, void* data);
extern void menuMoveY(GUIAction action, void* data);
extern void menuMoveZ(GUIAction action, void* data);
extern void menuMoveE(GUIAction action, void* data);

#endif
