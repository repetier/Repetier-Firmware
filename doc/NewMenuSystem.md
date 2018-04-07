# New Menu System

## Introduction

While the system in V1 was compact, it proved to be very hard to maintain and it
showed also some problems if it comes to flexible menus. With the new system the 
maals are:
- Keep the structure easy and understandable.
- Keep it translatable.
- Make it more aware of graphic and character devices.
- More flexibility in what we see.
- Use u8g2 library for graphic displays.
- Remove 2 line display support.

## Basic structure

There will be a more distinct difference between the different display types.
For basic tasks we assume we have 4 display lines of 20 characters at least.
The graphic display will show a status bar at the top to use the extra place.
That way we homogenise the most functions making them easier to implement.
In addition w ealso provide per driver special notification functions, so we
can make use of an icon plus 2 Lines of notification.

Other content type is a scrollable list and a changeable int/float.

To glue this all together, we use functions as callbacks and a stack of views.
New content can be pushed to the stack and popping from it will show the
previous content. For functions like wizards there is also the possibility of
a replace.

Main entry is the variable ui of class UI.

Each display function is of the form:

enum DisplayAction {
  SHOW = 0,
  SELECT = 1,
  NEXT = 2,
  PREVIOUS = 3
};
void pageFunction(DisplayAction action);

You see the operations are now not spread across the sources but centralized in
one function. 

## UI

### Initialize

ui.init(DisplayDriver, &topMenu)

Sets the driver to be used and defines the first menu entry.

### Stack function

ui.push(&function, bool sticky)

Push a function on stack. If it is not sticky, it will return to main level after a
timeout.

## DisplayDriver

The driver is your helper to quickly create your menus.

### show4

show4(char *line1,char *line2,char *line3,char *line4)

Parses the 4 lines and shows them below each other. 

### listStart

listStart()

Starts displaying a list.

### listAdd 

listAdd(pageFunction *function, char *line)

Adds a new line to display. If it is in the visible area it will be shown.
If it matches the current row it will also be marked active and also be stored
in ui.hoverPage. Lines with nullptr can not be marked and will be skipped on next/previous
moves.

### listEnd

listEnd()

Marks that the list is finished. Will remove display parts from previous display if
there are not all lines used.

### error

error(char *line1, char* line2)

Shows an error message.

### info

info(char *line1, char *line2)

Shows an info message

