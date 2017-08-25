
extern bool EventUnhandledGCode(GCode *com);
extern void SelectExtruder500XL(uint8_t t);

#undef EVENT_UNHANDLED_G_CODE

#define EVENT_UNHANDLED_G_CODE(c) EventUnhandledGCode(c)
