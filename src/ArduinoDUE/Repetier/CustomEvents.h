
extern bool EventUnhandledGCode(GCode *com);

#undef EVENT_UNHANDLED_G_CODE
#undef EVENT_UNHANDLED_M_CODE

#define EVENT_UNHANDLED_G_CODE(c) EventUnhandledGCode(c)
#define EVENT_UNHANDLED_M_CODE(c) EventUnhandledGCode(c)
