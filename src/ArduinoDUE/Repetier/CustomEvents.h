
extern bool EventUnhandledGCode(GCode *com);

#undef EVENT_UNHANDLED_M_CODE

#define EVENT_UNHANDLED_M_CODE(c) EventUnhandledGCode(c)
