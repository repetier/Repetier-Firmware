extern void Felix100MS();
extern void Felix500MS();

#undef EVENT_TIMER_100MS
#undef EVENT_TIMER_500MS
#define EVENT_TIMER_100MS {Felix100MS();}
#define EVENT_TIMER_500MS {Felix500MS();}

