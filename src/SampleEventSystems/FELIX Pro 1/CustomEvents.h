extern void Felix100MS();
extern void Felix500MS();
extern void FelixContainCoordinates();

#undef EVENT_TIMER_100MS
#undef EVENT_TIMER_500MS
#undef EVENT_CONTRAIN_DESTINATION_COORDINATES

#define EVENT_TIMER_100MS {Felix100MS();}
#define EVENT_TIMER_500MS {Felix500MS();}
#define EVENT_CONTRAIN_DESTINATION_COORDINATES FelixContainCoordinates();
