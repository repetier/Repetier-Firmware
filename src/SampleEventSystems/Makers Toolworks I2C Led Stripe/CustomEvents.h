#ifndef CUSTOM_EVENTS_H_INCLUDED
#define CUSTOM_EVENTS_H_INCLUDED

#define EVENT_WAITING_HEATER(id) {MTWLED_WaitingHeater(id);}
#define EVENT_HEATING_FINISHED(id) {MTWLED_HeatingFinished(id);}
#define EVENT_TIMER_100MS {MTWLED_Update();}

void MTWLED_Update();
void MTWLED_WaitingHeater(int8_t id);
void MTWLED_HeatingFinished(int8_t id);

#endif //CUSTOM_EVENTS_H_INCLUDED
