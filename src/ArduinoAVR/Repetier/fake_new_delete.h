#ifndef _FAKE_NEW_DELETE_H_
#define _FAKE_NEW_DELETE_H_ 1
/*Credit for this goes to:
https://www.avrfreaks.net/forum/avr-c-micro-how
*/
void * operator new(size_t size);
void operator delete(void * ptr);
__extension__ typedef int __guard __attribute__((mode (__DI__)));

extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *);
extern "C" void __cxa_pure_virtual(void);

#endif
