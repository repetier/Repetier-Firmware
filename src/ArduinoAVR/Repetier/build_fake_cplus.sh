#!/bin/bash
# Create the files fake_new_delete.c and fake_new_delete.h
#
bfname="fake_new_delete"
echo '/* All credit for this goes to:
https://www.avrfreaks.net/forum/avr-c-micro-how
*/
#include <stdlib.h>
__extension__ typedef int __guard __attribute__((mode (__DI__)));
/**To quote Linus - You are Stupid and Ugly if you use C++ in embedded **/
extern "C" {
						
void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);}
void __cxa_guard_release (__guard *g) {*(char *)g = 1;}
void __cxa_guard_abort (__guard *) {}

void __cxa_pure_virtual(void) {}
						
}
' > $bfname.c
echo '#ifndef _FAKE_NEW_DELETE_H_
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
' > $bfname.h
