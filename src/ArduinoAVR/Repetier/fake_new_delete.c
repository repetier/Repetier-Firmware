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

