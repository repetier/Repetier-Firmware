#undef CONFIG_EXTERN
#undef CONFIG_VARIABLE
#undef CONFIG_VARIABLE_EQ
#define CONFIG_EXTERN
#define CONFIG_VARIABLE(tp, name, values) tp name values;
#define CONFIG_VARIABLE_EQ(tp, name, values) tp name = values;

#include "Repetier.h"

// Create class instances form Configuration_io.h

#undef IO_TARGET
#define IO_TARGET 6
#include "src/io/redefine.h"

void updateEndstops() {
#undef IO_TARGET
#define IO_TARGET 5
#include "src/io/redefine.h"
}
