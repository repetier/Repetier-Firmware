#undef CONFIG_EXTERN
#undef CONFIG_VARIABLE
#undef CONFIG_VARIABLE_EQ
#define CONFIG_EXTERN
#define CONFIG_VARIABLE(tp, name, values) tp name values;
#define CONFIG_VARIABLE_EQ(tp, name, values) tp name = values;

#include "Repetier.h"

// Create class instances form Configuration_io.h

#undef IO_TARGET
#define IO_TARGET IO_TARGET_DEFINE_VARIABLES
#include "io/redefine.h"

void updateEndstops() {
#undef IO_TARGET
#define IO_TARGET IO_TARGET_ENDSTOP_UPDATE
#include "io/redefine.h"
}
