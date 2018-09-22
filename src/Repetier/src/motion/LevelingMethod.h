#ifndef _LEVELING_METHOD_H
#define _LEVELING_METHOD_H

#ifndef LEVELING_METHOD
#define LEVELING_METHOD 0
#endif

#if LEVELING_METHOD == 0 // No leveling

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void measure() {}
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
};

#elif LEVELING_METHOD == 1 // Grid leveling

#ifndef GRID_SIZE
#define GRID_SIZE 3
#endif

class Leveling {
    float grid[GRID_SIZE][GRID_SIZE];

public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void measure() {}
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
};

#else
#error Unknown value for LEVELING_METHOD set!
#endif

#endif
