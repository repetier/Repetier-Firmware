#ifndef _LEVELING_METHOD_H
#define _LEVELING_METHOD_H

#ifndef LEVELING_METHOD
#define LEVELING_METHOD 0
#endif

#ifndef LEVELING_CORRECTOR
#define LEVELING_CORRECTOR 0
#endif

class Plane;

#if LEVELING_CORRECTOR == 0 // software correction

class LevelingCorrector {
public:
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static void correct(Plane* plane);
};

#elif LEVELING_CORRECTOR == 1 // Motorized correction

class LevelingCorrector {
    static float points[3][2];

public:
    static void init();
    static void handleEeprom();
    static void resetEeprom();
    static void correct(Plane* plane);
};

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
    inline static void execute_G32(GCode* com) {}
};

#elif LEVELING_METHOD == 1 // Grid leveling

#ifndef GRID_SIZE
#define GRID_SIZE 3
#endif

class Leveling {
    static float grid[GRID_SIZE][GRID_SIZE];
    static float xMin, xMax, yMin, yMax;
    static float dx, dy;
    static uint16_t eprStart;
    static uint8_t distortionEnabled;

public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void measure();
    inline static void init();
    inline static void handleEeprom();
    inline static void resetEeprom();
    inline static void execute_G32(GCode* com);
    inline static void updateDerived();
};

#elif LEVELING_METHOD == 2 // 4 point symmetric

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    static void measure();
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static void execute_G32(GCode* com);
};

#else
#error Unknown value for LEVELING_METHOD set!
#endif

#endif
