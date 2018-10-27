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
    inline static void execute_G33(GCode* com) {}
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
    inline static float xPosFor(fast8_t index) {
        return xMin + dx * index;
    }
    inline static float yPosFor(fast8_t index) {
        return yMin + dy * index;
    }
    static void extrapolateGrid();
    static bool extrapolateableNeighbours(int x, int y);
    static float extrapolateNeighbours(int x, int y);
    inline static bool validGridIndex(int x, int y) {
        return x >= 0 && y >= 0 && x < GRID_SIZE && y < GRID_SIZE;
    }
    static bool gridIndexForDir(int dir, int dist, int& x, int& y);

public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    static void measure();
    static void init();
    static void handleEeprom();
    static void resetEeprom();
    static void updateDerived();
    static void execute_G32(GCode* com);
    static void execute_G33(GCode* com);
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
    inline static void execute_G33(GCode* com) {}
};

#elif LEVELING_METHOD == 3 // 3 points

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    static void measure();
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static void execute_G32(GCode* com);
    inline static void execute_G33(GCode* com) {}
};

#else
#error Unknown value for LEVELING_METHOD set!
#endif

#endif
