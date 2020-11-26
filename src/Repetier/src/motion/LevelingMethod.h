#ifndef _LEVELING_METHOD_H
#define _LEVELING_METHOD_H

#ifndef LEVELING_METHOD
#define LEVELING_METHOD LEVELING_METHOD_NONE
#endif

#ifndef LEVELING_CORRECTOR
#define LEVELING_CORRECTOR 0
#endif

#if ENABLE_BUMP_CORRECTION && LEVELING_METHOD != LEVELING_METHOD_GRID
#undef ENABLE_BUMP_CORRECTION
#define ENABLE_BUMP_CORRECTION 0 // Disable if not supported
#endif

class Plane;

#if LEVELING_CORRECTOR == LEVELING_CORRECTOR_SOFTWARE // software correction

class LevelingCorrector {
public:
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static void correct(Plane* plane);
};

#elif LEVELING_CORRECTOR == LEVELING_CORRECTOR_MOTOR // Motorized correction

class LevelingCorrector {
public:
    static void init();
    static void handleEeprom();
    static void resetEeprom();
    static void correct(Plane* plane);
};

#endif

#if LEVELING_METHOD == LEVELING_METHOD_NONE // No leveling

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void setDistortionEnabled(bool newState) {}
    inline static bool isDistortionEnabled() { return false; }
    inline static float distortionAt(float xp, float yp) { return 0; }
    static void importBumpMatrix(char* filename) {}
    static void exportBumpMatrix(char* filename) {}
    inline static bool measure(GCode* com) { return true; }
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    inline static bool execute_G32(GCode* com) { return true; }
    inline static void execute_G33(GCode* com) {}
    inline static void execute_M323(GCode* com) {}
};

#elif LEVELING_METHOD == LEVELING_METHOD_GRID // Grid leveling

#ifndef BUMP_DEFAULT_AUTOIMPORT_DIR
#define BUMP_DEFAULT_AUTOIMPORT_DIR "matrixes/"
#endif

#ifndef MAX_GRID_SIZE
#ifdef GRID_SIZE // Old config backwards compatibility
#define MAX_GRID_SIZE GRID_SIZE
#else
#define MAX_GRID_SIZE 3
#endif
#endif

class Leveling {
    static float grid[MAX_GRID_SIZE][MAX_GRID_SIZE]; // Bumps up have negative values!
    static float gridTemp;
    static float xMin, xMax, yMin, yMax;
    static float dx, dy, invDx, invDy;
    static float startDegrade, endDegrade, diffDegrade;
    static char autoImportDir[LONG_FILENAME_LENGTH + 1];
    static uint16_t eprStart;
    static uint8_t distortionEnabled;
    static uint8_t curGridSize;
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
        return x >= 0 && y >= 0 && x < curGridSize && y < curGridSize;
    }
    static bool gridIndexForDir(int dir, int dist, int& x, int& y);
#if ENABLE_BUMP_CORRECTION
    static void showMatrix();
    static void set(float x, float y, float z);
#endif
public:
    static void setDistortionEnabled(bool newState);
#if ENABLE_BUMP_CORRECTION
    static void addDistortion(float* pos); // ads bumps so you get required z position, printer coordinates
    static void subDistortion(float* pos); // printer coordinates
    inline static bool isDistortionEnabled() { return distortionEnabled; }
    static float distortionAt(float xp, float yp); // printer coordinates
    static void importBumpMatrix(char* filename);
    static void exportBumpMatrix(char* filename);
    static void execute_M323(GCode* com);
#else
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static bool isDistortionEnabled() { return false; }
    inline static void execute_M323(GCode* com) {}
    inline static float distortionAt(float xp, float yp) { return 0.0f; }
    static void importBumpMatrix(char* filename) {}
    static void exportBumpMatrix(char* filename) {}
#endif
    static void reportDistortionStatus();
    static bool measure(GCode* com);
    static void init();
    static void handleEeprom();
    static void resetEeprom();
    static void updateDerived();
    static bool execute_G32(GCode* com);
    static void execute_G33(GCode* com);
};

#elif LEVELING_METHOD == LEVELING_METHOD_4_POINT_SYMMETRIC // 4 point symmetric

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void setDistortionEnabled(bool newState) {}
    inline static bool isDistortionEnabled() { return false; }
    inline static float distortionAt(float xp, float yp) { return 0; }
    static bool measure(GCode* com);
    static void importBumpMatrix(char* filename) {}
    static void exportBumpMatrix(char* filename) {}
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static bool execute_G32(GCode* com);
    inline static void execute_G33(GCode* com) {}
    inline static void execute_M323(GCode* com) {}
};

#elif LEVELING_METHOD == LEVELING_METHOD_3_POINTS // 3 points

class Leveling {
public:
    inline static void addDistortion(float* pos) {}
    inline static void subDistortion(float* pos) {}
    inline static void setDistortionEnabled(bool newState) {}
    inline static bool isDistortionEnabled() { return false; }
    inline static float distortionAt(float xp, float yp) { return 0; }
    static bool measure(GCode* com);
    static void importBumpMatrix(char* filename) {}
    static void exportBumpMatrix(char* filename) {}
    inline static void init() {}
    inline static void handleEeprom() {}
    inline static void resetEeprom() {}
    static bool execute_G32(GCode* com);
    inline static void execute_G33(GCode* com) {}
    inline static void execute_M323(GCode* com) {}
};

#else
#error Unknown value for LEVELING_METHOD set!
#endif

#endif
