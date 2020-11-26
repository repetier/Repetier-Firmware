#if Z_PROBE_TYPE == Z_PROBE_TYPE_NONE

class ZProbeHandler {
public:
    static bool activate() { return true; }
    static void deactivate() { }
    static float runProbe(uint8_t repetitions = Z_PROBE_REPETITIONS, bool useMedian = Z_PROBE_USE_MEDIAN) { return 0; }
    static bool probingPossible() { return false; }
    static float xOffset() { return 0; }
    static float yOffset() { return 0; }
    static void init() { }
    static void eepromHandle() { }
    static void eepromReset() { }
    static float optimumProbingHeight() { return 0; }
    static bool isActive() { return false; }

    static void setXOffset(float val) { }
    static void setYOffset(float val) { }

    static float getZProbeHeight() { return 0; };
    static void setZProbeHeight(float height) {};

    static float getCoating() { return 0; }
    static void setCoating(float val) { }

    static float getBedDistance() { return 0; }
    static void setBedDistance(float val) { }

    static void showConfigMenu(GUIAction action) { }
    static bool hasConfigMenu() { return false; }

    static void showControlMenu(GUIAction action) { }
    static bool hasControlMenu() { return false; }

    static bool getHeaterPause() { return false; }
    static void setHeaterPause(bool set) { }

    static float getSpeed() { return 0; }
    static void setSpeed(float val) { }
};

#elif Z_PROBE_TYPE == Z_PROBE_TYPE_DEFAULT

// Default handler for switches, induction
class ZProbeHandler {
    static uint16_t eprStart;
    static float height;
    static float bedDistance;
    static float coating;
    static float offsetX;
    static float offsetY;
    static float speed;
    static bool activated;
    static uint16_t userPausedHeaters;
    static bool pauseHeaters;

public:
    static bool activate();
    static void deactivate();
    static float runProbe(uint8_t repetitions = Z_PROBE_REPETITIONS, bool useMedian = Z_PROBE_USE_MEDIAN);
    static bool probingPossible();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float optimumProbingHeight();
    static bool isActive() { return activated; }

    static float xOffset();
    static float yOffset();

    static void setXOffset(float val) { offsetX = val; }
    static void setYOffset(float val) { offsetY = val; }

    static float getZProbeHeight();
    static void setZProbeHeight(float height);

    static float getCoating() { return coating; }
    static void setCoating(float val) { coating = val; }

    static float getBedDistance() { return bedDistance; }
    static void setBedDistance(float val) { bedDistance = val; }

    static void showConfigMenu(GUIAction action);
    static bool hasConfigMenu() { return true; }

    static void showControlMenu(GUIAction action) {};
    static bool hasControlMenu() { return false; }

    static bool getHeaterPause() { return pauseHeaters; }
    static void setHeaterPause(bool set) { pauseHeaters = set; }

    static float getSpeed() { return speed; }
    static void setSpeed(float val) { speed = val; }
};

#elif Z_PROBE_TYPE == Z_PROBE_TYPE_NOZZLE

// Nozzle based limit switch. Ensures a minimum temperature. Offset is
// not required since nozzle is the probe we use nozzle offset.

class ZProbeHandler {
    static uint16_t eprStart;
    static float height;
    static float bedDistance;
    static float speed;
    static bool activated;
    static int16_t probeTemperature;
    static int16_t activateTemperature;

    static uint16_t userPausedHeaters;
    static bool pauseHeaters;

public:
    static bool activate();
    static void deactivate();
    static float runProbe(uint8_t repetitions = Z_PROBE_REPETITIONS, bool useMedian = Z_PROBE_USE_MEDIAN);
    static bool probingPossible();
    static float xOffset();
    static float yOffset();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float optimumProbingHeight();
    static bool isActive() { return activated; }

    static void setXOffset(float val) { }
    static void setYOffset(float val) { }

    static float getProbingTemp() { return probeTemperature; }
    static void setProbingTemp(float val) { probeTemperature = val; }

    static float getZProbeHeight();
    static void setZProbeHeight(float height);

    static float getCoating() { return 0; }
    static void setCoating(float val) { }

    static float getBedDistance() { return bedDistance; }
    static void setBedDistance(float val) { bedDistance = val; }

    static void showConfigMenu(GUIAction action);
    static bool hasConfigMenu() { return true; }

    static void showControlMenu(GUIAction action) {};
    static bool hasControlMenu() { return false; }

    static bool getHeaterPause() { return pauseHeaters; }
    static void setHeaterPause(bool set) { pauseHeaters = set; }

    static float getSpeed() { return speed; }
    static void setSpeed(float val) { speed = val; }
};

#elif Z_PROBE_TYPE == Z_PROBE_TYPE_BLTOUCH

// Default handler for switches, induction
class ZProbeHandler {
    static uint16_t eprStart;
    static float height;
    static float bedDistance;
    static float offsetX;
    static float offsetY;
    static float speed;
    static bool activated;
    static uint16_t userPausedHeaters;
    static int16_t deployDelay;
    static bool pauseHeaters;

    static bool isAlarmOn();
    static void disableAlarmIfOn();

public:
    static bool activate();
    static void deactivate();
    static float runProbe(uint8_t repetitions = Z_PROBE_REPETITIONS, bool useMedian = Z_PROBE_USE_MEDIAN);
    static bool probingPossible();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float optimumProbingHeight();
    static bool isActive() { return activated; }

    static void setDeployDelay(uint16_t in) { deployDelay = in; }
    static uint16_t getDeployDelay() { return deployDelay; }

    static float xOffset();
    static float yOffset();

    static void setXOffset(float val) { offsetX = val; }
    static void setYOffset(float val) { offsetY = val; }

    static float getZProbeHeight();
    static void setZProbeHeight(float height);

    static float getCoating() { return 0; }
    static void setCoating(float val) { }

    static float getBedDistance() { return bedDistance; }
    static void setBedDistance(float val) { bedDistance = val; }

    static void showConfigMenu(GUIAction action);
    static bool hasConfigMenu() { return true; }

    static void showControlMenu(GUIAction action);
    static bool hasControlMenu() { return true; }

    static bool getHeaterPause() { return pauseHeaters; }
    static void setHeaterPause(bool set) { pauseHeaters = set; }

    static float getSpeed() { return speed; }
    static void setSpeed(float val) { speed = val; }
};

#else

#error Unknown Z_PROBE_TYPE value

#endif
