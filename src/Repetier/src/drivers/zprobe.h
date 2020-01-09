#if Z_PROBE_TYPE == Z_PROBE_TYPE_NONE

class ZProbeHandler {
public:
    static float getZProbeHeight() { return 0; }
    static void setZProbeHeight(float height) {}
    static void activate() {}
    static void deactivate() {}
    static float runProbe() { return 0; }
    static bool probingPossible() { return false; }
    static float xOffset() { return 0; }
    static float yOffset() { return 0; }
    static void init() {}
    static void eepromHandle() {}
    static void eepromReset() {}
    static float getCoating() { return 0; }
    static void setCoating(float val) {}
    static float getBedDistance() { return 0; }
    static float optimumProbingHeight() { return 0; }
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

public:
    static float getZProbeHeight();
    static void setZProbeHeight(float height);
    static void activate();
    static void deactivate();
    static float runProbe();
    static bool probingPossible();
    static float xOffset();
    static float yOffset();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float getCoating() { return coating; }
    static void setCoating(float val) { coating = val; }
    static float getBedDistance() { return bedDistance; }
    static float optimumProbingHeight();
};

#elif Z_PROBE_TYPE == Z_PROBE_TYPE_NOZZLE

// Nozzle based limit switch. Ensures a minimum temperature. Offset is
// not required since nozzle is the probe we use nozzle offset.

class ZProbeHandler {
    static uint16_t eprStart;
    static float height;
    static float bedDistance;
    static float speed;
    static int16_t probeTemperature;
    static bool activated;
    static int16_t activateTemperature;

public:
    static float getZProbeHeight();
    static void setZProbeHeight(float height);
    static void activate();
    static void deactivate();
    static float runProbe();
    static bool probingPossible();
    static float xOffset();
    static float yOffset();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float getCoating() { return 0; }
    static void setCoating(float val) {}
    static float getBedDistance() { return bedDistance; }
    static float optimumProbingHeight();
};

#elif Z_PROBE_TYPE == Z_PROBE_TYPE_BLTOUCH

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

    static bool isAlarmOn();
    static void disableAlarmIfOn();

public:
    static float getZProbeHeight();
    static void setZProbeHeight(float height);
    static void activate();
    static void deactivate();
    static float runProbe();
    static bool probingPossible();
    static float xOffset();
    static float yOffset();
    static void init();
    static void eepromHandle();
    static void eepromReset();
    static float getCoating() { return coating; }
    static void setCoating(float val) { coating = val; }
    static float getBedDistance() { return bedDistance; }
    static float optimumProbingHeight();
};

#else

#error Unknown Z_PROBE_TYPE value

#endif
