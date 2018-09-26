#if ZPROBE_TYPE == 0

class ZProbeHandler {
public:
    static float getZProbeHeight() { return 0; }
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
};

#elif ZPROBE_TYPE == 1

#define EPR_Z_PROBE_HEIGHT 0
#define EPR_Z_PROBE_BED_DISTANCE 4
#define EPR_Z_PROBE_SPEED 8
#define EPR_Z_PROBE_X_OFFSET 12
#define EPR_Z_PROBE_Y_OFFSET 16
#define EPR_Z_PROBE_MEMORY 20
// TODO: Real z probe ZProbeHandler
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
    static void activate();
    static void deactivate();
    static float runProbe();
    static bool probingPossible();
    static float xOffset();
    static float yOffset();
    static void init();
    static void eepromHandle();
    static void eepromReset() {}
    static float getCoating() { return coating; }
    static void setCoating(float val) { coating = val; }
};

#else

#error Unknown ZPROBE_TYPE value

#endif
