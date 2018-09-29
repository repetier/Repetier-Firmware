#if ZPROBE_TYPE == 0

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
};

#elif ZPROBE_TYPE == 1

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
};

#else

#error Unknown ZPROBE_TYPE value

#endif
