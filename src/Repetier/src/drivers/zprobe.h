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
};

#else

// TODO: Real z probe ZProbeHandler

#endif
