#undef min
#undef max

class RMath {
public:
    static inline float min(float a, float b) {
        if (a < b)
            return a;
        return b;
    }
    static inline float max(float a, float b) {
        if (a < b)
            return b;
        return a;
    }
    static inline int32_t min(int32_t a, int32_t b) {
        if (a < b)
            return a;
        return b;
    }
    static inline int32_t min(int32_t a, int32_t b, int32_t c) {
        if (a < b)
            return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline float min(float a, float b, float c) {
        if (a < b)
            return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a, int32_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline int min(int a, int b) {
        if (a < b)
            return a;
        return b;
    }
    static inline uint16_t min(uint16_t a, uint16_t b) {
        if (a < b)
            return a;
        return b;
    }
    static inline int16_t max(int16_t a, int16_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline uint16_t max(uint16_t a, uint16_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline unsigned long absLong(long a) {
        return a >= 0 ? a : -a;
    }
    static inline int32_t sqr(int32_t a) {
        return a * a;
    }
    static inline uint32_t sqr(uint32_t a) {
        return a * a;
    }
#ifdef SUPPORT_64_BIT_MATH
    static inline int64_t sqr(int64_t a) {
        return a * a;
    }
    static inline uint64_t sqr(uint64_t a) {
        return a * a;
    }
#endif

    static inline float sqr(float a) {
        return a * a;
    }
};
