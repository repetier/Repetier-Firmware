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

    static inline float sqr(float a) {
        return a * a;
    }

    template <typename T, typename X>
    static T GCD(T a, X b) {
        while (b != 0) {
            T t = a % b;
            a = b;
            b = t;
        }
        return a;
    }
    
    template <typename T, typename X>
    constexpr static T LCM(T __m, X __n) {
        return (__m != 0 && __n != 0) ? (__m / GCD(__m, __n)) * __n : 0;
    }
};

class Quadratic1D {
public:
    float a, b, c;
    Quadratic1D(float _a = 0, float _b = 0, float _c = 0)
        : a(_a)
        , b(_b)
        , c(_c) {}
    inline float y(float x) {
        return a * x * x + b * x + c;
    }
};

/** One dimensional least squre curve 
  y = a * x^2 + b * x + c
  based of any number of xy pairs. */

class LeastSquareQuadraticRegression1D {
    float n, x, x2, x3, x4;
    float y, xy, x2y;

public:
    void reset();
    void add(float px, float py);
    Quadratic1D getRegression();
};