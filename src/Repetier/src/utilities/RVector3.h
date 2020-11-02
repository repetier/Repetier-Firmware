class RVector3 {
public:
    float x, y, z;
    RVector3(float _x = 0, float _y = 0, float _z = 0)
        : x(_x)
        , y(_y)
        , z(_z) {};
    RVector3(const RVector3& a)
        : x(a.x)
        , y(a.y)
        , z(a.z) {};

    /*    const float &operator[](std::size_t idx) const
    {
        if(idx == 0) return x;
        if(idx == 1) return y;
        return z;
    };

    float &operator[](std::size_t idx)
    {
        switch(idx) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        }
        return 0;
    };*/

    inline bool operator==(const RVector3& rhs) {
        // file deepcode ignore FloatingPointEquals: <please specify a reason of ignoring this>
        // file deepcode ignore FloatingPointEquals: <please specify a reason of ignoring this>
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    inline bool operator!=(const RVector3& rhs) {
        return !(*this == rhs);
    }
    inline RVector3& operator=(const RVector3& rhs) {
        if (this != &rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    inline RVector3& operator+=(const RVector3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline RVector3& operator-=(const RVector3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    inline RVector3 operator-() const {
        return RVector3(-x, -y, -z);
    }

    inline float length() const {
        return sqrtf(x * x + y * y + z * z);
    }

    inline float lengthSquared() const {
        return (x * x + y * y + z * z);
    }

    inline RVector3 cross(const RVector3& b) const {
        return RVector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
    inline float scalar(const RVector3& b) const {
        return (x * b.x + y * b.y + z * b.z);
    }
    inline RVector3 scale(float factor) const {
        return RVector3(x * factor, y * factor, z * factor);
    }
    inline void scaleIntern(float factor) {
        x *= factor;
        y *= factor;
        z *= factor;
    }
    inline void setMinimum(const RVector3& b) {
        x = RMath::min(x, b.x);
        y = RMath::min(y, b.y);
        z = RMath::min(z, b.z);
    }
    inline void setMaximum(const RVector3& b) {
        x = RMath::max(x, b.x);
        y = RMath::max(y, b.y);
        z = RMath::max(z, b.z);
    }
    inline float distance(const RVector3& b) const {
        float dx = b.x - x, dy = b.y - y, dz = b.z - z;
        return (sqrt(dx * dx + dy * dy + dz * dz));
    }
    inline float angle(RVector3& direction) {
        return static_cast<float>(acos(scalar(direction) / (length() * direction.length())));
    }

    inline RVector3 normalize() const {
        float len = length();
        if (len != 0)
            len = static_cast<float>(1.0 / len);
        return RVector3(x * len, y * len, z * len);
    }

    inline RVector3 interpolatePosition(const RVector3& b, float pos) const {
        float pos2 = 1.0f - pos;
        return RVector3(x * pos2 + b.x * pos, y * pos2 + b.y * pos, z * pos2 + b.z * pos);
    }

    inline RVector3 interpolateDirection(const RVector3& b, float pos) const {
        //float pos2 = 1.0f - pos;

        float dot = scalar(b);
        if (dot > 0.9995 || dot < -0.9995)
            return interpolatePosition(b, pos); // cases cause trouble, use linear interpolation here

        float theta = acos(dot) * pos; // interpolated position
        float st = sin(theta);
        RVector3 t(b);
        t -= scale(dot);
        float lengthSq = t.lengthSquared();
        float dl = st * ((lengthSq < 0.0001f) ? 1.0f : 1.0f / sqrt(lengthSq));
        t.scaleIntern(dl);
        t += scale(cos(theta));
        return t.normalize();
    }
};
inline RVector3 operator+(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

inline RVector3 operator-(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

inline RVector3 operator*(const RVector3& lhs, float rhs) {
    return lhs.scale(rhs);
}

inline RVector3 operator*(float lhs, const RVector3& rhs) {
    return rhs.scale(lhs);
}

template <int rows, int cols>
class RMatrix {
    float data[rows][cols];

public:
    RMatrix() {
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                data[r][c] = 0.0;
            }
        }
    }

    void print(FSTRINGPARAM(name)) {
        Com::printF("Matrix ");
        Com::printFLN(name);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                Com::printF(Com::tEmpty, data[r][c], 4);
                if (c < cols - 1) {
                    Com::printF(PSTR(", "));
                } else {
                    Com::println();
                }
            }
        }
    }

    void swapRows(int i, int j) {
        for (int c = 0; c < cols; c++) {
            float tmp = data[i][c];
            data[i][c] = data[j][c];
            data[j][c] = tmp;
        }
    }

    void gaussJordan(float solution[rows], int numRows = rows) {
        int i, j, k;
        for (i = 0; i < numRows; i++) {
            float vmax = fabs(data[i][i]);
            for (j = i + 1; j < numRows; j++) { // ensure biggest diagonal entry for stability
                float rmax = fabs(data[j][i]);
                if (rmax > vmax) {
                    swapRows(i, j);
                    vmax = rmax;
                }
            }
            float v = data[i][i];
            for (j = 0; j < i; j++) {
                float factor = data[j][i] / v;
                data[j][i] = 0.0;
                for (k = i + 1; k <= numRows; k++) {
                    data[j][k] -= data[i][k] * factor;
                }
            }
            for (j = i + 1; j < numRows; j++) {
                float factor = data[j][i] / v;
                data[j][i] = 0.0;
                for (k = i + 1; k <= numRows; k++) {
                    data[j][k] -= data[i][k] * factor;
                }
            }
        }
        for (i = 0; i < numRows; i++) {
            solution[i] = data[i][numRows] / data[i][i];
        }
    }
    float& operator()(int r, int c) { return data[r][c]; }
};