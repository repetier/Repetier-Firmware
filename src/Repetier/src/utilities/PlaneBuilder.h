class PlaneBuilder {
    float sum_xx, sum_xy, sum_yy, sum_x, sum_y, sum_xz, sum_yz, sum_z, n;

public:
    PlaneBuilder() {
        reset();
    }
    void reset() {
        sum_xx = sum_xy = sum_yy = sum_x = sum_y = sum_xz = sum_yz = sum_z = n = 0;
    }
    void addPoint(float x, float y, float z) {
        n++;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_yy += y * y;
        sum_x += x;
        sum_y += y;
        sum_xz += x * z;
        sum_yz += y * z;
        sum_z += z;
    }
    void createPlane(Plane& plane, bool silent = false) {
        float det = (sum_x * (sum_xy * sum_y - sum_x * sum_yy) + sum_xx * (n * sum_yy - sum_y * sum_y) + sum_xy * (sum_x * sum_y - n * sum_xy));
        plane.a = ((sum_xy * sum_y - sum_x * sum_yy) * sum_z + (sum_x * sum_y - n * sum_xy) * sum_yz + sum_xz * (n * sum_yy - sum_y * sum_y)) / det;
        plane.b = ((sum_x * sum_xy - sum_xx * sum_y) * sum_z + (n * sum_xx - sum_x * sum_x) * sum_yz + sum_xz * (sum_x * sum_y - n * sum_xy)) / det;
        plane.c = ((sum_xx * sum_yy - sum_xy * sum_xy) * sum_z + (sum_x * sum_xy - sum_xx * sum_y) * sum_yz + sum_xz * (sum_xy * sum_y - sum_x * sum_yy)) / det;
        if (!silent) {
            Com::printF(PSTR("plane: a = "), plane.a, 4);
            Com::printF(PSTR(" b = "), plane.b, 4);
            Com::printFLN(PSTR(" c = "), plane.c, 4);
            Com::printFLN(PSTR("z = a * x + y * b + c"));
        }
    }
    inline int numPoints() { return n; }
};
