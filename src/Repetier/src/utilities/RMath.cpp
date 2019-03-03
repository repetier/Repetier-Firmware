#include "Repetier.h"

void LeastSquareQuadraticRegression1D::reset() {
    n = x = x2 = x3 = x4 = y = xy = x2y = 0;
}

void LeastSquareQuadraticRegression1D::add(float px, float py) {
    float px2 = px * px;
    n += 1;
    x += px;
    y += py;
    x2 += px2;
    x3 += px2 * px;
    x4 += px2 * px2;
    xy += px * py;
    x2y = px2 * py;
}

Quadratic1D LeastSquareQuadraticRegression1D::getRegression() {
    // https://www.azdhs.gov/documents/preparedness/state-laboratory/lab-licensure-certification/technical-resources/calibration-training/12-quadratic-least-squares-regression-calib.pdf
    float a, b, c;
    float sxx, sxy, sxx2, sx2y, sx2x2;
    sxx = x2 - x * x / n;
    sxy = xy - x * y / n;
    sxx2 = x3 - x * x2 / n;
    sx2y = x2y - x2 * y / n;
    sx2x2 = x4 - x2 * x2 / n;
    float det = sxx * sx2x2 - sxx2 * sxx2;
    a = (sx2y * sxx - sxy * sxx2) / det;
    b = (sxy * sx2x2 - sx2y * sxx2) / det;
    c = y / n - b * x / n - a * x2 / n;
    return Quadratic1D(a, b, c);
}
