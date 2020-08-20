#include "vec_ops.h"
#include <math.h>

using namespace std;

double inner_prod(double v1[3], double v2[3]) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


double norm2(double v[3]) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

double dist2(double v1[3], double v2[3]) {
    return (v1[0] - v2[0]) * (v1[0] - v2[0]) 
            + (v1[1] - v2[1]) * (v1[1] - v2[1])
            + (v1[2] - v2[2]) * (v1[2] - v2[2]);
}

void normalize(double v[3]) {
    static double tmp;
    tmp = 1.0 / sqrt(norm2(v));
    v[0] *= tmp;
    v[1] *= tmp;
    v[2] *= tmp;

    return;
}