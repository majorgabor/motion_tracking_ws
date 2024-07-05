#ifndef _VECTOR3_H_
#define _VECTOR3_H_

struct Vector3
{
    double x, y, z;

    Vector3(const double a, const double b, const double c) : x(a), y(b), z(c) {}
};

#endif