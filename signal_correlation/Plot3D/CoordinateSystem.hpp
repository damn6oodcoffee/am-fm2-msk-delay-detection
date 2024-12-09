#ifndef COORDINATE_SYSTEM_HPP
#define COORDINATE_SYSTEM_HPP

#include <cmath>

struct CartesianPoint {
    double x, y, z;
};

struct UVPoint {
    double u, v, z;
};

struct SphericalPoint {
    double r, theta, phi;
};

inline CartesianPoint sphericalToCartesian(const SphericalPoint& point) {
    return CartesianPoint{
        point.r * std::sin(point.theta) * std::cos(point.phi),
        point.r * std::sin(point.theta) * std::sin(point.phi),
        point.r * std::cos(point.theta),
    };
}

inline UVPoint sphericalToUV(const SphericalPoint& point) {
    return UVPoint{
        std::sin(point.theta) * std::cos(point.phi),
        std::sin(point.theta) * std::sin(point.phi),
        point.r,
    };
}

#endif

