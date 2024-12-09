#ifndef NICE_RANGE_HPP
#define NICE_RANGE_HPP

#include <cmath> 
#include <stdexcept>
#include <algorithm>

struct NiceRange {
    double startTick;
    double endTick;
    double tickWidth;
    int fractionPrecision;
};

inline double computeNiceNumber(double value, bool round) {
    int exponent { static_cast<int>(std::floor(std::log10(value))) };
    double fraction { value / std::pow(10, static_cast<double>(exponent)) };
    double niceFraction;

    if (round) {
        if (fraction < 1.5)
            niceFraction = 1.0;
        else if (fraction < 3.0)
            niceFraction = 2.0;
        else if (fraction < 7.0)
            niceFraction = 5.0;
        else
            niceFraction = 10.0;
    }
    else {
        if (fraction <= 1.0)
            niceFraction = 1.0;
        else if (fraction <= 2.0)
            niceFraction = 2.0;
        else if (fraction <= 5.0)
            niceFraction = 5.0;
        else
            niceFraction = 10.0;
    }
    return niceFraction * std::pow(10, static_cast<double>(exponent));
}

inline NiceRange getNiceRange(double startTick, double endTick, int tickCount) {
    double extent{};
    if (startTick > endTick || tickCount < 2)
        throw std::invalid_argument("Invalid arguments.");
    if (startTick == endTick)
        extent = 0.1 * startTick;
    else 
        extent = endTick - startTick;

    double niceExtent { computeNiceNumber(extent, false) };
    double niceTickWidth { computeNiceNumber(niceExtent / (tickCount - 1), true) };
    int fractionPrecision { static_cast<int>(std::max(-std::floor(std::log10(niceTickWidth)), 0.0)) }; 
    startTick = std::floor(startTick / niceTickWidth) * niceTickWidth;
    endTick = std::ceil(endTick / niceTickWidth) * niceTickWidth;
    return {startTick, endTick, niceTickWidth, fractionPrecision};
}

#endif