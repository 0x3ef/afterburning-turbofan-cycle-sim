#pragma once

#include <vector>

namespace turbofan {

struct MapPoint {
    double corrected_flow;
    double pressure_ratio;
    double efficiency;
};

struct MapSample {
    double corrected_speed;
    double coordinate;
    MapPoint point;
};

class PerformanceMap {
public:
    // One sample per pair of coordinates on a complete rectilinear grid.
    explicit PerformanceMap(std::vector<MapSample> samples);

    // Bilinear interpolation on the closed grid domain. Queries outside it are rejected.
    MapPoint evaluate(double corrected_speed, double coordinate) const;

    // Canonical order: increasing corrected speed, then increasing coordinate.
    std::vector<MapSample> samples() const;

private:
    std::vector<double> speeds_;
    std::vector<double> coordinates_;
    std::vector<MapSample> samples_;
};

}  // namespace turbofan
