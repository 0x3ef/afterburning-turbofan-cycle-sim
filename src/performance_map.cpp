#include "turbofan/performance_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace turbofan {
namespace {

void sort_unique(std::vector<double>& axis) {
    std::sort(axis.begin(), axis.end());
    axis.erase(std::unique(axis.begin(), axis.end()), axis.end());
}

struct Interval {
    std::size_t lower;
    double fraction;
};

Interval interval(const std::vector<double>& axis, double value, const char* error) {
    if (!std::isfinite(value) || value < axis.front() || value > axis.back()) {
        throw std::domain_error(error);
    }
    const auto upper = std::upper_bound(axis.begin(), axis.end() - 1, value);
    const double lower_value = *(upper - 1);
    const double upper_value = *upper;
    const double width = upper_value - lower_value;
    // Scale before subtraction if opposite-sign coordinates span more than double can hold.
    const double fraction = std::isfinite(width)
        ? (value - lower_value) / width
        : (value / 2.0 - lower_value / 2.0) / (upper_value / 2.0 - lower_value / 2.0);
    return {static_cast<std::size_t>(std::distance(axis.begin(), upper) - 1), fraction};
}

MapPoint interpolate(MapPoint a, MapPoint b, double fraction) {
    return {std::lerp(a.corrected_flow, b.corrected_flow, fraction),
            std::lerp(a.pressure_ratio, b.pressure_ratio, fraction),
            std::lerp(a.efficiency, b.efficiency, fraction)};
}

}  // namespace

PerformanceMap::PerformanceMap(std::vector<MapSample> samples) : samples_(std::move(samples)) {
    speeds_.reserve(samples_.size());
    coordinates_.reserve(samples_.size());
    for (const MapSample& sample : samples_) {
        const MapPoint& point = sample.point;
        if (!std::isfinite(sample.corrected_speed) || sample.corrected_speed <= 0.0 ||
            !std::isfinite(sample.coordinate) || !std::isfinite(point.corrected_flow) ||
            point.corrected_flow <= 0.0 || !std::isfinite(point.pressure_ratio) ||
            point.pressure_ratio < 1.0 || !std::isfinite(point.efficiency) ||
            point.efficiency <= 0.0 || point.efficiency > 1.0) {
            throw std::invalid_argument("performance map sample has invalid axes or values");
        }
        speeds_.push_back(sample.corrected_speed);
        coordinates_.push_back(sample.coordinate);
    }
    sort_unique(speeds_);
    sort_unique(coordinates_);
    if (speeds_.size() < 2 || coordinates_.size() < 2) {
        throw std::invalid_argument("performance map requires at least two values on each axis");
    }
    if (samples_.size() % speeds_.size() != 0 ||
        samples_.size() / speeds_.size() != coordinates_.size()) {
        throw std::invalid_argument("performance map requires a complete grid without duplicates");
    }

    std::sort(samples_.begin(), samples_.end(), [](const MapSample& a, const MapSample& b) {
        return std::tie(a.corrected_speed, a.coordinate) < std::tie(b.corrected_speed, b.coordinate);
    });
    std::size_t index = 0;
    for (const double speed : speeds_) {
        for (const double coordinate : coordinates_) {
            const MapSample& sample = samples_[index++];
            if (sample.corrected_speed != speed || sample.coordinate != coordinate) {
                throw std::invalid_argument("performance map has a missing or duplicate sample");
            }
        }
    }
}

MapPoint PerformanceMap::evaluate(double corrected_speed, double coordinate) const {
    const Interval speed = interval(speeds_, corrected_speed, "corrected speed outside map domain");
    const Interval position = interval(coordinates_, coordinate, "coordinate outside map domain");
    const std::size_t lower = speed.lower * coordinates_.size() + position.lower;
    const std::size_t upper = lower + coordinates_.size();
    const MapPoint low = interpolate(samples_[lower].point, samples_[lower + 1].point,
                                     position.fraction);
    const MapPoint high = interpolate(samples_[upper].point, samples_[upper + 1].point,
                                      position.fraction);
    return interpolate(low, high, speed.fraction);
}

std::vector<MapSample> PerformanceMap::samples() const {
    return samples_;
}

}  // namespace turbofan
