#include "turbofan/performance_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace {

using turbofan::MapPoint;
using turbofan::MapSample;
using turbofan::PerformanceMap;

MapPoint reference(double speed, double position) {
    return {40.0 + 4.0 * speed + 2.0 * position + 3.0 * speed * position,
            2.0 + 0.1 * speed + 0.04 * position + 0.006 * speed * position,
            0.78 + 0.007 * speed + 0.003 * position + 0.002 * speed * position};
}

std::vector<MapSample> sample_grid() {
    std::vector<MapSample> samples;
    for (const double coordinate : {4.0, -1.0, 0.0}) {
        for (const double speed : {2.5, 5.0, 1.0}) {
            samples.push_back({speed, coordinate, reference(speed, coordinate)});
        }
    }
    return samples;
}

void near(MapPoint actual, MapPoint expected, double tolerance, const char* message) {
    const auto within = [tolerance](double a, double b) {
        return std::isfinite(a) && std::abs(a - b) <= tolerance;
    };
    if (!within(actual.corrected_flow, expected.corrected_flow) ||
        !within(actual.pressure_ratio, expected.pressure_ratio) ||
        !within(actual.efficiency, expected.efficiency)) {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Operation>
void expect_error(Operation operation, const char* message) {
    try {
        operation();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

void test_interpolation() {
    const auto unordered = sample_grid();
    const PerformanceMap map(unordered);
    for (const MapSample& sample : unordered) {
        near(map.evaluate(sample.corrected_speed, sample.coordinate), sample.point, 0.0,
             "map did not preserve the value at a grid node");
    }
    near(map.evaluate(1.7, -0.25), reference(1.7, -0.25), 1e-12, "first cell interpolation");
    near(map.evaluate(4.22, 3.1), reference(4.22, 3.1), 1e-12, "remaining cell interpolation");
    near(map.evaluate(5.0, 0.3), reference(5.0, 0.3), 1e-12, "upper speed edge interpolation");
    near(map.evaluate(1.7, 4.0), reference(1.7, 4.0), 1e-12, "upper coordinate edge interpolation");
    near(map.evaluate(1.0, 2.0), reference(1.0, 2.0), 1e-12, "lower speed edge interpolation");
    near(map.evaluate(3.0, -1.0), reference(3.0, -1.0), 1e-12, "lower coordinate edge interpolation");

    const auto canonical = map.samples();
    if (!std::is_sorted(canonical.begin(), canonical.end(), [](const MapSample& a, const MapSample& b) {
            return std::tie(a.corrected_speed, a.coordinate) < std::tie(b.corrected_speed, b.coordinate);
        }) || canonical.size() != unordered.size()) {
        throw std::runtime_error("map samples were not exported in canonical order");
    }
    const PerformanceMap restored(canonical);
    near(restored.evaluate(4.22, 3.1), map.evaluate(4.22, 3.1), 0.0, "map sample round trip");
}

void test_query_domain() {
    const PerformanceMap map(sample_grid());
    expect_error<std::domain_error>([&] { map.evaluate(0.99, 0.0); }, "map extrapolated below speed");
    expect_error<std::domain_error>([&] { map.evaluate(5.01, 0.0); }, "map extrapolated above speed");
    expect_error<std::domain_error>(
        [&] { map.evaluate(3.0, -1.01); }, "map extrapolated below coordinate");
    expect_error<std::domain_error>(
        [&] { map.evaluate(3.0, 4.01); }, "map extrapolated above coordinate");
    expect_error<std::domain_error>(
        [&] { map.evaluate(3.0, std::numeric_limits<double>::quiet_NaN()); },
        "map accepted a nonfinite query");
}

void test_invalid_grids() {
    expect_error<std::invalid_argument>([] { PerformanceMap map({}); }, "map accepted an empty grid");
    auto samples = sample_grid();
    samples.pop_back();
    expect_error<std::invalid_argument>([&] { PerformanceMap map(samples); }, "map accepted a hole");
    samples.push_back(samples.front());
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted a duplicate and a hole");

    samples = sample_grid();
    samples.resize(3);
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted only one coordinate");
    samples = sample_grid();
    samples[0].corrected_speed = 0.0;
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted a nonpositive corrected speed");
    samples = sample_grid();
    samples[0].coordinate = std::numeric_limits<double>::infinity();
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted a nonfinite coordinate");
    samples = sample_grid();
    samples[0].point.corrected_flow = 0.0;
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted zero corrected flow");
    samples = sample_grid();
    samples[0].point.pressure_ratio = 0.99;
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted pressure ratio below one");
    samples = sample_grid();
    samples[0].point.efficiency = 1.01;
    expect_error<std::invalid_argument>(
        [&] { PerformanceMap map(samples); }, "map accepted efficiency above one");
}

void test_large_coordinate_span() {
    const double max = std::numeric_limits<double>::max();
    const MapPoint low{8.0, 1.0, 0.5};
    const MapPoint high{12.0, 2.0, 1.0};
    const PerformanceMap map({{1.0, -max, low}, {1.0, max, high},
                              {2.0, -max, low}, {2.0, max, high}});
    near(map.evaluate(1.5, 0.0), {10.0, 1.5, 0.75}, 0.0, "large coordinate span interpolation");
}

}  // namespace

int main() {
    try {
        test_interpolation();
        test_query_domain();
        test_invalid_grids();
        test_large_coordinate_span();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
