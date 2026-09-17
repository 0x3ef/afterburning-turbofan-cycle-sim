#include "turbofan/reference_conditions.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void near(double actual, double expected, double tolerance, const char* explanation) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(explanation);
    }
}

template <typename Operation>
void require_domain_error(Operation operation, const char* explanation) {
    try {
        operation();
    } catch (const std::domain_error&) {
        return;
    }
    throw std::runtime_error(explanation);
}

void test_atmosphere() {
    using turbofan::standard_atmosphere;
    const auto sea_level = standard_atmosphere(0.0);
    near(sea_level.pressure, 101325.0, 0.0, "standard sea-level pressure");
    near(sea_level.temperature, 288.15, 0.0, "standard sea-level temperature");

    const auto tropopause = standard_atmosphere(11000.0);
    near(tropopause.pressure, 22632.06, 0.02, "standard tropopause pressure");
    near(tropopause.temperature, 216.65, 1e-10, "standard tropopause temperature");
    const auto just_above = standard_atmosphere(std::nextafter(11000.0, 20000.0));
    near(just_above.pressure, tropopause.pressure, 1e-9, "pressure must be continuous at 11 km");
    near(just_above.temperature, tropopause.temperature, 1e-10,
         "temperature must be continuous at 11 km");

    const auto at_40000_feet = standard_atmosphere(12192.0);
    near(at_40000_feet.pressure, 18753.93, 0.02, "standard pressure at 40,000 ft");
    near(at_40000_feet.temperature, 216.65, 1e-10, "isothermal temperature at 40,000 ft");
    const auto ceiling = standard_atmosphere(20000.0);
    near(ceiling.pressure, 5474.89, 0.02, "standard pressure at 20 km");

    require_domain_error([] { standard_atmosphere(-1.0); }, "negative altitude accepted");
    require_domain_error([] { standard_atmosphere(20000.1); }, "altitude above 20 km accepted");
    require_domain_error(
        [] { standard_atmosphere(std::numeric_limits<double>::quiet_NaN()); },
        "NaN altitude accepted");
}

void test_recovery() {
    using turbofan::tm84903_inlet_recovery;
    near(tm84903_inlet_recovery(0.0), 0.879, 0.0, "zero Mach recovery");
    near(tm84903_inlet_recovery(0.2), 0.936, 1e-12, "subsonic data node");
    near(tm84903_inlet_recovery(0.9), 0.988, 1e-12, "recovery near the peak");
    near(tm84903_inlet_recovery(1.35), 0.9735, 1e-12, "linear interpolation");
    near(tm84903_inlet_recovery(2.0), 0.943, 0.0, "upper Mach endpoint");

    require_domain_error([] { tm84903_inlet_recovery(-0.1); }, "negative Mach accepted");
    require_domain_error(
        [] { tm84903_inlet_recovery(std::nextafter(2.0, 3.0)); },
        "Mach above the curve domain accepted");
    require_domain_error(
        [] { tm84903_inlet_recovery(std::numeric_limits<double>::infinity()); },
        "infinite Mach accepted");
}

}  // namespace

int main() {
    try {
        test_atmosphere();
        test_recovery();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
