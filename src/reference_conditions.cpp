#include "turbofan/reference_conditions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace turbofan {
namespace {

constexpr double sea_level_pressure = 101325.0;
constexpr double sea_level_temperature = 288.15;
constexpr double standard_gravity = 9.80665;
constexpr double dry_air_gas_constant = 8314.32 / 28.9644;  // J/(kg K), 1976 standard.
constexpr double lapse_rate = 0.0065;                     // K/m below the tropopause.
constexpr double tropopause = 11000.0;                    // Geopotential meters.
constexpr double tropopause_temperature = sea_level_temperature - lapse_rate * tropopause;

double tropospheric_pressure(double temperature) {
    return sea_level_pressure *
           std::pow(temperature / sea_level_temperature,
                    standard_gravity / (dry_air_gas_constant * lapse_rate));
}

// NASA TM 84903, printed p. 9, Fig. 6. Line-center readings at increments of 0.1 Mach.
// The curve was an assumed Pt2/Pt0 input to the report's simulation. These values
// have about 0.002 absolute reading uncertainty, rising to about 0.003 near the ends.
constexpr std::array recovery{
    0.879, 0.911, 0.936, 0.953, 0.964, 0.973, 0.979,
    0.984, 0.986, 0.988, 0.986, 0.981, 0.977, 0.974,
    0.973, 0.972, 0.971, 0.968, 0.963, 0.955, 0.943,
};

}  // namespace

AmbientConditions standard_atmosphere(double altitude_m) {
    if (!std::isfinite(altitude_m) || altitude_m < 0.0 || altitude_m > 20000.0) {
        throw std::domain_error("standard atmosphere altitude must be in [0, 20000] geopotential m");
    }
    // Hydrostatic pressure with the standard constant lapse rate below 11 km.
    if (altitude_m <= tropopause) {
        const double temperature = sea_level_temperature - lapse_rate * altitude_m;
        return {tropospheric_pressure(temperature), temperature};
    }

    // The 11–20 km layer is isothermal. Take its base pressure from the lower layer.
    const double pressure = tropospheric_pressure(tropopause_temperature) *
                            std::exp(-standard_gravity * (altitude_m - tropopause) /
                                     (dry_air_gas_constant * tropopause_temperature));
    return {pressure, tropopause_temperature};
}

double tm84903_inlet_recovery(double mach) {
    if (!std::isfinite(mach) || mach < 0.0 || mach > 2.0) {
        throw std::domain_error("TM 84903 inlet recovery Mach must be in [0, 2]");
    }
    const double position = 10.0 * mach;
    const std::size_t lower = std::min(static_cast<std::size_t>(position), recovery.size() - 2);
    return std::lerp(recovery[lower], recovery[lower + 1], position - static_cast<double>(lower));
}

}  // namespace turbofan
