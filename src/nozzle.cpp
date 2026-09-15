#include "turbofan/nozzle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace turbofan {
namespace {

constexpr double temperature_factor = (gamma - 1.0) / 2.0;
constexpr double pressure_exponent = gamma / (gamma - 1.0);
constexpr double area_exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));

void require_positive(double value, const char* message) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::domain_error(message);
    }
}

bool exceeds_pressure(double pressure, double threshold) {
    // Treat rounding-level differences as the documented boundary regime.
    return pressure - threshold > 8.0 * std::numeric_limits<double>::epsilon() * threshold;
}

StaticState isentropic_exit(TotalState in, double mach) {
    const double factor = 1.0 + temperature_factor * mach * mach;
    const double pressure = std::exp(std::log(in.pressure) - pressure_exponent * std::log(factor));
    const double temperature = in.temperature / factor;
    require_positive(pressure, "nozzle exit pressure must be finite and positive");
    require_positive(temperature, "nozzle exit temperature must be finite and positive");
    return {pressure, temperature, mach};
}

StaticState subsonic_exit(TotalState in, double back_pressure) {
    const double log_pressure_ratio =
        std::log1p((back_pressure - in.pressure) / in.pressure);
    const double temperature_ratio = std::exp(log_pressure_ratio / pressure_exponent);
    const double mach =
        std::sqrt(std::expm1(-log_pressure_ratio / pressure_exponent) / temperature_factor);
    return {back_pressure, in.temperature * temperature_ratio, mach};
}

double log_area_ratio(double mach) {
    const double departure_from_sonic = (mach - 1.0) * (mach + 1.0);
    return area_exponent * std::log1p(temperature_factor / (1.0 + temperature_factor) *
                                     departure_from_sonic) -
           std::log(mach);
}

double supersonic_exit_mach(double throat_area, double exit_area) {
    const double area_increment = exit_area - throat_area;
    const double target = area_increment <= throat_area
                              ? std::log1p(area_increment / throat_area)
                              : std::log(exit_area) - std::log(throat_area);
    double low = 1.0;
    double high = 2.0;
    while (log_area_ratio(high) < target) {
        low = high;
        high *= 2.0;
    }
    while (high - low > std::numeric_limits<double>::epsilon() * high) {
        const double mid = std::midpoint(low, high);
        if (log_area_ratio(mid) < target) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return std::midpoint(low, high);
}

}  // namespace

NozzleResult convergent_divergent_nozzle(TotalState in, double throat_area, double exit_area,
                                          double back_pressure) {
    require_positive(in.pressure, "nozzle inlet pressure must be finite and positive");
    require_positive(in.temperature, "nozzle inlet temperature must be finite and positive");
    require_positive(in.mass_flow, "nozzle inlet mass flow must be finite and positive");
    require_positive(throat_area, "nozzle throat area must be finite and positive");
    require_positive(exit_area, "nozzle exit area must be finite and positive");
    require_positive(back_pressure, "nozzle back pressure must be finite and positive");
    if (exit_area < throat_area) {
        throw std::domain_error("nozzle exit area must be at least the throat area");
    }
    if (back_pressure >= in.pressure) {
        return {0.0, NozzleRegime::no_forward_flow, std::nullopt};
    }

    const double critical_pressure =
        in.pressure * std::pow(1.0 + temperature_factor, -pressure_exponent);
    const double choked_capacity = nozzle_capacity(in, throat_area, critical_pressure);
    if (exit_area == throat_area) {
        if (exceeds_pressure(back_pressure, critical_pressure)) {
            return {nozzle_capacity(in, throat_area, back_pressure), NozzleRegime::subsonic,
                    subsonic_exit(in, back_pressure)};
        }
        return {choked_capacity, NozzleRegime::choked_convergent,
                isentropic_exit(in, 1.0)};
    }

    // A sonic throat can still have a subsonic exit. Its exit pressure, not the
    // sonic throat pressure, determines the onset of choking in a divergent nozzle.
    if (back_pressure > critical_pressure) {
        const auto choked_subsonic = subsonic_static(
            {in.pressure, in.temperature, choked_capacity}, exit_area);
        if (!exceeds_pressure(choked_subsonic.pressure, back_pressure)) {
            const double capacity = nozzle_capacity(in, exit_area, back_pressure);
            return {std::min(capacity, choked_capacity), NozzleRegime::subsonic,
                    subsonic_exit(in, back_pressure)};
        }
    }

    const double exit_mach = supersonic_exit_mach(throat_area, exit_area);
    const double mach_excess = (exit_mach - 1.0) * (exit_mach + 1.0);
    const double log_pressure_jump = std::log1p(2.0 * gamma / (gamma + 1.0) * mach_excess);
    const double log_isentropic_pressure_drop =
        pressure_exponent * std::log1p(temperature_factor * exit_mach * exit_mach);
    const double shock_at_exit_pressure = std::exp(
        std::log(in.pressure) - log_isentropic_pressure_drop + log_pressure_jump);
    if (exceeds_pressure(back_pressure, shock_at_exit_pressure)) {
        return {choked_capacity, NozzleRegime::unsupported_internal_shock, std::nullopt};
    }
    return {choked_capacity, NozzleRegime::supersonic, isentropic_exit(in, exit_mach)};
}

}  // namespace turbofan
