#include "turbofan/thermo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace turbofan {
namespace {

constexpr double temperature_factor = (gamma - 1.0) / 2.0;
constexpr double pressure_exponent = gamma / (gamma - 1.0);
constexpr double flow_exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0));

void require_positive(double value, const char* message) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::domain_error(message);
    }
}

void require_nonnegative(double value, const char* message) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::domain_error(message);
    }
}

void require_fraction(double value, const char* message) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
        throw std::domain_error(message);
    }
}

void require_pressure_ratio(double value) {
    if (!std::isfinite(value) || value < 1.0) {
        throw std::domain_error("pressure ratio must be finite and at least one");
    }
}

TotalState checked(TotalState state) {
    require_positive(state.pressure, "total pressure must be finite and positive");
    require_positive(state.temperature, "total temperature must be finite and positive");
    require_positive(state.mass_flow, "mass flow must be finite and positive");
    return state;
}

double static_factor(double mach) {
    return 1.0 + temperature_factor * mach * mach;
}

double flow_at_mach(double scale, double mach) {
    return scale * mach / std::pow(static_factor(mach), flow_exponent);
}

double flow_scale(TotalState in, double area) {
    const double scale =
        std::sqrt(gamma / gas_constant) * area * (in.pressure / std::sqrt(in.temperature));
    require_positive(scale, "flow scale must be finite and positive");
    return scale;
}

}  // namespace

TotalState inlet(double ambient_pressure, double ambient_temperature, double mach,
                 double recovery, double mass_flow) {
    require_positive(ambient_pressure, "ambient pressure must be finite and positive");
    require_positive(ambient_temperature, "ambient temperature must be finite and positive");
    require_nonnegative(mach, "Mach number must be finite and nonnegative");
    require_fraction(recovery, "inlet recovery must be in (0, 1]");
    const double factor = static_factor(mach);
    return checked({ambient_pressure * recovery * std::pow(factor, pressure_exponent),
                    ambient_temperature * factor, mass_flow});
}

TotalState compress(TotalState in, double pressure_ratio, double efficiency) {
    checked(in);
    require_pressure_ratio(pressure_ratio);
    require_fraction(efficiency, "compressor efficiency must be in (0, 1]");
    const double isentropic_rise = std::expm1(std::log(pressure_ratio) / pressure_exponent);
    return checked({in.pressure * pressure_ratio,
                    in.temperature * (1.0 + isentropic_rise / efficiency), in.mass_flow});
}

TotalState expand(TotalState in, double pressure_ratio, double efficiency) {
    checked(in);
    require_pressure_ratio(pressure_ratio);
    require_fraction(efficiency, "turbine efficiency must be in (0, 1]");
    const double isentropic_temperature_ratio =
        std::pow(pressure_ratio, -1.0 / pressure_exponent);
    const double temperature_ratio =
        (1.0 - efficiency) + efficiency * isentropic_temperature_ratio;
    return checked({in.pressure / pressure_ratio, in.temperature * temperature_ratio,
                    in.mass_flow});
}

TotalState burn(TotalState in, double fuel_flow, double heating_value, double efficiency,
                double recovery) {
    checked(in);
    require_nonnegative(fuel_flow, "fuel flow must be finite and nonnegative");
    require_positive(heating_value, "fuel heating value must be finite and positive");
    require_fraction(efficiency, "combustion efficiency must be in (0, 1]");
    require_fraction(recovery, "combustor recovery must be in (0, 1]");
    const double mass_flow = in.mass_flow + fuel_flow;
    const double temperature = (in.mass_flow / mass_flow) * in.temperature +
                               (fuel_flow / mass_flow) * (heating_value / cp) * efficiency;
    return checked({in.pressure * recovery, temperature, mass_flow});
}

TotalState augment(TotalState in, double air_mass_flow, double upstream_fuel_flow,
                   double augmentor_fuel_flow, double heating_value, double efficiency,
                   double recovery, double stoichiometric_fuel_air_ratio) {
    checked(in);
    require_positive(air_mass_flow, "augmentor air mass flow must be finite and positive");
    require_nonnegative(upstream_fuel_flow, "upstream fuel flow must be finite and nonnegative");
    require_nonnegative(augmentor_fuel_flow, "augmentor fuel flow must be finite and nonnegative");
    require_positive(stoichiometric_fuel_air_ratio,
                     "stoichiometric fuel-air ratio must be finite and positive");

    const double inlet_mass_flow = air_mass_flow + upstream_fuel_flow;
    constexpr double mass_tolerance = 16.0 * std::numeric_limits<double>::epsilon();
    if (!std::isfinite(inlet_mass_flow) ||
        std::abs(in.mass_flow - inlet_mass_flow) / std::max(in.mass_flow, inlet_mass_flow) >
            mass_tolerance) {
        throw std::domain_error("augmentor inlet mass flow must match air plus upstream fuel");
    }

    if (augmentor_fuel_flow > 0.0) {
        const double fuel_remaining =
            std::fma(stoichiometric_fuel_air_ratio, air_mass_flow, -upstream_fuel_flow);
        if (augmentor_fuel_flow > fuel_remaining) {
            throw std::domain_error("augmentor fuel exceeds the remaining fuel-air allocation");
        }
    }
    return burn(in, augmentor_fuel_flow, heating_value, efficiency, recovery);
}

TotalState mix(TotalState core, TotalState bypass, double recovery) {
    checked(core);
    checked(bypass);
    require_fraction(recovery, "mixer recovery must be in (0, 1]");
    const double mass_flow = core.mass_flow + bypass.mass_flow;
    const double temperature = (core.mass_flow / mass_flow) * core.temperature +
                               (bypass.mass_flow / mass_flow) * bypass.temperature;
    const double pressure = std::min(core.pressure, bypass.pressure) * recovery;
    return checked({pressure, temperature, mass_flow});
}

StaticState subsonic_static(TotalState in, double area) {
    checked(in);
    require_positive(area, "flow area must be finite and positive");
    const double scale = flow_scale(in, area);
    const double capacity = flow_at_mach(scale, 1.0);
    const double roundoff = 8.0 * std::numeric_limits<double>::epsilon() * capacity;
    if (in.mass_flow - capacity > roundoff) {
        throw std::domain_error("mass flow exceeds the sonic capacity of the area");
    }

    double low = 0.0;
    double high = 1.0;
    // Bracket the monotone subsonic solution. Relative termination also handles low Mach.
    while (high - low > std::numeric_limits<double>::epsilon() * high) {
        const double mid = low + (high - low) / 2.0;
        if (mid == low || mid == high) {
            break;
        }
        const double flow = flow_at_mach(scale, mid);
        if (flow == in.mass_flow) {
            low = high = mid;
        } else if (flow < in.mass_flow) {
            low = mid;
        } else {
            high = mid;
        }
    }
    const double mach = in.mass_flow >= capacity - roundoff ? 1.0 : high;
    const double factor = static_factor(mach);
    return {in.pressure / std::pow(factor, pressure_exponent), in.temperature / factor, mach};
}

double nozzle_capacity(TotalState in, double throat_area, double back_pressure) {
    checked(in);
    require_positive(throat_area, "nozzle throat area must be finite and positive");
    require_positive(back_pressure, "nozzle back pressure must be finite and positive");
    if (back_pressure >= in.pressure) {
        return 0.0;
    }

    const double critical_ratio = std::pow(static_factor(1.0), -pressure_exponent);
    const double pressure_ratio = back_pressure / in.pressure;
    double mach = 1.0;
    if (pressure_ratio > critical_ratio) {
        const double log_ratio = std::log1p((back_pressure - in.pressure) / in.pressure);
        mach = std::sqrt(std::expm1(-log_ratio / pressure_exponent) / temperature_factor);
    }
    return flow_at_mach(flow_scale(in, throat_area), mach);
}

}  // namespace turbofan
