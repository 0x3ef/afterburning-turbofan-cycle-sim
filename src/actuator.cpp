#include "turbofan/actuator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace turbofan {
namespace {

bool nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool valid_response(ActuatorResponse response) {
    return positive(response.time_constant_seconds) && nonnegative(response.rate_limit_per_second);
}

bool valid_commands(ControllerCommands commands, double area_ratio) {
    const double expected_exit_area = commands.throat_area * area_ratio;
    const double tolerance = 16.0 * std::numeric_limits<double>::epsilon() * expected_exit_area;
    return nonnegative(commands.main_fuel_flow) && nonnegative(commands.augmentor_fuel_flow) &&
           positive(commands.throat_area) && std::isfinite(commands.exit_area) &&
           commands.exit_area >= commands.throat_area && std::isfinite(expected_exit_area) &&
           std::abs(commands.exit_area - expected_exit_area) <= tolerance;
}

double advance(double current, double target, ActuatorResponse response, double dt_seconds) {
    const double fraction = -std::expm1(-dt_seconds / response.time_constant_seconds);
    const double maximum_change = response.rate_limit_per_second * dt_seconds;
    const double change = std::clamp((target - current) * fraction, -maximum_change, maximum_change);
    return std::clamp(current + change, std::min(current, target), std::max(current, target));
}

}  // namespace

ControllerCommands advance_actuators(ControllerCommands current, ControllerCommands target,
                                     const ActuatorConfig& config, double dt_seconds) {
    if (!positive(dt_seconds) || !valid_response(config.main_fuel) ||
        !valid_response(config.augmentor_fuel) || !valid_response(config.throat_area) ||
        !std::isfinite(config.exit_to_throat_ratio) || config.exit_to_throat_ratio < 1.0) {
        throw std::domain_error("invalid actuator response, area ratio or sample interval");
    }
    if (!valid_commands(current, config.exit_to_throat_ratio) ||
        !valid_commands(target, config.exit_to_throat_ratio)) {
        throw std::domain_error("invalid actuator command or exit-to-throat area ratio");
    }

    const double throat_area =
        advance(current.throat_area, target.throat_area, config.throat_area, dt_seconds);
    return {advance(current.main_fuel_flow, target.main_fuel_flow, config.main_fuel, dt_seconds),
            advance(current.augmentor_fuel_flow, target.augmentor_fuel_flow, config.augmentor_fuel,
                    dt_seconds),
            throat_area, throat_area * config.exit_to_throat_ratio};
}

}  // namespace turbofan
