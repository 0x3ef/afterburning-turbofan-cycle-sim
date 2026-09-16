#include "turbofan/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace turbofan {
namespace {

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool valid_feedback(ControllerFeedback feedback) {
    return positive(feedback.lp_speed_rpm) && positive(feedback.pressure_ratio);
}

bool valid_pi_settings(PiSettings pi) {
    return nonnegative(pi.proportional_gain) && nonnegative(pi.integral_gain) &&
           nonnegative(pi.minimum) && std::isfinite(pi.maximum) && pi.minimum <= pi.maximum;
}

bool within(double command, PiSettings pi) {
    return std::isfinite(command) && command >= pi.minimum && command <= pi.maximum;
}

bool agree_at_trim(double value, double expected) {
    return nonnegative(value) &&
           std::abs(value - expected) <= 16.0 * std::numeric_limits<double>::epsilon() * expected;
}

void validate_config(const ControllerConfig& config) {
    if (config.schedule.size() < 2 || config.schedule.front().throttle != 0.0 ||
        config.schedule.back().throttle != 1.0) {
        throw std::domain_error("controller schedule must cover throttle from 0 to 1");
    }
    double previous = -1.0;
    for (const auto& point : config.schedule) {
        if (!nonnegative(point.throttle) || point.throttle <= previous ||
            !valid_feedback({point.lp_speed_rpm, point.pressure_ratio}) ||
            !nonnegative(point.augmentor_fuel_flow)) {
            throw std::domain_error("invalid controller schedule knot");
        }
        previous = point.throttle;
    }
    if (!valid_pi_settings(config.fuel) || !valid_pi_settings(config.nozzle) ||
        !positive(config.nozzle.minimum) || !positive(config.exit_to_throat_ratio) ||
        config.exit_to_throat_ratio < 1.0 ||
        !std::isfinite(config.exit_to_throat_ratio * config.nozzle.maximum)) {
        throw std::domain_error("invalid controller gains or actuator limits");
    }
}

struct PiUpdate {
    double command;
    double next_bias;
    CommandLimit limit;
};

PiUpdate update_pi(PiSettings pi, double bias, double error, double trim_error, double dt_seconds) {
    const double proportional = pi.proportional_gain * (error - trim_error);
    const double raw = bias + proportional;
    if (!std::isfinite(raw)) {
        throw std::domain_error("non-finite controller command");
    }
    const double command = std::clamp(raw, pi.minimum, pi.maximum);
    CommandLimit limit = CommandLimit::none;
    if (raw <= pi.minimum) {
        limit = CommandLimit::minimum;
    } else if (raw >= pi.maximum) {
        limit = CommandLimit::maximum;
    }

    // Stop integration into a limit. Permit reverse integration and stop it at the
    // other bound if a large sample interval would otherwise wind up the integral.
    double next_bias = bias;
    if (error > 0.0 && raw < pi.maximum) {
        next_bias = std::min(bias + pi.integral_gain * error * dt_seconds,
                             pi.maximum - proportional);
    } else if (error < 0.0 && raw > pi.minimum) {
        next_bias = std::max(bias + pi.integral_gain * error * dt_seconds,
                             pi.minimum - proportional);
    }
    if (!std::isfinite(next_bias)) {
        throw std::domain_error("non-finite controller integral");
    }
    return {command, next_bias, limit};
}

}  // namespace

ReferenceController::ReferenceController(ControllerConfig config, double trim_throttle,
                                         ControllerFeedback trim_feedback,
                                         ControllerCommands trim_commands)
    : config_(std::move(config)),
      fuel_bias_(trim_commands.main_fuel_flow),
      nozzle_bias_(trim_commands.throat_area) {
    validate_config(config_);
    const auto targets = targets_at(trim_throttle);
    if (!valid_feedback(trim_feedback) || !within(trim_commands.main_fuel_flow, config_.fuel) ||
        !within(trim_commands.throat_area, config_.nozzle) ||
        !agree_at_trim(trim_commands.augmentor_fuel_flow, targets.augmentor_fuel_flow) ||
        !agree_at_trim(trim_commands.exit_area,
                       trim_commands.throat_area * config_.exit_to_throat_ratio)) {
        throw std::domain_error("trim commands or feedback are inconsistent with the controller");
    }
    trim_fuel_error_ = targets.lp_speed_rpm - trim_feedback.lp_speed_rpm;
    trim_nozzle_error_ = trim_feedback.pressure_ratio - targets.pressure_ratio;
}

ControllerSchedulePoint ReferenceController::targets_at(double throttle) const {
    if (!nonnegative(throttle) || throttle > 1.0) {
        throw std::domain_error("controller throttle must be in [0, 1]");
    }
    const auto upper = std::lower_bound(
        config_.schedule.begin(), config_.schedule.end(), throttle,
        [](const ControllerSchedulePoint& point, double value) { return point.throttle < value; });
    if (upper->throttle == throttle) {
        return *upper;
    }
    const auto& lower = *(upper - 1);
    const double fraction = (throttle - lower.throttle) / (upper->throttle - lower.throttle);
    return {throttle, std::lerp(lower.lp_speed_rpm, upper->lp_speed_rpm, fraction),
            std::lerp(lower.pressure_ratio, upper->pressure_ratio, fraction),
            std::lerp(lower.augmentor_fuel_flow, upper->augmentor_fuel_flow, fraction)};
}

ControllerSample ReferenceController::step(double throttle, ControllerFeedback feedback,
                                            double dt_seconds) {
    if (!valid_feedback(feedback) || !positive(dt_seconds)) {
        throw std::domain_error("invalid controller feedback or sample interval");
    }

    const auto targets = targets_at(throttle);
    const auto fuel = update_pi(config_.fuel, fuel_bias_, targets.lp_speed_rpm - feedback.lp_speed_rpm,
                                trim_fuel_error_, dt_seconds);
    const auto nozzle = update_pi(config_.nozzle, nozzle_bias_,
                                  feedback.pressure_ratio - targets.pressure_ratio,
                                  trim_nozzle_error_, dt_seconds);
    fuel_bias_ = fuel.next_bias;
    nozzle_bias_ = nozzle.next_bias;

    return {{fuel.command, targets.augmentor_fuel_flow, nozzle.command,
             nozzle.command * config_.exit_to_throat_ratio},
            targets, fuel.limit, nozzle.limit};
}

}  // namespace turbofan
