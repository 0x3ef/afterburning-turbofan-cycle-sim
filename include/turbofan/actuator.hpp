#pragma once

#include "turbofan/controller.hpp"

namespace turbofan {

struct ActuatorResponse {
    double time_constant_seconds;
    // Command units per second. A zero rate holds the channel stationary.
    double rate_limit_per_second;
};

struct ActuatorConfig {
    ActuatorResponse main_fuel;
    ActuatorResponse augmentor_fuel;
    ActuatorResponse throat_area;
    double exit_to_throat_ratio;
};

// Advance synthetic actuators over a positive sample interval with a held target.
// Each channel takes its exact first-order increment, capped by its rate limit.
// Both endpoint commands must use the configured exit-to-throat area ratio.
ControllerCommands advance_actuators(ControllerCommands current, ControllerCommands target,
                                     const ActuatorConfig& config, double dt_seconds);

}  // namespace turbofan
