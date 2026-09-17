#pragma once

#include <cstddef>
#include <iosfwd>
#include <vector>

#include "turbofan/actuator.hpp"
#include "turbofan/controller.hpp"

namespace turbofan {

struct ThrottlePoint {
    double time;  // seconds
    double throttle;
};

struct ControlProfile {
    ControllerConfig controller;
    ActuatorConfig actuators;
    double trim_throttle;
    double step_seconds;
    std::size_t steps;
    std::vector<ThrottlePoint> demand;
};

// Validate a file-backed or programmatic profile. Invalid inputs throw
// std::invalid_argument.
void validate_control_profile(const ControlProfile& profile);
ControlProfile load_control_profile(std::istream& input);
void write_control_profile(std::ostream& output, const ControlProfile& profile);

// Look up demand in a valid profile over its time horizon, including both endpoints.
// A new value takes effect at its stated time and holds until the next point.
double throttle_at(const ControlProfile& profile, double time);

}  // namespace turbofan
