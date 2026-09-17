#pragma once

#include "turbofan/cycle.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace turbofan {

struct ControlProfile;

struct TransientSample {
    double time;  // seconds
    WetCase command;
    CycleResult result;
};

struct TransientTiming {
    double initialization_seconds{};
    std::vector<double> step_seconds;
};

// Sample zero is the steady solution at setup.point. At each subsequent step,
// command is sampled once at the left endpoint with the last accepted result.
// The returned command is held through that step and recorded at the right endpoint.
// Optional elapsed times cover the initial history allocation and steady solve,
// then the command, flow matches, and recording of each accepted step.
std::vector<TransientSample> simulate_transient(
    const CycleSetup& setup, const MapDataset& maps, double step_seconds, std::size_t steps,
    const std::function<WetCase(double, const CycleResult&)>& command,
    TransientTiming* timing = nullptr);

// Run the sampled controller and actuator model with the supplied profile.
std::vector<TransientSample> simulate_controlled(
    const CycleSetup& setup, const MapDataset& maps, const ControlProfile& profile,
    TransientTiming* timing = nullptr);

}  // namespace turbofan
