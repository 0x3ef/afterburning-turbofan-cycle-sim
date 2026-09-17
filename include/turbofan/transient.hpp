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

// Sample zero is the steady solution at setup.point. At each subsequent step,
// command is sampled once at the left endpoint with the last accepted result.
// The returned command is held through that step and recorded at the right endpoint.
std::vector<TransientSample> simulate_transient(
    const CycleSetup& setup, const MapDataset& maps, double step_seconds, std::size_t steps,
    const std::function<WetCase(double, const CycleResult&)>& command);

// Run the sampled controller and actuator model with the supplied profile.
std::vector<TransientSample> simulate_controlled(
    const CycleSetup& setup, const MapDataset& maps, const ControlProfile& profile);

}  // namespace turbofan
