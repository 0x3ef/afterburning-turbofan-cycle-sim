#pragma once

#include "turbofan/cycle.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace turbofan {

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

}  // namespace turbofan
