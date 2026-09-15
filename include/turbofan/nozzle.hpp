#pragma once

#include <optional>

#include "turbofan/thermo.hpp"

namespace turbofan {

enum class NozzleRegime {
    no_forward_flow,
    subsonic,
    choked_convergent,
    supersonic,
    unsupported_internal_shock,
};

struct NozzleResult {
    double mass_flow_capacity;  // kg/s
    NozzleRegime regime;
    std::optional<StaticState> exit;
};

// Ideal quasi-one-dimensional flow with a sonic throat at choking. The exit at onset
// of choking stays on the subsonic branch when exit area exceeds throat area.
// A shock exactly at the exit is outside the modeled nozzle, which reports its
// upstream state. For an internal shock only the choked mass capacity is available.
// The incoming mass_flow is not used to impose a nozzle operating point.
NozzleResult convergent_divergent_nozzle(TotalState in, double throat_area, double exit_area,
                                          double back_pressure);

}  // namespace turbofan
