#pragma once

#include <array>
#include <cstddef>
#include <functional>

namespace turbofan::detail {

using Vector = std::array<double, 8>;
using FlowVector = std::array<double, 6>;

template <std::size_t dimension>
struct BasicMatchResult {
    std::array<double, dimension> x;
    std::array<double, dimension> residuals;
    int iterations;
    bool converged;
};

using MatchResult = BasicMatchResult<8>;
using FlowMatchResult = BasicMatchResult<6>;

// Use nondimensional variables and residuals with comparable scales.
// The initial point must be feasible. Subsequent infeasible trials are rejected.
MatchResult match(const std::function<Vector(const Vector&)>& evaluate, Vector initial);
FlowMatchResult match_flows(const std::function<FlowVector(const FlowVector&)>& evaluate,
                            FlowVector initial);

} // namespace turbofan::detail
