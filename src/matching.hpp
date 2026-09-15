#pragma once

#include <array>
#include <functional>

namespace turbofan::detail {

using Vector = std::array<double, 8>;

struct MatchResult {
    Vector x;
    Vector residuals;
    int iterations;
    bool converged;
};

// Use nondimensional variables and residuals with comparable scales.
// The initial point must be feasible. Subsequent infeasible trials are rejected.
MatchResult match(const std::function<Vector(const Vector&)>& evaluate, Vector initial);

} // namespace turbofan::detail
