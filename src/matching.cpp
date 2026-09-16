#include "matching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace turbofan::detail {
namespace {

constexpr double tolerance = 1e-8;
constexpr int max_iterations = 40;

template <std::size_t dimension>
using Values = std::array<double, dimension>;

template <std::size_t dimension>
using Matrix = std::array<Values<dimension>, dimension>;

template <std::size_t dimension>
using Evaluator = std::function<Values<dimension>(const Values<dimension>&)>;

template <std::size_t dimension>
bool finite(const Values<dimension>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

template <std::size_t dimension>
bool converged(const Values<dimension>& residuals)
{
    return std::all_of(residuals.begin(), residuals.end(), [](double residual) {
        return std::abs(residual) <= tolerance;
    });
}

template <std::size_t dimension>
double merit(const Values<dimension>& residuals)
{
    double norm = 0.0;
    for (const double residual : residuals) {
        norm = std::hypot(norm, residual);
    }
    return norm;
}

template <std::size_t dimension>
std::optional<Values<dimension>> trial_residuals(const Evaluator<dimension>& evaluate,
                                                const Values<dimension>& x)
{
    if (!finite(x)) {
        return std::nullopt;
    }

    try {
        Values<dimension> residuals = evaluate(x);
        if (finite(residuals)) {
            return residuals;
        }
    } catch (const std::domain_error&) {
        // A trial can leave the map or the supported flow regime.
    }
    return std::nullopt;
}

template <std::size_t dimension>
bool differentiate(const Evaluator<dimension>& evaluate, const BasicMatchResult<dimension>& current,
                   Matrix<dimension>& jacobian)
{
    for (std::size_t column = 0; column < dimension; ++column) {
        const double coordinate = current.x[column];
        double offset = 1e-5 * std::max(1.0, std::abs(coordinate));
        bool found = false;

        for (int attempt = 0; attempt < 5 && !found; ++attempt) {
            Values<dimension> upper_point = current.x;
            Values<dimension> lower_point = current.x;
            upper_point[column] += offset;
            lower_point[column] -= offset;

            const auto upper = trial_residuals(evaluate, upper_point);
            const auto lower = trial_residuals(evaluate, lower_point);
            if (!upper && !lower) {
                offset *= 0.25;
                continue;
            }

            const Values<dimension>& upper_value = upper ? *upper : current.residuals;
            const Values<dimension>& lower_value = lower ? *lower : current.residuals;
            const double upper_coordinate = upper ? upper_point[column] : coordinate;
            const double lower_coordinate = lower ? lower_point[column] : coordinate;

            for (std::size_t row = 0; row < dimension; ++row) {
                jacobian[row][column] = (upper_value[row] - lower_value[row])
                    / (upper_coordinate - lower_coordinate);
            }
            found = true;
        }
        if (!found) {
            return false;
        }
    }

    return std::all_of(jacobian.begin(), jacobian.end(), finite<dimension>);
}

template <std::size_t dimension>
std::optional<Values<dimension>> solve_linear(Matrix<dimension> matrix, Values<dimension> rhs)
{
    Values<dimension> row_scale{};
    for (std::size_t row = 0; row < dimension; ++row) {
        for (const double coefficient : matrix[row]) {
            row_scale[row] = std::max(row_scale[row], std::abs(coefficient));
        }
        if (row_scale[row] == 0.0) {
            return std::nullopt;
        }
    }

    // Pivot relative to each row's original scale.
    constexpr double pivot_tolerance = 64.0 * std::numeric_limits<double>::epsilon();
    for (std::size_t column = 0; column < dimension; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < dimension; ++row) {
            if (std::abs(matrix[row][column]) / row_scale[row]
                > std::abs(matrix[pivot][column]) / row_scale[pivot]) {
                pivot = row;
            }
        }
        const double pivot_ratio = std::abs(matrix[pivot][column]) / row_scale[pivot];
        if (!std::isfinite(pivot_ratio) || pivot_ratio <= pivot_tolerance) {
            return std::nullopt;
        }
        std::swap(matrix[column], matrix[pivot]);
        std::swap(rhs[column], rhs[pivot]);
        std::swap(row_scale[column], row_scale[pivot]);

        for (std::size_t row = column + 1; row < dimension; ++row) {
            const double factor = matrix[row][column] / matrix[column][column];
            for (std::size_t k = column + 1; k < dimension; ++k) {
                matrix[row][k] -= factor * matrix[column][k];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    Values<dimension> step{};
    for (std::size_t row = dimension; row-- > 0;) {
        double remainder = rhs[row];
        for (std::size_t k = row + 1; k < dimension; ++k) {
            remainder -= matrix[row][k] * step[k];
        }
        step[row] = remainder / matrix[row][row];
    }
    if (!finite(step)) {
        return std::nullopt;
    }
    return step;
}

template <std::size_t dimension>
bool take_step(const Evaluator<dimension>& evaluate, const Values<dimension>& step,
               BasicMatchResult<dimension>& current)
{
    const double old_merit = merit(current.residuals);
    double damping = 1.0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        Values<dimension> trial{};
        for (std::size_t i = 0; i < dimension; ++i) {
            trial[i] = current.x[i] + damping * step[i];
        }
        const auto residuals = trial_residuals(evaluate, trial);
        if (residuals && (converged(*residuals)
            || merit(*residuals) < old_merit * (1.0 - 1e-4 * damping))) {
            current.x = trial;
            current.residuals = *residuals;
            ++current.iterations;
            return true;
        }
        damping *= 0.5;
    }
    return false;
}

template <std::size_t dimension>
BasicMatchResult<dimension> run_match(const Evaluator<dimension>& evaluate, Values<dimension> initial)
{
    if (!finite(initial)) {
        throw std::domain_error("matching initial variables must be finite");
    }
    BasicMatchResult<dimension> result{initial, evaluate(initial), 0, false};
    if (!finite(result.residuals)) {
        throw std::domain_error("matching initial residuals must be finite");
    }

    while (!converged(result.residuals) && result.iterations < max_iterations) {
        Matrix<dimension> jacobian{};
        if (!differentiate(evaluate, result, jacobian)) {
            break;
        }

        Values<dimension> rhs = result.residuals;
        for (double& residual : rhs) {
            residual = -residual;
        }
        const auto step = solve_linear(jacobian, rhs);
        if (!step || !take_step(evaluate, *step, result)) {
            break;
        }
    }

    result.converged = converged(result.residuals);
    return result;
}

} // namespace

MatchResult match(const std::function<Vector(const Vector&)>& evaluate, Vector initial)
{
    return run_match(evaluate, initial);
}

FlowMatchResult match_flows(const std::function<FlowVector(const FlowVector&)>& evaluate,
                            FlowVector initial)
{
    return run_match(evaluate, initial);
}

} // namespace turbofan::detail
