#include "turbofan/transient.hpp"

#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace turbofan {
namespace {

struct Speeds {
    double lp;
    double hp;
};

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

double speed_rate(double rpm, double net_power, double inertia) {
    constexpr double radians_per_second_per_rpm = 2.0 * std::numbers::pi / 60.0;
    // J * omega * d(omega)/dt = net power. Convert the rate back to rpm/s.
    const double omega = rpm * radians_per_second_per_rpm;
    const double rate = net_power / (inertia * omega) / radians_per_second_per_rpm;
    if (!std::isfinite(rate)) {
        throw std::domain_error("Transient shaft speed rate must be finite");
    }
    return rate;
}

Speeds speed_rates(const CycleResult& result, const EngineConfig& engine) {
    return {speed_rate(result.lp_speed_rpm, result.lp_net_power, engine.lp_inertia),
            speed_rate(result.hp_speed_rpm, result.hp_net_power, engine.hp_inertia)};
}

CycleSeed advance_speeds(CycleSeed seed, Speeds start, Speeds rates, double step_seconds) {
    seed.lp_speed_rpm = std::fma(step_seconds, rates.lp, start.lp);
    seed.hp_speed_rpm = std::fma(step_seconds, rates.hp, start.hp);
    if (!positive(seed.lp_speed_rpm) || !positive(seed.hp_speed_rpm)) {
        throw std::domain_error("Transient step leaves the positive shaft speed regime");
    }
    return seed;
}

CycleResult require_match(CycleResult result, double time, const char* stage) {
    if (!result.converged) {
        throw std::runtime_error(std::string("Transient ") + stage + " match failed at " +
                                 std::to_string(time) + " s");
    }
    return result;
}

}  // namespace

std::vector<TransientSample> simulate_transient(
    const CycleSetup& setup, const MapDataset& maps, double step_seconds, std::size_t steps,
    const std::function<WetCase(double, const CycleResult&)>& command) {
    std::vector<TransientSample> history;
    if (!positive(step_seconds) || steps == 0 || steps >= history.max_size() ||
        !std::isfinite(static_cast<double>(steps) * step_seconds)) {
        throw std::invalid_argument("Transient requires a finite, nonempty positive time grid");
    }
    if (!positive(setup.engine.lp_inertia) || !positive(setup.engine.hp_inertia)) {
        throw std::invalid_argument("Transient shaft inertias must be finite and positive");
    }
    if (!command) {
        throw std::invalid_argument("Transient requires a command callback");
    }

    const EngineConfig& engine = setup.engine;
    history.reserve(steps + 1);
    history.push_back({0.0, setup.point,
                       require_match(solve_steady(setup.point, engine, maps, setup.seed), 0.0,
                                     "initial steady")});
    for (std::size_t step = 0; step < steps; ++step) {
        const TransientSample& previous = history.back();
        const double time = previous.time;
        const double next_time = static_cast<double>(step + 1) * step_seconds;
        if (next_time <= time) {
            throw std::invalid_argument("Transient times must be strictly increasing");
        }
        const WetCase point = command(time, previous.result);
        const Speeds start{previous.result.lp_speed_rpm, previous.result.hp_speed_rpm};

        // Heun predictor-corrector with a held command. Recompute the gas path
        // after the correction so the reported state corresponds to the accepted speeds.
        const CycleResult current = require_match(
            solve_at_speeds(point, engine, maps, previous.result.next_seed), time, "start");
        const Speeds first = speed_rates(current, engine);
        const CycleSeed prediction = advance_speeds(current.next_seed, start, first, step_seconds);
        const CycleResult predicted = require_match(
            solve_at_speeds(point, engine, maps, prediction), next_time, "predictor");
        const Speeds second = speed_rates(predicted, engine);
        const Speeds average{std::midpoint(first.lp, second.lp), std::midpoint(first.hp, second.hp)};
        const CycleSeed correction = advance_speeds(predicted.next_seed, start, average, step_seconds);
        CycleResult accepted = require_match(solve_at_speeds(point, engine, maps, correction),
                                             next_time, "accepted state");
        history.push_back({next_time, point, std::move(accepted)});
    }
    return history;
}

}  // namespace turbofan
