#include "turbofan/transient.hpp"

#include "turbofan/control_profile.hpp"

#include <chrono>
#include <cmath>
#include <numbers>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace turbofan {
namespace {

struct Speeds {
    double lp;
    double hp;
};

using Clock = std::chrono::steady_clock;

void reset_timing(TransientTiming* timing) {
    if (timing) {
        timing->initialization_seconds = 0.0;
        timing->step_seconds.clear();
    }
}

double elapsed_seconds(Clock::time_point started) {
    return std::chrono::duration<double>(Clock::now() - started).count();
}

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
    const std::function<WetCase(double, const CycleResult&)>& command, TransientTiming* timing) {
    reset_timing(timing);
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
    if (timing) {
        timing->step_seconds.reserve(steps);
    }
    const auto initialization_started = timing ? Clock::now() : Clock::time_point{};
    history.reserve(steps + 1);
    history.push_back({0.0, setup.point,
                       require_match(solve_steady(setup.point, engine, maps, setup.seed), 0.0,
                                     "initial steady")});
    if (timing) {
        timing->initialization_seconds = elapsed_seconds(initialization_started);
    }
    for (std::size_t step = 0; step < steps; ++step) {
        const auto step_started = timing ? Clock::now() : Clock::time_point{};
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
        if (timing) {
            timing->step_seconds.push_back(elapsed_seconds(step_started));
        }
    }
    return history;
}

std::vector<TransientSample> simulate_controlled(
    const CycleSetup& setup, const MapDataset& maps, const ControlProfile& profile,
    TransientTiming* timing) {
    reset_timing(timing);
    validate_control_profile(profile);
    const auto& point = setup.point;
    ControllerCommands actual{point.base.fuel_flow, point.augmentor_fuel_flow,
                              point.base.throat_area, point.exit_area};
    std::optional<ReferenceController> controller;
    return simulate_transient(setup, maps, profile.step_seconds, profile.steps,
        [&](double time, const CycleResult& result) {
            const ControllerFeedback measured{
                result.lp_speed_rpm,
                result.stations[7].total.pressure / result.stations[0].total.pressure,
            };
            if (!controller) {
                controller.emplace(profile.controller, profile.trim_throttle, measured, actual);
            }

            const auto requested = controller->step(
                throttle_at(profile, time), measured, profile.step_seconds);
            actual = advance_actuators(actual, requested.commands, profile.actuators,
                                       profile.step_seconds);
            auto applied = point;
            applied.base.fuel_flow = actual.main_fuel_flow;
            applied.augmentor_fuel_flow = actual.augmentor_fuel_flow;
            applied.base.throat_area = actual.throat_area;
            applied.exit_area = actual.exit_area;
            return applied;
        }, timing);
}

}  // namespace turbofan
