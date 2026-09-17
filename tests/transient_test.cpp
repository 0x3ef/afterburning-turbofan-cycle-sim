#include "turbofan/transient.hpp"

#include "turbofan/control_profile.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

using turbofan::CycleResult;
using turbofan::CycleSetup;
using turbofan::TransientTiming;
using turbofan::WetCase;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Operation>
void expect_error(Operation operation, const char* message) {
    try {
        operation();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

CycleSetup dry_setup() {
    const auto dry = turbofan::synthetic_design_case();
    return turbofan::synthetic_setup({dry, 0.0, dry.throat_area});
}

double speed_distance(const CycleResult& a, const CycleResult& b) {
    return std::hypot(a.lp_speed_rpm - b.lp_speed_rpm, a.hp_speed_rpm - b.hp_speed_rpm);
}

void require_flow_match(const CycleResult& result) {
    require(result.converged, "Accepted sample must have a converged flow match");
    require(std::abs(result.nozzle_mass_capacity - result.stations.back().total.mass_flow) < 1e-5,
            "Transient nozzle capacity must match the flow");
    require(std::abs(result.core_mixer_pressure - result.bypass_mixer_pressure) < 0.005,
            "Transient mixer pressures must match");
    require(result.next_seed.lp_speed_rpm == result.lp_speed_rpm &&
                result.next_seed.hp_speed_rpm == result.hp_speed_rpm,
            "The flow solve must return the accepted shaft speeds");
}

void require_timing(const TransientTiming& timing, std::size_t steps) {
    require(std::isfinite(timing.initialization_seconds) && timing.initialization_seconds >= 0.0,
            "Elapsed initialization time must be finite and nonnegative");
    require(timing.step_seconds.size() == steps,
            "Each accepted integration step must have exactly one elapsed time");
    for (const double duration : timing.step_seconds) {
        require(std::isfinite(duration) && duration >= 0.0,
                "Elapsed step time must be finite and nonnegative");
    }
}

void test_equilibrium() {
    const auto setup = dry_setup();
    const auto& maps = turbofan::synthetic_map_dataset();
    constexpr double step = 0.02;
    constexpr std::size_t steps = 12;
    std::size_t calls = 0;
    TransientTiming timing{-1.0, {-1.0, -1.0}};
    const auto history = turbofan::simulate_transient(setup, maps, step, steps,
        [&](double time, const CycleResult& previous) {
            require(time == static_cast<double>(calls) * step,
                    "Commands must be sampled at the left endpoint");
            require_flow_match(previous);
            ++calls;
            return setup.point;
        }, &timing);
    require_timing(timing, steps);
    require(calls == steps && history.size() == steps + 1 && history.front().time == 0.0,
            "History and command callback must match the requested grid");
    require(history.back().time == static_cast<double>(steps) * step,
            "The last sample must be at the end of the grid");
    for (const auto& sample : history) {
        require_flow_match(sample.result);
        require(speed_distance(sample.result, history.front().result) < 1e-4,
                "With constant commands an equilibrium must not drift");
    }
}

void test_disturbance_and_refinement() {
    const auto setup = dry_setup();
    const auto& maps = turbofan::synthetic_map_dataset();
    WetCase stepped = setup.point;
    stepped.base.fuel_flow *= 1.006;
    const auto command = [&](double, const CycleResult&) { return stepped; };

    const auto coarse = turbofan::simulate_transient(setup, maps, 0.06, 4, command);
    const auto refined = turbofan::simulate_transient(setup, maps, 0.03, 8, command);
    const auto reference = turbofan::simulate_transient(setup, maps, 0.0075, 32, command);
    require(coarse.front().command.base.fuel_flow == setup.point.base.fuel_flow &&
                coarse[1].command.base.fuel_flow == stepped.base.fuel_flow,
            "The initial command and sampled command must be recorded separately");
    for (const auto& sample : refined) {
        require_flow_match(sample.result);
    }
    require(speed_distance(refined.back().result, refined.front().result) > 1e-2,
            "After a fuel change, spool speeds must respond");
    require(speed_distance(refined.back().result, reference.back().result) <
                0.6 * speed_distance(coarse.back().result, reference.back().result),
            "Time step refinement must improve the solution");
}

void test_wet_control_profile() {
    const auto setup = turbofan::synthetic_setup(turbofan::synthetic_wet_case());
    const auto& maps = turbofan::synthetic_map_dataset();
    const auto trim = turbofan::solve_steady(setup.point, setup.engine, maps, setup.seed);
    require(trim.converged, "The wet fixture must have a steady trim");
    const double pressure_ratio =
        trim.stations[7].total.pressure / trim.stations.front().total.pressure;
    const double rpm = trim.lp_speed_rpm;
    const double reheat = setup.point.augmentor_fuel_flow;
    const double fuel = setup.point.base.fuel_flow;
    const double area = setup.point.base.throat_area;
    const double ratio = setup.point.exit_area / area;
    turbofan::ControlProfile profile{};
    profile.controller.schedule = {
        {0.0, rpm, pressure_ratio, reheat},
        {0.5, rpm, pressure_ratio, reheat},
        {1.0, rpm, pressure_ratio, reheat + 0.02},
    };
    profile.controller.fuel = {1e-5, 2e-5, 0.98 * fuel, 1.02 * fuel};
    profile.controller.nozzle = {0.01 * area, 0.02 * area, 0.99 * area, 1.01 * area};
    profile.controller.exit_to_throat_ratio = ratio;
    profile.actuators = {{0.04, 0.20}, {0.08, 0.10}, {0.06, 0.05}, ratio};
    profile.trim_throttle = 0.5;
    profile.step_seconds = 0.01;
    profile.steps = 20;
    profile.demand = {{0.0, 0.5}, {0.05, 0.75}};
    TransientTiming timing;
    const auto history = turbofan::simulate_controlled(setup, maps, profile, &timing);
    require_timing(timing, profile.steps);
    require(history.size() == 21, "The control profile must determine the simulation horizon");
    require(std::abs(history[5].command.augmentor_fuel_flow - reheat) < 1e-12,
            "Trim reheat must hold until demand changes");
    require(history[6].command.augmentor_fuel_flow > reheat &&
                history[6].command.augmentor_fuel_flow < reheat + 0.01,
            "Achieved reheat must ramp after the demand step");
    require(history.back().result.stations.back().total.temperature >
                history.front().result.stations.back().total.temperature,
            "The additional fuel must raise nozzle inlet total temperature");
    for (const auto& sample : history) {
        require_flow_match(sample.result);
    }
}

void test_invalid_inputs() {
    auto setup = dry_setup();
    const auto& maps = turbofan::synthetic_map_dataset();
    const auto command = [&](double, const CycleResult&) { return setup.point; };
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_transient(setup, maps, 0.0, 5, command); },
        "Zero time step must be rejected");
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_transient(setup, maps, 0.02, 0, command); },
        "Empty time grid must be rejected");
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_transient(setup, maps, 0.02, 5, {}); },
        "A command callback is required");
    setup.engine.lp_inertia = 0.0;
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_transient(setup, maps, 0.02, 5, command); },
        "Zero inertia must be rejected");
}

void test_timing_failure_preserves_completed_steps() {
    const auto setup = dry_setup();
    const auto& maps = turbofan::synthetic_map_dataset();
    TransientTiming timing{-1.0, {-1.0, -1.0}};
    std::size_t calls = 0;
    const auto failing_command = [&](double, const CycleResult&) {
        if (++calls == 2) {
            throw std::runtime_error("Test command failure");
        }
        return setup.point;
    };
    expect_error<std::runtime_error>(
        [&] { turbofan::simulate_transient(setup, maps, 0.02, 5, failing_command, &timing); },
        "A failed command must stop the simulation");
    require(calls == 2, "Only the first command should have completed");
    require_timing(timing, 1);

    const auto command = [&](double, const CycleResult&) { return setup.point; };
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_transient(setup, maps, 0.0, 5, command, &timing); },
        "An invalid time grid must be rejected");
    require(timing.initialization_seconds == 0.0 && timing.step_seconds.empty(),
            "Starting a new simulation must clear prior timing results");

    timing = {-1.0, {-1.0}};
    expect_error<std::invalid_argument>(
        [&] { turbofan::simulate_controlled(setup, maps, turbofan::ControlProfile{}, &timing); },
        "An invalid control profile must be rejected");
    require(timing.initialization_seconds == 0.0 && timing.step_seconds.empty(),
            "Starting a controlled simulation must clear prior timing results");
}

}  // namespace

int main() {
    try {
        test_equilibrium();
        test_disturbance_and_refinement();
        test_invalid_inputs();
        test_wet_control_profile();
        test_timing_failure_preserves_completed_steps();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
