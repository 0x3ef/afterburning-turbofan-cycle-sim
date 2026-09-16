#include "turbofan/transient.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

using turbofan::CycleResult;
using turbofan::CycleSetup;
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

void test_equilibrium() {
    const auto setup = dry_setup();
    const auto& maps = turbofan::synthetic_map_dataset();
    constexpr double step = 0.02;
    constexpr std::size_t steps = 12;
    std::size_t calls = 0;
    const auto history = turbofan::simulate_transient(setup, maps, step, steps,
        [&](double time, const CycleResult& previous) {
            require(time == static_cast<double>(calls) * step,
                    "Commands must be sampled at the left endpoint");
            require_flow_match(previous);
            ++calls;
            return setup.point;
        });
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

}  // namespace

int main() {
    try {
        test_equilibrium();
        test_disturbance_and_refinement();
        test_invalid_inputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
