#include "turbofan/actuator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

using turbofan::ActuatorConfig;
using turbofan::ControllerCommands;
using turbofan::advance_actuators;

constexpr ActuatorConfig config{{2.0, 4.0}, {1.0, 2.0}, {0.5, 2.0}, 1.5};
constexpr ControllerCommands trim{1.0, 0.8, 0.2, 0.3};
constexpr ControllerCommands target{3.0, 0.0, 0.25, 0.375};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= 1e-14, message);
}

template <typename Operation>
void expect_domain_error(Operation operation, const char* message) {
    try {
        operation();
    } catch (const std::domain_error&) {
        return;
    }
    throw std::runtime_error(message);
}

void test_trim_and_lag() {
    const auto equal = advance_actuators(trim, trim, config, 1.0);
    near(equal.main_fuel_flow, trim.main_fuel_flow, "main fuel trim");
    near(equal.augmentor_fuel_flow, trim.augmentor_fuel_flow, "augmentor fuel trim");
    near(equal.throat_area, trim.throat_area, "throat area trim");
    near(equal.exit_area, trim.exit_area, "exit area trim");

    const auto lagged = advance_actuators(trim, target, config, 1.0);
    near(lagged.main_fuel_flow, 3.0 - 2.0 * std::exp(-0.5), "first-order main fuel");
    near(lagged.augmentor_fuel_flow, 0.8 * std::exp(-1.0), "first-order augmentor fuel");
    near(lagged.throat_area, 0.25 - 0.05 * std::exp(-2.0), "first-order throat area");
    near(lagged.exit_area, lagged.throat_area * 1.5, "exit follows the throat");

    const auto half = advance_actuators(trim, target, config, 0.5);
    const auto twice = advance_actuators(half, target, config, 0.5);
    near(twice.main_fuel_flow, lagged.main_fuel_flow, "unlimited fixed-target sample invariance");
    near(twice.augmentor_fuel_flow, lagged.augmentor_fuel_flow, "augmentor sample invariance");
    near(twice.throat_area, lagged.throat_area, "throat area sample invariance");
}

void test_rate_limit() {
    constexpr ActuatorConfig slow{{0.1, 0.125}, {0.1, 0.25}, {0.1, 0.025}, 1.5};
    const auto limited = advance_actuators(trim, target, slow, 0.5);
    near(limited.main_fuel_flow, 1.0625, "positive main fuel rate limited");
    near(limited.augmentor_fuel_flow, 0.675, "negative augmentor rate limited");
    near(limited.throat_area, 0.2125, "positive area rate limited");
    near(limited.exit_area, 0.31875, "exit area follows rate-limited throat");

    const auto settled = advance_actuators(limited, target, slow, 1000.0);
    near(settled.main_fuel_flow, target.main_fuel_flow, "large sample does not overshoot main fuel");
    near(settled.augmentor_fuel_flow, target.augmentor_fuel_flow, "large sample does not overshoot augmentor");
    near(settled.throat_area, target.throat_area, "large sample does not overshoot throat");

    ActuatorConfig locked = slow;
    locked.augmentor_fuel.rate_limit_per_second = 0.0;
    near(advance_actuators(trim, target, locked, 10.0).augmentor_fuel_flow, trim.augmentor_fuel_flow,
         "zero slew rate holds the augmentor actuator");
}

void test_short_sample() {
    constexpr ControllerCommands dry{1.0, 0.0, 0.2, 0.3};
    constexpr double dt = 1e-20;
    const auto advanced = advance_actuators(dry, trim, config, dt);
    require(advanced.augmentor_fuel_flow > 0.0 &&
                std::abs(advanced.augmentor_fuel_flow / dt - 0.8) < 1e-15,
            "short sample must retain the first-order increment");
}

void test_invalid_inputs() {
    expect_domain_error([&] { advance_actuators(trim, target, config, 0.0); },
                        "nonpositive sample interval accepted");
    expect_domain_error(
        [&] {
            advance_actuators(trim, target, config, std::numeric_limits<double>::infinity());
        },
        "non-finite sample interval accepted");

    auto settings = config;
    settings.main_fuel.time_constant_seconds = 0.0;
    expect_domain_error([&] { advance_actuators(trim, target, settings, 0.5); },
                        "nonpositive time constant accepted");
    settings = config;
    settings.augmentor_fuel.rate_limit_per_second = -1.0;
    expect_domain_error([&] { advance_actuators(trim, target, settings, 0.5); },
                        "negative actuator rate accepted");
    settings = config;
    settings.exit_to_throat_ratio = 0.99;
    expect_domain_error([&] { advance_actuators(trim, target, settings, 0.5); },
                        "exit-to-throat ratio below one accepted");

    auto commands = trim;
    commands.exit_area = 0.299;
    expect_domain_error([&] { advance_actuators(commands, target, config, 0.5); },
                        "current area ratio mismatch accepted");
    commands = target;
    commands.exit_area = 0.4;
    expect_domain_error([&] { advance_actuators(trim, commands, config, 0.5); },
                        "target area ratio mismatch accepted");
    commands = target;
    commands.throat_area = 0.0;
    expect_domain_error([&] { advance_actuators(trim, commands, config, 0.5); },
                        "zero throat area accepted");
    commands = target;
    commands.augmentor_fuel_flow = -0.01;
    expect_domain_error([&] { advance_actuators(trim, commands, config, 0.5); },
                        "negative target fuel accepted");
    commands = target;
    commands.main_fuel_flow = std::numeric_limits<double>::quiet_NaN();
    expect_domain_error([&] { advance_actuators(trim, commands, config, 0.5); },
                        "non-finite target fuel accepted");
}

}  // namespace

int main() {
    try {
        test_trim_and_lag();
        test_rate_limit();
        test_short_sample();
        test_invalid_inputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
