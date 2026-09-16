#include "turbofan/controller.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

using turbofan::CommandLimit;
using turbofan::ControllerCommands;
using turbofan::ControllerConfig;
using turbofan::ControllerFeedback;
using turbofan::ReferenceController;

constexpr double trim_throttle = 0.5;
constexpr ControllerFeedback trim_feedback{6000.0, 3.0};
constexpr ControllerCommands trim_commands{1.5, 0.0, 0.08, 0.12};

ControllerConfig config() {
    return {{{0.0, 3000.0, 2.0, 0.0}, {0.5, 6000.0, 3.0, 0.0}, {1.0, 9000.0, 4.0, 1.2}},
            {0.0002, 0.001, 0.5, 2.5},
            {0.02, 0.01, 0.04, 0.12},
            1.5};
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= 1.0e-12, message);
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

void test_trim_and_schedule() {
    ReferenceController controller(config(), trim_throttle, trim_feedback, trim_commands);
    const auto trim = controller.step(trim_throttle, trim_feedback, 0.1);
    near(trim.commands.main_fuel_flow, trim_commands.main_fuel_flow, "trim main fuel");
    near(trim.commands.augmentor_fuel_flow, trim_commands.augmentor_fuel_flow, "trim augmentor fuel");
    near(trim.commands.throat_area, trim_commands.throat_area, "trim throat area");
    near(trim.commands.exit_area, trim_commands.exit_area, "trim exit area");
    require(trim.fuel_limit == CommandLimit::none && trim.nozzle_limit == CommandLimit::none,
            "trim should be within actuator bounds");

    const auto step = controller.step(0.75, trim_feedback, 0.1);
    near(step.targets.lp_speed_rpm, 7500.0, "interpolated speed target");
    near(step.targets.pressure_ratio, 3.5, "interpolated pressure ratio target");
    near(step.commands.augmentor_fuel_flow, 0.6, "interpolated augmentor fuel");
    near(step.commands.main_fuel_flow, 1.8, "proportional main fuel increase");
    near(step.commands.throat_area, 0.07, "proportional throat area decrease");
    near(step.commands.exit_area, 0.105, "exit area follows the throat");

    const auto followup = controller.step(0.75, trim_feedback, 0.1);
    near(followup.commands.main_fuel_flow, 1.95, "sampled fuel integral");
    near(followup.commands.throat_area, 0.0695, "sampled area integral");
    near(controller.step(1.0, trim_feedback, 0.1).commands.augmentor_fuel_flow, 1.2,
         "full-throttle augmentor schedule endpoint");
    near(controller.step(0.0, trim_feedback, 0.1).commands.augmentor_fuel_flow, 0.0,
         "zero-throttle augmentor schedule endpoint");
}

void test_bumpless_trim_with_reference_offset() {
    const ControllerFeedback offset_feedback{5900.0, 2.9};
    ReferenceController controller(config(), trim_throttle, offset_feedback, trim_commands);
    const auto first = controller.step(trim_throttle, offset_feedback, 0.5);
    near(first.commands.main_fuel_flow, trim_commands.main_fuel_flow,
         "trim speed error should not produce a proportional jump");
    near(first.commands.throat_area, trim_commands.throat_area,
         "trim pressure error should not produce a proportional jump");

    const auto second = controller.step(trim_throttle, offset_feedback, 0.5);
    near(second.commands.main_fuel_flow, 1.55, "integrator responds to residual speed error");
    near(second.commands.throat_area, 0.0795, "integrator responds to residual ratio error");
}

void test_anti_windup() {
    auto settings = config();
    settings.fuel.proportional_gain = 0.0;
    settings.nozzle.proportional_gain = 0.0;
    ReferenceController controller(settings, trim_throttle, trim_feedback, trim_commands);
    const ControllerFeedback high_command{1000.0, 5.0};
    const ControllerFeedback low_command{6100.0, 2.9};

    controller.step(trim_throttle, high_command, 100.0);
    for (int i = 0; i < 10; ++i) {
        const auto upper = controller.step(trim_throttle, high_command, 100.0);
        require(upper.fuel_limit == CommandLimit::maximum &&
                    upper.nozzle_limit == CommandLimit::maximum,
                "upper actuator limits should be reported");
        near(upper.commands.main_fuel_flow, settings.fuel.maximum, "main fuel upper bound");
        near(upper.commands.throat_area, settings.nozzle.maximum, "area upper bound");
    }

    controller.step(trim_throttle, low_command, 1.0);
    const auto release = controller.step(trim_throttle, low_command, 1.0);
    require(release.fuel_limit == CommandLimit::none && release.nozzle_limit == CommandLimit::none,
            "reverse integration should immediately leave upper bounds on the next sample");
    near(release.commands.main_fuel_flow, 2.4, "main fuel releases without windup");
    near(release.commands.throat_area, 0.119, "area releases without windup");

    controller.step(trim_throttle, low_command, 100.0);
    const auto lower = controller.step(trim_throttle, low_command, 100.0);
    require(lower.fuel_limit == CommandLimit::minimum && lower.nozzle_limit == CommandLimit::minimum,
            "lower actuator limits should be reported");
    near(lower.commands.main_fuel_flow, settings.fuel.minimum, "main fuel lower bound");
    near(lower.commands.throat_area, settings.nozzle.minimum, "area lower bound");
}

void test_invalid_inputs() {
    auto bad_schedule = config();
    bad_schedule.schedule[1].throttle = 0.0;
    expect_domain_error(
        [&] {
            ReferenceController controller(bad_schedule, trim_throttle, trim_feedback, trim_commands);
        },
        "schedule accepted duplicate throttle knots");

    auto bad_ratio = config();
    bad_ratio.exit_to_throat_ratio = 0.9;
    expect_domain_error(
        [&] {
            ReferenceController controller(bad_ratio, trim_throttle, trim_feedback, trim_commands);
        },
        "controller accepted an exit area smaller than its throat");
    expect_domain_error(
        [] {
            constexpr ControllerCommands inconsistent_trim{1.5, 0.1, 0.08, 0.12};
            ReferenceController controller(config(), trim_throttle, trim_feedback, inconsistent_trim);
        },
        "controller accepted a trim inconsistent with its augmentor schedule");

    ReferenceController controller(config(), trim_throttle, trim_feedback, trim_commands);
    expect_domain_error([&] { controller.step(1.01, trim_feedback, 0.1); },
                        "controller extrapolated above full throttle");
    expect_domain_error([&] { controller.step(trim_throttle, trim_feedback, 0.0); },
                        "controller accepted a nonpositive sample interval");
    expect_domain_error(
        [&] {
            controller.step(trim_throttle,
                            {std::numeric_limits<double>::quiet_NaN(), 3.0}, 0.1);
        },
        "controller accepted non-finite feedback");
    near(controller.step(trim_throttle, trim_feedback, 0.1).commands.main_fuel_flow, 1.5,
         "invalid samples must leave the controller state unchanged");
}

}  // namespace

int main() {
    try {
        test_trim_and_schedule();
        test_bumpless_trim_with_reference_offset();
        test_anti_windup();
        test_invalid_inputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
