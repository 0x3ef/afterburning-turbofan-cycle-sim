#include "turbofan/control_profile.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

constexpr std::string_view fixture = R"(turbofan-control-v1
schedule,0,8000,1.8,0
throat_actuator,0.055,0.042
fuel_pi,0.0000131234567890123,0.000025,0,1.5
timing,0.025,40,0.4
demand,0,0.4
augmentor_actuator,0.08,0
schedule,0.4,9000,2.1,0
nozzle_pi,0.002,0,0.1,0.12
fuel_actuator,0.04,0.2
demand,0.125,0.75
schedule,1,9700,2.4,0.06
area_ratio,1.25
demand,1,0.15
)";

void require(bool condition, const char* explanation) {
    if (!condition) {
        throw std::runtime_error(explanation);
    }
}

turbofan::ControlProfile parse(const std::string& csv) {
    std::istringstream input(csv);
    return turbofan::load_control_profile(input);
}

std::string replace(std::string_view before, std::string_view after) {
    std::string modified(fixture);
    const auto location = modified.find(before);
    require(location != std::string::npos, "test row was missing from the fixture");
    modified.replace(location, before.size(), after);
    return modified;
}

template <typename Exception, typename Function>
void reject(Function operation, std::string_view diagnostic) {
    try {
        operation();
    } catch (const Exception& error) {
        require(std::string_view(error.what()).find(diagnostic) != std::string_view::npos,
                error.what());
        return;
    }
    throw std::runtime_error("invalid control profile was accepted");
}

void test_load_and_hold() {
    std::string crlf;
    for (const char c : fixture) {
        if (c == '\n') {
            crlf += '\r';
        }
        crlf += c;
    }
    const auto profile = parse(crlf);
    require(profile.step_seconds == 0.025 && profile.steps == 40 && profile.trim_throttle == 0.4,
            "profile timing should be loaded");
    require(profile.controller.fuel.proportional_gain == 0.0000131234567890123 &&
                profile.controller.nozzle.minimum == 0.1,
            "PI rows should populate their respective gains and bounds");
    require(profile.controller.schedule.size() == 3 &&
                profile.controller.schedule.back().augmentor_fuel_flow == 0.06,
            "schedule should retain input order and reheat flow");
    require(profile.actuators.main_fuel.time_constant_seconds == 0.04 &&
                profile.actuators.augmentor_fuel.rate_limit_per_second == 0.0 &&
                profile.actuators.throat_area.rate_limit_per_second == 0.042,
            "actuator responses should be loaded, including zero rate");
    require(profile.controller.exit_to_throat_ratio == 1.25 &&
                profile.actuators.exit_to_throat_ratio == 1.25,
            "one area ratio must set both configurations");

    using turbofan::throttle_at;
    require(throttle_at(profile, 0.0) == 0.4, "initial trim demand");
    require(throttle_at(profile, std::nextafter(0.125, 0.0)) == 0.4, "hold up to the knot");
    require(throttle_at(profile, 0.125) == 0.75, "new demand at the exact knot");
    require(throttle_at(profile, 0.75) == 0.75, "hold after the knot");
    require(throttle_at(profile, 1.0) == 0.15, "end of run is included");
    reject<std::domain_error>([&] { throttle_at(profile, 1.001); }, "within the run");
    reject<std::domain_error>([&] { throttle_at(profile, -0.01); }, "within the run");
    reject<std::domain_error>(
        [&] { throttle_at(profile, std::numeric_limits<double>::quiet_NaN()); }, "within the run");

    const auto no_final_newline = parse(std::string(fixture.substr(0, fixture.size() - 1)));
    require(no_final_newline.demand.size() == profile.demand.size(),
            "last line should not require a newline");
}

void test_invalid_input() {
    const auto bad = [](std::string_view before, std::string_view after, std::string_view message) {
        reject<std::runtime_error>([&] { parse(replace(before, after)); }, message);
    };
    bad("turbofan-control-v1", "turbofan-control-v2", "line 1");
    bad("throat_actuator,0.055,0.042", "unknown,0.055,0.042", "line 3");
    bad("area_ratio,1.25", "area_ratio,1.25,0", "fields for area_ratio");
    bad("area_ratio,1.25", "area_ratio,\"1.25\"", "unquoted");
    bad("area_ratio,1.25\n", "", "missing row area_ratio");
    bad("area_ratio,1.25", "area_ratio,1.25\narea_ratio,1.25", "duplicate row area_ratio");
    bad("timing,0.025,40,0.4", "timing,0.025,40.0,0.4", "invalid number");
    bad("timing,0.025,40,0.4", "timing,0.025,0,0.4", "positive finite time grid");
    bad("timing,0.025,40,0.4", "timing,1e308,40,0.4", "positive finite time grid");
    bad("timing,0.025,40,0.4", "timing,0.025,40,nan", "finite number");
    bad("0.0000131234567890123", "-0.0000131234567890123", "fuel_pi");
    bad("nozzle_pi,0.002,0,0.1,0.12", "nozzle_pi,0.002,0,0,0.12", "positive minimum area");
    bad("nozzle_pi,0.002,0,0.1,0.12", "nozzle_pi,0.002,0,0.12,0.1", "invalid gains or bounds");
    bad("augmentor_actuator,0.08,0", "augmentor_actuator,0,0", "invalid time constant");
    bad("fuel_actuator,0.04,0.2", "fuel_actuator,0.04,-0.2", "invalid time constant or rate");
    bad("area_ratio,1.25", "area_ratio,0.95", "area_ratio");
    bad("schedule,0,8000", "schedule,0.1,8000", "cover throttle from 0 to 1");
    bad("schedule,0.4,9000", "schedule,0,9000", "out-of-order schedule");
    bad("schedule,1,9700,2.4,0.06", "schedule,1,9700,2.4,-0.06", "invalid");
    bad("demand,0,0.4", "demand,0.001,0.4", "start at time zero");
    bad("demand,0,0.4", "demand,0,0.41", "trim throttle");
    bad("demand,0.125,0.75", "demand,0,0.75", "increasing times");
    bad("demand,1,0.15", "demand,1.1,0.15", "within the run");
    bad("demand,1,0.15", "demand,1,1.15", "[0, 1]");
}

class DecimalComma : public std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

void test_round_trip() {
    auto profile = parse(std::string(fixture));
    std::ostringstream output;
    output.imbue(std::locale(std::locale::classic(), new DecimalComma));
    output << std::fixed << std::setprecision(2) << std::setw(40);
    turbofan::write_control_profile(output, profile);
    require(output.str().starts_with("turbofan-control-v1\ntiming,"),
            "export order and formatting must be independent of output stream flags");
    const auto imported = parse(output.str());
    require(imported.controller.fuel.proportional_gain == profile.controller.fuel.proportional_gain &&
                imported.demand[1].throttle == profile.demand[1].throttle,
            "export should preserve full precision");
    std::ostringstream repeated;
    turbofan::write_control_profile(repeated, imported);
    require(output.str() == repeated.str(), "serialized profiles should be stable");

    profile.actuators.exit_to_throat_ratio = 1.0;
    reject<std::invalid_argument>([&] { turbofan::validate_control_profile(profile); }, "area_ratio");
    std::ostringstream invalid;
    reject<std::invalid_argument>([&] { turbofan::write_control_profile(invalid, profile); },
                                  "area_ratio");
    require(invalid.str().empty(), "invalid profiles should fail before writing");
}

}  // namespace

int main() {
    try {
        test_load_and_hold();
        test_invalid_input();
        test_round_trip();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
