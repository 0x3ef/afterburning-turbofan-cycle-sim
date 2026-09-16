#include "turbofan/engine_config.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

constexpr std::string_view fixture = R"(turbofan-case-v1
variant, synthetic twin spool
mach,0
inlet_flow_kg_s,90.12345678901234
bypass_fraction,0.43
ambient_temperature_k,288.15
provenance, sample case for tests
ambient_pressure_pa,+101325.12345678901
core_mixer_area_m2,0.25
bypass_mixer_area_m2,0.30
shaft_efficiency,0.99
bypass_recovery,0.97
mixer_recovery,0.98
burner_recovery,0.96
dry_augmentor_recovery,0.98
heating_value_j_kg,4.3e7
burner_efficiency,1
augmentor_efficiency,0.95
stoichiometric_fuel_air_ratio,0.068
reheat_loss_coefficient,0
lp_inertia_kg_m2,1.1
hp_inertia_kg_m2,0.6
inlet_recovery,0.99
fuel_flow_kg_s,0
throat_area_m2,0.14
augmentor_fuel_flow_kg_s,0
exit_area_m2,0.14
fan_coordinate,1
compressor_coordinate,-0.125
hp_expansion,2.2
lp_expansion,1
lp_speed_rpm,9000
hp_speed_rpm,14000
)";

void require(bool condition, const char* explanation) {
    if (!condition) {
        throw std::runtime_error(explanation);
    }
}

turbofan::CycleSetup parse(const std::string& value) {
    std::istringstream input(value);
    return turbofan::load_cycle_setup(input);
}

std::string replace(std::string_view before, std::string_view after) {
    std::string modified(fixture);
    const auto start = modified.find(before);
    require(start != std::string::npos, "test field was missing from fixture");
    modified.replace(start, before.size(), after);
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
    throw std::runtime_error("invalid case input was accepted");
}

void test_loading() {
    std::string windows_text;
    for (const char c : fixture) {
        if (c == '\n') {
            windows_text += '\r';
        }
        windows_text += c;
    }
    const auto setup = parse(windows_text);
    require(setup.engine.variant == "synthetic twin spool" &&
                setup.engine.provenance == "sample case for tests",
            "metadata was not read and trimmed");
    require(setup.engine.heating_value == 43e6 && setup.engine.burner_efficiency == 1.0 &&
                setup.engine.reheat_loss_coefficient == 0.0,
            "engine parameters were not read");
    require(setup.point.base.ambient_pressure == 101325.12345678901 &&
                setup.point.base.mach == 0.0 && setup.point.base.fuel_flow == 0.0 &&
                setup.point.augmentor_fuel_flow == 0.0 &&
                setup.point.exit_area == setup.point.base.throat_area,
            "operating point should accept a stationary unfueled case with a convergent nozzle");
    require(setup.seed.inlet_flow == 90.12345678901234 &&
                setup.seed.compressor_coordinate == -0.125 && setup.seed.lp_expansion == 1.0 &&
                setup.seed.hp_speed_rpm == 14000.0,
            "initial guesses were not read");
    require(parse(std::string(fixture.substr(0, fixture.size() - 1))).seed.hp_speed_rpm ==
                setup.seed.hp_speed_rpm,
            "a valid last line without a final newline should load");
}

void test_input_errors() {
    const auto bad = [](std::string_view before, std::string_view after, std::string_view message) {
        reject<std::runtime_error>([&] { parse(replace(before, after)); }, message);
    };
    bad("turbofan-case-v1", "turbofan-case-v2", "line 1");
    bad("variant, synthetic twin spool", "variant,   ", "line 2");
    bad("variant, synthetic twin spool", "variant,\"synthetic twin spool\"", "line 2");
    bad("mach,0", "mach,0,0", "line 3");
    bad("mach,0", "mach,NaN", "line 3");
    bad("mach,0", "mach,+-0.1", "line 3");
    bad("mach,0", "mach,0rpm", "line 3");
    bad("mach,0", "mach,-0.1", "mach");
    bad("mach,0", "something_else,0", "unknown key something_else");
    bad("mach,0", "mach,0\nmach,0", "duplicate key mach");
    bad("mach,0\n", "", "missing key mach");
    bad("variant, synthetic twin spool\n", "", "missing key variant");
    bad("shaft_efficiency,0.99", "shaft_efficiency,1.01", "shaft_efficiency");
    bad("mixer_recovery,0.98", "mixer_recovery,0", "mixer_recovery");
    bad("bypass_fraction,0.43", "bypass_fraction,1", "bypass_fraction");
    bad("lp_inertia_kg_m2,1.1", "lp_inertia_kg_m2,0", "lp_inertia_kg_m2");
    bad("lp_expansion,1", "lp_expansion,0.99", "lp_expansion");
    bad("exit_area_m2,0.14", "exit_area_m2,0.13", "exit_area_m2");
    bad("mach,0", "mach,0\nvariant,duplicate", "duplicate key variant");
    reject<std::runtime_error>([] { parse(std::string(fixture) + "\n"); }, "line 34");
}

class DecimalComma : public std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

void test_validation_and_round_trip() {
    auto setup = parse(std::string(fixture));
    std::ostringstream output;
    output.imbue(std::locale(std::locale::classic(), new DecimalComma));
    output << std::fixed << std::setprecision(2) << std::setw(40);
    turbofan::write_cycle_setup(output, setup);
    require(output.str().starts_with("turbofan-case-v1\n"),
            "export must not use the destination stream's formatting");

    const auto reloaded = parse(output.str());
    require(setup.point.base.ambient_pressure == reloaded.point.base.ambient_pressure &&
                setup.seed.inlet_flow == reloaded.seed.inlet_flow,
            "floating-point values must round-trip exactly");
    std::ostringstream repeated;
    turbofan::write_cycle_setup(repeated, reloaded);
    require(repeated.str() == output.str(), "serialized values must be stable after reloading");

    setup.seed.fan_coordinate = std::numeric_limits<double>::infinity();
    reject<std::invalid_argument>([&] { turbofan::validate_cycle_setup(setup); },
                                  "fan_coordinate");
    setup.seed.fan_coordinate = 1.0;
    setup.engine.provenance = "field, not supported";
    std::ostringstream unchanged;
    reject<std::invalid_argument>([&] { turbofan::write_cycle_setup(unchanged, setup); },
                                  "provenance");
    require(unchanged.str().empty(), "invalid metadata must be rejected before writing");
}

}  // namespace

int main() {
    try {
        test_loading();
        test_input_errors();
        test_validation_and_round_trip();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
