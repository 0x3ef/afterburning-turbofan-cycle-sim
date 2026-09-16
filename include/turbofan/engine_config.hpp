#pragma once

#include <iosfwd>
#include <string>

namespace turbofan {

struct DryCase {
    double ambient_pressure;
    double ambient_temperature;
    double mach;
    double inlet_recovery;
    double fuel_flow;
    double throat_area;
};

struct WetCase {
    DryCase base;
    double augmentor_fuel_flow;
    double exit_area;
};

struct EngineConfig {
    std::string variant;
    std::string provenance;
    double core_mixer_area;
    double bypass_mixer_area;
    double shaft_efficiency;
    double bypass_recovery;
    double mixer_recovery;
    double burner_recovery;
    double dry_augmentor_recovery;
    double heating_value;
    double burner_efficiency;
    double augmentor_efficiency;
    double stoichiometric_fuel_air_ratio;
    double reheat_loss_coefficient;
    double lp_inertia;
    double hp_inertia;
};

struct CycleSeed {
    double inlet_flow;
    double bypass_fraction;
    double fan_coordinate;
    double compressor_coordinate;
    double hp_expansion;
    double lp_expansion;
    double lp_speed_rpm;
    double hp_speed_rpm;
};

struct CycleSetup {
    EngineConfig engine;
    WetCase point;
    CycleSeed seed;
};

// Validate input domains and nozzle geometry. Conditions for a solution also
// depend on the component maps and the gas-path balances.
void validate_cycle_setup(const CycleSetup& setup);

// Round-trip a complete case in the unquoted turbofan-case-v1 format.
CycleSetup load_cycle_setup(std::istream& input);
void write_cycle_setup(std::ostream& output, const CycleSetup& setup);

}  // namespace turbofan
