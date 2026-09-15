#pragma once

#include "turbofan/thermo.hpp"

#include <array>
#include <string_view>

namespace turbofan {

struct DryCase {
    double ambient_pressure;
    double ambient_temperature;
    double mach;
    double inlet_recovery;
    double fuel_flow;
    double throat_area;
};

struct Station {
    std::string_view name;
    TotalState total;
    double fuel_air_ratio;
};

struct DryResult {
    std::array<Station, 9> stations;
    double lp_speed_rpm;
    double hp_speed_rpm;
    double bypass_fraction;
    double core_mixer_pressure;
    double bypass_mixer_pressure;
    double nozzle_mass_capacity;
    double lp_net_power;
    double hp_net_power;
    std::array<double, 8> scaled_residuals;
    int iterations;
    bool converged;
};

DryCase synthetic_design_case();
DryCase synthetic_off_design_case();
DryResult solve_synthetic_dry(DryCase operating_point);

}  // namespace turbofan
