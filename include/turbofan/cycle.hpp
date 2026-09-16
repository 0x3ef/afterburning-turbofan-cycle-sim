#pragma once

#include "turbofan/engine_config.hpp"
#include "turbofan/map_dataset.hpp"
#include "turbofan/nozzle.hpp"
#include "turbofan/thermo.hpp"

#include <array>
#include <string_view>

namespace turbofan {

struct Station {
    std::string_view name;
    TotalState total;
    double fuel_air_ratio;
};

struct CycleResult {
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
    NozzleResult nozzle;
    CycleSeed next_seed;
};

using DryResult = CycleResult;

DryCase synthetic_design_case();
DryCase synthetic_off_design_case();
WetCase synthetic_wet_case();
CycleSetup synthetic_setup(WetCase operating_point);
DryResult solve_synthetic_dry(DryCase operating_point);
CycleResult solve_synthetic_wet(WetCase operating_point);

const MapDataset& synthetic_map_dataset();
// Use the supplied tables with the synthetic geometry, losses, and initial operating point.
CycleResult solve_with_synthetic_geometry(WetCase operating_point, const MapDataset& maps);

CycleResult solve_steady(const WetCase& operating_point, const EngineConfig& engine,
                         const MapDataset& maps, const CycleSeed& seed);
// Match the first six residuals while holding both seed shaft speeds fixed.
CycleResult solve_at_speeds(const WetCase& operating_point, const EngineConfig& engine,
                            const MapDataset& maps, const CycleSeed& seed);

}  // namespace turbofan
