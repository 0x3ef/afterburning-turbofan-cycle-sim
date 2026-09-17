#include "public_comparison.hpp"

#include "turbofan/cycle.hpp"
#include "turbofan/reference_conditions.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <string>

namespace turbofan {
namespace {

constexpr double altitude_m = 12192.0;

struct Observation {
    double mach;
    double flight_epr;
    double deck_epr;
    double flight_fuel_kg_h;
    double deck_fuel_kg_h;
};

// TM 84903 Figs. 9(a) and 14. Only unambiguous 40,000 ft flight EPR markers are used.
constexpr std::array observations{
    Observation{0.8, 3.08, 3.30, 7100, 7100},
    Observation{1.0, 3.00, 3.17, 8000, 8400},
    Observation{1.4, 2.59, 2.56, 11900, 12200},
    Observation{2.0, 1.60, 1.62, 18600, 20000},
};

struct Comparison {
    double inlet_recovery;
    double surrogate_epr_proxy;
    double surrogate_imposed_fuel;
};

Comparison compare(Observation observation, const CycleSetup& base, const CycleResult& trim,
                   const MapDataset& maps, AmbientConditions ambient) {
    auto point = base.point;
    const double recovery = tm84903_inlet_recovery(observation.mach);
    point.base.ambient_pressure = ambient.pressure;
    point.base.ambient_temperature = ambient.temperature;
    point.base.mach = observation.mach;
    point.base.inlet_recovery = recovery;
    const auto inlet_reference = trim.stations.front().total;
    const auto inlet_target = inlet(ambient.pressure, ambient.temperature, observation.mach,
                                    recovery, 1.0);
    const double temperature_scale = inlet_target.temperature / inlet_reference.temperature;
    const double speed_scale = std::sqrt(temperature_scale);
    const double flow_scale = (inlet_target.pressure / inlet_reference.pressure) / speed_scale;

    // Transfer only the synthetic trim. Matching inlet corrected flow and speed sets
    // the seed, and fuel follows the first-order air-flow and enthalpy scales.
    const double fuel_scale = flow_scale * temperature_scale;
    point.base.fuel_flow *= fuel_scale;
    point.augmentor_fuel_flow *= fuel_scale;
    auto seed = trim.next_seed;
    seed.inlet_flow *= flow_scale;
    seed.lp_speed_rpm *= speed_scale;
    seed.hp_speed_rpm *= speed_scale;
    const auto result = solve_steady(point, base.engine, maps, seed);
    if (!result.converged) {
        throw std::runtime_error("Synthetic comparison did not converge at Mach " +
                                 std::to_string(observation.mach));
    }
    return {recovery, result.stations[7].total.pressure / result.stations.front().total.pressure,
            point.base.fuel_flow + point.augmentor_fuel_flow};
}

}  // namespace

void write_tm84903_comparison(std::ostream& output) {
    const auto base = synthetic_setup(synthetic_wet_case());
    const auto& maps = synthetic_map_dataset();
    const auto trim = solve_steady(base.point, base.engine, maps, base.seed);
    if (!trim.converged) {
        throw std::runtime_error("The public comparison requires a converged synthetic trim");
    }
    const auto ambient = standard_atmosphere(altitude_m);
    std::array<Comparison, observations.size()> results{};
    for (std::size_t i = 0; i < observations.size(); ++i) {
        results[i] = compare(observations[i], base, trim, maps, ambient);
    }

    output << std::setprecision(8)
           << "model,mach,altitude_m,inlet_recovery,flight_epr,deck_epr,surrogate_epr_proxy,"
              "proxy_minus_flight_epr,flight_total_fuel_kg_s,deck_total_fuel_kg_s,"
              "surrogate_imposed_total_fuel_kg_s\n";
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const auto observation = observations[i];
        const auto result = results[i];
        output << "synthetic-similarity," << observation.mach << ',' << altitude_m << ','
               << result.inlet_recovery << ',' << observation.flight_epr << ',' << observation.deck_epr
               << ',' << result.surrogate_epr_proxy << ','
               << result.surrogate_epr_proxy - observation.flight_epr << ','
               << observation.flight_fuel_kg_h / 3600.0 << ',' << observation.deck_fuel_kg_h / 3600.0
               << ',' << result.surrogate_imposed_fuel << '\n';
    }
}

}  // namespace turbofan
