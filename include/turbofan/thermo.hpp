#pragma once

namespace turbofan {

inline constexpr double gamma = 1.4;
inline constexpr double gas_constant = 287.05;  // J/(kg K)
inline constexpr double cp = gamma * gas_constant / (gamma - 1.0);

struct TotalState {
    double pressure;     // Pa
    double temperature;  // K
    double mass_flow;    // kg/s
};

struct StaticState {
    double pressure;     // Pa
    double temperature;  // K
    double mach;
};

TotalState inlet(double ambient_pressure, double ambient_temperature, double mach,
                 double recovery, double mass_flow);

// Pressure ratios are at least one: outlet/inlet for compression and inlet/outlet for expansion.
TotalState compress(TotalState in, double pressure_ratio, double efficiency);
TotalState expand(TotalState in, double pressure_ratio, double efficiency);

// Fuel enters with zero sensible enthalpy in the reference used by this model.
TotalState burn(TotalState in, double fuel_flow, double heating_value, double efficiency,
                double recovery);

// The fuel-air limit assumes complete mixing and uses the sum of both fuel flows.
// This lumped check does not resolve local oxygen availability or combustion stability.
TotalState augment(TotalState in, double air_mass_flow, double upstream_fuel_flow,
                   double augmentor_fuel_flow, double heating_value, double efficiency,
                   double recovery, double stoichiometric_fuel_air_ratio);

// The mixed total pressure is the lower inlet total pressure times recovery.
// Static pressure compatibility at the mixer entry is a separate matching constraint.
TotalState mix(TotalState core, TotalState bypass, double recovery);

// Invert the compressible mass-flow relation on the subsonic branch, including Mach 1.
StaticState subsonic_static(TotalState in, double area);

// Ideal forward nozzle capacity in kg/s, limited by choking at the throat.
double nozzle_capacity(TotalState in, double throat_area, double back_pressure);

}  // namespace turbofan
