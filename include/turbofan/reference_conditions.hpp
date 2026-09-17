#pragma once

namespace turbofan {

struct AmbientConditions {
    double pressure;     // Static pressure in Pa.
    double temperature;  // Static temperature in K.
};

// Dry 1976 U.S. Standard Atmosphere at geopotential altitudes of 0 to 20,000 m.
AmbientConditions standard_atmosphere(double altitude_m);

// Interpolate the assumed total-pressure recovery from TM 84903, Fig. 6, for Mach 0 to 2.
double tm84903_inlet_recovery(double mach);

}  // namespace turbofan
