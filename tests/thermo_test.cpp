#include "turbofan/thermo.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << "Expected " << expected << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
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

void test_inlet() {
    const auto still_air = turbofan::inlet(101325.0, 288.15, 0.0, 0.98, 20.0);
    near(still_air.pressure, 99298.5, 1e-8, "stationary inlet pressure recovery");
    near(still_air.temperature, 288.15, 1e-10, "stationary inlet temperature");
    near(still_air.mass_flow, 20.0, 1e-10, "stationary inlet mass flow");

    const auto flight = turbofan::inlet(100000.0, 250.0, 1.0, 1.0, 20.0);
    near(flight.pressure, 100000.0 * std::pow(1.2, 3.5), 1e-8, "flight inlet pressure");
    near(flight.temperature, 300.0, 1e-10, "flight inlet temperature");
}

void test_turbomachinery() {
    const turbofan::TotalState in{100000.0, 300.0, 6.0};
    const double efficiency = 0.8;
    const auto compressed = turbofan::compress(in, 4.0, efficiency);
    const double isentropic_out = 300.0 * std::pow(4.0, 2.0 / 7.0);
    near(compressed.pressure, 400000.0, 1e-8, "compressor pressure ratio");
    near(compressed.temperature, 300.0 + (isentropic_out - 300.0) / efficiency,
         1e-10, "compressor temperature with loss");
    near(compressed.mass_flow, in.mass_flow, 1e-10, "compressor mass conservation");

    const auto expanded = turbofan::expand(compressed, 4.0, efficiency);
    near(expanded.pressure, in.pressure, 1e-8, "turbine expansion ratio");
    near(expanded.temperature,
         compressed.temperature * (1.0 - efficiency + efficiency * std::pow(4.0, -2.0 / 7.0)),
         1e-10, "turbine temperature with loss");
    near(expanded.mass_flow, in.mass_flow, 1e-10, "turbine mass conservation");

    const auto ideal = turbofan::expand(turbofan::compress(in, 4.0, 1.0), 4.0, 1.0);
    near(ideal.temperature, in.temperature, 1e-10, "ideal compression and expansion round trip");
}

void test_heat_and_mixing() {
    const turbofan::TotalState core{200000.0, 500.0, 10.0};
    const turbofan::TotalState bypass{190000.0, 300.0, 4.0};
    const double fuel = 0.1;
    const double lhv = 43e6;
    const double efficiency = 0.95;
    const auto hot = turbofan::burn(core, fuel, lhv, efficiency, 0.98);
    near(hot.pressure, 196000.0, 1e-8, "combustor pressure recovery");
    near(hot.mass_flow, core.mass_flow + fuel, 1e-10, "combustor fuel mass addition");
    near(hot.mass_flow * turbofan::cp * hot.temperature,
         core.mass_flow * turbofan::cp * core.temperature + fuel * lhv * efficiency,
         1e-8, "combustor energy balance");

    const auto dry = turbofan::burn(core, 0.0, lhv, efficiency, 0.98);
    near(dry.temperature, core.temperature, 1e-10, "dry combustor temperature");
    near(dry.pressure, hot.pressure, 1e-8, "dry combustor pressure recovery");
    near(dry.mass_flow, core.mass_flow, 1e-10, "dry combustor mass conservation");

    const auto mixed = turbofan::mix(hot, bypass, 0.99);
    near(mixed.pressure, 188100.0, 1e-8, "mixer total pressure closure");
    near(mixed.mass_flow, hot.mass_flow + bypass.mass_flow, 1e-10, "mixed mass flow");
    near(mixed.mass_flow * turbofan::cp * mixed.temperature,
         turbofan::cp * (hot.mass_flow * hot.temperature + bypass.mass_flow * bypass.temperature),
         1e-8, "mixer energy balance");
}

void test_static_and_nozzle() {
    const turbofan::TotalState upstream{200000.0, 600.0, 5.0};
    const double area = 0.06;
    const auto state = turbofan::subsonic_static(upstream, area);
    if (state.mach <= 0.0 || state.mach >= 1.0) {
        throw std::runtime_error("area inversion did not select the subsonic branch");
    }
    const double density = state.pressure / (turbofan::gas_constant * state.temperature);
    const double sound_speed =
        std::sqrt(turbofan::gamma * turbofan::gas_constant * state.temperature);
    near(density * state.mach * sound_speed * area, upstream.mass_flow, 1e-10,
         "subsonic area inversion conserves mass");
    near(turbofan::nozzle_capacity(upstream, area, state.pressure), upstream.mass_flow, 1e-10,
         "unchoked flow matches the area inversion");

    const double sonic_pressure = upstream.pressure / std::pow(1.2, 3.5);
    const double sonic_temperature = upstream.temperature / 1.2;
    const double sonic_mass_flow = area * sonic_pressure *
                                   std::sqrt(turbofan::gamma /
                                             (turbofan::gas_constant * sonic_temperature));
    near(turbofan::nozzle_capacity(upstream, area, sonic_pressure), sonic_mass_flow, 1e-10,
         "critical nozzle flow");
    near(turbofan::nozzle_capacity(upstream, area, 50000.0), sonic_mass_flow, 1e-10,
         "choked nozzle flow is independent of back pressure");
    const auto sonic = turbofan::subsonic_static(
        {upstream.pressure, upstream.temperature, sonic_mass_flow}, area);
    near(sonic.mach, 1.0, 1e-10, "subsonic area inversion includes the sonic limit");
    near(sonic.pressure, sonic_pressure, 1e-8, "sonic static pressure");
    near(turbofan::nozzle_capacity(upstream, area, upstream.pressure), 0.0, 0.0,
         "equal total and back pressures give zero forward flow");
}

void test_invalid_inputs() {
    const turbofan::TotalState in{100000.0, 300.0, 6.0};
    expect_domain_error([&] { turbofan::inlet(0.0, 300.0, 0.0, 1.0, 1.0); },
                        "inlet accepted zero pressure");
    expect_domain_error([&] { turbofan::compress(in, 0.5, 1.0); },
                        "compressor accepted a pressure ratio below one");
    expect_domain_error([&] { turbofan::expand(in, 2.0, 1.01); },
                        "turbine accepted an efficiency greater than one");
    expect_domain_error([&] { turbofan::burn(in, -0.1, 43e6, 1.0, 1.0); },
                        "combustor accepted negative fuel flow");
    expect_domain_error([&] { turbofan::mix(in, {100000.0, 300.0, 0.0}, 1.0); },
                        "mixer accepted a zero bypass flow");
    expect_domain_error([&] { turbofan::subsonic_static(in, 0.001); },
                        "static conversion accepted flow above the sonic capacity");
    expect_domain_error(
        [&] { turbofan::nozzle_capacity(in, 0.1, std::numeric_limits<double>::quiet_NaN()); },
        "nozzle accepted a non-finite back pressure");
}

}  // namespace

int main() {
    try {
        test_inlet();
        test_turbomachinery();
        test_heat_and_mixing();
        test_static_and_nozzle();
        test_invalid_inputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
