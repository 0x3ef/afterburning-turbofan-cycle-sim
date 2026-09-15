#include "turbofan/nozzle.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

using turbofan::NozzleRegime;
using turbofan::StaticState;
using turbofan::TotalState;
using turbofan::convergent_divergent_nozzle;

constexpr TotalState inlet{200000.0, 750.0, 5.0};
constexpr double throat_area = 0.05;
constexpr double mach_two_exit_area = 1.6875 * throat_area;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, double tolerance, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
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

double mass_flux(StaticState state) {
    const double density = state.pressure / (turbofan::gas_constant * state.temperature);
    const double sound_speed =
        std::sqrt(turbofan::gamma * turbofan::gas_constant * state.temperature);
    return density * sound_speed * state.mach;
}

void test_convergent() {
    const double critical_pressure = inlet.pressure * std::pow(1.2, -3.5);
    const double sonic_capacity = throat_area *
                                  mass_flux({critical_pressure, inlet.temperature / 1.2, 1.0});
    for (const double back_pressure : {inlet.pressure, 1.1 * inlet.pressure}) {
        const auto result =
            convergent_divergent_nozzle(inlet, throat_area, throat_area, back_pressure);
        require(result.regime == NozzleRegime::no_forward_flow && !result.exit,
                "no pressure drop should produce no forward-flow state");
        near(result.mass_flow_capacity, 0.0, 0.0, "no forward-flow capacity");
    }

    const auto unchoked =
        convergent_divergent_nozzle(inlet, throat_area, throat_area, 0.8 * inlet.pressure);
    require(unchoked.regime == NozzleRegime::subsonic && unchoked.exit,
            "convergent nozzle should have a subsonic exit at high back pressure");
    near(unchoked.exit->pressure, 0.8 * inlet.pressure, 1e-9, "unchoked exit pressure");
    near(unchoked.mass_flow_capacity, throat_area * mass_flux(*unchoked.exit), 1e-10,
         "unchoked exit flow continuity");

    for (const double back_pressure : {critical_pressure, 0.2 * inlet.pressure}) {
        const auto choked =
            convergent_divergent_nozzle(inlet, throat_area, throat_area, back_pressure);
        require(choked.regime == NozzleRegime::choked_convergent && choked.exit,
                "choked convergent nozzle should report a sonic exit");
        near(choked.exit->mach, 1.0, 0.0, "sonic exit Mach");
        near(choked.exit->pressure, critical_pressure, 1e-8, "sonic exit pressure");
        near(choked.mass_flow_capacity, sonic_capacity, 1e-10, "sonic flow capacity");
    }
}

void test_divergent_choking() {
    const double sonic_pressure = inlet.pressure * std::pow(1.2, -3.5);
    const double sonic_capacity = throat_area *
                                  mass_flux({sonic_pressure, inlet.temperature / 1.2, 1.0});
    const auto at_onset = turbofan::subsonic_static(
        {inlet.pressure, inlet.temperature, sonic_capacity}, mach_two_exit_area);
    require(at_onset.pressure > sonic_pressure,
            "subsonic onset exit pressure should exceed sonic throat pressure");

    const double high_back_pressure = (at_onset.pressure + inlet.pressure) / 2.0;
    const auto unchoked =
        convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area, high_back_pressure);
    require(unchoked.regime == NozzleRegime::subsonic && unchoked.exit,
            "divergent nozzle should support unchoked flow");
    near(unchoked.exit->pressure, high_back_pressure, 1e-9, "unchoked CD exit pressure");
    near(unchoked.mass_flow_capacity, mach_two_exit_area * mass_flux(*unchoked.exit), 1e-10,
         "unchoked CD capacity uses exit area");
    require(unchoked.mass_flow_capacity < sonic_capacity,
            "unchoked CD capacity should be below the choked throat capacity");

    const auto just_above_onset =
        convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area,
                                    at_onset.pressure * (1.0 + 1e-12));
    near(just_above_onset.mass_flow_capacity, sonic_capacity, 1e-9,
         "flow capacity is continuous at onset of choking");

    const auto onset =
        convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area, at_onset.pressure);
    require(onset.regime == NozzleRegime::subsonic && onset.exit,
            "exit remains subsonic at onset of choking");
    near(onset.exit->mach, at_onset.mach, 1e-12, "subsonic Mach at onset of choking");
    near(onset.mass_flow_capacity, sonic_capacity, 1e-10, "flow capacity at onset of choking");

    const double low_back_pressure = (at_onset.pressure + sonic_pressure) / 2.0;
    const auto internal =
        convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area, low_back_pressure);
    require(internal.regime == NozzleRegime::unsupported_internal_shock && !internal.exit,
            "divergence can choke the throat before sonic throat pressure is reached at the exit");
    near(internal.mass_flow_capacity, sonic_capacity, 1e-10, "internal shock capacity stays choked");
}

void test_supersonic_exit() {
    const double ideal_mach_two_temperature = inlet.temperature / 1.8;
    const double ideal_mach_two_pressure = inlet.pressure * std::pow(1.8, -3.5);
    const double shock_at_exit_pressure = 4.5 * ideal_mach_two_pressure;

    for (const double back_pressure :
         {0.5 * ideal_mach_two_pressure, ideal_mach_two_pressure, shock_at_exit_pressure}) {
        const auto result =
            convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area, back_pressure);
        require(result.regime == NozzleRegime::supersonic && result.exit,
                "supersonic exit should hold for underexpansion and overexpansion up to exit shock");
        near(result.exit->mach, 2.0, 1e-12, "area-Mach relation for Ae/At=1.6875");
        near(result.exit->temperature, ideal_mach_two_temperature, 1e-10,
             "isentropic exit temperature");
        near(result.exit->pressure, ideal_mach_two_pressure, 1e-8, "isentropic exit pressure");
        near(result.mass_flow_capacity, mach_two_exit_area * mass_flux(*result.exit), 1e-10,
             "supersonic exit mass continuity");
    }

    const auto above_exit_shock =
        convergent_divergent_nozzle(inlet, throat_area, mach_two_exit_area,
                                    shock_at_exit_pressure * (1.0 + 1e-10));
    require(above_exit_shock.regime == NozzleRegime::unsupported_internal_shock &&
                !above_exit_shock.exit,
            "back pressure above the exit shock threshold requires an internal shock");

    const auto close_to_convergent = convergent_divergent_nozzle(
        inlet, throat_area, std::nextafter(throat_area, 1.0), ideal_mach_two_pressure);
    require(close_to_convergent.regime == NozzleRegime::supersonic && close_to_convergent.exit,
            "arbitrarily small positive divergence retains the supersonic branch");
    near(close_to_convergent.exit->mach, 1.0, 1e-7, "divergent exit approaches Mach 1 as Ae/At → 1");
}

void test_invalid_inputs() {
    expect_domain_error(
        [] { convergent_divergent_nozzle(inlet, throat_area, throat_area / 2.0, 100000.0); },
        "nozzle accepted an exit area smaller than the throat");
    expect_domain_error(
        [] { convergent_divergent_nozzle(inlet, 0.0, throat_area, 100000.0); },
        "nozzle accepted a zero throat area");
    expect_domain_error(
        [] {
            convergent_divergent_nozzle(
                inlet, throat_area, throat_area, std::numeric_limits<double>::quiet_NaN());
        },
        "nozzle accepted a non-finite back pressure");
    expect_domain_error(
        [] { convergent_divergent_nozzle({inlet.pressure, 0.0, 20.0},
                                          throat_area, throat_area, inlet.pressure); },
        "nozzle accepted an invalid inlet thermodynamic state");
}

}  // namespace

int main() {
    try {
        test_convergent();
        test_divergent_choking();
        test_supersonic_exit();
        test_invalid_inputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
