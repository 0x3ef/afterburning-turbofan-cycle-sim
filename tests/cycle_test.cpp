#include "turbofan/cycle.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* error) {
    if (!condition) {
        throw std::runtime_error(error);
    }
}

bool close(double actual, double expected, double tolerance) {
    return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}

void verify_balances(const turbofan::DryResult& result, double fuel_flow, double reheat_fuel = 0.0) {
    require(result.converged, "The matching solve did not converge");
    for (const double residual : result.scaled_residuals) {
        require(close(residual, 0.0, 1e-8), "An individual residual exceeds tolerance");
    }

    const auto& [inlet, fan, compressor, combustor, hpt, lpt, bypass, mixer, nozzle] = result.stations;
    require(close(fan.total.mass_flow, compressor.total.mass_flow + bypass.total.mass_flow, 1e-9),
            "The split must conserve inlet air flow");
    require(close(combustor.total.mass_flow, compressor.total.mass_flow + fuel_flow, 1e-9),
            "The combustor must account for fuel mass");
    require(close(nozzle.total.mass_flow, inlet.total.mass_flow + fuel_flow + reheat_fuel, 1e-9),
            "Fuel and air must be accounted for through the nozzle");
    require(close(combustor.fuel_air_ratio * compressor.total.mass_flow, fuel_flow, 1e-12) &&
                close(mixer.fuel_air_ratio * inlet.total.mass_flow, fuel_flow, 1e-12) &&
                bypass.fuel_air_ratio == 0.0,
            "Fuel-to-air ratios must use air mass flow before and after mixing");
    require(close(mixer.total.mass_flow * mixer.total.temperature,
                  lpt.total.mass_flow * lpt.total.temperature +
                      bypass.total.mass_flow * bypass.total.temperature, 1e-8),
            "The mixer must conserve total sensible enthalpy");
    require(close(nozzle.fuel_air_ratio * inlet.total.mass_flow, fuel_flow + reheat_fuel, 1e-12),
            "The nozzle fuel-air ratio must account for both metered fuel streams");
    require(close(result.nozzle_mass_capacity, nozzle.total.mass_flow, 1e-6),
            "The nozzle must swallow the gas path flow");
    require(close(result.core_mixer_pressure, result.bypass_mixer_pressure, 0.002),
            "Mixer entry static pressures must agree");
    require(close(result.hp_net_power, 0.0, 0.2) && close(result.lp_net_power, 0.0, 0.2),
            "Both shafts must balance");
    require(compressor.total.pressure > fan.total.pressure &&
                combustor.total.temperature > compressor.total.temperature &&
                combustor.total.temperature > hpt.total.temperature &&
                hpt.total.temperature > lpt.total.temperature &&
                nozzle.total.pressure < mixer.total.pressure,
            "Station changes must follow the flow path");
}

}  // namespace

int main() {
    try {
        const auto design_case = turbofan::synthetic_design_case();
        const auto design = turbofan::solve_synthetic_dry(design_case);
        verify_balances(design, design_case.fuel_flow);
        require(close(design.stations.front().total.mass_flow, 90.0, 1e-7) &&
                    close(design.lp_speed_rpm, 9000.0, 1e-5) &&
                    close(design.hp_speed_rpm, 14000.0, 1e-5),
                "The synthetic design point must reproduce its specified anchors");

        const auto off_design_case = turbofan::synthetic_off_design_case();
        const auto off_design = turbofan::solve_synthetic_dry(off_design_case);
        verify_balances(off_design, off_design_case.fuel_flow);
        require(off_design.iterations > 0 &&
                    std::abs(off_design.hp_speed_rpm - design.hp_speed_rpm) > 1.0 &&
                    std::abs(off_design.bypass_fraction - design.bypass_fraction) > 1e-4,
                "The off-design case must match new shaft speeds and an independent split");

        const auto zero_reheat = turbofan::solve_synthetic_wet({design_case, 0.0, design_case.throat_area});
        verify_balances(zero_reheat, design_case.fuel_flow);
        require(close(zero_reheat.stations.back().total.temperature,
                      design.stations.back().total.temperature, 1e-10),
                "Zero reheat must retain the dry fixture");
        require(design.nozzle.regime == turbofan::NozzleRegime::choked_convergent &&
                    design.nozzle.exit && design.nozzle.exit->mach == 1.0,
                "The original dry fixture must use the equivalent choked convergent throat");

        const auto wet_case = turbofan::synthetic_wet_case();
        const auto wet = turbofan::solve_synthetic_wet(wet_case);
        verify_balances(wet, wet_case.base.fuel_flow, wet_case.augmentor_fuel_flow);
        const auto& mixed = wet.stations[7].total;
        const auto& reheated = wet.stations[8].total;
        require(reheated.temperature > mixed.temperature &&
                    close(reheated.mass_flow, mixed.mass_flow + wet_case.augmentor_fuel_flow, 1e-9),
                "Reheat must add fuel mass and sensible heat");
        require(close(turbofan::cp * (reheated.mass_flow * reheated.temperature -
                                     mixed.mass_flow * mixed.temperature),
                      wet_case.augmentor_fuel_flow * 43.0e6 * 0.95, 1e-6),
                "The augmentor must close the energy balance");
        require(reheated.pressure / mixed.pressure < 0.98,
                "Reheat must add pressure loss on top of the dry loss");
        require(wet.nozzle.regime == turbofan::NozzleRegime::supersonic &&
                    wet.nozzle.exit && wet.nozzle.exit->mach > 1.0,
                "The wet fixture must have a supported supersonic exit");

        auto invalid = design_case;
        invalid.throat_area = -1.0;
        bool rejected = false;
        try {
            static_cast<void>(turbofan::solve_synthetic_dry(invalid));
        } catch (const std::domain_error&) {
            rejected = true;
        }
        require(rejected, "Invalid nozzle geometry was accepted");
        auto overfueled = wet_case;
        overfueled.augmentor_fuel_flow = 10.0;
        rejected = false;
        try {
            static_cast<void>(turbofan::solve_synthetic_wet(overfueled));
        } catch (const std::domain_error&) {
            rejected = true;
        }
        require(rejected, "An overfueled augmentor case was accepted");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
