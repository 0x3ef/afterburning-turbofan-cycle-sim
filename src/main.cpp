#include "turbofan/cycle.hpp"

#include <array>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {

std::string_view regime_name(turbofan::NozzleRegime regime) {
    using enum turbofan::NozzleRegime;
    switch (regime) {
        case no_forward_flow: return "no-forward-flow";
        case subsonic: return "subsonic";
        case choked_convergent: return "choked-convergent";
        case supersonic: return "supersonic";
        case unsupported_internal_shock: return "unsupported-internal-shock";
    }
    return "unknown";
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string_view mode = argc == 2 ? argv[1] : "";
    if (mode != "design" && mode != "off-design" && mode != "wet") {
        std::cerr << "Usage: turbofan-cli design|off-design|wet\n";
        return 2;
    }

    try {
        const bool wet = mode == "wet";
        const std::string_view model = wet ? "synthetic-mixed-flow-wet" : "synthetic-mixed-flow-dry";
        const auto result = wet
            ? turbofan::solve_synthetic_wet(turbofan::synthetic_wet_case())
            : turbofan::solve_synthetic_dry(mode == "design" ? turbofan::synthetic_design_case()
                                                             : turbofan::synthetic_off_design_case());
        const std::array residual_names{
            "fan_flow", "compressor_flow", "hp_turbine_flow", "lp_turbine_flow",
            "mixer_pressure", "nozzle_capacity", "hp_shaft_power", "lp_shaft_power",
        };
        std::cerr << "model=" << model << " case=" << mode
                  << " converged=" << std::boolalpha << result.converged
                  << " iterations=" << result.iterations << '\n'
                  << "lp_speed_rpm=" << result.lp_speed_rpm
                  << " hp_speed_rpm=" << result.hp_speed_rpm
                  << " bypass_air_fraction=" << result.bypass_fraction << '\n';
        for (std::size_t i = 0; i < residual_names.size(); ++i) {
            std::cerr << residual_names[i] << '=' << result.scaled_residuals[i] << '\n';
        }
        if (!result.converged) {
            return 1;
        }

        std::cerr << "nozzle_regime=" << regime_name(result.nozzle.regime);
        if (result.nozzle.exit) {
            const auto& exit = *result.nozzle.exit;
            std::cerr << " exit_ps_pa=" << exit.pressure << " exit_ts_k=" << exit.temperature
                      << " exit_mach=" << exit.mach;
        }
        std::cerr << '\n';

        std::cout << std::setprecision(12)
                  << "model,case,station,pt_pa,tt_k,flow_kg_s,h_total_j_kg,fuel_air_ratio\n";
        for (const auto& station : result.stations) {
            std::cout << model << ',' << mode << ',' << station.name << ','
                      << station.total.pressure << ',' << station.total.temperature << ','
                      << station.total.mass_flow << ','
                      << turbofan::cp * station.total.temperature << ',' << station.fuel_air_ratio << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "Cycle evaluation failed: " << error.what() << '\n';
        return 1;
    }
}
