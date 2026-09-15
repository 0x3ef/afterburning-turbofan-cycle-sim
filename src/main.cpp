#include "turbofan/cycle.hpp"

#include <array>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    const std::string_view mode = argc == 2 ? argv[1] : "";
    if (mode != "design" && mode != "off-design") {
        std::cerr << "Usage: turbofan-cli design|off-design\n";
        return 2;
    }

    try {
        const auto point = mode == "design" ? turbofan::synthetic_design_case()
                                            : turbofan::synthetic_off_design_case();
        const auto result = turbofan::solve_synthetic_dry(point);
        const std::array residual_names{
            "fan_flow", "compressor_flow", "hp_turbine_flow", "lp_turbine_flow",
            "mixer_pressure", "nozzle_capacity", "hp_shaft_power", "lp_shaft_power",
        };
        std::cerr << "model=synthetic-mixed-flow-dry case=" << mode
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

        std::cout << std::setprecision(12)
                  << "model,case,station,pt_pa,tt_k,flow_kg_s,h_total_j_kg,fuel_air_ratio\n";
        for (const auto& station : result.stations) {
            std::cout << "synthetic-mixed-flow-dry," << mode << ',' << station.name << ','
                      << station.total.pressure << ',' << station.total.temperature << ','
                      << station.total.mass_flow << ','
                      << turbofan::cp * station.total.temperature << ',' << station.fuel_air_ratio << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << "Dry cycle evaluation failed: " << error.what() << '\n';
        return 1;
    }
}
