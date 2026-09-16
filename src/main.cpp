#include "turbofan/cycle.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
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
    const std::string_view mode = argc >= 2 ? argv[1] : "";
    const bool export_maps = mode == "export-maps" && argc == 2;
    const bool use_external_maps = argc == 4 && std::string_view(argv[2]) == "--maps";
    const bool run_cycle = (mode == "design" || mode == "off-design" || mode == "wet") &&
                           (argc == 2 || use_external_maps);
    if (!run_cycle && !export_maps) {
        std::cerr << "Usage: turbofan-cli design|off-design|wet [--maps FILE]\n"
                     "       turbofan-cli export-maps\n";
        return 2;
    }

    try {
        if (export_maps) {
            turbofan::write_map_dataset(std::cout, turbofan::synthetic_map_dataset());
            return 0;
        }
        const bool wet = mode == "wet";
        const std::string_view model = use_external_maps ? "external-maps-synthetic-geometry"
            : wet ? "synthetic-mixed-flow-wet" : "synthetic-mixed-flow-dry";
        const auto point = [&] {
            if (wet) {
                return turbofan::synthetic_wet_case();
            }
            const auto dry = mode == "design" ? turbofan::synthetic_design_case()
                                               : turbofan::synthetic_off_design_case();
            return turbofan::WetCase{dry, 0.0, dry.throat_area};
        }();
        const auto result = [&] {
            if (!use_external_maps) {
                return turbofan::solve_synthetic_wet(point);
            }
            std::ifstream file(argv[3]);
            if (!file) {
                throw std::runtime_error("Unable to open the component map file");
            }
            const auto maps = turbofan::load_map_dataset(file);
            std::cerr << "map_variant=" << std::quoted(maps.variant)
                      << " map_provenance=" << std::quoted(maps.provenance) << '\n';
            return turbofan::solve_with_synthetic_geometry(point, maps);
        }();
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
