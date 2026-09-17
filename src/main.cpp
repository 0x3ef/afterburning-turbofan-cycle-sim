#include "turbofan/control_profile.hpp"
#include "turbofan/cycle.hpp"
#include "turbofan/transient.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

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

bool is_preset(std::string_view mode) {
    return mode == "design" || mode == "off-design" || mode == "wet";
}

turbofan::WetCase preset(std::string_view mode) {
    if (mode == "wet") {
        return turbofan::synthetic_wet_case();
    }
    const auto dry = mode == "design" ? turbofan::synthetic_design_case()
                                       : turbofan::synthetic_off_design_case();
    return {dry, 0.0, dry.throat_area};
}

turbofan::MapDataset read_maps(const char* path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Unable to open the component map file");
    }
    auto maps = turbofan::load_map_dataset(file);
    std::cerr << "map_variant=" << std::quoted(maps.variant)
              << " map_provenance=" << std::quoted(maps.provenance) << '\n';
    return maps;
}

turbofan::CycleSetup read_setup(const char* path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Unable to open the case file");
    }
    auto setup = turbofan::load_cycle_setup(file);
    std::cerr << "engine_variant=" << std::quoted(setup.engine.variant)
              << " engine_provenance=" << std::quoted(setup.engine.provenance) << '\n';
    return setup;
}

constexpr double time_step = 0.01;
constexpr std::size_t time_steps = 100;
constexpr double step_time = 0.1;

std::vector<turbofan::TransientSample> run_fuel_step(const turbofan::CycleSetup& setup,
                                                      const turbofan::MapDataset& maps) {
    return turbofan::simulate_transient(setup, maps, time_step, time_steps,
        [&](double time, const turbofan::CycleResult&) {
            auto point = setup.point;
            if (time >= step_time) {
                point.base.fuel_flow *= 1.006;
            }
            return point;
        });
}

turbofan::ControlProfile reference_profile(const turbofan::CycleSetup& setup,
                                           const turbofan::MapDataset& maps) {
    const auto trim = turbofan::solve_steady(setup.point, setup.engine, maps, setup.seed);
    if (!trim.converged) {
        throw std::runtime_error("The reference control profile requires a converged trim");
    }
    const auto& point = setup.point;
    const double fuel = point.base.fuel_flow;
    const double throat = point.base.throat_area;
    const double rpm = trim.lp_speed_rpm;
    const double pr = trim.stations[7].total.pressure / trim.stations[0].total.pressure;
    const double ratio = point.exit_area / throat;
    return {
        {{{0.0, 0.99 * rpm, 0.995 * pr, 0.0},
          {0.5, rpm, pr, 0.0},
          {1.0, 1.01 * rpm, 1.005 * pr, 0.0}},
         {1.2e-5, 2.5e-5, 0.90 * fuel, 1.10 * fuel},
         {0.05 * throat, 0.10 * throat, 0.97 * throat, 1.03 * throat},
         ratio},
        {{0.04, 0.20 * fuel}, {0.08, 0.20 * fuel}, {0.06, 0.15 * throat}, ratio},
        0.5, time_step, time_steps, {{0.0, 0.5}, {step_time, 0.75}},
    };
}

turbofan::ControlProfile read_profile(const char* path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Unable to open the control profile");
    }
    return turbofan::load_control_profile(file);
}

void write_transient(std::string_view mode, std::string_view variant,
                     const std::vector<turbofan::TransientSample>& history) {
    std::cout << std::setprecision(12)
              << "case,variant,time_s,station,pt_pa,tt_k,flow_kg_s,h_total_j_kg,fuel_air_ratio,"
                 "lp_rpm,hp_rpm,lp_net_power_w,hp_net_power_w,main_fuel_kg_s,augmentor_fuel_kg_s,"
                 "throat_m2,exit_m2\n";
    for (const auto& sample : history) {
        for (const auto& station : sample.result.stations) {
            const auto& result = sample.result;
            std::cout << mode << ',' << variant << ',' << sample.time << ',' << station.name << ','
                      << station.total.pressure << ',' << station.total.temperature << ','
                      << station.total.mass_flow << ',' << turbofan::cp * station.total.temperature
                      << ',' << station.fuel_air_ratio << ',' << result.lp_speed_rpm << ','
                      << result.hp_speed_rpm << ',' << result.lp_net_power << ','
                      << result.hp_net_power << ',' << sample.command.base.fuel_flow << ','
                      << sample.command.augmentor_fuel_flow << ',' << sample.command.base.throat_area
                      << ',' << sample.command.exit_area << '\n';
        }
    }
}

void benchmark() {
    const auto setup = turbofan::synthetic_setup(preset("design"));
    const auto& maps = turbofan::synthetic_map_dataset();
    const auto profile = reference_profile(setup, maps);
    static_cast<void>(turbofan::simulate_controlled(setup, maps, profile));
    constexpr int trials = 10;
    double elapsed = 0.0;
    double slowest = 0.0;
    double simulated_seconds = 0.0;
    for (int i = 0; i < trials; ++i) {
        const auto start = std::chrono::steady_clock::now();
        const auto history = turbofan::simulate_controlled(setup, maps, profile);
        const auto end = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(end - start).count();
        elapsed += seconds;
        slowest = std::max(slowest, seconds);
        simulated_seconds += history.back().time;
    }
    if (!(elapsed > 0.0)) {
        throw std::runtime_error("Benchmark duration did not exceed timer resolution");
    }
    std::cout << std::setprecision(6)
              << "trials=" << trials << " steps_per_trial=" << profile.steps
              << " step_ms=" << 1000.0 * profile.step_seconds
              << " mean_run_ms=" << 1000.0 * elapsed / trials
              << " slowest_run_ms=" << 1000.0 * slowest
              << " mean_step_ms=" << 1000.0 * elapsed / (trials * static_cast<double>(profile.steps))
              << " simulated_seconds_per_wall_second=" << simulated_seconds / elapsed << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string_view mode = argc >= 2 ? argv[1] : "";
    const bool export_maps = mode == "export-maps" && argc == 2;
    const bool export_case = mode == "export-case" && argc == 3 && is_preset(argv[2]);
    const bool solve_case = mode == "solve-case" && argc == 4;
    const bool transient = mode == "transient" && (argc == 2 || argc == 4);
    const bool controlled = mode == "controlled" && (argc == 2 || argc == 5);
    const bool export_control = mode == "export-control" && argc == 2;
    const bool time_benchmark = mode == "benchmark" && argc == 2;
    const bool use_external_maps = argc == 4 && std::string_view(argv[2]) == "--maps";
    const bool run_cycle = is_preset(mode) && (argc == 2 || use_external_maps);
    if (!run_cycle && !export_maps && !export_case && !solve_case &&
        !transient && !controlled && !export_control && !time_benchmark) {
        std::cerr << "Usage: turbofan-cli design|off-design|wet [--maps FILE]\n"
                     "       turbofan-cli export-maps\n"
                     "       turbofan-cli export-case design|off-design|wet\n"
                     "       turbofan-cli solve-case CASE MAPS\n"
                     "       turbofan-cli transient [CASE MAPS]\n"
                     "       turbofan-cli controlled [CASE MAPS PROFILE]\n"
                     "       turbofan-cli export-control|benchmark\n";
        return 2;
    }

    try {
        if (time_benchmark) {
            benchmark();
            return 0;
        }
        if (export_control) {
            const auto setup = turbofan::synthetic_setup(preset("design"));
            turbofan::write_control_profile(
                std::cout, reference_profile(setup, turbofan::synthetic_map_dataset()));
            return 0;
        }
        if (controlled) {
            if (argc == 5) {
                const auto setup = read_setup(argv[2]);
                const auto maps = read_maps(argv[3]);
                const auto profile = read_profile(argv[4]);
                write_transient(mode, setup.engine.variant,
                                turbofan::simulate_controlled(setup, maps, profile));
            } else {
                const auto setup = turbofan::synthetic_setup(preset("design"));
                const auto& maps = turbofan::synthetic_map_dataset();
                write_transient(mode, setup.engine.variant,
                                turbofan::simulate_controlled(setup, maps, reference_profile(setup, maps)));
            }
            return 0;
        }
        if (transient) {
            if (argc == 4) {
                const auto setup = read_setup(argv[2]);
                const auto maps = read_maps(argv[3]);
                write_transient(mode, setup.engine.variant, run_fuel_step(setup, maps));
            } else {
                const auto setup = turbofan::synthetic_setup(preset("design"));
                write_transient(mode, setup.engine.variant,
                                run_fuel_step(setup, turbofan::synthetic_map_dataset()));
            }
            return 0;
        }
        if (export_maps) {
            turbofan::write_map_dataset(std::cout, turbofan::synthetic_map_dataset());
            return 0;
        }
        if (export_case) {
            turbofan::write_cycle_setup(std::cout, turbofan::synthetic_setup(preset(argv[2])));
            return 0;
        }
        const std::string_view model = solve_case ? "configured-cycle"
            : use_external_maps ? "external-maps-synthetic-geometry"
            : mode == "wet" ? "synthetic-mixed-flow-wet" : "synthetic-mixed-flow-dry";
        const auto result = [&] {
            if (solve_case) {
                const auto setup = read_setup(argv[2]);
                const auto maps = read_maps(argv[3]);
                return turbofan::solve_steady(setup.point, setup.engine, maps, setup.seed);
            }
            const auto point = preset(mode);
            if (use_external_maps) {
                const auto maps = read_maps(argv[3]);
                return turbofan::solve_with_synthetic_geometry(point, maps);
            }
            return turbofan::solve_synthetic_wet(point);
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
