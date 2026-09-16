#include "turbofan/cycle.hpp"

#include "matching.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace turbofan {
namespace {

constexpr double reference_temperature = 288.15;
constexpr double reference_pressure = 101325.0;
constexpr double inlet_flow = 90.0;
constexpr double bypass_fraction = 0.43;
constexpr double lp_rpm = 9000.0;
constexpr double hp_rpm = 14000.0;
constexpr double heating_value = 43.0e6;
constexpr double burner_efficiency = 0.98;
constexpr double burner_recovery = 0.96;
constexpr double bypass_recovery = 0.97;
constexpr double shaft_efficiency = 0.99;
constexpr double mixer_recovery = 0.98;
constexpr double dry_augmentor_recovery = 0.98;
constexpr double augmentor_efficiency = 0.95;
constexpr double stoichiometric_fuel_air_ratio = 0.068;
constexpr double reheat_loss_coefficient = 3.0;
constexpr std::array grid{0.65, 1.0, 1.35};
constexpr double exponent = (gamma - 1.0) / gamma;

struct Fixture {
    DryCase design;
    MapDataset maps;
    double core_mixer_area;
    double bypass_mixer_area;
    double hp_expansion;
    double lp_expansion;
    double hp_power_scale;
    double lp_power_scale;
};

double corrected_flow(TotalState state, double temperature_ref, double pressure_ref) {
    return state.mass_flow * std::sqrt(state.temperature / temperature_ref) /
           (state.pressure / pressure_ref);
}

double corrected_speed(double speed_rpm, TotalState in, double temperature_ref) {
    return speed_rpm / std::sqrt(in.temperature / temperature_ref);
}

double power(TotalState in, TotalState out) {
    return in.mass_flow * cp * (out.temperature - in.temperature);
}

PerformanceMap synthetic_map(TotalState in, double speed_rpm, double ratio,
                             double efficiency, bool turbine) {
    std::vector<MapSample> samples;
    samples.reserve(grid.size() * grid.size());
    const double flow = corrected_flow(in, reference_temperature, reference_pressure);
    const double speed_rpm_corrected = corrected_speed(speed_rpm, in, reference_temperature);
    for (const double speed_fraction : grid) {
        for (const double position_fraction : grid) {
            const double speed = speed_fraction - 1.0;
            const double position = position_fraction - 1.0;
            const double flow_factor = turbine ? 1.0 + 0.08 * speed + 0.20 * position
                                               : 1.0 + 0.55 * speed + 0.35 * position;
            const double mapped_ratio = turbine ? ratio * position_fraction
                                                : 1.0 + (ratio - 1.0) *
                                                    (1.0 + 1.25 * speed - 0.30 * position);
            samples.push_back({speed_rpm_corrected * speed_fraction,
                               turbine ? mapped_ratio : position_fraction,
                               {flow * flow_factor, mapped_ratio,
                                efficiency - 0.12 * (speed * speed + position * position)}});
        }
    }
    return PerformanceMap(std::move(samples));
}

MapPoint lookup(const PerformanceMap& map, const char* component, double speed, double position) {
    try {
        return map.evaluate(speed, position);
    } catch (const std::domain_error& error) {
        throw std::domain_error(std::string(component) + " map at corrected rpm " +
                                std::to_string(speed) + " and coordinate " +
                                std::to_string(position) + ": " + error.what());
    }
}

double expansion_for_power(TotalState in, double load, double efficiency) {
    const double temperature_fraction = 1.0 - load /
        (shaft_efficiency * in.mass_flow * cp * in.temperature * efficiency);
    return std::pow(temperature_fraction, -1.0 / exponent);
}

Fixture make_fixture() {
    DryCase design{reference_pressure, reference_temperature, 0.0, 0.99, 0.95, 0.0};
    const auto station2 = inlet(design.ambient_pressure, design.ambient_temperature,
                                design.mach, design.inlet_recovery, inlet_flow);
    const auto fan_exit = compress(station2, 2.2, 0.85);
    auto core = fan_exit;
    core.mass_flow *= 1.0 - bypass_fraction;
    auto bypass = fan_exit;
    bypass.mass_flow *= bypass_fraction;
    bypass.pressure *= bypass_recovery;

    const auto compressor_exit = compress(core, 5.4, 0.86);
    const auto burner_exit = burn(compressor_exit, design.fuel_flow, heating_value,
                                  burner_efficiency, burner_recovery);
    const double hp_load = power(core, compressor_exit);
    const double lp_load = power(station2, fan_exit);
    const double hp_ratio = expansion_for_power(burner_exit, hp_load, 0.89);
    const auto hp_exit = expand(burner_exit, hp_ratio, 0.89);
    const double lp_ratio = expansion_for_power(hp_exit, lp_load, 0.90);
    const auto lp_exit = expand(hp_exit, lp_ratio, 0.90);
    const double mixer_pressure = 0.90 * std::min(lp_exit.pressure, bypass.pressure);
    const double core_area = lp_exit.mass_flow / nozzle_capacity(lp_exit, 1.0, mixer_pressure);
    const double bypass_area = bypass.mass_flow / nozzle_capacity(bypass, 1.0, mixer_pressure);
    auto dry_exit = mix(lp_exit, bypass, mixer_recovery);
    dry_exit.pressure *= dry_augmentor_recovery;
    design.throat_area = dry_exit.mass_flow / nozzle_capacity(dry_exit, 1.0, design.ambient_pressure);

    MapDataset maps{"synthetic-mixed-flow", "Built-in reference cycle fixture",
                    reference_pressure, reference_temperature,
                    synthetic_map(station2, lp_rpm, 2.2, 0.85, false),
                    synthetic_map(core, hp_rpm, 5.4, 0.86, false),
                    synthetic_map(burner_exit, hp_rpm, hp_ratio, 0.89, true),
                    synthetic_map(hp_exit, lp_rpm, lp_ratio, 0.90, true)};
    return {design, std::move(maps), core_area, bypass_area, hp_ratio, lp_ratio, hp_load, lp_load};
}

const Fixture& fixture() {
    static const Fixture data = make_fixture();
    return data;
}

CycleResult evaluate(const Fixture& data, const MapDataset& maps, const WetCase& commands,
                     const detail::Vector& x) {
    const DryCase& point = commands.base;
    const auto [flow_fraction, split, fan_line, compressor_line, hp_expansion,
                lp_expansion, lp_speed, hp_speed] = x;
    if (!(0.0 < split && split < 1.0) || !(lp_speed > 0.0 && hp_speed > 0.0)) {
        throw std::domain_error("Trial split or rotor speed is outside its physical domain");
    }

    const auto station2 = inlet(point.ambient_pressure, point.ambient_temperature,
                                point.mach, point.inlet_recovery, flow_fraction * inlet_flow);
    const auto fan = lookup(maps.fan, "Fan",
                            corrected_speed(lp_speed * lp_rpm, station2, maps.reference_temperature),
                            fan_line);
    const auto fan_exit = compress(station2, fan.pressure_ratio, fan.efficiency);
    auto core = fan_exit;
    core.mass_flow *= 1.0 - split;
    auto bypass = fan_exit;
    bypass.mass_flow *= split;
    bypass.pressure *= bypass_recovery;

    const auto compressor = lookup(maps.compressor, "Compressor",
                                   corrected_speed(hp_speed * hp_rpm, core, maps.reference_temperature),
                                   compressor_line);
    const auto compressor_exit = compress(core, compressor.pressure_ratio, compressor.efficiency);
    const auto burner_exit = burn(compressor_exit, point.fuel_flow, heating_value,
                                  burner_efficiency, burner_recovery);
    const auto hpt = lookup(maps.hp_turbine, "HP turbine",
                            corrected_speed(hp_speed * hp_rpm, burner_exit, maps.reference_temperature),
                            hp_expansion * data.hp_expansion);
    const auto hp_exit = expand(burner_exit, hpt.pressure_ratio, hpt.efficiency);
    const auto lpt = lookup(maps.lp_turbine, "LP turbine",
                            corrected_speed(lp_speed * lp_rpm, hp_exit, maps.reference_temperature),
                            lp_expansion * data.lp_expansion);
    const auto lp_exit = expand(hp_exit, lpt.pressure_ratio, lpt.efficiency);

    const double core_pressure = subsonic_static(lp_exit, data.core_mixer_area).pressure;
    const double bypass_pressure = subsonic_static(bypass, data.bypass_mixer_area).pressure;
    const auto mixed = mix(lp_exit, bypass, mixer_recovery);
    const double augmentor_far = commands.augmentor_fuel_flow / station2.mass_flow;
    const double augmentor_recovery = dry_augmentor_recovery *
        std::exp(-reheat_loss_coefficient * augmentor_far);
    const auto augmentor_exit = augment(mixed, station2.mass_flow, point.fuel_flow,
                                        commands.augmentor_fuel_flow, heating_value,
                                        augmentor_efficiency, augmentor_recovery,
                                        stoichiometric_fuel_air_ratio);
    const auto nozzle = convergent_divergent_nozzle(augmentor_exit, point.throat_area,
                                                    commands.exit_area, point.ambient_pressure);
    const double capacity = nozzle.mass_flow_capacity;
    const double lp_net = -shaft_efficiency * power(hp_exit, lp_exit) - power(station2, fan_exit);
    const double hp_net = -shaft_efficiency * power(burner_exit, hp_exit) - power(core, compressor_exit);
    const auto flow_residual = [&](TotalState in, MapPoint map) {
        const double trial_flow = corrected_flow(in, maps.reference_temperature, maps.reference_pressure);
        return (trial_flow - map.corrected_flow) / trial_flow;
    };
    const detail::Vector residuals{
        flow_residual(station2, fan),
        flow_residual(core, compressor),
        flow_residual(burner_exit, hpt),
        flow_residual(hp_exit, lpt),
        (core_pressure - bypass_pressure) / reference_pressure,
        (augmentor_exit.mass_flow - capacity) / inlet_flow,
        hp_net / data.hp_power_scale,
        lp_net / data.lp_power_scale,
    };

    if (!std::all_of(residuals.begin(), residuals.end(), [](double r) { return std::isfinite(r); })) {
        throw std::domain_error("Nonfinite gas-path residual");
    }

    const double core_fuel_air_ratio = point.fuel_flow / core.mass_flow;
    const double mixed_fuel_air_ratio = point.fuel_flow / station2.mass_flow;
    const std::array stations{
        Station{"engine_inlet", station2, 0.0},
        Station{"fan_exit", fan_exit, 0.0},
        Station{"compressor_exit", compressor_exit, 0.0},
        Station{"combustor_exit", burner_exit, core_fuel_air_ratio},
        Station{"hp_turbine_exit", hp_exit, core_fuel_air_ratio},
        Station{"lp_turbine_exit", lp_exit, core_fuel_air_ratio},
        Station{"bypass_duct_exit", bypass, 0.0},
        Station{"mixer_exit", mixed, mixed_fuel_air_ratio},
        Station{"nozzle_inlet", augmentor_exit, mixed_fuel_air_ratio + augmentor_far},
    };

    return {stations, lp_speed * lp_rpm, hp_speed * hp_rpm, split, core_pressure,
            bypass_pressure, capacity, lp_net, hp_net, residuals, 0, false, nozzle};
}

}  // namespace

DryCase synthetic_design_case() {
    return fixture().design;
}

DryCase synthetic_off_design_case() {
    DryCase point = synthetic_design_case();
    point.fuel_flow *= 0.94;
    point.throat_area *= 1.02;
    return point;
}

WetCase synthetic_wet_case() {
    auto base = synthetic_design_case();
    base.throat_area *= 1.10;
    return {base, 0.22, 1.20 * base.throat_area};
}

DryResult solve_synthetic_dry(DryCase operating_point) {
    return solve_synthetic_wet({operating_point, 0.0, operating_point.throat_area});
}

CycleResult solve_synthetic_wet(WetCase operating_point) {
    return solve_with_synthetic_geometry(operating_point, synthetic_map_dataset());
}

const MapDataset& synthetic_map_dataset() {
    return fixture().maps;
}

CycleResult solve_with_synthetic_geometry(WetCase operating_point, const MapDataset& maps) {
    if (!(std::isfinite(maps.reference_pressure) && maps.reference_pressure > 0.0) ||
        !(std::isfinite(maps.reference_temperature) && maps.reference_temperature > 0.0)) {
        throw std::invalid_argument("Map correction references must be finite and positive");
    }
    const auto& data = fixture();
    const detail::Vector initial{1.0, bypass_fraction, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const auto matching = detail::match(
        [&](const detail::Vector& x) { return evaluate(data, maps, operating_point, x).scaled_residuals; },
        initial);
    auto result = evaluate(data, maps, operating_point, matching.x);
    result.iterations = matching.iterations;
    result.converged = matching.converged;
    return result;
}

}  // namespace turbofan
