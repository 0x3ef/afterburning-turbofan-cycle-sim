#include "turbofan/cycle.hpp"

#include "matching.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

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

struct MapPoint {
    double corrected_flow;
    double ratio;
    double efficiency;
};

struct Map {
    std::array<std::array<MapPoint, 3>, 3> rows;

    MapPoint evaluate(double speed, double position) const {
        if (!(grid.front() <= speed && speed <= grid.back()) ||
            !(grid.front() <= position && position <= grid.back())) {
            throw std::domain_error("Synthetic map query is outside the grid");
        }

        const std::size_t i = speed < grid[1] ? 0 : 1;
        const std::size_t j = position < grid[1] ? 0 : 1;
        const double u = (speed - grid[i]) / (grid[i + 1] - grid[i]);
        const double v = (position - grid[j]) / (grid[j + 1] - grid[j]);
        const auto interpolate = [&](double MapPoint::*member) {
            const double low = std::lerp(rows[i][j].*member, rows[i][j + 1].*member, v);
            const double high = std::lerp(rows[i + 1][j].*member, rows[i + 1][j + 1].*member, v);
            return std::lerp(low, high, u);
        };

        return {interpolate(&MapPoint::corrected_flow), interpolate(&MapPoint::ratio),
                interpolate(&MapPoint::efficiency)};
    }
};

struct Fixture {
    DryCase design;
    Map fan;
    Map compressor;
    Map hp_turbine;
    Map lp_turbine;
    double core_mixer_area;
    double bypass_mixer_area;
    double fan_inlet_temperature;
    double compressor_inlet_temperature;
    double hpt_inlet_temperature;
    double lpt_inlet_temperature;
    double hp_power_scale;
    double lp_power_scale;
};

double corrected_flow(TotalState state) {
    return state.mass_flow * std::sqrt(state.temperature / reference_temperature) /
           (state.pressure / reference_pressure);
}

double power(TotalState in, TotalState out) {
    return in.mass_flow * cp * (out.temperature - in.temperature);
}

Map synthetic_map(TotalState in, double ratio, double efficiency, bool turbine) {
    Map map{};
    const double flow = corrected_flow(in);
    for (std::size_t i = 0; i < grid.size(); ++i) {
        for (std::size_t j = 0; j < grid.size(); ++j) {
            const double speed = grid[i] - 1.0;
            const double position = grid[j] - 1.0;
            const double flow_factor = turbine ? 1.0 + 0.08 * speed + 0.20 * position
                                               : 1.0 + 0.55 * speed + 0.35 * position;
            const double mapped_ratio = turbine ? ratio * grid[j]
                                                : 1.0 + (ratio - 1.0) *
                                                    (1.0 + 1.25 * speed - 0.30 * position);
            map.rows[i][j] = {flow * flow_factor, mapped_ratio,
                              efficiency - 0.12 * (speed * speed + position * position)};
        }
    }
    return map;
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

    return {design, synthetic_map(station2, 2.2, 0.85, false),
            synthetic_map(core, 5.4, 0.86, false),
            synthetic_map(burner_exit, hp_ratio, 0.89, true),
            synthetic_map(hp_exit, lp_ratio, 0.90, true),
            core_area, bypass_area, station2.temperature, core.temperature,
            burner_exit.temperature, hp_exit.temperature, hp_load, lp_load};
}

const Fixture& fixture() {
    static const Fixture data = make_fixture();
    return data;
}

double corrected_speed_fraction(double speed_fraction, TotalState in, double design_temperature) {
    return speed_fraction * std::sqrt(design_temperature / in.temperature);
}

CycleResult evaluate(const Fixture& data, const WetCase& commands, const detail::Vector& x) {
    const DryCase& point = commands.base;
    const auto [flow_fraction, split, fan_line, compressor_line, hp_expansion,
                lp_expansion, lp_speed, hp_speed] = x;
    if (!(0.0 < split && split < 1.0) || !(lp_speed > 0.0 && hp_speed > 0.0)) {
        throw std::domain_error("Trial split or rotor speed is outside its physical domain");
    }

    const auto station2 = inlet(point.ambient_pressure, point.ambient_temperature,
                                point.mach, point.inlet_recovery, flow_fraction * inlet_flow);
    const auto fan = data.fan.evaluate(
        corrected_speed_fraction(lp_speed, station2, data.fan_inlet_temperature), fan_line);
    const auto fan_exit = compress(station2, fan.ratio, fan.efficiency);
    auto core = fan_exit;
    core.mass_flow *= 1.0 - split;
    auto bypass = fan_exit;
    bypass.mass_flow *= split;
    bypass.pressure *= bypass_recovery;

    const auto compressor = data.compressor.evaluate(
        corrected_speed_fraction(hp_speed, core, data.compressor_inlet_temperature), compressor_line);
    const auto compressor_exit = compress(core, compressor.ratio, compressor.efficiency);
    const auto burner_exit = burn(compressor_exit, point.fuel_flow, heating_value,
                                  burner_efficiency, burner_recovery);
    const auto hpt = data.hp_turbine.evaluate(
        corrected_speed_fraction(hp_speed, burner_exit, data.hpt_inlet_temperature), hp_expansion);
    const auto hp_exit = expand(burner_exit, hpt.ratio, hpt.efficiency);
    const auto lpt = data.lp_turbine.evaluate(
        corrected_speed_fraction(lp_speed, hp_exit, data.lpt_inlet_temperature), lp_expansion);
    const auto lp_exit = expand(hp_exit, lpt.ratio, lpt.efficiency);

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
    const detail::Vector residuals{
        (corrected_flow(station2) - fan.corrected_flow) / corrected_flow(station2),
        (corrected_flow(core) - compressor.corrected_flow) / corrected_flow(core),
        (corrected_flow(burner_exit) - hpt.corrected_flow) / corrected_flow(burner_exit),
        (corrected_flow(hp_exit) - lpt.corrected_flow) / corrected_flow(hp_exit),
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
    const auto& data = fixture();
    const detail::Vector initial{1.0, bypass_fraction, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const auto matching = detail::match(
        [&](const detail::Vector& x) { return evaluate(data, operating_point, x).scaled_residuals; },
        initial);
    auto result = evaluate(data, operating_point, matching.x);
    result.iterations = matching.iterations;
    result.converged = matching.converged;
    return result;
}

}  // namespace turbofan
