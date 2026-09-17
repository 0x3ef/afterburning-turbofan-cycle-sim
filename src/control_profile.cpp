#include "turbofan/control_profile.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <istream>
#include <iterator>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

namespace turbofan {
namespace {

constexpr std::string_view format = "turbofan-control-v1";

enum class RowKind {
    timing, fuel_pi, nozzle_pi, fuel_actuator, augmentor_actuator, throat_actuator,
    area_ratio, schedule, demand
};

struct RowSpec {
    RowKind kind;
    std::string_view name;
    std::size_t columns;
    bool singleton;
};

using enum RowKind;
constexpr std::array row_specs{
    RowSpec{timing, "timing", 4, true},
    RowSpec{fuel_pi, "fuel_pi", 5, true},
    RowSpec{nozzle_pi, "nozzle_pi", 5, true},
    RowSpec{fuel_actuator, "fuel_actuator", 3, true},
    RowSpec{augmentor_actuator, "augmentor_actuator", 3, true},
    RowSpec{throat_actuator, "throat_actuator", 3, true},
    RowSpec{area_ratio, "area_ratio", 2, true},
    RowSpec{schedule, "schedule", 5, false},
    RowSpec{demand, "demand", 3, false},
};

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool nonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool normalized(double value) {
    return nonnegative(value) && value <= 1.0;
}

double run_duration(const ControlProfile& profile) {
    return profile.step_seconds * static_cast<double>(profile.steps);
}

[[noreturn]] void line_error(std::size_t line, const std::string& explanation) {
    throw std::runtime_error("control line " + std::to_string(line) + " " + explanation);
}

std::string_view trim(std::string_view field) {
    constexpr std::string_view whitespace = " \t\r\n\f\v";
    const auto start = field.find_first_not_of(whitespace);
    if (start == std::string_view::npos) {
        return {};
    }
    return field.substr(start, field.find_last_not_of(whitespace) - start + 1);
}

using Fields = std::array<std::string_view, 4>;

Fields read_fields(std::string_view line, std::size_t line_number, const RowSpec& spec) {
    if (line.find_first_of("\"\r\n") != std::string_view::npos ||
        line.find('\0') != std::string_view::npos) {
        line_error(line_number, "contains a quote or line break in an unquoted field");
    }
    Fields result{};
    for (std::size_t i = 0; i < spec.columns - 1; ++i) {
        const auto comma = line.find(',');
        const bool last = i + 2 == spec.columns;
        if (last != (comma == std::string_view::npos)) {
            line_error(line_number, "expected " + std::to_string(spec.columns) +
                                   " fields for " + std::string(spec.name));
        }
        result[i] = trim(line.substr(0, comma));
        if (!last) {
            line.remove_prefix(comma + 1);
        }
    }
    return result;
}

template <typename T>
T read_number(std::string_view field, std::size_t line_number, std::string_view tag) {
    const bool has_plus = field.starts_with('+');
    if (has_plus) {
        field.remove_prefix(1);
    }
    if (field.empty() || (has_plus && field.starts_with('-'))) {
        line_error(line_number, "invalid number in " + std::string(tag));
    }
    T value{};
    const auto end = field.data() + field.size();
    const auto parsed = std::from_chars(field.data(), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end) {
        line_error(line_number, "invalid number in " + std::string(tag));
    }
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(value)) {
            line_error(line_number, "expected a finite number in " + std::string(tag));
        }
    }
    return value;
}

void read_row(ControlProfile& profile, const RowSpec& spec, const Fields& fields,
              std::size_t line_number) {
    const auto number = [&](std::size_t i) {
        return read_number<double>(fields[i], line_number, spec.name);
    };
    switch (spec.kind) {
        case timing:
            profile.step_seconds = number(0);
            profile.steps = read_number<std::size_t>(fields[1], line_number, spec.name);
            profile.trim_throttle = number(2);
            break;
        case fuel_pi:
            profile.controller.fuel = {number(0), number(1), number(2), number(3)};
            break;
        case nozzle_pi:
            profile.controller.nozzle = {number(0), number(1), number(2), number(3)};
            break;
        case fuel_actuator:
            profile.actuators.main_fuel = {number(0), number(1)};
            break;
        case augmentor_actuator:
            profile.actuators.augmentor_fuel = {number(0), number(1)};
            break;
        case throat_actuator:
            profile.actuators.throat_area = {number(0), number(1)};
            break;
        case area_ratio:
            profile.controller.exit_to_throat_ratio = profile.actuators.exit_to_throat_ratio =
                number(0);
            break;
        case schedule:
            profile.controller.schedule.push_back({number(0), number(1), number(2), number(3)});
            break;
        case demand:
            profile.demand.push_back({number(0), number(1)});
            break;
    }
}

void write_pi(std::ostream& output, std::string_view tag, PiSettings pi) {
    output << tag << ',' << pi.proportional_gain << ',' << pi.integral_gain << ','
           << pi.minimum << ',' << pi.maximum << '\n';
}

void write_response(std::ostream& output, std::string_view tag, ActuatorResponse response) {
    output << tag << ',' << response.time_constant_seconds << ','
           << response.rate_limit_per_second << '\n';
}

}  // namespace

void validate_control_profile(const ControlProfile& profile) {
    const double duration = run_duration(profile);
    if (!positive(profile.step_seconds) || profile.steps == 0 || !positive(duration) ||
        !normalized(profile.trim_throttle)) {
        throw std::invalid_argument("control timing requires a positive finite time grid and "
                                    "a trim throttle in [0, 1]");
    }
    const auto validate_pi = [](PiSettings pi, const char* name) {
        if (!nonnegative(pi.proportional_gain) || !nonnegative(pi.integral_gain) ||
            !nonnegative(pi.minimum) || !std::isfinite(pi.maximum) || pi.maximum < pi.minimum) {
            throw std::invalid_argument(std::string("control invalid gains or bounds for ") + name);
        }
    };
    const auto& controller = profile.controller;
    const auto& actuators = profile.actuators;
    validate_pi(controller.fuel, "fuel_pi");
    validate_pi(controller.nozzle, "nozzle_pi");
    if (!positive(controller.nozzle.minimum)) {
        throw std::invalid_argument("control nozzle_pi requires a positive minimum area");
    }
    const double ratio = controller.exit_to_throat_ratio;
    if (!std::isfinite(ratio) || ratio < 1.0 || ratio != actuators.exit_to_throat_ratio ||
        !std::isfinite(controller.nozzle.maximum * ratio)) {
        throw std::invalid_argument("control area_ratio must be at least one and agree between "
                                    "controller and actuators, with finite exit areas");
    }
    const auto validate_response = [](ActuatorResponse response, const char* name) {
        if (!positive(response.time_constant_seconds) ||
            !nonnegative(response.rate_limit_per_second)) {
            throw std::invalid_argument(std::string("control invalid time constant or rate for ") +
                                        name);
        }
    };
    validate_response(actuators.main_fuel, "fuel_actuator");
    validate_response(actuators.augmentor_fuel, "augmentor_actuator");
    validate_response(actuators.throat_area, "throat_actuator");

    const auto& knots = controller.schedule;
    if (knots.size() < 2 || knots.front().throttle != 0.0 || knots.back().throttle != 1.0) {
        throw std::invalid_argument("control schedule must cover throttle from 0 to 1");
    }
    double last = -1.0;
    for (const auto& point : knots) {
        if (!normalized(point.throttle) || point.throttle <= last ||
            !positive(point.lp_speed_rpm) || !positive(point.pressure_ratio) ||
            !nonnegative(point.augmentor_fuel_flow)) {
            throw std::invalid_argument("control invalid or out-of-order schedule point");
        }
        last = point.throttle;
    }

    const auto& points = profile.demand;
    if (points.empty() || points.front().time != 0.0 ||
        points.front().throttle != profile.trim_throttle) {
        throw std::invalid_argument("control demand must start at time zero and trim throttle");
    }
    last = -1.0;
    for (const auto& point : points) {
        if (!nonnegative(point.time) || point.time <= last || point.time > duration ||
            !normalized(point.throttle)) {
            throw std::invalid_argument("control demand values must be in [0, 1] at increasing "
                                        "times within the run");
        }
        last = point.time;
    }
}

ControlProfile load_control_profile(std::istream& input) {
    std::string line;
    if (!std::getline(input, line) || trim(line) != format) {
        line_error(1, "expected " + std::string(format));
    }
    std::size_t line_number = 1;
    std::array<bool, row_specs.size()> seen{};
    ControlProfile profile{};
    while (std::getline(input, line)) {
        ++line_number;
        const std::string_view row = trim(line);
        const auto comma = row.find(',');
        const auto tag = trim(row.substr(0, comma));
        const auto spec = std::find_if(row_specs.begin(), row_specs.end(),
                                       [tag](const auto& value) { return tag == value.name; });
        if (spec == row_specs.end()) {
            line_error(line_number, "unknown row " + std::string(tag));
        }
        if (comma == std::string_view::npos) {
            line_error(line_number, "missing fields for " + std::string(tag));
        }
        const auto index = static_cast<std::size_t>(spec - row_specs.begin());
        if (spec->singleton && seen[index]) {
            line_error(line_number, "duplicate row " + std::string(tag));
        }
        read_row(profile, *spec, read_fields(row.substr(comma + 1), line_number, *spec),
                 line_number);
        seen[index] = true;
    }
    if (!input.eof() || input.bad()) {
        line_error(line_number + 1, "could not be read");
    }
    for (std::size_t i = 0; i < row_specs.size(); ++i) {
        if (row_specs[i].singleton && !seen[i]) {
            throw std::runtime_error("control profile is missing row " +
                                     std::string(row_specs[i].name));
        }
    }
    try {
        validate_control_profile(profile);
    } catch (const std::invalid_argument& error) {
        throw std::runtime_error(error.what());
    }
    return profile;
}

void write_control_profile(std::ostream& output, const ControlProfile& profile) {
    validate_control_profile(profile);
    std::ostringstream csv;
    csv.imbue(std::locale::classic());
    csv << std::setprecision(std::numeric_limits<double>::max_digits10) << format << '\n'
        << "timing," << profile.step_seconds << ',' << profile.steps << ','
        << profile.trim_throttle << '\n';
    write_pi(csv, "fuel_pi", profile.controller.fuel);
    write_pi(csv, "nozzle_pi", profile.controller.nozzle);
    write_response(csv, "fuel_actuator", profile.actuators.main_fuel);
    write_response(csv, "augmentor_actuator", profile.actuators.augmentor_fuel);
    write_response(csv, "throat_actuator", profile.actuators.throat_area);
    csv << "area_ratio," << profile.controller.exit_to_throat_ratio << '\n';
    for (const auto& point : profile.controller.schedule) {
        csv << "schedule," << point.throttle << ',' << point.lp_speed_rpm << ','
            << point.pressure_ratio << ',' << point.augmentor_fuel_flow << '\n';
    }
    for (const auto& point : profile.demand) {
        csv << "demand," << point.time << ',' << point.throttle << '\n';
    }
    const std::string data = csv.str();
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!output) {
        throw std::runtime_error("control profile could not be written");
    }
}

double throttle_at(const ControlProfile& profile, double time) {
    if (!nonnegative(time) || !(time <= run_duration(profile)) || profile.demand.empty()) {
        throw std::domain_error("control demand time must be within the run");
    }
    const auto upper = std::upper_bound(profile.demand.begin(), profile.demand.end(), time,
        [](double value, const ThrottlePoint& point) { return value < point.time; });
    if (upper == profile.demand.begin()) {
        throw std::domain_error("control demand is undefined at the requested time");
    }
    return std::prev(upper)->throttle;
}

}  // namespace turbofan
