#include "turbofan/engine_config.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <istream>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

namespace turbofan {
namespace {

constexpr std::string_view format = "turbofan-case-v1";

enum class Domain { finite, positive, nonnegative, fraction, open_fraction, at_least_one };

template <typename Value>
struct NumericField {
    std::string_view key;
    Value& value;
    Domain domain;
};

// Shared field order and units for validation, parsing, and serialization.
template <typename Setup>
auto numeric_fields(Setup& setup) {
    auto& [engine, point, seed] = setup;
    auto& base = point.base;
    using Field = NumericField<std::remove_reference_t<decltype((base.mach))>>;
    using enum Domain;
    return std::array{
        Field{"core_mixer_area_m2", engine.core_mixer_area, positive},
        Field{"bypass_mixer_area_m2", engine.bypass_mixer_area, positive},
        Field{"shaft_efficiency", engine.shaft_efficiency, fraction},
        Field{"bypass_recovery", engine.bypass_recovery, fraction},
        Field{"mixer_recovery", engine.mixer_recovery, fraction},
        Field{"burner_recovery", engine.burner_recovery, fraction},
        Field{"dry_augmentor_recovery", engine.dry_augmentor_recovery, fraction},
        Field{"heating_value_j_kg", engine.heating_value, positive},
        Field{"burner_efficiency", engine.burner_efficiency, fraction},
        Field{"augmentor_efficiency", engine.augmentor_efficiency, fraction},
        Field{"stoichiometric_fuel_air_ratio", engine.stoichiometric_fuel_air_ratio, positive},
        Field{"reheat_loss_coefficient", engine.reheat_loss_coefficient, nonnegative},
        Field{"lp_inertia_kg_m2", engine.lp_inertia, positive},
        Field{"hp_inertia_kg_m2", engine.hp_inertia, positive},
        Field{"ambient_pressure_pa", base.ambient_pressure, positive},
        Field{"ambient_temperature_k", base.ambient_temperature, positive},
        Field{"mach", base.mach, nonnegative},
        Field{"inlet_recovery", base.inlet_recovery, fraction},
        Field{"fuel_flow_kg_s", base.fuel_flow, nonnegative},
        Field{"throat_area_m2", base.throat_area, positive},
        Field{"augmentor_fuel_flow_kg_s", point.augmentor_fuel_flow, nonnegative},
        Field{"exit_area_m2", point.exit_area, positive},
        Field{"inlet_flow_kg_s", seed.inlet_flow, positive},
        Field{"bypass_fraction", seed.bypass_fraction, open_fraction},
        Field{"fan_coordinate", seed.fan_coordinate, finite},
        Field{"compressor_coordinate", seed.compressor_coordinate, finite},
        Field{"hp_expansion", seed.hp_expansion, at_least_one},
        Field{"lp_expansion", seed.lp_expansion, at_least_one},
        Field{"lp_speed_rpm", seed.lp_speed_rpm, positive},
        Field{"hp_speed_rpm", seed.hp_speed_rpm, positive},
    };
}

std::string_view invalid_range(double value, Domain domain) {
    if (!std::isfinite(value)) {
        return "must be finite";
    }
    switch (domain) {
        case Domain::finite: return {};
        case Domain::positive: return value > 0.0 ? "" : "must be positive";
        case Domain::nonnegative: return value >= 0.0 ? "" : "must be nonnegative";
        case Domain::fraction: return value > 0.0 && value <= 1.0 ? "" : "must be in (0, 1]";
        case Domain::open_fraction: return value > 0.0 && value < 1.0 ? "" : "must be in (0, 1)";
        case Domain::at_least_one: return value >= 1.0 ? "" : "must be at least one";
    }
    return "has an unknown domain";
}

std::string_view trim(std::string_view field) {
    constexpr std::string_view whitespace = " \t\r\n\f\v";
    const auto begin = field.find_first_not_of(whitespace);
    if (begin == std::string_view::npos) {
        return {};
    }
    const auto end = field.find_last_not_of(whitespace);
    return field.substr(begin, end - begin + 1);
}

void validate_metadata(std::string_view key, std::string_view field) {
    if (field.empty() || field != trim(field) ||
        field.find_first_of(",\"\r\n") != std::string_view::npos ||
        field.find('\0') != std::string_view::npos) {
        throw std::invalid_argument("case " + std::string(key) +
                                    " must be nonempty unquoted text without surrounding spaces");
    }
}

[[noreturn]] void line_error(std::size_t line, const std::string& message) {
    throw std::runtime_error("case line " + std::to_string(line) + " " + message);
}

std::pair<std::string_view, std::string_view> key_value(std::string_view line,
                                                        std::size_t line_number) {
    const auto comma = line.find(',');
    if (comma == std::string_view::npos || line.find(',', comma + 1) != std::string_view::npos ||
        line.find_first_of("\"\r\n") != std::string_view::npos ||
        line.find('\0') != std::string_view::npos) {
        line_error(line_number, "expected an unquoted key and value");
    }
    const auto key = trim(line.substr(0, comma));
    const auto value = trim(line.substr(comma + 1));
    if (key.empty() || value.empty()) {
        line_error(line_number, "expected a nonempty key and value");
    }
    return {key, value};
}

double parse_number(std::string_view text, std::size_t line, std::string_view key) {
    const bool has_plus = text.starts_with('+');
    if (has_plus) {
        text.remove_prefix(1);
    }
    if (text.empty() || (has_plus && text.starts_with('-'))) {
        line_error(line, "expected a finite number for " + std::string(key));
    }
    double value = 0.0;
    const auto end = text.data() + text.size();
    const auto result = std::from_chars(text.data(), end, value);
    if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(value)) {
        line_error(line, "expected a finite number for " + std::string(key));
    }
    return value;
}

}  // namespace

void validate_cycle_setup(const CycleSetup& setup) {
    validate_metadata("variant", setup.engine.variant);
    validate_metadata("provenance", setup.engine.provenance);
    for (const auto& field : numeric_fields(setup)) {
        const auto error = invalid_range(field.value, field.domain);
        if (!error.empty()) {
            throw std::invalid_argument("case " + std::string(field.key) + " " + std::string(error));
        }
    }
    if (setup.point.exit_area < setup.point.base.throat_area) {
        throw std::invalid_argument("case exit_area_m2 must be at least throat_area_m2");
    }
}

CycleSetup load_cycle_setup(std::istream& input) {
    std::string line;
    if (!std::getline(input, line) || trim(line) != format) {
        line_error(1, "expected " + std::string(format));
    }

    CycleSetup setup{};
    auto fields = numeric_fields(setup);
    std::array<bool, fields.size()> seen{};
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        const auto [key, value] = key_value(trim(line), line_number);
        if (key == "variant" || key == "provenance") {
            auto& metadata = key == "variant" ? setup.engine.variant : setup.engine.provenance;
            if (!metadata.empty()) {
                line_error(line_number, "duplicate key " + std::string(key));
            }
            metadata = value;
            continue;
        }

        const auto field = std::find_if(fields.begin(), fields.end(),
                                        [key](const auto& candidate) { return candidate.key == key; });
        if (field == fields.end()) {
            line_error(line_number, "unknown key " + std::string(key));
        }
        const auto index = static_cast<std::size_t>(field - fields.begin());
        if (seen[index]) {
            line_error(line_number, "duplicate key " + std::string(key));
        }
        field->value = parse_number(value, line_number, key);
        seen[index] = true;
    }
    if (!input.eof() || input.bad()) {
        line_error(line_number + 1, "could not be read");
    }
    for (std::size_t index = 0; index < fields.size(); ++index) {
        if (!seen[index]) {
            throw std::runtime_error("case is missing key " + std::string(fields[index].key));
        }
    }
    if (setup.engine.variant.empty() || setup.engine.provenance.empty()) {
        throw std::runtime_error(std::string("case is missing key ") +
                                 (setup.engine.variant.empty() ? "variant" : "provenance"));
    }
    try {
        validate_cycle_setup(setup);
    } catch (const std::invalid_argument& error) {
        throw std::runtime_error(error.what());
    }
    return setup;
}

void write_cycle_setup(std::ostream& output, const CycleSetup& setup) {
    validate_cycle_setup(setup);
    std::ostringstream csv;
    csv.imbue(std::locale::classic());
    csv << std::setprecision(std::numeric_limits<double>::max_digits10) << format << '\n'
        << "variant," << setup.engine.variant << '\n'
        << "provenance," << setup.engine.provenance << '\n';
    for (const auto& field : numeric_fields(setup)) {
        csv << field.key << ',' << field.value << '\n';
    }
    const std::string data = csv.str();
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!output) {
        throw std::runtime_error("case could not be written");
    }
}

}  // namespace turbofan
