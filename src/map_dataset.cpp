#include "turbofan/map_dataset.hpp"

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
#include <utility>
#include <vector>

namespace turbofan {
namespace {

constexpr std::string_view magic = "turbofan-map-v1";
constexpr std::array<std::string_view, 6> column_names{
    "component", "corrected_speed_rpm", "coordinate", "corrected_flow_kg_s",
    "pressure_ratio", "efficiency"
};
constexpr std::array<std::string_view, 4> component_names{
    "fan", "compressor", "hp_turbine", "lp_turbine"
};

[[noreturn]] void input_error(std::size_t line, const std::string& message) {
    throw std::runtime_error("map dataset line " + std::to_string(line) + " " + message);
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

template <std::size_t Count>
std::array<std::string_view, Count> fields(std::string_view line, std::size_t line_number) {
    if (line.find_first_of("\"\r\n") != std::string_view::npos) {
        input_error(line_number, "contains a quote or line break in an unquoted field");
    }

    std::array<std::string_view, Count> result;
    for (std::size_t index = 0; index < Count; ++index) {
        const auto separator = line.find(',');
        const bool last = index + 1 == Count;
        if (last != (separator == std::string_view::npos)) {
            input_error(line_number, "expected " + std::to_string(Count) + " CSV fields");
        }
        result[index] = trim(line.substr(0, separator));
        if (!last) {
            line.remove_prefix(separator + 1);
        }
    }
    return result;
}

std::string_view required_line(std::istream& input, std::string& buffer, std::size_t line_number) {
    if (!std::getline(input, buffer)) {
        input_error(line_number, input.eof() && !input.bad() ? "is missing" : "could not be read");
    }
    return trim(buffer);
}

std::string_view metadata(std::istream& input, std::string& buffer, std::size_t line_number,
                          std::string_view key) {
    const auto pair = fields<2>(required_line(input, buffer, line_number), line_number);
    if (pair[0] != key || pair[1].empty()) {
        input_error(line_number, "expected " + std::string(key) + " and a nonempty value");
    }
    return pair[1];
}

double number(std::string_view field, std::size_t line_number, std::string_view label) {
    const bool has_plus = field.starts_with('+');
    if (has_plus) {
        field.remove_prefix(1);
    }
    if (field.empty() || (has_plus && field.starts_with('-'))) {
        input_error(line_number, "expected a finite number for " + std::string(label));
    }

    double value = 0.0;
    const auto end = field.data() + field.size();
    const auto result = std::from_chars(field.data(), end, value);
    if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(value)) {
        input_error(line_number, "expected a finite number for " + std::string(label));
    }
    return value;
}

bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool valid_turbine_sample(const MapSample& sample) {
    const double ratio = sample.point.pressure_ratio;
    const double coordinate = sample.coordinate;
    const double tolerance = 8.0 * std::numeric_limits<double>::epsilon() *
                             std::max(ratio, coordinate);
    return ratio >= 1.0 && coordinate >= 1.0 && std::abs(ratio - coordinate) <= tolerance;
}

bool turbine(std::string_view component) {
    return component == "hp_turbine" || component == "lp_turbine";
}

PerformanceMap make_map(std::vector<MapSample> samples, std::string_view name) {
    try {
        return PerformanceMap(std::move(samples));
    } catch (const std::invalid_argument& error) {
        throw std::runtime_error("map dataset " + std::string(name) + " map " + error.what());
    }
}

void write_metadata(std::ostream& output, std::string_view key, std::string_view value) {
    if (value.empty() || value != trim(value) ||
        value.find_first_of(",\"\r\n") != std::string_view::npos) {
        throw std::runtime_error("map dataset " + std::string(key) +
                                 " must be nonempty unquoted CSV text without surrounding spaces");
    }
    output << key << ',' << value << '\n';
}

}  // namespace

MapDataset load_map_dataset(std::istream& input) {
    std::string line;
    if (required_line(input, line, 1) != magic) {
        input_error(1, "expected " + std::string(magic));
    }
    std::string variant(metadata(input, line, 2, "variant"));
    std::string provenance(metadata(input, line, 3, "provenance"));
    const double pressure = number(metadata(input, line, 4, "reference_pressure_pa"), 4,
                                   "reference_pressure_pa");
    const double temperature = number(metadata(input, line, 5, "reference_temperature_k"), 5,
                                      "reference_temperature_k");
    if (!positive(pressure)) {
        input_error(4, "reference_pressure_pa must be positive");
    }
    if (!positive(temperature)) {
        input_error(5, "reference_temperature_k must be positive");
    }
    if (fields<6>(required_line(input, line, 6), 6) != column_names) {
        input_error(6, "contains unexpected map columns");
    }

    std::array<std::vector<MapSample>, 4> samples;
    std::size_t line_number = 6;
    while (std::getline(input, line)) {
        ++line_number;
        const auto row = fields<6>(trim(line), line_number);
        const auto component = std::find(component_names.begin(), component_names.end(), row[0]);
        if (component == component_names.end()) {
            input_error(line_number, "unknown component " + std::string(row[0]));
        }
        MapSample sample{
            number(row[1], line_number, column_names[1]),
            number(row[2], line_number, column_names[2]),
            {number(row[3], line_number, column_names[3]),
             number(row[4], line_number, column_names[4]),
             number(row[5], line_number, column_names[5])}
        };
        if (turbine(*component)) {
            if (!valid_turbine_sample(sample)) {
                input_error(line_number,
                            "turbine coordinate must equal its expansion ratio of at least one");
            }
            sample.point.pressure_ratio = sample.coordinate;
        }
        const auto index = static_cast<std::size_t>(component - component_names.begin());
        samples[index].push_back(sample);
    }
    if (!input.eof() || input.bad()) {
        input_error(line_number + 1, "could not be read");
    }

    return {std::move(variant), std::move(provenance), pressure, temperature,
            make_map(std::move(samples[0]), component_names[0]),
            make_map(std::move(samples[1]), component_names[1]),
            make_map(std::move(samples[2]), component_names[2]),
            make_map(std::move(samples[3]), component_names[3])};
}

void write_map_dataset(std::ostream& output, const MapDataset& dataset) {
    if (!positive(dataset.reference_pressure) || !positive(dataset.reference_temperature)) {
        throw std::runtime_error("map dataset reference pressure and temperature must be "
                                 "positive and finite");
    }

    std::ostringstream csv;
    csv.imbue(std::locale::classic());
    csv << std::setprecision(std::numeric_limits<double>::max_digits10) << magic << '\n';
    write_metadata(csv, "variant", dataset.variant);
    write_metadata(csv, "provenance", dataset.provenance);
    csv << "reference_pressure_pa," << dataset.reference_pressure << '\n'
        << "reference_temperature_k," << dataset.reference_temperature << '\n';
    csv << column_names.front();
    for (std::size_t index = 1; index < column_names.size(); ++index) {
        csv << ',' << column_names[index];
    }
    csv << '\n';

    const std::array<const PerformanceMap*, 4> maps{
        &dataset.fan, &dataset.compressor, &dataset.hp_turbine, &dataset.lp_turbine
    };
    for (std::size_t index = 0; index < maps.size(); ++index) {
        const auto component = component_names[index];
        for (const auto& sample : maps[index]->samples()) {
            if (turbine(component) && !valid_turbine_sample(sample)) {
                throw std::runtime_error("map dataset " + std::string(component) +
                                         " coordinate must equal its expansion ratio of at least one");
            }
            const double ratio = turbine(component) ? sample.coordinate : sample.point.pressure_ratio;
            csv << component << ',' << sample.corrected_speed << ',' << sample.coordinate << ','
                << sample.point.corrected_flow << ',' << ratio << ',' << sample.point.efficiency
                << '\n';
        }
    }
    const std::string data = csv.str();
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!output) {
        throw std::runtime_error("map dataset could not be written");
    }
}

}  // namespace turbofan
