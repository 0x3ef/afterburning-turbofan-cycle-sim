#include "turbofan/map_dataset.hpp"

#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

constexpr std::string_view fixture = R"(turbofan-map-v1
variant, test engine
provenance, analytic test fixture
reference_pressure_pa,+101325.12345678901
reference_temperature_k,288.15
component,corrected_speed_rpm,coordinate,corrected_flow_kg_s,pressure_ratio,efficiency
fan,10000,0,10.123456789012345,1.6,0.8
fan,20000,1,22,2.5,0.83
fan,20000,0,20,2.6,0.82
fan,10000,1,12,1.5,0.81
compressor,20000,1,32,6.0,0.86
compressor,10000,-1,13,4.0,0.82
compressor,20000,-1,29,6.5,0.84
compressor,10000,1,15,3.5,0.83
hp_turbine,12000,1.5,16,1.5000000000000002,0.80
hp_turbine,12000,2.5,17,2.5,0.81
hp_turbine,22000,1.5,18,1.5,0.82
hp_turbine,22000,2.5,19,2.5,0.83
lp_turbine,8500,1.25,18,1.25,0.84
lp_turbine,8500,2.25,19,2.25,0.85
lp_turbine,14500,1.25,20,1.25,0.86
lp_turbine,14500,2.25,21,2.25,0.87
)";

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

turbofan::MapDataset parse(const std::string& value) {
    std::istringstream input(value);
    return turbofan::load_map_dataset(input);
}

void reject(const std::string& value, std::string_view diagnostic) {
    try {
        parse(value);
    } catch (const std::runtime_error& error) {
        require(std::string_view(error.what()).find(diagnostic) != std::string_view::npos,
                error.what());
        return;
    }
    throw std::runtime_error("invalid map data were accepted");
}

std::string replace(std::string_view before, std::string_view after) {
    std::string result(fixture);
    const auto offset = result.find(before);
    require(offset != std::string::npos, "test row was missing from the fixture");
    result.replace(offset, before.size(), after);
    return result;
}

void test_parse() {
    std::string crlf;
    for (const char c : fixture) {
        if (c == '\n') {
            crlf += '\r';
        }
        crlf += c;
    }
    const auto dataset = parse(crlf);
    require(dataset.variant == "test engine" && dataset.provenance == "analytic test fixture",
            "whitespace around metadata should be trimmed");
    require(dataset.reference_pressure == 101325.12345678901 &&
                dataset.reference_temperature == 288.15,
            "numeric reference conditions were not loaded");
    require(dataset.fan.evaluate(20000.0, 1.0).corrected_flow == 22.0,
            "fan map should support out-of-order rows");
    require(dataset.compressor.evaluate(15000.0, 0.0).corrected_flow == 22.25,
            "compressor map should interpolate sample flows");
    require(dataset.hp_turbine.evaluate(12000.0, 1.5).pressure_ratio == 1.5,
            "roundoff-level ratio differences should be normalized to the turbine coordinate");
    require(dataset.lp_turbine.evaluate(14500.0, 2.25).efficiency == 0.87,
            "last turbine row was not loaded");
    require(parse(std::string(fixture.substr(0, fixture.size() - 1))).variant == dataset.variant,
            "last row without a final newline should be valid");
}

void test_invalid_csv() {
    reject(replace("turbofan-map-v1", "turbofan-map-v2"), "line 1");
    reject(replace("variant, test engine", "variant,  "), "line 2");
    reject(replace("variant, test engine", "variant,\"test engine\""), "line 2");
    reject(replace("provenance, analytic test fixture", "provenance, part 1, part 2"),
           "line 3");
    reject(replace("+101325.12345678901", "0"), "line 4");
    reject(replace("288.15", "inf"), "line 5");
    reject(replace("corrected_flow_kg_s", "flow"), "line 6");
    reject(replace("fan,10000,0", "unexpected,10000,0"), "line 7");
    reject(replace("10.123456789012345", "NaN"), "line 7");
    reject(replace("10.123456789012345", "10kg"), "line 7");
    reject(replace("10.123456789012345", "+-10"), "line 7");
    reject(replace("0.8\nfan", "0.8,\nfan"), "line 7");
    reject(replace("1.5000000000000002", "1.6"), "line 15");
    reject(replace("1.25,18,1.25", "0.75,18,0.75"), "line 19");
    reject(std::string(fixture) + "variant,extra\n", "line 23");
    reject(std::string(fixture) + '\n', "line 23");
    reject(replace("fan,10000,1,12,1.5,0.81\n", ""), "fan map");
    reject(std::string(fixture) + "fan,10000,0,10,1.6,0.8\n", "fan map");
    reject(replace("0.87", "1.1"), "lp_turbine map");

    const auto last_map = fixture.find("lp_turbine,");
    reject(std::string(fixture.substr(0, last_map)), "lp_turbine map");
}

class DecimalComma : public std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

void test_export() {
    auto dataset = parse(std::string(fixture));
    std::ostringstream output;
    output.imbue(std::locale(std::locale::classic(), new DecimalComma));
    output << std::fixed << std::setprecision(2) << std::setw(40);
    turbofan::write_map_dataset(output, dataset);
    require(output.str().starts_with("turbofan-map-v1\n"),
            "serialization should ignore the destination's formatting flags");
    const auto imported = parse(output.str());
    require(imported.reference_pressure == dataset.reference_pressure &&
                imported.reference_temperature == dataset.reference_temperature,
            "reference values should round-trip with full precision");
    require(imported.variant == dataset.variant && imported.provenance == dataset.provenance,
            "metadata should round-trip");
    require(imported.fan.evaluate(10000.0, 0.0).corrected_flow == 10.123456789012345,
            "sample values should round-trip with full precision");
    require(imported.lp_turbine.evaluate(14500.0, 2.25).efficiency == 0.87,
            "exports should include all four maps");

    std::ostringstream refused;
    dataset.provenance = "unsupported, comma";
    try {
        turbofan::write_map_dataset(refused, dataset);
    } catch (const std::runtime_error& error) {
        require(std::string_view(error.what()).find("provenance") != std::string_view::npos &&
                    refused.str().empty(),
                "invalid metadata must be reported before output is written");
        return;
    }
    throw std::runtime_error("export accepted metadata outside the unquoted CSV contract");
}

}  // namespace

int main() {
    try {
        test_parse();
        test_invalid_csv();
        test_export();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
