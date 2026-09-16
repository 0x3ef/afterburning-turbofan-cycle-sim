#pragma once

#include <iosfwd>
#include <string>

#include "turbofan/performance_map.hpp"

namespace turbofan {

struct MapDataset {
    std::string variant;
    std::string provenance;
    double reference_pressure;     // Pa
    double reference_temperature;  // K
    PerformanceMap fan;
    PerformanceMap compressor;
    PerformanceMap hp_turbine;
    PerformanceMap lp_turbine;
};

// Read or write the versioned, unquoted turbofan-map-v1 CSV format.
MapDataset load_map_dataset(std::istream& input);
void write_map_dataset(std::ostream& output, const MapDataset& dataset);

}  // namespace turbofan
