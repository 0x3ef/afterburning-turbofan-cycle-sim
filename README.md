# Turbofan Afterburner Simulator

A C++20 simulator for a twin-spool, mixed-flow turbofan with afterburning. It computes steady operating points and rotor transients, with CSV output for pressure, temperature, and mass flow at each station. Engine geometry, component maps, and reference-controller settings can be loaded from files.

The included cases use synthetic maps and a reference controller. NASA reports inform the engine layout and provide a comparison with selected F100-PW-100 flight results. [Comparison and limitations](docs/pw100-public-comparison.md)

## Build and run

Requirements are CMake 3.20 or newer and a C++20 compiler.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure

./build/turbofan-cli design > dry.csv
./build/turbofan-cli wet > wet.csv
./build/turbofan-cli controlled > transient.csv
./build/turbofan-cli compare-pw100 > comparison.csv
```

## Notebook and documentation

The [executed analysis notebook](notebooks/turbofan_analysis.ipynb) includes station plots, T–s and p–v diagrams, an ideal Brayton reference, and archived NASA figures. See the [notebook guide](docs/notebook-analysis.md) to regenerate the plots and export them as PNG or SVG.

- [Engine configuration and cases](docs/engine-cases.md)
- [Component map format](docs/component-maps.md)
- [Transient simulation and benchmark](docs/transient-simulation.md)
- [Controller profiles](docs/control-profiles.md)
