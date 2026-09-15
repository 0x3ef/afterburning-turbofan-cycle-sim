# Synthetic dry cycle

This first C++20 implementation evaluates a steady mixed-flow turbofan with imposed main fuel flow and nozzle throat area. The output identifies the model as `synthetic-mixed-flow-dry`. The map values and geometry are constructed to exercise the solver. Use the [DEEC evidence review](deec-evidence-and-validation.md) for the separate PW-100 calibration work.

## Build and run

Use CMake 3.20 or newer and a compiler with C++20 support. Run these commands from the repository root. The project has no external C++ dependencies.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/turbofan-cli design > design.csv
./build/turbofan-cli off-design > off-design.csv
```

The CLI writes CSV to standard output and diagnostic information to standard error. For the added reheat case and exit diagnostics, see [the wet cycle guide](wet-cycle.md). The nine station rows report total pressure in Pa, total temperature in K, gas mass flow in kg/s, sensible total enthalpy in J/kg, and the metered fuel-to-air mass ratio. The fuel ratio tracks supplied fuel mass for bookkeeping. It is independent of the calorically perfect gas properties and gives no species prediction. Diagnostics identify each scaled residual, the converged spool speeds and split, and the number of accepted Newton steps. A failed match produces a nonzero exit code and no station CSV.

## Gas path and matching

- The gas path uses a common fan exit, separate core and bypass streams, a main combustor, two turbines, a mixer, a dry augmentor pressure loss and an ideal nozzle throat. It accounts for metered fuel mass and sensible heat. Shaft efficiency and pressure recovery are specified constants.
- The reference case fixes synthetic flow, split, shaft speeds, fan and compressor pressure ratios, efficiencies and main fuel. The fixture derives turbine pressure ratios from shaft power and flow areas from a common mixer static pressure and nozzle flow. The off-design case retains the maps and geometric mixer areas and imposes 6% less main fuel and a 2% larger throat.
- The fixture samples bounded 3-by-3 maps. Bilinear lookup interpolates flow, pressure ratio and efficiency against corrected speed and a normalized map coordinate. Turbine coordinates express expansion ratio relative to the synthetic reference. A trial outside the map or admissible mixer flow is rejected.
- One damped Newton solve matches four corrected component flows, mixer-entry static pressure, nozzle capacity and both shaft powers. The four map residuals are relative to the trial flows. The remaining residuals use sea-level reference pressure, design inlet flow and design shaft loads. The solve requires each absolute scaled residual to be at most `1e-8`.
- The mixer solves for compatible subsonic entry pressures and conserves mass and enthalpy. Its mixed total pressure is modeled as a fixed recovery times the smaller inlet total pressure. The nozzle checks forward flow and choking. It currently reports throat capacity through the matching residual, with no exit expansion calculation.

## Verification

CTest checks independent component relations, including subsonic inversion and nozzle choking. The dry-cycle test checks the design point and a simultaneous off-design match, as well as mass and mixer enthalpy conservation, shaft power, and invalid geometry. The design solution starts at its constructed anchors. The off-design case took three accepted Newton steps in the verified build, with a maximum absolute scaled residual of about `1.4e-10`.

| Synthetic case | Airflow (kg/s) | LP speed (rpm) | HP speed (rpm) | Bypass air fraction |
| --- | ---: | ---: | ---: | ---: |
| Design | 90.000 | 9,000 | 14,000 | 0.4300 |
| Off-design | 90.008 | 8,638 | 13,752 | 0.4532 |

Variant calibration, external map loading, variable gas properties, cooling and bleed streams, and DEEC dynamics remain future steps. The NASA Technical Reports Server returned a connection-proxy 403 during the calibration search. No PW-100 component map tables or CCD 1148.0 data were verified. [NASA TM 84903 catalog entry](https://ntrs.nasa.gov/citations/19840006076)
