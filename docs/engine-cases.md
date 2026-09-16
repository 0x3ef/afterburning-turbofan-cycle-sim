# Configured engine cases

The case file supplies the plant geometry, losses, fuel and actuator settings, ambient conditions, and the initial matching point. Its engine variant must match the variant of the component map dataset exactly. The two sources retain independent provenance. A matching variant string checks input consistency. Quantitative calibration requires independent evidence for both files.

## Export, edit, and run

```sh
./build/turbofan-cli export-case wet > engine-case.csv
./build/turbofan-cli export-maps > maps.csv
./build/turbofan-cli solve-case engine-case.csv maps.csv > stations.csv
```

The other presets are `design` and `off-design`. The case starts with `turbofan-case-v1`, followed by unquoted `key,value` lines in any order. Every key in the export is required once. Numerical fields use decimal points and SI units, with rpm for rotational speed. Unknown or repeated keys, incompatible variants, and nonphysical values are rejected. The output includes configured-cycle station rows and reports both source descriptions on standard error.

| Group | Inputs and interpretation |
| --- | --- |
| Identity | `variant` and `provenance` specify the geometry and case source. The map file has separate source metadata. |
| Gas path | Core and bypass mixer areas in m². Bypass, mixer, burner and dry augmentor total-pressure recoveries. Main and augmentor heat-release efficiencies, fuel heating value in J/kg, stoichiometric fuel-air ratio and wet-loss coefficient. |
| Shaft | Mechanical efficiency and positive LP/HP inertias in kg·m². Steady matching uses net shaft power. A fixed-speed match leaves those powers available for rotor dynamics. |
| Operating conditions | Ambient pressure in Pa, temperature in K, Mach and inlet pressure recovery. Selected main and augmentor fuel in kg/s and effective throat/exit areas in m². |
| Initial matching point | Inlet airflow, bypass air fraction, two compressor map coordinates, two turbine expansion ratios and both shaft speeds. Together with the boundary data these must produce a feasible trial for each map and the mixer. |

The iteration adjusts the seed when solving steady conditions. In a fixed-speed solve it adjusts inlet flow, split and the four map coordinates, and holds the two speeds exactly at their seed values. Six scaled flow, mixer-pressure and nozzle-capacity residuals determine convergence in fixed-speed mode. Net powers remain physical outputs. The steady form additionally closes both shaft residuals. Pressure and flow residuals use the ambient pressure and seed flow as scales. Shaft residuals use the seed flow times the ambient perfect-gas enthalpy.

Tests cover case export/import, geometry changing the converged bypass split, variant mismatch, and the fixed-speed solve preserving both speeds while reporting a nonzero shaft power after a fuel change. [Component maps](component-maps.md) · [C++ build instructions](dry-cycle.md)
