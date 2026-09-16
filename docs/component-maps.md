# Component map datasets

The solver can load rectilinear fan, high-pressure compressor, and high- and low-pressure turbine maps from a single versioned CSV file. The built-in synthetic maps use the same interpolation and correction code.

## Run with a dataset

Build the executable using the [cycle build instructions](dry-cycle.md). Export the built-in dataset, then feed it back into the solver or use it as a template for a converted dataset.

```sh
./build/turbofan-cli export-maps > synthetic-maps.csv
./build/turbofan-cli design --maps synthetic-maps.csv > design.csv
./build/turbofan-cli off-design --maps synthetic-maps.csv > off-design.csv
./build/turbofan-cli wet --maps synthetic-maps.csv > wet.csv
```

With `--maps`, the CLI reports the input variant and provenance on standard error and labels the results `external-maps-synthetic-geometry` in the station CSV. It uses the selected synthetic case's boundary conditions, flow areas, losses and initial operating point. Those assumptions can make a valid map file incompatible with the case. A rejected initial map query names the component, corrected speed, and coordinate. To supply geometry, operating conditions and initial guesses as well, use [configured engine cases](engine-cases.md). Variant calibration still requires independent validation data.

## Version 1 format

Use the following six lines, in order, before the table rows. Reference values are illustrative. The correction conditions must match the source tables.

```text
turbofan-map-v1
variant,Synthetic example
provenance,Original report and table identification
reference_pressure_pa,101325
reference_temperature_k,288.15
component,corrected_speed_rpm,coordinate,corrected_flow_kg_s,pressure_ratio,efficiency
```

- Supply a rectangular grid with at least two distinct values per axis for each of `fan`, `compressor`, `hp_turbine` and `lp_turbine`. Every speed-coordinate pair must have exactly one row. Input row order is arbitrary.
- Use unquoted fields and decimal points. Variant and provenance must be nonempty and cannot contain commas, quotes or line breaks. Include source revision, table or page, and preprocessing and rights information where applicable. Efficiency is a fraction in `(0, 1]` and corrected flow must be positive.
- For fan and compressor, the coordinate locates a point along a speed line and pressure ratio is outlet total pressure over inlet total pressure. The current fixture starts both map coordinates at 1. A converted compressor coordinate must place that seed within the supplied axis before the case can start.
- For each turbine, the coordinate and pressure-ratio column both give expansion ratio, inlet total pressure over outlet total pressure. At loading, a discrepancy beyond floating-point roundoff is rejected.
- All maps share the declared reference pressure and temperature. Calculate `theta = Tt_in / reference_temperature_k` and `delta = Pt_in / reference_pressure_pa` using each component's own inlet totals. The solver queries `corrected_speed_rpm = mechanical_rpm / sqrt(theta)` and compares `corrected_flow_kg_s = mass_flow_kg_s * sqrt(theta) / delta` against the mapped flow.
- Lookup is bilinear. It includes the boundary nodes and rejects extrapolation. This format presently covers fully populated 2-D grids. Masked cells, nonrectangular speed lines, surge limits and geometry-dependent tables require an explicit extension.

Verification covers out-of-order valid grids, nodes and interstitial interpolation, malformed or incomplete tables, full-precision export and reload, equivalent map corrections under different reference conditions, and changes to the actual cycle solution when a map is modified.

The NASA NTRS search for PW-100 component tables remains blocked by the runtime network proxy. No external PW-100 table was included.
