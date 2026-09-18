# Turbofan analysis notebook

The [analysis notebook](../notebooks/turbofan_analysis.ipynb) runs the C++ simulator, shows its tables and plots next to the commands that produced them, and exports the source CSVs alongside the figures. The committed notebook includes its executed figures for viewing directly. This keeps a case reproducible while you explore inputs or prepare figures for a report.

## Run on Linux

From the repository root, with CMake 3.20 or newer, a C++20 compiler, and Python 3.10 or newer with `venv` installed, run the following commands.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r notebooks/requirements.txt
python -m jupyterlab notebooks/turbofan_analysis.ipynb
```

Select the Python kernel from `.venv`. Use **Run → Restart Kernel and Run All Cells** to regenerate the tables and plots from a clean kernel.

## Set up a case and export results

The setup cell exposes `DRY_COMMAND`, `WET_COMMAND`, and `TRANSIENT_COMMAND`. Set `TRACE_STATION` to choose which station provides the temperature time series. Each tuple holds arguments to `turbofan-cli`. The notebook runs the executable from the repository root, so relative paths in those tuples resolve from there. For example, `("controlled", "case.csv", "maps.csv", "control.csv")` selects a configured closed-loop run. Use `TURBOFAN_CLI` to point to a different build of the executable and `TURBOFAN_ANALYSIS_DIR` to change the export directory. Set these environment variables before starting JupyterLab. [Configured engine cases](engine-cases.md) · [Control profiles](control-profiles.md)

- Run the baseline dry and wet cases to compare station pressure, temperature, and flow. Use the transient plots to examine the synthetic closed-loop response.
- Use the PW-100 comparison to inspect the flight values digitized from NASA TM 84903, the historical deck values, and the simulator's synthetic pressure-ratio proxy. Preserve those labels and the chart-reading uncertainty when sharing the comparison. [PW-100 public comparison](pw100-public-comparison.md)
- Select figures from `build/analysis/` after rerunning the notebook. Use SVG for scalable report figures or PNG for slides. Keep the raw CSVs with the figures so reviewers can reproduce the plotted values. [Transient simulation](transient-simulation.md)


## Thermodynamic diagrams and historical figures

- The synthetic open-cycle T–s and p–v plots follow the core and bypass branches to the nozzle inlet. Entropy and stagnation specific volume come from the same constant-property gas assumption as the C++ simulator. The ideal Brayton plot is a separate closed air-standard reference with analytic compression, expansion, heating and heat rejection.
- Selected panels from HIDEC Figures 7–8 were redrawn from documented chart readings. The nozzle correction from Equation (B46) in NASA TM X-3261 was evaluated directly as a historical curve.
- The complete numbered figure galleries include all 12 HIDEC figures and all 23 figures from the 1975 hybrid simulation report. Their original captions, legends, and axes are retained. Several figures span more than one panel or page. The galleries show the published sources with their original conditions and model variants. [HIDEC figure index](../notebooks/references/hidec/README.md) · [Hybrid simulation figure index](../notebooks/references/nasa-19750019996/README.md)

The 1975 publication provides normalized PW-100 fan, compressor and turbine plots and selected model constants. Its digital listing reads map values and scale factors from external cards, and those cards are absent. The notebook preserves the plotted evidence. The simulator cases and thermodynamic diagrams continue to use the synthetic configuration.
