# Public comparison with F100-PW-100 flight results

Run the stationary comparison with the repository's C++ executable.

```sh
./build/turbofan-cli compare-pw100 > comparison.csv
```

## Evidence and conditions

NASA TM 84903 compares flight results from the rebuilt test engine 680063 and the nominal CCD 1148.0 simulation based on the older FX-227 test article. The comparison uses four identifiable flight points from Fig. 9(a) at 40,000 ft and maximum afterburning power. Flight EPR and the report's separate deck EPR were read from the graph. The estimated chart-reading uncertainty is approximately ±0.01 Mach and ±0.04 EPR. Total fuel points come from Fig. 14 with an estimated chart-reading uncertainty of ±250 kg/h. These are reading tolerances, separate from flight instrumentation and historical model uncertainty. Two partly overlapping flight markers at Mach 1.2 and 1.6 and an unclear marker at Mach 1.8 were excluded. [NASA TM 84903, printed pp. 11 and 16](https://ntrs.nasa.gov/citations/19840006076)

The inlet uses the Fig. 6 assumed recovery for the historical deck, `Pt2/Pt0`. The code digitizes that curve from Mach 0 through 2.0 in 0.1 steps, interpolates linearly, and rejects extrapolation. Its estimated chart-reading uncertainty is about ±0.002 absolute, or ±0.003 at the endpoints. This is a published boundary assumption. The ambient state uses a dry standard atmosphere at 12,192 m geopotential, yielding approximately 216.65 K and 18.754 kPa. Actual flight ambient conditions are unavailable for the selected points. [NASA TM 84903, printed p. 9](https://ntrs.nasa.gov/citations/19840006076)

## Method and result

The comparison holds the synthetic wet-cycle maps, geometry and losses. It transfers the converged synthetic trim to the new inlet using the inlet total pressure scale `p_s = Pt2_new / Pt2_trim` and temperature scale `t_s = Tt2_new / Tt2_trim`. Seed airflow scales by `p_s / sqrt(t_s)` and seed speeds by `sqrt(t_s)` to preserve the inlet-based corresponding flow and speed scales. Main and augmentor fuel follow airflow times `t_s` as a first-order thermal scaling assumption. The gas-path solver then re-matches each point. The observed fuel values are printed as context and never used as inputs or treated as model predictions.

The computed EPR proxy is the modeled mixer-exit total pressure divided by modeled station-2 total pressure. The report's EPR uses `PT6M/PT2` from its own sensing and estimation chain. The residual is the arithmetic difference between the proxy and the flight value.

| Mach | Flight EPR | Historical deck EPR | Synthetic proxy | Proxy minus flight |
| ---: | ---: | ---: | ---: | ---: |
| 0.8 | 3.08 | 3.30 | 2.069 | -1.011 |
| 1.0 | 3.00 | 3.17 | 2.070 | -0.930 |
| 1.4 | 2.59 | 2.56 | 2.072 | -0.518 |
| 2.0 | 1.60 | 1.62 | 2.076 | +0.476 |

The synthetic proxy is nearly flat while flight EPR decreases markedly. The residuals are much larger than chart-reading uncertainty. This exercise quantifies a discrepancy but cannot isolate maps, fuel and geometry schedules, cycle assumptions, and instrumentation effects. The available measurements underdetermine all four component characteristics. A fitted EPR curve by itself would leave the internal states and off-design map behavior unverified.

CP 2298 pp. 241–242 documents a simulation architecture that consumes component characteristics. Its inlet distortion maps describe pressure across an inlet section. Neither source supplied a reusable PW-100 map dataset. The current environment's NTRS proxy returned HTTP 403 during the catalog check. No additional candidate component tables were verified. [NASA CP 2298](https://ntrs.nasa.gov/citations/19860015871)
