# Lumped wet cycle and nozzle regimes

The synthetic wet fixture adds prescribed augmentor fuel after the mixer and replaces the equivalent convergent nozzle with a fixed convergent-divergent geometry. The same matching solve closes component flows, mixer entry pressure, nozzle capacity and both shaft powers. [Dry cycle build instructions](dry-cycle.md)

```sh
./build/turbofan-cli wet > wet.csv
```

The station CSV keeps the dry case schema and reports the wet nozzle inlet as the ninth station. Standard error includes the nozzle regime and, where supported, exit static pressure in Pa, static temperature in K and Mach number. Library callers receive those values in the optional `CycleResult::nozzle.exit`. The dry case commands retain equal throat and exit areas and zero augmentor fuel.

## Augmentor assumptions

The wet fixture uses the synthetic design inlet and main fuel command, adds 0.22 kg/s of reheat fuel, increases throat area by 10%, and sets exit area to 1.20 times throat area. The augmentor adds fuel mass and sensible heat using the same energy balance as the main combustor. An effective heat-release efficiency of 0.95 and a heating value of 43 MJ/kg are prescribed fixture values. Total-pressure recovery is `0.98 * exp(-3 * augmentor_fuel_flow / inlet_air_flow)`. This empirical placeholder recovers the dry pressure loss continuously as fuel goes to zero.

A synthetic global fuel-air limit of 0.068 kg/kg accounts for fuel already metered upstream. The calculation assumes complete mixing and uses constant gas properties. Fuel-air bookkeeping can catch an excess above the global limit. Modeling local oxygen, separate core and duct fuel schedules, ignition, flameholding, and thermal choking requires additional physics and data. The fixture values are provided for software verification.

## Nozzle closure

The ideal model solves the area–Mach relation for subsonic or supersonic flow. For a divergent nozzle, the pressure at the exit on the subsonic branch with a sonic throat sets the onset of choking. Choked flow capacity is set by the throat. The pressure downstream of a normal shock located at the exit sets the boundary for supported supersonic internal flow.

| Condition | Capacity | Exit diagnostic |
| --- | --- | --- |
| Back pressure at or above the inlet total pressure | Zero forward capacity | Absent |
| Unchoked flow, including the divergent subsonic exit at onset of choking | Matched to exit pressure and area | Subsonic exit |
| Equal exit and throat areas with a choked throat | Sonic capacity | Mach 1 exit |
| Divergent nozzle with a normal shock inside | Sonic capacity | Absent with `unsupported-internal-shock` |
| Divergent nozzle with supersonic flow through the exit | Sonic capacity | Isentropic supersonic exit on the upstream side of a shock at the boundary |

Choked mass capacity remains useful for gas-path matching when the modeled exit state is unavailable. The model omits shock position and post-shock profiles, flow separation, and external expansion.

## Verification

The test suites check heat and mass addition, the dry limit, fuel allocation, nozzle regime boundaries and the integrated wet balances. The built-in wet fixture took three Newton steps in the verified build. The maximum absolute scaled residual was about `5.1e-9`. Reheat raised total temperature from about 712.39 K at the mixer exit to 806.90 K at the nozzle inlet. The ideal nozzle had a computed exit Mach of about 1.534 and exit static temperature of about 548.64 K. These results are software verification cases. PW-100 performance comparisons require variant-specific calibration and test data.
