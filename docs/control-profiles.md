# Configurable reference control profiles

The closed-loop runner accepts a case, a matching component-map dataset, and a versioned control profile. The profile provides the schedule, controller, and actuator data independently of the thermodynamic model. Generate a runnable set of synthetic inputs with these commands.

```sh
./build/turbofan-cli export-case design > case.csv
./build/turbofan-cli export-maps > maps.csv
./build/turbofan-cli export-control > control.csv
./build/turbofan-cli controlled case.csv maps.csv control.csv > trajectory.csv
```

The built-in control profile is linked to the synthetic design trim. For a different case, the trim fuel and throat area must fit within the profile limits. Its exit-to-throat ratio and trim augmentor fuel must agree with the initial case. The controller initializes from the converged station states, so the first command reproduces the trim. The engine and map variant identifiers must still match. [Configured engine cases](engine-cases.md)

## Version 1 format

Use `turbofan-control-v1` on the first line. Each singleton row below must be present exactly once. Include at least two `schedule` rows ordered by throttle from 0 to 1, and at least one `demand` row ordered by time. Exported numerical values carry full precision.

| Row | Values after the row name |
| --- | --- |
| `timing` | Sample period in seconds, number of integration steps, normalized trim throttle |
| `fuel_pi` | Proportional gain, integral gain, minimum and maximum main fuel in kg/s |
| `nozzle_pi` | Proportional gain, integral gain, minimum and maximum throat area in m² |
| `fuel_actuator` | First-order time constant in seconds and fuel slew limit in kg/s per second |
| `augmentor_actuator` | First-order time constant in seconds and augmentor fuel slew limit in kg/s per second |
| `throat_actuator` | First-order time constant in seconds and throat area slew limit in m² per second |
| `area_ratio` | Exit area divided by throat area |
| `schedule` | Normalized throttle, LP speed target in rpm, pressure ratio target, augmentor fuel setting in kg/s |
| `demand` | Time in seconds and normalized throttle to hold from that time onward |

The first demand row starts at zero with the trim throttle. Both PI loops sample once at the start of each integration interval. A demand change between sample instants takes effect at the next sample. The plant holds achieved actuator values across the integration interval. The profile sets the timestep for both the plant and the reference controller. Larger time steps, aggressive gains, or commands outside the supported map domain can make a run fail.

The speed-loop gains use kg/s per rpm and kg/s per rpm per second. The nozzle-loop gains use m² per pressure-ratio error and m² per pressure-ratio error per second. This reference pressure ratio is the mixed-stream total pressure divided by the inlet total pressure. The file represents a simulation reference controller. The published DEEC schedules, transient limit priorities, sensor emulation, and variant-specific settings require independent data. [Transient simulation](transient-simulation.md)
