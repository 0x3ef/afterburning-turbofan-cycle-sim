# Transient simulation and reference control

The simulator can drive the matched gas path with a prescribed command sequence or with a synthetic sampled controller and rate-limited actuators. Both paths initialize from a converged steady operating point.

```sh
./build/turbofan-cli transient > fuel-step.csv
./build/turbofan-cli controlled > controlled.csv
./build/turbofan-cli benchmark

./build/turbofan-cli export-case wet > wet-case.csv
./build/turbofan-cli export-maps > maps.csv
./build/turbofan-cli transient wet-case.csv maps.csv > wet-fuel-step.csv
```

The built-in demos simulate one second in 10 ms steps. The open-loop demo increases main fuel by 0.6% at 0.1 s while holding the other inputs. In the closed-loop demo, normalized throttle goes from 0.5 to 0.75 at 0.1 s. The CSV identifies the variant and includes nine station rows per sampled time. Each sample includes total station states, both rotor speeds and net powers, and the fuel flows and geometry applied over the preceding step. At time zero it records the steady initial commands. The library accepts an arbitrary prescribed-command callback through `simulate_transient`.

## Time integration

At any instantaneous rotor speed, the gas path solves six algebraic balances for airflow, split, map operating points, mixer pressure and nozzle capacity. Net shaft powers drive the two speed derivatives via `J * omega * d(omega)/dt = P_net`. The predictor and corrector evaluate the gas path at their respective fixed speeds, hold the sampled actuator command during the integration interval, and match again at the accepted speeds for output. The previously accepted solution supplies the next matching seed. The solver stops at an invalid or unconverged match.

The dynamics are quasi-steady in the gas path. This milestone models rotor inertia and actuator lag. It leaves out pressure-volume storage, low-speed starter dynamics, surge and combustion transients.

## Reference controller

The controller interpolates a synthetic schedule for fan speed, mixed-stream pressure ratio and augmentor fuel based on normalized throttle. Main fuel tracks fan speed. Nozzle throat area tracks the pressure ratio of mixer outlet total pressure to inlet total pressure. This signal is a plant proxy. The historical DEEC estimator has its own sensor locations and correlations.

Each PI loop initializes from the trim command, limits its output and prevents integration farther into saturation. The actuator model applies lag and per-channel slew limits to main fuel, augmentor fuel and throat area. Exit area follows a fixed area ratio. The public controller and actuator APIs allow callers to supply their own schedules, limits and parameters. The built-in closed-loop demonstration remains dry. The prescribed-input mode can also run wet. Engine-specific gains, schedules and protection laws require separate evidence. [DEEC evidence and validation](deec-evidence-and-validation.md)

## Verification and timing

CTest covers steady equilibrium, an imposed fuel step, timestep refinement, fixed-speed flow closure, controller trim and limits, actuator lag and rate limits, and the integrated closed-loop demonstration. The command-line runs also exercised the wet case loaded from case and map files.

A Release build in the cloud environment simulated the 100-step closed-loop case in about 21 ms on average over ten timed runs, about 47 simulated seconds per wall-clock second. The benchmark includes initialization and numerical integration and omits CSV serialization. It measures throughput for this workload. The fixed-step implementation provides no deadline or worst-case guarantee on other workloads or hosts.
