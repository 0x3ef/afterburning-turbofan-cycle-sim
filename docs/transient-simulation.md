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

Each PI loop initializes from the trim command, limits its output and prevents integration farther into saturation. The actuator model applies lag and per-channel slew limits to main fuel, augmentor fuel and throat area. Exit area follows a fixed area ratio. To supply schedules, limits and actuator settings through files, use [configurable reference control profiles](control-profiles.md). The library exposes the same closed-loop runner as `simulate_controlled`. The built-in closed-loop demonstration is dry. The prescribed-input mode and a compatible control profile can also run a wet case. Engine-specific gains, schedules and protection laws require separate evidence. [DEEC evidence and validation](deec-evidence-and-validation.md)

## Verification and timing

CTest covers steady equilibrium, an imposed fuel step, timestep refinement, fixed-speed flow closure, controller trim and limits, actuator lag and rate limits, and the integrated closed-loop demonstration. A wet control-profile test verifies the augmentor lag and checks matching after a reheat demand change. Command-line verification exported and reloaded the case, maps, and controller, producing an identical closed-loop trajectory.

The benchmark runs both synthetic dry and wet closed-loop profiles. It reports initialization separately and measures each accepted step from command sampling through the final flow match and history insertion. It reports nearest-rank p50, p95 and p99 latencies, the worst observed step, and the number exceeding the profile's step duration. The benchmark excludes CSV serialization and does not pace itself to wall time.

```sh
./build/turbofan-cli benchmark
./build/turbofan-cli benchmark case.csv maps.csv control.csv
```

In one Release run in the cloud environment, the simulator was pinned to a single logical CPU. The contention run used the same affinity with one additional CPU-bound process. Each row contains 1,000 measured steps with a 10 ms deadline.

| Workload | CPU condition | p95 (ms) | p99 (ms) | Maximum (ms) | Steps over 10 ms |
| --- | --- | ---: | ---: | ---: | ---: |
| Dry | Baseline | 0.658 | 1.390 | 2.562 | 0 |
| Wet | Baseline | 0.619 | 2.031 | 3.316 | 0 |
| Dry | Competing process | 3.038 | 6.473 | 10.918 | 1 |
| Wet | Competing process | 4.217 | 5.500 | 10.375 | 1 |

These are host-specific measurements. The observed misses under contention preclude a hard real-time deadline guarantee for this environment.
