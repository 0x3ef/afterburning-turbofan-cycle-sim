#pragma once

#include <vector>

namespace turbofan {

struct ControllerFeedback {
    double lp_speed_rpm;
    // Mixed-stream outlet total pressure divided by inlet total pressure.
    double pressure_ratio;
};

struct ControllerCommands {
    double main_fuel_flow;       // kg/s
    double augmentor_fuel_flow;  // kg/s
    double throat_area;          // m²
    double exit_area;            // m²
};

struct ControllerSchedulePoint {
    double throttle;             // Normalized to [0, 1].
    double lp_speed_rpm;
    double pressure_ratio;
    double augmentor_fuel_flow;  // kg/s
};

struct PiSettings {
    double proportional_gain;  // Command units per unit of error.
    double integral_gain;      // Command units per unit of error per second.
    double minimum;
    double maximum;
};

struct ControllerConfig {
    // At least two increasing throttle knots with endpoints 0 and 1.
    std::vector<ControllerSchedulePoint> schedule;
    PiSettings fuel;
    PiSettings nozzle;
    double exit_to_throat_ratio;
};

enum class CommandLimit { none, minimum, maximum };

struct ControllerSample {
    ControllerCommands commands;
    ControllerSchedulePoint targets;
    CommandLimit fuel_limit;
    CommandLimit nozzle_limit;
};

// Synthetic reference controller for simulation. Caller supplies all schedule values and gains.
// Fuel error is target minus measured speed. Nozzle error is measured minus target pressure ratio.
class ReferenceController {
public:
    // The first sample at the trim throttle and feedback reproduces the trim commands
    // within floating-point roundoff. The augmentor schedule and area ratio must agree
    // with the given trim.
    ReferenceController(ControllerConfig config, double trim_throttle,
                        ControllerFeedback trim_feedback, ControllerCommands trim_commands);

    // Sample commands at the current feedback, then advance the integrators by dt_seconds.
    ControllerSample step(double throttle, ControllerFeedback feedback, double dt_seconds);

private:
    ControllerSchedulePoint targets_at(double throttle) const;

    ControllerConfig config_;
    double trim_fuel_error_{};
    double trim_nozzle_error_{};
    double fuel_bias_;
    double nozzle_bias_;
};

}  // namespace turbofan
