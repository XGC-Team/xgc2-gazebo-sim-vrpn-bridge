#pragma once

#include <cmath>
#include <cstdint>

namespace gazebo_sim_vrpn_bridge {

// Decides which Gazebo UpdateEnd steps are captured for VRPN reporting. The
// cadence is defined in simulation time, the clock domain of the report
// timestamps, so at most one sample is taken per period of simulated time at any
// real-time factor. A wall-clock cadence would oversample simulation time when
// the real-time factor falls below one and emit distinct poses that share a
// (quantised) downstream timestamp.
class SimulationCapturePacer {
  public:
    // Returns true when the step at simulation_ns should be captured.
    bool due(std::int64_t simulation_ns, double rate_hz) {
        if (have_capture_ && simulation_ns < last_simulation_ns_) {
            reset(); // Simulation time moved backwards (world reset): restart the grid.
        }
        last_simulation_ns_ = simulation_ns;
        if (have_capture_ && simulation_ns < next_capture_ns_) {
            return false;
        }
        const std::int64_t period_ns = periodNs(rate_hz);
        if (!have_capture_) {
            next_capture_ns_ = simulation_ns;
            have_capture_ = true;
        }
        do {
            next_capture_ns_ += period_ns;
        } while (next_capture_ns_ <= simulation_ns);
        return true;
    }

    void reset() {
        have_capture_ = false;
        next_capture_ns_ = 0;
        last_simulation_ns_ = 0;
    }

  private:
    static std::int64_t periodNs(double rate_hz) {
        const auto period = static_cast<std::int64_t>(std::llround(1e9 / rate_hz));
        return period > 0 ? period : 1;
    }

    bool have_capture_{false};
    std::int64_t next_capture_ns_{0};
    std::int64_t last_simulation_ns_{0};
};

} // namespace gazebo_sim_vrpn_bridge
