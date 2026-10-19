#include "gazebo_sim_vrpn_bridge/capture_pacing.h"

#include <cstdint>
#include <gtest/gtest.h>

using gazebo_sim_vrpn_bridge::SimulationCapturePacer;

namespace {

constexpr std::int64_t kMs = 1000000;

// Steps a fake simulation clock in step_ns increments and counts captures.
int capturesOver(SimulationCapturePacer& pacer, double rate_hz, std::int64_t step_ns, std::int64_t duration_ns,
                 std::int64_t start_ns = 0) {
    int captures = 0;
    for (std::int64_t t = start_ns; t < start_ns + duration_ns; t += step_ns) {
        if (pacer.due(t, rate_hz)) ++captures;
    }
    return captures;
}

} // namespace

TEST(CapturePacing, CapturesAtConfiguredRateInSimulationTime) {
    SimulationCapturePacer pacer;
    EXPECT_EQ(capturesOver(pacer, 125.0, kMs, 1000 * kMs), 125);
}

TEST(CapturePacing, RateIsIndependentOfWallClockPace) {
    // The pacer sees only simulation time, so a slow real-time factor (few
    // steps per wall second) cannot raise the simulation-domain report rate.
    SimulationCapturePacer fast;
    SimulationCapturePacer slow;
    EXPECT_EQ(capturesOver(fast, 125.0, kMs, 2000 * kMs), capturesOver(slow, 125.0, kMs, 2000 * kMs));
}

TEST(CapturePacing, NeverCapturesTwiceForOneSimulationInstant) {
    SimulationCapturePacer pacer;
    EXPECT_TRUE(pacer.due(100 * kMs, 125.0));
    EXPECT_FALSE(pacer.due(100 * kMs, 125.0)); // paused world: UpdateEnd repeats the same time
    EXPECT_FALSE(pacer.due(100 * kMs, 125.0));
    EXPECT_FALSE(pacer.due(107 * kMs, 125.0));
    EXPECT_TRUE(pacer.due(108 * kMs, 125.0));
}

TEST(CapturePacing, CapturesAreAtLeastOnePeriodApart) {
    SimulationCapturePacer pacer;
    std::int64_t previous = -1;
    for (std::int64_t t = 0; t < 1000 * kMs; t += kMs) {
        if (!pacer.due(t, 125.0)) continue;
        if (previous >= 0) EXPECT_GE(t - previous, 8 * kMs);
        previous = t;
    }
}

TEST(CapturePacing, CoarseStepsCaptureEveryStepWithoutBacklog) {
    // Steps longer than the period capture once per step and do not burst to
    // catch up afterwards.
    SimulationCapturePacer pacer;
    EXPECT_EQ(capturesOver(pacer, 250.0, 20 * kMs, 200 * kMs), 10);
}

TEST(CapturePacing, NonDividingRateKeepsAverageRate) {
    SimulationCapturePacer pacer;
    const int captures = capturesOver(pacer, 120.0, kMs, 10000 * kMs);
    EXPECT_NEAR(captures, 1200, 2);
}

TEST(CapturePacing, SimulationTimeMovingBackwardsRestartsTheGrid) {
    SimulationCapturePacer pacer;
    EXPECT_TRUE(pacer.due(5000 * kMs, 100.0));
    EXPECT_FALSE(pacer.due(5001 * kMs, 100.0));
    EXPECT_TRUE(pacer.due(0, 100.0)); // world time reset
    EXPECT_FALSE(pacer.due(5 * kMs, 100.0));
    EXPECT_TRUE(pacer.due(10 * kMs, 100.0));
}

TEST(CapturePacing, ResetCapturesImmediately) {
    SimulationCapturePacer pacer;
    EXPECT_TRUE(pacer.due(0, 10.0));
    EXPECT_FALSE(pacer.due(1 * kMs, 10.0));
    pacer.reset();
    EXPECT_TRUE(pacer.due(1 * kMs, 10.0));
}

TEST(CapturePacing, LiveRateChangeAppliesFromNextCapture) {
    SimulationCapturePacer pacer;
    EXPECT_TRUE(pacer.due(0, 100.0));
    EXPECT_TRUE(pacer.due(10 * kMs, 1000.0));
    EXPECT_FALSE(pacer.due(10 * kMs, 1000.0));
    EXPECT_TRUE(pacer.due(11 * kMs, 1000.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
