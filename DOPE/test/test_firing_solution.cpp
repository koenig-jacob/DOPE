#include "bce/firing_solution.h"
#include <gtest/gtest.h>

TEST(FiringSolutionTest, DefaultValues) {
    FiringSolution solution;

    EXPECT_EQ(solution.solution_mode, 0);
    EXPECT_EQ(solution.fault_flags, 0);
    EXPECT_EQ(solution.defaults_active, 0);
    EXPECT_FLOAT_EQ(solution.hold_elevation_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.hold_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.range_m, 0.0f);
    EXPECT_FLOAT_EQ(solution.horizontal_range_m, 0.0f);
    EXPECT_FLOAT_EQ(solution.tof_ms, 0.0f);
    EXPECT_FLOAT_EQ(solution.velocity_at_target_ms, 0.0f);
    EXPECT_FLOAT_EQ(solution.energy_at_target_j, 0.0f);
    EXPECT_FLOAT_EQ(solution.coriolis_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.coriolis_elevation_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.spin_drift_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.wind_only_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.earth_spin_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.offsets_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.cant_windage_moa, 0.0f);
    EXPECT_FLOAT_EQ(solution.cant_angle_deg, 0.0f);
    EXPECT_FLOAT_EQ(solution.heading_deg_true, 0.0f);
    EXPECT_FLOAT_EQ(solution.air_density_kgm3, 0.0f);
} 

// Additional tests for FiringSolution can be added here.