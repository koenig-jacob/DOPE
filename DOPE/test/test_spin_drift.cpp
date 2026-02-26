#include "bce/spin_drift.h"
#include <gtest/gtest.h>

TEST(SpinDriftTest, BasicFunctionality) {
    // Test parameters
    float time_of_flight = 1.0f; // seconds
    float twist_direction = 1.0f; // clockwise

    // Expected drift calculation
    float expected_drift = time_of_flight * time_of_flight * time_of_flight; // Placeholder for actual calculation

    // Calculate spin drift
    float calculated_drift = CalculateSpinDrift(time_of_flight, twist_direction);

    // Check if the calculated drift is as expected
    EXPECT_NEAR(calculated_drift, expected_drift, 0.01f);
}

TEST(SpinDriftTest, NegativeTwistDirection) {
    // Test parameters
    float time_of_flight = 1.0f; // seconds
    float twist_direction = -1.0f; // counter-clockwise

    // Expected drift calculation
    float expected_drift = -time_of_flight * time_of_flight * time_of_flight; // Placeholder for actual calculation

    // Calculate spin drift
    float calculated_drift = CalculateSpinDrift(time_of_flight, twist_direction);

    // Check if the calculated drift is as expected
    EXPECT_NEAR(calculated_drift, expected_drift, 0.01f);
}