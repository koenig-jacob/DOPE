#include "gtest/gtest.h"
#include "bce/coriolis.h"

TEST(CoriolisTest, CalculateCoriolisEffect) {
    // Test parameters
    float latitude = 45.0; // Example latitude in degrees
    float velocity = 300.0; // Example velocity in m/s

    // Expected result (calculated manually or from a reliable source)
    float expected_effect = /* expected value based on the formula */;

    // Call the function to test
    float coriolis_effect = CalculateCoriolisEffect(latitude, velocity);

    // Check if the result is as expected
    EXPECT_NEAR(coriolis_effect, expected_effect, 0.01); // Allow a small margin of error
}

TEST(CoriolisTest, HandleInvalidLatitude) {
    float latitude = 100.0; // Invalid latitude
    float velocity = 300.0;

    // Call the function to test
    float coriolis_effect = CalculateCoriolisEffect(latitude, velocity);

    // Check if the function handles invalid latitude gracefully
    EXPECT_EQ(coriolis_effect, 0.0); // Assuming the function returns 0 for invalid input
}

TEST(CoriolisTest, HandleZeroVelocity) {
    float latitude = 45.0; // Valid latitude
    float velocity = 0.0; // Zero velocity

    // Call the function to test
    float coriolis_effect = CalculateCoriolisEffect(latitude, velocity);

    // Check if the effect is zero when velocity is zero
    EXPECT_EQ(coriolis_effect, 0.0);
}