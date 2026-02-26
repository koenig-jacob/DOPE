#include "bce/cant.h"
#include "gtest/gtest.h"

TEST(CantTest, ZeroCant) {
    float cant_angle = 0.0f;
    float expected_offset = 0.0f;
    float actual_offset = DOPE_CalculateCantOffset(cant_angle);
    EXPECT_FLOAT_EQ(expected_offset, actual_offset);
}

TEST(CantTest, PositiveCant) {
    float cant_angle = 5.0f; // Example cant angle in degrees
    float expected_offset = /* expected offset calculation based on cant angle */;
    float actual_offset = DOPE_CalculateCantOffset(cant_angle);
    EXPECT_FLOAT_EQ(expected_offset, actual_offset);
}

TEST(CantTest, NegativeCant) {
    float cant_angle = -5.0f; // Example cant angle in degrees
    float expected_offset = /* expected offset calculation based on cant angle */;
    float actual_offset = DOPE_CalculateCantOffset(cant_angle);
    EXPECT_FLOAT_EQ(expected_offset, actual_offset);
}

TEST(CantTest, ExtremeCant) {
    float cant_angle = 90.0f; // Extreme cant angle
    float expected_offset = /* expected offset calculation based on cant angle */;
    float actual_offset = DOPE_CalculateCantOffset(cant_angle);
    EXPECT_FLOAT_EQ(expected_offset, actual_offset);
}