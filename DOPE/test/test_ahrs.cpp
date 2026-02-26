#include "bce/ahrs.h"
#include "gtest/gtest.h"

class AHRS_Test : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize AHRS system or any required setup
    }

    void TearDown() override {
        // Clean up after tests
    }
};

TEST_F(AHRS_Test, TestQuaternionNormalization) {
    // Example test for quaternion normalization
    float q[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    NormalizeQuaternion(q);
    // Check if the quaternion is normalized
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    EXPECT_NEAR(norm, 1.0f, 1e-6);
}

TEST_F(AHRS_Test, TestAHRSUpdate) {
    // Example test for AHRS update function
    SensorFrame frame;
    // Populate frame with test data
    UpdateAHRS(frame);
    // Validate AHRS state after update
    // Add assertions based on expected AHRS state
}

TEST_F(AHRS_Test, TestBiasCalibration) {
    // Example test for bias calibration
    float accel_bias[3] = {0.1f, 0.2f, 0.3f};
    float gyro_bias[3] = {0.01f, 0.02f, 0.03f};
    DOPE_SetIMUBias(accel_bias, gyro_bias);
    // Validate that biases are set correctly
    // Add assertions based on expected bias values
}