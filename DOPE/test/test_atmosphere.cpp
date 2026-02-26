#include "gtest/gtest.h"
#include "bce/atmosphere.h"

TEST(AtmosphereTest, PressureAtSeaLevel) {
    float expected_pressure = 101325.0f; // Pa
    float pressure = CalculatePressure(0.0f, 15.0f, 0.5f); // Altitude 0 m, Temperature 15 °C, Humidity 0.5
    EXPECT_NEAR(pressure, expected_pressure, 1.0f); // Allowable error of 1 Pa
}

TEST(AtmosphereTest, DensityAtSeaLevel) {
    float expected_density = 1.225f; // kg/m^3
    float density = CalculateDensity(101325.0f, 15.0f); // Pressure 101325 Pa, Temperature 15 °C
    EXPECT_NEAR(density, expected_density, 0.001f); // Allowable error of 0.001 kg/m^3
}

TEST(AtmosphereTest, PressureAtAltitude) {
    float altitude = 1000.0f; // 1000 m
    float expected_pressure = 89874.0f; // Pa
    float pressure = CalculatePressure(altitude, 15.0f, 0.5f);
    EXPECT_NEAR(pressure, expected_pressure, 1.0f); // Allowable error of 1 Pa
}

TEST(AtmosphereTest, DensityAtAltitude) {
    float altitude = 1000.0f; // 1000 m
    float expected_density = 1.112f; // kg/m^3
    float density = CalculateDensity(89874.0f, 15.0f); // Pressure at 1000 m
    EXPECT_NEAR(density, expected_density, 0.001f); // Allowable error of 0.001 kg/m^3
}