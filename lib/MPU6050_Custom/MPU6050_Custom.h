#ifndef MPU6050_CUSTOM_H
#define MPU6050_CUSTOM_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050_Custom {
public:
    MPU6050_Custom(uint8_t address = 0x68);
    
    // Setup and calibration
    bool begin();
    void calibrateGyro(int samples = 400);
    bool healthCheck(); // For your self_check function

    // Read scaled data
    bool readData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

    // Getters for biases
    float getGyroBiasZ() const { return gyro_bias_z; }
    float getGyroZScale() const { return GYRO_Z_SCALE; }
    void updateGyroBiasZ(float new_bias) { gyro_bias_z = new_bias; }

private:
    uint8_t _addr;
    float gyro_bias_z;
    const float G = 9.80665f;
    // Gyro Z scale (true rate = scale * measured). 1.0 = uncalibrated.
    // Calibrate: rotate the robot exactly 360 deg by hand with scale = 1, then scale = 2*pi / integral(gz dt).
    static constexpr float GYRO_Z_SCALE = 1.0f;
    
    bool readRaw(int16_t v[7]);
};

#endif