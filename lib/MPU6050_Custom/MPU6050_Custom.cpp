#include "MPU6050_Custom.h"

MPU6050_Custom::MPU6050_Custom(uint8_t address) : _addr(address), gyro_bias_z(0.0f) {}

bool MPU6050_Custom::begin() {
    Wire.beginTransmission(_addr);
    Wire.write(0x6B); // Power management register
    Wire.write(0);    // Wake up
    if (Wire.endTransmission(true) != 0) return false;

    // Set Digital Low Pass Filter (DLPF) to ~44 Hz to cut motor vibration
    Wire.beginTransmission(_addr);
    Wire.write(0x1A); 
    Wire.write(0x03);
    Wire.endTransmission(true);

    return true;
}

bool MPU6050_Custom::readRaw(int16_t v[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)_addr, 14, true) != 14) return false;
    
    for (int i = 0; i < 7; i++) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        v[i] = (int16_t)((hi << 8) | lo);
    }
    return true;
}

void MPU6050_Custom::calibrateGyro(int samples) {
    double sum = 0; 
    int n = 0;           
    for (int i = 0; i < samples; i++) {
        int16_t v[7];
        if (readRaw(v)) { 
            sum += v[6]; 
            n++; 
        }
        delay(3);
    }
    if (n >= (samples / 4)) {
        // Convert to rad/s
        gyro_bias_z = (float)(sum / n) / 131.0f * (PI / 180.0f);
    }
}

bool MPU6050_Custom::readData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    int16_t v[7];
    if (!readRaw(v)) return false;

    // Scale to m/s^2
    ax = v[0] / 16384.0f * G;
    ay = v[1] / 16384.0f * G;
    az = v[2] / 16384.0f * G;

    // Scale to rad/s (and apply bias to Z)
    gx = (v[4] / 131.0f) * (PI / 180.0f);
    gy = (v[5] / 131.0f) * (PI / 180.0f);
    gz = ((v[6] / 131.0f) * (PI / 180.0f)) - gyro_bias_z;

    return true;
}

bool MPU6050_Custom::healthCheck() {
    Wire.beginTransmission(_addr);
    Wire.write(0x75); // WHO_AM_I
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)_addr, 1, true) != 1) return false;
    
    uint8_t id = Wire.read();
    if (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x72) return false;   

    double amag = 0; 
    int n = 0;
    for (int i = 0; i < 20; i++) {
        int16_t v[7];
        if (readRaw(v)) {
            amag += sqrt((double)v[0]*v[0] + (double)v[1]*v[1] + (double)v[2]*v[2]);
            n++;
        }
        delay(5);
    }
    if (n < 15) return false;
    amag /= n;
    
    // Check gravity magnitude (0.8 to 1.2 g at the +/-2 g range)
    if (amag < 13100 || amag > 19660) return false;   
    
    // Check for excessive zero-rate offset
    if (fabsf(gyro_bias_z) > 0.35f) return false;     
    
    return true;
}