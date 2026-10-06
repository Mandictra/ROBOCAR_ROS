#ifndef PINS_H
#define PINS_H

// ---------- Drive Motors (H-bridge IN1/IN2 + quadrature encoder A/B) ----------

// ------- MOTOR 1 (FL)
#define FL_IN1 13
#define FL_IN2 14
#define ENC_FL_A 11
#define ENC_FL_B 12

// ------- MOTOR 2 (FR)
#define FR_IN1 9
#define FR_IN2 10
#define ENC_FR_A 8
#define ENC_FR_B 46   // GPIO46: strapping pin on the ESP32-S3

// ------- MOTOR 3 (RL), unchanged
#define RL_IN1 4
#define RL_IN2 5
#define ENC_RL_A 7
#define ENC_RL_B 6

// ------- MOTOR 4 (RR)
#define RR_IN1 15
#define RR_IN2 16
#define ENC_RR_A 17
#define ENC_RR_B 18

// +1 = positive command drives that wheel forward, -1 = reversed
#define MOTOR_DIR_FL 1
#define MOTOR_DIR_FR -1
#define MOTOR_DIR_RL -1
#define MOTOR_DIR_RR 1

// ---------- Conveyor Stepper ----------
#define CONV_IN1 41
#define CONV_IN2 42
#define CONV_IN3 2
#define CONV_IN4 1

// ---------- Servo Actuators ----------
#define PAN_PIN   21
#define TILT_PIN  47
#define DUMP_PIN  48

// ---------- Sensors ----------


// ---------- I2C Bus (Shared by Color Sensor & IMU) ----------
//#define I2C_SDA  39// Add your SDA pin number here
//#define I2C_SCL  40// Add your SCL pin number here

// ---------- Limits ----------
#define PAN_MIN   0
#define PAN_MAX   180
#define TILT_MIN  30
#define TILT_MAX  150
#define CMD_TIMEOUT_MS  500

#endif // PINS_H