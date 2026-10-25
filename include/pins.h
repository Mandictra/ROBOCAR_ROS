#ifndef PINS_H
#define PINS_H

// ---------- Drive Motors (H-bridge IN1/IN2 + quadrature encoder A/B) ----------

// Wheel labels are from the robot's own perspective (standing behind it, looking the way it drives).

// ------- MOTOR FL
#define FL_IN1 13
#define FL_IN2 14
#define ENC_FL_A 46   // A/B swapped so the count rises when the wheel rolls forward (GPIO46 is a strapping pin, verified OK)
#define ENC_FL_B 8

// ------- MOTOR FR
#define FR_IN1 9
#define FR_IN2 10
#define ENC_FR_A 12
#define ENC_FR_B 11

// ------- MOTOR RL
#define RL_IN1 4
#define RL_IN2 5
#define ENC_RL_A 17
#define ENC_RL_B 18

// ------- MOTOR RR
#define RR_IN1 15
#define RR_IN2 16
#define ENC_RR_A 7
#define ENC_RR_B 6

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

// ---------- Servo Actuators (all on a PCA9685 over I2C, no direct servo pins any more) ----------

#define PCA_ADDR  0x40   // PCA9685 I2C address (default, all address jumpers open)
#define CH_PAN    0      // continuous-rotation servo: speed command, no angle
#define CH_TILT   1      // positional servo
#define CH_DUMP   2      // positional servo

// ---------- Sensors ----------

// ---------- I2C Bus (Shared by Color Sensor, IMU and PCA9685) ----------
#define I2C_SDA  38
#define I2C_SCL  39

// ---------- Limits ----------
#define TILT_MIN  30
#define TILT_MAX  150
#define CMD_TIMEOUT_MS  500

#endif // PINS_H