#ifndef PINS_H
#define PINS_H

// ---------- DRV8833 #1 (Front Wheels) ----------
#define FL_IN1 4
#define FL_IN2 5
#define FR_IN1 6
#define FR_IN2 7

// ---------- DRV8833 #2 (Rear Wheels) ----------
#define RL_IN1 15
#define RL_IN2 16
#define RR_IN1 12
#define RR_IN2 11

// ---------- Conveyor Stepper ----------
#define CONV_IN1 41 
#define CONV_IN2 42 
#define CONV_IN3 2  
#define CONV_IN4 1  

// ---------- Servo Actuators ----------
// ---------- Servo Actuators ----------
#define PAN_PIN   36
#define TILT_PIN  37
#define DUMP_PIN  38

// ---------- I2C Color Sensor ----------
#define I2C_SDA 47
#define I2C_SCL 48

// ---------- PCNT Encoders ----------
#define ENC_FL_A 17
#define ENC_FL_B 18
#define ENC_RL_A 9
#define ENC_RL_B 10
#define ENC_FR_A 40
#define ENC_FR_B 39
#define ENC_RR_A 35
#define ENC_RR_B 21

// ---------- Sensor & Limits ----------
#define PAN_MIN   0
#define PAN_MAX   180
#define TILT_MIN  30
#define TILT_MAX  150
#define CMD_TIMEOUT_MS  500

#endif // PINS_H