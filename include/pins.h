#ifndef _pins_h_
#define _pins_h_
// Driver enable
#define DRIVER_ENABLE 4
// Corresponds to EN1
#define ALT_MOTOR_PWM 2
#define ALT_MOTOR_PWM_CH 0
// Corresponds to IN1
#define ALT_MOTOR_DIR1 12
// Corresponds to IN2
#define ALT_MOTOR_DIR2 32
//uncomment below to reverse motor direction
//#define ALT_REVERSED
#define ALT_ENCODER 34

// Corresponds to EN2
#define AZI_MOTOR_PWM 0
#define AZI_MOTOR_PWM_CH 1
// Corresponds to IN3
#define AZI_MOTOR_DIR1 25
// Corresponds to IN4
#define AZI_MOTOR_DIR2 27
//uncomment below to reverse motor direction
//#define AZI_REVERSED
#define AZI_ENCODER 35

#define I2C_SCL 22
#define I2C_SDA 21 
#endif