#ifndef _CONFIG_H_
#define _CONFIG_H_
#define WIFI_SSID "GraziePerIlPesce2.4"
#define WIFI_PASSWORD "DouglasRules"
#define UTC_OFFSET (+1)

#define TELNET_PORT 23

#define DEFAULT_GEO_LAT 43.7167
#define DEFAULT_GEO_LON 10.4
#define ATM_PRESSURE 101.0 //kPa
#define ATM_TEMPERATURE 298.0 // K

//#define USE_MPU6050
#define USE_GY271

#define DEFAULT_AZI_KP 10.
#define DEFAULT_AZI_KD 0.
#define DEFAULT_AZI_KI 0.

#define DEFAULT_ALT_KP 10.
#define DEFAULT_ALT_KD 0.
#define DEFAULT_ALT_KI 0.

#define PID_INTERRUPT_MS 10

//#define ALT_MOTOR_INVERTED
//#define AZI_MOTOR_INVERTED
#define ENCODER_OVERSAMPLING 10
#define DEFAULT_ALT_ENCODER_ZERO 0.0
#define DEFAULT_AZI_ENCODER_ZERO 0.0
#define DEFAULT_ALT_MIN_E 0.0
#define DEFAULT_AZI_MIN_E 0.0
#define MIN_MOTOR_PWM 90

#define MAX_SLEEP_S (2*3600)


#ifndef USE_MPU6050
#define USE_MPU9250
#endif

#endif