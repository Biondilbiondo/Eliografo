#ifndef _CONFIG_H_
#define _CONFIG_H_
#define WIFI_SSID "My_Wifi_Network"
#define WIFI_PASSWORD "My_Wifi_Password"
#define UTC_OFFSET (+1)

#define TELNET_PORT 23

#define DEFAULT_GEO_LAT 0.0
#define DEFAULT_GEO_LON 0.0
#define ATM_PRESSURE 101.0 //kPa
#define ATM_TEMPERATURE 298.0 // K

#define DEFAULT_AZI_KP 10.
#define DEFAULT_AZI_KD 0.
#define DEFAULT_AZI_KI 0.
#define DEFAULT_AZI_MIN_E 5.0

#define DEFAULT_ALT_KP 10.
#define DEFAULT_ALT_KD 0.
#define DEFAULT_ALT_KI 0.
#define DEFAULT_ALT_MIN_E 5.0

#define PID_INTERRUPT_MS 10
//#define ALT_MOTOR_INVERTED
//#define AZI_MOTOR_INVERTED
#define ALT_ENCODER_OVERSAMPLING 10
#define DEFAULT_ALT_ENCODER_ZERO 0.0
#define DEFAULT_AZI_ENCODER_ZERO 0.0


#define USE_MPU6050

#ifndef USE_MPU6050
#define USE_MPU9250
#endif

#endif