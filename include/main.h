#include "configs.h"
#include "frames.h"

#include <Arduino.h>
// WiFi connection
#include <WiFi.h>
// ESP Telnet
//#include <WiFiUdp.h>
#include "ESPTelnet.h"
// NTP
#include <NTPClient.h>
// Real Time Clock
#include <ESP32Time.h>
#include "RTClib.h"
#define DS1307_ADDRESS 0x68 
// SolTrack
#include <SolTrack.h>
// Math Lib
#include <math.h>
//Non volatile memory
#include <Preferences.h>
#include <nvs_flash.h>

//Battery
#include "Battery.h"

// MPU gyro accell sensor
#ifdef USE_MPU6050
#include <Adafruit_MPU6050.h>
#endif
#ifdef USE_MPU9250
#include <Adafruit_MPU9250.h>
#endif
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>

// PID motor control
#include "ring_pid.h"

// Little FS to save our scenes
#include "FS.h"
#include <LittleFS.h>

// PWM definitions
#define MOTOR_PWM_RES 8
#define PWM_MAX_VALUE 127
#define MOTOR_PWM_FREQ 35000

#define CMD_BUF_LEN 256

#define PREF_RW_MODE false
#define PREF_RO_MODE true

#define LOG_DEBUG 6
#define LOG_INFO 2
#define LOG_WARNING 1
#define LOG_ERROR 0 

#define NO_TASK 0
#define WIFI_TASK 1
#define SEQUENCE_TASK 2

//in ms
#define SCENE_DT 500



void setup_wifi();
void setup_telnet();

void setup_ntp(void);
void setup_rtc(void);

bool wifi_on(void);
bool wifi_off(void);

void MPU_update_rot_frame(float **rf);

float get_float_cfg(const char *);

void update_time_from_NTP(void);
void get_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);
void get_time_RTC(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);
void get_time_NTP(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);

void get_reflection_vec(float *in, float *mir, float *out);
void get_normal_vec(float *in, float *out, float *mir);
void get_sun_vec(float lon, float lat, 
                 int yr, int month, int day, int hour, int min, float sec, 
                 float *sun);

void set_azi_motor_speed(int8_t speed);
void set_alt_motor_speed(int8_t speed);
void azi_motor_standby(void);
void alt_motor_standby(void);
void azi_motor_enable(void);
void alt_motor_enable(void);
float read_azi_encoder(void);
float read_alt_encoder(void);

void set_pid_goto(float, float);
void pid_loop(void);
bool run_sequence(uint8_t *scenes_seq, uint8_t sequence_len, float run_timestamp);
void sys_log(uint8_t type, const char *format, ...);