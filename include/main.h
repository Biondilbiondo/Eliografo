#include <Arduino.h>
// WiFi connection
#include <ESP8266WiFi.h>
// NTP
#include <NTPClient.h>
#include <WiFiUdp.h>
// Real Time Clock
#include <ESP32Time.h>
// SolTrack
#include <SolTrack.h>
// Math Lib
#include <math.h>
//Non volatile memory
#include <Preferences.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define _x_ 0
#define _y_ 1
#define _z_ 2

#define CMD_BUF_LEN 256

#define PREF_RW_MODE false
#define PREF_RO_MODE true

void setup_wifi();

void setup_ntp(void);
void setup_rtc(void);

void MPU_update_rot_frame(float **rf);

float get_lat(void);
float get_lon(void);

void update_time_from_NTP(void);
void get_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);
void get_time_RTC(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);
void get_time_NTP(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds);

void serial_log();

void compute_frame_rotation(float *g, float *m, float **r);
void frame_transform(float *, float **, float *);
float vec_to_azi(float *i);
float vec_to_alt(float *i);
float sc2a(float sine, float cosine);
void get_reflection_vec(float *in, float *mir, float *out);
void get_normal_vec(float *in, float *out, float *mir);
void get_sun_vec(float lon, float lat, 
                 int yr, int month, int day, int hour, int min, float sec, 
                 float *sun);