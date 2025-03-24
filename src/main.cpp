#include <Arduino.h>
// WiFi connection
#include <ESP8266WiFi.h>
// NTP
#include <NTPClient.h>
#include <WiFiUdp.h>
// SolTrack
#include <SolTrack.h>

#define _x_ 0
#define _y_ 1
#define _z_ 2

#include "configs.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

float sun[3];
float mir[3];
float ory[3];

void serial_log(void);
void get_reflection_vec(float *, float *, float *);
void get_sun_vec(float, float, 
                 int, int, int, int, int, float, 
                 float *);

void setup_wifi(){
    delay(1000);
   
    WiFi.begin(ssid, password);
  
    Serial.println();
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
  
    Serial.println("success!");
    Serial.print("IP Address is: ");
    Serial.println(WiFi.localIP());
}

void setup_ntp(void){
    timeClient.begin();
    timeClient.setTimeOffset(3600 * UTC_OFFSET);
}

void setup() {
    // Global arrays initialization
    sun[_x_] = sun[_y_] = sun[_z_] = 0.;
    mir[_x_] = mir[_y_] = mir[_z_] = 0.;
    ory[_x_] = ory[_y_] = ory[_z_] = 0.;
    
    // Serial communication
    Serial.begin(9600);

    // WiFi
    setup_wifi();
    setup_ntp();
}

void update_time_from_NTP(void){
    while(!timeClient.update()) {
        timeClient.forceUpdate();
    }
}

void get_time_NTP(int *year, int *month, int *day, int *hours, int *minutes, float *seconds){
    static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

    *hours = timeClient.getHours();
    *minutes = timeClient.getMinutes();
    *seconds = (float) timeClient.getSeconds();
    
    // Now compute days years and months
    unsigned long epoch = timeClient.getEpochTime();
    unsigned long rawTime = epoch / 86400L;  // in days

    *day = 0;
    *year = 1970;

    while((*day += (LEAP_YEAR(*year) ? 366 : 365)) <= rawTime)
      (*year)++;
    rawTime -= *day - (LEAP_YEAR(*year) ? 366 : 365); // now it is days in this year, starting at 0
    *day = 0;
    for(*month=0; *month<12; (*month)++) {
      uint8_t monthLength;
      if(*month == 1){ // february
        monthLength = LEAP_YEAR(*year) ? 29 : 28;
      } 
      else{
        monthLength = monthDays[*month];
      }
      if(rawTime < monthLength) 
          break;
      rawTime -= monthLength;
    }
    // Gennuary is one
    (*month)++;
    // First day of a month is one
    (*day)++;
}

void loop() {
    int year, month, day, hours, minutes;
    float seconds; 
    get_time_NTP(&year, &month, &day, &hours, &minutes, &seconds);
    get_sun_vec(GEO_LON, GEO_LAT, year, month, day, hours, minutes, seconds, sun);
    serial_log();
    delay(1000); // Wait for 1 second
}

void serial_log(){
    char buf[256];
    sprintf(buf, "SUN      : [%.4f %.4f %.4f]", sun[_x_], sun[_y_], sun[_z_]);
    Serial.println(buf);
    sprintf(buf, "MIRROR   : [%.4f %.4f %.4f]", sun[_x_], sun[_y_], sun[_z_]);
    Serial.println(buf);
    sprintf(buf, "OUT RAY  : [%.4f %.4f %.4f]", sun[_x_], sun[_y_], sun[_z_]);
    Serial.println(buf);

    String outs = timeClient.getFormattedDate();
    Serial.println(outs);
}

void get_reflection_vec(float *in, float *mir, float *out){
    float in_dot_mir = in[_x_] * mir[_x_] + \
                       in[_y_] * mir[_y_] + \
                       in[_z_] * mir[_z_];
    
    for(int i=0; i < 3; i++)
        out[i] = in[i] - 2 * in_dot_mir * mir[i];

    return ;
}

void get_normal_vec(float *in, float *out, float *mir){
    float norm = - in[_x_] * out[_x_] + \
                 - in[_y_] * out[_y_] + \
                 - in[_z_] * out[_z_];

    norm = - 2.0 * sqrt((1+norm)/2.0);
    for(int i=0; i < 3; i++)
        mir[i] = (in[i] - out[i])/norm; 
}

void get_sun_vec(float lon, float lat, 
                 int yr, int month, int day, int hour, int min, float sec, 
                 float *sun){         
    int useDegrees = 1;             // Input (geographic position) and output are in degrees
    int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
    int computeRefrEquatorial = 0;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
    int computeDistance = 0;        // Compute the distance to the Sun in AU: 0-no, 1-yes

    struct STTime time;               // Struct for date and time variables
    struct STLocation loc;            // Struct for geographic location variables

    time.year   = yr;
    time.month  = month;
    time.day    = day;
    time.hour   = hour;
    time.minute = min;
    time.second = sec;

    loc.longitude   = lon;  // HAN University of applied sciences, Arnhem, The Netherlands
    loc.latitude    = lat;
    loc.pressure    = 101.0;      // Atmospheric pressure in kPa
    loc.temperature = 283.0;      // Atmospheric temperature in K
        
    // Compute Sun position:
    struct STPosition pos;
    SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

    float alt = pos.altitudeRefract / R2D,
          az  = pos.azimuthRefract / R2D;
      
    sun[_x_] = cos(PI/2.0 - az) * cos(alt);
    sun[_y_] = sin(PI/2.0 - az) * cos(alt);
    sun[_z_] = sin(alt);
}

/*int main(int argc, char **argv){

    float sun[3];
    float mir[] = {0.09925833, -0.99258333,  0.07018624};
    float out[3], in[3], mir2[3]; 
    
    get_sun_vec(10.4036, 43.70853, 2025, 3, 14, 12, 0, 0, sun);
    
    for(int  i=0; i < 3; i++)
        in[i] = -sun[i];

    get_reflection_vec(in, mir, out);
    get_normal_vec(in, out, mir2);

    printf("Sun:       \t%8.4f %8.4f %8.4f\n", sun[_x_], sun[_y_], sun[_z_]);
    printf("Mirror:    \t%8.4f %8.4f %8.4f\n", mir[_x_], mir[_y_], mir[_z_]);
    printf("Reflection:\t%8.4f %8.4f %8.4f\n", out[_x_], out[_y_], out[_z_]);
    printf("Mirror2:   \t%8.4f %8.4f %8.4f\n", mir2[_x_], mir2[_y_], mir2[_z_]);

}*/