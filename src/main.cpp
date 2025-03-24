#include "main.h"
#include "configs.h"
#include "pins.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

uint32_t chip_id;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
bool NTP_ok = false;

ESP32Time rtc;
bool RTC_ok = false;

Adafruit_MPU6050 mpu;


float sun[3];
float mir[3];
float ory[3];

void setup_i2c(void){
    return;
}

void setup_accell_compass(void){
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
        delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
    
}

void setup_wifi(){
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
    NTP_ok = true;
}

void setup_rtc(void){
    if(NTP_ok){
        update_time_from_NTP();
        rtc.setTime(timeClient.getEpochTime());
        RTC_ok = true;
    }
}

void setup() {
    // Global arrays initialization
    sun[_x_] = sun[_y_] = sun[_z_] = 0.;
    mir[_x_] = mir[_y_] = mir[_z_] = 0.;
    ory[_x_] = ory[_y_] = ory[_z_] = 0.;
    
    chip_id = ESP.getChipId();

    // Serial communication
    Serial.begin(9600);

    delay(2000);

    Serial.printf("CHIP ID: %012x\n", chip_id);

    setup_wifi();
    setup_ntp();
    setup_rtc();
}

void update_time_from_NTP(void){
    while(!timeClient.update()) {
        timeClient.forceUpdate();
    }
}

void get_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
    if(RTC_ok){
        get_time_RTC(year, month, day, hours, minutes, seconds);
        return;
    }

    if(NTP_ok){
        get_time_RTC(year, month, day, hours, minutes, seconds);
        return;
    }

    *hours = 0;
    *minutes = 0;
    *seconds = 0.;
    *day = 1;
    *month = 1;
    *year = 1970;
}

void get_time_RTC(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
    *hours = rtc.getHour(true);
    *minutes = rtc.getMinute();
    *seconds = (float) rtc.getSecond() + (float) rtc.getMillis() / 1000.0;
    *day = rtc.getDay();
    *month = rtc.getMonth() + 1;
    *year = rtc.getYear();
}

void get_time_NTP(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
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
    (*day) = rawTime + 1;
}

void loop() {
    uint16_t year;
    uint8_t month, day, hours, minutes;
    float seconds; 
    char buf[256];

    get_time(&year, &month, &day, &hours, &minutes, &seconds);
    sprintf(buf, "%d-%d-%d %d:%d:%.1f UTC\n", year, month, day, hours, minutes, seconds);
    Serial.println(buf);
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

    loc.longitude   = lon;
    loc.latitude    = lat;
    loc.pressure    = ATM_PRESSURE;
    loc.temperature = ATM_TEMPERATURE;
        
    // Compute Sun position:
    struct STPosition pos;
    SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

    float alt = pos.altitudeRefract / R2D,
          az  = pos.azimuthRefract / R2D;

    /*char buf[256];
    sprintf(buf, "Sun Azimut: %.5f", pos.azimuthRefract);
    Serial.println(buf);
    sprintf(buf, "Sun Altit : %.5f", pos.altitudeRefract);
    Serial.println(buf);*/
      
    sun[_x_] = cos(PI/2.0 - az) * cos(alt);
    sun[_y_] = sin(PI/2.0 - az) * cos(alt);
    sun[_z_] = sin(alt);
}