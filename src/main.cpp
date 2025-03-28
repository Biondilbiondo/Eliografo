#include "main.h"
#include "configs.h"
#include "pins.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
bool WiFi_ok = false;

WiFiServer ComServer(23);
WiFiClient Controller;

uint32_t chip_id;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
bool NTP_ok = false;

ESP32Time rtc;
bool RTC_ok = false;

Adafruit_MPU6050 mpu;
bool MPU_ok = false;

float sun[3];
float mir[3];
float ory[3];

void setup_remote_com(void){
    ComServer.begin();
}

void setup_accell_compass_MPU6050(void){
    Serial.println("Initializing MPU6050");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        MPU_ok = false;
    }
    else{
        Serial.println("MPU6050 Found!");
        MPU_ok = true;
    }
}

void setup_wifi(){
    WiFi.begin(ssid, password);
  
    Serial.println();
    Serial.print("Connecting");
    for(uint8_t i=0;i < 20 && WiFi.status() != WL_CONNECTED; i++)
    {
      delay(500);
      Serial.print(".");
    }
    if(WiFi.status() == WL_CONNECTED){
        Serial.println("Connected");
        Serial.print("IP Address is: ");
        Serial.println(WiFi.localIP());
        WiFi_ok = true;
    }
    else{
        Serial.println("Connection failed after 10s.");
        WiFi_ok = false;
    }
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

    setup_wifi();
    setup_ntp();
    setup_rtc();
    setup_accell_compass_MPU6050();
    setup_remote_com();
}

uint16_t com_get_next_cmd(char *buf, uint16_t minbuf, uint16_t maxbuf){
    uint16_t i;
    for(i = minbuf; i < maxbuf && (buf[i] = Controller.read()) != '\n' && buf[i] != -1; i++);
    if(buf[i] == '\n' || i == maxbuf){
        buf[i] = '\0';
        return maxbuf;
    }
    else
        return i;
}

bool cmd_id(void){
    char buf[16];
    sprintf(buf, "%012x\n", chip_id);
    Controller.write(buf);
    return true;
}

bool cmd_err(char *cmd){
    char buf[128];
    sprintf(buf, "command unknown %s\n", cmd);
    Controller.write(buf);
    return false;
}

bool cmd_time(void){
    char buf[32];
    uint16_t y;
    uint8_t m,d, h, mi;
    float s;
    get_time(&y, &m, &d, &h, &mi, &s);
    sprintf(buf, "%02d:%02d:%06.3f %02d-%02d-%04d UTC\n", h, mi, s, d, m, y);
    Controller.write(buf);
    return true;
}

bool cmd_reboot(void){
    ESP.restart();
    return true;
}

bool cmd_test_rotframe(void){
    //float m[] = {0.35, 0.72, -0.05};
    //float g[] = {0.1, 0.07, -1.};

    float m[] = {0.0, 1.0, 0.0};
    float g[] = {0.0, 0.0, -1.0};
    float ray[] = {3.2, 5.7, 1.4};
    float *rf[3], _rotframe[9], outray[3];

    char buf[256];

    rf[0] = &(_rotframe[0]);
    rf[1] = &(_rotframe[3]);
    rf[2] = &(_rotframe[6]);

    compute_frame_rotation(g, m, rf);
    Serial.printf("Done");
    sprintf(buf, "%+6.4f %+6.4f %+6.4f\n%+6.4f %+6.4f %+6.4f\n%+6.4f %+6.4f %+6.4f\n", rf[0][0], rf[0][1], rf[0][2],
                                                                                       rf[1][0], rf[1][1], rf[1][2], 
                                                                                       rf[2][0], rf[2][1], rf[2][2]);
    Controller.write(buf);
    frame_transform(ray, rf, outray);
    sprintf(buf, "%+6.4f %+6.4f %+6.4f\n", outray[0], outray[1], outray[2]);
    Controller.write(buf);

    float alt = vec_to_alt(outray) * 180./PI,
          azi = vec_to_azi(outray) * 180./PI;
    sprintf(buf, "AZI: %05.1f (N) ALT: %05.1f\n", 90.-azi, alt);
    Controller.write(buf);
    return true;
}

bool cmd_parse(char *buf){
    char *tok, *rest;
    const char *delim = " \n\r";
    rest = buf;
    tok = strtok_r(buf, delim, &rest);

    //for(int i=0; tok[i]  != '\0'; i++)
    //    Serial.printf("Char %d : '%c'\n", i, tok[i]);
    if(tok == NULL){
        return true;
    }
    else if(strcmp(tok, "id") == 0){
        return cmd_id();
    }
    else if(strcmp(tok, "time") == 0){
        return cmd_time();
    }
    else if(strcmp(tok, "reboot") == 0){
        return cmd_reboot();
    }
    else if(strcmp(tok, "test-rf") == 0){
        return cmd_test_rotframe();
    }
    else{
        return cmd_err(tok);
    }

}

void check_com_connections(void){
  if (ComServer.hasClient())
  {
    // If we are already connected to another computer, 
    // then reject the new connection. Otherwise accept
    // the connection. 
    if (Controller.connected())
    {
      Serial.println("Connection rejected");
      ComServer.available().stop();
    }
    else
    {
      Serial.println("Connection accepted");
      Controller = ComServer.available();
      Controller.write("[  ] > ");
    }
  }
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

    uint16_t buf_last = 0;
    char cmd_buf[CMD_BUF_LEN];

    if(Controller.available()){
        buf_last = com_get_next_cmd(cmd_buf, buf_last, CMD_BUF_LEN);
        if(buf_last == CMD_BUF_LEN){
            if( cmd_parse(cmd_buf) ){
                Controller.write("[OK] ");
            }
            else{
                Controller.write("[!!] ");
            }
            Controller.write("> ");
            buf_last = 0;
        }
    }
    else{
        check_com_connections();
        if(Controller.available()){
            Controller.flush();
            // Don't know why the one above does not work...
            com_get_next_cmd(cmd_buf, 0, CMD_BUF_LEN);
        }
    }
    //get_time(&year, &month, &day, &hours, &minutes, &seconds);
    //sprintf(buf, "%d-%d-%d %d:%d:%.1f UTC\n", year, month, day, hours, minutes, seconds);
    //Serial.println(buf);
    get_sun_vec(GEO_LON, GEO_LAT, year, month, day, hours, minutes, seconds, sun);
    //serial_log();
    //delay(1000); // Wait for 1 second
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

void frame_transform(float *i, float **r, float *o){
    /*Given a vector apply to it the transformation r*/
    o[_x_] = i[_x_] * r[_x_][_x_] + i[_y_] * r[_x_][_y_] + i[_z_] * r[_x_][_z_];
    o[_y_] = i[_x_] * r[_y_][_x_] + i[_y_] * r[_y_][_y_] + i[_z_] * r[_y_][_z_];
    o[_z_] = i[_x_] * r[_z_][_x_] + i[_y_] * r[_z_][_y_] + i[_z_] * r[_z_][_z_];
}

float vec_to_azi(float *i){
    float norm = sqrt(i[_x_]*i[_x_] + i[_y_]*i[_y_]);
    float azi = sc2a(i[_y_]/norm, i[_x_]/norm);
    return azi;
}

float vec_to_alt(float *i){
    float norm = sqrt(i[_x_]*i[_x_] + i[_y_]*i[_y_] + i[_z_]*i[_z_]);
    float alt = sc2a(i[_z_]/norm, sqrt(i[_x_]*i[_x_] + i[_y_]*i[_y_])/norm);
    return alt;
}

float sc2a(float sine, float cosine){
    float angle;
    if(abs(sine) > abs(cosine)){
        angle = asin(sine);
        if(cosine < 0)
            angle = PI/2 - angle;
    }
    else{
        angle = acos(cosine);
        if(sine < 0)
            angle = -angle;
    }
    return angle;
}

void compute_frame_rotation(float *g, float *m, float **r){
    /* Given the magnetic vector m and the gravity vector g, compute
    the rotation matrix (r) from absolute frame (EST = x, NORD = y, UP = z)
    to the internal frame.*/

    float g_n[3], m_n[3], nord[3], up[3], est[3], norm;
    
    // Compute normalized g and m
    norm = sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    g[0] /= norm;
    g[1] /= norm;
    g[2] /= norm;

    norm = sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    m[0] /= norm;
    m[1] /= norm;
    m[2] /= norm;

    // Compute nord vector as the projection of m on the 
    // plane orthogonal to g
    // nord = m - (g.m)g
    float g_dot_m = g[_x_]*m[_x_] + g[_y_]*m[_y_] + g[_z_]*m[_z_];
    nord[_x_] = m[_x_] - g_dot_m * g[_x_];
    nord[_y_] = m[_y_] - g_dot_m * g[_y_];
    nord[_z_] = m[_z_] - g_dot_m * g[_z_];
    Serial.printf("NORD: %+6.4f %+6.4f %+6.4f\n", nord[_x_], nord[_y_], nord[_z_]);

    // Normalize the nord
    norm = sqrt(nord[0]*nord[0] + nord[1]*nord[1] + nord[2]*nord[2]);
    nord[_x_] /= norm;
    nord[_y_] /= norm;
    nord[_z_] /= norm;

    // Up is the inverse of gravity
    up[_x_] = -g[_x_];
    up[_y_] = -g[_y_];
    up[_z_] = -g[_z_];

    // Est is nord x up to give e right-handed system
    est[_x_] = nord[_y_] * up[_z_] - nord[_z_] * up[_y_];
    est[_y_] = nord[_z_] * up[_x_] - nord[_x_] * up[_z_];
    est[_z_] = nord[_x_] * up[_y_] - nord[_y_] * up[_x_];

    r[_x_][_x_] = est[_x_];
    r[_x_][_y_] = nord[_x_];
    r[_x_][_z_] = up[_x_];
    
    r[_y_][_x_] = est[_y_];
    r[_y_][_y_] = nord[_y_];
    r[_y_][_z_] = up[_y_];

    r[_z_][_x_] = est[_z_];
    r[_z_][_y_] = nord[_z_];
    r[_z_][_z_] = up[_z_];
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