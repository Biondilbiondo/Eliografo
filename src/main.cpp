#include "main.h"
#include "configs.h"
#include "pins.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
bool WiFi_ok = false;

//WiFiServer ComServer(23);
//WiFiClient Controller;
ESPTelnet telnet;

uint32_t chip_id;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
bool NTP_ok = false;

ESP32Time rtc;
bool RTC_ok = false;

#ifdef USE_MPU6050
Adafruit_MPU6050 mpu;
#endif
#ifdef USE_MPU9250
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#endif

bool MPU_ok = false, ROTF_ok=false;

// Non volatile memory
Preferences HGPrefs; 

// Control variables
float azi_setpoint, alt_setpoint,
      azi_encoder_val, alt_encoder_val,
      azi_motor_speed, alt_motor_speed,
      alt_encoder_zero, azi_encoder_zero;
float current_lat, current_lon;

ringPID aziPID(&azi_encoder_val, &azi_motor_speed, &azi_setpoint);
ringPID altPID(&alt_encoder_val, &alt_motor_speed, &alt_setpoint);
bool azi_PID_enabled = false, alt_PID_enabled = false;
bool solar_control_enabled = false, manual_control_enabled = false;
hw_timer_t *PID_timer_cfg = NULL;

float sun[3];
float mir[3];
float ory[3];
float ory_alt = 0.0, ory_azi = 0.0;

// Time/RTC Routines
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

// Preferences Routines

bool cfg_key_exists(const char *key){
    return HGPrefs.isKey(key);
}

float get_float_default_cfg(const char *k){
    if(strcmp(k, "azi_kp") == 0)
        return DEFAULT_AZI_KP;
    if(strcmp(k, "azi_kd") == 0)
        return DEFAULT_AZI_KD;
    if(strcmp(k, "azi_ki") == 0)
        return DEFAULT_AZI_KI;
    if(strcmp(k, "alt_kp") == 0)
        return DEFAULT_ALT_KP;
    if(strcmp(k, "alt_kd") == 0)
        return DEFAULT_ALT_KD;
    if(strcmp(k, "alt_ki") == 0)
        return DEFAULT_ALT_KI;
    if(strcmp(k, "lat") == 0)
        return DEFAULT_GEO_LAT;
    if(strcmp(k, "lon") == 0)
        return DEFAULT_GEO_LON;
    if(strcmp(k, "alte0") == 0)
        return DEFAULT_ALT_ENCODER_ZERO;
    if(strcmp(k, "azie0") == 0)
        return DEFAULT_AZI_ENCODER_ZERO;
    if(strcmp(k, "azi_me") == 0)
        return DEFAULT_AZI_MIN_E;
    if(strcmp(k, "alt_me") == 0)
        return DEFAULT_ALT_MIN_E;
    return 0.0;
}

float get_float_cfg(const char *k){
    if(!cfg_key_exists(k))
        HGPrefs.putFloat(k, get_float_default_cfg(k));
    return HGPrefs.getFloat(k);
}

// Reflection Routines
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

    float alt =  pos.altitudeRefract / R2D,
          az  = pos.azimuthRefract / R2D;
    

    /*Serial.printf("SUN alt %f azi %f\n", pos.altitudeRefract, pos.azimuthRefract);
    sun[_x_] = cos(PI/2.0 - az) * cos(alt);
    sun[_y_] = sin(PI/2.0 - az) * cos(alt);
    sun[_z_] = sin(alt);
    Serial.printf("Sun vector %f %f %f\n", sun[_x_], sun[_y_], sun[_z_]);*/
    geo_to_absolute(pos.altitudeRefract, pos.azimuthRefract, sun);
    //Serial.printf("New vector %f %f %f\n", sun[_x_], sun[_y_], sun[_z_]);
}

// Motors and Encoders Routine
void set_alt_motor_speed(int8_t speed){
    if(speed > 0){
        // Forward motion
        ledcWrite(ALT_MOTOR_PWM_CH, 0);
#ifndef ALT_MOTOR_INVERTED
        digitalWrite(ALT_MOTOR_DIR1, LOW);
        digitalWrite(ALT_MOTOR_DIR2, HIGH);
#else
        digitalWrite(ALT_MOTOR_DIR1, HIGH);
        digitalWrite(ALT_MOTOR_DIR2, LOW);
#endif
        ledcWrite(ALT_MOTOR_PWM_CH, speed*2);
    }
    else if(speed < 0){
        // Bakward motion
        ledcWrite(ALT_MOTOR_PWM_CH, 0);
#ifndef ALT_MOTOR_INVERTED
        digitalWrite(ALT_MOTOR_DIR1, HIGH);
        digitalWrite(ALT_MOTOR_DIR2, LOW);
#else
        digitalWrite(ALT_MOTOR_DIR1, LOW);
        digitalWrite(ALT_MOTOR_DIR2, HIGH);
#endif
        ledcWrite(ALT_MOTOR_PWM_CH, -speed*2);
    }
    else{
        // Blocked 
        ledcWrite(ALT_MOTOR_PWM_CH, 0);
        digitalWrite(ALT_MOTOR_DIR1, LOW);
        digitalWrite(ALT_MOTOR_DIR2, LOW);
    }
}

void set_azi_motor_speed(int8_t speed){
    if(speed > 0){
        // Forward motion
        ledcWrite(AZI_MOTOR_PWM_CH, 0);
#ifndef AZI_MOTOR_INVERTED
        digitalWrite(AZI_MOTOR_DIR1, LOW);
        digitalWrite(AZI_MOTOR_DIR2, HIGH);
#else
        digitalWrite(AZI_MOTOR_DIR1, HIGH);
        digitalWrite(AZI_MOTOR_DIR2, LOW);
#endif

        ledcWrite(AZI_MOTOR_PWM_CH, speed*2);
    }
    else if(speed < 0){
        // Bakward motion
        ledcWrite(AZI_MOTOR_PWM_CH, 0);
#ifndef AZI_MOTOR_INVERTED
        digitalWrite(AZI_MOTOR_DIR1, HIGH);
        digitalWrite(AZI_MOTOR_DIR2, LOW);
#else
        digitalWrite(AZI_MOTOR_DIR1, LOW);
        digitalWrite(AZI_MOTOR_DIR2, HIGH);
#endif
        ledcWrite(AZI_MOTOR_PWM_CH, -speed*2);
    }
    else{
        // Blocked 
        ledcWrite(AZI_MOTOR_PWM_CH, 0);
        digitalWrite(AZI_MOTOR_DIR1, LOW);
        digitalWrite(AZI_MOTOR_DIR2, LOW);
    }
}

void azi_motor_standby(void){
    ledcWrite(AZI_MOTOR_PWM_CH, 0);
    digitalWrite(AZI_MOTOR_DIR1, LOW);
    digitalWrite(AZI_MOTOR_DIR2, LOW);
}

void alt_motor_standby(void){
    ledcWrite(ALT_MOTOR_PWM_CH, 0);
    digitalWrite(ALT_MOTOR_DIR1, LOW);
    digitalWrite(ALT_MOTOR_DIR2, LOW);
}

void azi_motor_enable(void){
    azi_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    azi_PID_enabled = true;
}

void alt_motor_enable(void){
    alt_encoder_val = read_alt_encoder();
    alt_setpoint = alt_encoder_val;
    alt_PID_enabled = true;
}



float read_azi_encoder(void){
    // This is actual value in internal frame;
    // NOTE: it should be in degrees and increasing ccw
    float f_v = 0.0;
    for(int i=0; i < AZI_ENCODER_OVERSAMPLING; i++){
        f_v += (float) analogReadMilliVolts(AZI_ENCODER);
        delay(1);
    }
    f_v /= AZI_ENCODER_OVERSAMPLING;
    f_v *= (ENCODER_R1 + ENCODER_R2) / ENCODER_R2 ; // Real tension value
    f_v /= 1000.0;
    float deg = f_v * ENCODER_VOLT_TO_DEG - azi_encoder_zero;
    if(deg < 0) deg+=360;
    return deg; // Just to put something here
}

float read_alt_encoder(void){
    float f_v = 0.0;
    for(int i=0; i < ALT_ENCODER_OVERSAMPLING; i++){
        f_v += (float) analogReadMilliVolts(ALT_ENCODER);
        delay(1);
    }
    f_v /= ALT_ENCODER_OVERSAMPLING;
    // This is actual value in internal frame;
    // NOTE: it should be in degrees and increasing rotating upward
    
    f_v *= (ENCODER_R1 + ENCODER_R2) / ENCODER_R2 ; // Real tension value
    f_v /= 1000.0;
    float deg = f_v * ENCODER_VOLT_TO_DEG - alt_encoder_zero;
    if(deg < 0) deg+=360;
    return deg; // Just to put something here
}

// Telnet Shell commands
bool cmd_id(void){
    char buf[16];
    sprintf(buf, "%012x\n", chip_id);
    telnet.print(buf);
    return true;
}

bool cmd_err(char *cmd){
    char buf[128];
    sprintf(buf, "command unknown %s\n", cmd);
    telnet.print(buf);
    return false;
}

bool cmd_time(void){
    char buf[32];
    uint16_t y;
    uint8_t m,d, h, mi;
    float s;
    get_time(&y, &m, &d, &h, &mi, &s);
    sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%06.4fZ\n", y, m, d, h, mi, s);
    telnet.print(buf);
    return true;
}

bool cmd_reboot(void){
    ESP.restart();
    return true;
}

bool cmd_set_geo(char *buf){
    float lat = get_float_cfg("lat"), 
          lon = get_float_cfg("lon");

    sscanf(buf, "%f%f", &lat, &lon);
    HGPrefs.putFloat("lat", lat);
    HGPrefs.putFloat("lon", lon);
    return true;
}

bool cmd_get_geo(void){
    float lat = get_float_cfg("lat"), 
          lon = get_float_cfg("lon");

    char buf[32];
    sprintf(buf, "LAT: %08.4f LON: %08.4f\n", lat, lon);
    telnet.print(buf);
    return true;
}

//TODO: Remove
bool cmd_pid_prm(void){
    char buf[64];
    sprintf(buf, "ALT: P %8.1f I %8.1f D %8.1f\n", get_float_cfg("alt_kp"), get_float_cfg("alt_ki"), get_float_cfg("alt_kd"));
    telnet.print(buf);
    sprintf(buf, "AZI: P %8.1f I %8.1f D %8.1f\n", get_float_cfg("azi_kp"), get_float_cfg("azi_ki"), get_float_cfg("azi_kd"));
    telnet.print(buf);
    return true;
}
//TODO: Remove
bool cmd_pid_vals(void){
    char buf[64];
    sprintf(buf, "ALT: SPEED %f VAL %f SET %f\n", alt_motor_speed, alt_encoder_val, alt_setpoint);
    telnet.print(buf);
    sprintf(buf, "AZI: SPEED %f VAL %f SET %f\n", azi_motor_speed, azi_encoder_val, azi_setpoint);
    telnet.print(buf);
    return true;
}

bool cmd_set(char *buf){
    float val;
    char *key, *rest;

    key = strtok_r(NULL, " \n\r", &buf);
    rest = strtok_r(NULL, "\r\n", &buf);
    sscanf(rest, "%f", &val);
    if(cfg_key_exists(key)){
        HGPrefs.putFloat(key, val);
        return true;
    }
    else{
        char obuf[32];
        sprintf(obuf, "Key %s not found.\n", key);
        telnet.print(obuf);
        return false;
    }
}

bool cmd_get(char *buf){
    float val;
    char *key;
    char obuf[32];
    
    key = strtok_r(NULL, " \n\r", &buf);
    if(cfg_key_exists(key)){
        val = get_float_cfg(key);
        sprintf(obuf, "%15s = %10g\n", key, val);
        telnet.print(obuf);
        return true;
    }
    else{
        sprintf(obuf, "Key %s not found.\n", key);
        telnet.print(obuf);
        return false;
    }
}

bool cmd_factory_reset(void){
    nvs_flash_erase();
    nvs_flash_init();
    return cmd_reboot();
}

bool cmd_set_ory(char *buf){
    float alt, azi;
    sscanf(buf, "%f %f", &alt, &azi);
    geo_to_absolute(alt, azi, ory);
    
    return true;
}

bool cmd_mirror_log(void){
    char buf[256];
    
    sprintf(buf, "SUN      : %.4f %.4f %.4f\n", sun[_x_], sun[_y_], sun[_z_]);
    telnet.print(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(sun), absolute_to_geo_azi(sun));
    telnet.print(buf);

    sprintf(buf, "OUT-RAY  : %.4f %.4f %.4f\n", ory[_x_], ory[_y_], ory[_z_]);
    telnet.print(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(ory), absolute_to_geo_azi(ory));
    telnet.print(buf);

    sprintf(buf, "MIRROR   : %.4f %.4f %.4f\n", mir[_x_], mir[_y_], mir[_z_]);
    telnet.print(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(mir), absolute_to_geo_azi(mir));
    telnet.print(buf);


    String outs = timeClient.getFormattedDate();
    sprintf(buf, "TIME     : %s\n", outs.c_str());
    telnet.print(buf);
    return true;
}

bool cmd_current_position(void){
    float internal_alt = read_alt_encoder(),
          internal_azi = read_azi_encoder();

    char buf[256];
    
    sprintf(buf, "INTERNAL ALT %.4f AZI %.4f\n", internal_alt, internal_azi);
    telnet.print(buf);
    sprintf(buf, "ABSOLUTE ALT %.4f AZI %.4f\n", internal_to_geo_alt(internal_alt, internal_azi), 
                                                 internal_to_geo_azi(internal_alt, internal_azi));
    telnet.print(buf);
    return true;
}

//TODO: Remove
bool cmd_test_motor2(int8_t speed){
    char buf[256];
    
    for(int i=0; i < 100; i++){
        set_alt_motor_speed(120);
        delay(100);
        alt_motor_standby();
        sprintf(buf, "ALT %f\n", read_alt_encoder());
        telnet.print(buf);
    }
    for(int i=0; i < 100; i++){
        set_azi_motor_speed(120);
        delay(100);
        azi_motor_standby();
        sprintf(buf, "ALT %f\n", read_azi_encoder());
        telnet.print(buf);
    }

    return true;
}

//TODO: Remove
bool cmd_alt_goto(char *buf){
    float dest, pos;
    char outm[256];
    sscanf(buf, "%f", &dest);
    pos = read_alt_encoder();
    sprintf(outm, "GOING TO ALT %.4f\n", pos);
    telnet.print(outm);
    if(pos > dest){
        for(int i=0; i<1000;i++){
            set_alt_motor_speed(-120);
            delay(100);
            alt_motor_standby();
            pos = read_alt_encoder();
            sprintf(outm, "INTERNAL ALT %.4f\n", pos);
            telnet.print(outm);
            if(pos < dest) break;
        }
    }
    else{
        for(int i=0; i<1000;i++){
            set_alt_motor_speed(120);
            delay(100);
            alt_motor_standby();
            pos = read_alt_encoder();
            sprintf(outm, "INTERNAL ALT %.4f\n", pos);
            telnet.print(outm);
            if(pos > dest) break;
        }
    }
    return true;
}
//TODO: Remove
bool cmd_alt_pid_setpoint(char *buf){
    float dest;
    char outm[256];
    sscanf(buf, "%f", &dest);
    alt_setpoint = dest;
    sprintf(outm, "GOING TO ALT %.4f\n", dest);
    telnet.print(outm);
    manual_control_enabled = true;
    alt_PID_enabled = true;

    return true;
}
//TODO: Remove
bool cmd_azi_pid_setpoint(char *buf){
    float dest;
    char outm[256];
    sscanf(buf, "%f", &dest);
    azi_setpoint = dest;
    sprintf(outm, "GOING TO AZI %.4f\n", dest);
    telnet.print(outm);
    manual_control_enabled = true;
    azi_PID_enabled = true;

    return true;
}
//TODO: Remove
bool cmd_alt_pid_disable(void){
    alt_PID_enabled = false;
    alt_motor_standby();

    return true;
}

bool cmd_reconfigure(void){
    manual_control_enabled = false;
    solar_control_enabled = false;
    alt_PID_enabled = false;
    azi_PID_enabled = false;

    azi_encoder_zero = get_float_cfg("azie0");
    alt_encoder_zero = get_float_cfg("alte0");

    azi_encoder_val = read_azi_encoder();
    alt_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    alt_setpoint = alt_encoder_val;
    alt_motor_speed = 0.;
    azi_motor_speed = 0.;

    aziPID.set_PID_params(get_float_cfg("azi_kp"),
                          get_float_cfg("azi_ki"),
                          get_float_cfg("azi_kd"),
                          -126, 126,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ki"),
                          get_float_cfg("alt_kd"),
                          -126, 126,
                          get_float_cfg("alt_me"));

    sun[_x_] = sun[_y_] = sun[_z_] = 0.;
    mir[_x_] = mir[_y_] = mir[_z_] = 0.;
    ory[_x_] = ory[_y_] = ory[_z_] = 0.;
    
    current_lon = get_float_cfg("lon");
    current_lat = get_float_cfg("lat");

    return true;
}

bool cmd_control_reset(void){
    solar_control_enabled = false;
    manual_control_enabled = false;
    azi_PID_enabled = false;
    alt_PID_enabled = false;
    
    return true;
}

bool cmd_manual_control_geo(char *buf){
    float alt, azi;

    sscanf(buf, "%f%f", &alt, &azi);
    cmd_control_reset();
    alt_setpoint = geo_to_internal_alt(alt, azi);
    azi_setpoint = geo_to_internal_azi(alt, azi);
    manual_control_enabled = true;
    azi_PID_enabled = true;
    alt_PID_enabled = true;
    
    char msg[256];
    sprintf(msg, "Moving to GEO ALT %f AZI %f (INT ALT %f, INT AZI %f)\n", alt, azi, alt_setpoint, azi_setpoint);
    telnet.print(msg);
    return true;
}

bool cmd_manual_control_internal(char *buf){
    float alt, azi;

    sscanf(buf, "%f%f", &alt, &azi);
    cmd_control_reset();
    alt_setpoint = alt;
    azi_setpoint = azi;
    manual_control_enabled = true;
    azi_PID_enabled = true;
    alt_PID_enabled = true;
    
    char msg[256];
    sprintf(msg, "Moving to INT ALT %f AZI %f (GEO ALT %f, GEO AZI %f)\n", alt, azi,
                                                                           internal_to_geo_alt(alt, azi), 
                                                                           internal_to_geo_azi(alt, azi));
    telnet.print(msg);
    return true;
}

bool cmd_solar_control(char *buf){
    float alt, azi;

    sscanf(buf, "%f%f", &alt, &azi);
    cmd_control_reset();
    solar_control_enabled = true;
    ory_alt = alt;
    ory_azi = azi;
    azi_PID_enabled = true;
    alt_PID_enabled = true;
    
    return true;
}

bool cmd_parse(char *buf){
    char *tok, *rest;
    const char *delim = " \n\r";
    rest = buf;
    tok = strtok_r(buf, delim, &rest);

    if(tok == NULL){
        return true;
    }
    else if(strcmp(tok, "set-ory") == 0){
        return cmd_set_ory(rest);
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
    else if(strcmp(tok, "set-geo") == 0){
        return cmd_set_geo(rest);
    }
    else if(strcmp(tok, "set") == 0){
        return cmd_set(rest);
    }
    else if(strcmp(tok, "get-geo") == 0){
        return cmd_get_geo();
    }
    else if(strcmp(tok, "get") == 0){
        return cmd_get(rest);
    }
    else if(strcmp(tok, "pid-prm") == 0){
        return cmd_pid_prm();
    }
    else if(strcmp(tok, "factory-reset") == 0){
        return cmd_factory_reset();
    }
    else if(strcmp(tok, "mirror-log") == 0){
        return cmd_mirror_log();
    }
    else if(strcmp(tok, "quit") == 0){
        telnet.println("Use quit to disconnect");
        return true;
    }
    else if(strcmp(tok, "current-position") == 0){
        return cmd_current_position();
    }
    else if(strcmp(tok, "test-motors") == 0){
        return cmd_test_motor2(128);
    }
    else if(strcmp(tok, "alt-fwd") == 0){
        telnet.print("ALT FWD\n");
        set_alt_motor_speed(120);
        sleep(2);
        telnet.print("ALT STANDBY\n");
        alt_motor_standby();
        return true;
    }
    else if(strcmp(tok, "alt-bck") == 0){
        telnet.print("ALT REV\n");
        set_alt_motor_speed(-120);
        sleep(2);
        telnet.print("ALT STANDBY\n");
        alt_motor_standby();
        return true;
    }
    else if(strcmp(tok, "azi-fwd") == 0){
        telnet.print("AZI FWD\n");
        set_azi_motor_speed(120);
        sleep(2);
        telnet.print("AZI STANDBY\n");
        azi_motor_standby();
        return true;
    }
    else if(strcmp(tok, "azi-bck") == 0){
        telnet.print("AZI REV\n");
        set_azi_motor_speed(-120);
        sleep(2);
        telnet.print("AZI STANDBY\n");
        azi_motor_standby();
        return true;
    }
    else if(strcmp(tok, "alt-goto") == 0){
        return cmd_alt_goto(rest);
    }
    else if(strcmp(tok, "alt-pid-setpoint") == 0){
        return cmd_alt_pid_setpoint(rest);
    }
    else if(strcmp(tok, "azi-pid-setpoint") == 0){
        return cmd_azi_pid_setpoint(rest);
    }
    else if(strcmp(tok, "pid-vals") == 0){
        return cmd_pid_vals();
    }
    else if(strcmp(tok, "reconfigure") == 0){
        return cmd_pid_vals();
    }
    else if(strcmp(tok, "mcg") == 0 || strcmp(tok, "manual-control-geo") == 0){
        return cmd_manual_control_geo(rest);
    }
    else if(strcmp(tok, "mci") == 0 || strcmp(tok, "manual-control-int") == 0){
        return cmd_manual_control_internal(rest);
    }
    else if(strcmp(tok, "sc") == 0 || strcmp(tok, "solar-control") == 0){
        return cmd_solar_control(rest);
    }
    else if(strcmp(tok, "control-reset") == 0 || strcmp(tok, "stop") == 0){
        alt_motor_standby();
        azi_motor_standby();
        return cmd_control_reset();
    }
    else{
        return cmd_err(tok);
    }
}

// Setup Routines

void onTelnetConnect(String ip) {
    Serial.print("- Telnet: ");
    Serial.print(ip);
    Serial.println(" connected");

    char buf[16];
    sprintf(buf, "%012x\n", chip_id);
  
    telnet.println("\nWelcome to Heliograph\nIP: " + telnet.getIP() + "\nID: " + buf);
    telnet.println("Use quit to disconnect");
    telnet.print("[  ] > ");
}

void onTelnetDisconnect(String ip) {
    Serial.print("- Telnet: ");
    Serial.print(ip);
    Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
    Serial.print("- Telnet: ");
    Serial.print(ip);
    Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
    Serial.print("- Telnet: ");
    Serial.print(ip);
    Serial.println(" tried to connected");
}

void onTelnetInput(String str) {
    char *a;
    a = new char[str.length() + 1];
    strncpy(a, str.c_str(), str.length());
    a[str.length()] = '\0';
    if( cmd_parse(a) ){
        telnet.print("[OK] ");
    }
    else{
        telnet.print("[!!] ");
    }
    telnet.print("> ");
}

void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);

  Serial.print("- Telnet: ");
  if (telnet.begin(TELNET_PORT)) {
    Serial.println("running");
  } 
  else {
    Serial.println("error.");
  }
}

void setup_pref(void){
    HGPrefs.begin("hg", PREF_RW_MODE);
}


void setup_motors(){
    pinMode(ALT_MOTOR_DIR1, OUTPUT);
    pinMode(ALT_MOTOR_DIR2, OUTPUT);
    pinMode(ALT_MOTOR_PWM, OUTPUT);

    ledcSetup(ALT_MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(ALT_MOTOR_PWM, ALT_MOTOR_PWM_CH);

    pinMode(AZI_MOTOR_DIR1, OUTPUT);
    pinMode(AZI_MOTOR_DIR2, OUTPUT);
    pinMode(AZI_MOTOR_PWM, OUTPUT);

    ledcSetup(AZI_MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(AZI_MOTOR_PWM, AZI_MOTOR_PWM_CH);

    azi_encoder_zero = get_float_cfg("azie0");
    alt_encoder_zero = get_float_cfg("alte0");

    azi_encoder_val = read_azi_encoder();
    alt_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    alt_setpoint = alt_encoder_val;
    aziPID.set_PID_params(get_float_cfg("azi_kp"),
                          get_float_cfg("azi_ki"),
                          get_float_cfg("azi_kd"),
                          50, 126,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ki"),
                          get_float_cfg("alt_kd"),
                          50, 126,
                          get_float_cfg("alt_me"));

    //Interrupt
    /*PID_timer_cfg = timerBegin(0, 40000, true);
    timerAttachInterrupt(PID_timer_cfg, &PID_isr, true);
    timerAlarmWrite(PID_timer_cfg, PID_INTERRUPT_MS, true);
    timerAlarmEnable(PID_timer_cfg);*/

    azi_motor_standby();
    alt_motor_standby();
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

    chip_id = (uint32_t) ESP.getEfuseMac();

    // Serial communication
    Serial.begin(9600);

    delay(2000);
    // TODO, this is Berlin
    current_lon = 13.405;// get_float_cfg("lon");
    current_lat = 52.52; //get_float_cfg("lat");

    setup_pref();
    setup_wifi();
    setup_ntp();
    setup_rtc();

    setupTelnet();
    setup_motors();
}

void loop() {
    uint16_t year;
    uint8_t month, day, hours, minutes;
    float seconds; 
    char buf[256];

    uint16_t buf_last = 0;
    char cmd_buf[CMD_BUF_LEN];

    telnet.loop();
    
    delay(10);
    char outm[256];

    if(true){
        if(solar_control_enabled){
            // Compute the ory position in abs frame
            geo_to_absolute(ory_alt, ory_azi, ory);
            //sprintf(outm, "ORY %f %f %f\n", ory[0], ory[1], ory[2]);
            //telnet.print(outm);
            // Compute sun position
            
            get_time(&year, &month, &day, &hours, &minutes, &seconds);
            hours = 12; minutes = 0; seconds = 0.;
            get_sun_vec(current_lon, current_lat, year, month, day, hours, minutes, seconds, sun);
            //sprintf(outm, "SUN %f %f %f\n", sun[0], sun[1], sun[2]);
            //telnet.print(outm);
            // Just for debug
            //float sun_alt = absolute_to_geo_alt(sun),
            //      sun_azi = absolute_to_geo_azi(sun);
            //sprintf(outm, "SUN %f %f\n", sun_alt, sun_azi);
            //telnet.print(outm);


            get_normal_vec(sun, ory, mir);
            //sprintf(outm, "MIR %f %f %f\n", mir[0], mir[1], mir[2]);
            //telnet.print(outm);
            alt_setpoint = absolute_to_internal_alt(mir) + 90.0;
            azi_setpoint = absolute_to_internal_azi(mir);

            if(azi_setpoint < 0) azi_setpoint += 360.0;
            sprintf(outm, "SC INTERNAL SETP %f %f\n", azi_setpoint, alt_setpoint);
            //telnet.print(outm);
        }

        if(manual_control_enabled || solar_control_enabled){
            if(alt_PID_enabled){
                alt_encoder_val = read_alt_encoder();
                altPID.update();
                set_alt_motor_speed((int8_t) alt_motor_speed);
            }
            if(azi_PID_enabled){
                azi_encoder_val = read_azi_encoder();
                aziPID.update();
                set_azi_motor_speed((int8_t) azi_motor_speed);
            }
            if(abs(aziPID.get_last_error()) < aziPID.get_min_error() && abs(altPID.get_last_error()) < altPID.get_min_error()){
                // We have done, stop moving
                azi_motor_standby();
                alt_motor_standby();
                manual_control_enabled = false;
                solar_control_enabled = false;
            }
        }
    }
}