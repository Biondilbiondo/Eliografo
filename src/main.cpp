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

#ifdef USE_MPU6050
Adafruit_MPU6050 mpu;
#endif
#ifdef USE_MPU9250
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#endif

// Rotation matrix initialized with identity
float *rf[3], _rf[9] = {1., 0., 0.,
                        0., 1., 0.,
                        0., 0., 1.}; 
bool MPU_ok = false, ROTF_ok=false;

// Non volatile memory
Preferences HGPrefs; 

// Control variables
float azi_setpoint, alt_setpoint,
      azi_encoder_val, alt_encoder_val,
      azi_motor_speed, alt_motor_speed;

QuickPID aziPID(&azi_encoder_val, &azi_motor_speed, &azi_setpoint);
QuickPID altPID(&alt_encoder_val, &alt_motor_speed, &alt_setpoint);
bool azi_PID_enabled = false, alt_PID_enabled = false;
hw_timer_t *PID_timer_cfg = NULL;

float sun[3];
float mir[3];
float ory[3];

void setup_remote_com(void){
    ComServer.begin();
}

void setup_pref(void){
    HGPrefs.begin("hg", PREF_RW_MODE);
}

#ifdef USE_MPU6050
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
    if(MPU_ok){
        rf[0] = &(_rf[0]);
        rf[1] = &(_rf[3]);
        rf[2] = &(_rf[6]);
        MPU_update_rot_frame(rf);
        ROTF_ok = true;
    }
}
#endif

#ifdef USE_MPU9250
void setup_accell_compass_MPU9250(void){
    // TODO
    // Check examples from https://github.com/jfredine/Adafruit_MPU9250/
}
#endif

void setup_motors(){
    pinMode(ALT_MOTOR_DIR1, OUTPUT);
    pinMode(ALT_MOTOR_DIR2, OUTPUT);
    ledcSetup(ALT_MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(ALT_MOTOR_PWM, ALT_MOTOR_PWM_CH);

    pinMode(AZI_MOTOR_DIR1, OUTPUT);
    pinMode(AZI_MOTOR_DIR2, OUTPUT);
    ledcSetup(AZI_MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(AZI_MOTOR_PWM, AZI_MOTOR_PWM_CH);

    azi_encoder_val = read_azi_encoder();
    alt_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    alt_setpoint = alt_encoder_val;
    aziPID.SetTunings(get_float_cfg("azi_kp"),
                      get_float_cfg("azi_ki"),
                      get_float_cfg("azi_kd"));
    aziPID.SetOutputLimits(-126, 126);
#ifdef AZI_REVERSED
    aziPID.SetControllerDirection(QuickPID::Action::reverse);
#endif
    altPID.SetTunings(get_float_cfg("alt_kp"),
                      get_float_cfg("alt_ki"),
                      get_float_cfg("alt_kd"));
    altPID.SetOutputLimits(-126, 126);
    // uncomment to reverse motor direction
#ifdef ALT_REVERSED
    altPID.SetControllerDirection(QuickPID::Action::reverse);
#endif

    PID_timer_cfg = timerBegin(0, 40000, true);
    timerAttachInterrupt(PID_timer_cfg, &PID_isr, true);
    timerAlarmWrite(PID_timer_cfg, PID_INTERRUPT_MS, true);
    timerAlarmEnable(PID_timer_cfg);

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

    setup_pref();
    setup_wifi();
    setup_ntp();
    setup_rtc();
#ifdef USE_MPU6050
    setup_accell_compass_MPU6050();
#endif 
#ifdef USE_MPU9250
    setup_accell_compass_MPU9250();
#endif
    setup_remote_com();
    setup_motors();
}

uint16_t com_get_next_cmd(char *buf, uint16_t minbuf, uint16_t maxbuf){
    uint16_t i;
    for(i = minbuf; i < maxbuf && (buf[i] = Controller.read()) != '\n' && buf[i] != -1; i++)
        ;//Serial.printf("%d %d %d\n", i, minbuf, maxbuf);
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
    float m[] = {1.0, 1.0, 0.0};
    float g[] = {0.0, 0.0, -1.0};
    float ray[] = {3.2, 5.7, 1.4};
    float outray[3];

    char buf[256];

    rf[0] = &(_rf[0]);
    rf[1] = &(_rf[3]);
    rf[2] = &(_rf[6]);

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

bool cmd_set_geo(char *buf){
    float lat = get_float_cfg("lat"), 
          lon = get_float_cfg("lon");

    sscanf(buf, "%f%f", &lat, &lon);
    HGPrefs.putFloat("lat", lat);
    HGPrefs.putFloat("lon", lon);
    return true;
}

bool cmd_set(char *buf){
    float val;
    char *key, *rest;

    key = strtok_r(NULL, " \n\r", &buf);
    rest = strtok_r(NULL, "\r\n", &buf);
    sscanf(rest, "%f", &val);
    HGPrefs.putFloat(key, val);
    return true;
}

bool cmd_get_geo(void){
    float lat = get_float_cfg("lat"), 
          lon = get_float_cfg("lon");

    char buf[32];
    sprintf(buf, "LAT: %08.4f, LON: %08.4f\n", lat, lon);
    Controller.write(buf);
    return true;
}

bool cmd_pid_prm(void){
    char buf[64];
    sprintf(buf, "ALT: P %8.1f I %8.1f D %8.1f\n", get_float_cfg("alt_kp"), get_float_cfg("alt_ki"), get_float_cfg("alt_kd"));
    Controller.write(buf);
    sprintf(buf, "AZI: P %8.1f I %8.1f D %8.1f\n", get_float_cfg("azi_kp"), get_float_cfg("azi_ki"), get_float_cfg("azi_kd"));
    Controller.write(buf);
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
        // TODO REMOVE
        return cmd_test_rotframe();
    }
    else if(strcmp(tok, "set-geo") == 0){
        return cmd_set_geo(rest);
    }
    else if(strcmp(tok, "set") == 0){
        //tok = strtok_r(NULL, delim, &rest);
        //Serial.printf("%s\n", tok);
        return cmd_set(rest);
    }
    else if(strcmp(tok, "get-geo") == 0){
        return cmd_get_geo();
    }
    else if(strcmp(tok, "pid-prm") == 0){
        return cmd_pid_prm();
    }
    else if(strcmp(tok, "quit") == 0){
        Controller.stop();
        return true;
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

    //Serial.println("PORCODIO x");
    //Serial.flush();
    if(Controller.available()){
        buf_last = com_get_next_cmd(cmd_buf, buf_last, CMD_BUF_LEN-1);
        if(buf_last == CMD_BUF_LEN-1){
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
            com_get_next_cmd(cmd_buf, (uint16_t) 0, CMD_BUF_LEN-1);
        }
    }
    //get_time(&year, &month, &day, &hours, &minutes, &seconds);
    //sprintf(buf, "%d-%d-%d %d:%d:%.1f UTC\n", year, month, day, hours, minutes, seconds);
    //Serial.println(buf);
    get_sun_vec(get_float_cfg("lon"), get_float_cfg("lat"), year, month, day, hours, minutes, seconds, sun);
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

void MPU_update_rot_frame(float **rf){
    float m[3], g[3];
    // TODO Read m and g from the sensor
    // ...
    compute_frame_rotation(g, m, rf);
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
      
    sun[_x_] = cos(PI/2.0 - az) * cos(alt);
    sun[_y_] = sin(PI/2.0 - az) * cos(alt);
    sun[_z_] = sin(alt);
}

void set_alt_motor_speed(int8_t speed){
    if(speed > 0){
        // Forward motion
        ledcWrite(ALT_MOTOR_PWM, 0);
        digitalWrite(ALT_MOTOR_DIR1, LOW);
        digitalWrite(ALT_MOTOR_DIR2, HIGH);
        ledcWrite(ALT_MOTOR_PWM, abs(speed)*2);
    }
    else if(speed < 0){
        // Bakward motion
        ledcWrite(ALT_MOTOR_PWM, 0);
        digitalWrite(ALT_MOTOR_DIR1, HIGH);
        digitalWrite(ALT_MOTOR_DIR2, LOW);
        ledcWrite(ALT_MOTOR_PWM, abs(speed)*2);
    }
    else{
        // Blocked 
        ledcWrite(ALT_MOTOR_PWM, 0);
        digitalWrite(ALT_MOTOR_DIR1, LOW);
        digitalWrite(ALT_MOTOR_DIR2, LOW);
        ledcWrite(ALT_MOTOR_PWM, 255);
    }
}

void set_azi_motor_speed(int8_t speed){
    if(speed > 0){
        // Forward motion
        ledcWrite(AZI_MOTOR_PWM, 0);
        digitalWrite(AZI_MOTOR_DIR1, LOW);
        digitalWrite(AZI_MOTOR_DIR2, HIGH);
        ledcWrite(AZI_MOTOR_PWM, abs(speed)*2);
    }
    else if(speed < 0){
        // Bakward motion
        ledcWrite(AZI_MOTOR_PWM, 0);
        digitalWrite(AZI_MOTOR_DIR1, HIGH);
        digitalWrite(AZI_MOTOR_DIR2, LOW);
        ledcWrite(AZI_MOTOR_PWM, abs(speed)*2);
    }
    else{
        // Blocked 
        ledcWrite(AZI_MOTOR_PWM, 0);
        digitalWrite(AZI_MOTOR_DIR1, LOW);
        digitalWrite(AZI_MOTOR_DIR2, LOW);
        ledcWrite(AZI_MOTOR_PWM, 255);
    }
}

void azi_motor_standby(void){
    azi_PID_enabled = false;
    ledcWrite(AZI_MOTOR_PWM, 0);
    digitalWrite(AZI_MOTOR_DIR1, LOW);
    digitalWrite(AZI_MOTOR_DIR2, LOW);
}

void alt_motor_standby(void){
    alt_PID_enabled = false;
    ledcWrite(ALT_MOTOR_PWM, 0);
    digitalWrite(ALT_MOTOR_DIR1, LOW);
    digitalWrite(ALT_MOTOR_DIR2, LOW);
}

void azi_motor_enable(void){
    azi_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    aziPID.Initialize();
    azi_PID_enabled = true;
}

void alt_motor_enable(void){
    alt_encoder_val = read_alt_encoder();
    alt_setpoint = alt_encoder_val;
    altPID.Initialize();
    alt_PID_enabled = true;
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
    return 0.0;
}

float get_float_cfg(const char *k){
    if(!HGPrefs.isKey(k))
        HGPrefs.putFloat(k, get_float_default_cfg(k));
    return HGPrefs.getFloat(k);
}

float read_azi_encoder(void){
    uint32_t mv = analogReadMilliVolts(AZI_ENCODER);
    // This should be elaborated to return radians 
    return mv / 3300 * 2 * PI; // Just to put something here
}

float read_alt_encoder(void){
    uint32_t mv = analogReadMilliVolts(ALT_ENCODER);
    // This should be elaborated to return radians 
    return mv / 3300 * 2 * PI; // Just to put something here
}

void IRAM_ATTR PID_isr(void){
    if(alt_PID_enabled){
        alt_encoder_val = read_alt_encoder();
        altPID.Compute();
        set_alt_motor_speed(alt_motor_speed);
    }
    if(azi_PID_enabled){
        azi_encoder_val = read_azi_encoder();
        aziPID.Compute();
        set_alt_motor_speed(azi_motor_speed);
    }
}