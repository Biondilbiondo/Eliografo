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

bool MPU_ok = false, ROTF_ok=false;

// Non volatile memory
Preferences HGPrefs; 

class ringPID{
    private:
        // Input should be 0-360
        float *input;
        float *output;
        float *setpoint;

        float ki, kp, kd;
        float outmax, outmin;
        float min_error;
        uint32_t lastT;

    public:
        ringPID(float *In, float *Out, float *SetP){
            set_PID_vars(In, Out, SetP);
            min_error = 0.0;
        }

        void set_PID_params(float Kp, float Ki, float Kd, float OutMin, float OutMax, float MinErr){
            kp = Kp;
            ki = Ki;
            kd = Kd;
            outmax = OutMax;
            outmin = OutMin;
            min_error = MinErr;
        }

        void set_PID_vars(float *In, float *Out, float *SetP){
            input = In;
            output = Out;
            setpoint = SetP;
        }

        void update(void){
            uint32_t now = micros();
            uint32_t timeChange = (now - lastT);
            char outm[256];


            float error = (*setpoint) - (*input);
            if(error > 180.0){
                error -= 360.0;
            }
            if(abs(error) < min_error){
                *output = 0.0;
                return;
            }

            //sprintf(outm, "SETP %f - INP %f = ERR %f\n", *setpoint, *input, error);
            //Controller.write(outm);
            //Serial.print(outm);
            sprintf(outm, "ERR %f MIN ERR %f\n", error, min_error);
            //Controller.write(outm);
            Serial.print(outm);
            float p_comp = kp * error;
            float i_comp = 0.0;
            float d_comp = 0.0;

            *output = p_comp + i_comp + d_comp;
            if(*output > outmax) *output = outmax;
            if(*output < outmin) *output = outmin;


            //sprintf(outm, "ERROR %f P %f D %f I %f OUT %f\n", error, p_comp, d_comp, i_comp, *output);
            //Controller.write(outm);
            Serial.print(outm);

            lastT=now;
        }
};

// Control variables
float azi_setpoint, alt_setpoint,
      azi_encoder_val, alt_encoder_val,
      azi_motor_speed, alt_motor_speed,
      alt_encoder_zero, azi_encoder_zero;

ringPID aziPID(&azi_encoder_val, &azi_motor_speed, &azi_setpoint);
ringPID altPID(&alt_encoder_val, &alt_motor_speed, &alt_setpoint);
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
                          -126, 126,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ki"),
                          get_float_cfg("alt_kd"),
                          -126, 126,
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
    sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%06.4fZ\n", y, m, d, h, mi, s);
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
    float ray[3], outray[3];

    char buf[256];

    float r[3][3];

    initialize_rotation_frame(g, m);
    geo_to_absolute(10.0, 90.0, ray);
    sprintf(buf, "\nABSOLU. VEC: %+6.4f %+6.4f %+6.4f\n", ray[0], ray[1], ray[2]);
    Controller.write(buf);
    internal_frame_get_rotation_matrix(r);
    Serial.printf("Done");
    sprintf(buf, "%+6.4f %+6.4f %+6.4f\n%+6.4f %+6.4f %+6.4f\n%+6.4f %+6.4f %+6.4f\n", r[0][0], r[0][1], r[0][2],
                                                                                       r[1][0], r[1][1], r[1][2], 
                                                                                       r[2][0], r[2][1], r[2][2]);
    Controller.write(buf);
    absolute_to_internal_v(ray, outray);
    sprintf(buf, "\nINTERNAL VEC: %+6.4f %+6.4f %+6.4f\n", outray[0], outray[1], outray[2]);
    Controller.write(buf);

    float alt = absolute_to_internal_alt(ray),
          azi = absolute_to_internal_azi(ray);
    sprintf(buf, "INTERNAL AZI: %05.1f\nINTERNAL ALT: %05.1f\n", azi, alt);
    Controller.write(buf);

    float test_abs[3];
    internal_to_absolute(alt, azi, test_abs);
    sprintf(buf, "\nRECOMP. ABS. VEC: %+6.4f %+6.4f %+6.4f\n", test_abs[0], test_abs[1], test_abs[2]);
    Controller.write(buf);

    sprintf(buf, "\nRECOMP. GEO ALT: %+6.4f\n", internal_to_geo_alt(alt, azi));
    Controller.write(buf);
    sprintf(buf, "\nRECOMP. GEO AZI: %+6.4f\n", internal_to_geo_azi(alt, azi));
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

bool cmd_get_geo(void){
    float lat = get_float_cfg("lat"), 
          lon = get_float_cfg("lon");

    char buf[32];
    sprintf(buf, "LAT: %08.4f LON: %08.4f\n", lat, lon);
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

bool cmd_pid_vals(void){
    char buf[64];
    sprintf(buf, "ALT: SPEED %f VAL %f SET %f\n", alt_motor_speed, alt_encoder_val, alt_setpoint);
    Controller.write(buf);
    sprintf(buf, "AZI: SPEED %f VAL %f SET %f\n", azi_motor_speed, azi_encoder_val, azi_setpoint);
    Controller.write(buf);
    return true;
}

bool cfg_key_exists(const char *key){
    return HGPrefs.isKey(key);
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
        Controller.write(obuf);
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
        Controller.write(obuf);
        return true;
    }
    else{
        sprintf(obuf, "Key %s not found.\n", key);
        Controller.write(obuf);
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
    // TODO
    return true;
}

bool cmd_mirror_log(void){
    char buf[256];
    
    sprintf(buf, "SUN      : %.4f %.4f %.4f\n", sun[_x_], sun[_y_], sun[_z_]);
    Controller.write(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(sun), absolute_to_geo_azi(sun));
    Controller.write(buf);

    sprintf(buf, "OUT-RAY  : %.4f %.4f %.4f\n", ory[_x_], ory[_y_], ory[_z_]);
    Controller.write(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(ory), absolute_to_geo_azi(ory));
    Controller.write(buf);

    sprintf(buf, "MIRROR   : %.4f %.4f %.4f\n", mir[_x_], mir[_y_], mir[_z_]);
    Controller.write(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(mir), absolute_to_geo_azi(mir));
    Controller.write(buf);


    String outs = timeClient.getFormattedDate();
    sprintf(buf, "TIME     : %s\n", outs.c_str());
    Controller.write(buf);
    return true;
}

bool cmd_current_position(void){
    float internal_alt = read_alt_encoder(),
          internal_azi = read_azi_encoder();

    char buf[256];
    
    sprintf(buf, "INTERNAL ALT %.4f AZI %.4f\n", internal_alt, internal_azi);
    Controller.write(buf);
    sprintf(buf, "ABSOLUTE ALT %.4f AZI %.4f\n", internal_to_geo_alt(internal_alt, internal_azi), 
                                                 internal_to_geo_azi(internal_alt, internal_azi));
    Controller.write(buf);
    return true;
}

bool cmd_test_motor(int8_t speed){
    char buf[256];
    
    Controller.write("ALT ENABLE\n");
    alt_motor_enable();
    sleep(1);
    Controller.write("ALT FWD\n");
    set_alt_motor_speed(120);
    sleep(2);
    Controller.write("ALT REV\n");
    set_alt_motor_speed(-120);
    sleep(2);
    Controller.write("ALT STANDBY\n");
    alt_motor_standby();
    sleep(2);

    Controller.write("AZI ENABLE\n");
    azi_motor_enable();
    sleep(1);
    Controller.write("AZI FWD\n");
    set_azi_motor_speed(120);
    sleep(2);
    Controller.write("AZI REV\n");
    set_azi_motor_speed(-120);
    sleep(2);
    Controller.write("AZI STANDBY\n");
    azi_motor_standby();
    sleep(2);

    return true;
}

bool cmd_alt_goto(char *buf){
    float dest, pos;
    char outm[256];
    sscanf(buf, "%f", &dest);
    pos = read_alt_encoder();
    sprintf(outm, "GOING TO ALT %.4f\n", pos);
    Controller.write(outm);
    if(pos > dest){
        for(int i=0; i<1000;i++){
            set_alt_motor_speed(-120);
            delay(100);
            alt_motor_standby();
            pos = read_alt_encoder();
            sprintf(outm, "INTERNAL ALT %.4f\n", pos);
            Controller.write(outm);
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
            Controller.write(outm);
            if(pos > dest) break;
        }
    }
    return true;
}

bool cmd_alt_pid_setpoint(char *buf){
    float dest;
    char outm[256];
    sscanf(buf, "%f", &dest);
    alt_setpoint = dest;
    sprintf(outm, "GOING TO ALT %.4f\n", dest);
    Controller.write(outm);
    alt_PID_enabled = true;

    return true;
}

bool cmd_alt_pid_disable(void){
    alt_PID_enabled = false;
    alt_motor_standby();

    return true;
}

bool cmd_reconfigure(void){
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
    else if(strcmp(tok, "test-rf") == 0){
        // TODO REMOVE
        return cmd_test_rotframe();
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
        Controller.stop();
        return true;
    }
    else if(strcmp(tok, "current-position") == 0){
        return cmd_current_position();
    }
    else if(strcmp(tok, "test-motors") == 0){
        return cmd_test_motor(128);
    }
    else if(strcmp(tok, "alt-fwd") == 0){
        Controller.write("ALT FWD\n");
        set_alt_motor_speed(120);
        sleep(2);
        Controller.write("ALT STANDBY\n");
        alt_motor_standby();
        return true;
    }
    else if(strcmp(tok, "alt-bck") == 0){
        Controller.write("ALT REV\n");
        set_alt_motor_speed(-120);
        sleep(2);
        Controller.write("ALT STANDBY\n");
        alt_motor_standby();
        return true;
    }
    else if(strcmp(tok, "azi-fwd") == 0){
        Controller.write("AZI FWD\n");
        set_azi_motor_speed(120);
        sleep(2);
        Controller.write("AZI STANDBY\n");
        azi_motor_standby();
        return true;
    }
    else if(strcmp(tok, "azi-bck") == 0){
        Controller.write("AZI REV\n");
        set_azi_motor_speed(-120);
        sleep(2);
        Controller.write("AZI STANDBY\n");
        azi_motor_standby();
        return true;
    }
    else if(strcmp(tok, "alt-goto") == 0){
        return cmd_alt_goto(rest);
    }
    else if(strcmp(tok, "alt-pid-setpoint") == 0){
        return cmd_alt_pid_setpoint(rest);
    }
    else if(strcmp(tok, "alt-pid-disable") == 0){
        return cmd_alt_pid_disable();
    }
    else if(strcmp(tok, "pid-vals") == 0){
        return cmd_pid_vals();
    }
    else if(strcmp(tok, "reconfigure") == 0){
        return cmd_pid_vals();
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
    //get_sun_vec(get_float_cfg("lon"), get_float_cfg("lat"), year, month, day, hours, minutes, seconds, sun);
    if(alt_PID_enabled){
        alt_encoder_val = read_alt_encoder();
        altPID.update();
        //Serial.printf("ALT SPEED %f %d\n", alt_motor_speed, (int8_t) alt_motor_speed);
        set_alt_motor_speed((int8_t) alt_motor_speed);
    }
    if(azi_PID_enabled){
        azi_encoder_val = read_azi_encoder();
        aziPID.update();
        set_azi_motor_speed((int8_t) azi_motor_speed);
    }

    //serial_log();
    delay(100); // Wait for 1 second
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
    initialize_rotation_frame(g, m);
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

    float alt =  pos.altitudeRefract/ R2D,
          az  = pos.azimuthRefract / R2D;

    //Serial.printf("SUN alt %f azi %f\n", pos.altitudeRefract, pos.azimuthRefract);
    /*sun[_x_] = cos(PI/2.0 - az) * cos(alt);
    sun[_y_] = sin(PI/2.0 - az) * cos(alt);
    sun[_z_] = sin(alt);*/
    //Serial.printf("Sun vector %f %f %f\n", sun[_x_], sun[_y_], sun[_z_]);
    geo_to_absolute(pos.altitudeRefract, pos.azimuthRefract, sun);
}

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
        ledcWrite(ALT_MOTOR_PWM, 255);
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
        ledcWrite(AZI_MOTOR_PWM, 255);
    }
}

void azi_motor_standby(void){
    azi_PID_enabled = false;
    ledcWrite(AZI_MOTOR_PWM_CH, 0);
    digitalWrite(AZI_MOTOR_DIR1, LOW);
    digitalWrite(AZI_MOTOR_DIR2, LOW);
}

void alt_motor_standby(void){
    alt_PID_enabled = false;
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
    if(strcmp(k, "azie_me") == 0)
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

float read_azi_encoder(void){
    uint32_t mv = analogReadMilliVolts(AZI_ENCODER);
    // This is actual value in internal frame;
    // NOTE: it should be in degrees and increasing ccw
    float f_v = (float) (mv) * (ENCODER_R1 + ENCODER_R2) / ENCODER_R2 ; // Real tension value
    f_v /= 1000.0;
    float deg = f_v * ENCODER_VOLT_TO_DEG - azi_encoder_zero;
    if(deg < 0) deg+=360;
    return deg; // Just to put something here
}

float read_alt_encoder(void){
    float f_v = 0.0;
    for(int i=0; i < ALT_ENCODER_OVERSAMPLING; i++)
        f_v += (float) analogReadMilliVolts(ALT_ENCODER);
    f_v /= ALT_ENCODER_OVERSAMPLING;
    // This is actual value in internal frame;
    // NOTE: it should be in degrees and increasing rotating upward
    
    f_v *= (ENCODER_R1 + ENCODER_R2) / ENCODER_R2 ; // Real tension value
    f_v /= 1000.0;
    float deg = f_v * ENCODER_VOLT_TO_DEG - alt_encoder_zero;
    if(deg < 0) deg+=360;
    return deg; // Just to put something here
}