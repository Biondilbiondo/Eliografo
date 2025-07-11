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

ESP32Time internal_rtc;
RTC_DS1307 rtc;
bool RTC_ok = false, internal_RTC_ok = false;

#ifdef USE_MPU6050
Adafruit_MPU6050 mpu;
#endif
#ifdef USE_MPU9250
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#endif

bool MPU_ok = false, ROTF_ok=false;

Adafruit_ADS1115 external_adc;
bool external_ADC_ok = false;

// Non volatile memory
Preferences HGPrefs; 

// Daily Tasks
#define MAX_DAILY_TASKS 32
#define SCH_TIME_DEL 1
uint64_t sch_timestamp[MAX_DAILY_TASKS];
bool sch_run[MAX_DAILY_TASKS] = {false};
uint32_t task_cnt = 0;

// Scene variables
#define SCENE_LEN 120
#define SCENE_DT .25
#define MAX_SCENES 16

uint8_t scene_cnt;
float scenes[MAX_SCENES][SCENE_LEN*2] = {{0}};
int scene_len[MAX_SCENES];
String scenes_name[MAX_SCENES];

// Control variables
bool driver_on = false;
float azi_setpoint, alt_setpoint,
      azi_encoder_val, alt_encoder_val,
      azi_motor_speed, alt_motor_speed,
      alt_encoder_zero, azi_encoder_zero;
float pid_watchdog_t0, pid_watchdog_elap;
float azi_motor_min_s, azi_motor_max_sv, azi_motor_min_sv, azi_motor_m_s;
float alt_motor_min_s, alt_motor_max_sv, alt_motor_min_sv, alt_motor_m_s;

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

int bootn = 0;

float logs[1000];
int logs_cnt = 0;

// Time/RTC Routines
void update_time_from_NTP(void){
    while(!timeClient.update()) {
        timeClient.forceUpdate();
    }
}

bool sync_internal_rtc(void){
    if(!RTC_ok) return false;
    internal_rtc.setTime(rtc.now().unixtime());
    internal_RTC_ok = true;
    return true;
}

bool sync_RTC_from_NTP(void){
    if(NTP_ok && RTC_ok){
        update_time_from_NTP();
        DateTime now(timeClient.getEpochTime());
        rtc.adjust(now);
        return sync_internal_rtc();
    }
    return false;
}

bool sync_internal_rtc_from_NTP(void){
    if(NTP_ok){
        update_time_from_NTP();
        internal_rtc.setTime(timeClient.getEpochTime());
        internal_RTC_ok = true;
        return true;
    }
    return false;
}

float get_timestamp_RTC(void){
    return (float) internal_rtc.getHour() * 3600. + (float) internal_rtc.getMinute() * 60 + internal_rtc.getSecond() + internal_rtc.getMicros() / 1e6;
}

void get_time_RTC(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
    if(!driver_on){
        *hours = 0;
        *minutes = 0;
        *seconds = 0.;
        *day = 0;
        *month = 0;
        *year = 0;
        return;
    }

    DateTime now = rtc.now();
    *hours = now.hour();
    *minutes = now.minute();
    *seconds = (float) now.second();
    *day = now.day();
    *month = now.month();
    *year = now.year();
}

void get_time_internal_RTC(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
    *hours = internal_rtc.getHour(true);
    *minutes = internal_rtc.getMinute();
    *seconds = (float) internal_rtc.getSecond() + (float) internal_rtc.getMillis() / 1000.0;
    *day = internal_rtc.getDay();
    *month = internal_rtc.getMonth() + 1;
    *year = internal_rtc.getYear();
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

void get_time(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hours, uint8_t *minutes, float *seconds){
    if(RTC_ok && driver_on){
        get_time_RTC(year, month, day, hours, minutes, seconds);
        return;
    }
    if(internal_RTC_ok){
        get_time_internal_RTC(year, month, day, hours, minutes, seconds);
        return;
    }
    if(NTP_ok){
        get_time_NTP(year, month, day, hours, minutes, seconds);
        return;
    }

    *hours = 0;
    *minutes = 0;
    *seconds = 0.;
    *day = 1;
    *month = 1;
    *year = 1970;
}

// Scheduled tasks
void add_scheduled_task(uint32_t h, uint32_t m, uint32_t s){
    if(task_cnt < MAX_DAILY_TASKS){
        sch_timestamp[task_cnt] = h * 3600 + m * 60 + s;
        task_cnt++;
    }
}

void schedule_task_loop(void){
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    float seconds;
    get_time(&year, &month, &day, &hours, &minutes, &seconds);
    float timestamp = hours * 3600 + minutes * 60 + seconds;
    for(int i=0; i<task_cnt; i++){
        if(sch_run[i]){ 
            // Reload
            if(abs(timestamp - (float) sch_timestamp[i]) > 2 * SCH_TIME_DEL) sch_run[i] = false;
            // Skip
            continue;
        }

        if(abs(timestamp - (float) sch_timestamp[i]) < SCH_TIME_DEL){
            /*telnet.print("Running daily task\n");
            telnet.printf("%02d at %02d:%02d:%02d\n", i, hours, minutes, (int) seconds);*/
            wifi_on();
            sch_run[i] = true;
        }   
    }
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
    
    if(strcmp(k, "azi_ms") == 0)
        return DEFAULT_MIN_SPEED;
    if(strcmp(k, "alt_ms") == 0)
        return DEFAULT_MIN_SPEED;

    if(strcmp(k, "azi_msv") == 0)
        return DEFAULT_MIN_SPEED_VALUE;
    if(strcmp(k, "alt_msv") == 0)
        return DEFAULT_MIN_SPEED_VALUE;

    if(strcmp(k, "azi_Msv") == 0)
        return DEFAULT_MAX_SPEED_VALUE;
    if(strcmp(k, "alt_Msv") == 0)
        return DEFAULT_MAX_SPEED_VALUE;

    if(strcmp(k, "azi_sm") == 0)
        return DEFAULT_M_SPEED;
    if(strcmp(k, "alt_sm") == 0)
        return DEFAULT_M_SPEED;

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
    float norm = in[_x_] * out[_x_] + \
                 in[_y_] * out[_y_] + \
                 in[_z_] * out[_z_];

    norm = - 2.0 * sqrt((1+norm)/2.0);
    for(int i=0; i < 3; i++)
        mir[i] = (-in[i] - out[i])/norm; 
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

void update_sun_vec(void){
    uint16_t year;
    uint8_t month, day, hours, minutes;
    float seconds; 
            
    get_time(&year, &month, &day, &hours, &minutes, &seconds);
    get_sun_vec(current_lon, current_lat, year, month, day, hours, minutes, seconds, sun);
}

// Motors and Encoders Routine
void motor_driver_enable(void){
    driver_on = true;
    digitalWrite(DRIVER_ENABLE, LOW);
    delay(100);
}

void motor_driver_disable(void){
    driver_on = false;
    digitalWrite(DRIVER_ENABLE, HIGH);
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

void check_external_ADC(void){
    byte error, address;
    Wire.beginTransmission(0x48);
    error = Wire.endTransmission();
    if (error == 0) {
        external_ADC_ok = true;
    }
    else{
        external_ADC_ok = false;
    }
}

float azi_encoder_to_degrees(uint32_t val){
    return (float) map(val, 0, 2500, 0, 360);
}

float alt_encoder_to_degrees(uint32_t val){
    return (float) map(val, 0, 2500, 0, 360);
}

float read_azi_encoder(void){
    float f_v = 0.0, val, del, first;
    if(!external_ADC_ok) return 0.0;
    for(int i=0; i < ENCODER_OVERSAMPLING; i++){
        val = azi_encoder_to_degrees((uint32_t) (external_adc.computeVolts(external_adc.readADC_SingleEnded(0))*1000.));
        if(i==1){
            f_v += val;
            first = val;
        }
        else{
            del = first - val;
            if(del < -300){
                val -= 360;
            }
            if(del > 300){
                val += 360;
            }
            f_v += val;
        }
        delay(2);
    }

    f_v /= ENCODER_OVERSAMPLING;
    float deg = f_v - azi_encoder_zero;
    if(deg < 0) deg+=360;
    if(deg > 360) deg -= 360;
    return deg; // Just to put something here
}

float read_alt_encoder(void){
    float f_v = 0.0, val, del, first;
    if(!external_ADC_ok) return 0.0;
    for(int i=0; i < ENCODER_OVERSAMPLING; i++){

        val = alt_encoder_to_degrees((uint32_t) (external_adc.computeVolts(external_adc.readADC_SingleEnded(1))*1000.));
        if(i==1){
            f_v += val;
            first = val;
        }
        else{
            del = first - val;
            if(del < -300){
                val -= 360;
            }
            if(del > 300){
                val += 360;
            }
            f_v += val;
            }
        delay(2);
    }

    f_v /= ENCODER_OVERSAMPLING;
    float deg = f_v - alt_encoder_zero;
    if(deg < 0) deg+=360;
    if(deg > 360) deg -= 360;
    return deg; // Just to put something here
}

void start_pid(void){
    alt_PID_enabled = true;
    azi_PID_enabled = true;
}

void set_pid_watchdog(uint32_t watchdog_time_s){
    pid_watchdog_t0 = get_timestamp_RTC();
    pid_watchdog_elap = watchdog_time_s;
}

void reset_pid_watchdog(void){
    pid_watchdog_t0 = -1;
    pid_watchdog_elap = -1;
}

void pid_loop(void){
    if(alt_PID_enabled){
        alt_encoder_val = read_alt_encoder();
        //sprintf(outm, "ALT ENC %f SETP %f\n", alt_encoder_val, alt_setpoint);
        //telnet.print(outm);
        altPID.update();
        set_alt_motor_speed((int8_t) alt_motor_speed);
    }
    if(azi_PID_enabled){
        azi_encoder_val = read_azi_encoder();
        //sprintf(outm, "AZI ENC %f SETP %f\n", azi_encoder_val, azi_setpoint);
        //telnet.print(outm);
        aziPID.update();
        set_azi_motor_speed((int8_t) azi_motor_speed);
    }
    if(abs(aziPID.get_last_error()) < aziPID.get_min_error() && abs(altPID.get_last_error()) < altPID.get_min_error()){
        // We have done, stop moving
        //telnet.print("Disabled\n");
        azi_motor_standby();
        alt_motor_standby();
        azi_PID_enabled = false;
        alt_PID_enabled = false;
    }
    if(pid_watchdog_t0 > 0 && pid_watchdog_elap > 0){
        float dt = get_timestamp_RTC() - pid_watchdog_t0;
        if(dt > pid_watchdog_elap || dt < 0){
            azi_motor_standby();
            alt_motor_standby();
            azi_PID_enabled = false;
            alt_PID_enabled = false;
            telnet.print("Killed by watchdog!\n");
        }
    }
}

bool calibrate_speed(void){
#define TIME_CAL 2
#define FAST_INCREMENT 10
#define SLOW_INCREMENT 2
    int16_t speed;
    float a0, a1, t0, t1, sfwd, srev, smean, estimated_m;
    int16_t minspeed = -1, mcnt;
    float minspeed_val, maxspeed_val;
    bool alt_reversed, azi_reversed;
    motor_driver_enable();

    for(int i=0; i < 10; i++){
        t0 = get_timestamp_RTC();
        a0 = read_alt_encoder();
        t1 = get_timestamp_RTC();
        smean += t1-t0;
    }
    telnet.printf("Estimated time for encoder read %fs\n", smean/10);

    // Check motor/encoder polarity
    a0 = read_alt_encoder();
    t0 = get_timestamp_RTC();
    set_alt_motor_speed(127);
    delay(TIME_CAL * 1000);
    set_alt_motor_speed(0);
    t1 = get_timestamp_RTC();
    a1 = read_alt_encoder();
    if((a1-a0)/(t1-t0) < 0){
        telnet.printf("Alt motor or encoder is reversed.\n");
        telnet.printf("If mirror moved upword, reverse encoder.\n");
        telnet.printf("If mirror moved downward, reverse motor.\n");
        alt_reversed = true;
    }
    else{
        telnet.printf("Alt motor and encoder are OK.\n");
        telnet.printf("If mirror moved downword, reverse axis.\n");
        alt_reversed = false;
    }
    
    delay(1000);

    a0 = read_azi_encoder();
    t0 = get_timestamp_RTC();
    set_azi_motor_speed(127);
    delay(TIME_CAL * 1000);
    set_azi_motor_speed(0);
    t1 = get_timestamp_RTC();
    a1 = read_azi_encoder();
    if((a1-a0)/(t1-t0) < 0){
        telnet.printf("Azi motor or encoder is reversed.\n");
        telnet.printf("If mirror moved clockwise, reverse encoder.\n");
        telnet.printf("If mirror moved counterclockwise, reverse motor.\n");
        azi_reversed = true;
    }
    else{
        telnet.printf("Azi motor and encoder are OK.\n");
        telnet.printf("If mirror moved counterclockwise, reverse axis.\n");
        azi_reversed = false;
    }

    if(alt_reversed || azi_reversed){
        motor_driver_disable();
        return false;
    }


    // Go to 90 degrees to simplify calculations...
    alt_setpoint = 90.;
    azi_setpoint = 90.;
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    delay(1000);

    int16_t increment = FAST_INCREMENT;
    maxspeed_val = 0.;
    mcnt = 0;
    for(speed=0; speed < 128; speed += increment){
        a0 = read_alt_encoder();
        t0 = get_timestamp_RTC();
        set_alt_motor_speed(speed);
        delay(TIME_CAL * 1000);
        set_alt_motor_speed(0);
        t1 = get_timestamp_RTC();
        a1 = read_alt_encoder();
        sfwd = (a1-a0)/(t1-t0);

        //telnet.printf("(FWD %d) A1 %.1f A0 %.1f\n", speed, a1, a0);
        telnet.printf("(FWD %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        //telnet.printf("%+03d %.2f\n", speed, sfwd);
        a0 = a1;
        t0 = get_timestamp_RTC();
        set_alt_motor_speed(-speed);
        delay(TIME_CAL * 1000);
        set_alt_motor_speed(0);
        t1 = get_timestamp_RTC();
        a1 = read_alt_encoder();
        //telnet.printf("(REV %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        srev = (a1-a0)/(t1-t0);

        //telnet.printf("(REV %d) A1 %.1f A0 %.1f\n", speed, a1, a0);
        telnet.printf("(REV %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        //telnet.printf("%+03d %.2f\n", -speed, srev);
        smean = (sfwd - srev) / 2;

        if(smean > MIN_ALLOWED_SPEED && minspeed < 0){
            minspeed = speed;
            minspeed_val = smean;
            increment = SLOW_INCREMENT;
            //break;
        }
        if(minspeed > 0 && speed > 122){
            maxspeed_val += smean;
            mcnt++;
        }
    }
    
    maxspeed_val /= mcnt;
    estimated_m = (maxspeed_val-minspeed_val) / (127-minspeed);
    telnet.printf("Alt Min speed: %d %.2f\nAlt Max speed %d %.2f\nAlt Estimated m: %f\n", minspeed, minspeed_val, 127, maxspeed_val, estimated_m);
    HGPrefs.putFloat("alt_ms", minspeed);
    HGPrefs.putFloat("alt_msv", minspeed_val);
    HGPrefs.putFloat("alt_Msv", maxspeed_val);
    HGPrefs.putFloat("alt_sm", estimated_m);

    minspeed = -1;
    increment = FAST_INCREMENT;
    maxspeed_val = 0.;
    mcnt = 0;
    for(speed=0; speed < 128; speed += increment){
        a0 = read_azi_encoder();
        t0 = get_timestamp_RTC();
        set_azi_motor_speed(speed);
        delay(TIME_CAL * 1000);
        set_azi_motor_speed(0);
        t1 = get_timestamp_RTC();
        a1 = read_azi_encoder();
        sfwd = (a1-a0)/(t1-t0);

        //telnet.printf("(FWD %d) A1 %.1f A0 %.1f\n", speed, a1, a0);
        telnet.printf("(FWD %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        //telnet.printf("%+03d %.2f\n", speed, sfwd);
        a0 = a1;
        t0 = get_timestamp_RTC();
        set_azi_motor_speed(-speed);
        delay(TIME_CAL * 1000);
        set_azi_motor_speed(0);
        t1 = get_timestamp_RTC();
        a1 = read_azi_encoder();
        //telnet.printf("(REV %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        srev = (a1-a0)/(t1-t0);

        //telnet.printf("(REV %d) A1 %.1f A0 %.1f\n", speed, a1, a0);
        telnet.printf("(REV %d) Distance %.1f deg, time %.2f s, speed %.1f deg/s\n", speed, a1-a0, t1-t0, (a1-a0)/(t1-t0));
        //telnet.printf("%+03d %.2f\n", -speed, srev);
        smean = (sfwd - srev) / 2;

        if(smean > MIN_ALLOWED_SPEED && minspeed < 0){
            minspeed = speed;
            minspeed_val = smean;
            increment = SLOW_INCREMENT;
            //break;
        }
        if(minspeed > 0 && speed > 122){
            maxspeed_val += smean;
            mcnt++;
        }
    }
    
    maxspeed_val /= mcnt;
    estimated_m = (maxspeed_val-minspeed_val) / (127-minspeed);
    telnet.printf("Azi Min speed: %d %.2f\nAzi Max speed %d %.2f\nAzi Estimated m: %f\n", minspeed, minspeed_val, 127, maxspeed_val, estimated_m);
    
    HGPrefs.putFloat("azi_ms", minspeed);
    HGPrefs.putFloat("azi_msv", minspeed_val);
    HGPrefs.putFloat("azi_Msv", maxspeed_val);
    HGPrefs.putFloat("azi_sm", estimated_m);

    motor_driver_disable();
    return true;
}

void ray_to_setpoints(float alt, float azi){
    geo_to_absolute(alt, azi, ory);
    update_sun_vec();
    get_normal_vec(sun, ory, mir);
    alt_setpoint = absolute_to_geo_alt(mir);
    azi_setpoint = absolute_to_geo_azi(mir);
    if(azi_setpoint < 0) azi_setpoint += 360.0;
}

// littleFS routine
String *list_dir(const char *dirname){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = LittleFS.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return NULL;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return NULL;
    }

    File file = root.openNextFile();
    uint32_t nfiles = 0;
    while(file){
        nfiles ++;
        file = root.openNextFile();
    }
    
    root.rewindDirectory();
    String *fnames = new String[nfiles+1];
    for(int i=0; i < nfiles; i++){
        file = root.openNextFile();
        fnames[i] = file.name();
    }
    fnames[nfiles] = "<<END>>";
    root.close();

    return fnames;
}

bool write_scene_on_file(int s){
    Serial.printf("Writing file: %s\r\n", "/scenes/"+scenes_name[s]);

    File file = LittleFS.open("/scenes/"+scenes_name[s], FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return false;
    }
    for(int i=0; i < scene_len[s]; i++){
        file.printf("%05.1f %05.1f\n", scenes[s][i*2], scenes[s][i*2+1]);
    }
    file.close();
    return true;
}

int create_new_empty_scene(const char *name){
    if(scene_cnt >= MAX_SCENES) return -1;
    for(int i=0; i < scene_cnt; i++){
        if(scenes_name[i] == name) return -1;
    }
    scenes_name[scene_cnt] = name;
    for(int i=0; i < SCENE_LEN*2; i++)
        scenes[scene_cnt][i] = 0.;
    scene_len[scene_cnt] = 0;
    write_scene_on_file(scene_cnt);
    scene_cnt++;
    return scene_cnt-1;
}

bool remove_file(String name){
    Serial.printf("Deleting file: %s\r\n", name.c_str());
    if(LittleFS.remove(name.c_str())){
        Serial.println("- file deleted");
        return true;
    } else {
        Serial.println("- delete failed");
        return false;
    }
}

bool remove_scene(int s){
    String fname;
    if(s >= scene_cnt || scene_cnt < 0){
        return false;
    }
    if(s == scene_cnt-1){
        scene_len[s] = 0;
        fname = scenes_name[s];
    }
    else{
        fname = scenes_name[s];
        for(int i=s+1; i<scene_cnt; i++){
            scene_len[i-1] = scene_len[i];
            scenes_name[i-1] = scenes_name[i];
            for(int j=0; j<scene_len[i]*2; j++){
                scenes[i-1][j] = scenes[i][j];
            }
        }
    }
    if(!remove_file("/scenes/"+fname)) return false;
    scene_cnt--;
    return true;
}

int load_scene_from_file(const char *path, float *scene_data, int seq_len){
    String s;
    Serial.printf("Reading file: %s\r\n", path);

    File file = LittleFS.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return -1;
    }

    Serial.println("- read from file:");
    int i = 0;
    while(file.available() && i < seq_len){
        s = file.readStringUntil('\n');
        sscanf(s.c_str(), "%f%f", &(scene_data[i*2]), &(scene_data[i*2+1]));
        i++;
    }
    file.close();
    return i;
}

bool load_scenes_from_file(void){
    // Load all the scenes from folder /scene
    String *fnames = list_dir("/scenes");
    for(scene_cnt=0; scene_cnt < MAX_SCENES; scene_cnt++){
        if(fnames[scene_cnt] == "<<END>>") break;
        scenes_name[scene_cnt] = fnames[scene_cnt];
        String path = "/scenes/"+fnames[scene_cnt];
        int l = load_scene_from_file(path.c_str(), &(scenes[scene_cnt][0]), SCENE_LEN);
        scene_len[scene_cnt] = l;
    }
    return true;
}

bool scene_add_frame(int s, float alt, float azi){
    // Load all the scenes from folder /scene
    if(s >= scene_cnt || scene_cnt < 0)  return false;
    if(scene_len[s] >= SCENE_LEN) return false;
    scenes[s][scene_len[s]*2] = alt;
    scenes[s][scene_len[s]*2+1] = azi;
    scene_len[s]++;
    return write_scene_on_file(s);
}

bool run_scene(){
    float now, next;
    float scene[SCENE_LEN*2];
    char outm[250];
    char path[] = "/1.txt";

    if(!load_scene_from_file(path, scene, SCENE_LEN)){
        return false;
    }
    for(int i=0; i < SCENE_LEN; i++){
        sprintf(outm, "%f %f\n", scene[i*2], scene[i*2+1]);
        telnet.print(outm);
    }
    motor_driver_enable();
    ray_to_setpoints(scene[0], scene[1]);
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    //telnet.print("Initial position\n");

    now = get_timestamp_RTC();
    next = now - 1;
    
    int i = 0;
    next = now - 1;
    while(i < SCENE_LEN){
        now = get_timestamp_RTC();
        if(now >= next){
            ray_to_setpoints(scene[i*2], scene[i*2+1]);
            next = now + SCENE_DT;
            start_pid();
            i++;
            //sprintf(outm, "STEP %d\n", i);
            //telnet.print(outm);
        }
        pid_loop();
    }
    while(now < next){
        now = get_timestamp_RTC();
        pid_loop();
    }

    motor_driver_disable();
    return true;
}

bool wifi_off(void){
    if(!WiFi_ok) return false;
    if(telnet.isConnected()) telnet.disconnectClient();
    delay(100);
    telnet.stop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi_ok = false;
    NTP_ok = false;
    return true;
}

bool wifi_on(void){
    if(WiFi_ok) return false;
    setup_wifi();
    if(WiFi_ok) setup_ntp();
    if(WiFi_ok) setup_telnet();
    return true;
}

bool run_pid_test(){
    float alt = 0.0, azi=0.0;

    motor_driver_enable();
    alt_setpoint = 0.0;
    azi_setpoint = 0.0;
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();

    for(alt = 0.0; alt < 360.; alt += 10.){
        telnet.printf("Going to ALT %f\n", alt);
        alt_setpoint = alt;
        start_pid();
        while(alt_PID_enabled || azi_PID_enabled) pid_loop();
        telnet.printf("Done\n");
    }
    telnet.print("Going to ALT 0.0\n");
    alt_setpoint = 0.;
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    telnet.printf("Done\n");

    for(azi = 0.0; azi < 360.; azi += 10.){
        telnet.printf("Going to ALT %f\n", azi);
        azi_setpoint = azi;
        start_pid();
        while(alt_PID_enabled || azi_PID_enabled) pid_loop();
        telnet.printf("Done\n");
    }
    telnet.print("Going to AZI 0.0\n");
    azi_setpoint = 0.;
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    telnet.printf("Done\n");

    return true;
}

bool cmd_system_status(void){
    if(NTP_ok) 
        telnet.println("NTP: OK");
    else
        telnet.println("NTP: NOT OK");
    if(RTC_ok) 
        telnet.println("RTC: OK");
    else
        telnet.println("RTC: NOT OK");
    if(internal_RTC_ok) 
        telnet.println("internal RTC: OK");
    else
        telnet.println("internal RTC: NOT OK");
    if(external_ADC_ok) 
        telnet.println("external ADC: OK");
    else
        telnet.println("external ADC: NOT OK");

    return RTC_ok && internal_RTC_ok && external_ADC_ok;
}
void sleep_for_seconds(uint32_t tts){
    if(tts > MAX_SLEEP_S) tts = MAX_SLEEP_S;
    float t0 = get_timestamp_RTC();
    motor_driver_enable();
    alt_setpoint = 270.;
    azi_setpoint = 0.;
    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    motor_driver_disable();
    /*Go in sleep here*/
    wifi_off();
    uint64_t dt = (uint64_t) ((get_timestamp_RTC() - t0) * 1000000.0F);
    esp_sleep_enable_timer_wakeup(tts * 1000000ULL - dt);
    esp_deep_sleep_start();
}

// Telnet Shell commands
bool cmd_id(void){
    char buf[16];
    sprintf(buf, "%012x\n", chip_id);
    telnet.print(buf);
    return true;
}

bool cmd_sleep(char *buf){
    uint64_t tts;
    sscanf(buf, "%d", &tts);
    sleep_for_seconds(tts);
    return true;
}

bool cmd_list_i2c_devices(void){
    byte error, address;
    int nDevices;
    telnet.println("Scanning...");
    nDevices = 0;
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            telnet.print("I2C device found at address 0x");
            if (address<16) {
                telnet.print("0");
            }
            telnet.println(address,HEX);
            nDevices++;
        }
        else if (error==4) {
            telnet.print("Unknow error at address 0x");
            if (address<16) {
                telnet.print("0");
            }
            telnet.println(address,HEX);
        }    
    }
    if (nDevices == 0) {
        telnet.println("No I2C devices found\n");
    }
    else {
        telnet.println("done\n");
    }
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
    get_time_RTC(&y, &m, &d, &h, &mi, &s);
    sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%06.4fZ\n", y, m, d, h, mi, s);
    telnet.print(buf);
    get_time_internal_RTC(&y, &m, &d, &h, &mi, &s);
    sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%06.4fZ\n", y, m, d, h, mi, s);
    telnet.print(buf);
    sprintf(buf, "TIMESTAMP %f\n", get_timestamp_RTC());
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
    sprintf(buf, "%f\n\n", get_timestamp_RTC());
    telnet.print(buf);
    for(int i=0; i<logs_cnt; i++){
        sprintf(buf, "log: %d %f\n", i, logs[i]);
        telnet.print(buf);
    }
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
    
    update_sun_vec();

    sprintf(buf, "SUN      : %.4f %.4f %.4f\n", sun[_x_], sun[_y_], sun[_z_]);
    telnet.print(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(sun), absolute_to_geo_azi(sun));
    telnet.print(buf);

    sprintf(buf, "OUT-RAY  : %.4f %.4f %.4f\n", ory[_x_], ory[_y_], ory[_z_]);
    telnet.print(buf);
    sprintf(buf, "           ALT: %.4f AZI: %.4f\n", absolute_to_geo_alt(ory), absolute_to_geo_azi(ory));
    telnet.print(buf);

    get_normal_vec(sun, ory, mir);

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
    float alt = read_alt_encoder(),
          azi = read_azi_encoder();

    char buf[256];
    
    sprintf(buf, "ABSOLUTE ALT %.4f AZI %.4f\n", alt, azi);
    telnet.print(buf);
    return true;
}

//TODO: Remove

bool cmd_test_adc_encoder(void){
    int16_t r1 = external_adc.readADC_SingleEnded(1);
    int16_t r0 = external_adc.readADC_SingleEnded(0);
    float v1 = external_adc.computeVolts(r1);
    float v0 = external_adc.computeVolts(r0);

    telnet.printf("ADC0 %d %fV\n", r0, v0);
    telnet.printf("ADC1 %d %fV\n", r1, v1);
    return true;
}

bool cmd_test_motor2(int8_t speed){
    char buf[256];
    
    for(int i=0; i < 20; i++){
        set_alt_motor_speed(120);
        delay(100);
        alt_motor_standby();
        sprintf(buf, "ALT %f\n", read_alt_encoder());
        telnet.print(buf);
    }
    telnet.print("\n");
    for(int i=0; i < 20; i++){
        set_alt_motor_speed(-120);
        delay(100);
        alt_motor_standby();
        sprintf(buf, "ALT %f\n", read_alt_encoder());
        telnet.print(buf);
    }
    telnet.print("\n");
    for(int i=0; i < 20; i++){
        set_azi_motor_speed(120);
        delay(100);
        azi_motor_standby();
        sprintf(buf, "AZI%f\n", read_azi_encoder());
        telnet.print(buf);
    }
    telnet.print("\n");
    for(int i=0; i < 20; i++){
        set_azi_motor_speed(-120);
        delay(100);
        azi_motor_standby();
        sprintf(buf, "AZI %f\n", read_azi_encoder());
        telnet.print(buf);
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

bool cmd_control_reset(void){
    solar_control_enabled = false;
    manual_control_enabled = false;
    azi_PID_enabled = false;
    alt_PID_enabled = false;
    
    return true;
}

bool cmd_manual_control(char *buf){
    float azi, alt;

    sscanf(buf, "%f%f", &alt, &azi);
    cmd_control_reset();
    alt_setpoint = alt;
    azi_setpoint = azi;

    float delta_alt, delta_azi;
    float c_alt, c_azi;
    float est_t_alt, est_t_azi, maxt;
    c_alt = read_alt_encoder();
    c_azi = read_azi_encoder();
    delta_alt = abs(c_alt-alt);
    if(delta_alt > 180.) delta_alt -= 360;
    delta_azi = abs(c_azi-azi);
    if(delta_azi > 180.) delta_azi -= 360;
    est_t_alt = delta_alt/alt_motor_max_sv;
    est_t_azi = delta_azi/azi_motor_max_sv;
    if(est_t_alt > est_t_azi)
        maxt = est_t_alt;
    else
        maxt = est_t_azi;
    if(maxt < 1.) maxt = 1.;
    set_pid_watchdog(WATCHDOG_TIME_FACTOR*maxt);

    /*
    SKETCH FOR SYNC MOVE
    float alts, azs, calt, cazi, exp_alt_t, exp_azi_t, tmov;
    calt = read_alt_encoder();
    cazi = read_azi_encoder();
    exp_alt_t = abs(calt - alt) / get_float_cfg("alt_Msv");
    exp_azi_t = abs(cazi - azi) / get_float_cfg("azi_Msv");
    telnet.printf("ALT A %f T %f\nAZI A %f T %f\n", abs(calt - alt), exp_alt_t, abs(cazi - azi), exp_azi_t);
    if(exp_azi_t > exp_alt_t) 
        tmov = exp_azi_t;
    else
        tmov = exp_alt_t;
    tmov *= 1.2;
    alts = get_float_cfg("alt_ms") + abs(calt - alt)/(tmov * get_float_cfg("alt_sm") );
    azis = get_float_cfg("azi_ms") + abs(cazi - azi)/(tmov * get_float_cfg("azi_sm") );
    if(azis > 127) azis = 127;
    if(alts > 127) alts = 127;
    altPID.set_max_speed(alts);
    aziPID.set_max_speed(azis);
    telnet.printf("Time to move %f, azi speed %f alt speed %f\n", tmov, azis, alts );*/
    manual_control_enabled = true;
    azi_PID_enabled = true;
    alt_PID_enabled = true;

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

bool cmd_driver_on(void){
    motor_driver_enable();
    return true;
}

bool cmd_driver_off(void){
    motor_driver_disable();
    return true;
}

bool cmd_alt_move(char *buf){
    int t;

    sscanf(buf, "%d", &t);
    if(t > 0){
        set_alt_motor_speed(120);
        delay(t);
        alt_motor_standby();
    }
    else {
        set_alt_motor_speed(-120);
        delay(-t);
        alt_motor_standby();
    }
    
    return true;
}

bool cmd_azi_move(char *buf){
    int t;

    sscanf(buf, "%d", &t);
    if(t > 0){
        set_azi_motor_speed(120);
        delay(t);
        azi_motor_standby();
    }
    else {
        set_azi_motor_speed(-120);
        delay(-t);
        azi_motor_standby();
    }
    
    return true;
}

bool cmd_add_schedule(char *buf){
    int h, m, s;

    sscanf(buf, "%d%d%d", &h, &m, &s);
    add_scheduled_task(h, m, s);
    return true;
}

bool cmd_ls(char *buf){
    String *fnames = list_dir(buf);
    if(fnames == NULL){
        telnet.printf("Cannot list %s\n", buf);
        return false;
    }
    else{
        int i = 0;
        while(fnames[i] != "<<END>>")
            telnet.println(fnames[i++]);
    }

    delete[] fnames;
    return true;
}

bool cmd_list_scenes(void){
    for(int i=0; i < scene_cnt; i++){
        telnet.printf("(%02d) %20s %03d f %.2f s\n", i, 
                      scenes_name[i], scene_len[i], 1.0 * scene_len[i] * SCENE_DT);
    }
    return true;
}

bool cmd_new_scene(char *rest){
    int ns = create_new_empty_scene(rest);
    if(ns < 0){
        telnet.printf("Error in creating scene.\n");
        return false;
    }
    telnet.printf("Created new scene %d\n", ns);
    return true;
}

bool cmd_remove_scene(char *rest){
    int ns;
    sscanf(rest, "%d", &ns);
    if(!remove_scene(ns)){
        telnet.printf("Error in removing scene.\n");
        return false;
    }
    telnet.printf("Removed scene %d\n", ns);
    return true;
}

bool cmd_scene_add_frame(char *rest){
    int ns;
    float alt, azi;
    sscanf(rest, "%d%f%f", &ns, &alt, &azi);
    return  scene_add_frame(ns, alt, azi);
}

bool cmd_print_scene(char *rest){
    int ns;
    sscanf(rest, "%d", &ns);
    if(ns < 0 || ns >= scene_cnt) return false;
    telnet.printf("%s\n", scenes_name[ns].c_str());
    for(int i=0; i < scene_len[ns]; i++){
        telnet.printf("(%03d - %06.2f s) %05.1f %05.1f\n", i, 1.0*i*SCENE_DT, scenes[ns][i*2], scenes[ns][i*2+1]);
    }
    return  true;
}

bool cmd_parse(char *buf){
    char *tok, *rest;
    const char *delim = " \n\r";
    rest = buf;
    tok = strtok_r(buf, delim, &rest);

    if(tok == NULL){
        return true;
    }
    else if(strcmp(tok, "sc") == 0 || strcmp(tok, "solar-control") == 0){
        return cmd_solar_control(rest);
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
        telnet.disconnectClient();
        return true;
    }
    else if(strcmp(tok, "current-position") == 0){
        return cmd_current_position();
    }
    else if(strcmp(tok, "test-motors") == 0){
        return cmd_test_motor2(128);
    }
    else if(strcmp(tok, "alt-move") == 0){
        return cmd_alt_move(rest);
    }
    else if(strcmp(tok, "azi-move") == 0){
        return cmd_azi_move(rest);
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
    else if(strcmp(tok, "mc") == 0){
        return cmd_manual_control(rest);
    }
    else if(strcmp(tok, "control-reset") == 0 || strcmp(tok, "stop") == 0){
        alt_motor_standby();
        azi_motor_standby();
        return cmd_control_reset();
    }
    else if(strcmp(tok, "driver-on") == 0){
        return cmd_driver_on();
    }
    else if(strcmp(tok, "driver-off") == 0){
        return cmd_driver_off();
    }
    else if(strcmp(tok, "test-scene") == 0){
        return run_scene();
    }
    else if(strcmp(tok, "sleep") == 0){
        return cmd_sleep(rest);
    }
    else if(strcmp(tok, "i2c-scan") == 0){
        return cmd_list_i2c_devices();
    }
    else if(strcmp(tok, "status") == 0){
        return cmd_system_status();
    }
    else if(strcmp(tok, "scheduled-task") == 0){
        return cmd_add_schedule(rest);
    }
    else if(strcmp(tok, "ls") == 0){
        return cmd_ls(rest);
    }
    else if(strcmp(tok, "list-scene") == 0){
        return cmd_list_scenes();
    }
    else if(strcmp(tok, "new-scene") == 0){
        return cmd_new_scene(rest);
    }
    else if(strcmp(tok, "remove-scene") == 0){
        return cmd_remove_scene(rest);
    }
    else if(strcmp(tok, "print-scene") == 0){
        return cmd_print_scene(rest);
    }
    else if(strcmp(tok, "add-frame-scene") == 0){
        return cmd_scene_add_frame(rest);
    }
    else if(strcmp(tok, "test-adc") == 0){
        return cmd_test_adc_encoder();
    }
    else if(strcmp(tok, "test-pid") == 0){
        return run_pid_test();
    }
    else if(strcmp(tok, "sync-rtc-ntp") == 0){
        return sync_RTC_from_NTP();
    }
    else if(strcmp(tok, "wifi-off") == 0){
        return wifi_off();
    }
    else if(strcmp(tok, "calibrate-speed") == 0){
        calibrate_speed();
        return true;
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
  
    telnet.println("\nWelcome to heliograph\nIP: " + telnet.getIP() + "\nID: " + buf);
    telnet.println("Use quit to disconnect\n");
    cmd_system_status();
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
        telnet.print("[OK] > ");
    }
    else{
        telnet.print("[!!] > ");
    }
}

void setup_telnet() {  
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

void setup_littlefs(void){
    if(!LittleFS.begin(true)){
      Serial.println("LittleFS Mount Failed");
      return;
   }
   else{
       Serial.println("Little FS Mounted Successfully");
   }
}

void setup_motors(){
    pinMode(DRIVER_ENABLE, OUTPUT);
    motor_driver_disable();

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

    azi_motor_min_s = get_float_cfg("azi_ms");
    azi_motor_max_sv = get_float_cfg("azi_Msv");
    azi_motor_min_sv = get_float_cfg("azi_msv");
    azi_motor_m_s = get_float_cfg("azi_sm");

    alt_motor_min_s = get_float_cfg("alt_ms");
    alt_motor_max_sv = get_float_cfg("alt_Msv");
    alt_motor_min_sv = get_float_cfg("alt_msv");
    alt_motor_m_s = get_float_cfg("alt_sm");

    azi_encoder_val = read_azi_encoder();
    alt_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    alt_setpoint = alt_encoder_val;
    aziPID.set_PID_params(get_float_cfg("azi_kp"),
                          get_float_cfg("azi_ki"),
                          get_float_cfg("azi_kd"),
                          get_float_cfg("azi_ms"),
                          127,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ki"),
                          get_float_cfg("alt_kd"),
                          get_float_cfg("alt_ms"),
                          127,
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
    if(rtc.begin()){
        RTC_ok = true;
    }
    else{
        RTC_ok = false;
    }

    if(RTC_ok){
        // TODO Copy from RTC to internal
        sync_internal_rtc();
    }
}

void setup_adc(void){
    if (external_adc.begin()){
        external_ADC_ok = true;
    }
    else{
        external_ADC_ok = false;
    }
}

void setup_adc_internal(void){
    pinMode(AZI_ENCODER, INPUT);
    pinMode(ALT_ENCODER, INPUT);
}

void setup_i2c(void){
    Wire.begin();
}

void setup() {
    // Global arrays initialization
    sun[_x_] = sun[_y_] = sun[_z_] = 0.;
    mir[_x_] = mir[_y_] = mir[_z_] = 0.;
    ory[_x_] = ory[_y_] = ory[_z_] = 0.;

    chip_id = (uint32_t) ESP.getEfuseMac();

    // Serial communication
    Serial.begin(9600);

    Serial.printf("\nHelioGraph %012x\n", chip_id);
    Serial.printf("Bootnumber %d\n", bootn++);

    setup_pref();
    current_lon = get_float_cfg("lon");
    current_lat = get_float_cfg("lat");
    setup_littlefs();
    load_scenes_from_file();

    setup_i2c();

    setup_wifi();
    if(WiFi_ok) setup_ntp();
    setup_adc();
    if(external_ADC_ok) setup_motors();
    motor_driver_enable();
    setup_rtc();
    if(WiFi_ok && NTP_ok && RTC_ok) sync_RTC_from_NTP();
    if(RTC_ok) sync_internal_rtc();
    if(!RTC_ok && NTP_ok) sync_internal_rtc_from_NTP();
    motor_driver_disable();

    if(WiFi_ok) setup_telnet();
    reset_pid_watchdog();

    /*Just for testing sleep mode*/
    /*delay(1000);
    motor_driver_enable();
    alt_setpoint = 0.;
    azi_setpoint = 0.;
    azi_PID_enabled = true;
    alt_PID_enabled = true;

    while(azi_PID_enabled || alt_PID_enabled) pid_loop();
    motor_driver_disable();*/
}

void loop() {
    if(WiFi_ok) telnet.loop();
    schedule_task_loop();

    check_external_ADC();
    if(!external_ADC_ok){
        motor_driver_disable();
        return;
    }

    
    if(manual_control_enabled || solar_control_enabled){
        if(solar_control_enabled){
            ray_to_setpoints(ory_alt, ory_azi);
        }
        pid_loop();
        if(!azi_PID_enabled && !alt_PID_enabled){
            manual_control_enabled = false;
            solar_control_enabled = false;
        }
    }
}