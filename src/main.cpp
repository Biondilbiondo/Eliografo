#include "main.h"
//#include "driver/periph_ctrl.h"
//#include "driver/rtc_io.h"
#include "configs.h"
#include "pins.h"


char wifi_ssids[MAX_WIFI_NETWORKS][128];
char wifi_passwords[MAX_WIFI_NETWORKS][128];
uint8_t wifi_net_cnt = 0;
bool WiFi_ok = false;
uint32_t wifi_watchdog_0;
uint32_t telnet_watchdog_0;

//WiFiServer ComServer(23);
//WiFiClient Controller;
ESPTelnet telnet;

uint32_t chip_id;
int32_t bootn;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
bool NTP_ok = false;

ESP32Time internal_rtc;
RTC_DS1307 rtc;
bool RTC_ok = false, internal_RTC_ok = false;

Adafruit_ADS1115 external_adc;
bool external_ADC_ok = false;

// Non volatile memory
Preferences HGPrefs; 

// Daily Tasks
uint64_t sch_timestamp[MAX_DAILY_TASKS];
bool sch_run[MAX_DAILY_TASKS] = {false};
uint8_t sch_type[MAX_DAILY_TASKS] = {NO_TASK};
uint32_t task_cnt = 0;
uint8_t sch_sequence[MAX_DAILY_TASKS][MAX_SCENES_IN_SEQUENCE] = {0};
uint8_t sch_sequence_len[MAX_DAILY_TASKS] = {0};

// Scene variables
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
uint32_t pid_watchdog_t0, pid_watchdog_elap;
float azi_motor_max_angular_speed;
float alt_motor_max_angular_speed;
float alt_encoder_volt_to_deg, azi_encoder_volt_to_deg;
uint32_t encoder_oversampling = ENCODER_OVERSAMPLING;

float sleep_alt, sleep_azi;

float angular_speed_to_pwm_alt, angular_speed_to_pwm_azi;
int32_t pwm_min_alt, pwm_min_azi;

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

bool littleFS_ok = false;

uint32_t log_level = 0;
int32_t log_delete_after_days = 0;

void system_checks(void){
    if(alt_encoder_zero <= 100 || alt_encoder_zero >= 260){
        sys_log(LOG_WARNING, "Zero of altitude encoder is in the range ov movement and this can cause inaccuracy.");
    }
}
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

float get_timestamp(void){
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    float seconds;
    get_time(&year, &month, &day, &hours, &minutes, &seconds);
    float timestamp = (float) hours * 3600. + (float) minutes * 60. + seconds;
    return timestamp;
}

float get_timestamp_RTC(void){
    return (float) internal_rtc.getHour(true) * 3600. + (float) internal_rtc.getMinute() * 60 + internal_rtc.getSecond() + internal_rtc.getMicros() / 1e6;
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

uint32_t days_since_epoch(int year, int month, int day){
    uint16_t a = (14 - month) / 12;
    uint16_t y = year + 4800 - a;
    uint16_t m = month + 12 * a - 3;
    uint32_t days = day + ((153ul * m + 2) / 5) + (365ul * y) + (y / 4ul) - (y / 100ul) + y / 400ul - 32045ul - 1721426ul;
    return days;
}

bool check_time(void){
    uint16_t y;
    uint8_t m, d, h, min;
    float s;
    get_time(&y, &m, &d, &h, &min, &s);

    if(y < SAFETY_MIN_YEAR) return false;
    if(y > SAFETY_MAX_YEAR) return false;
    return true;
}


void sys_log(uint8_t type, const char *format, ...)
{
    if(log_level < type) return;

    char loc_buf[64];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return;
    }
    if(len >= (int)sizeof(loc_buf)){  // comparation of same sign type for the compiler
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return;
        }
        len = vsnprintf(temp, len+1, format, arg);
    }
    va_end(arg);

    char header[64];
    char dc;
    uint8_t m, d, h, min;
    uint16_t y;
    float s;
    get_time(&y, &m, &d, &h, &min, &s);
    switch(type){
        case LOG_DEBUG:
            dc = 'D';
            break;
        case LOG_INFO:
            dc = 'I';
            break;
        case LOG_WARNING:
            dc = 'W';
            break;
        case LOG_ERROR:
            dc = 'E';
            break;
        default:
            dc = '?';
    }

    sprintf(header, "[%04d-%02d-%02d %02d:%02d:%04.1f BN %05d][%c] ", y, m, d, h, min, s, bootn, dc);

#ifdef SERIAL_LOG_ENABLED
        Serial.printf("%s %s\n", header, temp);
#endif
#ifdef LITTLEFS_LOG_ENABLED
    char path[64];
    sprintf(path, "/log/%04d_%02d_%02d.log", y, m, d);
    if(littleFS_ok){
        File file;
        if(LittleFS.exists(path)) 
            file = LittleFS.open(path, FILE_APPEND);
        else
            file = LittleFS.open(path, FILE_WRITE);
        if(!file) return;
        file.printf("%s %s\n", header, temp);
        file.close();
    }
#endif
    
    if(temp != loc_buf){
        free(temp);
    }
    return;
}

// littleFS routine
String *list_dir(const char *dirname){
    File root = LittleFS.open(dirname);
    if(dirname == NULL) 
        return NULL;
    if(!root){
        sys_log(LOG_ERROR, "Failed to open directory %s", dirname);
        return NULL;
    }
    if(!root.isDirectory()){
        sys_log(LOG_ERROR, "%s is not a directory, cannot list", dirname);
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

bool remove_file(String name){
    sys_log(LOG_INFO, "Deleting file: %s", name.c_str());
    if(LittleFS.remove(name.c_str())){
        return true;
    } else {
        sys_log(LOG_ERROR, "Delete failed");
        return false;
    }
}

void cleanup_syslog(void){
    String *fnames = list_dir("/log/");
    String s;
    uint16_t y;
    uint8_t m, d, h, min;
    float sec;
    
    uint32_t today, log_date;
    if(log_delete_after_days < 0) return;
    get_time(&y, &m, &d, &h, &min, &sec);
    today = days_since_epoch((int) y, (int) m, (int) d);
    for(int i=0; i < MAX_LOG_FILES; i++){
        if(fnames[i] == "<<END>>") break;
        int ylog, mlog, dlog;
        sscanf(fnames[i].c_str(), "%04d_%02d_%02d.log", &ylog, &mlog, &dlog);
        sys_log(LOG_DEBUG, "Filename %s: year %d, month %d, day %d", fnames[i].c_str(), y, m, d);
        log_date = days_since_epoch((int) ylog, (int) mlog, (int) dlog);
        sys_log(LOG_DEBUG, "Today is %d d.a.e. logfile %s is %d d.a.e", today, fnames[i].c_str(), log_date);
        if(today - log_date > log_delete_after_days){
            remove_file("/log/"+fnames[i]);
        }
        
    }
}

// WiFi credentials
bool load_wifi_credentials_from_file(const char *path){
    String s;
    char cs[1024];
    char *tok;
    sys_log(LOG_INFO, "Reading wifi credentials from file: %s", path);

    File file = LittleFS.open(path);
    if(!file || file.isDirectory()){
        sys_log(LOG_ERROR, "Failed to open file for reading");
        return false;
    }

    while(file.available() && wifi_net_cnt < MAX_WIFI_NETWORKS){
        s = file.readStringUntil('\n');
        strcpy(cs, s.c_str());
        tok = strtok(cs, " ");
        strcpy(wifi_ssids[wifi_net_cnt], tok);
        tok = strtok(NULL, " ");
        strcpy(wifi_passwords[wifi_net_cnt], tok);
        wifi_net_cnt++;
    }
    file.close();
    return true;
}

bool save_wifi_credentials_on_file(const char *path){
    String s;
    char cs[1024];
    char *tok;

    File file = LittleFS.open(path, FILE_WRITE);
    if(!file || file.isDirectory()){
        sys_log(LOG_ERROR, "Failed to open %s for writing", path);
        return false;
    }

    for(int i=0; i < wifi_net_cnt; i++)
        file.printf("%s %s\n", wifi_ssids[i], wifi_passwords[i]);
    
    file.close();
    return true;
}

bool delete_wifi_credentials(int c){
    if(c >= wifi_net_cnt || c < 0) return false;
    for(int i=c+1; i<wifi_net_cnt; i++){
        strcpy(wifi_ssids[i-1], wifi_ssids[i]);
        strcpy(wifi_passwords[i-1], wifi_passwords[i]);
    }
    wifi_net_cnt--;
    return true;
}

// Scheduled tasks
bool add_scheduled_task_wifi(uint32_t h, uint32_t m, uint32_t s){

    if(task_cnt < MAX_DAILY_TASKS){
        sch_timestamp[task_cnt] = h * 3600 + m * 60 + s;
        sch_type[task_cnt] = WIFI_TASK;
        task_cnt++;
        return true;
    }
    return false;
}

bool add_scheduled_task_sequence(uint32_t h, uint32_t m, uint32_t s, uint8_t *seq, uint8_t seq_len){
    if(task_cnt < MAX_DAILY_TASKS){
        sch_timestamp[task_cnt] = h * 3600 + m * 60 + s - SAFETY_TIME_BEFORE_SEQUENCE;
        sch_type[task_cnt] = SEQUENCE_TASK;
        for(int i=0; i < seq_len && i < MAX_SCENES_IN_SEQUENCE; i++){
            sch_sequence[task_cnt][i] = seq[i];
        }
        sch_sequence_len[task_cnt] = seq_len;
        task_cnt++;
        return true;
    }
    return false;
}

bool delete_schedule(uint16_t s){
    if(s >= task_cnt) return false;
    for(int i=s+1; i < task_cnt; i++){
        sch_timestamp[i-1] = sch_timestamp[i];
        sch_type[i-1] = sch_type[i];
        for(int j=0; j < MAX_SCENES_IN_SEQUENCE; j++){
            sch_sequence[i-1][j] = sch_sequence[i][j];
        }
        sch_sequence_len[i-1] = sch_sequence_len[i];
    }
    task_cnt--;
    return true;

}

bool load_schedule_from_file(const char *path){
    String s;
    char cs[1024];
    char *tok;
    sys_log(LOG_INFO, "Reading schedule file: %s", path);

    File file = LittleFS.open(path);
    if(!file || file.isDirectory()){
        sys_log(LOG_ERROR, "Failed to open file for reading");
        return false;
    }

    int i = 0;
    uint32_t h, m, sec;
    uint8_t type;

    while(file.available()){
        s = file.readStringUntil('\n');
        strcpy(cs, s.c_str());
        tok = strtok(cs, " ");
        sscanf(tok, "%d", &h);
        tok = strtok(NULL, " ");
        sscanf(tok, "%d", &m);
        tok = strtok(NULL, " ");
        sscanf(tok, "%d", &sec);

        tok = strtok(NULL, " ");
        sscanf(tok, "%d", &m);
        if(strcmp(tok, "wifi") == 0){
            type = WIFI_TASK;
            add_scheduled_task_wifi(h, m, sec);
            sys_log(LOG_DEBUG, "Added wifi task at %02d:%02d:%02d", h, m, sec);
        }
        else if(strcmp(tok, "sequence") == 0){
            type = SEQUENCE_TASK;
            uint8_t myseq[MAX_SCENES_IN_SEQUENCE] = {0};
            uint8_t seq_len = 0;
            char *next_scene_name;
            bool error = false;

            next_scene_name = strtok(NULL, " ");
            while(next_scene_name != NULL){
                String nsn = String(next_scene_name);
                int j;
                for(j=0; j < scene_cnt; j++){
                    if(nsn == scenes_name[j]){
                        myseq[seq_len++] = j;
                        break;
                    }
                }
                if(j == scene_cnt){
                    error = true;
                    sys_log(LOG_ERROR, "Error while parsing line \"%s\" in file %s.", s.c_str(), path);
                    sys_log(LOG_ERROR, "Scene \"%s\" not found.", nsn);
                }
                next_scene_name = strtok(NULL, " ");
            }
            if(!error){
                add_scheduled_task_sequence(h, m, sec, myseq, seq_len);
                sys_log(LOG_DEBUG, "Added sequence task at %02d:%02d:%02d (it will be scheduled %d'%d\" before)", 
                        h, m, sec, (int) SAFETY_TIME_BEFORE_SEQUENCE/60, (int) SAFETY_TIME_BEFORE_SEQUENCE%60 );
            }
        }
        else{
            sys_log(LOG_ERROR, "Error while parsing line \"%s\" in file %s.", s.c_str(), path);
        }
        i++;
    }
    file.close();
    return true;
}

void schedule_task_loop(void){
    float timestamp = get_timestamp();
    bool was_on_wifi = false;

    for(int i=0; i<task_cnt; i++){
        if(sch_run[i]){ 
            // Reload
            if(abs(timestamp - (float) sch_timestamp[i]) > 2 * SCHEDULE_TIME_DELTA) sch_run[i] = false;
            // Skip
            continue;
        }

        if(abs(timestamp - (float) sch_timestamp[i]) < SCHEDULE_TIME_DELTA){
            /*telnet.print("Running daily task\n");
            telnet.printf("%02d at %02d:%02d:%02d\n", i, hours, minutes, (int) seconds);*/
            if(sch_type[i] == WIFI_TASK && !WiFi_ok){
                sys_log(LOG_INFO, "WiFi turned on by task number %d", i);
                sys_log(LOG_DEBUG, "Timestamp %f schedule timestamp %f", timestamp, (float) sch_timestamp[i]);
                wifi_on();
            }
            if(sch_type[i] == SEQUENCE_TASK){
                sys_log(LOG_INFO, "Starting sequence task %d right now", i);
                sys_log(LOG_DEBUG, "Timestamp %f schedule timestamp %f", timestamp, (float) sch_timestamp[i]);
                if(WiFi_ok){
                    if(telnet.isConnected()){
                        telnet.printf("Sorry but a sequence is starting right now, you can reconnect at the end.\n");
                        telnet.disconnectClient();
                    }
                    wifi_off();
                    was_on_wifi = true;
                }
                run_sequence(sch_sequence[i], sch_sequence_len[i], (float) sch_timestamp[i] + SAFETY_TIME_BEFORE_SEQUENCE);
                if(was_on_wifi){
                    wifi_on();
                    wifi_watchdog_0 = millis();
                }
            }
            sch_run[i] = true;
        }   
    }
}

float seconds_to_next_schedule(void){
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    float seconds;

    get_time(&year, &month, &day, &hours, &minutes, &seconds);
    float timestamp = hours * 3600 + minutes * 60 + seconds;
    float next_schedule = -1, dt;

    for(int i=0; i<task_cnt; i++){
        dt = sch_timestamp[i] - timestamp;
        if(dt < 0) dt = sch_timestamp[i] + (SECONDS_PER_DAY - timestamp);
        if(next_schedule < 0 || dt < next_schedule) next_schedule = dt;
    }
    return next_schedule;
}

// Preferences Routines
bool cfg_key_exists(const char *key){
    return HGPrefs.isKey(key);
}

float get_float_default_cfg(const char *k){
    if(strcmp(k, "azi_kp") == 0)
        return DEFAULT_AZI_KP;
    if(strcmp(k, "alt_kp") == 0)
        return DEFAULT_ALT_KP;
    
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
    if(strcmp(k, "azi_Msv") == 0)
        return DEFAULT_MAX_SPEED_VALUE;
    if(strcmp(k, "alt_Msv") == 0)
        return DEFAULT_MAX_SPEED_VALUE;
    if(strcmp(k, "alt_s2p") == 0)
        return DEFAULT_SPEED_TO_PWM_ALT;
    if(strcmp(k, "azi_s2p") == 0)
        return DEFAULT_SPEED_TO_PWM_AZI;
    if(strcmp(k, "altv2d") == 0)
        return DEFAULT_VOLT_TO_DEG;
    if(strcmp(k, "aziv2d") == 0)
        return DEFAULT_VOLT_TO_DEG;
    if(strcmp(k, "azinap") == 0)
        return DEFAULT_SLEEP_AZI;
    if(strcmp(k, "altnap") == 0)
        return DEFAULT_SLEEP_ALT;

    return 0.0;
}

int32_t get_int_default_cfg(const char *k){
    // Actually integers...
    if(strcmp(k, "overs") == 0)
        return ENCODER_OVERSAMPLING;
    if(strcmp(k, "bootn") == 0)
        return 0L;
    if(strcmp(k, "lastnet") == 0)
        return 0L;
    if(strcmp(k, "alt_mPWM") == 0)
        return DEFAULT_PWM_MIN_ALT;
    if(strcmp(k, "azi_mPWM") == 0)
        return DEFAULT_PWM_MIN_AZI;
    if(strcmp(k, "logl") == 0)
        return DEFAULT_LOG_LEVEL;
    if(strcmp(k, "logr") == 0)
        return DEFAULT_LOG_DELETE_AFTER_DAYS;

    return 0L;
}

bool cfg_is_int(const char *k){
    // Actually integers...
    if(strcmp(k, "overs") == 0)
        return true;
    if(strcmp(k, "bootn") == 0)
        return true;
    if(strcmp(k, "lastnet") == 0)
        return true;
    if(strcmp(k, "alt_mPWM") == 0)
        return true;
    if(strcmp(k, "azi_mPWM") == 0)
        return true;
    if(strcmp(k, "logl") == 0)
        return true;
    if(strcmp(k, "logr") == 0)
        return true;

    return false;
}

float get_float_cfg(const char *k){
    if(!cfg_key_exists(k))
        HGPrefs.putFloat(k, get_float_default_cfg(k));
    return HGPrefs.getFloat(k);
}

bool set_float_cfg(const char *key, float val){
    if(cfg_key_exists(key)){
        HGPrefs.putFloat(key, val);
        return true;
    }
    else{
        sys_log(LOG_ERROR, "Requested key to set %s not found.", key);
        return false;
    }
}

int32_t get_int_cfg(const char *k){
    if(!cfg_key_exists(k))
        HGPrefs.putLong(k, get_float_default_cfg(k));
    return HGPrefs.getLong(k);
}

bool set_int_cfg(const char *key, int32_t val){
    if(cfg_key_exists(key)){
        HGPrefs.putLong(key, val);
        return true;
    }
    else{
        sys_log(LOG_ERROR, "Requested key to set %s not found.", key);
        return false;
    }
}

void configurations_log(void){
    telnet.printf("%-35s %d\n", "ENCODER_OVERSAMPLING", get_int_cfg("overs"));
    telnet.printf("%-35s %d\n", "LAST_NETWORK", get_int_cfg("lastnet"));
    telnet.printf("%-35s %d\n", "PWM_MIN_ALT", get_int_cfg("alt_mPWM"));
    telnet.printf("%-35s %d\n", "PWM_MIN_AZI", get_int_cfg("azi_mPWM"));
    telnet.printf("%-35s %d\n", "LOG_LEVEL", get_int_cfg("logl"));
    telnet.printf("%-35s %d\n", "LOG_DELETE_AFTER_DAYS", get_int_cfg("logr"));
    telnet.printf("%-35s %d\n", "MAX_BLIND_MOVE_TIME_MS", MAX_BLIND_MOVE_TIME_MS);
    telnet.printf("%-35s %d\n", "MAX_DAILY_TASKS", MAX_DAILY_TASKS);
    telnet.printf("%-35s %d\n", "MAX_LOG_FILES", MAX_LOG_FILES);
    telnet.printf("%-35s %d\n", "MAX_SCENES", MAX_SCENES);
    telnet.printf("%-35s %d\n", "MAX_SCENES_IN_SEQUENCE", MAX_SCENES_IN_SEQUENCE);
    telnet.printf("%-35s %d\n", "MAX_SLEEP_S", MAX_SLEEP_S);
    telnet.printf("%-35s %d\n", "MAX_WIFI_NETWORKS", MAX_WIFI_NETWORKS);
    telnet.printf("%-35s %d\n", "N_WIFI_ATTEMPTS", N_WIFI_ATTEMPTS);
    telnet.printf("%-35s %d\n", "SAFETY_MAX_YEAR", SAFETY_MAX_YEAR);
    telnet.printf("%-35s %d\n", "SAFETY_MIN_YEAR", SAFETY_MIN_YEAR);
    telnet.printf("%-35s %d\n", "SAFETY_TIME_BEFORE_FIRST_SCENE", SAFETY_TIME_BEFORE_FIRST_SCENE);
    telnet.printf("%-35s %d\n", "SAFETY_TIME_BEFORE_SEQUENCE", SAFETY_TIME_BEFORE_SEQUENCE);
    telnet.printf("%-35s %d\n", "SCENE_LEN", SCENE_LEN);
    telnet.printf("%-35s %d\n", "SCENE_LEN_SECONDS", SCENE_LEN_SECONDS);
    telnet.printf("%-35s %d\n", "SCHEDULE_TIME_DELTA", SCHEDULE_TIME_DELTA);
    telnet.printf("%-35s %d\n", "SLEEP_TIME_S", SLEEP_TIME_S);
    telnet.printf("%-35s %d\n", "TELNET_WATCHDOG_TIME", TELNET_WATCHDOG_TIME);
    telnet.printf("%-35s %d\n", "WAKEUP_TIME_BEFORE_SCHEDULE_S", WAKEUP_TIME_BEFORE_SCHEDULE_S);
    telnet.printf("%-35s %d\n", "WIFI_WATCHDOG_TIME", WIFI_WATCHDOG_TIME);
    telnet.printf("%-35s %d\n", "PWM_MAX_VALUE", PWM_MAX_VALUE);

    telnet.printf("%-35s %f\n", "ATM_PRESSURE", ATM_PRESSURE);
    telnet.printf("%-35s %f\n", "ATM_TEMPERATURE", ATM_TEMPERATURE);
    telnet.printf("%-35s %f\n", "BATTERY_MAX_mV", BATTERY_MAX_mV);
    telnet.printf("%-35s %f\n", "BATTERY_MIN_mV", BATTERY_MIN_mV);
    telnet.printf("%-35s %f\n", "BATTERY_R1_kOHM", BATTERY_R1_kOHM);
    telnet.printf("%-35s %f\n", "BATTERY_R2_kOHM", BATTERY_R2_kOHM);
    telnet.printf("%-35s %f\n", "BATTERY_VOLTAGE_DIVIDER_FAC", BATTERY_VOLTAGE_DIVIDER_FAC);
    telnet.printf("%-35s %f\n", "MIN_ALLOWED_SPEED", MIN_ALLOWED_SPEED);
    telnet.printf("%-35s %f\n", "WATCHDOG_TIME_FACTOR", WATCHDOG_TIME_FACTOR);
    
    telnet.printf("%-35s %f\n", "ALT_ENCODER_ZERO", get_float_cfg("alte0"));
    telnet.printf("%-35s %f\n", "ALT_KP", get_float_cfg("alt_kp"));
    telnet.printf("%-35s %f\n", "ALT_MIN_E", get_float_cfg("alt_me"));
    telnet.printf("%-35s %f\n", "AZI_ENCODER_ZERO", get_float_cfg("azie0"));
    telnet.printf("%-35s %f\n", "AZI_KP", get_float_cfg("azi_kp"));
    telnet.printf("%-35s %f\n", "AZI_MIN_E", get_float_cfg("azi_me"));
    telnet.printf("%-35s %f\n", "GEO_LAT", get_float_cfg("lat"));
    telnet.printf("%-35s %f\n", "GEO_LON", get_float_cfg("lon"));
    telnet.printf("%-35s %f\n", "AZI_MAX_SPEED_VALUE", get_float_cfg("azi_Msv"));
    telnet.printf("%-35s %f\n", "AZI_VOLT_TO_DEG", get_float_cfg("aziv2d"));
    telnet.printf("%-35s %f\n", "ALT_MAX_SPEED_VALUE", get_float_cfg("alt_Msv"));
    telnet.printf("%-35s %f\n", "ALT_VOLT_TO_DEG", get_float_cfg("aziv2d"));
    telnet.printf("%-35s %f\n", "SPEED_TO_PWM_ALT", get_float_cfg("alt_s2p"));
    telnet.printf("%-35s %f\n", "SPEED_TO_PWM_AZI", get_float_cfg("azi_s2p"));
    telnet.printf("%-35s %f\n", "SLEEP_ALT", get_float_cfg("altnap"));
    telnet.printf("%-35s %f\n", "SLEEP_AZI", get_float_cfg("azinap"));
    return;
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

bool is_sun_above_horizon(void){
    update_sun_vec();
    float alt = absolute_to_geo_alt(sun);
    if(alt > 180)
        alt -= 360;
    if(alt < -180)
        alt += 360;

    return alt > 0;
}

// Motors and Encoders Routine
static inline uint8_t _sigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	// slow
	// uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

	// steep
	// uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

	// normal
    telnet.printf("%d %d %d\n", voltage, minVoltage, maxVoltage);
	uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
    telnet.printf("%d\n", result);
	return result >= 100 ? 100 : result;
}

float read_battery_charge(void){
    float f_v = 0.0;
    if(!external_ADC_ok) return -1.0;
    for(int i=0; i < encoder_oversampling; i++){
        f_v += external_adc.computeVolts(external_adc.readADC_SingleEnded(2));
        if(i != encoder_oversampling-1) delay(1);
    }

    f_v /= encoder_oversampling;
    telnet.printf("Raw V: %f Bat V: %f\n", f_v,  f_v * BATTERY_VOLTAGE_DIVIDER_FAC);
    f_v *= BATTERY_VOLTAGE_DIVIDER_FAC;
    return (float) _sigmoidal(int16_t (f_v*1000.), BATTERY_MIN_mV, BATTERY_MAX_mV);
}

void motor_driver_enable(void){
    if(!driver_on){
        driver_on = true;
        digitalWrite(DRIVER_ENABLE, HIGH);
    }
}

void motor_driver_disable(void){
    if(driver_on){
        driver_on = false;
        digitalWrite(DRIVER_ENABLE, LOW);
    }
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

void check_external_ADC(void){
    byte error, address;
    Wire.beginTransmission(ADS1X15_ADDRESS);
    error = Wire.endTransmission();
    if(error == 0) {
        external_ADC_ok = true;
    }
    else{
        external_ADC_ok = false;
    }
}

float azi_encoder_to_degrees(float val){
    return val * azi_encoder_volt_to_deg;
}

float read_azi_encoder(void){
    float f_v = 0.0, val, del, first;
    if(!external_ADC_ok) return 0.0;
    for(int i=0; i < ENCODER_OVERSAMPLING; i++){
        val = azi_encoder_to_degrees(external_adc.computeVolts(external_adc.readADC_SingleEnded(0)));
        if(i==0){
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
        delay(1);
    }

    f_v /= ENCODER_OVERSAMPLING;
    float deg = f_v - azi_encoder_zero;
    if(deg < 0) deg+=360;
    if(deg > 360) deg -= 360;
    return deg; // Just to put something here
}

float alt_encoder_to_degrees(float val){
    return val * alt_encoder_volt_to_deg;
}

float read_alt_encoder(void){
    float f_v = 0.0, val, del, first;
    if(!external_ADC_ok) return 0.0;
    for(int i=0; i < encoder_oversampling; i++){
        val = alt_encoder_to_degrees(external_adc.computeVolts(external_adc.readADC_SingleEnded(1)));
        if(i==0){
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
        if(i != encoder_oversampling-1) delay(1);
    }

    f_v /= encoder_oversampling;
    float deg = f_v - alt_encoder_zero;
    if(deg < 0) deg+=360;
    if(deg > 360) deg -= 360;
    return deg; // Just to put something here
}

void set_pid_watchdog(uint32_t watchdog_time_s){
    pid_watchdog_t0 = millis();
    pid_watchdog_elap = watchdog_time_s*1000;
}

void reset_pid_watchdog(void){
    pid_watchdog_t0 = -1;
    pid_watchdog_elap = -1;
}

void pid_control_reset(void){
    if(!external_ADC_ok) return;

    solar_control_enabled = false;
    manual_control_enabled = false;
    azi_PID_enabled = false;
    alt_PID_enabled = false;

    azi_encoder_val = read_azi_encoder();
    alt_encoder_val = read_alt_encoder();
    azi_setpoint = azi_encoder_val;
    alt_setpoint = alt_encoder_val;

    reset_pid_watchdog();
}

void start_pid(void){
    alt_PID_enabled = true;
    azi_PID_enabled = true;
}

int8_t compute_speed(float dangle, float dt, float min, float max, float gain, float pwm_min){
    float speed;

    if(dangle > 0)
        speed = (pwm_min + 1e3 * dangle / dt * gain);
    else
        speed = (-pwm_min + 1e3 * dangle / dt * gain);

    if(speed > max) speed = max;
    if(speed < min) speed = min;
    return (int8_t) speed;
}

float compute_angle_delta(float a1, float a2){
    float delta;
    delta = a1 - a2;
    if(delta > 180.) delta -= 360;
    else if(delta < -180) delta += 360;
    //if(abs(delta) < .5) delta = 0.;
    return delta;
}

void set_pid_goto(float alt, float azi){
    float est_t_alt, est_t_azi, maxt;
    float delta_alt, delta_azi;

    reset_pid_watchdog();

    pid_control_reset();
    alt_setpoint = alt;
    azi_setpoint = azi;

    if(!external_ADC_ok) return;
    alt_encoder_val = read_alt_encoder();
    azi_encoder_val = read_azi_encoder();
    delta_alt = abs(compute_angle_delta(alt_encoder_val, alt_setpoint));
    delta_azi = abs(compute_angle_delta(azi_encoder_val, azi_setpoint));
    
    est_t_alt = delta_alt/alt_motor_max_angular_speed;
    est_t_azi = delta_azi/azi_motor_max_angular_speed;
    if(est_t_alt > est_t_azi)
        maxt = est_t_alt;
    else
        maxt = est_t_azi;
    
    if(maxt < 1.) maxt = 1.;
    set_pid_watchdog(WATCHDOG_TIME_FACTOR*maxt);

    start_pid();
}

void pid_loop(void){
    if(pid_watchdog_t0 > 0 && pid_watchdog_elap > 0){
        uint32_t dt = millis() - pid_watchdog_t0;
        if(dt > pid_watchdog_elap || dt < 0){
            azi_motor_standby();
            alt_motor_standby();
            azi_PID_enabled = false;
            alt_PID_enabled = false;
            sys_log(LOG_ERROR, "PID loop killed by watchdog.");
            return;
        }
    }
    check_external_ADC();
    if(!external_ADC_ok){
        azi_motor_standby();
        alt_motor_standby();
        azi_PID_enabled = false;
        alt_PID_enabled = false;
        sys_log(LOG_ERROR, "PID loop killed because ADC is not working.");
        return;
    }
    if(alt_PID_enabled){
        alt_encoder_val = read_alt_encoder();
        //telnet.printf("ALT ENC %f SETP %f\n", alt_encoder_val, alt_setpoint);
        altPID.update();
        //telnet.printf("ALT motor speed %d\n", (int8_t) alt_motor_speed);
        set_alt_motor_speed((int8_t) alt_motor_speed);
    }
    if(azi_PID_enabled){
        azi_encoder_val = read_azi_encoder();
        //telnet.printf("AZI ENC %f SETP %f\n", azi_encoder_val, azi_setpoint);
        aziPID.update();
        //telnet.printf("AZI motor speed %d\n", (int8_t) azi_motor_speed);
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
}

void ray_to_setpoints(float alt, float azi){
    geo_to_absolute(alt, azi, ory);
    update_sun_vec();
    get_normal_vec(sun, ory, mir);
    alt_setpoint = absolute_to_geo_alt(mir);
    azi_setpoint = absolute_to_geo_azi(mir);

    if(alt_setpoint < 0.) alt_setpoint += 360;
    if(alt_setpoint > 360.0) alt_setpoint -= 360.;
    if(alt_setpoint > 90 && alt_setpoint < 270.){
        sys_log(LOG_INFO, "Applying coordinate system correction");
        alt_setpoint = 180. - alt_setpoint;
        azi_setpoint += 180;
        if(alt_setpoint < 0.) alt_setpoint += 360;
        if(alt_setpoint > 360.0) alt_setpoint -= 360.;  //Should never happen
    }
    if(azi_setpoint < 0.) azi_setpoint += 360.0;
    if(azi_setpoint > 360.) azi_setpoint -= 360.0;
}

bool write_scene_on_file(int s){
    sys_log(LOG_INFO, "Writing file: %s", "/scenes/"+scenes_name[s]);

    File file = LittleFS.open("/scenes/"+scenes_name[s], FILE_WRITE);
    if(!file){
        sys_log(LOG_INFO, "Error in writing file: %s", "/scenes/"+scenes_name[s]);
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
    sys_log(LOG_INFO, "Reading scene file: %s", path);

    File file = LittleFS.open(path);
    if(!file || file.isDirectory()){
        sys_log(LOG_ERROR, "Failed to open file for reading");
        return -1;
    }

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

bool run_scene(uint8_t sn, float run_timestamp=-1., float alt_0=0., float azi_0=0.){
    //TODO watchdog 
    //TODO syncmove
    uint32_t now, next, t0, scene_beg;
    int32_t dt;

    int8_t alt_target_speed, alt_max_speed, alt_min_speed;
    int8_t azi_target_speed, azi_max_speed, azi_min_speed;

    float speed_range = 0.2;
    
    float delta_alt, delta_azi;
    int8_t alt_speed_i, azi_speed_i;

    float current_timestamp;
    bool wait = true;

// ENABLE JUST FOR DEBUG POURPOSE!
#define DEBUG_VERBOSE_SCENE
//#define DEBUG_USE_ABSOLUTE_INSTEAD_OF_SOLAR

    current_timestamp = get_timestamp_RTC();
#ifdef DEBUG_VERBOSE_SCENE
    sys_log(LOG_DEBUG, "Running scene %d %s", sn, scenes_name[sn]);
    sys_log(LOG_DEBUG, "Current timestamp %.1f, run timestamp %.1f", current_timestamp, run_timestamp);
#endif
    if(run_timestamp < 0 || current_timestamp > run_timestamp){
        // Run immediately
        wait = false;
#ifdef DEBUG_VERBOSE_SCENE
        sys_log(LOG_DEBUG, "I don't have to wait!");
#endif
    }

    motor_driver_enable();
#ifdef DEBUG_USE_ABSOLUTE_INSTEAD_OF_SOLAR
    alt_setpoint = scenes[sn][0]+alt_0;
    azi_setpoint = scenes[sn][1]+azi_0;
#else
    ray_to_setpoints(scenes[sn][0]+alt_0, scenes[sn][1]+azi_0);
#endif

    start_pid();
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
#ifdef DEBUG_VERBOSE_SCENE
    telnet.print("Initial position\n");
#endif
    if(wait){
        do{
            current_timestamp = get_timestamp_RTC();
#ifdef DEBUG_VERBOSE_SCENE
            sys_log(LOG_DEBUG, "Current timestamp %.1f, run timestamp %.1f", current_timestamp, run_timestamp);
#endif
            delay(10);
        }while(run_timestamp > current_timestamp);
    }
    sys_log(LOG_INFO, "Scene started at %.1f (required %.1f)", current_timestamp, run_timestamp);

    now = millis();
    scene_beg = now;
    
    int i = 0;
    next = now;

    while(i < scene_len[sn]){
        t0 = millis();
        alt_encoder_val = read_alt_encoder();
        azi_encoder_val = read_azi_encoder();
        
        delta_alt = compute_angle_delta(alt_setpoint, alt_encoder_val);
        delta_azi = compute_angle_delta(azi_setpoint, azi_encoder_val);

        now = millis();
        dt = next-now;
        if(dt > 100){
            
            alt_speed_i = compute_speed(delta_alt, dt, alt_min_speed, alt_max_speed, 
                                        angular_speed_to_pwm_alt, (float) pwm_min_alt);
            azi_speed_i = compute_speed(delta_azi, dt, azi_min_speed, 
                                        azi_max_speed, angular_speed_to_pwm_azi, (float) pwm_min_azi);
#ifdef DEBUG_VERBOSE_SCENE
            telnet.printf("dt = %d\n", dt);
            telnet.printf("alt %f azi %f \n", delta_alt, delta_azi);
            telnet.printf("speeds %d %d\n",alt_speed_i, azi_speed_i);
#endif

            set_alt_motor_speed(alt_speed_i);
            set_azi_motor_speed(azi_speed_i);
            // Delay in such a way that it last at least 150ms
            delay(150 - millis() + t0);
        }

        if(now >= next){
#ifdef DEBUG_VERBOSE_SCENE
            telnet.printf("END DALT %f DAZI %f\n", delta_alt, delta_azi);
#endif
#ifdef DEBUG_USE_ABSOLUTE_INSTEAD_OF_SOLAR
            alt_setpoint = scenes[sn][i*2]+alt_0;
            azi_setpoint = scenes[sn][i*2+1]+azi_0;
#else
            ray_to_setpoints(scenes[sn][i*2]+alt_0, scenes[sn][i*2+1]+azi_0);
#endif

            alt_target_speed = compute_speed(compute_angle_delta(alt_setpoint, alt_encoder_val), 
                                             SCENE_DT, 
                                             -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                             angular_speed_to_pwm_alt, (float) pwm_min_alt);
            alt_max_speed = compute_speed(compute_angle_delta(alt_setpoint, alt_encoder_val)*(1.+speed_range), 
                                          SCENE_DT,
                                          -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                          angular_speed_to_pwm_alt, (float) pwm_min_alt);
            alt_min_speed = compute_speed(compute_angle_delta(alt_setpoint, alt_encoder_val)*(1.-speed_range), 
                                          SCENE_DT,
                                          -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                          angular_speed_to_pwm_alt, (float) pwm_min_alt);

            azi_target_speed = compute_speed(compute_angle_delta(azi_setpoint, azi_encoder_val),
                                             SCENE_DT, 
                                             -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                             angular_speed_to_pwm_azi, (float) pwm_min_azi);
            azi_max_speed = compute_speed(compute_angle_delta(azi_setpoint, azi_encoder_val)*(1.+speed_range),
                                          SCENE_DT,
                                          -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                          angular_speed_to_pwm_azi, (float) pwm_min_azi);
            azi_min_speed = compute_speed(compute_angle_delta(azi_setpoint, azi_encoder_val)*(1.-speed_range),
                                          SCENE_DT,
                                          -PWM_MAX_VALUE, PWM_MAX_VALUE,
                                          angular_speed_to_pwm_azi, (float) pwm_min_azi);
#ifdef DEBUG_VERBOSE_SCENE
            telnet.printf("ALT %d %d %d\n", alt_min_speed, alt_target_speed, alt_max_speed);
            telnet.printf("AZI %d %d %d\n", azi_min_speed, azi_target_speed, azi_max_speed);
#endif
            
            set_alt_motor_speed(alt_target_speed);
            set_azi_motor_speed(azi_target_speed);

            now = millis();
            next = scene_beg + SCENE_DT * (i+1);

            i++;
        }
    }

    sys_log(LOG_INFO, "Scene ended after %.1f s (expected %.1f)\n", 1e-3*(millis()-scene_beg), 1e-3*SCENE_DT*(scene_len[sn]-1));

    alt_motor_standby();
    azi_motor_standby();
    return true;
}

bool run_sequence(uint8_t *scenes_seq, uint8_t sequence_len, float run_timestamp){
    float current_timestamp;
    bool wait = true;
    current_timestamp = get_timestamp_RTC();
    if(run_timestamp < 0 || current_timestamp < run_timestamp - SAFETY_TIME_BEFORE_FIRST_SCENE ){
        // Run immediately
        wait = false;
    }

    motor_driver_enable();
    if(wait){
        while(run_timestamp - SAFETY_TIME_BEFORE_FIRST_SCENE > current_timestamp){
            current_timestamp = get_timestamp_RTC();
            delay(10);
        }
    }

    uint32_t expected_time = 0;
    for(uint8_t i=0; i < sequence_len && i < MAX_SCENES_IN_SEQUENCE; i++){
        if(i==0){
            run_scene(scenes_seq[0], run_timestamp);
        }
        else{
            run_scene(scenes_seq[i]);
        }
        expected_time += (scene_len[scenes_seq[i]] - 1) * SCENE_DT;
    }
    current_timestamp = get_timestamp_RTC();

    sys_log(LOG_INFO, "Scene start time %.1f, scene end %.1f, elapsed %.1f (expected) %.1f\n", 
            run_timestamp, current_timestamp, current_timestamp-run_timestamp, 1.0*expected_time/1000);
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

bool wifi_watchdog_loop(void){
    if(telnet.isConnected()){
        if(millis() - telnet_watchdog_0 > TELNET_WATCHDOG_TIME){
            sys_log(LOG_INFO, "Telnet session disconnected by watchdog.");
            telnet.printf("\nSession disconnected by watchdog.\n");
            telnet.disconnectClient();
        }
        wifi_watchdog_0 = millis();
    }
    if(millis() - wifi_watchdog_0 > WIFI_WATCHDOG_TIME){
        if(WiFi_ok){
            if(telnet.isConnected()){
                wifi_watchdog_0 = millis();
                sys_log(LOG_INFO, "Wifi watchdog reset, a telnet client is conneccted.");
            }
            else{
                wifi_off();
                sys_log(LOG_INFO, "Wifi turned off by watchdog");
                return true;
            }
        }
    }
    return false;
}

bool run_pid_test(){
    float alt = 0.0, azi=0.0;

    motor_driver_enable();
    set_pid_goto(0., 0.);
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();

    for(alt = 0.0; alt < 360.; alt += 10.){
        telnet.printf("Going to ALT %f\n", alt);
        set_pid_goto(alt, 0.);
        while(alt_PID_enabled || azi_PID_enabled) pid_loop();
        telnet.printf("Done\n");
    }
    telnet.print("Going to ALT 0.0\n");
    set_pid_goto(0., 0.);
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();
    telnet.printf("Done\n");

    for(azi = 0.0; azi < 360.; azi += 10.){
        telnet.printf("Going to ALT %f\n", azi);
        set_pid_goto(0., azi);
        while(alt_PID_enabled || azi_PID_enabled) pid_loop();
        telnet.printf("Done\n");
    }
    telnet.print("Going to AZI 0.0\n");
    set_pid_goto(0., 0.);
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

    return true;
}

void sleep_for_seconds(uint32_t tts){
    uint32_t t0 = millis();
    if(tts > MAX_SLEEP_S) tts = MAX_SLEEP_S;
    sys_log(LOG_INFO, "Going to sleep for %d senonds.", tts);

    motor_driver_enable();
    set_pid_goto(sleep_alt, sleep_azi);
    while(alt_PID_enabled || azi_PID_enabled) pid_loop();

    wifi_off();
    Serial.flush();

    motor_driver_disable();
    
    //deep_sleep_hold_start();
    
    uint64_t dt = (uint64_t) (millis() - t0) * 1000ULL;
    esp_sleep_enable_timer_wakeup(tts * 1000000ULL - dt);
    esp_deep_sleep_start();
}

void sleep_loop(void){
    if(WiFi_ok){
        //sys_log(LOG_DEBUG, "Not going to sleep waiting for WiFi to be turned off by the watchdog.");
        return;
    }

    float next_schedule = seconds_to_next_schedule();
    //sys_log(LOG_DEBUG, "Next schedule %f (%f).", next_schedule, 1.2*SLEEP_TIME_S);
    if(next_schedule < 1.2*SLEEP_TIME_S){
        if(next_schedule < 2.0 * WAKEUP_TIME_BEFORE_SCHEDULE_S) return;
        else sleep_for_seconds((uint32_t) next_schedule - WAKEUP_TIME_BEFORE_SCHEDULE_S);
    }
    else sleep_for_seconds(SLEEP_TIME_S);
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
    sprintf(buf, "TIMESTAMP RTC %f\n", get_timestamp_RTC());
    telnet.print(buf);
    sprintf(buf, "TIMESTAMP DEFAULT %f\n", get_timestamp());
    telnet.print(buf);
    return true;
}

bool cmd_reboot(void){
    if(bootn > 0) set_int_cfg("bootn", 0L);
    telnet.disconnectClient();
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

bool cmd_set(char *buf){
    float valf;
    int32_t vali;
    char *key, *rest;

    key = strtok_r(NULL, " \n\r", &buf);
    rest = strtok_r(NULL, "\r\n", &buf);
    if(cfg_is_int(key)){
        sscanf(rest, "%d", &vali);

        if(set_int_cfg(key, vali)){
            return true;
        }
        else{
            telnet.printf("Key %s not found.\n", key);
            return false;
        }
    }
    else{
        sscanf(rest, "%f", &valf);

        if(set_float_cfg(key, valf)){
            return true;
        }
        else{
            telnet.printf("Key %s not found.\n", key);
            return false;
        }
    }
}

bool cmd_get(char *buf){
    float valf;
    int32_t vali;
    char *key;
    
    key = strtok_r(NULL, " \n\r", &buf);
    if(cfg_key_exists(key)){
        if(cfg_is_int(key)){
            vali = get_int_cfg(key);
            telnet.printf("%s = %d\n", key, vali);
        }
        else{
            valf = get_float_cfg(key);
            telnet.printf("%s = %g\n", key, valf);
        }
        return true;
    }
    else{
        telnet.printf("Key %s not found.\n", key);
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
    uint32_t t0, t1, tread, tconv;
    int16_t r1 = external_adc.readADC_SingleEnded(1);
    int16_t r0 = external_adc.readADC_SingleEnded(0);
    float v1 = external_adc.computeVolts(r1);
    float v0 = external_adc.computeVolts(r0);
    float a1, a0;

    t0 = micros();
    r1 = external_adc.readADC_SingleEnded(1);
    t1 = micros();
    tread = t1-t0;
    
    t0 = micros();
    v1 = external_adc.computeVolts(r1);
    t1 = micros();
    tconv = t1-t0;

    t0 = micros();
    a1 = read_alt_encoder();
    t1 = micros();

    telnet.printf("ADC0 %d %fV\n", r0, v0);
    telnet.printf("ADC1 %d %fV\n", r1, v1);
    telnet.printf("Tread = %d us Tconv = %d us\n", tread, tconv);
    telnet.printf("Tfull_read = %d  us\n", t1-t0);
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

bool cmd_control_reset(void){
    pid_control_reset();
    
    return true;
}

bool cmd_manual_control(char *buf){
    float azi, alt;

    sscanf(buf, "%f%f", &alt, &azi);
    
    set_pid_goto(alt, azi);

    manual_control_enabled = true;
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
    // Blind version use with coution only during calibration
    int t, s;

    sscanf(buf, "%d%d", &t, &s);
    if(s > PWM_MAX_VALUE) s = PWM_MAX_VALUE;
    if(s < -PWM_MAX_VALUE) s = -PWM_MAX_VALUE;
    if(t > MAX_BLIND_MOVE_TIME_MS) t=MAX_BLIND_MOVE_TIME_MS;
    if(s > 0){
        set_alt_motor_speed(s);
        delay(t);
        alt_motor_standby();
    }
    else {
        set_alt_motor_speed(s);
        delay(t);
        alt_motor_standby();
    }
    
    return true;
}

bool cmd_azi_move(char *buf){
    // Blind version use with coution only during calibration
    int t, s;

    sscanf(buf, "%d%d", &t, &s);
    if(s > PWM_MAX_VALUE) s = PWM_MAX_VALUE;
    if(s < -PWM_MAX_VALUE) s = -PWM_MAX_VALUE;
    if(t > MAX_BLIND_MOVE_TIME_MS) t=MAX_BLIND_MOVE_TIME_MS;
    if(s > 0){
        set_azi_motor_speed(s);
        delay(t);
        azi_motor_standby();
    }
    else {
        set_azi_motor_speed(s);
        delay(t);
        azi_motor_standby();
    }
    
    return true;
}

bool cmd_accel_curve(char *buf){
    // Blind version use with coution only during calibration
    int s, t;

    sscanf(buf, "%d%d", &t, &s);
    if(s < 0) s = -s;
    if(s > PWM_MAX_VALUE) s = PWM_MAX_VALUE;
    t *= 1000;
    if(t < 0) return false;
    if(t > MAX_BLIND_MOVE_TIME_MS) t = MAX_BLIND_MOVE_TIME_MS;

    float current_angle = read_alt_encoder(), last_angle;
    uint32_t t0, tc_1, tc;
    telnet.printf("ALT\n");
    t0 = millis();
    set_alt_motor_speed(s);
    tc = millis();
    while(tc-t0 < t){
        last_angle = current_angle;
        tc_1 = tc;
        current_angle = read_alt_encoder();
        tc = millis();
        telnet.printf("%d %.1f\n", tc-t0, current_angle);
    }
    set_alt_motor_speed(-s);
    while(tc-t0 < t*2){
        last_angle = current_angle;
        tc_1 = tc;
        current_angle = read_alt_encoder();
        tc = millis();
        telnet.printf("%d %.1f\n", tc-t0, current_angle);
    }
    alt_motor_standby();

    telnet.printf("AZI\n");
    current_angle = read_azi_encoder();
    t0 = millis();
    set_azi_motor_speed(s);
    tc = millis();
    while(tc-t0 < t){
        last_angle = current_angle;
        tc_1 = tc;
        current_angle = read_azi_encoder();
        tc = millis();
        telnet.printf("%d %.1f\n", tc-t0, current_angle);
    }
    set_azi_motor_speed(-s);
    while(tc-t0 < t*2){
        last_angle = current_angle;
        tc_1 = tc;
        current_angle = read_azi_encoder();
        tc = millis();
        telnet.printf("%d %.1f\n", tc-t0, current_angle);
    }
    azi_motor_standby();
    
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
                      scenes_name[i], scene_len[i], 1.0e-3 * scene_len[i] * SCENE_DT);
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

bool cmd_write_scene(char *rest){
    int ns;
    sscanf(rest, "%d%f%f", &ns);
    return  write_scene_on_file(ns);
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

bool cmd_sys_log(void){
    //TODO sort by days
    String *fnames = list_dir("/log/");
    String s;
    for(int i=0; i < MAX_LOG_FILES; i++){
        if(fnames[i] == "<<END>>") break;
        File file = LittleFS.open("/log/"+fnames[i]);
        if(!file || file.isDirectory()){
            sys_log(LOG_ERROR, "Failed to open file for reading %s", ("/log/"+fnames[i]).c_str());
            return false;
        }

        int j = 0;
        while(file.available()){
            s = file.readStringUntil('\n');
            telnet.println(s.c_str());
        }
        file.close();
    }
    return true;
}

bool cmd_time_to_next_schedule(void){
    float ttg = seconds_to_next_schedule();
    if(ttg > 0)
        telnet.printf("Next sheduled activity in %.0f s\n", ttg);
    else
        telnet.print("No next schedule.\n");
    return true;
}

bool cmd_battery_charge(void){
    float chg = read_battery_charge();
    if(chg > 0){
        telnet.printf("%.1f %%\n", chg);
        return true;
    }
    else{
        return false;
    }
}

bool cmd_test_scene(char *buf){
    char *sname;

    sname = strtok_r(NULL, "  ", &buf);
    String sn = String(sname);
    int j;
    for(j=0; j < scene_cnt; j++){
        if(sn == scenes_name[j]){
            break;
        }
    }
    if(j == scene_cnt){
        telnet.printf("Scene %s not found.\n", sn);
        return false;
    }
    run_scene(j);
    return true;
}

bool cmd_add_wifi_schedule(char *buf){
    uint32_t h, m, s;
    sscanf(buf, "%d %d %d", &h, &m, &s);
    if(h < 0 || h > 23) return false;
    if(m < 0 || m > 59) return false;
    if(s < 0 || s > 59) return false;
    add_scheduled_task_wifi(h, m, s);
    return true;
}

bool cmd_add_sequence_schedule(char *buf){
    uint8_t myseq[MAX_SCENES_IN_SEQUENCE] = {0};
    uint8_t seq_len = 0;
    char *next_scene_name, *tok;
    bool error = false;
    uint32_t h, m, s;

    tok = strtok(buf, " ");
    sscanf(tok, "%d", &h);
    if(h < 0 || h > 23) return false;
    tok = strtok(NULL, " ");
    sscanf(tok, "%d", &m);
    if(m < 0 || m > 59) return false;
    tok = strtok(NULL, " ");
    sscanf(tok, "%d", &s);
    if(s < 0 || s > 59) return false;

    next_scene_name = strtok(NULL, " ");

    while(next_scene_name != NULL){
        String nsn = String(next_scene_name);
        int j;
        for(j=0; j < scene_cnt; j++){
            if(nsn == scenes_name[j]){
                myseq[seq_len++] = j;
                break;
            }
        }
        if(j == scene_cnt){
            error = true;
            sys_log(LOG_ERROR, "Scene \"%s\" not found.", nsn);
        }
        next_scene_name = strtok(NULL, " ");
    }
    if(!error){
        add_scheduled_task_sequence(h, m, s, myseq, seq_len);
        sys_log(LOG_INFO, "Added sequence task at %02d:%02d:%02d (it will be scheduled %d'%d\" before)", 
                h, m, s, (int) SAFETY_TIME_BEFORE_SEQUENCE/60, (int) SAFETY_TIME_BEFORE_SEQUENCE%60 );
    }
    return true;
}

bool cmd_print_schedule(void){
    uint32_t h, m, s;

    for(int i=0; i<task_cnt; i++){
        uint64_t timestamp = sch_timestamp[i];
        if(sch_type[i] == SEQUENCE_TASK) timestamp += SAFETY_TIME_BEFORE_SEQUENCE;

        h = timestamp / 3600;
        m = (timestamp - 3600 * h) / 60;
        s = (timestamp - 3600 * h) % 60;

        telnet.printf("[%d] %02d:%02d:%02d ", i, h, m, s);

        if(sch_type[i] == WIFI_TASK){
            telnet.printf("wifi\n");
        }
        if(sch_type[i] == SEQUENCE_TASK){
            telnet.printf("sequence");
            for(int j=0; j < sch_sequence_len[i]; j++)
                telnet.printf(" %s", scenes_name[sch_sequence[i][j]]);
            telnet.printf("\n");
        }   
    }    
    return true;
}

bool cmd_save_current_schedule(void){
    uint32_t h, m, s;

    File file = LittleFS.open("/schedules/show.sch", FILE_WRITE);
    if(!file){
        sys_log(LOG_INFO, "Error in writing file: /schedules/show.sch");
        return false;
    }

    // Sequence tasks
    for(int i=0; i<task_cnt; i++){
        if(sch_type[i] != SEQUENCE_TASK) continue;
        
        uint64_t timestamp = sch_timestamp[i];
        timestamp += SAFETY_TIME_BEFORE_SEQUENCE;

        h = timestamp / 3600;
        m = (timestamp - 3600 * h) / 60;
        s = (timestamp - 3600 * h) % 60;

        file.printf("%02d %02d %02d sequence", h, m, s);
        for(int j=0; j < sch_sequence_len[i]; j++)
            file.printf(" %s", scenes_name[sch_sequence[i][j]]);
        file.printf("\n");
    }    

    file.close();
    
    file = LittleFS.open("/schedules/wifi.sch", FILE_WRITE);
    if(!file){
        sys_log(LOG_INFO, "Error in writing file: /schedules/wifi.sch");
        return false;
    }
    
    for(int i=0; i<task_cnt; i++){
        if(sch_type[i] != WIFI_TASK) continue;
        uint64_t timestamp = sch_timestamp[i];

        h = timestamp / 3600;
        m = (timestamp - 3600 * h) / 60;
        s = (timestamp - 3600 * h) % 60;

        file.printf("%02d %02d %02d wifi\n", h, m, s);
    }    
    file.close();
    return true;
}

bool cmd_delete_schedule(char *buf){
    uint16_t sn;
    sscanf(buf, "%d", &sn);
    if(sn >= 0  && sn < task_cnt){
        telnet.printf("Deleting schedule %d\n", sn);
        return delete_schedule(sn);
    }
    return false;
}

bool cmd_run_sequence(char *buf){
    uint8_t myseq[MAX_SCENES_IN_SEQUENCE] = {0};
    uint8_t seq_len = 0;
    char *next_scene_name, *tok;
    bool error = false;
    uint32_t h, m, s;

    next_scene_name = strtok(buf, " ");

    while(next_scene_name != NULL){
        String nsn = String(next_scene_name);
        int j;
        for(j=0; j < scene_cnt; j++){
            if(nsn == scenes_name[j]){
                myseq[seq_len++] = j;
                break;
            }
        }
        if(j == scene_cnt){
            error = true;
            telnet.printf("Scene \"%s\" not found.", nsn);
        }
        telnet.printf("Scene %d %s\n", myseq[seq_len-1], nsn.c_str());
        next_scene_name = strtok(NULL, " ");
    }
    if(!error && seq_len != 0){
        error = run_sequence(myseq, seq_len, get_timestamp() + 30);
        telnet_watchdog_0 = millis();
        return error;
    }
    telnet_watchdog_0 = millis();
    return false;
}

bool cmd_add_wifi(char *buf){
    char *ssid = strtok(buf, " ");
    char *pasw = strtok(NULL, " ");

    if(wifi_net_cnt < MAX_WIFI_NETWORKS){
        strcpy(wifi_ssids[wifi_net_cnt], ssid);
        strcpy(wifi_passwords[wifi_net_cnt], pasw);
        wifi_net_cnt++;
        return true;
    }
    return false;
}

bool cmd_delete_wifi(char *buf){
    int i;
    sscanf(buf, "%d", &i);
    return delete_wifi_credentials(i);
}

bool cmd_print_wifi(){
    for(int i=0; i < wifi_net_cnt; i++)
        telnet.printf("[%d] %s %s\n", i, wifi_ssids[i], wifi_passwords[i]);
    return true;
}

bool cmd_reload_prm(){
    azi_encoder_zero = get_float_cfg("azie0");
    alt_encoder_zero = get_float_cfg("alte0");

    alt_encoder_volt_to_deg = get_float_cfg("altv2d");
    azi_encoder_volt_to_deg = get_float_cfg("aziv2d");

    azi_motor_max_angular_speed = get_float_cfg("azi_Msv");
    alt_motor_max_angular_speed = get_float_cfg("alt_Msv");
    angular_speed_to_pwm_alt = get_float_cfg("alt_s2p");
    angular_speed_to_pwm_azi = get_float_cfg("azi_s2p");
    pwm_min_alt = get_int_cfg("alt_mPWM");
    pwm_min_azi = get_int_cfg("azi_mPWM");

    encoder_oversampling = get_int_cfg("overs");

    aziPID.set_PID_params(get_float_cfg("azi_kp"),
                          get_float_cfg("azi_ms"),
                          PWM_MAX_VALUE,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ms"),
                          PWM_MAX_VALUE,
                          get_float_cfg("alt_me"));
    
    sleep_alt = get_float_cfg("altnap");
    sleep_azi = get_float_cfg("azinap");
    log_level = get_int_cfg("logl");
    log_delete_after_days = get_int_cfg("logr");
    current_lon = get_float_cfg("lon");
    current_lat = get_float_cfg("lat");
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
    else if(strcmp(tok, "configs") == 0){
        configurations_log();
        return true;
    }
    else if(strcmp(tok, "factory-reset") == 0){
        return cmd_factory_reset();
    }
    else if(strcmp(tok, "mirror-log") == 0){
        return cmd_mirror_log();
    }
    else if(strcmp(tok, "quit") == 0){
        //WiFi watchdog start from now
        wifi_watchdog_0 = millis();
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
        return cmd_test_scene(rest);
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
    else if(strcmp(tok, "write-scene") == 0){
        return cmd_write_scene(rest);
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
    else if(strcmp(tok, "syslog") == 0){
        return cmd_sys_log();
    }  
    else if(strcmp(tok, "next-schedule") == 0){
        return cmd_time_to_next_schedule();
    }  
    else if(strcmp(tok, "battery") == 0){
        return cmd_battery_charge();
    }  
    else if(strcmp(tok, "add-task-wifi") == 0){
        return cmd_add_wifi_schedule(rest);
    } 
    else if(strcmp(tok, "add-task-sequence") == 0){
        return cmd_add_sequence_schedule(rest);
    } 
    else if(strcmp(tok, "print-schedule") == 0){
        return cmd_print_schedule();
    } 
    else if(strcmp(tok, "save-schedule") == 0){
        return cmd_save_current_schedule();
    }
    else if(strcmp(tok, "delete-schedule") == 0){
        return cmd_delete_schedule(rest);
    }  
    else if(strcmp(tok, "run-test-sequence") == 0){
        return cmd_run_sequence(rest);
    } 
    else if(strcmp(tok, "save-wifi") == 0){
        return save_wifi_credentials_on_file("/wifi_net.txt");
    } 
    else if(strcmp(tok, "add-wifi") == 0){
        return cmd_add_wifi(rest);
    } 
    else if(strcmp(tok, "delete-wifi") == 0){
        return cmd_delete_wifi(rest);
    } 
    else if(strcmp(tok, "print-wifi") == 0){
        return cmd_print_wifi();
    } 
    else if(strcmp(tok, "accel-calibration") == 0){
        return cmd_accel_curve(rest);
    }    
    else if(strcmp(tok, "reload-prm") == 0){
        return cmd_reload_prm();
    }    
    else{
        return cmd_err(tok);
    }
}

// Setup Routines

void onTelnetConnect(String ip) {
    sys_log(LOG_INFO, "Telnet %s client connected", ip.c_str());

    char buf[16];
    sprintf(buf, "%012x\n", chip_id);
  
    telnet.println("\nWelcome to heliograph\nIP: " + telnet.getIP() + "\nID: " + buf);
    telnet.println("Use quit to disconnect\n");
    cmd_system_status();
    telnet.print("[  ] > ");
    telnet_watchdog_0 = millis();
}

void onTelnetDisconnect(String ip) {
    sys_log(LOG_INFO, "Telnet %s client disconnected", ip.c_str());
}

void onTelnetReconnect(String ip) {
    sys_log(LOG_INFO, "Telnet %s client reconnected", ip.c_str());
}

void onTelnetConnectionAttempt(String ip) {
    sys_log(LOG_INFO, "Telnet %s client tried to connect", ip.c_str());
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
    telnet_watchdog_0 = millis();
}

void setup_telnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);

  if (telnet.begin(TELNET_PORT)) {
        sys_log(LOG_INFO, "Telnet running");
  } 
  else {
        sys_log(LOG_ERROR, "Telnet startup error");
  }
}

void setup_pref(void){
    HGPrefs.begin("hg", PREF_RW_MODE);
}

void setup_littlefs(void){
    if(!LittleFS.begin(true)){
        sys_log(LOG_ERROR, "LittleFS Mount Failed");
        littleFS_ok = false;
        return;
   }
   else{
        sys_log(LOG_INFO, "Little FS Mounted Successfully");
        littleFS_ok = true;
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

    alt_encoder_volt_to_deg = get_float_cfg("altv2d");
    azi_encoder_volt_to_deg = get_float_cfg("aziv2d");

    azi_motor_max_angular_speed = get_float_cfg("azi_Msv");
    alt_motor_max_angular_speed = get_float_cfg("alt_Msv");
    angular_speed_to_pwm_alt = get_float_cfg("alt_s2p");
    angular_speed_to_pwm_azi = get_float_cfg("azi_s2p");
    pwm_min_alt = get_int_cfg("alt_mPWM");
    pwm_min_azi = get_int_cfg("azi_mPWM");

    encoder_oversampling = get_int_cfg("overs");

    aziPID.set_PID_params(get_float_cfg("azi_kp"),
                          get_float_cfg("azi_ms"),
                          PWM_MAX_VALUE,
                          get_float_cfg("azi_me"));

    altPID.set_PID_params(get_float_cfg("alt_kp"),
                          get_float_cfg("alt_ms"),
                          PWM_MAX_VALUE,
                          get_float_cfg("alt_me"));
    
    sleep_alt = get_float_cfg("altnap");
    sleep_azi = get_float_cfg("azinap");

    azi_motor_standby();
    alt_motor_standby();
}

bool wifi_connect(uint8_t conn_idx){
    WiFi.begin(wifi_ssids[conn_idx], wifi_passwords[conn_idx]);
  
    sys_log(LOG_INFO, "Attempting wifi connection to %s", wifi_ssids[conn_idx]);
    sys_log(LOG_DEBUG, "Wifi password is %s", wifi_passwords[conn_idx]);

    for(uint8_t i=0;i < N_WIFI_ATTEMPTS && WiFi.status() != WL_CONNECTED; i++)
    {
      delay(1000);
    }
    if(WiFi.status() == WL_CONNECTED){
        sys_log(LOG_INFO, "Connected to Wifi IP address is %s.", WiFi.localIP().toString().c_str());
        WiFi_ok = true;
    }
    else{
        sys_log(LOG_WARNING, "WiFi connection failed after %d s.", N_WIFI_ATTEMPTS);
        WiFi_ok = false;
    }
    wifi_watchdog_0 = millis();
    return WiFi_ok;
}

void setup_wifi(){
    uint8_t last_wifi_net = get_int_cfg("lastnet");
    if(last_wifi_net < wifi_net_cnt){
        if(wifi_connect(last_wifi_net)) return;
    }
    else{
        sys_log(LOG_WARNING, "Network with index %d is no longer available", last_wifi_net);
    }
    
    sys_log(LOG_INFO, "Scanning all known networks.");
    uint8_t i;
    for(i=0; i < wifi_net_cnt && i < MAX_WIFI_NETWORKS; i++){
        if(i == last_wifi_net) continue;
        if(wifi_connect(i)) break;
    }
    set_int_cfg("lastnet", (int32_t) i);
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
    if(external_ADC_ok){
        external_adc.setDataRate(RATE_ADS1115_860SPS);
        external_adc.setGain(GAIN_ONE);
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

    setup_pref();
    chip_id = (uint32_t) ESP.getEfuseMac();
    bootn = get_int_cfg("bootn");
    log_level = get_int_cfg("logl");
    log_delete_after_days = get_int_cfg("logr");

    // Serial communication
    Serial.begin(9600);
    //deep_sleep_hold_stop();
    setup_motors();
    motor_driver_enable();

    setup_i2c();
    setup_rtc();
    setup_littlefs();

    uint16_t y;
    uint8_t m,d, h, mi;
    float s;
    get_time(&y, &m, &d, &h, &mi, &s);

    sys_log(LOG_INFO, "This is heligraph %012x", chip_id);
    sys_log(LOG_INFO, "Bootnumber %d @ %04d-%02d-%02dT%02d:%02d:%06.4fZ\n", bootn, y, m, d, h, mi, s);

    current_lon = get_float_cfg("lon");
    current_lat = get_float_cfg("lat");
    sys_log(LOG_INFO, "My Position is LON %.3f LAT %.3f", current_lon, current_lat);
    load_wifi_credentials_from_file("/wifi_net.txt");
    load_scenes_from_file();
    load_schedule_from_file("/schedules/show.sch");
    load_schedule_from_file("/schedules/wifi.sch");

    if(bootn <= 0 || !RTC_ok || (RTC_ok && !check_time())){
        if(bootn <= 0){
            sys_log(LOG_WARNING, "Boot number is < 0. Until it grows above 0 wifi will be on at every startup (%d s)", MAX_SLEEP_S);
            sys_log(LOG_INFO, "Use set bootn 1 to stop this from happening.");
        }
        else{
            if(!RTC_ok) sys_log(LOG_WARNING, "RTC is not working.");
            if(RTC_ok && !check_time()) sys_log(LOG_WARNING, "RTC is working, but date is wrong.");
            if(bootn != 0) sys_log(LOG_INFO, "Attempting WiFi connection to syncronize RTC.");
        }
        setup_wifi();
    }
    else{
        WiFi.mode(WIFI_OFF);
    }

    if(WiFi_ok){
        setup_ntp();
        sys_log(LOG_INFO, "NTP connected");
    }
    if(WiFi_ok && NTP_ok && RTC_ok){
        sync_RTC_from_NTP();
        sys_log(LOG_INFO, "External RTC synced with NTP");
    }
    if(RTC_ok){
        sync_internal_rtc();
        sys_log(LOG_INFO, "Internal RTC synced with external RTC");
    }
    if(!RTC_ok && NTP_ok){
        sync_internal_rtc_from_NTP();
        sys_log(LOG_INFO, "Internal RTC synced with NTP");
    }
    sys_log(LOG_DEBUG, "Default timestamp %.1f, internal timestamp %.1f");

    if(WiFi_ok) setup_telnet();

    setup_adc();
    if(external_ADC_ok) pid_control_reset();

    set_int_cfg("bootn", 1 + bootn);

    if(!check_time()){
        sys_log(LOG_ERROR, "No time information is available. Sleeping until I now what time is it.");
        sleep_for_seconds(MAX_SLEEP_S);
    }
    if(!external_ADC_ok){
        sys_log(LOG_ERROR, "External ADC not working, system is unable to move.");
    }
    cleanup_syslog();
    system_checks();
}

void loop() {
    //sys_log(LOG_DEBUG, "Running main loop");
    sleep_loop();
    if(WiFi_ok) telnet.loop();
    schedule_task_loop();
    wifi_watchdog_loop();
    
    if(!external_ADC_ok){
        alt_motor_standby();
        azi_motor_standby();
    }
    else{
        if(manual_control_enabled || solar_control_enabled){
            if(solar_control_enabled){
                ray_to_setpoints(ory_alt, ory_azi);
            }
            pid_loop();
            if(!azi_PID_enabled && !alt_PID_enabled){
                manual_control_enabled = false;
                solar_control_enabled = false;
                reset_pid_watchdog();
            }
        }
    }
}