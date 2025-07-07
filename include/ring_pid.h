#ifndef _ring_pid_h_
#define _ring_pid_h_
#include <stdint.h>
#include <Arduino.h>

class ringPID{
    private:
        // Input should be 0-360
        float *input;
        float *output;
        float *setpoint;

        float ki, kp, kd;
        float outmax, outmin;
        float min_error, max_error;
        float last_error;
        uint32_t lastT;

    public:
        ringPID(float *In, float *Out, float *SetP);
        float get_last_error(void);
        float get_min_error(void);
        void set_PID_params(float Kp, float Ki, float Kd, float OutMin, float OutMax, float MinErr);
        void set_PID_vars(float *In, float *Out, float *SetP);
        void update(void);
};
#endif