#include "ring_pid.h"


ringPID::ringPID(float *In, float *Out, float *SetP){
    set_PID_vars(In, Out, SetP);
    min_error = 0.0;
    max_error = 30.0;
    last_error = 0.0;
}

float ringPID::get_last_error(void){
    return last_error;
}

float ringPID::get_min_error(void){
    return min_error;
}

void ringPID::set_PID_params(float Kp, float Ki, float Kd, float OutMin, float OutMax, float MinErr){
    kp = Kp;
    ki = Ki;
    kd = Kd;
    outmax = OutMax;
    outmin = OutMin;
    min_error = MinErr;
}

void ringPID::set_max_speed(float OutMax){
    outmax = OutMax;
}

void ringPID::set_PID_vars(float *In, float *Out, float *SetP){
    input = In;
    output = Out;
    setpoint = SetP;
}

void ringPID::update(void){
    uint32_t now = micros();
    uint32_t timeChange = (now - lastT);
    char outm[256];


    float error = (*setpoint) - (*input);
    if(error > 180.0){
        error -= 360.0;
    }
    if(error < -180.0){
        error += 360.0;
    }

    last_error = error;

    if(abs(error) < min_error){
        *output = 0.0;
        return;
    }

    float p_comp = kp * error;
    float i_comp = 0.0;
    float d_comp = 0.0;

    *output = p_comp + i_comp + d_comp;
    if(*output > 0){
        if(*output > outmax) *output = outmax;
        if(*output < outmin) *output = outmin;
    }
    else{
        if(*output < -outmax) *output = -outmax;
        if(*output > -outmin) *output = -outmin;
    }

    lastT=now;
}