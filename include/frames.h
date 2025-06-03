#ifndef _frames_h_
#define _frames_h_
#include <math.h>

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

#define _x_ 0
#define _y_ 1
#define _z_ 2

float sc2a(float sine, float cosine);
void geo_to_absolute(float alt, float azi, float *v);
float absolute_to_geo_alt(float *v);
float absolute_to_geo_azi(float *v);

#endif