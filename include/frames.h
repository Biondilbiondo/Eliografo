#ifndef _frames_h_
#define _frames_h_
#include <math.h>

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

#define _x_ 0
#define _y_ 1
#define _z_ 2

// Low Level
float sc2a(float sine, float cosine);
void frame_transform(float *, float **, float *);

// Geo <-> Absolute
void geo_to_absolute(float alt, float azi, float *v);
float absolute_to_geo_alt(float *v);
float absolute_to_geo_azi(float *v);

// Absolute <-> Internal
void initialize_rotation_frame(float *g, float *m);
void initialize_frame_rotation_empty(void);
void internal_frame_get_rotation_matrix(float out[3][3]);

void absolute_to_internal_v(float *a, float *i);
float absolute_to_internal_azi(float *a);
float absolute_to_internal_alt(float *a);
void internal_to_absolute(float alt, float azi, float *a);

// Geo <-> Internal
float internal_to_geo_alt(float internal_alt, float internal_azi);
float internal_to_geo_azi(float internal_alt, float internal_azi);
float geo_to_internal_azi(float geo_alt, float geo_azi);
float geo_to_internal_alt(float geo_alt, float geo_azi);

#endif