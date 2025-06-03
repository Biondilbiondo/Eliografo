#include "frames.h"

/* FRAMES
There are different frames for different pourposes:
    - geo_frame, is an absolute frame used for input/output, here coordinates are 
      always given as a alt/azi pair in degrees. Alt is from horizon up, Azi is from 
      north clockwise.
    - absolute_frame, is the frame used for internal calculations, is a cartesian frame
      so as far as possible every point is represented with coordinates and every direction
      with unit vectors. 
      If needed Alt/Azi are given as follow: 
        Azi from X axis counterclockwise
        Alt from XY plane through Z axis
      Axis are binded to absolute geographic direction. 
      X axis is East, Y axis is North, Z axis is Up. This is a right-handed system
    - internal_frame, this is a frame internal to the structure, so that motors directly
      correspond to Alt or Azi angles. It is in practice a rotated absolute_frame.
*/

float sc2a(float sine, float cosine){
    // Given sine and cosine of an angle compute the angle in radians

    float angle = asin(sine);
    if(cosine < 0){
        angle = M_PI - angle;
    }
    return angle;
}

void geo_to_absolute(float alt, float azi, float *v){
    // Convert geo_frame alt/azi coord to unit vector in absolute frame.

    float azi_rad, absolute_azi_rad, absolute_alt_rad;

    // Convert azimuth in absolute frame, so ccw from East
    azi_rad = azi * DEG2RAD;
    absolute_azi_rad = M_PI/2 - azi_rad;
    absolute_alt_rad = alt * DEG2RAD;

    v[_x_] = cos(absolute_alt_rad) * cos(absolute_azi_rad);
    v[_y_] = cos(absolute_alt_rad) * sin(absolute_azi_rad);
    v[_z_] = sin(absolute_alt_rad);
}

float absolute_to_geo_alt(float *v){
    //Compute Alt in geo_frame from unit vector in absolute frame
    float absolute_alt_sine, absolute_alt_cosine, absolute_alt;

    absolute_alt_cosine = sqrt(v[_x_] * v[_x_] + v[_y_] * v[_y_]);
    absolute_alt_sine = v[_z_];
    absolute_alt = sc2a(absolute_alt_sine, absolute_alt_cosine);
    return absolute_alt * RAD2DEG;
}

float absolute_to_geo_azi(float *v){
    //Compute Azi in geo_frame from unit vector in absolute frame
    float norm, absolute_azi_sine, absolute_azi_cosine, absolute_azi;

    norm = sqrt(v[_x_] * v[_x_] + v[_y_] * v[_y_]);
    absolute_azi_sine = v[_y_] / norm;
    absolute_azi_cosine = v[_x_] / norm;
    absolute_azi = sc2a(absolute_azi_sine, absolute_azi_cosine);
    return 90.0 - absolute_azi * RAD2DEG;
}