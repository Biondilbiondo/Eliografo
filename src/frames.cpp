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
      correspond to Alt or Azi angles. It is in practice a rotated absolute_frame. It is also a 
      righte endend frame; Coordinates are generaly exposed as Alt/Azi angles in degrees. 
*/

// Rotation matrix initialized with identity this matrix is the rotation from absolute
// to internal frame.
float r[3][3] = {{1., 0., 0.},
                 {0., 1., 0.},
                 {0., 0., 1.}};

float inverse_r[3][3] = {{1., 0., 0.},
                         {0., 1., 0.},
                         {0., 0., 1.}};

bool is_rotation_frame_init = false;

float sc2a(float sine, float cosine){
    // Given sine and cosine of an angle compute the angle in radians

    float angle = asin(sine);
    if(cosine < 0){
        angle = M_PI - angle;
    }
    return angle;
}


void invert_transformation(float mat[3][3], float inv_mat[3][3]){
    // For unitary transformation det should be one
    float det = mat[0][0]*(mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1]) -
                mat[0][1]*(mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0]) +
                mat[0][2]*(mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0]);

    // Cofactor matrix
    float cof[3][3];

    cof[0][0] =  (mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1]);
    cof[0][1] = -(mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0]);
    cof[0][2] =  (mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0]);

    cof[1][0] = -(mat[0][1]*mat[2][2] - mat[0][2]*mat[2][1]);
    cof[1][1] =  (mat[0][0]*mat[2][2] - mat[0][2]*mat[2][0]);
    cof[1][2] = -(mat[0][0]*mat[2][1] - mat[0][1]*mat[2][0]);

    cof[2][0] =  (mat[0][1]*mat[1][2] - mat[0][2]*mat[1][1]);
    cof[2][1] = -(mat[0][0]*mat[1][2] - mat[0][2]*mat[1][0]);
    cof[2][2] =  (mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0]);

    // Transpose and divide by determinant to get inverse
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            inv_mat[i][j] = cof[j][i] / det;
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

void frame_transform(float *i, float r[3][3], float *o){
    /*Given a vector apply the transformation r to it, used to ratate internal frame to absolute and vice-versa*/
    o[_x_] = i[_x_] * r[_x_][_x_] + i[_y_] * r[_x_][_y_] + i[_z_] * r[_x_][_z_];
    o[_y_] = i[_x_] * r[_y_][_x_] + i[_y_] * r[_y_][_y_] + i[_z_] * r[_y_][_z_];
    o[_z_] = i[_x_] * r[_z_][_x_] + i[_y_] * r[_z_][_y_] + i[_z_] * r[_z_][_z_];
}

void absolute_to_internal_v(float *a, float *i){
    // TODO check frame initialization
    frame_transform(a, r, i);
}

float absolute_to_internal_azi(float *a){
    float iv[3], internal_azi_cosine, internal_azi_sine, n;
    frame_transform(a, r, iv);

    n = sqrt(iv[_x_] * iv[_x_] + iv[_y_] * iv[_y_]);
    internal_azi_sine = iv[_y_] / n;
    internal_azi_cosine = iv[_x_] / n;
    return sc2a(internal_azi_sine, internal_azi_cosine) * RAD2DEG;
}

float absolute_to_internal_alt(float *a){
    float iv[3], internal_alt_cosine, internal_alt_sine, n;
    frame_transform(a, r, iv);

    internal_alt_cosine = sqrt(iv[_x_] * iv[_x_] + iv[_y_] * iv[_y_]);
    internal_alt_sine = iv[_z_];
    n = sqrt(internal_alt_cosine *  internal_alt_cosine + internal_alt_sine * internal_alt_sine);
    return sc2a(internal_alt_sine/n, internal_alt_cosine/n) * RAD2DEG;
}

void internal_to_absolute(float alt, float azi, float *a){
    float i[3];
    // Compute internal unit vector
    i[_x_] = cos(alt * DEG2RAD) * cos(azi * DEG2RAD);
    i[_y_] = cos(alt * DEG2RAD) * sin(azi * DEG2RAD);
    i[_z_] = sin(alt * DEG2RAD);

    frame_transform(i, inverse_r, a);
}

void initialize_frame_rotation_empty(void){ 
    /* Initialize rotation matrix to identity, this means that the
       internal and absolute frame are just the same.
    */   


    r[_x_][_x_] = 1.0;
    r[_x_][_y_] = 0.0;
    r[_x_][_z_] = 0.0;

    r[_y_][_x_] = 0.0;
    r[_y_][_y_] = 1.0;
    r[_y_][_z_] = 0.0;

    r[_z_][_x_] = 0.0;
    r[_z_][_y_] = 0.0;
    r[_z_][_z_] = 1.0;

    invert_transformation(r, inverse_r);

    is_rotation_frame_init = true;
}
void internal_frame_get_rotation_matrix(float out[3][3]){
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = r[i][j];
}

void initialize_rotation_frame(float *g, float *m){
    /* Given the magnetic vector m and the gravity vector g, compute
    the rotation matrix (r) from absolute frame (EST = x, NORD = y, UP = z)
    to the internal frame.*/

    float g_n[3], m_n[3], nord[3], up[3], est[3], norm;
    
    is_rotation_frame_init = false;

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
    
    //Serial.printf("NORD: %+6.4f %+6.4f %+6.4f\n", nord[_x_], nord[_y_], nord[_z_]);

    // Normalize the nord
    norm = sqrt(nord[0]*nord[0] + nord[1]*nord[1] + nord[2]*nord[2]);
    nord[_x_] /= norm;
    nord[_y_] /= norm;
    nord[_z_] /= norm;

    // Up is the inverse of gravity
    up[_x_] = -g[_x_];
    up[_y_] = -g[_y_];
    up[_z_] = -g[_z_];

    // Est is nord x up to give a right-handed system
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

    invert_transformation(r, inverse_r);

    is_rotation_frame_init = true;
}

float internal_to_geo_alt(float internal_alt, float internal_azi){
    float abs[3];
    internal_to_absolute(internal_alt, internal_azi, abs);
    return absolute_to_geo_alt(abs);
}

float internal_to_geo_azi(float internal_alt, float internal_azi){
    float abs[3];
    internal_to_absolute(internal_alt, internal_azi, abs);
    return absolute_to_geo_azi(abs);
}

float geo_to_internal_azi(float geo_alt, float geo_azi){
    float abs[3];
    geo_to_absolute(geo_alt, geo_azi, abs);
    return absolute_to_internal_azi(abs);
}

float geo_to_internal_alt(float geo_alt, float geo_azi){
    float abs[3];
    geo_to_absolute(geo_alt, geo_azi, abs);
    return absolute_to_internal_alt(abs);
}