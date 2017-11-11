/****************************************************************************
* Copyright (c) 2014, Paul Riseborough All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of the {organization} nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/**
 * @file estimator_utilities.cpp
 *
 * Estimator support utilities.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "estimator_utilities.h"

// Define EKF_DEBUG here to enable the debug print calls
// if the macro is not set, these will be completely
// optimized out by the compiler.
//#define EKF_DEBUG

#ifdef EKF_DEBUG
#include <stdio.h>
#include <stdarg.h>

static void
ekf_debug_print(const char *fmt, va_list args)
{
    fprintf(stderr, "%s: ", "[ekf]");
    vfprintf(stderr, fmt, args);

    fprintf(stderr, "\n");
}

void
ekf_debug(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    ekf_debug_print(fmt, args);
    va_end(args);
}

#else

void ekf_debug(const char *fmt, ...) { while(0){} }
#endif

/* we don't want to pull in the standard lib just to swap two floats */
void swap_var(float &d1, float &d2);

float Vector3f::length() const
{
    return sqrtf(x*x + y*y + z*z);
}

void Vector3f::zero()
{
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

Mat3f::Mat3f() :
    x{1.0f, 0.0f, 0.0f},
    y{0.0f, 1.0f, 0.0f},
    z{0.0f, 0.0f, 1.0f}
{
}

void Mat3f::identity() {
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;

    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;

    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
}

Mat3f Mat3f::transpose() const
{
    Mat3f ret = *this;
    swap_var(ret.x.y, ret.y.x);
    swap_var(ret.x.z, ret.z.x);
    swap_var(ret.y.z, ret.z.y);
    return ret;
}

void calcvel_ned(float (&vel_ne_dr)[3], float gps_course, float gps_gnd_spd, float gps_vel_d)
{
    vel_ne_dr[0] = gps_gnd_spd*cosf(gps_course);
    vel_ne_dr[1] = gps_gnd_spd*sinf(gps_course);
    vel_ne_dr[2] = gps_vel_d;
}

void calcpos_ned(float (&pos_ne_dr)[3], double lat, double lon, float hgt, double lat_reference, double lon_reference, float hgt_reference)
{
    pos_ne_dr[0] = earthRadius * (lat - lat_reference);
    pos_ne_dr[1] = earthRadius * cos(lat_reference) * (lon - lon_reference);
    pos_ne_dr[2] = -(hgt - hgt_reference);
}

void calc_llh(float pos_ne_di[3], double &lat, double &lon, float &hgt, double lat_ref, double lon_ref, float hgt_ref)
{
    lat = lat_ref + (double)pos_ne_di[0] * earthRadiusInv;
    lon = lon_ref + (double)pos_ne_di[1] * earthRadiusInv / cos(lat_ref);
    hgt = hgt_ref - pos_ne_di[2];
}

// overload + operator to provide a vector addition
Vector3f operator+(const Vector3f &vec_in1, const Vector3f &vec_in2)
{
    Vector3f vec_out;
    vec_out.x = vec_in1.x + vec_in2.x;
    vec_out.y = vec_in1.y + vec_in2.y;
    vec_out.z = vec_in1.z + vec_in2.z;
    return vec_out;
}

// overload - operator to provide a vector subtraction
Vector3f operator-(const Vector3f &vec_in1, const Vector3f &vec_in2)
{
    Vector3f vec_out;
    vec_out.x = vec_in1.x - vec_in2.x;
    vec_out.y = vec_in1.y - vec_in2.y;
    vec_out.z = vec_in1.z - vec_in2.z;
    return vec_out;
}

// overload * operator to provide a matrix vector product
Vector3f operator*(const Mat3f &mat_in, const Vector3f &vec_in)
{
    Vector3f vec_out;
    vec_out.x = mat_in.x.x*vec_in.x + mat_in.x.y*vec_in.y + mat_in.x.z*vec_in.z;
    vec_out.y = mat_in.y.x*vec_in.x + mat_in.y.y*vec_in.y + mat_in.y.z*vec_in.z;
    vec_out.z = mat_in.z.x*vec_in.x + mat_in.z.y*vec_in.y + mat_in.z.z*vec_in.z;
    return vec_out;
}

// overload * operator to provide a matrix product
Mat3f operator*(const Mat3f &mat_in1, const Mat3f &mat_in2)
{
    Mat3f mat_out;
    mat_out.x.x = mat_in1.x.x*mat_in2.x.x + mat_in1.x.y*mat_in2.y.x + mat_in1.x.z*mat_in2.z.x;
    mat_out.x.y = mat_in1.x.x*mat_in2.x.y + mat_in1.x.y*mat_in2.y.y + mat_in1.x.z*mat_in2.z.y;
    mat_out.x.z = mat_in1.x.x*mat_in2.x.z + mat_in1.x.y*mat_in2.y.z + mat_in1.x.z*mat_in2.z.z;

    mat_out.y.x = mat_in1.y.x*mat_in2.x.x + mat_in1.y.y*mat_in2.y.x + mat_in1.y.z*mat_in2.z.x;
    mat_out.y.y = mat_in1.y.x*mat_in2.x.y + mat_in1.y.y*mat_in2.y.y + mat_in1.y.z*mat_in2.z.y;
    mat_out.y.z = mat_in1.y.x*mat_in2.x.z + mat_in1.y.y*mat_in2.y.z + mat_in1.y.z*mat_in2.z.z;

    mat_out.z.x = mat_in1.z.x*mat_in2.x.x + mat_in1.z.y*mat_in2.y.x + mat_in1.z.z*mat_in2.z.x;
    mat_out.z.y = mat_in1.z.x*mat_in2.x.y + mat_in1.z.y*mat_in2.y.y + mat_in1.z.z*mat_in2.z.y;
    mat_out.z.z = mat_in1.z.x*mat_in2.x.z + mat_in1.z.y*mat_in2.y.z + mat_in1.z.z*mat_in2.z.z;

    return mat_out;
}

// overload % operator to provide a vector cross product
Vector3f operator%(const Vector3f &vec_in1, const Vector3f &vec_in2)
{
    Vector3f vec_out;
    vec_out.x = vec_in1.y*vec_in2.z - vec_in1.z*vec_in2.y;
    vec_out.y = vec_in1.z*vec_in2.x - vec_in1.x*vec_in2.z;
    vec_out.z = vec_in1.x*vec_in2.y - vec_in1.y*vec_in2.x;
    return vec_out;
}

// overload * operator to provide a vector scaler product
Vector3f operator*(const Vector3f &vec_in1, const float scl_in1)
{
    Vector3f vec_out;
    vec_out.x = vec_in1.x * scl_in1;
    vec_out.y = vec_in1.y * scl_in1;
    vec_out.z = vec_in1.z * scl_in1;
    return vec_out;
}

// overload * operator to provide a vector scaler product
Vector3f operator*(float scl_in1, const Vector3f &vec_in1)
{
    Vector3f vec_out;
    vec_out.x = vec_in1.x * scl_in1;
    vec_out.y = vec_in1.y * scl_in1;
    vec_out.z = vec_in1.z * scl_in1;
    return vec_out;
}

// overload / operator to provide a vector scalar division
Vector3f operator/(const Vector3f &vec, const float scalar)
{
    Vector3f vec_out;
    vec_out.x = vec.x / scalar;
    vec_out.y = vec.y / scalar;
    vec_out.z = vec.z / scalar;
    return vec_out;
}

void swap_var(float &d1, float &d2)
{
    float tmp = d1;
    d1 = d2;
    d2 = tmp;
}
