/****************************************************************************
 *
 *   Copyright (c) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file calibration_routines.cpp
 * Calibration routines implementations.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <string.h>
#include <mathlib/mathlib.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/sensor_combined.h>

#include <drivers/drv_tone_alarm.h>

#include "calibration_routines.h"
#include "calibration_messages.h"
#include "commander_helper.h"

int sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
			     float *sphere_radius)
{

	float x_sumplain = 0.0f;
	float x_sumsq = 0.0f;
	float x_sumcube = 0.0f;

	float y_sumplain = 0.0f;
	float y_sumsq = 0.0f;
	float y_sumcube = 0.0f;

	float z_sumplain = 0.0f;
	float z_sumsq = 0.0f;
	float z_sumcube = 0.0f;

	float xy_sum = 0.0f;
	float xz_sum = 0.0f;
	float yz_sum = 0.0f;

	float x2y_sum = 0.0f;
	float x2z_sum = 0.0f;
	float y2x_sum = 0.0f;
	float y2z_sum = 0.0f;
	float z2x_sum = 0.0f;
	float z2y_sum = 0.0f;

	for (unsigned int i = 0; i < size; i++) {

		float x2 = x[i] * x[i];
		float y2 = y[i] * y[i];
		float z2 = z[i] * z[i];

		x_sumplain += x[i];
		x_sumsq += x2;
		x_sumcube += x2 * x[i];

		y_sumplain += y[i];
		y_sumsq += y2;
		y_sumcube += y2 * y[i];

		z_sumplain += z[i];
		z_sumsq += z2;
		z_sumcube += z2 * z[i];

		xy_sum += x[i] * y[i];
		xz_sum += x[i] * z[i];
		yz_sum += y[i] * z[i];

		x2y_sum += x2 * y[i];
		x2z_sum += x2 * z[i];

		y2x_sum += y2 * x[i];
		y2z_sum += y2 * z[i];

		z2x_sum += z2 * x[i];
		z2y_sum += z2 * y[i];
	}

	//
	//Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
	//
	//    P is a structure that has been computed with the data earlier.
	//    P.npoints is the number of elements; the length of X,Y,Z are identical.
	//    P's members are logically named.
	//
	//    X[n] is the x component of point n
	//    Y[n] is the y component of point n
	//    Z[n] is the z component of point n
	//
	//    A is the x coordiante of the sphere
	//    B is the y coordiante of the sphere
	//    C is the z coordiante of the sphere
	//    Rsq is the radius squared of the sphere.
	//
	//This method should converge; maybe 5-100 iterations or more.
	//
	float x_sum = x_sumplain / size;        //sum( X[n] )
	float x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
	float x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
	float y_sum = y_sumplain / size;        //sum( Y[n] )
	float y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
	float y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
	float z_sum = z_sumplain / size;        //sum( Z[n] )
	float z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
	float z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )

	float xy = xy_sum / size;        //sum( X[n] * Y[n] )
	float xz = xz_sum / size;        //sum( X[n] * Z[n] )
	float yz = yz_sum / size;        //sum( Y[n] * Z[n] )
	float x2_y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
	float x2_z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
	float y2_x = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
	float y2_z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
	float z2_x = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
	float z2_y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	float f0 = x_sum2 + y_sum2 + z_sum2;
	float f1 =  0.5f * f0;
	float f2 = -8.0f * (x_sum3 + y2_x + z2_x);
	float f3 = -8.0f * (x2_y + y_sum3 + z2_y);
	float f4 = -8.0f * (x2_z + y2_z + z_sum3);

	//Set initial conditions:
	float a = x_sum;
	float b = y_sum;
	float c = z_sum;

	//First iteration computation:
	float a2 = a * a;
	float b2 = b * b;
	float c2 = c * c;
	float qs = a2 + b2 + c2;
	float qb = -2.0f * (a * x_sum + b * y_sum + c * z_sum);

	//Set initial conditions:
	float rsq = f0 + qb + qs;

	//First iteration computation:
	float q0 = 0.5f * (qs - rsq);
	float q1 = f1 + q0;
	float q2 = 8.0f * (qs - rsq + qb + f0);
	float a_a, a_b, a_c, n_a, n_b, n_c, d_a, d_b, d_c;

	//Iterate N times, ignore stop condition.
	unsigned int n = 0;

	while (n < max_iterations) {
		n++;

		//Compute denominator:
		a_a = q2 + 16.0f * (a2 - 2.0f * a * x_sum + x_sum2);
		a_b = q2 + 16.0f * (b2 - 2.0f * b * y_sum + y_sum2);
		a_c = q2 + 16.0f * (c2 - 2.0f * c * z_sum + z_sum2);
		a_a = (fabsf(a_a) < FLT_EPSILON) ? 1.0f : a_a;
		a_b = (fabsf(a_b) < FLT_EPSILON) ? 1.0f : a_b;
		a_c = (fabsf(a_c) < FLT_EPSILON) ? 1.0f : a_c;

		//Compute next iteration
		n_a = a - ((f2 + 16.0f * (b * xy + c * xz + x_sum * (-a2 - q0) + a * (x_sum2 + q1 - c * z_sum - b * y_sum))) / a_a);
		n_b = b - ((f3 + 16.0f * (a * xy + c * yz + y_sum * (-b2 - q0) + b * (y_sum2 + q1 - a * x_sum - c * z_sum))) / a_b);
		n_c = c - ((f4 + 16.0f * (a * xz + b * yz + z_sum * (-c2 - q0) + c * (z_sum2 + q1 - a * x_sum - b * y_sum))) / a_c);

		//Check for stop condition
		d_a = (n_a - a);
		d_b = (n_b - b);
		d_c = (n_c - c);

		if ((d_a * d_a + d_b * d_b + d_c * d_c) <= delta) { break; }

		//Compute next iteration's values
		a = n_a;
		b = n_b;
		c = n_c;
		a2 = a * a;
		b2 = b * b;
		c2 = c * c;
		qs = a2 + b2 + c2;
		qb = -2.0f * (a * x_sum + b * y_sum + c * z_sum);
		rsq = f0 + qb + qs;
		q0 = 0.5f * (qs - rsq);
		q1 = f1 + q0;
		q2 = 8.0f * (qs - rsq + qb + f0);
	}

	*sphere_x = a;
	*sphere_y = b;
	*sphere_z = c;
	*sphere_radius = sqrtf(rsq);

	return 0;
}

int ellipsoid_fit_least_squares(const float x[], const float y[], const float z[],
				unsigned int size, unsigned int max_iterations, float delta, float *offset_x, float *offset_y, float *offset_z,
				float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	float fitness = 1.0e30f, sphere_lambda = 1.0f, ellipsoid_lambda = 1.0f;

	for (int i = 0; i < max_iterations; i++) {
		run_lm_sphere_fit(x, y, z, fitness, sphere_lambda,
				  size, offset_x, offset_y, offset_z,
				  sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);

	}

	fitness = 1.0e30f;

	for (int i = 0; i < max_iterations; i++) {
		run_lm_ellipsoid_fit(x, y, z, fitness, ellipsoid_lambda,
				     size, offset_x, offset_y, offset_z,
				     sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);
	}

	return 0;
}

int run_lm_sphere_fit(const float x[], const float y[], const float z[], float &fitness, float &sphere_lambda,
		      unsigned int size, float *offset_x, float *offset_y, float *offset_z,
		      float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float samples_collected = size;
	float fitness = fitness;
	float fit1 = 0.0f, fit2 = 0.0f;

	float jtj[16];
	float jt_j2[16];
	float jtfi[4];
	float residual = 0.0f;
	memset(jtj, 0, sizeof(jtj));
	memset(jt_j2, 0, sizeof(jt_j2));
	memset(jtfi, 0, sizeof(jtfi));

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		float sphere_jacob[4];
		//Calculate Jacobian
		float a = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float b = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float c = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(a * a + b * b + c * c);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((*diag_x    * a) + (*offdiag_x * b) + (*offdiag_y * c)) / length);
		sphere_jacob[2] = 1.0f * (((*offdiag_x * a) + (*diag_y    * b) + (*offdiag_z * c)) / length);
		sphere_jacob[3] = 1.0f * (((*offdiag_y * a) + (*offdiag_z * b) + (*diag_z    * c)) / length);
		residual = *sphere_radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				jtj[i * 4 + j] += sphere_jacob[i] * sphere_jacob[j];
				jt_j2[i * 4 + j] += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
			}

			jtfi[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {*sphere_radius, *offset_x, *offset_y, *offset_z};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 4; i++) {
		jtj[i * 4 + i] += sphere_lambda;
		jt_j2[i * 4 + i] += sphere_lambda / lma_damping;
	}

	if (!inverse4x4(jtj, jtj)) {
		return -1;
	}

	if (!inverse4x4(jt_j2, jt_j2)) {
		return -1;
	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= jtfi[col] * jtj[row * 4 + col];
			fit2_params[row] -= jtfi[col] * jt_j2[row * 4 + col];
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float a = (*diag_x    * (x[k] - fit1_params[1])) + (*offdiag_x * (y[k] - fit1_params[2])) + (*offdiag_y *
				(z[k] + fit1_params[3]));
		float b = (*offdiag_x * (x[k] - fit1_params[1])) + (*diag_y    * (y[k] - fit1_params[2])) + (*offdiag_z *
				(z[k] + fit1_params[3]));
		float c = (*offdiag_y * (x[k] - fit1_params[1])) + (*offdiag_z * (y[k] - fit1_params[2])) + (*diag_z    *
				(z[k] - fit1_params[3]));
		float length = sqrtf(a * a + b * b + c * c);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		a = (*diag_x    * (x[k] - fit2_params[1])) + (*offdiag_x * (y[k] - fit2_params[2])) + (*offdiag_y *
				(z[k] - fit2_params[3]));
		b = (*offdiag_x * (x[k] - fit2_params[1])) + (*diag_y    * (y[k] - fit2_params[2])) + (*offdiag_z *
				(z[k] - fit2_params[3]));
		c = (*offdiag_y * (x[k] - fit2_params[1])) + (*offdiag_z * (y[k] - fit2_params[2])) + (*diag_z    *
				(z[k] - fit2_params[3]));
		length = sqrtf(a * a + b * b + c * c);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > fitness && fit2 > fitness) {
		sphere_lambda *= lma_damping;

	} else if (fit2 < fitness && fit2 < fit1) {
		sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (PX4_ISFINITE(fitness) && fitness < fitness) {
		fitness = fitness;
		*sphere_radius = fit1_params[0];
		*offset_x = fit1_params[1];
		*offset_y = fit1_params[2];
		*offset_z = fit1_params[3];
		return 0;

	} else {
		return -1;
	}
}

int run_lm_ellipsoid_fit(const float x[], const float y[], const float z[], float &fitness, float &sphere_lambda,
			 unsigned int size, float *offset_x, float *offset_y, float *offset_z,
			 float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float samples_collected = size;
	float fitness = fitness;
	float fit1 = 0.0f, fit2 = 0.0f;

	float jtj[81];
	float jt_j2[81];
	float jtfi[9];
	float residual = 0.0f;
	memset(jtj, 0, sizeof(jtj));
	memset(jt_j2, 0, sizeof(jt_j2));
	memset(jtfi, 0, sizeof(jtfi));
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		//Calculate Jacobian
		float a = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float b = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float c = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(a * a + b * b + c * c);
		residual = *sphere_radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((*diag_x    * a) + (*offdiag_x * b) + (*offdiag_y * c)) / length);
		ellipsoid_jacob[1] = 1.0f * (((*offdiag_x * a) + (*diag_y    * b) + (*offdiag_z * c)) / length);
		ellipsoid_jacob[2] = 1.0f * (((*offdiag_y * a) + (*offdiag_z * b) + (*diag_z    * c)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((x[k] + *offset_x) * a) / length;
		ellipsoid_jacob[4] = -1.0f * ((y[k] + *offset_y) * b) / length;
		ellipsoid_jacob[5] = -1.0f * ((z[k] + *offset_z) * c) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((y[k] + *offset_y) * a) + ((x[k] + *offset_x) * b)) / length;
		ellipsoid_jacob[7] = -1.0f * (((z[k] + *offset_z) * a) + ((x[k] + *offset_x) * c)) / length;
		ellipsoid_jacob[8] = -1.0f * (((z[k] + *offset_z) * b) + ((y[k] + *offset_y) * c)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				jtj[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
				jt_j2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; //a backup JTJ for LM
			}

			jtfi[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {*offset_x, *offset_y, *offset_z, *diag_x, *diag_y, *diag_z, *offdiag_x, *offdiag_y, *offdiag_z};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 9; i++) {
		jtj[i * 9 + i] += sphere_lambda;
		jt_j2[i * 9 + i] += sphere_lambda / lma_damping;
	}


	if (!mat_inverse(jtj, jtj, 9)) {
		return -1;
	}

	if (!mat_inverse(jt_j2, jt_j2, 9)) {
		return -1;
	}



	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= jtfi[col] * jtj[row * 9 + col];
			fit2_params[row] -= jtfi[col] * jt_j2[row * 9 + col];
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float a = (fit1_params[3]    * (x[k] - fit1_params[0])) + (fit1_params[6] * (y[k] - fit1_params[1])) + (fit1_params[7] *
				(z[k] - fit1_params[2]));
		float b = (fit1_params[6] * (x[k] - fit1_params[0])) + (fit1_params[4]   * (y[k] - fit1_params[1])) + (fit1_params[8] *
				(z[k] - fit1_params[2]));
		float c = (fit1_params[7] * (x[k] - fit1_params[0])) + (fit1_params[8] * (y[k] - fit1_params[1])) + (fit1_params[5]    *
				(z[k] - fit1_params[2]));
		float length = sqrtf(a * a + b * b + c * c);
		residual = *sphere_radius - length;
		fit1 += residual * residual;

		a = (fit2_params[3]    * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
				(z[k] - fit2_params[2]));
		b = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4]   * (y[k] - fit2_params[1])) + (fit2_params[8] *
				(z[k] - fit2_params[2]));
		c = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5]    *
				(z[k] - fit2_params[2]));
		length = sqrtf(a * a + b * b + c * c);
		residual = *sphere_radius - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > fitness && fit2 > fitness) {
		sphere_lambda *= lma_damping;

	} else if (fit2 < fitness && fit2 < fit1) {
		sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness < fitness) {
		fitness = fitness;
		*offset_x = fit1_params[0];
		*offset_y = fit1_params[1];
		*offset_z = fit1_params[2];
		*diag_x = fit1_params[3];
		*diag_y = fit1_params[4];
		*diag_z = fit1_params[5];
		*offdiag_x = fit1_params[6];
		*offdiag_y = fit1_params[7];
		*offdiag_z = fit1_params[8];
		return 0;

	} else {
		return -1;
	}
}

enum detect_orientation_return detect_orientation(orb_advert_t *mavlink_log_pub, int cancel_sub, int accel_sub,
		bool lenient_still_position)
{
	const unsigned ndim = 3;

	struct sensor_combined_s sensor;
	float		accel_ema[ndim] = { 0.0f };		// exponential moving average of accel
	float		accel_disp[3] = { 0.0f, 0.0f, 0.0f };	// max-hold dispersion of accel
	float		ema_len = 0.5f;				// EMA time constant in seconds
	const float	normal_still_thr = 0.25;		// normal still threshold
	float		still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
	float		accel_err_thr = 5.0f;			// set accel error threshold to 5m/s^2
	hrt_abstime	still_time = lenient_still_position ? 500000 : 1300000;	// still time required in us

	px4_pollfd_struct_t fds[1];
	fds[0].fd = accel_sub;
	fds[0].events = POLLIN;

	hrt_abstime t_start = hrt_absolute_time();
	/* set timeout to 30s */
	hrt_abstime timeout = 30000000;
	hrt_abstime t_timeout = t_start + timeout;
	hrt_abstime t = t_start;
	hrt_abstime t_prev = t_start;
	hrt_abstime t_still = 0;

	unsigned poll_errcount = 0;

	while (true) {
		/* wait blocking for new data */
		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), accel_sub, &sensor);
			t = hrt_absolute_time();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;

			for (unsigned i = 0; i < ndim; i++) {

				float di = sensor.accelerometer_m_s2[i];

				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);

				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}

				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}

			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2) {
				/* is still now */
				if (t_still == 0) {
					/* first time */
					calibration_log_info(mavlink_log_pub, "[cal] detected rest position, hold still...");
					t_still = t;
					t_timeout = t + timeout;

				} else {
					/* still since t_still */
					if (t > t_still + still_time) {
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}

			} else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) {
				/* not still, reset still start time */
				if (t_still != 0) {
					calibration_log_info(mavlink_log_pub, "[cal] detected motion, hold still...");
					usleep(200000);
					t_still = 0;
				}
			}

		} else if (poll_ret == 0) {
			poll_errcount++;
		}

		if (t > t_timeout) {
			poll_errcount++;
		}

		if (poll_errcount > 1000) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SENSOR_MSG);
			return DETECT_ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_TAIL_DOWN;        // [ g, 0, 0 ]
	}

	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_LEFT;        // [ 0, g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_UPSIDE_DOWN;        // [ 0, 0, g ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
		return DETECT_ORIENTATION_RIGHTSIDE_UP;        // [ 0, 0, -g ]
	}

	calibration_log_critical(mavlink_log_pub, "[cal] ERROR: invalid orientation");

	return DETECT_ORIENTATION_ERROR;	// Can't detect orientation
}

const char *detect_orientation_str(enum detect_orientation_return orientation)
{
	static const char *rg_orientation_strs[] = {
		"back",		// tail down
		"front",	// nose down
		"left",
		"right",
		"up",		// upside-down
		"down",		// right-side up
		"error"
	};

	return rg_orientation_strs[orientation];
}

calibrate_return calibrate_from_orientation(orb_advert_t *mavlink_log_pub,
		int		cancel_sub,
		bool	side_data_collected[detect_orientation_side_count],
		calibration_from_orientation_worker_t calibration_worker,
		void	*worker_data,
		bool	lenient_still_position)
{
	calibrate_return result = calibrate_return_ok;

	// Setup subscriptions to onboard accel sensor

	int sub_accel = orb_subscribe(ORB_ID(sensor_combined));

	if (sub_accel < 0) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "No onboard accel");
		return calibrate_return_error;
	}

	unsigned orientation_failures = 0;

	// Rotate through all requested orientation
	while (true) {
		if (calibrate_cancel_check(mavlink_log_pub, cancel_sub)) {
			result = calibrate_return_cancelled;
			break;
		}

		if (orientation_failures > 4) {
			result = calibrate_return_error;
			calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "timeout: no motion");
			break;
		}

		unsigned int side_complete_count = 0;

		// Update the number of completed sides
		for (unsigned i = 0; i < detect_orientation_side_count; i++) {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}

		if (side_complete_count == detect_orientation_side_count) {
			// We have completed all sides, move on
			break;
		}

		/* inform user which orientations are still needed */
		char pending_str[80];
		pending_str[0] = 0;

		for (unsigned int cur_orientation = 0; cur_orientation < detect_orientation_side_count; cur_orientation++) {
			if (!side_data_collected[cur_orientation]) {
				strncat(pending_str, " ", sizeof(pending_str) - 1);
				strncat(pending_str, detect_orientation_str((enum detect_orientation_return)cur_orientation), sizeof(pending_str) - 1);
			}
		}

		calibration_log_info(mavlink_log_pub, "[cal] pending:%s", pending_str);
		usleep(20000);
		calibration_log_info(mavlink_log_pub, "[cal] hold vehicle still on a pending side");
		usleep(20000);
		enum detect_orientation_return orient = detect_orientation(mavlink_log_pub, cancel_sub, sub_accel,
							lenient_still_position);

		if (orient == DETECT_ORIENTATION_ERROR) {
			orientation_failures++;
			calibration_log_info(mavlink_log_pub, "[cal] detected motion, hold still...");
			usleep(20000);
			continue;
		}

		/* inform user about already handled side */
		if (side_data_collected[orient]) {
			orientation_failures++;
			set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
			calibration_log_info(mavlink_log_pub, "[cal] %s side already completed", detect_orientation_str(orient));
			usleep(20000);
			continue;
		}

		calibration_log_info(mavlink_log_pub, CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		usleep(20000);
		calibration_log_info(mavlink_log_pub, CAL_QGC_ORIENTATION_DETECTED_MSG, detect_orientation_str(orient));
		usleep(20000);
		orientation_failures = 0;

		// Call worker routine
		result = calibration_worker(orient, cancel_sub, worker_data);

		if (result != calibrate_return_ok) {
			break;
		}

		calibration_log_info(mavlink_log_pub, CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		usleep(20000);
		calibration_log_info(mavlink_log_pub, CAL_QGC_SIDE_DONE_MSG, detect_orientation_str(orient));
		usleep(20000);

		// Note that this side is complete
		side_data_collected[orient] = true;

		// output neutral tune
		set_tune(TONE_NOTIFY_NEUTRAL_TUNE);

		// temporary priority boost for the white blinking led to come trough
		rgbled_set_color_and_mode(led_control_s::color_white, led_control_s::mode_blink_fast, 3, 1);
		usleep(200000);
	}

	if (sub_accel >= 0) {
		px4_close(sub_accel);
	}

	return result;
}

int calibrate_cancel_subscribe()
{
	return orb_subscribe(ORB_ID(vehicle_command));
}

void calibrate_cancel_unsubscribe(int cmd_sub)
{
	orb_unsubscribe(cmd_sub);
}

static void calibrate_answer_command(orb_advert_t *mavlink_log_pub, struct vehicle_command_s &cmd, unsigned result)
{
	switch (result) {
	case vehicle_command_s::vehicle_cmd_result_accepted:
		tune_positive(true);
		break;

	case vehicle_command_s::vehicle_cmd_result_denied:
		mavlink_log_critical(mavlink_log_pub, "command denied during calibration: %u", cmd.command);
		tune_negative(true);
		break;

	default:
		break;
	}
}

bool calibrate_cancel_check(orb_advert_t *mavlink_log_pub, int cancel_sub)
{
	px4_pollfd_struct_t fds[1];
	fds[0].fd = cancel_sub;
	fds[0].events = POLLIN;

	if (px4_poll(&fds[0], 1, 0) > 0) {
		struct vehicle_command_s cmd;

		orb_copy(ORB_ID(vehicle_command), cancel_sub, &cmd);

		// ignore internal commands, such as VEHICLE_CMD_DO_MOUNT_CONTROL from vmount
		if (cmd.from_external) {
			if (cmd.command == vehicle_command_s::vehicle_cmd_preflight_calibration &&
					(int)cmd.param1 == 0 &&
					(int)cmd.param2 == 0 &&
					(int)cmd.param3 == 0 &&
					(int)cmd.param4 == 0 &&
					(int)cmd.param5 == 0 &&
					(int)cmd.param6 == 0) {
				calibrate_answer_command(mavlink_log_pub, cmd, vehicle_command_s::vehicle_cmd_result_accepted);
				mavlink_log_critical(mavlink_log_pub, CAL_QGC_CANCELLED_MSG);
				return true;

			} else {
				calibrate_answer_command(mavlink_log_pub, cmd, vehicle_command_s::vehicle_cmd_result_denied);
			}
		}
	}

	return false;
}
