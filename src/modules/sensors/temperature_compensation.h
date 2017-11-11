/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file temperature_compensation.h
 *
 * Sensor correction methods
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>

#include "common.h"


namespace sensors
{

static_assert(gyro_count_max == 3,
	      "GYRO_COUNT_MAX must be 3 (if changed, add/remove TC_* params to match the count)");
static_assert(accel_count_max == 3,
	      "ACCEL_COUNT_MAX must be 3 (if changed, add/remove TC_* params to match the count)");
static_assert(baro_count_max == 3,
	      "BARO_COUNT_MAX must be 3 (if changed, add/remove TC_* params to match the count)");

/**
 ** class TemperatureCompensation
 * Applies temperature compensation to sensor data. Loads the parameters from PX4 param storage.
 */
class TemperatureCompensation
{
public:

	/** (re)load the parameters. Make sure to call this on startup as well */
	int parametersUpdate();

	/** supply information which device_id matches a specific uORB topic_instance
	 *  (needed if a system has multiple sensors of the same type)
	 *  @return index for compensation parameter entry containing matching device ID on success, <0 otherwise */
	int setSensorIdGyro(uint32_t device_id, int topic_instance);
	int setSensorIdAccel(uint32_t device_id, int topic_instance);
	int setSensorIdBaro(uint32_t device_id, int topic_instance);


	/**
	 * Apply Thermal corrections to gyro (& other) sensor data.
	 * @param topic_instance uORB topic instance
	 * @param sensor_data input sensor data, output sensor data with applied corrections
	 * @param temperature measured current temperature
	 * @param offsets returns offsets that were applied (length = 3, except for baro), depending on return value
	 * @param scales returns scales that were applied (length = 3), depending on return value
	 * @return -1: error: correction enabled, but no sensor mapping set (@see set_sendor_id_gyro)
	 *         0: no changes (correction not enabled),
	 *         1: corrections applied but no changes to offsets & scales,
	 *         2: corrections applied and offsets & scales updated
	 */
	int applyCorrectionsGyro(int topic_instance, math::Vector<3> &sensor_data, float temperature,
				   float *offsets, float *scales);

	int applyCorrectionsAccel(int topic_instance, math::Vector<3> &sensor_data, float temperature,
				    float *offsets, float *scales);

	int applyCorrectionsBaro(int topic_instance, float &sensor_data, float temperature,
				   float *offsets, float *scales);

	/** output current configuration status to console */
	void printStatus();
private:

	/* Struct containing parameters used by the single axis 5th order temperature compensation algorithm

	Input:

	measured_temp : temperature measured at the sensor (deg C)
	raw_value : reading from the sensor before compensation
	corrected_value : reading from the sensor after compensation for errors

	Compute:

	delta_temp = measured_temp - ref_temp
	offset = x5 * delta_temp^5 + x4 * delta_temp^4 + x3 * delta_temp^3 + x2 * delta_temp^2 + x1 * delta_temp + x0
	corrected_value = (raw_value - offset) * scale

	*/
	struct sensor_cal_data1_d {
		int32_t ID;
		float x5;
		float x4;
		float x3;
		float x2;
		float x1;
		float x0;
		float scale;
		float ref_temp;
		float min_temp;
		float max_temp;
	};

	struct sensor_cal_handles1_d {
		param_t ID;
		param_t x5;
		param_t x4;
		param_t x3;
		param_t x2;
		param_t x1;
		param_t x0;
		param_t scale;
		param_t ref_temp;
		param_t min_temp;
		param_t max_temp;
	};


	/* Struct containing parameters used by the 3-axis 3rd order temperature compensation algorithm

	Input:

	measured_temp : temperature measured at the sensor (deg C)
	raw_value[3] : XYZ readings from the sensor before compensation
	corrected_value[3] : XYZ readings from the sensor after compensation for errors

	Compute for each measurement index:

	delta_temp = measured_temp - ref_temp
	offset = x3 * delta_temp^3 + x2 * delta_temp^2 + x1 * delta_temp + x0
	corrected_value = (raw_value - offset) * scale

	 */
	struct sensor_cal_data3_d {
		int32_t ID;		/**< sensor device ID*/
		float x3[3];		/**< x^3 term of polynomial */
		float x2[3];		/**< x^2 term of polynomial */
		float x1[3];		/**< x^1 term of polynomial */
		float x0[3];		/**< x^0 / offset term of polynomial */
		float scale[3];		/**< scale factor correction */
		float ref_temp;		/**< reference temperature used by the curve-fit */
		float min_temp;		/**< minimum temperature with valid compensation data */
		float max_temp;		/**< maximum temperature with valid compensation data */
	};

	struct sensor_cal_handles3_d {
		param_t ID;
		param_t x3[3];
		param_t x2[3];
		param_t x1[3];
		param_t x0[3];
		param_t scale[3];
		param_t ref_temp;
		param_t min_temp;
		param_t max_temp;
	};

	// create a struct containing all thermal calibration parameters
	struct parameters {
		int32_t gyro_tc_enable;
		sensor_cal_data3_d gyro_cal_data[gyro_count_max];
		int32_t accel_tc_enable;
		sensor_cal_data3_d accel_cal_data[accel_count_max];
		int32_t baro_tc_enable;
		sensor_cal_data1_d baro_cal_data[baro_count_max];
	};

	// create a struct containing the handles required to access all calibration parameters
	struct parameter_handles {
		param_t gyro_tc_enable;
		sensor_cal_handles3_d gyro_cal_handles[gyro_count_max];
		param_t accel_tc_enable;
		sensor_cal_handles3_d accel_cal_handles[accel_count_max];
		param_t baro_tc_enable;
		sensor_cal_handles1_d baro_cal_handles[baro_count_max];
	};


	/**
	 * initialize ParameterHandles struct
	 * @return 0 on succes, <0 on error
	 */
	static int initializeParameterHandles(parameter_handles &parameter_handles);


	/**

	Calculate the offset required to compensate the sensor for temperature effects using a 5th order method
	If the measured temperature is outside the calibration range, clip the temperature to remain within the range and return false.
	If the measured temperature is within the calibration range, return true.

	Arguments:

	coef : reference to struct containing calibration coefficients
	measured_temp : temperature measured at the sensor (deg C)
	offset : reference to sensor offset

	Returns:

	Boolean true if the measured temperature is inside the valid range for the compensation

	*/
	bool calcThermalOffsets1D(sensor_cal_data1_d &coef, float measured_temp, float &offset);

	/**

	Calculate the offsets required to compensate the sensor for temperature effects
	If the measured temperature is outside the calibration range, clip the temperature to remain within the range and return false.
	If the measured temperature is within the calibration range, return true.

	Arguments:

	coef : reference to struct containing calibration coefficients
	measured_temp : temperature measured at the sensor (deg C)
	offset : reference to sensor offset - array of 3

	Returns:

	Boolean true if the measured temperature is inside the valid range for the compensation

	*/
	bool calcThermalOffsets3D(const SensorCalData3D &coef, float measured_temp, float offset[]);


	parameters _parameters;


	struct per_sensor_data {
		per_sensor_data()
		{
			for (int i = 0; i < sensor_count_max; ++i) { device_mapping[i] = 255; last_temperature[i] = -100.0f; }
		}
		void resetTemperature()
		{
			for (int i = 0; i < sensor_count_max; ++i) { last_temperature[i] = -100.0f; }
		}
		uint8_t device_mapping[sensor_count_max]; /// map a topic instance to the parameters index
		float last_temperature[sensor_count_max];
	};
	per_sensor_data _gyro_data;
	per_sensor_data _accel_data;
	per_sensor_data _baro_data;


	template<typename T>
	static inline int setSensorId(uint32_t device_id, int topic_instance, per_sensor_data &sensor_data,
					const T *sensor_cal_data, uint8_t sensor_count_max);
};

}
