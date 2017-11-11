#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		req_baro_init_count = 100;
static const uint32_t		baro_timeout = 100000;	// 0.1 s

void BlockLocalPositionEstimator::baroInit()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) {
		_baroStats.reset();
		return;
	}

	// if finished
	if (_baroStats.getCount() > req_baro_init_count) {
		_baroAltOrigin = _baroStats.getMean()(0);
		mavlink_and_console_log_info(&mavlink_log_pub,
					     "[lpe] baro init %d m std %d cm",
					     (int)_baroStats.getMean()(0),
					     (int)(100 * _baroStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_BARO;
		_sensorFault &= ~SENSOR_BARO;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = _baroAltOrigin;
		}
	}
}

int BlockLocalPositionEstimator::baroMeasure(Vector<float, n_y_baro> &y)
{
	//measure
	y.setZero();
	y(0) = _sub_sensor.get().baro_alt_meter;
	_baroStats.update(y);
	_time_last_baro = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::baroCorrect()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) { return; }

	// subtract baro origin alt
	y -= _baroAltOrigin;

	// baro measurement matrix
	Matrix<float, n_y_baro, n_x> c;
	c.setZero();
	c(Y_baro_z, X_z) = -1;	// measured altitude, negative down dir.

	Matrix<float, n_y_baro, n_y_baro> r;
	r.setZero();
	r(0, 0) = _baro_stddev.get() * _baro_stddev.get();

	// residual
	Matrix<float, n_y_baro, n_y_baro> s_i =
		inv<float, n_y_baro>((c * _P * c.transpose()) + r);
	Vector<float, n_y_baro> r = y - (c * _x);

	// fault detection
	float beta = (r.transpose() * (s_i * r))(0, 0);

	if (beta > beta_table[n_y_baro]) {
		if (!(_sensorFault & SENSOR_BARO)) {
			mavlink_log_critical(&mavlink_log_pub, "[lpe] baro fault, r %5.2f m, beta %5.2f",
					     double(r(0)), double(beta));
			_sensorFault |= SENSOR_BARO;
		}

	} else if (_sensorFault & SENSOR_BARO) {
		_sensorFault &= ~SENSOR_BARO;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_baro> k = _P * c.transpose() * s_i;
	Vector<float, n_x> dx = k * r;
	_x += dx;
	_P -= k * c * _P;
}

void BlockLocalPositionEstimator::baroCheckTimeout()
{
	if (_timeStamp - _time_last_baro > baro_timeout) {
		if (!(_sensorTimeout & SENSOR_BARO)) {
			_sensorTimeout |= SENSOR_BARO;
			_baroStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro timeout ");
		}
	}
}
