#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t		req_land_init_count = 1;
static const uint32_t		land_timeout = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::landInit()
{
	// measure
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) {
		_landCount = 0;
	}

	// if finished
	if (_landCount > req_land_init_count) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land init");
		_sensorTimeout &= ~SENSOR_LAND;
		_sensorFault &= ~SENSOR_LAND;
	}
}

int BlockLocalPositionEstimator::landMeasure(Vector<float, n_y_land> &y)
{
	_time_last_land = _timeStamp;
	y.setZero();
	_landCount += 1;
	return OK;
}

void BlockLocalPositionEstimator::landCorrect()
{
	// measure land
	Vector<float, n_y_land> y;

	if (landMeasure(y) != OK) { return; }

	// measurement matrix
	Matrix<float, n_y_land, n_x> c;
	c.setZero();
	// y = -(z - tz)
	c(Y_land_vx, X_vx) = 1;
	c(Y_land_vy, X_vy) = 1;
	c(Y_land_agl, X_z) = -1;// measured altitude, negative down dir.
	c(Y_land_agl, X_tz) = 1;// measured altitude, negative down dir.

	// use parameter covariance
	SquareMatrix<float, n_y_land> r;
	r.setZero();
	r(Y_land_vx, Y_land_vx) = _land_vxy_stddev.get() * _land_vxy_stddev.get();
	r(Y_land_vy, Y_land_vy) = _land_vxy_stddev.get() * _land_vxy_stddev.get();
	r(Y_land_agl, Y_land_agl) = _land_z_stddev.get() * _land_z_stddev.get();

	// residual
	Matrix<float, n_y_land, n_y_land> s_i = inv<float, n_y_land>((c * _P * c.transpose()) + r);
	Vector<float, n_y_land> r = y - c * _x;
	_pub_innov.get().hagl_innov = r(Y_land_agl);
	_pub_innov.get().hagl_innov_var = r(Y_land_agl, Y_land_agl);

	// fault detection
	float beta = (r.transpose() * (s_i * r))(0, 0);

	// artifically increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	if (beta / beta_table[n_y_land] > beta_thresh) {
		if (!(_sensorFault & SENSOR_LAND)) {
			_sensorFault |= SENSOR_LAND;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LAND) {
		_sensorFault &= ~SENSOR_LAND;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land OK");
	}

	// kalman filter correction always for land detector
	Matrix<float, n_x, n_y_land> k = _P * c.transpose() * s_i;
	Vector<float, n_x> dx = k * r;
	_x += dx;
	_P -= k * c * _P;
}

void BlockLocalPositionEstimator::landCheckTimeout()
{
	if (_timeStamp - _time_last_land > land_timeout) {
		if (!(_sensorTimeout & SENSOR_LAND)) {
			_sensorTimeout |= SENSOR_LAND;
			_landCount = 0;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] land timeout ");
		}
	}
}
