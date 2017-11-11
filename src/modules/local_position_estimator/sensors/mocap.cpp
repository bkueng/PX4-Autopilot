#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		req_mocap_init_count = 20;
static const uint32_t		mocap_timeout = 200000;	// 0.2 s

void BlockLocalPositionEstimator::mocapInit()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) {
		_mocapStats.reset();
		return;
	}

	// if finished
	if (_mocapStats.getCount() > req_mocap_init_count) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position init: "
					     "%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m",
					     double(_mocapStats.getMean()(0)),
					     double(_mocapStats.getMean()(1)),
					     double(_mocapStats.getMean()(2)),
					     double(_mocapStats.getStdDev()(0)),
					     double(_mocapStats.getStdDev()(1)),
					     double(_mocapStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_MOCAP;
		_sensorFault &= ~SENSOR_MOCAP;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
		}
	}
}

int BlockLocalPositionEstimator::mocapMeasure(Vector<float, n_y_mocap> &y)
{
	y.setZero();
	y(Y_mocap_x) = _sub_mocap.get().x;
	y(Y_mocap_y) = _sub_mocap.get().y;
	y(Y_mocap_z) = _sub_mocap.get().z;
	_mocapStats.update(y);
	_time_last_mocap = _sub_mocap.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::mocapCorrect()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) { return; }

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> c;
	c.setZero();
	c(Y_mocap_x, X_x) = 1;
	c(Y_mocap_y, X_y) = 1;
	c(Y_mocap_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> r;
	r.setZero();
	float mocap_p_var = _mocap_p_stddev.get() * \
			    _mocap_p_stddev.get();
	r(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	r(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	r(Y_mocap_z, Y_mocap_z) = mocap_p_var;

	// residual
	Vector<float, n_y_mocap> r = y - c * _x;
	// residual covariance
	Matrix<float, n_y_mocap, n_y_mocap> s = c * _P * c.transpose() + r;

	// publish innovations
	for (int i = 0; i < 3; i++) {
		_pub_innov.get().vel_pos_innov[i] = r(i);
		_pub_innov.get().vel_pos_innov_var[i] = s(i, i);
	}

	for (int i = 3; i < 6; i++) {
		_pub_innov.get().vel_pos_innov[i] = 0;
		_pub_innov.get().vel_pos_innov_var[i] = 1;
	}

	// residual covariance, (inverse)
	Matrix<float, n_y_mocap, n_y_mocap> s_i = inv<float, n_y_mocap>(s);

	// fault detection
	float beta = (r.transpose() * (s_i * r))(0, 0);

	if (beta > beta_table[n_y_mocap]) {
		if (!(_sensorFault & SENSOR_MOCAP)) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_MOCAP;
		}

	} else if (_sensorFault & SENSOR_MOCAP) {
		_sensorFault &= ~SENSOR_MOCAP;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_mocap> k = _P * c.transpose() * s_i;
	Vector<float, n_x> dx = k * r;
	_x += dx;
	_P -= k * c * _P;
}

void BlockLocalPositionEstimator::mocapCheckTimeout()
{
	if (_timeStamp - _time_last_mocap > mocap_timeout) {
		if (!(_sensorTimeout & SENSOR_MOCAP)) {
			_sensorTimeout |= SENSOR_MOCAP;
			_mocapStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap timeout ");
		}
	}
}
