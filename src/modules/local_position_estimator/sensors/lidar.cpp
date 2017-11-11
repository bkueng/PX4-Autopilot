#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//
static const uint32_t		req_lidar_init_count = 10;
static const uint32_t		lidar_timeout = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::lidarInit()
{
	// measure
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) {
		_lidarStats.reset();
	}

	// if finished
	if (_lidarStats.getCount() > req_lidar_init_count) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar init: "
					     "mean %d cm stddev %d cm",
					     int(100 * _lidarStats.getMean()(0)),
					     int(100 * _lidarStats.getStdDev()(0)));
		_sensorTimeout &= ~SENSOR_LIDAR;
		_sensorFault &= ~SENSOR_LIDAR;
	}
}

int BlockLocalPositionEstimator::lidarMeasure(Vector<float, n_y_lidar> &y)
{
	// measure
	float d = _sub_lidar->get().current_distance;
	float eps = 0.01f;	// 1 cm
	float min_dist = _sub_lidar->get().min_distance + eps;
	float max_dist = _sub_lidar->get().max_distance - eps;

	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_lidarStats.update(Scalarf(d));
	_time_last_lidar = _timeStamp;
	y.setZero();
	y(0) = (d + _lidar_z_offset.get()) *
	       cosf(_eul(0)) *
	       cosf(_eul(1));
	return OK;
}

void BlockLocalPositionEstimator::lidarCorrect()
{
	// measure lidar
	Vector<float, n_y_lidar> y;

	if (lidarMeasure(y) != OK) { return; }

	// measurement matrix
	Matrix<float, n_y_lidar, n_x> c;
	c.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	c(Y_lidar_z, X_z) = -1;	// measured altitude, negative down dir.
	c(Y_lidar_z, X_tz) = 1;	// measured altitude, negative down dir.

	// use parameter covariance unless sensor provides reasonable value
	SquareMatrix<float, n_y_lidar> r;
	r.setZero();
	float cov = _sub_lidar->get().covariance;

	if (cov < 1.0e-3f) {
		r(0, 0) = _lidar_z_stddev.get() * _lidar_z_stddev.get();

	} else {
		r(0, 0) = cov;
	}

	// residual
	Vector<float, n_y_lidar> r = y - c * _x;
	// residual covariance
	Matrix<float, n_y_lidar, n_y_lidar> s = c * _P * c.transpose() + r;

	// publish innovations
	_pub_innov.get().hagl_innov = r(0);
	_pub_innov.get().hagl_innov_var = s(0, 0);

	// residual covariance, (inverse)
	Matrix<float, n_y_lidar, n_y_lidar> s_i = inv<float, n_y_lidar>(s);

	// fault detection
	float beta = (r.transpose() * (s_i * r))(0, 0);

	if (beta > beta_table[n_y_lidar]) {
		if (!(_sensorFault & SENSOR_LIDAR)) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar fault,  beta %5.2f", double(beta));
			_sensorFault |= SENSOR_LIDAR;
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_LIDAR) {
		_sensorFault &= ~SENSOR_LIDAR;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_lidar> k = _P * c.transpose() * s_i;
	Vector<float, n_x> dx = k * r;
	_x += dx;
	_P -= k * c * _P;
}

void BlockLocalPositionEstimator::lidarCheckTimeout()
{
	if (_timeStamp - _time_last_lidar > lidar_timeout) {
		if (!(_sensorTimeout & SENSOR_LIDAR)) {
			_sensorTimeout |= SENSOR_LIDAR;
			_lidarStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] lidar timeout ");
		}
	}
}
