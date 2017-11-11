#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int	req_sonar_init_count = 10;
static const uint32_t	sonar_timeout = 5000000;	// 2.0 s
static const float	sonar_max_init_std = 0.3f;	// meters

void BlockLocalPositionEstimator::sonarInit()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (_sonarStats.getCount() == 0) {
		_time_init_sonar = _timeStamp;
	}

	if (sonarMeasure(y) != OK) {
		return;
	}

	// if finished
	if (_sonarStats.getCount() > req_sonar_init_count) {
		if (_sonarStats.getStdDev()(0) > sonar_max_init_std) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init std > min");
			_sonarStats.reset();

		} else if ((_timeStamp - _time_init_sonar) > sonar_timeout) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init timeout ");
			_sonarStats.reset();

		} else {
			PX4_INFO("[lpe] sonar init "
				 "mean %d cm std %d cm",
				 int(100 * _sonarStats.getMean()(0)),
				 int(100 * _sonarStats.getStdDev()(0)));
			_sensorTimeout &= ~SENSOR_SONAR;
			_sensorFault &= ~SENSOR_SONAR;
		}
	}
}

int BlockLocalPositionEstimator::sonarMeasure(Vector<float, n_y_sonar> &y)
{
	// measure
	float d = _sub_sonar->get().current_distance;
	float eps = 0.01f;	// 1 cm
	float min_dist = _sub_sonar->get().min_distance + eps;
	float max_dist = _sub_sonar->get().max_distance - eps;

	// prevent driver from setting min dist below eps
	if (min_dist < eps) {
		min_dist = eps;
	}

	// check for bad data
	if (d > max_dist || d < min_dist) {
		return -1;
	}

	// update stats
	_sonarStats.update(Scalarf(d));
	_time_last_sonar = _timeStamp;
	y.setZero();
	y(0) = (d + _sonar_z_offset.get()) *
	       cosf(_eul(0)) *
	       cosf(_eul(1));
	return OK;
}

void BlockLocalPositionEstimator::sonarCorrect()
{
	// measure
	Vector<float, n_y_sonar> y;

	if (sonarMeasure(y) != OK) { return; }

	// do not use sonar if lidar is active
	//if (_lidarInitialized && (_lidarFault < fault_lvl_disable)) { return; }

	// calculate covariance
	float cov = _sub_sonar->get().covariance;

	if (cov < 1.0e-3f) {
		// use sensor value if reasoanble
		cov = _sonar_z_stddev.get() * _sonar_z_stddev.get();
	}

	// sonar measurement matrix and noise matrix
	Matrix<float, n_y_sonar, n_x> c;
	c.setZero();
	// y = -(z - tz)
	// TODO could add trig to make this an EKF correction
	c(Y_sonar_z, X_z) = -1;	// measured altitude, negative down dir.
	c(Y_sonar_z, X_tz) = 1;	// measured altitude, negative down dir.

	// covariance matrix
	SquareMatrix<float, n_y_sonar> r;
	r.setZero();
	r(0, 0) = cov;

	// residual
	Vector<float, n_y_sonar> r = y - c * _x;
	// residual covariance
	Matrix<float, n_y_sonar, n_y_sonar> s = c * _P * c.transpose() + r;

	// publish innovations
	_pub_innov.get().hagl_innov = r(0);
	_pub_innov.get().hagl_innov_var = s(0, 0);

	// residual covariance, (inverse)
	Matrix<float, n_y_sonar, n_y_sonar> s_i = inv<float, n_y_sonar>(s);

	// fault detection
	float beta = (r.transpose()  * (s_i * r))(0, 0);

	if (beta > beta_table[n_y_sonar]) {
		if (!(_sensorFault & SENSOR_SONAR)) {
			_sensorFault |= SENSOR_SONAR;
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar fault,  beta %5.2f", double(beta));
		}

		// abort correction
		return;

	} else if (_sensorFault & SENSOR_SONAR) {
		_sensorFault &= ~SENSOR_SONAR;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_SONAR)) {
		Matrix<float, n_x, n_y_sonar> k =
			_P * c.transpose() * s_i;
		Vector<float, n_x> dx = k * r;
		_x += dx;
		_P -= k * c * _P;
	}
}

void BlockLocalPositionEstimator::sonarCheckTimeout()
{
	if (_timeStamp - _time_last_sonar > sonar_timeout) {
		if (!(_sensorTimeout & SENSOR_SONAR)) {
			_sensorTimeout |= SENSOR_SONAR;
			_sonarStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar timeout ");
		}
	}
}
