#include "IEKF.hpp"
#include "matrix/filter.hpp"

static const float mag_inclination = 1.0f;
static const float mag_declination = 0;

IEKF::IEKF() :
	_nh(), // node handle
	_subImu(_nh.subscribe("sensor_combined", 0, &IEKF::callbackImu, this)),
	_subGps(_nh.subscribe("vehicle_gps_position", 0, &IEKF::correctGps, this)),
	_subAirspeed(_nh.subscribe("airspeed", 0, &IEKF::correctAirspeed, this)),
	_pubAttitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pubLocalPosition(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pubGlobalPosition(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_pubControlState(_nh.advertise<control_state_s>("control_state", 0)),
	_pubEstimatorStatus(_nh.advertise<estimator_status_s>("estimator_status", 0)),
	_x0(),
	_P0Diag(),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -9.8),
	_B_n(),
	_origin(),
	_timestampAccel(),
	_timestampMag(),
	_timestampBaro(),
	_timestampGps()
{
	// initialize state
	_x0(X::q_nb_0) = 1;
	_x0(X::q_nb_1) = 0;
	_x0(X::q_nb_2) = 0;
	_x0(X::q_nb_3) = 0;
	_x0(X::accel_scale) = 1;
	_x = _x0;

	// initialize covariance
	_P0Diag(Xe::rot_N) = 1;
	_P0Diag(Xe::rot_E) = 1;
	_P0Diag(Xe::rot_D) = 3;
	_P0Diag(Xe::vel_N) = 1e9;
	_P0Diag(Xe::vel_E) = 1e9;
	_P0Diag(Xe::vel_D) = 1e9;
	_P0Diag(Xe::gyro_bias_N) = 1e-4;
	_P0Diag(Xe::gyro_bias_E) = 1e-4;
	_P0Diag(Xe::gyro_bias_D) = 1e-4;
	_P0Diag(Xe::accel_scale) = 1e-2;
	_P0Diag(Xe::pos_N) = 1e9;
	_P0Diag(Xe::pos_E) = 1e9;
	_P0Diag(Xe::pos_D) = 1e9;
	_P0Diag(Xe::terrain_alt) = 1e9;
	_P0Diag(Xe::baro_bias) = 1e9;
	_P0Diag(Xe::wind_N) = 0;
	_P0Diag(Xe::wind_E) = 0;
	_P0Diag(Xe::wind_D) = 0;
	_P = diag(_P0Diag);

	// initial magnetic field guess
	_B_n = Vector3f(0.21523, 0.00771, -0.42741);
}

Vector<float, X::n> IEKF::dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Quatf q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Vector3f as_n = a_n - _g_n;
	Vector3f gyro_bias_b(_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	Vector3f omega_nb_b(_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quatf dq_nb = q_nb * Quatf(0, omega_nb_b_corrected(0),
				   omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;
	//ROS_INFO("a_b: %10.4f %10.4f %10.4f\n", double(a_b(0)), double(a_b(1)), double(a_b(2)));
	//ROS_INFO("as_n: %10.4f %10.4f %10.4f\n", double(as_n(0)), double(as_n(1)), double(as_n(2)));

	Vector<float, X::n> dx;
	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);
	dx(X::vel_N) = as_n(0);
	dx(X::vel_E) = as_n(1);
	dx(X::vel_D) = as_n(2);
	dx(X::gyro_bias_bX) = 0;
	dx(X::gyro_bias_bY) = 0;
	dx(X::gyro_bias_bZ) = 0;
	dx(X::accel_scale) = 0;
	dx(X::pos_N) = x(X::vel_N);
	dx(X::pos_E) = x(X::vel_E);
	dx(X::pos_D) = x(X::vel_D);
	dx(X::terrain_alt) = 0;
	dx(X::baro_bias) = 0;
	dx(X::wind_N) = 0;
	dx(X::wind_E) = 0;
	dx(X::wind_D) = 0;
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//ROS_INFO("imu callback");
	_u(U::omega_nb_bX) = msg->gyro_rad[0];
	_u(U::omega_nb_bY) = msg->gyro_rad[1];
	_u(U::omega_nb_bZ) = msg->gyro_rad[2];
	_u(U::accel_bX) = msg->accelerometer_m_s2[0];
	_u(U::accel_bY) = msg->accelerometer_m_s2[1];
	_u(U::accel_bZ) = msg->accelerometer_m_s2[2];

	// predict driven by gyro callback
	if (msg->gyro_integral_dt > 0) {
		predict(msg->gyro_integral_dt);
	};

	// correct  if new data
	correctAccel(msg);

	correctMag(msg);

	correctBaro(msg);

	publish();
}

void IEKF::correctAccel(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampAccelNew = msg->timestamp + msg->accelerometer_timestamp_relative;

	if (timestampAccelNew != _timestampAccel) {
		dt = (timestampAccelNew - _timestampAccel) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampAccel = timestampAccelNew;

	} else {
		return;
	}

	// measurement
	Vector3f y_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	// don't correct if accelerating
	float relNormError = (Vector3f(y_b / _x(X::accel_scale)).norm()
			      - _g_n.norm()) / _g_n.norm();

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_g_n = q_nb.conjugate(y_b / _x(X::accel_scale));
	Vector3f r = y_g_n - _g_n;

	// define R
	// worst accel dir change is if accel is normal to gravity,
	// assume this and calculate angle covariance based on accel norm error
	Matrix<float, Y_accel::n, Y_accel::n> R;
	R(Y_accel::accel_bX, Y_accel::accel_bX) = 1e-2f / dt + relNormError * relNormError;
	R(Y_accel::accel_bY, Y_accel::accel_bY) = 1e-2f / dt + relNormError * relNormError;
	R(Y_accel::accel_bZ, Y_accel::accel_bZ) = 1e-2f / dt + relNormError * relNormError;

	// define H
	Matrix<float, Y_accel::n, Xe::n> H;
	Matrix3f tmp = _g_n.unit().hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_accel::accel_bX + i, Xe::rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_accel::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_accel::n]) {
		ROS_DEBUG("accel fault");
		return;
	}

	//ROS_INFO("accel correction");
	//dxe.print();

	// don't allow yaw correction
	//dxe(Xe::rot_D) = 0;

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Quatf q_nb2(x(X::q_nb_0), x(X::q_nb_1),
		    x(X::q_nb_2), x(X::q_nb_3));
	Vector3f r2 = q_nb2.conjugate(y_b / x(X::accel_scale)) - _g_n;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_INFO("accel linearization error!!!!!!!!!!!!!!!!!!!!");
		Vector3f rot(dxe(Xe::rot_N), dxe(Xe::rot_E), dxe(Xe::rot_D));
		float angle = rot.norm();
		float angle_max = 0.1f * acosf(y_g_n.dot(_g_n)) / y_g_n.norm() / _g_n.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_g_n.cross(_g_n).unit();
		dxe.setZero();
		dxe(Xe::rot_N) = axis(0) * angle;
		dxe(Xe::rot_E) = axis(1) * angle;
		dxe(Xe::rot_D) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);

	} else {
		setX(x);
	}

	setP(_P + dP);
}

void IEKF::correctMag(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampMagNew = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (timestampMagNew != _timestampMag) {
		dt = (timestampMagNew - _timestampMag) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampMag = timestampMagNew;

	} else {
		return;
	}

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_b = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();
	Vector3f yh = _B_n.unit();
	Vector3f y = q_nb.conjugate(y_b);
	Vector3f r = y - yh;

	// define R
	Matrix<float, Y_mag::n, Y_mag::n> R;
	R(Y_mag::mag_N, Y_mag::mag_N) = 2e-3f / dt;
	R(Y_mag::mag_E, Y_mag::mag_E) = 2e-3f / dt;
	R(Y_mag::mag_D, Y_mag::mag_D) = 20e-3f / dt; // less certain about local inclination

	// define H
	Matrix<float, Y_mag::n, Xe::n> H;
	Matrix3f tmp = yh.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_mag::mag_N + i, Xe::rot_N + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_mag::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_mag::n]) {
		ROS_DEBUG("mag fault");
	}

	//ROS_INFO("mag correction");
	//dxe.print();

	// don't allow roll/ pitch correction
	//dxe(Xe::rot_N) = 0;
	//dxe(Xe::rot_E) = 0;

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Vector3f r2 = Quatf(x(X::q_nb_0), x(X::q_nb_1),
			    x(X::q_nb_2), x(X::q_nb_3)).conjugate(y_b) - yh;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_INFO("mag linearization error!!!!!!!!!!!!!!!!!!!!");
		Vector3f rot(dxe(Xe::rot_N), dxe(Xe::rot_E), dxe(Xe::rot_D));
		Vector3f y_xy = Vector3f(y(0), y(1), 0);
		Vector3f yh_xy = Vector3f(yh(0), yh(1), 0);
		float angle = rot.norm();
		float angle_max = 0.1f * acosf(y_xy.dot(yh_xy)) / y_xy.norm() / yh_xy.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_xy.cross(yh_xy).unit();
		dxe.setZero();
		dxe(Xe::rot_N) = axis(0) * angle;
		dxe(Xe::rot_E) = axis(1) * angle;
		dxe(Xe::rot_D) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);

	} else {
		setX(x);
	}

	setP(_P + dP);
}

void IEKF::correctBaro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampBaroNew = msg->timestamp + msg->baro_timestamp_relative;

	if (timestampBaroNew != _timestampBaro) {
		dt = (timestampBaroNew - _timestampBaro) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampBaro = timestampBaroNew;

	} else {
		return;
	}

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->baro_alt_meter;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= -_x(X::pos_D) + _x(X::baro_bias) - _origin.getAlt();
	Vector<float, Y_baro::n> r = y - yh;

	// define R
	Matrix<float, Y_baro::n, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = 3e-3f / dt;

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::pos_D) = -1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_baro::n>(_P, H, R, r, dxe, dP, beta);

	//ROS_INFO("baro residual");
	//r.print();

	if (beta > BETA_TABLE[Y_baro::n]) {
		ROS_DEBUG("baro fault");
	}

	//ROS_INFO("baro correction");
	//dxe.print();

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
	//dxe(Xe::rot_N) = 0;
	//dxe(Xe::rot_E) = 0;
	//dxe(Xe::rot_D) = 0;
	//dxe(Xe::gyro_bias_N) = 0;
	//dxe(Xe::gyro_bias_E) = 0;
	//dxe(Xe::gyro_bias_D) = 0;
}

void IEKF::correctGps(const vehicle_gps_position_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampGpsNew = msg->timestamp;

	if (timestampGpsNew == _timestampGps) {
		return;
	}

	dt = (timestampGpsNew - _timestampGps) / 1.0e6f;

	if (dt < 0) {
		return;
	}

	_timestampGps = timestampGpsNew;


	// check for good gps signal
	if (msg->satellites_used < 6 || msg->fix_type < 3) {
		return;
	}

	_timestampGps = msg->timestamp;
	double lat_deg = msg->lat * 1e-7;
	double lon_deg = msg->lon * 1e-7;
	float alt_m = msg->alt * 1e-3;
	float vel_stddev = sqrt(
				   _P(Xe::vel_N, Xe::vel_N)
				   + _P(Xe::vel_E, Xe::vel_E)
				   + _P(Xe::vel_D, Xe::vel_D));

	// init global reference
	if (!_origin.xyInitialized() && vel_stddev < 1.0f) {
		ROS_INFO("gps map ref init %12.6f %12.6f", double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, msg->timestamp);
	}

	if (!_origin.altInitialized() && sqrtf(_P(Xe::pos_D, Xe::pos_D)) < 0.1f) {
		ROS_INFO("gps alt init %12.2f", double(alt_m));
		_origin.altInitialize(alt_m, msg->timestamp);
	}

	// calculate residual
	float gps_pos_N = 0;
	float gps_pos_E = 0;
	float gps_pos_D = 0;
	_origin.globalToLocal(lat_deg, lon_deg, alt_m,
			      gps_pos_N, gps_pos_E, gps_pos_D);

	Vector<float, Y_gps::n> y;
	y(Y_gps::pos_N) = gps_pos_N;
	y(Y_gps::pos_E) = gps_pos_E;
	y(Y_gps::pos_D) = gps_pos_D;
	y(Y_gps::vel_N) = msg->vel_n_m_s;
	y(Y_gps::vel_E) = msg->vel_e_m_s;
	y(Y_gps::vel_D) = msg->vel_d_m_s;

	Vector<float, Y_gps::n> yh;
	yh(Y_gps::pos_N) = _x(X::pos_N);
	yh(Y_gps::pos_E) = _x(X::pos_E);
	yh(Y_gps::pos_D) = _x(X::pos_D);
	yh(Y_gps::vel_N) = _x(X::vel_N);
	yh(Y_gps::vel_E) = _x(X::vel_E);
	yh(Y_gps::vel_D) = _x(X::vel_D);

	Vector<float, Y_gps::n> r = y - yh;

	// define R
	Matrix<float, Y_gps::n, Y_gps::n> R;
	R(Y_gps::pos_N, Y_gps::pos_N) = 0.2f / dt;
	R(Y_gps::pos_E, Y_gps::pos_E) = 0.2f / dt;
	R(Y_gps::pos_D, Y_gps::pos_D) = 0.2f / dt;
	R(Y_gps::vel_N, Y_gps::vel_N) = 0.2f / dt;
	R(Y_gps::vel_E, Y_gps::vel_E) = 0.2f / dt;
	R(Y_gps::vel_D, Y_gps::vel_D) = 0.2f / dt;

	// define H
	Matrix<float, Y_gps::n, Xe::n> H;
	H(Y_gps::pos_N, Xe::pos_N) = 1;
	H(Y_gps::pos_E, Xe::pos_E) = 1;
	H(Y_gps::pos_D, Xe::pos_D) = 1;
	H(Y_gps::vel_N, Xe::vel_N) = 1;
	H(Y_gps::vel_E, Xe::vel_E) = 1;
	H(Y_gps::vel_D, Xe::vel_D) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_gps::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_gps::n]) {
		ROS_DEBUG("gps fault");
	}

	//ROS_INFO("gps correction");
	//dxe.print();

	//ROS_INFO("gps rot correct %10.4f %10.4f %10.4f",
	//double(dxe(Xe::rot_N)),
	//double(dxe(Xe::rot_E)),
	//double(dxe(Xe::rot_D)));
	//dxe(Xe::rot_N) = 0;
	//dxe(Xe::rot_E) = 0;
	//dxe(Xe::rot_D) = 0;
	//dxe(Xe::gyro_bias_N) = 0;
	//dxe(Xe::gyro_bias_E) = 0;
	//dxe(Xe::gyro_bias_D) = 0;
	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}


void IEKF::correctAirspeed(const airspeed_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampAirspeedNew = msg->timestamp;

	if (timestampAirspeedNew == _timestampAirspeed) {
		return;
	}

	dt = (timestampAirspeedNew - _timestampAirspeed) / 1.0e6f;

	if (dt < 0) {
		return;
	}

	//ROS_INFO("airspeed dt %10.4f", double(dt));

	_timestampAirspeed = timestampAirspeedNew;

	// attitude info
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Dcmf C_nb = q_nb;

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float yh = -wind_rel_b(0); // body -x component aligned with pitot tube

	// measured airspeed
	float y = msg->true_airspeed_unfiltered_m_s;

	Vector<float, 1> r;
	r(0) = y - yh;
	//ROS_INFO("airspeed y: %10.4f, yh: %10.4f, residual %10.4f",
	//double(y), double(yh), double(r(0)));
	//ROS_INFO("wind_n");
	//wind_n.print();

	//ROS_INFO("vel_n");
	//vel_n.print();

	// define R
	Matrix<float, Y_airspeed::n, Y_airspeed::n> R;
	R(Y_airspeed::airspeed, Y_airspeed::airspeed) = 100.0f / dt;

	// define H
	// TODO make this invariant
	Matrix<float, Y_airspeed::n, Xe::n> H;
	H(Y_airspeed::airspeed, Xe::vel_N) = C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::vel_E) = C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::vel_D) = C_nb(2, 0);
	H(Y_airspeed::airspeed, Xe::wind_N) = -C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::wind_E) = -C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::wind_D) = -C_nb(2, 0);

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_airspeed::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_airspeed::n]) {
		ROS_DEBUG("airspeed fault");
	}

	//ROS_INFO("airspeed correction");
	//dxe.print();

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}

void IEKF::predict(float dt)
{
	// define process noise matrix
	Matrix<float, Xe::n, Xe::n> Q;
	Q(Xe::rot_N, Xe::rot_N) = 1e-6;
	Q(Xe::rot_E, Xe::rot_E) = 1e-6;
	Q(Xe::rot_D, Xe::rot_D) = 1e-6;
	Q(Xe::vel_N, Xe::vel_N) = 1e-3;
	Q(Xe::vel_E, Xe::vel_E) = 1e-3;
	Q(Xe::vel_D, Xe::vel_D) = 1e-3;
	Q(Xe::gyro_bias_N, Xe::gyro_bias_N) = 1e-6;
	Q(Xe::gyro_bias_E, Xe::gyro_bias_E) = 1e-6;
	Q(Xe::gyro_bias_D, Xe::gyro_bias_D) = 1e-6;
	Q(Xe::accel_scale, Xe::accel_scale) = 1e-6;
	Q(Xe::pos_N, Xe::pos_N) = 1e-6;
	Q(Xe::pos_E, Xe::pos_E) = 1e-6;
	Q(Xe::pos_D, Xe::pos_D) = 1e-6;
	Q(Xe::terrain_alt, Xe::terrain_alt) = 1e-1f;
	Q(Xe::baro_bias, Xe::baro_bias) = 1e-3f;
	Q(Xe::wind_N, Xe::wind_N) = 1e-1f;
	Q(Xe::wind_E, Xe::wind_E) = 1e-1f;
	Q(Xe::wind_D, Xe::wind_D) = 1e-4f;

	// define A matrix
	Matrix<float, Xe::n, Xe::n> A;

	// derivative of rotation error is -0.5 * gyro bias
	A(Xe::rot_N, Xe::Xe::gyro_bias_N) = -0.5;
	A(Xe::rot_E, Xe::Xe::gyro_bias_E) = -0.5;
	A(Xe::rot_D, Xe::Xe::gyro_bias_D) = -0.5;

	// derivative of velocity
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		ROS_DEBUG("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		q_nb.normalize();
		_x(X::q_nb_0) = q_nb(0);
		_x(X::q_nb_1) = q_nb(1);
		_x(X::q_nb_2) = q_nb(2);
		_x(X::q_nb_3) = q_nb(3);
	}

	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f J_a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::vel_N + i, Xe::rot_N + j) = a_tmp(i, j);
		}

		A(Xe::vel_N + i, Xe::accel_scale) = -J_a_n(i);
	}

	// derivative of gyro bias
	Vector3f omega_nb_b(
		_u(U::omega_nb_bX), _u(U::omega_nb_bY), _u(U::omega_nb_bZ));
	Vector3f gyro_bias_b(
		_x(X::gyro_bias_bX), _x(X::gyro_bias_bY), _x(X::gyro_bias_bZ));
	Vector3f J_omega_n = q_nb.conjugate(omega_nb_b - gyro_bias_b);
	Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::gyro_bias_N + i, Xe::rot_N + j) = g_tmp(i, j);
		}
	}

	// derivative of position is velocity
	A(Xe::pos_N, Xe::vel_N) = 1;
	A(Xe::pos_E, Xe::vel_E) = 1;
	A(Xe::pos_D, Xe::vel_D) = 1;

	// derivative of terrain alt is zero

	// derivative of baro bias is zero

	//ROS_INFO("A:");
	//for (int i=0;i<Xe::n; i++) {
	//for (int j=0;j<Xe::n; j++) {
	//printf("%10.3f, ", double(A(i, j)));
	//}
	//printf("\n");
	//}

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = dt;
	Vector<float, X::n> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, X::n> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//ROS_INFO("dx predict \n");
	//dx.print();
	setX(_x + dx);

	// propgate covariance using euler integration
	Matrix<float, Xe::n, Xe::n> dP = (A * _P + _P * A.T() + Q) * dt;
	setP(_P + dP);

	//ROS_INFO("P:");
	//_P.print();
}

Vector<float, X::n> IEKF::applyErrorCorrection(const Vector<float, Xe::n> &d_xe)
{
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1), _x(X::q_nb_2), _x(X::q_nb_3));
	Quatf d_q_nb = Quatf(0,
			     d_xe(Xe::rot_N), d_xe(Xe::rot_E), d_xe(Xe::rot_D)) * q_nb;
	//ROS_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3f d_gyro_bias_b = q_nb.conjugate_inversed(
					 Vector3f(d_xe(Xe::gyro_bias_N),
							 d_xe(Xe::gyro_bias_E),
							 d_xe(Xe::gyro_bias_D)));

	// linear term correction is the same
	// as the error correction
	Vector<float, X::n> x = _x;
	x(X::q_nb_0) += d_q_nb(0);
	x(X::q_nb_1) += d_q_nb(1);
	x(X::q_nb_2) += d_q_nb(2);
	x(X::q_nb_3) += d_q_nb(3);
	x(X::vel_N) += d_xe(Xe::vel_N);
	x(X::vel_E) += d_xe(Xe::vel_E);
	x(X::vel_D) += d_xe(Xe::vel_D);
	x(X::gyro_bias_bX) += d_gyro_bias_b(0);
	x(X::gyro_bias_bY) += d_gyro_bias_b(1);
	x(X::gyro_bias_bZ) +=  d_gyro_bias_b(2);
	x(X::accel_scale) += _x(X::accel_scale) * d_xe(Xe::accel_scale);
	x(X::pos_N) += d_xe(Xe::pos_N);
	x(X::pos_E) += d_xe(Xe::pos_E);
	x(X::pos_D) += d_xe(Xe::pos_D);
	x(X::terrain_alt) += d_xe(Xe::terrain_alt);
	x(X::baro_bias) += d_xe(Xe::baro_bias);
	x(X::wind_N) += d_xe(Xe::wind_N);
	x(X::wind_E) += d_xe(Xe::wind_E);
	x(X::wind_D) += d_xe(Xe::wind_D);
	return x;
}

void IEKF::setP(const SquareMatrix<float, Xe::n> &P)
{
	_P = P;

	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				ROS_DEBUG("P(%d, %d) NaN, resetting", i, j);

				if (i == j) {
					_P(i, j) = _P0Diag(i);

				} else {
					_P(i, j) = 0;
				}
			}

			if (_P(i, j) > 1e6f) {
				// upper bound
				_P(i, j) = 1e6f;
			}
		}

		// force positive diagonal
		if (_P(i, i) < 1e-6f) {
			ROS_DEBUG("P(%d, %d) < 1e-6, setting to 1e-6", i, i);
			_P(i, i) = 1e-6f;
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::setX(const Vector<float, X::n> &x)
{
	// set private state
	_x = x;

	// for quaterinons we bound at 2
	// so that saturation doesn't change
	// the direction of the vectors typicall
	// and normalization
	// handles small errors
	Vector<float, X::n> lowerBound;
	lowerBound(X::q_nb_0) = -2;
	lowerBound(X::q_nb_1) = -2;
	lowerBound(X::q_nb_2) = -2;
	lowerBound(X::q_nb_3) = -2;
	lowerBound(X::vel_N) = -100;
	lowerBound(X::vel_E) = -100;
	lowerBound(X::vel_D) = -100;
	lowerBound(X::gyro_bias_bX) = 0;
	lowerBound(X::gyro_bias_bY) = 0;
	lowerBound(X::gyro_bias_bZ) = 0;
	lowerBound(X::accel_scale) = 0.8;
	lowerBound(X::pos_N) = -1e9;
	lowerBound(X::pos_E) = -1e9;
	lowerBound(X::pos_D) = -1e9;
	lowerBound(X::terrain_alt) = -1e6;
	lowerBound(X::baro_bias) = -1e6;
	lowerBound(X::wind_N) = -100;
	lowerBound(X::wind_E) = -100;
	lowerBound(X::wind_D) = -100;

	Vector<float, X::n> upperBound;
	upperBound(X::q_nb_0) = 2;
	upperBound(X::q_nb_1) = 2;
	upperBound(X::q_nb_2) = 2;
	upperBound(X::q_nb_3) = 2;
	upperBound(X::vel_N) = 100;
	upperBound(X::vel_E) = 100;
	upperBound(X::vel_D) = 100;
	upperBound(X::gyro_bias_bX) = 0;
	upperBound(X::gyro_bias_bY) = 0;
	upperBound(X::gyro_bias_bZ) = 0;
	upperBound(X::accel_scale) = 1.5;
	upperBound(X::pos_N) = 1e9;
	upperBound(X::pos_E) = 1e9;
	upperBound(X::pos_D) = 1e9;
	upperBound(X::terrain_alt) = 1e6;
	upperBound(X::baro_bias) = 1e6;
	upperBound(X::wind_N) = 100;
	upperBound(X::wind_E) = 100;
	upperBound(X::wind_D) = 100;

	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			ROS_WARN("x(%d) NaN, setting to %10.4f", i, double(_x0(i)));
			_x(i) = _x0(i);
		}

		if (_x(i) < lowerBound(i)) {
			//ROS_INFO("x(%d) < lower bound, saturating", i);
			_x(i) = lowerBound(i);

		} else if (_x(i) > upperBound(i)) {
			//ROS_INFO("x(%d) > upper bound, saturating", i);
			_x(i) = upperBound(i);
		}
	}
}

void IEKF::publish()
{
	//ROS_INFO("x:");
	//_x.print();

	//ROS_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_N, Xe::pos_N) + _P(Xe::pos_E, Xe::pos_E));
	float epv = _P(Xe::pos_D, Xe::pos_D);
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Euler<float> euler_nb = q_nb;
	Vector3f a_b(_u(U::accel_bX), _u(U::accel_bY), _u(U::accel_bZ));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	ros::Time now = ros::Time::now();

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_N), _x(X::wind_E), _x(X::wind_D));
	Vector3f vel_n(_x(X::vel_N), _x(X::vel_E), _x(X::vel_D));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float airspeed = -wind_rel_b(0); // body -x component aligned with pitot tube

	bool attitudeValid = sqrtf(_P(Xe::rot_N, Xe::rot_N)
				   + _P(Xe::rot_E, Xe::rot_E)
				   + _P(Xe::rot_D, Xe::rot_D)) < 0.1f;

	bool velocityValid = sqrtf(_P(Xe::vel_N, Xe::vel_N)
				   + _P(Xe::vel_E, Xe::vel_E)
				   + _P(Xe::vel_D, Xe::vel_D)) < 1.0f;

	bool positionValid = sqrtf(_P(Xe::pos_N, Xe::pos_N)
				   + _P(Xe::pos_E, Xe::pos_E)
				   + _P(Xe::pos_D, Xe::pos_D)) < 1.0f;

	// publish attitude
	if (attitudeValid) {
		vehicle_attitude_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.rollspeed = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitchspeed = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yawspeed = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		_pubAttitude.publish(msg);
	}

	// publish local position
	if (attitudeValid && velocityValid) {
		vehicle_local_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.xy_valid = positionValid;
		msg.z_valid = positionValid;
		msg.v_xy_valid = velocityValid;
		msg.v_z_valid = velocityValid;
		msg.x = _x(X::pos_N);
		msg.y = _x(X::pos_E);
		msg.z = _x(X::pos_D);
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_N);
		msg.vy = _x(X::vel_E);
		msg.vz = _x(X::vel_D);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _origin.getAlt();
		msg.dist_bottom = -_x(X::pos_D) - _x(X::terrain_alt);
		msg.dist_bottom_rate = -_x(X::vel_D);
		msg.surface_bottom_timestamp = 0;
		msg.dist_bottom_valid = true;
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.publish(msg);
	}

	// publish global position
	if (_origin.xyInitialized() && _origin.altInitialized() && positionValid && velocityValid) {
		double lat_deg = 0;
		double lon_deg = 0;
		float alt_m = 0;
		_origin.localToGlobal(_x(X::pos_N), _x(X::pos_E), _x(X::pos_D), lat_deg, lon_deg, alt_m);
		//ROS_INFO("alt %10.4f m", double(alt_m));
		vehicle_global_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.time_utc_usec = 0; // TODO
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = alt_m;
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_N);
		msg.vel_e = _x(X::vel_E);
		msg.vel_d = _x(X::vel_D);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_alt) + _origin.getAlt();
		msg.terrain_alt_valid = true;
		msg.dead_reckoning = false;
		msg.pressure_alt = alt_m; // TODO
		_pubGlobalPosition.publish(msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.x_acc = a_n(0);
		msg.y_acc = a_n(1);
		msg.z_acc = a_n(2);
		msg.x_vel = _x(X::vel_N);
		msg.y_vel = _x(X::vel_E);
		msg.z_vel = _x(X::vel_D);
		msg.x_pos = _x(X::pos_N);
		msg.y_pos = _x(X::pos_E);
		msg.z_pos = _x(X::pos_D);
		msg.airspeed = airspeed;
		msg.airspeed_valid = true;
		msg.vel_variance[0] = _P(Xe::vel_N, Xe::vel_N);
		msg.vel_variance[1] = _P(Xe::vel_E, Xe::vel_E);
		msg.vel_variance[2] = _P(Xe::vel_D, Xe::vel_D);
		msg.pos_variance[0] = _P(Xe::pos_N, Xe::pos_N);
		msg.pos_variance[1] = _P(Xe::pos_E, Xe::pos_E);
		msg.pos_variance[2] = _P(Xe::pos_D, Xe::pos_D);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bX) - _x(X::gyro_bias_bX);
		msg.pitch_rate = _u(U::omega_nb_bY) - _x(X::gyro_bias_bY);
		msg.yaw_rate = _u(U::omega_nb_bZ) - _x(X::gyro_bias_bZ);
		msg.horz_acc_mag = 0;
		_pubControlState.publish(msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now.toNSec();
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
			// offset by 1 so covariances lined up with states
			// except for quaternoin and rotations error
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = 0; // TODO
		msg.vel_test_ratio = 0; // TODO
		msg.pos_test_ratio = 0; // TODO
		msg.hgt_test_ratio = 0; // TODO
		msg.tas_test_ratio = 0; // TODO
		msg.hagl_test_ratio = 0; // TODO
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.publish(msg);
	}
}
