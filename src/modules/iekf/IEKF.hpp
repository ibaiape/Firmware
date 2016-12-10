#include "ros/ros.hpp"
#include "matrix/math.hpp"
#include <lib/geo/geo.h>

#include "Origin.hpp"
#include "constants.hpp"

using namespace matrix;

/**
 * Main class for invariant extended kalman filter
 *
 * inspired by: https://hal.archives-ouvertes.fr/hal-00494342/document
 *
 * Also see python directory for simulation and python version.
 */
class IEKF
{
public:
	IEKF();
	Vector<float, X::n> dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u);
	bool ok() { return _nh.ok(); }
	void callbackImu(const sensor_combined_s *msg);
	void correctAccel(const sensor_combined_s *msg);
	void correctMag(const sensor_combined_s *msg);
	void correctBaro(const sensor_combined_s *msg);
	void correctGps(const vehicle_gps_position_s *msg);
	void correctAirspeed(const airspeed_s *msg);
	void predict(float dt);
	Vector<float, X::n> applyErrorCorrection(const Vector<float, Xe::n> &d_xe);
	void setP(const SquareMatrix<float, Xe::n> &P);
	void setX(const Vector<float, X::n> &x);
	void publish();

private:
	ros::NodeHandle _nh;

	// subscriptions
	ros::Subscriber _subImu;
	ros::Subscriber _subGps;
	ros::Subscriber _subAirspeed;

	// publishers
	ros::Publisher _pubAttitude;
	ros::Publisher _pubLocalPosition;
	ros::Publisher _pubGlobalPosition;
	ros::Publisher _pubControlState;
	ros::Publisher _pubEstimatorStatus;

	// data
	Vector<float, X::n> _x0; // initial state
	Vector<float, Xe::n> _P0Diag; // initial state diagonal
	Vector<float, X::n> _x; // state
	SquareMatrix<float, Xe::n> _P; // covariance
	Vector<float, U::n> _u;
	Vector3f _g_n;
	Vector3f _B_n;
	Origin _origin;
	uint64_t _timestampAccel;
	uint64_t _timestampMag;
	uint64_t _timestampBaro;
	uint64_t _timestampGps;
	uint64_t _timestampAirspeed;
};
