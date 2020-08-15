#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/LU>

class AttitudeEstimationWallsEKF{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_inipose;
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_bias;
		ros::Subscriber _sub_lidar_g;
		ros::Subscriber _sub_camera_g;
		/*publisher*/
		ros::Publisher _pub_quat_rp;
		ros::Publisher _pub_quat_rpy;
		/*state*/
		Eigen::Vector2d _x;
		Eigen::Matrix2d _P;
		/*time*/
		ros::Time _stamp_imu_last;
		/*bias*/
		sensor_msgs::Imu _bias;
		/*yaw*/
		double _yaw = 0.0;
		/*flag*/
		bool _got_inipose = false;
		bool _got_first_imu = false;
		bool _got_bias = false;
		/*parameter*/
		bool _wait_inipose;
		std::string _frame_id;
		double _sigma_ini;
		double _sigma_imu;
		double _sigma_lidar_g;
		double _sigma_camera_g;

	public:
		AttitudeEstimationWallsEKF();
		void initializeState(void);
		void callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackBias(const sensor_msgs::ImuConstPtr& msg);
		void callbackLidarG(const geometry_msgs::Vector3StampedConstPtr& msg);
		void callbackCameraG(const geometry_msgs::Vector3StampedConstPtr& msg);
		void predictionIMU(sensor_msgs::Imu imu, double dt);
		void observationG(geometry_msgs::Vector3Stamped g, double sigma);
		void publication(ros::Time stamp);
		void estimateYaw(sensor_msgs::Imu imu, double dt);
		void getRotMatrixRP(double r, double p, Eigen::MatrixXd& Rot);
		void getRotMatrixY(double r, double p, Eigen::MatrixXd& Rot);
		void anglePiToPi(double& angle);
};

AttitudeEstimationWallsEKF::AttitudeEstimationWallsEKF()
	: _nhPrivate("~")
{
	std::cout << "--- attitude_estimation_walls_ekf ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("wait_inipose", _wait_inipose, true);
	std::cout << "_wait_inipose = " << (bool)_wait_inipose << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/base_link"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("sigma_ini", _sigma_ini, 1.0e-10);
	std::cout << "_sigma_ini = " << _sigma_ini << std::endl;
	_nhPrivate.param("sigma_imu", _sigma_imu, 1.0e-4);
	std::cout << "_sigma_imu = " << _sigma_imu << std::endl;
	_nhPrivate.param("sigma_lidar_g", _sigma_lidar_g, 1.0e-1);
	std::cout << "_sigma_lidar_g = " << _sigma_lidar_g << std::endl;
	_nhPrivate.param("sigma_camera_g", _sigma_camera_g, 1.0e-1);
	std::cout << "_sigma_camera_g = " << _sigma_camera_g << std::endl;
	/*subscriber*/
	_sub_inipose = _nh.subscribe("/initial_orientation", 1, &AttitudeEstimationWallsEKF::callbackIniPose, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &AttitudeEstimationWallsEKF::callbackIMU, this);
	_sub_bias = _nh.subscribe("/imu/bias", 1, &AttitudeEstimationWallsEKF::callbackBias, this);
	_sub_lidar_g = _nh.subscribe("/lidar/g_vector", 1, &AttitudeEstimationWallsEKF::callbackLidarG, this);
	_sub_camera_g = _nh.subscribe("/dnn/g_vector", 1, &AttitudeEstimationWallsEKF::callbackCameraG, this);
	/*publisher*/
	_pub_quat_rp = _nh.advertise<geometry_msgs::QuaternionStamped>("/ekf/quat_rp", 1);
	_pub_quat_rpy = _nh.advertise<geometry_msgs::QuaternionStamped>("/ekf/quat_rpy", 1);
	/*initialize*/
	initializeState();
}

void AttitudeEstimationWallsEKF::initializeState(void)
{
	_x = Eigen::Vector2d::Zero();
	_P = _sigma_ini*Eigen::Matrix2d::Identity();

	if(!_wait_inipose)	_got_inipose = true;

	std::cout << "_x = " << std::endl << _x << std::endl;
	std::cout << "_P = " << std::endl << _P << std::endl;
}

void AttitudeEstimationWallsEKF::callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	if(!_got_inipose){
		tf::Quaternion q;
		quaternionMsgToTF(msg->quaternion, q);
		double r, p, y;
		tf::Matrix3x3(q).getRPY(r, p, y);
		_x <<
			r,
			p;
		_got_inipose = true;
	} 
}

void AttitudeEstimationWallsEKF::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*wait initial orientation*/
	if(!_got_inipose)	return;
	/*skip first callback*/
	if(!_got_first_imu){
		_stamp_imu_last = msg->header.stamp;
		_got_first_imu = true;
		return;
	}
	/*get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _stamp_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	/*(estimate yaw)*/
	estimateYaw(*msg, dt);
	/*prediction*/
	predictionIMU(*msg, dt);
	/*publication*/
	publication(msg->header.stamp);
	/*reset*/
	_stamp_imu_last = msg->header.stamp;

	/*test*/
	/* geometry_msgs::Vector3Stamped tmp; */
	/* tmp.vector.x = -msg->linear_acceleration.x; */
	/* tmp.vector.y = -msg->linear_acceleration.y; */
	/* tmp.vector.z = -msg->linear_acceleration.z; */
	/* observationG(tmp, 1.0e-0); */
}

void AttitudeEstimationWallsEKF::callbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!_got_bias){
		_bias = *msg;
		_got_bias = true;
	}
}

void AttitudeEstimationWallsEKF::callbackLidarG(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	/*wait initial orientation*/
	if(!_got_inipose)	return;
	/*observation*/
	observationG(*msg, _sigma_lidar_g);
	/*publication*/
	publication(msg->header.stamp);
}

void AttitudeEstimationWallsEKF::callbackCameraG(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	/*wait initial orientation*/
	if(!_got_inipose)	return;
	/*observation*/
	observationG(*msg, _sigma_camera_g);
	/*publication*/
	publication(msg->header.stamp);
}

void AttitudeEstimationWallsEKF::predictionIMU(sensor_msgs::Imu imu, double dt)
{
	/*u*/
	Eigen::Vector3d u;
	u <<
		imu.angular_velocity.x*dt,
		imu.angular_velocity.y*dt,
		imu.angular_velocity.z*dt;
	if(_got_bias){
		Eigen::Vector3d b;
		b <<
			_bias.angular_velocity.x*dt,
			_bias.angular_velocity.y*dt,
			_bias.angular_velocity.z*dt;
		u = u - b;
	}
	/*f*/
	Eigen::MatrixXd Rot(_x.size(), u.size());
	getRotMatrixRP(_x(0), _x(1), Rot);
	Eigen::VectorXd f = _x + Rot*u;
	/*jF*/
	Eigen::MatrixXd jF(_x.size(), _x.size());
	jF(0, 0) = 1 + u(1)*cos(_x(0))*tan(_x(1)) - u(2)*sin(_x(0))*tan(_x(1));
	jF(0, 1) = u(1)*sin(_x(0))/cos(_x(1))/cos(_x(1)) + u(2)*cos(_x(0))/cos(_x(1))/cos(_x(1));
	jF(1, 0) = -u(1)*sin(_x(0)) - u(2)*cos(_x(0));
	jF(1, 1) = 1;
	/*Q*/
	Eigen::MatrixXd Q = _sigma_imu*Eigen::MatrixXd::Identity(_x.size(), _x.size());
	/*Update*/
	_x = f;
	_P = jF*_P*jF.transpose() + Q;
	/*-pi to pi*/
	for(int i=0;i<_x.size();++i)	anglePiToPi(_x(i));
}

void AttitudeEstimationWallsEKF::observationG(geometry_msgs::Vector3Stamped g_msg, double sigma)
{
	/*print*/
	std::cout
		<< "r[deg]: " << _x(0)/M_PI*180.0
		<< ", "
		<< "p[deg]: " << _x(1)/M_PI*180.0
	<< std::endl;
	/*z*/
	Eigen::Vector3d z(g_msg.vector.x, g_msg.vector.y, g_msg.vector.z);
	z.normalize();
	std::cout << "z: " << z(0) << ", " << z(1) << ", " << z(2) << std::endl;
	/*zp*/
	const double g = -1.0;
	Eigen::Vector3d zp(
		-g*sin(_x(1)),
		g*sin(_x(0))*cos(_x(1)),
		g*cos(_x(0))*cos(_x(1))
	);
	std::cout << "zp: " << zp(0) << ", " << zp(1) << ", " << zp(2) << std::endl;
	/*jH*/
	Eigen::MatrixXd jH(z.size(), _x.size());
	/* jH << */
	jH <<
		0.0,						-g*cos(_x(1)),
		g*cos(_x(0))*cos(_x(1)),	-g*sin(_x(0))*sin(_x(1)),
		-g*sin(_x(0))*cos(_x(1)),	-g*cos(_x(0))*sin(_x(1));
	/*R*/
	Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(z.size(), z.size());
	/*I*/
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_x.size(), _x.size());
	/*y, s, K*/
	Eigen::Vector3d y = z - zp;
	Eigen::MatrixXd S = jH*_P*jH.transpose() + R;
	Eigen::MatrixXd K = _P*jH.transpose()*S.inverse();
	/*update*/
	_x = _x + K*y;
	_P = (I - K*jH)*_P;
	/*-pi to pi*/
	for(int i=0;i<_x.size();++i)	anglePiToPi(_x(i));
}

void AttitudeEstimationWallsEKF::publication(ros::Time stamp)
{
	/*RP*/
	tf::Quaternion q_rp = tf::createQuaternionFromRPY(_x(0), _x(1), 0.0);
	geometry_msgs::QuaternionStamped q_rp_msg;
	q_rp_msg.header.frame_id = _frame_id;
	q_rp_msg.header.stamp = stamp;
	q_rp_msg.quaternion.x = q_rp.x();
	q_rp_msg.quaternion.y = q_rp.y();
	q_rp_msg.quaternion.z = q_rp.z();
	q_rp_msg.quaternion.w = q_rp.w();
	_pub_quat_rp.publish(q_rp_msg);
	/*RPY*/
	tf::Quaternion q_rpy = tf::createQuaternionFromRPY(_x(0), _x(1), _yaw);
	geometry_msgs::QuaternionStamped q_rpy_msg;
	q_rpy_msg.header.frame_id = _frame_id;
	q_rpy_msg.header.stamp = stamp;
	q_rpy_msg.quaternion.x = q_rpy.x();
	q_rpy_msg.quaternion.y = q_rpy.y();
	q_rpy_msg.quaternion.z = q_rpy.z();
	q_rpy_msg.quaternion.w = q_rpy.w();
	_pub_quat_rpy.publish(q_rpy_msg);
	/*print*/
	// std::cout << "r[deg]: " << _x(0) << " p[deg]: " << _x(1) << std::endl;
}

void AttitudeEstimationWallsEKF::estimateYaw(sensor_msgs::Imu imu, double dt)
{
	/*u*/
	Eigen::Vector3d u;
	u <<
		imu.angular_velocity.x*dt,
		imu.angular_velocity.y*dt,
		imu.angular_velocity.z*dt;
	if(_got_bias){
		Eigen::Vector3d b;
		b <<
			_bias.angular_velocity.x*dt,
			_bias.angular_velocity.y*dt,
			_bias.angular_velocity.z*dt;
		u = u - b;
	}
	/*f*/
	Eigen::MatrixXd Rot(1, u.size());
	getRotMatrixY(_x(0), _x(1), Rot);
	_yaw = _yaw + (Rot*u)[0];
	anglePiToPi(_yaw);
}

void AttitudeEstimationWallsEKF::getRotMatrixRP(double r, double p, Eigen::MatrixXd& Rot)
{
	Rot <<
		1,	sin(r)*tan(p),	cos(r)*tan(p),
		0,	cos(r),			-sin(r);
}

void AttitudeEstimationWallsEKF::getRotMatrixY(double r, double p, Eigen::MatrixXd& Rot)
{
	Rot <<
		0,	sin(r)/cos(p),	cos(r)/cos(p);
}

void AttitudeEstimationWallsEKF::anglePiToPi(double& angle)
{
	angle = atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_estimation_walls_ekf");
	
	AttitudeEstimationWallsEKF attitude_estimation_walls_ekf;

	ros::spin();
}
