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
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_lidar_g;
		/*publisher*/
		ros::Publisher _pub_quat;
		/*state*/
		Eigen::Vector2d _x;
		Eigen::Matrix2d _P;
		/*time*/
		ros::Time _stamp_imu_last;
		/*flag*/
		bool _got_first_imu = false;
		/*parameter*/
		std::string _frame_id;
		double _sigma_imu;

	public:
		AttitudeEstimationWallsEKF();
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackLidarG(const geometry_msgs::Vector3StampedConstPtr& msg);
		void predictionIMU(sensor_msgs::Imu imu, double dt);
		void publication(ros::Time stamp);
		void getRotMatrixRP(double r, double p, Eigen::MatrixXd& Rot);
};

AttitudeEstimationWallsEKF::AttitudeEstimationWallsEKF()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("frame_id", _frame_id, std::string("/base_link"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("sigma_imu", _sigma_imu, 1.0e-4);
	std::cout << "_sigma_imu = " << _sigma_imu << std::endl;
	/*sub*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &AttitudeEstimationWallsEKF::callbackIMU, this);
	_sub_lidar_g = _nh.subscribe("/lidar_g", 1, &AttitudeEstimationWallsEKF::callbackLidarG, this);
	/*pub*/
	_pub_quat = _nh.advertise<geometry_msgs::QuaternionStamped>("/attitude", 1);
}

void AttitudeEstimationWallsEKF::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
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
	/*prediction*/
	predictionIMU(*msg, dt);
	/*publication*/
	publication(msg->header.stamp);
	/*reset*/
	_stamp_imu_last = msg->header.stamp;
}

void AttitudeEstimationWallsEKF::callbackLidarG(const geometry_msgs::Vector3StampedConstPtr& msg)
{
}

void AttitudeEstimationWallsEKF::predictionIMU(sensor_msgs::Imu imu, double dt)
{
	/*u*/
	Eigen::Vector3d u;
	u <<
		imu.angular_velocity.x*dt,
		imu.angular_velocity.y*dt,
		imu.angular_velocity.z*dt;
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
}

void AttitudeEstimationWallsEKF::publication(ros::Time stamp)
{
	/*pub*/
	tf::Quaternion q = tf::createQuaternionFromRPY(_x(0), _x(1), 0.0);
	geometry_msgs::QuaternionStamped q_msg;
	q_msg.header.stamp = stamp;
	q_msg.quaternion.x = q.x();
	q_msg.quaternion.y = q.y();
	q_msg.quaternion.z = q.z();
	q_msg.quaternion.w = q.w();
	_pub_quat.publish(q_msg);
	/*print*/
	/* std::cout << "r[deg]: " << _x(0) << " p[deg]: " << _x(1) << std::endl; */
}

void AttitudeEstimationWallsEKF::getRotMatrixRP(double r, double p, Eigen::MatrixXd& Rot)
{
	Rot <<
		1,	sin(r)*tan(p),	cos(r)*tan(p),
		0,	cos(r),			-sin(r);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_estimation_walls_ekf");
	
	AttitudeEstimationWallsEKF attitude_estimation_walls_ekf;

	ros::spin();
}
