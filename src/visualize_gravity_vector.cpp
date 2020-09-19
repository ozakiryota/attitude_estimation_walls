#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

class VisualizeGravityVector{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_quat;
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_vector;
		/*publisher*/
		ros::Publisher _pub_vismarker;	//visualization
		/*flag*/
		//bool _got_first_msg = false;
		/*visualization marker*/
		visualization_msgs::Marker _g;	//visualization
		/*parameter*/
		std::string _frame_id;

	public:
		VisualizeGravityVector();
		void initializeVisMarker(void);
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg);
		geometry_msgs::Point quatToGravityVector(geometry_msgs::QuaternionStamped quat);
		bool varIsSmallEnough(sensor_msgs::Imu imu_msg);
		void inputVisMarker(std_msgs::Header header, geometry_msgs::Point end, double shaft_length, double shaft_diameter);
		void publication(bool g_is_available);
};

VisualizeGravityVector::VisualizeGravityVector()
	: _nhPrivate("~")
{
	std::cout << "--- visualize_gravity_vector ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("frame_id", _frame_id, std::string("/frame"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	/*subscriber*/
	_sub_quat = _nh.subscribe("/ekf/quat_rpy", 1, &VisualizeGravityVector::callbackQuat, this);
	_sub_imu = _nh.subscribe("/dnn/g_vector_with_cov", 1, &VisualizeGravityVector::callbackIMU, this);
	_sub_vector = _nh.subscribe("/lidar/g_vector", 1, &VisualizeGravityVector::callbackVector, this);
	/*subscriber*/
	_pub_vismarker = _nh.advertise<visualization_msgs::Marker>("/vis/gravity", 1);
	/*initialize*/
	initializeVisMarker();
}

void VisualizeGravityVector::initializeVisMarker(void)
{
	const double head_diameter = 0.75;
	// const double head_length = 1;
	_g.header.frame_id = _frame_id;
	_g.points.resize(2);
	_g.points[0].x = 0.0;
	_g.points[0].y = 0.0;
	_g.points[0].z = 0.0;
	_g.ns = "gravity";
	_g.id = 0;
	_g.type = visualization_msgs::Marker::ARROW;
	_g.action = visualization_msgs::Marker::ADD;
	_g.scale.y = head_diameter;
	// _g.scale.z = head_length;
}

void VisualizeGravityVector::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
	/*id*/
	_g.id = 0;
	/*end point*/
	geometry_msgs::Point end = quatToGravityVector(*msg);
	/*color*/
	_g.color.r = 1.0;
	_g.color.g = 1.0;
	_g.color.b = 1.0;
	_g.color.a = 1.0;
	/*input*/
	inputVisMarker(msg->header, end, 12.5, 0.25);
	/*publication*/
	publication(true);
}

void VisualizeGravityVector::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	if(varIsSmallEnough(*msg)){
		/*id*/
		_g.id = 1;
		/*end point*/
		geometry_msgs::Point end;
		end.x = msg->linear_acceleration.x;
		end.y = msg->linear_acceleration.y;
		end.z = msg->linear_acceleration.z;
		/*color*/
		_g.color.r = 1.0;
		_g.color.g = 1.0;
		_g.color.b = 0.0;
		_g.color.a = 1.0;
		/*input*/
		inputVisMarker(msg->header, end, 10.0, 0.5);
		/*publication*/
		publication(true);
	}
	else	publication(false);
}

void VisualizeGravityVector::callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	/*end point*/
	geometry_msgs::Point end;
	end.x = msg->vector.x;
	end.y = msg->vector.y;
	end.z = msg->vector.z;
	/*color*/
	_g.color.r = 0.5;
	_g.color.g = 1.0;
	_g.color.b = 0.0;
	_g.color.a = 1.0;
	/*input*/
	inputVisMarker(msg->header, end, 10.0, 0.5);
	/*id*/
	_g.id = 2;
	/*publication*/
	publication(true);
}

geometry_msgs::Point VisualizeGravityVector::quatToGravityVector(geometry_msgs::QuaternionStamped quat)
{
	tf::Quaternion q_pose;
	quaternionMsgToTF(quat.quaternion, q_pose);
	tf::Quaternion q_gravity_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_gravity_local = q_pose.inverse()*q_gravity_global*q_pose;

	geometry_msgs::Point g;
	g.x = q_gravity_local.x();
	g.y = q_gravity_local.y();
	g.z = q_gravity_local.z();
	return g;
}

bool VisualizeGravityVector::varIsSmallEnough(sensor_msgs::Imu imu_msg)
{
	/*judge*/
	const double th_mul_sigma = 8.0e-5;
	double mul_sigma = sqrt(imu_msg.linear_acceleration_covariance[0])
		*sqrt(imu_msg.linear_acceleration_covariance[4])
		*sqrt(imu_msg.linear_acceleration_covariance[8]);
	if(mul_sigma > th_mul_sigma){
		std::cout << "REJECT: mul_sigma = " << mul_sigma << " > " << th_mul_sigma << std::endl;
		return false;
	}
	std::cout << "mul_sigma = " << mul_sigma << std::endl;
	return true;
}

void VisualizeGravityVector::inputVisMarker(std_msgs::Header header, geometry_msgs::Point end, double shaft_length, double shaft_diameter)
{
	_g.header.stamp = header.stamp;
	_g.points[1].x = end.x*shaft_length;
	_g.points[1].y = end.y*shaft_length;
	_g.points[1].z = end.z*shaft_length;
	_g.scale.x = shaft_diameter;
}

void VisualizeGravityVector::publication(bool g_is_available)
{
	/*reset*/
	visualization_msgs::Marker delete_markers;
	delete_markers.action = visualization_msgs::Marker::DELETEALL;
	_pub_vismarker.publish(delete_markers);
	/*_g*/
	if(g_is_available)	_pub_vismarker.publish(_g);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_gravity_vector");
	
	VisualizeGravityVector visualize_gravity_vector;

	ros::spin();
}
