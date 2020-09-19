#include <ros/ros.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
// #include <sensor_msgs/PointCloud2.h>

class RecordSubTiming{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_imu_0;
		ros::Subscriber _sub_imu_1;
		ros::Subscriber _sub_vector_0;
		// ros::Subscriber _sub_pc_0;
		/*file*/
		std::ofstream _csvfile;
		/*time*/
		ros::Time _stamp_start;
		/*counter*/
		std::vector<int> _sub_counter;
		/*flag*/
		bool _got_first_msg = false;
		/*parameter*/
		int _num_sub = 3;
		std::string _save_csv_path;
		bool _use_ros_time_now;
		double slide = 0.025;

	public:
		RecordSubTiming();
		void csvInitialization(void);
		void callbackIMU0(const sensor_msgs::ImuConstPtr& msg);
		void callbackIMU1(const sensor_msgs::ImuConstPtr& msg);
		void callbackVector0(const geometry_msgs::Vector3StampedConstPtr& msg);
		//void callbackPC0(const sensor_msgs::PointCloud2ConstPtr &msg);
		bool isCountedIMU1(sensor_msgs::Imu imu_msg);
		void record(int index, ros::Time stamp);
};

RecordSubTiming::RecordSubTiming()
	: _nhPrivate("~")
{
	std::cout << "--- record_sub_timing ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("save_csv_path", _save_csv_path, std::string("record_sub_timing.csv"));
	std::cout << "_save_csv_path = " << _save_csv_path << std::endl;
	_nhPrivate.param("use_ros_time_now", _use_ros_time_now, false);
	std::cout << "_use_ros_time_now = " << (bool)_use_ros_time_now << std::endl;
	/*subscriber*/
	_sub_imu_0 = _nh.subscribe("/imu_0", 1, &RecordSubTiming::callbackIMU0, this);
	_sub_imu_1 = _nh.subscribe("/imu_1", 1, &RecordSubTiming::callbackIMU1, this);
	_sub_vector_0 = _nh.subscribe("/vector_0", 1, &RecordSubTiming::callbackVector0, this);
	//_sub_pc_0 = _nh.subscribe("/cloud_0", 1, &RecordSubTiming::callbackPC0, this);
	/*initialize*/
	csvInitialization();
	_sub_counter = std::vector<int>(_num_sub, 0);
}

void RecordSubTiming::csvInitialization(void)
{
	/*open*/
	_csvfile.open(_save_csv_path, std::ios::out);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_csv_path << std::endl;
		exit(1);
	}
	_csvfile 
		<< "time" << "," 
		<< "imu_0" << "," << "," << ","
		<< "imu_1" << "," << "," << ","
		<< "vector_0" << "," << ","
		<< std::endl;
}

void RecordSubTiming::callbackIMU0(const sensor_msgs::ImuConstPtr& msg)
{
	const int index = 0;

	/*keep*/
	if(!_got_first_msg){
		if(_use_ros_time_now)	_stamp_start = ros::Time::now();
		else	_stamp_start = msg->header.stamp;
		_got_first_msg = true;
	}
	/*counter*/
	++_sub_counter[index];
	/*record*/
	record(index, msg->header.stamp);
}

void RecordSubTiming::callbackIMU1(const sensor_msgs::ImuConstPtr& msg)
{
	const int index = 1;

	/*keep*/
	if(!_got_first_msg){
		if(_use_ros_time_now)	_stamp_start = ros::Time::now();
		else	_stamp_start = msg->header.stamp;
		_got_first_msg = true;
	}
	/*judge*/
	if(isCountedIMU1(*msg)){
		/*counter*/
		++_sub_counter[index];
		/*record*/
		record(index, msg->header.stamp);
	}
}

void RecordSubTiming::callbackVector0(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	const int index = 2;

	/*keep*/
	if(!_got_first_msg){
		if(_use_ros_time_now)	_stamp_start = ros::Time::now();
		else	_stamp_start = msg->header.stamp;
		_got_first_msg = true;
	}
	/*counter*/
	++_sub_counter[index];
	/*record*/
	record(index, msg->header.stamp);
}

//void RecordSubTiming::callbackPC0(const sensor_msgs::PointCloud2ConstPtr &msg)
//{
//	const int index = 3;
//
//	/*keep*/
//	if(!_got_first_msg){
//		if(_use_ros_time_now)	_stamp_start = ros::Time::now();
//		else	_stamp_start = msg->header.stamp;
//		_got_first_msg = true;
//	}
//	/*counter*/
//	++_sub_counter[index];
//	/*record*/
//	record(index, msg->header.stamp);
//}

bool RecordSubTiming::isCountedIMU1(sensor_msgs::Imu imu_msg)
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

void RecordSubTiming::record(int index, ros::Time stamp)
{
	/*csv*/
	double t;
	if(_use_ros_time_now)	t = (ros::Time::now() - _stamp_start).toSec();
	else	t = (stamp - _stamp_start).toSec();
	_csvfile << t << ",";
	for(int i=0; i<_num_sub; ++i){
		if(i == index)	_csvfile << 1 << "," << 1 + i*slide << "," << _sub_counter[i] << ",";
		//else	_csvfile << 0 << ",";
		else	_csvfile << "," << "," << ",";
	}
	_csvfile << std::endl;

	/*print*/
	for(int i=0; i<_num_sub; ++i){
		std::cout << "_sub_counter[" << i << "] = " << _sub_counter[i] << " ";
	}
	std::cout << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_sub_timing");
	
	RecordSubTiming record_sub_timing;

	ros::spin();
}
