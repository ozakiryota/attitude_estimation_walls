#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>

class DgsphereToGravity{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_quat;
		/*publish*/
		ros::Publisher _pub_vector;
		ros::Publisher _pub_markerarray;	//visualization
		/*pcl*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};
		/*gravity*/
		Eigen::Vector3d _g_vector{0.0, 0.0, -1.0};	//tmp
		/*clusters*/
		std::vector<Eigen::Vector3d> _clusters;
		std::vector<int> _num_cluster_members;
		/*flag*/
		bool _g_is_available = false;
		/*visualization marker*/
		visualization_msgs::MarkerArray _arrows;	//visualization
		/*parameters*/
		double _th_clustering_angle_deg;
		int _min_num_cluster_members;
		double _th_anglediff_gnew_glast_deg;

	public:
		DgsphereToGravity();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg);
		void clearList(void);
		void addToClusters(void);
		void eraseSmallCluster(void);
		bool estimateG(void);
		void flipG(const Eigen::Vector3d& g_last, Eigen::Vector3d& g_new);
		void quatToGravityVector(geometry_msgs::QuaternionStamped quat);
		void publication(std_msgs::Header header);
		void inputMarkerArray(std_msgs::Header header);
		double getAngleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
};

DgsphereToGravity::DgsphereToGravity()
	:_nhPrivate("~")
{
	std::cout << "--- dgsphere_to_gravity ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("th_clustering_angle_deg", _th_clustering_angle_deg, 5.0);
	std::cout << "_th_clustering_angle_deg = " << _th_clustering_angle_deg << std::endl;
	_nhPrivate.param("min_num_cluster_members", _min_num_cluster_members, 10);
	std::cout << "_min_num_cluster_members = " << _min_num_cluster_members << std::endl;
	_nhPrivate.param("th_anglediff_gnew_glast_deg", _th_anglediff_gnew_glast_deg, 10.0);
	std::cout << "_th_anglediff_gnew_glast_deg = " << _th_anglediff_gnew_glast_deg << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &DgsphereToGravity::callbackPC, this);
	_sub_quat = _nh.subscribe("/quat", 1, &DgsphereToGravity::callbackQuat, this);
	/*publisher*/
	_pub_vector = _nh.advertise<geometry_msgs::Vector3Stamped>("/lidar/g_vector", 1);
	_pub_markerarray = _nh.advertise<visualization_msgs::MarkerArray>("/dgsphere/clustered", 1);
}

void DgsphereToGravity::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *_pc);

	clearList();
	addToClusters();

	/*print*/
	std::cout << "-----" << std::endl;
	std::cout << "_pc->points.size() = " << _pc->points.size() << std::endl;
	int sum_num_cluster_members = 0;
	for(size_t i=0; i<_clusters.size(); ++i){
		std::cout << "i = " << i << std::endl;
		std::cout << "_clusters[i]: " << _clusters[i](0) << ", " << _clusters[i](1) << ", " << _clusters[i](2) << std::endl;
		std::cout << "_num_cluster_members[i]: " << _num_cluster_members[i] << std::endl;
		sum_num_cluster_members +=  _num_cluster_members[i];
	}

	std::cout << "sum_num_cluster_members = " << sum_num_cluster_members << std::endl;
	eraseSmallCluster();
	_g_is_available = estimateG();
	// if(!_clusters.empty()){
	inputMarkerArray(msg->header);
	publication(msg->header);
	// }
}

void DgsphereToGravity::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
	quatToGravityVector(*msg);
}

void DgsphereToGravity::clearList(void)
{
	_clusters.clear();
	_num_cluster_members.clear();
	_arrows.markers.clear();
}

void DgsphereToGravity::addToClusters(void)
{
	for(size_t point_i=0; point_i<_pc->points.size(); ++point_i){
		bool found_cluster = false;
		Eigen::Vector3d v_p(_pc->points[point_i].x, _pc->points[point_i].y, _pc->points[point_i].z);
		for(size_t cluster_i=0; cluster_i<_clusters.size(); ++cluster_i){
			double angle1 = getAngleBetweenVectors(v_p, _clusters[cluster_i]);
			if(angle1/M_PI*180.0 < _th_clustering_angle_deg){
				_clusters[cluster_i] += v_p;
				_num_cluster_members[cluster_i] += 1;
				found_cluster = true;
				break;
			}
			double angle2 = getAngleBetweenVectors(-v_p, _clusters[cluster_i]);
			if(angle2/M_PI*180.0 < _th_clustering_angle_deg){
				_clusters[cluster_i] -= v_p;
				_num_cluster_members[cluster_i] += 1;
				found_cluster = true;
				break;
			}
		}
		if(!found_cluster){
			_clusters.push_back(v_p);
			_num_cluster_members.push_back(1);
		}
	}
}

void DgsphereToGravity::eraseSmallCluster(void)
{
	for(size_t i=0; i<_clusters.size(); ){
		if(_num_cluster_members[i] < _min_num_cluster_members){
			_clusters.erase(_clusters.begin() + i);
			_num_cluster_members.erase(_num_cluster_members.begin() + i);
		}
		else	++i;
	}
}

bool DgsphereToGravity::estimateG(void)
{
	Eigen::Vector3d _new_g_vector(0.0, 0.0, 0.0);

	if(_clusters.size() == 0)	return false;
	else if(_clusters.size() == 1){
		_new_g_vector = _g_vector - _g_vector.dot(_clusters[0])/_clusters[0].norm()/_clusters[0].norm()*_clusters[0];
		flipG(_g_vector, _new_g_vector);
	}
	else{
		for(size_t i=0; i<_clusters.size()-1; ++i){
			for(size_t j=i+1; j<_clusters.size(); ++j){
				Eigen::Vector3d cross = _clusters[i].cross(_clusters[j]);
				// std::cout << "_clusters[i]: " << _clusters[i](0) << ", " << _clusters[i](1) << ", " << _clusters[i](2) << std::endl;
				// std::cout << "_clusters[j]: " << _clusters[j](0) << ", " << _clusters[j](1) << ", " << _clusters[j](2) << std::endl;
				// std::cout << "cross: " << cross(0) << ", " << cross(1) << ", " << cross(2) << std::endl;
				flipG(_g_vector, cross);
				_new_g_vector += cross;
			}
		}
	}
	_new_g_vector.normalize();
	/*final judge*/
	if(getAngleBetweenVectors(_g_vector, _new_g_vector)/M_PI*180.0 < _th_anglediff_gnew_glast_deg){
		_g_vector = _new_g_vector;
		std::cout << "_new_g_vector: " << _new_g_vector(0) << ", " << _new_g_vector(1) << ", " << _new_g_vector(2) << std::endl;
		return true;
	}
	else{
		std::cout << "getAngleBetweenVectors(_g_vector, _new_g_vector)/M_PI*180.0 = " << getAngleBetweenVectors(_g_vector, _new_g_vector)/M_PI*180.0 << " > " << _th_anglediff_gnew_glast_deg << std::endl;
		std::cout << "_new_g_vector: " << _new_g_vector(0) << ", " << _new_g_vector(1) << ", " << _new_g_vector(2) << std::endl;
		// exit(1);
		return false;
	}
}

void DgsphereToGravity::flipG(const Eigen::Vector3d& g_last, Eigen::Vector3d& g_new)
{
	const double th_flip_angle_deg = 90.0;
	double angle = getAngleBetweenVectors(g_last, g_new);
	if(angle/M_PI*180.0 > th_flip_angle_deg)	g_new *= -1;
}

void DgsphereToGravity::quatToGravityVector(geometry_msgs::QuaternionStamped quat)
{
	tf::Quaternion q_pose;
	quaternionMsgToTF(quat.quaternion, q_pose);
	tf::Quaternion q_gravity_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_gravity_local = q_pose.inverse()*q_gravity_global*q_pose;
	_g_vector(0) = q_gravity_local.x();
	_g_vector(1) = q_gravity_local.y();
	_g_vector(2) = q_gravity_local.z();
}

void DgsphereToGravity::publication(std_msgs::Header header)
{
	/*delay*/
	std::cout << "delay[s]: " << (ros::Time::now() - header.stamp).toSec() << std::endl;
	/*_arrows*/
	_pub_markerarray.publish(_arrows);
	/*_g_vector*/
	if(_g_is_available){
		geometry_msgs::Vector3Stamped vector_msg;
		vector_msg.header = header;
		vector_msg.vector.x = _g_vector(0);
		vector_msg.vector.y = _g_vector(1);
		vector_msg.vector.z = _g_vector(2);
		_pub_vector.publish(vector_msg);
	}
}

void DgsphereToGravity::inputMarkerArray(std_msgs::Header header)
{
	/*delete all*/
	visualization_msgs::Marker delete_markers;
	delete_markers.action = visualization_msgs::Marker::DELETEALL;
	_arrows.markers.push_back(delete_markers);
	/*clustered dgsphere*/
	const double shaft_diameter_n = 0.1;
	const double head_diameter_n = 0.2;
	// const double head_length = 1;
	geometry_msgs::Point start;
	start.x = 0.0;
	start.y = 0.0;
	start.z = 0.0;
	for(size_t i=0; i<_clusters.size(); ++i){
		visualization_msgs::Marker tmp;
		tmp.header = header;
		tmp.ns = "dgsphere";
		tmp.id = _arrows.markers.size();
		tmp.type = visualization_msgs::Marker::ARROW;
		tmp.action = visualization_msgs::Marker::ADD;
		tmp.scale.x = shaft_diameter_n;
		tmp.scale.y = head_diameter_n;
		// tmp.scale.z = head_length;
		tmp.color.r = 1.0;
		tmp.color.b = 1.0;
		tmp.color.a = 1.0;
		geometry_msgs::Point end;
		end.x = _clusters[i](0);
		end.y = _clusters[i](1);
		end.z = _clusters[i](2);
		tmp.points.push_back(start);
		tmp.points.push_back(end);
		_arrows.markers.push_back(tmp);
	}
	/*gravity*/
	const double shaft_diameter_g = 0.5;
	const double head_diameter_g = 0.75;
	if(_g_is_available){
		const double gravity_lengh = 10.0;
		visualization_msgs::Marker gravity;
		gravity.header = header;
		gravity.ns = "gravity";
		gravity.type = visualization_msgs::Marker::ARROW;
		gravity.action = visualization_msgs::Marker::ADD;
		gravity.scale.x = shaft_diameter_g;
		gravity.scale.y = head_diameter_g;
		gravity.color.g = 1.0;
		gravity.color.a = 1.0;
		geometry_msgs::Point end;
		end.x = _g_vector(0)*gravity_lengh;
		end.y = _g_vector(1)*gravity_lengh;
		end.z = _g_vector(2)*gravity_lengh;
		gravity.points.push_back(start);
		gravity.points.push_back(end);
		_arrows.markers.push_back(gravity);
	}
}

double DgsphereToGravity::getAngleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
	return acos(v1.dot(v2)/v1.norm()/v2.norm());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dgsphere_to_gravity");
	
	DgsphereToGravity dgsphere_to_gravity;

	ros::spin();
}
