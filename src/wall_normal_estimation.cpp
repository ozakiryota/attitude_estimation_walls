#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <omp.h>

class WallNormalEstimation{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_quat;
		/*publish*/
		ros::Publisher _pub_nc;
		ros::Publisher _pub_selected_nc;
		ros::Publisher _pub_selected_d_gsphere;
		/*pcl*/
		pcl::visualization::PCLVisualizer _viewer {"wall_normal_estimation"};
		pcl::KdTreeFLANN<pcl::PointXYZ> _kdtree;
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _nc {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _selected_nc {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _d_gsphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _selected_d_gsphere {new pcl::PointCloud<pcl::PointXYZ>};
		/*objects*/
		Eigen::Vector3f _g_vector{0.0, 0.0, -1.0};	//tmp
		/*parameters*/
		bool _mode_open_viewer;
		int _omp_num_threads;
		int _skip;
		double _search_radius_ratio;
		double _min_search_radius;
		// double _max_search_radius;
		bool _mode_selection;
		bool _mode_remove_ground;
		int _th_num_neighborpoints;
		double _th_horizontal_angle;	//[deg]
		double _th_fitting_error;	//[m]

	public:
		WallNormalEstimation();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg);
		void clearPC(void);
		void computeNormal(void);
		double getDepth(pcl::PointXYZ point);
		std::vector<int> kdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		bool judgeForSelecting(const Eigen::Vector4f& plane_parameters, std::vector<int> indices);
		double getAngleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
		double computeFittingError(const Eigen::Vector4f& N, std::vector<int> indices);
		void quatToGravityVector(geometry_msgs::QuaternionStamped quat);
		void visualization(void);
		void publication(std_msgs::Header header);
};

WallNormalEstimation::WallNormalEstimation()
	:_nhPrivate("~")
{
	std::cout << "--- wall_normal_estimation ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("mode_open_viewer", _mode_open_viewer, true);
	std::cout << "_mode_open_viewer = " << (bool)_mode_open_viewer << std::endl;
	_nhPrivate.param("skip", _skip, 3);
	std::cout << "_skip = " << _skip << std::endl;
	_nhPrivate.param("omp_num_threads", _omp_num_threads, omp_get_max_threads());
	std::cout << "_omp_num_threads = " << _omp_num_threads << std::endl;
	_nhPrivate.param("search_radius_ratio", _search_radius_ratio, 0.09);
	std::cout << "_search_radius_ratio = " << _search_radius_ratio << std::endl;
	_nhPrivate.param("min_search_radius", _min_search_radius, 0.1);
	std::cout << "_min_search_radius = " << _min_search_radius << std::endl;
	// _nhPrivate.param("max_search_radius", _max_search_radius, 2.0);
	// std::cout << "_max_search_radius = " << _max_search_radius << std::endl;
	_nhPrivate.param("mode_remove_ground", _mode_remove_ground, true);
	std::cout << "_mode_remove_ground = " << (bool)_mode_remove_ground << std::endl;
	_nhPrivate.param("mode_selection", _mode_selection, true);
	std::cout << "_mode_selection = " << (bool)_mode_selection << std::endl;
	_nhPrivate.param("th_num_neighborpoints", _th_num_neighborpoints, 10);
	std::cout << "_th_num_neighborpoints = " << _th_num_neighborpoints << std::endl;
	_nhPrivate.param("th_horizontal_angle", _th_horizontal_angle, 15.0);
	std::cout << "_th_horizontal_angle = " << _th_horizontal_angle << std::endl;
	_nhPrivate.param("th_fitting_error", _th_fitting_error, 0.01);
	std::cout << "_th_fitting_error = " << _th_fitting_error << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &WallNormalEstimation::callbackPC, this);
	_sub_quat = _nh.subscribe("/quat", 1, &WallNormalEstimation::callbackQuat, this);
	/*publisher*/
	_pub_nc = _nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	_pub_selected_nc = _nh.advertise<sensor_msgs::PointCloud2>("/normals/selected", 1);
	_pub_selected_d_gsphere = _nh.advertise<sensor_msgs::PointCloud2>("/dgsphere/selected", 1);
	/*OMP*/
	if(_omp_num_threads < 1 || _omp_num_threads > omp_get_max_threads())	_omp_num_threads = omp_get_max_threads();
	/*viewer*/
	_viewer.setBackgroundColor(1, 1, 1);
	_viewer.addCoordinateSystem(1.0, "axis");
	// _viewer.setCameraPosition(-30.0, 0.0, 10.0, 0.0, 0.0, 1.0);
	_viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);
	if(!_mode_open_viewer)	_viewer.close();
}

void WallNormalEstimation::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *_pc);
	std::cout << "-----" << std::endl;
	std::cout << "_pc->points.size() = " << _pc->points.size() << std::endl;
	clearPC();

	computeNormal();

	if(_mode_open_viewer)	visualization();
	publication(msg->header);
}

void WallNormalEstimation::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
	quatToGravityVector(*msg);
}

void WallNormalEstimation::clearPC(void)
{
	_nc->points.clear();
	_selected_nc->points.clear();
	_d_gsphere->points.clear();
	_selected_d_gsphere->points.clear();
}

void WallNormalEstimation::computeNormal(void)
{
	std::cout << "_omp_num_threads = " << _omp_num_threads << " out of " << omp_get_max_threads() << std::endl;

	double time_start = ros::Time::now().toSec();

	_kdtree.setInputCloud(_pc);

	_nc->points.resize((_pc->points.size()-1)/_skip + 1);
	_d_gsphere->points.resize((_pc->points.size()-1)/_skip + 1);
	std::vector<bool> extract_indices((_pc->points.size()-1)/_skip + 1, false);

	#ifdef _OPENMP
	#pragma omp parallel for num_threads(_omp_num_threads)
	#endif
	for(size_t i=0; i<_pc->points.size(); i+=_skip){
		size_t normal_index = i/_skip;
		/*search neighbor points*/
		double laser_distance = getDepth(_pc->points[i]);
		double search_radius = _search_radius_ratio*laser_distance;
		if(search_radius < _min_search_radius)	search_radius = _min_search_radius;
		// if(search_radius > _max_search_radius)	search_radius = _max_search_radius;
		std::vector<int> indices = kdtreeSearch(_pc->points[i], search_radius);
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*_pc, indices, plane_parameters, curvature);
		/*judge*/
		if(_mode_selection)	extract_indices[normal_index] = judgeForSelecting(plane_parameters, indices);
		else	extract_indices[normal_index] = true;
		/*input*/
		_nc->points[normal_index].x = _pc->points[i].x;
		_nc->points[normal_index].y = _pc->points[i].y;
		_nc->points[normal_index].z = _pc->points[i].z;
		_nc->points[normal_index].data_n[0] = plane_parameters(0);
		_nc->points[normal_index].data_n[1] = plane_parameters(1);
		_nc->points[normal_index].data_n[2] = plane_parameters(2);
		_nc->points[normal_index].data_n[3] = plane_parameters(3);
		_nc->points[normal_index].curvature = curvature;
		flipNormalTowardsViewpoint(_pc->points[i], 0.0, 0.0, 0.0, _nc->points[normal_index].normal_x, _nc->points[normal_index].normal_y, _nc->points[normal_index].normal_z);
		/*Gauss map*/
		_d_gsphere->points[normal_index].x = -fabs(_nc->points[normal_index].data_n[3])*_nc->points[normal_index].data_n[0];
		_d_gsphere->points[normal_index].y = -fabs(_nc->points[normal_index].data_n[3])*_nc->points[normal_index].data_n[1];
		_d_gsphere->points[normal_index].z = -fabs(_nc->points[normal_index].data_n[3])*_nc->points[normal_index].data_n[2];
	}
	if(_mode_selection){
		for(size_t i=0; i<extract_indices.size(); i++){
			if(extract_indices[i]){
				/*selected normals*/
				_selected_nc->points.push_back(_nc->points[i]);
				/*selected d-gaussian shpere*/
				_selected_d_gsphere->points.push_back(_d_gsphere->points[i]);
			}
		}
	}
	else{
		_selected_nc = _nc;
		_selected_d_gsphere = _d_gsphere;
	}

	std::cout << "duration[s] = " << ros::Time::now().toSec() - time_start << " / frequency[hz] = " << 1.0/(ros::Time::now().toSec() - time_start) << std::endl;
	std::cout << "_selected_nc->points.size() = " << _selected_nc->points.size() << "(" << _selected_nc->points.size()/(double)_pc->points.size()*100.0 << " %)" << std::endl;
}

double WallNormalEstimation::getDepth(pcl::PointXYZ point)
{
	double depth = sqrt(
		point.x*point.x
		+ point.y*point.y
		+ point.z*point.z
	);
	return depth;
}

std::vector<int> WallNormalEstimation::kdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(_kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

bool WallNormalEstimation::judgeForSelecting(const Eigen::Vector4f& plane_parameters, std::vector<int> indices)
{
	/*number of neighbor-points*/
	if(indices.size() < _th_num_neighborpoints)	return false;
	/*nan*/
	if(std::isnan(plane_parameters(0)) || std::isnan(plane_parameters(1)) || std::isnan(plane_parameters(2)))	return false;
	/*angle*/
	if(_mode_remove_ground){
		if(fabs(getAngleBetweenVectors(plane_parameters.segment(0, 3), _g_vector)-M_PI/2.0) > _th_horizontal_angle/180.0*M_PI)	return false;
	}
	/*fitting error*/
	if(computeFittingError(plane_parameters, indices) > _th_fitting_error)	return false;
	/*pass*/
	return true;
}

double WallNormalEstimation::getAngleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
	double angle = acos(v1.dot(v2)/v1.norm()/v2.norm());
	return angle;
}

double WallNormalEstimation::computeFittingError(const Eigen::Vector4f& N, std::vector<int> indices)
{
	double ave_fitting_error = 0.0;
	for(size_t i=0; i<indices.size(); ++i){
		Eigen::Vector3f P(
			_pc->points[indices[i]].x,
			_pc->points[indices[i]].y,
			_pc->points[indices[i]].z
		);
		ave_fitting_error += fabs(N.segment(0, 3).dot(P) + N(3))/N.segment(0, 3).norm();
	}
	ave_fitting_error /= (double)indices.size();
	return ave_fitting_error;
}

void WallNormalEstimation::quatToGravityVector(geometry_msgs::QuaternionStamped quat)
{
	tf::Quaternion q_pose;
	quaternionMsgToTF(quat.quaternion, q_pose);
	tf::Quaternion q_gravity_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_gravity_local = q_pose.inverse()*q_gravity_global*q_pose;
	_g_vector(0) = q_gravity_local.x();
	_g_vector(1) = q_gravity_local.y();
	_g_vector(2) = q_gravity_local.z();
}

void WallNormalEstimation::visualization(void)
{
	_viewer.removeAllPointClouds();

	/*cloud*/
	_viewer.addPointCloud(_pc, "_pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "_pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "_pc");
	/*normals*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_nc, 1, 0.5, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "_nc");
	/*selected normals*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_selected_nc, 1, 0.5, "_selected_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "_selected_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "_selected_nc");
	/*d-gaussian sphere*/
	_viewer.addPointCloud(_d_gsphere, "_d_gsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "_d_gsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "_d_gsphere");
	/*selected d-gaussian sphere*/
	_viewer.addPointCloud(_selected_d_gsphere, "_selected_d_gsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "_selected_d_gsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "_selected_d_gsphere");
	
	_viewer.spinOnce();
}

void WallNormalEstimation::publication(std_msgs::Header header)
{
	/*normals*/
	//_nc->header.stamp = _pc->header.stamp;
	//_nc->header.frame_id = _pc->header.frame_id;
	sensor_msgs::PointCloud2 nc_msg;
	pcl::toROSMsg(*_nc, nc_msg);
	nc_msg.header = header;
	_pub_nc.publish(nc_msg);	
	/*selected normals*/
	if(_mode_selection){
		//_selected_nc->header.stamp = _pc->header.stamp;
		//_selected_nc->header.frame_id = _pc->header.frame_id;
		sensor_msgs::PointCloud2 selected_nc_msg;
		pcl::toROSMsg(*_selected_nc, selected_nc_msg);
		selected_nc_msg.header = header;
		_pub_selected_nc.publish(selected_nc_msg);
	}
	/*selected d-gaussian sphere*/
	//_selected_d_gsphere->header.stamp = _pc->header.stamp;
	//_selected_d_gsphere->header.frame_id = _pc->header.frame_id;
	sensor_msgs::PointCloud2 selected_gsphere_msg;
	pcl::toROSMsg(*_selected_d_gsphere, selected_gsphere_msg);
	selected_gsphere_msg.header = header;
	_pub_selected_d_gsphere.publish(selected_gsphere_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_normal_estimation");
	
	WallNormalEstimation wall_normal_estimation;

	ros::spin();
}
