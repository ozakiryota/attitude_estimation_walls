#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>


class SQLidarPlanarNormalEstimation{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_pc;
		ros::Publisher _pub_plane;
		ros::Publisher _pub_nc;
		ros::Publisher _pub_gsphere;
		ros::Publisher _pub_g;
		/*tf*/
		tf::TransformListener _tflistener;
		/*point cloud*/
		pcl::PointCloud<pcl::InterestPoint>::Ptr _pc {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _pc_plane_now {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _pc_plane_last {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr _nc {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _dgsphere {new pcl::PointCloud<pcl::PointXYZ>};
		/*list*/
		std::vector<size_t> _list_num_points;
		/*clusters*/
		std::vector<Eigen::Vector3d> _clusters;
		/*gravity vector*/
		Eigen::Vector3d _g{0.0, 0.0, -1.0};
		/*odom*/
		nav_msgs::Odometry _odom_now;
		nav_msgs::Odometry _odom_last;
		/*viewer*/
		pcl::visualization::PCLVisualizer _viewer{"sqlidar_planar_normal_estimation"};
		/*flag*/
		bool got_first_odom = false;
		/*parameter*/
		const double _laser_range = 50.0;
		std::string _pubmsg_frame;
		std::string _target_frame;
		int _limit_store_scan;
		int _curvature_region;
		double _th_flatness;
		double _th_assoc_squareddist;
		double _th_assoc_crossangle_deg;
		double _th_angle_from_horizon_deg;
		double _th_clustering_angle_deg;
		double _min_cluster_length;
		double _th_anglediff_gnew_glast_deg;

	public:
		SQLidarPlanarNormalEstimation();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void orientationToG(geometry_msgs::Quaternion ori);
		void reset(void);
		void print(void);
		void erasePoints();
		bool transformPCWithTF(const sensor_msgs::PointCloud2& pc2_in, sensor_msgs::PointCloud2& pc2_out, std::string target_frame, ros::Time target_stamp);
		void transformPCWithOdom(pcl::PointCloud<pcl::PointNormal>::Ptr pc_inout, nav_msgs::Odometry odom_last, nav_msgs::Odometry odom_now);
		void eraseNanPoint(void);
		void extractWall(void);
		bool estimateNormal(pcl::KdTreeFLANN<pcl::PointNormal>& kdtree, pcl::PointNormal& n);
		bool isVerticalPlane(pcl::PointNormal n);
		void dGaussMap(const pcl::PointNormal& n, pcl::PointXYZ& p);
		void eraseSmallCluster(void);
		void addToClusters(pcl::PointXYZ p);
		bool estimateG(void);
		void publication(ros::Time stamp, bool g_is_available);
		void visualization(void);
		double norm(double x, double y, double z);
		double angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
		void flipToHalfSphere(Eigen::Vector3d& v);
		void flipG(const Eigen::Vector3d& g_last, Eigen::Vector3d& g_new);
		void printGToRP(const Eigen::Vector3d& g);
};

SQLidarPlanarNormalEstimation::SQLidarPlanarNormalEstimation()
	: _nhPrivate("~")
{
	std::cout << "--- sqlidar_gravity_estimation ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("pubmsg_frame", _pubmsg_frame, std::string("/laser"));
	std::cout << "_pubmsg_frame = " << _pubmsg_frame << std::endl;
	_nhPrivate.param("target_frame", _target_frame, std::string("/base_link"));
	std::cout << "_target_frame = " << _target_frame << std::endl;
	_nhPrivate.param("limit_store_scan", _limit_store_scan, 15);	//417points/scan
	std::cout << "_limit_store_scan = " << _limit_store_scan << std::endl;
	_nhPrivate.param("curvature_region", _curvature_region, 5);
	std::cout << "_curvature_region = " << _curvature_region << std::endl;
	_nhPrivate.param("th_flatness", _th_flatness, 1.0e-3);
	std::cout << "_th_flatness = " << _th_flatness << std::endl;
	_nhPrivate.param("th_assoc_squareddist",  _th_assoc_squareddist, 5.0e-1);
	std::cout << " _th_assoc_squareddist = " <<  _th_assoc_squareddist << std::endl;
	_nhPrivate.param("th_assoc_crossangle_deg", _th_assoc_crossangle_deg, 5.0);
	std::cout << "_th_assoc_crossangle_deg = " << _th_assoc_crossangle_deg << std::endl;
	_nhPrivate.param("th_angle_from_horizon_deg", _th_angle_from_horizon_deg, 10.0);
	std::cout << "_th_angle_from_horizon_deg = " << _th_angle_from_horizon_deg << std::endl;
	_nhPrivate.param("th_clustering_angle_deg", _th_clustering_angle_deg, 5.0);
	std::cout << "_th_clustering_angle_deg = " << _th_clustering_angle_deg << std::endl;
	_nhPrivate.param("min_cluster_length", _min_cluster_length, 10.0);
	std::cout << "_min_cluster_length = " << _min_cluster_length << std::endl;
	_nhPrivate.param("th_anglediff_gnew_glast_deg", _th_anglediff_gnew_glast_deg, 10.0);
	std::cout << "_th_anglediff_gnew_glast_deg = " << _th_anglediff_gnew_glast_deg << std::endl;
	/*sub*/
	_sub_pc = _nh.subscribe("/cloud", 1, &SQLidarPlanarNormalEstimation::callbackPC, this);
	_sub_odom = _nh.subscribe("/odom", 1, &SQLidarPlanarNormalEstimation::callbackOdom, this);
	/*pub*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/with_flatness", 1);
	_pub_plane = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/plane", 1);
	_pub_nc = _nh.advertise<sensor_msgs::PointCloud2>("/normals", 1);
	_pub_gsphere = _nh.advertise<sensor_msgs::PointCloud2>("/gsphere", 1);
	_pub_g = _nh.advertise<geometry_msgs::Vector3Stamped>("/lidar/g_vector", 1);
	/*viewer*/
	_viewer.setBackgroundColor(1, 1, 1);
	_viewer.addCoordinateSystem(0.5, "axis");
}

void SQLidarPlanarNormalEstimation::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom_now = *msg;
	orientationToG(msg->pose.pose.orientation);
	if(!got_first_odom){
		_odom_last = _odom_now;
		got_first_odom = true;
	}
}

void SQLidarPlanarNormalEstimation::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!got_first_odom)	return;

	reset();

	sensor_msgs::PointCloud2 pc_trans;
	if(!transformPCWithTF(*msg, pc_trans, _target_frame, msg->header.stamp))	return;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);	//avoiding "falied to find match for field 'strength'"
	pcl::fromROSMsg(pc_trans, *tmp);
	copyPointCloud(*tmp, *_pc);
	eraseNanPoint();

	extractWall();
	eraseSmallCluster();
	if(!_clusters.empty())	print();
	bool g_is_available = estimateG();
	publication(msg->header.stamp, g_is_available);
	visualization();
}

void SQLidarPlanarNormalEstimation::orientationToG(geometry_msgs::Quaternion ori)
{
	tf::Quaternion q_ori;
	quaternionMsgToTF(ori, q_ori);
	double r, p, y;
	tf::Matrix3x3(q_ori).getRPY(r, p, y);
	tf::Quaternion q_rp = tf::createQuaternionFromRPY(r, p, 0.0);

	tf::Quaternion q_g_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_g_local = q_rp.inverse()*q_g_global*q_rp;

	_g(0) = q_g_local.x();
	_g(1) = q_g_local.y();
	_g(2) = q_g_local.z();
}

void SQLidarPlanarNormalEstimation::reset(void)
{
	*_pc_plane_last += *_pc_plane_now;
	_list_num_points.push_back(_pc_plane_now->points.size());
	erasePoints();
	transformPCWithOdom(_pc_plane_last, _odom_last, _odom_now);
	_odom_last = _odom_now;
	_pc_plane_now->points.clear();
	_nc->points.clear();
	_dgsphere->points.clear();
	_clusters.clear();
}

void SQLidarPlanarNormalEstimation::print(void)
{
	std::cout << "----------" << std::endl;
	for(size_t i=0;i<_nc->points.size();++i){
		std::cout << "_nc->points[i]: " << _nc->points[i].normal_x << ", " << _nc->points[i].normal_y << ", " << _nc->points[i].normal_z << std::endl;
	}
	for(size_t i=0;i<_nc->points.size();++i){
		std::cout << "_dgsphere->points[i]: " << _dgsphere->points[i].x << ", " << _dgsphere->points[i].y << ", " << _dgsphere->points[i].z << std::endl;
	}
	for(size_t i=0;i<_clusters.size();++i){
		std::cout << "_clusters[i]: " << _clusters[i](0) << ", " << _clusters[i](1) << ", " << _clusters[i](2) << std::endl;
	}
	std::cout << "_g: " << _g(0) << ", " << _g(1) << ", " << _g(2) << std::endl;
}

void SQLidarPlanarNormalEstimation::erasePoints()
{
	if(_list_num_points.size() > _limit_store_scan){
		_pc_plane_last->points.erase(_pc_plane_last->points.begin(), _pc_plane_last->points.begin() + _list_num_points[0]);
		_pc_plane_last->width = _pc_plane_last->points.size();
		_pc_plane_last->height = 1;
		_list_num_points.erase(_list_num_points.begin());
	}
}

bool SQLidarPlanarNormalEstimation::transformPCWithTF(const sensor_msgs::PointCloud2& pc2_in, sensor_msgs::PointCloud2& pc2_out, std::string target_frame, ros::Time target_stamp)
{
	sensor_msgs::PointCloud pc1_in;
	sensor_msgs::PointCloud pc1_out;

	sensor_msgs::convertPointCloud2ToPointCloud(pc2_in, pc1_in);

	try{
		_tflistener.waitForTransform(target_frame, pc2_in.header.frame_id, target_stamp, ros::Duration(1.0));
		_tflistener.transformPointCloud(target_frame, target_stamp, pc1_in, pc2_in.header.frame_id, pc1_out);
		sensor_msgs::convertPointCloudToPointCloud2(pc1_out, pc2_out);
		return true;
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return false;
	}
}

void SQLidarPlanarNormalEstimation::transformPCWithOdom(pcl::PointCloud<pcl::PointNormal>::Ptr pc_inout, nav_msgs::Odometry odom_last, nav_msgs::Odometry odom_now)
{
	tf::Quaternion q_now;
	tf::Quaternion q_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_now);
	quaternionMsgToTF(odom_last.pose.pose.orientation, q_last);
	tf::Quaternion q_rel_rot = q_last*q_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(
		q_rel_rot.w(),
		q_rel_rot.x(),
		q_rel_rot.y(),
		q_rel_rot.z()
	);
	tf::Quaternion q_global_move(
		odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_last.inverse()*q_global_move*q_last;
	Eigen::Vector3f offset(
		q_local_move.x(),
		q_local_move.y(),
		q_local_move.z()
	);
	pcl::transformPointCloudWithNormals(*pc_inout, *pc_inout, offset, rotation);
}

void SQLidarPlanarNormalEstimation::eraseNanPoint(void)
{
	for(size_t i=0;i<_pc->points.size();){
		double depth = norm(_pc->points[i].x, _pc->points[i].y, _pc->points[i].z);
		if(depth > _laser_range || std::isnan(depth)){
			_pc->points.erase(_pc->points.begin() + i);
		}
		else	++i;
	}
	_pc->width = _pc->points.size();
}

void SQLidarPlanarNormalEstimation::extractWall(void)
{
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	if(!_pc_plane_last->points.empty())	kdtree.setInputCloud(_pc_plane_last);

	for(size_t i=0;i<_pc->points.size();++i){
		if(i < (size_t)_curvature_region || i >= _pc->points.size()-_curvature_region){
			/* _pc->points[i].curvature = std::nan(""); */
			_pc->points[i].strength = -1;
		}
		else{
			Eigen::Vector3d v_sum_diff = Eigen::Vector3d::Zero();
			Eigen::Vector3d v_abs_diff = Eigen::Vector3d::Zero();
			for(size_t j=i-_curvature_region;j<=i+_curvature_region;++j){
				if(i==j)	continue;

				Eigen::Vector3d v_diff(
					_pc->points[i].x - _pc->points[j].x,
					_pc->points[i].y - _pc->points[j].y,
					_pc->points[i].z - _pc->points[j].z
				);
				v_sum_diff += v_diff;

				if(j < i)	v_abs_diff -= v_diff;
				if(j > i)	v_abs_diff += v_diff;
			}
			double depth = norm(_pc->points[i].x, _pc->points[i].y, _pc->points[i].z);
			_pc->points[i].strength = v_sum_diff.norm()/(depth*depth*2.0*_curvature_region);
			v_abs_diff.normalize();

			if(_pc->points[i].strength < _th_flatness){
				pcl::PointNormal tmp_n;
				tmp_n.x = _pc->points[i].x;
				tmp_n.y = _pc->points[i].y;
				tmp_n.z = _pc->points[i].z;
				tmp_n.curvature = _pc->points[i].strength;
				tmp_n.normal_x = v_abs_diff(0);
				tmp_n.normal_y = v_abs_diff(1);
				tmp_n.normal_z = v_abs_diff(2);
				_pc_plane_now->points.push_back(tmp_n);

				if(!_pc_plane_last->points.empty()){
					if(estimateNormal(kdtree, tmp_n)){
						if(isVerticalPlane(tmp_n)){
							_nc->points.push_back(tmp_n);
							pcl::PointXYZ tmp_p;
							dGaussMap(tmp_n, tmp_p);
							_dgsphere->points.push_back(tmp_p);
							addToClusters(tmp_p);
						}
					}
				}
			}
		}
		/* std::cout << "_nc->points.size() = " << _nc->points.size() << std::endl; */
		/* std::cout << "_pc->points[i].strength = " << _pc->points[i].strength << std::endl; */
	}
}

void SQLidarPlanarNormalEstimation::eraseSmallCluster(void)
{
	for(size_t i=0;i<_clusters.size();){
		if(_clusters[i].norm() < _min_cluster_length)	_clusters.erase(_clusters.begin() + i);
		else	++i;
	}
}

bool SQLidarPlanarNormalEstimation::estimateNormal(pcl::KdTreeFLANN<pcl::PointNormal>& kdtree, pcl::PointNormal& n)
{
	const int k = 1;
	std::vector<int> list_index;
	std::vector<float> list_squareddist;
	kdtree.nearestKSearch(n, k, list_index, list_squareddist);
	/* std::cout << "list_squareddist[0] = " << list_squareddist[0] << std::endl; */
	if(list_squareddist[0] >  _th_assoc_squareddist)	return false;
	Eigen::Vector3d v_now(
		n.normal_x,
		n.normal_y,
		n.normal_z
	);
	Eigen::Vector3d v_last(
		_pc_plane_last->points[list_index[0]].normal_x,
		_pc_plane_last->points[list_index[0]].normal_y,
		_pc_plane_last->points[list_index[0]].normal_z
	);
	Eigen::Vector3d v_rel(
		n.x - _pc_plane_last->points[list_index[0]].x,
		n.y - _pc_plane_last->points[list_index[0]].y,
		n.z - _pc_plane_last->points[list_index[0]].z
	);
	double cross_angle = angleBetweenVectors(v_rel.cross(v_now), v_rel.cross(v_last));
	if(cross_angle/M_PI*180.0 > _th_assoc_crossangle_deg)	return false;
	Eigen::Vector3d v_normal = (v_rel.cross(v_now) + v_rel.cross(v_last)).normalized();
	/*flip*/
	flipNormalTowardsViewpoint(n, 0.0, 0.0, 0.0, v_normal);
	/*input*/
	n.normal_x = v_normal(0);
	n.normal_y = v_normal(1);
	n.normal_z = v_normal(2);
	return true;
}

bool SQLidarPlanarNormalEstimation::isVerticalPlane(pcl::PointNormal n)
{
	Eigen::Vector3d v_n(n.normal_x, n.normal_y, n.normal_z);
	double angle = angleBetweenVectors(v_n, _g);
	double angle_from_horizon = std::abs(angle - M_PI/2.0);
	if(angle_from_horizon/M_PI*180.0 < _th_angle_from_horizon_deg)	return true;
	else	return false;
}

void SQLidarPlanarNormalEstimation::dGaussMap(const pcl::PointNormal& n, pcl::PointXYZ& p)
{
	double dot = n.x*n.normal_x + n.y*n.normal_y + n.z*n.normal_z;
	double n_norm = norm(n.normal_x, n.normal_y, n.normal_z);
	double depth = dot/n_norm;
	p.x = -std::abs(depth)*n.data_n[0];
	p.y = -std::abs(depth)*n.data_n[1];
	p.z = -std::abs(depth)*n.data_n[2];
}

void SQLidarPlanarNormalEstimation::addToClusters(pcl::PointXYZ p)
{
	Eigen::Vector3d v_p(p.x, p.y, p.z);
	for(size_t i=0;i<_clusters.size();++i){
		double angle1 = angleBetweenVectors(v_p, _clusters[i]);
		if(angle1/M_PI*180.0 < _th_clustering_angle_deg){
			_clusters[i] += v_p;
			return;
		}
		double angle2 = angleBetweenVectors(-v_p, _clusters[i]);
		if(angle2/M_PI*180.0 < _th_clustering_angle_deg){
			_clusters[i] -= v_p;
			return;
		}
	}
	_clusters.push_back(v_p);
}

bool SQLidarPlanarNormalEstimation::estimateG(void)
{
	Eigen::Vector3d g_new(0.0, 0.0, 0.0);

	if(_clusters.size() == 0)	return false;
	else if(_clusters.size() == 1){
		g_new = _g - _g.dot(_clusters[0])/_clusters[0].norm()/_clusters[0].norm()*_clusters[0];
		flipG(_g, g_new);
	}
	else{
		for(size_t i=0;i<_clusters.size()-1;++i){
			for(size_t j=i+1;j<_clusters.size();++j){
				Eigen::Vector3d cross = _clusters[i].cross(_clusters[j]);
				std::cout << "_clusters[i]: " << _clusters[i](0) << ", " << _clusters[i](1) << ", " << _clusters[i](2) << std::endl;
				std::cout << "_clusters[j]: " << _clusters[j](0) << ", " << _clusters[j](1) << ", " << _clusters[j](2) << std::endl;
				std::cout << "cross: " << cross(0) << ", " << cross(1) << ", " << cross(2) << std::endl;
				flipG(_g, cross);
				g_new += cross;
			}
		}
	}
	g_new.normalize();
	/*print RP*/
	printGToRP(g_new);
	/*final judge*/
	if(angleBetweenVectors(_g, g_new)/M_PI*180.0 < _th_anglediff_gnew_glast_deg){
		_g = g_new;
		std::cout << "g_new: " << g_new(0) << ", " << g_new(1) << ", " << g_new(2) << std::endl;
		return true;
	}
	else{
		std::cout << "angleBetweenVectors(_g, g_new)/M_PI*180.0 = " << angleBetweenVectors(_g, g_new)/M_PI*180.0 << " > " << _th_anglediff_gnew_glast_deg << std::endl;
		std::cout << "g_new: " << g_new(0) << ", " << g_new(1) << ", " << g_new(2) << std::endl;
		// exit(1);
		return false;
	}
}

void SQLidarPlanarNormalEstimation::publication(ros::Time stamp, bool g_is_available)
{
	/*pc*/
	if(!_pc->points.empty()){
		sensor_msgs::PointCloud2 msg_pc;
		pcl::toROSMsg(*_pc, msg_pc);
		msg_pc.header.frame_id = _pubmsg_frame;
		_pub_pc.publish(msg_pc);
	}
	/*plane*/
	// if(!_pc_plane_now->points.empty()){
	// 	sensor_msgs::PointCloud2 msg_pc_plane;
	// 	pcl::toROSMsg(*_pc_plane_now, msg_pc_plane);
	// 	msg_pc_plane.header.frame_id = _pubmsg_frame;
	// 	msg_pc_plane.header.stamp = stamp;
	// 	_pub_plane.publish(msg_pc_plane);
	// }
	if(!_pc_plane_last->points.empty()){
		sensor_msgs::PointCloud2 msg_pc_plane;
		pcl::toROSMsg(*_pc_plane_last, msg_pc_plane);
		msg_pc_plane.header.frame_id = _pubmsg_frame;
		msg_pc_plane.header.stamp = stamp;
		_pub_plane.publish(msg_pc_plane);
	}
	/*nc*/
	if(!_nc->points.empty()){
		sensor_msgs::PointCloud2 msg_nc;
		pcl::toROSMsg(*_nc, msg_nc);
		msg_nc.header.frame_id = _pubmsg_frame;
		msg_nc.header.stamp = stamp;
		_pub_nc.publish(msg_nc);
	}
	/*nc*/
	if(!_dgsphere->points.empty()){
		sensor_msgs::PointCloud2 msg_gsphere;
		pcl::toROSMsg(*_dgsphere, msg_gsphere);
		msg_gsphere.header.frame_id = _pubmsg_frame;
		msg_gsphere.header.stamp = stamp;
		_pub_gsphere.publish(msg_gsphere);
	}
	/*gravity Vector*/
	if(g_is_available){
		geometry_msgs::Vector3Stamped msg_g;
		msg_g.header.frame_id = _pubmsg_frame;
		msg_g.header.stamp = stamp;
		msg_g.vector.x = _g(0);
		msg_g.vector.y = _g(1);
		msg_g.vector.z = _g(2);
		_pub_g.publish(msg_g);
	}
}

void SQLidarPlanarNormalEstimation::visualization(void)
{
	_viewer.removeAllPointClouds();

	/*_pc*/
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::InterestPoint> intensity_distribution(_pc, "strength"); 
	_viewer.addPointCloud<pcl::InterestPoint>(_pc, intensity_distribution, "_pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "_pc");
	/* _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "_pc"); */
	/*_pc_plane_now*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_pc_plane_now, 1, 0.5, "_pc_plane_now");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "_pc_plane_now");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "_pc_plane_now");
	/*_nc*/
	_viewer.addPointCloudNormals<pcl::PointNormal>(_nc, 1, 0.5, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "_nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "_nc");
	/*_dgsphere*/
	_viewer.addPointCloud(_dgsphere, "_dgsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "_dgsphere");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "_dgsphere");
	/*gravity vector*/
	pcl::PointCloud<pcl::PointNormal>::Ptr g {new pcl::PointCloud<pcl::PointNormal>};
	pcl::PointNormal tmp;
	tmp.x = 0.0;
	tmp.x = 0.0;
	tmp.x = 0.0;
	tmp.normal_x = _g(0);
	tmp.normal_y = _g(1);
	tmp.normal_z = _g(2);
	g->points.push_back(tmp);
	_viewer.addPointCloudNormals<pcl::PointNormal>(g, 1, 1.0, "g");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "g");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "g");

	_viewer.spinOnce();
}

double SQLidarPlanarNormalEstimation::norm(double x, double y, double z)
{
	return sqrt(x*x + y*y + z*z);
}

double SQLidarPlanarNormalEstimation::angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
	return acos(v1.dot(v2)/v1.norm()/v2.norm());
}

void SQLidarPlanarNormalEstimation::flipToHalfSphere(Eigen::Vector3d& v)
{
	if(v(0) < 0)	v *= -1;
}

void SQLidarPlanarNormalEstimation::flipG(const Eigen::Vector3d& g_last, Eigen::Vector3d& g_new)
{
	const double th_flip_angle_deg = 90.0;
	double angle = angleBetweenVectors(g_last, g_new);
	if(angle/M_PI*180.0 > th_flip_angle_deg)	g_new *= -1;
}

void SQLidarPlanarNormalEstimation::printGToRP(const Eigen::Vector3d& g)
{
	Eigen::Vector3d inv_g = -g;
	double r = atan2(inv_g(1), inv_g(2));
	r = atan2(sin(r), cos(r));
	double p = atan2(-inv_g(0), sqrt(inv_g(1)*inv_g(1) + inv_g(2)*inv_g(2)));
	p = atan2(sin(p), cos(p));

	std::cout
		<< "r[deg]: " << r/M_PI*180.0
		<< ", "
		<< "p[deg]: " << p/M_PI*180.0
	<< std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sqlidar_planar_normal_estimation");
	
	SQLidarPlanarNormalEstimation sqlidar_gravity_estimation;

	ros::spin();
}
