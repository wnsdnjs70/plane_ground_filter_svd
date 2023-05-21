#include "plane_ground_filter_core.h"

bool point_cmp(VPoint a, VPoint b)
{
    return a.z < b.z;
}
 
PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
    std::string input_topic;
    nh.getParam("input_topic", input_topic);
    
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 1, &PlaneGroundFilter::point_cb, this);


    // init publisher
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);
    
//track parameter
    nh.getParam("x_min_t", x_min_);
    ROS_INFO("x_min_t: %f", x_min_);
    nh.getParam("x_max_t", x_max_);
    ROS_INFO("x_max_t: %f", x_max_);
    nh.getParam("y_min_t", y_min_);
    ROS_INFO("y_min_t: %f", y_min_);
    nh.getParam("y_max_t", y_max_);
    ROS_INFO("y_max_t: %f", y_max_);
    nh.getParam("z_min_t", z_min_);
    ROS_INFO("z_min_t: %f", z_min_);
    nh.getParam("z_max_t", z_max_);
    ROS_INFO("z_max_t: %f", z_max_);

    
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);

	ros::spin();
}

PlaneGroundFilter::~PlaneGroundFilter() {}

void PlaneGroundFilter::estimate_plane_(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
}

//main_code
void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<VPoint>::Ptr laserCloudIn (new pcl::PointCloud<VPoint>);
    
    pcl::fromROSMsg(*in_cloud_ptr, *laserCloudIn);

    //voxel
    pcl::VoxelGrid<VPoint> vg;

    vg.setInputCloud (laserCloudIn);
	vg.setLeafSize (0.1f, 0.1f, 0.1f); // grid size: 5cm
	vg.filter (*laserCloudIn);

    pcl::PassThrough<VPoint> pass;
    pass.setInputCloud(laserCloudIn);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits(x_min_*cos(15*M_PI/180)-z_min_*sin(15*M_PI/180)+1.1*sin(15*M_PI/180),x_max_*cos(15*M_PI/180)-z_max_*sin(15*M_PI/180)+1.1*sin(15*M_PI/180)); 
	pass.filter(*laserCloudIn);
		
	pass.setInputCloud(laserCloudIn);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y_min_, y_max_);
	pass.filter(*laserCloudIn);
	
	pass.setInputCloud(laserCloudIn);  
	pass.setFilterFieldName("z");
	pass.setFilterLimits(x_min_*sin(15*M_PI/180)+z_min_*cos(15*M_PI/180)-1.1*cos(15*M_PI/180),x_max_*sin(15*M_PI/180)+z_max_*cos(15*M_PI/180)-1.1*cos(15*M_PI/180)); 
	pass.filter(*laserCloudIn);


    //=============================================Plane_Ground Filter==================================================================//
    pcl::PointCloud<VPoint> laserCloudIn_org;
    SLRPointXYZIRL point;  
    float x;
    float z;

    for (size_t i = 0; i < laserCloudIn->points.size(); i++)
    {
    	x = laserCloudIn->points[i].x*cos(15*M_PI/180) + laserCloudIn->points[i].z*sin(15*M_PI/180);  
    	z = -laserCloudIn->points[i].x*sin(15*M_PI/180) + laserCloudIn->points[i].z*cos(15*M_PI/180)+1.1; 
        point.x = x;
        point.y = laserCloudIn->points[i].y;
        point.z = z;
        laserCloudIn->points[i].x=x;
        laserCloudIn->points[i].z=z;
        point.intensity = laserCloudIn->points[i].intensity;
        point.ring = laserCloudIn->points[i].ring;
        point.label = 0u; // 0 means uncluster
        g_all_pc->points.push_back(point);
    }

    laserCloudIn_org = *laserCloudIn;

    pcl::PointCloud<VPoint>::iterator it = laserCloudIn->points.begin();
    for (int i = 0; i < laserCloudIn->points.size(); i++)
    {
        if (laserCloudIn->points[i].z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn->points.erase(laserCloudIn->points.begin(), it);

    g_ground_pc = g_seeds_pc;

    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                g_all_pc->points[r].label = 1u; // means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                g_all_pc->points[r].label = 0u; // means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_ptr->header.stamp;
    ground_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_ground_.publish(ground_msg);

    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*g_not_ground_pc, groundless_msg); 
    groundless_msg.header.stamp = in_cloud_ptr->header.stamp;
    groundless_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_ptr->header.stamp;
    all_points_msg.header.frame_id = in_cloud_ptr->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();

}
