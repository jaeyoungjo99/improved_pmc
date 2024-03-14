#include "range_pmc_wrapper.hpp"

RangePmcWrapper::RangePmcWrapper(){
    Init();
};
RangePmcWrapper::~RangePmcWrapper(){};

void RangePmcWrapper::Init(){
    ROS_INFO("Init function");
    NodeHandle nh;
    
    range_pmc_params params;

    cfg_vec_f_ego_to_lidar_.resize(3);

    if (!nh.getParam("/topic_name/vehicle_state", cfg_str_odom_topic_name_)) cfg_str_odom_topic_name_ = "";
    if (!nh.getParam("/common_variable/dataset", cfg_str_dataset_)) cfg_str_dataset_ = "novatel";
    if (!nh.getParam("/common_variable/lidar_topic_name", cfg_str_lidar_topic_name_)) cfg_str_lidar_topic_name_ = "";
    if (!nh.getParam("/common_variable/lidar_type", cfg_str_lidar_type_)) cfg_str_lidar_type_ = "velodyne";
    if (!nh.getParam("/common_variable/ego_to_lidar_rotation/x", cfg_vec_f_ego_to_lidar_[0])) cfg_vec_f_ego_to_lidar_[0] = 1.2;
    if (!nh.getParam("/common_variable/ego_to_lidar_rotation/y", cfg_vec_f_ego_to_lidar_[1])) cfg_vec_f_ego_to_lidar_[1] = 0.0;
    if (!nh.getParam("/common_variable/ego_to_lidar_rotation/z", cfg_vec_f_ego_to_lidar_[2])) cfg_vec_f_ego_to_lidar_[2] = 1.8;

    if (!nh.getParam("/range_pmc/deskewing", cfg_b_deskewing_)) cfg_b_deskewing_ = false;
    if (!nh.getParam("/range_pmc/horizontal_resolution", cfg_f_horizontal_resolution_)) cfg_f_horizontal_resolution_ = 1.0;
    if (!nh.getParam("/range_pmc/vertical_resolution", cfg_f_vertical_resolution_)) cfg_f_vertical_resolution_ = 1.0;
    if (!nh.getParam("/range_pmc/min_range", cfg_f_min_range_)) cfg_f_min_range_ = 1.0;
    if (!nh.getParam("/range_pmc/max_range", cfg_f_max_range_))  cfg_f_max_range_ = 100.0;
    if (!nh.getParam("/range_pmc/max_points_in_pixel", cfg_i_max_points_in_pixel_))  cfg_i_max_points_in_pixel_ = 100;
    if (!nh.getParam("/range_pmc/range_threshold", cfg_f_range_threshold_))  cfg_f_range_threshold_ = 0.1;
    if (!nh.getParam("/range_pmc/max_range_image_num", cfg_i_max_range_image_num_)) cfg_i_max_range_image_num_ = 10;
    if (!nh.getParam("/range_pmc/min_occluded_num", cfg_i_min_occluded_num_)) cfg_i_min_occluded_num_ = 5;
    if (!nh.getParam("/range_pmc/min_key_frame_time", cfg_f_min_key_frame_time_))cfg_f_min_key_frame_time_ = 0.1;
    if (!nh.getParam("/range_pmc/min_key_frame_rot", cfg_f_min_key_frame_rot_)) cfg_f_min_key_frame_rot_ = 1; 
    if (!nh.getParam("/range_pmc/fov_up", cfg_f_fov_up_)) cfg_f_fov_up_ = 90;
    if (!nh.getParam("/range_pmc/fov_down", cfg_f_fov_down_)) cfg_f_fov_down_ = -90;
    if (!nh.getParam("/range_pmc/fov_left", cfg_f_fov_left_)) cfg_f_fov_left_ = 180;
    if (!nh.getParam("/range_pmc/fov_right", cfg_f_fov_right_)) cfg_f_fov_right_ = -180;
    if (!nh.getParam("/range_pmc/neighbor_pixel_max", cfg_i_neighbor_pixel_max_)) cfg_i_neighbor_pixel_max_ = 3;
    if (!nh.getParam("/range_pmc/height_filter", cfg_b_height_filter_)) cfg_b_height_filter_ = false;

    if (!nh.getParam("/range_pmc/ground_angle", cfg_f_ground_angle_)) cfg_f_ground_angle_ = 0.0;
    if (!nh.getParam("/range_pmc/dist_threshold_m", cfg_f_dist_threshold_m_)) cfg_f_dist_threshold_m_ = 1.0;
    if (!nh.getParam("/range_pmc/segment_min_point_num", cfg_i_segment_min_point_num_)) cfg_i_segment_min_point_num_ = 100;
    if (!nh.getParam("/range_pmc/segment_valid_poiint_num", cfg_i_segment_valid_poiint_num_)) cfg_i_segment_valid_poiint_num_ = 10;
    if (!nh.getParam("/range_pmc/segment_valid_line_num", cfg_i_segment_valid_line_num_)) cfg_i_segment_valid_line_num_ = 3;
    if (!nh.getParam("/range_pmc/cluster_level_filtering", cfg_b_cluster_level_filtering_)) cfg_b_cluster_level_filtering_ = false;
    if (!nh.getParam("/range_pmc/moving_confidence", cfg_f_moving_confidence_)) cfg_f_moving_confidence_ = 0.5;
    if (!nh.getParam("/range_pmc/static_confidence", cfg_f_static_confidence_)) cfg_f_static_confidence_ = 0.5;
    if (!nh.getParam("/range_pmc/gaussian_sigma", cfg_f_gaussian_sigma_)) cfg_f_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/static_gaussian_sigma", cfg_f_static_gaussian_sigma_)) cfg_f_static_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/dynamic_gaussian_sigma", cfg_f_dynamic_gaussian_sigma_)) cfg_f_dynamic_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/sigma_epsilon", cfg_f_sigma_epsilon_)) cfg_f_sigma_epsilon_ = 0.1;

    if (!nh.getParam("/range_pmc/output_static_point", cfg_b_output_static_point_)) cfg_b_output_static_point_ = false;
    if (!nh.getParam("/range_pmc/output_min_range", cfg_b_output_min_range_)) cfg_b_output_min_range_ = false;
    if (!nh.getParam("/range_pmc/debug_image", cfg_b_debug_image_)) cfg_b_debug_image_ = false;

    std::cout<<"Cal: "<<cfg_vec_f_ego_to_lidar_[0]<<" "<<cfg_vec_f_ego_to_lidar_[1]<<" "<<cfg_vec_f_ego_to_lidar_[2]<<std::endl;


    params.f_horizontal_resolution = cfg_f_horizontal_resolution_ * M_PI/180.0;
    params.f_vertical_resolution = cfg_f_vertical_resolution_ * M_PI/180.0;
    params.f_min_range = cfg_f_min_range_;
    params.f_max_range = cfg_f_max_range_;
    params.i_max_points_in_pixel = cfg_i_max_points_in_pixel_;
    params.f_range_threshold = cfg_f_range_threshold_;
    params.i_max_range_image_num = cfg_i_max_range_image_num_;
    params.f_min_key_frame_time = cfg_f_min_key_frame_time_;
    params.f_min_key_frame_rot = cfg_f_min_key_frame_rot_;

    params.f_fov_up = cfg_f_fov_up_;
    params.f_fov_down = cfg_f_fov_down_;
    params.f_fov_left = cfg_f_fov_left_;
    params.f_fov_right = cfg_f_fov_right_;
    params.i_min_occluded_num = cfg_i_min_occluded_num_;

    params.vec_f_ego_to_lidar = cfg_vec_f_ego_to_lidar_;
    params.i_neighbor_pixel_max = cfg_i_neighbor_pixel_max_;
    params.f_ground_angle = cfg_f_ground_angle_;
    params.f_dist_threshold_m = cfg_f_dist_threshold_m_;
    params.i_segment_min_point_num = cfg_i_segment_min_point_num_;
    params.i_segment_valid_point_num = cfg_i_segment_valid_poiint_num_;
    params.i_segment_valid_line_num = cfg_i_segment_valid_line_num_;

    params.f_moving_confidence = cfg_f_moving_confidence_;
    params.f_static_confidence = cfg_f_static_confidence_;
    params.f_gaussian_sigma = cfg_f_gaussian_sigma_;
    params.f_static_gaussian_sigma = cfg_f_static_gaussian_sigma_;
    params.f_dynamic_gaussian_sigma = cfg_f_dynamic_gaussian_sigma_;
    params.f_sigma_epsilon = cfg_f_sigma_epsilon_;

    params.b_cluster_level_filtering = cfg_b_cluster_level_filtering_;
    params.b_output_static_point = cfg_b_output_static_point_;
    params.b_output_min_range = cfg_b_output_min_range_;
    params.b_debug_image = cfg_b_debug_image_;


    sub_point_cloud_        = nh.subscribe(cfg_str_lidar_topic_name_, 1, &RangePmcWrapper::CallbackPointCloud, this);

    if(cfg_str_dataset_ == "novatel")
        sub_odometry_           = nh.subscribe(cfg_str_odom_topic_name_, 1, &RangePmcWrapper::CallbackOdometry, this);
    else
        sub_odometry_           = nh.subscribe("ground_truth", 1, &RangePmcWrapper::CallbackKittiGeo, this);

    pub_pmc_point_cloud_    = nh.advertise<sensor_msgs::PointCloud2>("app/perc/pmc_point_cloud", 10);
    pub_key_frame_point_cloud_  = nh.advertise<sensor_msgs::PointCloud2>("app/perc/key_frame_point_cloud", 10);
    pub_cluster_point_cloud_    = nh.advertise<sensor_msgs::PointCloud2>("app/perc/cluster_point_cloud", 10);

    pub_range_image_        = nh.advertise<sensor_msgs::Image>("app/perc/range_image", 10);
    pub_incident_image_     = nh.advertise<sensor_msgs::Image>("app/perc/incident_image", 10);
    pub_dynamic_image_      = nh.advertise<sensor_msgs::Image>("app/perc/dynamic_image", 10);
    pub_ground_image_       = nh.advertise<sensor_msgs::Image>("app/perc/ground_image", 10);
    pub_cluster_image_      = nh.advertise<sensor_msgs::Image>("app/perc/cluster_image", 10);
    pub_angle_image_        = nh.advertise<sensor_msgs::Image>("app/perc/angle_image", 10);

    i_tmp_ouster_cloud_ptr_.reset(new pcl::PointCloud<OusterPointXYZIRT>());
    i_tmp_kitti_cloud_ptr_.reset(new pcl::PointCloud<PointXYZIRGBRTLIS>());
    i_xyzirt_point_cloud_ptr_.reset(new pcl::PointCloud<PointXYZIRT>());
    i_xyzirt_point_cloud_deskwed_ptr_.reset(new pcl::PointCloud<PointXYZIRT>());
    i_xyzin_point_cloud_ptr_.reset(new PointCloudXYZI());
    o_pmc_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    o_key_frame_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    o_cluster_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());


    

    RangePmcPtr->Init(params);

    ROS_INFO("Init function Done");

}

void RangePmcWrapper::Run(){

    ros::spin();
}


void RangePmcWrapper::Publish(){
    pub_pmc_point_cloud_.publish(o_pmc_point_cloud_msg_);
    pub_key_frame_point_cloud_.publish(o_key_frame_point_cloud_msg_);
    pub_cluster_point_cloud_.publish(o_cluster_point_cloud_msg_);
    pub_range_image_.publish(o_range_image_msg_);
    pub_incident_image_.publish(o_incident_image_msg_);
    pub_dynamic_image_.publish(o_dynamic_image_msg_);
    pub_ground_image_.publish(o_ground_image_msg_);
    pub_cluster_image_.publish(o_cluster_image_msg_);
    pub_angle_image_.publish(o_angle_image_msg_);
}

void RangePmcWrapper::UpdatePmcPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_pmc_point_cloud_msg_);
    o_pmc_point_cloud_msg_.header.frame_id = str_main_lidar_frame_id_;
    o_pmc_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

void RangePmcWrapper::UpdateKeyFramePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_key_frame_point_cloud_msg_);
    o_key_frame_point_cloud_msg_.header.frame_id = "world";
    o_key_frame_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

void RangePmcWrapper::UpdateClusterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_cluster_point_cloud_msg_);
    o_cluster_point_cloud_msg_.header.frame_id = str_main_lidar_frame_id_;
    o_cluster_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

bool RangePmcWrapper::RunDeskewing(pcl::PointCloud<PointXYZIRT>::Ptr i_point_cloud,
                pcl::PointCloud<PointXYZIRT>::Ptr& o_deskewed_point_cloud)
{   
    double tout_start = omp_get_wtime();

    o_deskewed_point_cloud->clear();
    int cloudSize = i_point_cloud->points.size();

    if(cloudSize == 0 || deque_time_lidar_pose_.size() < 2){
        std::cout<<"[RunDeskewing] Data empty. CloudSize: "<< cloudSize <<" deque_time_lidar_pose_ "<<deque_time_lidar_pose_.size()<<std::endl;
        return false;
    }

    double time_compensate_sec = 0.0;
    if(cfg_str_lidar_type_ == "velodyne") time_compensate_sec = 0.2;

    d_time_scan_cur_ = i_point_cloud->header.stamp * 1e-6 - time_compensate_sec; // pcl header time is micro sec
    d_time_scan_end_ = i_point_cloud->header.stamp * 1e-6 + i_point_cloud->points.back().time - time_compensate_sec;

    std::cout<<"i_point_cloud->points.back().time: "<<i_point_cloud->points.back().time<<std::endl;

    std::cout<<"d_time_scan_cur_: "<<d_time_scan_cur_<<std::endl;
    std::cout<<"d_time_scan_end_: "<<d_time_scan_end_<<std::endl;
    std::cout<<"IMU time start  : "<<deque_time_lidar_pose_.front().first<<std::endl;
    std::cout<<"IMU time back   : "<<deque_time_lidar_pose_.back().first<<std::endl;

    // Find end vehicle state
    if (deque_time_lidar_pose_.back().first < d_time_scan_cur_){
        std::cout<<"[RunDeskewing] IMU Data Too late"<<std::endl;
        return false;
    }

    // Find start vehicle state
    bool find_time_cur = false;
    bool find_time_end = false;

    for (int i = 0; i < (int)deque_time_lidar_pose_.size(); ++i)
    {
        if (deque_time_lidar_pose_[i].first < d_time_scan_cur_){
            scan_start_lidar_time_pose_ = deque_time_lidar_pose_[i];
            find_time_cur = true;
            continue;
        }
        else{
            break; // if vehicle state time is bigger than scan time
        }
    }
    for (int i = 0; i < (int)deque_time_lidar_pose_.size(); ++i)
    {

        if (deque_time_lidar_pose_[i].first < d_time_scan_end_){
            scan_end_lidar_time_pose_ = deque_time_lidar_pose_[i];
            find_time_end = true;
            continue;
        }
        else{
            break;
        }
    }

    if(find_time_cur == false || find_time_end == false){
        std::cout<<"[RunDeskewing] Cannot Find Time zone"<<std::endl;
        return false;
    }

    // lidar_synced_pose_ = scan_start_lidar_time_pose_.second;
    // cur_time = scan_start_lidar_time_pose_.first;

    

    // Make Odom incre
    Eigen::Affine3f transBt = scan_start_lidar_time_pose_.second.inverse() * scan_end_lidar_time_pose_.second;

    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    double d_scan_delta_time_sec = d_time_scan_end_ - d_time_scan_cur_;
    double d_vehicle_state_delta_time_sec = scan_end_lidar_time_pose_.first - scan_start_lidar_time_pose_.first;
    double d_pos_start_to_scan_cur_time_sec = d_time_scan_cur_ - scan_start_lidar_time_pose_.first;
    cur_time = d_time_scan_cur_;

    std::cout<<"[RunDeskewing] d_pos_start_to_scan_cur_time_sec "<<d_pos_start_to_scan_cur_time_sec<<std::endl;

    if(d_vehicle_state_delta_time_sec < 0.01){
        std::cout.precision(20);
        std::cout<<"[RunDeskewing] Vehicle state for interpolation is too tight"<<std::endl;
        std::cout<<"[RunDeskewing] start_pos_time: "<< scan_start_lidar_time_pose_.first<<std::endl;
        std::cout<<"[RunDeskewing] end_pos_time  : "<< scan_end_lidar_time_pose_.first<<std::endl;
        return false;
    }


    // fit time gap difference
    odomIncreX  *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;
    odomIncreY  *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;
    odomIncreZ  *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;
    rollIncre   *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;
    pitchIncre  *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;
    yawIncre    *= d_scan_delta_time_sec / d_vehicle_state_delta_time_sec;

    odomIncreXs2p  = odomIncreX *   d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;
    odomIncreYs2p  = odomIncreY *   d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;
    odomIncreZs2p  = odomIncreZ *   d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;
    rollIncres2p   = rollIncre *    d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;
    pitchIncres2p  = pitchIncre *   d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;
    yawIncres2p    = yawIncre *     d_pos_start_to_scan_cur_time_sec / d_scan_delta_time_sec;

    Eigen::Affine3f pos_start_to_cur = pcl::getTransformation(odomIncreXs2p, odomIncreYs2p, odomIncreZs2p, rollIncres2p, pitchIncres2p, yawIncres2p);

    lidar_synced_pose_ = scan_start_lidar_time_pose_.second * pos_start_to_cur;


    o_deskewed_point_cloud->resize(cloudSize);

    if(cfg_b_deskewing_ == true){
        for(int i = 0; i < cloudSize; ++i){
            double d_rel_time = i_point_cloud->points[i].time;
            DeskewPoint(&i_point_cloud->points[i], &o_deskewed_point_cloud->points[i], d_rel_time);
        }
    }
    else{
        *o_deskewed_point_cloud = *i_point_cloud;
    }




    double tout_end = omp_get_wtime() - tout_start;
    std::cout<<"[Deskewing] Total Time: "<< tout_end*1000.0 << " ms"<<std::endl;
    return true;
    
}

void RangePmcWrapper::DeskewPoint(PointXYZIRT *point, PointXYZIRT *o_point, double d_rel_time){
    double d_point_time = d_time_scan_cur_ + d_rel_time;

    float rotXCur, rotYCur, rotZCur;
    // rotXCur = 0; rotYCur = 0; rotZCur = 0;
    FindRotation(d_rel_time, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    FindPosition(d_rel_time, &posXCur, &posYCur, &posZCur);

    // transform points to start
    Eigen::Affine3f transBt = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);

    o_point->x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    o_point->y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    o_point->z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    o_point->intensity = point->intensity;
    o_point->ring = point->ring;
    o_point->time = point->time;

}

void RangePmcWrapper::FindRotation(double relTime, float *rotXCur, float *rotYCur, float *rotZCur){
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    float ratio = relTime / (d_time_scan_end_ - d_time_scan_cur_);

    *rotXCur = ratio * rollIncre;
    *rotYCur = ratio * pitchIncre;
    *rotZCur = ratio * yawIncre;   
}

void RangePmcWrapper::FindPosition(double relTime, float *posXCur, float *posYCur, float *posZCur){
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    float ratio = relTime / (d_time_scan_end_ - d_time_scan_cur_);

    *posXCur = ratio * odomIncreX;
    *posYCur = ratio * odomIncreY;
    *posZCur = ratio * odomIncreZ;   
}



int main(int argc, char **argv) {
    std::string node_name = "range_pmc";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    RangePmcWrapper NC;

    NC.Run();

    return 0;
}

