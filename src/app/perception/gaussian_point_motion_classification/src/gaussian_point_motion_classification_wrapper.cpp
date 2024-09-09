#include "gaussian_point_motion_classification_wrapper.hpp"

GaussianPointMotionClassiWrapper::GaussianPointMotionClassiWrapper(){
    Init();
};
GaussianPointMotionClassiWrapper::~GaussianPointMotionClassiWrapper(){};

void GaussianPointMotionClassiWrapper::Init(){
    ROS_INFO("Init function");
    NodeHandle nh;
    
    gs_pmc_params params;

    cfg_vec_f_ego_to_lidar_.resize(3);
    cfg_vec_f_ego_to_lidar_rotation_.resize(3);

    if (!nh.getParam("/topic_name/vehicle_state", cfg_str_odom_topic_name_)) cfg_str_odom_topic_name_ = "";
    if (!nh.getParam("/common_variable/dataset", cfg_str_dataset_)) cfg_str_dataset_ = "novatel";
    if (!nh.getParam("/common_variable/lidar_topic_name", cfg_str_lidar_topic_name_)) cfg_str_lidar_topic_name_ = "";
    if (!nh.getParam("/common_variable/lidar_type", cfg_str_lidar_type_)) cfg_str_lidar_type_ = "velodyne";

    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_transform/x", cfg_vec_f_ego_to_lidar_[0])) cfg_vec_f_ego_to_lidar_[0] = 1.2;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_transform/y", cfg_vec_f_ego_to_lidar_[1])) cfg_vec_f_ego_to_lidar_[1] = 0.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_transform/z", cfg_vec_f_ego_to_lidar_[2])) cfg_vec_f_ego_to_lidar_[2] = 1.8;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_rotation/roll", cfg_vec_f_ego_to_lidar_rotation_[0])) cfg_vec_f_ego_to_lidar_rotation_[0] = 0.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_rotation/pitch", cfg_vec_f_ego_to_lidar_rotation_[1])) cfg_vec_f_ego_to_lidar_rotation_[1] = 0.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/ego_to_lidar_rotation/yaw", cfg_vec_f_ego_to_lidar_rotation_[2])) cfg_vec_f_ego_to_lidar_rotation_[2] = 0.0;

    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/horizontal_resolution", cfg_f_horizontal_resolution_)) cfg_f_horizontal_resolution_ = 1.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/vertical_resolution", cfg_f_vertical_resolution_)) cfg_f_vertical_resolution_ = 1.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/max_points_in_pixel", cfg_i_max_points_in_pixel_))  cfg_i_max_points_in_pixel_ = 100;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/min_range", cfg_f_min_range_)) cfg_f_min_range_ = 1.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/max_range", cfg_f_max_range_))  cfg_f_max_range_ = 100.0;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/fov_up", cfg_f_fov_up_)) cfg_f_fov_up_ = 90;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/fov_down", cfg_f_fov_down_)) cfg_f_fov_down_ = -90;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/fov_left", cfg_f_fov_left_)) cfg_f_fov_left_ = 180;
    if (!nh.getParam("/"+ cfg_str_lidar_type_ +"/fov_right", cfg_f_fov_right_)) cfg_f_fov_right_ = -180;

    if (!nh.getParam("/range_pmc/deskewing", cfg_b_deskewing_)) cfg_b_deskewing_ = false;

    if (!nh.getParam("/range_pmc/range_threshold", cfg_f_range_threshold_))  cfg_f_range_threshold_ = 0.1;
    if (!nh.getParam("/range_pmc/max_range_image_num", cfg_i_max_range_image_num_)) cfg_i_max_range_image_num_ = 10;
    if (!nh.getParam("/range_pmc/min_occluded_num", cfg_i_min_occluded_num_)) cfg_i_min_occluded_num_ = 5;
    if (!nh.getParam("/range_pmc/min_key_frame_time", cfg_f_min_key_frame_time_))cfg_f_min_key_frame_time_ = 0.1;
    if (!nh.getParam("/range_pmc/min_key_frame_rot", cfg_f_min_key_frame_rot_)) cfg_f_min_key_frame_rot_ = 1; 

    if (!nh.getParam("/range_pmc/vertical_neighbor_pixel", cfg_i_vertical_neighbor_pixel_)) cfg_i_vertical_neighbor_pixel_ = 3;
    if (!nh.getParam("/range_pmc/horizontal_neighbor_pixel", cfg_i_horizontal_neighbor_pixel_)) cfg_i_horizontal_neighbor_pixel_ = 3;

    if (!nh.getParam("/range_pmc/height_filter", cfg_b_height_filter_)) cfg_b_height_filter_ = false;

    if (!nh.getParam("/range_pmc/ground_angle", cfg_f_ground_angle_)) cfg_f_ground_angle_ = 0.0;
    if (!nh.getParam("/range_pmc/dist_threshold_m", cfg_f_dist_threshold_m_)) cfg_f_dist_threshold_m_ = 1.0;
    if (!nh.getParam("/range_pmc/segment_min_point_num", cfg_i_segment_min_point_num_)) cfg_i_segment_min_point_num_ = 100;
    if (!nh.getParam("/range_pmc/segment_valid_poiint_num", cfg_i_segment_valid_poiint_num_)) cfg_i_segment_valid_poiint_num_ = 10;
    if (!nh.getParam("/range_pmc/segment_valid_line_num", cfg_i_segment_valid_line_num_)) cfg_i_segment_valid_line_num_ = 3;
    if (!nh.getParam("/range_pmc/moving_confidence", cfg_f_moving_confidence_)) cfg_f_moving_confidence_ = 0.5;
    if (!nh.getParam("/range_pmc/static_confidence", cfg_f_static_confidence_)) cfg_f_static_confidence_ = 0.5;
    if (!nh.getParam("/range_pmc/gaussian_sigma", cfg_f_gaussian_sigma_)) cfg_f_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/static_gaussian_sigma", cfg_f_static_gaussian_sigma_)) cfg_f_static_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/dynamic_gaussian_sigma", cfg_f_dynamic_gaussian_sigma_)) cfg_f_dynamic_gaussian_sigma_ = 4;
    if (!nh.getParam("/range_pmc/sigma_epsilon", cfg_f_sigma_epsilon_)) cfg_f_sigma_epsilon_ = 0.1;

    if (!nh.getParam("/range_pmc/ground_filtering", cfg_b_ground_filtering_)) cfg_b_ground_filtering_ = false;
    if (!nh.getParam("/range_pmc/cluster_level_filtering", cfg_b_cluster_level_filtering_)) cfg_b_cluster_level_filtering_ = false;
    if (!nh.getParam("/range_pmc/run_case_2", cfg_b_run_case_2_)) cfg_b_run_case_2_ = false;
    if (!nh.getParam("/range_pmc/run_region_growing", cfg_b_run_region_growing_)) cfg_b_run_region_growing_ = false;
    if (!nh.getParam("/range_pmc/run_small_dynamic_filtering", cfg_b_run_small_dynamic_filtering_)) cfg_b_run_small_dynamic_filtering_ = false;
    if (!nh.getParam("/range_pmc/min_voxel_cluster_num", cfg_i_min_voxel_cluster_num_)) cfg_i_min_voxel_cluster_num_ = 5;

    if (!nh.getParam("/range_pmc/output_static_point", cfg_b_output_static_point_)) cfg_b_output_static_point_ = false;
    if (!nh.getParam("/range_pmc/output_discrete_label", cfg_b_output_discrete_label_)) cfg_b_output_discrete_label_ = false;
    if (!nh.getParam("/range_pmc/output_min_range", cfg_b_output_min_range_)) cfg_b_output_min_range_ = false;
    if (!nh.getParam("/range_pmc/debug_image", cfg_b_debug_image_)) cfg_b_debug_image_ = false;

    if (!nh.getParam("/range_pmc/run_evaluation", cfg_b_run_evaluation_)) cfg_b_run_evaluation_ = false;
    if (!nh.getParam("/range_pmc/evaluation_dataset", cfg_str_evaluation_dataset_)) cfg_str_evaluation_dataset_ = "";

    std::cout<<"Trans: "<<cfg_vec_f_ego_to_lidar_[0]<<" "<<cfg_vec_f_ego_to_lidar_[1]<<" "<<cfg_vec_f_ego_to_lidar_[2]<<std::endl;
    std::cout<<"Rota: "<<cfg_vec_f_ego_to_lidar_rotation_[0]<<" "<<cfg_vec_f_ego_to_lidar_rotation_[1]<<" "<<cfg_vec_f_ego_to_lidar_rotation_[2]<<std::endl;


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
    params.vec_f_ego_to_lidar_rotation = cfg_vec_f_ego_to_lidar_rotation_;

    params.i_vertical_neighbor_pixel = cfg_i_vertical_neighbor_pixel_;
    params.i_horizontal_neighbor_pixel = cfg_i_horizontal_neighbor_pixel_;

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
    
    params.b_ground_filtering = cfg_b_ground_filtering_;
    params.b_cluster_level_filtering = cfg_b_cluster_level_filtering_;
    params.b_run_case_2 = cfg_b_run_case_2_;
    params.b_run_region_growing = cfg_b_run_region_growing_;
    params.b_run_small_dynamic_filtering = cfg_b_run_small_dynamic_filtering_;
    params.i_min_voxel_cluster_num = cfg_i_min_voxel_cluster_num_;

    params.b_output_static_point = cfg_b_output_static_point_;
    params.b_output_discrete_label = cfg_b_output_discrete_label_;
    params.b_output_min_range = cfg_b_output_min_range_;
    params.b_debug_image = cfg_b_debug_image_;


    sub_point_cloud_        = nh.subscribe(cfg_str_lidar_topic_name_, 1, &GaussianPointMotionClassiWrapper::CallbackPointCloud, this);

    if(cfg_str_dataset_ == "novatel")
        sub_odometry_           = nh.subscribe(cfg_str_odom_topic_name_, 1, &GaussianPointMotionClassiWrapper::CallbackOdometry, this);
    else
        sub_odometry_           = nh.subscribe("ground_truth", 1, &GaussianPointMotionClassiWrapper::CallbackKittiGeo, this);

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
    gt_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    o_key_frame_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    o_cluster_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());


    

    GsPmcPtr->Init(params);

    ROS_INFO("Init function Done");

}

void GaussianPointMotionClassiWrapper::Run(){

    // ros::spin();

    std::lock_guard<std::mutex> lock(mutex_point_cloud_);

    if(is_odom_init == false) return;
    if(b_new_lidar_ == false) return;

    // if(fabs(deque_time_lidar_pose_.back().first - lidar_msg_->header.stamp.toSec()) > 0.02) return;

    
    str_main_lidar_frame_id_ = lidar_msg_->header.frame_id;
    main_lidar_time_ = lidar_msg_->header.stamp;
    
    std::cout<<std::endl;
    std::cout.precision(20);

    ROS_WARN_STREAM("PointCloud Callback: "<<main_lidar_time_.toSec());
    
    if(cfg_str_lidar_type_ == "velodyne"){
        main_lidar_time_ -= ros::Duration(0.1);

        pcl::fromROSMsg(*lidar_msg_, *i_xyzirt_point_cloud_ptr_);
        // pcl::fromROSMsg(*lidar_msg_, *i_xyzin_point_cloud_ptr_);

        for (size_t i = 0; i < i_xyzirt_point_cloud_ptr_->size(); i++)
        {
            i_xyzirt_point_cloud_ptr_->points[i].time += 0.1;
        }
    }
    else if (cfg_str_lidar_type_ == "ouster"){
        pcl::fromROSMsg(*lidar_msg_, *i_tmp_ouster_cloud_ptr_);
        pcl_conversions::toPCL(lidar_msg_->header, i_xyzirt_point_cloud_ptr_->header);

        i_xyzirt_point_cloud_ptr_->points.resize(i_tmp_ouster_cloud_ptr_->size());
        i_xyzirt_point_cloud_ptr_->is_dense = i_tmp_ouster_cloud_ptr_->is_dense;
        for (size_t i = 0; i < i_tmp_ouster_cloud_ptr_->size(); i++)
        {
            auto &src = i_tmp_ouster_cloud_ptr_->points[i];
            auto &dst = i_xyzirt_point_cloud_ptr_->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;

        }

    }
    else if (cfg_str_lidar_type_ == "kitti"){
        pcl::fromROSMsg(*lidar_msg_, *i_tmp_kitti_cloud_ptr_);
        pcl_conversions::toPCL(lidar_msg_->header, i_xyzirt_point_cloud_ptr_->header);

        i_xyzirt_point_cloud_ptr_->points.resize(i_tmp_kitti_cloud_ptr_->size());
        i_xyzirt_point_cloud_ptr_->is_dense = i_tmp_kitti_cloud_ptr_->is_dense;
        for (size_t i = 0; i < i_tmp_kitti_cloud_ptr_->size(); i++)
        {
            auto &src = i_tmp_kitti_cloud_ptr_->points[i];
            auto &dst = i_xyzirt_point_cloud_ptr_->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.time;
        }   
    }
    else if (cfg_str_lidar_type_ == "scala"){

        pcl::fromROSMsg(*lidar_msg_, *i_xyzirt_point_cloud_ptr_);
        // pcl::fromROSMsg(*lidar_msg_, *i_xyzin_point_cloud_ptr_);
        if(deque_time_lidar_pose_.size() > 0)
            i_xyzirt_point_cloud_ptr_->header.stamp = deque_time_lidar_pose_.back().first * 1e6;

        for (size_t i = 0; i < i_xyzirt_point_cloud_ptr_->size(); i++)
        {
            i_xyzirt_point_cloud_ptr_->points[i].time = 0.0;
        }
    }
    else if (cfg_str_lidar_type_ == "moe"){
        pcl::fromROSMsg(*lidar_msg_, *i_xyzirt_point_cloud_ptr_);
        pcl::fromROSMsg(*lidar_msg_, *gt_xyzrgb_pcptr_);

        for (size_t i = 0; i < i_xyzirt_point_cloud_ptr_->size(); i++)
        {
            i_xyzirt_point_cloud_ptr_->points[i].time = 0.0;
        }   
    }


    if(cfg_b_deskewing_ == true){
        if(RunDeskewing(i_xyzirt_point_cloud_ptr_, i_xyzirt_point_cloud_deskwed_ptr_) == false){
            b_new_lidar_ = false;
            return;
        }
            
    }
    else{
        *i_xyzirt_point_cloud_deskwed_ptr_ = *i_xyzirt_point_cloud_ptr_;
    }

    i_xyzin_point_cloud_ptr_->points.resize(i_xyzirt_point_cloud_deskwed_ptr_->size());
    i_xyzin_point_cloud_ptr_->is_dense = i_xyzirt_point_cloud_deskwed_ptr_->is_dense;
    for (size_t i = 0; i < i_xyzirt_point_cloud_deskwed_ptr_->size(); i++)
    {
        auto &src = i_xyzirt_point_cloud_deskwed_ptr_->points[i];
        auto &dst = i_xyzin_point_cloud_ptr_->points[i];

        dst.x = src.x;  // X 좌표
        dst.y = src.y;  // Y 좌표
        dst.z = src.z;  // Z 좌표
        dst.intensity = src.intensity;  // Intensity 값 복사

        dst.normal_x = 0.0f;
        dst.normal_y = 0.0f;
        dst.normal_z = 0.0f;
        dst.curvature = 0.0f;

    }


    // PMC
    if(cfg_b_deskewing_ == true)
        GsPmcPtr->Filter(i_xyzin_point_cloud_ptr_, lidar_synced_pose_.rotation().cast<double>(), lidar_synced_pose_.translation().cast<double>(), cur_time);
    else
        GsPmcPtr->Filter(i_xyzin_point_cloud_ptr_, 
                        deque_time_lidar_pose_.back().second.rotation().cast<double>(), 
                        deque_time_lidar_pose_.back().second.translation().cast<double>(), 
                        deque_time_lidar_pose_.back().first);


    GsPmcPtr->GetFilteredPoint(o_pmc_xyzrgb_pcptr_);
    GsPmcPtr->GetKeyFramePoint(o_key_frame_xyzrgb_pcptr_);
    GsPmcPtr->GetClusterPoint(o_cluster_xyzrgb_pcptr_);

    if(cfg_b_run_evaluation_){

        if(cfg_str_evaluation_dataset_ == "moe"){
            Eval_struct cur_eval = EvaluationMoeLabel(o_pmc_xyzrgb_pcptr_, gt_xyzrgb_pcptr_);
            i_iou_count_++;
            f_iou_sum_ += cur_eval.iou;
            f_precision_sum_ += cur_eval.precision;
            f_recall_sum_ += cur_eval.recall;
        }
        else if(cfg_str_evaluation_dataset_ == "kitti"){
            Eval_struct cur_eval = EvaluationKittiLabel(o_pmc_xyzrgb_pcptr_, i_tmp_kitti_cloud_ptr_);
            i_iou_count_++;
            f_iou_sum_ += cur_eval.iou;
            f_precision_sum_ += cur_eval.precision;
            f_recall_sum_ += cur_eval.recall;
        }
    }

    UpdatePmcPointCloud(o_pmc_xyzrgb_pcptr_);
    UpdateKeyFramePointCloud(o_key_frame_xyzrgb_pcptr_);
    UpdateClusterPointCloud(o_cluster_xyzrgb_pcptr_);

    if(cfg_b_debug_image_ == false){
        Publish();
        b_new_lidar_ = false;
        return;
    };

    std_msgs::Header img_header_range;
    cv_bridge::CvImage img_bridge_range, img_bridge_incident;
    
    // RGB Range Image
    cv::Mat mat_range_img = GsPmcPtr->GetRangeImageCv();
    float min_distance = 0.5f; // 최소 거리 (미터)
    float max_distance = 40.0f; // 최대 거리 (미터)

    cv::Mat mat_vis_range_img(mat_range_img.rows, mat_range_img.cols, CV_8UC3);
    for (int y = 0; y < mat_range_img.rows; y++) {
        for (int x = 0; x < mat_range_img.cols; x++) {
            float distance = mat_range_img.at<float>(y, x);
            double filtered_distance = std::min(std::max(distance, min_distance), max_distance);
            float normalized = (filtered_distance - min_distance) / (max_distance - min_distance);
            
            uchar blue = static_cast<uchar>((1.0f - normalized) * 255.0f);
            uchar green = static_cast<uchar>((1.0f - std::abs(normalized - 0.5f) * 2) * 255.0f);
            uchar red = static_cast<uchar>(normalized * 255.0f);

            if(distance < min_distance){
                blue = 0;
                green = 0;
                red = 0;
            }
            
            mat_vis_range_img.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);

        }
    }
    img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_vis_range_img );
    img_bridge_range.toImageMsg(o_range_image_msg_);

    // RGB Incident Image
    cv::Mat mat_incident_img = GsPmcPtr->GetIncidentImageCv();
    float min_incident = 0.0f; // red
    float max_incident = 90.0f; // blue

    int count = 0;

    cv::Mat mat_vis_incident_img(mat_incident_img.rows, mat_incident_img.cols, CV_8UC3);
    for (int y = 0; y < mat_incident_img.rows; y++) {
        for (int x = 0; x < mat_incident_img.cols; x++) {
            float incident = mat_incident_img.at<float>(y, x) * 180/M_PI;
            double filtered_incident = std::min(std::max(incident, min_incident), max_incident);
            float normalized = (filtered_incident - min_incident) / (max_incident - min_incident);
            
            uchar blue = static_cast<uchar>((1.0f - normalized) * 255.0f);
            uchar green = static_cast<uchar>((1.0f - std::abs(normalized - 0.5f) * 2) * 255.0f);
            uchar red = static_cast<uchar>(normalized * 255.0f);

            if(incident < -10E-5){
                blue = 0;
                green = 0;
                red = 0;
                count++;
            }
            
            mat_vis_incident_img.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);

        }
    }


    // cv::Mat sobelImage = applySobel(mat_vis_incident_img);

    std::cout<<"mat_vis_incident_img: "<<count<<std::endl;
    img_bridge_incident = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_vis_incident_img );
    img_bridge_incident.toImageMsg(o_incident_image_msg_);

    cv::Mat mat_cluster_img = GsPmcPtr->GetClusterImageCv();
    img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_cluster_img );
    img_bridge_range.toImageMsg(o_cluster_image_msg_);

    cv::Mat mat_dynamic_img = GsPmcPtr->GetDynamicImageCv();
    img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_dynamic_img );
    img_bridge_range.toImageMsg(o_dynamic_image_msg_);


    cv::Mat mat_ground_img = GsPmcPtr->GetGroundImageCv();
    cv::Mat mat_vis_ground_img;
    mat_ground_img.convertTo(mat_vis_ground_img, CV_8U, std::numeric_limits<uint8_t>::max() / 2);
    img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::MONO8, mat_vis_ground_img );
    img_bridge_range.toImageMsg(o_ground_image_msg_);


    Publish();
    b_new_lidar_ = false;
}


void GaussianPointMotionClassiWrapper::Publish(){
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

void GaussianPointMotionClassiWrapper::UpdatePmcPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_pmc_point_cloud_msg_);
    o_pmc_point_cloud_msg_.header.frame_id = str_main_lidar_frame_id_;
    o_pmc_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

void GaussianPointMotionClassiWrapper::UpdateKeyFramePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_key_frame_point_cloud_msg_);
    o_key_frame_point_cloud_msg_.header.frame_id = "world";
    o_key_frame_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

void GaussianPointMotionClassiWrapper::UpdateClusterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::toROSMsg(*cloud, o_cluster_point_cloud_msg_);
    o_cluster_point_cloud_msg_.header.frame_id = str_main_lidar_frame_id_;
    o_cluster_point_cloud_msg_.header.stamp = ros::Time(cur_time);
}

bool GaussianPointMotionClassiWrapper::RunDeskewing(pcl::PointCloud<PointXYZIRT>::Ptr i_point_cloud,
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
    if(cfg_str_lidar_type_ == "kitti") time_compensate_sec = 0.1;

    d_time_scan_cur_ = i_point_cloud->header.stamp * 1e-6 + i_point_cloud->points.front().time - time_compensate_sec; // pcl header time is micro sec
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


    for(int i = 0; i < cloudSize; ++i){
        double d_rel_time = i_point_cloud->points[i].time;
        DeskewPoint(&i_point_cloud->points[i], &o_deskewed_point_cloud->points[i], d_rel_time);
    }




    double tout_end = omp_get_wtime() - tout_start;
    std::cout<<"[Deskewing] Total Time: "<< tout_end*1000.0 << " ms"<<std::endl;
    return true;
    
}

void GaussianPointMotionClassiWrapper::DeskewPoint(PointXYZIRT *point, PointXYZIRT *o_point, double d_rel_time){
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

void GaussianPointMotionClassiWrapper::FindRotation(double relTime, float *rotXCur, float *rotYCur, float *rotZCur){
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    float ratio = relTime / (d_time_scan_end_ - d_time_scan_cur_);

    *rotXCur = ratio * rollIncre;
    *rotYCur = ratio * pitchIncre;
    *rotZCur = ratio * yawIncre;   
}

void GaussianPointMotionClassiWrapper::FindPosition(double relTime, float *posXCur, float *posYCur, float *posZCur){
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    float ratio = relTime / (d_time_scan_end_ - d_time_scan_cur_);

    *posXCur = ratio * odomIncreX;
    *posYCur = ratio * odomIncreY;
    *posZCur = ratio * odomIncreZ;   
}

cv::Mat GaussianPointMotionClassiWrapper::applySobel(const cv::Mat& inputImage, int scale, int delta, int ddepth) {
    cv::Mat gray, grad;
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // 입력 이미지를 그레이스케일로 변환
    if (inputImage.channels() == 3) {
        cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);
    } else {
        // 이미 그레이스케일이면 복사만 실행
        gray = inputImage.clone();
    }

    // X 방향의 Sobel 필터 적용
    cv::Sobel(gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    // 결과값의 절대값 계산
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Y 방향의 Sobel 필터 적용
    cv::Sobel(gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    // 결과값의 절대값 계산
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // 결과를 합치기
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    std::cout<<"Sobel Done"<<std::endl;
    return grad;
}


Eval_struct GaussianPointMotionClassiWrapper::EvaluationMoeLabel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result_pcptr, 
                                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gt_pcptr)
{
    ROS_INFO("[Evaluation Label] started");

    Eval_struct eval_result;
    float iou;

    int i_result_size = result_pcptr->points.size();

    for(int i = 0; i < i_result_size; i++){

        if(gt_pcptr->points[i].g == 1 && (result_pcptr->points[i].r == 0 && result_pcptr->points[i].g == 255)){
            eval_result.tp++; continue;
        }
        if(gt_pcptr->points[i].g == 0 && (result_pcptr->points[i].r == 0 && result_pcptr->points[i].g == 255)){
            eval_result.fp++; continue;
        }  
        if(gt_pcptr->points[i].g == 1 && result_pcptr->points[i].g != 255){
            eval_result.fn++; continue;
        }  
        if(gt_pcptr->points[i].g == 0 && result_pcptr->points[i].g != 255){
            eval_result.tn++; continue;
        }  
    }

    eval_result.iou = (float)eval_result.tp / (float)(eval_result.tp + eval_result.fp + eval_result.fn);
    eval_result.precision = float(eval_result.tp) / (float)(eval_result.tp + eval_result.fp);
    eval_result.recall = float(eval_result.tp) / (float)(eval_result.tp + eval_result.fn);


    ROS_INFO("[Evaluation Label] TP %d, FP, %d, FN %d, TN %d, IoU %f", eval_result.tp,eval_result.fp,eval_result.fn,eval_result.tn,eval_result.iou);
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fp + eval_result.fn) == 0) eval_result.iou = 0.0;
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fp) == 0) eval_result.precision = 0.0;
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fn) == 0) eval_result.recall = 0.0;
    
    return eval_result;
}

Eval_struct GaussianPointMotionClassiWrapper::EvaluationKittiLabel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result_pcptr, 
                                            const pcl::PointCloud<PointXYZIRGBRTLIS>::Ptr& gt_pcptr)
{
    ROS_INFO("[Evaluation Label] started");

    Eval_struct eval_result;
    float iou;

    int i_result_size = result_pcptr->points.size();

    for(int i = 0; i < i_result_size; i++){

        if((gt_pcptr->points[i].label >= 251 && gt_pcptr->points[i].label <= 259) && (result_pcptr->points[i].r == 0 && result_pcptr->points[i].g == 255)){
            eval_result.tp++; continue;
        }
        if((gt_pcptr->points[i].label < 251 || gt_pcptr->points[i].label > 259) && (result_pcptr->points[i].r == 0 && result_pcptr->points[i].g == 255)){
            eval_result.fp++; continue;
        }  
        if((gt_pcptr->points[i].label >= 251 && gt_pcptr->points[i].label <= 259) && result_pcptr->points[i].g != 255){
            eval_result.fn++; continue;
        }  
        if((gt_pcptr->points[i].label < 251 || gt_pcptr->points[i].label > 259) && result_pcptr->points[i].g != 255){
            eval_result.tn++; continue;
        }  
    }

    eval_result.iou = (float)eval_result.tp / (float)(eval_result.tp + eval_result.fp + eval_result.fn);
    eval_result.precision = float(eval_result.tp) / (float)(eval_result.tp + eval_result.fp);
    eval_result.recall = float(eval_result.tp) / (float)(eval_result.tp + eval_result.fn);


    ROS_INFO("[Evaluation Label] TP %d, FP, %d, FN %d, TN %d, IoU %f", eval_result.tp,eval_result.fp,eval_result.fn,eval_result.tn,eval_result.iou);
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fp + eval_result.fn) == 0) eval_result.iou = 0.0;
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fp) == 0) eval_result.precision = 0.0;
    if(eval_result.tp == 0 || (eval_result.tp + eval_result.fn) == 0) eval_result.recall = 0.0;

    return eval_result;
}



int main(int argc, char **argv) {
    std::string node_name = "gaussian_point_motion_classification";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    GaussianPointMotionClassiWrapper GsPmc;
    ros::Rate rate(200); // 100 Hz
    while(ros::ok()){
        GsPmc.Run();

        ros::spinOnce(); // Handle ROS callbacks
        rate.sleep();    // Sleep to control the loop rate
    }

    
    float miou = GsPmc.f_iou_sum_ / GsPmc.i_iou_count_;
    float precision = GsPmc.f_precision_sum_ / GsPmc.i_iou_count_;
    float recall = GsPmc.f_recall_sum_ / GsPmc.i_iou_count_;
    std::cout.precision(5);
    std::cout<<"MIoU: "<<miou <<" precision: "<<precision<<" recall: "<<recall<<std::endl;

    return 0;
}

