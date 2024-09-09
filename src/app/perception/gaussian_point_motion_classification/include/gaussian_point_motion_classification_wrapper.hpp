#ifndef __GAUSSIAN_POINT_MOTION_CLASSIFICATION_WRAPPER__
#define __GAUSSIAN_POINT_MOTION_CLASSIFICATION_WRAPPER__
#pragma once

#include <gaussian_point_motion_classification.hpp>

// ROS header
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>



using namespace ros;
using namespace tf;
using namespace std;

struct Eval_struct{
    int tp = 0;
    int fp = 0;
    int tn = 0;
    int fn = 0;
    float iou = 0.0;
    float precision = 0.0;
    float recall = 0.0;
};

class GaussianPointMotionClassiWrapper{
    public:
    GaussianPointMotionClassiWrapper();
    ~GaussianPointMotionClassiWrapper();

    void Init();
    void Run();
    void Publish();

    inline void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
    {       
        std::lock_guard<std::mutex> lock(mutex_point_cloud_);
        lidar_msg_= msg;
        b_new_lidar_ = true;    
    }
    inline void CallbackOdometry(const geometry_msgs::PoseWithCovarianceStamped& cur_odom)
    {
        std::lock_guard<std::mutex> lock(mutex_odometry_);


        Eigen::Translation3f ego_to_gps_affine(cfg_vec_f_ego_to_lidar_[0], cfg_vec_f_ego_to_lidar_[1], cfg_vec_f_ego_to_lidar_[2]);
        Eigen::AngleAxisf rollAngle (cfg_vec_f_ego_to_lidar_rotation_[0]*M_PI/180, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(cfg_vec_f_ego_to_lidar_rotation_[1]*M_PI/180, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle  (cfg_vec_f_ego_to_lidar_rotation_[2]*M_PI/180, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf rotationQuaternion = yawAngle * pitchAngle * rollAngle;

        Eigen::Affine3f gpsToLidarTransform = ego_to_gps_affine * rotationQuaternion;

        Eigen::Quaternionf rotation(cur_odom.pose.pose.orientation.w, cur_odom.pose.pose.orientation.x, cur_odom.pose.pose.orientation.y, cur_odom.pose.pose.orientation.z);
        rotation.normalize(); // 정규화를 통해 유효한 회전을 보장
        Eigen::Vector3f position(cur_odom.pose.pose.position.x, 
                                    cur_odom.pose.pose.position.y, 
                                    cur_odom.pose.pose.position.z);
        Eigen::Affine3f transform = Eigen::Translation3f(position) * rotation;

        Eigen::Affine3f iter_lidar_pose = transform * gpsToLidarTransform;

        deque_time_lidar_pose_.push_back(std::make_pair(cur_odom.header.stamp.toSec() ,iter_lidar_pose));
        std::cout.precision(20);
        std::cout<<"IMU Time pushback: "<<cur_odom.header.stamp.toSec()<<std::endl;

        while(deque_time_lidar_pose_.size() > 200) deque_time_lidar_pose_.pop_front();


        is_odom_init = true;

    }

    inline void CallbackKittiGeo(const geometry_msgs::PoseStamped& cur_odom)
    {
        std::lock_guard<std::mutex> lock(mutex_odometry_);

        Eigen::Translation3f ego_to_gps_affine(cfg_vec_f_ego_to_lidar_[0], cfg_vec_f_ego_to_lidar_[1], cfg_vec_f_ego_to_lidar_[2]);
        Eigen::AngleAxisf rollAngle (cfg_vec_f_ego_to_lidar_rotation_[0]*M_PI/180, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(cfg_vec_f_ego_to_lidar_rotation_[1]*M_PI/180, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle  (cfg_vec_f_ego_to_lidar_rotation_[2]*M_PI/180, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf rotationQuaternion = yawAngle * pitchAngle * rollAngle;

        Eigen::Affine3f gpsToLidarTransform = ego_to_gps_affine * rotationQuaternion;

        Eigen::Quaternionf rotation(cur_odom.pose.orientation.w, cur_odom.pose.orientation.x, cur_odom.pose.orientation.y, cur_odom.pose.orientation.z);
        rotation.normalize(); // 정규화를 통해 유효한 회전을 보장
        Eigen::Vector3f position(cur_odom.pose.position.x, 
                                    cur_odom.pose.position.y, 
                                    cur_odom.pose.position.z);
        Eigen::Affine3f transform = Eigen::Translation3f(position) * rotation;

        Eigen::Affine3f iter_lidar_pose = transform * gpsToLidarTransform;

        deque_time_lidar_pose_.push_back(std::make_pair(cur_odom.header.stamp.toSec() + 0.1, iter_lidar_pose));

        while(deque_time_lidar_pose_.size() > 200) deque_time_lidar_pose_.pop_front();

        is_odom_init = true;
    }

    void UpdatePmcPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void UpdateKeyFramePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void UpdateClusterPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);


    bool RunDeskewing(pcl::PointCloud<PointXYZIRT>::Ptr i_point_cloud,
                pcl::PointCloud<PointXYZIRT>::Ptr& o_deskewed_point_cloud);
    void DeskewPoint(PointXYZIRT *point, PointXYZIRT *o_point, double d_rel_time);
    void FindRotation(double relTime, float *rotXCur, float *rotYCur, float *rotZCur);
    void FindPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    cv::Mat applySobel(const cv::Mat& inputImage, int scale = 1, int delta = 0, int ddepth = CV_16S);

    Eval_struct EvaluationMoeLabel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result_pcptr, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gt_pcptr);
    Eval_struct EvaluationKittiLabel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result_pcptr, const pcl::PointCloud<PointXYZIRGBRTLIS>::Ptr& gt_pcptr);

    // ROS
    private:

        Subscriber sub_point_cloud_;
        Subscriber sub_odometry_;

        Publisher pub_pmc_point_cloud_;
        Publisher pub_key_frame_point_cloud_;
        Publisher pub_cluster_point_cloud_;

        Publisher pub_range_image_;
        Publisher pub_incident_image_;
        Publisher pub_dynamic_image_;
        Publisher pub_ground_image_;
        Publisher pub_cluster_image_;
        Publisher pub_angle_image_;

    // Data
    private:
        string str_main_lidar_frame_id_;
        ros::Time main_lidar_time_;

        pcl::PointCloud<OusterPointXYZIRT>::Ptr i_tmp_ouster_cloud_ptr_;
        pcl::PointCloud<PointXYZIRGBRTLIS>::Ptr i_tmp_kitti_cloud_ptr_;
        pcl::PointCloud<PointXYZIRT>::Ptr i_xyzirt_point_cloud_ptr_;
        pcl::PointCloud<PointXYZIRT>::Ptr i_xyzirt_point_cloud_deskwed_ptr_;
        PointCloudXYZI::Ptr i_xyzin_point_cloud_ptr_;
        geometry_msgs::PoseWithCovarianceStamped i_odometry_;

        sensor_msgs::PointCloud2ConstPtr lidar_msg_;
        

        M3D cur_rot = Eigen::Matrix3d::Identity();
        V3D cur_pos = Eigen::Vector3d::Zero();
        double cur_time;
        bool is_odom_init = false;

    // Deskewing
        double  d_time_scan_cur_;
        double  d_time_scan_end_;

        float odomIncreX;
        float odomIncreY;
        float odomIncreZ;
        float rollIncre;
        float pitchIncre;
        float yawIncre;

        float odomIncreXs2p;
        float odomIncreYs2p;
        float odomIncreZs2p;
        float rollIncres2p;
        float pitchIncres2p;
        float yawIncres2p;

        std::deque<std::pair<double, Eigen::Affine3f>>    deque_time_lidar_pose_;
        std::pair<double, Eigen::Affine3f> scan_start_lidar_time_pose_;
        std::pair<double, Eigen::Affine3f> scan_end_lidar_time_pose_;
        Eigen::Affine3f lidar_synced_pose_;



    // Output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_pmc_xyzrgb_pcptr_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_key_frame_xyzrgb_pcptr_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_cluster_xyzrgb_pcptr_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_xyzrgb_pcptr_;

        sensor_msgs::PointCloud2 o_pmc_point_cloud_msg_;
        sensor_msgs::PointCloud2 o_key_frame_point_cloud_msg_;
        sensor_msgs::PointCloud2 o_cluster_point_cloud_msg_;

        sensor_msgs::Image o_range_image_msg_;
        sensor_msgs::Image o_incident_image_msg_;
        sensor_msgs::Image o_dynamic_image_msg_;
        sensor_msgs::Image o_ground_image_msg_;
        sensor_msgs::Image o_cluster_image_msg_;
        sensor_msgs::Image o_angle_image_msg_;
        

    // Mutex
    private:
        mutex mutex_point_cloud_;
        mutex mutex_odometry_;

        bool b_synced_ = false;
        bool b_new_lidar_ = false;

    // Algorithm
        // shared_ptr<GaussianPointMotionClassi> GsPmcPtr(new GaussianPointMotionClassi());
        std::shared_ptr<GaussianPointMotionClassi> GsPmcPtr = std::make_shared<GaussianPointMotionClassi>();


    // configure;
    private:

        string  cfg_str_dataset_;
        string  cfg_str_lidar_topic_name_;
        string  cfg_str_odom_topic_name_;
        string  cfg_str_lidar_type_;
        
        bool    cfg_b_deskewing_;
        float   cfg_f_horizontal_resolution_;
        float   cfg_f_vertical_resolution_;
        float   cfg_f_min_range_;
        float   cfg_f_max_range_;
        int     cfg_i_max_points_in_pixel_;
        float   cfg_f_range_threshold_;
        int     cfg_i_max_range_image_num_;
        float   cfg_f_min_key_frame_time_;
        float   cfg_f_min_key_frame_rot_;
        int     cfg_i_frame_rate_;
        float   cfg_f_fov_up_;
        float   cfg_f_fov_down_;
        float   cfg_f_fov_left_;
        float   cfg_f_fov_right_;
        float   cfg_i_min_occluded_num_;

        int     cfg_i_vertical_neighbor_pixel_;
        int     cfg_i_horizontal_neighbor_pixel_;
        bool    cfg_b_height_filter_;

        float   cfg_f_ground_angle_;
        float   cfg_f_dist_threshold_m_;
        int     cfg_i_segment_min_point_num_;
        int     cfg_i_segment_valid_poiint_num_;
        int     cfg_i_segment_valid_line_num_;

        float   cfg_f_moving_confidence_;
        float   cfg_f_static_confidence_;
        float   cfg_f_gaussian_sigma_;
        float   cfg_f_static_gaussian_sigma_;
        float   cfg_f_dynamic_gaussian_sigma_;
        float   cfg_f_sigma_epsilon_;
            
        bool    cfg_b_ground_filtering_;
        bool    cfg_b_cluster_level_filtering_;
        bool    cfg_b_run_case_2_;
        bool    cfg_b_run_region_growing_;
        bool    cfg_b_run_small_dynamic_filtering_;
        int     cfg_i_min_voxel_cluster_num_;

        bool    cfg_b_output_static_point_;
        bool    cfg_b_output_discrete_label_;
        bool    cfg_b_output_min_range_;
        bool    cfg_b_debug_image_;

        bool    cfg_b_run_evaluation_;
        string  cfg_str_evaluation_dataset_;

        std::vector<float> cfg_vec_f_ego_to_lidar_;
        std::vector<float> cfg_vec_f_ego_to_lidar_rotation_;

    public:
        int i_iou_count_ = 0;
        float f_iou_sum_ = 0.0;
        float f_precision_sum_ = 0.0;
        float f_recall_sum_ = 0.0;
        int i_tp_sum_ = 0.0;
        int i_fp_sum_ = 0.0;
        int i_tn_sum_ = 0.0;
        int i_fn_sum_ = 0.0;
};

#endif