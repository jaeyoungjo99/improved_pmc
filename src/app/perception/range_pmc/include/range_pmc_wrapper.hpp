#ifndef __RANGE_PMC_WRAPPER__
#define __RANGE_PMC_WRAPPER__
#pragma once

#include <range_pmc.hpp>

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



using namespace ros;
using namespace tf;
using namespace std;


class RangePmcWrapper{
    public:
    RangePmcWrapper();
    ~RangePmcWrapper();

    void Init();
    void Run();
    void Publish();

    inline void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_point_cloud_);

        if(is_odom_init == false) return;
        
        str_main_lidar_frame_id_ = msg->header.frame_id;
        main_lidar_time_ = msg->header.stamp;
        
        std::cout<<std::endl;
        std::cout.precision(20);

        ROS_WARN_STREAM("PointCloud Callback: "<<main_lidar_time_.toSec());
        
        if(cfg_str_lidar_type_ == "velodyne"){
            main_lidar_time_ -= ros::Duration(0.1);

            pcl::fromROSMsg(*msg, *i_xyzirt_point_cloud_ptr_);
            // pcl::fromROSMsg(*msg, *i_xyzin_point_cloud_ptr_);

            for (size_t i = 0; i < i_xyzirt_point_cloud_ptr_->size(); i++)
            {
                i_xyzirt_point_cloud_ptr_->points[i].time += 0.1;
            }
        }
        else if (cfg_str_lidar_type_ == "ouster"){
            pcl::fromROSMsg(*msg, *i_tmp_ouster_cloud_ptr_);
            pcl_conversions::toPCL(msg->header, i_xyzirt_point_cloud_ptr_->header);

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
            pcl::fromROSMsg(*msg, *i_tmp_kitti_cloud_ptr_);
            pcl_conversions::toPCL(msg->header, i_xyzirt_point_cloud_ptr_->header);

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

        if(RunDeskewing(i_xyzirt_point_cloud_ptr_, i_xyzirt_point_cloud_deskwed_ptr_) == false)
            return;

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

        if(cfg_b_height_filter_ == true){
            pcl::PassThrough<pcl::PointXYZINormal> pass;
            pass.setInputCloud(i_xyzin_point_cloud_ptr_); // 입력 클라우드 설정
            pass.setFilterFieldName("z"); // z 필드(높이)를 필터링 기준으로 설정
            pass.setFilterLimits(-1.0, std::numeric_limits<float>::max()); // minHeight 이상의 높이를 갖는 점들만 남김
            pass.filter(*i_xyzin_point_cloud_ptr_); // 필터링 적용
        }

        // PMC
        RangePmcPtr->Filter(i_xyzin_point_cloud_ptr_, lidar_synced_pose_.rotation().cast<double>(), lidar_synced_pose_.translation().cast<double>(), cur_time);
        RangePmcPtr->GetFilteredPoint(o_pmc_xyzrgb_pcptr_);
        RangePmcPtr->GetKeyFramePoint(o_key_frame_xyzrgb_pcptr_);
        RangePmcPtr->GetClusterPoint(o_cluster_xyzrgb_pcptr_);

        UpdatePmcPointCloud(o_pmc_xyzrgb_pcptr_);
        UpdateKeyFramePointCloud(o_key_frame_xyzrgb_pcptr_);
        UpdateClusterPointCloud(o_cluster_xyzrgb_pcptr_);

        if(cfg_b_debug_image_ == false){
            Publish();
            return;
        };

        std_msgs::Header img_header_range;
        cv_bridge::CvImage img_bridge_range, img_bridge_incident;
        
        // RGB Range Image
        cv::Mat mat_range_img = RangePmcPtr->GetRangeImageCv();
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
        cv::Mat mat_incident_img = RangePmcPtr->GetIncidentImageCv();
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
        std::cout<<"mat_vis_incident_img: "<<count<<std::endl;
        img_bridge_incident = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_vis_incident_img );
        img_bridge_incident.toImageMsg(o_incident_image_msg_);

        cv::Mat mat_cluster_img = RangePmcPtr->GetClusterImageCv();
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_cluster_img );
        img_bridge_range.toImageMsg(o_cluster_image_msg_);

        cv::Mat mat_dynamic_img = RangePmcPtr->GetDynamicImageCv();
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_dynamic_img );
        img_bridge_range.toImageMsg(o_dynamic_image_msg_);


        cv::Mat mat_ground_img = RangePmcPtr->GetGroundImageCv();
        cv::Mat mat_vis_ground_img;
        mat_ground_img.convertTo(mat_vis_ground_img, CV_8U, std::numeric_limits<uint8_t>::max() / 2);
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::MONO8, mat_vis_ground_img );
        img_bridge_range.toImageMsg(o_ground_image_msg_);


        Publish();
    }
    inline void CallbackOdometry(const geometry_msgs::PoseWithCovarianceStamped& cur_odom)
    {
        std::lock_guard<std::mutex> lock(mutex_odometry_);

        Eigen::Quaterniond cur_q;
        geometry_msgs::Quaternion tmp_q;
        // tmp_q = cur_odom.pose.pose.orientation;
        // tf::quaternionMsgToEigen(tmp_q, cur_q);
        // cur_rot = cur_q.matrix();

        Eigen::Quaterniond eigen_quat(cur_odom.pose.pose.orientation.w, cur_odom.pose.pose.orientation.x, cur_odom.pose.pose.orientation.y, cur_odom.pose.pose.orientation.z);
        cur_rot = eigen_quat.toRotationMatrix();
        
        // ego_frame에서 LiDAR까지의 변위를 고려한 LiDAR의 현재 위치를 계산합니다.
        Eigen::Vector3d lidar_offset(cfg_vec_f_ego_to_lidar_[0], cfg_vec_f_ego_to_lidar_[1], cfg_vec_f_ego_to_lidar_[2]);
        Eigen::Vector3d cur_pos_ego(cur_odom.pose.pose.position.x, 
                                    cur_odom.pose.pose.position.y, 
                                    cur_odom.pose.pose.position.z);
        
        // LiDAR 위치를 업데이트합니다.
        cur_pos = cur_pos_ego + cur_rot * lidar_offset;


        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        // transform.rotate(cur_rot);
        // transform.translate(cur_pos);

        transform = Eigen::Translation3d(cur_pos) * Eigen::AngleAxisd(cur_rot);
        Eigen::Affine3f iter_lidar_pose = transform.cast<float>();

        deque_time_lidar_pose_.push_back(std::make_pair(cur_odom.header.stamp.toSec() ,iter_lidar_pose));
        // std::cout<<"IMU Time pushback: "<<cur_odom.header.stamp.toSec()<<std::endl;

        while(deque_time_lidar_pose_.size() > 200) deque_time_lidar_pose_.pop_front();


        is_odom_init = true;

    }

    inline void CallbackKittiGeo(const geometry_msgs::PoseStamped& cur_odom)
    {
        std::lock_guard<std::mutex> lock(mutex_odometry_);

        Eigen::Quaterniond cur_q;
        geometry_msgs::Quaternion tmp_q;
        // tmp_q = cur_odom.pose.pose.orientation;
        // tf::quaternionMsgToEigen(tmp_q, cur_q);
        // cur_rot = cur_q.matrix();

        Eigen::Quaterniond eigen_quat(cur_odom.pose.orientation.w, cur_odom.pose.orientation.x, cur_odom.pose.orientation.y, cur_odom.pose.orientation.z);
        cur_rot = eigen_quat.toRotationMatrix();
        
        // ego_frame에서 LiDAR까지의 변위를 고려한 LiDAR의 현재 위치를 계산합니다.
        Eigen::Vector3d lidar_offset(cfg_vec_f_ego_to_lidar_[0], cfg_vec_f_ego_to_lidar_[1], cfg_vec_f_ego_to_lidar_[2]);
        Eigen::Vector3d cur_pos_ego(cur_odom.pose.position.x, 
                                    cur_odom.pose.position.y, 
                                    cur_odom.pose.position.z);
        
        // LiDAR 위치를 업데이트합니다.
        cur_pos = cur_pos_ego + cur_rot * lidar_offset;


        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        // transform.rotate(cur_rot);
        // transform.translate(cur_pos);

        transform = Eigen::Translation3d(cur_pos) * Eigen::AngleAxisd(cur_rot);
        Eigen::Affine3f iter_lidar_pose = transform.cast<float>();

        deque_time_lidar_pose_.push_back(std::make_pair(cur_odom.header.stamp.toSec() + 0.1 ,iter_lidar_pose));
        // std::cout<<"IMU Time pushback: "<<cur_odom.header.stamp.toSec()<<std::endl;

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

    // Algorithm
        // shared_ptr<RangePmc> RangePmcPtr(new RangePmc());
        std::shared_ptr<RangePmc> RangePmcPtr = std::make_shared<RangePmc>();


    // configure;
    private:

        string cfg_str_dataset_;
        string cfg_str_lidar_topic_name_;
        string cfg_str_odom_topic_name_;
        string cfg_str_lidar_type_;
        
        bool  cfg_b_deskewing_;
        float cfg_f_horizontal_resolution_;
        float cfg_f_vertical_resolution_;
        float cfg_f_min_range_;
        float cfg_f_max_range_;
        float cfg_f_range_threshold_;
        int cfg_i_max_range_image_num_;
        float cfg_f_min_key_frame_time_;
        float cfg_f_min_key_frame_rot_;
        int cfg_i_frame_rate_;
        float cfg_f_fov_up_;
        float cfg_f_fov_down_;
        float cfg_f_fov_left_;
        float cfg_f_fov_right_;
        float cfg_i_min_occluded_num_;
        int   cfg_i_neighbor_pixel_max_;
        bool  cfg_b_height_filter_;

        float cfg_f_ground_angle_;
        float cfg_f_dist_threshold_m_;
        int   cfg_i_segment_min_point_num_;
        int   cfg_i_segment_valid_poiint_num_;
        int   cfg_i_segment_valid_line_num_;

        float cfg_f_moving_confidence_;
        float cfg_f_static_confidence_;
        float cfg_f_gaussian_sigma_;
        float cfg_f_static_gaussian_sigma_;
        float cfg_f_dynamic_gaussian_sigma_;
            
        bool cfg_b_cluster_level_filtering_;

        bool cfg_b_output_static_point_;
        bool cfg_b_output_min_range_;
        bool cfg_b_debug_image_;

        std::vector<float> cfg_vec_f_ego_to_lidar_;

    private:

};

#endif