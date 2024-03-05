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
        std::cout<<"PointCloud Callback: "<<main_lidar_time_.toSec()<<std::endl;
        
        if(cfg_str_lidar_type_ == "velodyne"){
            main_lidar_time_ -= ros::Duration(0.1);

            pcl::fromROSMsg(*msg, *i_xyzirt_point_cloud_ptr_);
            // pcl::fromROSMsg(*msg, *i_xyzin_point_cloud_ptr_);

            for (size_t i = 0; i < i_xyzirt_point_cloud_ptr_->size(); i++)
            {
                i_xyzirt_point_cloud_ptr_->points[i].time += 0.1;
            }
        }
        else{
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


        // TODO Deskewing
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
        // RangePmcPtr->Filter(i_xyzin_point_cloud_ptr_, cur_rot, cur_pos, cur_time);
        RangePmcPtr->GetFilteredPoint(o_pmc_xyzrgb_pcptr_);


        std_msgs::Header img_header_range;
        cv_bridge::CvImage img_bridge_range;

        cv::Mat mat_range_img = RangePmcPtr->GetRangeImageCv();
        // cv::Mat mat_vis_range_img;
        // float f_scale_factor_32F_to_8u_range = (float)std::numeric_limits<uint8_t>::max()/( 80 - 0.5 ); // normalize - max: 80, min: 0.5 [m]
        // mat_range_img.convertTo(mat_vis_range_img, CV_8U, f_scale_factor_32F_to_8u_range );

        float min_distance = 0.5f; // 최소 거리 (미터)
        float max_distance = 50.0f; // 최대 거리 (미터)

        cv::Mat mat_vis_range_img(mat_range_img.rows, mat_range_img.cols, CV_8UC3);
        for (int y = 0; y < mat_range_img.rows; y++) {
            for (int x = 0; x < mat_range_img.cols; x++) {
                // 현재 픽셀의 거리값을 가져옵니다.
                float distance = mat_range_img.at<float>(y, x);
        
                // 거리를 0.5 ~ 80.0 범위 내로 제한합니다.
                double filtered_distance = std::min(std::max(distance, min_distance), max_distance);
                
                // 거리에 따른 색상을 계산합니다 (0.5에 가까울수록 파란색, 80에 가까울수록 빨간색).
                float normalized = (filtered_distance - min_distance) / (max_distance - min_distance);
                
                // RGB 채널을 거리에 따라 계산합니다.
                // 파란색에서 빨간색으로 (중간은 녹색).
                uchar blue = static_cast<uchar>((1.0f - normalized) * 255.0f);
                uchar green = static_cast<uchar>((1.0f - std::abs(normalized - 0.5f) * 2) * 255.0f);
                uchar red = static_cast<uchar>(normalized * 255.0f);

                if(distance < min_distance){
                    blue = 0;
                    green = 0;
                    red = 0;
                }
                
                // RGB 이미지에 색상을 설정합니다.
                mat_vis_range_img.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);

            }
        }
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_vis_range_img );

        // img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::MONO8, mat_vis_range_img );
        img_bridge_range.toImageMsg(o_range_image_msg_);


        cv::Mat mat_dynamic_img = RangePmcPtr->GetDynamicImageCv();
        cv::Mat mat_vis_dynamic_img;
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::RGB8, mat_dynamic_img );
        img_bridge_range.toImageMsg(o_dynamic_image_msg_);


        UpdatePmcPointCloud(o_pmc_xyzrgb_pcptr_);
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
        std::cout<<"IMU Time pushback: "<<cur_odom.header.stamp.toSec()<<std::endl;

        while(deque_time_lidar_pose_.size() > 200) deque_time_lidar_pose_.pop_front();


        is_odom_init = true;

    }

    void UpdatePmcPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);


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
        Publisher pub_range_image_;
        Publisher pub_dynamic_image_;
        Publisher pub_angle_image_;

    // Data
    private:
        string str_main_lidar_frame_id_;
        ros::Time main_lidar_time_;

        pcl::PointCloud<OusterPointXYZIRT>::Ptr i_tmp_ouster_cloud_ptr_;
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

        std::deque<std::pair<double, Eigen::Affine3f>>    deque_time_lidar_pose_;
        std::pair<double, Eigen::Affine3f> scan_start_lidar_time_pose_;
        std::pair<double, Eigen::Affine3f> scan_end_lidar_time_pose_;
        Eigen::Affine3f lidar_synced_pose_;

    // Output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr o_pmc_xyzrgb_pcptr_;
        sensor_msgs::PointCloud2 o_pmc_point_cloud_msg_;
        sensor_msgs::Image o_range_image_msg_;
        sensor_msgs::Image o_dynamic_image_msg_;
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

        string cfg_str_lidar_topic_name_;
        string cfg_str_odom_topic_name_;
        string cfg_str_lidar_type_;
        
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

        std::vector<float> cfg_vec_f_ego_to_lidar_;

    private:

};

#endif