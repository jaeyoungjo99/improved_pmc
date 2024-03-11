#ifndef __NOVATEL_CONVERTER__
#define __NOVATEL_CONVERTER__
#pragma once

// STD header
#include <iostream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>

// ROS header
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <autoku_msgs/VehicleState.h>
#include <novatel_oem7_msgs/CORRIMU.h>
#include <novatel_oem7_msgs/INSPVAX.h>

// Utility header
#include <boost/filesystem.hpp>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

using namespace ros;
using namespace tf;
using namespace std;

class NovatelConverter{
    public:
    NovatelConverter();
    ~NovatelConverter();

    void Init();
    void Run();
    void UpdateVehicleState();
    void UpdateEgoCovMarker(const novatel_oem7_msgs::INSPVAX inspvax_msg);
    void UpdateEgoGeo(const novatel_oem7_msgs::INSPVAX inspvax_msg);
    jsk_rviz_plugins::OverlayText UpdateNovatelPosTypeText(const novatel_oem7_msgs::INSPVAX inspvax_msg);
    void Publish();

    void CallbackINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg);
    void CallbackKittiGeo(const geometry_msgs::PoseStamped::ConstPtr& msg);

    private:
    ros::Subscriber sub_novatel_inspvax_;
    ros::Subscriber sub_kitti_geo_;
    ros::Publisher pub_vehicle_state_;
    ros::Publisher pub_ego_marker_;
    ros::Publisher pub_ego_cov_marker_;
    ros::Publisher pub_rviz_novatel_pos_type_;

    ros::Publisher pub_ego_geometry_msg_;

    private:
    std::string str_dataset_ = "";
    std::string str_ego_pose_msg_ = "";
    std::string projection_mode_ = "";
    bool b_use_init_lat_lon_ = false;
    double ref_latitude_ = 0.0;
    double ref_longitude_ = 0.0;
    double novatel_heading_bias_deg_ = 0.0;

    novatel_oem7_msgs::INSPVAX i_novatel_inspvax_;
    geometry_msgs::PoseStamped i_kitti_geo_;
    autoku_msgs::VehicleState     o_vehicle_state_;
    visualization_msgs::Marker o_ego_marker_msgs_;
    visualization_msgs::Marker o_ego_cov_marker_msgs_;
    geometry_msgs::PoseWithCovarianceStamped o_novatel_ego_geo_;

    jsk_rviz_plugins::OverlayText o_rviz_pos_type_;

    private:
    
    bool b_is_ref_init_ = false;
    bool b_is_new_msg_ = false;



};

#endif