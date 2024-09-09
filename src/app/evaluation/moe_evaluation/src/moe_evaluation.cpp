#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <fstream>
#include <rosbag/bag.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "moe_evaluation");
    ros::NodeHandle nh;

    double dt = 0.1;
    double b_error = false;

    std::string pcd_folder_path, label_folder_path, pose_file_path, result_bag_path;
    if (!nh.getParam("/moe_evaluation/pcd_folder_path", pcd_folder_path)) pcd_folder_path = "";
    if (!nh.getParam("/moe_evaluation/label_folder_path", label_folder_path)) label_folder_path = "";
    if (!nh.getParam("/moe_evaluation/pose_file_path", pose_file_path)) pose_file_path = "";
    if (!nh.getParam("/moe_evaluation/result_bag_path", result_bag_path)) result_bag_path = "";

    ROS_INFO_STREAM("pcd_folder_path: " <<pcd_folder_path);
    ROS_INFO_STREAM("label_folder_path: " <<label_folder_path);
    ROS_INFO_STREAM("pose_file_path: "<<pose_file_path);
    ROS_INFO_STREAM("result_bag_path: " <<result_bag_path);

    rosbag::Bag bag;
    bag.open(result_bag_path, rosbag::bagmode::Write);

    std::ifstream pose_file(pose_file_path);
    std::vector<geometry_msgs::PoseWithCovarianceStamped> poses;

    // 위치 정보 로드
    float r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z;
    int index = 0;
    while (pose_file >> r11 >> r12 >> r13 >> x >>  r21 >> r22 >> r23 >> y >> r31 >> r32 >> r33 >> z) {
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = z;

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << r11, r12, r13,
                            r21, r22, r23,
                            r31, r32, r33;

        Eigen::Quaterniond quaternion(rotation_matrix);

        pose.pose.pose.orientation.x = quaternion.x();
        pose.pose.pose.orientation.y = quaternion.y();
        pose.pose.pose.orientation.z = quaternion.z();
        pose.pose.pose.orientation.w = quaternion.w();
        pose.header.frame_id = "world";

        // // 공분산 값 설정
        // std::fill(std::begin(pose.pose.covariance), std::end(pose.pose.covariance), 0.0);
        // pose.pose.covariance[0] = 0.01;  // x의 분산
        // pose.pose.covariance[7] = 0.01;  // y의 분산
        // pose.pose.covariance[14] = 0.01; // z의 분산
        // pose.pose.covariance[21] = 0.01; // roll의 분산
        // pose.pose.covariance[28] = 0.01; // pitch의 분산
        // pose.pose.covariance[35] = 0.01; // yaw의 분산

        poses.push_back(pose);
    }
    std::cout<<"Pose size: "<< poses.size()<<std::endl;


    // 모든 PCD 파일 처리
    for (int i = 0; i < poses.size(); ++i) {
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << i;
        std::string pcd_filename = pcd_folder_path + ss.str() + ".pcd";
        std::string label_filename = label_folder_path + ss.str() + ".txt";

        sensor_msgs::PointCloud2 output_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_filename, cloud) == -1) {
            ROS_ERROR("Couldn't read pcd file %s", pcd_filename.c_str());
            b_error = true;
            break;
        }

        std::ifstream label_file(label_filename);
        int moveable_id, moving_status, class_id;

        int point_ind = 0;
        while(label_file >> moveable_id >> moving_status >> class_id){
            cloud.points[point_ind].r = moveable_id;
            cloud.points[point_ind].g = moving_status;
            cloud.points[point_ind].b = class_id;

            point_ind++;
        }


        pcl::toROSMsg(cloud, output_cloud);
        output_cloud.header.frame_id = "moe";
        output_cloud.header.stamp = ros::Time(ros::TIME_MIN.toSec() + i * dt);
        // output_cloud.header.stamp = ros::Time::now();

        // 해당 PCD 파일에 맞는 위치 정보 설정
        poses[i].header.stamp = output_cloud.header.stamp;
        bag.write("/moe_points", output_cloud.header.stamp, output_cloud);
        bag.write("/app/loc/novatel_geo", poses[i].header.stamp, poses[i]);

        if(i % 10 == 0){
            double progress = 100.0 * i / (poses.size() - 1);
            std::cout << "Progress: " << std::fixed << std::setprecision(2) << progress << "%\r";
            std::cout.flush();
        }

    }

    bag.close();

    if(b_error){
        ROS_ERROR("Error Done");
    }
    else{
        ROS_INFO("Done");
    }
    return 0;
}