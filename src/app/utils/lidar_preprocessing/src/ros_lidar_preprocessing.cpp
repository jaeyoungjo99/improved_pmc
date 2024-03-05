#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
// // opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "range_image/range_image.hpp"
#include "point_type/pointXYZIRT.hpp"
#include "segmentation/line_based_segmentation.hpp"
#include "deskewing/deskewing.hpp"
#include "zrp_estimation/zrp_estimation.hpp"
#include "util/debugprintutil/debug_print.hpp"


#define COLOR_CYAN 0.55  // cyan
#define COLOR_GREEN 0.2  // green
#define COLOR_BLUE 0.0   // blue
#define COLOR_RED 1.0    // red

std::vector<float> COLOR_MAP = {COLOR_CYAN, COLOR_GREEN, COLOR_BLUE, COLOR_RED};

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range)
)

// using pointType = pcl::PointXYZI;
// using pointType = pointXYZAIRRT;
using pointType = PointXYZIRT;

// Variables for configurations
ros::Publisher g_pub_cloud;
ros::Publisher g_pub_ground_points;
ros::Publisher g_pub_non_ground_points;
ros::Publisher g_pub_pole_points;
ros::Publisher g_pub_rospub_range_image;
ros::Publisher g_pub_rospub_cluster_image;
ros::Publisher g_pub_rospub_index_image;
ros::Publisher g_pub_rospub_ground_mask_image;
ros::Publisher g_pub_pole_normals;
ros::Publisher g_pub_jsk_ground_plane;

jsk_recognition_msgs::PolygonArray ground_plane_jsk_;

std::string g_cfg_str_input_topic_name;
std::string g_cfg_str_lidar_type;
std::string g_cfg_str_output_topic_name; 
std::string g_cfg_str_output_topic_name_ground;
std::string g_cfg_str_output_topic_name_non_ground;
std::string g_cfg_str_output_topic_frame;

bool  g_b_image_flip_x;
bool  g_b_image_flip_y;
float g_f_range_img_min_value = 0 ;
float g_f_range_img_max_value = 0 ;

std::unique_ptr<CLineBasedSegmentation> line_based_segmentation;

std::shared_ptr<CRangeImage> range_image;

bool rosInit(ros::NodeHandle nh)
{
    std::string g_cfg_str_range_image_ini_path;
    std::string g_cfg_str_segmentation_path;
    if(nh.getParam("/lidar_preprocessing/ros_config_lidar_type",                        g_cfg_str_lidar_type) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_lidar_type");              return false; }
    if(nh.getParam("/lidar_preprocessing/ros_config_input_topic_name",                  g_cfg_str_input_topic_name) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_input_topic_name");              return false; }
    if(nh.getParam("/lidar_preprocessing/ros_config_output_topic",                      g_cfg_str_output_topic_name) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_output_topic");                  return false; }
    if(nh.getParam("/lidar_preprocessing/ros_config_output_topic_frame",                g_cfg_str_output_topic_frame) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_output_topic_frame");                  return false; }  
    if(nh.getParam("/lidar_preprocessing/ros_config_output_topic_ground",               g_cfg_str_output_topic_name_ground) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_output_topic_ground");           return false; }
    if(nh.getParam("/lidar_preprocessing/ros_config_output_topic_non_ground",           g_cfg_str_output_topic_name_non_ground) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_output_topic_non_ground");       return false; }
    if(nh.getParam("/lidar_preprocessing/ros_config_range_image_ini_file_path",                     g_cfg_str_range_image_ini_path) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_range_image_ini_file_path");   return false; }
    

    if(nh.getParam("/lidar_preprocessing/ros_config_segmentation_ini_file_path",                     g_cfg_str_segmentation_path) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - ros_config_segmentation_ini_file_path");   return false; }
    

    if(nh.getParam("/lidar_preprocessing/ros_image_flip_x",                             g_b_image_flip_x) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - g_b_image_flip_x"                      );   return false; }
    if(nh.getParam("/lidar_preprocessing/ros_image_flip_y",                             g_b_image_flip_y) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - g_b_image_flip_y"                      );   return false; }
    if(nh.getParam("/lidar_preprocessing/ros_range_img_min_value",                      g_f_range_img_min_value) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - g_f_range_img_min_value"               );   return false; }
    if(nh.getParam("/lidar_preprocessing/ros_range_img_max_value",                      g_f_range_img_max_value) == false)
        {ROS_ERROR("[lidar_preprocessing] ROS Param - g_f_range_img_max_value"               );   return false; }

    std::shared_ptr<CRangeImage> tmp_range_image(new CRangeImage(g_cfg_str_range_image_ini_path));
    range_image = tmp_range_image;

    line_based_segmentation = std::make_unique<CLineBasedSegmentation>();
    line_based_segmentation->Init(g_cfg_str_segmentation_path);
    return true;
}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
pcl::PointCloud<pointType> ousterCloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<pointType> cloudresult;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr i_tmp_ouster_cloud_ptr_(new pcl::PointCloud<OusterPointXYZIRT>());
    pcl::PointCloud<pointType>::Ptr tmp_cloud_result(new pcl::PointCloud<pointType>());

    pcl::moveFromROSMsg(cloudmsg, *i_tmp_ouster_cloud_ptr_);
    tmp_cloud_result->points.resize(i_tmp_ouster_cloud_ptr_->size());
    tmp_cloud_result->is_dense = i_tmp_ouster_cloud_ptr_->is_dense;
    for (size_t i = 0; i < i_tmp_ouster_cloud_ptr_->size(); i++)
    {
        auto &src = i_tmp_ouster_cloud_ptr_->points[i];
        auto &dst = tmp_cloud_result->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.ring = src.ring;
        dst.time = src.t * 1e-9f;
    }

    cloudresult = *tmp_cloud_result;
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


bool estimateGroundPlaneCoef(pcl::PointCloud<PointXYZIRT>::Ptr i_ground_point_ptr, double* o_model_coefficient)
{

    // Create model coefficient outputs
    pcl::ModelCoefficients::Ptr pcl_model_coefficient(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      pcl_inlier_points_indices(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr i_original_point_cloud(new  pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*i_ground_point_ptr, *i_original_point_cloud);

    size_t i_ground_point_num = i_ground_point_ptr->points.size();
    for(size_t i = 0; i < i_ground_point_num ; ++i){
        pcl::PointXYZ temp_point; 
        if (sqrt(i_ground_point_ptr->points[i].x*i_ground_point_ptr->points[i].x + 
                i_ground_point_ptr->points[i].y*i_ground_point_ptr->points[i].y) < 8.0){
            temp_point.x = i_ground_point_ptr->points[i].x;
            temp_point.y = i_ground_point_ptr->points[i].y;
            temp_point.z = i_ground_point_ptr->points[i].z;
            i_original_point_cloud->points.push_back(temp_point);
        }
    }
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> pc_segmentation_algorithm;

    // Optional
    pc_segmentation_algorithm.setOptimizeCoefficients(true);

    // Mandatory
    pc_segmentation_algorithm.setMaxIterations(50);
    pc_segmentation_algorithm.setModelType(pcl::SACMODEL_PLANE);
    pc_segmentation_algorithm.setMethodType(pcl::SAC_RANSAC);
    pc_segmentation_algorithm.setDistanceThreshold(0.1);

    pc_segmentation_algorithm.setInputCloud(i_original_point_cloud);
    pc_segmentation_algorithm.segment(*pcl_inlier_points_indices, *pcl_model_coefficient);

    o_model_coefficient[0] = pcl_model_coefficient->values[0];
    o_model_coefficient[1] = pcl_model_coefficient->values[1];
    o_model_coefficient[2] = pcl_model_coefficient->values[2];
    o_model_coefficient[3] = pcl_model_coefficient->values[3];

    return true;
}


void UpdateRoughnessRefPlane(double* model_coeff)
{
    if (!ground_plane_jsk_.polygons.empty()) 
        ground_plane_jsk_.polygons.clear();
    
    if (!ground_plane_jsk_.likelihood.empty()) 
        ground_plane_jsk_.likelihood.clear();
    
    ground_plane_jsk_.header.frame_id = g_cfg_str_output_topic_frame;
    ground_plane_jsk_.header.stamp = ros::Time::now();

    geometry_msgs::PolygonStamped polygons;
    geometry_msgs::Point32        point;

    polygons.header.frame_id = g_cfg_str_output_topic_frame;
    polygons.header.stamp = ros::Time::now();

    double a = model_coeff[0];
    double b = model_coeff[1];
    double c = model_coeff[2];
    double d = model_coeff[3];

    for(int i = 0; i < 36; i++){
        point.x = 20.0 * sin(i*10.0*M_PI/180.0);
        point.y = 20.0 * cos(i*10.0*M_PI/180.0);
        point.z = -(a*point.x + b*point.y + d) / c;
        polygons.polygon.points.push_back(point);    
    }

    int status = 0;

    ground_plane_jsk_.likelihood.emplace_back(COLOR_MAP[status]);
    ground_plane_jsk_.polygons.emplace_back(polygons);
}

void callbackNode(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    pcl::PointCloud<pointType>::Ptr pcptr_input_lidar_scan( new pcl::PointCloud<pointType> );
    std::vector<int> tmp_indices;

    ros::Time cur_scan_time = msg->header.stamp;
    if(g_cfg_str_lidar_type == "ouster"){
        *pcptr_input_lidar_scan = ousterCloudmsg2cloud<pointType>(*msg);
    }
    else{
        *pcptr_input_lidar_scan = cloudmsg2cloud<pointType>(*msg);
    }

    if( (pcptr_input_lidar_scan->size() > 0) == false )
        { ROS_WARN_STREAM("INPUT POINT CLOUD IS EMPTY!");}
    MeasureComputingTime cal_time;
    
    range_image->SetCloud(pcptr_input_lidar_scan);
    DebugPrintInfoTime("[range_image]- setCloud", cal_time, true);
    // std::shared_ptr<RangeImage<pointType>> ptr_range_image = std::make_shared<RangeImage<pointType>>(range_image);
    // Line Fit Ground Extraction Visualization
    if( line_based_segmentation->SetRangeImage(range_image) )
    {
        DebugPrintInfoTime("[LineFit]- SetRangeImage", cal_time, true);
        line_based_segmentation->FilterGrounds();
        DebugPrintInfoTime("[LineFit]- FilterGround", cal_time, true);
        line_based_segmentation->FilterObjects();
        DebugPrintInfoTime("[Object]- FilterObjects", cal_time, true);
        // line_based_segmentation.FilterPoles();
        // DebugPrintInfoTime("[Object]- FilterPoles", cal_time, true);

        std_msgs::Header img_header;


        cv::Mat mat_range_img ((int)range_image->Height() , (int)range_image->Width(), CV_32F , cv::Scalar::all(FLT_MAX));  // CV_32F: Float 32-bit floating-point numbers(-FLT_MAX ~ FLT_MAX, INF, MIN)
        mat_range_img = line_based_segmentation->m_p_range_image->GetRangeCVImage();
        cv::Mat mat_vis_range_img;
        float f_scale_factor_32F_to_8u_range = (float)std::numeric_limits<uint8_t>::max()/( 80 - 0.5 ); // normalize - max: 80, min: 3 [m]
        mat_range_img.convertTo(mat_vis_range_img, CV_8U, f_scale_factor_32F_to_8u_range );
        cv_bridge::CvImage img_bridge_range;
        sensor_msgs::Image img_msg_range_image;
        std_msgs::Header img_header_range;
        img_header.stamp = ros::Time::now();
        img_bridge_range = cv_bridge::CvImage(img_header_range, sensor_msgs::image_encodings::MONO8, mat_vis_range_img );
        img_bridge_range.toImageMsg(img_msg_range_image);

        // ground mask image pub
        float f_scale_factor_bool_to_8u_ground_mask = (float)std::numeric_limits<uint8_t>::max()/( 1. );
        cv::Mat mat_ground_mask_img ((int)range_image->Height() , (int)range_image->Width(), CV_32F , cv::Scalar::all(0.));  // CV_32F: Float 32-bit floating-point numbers(-FLT_MAX ~ FLT_MAX, INF, MIN)
        mat_ground_mask_img = line_based_segmentation->m_p_range_image->GetMaskCVImage("ground");
        cv::Mat mat_vis_ground_img;
        mat_ground_mask_img.convertTo(mat_vis_ground_img, CV_8U, f_scale_factor_bool_to_8u_ground_mask);
        cv_bridge::CvImage img_bridge_ground;
        sensor_msgs::Image img_msg_ground_mask_image;
        std_msgs::Header img_header_ground_mask;
        img_header_ground_mask.stamp = ros::Time::now();
        img_bridge_ground = cv_bridge::CvImage(img_header_ground_mask, sensor_msgs::image_encodings::MONO8, mat_vis_ground_img );
        img_bridge_ground.toImageMsg(img_msg_ground_mask_image);

        // angle mask image pub
        float f_scale_factor_float_to_8u_angle_mask = (float)std::numeric_limits<uint8_t>::max()/( 30.);
        cv::Mat mat_angle_mask_img ((int)range_image->Height() , (int)range_image->Width(), CV_32F , cv::Scalar::all(0.));  // CV_32F: Float 32-bit floating-point numbers(-FLT_MAX ~ FLT_MAX, INF, MIN)
        mat_angle_mask_img = line_based_segmentation->m_p_range_image->GetFloatMaskCVImage("angle");
        cv::Mat mat_vis_angle_img;
        mat_angle_mask_img.convertTo(mat_vis_angle_img, CV_8U, f_scale_factor_float_to_8u_angle_mask);
        cv_bridge::CvImage img_bridge_angle;
        sensor_msgs::Image img_msg_angle_mask_image;
        std_msgs::Header img_header_angle_mask;
        img_header_angle_mask.stamp = ros::Time::now();
        img_bridge_angle = cv_bridge::CvImage(img_header_angle_mask, sensor_msgs::image_encodings::MONO8, mat_vis_angle_img );
        img_bridge_angle.toImageMsg(img_msg_angle_mask_image);

        g_pub_rospub_ground_mask_image.publish(img_msg_ground_mask_image);

        g_pub_rospub_index_image.publish(img_msg_angle_mask_image);

        g_pub_rospub_range_image.publish(img_msg_range_image);

        // ground pointcloud pub
        sensor_msgs::PointCloud2 ground_point_cloud;
        boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_ground_point_cloud(new pcl::PointCloud<PointXYZIRT>);
        line_based_segmentation->GetGroundPC(pcptr_ground_point_cloud);
        pcl::toROSMsg(*pcptr_ground_point_cloud, ground_point_cloud);
        ground_point_cloud.header.frame_id = g_cfg_str_output_topic_frame;
        ground_point_cloud.header.stamp = cur_scan_time;
        g_pub_ground_points.publish(ground_point_cloud);

        double d_arr_model_coeff[4];
        estimateGroundPlaneCoef(pcptr_ground_point_cloud, d_arr_model_coeff);
        UpdateRoughnessRefPlane(d_arr_model_coeff);
        g_pub_jsk_ground_plane.publish(ground_plane_jsk_);

        // non ground pointcloud pub
        sensor_msgs::PointCloud2 non_ground_point_cloud;
        boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_non_ground_point_cloud(new pcl::PointCloud<PointXYZIRT>);
        line_based_segmentation->GetNonGroundPC(pcptr_non_ground_point_cloud);
        pcl::toROSMsg(*pcptr_non_ground_point_cloud, non_ground_point_cloud);
        non_ground_point_cloud.header.frame_id = g_cfg_str_output_topic_frame;
        non_ground_point_cloud.header.stamp = cur_scan_time;
        g_pub_non_ground_points.publish(non_ground_point_cloud);

        // object pointcloud pub
        sensor_msgs::PointCloud2 object_point_cloud;
        std::vector<boost::shared_ptr<pcl::PointCloud<PointXYZIRT>>> vec_pcptr_objects;
        line_based_segmentation->GetObjectsPC(vec_pcptr_objects);
        boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_object_point_cloud(new pcl::PointCloud<PointXYZIRT>);
        for(int idx = 0; idx < vec_pcptr_objects.size(); idx++)
        {
            for (int point_idx = 0; point_idx < vec_pcptr_objects[idx]->points.size(); point_idx++)
                vec_pcptr_objects[idx]->points[point_idx].intensity = idx;
            *pcptr_object_point_cloud += *(vec_pcptr_objects[idx]);
        }
        pcl::toROSMsg(*pcptr_object_point_cloud, object_point_cloud);
        object_point_cloud.header.stamp = cur_scan_time;
        object_point_cloud.header.frame_id = g_cfg_str_output_topic_frame;
        g_pub_cloud.publish(object_point_cloud);

        // // pole pointcloud pub
        // sensor_msgs::PointCloud2 pole_point_cloud;
        // std::vector<boost::shared_ptr<pcl::PointCloud<PointXYZIRT>>> vec_pcptr_poles;
        // line_based_segmentation->GetPolesPlanePC(vec_pcptr_poles, d_arr_model_coeff);
        // boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_pole_point_cloud(new pcl::PointCloud<PointXYZIRT>);
        // for(int idx = 0; idx < vec_pcptr_poles.size(); idx++)
        // {
        //     for (int point_idx = 0; point_idx < vec_pcptr_poles[idx]->points.size(); point_idx++)
        //         vec_pcptr_poles[idx]->points[point_idx].intensity = idx;
        //     *pcptr_pole_point_cloud += *(vec_pcptr_poles[idx]);
        // }
        // pcl::toROSMsg(*pcptr_pole_point_cloud, pole_point_cloud);
        // pole_point_cloud.header.stamp = cur_scan_time;
        // pole_point_cloud.header.frame_id = g_cfg_str_output_topic_frame;
        // g_pub_pole_points.publish(pole_point_cloud);
    }
    else
        DebugPrintError("[GroundSegmentation] - SetPointCloud is wrong.", true);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_preprocessing");
    ros::NodeHandle nh;
    if (rosInit(nh) == false)
    {
        ROS_ERROR("[lidar_preprocessing] rosInit - Initalizatio has failed.");
        return false;
    }
    ros::Subscriber NodeSubscriber = nh.subscribe<sensor_msgs::PointCloud2>( g_cfg_str_input_topic_name, 1, callbackNode);

    g_pub_cloud                    = nh.advertise<sensor_msgs::PointCloud2>(g_cfg_str_output_topic_name, 100);
    g_pub_ground_points            = nh.advertise<sensor_msgs::PointCloud2>(g_cfg_str_output_topic_name_ground, 100);
    g_pub_non_ground_points        = nh.advertise<sensor_msgs::PointCloud2>(g_cfg_str_output_topic_name_non_ground, 100);
    g_pub_pole_points              = nh.advertise<sensor_msgs::PointCloud2>("/processing/pole_points", 100);
    g_pub_rospub_range_image       = nh.advertise<sensor_msgs::Image>("/range_image",1);
    g_pub_rospub_cluster_image     = nh.advertise<sensor_msgs::Image>("/cluster_image",1);
    g_pub_rospub_index_image       = nh.advertise<sensor_msgs::Image>("/angle_image",1);
    g_pub_rospub_ground_mask_image = nh.advertise<sensor_msgs::Image>("/ground_maks_image",1);
    g_pub_jsk_ground_plane         = nh.advertise<jsk_recognition_msgs::PolygonArray>("/ground_plane", 10);
    ros::spin();

    return 0;
}
