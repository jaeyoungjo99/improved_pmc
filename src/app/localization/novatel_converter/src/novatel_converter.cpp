#include "novatel_converter.hpp"

NovatelConverter::NovatelConverter(){
    Init();
};
NovatelConverter::~NovatelConverter(){};

void NovatelConverter::Init(){
    ROS_INFO("Init function");
    NodeHandle nh;

    if (!nh.getParam("/topic_name/vehicle_state", str_ego_pose_msg_)) {
        str_ego_pose_msg_ = "";
    }  
    if (!nh.getParam("/common_variable/dataset", str_dataset_)) {
        str_dataset_ = "novatel";
    }   
    if (!nh.getParam("/common_variable/projection_mode", projection_mode_)) {
        projection_mode_ = "";
    }  
    if (!nh.getParam("/common_variable/use_init_lat_lon", b_use_init_lat_lon_)) {
        b_use_init_lat_lon_ = false;
    }  
    if (!nh.getParam("/common_variable/ref_latitude", ref_latitude_)) {
        ref_latitude_ = 37.0;
    }  
    if (!nh.getParam("/common_variable/ref_longitude", ref_longitude_)) {
        ref_longitude_ = 128;
    }  
    if (!nh.getParam("/common_variable/novatel_heading_bias_deg", novatel_heading_bias_deg_)) {
        novatel_heading_bias_deg_ = 0.0;
    }  

    ROS_INFO_STREAM("projection_mode_ "<<projection_mode_);
    ROS_INFO_STREAM("str_dataset_ "<<str_dataset_);
    ROS_INFO_STREAM("novatel_heading_bias_deg_ "<<novatel_heading_bias_deg_);

    if(b_use_init_lat_lon_ = false){
        ROS_INFO_STREAM("ref_latitude_ "<<ref_latitude_);
        ROS_INFO_STREAM("ref_longitude_ "<<ref_longitude_);

        b_is_ref_init_ = true;
    }

    sub_novatel_inspvax_ = nh.subscribe("novatel/oem7/inspvax", 1, &NovatelConverter::CallbackINSPVAX, this);
    sub_kitti_geo_ = nh.subscribe("/ground_truth", 1, &NovatelConverter::CallbackKittiGeo, this);
    pub_vehicle_state_ = nh.advertise<autoku_msgs::VehicleState>("/app/loc/vehicle_state", 10);
    pub_ego_marker_ = nh.advertise<visualization_msgs::Marker>("/app/loc/ego_marker", 1);
    pub_ego_cov_marker_ = nh.advertise<visualization_msgs::Marker>("/app/loc/ego_cov_marker", 1);
    pub_rviz_novatel_pos_type_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/app/loc/pos_type_text", 1);

    pub_ego_geometry_msg_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(str_ego_pose_msg_, 10);

    
    o_ego_marker_msgs_.type = visualization_msgs::Marker::CUBE;
    o_ego_marker_msgs_.pose.position.x = 2.7/2.0;
    o_ego_marker_msgs_.pose.position.y = 0.0;
    o_ego_marker_msgs_.pose.position.z = 1.44/2.0;

    o_ego_marker_msgs_.scale.x = 4.57;
    o_ego_marker_msgs_.scale.y = 1.8;
    o_ego_marker_msgs_.scale.z = 1.44;

    o_ego_marker_msgs_.color.a = 0.5;
    o_ego_marker_msgs_.color.g = 1.0;
    

    ROS_INFO("Init function Done");

}

void NovatelConverter::Run(){
    ros::Rate rate(200); // 100 Hz

    while (ros::ok()) {
        // Your code to be executed every 10 seconds
        UpdateVehicleState();

        if(b_is_new_msg_ == true){
            Publish();
            b_is_new_msg_ = false;
        }

        ros::spinOnce(); // Handle ROS callbacks
        rate.sleep();    // Sleep to control the loop rate
    }
}

void NovatelConverter::UpdateVehicleState(){
    
    if(b_is_ref_init_ == false) return;


    if(str_dataset_ == "novatel"){
        o_rviz_pos_type_ = UpdateNovatelPosTypeText(i_novatel_inspvax_);
        UpdateEgoCovMarker(i_novatel_inspvax_);
        UpdateEgoGeo(i_novatel_inspvax_);
    }
    else{
        geometry_msgs::PoseWithCovarianceStamped ego_geo;

        ego_geo.header.stamp = i_kitti_geo_.header.stamp;
        ego_geo.header.frame_id = "ego_frame";
        ego_geo.pose.pose.position = i_kitti_geo_.pose.position;
        ego_geo.pose.pose.orientation = i_kitti_geo_.pose.orientation;
        
        o_novatel_ego_geo_ = ego_geo;
    }

}

void NovatelConverter::UpdateEgoCovMarker(const novatel_oem7_msgs::INSPVAX inspvax_msg){
    o_ego_cov_marker_msgs_.header.frame_id = "ego_frame";
    o_ego_cov_marker_msgs_.header.stamp = inspvax_msg.header.stamp;

    o_ego_cov_marker_msgs_.type = visualization_msgs::Marker::CYLINDER;
    o_ego_cov_marker_msgs_.pose.position.x = 0.0;
    o_ego_cov_marker_msgs_.pose.position.y = 0.0;
    o_ego_cov_marker_msgs_.pose.position.z = 0.2;

    o_ego_cov_marker_msgs_.scale.x = inspvax_msg.longitude_stdev;
    o_ego_cov_marker_msgs_.scale.y = inspvax_msg.latitude_stdev;
    o_ego_cov_marker_msgs_.scale.z = 0.1;

    o_ego_cov_marker_msgs_.color.a = 0.4;
    o_ego_cov_marker_msgs_.color.r = 0.3;
    o_ego_cov_marker_msgs_.color.g = 1.0;
    o_ego_cov_marker_msgs_.color.b = 0.3;

    o_ego_cov_marker_msgs_.pose.orientation.w = 1.0;

    o_ego_marker_msgs_.header.frame_id = "ego_frame";
    o_ego_marker_msgs_.header.stamp = inspvax_msg.header.stamp;

}

void NovatelConverter::UpdateEgoGeo(const novatel_oem7_msgs::INSPVAX inspvax_msg){
    geometry_msgs::PoseWithCovarianceStamped ego_geo;

    ego_geo.header.stamp = inspvax_msg.header.stamp;
    ego_geo.header.frame_id = "ego_frame";


    lanelet::GPSPoint v_gps_point;
    v_gps_point.lat = inspvax_msg.latitude;
    v_gps_point.lon = inspvax_msg.longitude;
    v_gps_point.ele = inspvax_msg.height;

    lanelet::BasicPoint3d projected_point;

    if ( v_gps_point.lat == 0 && v_gps_point.lon == 0 ) {

    }

    if(projection_mode_ == "UTM"){
        lanelet::projection::UtmProjector utm_projector_(lanelet::Origin({ref_latitude_, ref_longitude_}));
        projected_point = utm_projector_.forward(v_gps_point);
    }
    else{
        lanelet::projection::LocalCartesianProjector cartesian_projector_(lanelet::Origin({ref_latitude_, ref_longitude_}));
        projected_point = cartesian_projector_.forward(v_gps_point);
    }

    ego_geo.pose.pose.position.x = projected_point.x();
    ego_geo.pose.pose.position.y = projected_point.y();
    ego_geo.pose.pose.position.z = inspvax_msg.height - ref_height_;

    double roll = inspvax_msg.roll * M_PI / 180.0;
    double pitch = -inspvax_msg.pitch * M_PI / 180.0;
    double yaw = (90.0 - inspvax_msg.azimuth + novatel_heading_bias_deg_) * M_PI / 180.0;


    // Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    //                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());



    ego_geo.pose.pose.orientation.x = quat.x();
    ego_geo.pose.pose.orientation.y = quat.y();
    ego_geo.pose.pose.orientation.z = quat.z();
    ego_geo.pose.pose.orientation.w = quat.w();

    ego_geo.pose.covariance[0] = inspvax_msg.latitude_stdev;
    ego_geo.pose.covariance[7] = inspvax_msg.longitude_stdev;
    ego_geo.pose.covariance[14] = inspvax_msg.azimuth_stdev;
    
    o_novatel_ego_geo_ = ego_geo;

}


jsk_rviz_plugins::OverlayText NovatelConverter::UpdateNovatelPosTypeText(const novatel_oem7_msgs::INSPVAX inspvax_msg) {
    jsk_rviz_plugins::OverlayText rviz_pos_type;
    rviz_pos_type.left      = 100;
    rviz_pos_type.top       = 0;
    rviz_pos_type.width     = 1000;
    rviz_pos_type.height    = 300;
    rviz_pos_type.text      = "PosType: ";
    rviz_pos_type.text_size = 16;

    rviz_pos_type.fg_color.r = 1.0f;
    rviz_pos_type.fg_color.g = 0.0f;
    rviz_pos_type.fg_color.b = 0.0f;
    rviz_pos_type.fg_color.a = 1.0f;


    rviz_pos_type.text = rviz_pos_type.text + std::to_string(inspvax_msg.pos_type.type);
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Lat_std:\t"+ std::to_string(inspvax_msg.latitude_stdev);
    rviz_pos_type.text = rviz_pos_type.text + "\n" + "Lon_std:\t"+ std::to_string(inspvax_msg.longitude_stdev);


    return rviz_pos_type;
}

void NovatelConverter::Publish(){
    pub_vehicle_state_.publish(o_vehicle_state_);   
    pub_ego_marker_.publish(o_ego_marker_msgs_);
    pub_ego_cov_marker_.publish(o_ego_cov_marker_msgs_);
    pub_rviz_novatel_pos_type_.publish(o_rviz_pos_type_);
    pub_ego_geometry_msg_.publish(o_novatel_ego_geo_);
}

void NovatelConverter::CallbackINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg){
    i_novatel_inspvax_ = *msg;

    if(b_is_ref_init_ == false){
        ref_latitude_ = i_novatel_inspvax_.latitude;
        ref_longitude_ = i_novatel_inspvax_.longitude;
        ref_height_ = i_novatel_inspvax_.height;
        b_is_ref_init_ = true;
    }
    b_is_new_msg_ = true;
}

void NovatelConverter::CallbackKittiGeo(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    i_kitti_geo_ = *msg;
    b_is_new_msg_ = true;
    b_is_ref_init_ = true;
}

int main(int argc, char **argv) {
    std::string node_name = "novatel_converter";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    NovatelConverter NC;

    NC.Run();

    return 0;
}

