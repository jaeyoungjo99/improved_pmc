/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, kchsu531@gmail.com, wodud3743@gmail.com
 * @file deskewing.cpp
 * @brief pointcloud deskwing input point cloud need time field source file
 * @version 1.0
 * @date 06-10-2022
 * @bug No known bugs
 * @warning No warnings
 */

#include "deskewing/deskewing.hpp"

/*
Brief
- Deskewing the pointcloud.
- Input point cloud's origin and motion's origin have to aligned
Input
- i_pc_skewed: input point cloud
- lidar_time_sec: input piont cloud's timestamp
- time_motion_que: sensor's motion
Output
- o_pc_deskewd: output point cloud
Return
- true or false : true - normaly operated, false - occure some error
*/
template <typename pointType>
bool CDeskewing::Deskewing(  const pcl::PointCloud<pointType>& i_pc_skewed,
                            double lidar_time_sec,
                            std::deque<std::pair<double, std::vector<double>>> time_motion_que,
                            pcl::PointCloud<pointType>& o_pc_deskewd)
{
    // WARN: lidar_time_sec is end time of scan
    size_t cloudSize = i_pc_skewed.points.size();
    size_t motionSize = time_motion_que.size();

    if (motionSize < 2 || cloudSize == 0)
        return false;

    double d_scan_dt_sec =  i_pc_skewed.points[cloudSize-1].time - i_pc_skewed.points[0].time;
    std::deque<std::vector<double>> pose_deque;

    double d_motion_dt_sec = 0.;
    double d_imu_time_now_sec = 0;
    double d_imu_prev_time_sec = 0.;
    double d_scan_start_time_sec = lidar_time_sec - d_scan_dt_sec;

    std::vector<double> v_tmp_state(8);             // time, q0, q1, q2, q3, x, y, x
    std::vector<double> v_quat_state = {1.,0.,0.,0};// q0, q1, q2, q3
    std::vector<double> v_xyz_state(3);             // x, y, z

    /*
        if oldest motion time is later than scan start time,
        or latest motion time is earlier than scan start time, return
        This ensure time_motion_que has at least two motion data before and after lidar_scan_start time
    */
    if (time_motion_que[0].first > d_scan_start_time_sec || time_motion_que[motionSize-1].first < d_scan_start_time_sec )
        return false;

    // Search motion from oldest to latest
    for (size_t motion_idx = 0 ; motion_idx < motionSize ; ++motion_idx)
    {
        d_imu_time_now_sec = time_motion_que[motion_idx].first;

        // Skip motion data before scan start time
        if (d_imu_time_now_sec < d_scan_start_time_sec)
            continue;

        // Check first motion after scan start time
        if (d_imu_time_now_sec >= d_scan_start_time_sec && motion_idx-1 >= 0 )
        {
            if(time_motion_que[motion_idx-1].first < d_scan_start_time_sec)
            {
                v_tmp_state[0] = d_scan_start_time_sec; // motion time
                v_tmp_state[1] = v_quat_state[0];
                v_tmp_state[2] = v_quat_state[1];
                v_tmp_state[3] = v_quat_state[2];
                v_tmp_state[4] = v_quat_state[3];

                v_tmp_state[5] = v_xyz_state[0];
                v_tmp_state[6] = v_xyz_state[1];
                v_tmp_state[7] = v_xyz_state[2];
                d_imu_prev_time_sec = d_scan_start_time_sec;
                pose_deque.push_back(v_tmp_state);

                continue;
            }
        }
        // check motion over lidar end time. This might never happen.
        if(d_imu_time_now_sec > lidar_time_sec && motion_idx-1 >= 0) // motion time later than lidar end time
        {
            d_motion_dt_sec = lidar_time_sec - time_motion_que[motion_idx-1].first;
            double d_arr_prev_state[4] = {v_quat_state[0],v_quat_state[1],v_quat_state[2],v_quat_state[3]};
            double d_arr_motion_rpy[3] = {time_motion_que[motion_idx-1].second[3],
                                          time_motion_que[motion_idx-1].second[4],
                                          time_motion_que[motion_idx-1].second[5]};

            v_quat_state = quaternionPredictionModel(d_arr_prev_state,d_arr_motion_rpy,d_motion_dt_sec);

            v_xyz_state[0] += time_motion_que[motion_idx-1].second[0] * d_motion_dt_sec;
            v_xyz_state[1] += time_motion_que[motion_idx-1].second[1] * d_motion_dt_sec;
            v_xyz_state[2] += time_motion_que[motion_idx-1].second[2] * d_motion_dt_sec;

            v_tmp_state[0] = lidar_time_sec;    // motion time
            v_tmp_state[1] = v_quat_state[0];
            v_tmp_state[2] = v_quat_state[1];
            v_tmp_state[3] = v_quat_state[2];
            v_tmp_state[4] = v_quat_state[3];

            v_tmp_state[5] = v_xyz_state[0];
            v_tmp_state[6] = v_xyz_state[1];
            v_tmp_state[7] = v_xyz_state[2];

            pose_deque.push_back(v_tmp_state);

            break;  // Motion time is over lidar end time. End motion_que search.
        }

        d_motion_dt_sec = d_imu_time_now_sec - d_imu_prev_time_sec;
        if(d_motion_dt_sec < __FLT_MIN__){  // Check current and last motion time gap goes 0.
            continue;
        }

        double d_arr_prev_state[4] = {v_quat_state[0],v_quat_state[1],v_quat_state[2],v_quat_state[3]};
        double d_arr_motion_rpy[3] = {time_motion_que[motion_idx-1].second[3],
                                time_motion_que[motion_idx-1].second[4],
                                time_motion_que[motion_idx-1].second[5]};

        v_quat_state = quaternionPredictionModel(d_arr_prev_state,d_arr_motion_rpy,d_motion_dt_sec);

        v_xyz_state[0] += time_motion_que[motion_idx-1].second[0] * d_motion_dt_sec;
        v_xyz_state[1] += time_motion_que[motion_idx-1].second[1] * d_motion_dt_sec;
        v_xyz_state[2] += time_motion_que[motion_idx-1].second[2] * d_motion_dt_sec;

        v_tmp_state[0] = d_imu_time_now_sec;    // motion time
        v_tmp_state[1] = v_quat_state[0];
        v_tmp_state[2] = v_quat_state[1];
        v_tmp_state[3] = v_quat_state[2];
        v_tmp_state[4] = v_quat_state[3];

        v_tmp_state[5] = v_xyz_state[0];
        v_tmp_state[6] = v_xyz_state[1];
        v_tmp_state[7] = v_xyz_state[2];
        d_imu_prev_time_sec = d_imu_time_now_sec;

        pose_deque.push_back(v_tmp_state);
    }

    o_pc_deskewd = i_pc_skewed;

    double d_point_time_sec, d_dt_motion_to_point_sec, d_dt_motion_sec;

    int deque_size = pose_deque.size();

    // quaternion
    double quat_0,quat_1,quat_2,quat_3;
    double d_time_ratio;

    Eigen::Quaterniond quat_prev(1.,0.,0.,0.);
    Eigen::Quaterniond quat_next(1.,0.,0.,0.);
    Eigen::Quaterniond quat_slerp(1.,0.,0.,0.);

    int deque_idx = 0;
    int i_max_deque_idx = deque_size-1;

    double x_prev,y_prev,z_prev;
    double x_next,y_next,z_next;
    double x_est, y_est, z_est;

    for (size_t point_idx = 0 ; point_idx < cloudSize ; ++point_idx ){
        d_point_time_sec = d_scan_start_time_sec + i_pc_skewed.points[point_idx].time;

        if(point_idx == 0)
        {
            quat_prev = Eigen::Quaterniond(pose_deque[deque_idx][1], pose_deque[deque_idx][2], pose_deque[deque_idx][3], pose_deque[deque_idx][4]);
            quat_next = Eigen::Quaterniond(pose_deque[deque_idx+1][1], pose_deque[deque_idx+1][2], pose_deque[deque_idx+1][3], pose_deque[deque_idx+1][4]);
            x_prev = pose_deque[deque_idx][5];
            y_prev = pose_deque[deque_idx][6];
            z_prev = pose_deque[deque_idx][7];

            x_next = pose_deque[deque_idx+1][5];
            y_next = pose_deque[deque_idx+1][6];
            z_next = pose_deque[deque_idx+1][7];
        }

        if ((deque_idx + 1) <= i_max_deque_idx)
        {
            if (d_point_time_sec > pose_deque[deque_idx+1][0])
            {
                deque_idx = deque_idx+1;
                quat_prev = Eigen::Quaterniond(pose_deque[deque_idx][1], pose_deque[deque_idx][2], pose_deque[deque_idx][3], pose_deque[deque_idx][4]);
                quat_next = Eigen::Quaterniond(pose_deque[deque_idx+1][1], pose_deque[deque_idx+1][2], pose_deque[deque_idx+1][3], pose_deque[deque_idx+1][4]); 

                x_prev = pose_deque[deque_idx][5];
                y_prev = pose_deque[deque_idx][6];
                z_prev = pose_deque[deque_idx][7];

                x_next = pose_deque[deque_idx+1][5];
                y_next = pose_deque[deque_idx+1][6];
                z_next = pose_deque[deque_idx+1][7];
            }
        }

        d_dt_motion_sec = pose_deque[deque_idx+1][0] - pose_deque[deque_idx][0];
        d_dt_motion_to_point_sec = d_point_time_sec - pose_deque[deque_idx][0];

        d_time_ratio = d_dt_motion_to_point_sec/d_dt_motion_sec;

        quat_slerp = quat_prev.slerp(d_time_ratio,quat_next);

        x_est = (x_prev*(d_dt_motion_sec - d_dt_motion_to_point_sec) + x_next*d_dt_motion_to_point_sec) / d_dt_motion_sec;
        y_est = (y_prev*(d_dt_motion_sec - d_dt_motion_to_point_sec) + y_next*d_dt_motion_to_point_sec) / d_dt_motion_sec;
        z_est = (z_prev*(d_dt_motion_sec - d_dt_motion_to_point_sec) + z_next*d_dt_motion_to_point_sec) / d_dt_motion_sec;

        /// quaternion based RT
        Eigen::Quaterniond p(0.,i_pc_skewed.points[point_idx].x,i_pc_skewed.points[point_idx].y,i_pc_skewed.points[point_idx].z );
        Eigen::Quaterniond quat_rotation = quat_slerp * p * quat_slerp.inverse();
        o_pc_deskewd.points[point_idx].x = quat_rotation.vec()(0) + x_est;
        o_pc_deskewd.points[point_idx].y = quat_rotation.vec()(1) + y_est;
        o_pc_deskewd.points[point_idx].z = quat_rotation.vec()(2) + z_est;
    }
    return true;
}


std::vector<double> CDeskewing::PrediceModelCV(double prev_state[],
                        double angular_velocity[],
                        double motion_dt)
{
    double prev_q0, prev_q1, prev_q2, prev_q3;
    double q0, q1, q2, q3;
    double roll, pitch, yaw;

    double angular_vel_x = angular_velocity[0];
    double angular_vel_y = angular_velocity[1];
    double angular_vel_z = angular_velocity[2];
    std::vector<double> output_rpy_vector_rad;
    // euler to quarternion conversion:

    prev_q0 = cos(prev_state[0]/2.0) * cos(prev_state[1]/2.0) * cos(prev_state[2]/2.0) +
              sin(prev_state[0]/2.0) * sin(prev_state[1]/2.0) * sin(prev_state[2]/2.0);

    prev_q1 = sin(prev_state[0]/2.0) * cos(prev_state[1]/2.0) * cos(prev_state[2]/2.0) -
              cos(prev_state[0]/2.0) * sin(prev_state[1]/2.0) * sin(prev_state[2]/2.0);

    prev_q2 = cos(prev_state[0]/2.0) * sin(prev_state[1]/2.0) * cos(prev_state[2]/2.0) +
              sin(prev_state[0]/2.0) * cos(prev_state[1]/2.0) * sin(prev_state[2]/2.0);

    prev_q3 = cos(prev_state[0]/2.0) * cos(prev_state[1]/2.0) * sin(prev_state[2]/2.0) -
              sin(prev_state[0]/2.0) * sin(prev_state[1]/2.0) * cos(prev_state[2]/2.0);

    double quat_ang = sqrt(angular_vel_x*angular_vel_x +
                            angular_vel_y*angular_vel_y +
                            angular_vel_z*angular_vel_z) * motion_dt;

    if(abs(quat_ang) < __FLT_MIN__)
    {
        q0 = prev_q0;
        q1 = prev_q1;
        q2 = prev_q2;
        q3 = prev_q3;
    }
    else
    {
        q0 = cos(quat_ang/2.0) * prev_q0 +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(-angular_vel_x * prev_q1 - angular_vel_y*prev_q2 - angular_vel_z*prev_q3))*motion_dt;

        q1 = cos(quat_ang/2.0) * prev_q1 +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_x * prev_q0 + angular_vel_z*prev_q2 - angular_vel_y*prev_q3))*motion_dt;

        q2 = cos(quat_ang/2.0) * prev_q2 +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_y * prev_q0 - angular_vel_z*prev_q1 + angular_vel_x*prev_q3))*motion_dt;

        q3 = cos(quat_ang/2.0) * prev_q3 +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_z * prev_q0 + angular_vel_y*prev_q1 - angular_vel_x*prev_q2))*motion_dt;
    }

    roll = atan2( 2.0 * (q0*q1 + q2*q3) , 1 - 2.0 * (q1*q1 + q2*q2) );
    pitch = asin( 2.0 * (q0*q2 - q3*q1));
    yaw = atan2( 2.0 * (q0*q3 + q1*q2) , 1 - 2.0 * (q2*q2 + q3*q3) );

    output_rpy_vector_rad.push_back(roll);
    output_rpy_vector_rad.push_back(pitch);
    output_rpy_vector_rad.push_back(yaw);

    return output_rpy_vector_rad;
}

std::vector<double> CDeskewing::quaternionPredictionModel(double prev_state[],
                        double angular_velocity[], // edit: change variable name with unit
                        double motion_dt) // edit: change variable name with unit
{
    double q0, q1, q2, q3;

    double angular_vel_x = angular_velocity[0]; // edit: change variable name with unit
    double angular_vel_y = angular_velocity[1]; // edit: change variable name with unit
    double angular_vel_z = angular_velocity[2]; // edit: change variable name with unit
    std::vector<double> output_quat_vector;
    // euler to quarternion conversion:

    double quat_ang = sqrt(angular_vel_x*angular_vel_x +
                            angular_vel_y*angular_vel_y +
                            angular_vel_z*angular_vel_z) * motion_dt;

    if(quat_ang == 0)
    {
        q0 = prev_state[0];
        q1 = prev_state[1];
        q2 = prev_state[2];
        q3 = prev_state[3];
    }
    else
    {
        q0 = cos(quat_ang/2.0) * prev_state[0] +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(-angular_vel_x * prev_state[1] - angular_vel_y*prev_state[2] - angular_vel_z*prev_state[3]))*motion_dt;

        q1 = cos(quat_ang/2.0) * prev_state[1] +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_x * prev_state[0] + angular_vel_z*prev_state[2] - angular_vel_y*prev_state[3]))*motion_dt;

        q2 = cos(quat_ang/2.0) * prev_state[2] +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_y * prev_state[0] - angular_vel_z*prev_state[1] + angular_vel_x*prev_state[3]))*motion_dt;

        q3 = cos(quat_ang/2.0) * prev_state[3] +
            0.5*(sin(quat_ang/2.0)/(quat_ang/2.0)*(angular_vel_z * prev_state[0] + angular_vel_y*prev_state[1] - angular_vel_x*prev_state[2]))*motion_dt;
    }

    output_quat_vector.push_back(q0);
    output_quat_vector.push_back(q1);
    output_quat_vector.push_back(q2);
    output_quat_vector.push_back(q3);

    return output_quat_vector;
}

double CDeskewing::AngleDiff_deg(double dRef_deg, double dRel_deg)
{
	double dAngleDiffBuff_deg = dRel_deg - dRef_deg;

	// calculate angle difference
	if (dAngleDiffBuff_deg > 180.)
		dAngleDiffBuff_deg = dAngleDiffBuff_deg - 360.;

	else if (dAngleDiffBuff_deg < -180.)
		dAngleDiffBuff_deg = dAngleDiffBuff_deg + 360.;

	return dAngleDiffBuff_deg;
}