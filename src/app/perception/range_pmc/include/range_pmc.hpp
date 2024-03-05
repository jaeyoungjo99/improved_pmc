#ifndef __RANGE_PMC__
#define __RANGE_PMC__
#pragma once

// STD header
#include <iostream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <chrono>
#include <execution>
#include <omp.h>
#include <math.h>
#include <omp.h>


#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

// Algorithm header
#include <types.h>
// #include <parallel_q.h>

// 

#include <Eigen/LU>



using namespace std;

// 
#define MAX_2D_N       (324000)     // MAX_1D * MAX_1D_HALF
#define MAX_1D         (900)       // maximum horizontal point of Range Image.     2pi / horizontal_resolution
#define MAX_1D_HALF    (360)        // maximum vertical channel num of Range Image. pi / vertical_resolution
#define MAX_POINT      (524288)

struct range_pmc_params{
    float f_horizontal_resolution;
    float f_vertical_resolution;
    float f_min_range;
    float f_max_range;
    float f_range_threshold;
    int i_max_range_image_num;
    float f_min_key_frame_time;
    float f_min_key_frame_rot;
    float f_fov_up;
    float f_fov_down;
    float f_fov_left;
    float f_fov_right;
    int i_min_occluded_num;
    int i_neighbor_pixel_max;

    float f_ground_angle;

    std::vector<float> vec_f_ego_to_lidar;
};


enum dyn_obj_flg {STATIC, CASE1, CASE2, CASE3, SELF, UNCERTAIN, INVALID};

// Point struct in range image
struct point_soph
{
    int         hor_ind;
    V3F         vec; // horizontal angle, vertical angle, distance
    int         ver_ind;
    int         cluster_ind;
    bool        ground;
    int         position; // Point index in range image
    double      time;
    dyn_obj_flg  dyn;

    float       incident; // Incident angle. 0->normal is parrell to beam, pi/2 --> normal is perpendicular to beam

    M3D         rot;        // 이 포인트 집합의 대표 rot
    V3D         transl;     // 이 포인트 집합의 대표 transl
    V3D         glob;       // 이 포인트의 global 좌표상의 위치
    V3D         local;      // 이 포인트의 라이다 좌표상의 위치 

    float       intensity;

    typedef boost::shared_ptr<point_soph> Ptr;
    point_soph(V3D & point, float & hor_resolution_max, float & ver_resolution_max) // Constructor with resolution and point xyz
    {
        vec(2)     = float(point.norm()); // range
        vec(0)     = atan2f(float(point(1)), float(point(0))); // horizontal angle
        vec(1)     = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2))); // vertical angle
        hor_ind    = floor((vec(0) + M_PI) / hor_resolution_max); // 
        ver_ind    = floor((vec(1) + 0.5 * M_PI) / ver_resolution_max);
        position   = hor_ind * MAX_1D_HALF + ver_ind;
        cluster_ind = -1;
        ground      = false;
        incident   = -1;
        time       = -1;
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        local.setZero();
    };
    point_soph() // Default Constructor
    {
        vec.setZero();
        hor_ind  = ver_ind = position = 0;
        cluster_ind = -1;
        ground      = false;
        incident   = -1;
        time       = -1;
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        local.setZero();
    };
    point_soph(V3F s, int ind1, int ind2, int pos) // Constructor with s(hor_angle, ver_angle, range) image_index, range_index
    {
        vec = s;
        hor_ind = ind1;
        ver_ind = ind2;
        cluster_ind = -1;
        ground     = false;
        incident   = -1;
        position = pos;
        time = -1;
        transl.setZero();
        glob.setZero();
        rot.setOnes();
        local.setZero();
    };
    point_soph(const point_soph & cur) // Copy
    {   
        vec = cur.vec;
        hor_ind  = cur.hor_ind;
        ver_ind  = cur.ver_ind;
        cluster_ind = cur.cluster_ind;
        ground      = cur.ground;
        incident    = cur.incident;
        position  = cur.position;
        time = cur.time;
        transl = cur.transl;
        glob = cur.glob;
        rot = cur.rot;
        dyn = cur.dyn;
        local = cur.local;
    };

    ~point_soph(){
    };

    void GetVec(V3D & point, float & hor_resolution_max, float & ver_resolution_max) // Function with resolution and point xyz
    {
        vec(2)    = float(point.norm());
        vec(0)    = atan2f(float(point(1)), float(point(0)));
        vec(1)    = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
        hor_ind   = floor((vec(0) + M_PI) / hor_resolution_max);
        ver_ind   = floor((vec(1) + 0.5 * M_PI) / ver_resolution_max);
        position  = hor_ind * MAX_1D_HALF + ver_ind;
        ground    = false;
        incident  = -1;
        cluster_ind = -1;
    };

    void reset()
    {
        // ??
    };

};

// Range image class

typedef std::vector<std::vector<point_soph*>>  RangeImage2D; // image --> pixel --> points in pixel

class RangeImage
{
public:
    RangeImage2D        range_image;
    double              time;       // time of range image
    int                 map_index; // Index of current range image
    M3D                 project_R; // Rotation of range image
    V3D                 project_T; // Transform of range image
    std::vector<point_soph::Ptr>    point_sopth_pointer; // To manage points in range image
    float*           min_range_static = nullptr;
    float*           min_range_all = nullptr;
    float*           max_range_all = nullptr;
    float*           max_range_static = nullptr;
    int*             max_range_index_all = nullptr;
    int*             min_range_index_all = nullptr;

    std::vector<int> index_vector; // Index of point in rangeimage

    std::vector<std::vector<int>>   cluster_index_vector; // Cluster index --> point index

    typedef boost::shared_ptr<RangeImage> Ptr;

    RangeImage()
    {
        range_image.assign(MAX_2D_N, std::vector<point_soph*>()); // max_point * point_array vector
        
        time = 0.;
        project_R.setIdentity(3,3);
        project_T.setZero(3, 1);

        min_range_static = new float[MAX_2D_N];
        min_range_all = new float[MAX_2D_N];
        max_range_all = new float[MAX_2D_N];
        max_range_static = new float[MAX_2D_N];
        fill_n(min_range_static, MAX_2D_N, 0.0);
        fill_n(min_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_static, MAX_2D_N, 0.0);
        max_range_index_all = new int[MAX_2D_N];
        min_range_index_all = new int[MAX_2D_N];
        fill_n(min_range_index_all, MAX_2D_N, -1);
        fill_n(max_range_index_all, MAX_2D_N, -1);

        map_index = -1;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) { // Reset point index from 0 to MAX_2D_N-1
            index_vector[i] = i;
        }
        
    }

    RangeImage(M3D rot, V3D transl, double cur_time, int frame)
    {   
        range_image.assign(MAX_2D_N, std::vector<point_soph*>());       
        time = cur_time;
        project_R = rot;
        project_T = transl;
        min_range_static = new float[MAX_2D_N];
        min_range_all = new float[MAX_2D_N];
        max_range_all = new float[MAX_2D_N];
        max_range_static = new float[MAX_2D_N];
        fill_n(min_range_static, MAX_2D_N, 0.0);
        fill_n(min_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_static, MAX_2D_N, 0.0);
        max_range_index_all = new int[MAX_2D_N];
        min_range_index_all = new int[MAX_2D_N];
        fill_n(min_range_index_all, MAX_2D_N, -1);
        fill_n(max_range_index_all, MAX_2D_N, -1);
        map_index = frame;
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    RangeImage(const RangeImage & cur)
    {   
        range_image = cur.range_image;       
        time = cur.time;
        project_R = cur.project_R;
        project_T = cur.project_T;
        point_sopth_pointer = cur.point_sopth_pointer;
        min_range_static = new float[MAX_2D_N];
        min_range_all = new float[MAX_2D_N];
        max_range_all = new float[MAX_2D_N];
        max_range_static = new float[MAX_2D_N];   
        fill_n(min_range_static, MAX_2D_N, 0.0);
        fill_n(min_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_static, MAX_2D_N, 0.0);
        max_range_index_all = new int[MAX_2D_N];       
        min_range_index_all = new int[MAX_2D_N];
        map_index = cur.map_index;      
        for(int i = 0; i < MAX_2D_N; i++)
        {
            min_range_static[i] = cur.min_range_static[i];
            max_range_static[i] = cur.max_range_static[i];
            min_range_all[i] = cur.min_range_all[i];
            max_range_all[i] = cur.max_range_all[i];
            max_range_index_all[i] = cur.max_range_index_all[i];
            min_range_index_all[i] = cur.min_range_index_all[i];
        }
        index_vector.assign(MAX_2D_N, 0);
        for (int i = 0; i < MAX_2D_N; i++) {
            index_vector[i] = i;
        }
    }

    ~RangeImage()
    {
        if(min_range_static != nullptr) delete [] min_range_static;
        if(min_range_all  != nullptr) delete [] min_range_all;
        if(max_range_all  != nullptr) delete [] max_range_all;
        if(max_range_static  != nullptr) delete [] max_range_static;
        if(max_range_index_all  != nullptr) delete [] max_range_index_all;
        if(min_range_index_all  != nullptr) delete [] min_range_index_all;
    }

    void Reset(M3D rot, V3D transl, double cur_time, int frame)
    {   
        time = cur_time;
        project_R = rot;
        project_T = transl;
        map_index = frame;
        // std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
        // {
        //     range_image[i].clear();
        // });

        for (auto i : index_vector)
        {
            range_image[i].clear();
        }
        fill_n(min_range_static, MAX_2D_N, 0.0);
        fill_n(min_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_all, MAX_2D_N, 0.0);
        fill_n(max_range_static, MAX_2D_N, 0.0);
        fill_n(max_range_index_all, MAX_2D_N, -1);
        fill_n(min_range_index_all, MAX_2D_N, -1);
    }

};

class RangePmc{
    public:
    RangePmc();
    ~RangePmc();

    void Init(range_pmc_params params);
    void Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time);
    void GetFilteredPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_pmc_xyzrgb_pcptr);

    public:

    // Test
    bool Case1(point_soph & p);
    bool Case1Enter(const point_soph & p, const RangeImage &image_info, bool & is_valid);
    

    // Range Image Generation
    
    // 시간, rot, transl를 고려하여 range image 생성
    void GenerateRangeImage(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl); 
    void GenerateRangeImage(point_soph* &points, double cur_time, M3D rot, V3D transl, int len, RangeImage::Ptr range_image_pointer); 

    void GroundSegmentation(RangeImage::Ptr range_image_ptr);
    void ObjectSegmentation(RangeImage::Ptr range_image_ptr);

    // Utils
    void SphericalProjection(point_soph &p, int range_index, const M3D &rot, const V3D &transl, point_soph &p_spherical);
    bool KeyFrameCheck(double cur_time, M3D rot, V3D transl);

    bool CheckVerFoV(const point_soph & p, const RangeImage &image_info);
    bool CheckNeighbor(const point_soph & p, const RangeImage &image_info, float &max_range, float &min_range);

    // Output
    void OutputPmcPC(std::vector<point_soph*> &points);
    void OutputRangeImagesPc();
    cv::Mat GetRangeImageCv();
    cv::Mat GetDynamicImageCv();

    public:
    std::deque<RangeImage::Ptr> range_image_list; // back이 최신, front가 old
    std::vector<point_soph*> point_soph_pointers; // range image 내 point_soph의 벡터. 메모리 관리를 위해 고정적 크기로 관리 

    point_soph* temp_point_soph_pointer;
    int cur_point_soph_pointers = 0; // Range Image의 인덱스 
    int max_pointers_num = 0; // Range Image 개수와 같음 일단 

    RangeImage::Ptr temp_image_pointer;

    public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pmc_xyzrgb_pcptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr range_image_xyzrgb_pcptr_;

    double time_search = 0.0, time_search_0 = 0.0, time_research = 0.0, time_build = 0.0, time_other0 = 0.0, time_total = 0.0, time_total_avr = 0.0;
    int    pixel_fov_up, pixel_fov_down, pixel_fov_left, pixel_fov_right;
    int    occu_time_th = 3, is_occu_time_th = 3, map_index = 0;

    int max_pixel_points = 50; // 픽셀당 최대 포인트 수 

    bool is_key_frame;

    // configure;
    private:
    range_pmc_params params_;



};

#endif