#ifndef __GAUSSIAN_POINT_MOTION_CLASSIFICATION__
#define __GAUSSIAN_POINT_MOTION_CLASSIFICATION__
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
#include <Eigen/Core>


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
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

// Algorithm header
#include <types.h>
#include <cluster_predict/voxel_cluster.h>
// #include <parallel_q.h>

// 

#include <Eigen/LU>



using namespace std;
using namespace Eigen;
using namespace cv;

// 외부 파일에서 접근 할 수 없도록 static
static int m_i_max_2d_n;  // m_i_max_1d * m_i_max_1d_half . 구체형 전체를 resolution으로 나눈 것
static int m_i_max_1d; // 2pi / horizontal resolution
static int m_i_max_1d_half; // pi / vertical resolution
static int m_i_max_point; // max point num in pointcloud

static int    m_i_pixel_fov_up, m_i_pixel_fov_down, m_i_pixel_fov_left, m_i_pixel_fov_right;
static int    m_i_row_size, m_i_col_size;

typedef unsigned char EvidType;
typedef unsigned short EvidSqaureType;
static const EvidType MAX_EVIDENCE = 255;

struct gs_pmc_params{
    float   f_horizontal_resolution;
    float   f_vertical_resolution;
    float   f_min_range;
    float   f_max_range;
    int     i_max_points_in_pixel;
    float   f_range_threshold;
    int     i_max_range_image_num;
    float   f_min_key_frame_time;
    float   f_min_key_frame_rot;
    float   f_fov_up;
    float   f_fov_down;
    float   f_fov_left;
    float   f_fov_right;
    int     i_min_occluded_num;
    int     i_vertical_neighbor_pixel;
    int     i_horizontal_neighbor_pixel;

    float   f_ground_angle;

    float   f_dist_threshold_m;
    int     i_segment_min_point_num;
    int     i_segment_valid_point_num;
    int     i_segment_valid_line_num;

    float   f_moving_confidence;
    float   f_static_confidence;
    float   f_gaussian_sigma;
    float   f_static_gaussian_sigma;
    float   f_dynamic_gaussian_sigma;
    float   f_sigma_epsilon;

    bool    b_ground_filtering;
    bool    b_cluster_level_filtering;

    bool    b_run_case_2;
    bool    b_run_region_growing;
    bool    b_run_small_dynamic_filtering;
    int     i_min_voxel_cluster_num;
    

    bool    b_output_static_point;
    bool    b_output_discrete_label;
    bool    b_output_min_range;
    bool    b_debug_image;

    std::vector<float> vec_f_ego_to_lidar;
    std::vector<float> vec_f_ego_to_lidar_rotation;
};


struct by_size_decent
{
    bool operator()(std::vector<int> const &left, std::vector<int> const &right)
    {
        return left.size() > right.size();
    }
};


enum dyn_obj_flg {UNKNOWN, STATIC, CASE1, CASE2, CASE3, OCCLUDED, INVALID};

// Point struct in range image
struct point_soph
{
    int         ind;
    int         hor_ind;
    V3F         vec; // horizontal angle, vertical angle, distance
    int         ver_ind;
    int         cluster_ind; // 기본 -1. 검색되면 -2, 할당되면 0 이상 
    bool        ground;
    bool        propagated;
    int         position; // Point index in range image
    double      time;
    dyn_obj_flg  dyn;

    float       incident; // Incident angle. 0->normal is parrell to beam, pi/2 --> normal is perpendicular to beam

    // TEST
    bool        occluded_by_previous;
    bool        valid;
    bool        key_point; 
    Vector3i     occu_index; // 가려지게한 맵 인덱스, 해당 맵에서의 position

    bool        fp_dynamic;

    // M3D         sensor_rot;        // 이 포인트 집합의 대표 sensor_rot TODO: 이 데이터들 중복되니 따로. 72 바이트
    V3D         sensor_transl;     // 이 포인트 집합의 대표 sensor_transl TODO: 이 데이터들 중복되니 따로. 24 바이트
    V3D         glob;       // 이 포인트의 global 좌표상의 위치
    V3D         local;      // 이 포인트의 라이다 좌표상의 위치

    uint8_t     r; // 0 ~ 255, 1 바이트
    uint8_t     g; // 0 ~ 255, 1 바이트
    uint8_t     b; // 0 ~ 255, 1 바이트

    float       intensity;

    typedef boost::shared_ptr<point_soph> Ptr;
    point_soph(V3D & point, float & hor_resolution_max, float & ver_resolution_max) // Constructor with resolution and point xyz
    {
        vec(2)     = float(point.norm()); // range
        vec(0)     = atan2f(float(point(1)), float(point(0))); // horizontal angle
        vec(1)     = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2))); // vertical angle
        hor_ind    = floor((vec(0) + M_PI) / hor_resolution_max) - m_i_pixel_fov_right; // 
        ver_ind    = floor((vec(1) + 0.5 * M_PI) / ver_resolution_max) - m_i_pixel_fov_down;
        position   = hor_ind * m_i_max_1d_half + ver_ind;
        cluster_ind = -1;
        ground      = false;
        propagated  = false;
        occluded_by_previous = false;
        valid = false;
        key_point = false;
        fp_dynamic = false;
        incident   = -1.0;
        time       = -1;
        occu_index = -1*Vector3i::Ones();
        sensor_transl.setZero();
        glob.setZero();
        // sensor_rot.setOnes();
        local.setZero();
        r = 0;
        g = 0;
        b = 255;
    };
    point_soph() // Default Constructor
    {
        vec.setZero();
        ind = hor_ind  = ver_ind = position = 0;
        cluster_ind = -1;
        ground      = false;
        propagated  = false;
        occluded_by_previous = false;
        valid = false;
        key_point = false;
        fp_dynamic = false;
        incident   = -1.0;
        time       = -1;
        occu_index = -1*Vector3i::Ones();
        sensor_transl.setZero();
        glob.setZero();
        // sensor_rot.setOnes();
        local.setZero();
        r = 0;
        g = 0;
        b = 255;
    };
    point_soph(V3F s, int ind1, int ind2, int pos) // Constructor with s(hor_angle, ver_angle, range) image_index, range_index
    {
        vec = s;
        hor_ind = ind1;
        ver_ind = ind2;
        cluster_ind = -1;
        ground     = false;
        propagated = false;
        occluded_by_previous = false;
        valid = false;
        key_point = false;
        fp_dynamic = false;
        incident   = -1.0;
        position = pos;
        time = -1;
        occu_index = -1*Vector3i::Ones();
        sensor_transl.setZero();
        glob.setZero();
        // sensor_rot.setOnes();
        local.setZero();
        r = 0;
        g = 0;
        b = 255;
    };
    point_soph(const point_soph & cur) // Copy
    {   
        vec = cur.vec;
        ind = cur.ind;
        hor_ind  = cur.hor_ind;
        ver_ind  = cur.ver_ind;
        cluster_ind = cur.cluster_ind;
        ground      = cur.ground;
        propagated  = cur.propagated;
        occluded_by_previous = cur.occluded_by_previous;
        valid = cur.valid;
        key_point = cur.key_point;
        fp_dynamic = cur.fp_dynamic;
        occu_index = cur.occu_index;
        incident    = cur.incident;
        position  = cur.position;
        time = cur.time;
        sensor_transl = cur.sensor_transl;
        glob = cur.glob;
        // sensor_rot = cur.sensor_rot;
        dyn = cur.dyn;
        local = cur.local;
        r = cur.r;
        g = cur.g;
        b = cur.b;
    };

    ~point_soph(){
    };

    void GetVec(V3D & point, float & hor_resolution_max, float & ver_resolution_max) // Function with resolution and point xyz
    {
        vec(2)    = float(point.norm()); // range
        vec(0)    = atan2f(float(point(1)), float(point(0))); // hor
        vec(1)    = atan2f(float(point(2)), sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2))); // ver
        hor_ind   = floor((vec(0) + M_PI) / hor_resolution_max) - m_i_pixel_fov_right;
        ver_ind   = floor((vec(1) + 0.5 * M_PI) / ver_resolution_max) - m_i_pixel_fov_down;
        position  = hor_ind * m_i_max_1d_half + ver_ind;
    };

    void reset()
    {
        occu_index = -1*Vector3i::Ones();
    };

};

// Range image class

typedef std::vector<std::vector<point_soph*>>  RangeImage2D; // image --> pixel --> points in pixel
//           position     points_in_pixel   pixel_pointer
class RangeImage
{
public:
    RangeImage2D        range_image;
    double              time;       // time of range image
    int                 map_index; // Index of current range image
    M3D                 project_R; // Rotation of range image
    V3D                 project_T; // Transform of range image
    // std::vector<point_soph::Ptr>    point_sopth_pointer; // To manage points in range image
    float*           min_range_static = nullptr;
    float*           min_range_all = nullptr;
    float*           max_range_all = nullptr;
    float*           max_range_static = nullptr;
    int*             max_range_index_all = nullptr;
    int*             min_range_index_all = nullptr;
    bool*            ground_all = nullptr;
    float*           incident_all = nullptr;
    int*             cluster_idx_all = nullptr;

    std::vector<std::vector<int>> cluster_poses_vec; // cluster_ind --> pixel_pos

    std::vector<int> index_vector; // Index of point in rangeimage

    typedef boost::shared_ptr<RangeImage> Ptr;

    RangeImage()
    {
        range_image.assign(m_i_max_2d_n, std::vector<point_soph*>()); // max_point * point_array vector
        
        time = 0.;
        project_R.setIdentity(3,3);
        project_T.setZero(3, 1);

        min_range_static = new float[m_i_max_2d_n];
        min_range_all = new float[m_i_max_2d_n];
        max_range_all = new float[m_i_max_2d_n];
        max_range_static = new float[m_i_max_2d_n];
        fill_n(min_range_static, m_i_max_2d_n, 0.0);
        fill_n(min_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_static, m_i_max_2d_n, 0.0);
        max_range_index_all = new int[m_i_max_2d_n];
        min_range_index_all = new int[m_i_max_2d_n];
        ground_all = new bool[m_i_max_2d_n];
        incident_all = new float[m_i_max_2d_n];
        cluster_idx_all = new int[m_i_max_2d_n];
        fill_n(min_range_index_all, m_i_max_2d_n, -1);
        fill_n(max_range_index_all, m_i_max_2d_n, -1);
        fill_n(ground_all, m_i_max_2d_n, false);
        fill_n(incident_all, m_i_max_2d_n, 0.0);
        fill_n(cluster_idx_all, m_i_max_2d_n, -1);

        map_index = -1;
        index_vector.assign(m_i_max_2d_n, 0);
        for (int i = 0; i < m_i_max_2d_n; i++) { 
            index_vector[i] = i;
        }

        cluster_poses_vec.clear();
        
    }

    RangeImage(M3D rot, V3D transl, double cur_time, int frame)
    {   
        range_image.assign(m_i_max_2d_n, std::vector<point_soph*>());       
        time = cur_time;
        project_R = rot;
        project_T = transl;
        min_range_static = new float[m_i_max_2d_n];
        min_range_all = new float[m_i_max_2d_n];
        max_range_all = new float[m_i_max_2d_n];
        max_range_static = new float[m_i_max_2d_n];
        fill_n(min_range_static, m_i_max_2d_n, 0.0);
        fill_n(min_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_static, m_i_max_2d_n, 0.0);
        max_range_index_all = new int[m_i_max_2d_n];
        min_range_index_all = new int[m_i_max_2d_n];
        ground_all = new bool[m_i_max_2d_n];
        incident_all = new float[m_i_max_2d_n];
        cluster_idx_all = new int[m_i_max_2d_n];
        fill_n(min_range_index_all, m_i_max_2d_n, -1);
        fill_n(max_range_index_all, m_i_max_2d_n, -1);
        fill_n(ground_all, m_i_max_2d_n, false);
        fill_n(incident_all, m_i_max_2d_n, 0.0);
        fill_n(cluster_idx_all, m_i_max_2d_n, -1);

        map_index = frame;
        index_vector.assign(m_i_max_2d_n, 0);
        for (int i = 0; i < m_i_max_2d_n; i++) {
            index_vector[i] = i;
        }

        cluster_poses_vec.clear();
    }

    RangeImage(const RangeImage & cur)
    {   
        range_image = cur.range_image;       
        time = cur.time;
        project_R = cur.project_R;
        project_T = cur.project_T;
        // point_sopth_pointer = cur.point_sopth_pointer;
        min_range_static = new float[m_i_max_2d_n];
        min_range_all = new float[m_i_max_2d_n];
        max_range_all = new float[m_i_max_2d_n];
        max_range_static = new float[m_i_max_2d_n];   
        fill_n(min_range_static, m_i_max_2d_n, 0.0);
        fill_n(min_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_static, m_i_max_2d_n, 0.0);
        max_range_index_all = new int[m_i_max_2d_n];       
        min_range_index_all = new int[m_i_max_2d_n];
        ground_all = new bool[m_i_max_2d_n];
        incident_all = new float[m_i_max_2d_n];
        cluster_idx_all = new int[m_i_max_2d_n];
        map_index = cur.map_index;      

        for(int i = 0; i < m_i_max_2d_n; i++)
        {
            min_range_static[i] = cur.min_range_static[i];
            max_range_static[i] = cur.max_range_static[i];
            min_range_all[i] = cur.min_range_all[i];
            max_range_all[i] = cur.max_range_all[i];
            max_range_index_all[i] = cur.max_range_index_all[i];
            min_range_index_all[i] = cur.min_range_index_all[i];
            ground_all[i] = cur.ground_all[i];
            incident_all[i] = cur.incident_all[i];
            cluster_idx_all[i] = cur.cluster_idx_all[i];
        }
        index_vector.assign(m_i_max_2d_n, 0);
        for (int i = 0; i < m_i_max_2d_n; i++) {
            index_vector[i] = i;
        }

        cluster_poses_vec.clear();
    }

    ~RangeImage()
    {
        if(min_range_static != nullptr) delete [] min_range_static;
        if(min_range_all  != nullptr) delete [] min_range_all;
        if(max_range_all  != nullptr) delete [] max_range_all;
        if(max_range_static  != nullptr) delete [] max_range_static;
        if(max_range_index_all  != nullptr) delete [] max_range_index_all;
        if(min_range_index_all  != nullptr) delete [] min_range_index_all;
        if(ground_all  != nullptr) delete [] ground_all;
        if(incident_all  != nullptr) delete [] incident_all;
        if(cluster_idx_all  != nullptr) delete [] cluster_idx_all;
    }

    void Reset(M3D rot, V3D transl, double cur_time, int frame)
    {   
        time = cur_time;
        project_R = rot;
        project_T = transl;
        map_index = frame;

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, index_vector.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                for(size_t i=r.begin(); i!=r.end(); ++i) {
                    range_image[index_vector[i]].clear();
                }
            }
        );

        fill_n(min_range_static, m_i_max_2d_n, 0.0);
        fill_n(min_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_all, m_i_max_2d_n, 0.0);
        fill_n(max_range_static, m_i_max_2d_n, 0.0);
        fill_n(max_range_index_all, m_i_max_2d_n, -1);
        fill_n(min_range_index_all, m_i_max_2d_n, -1);
        fill_n(ground_all, m_i_max_2d_n, false);
        fill_n(incident_all, m_i_max_2d_n, 0.0);
        fill_n(cluster_idx_all, m_i_max_2d_n, -1);

        cluster_poses_vec.clear();
    }

};

class GaussianPointMotionClassi{
    public:
    GaussianPointMotionClassi();
    ~GaussianPointMotionClassi();

    void Init(gs_pmc_params params);
    void Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time);

    private:

    // Test
    void Case1(point_soph & p, RangeImage::Ptr range_image_ptr);
    bool Case1Enter(const point_soph & p, const RangeImage &image_info, dyn_obj_flg & p_dyn);
    bool Case1Gaussian(const point_soph & p, const RangeImage &image_info, dyn_obj_flg & p_dyn, uint8_t & p_r, uint8_t & p_g, uint8_t & p_b);

    void Case2(point_soph & p, RangeImage::Ptr range_image_ptr);

    // Range Image Generation
    
    // 시간, rot, transl를 고려하여 range image 생성
    void GenerateRangeImage(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl, RangeImage::Ptr range_image_ptr); 
    void GenerateRangeImageParallel(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl, RangeImage::Ptr range_image_ptr); 

    void GroundSegmentation(RangeImage::Ptr range_image_ptr);
    void ObjectSegmentation(RangeImage::Ptr range_image_ptr);

    // Utils
    void SphericalProjection(point_soph &p, int range_index, const M3D &rot, const V3D &transl, point_soph &p_spherical);
    bool KeyFrameCheck(double cur_time, M3D rot, V3D transl);
    inline float GaussianWeight(double value, double sigma);
    inline float GaussianWeight2(double value, double sigma);

    void DempsterCombination(EvidType* EvidSrc, EvidType* EvidOther);

    void CheckNeighbor(const point_soph & p, const RangeImage &image_info, float &max_range, float &min_range, float &occluded_max_range);

    // Clustering Functions
    void NeighborAssign(unsigned int hor_neighbor, unsigned int ver_neighbor);
    void LabelComponents(RangeImage::Ptr range_image_ptr, uint16_t row, uint16_t col);
    void ResetClustering();

    // Dynamic point filtering
    void SmallDynamicFiltering(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr);

    void StaticRegionGrowing(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr);
    void ImageRegionGrowing(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr);
    void RegionGrowingWithCluster(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr);

    bool ClusterDemster(RangeImage::Ptr range_image_ptr, point_soph* point,
                                         int iter_ver_search_index, int iter_hor_search_index);
    bool ClusterDemsterWithPropagation(RangeImage::Ptr range_image_ptr, point_soph* point,
                                         int iter_ver_search_index, int iter_hor_search_index);

    

    // Output
    public:
    void OutputPmcPC(const std::vector<point_soph*> &points, const RangeImage::Ptr range_image_ptr);
    void GetFilteredPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_pmc_xyzrgb_pcptr);
    void GetKeyFramePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_key_frame_xyzrgb_pcptr);
    void GetClusterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_cluster_xyzrgb_pcptr);
    
    cv::Mat GetRangeImageCv();
    cv::Mat GetDynamicImageCv();
    cv::Mat GetIncidentImageCv();
    cv::Mat GetClusterImageCv();
    cv::Mat GetGroundImageCv();


    public:
    std::deque<RangeImage::Ptr> m_range_image_ptr_list; // back이 최신, front가 old
    RangeImage::Ptr m_temp_range_image_ptr;

    point_soph* m_temp_point_soph_pointer;
    std::vector<point_soph*> m_point_soph_pointers; // range image 내 point_soph의 벡터. 메모리 관리를 위해 고정적 크기로 관리
    // 이렇게 레인지 이미지로 여러개로 빼 놓은 것은, m_detector에선 현재 point의 동적 여부 체크와, 과거 이미지의 range image 생성을 동시에 하기 때문에
    // 메모리를 미리 전역변수로 할당해 놓고, 사용 하는 것이다. 


    int m_i_cur_point_soph_pointers = 0; // Range Image의 인덱스 

    public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pmc_xyzrgb_pcptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_range_image_xyzrgb_pcptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cluster_xyzrgb_pcptr;
    std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_deque_range_image_xyzrgb_pcptr;

    
    int    m_i_map_index = 0;
    bool    m_b_key_frame = false;
    
    // Calcuation time
    double time_total = 0.0, time_cluster_labeling = 0.0, time_demster = 0.0, time_gaussian = 0.0;
    double time_case_1 = 0.0, time_case_2 = 0.0, time_case_3 = 0.0;
    int i_small_incident = 0;
    int i_total_incident = 0;
    int i_ground_skip = 0;
    int i_prop_num = 0;
    int i_gaussian = 0;
    int i_dempster = 0;
    int i_valid_position_num = 0;
    
    int i_case_2_used = 0;
    int i_case_2_tested = 0;
    int i_case_2 = 0;

    int i_propgation_used = 0;
    int i_propagation_propagated = 0;

    private:
    // Object segmentation

    std::vector<std::pair<int8_t, int8_t> > m_v_neighbor_iter;

    std::vector<uint16_t> m_v_ui16_queue_idx_x; // array for breadth-first search process of segmentation, for speed
    std::vector<uint16_t> m_v_ui16_queue_idx_y;

    int m_i_cluster_idx = 0;

    private: 

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr m_tbb_dynamic_points_pcptr;

    pcl::PointCloud<PointType>::Ptr m_tbb_points_normal_pcptr;
    pcl::PointCloud<PointType>::Ptr m_tbb_dynamic_points_normal_pcptr;

    // configure
    private:
    gs_pmc_params m_params;



};

#endif