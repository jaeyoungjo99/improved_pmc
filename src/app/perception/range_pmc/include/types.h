#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>



typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Vector2f V2F;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>

/* Types */
struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float    intensity;
    uint16_t ring;
    float    time;      // point time after scan start
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRT,
                    ( float, x, x )
                    ( float, y, y )
                    ( float, z, z )
                    ( float, intensity, intensity )
                    ( uint16_t, ring, ring )
				    ( float, time, time )
)

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

struct EIGEN_ALIGN16 PointXYZIRGBRTLIS
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    PCL_ADD_RGB
    std::uint16_t ring;
    float time;
    std::uint16_t label;
    std::uint32_t index;
   std::uint16_t scan_id;
    PCL_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} ;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBRTLIS,
                                (float, x, x) (float, y, y) (float, z, z) 
                                (float, intensity, intensity)
                                (std::uint32_t, rgb, rgb)
                                (std::uint16_t, ring, ring)
                                (float, time, time)
                                (std::uint16_t, label, label)
                                (std::uint32_t, index, index)
                                (std::uint16_t, scan_id, scan_id))

/*
if(gt_label >= 0 && gt_label <= 1) // unlabeled
{
    continue;
}
else if(gt_label >= 9 && gt_label <= 99) // static
{
    pmc_save_pcptr->points[idx].motion_gt = 0;
    gt_is_dynamic = false;
}
else if(gt_label >= 251 && gt_label <= 259)
{
    pmc_save_pcptr->points[idx].motion_gt = 1;
    gt_is_dynamic = true;
}
else
{
    continue;
}
*/

#endif