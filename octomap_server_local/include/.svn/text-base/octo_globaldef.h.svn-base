#ifndef OCO_GLOBALDEF_H
#define OCO_GLOBALDEF_H
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "octomap/octomap_types.h"
#include "octomap/Pointcloud.h"
#include "colortable.h"
#include "Eigen/Core"
#include <vector>
#include <string>
using namespace std;

namespace octomap
{
    class Pointcloud;
}
namespace octowrapper
{
//typedef pcl::PointXYZRGB point_type;
typedef pcl::PointXYZ point_type;
// typedef pcl::PointXYZRGBA point_type;
typedef pcl::PointCloud<point_type> point_cloud;
typedef pcl::PointCloud<point_type>::Ptr point_cloud_ptr;
typedef pcl::PointCloud<point_type>::ConstPtr point_cloud_cptr;

typedef pcl::PointXYZRGBA color_point_type;
typedef pcl::PointCloud<color_point_type> color_point_cloud;
typedef pcl::PointCloud<color_point_type>::Ptr color_pc_ptr;
typedef pcl::PointCloud<color_point_type>::ConstPtr color_pc_cptr;
}

extern void fromPCL2OctoPC(octowrapper::point_cloud& , octomap::Pointcloud&, octomap::point3d& );
extern void fromEigen2Pose6d(Eigen::Matrix4f& , octomap::pose6d&);
extern void fromPose6d2Eigen(Eigen::Matrix4f& , octomap::pose6d&);
extern void fromRot2RPY(double&, double&, double&, Eigen::Matrix3f&);

template<typename PointT>
void fromColorPCL2OctoPC(pcl::PointCloud<PointT>& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose, vector<gl_color>& color)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        PointT& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
        color.push_back(gl_color(pt.r,pt.g,pt.b));
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

#endif
