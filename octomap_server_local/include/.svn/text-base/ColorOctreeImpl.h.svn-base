#ifndef COLOR_OCTREE_IMPL_H
#define COLOR_OCTREE_IMPL_H

#include "octo_globaldef.h"
#include "octomap/ColorOcTree.h"
#include "octomap/Pointcloud.h"
using namespace std;

class ColorOctreeImpl : public octomap::ColorOcTree
{
public:
    ColorOctreeImpl(float res=0.1);
    virtual ~ColorOctreeImpl();
    virtual void insertColorPointCloud(const octomap::Pointcloud&, const octomap::point3d&, vector<gl_color>&, double maxrange = -1, bool lazy_eval = false);
    virtual void insertColorPointCloud(const octomap::Pointcloud&, const octomap::point3d&, vector<gl_color>&, const octomap::pose6d&, double maxrange = -1, bool lazy_eval = false);
    virtual void insertPointCloud(const octomap::Pointcloud& , const octomap::point3d& , double maxrange=-1., bool lazy_eval = false);
    virtual void insertPointCloud(const octomap::Pointcloud& , const octomap::point3d& , const octomap::pose6d&, double maxrange = -1., bool lazy_eval = false);

    static void setOctreeColor(gl_color&);
    static gl_color s_octree_color;
    
    template<typename PointT>
    void insertPointCloud(pcl::PointCloud<PointT>&, double maxrange = -1, bool lazy_eval = false);
    template<typename PointT>
    void insertPointCloud(pcl::PointCloud<PointT>&, octomap::pose6d&, double maxrange = -1, bool lazy_eval = false);
    template<typename PointT>
    void insertPointCloud(pcl::PointCloud<PointT>&, float p[7], double maxrange = -1, bool lazy_eval = false);
};

template <typename PointT>
void ColorOctreeImpl::insertPointCloud(pcl::PointCloud<PointT>& pc, double maxrange , bool lazy_eval)
{
    octomap::Pointcloud oct_pc;
    octomap::point3d ori_pose;
    vector<gl_color> colors;
    fromColorPCL2OctoPC(pc, oct_pc, ori_pose, colors);
    insertColorPointCloud(oct_pc, ori_pose, colors, maxrange, lazy_eval);
}

template <typename PointT>
void ColorOctreeImpl::insertPointCloud(pcl::PointCloud<PointT>& pc, octomap::pose6d& frame_ori, double maxrange , bool lazy_eval)
{
    octomap::Pointcloud oct_pc;
    octomap::point3d ori_pose;
    vector<gl_color> colors;
    fromColorPCL2OctoPC(pc, oct_pc, ori_pose, colors);
    insertColorPointCloud(oct_pc, ori_pose, colors, frame_ori, maxrange, lazy_eval);
}

template<typename PointT>
void ColorOctreeImpl::insertPointCloud(pcl::PointCloud<PointT>& pc, float p[7], double maxrange , bool lazy_eval)
{
    octomath::Vector3 trans(p[0],p[1],p[2]); // tx, ty, tz
    octomath::Quaternion quaternion(p[3],p[4],p[5],p[6]);// u, x, y, z
    octomap::pose6d frame_ori(trans, quaternion);
    insertPointCloud(pc, frame_ori, maxrange, lazy_eval);
}


#endif
