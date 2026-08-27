#pragma once

#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4ub>

#include <cstdint>
#include <vector>

// 工作线程解码得到的单节点 CPU 数据。
// positions 使用相对 origin 的 float 坐标以保持精度；主线程挂载成功后，数组会
// 被复制/交给 OSG Geometry，OctreeNode::data 随即释放以控制 CPU 峰值。
struct PointCloudNodeData {
    osg::Vec3d origin;
    std::vector<osg::Vec3f> positions;
    std::vector<osg::Vec4ub> colors;

    std::uint64_t cpuBytes() const
    {
        return static_cast<std::uint64_t>(positions.size()) * sizeof(osg::Vec3f)
            + static_cast<std::uint64_t>(colors.size()) * sizeof(osg::Vec4ub);
    }
};
