#pragma once

#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4ub>

#include <cstdint>
#include <vector>

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
