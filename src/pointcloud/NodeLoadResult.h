#pragma once

#include "pointcloud/HierarchyPatch.h"
#include "pointcloud/PointCloudNodeData.h"

#include <QString>

#include <cstdint>
#include <memory>
#include <string>

struct NodeLoadResult {
    std::uint64_t datasetGeneration = 0;
    std::string nodeId;
    std::uint64_t requestGeneration = 0;
    HierarchyPatch hierarchyPatch;
    std::shared_ptr<PointCloudNodeData> pointData;
    bool hierarchyOnly = false;
    QString error;
};
