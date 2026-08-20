#pragma once

#include "pointcloud/CameraState.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>
#include <vector>

struct LodSelectionSettings {
    std::uint64_t pointBudget = 1000000;
    double minimumNodePixelSize = 30.0;
    std::uint32_t maxLevel = 0;
};

struct NodeSelection {
    std::string nodeId;
    double weight = 0.0;
    std::uint64_t pointCount = 0;
    bool resident = false;
    bool cpuReady = false;
    bool hierarchyProxy = false;
};

struct NodeRequestCandidate {
    std::string nodeId;
    double weight = 0.0;
    bool hierarchyProxy = false;
};

struct SelectionResult {
    std::vector<NodeSelection> selectedNodes;
    std::vector<NodeRequestCandidate> loadCandidates;
    std::uint64_t selectedPointCount = 0;
    std::uint32_t highestSelectedLevel = 0;
    bool overBudget = false;
};

class LodSelector {
public:
    SelectionResult select(const OctreeNode& root,
                           const CameraState& camera,
                           const LodSelectionSettings& settings) const;

    static double projectedPixelRadius(const OctreeNode& node,
                                       const CameraState& camera);

private:
    static bool intersectsFrustum(const OctreeNode& node,
                                  const CameraState& camera);
};
