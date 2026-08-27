#pragma once

#include "pointcloud/HierarchyPatch.h"
#include "pointcloud/PointCloudNodeData.h"

#include <QString>

#include <cstdint>
#include <memory>
#include <string>

// 工作线程返回主线程的完成值对象。
//
// 一个 Proxy 请求可能同时返回 hierarchyPatch 和 pointData；普通节点通常只返回
// pointData。工作线程只组装结果，live Octree 和 OSG 场景由主/update线程修改。
struct NodeLoadResult {
    // 原样回传请求身份，PointCloudRuntime 用它们过滤取消后或重提后的晚到结果。
    std::uint64_t datasetGeneration = 0;
    std::string nodeId;
    std::uint64_t requestGeneration = 0;

    // 非空 patch 先在主线程应用到 Octree；pointData 随后转为 CpuReady。
    HierarchyPatch hierarchyPatch;
    std::shared_ptr<PointCloudNodeData> pointData;
    bool hierarchyOnly = false;

    // 非空表示后台读取/解析/解码失败，主线程把对应状态标记为 Failed。
    QString error;
};
