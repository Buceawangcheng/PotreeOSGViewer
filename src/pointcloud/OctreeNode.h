#pragma once

#include "pointcloud/BoundingBox.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

struct PointCloudNodeData;

// Potree hierarchy 记录描述的节点类型。
enum class OctreeNodeType {
    Normal, // hierarchy 已解析，并且还可能有子节点。
    Leaf,   // hierarchy 已解析，没有需要继续展开的子节点。
    Proxy   // 当前记录只给出另一个 hierarchy.bin 分块的位置，需要异步展开。
};

// hierarchy 信息的生命周期。它只描述“节点结构/字节范围是否已知”，不代表
// 点数据是否已解码，也不代表节点是否已提交给 GPU。
enum class HierarchyState {
    Resolved, // 当前节点的真实类型、点范围及已知子节点已经解析。
    Proxy,    // 尚未读取该节点指向的 hierarchy 分块。
    Queued,   // hierarchy 请求已经提交给 NodeLoadScheduler。
    Loading,  // 为显式加载阶段预留；当前 live Octree 通常由 scheduler 计数跟踪加载。
    Failed    // hierarchy 读取、解析或主线程应用失败。
};

// 点数据从磁盘到 CPU 内存的生命周期。与 HierarchyState、GpuState 相互独立。
enum class PointDataState {
    Unloaded, // 尚未提交 octree.bin 读取请求。
    Queued,   // 请求已经提交；live Octree 在工作线程执行期间通常仍保持此状态。
    Loading,  // Provider 内部加载阶段使用；工作线程不会直接修改 live Octree。
    CpuReady, // 完成结果已在主线程验证，data 中保存可挂载的 CPU 数组。
    Failed    // 读取、解码、结果校验或 OSG 挂载失败。
};

// 点数据从 CPU-ready 到 OSG 场景/GPU 资源的生命周期。
enum class GpuState {
    Detached, // 当前没有对应的 PotreeSceneBackend 节点。
    Resident  // 已创建 OSG Geometry，并加入 Potree layer 场景树。
};

// OctreeNode 是主线程拥有的运行时节点。
// 工作线程不得持有或修改它；异步边界使用 NodeLoadRequest/NodeLoadResult 值对象。
struct OctreeNode {
    // 稳定身份和空间信息。Potree id 形如 r、r0、r012。
    std::string id;
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    std::uint8_t childMask = 0;

    // 三条正交状态轴：hierarchy、CPU 点数据、OSG/GPU 驻留状态。
    OctreeNodeType type = OctreeNodeType::Leaf;
    HierarchyState hierarchyState = HierarchyState::Resolved;
    PointDataState pointDataState = PointDataState::Unloaded;
    GpuState gpuState = GpuState::Detached;

    // Proxy 节点从 hierarchy.bin 读取；普通/叶节点从 octree.bin 读取。
    std::uint64_t hierarchyByteOffset = 0;
    std::uint64_t hierarchyByteSize = 0;
    std::uint64_t pointByteOffset = 0;
    std::uint64_t pointByteSize = 0;

    // LOD、可见性和 LRU 淘汰使用的帧号。
    std::uint64_t lastSelectedFrame = 0;
    std::uint64_t lastVisibleFrame = 0;
    std::uint64_t lastAccessFrame = 0;

    // 每次重新提交同一节点时递增；晚到结果必须同时匹配 dataset generation
    // 和 request generation，才能应用到当前节点。
    std::uint64_t requestGeneration = 0;
    double selectionWeight = 0.0;
    double requestWeight = 0.0;
    std::uint64_t lastRequestedFrame = 0;

    // data 只在 CpuReady 且尚未挂载时保留；成功创建 OSG Geometry 后会释放。
    std::uint64_t cpuBytes = 0;
    std::uint64_t gpuBytes = 0;
    std::shared_ptr<PointCloudNodeData> data;

    // childMask 的 8 个八叉树槽位，未加载/不存在的槽位为空。
    std::array<std::unique_ptr<OctreeNode>, 8> children;
};
