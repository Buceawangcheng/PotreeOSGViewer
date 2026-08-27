#pragma once

#include "pointcloud/NodeLoadRequest.h"
#include "pointcloud/NodeLoadResult.h"
#include "pointcloud/PointCloudDataset.h"

#include <functional>
#include <memory>
#include <thread>
#include <vector>

// 节点后台加载调度器。
//
// 线程边界：
// - 主线程负责提交 NodeLoadRequest、取回 NodeLoadResult，并修改 Octree/OSG 状态；
// - 工作线程只执行文件 IO、hierarchy 解析和点数据 CPU 解码；
// - 请求队列、完成队列以及调度状态封装在共享 State 中，并由 State::mutex 保护。
//
// 调度器使用固定数量的常驻工作线程，避免为每个节点反复创建线程。析构时会
// 通知所有工作线程退出，并等待正在执行的任务结束。
class NodeLoadScheduler {
public:
    NodeLoadScheduler();
    ~NodeLoadScheduler();

    // 切换当前数据集，同时丢弃尚未开始的请求和尚未消费的完成结果。
    // 已经在工作线程中执行的旧请求无法被强制取消，调用方仍需通过 generation
    // 校验丢弃其晚到结果。
    void setDataset(std::shared_ptr<PointCloudDataset> dataset);
    void clear();

    // 限制同时执行 IO/解码的工作线程数量，最小值为 1。
    void setMaxConcurrentLoads(std::size_t maxConcurrentLoads);

    // 完成回调在工作线程中调用，只应用于唤醒/请求主线程更新；回调不能直接
    // 修改 Octree、OSG 或 OpenGL 对象。
    void setCompletionCallback(std::function<void()> callback);

    // 按 requestWeight 优先级把请求加入等待队列。
    void schedule(NodeLoadRequest request);

    // 主线程批量取走当前全部完成结果。使用 swap 缩短持锁时间。
    std::vector<NodeLoadResult> drainCompleted();

    // 以下计数是调用瞬间的线程安全快照。
    std::size_t loadingCount() const;
    std::size_t queuedCount() const;
    std::size_t outstandingCount() const;

private:
    // State 会被调度器和所有工作线程共享，其内部可变成员必须在加锁后访问。
    struct State;

    static void workerLoop(const std::shared_ptr<State>& state);

    std::shared_ptr<State> m_state;
    std::vector<std::thread> m_workers;
};
