#include "pointcloud/NodeLoadScheduler.h"

#include "pointcloud/Potree2Provider.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

namespace
{
// 常驻线程总数。实际同时工作的线程数还会受到 State::maxConcurrentLoads 限制。
constexpr std::size_t PersistentWorkerCount = 4;
}

struct NodeLoadScheduler::State {
    struct PendingLess {
        bool operator()(const NodeLoadRequest& lhs, const NodeLoadRequest& rhs) const
        {
            // std::priority_queue 会把“更大”的元素放在队首，因此权重更高的
            // 可见节点优先加载；权重相同时，requestGeneration 较小者优先。
            if (lhs.requestWeight == rhs.requestWeight) {
                return lhs.requestGeneration > rhs.requestGeneration;
            }
            return lhs.requestWeight < rhs.requestWeight;
        }
    };

    // mutex 保护下面所有共享状态。锁只覆盖入队、出队和状态更新，不覆盖耗时的
    // 文件 IO、hierarchy 解析或点数据解码。
    mutable std::mutex mutex;

    // 工作线程在没有任务、没有数据集或并发额度已满时休眠，避免空转占用 CPU。
    std::condition_variable condition;
    std::shared_ptr<PointCloudDataset> dataset;
    std::priority_queue<NodeLoadRequest, std::vector<NodeLoadRequest>, PendingLess> pending;
    std::vector<NodeLoadResult> completed;
    std::size_t maxConcurrentLoads = 4;
    std::size_t loading = 0;
    bool shutdown = false;

    // 工作线程完成任务后复制回调，并在释放 mutex 后调用，避免回调重入调度器
    // 时发生死锁。
    std::function<void()> completionCallback;
};

NodeLoadScheduler::NodeLoadScheduler()
    : m_state(std::make_shared<State>())
{
    m_workers.reserve(PersistentWorkerCount);
    for (std::size_t index = 0; index < PersistentWorkerCount; ++index) {
        m_workers.emplace_back(&NodeLoadScheduler::workerLoop, m_state);
    }
}

NodeLoadScheduler::~NodeLoadScheduler()
{
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        m_state->shutdown = true;
        m_state->completionCallback = {};
        while (!m_state->pending.empty()) {
            m_state->pending.pop();
        }
        m_state->dataset.reset();
    }
    // 唤醒所有可能阻塞在 condition.wait() 的工作线程。正在执行 IO/解码的线程
    // 会先完成当前任务，然后在下一轮循环观察到 shutdown 并退出。
    m_state->condition.notify_all();
    for (std::thread& worker : m_workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void NodeLoadScheduler::setDataset(std::shared_ptr<PointCloudDataset> dataset)
{
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        m_state->dataset = std::move(dataset);
        // 只能取消尚未出队的请求。已经被工作线程取走的请求持有自己的 dataset
        // shared_ptr 快照，其结果最终由运行时的 generation 校验过滤。
        while (!m_state->pending.empty()) {
            m_state->pending.pop();
        }
        m_state->completed.clear();
    }
    m_state->condition.notify_all();
}

void NodeLoadScheduler::clear()
{
    setDataset(nullptr);
}

void NodeLoadScheduler::setMaxConcurrentLoads(std::size_t maxConcurrentLoads)
{
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        m_state->maxConcurrentLoads = std::max<std::size_t>(1, maxConcurrentLoads);
    }
    m_state->condition.notify_all();
}

void NodeLoadScheduler::setCompletionCallback(std::function<void()> callback)
{
    std::lock_guard<std::mutex> lock(m_state->mutex);
    m_state->completionCallback = std::move(callback);
}

void NodeLoadScheduler::schedule(NodeLoadRequest request)
{
    {
        std::lock_guard<std::mutex> lock(m_state->mutex);
        if (m_state->shutdown || !m_state->dataset) {
            return;
        }
        m_state->pending.push(std::move(request));
    }
    // 修改共享状态后再通知；此时工作线程醒来即可立即取得 mutex 并看到新请求。
    m_state->condition.notify_all();
}

std::vector<NodeLoadResult> NodeLoadScheduler::drainCompleted()
{
    std::vector<NodeLoadResult> completed;
    std::lock_guard<std::mutex> lock(m_state->mutex);
    // 常数时间交换容器，结果的析构和后续处理都在锁外完成。
    completed.swap(m_state->completed);
    return completed;
}

std::size_t NodeLoadScheduler::loadingCount() const
{
    std::lock_guard<std::mutex> lock(m_state->mutex);
    return m_state->loading;
}

std::size_t NodeLoadScheduler::queuedCount() const
{
    std::lock_guard<std::mutex> lock(m_state->mutex);
    return m_state->pending.size();
}

std::size_t NodeLoadScheduler::outstandingCount() const
{
    std::lock_guard<std::mutex> lock(m_state->mutex);
    return m_state->pending.size() + m_state->loading;
}

void NodeLoadScheduler::workerLoop(const std::shared_ptr<State>& state)
{
    for (;;) {
        NodeLoadRequest request;
        std::shared_ptr<PointCloudDataset> dataset;
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            // wait(lock, predicate) 会在休眠时释放 mutex，醒来后重新加锁并检查条件，
            // 因而可以正确处理虚假唤醒。
            state->condition.wait(lock, [&]() {
                return state->shutdown
                    || (state->dataset
                        && !state->pending.empty()
                        && state->loading < state->maxConcurrentLoads);
            });
            if (state->shutdown) {
                return;
            }

            request = state->pending.top();
            state->pending.pop();
            dataset = state->dataset;
            ++state->loading;
        }

        // 最耗时的 IO、解析和解码明确放在锁外；其他工作线程和主线程不会因此
        // 被阻塞。
        Potree2Provider provider;
        NodeLoadResult result = provider.ensureNodeReady(*dataset, request);

        std::function<void()> callback;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->completed.push_back(std::move(result));
            --state->loading;
            callback = state->completionCallback;
        }

        // 回调可能间接请求一次 viewer/update，必须在释放 mutex 后执行。
        if (callback) {
            callback();
        }
        state->condition.notify_all();
    }
}
