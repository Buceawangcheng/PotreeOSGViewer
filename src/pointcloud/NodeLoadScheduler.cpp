#include "pointcloud/NodeLoadScheduler.h"

#include "pointcloud/Potree2Provider.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

namespace
{
constexpr std::size_t PersistentWorkerCount = 4;
}

struct NodeLoadScheduler::State {
    struct PendingLess {
        bool operator()(const NodeLoadRequest& lhs, const NodeLoadRequest& rhs) const
        {
            if (lhs.requestWeight == rhs.requestWeight) {
                return lhs.requestGeneration > rhs.requestGeneration;
            }
            return lhs.requestWeight < rhs.requestWeight;
        }
    };

    mutable std::mutex mutex;
    std::condition_variable condition;
    std::shared_ptr<PointCloudDataset> dataset;
    std::priority_queue<NodeLoadRequest, std::vector<NodeLoadRequest>, PendingLess> pending;
    std::vector<NodeLoadResult> completed;
    std::size_t maxConcurrentLoads = 4;
    std::size_t loading = 0;
    bool shutdown = false;
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
    m_state->condition.notify_all();
}

std::vector<NodeLoadResult> NodeLoadScheduler::drainCompleted()
{
    std::vector<NodeLoadResult> completed;
    std::lock_guard<std::mutex> lock(m_state->mutex);
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

        Potree2Provider provider;
        NodeLoadResult result = provider.ensureNodeReady(*dataset, request);

        std::function<void()> callback;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->completed.push_back(std::move(result));
            --state->loading;
            callback = state->completionCallback;
        }

        if (callback) {
            callback();
        }
        state->condition.notify_all();
    }
}
