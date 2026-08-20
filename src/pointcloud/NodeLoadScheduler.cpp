#include "pointcloud/NodeLoadScheduler.h"

#include "pointcloud/Potree2Provider.h"

#include <algorithm>
#include <mutex>
#include <queue>
#include <thread>

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
}

NodeLoadScheduler::~NodeLoadScheduler()
{
    std::lock_guard<std::mutex> lock(m_state->mutex);
    m_state->shutdown = true;
    while (!m_state->pending.empty()) {
        m_state->pending.pop();
    }
    m_state->dataset.reset();
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
    dispatch(m_state);
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
    dispatch(m_state);
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
    dispatch(m_state);
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

void NodeLoadScheduler::dispatch(const std::shared_ptr<State>& state)
{
    std::vector<NodeLoadRequest> requests;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        while (!state->shutdown
               && state->dataset
               && state->loading < state->maxConcurrentLoads
               && !state->pending.empty()) {
            requests.push_back(state->pending.top());
            state->pending.pop();
            ++state->loading;
        }
    }

    for (NodeLoadRequest& request : requests) {
        std::thread(&NodeLoadScheduler::runRequest, state, std::move(request)).detach();
    }
}

void NodeLoadScheduler::runRequest(const std::shared_ptr<State>& state,
                                   NodeLoadRequest request)
{
    std::shared_ptr<PointCloudDataset> dataset;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        dataset = state->dataset;
    }

    NodeLoadResult result;
    if (dataset) {
        Potree2Provider provider;
        result = provider.ensureNodeReady(*dataset, request);
    } else {
        result.datasetGeneration = request.datasetGeneration;
        result.nodeId = request.nodeId;
        result.requestGeneration = request.requestGeneration;
        result.error = QStringLiteral("Point cloud dataset is no longer available.");
    }

    std::function<void()> callback;
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->completed.push_back(std::move(result));
        if (state->loading > 0) {
            --state->loading;
        }
        callback = state->completionCallback;
    }

    if (callback) {
        callback();
    }
    dispatch(state);
}
