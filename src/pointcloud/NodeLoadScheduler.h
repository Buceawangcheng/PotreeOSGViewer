#pragma once

#include "pointcloud/NodeLoadRequest.h"
#include "pointcloud/NodeLoadResult.h"
#include "pointcloud/PointCloudDataset.h"

#include <functional>
#include <memory>
#include <vector>

class NodeLoadScheduler {
public:
    NodeLoadScheduler();
    ~NodeLoadScheduler();

    void setDataset(std::shared_ptr<PointCloudDataset> dataset);
    void clear();
    void setMaxConcurrentLoads(std::size_t maxConcurrentLoads);
    void setCompletionCallback(std::function<void()> callback);

    void schedule(NodeLoadRequest request);
    std::vector<NodeLoadResult> drainCompleted();
    std::size_t loadingCount() const;
    std::size_t queuedCount() const;

private:
    struct State;

    static void dispatch(const std::shared_ptr<State>& state);
    static void runRequest(const std::shared_ptr<State>& state,
                           NodeLoadRequest request);

    std::shared_ptr<State> m_state;
};
