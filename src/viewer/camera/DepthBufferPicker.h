#pragma once

#include <osg/Camera>
#include <osg/Vec3d>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

enum class PickAction
{
    Zoom,
    BeginPan,
    BeginRotate
};

enum class DepthPickOutcome
{
    Unknown,
    Hit,
    MissingCameraOrViewport,
    InvalidReadRegion,
    ReadPixelsError,
    NoValidDepth,
    UnprojectionFailed
};

struct DepthPickRequest
{
    std::uint64_t generation = 0;
    std::uint64_t sequence = 0;
    int pixelX = 0;
    int pixelY = 0;
    PickAction action = PickAction::Zoom;
    double wheelSteps = 0.0;
};

struct DepthPickResult
{
    std::uint64_t generation = 0;
    std::uint64_t sequence = 0;
    PickAction action = PickAction::Zoom;
    bool hitScene = false;
    osg::Vec3d worldPoint;
    double wheelSteps = 0.0;
    DepthPickOutcome outcome = DepthPickOutcome::Unknown;
    int requestedPixelX = 0;
    int requestedPixelY = 0;
    int viewportX = 0;
    int viewportY = 0;
    int viewportWidth = 0;
    int viewportHeight = 0;
    int readX = 0;
    int readY = 0;
    int readWidth = 0;
    int readHeight = 0;
    int validDepthSampleCount = 0;
    float minimumReadDepth = 1.0f;
    float maximumReadDepth = 1.0f;
    int selectedPixelX = 0;
    int selectedPixelY = 0;
    float selectedDepth = 1.0f;
    int framebufferBinding = -1;
    int readFramebufferBinding = -1;
    int depthBits = 0;
    int samples = 0;
    unsigned int glError = 0;
};

struct DepthReadRegion
{
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

struct DepthSampleSelection
{
    int pixelX = 0;
    int pixelY = 0;
    float depth = 1.0f;
};

class DepthBufferPicker : public osg::Camera::DrawCallback
{
public:
    static constexpr int PickSize = 5;

    void requestPick(const DepthPickRequest& request);
    void clear();
    bool consumeResult(DepthPickResult& result);

    static bool computeReadRegion(int pixelX,
                                  int pixelY,
                                  const osg::Viewport& viewport,
                                  DepthReadRegion& region);
    static bool selectNearestValidDepth(const float* depths,
                                        std::size_t depthCount,
                                        const DepthReadRegion& region,
                                        int requestedPixelX,
                                        int requestedPixelY,
                                        DepthSampleSelection& selection);
    static bool isResultCurrent(const DepthPickResult& result,
                                std::uint64_t generation,
                                std::uint64_t latestSequence);

    void operator()(osg::RenderInfo& renderInfo) const override;

protected:
    ~DepthBufferPicker() override = default;
    bool takePendingRequest(DepthPickRequest& request) const;
    void publishResult(const DepthPickResult& result) const;

private:
    mutable std::mutex m_mutex;
    mutable std::optional<DepthPickRequest> m_pendingRequest;
    mutable std::optional<DepthPickResult> m_result;
};
