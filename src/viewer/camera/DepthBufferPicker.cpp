#include "viewer/camera/DepthBufferPicker.h"

#include "viewer/camera/CameraMath.h"

#include <osg/GL>
#include <osg/RenderInfo>
#include <osg/Viewport>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
constexpr GLenum FramebufferBindingParameter = 0x8CA6;
constexpr GLenum ReadFramebufferBindingParameter = 0x8CAA;
constexpr GLenum SamplesParameter = 0x80A9;

bool tokenIsNewerOrEqual(std::uint64_t generation,
                         std::uint64_t sequence,
                         std::uint64_t otherGeneration,
                         std::uint64_t otherSequence)
{
    return generation > otherGeneration
        || (generation == otherGeneration && sequence >= otherSequence);
}
} // namespace

void DepthBufferPicker::requestPick(const DepthPickRequest& request)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_pendingRequest
        || tokenIsNewerOrEqual(request.generation,
                               request.sequence,
                               m_pendingRequest->generation,
                               m_pendingRequest->sequence)) {
        m_pendingRequest = request;
    }
}

void DepthBufferPicker::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pendingRequest.reset();
    m_result.reset();
}

bool DepthBufferPicker::consumeResult(DepthPickResult& result)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_result) {
        return false;
    }

    result = *m_result;
    m_result.reset();
    return true;
}

bool DepthBufferPicker::computeReadRegion(int pixelX,
                                          int pixelY,
                                          const osg::Viewport& viewport,
                                          DepthReadRegion& region)
{
    if (!std::isfinite(viewport.x()) || !std::isfinite(viewport.y())
        || !std::isfinite(viewport.width())
        || !std::isfinite(viewport.height())
        || viewport.width() <= 0.0 || viewport.height() <= 0.0) {
        return false;
    }

    const int viewportMinX = static_cast<int>(std::ceil(viewport.x()));
    const int viewportMinY = static_cast<int>(std::ceil(viewport.y()));
    const int viewportMaxX = static_cast<int>(
        std::ceil(viewport.x() + viewport.width())) - 1;
    const int viewportMaxY = static_cast<int>(
        std::ceil(viewport.y() + viewport.height())) - 1;
    if (pixelX < viewportMinX || pixelX > viewportMaxX
        || pixelY < viewportMinY || pixelY > viewportMaxY) {
        return false;
    }

    constexpr int halfSize = PickSize / 2;
    const int readMinX = std::max(viewportMinX, pixelX - halfSize);
    const int readMinY = std::max(viewportMinY, pixelY - halfSize);
    const int readMaxX = std::min(viewportMaxX, pixelX + halfSize);
    const int readMaxY = std::min(viewportMaxY, pixelY + halfSize);

    region.x = readMinX;
    region.y = readMinY;
    region.width = readMaxX - readMinX + 1;
    region.height = readMaxY - readMinY + 1;
    return region.width > 0 && region.height > 0;
}

bool DepthBufferPicker::selectNearestValidDepth(
    const float* depths,
    std::size_t depthCount,
    const DepthReadRegion& region,
    int requestedPixelX,
    int requestedPixelY,
    DepthSampleSelection& selection)
{
    if (!depths || region.width <= 0 || region.height <= 0) {
        return false;
    }

    const std::size_t requiredCount = static_cast<std::size_t>(region.width)
        * static_cast<std::size_t>(region.height);
    if (depthCount < requiredCount) {
        return false;
    }

    bool found = false;
    int bestDistanceSquared = std::numeric_limits<int>::max();
    for (int localY = 0; localY < region.height; ++localY) {
        for (int localX = 0; localX < region.width; ++localX) {
            const std::size_t index = static_cast<std::size_t>(localY)
                    * static_cast<std::size_t>(region.width)
                + static_cast<std::size_t>(localX);
            const float depth = depths[index];
            if (!std::isfinite(depth) || depth <= 0.0f
                || depth >= 1.0f) {
                continue;
            }

            const int sampleX = region.x + localX;
            const int sampleY = region.y + localY;
            const int deltaX = sampleX - requestedPixelX;
            const int deltaY = sampleY - requestedPixelY;
            const int distanceSquared = deltaX * deltaX + deltaY * deltaY;
            if (!found || distanceSquared < bestDistanceSquared) {
                found = true;
                bestDistanceSquared = distanceSquared;
                selection.pixelX = sampleX;
                selection.pixelY = sampleY;
                selection.depth = depth;
            }
        }
    }

    return found;
}

bool DepthBufferPicker::isResultCurrent(const DepthPickResult& result,
                                        std::uint64_t generation,
                                        std::uint64_t latestSequence)
{
    return result.generation == generation
        && result.sequence == latestSequence;
}

void DepthBufferPicker::operator()(osg::RenderInfo& renderInfo) const
{
    DepthPickRequest request;
    if (!takePendingRequest(request)) {
        return;
    }

    DepthPickResult result;
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.wheelSteps = request.wheelSteps;
    result.requestedPixelX = request.pixelX;
    result.requestedPixelY = request.pixelY;

    osg::Camera* camera = renderInfo.getCurrentCamera();
    const osg::Viewport* viewport = camera ? camera->getViewport() : nullptr;
    if (!camera || !viewport) {
        result.outcome = DepthPickOutcome::MissingCameraOrViewport;
        publishResult(result);
        return;
    }

    result.viewportX = static_cast<int>(viewport->x());
    result.viewportY = static_cast<int>(viewport->y());
    result.viewportWidth = static_cast<int>(viewport->width());
    result.viewportHeight = static_cast<int>(viewport->height());
    DepthReadRegion region;
    if (!computeReadRegion(
            request.pixelX, request.pixelY, *viewport, region)) {
        result.outcome = DepthPickOutcome::InvalidReadRegion;
        publishResult(result);
        return;
    }
    result.readX = region.x;
    result.readY = region.y;
    result.readWidth = region.width;
    result.readHeight = region.height;

    glGetIntegerv(FramebufferBindingParameter, &result.framebufferBinding);
    glGetIntegerv(ReadFramebufferBindingParameter,
                  &result.readFramebufferBinding);
    glGetIntegerv(GL_DEPTH_BITS, &result.depthBits);
    glGetIntegerv(SamplesParameter, &result.samples);

    std::array<float, PickSize * PickSize> depths;
    depths.fill(1.0f);
    glReadPixels(region.x,
                 region.y,
                 region.width,
                 region.height,
                 GL_DEPTH_COMPONENT,
                 GL_FLOAT,
                 depths.data());
    result.glError = static_cast<unsigned int>(glGetError());

    const std::size_t readCount = static_cast<std::size_t>(region.width)
        * static_cast<std::size_t>(region.height);
    bool foundFiniteDepth = false;
    for (std::size_t index = 0; index < readCount; ++index) {
        const float depth = depths[index];
        if (!std::isfinite(depth)) {
            continue;
        }
        if (!foundFiniteDepth) {
            result.minimumReadDepth = depth;
            result.maximumReadDepth = depth;
            foundFiniteDepth = true;
        } else {
            result.minimumReadDepth = std::min(
                result.minimumReadDepth, depth);
            result.maximumReadDepth = std::max(
                result.maximumReadDepth, depth);
        }
        if (depth > 0.0f && depth < 1.0f) {
            ++result.validDepthSampleCount;
        }
    }

    if (result.glError != GL_NO_ERROR) {
        result.outcome = DepthPickOutcome::ReadPixelsError;
        publishResult(result);
        return;
    }

    DepthSampleSelection selection;
    if (!selectNearestValidDepth(depths.data(),
                                 depths.size(),
                                 region,
                                 request.pixelX,
                                 request.pixelY,
                                 selection)) {
        result.outcome = DepthPickOutcome::NoValidDepth;
        publishResult(result);
        return;
    }

    result.selectedPixelX = selection.pixelX;
    result.selectedPixelY = selection.pixelY;
    result.selectedDepth = selection.depth;
    if (!CameraMath::unprojectFramebufferPoint(selection.pixelX,
                                                selection.pixelY,
                                                selection.depth,
                                                *camera,
                                                result.worldPoint)) {
        result.outcome = DepthPickOutcome::UnprojectionFailed;
        publishResult(result);
        return;
    }

    result.hitScene = true;
    result.outcome = DepthPickOutcome::Hit;

    publishResult(result);
}

bool DepthBufferPicker::takePendingRequest(DepthPickRequest& request) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_pendingRequest) {
        return false;
    }
    request = *m_pendingRequest;
    m_pendingRequest.reset();
    return true;
}

void DepthBufferPicker::publishResult(const DepthPickResult& result) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_result
        || tokenIsNewerOrEqual(result.generation,
                               result.sequence,
                               m_result->generation,
                               m_result->sequence)) {
        m_result = result;
    }
}
