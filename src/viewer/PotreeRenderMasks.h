#pragma once

#include <osg/Node>

namespace PotreeRenderMasks
{
inline constexpr osg::Node::NodeMask Points = 1u << 0;
inline constexpr osg::Node::NodeMask BoundingBoxes = 1u << 1;
inline constexpr osg::Node::NodeMask PickDebug = 1u << 2;
inline constexpr osg::Node::NodeMask VisibleNode = Points | BoundingBoxes;
} // namespace PotreeRenderMasks
