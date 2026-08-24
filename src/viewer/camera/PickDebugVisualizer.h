#pragma once

#include "viewer/camera/DepthBufferPicker.h"

#include <osg/Array>
#include <osg/Referenced>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <osg/ref_ptr>

namespace osg {
class AutoTransform;
class Geometry;
class Node;
class Switch;
}

class PickDebugVisualizer : public osg::Referenced
{
public:
    PickDebugVisualizer();

    osg::Node* node();
    const osg::Node* node() const;

    void setVisible(bool visible);
    bool visible() const;

    void show(const osg::Vec3d& worldPoint, PickAction action);
    void clear();

    bool hasMarker() const;
    const osg::Vec3d& worldPoint() const;
    const osg::Vec4& color() const;

    static osg::Vec4 colorForAction(PickAction action);

protected:
    ~PickDebugVisualizer() override = default;

private:
    void updateSwitch();

    osg::ref_ptr<osg::Switch> m_root;
    osg::ref_ptr<osg::AutoTransform> m_transform;
    osg::ref_ptr<osg::Geometry> m_geometry;
    osg::ref_ptr<osg::Vec4Array> m_colors;
    bool m_visible = false;
    bool m_hasMarker = false;
    osg::Vec3d m_worldPoint;
    osg::Vec4 m_color;
};
