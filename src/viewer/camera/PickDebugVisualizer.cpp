#include "viewer/camera/PickDebugVisualizer.h"

#include "viewer/PotreeRenderMasks.h"

#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/StateSet>
#include <osg/Switch>

PickDebugVisualizer::PickDebugVisualizer()
    : m_root(new osg::Switch)
    , m_transform(new osg::AutoTransform)
    , m_geometry(new osg::Geometry)
    , m_colors(new osg::Vec4Array)
    , m_color(colorForAction(PickAction::Zoom))
{
    m_root->setName("CameraPickDebugRoot");
    m_root->setNodeMask(PotreeRenderMasks::PickDebug);
    m_root->setCullingActive(false);

    m_transform->setName("CameraPickDebugMarker");
    m_transform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
    m_transform->setAutoScaleToScreen(true);
    m_transform->setCullingActive(false);

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-8.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(8.0f, 0.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, -8.0f, 0.0f));
    vertices->push_back(osg::Vec3(0.0f, 8.0f, 0.0f));
    vertices->push_back(osg::Vec3(-5.0f, -5.0f, 0.0f));
    vertices->push_back(osg::Vec3(5.0f, 5.0f, 0.0f));
    vertices->push_back(osg::Vec3(-5.0f, 5.0f, 0.0f));
    vertices->push_back(osg::Vec3(5.0f, -5.0f, 0.0f));

    m_colors->push_back(m_color);
    m_geometry->setVertexArray(vertices.get());
    m_geometry->setColorArray(m_colors.get(), osg::Array::BIND_OVERALL);
    m_geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, vertices->size()));
    m_geometry->setUseDisplayList(false);
    m_geometry->setUseVertexBufferObjects(true);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setCullingActive(false);
    geode->addDrawable(m_geometry.get());
    m_transform->addChild(geode.get());
    m_root->addChild(m_transform.get(), false);

    osg::StateSet* stateSet = m_root->getOrCreateStateSet();
    stateSet->setMode(
        GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    stateSet->setMode(
        GL_DEPTH_TEST, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setAttributeAndModes(
        new osg::Depth(osg::Depth::ALWAYS, 0.0, 1.0, false),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setAttributeAndModes(
        new osg::LineWidth(2.0f),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setRenderBinDetails(10000, "RenderBin");
}

osg::Node* PickDebugVisualizer::node()
{
    return m_root.get();
}

const osg::Node* PickDebugVisualizer::node() const
{
    return m_root.get();
}

void PickDebugVisualizer::setVisible(bool visible)
{
    m_visible = visible;
    updateSwitch();
}

bool PickDebugVisualizer::visible() const
{
    return m_visible;
}

void PickDebugVisualizer::show(const osg::Vec3d& worldPoint,
                               PickAction action)
{
    m_worldPoint = worldPoint;
    m_color = colorForAction(action);
    m_colors->front() = m_color;
    m_colors->dirty();
    m_geometry->dirtyDisplayList();
    m_geometry->dirtyBound();
    m_transform->setPosition(worldPoint);
    m_hasMarker = true;
    updateSwitch();
}

void PickDebugVisualizer::clear()
{
    m_hasMarker = false;
    updateSwitch();
}

bool PickDebugVisualizer::hasMarker() const
{
    return m_hasMarker;
}

const osg::Vec3d& PickDebugVisualizer::worldPoint() const
{
    return m_worldPoint;
}

const osg::Vec4& PickDebugVisualizer::color() const
{
    return m_color;
}

osg::Vec4 PickDebugVisualizer::colorForAction(PickAction action)
{
    switch (action) {
    case PickAction::Zoom:
        return osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
    case PickAction::BeginPan:
        return osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f);
    case PickAction::BeginRotate:
        return osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f);
    }
    return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
}

void PickDebugVisualizer::updateSwitch()
{
    m_root->setValue(0, m_visible && m_hasMarker);
}
