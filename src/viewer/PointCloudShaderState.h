#pragma once

#include "viewer/PotreeColorMode.h"

#include <osg/Program>
#include <osg/Texture1D>
#include <osg/Uniform>
#include <osg/ref_ptr>

#include <QString>

#include <cstdint>

class PointCloudShaderState
{
public:
    bool initialize(QString* errorMessage = nullptr);
    bool isInitialized() const;
    void applyTo(osg::StateSet* stateSet, std::uint32_t level) const;
    void setColorMode(PotreeColorMode mode);
    void setPointSize(float value);
    void setHeightRange(float minimum, float maximum);

    osg::Program* program() const;

private:
    osg::ref_ptr<osg::Program> m_program;
    osg::ref_ptr<osg::Texture1D> m_turboPalette;
    osg::ref_ptr<osg::Texture1D> m_lodPalette;
    osg::ref_ptr<osg::Uniform> m_colorModeUniform;
    osg::ref_ptr<osg::Uniform> m_pointSizeUniform;
    osg::ref_ptr<osg::Uniform> m_heightMinUniform;
    osg::ref_ptr<osg::Uniform> m_heightMaxUniform;
    osg::ref_ptr<osg::Uniform> m_turboSamplerUniform;
    osg::ref_ptr<osg::Uniform> m_lodSamplerUniform;
};
