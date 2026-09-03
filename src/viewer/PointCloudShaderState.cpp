#include "viewer/PointCloudShaderState.h"

#include <osg/Shader>
#include <osg/Image>
#include <osg/GL>
#include <osg/StateAttribute>
#include <osg/StateSet>

#include <QFile>
#include <QImage>

namespace
{
osg::ref_ptr<osg::Shader> loadShader(osg::Shader::Type type,
                                     const QString& resourcePath,
                                     QString* errorMessage)
{
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open Shader resource: %1")
                                .arg(resourcePath);
        }
        return nullptr;
    }

    const QByteArray source = file.readAll();
    if (source.trimmed().isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Shader resource is empty: %1")
                                .arg(resourcePath);
        }
        return nullptr;
    }

    osg::ref_ptr<osg::Shader> shader = new osg::Shader(type, source.constData());
    shader->setName(resourcePath.toStdString());
    return shader;
}

osg::ref_ptr<osg::Texture1D> loadPalette(const QString& resourcePath,
                                         osg::Texture::FilterMode filter,
                                         QString* errorMessage)
{
    QImage image(resourcePath);
    if (image.isNull() || image.width() <= 0 || image.height() != 1) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to load one-row palette resource: %1")
                                .arg(resourcePath);
        }
        return nullptr;
    }

    image = image.convertToFormat(QImage::Format_RGBA8888);
    osg::ref_ptr<osg::Image> osgImage = new osg::Image;
    osgImage->allocateImage(image.width(), 1, 1, GL_RGBA, GL_UNSIGNED_BYTE);
    for (int x = 0; x < image.width(); ++x) {
        const QRgb pixel = image.pixel(x, 0);
        unsigned char* target = osgImage->data(x, 0);
        target[0] = static_cast<unsigned char>(qRed(pixel));
        target[1] = static_cast<unsigned char>(qGreen(pixel));
        target[2] = static_cast<unsigned char>(qBlue(pixel));
        target[3] = static_cast<unsigned char>(qAlpha(pixel));
    }
    osgImage->setName(resourcePath.toStdString());

    osg::ref_ptr<osg::Texture1D> texture = new osg::Texture1D;
    texture->setName(resourcePath.toStdString());
    texture->setImage(osgImage.get());
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture::MIN_FILTER, filter);
    texture->setFilter(osg::Texture::MAG_FILTER, filter);
    texture->setResizeNonPowerOfTwoHint(false);
    return texture;
}
} // namespace

bool PointCloudShaderState::initialize(QString* errorMessage)
{
    osg::ref_ptr<osg::Shader> vertexShader = loadShader(
        osg::Shader::VERTEX, QStringLiteral(":/shaders/pointcloud.vert"), errorMessage);
    if (!vertexShader.valid()) {
        return false;
    }

    osg::ref_ptr<osg::Shader> fragmentShader = loadShader(
        osg::Shader::FRAGMENT, QStringLiteral(":/shaders/pointcloud.frag"), errorMessage);
    if (!fragmentShader.valid()) {
        return false;
    }

    osg::ref_ptr<osg::Texture1D> turboPalette = loadPalette(
        QStringLiteral(":/colormaps/turbo.ppm"), osg::Texture::LINEAR, errorMessage);
    if (!turboPalette.valid()) {
        return false;
    }

    osg::ref_ptr<osg::Texture1D> lodPalette = loadPalette(
        QStringLiteral(":/colormaps/lod.ppm"), osg::Texture::NEAREST, errorMessage);
    if (!lodPalette.valid()) {
        return false;
    }

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->setName("Potree point cloud program");
    program->addShader(vertexShader.get());
    program->addShader(fragmentShader.get());
    m_program = program;
    m_turboPalette = turboPalette;
    m_lodPalette = lodPalette;
    m_colorModeUniform = new osg::Uniform("uColorMode", 0);
    m_pointSizeUniform = new osg::Uniform("uPointSize", 3.0f);
    m_heightMinUniform = new osg::Uniform("uHeightMin", 0.0f);
    m_heightMaxUniform = new osg::Uniform("uHeightMax", 1.0f);
    m_turboSamplerUniform = new osg::Uniform("uTurboPalette", 0);
    m_lodSamplerUniform = new osg::Uniform("uLodPalette", 1);
    return true;
}

bool PointCloudShaderState::isInitialized() const
{
    return m_program.valid();
}

void PointCloudShaderState::applyTo(osg::StateSet* stateSet,
                                    std::uint32_t level) const
{
    if (!stateSet || !m_program.valid()) {
        return;
    }

    stateSet->setAttributeAndModes(
        m_program.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setMode(GL_PROGRAM_POINT_SIZE,
                      osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setMode(GL_POINT_SPRITE,
                      osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    stateSet->setTextureAttributeAndModes(0, m_turboPalette.get(), osg::StateAttribute::ON);
    stateSet->setTextureAttributeAndModes(1, m_lodPalette.get(), osg::StateAttribute::ON);
    stateSet->addUniform(m_colorModeUniform.get());
    stateSet->addUniform(m_pointSizeUniform.get());
    stateSet->addUniform(m_heightMinUniform.get());
    stateSet->addUniform(m_heightMaxUniform.get());
    stateSet->addUniform(m_turboSamplerUniform.get());
    stateSet->addUniform(m_lodSamplerUniform.get());
    stateSet->addUniform(new osg::Uniform("uLodLevel", static_cast<int>(level)));
}

void PointCloudShaderState::setColorMode(PotreeColorMode mode)
{
    if (m_colorModeUniform.valid()) {
        m_colorModeUniform->set(static_cast<int>(mode));
    }
}

void PointCloudShaderState::setPointSize(float value)
{
    if (m_pointSizeUniform.valid()) {
        m_pointSizeUniform->set(value);
    }
}

void PointCloudShaderState::setHeightRange(float minimum, float maximum)
{
    if (m_heightMinUniform.valid() && m_heightMaxUniform.valid()) {
        m_heightMinUniform->set(minimum);
        m_heightMaxUniform->set(maximum);
    }
}

osg::Program* PointCloudShaderState::program() const
{
    return m_program.get();
}
