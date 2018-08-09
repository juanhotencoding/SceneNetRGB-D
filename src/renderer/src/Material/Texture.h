/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include "Material.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgba.h>


class Texture : public Material
{
public:
    Texture(std::string& textureAbsoluteFilePath, unsigned int class_id, unsigned int instance_id);
    Texture(std::string& textureAbsoluteFilePath,
            std::string& normalMapAbsoluteFilePath, unsigned int class_id, unsigned int instance_id);
    virtual ~Texture();
    virtual optix::Material getOptixMaterial(optix::Context & context);
    virtual void registerGeometryInstanceValues(optix::GeometryInstance & instance);
    virtual std::string materialType() { return "Texture"; };

private:
    void loadDiffuseImage(std::string& textureAbsoluteFilePath );
    void loadNormalMapImage(std::string& normalsAbsoluteFilePath );
    optix::TextureSampler createTextureSamplerFromBuffer(optix::Context & context, optix::Buffer buffer);
    optix::Buffer createBufferFromImage(optix::Context & context,
                                        const CVD::Image<CVD::Rgba<CVD::byte> > & image);

    static bool m_optixMaterialIsCreated;
    static optix::Material m_optixMaterial;
    optix::TextureSampler m_diffuseSampler;
    optix::TextureSampler m_normalMapSampler;
    CVD::Image<CVD::Rgba<CVD::byte> > m_diffuseImage;
    CVD::Image<CVD::Rgba<CVD::byte> > m_normalMapImage;
};

