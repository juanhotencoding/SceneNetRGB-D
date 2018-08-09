/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include "Material.h"
#include "Math/Vector3.h"
#include "Util/ptxpath.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgba.h>


class SpecularTexture : public Material
{
public:
    SpecularTexture(std::string& textureAbsoluteFilePath,const Vector3 & Ks,const float alpha,  unsigned int class_id, unsigned int instance_id);
    SpecularTexture(std::string& textureAbsoluteFilePath,
            std::string& normalMapAbsoluteFilePath,const Vector3 & Ks,const float alpha, unsigned int class_id, unsigned int instance_id);
    virtual ~SpecularTexture();
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
    Vector3 Ks;
    float alpha;
};

