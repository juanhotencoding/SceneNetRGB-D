/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include "Texture.h"
#include "Renderer/RayType.h"

#include <cvd/image_io.h>


bool Texture::m_optixMaterialIsCreated = false;
optix::Material Texture::m_optixMaterial;

Texture::Texture(std::string& textureAbsoluteFilePath, unsigned int class_id, unsigned int instance_id)
    : Material(class_id,instance_id)
{
    loadDiffuseImage(textureAbsoluteFilePath);

//    CVD::img_save(m_diffuseImage,"diffuse_image.png");
}

Texture::Texture(std::string & textureAbsoluteFilePath,
                 std::string & normalMapAbosoluteFilePath,
                 unsigned int class_id,
                 unsigned int instance_id)
    : Material(class_id,instance_id)
{
    loadDiffuseImage(textureAbsoluteFilePath);
    loadNormalMapImage(normalMapAbosoluteFilePath);

//    CVD::img_save(m_diffuseImage,"diffuse_image.png");
//    CVD::img_save(m_normalMapImage,"normal_image.png");
}

Texture::~Texture()
{
//    delete m_diffuseImage;
//    delete m_normalMapImage;
}

void Texture::loadDiffuseImage(std::string& textureAbsoluteFilePath )
{
    std::cout<<"textureAbsolutePath = " << textureAbsoluteFilePath << std::endl;
    CVD::img_load(m_diffuseImage,textureAbsoluteFilePath);
    //std::cout<<"File has been loaded.. " << std::endl;

}

void Texture::loadNormalMapImage(std::string& normalMapAbsoluteFilePath )
{
        CVD::img_load(m_normalMapImage,normalMapAbsoluteFilePath);
     //   printf("Loaded normals: %s\n", normalMapAbsoluteFilePath.c_str());
}

optix::Material Texture::getOptixMaterial(optix::Context & context)
{
    if(!m_optixMaterialIsCreated)
    {
        m_optixMaterial = context->createMaterial();
        optix::Program radianceProgram = context->createProgramFromPTXFile(ptxpath() + "Texture.cu.ptx", "closestHitRadiance");
        optix::Program photonProgram = context->createProgramFromPTXFile(ptxpath() + "Texture.cu.ptx", "closestHitPhoton");

        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE_IN_PARTICIPATING_MEDIUM, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON, photonProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON_IN_PARTICIPATING_MEDIUM, photonProgram);
        m_optixMaterial->validate();
        this->registerMaterialWithShadowProgram(context, m_optixMaterial);
        this->registerMaterialWithGroundTruthGenerators(context, m_optixMaterial);
        m_optixMaterialIsCreated = true;
    }
    
    // Diffuse buffer

    optix::Buffer buffer = createBufferFromImage(context, m_diffuseImage);
    m_diffuseSampler = createTextureSamplerFromBuffer(context, buffer);

    optix::Buffer normalsBuffer;
    if(m_normalMapImage.size().x>0)
    {
        std::cout<<"It does have a normal buffer map" << std::endl;
        normalsBuffer = createBufferFromImage(context, m_normalMapImage);
    }
    else
    {
        std::cout<<"It does not have a normal buffer map" << std::endl;
        normalsBuffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 0, 0);
    }

    m_normalMapSampler = createTextureSamplerFromBuffer(context, normalsBuffer);
    return m_optixMaterial;
}

void Texture::registerGeometryInstanceValues(optix::GeometryInstance & instance )
{
    instance["diffuseSampler"]->setTextureSampler(m_diffuseSampler);
    instance["hasNormals"]->setUint(m_normalMapImage.size().x != 0);
    instance["normalMapSampler"]->setTextureSampler(m_normalMapSampler);
}

optix::TextureSampler Texture::createTextureSamplerFromBuffer( optix::Context & context, optix::Buffer buffer )
{
    std::cout<<"Making texture sampler from buffer"<<std::endl;
    optix::TextureSampler sampler = context->createTextureSampler();
    sampler->setWrapMode(0, RT_WRAP_REPEAT);
    sampler->setWrapMode(1, RT_WRAP_REPEAT);
    sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);
    sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    sampler->setMaxAnisotropy(4.f);
    sampler->setArraySize(1);
    sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    sampler->setMipLevelCount(1);
    sampler->setBuffer(0, 0, buffer);
    return sampler;
}

optix::Buffer Texture::createBufferFromImage(optix::Context & context,
                                             const CVD::Image<CVD::Rgba<CVD::byte> > & image )
{
    std::cout<<"Making buffer from cvd image"<<std::endl;
    optix::Buffer buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4,
                                                 image.size().x,
                                                 image.size().y);

    optix::char4* buffer_Host = (optix::char4*)buffer->map();
    memcpy(buffer_Host, image.data(), image.size().x *image.size().y*4*sizeof(unsigned char));
    buffer->unmap();
    return buffer;
}
