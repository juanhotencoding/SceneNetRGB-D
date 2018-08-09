/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include "SpecularTexture.h"
#include "Renderer/RayType.h"

#include <cvd/image_io.h>


bool SpecularTexture::m_optixMaterialIsCreated = false;
optix::Material SpecularTexture::m_optixMaterial;

SpecularTexture::SpecularTexture(std::string& textureAbsoluteFilePath,const Vector3 & Ks,const float alpha,  unsigned int class_id, unsigned int instance_id)
    : Material(class_id,instance_id)
{
    loadDiffuseImage(textureAbsoluteFilePath);
    this->Ks = Ks;
    this->alpha = alpha;
}

SpecularTexture::SpecularTexture(std::string& textureAbsoluteFilePath,std::string& normalMapAbsoluteFilePath,const Vector3 & Ks,const float alpha,  unsigned int class_id, unsigned int instance_id)
    : Material(class_id,instance_id)
{
    loadDiffuseImage(textureAbsoluteFilePath);
    loadNormalMapImage(normalMapAbsoluteFilePath);
}

SpecularTexture::~SpecularTexture() { }

void SpecularTexture::loadDiffuseImage(std::string& textureAbsoluteFilePath )
{
    //std::cout<<"textureAbsolutePath = " << textureAbsoluteFilePath << std::endl;
    CVD::img_load(m_diffuseImage,textureAbsoluteFilePath);
    //std::cout<<"File has been loaded.. " << std::endl;

}

void SpecularTexture::loadNormalMapImage(std::string& normalMapAbsoluteFilePath )
{
        CVD::img_load(m_normalMapImage,normalMapAbsoluteFilePath);
        //printf("Loaded normals: %s\n", normalMapAbsoluteFilePath.c_str());
}

optix::Material SpecularTexture::getOptixMaterial(optix::Context & context)
{
//    std::cout<<"**********************************************" << std::endl;

    if(!m_optixMaterialIsCreated)
    {
        m_optixMaterial = context->createMaterial();
        optix::Program radianceProgram = context->createProgramFromPTXFile(ptxpath() + "SpecularTexture.cu.ptx", "closestHitRadiance");
        optix::Program photonProgram = context->createProgramFromPTXFile(ptxpath() + "SpecularTexture.cu.ptx", "closestHitPhoton");

        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE_IN_PARTICIPATING_MEDIUM, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON, photonProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON_IN_PARTICIPATING_MEDIUM, photonProgram);
        m_optixMaterial->validate();
        this->registerMaterialWithShadowProgram(context, m_optixMaterial);
        this->registerMaterialWithGroundTruthGenerators(context, m_optixMaterial);
        m_optixMaterialIsCreated = true;

//        std::cout<<"++++++++++++++++++Entered*******************" << std::endl;
    }
    
    // Diffuse buffer

    optix::Buffer buffer = createBufferFromImage(context, m_diffuseImage);
    m_diffuseSampler = createTextureSamplerFromBuffer(context, buffer);

    optix::Buffer normalsBuffer;
    if(m_normalMapImage.size().x>0)
    {
//        std::cout<<"It does have a normal buffer map" << std::endl;
        normalsBuffer = createBufferFromImage(context, m_normalMapImage);
    }
    else
    {
//        std::cout<<"It does not have a normal buffer map" << std::endl;
        normalsBuffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 0, 0);
    }

    m_normalMapSampler = createTextureSamplerFromBuffer(context, normalsBuffer);
    return m_optixMaterial;
}

void SpecularTexture::registerGeometryInstanceValues(optix::GeometryInstance & instance )
{
    instance["diffuseSampler"]->setTextureSampler(m_diffuseSampler);
    instance["hasNormals"]->setUint(m_normalMapImage.size().x != 0);
    instance["normalMapSampler"]->setTextureSampler(m_normalMapSampler);
    instance["Ks"]->setFloat(this->Ks);
    instance["alpha"]->setFloat(this->alpha);
}

optix::TextureSampler SpecularTexture::createTextureSamplerFromBuffer( optix::Context & context, optix::Buffer buffer )
{
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

optix::Buffer SpecularTexture::createBufferFromImage(optix::Context & context,
                                             const CVD::Image<CVD::Rgba<CVD::byte> > & image )
{
    optix::Buffer buffer = context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4,
                                                 image.size().x,
                                                 image.size().y);

    optix::char4* buffer_Host = (optix::char4*)buffer->map();
    memcpy(buffer_Host, image.data(), image.size().x *image.size().y*4*sizeof(unsigned char));
    buffer->unmap();
    return buffer;
}
