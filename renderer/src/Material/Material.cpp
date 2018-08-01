/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include "Material.h"
#include "Renderer/RayType.h"

bool Material::m_hasLoadedOptixAnyHitProgram = false;
optix::Program Material::m_optixAnyHitProgram;

bool Material::m_hasLoadedGroundTruthProgram = false;
optix::Program Material::m_optixGroundTruthHitProgram;

Material::~Material()
{

}

void Material::registerBaseGeometryInstanceValues( optix::GeometryInstance & instance ) {
  //instance["ClassID"]->setUint(this->class_id_);
  instance["InstanceID"]->setUint(this->instance_id_);
  this->registerGeometryInstanceValues(instance);
}

void Material::registerMaterialWithGroundTruthGenerators(optix::Context & context, optix::Material & material)
{
    if(!m_hasLoadedGroundTruthProgram)
    {
        m_optixGroundTruthHitProgram = context->createProgramFromPTXFile(ptxpath() +  "GroundTruthMaterial.cu.ptx", "closestHitProgram");
        m_hasLoadedGroundTruthProgram = true;
    }
    material->setClosestHitProgram(RayType::GROUND_TRUTH, m_optixGroundTruthHitProgram);
}

void Material::registerMaterialWithShadowProgram( optix::Context & context, optix::Material & material )
{
    if(!m_hasLoadedOptixAnyHitProgram)
    {
        m_optixAnyHitProgram = context->createProgramFromPTXFile(ptxpath() +  "DirectRadianceEstimation.cu.ptx", "gatherAnyHitOnNonEmitter");
        m_hasLoadedOptixAnyHitProgram = true;
    }
    material->setAnyHitProgram(RayType::SHADOW, m_optixAnyHitProgram);
}
