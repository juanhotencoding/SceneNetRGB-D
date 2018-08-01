/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include "SpecularDiffuse.h"
#include "Renderer/RayType.h"

bool SpecularDiffuse::m_optixMaterialIsCreated = false;
optix::Material SpecularDiffuse::m_optixMaterial;

SpecularDiffuse::SpecularDiffuse(const Vector3 & Kd, const Vector3 & Ks, const float alpha, int class_id, int instance_id)
  : Material(class_id,instance_id)
{
  this->Kd = Kd;
  this->Ks = Ks;
  this->alpha = alpha;
}

optix::Material SpecularDiffuse::getOptixMaterial(optix::Context & context)
{
    if(!m_optixMaterialIsCreated)
    {
        m_optixMaterial = context->createMaterial();
        optix::Program radianceProgram = context->createProgramFromPTXFile(ptxpath() + "SpecularDiffuse.cu.ptx", "closestHitRadiance");
        optix::Program photonProgram = context->createProgramFromPTXFile(ptxpath() + "SpecularDiffuse.cu.ptx", "closestHitPhoton");
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE_IN_PARTICIPATING_MEDIUM, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON, photonProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON_IN_PARTICIPATING_MEDIUM, photonProgram);
        m_optixMaterial->validate();

        this->registerMaterialWithShadowProgram(context, m_optixMaterial);
        this->registerMaterialWithGroundTruthGenerators(context, m_optixMaterial);
        m_optixMaterialIsCreated = true;
    }
    return m_optixMaterial;
}

/*
// Register any material-dependent values to be available in the optix program.
*/

void SpecularDiffuse::registerGeometryInstanceValues(optix::GeometryInstance & instance )
{
    instance["Kd"]->setFloat(this->Kd);
    instance["Ks"]->setFloat(this->Ks);
    instance["alpha"]->setFloat(this->alpha);
}
