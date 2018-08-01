/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include "Material.h"
#include "Util/ptxpath.h"

class ParticipatingMedium : public Material
{
public:
    ParticipatingMedium(float sigma_s, float sigma_a, int class_id, int instance_id);
    virtual optix::Material getOptixMaterial(optix::Context & context);
    virtual void registerGeometryInstanceValues(optix::GeometryInstance & instance);
    virtual std::string materialType() { return "ParticipatingMedium"; };
private:
    //float indexOfRefraction;
    static bool m_optixMaterialIsCreated;
    static optix::Material m_optixMaterial;
    float m_sigma_s;
    float m_sigma_a;
};
