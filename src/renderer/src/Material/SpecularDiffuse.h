/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include "Material.h"
#include "Math/Vector3.h"
#include "Util/ptxpath.h"

class SpecularDiffuse : public Material
{
private:
    Vector3 Kd;
    Vector3 Ks;
    float alpha;
    static bool m_optixMaterialIsCreated;
    static optix::Material m_optixMaterial;
public:
    SpecularDiffuse(const Vector3 & Kd,const Vector3 & Ks,const float alpha, int class_id, int instance_id);
    virtual optix::Material getOptixMaterial(optix::Context & context);
    virtual void registerGeometryInstanceValues(optix::GeometryInstance & instance);
    virtual std::string materialType() { return "Diffuse"; };
};
