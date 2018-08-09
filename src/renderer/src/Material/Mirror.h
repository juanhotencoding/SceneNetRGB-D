/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include "Material.h"
#include "Math/Vector3.h"
#include "Util/ptxpath.h"

class Mirror : public Material
{
public:
    Mirror(const Vector3 & Kr, int class_id, int instance_id);
    virtual optix::Material getOptixMaterial(optix::Context & context);
    virtual void registerGeometryInstanceValues(optix::GeometryInstance & instance);
    virtual std::string materialType() { return "Mirror"; };
private:
    Vector3 Kr;
    static bool m_optixMaterialIsCreated;
    static optix::Material m_optixMaterial;
};
