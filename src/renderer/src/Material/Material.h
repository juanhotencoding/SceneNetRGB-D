/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#pragma once
#include <optixu/optixpp_namespace.h>
#include "Util/ptxpath.h"
#include <string>

class Material
{
public:
    Material(int class_id, int instance_id)
      : class_id_(class_id)
      , instance_id_(instance_id)
    {}
    virtual ~Material();
    virtual optix::Material getOptixMaterial(optix::Context & context) = 0;
    void registerBaseGeometryInstanceValues( optix::GeometryInstance & instance );
    virtual void registerGeometryInstanceValues( optix::GeometryInstance & instance ) = 0;
    virtual std::string materialType() = 0;
protected:
    static void registerMaterialWithShadowProgram(optix::Context & context, optix::Material & material);
    static void registerMaterialWithGroundTruthGenerators(optix::Context & context, optix::Material & material);
    int class_id_;
    int instance_id_;

private:
    static bool m_hasLoadedOptixAnyHitProgram;
    static optix::Program m_optixAnyHitProgram;

    static bool m_hasLoadedGroundTruthProgram;
    static optix::Program m_optixGroundTruthHitProgram;
};
