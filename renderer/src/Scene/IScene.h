/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/
#ifndef ISCENENET_H
#define ISCENENET_H

#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <vector>
#include "Renderer/Light.h"
#include "Math/AAB.h"
#include "Math/Vector3.h"

class IScene
{
public:
    IScene();
    virtual ~IScene();
    virtual optix::Group getSceneRootGroup(optix::Context & context) = 0;
    virtual const std::vector<Light> & getSceneLights() const = 0;
    virtual const char* getSceneName() const = 0;
    virtual AAB getSceneAABB() const = 0;
    virtual optix::float3 scene_min() const { return optix::make_float3(0); }
    virtual optix::float3 scene_max() const { return optix::make_float3(0); }
    virtual float getSceneInitialPPMRadiusEstimate() const;
    virtual unsigned int getNumTriangles() const = 0;
};

#endif
