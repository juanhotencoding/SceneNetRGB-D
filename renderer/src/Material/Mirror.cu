/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "rendererConfig.h"
#include "Renderer/RayType.h"
#include "Renderer/RadiancePRD.h"
#include "Renderer/PhotonMapping/PhotonPRD.h"
#include "Renderer/PhotonMapping/Photon.h"

using namespace optix;

rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(PhotonPRD, photonPrd, rtPayload, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, tHit, rtIntersectionDistance, );

rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, ); 
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, ); 

rtDeclareVariable(rtObject, sceneRootObject, , );
rtDeclareVariable(float3, Kr, , );


RT_PROGRAM void closestHitRadiance()
{
    float3 worldShadingNormal = normalize( rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal ) );
    float3 hitPoint = ray.origin + tHit*ray.direction;
    radiancePrd.depth++;
    if(radiancePrd.depth <= MAX_RADIANCE_TRACE_DEPTH)
    {
        radiancePrd.attenuation *= Kr;
        float3 newRayDirection = reflect(ray.direction, worldShadingNormal);
        Ray newRay ( hitPoint, newRayDirection, RayType::RADIANCE, 0.01, RT_DEFAULT_MAX );
        rtTrace( sceneRootObject, newRay, radiancePrd );
    }
    radiancePrd.lastTHit = tHit;
}

RT_PROGRAM void closestHitPhoton()
{
    float3 worldShadingNormal = normalize( rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal ) );
    float3 hitPoint = ray.origin + tHit*ray.direction;
    photonPrd.depth++;
    if (photonPrd.depth <= MAX_PHOTON_TRACE_DEPTH)
    {
        photonPrd.power *= Kr;
        float3 newPhotonDirection = reflect(ray.direction, worldShadingNormal);
        Ray newPhoton (hitPoint, newPhotonDirection, RayType::PHOTON, 0.01 );
        rtTrace(sceneRootObject, newPhoton, photonPrd);
    }
}
